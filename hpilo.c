// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Driver for the HP iLO management processor.
 *
 * Copyright (C) 2008 Hewlett-Packard Development Company, L.P.
 *	David Altobelli <david.altobelli@hp.com>
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/poll.h>
#include <sys/priority.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/uio.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "hpilo.h"

static int max_ccb = 16;
SYSCTL_INT(_hw_hpilo, OID_AUTO, max_ccb, CTLFLAG_RDTUN, &max_ccb,
    0, "Maximum number of HP iLO channels to attach");
TUNABLE_INT("hw.hpilo.max_ccb", &max_ccb);

static char ilo_hwdev[MAX_ILO_DEV];

static int ilo_close(struct cdev *dev, int flags, int fmt, struct thread *td);
static int ilo_open(struct cdev *dev, int flags, int fmt, struct thread *td);
static int ilo_poll(struct cdev *dev, int events, struct thread *td);
static int ilo_read(struct cdev *dev, struct uio *uio, int ioflag);
static int ilo_write(struct cdev *dev, struct uio *uio, int ioflag);

static struct cdevsw ilo_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	"hpilo",
	.d_open =	ilo_open,
	.d_close =	ilo_close,
	.d_read =	ilo_read,
	.d_write =	ilo_write,
	.d_poll =	ilo_poll,
};

struct ilo_device_id {
	uint16_t	vendor;
	uint16_t	device;
	uint16_t	subvendor;
	uint16_t	subdevice;
};

/* devices supported by this driver */
static const struct ilo_device_id ilo_devices[] = {
	{ 0x0e11, 0xB204, 0, 0 },
	{ 0x103c, 0x3307, 0, 0 },
};

/* subsystems to skip: auxiliary iLO and 3PAR CL */
static const struct ilo_device_id ilo_blacklist[] = {
	{ 0x103c, 0x3307, 0x103c, 0x1979 },
	{ 0x103c, 0x3307, 0x1590, 0x0289 },
};

static int
is_blacklisted(uint16_t vendor, uint16_t device,
    uint16_t subvendor, uint16_t subdevice)
{
	int i;

	for (i = 0; i < nitems(ilo_blacklist); i++) {
		if (ilo_blacklist[i].vendor == vendor &&
		    ilo_blacklist[i].device == device &&
		    ilo_blacklist[i].subvendor == subvendor &&
		    ilo_blacklist[i].subdevice == subdevice)
			return (1);
	}
	return (0);
}

static int ilo_probe(device_t dev);
static int ilo_attach(device_t dev);
static int ilo_detach(device_t dev);

static device_method_t ilo_methods[] = {
	DEVMETHOD(device_probe,		ilo_probe),
	DEVMETHOD(device_attach,	ilo_attach),
	DEVMETHOD(device_detach,	ilo_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD_END
};

static driver_t ilo_driver = {
	"hpilo",
	ilo_methods,
	sizeof(struct ilo_hwinfo)
};

DRIVER_MODULE(hpilo, pci, ilo_driver, 0, 0);

static inline int
get_entry_id(int entry)
{
	return (entry & ENTRY_MASK_DESCRIPTOR) >> ENTRY_BITPOS_DESCRIPTOR;
}

static inline int
get_entry_len(int entry)
{
	return ((entry & ENTRY_MASK_QWORDS) >> ENTRY_BITPOS_QWORDS) << 3;
}

static inline int
mk_entry(int id, int len)
{
	int qlen = len & 7 ? (len >> 3) + 1 : len >> 3;
	return (id << ENTRY_BITPOS_DESCRIPTOR | qlen << ENTRY_BITPOS_QWORDS);
}

static inline int
desc_mem_sz(int nr_entry)
{
	return (nr_entry << L2_QENTRY_SZ);
}

static int
fifo_enqueue(struct ilo_hwinfo *hw, char *fifobar, int entry)
{
	struct fifo *fifo_q = FIFOBARTOHANDLE(fifobar);
	int ret = 0;

	mtx_lock(&hw->fifo_lock);
	if (!(fifo_q->fifobar[(fifo_q->tail + 1) & fifo_q->imask]
	      & ENTRY_MASK_O)) {
		fifo_q->fifobar[fifo_q->tail & fifo_q->imask] |=
				(entry & ENTRY_MASK_NOSTATE) | fifo_q->merge;
		fifo_q->tail += 1;
		ret = 1;
	}
	mtx_unlock(&hw->fifo_lock);

	return (ret);
}

static int
fifo_dequeue(struct ilo_hwinfo *hw, char *fifobar, int *entry)
{
	struct fifo *fifo_q = FIFOBARTOHANDLE(fifobar);
	int ret = 0;
	uint64_t c;

	mtx_lock(&hw->fifo_lock);
	c = fifo_q->fifobar[fifo_q->head & fifo_q->imask];
	if (c & ENTRY_MASK_C) {
		if (entry)
			*entry = c & ENTRY_MASK_NOSTATE;

		fifo_q->fifobar[fifo_q->head & fifo_q->imask] =
							(c | ENTRY_MASK) + 1;
		fifo_q->head += 1;
		ret = 1;
	}
	mtx_unlock(&hw->fifo_lock);

	return (ret);
}

static int
fifo_check_recv(struct ilo_hwinfo *hw, char *fifobar)
{
	struct fifo *fifo_q = FIFOBARTOHANDLE(fifobar);
	int ret = 0;
	uint64_t c;

	mtx_lock(&hw->fifo_lock);
	c = fifo_q->fifobar[fifo_q->head & fifo_q->imask];
	if (c & ENTRY_MASK_C)
		ret = 1;
	mtx_unlock(&hw->fifo_lock);

	return (ret);
}

static int
ilo_pkt_enqueue(struct ilo_hwinfo *hw, struct ilo_ccb *ccb,
    int dir, int id, int len)
{
	char *fifobar;
	int entry;

	if (dir == SENDQ)
		fifobar = ccb->ccb_u1.send_fifobar;
	else
		fifobar = ccb->ccb_u3.recv_fifobar;

	entry = mk_entry(id, len);
	return (fifo_enqueue(hw, fifobar, entry));
}

static int
ilo_pkt_dequeue(struct ilo_hwinfo *hw, struct ilo_ccb *ccb,
    int dir, int *id, int *len, void **pkt)
{
	char *fifobar, *desc;
	int entry = 0, pkt_id = 0;
	int ret;

	if (dir == SENDQ) {
		fifobar = ccb->ccb_u1.send_fifobar;
		desc = ccb->ccb_u2.send_desc;
	} else {
		fifobar = ccb->ccb_u3.recv_fifobar;
		desc = ccb->ccb_u4.recv_desc;
	}

	ret = fifo_dequeue(hw, fifobar, &entry);
	if (ret) {
		pkt_id = get_entry_id(entry);
		if (id)
			*id = pkt_id;
		if (len)
			*len = get_entry_len(entry);
		if (pkt)
			*pkt = (void *)(desc + desc_mem_sz(pkt_id));
	}

	return (ret);
}

static int
ilo_pkt_recv(struct ilo_hwinfo *hw, struct ilo_ccb *ccb)
{
	char *fifobar = ccb->ccb_u3.recv_fifobar;

	return (fifo_check_recv(hw, fifobar));
}

#define ilo_mmio_read1(hw, off)		bus_space_read_1((hw)->mmio_bt, (hw)->mmio_bh, (off))
#define ilo_mmio_write1(hw, off, v)	bus_space_write_1((hw)->mmio_bt, (hw)->mmio_bh, (off), (v))
#define ilo_mmio_read4(hw, off)		bus_space_read_4((hw)->mmio_bt, (hw)->mmio_bh, (off))
#define ilo_mmio_write4(hw, off, v)	bus_space_write_4((hw)->mmio_bt, (hw)->mmio_bh, (off), (v))

#define ilo_db_read1(ccb, off)		bus_space_read_1((ccb)->db_bt, (ccb)->db_bh, (off))
#define ilo_db_write1(ccb, off, v)	bus_space_write_1((ccb)->db_bt, (ccb)->db_bh, (off), (v))

static inline void
doorbell_set(struct ccb_data *data)
{
	ilo_db_write1(data, data->ccb_minor << L2_DB_SIZE, 1);
}

static inline void
doorbell_clr(struct ccb_data *data)
{
	ilo_db_write1(data, data->ccb_minor << L2_DB_SIZE, 2);
}

static inline int
ctrl_set(int l2sz, int idxmask, int desclim)
{
	int active = 0, go = 1;
	return (l2sz << CTRL_BITPOS_L2SZ |
	    idxmask << CTRL_BITPOS_FIFOINDEXMASK |
	    desclim << CTRL_BITPOS_DESCLIMIT |
	    active << CTRL_BITPOS_A |
	    go << CTRL_BITPOS_G);
}

static void
ctrl_setup(struct ilo_ccb *ccb, int nr_desc, int l2desc_sz)
{
	ccb->send_ctrl = ctrl_set(l2desc_sz, nr_desc - 1, nr_desc - 1);
	ccb->recv_ctrl = ctrl_set(l2desc_sz, nr_desc - 1, nr_desc - 1);
}

static inline int
fifo_sz(int nr_entry)
{
	return (nr_entry * sizeof(uint64_t)) + FIFOHANDLESIZE;
}

static void
fifo_setup(void *base_addr, int nr_entry)
{
	struct fifo *fifo_q = base_addr;
	int i;

	fifo_q->head = 0;
	fifo_q->tail = 0;
	fifo_q->reset = 0;
	fifo_q->nrents = nr_entry;
	fifo_q->imask = nr_entry - 1;
	fifo_q->merge = ENTRY_MASK_O;

	for (i = 0; i < nr_entry; i++)
		fifo_q->fifobar[i] = 0;
}

static int
ilo_dma_map_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *baddr = arg;
	if (error == 0 && nseg > 0)
		*baddr = segs[0].ds_addr;
	return (0);
}

static void
ilo_ccb_close(device_t dev, struct ccb_data *data)
{
	struct ilo_ccb *driver_ccb = &data->driver_ccb;
	struct ilo_hwinfo *hw = data->ilo_hw;
	bus_space_tag_t ram_bt = hw->ram_bt;
	bus_space_handle_t ram_bh = hw->ram_bh + data->ccb_minor * ILOHW_CCB_SZ;
	int retries;

	doorbell_clr(data);

	bus_space_write_4(ram_bt, ram_bh, 0x10,
	    bus_space_read_4(ram_bt, ram_bh, 0x10) & ~(1 << CTRL_BITPOS_G));
	bus_space_write_4(ram_bt, ram_bh, 0x28,
	    bus_space_read_4(ram_bt, ram_bh, 0x28) & ~(1 << CTRL_BITPOS_G));

	for (retries = MAX_WAIT; retries > 0; retries--) {
		doorbell_set(data);
		DELAY(WAIT_TIME);
		if (!(bus_space_read_4(ram_bt, ram_bh, 0x10) &
		    (1 << CTRL_BITPOS_A)) &&
		    !(bus_space_read_4(ram_bt, ram_bh, 0x28) &
		    (1 << CTRL_BITPOS_A)))
			break;
	}
	if (retries == 0)
		device_printf(dev, "Closing, but controller still active\n");

	bus_space_set_region_1(ram_bt, ram_bh, 0, 0, sizeof(struct ilo_ccb));

	bus_dmamap_unload(data->dma_tag, data->dma_map);
	bus_dmamem_free(data->dma_tag, data->dma_va, data->dma_map);
	bus_dma_tag_destroy(data->dma_tag);
}

static int
ilo_ccb_setup(struct ilo_hwinfo *hw, struct ccb_data *data, int slot)
{
	struct ilo_ccb *driver_ccb = &data->driver_ccb;
	struct ilo_ccb *ilo_ccb = &data->ilo_ccb;
	int err;

	data->dma_size = 2 * fifo_sz(NR_QENTRY) +
			 2 * desc_mem_sz(NR_QENTRY) +
			 ILO_START_ALIGN + ILO_CACHE_SZ;

	err = bus_dma_tag_create(bus_get_dma_tag(hw->ilo_dev),
	    ILO_START_ALIGN, 0,
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    data->dma_size, 1,
	    data->dma_size,
	    0,
	    NULL, NULL,
	    &data->dma_tag);
	if (err) {
		device_printf(hw->ilo_dev,
		    "Could not create dma tag: %d\n", err);
		return (err);
	}

	err = bus_dmamem_alloc(data->dma_tag, &data->dma_va,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &data->dma_map);
	if (err) {
		bus_dma_tag_destroy(data->dma_tag);
		return (err);
	}

	err = bus_dmamap_load(data->dma_tag, data->dma_map,
	    data->dma_va, data->dma_size,
	    ilo_dma_map_cb, &data->dma_pa, BUS_DMA_NOWAIT);
	if (err) {
		bus_dmamem_free(data->dma_tag, data->dma_va, data->dma_map);
		bus_dma_tag_destroy(data->dma_tag);
		return (err);
	}

	ctrl_setup(driver_ccb, NR_QENTRY, L2_QENTRY_SZ);
	ctrl_setup(ilo_ccb, NR_QENTRY, L2_QENTRY_SZ);

	char *dma_va = (char *)data->dma_va;
	bus_addr_t dma_pa = data->dma_pa;

	dma_va = (char *)roundup((uintptr_t)dma_va, ILO_START_ALIGN);
	dma_pa = roundup(dma_pa, ILO_START_ALIGN);

	fifo_setup(dma_va, NR_QENTRY);
	driver_ccb->ccb_u1.send_fifobar = dma_va + FIFOHANDLESIZE;
	ilo_ccb->ccb_u1.send_fifobar_pa = dma_pa + FIFOHANDLESIZE;
	dma_va += fifo_sz(NR_QENTRY);
	dma_pa += fifo_sz(NR_QENTRY);

	dma_va = (char *)roundup((uintptr_t)dma_va, ILO_CACHE_SZ);
	dma_pa = roundup(dma_pa, ILO_CACHE_SZ);

	fifo_setup(dma_va, NR_QENTRY);
	driver_ccb->ccb_u3.recv_fifobar = dma_va + FIFOHANDLESIZE;
	ilo_ccb->ccb_u3.recv_fifobar_pa = dma_pa + FIFOHANDLESIZE;
	dma_va += fifo_sz(NR_QENTRY);
	dma_pa += fifo_sz(NR_QENTRY);

	driver_ccb->ccb_u2.send_desc = dma_va;
	ilo_ccb->ccb_u2.send_desc_pa = dma_pa;
	dma_pa += desc_mem_sz(NR_QENTRY);
	dma_va += desc_mem_sz(NR_QENTRY);

	driver_ccb->ccb_u4.recv_desc = dma_va;
	ilo_ccb->ccb_u4.recv_desc_pa = dma_pa;

	driver_ccb->channel = slot;
	ilo_ccb->channel = slot;

	driver_ccb->ccb_u5.db_base = 0;
	ilo_ccb->ccb_u5.db_base = 0;

	return (0);
}

static void
ilo_ccb_open(struct ilo_hwinfo *hw, struct ccb_data *data, int slot)
{
	int pkt_id, pkt_sz;
	struct ilo_ccb *driver_ccb = &data->driver_ccb;
	bus_space_tag_t ram_bt = hw->ram_bt;
	bus_space_handle_t ram_bh = hw->ram_bh + slot * ILOHW_CCB_SZ;

	data->mapped_ccb_bt = ram_bt;
	data->mapped_ccb_bh = ram_bh;

	bus_space_write_region_1(ram_bt, ram_bh, 0,
	    (uint8_t *)&data->ilo_ccb, sizeof(struct ilo_ccb));

	pkt_sz = 0;
	for (pkt_id = 0; pkt_id < NR_QENTRY; pkt_id++) {
		ilo_pkt_enqueue(hw, driver_ccb, SENDQ, pkt_id, pkt_sz);
		doorbell_set(data);
	}

	pkt_sz = desc_mem_sz(1);
	for (pkt_id = 0; pkt_id < NR_QENTRY; pkt_id++)
		ilo_pkt_enqueue(hw, driver_ccb, RECVQ, pkt_id, pkt_sz);

	doorbell_clr(data);
}

static int
ilo_ccb_verify(struct ilo_hwinfo *hw, struct ccb_data *data)
{
	int pkt_id, i;
	struct ilo_ccb *driver_ccb = &data->driver_ccb;

	for (i = MAX_WAIT; i > 0; i--) {
		if (ilo_pkt_dequeue(hw, driver_ccb, SENDQ, &pkt_id, NULL, NULL))
			break;
		DELAY(WAIT_TIME);
	}

	if (i == 0) {
		device_printf(hw->ilo_dev, "Open could not dequeue a packet\n");
		return (EBUSY);
	}

	ilo_pkt_enqueue(hw, driver_ccb, SENDQ, pkt_id, 0);
	doorbell_set(data);
	return (0);
}

static inline int
is_channel_reset(struct ilo_ccb *ccb)
{
	return (FIFOBARTOHANDLE(ccb->ccb_u1.send_fifobar)->reset);
}

static inline void
set_channel_reset(struct ilo_ccb *ccb)
{
	FIFOBARTOHANDLE(ccb->ccb_u1.send_fifobar)->reset = 1;
}

static inline int
get_device_outbound(struct ilo_hwinfo *hw)
{
	return (ilo_mmio_read4(hw, DB_OUT));
}

static inline int
is_db_reset(int db_out)
{
	return (db_out & (1 << DB_RESET));
}

static inline void
clear_pending_db(struct ilo_hwinfo *hw, int clr)
{
	ilo_mmio_write4(hw, DB_OUT, clr);
}

static inline void
clear_device(struct ilo_hwinfo *hw)
{
	clear_pending_db(hw, -1);
}

static inline void
ilo_enable_interrupts(struct ilo_hwinfo *hw)
{
	ilo_mmio_write1(hw, DB_IRQ, ilo_mmio_read1(hw, DB_IRQ) | 1);
}

static inline void
ilo_disable_interrupts(struct ilo_hwinfo *hw)
{
	ilo_mmio_write1(hw, DB_IRQ, ilo_mmio_read1(hw, DB_IRQ) & ~1);
}

static void
ilo_set_reset(struct ilo_hwinfo *hw)
{
	int slot;

	for (slot = 0; slot < max_ccb; slot++) {
		if (!hw->ccb_alloc[slot])
			continue;
		set_channel_reset(&hw->ccb_alloc[slot]->driver_ccb);
	}
}

static int
ilo_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	int err, found, cnt, pkt_id, pkt_len, slot;
	struct ilo_hwinfo *hw = dev->si_drv1;
	struct ccb_data *data;
	struct ilo_ccb *driver_ccb;
	void *pkt;

	slot = minor(dev->si_rdev) % max_ccb;
	mtx_lock(&hw->alloc_lock);
	data = hw->ccb_alloc[slot];
	mtx_unlock(&hw->alloc_lock);
	if (data == NULL)
		return (ENODEV);
	driver_ccb = &data->driver_ccb;

	if (is_channel_reset(driver_ccb))
		return (ENODEV);

	cnt = 20;
	do {
		found = ilo_pkt_dequeue(hw, driver_ccb, RECVQ, &pkt_id,
					&pkt_len, &pkt);
		if (found)
			break;
		if (--cnt == 0)
			break;
		mtx_lock(&hw->alloc_lock);
		/*
		 * alloc_lock serializes against ilo_intr() so a wakeup
		 * between the dequeue check and the sleep is not lost.
		 * msleep() drops the lock while asleep and reacquires it
		 * before returning.
		 */
		msleep(&data->ccb_selinfo, &hw->alloc_lock, PRIBIO, "hpilo",
		    hz / 10);
		mtx_unlock(&hw->alloc_lock);
	} while (1);

	if (!found)
		return (EAGAIN);

	if (pkt_len < uio->uio_resid)
		uio->uio_resid = pkt_len;

	err = uiomove(pkt, uio->uio_resid, uio);

	ilo_pkt_enqueue(hw, driver_ccb, RECVQ, pkt_id, desc_mem_sz(1));

	return (err ? EFAULT : 0);
}

static int
ilo_write(struct cdev *dev, struct uio *uio, int ioflag)
{
	int err, pkt_id, pkt_len, slot;
	struct ilo_hwinfo *hw = dev->si_drv1;
	struct ccb_data *data;
	struct ilo_ccb *driver_ccb;
	void *pkt;

	slot = minor(dev->si_rdev) % max_ccb;
	mtx_lock(&hw->alloc_lock);
	data = hw->ccb_alloc[slot];
	mtx_unlock(&hw->alloc_lock);
	if (data == NULL)
		return (ENODEV);
	driver_ccb = &data->driver_ccb;

	if (is_channel_reset(driver_ccb))
		return (ENODEV);

	if (!ilo_pkt_dequeue(hw, driver_ccb, SENDQ, &pkt_id, &pkt_len, &pkt))
		return (EBUSY);

	if (pkt_len < uio->uio_resid)
		uio->uio_resid = pkt_len;

	err = uiomove(pkt, uio->uio_resid, uio);
	if (err)
		uio->uio_resid = 0;

	ilo_pkt_enqueue(hw, driver_ccb, SENDQ, pkt_id, uio->uio_resid);
	doorbell_set(data);

	return (err ? EFAULT : 0);
}

static int
ilo_poll(struct cdev *dev, int events, struct thread *td)
{
	int revents = 0, slot;
	struct ilo_hwinfo *hw = dev->si_drv1;
	struct ccb_data *data;
	struct ilo_ccb *driver_ccb;

	slot = minor(dev->si_rdev) % max_ccb;
	mtx_lock(&hw->alloc_lock);
	data = hw->ccb_alloc[slot];
	mtx_unlock(&hw->alloc_lock);
	if (data == NULL)
		return (0);
	driver_ccb = &data->driver_ccb;

	if (is_channel_reset(driver_ccb))
		return (POLLERR);
	if (events & (POLLIN | POLLRDNORM)) {
		if (ilo_pkt_recv(hw, driver_ccb))
			revents |= events & (POLLIN | POLLRDNORM);
		else
			selrecord(td, &data->ccb_selinfo);
	}
	return (revents);
}

static int
ilo_close(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	int slot;
	struct ccb_data *data;
	struct ilo_hwinfo *hw;

	slot = minor(dev->si_rdev) % max_ccb;
	hw = dev->si_drv1;

	sx_xlock(&hw->open_lock);

	data = hw->ccb_alloc[slot];
	if (data == NULL) {
		sx_xunlock(&hw->open_lock);
		return (0);
	}

	if (data->ccb_cnt == 1) {
		mtx_lock(&hw->alloc_lock);
		hw->ccb_alloc[slot] = NULL;
		mtx_unlock(&hw->alloc_lock);

		ilo_ccb_close(hw->ilo_dev, data);

		selinfo_destroy(&data->ccb_selinfo);
		free(data, M_DEVBUF);
	} else
		data->ccb_cnt--;

	sx_xunlock(&hw->open_lock);

	return (0);
}

static int
ilo_open(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	int slot, error;
	struct ccb_data *data;
	struct ilo_hwinfo *hw;

	slot = minor(dev->si_rdev) % max_ccb;
	hw = dev->si_drv1;

	data = malloc(sizeof(*data), M_DEVBUF, M_WAITOK | M_ZERO);
	if (data == NULL)
		return (ENOMEM);

	sx_xlock(&hw->open_lock);

	if (hw->detaching) {
		free(data, M_DEVBUF);
		error = ENXIO;
		goto out;
	} else if (hw->ccb_alloc[slot] == NULL) {
		error = ilo_ccb_setup(hw, data, slot);
		if (error) {
			free(data, M_DEVBUF);
			goto out;
		}

		data->ccb_cnt = 1;
		data->ccb_excl = flags & O_EXCL;
		data->ilo_hw = hw;
		data->ccb_minor = slot;
		data->db_bt = hw->db_bt;
		data->db_bh = hw->db_bh;
		selinfo_init(&data->ccb_selinfo, "hpilo");

		mtx_lock(&hw->alloc_lock);
		ilo_ccb_open(hw, data, slot);
		hw->ccb_alloc[slot] = data;
		mtx_unlock(&hw->alloc_lock);

		error = ilo_ccb_verify(hw, data);
		if (error) {
			mtx_lock(&hw->alloc_lock);
			hw->ccb_alloc[slot] = NULL;
			mtx_unlock(&hw->alloc_lock);

			ilo_ccb_close(hw->ilo_dev, data);
			selinfo_destroy(&data->ccb_selinfo);
			free(data, M_DEVBUF);
			goto out;
		}

	} else {
		free(data, M_DEVBUF);
		if ((flags & O_EXCL) || hw->ccb_alloc[slot]->ccb_excl) {
			error = EBUSY;
		} else {
			hw->ccb_alloc[slot]->ccb_cnt++;
			error = 0;
		}
	}
out:
	sx_xunlock(&hw->open_lock);

	return (error);
}

static int
ilo_filter(void *arg)
{
	struct ilo_hwinfo *hw = arg;
	uint32_t pending;

	pending = ilo_mmio_read4(hw, DB_OUT);
	if (pending == 0)
		return (FILTER_STRAY);
	return (FILTER_HANDLED);
}

static void
ilo_intr(void *arg)
{
	struct ilo_hwinfo *hw = arg;
	int pending, i;

	mtx_lock(&hw->alloc_lock);

	pending = get_device_outbound(hw);
	if (!pending) {
		mtx_unlock(&hw->alloc_lock);
		return;
	}

	if (is_db_reset(pending)) {
		pending = -1;
		ilo_set_reset(hw);
	}

	for (i = 0; i < max_ccb; i++) {
		if (!hw->ccb_alloc[i])
			continue;
		if (pending & (1 << i)) {
			wakeup(&hw->ccb_alloc[i]->ccb_selinfo);
			selwakeup(&hw->ccb_alloc[i]->ccb_selinfo);
		}
	}

	clear_pending_db(hw, pending);

	mtx_unlock(&hw->alloc_lock);
}

static void
ilo_unmap_device(device_t dev, struct ilo_hwinfo *hw)
{

	if (hw->db_res != NULL)
		bus_release_resource(dev, hw->db_res);
	if (hw->ram_res != NULL)
		bus_release_resource(dev, hw->ram_res);
	if (hw->mmio_res != NULL)
		bus_release_resource(dev, hw->mmio_res);
}

static int
ilo_map_device(device_t dev, struct ilo_hwinfo *hw)
{
	uint32_t pci_rev_id;

	hw->mmio_rid = PCIR_BAR(1);
	hw->mmio_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &hw->mmio_rid, RF_ACTIVE);
	if (hw->mmio_res == NULL) {
		device_printf(dev, "Error mapping mmio\n");
		return (ENOMEM);
	}
	hw->mmio_bt = rman_get_bustag(hw->mmio_res);
	hw->mmio_bh = rman_get_bushandle(hw->mmio_res);

	pci_rev_id = pci_read_config(dev, PCIR_REVID, 1);

	if (pci_rev_id >= PCI_REV_ID_NECHES) {
		hw->ram_rid = PCIR_BAR(5);
		hw->ram_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &hw->ram_rid, RF_ACTIVE);
		if (hw->ram_res == NULL) {
			device_printf(dev, "Error mapping shared mem\n");
			goto err_mmio;
		}
		hw->ram_bt = rman_get_bustag(hw->ram_res);
		hw->ram_bh = rman_get_bushandle(hw->ram_res);
		hw->ram_bh += rman_get_size(hw->ram_res) - 0x2000;
	} else {
		hw->ram_rid = PCIR_BAR(2);
		hw->ram_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &hw->ram_rid, RF_ACTIVE);
		if (hw->ram_res == NULL) {
			device_printf(dev, "Error mapping shared mem\n");
			goto err_mmio;
		}
		hw->ram_bt = rman_get_bustag(hw->ram_res);
		hw->ram_bh = rman_get_bushandle(hw->ram_res);
	}

	hw->db_rid = PCIR_BAR(3);
	hw->db_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &hw->db_rid, RF_ACTIVE);
	if (hw->db_res == NULL) {
		device_printf(dev, "Error mapping doorbell\n");
		goto err_ram;
	}
	hw->db_bt = rman_get_bustag(hw->db_res);
	hw->db_bh = rman_get_bushandle(hw->db_res);

	return (0);

err_ram:
	bus_release_resource(dev, hw->ram_res);
err_mmio:
	bus_release_resource(dev, hw->mmio_res);
	return (ENOMEM);
}

static int
ilo_probe(device_t dev)
{
	uint16_t vendor, device, subvendor, subdevice;
	int i;

	vendor = pci_get_vendor(dev);
	device = pci_get_device(dev);
	subvendor = pci_get_subvendor(dev);
	subdevice = pci_get_subdevice(dev);

	for (i = 0; i < nitems(ilo_devices); i++) {
		if (ilo_devices[i].vendor != vendor ||
		    ilo_devices[i].device != device)
			continue;
		if (is_blacklisted(vendor, device, subvendor, subdevice))
			return (ENXIO);
		device_set_desc(dev, "HP iLO");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
ilo_attach(device_t dev)
{
	struct ilo_hwinfo *hw;
	int error, minor, devnum, irq_rid;

	hw = device_get_softc(dev);
	bzero(hw, sizeof(*hw));
	hw->ilo_dev = dev;

	if (max_ccb < MIN_CCB)
		max_ccb = MIN_CCB;
	if (max_ccb > MAX_CCB)
		max_ccb = MAX_CCB;

	sx_init(&hw->open_lock, "hpilo_open");
	mtx_init(&hw->alloc_lock, "hpilo_alloc", NULL, MTX_DEF);
	mtx_init(&hw->fifo_lock, "hpilo_fifo", NULL, MTX_DEF);

	pci_enable_busmaster(dev);

	error = ilo_map_device(dev, hw);
	if (error)
		goto fail_locks;

	irq_rid = 0;
	hw->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	hw->irq_rid = irq_rid;
	if (hw->irq_res == NULL) {
		device_printf(dev, "Could not allocate IRQ\n");
		error = ENXIO;
		goto fail_map;
	}

	error = bus_setup_intr(dev, hw->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    ilo_filter, ilo_intr, hw, &hw->intrhand);
	if (error) {
		device_printf(dev, "Could not setup interrupt\n");
		error = ENXIO;
		goto fail_irq;
	}

	ilo_enable_interrupts(hw);

	for (devnum = 0; devnum < MAX_ILO_DEV; devnum++) {
		if (ilo_hwdev[devnum] == 0) {
			ilo_hwdev[devnum] = 1;
			break;
		}
	}
	if (devnum == MAX_ILO_DEV) {
		device_printf(dev, "Error finding free device\n");
		error = ENODEV;
		goto fail_intr;
	}
	hw->devnum = devnum;

	for (minor = 0; minor < max_ccb; minor++) {
		hw->cdev[minor] = make_dev(&ilo_cdevsw,
		    devnum * max_ccb + minor,
		    UID_ROOT, GID_OPERATOR, 0600,
		    "hpilo/d%dccb%d", devnum, minor);
		if (hw->cdev[minor] == NULL) {
			device_printf(dev, "Error creating device node\n");
			error = ENOMEM;
			goto fail_cdev;
		}
		hw->cdev[minor]->si_drv1 = hw;
	}

	return (0);

fail_cdev:
	for (minor = 0; minor < max_ccb; minor++) {
		if (hw->cdev[minor] != NULL)
			destroy_dev(hw->cdev[minor]);
	}
	ilo_hwdev[hw->devnum] = 0;
fail_intr:
	ilo_disable_interrupts(hw);
	bus_teardown_intr(dev, hw->irq_res, hw->intrhand);
fail_irq:
	bus_release_resource(dev, hw->irq_res);
fail_map:
	ilo_unmap_device(dev, hw);
fail_locks:
	sx_destroy(&hw->open_lock);
	mtx_destroy(&hw->alloc_lock);
	mtx_destroy(&hw->fifo_lock);
	return (error);
}

static int
ilo_detach(device_t dev)
{
	struct ilo_hwinfo *hw = device_get_softc(dev);
	int minor;

	/* Refuse to detach while any channel is still open. */
	sx_xlock(&hw->open_lock);
	for (minor = 0; minor < max_ccb; minor++) {
		if (hw->ccb_alloc[minor] != NULL) {
			sx_xunlock(&hw->open_lock);
			return (EBUSY);
		}
	}
	hw->detaching = 1;
	sx_xunlock(&hw->open_lock);

	for (minor = 0; minor < max_ccb; minor++) {
		if (hw->cdev[minor] != NULL)
			destroy_dev(hw->cdev[minor]);
	}

	ilo_disable_interrupts(hw);
	bus_teardown_intr(dev, hw->irq_res, hw->intrhand);
	bus_release_resource(dev, hw->irq_res);
	ilo_unmap_device(dev, hw);

	/*
	 * open_lock is intentionally not destroyed here: a blocked opener
	 * granted entry after detaching=1 relies on it staying valid, and
	 * the reference drivers (mfi, led) keep their locks across detach.
	 */
	mtx_destroy(&hw->alloc_lock);
	mtx_destroy(&hw->fifo_lock);

	ilo_hwdev[hw->devnum] = 0;

	return (0);
}
