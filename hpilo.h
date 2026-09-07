// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Driver for the HP iLO management processor.
 *
 * Copyright (C) 2008 Hewlett-Packard Development Company, L.P.
 *	David Altobelli <david.altobelli@hp.com>
 */

#ifndef __HPILO_H
#define __HPILO_H

#define ILO_NAME "hpilo"

/* iLO ASIC PCI revision id */
#define PCI_REV_ID_NECHES	7

/* max number of open channel control blocks per device, hw limited to 32 */
#define MAX_CCB	       24
/* min number of open channel control blocks per device, hw limited to 32 */
#define MIN_CCB		8
/* max number of supported devices */
#define MAX_ILO_DEV	1
/* max number of files */
#define MAX_OPEN	(MAX_CCB * MAX_ILO_DEV)
/* total wait time in usec */
#define MAX_WAIT_TIME	10000
/* per spin wait time in usec */
#define WAIT_TIME	10
/* spin counter for open/close delay */
#define MAX_WAIT	(MAX_WAIT_TIME / WAIT_TIME)

/*
 * Per device, used to track global memory allocations.
 */
struct ilo_hwinfo {
	/* mmio registers on device */
	struct resource     *mmio_res;
	int                 mmio_rid;
	bus_space_tag_t     mmio_bt;
	bus_space_handle_t  mmio_bh;

	/* doorbell registers on device */
	struct resource     *db_res;
	int                 db_rid;
	bus_space_tag_t     db_bt;
	bus_space_handle_t  db_bh;

	/* shared memory on device used for channel control blocks */
	struct resource     *ram_res;
	int                 ram_rid;
	bus_space_tag_t     ram_bt;
	bus_space_handle_t  ram_bh;

	/* files corresponding to this device */
	struct ccb_data     *ccb_alloc[MAX_CCB];

	struct cdev         *cdev[MAX_CCB];

	device_t            ilo_dev;

	/* DMA resources for CCB buffers */
	bus_dma_tag_t       parent_dmat;
	bus_dma_tag_t       ccb_dmat;
	bus_dmamap_t        ccb_dmamap;
	void                *ccb_dma_va;
	bus_addr_t          ccb_dma_pa;

	/* Interrupt */
	struct resource     *irq_res;
	int                 irq_rid;
	void                *intrhand;

	/* slot in ilo_hwdev[], released on detach */
	int                 devnum;

	/* set when the device is being detached, blocks new opens */
	int                 detaching;

	/*
	 * open_lock      serializes ccb_cnt during open and close
	 * [ irq disabled ]
	 * -> alloc_lock  used when adding/removing/searching ccb_alloc,
	 *                which represents all ccbs open on the device
	 * --> fifo_lock  controls access to fifo queues shared with hw
	 *
	 * Locks must be taken in this order, but open_lock and alloc_lock
	 * are optional, they do not need to be held in order to take a
	 * lower level lock.
	 */
	struct sx           open_lock;
	struct mtx          alloc_lock;
	struct mtx          fifo_lock;
};

/* offset from mmio_vaddr for enabling doorbell interrupts */
#define DB_IRQ		0xB2
/* offset from mmio_vaddr for outbound communications */
#define DB_OUT		0xD4
/* DB_OUT reset bit */
#define DB_RESET	26

/*
 * Channel control block. Used to manage hardware queues.
 * The format must match hw's version.  The hw ccb is 128 bytes,
 * but the context area shouldn't be touched by the driver.
 */
#define ILOSW_CCB_SZ	64
#define ILOHW_CCB_SZ	128
struct ilo_ccb {
	union {
		char *send_fifobar;
		uint64_t send_fifobar_pa;
	} ccb_u1;
	union {
		char *send_desc;
		uint64_t send_desc_pa;
	} ccb_u2;
	uint64_t send_ctrl;

	union {
		char *recv_fifobar;
		uint64_t recv_fifobar_pa;
	} ccb_u3;
	union {
		char *recv_desc;
		uint64_t recv_desc_pa;
	} ccb_u4;
	uint64_t recv_ctrl;

	union {
		char *db_base;
		uint64_t padding5;
	} ccb_u5;

	uint64_t channel;

	/* unused context area (64 bytes) */
};

/* ccb queue parameters */
#define SENDQ		1
#define RECVQ	2
#define NR_QENTRY	4
#define L2_QENTRY_SZ	12

/* ccb ctrl bitfields */
#define CTRL_BITPOS_L2SZ             0
#define CTRL_BITPOS_FIFOINDEXMASK    4
#define CTRL_BITPOS_DESCLIMIT        18
#define CTRL_BITPOS_A                30
#define CTRL_BITPOS_G                31

/* ccb doorbell macros */
#define L2_DB_SIZE		14
#define ONE_DB_SIZE		(1 << L2_DB_SIZE)

/*
 * Per fd structure used to track the ccb allocated to that dev file.
 */
struct ccb_data {
	/* software version of ccb, using virtual addrs */
	struct ilo_ccb  driver_ccb;

	/* hardware version of ccb, using physical addrs */
	struct ilo_ccb  ilo_ccb;

	/* hardware ccb is written to this shared mapped device memory */
	bus_space_handle_t  mapped_ccb_bh;
	bus_space_tag_t     mapped_ccb_bt;

	/* dma'able memory used for send/recv queues */
	void       *dma_va;
	bus_addr_t  dma_pa;
	bus_size_t  dma_size;
	bus_dma_tag_t dma_tag;
	bus_dmamap_t  dma_map;

	/* pointer to hardware device info */
	struct ilo_hwinfo *ilo_hw;

	/* doorbell aperture for this channel */
	bus_space_tag_t     db_bt;
	bus_space_handle_t  db_bh;

	/* queue for this ccb to wait for recv data */
	struct selinfo      ccb_selinfo;

	/* usage count, to allow for shared ccb's */
	int	    ccb_cnt;

	/* open wanted exclusive access to this ccb */
	int	    ccb_excl;

	/* minor number for this ccb */
	int	    ccb_minor;
};

/*
 * FIFO queue structure, shared with hw.
 */
#define ILO_START_ALIGN	4096
#define ILO_CACHE_SZ	 128
struct fifo {
	uint64_t nrents;	/* user requested number of fifo entries */
	uint64_t imask;  /* mask to extract valid fifo index */
	uint64_t merge;	/*  O/C bits to merge in during enqueue operation */
	uint64_t reset;	/* set to non-zero when the target device resets */
	uint8_t  pad_0[ILO_CACHE_SZ - (sizeof(uint64_t) * 4)];

	uint64_t head;
	uint8_t  pad_1[ILO_CACHE_SZ - (sizeof(uint64_t))];

	uint64_t tail;
	uint8_t  pad_2[ILO_CACHE_SZ - (sizeof(uint64_t))];

	uint64_t fifobar[1];
};

/* convert between struct fifo, and the fifobar, which is saved in the ccb */
#define FIFOHANDLESIZE (sizeof(struct fifo) - sizeof(uint64_t))
#define FIFOBARTOHANDLE(_fifo) \
	((struct fifo *)(((char *)(_fifo)) - FIFOHANDLESIZE))

/* the number of qwords to consume from the entry descriptor */
#define ENTRY_BITPOS_QWORDS      0
/* descriptor index number (within a specified queue) */
#define ENTRY_BITPOS_DESCRIPTOR  10
/* state bit, fifo entry consumed by consumer */
#define ENTRY_BITPOS_C           22
/* state bit, fifo entry is occupied */
#define ENTRY_BITPOS_O           23

#define ENTRY_BITS_QWORDS        10
#define ENTRY_BITS_DESCRIPTOR    12
#define ENTRY_BITS_C             1
#define ENTRY_BITS_O             1
#define ENTRY_BITS_TOTAL	\
	(ENTRY_BITS_C + ENTRY_BITS_O + \
	 ENTRY_BITS_QWORDS + ENTRY_BITS_DESCRIPTOR)

/* extract various entry fields */
#define ENTRY_MASK ((1 << ENTRY_BITS_TOTAL) - 1)
#define ENTRY_MASK_C (((1 << ENTRY_BITS_C) - 1) << ENTRY_BITPOS_C)
#define ENTRY_MASK_O (((1 << ENTRY_BITS_O) - 1) << ENTRY_BITPOS_O)
#define ENTRY_MASK_QWORDS \
	(((1 << ENTRY_BITS_QWORDS) - 1) << ENTRY_BITPOS_QWORDS)
#define ENTRY_MASK_DESCRIPTOR \
	(((1 << ENTRY_BITS_DESCRIPTOR) - 1) << ENTRY_BITPOS_DESCRIPTOR)

#define ENTRY_MASK_NOSTATE (ENTRY_MASK >> (ENTRY_BITS_C + ENTRY_BITS_O))

#endif /* __HPILO_H */
