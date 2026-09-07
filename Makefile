.PATH:	${SRCTOP}/sys/dev/hpilo

KMOD	= hpilo
SRCS	= hpilo.c
SRCS	+= bus_if.h device_if.h pci_if.h

.include <bsd.kmod.mk>