#
# USB composite gadget drivers
#

g_composite-objs	:= composite.o charger.o \
				ether.o \
				serial4.o \
				file_storage.o \
				inode.o \
				../usbstring.o ../config.o ../epautoconf.o

ifeq ($(CONFIG_USB_ETH_RNDIS),y)
	g_composite-objs	+= ../rndis.o
endif

obj-$(CONFIG_USB_COMPOSITE)	+= g_composite.o
