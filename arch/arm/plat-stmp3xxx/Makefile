#
# Makefile for the linux kernel.
#
# Object file lists.
obj-y += core.o timer.o irq.o dma.o clock.o pinmux.o devices.o \
	 lradc.o spi.o mmc.o \
	  power-test.o

obj-$(CONFIG_ARCH_STMP378X) += dcp-bootstream.o usb_common.o utmixc.o

# Power Management
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_STMP3XXX_UNIQUE_ID)	+= unique-id.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
    obj-y += usb_dr.o
endif

# charging/current limitation testing
obj-m += power-test.o


