#
# Makefile for the linux kernel.
#

# Object file lists.


obj-y   := system.o iomux.o cpu.o mm.o clock.o devices.o serial.o dma.o lpmodes.o pm.o

obj-$(CONFIG_CPU_V7)        += wfi.o suspend.o

obj-$(CONFIG_MACH_MX51_3DS) += mx51_3stack.o mx51_3stack_gpio.o

obj-$(CONFIG_USB_EHCI_ARC_H1)   += usb_h1.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
    obj-y   += usb_dr.o
endif

