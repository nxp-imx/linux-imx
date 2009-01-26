#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y := mm.o dma.o gpio_mux.o clock.o devices.o serial.o system.o cpu.o dptc.o
obj-$(CONFIG_MACH_MX27ADS)	+= mx27ads.o mx27ads_gpio.o

# power management
obj-$(CONFIG_PM) 		+= pm.o mxc_pm.o

obj-$(CONFIG_USB_EHCI_ARC_H1)	+= usb_h1.o
obj-$(CONFIG_USB_EHCI_ARC_H2)	+= usb_h2.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y	+= usb_dr.o
endif
