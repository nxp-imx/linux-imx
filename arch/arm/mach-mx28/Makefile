#
# Makefile for the linux kernel.
#
obj-y += pinctrl.o clock.o device.o serial.o power.o bus_freq.o pm.o sleep.o

# Board select
obj-$(CONFIG_MACH_MX28EVK) += mx28evk.o mx28evk_pins.o
obj-$(CONFIG_GENERIC_GPIO) += gpio.o
obj-$(CONFIG_MXS_RAM_FREQ_SCALING) +=emi.o emi_settings.o

# USB support
ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y   += usb_dr.o
endif
obj-$(CONFIG_USB_EHCI_ARC_H1)   += usb_h1.o
