#
# Makefile for the linux kernel.
#
obj-y += pinctrl.o clock.o device.o serial.o power.o pm.o sleep.o bus_freq.o

# Board select
obj-$(CONFIG_MACH_MX23EVK) += mx23evk.o mx23evk_pins.o
obj-$(CONFIG_GENERIC_GPIO) += gpio.o
obj-$(CONFIG_MXS_RAM_FREQ_SCALING) +=emi.o
obj-$(CONFIG_MXS_UNIQUE_ID_OTP)    += otp.o

# USB support
ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y   += usb_dr.o
endif
