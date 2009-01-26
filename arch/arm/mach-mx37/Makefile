#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y   := system.o iomux.o cpu.o mm.o clock.o devices.o serial.o dma.o lpmodes.o dptc.o


obj-$(CONFIG_MACH_MX37_3DS) += mx37_3stack.o mx37_3stack_gpio.o 
obj-$(CONFIG_SPI_MXC) += mx37_3stack_cpld.o
obj-$(CONFIG_REGULATOR_WM8350) += mx37_3stack_pmic_wm8350.o

# power management
obj-$(CONFIG_PM)        += pm.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
    obj-y   += usb_dr.o
endif
