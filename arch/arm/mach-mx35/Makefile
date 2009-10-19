#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y		:= system.o iomux.o cpu.o mm.o clock.o devices.o serial.o
obj-$(CONFIG_MXC_SDMA_API) 	+= dma.o
obj-$(CONFIG_MACH_MX35_3DS)	+= mx35_3stack.o mx35_3stack_gpio.o mx35_3stack_cpld.o dvfs.o mx35_3stack_pmic_mc13892.o mx35_3stack_pmic_mc9s08dz60.o
obj-$(CONFIG_MACH_MX35EVB)	+= mx35evb.o mx35evb_cpld.o mx35evb_gpio.o

obj-$(CONFIG_MXC_PSEUDO_IRQS)	+= mx35_3stack_irq.o
obj-$(CONFIG_PM) 		+= pm.o

obj-$(CONFIG_USB_EHCI_ARC_H2)	+= usb_h2.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y	+= usb_dr.o
endif
