obj-y		:= system.o iomux.o cpu.o mm.o clock.o bus_freq.o devices.o serial.o

obj-$(CONFIG_MXC_SDMA_API) 	+= dma.o
obj-$(CONFIG_SPI_MXC)		+= mx25_3stack_cpld.o
obj-$(CONFIG_MACH_MX25_3DS)	+= mx25_3stack.o mx25_3stack_gpio.o mx25_3stack_pmic_mc34704.o

obj-$(CONFIG_USB_EHCI_ARC_H2)	+= usb_h2.o

obj-$(CONFIG_PM)		+= pm.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y	+= usb_dr.o
endif
