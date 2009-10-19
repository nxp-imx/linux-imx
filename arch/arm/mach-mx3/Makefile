#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= system.o iomux.o cpu.o mm.o clock.o dptc.o devices.o serial.o dma.o mxc_pm.o dvfs_v2.o
obj-$(CONFIG_MACH_MX31ADS)	+= mx31ads.o mx31ads_gpio.o
obj-$(CONFIG_MACH_MX31LILLY)	+= mx31lilly.o mx31lilly-db.o
obj-$(CONFIG_MACH_MX31LITE)	+= mx31lite.o
obj-$(CONFIG_MACH_PCM037)	+= pcm037.o
obj-$(CONFIG_MACH_PCM037_EET)	+= pcm037_eet.o
obj-$(CONFIG_MACH_MX31_3DS)	+= mx3_3stack.o mx3_3stack_gpio.o mx3_3stack_pmic_mc13783.o
obj-$(CONFIG_MACH_MX31MOBOARD)	+= mx31moboard.o mx31moboard-devboard.o \
				   mx31moboard-marxbot.o
obj-$(CONFIG_MACH_QONG)		+= qong.o
obj-$(CONFIG_MACH_PCM043)	+= pcm043.o
obj-$(CONFIG_MACH_ARMADILLO5X0) += armadillo5x0.o

# power management
obj-$(CONFIG_PM) 		+= pm.o

obj-$(CONFIG_USB_EHCI_ARC_H1)	+= usb_h1.o
obj-$(CONFIG_USB_EHCI_ARC_H2)	+= usb_h2.o

ifneq ($(strip $(CONFIG_USB_GADGET_ARC) $(CONFIG_USB_EHCI_ARC_OTG)),)
	obj-y	+= usb_dr.o
endif
