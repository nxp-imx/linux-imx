#
# Makefile for the linux kernel.
#

# Common support
obj-y := cpu.o cpu_common.o gpio.o clock.o wdog.o snoop.o io.o time.o

obj-$(CONFIG_ARCH_MX1) += iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MX2) += iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MXC_IOMUX_V3) += iomux-v3.o
obj-$(CONFIG_MXC_PWM)  += pwm.o

ifneq ($(CONFIG_ARCH_MX27),y)
obj-y += spba.o sdma/
endif

ifeq ($(CONFIG_MXC_TZIC),y)
obj-y += tzic.o
else
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_MX27) += dma_mx2.o usb_common.o
obj-$(CONFIG_ARCH_MX3) += dptc.o usb_common.o entry-pm.o
obj-$(CONFIG_ARCH_MX35) += usb_common.o serialxc.o
obj-$(CONFIG_ARCH_MX37) += usb_common.o utmixc.o dptc.o dvfs_core.o
obj-$(CONFIG_ARCH_MX51) += usb_common.o utmixc.o dvfs_core.o

# LEDs support
obj-$(CONFIG_LEDS) += leds.o

# CPU FREQ support
obj-$(CONFIG_CPU_FREQ_IMX)    += cpufreq.o

# DVFS-PER support
obj-$(CONFIG_MXC_DVFS_PER) 	+= dvfs_per.o

# USB support
obj-$(CONFIG_ISP1504_MXC) += isp1504xc.o
obj-$(CONFIG_ISP1301_MXC) += isp1301xc.o
obj-$(CONFIG_MC13783_MXC) += mc13783_xc.o

# obj-$(CONFIG_USB_EHCI_FSL_UTMI) += utmixc.o
ifneq ($(strip $(CONFIG_USB_EHCI_FSL_UTMI) $(CONFIG_USB_GADGET_FSL_UTMI)),)
obj-y += utmixc.o
endif

ifneq ($(CONFIG_USB_EHCI_ARC_H1)$(CONFIG_USB_EHCI_ARC_H2),)
ifneq ($(CONFIG_ARCH_MX51),y)
obj-y += serialxc.o
else
obj-y += isp1504xc.o
endif
endif

ifneq ($(CONFIG_ARCH_MX25)$(CONFIG_USB),)
obj-y += usb_common.o
endif

ifeq ($(CONFIG_ARCH_MX25),y)
ifneq ($(CONFIG_USB_EHCI_ARC_H2),)
obj-y += serialxc.o
endif
endif
