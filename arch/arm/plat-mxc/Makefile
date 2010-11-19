#
# Makefile for the linux kernel.
#

# Common support
obj-y :=cpu.o cpu_common.o system.o gpio.o clock.o snoop.o io.o time.o devices.o usb_common.o usb_wakeup.o

obj-$(CONFIG_IRAM_ALLOC) += iram.o

# MX51 uses the TZIC interrupt controller, older platforms use AVIC (irq.o)
ifeq ($(CONFIG_MXC_TZIC),y)
obj-$(CONFIG_MXC_TZIC) += tzic.o
else
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_MX1) += dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MX2) += dma-mx1-mx2.o
obj-$(CONFIG_IMX_HAVE_IOMUX_V1) += iomux-v1.o
obj-$(CONFIG_ARCH_MXC_IOMUX_V3) += iomux-v3.o
obj-$(CONFIG_MXC_PWM)  += pwm.o
obj-$(CONFIG_USB_EHCI_MXC) += ehci.o
obj-$(CONFIG_MXC_ULPI) += ulpi.o
obj-$(CONFIG_ARCH_MXC_AUDMUX_V1) += audmux-v1.o
obj-$(CONFIG_ARCH_MXC_AUDMUX_V2) += audmux-v2.o
obj-$(CONFIG_MXC_DEBUG_BOARD) += 3ds_debugboard.o
ifdef CONFIG_SND_IMX_SOC
obj-y += ssi-fiq.o
obj-y += ssi-fiq-ksym.o
endif

ifneq ($(CONFIG_ARCH_MX27),y)
obj-y += sdma/
endif

obj-$(CONFIG_ARCH_MX37) += dptc.o dvfs_core.o
obj-$(CONFIG_ARCH_MX5) += dvfs_core.o
obj-$(CONFIG_ARCH_MX5) += ahci_sata.o

# CPU FREQ support
obj-$(CONFIG_CPU_FREQ_IMX)    += cpufreq.o

# DVFS-PER support
obj-$(CONFIG_MXC_DVFS_PER) 	+= dvfs_per.o

# USB support
obj-$(CONFIG_ISP1504_MXC) += isp1504xc.o
obj-$(CONFIG_ISP1301_MXC) += isp1301xc.o
obj-$(CONFIG_MC13783_MXC) += mc13783_xc.o
obj-$(CONFIG_UTMI_MXC) += utmixc.o
obj-$(CONFIG_USB) += serialxc.o

# ZQ calibration
obj-$(CONFIG_MXC_ZQ_CALIBRATION) += zq_calib.o

obj-y += devices/
