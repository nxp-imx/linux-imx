#
# Makefile for the linux kernel.
#

# Common support
obj-y := clock.o gpio.o time.o devices.o cpu.o system.o irq-common.o usb_common.o usb_wakeup.o

# MX51 uses the TZIC interrupt controller, older platforms use AVIC
obj-$(CONFIG_MXC_TZIC) += tzic.o
obj-$(CONFIG_MXC_AVIC) += avic.o

obj-$(CONFIG_IMX_HAVE_IOMUX_V1) += iomux-v1.o
obj-$(CONFIG_ARCH_MXC_IOMUX_V3) += iomux-v3.o
obj-$(CONFIG_IRAM_ALLOC) += iram_alloc.o
obj-$(CONFIG_MXC_PWM)  += pwm.o
obj-$(CONFIG_MXC_ULPI) += ulpi.o
obj-$(CONFIG_MXC_EPIT) += epit.o
obj-$(CONFIG_ARCH_MXC_AUDMUX_V1) += audmux-v1.o
obj-$(CONFIG_ARCH_MXC_AUDMUX_V2) += audmux-v2.o
obj-$(CONFIG_MXC_DEBUG_BOARD) += 3ds_debugboard.o
obj-$(CONFIG_CPU_FREQ_IMX)    += cpufreq.o
obj-$(CONFIG_IMX_HAVE_PLATFORM_AHCI)	+= ahci_sata.o
ifdef CONFIG_SND_IMX_SOC
obj-y += ssi-fiq.o
obj-y += ssi-fiq-ksym.o
endif

obj-y += devices/

obj-$(CONFIG_ARCH_MX5) += dvfs_core.o
obj-$(CONFIG_ARCH_MX6) += dvfs_core.o fuse.o

# DVFS-PER support
obj-$(CONFIG_MXC_DVFS_PER) 	+= dvfs_per.o

# USB support
obj-$(CONFIG_ISP1504_MXC) += isp1504xc.o
obj-$(CONFIG_USB)    += utmixc.o
obj-$(CONFIG_USB)         += serialxc.o
obj-$(CONFIG_USB)         += usb_hsic_xcvr.o
