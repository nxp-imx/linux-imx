#
# Makefile for the linux kernel.
#

# Common support
obj-y := cpu.o cpu_common.o gpio.o clock.o wdog.o snoop.o io.o time.o devices.o usb_common.o

obj-$(CONFIG_IRAM_ALLOC) += iram.o

obj-$(CONFIG_ARCH_MX1) += iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MX2) += iomux-mx1-mx2.o dma-mx1-mx2.o
obj-$(CONFIG_ARCH_MXC_IOMUX_V3) += iomux-v3.o
obj-$(CONFIG_MXC_PWM)  += pwm.o

ifneq ($(CONFIG_ARCH_MX27),y)
obj-y += sdma/
endif

ifeq ($(CONFIG_MXC_TZIC),y)
obj-y += tzic.o
else
obj-y += irq.o
endif

obj-$(CONFIG_ARCH_MX27) += dma_mx2.o
obj-$(CONFIG_ARCH_MX3) += dptc.o entry-pm.o
obj-$(CONFIG_ARCH_MX37) += dptc.o dvfs_core.o
obj-$(CONFIG_ARCH_MX5) += dvfs_core.o

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
obj-$(CONFIG_UTMI_MXC) += utmixc.o
obj-$(CONFIG_USB) += serialxc.o

