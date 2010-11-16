#
# Makefile for the linux kernel.
#
# Object file lists.
obj-y += core.o clock.o pinctrl.o device.o timer-nomatch.o usb_common.o usb_wakeup.o

obj-$(CONFIG_MXS_TIMER_WITH_MACH) += timer-match.o
obj-$(CONFIG_IRAM_ALLOC) += iram.o
obj-$(CONFIG_GENERIC_GPIO) += gpio.o

obj-$(CONFIG_MXS_UNIQUE_ID)        += unique-id.o

obj-$(CONFIG_MXS_ICOLL) += icoll.o

obj-$(CONFIG_MXS_DMA_ENGINE) += dmaengine.o dma-apbh.o dma-apbx.o

obj-$(CONFIG_USB_SUPPORT) += utmixc.o

obj-$(CONFIG_MXS_LRADC) += lradc.o
# Power Management
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

# charging/current limitation testing

