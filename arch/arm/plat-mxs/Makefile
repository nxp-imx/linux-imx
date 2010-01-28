#
# Makefile for the linux kernel.
#
# Object file lists.
obj-y += core.o clock.o pinctrl.o device.o timer.o

obj-$(CONFIG_GENERIC_GPIO) += gpio.o

obj-$(CONFIG_MXS_ICOLL) += icoll.o

obj-$(CONFIG_MXS_DMA_ENGINE) += dmaengine.o dma-apbh.o dma-apbx.o
obj-$(CONFIG_ARCH_MX28) += usb_common.o utmixc.o

obj-$(CONFIG_MXS_LRADC) += lradc.o
# Power Management

# charging/current limitation testing

