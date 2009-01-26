#
# Makefile for the kernel Digital Audio MUX (DAM) device driver.
#

ifeq ($(CONFIG_ARCH_MX27),y)
	obj-$(CONFIG_MXC_DAM) += dam_v1.o
else
	obj-$(CONFIG_MXC_DAM) += dam.o
endif
