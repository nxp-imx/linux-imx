#
# Makefile for the MPEG4 Encoder kernel module.

obj-$(CONFIG_MXC_HMP4E) 		+= mxc_hmp4e.o

ifeq ($(CONFIG_MXC_HMP4E_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
