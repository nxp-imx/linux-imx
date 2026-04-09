#
# Makefile for the VPU drivers.
#

obj-y += hantrodec/
obj-y += hantroenc/
obj-y += memory_usage/
obj-y += coda980/
obj-y += wave511/
obj-y += wave6/

obj-$(CONFIG_MXC_VPU)                  += mxc_vpu.o

ifeq ($(CONFIG_MXC_VPU_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif


