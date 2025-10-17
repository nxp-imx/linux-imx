#
# Makefile for the VPU drivers.
#

obj-y += memory_usage/
obj-y += wave6/

obj-$(CONFIG_MXC_VPU)                  += mxc_vpu.o

ifeq ($(CONFIG_MXC_VPU_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif


