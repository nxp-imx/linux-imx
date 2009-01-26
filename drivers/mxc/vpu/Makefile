#
# Makefile for the VPU drivers.
#

obj-$(CONFIG_MXC_VPU)                  += vpu.o
vpu-objs	:= mxc_vpu.o mxc_vl2cc.o

ifeq ($(CONFIG_MXC_VPU_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
