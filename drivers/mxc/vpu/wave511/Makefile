# SPDX-License-Identifier: GPL-2.0

wave5-objs := \
	wave5-vpu.o \
	wave5-vpuapi.o \
	wave5-helper.o \
	wave5-vpu-dec.o \
	wave5-hw.o

wave5-ctrl-objs := wave5-vpu-ctrl.o \
		   wave5-vdi.o

obj-$(CONFIG_MXC_VIDEO_WAVE511_CTRL) += wave5-ctrl.o
obj-$(CONFIG_MXC_VIDEO_WAVE511) += wave5.o

EXTRA_CFLAGS += -Wframe-larger-than=1024
# EXTRA_CFLAGS += -Wstack-usage=256
