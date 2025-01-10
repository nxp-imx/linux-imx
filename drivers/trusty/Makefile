# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for trusty components
#

obj-$(CONFIG_TRUSTY)		+= trusty-core.o
trusty-core-objs		+= trusty.o trusty-mem.o trusty-sched-share.o
trusty-core-$(CONFIG_TRUSTY_IRQ)+= trusty-irq.o
obj-$(CONFIG_TRUSTY_LOG)	+= trusty-log.o
obj-$(CONFIG_TRUSTY_TEST)	+= trusty-test.o
trusty-test-objs		+= trusty-test-main.o
trusty-test-$(CONFIG_ARM64)	+= trusty-test-arm64.o
obj-$(CONFIG_TRUSTY_VIRTIO)	+= trusty-virtio.o
obj-$(CONFIG_TRUSTY_VIRTIO_IPC)	+= trusty-ipc.o

obj-$(CONFIG_TRUSTY_SMC_TRANSPORT)	+= trusty-smc.o
trusty-smc-objs 			+= trusty-smc-transport.o
trusty-smc-$(CONFIG_ARM)		+= trusty-smc-arm.o
trusty-smc-$(CONFIG_ARM64)		+= trusty-smc-arm64.o

obj-$(CONFIG_TRUSTY_FFA_TRANSPORT)	+= trusty-ffa.o
trusty-ffa-objs 			+= trusty-ffa-transport.o

obj-$(CONFIG_TRUSTY_POPULATE)		+= trusty-populate.o

ccflags-y			+= -I$(src) -I$(src)/../../include
