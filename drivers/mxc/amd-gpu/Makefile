EXTRA_CFLAGS := \
	-D_LINUX \
	-I$(obj)/include \
	-I$(obj)/include/api \
	-I$(obj)/include/ucode \
	-I$(obj)/platform/hal/linux \
	-I$(obj)/os/include \
	-I$(obj)/os/kernel/include \
	-I$(obj)/os/user/include

obj-$(CONFIG_MXC_AMD_GPU) += gpu.o
gpu-objs += 	common/gsl_cmdstream.o \
		common/gsl_cmdwindow.o \
		common/gsl_context.o \
		common/gsl_debug_pm4.o \
		common/gsl_device.o \
		common/gsl_drawctxt.o \
		common/gsl_driver.o \
		common/gsl_g12.o \
		common/gsl_intrmgr.o \
		common/gsl_memmgr.o \
		common/gsl_mmu.o \
		common/gsl_ringbuffer.o \
		common/gsl_sharedmem.o \
		common/gsl_yamato.o \
		platform/hal/linux/gsl_linux_map.o \
		platform/hal/linux/gsl_kmod.o \
		platform/hal/linux/gsl_kmod_cleanup.o \
		platform/hal/linux/misc.o \
		os/kernel/src/linux/kos_lib.o
ifeq ($(CONFIG_ARCH_MX5),y)
EXTRA_CFLAGS += -DMX51=1 \
		-I$(obj)/platform/hal/MX51 \
		-I$(obj)/platform/hal/MX51/linux \
		-I$(obj)/platform/hal/MX51/memcfg

gpu-objs +=	platform/hal/MX51/linux/gsl_hal.o \
		platform/hal/MX51/memcfg/gsl_memcfg.o
endif

ifeq ($(CONFIG_ARCH_MX35),y)
EXTRA_CFLAGS += -DMX35=1 \
		-I$(obj)/platform/hal/MX35 \
		-I$(obj)/platform/hal/MX35/linux \
		-I$(obj)/platform/hal/MX35/memcfg

gpu-objs +=	platform/hal/MX35/linux/gsl_hal.o \
		platform/hal/MX35/memcfg/gsl_memcfg.o
endif
