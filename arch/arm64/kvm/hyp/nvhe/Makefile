# SPDX-License-Identifier: GPL-2.0

lib-objs := clear_page.o copy_page.o memcpy.o memset.o
lib-objs := $(addprefix ../../../lib/, $(lib-objs))

CFLAGS_switch.nvhe.o += -Wno-override-init

hyp-obj-y := timer-sr.o sysreg-sr.o debug-sr.o switch.o tlb.o hyp-init.o host.o \
	 hyp-main.o hyp-smp.o psci-relay.o alloc.o early_alloc.o page_alloc.o \
	 cache.o setup.o mm.o mem_protect.o sys_regs.o pkvm.o stacktrace.o ffa.o \
	 serial.o alloc_mgt.o iommu/iommu.o power/hvc.o  device/device.o
hyp-obj-y += ../vgic-v3-sr.o ../aarch32.o ../vgic-v2-cpuif-proxy.o ../entry.o \
	 ../fpsimd.o ../hyp-entry.o ../exception.o ../pgtable.o ../../config.o
hyp-obj-y += ../../../kernel/smccc-call.o  iommu/pviommu-host.o iommu/pviommu.o
hyp-obj-$(CONFIG_LIST_HARDENED) += list_debug.o
hyp-obj-$(CONFIG_TRACING) += clock.o events.o trace.o
hyp-obj-$(CONFIG_PKVM_FTRACE) += ftrace.o
hyp-obj-$(CONFIG_MODULES) += modules.o
hyp-obj-y += $(lib-objs)

KBUILD_CFLAGS += -D __NO_FORTIFY

# Only for built-in, not modules
HYP_SMMU_V3_DRV_PATH = ../../../../../drivers/iommu/arm/arm-smmu-v3

ifeq ($(CONFIG_ARM_SMMU_V3_PKVM),y)
subdir-ccflags-y += -I$(srctree)/drivers/iommu/arm/arm-smmu-v3/pkvm/
hyp-obj-$(CONFIG_ARM_SMMU_V3_PKVM) += $(HYP_SMMU_V3_DRV_PATH)/pkvm/nested/arm-smmu-v3.o \
	$(HYP_SMMU_V3_DRV_PATH)/arm-smmu-v3-common-lib.o \
	$(HYP_SMMU_V3_DRV_PATH)/pkvm/io-pgtable-arm-hyp.o \
	$(HYP_SMMU_V3_DRV_PATH)/../../io-pgtable-arm.o \
	$(HYP_SMMU_V3_DRV_PATH)/pkvm/arm-smmu-v3-lib-hyp.o
endif

ifeq ($(CONFIG_ARM_SMMU_V3_PKVM_PV),y)
subdir-ccflags-y += -I$(srctree)/drivers/iommu/arm/arm-smmu-v3/pkvm/
hyp-obj-$(CONFIG_ARM_SMMU_V3_PKVM_PV) += $(HYP_SMMU_V3_DRV_PATH)/pkvm/pv/arm-smmu-v3-pv.o \
	$(HYP_SMMU_V3_DRV_PATH)/arm-smmu-v3-common-lib.o \
	$(HYP_SMMU_V3_DRV_PATH)/pkvm/io-pgtable-arm-hyp.o \
	$(HYP_SMMU_V3_DRV_PATH)/../../io-pgtable-arm.o \
	$(HYP_SMMU_V3_DRV_PATH)/pkvm/arm-smmu-v3-lib-hyp.o
endif

$(obj)/hyp.lds: $(src)/hyp.lds.S FORCE
	$(call if_changed_dep,cpp_lds_S)

include $(srctree)/arch/arm64/kvm/hyp/nvhe/Makefile.nvhe

CFLAGS_REMOVE_trace.nvhe.o := -fpatchable-function-entry=3
CFLAGS_REMOVE_events.nvhe.o := -fpatchable-function-entry=3
CFLAGS_REMOVE_ftrace.nvhe.o := -fpatchable-function-entry=3
CFLAGS_REMOVE_setup.nvhe.o := -fpatchable-function-entry=3
CFLAGS_REMOVE_stacktrace.nvhe.o := -fpatchable-function-entry=3

obj-y := kvm_nvhe.o

ifeq ($(CONFIG_UBSAN_KVM_EL2),y)
UBSAN_SANITIZE := y
# Always use brk and not hooks
ccflags-y += $(CFLAGS_UBSAN_TRAP)
endif
