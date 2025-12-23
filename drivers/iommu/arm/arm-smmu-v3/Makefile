# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_ARM_SMMU_V3) += arm_smmu_v3.o
arm_smmu_v3-y := arm-smmu-v3.o arm-smmu-v3-common-lib.o arm-smmu-v3-common.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_IOMMUFD) += arm-smmu-v3-iommufd.o
arm_smmu_v3-$(CONFIG_ARM_SMMU_V3_SVA) += arm-smmu-v3-sva.o
arm_smmu_v3-$(CONFIG_TEGRA241_CMDQV) += tegra241-cmdqv.o

obj-$(CONFIG_ARM_SMMU_V3_PKVM) += arm_smmu_v3_kvm.o
ccflags-$(CONFIG_ARM_SMMU_V3_PKVM) += -Iarch/arm64/kvm/
arm_smmu_v3_kvm-objs-y += arm-smmu-v3-kvm.o
arm_smmu_v3_kvm-objs := $(arm_smmu_v3_kvm-objs-y)

# Build hypervisor objects for module
ifeq ($(CONFIG_ARM_SMMU_V3_PKVM),m)
$(obj)/pkvm/nested/kvm_nvhe.o: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/pkvm/nested $(obj)/pkvm/nested/kvm_nvhe.o
arm_smmu_v3_kvm-objs += pkvm/nested/kvm_nvhe.o
endif

obj-$(CONFIG_ARM_SMMU_V3_PKVM_PV) += arm_smmu_v3_kvm_pv.o
ccflags-$(CONFIG_ARM_SMMU_V3_PKVM_PV) += -Iarch/arm64/kvm/
arm_smmu_v3_kvm_pv-objs-y += arm-smmu-v3-kvm-pv.o arm-smmu-v3-common.o
arm_smmu_v3_kvm_pv-objs := $(arm_smmu_v3_kvm_pv-objs-y)

# Build hypervisor objects for module
ifeq ($(CONFIG_ARM_SMMU_V3_PKVM_PV),m)
$(obj)/pkvm/pv/kvm_nvhe.o: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/pkvm/pv $(obj)/pkvm/pv/kvm_nvhe.o

arm_smmu_v3_kvm_pv-objs += pkvm/pv/kvm_nvhe.o
endif

obj-$(CONFIG_ARM_SMMU_V3_KUNIT_TEST) += arm-smmu-v3-test.o
