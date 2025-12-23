#ifndef __PKVM_IOMMU_TEMP_PKVM_PKVM_IOMMU_TEMP_H_
#define __PKVM_IOMMU_TEMP_PKVM_PKVM_IOMMU_TEMP_H_

int kvm_nvhe_sym(pkvm_iommu_temp_hyp_init)(const struct pkvm_module_ops *ops);
extern struct kvm_iommu_ops kvm_nvhe_sym(iommu_temp_ops);

#endif  /* __PKVM_IOMMU_TEMP_PKVM_PKVM_IOMMU_TEMP_H_ */
