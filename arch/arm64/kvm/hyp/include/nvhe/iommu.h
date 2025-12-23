/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ARM64_KVM_NVHE_IOMMU_H__
#define __ARM64_KVM_NVHE_IOMMU_H__

#include <asm/kvm_host.h>
#include <asm/kvm_pgtable.h>

#include <linux/iommu.h>

#include <nvhe/alloc_mgt.h>

struct kvm_iommu_ops;
struct kvm_hyp_iommu_domain {
	atomic_t		refs;
	pkvm_handle_t		domain_id;
	void			*priv;
	struct kvm_iommu_ops	*driver;
	struct pkvm_hyp_vm		*vm;
};

int kvm_iommu_dev_block_dma(pkvm_handle_t iommu_id, u32 endpoint_id, bool host_to_guest);

int kvm_iommu_force_free_domain(pkvm_handle_t domain_id, struct pkvm_hyp_vm *vm);
int kvm_iommu_id_to_token(pkvm_handle_t smmu_id, u64 *out_token);

struct kvm_iommu_ops {
	int (*init)(pkvm_handle_t drv_id);
	void (*host_stage2_idmap)(phys_addr_t start, phys_addr_t end, int prot);
	int (*attach_dev)(pkvm_handle_t iommu, struct kvm_hyp_iommu_domain *domain,
			  pkvm_handle_t dev, u32 pasid, u32 pasid_bits, unsigned long flags);
	int (*detach_dev)(pkvm_handle_t iommu, struct kvm_hyp_iommu_domain *domain,
			  pkvm_handle_t dev, u32 pasid);
	bool (*dabt_handler)(struct user_pt_regs *regs, u64 esr, u64 addr);
	void (*host_stage2_idmap_complete)(bool map);
	int (*alloc_domain)(pkvm_handle_t iommu_id, struct kvm_hyp_iommu_domain *domain, int type);
	void (*free_domain)(struct kvm_hyp_iommu_domain *domain);
	int (*map_pages)(struct kvm_hyp_iommu_domain *domain, unsigned long iova,
			 phys_addr_t paddr, size_t pgsize,
			 size_t pgcount, int prot, size_t *total_mapped);
	size_t (*unmap_pages)(struct kvm_hyp_iommu_domain *domain, unsigned long iova,
			      size_t pgsize, size_t pgcount,
			      struct iommu_iotlb_gather *gather);
	phys_addr_t (*iova_to_phys)(struct kvm_hyp_iommu_domain *domain, unsigned long iova);
	void (*iotlb_sync)(struct kvm_hyp_iommu_domain *domain,
			   struct iommu_iotlb_gather *gather);
	int (*set_identity)(pkvm_handle_t iommu, pkvm_handle_t dev, bool state);
	int (*iotlb_sync_map)(struct kvm_hyp_iommu_domain *domain,
			      unsigned long iova, size_t size);
	int (*dev_block_dma)(pkvm_handle_t iommu, u32 endpoint_id,
			     bool is_host_to_guest);
	int (*get_iommu_token_by_id)(pkvm_handle_t smmu_id, u64 *out_token);
};

int kvm_iommu_init(void *pool_base, size_t nr_pages);
int kvm_iommu_register_ops(struct kvm_iommu_ops *ops, pkvm_handle_t *drv_id);

void kvm_iommu_host_stage2_idmap(phys_addr_t start, phys_addr_t end,
				 enum kvm_pgtable_prot prot);
void *kvm_iommu_donate_pages_atomic(u8 order);
void kvm_iommu_reclaim_pages_atomic(void *ptr);
bool kvm_iommu_host_dabt_handler(struct user_pt_regs *regs, u64 esr, u64 addr);
void kvm_iommu_host_stage2_idmap_complete(bool map);

/* Hypercall handlers */
int kvm_iommu_alloc_domain(pkvm_handle_t drv_id, pkvm_handle_t iommu_id,
			   pkvm_handle_t domain_id, int type);
int kvm_iommu_free_domain(pkvm_handle_t domain_id);
int kvm_iommu_attach_dev(pkvm_handle_t iommu_id, pkvm_handle_t domain_id,
			 u32 endpoint_id, u32 pasid, u32 pasid_bits,
			 unsigned long flags);
int kvm_iommu_detach_dev(pkvm_handle_t iommu_id, pkvm_handle_t domain_id,
			 u32 endpoint_id, u32 pasid);

int kvm_iommu_map_pages(pkvm_handle_t domain_id,
			unsigned long iova, phys_addr_t paddr, size_t pgsize,
			size_t pgcount, int prot, unsigned long *mapped);
size_t kvm_iommu_unmap_pages(pkvm_handle_t domain_id, unsigned long iova,
			     size_t pgsize, size_t pgcount);
phys_addr_t kvm_iommu_iova_to_phys(pkvm_handle_t domain_id, unsigned long iova);
int kvm_iommu_set_identity(pkvm_handle_t drv_id, pkvm_handle_t iommu,
			   pkvm_handle_t dev, bool on);
size_t kvm_iommu_map_sg(pkvm_handle_t domain, unsigned long iova, struct kvm_iommu_sg *sg,
			unsigned int nent, unsigned int prot);
int kvm_iommu_iotlb_sync_map(pkvm_handle_t domain_id,
			     unsigned long iova, size_t size);
/* Flags not used and added for future use. */
void *kvm_iommu_donate_pages(u8 order, int flags);
void kvm_iommu_reclaim_pages(void *p, u8 order);

#define kvm_iommu_donate_page()		kvm_iommu_donate_pages(0, 0)
#define kvm_iommu_reclaim_page(p)		kvm_iommu_reclaim_pages(p, 0)

/* IOMMU variants for drivers which are clueless about VCPUs. */
int iommu_pkvm_use_dma(u64 phys_addr, size_t size);
int iommu_pkvm_unuse_dma(u64 phys_addr, size_t size);

void kvm_iommu_iotlb_gather_add_page(struct kvm_hyp_iommu_domain *domain,
				     struct iommu_iotlb_gather *gather,
				     unsigned long iova, size_t size);

int kvm_iommu_register_pviommu_drv(pkvm_handle_t iommu_id);

extern struct hyp_mgt_allocator_ops kvm_iommu_allocator_ops;
extern pkvm_handle_t pviommu_drv_id;
#endif /* __ARM64_KVM_NVHE_IOMMU_H__ */
