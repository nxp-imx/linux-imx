// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Google LLC
 * Author: Mostafa Saleh <smostafa@google.com>
 */

#include <asm/kvm_host.h>
#include <asm/kvm_mmu.h>
#include <asm/kvm_pkvm.h>

#include <linux/io-pgtable.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#include "pkvm/arm_smmu_v3.h"
#include "arm-smmu-v3.h"
#include "../../io-pgtable-arm.h"

extern struct kvm_iommu_ops kvm_nvhe_sym(smmu_pv_ops);

struct kvm_arm_smmu_domain {
	struct iommu_domain		domain;
	struct arm_smmu_device		*smmu;
	pkvm_handle_t			id;
};

static pkvm_handle_t hyp_drv_id;

#define to_kvm_smmu_domain(_domain) \
	container_of(_domain, struct kvm_arm_smmu_domain, domain)

#ifdef MODULE
#define ksym_ref_addr_nvhe(x) \
	((typeof(kvm_nvhe_sym(x)) *)(pkvm_el2_mod_va(&kvm_nvhe_sym(x))))

int kvm_nvhe_sym(smmu_init_hyp_module)(const struct pkvm_module_ops *ops);
#else
#define ksym_ref_addr_nvhe(x) \
	((typeof(kvm_nvhe_sym(x)) *)(kern_hyp_va(lm_alias(&kvm_nvhe_sym(x)))))
#endif

static size_t smmu_hyp_pgt_pages(void)
{
	/*
	 * SMMUv3 uses the same format as stage-2 and hence have the same memory
	 * requirements, we add extra 500 pages for L2 ste.
	 * For modules, we can't use host_s2_pgtable_pages(), so we return 1 page,
	 * and rely on the vendor passing the right commandline arg.
	 */
	if (of_find_compatible_node(NULL, NULL, "arm,smmu-v3")) {
#ifdef MODULE
		return 1;
#else
		return host_s2_pgtable_pages() + 500;
#endif
	}
	return 0;
}

struct host_arm_smmu_device {
	struct arm_smmu_device		smmu;
	pkvm_handle_t			id;
	u32				boot_gbpa;
	phys_addr_t			ioaddr;
	struct kvm_power_domain         power_domain;
};

#define smmu_to_host(_smmu) \
	container_of(_smmu, struct host_arm_smmu_device, smmu);

static size_t				kvm_arm_smmu_cur;
static size_t				kvm_arm_smmu_count;
static struct hyp_arm_smmu_v3_device_pv	*kvm_arm_smmu_array;
static DEFINE_IDA(kvm_arm_smmu_domain_ida);

static struct platform_driver kvm_arm_smmu_driver;

static struct arm_smmu_device *
kvm_arm_smmu_get_by_fwnode(struct fwnode_handle *fwnode)
{
	struct device *dev;
	dev = driver_find_device_by_fwnode(&kvm_arm_smmu_driver.driver, fwnode);
	put_device(dev);
	return dev ? dev_get_drvdata(dev) : NULL;
}

static struct iommu_device *kvm_arm_smmu_probe_device(struct device *dev)
{
	struct arm_smmu_device *smmu;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct arm_smmu_master *master;
	int ret;
	if (WARN_ON_ONCE(dev_iommu_priv_get(dev)))
		return ERR_PTR(-EBUSY);
	smmu = kvm_arm_smmu_get_by_fwnode(fwspec->iommu_fwnode);
	if (!smmu)
		return ERR_PTR(-ENODEV);
	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master)
		return ERR_PTR(-ENOMEM);
	master->dev = dev;
	master->smmu = smmu;
	dev_iommu_priv_set(dev, master);
	device_property_read_u32(dev, "pasid-num-bits", &master->ssid_bits);
	master->ssid_bits = min(smmu->ssid_bits, master->ssid_bits);
	ret = arm_smmu_insert_master(smmu, master, false);
	if (ret)
		goto err_free_master;
	device_link_add(dev, smmu->dev,
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_AUTOREMOVE_SUPPLIER);

	return &smmu->iommu;
err_free_master:
	kfree(master);
	return ERR_PTR(ret);
}

static bool kvm_arm_smmu_capable(struct device *dev, enum iommu_cap cap)
{
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return master->smmu->features & ARM_SMMU_FEAT_COHERENCY;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static void kvm_arm_smmu_detach_dev_pasid(struct device *dev,
					  struct iommu_domain *domain,
					  ioasid_t pasid)
{
	int i;
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(master->smmu);
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);

	if (domain->type == IOMMU_DOMAIN_IDENTITY) {
		if (WARN_ON(pasid))
			return;
		for (i = 0; i < master->num_streams; i++) {
			int ret;
			u32 sid = master->streams[i].id;

			ret = kvm_iommu_set_identity(hyp_drv_id, host_smmu->id, sid, false);
			if (ret)
				dev_err(dev, "Failed to disable identity(sid=0x%x) %d\n",
					sid, ret);
		}
		return;
	}

	for (i = 0; i < master->num_streams; i++)
		kvm_iommu_detach_dev(host_smmu->id, kvm_smmu_domain->id,
				     master->streams[i].id, pasid);
}
static void kvm_arm_smmu_release_device(struct device *dev)
{
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);

	if (domain)
		kvm_arm_smmu_detach_dev_pasid(dev, domain, 0);
	arm_smmu_remove_master(master);
}

static int kvm_arm_smmu_attach_dev_pasid(struct iommu_domain *domain,
					 struct device *dev, ioasid_t pasid,
					 struct iommu_domain *old)
{
	int i, ret = 0;
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(master->smmu);
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);

	/* Blocked dev's will have invalid STEs as they are detached.*/
	if (old && old->type != IOMMU_DOMAIN_BLOCKED)
		kvm_arm_smmu_detach_dev_pasid(dev, old, pasid);

	if (domain->type == IOMMU_DOMAIN_BLOCKED)
		return 0;

	/* IOMMU_DOMAIN_BLOCKED are not backed by kvm_arm_smmu_domain. */
	if (master->smmu != kvm_smmu_domain->smmu)
		return -EINVAL;

	for (i = 0; i < master->num_streams; i++) {
		ret = kvm_iommu_attach_dev(host_smmu->id, kvm_smmu_domain->id,
					   master->streams[i].id, pasid,
					   master->ssid_bits, 0);
		if (ret) {
			dev_err(dev, "Failed to attach device to SMMUv3: %d\n", ret);
			goto out_err;
		}
	}
	return ret;
out_err:
	while (i--)
		kvm_iommu_detach_dev(host_smmu->id, kvm_smmu_domain->id,
				     master->streams[i].id, pasid);
	return ret;
}
static int kvm_arm_smmu_attach_dev(struct iommu_domain *domain,
				   struct device *dev)
{
	struct iommu_domain *old = iommu_get_domain_for_dev(dev);

	return kvm_arm_smmu_attach_dev_pasid(domain, dev, 0, old);
}

/* The main kvm_arm_smmu_attach_dev() handles also the blocked domain. */
static const struct iommu_domain_ops kvm_arm_smmu_blocked_ops = {
	.attach_dev = kvm_arm_smmu_attach_dev,
};

static int kvm_arm_smmu_domain_finalize(struct kvm_arm_smmu_domain *kvm_smmu_domain,
					struct arm_smmu_master *master)
{
	int ret = 0;
	struct arm_smmu_device *smmu = master->smmu;
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);
	struct io_pgtable_cfg cfg;

	cfg.ias = smmu->ias;
	cfg.oas = smmu->oas;
	cfg.pgsize_bitmap = smmu->pgsize_bitmap;

	arm_lpae_restrict_pgsizes(&cfg);
	kvm_smmu_domain->domain.pgsize_bitmap = cfg.pgsize_bitmap;
	kvm_smmu_domain->domain.geometry.aperture_end = (1UL << cfg.ias) - 1;
	kvm_smmu_domain->domain.geometry.force_aperture = true;

	ret = ida_alloc_range(&kvm_arm_smmu_domain_ida, 1,
			      KVM_IOMMU_MAX_HOST_DOMAINS, GFP_KERNEL);
	if (ret < 0)
		return ret;

	kvm_smmu_domain->id = ret;
	ret = kvm_iommu_alloc_domain(hyp_drv_id, host_smmu->id, kvm_smmu_domain->id, KVM_ARM_SMMU_DOMAIN_S1);
	if (ret) {
		ida_free(&kvm_arm_smmu_domain_ida, kvm_smmu_domain->id);
		return ret;
	}

	kvm_smmu_domain->smmu = smmu;
	return ret;
}

static int kvm_arm_smmu_map_pages(struct iommu_domain *domain,
				  unsigned long iova, phys_addr_t paddr,
				  size_t pgsize, size_t pgcount, int prot,
				  gfp_t gfp, size_t *total_mapped)
{
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);

	if (!kvm_smmu_domain->smmu)
		return -ENODEV;

	return kvm_iommu_map_pages(kvm_smmu_domain->id, iova, paddr,
				   pgsize, pgcount, prot, gfp, total_mapped);
}
static size_t kvm_arm_smmu_unmap_pages(struct iommu_domain *domain,
				       unsigned long iova, size_t pgsize,
				       size_t pgcount,
				       struct iommu_iotlb_gather *iotlb_gather)
{
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);

	if (!kvm_smmu_domain->smmu)
		return 0;

	return kvm_iommu_unmap_pages(kvm_smmu_domain->id, iova, pgsize, pgcount);
}
static phys_addr_t kvm_arm_smmu_iova_to_phys(struct iommu_domain *domain,
					     dma_addr_t iova)
{
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);

	if (!kvm_smmu_domain->smmu)
		return 0;

	return kvm_iommu_iova_to_phys(kvm_smmu_domain->id, iova);
}

static struct kvm_arm_smmu_domain *kvm_arm_smmu_domain_alloc(void)
{
	struct kvm_arm_smmu_domain *smmu_domain;

	smmu_domain = kzalloc(sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return ERR_PTR(-ENOMEM);
	return smmu_domain;
}

static struct iommu_domain *kvm_arm_smmu_domain_alloc_paging(struct device *dev)
{
	struct kvm_arm_smmu_domain *smmu_domain = kvm_arm_smmu_domain_alloc();
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	int ret;

	if (IS_ERR(smmu_domain))
		return ERR_PTR(PTR_ERR(smmu_domain));

	ret = kvm_arm_smmu_domain_finalize(smmu_domain, master);
	if (ret) {
		kfree(smmu_domain);
		return ERR_PTR(ret);
	}
	return &smmu_domain->domain;
}

static void kvm_arm_smmu_free_domain(struct iommu_domain *domain)
{
	int ret;
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(domain);
	struct arm_smmu_device *smmu = kvm_smmu_domain->smmu;

	if (smmu) {
		ret = kvm_iommu_free_domain(kvm_smmu_domain->id);
		if (ret)
			dev_err(smmu->dev, "Failed to free domain %d\n", ret);
		ida_free(&kvm_arm_smmu_domain_ida, kvm_smmu_domain->id);
	}
	kfree(kvm_smmu_domain);
}

static struct iommu_domain kvm_arm_smmu_blocked_domain = {
	.type = IOMMU_DOMAIN_BLOCKED,
	.ops = &kvm_arm_smmu_blocked_ops,
};

static int kvm_arm_smmu_def_domain_type(struct device *dev)
{
	if (device_property_read_bool(dev, "iommu-idmapped"))
		return IOMMU_DOMAIN_IDENTITY;
	return 0;
}

static int kvm_arm_smmu_attach_dev_identity(struct iommu_domain *domain,
					    struct device *dev)
{
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);
	struct arm_smmu_device *smmu = master->smmu;
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);
	int i, ret;

	for (i = 0; i < master->num_streams; i++) {
		u32 sid = master->streams[i].id;

		ret = kvm_iommu_set_identity(hyp_drv_id, host_smmu->id, sid, true);
		if (ret) {
			dev_err(dev, "Failed to enable identity(sid=0x%x) %d\n", sid, ret);
			return ret;
		}
	}
	return 0;
}

static const struct iommu_domain_ops kvm_arm_smmu_identity_ops = {
	.attach_dev = kvm_arm_smmu_attach_dev_identity,
};

static struct iommu_domain kvm_arm_smmu_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &kvm_arm_smmu_identity_ops,

};

struct kvm_arm_smmu_map_sg {
	struct iommu_map_cookie_sg cookie;
	struct kvm_iommu_sg *sg;
	unsigned int ptr;
	unsigned long iova;
	int prot;
	gfp_t gfp;
	unsigned int nents;
	size_t total_mapped;
	size_t size; /* Total size of entries not mapped yet. */
};

static struct iommu_map_cookie_sg *kvm_arm_smmu_alloc_cookie_sg(unsigned long iova,
								int prot,
								unsigned int nents,
								gfp_t gfp)
{
	int ret;
	struct kvm_arm_smmu_map_sg *map_sg = kzalloc(sizeof(*map_sg), gfp);

	if (!map_sg)
		return NULL;

	/* Rounds nents to allocate to page aligned size. */
	map_sg->nents = kvm_iommu_sg_nents_round(nents);
	map_sg->sg = kvm_iommu_sg_alloc(map_sg->nents, gfp);
	if (!map_sg->sg)
		return NULL;
	map_sg->iova = iova;
	map_sg->prot = prot;
	map_sg->gfp = gfp;
	ret = kvm_iommu_share_hyp_sg(map_sg->sg, map_sg->nents);
	if (ret) {
		kvm_iommu_sg_free(map_sg->sg, map_sg->nents);
		kfree(map_sg);
		return NULL;
	}

	return &map_sg->cookie;
}

static int kvm_arm_smmu_add_deferred_map_sg(struct iommu_map_cookie_sg *cookie,
					    phys_addr_t paddr, size_t pgsize, size_t pgcount)
{
	struct kvm_arm_smmu_map_sg *map_sg = container_of(cookie, struct kvm_arm_smmu_map_sg,
							  cookie);
	struct kvm_iommu_sg *sg = map_sg->sg;
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(map_sg->cookie.domain);
	size_t mapped;

	/* Out of space, flush the list. */
	if (map_sg->nents == map_sg->ptr) {
		mapped = kvm_iommu_map_sg(kvm_smmu_domain->id, sg, map_sg->iova,
					  map_sg->ptr, map_sg->prot, map_sg->gfp);
		/*
		 * Something went wrong, undo the mappings from the current sg list,
		 * leaving total mapped as it would be unmapped from core code as
		 * kvm_arm_smmu_consume_deferred_map_sg() would return total_mapped.
		 */
		if (mapped != map_sg->size) {
			iommu_unmap(&kvm_smmu_domain->domain, map_sg->iova, mapped);
			/*
			 * The core code will try to consume the list in the error path
			 * don't attempt to map this list again as it already failed, so
			 * no need to waste time.
			 */
			map_sg->ptr = 0;
			return -EINVAL;
		}
		map_sg->ptr = 0;
		map_sg->iova += mapped;
		map_sg->total_mapped += mapped;
		map_sg->size = 0;
	}

	sg[map_sg->ptr].phys = paddr;
	sg[map_sg->ptr].pgsize = pgsize;
	sg[map_sg->ptr].pgcount = pgcount;
	map_sg->size += pgsize * pgcount;
	map_sg->ptr++;
	return 0;
}

static size_t kvm_arm_smmu_consume_deferred_map_sg(struct iommu_map_cookie_sg *cookie)
{
	struct kvm_arm_smmu_map_sg *map_sg = container_of(cookie, struct kvm_arm_smmu_map_sg,
							  cookie);
	struct kvm_iommu_sg *sg = map_sg->sg;
	size_t total_mapped = map_sg->total_mapped;
	struct kvm_arm_smmu_domain *kvm_smmu_domain = to_kvm_smmu_domain(map_sg->cookie.domain);

	/* Might be cleared from error path. */
	if (map_sg->ptr)
		total_mapped += kvm_iommu_map_sg(kvm_smmu_domain->id, sg, map_sg->iova,
						 map_sg->ptr, map_sg->prot, map_sg->gfp);
	kvm_iommu_unshare_hyp_sg(sg, map_sg->nents);
	kvm_iommu_sg_free(sg, map_sg->nents);
	kfree(map_sg);
	return total_mapped;
}

static struct iommu_ops kvm_arm_smmu_ops = {
	.identity_domain	= &kvm_arm_smmu_identity_domain,
	.capable		= kvm_arm_smmu_capable,
	.device_group		= arm_smmu_device_group,
	.of_xlate		= arm_smmu_of_xlate,
	.get_resv_regions	= arm_smmu_get_resv_regions,
	.probe_device		= kvm_arm_smmu_probe_device,
	.release_device		= kvm_arm_smmu_release_device,
	.owner			= THIS_MODULE,
	.domain_alloc_paging	= kvm_arm_smmu_domain_alloc_paging,
	.blocked_domain		= &kvm_arm_smmu_blocked_domain,
	.def_domain_type	= kvm_arm_smmu_def_domain_type,
	.default_domain_ops 	=  &(const struct iommu_domain_ops) {
		.attach_dev	= kvm_arm_smmu_attach_dev,
		.set_dev_pasid	= kvm_arm_smmu_attach_dev_pasid,
		.iova_to_phys	= kvm_arm_smmu_iova_to_phys,
		.map_pages	= kvm_arm_smmu_map_pages,
		.unmap_pages	= kvm_arm_smmu_unmap_pages,
		.free		= kvm_arm_smmu_free_domain,
		.alloc_cookie_sg = kvm_arm_smmu_alloc_cookie_sg,
		.add_deferred_map_sg = kvm_arm_smmu_add_deferred_map_sg,
		.consume_deferred_map_sg = kvm_arm_smmu_consume_deferred_map_sg,
	}
};

static bool kvm_arm_smmu_validate_features(struct arm_smmu_device *smmu)
{
	unsigned int required_features =
		ARM_SMMU_FEAT_TT_LE |
		ARM_SMMU_FEAT_TRANS_S2;
	unsigned int forbidden_features =
		ARM_SMMU_FEAT_STALL_FORCE;
	unsigned int keep_features =
		ARM_SMMU_FEAT_2_LVL_STRTAB	|
		ARM_SMMU_FEAT_2_LVL_CDTAB	|
		ARM_SMMU_FEAT_TT_LE		|
		ARM_SMMU_FEAT_SEV		|
		ARM_SMMU_FEAT_COHERENCY		|
		ARM_SMMU_FEAT_TRANS_S1		|
		ARM_SMMU_FEAT_TRANS_S2		|
		ARM_SMMU_FEAT_VAX		|
		ARM_SMMU_FEAT_RANGE_INV;

	if (smmu->options & ARM_SMMU_OPT_PAGE0_REGS_ONLY) {
		dev_err(smmu->dev, "unsupported layout\n");
		return false;
	}

	if ((smmu->features & required_features) != required_features) {
		dev_err(smmu->dev, "missing features 0x%x\n",
			required_features & ~smmu->features);
		return false;
	}

	if (smmu->features & forbidden_features) {
		dev_err(smmu->dev, "features 0x%x forbidden\n",
			smmu->features & forbidden_features);
		return false;
	}

	smmu->features &= keep_features;

	return true;
}

static irqreturn_t kvm_arm_smmu_evt_handler(int irq, void *dev)
{
	return arm_smmu_evtq_common(irq, dev, arm_smmu_handle_event);
}

static void kvm_arm_smmu_cmdq_err(struct arm_smmu_device *smmu)
{
	dev_err(smmu->dev, "Hypervisor command queue corrupted!\n");
	BUG();
}

static irqreturn_t kvm_arm_smmu_gerror_handler(int irq, void *dev)
{
	return arm_smmu_gerror_common(irq, dev, kvm_arm_smmu_cmdq_err);
}

static irqreturn_t kvm_arm_smmu_combined_handler(int irq, void *dev)
{
	kvm_arm_smmu_gerror_handler(irq, dev);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t kvm_arm_smmu_pri_handler(int irq, void *dev)
{
	struct arm_smmu_device *smmu = dev;

	dev_err(smmu->dev, "PRI not supported in KVM driver!\n");
	return IRQ_HANDLED;
}

static int kvm_arm_smmu_device_reset(struct host_arm_smmu_device *host_smmu)
{
	int ret;
	u32 reg;
	struct arm_smmu_device *smmu = &host_smmu->smmu;

	reg = readl_relaxed(smmu->base + ARM_SMMU_CR0);
	if (reg & CR0_SMMUEN)
		dev_warn(smmu->dev, "SMMU currently enabled! Resetting...\n");

	/* Disable bypass */
	host_smmu->boot_gbpa = readl_relaxed(smmu->base + ARM_SMMU_GBPA);
	ret = arm_smmu_update_gbpa(smmu, GBPA_ABORT, 0);
	if (ret)
		return ret;

	ret = arm_smmu_device_disable(smmu);
	if (ret)
		return ret;

	/* Stream table */
	arm_smmu_write_strtab(smmu);

	/* Command queue */
	writeq_relaxed(smmu->cmdq.q.q_base, smmu->base + ARM_SMMU_CMDQ_BASE);

	/* Event queue */
	writeq_relaxed(smmu->evtq.q.q_base, smmu->base + ARM_SMMU_EVTQ_BASE);
	writel_relaxed(smmu->evtq.q.llq.prod, smmu->base + SZ_64K + ARM_SMMU_EVTQ_PROD);
	writel_relaxed(smmu->evtq.q.llq.cons, smmu->base + SZ_64K + ARM_SMMU_EVTQ_CONS);

	ret = arm_smmu_setup_irqs(smmu,
				  kvm_arm_smmu_evt_handler,
				  kvm_arm_smmu_combined_handler,
				  kvm_arm_smmu_evt_handler,
				  kvm_arm_smmu_gerror_handler,
				  kvm_arm_smmu_pri_handler);
	return 0;
}

static int kvm_arm_probe_power_domain(struct device *dev,
				      struct kvm_power_domain *pd)
{
	int ret;
	struct of_phandle_args args;

	if (!of_get_property(dev->of_node, "power-domains", NULL))
		return 0;

	ret = of_parse_phandle_with_args(dev->of_node, "power-domains",
					 "#power-domain-cells", 0, &args);
	if (ret)
		return ret;

	pd->type = KVM_POWER_DOMAIN_HOST_HVC;
	pd->device_id = kvm_arm_smmu_cur;
	of_node_put(args.np);
	return ret;
}

static int kvm_arm_smmu_probe(struct platform_device *pdev)
{
	int ret;
	size_t size;
	struct resource *res;
	struct arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;
	struct host_arm_smmu_device *host_smmu;
	struct hyp_arm_smmu_v3_device_pv *hyp_smmu;

	if (kvm_arm_smmu_cur >= kvm_arm_smmu_count)
		return -ENOSPC;

	hyp_smmu = &kvm_arm_smmu_array[kvm_arm_smmu_cur];

	host_smmu = devm_kzalloc(dev, sizeof(*host_smmu), GFP_KERNEL);
	if (!host_smmu)
		return -ENOMEM;

	smmu = &host_smmu->smmu;
	smmu->dev = dev;

	ret = arm_smmu_fw_probe(pdev, smmu);
	if (ret)
		return ret;

	ret = kvm_arm_probe_power_domain(dev, &host_smmu->power_domain);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = resource_size(res);
	if (size < SZ_128K) {
		dev_err(dev, "unsupported MMIO region size (%pr)\n", res);
		return -EINVAL;
	}
	host_smmu->ioaddr = res->start;
	host_smmu->id = kvm_arm_smmu_cur;

	smmu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(smmu->base))
		return PTR_ERR(smmu->base);

	arm_smmu_probe_irq(pdev, smmu);

	ret = arm_smmu_device_hw_probe(smmu);
	if (ret)
		return ret;

	if (!kvm_arm_smmu_validate_features(smmu))
		return -ENODEV;

	ret = arm_smmu_init_one_queue(smmu, &smmu->cmdq.q, smmu->base,
				      ARM_SMMU_CMDQ_PROD, ARM_SMMU_CMDQ_CONS,
				      CMDQ_ENT_DWORDS, "cmdq");
	if (ret)
		return ret;

	ret = arm_smmu_init_one_queue(smmu, &smmu->evtq.q, smmu->base + SZ_64K,
				      ARM_SMMU_EVTQ_PROD, ARM_SMMU_EVTQ_CONS,
				      EVTQ_ENT_DWORDS, "evtq");
	if (ret)
		return ret;

	ret = arm_smmu_init_strtab(smmu);
	if (ret)
		return ret;

	ret = kvm_arm_smmu_device_reset(host_smmu);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, smmu);

	if (host_smmu->power_domain.type != KVM_POWER_DOMAIN_NONE) {
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
		/*
		 * Take a reference to keep the SMMU powered on while the hypervisor
		 * initializes it.
		 */
		pm_runtime_resume_and_get(dev);
	}

	/* Hypervisor parameters */
	hyp_smmu->common.cmdq = smmu->cmdq.q;
	hyp_smmu->common.strtab_cfg = smmu->strtab_cfg;
	hyp_smmu->common.pgsize_bitmap = smmu->pgsize_bitmap;
	hyp_smmu->common.oas = smmu->oas;
	hyp_smmu->common.ias = smmu->ias;
	hyp_smmu->common.mmio_addr = host_smmu->ioaddr;
	hyp_smmu->common.mmio_size = size;
	hyp_smmu->common.features = smmu->features;
	hyp_smmu->ssid_bits = smmu->ssid_bits;
	hyp_smmu->evtq = smmu->evtq.q;
	hyp_smmu->power_domain = host_smmu->power_domain;
	kvm_arm_smmu_cur++;

	return 0;
}

static void kvm_arm_smmu_remove(struct platform_device *pdev)
{
	struct arm_smmu_device *smmu = platform_get_drvdata(pdev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);

	/*
	 * There was an error during hypervisor setup. The hyp driver may
	 * have already enabled the device, so disable it.
	 */
	arm_smmu_device_disable(smmu);
	arm_smmu_update_gbpa(smmu, host_smmu->boot_gbpa, GBPA_ABORT);
	if (host_smmu->power_domain.type != KVM_POWER_DOMAIN_NONE) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
	}
	arm_smmu_unregister_iommu(smmu);
}

static int kvm_arm_smmu_suspend(struct device *dev)
{
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);

	if (host_smmu->power_domain.type == KVM_POWER_DOMAIN_HOST_HVC)
		return pkvm_iommu_suspend(host_smmu->id);
	return 0;
}

static int kvm_arm_smmu_resume(struct device *dev)
{
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);

	if (host_smmu->power_domain.type == KVM_POWER_DOMAIN_HOST_HVC)
		return pkvm_iommu_resume(host_smmu->id);
	return 0;
}

static const struct dev_pm_ops kvm_arm_smmu_pm_ops = {
	SET_RUNTIME_PM_OPS(kvm_arm_smmu_suspend, kvm_arm_smmu_resume, NULL)
};

static const struct of_device_id arm_smmu_of_match[] = {
	{ .compatible = "arm,smmu-v3", },
	{ },
};

static struct platform_driver kvm_arm_smmu_driver = {
	.driver = {
		.name = "kvm-arm-smmu-v3",
		.of_match_table = arm_smmu_of_match,
		.pm = &kvm_arm_smmu_pm_ops,
	},
	.remove = kvm_arm_smmu_remove,
};

static int kvm_arm_smmu_array_alloc(void)
{
	int smmu_order;
	struct device_node *np;

	kvm_arm_smmu_count = 0;
	for_each_compatible_node(np, NULL, "arm,smmu-v3")
		kvm_arm_smmu_count++;

	if (!kvm_arm_smmu_count)
		return 0;

	/* Allocate the parameter list shared with the hypervisor */
	smmu_order = get_order(kvm_arm_smmu_count * sizeof(*kvm_arm_smmu_array));
	kvm_arm_smmu_array = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
						      smmu_order);
	if (!kvm_arm_smmu_array)
		return -ENOMEM;

	return 0;
}

static void kvm_arm_smmu_array_free(void)
{
	int order;

	order = get_order(kvm_arm_smmu_count * sizeof(*kvm_arm_smmu_array));
	free_pages((unsigned long)kvm_arm_smmu_array, order);
}

static int smmu_fin_device(struct device *dev, void *data)
{
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);

	if (host_smmu->power_domain.type != KVM_POWER_DOMAIN_NONE)
		pm_runtime_put(dev);

	return arm_smmu_register_iommu(smmu, &kvm_arm_smmu_ops, host_smmu->ioaddr);
}

static int kvm_arm_smmu_v3_post_init(void)
{
	if (!kvm_arm_smmu_count)
		return 0;
	WARN_ON(driver_for_each_device(&kvm_arm_smmu_driver.driver, NULL,
		NULL, smmu_fin_device));
	return 0;
}

static int kvm_arm_smmu_v3_init_drv(void)
{
	int ret;

	ret = platform_driver_probe(&kvm_arm_smmu_driver, kvm_arm_smmu_probe);
	if (ret)
		goto err_free;

	if (kvm_arm_smmu_cur != kvm_arm_smmu_count) {
		/* A device exists but failed to probe */
		ret = -EUNATCH;
		goto err_free;
	}

#ifdef MODULE
	ret = pkvm_load_el2_module(kvm_nvhe_sym(smmu_init_hyp_module));
	if (ret) {
		pr_err("Failed to load SMMUv3 IOMMU EL2 module: %d\n", ret);
		goto err_free;
	}
#endif
	/*
	 * These variables are stored in the nVHE image, and won't be accessible
	 * after KVM initialization. Ownership of kvm_arm_smmu_array will be
	 * transferred to the hypervisor as well.
	 */
	kvm_hyp_arm_smmu_v3_pv_smmus = kvm_arm_smmu_array;
	kvm_hyp_arm_smmu_v3_pv_count = kvm_arm_smmu_count;

	ret = kvm_iommu_register_hyp_ops(ksym_ref_addr_nvhe(smmu_pv_ops), &hyp_drv_id);
	if (ret)
		goto err_free;

	return kvm_arm_smmu_v3_post_init();
err_free:
	kvm_arm_smmu_array_free();
	return ret;
}

static int kvm_arm_smmu_v3_id(struct device *dev)
{
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	struct host_arm_smmu_device *host_smmu = smmu_to_host(smmu);

	return host_smmu->id;
}

static int kvm_arm_v3_id_by_of(struct device_node *np, pkvm_handle_t *out_id)
{
	struct device *dev;

	dev = driver_find_device_by_of_node(&kvm_arm_smmu_driver.driver, np);
	if (!dev)
		return -ENODEV;

	*out_id = kvm_arm_smmu_v3_id(dev);
	put_device(dev);
	return 0;
}

static int kvm_arm_smmu_v3_num_ids(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec)
		return 0;

	return fwspec->num_ids;
}

static int kvm_arm_smmu_v3_device_id(struct device *dev, u32 idx,
				     pkvm_handle_t *out_iommu, u32 *out_sid)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct arm_smmu_master *master = dev_iommu_priv_get(dev);

	if (!fwspec || !master)
		return -ENODEV;
	if (idx >= fwspec->num_ids)
		return -ENOENT;

	*out_sid = fwspec->ids[idx];
	*out_iommu = kvm_arm_smmu_v3_id(master->smmu->dev);
	return 0;
}

static struct kvm_iommu_driver kvm_smmu_v3_ops = {
	.init_driver = kvm_arm_smmu_v3_init_drv,
	.get_iommu_id_by_of = kvm_arm_v3_id_by_of,
	.get_device_iommu_num_ids = kvm_arm_smmu_v3_num_ids,
	.get_device_iommu_id = kvm_arm_smmu_v3_device_id,
};

static int kvm_arm_smmu_v3_register(void)
{
	int ret;

	if (!is_protected_kvm_enabled())
		return 0;

	ret = kvm_arm_smmu_array_alloc();
	if (ret || !kvm_arm_smmu_count)
		return ret;

	ret = kvm_iommu_register_driver(&kvm_smmu_v3_ops, smmu_hyp_pgt_pages());
	if (ret)
		kvm_arm_smmu_array_free();
	return ret;
};

/*
 * Register must be run before de-privliage before kvm_iommu_init_driver
 * for module case, it should be loaded using pKVM early loading which
 * loads it before this point.
 * For builtin drivers we use core_initcall
 */
#ifdef MODULE
module_init(kvm_arm_smmu_v3_register);
#else
core_initcall(kvm_arm_smmu_v3_register);
#endif
MODULE_LICENSE("GPL v2");
