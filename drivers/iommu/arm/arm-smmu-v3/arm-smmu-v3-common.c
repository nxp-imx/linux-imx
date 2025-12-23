// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/string_choices.h>
#include <kunit/visibility.h>

#include "arm-smmu-v3.h"
#include "../../dma-iommu.h"

#define IIDR_IMPLEMENTER_ARM		0x43b
#define IIDR_PRODUCTID_ARM_MMU_600	0x483
#define IIDR_PRODUCTID_ARM_MMU_700	0x487

struct arm_smmu_option_prop {
	u32 opt;
	const char *prop;
};

static phys_addr_t arm_smmu_msi_cfg[ARM_SMMU_MAX_MSIS][3] = {
	[EVTQ_MSI_INDEX] = {
		ARM_SMMU_EVTQ_IRQ_CFG0,
		ARM_SMMU_EVTQ_IRQ_CFG1,
		ARM_SMMU_EVTQ_IRQ_CFG2,
	},
	[GERROR_MSI_INDEX] = {
		ARM_SMMU_GERROR_IRQ_CFG0,
		ARM_SMMU_GERROR_IRQ_CFG1,
		ARM_SMMU_GERROR_IRQ_CFG2,
	},
	[PRIQ_MSI_INDEX] = {
		ARM_SMMU_PRIQ_IRQ_CFG0,
		ARM_SMMU_PRIQ_IRQ_CFG1,
		ARM_SMMU_PRIQ_IRQ_CFG2,
	},
};

static const char * const event_str[] = {
	[EVT_ID_BAD_STREAMID_CONFIG] = "C_BAD_STREAMID",
	[EVT_ID_STE_FETCH_FAULT] = "F_STE_FETCH",
	[EVT_ID_BAD_STE_CONFIG] = "C_BAD_STE",
	[EVT_ID_STREAM_DISABLED_FAULT] = "F_STREAM_DISABLED",
	[EVT_ID_BAD_SUBSTREAMID_CONFIG] = "C_BAD_SUBSTREAMID",
	[EVT_ID_CD_FETCH_FAULT] = "F_CD_FETCH",
	[EVT_ID_BAD_CD_CONFIG] = "C_BAD_CD",
	[EVT_ID_TRANSLATION_FAULT] = "F_TRANSLATION",
	[EVT_ID_ADDR_SIZE_FAULT] = "F_ADDR_SIZE",
	[EVT_ID_ACCESS_FAULT] = "F_ACCESS",
	[EVT_ID_PERMISSION_FAULT] = "F_PERMISSION",
	[EVT_ID_VMS_FETCH_FAULT] = "F_VMS_FETCH",
};

static const char * const event_class_str[] = {
	[0] = "CD fetch",
	[1] = "Stage 1 translation table fetch",
	[2] = "Input address caused fault",
	[3] = "Reserved",
};

static void arm_smmu_device_iidr_probe(struct arm_smmu_device *smmu)
{
	u32 reg;
	unsigned int implementer, productid, variant, revision;

	reg = readl_relaxed(smmu->base + ARM_SMMU_IIDR);
	implementer = FIELD_GET(IIDR_IMPLEMENTER, reg);
	productid = FIELD_GET(IIDR_PRODUCTID, reg);
	variant = FIELD_GET(IIDR_VARIANT, reg);
	revision = FIELD_GET(IIDR_REVISION, reg);

	switch (implementer) {
	case IIDR_IMPLEMENTER_ARM:
		switch (productid) {
		case IIDR_PRODUCTID_ARM_MMU_600:
			/* Arm erratum 1076982 */
			if (variant == 0 && revision <= 2)
				smmu->features &= ~ARM_SMMU_FEAT_SEV;
			/* Arm erratum 1209401 */
			if (variant < 2)
				smmu->features &= ~ARM_SMMU_FEAT_NESTING;
			break;
		case IIDR_PRODUCTID_ARM_MMU_700:
			/* Arm erratum 2812531 */
			smmu->features &= ~ARM_SMMU_FEAT_BTM;
			smmu->options |= ARM_SMMU_OPT_CMDQ_FORCE_SYNC;
			/* Arm errata 2268618, 2812531 */
			smmu->features &= ~ARM_SMMU_FEAT_NESTING;
			break;
		}
		break;
	}
}

static void arm_smmu_get_httu(struct arm_smmu_device *smmu, u32 reg)
{
	u32 fw_features = smmu->features & (ARM_SMMU_FEAT_HA | ARM_SMMU_FEAT_HD);
	u32 hw_features = 0;

	switch (FIELD_GET(IDR0_HTTU, reg)) {
	case IDR0_HTTU_ACCESS_DIRTY:
		hw_features |= ARM_SMMU_FEAT_HD;
		fallthrough;
	case IDR0_HTTU_ACCESS:
		hw_features |= ARM_SMMU_FEAT_HA;
	}

	if (smmu->dev->of_node)
		smmu->features |= hw_features;
	else if (hw_features != fw_features)
		/* ACPI IORT sets the HTTU bits */
		dev_warn(smmu->dev,
			 "IDR0.HTTU features(0x%x) overridden by FW configuration (0x%x)\n",
			  hw_features, fw_features);
}

int arm_smmu_device_hw_probe(struct arm_smmu_device *smmu)
{
	u32 reg;
	bool coherent = smmu->features & ARM_SMMU_FEAT_COHERENCY;

	/* IDR0 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR0);

	smmu->features |= smmu_idr0_features(reg);
	if (FIELD_GET(IDR0_TTENDIAN, reg) == IDR0_TTENDIAN_RESERVED) {
		dev_err(smmu->dev, "unknown/unsupported TT endianness!\n");
		return -ENXIO;
	}

	if (smmu->features & ARM_SMMU_FEAT_HYP &&
	    cpus_have_cap(ARM64_HAS_VIRT_HOST_EXTN))
		smmu->features |= ARM_SMMU_FEAT_E2H;

	arm_smmu_get_httu(smmu, reg);

	/*
	 * The coherency feature as set by FW is used in preference to the ID
	 * register, but warn on mismatch.
	 */
	if (!!(reg & IDR0_COHACC) != coherent)
		dev_warn(smmu->dev, "IDR0.COHACC overridden by FW configuration (%s)\n",
			 str_true_false(coherent));

	if (!(smmu->features & (ARM_SMMU_FEAT_TRANS_S1 | ARM_SMMU_FEAT_TRANS_S2))) {
		dev_err(smmu->dev, "no translation support!\n");
		return -ENXIO;
	}

	/* We only support the AArch64 table format at present */
	switch (FIELD_GET(IDR0_TTF, reg)) {
	case IDR0_TTF_AARCH32_64:
		smmu->ias = 40;
		fallthrough;
	case IDR0_TTF_AARCH64:
		break;
	default:
		dev_err(smmu->dev, "AArch64 table format not supported!\n");
		return -ENXIO;
	}

	/* ASID/VMID sizes */
	smmu->asid_bits = reg & IDR0_ASID16 ? 16 : 8;
	smmu->vmid_bits = reg & IDR0_VMID16 ? 16 : 8;

	/* IDR1 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR1);
	if (reg & (IDR1_TABLES_PRESET | IDR1_QUEUES_PRESET | IDR1_REL)) {
		dev_err(smmu->dev, "embedded implementation not supported\n");
		return -ENXIO;
	}

	if (reg & IDR1_ATTR_TYPES_OVR)
		smmu->features |= ARM_SMMU_FEAT_ATTR_TYPES_OVR;

	/* Queue sizes, capped to ensure natural alignment */
	smmu->cmdq.q.llq.max_n_shift = min_t(u32, CMDQ_MAX_SZ_SHIFT,
					     FIELD_GET(IDR1_CMDQS, reg));
	if (smmu->cmdq.q.llq.max_n_shift <= ilog2(CMDQ_BATCH_ENTRIES)) {
		/*
		 * We don't support splitting up batches, so one batch of
		 * commands plus an extra sync needs to fit inside the command
		 * queue. There's also no way we can handle the weird alignment
		 * restrictions on the base pointer for a unit-length queue.
		 */
		dev_err(smmu->dev, "command queue size <= %d entries not supported\n",
			CMDQ_BATCH_ENTRIES);
		return -ENXIO;
	}

	smmu->evtq.q.llq.max_n_shift = min_t(u32, EVTQ_MAX_SZ_SHIFT,
					     FIELD_GET(IDR1_EVTQS, reg));
	smmu->priq.q.llq.max_n_shift = min_t(u32, PRIQ_MAX_SZ_SHIFT,
					     FIELD_GET(IDR1_PRIQS, reg));

	/* SID/SSID sizes */
	smmu->ssid_bits = FIELD_GET(IDR1_SSIDSIZE, reg);
	smmu->sid_bits = FIELD_GET(IDR1_SIDSIZE, reg);
	smmu->iommu.max_pasids = 1UL << smmu->ssid_bits;

	/*
	 * If the SMMU supports fewer bits than would fill a single L2 stream
	 * table, use a linear table instead.
	 */
	if (smmu->sid_bits <= STRTAB_SPLIT)
		smmu->features &= ~ARM_SMMU_FEAT_2_LVL_STRTAB;

	/* IDR3 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR3);
	smmu->features |= smmu_idr3_features(reg);

	if (FIELD_GET(IDR3_BBM, reg) == 2)
		smmu->features |= ARM_SMMU_FEAT_BBML2;

	/* IDR5 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR5);

	/* Maximum number of outstanding stalls */
	smmu->evtq.max_stalls = FIELD_GET(IDR5_STALL_MAX, reg);

	/* Page sizes */
	smmu->pgsize_bitmap = smmu_idr5_to_pgsize(reg);

	/* Input address size */
	if (FIELD_GET(IDR5_VAX, reg) == IDR5_VAX_52_BIT)
		smmu->features |= ARM_SMMU_FEAT_VAX;

	smmu->oas = smmu_idr5_to_oas(reg);
	if (smmu->oas == 52)
		smmu->pgsize_bitmap |= 1ULL << 42; /* 4TB */
	else if (!smmu->oas) {
		dev_info(smmu->dev,
			 "unknown output address size. Truncating to 48-bit\n");
		smmu->oas = 48;
	}

	/* Set the DMA mask for our table walker */
	if (dma_set_mask_and_coherent(smmu->dev, DMA_BIT_MASK(smmu->oas)))
		dev_warn(smmu->dev,
			 "failed to set DMA mask for table walker\n");

	smmu->ias = max(smmu->ias, smmu->oas);

	if ((smmu->features & ARM_SMMU_FEAT_TRANS_S1) &&
	    (smmu->features & ARM_SMMU_FEAT_TRANS_S2))
		smmu->features |= ARM_SMMU_FEAT_NESTING;

	arm_smmu_device_iidr_probe(smmu);

	dev_info(smmu->dev, "ias %lu-bit, oas %lu-bit (features 0x%08x)\n",
		 smmu->ias, smmu->oas, smmu->features);
	return 0;
}

int arm_smmu_write_reg_sync(struct arm_smmu_device *smmu, u32 val,
			    unsigned int reg_off, unsigned int ack_off)
{
	u32 reg;

	writel_relaxed(val, smmu->base + reg_off);
	return readl_relaxed_poll_timeout(smmu->base + ack_off, reg, reg == val,
					  1, ARM_SMMU_POLL_TIMEOUT_US);
}

/* GBPA is "special" */
int arm_smmu_update_gbpa(struct arm_smmu_device *smmu, u32 set, u32 clr)
{
	int ret;
	u32 reg, __iomem *gbpa = smmu->base + ARM_SMMU_GBPA;

	ret = readl_relaxed_poll_timeout(gbpa, reg, !(reg & GBPA_UPDATE),
					 1, ARM_SMMU_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	reg &= ~clr;
	reg |= set;
	writel_relaxed(reg | GBPA_UPDATE, gbpa);
	ret = readl_relaxed_poll_timeout(gbpa, reg, !(reg & GBPA_UPDATE),
					 1, ARM_SMMU_POLL_TIMEOUT_US);

	if (ret)
		dev_err(smmu->dev, "GBPA not responding to update\n");
	return ret;
}

int arm_smmu_device_disable(struct arm_smmu_device *smmu)
{
	int ret;

	ret = arm_smmu_write_reg_sync(smmu, 0, ARM_SMMU_CR0, ARM_SMMU_CR0ACK);
	if (ret)
		dev_err(smmu->dev, "failed to clear cr0\n");

	return ret;
}

struct iommu_group *arm_smmu_device_group(struct device *dev)
{
	struct iommu_group *group;

	/*
	 * We don't support devices sharing stream IDs other than PCI RID
	 * aliases, since the necessary ID-to-device lookup becomes rather
	 * impractical given a potential sparse 32-bit stream ID space.
	 */
	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
}

int arm_smmu_of_xlate(struct device *dev, const struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

void arm_smmu_get_resv_regions(struct device *dev, struct list_head *head)
{
	struct iommu_resv_region *region;
	int prot = IOMMU_WRITE | IOMMU_NOEXEC | IOMMU_MMIO;

	region = iommu_alloc_resv_region(MSI_IOVA_BASE, MSI_IOVA_LENGTH,
					 prot, IOMMU_RESV_SW_MSI, GFP_KERNEL);
	if (!region)
		return;

	list_add_tail(&region->list, head);

	iommu_dma_get_resv_regions(dev, head);
}

int arm_smmu_init_one_queue(struct arm_smmu_device *smmu,
			    struct arm_smmu_queue *q, void __iomem *page,
			    unsigned long prod_off, unsigned long cons_off,
			    size_t dwords, const char *name)
{
	size_t qsz;

	do {
		qsz = PAGE_ALIGN(((1 << q->llq.max_n_shift) * dwords) << 3);
		q->base = dmam_alloc_coherent(smmu->dev, qsz, &q->base_dma,
					      GFP_KERNEL);
		if (q->base)
			break;

		q->llq.max_n_shift--;
	} while (1);

	if (!q->base) {
		dev_err(smmu->dev,
			"failed to allocate queue (0x%zx bytes) for %s\n",
			qsz, name);
		return -ENOMEM;
	}

	if (!WARN_ON(q->base_dma & (qsz - 1))) {
		dev_info(smmu->dev, "allocated %u entries for %s\n",
			 1 << q->llq.max_n_shift, name);
	}

	q->prod_reg	= page + prod_off;
	q->cons_reg	= page + cons_off;
	q->ent_dwords	= dwords;

	q->q_base  = Q_BASE_RWA;
	q->q_base |= q->base_dma & Q_BASE_ADDR_MASK;
	q->q_base |= FIELD_PREP(Q_BASE_LOG2SIZE, q->llq.max_n_shift);

	q->llq.prod = q->llq.cons = 0;
	return 0;
}

/* Stream table manipulation functions */
static int arm_smmu_init_strtab_2lvl(struct arm_smmu_device *smmu)
{
	u32 l1size;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;
	unsigned int last_sid_idx =
		arm_smmu_strtab_l1_idx((1ULL << smmu->sid_bits) - 1);

	/* Calculate the L1 size, capped to the SIDSIZE. */
	cfg->l2.num_l1_ents = min(last_sid_idx + 1, STRTAB_MAX_L1_ENTRIES);
	if (cfg->l2.num_l1_ents <= last_sid_idx)
		dev_warn(smmu->dev,
			 "2-level strtab only covers %u/%u bits of SID\n",
			 ilog2(cfg->l2.num_l1_ents * STRTAB_NUM_L2_STES),
			 smmu->sid_bits);

	l1size = PAGE_ALIGN(cfg->l2.num_l1_ents * sizeof(struct arm_smmu_strtab_l1));
	cfg->l2.l1tab = dmam_alloc_coherent(smmu->dev, l1size, &cfg->l2.l1_dma,
					    GFP_KERNEL);
	if (!cfg->l2.l1tab) {
		dev_err(smmu->dev,
			"failed to allocate l1 stream table (%u bytes)\n",
			l1size);
		return -ENOMEM;
	}

	cfg->l2.l2ptrs = devm_kcalloc(smmu->dev, cfg->l2.num_l1_ents,
				      sizeof(*cfg->l2.l2ptrs), GFP_KERNEL);
	if (!cfg->l2.l2ptrs)
		return -ENOMEM;

	return 0;
}

void arm_smmu_make_abort_ste(struct arm_smmu_ste *target)
{
	memset(target, 0, sizeof(*target));
	target->data[0] = cpu_to_le64(
		STRTAB_STE_0_V |
		FIELD_PREP(STRTAB_STE_0_CFG, STRTAB_STE_0_CFG_ABORT));
}

/*
 * This can safely directly manipulate the STE memory without a sync sequence
 * because the STE table has not been installed in the SMMU yet.
 */
void arm_smmu_init_initial_stes(struct arm_smmu_ste *strtab,
				unsigned int nent)
{
	unsigned int i;

	for (i = 0; i < nent; ++i) {
		arm_smmu_make_abort_ste(strtab);
		strtab++;
	}
}

static int arm_smmu_init_strtab_linear(struct arm_smmu_device *smmu)
{
	u32 size;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;

	size = PAGE_ALIGN((1 << smmu->sid_bits) * sizeof(struct arm_smmu_ste));
	cfg->linear.table = dmam_alloc_coherent(smmu->dev, size,
						&cfg->linear.ste_dma,
						GFP_KERNEL);
	if (!cfg->linear.table) {
		dev_err(smmu->dev,
			"failed to allocate linear stream table (%u bytes)\n",
			size);
		return -ENOMEM;
	}
	cfg->linear.num_ents = 1 << smmu->sid_bits;

	arm_smmu_init_initial_stes(cfg->linear.table, cfg->linear.num_ents);
	return 0;
}

int arm_smmu_init_strtab(struct arm_smmu_device *smmu)
{
	int ret;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB)
		ret = arm_smmu_init_strtab_2lvl(smmu);
	else
		ret = arm_smmu_init_strtab_linear(smmu);
	if (ret)
		return ret;

	ida_init(&smmu->vmid_map);

	return 0;
}

void arm_smmu_write_strtab(struct arm_smmu_device *smmu)
{
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;
	dma_addr_t dma;
	u32 reg;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB) {
		reg = FIELD_PREP(STRTAB_BASE_CFG_FMT,
				 STRTAB_BASE_CFG_FMT_2LVL) |
		      FIELD_PREP(STRTAB_BASE_CFG_LOG2SIZE,
				 ilog2(cfg->l2.num_l1_ents) + STRTAB_SPLIT) |
		      FIELD_PREP(STRTAB_BASE_CFG_SPLIT, STRTAB_SPLIT);
		dma = cfg->l2.l1_dma;
	} else {
		reg = FIELD_PREP(STRTAB_BASE_CFG_FMT,
				 STRTAB_BASE_CFG_FMT_LINEAR) |
		      FIELD_PREP(STRTAB_BASE_CFG_LOG2SIZE, smmu->sid_bits);
		dma = cfg->linear.ste_dma;
	}
	writeq_relaxed((dma & STRTAB_BASE_ADDR_MASK) | STRTAB_BASE_RA,
		       smmu->base + ARM_SMMU_STRTAB_BASE);
	writel_relaxed(reg, smmu->base + ARM_SMMU_STRTAB_BASE_CFG);
}

static struct arm_smmu_option_prop arm_smmu_options[] = {
	{ ARM_SMMU_OPT_SKIP_PREFETCH, "hisilicon,broken-prefetch-cmd" },
	{ ARM_SMMU_OPT_PAGE0_REGS_ONLY, "cavium,cn9900-broken-page1-regspace"},
	{ 0, NULL},
};

static void parse_driver_options(struct arm_smmu_device *smmu)
{
	int i = 0;

	do {
		if (of_property_read_bool(smmu->dev->of_node,
					  arm_smmu_options[i].prop)) {
			smmu->options |= arm_smmu_options[i].opt;
			dev_notice(smmu->dev, "option %s\n",
				   arm_smmu_options[i].prop);
		}
	} while (arm_smmu_options[++i].opt);
}

#ifdef CONFIG_ACPI
#ifdef CONFIG_TEGRA241_CMDQV
static void acpi_smmu_dsdt_probe_tegra241_cmdqv(struct acpi_iort_node *node,
						struct arm_smmu_device *smmu)
{
	const char *uid = kasprintf(GFP_KERNEL, "%u", node->identifier);
	struct acpi_device *adev;

	/* Look for an NVDA200C node whose _UID matches the SMMU node ID */
	adev = acpi_dev_get_first_match_dev("NVDA200C", uid, -1);
	if (adev) {
		/* Tegra241 CMDQV driver is responsible for put_device() */
		smmu->impl_dev = &adev->dev;
		smmu->options |= ARM_SMMU_OPT_TEGRA241_CMDQV;
		dev_info(smmu->dev, "found companion CMDQV device: %s\n",
			 dev_name(smmu->impl_dev));
	}
	kfree(uid);
}
#else
static void acpi_smmu_dsdt_probe_tegra241_cmdqv(struct acpi_iort_node *node,
						struct arm_smmu_device *smmu)
{
}
#endif

static int acpi_smmu_iort_probe_model(struct acpi_iort_node *node,
				      struct arm_smmu_device *smmu)
{
	struct acpi_iort_smmu_v3 *iort_smmu =
		(struct acpi_iort_smmu_v3 *)node->node_data;

	switch (iort_smmu->model) {
	case ACPI_IORT_SMMU_V3_CAVIUM_CN99XX:
		smmu->options |= ARM_SMMU_OPT_PAGE0_REGS_ONLY;
		break;
	case ACPI_IORT_SMMU_V3_HISILICON_HI161X:
		smmu->options |= ARM_SMMU_OPT_SKIP_PREFETCH;
		break;
	case ACPI_IORT_SMMU_V3_GENERIC:
		/*
		 * Tegra241 implementation stores its SMMU options and impl_dev
		 * in DSDT. Thus, go through the ACPI tables unconditionally.
		 */
		acpi_smmu_dsdt_probe_tegra241_cmdqv(node, smmu);
		break;
	}

	dev_notice(smmu->dev, "option mask 0x%x\n", smmu->options);
	return 0;
}

static int arm_smmu_device_acpi_probe(struct platform_device *pdev,
				      struct arm_smmu_device *smmu)
{
	struct acpi_iort_smmu_v3 *iort_smmu;
	struct device *dev = smmu->dev;
	struct acpi_iort_node *node;

	node = *(struct acpi_iort_node **)dev_get_platdata(dev);

	/* Retrieve SMMUv3 specific data */
	iort_smmu = (struct acpi_iort_smmu_v3 *)node->node_data;

	if (iort_smmu->flags & ACPI_IORT_SMMU_V3_COHACC_OVERRIDE)
		smmu->features |= ARM_SMMU_FEAT_COHERENCY;

	switch (FIELD_GET(ACPI_IORT_SMMU_V3_HTTU_OVERRIDE, iort_smmu->flags)) {
	case IDR0_HTTU_ACCESS_DIRTY:
		smmu->features |= ARM_SMMU_FEAT_HD;
		fallthrough;
	case IDR0_HTTU_ACCESS:
		smmu->features |= ARM_SMMU_FEAT_HA;
	}

	return acpi_smmu_iort_probe_model(node, smmu);
}
#else
static inline int arm_smmu_device_acpi_probe(struct platform_device *pdev,
					     struct arm_smmu_device *smmu)
{
	return -ENODEV;
}
#endif

static int arm_smmu_device_dt_probe(struct platform_device *pdev,
				    struct arm_smmu_device *smmu)
{
	struct device *dev = &pdev->dev;
	u32 cells;
	int ret = -EINVAL;

	if (of_property_read_u32(dev->of_node, "#iommu-cells", &cells))
		dev_err(dev, "missing #iommu-cells property\n");
	else if (cells != 1)
		dev_err(dev, "invalid #iommu-cells value (%d)\n", cells);
	else
		ret = 0;

	parse_driver_options(smmu);

	if (of_dma_is_coherent(dev->of_node))
		smmu->features |= ARM_SMMU_FEAT_COHERENCY;

	return ret;
}

int arm_smmu_fw_probe(struct platform_device *pdev,
		      struct arm_smmu_device *smmu)
{
	if (smmu->dev->of_node)
		return arm_smmu_device_dt_probe(pdev, smmu);
	else
		return arm_smmu_device_acpi_probe(pdev, smmu);
}

int arm_smmu_register_iommu(struct arm_smmu_device *smmu,
			    const struct iommu_ops *ops, phys_addr_t ioaddr)
{
	int ret;
	struct device *dev = smmu->dev;

	ret = iommu_device_sysfs_add(&smmu->iommu, dev, NULL,
				     "smmu3.%pa", &ioaddr);
	if (ret)
		return ret;

	ret = iommu_device_register(&smmu->iommu, ops, dev);
	if (ret) {
		dev_err(dev, "Failed to register iommu\n");
		iommu_device_sysfs_remove(&smmu->iommu);
		return ret;
	}

	return 0;
}

void arm_smmu_unregister_iommu(struct arm_smmu_device *smmu)
{
	iommu_device_unregister(&smmu->iommu);
	iommu_device_sysfs_remove(&smmu->iommu);
}

static void arm_smmu_free_msis(void *data)
{
	struct device *dev = data;

	platform_device_msi_free_irqs_all(dev);
}

static void arm_smmu_write_msi_msg(struct msi_desc *desc, struct msi_msg *msg)
{
	phys_addr_t doorbell;
	struct device *dev = msi_desc_to_dev(desc);
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	phys_addr_t *cfg = arm_smmu_msi_cfg[desc->msi_index];

	doorbell = (((u64)msg->address_hi) << 32) | msg->address_lo;
	doorbell &= MSI_CFG0_ADDR_MASK;

	writeq_relaxed(doorbell, smmu->base + cfg[0]);
	writel_relaxed(msg->data, smmu->base + cfg[1]);
	writel_relaxed(ARM_SMMU_MEMATTR_DEVICE_nGnRE, smmu->base + cfg[2]);
}

static void arm_smmu_setup_msis(struct arm_smmu_device *smmu)
{
	int ret, nvec = ARM_SMMU_MAX_MSIS;
	struct device *dev = smmu->dev;

	/* Clear the MSI address regs */
	writeq_relaxed(0, smmu->base + ARM_SMMU_GERROR_IRQ_CFG0);
	writeq_relaxed(0, smmu->base + ARM_SMMU_EVTQ_IRQ_CFG0);

	if (smmu->features & ARM_SMMU_FEAT_PRI)
		writeq_relaxed(0, smmu->base + ARM_SMMU_PRIQ_IRQ_CFG0);
	else
		nvec--;

	if (!(smmu->features & ARM_SMMU_FEAT_MSI))
		return;

	if (!dev->msi.domain) {
		dev_info(smmu->dev, "msi_domain absent - falling back to wired irqs\n");
		return;
	}

	/* Allocate MSIs for evtq, gerror and priq. Ignore cmdq */
	ret = platform_device_msi_init_and_alloc_irqs(dev, nvec, arm_smmu_write_msi_msg);
	if (ret) {
		dev_warn(dev, "failed to allocate MSIs - falling back to wired irqs\n");
		return;
	}

	smmu->evtq.q.irq = msi_get_virq(dev, EVTQ_MSI_INDEX);
	smmu->gerr_irq = msi_get_virq(dev, GERROR_MSI_INDEX);
	smmu->priq.q.irq = msi_get_virq(dev, PRIQ_MSI_INDEX);

	/* Add callback to free MSIs on teardown */
	devm_add_action_or_reset(dev, arm_smmu_free_msis, dev);
}

static void arm_smmu_setup_unique_irqs(struct arm_smmu_device *smmu,
				       irqreturn_t evtqirq(int irq, void *dev),
				       irqreturn_t gerrorirq(int irq, void *dev),
				       irqreturn_t priirq(int irq, void *dev))
{
	int irq, ret;

	arm_smmu_setup_msis(smmu);

	/* Request interrupt lines */
	irq = smmu->evtq.q.irq;
	if (irq) {
		ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
						evtqirq,
						IRQF_ONESHOT,
						"arm-smmu-v3-evtq", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable evtq irq\n");
	} else {
		dev_warn(smmu->dev, "no evtq irq - events will not be reported!\n");
	}

	irq = smmu->gerr_irq;
	if (irq) {
		ret = devm_request_irq(smmu->dev, irq, gerrorirq,
				       0, "arm-smmu-v3-gerror", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable gerror irq\n");
	} else {
		dev_warn(smmu->dev, "no gerr irq - errors will not be reported!\n");
	}

	if (smmu->features & ARM_SMMU_FEAT_PRI) {
		irq = smmu->priq.q.irq;
		if (irq) {
			ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
							priirq,
							IRQF_ONESHOT,
							"arm-smmu-v3-priq",
							smmu);
			if (ret < 0)
				dev_warn(smmu->dev,
					 "failed to enable priq irq\n");
		} else {
			dev_warn(smmu->dev, "no priq irq - PRI will be broken\n");
		}
	}
}

int arm_smmu_setup_irqs(struct arm_smmu_device *smmu,
			irqreturn_t combined_thrd(int irq, void *dev),
			irqreturn_t combined_irq(int irq, void *dev),
			irqreturn_t evtqirq(int irq, void *dev),
			irqreturn_t gerrorirq(int irq, void *dev),
			irqreturn_t priirq(int irq, void *dev))
{
	int ret, irq;
	u32 irqen_flags = IRQ_CTRL_EVTQ_IRQEN | IRQ_CTRL_GERROR_IRQEN;

	/* Disable IRQs first */
	ret = arm_smmu_write_reg_sync(smmu, 0, ARM_SMMU_IRQ_CTRL,
				      ARM_SMMU_IRQ_CTRLACK);
	if (ret) {
		dev_err(smmu->dev, "failed to disable irqs\n");
		return ret;
	}

	irq = smmu->combined_irq;
	if (irq) {
		/*
		 * Cavium ThunderX2 implementation doesn't support unique irq
		 * lines. Use a single irq line for all the SMMUv3 interrupts.
		 */
		ret = devm_request_threaded_irq(smmu->dev, irq,
					combined_irq,
					combined_thrd,
					IRQF_ONESHOT,
					"arm-smmu-v3-combined-irq", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable combined irq\n");
	} else
		arm_smmu_setup_unique_irqs(smmu, evtqirq,
					   gerrorirq, priirq);

	if (smmu->features & ARM_SMMU_FEAT_PRI)
		irqen_flags |= IRQ_CTRL_PRIQ_IRQEN;

	/* Enable interrupt generation on the SMMU */
	ret = arm_smmu_write_reg_sync(smmu, irqen_flags,
				      ARM_SMMU_IRQ_CTRL, ARM_SMMU_IRQ_CTRLACK);
	if (ret)
		dev_warn(smmu->dev, "failed to enable irqs\n");

	return 0;
}

void arm_smmu_probe_irq(struct platform_device *pdev,
			struct arm_smmu_device *smmu)
{
	int irq;

	irq = platform_get_irq_byname_optional(pdev, "combined");
	if (irq > 0)
		smmu->combined_irq = irq;
	else {
		irq = platform_get_irq_byname_optional(pdev, "eventq");
		if (irq > 0)
			smmu->evtq.q.irq = irq;

		irq = platform_get_irq_byname_optional(pdev, "priq");
		if (irq > 0)
			smmu->priq.q.irq = irq;

		irq = platform_get_irq_byname_optional(pdev, "gerror");
		if (irq > 0)
			smmu->gerr_irq = irq;
	}
}

static int arm_smmu_init_l2_strtab(struct arm_smmu_device *smmu, u32 sid)
{
	dma_addr_t l2ptr_dma;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;
	struct arm_smmu_strtab_l2 **l2table;

	l2table = &cfg->l2.l2ptrs[arm_smmu_strtab_l1_idx(sid)];
	if (*l2table)
		return 0;

	*l2table = dmam_alloc_coherent(smmu->dev, sizeof(**l2table),
				       &l2ptr_dma, GFP_KERNEL);
	if (!*l2table) {
		dev_err(smmu->dev,
			"failed to allocate l2 stream table for SID %u\n",
			sid);
		return -ENOMEM;
	}

	arm_smmu_init_initial_stes((*l2table)->stes,
				   ARRAY_SIZE((*l2table)->stes));
	arm_smmu_write_strtab_l1_desc(&cfg->l2.l1tab[arm_smmu_strtab_l1_idx(sid)],
				      l2ptr_dma);
	return 0;
}

static bool arm_smmu_sid_in_range(struct arm_smmu_device *smmu, u32 sid)
{
	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB)
		return arm_smmu_strtab_l1_idx(sid) < smmu->strtab_cfg.l2.num_l1_ents;
	return sid < smmu->strtab_cfg.linear.num_ents;
}

int arm_smmu_init_sid_strtab(struct arm_smmu_device *smmu, u32 sid)
{
	/* Check the SIDs are in range of the SMMU and our stream table */
	if (!arm_smmu_sid_in_range(smmu, sid))
		return -ERANGE;

	/* Ensure l2 strtab is initialised */
	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB)
		return arm_smmu_init_l2_strtab(smmu, sid);

	return 0;
}

irqreturn_t arm_smmu_gerror_common(int irq, void *dev,
				   void(*cmd_err)(struct arm_smmu_device *smmu))
{
	u32 gerror, gerrorn, active;
	struct arm_smmu_device *smmu = dev;

	gerror = readl_relaxed(smmu->base + ARM_SMMU_GERROR);
	gerrorn = readl_relaxed(smmu->base + ARM_SMMU_GERRORN);

	active = gerror ^ gerrorn;
	if (!(active & GERROR_ERR_MASK))
		return IRQ_NONE; /* No errors pending */

	dev_warn(smmu->dev,
		 "unexpected global error reported (0x%08x), this could be serious\n",
		 active);

	if (active & GERROR_SFM_ERR) {
		dev_err(smmu->dev, "device has entered Service Failure Mode!\n");
		arm_smmu_device_disable(smmu);
	}

	if (active & GERROR_MSI_GERROR_ABT_ERR)
		dev_warn(smmu->dev, "GERROR MSI write aborted\n");

	if (active & GERROR_MSI_PRIQ_ABT_ERR)
		dev_warn(smmu->dev, "PRIQ MSI write aborted\n");

	if (active & GERROR_MSI_EVTQ_ABT_ERR)
		dev_warn(smmu->dev, "EVTQ MSI write aborted\n");

	if (active & GERROR_MSI_CMDQ_ABT_ERR)
		dev_warn(smmu->dev, "CMDQ MSI write aborted\n");

	if (active & GERROR_PRIQ_ABT_ERR)
		dev_err(smmu->dev, "PRIQ write aborted -- events may have been lost\n");

	if (active & GERROR_EVTQ_ABT_ERR)
		dev_err(smmu->dev, "EVTQ write aborted -- events may have been lost\n");

	if (active & GERROR_CMDQ_ERR)
		cmd_err(smmu);

	writel(gerror, smmu->base + ARM_SMMU_GERRORN);
	return IRQ_HANDLED;
}


static void arm_smmu_dump_raw_event(struct arm_smmu_device *smmu, u64 *raw,
				    struct arm_smmu_event *event)
{
	int i;

	dev_err(smmu->dev, "event 0x%02x received:\n", event->id);

	for (i = 0; i < EVTQ_ENT_DWORDS; ++i)
		dev_err(smmu->dev, "\t0x%016llx\n", raw[i]);
}

#define ARM_SMMU_EVT_KNOWN(e)	((e)->id < ARRAY_SIZE(event_str) && event_str[(e)->id])
#define ARM_SMMU_LOG_EVT_STR(e) ARM_SMMU_EVT_KNOWN(e) ? event_str[(e)->id] : "UNKNOWN"
#define ARM_SMMU_LOG_CLIENT(e)	(e)->dev ? dev_name((e)->dev) : "(unassigned sid)"

static void arm_smmu_dump_event(struct arm_smmu_device *smmu, u64 *raw,
				struct arm_smmu_event *evt,
				struct ratelimit_state *rs)
{
	if (!__ratelimit(rs))
		return;

	arm_smmu_dump_raw_event(smmu, raw, evt);

	switch (evt->id) {
	case EVT_ID_TRANSLATION_FAULT:
	case EVT_ID_ADDR_SIZE_FAULT:
	case EVT_ID_ACCESS_FAULT:
	case EVT_ID_PERMISSION_FAULT:
		dev_err(smmu->dev, "event: %s client: %s sid: %#x ssid: %#x iova: %#llx ipa: %#llx",
			ARM_SMMU_LOG_EVT_STR(evt), ARM_SMMU_LOG_CLIENT(evt),
			evt->sid, evt->ssid, evt->iova, evt->ipa);

		dev_err(smmu->dev, "%s %s %s %s \"%s\"%s%s stag: %#x",
			evt->privileged ? "priv" : "unpriv",
			evt->instruction ? "inst" : "data",
			str_read_write(evt->read),
			evt->s2 ? "s2" : "s1", event_class_str[evt->class],
			evt->class_tt ? (evt->ttrnw ? " ttd_read" : " ttd_write") : "",
			evt->stall ? " stall" : "", evt->stag);

		break;

	case EVT_ID_STE_FETCH_FAULT:
	case EVT_ID_CD_FETCH_FAULT:
	case EVT_ID_VMS_FETCH_FAULT:
		dev_err(smmu->dev, "event: %s client: %s sid: %#x ssid: %#x fetch_addr: %#llx",
			ARM_SMMU_LOG_EVT_STR(evt), ARM_SMMU_LOG_CLIENT(evt),
			evt->sid, evt->ssid, evt->fetch_addr);

		break;

	default:
		dev_err(smmu->dev, "event: %s client: %s sid: %#x ssid: %#x",
			ARM_SMMU_LOG_EVT_STR(evt), ARM_SMMU_LOG_CLIENT(evt),
			evt->sid, evt->ssid);
	}
}

static int arm_smmu_streams_cmp_key(const void *lhs, const struct rb_node *rhs)
{
	struct arm_smmu_stream *stream_rhs =
		rb_entry(rhs, struct arm_smmu_stream, node);
	const u32 *sid_lhs = lhs;

	if (*sid_lhs < stream_rhs->id)
		return -1;
	if (*sid_lhs > stream_rhs->id)
		return 1;
	return 0;
}

static int arm_smmu_streams_cmp_node(struct rb_node *lhs,
				     const struct rb_node *rhs)
{
	return arm_smmu_streams_cmp_key(
		&rb_entry(lhs, struct arm_smmu_stream, node)->id, rhs);
}

struct arm_smmu_master *
arm_smmu_find_master(struct arm_smmu_device *smmu, u32 sid)
{
	struct rb_node *node;

	lockdep_assert_held(&smmu->streams_mutex);

	node = rb_find(&sid, &smmu->streams, arm_smmu_streams_cmp_key);
	if (!node)
		return NULL;
	return rb_entry(node, struct arm_smmu_stream, node)->master;
}

int arm_smmu_insert_master(struct arm_smmu_device *smmu,
			   struct arm_smmu_master *master,
			   bool init_ste)
{
	int i;
	int ret = 0;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(master->dev);

	master->streams = kcalloc(fwspec->num_ids, sizeof(*master->streams),
				  GFP_KERNEL);
	if (!master->streams)
		return -ENOMEM;
	master->num_streams = fwspec->num_ids;

	mutex_lock(&smmu->streams_mutex);
	for (i = 0; i < fwspec->num_ids; i++) {
		struct arm_smmu_stream *new_stream = &master->streams[i];
		struct rb_node *existing;
		u32 sid = fwspec->ids[i];

		new_stream->id = sid;
		new_stream->master = master;

		if (init_ste) {
			ret = arm_smmu_init_sid_strtab(smmu, sid);
			if (ret)
				break;
		}

		/* Insert into SID tree */
		existing = rb_find_add(&new_stream->node, &smmu->streams,
				       arm_smmu_streams_cmp_node);
		if (existing) {
			struct arm_smmu_master *existing_master =
				rb_entry(existing, struct arm_smmu_stream, node)
					->master;

			/* Bridged PCI devices may end up with duplicated IDs */
			if (existing_master == master)
				continue;

			dev_warn(master->dev,
				 "Aliasing StreamID 0x%x (from %s) unsupported, expect DMA to be broken\n",
				 sid, dev_name(existing_master->dev));
			ret = -ENODEV;
			break;
		}
	}

	if (ret) {
		for (i--; i >= 0; i--)
			rb_erase(&master->streams[i].node, &smmu->streams);
		kfree(master->streams);
	}
	mutex_unlock(&smmu->streams_mutex);

	return ret;
}

void arm_smmu_remove_master(struct arm_smmu_master *master)
{
	int i;
	struct arm_smmu_device *smmu = master->smmu;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(master->dev);

	if (!smmu || !master->streams)
		return;

	mutex_lock(&smmu->streams_mutex);
	for (i = 0; i < fwspec->num_ids; i++)
		rb_erase(&master->streams[i].node, &smmu->streams);
	mutex_unlock(&smmu->streams_mutex);

	kfree(master->streams);
}

/* IRQ and event handlers */
static void arm_smmu_decode_event(struct arm_smmu_device *smmu, u64 *raw,
				  struct arm_smmu_event *event)
{
	struct arm_smmu_master *master;

	event->id = FIELD_GET(EVTQ_0_ID, raw[0]);
	event->sid = FIELD_GET(EVTQ_0_SID, raw[0]);
	event->ssv = FIELD_GET(EVTQ_0_SSV, raw[0]);
	event->ssid = event->ssv ? FIELD_GET(EVTQ_0_SSID, raw[0]) : IOMMU_NO_PASID;
	event->privileged = FIELD_GET(EVTQ_1_PnU, raw[1]);
	event->instruction = FIELD_GET(EVTQ_1_InD, raw[1]);
	event->s2 = FIELD_GET(EVTQ_1_S2, raw[1]);
	event->read = FIELD_GET(EVTQ_1_RnW, raw[1]);
	event->stag = FIELD_GET(EVTQ_1_STAG, raw[1]);
	event->stall = FIELD_GET(EVTQ_1_STALL, raw[1]);
	event->class = FIELD_GET(EVTQ_1_CLASS, raw[1]);
	event->iova = FIELD_GET(EVTQ_2_ADDR, raw[2]);
	event->ipa = raw[3] & EVTQ_3_IPA;
	event->fetch_addr = raw[3] & EVTQ_3_FETCH_ADDR;
	event->ttrnw = FIELD_GET(EVTQ_1_TT_READ, raw[1]);
	event->class_tt = false;
	event->dev = NULL;

	if (event->id == EVT_ID_PERMISSION_FAULT)
		event->class_tt = (event->class == EVTQ_1_CLASS_TT);

	mutex_lock(&smmu->streams_mutex);
	master = arm_smmu_find_master(smmu, event->sid);
	if (master)
		event->dev = get_device(master->dev);
	mutex_unlock(&smmu->streams_mutex);
}

irqreturn_t arm_smmu_evtq_common(int irq, void *dev,
				 int handle_event(struct arm_smmu_device *smmu,
						  u64 *evt, struct arm_smmu_event *event))
{
	u64 evt[EVTQ_ENT_DWORDS];
	struct arm_smmu_event event = {0};
	struct arm_smmu_device *smmu = dev;
	struct arm_smmu_queue *q = &smmu->evtq.q;
	struct arm_smmu_ll_queue *llq = &q->llq;
	static DEFINE_RATELIMIT_STATE(rs, DEFAULT_RATELIMIT_INTERVAL,
				      DEFAULT_RATELIMIT_BURST);

	do {
		while (!queue_remove_raw(q, evt)) {
			arm_smmu_decode_event(smmu, evt, &event);
			if (handle_event(smmu, evt, &event))
				arm_smmu_dump_event(smmu, evt, &event, &rs);

			put_device(event.dev);
			cond_resched();
		}

		/*
		 * Not much we can do on overflow, so scream and pretend we're
		 * trying harder.
		 */
		if (queue_sync_prod_in(q) == -EOVERFLOW)
			dev_err(smmu->dev, "EVTQ overflow detected -- events lost\n");
	} while (!queue_empty(llq));

	/* Sync our overflow flag, as we believe we're up to speed */
	queue_sync_cons_ovf(q);
	return IRQ_HANDLED;
}

int arm_smmu_handle_event(struct arm_smmu_device *smmu, u64 *evt,
			  struct arm_smmu_event *event)
{
	int ret = 0;
	u32 perm = 0;
	struct arm_smmu_master *master;
	struct iopf_fault fault_evt = { };
	struct iommu_fault *flt = &fault_evt.fault;

	switch (event->id) {
	case EVT_ID_BAD_STE_CONFIG:
	case EVT_ID_STREAM_DISABLED_FAULT:
	case EVT_ID_BAD_SUBSTREAMID_CONFIG:
	case EVT_ID_BAD_CD_CONFIG:
	case EVT_ID_TRANSLATION_FAULT:
	case EVT_ID_ADDR_SIZE_FAULT:
	case EVT_ID_ACCESS_FAULT:
	case EVT_ID_PERMISSION_FAULT:
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (event->stall) {
		if (event->read)
			perm |= IOMMU_FAULT_PERM_READ;
		else
			perm |= IOMMU_FAULT_PERM_WRITE;

		if (event->instruction)
			perm |= IOMMU_FAULT_PERM_EXEC;

		if (event->privileged)
			perm |= IOMMU_FAULT_PERM_PRIV;

		flt->type = IOMMU_FAULT_PAGE_REQ;
		flt->prm = (struct iommu_fault_page_request){
			.flags = IOMMU_FAULT_PAGE_REQUEST_LAST_PAGE,
			.grpid = event->stag,
			.perm = perm,
			.addr = event->iova,
		};

		if (event->ssv) {
			flt->prm.flags |= IOMMU_FAULT_PAGE_REQUEST_PASID_VALID;
			flt->prm.pasid = event->ssid;
		}
	}

	mutex_lock(&smmu->streams_mutex);
	master = arm_smmu_find_master(smmu, event->sid);
	if (!master) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (event->stall)
		ret = iommu_report_device_fault(master->dev, &fault_evt);
	else if (master->vmaster && !event->s2)
		ret = arm_vmaster_report_event(master->vmaster, evt);
	else
		ret = -EOPNOTSUPP; /* Unhandled events should be pinned */
out_unlock:
	mutex_unlock(&smmu->streams_mutex);
	return ret;
}
