/* * CAAM control-plane driver backend
 * Controller-level driver, kernel property detection, initialization
 *
 * Copyright 2008-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "jr.h"
#include "desc_constr.h"
#include "error.h"
#include "ctrl.h"
#include "sm.h"

bool caam_little_end;
EXPORT_SYMBOL(caam_little_end);

/*
 * i.MX targets tend to have clock control subsystems that can
 * enable/disable clocking to our device.
 */
#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_IMX
static inline struct clk *caam_drv_identify_clk(struct device *dev,
						char *clk_name)
{
	return devm_clk_get(dev, clk_name);
}
#else
static inline struct clk *caam_drv_identify_clk(struct device *dev,
						char *clk_name)
{
	return NULL;
}
#endif

static int caam_remove(struct platform_device *pdev)
{
	struct device *ctrldev;
	struct caam_drv_private *ctrlpriv;
	struct caam_ctrl __iomem *ctrl;
	int ring;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;

	/* Remove platform devices for JobRs */
	for (ring = 0; ring < ctrlpriv->total_jobrs; ring++) {
		if (ctrlpriv->jrpdev[ring])
			of_device_unregister(ctrlpriv->jrpdev[ring]);
	}

	/* Shut down debug views */
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(ctrlpriv->dfs_root);
#endif

	/* Unmap controller region */
	iounmap(ctrl);

	/* shut clocks off before finalizing shutdown */
	clk_disable_unprepare(ctrlpriv->caam_ipg);
	clk_disable_unprepare(ctrlpriv->caam_mem);
	clk_disable_unprepare(ctrlpriv->caam_aclk);
	clk_disable_unprepare(ctrlpriv->caam_emi_slow);

	return 0;
}

static void detect_era(struct caam_drv_private *ctrlpriv)
{
	int ret, i;
	u32 caam_era;
	u32 caam_id_ms;
	char *era_source;
	struct device_node *caam_node;
	struct sec_vid sec_vid;
	static const struct {
		u16 ip_id;
		u8 maj_rev;
		u8 era;
	} caam_eras[] = {
		{0x0A10, 1, 1},
		{0x0A10, 2, 2},
		{0x0A12, 1, 3},
		{0x0A14, 1, 3},
		{0x0A10, 3, 4},
		{0x0A11, 1, 4},
		{0x0A14, 2, 4},
		{0x0A16, 1, 4},
		{0x0A18, 1, 4},
		{0x0A11, 2, 5},
		{0x0A12, 2, 5},
		{0x0A13, 1, 5},
		{0x0A1C, 1, 5},
		{0x0A12, 4, 6},
		{0x0A13, 2, 6},
		{0x0A16, 2, 6},
		{0x0A17, 1, 6},
		{0x0A18, 2, 6},
		{0x0A1A, 1, 6},
		{0x0A1C, 2, 6},
		{0x0A14, 3, 7},
		{0x0A10, 4, 8},
		{0x0A11, 3, 8},
		{0x0A11, 4, 8},
		{0x0A12, 5, 8},
		{0x0A16, 3, 8},
	};

	/* If the user or bootloader has set the property we'll use that */
	caam_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	ret = of_property_read_u32(caam_node, "fsl,sec-era", &caam_era);
	of_node_put(caam_node);

	if (!ret) {
		era_source = "device tree";
		goto era_found;
	}

	i = ctrlpriv->first_jr_index;
	/* If ccbvid has the era, use that (era 6 and onwards) */
	if (ctrlpriv->has_seco)
		caam_era = rd_reg32(&ctrlpriv->jr[i]->perfmon.ccb_id);
	else
		caam_era = rd_reg32(&ctrlpriv->ctrl->perfmon.ccb_id);

	caam_era = caam_era >> CCB_VID_ERA_SHIFT & CCB_VID_ERA_MASK;
	if (caam_era) {
		era_source = "CCBVID";
		goto era_found;
	}

	/* If we can match caamvid to known versions, use that */
	if (ctrlpriv->has_seco)
		caam_id_ms = rd_reg32(&ctrlpriv->jr[i]->perfmon.caam_id_ms);
	else
		caam_id_ms = rd_reg32(&ctrlpriv->ctrl->perfmon.caam_id_ms);
	sec_vid.ip_id = caam_id_ms >> SEC_VID_IPID_SHIFT;
	sec_vid.maj_rev = (caam_id_ms & SEC_VID_MAJ_MASK) >> SEC_VID_MAJ_SHIFT;

	for (i = 0; i < ARRAY_SIZE(caam_eras); i++)
		if (caam_eras[i].ip_id == sec_vid.ip_id &&
		    caam_eras[i].maj_rev == sec_vid.maj_rev) {
			caam_era = caam_eras[i].era;
			era_source = "CAAMVID";
			goto era_found;
		}

	ctrlpriv->era = -ENOTSUPP;
	dev_info(&ctrlpriv->pdev->dev, "ERA undetermined!.\n");
	return;

era_found:
	ctrlpriv->era = caam_era;
	dev_info(&ctrlpriv->pdev->dev, "ERA source: %s.\n", era_source);
}

static void handle_imx6_err005766(struct caam_drv_private *ctrlpriv)
{
	/*
	 * ERRATA:  mx6 devices have an issue wherein AXI bus transactions
	 * may not occur in the correct order. This isn't a problem running
	 * single descriptors, but can be if running multiple concurrent
	 * descriptors. Reworking the driver to throttle to single requests
	 * is impractical, thus the workaround is to limit the AXI pipeline
	 * to a depth of 1 (from it's default of 4) to preclude this situation
	 * from occurring.
	 */

	u32 mcr_val;

	if (ctrlpriv->era != IMX_ERR005766_ERA)
		return;

	if (of_machine_is_compatible("fsl,imx6q") ||
	    of_machine_is_compatible("fsl,imx6dl") ||
	    of_machine_is_compatible("fsl,imx6qp")) {
		dev_info(&ctrlpriv->pdev->dev,
			 "AXI pipeline throttling enabled.\n");
		mcr_val = rd_reg32(&ctrlpriv->ctrl->mcr);
		wr_reg32(&ctrlpriv->ctrl->mcr,
			 (mcr_val & ~(MCFGR_AXIPIPE_MASK)) |
			 ((1 << MCFGR_AXIPIPE_SHIFT) & MCFGR_AXIPIPE_MASK));
	}
}

#ifdef CONFIG_DEBUG_FS
static int caam_debugfs_u64_get(void *data, u64 *val)
{
	*val = caam64_to_cpu(*(u64 *)data);
	return 0;
}

static int caam_debugfs_u32_get(void *data, u64 *val)
{
	*val = caam32_to_cpu(*(u32 *)data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(caam_fops_u32_ro, caam_debugfs_u32_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(caam_fops_u64_ro, caam_debugfs_u64_get, NULL, "%llu\n");
#endif

static void init_debugfs(struct caam_drv_private *ctrlpriv)
{
#ifdef CONFIG_DEBUG_FS
	struct caam_perfmon *perfmon;

	/*
	 * FIXME: needs better naming distinction, as some amalgamation of
	 * "caam" and nprop->full_name. The OF name isn't distinctive,
	 * but does separate instances
	 */
	perfmon = (struct caam_perfmon __force *)&ctrlpriv->ctrl->perfmon;

	ctrlpriv->dfs_root = debugfs_create_dir(dev_name(ctrlpriv->dev), NULL);
	ctrlpriv->ctl = debugfs_create_dir("ctl", ctrlpriv->dfs_root);

	/* Controller-level - performance monitor counters */

	ctrlpriv->ctl_rq_dequeued =
		debugfs_create_file("rq_dequeued",
				    0444,
				    ctrlpriv->ctl, &perfmon->req_dequeued,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ob_enc_req =
		debugfs_create_file("ob_rq_encrypted",
				    0444,
				    ctrlpriv->ctl, &perfmon->ob_enc_req,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ib_dec_req =
		debugfs_create_file("ib_rq_decrypted",
				    0444,
				    ctrlpriv->ctl, &perfmon->ib_dec_req,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ob_enc_bytes =
		debugfs_create_file("ob_bytes_encrypted",
				    0444,
				    ctrlpriv->ctl, &perfmon->ob_enc_bytes,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ob_prot_bytes =
		debugfs_create_file("ob_bytes_protected",
				    0444,
				    ctrlpriv->ctl, &perfmon->ob_prot_bytes,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ib_dec_bytes =
		debugfs_create_file("ib_bytes_decrypted",
				    0444,
				    ctrlpriv->ctl, &perfmon->ib_dec_bytes,
				    &caam_fops_u64_ro);
	ctrlpriv->ctl_ib_valid_bytes =
		debugfs_create_file("ib_bytes_validated",
				    0444,
				    ctrlpriv->ctl, &perfmon->ib_valid_bytes,
				    &caam_fops_u64_ro);

	/* Controller level - global status values */
	ctrlpriv->ctl_faultaddr =
		debugfs_create_file("fault_addr",
				    0444,
				    ctrlpriv->ctl, &perfmon->faultaddr,
				    &caam_fops_u32_ro);
	ctrlpriv->ctl_faultdetail =
		debugfs_create_file("fault_detail",
				    0444,
				    ctrlpriv->ctl, &perfmon->faultdetail,
				    &caam_fops_u32_ro);
	ctrlpriv->ctl_faultstatus =
		debugfs_create_file("fault_status",
				    0444,
				    ctrlpriv->ctl, &perfmon->status,
				    &caam_fops_u32_ro);

	/* Internal covering keys (useful in non-secure mode only) */
	ctrlpriv->ctl_kek_wrap.data = &ctrlpriv->ctrl->kek[0];
	ctrlpriv->ctl_kek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_kek = debugfs_create_blob("kek",
						0444,
						ctrlpriv->ctl,
						&ctrlpriv->ctl_kek_wrap);

	ctrlpriv->ctl_tkek_wrap.data = &ctrlpriv->ctrl->tkek[0];
	ctrlpriv->ctl_tkek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tkek = debugfs_create_blob("tkek",
						 0444,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tkek_wrap);

	ctrlpriv->ctl_tdsk_wrap.data = &ctrlpriv->ctrl->tdsk[0];
	ctrlpriv->ctl_tdsk_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tdsk = debugfs_create_blob("tdsk",
						 0444,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tdsk_wrap);
#endif
}

static int init_clocks(struct caam_drv_private *ctrlpriv)
{
	struct clk *clk;
	struct device *dev = ctrlpriv->dev;
	int ret = 0;

	/* Enable clocking */
	clk = caam_drv_identify_clk(dev, "ipg");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "can't identify CAAM ipg clk: %d\n", ret);
		return ret;
	}
	ctrlpriv->caam_ipg = clk;

	ret = clk_prepare_enable(ctrlpriv->caam_ipg);
	if (ret < 0) {
		dev_err(dev, "can't enable CAAM ipg clock: %d\n", ret);
		return ret;
	}

	clk = caam_drv_identify_clk(dev, "aclk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "can't identify CAAM aclk clk: %d\n", ret);
		return ret;
	}
	ctrlpriv->caam_aclk = clk;

	ret = clk_prepare_enable(ctrlpriv->caam_aclk);
	if (ret < 0) {
		dev_err(dev, "can't enable CAAM aclk clock: %d\n", ret);
		return ret;
	}

	if (!(of_find_compatible_node(NULL, NULL, "fsl,imx7d-caam"))) {

		clk = caam_drv_identify_clk(dev, "mem");
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(dev, "can't identify CAAM mem clk: %d\n", ret);
			return ret;
		}
		ctrlpriv->caam_mem = clk;

		ret = clk_prepare_enable(ctrlpriv->caam_mem);
		if (ret < 0) {
			dev_err(dev, "can't enable CAAM secure mem clock: %d\n",
				ret);
			return ret;
		}

		if (!(of_find_compatible_node(NULL, NULL, "fsl,imx6ul-caam"))) {
			clk = caam_drv_identify_clk(dev, "emi_slow");
			if (IS_ERR(clk)) {
				ret = PTR_ERR(clk);
				dev_err(dev,
					"can't identify CAAM emi_slow clk: %d\n",
					ret);
				return ret;
			}
			ctrlpriv->caam_emi_slow = clk;

			ret = clk_prepare_enable(ctrlpriv->caam_emi_slow);
			if (ret < 0) {
				dev_err(dev,
					"can't enable CAAM emi slow clock: %d\n",
					ret);
				return ret;
			}
		}
	}

	return ret;
}

static void check_virt(struct caam_drv_private *ctrlpriv, u32 comp_params)
{
	/*
	 *  Read the Compile Time parameters and SCFGR to determine
	 * if Virtualization is enabled for this platform
	 */
	u32 scfgr;

	scfgr = rd_reg32(&ctrlpriv->ctrl->scfgr);

	ctrlpriv->virt_en = 0;
	if (comp_params & CTPR_MS_VIRT_EN_INCL) {
		/* VIRT_EN_INCL = 1 & VIRT_EN_POR = 1 or
		 * VIRT_EN_INCL = 1 & VIRT_EN_POR = 0 & SCFGR_VIRT_EN = 1
		 */
		if ((comp_params & CTPR_MS_VIRT_EN_POR) ||
		    (!(comp_params & CTPR_MS_VIRT_EN_POR) &&
		       (scfgr & SCFGR_VIRT_EN)))
			ctrlpriv->virt_en = 1;
	} else {
		/* VIRT_EN_INCL = 0 && VIRT_EN_POR_VALUE = 1 */
		if (comp_params & CTPR_MS_VIRT_EN_POR)
			ctrlpriv->virt_en = 1;
	}

	if (ctrlpriv->virt_en == 1)
		clrsetbits_32(&ctrlpriv->ctrl->jrstart, 0, JRSTART_JR0_START |
			      JRSTART_JR1_START | JRSTART_JR2_START |
			      JRSTART_JR3_START);
}

static int enable_jobrings(struct caam_drv_private *ctrlpriv, int block_offset)
{
	int ring, index;
	int rspec = 0;
	struct device_node *nprop, *np;

	/*
	 * Detect and enable JobRs
	 * First, find out how many ring spec'ed, allocate references
	 * for all, then go probe each one.
	 */
	nprop = ctrlpriv->pdev->dev.of_node;

	for_each_available_child_of_node(nprop, np)
		if (of_device_is_compatible(np, "fsl,sec-v4.0-job-ring") ||
		    of_device_is_compatible(np, "fsl,sec4.0-job-ring"))
			rspec++;

	ctrlpriv->jrpdev = devm_kcalloc(ctrlpriv->dev, rspec,
					sizeof(*ctrlpriv->jrpdev), GFP_KERNEL);
	if (ctrlpriv->jrpdev == NULL)
		return -ENOMEM;

	ring = 0;
	ctrlpriv->total_jobrs = 0;
	for_each_available_child_of_node(nprop, np)
		if (of_device_is_compatible(np, "fsl,sec-v4.0-job-ring") ||
		    of_device_is_compatible(np, "fsl,sec4.0-job-ring")) {
			ctrlpriv->jrpdev[ring] =
				of_platform_device_create(np, NULL,
							  ctrlpriv->dev);
			if (!ctrlpriv->jrpdev[ring]) {
				pr_warn("JR%d Platform device creation error\n",
					ring);
				continue;
			}

			if (of_property_read_u32_index(np, "reg", 0, &index)) {
				pr_warn("%s read reg property error %d.",
					np->full_name, index);
				continue;
			}
			/* Get actual job ring index from its offset
			 * ex: CAAM JR2 offset 0x30000 index = 2
			 */
			while (index > 16)
				index = index >> 4;
			index -= 1;
			ctrlpriv->jr[index] = (struct caam_job_ring __force *)
					     ((uint8_t *)ctrlpriv->ctrl +
					     (index + JR_BLOCK_NUMBER) *
					      block_offset);
			ctrlpriv->total_jobrs++;
			ring++;
		}
	return 0;
}

static void enable_qi(struct caam_drv_private *ctrlpriv, int block_offset)
{
	/* Check to see if QI present. If so, enable */
	ctrlpriv->qi_present =
			!!(rd_reg32(&ctrlpriv->ctrl->perfmon.comp_parms_ms) &
			   CTPR_MS_QI_MASK);
	if (ctrlpriv->qi_present) {
		ctrlpriv->qi = (struct caam_queue_if __force *)
			       ((uint8_t *)ctrlpriv->ctrl +
				 block_offset * QI_BLOCK_NUMBER);
		/* This is all that's required to physically enable QI */
		wr_reg32(&ctrlpriv->qi->qi_control_lo, QICTL_DQEN);
	}
}

static int read_first_jr_index(struct caam_drv_private *ctrlpriv)
{
	struct device_node *caam_node;
	int ret;
	u32 first_index;

	caam_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	ret = of_property_read_u32(caam_node,
		   "fsl,first-jr-index", &first_index);
	of_node_put(caam_node);
	if (ret == 0)
		if (first_index > 0 && first_index < 4)
			ctrlpriv->first_jr_index = first_index;
	return ret;
}

static int probe_w_seco(struct caam_drv_private *ctrlpriv)
{
	int ret = 0;
	struct device_node *np;
	u32 idx, status;

	ctrlpriv->has_seco = true;
	/*
	 * For imx8 page size is 64k, we can't access ctrl regs to dynamically
	 * obtain this info.
	 */
	ret = enable_jobrings(ctrlpriv, PG_SIZE_64K);
	if (ret)
		return ret;
	if (!ctrlpriv->total_jobrs) {
		dev_err(ctrlpriv->dev, "no job rings configured!\n");
		return -ENODEV;
	}

	/*
	 * Read first job ring index for aliased registers
	 */
	if (read_first_jr_index(ctrlpriv)) {
		dev_err(ctrlpriv->dev, "missing first job ring index!\n");
		return -ENODEV;
	}
	idx = ctrlpriv->first_jr_index;
	status = rd_reg32(&ctrlpriv->jr[idx]->perfmon.status);
	caam_little_end = !(bool)(status & (CSTA_PLEND | CSTA_ALT_PLEND));
	ctrlpriv->assure = ((struct caam_assurance __force *)
			    ((uint8_t *)ctrlpriv->ctrl +
			     PG_SIZE_64K * ASSURE_BLOCK_NUMBER));
	ctrlpriv->deco = ((struct caam_deco __force *)
			  ((uint8_t *)ctrlpriv->ctrl +
			   PG_SIZE_64K * DECO_BLOCK_NUMBER));

	detect_era(ctrlpriv);

	/* Get CAAM-SM node and of_iomap() and save */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (!np) {
		dev_warn(ctrlpriv->dev, "No CAAM-SM node found!\n");
		return -ENODEV;
	}

	ctrlpriv->sm_base = of_iomap(np, 0);
	ctrlpriv->sm_size = 0x3fff;

	/* Can't enable DECO WD and LPs those are in MCR */

	/*
	 * can't check for virtualization because we need access to SCFGR for it
	 */

	/* Set DMA masks according to platform ranging */
	if (sizeof(dma_addr_t) == sizeof(u64))
		if (of_device_is_compatible(ctrlpriv->pdev->dev.of_node,
					    "fsl,sec-v5.0"))
			dma_set_mask_and_coherent(ctrlpriv->dev,
						  DMA_BIT_MASK(40));
		else
			dma_set_mask_and_coherent(ctrlpriv->dev,
						  DMA_BIT_MASK(36));
	else
		dma_set_mask_and_coherent(ctrlpriv->dev, DMA_BIT_MASK(32));

	/*
	 * this is where we should run the descriptor for DRNG init
	 * TRNG must be initialized by SECO
	 */
	return ret;
}

/* Probe routine for CAAM top (controller) level */
static int caam_probe(struct platform_device *pdev)
{
	int ret;
	u64 caam_id;
	struct device *dev;
	struct device_node *nprop, *np;
	struct resource res_regs;
	struct caam_ctrl __iomem *ctrl;
	struct caam_drv_private *ctrlpriv;
	u32 comp_params;
	int pg_size;
	int block_offset = 0;

	ctrlpriv = devm_kzalloc(&pdev->dev, sizeof(*ctrlpriv), GFP_KERNEL);
	if (!ctrlpriv)
		return -ENOMEM;

	dev = &pdev->dev;
	dev_set_drvdata(dev, ctrlpriv);
	ctrlpriv->dev = dev;
	ctrlpriv->pdev = pdev;
	nprop = pdev->dev.of_node;

	if (!of_machine_is_compatible("fsl,imx8mq") &&
	     !of_machine_is_compatible("fsl,imx8qm") &&
	     !of_machine_is_compatible("fsl,imx8qxp")) {
		ret = init_clocks(ctrlpriv);
		if (ret)
			goto disable_clocks;
	}
	/* Get configuration properties from device tree */
	/* First, get register page */
	ctrl = of_iomap(nprop, 0);
	if (ctrl == NULL) {
		dev_err(dev, "caam: of_iomap() failed\n");
		ret = -ENOMEM;
		goto disable_clocks;
	}
	ctrlpriv->ctrl = (struct caam_ctrl __force *)ctrl;

	if (of_machine_is_compatible("fsl,imx8qm") ||
		 of_machine_is_compatible("fsl,imx8qxp")) {
		ret = probe_w_seco(ctrlpriv);
		if (ret)
			goto iounmap_ctrl;
		return ret;
	}

	ctrlpriv->has_seco = false;

	caam_little_end = !(bool)(rd_reg32(&ctrl->perfmon.status) &
				  (CSTA_PLEND | CSTA_ALT_PLEND));

	/* Finding the page size for using the CTPR_MS register */
	comp_params = rd_reg32(&ctrl->perfmon.comp_parms_ms);
	pg_size = (comp_params & CTPR_MS_PG_SZ_MASK) >> CTPR_MS_PG_SZ_SHIFT;

	/* Allocating the block_offset based on the supported page size on
	 * the platform
	 */
	if (pg_size == 0)
		block_offset = PG_SIZE_4K;
	else
		block_offset = PG_SIZE_64K;

	ctrlpriv->assure = (struct caam_assurance __force *)
			   ((uint8_t *)ctrl +
			    block_offset * ASSURE_BLOCK_NUMBER);
	ctrlpriv->deco = (struct caam_deco __force *)
			 ((uint8_t *)ctrl +
			 block_offset * DECO_BLOCK_NUMBER);

	detect_era(ctrlpriv);

	/* Get CAAM-SM node and of_iomap() and save */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (!np) {
		ret = -ENODEV;
		goto disable_clocks;
	}

	/* Get CAAM SM registers base address from device tree */
	ret = of_address_to_resource(np, 0, &res_regs);
	if (ret) {
		dev_err(dev, "failed to retrieve registers base from device tree\n");
		ret = -ENODEV;
		goto disable_clocks;
	}

	ctrlpriv->sm_phy = res_regs.start;
	ctrlpriv->sm_base = devm_ioremap_resource(dev, &res_regs);
	if (IS_ERR(ctrlpriv->sm_base)) {
		ret = PTR_ERR(ctrlpriv->sm_base);
		goto disable_clocks;
	}

	if (!of_machine_is_compatible("fsl,imx8mq") &&
	     !of_machine_is_compatible("fsl,imx8qm") &&
	     !of_machine_is_compatible("fsl,imx8qxp"))
		ctrlpriv->sm_size = resource_size(&res_regs);
	else
	    ctrlpriv->sm_size = PG_SIZE_64K;

	/*
	 * Enable DECO watchdogs and, if this is a PHYS_ADDR_T_64BIT kernel,
	 * long pointers in master configuration register
	 */
	clrsetbits_32(&ctrl->mcr, MCFGR_AWCACHE_MASK | MCFGR_LONG_PTR,
		      MCFGR_AWCACHE_CACH | MCFGR_AWCACHE_BUFF |
		      MCFGR_WDENABLE | MCFGR_LARGE_BURST |
		      (sizeof(dma_addr_t) == sizeof(u64) ? MCFGR_LONG_PTR : 0));

	handle_imx6_err005766(ctrlpriv);

	check_virt(ctrlpriv, comp_params);

	/* Set DMA masks according to platform ranging */
	if (sizeof(dma_addr_t) == sizeof(u64))
		if (of_device_is_compatible(nprop, "fsl,sec-v5.0"))
			dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
		else
			dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	else
		dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	ret = enable_jobrings(ctrlpriv, block_offset);
	if (ret)
		goto iounmap_ctrl;

	enable_qi(ctrlpriv, block_offset);

	/* If no QI and no rings specified, quit and go home */
	if ((!ctrlpriv->qi_present) && (!ctrlpriv->total_jobrs)) {
		dev_err(dev, "no queues configured, terminating\n");
		ret = -ENOMEM;
		goto caam_remove;
	}

	/* NOTE: RTIC detection ought to go here, around Si time */

	caam_id = (u64)rd_reg32(&ctrl->perfmon.caam_id_ms) << 32 |
		  (u64)rd_reg32(&ctrl->perfmon.caam_id_ls);

	dev_info(dev, "device ID = 0x%016llx (Era %d)\n"
			"job rings = %d, qi = %d\n",
			caam_id,
			ctrlpriv->era,
			ctrlpriv->total_jobrs, ctrlpriv->qi_present);

	init_debugfs(ctrlpriv);
	return 0;

caam_remove:
	caam_remove(pdev);
	return ret;

iounmap_ctrl:
	iounmap(ctrl);
disable_clocks:
	if (!of_machine_is_compatible("fsl,imx8mq") &&
	     !of_machine_is_compatible("fsl,imx8qm") &&
	     !of_machine_is_compatible("fsl,imx8qxp")) {
		clk_disable_unprepare(ctrlpriv->caam_emi_slow);
		clk_disable_unprepare(ctrlpriv->caam_aclk);
		clk_disable_unprepare(ctrlpriv->caam_mem);
		clk_disable_unprepare(ctrlpriv->caam_ipg);
	}
	return ret;
}

static struct of_device_id caam_match[] = {
	{
		.compatible = "fsl,sec-v4.0",
	},
	{
		.compatible = "fsl,sec4.0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, caam_match);

static struct platform_driver caam_driver = {
	.driver = {
		.name = "caam",
		.of_match_table = caam_match,
	},
	.probe       = caam_probe,
	.remove      = caam_remove,
};

module_platform_driver(caam_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL CAAM request backend");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
