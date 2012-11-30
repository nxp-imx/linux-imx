/*
 * CAAM control-plane driver backend
 * Controller-level driver, kernel property detection, initialization
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 */

#include "compat.h"
#include "regs.h"
#include "snvsregs.h"
#include "intern.h"
#include "jr.h"
#include "desc_constr.h"
#include "error.h"

static int caam_remove(struct platform_device *pdev)
{
	struct device *ctrldev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_jr *jrpriv;
	struct caam_full __iomem *topregs;
	int ring, ret = 0;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	topregs = (struct caam_full __iomem *)ctrlpriv->ctrl;

#ifndef CONFIG_OF
#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_SECVIO
	caam_secvio_shutdown(pdev);
#endif /* SECVIO */

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_SM
	caam_sm_shutdown(pdev);
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API
	if (ctrlpriv->rng_inst)
		caam_rng_shutdown();
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API
	caam_algapi_hash_shutdown(pdev);
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API
	caam_algapi_shutdown(pdev);
#endif
#endif
	/* shut down JobRs */
	for (ring = 0; ring < ctrlpriv->total_jobrs; ring++) {
		ret |= caam_jr_shutdown(ctrlpriv->jrdev[ring]);
		jrpriv = dev_get_drvdata(ctrlpriv->jrdev[ring]);
		irq_dispose_mapping(jrpriv->irq);
	}

	/* Shut down debug views */
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(ctrlpriv->dfs_root);
#endif

	/* Unmap SNVS and Secure Memory */
	iounmap(ctrlpriv->snvs);
	iounmap(ctrlpriv->sm_base);

	/* Unmap controller region */
	iounmap(&topregs->ctrl);

	/* shut clocks off before finalizing shutdown */
	clk_disable(ctrlpriv->caam_clk);

	kfree(ctrlpriv->jrdev);
	kfree(ctrlpriv);

	return ret;
}

/*
 * Descriptor to instantiate RNG State Handle 0 in normal mode and
 * load the JDKEK, TDKEK and TDSK registers
 */
static void build_instantiation_desc(u32 *desc)
{
	u32 *jump_cmd;

	init_job_desc(desc, 0);

	/* INIT RNG in non-test mode */
	append_operation(desc, OP_TYPE_CLASS1_ALG | OP_ALG_ALGSEL_RNG |
			 OP_ALG_AS_INIT);

	/* wait for done */
	jump_cmd = append_jump(desc, JUMP_CLASS_CLASS1);
	set_jump_tgt_here(desc, jump_cmd);

	/*
	 * load 1 to clear written reg:
	 * resets the done interrrupt and returns the RNG to idle.
	 */
	append_load_imm_u32(desc, 1, LDST_SRCDST_WORD_CLRW);

	/* generate secure keys (non-test) */
	append_operation(desc, OP_TYPE_CLASS1_ALG | OP_ALG_ALGSEL_RNG |
			 OP_ALG_RNG4_SK);
}

struct instantiate_result {
	struct completion completion;
	int err;
};

static void rng4_init_done(struct device *dev, u32 *desc, u32 err,
			   void *context)
{
	struct instantiate_result *instantiation = context;

	if (err) {
		char tmp[CAAM_ERROR_STR_MAX];

		dev_err(dev, "%08x: %s\n", err, caam_jr_strstatus(tmp, err));
	}

	instantiation->err = err;
	complete(&instantiation->completion);
}

static int instantiate_rng(struct device *jrdev)
{
	struct instantiate_result instantiation;

	dma_addr_t desc_dma;
	u32 *desc;
	int ret;

	desc = kmalloc(CAAM_CMD_SZ * 6, GFP_KERNEL | GFP_DMA);
	if (!desc) {
		dev_err(jrdev, "cannot allocate RNG init descriptor memory\n");
		return -ENOMEM;
	}

	build_instantiation_desc(desc);
	desc_dma = dma_map_single(jrdev, desc, desc_bytes(desc), DMA_TO_DEVICE);
	dma_sync_single_for_device(jrdev, desc_dma, desc_bytes(desc),
				   DMA_TO_DEVICE);
	init_completion(&instantiation.completion);

	ret = caam_jr_enqueue(jrdev, desc, rng4_init_done, &instantiation);
	if (!ret) {
		wait_for_completion_interruptible(&instantiation.completion);
		ret = instantiation.err;
		if (ret)
			dev_err(jrdev, "unable to instantiate RNG\n");
	}

	dma_unmap_single(jrdev, desc_dma, desc_bytes(desc), DMA_TO_DEVICE);

	kfree(desc);

	return ret;
}

/*
 * By default, the TRNG runs for 200 clocks per sample;
 * 1600 clocks per sample generates better entropy.
 */
static void kick_trng(struct platform_device *pdev)
{
	struct device *ctrldev = &pdev->dev;
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(ctrldev);
	struct caam_full __iomem *topregs;
	struct rng4tst __iomem *r4tst;
	u32 val;

	topregs = (struct caam_full __iomem *)ctrlpriv->ctrl;
	r4tst = &topregs->ctrl.r4tst[0];

	/* put RNG4 into program mode */
	setbits32(&r4tst->rtmctl, RTMCTL_PRGM);
	/* Set clocks per sample to the default, and divider to zero */
	val = rd_reg32(&r4tst->rtsdctl);
	val = ((val & ~RTSDCTL_ENT_DLY_MASK) |
	       (RNG4_ENT_CLOCKS_SAMPLE << RTSDCTL_ENT_DLY_SHIFT)) &
	       ~RTMCTL_OSC_DIV_MASK;
	wr_reg32(&r4tst->rtsdctl, val);
	/* min. freq. count */
	wr_reg32(&r4tst->rtfrqmin, RNG4_ENT_CLOCKS_SAMPLE / 4);
	/* max. freq. count */
	wr_reg32(&r4tst->rtfrqmax, RNG4_ENT_CLOCKS_SAMPLE * 8);
	/* put RNG4 into run mode */
	clrbits32(&r4tst->rtmctl, RTMCTL_PRGM);
}

/* Probe routine for CAAM top (controller) level */
static int caam_probe(struct platform_device *pdev)
{
	int d, ring, rspec, status;
	struct device *dev;
	struct device_node *np;
	struct caam_ctrl __iomem *ctrl;
	struct caam_full __iomem *topregs;
	struct snvs_full __iomem *snvsregs;
	void __iomem *sm_base;
	struct caam_drv_private *ctrlpriv;
	u32 deconum;
#ifdef CONFIG_DEBUG_FS
	struct caam_perfmon *perfmon;
#endif
#ifdef CONFIG_OF
	struct device_node *nprop;
#else
	struct resource *res;
	char *rname, inst;
#endif
#ifdef CONFIG_ARM
	int ret = 0;
#endif

	ctrlpriv = kzalloc(sizeof(struct caam_drv_private), GFP_KERNEL);
	if (!ctrlpriv)
		return -ENOMEM;

	dev = &pdev->dev;
	dev_set_drvdata(dev, ctrlpriv);
	ctrlpriv->pdev = pdev;

	/* Get configuration properties from device tree */
	/* First, get register page */
#ifdef CONFIG_OF
	nprop = pdev->dev.of_node;
	ctrl = of_iomap(nprop, 0);
	if (ctrl == NULL) {
		dev_err(dev, "caam: of_iomap() failed\n");
		return -ENOMEM;
	}
#else
	/* Get the named resource for the controller base address */
	res = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "iobase_caam");
	if (!res) {
		dev_err(dev, "caam: invalid address resource type\n");
		return -ENODEV;
	}
	ctrl = ioremap(res->start, SZ_64K);
	if (ctrl == NULL) {
		dev_err(dev, "caam: ioremap() failed\n");
		return -ENOMEM;
	}
#endif

	ctrlpriv->ctrl = (struct caam_ctrl __force *)ctrl;

	/* topregs used to derive pointers to CAAM sub-blocks only */
	topregs = (struct caam_full __iomem *)ctrl;

	/*
	 * Next, map SNVS register page
	 * FIXME: MX6 has a separate RTC driver using SNVS. This driver
	 * will have a mapped pointer to SNVS registers also, which poses
	 * a conflict if we're not very careful to stay away from registers
	 * and interrupts that it uses. In the future, pieces of that driver
	 * may need to migrate down here. In the meantime, use caution with
	 * this pointer. Also note that the snvs-rtc driver probably controls
	 * SNVS device clocks.
	 */
#ifdef CONFIG_OF
	/* Get SNVS register page */
#else
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iobase_snvs");
	if (!res) {
		dev_err(dev, "snvs: invalid address resource type\n");
		return -ENODEV;
	}
	snvsregs = ioremap(res->start, res->end - res->start + 1);
	if (snvsregs == NULL) {
		dev_err(dev, "snvs: ioremap() failed\n");
		iounmap(ctrl);
		return -ENOMEM;
	}
#endif
	ctrlpriv->snvs = (struct snvs_full __force *)snvsregs;

	/* Now map CAAM-Secure Memory Space */
#ifdef CONFIG_OF
	/* Get CAAM-SM node and of_iomap() and save */
#else
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "iobase_caam_sm");
	if (!res) {
		dev_err(dev, "caam_sm: invalid address resource type\n");
		return -ENODEV;
	}
	sm_base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (sm_base == NULL) {
		dev_err(dev, "caam_sm: ioremap_nocache() failed\n");
		iounmap(ctrl);
		iounmap(snvsregs);
		return -ENOMEM;
	}
#endif
	ctrlpriv->sm_base = (void __force *)sm_base;
	ctrlpriv->sm_size = res->end - res->start + 1;

	/*
	 * Get the IRQ for security violations
	 */
#ifdef CONFIG_OF
	ctrlpriv->secvio_irq = of_irq_to_resource(nprop, 0, NULL);
#else
	res = platform_get_resource_byname(pdev,
		IORESOURCE_IRQ, "irq_sec_vio");
	if (!res) {
		dev_err(dev, "caam: invalid IRQ resource type\n");
		return -ENODEV;
	}
	ctrlpriv->secvio_irq = res->start;
#endif

/*
 * ARM targets tend to have clock control subsystems that can
 * enable/disable clocking to our device. Turn clocking on to proceed
 */
#ifdef CONFIG_ARM
	ctrlpriv->caam_clk = clk_get(&ctrlpriv->pdev->dev, "caam_clk");
	if (IS_ERR(ctrlpriv->caam_clk)) {
		ret = PTR_ERR(ctrlpriv->caam_clk);
		dev_err(&ctrlpriv->pdev->dev,
			"can't identify CAAM bus clk: %d\n", ret);
		return -ENODEV;
	}

	ret = clk_enable(ctrlpriv->caam_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't enable CAAM bus clock: %d\n", ret);
		return -ENODEV;
	}

	pr_debug("%s caam_clk:%d\n", __func__,
		(int)clk_get_rate(ctrlpriv->caam_clk));
#endif

	/*
	 * Enable DECO watchdogs and, if this is a PHYS_ADDR_T_64BIT kernel,
	 * 36-bit pointers in master configuration register
	 */
	setbits32(&topregs->ctrl.mcr, MCFGR_WDENABLE |
		  (sizeof(dma_addr_t) == sizeof(u64) ? MCFGR_LONG_PTR : 0));

#ifdef CONFIG_ARCH_MX6
	/*
	 * ERRATA:  mx6 devices have an issue wherein AXI bus transactions
	 * may not occur in the correct order. This isn't a problem running
	 * single descriptors, but can be if running multiple concurrent
	 * descriptors. Reworking the driver to throttle to single requests
	 * is impractical, thus the workaround is to limit the AXI pipeline
	 * to a depth of 1 (from it's default of 4) to preclude this situation
	 * from occurring.
	 */
	wr_reg32(&topregs->ctrl.mcr,
		 (rd_reg32(&topregs->ctrl.mcr) & ~(MCFGR_AXIPIPE_MASK)) |
		 ((1 << MCFGR_AXIPIPE_SHIFT) & MCFGR_AXIPIPE_MASK));
#endif

	/* Set DMA masks according to platform ranging */
	if (sizeof(dma_addr_t) == sizeof(u64))
		dma_set_mask(dev, DMA_BIT_MASK(36));
	else
		dma_set_mask(dev, DMA_BIT_MASK(32));

	/* Find out how many DECOs are present */
	deconum = (rd_reg64(&topregs->ctrl.perfmon.cha_num) &
		   CHA_NUM_DECONUM_MASK) >> CHA_NUM_DECONUM_SHIFT;

	ctrlpriv->deco = kmalloc(deconum * sizeof(struct caam_deco *),
				 GFP_KERNEL);

	/*
	 * Detect and enable JobRs
	 * First, find out how many ring spec'ed, allocate references
	 * for all, then go probe each one.
	 */
	rspec = 0;
#ifdef CONFIG_OF
	for_each_compatible_node(np, NULL, "fsl,sec-v4.0-job-ring")
		rspec++;
#else
	np = NULL;

	/* Build the name of the IRQ platform resources to identify */
	rname = kzalloc(strlen(JR_IRQRES_NAME_ROOT) + 1, 0);
	if (rname == NULL) {
		iounmap(&topregs->ctrl);
		return -ENOMEM;
	}

	/*
	 * Emulate behavor of for_each_compatible_node() for non OF targets
	 * Identify all IRQ platform resources present
	 */
	for (d = 0; d < 4; d++)	{
		rname[0] = 0;
		inst = '0' + d;
		strcat(rname, JR_IRQRES_NAME_ROOT);
		strncat(rname, &inst, 1);
		res = platform_get_resource_byname(pdev,
					IORESOURCE_IRQ, rname);
		if (res)
			rspec++;
	}
	kfree(rname);
#endif
	ctrlpriv->jrdev = kzalloc(sizeof(struct device *) * rspec, GFP_KERNEL);
	if (ctrlpriv->jrdev == NULL) {
		iounmap(&topregs->ctrl);
		return -ENOMEM;
	}

	ring = 0;
	ctrlpriv->total_jobrs = 0;
#ifdef CONFIG_OF
	for_each_compatible_node(np, NULL, "fsl,sec-v4.0-job-ring") {
#else
	for (d = 0; d < rspec; d++)	{
#endif
		caam_jr_probe(pdev, np, ring);
		ctrlpriv->total_jobrs++;
		ring++;
	}

	/* Check to see if QI present. If so, enable */
	ctrlpriv->qi_present = !!(rd_reg64(&topregs->ctrl.perfmon.comp_parms) &
				  CTPR_QI_MASK);
	if (ctrlpriv->qi_present) {
		ctrlpriv->qi = (struct caam_queue_if __force *)&topregs->qi;
		/* This is all that's required to physically enable QI */
		wr_reg32(&topregs->qi.qi_control_lo, QICTL_DQEN);
	}

	/* If no QI and no rings specified, quit and go home */
	if ((!ctrlpriv->qi_present) && (!ctrlpriv->total_jobrs)) {
		dev_err(dev, "no queues configured, terminating\n");
		caam_remove(pdev);
		return -ENOMEM;
	}

	/*
	 * RNG4 based SECs (v5+ | >= i.MX6) need special initialization prior
	 * to executing any descriptors. If there's a problem with init,
	 * remove other subsystems and return; internal padding functions
	 * cannot run without an RNG. This procedure assumes a single RNG4
	 * instance.
	 */
	if ((rd_reg64(&topregs->ctrl.perfmon.cha_id) & CHA_ID_RNG_MASK)
	    == CHA_ID_RNG_4) {
		kick_trng(pdev);
		ret = instantiate_rng(ctrlpriv->jrdev[0]);
		if (ret) {
			caam_remove(pdev);
			return -ENODEV;
		}
		ctrlpriv->rng_inst++;
	}

	/* NOTE: RTIC detection ought to go here, around Si time */

	/* Initialize queue allocator lock */
	spin_lock_init(&ctrlpriv->jr_alloc_lock);

	/* Report "alive" for developer to see */
	dev_info(dev, "device ID = 0x%016llx\n",
		 rd_reg64(&topregs->ctrl.perfmon.caam_id));
	dev_info(dev, "job rings = %d, qi = %d\n",
		 ctrlpriv->total_jobrs, ctrlpriv->qi_present);

#ifdef CONFIG_DEBUG_FS
	/*
	 * FIXME: needs better naming distinction, as some amalgamation of
	 * "caam" and nprop->full_name. The OF name isn't distinctive,
	 * but does separate instances
	 */
	perfmon = (struct caam_perfmon __force *)&ctrl->perfmon;

	ctrlpriv->dfs_root = debugfs_create_dir("caam", NULL);
	ctrlpriv->ctl = debugfs_create_dir("ctl", ctrlpriv->dfs_root);

	/* Controller-level - performance monitor counters */
	ctrlpriv->ctl_rq_dequeued =
		debugfs_create_u64("rq_dequeued",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->req_dequeued);
	ctrlpriv->ctl_ob_enc_req =
		debugfs_create_u64("ob_rq_encrypted",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ob_enc_req);
	ctrlpriv->ctl_ib_dec_req =
		debugfs_create_u64("ib_rq_decrypted",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ib_dec_req);
	ctrlpriv->ctl_ob_enc_bytes =
		debugfs_create_u64("ob_bytes_encrypted",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ob_enc_bytes);
	ctrlpriv->ctl_ob_prot_bytes =
		debugfs_create_u64("ob_bytes_protected",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ob_prot_bytes);
	ctrlpriv->ctl_ib_dec_bytes =
		debugfs_create_u64("ib_bytes_decrypted",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ib_dec_bytes);
	ctrlpriv->ctl_ib_valid_bytes =
		debugfs_create_u64("ib_bytes_validated",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->ib_valid_bytes);

	/* Controller level - global status values */
	ctrlpriv->ctl_faultaddr =
		debugfs_create_u64("fault_addr",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->faultaddr);
	ctrlpriv->ctl_faultdetail =
		debugfs_create_u32("fault_detail",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->faultdetail);
	ctrlpriv->ctl_faultstatus =
		debugfs_create_u32("fault_status",
				   S_IRUSR | S_IRGRP | S_IROTH,
				   ctrlpriv->ctl, &perfmon->status);

	/* Internal covering keys (useful in non-secure mode only) */
	ctrlpriv->ctl_kek_wrap.data = &ctrlpriv->ctrl->kek[0];
	ctrlpriv->ctl_kek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_kek = debugfs_create_blob("kek",
						S_IRUSR |
						S_IRGRP | S_IROTH,
						ctrlpriv->ctl,
						&ctrlpriv->ctl_kek_wrap);

	ctrlpriv->ctl_tkek_wrap.data = &ctrlpriv->ctrl->tkek[0];
	ctrlpriv->ctl_tkek_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tkek = debugfs_create_blob("tkek",
						 S_IRUSR |
						 S_IRGRP | S_IROTH,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tkek_wrap);

	ctrlpriv->ctl_tdsk_wrap.data = &ctrlpriv->ctrl->tdsk[0];
	ctrlpriv->ctl_tdsk_wrap.size = KEK_KEY_SIZE * sizeof(u32);
	ctrlpriv->ctl_tdsk = debugfs_create_blob("tdsk",
						 S_IRUSR |
						 S_IRGRP | S_IROTH,
						 ctrlpriv->ctl,
						 &ctrlpriv->ctl_tdsk_wrap);
#endif

/*
 * Non OF configurations use plaform_device, and therefore cannot simply
 * go and get a device node by name, which the algapi module startup code
 * assumes is possible. Therefore, non OF configurations will have to
 * start up the API code explicitly, and forego modularization
 */
#ifndef CONFIG_OF
#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_CRYPTO_API
	status = caam_algapi_startup(pdev);
	if (status) {
		caam_remove(pdev);
		return status;
	}
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API
	status = caam_algapi_hash_startup(pdev);
	if (status) {
		caam_remove(pdev);
		return status;
	}
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API
	if (ctrlpriv->rng_inst)
		caam_rng_startup(pdev);
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_SM
	status = caam_sm_startup(pdev);
	if (status) {
		caam_remove(pdev);
		return status;
	}
#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_SM_TEST
	caam_sm_example_init(pdev);
#endif /* SM_TEST */
#endif /* SM */

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_SECVIO
	caam_secvio_startup(pdev);
#endif /* SECVIO */

#endif /* CONFIG_OF */
	return status;
}

#ifdef CONFIG_OF
static struct of_device_id caam_match[] = {
	{
		.compatible = "fsl,sec-v4.0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, caam_match);
#endif /* CONFIG_OF */

static struct platform_driver caam_driver = {
	.driver = {
		.name = "caam",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = caam_match,
#else

#endif
	},
	.probe       = caam_probe,
	.remove      = __devexit_p(caam_remove),
};

static int __init caam_base_init(void)
{
#ifdef CONFIG_OF
	return of_register_platform_driver(&caam_driver);
#else
	return platform_driver_register(&caam_driver);
#endif
}

static void __exit caam_base_exit(void)
{
#ifdef CONFIG_OF
	return of_unregister_platform_driver(&caam_driver);
#else
	return platform_driver_unregister(&caam_driver);
#endif
}

module_init(caam_base_init);
module_exit(caam_base_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL CAAM request backend");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
