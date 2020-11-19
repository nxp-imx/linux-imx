// SPDX-License-Identifier: GPL-2.0-only
/*
 * IMX MIC Platform Software Stack
 *
 * Copyright(c) 2020 NXP.
 *
 * Intel MIC Host driver.
 */

#include "imx_mic_main.h"

static const char imx_mic_driver_name[] = "imx_mic";

static const struct pci_device_id imx_mic_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_IMX_MIC)},

	/* required last entry */
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, imx_mic_pci_tbl);

/* ID allocator for MIC devices */
static struct ida g_imx_mic_ida;

/* Initialize the MIC bootparams */
static void imx_mic_bootparam_init(struct imx_mic_device *mdev)
{
	struct mic_bootparam *bootparam = mdev->dp;

	bootparam->magic = cpu_to_le32(MIC_MAGIC);
	bootparam->h2c_config_db = -1;
	bootparam->node_id = mdev->id + 1;
}

static int find_cma(struct cma *cma, void *p)
{
	if (strcmp(cma_get_name(cma), "fsl,imx_mic") == 0)
		*((struct cma **)p) = cma;

	return 0;
}

/**
 * imx_mic_probe - Device Initialization Routine
 *
 * @pdev: PCI device structure
 * @ent: entry in imx_mic_pci_tbl
 *
 * returns 0 on success, < 0 on failure.
 */
static int imx_mic_probe(struct pci_dev *pdev,
		     const struct pci_device_id *ent)
{
	int rc;
	struct imx_mic_device *mdev;
	struct cma *cma = NULL;
	dma_addr_t dp_dma_addr;
	void *dp_va;
	void __iomem *temp_bar_base;

	mdev = kzalloc(sizeof(*mdev), GFP_KERNEL);
	if (!mdev) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "mdev kmalloc failed rc %d\n", rc);
		goto mdev_alloc_fail;
	}

	mdev->id = ida_simple_get(&g_imx_mic_ida, 0, MIC_MAX_NUM_DEVS, GFP_KERNEL);
	if (mdev->id < 0) {
		rc = mdev->id;
		dev_err(&pdev->dev, "ida_simple_get failed rc %d\n", rc);
		goto ida_fail;
	}

	mdev->pdev = pdev;
	mdev->family = MIC_FAMILY_IMX8;
	mdev->stepping = pdev->revision;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "failed to enable pci device.\n");
		goto ida_remove;
	}

	pci_set_master(pdev);

	rc = pci_request_regions(pdev, imx_mic_driver_name);
	if (rc) {
		dev_err(&pdev->dev, "failed to get pci regions.\n");
		goto disable_device;
	}

	/* card MU registers */
	mdev->mmio.pa = pci_resource_start(pdev, IMX_MIC_MMIO_BAR);
	mdev->mmio.len = pci_resource_len(pdev, IMX_MIC_MMIO_BAR);
	mdev->mmio.va = pci_ioremap_bar(pdev, IMX_MIC_MMIO_BAR);
	if (!mdev->mmio.va) {
		dev_err(&pdev->dev, "Cannot remap MMIO BAR\n");
		rc = -EIO;
		goto release_regions;
	}

	/* card swiotlb region */
	mdev->aper.pa = pci_resource_start(pdev, IMX_MIC_APER_BAR);

	/* host allocate device page from cma to map for card */
	cma_for_each_area(find_cma, &cma);
	if (cma)
		dev_set_cma_area(&pdev->dev, cma);
	else
		dev_warn(&pdev->dev, "Can't set the specific cma region for device\n");
	dp_va = dma_alloc_coherent(&pdev->dev, MIC_DP_SIZE, &dp_dma_addr, GFP_KERNEL);
	if (!dp_va) {
		dev_err(&pdev->dev, "Failed to allocate ddr space for EP\n");
		rc = -ENOMEM;
		goto unmap_mmio;
	}

	/* save the device page address */
	mdev->dp_dma_addr = dp_dma_addr;
	mdev->dp = dp_va;

	imx_mic_bootparam_init(mdev);

	/* TEMP BAR is used for host to transfer device page address and size to card */
	temp_bar_base = pci_ioremap_bar(pdev, IMX_MIC_TEMP_BAR);
	if (!temp_bar_base) {
		dev_err(&pdev->dev, "Cannot remap TEMP BAR\n");
		rc = -EIO;
		goto free_dma;
	}
	writeq(mdev->aper.pa, temp_bar_base + RC_MAP_ADDR_OFFSET);
	writeq(SZ_32M, temp_bar_base + RC_SHMEM_SIZE_OFFSET);
	writeq(mdev->dp_dma_addr, temp_bar_base + RC_SHMEM_ADDR_OFFSET);
	mdev->temp_bar_base = temp_bar_base;

	pci_set_drvdata(pdev, mdev);

	mdev->cosm_dev = cosm_register_device(&mdev->pdev->dev, &cosm_hw_ops);
	if (IS_ERR(mdev->cosm_dev)) {
		rc = PTR_ERR(mdev->cosm_dev);
		dev_err(&pdev->dev, "cosm_add_device failed\n");
		goto unmap_temp;
	}

	return 0;
unmap_temp:
	iounmap(mdev->temp_bar_base);
free_dma:
	dma_free_coherent(&pdev->dev, MIC_DP_SIZE, mdev->dp, mdev->dp_dma_addr);
unmap_mmio:
	iounmap(mdev->mmio.va);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
ida_remove:
	ida_simple_remove(&g_imx_mic_ida, mdev->id);
ida_fail:
	kfree(mdev);
mdev_alloc_fail:
	dev_err(&pdev->dev, "Probe failed rc %d\n", rc);
	return rc;
}

/**
 * imx_mic_remove - Device Removal Routine
 * imx_mic_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 *
 * @pdev: PCI device structure
 */
static void imx_mic_remove(struct pci_dev *pdev)
{
	struct imx_mic_device *mdev;

	mdev = pci_get_drvdata(pdev);
	if (!mdev)
		return;

	cosm_unregister_device(mdev->cosm_dev);
	imx_mic_bootparam_init(mdev);
	iounmap(mdev->temp_bar_base);
	dma_free_coherent(&pdev->dev, MIC_DP_SIZE, mdev->dp, mdev->dp_dma_addr);
	iounmap(mdev->mmio.va);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	ida_simple_remove(&g_imx_mic_ida, mdev->id);
	kfree(mdev);
}

static struct pci_driver imx_mic_driver = {
	.name = imx_mic_driver_name,
	.id_table = imx_mic_pci_tbl,
	.probe = imx_mic_probe,
	.remove = imx_mic_remove
};

static int __init imx_mic_init(void)
{
	int ret;

	ida_init(&g_imx_mic_ida);
	ret = pci_register_driver(&imx_mic_driver);
	if (ret) {
		pr_err("pci_register_driver failed ret %d\n", ret);
		goto cleanup_ida;
	}
	return 0;
cleanup_ida:
	ida_destroy(&g_imx_mic_ida);
	return ret;
}

static void __exit imx_mic_exit(void)
{
	pci_unregister_driver(&imx_mic_driver);
	ida_destroy(&g_imx_mic_ida);
}

module_init(imx_mic_init);
module_exit(imx_mic_exit);

MODULE_AUTHOR("NXP Corporation");
MODULE_DESCRIPTION("NXP MIC Host driver");
MODULE_LICENSE("GPL v2");
