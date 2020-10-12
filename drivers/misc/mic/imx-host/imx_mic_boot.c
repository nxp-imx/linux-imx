// SPDX-License-Identifier: GPL-2.0-only
/*
 * IMX MIC Platform Software Stack
 *
 * Copyright(c) 2020 NXP.
 *
 * Intel MIC Host driver.
 */

#include "imx_mic_main.h"

static inline struct imx_mic_device *vpdev_to_mdev(struct device *dev)
{
	return dev_get_drvdata(dev->parent);
}

static struct mic_irq *
imx_mic_request_irq(struct vop_device *vpdev,
		  irqreturn_t (*func)(int irq, void *data),
		  const char *name, void *data, int intr_src)
{
	struct imx_mic_device *mdev = vpdev_to_mdev(&vpdev->dev);
	struct pci_dev *pdev = mdev->pdev;
	unsigned long cookie = intr_src;
	int irq_num = -1;
	int rc = 0;

	irq_num = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (irq_num < 0) {
		dev_err(&pdev->dev, "Failed to get MSI interrupts\n");
		rc = -ENOSPC;
		goto err;
	}
	mdev->num_vectors = irq_num;

	rc = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, 0),
			       func, IRQF_SHARED, name, data);
	if (rc) {
		dev_err(&pdev->dev, "failed to request IRQ %d for MSI\n",
					pci_irq_vector(pdev, 0));
		goto free_irq;
	}
	return (struct mic_irq *)cookie;

free_irq:
	pci_free_irq_vectors(pdev);
err:
	return ERR_PTR(rc);
}

static void imx_mic_free_irq(struct vop_device *vpdev,
			   struct mic_irq *cookie, void *data)
{
	struct imx_mic_device *mdev = vpdev_to_mdev(&vpdev->dev);
	struct pci_dev *pdev = mdev->pdev;

	devm_free_irq(&pdev->dev, pci_irq_vector(pdev, 0), data);
	mdev->num_vectors = 0;
	pci_free_irq_vectors(pdev);
}

static void imx_mic_ack_interrupt(struct vop_device *vpdev, int num)
{
	/*
	 * Nothing to do here, interrupt sources register was cleared by hardware, upper
	 * layer force to implement this since it will be called without Null check.
	 */
}

static int imx_mic_next_db(struct vop_device *vpdev)
{
	/* return the only one host mu interrupt number */
	return MIC_CONFIG_DB;
}

static void *imx_mic_get_dp(struct vop_device *vpdev)
{
	struct imx_mic_device *mdev = vpdev_to_mdev(&vpdev->dev);

	return mdev->dp;
}

static dma_addr_t imx_mic_get_dp_dma(struct vop_device *vpdev)
{
	struct imx_mic_device *mdev = vpdev_to_mdev(&vpdev->dev);

	return mdev->dp_dma_addr;
}

static void __iomem *imx_mic_get_remote_dp(struct vop_device *vpdev)
{
	return NULL;
}

static void imx_mic_intr_send(void __iomem *base, int index)
{
	u32 val;

	val = readl(base + MIC_INTR_MU_CR);
	val |= MIC_INTR_MU_BCR_GIRn(index);
	writel(val, base + MIC_INTR_MU_CR);
}

static void imx_mic_send_intr(struct vop_device *vpdev, int db)
{
	struct imx_mic_device *mdev = vpdev_to_mdev(&vpdev->dev);

	if (db == MIC_CONFIG_DB || db == MIC_DATA_DB)
		imx_mic_intr_send(mdev->mmio.va, db);
	else
		dev_err(&vpdev->dev, "invalid doorbell index\n");
}

static void __iomem *imx_mic_ioremap(struct vop_device *vpdev,
				   dma_addr_t pa, size_t len)
{
	return ioremap(pa, len);
}

static void imx_mic_iounmap(struct vop_device *vpdev, void __iomem *va)
{
	return iounmap(va);
}

static struct vop_hw_ops vop_hw_ops = {
	.request_irq = imx_mic_request_irq,
	.free_irq = imx_mic_free_irq,
	.ack_interrupt = imx_mic_ack_interrupt,
	.next_db = imx_mic_next_db,
	.get_dp = imx_mic_get_dp,
	.get_dp_dma = imx_mic_get_dp_dma,
	.get_remote_dp = imx_mic_get_remote_dp,
	.send_intr = imx_mic_send_intr,
	.remap = imx_mic_ioremap,
	.unmap = imx_mic_iounmap,
};

static inline struct imx_mic_device *cosmdev_to_mdev(struct cosm_device *cdev)
{
	return dev_get_drvdata(cdev->dev.parent);
}

static void imx_mic_reset(struct cosm_device *cdev)
{
	/*
	 * Nothing to do here, everything needed will be done in imx_mic_stop, upper
	 * layer force to implement this since it will be called without Null check.
	 */
}

static bool imx_mic_ready(struct cosm_device *cdev)
{
	return true;
}

/**
 * imx_mic_start - Start the IMX MIC.
 * @cdev: pointer to cosm_device instance
 * @id: IMX MIC device id/index provided by COSM
 *
 * This function prepares an IMX MIC for boot and initiates boot.
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 *
 * For all cosm_hw_ops the caller holds a mutex to ensure serialization.
 */
static int imx_mic_start(struct cosm_device *cdev, int id)
{
	struct imx_mic_device *mdev = cosmdev_to_mdev(cdev);
	int rc;

	mdev->vpdev = vop_register_device(&mdev->pdev->dev,
					  VOP_DEV_TRNSP, NULL,
					  &vop_hw_ops, id + 1, NULL,
					  NULL);
	if (IS_ERR(mdev->vpdev)) {
		rc = PTR_ERR(mdev->vpdev);
		return rc;
	}

	return 0;
}

/**
 * imx_mic_stop - Prepare the IMX MIC for reset and trigger reset.
 * @cdev: pointer to cosm_device instance
 * @force: force a IMX MIC to reset even if it is already offline.
 *
 * RETURNS: None.
 */
static void imx_mic_stop(struct cosm_device *cdev, bool force)
{
	struct imx_mic_device *mdev = cosmdev_to_mdev(cdev);

	vop_unregister_device(mdev->vpdev);
}

struct cosm_hw_ops cosm_hw_ops = {
	.reset = imx_mic_reset,
	.ready = imx_mic_ready,
	.start = imx_mic_start,
	.stop = imx_mic_stop,
};
