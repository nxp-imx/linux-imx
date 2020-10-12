// SPDX-License-Identifier: GPL-2.0
// Copyright 2020 NXP

#include <linux/imx_mic_epf.h>
#include "imx_mic_boot.h"
#include "../common/imx_mic_common.h"

static const char imx_mic_driver_name[] = "imx_mic";

static struct imx_mic_driver *g_mdrv;

/*
 * Just enable irq here since irq is requested in imx_mic_driver_init function.
 */
static struct mic_irq *_imx_mic_request_irq(struct vop_device *vpdev,
					    irqreturn_t (*func)(int irq, void *data),
					    const char *name, void *data, int intr_src)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	struct imx_mic_intr *mintr = mdrv->mintr;
	unsigned long cookie = intr_src;
	int ret;

	if (cookie == MIC_CONFIG_DB) {
		mintr->config_intr = func;
		mintr->config_intr_priv = data;
	} else if (cookie == MIC_DATA_DB) {
		mintr->data_intr = func;
		mintr->data_intr_priv = data;
	} else {
		dev_err(mdrv->dev, "invalid doorbell index\n");
		ret = -EINVAL;
		goto err;
	}

	imx_mic_intr_enable(mdrv, cookie);

	return (struct mic_irq *)cookie;

err:
	return ERR_PTR(ret);
}

/*
 * Just disable irq here since irq is freed in imx_mic_driver_uninit function.
 */
static void _imx_mic_free_irq(struct vop_device *vpdev,
			      struct mic_irq *cookie, void *data)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	int index = (unsigned long)cookie;

	imx_mic_intr_disable(mdrv, index);
}

static void _imx_mic_ack_interrupt(struct vop_device *vpdev, int num)
{
	struct imx_mic_driver *mdrv = g_mdrv;

	imx_mic_intr_ack(mdrv, num);
}

static int _imx_mic_next_db(struct vop_device *vpdev)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	static unsigned int i;
	unsigned int db_index;

	if (i == 0)
		db_index = MIC_CONFIG_DB;
	if (i == 1)
		db_index = MIC_DATA_DB;

	i++;
	if (i > 2)
		dev_err(mdrv->dev, "doorbell cannot be trusted\n");

	return db_index;
}

static void __iomem *_imx_mic_get_remote_dp(struct vop_device *vpdev)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	struct imx_mic_device *mdev = &mdrv->mdev;

	return mdev->mmio.va;
}

static void _imx_mic_send_intr(struct vop_device *vpdev, int db)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	struct imx_mic_epf *mepf = mdrv->mepf;
	struct pci_epf *epf = mepf->epf;
	struct pci_epc *epc = epf->epc;

	pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI, MIC_PCI_MSI_INDEX);
}

static void __iomem *_imx_mic_ioremap(struct vop_device *vpdev,
				 dma_addr_t pa, size_t len)
{
	struct imx_mic_driver *mdrv = g_mdrv;
	struct imx_mic_device *mdev = &mdrv->mdev;

	/* Note: pa should be RC's absolute physical address */
	return (mdev->mmio.va + (pa - mdev->mmio.pa));
}

static void _imx_mic_iounmap(struct vop_device *vpdev, void __iomem *va)
{
	/* nothing to do */
}

static struct vop_hw_ops imx_mic_hw_ops = {
	.request_irq = _imx_mic_request_irq,
	.free_irq = _imx_mic_free_irq,
	.ack_interrupt = _imx_mic_ack_interrupt,
	.next_db = _imx_mic_next_db,
	.get_remote_dp = _imx_mic_get_remote_dp,
	.send_intr = _imx_mic_send_intr,
	.remap = _imx_mic_ioremap,
	.unmap = _imx_mic_iounmap,
};

static int __init imx_mic_driver_init(struct imx_mic_driver *mdrv)
{
	int ret;

	ret = imx_mic_intr_init(mdrv);
	if (ret) {
		dev_err(mdrv->dev, "imx_mic_intr_init failed\n");
		return ret;
	}

	mdrv->vpdev = vop_register_device(mdrv->dev, VOP_DEV_TRNSP,
					  NULL, &imx_mic_hw_ops, 0,
					  NULL, NULL);
	if (IS_ERR(mdrv->vpdev)) {
		ret = PTR_ERR(mdrv->vpdev);
		return ret;
	}

	return 0;
}

static void imx_mic_driver_uninit(struct imx_mic_driver *mdrv)
{
	vop_unregister_device(mdrv->vpdev);
	imx_mic_intr_uninit(mdrv);
}

int imx_mic_probe(struct imx_mic_epf *mic_epf)
{
	struct imx_mic_driver *mdrv;
	struct imx_mic_device *mdev;
	int ret;

	mdrv = devm_kzalloc(mic_epf->dev, sizeof(*mdrv), GFP_KERNEL);
	if (!mdrv)
		return -ENOMEM;
	g_mdrv = mdrv;
	mdev = &mdrv->mdev;

	mdrv->mepf = mic_epf;
	mdrv->dev = mic_epf->dev;
	snprintf(mdrv->name, sizeof(imx_mic_driver_name), imx_mic_driver_name);

	/* fetch reserved memory from rc side */
	mdev->mmio.pa = mic_epf->aper.rc_shmem_pa;
	mdev->mmio.va = mic_epf->aper.pci_va;
	mdev->mmio.len = mic_epf->aper.pci_len;

	ret = imx_mic_driver_init(mdrv);
	if (ret) {
		dev_err(mic_epf->dev, "imx_mic_driver_init failed\n");
		return ret;
	}

	mdrv->vpdev->dev.dma_pfn_offset = (SWIOTLB_REGION_ADDR - mic_epf->aper.rc_map_addr) >> PAGE_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_mic_probe);

void imx_mic_remove(void)
{
	struct imx_mic_driver *mdrv = g_mdrv;

	imx_mic_driver_uninit(mdrv);
}
EXPORT_SYMBOL_GPL(imx_mic_remove);
