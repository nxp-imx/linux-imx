// SPDX-License-Identifier: GPL-2.0
// Copyright 2020 NXP

#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci_ids.h>

#include <linux/pci_regs.h>
#include <linux/imx_mic_epf.h>

#define PCI_DEVICE_ID_IMX_MIC		0xCBED
static struct workqueue_struct *imx_mic_workqueue;

/**
 * struct imx_mic_bar0 - Use bar0 to transfer some required values between EP and RC.
 *
 * @rc_shmem_addr: rc ddr address(include devicepage and rc/tx vring region).
 * @rc_shmem_size: size of rc ddr mapped to ep.
 * @rc_map_addr: the rc pci address which is mapped to ep swiotlb region.
 */
struct imx_mic_bar0 {
	u64 rc_shmem_addr;
	u64 rc_shmem_size;
	u64 rc_map_addr;
};

struct pci_epf_header imx_mic_header = {
	.vendorid	= PCI_VENDOR_ID_FREESCALE,
	.deviceid	= PCI_DEVICE_ID_IMX_MIC,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static void imx_mic_epf_linkup(struct pci_epf *epf)
{
}

static void imx_mic_epf_free_space(struct pci_epf *epf, enum pci_barno bar)
{
	epf->bar[bar].phys_addr = 0;
	epf->bar[bar].size = 0;
	epf->bar[bar].barno = 0;
	epf->bar[bar].flags = 0;
}

static void imx_mic_epf_unbind(struct pci_epf *epf)
{
	struct imx_mic_epf *imx_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	cancel_delayed_work(&imx_epf->bar0_handler);

	imx_mic_remove();

	pci_epc_stop(epc);

	if (imx_epf->reg[BAR_0]) {
		pci_epc_clear_bar(epc, epf->func_no, &epf->bar[BAR_0]);
		pci_epf_free_space(epf, imx_epf->reg[BAR_0], BAR_0);
	}

	pci_epc_clear_bar(epc, epf->func_no, &epf->bar[BAR_2]);
	imx_mic_epf_free_space(epf, BAR_2);
	pci_epc_clear_bar(epc, epf->func_no, &epf->bar[BAR_4]);
	imx_mic_epf_free_space(epf, BAR_4);
}

static int imx_mic_epf_set_outbound(struct imx_mic_epf *imx_epf,
				    struct imx_mic_bar0 *rc_shmem_reg)
{
	struct pci_epf *epf = imx_epf->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	void __iomem *shmem_addr;
	phys_addr_t phys_addr;
	int ret;

	shmem_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, rc_shmem_reg->rc_shmem_size);
	if (!shmem_addr) {
		dev_err(dev, "Failed to allocate address\n");
		return -ENOMEM;
	}

	imx_epf->aper.pci_pa = phys_addr;
	imx_epf->aper.pci_va = shmem_addr;
	imx_epf->aper.pci_len = rc_shmem_reg->rc_shmem_size;
	imx_epf->aper.rc_shmem_pa = rc_shmem_reg->rc_shmem_addr;
	imx_epf->aper.rc_map_addr = rc_shmem_reg->rc_map_addr;

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr,
			       rc_shmem_reg->rc_shmem_addr, rc_shmem_reg->rc_shmem_size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		pci_epc_mem_free_addr(epc, phys_addr, shmem_addr, rc_shmem_reg->rc_shmem_size);
		return -ENOMEM;
	}

	dev_info(dev, "%s: rc map share region to ep (rc->ep: 0x%llx->0x%llx), size 0x%llx", __func__,
		 imx_epf->aper.rc_shmem_pa, imx_epf->aper.pci_pa, imx_epf->aper.pci_len);

	ret = imx_mic_probe(imx_epf);
	if (ret) {
		dev_err(dev, "Failed to probe i.MX MIC\n");
		return ret;
	}

	return 0;
}

static void imx_mic_epf_bar0_handler(struct work_struct *work)
{
	struct imx_mic_epf *imx_epf = container_of(work, struct imx_mic_epf,
						   bar0_handler.work);
	struct imx_mic_bar0 *rc_shmem_reg = imx_epf->reg[BAR_0];
	u64 rc_shmem_addr;

	rc_shmem_addr = rc_shmem_reg->rc_shmem_addr;
	if (!rc_shmem_addr)
		goto reset_handler;

	imx_mic_epf_set_outbound(imx_epf, rc_shmem_reg);

	return;

reset_handler:
	queue_delayed_work(imx_mic_workqueue, &imx_epf->bar0_handler,
			   msecs_to_jiffies(1));
}

static int imx_mic_epf_set_bar(struct pci_epf *epf)
{
	struct imx_mic_epf *imx_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	int ret;

	ret = pci_epc_set_bar(epc, epf->func_no, &epf->bar[BAR_0]);
	if (ret) {
		pci_epf_free_space(epf, imx_epf->reg[BAR_0], BAR_0);
		dev_err(dev, "Failed to set BAR%d\n", BAR_0);
		return ret;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, &epf->bar[BAR_2]);
	if (ret) {
		imx_mic_epf_free_space(epf, BAR_2);
		dev_err(dev, "Failed to set BAR%d\n", BAR_2);
		return ret;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, &epf->bar[BAR_4]);
	if (ret) {
		imx_mic_epf_free_space(epf, BAR_4);
		dev_err(dev, "Failed to set BAR%d\n", BAR_4);
		return ret;
	}

	return 0;
}

static int imx_mic_epf_alloc_space(struct pci_epf *epf)
{
	struct imx_mic_epf *imx_epf = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	const struct pci_epc_features *epc_features;
	size_t bar0_size;
	void *base;
	u32 doorbell_reg_base;
	u32 doorbell_reg_size;

	epc_features = imx_epf->epc_features;

	/* BAR0 is used for rc to transfer share memory address and size to ep */
	bar0_size = sizeof(struct imx_mic_bar0);
	base = pci_epf_alloc_space(epf, bar0_size, BAR_0, epc_features->align);
	if (!base) {
		dev_err(dev, "Failed to allocated bar%d space\n", BAR_0);
		return -ENOMEM;
	}
	imx_epf->reg[BAR_0] = base;

	/* BAR2 is used for mapping ep doorbell register */
	imx_mic_get_doorbell_info(&doorbell_reg_base, &doorbell_reg_size);
	epf->bar[BAR_2].phys_addr = doorbell_reg_base;
	epf->bar[BAR_2].size = doorbell_reg_size;
	epf->bar[BAR_2].barno = BAR_2;
	epf->bar[BAR_2].flags |= PCI_BASE_ADDRESS_MEM_TYPE_32;

	/* BAR4 is used for mapping ep swiotlb region */
	epf->bar[BAR_4].phys_addr = SWIOTLB_REGION_ADDR;
	epf->bar[BAR_4].size = SZ_128M;
	epf->bar[BAR_4].barno = BAR_4;
	epf->bar[BAR_4].flags |= PCI_BASE_ADDRESS_MEM_TYPE_32;

	return 0;
}

static void pci_epf_configure_bar(struct pci_epf *epf,
				  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;

	epf_bar = &epf->bar[BAR_2];
	bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << BAR_2));
	if (bar_fixed_64bit)
		epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
}

static int imx_mic_epf_bind(struct pci_epf *epf)
{
	struct imx_mic_epf *imx_epf = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool msi_capable = true;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		pci_epf_configure_bar(epf, epc_features);
		msi_capable = epc_features->msi_capable;
	}

	imx_epf->epc_features = epc_features;

	ret = pci_epc_write_header(epc, epf->func_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}

	ret = imx_mic_epf_alloc_space(epf);
	if (ret)
		return ret;

	ret = imx_mic_epf_set_bar(epf);
	if (ret)
		return ret;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	queue_delayed_work(imx_mic_workqueue, &imx_epf->bar0_handler,
			   msecs_to_jiffies(1));

	return 0;
}

static int imx_mic_epf_probe(struct pci_epf *epf)
{
	struct imx_mic_epf *imx_epf;
	struct device *dev = &epf->dev;

	imx_epf = devm_kzalloc(dev, sizeof(*imx_epf), GFP_KERNEL);
	if (!imx_epf)
		return -ENOMEM;

	epf->header = &imx_mic_header;
	imx_epf->epf = epf;
	imx_epf->dev = dev;

	INIT_DELAYED_WORK(&imx_epf->bar0_handler, imx_mic_epf_bar0_handler);

	epf_set_drvdata(epf, imx_epf);

	return 0;
}

static struct pci_epf_ops imx_mic_epf_ops = {
	.unbind	= imx_mic_epf_unbind,
	.bind	= imx_mic_epf_bind,
	.linkup = imx_mic_epf_linkup,
};

static const struct pci_epf_device_id imx_mic_epf_ids[] = {
	{
		.name = "imx_mic_epf",
	},
	{},
};

static struct pci_epf_driver imx_epf_driver = {
	.driver.name	= "imx_mic_epf",
	.probe		= imx_mic_epf_probe,
	.id_table	= imx_mic_epf_ids,
	.ops		= &imx_mic_epf_ops,
	.owner		= THIS_MODULE,
};

static int __init imx_mic_epf_init(void)
{
	int ret;

	imx_mic_workqueue = alloc_workqueue("imx_mic",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!imx_mic_workqueue) {
		pr_err("Failed to allocate the imx_mic work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&imx_epf_driver);
	if (ret) {
		pr_err("Failed to register imx mic epf driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(imx_mic_epf_init);

static void __exit imx_mic_epf_exit(void)
{
	pci_epf_unregister_driver(&imx_epf_driver);
}
module_exit(imx_mic_epf_exit);

MODULE_AUTHOR("NXP Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX MIC Endpoint Function");
