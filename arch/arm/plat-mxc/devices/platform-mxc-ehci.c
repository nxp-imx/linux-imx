/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>
#define imx_mxc_ehci_data_entry_single(soc, _id, hs)			\
	{								\
		.id = _id,						\
		.iobase = soc ## _USB_ ## hs ## _BASE_ADDR,		\
		.irq = soc ## _INT_USB_ ## hs,				\
	}

#ifdef CONFIG_SOC_IMX25
const struct imx_mxc_ehci_data imx25_mxc_ehci_otg_data __initconst =
	imx_mxc_ehci_data_entry_single(MX25, 0, OTG);
const struct imx_mxc_ehci_data imx25_mxc_ehci_hs_data __initconst =
	imx_mxc_ehci_data_entry_single(MX25, 1, HS);
#endif /* ifdef CONFIG_SOC_IMX25 */

#ifdef CONFIG_SOC_IMX27
const struct imx_mxc_ehci_data imx27_mxc_ehci_otg_data __initconst =
	imx_mxc_ehci_data_entry_single(MX27, 0, OTG);
const struct imx_mxc_ehci_data imx27_mxc_ehci_hs_data[] __initconst = {
	imx_mxc_ehci_data_entry_single(MX27, 1, HS1),
	imx_mxc_ehci_data_entry_single(MX27, 2, HS2),
};
#endif /* ifdef CONFIG_SOC_IMX27 */

#ifdef CONFIG_SOC_IMX31
const struct imx_mxc_ehci_data imx31_mxc_ehci_otg_data __initconst =
	imx_mxc_ehci_data_entry_single(MX31, 0, OTG);
const struct imx_mxc_ehci_data imx31_mxc_ehci_hs_data[] __initconst = {
	imx_mxc_ehci_data_entry_single(MX31, 1, HS1),
	imx_mxc_ehci_data_entry_single(MX31, 2, HS2),
};
#endif /* ifdef CONFIG_SOC_IMX31 */

#ifdef CONFIG_SOC_IMX35
const struct imx_mxc_ehci_data imx35_mxc_ehci_otg_data __initconst =
	imx_mxc_ehci_data_entry_single(MX35, 0, OTG);
const struct imx_mxc_ehci_data imx35_mxc_ehci_hs_data __initconst =
	imx_mxc_ehci_data_entry_single(MX35, 1, HS);
#endif /* ifdef CONFIG_SOC_IMX35 */

#ifdef CONFIG_SOC_IMX6Q
const struct imx_mxc_ehci_data imx6q_mxc_ehci_otg_data __initconst =
	imx_mxc_ehci_data_entry_single(MX6Q, 0, OTG);
const struct imx_mxc_ehci_data imx6q_mxc_ehci_hs_data[] __initconst = {
	imx_mxc_ehci_data_entry_single(MX6Q, 1, HS1),
	imx_mxc_ehci_data_entry_single(MX6Q, 2, HS2),
	imx_mxc_ehci_data_entry_single(MX6Q, 3, HS3),
};

const struct imx_mxc_ehci_data imx6sl_mxc_ehci_hs_data[] __initconst = {
	imx_mxc_ehci_data_entry_single(MX6SL, 1, HS1),
	imx_mxc_ehci_data_entry_single(MX6SL, 2, HS2),
	imx_mxc_ehci_data_entry_single(MX6SL, 3, HS3),
};
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_mxc_ehci(
		const struct imx_mxc_ehci_data *data,
		const struct mxc_usbh_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_512 - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};
	return imx_add_platform_device_dmamask("mxc-ehci", data->id,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
EXPORT_SYMBOL(imx_add_mxc_ehci);

/* FSL internal non-upstream code */
struct platform_device *__init imx_add_fsl_ehci(
		const struct imx_mxc_ehci_data *data,
		const struct fsl_usb2_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_512 - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};
	int ret = -ENOMEM;
	const char *name = "fsl-ehci";
	int id = data->id;
	unsigned int num_resources = ARRAY_SIZE(res);
	size_t size_data = sizeof(*pdata);
	u64 dmamask = DMA_BIT_MASK(32);
	struct platform_device *pdev;

	pdev = platform_device_alloc(name, id);
	if (!pdev)
		goto err;

	if (dmamask) {
		/*
		 * This memory isn't freed when the device is put,
		 * I don't have a nice idea for that though.  Conceptually
		 * dma_mask in struct device should not be a pointer.
		 * See http://thread.gmane.org/gmane.linux.kernel.pci/9081
		 */
		pdev->dev.dma_mask =
			kmalloc(sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
		if (!pdev->dev.dma_mask)
			/* ret is still -ENOMEM; */
			goto err;

		*pdev->dev.dma_mask = dmamask;
		pdev->dev.coherent_dma_mask = dmamask;
	}

	ret = platform_device_add_resources(pdev, res, num_resources);
	if (ret)
		goto err;

    if (data) {
		ret = platform_device_add_data(pdev, pdata, size_data);
		if (ret)
			goto err;
	}

	return pdev;

err:
		if (dmamask)
			kfree(pdev->dev.dma_mask);
		platform_device_put(pdev);
		return ERR_PTR(ret);

}
EXPORT_SYMBOL(imx_add_fsl_ehci);
