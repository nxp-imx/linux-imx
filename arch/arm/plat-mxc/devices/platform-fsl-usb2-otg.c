/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on Uwe Kleine-Koenig's platform-fsl-usb2-udc.c
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_fsl_usb2_otg_data_entry_single(soc)				\
	{								\
		.iobase = soc ## _USB_OTG_BASE_ADDR,			\
		.irq = soc ## _INT_USB_OTG,				\
	}


#ifdef CONFIG_SOC_IMX6Q
const struct imx_fsl_usb2_otg_data imx6q_fsl_usb2_otg_data __initconst =
	imx_fsl_usb2_otg_data_entry_single(MX6Q);
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_fsl_usb2_otg(
		const struct imx_fsl_usb2_otg_data *data,
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
	const char *name = "fsl-usb2-otg";
	int id = -1;
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
EXPORT_SYMBOL(imx_add_fsl_usb2_otg);
