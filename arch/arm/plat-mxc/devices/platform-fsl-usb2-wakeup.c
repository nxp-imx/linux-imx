/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>
#define imx_fsl_usb2_wakeup_data_entry_single(soc, _id, hs)			\
	{								\
		.id = _id,						\
		.irq_phy = soc ## _INT_USB_PHY ## _id,		\
		.irq_core = soc ## _INT_USB_ ## hs,				\
	}

#ifdef CONFIG_SOC_IMX6Q
const struct imx_fsl_usb2_wakeup_data imx6q_fsl_otg_wakeup_data __initconst =
	imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 0, OTG);
const struct imx_fsl_usb2_wakeup_data imx6q_fsl_hs_wakeup_data[] __initconst = {
	imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 1, HS1),
	imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 2, HS2),
	imx_fsl_usb2_wakeup_data_entry_single(MX6Q, 3, HS3),
};

const struct imx_fsl_usb2_wakeup_data imx6sl_fsl_hs_wakeup_data[] __initconst = {
	imx_fsl_usb2_wakeup_data_entry_single(MX6SL, 1, HS1),
	imx_fsl_usb2_wakeup_data_entry_single(MX6SL, 2, HS2),
	imx_fsl_usb2_wakeup_data_entry_single(MX6SL, 3, HS3),
};
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_fsl_usb2_wakeup(
		const struct imx_fsl_usb2_wakeup_data *data,
		const struct fsl_usb2_wakeup_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->irq_phy,
			.end = data->irq_phy,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = data->irq_core,
			.end = data->irq_core,
			.flags = IORESOURCE_IRQ,
		},
	};
	return imx_add_platform_device_dmamask("usb-wakeup", data->id,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
EXPORT_SYMBOL(imx_add_fsl_usb2_wakeup);
