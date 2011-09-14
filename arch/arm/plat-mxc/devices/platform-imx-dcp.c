/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_ocp_data_entry_single(soc)	\
{						\
	.iobase = soc ## _DCP_BASE_ADDR,	\
	.irq1 = soc ## _INT_DCP_CHAN0,		\
	.irq2 = soc ## _INT_DCP_CHAN1_3,	\
}

#ifdef CONFIG_SOC_IMX50
const struct imx_dcp_data imx50_dcp_data __initconst =
	imx_ocp_data_entry_single(MX50);
#endif /* ifdef CONFIG_SOC_IMX50 */

struct platform_device *__init imx_add_dcp(
		const struct imx_dcp_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_8K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq1,
			.end = data->irq1,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = data->irq2,
			.end = data->irq2,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("dcp", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
