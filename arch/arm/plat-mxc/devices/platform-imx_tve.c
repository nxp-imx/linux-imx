/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Jason Chen <jason.chen@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx5_tve_data_entry_single(soc)				\
	{							\
		.iobase = soc ## _TVE_BASE_ADDR,		\
		.irq = soc ## _INT_TVE,				\
	}

#ifdef CONFIG_SOC_IMX51
const struct imx_tve_data imx51_tve_data __initconst =
			imx5_tve_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_tve_data imx53_tve_data __initconst =
			imx5_tve_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

struct platform_device *__init imx_add_tve(
		const struct imx_tve_data *data,
		const struct fsl_mxc_tve_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("mxc_tve", -1,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

