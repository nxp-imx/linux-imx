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

#define imx_dvfs_core_data_entry_single(soc)				\
	{							\
		.iobase = soc ## _DVFSCORE_BASE_ADDR,		\
		.irq = soc ## _INT_GPC1,				\
	}

#ifdef CONFIG_SOC_IMX50
const struct imx_dvfs_core_data imx50_dvfs_core_data __initconst =
			imx_dvfs_core_data_entry_single(MX50);
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX51
const struct imx_dvfs_core_data imx51_dvfs_core_data __initconst =
			imx_dvfs_core_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_dvfs_core_data imx53_dvfs_core_data __initconst =
			imx_dvfs_core_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

#ifdef CONFIG_SOC_IMX6Q
const struct imx_dvfs_core_data imx6q_dvfs_core_data __initconst =
			imx_dvfs_core_data_entry_single(MX6Q);
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_dvfs_core(
		const struct imx_dvfs_core_data *data,
		const struct mxc_dvfs_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + 4 * SZ_16 - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx_dvfscore", 0,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

struct platform_device *__init imx_add_busfreq()
{
	return imx_add_platform_device("imx_busfreq", 0,
			NULL, 0, NULL, 0);
}

