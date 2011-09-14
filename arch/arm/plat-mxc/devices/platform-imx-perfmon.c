/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/fsl_devices.h>

#ifdef CONFIG_SOC_IMX50
static struct mxs_perfmon_bit_config
mx50_perfmon_bit_config[] = {
	{.field = (1 << 0),	.name = "MID0-CORE" },
	{.field = (1 << 1),	.name = "MID1-DCP" },
	{.field = (1 << 2),	.name = "MID2-PXP" },
	{.field = (1 << 3),	.name = "MID3-USB" },
	{.field = (1 << 4),	.name = "MID4-GPU2D" },
	{.field = (1 << 5),	.name = "MID5-BCH" },
	{.field = (1 << 6),	.name = "MID6-AHB" },
	{.field = (1 << 7),	.name = "MID7-EPDC" },
	{.field = (1 << 8),	.name = "MID8-LCDIF" },
	{.field = (1 << 9),	.name = "MID9-SDMA" },
	{.field = (1 << 10),	.name = "MID10-FEC" },
	{.field = (1 << 11),	.name = "MID11-MSHC" }
};

struct mxs_platform_perfmon_data mxc_perfmon_data = {
	.bit_config_tab = mx50_perfmon_bit_config,
	.bit_config_cnt = ARRAY_SIZE(mx50_perfmon_bit_config),
};


const struct imx_perfmon_data imx50_perfmon_data = {
	.iobase = MX50_PERFMON_BASE_ADDR,
	.pdata = &mxc_perfmon_data,
};
#endif


struct platform_device *__init imx_add_perfmon(
		const struct imx_perfmon_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}
	};

	return imx_add_platform_device("mxs-perfmon", 0,
			res, ARRAY_SIZE(res), data->pdata,
			sizeof(struct mxs_platform_perfmon_data));
}

