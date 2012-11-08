/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_pm_imx_data_entry_single(soc)		\
	{					\
	}

#ifdef CONFIG_SOC_IMX6Q
const struct imx_pm_imx_data imx6q_pm_imx_data[] __initconst =
	imx_pm_imx_data_entry_single(MX6Q);
#endif

#ifdef CONFIG_SOC_IMX6SL
const struct imx_pm_imx_data imx6sl_pm_imx_data[] __initconst =
	imx_pm_imx_data_entry_single(MX6SL);
#endif

struct platform_device *__init imx_add_pm_imx(
		const struct imx_pm_imx_data *data,
		const struct pm_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
	};

	return imx_add_platform_device("imx_pm", 0,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}
