/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#ifdef CONFIG_SOC_IMX53
struct platform_device *__init imx_add_mlb(
		const struct mxc_mlb_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = MX53_MLB_BASE_ADDR,
			.end = MX53_MLB_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		{
			.start = MX53_INT_MLB,
			.end = MX53_INT_MLB,
			.flags = IORESOURCE_IRQ,
		},
	};
	return imx_add_platform_device("mxc_mlb", 0,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}
#endif /* ifdef CONFIG_SOC_IMX53 */

#ifdef CONFIG_SOC_IMX6Q

struct platform_device *__init imx_add_mlb(
		const struct mxc_mlb_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = MLB_BASE_ADDR,
			.end = MLB_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		{
			.start = MXC_INT_MLB,
			.end = MXC_INT_MLB,
			.flags = IORESOURCE_IRQ,
		},
		{
			.start = MXC_INT_MLB_AHB0,
			.end = MXC_INT_MLB_AHB0,
			.flags = IORESOURCE_IRQ,
		},
		{
			.start = MXC_INT_MLB_AHB1,
			.end = MXC_INT_MLB_AHB1,
			.flags = IORESOURCE_IRQ,
		},
	};

	if (!fuse_dev_is_available(MXC_DEV_MLB))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device("mxc_mlb150", 0,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}
#endif
