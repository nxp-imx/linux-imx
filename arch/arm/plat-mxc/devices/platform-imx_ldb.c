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
#include <linux/clk.h>

#define imx_ldb_data_entry_single(soc, size)	\
	{								\
		.iobase = soc ## _IOMUXC_BASE_ADDR,			\
		.iosize = size,						\
	}

#ifdef CONFIG_SOC_IMX53
const struct imx_ldb_data imx53_ldb_data __initconst =
			imx_ldb_data_entry_single(MX53, SZ_4K);
#endif

#ifdef CONFIG_SOC_IMX6Q
const struct imx_ldb_data imx6q_ldb_data __initconst =
			imx_ldb_data_entry_single(MX6Q, SZ_4K);
#endif

struct platform_device *__init imx_add_ldb(
		const struct imx_ldb_data *data,
		struct fsl_mxc_ldb_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		},
	};

	return imx_add_platform_device("mxc_ldb", -1,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

