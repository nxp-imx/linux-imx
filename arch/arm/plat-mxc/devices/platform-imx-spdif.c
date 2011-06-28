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

#define imx5_spdif_data_entry_single(soc)			\
	{							\
		.iobase = soc ## _SPDIF_BASE_ADDR,		\
		.irq = soc ## _INT_SPDIF,			\
	}

#ifdef CONFIG_SOC_IMX50
const struct imx_spdif_data imx50_imx_spdif_data __initconst =
			imx5_spdif_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX51
const struct imx_spdif_data imx51_imx_spdif_data __initconst =
			imx5_spdif_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_spdif_data imx53_imx_spdif_data __initconst =
			imx5_spdif_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

#ifdef CONFIG_SOC_IMX6Q
const struct imx_spdif_data imx6q_imx_spdif_data __initconst =
			imx5_spdif_data_entry_single(MX6Q);
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_spdif(
		const struct imx_spdif_data *data,
		const struct mxc_spdif_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + 0x50,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("mxc_spdif", 0,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

