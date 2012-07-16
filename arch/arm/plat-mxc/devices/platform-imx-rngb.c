/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#define imx_rngb_data_entry_single(soc)	\
{						\
	.iobase = soc ## _RNGB_BASE_ADDR,	\
	.irq = soc ## _INT_RNGB,		\
}

#ifdef CONFIG_SOC_IMX50
const struct imx_rngb_data imx50_rngb_data __initconst =
	imx_rngb_data_entry_single(MX50);
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX6SL
const struct imx_rngb_data imx6sl_rngb_data __initconst =
	imx_rngb_data_entry_single(MX6SL);
#endif /* ifdef CONFIG_SOC_IMX6SL */

struct platform_device *__init imx_add_rngb(
		const struct imx_rngb_data *data)
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

	return imx_add_platform_device("fsl_rngc", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
