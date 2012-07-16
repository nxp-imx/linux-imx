/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
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

#ifdef CONFIG_SOC_IMX50
#define imx_dcp_data_entry_single(soc)	\
{						\
	.iobase = soc ## _DCP_BASE_ADDR,	\
	.irq1 = soc ## _INT_DCP_CHAN0,		\
	.irq2 = soc ## _INT_DCP_CHAN1_3,	\
}

const struct imx_dcp_data imx50_dcp_data __initconst =
	imx_dcp_data_entry_single(MX50);
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX6SL
#define imx_dcp_data_entry_single(soc)	\
{						\
	.iobase = soc ## _DCP_BASE_ADDR,	\
	.irq1 = soc ## _INT_DCP_CH0,		\
	.irq2 = soc ## _INT_DCP_GEN,		\
}

const struct imx_dcp_data imx6sl_dcp_data __initconst =
	imx_dcp_data_entry_single(MX6SL);
#endif /* ifdef CONFIG_SOC_IMX6SL */

struct platform_device *__init imx_add_dcp(
		const struct imx_dcp_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_8K - 1,
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->irq1,
			.end = data->irq1,
			.flags = IORESOURCE_IRQ,
		},
		{
			.start = data->irq2,
			.end = data->irq2,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device_dmamask("dcp", 0,
			res, ARRAY_SIZE(res), NULL, 0, DMA_BIT_MASK(32));
}
