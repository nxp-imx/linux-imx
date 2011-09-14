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

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_imx_srtc_data_entry_single(soc)				\
	{								\
		.iobase = soc ## _SRTC_BASE_ADDR,			\
		.irq = soc ## _INT_SRTC_NTZ,				\
	}

#ifdef CONFIG_SOC_IMX50
const struct imx_srtc_data imx50_imx_srtc_data __initconst =
	imx_imx_srtc_data_entry_single(MX50);
#endif /* ifdef CONFIG_SOC_IMX50 */

#ifdef CONFIG_SOC_IMX51
const struct imx_srtc_data imx51_imx_srtc_data __initconst =
	imx_imx_srtc_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_srtc_data imx53_imx_srtc_data __initconst =
	imx_imx_srtc_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

struct platform_device *__init imx_add_srtc(
		const struct imx_srtc_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("mxc_rtc", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
