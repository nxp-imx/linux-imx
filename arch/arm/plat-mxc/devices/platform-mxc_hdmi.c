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

#define imx6_mxc_hdmi_data_entry_single(soc, size)	\
	{								\
		.iobase = soc ## _HDMI_ARB_BASE_ADDR,			\
		.irq = soc ## _INT_HDMI_TX,				\
		.irq_wakeup = soc ## _INT_HDMI_TX_WAKEUP,		\
		.iosize = size,						\
	}

#ifdef CONFIG_SOC_IMX6Q
const struct imx_mxc_hdmi_data imx6q_mxc_hdmi_data __initconst =
			imx6_mxc_hdmi_data_entry_single(MX6Q, SZ_32K + SZ_4K);
#endif

struct platform_device *__init imx_add_mxc_hdmi(
		const struct imx_mxc_hdmi_data *data,
		const struct fsl_mxc_lcd_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device_dmamask("mxc_hdmi", -1,
		res, ARRAY_SIZE(res), pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}

