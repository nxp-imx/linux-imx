/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 */

/*
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

#define imx_hdmi_soc_data_entry_single(soc)				\
	{								\
		.irq = soc ## _INT_HDMI_TX,				\
	}

#ifdef CONFIG_SOC_IMX6Q
const struct imx_hdmi_soc_data imx6q_imx_hdmi_soc_dai_data __initconst =
			imx_hdmi_soc_data_entry_single(MX6Q);
#endif

struct platform_device *__init imx_add_hdmi_soc_dai(
		const struct imx_hdmi_soc_data *data)
{
	struct resource res[] = {
		{
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	if (!fuse_dev_is_available(MXC_DEV_HDMI))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device("imx-hdmi-soc-dai", 0,
				res, ARRAY_SIZE(res), NULL, 0);
}
