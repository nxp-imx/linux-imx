/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define imx_pxp_data_entry_single(soc, size)	\
	{								\
		.iobase = soc ## _EPXP_BASE_ADDR,			\
		.irq = soc ## _INT_EPXP,				\
		.iosize = size,						\
	}

#ifdef CONFIG_SOC_IMX50
const struct imx_pxp_data imx50_pxp_data __initconst =
			imx_pxp_data_entry_single(MX50, SZ_4K);
#endif

#ifdef CONFIG_SOC_IMX6Q
const struct imx_pxp_data imx6dl_pxp_data __initconst =
			imx_pxp_data_entry_single(MX6DL, SZ_16K);
#endif

struct platform_device *__init imx_add_imx_pxp(
		const struct imx_pxp_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	if (!fuse_dev_is_available(MXC_DEV_PXP))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device_dmamask("imx-pxp", -1,
		res, ARRAY_SIZE(res), NULL, 0, DMA_BIT_MASK(32));
}

struct platform_device *__init imx_add_imx_pxp_client()
{
	if (!fuse_dev_is_available(MXC_DEV_PXP))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device("imx-pxp-client", -1,
		NULL, 0, NULL, 0);
}

struct platform_device *__init imx_add_imx_pxp_v4l2()
{
	if (!fuse_dev_is_available(MXC_DEV_PXP))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device_dmamask("pxp-v4l2", -1,
		NULL, 0, NULL, 0, DMA_BIT_MASK(32));
}
