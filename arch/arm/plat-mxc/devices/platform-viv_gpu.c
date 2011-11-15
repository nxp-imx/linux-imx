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

#ifdef CONFIG_ARCH_MX6
const struct imx_viv_gpu_data imx6_gpu_data __initconst = {
	.iobase_3d = GPU_3D_ARB_BASE_ADDR,
	.irq_3d = MXC_INT_GPU3D_IRQ,
	.iobase_2d = GPU_2D_ARB_BASE_ADDR,
	.irq_2d = MXC_INT_GPU2D_IRQ,
	.iobase_vg = OPENVG_ARB_BASE_ADDR,
	.irq_vg = MXC_INT_OPENVG_XAQ2,
};
#endif

struct platform_device *__init imx_add_viv_gpu(
		const struct imx_viv_gpu_data *data,
		const struct viv_gpu_platform_data *pdata)
{
	struct resource res[] = {
		{
			.name = "iobase_3d",
			.start = data->iobase_3d,
			.end = data->iobase_3d + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "irq_3d",
			.start = data->irq_3d,
			.end = data->irq_3d,
			.flags = IORESOURCE_IRQ,
		}, {
			.name = "iobase_2d",
			.start = data->iobase_2d,
			.end = data->iobase_2d + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "irq_2d",
			.start = data->irq_2d,
			.end = data->irq_2d,
			.flags = IORESOURCE_IRQ,
		}, {
			.name = "iobase_vg",
			.start = data->iobase_vg,
			.end = data->iobase_vg + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "irq_vg",
			.start = data->irq_vg,
			.end = data->irq_vg,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device_dmamask("galcore", 0,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata),
			DMA_BIT_MASK(32));
}
