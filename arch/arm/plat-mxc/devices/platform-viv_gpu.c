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
const struct imx_viv_gpu_data imx6_gc2000_data __initconst = {
	.iobase = GPU_3D_ARB_BASE_ADDR,
	.irq = MXC_INT_GPU3D_IRQ,
};

const struct imx_viv_gpu_data imx6_gc320_data __initconst = {
	.iobase = GPU_2D_ARB_BASE_ADDR,
	.irq = MXC_INT_GPU2D_IRQ,
};

const struct imx_viv_gpu_data imx6_gc355_data __initconst = {
	.iobase = OPENVG_ARB_BASE_ADDR,
	.irq = MXC_INT_OPENVG_XAQ2,
};
#endif

struct platform_device *__init imx_add_viv_gpu(
		const char *name,
		const struct imx_viv_gpu_data *data,
		const struct viv_gpu_platform_data *pdata)
{
	struct resource res[] = {
		{
			.name = "gpu_base",
			.start = data->iobase,
			.end = data->iobase + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "gpu_irq",
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device_dmamask(name, 0,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata),
			DMA_BIT_MASK(32));
}
