/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define imx_mxc_gpu_entry_3d_2d(soc)					\
	{								\
		.irq_3d = soc ## _INT_GPU,				\
		.irq_2d = soc ## _INT_GPU2_IRQ,				\
		.iobase_3d = soc ## _GPU_BASE_ADDR,			\
		.iobase_2d = soc ## _GPU2D_BASE_ADDR,			\
		.gmem_base = soc ## _GPU_GMEM_BASE_ADDR,		\
		.gmem_size = soc ## _GPU_GMEM_SIZE,			\
		.gmem_reserved_base = 0,				\
		.gmem_reserved_size = SZ_128M,				\
	}

#define imx_mxc_gpu_entry_2d(soc)					\
	{								\
		.irq_2d = soc ## _INT_GPU2_IRQ,				\
		.iobase_2d = soc ## _GPU2D_BASE_ADDR,			\
	}

#ifdef CONFIG_SOC_IMX35
const struct imx_mxc_gpu_data imx35_gpu_data __initconst =
	imx_mxc_gpu_entry_2d(MX35);
#endif

#ifdef CONFIG_SOC_IMX50
const struct imx_mxc_gpu_data imx50_gpu_data __initconst =
	imx_mxc_gpu_entry_2d(MX50);
#endif

#ifdef CONFIG_SOC_IMX51
const struct imx_mxc_gpu_data imx51_gpu_data __initconst =
	imx_mxc_gpu_entry_3d_2d(MX51);
#endif

#ifdef CONFIG_SOC_IMX53
struct imx_mxc_gpu_data imx53_gpu_data =
	imx_mxc_gpu_entry_3d_2d(MX53);
#endif

struct platform_device *__init imx_add_mxc_gpu(
		const struct imx_mxc_gpu_data *data,
		const struct mxc_gpu_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->irq_2d,
			.end = data->irq_2d,
			.name = "gpu_2d_irq",
			.flags = IORESOURCE_IRQ,
		},
		{
			.start = data->irq_3d,
			.end = data->irq_3d,
			.name = "gpu_3d_irq",
			.flags = IORESOURCE_IRQ,
		},
		{
			.start = data->iobase_2d,
			.end = data->iobase_2d + SZ_4K - 1,
			.name = "gpu_2d_registers",
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->iobase_3d,
			.end = data->iobase_3d + SZ_128K - 1,
			.name = "gpu_3d_registers",
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->gmem_base,
			.end = data->gmem_base + data->gmem_size - 1,
			.name = "gpu_graphics_mem",
			.flags = IORESOURCE_MEM,
		},
		{
			.start = data->gmem_reserved_base,
			.end = data->gmem_reserved_base +
					data->gmem_reserved_size - 1,
			.name = "gpu_reserved_mem",
			.flags = IORESOURCE_MEM,
		},
	};

	return imx_add_platform_device_dmamask("mxc_gpu", 0,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
