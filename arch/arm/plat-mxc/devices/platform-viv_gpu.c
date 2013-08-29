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

#include <mach/hardware.h>
#include <mach/devices-common.h>

#ifdef CONFIG_ARCH_MX6
#ifdef CONFIG_SOC_IMX6SL
const struct imx_viv_gpu_data imx6_gpu_data __initconst = {
	.phys_baseaddr = MX6SL_MMDC0_ARB_BASE_ADDR,
	.iobase_3d = 0,
	.irq_3d = -1,
	.iobase_2d = MX6SL_GPU_2D_ARB_BASE_ADDR,
	.irq_2d = MXC_INT_GPU2D_IRQ,
	.iobase_vg = OPENVG_ARB_BASE_ADDR,
	.irq_vg = MXC_INT_OPENVG_XAQ2,
};
#else
const struct imx_viv_gpu_data imx6_gpu_data __initconst = {
	.phys_baseaddr = MMDC0_ARB_BASE_ADDR,
	.iobase_3d = GPU_3D_ARB_BASE_ADDR,
	.irq_3d = MXC_INT_GPU3D_IRQ,
	.iobase_2d = GPU_2D_ARB_BASE_ADDR,
	.irq_2d = MXC_INT_GPU2D_IRQ,
	.iobase_vg = OPENVG_ARB_BASE_ADDR,
	.irq_vg = MXC_INT_OPENVG_XAQ2,
};
#endif
#endif

struct platform_device *__init imx_add_viv_gpu(
		const struct imx_viv_gpu_data *data,
		const struct viv_gpu_platform_data *pdata)
{
	u32 res_count = 0;
	struct resource res[] = {
		{
			.name = "phys_baseaddr",
			.start = data->phys_baseaddr,
			.end = data->phys_baseaddr,
			.flags = IORESOURCE_MEM,
		}, {

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

	res_count = ARRAY_SIZE(res);
	BUG_ON(!res_count);

	if (!fuse_dev_is_available(MXC_DEV_3D)) {
		res[1].start = 0;
		res[1].end = 0;
		res[2].start = -1;
		res[2].end = -1;
	}

	if (!fuse_dev_is_available(MXC_DEV_2D)) {
		res[3].start = 0;
		res[3].end = 0;
		res[4].start = -1;
		res[4].end = -1;
	}

	if (!fuse_dev_is_available(MXC_DEV_OVG)) {
		res[5].start = 0;
		res[5].end = 0;
		res[6].start = -1;
		res[6].end = -1;
	}

	/* None GPU core exists */
	if ((res[2].start == -1) &&
			(res[4].start == -1) &&
			(res[6].start == -1))
		return ERR_PTR(-ENODEV);

	return imx_add_platform_device_dmamask("galcore", 0,
			res, res_count,
			pdata, sizeof(*pdata),
			DMA_BIT_MASK(32));
}
