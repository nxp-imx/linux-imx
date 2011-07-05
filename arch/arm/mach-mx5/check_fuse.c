/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/io.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <mach/check_fuse.h>

int mxc_fuse_get_gpu_status(void)
{
	void __iomem *reg_base = NULL;
	u32 reg_val = 0;
	int bit_status = 0;

	if (cpu_is_mx53() || cpu_is_mx51()) {
		reg_base = IO_ADDRESS(IIM_BASE_ADDR);
		reg_val = readl(reg_base + MXC_IIM_MX5_DISABLERS_OFFSET);
		bit_status = (reg_val & MXC_IIM_MX5_DISABLERS_GPU_MASK)
				>> MXC_IIM_MX5_DISABLERS_GPU_SHIFT;
	} else if (cpu_is_mx50()) {
		reg_base = ioremap(OCOTP_CTRL_BASE_ADDR, SZ_8K);
		reg_val = readl(reg_base + FSL_OCOTP_MX5_CFG2_OFFSET);
		bit_status = (reg_val & FSL_OCOTP_MX5_DISABLERS_GPU_MASK)
				>> FSL_OCOTP_MX5_DISABLERS_GPU_SHIFT;
	}

	return (1 == bit_status);
}
EXPORT_SYMBOL(mxc_fuse_get_gpu_status);

int mxc_fuse_get_vpu_status(void)
{
	void __iomem *reg_base = NULL;
	u32 reg_val = 0;
	int bit_status = 0;

	if (cpu_is_mx53()) {
		reg_base = IO_ADDRESS(IIM_BASE_ADDR);
		reg_val = readl(reg_base + MXC_IIM_MX5_DISABLERS_OFFSET);
		bit_status = (reg_val & MXC_IIM_MX5_DISABLERS_VPU_MASK)
				>> MXC_IIM_MX5_DISABLERS_VPU_SHIFT;
	}

	return (1 == bit_status);
}
EXPORT_SYMBOL(mxc_fuse_get_vpu_status);

