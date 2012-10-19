/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ARCH_ARM_MACH_MX5_FUSE_CHECK_H__
#define __ARCH_ARM_MACH_MX5_FUSE_CHECK_H__

#define MXC_IIM_MX5_DISABLERS_OFFSET 0x8
#define MXC_IIM_MX5_DISABLERS_GPU_MASK 0x4
#define MXC_IIM_MX5_DISABLERS_GPU_SHIFT 0x2
#define MXC_IIM_MX5_DISABLERS_VPU_MASK 0x2
#define MXC_IIM_MX5_DISABLERS_VPU_SHIFT 0x1

#define FSL_OCOTP_MX5_CFG2_OFFSET 0x060
#define FSL_OCOTP_MX5_DISABLERS_GPU_MASK 0x2000000
#define FSL_OCOTP_MX5_DISABLERS_GPU_SHIFT 0x19

int mxc_fuse_get_gpu_status(void);
int mxc_fuse_get_vpu_status(void);

#endif
