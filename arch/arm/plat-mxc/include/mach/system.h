/*
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_MXC_SYSTEM_H__
#define __ASM_ARCH_MXC_SYSTEM_H__

#include <mach/hardware.h>
#include <mach/common.h>

extern void mx5_cpu_lp_set(enum mxc_cpu_pwr_mode mode);

void arch_idle(void);

void arch_reset(char mode, const char *cmd);
int mxs_reset_block(void __iomem *hwreg, int just_enable);
#endif /* __ASM_ARCH_MXC_SYSTEM_H__ */
