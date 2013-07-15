/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARCH_IMX_PCIE_H
#define __ASM_ARCH_IMX_PCIE_H

/**
 * struct imx_pcie_platform_data - optional platform data for pcie on i.MX
 *
 * @pcie_pwr_en:	used for enable/disable pcie power (-EINVAL if unused)
 * @pcie_rst:		used for reset pcie ep (-EINVAL if unused)
 * @pcie_wake_up:	used for wake up (-EINVAL if unused)
 * @pcie_dis:		used for disable pcie ep (-EINVAL if unused)
 */

struct imx_pcie_platform_data {
	unsigned int pcie_pwr_en;
	unsigned int pcie_rst;
	unsigned int pcie_wake_up;
	unsigned int pcie_dis;
	unsigned int type_ep; /* 1 EP, 0 RC */
	unsigned int pcie_power_always_on;
};
#endif /* __ASM_ARCH_IMX_PCIE_H */
