/*
 * Copyright (C) 2013 Boundary Devices, Inc. All Rights Reserved.
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

extern void imx_pcie_enable_irq(unsigned pos, int set);
void imx_pcie_mask_irq(unsigned pos, int set);
unsigned imx_pcie_msi_pending(unsigned index);

#define MSI_MATCH_ADDR  0x01FF8000

void imx_msi_init(void);
