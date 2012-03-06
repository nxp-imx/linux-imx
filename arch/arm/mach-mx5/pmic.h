/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MACH_PMIC_H__
#define __ASM_ARCH_MACH_PMIC_H__

extern int __init mx53_loco_init_da9052(void);
extern int __init mx53_loco_init_mc34708(void);
extern int da9053_suspend_cmd_sw(void);
extern int da9053_suspend_cmd_hw(void);
extern int da9053_restore_volt_settings(void);
extern void pm_i2c_init(u32 base_addr);
extern int pm_i2c_imx_xfer(struct i2c_msg *msgs, int num);

#endif
