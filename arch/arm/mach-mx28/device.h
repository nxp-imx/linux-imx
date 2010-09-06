/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARCH_MACH_DEVICE_H__
#define __ASM_ARCH_MACH_DEVICE_H__

extern struct mxs_sys_timer mx28_timer;

extern void __init mx28_map_io(void);
extern void __init mx28_clock_init(void);
extern void __init mx28_irq_init(void);
extern int __init mx28_pinctrl_init(void);
extern int __init mx28_gpio_init(void);
extern int __init mx28_device_init(void);
extern void __init mx28_init_auart(void);
extern void __init
mx28_set_input_clk(unsigned long, unsigned long, unsigned long, unsigned long);

#endif
