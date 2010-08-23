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

#ifndef __ASM_ARM_MACH_MX28EVK_H
#define __ASM_ARM_MACH_MX28EVK_H

extern void __init mx28evk_pins_init(void);
extern int mx28evk_enet_gpio_init(void);
void mx28evk_enet_io_lowerpower_enter(void);
void mx28evk_enet_io_lowerpower_exit(void);


#endif /* __ASM_ARM_MACH_MX28EVK_H */
