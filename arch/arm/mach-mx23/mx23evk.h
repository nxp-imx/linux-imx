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

#ifndef __ASM_ARM_MACH_MX23EVK_H
#define __ASM_ARM_MACH_MX23EVK_H

extern void __init mx23evk_pins_init(void);
extern void  mx23evk_mma7450_pin_init(void);
extern int mx23evk_mma7450_pin_release(void);
extern int mxs_spi_enc_pin_init(void);
extern int mxs_spi_enc_pin_release(void);
extern int mxs_mmc_get_wp_mmc0(void);
extern int mxs_mmc_hw_init_mmc0(void);
extern void mxs_mmc_hw_release_mmc0(void);
extern void mxs_mmc_cmd_pullup_mmc0(int enable);

#endif /* __ASM_ARM_MACH_MX23EVK_H */
