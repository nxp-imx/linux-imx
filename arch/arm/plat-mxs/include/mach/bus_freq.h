/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef BUS_FREQ_H__
#define BUS_FREQ_H__

#define VERY_HI_RATE		2000000000
#define LCD_ON_CPU_FREQ_KHZ 261818
#define OPERATION_WP_SUPPORTED	6

struct profile {
	int cpu;
	int ahb;
	int emi;
	int ss;
	int vddd;
	int vddd_bo;
	int cur;
	int vddio;
	int vdda;
	u16 xbus;
	/* map of the upper 16 bits of HW_CLKCTRL_HBUS register */
	u16 h_autoslow_flags;
};

/* map of the upper 16 bits of HW_CLKCTRL_HBUS register */
int is_hclk_autoslow_ok(void);

extern int timing_ctrl_rams(int ss);

#endif
