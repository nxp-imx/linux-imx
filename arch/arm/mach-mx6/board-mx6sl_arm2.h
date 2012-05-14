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

#ifndef _BOARD_MX6SL_ARM2_H
#define _BOARD_MX6SL_ARM2_H
#include <mach/iomux-mx6sl.h>

static iomux_v3_cfg_t mx6sl_arm2_pads[] = {
	/* UART1 */
	MX6SL_PAD_UART1_RXD__UART1_RXD,
	MX6SL_PAD_UART1_TXD__UART1_TXD,
};

#endif
