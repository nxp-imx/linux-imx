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

#ifndef __ASM_ARCH_MACH_DMA_H__
#define __ASM_ARCH_MACH_DMA_H__

enum {
	MXS_DMA_CHANNEL_AHB_APBH = 0,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI0 = MXS_DMA_CHANNEL_AHB_APBH,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI1,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI2,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI3,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI4,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI5,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI6,
	MXS_DMA_CHANNEL_AHB_APBH_GPMI7,
	MXS_DMA_CHANNEL_AHB_APBH_SSP,
	MXS_MAX_DMA_CHANNELS,
};
#endif
