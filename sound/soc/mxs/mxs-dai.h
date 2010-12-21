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

#ifndef _MXS_SAIF_H
#define _MXS_SAIF_H

#include <mach/hardware.h>

/* SAIF clock sources */
#define IMX_SSP_SYS_CLK			0
#define IMX_SSP_SYS_MCLK		1

#define SAIF0 0
#define SAIF1 1

/*private info*/
struct mxs_saif {
	u8 saif_clk;
#define PLAYBACK_SAIF0_CAPTURE_SAIF1 0
#define PLAYBACK_SAIF1_CAPTURE_SAIF0 1
	u16 stream_mapping;
	struct clk *saif_mclk;
};

extern struct snd_soc_dai mxs_saif_dai[];

#endif
