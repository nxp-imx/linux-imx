/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __MACH_ASRC_H
#define __MACH_ASRC_H

struct imx_asrc_platform_data {
	struct clk *asrc_core_clk;
	struct clk *asrc_audio_clk;
	unsigned int channel_bits;
	int clk_map_ver;
};

#endif /* __MACH_ASRC_H */

