/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
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

#ifndef __MACH_IPU_V3_H_
#define __MACH_IPU_V3_H_

struct ipuv3_fb_platform_data {
	const struct fb_videomode	*modes;
	int				num_modes;
	char				*mode_str;
	u32				interface_pix_fmt;

	/* reserved mem */
	resource_size_t 		res_base;
	resource_size_t 		res_size;
};

struct imx_ipuv3_platform_data {
	int rev;
	int (*init) (void);
	void (*pg) (int);
	struct ipuv3_fb_platform_data	*fb_head0_platform_data;
	struct ipuv3_fb_platform_data	*fb_head1_platform_data;
#define MXC_PRI_DI0	1
#define MXC_PRI_DI1	2
	int primary_di;
	struct clk  *csi_clk[2];
};

#endif /* __MACH_IPU_V3_H_ */
