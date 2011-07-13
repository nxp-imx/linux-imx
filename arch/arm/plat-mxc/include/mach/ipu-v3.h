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
	char				disp_dev[32];
	u32				interface_pix_fmt;
	char				*mode_str;
	int				default_bpp;
	bool				int_clk;

	/* reserved mem */
	resource_size_t 		res_base;
	resource_size_t 		res_size;
};

struct imx_ipuv3_platform_data {
	int rev;
	int (*init) (int);
	void (*pg) (int);

	char *csi_clk[2];
};

#endif /* __MACH_IPU_V3_H_ */
