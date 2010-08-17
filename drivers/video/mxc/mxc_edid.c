/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_edid.c
 *
 * @brief MXC EDID tools
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/fb.h>

#define EDID_LENGTH 128

static u8 edid[EDID_LENGTH];

int read_edid(struct i2c_adapter *adp,
	      struct fb_var_screeninfo *einfo,
	      int *dvi)
{
	u8 buf0[2] = {0, 0};
	int dat = 0;
	u16 addr = 0x50;
	struct i2c_msg msg[2] = {
		{
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= buf0,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	if (adp == NULL || einfo == NULL)
		return -EINVAL;

	buf0[0] = 0x00;
	memset(&edid, 0, sizeof(edid));
	memset(einfo, 0, sizeof(struct fb_var_screeninfo));
	dat = i2c_transfer(adp, msg, 2);

	/* If 0x50 fails, try 0x37. */
	if (edid[1] == 0x00) {
		msg[0].addr = msg[1].addr = 0x37;
		dat = i2c_transfer(adp, msg, 2);
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	*dvi = 0;
	if ((edid[20] == 0x80) || (edid[20] == 0x88) || (edid[20] == 0))
		*dvi = 1;

	dat = fb_parse_edid(edid, einfo);
	if (dat)
		return -dat;

	/* This is valid for version 1.3 of the EDID */
	if ((edid[18] == 1) && (edid[19] == 3)) {
		einfo->height = edid[21] * 10;
		einfo->width = edid[22] * 10;
	}

	return 0;
}
