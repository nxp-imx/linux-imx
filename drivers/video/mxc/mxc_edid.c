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
#include <linux/i2c.h>

#define EDID_LENGTH 128

/* make sure edid has 256 bytes*/
int read_edid(struct i2c_adapter *adp, unsigned char *edid)
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

	if (adp == NULL)
		return -EINVAL;

	buf0[0] = 0x00;
	dat = i2c_transfer(adp, msg, 2);

	/* If 0x50 fails, try 0x37. */
	if (edid[1] == 0x00) {
		msg[0].addr = msg[1].addr = 0x37;
		dat = i2c_transfer(adp, msg, 2);
		if (dat < 0)
			return dat;
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	/* need read ext block? Only support one more blk now*/
	if (edid[0x7E]) {
		if (edid[0x7E] > 1)
			printk(KERN_WARNING "Edid has %d ext block, \
					but now only support 1 ext blk\n", edid[0x7E]);
		buf0[0] = 0x80;
		msg[1].buf = edid + EDID_LENGTH;
		dat = i2c_transfer(adp, msg, 2);
		if (dat < 0)
			return dat;
	}

	return 0;
}
