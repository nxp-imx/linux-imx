/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __MXC_I2C_SLAVE_H__
#define __MXC_I2C_SLAVE_H__

#include <linux/clk.h>
#include "i2c_slave_device.h"

#define MXC_I2C_SLAVE_NAME "MXC_I2C_SLAVE"
#define MXC_I2C_SLAVE_ADDRESS 0x55
#define MXC_I2C_SLAVE_FREQ 1000*100

struct mxc_i2c_slave_device {
	/*!
	 * The default clock divider value to be used.
	 */
	unsigned int clkdiv;

	/*!
	 * The clock source for the device.
	 */
	struct clk *clk;

	/*i2c id on soc */
	int id;

	int irq;
	unsigned long reg_base;
	bool state;		/*0:stop, 1:start */
	i2c_slave_device_t *dev;
};

#endif
