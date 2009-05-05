/*
 * Freescale STMP378X I2C low-level/dma functions
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _ARM_ARCH_I2C_H
#define _ARM_ARCH_I2C_H

#include <linux/device.h>
#include <linux/module.h>

#include <linux/completion.h>
#include <linux/i2c.h>

#define I2C_READ   1
#define I2C_WRITE  0

void hw_i2c_clear_dma_interrupt(void);
int hw_i2c_init(struct device *dev);
void hw_i2c_stop(struct device *dev);
void hw_i2c_setup_write(u8 addr, void *buff, int len, int flags);
void hw_i2c_setup_read(u8 addr, void *buff, int len, int flags);
void hw_i2c_run(int dir);
void hw_i2c_reset_dma(void);
void hw_i2c_finish_read(void *buff, int len);

struct stmp378x_i2c_dev {
	struct device		*dev;
	int			irq_dma;
	int			irq_err;
	struct completion	cmd_complete;
	u32			cmd_err;
	struct i2c_adapter	adapter;
};

#endif
