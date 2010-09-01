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

#ifndef _I2C_H
#define _I2C_H

#define I2C_READ   1
#define I2C_WRITE  0

struct mxs_i2c_dev {
	struct device		*dev;
	void			*buf;
	unsigned long		regbase;
	u32			flags;
#define	MXS_I2C_DMA_MODE	0x1
#define	MXS_I2C_PIOQUEUE_MODE	0x2
	int			dma_chan;
	int			irq_dma;
	int			irq_err;
	struct completion	cmd_complete;
	u32			cmd_err;
	struct i2c_adapter	adapter;
	spinlock_t		lock;
	wait_queue_head_t	queue;
	struct work_struct 	work;
};
#endif
