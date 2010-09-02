/*
 * Freescale MXS SPI master driver
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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

#ifndef __SPI_STMP_H
#define __SPI_STMP_H

#include <mach/dma.h>

struct mxs_spi {
	void __iomem *regs;	/* vaddr of the control registers */

	u32 irq_dma;
	u32 irq_err;
	u32 dma;
	struct mxs_dma_desc *pdesc;

	u32 speed_khz;
	u32 saved_timings;
	u32 divider;

	struct clk *clk;
	struct device *master_dev;

	struct work_struct work;
	struct workqueue_struct *workqueue;
	spinlock_t lock;
	struct list_head queue;

	u32 ver_major;

	struct completion done;
};

#endif /* __SPI_STMP_H */
