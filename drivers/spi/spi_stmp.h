/*
 * Freescale STMP378X SPI master driver
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
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
#ifndef __SPI_STMP_H
#define __SPI_STMP_H

#include <mach/dma.h>

/* These two come from arch/arm/mach-xxxxx/spi.c */
int stmp37xx_spi_pins_request(char *id, int ssp);
void stmp37xx_spi_pins_release(char *id, int ssp);

struct stmp_spi {
	int		id;

	void __iomem	*regs;	/* vaddr of the control registers */

	u32		irq;
	u32		dma;
	struct stmp3xxx_dma_descriptor d;

	u32		speed_khz;
	u32		saved_timings;
	u32		divider;

	struct clk	*clk;
	struct device	*master_dev;

	struct work_struct work;
	struct workqueue_struct *workqueue;
	spinlock_t lock;
	struct list_head queue;

	struct completion done;
};

#endif /* __SPI_STMP_H */
