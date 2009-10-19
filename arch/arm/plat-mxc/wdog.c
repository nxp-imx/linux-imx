/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file plat-mxc/wdog.c
 * @brief This file contains watchdog timer implementations.
 *
 * This file contains watchdog timer implementations for timer tick.
 *
 * @ingroup WDOG
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/hardware.h>

#define WDOG_WT                 0x8	/* WDOG WT starting bit inside WCR */
#define WCR_WOE_BIT             (1 << 6)
#define WCR_WDA_BIT             (1 << 5)
#define WCR_SRS_BIT             (1 << 4)
#define WCR_WRE_BIT             (1 << 3)
#define WCR_WDE_BIT             (1 << 2)
#define WCR_WDBG_BIT            (1 << 1)
#define WCR_WDZST_BIT           (1 << 0)

/*
 * WatchDog
 */
#define WDOG_WCR	0	/* 16bit watchdog control reg */
#define WDOG_WSR	2	/* 16bit watchdog service reg */
#define WDOG_WRSR	4	/* 16bit watchdog reset status reg */

/*!
 * The base addresses for the WDOG modules
 */
static void __iomem *wdog_base[2] = {
	IO_ADDRESS(WDOG1_BASE_ADDR),
#ifdef WDOG2_BASE_ADDR
	IO_ADDRESS(WDOG2_BASE_ADDR),
#endif
};

void mxc_wd_reset(void)
{
	u16 reg;
	struct clk *clk;

	clk = clk_get(NULL, "wdog_clk");
	clk_enable(clk);

	reg = __raw_readw(wdog_base[0] + WDOG_WCR) & ~WCR_SRS_BIT;
	reg |= WCR_WDE_BIT;
	__raw_writew(reg, wdog_base[0] + WDOG_WCR);
}
