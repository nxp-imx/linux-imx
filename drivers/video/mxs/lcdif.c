/*
 * Freescale MXS LCDIF low-level routines
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/lcdif.h>
#include <mach/regs-lcdif.h>
#include <mach/system.h>

#define REGS_LCDIF_BASE IO_ADDRESS(LCDIF_PHYS_ADDR)

void mxs_init_lcdif(void)
{
	__raw_writel(BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	/* Reset controller */
	__raw_writel(BM_LCDIF_CTRL_SFTRST, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	udelay(10);

	/* Take controller out of reset */
	__raw_writel(BM_LCDIF_CTRL_SFTRST | BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	/* Setup the bus protocol */
	__raw_writel(BM_LCDIF_CTRL1_MODE86,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	__raw_writel(BM_LCDIF_CTRL1_BUSY_ENABLE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);

	/* Take display out of reset */
	__raw_writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	/* VSYNC is an input by default */
	__raw_writel(BM_LCDIF_VDCTRL0_VSYNC_OEB,
		     REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_SET);

	/* Reset display */
	__raw_writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	udelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);
	udelay(10);
}
EXPORT_SYMBOL(mxs_init_lcdif);

int mxs_lcdif_dma_init(struct device *dev, dma_addr_t phys, int memsize)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	__raw_writel(phys, REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF);
	__raw_writel(phys, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	return 0;
}
EXPORT_SYMBOL(mxs_lcdif_dma_init);

void mxs_lcdif_dma_release(void)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	return;
}
EXPORT_SYMBOL(mxs_lcdif_dma_release);

void mxs_lcdif_run(void)
{
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	__raw_writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
}
EXPORT_SYMBOL(mxs_lcdif_run);

void mxs_lcdif_stop(void)
{
	__raw_writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	udelay(100);
}
EXPORT_SYMBOL(mxs_lcdif_stop);

int mxs_lcdif_pan_display(dma_addr_t addr)
{
	__raw_writel(addr, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	return 0;
}

EXPORT_SYMBOL(mxs_lcdif_pan_display);

static BLOCKING_NOTIFIER_HEAD(lcdif_client_list);

int mxs_lcdif_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lcdif_client_list, nb);
}

EXPORT_SYMBOL(mxs_lcdif_register_client);

void mxs_lcdif_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&lcdif_client_list, nb);
}
EXPORT_SYMBOL(mxs_lcdif_unregister_client);

void mxs_lcdif_notify_clients(unsigned long event,
			      struct mxs_platform_fb_entry *pentry)
{
	blocking_notifier_call_chain(&lcdif_client_list, event, pentry);
}
EXPORT_SYMBOL(mxs_lcdif_notify_clients);
