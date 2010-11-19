/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __IMX_IIM_H__
#define __IMX_IIM_H__

#include <linux/mutex.h>
#include <linux/cdev.h>

/* IIM Control Registers */
struct iim_regs {
	u32 stat;
	u32 statm;
	u32 err;
	u32 emask;
	u32 fctl;
	u32 ua;
	u32 la;
	u32 sdat;
	u32 prev;
	u32 srev;
	u32 preg_p;
	u32 scs0;
	u32 scs1;
	u32 scs2;
	u32 scs3;
};

#define IIM_STAT_BUSY   (1 << 7)
#define IIM_STAT_PRGD   (1 << 1)
#define IIM_STAT_SNSD   (1 << 0)

#define IIM_ERR_PRGE    (1 << 7)
#define IIM_ERR_WPE     (1 << 6)
#define IIM_ERR_OPE     (1 << 5)
#define IIM_ERR_RPE     (1 << 4)
#define IIM_ERR_WLRE    (1 << 3)
#define IIM_ERR_SNSE    (1 << 2)
#define IIM_ERR_PARITYE (1 << 1)

#define IIM_PROD_REV_SH         3
#define IIM_PROD_REV_LEN        5
#define IIM_SREV_REV_SH         4
#define IIM_SREV_REV_LEN        4
#define PROD_SIGNATURE_MX51     0x1

#define IIM_ERR_SHIFT       8
#define POLL_FUSE_PRGD      (IIM_STAT_PRGD | (IIM_ERR_PRGE << IIM_ERR_SHIFT))
#define POLL_FUSE_SNSD      (IIM_STAT_SNSD | (IIM_ERR_SNSE << IIM_ERR_SHIFT))

struct mxc_iim_data {
	const s8	*name;
	u32	virt_base;
	u32	reg_base;
	u32	reg_end;
	u32	reg_size;
	u32	bank_start;
	u32	bank_end;
	u32	irq;
	u32	action;
	struct mutex mutex;
	struct completion completion;
	spinlock_t lock;
	struct clk *clk;
	struct device *dev;
	void   (*enable_fuse)(void);
	void   (*disable_fuse)(void);
};

#endif
