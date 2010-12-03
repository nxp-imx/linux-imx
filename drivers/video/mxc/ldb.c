/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file mxc_ldb.c
 *
 * @brief This file contains the LDB driver device interface and fops
 * functions.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/ldb.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>

#define LDB_BGREF_RMODE_MASK		0x00008000
#define LDB_BGREF_RMODE_INT		0x00008000
#define LDB_BGREF_RMODE_EXT		0x0

#define LDB_DI1_VS_POL_MASK		0x00000400
#define LDB_DI1_VS_POL_ACT_LOW		0x00000400
#define LDB_DI1_VS_POL_ACT_HIGH		0x0
#define LDB_DI0_VS_POL_MASK		0x00000200
#define LDB_DI0_VS_POL_ACT_LOW		0x00000200
#define LDB_DI0_VS_POL_ACT_HIGH		0x0

#define LDB_BIT_MAP_CH1_MASK		0x00000100
#define LDB_BIT_MAP_CH1_JEIDA		0x00000100
#define LDB_BIT_MAP_CH1_SPWG		0x0
#define LDB_BIT_MAP_CH0_MASK		0x00000040
#define LDB_BIT_MAP_CH0_JEIDA		0x00000040
#define LDB_BIT_MAP_CH0_SPWG		0x0

#define LDB_DATA_WIDTH_CH1_MASK		0x00000080
#define LDB_DATA_WIDTH_CH1_24		0x00000080
#define LDB_DATA_WIDTH_CH1_18		0x0
#define LDB_DATA_WIDTH_CH0_MASK		0x00000020
#define LDB_DATA_WIDTH_CH0_24		0x00000020
#define LDB_DATA_WIDTH_CH0_18		0x0

#define LDB_CH1_MODE_MASK		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI1		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI0		0x00000004
#define LDB_CH1_MODE_DISABLE		0x0
#define LDB_CH0_MODE_MASK		0x00000003
#define LDB_CH0_MODE_EN_TO_DI1		0x00000003
#define LDB_CH0_MODE_EN_TO_DI0		0x00000001
#define LDB_CH0_MODE_DISABLE		0x0

#define LDB_SPLIT_MODE_EN		0x00000010

enum ldb_chan_mode_opt {
	LDB_SIN_DI0 = 0,
	LDB_SIN_DI1 = 1,
	LDB_SEP = 2,
	LDB_DUL_DI0 = 3,
	LDB_DUL_DI1 = 4,
	LDB_SPL_DI0 = 5,
	LDB_SPL_DI1 = 6,
};

static struct ldb_data {
	struct fb_info *fbi[2];
	bool ch_working[2];
	uint32_t chan_mode_opt;
	uint32_t chan_bit_map[2];
	uint32_t bgref_rmode;
	uint32_t base_addr;
	uint32_t *control_reg;
	struct clk *ldb_di_clk[2];
	struct regulator *lvds_bg_reg;
	struct list_head modelist;
} ldb;

static struct device *g_ldb_dev;
static u32 *ldb_reg;
static bool enabled[2];
static int g_chan_mode_opt;
static int g_chan_bit_map[2];
static bool g_enable_ldb;
static bool g_boot_cmd;

DEFINE_SPINLOCK(ldb_lock);

struct fb_videomode mxcfb_ldb_modedb[] = {
	{
	 "1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
};
int mxcfb_ldb_modedb_sz = ARRAY_SIZE(mxcfb_ldb_modedb);

static int bits_per_pixel(int pixel_fmt)
{
	switch (pixel_fmt) {
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 24;
		break;
	case IPU_PIX_FMT_BGR666:
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_LVDS666:
		return 18;
		break;
	default:
		break;
	}
	return 0;
}

static int valid_mode(int pixel_fmt)
{
	return ((pixel_fmt == IPU_PIX_FMT_RGB24) ||
		(pixel_fmt == IPU_PIX_FMT_BGR24) ||
		(pixel_fmt == IPU_PIX_FMT_LVDS666) ||
		(pixel_fmt == IPU_PIX_FMT_RGB666) ||
		(pixel_fmt == IPU_PIX_FMT_BGR666));
}

static void ldb_disable(int ipu_di)
{
	uint32_t reg;
	int i = 0;

	spin_lock(&ldb_lock);

	switch (ldb.chan_mode_opt) {
	case LDB_SIN_DI0:
		if (ipu_di != 0 || !ldb.ch_working[0] || !enabled[0]) {
			spin_unlock(&ldb_lock);
			return;
		}

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
			     LDB_CH0_MODE_DISABLE,
			     ldb.control_reg);

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		clk_disable(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[0]);

		ldb.ch_working[0] = false;
		enabled[0] = false;
		break;
	case LDB_SIN_DI1:
		if (ipu_di != 1 || !ldb.ch_working[1] || !enabled[1]) {
			spin_unlock(&ldb_lock);
			return;
		}

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
			     LDB_CH1_MODE_DISABLE,
			     ldb.control_reg);

		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		clk_disable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[1]);

		ldb.ch_working[1] = false;
		enabled[1] = false;
		break;
	case LDB_SPL_DI0:
	case LDB_DUL_DI0:
		if (ipu_di != 0 || !enabled[0]) {
			spin_unlock(&ldb_lock);
			return;
		}

		for (i = 0; i < 2; i++) {
			if (ldb.ch_working[i]) {
				reg = __raw_readl(ldb.control_reg);
				if (i == 0)
					__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
						     LDB_CH0_MODE_DISABLE,
						     ldb.control_reg);
				else
					__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
						     LDB_CH1_MODE_DISABLE,
						     ldb.control_reg);

				if (ldb.chan_mode_opt == LDB_SPL_DI0) {
					reg = __raw_readl(ldb.control_reg);
					__raw_writel(reg & ~LDB_SPLIT_MODE_EN,
						     ldb.control_reg);
				}

				ldb.ldb_di_clk[i] = clk_get(NULL, i ?
							"ldb_di1_clk" :
							"ldb_di0_clk");
				clk_disable(ldb.ldb_di_clk[i]);
				clk_put(ldb.ldb_di_clk[i]);

				ldb.ch_working[i] = false;
			}
		}
		enabled[0] = false;
		break;
	case LDB_SPL_DI1:
	case LDB_DUL_DI1:
		if (ipu_di != 1 || !enabled[1]) {
			spin_unlock(&ldb_lock);
			return;
		}

		for (i = 0; i < 2; i++) {
			if (ldb.ch_working[i]) {
				reg = __raw_readl(ldb.control_reg);
				if (i == 0)
					__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
						     LDB_CH0_MODE_DISABLE,
						     ldb.control_reg);
				else
					__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
						     LDB_CH1_MODE_DISABLE,
						     ldb.control_reg);

				if (ldb.chan_mode_opt == LDB_SPL_DI1) {
					reg = __raw_readl(ldb.control_reg);
					__raw_writel(reg & ~LDB_SPLIT_MODE_EN,
						     ldb.control_reg);
				}

				ldb.ldb_di_clk[i] = clk_get(NULL, i ?
							"ldb_di1_clk" :
							"ldb_di0_clk");
				clk_disable(ldb.ldb_di_clk[i]);
				clk_put(ldb.ldb_di_clk[i]);

				ldb.ch_working[i] = false;
			}
		}
		enabled[1] = false;
		break;
	case LDB_SEP:
		if (ldb.ch_working[ipu_di] && enabled[ipu_di]) {
			reg = __raw_readl(ldb.control_reg);
			if (ipu_di == 0)
				__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
					     LDB_CH0_MODE_DISABLE,
					     ldb.control_reg);
			else
				__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
					     LDB_CH1_MODE_DISABLE,
					     ldb.control_reg);

			ldb.ldb_di_clk[ipu_di] = clk_get(NULL, ipu_di ?
							       "ldb_di1_clk" :
							       "ldb_di0_clk");
			clk_disable(ldb.ldb_di_clk[ipu_di]);
			clk_put(ldb.ldb_di_clk[ipu_di]);

			ldb.ch_working[ipu_di] = false;
			enabled[ipu_di] = false;
		}
		break;
	default:
		break;
	}

	spin_unlock(&ldb_lock);
	return;
}

static void ldb_enable(int ipu_di)
{
	uint32_t reg;

	spin_lock(&ldb_lock);

	reg = __raw_readl(ldb.control_reg);
	switch (ldb.chan_mode_opt) {
	case LDB_SIN_DI0:
		if (ldb.ch_working[0] || ipu_di != 0 || enabled[0]) {
			spin_unlock(&ldb_lock);
			return;
		}

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		clk_enable(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[0]);
		__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
			      LDB_CH0_MODE_EN_TO_DI0, ldb.control_reg);
		ldb.ch_working[0] = true;
		enabled[0] = true;
		break;
	case LDB_SIN_DI1:
		if (ldb.ch_working[1] || ipu_di != 1 || enabled[1]) {
			spin_unlock(&ldb_lock);
			return;
		}

		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[1]);
		__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
			      LDB_CH1_MODE_EN_TO_DI1, ldb.control_reg);
		ldb.ch_working[1] = true;
		enabled[1] = true;
		break;
	case LDB_SEP:
		if (ldb.ch_working[ipu_di] || enabled[ipu_di]) {
			spin_unlock(&ldb_lock);
			return;
		}

		if (ipu_di == 0) {
			ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
			clk_enable(ldb.ldb_di_clk[0]);
			clk_put(ldb.ldb_di_clk[0]);
			__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
				      LDB_CH0_MODE_EN_TO_DI0,
				      ldb.control_reg);
			ldb.ch_working[0] = true;
		} else {
			ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
			clk_enable(ldb.ldb_di_clk[1]);
			clk_put(ldb.ldb_di_clk[1]);
			__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
				      LDB_CH1_MODE_EN_TO_DI1,
				      ldb.control_reg);
			ldb.ch_working[1] = true;
		}
		enabled[ipu_di] = true;
		break;
	case LDB_DUL_DI0:
	case LDB_SPL_DI0:
		if (ipu_di != 0 || enabled[0])
			return;
		else
			goto proc;
	case LDB_DUL_DI1:
	case LDB_SPL_DI1:
		if (ipu_di != 1 || enabled[1])
			return;
proc:
		if (ldb.ch_working[0] || ldb.ch_working[1]) {
			spin_unlock(&ldb_lock);
			return;
		}

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		clk_enable(ldb.ldb_di_clk[0]);
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[1]);

		if (ldb.chan_mode_opt == LDB_DUL_DI0 ||
		    ldb.chan_mode_opt == LDB_SPL_DI0) {
			__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
				      LDB_CH0_MODE_EN_TO_DI0,
				      ldb.control_reg);
			reg = __raw_readl(ldb.control_reg);
			__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
				      LDB_CH1_MODE_EN_TO_DI0,
				      ldb.control_reg);
		} else if (ldb.chan_mode_opt == LDB_DUL_DI1 ||
			   ldb.chan_mode_opt == LDB_SPL_DI1) {
			__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
				      LDB_CH0_MODE_EN_TO_DI1,
				      ldb.control_reg);
			reg = __raw_readl(ldb.control_reg);
			__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
				      LDB_CH1_MODE_EN_TO_DI1,
				      ldb.control_reg);
		}
		if (ldb.chan_mode_opt == LDB_SPL_DI0 ||
		    ldb.chan_mode_opt == LDB_SPL_DI1) {
			reg = __raw_readl(ldb.control_reg);
			__raw_writel(reg | LDB_SPLIT_MODE_EN,
				      ldb.control_reg);
		}
		ldb.ch_working[0] = true;
		ldb.ch_working[1] = true;
		enabled[ipu_di] = true;
		break;
	default:
		break;
	}
	spin_unlock(&ldb_lock);
	return;
}

int ldb_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	mm_segment_t old_fs;
	int ipu_di = 0;

	switch (val) {
	case FB_EVENT_BLANK:
		if (ldb.fbi[0] != fbi && ldb.fbi[1] != fbi)
			return 0;

		if (fbi->fbops->fb_ioctl) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			fbi->fbops->fb_ioctl(fbi,
					MXCFB_GET_FB_IPU_DI,
					(unsigned long)&ipu_di);
			set_fs(old_fs);
		} else
			return 0;

		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			ldb_enable(ipu_di);
		else
			ldb_disable(ipu_di);
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = ldb_fb_event,
};

static int mxc_ldb_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	uint32_t reg;

	switch (cmd) {
	case LDB_BGREF_RMODE:
		{
		ldb_bgref_parm parm;

		if (copy_from_user(&parm, (ldb_bgref_parm *) arg,
				   sizeof(ldb_bgref_parm)))
			return -EFAULT;

		spin_lock(&ldb_lock);
		reg = __raw_readl(ldb.control_reg);
		if (parm.bgref_mode == LDB_EXT_REF)
			__raw_writel((reg & ~LDB_BGREF_RMODE_MASK) |
				      LDB_BGREF_RMODE_EXT, ldb.control_reg);
		else if (parm.bgref_mode == LDB_INT_REF)
			__raw_writel((reg & ~LDB_BGREF_RMODE_MASK) |
				      LDB_BGREF_RMODE_INT, ldb.control_reg);
		spin_unlock(&ldb_lock);
		break;
		}
	case LDB_VSYNC_POL:
		{
		ldb_vsync_parm parm;

		if (copy_from_user(&parm, (ldb_vsync_parm *) arg,
				   sizeof(ldb_vsync_parm)))
			return -EFAULT;

		spin_lock(&ldb_lock);
		reg = __raw_readl(ldb.control_reg);
		if (parm.vsync_mode == LDB_VS_ACT_H) {
			if (parm.di == 0)
				__raw_writel((reg &
					~LDB_DI0_VS_POL_MASK) |
					LDB_DI0_VS_POL_ACT_HIGH,
					ldb.control_reg);
			else
				__raw_writel((reg &
					~LDB_DI1_VS_POL_MASK) |
					LDB_DI1_VS_POL_ACT_HIGH,
					ldb.control_reg);
		} else if (parm.vsync_mode == LDB_VS_ACT_L) {
			if (parm.di == 0)
				__raw_writel((reg &
					~LDB_DI0_VS_POL_MASK) |
					LDB_DI0_VS_POL_ACT_LOW,
					ldb.control_reg);
			else
				__raw_writel((reg &
					~LDB_DI1_VS_POL_MASK) |
					LDB_DI1_VS_POL_ACT_LOW,
					ldb.control_reg);

		}
		spin_unlock(&ldb_lock);
		break;
		}
	case LDB_BIT_MAP:
		{
		ldb_bitmap_parm parm;

		if (copy_from_user(&parm, (ldb_bitmap_parm *) arg,
				   sizeof(ldb_bitmap_parm)))
			return -EFAULT;

		spin_lock(&ldb_lock);
		reg = __raw_readl(ldb.control_reg);
		if (parm.bitmap_mode == LDB_BIT_MAP_SPWG) {
			if (parm.channel == 0)
				__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
					      LDB_BIT_MAP_CH0_SPWG,
					      ldb.control_reg);
			else
				__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
					      LDB_BIT_MAP_CH1_SPWG,
					      ldb.control_reg);
		} else if (parm.bitmap_mode == LDB_BIT_MAP_JEIDA) {
			if (parm.channel == 0)
				__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
					      LDB_BIT_MAP_CH0_JEIDA,
					      ldb.control_reg);
			else
				__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
					      LDB_BIT_MAP_CH1_JEIDA,
					      ldb.control_reg);
		}
		spin_unlock(&ldb_lock);
		break;
		}
	case LDB_DATA_WIDTH:
		{
		ldb_data_width_parm parm;

		if (copy_from_user(&parm, (ldb_data_width_parm *) arg,
				   sizeof(ldb_data_width_parm)))
			return -EFAULT;

		spin_lock(&ldb_lock);
		reg = __raw_readl(ldb.control_reg);
		if (parm.data_width == 24) {
			if (parm.channel == 0)
				__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
					      LDB_DATA_WIDTH_CH0_24,
					      ldb.control_reg);
			else
				__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
					      LDB_DATA_WIDTH_CH1_24,
					      ldb.control_reg);
		} else if (parm.data_width == 18) {
			if (parm.channel == 0)
				__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
					      LDB_DATA_WIDTH_CH0_18,
					      ldb.control_reg);
			else
				__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
					      LDB_DATA_WIDTH_CH1_18,
					      ldb.control_reg);
		}
		spin_unlock(&ldb_lock);
		break;
		}
	case LDB_CHAN_MODE:
		{
		ldb_chan_mode_parm parm;
		struct clk *pll4_clk;
		unsigned long pll4_rate = 0;

		if (copy_from_user(&parm, (ldb_chan_mode_parm *) arg,
				   sizeof(ldb_chan_mode_parm)))
			return -EFAULT;

		spin_lock(&ldb_lock);

		/* TODO:Set the correct pll4 rate for all situations */
		pll4_clk = clk_get(NULL, "pll4");
		pll4_rate = clk_get_rate(pll4_clk);
		pll4_rate = 455000000;
		clk_set_rate(pll4_clk, pll4_rate);
		clk_put(pll4_clk);

		reg = __raw_readl(ldb.control_reg);
		switch (parm.channel_mode) {
		case LDB_CHAN_MODE_SIN:
			if (parm.di == 0) {
				ldb.chan_mode_opt = LDB_SIN_DI0;

				ldb.ldb_di_clk[0] = clk_get(NULL,
							    "ldb_di0_clk");
				clk_set_rate(ldb.ldb_di_clk[0], pll4_rate/7);
				clk_put(ldb.ldb_di_clk[0]);

				__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
					      LDB_CH0_MODE_EN_TO_DI0,
					      ldb.control_reg);
			} else {
				ldb.chan_mode_opt = LDB_SIN_DI1;

				ldb.ldb_di_clk[1] = clk_get(NULL,
							    "ldb_di1_clk");
				clk_set_rate(ldb.ldb_di_clk[1], pll4_rate/7);
				clk_put(ldb.ldb_di_clk[1]);

				__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
					      LDB_CH1_MODE_EN_TO_DI1,
					      ldb.control_reg);
			}
			break;
		case LDB_CHAN_MODE_SEP:
			ldb.chan_mode_opt = LDB_SEP;

			ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
			clk_set_rate(ldb.ldb_di_clk[0], pll4_rate/7);
			clk_put(ldb.ldb_di_clk[0]);
			ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
			clk_set_rate(ldb.ldb_di_clk[1], pll4_rate/7);
			clk_put(ldb.ldb_di_clk[1]);

			__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
					      LDB_CH1_MODE_MASK)) |
				      LDB_CH0_MODE_EN_TO_DI0 |
				      LDB_CH1_MODE_EN_TO_DI1,
				      ldb.control_reg);
			break;
		case LDB_CHAN_MODE_DUL:
		case LDB_CHAN_MODE_SPL:
			ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
			ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
			if (parm.di == 0) {
				if (parm.channel_mode == LDB_CHAN_MODE_DUL) {
					ldb.chan_mode_opt = LDB_DUL_DI0;
					clk_set_rate(ldb.ldb_di_clk[0],
						     pll4_rate/7);
				} else {
					ldb.chan_mode_opt = LDB_SPL_DI0;
					clk_set_rate(ldb.ldb_di_clk[0],
						     2*pll4_rate/7);
					clk_set_rate(ldb.ldb_di_clk[1],
						     2*pll4_rate/7);
					reg = __raw_readl(ldb.control_reg);
					__raw_writel(reg | LDB_SPLIT_MODE_EN,
						      ldb.control_reg);
				}

				reg = __raw_readl(ldb.control_reg);
				__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
						      LDB_CH1_MODE_MASK)) |
					      LDB_CH0_MODE_EN_TO_DI0 |
					      LDB_CH1_MODE_EN_TO_DI0,
					      ldb.control_reg);
			} else {
				if (parm.channel_mode == LDB_CHAN_MODE_DUL) {
					ldb.chan_mode_opt = LDB_DUL_DI1;
					clk_set_rate(ldb.ldb_di_clk[1],
						     pll4_rate/7);
				} else {
					ldb.chan_mode_opt = LDB_SPL_DI1;
					clk_set_rate(ldb.ldb_di_clk[0],
						     2*pll4_rate/7);
					clk_set_rate(ldb.ldb_di_clk[1],
						     2*pll4_rate/7);
					reg = __raw_readl(ldb.control_reg);
					__raw_writel(reg | LDB_SPLIT_MODE_EN,
						      ldb.control_reg);
				}

				reg = __raw_readl(ldb.control_reg);
				__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
						      LDB_CH1_MODE_MASK)) |
					      LDB_CH0_MODE_EN_TO_DI1 |
					      LDB_CH1_MODE_EN_TO_DI1,
					      ldb.control_reg);
			}
			clk_put(ldb.ldb_di_clk[0]);
			clk_put(ldb.ldb_di_clk[1]);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		spin_unlock(&ldb_lock);
		break;
		}
	case LDB_ENABLE:
		{
		int ipu_di;

		if (copy_from_user(&ipu_di, (int *) arg, sizeof(int)))
			return -EFAULT;

		ldb_enable(ipu_di);
		break;
		}
	case LDB_DISABLE:
		{
		int ipu_di;

		if (copy_from_user(&ipu_di, (int *) arg, sizeof(int)))
			return -EFAULT;

		ldb_disable(ipu_di);
		break;
		}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mxc_ldb_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxc_ldb_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxc_ldb_mmap(struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

static const struct file_operations mxc_ldb_fops = {
	.owner = THIS_MODULE,
	.open = mxc_ldb_open,
	.mmap = mxc_ldb_mmap,
	.release = mxc_ldb_release,
	.ioctl = mxc_ldb_ioctl
};

/*!
 * This function is called by the driver framework to initialize the LDB
 * device.
 *
 * @param	dev	The device structure for the LDB passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int ldb_probe(struct platform_device *pdev)
{
	int ret = 0, i, ipu_di, ipu_di_pix_fmt[2];
	bool primary = false, find_1080p = false;
	struct resource *res;
	struct ldb_platform_data *plat_data = pdev->dev.platform_data;
	mm_segment_t old_fs;
	struct clk *ldb_clk_parent;
	unsigned long ldb_clk_prate = 455000000;
	struct fb_var_screeninfo *var[2];
	uint32_t reg;
	struct device *temp;
	int mxc_ldb_major;
	const struct fb_videomode *mode;
	struct class *mxc_ldb_class;

	if (g_enable_ldb == false)
		return -ENODEV;

	spin_lock_init(&ldb_lock);

	g_ldb_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res))
		return -ENODEV;

	memset(&ldb, 0, sizeof(struct ldb_data));
	enabled[0] = enabled[1] = false;
	var[0] = var[1] = NULL;
	if (g_boot_cmd) {
		ldb.chan_mode_opt = g_chan_mode_opt;
		ldb.chan_bit_map[0] = g_chan_bit_map[0];
		ldb.chan_bit_map[1] = g_chan_bit_map[1];
	}

	ldb.base_addr = res->start;
	ldb_reg = ioremap(ldb.base_addr, res->end - res->start + 1);
	ldb.control_reg = ldb_reg + 2;

	INIT_LIST_HEAD(&ldb.modelist);
	for (i = 0; i < mxcfb_ldb_modedb_sz; i++)
		fb_add_videomode(&mxcfb_ldb_modedb[i], &ldb.modelist);

	for (i = 0; i < num_registered_fb; i++) {
		if (registered_fb[i]->var.vmode == FB_VMODE_NONINTERLACED) {
			mode = fb_match_mode(&registered_fb[i]->var,
						&ldb.modelist);
			if (mode) {
				dev_dbg(g_ldb_dev, "fb mode found\n");
				ldb.fbi[i] = registered_fb[i];
				fb_videomode_to_var(&ldb.fbi[i]->var, mode);
			} else if (i == 0 && ldb.chan_mode_opt != LDB_SEP) {
				continue;
			} else {
				dev_warn(g_ldb_dev,
						"can't find video mode\n");
				goto err0;
			}
			/*
			 * Default ldb mode:
			 * 1080p: DI0 split, SPWG or DI1 split, SPWG
			 * others: single, SPWG
			 */
			if (g_boot_cmd == false) {
				if (fb_mode_is_equal(mode, &mxcfb_ldb_modedb[0])) {
					if (strcmp(ldb.fbi[i]->fix.id,
					    "DISP3 BG") == 0) {
						ldb.chan_mode_opt = LDB_SPL_DI0;
						dev_warn(g_ldb_dev,
							"default di0 split mode\n");
					} else if (strcmp(ldb.fbi[i]->fix.id,
						   "DISP3 BG - DI1") == 0) {
						ldb.chan_mode_opt = LDB_SPL_DI1;
						dev_warn(g_ldb_dev,
							"default di1 split mode\n");
					}
					ldb.chan_bit_map[0] = LDB_BIT_MAP_SPWG;
					ldb.chan_bit_map[1] = LDB_BIT_MAP_SPWG;
					find_1080p = true;
				} else if (!find_1080p) {
					if (strcmp(ldb.fbi[i]->fix.id,
					    "DISP3 BG") == 0) {
						ldb.chan_mode_opt = LDB_SIN_DI0;
						ldb.chan_bit_map[0] = LDB_BIT_MAP_SPWG;
						dev_warn(g_ldb_dev,
							 "default di0 single mode\n");
					} else if (strcmp(ldb.fbi[i]->fix.id,
						   "DISP3 BG - DI1") == 0) {
						ldb.chan_mode_opt = LDB_SIN_DI1;
						ldb.chan_bit_map[1] = LDB_BIT_MAP_SPWG;
						dev_warn(g_ldb_dev,
							 "default di1 single mode\n");
					}
				}
			}

			acquire_console_sem();
			fb_blank(ldb.fbi[i], FB_BLANK_POWERDOWN);
			release_console_sem();

			if (i == 0)
				primary = true;

			if (ldb.fbi[1] != NULL || ldb.chan_mode_opt != LDB_SEP)
				break;
		}
	}

	/*
	 * We cannot support two LVDS panel with different pixel clock rates
	 * except that one's pixel clock rate is two times of the others'.
	 */
	if (ldb.fbi[1] && ldb.fbi[0] != NULL) {
		if (ldb.fbi[0]->var.pixclock != ldb.fbi[1]->var.pixclock &&
		    ldb.fbi[0]->var.pixclock != 2 * ldb.fbi[1]->var.pixclock &&
		    ldb.fbi[1]->var.pixclock != 2 * ldb.fbi[0]->var.pixclock)
			return -EINVAL;
	}

	ldb.bgref_rmode = plat_data->ext_ref;
	ldb.lvds_bg_reg = regulator_get(&pdev->dev, plat_data->lvds_bg_reg);
	if (!IS_ERR(ldb.lvds_bg_reg)) {
		regulator_set_voltage(ldb.lvds_bg_reg, 2500000, 2500000);
		regulator_enable(ldb.lvds_bg_reg);
	}

	for (i = 0; i < 2; i++) {
		if (ldb.fbi[i] != NULL) {
			if (strcmp(ldb.fbi[i]->fix.id, "DISP3 BG") == 0)
				ipu_di = 0;
			else if (strcmp(ldb.fbi[i]->fix.id, "DISP3 BG - DI1")
				 == 0)
				ipu_di = 1;
			else {
				dev_err(g_ldb_dev, "Wrong framebuffer\n");
				goto err0;
			}

			var[ipu_di] = &ldb.fbi[i]->var;
			if (ldb.fbi[i]->fbops->fb_ioctl) {
				old_fs = get_fs();
				set_fs(KERNEL_DS);
				ldb.fbi[i]->fbops->fb_ioctl(ldb.fbi[i],
						MXCFB_GET_DIFMT,
						(unsigned long)&(ipu_di_pix_fmt[ipu_di]));
				set_fs(old_fs);
			} else {
				dev_err(g_ldb_dev, "Can't get framebuffer "
						   "information\n");
				goto err0;
			}

			if (!valid_mode(ipu_di_pix_fmt[ipu_di])) {
				dev_err(g_ldb_dev, "Unsupport pixel format "
						   "for ldb input\n");
				goto err0;
			}

			reg = __raw_readl(ldb.control_reg);
			if (var[ipu_di]->sync & FB_SYNC_VERT_HIGH_ACT) {
				if (ipu_di == 0)
					__raw_writel((reg &
						~LDB_DI0_VS_POL_MASK) |
						LDB_DI0_VS_POL_ACT_HIGH,
						ldb.control_reg);
				else
					__raw_writel((reg &
						~LDB_DI1_VS_POL_MASK) |
						LDB_DI1_VS_POL_ACT_HIGH,
						ldb.control_reg);
			} else {
				if (ipu_di == 0)
					__raw_writel((reg &
						~LDB_DI0_VS_POL_MASK) |
						LDB_DI0_VS_POL_ACT_LOW,
						ldb.control_reg);
				else
					__raw_writel((reg &
						~LDB_DI1_VS_POL_MASK) |
						LDB_DI1_VS_POL_ACT_LOW,
						ldb.control_reg);
			}

			/* TODO:Set the correct pll4 rate for all situations */
			if (ipu_di == 1) {
				ldb.ldb_di_clk[1] =
					clk_get(&pdev->dev, "ldb_di1_clk");
				ldb_clk_parent =
					clk_get_parent(ldb.ldb_di_clk[1]);
				clk_set_rate(ldb_clk_parent, ldb_clk_prate);
				clk_put(ldb.ldb_di_clk[1]);
			} else {
				ldb.ldb_di_clk[0] =
					clk_get(&pdev->dev, "ldb_di0_clk");
				ldb_clk_parent =
					clk_get_parent(ldb.ldb_di_clk[0]);
				clk_set_rate(ldb_clk_parent, ldb_clk_prate);
				clk_put(ldb.ldb_di_clk[0]);
			}
		}
	}

	reg = __raw_readl(ldb.control_reg);
	if (ldb.bgref_rmode == LDB_EXT_REF)
		__raw_writel((reg & ~LDB_BGREF_RMODE_MASK) |
			      LDB_BGREF_RMODE_EXT, ldb.control_reg);
	else
		__raw_writel((reg & ~LDB_BGREF_RMODE_MASK) |
			      LDB_BGREF_RMODE_INT, ldb.control_reg);

	switch (ldb.chan_mode_opt) {
	case LDB_SIN_DI0:
		if (var[0] == NULL) {
			dev_err(g_ldb_dev, "Can't find framebuffer on DI0\n");
			break;
		}

		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[0]) == 24)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
				      LDB_DATA_WIDTH_CH0_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[0]) == 18)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
				      LDB_DATA_WIDTH_CH0_18,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[0] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_JEIDA,
				      ldb.control_reg);

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		clk_set_rate(ldb.ldb_di_clk[0], ldb_clk_prate/7);
		clk_enable(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[0]);

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~LDB_CH0_MODE_MASK) |
			      LDB_CH0_MODE_EN_TO_DI0, ldb.control_reg);
		ldb.ch_working[0] = true;
		break;
	case LDB_SIN_DI1:
		if (var[1] == NULL) {
			dev_err(g_ldb_dev, "Can't find framebuffer on DI1\n");
			break;
		}

		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[1]) == 24)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH1_MASK) |
				      LDB_DATA_WIDTH_CH1_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[1]) == 18)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH1_MASK) |
				      LDB_DATA_WIDTH_CH1_18,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[1] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_JEIDA,
				      ldb.control_reg);

		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		clk_set_rate(ldb.ldb_di_clk[1], ldb_clk_prate/7);
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[1]);

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~LDB_CH1_MODE_MASK) |
			      LDB_CH1_MODE_EN_TO_DI1, ldb.control_reg);
		ldb.ch_working[1] = true;
		break;
	case LDB_SEP:
		if (var[0] == NULL || var[1] == NULL) {
			dev_err(g_ldb_dev, "Can't find framebuffers on DI0/1\n");
			break;
		}

		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[0]) == 24)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
				      LDB_DATA_WIDTH_CH0_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[0]) == 18)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH0_MASK) |
				      LDB_DATA_WIDTH_CH0_18,
				      ldb.control_reg);
		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[1]) == 24)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH1_MASK) |
				      LDB_DATA_WIDTH_CH1_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[1]) == 18)
			__raw_writel((reg & ~LDB_DATA_WIDTH_CH1_MASK) |
				      LDB_DATA_WIDTH_CH1_18,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[0] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_JEIDA,
				      ldb.control_reg);
		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[1] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_JEIDA,
				      ldb.control_reg);

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		clk_set_rate(ldb.ldb_di_clk[0], ldb_clk_prate/7);
		clk_enable(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[0]);
		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		clk_set_rate(ldb.ldb_di_clk[1], ldb_clk_prate/7);
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[1]);

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
				      LDB_CH1_MODE_MASK)) |
			      LDB_CH0_MODE_EN_TO_DI0 |
			      LDB_CH1_MODE_EN_TO_DI1, ldb.control_reg);
		ldb.ch_working[0] = true;
		ldb.ch_working[1] = true;
		break;
	case LDB_DUL_DI0:
	case LDB_SPL_DI0:
		if (var[0] == NULL) {
			dev_err(g_ldb_dev, "Can't find framebuffer on DI0\n");
			break;
		}

		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[0]) == 24)
			__raw_writel((reg & ~(LDB_DATA_WIDTH_CH0_MASK |
					      LDB_DATA_WIDTH_CH1_MASK)) |
				      LDB_DATA_WIDTH_CH0_24 |
				      LDB_DATA_WIDTH_CH1_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[0]) == 18)
			__raw_writel((reg & ~(LDB_DATA_WIDTH_CH0_MASK |
					      LDB_DATA_WIDTH_CH1_MASK)) |
				      LDB_DATA_WIDTH_CH0_18 |
				      LDB_DATA_WIDTH_CH1_18,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[0] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_JEIDA,
				      ldb.control_reg);
		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[1] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_JEIDA,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_mode_opt == LDB_SPL_DI0)
			__raw_writel(reg | LDB_SPLIT_MODE_EN,
				      ldb.control_reg);

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		if (ldb.chan_mode_opt == LDB_DUL_DI0) {
			clk_set_rate(ldb.ldb_di_clk[0], ldb_clk_prate/7);
		} else {
			clk_set_rate(ldb.ldb_di_clk[0], 2*ldb_clk_prate/7);
			clk_set_rate(ldb.ldb_di_clk[1], 2*ldb_clk_prate/7);
		}
		clk_enable(ldb.ldb_di_clk[0]);
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[1]);

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
				      LDB_CH1_MODE_MASK)) |
			      LDB_CH0_MODE_EN_TO_DI0 |
			      LDB_CH1_MODE_EN_TO_DI0, ldb.control_reg);
		ldb.ch_working[0] = true;
		ldb.ch_working[1] = true;
		break;
	case LDB_DUL_DI1:
	case LDB_SPL_DI1:
		if (var[1] == NULL) {
			dev_err(g_ldb_dev, "Can't find framebuffer on DI1\n");
			break;
		}

		reg = __raw_readl(ldb.control_reg);
		if (bits_per_pixel(ipu_di_pix_fmt[1]) == 24)
			__raw_writel((reg & ~(LDB_DATA_WIDTH_CH0_MASK |
					      LDB_DATA_WIDTH_CH1_MASK)) |
				      LDB_DATA_WIDTH_CH0_24 |
				      LDB_DATA_WIDTH_CH1_24,
				      ldb.control_reg);
		else if (bits_per_pixel(ipu_di_pix_fmt[1]) == 18)
			__raw_writel((reg & ~(LDB_DATA_WIDTH_CH0_MASK |
					      LDB_DATA_WIDTH_CH1_MASK)) |
				      LDB_DATA_WIDTH_CH0_18 |
				      LDB_DATA_WIDTH_CH1_18,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[0] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH0_MASK) |
				      LDB_BIT_MAP_CH0_JEIDA,
				      ldb.control_reg);
		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_bit_map[1] == LDB_BIT_MAP_SPWG)
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_SPWG,
				      ldb.control_reg);
		else
			__raw_writel((reg & ~LDB_BIT_MAP_CH1_MASK) |
				      LDB_BIT_MAP_CH1_JEIDA,
				      ldb.control_reg);

		reg = __raw_readl(ldb.control_reg);
		if (ldb.chan_mode_opt == LDB_SPL_DI1)
			__raw_writel(reg | LDB_SPLIT_MODE_EN,
				      ldb.control_reg);

		ldb.ldb_di_clk[0] = clk_get(NULL, "ldb_di0_clk");
		ldb.ldb_di_clk[1] = clk_get(NULL, "ldb_di1_clk");
		if (ldb.chan_mode_opt == LDB_DUL_DI1) {
			clk_set_rate(ldb.ldb_di_clk[1], ldb_clk_prate/7);
		} else {
			clk_set_rate(ldb.ldb_di_clk[0], 2*ldb_clk_prate/7);
			clk_set_rate(ldb.ldb_di_clk[1], 2*ldb_clk_prate/7);
		}
		clk_enable(ldb.ldb_di_clk[0]);
		clk_enable(ldb.ldb_di_clk[1]);
		clk_put(ldb.ldb_di_clk[0]);
		clk_put(ldb.ldb_di_clk[1]);

		reg = __raw_readl(ldb.control_reg);
		__raw_writel((reg & ~(LDB_CH0_MODE_MASK |
				      LDB_CH1_MODE_MASK)) |
			      LDB_CH0_MODE_EN_TO_DI1 |
			      LDB_CH1_MODE_EN_TO_DI1, ldb.control_reg);
		ldb.ch_working[0] = true;
		ldb.ch_working[1] = true;
		break;
	default:
		break;
	}

	mxc_ldb_major = register_chrdev(0, "mxc_ldb", &mxc_ldb_fops);
	if (mxc_ldb_major < 0) {
		dev_err(g_ldb_dev, "Unable to register MXC LDB as a char "
				   "device\n");
		ret = mxc_ldb_major;
		goto err0;
	}

	mxc_ldb_class = class_create(THIS_MODULE, "mxc_ldb");
	if (IS_ERR(mxc_ldb_class)) {
		dev_err(g_ldb_dev, "Unable to create class for MXC LDB\n");
		ret = PTR_ERR(mxc_ldb_class);
		goto err1;
	}

	temp = device_create(mxc_ldb_class, NULL, MKDEV(mxc_ldb_major, 0),
			NULL, "mxc_ldb");
	if (IS_ERR(temp)) {
		dev_err(g_ldb_dev, "Unable to create class device for "
				   "MXC LDB\n");
		ret = PTR_ERR(temp);
		goto err2;
	}

	ret = fb_register_client(&nb);
	if (ret < 0)
		goto err2;

	if (primary && ldb.fbi[0] != NULL) {
		acquire_console_sem();
		fb_blank(ldb.fbi[0], FB_BLANK_UNBLANK);
		release_console_sem();
		fb_show_logo(ldb.fbi[0], 0);
	}

	return ret;
err2:
	class_destroy(mxc_ldb_class);
err1:
	unregister_chrdev(mxc_ldb_major, "mxc_ldb");
err0:
	iounmap(ldb_reg);
	return ret;
}

static int ldb_remove(struct platform_device *pdev)
{
	int i;

	__raw_writel(0, ldb.control_reg);

	for (i = 0; i < 2; i++) {
		if (ldb.ch_working[i]) {
			ldb.ldb_di_clk[i] = clk_get(NULL,
					    i ? "ldb_di1_clk" : "ldb_di0_clk");
			clk_disable(ldb.ldb_di_clk[i]);
			clk_put(ldb.ldb_di_clk[i]);
			ldb.ch_working[i] = false;
		}
	}

	fb_unregister_client(&nb);
	return 0;
}

static int ldb_suspend(struct platform_device *pdev, pm_message_t state)
{
	switch (ldb.chan_mode_opt) {
	case LDB_SIN_DI0:
	case LDB_DUL_DI0:
	case LDB_SPL_DI0:
		ldb_disable(0);
		break;
	case LDB_SIN_DI1:
	case LDB_DUL_DI1:
	case LDB_SPL_DI1:
		ldb_disable(1);
		break;
	case LDB_SEP:
		ldb_disable(0);
		ldb_disable(1);
		break;
	default:
		break;
	}
	return 0;
}

static int ldb_resume(struct platform_device *pdev)
{
	switch (ldb.chan_mode_opt) {
	case LDB_SIN_DI0:
	case LDB_DUL_DI0:
	case LDB_SPL_DI0:
		ldb_enable(0);
		break;
	case LDB_SIN_DI1:
	case LDB_DUL_DI1:
	case LDB_SPL_DI1:
		ldb_enable(1);
		break;
	case LDB_SEP:
		ldb_enable(0);
		ldb_enable(1);
		break;
	default:
		break;
	}
	return 0;
}

static struct platform_driver mxcldb_driver = {
	.driver = {
		   .name = "mxc_ldb",
		   },
	.probe = ldb_probe,
	.remove = ldb_remove,
	.suspend = ldb_suspend,
	.resume = ldb_resume,
};

/*
 * Parse user specified options (`ldb=')
 * example:
 * ldb=single(separate, dual or split),(di=0 or di=1),
 *	ch0_map=SPWG or JEIDA,ch1_map=SPWG or JEIDA
 *
 */
static int __init ldb_setup(char *options)
{
	g_enable_ldb = true;

	if (!strlen(options))
		return 1;
	else if (!strsep(&options, "="))
		return 1;

	if (!strncmp(options, "single", 6)) {
		strsep(&options, ",");
		if (!strncmp(options, "di=0", 4))
			g_chan_mode_opt = LDB_SIN_DI0;
		else
			g_chan_mode_opt = LDB_SIN_DI1;
	} else if (!strncmp(options, "separate", 8)) {
		g_chan_mode_opt = LDB_SEP;
	} else if (!strncmp(options, "dual", 4)) {
		strsep(&options, ",");
		if (!strncmp(options, "di=", 3)) {
			if (simple_strtoul(options + 3, NULL, 0) == 0)
				g_chan_mode_opt = LDB_DUL_DI0;
			else
				g_chan_mode_opt = LDB_DUL_DI1;
		}
	} else if (!strncmp(options, "split", 5)) {
		strsep(&options, ",");
		if (!strncmp(options, "di=", 3)) {
			if (simple_strtoul(options + 3, NULL, 0) == 0)
				g_chan_mode_opt = LDB_SPL_DI0;
			else
				g_chan_mode_opt = LDB_SPL_DI1;
		}
	} else
		return 1;

	if ((strsep(&options, ",") != NULL) &&
	    !strncmp(options, "ch0_map=", 8)) {
		if (!strncmp(options + 8, "SPWG", 4))
			g_chan_bit_map[0] = LDB_BIT_MAP_SPWG;
		else
			g_chan_bit_map[0] = LDB_BIT_MAP_JEIDA;
	}

	if (!(g_chan_mode_opt == LDB_SIN_DI0 ||
	      g_chan_mode_opt == LDB_SIN_DI1) &&
	    (strsep(&options, ",") != NULL) &&
	    !strncmp(options, "ch1_map=", 8)) {
		if (!strncmp(options + 8, "SPWG", 4))
			g_chan_bit_map[1] = LDB_BIT_MAP_SPWG;
		else
			g_chan_bit_map[1] = LDB_BIT_MAP_JEIDA;
	}

	g_boot_cmd = true;

	return 1;
}
__setup("ldb", ldb_setup);

static int __init ldb_init(void)
{
	int ret;

	ret = platform_driver_register(&mxcldb_driver);
	return 0;
}

static void __exit ldb_uninit(void)
{
	platform_driver_unregister(&mxcldb_driver);
}

module_init(ldb_init);
module_exit(ldb_uninit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC LDB driver");
MODULE_LICENSE("GPL");
