/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file tve.c
 * @brief Driver for i.MX TV encoder
 *
 * @ingroup Framebuffer
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <mach/hardware.h>
#include "mxc_dispdrv.h"

#define TVE_ENABLE			(1UL)
#define TVE_DAC_FULL_RATE		(0UL<<1)
#define TVE_DAC_DIV2_RATE		(1UL<<1)
#define TVE_DAC_DIV4_RATE		(2UL<<1)
#define TVE_IPU_CLK_ENABLE		(1UL<<3)

#define CD_LM_INT		0x00000001
#define CD_SM_INT		0x00000002
#define CD_MON_END_INT		0x00000004
#define CD_CH_0_LM_ST		0x00000001
#define CD_CH_0_SM_ST		0x00000010
#define CD_CH_1_LM_ST		0x00000002
#define CD_CH_1_SM_ST		0x00000020
#define CD_CH_2_LM_ST		0x00000004
#define CD_CH_2_SM_ST		0x00000040
#define CD_MAN_TRIG		0x00000100

#define TVE_STAND_MASK			(0x0F<<8)
#define TVE_NTSC_STAND			(0UL<<8)
#define TVE_PAL_STAND			(3UL<<8)
#define TVE_HD720P60_STAND		(4UL<<8)
#define TVE_HD720P50_STAND		(5UL<<8)
#define TVE_HD720P30_STAND		(6UL<<8)
#define TVE_HD720P25_STAND		(7UL<<8)
#define TVE_HD720P24_STAND		(8UL<<8)
#define TVE_HD1080I60_STAND		(9UL<<8)
#define TVE_HD1080I50_STAND		(10UL<<8)
#define TVE_HD1035I60_STAND		(11UL<<8)
#define TVE_HD1080P30_STAND		(12UL<<8)
#define TVE_HD1080P25_STAND		(13UL<<8)
#define TVE_HD1080P24_STAND		(14UL<<8)
#define TVE_DAC_SAMPRATE_MASK		(0x3<<1)
#define TVEV2_DATA_SRC_MASK		(0x3<<4)

#define TVEV2_DATA_SRC_BUS_1		(0UL<<4)
#define TVEV2_DATA_SRC_BUS_2		(1UL<<4)
#define TVEV2_DATA_SRC_EXT		(2UL<<4)

#define TVEV2_INP_VIDEO_FORM		(1UL<<6)
#define TVEV2_P2I_CONV_EN		(1UL<<7)

#define TVEV2_DAC_GAIN_MASK		0x3F
#define TVEV2_DAC_TEST_MODE_MASK	0x7

#define TVOUT_FMT_OFF			0
#define TVOUT_FMT_NTSC			1
#define TVOUT_FMT_PAL			2
#define TVOUT_FMT_720P60		3
#define TVOUT_FMT_720P30		4
#define TVOUT_FMT_1080I60		5
#define TVOUT_FMT_1080I50		6
#define TVOUT_FMT_1080P30		7
#define TVOUT_FMT_1080P25		8
#define TVOUT_FMT_1080P24		9
#define TVOUT_FMT_VGA_SVGA		10
#define TVOUT_FMT_VGA_XGA		11
#define TVOUT_FMT_VGA_SXGA		12
#define TVOUT_FMT_VGA_WSXGA		13

#define DISPDRV_VGA 	"vga"
#define DISPDRV_TVE 	"tve"

struct tve_data {
	struct platform_device *pdev;
	int revision;
	int cur_mode;
	int output_mode;
	int detect;
	void *base;
	spinlock_t tve_lock;
	bool inited;
	int enabled;
	int irq;
	struct clk *clk;
	struct clk *di_clk;
	struct regulator *dac_reg;
	struct regulator *dig_reg;
	struct delayed_work cd_work;
	struct tve_reg_mapping *regs;
	struct tve_reg_fields_mapping *reg_fields;
	struct mxc_dispdrv_handle *disp_tve;
	struct mxc_dispdrv_handle *disp_vga;
	struct notifier_block nb;
};

struct tve_reg_mapping {
	u32 tve_com_conf_reg;
	u32 tve_cd_cont_reg;
	u32 tve_int_cont_reg;
	u32 tve_stat_reg;
	u32 tve_mv_cont_reg;
	u32 tve_tvdac_cont_reg;
	u32 tve_tst_mode_reg;
};

struct tve_reg_fields_mapping {
	u32 cd_en;
	u32 cd_trig_mode;
	u32 cd_lm_int;
	u32 cd_sm_int;
	u32 cd_mon_end_int;
	u32 cd_man_trig;
	u32 sync_ch_mask;
	u32 tvout_mode_mask;
	u32 sync_ch_offset;
	u32 tvout_mode_offset;
	u32 cd_ch_stat_offset;
};

static struct tve_reg_mapping tve_regs_v1 = {
	0, 0x14, 0x28, 0x2C, 0x48, 0x08, 0x30
};

static struct tve_reg_fields_mapping tve_reg_fields_v1 = {
	1, 2, 1, 2, 4, 0x00010000, 0x7000, 0x70, 12, 4, 8
};

static struct tve_reg_mapping tve_regs_v2 = {
	0, 0x34, 0x64, 0x68, 0xDC, 0x28, 0x6c
};

static struct tve_reg_fields_mapping tve_reg_fields_v2 = {
	1, 2, 1, 2, 4, 0x01000000, 0x700000, 0x7000, 20, 12, 16
};

static struct fb_videomode video_modes_tve[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 122, 15,
	 18, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 720p60 TV output */
	 "TV-720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 720p30 TV output */
	 "TV-720P30", 30, 1280, 720, 13468,
	 260, 1759,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 1080i60 TV output */
	 "TV-1080I60", 60, 1920, 1080, 13468,
	 192, 87,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 1080i50 TV output */
	 "TV-1080I50", 50, 1920, 1080, 13468,
	 192, 527,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 1080p30 TV output */
	 "TV-1080P30", 30, 1920, 1080, 13468,
	 192, 87,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 1080p25 TV output */
	 "TV-1080P25", 25, 1920, 1080, 13468,
	 192, 527,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* 1080p24 TV output */
	 "TV-1080P24", 24, 1920, 1080, 13468,
	 192, 637,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
};
static int tve_modedb_sz = ARRAY_SIZE(video_modes_tve);

static struct fb_videomode video_modes_vga[] = {
	{
	/* VGA 800x600 40M pixel clk output */
	 "VGA-SVGA", 60, 800, 600, 25000,
	 215, 28,
	 24, 2,
	 13, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	/* VGA 1024x768 65M pixel clk output */
	 "VGA-XGA", 60, 1024, 768, 15384,
	 160, 24,
	 29, 3,
	 136, 6,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	/* VGA 1280x1024 108M pixel clk output */
	"VGA-SXGA", 60, 1280, 1024, 9259,
	358, 38,
	38, 2,
	12, 2,
	0,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
	{
	/* VGA 1680x1050 294M pixel clk output */
	"VGA-WSXGA+", 60, 1680, 1050, 6796,
	288, 104,
	33, 2,
	184, 2,
	0,
	FB_VMODE_NONINTERLACED,
	FB_MODE_IS_DETAILED,},
};
static int vga_modedb_sz = ARRAY_SIZE(video_modes_vga);

enum tvout_mode {
	TV_OFF,
	CVBS0,
	CVBS2,
	CVBS02,
	SVIDEO,
	SVIDEO_CVBS,
	YPBPR,
	TVRGB
};

static unsigned short tvout_mode_to_channel_map[8] = {
	0,	/* TV_OFF */
	1,	/* CVBS0 */
	4,	/* CVBS2 */
	5,	/* CVBS02 */
	1,	/* SVIDEO */
	5,	/* SVIDEO_CVBS */
	1,	/* YPBPR */
	7	/* TVRGB */
};

static void tve_dump_regs(struct tve_data *tve)
{
	dev_dbg(&tve->pdev->dev, "tve_com_conf_reg 0x%x\n",
			readl(tve->base + tve->regs->tve_com_conf_reg));
	dev_dbg(&tve->pdev->dev, "tve_cd_cont_reg 0x%x\n",
			readl(tve->base + tve->regs->tve_cd_cont_reg));
	dev_dbg(&tve->pdev->dev, "tve_int_cont_reg 0x%x\n",
			readl(tve->base + tve->regs->tve_int_cont_reg));
	dev_dbg(&tve->pdev->dev, "tve_tst_mode_reg 0x%x\n",
			readl(tve->base + tve->regs->tve_tst_mode_reg));
	dev_dbg(&tve->pdev->dev, "tve_tvdac_cont_reg0 0x%x\n",
			readl(tve->base + tve->regs->tve_tvdac_cont_reg));
	dev_dbg(&tve->pdev->dev, "tve_tvdac_cont_reg1 0x%x\n",
			readl(tve->base + tve->regs->tve_tvdac_cont_reg + 4));
	dev_dbg(&tve->pdev->dev, "tve_tvdac_cont_reg2 0x%x\n",
			readl(tve->base + tve->regs->tve_tvdac_cont_reg + 8));
}

static int is_vga_enabled(struct tve_data *tve)
{
	u32 reg;

	if (tve->revision == 2) {
		reg = readl(tve->base + tve->regs->tve_tst_mode_reg);
		if (reg & TVEV2_DAC_TEST_MODE_MASK)
			return 1;
		else
			return 0;
	}
	return 0;
}

static inline int is_vga_mode(int mode)
{
	return ((mode == TVOUT_FMT_VGA_SVGA)
		|| (mode == TVOUT_FMT_VGA_XGA)
		|| (mode == TVOUT_FMT_VGA_SXGA)
		|| (mode == TVOUT_FMT_VGA_WSXGA));
}

static inline int valid_mode(int mode)
{
	return (is_vga_mode(mode)
		|| (mode == TVOUT_FMT_NTSC)
		|| (mode == TVOUT_FMT_PAL)
		|| (mode == TVOUT_FMT_720P30)
		|| (mode == TVOUT_FMT_720P60)
		|| (mode == TVOUT_FMT_1080I50)
		|| (mode == TVOUT_FMT_1080I60)
		|| (mode == TVOUT_FMT_1080P24)
		|| (mode == TVOUT_FMT_1080P25)
		|| (mode == TVOUT_FMT_1080P30));
}

static int get_video_mode(struct fb_info *fbi)
{
	int mode;

	if (fb_mode_is_equal(fbi->mode, &video_modes_tve[0])) {
		mode = TVOUT_FMT_NTSC;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[1])) {
		mode = TVOUT_FMT_PAL;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[2])) {
		mode = TVOUT_FMT_720P60;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[3])) {
		mode = TVOUT_FMT_720P30;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[4])) {
		mode = TVOUT_FMT_1080I60;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[5])) {
		mode = TVOUT_FMT_1080I50;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[6])) {
		mode = TVOUT_FMT_1080P30;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[7])) {
		mode = TVOUT_FMT_1080P25;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_tve[8])) {
		mode = TVOUT_FMT_1080P24;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_vga[0])) {
		mode = TVOUT_FMT_VGA_SVGA;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_vga[1])) {
		mode = TVOUT_FMT_VGA_XGA;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_vga[2])) {
		mode = TVOUT_FMT_VGA_SXGA;
	} else if (fb_mode_is_equal(fbi->mode, &video_modes_vga[3])) {
		mode = TVOUT_FMT_VGA_WSXGA;
	} else {
		mode = TVOUT_FMT_OFF;
	}
	return mode;
}

static void tve_disable_vga_mode(struct tve_data *tve)
{
	if (tve->revision == 2) {
		u32 reg;
		/* disable test mode */
		reg = readl(tve->base + tve->regs->tve_tst_mode_reg);
		reg = reg & ~TVEV2_DAC_TEST_MODE_MASK;
		writel(reg, tve->base + tve->regs->tve_tst_mode_reg);
	}
}

static void tve_set_tvout_mode(struct tve_data *tve, int mode)
{
	u32 conf_reg;

	/* clear sync_ch and tvout_mode fields */
	conf_reg = readl(tve->base + tve->regs->tve_com_conf_reg);
	conf_reg &= ~(tve->reg_fields->sync_ch_mask |
				tve->reg_fields->tvout_mode_mask);

	conf_reg = conf_reg & ~TVE_DAC_SAMPRATE_MASK;
	if (tve->revision == 2) {
		conf_reg = (conf_reg & ~TVEV2_DATA_SRC_MASK) |
			TVEV2_DATA_SRC_BUS_1;
		conf_reg = conf_reg & ~TVEV2_INP_VIDEO_FORM;
		conf_reg = conf_reg & ~TVEV2_P2I_CONV_EN;
	}

	conf_reg |=
		mode << tve->reg_fields->
		tvout_mode_offset | tvout_mode_to_channel_map[mode] <<
		tve->reg_fields->sync_ch_offset;
	writel(conf_reg, tve->base + tve->regs->tve_com_conf_reg);
}

static int _is_tvout_mode_hd_compatible(struct tve_data *tve)
{
	u32 conf_reg, mode;

	conf_reg = readl(tve->base + tve->regs->tve_com_conf_reg);
	mode = (conf_reg >> tve->reg_fields->tvout_mode_offset) & 7;
	if (mode == YPBPR || mode == TVRGB) {
		return 1;
	} else {
		return 0;
	}
}

static int tve_setup_vga(struct tve_data *tve)
{
	u32 reg;

	if (tve->revision == 2) {
		/* set gain */
		reg = readl(tve->base + tve->regs->tve_tvdac_cont_reg);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0xa;
		writel(reg, tve->base + tve->regs->tve_tvdac_cont_reg);
		reg = readl(tve->base + tve->regs->tve_tvdac_cont_reg + 4);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0xa;
		writel(reg, tve->base + tve->regs->tve_tvdac_cont_reg + 4);
		reg = readl(tve->base + tve->regs->tve_tvdac_cont_reg + 8);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0xa;
		writel(reg, tve->base + tve->regs->tve_tvdac_cont_reg + 8);

		/* set tve_com_conf_reg  */
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_DAC_SAMPRATE_MASK) | TVE_DAC_DIV2_RATE;
		reg = (reg & ~TVEV2_DATA_SRC_MASK) | TVEV2_DATA_SRC_BUS_2;
		reg = reg | TVEV2_INP_VIDEO_FORM;
		reg = reg & ~TVEV2_P2I_CONV_EN;
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P30_STAND;
		reg |= TVRGB << tve->reg_fields->tvout_mode_offset |
			1 << tve->reg_fields->sync_ch_offset;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);

		/* set test mode */
		reg = readl(tve->base + tve->regs->tve_tst_mode_reg);
		reg = (reg & ~TVEV2_DAC_TEST_MODE_MASK) | 1;
		writel(reg, tve->base + tve->regs->tve_tst_mode_reg);
	}

	return 0;
}

/**
 * tve_setup
 * initial the CH7024 chipset by setting register
 * @param:
 * 	vos: output video format
 * @return:
 * 	0 successful
 * 	otherwise failed
 */
static int tve_setup(struct tve_data *tve, int mode)
{
	u32 reg;
	struct clk *tve_parent_clk;
	unsigned long parent_clock_rate = 216000000, di1_clock_rate = 27000000;
	unsigned long tve_clock_rate = 216000000;
	unsigned long lock_flags;

	if (tve->cur_mode == mode)
		return 0;

	spin_lock_irqsave(&tve->tve_lock, lock_flags);

	switch (mode) {
	case TVOUT_FMT_PAL:
	case TVOUT_FMT_NTSC:
		parent_clock_rate = 216000000;
		di1_clock_rate = 27000000;
		break;
	case TVOUT_FMT_720P60:
	case TVOUT_FMT_1080I60:
	case TVOUT_FMT_1080I50:
	case TVOUT_FMT_720P30:
	case TVOUT_FMT_1080P30:
	case TVOUT_FMT_1080P25:
	case TVOUT_FMT_1080P24:
		parent_clock_rate = 297000000;
		tve_clock_rate = 297000000;
		di1_clock_rate = 74250000;
		break;
	case TVOUT_FMT_VGA_SVGA:
		parent_clock_rate = 160000000;
		tve_clock_rate = 80000000;
		di1_clock_rate = 40000000;
		break;
	case TVOUT_FMT_VGA_XGA:
		parent_clock_rate = 520000000;
		tve_clock_rate = 130000000;
		di1_clock_rate = 65000000;
		break;
	case TVOUT_FMT_VGA_SXGA:
		parent_clock_rate = 864000000;
		tve_clock_rate = 216000000;
		di1_clock_rate = 108000000;
		break;
	case TVOUT_FMT_VGA_WSXGA:
		parent_clock_rate = 588560000;
		tve_clock_rate = 294280000;
		di1_clock_rate = 147140000;
		break;
	}
	if (tve->enabled)
		clk_disable(tve->clk);

	tve_parent_clk = clk_get_parent(tve->clk);

	clk_set_rate(tve_parent_clk, parent_clock_rate);

	tve_clock_rate = clk_round_rate(tve->clk, tve_clock_rate);
	clk_set_rate(tve->clk, tve_clock_rate);

	clk_enable(tve->clk);
	di1_clock_rate = clk_round_rate(tve->di_clk, di1_clock_rate);
	clk_set_rate(tve->di_clk, di1_clock_rate);

	tve->cur_mode = mode;

	/* select output video format */
	if (mode == TVOUT_FMT_PAL) {
		tve_disable_vga_mode(tve);
		tve_set_tvout_mode(tve, YPBPR);
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_PAL_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to PAL video\n");
	} else if (mode == TVOUT_FMT_NTSC) {
		tve_disable_vga_mode(tve);
		tve_set_tvout_mode(tve, YPBPR);
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_NTSC_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to NTSC video\n");
	} else if (mode == TVOUT_FMT_720P60) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD720P60_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 720P60 video\n");
	} else if (mode == TVOUT_FMT_720P30) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD720P30_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 720P30 video\n");
	} else if (mode == TVOUT_FMT_1080I60) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080I60_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 1080I60 video\n");
	} else if (mode == TVOUT_FMT_1080I50) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080I50_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 1080I50 video\n");
	} else if (mode == TVOUT_FMT_1080P30) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P30_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 1080P30 video\n");
	} else if (mode == TVOUT_FMT_1080P25) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P25_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 1080P25 video\n");
	} else if (mode == TVOUT_FMT_1080P24) {
		tve_disable_vga_mode(tve);
		if (!_is_tvout_mode_hd_compatible(tve)) {
			tve_set_tvout_mode(tve, YPBPR);
			dev_dbg(&tve->pdev->dev, "The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P24_STAND;
		writel(reg, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to 1080P24 video\n");
	} else if (is_vga_mode(mode)) {
		/* do not need cable detect */
		tve_setup_vga(tve);
		dev_dbg(&tve->pdev->dev, "TVE: change to VGA video\n");
	} else if (mode == TVOUT_FMT_OFF) {
		writel(0x0, tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE: change to OFF video\n");
	} else {
		dev_dbg(&tve->pdev->dev, "TVE: no such video format.\n");
	}

	if (!tve->enabled)
		clk_disable(tve->clk);

	spin_unlock_irqrestore(&tve->tve_lock, lock_flags);
	return 0;
}

/**
 * tve_enable
 * Enable the tve Power to begin TV encoder
 */
static void tve_enable(struct tve_data *tve)
{
	u32 reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&tve->tve_lock, lock_flags);
	if (!tve->enabled) {
		tve->enabled = 1;
		clk_enable(tve->clk);
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		writel(reg | TVE_IPU_CLK_ENABLE | TVE_ENABLE,
					tve->base + tve->regs->tve_com_conf_reg);
		dev_dbg(&tve->pdev->dev, "TVE power on.\n");
	}

	if (is_vga_enabled(tve)) {
		/* disable interrupt */
		dev_dbg(&tve->pdev->dev, "TVE VGA disable cable detect.\n");
		writel(0xffffffff, tve->base + tve->regs->tve_stat_reg);
		writel(0, tve->base + tve->regs->tve_int_cont_reg);
	} else {
		/* enable interrupt */
		dev_dbg(&tve->pdev->dev, "TVE TVE enable cable detect.\n");
		writel(0xffffffff, tve->base + tve->regs->tve_stat_reg);
		writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT,
				tve->base + tve->regs->tve_int_cont_reg);
	}

	spin_unlock_irqrestore(&tve->tve_lock, lock_flags);

	tve_dump_regs(tve);
}

/**
 * tve_disable
 * Disable the tve Power to stop TV encoder
 */
static void tve_disable(struct tve_data *tve)
{
	u32 reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&tve->tve_lock, lock_flags);
	if (tve->enabled) {
		tve->enabled = 0;
		reg = readl(tve->base + tve->regs->tve_com_conf_reg);
		writel(reg & ~TVE_ENABLE & ~TVE_IPU_CLK_ENABLE,
				tve->base + tve->regs->tve_com_conf_reg);
		clk_disable(tve->clk);
		dev_dbg(&tve->pdev->dev, "TVE power off.\n");
	}
	spin_unlock_irqrestore(&tve->tve_lock, lock_flags);
}

static int tve_update_detect_status(struct tve_data *tve)
{
	int old_detect = tve->detect;
	u32 stat_lm, stat_sm, stat;
	u32 int_ctl;
	u32 cd_cont_reg;
	u32 timeout = 40;
	unsigned long lock_flags;
	char event_string[16];
	char *envp[] = { event_string, NULL };

	spin_lock_irqsave(&tve->tve_lock, lock_flags);

	if (!tve->enabled) {
		dev_warn(&tve->pdev->dev, "Warning: update tve status while it disabled!\n");
		tve->detect = 0;
		goto done;
	}

	int_ctl = readl(tve->base + tve->regs->tve_int_cont_reg);
	cd_cont_reg = readl(tve->base + tve->regs->tve_cd_cont_reg);

	if ((cd_cont_reg & 0x1) == 0) {
		dev_warn(&tve->pdev->dev, "Warning: pls enable TVE CD first!\n");
		goto done;
	}

	stat = readl(tve->base + tve->regs->tve_stat_reg);
	while (((stat & CD_MON_END_INT) == 0) && (timeout > 0)) {
		spin_unlock_irqrestore(&tve->tve_lock, lock_flags);
		msleep(2);
		spin_lock_irqsave(&tve->tve_lock, lock_flags);
		timeout -= 2;
		if (!tve->enabled) {
			dev_warn(&tve->pdev->dev, "Warning: update tve status while it disabled!\n");
			tve->detect = 0;
			goto done;
		} else
			stat = readl(tve->base + tve->regs->tve_stat_reg);
	}
	if (((stat & CD_MON_END_INT) == 0) && (timeout <= 0)) {
		dev_warn(&tve->pdev->dev, "Warning: get detect result without CD_MON_END_INT!\n");
		goto done;
	}

	stat = stat >> tve->reg_fields->cd_ch_stat_offset;
	stat_lm = stat & (CD_CH_0_LM_ST | CD_CH_1_LM_ST | CD_CH_2_LM_ST);
	if ((stat_lm == (CD_CH_0_LM_ST | CD_CH_1_LM_ST | CD_CH_2_LM_ST)) &&
		((stat & (CD_CH_0_SM_ST | CD_CH_1_SM_ST | CD_CH_2_SM_ST)) == 0)
		) {
			tve->detect = 3;
			tve->output_mode = YPBPR;
	} else if ((stat_lm == (CD_CH_0_LM_ST | CD_CH_1_LM_ST)) &&
		((stat & (CD_CH_0_SM_ST | CD_CH_1_SM_ST)) == 0)) {
			tve->detect = 4;
			tve->output_mode = SVIDEO;
	} else if (stat_lm == CD_CH_0_LM_ST) {
		stat_sm = stat & CD_CH_0_SM_ST;
		if (stat_sm != 0) {
			/* headset */
			tve->detect = 2;
			tve->output_mode = TV_OFF;
		} else {
			tve->detect = 1;
			tve->output_mode = CVBS0;
		}
	} else if (stat_lm == CD_CH_2_LM_ST) {
		stat_sm = stat & CD_CH_2_SM_ST;
		if (stat_sm != 0) {
			/* headset */
			tve->detect = 2;
			tve->output_mode = TV_OFF;
		} else {
			tve->detect = 1;
			tve->output_mode = CVBS2;
		}
	} else {
		/* none */
		tve->detect = 0;
		tve->output_mode = TV_OFF;
	}

	tve_set_tvout_mode(tve, tve->output_mode);

	/* clear interrupt */
	writel(CD_MON_END_INT | CD_LM_INT | CD_SM_INT,
			tve->base + tve->regs->tve_stat_reg);

	writel(int_ctl | CD_SM_INT | CD_LM_INT,
			tve->base + tve->regs->tve_int_cont_reg);

done:
	spin_unlock_irqrestore(&tve->tve_lock, lock_flags);

	if (old_detect != tve->detect) {
		sysfs_notify(&tve->pdev->dev.kobj, NULL, "headphone");
		if (tve->detect == 1)
			sprintf(event_string, "EVENT=CVBS0");
		else if (tve->detect == 3)
			sprintf(event_string, "EVENT=YPBPR");
		else if (tve->detect == 4)
			sprintf(event_string, "EVENT=SVIDEO");
		else
			sprintf(event_string, "EVENT=NONE");
		kobject_uevent_env(&tve->pdev->dev.kobj, KOBJ_CHANGE, envp);
	}

	dev_dbg(&tve->pdev->dev, "detect = %d mode = %d\n",
			tve->detect, tve->output_mode);
	return tve->detect;
}

static void cd_work_func(struct work_struct *work)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct tve_data *tve =
		container_of(delay_work, struct tve_data, cd_work);

	tve_update_detect_status(tve);
}

static irqreturn_t tve_detect_handler(int irq, void *data)
{
	struct tve_data *tve = data;

	u32 int_ctl = readl(tve->base + tve->regs->tve_int_cont_reg);

	/* disable INT first */
	int_ctl &= ~(CD_SM_INT | CD_LM_INT | CD_MON_END_INT);
	writel(int_ctl, tve->base + tve->regs->tve_int_cont_reg);

	writel(CD_MON_END_INT | CD_LM_INT | CD_SM_INT,
			tve->base + tve->regs->tve_stat_reg);

	schedule_delayed_work(&tve->cd_work, msecs_to_jiffies(1000));

	return IRQ_HANDLED;
}

/*!
 * FB suspend/resume routing
 */
static int tve_suspend(struct tve_data *tve)
{
	if (tve->enabled) {
		writel(0, tve->base + tve->regs->tve_int_cont_reg);
		writel(0, tve->base + tve->regs->tve_cd_cont_reg);
		writel(0, tve->base + tve->regs->tve_com_conf_reg);
		clk_disable(tve->clk);
	}
	return 0;
}

static int tve_resume(struct tve_data *tve, struct fb_info *fbi)
{
	int mode;

	if (tve->enabled) {
		clk_enable(tve->clk);

		/* Setup cable detect */
		if (tve->revision == 1)
			writel(0x01067701,
				tve->base + tve->regs->tve_cd_cont_reg);
		else
			writel(0x00770601,
				tve->base + tve->regs->tve_cd_cont_reg);

		if (valid_mode(tve->cur_mode)) {
			mode = tve->cur_mode;
			tve_disable(tve);
			tve->cur_mode = TVOUT_FMT_OFF;
			tve_setup(tve, mode);
		}
		tve_enable(tve);
	}

	return 0;
}

int tve_fb_setup(struct tve_data *tve, struct fb_info *fbi)
{
	int mode;

	fbi->mode = (struct fb_videomode *)fb_match_mode(&fbi->var,
			&fbi->modelist);

	if (!fbi->mode) {
		dev_warn(&tve->pdev->dev, "TVE: can not find mode for xres=%d, yres=%d\n",
				fbi->var.xres, fbi->var.yres);
		tve_disable(tve);
		tve->cur_mode = TVOUT_FMT_OFF;
		return 0;
	}

	dev_dbg(&tve->pdev->dev, "TVE: fb mode change event: xres=%d, yres=%d\n",
			fbi->mode->xres, fbi->mode->yres);

	mode = get_video_mode(fbi);
	if (mode != TVOUT_FMT_OFF) {
		tve_disable(tve);
		tve_setup(tve, mode);
		tve_enable(tve);
	} else {
		tve_disable(tve);
		tve_setup(tve, mode);
	}

	return 0;
}

static inline int tve_disp_setup(struct mxc_dispdrv_handle *disp,
	struct fb_info *fbi)
{
	struct tve_data *tve = mxc_dispdrv_getdata(disp);

	tve_fb_setup(tve, fbi);
	return 0;
}

int tve_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct tve_data *tve = container_of(nb, struct tve_data, nb);
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	/* only work for ipu0 di1*/
	if (strcmp(fbi->fix.id, "DISP3 BG - DI1"))
		return 0;

	switch (val) {
	case FB_EVENT_BLANK:
		if (fbi->mode == NULL)
			return 0;

		dev_dbg(&tve->pdev->dev, "TVE: fb blank event\n");

		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			int mode;
			mode = get_video_mode(fbi);
			if (mode != TVOUT_FMT_OFF) {
				if (tve->cur_mode != mode) {
					tve_disable(tve);
					tve_setup(tve, mode);
				}
				tve_enable(tve);
			} else
				tve_setup(tve, mode);
		} else
			tve_disable(tve);
		break;
	case FB_EVENT_SUSPEND:
		tve_suspend(tve);
		break;
	case FB_EVENT_RESUME:
		tve_resume(tve, fbi);
		break;
	}
	return 0;
}

static ssize_t show_headphone(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tve_data *tve = dev_get_drvdata(dev);
	int detect;

	if (!tve->enabled) {
		strcpy(buf, "tve power off\n");
		return strlen(buf);
	}

	detect = tve_update_detect_status(tve);

	if (detect == 0)
		strcpy(buf, "none\n");
	else if (detect == 1)
		strcpy(buf, "cvbs\n");
	else if (detect == 2)
		strcpy(buf, "headset\n");
	else if (detect == 3)
		strcpy(buf, "component\n");
	else
		strcpy(buf, "svideo\n");

	return strlen(buf);
}

static DEVICE_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static int _tve_get_revision(struct tve_data *tve)
{
	u32 conf_reg;
	u32 rev = 0;

	/* find out TVE rev based on the base addr default value
	 * can be used at the init/probe ONLY */
	conf_reg = readl(tve->base);
	switch (conf_reg) {
	case 0x00842000:
		rev = 1;
		break;
	case 0x00100000:
		rev = 2;
		break;
	}
	return rev;
}

static int tve_drv_init(struct mxc_dispdrv_handle *disp, bool vga,
	struct mxc_dispdrv_setting *setting)
{
	int ret;
	struct tve_data *tve = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_tve_platform_data *plat_data
			= tve->pdev->dev.platform_data;
	struct resource *res;
	struct fb_videomode *modedb;
	int modedb_sz;
	u32 conf_reg;

	if (tve->inited == true)
		return -ENODEV;

	/*tve&vga only use ipu0 and di1*/
	setting->dev_id = 0;
	setting->disp_id = 1;

	res = platform_get_resource(tve->pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENOMEM;
		goto get_res_failed;
	}

	tve->irq = platform_get_irq(tve->pdev, 0);
	if (tve->irq < 0) {
		ret = tve->irq;
		goto get_irq_failed;
	}

	tve->base = ioremap(res->start, res->end - res->start);
	if (!tve->base) {
		ret = -ENOMEM;
		goto ioremap_failed;
	}

	ret = device_create_file(&tve->pdev->dev, &dev_attr_headphone);
	if (ret < 0)
		goto dev_file_create_failed;

	tve->dac_reg = regulator_get(&tve->pdev->dev, plat_data->dac_reg);
	if (!IS_ERR(tve->dac_reg)) {
		regulator_set_voltage(tve->dac_reg, 2750000, 2750000);
		regulator_enable(tve->dac_reg);
	}
	tve->dig_reg = regulator_get(&tve->pdev->dev, plat_data->dig_reg);
	if (!IS_ERR(tve->dig_reg)) {
		regulator_set_voltage(tve->dig_reg, 1250000, 1250000);
		regulator_enable(tve->dig_reg);
	}

	tve->clk = clk_get(&tve->pdev->dev, "tve_clk");
	if (IS_ERR(tve->clk)) {
		ret = PTR_ERR(tve->clk);
		goto get_tveclk_failed;
	}
	tve->di_clk = clk_get(NULL, "ipu1_di1_clk");
	if (IS_ERR(tve->di_clk)) {
		ret = PTR_ERR(tve->di_clk);
		goto get_diclk_failed;
	}

	clk_set_rate(tve->clk, 216000000);
	clk_set_parent(tve->di_clk, tve->clk);
	clk_enable(tve->clk);

	tve->revision = _tve_get_revision(tve);
	if (tve->revision == 1) {
		tve->regs = &tve_regs_v1;
		tve->reg_fields = &tve_reg_fields_v1;
	} else {
		tve->regs = &tve_regs_v2;
		tve->reg_fields = &tve_reg_fields_v2;
	}

	/* adjust video mode for mx37 */
	if (cpu_is_mx37()) {
		video_modes_tve[0].left_margin = 121;
		video_modes_tve[0].right_margin = 16;
		video_modes_tve[0].upper_margin = 17;
		video_modes_tve[0].lower_margin = 5;
		video_modes_tve[1].left_margin = 131;
		video_modes_tve[1].right_margin = 12;
		video_modes_tve[1].upper_margin = 21;
		video_modes_tve[1].lower_margin = 3;
	}

	if (vga && cpu_is_mx53()) {
		setting->if_fmt = IPU_PIX_FMT_GBR24;
		modedb = video_modes_vga;
		modedb_sz = vga_modedb_sz;
	} else {
		setting->if_fmt = IPU_PIX_FMT_YUV444;
		if (tve->revision == 1) {
			modedb = video_modes_tve;
			modedb_sz = 3;
		} else {
			modedb = video_modes_tve;
			modedb_sz = tve_modedb_sz;
		}
	}

	fb_videomode_to_modelist(modedb, modedb_sz, &setting->fbi->modelist);

	/* must use spec video mode defined by driver */
	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (ret != 1)
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);

	ret = request_irq(tve->irq, tve_detect_handler, 0, tve->pdev->name, tve);
	if (ret < 0)
		goto req_irq_failed;

	/* Setup cable detect, for YPrPb mode, default use channel#-1 for Y */
	INIT_DELAYED_WORK(&tve->cd_work, cd_work_func);
	if (tve->revision == 1)
		writel(0x01067701, tve->base + tve->regs->tve_cd_cont_reg);
	else
		writel(0x00770601, tve->base + tve->regs->tve_cd_cont_reg);

	conf_reg = 0;
	writel(conf_reg, tve->base + tve->regs->tve_com_conf_reg);

	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg - 4 * 5);
	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg - 4 * 4);
	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg - 4 * 3);
	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg - 4 * 2);
	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg - 4);
	writel(0x00000000, tve->base + tve->regs->tve_mv_cont_reg);

	clk_disable(tve->clk);

	tve->nb.notifier_call = tve_fb_event;
	ret = fb_register_client(&tve->nb);
	if (ret < 0)
		goto reg_fbclient_failed;

	dev_set_drvdata(&tve->pdev->dev, tve);

	spin_lock_init(&tve->tve_lock);

	tve->inited = true;

	return 0;

reg_fbclient_failed:
	free_irq(tve->irq, tve->pdev);
req_irq_failed:
get_diclk_failed:
get_tveclk_failed:
	device_remove_file(&tve->pdev->dev, &dev_attr_headphone);
dev_file_create_failed:
	iounmap(tve->base);
ioremap_failed:
get_irq_failed:
get_res_failed:
	return ret;

}

static int tvout_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	return tve_drv_init(disp, 0, setting);
}

static int vga_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	return tve_drv_init(disp, 1, setting);
}

void tvout_deinit(struct mxc_dispdrv_handle *disp)
{
	struct tve_data *tve = mxc_dispdrv_getdata(disp);

	if (tve->enabled)
		clk_disable(tve->clk);

	fb_unregister_client(&tve->nb);
	free_irq(tve->irq, tve->pdev);
	device_remove_file(&tve->pdev->dev, &dev_attr_headphone);
	iounmap(tve->base);
}

static struct mxc_dispdrv_driver tve_drv = {
	.name 	= DISPDRV_TVE,
	.init  	= tvout_init,
	.deinit = tvout_deinit,
	.setup = tve_disp_setup,
};

static struct mxc_dispdrv_driver vga_drv = {
	.name 	= DISPDRV_VGA,
	.init 	= vga_init,
	.deinit	= tvout_deinit,
};

static int tve_dispdrv_init(struct tve_data *tve)
{
	tve->disp_tve = mxc_dispdrv_register(&tve_drv);
	mxc_dispdrv_setdata(tve->disp_tve, tve);
	tve->disp_vga = mxc_dispdrv_register(&vga_drv);
	mxc_dispdrv_setdata(tve->disp_vga, tve);
	return 0;
}

static void tve_dispdrv_deinit(struct tve_data *tve)
{
	mxc_dispdrv_puthandle(tve->disp_tve);
	mxc_dispdrv_puthandle(tve->disp_vga);
	mxc_dispdrv_unregister(tve->disp_tve);
	mxc_dispdrv_unregister(tve->disp_vga);
}

static int tve_probe(struct platform_device *pdev)
{
	int ret;
	struct tve_data *tve;

	tve = kzalloc(sizeof(struct tve_data), GFP_KERNEL);
	if (!tve) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	tve->pdev = pdev;
	ret = tve_dispdrv_init(tve);
	if (ret < 0)
		goto dispdrv_init_failed;

	dev_set_drvdata(&pdev->dev, tve);

	return 0;

dispdrv_init_failed:
	kfree(tve);
alloc_failed:
	return ret;
}

static int tve_remove(struct platform_device *pdev)
{
	struct tve_data *tve = dev_get_drvdata(&pdev->dev);

	tve_dispdrv_deinit(tve);
	kfree(tve);
	return 0;
}

static struct platform_driver tve_driver = {
	.driver = {
		   .name = "mxc_tve",
		   },
	.probe = tve_probe,
	.remove = tve_remove,
};

static int __init tve_init(void)
{
	return platform_driver_register(&tve_driver);
}

static void __exit tve_exit(void)
{
	platform_driver_unregister(&tve_driver);
}

module_init(tve_init);
module_exit(tve_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX TV encoder driver");
MODULE_LICENSE("GPL");
