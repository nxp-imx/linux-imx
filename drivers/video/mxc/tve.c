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
#define TVOUT_FMT_VGA_XGA		10
#define TVOUT_FMT_VGA_SXGA		11

static int enabled;		/* enable power on or not */
DEFINE_SPINLOCK(tve_lock);

static struct fb_info *tve_fbi;
static struct fb_modelist tve_modelist;
static bool g_enable_tve;

struct tve_data {
	struct platform_device *pdev;
	int revision;
	int cur_mode;
	int output_mode;
	int detect;
	void *base;
	int irq;
	int blank;
	struct clk *clk;
	struct clk *di_clk;
	struct regulator *dac_reg;
	struct regulator *dig_reg;
	struct delayed_work cd_work;
} tve;

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


struct tve_reg_mapping *tve_regs;
struct tve_reg_fields_mapping *tve_reg_fields;

/* For MX37 need modify some fields in tve_probe */
static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 122, 15,
	 18, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 720p60 TV output */
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 720p30 TV output */
	 "720P30", 30, 1280, 720, 13468,
	 260, 1759,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080i60 TV output */
	 "1080I60", 60, 1920, 1080, 13468,
	 192, 87,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080i50 TV output */
	 "1080I50", 50, 1920, 1080, 13468,
	 192, 527,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080p30 TV output */
	 "1080P30", 30, 1920, 1080, 13468,
	 192, 87,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p25 TV output */
	 "1080P25", 25, 1920, 1080, 13468,
	 192, 527,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p24 TV output */
	 "1080P24", 24, 1920, 1080, 13468,
	 192, 637,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
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
	{
	/* VGA 1280x1024 108M pixel clk output */
	"SXGA", 60, 1280, 1024, 9259,
	48, 248,
	1, 38,
	112, 3,
	0,
	FB_VMODE_NONINTERLACED,
	0,},
};

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

static void tve_dump_regs(void)
{
	dev_dbg(&tve.pdev->dev, "tve_com_conf_reg 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_com_conf_reg));
	dev_dbg(&tve.pdev->dev, "tve_cd_cont_reg 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_cd_cont_reg));
	dev_dbg(&tve.pdev->dev, "tve_int_cont_reg 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_int_cont_reg));
	dev_dbg(&tve.pdev->dev, "tve_tst_mode_reg 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_tst_mode_reg));
	dev_dbg(&tve.pdev->dev, "tve_tvdac_cont_reg0 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg));
	dev_dbg(&tve.pdev->dev, "tve_tvdac_cont_reg1 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg + 4));
	dev_dbg(&tve.pdev->dev, "tve_tvdac_cont_reg2 0x%x\n",
			__raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg + 8));
}

static int is_vga_mode(void)
{
	u32 reg;

	if (tve.revision == 2) {
		reg = __raw_readl(tve.base + tve_regs->tve_tst_mode_reg);
		if (reg & TVEV2_DAC_TEST_MODE_MASK)
			return 1;
		else
			return 0;
	}
	return 0;
}

static void tve_disable_vga_mode(void)
{
	if (tve.revision == 2) {
		u32 reg;
		/* disable test mode */
		reg = __raw_readl(tve.base + tve_regs->tve_tst_mode_reg);
		reg = reg & ~TVEV2_DAC_TEST_MODE_MASK;
		__raw_writel(reg, tve.base + tve_regs->tve_tst_mode_reg);
	}
}

static void tve_set_tvout_mode(int mode)
{
	u32 conf_reg;

	/* clear sync_ch and tvout_mode fields */
	conf_reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
	conf_reg &= ~(tve_reg_fields->sync_ch_mask |
				tve_reg_fields->tvout_mode_mask);

	conf_reg = conf_reg & ~TVE_DAC_SAMPRATE_MASK;
	if (tve.revision == 2) {
		conf_reg = (conf_reg & ~TVEV2_DATA_SRC_MASK) |
			TVEV2_DATA_SRC_BUS_1;
		conf_reg = conf_reg & ~TVEV2_INP_VIDEO_FORM;
		conf_reg = conf_reg & ~TVEV2_P2I_CONV_EN;
	}

	conf_reg |=
		mode << tve_reg_fields->
		tvout_mode_offset | tvout_mode_to_channel_map[mode] <<
		tve_reg_fields->sync_ch_offset;
	__raw_writel(conf_reg, tve.base + tve_regs->tve_com_conf_reg);
}

static int _is_tvout_mode_hd_compatible(void)
{
	u32 conf_reg, mode;

	conf_reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
	mode = (conf_reg >> tve_reg_fields->tvout_mode_offset) & 7;
	if (mode == YPBPR || mode == TVRGB) {
		return 1;
	} else {
		return 0;
	}
}

static int tve_setup_vga(void)
{
	u32 reg;

	if (tve.revision == 2) {
		/* set gain */
		reg = __raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0;
		__raw_writel(reg, tve.base + tve_regs->tve_tvdac_cont_reg);
		reg = __raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg + 4);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0;
		__raw_writel(reg, tve.base + tve_regs->tve_tvdac_cont_reg + 4);
		reg = __raw_readl(tve.base + tve_regs->tve_tvdac_cont_reg + 8);
		reg = (reg & ~TVEV2_DAC_GAIN_MASK) | 0;
		__raw_writel(reg, tve.base + tve_regs->tve_tvdac_cont_reg + 8);

		/* set tve_com_conf_reg  */
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_DAC_SAMPRATE_MASK) | TVE_DAC_DIV2_RATE;
		reg = (reg & ~TVEV2_DATA_SRC_MASK) | TVEV2_DATA_SRC_BUS_2;
		reg = reg | TVEV2_INP_VIDEO_FORM;
		reg = reg & ~TVEV2_P2I_CONV_EN;
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P30_STAND;
		reg |= TVRGB << tve_reg_fields->tvout_mode_offset |
			1 << tve_reg_fields->sync_ch_offset;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);

		/* set test mode */
		reg = __raw_readl(tve.base + tve_regs->tve_tst_mode_reg);
		reg = (reg & ~TVEV2_DAC_TEST_MODE_MASK) | 1;
		__raw_writel(reg, tve.base + tve_regs->tve_tst_mode_reg);
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
static int tve_setup(int mode)
{
	u32 reg;
	struct clk *tve_parent_clk;
	unsigned long parent_clock_rate = 216000000, di1_clock_rate = 27000000;
	unsigned long tve_clock_rate = 216000000;
	unsigned long lock_flags;

	if (tve.cur_mode == mode)
		return 0;

	spin_lock_irqsave(&tve_lock, lock_flags);

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
	case TVOUT_FMT_VGA_XGA:
		parent_clock_rate = 260000000;
		tve_clock_rate = 130000000;
		di1_clock_rate = 65000000;
		break;
	case TVOUT_FMT_VGA_SXGA:
		parent_clock_rate = 216000000;
		di1_clock_rate = 108000000;
		break;
	}
	if (enabled)
		clk_disable(tve.clk);

	tve_parent_clk = clk_get_parent(tve.clk);

	clk_set_rate(tve_parent_clk, parent_clock_rate);

	tve_clock_rate = clk_round_rate(tve.clk, tve_clock_rate);
	clk_set_rate(tve.clk, tve_clock_rate);

	clk_enable(tve.clk);
	di1_clock_rate = clk_round_rate(tve.di_clk, di1_clock_rate);
	clk_set_rate(tve.di_clk, di1_clock_rate);

	tve.cur_mode = mode;

	/* select output video format */
	if (mode == TVOUT_FMT_PAL) {
		tve_disable_vga_mode();
		tve_set_tvout_mode(YPBPR);
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_PAL_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to PAL video\n");
	} else if (mode == TVOUT_FMT_NTSC) {
		tve_disable_vga_mode();
		tve_set_tvout_mode(YPBPR);
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_NTSC_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to NTSC video\n");
	} else if (mode == TVOUT_FMT_720P60) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD720P60_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 720P60 video\n");
	} else if (mode == TVOUT_FMT_720P30) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD720P30_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 720P30 video\n");
	} else if (mode == TVOUT_FMT_1080I60) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080I60_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 1080I60 video\n");
	} else if (mode == TVOUT_FMT_1080I50) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080I50_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 1080I50 video\n");
	} else if (mode == TVOUT_FMT_1080P30) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P30_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 1080P30 video\n");
	} else if (mode == TVOUT_FMT_1080P25) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P25_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 1080P25 video\n");
	} else if (mode == TVOUT_FMT_1080P24) {
		tve_disable_vga_mode();
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD1080P24_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 1080P24 video\n");
	} else if ((mode == TVOUT_FMT_VGA_XGA) || (mode == TVOUT_FMT_VGA_SXGA)) {
		/* do not need cable detect */
		tve_setup_vga();
		pr_debug("TVE: change to VGA video\n");
	} else if (mode == TVOUT_FMT_OFF) {
		__raw_writel(0x0, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to OFF video\n");
	} else {
		pr_debug("TVE: no such video format.\n");
	}

	if (!enabled)
		clk_disable(tve.clk);

	spin_unlock_irqrestore(&tve_lock, lock_flags);
	return 0;
}

/**
 * tve_enable
 * Enable the tve Power to begin TV encoder
 */
static void tve_enable(void)
{
	u32 reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&tve_lock, lock_flags);
	if (!enabled) {
		enabled = 1;
		clk_enable(tve.clk);
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		__raw_writel(reg | TVE_IPU_CLK_ENABLE | TVE_ENABLE,
					tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE power on.\n");
	}

	if (is_vga_mode()) {
		/* disable interrupt */
		pr_debug("TVE VGA disable cable detect.\n");
		__raw_writel(0xffffffff, tve.base + tve_regs->tve_stat_reg);
		__raw_writel(0, tve.base + tve_regs->tve_int_cont_reg);
	} else {
		/* enable interrupt */
		pr_debug("TVE TVE enable cable detect.\n");
		__raw_writel(0xffffffff, tve.base + tve_regs->tve_stat_reg);
		__raw_writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT,
				tve.base + tve_regs->tve_int_cont_reg);
	}

	spin_unlock_irqrestore(&tve_lock, lock_flags);

	tve_dump_regs();
}

/**
 * tve_disable
 * Disable the tve Power to stop TV encoder
 */
static void tve_disable(void)
{
	u32 reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&tve_lock, lock_flags);
	if (enabled) {
		enabled = 0;
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		__raw_writel(reg & ~TVE_ENABLE & ~TVE_IPU_CLK_ENABLE,
				tve.base + tve_regs->tve_com_conf_reg);
		clk_disable(tve.clk);
		pr_debug("TVE power off.\n");
	}
	spin_unlock_irqrestore(&tve_lock, lock_flags);
}

static int tve_update_detect_status(void)
{
	int old_detect = tve.detect;
	u32 stat_lm, stat_sm, stat;
	u32 int_ctl;
	u32 cd_cont_reg;
	u32 timeout = 40;
	unsigned long lock_flags;

	spin_lock_irqsave(&tve_lock, lock_flags);

	if (!enabled) {
		pr_warning("Warning: update tve status while it disabled!\n");
		tve.detect = 0;
		goto done;
	}

	int_ctl = __raw_readl(tve.base + tve_regs->tve_int_cont_reg);
	cd_cont_reg = __raw_readl(tve.base + tve_regs->tve_cd_cont_reg);

	if ((cd_cont_reg & 0x1) == 0) {
		pr_warning("Warning: pls enable TVE CD first!\n");
		goto done;
	}

	stat = __raw_readl(tve.base + tve_regs->tve_stat_reg);
	while (((stat & CD_MON_END_INT) == 0) && (timeout > 0)) {
		spin_unlock_irqrestore(&tve_lock, lock_flags);
		msleep(2);
		spin_lock_irqsave(&tve_lock, lock_flags);
		timeout -= 2;
		if (!enabled) {
			pr_warning("Warning: update tve status while it disabled!\n");
			tve.detect = 0;
			goto done;
		} else
			stat = __raw_readl(tve.base + tve_regs->tve_stat_reg);
	}
	if (((stat & CD_MON_END_INT) == 0) && (timeout <= 0)) {
		pr_warning("Warning: get detect result without CD_MON_END_INT!\n");
		goto done;
	}

	stat = stat >> tve_reg_fields->cd_ch_stat_offset;
	stat_lm = stat & (CD_CH_0_LM_ST | CD_CH_1_LM_ST | CD_CH_2_LM_ST);
	if ((stat_lm == (CD_CH_0_LM_ST | CD_CH_1_LM_ST | CD_CH_2_LM_ST)) &&
		((stat & (CD_CH_0_SM_ST | CD_CH_1_SM_ST | CD_CH_2_SM_ST)) == 0)
		) {
			tve.detect = 3;
			tve.output_mode = YPBPR;
	} else if ((stat_lm == (CD_CH_0_LM_ST | CD_CH_1_LM_ST)) &&
		((stat & (CD_CH_0_SM_ST | CD_CH_1_SM_ST)) == 0)) {
			tve.detect = 4;
			tve.output_mode = SVIDEO;
	} else if (stat_lm == CD_CH_0_LM_ST) {
		stat_sm = stat & CD_CH_0_SM_ST;
		if (stat_sm != 0) {
			/* headset */
			tve.detect = 2;
			tve.output_mode = TV_OFF;
		} else {
			tve.detect = 1;
			tve.output_mode = CVBS0;
		}
	} else if (stat_lm == CD_CH_2_LM_ST) {
		stat_sm = stat & CD_CH_2_SM_ST;
		if (stat_sm != 0) {
			/* headset */
			tve.detect = 2;
			tve.output_mode = TV_OFF;
		} else {
			tve.detect = 1;
			tve.output_mode = CVBS2;
		}
	} else {
		/* none */
		tve.detect = 0;
		tve.output_mode = TV_OFF;
	}

	tve_set_tvout_mode(tve.output_mode);

	/* clear interrupt */
	__raw_writel(CD_MON_END_INT | CD_LM_INT | CD_SM_INT,
			tve.base + tve_regs->tve_stat_reg);

	__raw_writel(int_ctl | CD_SM_INT | CD_LM_INT,
			tve.base + tve_regs->tve_int_cont_reg);

	if (old_detect != tve.detect)
		sysfs_notify(&tve.pdev->dev.kobj, NULL, "headphone");

	dev_dbg(&tve.pdev->dev, "detect = %d mode = %d\n",
			tve.detect, tve.output_mode);
done:
	spin_unlock_irqrestore(&tve_lock, lock_flags);
	return tve.detect;
}

static void cd_work_func(struct work_struct *work)
{
	tve_update_detect_status();
}
#if 0
static int tve_man_detect(void)
{
	u32 cd_cont;
	u32 int_cont;

	if (!enabled)
		return -1;

	int_cont = __raw_readl(tve.base + tve_regs->tve_int_cont_reg);
	__raw_writel(int_cont &
				~(tve_reg_fields->cd_sm_int | tve_reg_fields->cd_lm_int),
				tve.base + tve_regs->tve_int_cont_reg);

	cd_cont = __raw_readl(tve.base + tve_regs->tve_cd_cont_reg);
	__raw_writel(cd_cont | tve_reg_fields->cd_trig_mode,
				tve.base + tve_regs->tve_cd_cont_reg);

	__raw_writel(tve_reg_fields->cd_sm_int | tve_reg_fields->
			cd_lm_int | tve_reg_fields->
			cd_mon_end_int | tve_reg_fields->cd_man_trig,
			tve.base + tve_regs->tve_stat_reg);

	while ((__raw_readl(tve.base + tve_regs->tve_stat_reg)
		& tve_reg_fields->cd_mon_end_int) == 0)
		msleep(5);

	tve_update_detect_status();

	__raw_writel(cd_cont, tve.base + tve_regs->tve_cd_cont_reg);
	__raw_writel(int_cont, tve.base + tve_regs->tve_int_cont_reg);

	return tve.detect;
}
#endif

static irqreturn_t tve_detect_handler(int irq, void *data)
{
	u32 int_ctl = __raw_readl(tve.base + tve_regs->tve_int_cont_reg);

	/* disable INT first */
	int_ctl &= ~(CD_SM_INT | CD_LM_INT | CD_MON_END_INT);
	__raw_writel(int_ctl, tve.base + tve_regs->tve_int_cont_reg);

	__raw_writel(CD_MON_END_INT | CD_LM_INT | CD_SM_INT,
			tve.base + tve_regs->tve_stat_reg);

	schedule_delayed_work(&tve.cd_work, msecs_to_jiffies(1000));

	return IRQ_HANDLED;
}

static void ipu_set_vga_delay(struct fb_info *fbi, uint32_t hsync_delay, uint32_t vsync_delay)
{
	uint32_t ipu_ch = CHAN_NONE;
	uint32_t hsync_polarity = 0, vsync_polarity = 0;
	mm_segment_t old_fs;

	pr_debug("TVE VGA set delay hsync/vsync\n");

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_GET_FB_IPU_CHAN,
				(unsigned long)&ipu_ch);
		set_fs(old_fs);
	}
	if (ipu_ch == CHAN_NONE) {
		pr_warning("TVE Can not get display ipu channel\n");
		return;
	}

	if (fbi->var.sync & FB_SYNC_HOR_HIGH_ACT)
		hsync_polarity = 1;
	if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT)
		vsync_polarity = 1;
	ipu_disable_channel(ipu_ch, 1);
	ipu_set_vga_delayed_hsync_vsync(fbi->var.xres, fbi->var.yres,
			fbi->var.left_margin, fbi->var.hsync_len,
			fbi->var.right_margin, fbi->var.upper_margin,
			fbi->var.vsync_len, fbi->var.lower_margin,
			hsync_delay, vsync_delay, hsync_polarity,
			vsync_polarity);
	ipu_enable_channel(ipu_ch);
}

static inline void tve_set_di_fmt(struct fb_info *fbi, unsigned int fmt)
{
	mm_segment_t old_fs;

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_SET_DIFMT, (unsigned long)&fmt);
		set_fs(old_fs);
	}
}

/*!
 * FB suspend/resume routing
 */
static int tve_suspend(void)
{
	if (enabled) {
		__raw_writel(0, tve.base + tve_regs->tve_int_cont_reg);
		__raw_writel(0, tve.base + tve_regs->tve_cd_cont_reg);
		__raw_writel(0, tve.base + tve_regs->tve_com_conf_reg);
		clk_disable(tve.clk);
	}
	return 0;
}

static int tve_resume(struct fb_info *fbi)
{
	if (enabled) {
		clk_enable(tve.clk);

		/* Setup cable detect */
		if (tve.revision == 1)
			__raw_writel(0x01067701,
				tve.base + tve_regs->tve_cd_cont_reg);
		else
			__raw_writel(0x00770601,
				tve.base + tve_regs->tve_cd_cont_reg);

		if (tve.cur_mode == TVOUT_FMT_NTSC) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_NTSC);
		} else if (tve.cur_mode == TVOUT_FMT_PAL) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_PAL);
		} else if (tve.cur_mode == TVOUT_FMT_720P60) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_720P60);
		} else if (tve.cur_mode == TVOUT_FMT_720P30) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_720P30);
		} else if (tve.cur_mode == TVOUT_FMT_1080I60) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_1080I60);
		} else if (tve.cur_mode == TVOUT_FMT_1080I50) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_1080I50);
		} else if (tve.cur_mode == TVOUT_FMT_1080P30) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_1080P30);
		} else if (tve.cur_mode == TVOUT_FMT_1080P25) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_1080P25);
		} else if (tve.cur_mode == TVOUT_FMT_1080P24) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_1080P24);
		} else if (tve.cur_mode == TVOUT_FMT_VGA_XGA) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_VGA_XGA);
			ipu_set_vga_delay(fbi, 1421, 803);
		} else if (tve.cur_mode == TVOUT_FMT_VGA_SXGA) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			tve_setup(TVOUT_FMT_VGA_SXGA);
			ipu_set_vga_delay(fbi, 1504, 1030);
		}
		tve_enable();
	}

	return 0;
}

int tve_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	if (strcmp(fbi->fix.id, "DISP3 BG - DI1"))
		return 0;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		pr_debug("fb registered event\n");
		if (tve_fbi != NULL)
			break;

		tve_fbi = fbi;
		fb_add_videomode(&video_modes[0], &tve_modelist.list);
		fb_add_videomode(&video_modes[1], &tve_modelist.list);
		fb_add_videomode(&video_modes[2], &tve_modelist.list);
		if (tve.revision == 2) {
			fb_add_videomode(&video_modes[3], &tve_modelist.list);
			fb_add_videomode(&video_modes[4], &tve_modelist.list);
			fb_add_videomode(&video_modes[5], &tve_modelist.list);
			fb_add_videomode(&video_modes[6], &tve_modelist.list);
			fb_add_videomode(&video_modes[7], &tve_modelist.list);
			fb_add_videomode(&video_modes[8], &tve_modelist.list);
			if (cpu_is_mx53()) {
				fb_add_videomode(&video_modes[9], &tve_modelist.list);
				fb_add_videomode(&video_modes[10], &tve_modelist.list);
			}
		}
		break;
	case FB_EVENT_MODE_CHANGE:
	{
		if (tve_fbi != fbi)
			break;

		fbi->mode = (struct fb_videomode *)fb_match_mode(&tve_fbi->var,
				&tve_modelist.list);

		if (!fbi->mode) {
			pr_warning("TVE: can not find mode for xres=%d, yres=%d\n",
					fbi->var.xres, fbi->var.yres);
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			return 0;
		}

		pr_debug("TVE: fb mode change event: xres=%d, yres=%d\n",
			 fbi->mode->xres, fbi->mode->yres);

		if (fb_mode_is_equal(fbi->mode, &video_modes[0])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_NTSC);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[1])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_PAL);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[2])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_720P60);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[3])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_720P30);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[4])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_1080I60);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[5])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_1080I50);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[6])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_1080P30);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[7])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_1080P25);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[8])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_YUV444);
			tve_disable();
			tve_setup(TVOUT_FMT_1080P24);
			if (tve.blank == FB_BLANK_UNBLANK)
				tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[9])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_GBR24);
			tve_disable();
			tve_setup(TVOUT_FMT_VGA_XGA);
			if (tve.blank == FB_BLANK_UNBLANK) {
				tve_enable();
				ipu_set_vga_delay(fbi, 1421, 803);
			}
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[10])) {
			tve_set_di_fmt(fbi, IPU_PIX_FMT_GBR24);
			tve_disable();
			tve_setup(TVOUT_FMT_VGA_SXGA);
			if (tve.blank == FB_BLANK_UNBLANK) {
				tve_enable();
				ipu_set_vga_delay(fbi, 1504, 1030);
			}
		} else {
			tve_disable();
			tve_setup(TVOUT_FMT_OFF);
		}
		break;
	}
	case FB_EVENT_BLANK:
		if ((tve_fbi != fbi) || (fbi->mode == NULL))
			return 0;

		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			if (tve.blank != FB_BLANK_UNBLANK) {
				if (fb_mode_is_equal(fbi->mode, &video_modes[0])) {
					if (tve.cur_mode != TVOUT_FMT_NTSC) {
						tve_disable();
						tve_setup(TVOUT_FMT_NTSC);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[1])) {
					if (tve.cur_mode != TVOUT_FMT_PAL) {
						tve_disable();
						tve_setup(TVOUT_FMT_PAL);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[2])) {
					if (tve.cur_mode != TVOUT_FMT_720P60) {
						tve_disable();
						tve_setup(TVOUT_FMT_720P60);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[3])) {
					if (tve.cur_mode != TVOUT_FMT_720P30) {
						tve_disable();
						tve_setup(TVOUT_FMT_720P30);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[4])) {
					if (tve.cur_mode != TVOUT_FMT_1080I60) {
						tve_disable();
						tve_setup(TVOUT_FMT_1080I60);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[5])) {
					if (tve.cur_mode != TVOUT_FMT_1080I50) {
						tve_disable();
						tve_setup(TVOUT_FMT_1080I50);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[6])) {
					if (tve.cur_mode != TVOUT_FMT_1080P30) {
						tve_disable();
						tve_setup(TVOUT_FMT_1080P30);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[7])) {
					if (tve.cur_mode != TVOUT_FMT_1080P25) {
						tve_disable();
						tve_setup(TVOUT_FMT_1080P25);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[8])) {
					if (tve.cur_mode != TVOUT_FMT_1080P24) {
						tve_disable();
						tve_setup(TVOUT_FMT_1080P24);
					}
					tve_enable();
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[9])) {
					if (tve.cur_mode != TVOUT_FMT_VGA_XGA) {
						tve_disable();
						tve_setup(TVOUT_FMT_VGA_XGA);
					}
					tve_enable();
					ipu_set_vga_delay(fbi, 1421, 803);
				} else if (fb_mode_is_equal(fbi->mode,
							&video_modes[10])) {
					if (tve.cur_mode != TVOUT_FMT_VGA_SXGA) {
						tve_disable();
						tve_setup(TVOUT_FMT_VGA_SXGA);
					}
					tve_enable();
					ipu_set_vga_delay(fbi, 1504, 1030);
				} else {
					tve_setup(TVOUT_FMT_OFF);
				}
				tve.blank = FB_BLANK_UNBLANK;
			}
		} else {
			tve_disable();
			tve.blank = FB_BLANK_POWERDOWN;
		}
		break;
	case FB_EVENT_SUSPEND:
		tve_suspend();
		break;
	case FB_EVENT_RESUME:
		tve_resume(fbi);
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = tve_fb_event,
};

static ssize_t show_headphone(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int detect;

	if (!enabled) {
		strcpy(buf, "tve power off\n");
		return strlen(buf);
	}

	detect = tve_update_detect_status();

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

static int _tve_get_revision(void)
{
	u32 conf_reg;
	u32 rev = 0;

	/* find out TVE rev based on the base addr default value
	 * can be used at the init/probe ONLY */
	conf_reg = __raw_readl(tve.base);
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

static int tve_probe(struct platform_device *pdev)
{
	int ret, i, primary = 0;
	struct resource *res;
	struct tve_platform_data *plat_data = pdev->dev.platform_data;
	u32 conf_reg;

	if (g_enable_tve == false)
		return -EPERM;

	INIT_LIST_HEAD(&tve_modelist.list);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENOMEM;

	tve.pdev = pdev;
	tve.base = ioremap(res->start, res->end - res->start);

	tve.irq = platform_get_irq(pdev, 0);
	if (tve.irq < 0) {
		ret = tve.irq;
		goto err0;
	}

	ret = request_irq(tve.irq, tve_detect_handler, 0, pdev->name, pdev);
	if (ret < 0)
		goto err0;

	ret = device_create_file(&pdev->dev, &dev_attr_headphone);
	if (ret < 0)
		goto err1;

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG - DI1") == 0) {
			tve_fbi = registered_fb[i];
			if (i == 0) {
				pr_info("TVE as primary display\n");
				primary = 1;
				acquire_console_sem();
				fb_blank(tve_fbi, FB_BLANK_POWERDOWN);
				release_console_sem();
			}
			break;
		}
	}

	tve.dac_reg = regulator_get(&pdev->dev, plat_data->dac_reg);
	if (!IS_ERR(tve.dac_reg)) {
		regulator_set_voltage(tve.dac_reg, 2750000, 2750000);
		regulator_enable(tve.dac_reg);
	}

	tve.dig_reg = regulator_get(&pdev->dev, plat_data->dig_reg);
	if (!IS_ERR(tve.dig_reg)) {
		regulator_set_voltage(tve.dig_reg, 1250000, 1250000);
		regulator_enable(tve.dig_reg);
	}

	tve.clk = clk_get(&pdev->dev, "tve_clk");
	if (IS_ERR(tve.clk)) {
		ret = PTR_ERR(tve.clk);
		goto err2;
	}
	tve.di_clk = clk_get(NULL, "ipu_di1_clk");
	if (IS_ERR(tve.di_clk)) {
		ret = PTR_ERR(tve.di_clk);
		goto err2;
	}
	clk_set_rate(tve.clk, 216000000);
	clk_set_parent(tve.di_clk, tve.clk);
	clk_enable(tve.clk);

	tve.revision = _tve_get_revision();
	if (tve.revision == 1) {
		tve_regs = &tve_regs_v1;
		tve_reg_fields = &tve_reg_fields_v1;
	} else {
		tve_regs = &tve_regs_v2;
		tve_reg_fields = &tve_reg_fields_v2;
	}

	/* adjust video mode for mx37 */
	if (cpu_is_mx37()) {
		video_modes[0].left_margin = 121;
		video_modes[0].right_margin = 16;
		video_modes[0].upper_margin = 17;
		video_modes[0].lower_margin = 5;
		video_modes[1].left_margin = 131;
		video_modes[1].right_margin = 12;
		video_modes[1].upper_margin = 21;
		video_modes[1].lower_margin = 3;
	}

	fb_add_videomode(&video_modes[0], &tve_modelist.list);
	fb_add_videomode(&video_modes[1], &tve_modelist.list);
	fb_add_videomode(&video_modes[2], &tve_modelist.list);
	if (tve.revision == 2) {
		fb_add_videomode(&video_modes[3], &tve_modelist.list);
		fb_add_videomode(&video_modes[4], &tve_modelist.list);
		fb_add_videomode(&video_modes[5], &tve_modelist.list);
		fb_add_videomode(&video_modes[6], &tve_modelist.list);
		fb_add_videomode(&video_modes[7], &tve_modelist.list);
		fb_add_videomode(&video_modes[8], &tve_modelist.list);
		if (cpu_is_mx53()) {
			fb_add_videomode(&video_modes[9], &tve_modelist.list);
			fb_add_videomode(&video_modes[10], &tve_modelist.list);
		}
	}

	/* Setup cable detect, for YPrPb mode, default use channel#0 for Y */
	INIT_DELAYED_WORK(&tve.cd_work, cd_work_func);
	if (tve.revision == 1)
		__raw_writel(0x01067701, tve.base + tve_regs->tve_cd_cont_reg);
	else
		__raw_writel(0x00770601, tve.base + tve_regs->tve_cd_cont_reg);

	conf_reg = 0;
	__raw_writel(conf_reg, tve.base + tve_regs->tve_com_conf_reg);

	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg - 4 * 5);
	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg - 4 * 4);
	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg - 4 * 3);
	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg - 4 * 2);
	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg - 4);
	__raw_writel(0x00000000, tve.base + tve_regs->tve_mv_cont_reg);

	clk_disable(tve.clk);

	ret = fb_register_client(&nb);
	if (ret < 0)
		goto err2;

	tve.blank = -1;

	/* is primary display? */
	if (primary) {
		struct fb_var_screeninfo var;
		const struct fb_videomode *mode;

		memset(&var, 0, sizeof(var));
		mode = fb_match_mode(&tve_fbi->var, &tve_modelist.list);
		if (mode) {
			pr_debug("TVE: fb mode found\n");
			fb_videomode_to_var(&var, mode);
			var.yres_virtual = var.yres * 3;
		} else {
			pr_warning("TVE: can not find video mode\n");
			goto done;
		}
		acquire_console_sem();
		tve_fbi->flags |= FBINFO_MISC_USEREVENT;
		fb_set_var(tve_fbi, &var);
		tve_fbi->flags &= ~FBINFO_MISC_USEREVENT;
		release_console_sem();

		acquire_console_sem();
		fb_blank(tve_fbi, FB_BLANK_UNBLANK);
		release_console_sem();

		fb_show_logo(tve_fbi, 0);
	}

done:
	return 0;
err2:
	device_remove_file(&pdev->dev, &dev_attr_headphone);
err1:
	free_irq(tve.irq, pdev);
err0:
	iounmap(tve.base);
	return ret;
}

static int tve_remove(struct platform_device *pdev)
{
	if (enabled) {
		clk_disable(tve.clk);
		enabled = 0;
	}
	free_irq(tve.irq, pdev);
	device_remove_file(&pdev->dev, &dev_attr_headphone);
	fb_unregister_client(&nb);
	return 0;
}

static struct platform_driver tve_driver = {
	.driver = {
		   .name = "tve",
		   },
	.probe = tve_probe,
	.remove = tve_remove,
};

static int __init enable_tve_setup(char *options)
{
	g_enable_tve = true;

	return 1;
}
__setup("tve", enable_tve_setup);

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
