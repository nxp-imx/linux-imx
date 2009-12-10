/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
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

#define TVOUT_FMT_OFF			0
#define TVOUT_FMT_NTSC			1
#define TVOUT_FMT_PAL			2
#define TVOUT_FMT_720P60		3

static int enabled;		/* enable power on or not */

static struct fb_info *tve_fbi;

struct tve_data {
	struct platform_device *pdev;
	int revision;
	int cur_mode;
	int output_mode;
	int detect;
	void *base;
	int irq;
	struct clk *clk;
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
	0, 0x14, 0x28, 0x2C, 0x48
};

static struct tve_reg_fields_mapping tve_reg_fields_v1 = {
	1, 2, 1, 2, 4, 0x00010000, 0x7000, 0x70, 12, 4, 8
};

static struct tve_reg_mapping tve_regs_v2 = {
	0, 0x34, 0x64, 0x68, 0xDC
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
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 720p60 TV output */
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT |
			FB_SYNC_EXT,
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
	RGB
};

static unsigned short tvout_mode_to_channel_map[8] = {
	0,	/* TV_OFF */
	1,	/* CVBS0 */
	4,	/* CVBS2 */
	5,	/* CVBS02 */
	1,	/* SVIDEO */
	5,	/* SVIDEO_CVBS */
	1,	/* YPBPR */
	7	/* RGB */
};


static void tve_set_tvout_mode(int mode)
{
	u32 conf_reg;

	conf_reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
	conf_reg &= ~(tve_reg_fields->sync_ch_mask |
				tve_reg_fields->tvout_mode_mask);
	/* clear sync_ch and tvout_mode fields */
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
	if (mode == YPBPR || mode == RGB) {
		return 1;
	} else {
		return 0;
	}
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
	struct clk *pll3_clk;
	unsigned long pll3_clock_rate = 216000000;
	struct clk *ipu_di0_clk;

	if (tve.cur_mode == mode)
		return 0;

	tve.cur_mode = mode;

	switch (mode) {
	case TVOUT_FMT_PAL:
	case TVOUT_FMT_NTSC:
		pll3_clock_rate = 216000000;
		break;
	case TVOUT_FMT_720P60:
		pll3_clock_rate = 297000000;
		break;
	}
	if (enabled)
		clk_disable(tve.clk);

	pll3_clk = clk_get(NULL, "pll3");
	ipu_di0_clk = clk_get(NULL, "ipu_di0_clk");
	if ((clk_get_parent(ipu_di0_clk) == pll3_clk) &&
		(clk_get_rate(pll3_clk) != pll3_clock_rate)) {
		printk(KERN_INFO "Cannot setup TV since display is using PLL3\n");
		return -EINVAL;
	}

	clk_disable(pll3_clk);
	clk_set_rate(pll3_clk, pll3_clock_rate);
	clk_enable(pll3_clk);

	clk_enable(tve.clk);

	/* select output video format */
	if (mode == TVOUT_FMT_PAL) {
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_PAL_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to PAL video\n");
	} else if (mode == TVOUT_FMT_NTSC) {
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_NTSC_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to NTSC video\n");
	} else if (mode == TVOUT_FMT_720P60) {
		if (!_is_tvout_mode_hd_compatible()) {
			tve_set_tvout_mode(YPBPR);
			pr_debug("The TV out mode is HD incompatible. Setting to YPBPR.");
		}
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		reg = (reg & ~TVE_STAND_MASK) | TVE_HD720P60_STAND;
		__raw_writel(reg, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to 720P60 video\n");
	} else if (mode == TVOUT_FMT_OFF) {
		__raw_writel(0x0, tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE: change to OFF video\n");
	} else {
		pr_debug("TVE: no such video format.\n");
		if (!enabled)
			clk_disable(tve.clk);
		return -EINVAL;
	}

	if (!enabled)
		clk_disable(tve.clk);

	return 0;
}

/**
 * tve_enable
 * Enable the tve Power to begin TV encoder
 */
static void tve_enable(void)
{
	u32 reg;

	if (!enabled) {
		enabled = 1;
		clk_enable(tve.clk);
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		__raw_writel(reg | TVE_IPU_CLK_ENABLE | TVE_ENABLE,
					tve.base + tve_regs->tve_com_conf_reg);
		pr_debug("TVE power on.\n");
	}

	/* enable interrupt */
	__raw_writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT,
				tve.base + tve_regs->tve_stat_reg);
	__raw_writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT,
				tve.base + tve_regs->tve_int_cont_reg);
}

/**
 * tve_disable
 * Disable the tve Power to stop TV encoder
 */
static void tve_disable(void)
{
	u32 reg;

	if (enabled) {
		enabled = 0;
		reg = __raw_readl(tve.base + tve_regs->tve_com_conf_reg);
		__raw_writel(reg & ~TVE_ENABLE & ~TVE_IPU_CLK_ENABLE,
				tve.base + tve_regs->tve_com_conf_reg);
		clk_disable(tve.clk);
		pr_debug("TVE power off.\n");
	}
}

static int tve_update_detect_status(void)
{
	int old_detect = tve.detect;
	u32 stat_lm, stat_sm, stat;
	u32 int_ctl = __raw_readl(tve.base + tve_regs->tve_int_cont_reg);
	u32 cd_cont_reg =
		__raw_readl(tve.base + tve_regs->tve_cd_cont_reg);
	u32 timeout = 40;

	if ((cd_cont_reg & 0x1) == 0) {
		pr_warning("Warning: pls enable TVE CD first!\n");
		return tve.detect;
	}

	stat = __raw_readl(tve.base + tve_regs->tve_stat_reg);
	while (((stat & CD_MON_END_INT) == 0) && (timeout > 0)) {
		msleep(2);
		timeout -= 2;
		stat = __raw_readl(tve.base + tve_regs->tve_stat_reg);
	}
	if (((stat & CD_MON_END_INT) == 0) && (timeout <= 0)) {
		pr_warning("Warning: get detect resultwithout CD_MON_END_INT!\n");
		return tve.detect;
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

int tve_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		pr_debug("fb registered event\n");
		if ((tve_fbi != NULL) || strcmp(fbi->fix.id, "DISP3 BG - DI1"))
			break;

		tve_fbi = fbi;
		fb_add_videomode(&video_modes[0], &tve_fbi->modelist);
		fb_add_videomode(&video_modes[1], &tve_fbi->modelist);
		fb_add_videomode(&video_modes[2], &tve_fbi->modelist);
		break;
	case FB_EVENT_MODE_CHANGE:
	{
		struct fb_videomode cur_mode;
		struct fb_videomode *mode;
		struct list_head *pos;
		struct fb_modelist *modelist;

		if (tve_fbi != fbi)
			break;

		fb_var_to_videomode(&cur_mode, &fbi->var);

		list_for_each(pos, &tve_fbi->modelist) {
			modelist = list_entry(pos, struct fb_modelist, list);
			mode = &modelist->mode;
			if (fb_mode_is_equal(&cur_mode, mode)) {
				fbi->mode = mode;
				break;
			}
		}

		if (!fbi->mode) {
			tve_disable();
			tve.cur_mode = TVOUT_FMT_OFF;
			return 0;
		}

		pr_debug("fb mode change event: xres=%d, yres=%d\n",
			 fbi->mode->xres, fbi->mode->yres);

		tve_disable();

		if (fb_mode_is_equal(fbi->mode, &video_modes[0])) {
			tve_setup(TVOUT_FMT_NTSC);
			tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[1])) {
			tve_setup(TVOUT_FMT_PAL);
			tve_enable();
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[2])) {
			tve_setup(TVOUT_FMT_720P60);
			tve_enable();
		} else {
			tve_setup(TVOUT_FMT_OFF);
		}
		break;
	}
	case FB_EVENT_BLANK:
		if ((tve_fbi != fbi) || (fbi->mode == NULL))
			return 0;

		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
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
			} else {
				tve_setup(TVOUT_FMT_OFF);
			}
		} else
			tve_disable();
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
	int ret, i;
	struct resource *res;
	struct tve_platform_data *plat_data = pdev->dev.platform_data;
	u32 conf_reg;

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
			break;
		}
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

	if (tve_fbi != NULL) {
		fb_add_videomode(&video_modes[0], &tve_fbi->modelist);
		fb_add_videomode(&video_modes[1], &tve_fbi->modelist);
		fb_add_videomode(&video_modes[2], &tve_fbi->modelist);
	}

	tve.dac_reg = regulator_get(&pdev->dev, plat_data->dac_reg);
	if (!IS_ERR(tve.dac_reg)) {
		regulator_set_voltage(tve.dac_reg, 2500000, 2500000);
		regulator_enable(tve.dac_reg);
	}

	tve.dig_reg = regulator_get(&pdev->dev, plat_data->dig_reg);
	if (!IS_ERR(tve.dig_reg)) {
		regulator_set_voltage(tve.dig_reg, 1250000, 1250000);
		regulator_enable(tve.dig_reg);
	}

	tve.clk = clk_get(&pdev->dev, "tve_clk");
	clk_set_rate(tve.clk, 216000000);
	clk_enable(tve.clk);

	tve.revision = _tve_get_revision();
	if (tve.revision == 1) {
		tve_regs = &tve_regs_v1;
		tve_reg_fields = &tve_reg_fields_v1;
	} else {
		tve_regs = &tve_regs_v2;
		tve_reg_fields = &tve_reg_fields_v2;
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

/*!
 * PM suspend/resume routing
 */
static int tve_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (enabled) {
		__raw_writel(0, tve.base + tve_regs->tve_int_cont_reg);
		__raw_writel(0, tve.base + tve_regs->tve_cd_cont_reg);
		__raw_writel(0, tve.base + tve_regs->tve_com_conf_reg);
		clk_disable(tve.clk);
	}
	return 0;
}

static int tve_resume(struct platform_device *pdev)
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
		}
		tve_enable();
	}

	return 0;
}

static struct platform_driver tve_driver = {
	.driver = {
		   .name = "tve",
		   },
	.probe = tve_probe,
	.remove = tve_remove,
	.suspend = tve_suspend,
	.resume = tve_resume,
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
