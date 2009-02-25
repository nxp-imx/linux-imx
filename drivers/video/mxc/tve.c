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
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <mach/gpio.h>

#define TVE_COM_CONF_REG	0
#define TVE_CD_CONT_REG		0x14
#define TVE_INT_CONT_REG	0x28
#define TVE_STAT_REG		0x2C
#define TVE_MV_CONT_REG		0x48

#define CD_EN			0x00000001
#define CD_TRIG_MODE		0x00000002

#define CD_LM_INT		0x00000001
#define CD_SM_INT		0x00000002
#define CD_MON_END_INT		0x00000004
#define CD_MAN_TRIG		0x00010000

#define TVOUT_FMT_OFF	0
#define TVOUT_FMT_NTSC	1
#define TVOUT_FMT_PAL	2

static int enabled;		/* enable power on or not */

static struct fb_info *tve_fbi;

struct tve_data {
	struct platform_device *pdev;
	int cur_mode;
	int detect;
	void *base;
	int irq;
	struct clk *clk;
	struct regulator *dac_reg;
	struct regulator *dig_reg;
} tve;

static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 121, 16,
	 17, 5,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 131, 12,
	 21, 3,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
};

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
	if (tve.cur_mode == mode)
		return 0;

	tve.cur_mode = mode;

	if (!enabled)
		clk_enable(tve.clk);

	/* select output video format */
	if (mode == TVOUT_FMT_PAL) {
		__raw_writel(0x00840328, tve.base + TVE_COM_CONF_REG);
		pr_debug("TVE: change to PAL video\n");
	} else if (mode == TVOUT_FMT_NTSC) {
		__raw_writel(0x00840028, tve.base + TVE_COM_CONF_REG);
		pr_debug("TVE: change to NTSC video\n");
	} else if (mode == TVOUT_FMT_OFF) {
		__raw_writel(0x0, tve.base + TVE_COM_CONF_REG);
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
		reg = __raw_readl(tve.base + TVE_COM_CONF_REG);
		__raw_writel(reg | 0x09, tve.base + TVE_COM_CONF_REG);
		pr_debug("TVE power on.\n");
	}
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
		reg = __raw_readl(tve.base + TVE_COM_CONF_REG);
		__raw_writel(reg & ~0x09, tve.base + TVE_COM_CONF_REG);
		clk_disable(tve.clk);
		pr_debug("TVE power off.\n");
	}
}

static int tve_update_detect_status(void)
{
	int old_detect = tve.detect;
	u32 stat = __raw_readl(tve.base + TVE_STAT_REG);

	if ((stat & CD_MON_END_INT) == 0)
		return tve.detect;

	if (stat & CD_LM_INT) {
		if (stat & CD_SM_INT)
			tve.detect = 2;
		else
			tve.detect = 1;
	} else {
		tve.detect = 0;
	}

	__raw_writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT,
		     tve.base + TVE_STAT_REG);

	if (old_detect != tve.detect)
		sysfs_notify(&tve.pdev->dev.kobj, NULL, "headphone");

	dev_dbg(&tve.pdev->dev, "detect = %d\n", tve.detect);
	return tve.detect;
}

static int tve_man_detect(void)
{
	u32 cd_cont;
	u32 int_cont;

	if (!enabled)
		return -1;

	int_cont = __raw_readl(tve.base + TVE_INT_CONT_REG);
	__raw_writel(int_cont & ~(CD_SM_INT | CD_LM_INT),
		     tve.base + TVE_INT_CONT_REG);

	cd_cont = __raw_readl(tve.base + TVE_CD_CONT_REG);
	__raw_writel(cd_cont | CD_TRIG_MODE, tve.base + TVE_CD_CONT_REG);

	__raw_writel(CD_SM_INT | CD_LM_INT | CD_MON_END_INT | CD_MAN_TRIG,
		     tve.base + TVE_STAT_REG);

	while ((__raw_readl(tve.base + TVE_STAT_REG) & CD_MON_END_INT) == 0)
		msleep(5);

	tve_update_detect_status();

	__raw_writel(cd_cont, tve.base + TVE_CD_CONT_REG);
	__raw_writel(int_cont, tve.base + TVE_INT_CONT_REG);

	return tve.detect;
}

static irqreturn_t tve_detect_handler(int irq, void *data)
{
	u32 stat;
	int old_detect = tve.detect;

	stat = __raw_readl(tve.base + TVE_STAT_REG);
	stat &= __raw_readl(tve.base + TVE_INT_CONT_REG);

	tve_update_detect_status();

	__raw_writel(stat | CD_MON_END_INT, tve.base + TVE_STAT_REG);

	if (old_detect != tve.detect)
		sysfs_notify(&tve.pdev->dev.kobj, NULL, "headphone");

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
		break;
	case FB_EVENT_MODE_CHANGE:
		if (tve_fbi != fbi)
			break;

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
		} else {
			tve_setup(TVOUT_FMT_OFF);
		}
		break;
	case FB_EVENT_BLANK:
		if ((tve_fbi != fbi) || (tve.cur_mode == TVOUT_FMT_OFF))
			return 0;

		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			tve_enable();
		else
			tve_disable();
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = tve_fb_event,
};

static ssize_t show_headphone(struct device_driver *dev, char *buf)
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
	else
		strcpy(buf, "headset\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, 0644, show_headphone, NULL);

static int tve_probe(struct platform_device *pdev)
{
	int ret, i;
	struct resource *res;
	struct tve_platform_data *plat_data = pdev->dev.platform_data;

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

	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	if (ret < 0)
		goto err1;

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG - DI1") == 0) {
			tve_fbi = registered_fb[i];
			break;
		}
	}
	if (tve_fbi != NULL) {
		fb_add_videomode(&video_modes[0], &tve_fbi->modelist);
		fb_add_videomode(&video_modes[1], &tve_fbi->modelist);
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

	/* Setup cable detect */
	__raw_writel(0x010777F1, tve.base + TVE_CD_CONT_REG);
	/* tve_man_detect(); not working */

	__raw_writel(CD_SM_INT | CD_LM_INT, tve.base + TVE_STAT_REG);
	__raw_writel(CD_SM_INT | CD_LM_INT, tve.base + TVE_INT_CONT_REG);

	__raw_writel(0x00000000, tve.base + 0x34);
	__raw_writel(0x00000000, tve.base + 0x38);
	__raw_writel(0x00000000, tve.base + 0x3C);
	__raw_writel(0x00000000, tve.base + 0x40);
	__raw_writel(0x00000000, tve.base + 0x44);
	__raw_writel(0x00000000, tve.base + TVE_MV_CONT_REG);

	clk_disable(tve.clk);

	ret = fb_register_client(&nb);
	if (ret < 0)
		goto err2;

	return 0;
err2:
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
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
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
	fb_unregister_client(&nb);
	return 0;
}

/*!
 * PM suspend/resume routing
 */
static int tve_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (enabled) {
		__raw_writel(0, tve.base + TVE_INT_CONT_REG);
		__raw_writel(0, tve.base + TVE_CD_CONT_REG);
		__raw_writel(0, tve.base + TVE_COM_CONF_REG);
		clk_disable(tve.clk);
	}
	return 0;
}

static int tve_resume(struct platform_device *pdev)
{
	if (enabled)
		clk_enable(tve.clk);

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
