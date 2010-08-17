/*
 * Freescale MXS Samsung LMS430 LCD panel initialization
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 * Copyright 2009 Embedded Alley Solutions, Inc All Rights Reserved.
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

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include <mach/device.h>
#include <mach/lcdif.h>
#include <mach/regs-pwm.h>
#include <mach/system.h>

#define REGS_PWM_BASE IO_ADDRESS(PWM_PHYS_ADDR)

#define DOTCLK_H_ACTIVE  480
#define DOTCLK_H_PULSE_WIDTH 1
#define DOTCLK_HF_PORCH  8
#define DOTCLK_HB_PORCH  15
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + (3 * DOTCLK_HB_PORCH))
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_ACTIVE  272
#define DOTCLK_V_PULSE_WIDTH  1
#define DOTCLK_VF_PORCH  4
#define DOTCLK_VB_PORCH  12
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct mxs_platform_bl_data bl_data;
static struct clk *lcd_clk;

static int init_panel(struct device *dev, dma_addr_t phys, int memsize,
		      struct mxs_platform_fb_entry *pentry)
{
	int ret = 0;
	lcd_clk = clk_get(NULL, "lcdif");
	if (IS_ERR(lcd_clk)) {
		ret = PTR_ERR(lcd_clk);
		goto out;
	}
	ret = clk_enable(lcd_clk);
	if (ret) {
		clk_put(lcd_clk);
		goto out;
	}

	ret = clk_set_rate(lcd_clk, 1000000000 / pentry->cycle_time_ns);	/* Hz */
	if (ret) {
		clk_disable(lcd_clk);
		clk_put(lcd_clk);
		goto out;
	}

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */
	mdelay(100);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	mdelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	mdelay(10);
	__raw_writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	mdelay(1);

	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			   DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			   DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			   DOTCLK_H_WAIT_CNT, DOTCLK_H_ACTIVE, 0);

	ret = mxs_lcdif_dma_init(dev, phys, memsize);
	if (ret)
		goto out;

	mxs_lcd_set_bl_pdata(pentry->bl_data);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_INIT, pentry);
	return 0;

out:
	return ret;
}

static void release_panel(struct device *dev,
			  struct mxs_platform_fb_entry *pentry)
{
	/* Reset LCD panel signel. */
	__raw_writel(BM_LCDIF_CTRL1_RESET,
		REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	mdelay(100);
	mxs_lcdif_notify_clients(MXS_LCDIF_PANEL_RELEASE, pentry);
	release_dotclk_panel();
	mxs_lcdif_dma_release();
	clk_disable(lcd_clk);
	clk_put(lcd_clk);
	__raw_writel(BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
}

static int blank_panel(int blank)
{
	int ret = 0, count;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
		for (count = 10000; count; count--) {
			if (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) &
			    BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}
		break;

	case FB_BLANK_UNBLANK:
		__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct mxs_platform_fb_entry fb_entry = {
	.name = "lms430",
	.x_res = 272,
	.y_res = 480,
	.bpp = 32,
	.cycle_time_ns = 150,
	.lcd_type = MXS_LCD_PANEL_DOTCLK,
	.init_panel = init_panel,
	.release_panel = release_panel,
	.blank_panel = blank_panel,
	.run_panel = mxs_lcdif_run,
	.stop_panel = mxs_lcdif_stop,
	.pan_display = mxs_lcdif_pan_display,
	.bl_data = &bl_data,
};

static struct clk *pwm_clk;

static int init_bl(struct mxs_platform_bl_data *data)
{
	int ret = 0;

	pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(pwm_clk)) {
		ret = PTR_ERR(pwm_clk);
		return ret;
	}
	clk_enable(pwm_clk);
	mxs_reset_block(REGS_PWM_BASE, 1);

	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);

	return 0;
}

static void free_bl(struct mxs_platform_bl_data *data)
{
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(599),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	__raw_writel(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

	clk_disable(pwm_clk);
	clk_put(pwm_clk);
}

static int values[] = { 0, 4, 9, 14, 20, 27, 35, 45, 57, 75, 100 };

static int power[] = {
	0, 1500, 3600, 6100, 10300,
	15500, 74200, 114200, 155200,
	190100, 191000
};

static int bl_to_power(int br)
{
	int base;
	int rem;

	if (br > 100)
		br = 100;
	base = power[br / 10];
	rem = br % 10;
	if (!rem)
		return base;
	else
		return base + (rem * (power[br / 10 + 1]) - base) / 10;
}

static int set_bl_intensity(struct mxs_platform_bl_data *data,
			    struct backlight_device *bd, int suspended)
{
	int intensity = bd->props.brightness;
	int scaled_int;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (suspended)
		intensity = 0;

	/*
	 * This is not too cool but what can we do?
	 * Luminance changes non-linearly...
	 */
	if (regulator_set_current_limit
	    (data->regulator, bl_to_power(intensity), bl_to_power(intensity)))
		return -EBUSY;

	scaled_int = values[intensity / 10];
	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity / 10);	/* r = i % 10; */
		scaled_int += rem * (values[intensity / 10 + 1] -
				     values[intensity / 10]) / 10;
	}
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	return 0;
}

static struct mxs_platform_bl_data bl_data = {
	.bl_max_intensity = 100,
	.bl_default_intensity = 50,
	.bl_cons_intensity = 50,
	.init_bl = init_bl,
	.free_bl = free_bl,
	.set_bl_intensity = set_bl_intensity,
};

static int __init register_devices(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return -ENODEV;

	mxs_lcd_register_entry(&fb_entry, pdev->dev.platform_data);

	return 0;
}

subsys_initcall(register_devices);
