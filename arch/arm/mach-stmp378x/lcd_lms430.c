/*
 * Freescale STMP378X Samsung LMS430 LCD panel initialization
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>

#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/lcdif.h>
#include <mach/pinmux.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>

#define DOTCLK_H_ACTIVE  480
#define DOTCLK_H_PULSE_WIDTH 1
#define DOTCLK_HF_PORCH  8
#define DOTCLK_HB_PORCH  15
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + (3 * DOTCLK_HB_PORCH))
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_PULSE_WIDTH  1
#define DOTCLK_V_ACTIVE  272
#define DOTCLK_VF_PORCH  4
#define DOTCLK_VB_PORCH  12
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct stmp3xxx_platform_bl_data bl_data;
extern struct pin_group lcd_pins;
extern unsigned lcd_spi_pins[];

static int init_pinmux(void)
{
	return stmp3xxx_request_pin_group(&lcd_pins, "lcd_lms430");
}

static int init_pinmux_spi(void)
{
	int ret = -EINVAL;

	ret = gpio_request(lcd_spi_pins[SPI_MOSI], "lcd_lms430");
	if (ret)
		goto out_1;

	ret = gpio_request(lcd_spi_pins[SPI_SCLK], "lcd_lms430");
	if (ret)
		goto out_2;
	ret = gpio_request(lcd_spi_pins[SPI_CS], "lcd_lms430");
	if (ret)
		goto out_3;

	/* Enable these pins as outputs */
	gpio_direction_output(lcd_spi_pins[SPI_MOSI], 1);
	gpio_direction_output(lcd_spi_pins[SPI_SCLK], 1);
	gpio_direction_output(lcd_spi_pins[SPI_CS], 1);

	return 0;
out_3:
	gpio_free(lcd_spi_pins[SPI_SCLK]);
out_2:
	gpio_free(lcd_spi_pins[SPI_MOSI]);
out_1:
	return ret;
}

static void uninit_pinmux(void)
{
	stmp3xxx_release_pin_group(&lcd_pins, "lcd_lms430");
}

static void uninit_pinmux_spi(void)
{
	gpio_free(lcd_spi_pins[SPI_MOSI]);
	gpio_free(lcd_spi_pins[SPI_SCLK]);
	gpio_free(lcd_spi_pins[SPI_CS]);
}

static struct clk *lcd_clk;

static int init_panel(struct device *dev, dma_addr_t phys, int memsize,
		      struct stmp3xxx_platform_fb_entry *pentry)
{
	int ret = 0;

	lcd_clk = clk_get(dev, "lcdif");
	if (IS_ERR(lcd_clk)) {
		ret = PTR_ERR(lcd_clk);
		goto out_1;
	}
	ret = clk_enable(lcd_clk);
	if (ret) {
		clk_put(lcd_clk);
		goto out_1;
	}
	ret = clk_set_rate(lcd_clk, 1000000 / pentry->cycle_time_ns);	/* kHz */
	if (ret) {
		clk_disable(lcd_clk);
		clk_put(lcd_clk);
		goto out_1;
	}

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	stmp3xxx_clearl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);	/* low */
	mdelay(100);
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);	/* high */
	mdelay(10);
	stmp3xxx_clearl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);	/* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	mdelay(10);
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);	/* high */
	mdelay(1);

	ret = init_pinmux();
	if (ret)
		goto out_1;
	ret = init_pinmux_spi();
	if (ret)
		goto out_2;

	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			   DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			   DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			   DOTCLK_H_WAIT_CNT, DOTCLK_H_ACTIVE, 0);

	ret = stmp3xxx_lcdif_dma_init(dev, phys, memsize, 1);
	if (ret)
		goto out_3;

	stmp3xxx_lcd_set_bl_pdata(pentry->bl_data);
	stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_INIT, pentry);
	return 0;
out_3:
	uninit_pinmux_spi();
out_2:
	uninit_pinmux();
out_1:
	return ret;
}

static void release_panel(struct device *dev,
			  struct stmp3xxx_platform_fb_entry *pentry)
{
	stmp3xxx_lcdif_notify_clients(STMP3XXX_LCDIF_PANEL_RELEASE, pentry);
	uninit_pinmux_spi();
	uninit_pinmux();
	release_dotclk_panel();
	stmp3xxx_lcdif_dma_release();
	clk_disable(lcd_clk);
	clk_put(lcd_clk);
}

static int blank_panel(int blank)
{
	int ret = 0, count;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		stmp3xxx_clearl(BM_LCDIF_CTRL_BYPASS_COUNT,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		for (count = 10000; count; count--) {
			if (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) &
			    BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}
		break;

	case FB_BLANK_UNBLANK:
		stmp3xxx_setl(BM_LCDIF_CTRL_BYPASS_COUNT,
			      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct stmp3xxx_platform_fb_entry fb_entry = {
	.name = "lms430",
	.x_res = 272,
	.y_res = 480,
	.bpp = 32,
	.cycle_time_ns = 150,
	.lcd_type = STMP3XXX_LCD_PANEL_DOTCLK,
	.init_panel = init_panel,
	.release_panel = release_panel,
	.blank_panel = blank_panel,
	.run_panel = stmp3xxx_lcdif_run,
	.stop_panel = stmp3xxx_lcdif_stop,
	.pan_display = stmp3xxx_lcdif_pan_display,
	.bl_data = &bl_data,
};

static struct clk *pwm_clk;

static int init_bl(struct stmp3xxx_platform_bl_data *data)
{
	int ret = 0;

	pwm_clk = clk_get(NULL, "pwm");
	if (IS_ERR(pwm_clk)) {
		ret = PTR_ERR(pwm_clk);
		goto out;
	}
	clk_enable(pwm_clk);
	stmp3xxx_reset_block(REGS_PWM_BASE, 1);

	ret = stmp3xxx_request_pin(PINID_PWM2, PIN_FUN1, "lcd_lms430");
	if (ret)
		goto out_mux;

	stmp3xxx_pin_voltage(PINID_PWM2, PIN_8MA, "lcd_lms430");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_lms430");

	__raw_writel(BF(0, PWM_ACTIVEn_INACTIVE) |
		     BF(0, PWM_ACTIVEn_ACTIVE), REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(6, PWM_PERIODn_CDIV) |	/* divide by 64 */
		     BF(2, PWM_PERIODn_INACTIVE_STATE) |	/* low */
		     BF(3, PWM_PERIODn_ACTIVE_STATE) |	/* high */
		     BF(599, PWM_PERIODn_PERIOD),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	stmp3xxx_setl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);

	return 0;

out_mux:
	clk_put(pwm_clk);
out:
	return ret;
}

static void free_bl(struct stmp3xxx_platform_bl_data *data)
{
	__raw_writel(BF(0, PWM_ACTIVEn_INACTIVE) |
		     BF(0, PWM_ACTIVEn_ACTIVE),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(6, PWM_PERIODn_CDIV) |	/* divide by 64 */
		     BF(2, PWM_PERIODn_INACTIVE_STATE) |	/* low */
		     BF(3, PWM_PERIODn_ACTIVE_STATE) |	/* high */
		     BF(599, PWM_PERIODn_PERIOD),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	stmp3xxx_pin_voltage(PINID_PWM2, PIN_4MA, "lcd_lms430");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_1_8V, "lcd_lms430");

	stmp3xxx_release_pin(PINID_PWM2, "lcd_lms430");
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

static int set_bl_intensity(struct stmp3xxx_platform_bl_data *data,
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
	__raw_writel(BF(scaled_int, PWM_ACTIVEn_INACTIVE) |
		     BF(0, PWM_ACTIVEn_ACTIVE),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(6, PWM_PERIODn_CDIV) |	/* divide by 64 */
		     BF(2, PWM_PERIODn_INACTIVE_STATE) |	/* low */
		     BF(3, PWM_PERIODn_ACTIVE_STATE) |	/* high */
		     BF(399, PWM_PERIODn_PERIOD),
		     REGS_PWM_BASE + HW_PWM_PERIODn(2));
	return 0;
}

static struct stmp3xxx_platform_bl_data bl_data = {
	.bl_max_intensity = 100,
	.bl_default_intensity = 50,
	.bl_cons_intensity = 50,
	.init_bl = init_bl,
	.free_bl = free_bl,
	.set_bl_intensity = set_bl_intensity,
};

static int __init register_devices(void)
{
	stmp3xxx_lcd_register_entry(&fb_entry,
				    stmp3xxx_framebuffer.dev.platform_data);
	return 0;
}

subsys_initcall(register_devices);
