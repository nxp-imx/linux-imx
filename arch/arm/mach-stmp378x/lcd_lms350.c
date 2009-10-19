/*
 * Freescale STMP378X Samsung LMS350 LCD panel initialization
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <mach/pinmux.h>
#include <mach/lcdif.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/cputype.h>


#define DOTCLK_H_ACTIVE  320
#define DOTCLK_H_PULSE_WIDTH 3
#define DOTCLK_HF_PORCH  5
#define DOTCLK_HB_PORCH  4
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + (3 * DOTCLK_HB_PORCH))
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_PULSE_WIDTH  2
#define DOTCLK_V_ACTIVE  240
#define DOTCLK_VF_PORCH  2
#define DOTCLK_VB_PORCH  5
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct stmp3xxx_platform_bl_data bl_data;
extern struct pin_group lcd_pins;
extern unsigned lcd_spi_pins[];

static void spi_write(u32 val)
{
	u32 mask;

	gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	gpio_set_value(lcd_spi_pins[SPI_CS], 0);

	for (mask = 0x00800000; mask != 0; mask >>= 1) {
		gpio_set_value(lcd_spi_pins[SPI_SCLK], 0);
		if (val & mask)
			gpio_set_value(lcd_spi_pins[SPI_MOSI], 1);
		else
			gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);

		gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	}

	udelay(10);
	gpio_set_value(lcd_spi_pins[SPI_MOSI], 1);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 1);
	gpio_set_value(lcd_spi_pins[SPI_CS], 1);
}

static void write_reg(u16 reg, u16 val)
{
	pr_debug("%s: writing %x to %x\n", __func__, reg, val);
	spi_write(0x00740000 | reg);
	spi_write(0x00760000 | val);
}

static const unsigned short pon_seq[] = {
	/* power on */
	0x07, 0x0000,  20,
	0x12, 0x1618,   0,
	0x11, 0x222f,   0,
	0x13, 0x40ca,   0,
	0x10, 0x3108, 300,
	0x12, 0x1658, 250,
	0x01, 0x2b1d,   0,
	0x02, 0x0300,   0,
	0x03, 0xD040,   0,
	0x08, (DOTCLK_VB_PORCH + DOTCLK_V_PULSE_WIDTH) - 2,   0,
	0x09, ((DOTCLK_H_PULSE_WIDTH / 3) + DOTCLK_HB_PORCH) - 2,   0,
	0x76, 0x2213,   0,
	0x0b, 0x33e1,   0,
	0x0c, 0x0020,   0,
	0x76, 0x0000,   0,
	0x0d, 0x0000,   0,
	0x0e, 0x0000,   0,
	0x14, 0x0000,   0,
	0x15, 0x0803,   0,
	0x16, 0x0000,   0,
	0x30, 0x0209,   0,
	0x31, 0x0404,   0,
	0x32, 0x0e07,   0,
	0x33, 0x0602,   0,
	0x34, 0x0707,   0,
	0x35, 0x0707,   0,
	0x36, 0x0707,   0,
	0x37, 0x0206,   0,
	0x38, 0x0f06,   0,
	0x39, 0x0611,  20,
};

static const unsigned short don_seq[] = {
	/* display on */
	0x07, 0x0001, 150,
	0x07, 0x0101, 150,
	0x76, 0x2213,   0,
	0x1c, 0x6650,   0,
	0x0b, 0x33e0,   0,
	0x76, 0x0000,   0,
	0x07, 0x0103,   0,
};


static const unsigned short doff_seq[] = {
	/* display off */
	0x0b, 0x33e1,   0,
	0x07, 0x0102, 150,
	0x07, 0x0100, 150,
	0x12, 0x0000,   0,
	0x10, 0x0000,   0,
};

static const unsigned short poff_seq[] = {
	/* power off */
	/* called after display off */
	0x07, 0x0000,    0,
	0x10, 0x0000,    0,
	0x11, 0x0000,    0,
};

static const unsigned short sby_seq[] = {
	/* standby */
	/* called after display off */
	0x10, 0x0001,     0
};

static const unsigned short csby_seq[] = {
	/* cancel standby */
	/* called after display on */
	0x10, 0x0000,     0
};

static void display_off(void)
{
	int i;
	const unsigned short *seq;

	seq = doff_seq;
	for (i = 0; i < ARRAY_SIZE(doff_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
}

static void display_on(void)
{
	int i;
	const unsigned short *seq;

	seq = don_seq;
	for (i = 0; i < ARRAY_SIZE(don_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
}

static void init_panel_hw(void)
{
	int i;
	const unsigned short *seq;

	seq = pon_seq;
	for (i = 0; i < ARRAY_SIZE(pon_seq); i += 3) {
		write_reg(seq[i], seq[i + 1]);
		if (seq[i + 2])
			udelay(seq[i + 2]);
	}
	display_on();
}

static int init_pinmux(void)
{
	return stmp3xxx_request_pin_group(&lcd_pins, "lcd_lms350");
}

static int init_pinmux_spi(void)
{
	int ret = -EINVAL;

	ret = gpio_request(lcd_spi_pins[SPI_MOSI], "lcd_lms350");
	if (ret)
		goto out_1;

	ret = gpio_request(lcd_spi_pins[SPI_SCLK], "lcd_lms350");
	if (ret)
		goto out_2;
	ret = gpio_request(lcd_spi_pins[SPI_CS], "lcd_lms350");
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
	stmp3xxx_release_pin_group(&lcd_pins, "lcd_lms350");
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
	ret = clk_set_rate(lcd_clk,
			1000000/pentry->cycle_time_ns); /* kHz */
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
	stmp3xxx_clearl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1); /* low */
	mdelay(100);
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1); /* high */
	mdelay(10);
	stmp3xxx_clearl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1); /* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	mdelay(10);
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1); /* high */
	mdelay(1);

	ret = init_pinmux();
	if (ret)
		goto out_1;
	ret = init_pinmux_spi();
	if (ret)
		goto out_2;
	init_panel_hw();

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
	display_off();
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
		stmp3xxx_clearl(BM_LCDIF_CTRL_BYPASS_COUNT, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		for (count = 10000; count; count--) {
			if (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) & BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}
		break;

	case FB_BLANK_UNBLANK:
		stmp3xxx_setl(BM_LCDIF_CTRL_BYPASS_COUNT, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static void stop_panel(void)
{
	stmp3xxx_lcdif_stop();
	display_off();
}

static void run_panel(void)
{
	display_on();
	stmp3xxx_lcdif_run();
}

static struct stmp3xxx_platform_fb_entry fb_entry = {
	.name		= "lms350",
	.x_res		= 240,
	.y_res		= 320,
	.bpp		= 32,
	.cycle_time_ns	= 200,
	.lcd_type	= STMP3XXX_LCD_PANEL_DOTCLK,
	.init_panel	= init_panel,
	.release_panel	= release_panel,
	.blank_panel	= blank_panel,
	.run_panel	= run_panel,
	.stop_panel	= stop_panel,
	.pan_display	= stmp3xxx_lcdif_pan_display,
	.bl_data	= &bl_data,
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

	ret = stmp3xxx_request_pin(PINID_PWM2, PIN_FUN1, "lcd_lms350");
	if (ret)
		goto out_mux;

	stmp3xxx_pin_voltage(PINID_PWM2, PIN_8MA, "lcd_lms350");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_lms350");

	__raw_writel(BF(0, PWM_ACTIVEn_INACTIVE) |
				BF(0, PWM_ACTIVEn_ACTIVE),
				REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
			BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
			BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
			BF(599, PWM_PERIODn_PERIOD), REGS_PWM_BASE + HW_PWM_PERIODn(2));
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
	__raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
		BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
		BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
		BF(599, PWM_PERIODn_PERIOD),
		REGS_PWM_BASE + HW_PWM_PERIODn(2));
	stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	stmp3xxx_pin_voltage(PINID_PWM2, PIN_4MA, "lcd_lms350");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_1_8V, "lcd_lms350");

	stmp3xxx_release_pin(PINID_PWM2, "lcd_lms350");
	clk_disable(pwm_clk);
	clk_put(pwm_clk);
}

static int values[] = { 6, 9, 12, 15, 19, 24, 30, 40, 55, 75, 100 };
static int power[] = {
	0, 1500, 3600, 6100, 10300,
	15500, 74200, 114200, 155200,
	190100, 191000
};

static int  bl_to_power(int br)
{
	int base;
	int rem;

	if (br > 100)
		br = 100;
	base = power[br/10];
	rem = br % 10;
	if (!rem)
		return base;
	else
		return base + (rem * (power[br/10 + 1]) - base) / 10;
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
	if (regulator_set_current_limit(data->regulator, bl_to_power(intensity),
		bl_to_power(intensity)))
		return -EBUSY;

	scaled_int = values[intensity/10];
	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity/10); /* r = i % 10;*/
		scaled_int += rem*(values[intensity/10 + 1] -
			      values[intensity/10])/10;
	}
	__raw_writel(BF(scaled_int, PWM_ACTIVEn_INACTIVE) |
		BF(0, PWM_ACTIVEn_ACTIVE), REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(6, PWM_PERIODn_CDIV) | /* divide by 64 */
		BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
		BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
		BF(599, PWM_PERIODn_PERIOD),
		REGS_PWM_BASE + HW_PWM_PERIODn(2));
	return 0;
}

static struct stmp3xxx_platform_bl_data bl_data = {
	.bl_max_intensity	= 100,
	.bl_default_intensity	= 50,
	.bl_cons_intensity      = 50,
	.init_bl		= init_bl,
	.free_bl		= free_bl,
	.set_bl_intensity	= set_bl_intensity,
};

static int __init register_devices(void)
{
	stmp3xxx_lcd_register_entry(&fb_entry,
				    stmp3xxx_framebuffer.dev.platform_data);
	return 0;
}
subsys_initcall(register_devices);
