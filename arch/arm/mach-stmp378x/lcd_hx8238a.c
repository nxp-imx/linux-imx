/*
 * Freescale STMP37XX/STMP378X dotclk panel initialization
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

#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/lcdif.h>
#include <mach/cpu.h>
#include <mach/stmp3xxx.h>

#include "common.h"

#define MAX_CHAIN_LEN		10

#define DOTCLK_H_ACTIVE  960
#define DOTCLK_H_PULSE_WIDTH 2
#define DOTCLK_HF_PORCH  1
#define DOTCLK_HB_PORCH  67
#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + (3 * DOTCLK_HB_PORCH))
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)

#define DOTCLK_V_PULSE_WIDTH  2
#define DOTCLK_V_ACTIVE  240
#define DOTCLK_VF_PORCH  1
#define DOTCLK_VB_PORCH  16
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct stmp3xxx_platform_bl_data bl_data;
extern struct pin_group lcd_pins;
extern unsigned lcd_spi_pins[];

static void spi_write(u32 val)
{
	u32 mask;

	gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 0);
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
	gpio_set_value(lcd_spi_pins[SPI_MOSI], 0);
	gpio_set_value(lcd_spi_pins[SPI_SCLK], 0);
	gpio_set_value(lcd_spi_pins[SPI_CS], 1);
}

static void write_reg(u16 reg, u16 val)
{
	pr_debug("%s: writing %x to %x\n", __func__, reg, val);
	spi_write(0x00700000 | reg);
	spi_write(0x00720000 | val);
}

static void init_panel_hw(void)
{
	int i;
	const unsigned short seq[] = {
		0x02, 0x0200,
		0x03, 0x6164,
		0x0E, 0x3380,
		0x1E, 0x00D2,
		0x01, 0x733F,
		0x04, 0x0448,
		0x05, 0xBC54,
		0x0A, 0x4008,
		0x0B, 0xD400,
		0x0D, 0x3229,
		0x0F, 0x0000,
		0x30, 0x0000,
		0x31, 0x0407,
		0x32, 0x0202,
		0x33, 0x0000,
		0x34, 0x0505,
		0x35, 0x0003,
		0x36, 0x0707,
		0x37, 0x0000,
		0x3A, 0x0904,
		0x3B, 0x0904,
	};

	for (i = 0; i < sizeof(seq) / sizeof(seq[0]); i += 2)
		write_reg(seq[i], seq[i + 1]);
}

static int init_pinmux(void)
{
	return stmp3xxx_request_pin_group(&lcd_pins, "lcd_hx8238a");
}

static int init_pinmux_spi(void)
{
	int ret = -EINVAL;

	ret = gpio_request(lcd_spi_pins[SPI_MOSI], "lcd_hx8238a");
	if (ret)
		goto out_1;

	ret = gpio_request(lcd_spi_pins[SPI_SCLK], "lcd_hx8238a");
	if (ret)
		goto out_2;
	ret = gpio_request(lcd_spi_pins[SPI_CS], "lcd_hx8238a");
	if (ret)
		goto out_3;

	/* Enable these pins as outputs */
	gpio_direction_output(lcd_spi_pins[SPI_MOSI], 0);
	gpio_direction_output(lcd_spi_pins[SPI_SCLK], 0);
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
	stmp3xxx_release_pin_group(&lcd_pins, "lcd_hx8238a");
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

	ret = init_pinmux();
	if (ret)
		goto out_1;
	ret = init_pinmux_spi();
	if (ret)
		goto out_2;
	init_panel_hw();

	ret = stmp3xxx_lcdif_dma_init(dev, phys, memsize, 0);
	if (ret)
		goto out_3;

	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			DOTCLK_V_WAIT_CNT, DOTCLK_H_ACTIVE, 1);

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
	int ret = 0;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		stmp3xxx_clearl(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		break;

	case FB_BLANK_UNBLANK:
		stmp3xxx_setl(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct stmp3xxx_platform_fb_entry fb_entry = {
	.name		= "hx8238a",
	.x_res		= 240,
	.y_res		= 320,
	.bpp		= 32,
	.cycle_time_ns	= 150,
	.lcd_type	= STMP3XXX_LCD_PANEL_DOTCLK,
	.init_panel	= init_panel,
	.release_panel	= release_panel,
	.blank_panel	= blank_panel,
	.run_panel	= stmp3xxx_lcdif_run,
	.stop_panel	= stmp3xxx_lcdif_stop,
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

	ret = stmp3xxx_request_pin(PINID_PWM2, PIN_FUN1, "lcd_hx8238a");
	if (ret)
		goto out_mux;
	stmp3xxx_pin_voltage(PINID_PWM2, PIN_12MA, "lcd_hx8238a");
	stmp3xxx_pin_strength(PINID_PWM2, PIN_3_3V, "lcd_hx8238a");

	stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	stmp3xxx_setl(BM_PWM_CTRL_PWM2_ANA_CTRL_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	__raw_writel(BF(10, PWM_ACTIVEn_INACTIVE) |
				BF(5, PWM_ACTIVEn_ACTIVE),
				REGS_PWM_BASE + HW_PWM_ACTIVEn(2));
	__raw_writel(BF(1, PWM_PERIODn_CDIV) | /* divide by 2 */
			BF(2, PWM_PERIODn_INACTIVE_STATE) | /* low */
			BF(3, PWM_PERIODn_ACTIVE_STATE) | /* high */
			BF(14, PWM_PERIODn_PERIOD),
			REGS_PWM_BASE + HW_PWM_PERIODn(2));
	return 0;

out_mux:
	clk_put(pwm_clk);
out:
	return ret;
}

static void free_bl(struct stmp3xxx_platform_bl_data *data)
{
	stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	stmp3xxx_release_pin(PINID_PWM2, "lcd_hx8238a");
	clk_disable(pwm_clk);
	clk_put(pwm_clk);
}

static void set_bl_intensity(struct stmp3xxx_platform_bl_data *data,
			struct backlight_device *bd, int suspended)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (suspended)
		intensity = 0;

	stmp3xxx_clearl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	if (intensity) {
		HW_LRADC_CTRL2_CLR(BM_LRADC_CTRL2_BL_BRIGHTNESS);
		HW_LRADC_CTRL2_SET(BM_LRADC_CTRL2_BL_ENABLE |
				BM_LRADC_CTRL2_BL_MUX_SELECT |
				BF(intensity - 1, LRADC_CTRL2_BL_BRIGHTNESS));
		stmp3xxx_setl(BM_PWM_CTRL_PWM2_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL);
	}
}

static struct stmp3xxx_platform_bl_data bl_data = {
	.bl_max_intensity	= (BM_LRADC_CTRL2_BL_BRIGHTNESS >>
					BP_LRADC_CTRL2_BL_BRIGHTNESS) + 1,
	.bl_default_intensity	= 0x10,
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
