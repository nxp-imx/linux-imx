/*
 * Freescale STMP37XX/STMP378X MMC pin multiplexing
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/mmc.h>
#include <mach/regs-ssp.h>

#if defined(CONFIG_MACH_STMP378X)
#define MMC_POWER	PINID_PWM3
#define MMC_WP		PINID_PWM4
#elif defined(CONFIG_MACH_STMP37XX)
#define MMC_POWER	PINID_PWM3
#define MMC_WP		PINID_PWM4
#else
#define MMC_POWER	PINID_NO_PIN
#define MMC_WP		PINID_NO_PIN
#endif

static int mmc_drive_power;
static int mmc_wp_supported;

static struct pin_desc mmc_pins_desc[] = {
	{ PINID_SSP1_DATA0, PIN_FUN1, PIN_8MA, PIN_3_3V, 1 },
	{ PINID_SSP1_DATA1, PIN_FUN1, PIN_8MA, PIN_3_3V, 1 },
	{ PINID_SSP1_DATA2, PIN_FUN1, PIN_8MA, PIN_3_3V, 1 },
	{ PINID_SSP1_DATA3, PIN_FUN1, PIN_8MA, PIN_3_3V, 1 },
	{ PINID_SSP1_CMD, PIN_FUN1, PIN_8MA, PIN_3_3V, 1 },
	{ PINID_SSP1_SCK, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_SSP1_DETECT, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
};

static struct pin_group mmc_pins = {
	.pins		= mmc_pins_desc,
	.nr_pins	= ARRAY_SIZE(mmc_pins_desc),
};

int stmp3xxxmmc_get_wp(void)
{
	if (mmc_wp_supported)
		return gpio_get_value(MMC_WP);

	return 0;
}

int stmp3xxxmmc_hw_init_ssp1(void)
{
	int ret;

	mmc_drive_power = stmp3xxx_valid_pin(MMC_POWER);
	mmc_wp_supported = stmp3xxx_valid_pin(MMC_WP);

	ret = stmp3xxx_request_pin_group(&mmc_pins, "mmc");
	if (ret)
		goto out;

	if (mmc_wp_supported) {
		/* Configure write protect GPIO pin */
		ret = gpio_request(MMC_WP, "mmc wp");
		if (ret)
			goto out_wp;

		gpio_set_value(MMC_WP, 0);
		gpio_direction_input(MMC_WP);
	}

	if (mmc_drive_power) {
		/* Configure POWER pin as gpio to drive power to MMC slot */
		ret = gpio_request(MMC_POWER, "mmc power");
		if (ret)
			goto out_power;

		gpio_direction_output(MMC_POWER, 0);
		mdelay(100);
	}

	return 0;

out_power:
	if (mmc_wp_supported)
		gpio_free(MMC_WP);
out_wp:
	stmp3xxx_release_pin_group(&mmc_pins, "mmc");
out:
	return ret;
}

void stmp3xxxmmc_hw_release_ssp1(void)
{
	if (mmc_drive_power)
		gpio_free(MMC_POWER);

	if (mmc_wp_supported)
		gpio_free(MMC_WP);

	stmp3xxx_release_pin_group(&mmc_pins, "mmc");
}

void stmp3xxxmmc_cmd_pullup_ssp1(int enable)
{
	stmp3xxx_pin_pullup(PINID_SSP1_CMD, enable, "mmc");
}

unsigned long stmp3xxxmmc_setclock_ssp1(unsigned long hz)
{
	struct clk *ssp = clk_get(NULL, "ssp"), *parent;
	char *p;
	long r;

	/* using SSP1, no timeout, clock rate 1 */
	__raw_writel(BF(2, SSP_TIMING_CLOCK_DIVIDE) |
			BF(0xFFFF, SSP_TIMING_TIMEOUT),
			REGS_SSP1_BASE + HW_SSP_TIMING);

	if (hz > 1000000)
		p = "io";
	else
		p = "osc_24M";

	parent = clk_get(NULL, p);
	clk_set_parent(ssp, parent);
	r = clk_set_rate(ssp, 2 * hz / 1000);
	clk_put(parent);
	clk_put(ssp);

	return hz;
}
