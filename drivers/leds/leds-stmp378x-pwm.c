/*
 * Freescale STMP378X PWM LED driver
 *
 * Author: Drew Benedetti <drewb@embeddedalley.com>
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
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <mach/regs-pwm.h>
#include <mach/regs-clkctrl.h>
#include <mach/pwm-led.h>
#include <mach/stmp3xxx.h>

/* Up to 5 PWM lines are available. */
#define PWM_MAX 5

/* PWM enables are the lowest PWM_MAX bits of HW_PWM_CTRL register */
#define BM_PWM_CTRL_PWM_ENABLE(n)	((1<<(n)) & ((1<<(PWM_MAX))-1))
#define BF_PWM_PERIODn_SETTINGS					\
		(BF_PWM_PERIODn_CDIV(5) | /* divide by 64 */ 	\
		BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */ 	\
		BF_PWM_PERIODn_ACTIVE_STATE(3) | /* high */ 	\
		BF_PWM_PERIODn_PERIOD(LED_FULL)) /* 255 cycles */

struct stmp378x_led {
	struct led_classdev led_dev;
	int in_use;
};

static struct stmp378x_led leds[PWM_MAX];

static struct clk *pwm_clk;

static void stmp378x_pwm_led_brightness_set(struct led_classdev *pled,
					    enum led_brightness value)
{
	unsigned int pwmn;

	pwmn = container_of(pled, struct stmp378x_led, led_dev) - leds;

	if (pwmn < PWM_MAX && leds[pwmn].in_use) {
		HW_PWM_CTRL_CLR(BM_PWM_CTRL_PWM_ENABLE(pwmn));
		HW_PWM_ACTIVEn_WR(pwmn, BF_PWM_ACTIVEn_INACTIVE(value) |
				BF_PWM_ACTIVEn_ACTIVE(0));
		HW_PWM_PERIODn_WR(pwmn, BF_PWM_PERIODn_SETTINGS);
		HW_PWM_CTRL_SET(BM_PWM_CTRL_PWM_ENABLE(pwmn));
	}
}

static int stmp378x_pwm_led_probe(struct platform_device *pdev)
{
	struct led_classdev *led;
	unsigned int pwmn;
	int leds_in_use = 0, rc = 0;
	int i;

	stmp3xxx_reset_block(REGS_PWM_BASE, 1);

	pwm_clk = clk_get(&pdev->dev, "pwm");
	if (IS_ERR(pwm_clk)) {
		rc = PTR_ERR(pwm_clk);
		return rc;
	}

	clk_enable(pwm_clk);

	for (i = 0; i < pdev->num_resources; i++) {

		if (pdev->resource[i].flags & IORESOURCE_DISABLED)
			continue;

		pwmn = pdev->resource[i].start;
		if (pwmn >= PWM_MAX) {
			dev_err(&pdev->dev, "PWM %d doesn't exist\n", pwmn);
			continue;
		}

		rc = pwm_led_pinmux_request(pwmn, "stmp378x_pwm_led");
		if (rc) {
			dev_err(&pdev->dev,
				"PWM %d is not available (err=%d)\n",
				pwmn, rc);
			continue;
		}

		led = &leds[pwmn].led_dev;

		led->flags = pdev->resource[i].flags;
		led->name = pdev->resource[i].name;
		led->brightness = LED_HALF;
		led->flags = 0;
		led->brightness_set = stmp378x_pwm_led_brightness_set;
		led->default_trigger = 0;

		rc = led_classdev_register(&pdev->dev, led);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"Unable to register LED device %d (err=%d)\n",
				pwmn, rc);
			pwm_led_pinmux_free(pwmn, "stmp378x_pwm_led");
			continue;
		}

		/* PWM LED is available now */
		leds[pwmn].in_use = !0;
		leds_in_use++;

		/* Set default brightness */
		stmp378x_pwm_led_brightness_set(led, LED_HALF);
	}

	if (leds_in_use == 0) {
		dev_info(&pdev->dev, "No PWM LEDs available\n");
		clk_disable(pwm_clk);
		clk_put(pwm_clk);
		return -ENODEV;
	}

	return 0;
}

static int stmp378x_pwm_led_remove(struct platform_device *pdev)
{
	unsigned int pwmn;

	for (pwmn = 0; pwmn < PWM_MAX; pwmn++) {

		if (!leds[pwmn].in_use)
			continue;

		/* Disable LED */
		HW_PWM_CTRL_CLR(BM_PWM_CTRL_PWM_ENABLE(pwmn));
		HW_PWM_ACTIVEn_WR(pwmn, BF_PWM_ACTIVEn_INACTIVE(0) |
				BF_PWM_ACTIVEn_ACTIVE(0));
		HW_PWM_PERIODn_WR(pwmn, BF_PWM_PERIODn_SETTINGS);

		led_classdev_unregister(&leds[pwmn].led_dev);
		pwm_led_pinmux_free(pwmn, "stmp378x_pwm_led");

		leds[pwmn].led_dev.name = 0;
		leds[pwmn].in_use = 0;
	}

	clk_disable(pwm_clk);
	clk_put(pwm_clk);

	return 0;
}


static struct platform_driver stmp378x_pwm_led_driver = {
	.probe   = stmp378x_pwm_led_probe,
	.remove  = stmp378x_pwm_led_remove,
	.driver  = {
		.name = "stmp378x-pwm-led",
	},
};

static int __init stmp378x_pwm_led_init(void)
{
	return platform_driver_register(&stmp378x_pwm_led_driver);
}

static void __exit stmp378x_pwm_led_exit(void)
{
	platform_driver_unregister(&stmp378x_pwm_led_driver);
}

module_init(stmp378x_pwm_led_init);
module_exit(stmp378x_pwm_led_exit);

MODULE_AUTHOR("Drew Benedetti <drewb@embeddedalley.com>");
MODULE_DESCRIPTION("STMP378X PWM LED driver");
MODULE_LICENSE("GPL");
