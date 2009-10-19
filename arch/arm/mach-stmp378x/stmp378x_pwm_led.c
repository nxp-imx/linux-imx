/*
 * Freescale STMP378X PWM LEDs pin multiplexing
 *
 * Author: Drew Bendetti <drewb@embeddedalley.com>
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
#include <linux/io.h>
#include <linux/module.h>
#include <mach/pinmux.h>

#define PWM_PINID(n)    STMP3XXX_PINID(1, 26 + n)

int pwm_led_pinmux_request(int pwmn, char *title)
{
	int rc = 0;

	/* PIN_FUN1 is PWM for these pins */
	rc = stmp3xxx_request_pin(PWM_PINID(pwmn), PIN_FUN1, title);
	if (rc)
		return rc;

	stmp3xxx_pin_voltage(PWM_PINID(pwmn), PIN_3_3V, title);
	/* pwm0-3 support 4,8,12mA; pwm4 supports 8,16,24mA
	 * I'm forcing 8 here since it's the only one in common
	 */
	stmp3xxx_pin_strength(PWM_PINID(pwmn), PIN_8MA, title);

	return 0;
}
EXPORT_SYMBOL_GPL(pwm_led_pinmux_request);

void pwm_led_pinmux_free(int pwmn, char *title)
{
	stmp3xxx_pin_voltage(PWM_PINID(pwmn), PIN_4MA, title);
	stmp3xxx_pin_strength(PWM_PINID(pwmn), PIN_1_8V, title);

	stmp3xxx_release_pin(PWM_PINID(pwmn), title);
}
EXPORT_SYMBOL_GPL(pwm_led_pinmux_free);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Drew Benedetti <drewb@embeddedalley.com>");
