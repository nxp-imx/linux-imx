/*
 * Freescale STMP37XX/STMP378X PWM LED arch-dependent structure
 * and functions declarations
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
#ifndef __ASM_PLAT_PWM_LED_H
#define __ASM_PLAT_PWM_LED_H

extern int pwm_led_pinmux_request(int, char *);
extern void pwm_led_pinmux_free(int, char *);

#endif /* __ASM_PLAT_PWM_LED_H */
