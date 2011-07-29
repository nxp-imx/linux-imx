/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * based on arch/arm/plat-mxc/pwm.c
 */

#ifndef __MC34708_PWM_H
#define __MC34708_PWM_H

struct pwm_device;

/*
 * pwm_request - request a PWM device
 */
struct pwm_device *mc34708_pwm_request(int pwm_id, const char *label);

/*
 * pwm_free - free a PWM device
 */
void mc34708_pwm_free(struct pwm_device *pwm);

/*
 * pwm_config - change a PWM device configuration
 */
int mc34708_pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns);

/*
 * pwm_enable - start a PWM output toggling
 */
int mc34708_pwm_enable(struct pwm_device *pwm);

/*
 * pwm_disable - stop a PWM output toggling
 */
void mc34708_pwm_disable(struct pwm_device *pwm);

#endif /* __MC34708_PWM_H */
