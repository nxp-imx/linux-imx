/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <mach/pinctrl.h>

#include "mx28_pins.h"

static struct pin_desc mx28evk_fixed_pins[] = {
	{
	 .name = "DUART.RX",
	 .id = PINID_PWM0,
	 .fun = PIN_FUN3,
	 },
	{
	 .name = "DUART.TX",
	 .id = PINID_PWM1,
	 .fun = PIN_FUN3,
	 },
};

void __init mx28evk_pins_init(void)
{
	int i;
	struct pin_desc *pin;
	for (i = 0; i < ARRAY_SIZE(mx28evk_fixed_pins); i++) {
		pin = &mx28evk_fixed_pins[i];
		if (pin->fun == PIN_GPIO)
			gpio_request(MXS_PIN_TO_GPIO(pin->id), pin->name);
		else
			mxs_request_pin(pin->id, pin->fun, pin->name);
		if (pin->drive) {
			mxs_set_strength(pin->id, pin->strength, pin->name);
			mxs_set_voltage(pin->id, pin->voltage, pin->name);
		}
		if (pin->pull)
			mxs_set_pullup(pin->id, pin->pullup, pin->name);
		if (pin->fun == PIN_GPIO) {
			if (pin->input)
				gpio_direction_input(MXS_PIN_TO_GPIO(pin->id));
			else
				gpio_direction_output(MXS_PIN_TO_GPIO(pin->id),
						      pin->data);
		}
	}
}
