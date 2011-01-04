/*
 * da9052 LED module declarations.
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_LED_H
#define __LINUX_MFD_DA9052_LED_H

struct da9052_led_platform_data {
#define DA9052_LED_4			4
#define DA9052_LED_5			5
#define DA9052_LED_MAX			2
	int id;
	const char *name;
	const char *default_trigger;
};

struct da9052_leds_platform_data {
	int num_leds;
	struct da9052_led_platform_data *led;
};

#endif /* __LINUX_MFD_DA9052_LED_H */
