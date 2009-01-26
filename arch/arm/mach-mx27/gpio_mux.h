/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 *@file mach-mx27/gpio_mux.h
 *@brief This file contains the private definition  .
 * @ingroup GPIO_MX27
 */

#ifndef __ARCH_ARM_MACH_MX27_GPIO_MUX_H__
#define __ARCH_ARM_MACH_MX27_GPIO_MUX_H__

#include "mx27_pins.h"

/*!
 * This enumeration data type defines the modes of the pin .
 *	GPIO_MUX_PRIMARY is the primary mode.
 *	GPIO_MUX_ALT is the alternate mode.
 *	GPIO_MUX_GPIO is the output mode and the signal source is data register.
 *	GPIO_MUX_INPUT1 is the input mode and the signal destination is A_OUT.
 *	GPIO_MUX_INPUT2 is the input mode and the signal destination is B_OUT.
 *	GPIO_MUX_OUTPUT1 is the output mode and the signal destination is A_IN.
 *	GPIO_MUX_OUTPUT2 is the output mode and the signal destination is B_IN.
 *	GPIO_MUX_OUTPUT3 is the output mode and the signal destination is C_IN.
 */
typedef enum {
	GPIO_MUX_PRIMARY,
	GPIO_MUX_ALT,
	GPIO_MUX_GPIO,
	GPIO_MUX_INPUT1,
	GPIO_MUX_INPUT2,
	GPIO_MUX_OUTPUT1,
	GPIO_MUX_OUTPUT2,
	GPIO_MUX_OUTPUT3,
} gpio_mux_mode_t;

/*!
 * This function is just used to request a pin and configure it.
 * @param pin	a pin number as defined in \b #iomux_pin_name_t
 * @param mode	a module as define in \b #gpio_mux_mode_t;
 * @return	0 if successful, Non-zero otherwise
 */
extern int gpio_request_mux(iomux_pin_name_t pin, gpio_mux_mode_t mode);

/*!
 * This function is just used to configure a pin .
 * @param pin	a pin number as defined in \b #iomux_pin_name_t
 * @param mode	a module as define in \b #gpio_mux_mode_t;
 * @return	0 if successful, Non-zero otherwise
 */
extern int gpio_config_mux(iomux_pin_name_t pin, gpio_mux_mode_t mode);

/*!
 * This function is just used to enable or disable the pull up feature .
 * @param pin   a pin number as defined in \b #iomux_pin_name_t
 * @param en    0 if disable, Non-zero enable
 * @return      0 if successful, Non-zero otherwise
 */
extern int gpio_set_puen(iomux_pin_name_t pin, bool en);

/*!
 * This function is just used to release a pin.
 * @param pin	a pin number as defined in \b #iomux_pin_name_t
 * @return	none
 */
extern void gpio_free_mux(iomux_pin_name_t pin);

#endif				/* __ARCH_ARM_MACH_MX27_GPIO_MUX_H__ */
