/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MACH_MX5_IOMUX_H__
#define __MACH_MX5_IOMUX_H__

#include <linux/types.h>
#include <mach/gpio.h>

/*!
 * @file mach-mx5/iomux.h
 *
 * @brief I/O Muxing control definitions and functions
 *
 * @ingroup GPIO_MX5
 */

/*!
 * @name IOMUX/PAD Bit field definitions
 */

/*! @{ */

/*!
 * In order to identify pins more effectively, each mux-controlled pin's
 * enumerated value is constructed in the following way:
 *
 * -------------------------------------------------------------------
 * 31-29 | 28 - 24 |  23 - 21 | 20  - 10| 9 - 0
 * -------------------------------------------------------------------
 * IO_P  |  IO_I  | GPIO_I | PAD_I  | MUX_I
 * -------------------------------------------------------------------
 *
 * Bit 0 to 9 contains MUX_I used to identify the register
 * offset (0-based. base is IOMUX_module_base) defined in the Section
 * "sw_pad_ctl & sw_mux_ctl details" of the IC Spec. The
 * similar field definitions are used for the pad control register.
 * For example, the MX51_PIN_ETM_D0 is defined in the enumeration:
 *    ( (0x28 - MUX_I_START) << MUX_I)|( (0x250 - PAD_I_START) << PAD_I)
 * It means the mux control register is at register offset 0x28. The pad control
 * register offset is: 0x250 and also occupy the least significant bits
 * within the register.
 */

/*!
 * Starting bit position within each entry of \b iomux_pins to represent the
 * MUX control register offset
 */
#define MUX_I			0
/*!
 * Starting bit position within each entry of \b iomux_pins to represent the
 * PAD control register offset
 */
#define PAD_I			10
/*!
 * Starting bit position within each entry of \b iomux_pins to represent which
 * mux mode is for GPIO (0-based)
 */
#define GPIO_I			21

#define NON_GPIO_PORT		0x7
#define PIN_TO_MUX_MASK		((1 << (PAD_I - MUX_I)) - 1)
#define PIN_TO_PAD_MASK		((1 << (GPIO_I - PAD_I)) - 1)
#define PIN_TO_ALT_GPIO_MASK		((1 << (MUX_IO_I - GPIO_I)) - 1)

#define NON_MUX_I		PIN_TO_MUX_MASK
#define NON_PAD_I		PIN_TO_PAD_MASK

#define PIN_TO_IOMUX_MUX(pin)	((pin >> MUX_I) & PIN_TO_MUX_MASK)
#define PIN_TO_IOMUX_PAD(pin)	((pin >> PAD_I) & PIN_TO_PAD_MASK)
#define PIN_TO_ALT_GPIO(pin)	((pin >> GPIO_I) & PIN_TO_ALT_GPIO_MASK)
#define PIN_TO_IOMUX_INDEX(pin)	(PIN_TO_IOMUX_MUX(pin) >> 2)

/*! @} End IOMUX/PAD Bit field definitions */

typedef unsigned int iomux_pin_name_t;
typedef unsigned int iomux_input_select_t;

/*!
 * various IOMUX output functions
 */
typedef enum iomux_config {
	IOMUX_CONFIG_ALT0,	/*!< used as alternate function 0 */
	IOMUX_CONFIG_ALT1,	/*!< used as alternate function 1 */
	IOMUX_CONFIG_ALT2,	/*!< used as alternate function 2 */
	IOMUX_CONFIG_ALT3,	/*!< used as alternate function 3 */
	IOMUX_CONFIG_ALT4,	/*!< used as alternate function 4 */
	IOMUX_CONFIG_ALT5,	/*!< used as alternate function 5 */
	IOMUX_CONFIG_ALT6,	/*!< used as alternate function 6 */
	IOMUX_CONFIG_ALT7,	/*!< used as alternate function 7 */
	IOMUX_CONFIG_GPIO,	/*!< added to help user use GPIO mode */
	IOMUX_CONFIG_SION = 0x1 << 4,	/*!< used as LOOPBACK:MUX SION bit */
} iomux_pin_cfg_t;

/*!
 * various IOMUX pad functions
 */
typedef enum iomux_pad_config {
	PAD_CTL_SRE_SLOW = 0x0 << 0,
	PAD_CTL_SRE_FAST = 0x1 << 0,
	PAD_CTL_DRV_LOW = 0x0 << 1,
	PAD_CTL_DRV_MEDIUM = 0x1 << 1,
	PAD_CTL_DRV_HIGH = 0x2 << 1,
	PAD_CTL_DRV_MAX = 0x3 << 1,
	PAD_CTL_ODE_OPENDRAIN_NONE = 0x0 << 3,
	PAD_CTL_ODE_OPENDRAIN_ENABLE = 0x1 << 3,
	PAD_CTL_100K_PD = 0x0 << 4,
	PAD_CTL_360K_PD = 0x0 << 4,
	PAD_CTL_47K_PU = 0x1 << 4,
	PAD_CTL_75k_PU = 0x1 << 4,
	PAD_CTL_100K_PU = 0x2 << 4,
	PAD_CTL_22K_PU = 0x3 << 4,
	PAD_CTL_PUE_KEEPER = 0x0 << 6,
	PAD_CTL_PUE_PULL = 0x1 << 6,
	PAD_CTL_PKE_NONE = 0x0 << 7,
	PAD_CTL_PKE_ENABLE = 0x1 << 7,
	PAD_CTL_HYS_NONE = 0x0 << 8,
	PAD_CTL_HYS_ENABLE = 0x1 << 8,
	PAD_CTL_DDR_INPUT_CMOS = 0x0 << 9,
	PAD_CTL_DDR_INPUT_DDR = 0x1 << 9,
	PAD_CTL_DRV_VOT_LOW = 0x0 << 13,
	PAD_CTL_DRV_VOT_HIGH = 0x1 << 13,
} iomux_pad_config_t;

/*!
 * various IOMUX input functions
 */
typedef enum iomux_input_config {
	INPUT_CTL_PATH0 = 0x0,
	INPUT_CTL_PATH1,
	INPUT_CTL_PATH2,
	INPUT_CTL_PATH3,
	INPUT_CTL_PATH4,
	INPUT_CTL_PATH5,
	INPUT_CTL_PATH6,
	INPUT_CTL_PATH7,
} iomux_input_config_t;

struct mxc_iomux_pin_cfg {
	iomux_pin_name_t pin;
	u8 mux_mode;
	u16 pad_cfg;
	u8 in_select;
	u8 in_mode;
};

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config      the ORed value of elements defined in
 *                             \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config);

/*!
 * This function gets the current pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @return		current pad value
 */
unsigned int mxc_iomux_get_pad(iomux_pin_name_t pin);

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in
 *                              \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config);

#endif				/*  __MACH_MX5_IOMUX_H__ */
