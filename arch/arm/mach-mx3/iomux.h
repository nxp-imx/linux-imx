/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MACH_MX31_IOMUX_H__
#define __MACH_MX31_IOMUX_H__

#include <linux/types.h>
#include <mach/gpio.h>
#include "mx31_pins.h"

typedef unsigned int iomux_pin_name_t;
/*!
 * @file mach-mx3/iomux.h
 *
 * @brief I/O Muxing control definitions and functions
 *
 * @ingroup GPIO_MX31
 */

/*!
 * various IOMUX output functions
 */
typedef enum iomux_output_config {
	OUTPUTCONFIG_GPIO = 0,	/*!< used as GPIO */
	OUTPUTCONFIG_FUNC,	/*!< used as function */
	OUTPUTCONFIG_ALT1,	/*!< used as alternate function 1 */
	OUTPUTCONFIG_ALT2,	/*!< used as alternate function 2 */
	OUTPUTCONFIG_ALT3,	/*!< used as alternate function 3 */
	OUTPUTCONFIG_ALT4,	/*!< used as alternate function 4 */
	OUTPUTCONFIG_ALT5,	/*!< used as alternate function 5 */
	OUTPUTCONFIG_ALT6	/*!< used as alternate function 6 */
} iomux_pin_ocfg_t;

/*!
 * various IOMUX input functions
 */
typedef enum iomux_input_config {
	INPUTCONFIG_NONE = 0,	/*!< not configured for input */
	INPUTCONFIG_GPIO = 1 << 0,	/*!< used as GPIO */
	INPUTCONFIG_FUNC = 1 << 1,	/*!< used as function */
	INPUTCONFIG_ALT1 = 1 << 2,	/*!< used as alternate function 1 */
	INPUTCONFIG_ALT2 = 1 << 3	/*!< used as alternate function 2 */
} iomux_pin_icfg_t;

/*!
 * various IOMUX pad functions
 */
typedef enum iomux_pad_config {
	PAD_CTL_NOLOOPBACK = 0x0 << 9,
	PAD_CTL_LOOPBACK = 0x1 << 9,
	PAD_CTL_PKE_NONE = 0x0 << 8,
	PAD_CTL_PKE_ENABLE = 0x1 << 8,
	PAD_CTL_PUE_KEEPER = 0x0 << 7,
	PAD_CTL_PUE_PUD = 0x1 << 7,
	PAD_CTL_100K_PD = 0x0 << 5,
	PAD_CTL_100K_PU = 0x1 << 5,
	PAD_CTL_47K_PU = 0x2 << 5,
	PAD_CTL_22K_PU = 0x3 << 5,
	PAD_CTL_HYS_CMOS = 0x0 << 4,
	PAD_CTL_HYS_SCHMITZ = 0x1 << 4,
	PAD_CTL_ODE_CMOS = 0x0 << 3,
	PAD_CTL_ODE_OpenDrain = 0x1 << 3,
	PAD_CTL_DRV_NORMAL = 0x0 << 1,
	PAD_CTL_DRV_HIGH = 0x1 << 1,
	PAD_CTL_DRV_MAX = 0x2 << 1,
	PAD_CTL_SRE_SLOW = 0x0 << 0,
	PAD_CTL_SRE_FAST = 0x1 << 0
} iomux_pad_config_t;

/*!
 * various IOMUX general purpose functions
 */
typedef enum iomux_gp_func {
	MUX_PGP_FIRI = 0x1 << 0,
	MUX_DDR_MODE = 0x1 << 1,
	MUX_PGP_CSPI_BB = 0x1 << 2,
	MUX_PGP_ATA_1 = 0x1 << 3,
	MUX_PGP_ATA_2 = 0x1 << 4,
	MUX_PGP_ATA_3 = 0x1 << 5,
	MUX_PGP_ATA_4 = 0x1 << 6,
	MUX_PGP_ATA_5 = 0x1 << 7,
	MUX_PGP_ATA_6 = 0x1 << 8,
	MUX_PGP_ATA_7 = 0x1 << 9,
	MUX_PGP_ATA_8 = 0x1 << 10,
	MUX_PGP_UH2 = 0x1 << 11,
	MUX_SDCTL_CSD0_SEL = 0x1 << 12,
	MUX_SDCTL_CSD1_SEL = 0x1 << 13,
	MUX_CSPI1_UART3 = 0x1 << 14,
	MUX_EXTDMAREQ2_MBX_SEL = 0x1 << 15,
	MUX_TAMPER_DETECT_EN = 0x1 << 16,
	MUX_PGP_USB_4WIRE = 0x1 << 17,
	MUX_PGB_USB_COMMON = 0x1 << 18,
	MUX_SDHC_MEMSTICK1 = 0x1 << 19,
	MUX_SDHC_MEMSTICK2 = 0x1 << 20,
	MUX_PGP_SPLL_BYP = 0x1 << 21,
	MUX_PGP_UPLL_BYP = 0x1 << 22,
	MUX_PGP_MSHC1_CLK_SEL = 0x1 << 23,
	MUX_PGP_MSHC2_CLK_SEL = 0x1 << 24,
	MUX_CSPI3_UART5_SEL = 0x1 << 25,
	MUX_PGP_ATA_9 = 0x1 << 26,
	MUX_PGP_USB_SUSPEND = 0x1 << 27,
	MUX_PGP_USB_OTG_LOOPBACK = 0x1 << 28,
	MUX_PGP_USB_HS1_LOOPBACK = 0x1 << 29,
	MUX_PGP_USB_HS2_LOOPBACK = 0x1 << 30,
	MUX_CLKO_DDR_MODE = 0x1 << 31,
} iomux_gp_func_t;

/*!
 * This function is used to configure a pin through the IOMUX module.
 *
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 * @return 		0 if successful; Non-zero otherwise
 */
int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		     iomux_pin_icfg_t in);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pins
 * @param  config       ORed value of elements defined in \b #iomux_pad_config_t
 */
void iomux_config_pad(iomux_pin_name_t pin, __u32 config);

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void iomux_config_gpr(iomux_gp_func_t gp, bool en);

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		      iomux_pin_icfg_t in);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		    iomux_pin_icfg_t in);

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config);

#endif
