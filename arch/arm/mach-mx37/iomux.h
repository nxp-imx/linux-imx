/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MACH_MX37_IOMUX_H__
#define __MACH_MX37_IOMUX_H__

#include <linux/types.h>
#include <mach/gpio.h>
#include "mx37_pins.h"

/*!
 * @file mach-mx37/iomux.h
 *
 * @brief I/O Muxing control definitions and functions
 *
 * @ingroup GPIO_MX37
 */

typedef unsigned int iomux_pin_name_t;

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
	PAD_CTL_47K_PU = 0x1 << 4,
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
 * various IOMUX general purpose functions
 */
typedef enum iomux_gp_func {
	MUX_IPD_ESDHC_DREQ_B = 0x0 << 0,
	MUX_XDRQ = 0x1 << 0,
	MUX_EMI_DMA_ACCESS_1 = 0x0 << 4,
	MUX_KEY_COL2 = 0x1 << 4,
	MUX_TAMPER_DETECT_EN = 0x1 << 8,
	MUX_IPUv3D_TVE = 0x0 << 12,
	MUX_IPUv3D_CAMP = 0x1 << 12,
} iomux_gp_func_t;

/*!
 * various IOMUX input select register index
 */
typedef enum iomux_input_select {
	MUX_IN_CCM_PLL1_BYPASS_CLK = 0,
	MUX_IN_CCM_PLL2_BYPASS_CLK,
	MUX_IN_CCM_PLL3_BYPASS_CLK,
	MUX_IN_CSPI3_CSPI_CLK,
	MUX_IN_CSPI3_MISO,
	MUX_IN_CSPI3_MOSI,
	MUX_IN_EMI_READ_MADDR_DATA_0,
	MUX_IN_EMI_READ_MADDR_DATA_10,
	MUX_IN_EMI_READ_MADDR_DATA_11,
	MUX_IN_EMI_READ_MADDR_DATA_12,
	MUX_IN_EMI_READ_MADDR_DATA_13,
	MUX_IN_EMI_READ_MADDR_DATA_14,
	MUX_IN_EMI_READ_MADDR_DATA_15,
	MUX_IN_EMI_READ_MADDR_DATA_1,
	MUX_IN_EMI_READ_MADDR_DATA_2,
	MUX_IN_EMI_READ_MADDR_DATA_3,
	MUX_IN_EMI_READ_MADDR_DATA_4,
	MUX_IN_EMI_READ_MADDR_DATA_5,
	MUX_IN_EMI_READ_MADDR_DATA_6,
	MUX_IN_EMI_READ_MADDR_DATA_7,
	MUX_IN_EMI_READ_MADDR_DATA_8,
	MUX_IN_EMI_READ_MADDR_DATA_9,
	MUX_IN_EMI_NFC_READ_DATA_IN_0,
	MUX_IN_EMI_NFC_READ_DATA_IN_10,
	MUX_IN_EMI_NFC_READ_DATA_IN_11,
	MUX_IN_EMI_NFC_READ_DATA_IN_12,
	MUX_IN_EMI_NFC_READ_DATA_IN_13,
	MUX_IN_EMI_NFC_READ_DATA_IN_14,
	MUX_IN_EMI_NFC_READ_DATA_IN_15,
	MUX_IN_EMI_NFC_READ_DATA_IN_1,
	MUX_IN_EMI_NFC_READ_DATA_IN_2,
	MUX_IN_EMI_NFC_READ_DATA_IN_3,
	MUX_IN_EMI_NFC_READ_DATA_IN_4,
	MUX_IN_EMI_NFC_READ_DATA_IN_5,
	MUX_IN_EMI_NFC_READ_DATA_IN_6,
	MUX_IN_EMI_NFC_READ_DATA_IN_7,
	MUX_IN_EMI_NFC_READ_DATA_IN_8,
	MUX_IN_EMI_NFC_READ_DATA_IN_9,
	MUX_IN_FEC_FEC_COL,
	MUX_IN_FEC_FEC_CRS, MUX_IN_FEC_FEC_MDI,
	MUX_IN_FEC_FEC_RDATA_0,
	MUX_IN_FEC_FEC_RX_CLK,
	MUX_IN_FEC_FEC_RX_DV,
	MUX_IN_FEC_FEC_RX_ER,
	MUX_IN_FEC_FEC_TX_CLK,
	MUX_IN_I2C1_SCL,
	MUX_IN_I2C1_SDA,
	MUX_IN_I2C2_SCL,
	MUX_IN_I2C2_SDA,
	MUX_IN_I2C3_SCL,
	MUX_IN_I2C3_SDA,
	MUX_IN_IPU_DI_0_IND_DISPB_D0_VSYNC,
	MUX_IN__IPU_DI_0_IND_DISPB_SD_D,
	MUX_IN_KPP_ROW_0,
	MUX_IN_KPP_ROW_1,
	MUX_IN_KPP_ROW_2,
	MUX_IN_KPP_ROW_3,
	MUX_IN_KPP_ROW_4,
	MUX_IN_KPP_ROW_5,
	MUX_IN_KPP_ROW_6,
	MUX_IN_KPP_ROW_7,
	MUX_IN_UART1_UART_RTS_B,
	MUX_IN_UART1_UART_RXD_MUX,
	MUX_IN_UART2_UART_RTS_B,
	MUX_IN_UART2_UART_RXD_MUX,
	MUX_IN_UART3_UART_RTS_B,
	MUX_IN_UART3_UART_RXD_MUX,
} iomux_input_select_t;

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
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 * @param  index  0 for GPR0 and 1 for GPR1
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en, u8 index);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
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
 * @param  input        index of input select register as defined in \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config);

#endif				/*  __MACH_MX37_IOMUX_H__ */
