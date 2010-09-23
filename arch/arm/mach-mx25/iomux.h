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
#ifndef __MACH_MX25_IOMUX_H__
#define __MACH_MX25_IOMUX_H__

#include <linux/types.h>
#include <mach/gpio.h>
#include "mx25_pins.h"

/*!
 * @file mach-mx25/iomux.h
 *
 * @brief I/O Muxing control definitions and functions
 *
 * @ingroup GPIO_MX25
 */

typedef unsigned int iomux_pin_name_t;

/*!
 * IOMUX functions
 * SW_MUX_CTL
 */
typedef enum iomux_pin_config {
	MUX_CONFIG_FUNC = 0,	/*!< used as function */
	MUX_CONFIG_ALT1,	/*!< used as alternate function 1 */
	MUX_CONFIG_ALT2,	/*!< used as alternate function 2 */
	MUX_CONFIG_ALT3,	/*!< used as alternate function 3 */
	MUX_CONFIG_ALT4,	/*!< used as alternate function 4 */
	MUX_CONFIG_ALT5,	/*!< used as alternate function 5 */
	MUX_CONFIG_ALT6,	/*!< used as alternate function 6 */
	MUX_CONFIG_ALT7,	/*!< used as alternate function 7 */
	MUX_CONFIG_SION = 0x1 << 4,	/*!< used as LOOPBACK:MUX SION bit */
	MUX_CONFIG_GPIO = MUX_CONFIG_ALT5,	/*!< used as GPIO */
} iomux_pin_cfg_t;

/*!
 * IOMUX pad functions
 * SW_PAD_CTL
 */
typedef enum iomux_pad_config {
	PAD_CTL_DRV_3_3V = 0x0 << 13,
	PAD_CTL_DRV_1_8V = 0x1 << 13,
	PAD_CTL_HYS_CMOS = 0x0 << 8,
	PAD_CTL_HYS_SCHMITZ = 0x1 << 8,
	PAD_CTL_PKE_NONE = 0x0 << 7,
	PAD_CTL_PKE_ENABLE = 0x1 << 7,
	PAD_CTL_PUE_KEEPER = 0x0 << 6,
	PAD_CTL_PUE_PULL = 0x1 << 6,
	PAD_CTL_PUE_PUD = 0x1 << 6,
	PAD_CTL_100K_PD = 0x0 << 4,
	PAD_CTL_47K_PU = 0x1 << 4,
	PAD_CTL_100K_PU = 0x2 << 4,
	PAD_CTL_22K_PU = 0x3 << 4,
	PAD_CTL_ODE_CMOS = 0x0 << 3,
	PAD_CTL_ODE_OpenDrain = 0x1 << 3,
	PAD_CTL_DRV_NORMAL = 0x0 << 1,
	PAD_CTL_DRV_HIGH = 0x1 << 1,
	PAD_CTL_DRV_MAX = 0x2 << 1,
	PAD_CTL_SRE_SLOW = 0x0 << 0,
	PAD_CTL_SRE_FAST = 0x1 << 0
} iomux_pad_config_t;

/*!
 * IOMUX general purpose functions
 * IOMUXC_GPR1
 */
typedef enum iomux_gp_func {
	MUX_SDCTL_CSD0_SEL = 0x1 << 0,
	MUX_SDCTL_CSD1_SEL = 0x1 << 1,
} iomux_gp_func_t;

/*!
 * IOMUX SELECT_INPUT register index
 * Base register is IOMUXSW_INPUT_CTL in iomux.c
 */
typedef enum iomux_input_select {
	MUX_IN_AUDMUX_P4_INPUT_DA_AMX = 0,
	MUX_IN_AUDMUX_P4_INPUT_DB_AMX,
	MUX_IN_AUDMUX_P4_INPUT_RXCLK_AMX,
	MUX_IN_AUDMUX_P4_INPUT_RXFS_AMX,
	MUX_IN_AUDMUX_P4_INPUT_TXCLK_AMX,
	MUX_IN_AUDMUX_P4_INPUT_TXFS_AMX,
	MUX_IN_AUDMUX_P7_INPUT_DA_AMX,
	MUX_IN_AUDMUX_P7_INPUT_TXFS_AMX,
	MUX_IN_CAN1_IPP_IND_CANRX,
	MUX_IN_CAN2_IPP_IND_CANRX,
	MUX_IN_CSI_IPP_CSI_D_0,
	MUX_IN_CSI_IPP_CSI_D_1,
	MUX_IN_CSPI1_IPP_IND_SS3_B,
	MUX_IN_CSPI2_IPP_CSPI_CLK_IN,
	MUX_IN_CSPI2_IPP_IND_DATAREADY_B,
	MUX_IN_CSPI2_IPP_IND_MISO,
	MUX_IN_CSPI2_IPP_IND_MOSI,
	MUX_IN_CSPI2_IPP_IND_SS0_B,
	MUX_IN_CSPI2_IPP_IND_SS1_B,
	MUX_IN_CSPI3_IPP_CSPI_CLK_IN,
	MUX_IN_CSPI3_IPP_IND_DATAREADY_B,
	MUX_IN_CSPI3_IPP_IND_MISO,
	MUX_IN_CSPI3_IPP_IND_MOSI,
	MUX_IN_CSPI3_IPP_IND_SS0_B,
	MUX_IN_CSPI3_IPP_IND_SS1_B,
	MUX_IN_CSPI3_IPP_IND_SS2_B,
	MUX_IN_CSPI3_IPP_IND_SS3_B,
	MUX_IN_ESDHC1_IPP_DAT4_IN,
	MUX_IN_ESDHC1_IPP_DAT5_IN,
	MUX_IN_ESDHC1_IPP_DAT6_IN,
	MUX_IN_ESDHC1_IPP_DAT7_IN,
	MUX_IN_ESDHC2_IPP_CARD_CLK_IN,
	MUX_IN_ESDHC2_IPP_CMD_IN,
	MUX_IN_ESDHC2_IPP_DAT0_IN,
	MUX_IN_ESDHC2_IPP_DAT1_IN,
	MUX_IN_ESDHC2_IPP_DAT2_IN,
	MUX_IN_ESDHC2_IPP_DAT3_IN,
	MUX_IN_ESDHC2_IPP_DAT4_IN,
	MUX_IN_ESDHC2_IPP_DAT5_IN,
	MUX_IN_ESDHC2_IPP_DAT6_IN,
	MUX_IN_ESDHC2_IPP_DAT7_IN,
	MUX_IN_FEC_FEC_COL,
	MUX_IN_FEC_FEC_CRS,
	MUX_IN_FEC_FEC_RDATA_2,
	MUX_IN_FEC_FEC_RDATA_3,
	MUX_IN_FEC_FEC_RX_CLK,
	MUX_IN_FEC_FEC_RX_ER,
	MUX_IN_I2C2_IPP_SCL_IN,
	MUX_IN_I2C2_IPP_SDA_IN,
	MUX_IN_I2C3_IPP_SCL_IN,
	MUX_IN_I2C3_IPP_SDA_IN,
	MUX_IN_KPP_IPP_IND_COL_4,
	MUX_IN_KPP_IPP_IND_COL_5,
	MUX_IN_KPP_IPP_IND_COL_6,
	MUX_IN_KPP_IPP_IND_COL_7,
	MUX_IN_KPP_IPP_IND_ROW_4,
	MUX_IN_KPP_IPP_IND_ROW_5,
	MUX_IN_KPP_IPP_IND_ROW_6,
	MUX_IN_KPP_IPP_IND_ROW_7,
	MUX_IN_SIM1_PIN_SIM_RCVD1_IN,
	MUX_IN_SIM1_PIN_SIM_SIMPD1,
	MUX_IN_SIM1_SIM_RCVD1_IO,
	MUX_IN_SIM2_PIN_SIM_RCVD1_IN,
	MUX_IN_SIM2_PIN_SIM_SIMPD1,
	MUX_IN_SIM2_SIM_RCVD1_IO,
	MUX_IN_UART3_IPP_UART_RTS_B,
	MUX_IN_UART3_IPP_UART_RXD_MUX,
	MUX_IN_UART4_IPP_UART_RTS_B,
	MUX_IN_UART4_IPP_UART_RXD_MUX,
	MUX_IN_UART5_IPP_UART_RTS_B,
	MUX_IN_UART5_IPP_UART_RXD_MUX,
	MUX_IN_USB_TOP_IPP_IND_OTG_USB_OC,
	MUX_IN_USB_TOP_IPP_IND_UH2_USB_OC,
} iomux_input_select_t;

/*!
 * IOMUX input functions
 * SW_SELECT_INPUT bits 2-0
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
} iomux_input_cfg_t;

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
 * @param  cfg		an input function as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t cfg);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  cfg		an input function as defined in \b #iomux_pin_cfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t cfg);

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
 * @param  config       the ORed value of elements defined in \b
 *				#iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config);

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in \b
 *				#iomux_input_select_t
 * @param  config       the binary value of elements defined in \b
 *				#iomux_input_cfg_t
 */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config);
#endif
