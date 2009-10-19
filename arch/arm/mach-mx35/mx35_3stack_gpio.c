/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include "board-mx35_3stack.h"
#include "iomux.h"

/*!
 * @file mach-mx35/mx35_3stack_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX35
 */

/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by \b fixup_mx31ads()
 * during system startup. This function is board specific.
 */
void mx35_3stack_gpio_init(void)
{
	/* config CS5 */
	mxc_request_iomux(MX35_PIN_CS5, MUX_CONFIG_FUNC);

	/* configure capture pin for ckil input */
	mxc_request_iomux(MX35_PIN_CAPTURE, MUX_CONFIG_ALT4);
	mxc_iomux_set_pad(MX35_PIN_CAPTURE,
			  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_PUD);
	mxc_iomux_set_input(MUX_IN_CCM_32K_MUXED, INPUT_CTL_PATH0);

}

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
	/*
	 * Configure the IOMUX control registers for the UART signals
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		mxc_request_iomux(MX35_PIN_RXD1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_TXD1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RTS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CTS1, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_RXD1,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD1,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX35_PIN_RTS1,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_CTS1,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);

		break;
		/* UART 2 IOMUX Configs */
	case 1:
		mxc_request_iomux(MX35_PIN_TXD2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RXD2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RTS2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CTS2, MUX_CONFIG_FUNC);
		mxc_iomux_set_pad(MX35_PIN_RXD2,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD2,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX35_PIN_RTS2,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_CTS2,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		mxc_request_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_ALT2);

		mxc_iomux_set_pad(MX35_PIN_FEC_TX_CLK,
				PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK,
				PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX35_PIN_FEC_RX_DV,
				PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_FEC_COL,
				PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);

		mxc_iomux_set_input(MUX_IN_UART3_UART_RTS_B, INPUT_CTL_PATH2);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH3);
		break;
	default:
		break;
	}

}

EXPORT_SYMBOL(gpio_uart_active);

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	switch (port) {
	case 0:
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RXD1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_TXD1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RTS1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CTS1), NULL);

		mxc_free_iomux(MX35_PIN_RXD1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_RTS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CTS1, MUX_CONFIG_GPIO);
		break;
	case 1:
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RXD2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_TXD2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_RTS2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CTS2), NULL);

		mxc_free_iomux(MX35_PIN_RXD2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_RTS2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CTS2, MUX_CONFIG_GPIO);
		break;
	case 2:
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_CLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_COL), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_DV), NULL);

		mxc_free_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);

		mxc_iomux_set_input(MUX_IN_UART3_UART_RTS_B, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH0);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_uart_inactive);

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{
}

EXPORT_SYMBOL(config_uartdma_event);

void gpio_fec_active(void)
{
	mxc_request_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_EN, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RX_ERR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_CRS, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_RDATA3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FEC_TDATA3, MUX_CONFIG_FUNC);

#define FEC_PAD_CTL_COMMON (PAD_CTL_DRV_3_3V|PAD_CTL_PUE_PUD| \
			PAD_CTL_ODE_CMOS|PAD_CTL_DRV_NORMAL|PAD_CTL_SRE_SLOW)
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_CLK, FEC_PAD_CTL_COMMON |
			  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_CLK,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_DV,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_COL,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA0,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_EN,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDC,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_MDIO,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_22K_PU);
	mxc_iomux_set_pad(MX35_PIN_FEC_TX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RX_ERR,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_CRS,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA1,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA2,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_RDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_SCHMITZ |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PD);
	mxc_iomux_set_pad(MX35_PIN_FEC_TDATA3,
			  FEC_PAD_CTL_COMMON | PAD_CTL_HYS_CMOS |
			  PAD_CTL_PKE_NONE | PAD_CTL_100K_PD);
#undef FEC_PAD_CTL_COMMON
	/* Pull GPIO1_5 to be high for routing signal to FEC */
	if (board_is_rev(BOARD_REV_2)) {
		mxc_request_iomux(MX35_PIN_COMPARE, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX35_PIN_COMPARE, PAD_CTL_DRV_NORMAL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
				PAD_CTL_DRV_3_3V | PAD_CTL_PUE_PUD |
				PAD_CTL_SRE_SLOW);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_COMPARE), "compare");
		gpio_direction_output(IOMUX_TO_GPIO(MX35_PIN_COMPARE), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX35_PIN_COMPARE), 1);
	}

	/* FEC enable */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 2, 1);
	/* FEC reset */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 7, 0);
	msleep(10);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 7, 1);
	msleep(100);
}

EXPORT_SYMBOL(gpio_fec_active);

void gpio_fec_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_CLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_CLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_DV), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_COL), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_EN), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_MDC), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_MDIO), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TX_ERR), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RX_ERR), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_CRS), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_RDATA3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_FEC_TDATA3), NULL);

	mxc_free_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TX_EN, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TX_ERR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RX_ERR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_CRS, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_RDATA3, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FEC_TDATA3, MUX_CONFIG_GPIO);

	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 2, 0);

	/* Free GPIO1_5 */
	if (board_is_rev(BOARD_REV_2)) {
		gpio_free(IOMUX_TO_GPIO(MX35_PIN_COMPARE));
		mxc_free_iomux(MX35_PIN_COMPARE, MUX_CONFIG_GPIO);
	}
}

EXPORT_SYMBOL(gpio_fec_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{

#define PAD_CONFIG (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_ODE_OpenDrain)

	switch (i2c_num) {
	case 0:
		mxc_request_iomux(MX35_PIN_I2C1_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C1_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C1_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C1_DAT, PAD_CONFIG);
		break;
	case 1:
		mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, PAD_CONFIG);

		break;
	case 2:
		mxc_request_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_ALT1);
		mxc_request_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX35_PIN_TX3_RX2, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_TX2_RX3, PAD_CONFIG);
		break;
	default:
		break;
	}

#undef PAD_CONFIG

}

EXPORT_SYMBOL(gpio_i2c_active);

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		mxc_request_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_GPIO);
		mxc_request_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_i2c_inactive);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_request_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_CSPI1_MOSI,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_MISO,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS0,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS1,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SCLK,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SPI_RDY,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_DRV_NORMAL);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_active);

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_MOSI), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_MISO), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SS0), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SS1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SCLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX35_PIN_CSPI1_SPI_RDY), NULL);

		mxc_free_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_GPIO);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_inactive);

/*!
 * Setup GPIO for LCD to be active
 */
void gpio_lcd_active(void)
{
	mxc_request_iomux(MX35_PIN_LD0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD6, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD7, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD8, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD9, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD10, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD11, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD12, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD13, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD14, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD15, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD16, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD17, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_VSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_HSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_FPSHIFT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_DRDY, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CONTRAST, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_lcd_active);

/*!
 * Setup GPIO for LCD to be inactive
 */
void gpio_lcd_inactive(void)
{
}

EXPORT_SYMBOL(gpio_lcd_inactive);

/*!
 * Setup pin for touchscreen
 */
void gpio_tsc_active(void)
{
	unsigned int pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU;
	mxc_request_iomux(MX35_PIN_CAPTURE, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX35_PIN_CAPTURE, pad_val);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_CAPTURE), "capture");
	gpio_direction_input(IOMUX_TO_GPIO(MX35_PIN_CAPTURE));
}

/*!
 * Release pin for touchscreen
 */
void gpio_tsc_inactive(void)
{
	gpio_free(IOMUX_TO_GPIO(MX35_PIN_CAPTURE));
	mxc_free_iomux(MX35_PIN_CAPTURE, MUX_CONFIG_GPIO);
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	unsigned int pad_val;

	switch (module) {
	case 0:
		mxc_request_iomux(MX35_PIN_SD1_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA0,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA1,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA2,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA3,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
#else
		/* MUX4_CTR , 0: SD2 to WIFI, 1:SD2 to SD1 8bit */
		if (board_is_rev(BOARD_REV_2))
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,
					      7, 1);
		mxc_request_iomux(MX35_PIN_SD2_CMD,
				  MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_CLK,
				  MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA0,
				  MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA1,
				  MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
#endif

		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_MAX |
		    PAD_CTL_47K_PU | PAD_CTL_SRE_FAST;
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2, pad_val);
		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_DRV_MAX | PAD_CTL_47K_PU | PAD_CTL_SRE_FAST;
		mxc_iomux_set_pad(MX35_PIN_SD1_CLK, pad_val);
		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_MAX |
		    PAD_CTL_100K_PU | PAD_CTL_SRE_FAST;
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3, pad_val);
#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
#else
		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_MAX |
		    PAD_CTL_47K_PU | PAD_CTL_SRE_FAST;
		mxc_iomux_set_pad(MX35_PIN_SD2_CMD, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA0, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA1, pad_val);
		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_DRV_MAX | PAD_CTL_47K_PU | PAD_CTL_SRE_FAST;
		mxc_iomux_set_pad(MX35_PIN_SD2_CLK, pad_val);
#endif
		break;
	case 1:
		/* MUX4_CTR , 0: SD2 to WIFI, 1:SD2 to SD1 8bit */
		if (board_is_rev(BOARD_REV_2))
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,
					      7, 0);
		mxc_request_iomux(MX35_PIN_SD2_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA0,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA1,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA2,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD2_DATA3,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_MAX |
		    PAD_CTL_47K_PU | PAD_CTL_SRE_FAST;

		mxc_iomux_set_pad(MX35_PIN_SD2_CLK, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_CMD, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA0, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA1, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA2, pad_val);

		pad_val = PAD_CTL_PUE_PUD | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_MAX |
		    PAD_CTL_100K_PU | PAD_CTL_SRE_FAST;

		mxc_iomux_set_pad(MX35_PIN_SD2_DATA3, pad_val);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
		mxc_free_iomux(MX35_PIN_SD1_CLK,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_CMD,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA0,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA1,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA2,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA3,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_CMD,
			       MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_CLK,
			       MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA0,
			       MUX_CONFIG_ALT2 | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA1,
			       MUX_CONFIG_ALT2 | MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_SD1_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		break;
	case 1:
		mxc_free_iomux(MX35_PIN_SD2_CLK,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_CMD,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA0,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA1,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA2,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD2_DATA3,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_SD2_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD2_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
unsigned int sdhc_get_card_det_status(struct device *dev)
{
	unsigned int ret;

	if (to_platform_device(dev)->id == 0) {
		if (0 != pmic_gpio_get_designation_bit_val(2, &ret))
			printk(KERN_ERR "Get cd status error.");
		return ret;
	} else {		/* config the det pin for SDHC2 */
		return 0;
	}
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*!
 * Get pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned int rc = 0;

	if (0 != pmic_gpio_get_designation_bit_val(3, &rc))
		printk(KERN_ERR "Get wp status error.");
	return rc;
}

EXPORT_SYMBOL(sdhc_write_protect);

/*
 *  USB Host2
 */
int gpio_usbh2_active(void)
{
	if (board_is_rev(BOARD_REV_2)) {
		/* MUX3_CTR to be low for USB Host2 DP&DM */
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 6, 0);
		/* CAN_PWDN to be high for USB Host2 Power&OC */
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 1, 1);
	}

	mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, 0x0040);

	mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_ALT2);
	mxc_iomux_set_input(MUX_IN_USB_UH2_USB_OC, INPUT_CTL_PATH0);
	mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, 0x01c0);

	return 0;
}

EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_I2C2_DAT), NULL);
	mxc_free_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_I2C2_CLK), NULL);
	mxc_free_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbh2_inactive);

/*
 *  USB OTG UTMI
 */
int gpio_usbotg_utmi_active(void)
{
	mxc_request_iomux(MX35_PIN_USBOTG_PWR, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_USBOTG_PWR, 0x0040);
	mxc_request_iomux(MX35_PIN_USBOTG_OC, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_USBOTG_OC, 0x01c0);

	return 0;
}

EXPORT_SYMBOL(gpio_usbotg_utmi_active);

void gpio_usbotg_utmi_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_USBOTG_PWR), NULL);
	mxc_free_iomux(MX35_PIN_USBOTG_PWR, MUX_CONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_USBOTG_OC), NULL);
	mxc_free_iomux(MX35_PIN_USBOTG_OC, MUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbotg_utmi_inactive);

void gpio_sensor_active(void)
{
	/*CSI D6 */
	mxc_request_iomux(MX35_PIN_TX1, MUX_CONFIG_ALT6);
	/*CSI D7 */
	mxc_request_iomux(MX35_PIN_TX0, MUX_CONFIG_ALT6);
	mxc_request_iomux(MX35_PIN_CSI_D8, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D9, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D10, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D11, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D12, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D13, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D14, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_D15, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_HSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_MCLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_PIXCLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CSI_VSYNC, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_sensor_active);

void gpio_sensor_inactive(void)
{
	mxc_request_iomux(MX35_PIN_TX1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX0, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Setup GPIO for spdif tx/rx to be active
 */
void gpio_spdif_active(void)
{
	/* SPDIF OUT */
	mxc_request_iomux(MX35_PIN_STXD5, MUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX35_PIN_STXD5, PAD_CTL_PKE_NONE | PAD_CTL_PUE_PUD);
	/* SPDIF IN */
	mxc_request_iomux(MX35_PIN_SRXD5, MUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX35_PIN_SRXD5, PAD_CTL_PKE_ENABLE
			  | PAD_CTL_100K_PU | PAD_CTL_HYS_SCHMITZ);
	/* SPDIF ext clock */
	mxc_request_iomux(MX35_PIN_SCK5, MUX_CONFIG_ALT1);
	if (board_is_rev(BOARD_REV_2))
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 5, 1);
	else
		pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_2, 0, 1);
}

EXPORT_SYMBOL(gpio_spdif_active);

/*!
 * Setup GPIO for spdif tx/rx to be inactive
 */
void gpio_spdif_inactive(void)
{
	/* SPDIF OUT */
	mxc_free_iomux(MX35_PIN_STXD5, MUX_CONFIG_ALT1);
	/* SPDIF IN */
	mxc_free_iomux(MX35_PIN_SRXD5, MUX_CONFIG_ALT1);
	/* SPDIF ext clock */
	mxc_free_iomux(MX35_PIN_SCK5, MUX_CONFIG_ALT1);
}

EXPORT_SYMBOL(gpio_spdif_inactive);

/*!
 * This function activates DAM ports 3 to enable
 * audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	unsigned int pad_val;

	mxc_request_iomux(MX35_PIN_STXD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SRXD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCK4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_STXFS4, MUX_CONFIG_FUNC);

	pad_val = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;
	mxc_iomux_set_pad(MX35_PIN_STXD4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SRXD4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SCK4, pad_val);
	mxc_iomux_set_pad(MX35_PIN_STXFS4, pad_val);
}

EXPORT_SYMBOL(gpio_activate_audio_ports);

/*!
 * This function deactivates DAM ports 3 to disable
 * audio I/O.
 */
void gpio_inactivate_audio_ports(void)
{
	mxc_free_iomux(MX35_PIN_STXD4, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SRXD4, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SCK4, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_STXFS4, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_inactivate_audio_ports);

/*!
 * This function activates DAM ports 5 to enable
 * audio I/O.
 */
void gpio_activate_bt_audio_port(void)
{
	unsigned int pad_val;

	mxc_request_iomux(MX35_PIN_STXD5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SRXD5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCK5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_STXFS5, MUX_CONFIG_FUNC);

	pad_val = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;
	mxc_iomux_set_pad(MX35_PIN_STXD5, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SRXD5, pad_val);
	mxc_iomux_set_pad(MX35_PIN_SCK5, pad_val);
	mxc_iomux_set_pad(MX35_PIN_STXFS5, pad_val);
	if (board_is_rev(BOARD_REV_2))
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 5, 0);
	else
		pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_2, 0, 0);
}

EXPORT_SYMBOL(gpio_activate_bt_audio_port);

/*!
 * Setup GPIO for bluetooth audio to be inactive
 */
void gpio_inactivate_bt_audio_port(void)
{
	mxc_free_iomux(MX35_PIN_STXD5, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SRXD5, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SCK5, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_STXFS5, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_inactivate_bt_audio_port);

/*!
 * Setup GPIO for ATA interface
 *
 */
void gpio_ata_active(void)
{
	unsigned int ata_ctl_pad_cfg, ata_dat_pad_cfg;

	/* HDD_ENBALE */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 3, 0);
	/* Power On the HDD */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 1);
	msleep(300);

	/*IOMUX Settings */
	/*PATA_DIOR */
	mxc_request_iomux(MX35_PIN_ATA_DIOR, MUX_CONFIG_FUNC);
	/*PATA_DIOW */
	mxc_request_iomux(MX35_PIN_ATA_DIOW, MUX_CONFIG_FUNC);
	/*PATA_DMARQ_B */
	mxc_request_iomux(MX35_PIN_ATA_DMARQ, MUX_CONFIG_FUNC);
	/*PATA_DMACK */
	mxc_request_iomux(MX35_PIN_ATA_DMACK, MUX_CONFIG_FUNC);
	/*PATA_RESET_B */
	mxc_request_iomux(MX35_PIN_ATA_RESET_B, MUX_CONFIG_FUNC);
	/*PATA_IORDY */
	mxc_request_iomux(MX35_PIN_ATA_IORDY, MUX_CONFIG_FUNC);
	/*PATA_INTRQ_B */
	mxc_request_iomux(MX35_PIN_ATA_INTRQ, MUX_CONFIG_FUNC);
	/*PATA_CS_0 */
	mxc_request_iomux(MX35_PIN_ATA_CS0, MUX_CONFIG_FUNC);
	/*PATA_CS_1 */
	mxc_request_iomux(MX35_PIN_ATA_CS1, MUX_CONFIG_FUNC);
	/*PATA_DA0 */
	mxc_request_iomux(MX35_PIN_ATA_DA0, MUX_CONFIG_FUNC);
	/*PATA_DA1 */
	mxc_request_iomux(MX35_PIN_ATA_DA1, MUX_CONFIG_FUNC);
	/*PATA_DA2 */
	mxc_request_iomux(MX35_PIN_ATA_DA2, MUX_CONFIG_FUNC);
	/* BUFFER_ENABLE - HDD_ENABLE_B  */
	mxc_request_iomux(MX35_PIN_ATA_BUFF_EN, MUX_CONFIG_FUNC);

	/*PATA_D0 */
	mxc_request_iomux(MX35_PIN_ATA_DATA0, MUX_CONFIG_FUNC);
	/*PATA_D1 */
	mxc_request_iomux(MX35_PIN_ATA_DATA1, MUX_CONFIG_FUNC);
	/*PATA_D2 */
	mxc_request_iomux(MX35_PIN_ATA_DATA2, MUX_CONFIG_FUNC);
	/*PATA_D3 */
	mxc_request_iomux(MX35_PIN_ATA_DATA3, MUX_CONFIG_FUNC);
	/*PATA_D4 */
	mxc_request_iomux(MX35_PIN_ATA_DATA4, MUX_CONFIG_FUNC);
	/*PATA_D5 */
	mxc_request_iomux(MX35_PIN_ATA_DATA5, MUX_CONFIG_FUNC);
	/*PATA_D6 */
	mxc_request_iomux(MX35_PIN_ATA_DATA6, MUX_CONFIG_FUNC);
	/*PATA_D7 */
	mxc_request_iomux(MX35_PIN_ATA_DATA7, MUX_CONFIG_FUNC);
	/*PATA_D8 */
	mxc_request_iomux(MX35_PIN_ATA_DATA8, MUX_CONFIG_FUNC);
	/*PATA_D9 */
	mxc_request_iomux(MX35_PIN_ATA_DATA9, MUX_CONFIG_FUNC);
	/*PATA_D10 */
	mxc_request_iomux(MX35_PIN_ATA_DATA10, MUX_CONFIG_FUNC);
	/*PATA_D11 */
	mxc_request_iomux(MX35_PIN_ATA_DATA11, MUX_CONFIG_FUNC);
	/*PATA_D12 */
	mxc_request_iomux(MX35_PIN_ATA_DATA12, MUX_CONFIG_FUNC);
	/*PATA_D13 */
	mxc_request_iomux(MX35_PIN_ATA_DATA13, MUX_CONFIG_FUNC);
	/*PATA_D14 */
	mxc_request_iomux(MX35_PIN_ATA_DATA14, MUX_CONFIG_FUNC);
	/*PATA_D15 */
	mxc_request_iomux(MX35_PIN_ATA_DATA15, MUX_CONFIG_FUNC);

	/* IOMUX Pad Settings */
	ata_ctl_pad_cfg = PAD_CTL_SRE_SLOW | PAD_CTL_DRV_NORMAL |
	    PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE |
	    PAD_CTL_PUE_PUD | PAD_CTL_100K_PD |
	    PAD_CTL_HYS_CMOS | PAD_CTL_DRV_3_3V;
	ata_dat_pad_cfg = PAD_CTL_SRE_FAST | PAD_CTL_DRV_MAX |
	    PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE |
	    PAD_CTL_PUE_PUD | PAD_CTL_100K_PD |
	    PAD_CTL_HYS_SCHMITZ | PAD_CTL_DRV_3_3V;

	mxc_iomux_set_pad(MX35_PIN_ATA_DMARQ, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DIOR, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DIOW, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DMACK, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_RESET_B, PAD_CTL_SRE_SLOW |
			  PAD_CTL_DRV_NORMAL | PAD_CTL_ODE_CMOS |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
			  PAD_CTL_100K_PU | PAD_CTL_HYS_CMOS |
			  PAD_CTL_DRV_3_3V);
	mxc_iomux_set_pad(MX35_PIN_ATA_IORDY, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_INTRQ, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_CS0, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_CS1, ata_ctl_pad_cfg);

	mxc_iomux_set_pad(MX35_PIN_ATA_DATA0, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA1, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA2, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA3, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA4, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA5, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA6, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA7, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA8, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA9, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA10, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA11, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA12, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA13, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA14, ata_dat_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DATA15, ata_dat_pad_cfg);

	mxc_iomux_set_pad(MX35_PIN_ATA_DA0, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DA1, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_DA2, ata_ctl_pad_cfg);
	mxc_iomux_set_pad(MX35_PIN_ATA_BUFF_EN, ata_ctl_pad_cfg);
}

EXPORT_SYMBOL(gpio_ata_active);

/*!
 * Restore ATA interface pins to reset values
 *
 */
void gpio_ata_inactive(void)
{
	/*Turn off the IOMUX for ATA group B signals */
	mxc_free_iomux(MX35_PIN_ATA_DATA0, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA1, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA2, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA3, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA4, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA5, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA6, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA7, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA8, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA9, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA10, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA11, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA12, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA13, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA14, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DATA15, MUX_CONFIG_FUNC);

	/* Config the multiplex pin of ATA interface DIR, DA0-2, INTRQ, DMARQ */
	mxc_free_iomux(MX35_PIN_ATA_DMARQ, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DIOR, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DIOW, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DMACK, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_RESET_B, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_IORDY, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_INTRQ, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_CS0, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_CS1, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DA0, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DA1, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_DA2, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_ATA_BUFF_EN, MUX_CONFIG_FUNC);

	/* Power Off the HDD */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 4, 0);
	/* HDD_ENBALE */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 3, 1);
}

EXPORT_SYMBOL(gpio_ata_inactive);

/*!
 * This function activates ESAI ports to enable
 * surround sound I/O
 */
void gpio_activate_esai_ports(void)
{
	unsigned int pad_val;
	/* ESAI TX - WM8580 */
	mxc_request_iomux(MX35_PIN_HCKT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCKT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FST, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_FUNC);

	/* ESAI RX - AK5702 */
	/*mxc_request_iomux(MX35_PIN_HCKR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_SCKR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_FSR, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_TX4_RX1, MUX_CONFIG_FUNC);*/

	pad_val = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;
	/* ESAI TX - WM8580 */
	mxc_iomux_set_pad(MX35_PIN_SCKT, pad_val);
	mxc_iomux_set_pad(MX35_PIN_FST, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX0, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX1, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX2_RX3, pad_val);

	/* ESAI RX - AK5702 */
	/*mxc_iomux_set_pad(MX35_PIN_SCKR, pad_val);
	mxc_iomux_set_pad(MX35_PIN_FSR, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX3_RX2, pad_val);
	mxc_iomux_set_pad(MX35_PIN_TX4_RX1, pad_val);*/

	pad_val =
	    PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
	    PAD_CTL_PUE_PUD;

	/* ESAI TX - WM8580 */
	mxc_iomux_set_pad(MX35_PIN_HCKT, pad_val);
	/* ESAI RX - AK5702 */
	/*mxc_iomux_set_pad(MX35_PIN_HCKR, pad_val);*/
}

EXPORT_SYMBOL(gpio_activate_esai_ports);

/*!
 * This function deactivates ESAI ports to disable
 * surround sound I/O
 */
void gpio_deactivate_esai_ports(void)
{

	mxc_free_iomux(MX35_PIN_HCKT, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_SCKT, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_FST, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX0, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX1, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_FUNC);
	/*mxc_free_iomux(MX35_PIN_HCKR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_SCKR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_FSR, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX35_PIN_TX4_RX1, MUX_CONFIG_GPIO);*/
}

EXPORT_SYMBOL(gpio_deactivate_esai_ports);

/*!
 * This function enable and reset GPS GPIO
 */
void gpio_gps_active(void)
{
	/* Pull GPIO1_5 to be low for routing signal to UART3/GPS */
	if (board_is_rev(BOARD_REV_2)) {
		mxc_request_iomux(MX35_PIN_COMPARE, MUX_CONFIG_GPIO);
		mxc_iomux_set_pad(MX35_PIN_COMPARE, PAD_CTL_DRV_NORMAL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU |
				PAD_CTL_DRV_3_3V | PAD_CTL_PUE_PUD |
				PAD_CTL_SRE_SLOW);
		gpio_direction_output(IOMUX_TO_GPIO(MX35_PIN_COMPARE), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX35_PIN_COMPARE), 0);
	}

	/* PWR_EN_GPS is set to be 0, will be toggled on in app by ioctl */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 0);

	/* GPS 32KHz clock enbale */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 7, 1);

	/* GPS reset */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 5, 0);
	msleep(5);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 5, 1);
	msleep(5);
}

EXPORT_SYMBOL(gpio_gps_active);

/*!
 * This function get GPS GPIO status.
 */
int gpio_gps_access(int para)
{
	unsigned int gps_val;

	if (para & 0x4) {	/* Read GPIO */
		if (para & 0x1) /* Read PWR_EN */
			pmic_gpio_get_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0,
						&gps_val);
		else		/* Read nReset */
			pmic_gpio_get_bit_val(MCU_GPIO_REG_RESET_1, 5,
						&gps_val);
		return gps_val;
	} else {		/* Write GPIO */
		gps_val = (para & 0x2) ? 1 : 0;
		if (para & 0x1)
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0,
						gps_val);
		else
			pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 5, gps_val);
	}
	return 0;
}

EXPORT_SYMBOL(gpio_gps_access);

/*!
 * This function disable GPS GPIO
 */
void gpio_gps_inactive(void)
{
	/* GPS disable */
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 0, 0);
	/* Free GPIO1_5 */
	if (board_is_rev(BOARD_REV_2))
		mxc_free_iomux(MX35_PIN_COMPARE, MUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_gps_inactive);

/*!
 * The MLB gpio configuration routine
 */
void gpio_mlb_active(void)
{
	mxc_request_iomux(MX35_PIN_MLB_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_MLB_SIG, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_MLB_DAT, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_mlb_active);

void gpio_mlb_inactive(void)
{
	mxc_free_iomux(MX35_PIN_MLB_CLK, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_MLB_SIG, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX35_PIN_MLB_DAT, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_mlb_inactive);

void gpio_can_active(int id)
{
	int pad;

	switch (id) {
	case 0:
		pad = PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | \
		    PAD_CTL_PUE_PUD | PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH;
		mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_ALT1);
		mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, pad);
		mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, pad);
		mxc_iomux_set_input(MUX_IN_CAN1_CANRX, INPUT_CTL_PATH0);
		break;
	case 1:
		pad = PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_100K_PU;
		mxc_request_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_ALT1);
		mxc_request_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX35_PIN_FEC_MDC, pad);
		mxc_iomux_set_pad(MX35_PIN_FEC_MDIO, pad);
		mxc_iomux_set_input(MUX_IN_CAN2_CANRX, INPUT_CTL_PATH2);
		break;
	default:
		printk(KERN_ERR "NO such device\n");
	}
}

void gpio_can_inactive(int id)
{
	switch (id) {
	case 0:
		mxc_free_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_ALT1);
		mxc_free_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_ALT1);
		mxc_iomux_set_input(MUX_IN_CAN1_CANRX, INPUT_CTL_PATH0);
		break;
	case 1:
		mxc_free_iomux(MX35_PIN_FEC_MDC, MUX_CONFIG_ALT1);
		mxc_free_iomux(MX35_PIN_FEC_MDIO, MUX_CONFIG_ALT1);
		mxc_iomux_set_input(MUX_IN_CAN2_CANRX, INPUT_CTL_PATH0);
		break;
	default:
		printk(KERN_ERR "NO such device\n");
	}
}

void gpio_pmic_active(void)
{
	unsigned int pad_val = PAD_CTL_SRE_SLOW | PAD_CTL_DRV_NORMAL
		| PAD_CTL_HYS_CMOS | PAD_CTL_100K_PU | PAD_CTL_DRV_3_3V;
	mxc_request_iomux(MX35_PIN_GPIO2_0, MUX_CONFIG_FUNC);
	mxc_iomux_set_pad(MX35_PIN_GPIO2_0, pad_val);
	gpio_request(IOMUX_TO_GPIO(MX35_PIN_GPIO2_0), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(MX35_PIN_GPIO2_0));
}

EXPORT_SYMBOL(gpio_pmic_active);
