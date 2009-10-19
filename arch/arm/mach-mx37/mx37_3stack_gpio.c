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

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/gpio.h>
#include <mach/irqs.h>

#include "iomux.h"

/*!
 * @file mx37_3stack_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */

void gpio_activate_audio_ports(void);

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
	 * and enable the UART transceivers
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		mxc_request_iomux(MX37_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_RXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART1_UART_RXD_MUX, INPUT_CTL_PATH4);
		mxc_request_iomux(MX37_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_TXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_request_iomux(MX37_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_RTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(MUX_IN_UART1_UART_RTS_B, INPUT_CTL_PATH4);
		mxc_request_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_CTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		break;
	case 1:
		mxc_request_iomux(MX37_PIN_UART1_DCD, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_DCD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART2_UART_RXD_MUX, INPUT_CTL_PATH1);
		mxc_request_iomux(MX37_PIN_UART1_RI, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_RI, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_request_iomux(MX37_PIN_UART1_DSR, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_DSR, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(MUX_IN_UART2_UART_RTS_B, INPUT_CTL_PATH1);
		mxc_request_iomux(MX37_PIN_UART1_DTR, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_DTR, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		break;
	case 2:
		mxc_request_iomux(MX37_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_AUD3_BB_TXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH0);
		mxc_request_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_AUD3_BB_RXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_request_iomux(MX37_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_AUD3_BB_CK, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RTS_B, INPUT_CTL_PATH0);
		mxc_request_iomux(MX37_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_AUD3_BB_FS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	/*
	 * Configure the IOMUX control registers for the UART signals
	 * and disable the UART transceivers
	 */
	switch (port) {
	case 0:
		mxc_request_iomux(MX37_PIN_UART1_RXD, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_TXD, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_RTS, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_RXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_TXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_RTS, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_GPIO);
		break;
	case 1:
		mxc_request_iomux(MX37_PIN_UART1_DCD, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_RI, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_DSR, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_UART1_DTR, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_DCD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_RI, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_DSR, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_UART1_DTR, IOMUX_CONFIG_GPIO);
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		mxc_request_iomux(MX37_PIN_AUD3_BB_TXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_AUD3_BB_TXD, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_AUD3_BB_CK, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_AUD3_BB_CK, IOMUX_CONFIG_GPIO);
		mxc_request_iomux(MX37_PIN_AUD3_BB_FS, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX37_PIN_AUD3_BB_FS, IOMUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{

}

EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);
EXPORT_SYMBOL(config_uartdma_event);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		break;
	case 1:
		/* SPI2 */
		mxc_request_iomux(MX37_PIN_CSPI2_MISO, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_MISO, PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_GRP_H9, PAD_CTL_HYS_ENABLE);

		mxc_request_iomux(MX37_PIN_CSPI2_MOSI, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_MOSI, PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_CTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE);

		mxc_request_iomux(MX37_PIN_CSPI2_SCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_SCLK, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX37_PIN_CSPI2_SS1, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_SS1, PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX37_PIN_CSPI2_SS0, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_SS0, PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_GRP_H10, PAD_CTL_HYS_ENABLE);
		break;
	case 2:
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_active(void)
{
 /*TODO*/}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_owire_active);
EXPORT_SYMBOL(gpio_owire_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	iomux_pad_config_t regval = 0;

	switch (i2c_num) {
	case 0:
		/* Touch */
		/* select I2C1_SCK as daisy chain input */
		mxc_request_iomux(MX37_PIN_I2C1_CLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_input(MUX_IN_I2C1_SCL, INPUT_CTL_PATH1);
		/* OpenDrain enabled, 100k PU enabled */
		regval =
		    PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_100K_PU |
		    PAD_CTL_PKE_ENABLE;
		mxc_iomux_set_pad(MX37_PIN_I2C1_CLK, regval);

		/*select I2C1_SDA as daisy chain input */
		mxc_request_iomux(MX37_PIN_I2C1_DAT, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_input(MUX_IN_I2C1_SDA, INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX37_PIN_I2C1_DAT, regval);
		mxc_iomux_set_pad(MX37_PIN_GRP_H3, PAD_CTL_HYS_ENABLE);
		break;
	case 1:
		/* PMIC */
		/*select I2C2_SCL as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C2_SCL, INPUT_CTL_PATH1);
		regval = PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_PUE_PULL | PAD_CTL_100K_PU |
		    PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_DRV_HIGH;
		mxc_iomux_set_pad(MX37_PIN_GPIO1_0, regval);
		mxc_request_iomux(MX37_PIN_GPIO1_0,
				  (IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT2));

		/*select I2C2_SDA as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C2_SDA, INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX37_PIN_GPIO1_1, regval);
		mxc_request_iomux(MX37_PIN_GPIO1_1,
				  (IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT2));
		break;
	case 2:
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
 /*TODO*/}

/*!
 * This function activates DAM ports 4 & 5 to enable
 * audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	unsigned int pad_val;

	/* AUD4_TXD */
	mxc_request_iomux(MX37_PIN_DISP1_DAT20, IOMUX_CONFIG_ALT5);
	/* AUD4_RXD */
	mxc_request_iomux(MX37_PIN_DISP1_DAT21, IOMUX_CONFIG_ALT5);
	/* AUD4_TXC */
	mxc_request_iomux(MX37_PIN_DISP1_DAT22, IOMUX_CONFIG_ALT5);
	/* AUD4_TXFS */
	mxc_request_iomux(MX37_PIN_DISP1_DAT23, IOMUX_CONFIG_ALT5);

	pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	    PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST;
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_CK, PAD_CTL_100K_PU | pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_CK, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_RXD, pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_TXD, pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_FS, PAD_CTL_100K_PU | pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_FS, IOMUX_CONFIG_ALT0);

	/* Enable hysteresis for AUD5_WB_CK, AUD5_WB_RXD, AUD5_WB_TXD, AUD5_WB_FS */
	mxc_iomux_set_pad(MX37_PIN_GRP_H5, PAD_CTL_HYS_ENABLE);
}

EXPORT_SYMBOL(gpio_activate_audio_ports);

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	switch (module) {
	case 0:
		mxc_request_iomux(MX37_PIN_SD1_CLK,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD1_CMD,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD1_DATA0,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD1_DATA1,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD1_DATA2,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD1_DATA3,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX37_PIN_SD1_CMD,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD1_CLK,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA0,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA1,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA2,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA3,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);

		/* Write Protected Pin */
		mxc_request_iomux(MX37_PIN_CSPI1_SS0,
				  IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT4);
		mxc_iomux_set_pad(MX37_PIN_CSPI1_SS0,
				  PAD_CTL_DRV_HIGH | PAD_CTL_HYS_NONE |
				  PAD_CTL_SRE_FAST);
		/*
		 * SW workaround for the eSDHC1 Write Protected feature
		 * The PSR of CSPI1_SS0 (GPIO3_2) should be read.
		 */
		gpio_request(IOMUX_TO_GPIO(MX37_PIN_CSPI1_SS0), "cspi1_ss0");
		gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_CSPI1_SS0), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_CSPI1_SS0), 1);
		break;
	case 1:
		mxc_request_iomux(MX37_PIN_SD2_CLK,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD2_CMD,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD2_DATA0,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD2_DATA1,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD2_DATA2,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX37_PIN_SD2_DATA3,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX37_PIN_SD2_CMD,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD2_CLK,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA0,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA1,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA2,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA3,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_PUE_PULL |
				  PAD_CTL_47K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
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
		mxc_free_iomux(MX37_PIN_SD1_CLK,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD1_CMD,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD1_DATA0,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD1_DATA1,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD1_DATA2,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD1_DATA3,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX37_PIN_SD1_CLK,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD1_CMD,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));

		/* Free Write Protected Pin */
		mxc_free_iomux(MX37_PIN_CSPI1_SS0,
			       IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT4);
		mxc_iomux_set_pad(MX37_PIN_CSPI1_SS0,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		break;
	case 1:
		mxc_free_iomux(MX37_PIN_SD2_CLK,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD2_CMD,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD2_DATA0,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD2_DATA1,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD2_DATA2,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX37_PIN_SD2_DATA3,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX37_PIN_SD2_CLK,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD2_CMD,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA0,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA1,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA2,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX37_PIN_SD2_DATA3,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	u32 gpio = IOMUX_TO_GPIO(MX37_PIN_OWIRE_LINE);

	if (to_platform_device(dev)->id == 0) {
		if (board_is_rev(BOARD_REV_2))
			ret = gpio_get_value(IOMUX_TO_GPIO(MX37_PIN_GPIO1_4));
		else
			ret = gpio_get_value(gpio);
		return ret;
	} else {		/* config the det pin for SDHC2 */
		return 0;
	}
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*
 * Return the card detect pin.
 */
int sdhc_init_card_det(int id)
{
	u32 gpio;

	if (id == 0) {
		if (board_is_rev(BOARD_REV_2)) {
			gpio = IOMUX_TO_GPIO(MX37_PIN_GPIO1_4);
			mxc_request_iomux(MX37_PIN_GPIO1_4, IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX37_PIN_GPIO1_4,
					  PAD_CTL_DRV_HIGH |
					  PAD_CTL_HYS_NONE |
					  PAD_CTL_ODE_OPENDRAIN_NONE |
					  PAD_CTL_PKE_NONE | PAD_CTL_SRE_FAST);
			gpio_request(gpio, "gpio1_4");
			gpio_direction_input(gpio);
			return IOMUX_TO_IRQ(MX37_PIN_GPIO1_4);
		} else {
			gpio = IOMUX_TO_GPIO(MX37_PIN_OWIRE_LINE);
			mxc_request_iomux(MX37_PIN_OWIRE_LINE,
					  IOMUX_CONFIG_ALT4);
			mxc_iomux_set_pad(MX37_PIN_OWIRE_LINE,
					  PAD_CTL_DRV_HIGH |
					  PAD_CTL_HYS_NONE |
					  PAD_CTL_ODE_OPENDRAIN_NONE |
					  PAD_CTL_PKE_NONE | PAD_CTL_SRE_FAST);
			gpio_request(gpio, "owire_line");
			gpio_direction_input(gpio);
			return IOMUX_TO_IRQ(MX37_PIN_OWIRE_LINE);
		}
	} else {		/* config the det pin for SDHC2 */
		return 0;

	}
}

EXPORT_SYMBOL(sdhc_init_card_det);

/*!
 * Get CSPI1_SS0 pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	rc = gpio_get_value(IOMUX_TO_GPIO(MX37_PIN_CSPI1_SS0));
	if (rc > 0)
		return 1;
	else
		return 0;
}

EXPORT_SYMBOL(sdhc_write_protect);

/*!
 * Setup GPIO for LCD to be active
 *
 */
void gpio_lcd_active(void)
{
	mxc_request_iomux(MX37_PIN_DI1_PIN2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_PAD_DI1_PIN3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_DISP_CLK, IOMUX_CONFIG_ALT0);
}

/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
 /*TODO*/}

/*!
 * Setup pins for SLCD to be active
 *
 */
void slcd_gpio_config(void)
{
 /*TODO*/}

/*!
 * Switch to the specified sensor - MX33 ADS has two
 *
 */
void gpio_sensor_select(int sensor)
{
 /*TODO*/}

/*!
 * Setup GPIO for sensor to be active
 *
 */
void gpio_sensor_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_sensor_active);

/*!
 * Setup GPIO for sensor to be inactive
 *
 */
void gpio_sensor_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Setup GPIO for ATA interface
 *
 */
void gpio_ata_active(void)
{
	/*IOMUX Settings */
	/*PATA_DMARQ_B */
	mxc_request_iomux(MX37_PIN_NANDF_RE_B, IOMUX_CONFIG_ALT1);
	/*PATA_DIOR */
	mxc_request_iomux(MX37_PIN_NANDF_ALE, IOMUX_CONFIG_ALT1);
	/*PATA_DIOW */
	mxc_request_iomux(MX37_PIN_NANDF_CLE, IOMUX_CONFIG_ALT1);
	/*PATA_DMACK */
	mxc_request_iomux(MX37_PIN_NANDF_WP_B, IOMUX_CONFIG_ALT1);
	/*PATA_RESET_B */
	mxc_request_iomux(MX37_PIN_NANDF_RB, IOMUX_CONFIG_ALT1);
	/*PATA_IORDY */
	mxc_request_iomux(MX37_PIN_NANDF_CS0, IOMUX_CONFIG_ALT1);
	/*PATA_INTRQ_B */
	mxc_request_iomux(MX37_PIN_NANDF_CS1, IOMUX_CONFIG_ALT1);
	/*PATA_CS_0 */
	mxc_request_iomux(MX37_PIN_NANDF_CS2, IOMUX_CONFIG_ALT1);
	/*PATA_CS_1 */
	mxc_request_iomux(MX37_PIN_NANDF_CS3, IOMUX_CONFIG_ALT1);

	/*PATA_D0 */
	mxc_request_iomux(MX37_PIN_EIM_D0, IOMUX_CONFIG_ALT1);
	/*PATA_D1 */
	mxc_request_iomux(MX37_PIN_EIM_D1, IOMUX_CONFIG_ALT1);
	/*PATA_D2 */
	mxc_request_iomux(MX37_PIN_EIM_D2, IOMUX_CONFIG_ALT1);
	/*PATA_D3 */
	mxc_request_iomux(MX37_PIN_EIM_D3, IOMUX_CONFIG_ALT1);
	/*PATA_D4 */
	mxc_request_iomux(MX37_PIN_EIM_D4, IOMUX_CONFIG_ALT1);
	/*PATA_D5 */
	mxc_request_iomux(MX37_PIN_EIM_D5, IOMUX_CONFIG_ALT1);
	/*PATA_D6 */
	mxc_request_iomux(MX37_PIN_EIM_D6, IOMUX_CONFIG_ALT1);
	/*PATA_D7 */
	mxc_request_iomux(MX37_PIN_EIM_D7, IOMUX_CONFIG_ALT1);
	/*PATA_D8 */
	mxc_request_iomux(MX37_PIN_EIM_D8, IOMUX_CONFIG_ALT1);
	/*PATA_D9 */
	mxc_request_iomux(MX37_PIN_EIM_D9, IOMUX_CONFIG_ALT1);
	/*PATA_D10 */
	mxc_request_iomux(MX37_PIN_EIM_D10, IOMUX_CONFIG_ALT1);
	/*PATA_D11 */
	mxc_request_iomux(MX37_PIN_EIM_D11, IOMUX_CONFIG_ALT1);
	/*PATA_D12 */
	mxc_request_iomux(MX37_PIN_EIM_D12, IOMUX_CONFIG_ALT1);
	/*PATA_D13 */
	mxc_request_iomux(MX37_PIN_EIM_D13, IOMUX_CONFIG_ALT1);
	/*PATA_D14 */
	mxc_request_iomux(MX37_PIN_EIM_D14, IOMUX_CONFIG_ALT1);
	/*PATA_D15 */
	mxc_request_iomux(MX37_PIN_EIM_D15, IOMUX_CONFIG_ALT1);
	/*PATA_DA0 */
	mxc_request_iomux(MX37_PIN_SD2_DATA3, IOMUX_CONFIG_ALT1);
	/*PATA_DA1 */
	mxc_request_iomux(MX37_PIN_SD2_DATA2, IOMUX_CONFIG_ALT1);
	/*PATA_DA2 */
	mxc_request_iomux(MX37_PIN_SD2_DATA1, IOMUX_CONFIG_ALT1);

	/* BUFFER_ENABLE - HDD_ENABLE_B  */
	mxc_request_iomux(MX37_PIN_NANDF_WE_B, IOMUX_CONFIG_ALT1);

	/* IOMUX Pad Settings */
	mxc_iomux_set_pad(MX37_PIN_EIM_D0, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D1, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D2, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D3, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D4, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D5, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D6, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D7, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D8, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D9, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D10, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D11, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D12, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D13, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D14, 0xc0);
	mxc_iomux_set_pad(MX37_PIN_EIM_D15, 0xc0);

	mxc_iomux_set_pad(MX37_PIN_NANDF_RE_B, 0x0080);
	mxc_iomux_set_pad(MX37_PIN_NANDF_CS1, 0x0020);
	mxc_iomux_set_pad(MX37_PIN_SD2_DATA3, 0x00);
	mxc_iomux_set_pad(MX37_PIN_SD2_DATA2, 0x00);
	mxc_iomux_set_pad(MX37_PIN_SD2_DATA1, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_ALE, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_CS2, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_CS3, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_WE_B, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_CLE, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_WP_B, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_RB, 0x00);
	mxc_iomux_set_pad(MX37_PIN_NANDF_CS0, 0x0020);

}

EXPORT_SYMBOL(gpio_ata_active);

/*!
 * Restore ATA interface pins to reset values
 *
 */
void gpio_ata_inactive(void)
{
	/*Turn off the IOMUX for ATA group B signals */
	mxc_request_iomux(MX37_PIN_EIM_D0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D6, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D7, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D8, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D9, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D10, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D11, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D12, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D13, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D14, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_EIM_D15, IOMUX_CONFIG_ALT0);

	/* Config the multiplex pin of ATA interface DIR, DA0-2, INTRQ, DMARQ */
	mxc_request_iomux(MX37_PIN_NANDF_RE_B, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_CS1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_SD2_DATA3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_SD2_DATA2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_SD2_DATA1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_ALE, IOMUX_CONFIG_ALT0);

	/* HDD_BUFF_EN (H:A->B, L:B->A) and HDD_ENABLE_B(H:Disable,L:Enable) */
	mxc_free_iomux(MX37_PIN_NANDF_WE_B, IOMUX_CONFIG_ALT0);

	/* These ATA pins are common to Group A and Group B */
	mxc_request_iomux(MX37_PIN_NANDF_CS2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_CS3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_ALE, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_CLE, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_WP_B, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_RB, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX37_PIN_NANDF_CS0, IOMUX_CONFIG_ALT0);

}

EXPORT_SYMBOL(gpio_ata_inactive);

/*!
 * Setup GPIO for Keypad  to be active
 *
 */
void gpio_keypad_active(void)
{
	int pad_val;

	/*
	 * Configure the IOMUX control register for keypad signals.
	 */
	/*KEY_INT */
	mxc_request_iomux(MX37_PIN_GPIO1_3, IOMUX_CONFIG_ALT0);
	/*KEY_WAKE */
	mxc_request_iomux(MX37_PIN_DISP1_DAT18, IOMUX_CONFIG_ALT4);

	/* fast slew rate */
	pad_val = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_22K_PU | \
	  PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_HYS_NONE | \
	  PAD_CTL_DDR_INPUT_CMOS | PAD_CTL_DRV_VOT_LOW);
	/*KEY_INT */
	mxc_iomux_set_pad(MX37_PIN_GPIO1_3, pad_val);

	/* fast slew rate */
	pad_val = (PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | \
  PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_HYS_NONE | \
  PAD_CTL_DDR_INPUT_CMOS | PAD_CTL_DRV_VOT_LOW);
	/*KEY_WAKE */
	mxc_iomux_set_pad(MX37_PIN_DISP1_DAT18, pad_val);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_DISP1_DAT18), "disp1_dat18");
	gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_DISP1_DAT18), 0);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_GPIO1_3), "gpio1_3");
	gpio_direction_input(IOMUX_TO_GPIO(MX37_PIN_GPIO1_3));

	/* drive initial value */
	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_DISP1_DAT18), 1);
}

EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Setup GPIO for Keypad to be inactive
 *
 */
void gpio_keypad_inactive(void)
{
	/*KEY_INT */
	mxc_request_iomux(MX37_PIN_GPIO1_3, IOMUX_CONFIG_ALT0);
	/*KEY_WAKE */
	mxc_request_iomux(MX37_PIN_DISP1_DAT18, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_keypad_inactive);

/*
 * USB OTG HS port
 */
int gpio_usbotg_hs_active(void)
{
	 /*TODO*/ return 0;
}

EXPORT_SYMBOL(gpio_usbotg_hs_active);

void gpio_usbotg_hs_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_usbotg_hs_inactive);

/*!
 * Setup GPIO for PCMCIA interface
 *
 */
void gpio_pcmcia_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_pcmcia_active);

/*!
 * Setup GPIO for pcmcia to be inactive
 */
void gpio_pcmcia_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_pcmcia_inactive);

/*!
 * Setup GPIO for fec to be active
 */
void gpio_fec_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_fec_active);
/*!
 * Setup GPIO for fec to be inactive
 */
void gpio_fec_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_fec_inactive);

void gpio_spdif_active(void)
{
	iomux_pad_config_t regval = 0;
	regval =
	    PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE |
	    PAD_CTL_100K_PU;
	mxc_iomux_set_pad(MX37_PIN_AUD3_BB_RXD, regval);
	mxc_request_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT1);
}

EXPORT_SYMBOL(gpio_spdif_active);

void gpio_spdif_inactive(void)
{
	mxc_free_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT1);
}

EXPORT_SYMBOL(gpio_spdif_inactive);

void gpio_pmic_active(void)
{
	mxc_request_iomux(MX37_PIN_OWIRE_LINE, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX37_PIN_OWIRE_LINE, PAD_CTL_SRE_SLOW |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_MEDIUM |
			  PAD_CTL_100K_PU |
			  PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DDR_INPUT_CMOS);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_OWIRE_LINE), "owire_line");
	gpio_direction_input(IOMUX_TO_GPIO(MX37_PIN_OWIRE_LINE));
}

EXPORT_SYMBOL(gpio_pmic_active);

void gpio_gps_active(void)
{
	/* PWR_EN */
	mxc_request_iomux(MX37_PIN_EIM_OE, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX37_PIN_EIM_OE, PAD_CTL_100K_PU |
			  PAD_CTL_DRV_HIGH | PAD_CTL_HYS_NONE |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_EIM_OE), "eim_oe");
	gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_EIM_OE), 0);

	/* RESET */
	mxc_request_iomux(MX37_PIN_EIM_BCLK, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX37_PIN_EIM_BCLK, PAD_CTL_DRV_HIGH |
			  PAD_CTL_HYS_NONE | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_EIM_BCLK), "eim_bclk");
	gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_EIM_BCLK), 0);

	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_EIM_OE), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_EIM_BCLK), 0);

	msleep(5);
	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_EIM_BCLK), 1);

	msleep(5);
}

EXPORT_SYMBOL(gpio_gps_active);

int gpio_gps_access(int para)
{
	iomux_pin_name_t pin;
	pin = (para & 0x1) ? MX37_PIN_EIM_OE : MX37_PIN_EIM_BCLK;

	if (para & 0x4)
		return gpio_get_value(IOMUX_TO_GPIO(pin));
	else if (para & 0x2)
		gpio_set_value(IOMUX_TO_GPIO(pin), 1);
	else
		gpio_set_value(IOMUX_TO_GPIO(pin), 0);

	return 0;
}

EXPORT_SYMBOL(gpio_gps_access);

void gpio_gps_inactive(void)
{
	mxc_free_iomux(MX37_PIN_EIM_BCLK, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX37_PIN_EIM_OE, IOMUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_gps_inactive);

int headphone_det_status(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXFS));
}

EXPORT_SYMBOL(headphone_det_status);
