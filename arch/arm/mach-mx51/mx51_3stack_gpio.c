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
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <mach/gpio.h>

#include "iomux.h"

/*!
 * @file mach-mx51/mx51_3stack_gpio.c
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
		mxc_request_iomux(MX51_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_RXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART1_IPP_UART_RXD_MUX_SELECT_INPUT,
				    INPUT_CTL_PATH0);
		mxc_request_iomux(MX51_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_TXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_request_iomux(MX51_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_RTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(MUX_IN_UART1_IPP_UART_RTS_B_SELECT_INPUT,
				    INPUT_CTL_PATH0);
		mxc_request_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART1_CTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		break;
		/* UART 2 IOMUX Configs */
	case 1:
		mxc_request_iomux(MX51_PIN_UART2_RXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART2_RXD, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART2_IPP_UART_RXD_MUX_SELECT_INPUT,
				    INPUT_CTL_PATH2);
		mxc_request_iomux(MX51_PIN_UART2_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART2_TXD, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		/* UART2_RTS */
		mxc_request_iomux(MX51_PIN_EIM_D26, IOMUX_CONFIG_ALT4);
		mxc_iomux_set_pad(MX51_PIN_EIM_D26, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART2_IPP_UART_RTS_B_SELECT_INPUT,
				    INPUT_CTL_PATH3);
		/* UART2_CTS */
		mxc_request_iomux(MX51_PIN_EIM_D25, IOMUX_CONFIG_ALT4);
		mxc_iomux_set_pad(MX51_PIN_EIM_D25, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		break;
	case 2:
		/* UART 3 IOMUX Configs */
		mxc_request_iomux(MX51_PIN_UART3_RXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART3_RXD, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RXD_MUX_SELECT_INPUT,
				    INPUT_CTL_PATH0);
		mxc_request_iomux(MX51_PIN_UART3_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_UART3_TXD, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		/* UART3_RTS */
		mxc_request_iomux(MX51_PIN_EIM_D27, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_EIM_D27, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RTS_B_SELECT_INPUT,
				    INPUT_CTL_PATH2);
		/* UART3_CTS */
		mxc_request_iomux(MX51_PIN_EIM_D24, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_EIM_D24, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
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
		/* UART 1 */
	case 0:
		mxc_request_gpio(MX51_PIN_UART1_RXD);
		mxc_request_gpio(MX51_PIN_UART1_TXD);
		mxc_request_gpio(MX51_PIN_UART1_RTS);
		mxc_request_gpio(MX51_PIN_UART1_CTS);

		mxc_free_iomux(MX51_PIN_UART1_RXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_UART1_TXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_UART1_RTS, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_GPIO);
		break;
		/* UART 2 */
	case 1:
		mxc_request_gpio(MX51_PIN_UART2_RXD);
		mxc_request_gpio(MX51_PIN_UART2_TXD);

		mxc_free_iomux(MX51_PIN_UART2_RXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_UART2_TXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_EIM_D26, IOMUX_CONFIG_ALT4);
		mxc_free_iomux(MX51_PIN_EIM_D25, IOMUX_CONFIG_ALT4);
		break;
	case 2:
		/* UART 3 */
		mxc_request_gpio(MX51_PIN_UART3_RXD);
		mxc_request_gpio(MX51_PIN_UART3_TXD);
		mxc_request_gpio(MX51_PIN_EIM_D27);
		mxc_request_gpio(MX51_PIN_EIM_D24);

		mxc_free_iomux(MX51_PIN_UART3_RXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_UART3_TXD, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_EIM_D27, IOMUX_CONFIG_GPIO);
		mxc_free_iomux(MX51_PIN_EIM_D24, IOMUX_CONFIG_GPIO);
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
		mxc_request_iomux(MX51_PIN_CSPI1_MISO, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_MISO, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_CSPI1_MOSI, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_MOSI, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_RDY, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_CSPI1_SCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_SCLK, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_SS1, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
				  PAD_CTL_SRE_FAST);
		break;
	case 1:
		/* SPI2 */
		mxc_request_iomux(MX51_PIN_NANDF_RB2, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX51_PIN_NANDF_RB2, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_PKE_ENABLE);

		mxc_request_iomux(MX51_PIN_NANDF_RB3, IOMUX_CONFIG_ALT2);
		mxc_iomux_set_pad(MX51_PIN_NANDF_RB3, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_PKE_ENABLE);

		if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
			mxc_request_iomux(MX51_PIN_NANDF_RB4, IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX51_PIN_NANDF_RB4, PAD_CTL_HYS_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
					  PAD_CTL_PKE_ENABLE);

			mxc_request_iomux(MX51_PIN_NANDF_RB7, IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX51_PIN_NANDF_RB7, PAD_CTL_DRV_VOT_HIGH |
					  PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
					  PAD_CTL_PUE_KEEPER | PAD_CTL_100K_PU |
					  PAD_CTL_ODE_OPENDRAIN_NONE |
					  PAD_CTL_DRV_HIGH);
		} else {
			mxc_request_iomux(MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX51_PIN_NANDF_RDY_INT, PAD_CTL_HYS_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
					  PAD_CTL_PKE_ENABLE);

			mxc_request_iomux(MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX51_PIN_NANDF_D15, PAD_CTL_HYS_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE |
					  PAD_CTL_PUE_KEEPER);

			mxc_request_iomux(MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT2);
			mxc_iomux_set_pad(MX51_PIN_NANDF_D12, PAD_CTL_HYS_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
					  PAD_CTL_PKE_ENABLE);
		}
		break;
	case 2:
		/* SPI3 */
		mxc_request_iomux(MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_USBH1_NXT, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_USBH1_DIR, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_USBH1_CLK, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_USBH1_DATA5, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
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
		mxc_free_iomux(MX51_PIN_CSPI1_MISO, IOMUX_CONFIG_ALT0);
		mxc_free_iomux(MX51_PIN_CSPI1_MOSI, IOMUX_CONFIG_ALT0);
		mxc_free_iomux(MX51_PIN_CSPI1_RDY, IOMUX_CONFIG_ALT0);
		mxc_free_iomux(MX51_PIN_CSPI1_SCLK, IOMUX_CONFIG_ALT0);
		mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
		mxc_free_iomux(MX51_PIN_CSPI1_SS1, IOMUX_CONFIG_ALT0);
		break;
	case 1:
		/* SPI2 */
		mxc_free_iomux(MX51_PIN_NANDF_RB2, IOMUX_CONFIG_ALT2);
		mxc_free_iomux(MX51_PIN_NANDF_RB3, IOMUX_CONFIG_ALT2);
		if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
			mxc_free_iomux(MX51_PIN_NANDF_RB4, IOMUX_CONFIG_ALT2);
			mxc_free_iomux(MX51_PIN_NANDF_RB7, IOMUX_CONFIG_ALT2);
		} else {
			mxc_free_iomux(MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT2);
			mxc_free_iomux(MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT2);
			mxc_free_iomux(MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT2);
		}
		break;
	case 2:
		/* SPI3 */
		mxc_free_iomux(MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT1);
		mxc_free_iomux(MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT1);
		mxc_free_iomux(MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT1);
		mxc_free_iomux(MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT1);
		break;
	default:
		break;
	}

}

EXPORT_SYMBOL(gpio_spi_inactive);

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_active(void)
{
	mxc_request_iomux(MX51_PIN_OWIRE_LINE, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_OWIRE_LINE, PAD_CTL_HYS_ENABLE |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_ODE_OPENDRAIN_ENABLE |
			  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
}

EXPORT_SYMBOL(gpio_owire_active);

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_inactive(void)
{
	mxc_free_iomux(MX51_PIN_OWIRE_LINE, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_owire_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		/*i2c1 sda */
		mxc_request_iomux(MX51_PIN_CSPI1_MOSI,
				  IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C1_IPP_SDA_IN_SELECT_INPUT,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_MOSI,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH |
				  PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_VOT_LOW | PAD_CTL_DDR_INPUT_CMOS);

		/*i2c1 scl */
		mxc_request_iomux(MX51_PIN_CSPI1_SCLK,
				  IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION);
		mxc_iomux_set_input(MUX_IN_I2C1_IPP_SCL_IN_SELECT_INPUT,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX51_PIN_CSPI1_SCLK,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH |
				  PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_VOT_LOW | PAD_CTL_DDR_INPUT_CMOS);
		break;

	case 1:
		mxc_request_iomux(MX51_PIN_GPIO1_2,
				  IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_GPIO1_3,
				  IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);

		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SDA_IN_SELECT_INPUT,
				    INPUT_CTL_PATH3);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SCL_IN_SELECT_INPUT,
				    INPUT_CTL_PATH3);

		mxc_iomux_set_pad(MX51_PIN_GPIO1_2,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH |
				  PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_VOT_LOW | PAD_CTL_DDR_INPUT_CMOS);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_3,
				  PAD_CTL_SRE_FAST |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE |
				  PAD_CTL_DRV_HIGH |
				  PAD_CTL_100K_PU |
				  PAD_CTL_HYS_ENABLE |
				  PAD_CTL_DRV_VOT_LOW | PAD_CTL_DDR_INPUT_CMOS);
		break;
	case 2:
		break;
	default:
		break;
	}
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
		/*i2c1 sda */
		mxc_free_iomux(MX51_PIN_CSPI1_MOSI,
			       IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION);
		/*i2c1 scl */
		mxc_free_iomux(MX51_PIN_CSPI1_SCLK,
			       IOMUX_CONFIG_ALT1 | IOMUX_CONFIG_SION);
		break;
	case 1:
		mxc_free_iomux(MX51_PIN_GPIO1_2,
			       IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_GPIO1_3,
			       IOMUX_CONFIG_ALT2 | IOMUX_CONFIG_SION);
		break;
	case 2:
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_i2c_inactive);

void gpio_i2c_hs_active(void)
{
	mxc_request_iomux(MX51_PIN_I2C1_CLK,
			  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
	mxc_iomux_set_pad(MX51_PIN_I2C1_CLK, 0x1E4);

	mxc_request_iomux(MX51_PIN_I2C1_DAT,
			  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
	mxc_iomux_set_pad(MX51_PIN_I2C1_DAT, 0x1E4);
}

EXPORT_SYMBOL(gpio_i2c_hs_active);

void gpio_i2c_hs_inactive(void)
{
	mxc_free_iomux(MX51_PIN_I2C1_CLK,
		       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
	mxc_free_iomux(MX51_PIN_I2C1_DAT,
		       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
}

EXPORT_SYMBOL(gpio_i2c_hs_inactive);

void gpio_pmic_active(void)
{
	mxc_request_iomux(MX51_PIN_GPIO1_5, IOMUX_CONFIG_GPIO
			  | IOMUX_CONFIG_SION);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_5, PAD_CTL_SRE_SLOW |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_MEDIUM |
			  PAD_CTL_100K_PU |
			  PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH | PAD_CTL_DDR_INPUT_CMOS);
	mxc_set_gpio_direction(MX51_PIN_GPIO1_5, 1);
}

EXPORT_SYMBOL(gpio_pmic_active);

/*!
 * This function activates DAM port 3 to enable audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	unsigned int pad_val;

	pad_val = PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH |
	    PAD_CTL_ODE_OPENDRAIN_NONE | PAD_CTL_100K_PU |
	    PAD_CTL_HYS_NONE | PAD_CTL_DDR_INPUT_CMOS | PAD_CTL_DRV_VOT_LOW;

	/* AUD3_TXD */
	mxc_request_iomux(MX51_PIN_AUD3_BB_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_AUD3_BB_TXD, pad_val);

	/* AUD3_RXD */
	mxc_request_iomux(MX51_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_AUD3_BB_RXD, pad_val);

	/* AUD3_CLK */
	mxc_request_iomux(MX51_PIN_AUD3_BB_CK, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_AUD3_BB_CK, pad_val);

	/* AUD3_FS */
	mxc_request_iomux(MX51_PIN_AUD3_BB_FS, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_AUD3_BB_FS, pad_val);

	/* EIM_D16 */
	/* osc_en is shared by SPDIF */
	mxc_request_iomux(MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_EIM_D16,
			  PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	mxc_set_gpio_direction(MX51_PIN_EIM_D16, 0);
	mxc_set_gpio_dataout(MX51_PIN_EIM_D16, 1);

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
		mxc_request_iomux(MX51_PIN_SD1_CMD,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_CLK,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_request_iomux(MX51_PIN_SD1_DATA0,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA1,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA2,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA3,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX51_PIN_SD1_CMD,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_CLK,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA0,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA1,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA2,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA3,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PD |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		/* Write Protected Pin */
		mxc_request_iomux(MX51_PIN_GPIO1_1, IOMUX_CONFIG_ALT0 |
				  IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_1, PAD_CTL_DRV_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU |
				  PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_SRE_FAST);
		mxc_set_gpio_direction(MX51_PIN_GPIO1_1, 1);
		break;
	case 1:
		mxc_request_iomux(MX51_PIN_SD2_CMD,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD2_CLK,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_request_iomux(MX51_PIN_SD2_DATA0,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD2_DATA1,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD2_DATA2,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD2_DATA3,
				  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX51_PIN_SD2_CMD,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD2_CLK,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA0,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA1,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA2,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				  PAD_CTL_PUE_PULL |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA3,
				  PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PD |
				  PAD_CTL_PUE_PULL |
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
		mxc_free_iomux(MX51_PIN_SD1_CMD,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD1_CLK,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD1_DATA0,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD1_DATA1,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD1_DATA2,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD1_DATA3,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX51_PIN_SD1_CLK,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD1_CMD,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));

		/* Free Write Protected Pin */
		mxc_free_iomux(MX51_PIN_GPIO1_1, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_1,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		break;
	case 1:
		mxc_free_iomux(MX51_PIN_SD2_CMD,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD2_CLK,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD2_DATA0,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD2_DATA1,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD2_DATA2,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_free_iomux(MX51_PIN_SD2_DATA3,
			       IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_iomux_set_pad(MX51_PIN_SD2_CLK,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD2_CMD,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA0,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA1,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA2,
				  (PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX51_PIN_SD2_DATA3,
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

	if (to_platform_device(dev)->id == 0) {
		ret = mxc_get_gpio_datain(MX51_PIN_GPIO1_0);
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
	if (id == 0) {
		mxc_request_iomux(MX51_PIN_GPIO1_0, IOMUX_CONFIG_ALT0 |
				  IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_0, PAD_CTL_DRV_HIGH |
				  PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_set_gpio_direction(MX51_PIN_GPIO1_0, 1);
		return IOMUX_TO_IRQ(MX51_PIN_GPIO1_0);
	} else {		/* config the det pin for SDHC2 */
		return 0;

	}
}

EXPORT_SYMBOL(sdhc_init_card_det);

/*!
 * Get WP pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = mxc_get_gpio_datain(MX51_PIN_GPIO1_1);
	else
		rc = 0;
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
	if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
#ifndef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
		mxc_request_iomux(MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_ALT4);
		mxc_set_gpio_direction(MX51_PIN_DI1_D1_CS, 0);
#endif
		mxc_request_iomux(MX51_PIN_DI1_D0_CS, IOMUX_CONFIG_ALT1);
		mxc_request_iomux(MX51_PIN_DI1_PIN11, IOMUX_CONFIG_ALT1);
		mxc_request_iomux(MX51_PIN_DI1_PIN12, IOMUX_CONFIG_ALT1);
		mxc_request_iomux(MX51_PIN_DI1_PIN13, IOMUX_CONFIG_ALT1);
	} else {
		mxc_request_iomux(MX51_PIN_DISP2_DAT15, IOMUX_CONFIG_ALT5);
		mxc_request_iomux(MX51_PIN_DI_GP2, IOMUX_CONFIG_ALT0);
		mxc_request_iomux(MX51_PIN_DI_GP3, IOMUX_CONFIG_ALT0);
	}
#ifdef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
	mxc_request_iomux(MX51_PIN_DI1_D1_CS, IOMUX_CONFIG_ALT4);
	mxc_set_gpio_direction(MX51_PIN_DI1_D1_CS, 0);
	mxc_set_gpio_dataout(MX51_PIN_DI1_D1_CS, 1);
	mxc_request_iomux(MX51_PIN_DISPB2_SER_DIO, IOMUX_CONFIG_ALT4);
	mxc_set_gpio_direction(MX51_PIN_DISPB2_SER_DIO, 0);
	mxc_set_gpio_dataout(MX51_PIN_DISPB2_SER_DIO, 0);
#endif
}

/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
}

/*!
 * Setup pins for SLCD to be active
 *
 */
void slcd_gpio_config(void)
{
}

/*!
 * Switch to the specified sensor
 *
 */
void gpio_sensor_select(int sensor)
{
}

/*!
 * Setup GPIO for sensor to be active
 *
 * @param   csi    csi 0 or csi 1
 *
 */
void gpio_sensor_active(unsigned int csi)
{
	switch (csi) {
	case 0:
		mxc_iomux_set_pad(MX51_PIN_CSI1_D10, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D11, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D12, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D13, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D14, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D15, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D16, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D17, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D18, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_D19, PAD_CTL_HYS_NONE);
		mxc_iomux_set_pad(MX51_PIN_CSI1_PKE0, PAD_CTL_PKE_ENABLE);

		mxc_request_iomux(MX51_PIN_CSI1_VSYNC, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI1_VSYNC, PAD_CTL_HYS_NONE |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI1_HSYNC, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI1_HSYNC, PAD_CTL_HYS_NONE |
				  PAD_CTL_SRE_SLOW);

		mxc_iomux_set_pad(MX51_PIN_CSI1_PIXCLK, PAD_CTL_HYS_NONE);

		if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
			/* Camera low power */
			mxc_request_iomux(MX51_PIN_CSI1_D8, IOMUX_CONFIG_ALT3);
			mxc_iomux_set_pad(MX51_PIN_CSI1_D8, PAD_CTL_HYS_NONE |
					  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
					  PAD_CTL_SRE_SLOW);
			mxc_iomux_set_input
			    (MUX_IN_GPIO3_IPP_IND_G_IN_12_SELECT_INPUT,
			     INPUT_CTL_PATH1);
			mxc_set_gpio_direction(MX51_PIN_CSI1_D8, 0);
			mxc_set_gpio_dataout(MX51_PIN_CSI1_D8, 0);

			/* Camera reset */
			mxc_request_iomux(MX51_PIN_CSI1_D9, IOMUX_CONFIG_ALT3);
			mxc_iomux_set_pad(MX51_PIN_CSI1_D9, PAD_CTL_HYS_NONE |
					  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
					  PAD_CTL_SRE_SLOW);
			mxc_set_gpio_direction(MX51_PIN_CSI1_D9, 0);
			mxc_set_gpio_dataout(MX51_PIN_CSI1_D9, 1);
		} else {
			mxc_request_iomux(MX51_PIN_EIM_EB2, IOMUX_CONFIG_ALT1);
			mxc_iomux_set_pad(MX51_PIN_EIM_EB2, PAD_CTL_HYS_NONE |
					  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
					  PAD_CTL_100K_PD | PAD_CTL_ODE_OPENDRAIN_NONE |
					  PAD_CTL_DRV_LOW | PAD_CTL_SRE_SLOW);
			mxc_set_gpio_direction(MX51_PIN_EIM_EB2, 0);
			mxc_set_gpio_dataout(MX51_PIN_EIM_EB2, 0);
		}

		mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT5);
		mxc_iomux_set_input
		    (MUX_IN_HSC_MIPI_MIX_IPP_IND_SENS2_DATA_EN_SELECT_INPUT,
		     INPUT_CTL_PATH0);
		mxc_iomux_set_pad(MX51_PIN_EIM_A26,
				  PAD_CTL_HYS_NONE | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PULL);
		break;
	case 1:
		mxc_iomux_set_pad(MX51_PIN_CSI2_PKE0, PAD_CTL_PKE_ENABLE);

		mxc_request_iomux(MX51_PIN_CSI2_D12, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_D12, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_D13, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_D13, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_D18, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_D18, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_D19, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_D19, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_VSYNC, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_VSYNC, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_HSYNC, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_HSYNC, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_CSI2_PIXCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX51_PIN_CSI2_PIXCLK, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_GPIO1_5, IOMUX_CONFIG_ALT6);
		mxc_iomux_set_pad(MX51_PIN_GPIO1_5, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_KEEPER |
				  PAD_CTL_DRV_LOW | PAD_CTL_ODE_OPENDRAIN_NONE |
				  PAD_CTL_SRE_SLOW);

		mxc_request_iomux(MX51_PIN_DI2_PIN4, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX51_PIN_DI2_PIN4, PAD_CTL_HYS_NONE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW |
				  PAD_CTL_SRE_SLOW);
		mxc_iomux_set_input(MUX_IN_FEC_FEC_COL_SELECT_INPUT,
				    INPUT_CTL_PATH1);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sensor_active);

/*!
 * Setup GPIO for sensor to be inactive
 *
 * @param   csi    csi 0 or csi 1
 *
 */
void gpio_sensor_inactive(unsigned int csi)
{
	switch (csi) {
	case 0:
		if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
			mxc_free_iomux(MX51_PIN_CSI1_D8, IOMUX_CONFIG_GPIO);
			mxc_request_iomux(MX51_PIN_CSI1_D8, IOMUX_CONFIG_ALT0);

			mxc_free_iomux(MX51_PIN_CSI1_D9, IOMUX_CONFIG_GPIO);
			mxc_request_iomux(MX51_PIN_CSI1_D9, IOMUX_CONFIG_ALT0);
		} else {
			mxc_free_iomux(MX51_PIN_EIM_EB2, IOMUX_CONFIG_GPIO);
			mxc_request_iomux(MX51_PIN_EIM_EB2, IOMUX_CONFIG_ALT0);
		}
		mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT0);
		break;
	case 1:
		mxc_request_iomux(MX51_PIN_GPIO1_5, IOMUX_CONFIG_ALT0);
		mxc_request_iomux(MX51_PIN_DI2_PIN4, IOMUX_CONFIG_ALT0);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Setup GPIO for ATA interface
 *
 */
void gpio_ata_active(void)
{
#define ATA_PAD_CONFIG PAD_CTL_DRV_HIGH | PAD_CTL_DRV_VOT_HIGH

	/*BUFFER_EN */
	mxc_request_iomux(MX51_PIN_NANDF_ALE, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_ALE, ATA_PAD_CONFIG);

	/*PATA_CS_0 */
	mxc_request_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CS2, ATA_PAD_CONFIG);

	/*PATA_CS_1 */
	mxc_request_iomux(MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CS3, ATA_PAD_CONFIG);

	/*PATA_DA0 */
	mxc_request_iomux(MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CS4, ATA_PAD_CONFIG);

	/*PATA_DA1 */
	mxc_request_iomux(MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CS5, ATA_PAD_CONFIG);

	/*PATA_DA2 */
	mxc_request_iomux(MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CS6, ATA_PAD_CONFIG);

	/*PATA_DIOR */
	mxc_request_iomux(MX51_PIN_NANDF_RE_B, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_RE_B, ATA_PAD_CONFIG);

	/*PATA_DIOW */
	mxc_request_iomux(MX51_PIN_NANDF_WE_B, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_WE_B, ATA_PAD_CONFIG);

	/*PATA_RESET */
	mxc_request_iomux(MX51_PIN_NANDF_CLE, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_CLE, ATA_PAD_CONFIG);

	/*PATA_DMARQ */
	mxc_request_iomux(MX51_PIN_NANDF_RB0, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_RB0, ATA_PAD_CONFIG);

	/*PATA_DMACK */
	mxc_request_iomux(MX51_PIN_NANDF_WP_B, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_WP_B, ATA_PAD_CONFIG);

	/*PATA_INTRQ */
	if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
		mxc_request_iomux(MX51_PIN_GPIO_NAND, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_GPIO_NAND, ATA_PAD_CONFIG);
	} else {
		mxc_request_iomux(MX51_PIN_NANDF_RB5, IOMUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX51_PIN_NANDF_RB5, ATA_PAD_CONFIG);
	}

	/*PATA_IORDY */
	mxc_request_iomux(MX51_PIN_NANDF_RB1, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_RB1, ATA_PAD_CONFIG);

	/*PATA_D0 */
	mxc_request_iomux(MX51_PIN_NANDF_D0, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D0, ATA_PAD_CONFIG);

	/*PATA_D1 */
	mxc_request_iomux(MX51_PIN_NANDF_D1, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D1, ATA_PAD_CONFIG);

	/*PATA_D2 */
	mxc_request_iomux(MX51_PIN_NANDF_D2, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D2, ATA_PAD_CONFIG);

	/*PATA_D3 */
	mxc_request_iomux(MX51_PIN_NANDF_D3, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D3, ATA_PAD_CONFIG);

	/*PATA_D4 */
	mxc_request_iomux(MX51_PIN_NANDF_D4, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D4, ATA_PAD_CONFIG);

	/*PATA_D5 */
	mxc_request_iomux(MX51_PIN_NANDF_D5, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D5, ATA_PAD_CONFIG);

	/*PATA_D6 */
	mxc_request_iomux(MX51_PIN_NANDF_D6, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D6, ATA_PAD_CONFIG);

	/*PATA_D7 */
	mxc_request_iomux(MX51_PIN_NANDF_D7, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D7, ATA_PAD_CONFIG);

	/*PATA_D8 */
	mxc_request_iomux(MX51_PIN_NANDF_D8, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D8, ATA_PAD_CONFIG);

	/*PATA_D9 */
	mxc_request_iomux(MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D9, ATA_PAD_CONFIG);

	/*PATA_D10 */
	mxc_request_iomux(MX51_PIN_NANDF_D10, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D10, ATA_PAD_CONFIG);

	/*PATA_D11 */
	mxc_request_iomux(MX51_PIN_NANDF_D11, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D11, ATA_PAD_CONFIG);

	/*PATA_D12 */
	mxc_request_iomux(MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D12, ATA_PAD_CONFIG);

	/*PATA_D13 */
	mxc_request_iomux(MX51_PIN_NANDF_D13, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D13, ATA_PAD_CONFIG);

	/*PATA_D14 */
	mxc_request_iomux(MX51_PIN_NANDF_D14, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D14, ATA_PAD_CONFIG);

	/*PATA_D15 */
	mxc_request_iomux(MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_NANDF_D15, ATA_PAD_CONFIG);

}

EXPORT_SYMBOL(gpio_ata_active);

/*!
 * Restore ATA interface pins to reset values
 *
 */
void gpio_ata_inactive(void)
{
	/*BUFFER_EN */
	mxc_request_iomux(MX51_PIN_NANDF_ALE, IOMUX_CONFIG_ALT0);

	/*PATA_CS_0 */
	mxc_request_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT0);

	/*PATA_CS_1 */
	mxc_request_iomux(MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT0);

	/*PATA_DA0 */
	mxc_request_iomux(MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT0);

	/*PATA_DA1 */
	mxc_request_iomux(MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT0);

	/*PATA_DA2 */
	mxc_request_iomux(MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT0);

	/*PATA_DIOR */
	mxc_request_iomux(MX51_PIN_NANDF_RE_B, IOMUX_CONFIG_ALT0);

	/*PATA_DIOW */
	mxc_request_iomux(MX51_PIN_NANDF_WE_B, IOMUX_CONFIG_ALT0);

	/*PATA_DMARQ */
	mxc_request_iomux(MX51_PIN_NANDF_RB0, IOMUX_CONFIG_ALT0);

	/*PATA_DMACK */
	mxc_request_iomux(MX51_PIN_NANDF_WP_B, IOMUX_CONFIG_ALT0);

	/*PATA_INTRQ */
	if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0) {
		mxc_request_iomux(MX51_PIN_GPIO_NAND, IOMUX_CONFIG_ALT0);
	} else {
		mxc_request_iomux(MX51_PIN_NANDF_RB5, IOMUX_CONFIG_ALT0);
	}

	/*PATA_IORDY */
	mxc_request_iomux(MX51_PIN_NANDF_RB1, IOMUX_CONFIG_ALT0);

	/*PATA_D0 */
	mxc_request_iomux(MX51_PIN_NANDF_D0, IOMUX_CONFIG_ALT0);

	/*PATA_D1 */
	mxc_request_iomux(MX51_PIN_NANDF_D1, IOMUX_CONFIG_ALT0);

	/*PATA_D2 */
	mxc_request_iomux(MX51_PIN_NANDF_D2, IOMUX_CONFIG_ALT0);

	/*PATA_D3 */
	mxc_request_iomux(MX51_PIN_NANDF_D3, IOMUX_CONFIG_ALT0);

	/*PATA_D4 */
	mxc_request_iomux(MX51_PIN_NANDF_D4, IOMUX_CONFIG_ALT0);

	/*PATA_D5 */
	mxc_request_iomux(MX51_PIN_NANDF_D5, IOMUX_CONFIG_ALT0);

	/*PATA_D6 */
	mxc_request_iomux(MX51_PIN_NANDF_D6, IOMUX_CONFIG_ALT0);

	/*PATA_D7 */
	mxc_request_iomux(MX51_PIN_NANDF_D7, IOMUX_CONFIG_ALT0);

	/*PATA_D8 */
	mxc_request_iomux(MX51_PIN_NANDF_D8, IOMUX_CONFIG_ALT0);

	/*PATA_D9 */
	mxc_request_iomux(MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT0);

	/*PATA_D10 */
	mxc_request_iomux(MX51_PIN_NANDF_D10, IOMUX_CONFIG_ALT0);

	/*PATA_D11 */
	mxc_request_iomux(MX51_PIN_NANDF_D11, IOMUX_CONFIG_ALT0);

	/*PATA_D12 */
	mxc_request_iomux(MX51_PIN_NANDF_D12, IOMUX_CONFIG_ALT0);

	/*PATA_D13 */
	mxc_request_iomux(MX51_PIN_NANDF_D13, IOMUX_CONFIG_ALT0);

	/*PATA_D14 */
	mxc_request_iomux(MX51_PIN_NANDF_D14, IOMUX_CONFIG_ALT0);

	/*PATA_D15 */
	mxc_request_iomux(MX51_PIN_NANDF_D15, IOMUX_CONFIG_ALT0);

	/*PATA_RESET */
	mxc_request_iomux(MX51_PIN_NANDF_CLE, IOMUX_CONFIG_ALT0);

}

EXPORT_SYMBOL(gpio_ata_inactive);

void gpio_nand_active(void)
{
	mxc_request_iomux(MX51_PIN_NANDF_CS0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_nand_active);

void gpio_nand_inactive(void)
{
	mxc_free_iomux(MX51_PIN_NANDF_CS0, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS1, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_nand_inactive);
/*!
 * Setup GPIO for Keypad  to be active
 *
 */
void gpio_keypad_active(void)
{
	/*
	 * Configure the IOMUX control register for keypad signals.
	 */
	mxc_request_iomux(MX51_PIN_KEY_COL0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW3, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Setup GPIO for Keypad to be inactive
 *
 */
void gpio_keypad_inactive(void)
{
	mxc_request_gpio(MX51_PIN_KEY_COL0);
	mxc_request_gpio(MX51_PIN_KEY_COL1);
	mxc_request_gpio(MX51_PIN_KEY_COL2);
	mxc_request_gpio(MX51_PIN_KEY_COL3);
	mxc_request_gpio(MX51_PIN_KEY_COL4);
	mxc_request_gpio(MX51_PIN_KEY_COL5);
	mxc_request_gpio(MX51_PIN_KEY_ROW0);
	mxc_request_gpio(MX51_PIN_KEY_ROW1);
	mxc_request_gpio(MX51_PIN_KEY_ROW2);
	mxc_request_gpio(MX51_PIN_KEY_ROW3);

	mxc_free_iomux(MX51_PIN_KEY_COL0, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_COL1, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_COL2, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_COL3, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_ROW0, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_ROW1, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_ROW2, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_KEY_ROW3, IOMUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_keypad_inactive);

/*
 * USB OTG HS port
 */
int gpio_usbotg_hs_active(void)
{
	/* USB_PWR */
	mxc_request_iomux(MX51_PIN_GPIO1_8, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_8, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_NONE | PAD_CTL_HYS_ENABLE);

	/* USB_OC */
	mxc_request_iomux(MX51_PIN_GPIO1_9, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_9, PAD_CTL_SRE_SLOW |
			  PAD_CTL_DRV_LOW | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE);
	return 0;
}

EXPORT_SYMBOL(gpio_usbotg_hs_active);

void gpio_usbotg_hs_inactive(void)
{
	mxc_request_gpio(MX51_PIN_GPIO1_8);
	mxc_request_gpio(MX51_PIN_GPIO1_9);

	mxc_free_iomux(MX51_PIN_GPIO1_8, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_GPIO1_9, IOMUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbotg_hs_inactive);

/*
 * USB Host1 HS port
 */
int gpio_usbh1_active(void)
{
	/* Set USBH1_STP to GPIO and toggle it */
	mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_GPIO |
			  IOMUX_CONFIG_SION);
	mxc_iomux_set_pad(MX51_PIN_USBH1_STP, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);
	mxc_set_gpio_direction(MX51_PIN_USBH1_STP, 0);
	mxc_set_gpio_dataout(MX51_PIN_USBH1_STP, 1);

	msleep(100);

	/* USBH1_CLK */
	mxc_request_iomux(MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_CLK, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_NONE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH1_DIR */
	mxc_request_iomux(MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DIR, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH1_NXT */
	mxc_request_iomux(MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_NXT, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH1_DATA0 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA0, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA1 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA1, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA1, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA2 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA2, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA2, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA3 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA3, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA4 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA4, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA5 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA5, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA6 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA6, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA6, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH1_DATA7 */
	mxc_request_iomux(MX51_PIN_USBH1_DATA7, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_DATA7, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USB_PWR */
	mxc_request_iomux(MX51_PIN_GPIO1_8, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_8, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_NONE | PAD_CTL_HYS_ENABLE);

	/* USB_OC */
	mxc_request_iomux(MX51_PIN_GPIO1_9, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_9, PAD_CTL_SRE_SLOW |
			  PAD_CTL_DRV_LOW | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE);

	/* FPGA_USBOTG_RST_B */
	mxc_request_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_EIM_D17, PAD_CTL_DRV_HIGH |
			  PAD_CTL_HYS_NONE | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_100K_PU | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	mxc_set_gpio_direction(MX51_PIN_EIM_D17, 0);
	mxc_set_gpio_dataout(MX51_PIN_EIM_D17, 1);

	msleep(100);

	return 0;
}

EXPORT_SYMBOL(gpio_usbh1_active);

void gpio_usbh1_setback_stp(void)
{
	/* setback USBH1_STP to be function */
	mxc_request_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_USBH1_STP, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);
}

EXPORT_SYMBOL(gpio_usbh1_setback_stp);

void gpio_usbh1_inactive(void)
{
	mxc_request_gpio(MX51_PIN_USBH1_CLK);
	mxc_request_gpio(MX51_PIN_USBH1_DIR);
	mxc_request_gpio(MX51_PIN_USBH1_NXT);
	mxc_request_gpio(MX51_PIN_USBH1_STP);
	mxc_request_gpio(MX51_PIN_USBH1_DATA0);
	mxc_request_gpio(MX51_PIN_USBH1_DATA1);
	mxc_request_gpio(MX51_PIN_USBH1_DATA2);
	mxc_request_gpio(MX51_PIN_USBH1_DATA3);
	mxc_request_gpio(MX51_PIN_USBH1_DATA4);
	mxc_request_gpio(MX51_PIN_USBH1_DATA5);
	mxc_request_gpio(MX51_PIN_USBH1_DATA6);
	mxc_request_gpio(MX51_PIN_USBH1_DATA7);
	mxc_request_gpio(MX51_PIN_GPIO1_8);
	mxc_request_gpio(MX51_PIN_GPIO1_9);

	mxc_free_iomux(MX51_PIN_USBH1_CLK, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DIR, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_NXT, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_STP, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA1, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA2, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA6, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_USBH1_DATA7, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_GPIO1_8, IOMUX_CONFIG_GPIO);
	mxc_free_iomux(MX51_PIN_GPIO1_9, IOMUX_CONFIG_GPIO);

	mxc_free_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbh1_inactive);

int gpio_usbh2_active(void)
{
	/* Set USBH2_STP to GPIO and toggle it */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT1);
	mxc_iomux_set_pad(MX51_PIN_EIM_A26, PAD_CTL_DRV_HIGH |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
	mxc_set_gpio_direction(MX51_PIN_EIM_A26, 0);
	mxc_set_gpio_dataout(MX51_PIN_EIM_A26, 1);

	msleep(100);

	/* USBH2_CLK */
	mxc_request_iomux(MX51_PIN_EIM_A24, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_A24, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH2_DIR */
	mxc_request_iomux(MX51_PIN_EIM_A25, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_A25, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH2_NXT */
	mxc_request_iomux(MX51_PIN_EIM_A27, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_A27, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);

	/* USBH2_DATA0 */
	mxc_request_iomux(MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D16, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA1 */
	mxc_request_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D17, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA2 */
	mxc_request_iomux(MX51_PIN_EIM_D18, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D18, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA3 */
	mxc_request_iomux(MX51_PIN_EIM_D19, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D19, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA4 */
	mxc_request_iomux(MX51_PIN_EIM_D20, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D20, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA5 */
	mxc_request_iomux(MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D21, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA6 */
	mxc_request_iomux(MX51_PIN_EIM_D22, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D22, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	/* USBH2_DATA7 */
	mxc_request_iomux(MX51_PIN_EIM_D23, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_D23, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_100K_PU | PAD_CTL_PUE_KEEPER |
			  PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE);

	msleep(100);
	return 0;
}

EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_inactive(void)
{
	mxc_request_gpio(MX51_PIN_EIM_D16);
	mxc_request_gpio(MX51_PIN_EIM_D17);
	mxc_request_gpio(MX51_PIN_EIM_D18);
	mxc_request_gpio(MX51_PIN_EIM_D19);
	mxc_request_gpio(MX51_PIN_EIM_D20);
	mxc_request_gpio(MX51_PIN_EIM_D21);
	mxc_request_gpio(MX51_PIN_EIM_D22);
	mxc_request_gpio(MX51_PIN_EIM_D23);
	mxc_request_gpio(MX51_PIN_EIM_A24);
	mxc_request_gpio(MX51_PIN_EIM_A25);
	mxc_request_gpio(MX51_PIN_EIM_A26);
	mxc_request_gpio(MX51_PIN_EIM_A27);



	mxc_free_iomux(MX51_PIN_EIM_A24, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_A25, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_A27, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D18, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D19, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D20, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D22, IOMUX_CONFIG_ALT0);
	mxc_free_iomux(MX51_PIN_EIM_D23, IOMUX_CONFIG_ALT0);
}

EXPORT_SYMBOL(gpio_usbh2_inactive);

void gpio_usbh2_setback_stp(void)
{
	/* setback USBH2_STP to be function */
	mxc_request_iomux(MX51_PIN_EIM_A26, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_EIM_A26, PAD_CTL_SRE_FAST |
			  PAD_CTL_DRV_HIGH | PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS |
			  PAD_CTL_DRV_VOT_LOW);
}

EXPORT_SYMBOL(gpio_usbh2_setback_stp);

/*!
 * Setup GPIO for PCMCIA interface
 *
 */
void gpio_pcmcia_active(void)
{
}

EXPORT_SYMBOL(gpio_pcmcia_active);

/*!
 * Setup GPIO for pcmcia to be inactive
 */
void gpio_pcmcia_inactive(void)
{
}

EXPORT_SYMBOL(gpio_pcmcia_inactive);

/*!
 * Setup GPIO for fec to be active
 */
void gpio_fec_active(void)
{
	/*TX_ER */
	mxc_request_iomux(MX51_PIN_DI_GP3, IOMUX_CONFIG_ALT2);
	/*CRS */
	mxc_request_iomux(MX51_PIN_DI2_PIN4, IOMUX_CONFIG_ALT2);
	/*MDC */
	mxc_request_iomux(MX51_PIN_DI2_PIN2, IOMUX_CONFIG_ALT2);
	/*MDIO */
	mxc_request_iomux(MX51_PIN_DI2_PIN3, IOMUX_CONFIG_ALT2);
	/*RDATA[1] */
	mxc_request_iomux(MX51_PIN_DI2_DISP_CLK, IOMUX_CONFIG_ALT2);
	/*RDATA[2] */
	mxc_request_iomux(MX51_PIN_DI_GP4, IOMUX_CONFIG_ALT2);
	/*RDATA[3] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT0, IOMUX_CONFIG_ALT2);
	/*RX_ER */
	mxc_request_iomux(MX51_PIN_DISP2_DAT1, IOMUX_CONFIG_ALT2);
	/*TDATA[1] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT6, IOMUX_CONFIG_ALT2);
	/*TDATA[2] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT7, IOMUX_CONFIG_ALT2);
	/*TDATA[3] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT8, IOMUX_CONFIG_ALT2);
	/*TX_EN */
	mxc_request_iomux(MX51_PIN_DISP2_DAT9, IOMUX_CONFIG_ALT2);
	/*COL */
	mxc_request_iomux(MX51_PIN_DISP2_DAT10, IOMUX_CONFIG_ALT2);
	/*RX_CLK */
	mxc_request_iomux(MX51_PIN_DISP2_DAT11, IOMUX_CONFIG_ALT2);
	/*RX_DV */
	mxc_request_iomux(MX51_PIN_DISP2_DAT12, IOMUX_CONFIG_ALT2);
	/*TX_CLK */
	mxc_request_iomux(MX51_PIN_DISP2_DAT13, IOMUX_CONFIG_ALT2);
	/*RDATA[0] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT14, IOMUX_CONFIG_ALT2);
	/*TDATA[0] */
	mxc_request_iomux(MX51_PIN_DISP2_DAT15, IOMUX_CONFIG_ALT2);

	/*TX_ER */
	mxc_iomux_set_pad(MX51_PIN_DI_GP3,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*CRS */
	mxc_iomux_set_pad(MX51_PIN_DI2_PIN4,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*MDC */
	mxc_iomux_set_pad(MX51_PIN_DI2_PIN2,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*MDIO */
	mxc_iomux_set_pad(MX51_PIN_DI2_PIN3,
			  PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH |
			  PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_22K_PU |
			  PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_VOT_HIGH);
	/*RDATA[1] */
	mxc_iomux_set_pad(MX51_PIN_DI2_DISP_CLK,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RDATA[2] */
	mxc_iomux_set_pad(MX51_PIN_DI_GP4,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RDATA[3] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT0,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RX_ER */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT1,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TDATA[1] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT6,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TDATA[2] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT7,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TDATA[3] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT8,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TX_EN */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT9,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);
	/*COL */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT10,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RX_CLK */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT11,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RX_DV */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT12,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TX_CLK */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT13,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*RDATA[0] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT14,
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	/*TDATA[0] */
	mxc_iomux_set_pad(MX51_PIN_DISP2_DAT15,
			  PAD_CTL_100K_PU | PAD_CTL_DRV_HIGH |
			  PAD_CTL_DRV_VOT_HIGH);

	/*COL */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_COL_SELECT_INPUT, INPUT_CTL_PATH1);
	/*CRS */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_CRS_SELECT_INPUT, INPUT_CTL_PATH1);
	/*MDIO */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_MDI_SELECT_INPUT, INPUT_CTL_PATH1);
	/*RDATA[0] */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_0_SELECT_INPUT,
			    INPUT_CTL_PATH1);
	/*RDATA[1] */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_1_SELECT_INPUT,
			    INPUT_CTL_PATH1);
	/*RDATA[2] */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_2_SELECT_INPUT,
			    INPUT_CTL_PATH1);
	/*RDATA[3] */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RDATA_3_SELECT_INPUT,
			    INPUT_CTL_PATH1);
	/*RX_CLK */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RX_CLK_SELECT_INPUT,
			    INPUT_CTL_PATH1);
	/*RX_DV */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RX_DV_SELECT_INPUT, INPUT_CTL_PATH1);
	/*RX_ER */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_RX_ER_SELECT_INPUT, INPUT_CTL_PATH1);
	/*TX_CLK */
	mxc_iomux_set_input(MUX_IN_FEC_FEC_TX_CLK_SELECT_INPUT,
			    INPUT_CTL_PATH1);

	/*reset */
	mxc_request_iomux(MX51_PIN_DISPB2_SER_DIO, IOMUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX51_PIN_DISPB2_SER_DIO,
			  PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU
			  | PAD_CTL_PUE_PULL | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);
	mxc_set_gpio_direction(MX51_PIN_DISPB2_SER_DIO, 0);
	mxc_set_gpio_dataout(MX51_PIN_DISPB2_SER_DIO, 0);
	msleep(10);
	mxc_set_gpio_dataout(MX51_PIN_DISPB2_SER_DIO, 1);
	msleep(100);
}

EXPORT_SYMBOL(gpio_fec_active);

/*!
 * Setup GPIO for fec to be inactive
 */
void gpio_fec_inactive(void)
{
	/*TX_ER */
	mxc_free_iomux(MX51_PIN_DI_GP3, IOMUX_CONFIG_ALT2);
	/*CRS */
	mxc_free_iomux(MX51_PIN_DI2_PIN4, IOMUX_CONFIG_ALT2);
	/*MDC */
	mxc_free_iomux(MX51_PIN_DI2_PIN2, IOMUX_CONFIG_ALT2);
	/*MDIO */
	mxc_free_iomux(MX51_PIN_DI2_PIN3, IOMUX_CONFIG_ALT2);
	/*RDATA[1] */
	mxc_free_iomux(MX51_PIN_DI2_DISP_CLK, IOMUX_CONFIG_ALT2);
	/*RDATA[2] */
	mxc_free_iomux(MX51_PIN_DI_GP4, IOMUX_CONFIG_ALT2);
	/*RDATA[3] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT0, IOMUX_CONFIG_ALT2);
	/*RX_ER */
	mxc_free_iomux(MX51_PIN_DISP2_DAT1, IOMUX_CONFIG_ALT2);
	/*TDATA[1] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT6, IOMUX_CONFIG_ALT2);
	/*TDATA[2] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT7, IOMUX_CONFIG_ALT2);
	/*TDATA[3] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT8, IOMUX_CONFIG_ALT2);
	/*TX_EN */
	mxc_free_iomux(MX51_PIN_DISP2_DAT9, IOMUX_CONFIG_ALT2);
	/*COL */
	mxc_free_iomux(MX51_PIN_DISP2_DAT10, IOMUX_CONFIG_ALT2);
	/*RX_CLK */
	mxc_free_iomux(MX51_PIN_DISP2_DAT11, IOMUX_CONFIG_ALT2);
	/*RX_DV */
	mxc_free_iomux(MX51_PIN_DISP2_DAT12, IOMUX_CONFIG_ALT2);
	/*TX_CLK */
	mxc_free_iomux(MX51_PIN_DISP2_DAT13, IOMUX_CONFIG_ALT2);
	/*RDATA[0] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT14, IOMUX_CONFIG_ALT2);
	/*TDATA[0] */
	mxc_free_iomux(MX51_PIN_DISP2_DAT15, IOMUX_CONFIG_ALT2);
	/*reset */
	mxc_free_iomux(MX51_PIN_DISPB2_SER_DIO, IOMUX_CONFIG_GPIO);

}

EXPORT_SYMBOL(gpio_fec_inactive);

void gpio_spdif_active(void)
{
	mxc_request_iomux(MX51_PIN_GPIO1_7, IOMUX_CONFIG_ALT2);
	mxc_iomux_set_pad(MX51_PIN_GPIO1_7,
			  PAD_CTL_DRV_HIGH | PAD_CTL_PUE_PULL |
			  PAD_CTL_100K_PU | PAD_CTL_PKE_ENABLE |
			  PAD_CTL_SRE_FAST);
}

EXPORT_SYMBOL(gpio_spdif_active);

void gpio_spdif_inactive(void)
{
	mxc_free_iomux(MX51_PIN_GPIO1_7, IOMUX_CONFIG_ALT2);

}

EXPORT_SYMBOL(gpio_spdif_inactive);

int headphone_det_status(void)
{
	return mxc_get_gpio_datain(MX51_PIN_EIM_A26);
}

EXPORT_SYMBOL(headphone_det_status);
