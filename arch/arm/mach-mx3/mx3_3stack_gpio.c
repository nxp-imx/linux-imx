/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/irq.h>
#include <linux/pmic_adc.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "board-mx3_3stack.h"
#include "iomux.h"

/*!
 * @file mach-mx3/mx3_3stack_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX31
 */

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
		mxc_request_iomux(MX31_PIN_RXD1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_TXD1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_RTS1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CTS1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		break;
		/* UART 2 IOMUX Configs */
	case 1:
		mxc_request_iomux(MX31_PIN_TXD2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_RXD2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);

		mxc_request_iomux(MX31_PIN_RTS2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CTS2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		mxc_request_iomux(MX31_PIN_CSPI3_MOSI, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI3_MISO, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI3_SCLK, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI3_SPI_RDY, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		break;
	default:
		break;
	}

	/*
	 * TODO: Configure the Pad registers for the UART pins
	 */
}

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
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_RXD1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_TXD1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_RTS1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_CTS1), NULL);

		mxc_free_iomux(MX31_PIN_RXD1, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		mxc_free_iomux(MX31_PIN_TXD1, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		mxc_free_iomux(MX31_PIN_RTS1, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		mxc_free_iomux(MX31_PIN_CTS1, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		break;
	case 1:
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_TXD2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_RXD2), NULL);

		mxc_free_iomux(MX31_PIN_TXD2, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		mxc_free_iomux(MX31_PIN_RXD2, OUTPUTCONFIG_GPIO,
			       INPUTCONFIG_GPIO);
		mxc_free_iomux(MX31_PIN_RTS2, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CTS2, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
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
	switch (port) {
	case 1:
		/* Configure to receive UART 2 SDMA events */
		mxc_iomux_set_gpr(MUX_PGP_FIRI, false);
		break;
	case 2:
		/* Configure to receive UART 3 SDMA events */
		mxc_iomux_set_gpr(MUX_CSPI1_UART3, true);
		break;
	case 4:
		/* Configure to receive UART 5 SDMA events */
		mxc_iomux_set_gpr(MUX_CSPI3_UART5_SEL, true);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);
EXPORT_SYMBOL(config_uartdma_event);

/*!
 * Setup GPIO for Keypad  to be active
 *
 */
void gpio_keypad_active(void)
{
	/*
	 * Configure the IOMUX control register for keypad signals.
	 */
	mxc_request_iomux(MX31_PIN_KEY_COL0, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_COL1, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_COL2, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_COL3, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_ROW0, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_ROW1, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_KEY_ROW2, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Setup GPIO for Keypad to be inactive
 *
 */
void gpio_keypad_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_COL0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_COL1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_COL2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_COL3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_ROW0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_ROW1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_KEY_ROW2), NULL);

	mxc_free_iomux(MX31_PIN_KEY_COL0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_COL1, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_COL2, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_COL3, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_ROW0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_ROW1, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_KEY_ROW2, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_keypad_inactive);

void gpio_power_key_active(void)
{
	mxc_request_iomux(MX31_PIN_GPIO1_2, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO1_2), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_GPIO1_2));
	mxc_iomux_set_pad(MX31_PIN_GPIO1_2, PAD_CTL_PKE_NONE);
}

EXPORT_SYMBOL(gpio_power_key_active);

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
		/* setup GPR for CSPI BB */
		iomux_config_gpr(MUX_PGP_CSPI_BB, true);
		/* CSPI1 clock and RDY use full UART ALT1 mode */
		mxc_request_iomux(MX31_PIN_DSR_DCE1, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_RI_DCE1, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		break;
	case 1:
		/* SPI2 */
		mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CSPI2_SPI_RDY, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		break;
	case 2:
		/* SPI3 */
		/*
		   mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_SPI_RDY, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_SS1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		   mxc_request_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
		 */
		break;
	default:
		break;
	}
}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_active(void)
{
	/*
	 * Configure the IOMUX control register for 1-wire signals.
	 */
	iomux_config_mux(MX31_PIN_BATT_LINE, OUTPUTCONFIG_FUNC,
			 INPUTCONFIG_FUNC);
	iomux_config_pad(MX31_PIN_BATT_LINE, PAD_CTL_LOOPBACK);
}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_inactive(void)
{
	/*
	 * Configure the IOMUX control register for 1-wire signals.
	 */
	iomux_config_mux(MX31_PIN_BATT_LINE, OUTPUTCONFIG_GPIO,
			 INPUTCONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_owire_active);
EXPORT_SYMBOL(gpio_owire_inactive);

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
		/* setup GPR for CSPI BB */
		iomux_config_gpr(MUX_PGP_CSPI_BB, false);
		/* CSPI1 clock and RDY use full UART ALT1 mode */
		mxc_free_iomux(MX31_PIN_DSR_DCE1, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_RI_DCE1, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		break;
	case 1:
		/* SPI2 */
		mxc_free_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CSPI2_SPI_RDY, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CSPI2_SS0, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		break;
	case 2:
		/* SPI3 */
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		mxc_request_iomux(MX31_PIN_I2C_CLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_I2C_DAT, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		break;
	case 1:
		mxc_request_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		break;
	case 2:
		mxc_request_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
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
	switch (i2c_num) {
	case 0:
		mxc_free_iomux(MX31_PIN_I2C_CLK, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_I2C_DAT, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		break;
	case 1:
		mxc_free_iomux(MX31_PIN_CSPI2_MOSI, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_ALT1);
		mxc_free_iomux(MX31_PIN_CSPI2_MISO, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_ALT1);
		break;
	case 2:
		mxc_request_iomux(MX31_PIN_CSPI2_SS2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_CSPI2_SCLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_ALT1);
		break;
	default:
		break;
	}
}

/*!
 * This function configures the IOMux block for PMIC standard operations.
 *
 */
void gpio_pmic_active(void)
{
	mxc_request_iomux(MX31_PIN_GPIO1_3, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO1_3), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_GPIO1_3));
}

EXPORT_SYMBOL(gpio_pmic_active);

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	switch (module) {
	case 0:
		mxc_request_iomux(MX31_PIN_SD1_CLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_CMD, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA0, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX31_PIN_SD1_DATA3, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);

		mxc_iomux_set_pad(MX31_PIN_SD1_CLK,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));
		mxc_iomux_set_pad(MX31_PIN_SD1_CMD,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST
				   | PAD_CTL_100K_PU));

		/*
		 * Active the Buffer Enable Pin only if there is
		 * a card in slot.
		 * To fix the card voltage issue caused by
		 * bi-directional chip TXB0108 on 3Stack
		 */
		if (gpio_get_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_1)))
			gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 0);
		else
			gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 1);
		break;
	case 1:
		mxc_request_iomux(MX31_PIN_PC_CD2_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_CD1_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_WAIT_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_READY, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_VS1, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX31_PIN_PC_PWRON, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
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
		mxc_free_iomux(MX31_PIN_SD1_CLK, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_SD1_CMD, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_SD1_DATA0, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_SD1_DATA1, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_SD1_DATA2, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);
		mxc_free_iomux(MX31_PIN_SD1_DATA3, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_FUNC);

		mxc_iomux_set_pad(MX31_PIN_SD1_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX31_PIN_SD1_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX31_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));

		/* Buffer Enable Pin of SD, Active HI */
		gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 0);
		break;
	case 1:
		/* TODO:what are the pins for SDHC2? */
		mxc_free_iomux(MX31_PIN_PC_CD2_B, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
		mxc_free_iomux(MX31_PIN_PC_CD1_B, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
		mxc_free_iomux(MX31_PIN_PC_WAIT_B, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
		mxc_free_iomux(MX31_PIN_PC_READY, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
		mxc_free_iomux(MX31_PIN_PC_VS1, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
		mxc_free_iomux(MX31_PIN_PC_PWRON, OUTPUTCONFIG_FUNC,
			       INPUTCONFIG_NONE);
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
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_1));
		/*
		 * Active the Buffer Enable Pin only if there is
		 * a card in slot.
		 * To fix the card voltage issue caused by
		 * bi-directional chip TXB0108 on 3Stack
		 */
		if (ret)
			gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 0);
		else
			gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 1);
		return ret;
	} else
		return gpio_get_value(IOMUX_TO_GPIO(MX31_PIN_GPIO1_2));
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*
 * Return the card detect pin.
 */
int sdhc_init_card_det(int id)
{
	if (id == 0) {
		/* Buffer Enable Pin, Active HI */
		mxc_request_iomux(MX31_PIN_GPIO3_0, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), "gpio3_0");
		gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 0);
		gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_GPIO3_0), 0);

		/* CD Pin */
		mxc_request_iomux(MX31_PIN_GPIO3_1, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_GPIO);
		mxc_iomux_set_pad(MX31_PIN_GPIO3_1, PAD_CTL_PKE_NONE);
		gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO3_1), "gpio3_1");
		gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_GPIO3_1));
		return IOMUX_TO_IRQ(MX31_PIN_GPIO3_1);
	} else {
		iomux_config_mux(MX31_PIN_GPIO1_2, OUTPUTCONFIG_GPIO,
				 INPUTCONFIG_GPIO);
		return IOMUX_TO_IRQ(MX31_PIN_GPIO1_2);

	}
}

EXPORT_SYMBOL(sdhc_init_card_det);

/*!
 * Get SD1_WP ADIN7 of ATLAS pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	pmic_adc_convert(GEN_PURPOSE_AD7, &rc);
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

	mxc_request_iomux(MX31_PIN_LD0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD9, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD10, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD11, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD12, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD13, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD14, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD15, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_LD16, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// LD16
	mxc_request_iomux(MX31_PIN_LD17, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// LD17
	mxc_request_iomux(MX31_PIN_VSYNC3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// VSYNC
	mxc_request_iomux(MX31_PIN_HSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// HSYNC
	mxc_request_iomux(MX31_PIN_FPSHIFT, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// CLK
	mxc_request_iomux(MX31_PIN_CONTRAST, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// CONTR

#ifdef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
	mxc_request_iomux(MX31_PIN_DRDY0,
			OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	/* DRDY */
	mxc_request_iomux(MX31_PIN_D3_REV,
			OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	/* REV */
	mxc_request_iomux(MX31_PIN_D3_SPL,
			OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	/* SPL */
	mxc_request_iomux(MX31_PIN_D3_CLS,
			OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	/* CLS */
#else
	/* ensure that LCDIO(1.8V) has been turn on */
	/* active reset line GPIO */
	mxc_request_iomux(MX31_PIN_LCS1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_LCS1), "lcs1");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_LCS1), 0);
	/* do reset */
	mdelay(10);		/* tRES >= 100us */
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_LCS1), 1);

	/* enable data */
	mxc_request_iomux(MX31_PIN_SER_RS, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SER_RS), "ser_rs");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_SER_RS), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SER_RS), 1);
#endif
}

/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SER_RS), 0);
}

/*!
 * Switch to the specified sensor - MX31 ADS has two
 *
 */
void gpio_sensor_select(int sensor)
{
}

/*!
 * Setup GPIO for sensor to be active
 *
 */
void gpio_sensor_active(void)
{
	gpio_sensor_select(0);

	/*
	 * Configure the iomuxen for the CSI.
	 */

	mxc_request_iomux(MX31_PIN_CSI_D5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_CSI_D5), "csi_d5");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_CSI_D5), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_CSI_D5), 0);

	mxc_request_iomux(MX31_PIN_CSI_D6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D9, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D10, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D11, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D12, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D13, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D14, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_D15, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_HSYNC, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_MCLK, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_PIXCLK, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_CSI_VSYNC, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);

	if (mxc_request_iomux(MX31_PIN_A23, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE)
	    == 0) {
		printk(KERN_ERR "%s:REGEN set request gpio ok\n", __func__);
	} else {
		printk(KERN_ERR "%s:REGEN set error, request gpio error\n",
		       __func__);
		return;
	}
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SD_D_IO), "sd_d_io");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_SD_D_IO), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SD_D_IO), 1);
}

EXPORT_SYMBOL(gpio_sensor_active);

void gpio_sensor_reset(bool flag)
{
}

EXPORT_SYMBOL(gpio_sensor_reset);

/*!
 * Setup GPIO for sensor to be inactive
 *
 */
void gpio_sensor_inactive(void)
{
	mxc_free_iomux(MX31_PIN_CSI_D5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D8, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D9, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D10, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D11, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D12, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D13, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D14, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_D15, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_HSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_MCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_PIXCLK, OUTPUTCONFIG_FUNC,
		       INPUTCONFIG_FUNC);
	mxc_free_iomux(MX31_PIN_CSI_VSYNC, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Setup GPIO for ATA interface
 *
 */
void gpio_ata_active(void)
{
	/*
	 * Configure the GPR for ATA group B signals
	 */
	mxc_iomux_set_gpr(MUX_PGP_ATA_8 | MUX_PGP_ATA_5 | MUX_PGP_ATA_4 |
			  MUX_PGP_ATA_3 | MUX_PGP_ATA_2, false);

	mxc_iomux_set_gpr(MUX_PGP_ATA_9 | MUX_PGP_ATA_7 | MUX_PGP_ATA_6 |
			  MUX_PGP_ATA_1, true);

	/*
	 * Configure the IOMUX for ATA group B signals
	 */

	mxc_request_iomux(MX31_PIN_CSPI1_MOSI, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D0
	mxc_request_iomux(MX31_PIN_CSPI1_MISO, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D1
	mxc_request_iomux(MX31_PIN_CSPI1_SS0, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D2
	mxc_request_iomux(MX31_PIN_CSPI1_SS1, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D3
	mxc_request_iomux(MX31_PIN_CSPI1_SS2, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D4
	mxc_request_iomux(MX31_PIN_CSPI1_SCLK, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D5
	mxc_request_iomux(MX31_PIN_CSPI1_SPI_RDY, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D6
	mxc_request_iomux(MX31_PIN_STXD3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D7
	mxc_request_iomux(MX31_PIN_SRXD3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D8
	mxc_request_iomux(MX31_PIN_SCK3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D9
	mxc_request_iomux(MX31_PIN_SFS3, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D10
	mxc_request_iomux(MX31_PIN_STXD6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D11
	mxc_request_iomux(MX31_PIN_SRXD6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D12
	mxc_request_iomux(MX31_PIN_SCK6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D13
	mxc_request_iomux(MX31_PIN_CAPTURE, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D14
	mxc_request_iomux(MX31_PIN_COMPARE, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_D15

	/* Config the multiplex pin of ATA interface DIR, DA0-2, INTRQ, DMARQ */
	mxc_request_iomux(MX31_PIN_KEY_COL4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_DMARQ_B
	mxc_request_iomux(MX31_PIN_KEY_ROW6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_INTRQ_B
	mxc_request_iomux(MX31_PIN_KEY_COL5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_DA0
	mxc_request_iomux(MX31_PIN_KEY_COL6, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_DA1
	mxc_request_iomux(MX31_PIN_KEY_COL7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_DA2
	mxc_request_iomux(MX31_PIN_KEY_ROW7, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);	// ATA_BUFFER_DIR

	/* HDD_ENABLE_B(H:Disable,L:Enable) */
	mxc_request_iomux(MX31_PIN_CSI_D4, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);	// HDD_ENABLE_B
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_CSI_D4), "csi_d4");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_CSI_D4), 0);
	mdelay(10);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_CSI_D4), 0);
	mdelay(10);

	/* These ATA pins are common to Group A and Group B */

	mxc_request_iomux(MX31_PIN_ATA_CS0, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_ATA_CS1, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_ATA_DIOR, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_ATA_DIOW, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_ATA_DMACK, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_ATA_RESET_B, OUTPUTCONFIG_FUNC,
			  INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_PWMO, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

	/* Need fast slew rate for UDMA mode */

#define ATA_DAT_PAD_CFG (PAD_CTL_SRE_FAST | PAD_CTL_PKE_NONE | PAD_CTL_100K_PU)
	mxc_iomux_set_pad(MX31_PIN_CSPI1_MISO, ATA_DAT_PAD_CFG);	// data 0
	mxc_iomux_set_pad(MX31_PIN_CSPI1_MOSI, ATA_DAT_PAD_CFG);	// data 1
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS0, ATA_DAT_PAD_CFG);	// data 2
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS1, ATA_DAT_PAD_CFG);	// data 3
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS2, ATA_DAT_PAD_CFG);	// data 4
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SCLK, ATA_DAT_PAD_CFG);	// data 5
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SPI_RDY, ATA_DAT_PAD_CFG);	// data 6
	mxc_iomux_set_pad(MX31_PIN_STXD3, ATA_DAT_PAD_CFG);	// data 7
	mxc_iomux_set_pad(MX31_PIN_SRXD3, ATA_DAT_PAD_CFG);	// data 8
	mxc_iomux_set_pad(MX31_PIN_SCK3, ATA_DAT_PAD_CFG);	// data 9
	mxc_iomux_set_pad(MX31_PIN_SFS3, ATA_DAT_PAD_CFG);	// data 10
	mxc_iomux_set_pad(MX31_PIN_STXD6, ATA_DAT_PAD_CFG);	// data 11
	mxc_iomux_set_pad(MX31_PIN_SRXD6, ATA_DAT_PAD_CFG);	// data 12
	mxc_iomux_set_pad(MX31_PIN_SCK6, ATA_DAT_PAD_CFG);	// data 13
	mxc_iomux_set_pad(MX31_PIN_CAPTURE, ATA_DAT_PAD_CFG);	// data 14
	mxc_iomux_set_pad(MX31_PIN_COMPARE, ATA_DAT_PAD_CFG);	// data 15
#undef ATA_DAT_PAD_CFG

#define ATA_CTL_PAD_CFG (PAD_CTL_SRE_SLOW | PAD_CTL_PKE_NONE)
	mxc_iomux_set_pad(MX31_PIN_KEY_COL4, ATA_CTL_PAD_CFG);	// ATA_DMARQ);
	mxc_iomux_set_pad(MX31_PIN_KEY_ROW6, ATA_CTL_PAD_CFG);	// ATA_INTRQ);
	mxc_iomux_set_pad(MX31_PIN_KEY_COL5, ATA_CTL_PAD_CFG);	//
	mxc_iomux_set_pad(MX31_PIN_KEY_COL6, ATA_CTL_PAD_CFG);	//
	mxc_iomux_set_pad(MX31_PIN_KEY_COL7, ATA_CTL_PAD_CFG);	//
	mxc_iomux_set_pad(MX31_PIN_KEY_ROW7, ATA_CTL_PAD_CFG);	//

	mxc_iomux_set_pad(MX31_PIN_ATA_CS0, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_CS1, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DIOR, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DIOW, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DMACK, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_RESET_B, ATA_CTL_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_PWMO, ATA_CTL_PAD_CFG);
#undef ATA_CTL_PAD_CFG
}

EXPORT_SYMBOL(gpio_ata_active);

/*!
 * Restore ATA interface pins to reset values
 *
 */
void gpio_ata_inactive(void)
{
	/*
	 * Turn off ATA group B signals
	 */
	mxc_request_iomux(MX31_PIN_CSPI1_MOSI, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D0
	mxc_request_iomux(MX31_PIN_CSPI1_MISO, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D1
	mxc_request_iomux(MX31_PIN_CSPI1_SS0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D2
	mxc_request_iomux(MX31_PIN_CSPI1_SS1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D3
	mxc_request_iomux(MX31_PIN_CSPI1_SS2, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D4
	mxc_request_iomux(MX31_PIN_CSPI1_SCLK, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D5
	mxc_request_iomux(MX31_PIN_CSPI1_SPI_RDY, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D6
	mxc_request_iomux(MX31_PIN_STXD3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D7
	mxc_request_iomux(MX31_PIN_SRXD3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D8
	mxc_request_iomux(MX31_PIN_SCK3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D9
	mxc_request_iomux(MX31_PIN_SFS3, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D10
	mxc_request_iomux(MX31_PIN_STXD6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D11
	mxc_request_iomux(MX31_PIN_SRXD6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D12
	mxc_request_iomux(MX31_PIN_SCK6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D13
	mxc_request_iomux(MX31_PIN_CAPTURE, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D14
	mxc_request_iomux(MX31_PIN_COMPARE, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_D15

	/* Config the multiplex pin of ATA interface DIR, DA0-2, INTRQ, DMARQ */
	mxc_request_iomux(MX31_PIN_KEY_COL4, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_DMARQ_B
	mxc_request_iomux(MX31_PIN_KEY_ROW6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_INTRQ_B
	mxc_request_iomux(MX31_PIN_KEY_COL5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_DA0
	mxc_request_iomux(MX31_PIN_KEY_COL6, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_DA1
	mxc_request_iomux(MX31_PIN_KEY_COL7, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_DA2
	mxc_request_iomux(MX31_PIN_KEY_ROW7, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);	// ATA_BUFFER_DIR

	/* HDD_BUFF_EN (H:A->B, L:B->A) and HDD_ENABLE_B(H:Disable,L:Enable) */
	mxc_free_iomux(MX31_PIN_CSI_D4, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);

	/* These ATA pins are common to Group A and Group B */

	mxc_request_iomux(MX31_PIN_ATA_CS0, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_ATA_CS1, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_ATA_DIOR, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_ATA_DIOW, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_ATA_DMACK, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_ATA_RESET_B, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_request_iomux(MX31_PIN_PWMO, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	/* Needed fast slew rate for UDMA mode */

#define ATA_DAT_PAD_CFG (PAD_CTL_SRE_SLOW | PAD_CTL_DRV_NORMAL | PAD_CTL_PKE_NONE)
	mxc_iomux_set_pad(MX31_PIN_KEY_COL4, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_KEY_ROW6, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_KEY_COL5, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_KEY_COL6, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_KEY_COL7, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_KEY_ROW7, ATA_DAT_PAD_CFG);

	mxc_iomux_set_pad(MX31_PIN_ATA_CS0, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_CS1, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DIOR, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DIOW, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_DMACK, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_ATA_RESET_B, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_PWMO, ATA_DAT_PAD_CFG);

	mxc_iomux_set_pad(MX31_PIN_CSPI1_MISO, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_MOSI, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS0, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS1, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS2, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SCLK, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SPI_RDY, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_STXD3, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SRXD3, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SCK3, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SFS3, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_STXD6, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SRXD6, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SCK6, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CAPTURE, ATA_DAT_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_COMPARE, ATA_DAT_PAD_CFG);
#undef ATA_DAT_PAD_CFG
}

EXPORT_SYMBOL(gpio_ata_inactive);

/* *INDENT-OFF* */
/*
 * USB Host 1
 * pins conflict with SPI1, ATA, UART3
 */
int gpio_usbh1_active(void)
{
	return 0;
}

EXPORT_SYMBOL(gpio_usbh1_active);

void gpio_usbh1_inactive(void)
{
	/* Do nothing as pins don't have/support GPIO mode */

}

EXPORT_SYMBOL(gpio_usbh1_inactive);

/*
 * USB Host 2
 * pins conflict with UART5, PCMCIA
 */
int gpio_usbh2_active(void)
{
	if (mxc_request_iomux(MX31_PIN_USBH2_CLK,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBH2_DIR,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBH2_NXT,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBH2_STP,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBH2_DATA0,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBH2_DATA1,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_PC_VS2,		/* USBH2_DATA2 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1) ||
	    mxc_request_iomux(MX31_PIN_PC_BVD1,		/* USBH2_DATA3 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1) ||
	    mxc_request_iomux(MX31_PIN_PC_BVD2,		/* USBH2_DATA4 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1) ||
	    mxc_request_iomux(MX31_PIN_PC_RST,		/* USBH2_DATA5 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1) ||
	    mxc_request_iomux(MX31_PIN_IOIS16,		/* USBH2_DATA6 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1) ||
	    mxc_request_iomux(MX31_PIN_PC_RW_B,		/* USBH2_DATA7 */
			      OUTPUTCONFIG_ALT1, INPUTCONFIG_ALT1)) {
		return -EINVAL;
	}
	mxc_iomux_set_pad(MX31_PIN_USBH2_CLK,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST |
			   PAD_CTL_PKE_NONE));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DIR,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_NXT,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_STP,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA0,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA1,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_VS2,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_BVD1,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_BVD2,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_RST,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_IOIS16,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_RW_B,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));

	mxc_request_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO,
			INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), "usb_byp");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), 0);
	mdelay(1);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), 1);
	return 0;
}

EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_inactive(void)
{
	mxc_iomux_set_pad(MX31_PIN_USBH2_CLK,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DIR,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_NXT,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_STP,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA0,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA1,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_VS2,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_BVD1,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_BVD2,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_RST,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_IOIS16,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_PC_RW_B,
			 (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_FAST));

	mxc_free_iomux(MX31_PIN_USBH2_CLK,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_USBH2_DIR,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_USBH2_NXT,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_USBH2_STP,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_USBH2_DATA0,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_USBH2_DATA1,
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	mxc_free_iomux(MX31_PIN_PC_VS2,			/* USBH2_DATA2 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_PC_BVD1,		/* USBH2_DATA3 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_PC_BVD2,		/* USBH2_DATA4 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_PC_RST,			/* USBH2_DATA5 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_IOIS16,			/* USBH2_DATA6 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_PC_RW_B,		/* USBH2_DATA7 */
		       OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	mxc_free_iomux(MX31_PIN_USB_BYP,		/* USBH2 PHY reset */
			OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
}

EXPORT_SYMBOL(gpio_usbh2_inactive);

/*
 * USB OTG HS port
 */
int gpio_usbotg_hs_active(void)
{
	if (mxc_request_iomux(MX31_PIN_USBOTG_DATA0,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA1,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA2,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA3,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA4,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA5,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA6,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA7,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_CLK,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DIR,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_NXT,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_STP,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC)) {
		return -EINVAL;
	}

	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA0,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA1,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA2,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA3,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA4,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA5,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA6,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA7,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_CLK,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DIR,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_NXT,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));
	mxc_iomux_set_pad(MX31_PIN_USBOTG_STP,
			  (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST));

	/* reset transceiver */
	mxc_request_iomux(MX31_PIN_USB_PWR, OUTPUTCONFIG_GPIO,
		INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_USB_PWR), "usb_pwr");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_USB_PWR), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_USB_PWR), 0);
	mdelay(1);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_USB_PWR), 1);

	return 0;
}

EXPORT_SYMBOL(gpio_usbotg_hs_active);

void gpio_usbotg_hs_inactive(void)
{
	mxc_free_iomux(MX31_PIN_USB_PWR, OUTPUTCONFIG_GPIO,
		INPUTCONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_usbotg_hs_inactive);

/*!
 * USB OTG FS port
 */
int gpio_usbotg_fs_active(void)
{
	if (mxc_request_iomux(MX31_PIN_USBOTG_DATA0,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA1,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA2,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA3,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA4,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA5,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA6,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DATA7,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_CLK,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_DIR,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_NXT,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USBOTG_STP,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC) ||
	    mxc_request_iomux(MX31_PIN_USB_PWR,
			      OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC))
		return -EINVAL;
	return 0;
}

EXPORT_SYMBOL(gpio_usbotg_fs_active);

void gpio_usbotg_fs_inactive(void)
{
	/* Do nothing as  pins doesn't have/support GPIO mode */

}

EXPORT_SYMBOL(gpio_usbotg_fs_inactive);

/*!
 * GPS GPIO
 */
void gpio_gps_active(void)
{
	/* POWER_EN */
	mxc_request_iomux(MX31_PIN_SCLK0,
			OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SCLK0), "sclk0");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_SCLK0), 0);
	/* Reset Pin */
	mxc_request_iomux(MX31_PIN_DCD_DTE1,
			OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_DCD_DTE1), "dcd_dte1");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_DCD_DTE1), 0);

	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SCLK0), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_DCD_DTE1), 0);

	msleep(5);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_DCD_DTE1), 1);
	msleep(5);
}

EXPORT_SYMBOL(gpio_gps_active);

int gpio_gps_access(int para)
{
	iomux_pin_name_t pin;
	pin = (para & 0x1) ? MX31_PIN_SCLK0 : MX31_PIN_DCD_DTE1;

	if (para & 0x4) /* Read GPIO */
		return gpio_get_value(IOMUX_TO_GPIO(pin));
	else if (para & 0x2) /* Write GPIO */
		gpio_set_value(IOMUX_TO_GPIO(pin), 1);
	else
		gpio_set_value(IOMUX_TO_GPIO(pin), 0);
	return 0;
}

EXPORT_SYMBOL(gpio_gps_access);

void gpio_gps_inactive(void)
{
	mxc_free_iomux(MX31_PIN_DCD_DTE1,
			OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_SCLK0,
			OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
}

EXPORT_SYMBOL(gpio_gps_inactive);
