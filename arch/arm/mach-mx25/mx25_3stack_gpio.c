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

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "board-mx25_3stack.h"
#include "iomux.h"

/*!
 * @file mach-mx25/mx25_3stack_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX25
 */
static struct mxc_iomux_pin_cfg __initdata mxc_iomux_pins[] = {
};

static struct mxc_iomux_pin_cfg __initdata sim_iomux_pins[] = {
	/* SIM1 */
	/* SIM1 CLK */
	{
	 MX25_PIN_CSI_D2, MUX_CONFIG_ALT4,
	 PAD_CTL_SRE_FAST |
	 PAD_CTL_DRV_HIGH | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_CMOS | PAD_CTL_47K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM1 RST */
	{
	 MX25_PIN_CSI_D3, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_HIGH | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_CMOS | PAD_CTL_47K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM1 VEN */
	{
	 MX25_PIN_CSI_D4, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_HIGH | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_CMOS | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM1 TX */
	{
	 MX25_PIN_CSI_D5, MUX_CONFIG_ALT4,
	 PAD_CTL_SRE_FAST |
	 PAD_CTL_DRV_HIGH | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_CMOS | PAD_CTL_22K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_OpenDrain | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM1 PD */
	{
	 MX25_PIN_CSI_D6, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_HIGH | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_CMOS | PAD_CTL_22K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM2 */
	/* SIM2 CLK */
	{
	 MX25_PIN_CSI_D8, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_NORMAL | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_SCHMITZ | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_KEEPER | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM2 RST */
	{
	 MX25_PIN_CSI_D9, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_NORMAL | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_SCHMITZ | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_KEEPER | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM2 VEN */
	{
	 MX25_PIN_CSI_MCLK, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_NORMAL | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_SCHMITZ | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_PULL | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_NONE,
	 },
	/* SIM2 TX */
	{
	 MX25_PIN_CSI_VSYNC, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_NORMAL | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_SCHMITZ | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_KEEPER | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
	/* SIM2 PD */
	{
	 MX25_PIN_CSI_HSYNC, MUX_CONFIG_ALT4,
	 PAD_CTL_DRV_NORMAL | PAD_CTL_DRV_3_3V |
	 PAD_CTL_HYS_SCHMITZ | PAD_CTL_100K_PU |
	 PAD_CTL_PUE_KEEPER | PAD_CTL_ODE_CMOS | PAD_CTL_PKE_ENABLE,
	 },
};

static int __initdata enable_sim = { 0 };
static int __init sim_setup(char *__unused)
{
	enable_sim = 1;
	return 1;
}

__setup("sim", sim_setup);

/*!
 * This system-wide GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by
 * \b fixup_mx25_3stack() during system startup. This function is board
 * specific.
 */
void __init mx25_3stack_gpio_init(void)
{
	int i, num = 0;
	struct mxc_iomux_pin_cfg *pin_ptr;

	for (i = 0; i < ARRAY_SIZE(mxc_iomux_pins); i++) {
		mxc_request_iomux(mxc_iomux_pins[i].pin,
				  mxc_iomux_pins[i].mux_mode);
		if (mxc_iomux_pins[i].pad_cfg)
			mxc_iomux_set_pad(mxc_iomux_pins[i].pin,
					  mxc_iomux_pins[i].pad_cfg);
		if (mxc_iomux_pins[i].in_select)
			mxc_iomux_set_input(mxc_iomux_pins[i].in_select,
					    mxc_iomux_pins[i].in_mode);
	}

	if (enable_sim) {
		pin_ptr = sim_iomux_pins;
		num = ARRAY_SIZE(sim_iomux_pins);
	}

	for (i = 0; i < num; i++) {
		mxc_request_iomux(pin_ptr[i].pin, pin_ptr[i].mux_mode);
		if (pin_ptr[i].pad_cfg)
			mxc_iomux_set_pad(pin_ptr[i].pin, pin_ptr[i].pad_cfg);
		if (pin_ptr[i].in_select)
			mxc_iomux_set_input(pin_ptr[i].in_select,
					pin_ptr[i].in_mode);
	}
}

/*!
 * Activate a UART port
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
	case 0:
		/* UART 1 IOMUX Configs */
		mxc_request_iomux(MX25_PIN_UART1_RXD, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART1_TXD, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART1_RTS, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART1_CTS, MUX_CONFIG_FUNC);
		mxc_iomux_set_pad(MX25_PIN_UART1_RXD,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX25_PIN_UART1_TXD,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX25_PIN_UART1_RTS,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX25_PIN_UART1_CTS,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);

		break;
	case 1:
		/* UART 2 IOMUX Configs */
		mxc_request_iomux(MX25_PIN_UART2_RXD, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART2_TXD, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART2_RTS, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_UART2_CTS, MUX_CONFIG_FUNC);
		mxc_iomux_set_pad(MX25_PIN_UART2_RXD,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX25_PIN_UART2_TXD, PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX25_PIN_UART2_RTS,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX25_PIN_UART2_CTS, PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		break;
	case 2:
		/* UART 3 IOMUX Configs */
		mxc_request_iomux(MX25_PIN_KPP_ROW0, MUX_CONFIG_ALT1); /*RXD*/
		mxc_request_iomux(MX25_PIN_KPP_ROW1, MUX_CONFIG_ALT1); /*TXD*/
		mxc_request_iomux(MX25_PIN_KPP_ROW2, MUX_CONFIG_ALT1); /*RTS*/
		mxc_request_iomux(MX25_PIN_KPP_ROW3, MUX_CONFIG_ALT1); /*CTS*/

		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RTS_B,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_input(MUX_IN_UART3_IPP_UART_RXD_MUX,
				    INPUT_CTL_PATH1);
		break;
	case 3:
		/* UART 4 IOMUX Configs */
		mxc_request_iomux(MX25_PIN_LD8, MUX_CONFIG_ALT2); /*RXD*/
		mxc_request_iomux(MX25_PIN_LD9, MUX_CONFIG_ALT2); /*TXD*/
		mxc_request_iomux(MX25_PIN_LD10, MUX_CONFIG_ALT2); /*RTS*/
		mxc_request_iomux(MX25_PIN_LD11, MUX_CONFIG_ALT2); /*CTS*/

		mxc_iomux_set_input(MUX_IN_UART4_IPP_UART_RTS_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_UART4_IPP_UART_RXD_MUX,
				    INPUT_CTL_PATH0);
	case 4:
		/* UART 5 IOMUX Configs */
		mxc_request_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_ALT1); /*RXD*/
		mxc_request_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_ALT1); /*TXD*/
		mxc_request_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_ALT1); /*RTS*/
		mxc_request_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_ALT1); /*CTS*/

		mxc_iomux_set_input(MUX_IN_UART5_IPP_UART_RTS_B,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_input(MUX_IN_UART5_IPP_UART_RXD_MUX,
				    INPUT_CTL_PATH1);
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_uart_active);

/*!
 * Inactivate a UART port
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	switch (port) {
	case 0:
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART1_RXD), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART1_TXD), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART1_RTS), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART1_CTS), NULL);

		mxc_free_iomux(MX25_PIN_UART1_RXD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART1_TXD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART1_RTS, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART1_CTS, MUX_CONFIG_GPIO);
		break;
	case 1:
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART2_RXD), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART2_TXD), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART2_RTS), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART2_CTS), NULL);

		mxc_free_iomux(MX25_PIN_UART2_RXD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART2_TXD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART2_RTS, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART2_CTS, MUX_CONFIG_GPIO);
		break;
	case 2:
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW0), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW3), NULL);

		mxc_free_iomux(MX25_PIN_KPP_ROW0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_KPP_ROW1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_KPP_ROW2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_KPP_ROW3, MUX_CONFIG_GPIO);
		break;
	case 3:
		mxc_free_iomux(MX25_PIN_LD8, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX25_PIN_LD9, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX25_PIN_LD10, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX25_PIN_LD11, MUX_CONFIG_FUNC);
		break;
	case 4:
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D3), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D4), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D5), NULL);

		mxc_free_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_GPIO);
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
 * Activate Keypad
 */
void gpio_keypad_active(void)
{
	mxc_request_iomux(MX25_PIN_KPP_ROW0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_ROW1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_ROW2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_ROW3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_COL0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_COL1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_COL2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_KPP_COL3, MUX_CONFIG_FUNC);

#define KPP_PAD_CTL_ROW (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | \
			 PAD_CTL_100K_PU)
#define KPP_PAD_CTL_COL (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | \
			 PAD_CTL_100K_PU | PAD_CTL_ODE_OpenDrain)

	mxc_iomux_set_pad(MX25_PIN_KPP_ROW0, KPP_PAD_CTL_ROW);
	mxc_iomux_set_pad(MX25_PIN_KPP_ROW1, KPP_PAD_CTL_ROW);
	mxc_iomux_set_pad(MX25_PIN_KPP_ROW2, KPP_PAD_CTL_ROW);
	mxc_iomux_set_pad(MX25_PIN_KPP_ROW3, KPP_PAD_CTL_ROW);
	mxc_iomux_set_pad(MX25_PIN_KPP_COL0, KPP_PAD_CTL_COL);
	mxc_iomux_set_pad(MX25_PIN_KPP_COL1, KPP_PAD_CTL_COL);
	mxc_iomux_set_pad(MX25_PIN_KPP_COL2, KPP_PAD_CTL_COL);
	mxc_iomux_set_pad(MX25_PIN_KPP_COL3, KPP_PAD_CTL_COL);

#undef KPP_PAD_CTL_ROW
#undef KPP_PAD_CTL_COL
}
EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Inactivate Keypad
 */
void gpio_keypad_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_ROW3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_COL0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_COL1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_COL2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_KPP_COL3), NULL);

	mxc_free_iomux(MX25_PIN_KPP_ROW0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_ROW1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_ROW2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_ROW3, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_COL0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_COL1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_COL2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_KPP_COL3, MUX_CONFIG_GPIO);
}
EXPORT_SYMBOL(gpio_keypad_inactive);

/*!
 * Activate FEC
 */
void gpio_fec_active(void)
{
	mxc_request_iomux(MX25_PIN_FEC_TX_CLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_RX_DV, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_RDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_TDATA0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_TX_EN, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_MDC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_MDIO, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_RDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_FEC_TDATA1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_POWER_FAIL, MUX_CONFIG_FUNC); /* PHY INT */

#define FEC_PAD_CTL1 (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PUE_PUD | \
		      PAD_CTL_PKE_ENABLE)
#define FEC_PAD_CTL2 (PAD_CTL_PUE_PUD)

	mxc_iomux_set_pad(MX25_PIN_FEC_TX_CLK, FEC_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_FEC_RX_DV, FEC_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_FEC_RDATA0, FEC_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_FEC_TDATA0, FEC_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_FEC_TX_EN, FEC_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_FEC_MDC, FEC_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_FEC_MDIO, FEC_PAD_CTL1 | PAD_CTL_22K_PU);
	mxc_iomux_set_pad(MX25_PIN_FEC_RDATA1, FEC_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_FEC_TDATA1, FEC_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_POWER_FAIL, FEC_PAD_CTL1);

	/*
	 * Set up the FEC_RESET_B and FEC_ENABLE GPIO pins.
	 * Assert FEC_RESET_B, then power up the PHY by asserting
	 * FEC_ENABLE, at the same time lifting FEC_RESET_B.
	 *
	 * FEC_RESET_B: gpio2[3] is ALT 5 mode of pin D12
	 * FEC_ENABLE_B: gpio4[8] is ALT 5 mode of pin A17
	 */
	mxc_request_iomux(MX25_PIN_A17, MUX_CONFIG_ALT5); /* FEC_EN */
	mxc_request_iomux(MX25_PIN_D12, MUX_CONFIG_ALT5); /* FEC_RESET_B */

	mxc_iomux_set_pad(MX25_PIN_A17, PAD_CTL_ODE_OpenDrain);
	mxc_iomux_set_pad(MX25_PIN_D12, 0);

	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A17), "a17");
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_D12), "d12");

	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_A17), 0); /* FEC_EN */
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_D12), 0); /* FEC_RESET_B */

	/* drop PHY power */
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A17), 0);	/* FEC_EN */

	/* assert reset */
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_D12), 0);	/* FEC_RESET_B */
	udelay(2);		/* spec says 1us min */

	/* turn on PHY power and lift reset */
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A17), 1);	/* FEC_EN */
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_D12), 1);	/* FEC_RESET_B */

#undef FEC_PAD_CTL_COMMON
#undef FEC_PAD_CTL1
#undef FEC_PAD_CTL2
}
EXPORT_SYMBOL(gpio_fec_active);

/*!
 * Inactivate FEC
 */
void gpio_fec_inactive(void)
{
	/*
	 * Turn off the PHY.
	 */
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A17), 0);

	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_TX_CLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_RX_DV), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_RDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_TDATA0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_TX_EN), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_MDC), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_MDIO), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_RDATA1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_TDATA1), NULL);

	mxc_free_iomux(MX25_PIN_FEC_TX_CLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_RDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_TDATA0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_TX_EN, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_MDC, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_MDIO, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_RDATA1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_FEC_TDATA1, MUX_CONFIG_GPIO);
	mxc_request_iomux(MX25_PIN_POWER_FAIL, MUX_CONFIG_FUNC); /* PHY INT */

	mxc_free_iomux(MX25_PIN_A17, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_D12, MUX_CONFIG_GPIO); /* FEC_RESET_B */

	/* We keep pin A17, so FEC_ENABLE doesn't float */
}
EXPORT_SYMBOL(gpio_fec_inactive);

/*!
 * Activate an I2C device
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
#define I2C_PAD_CTL (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | \
		     PAD_CTL_PUE_PUD | PAD_CTL_100K_PU | PAD_CTL_ODE_OpenDrain)

	switch (i2c_num) {
	case 0:
		/*I2C1*/
		mxc_request_iomux(MX25_PIN_I2C1_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX25_PIN_I2C1_DAT, MUX_CONFIG_SION);
		mxc_iomux_set_pad(MX25_PIN_I2C1_CLK, I2C_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_I2C1_DAT, I2C_PAD_CTL);
		break;
	case 1:
		/*I2C2*/
		mxc_request_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_ALT2); /*SCL*/
		mxc_request_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_ALT2); /*SDA*/
		mxc_iomux_set_pad(MX25_PIN_GPIO_C, I2C_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_GPIO_D, I2C_PAD_CTL);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SCL_IN, INPUT_CTL_PATH1);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SDA_IN, INPUT_CTL_PATH1);

#if 0
		/* Or use FEC pins if it is not used */
		mxc_request_iomux(MX25_PIN_FEC_RDATA1, MUX_CONFIG_ALT1); /*SCL*/
		mxc_request_iomux(MX25_PIN_FEC_RX_DV, MUX_CONFIG_ALT1); /*SDA*/
		mxc_iomux_set_pad(MX25_PIN_FEC_RDATA1, I2C_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_FEC_RX_DV, I2C_PAD_CTL);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SCL_IN, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_I2C2_IPP_SDA_IN, INPUT_CTL_PATH0);
#endif

		break;
	case 2:
		/*I2C3*/
		mxc_request_iomux(MX25_PIN_HSYNC, MUX_CONFIG_ALT2); /*SCL*/
		mxc_request_iomux(MX25_PIN_VSYNC, MUX_CONFIG_ALT2); /*SDA*/
		mxc_iomux_set_pad(MX25_PIN_HSYNC, I2C_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_VSYNC, I2C_PAD_CTL);
		mxc_iomux_set_input(MUX_IN_I2C3_IPP_SCL_IN, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_I2C3_IPP_SDA_IN, INPUT_CTL_PATH0);
		break;
	default:
		break;
	}
#undef I2C_PAD_CTL
}
EXPORT_SYMBOL(gpio_i2c_active);

/*!
 * Inactivate an I2C device
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		/*I2C1*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_I2C1_CLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_I2C1_DAT), NULL);
		mxc_free_iomux(MX25_PIN_I2C1_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_I2C1_DAT, MUX_CONFIG_GPIO);
		break;
	case 1:
		/*I2C2*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_C), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_D), NULL);
		mxc_free_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_GPIO);

#if 0
		/* Or use FEC pins if not in use */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_RDATA1, NULL); /*SCL*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_FEC_RX_DV, NULL); /*SDA*/
		mxc_free_iomux(MX25_PIN_FEC_RDATA1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);
#endif

		break;
	case 2:
		/*I2C3*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_HSYNC), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_VSYNC), NULL);
		mxc_free_iomux(MX25_PIN_HSYNC, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_VSYNC, MUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_i2c_inactive);

/*!
 * Activate a CSPI device
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
#define SPI_PAD_CTL1 (PAD_CTL_HYS_SCHMITZ|PAD_CTL_PKE_ENABLE| \
		      PAD_CTL_100K_PU)
#define SPI_PAD_CTL2 (PAD_CTL_HYS_SCHMITZ|PAD_CTL_PKE_ENABLE| \
		      PAD_CTL_PUE_PUD|PAD_CTL_100K_PU)

	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_request_iomux(MX25_PIN_CSPI1_MOSI, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_CSPI1_MISO, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_CSPI1_SS0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_CSPI1_SS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_CSPI1_SCLK, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_CSPI1_RDY, MUX_CONFIG_FUNC);
#ifndef CONFIG_CAN_FLEXCAN	/* MX25 3-stack uses this pin for CAN2 */
		mxc_request_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_ALT5); /*SS2*/
#endif
		mxc_request_iomux(MX25_PIN_VSTBY_ACK, MUX_CONFIG_ALT2); /*SS3*/

		/* Or if VSTBY_ACK is being used */
		/*mxc_request_iomux(MX25_PIN_NF_CE0, MUX_CONFIG_ALT1);*/ /*SS3*/

		mxc_iomux_set_pad(MX25_PIN_CSPI1_MOSI, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CSPI1_MISO, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CSPI1_SS0, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CSPI1_SS1, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CSPI1_SCLK, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CSPI1_RDY, SPI_PAD_CTL1);
#ifndef CONFIG_CAN_FLEXCAN	/* MX25 3-stack uses this pin for CAN2 */
		mxc_iomux_set_pad(MX25_PIN_GPIO_C, SPI_PAD_CTL2);
#endif
		mxc_iomux_set_pad(MX25_PIN_VSTBY_ACK, SPI_PAD_CTL1);

		mxc_iomux_set_input(MUX_IN_CSPI1_IPP_IND_SS3_B,
				    INPUT_CTL_PATH1);
		break;
	case 1:
		/* SPI2 */
		mxc_request_iomux(MX25_PIN_LD12, MUX_CONFIG_ALT2); /*MOSI*/
		mxc_request_iomux(MX25_PIN_LD13, MUX_CONFIG_ALT2); /*MISO*/
		mxc_request_iomux(MX25_PIN_LD14, MUX_CONFIG_ALT2); /*SCLK*/
		mxc_request_iomux(MX25_PIN_LD15, MUX_CONFIG_ALT2); /*RDY*/
		mxc_request_iomux(MX25_PIN_OE_ACD, MUX_CONFIG_ALT2); /*SS0*/
		mxc_request_iomux(MX25_PIN_CONTRAST, MUX_CONFIG_ALT2); /*SS1*/
		mxc_request_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_ALT7); /*SS2*/
		mxc_request_iomux(MX25_PIN_UART2_RTS, MUX_CONFIG_ALT6); /*SS3*/

		mxc_iomux_set_pad(MX25_PIN_LD12, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_LD13, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_LD14, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_LD15, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_OE_ACD, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_CONTRAST, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_GPIO_C, SPI_PAD_CTL2);
		mxc_iomux_set_pad(MX25_PIN_UART2_RTS, SPI_PAD_CTL2);

		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_CSPI_CLK_IN,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_IND_DATAREADY_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_IND_MISO, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_IND_MOSI, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_IND_SS0_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI2_IPP_IND_SS1_B,
				    INPUT_CTL_PATH0);
		break;
	case 2:
		/* SPI3 */
		mxc_request_iomux(MX25_PIN_EB0, MUX_CONFIG_ALT6); /*SS0*/
		mxc_request_iomux(MX25_PIN_EB1, MUX_CONFIG_ALT6); /*SS1*/
		mxc_request_iomux(MX25_PIN_CS4, MUX_CONFIG_ALT6); /*MOSI*/
		mxc_request_iomux(MX25_PIN_CS5, MUX_CONFIG_ALT6); /*MISO*/
		mxc_request_iomux(MX25_PIN_ECB, MUX_CONFIG_ALT6); /*SCLK*/
		mxc_request_iomux(MX25_PIN_LBA, MUX_CONFIG_ALT6); /*RDY*/
		mxc_request_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_ALT7); /*SS2*/
		mxc_request_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_ALT7); /*SS3*/

		mxc_iomux_set_pad(MX25_PIN_EB0, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_EB1, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CS4, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_CS5, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_ECB, SPI_PAD_CTL1);
		mxc_iomux_set_pad(MX25_PIN_LBA, SPI_PAD_CTL1);

		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_CSPI_CLK_IN,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_DATAREADY_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_MISO, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_MOSI, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_SS0_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_SS1_B,
				    INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_SS2_B,
				    INPUT_CTL_PATH1);
		mxc_iomux_set_input(MUX_IN_CSPI3_IPP_IND_SS3_B,
				    INPUT_CTL_PATH0);
		break;
	default:
		break;
	}
#undef SPI_PAD_CTL1
#undef SPI_PAD_CTL2
}
EXPORT_SYMBOL(gpio_spi_active);

/*!
 * Inactivate a CSPI device
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_MOSI), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_MISO), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_SS0), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_SS1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_SCLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSPI1_RDY), NULL);
#ifndef CONFIG_CAN_FLEXCAN	/* MX25 3-stack uses this pin for CAN2 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_C), NULL); /*SS2*/
#endif
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_VSTBY_ACK), NULL); /*SS3*/

		mxc_free_iomux(MX25_PIN_CSPI1_MOSI, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSPI1_MISO, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSPI1_SS0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSPI1_SS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSPI1_SCLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSPI1_RDY, MUX_CONFIG_GPIO);
#ifndef CONFIG_CAN_FLEXCAN	/* MX25 3-stack uses this pin for CAN2 */
		mxc_free_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_GPIO);
#endif
		mxc_free_iomux(MX25_PIN_VSTBY_ACK, MUX_CONFIG_GPIO);
		break;
	case 1:
		/* SPI2 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD12), NULL); /*MOSI*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD13), NULL); /*MISO*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD14), NULL); /*SCLK*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD15), NULL); /*RDY*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_OE_ACD), NULL); /*SS0*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CONTRAST), NULL); /*SS1*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_C), NULL); /*SS2*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_UART2_RTS), NULL); /*SS3*/

		mxc_free_iomux(MX25_PIN_LD12, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD13, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD14, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD15, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_OE_ACD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CONTRAST, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_UART2_RTS, MUX_CONFIG_GPIO);
		break;
	case 2:
		/* SPI3 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_EB0), NULL); /*SS0*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_EB1), NULL); /*SS1*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CS4), NULL); /*MOSI*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CS5), NULL); /*MISO*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_ECB), NULL); /*SCLK*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LBA), NULL); /*RDY*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_D), NULL); /*SS2*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D9), NULL); /*SS3*/

		mxc_free_iomux(MX25_PIN_EB0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_EB1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CS4, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CS5, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_ECB, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LBA, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_spi_inactive);

/*!
 * Activate LCD
 */
void gpio_lcdc_active(void)
{
	mxc_request_iomux(MX25_PIN_LD0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD6, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD7, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD8, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD9, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD10, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD11, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD12, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD13, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD14, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LD15, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_GPIO_E, MUX_CONFIG_ALT2); /*D16*/
	mxc_request_iomux(MX25_PIN_GPIO_F, MUX_CONFIG_ALT2); /*D17*/
	mxc_request_iomux(MX25_PIN_HSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_VSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_LSCLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_OE_ACD, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CONTRAST, MUX_CONFIG_FUNC);

#define LCD_PAD_CTL (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_100K_PU)
	mxc_iomux_set_pad(MX25_PIN_LD0, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD1, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD2, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD3, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD4, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD5, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD6, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD7, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD8, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD9, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD10, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD11, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD12, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD13, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD14, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LD15, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_GPIO_E, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_GPIO_F, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_HSYNC, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_VSYNC, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_LSCLK, LCD_PAD_CTL | PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_OE_ACD, LCD_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CONTRAST, LCD_PAD_CTL);
}
EXPORT_SYMBOL(gpio_lcdc_active);

/*!
 * Inactivate LCD
 */
void gpio_lcdc_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD0), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD1), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD4), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD5), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD6), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD7), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_E), NULL); /*D16*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_F), NULL); /*D17*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_HSYNC), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_VSYNC), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_LSCLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_OE_ACD), NULL);

	mxc_free_iomux(MX25_PIN_LD0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD3, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD4, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD5, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD6, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD7, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LD8, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD9, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD10, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD11, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD12, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD13, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD14, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_LD15, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_GPIO_E, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_GPIO_F, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_HSYNC, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_VSYNC, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_LSCLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_OE_ACD, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CONTRAST, MUX_CONFIG_FUNC);
}
EXPORT_SYMBOL(gpio_lcdc_inactive);

/*!
 * Activate SDHC
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
#define SDHC_PAD_CTL (PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | \
		      PAD_CTL_47K_PU | PAD_CTL_SRE_FAST)

	switch (module) {
	case 0:
		/* SDHC1 */
		mxc_request_iomux(MX25_PIN_SD1_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX25_PIN_SD1_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX25_PIN_SD1_DATA0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_SD1_DATA1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_SD1_DATA2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_SD1_DATA3, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX25_PIN_A14, MUX_CONFIG_ALT5); /*SD1_WP*/
		mxc_request_iomux(MX25_PIN_A15, MUX_CONFIG_ALT5); /*SD1_DET*/

		mxc_iomux_set_pad(MX25_PIN_SD1_CMD, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_SD1_CLK, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_SD1_DATA0, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_SD1_DATA1, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_SD1_DATA2, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_SD1_DATA3, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_A14, PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX25_PIN_A15, PAD_CTL_DRV_NORMAL);

		/* Set write protect and card detect gpio as inputs */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_A14), "a14");
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_A15), "a15");
		gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_A14)); /*SD1_WP*/
		gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_A15)); /*SD1_DET*/

		break;
	case 1:
		/* SDHC2 */
		mxc_request_iomux(MX25_PIN_LD8,
				  MUX_CONFIG_ALT6 | MUX_CONFIG_SION); /*CMD*/
		mxc_request_iomux(MX25_PIN_LD9,
				  MUX_CONFIG_ALT6 | MUX_CONFIG_SION); /*CLK*/
		mxc_request_iomux(MX25_PIN_LD10, MUX_CONFIG_ALT6); /*DAT0*/
		mxc_request_iomux(MX25_PIN_LD11, MUX_CONFIG_ALT6); /*DAT1*/
		mxc_request_iomux(MX25_PIN_LD12, MUX_CONFIG_ALT6); /*DAT2*/
		mxc_request_iomux(MX25_PIN_LD13, MUX_CONFIG_ALT6); /*DAT3*/

		mxc_iomux_set_pad(MX25_PIN_LD8, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_LD9, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_LD10, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_LD11, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_LD12, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_LD13, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_CSI_D2, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_CSI_D3, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_CSI_D4, SDHC_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_CSI_D5, SDHC_PAD_CTL);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Inactivate SDHC
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
		/* SDHC1 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_CMD), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_CLK), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_DATA0), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_DATA1), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_DATA2), NULL);
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_SD1_DATA3), NULL);

		mxc_free_iomux(MX25_PIN_SD1_CMD, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_SD1_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_SD1_DATA0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_SD1_DATA1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_SD1_DATA2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_SD1_DATA3, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_A14, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_A15, MUX_CONFIG_GPIO);
		break;
	case 1:
		/* SDHC2 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD8), NULL); /*CMD*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD9), NULL); /*CLK*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD10), NULL); /*DAT0*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD11), NULL); /*DAT1*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD12), NULL); /*DAT2*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_LD13), NULL); /*DAT3*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D2), NULL); /*DAT4*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D3), NULL); /*DAT5*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D4), NULL); /*DAT6*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D5), NULL); /*DAT7*/

		mxc_free_iomux(MX25_PIN_LD8, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD9, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD10, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD11, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD12, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_LD13, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_GPIO);
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
	unsigned int ret = 0;

	ret = gpio_get_value(IOMUX_TO_GPIO(MX25_PIN_A15));
	return ret;
}
EXPORT_SYMBOL(sdhc_get_card_det_status);

/*!
 * Get pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned int rc = 0;

	rc = gpio_get_value(IOMUX_TO_GPIO(MX25_PIN_A14));
	return rc;
}
EXPORT_SYMBOL(sdhc_write_protect);

/*
 *  USB Host2
 *
 *  This configuration uses the on-chip FS/LS serial transceiver.
 *  USBPHY2_{DP,DM} pins are not muxed.
 *  We just need to grab USBH2_PWR, USBH2_OC and the Bluetooth/USB
 *  mux control signal.
 */
int gpio_usbh2_active(void)
{
	if (mxc_request_iomux(MX25_PIN_D9, MUX_CONFIG_ALT6)  ||	/* PWR */
	    mxc_request_iomux(MX25_PIN_D8, MUX_CONFIG_ALT6)  ||	/* OC */
	    mxc_request_iomux(MX25_PIN_A21, MUX_CONFIG_ALT5)) {	/* BT_USB_CS */
		return -EINVAL;
	}

	/*
	 * This pin controls the mux that switches between
	 * the J18 connector and the on-board bluetooth module.
	 *  dir: 0 = out
	 *  pin: 0 = J18, 1 = BT
	 */
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A21), "a21");
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_A21), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A21), 0);

	return 0;
}
EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_inactive(void)
{
	mxc_free_iomux(MX25_PIN_D9, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_D8, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_A21, MUX_CONFIG_GPIO);
}

/*
 *  USB OTG UTMI
 *
 *  This configuration uses the on-chip UTMI transceiver.
 *  USBPHY1_{VBUS,DP,DM,UID,RREF} pins are not muxed.
 *  We just need to grab the USBOTG_PWR and USBOTG_OC pins.
 */
int gpio_usbotg_utmi_active(void)
{
	if (mxc_request_iomux(MX25_PIN_GPIO_A, MUX_CONFIG_ALT2)  || /* PWR */
	    mxc_request_iomux(MX25_PIN_GPIO_B, MUX_CONFIG_ALT2)) {  /* OC */
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(gpio_usbotg_utmi_active);

void gpio_usbotg_utmi_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_A), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_B), NULL);

	mxc_free_iomux(MX25_PIN_GPIO_A, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_GPIO_B, MUX_CONFIG_GPIO);
}
EXPORT_SYMBOL(gpio_usbotg_utmi_inactive);

/*!
 * Activate camera sensor
 */
void gpio_sensor_active(void)
{
	mxc_request_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D6, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D7, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D8, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_HSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_MCLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_PIXCLK, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_CSI_VSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX25_PIN_A19, MUX_CONFIG_ALT5); /*CSI_PWDN*/
	mxc_request_iomux(MX25_PIN_A20, MUX_CONFIG_ALT5); /*CMOS_RST*/

	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A19), "a19");
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A20), "a20");
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_A19), 0); /*CSI_PWDN*/
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A19), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_A20), 0); /*CMOS_RST*/
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A20), 0);
	mdelay(20);
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_A20), 1);

#define CSI_PAD_CTL1 (PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU)
#define CSI_PAD_CTL2 (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | \
		      PAD_CTL_100K_PU)

	mxc_iomux_set_pad(MX25_PIN_CSI_D2, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_D3, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_D4, CSI_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_CSI_D5, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_D6, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_D7, CSI_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_CSI_D8, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_D9, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_HSYNC, CSI_PAD_CTL1);
	mxc_iomux_set_pad(MX25_PIN_CSI_MCLK, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU | PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_CSI_PIXCLK, CSI_PAD_CTL2);
	mxc_iomux_set_pad(MX25_PIN_CSI_VSYNC, CSI_PAD_CTL1);
}
EXPORT_SYMBOL(gpio_sensor_active);

/*!
 * Inactivate camera sensor
 */
void gpio_sensor_inactive(void)
{
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D2), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D3), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D4), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D5), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D6), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D7), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D8), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_D9), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_HSYNC), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_MCLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_PIXCLK), NULL);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_CSI_VSYNC), NULL);

	mxc_free_iomux(MX25_PIN_A19, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_A20, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D6, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D7, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D8, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_HSYNC, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_MCLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_PIXCLK, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_CSI_VSYNC, MUX_CONFIG_GPIO);
}
EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Activate ESAI ports to enable surround sound I/O
 */
void gpio_activate_esai_ports(void)
{
	mxc_request_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_ALT3); /*SCKR*/
	mxc_request_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_ALT3); /*FSR*/
	mxc_request_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_ALT3); /*HCKR*/
	mxc_request_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_ALT3); /*SCKT*/
	mxc_request_iomux(MX25_PIN_CSI_D6, MUX_CONFIG_ALT3); /*FST*/
	mxc_request_iomux(MX25_PIN_CSI_D7, MUX_CONFIG_ALT3); /*HCKT*/
	mxc_request_iomux(MX25_PIN_CSI_D8, MUX_CONFIG_ALT3); /*TX5_RX0*/
	mxc_request_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_ALT3); /*TX4_RX1*/
	mxc_request_iomux(MX25_PIN_CSI_MCLK, MUX_CONFIG_ALT3); /*TX3_RX2*/
	mxc_request_iomux(MX25_PIN_CSI_VSYNC, MUX_CONFIG_ALT3); /*TX2_RX3*/
	mxc_request_iomux(MX25_PIN_CSI_HSYNC, MUX_CONFIG_ALT3); /*TX1*/
	mxc_request_iomux(MX25_PIN_CSI_PIXCLK, MUX_CONFIG_ALT3); /*TX0*/

#define ESAI_PAD_CTL (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | \
		      PAD_CTL_100K_PU | PAD_CTL_PUE_PUD)
	mxc_iomux_set_pad(MX25_PIN_CSI_D2, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D3, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D4, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D5, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D6, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D7, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D8, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_D9, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_MCLK, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_VSYNC, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_HSYNC, ESAI_PAD_CTL);
	mxc_iomux_set_pad(MX25_PIN_CSI_PIXCLK, ESAI_PAD_CTL);

#undef ESAI_PAD_CTL
}
EXPORT_SYMBOL(gpio_activate_esai_ports);

/*!
 * Inactivate ESAI ports to disable surround sound I/O
 */
void gpio_deactivate_esai_ports(void)
{
	mxc_free_iomux(MX25_PIN_CSI_D2, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D3, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D4, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D5, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D6, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D7, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D8, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_D9, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_MCLK, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_VSYNC, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_HSYNC, MUX_CONFIG_FUNC);
	mxc_free_iomux(MX25_PIN_CSI_PIXCLK, MUX_CONFIG_FUNC);
}
EXPORT_SYMBOL(gpio_deactivate_esai_ports);


/*!
 * Activate CAN
 */
void gpio_can_active(int id)
{
#define CAN_PAD_CTL (PAD_CTL_DRV_3_3V | PAD_CTL_PKE_NONE | PAD_CTL_ODE_CMOS | \
		     PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW)
#define CAN_PAD_IN_CTL (PAD_CTL_HYS_CMOS | PAD_CTL_PKE_NONE)

	switch (id) {
	case 0:
		/* CAN1 */
		mxc_request_iomux(MX25_PIN_GPIO_A, MUX_CONFIG_ALT6); /*TXCAN*/
		mxc_request_iomux(MX25_PIN_GPIO_B, MUX_CONFIG_ALT6); /*RXCAN*/

		mxc_iomux_set_pad(MX25_PIN_GPIO_A, CAN_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_GPIO_B, CAN_PAD_IN_CTL);

		mxc_iomux_set_input(MUX_IN_CAN1_IPP_IND_CANRX, INPUT_CTL_PATH1);
		break;
	case 1:
		/* CAN2 */
		mxc_request_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_ALT6); /*TXCAN*/
		mxc_request_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_ALT6); /*RXCAN*/
		mxc_request_iomux(MX25_PIN_D14, MUX_CONFIG_ALT5); /*PWDN*/

		mxc_iomux_set_pad(MX25_PIN_GPIO_C, CAN_PAD_CTL);
		mxc_iomux_set_pad(MX25_PIN_GPIO_D, CAN_PAD_IN_CTL);
		mxc_iomux_set_pad(MX25_PIN_D14, CAN_PAD_CTL);

		mxc_iomux_set_input(MUX_IN_CAN2_IPP_IND_CANRX, INPUT_CTL_PATH1);

		/* Configure CAN_PWDN as output */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_D14), "d14");
		gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_D14), 0);

		/* Enable input by setting PWDN/TLE6250.INH low (gpio4 bit6) */
		gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_D14), 0);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_can_active);

/*!
 * Inactivate CAN
 */
void gpio_can_inactive(int id)
{
	switch (id) {
	case 0:
		/* CAN1 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_A), NULL); /*TXCAN*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_B), NULL); /*RXCAN*/

		mxc_free_iomux(MX25_PIN_GPIO_A, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX25_PIN_GPIO_B, MUX_CONFIG_FUNC);

		break;
	case 1:
		/* CAN2 */
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_C), NULL); /*TXCAN*/
		gpio_request(IOMUX_TO_GPIO(MX25_PIN_GPIO_D), NULL); /*RXCAN*/

		mxc_free_iomux(MX25_PIN_GPIO_C, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX25_PIN_GPIO_D, MUX_CONFIG_FUNC);

		/* Disable input by setting PWDN/TLE6250.INH high */
		gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_D14), 1);
		mxc_free_iomux(MX25_PIN_D14, MUX_CONFIG_ALT5);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(gpio_can_inactive);

/*!
 * This function activates DAM port 4 to enable
 * audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	mxc_request_iomux(MX25_PIN_EB0, MUX_CONFIG_ALT4); /*SSI4_STXD*/
	mxc_request_iomux(MX25_PIN_EB1, MUX_CONFIG_ALT4); /*SSI4_SRXD*/
	mxc_request_iomux(MX25_PIN_RW, MUX_CONFIG_ALT4); /*SSI4_STXFS*/
	mxc_request_iomux(MX25_PIN_OE, MUX_CONFIG_ALT4); /*SSI4_SCK*/
	mxc_request_iomux(MX25_PIN_A10, MUX_CONFIG_ALT5); /*HP_DEC*/
	mxc_request_iomux(MX25_PIN_D13, MUX_CONFIG_ALT5); /*AMP_SHUTDOWN*/

	mxc_iomux_set_pad(MX25_PIN_EB0, PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_EB1, PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_RW, PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_OE, PAD_CTL_SRE_FAST);
	mxc_iomux_set_pad(MX25_PIN_D13, PAD_CTL_DRV_3_3V);

	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A10), "a10");
	gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_A10));
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_D13), "d13");
	gpio_direction_output(IOMUX_TO_GPIO(MX25_PIN_D13), 0);
}
EXPORT_SYMBOL(gpio_activate_audio_ports);

/*!
 * This function deactivates DAM port 4 for
 * audio I/O
 */
void gpio_deactive_audio_ports(void)
{
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_EB0), NULL); /*SSI4_STXD*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_EB1), NULL); /*SSI4_SRXD*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_RW), NULL); /*SSI4_STXFS*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_OE), NULL); /*SSI4_SCK*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_A10), NULL); /*HP_DEC*/
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_D13), NULL); /*AMP_SHUTDOWN*/

	mxc_free_iomux(MX25_PIN_EB0, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_EB1, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_RW, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_OE, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_A10, MUX_CONFIG_GPIO);
	mxc_free_iomux(MX25_PIN_D13, MUX_CONFIG_GPIO);
}
EXPORT_SYMBOL(gpio_deactive_audio_ports);

int headphone_det_status(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX25_PIN_A10));
}
EXPORT_SYMBOL(headphone_det_status);

void sgtl5000_enable_amp(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX25_PIN_D13), 1);
}
EXPORT_SYMBOL(sgtl5000_enable_amp);
