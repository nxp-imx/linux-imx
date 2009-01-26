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

#ifndef __ASM_ARCH_MXC_BOARD_MX35EVB_H__
#define __ASM_ARCH_MXC_BOARD_MX35EVB_H__

#ifdef CONFIG_MACH_MX35EVB
/*!
 * @defgroup BRDCFG_MX35 Board Configuration Options
 * @ingroup MSL_MX35
 */

/*!
 * @file mach-mx35/board-mx35evb.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX35 EVB Platform.
 *
 * @ingroup BRDCFG_MX35
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>

/*!
 * @name MXC UART EVB board level configurations
 */
/*! @{ */
/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV         0
/*!
 * Specifies if the Irda receive path is inverting
 */
#define MXC_IRDA_RX_INV         0

/* UART 1 configuration */
/*!
 * This define specifies if the UART port is configured to be in DTE or
 * DCE mode. There exists a define like this for each UART port. Valid
 * values that can be used are \b MODE_DTE or \b MODE_DCE.
 */
#define UART1_MODE              MODE_DCE
/*!
 * This define specifies if the UART is to be used for IRDA. There exists a
 * define like this for each UART port. Valid values that can be used are
 * \b IRDA or \b NO_IRDA.
 */
#define UART1_IR                NO_IRDA
/*!
 * This define is used to enable or disable a particular UART port. If
 * disabled, the UART will not be registered in the file system and the user
 * will not be able to access it. There exists a define like this for each UART
 * port. Specify a value of 1 to enable the UART and 0 to disable it.
 */
#define UART1_ENABLED           1
/*! @} */
/* UART 2 configuration */
#define UART2_MODE              MODE_DCE
#define UART2_IR                NO_IRDA
#define UART2_ENABLED           1

/* UART 3 configuration */
#define UART3_MODE              MODE_DTE
#define UART3_IR                NO_IRDA
#define UART3_ENABLED           1

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

/*!
 * @name PBC Controller parameters
 */
/*!
 * Base address of PBC controller
 */
/*! @{ */
#define PBC_BASE_ADDRESS 	CS4_BASE_ADDR

#define PBC_VERSION		0x0000
#define PBC_BCTRL1_SET		0x0008
#define PBC_BCTRL1_CLR		0x000C
#define PBC_BCTRL2_SET		0x0010
#define PBC_BCTRL2_CLR		0x0014
#define PBC_IMR1_SET		0x0018
#define PBC_IMR1_CLR		0x001C
#define PBC_IMR2_SET		0x0020
#define PBC_IMR2_CLR		0x0024
#define PBC_BSTAT1		0x0028
#define PBC_ISR			0x002C

#define ENET_BASE_ADDRESS    	(PBC_BASE_ADDRESS + 0x20000)

/*! Definitions in Board Control Register 1 */
#define PBC_BCTRL1_ENET_RESET_B	(1)
#define PBC_BCTRL1_FEC_RESET_B	(1<<2)

/*! @} */

#define EXPIO_PARENT_INT_0	IOMUX_TO_IRQ(MX35_PIN_GPIO1_0)
#define EXPIO_PARENT_INT_1	IOMUX_TO_IRQ(MX35_PIN_GPIO1_1)

#define EXPIO_PARENT_INT	EXPIO_PARENT_INT_1
#define EXPIO_PARENT_INT_PIN	MX35_PIN_GPIO1_1

#define EXPIO_INT_FEC		(EXPIO_PARENT_INT + 0)
#define EXPIO_INT_ENET		(EXPIO_PARENT_INT + 1)

extern unsigned int mx35evb_board_io;

#define MXC_BD_LED1             (1)
#define MXC_BD_LED2             (1 << 1)
#define MXC_BD_LED3             (1 << 2)
#define MXC_BD_LED4             (1 << 3)
#define MXC_BD_LED_ON(led)	\
	__raw_writew(led, mx35evb_bard_io + PBC_BCTRL2_SET)
#define MXC_BD_LED_OFF(led)	\
	__raw_writew(led, mx35evb_bard_io + PBC_BCTRL2_CLR)

#define AHB_FREQ                133000000
#define IPG_FREQ                66500000
/*!
 * Specifies if the MXC Write Protectect function is suppport or not.
 */
#define MXC_MMC_WRITE_PROTECT   1

extern void mx35evb_gpio_init(void) __init;

#endif				/* CONFIG_MACH_MX35EVB */
#endif				/* __ASM_ARCH_MXC_BOARD_MX35EVB_H__ */
