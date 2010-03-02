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

#ifndef __ASM_ARCH_MXC_BOARD_MX51_3STACK_H__
#define __ASM_ARCH_MXC_BOARD_MX51_3STACK_H__

/*!
 * @defgroup BRDCFG_MX51 Board Configuration Options
 * @ingroup MSL_MX51
 */

/*!
 * @file mach-mx51/board-mx51_3stack.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX51 3Stack Platform.
 *
 * @ingroup BRDCFG_MX51
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>

/*!
 * @name MXC UART board level configurations
 */
/*! @{ */
/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV	0
/*!
 * Specifies if the Irda receive path is inverting
 */
#define MXC_IRDA_RX_INV	0

/* UART 1 configuration */
/*!
 * This define specifies if the UART port is configured to be in DTE or
 * DCE mode. There exists a define like this for each UART port. Valid
 * values that can be used are \b MODE_DTE or \b MODE_DCE.
 */
#define UART1_MODE		MODE_DCE
/*!
 * This define specifies if the UART is to be used for IRDA. There exists a
 * define like this for each UART port. Valid values that can be used are
 * \b IRDA or \b NO_IRDA.
 */
#define UART1_IR		NO_IRDA
/*!
 * This define is used to enable or disable a particular UART port. If
 * disabled, the UART will not be registered in the file system and the user
 * will not be able to access it. There exists a define like this for each UART
 * port. Specify a value of 1 to enable the UART and 0 to disable it.
 */
#define UART1_ENABLED		1
/*! @} */
/* UART 2 configuration */
#define UART2_MODE		MODE_DCE
#define UART2_IR		NO_IRDA
#define UART2_ENABLED		1
/* UART 3 configuration */
#define UART3_MODE		MODE_DCE
#define UART3_IR		NO_IRDA
#define UART3_ENABLED		1

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#define DEBUG_BOARD_BASE_ADDRESS(n)	(n)
/* LAN9217 ethernet base address */
#define LAN9217_BASE_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n))
/* External UART */
#define UARTA_BASE_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n) + 0x8000)
#define UARTB_BASE_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n) + 0x10000)

#define BOARD_IO_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n) + 0x20000)
/* LED switchs */
#define LED_SWITCH_REG		0x00
/* buttons */
#define SWITCH_BUTTONS_REG	0x08
/* status, interrupt */
#define INTR_STATUS_REG	0x10
#define INTR_MASK_REG		0x38
#define INTR_RESET_REG		0x20
/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG	0x40
#define MAGIC_NUMBER2_REG	0x48
/* CPLD code version */
#define CPLD_CODE_VER_REG	0x50
/* magic word for debug CPLD */
#define MAGIC_NUMBER3_REG	0x58
/* module reset register*/
#define MODULE_RESET_REG	0x60
/* CPU ID and Personality ID */
#define MCU_BOARD_ID_REG	0x68

/* interrupts like external uart , external ethernet etc*/
#define EXPIO_PARENT_INT	IOMUX_TO_IRQ(MX51_PIN_GPIO1_6)

#define EXPIO_INT_ENET		(MXC_BOARD_IRQ_START + 0)
#define EXPIO_INT_XUART_A 	(MXC_BOARD_IRQ_START + 1)
#define EXPIO_INT_XUART_B 	(MXC_BOARD_IRQ_START + 2)
#define EXPIO_INT_BUTTON_A 	(MXC_BOARD_IRQ_START + 3)
#define EXPIO_INT_BUTTON_B 	(MXC_BOARD_IRQ_START + 4)

/*! This is System IRQ used by LAN9217 */
#define LAN9217_IRQ	EXPIO_INT_ENET

extern int __init mx51_3stack_init_mc13892(void);

#endif				/* __ASM_ARCH_MXC_BOARD_MX51_3STACK_H__ */
