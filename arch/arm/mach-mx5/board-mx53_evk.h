/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MXC_BOARD_MX53_EVK_H__
#define __ASM_ARCH_MXC_BOARD_MX53_EVK_H__

/*!
 * @defgroup BRDCFG_MX53 Board Configuration Options
 * @ingroup MSL_MX53
 */

/*!
 * @file mach-mx53/board-mx53_evk.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX53 EVK Platform.
 *
 * @ingroup BRDCFG_MX53
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
/* UART 4 configuration */
#define UART4_MODE		MODE_DCE
#define UART4_IR		NO_IRDA
#define UART4_ENABLED		1
/* UART 5 configuration */
#define UART5_MODE		MODE_DCE
#define UART5_IR		NO_IRDA
#define UART5_ENABLED		1

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

extern int __init mx53_evk_init_mc13892(void);

#endif				/* __ASM_ARCH_MXC_BOARD_MX53_EVK_H__ */
