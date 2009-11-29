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

#ifndef __ASM_ARCH_MXC_BOARD_MX37_3STACK_H__
#define __ASM_ARCH_MXC_BOARD_MX37_3STACK_H__

/*!
 * @defgroup BRDCFG_MX37 Board Configuration Options
 * @ingroup MSL_MX37
 */

/*!
 * @file mach-mx37/board-mx37_3stack.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX31 ADS Platform.
 *
 * @ingroup BRDCFG_MX37
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>
#include <mach/mxc_dptc.h>

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
#define UART3_MODE              MODE_DCE
#define UART3_IR                NO_IRDA
#define UART3_ENABLED           1

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#define DEBUG_BASE_ADDRESS	0x78000000	/* Use a Dummy base address */
/* LAN9217 ethernet base address */
#define LAN9217_BASE_ADDR	DEBUG_BASE_ADDRESS
/* External UART */
#define UARTA_BASE_ADDR		(DEBUG_BASE_ADDRESS + 0x8000)
#define UARTB_BASE_ADDR		(DEBUG_BASE_ADDRESS + 0x10000)

#define BOARD_IO_ADDR		0x20000
/* LED switchs */
#define LED_SWITCH_REG		BOARD_IO_ADDR + 0x00
/* buttons */
#define SWITCH_BUTTONS_REG	BOARD_IO_ADDR + 0x08
/* status, interrupt */
#define INTR_STATUS_REG		BOARD_IO_ADDR + 0x10
#define INTR_MASK_REG		BOARD_IO_ADDR + 0x38
#define INTR_RESET_REG		BOARD_IO_ADDR + 0x20
/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG	BOARD_IO_ADDR + 0x40
#define MAGIC_NUMBER2_REG	BOARD_IO_ADDR + 0x48
/* CPLD code version */
#define CPLD_CODE_VER_REG	BOARD_IO_ADDR + 0x50

extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_write_protect(struct device *dev);
extern int sdhc_init_card_det(int id);
extern struct tve_platform_data tve_data;
extern struct mxc_dptc_data dptc_lp_data;
extern struct mxc_dptc_data dptc_gp_data;
extern struct mxc_dvfs_platform_data dvfs_core_data;
extern struct mxc_dvfsper_data dvfs_per_data;
extern char *gp_reg_id;
extern char *lp_reg_id;

extern int headphone_det_status(void);
#endif				/* __ASM_ARCH_MXC_BOARD_MX37_3STACK_H__ */
