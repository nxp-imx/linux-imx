/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_MX25_3STACK_H__
#define __ASM_ARCH_MXC_BOARD_MX25_3STACK_H__

#ifdef CONFIG_MACH_MX25_3DS

/*!
 * @defgroup BRDCFG_MX25 Board Configuration Options
 * @ingroup MSL_MX25
 */

/*!
 * @file mach-mx25/board-mx25_3stack.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX25 3STACK Platform.
 *
 * @ingroup BRDCFG_MX25
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>

/*!
 * @name MXC UART board-level configurations
 */
/*! @{ */
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
#define UART3_ENABLED           0

/* UART 4 configuration */
#define UART4_MODE              MODE_DTE
#define UART4_IR                NO_IRDA
#define UART4_ENABLED           0

/* UART 5 configuration */
#define UART5_MODE              MODE_DTE
#define UART5_IR                NO_IRDA
#define UART5_ENABLED           0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

/*!
 * @name debug board parameters
 */
/*! @{ */
/*!
 * Base address of debug board
 */
#define DEBUG_BASE_ADDRESS      0x78000000	/* Use a dummy base address */

/* External ethernet LAN9217 base address */
#define LAN9217_BASE_ADDR       DEBUG_BASE_ADDRESS

/* External UART */
#define UARTA_BASE_ADDR     	(DEBUG_BASE_ADDRESS + 0x08000)
#define UARTB_BASE_ADDR     	(DEBUG_BASE_ADDRESS + 0x10000)

#define BOARD_IO_ADDR		0x20000

/* LED switchs */
#define LED_SWITCH_REG        	(BOARD_IO_ADDR + 0x00)
/* buttons */
#define SWITCH_BUTTON_REG     	(BOARD_IO_ADDR + 0x08)
/* status, interrupt */
#define INTR_STATUS_REG       	(BOARD_IO_ADDR + 0x10)
#define INTR_RESET_REG        	(BOARD_IO_ADDR + 0x20)
/*CPLD configuration*/
#define CONFIG1_REG           	(BOARD_IO_ADDR + 0x28)
#define CONFIG2_REG           	(BOARD_IO_ADDR + 0x30)
/*interrupt mask */
#define INTR_MASK_REG        	(BOARD_IO_ADDR + 0x38)

/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG     	(BOARD_IO_ADDR + 0x40)
#define	MAGIC_NUMBER2_REG     	(BOARD_IO_ADDR + 0x48)
/* CPLD code version */
#define CPLD_CODE_VER_REG       (BOARD_IO_ADDR + 0x50)
/* magic word for debug CPLD */
#define MAGIC3_NUMBER3_REG     	(BOARD_IO_ADDR + 0x58)
/* module reset register*/
#define CONTROL_REG      	(BOARD_IO_ADDR + 0x60)
/* CPU ID and Personality ID*/
#define IDENT_REG         	(BOARD_IO_ADDR + 0x68)

/* For interrupts like xuart, enet etc */
#define EXPIO_PARENT_INT        MX25_PIN_GPIO1_1

#define EXPIO_INT_ENET_INT          (MXC_BOARD_IRQ_START + 0)
#define EXPIO_INT_XUARTA_INT        (MXC_BOARD_IRQ_START + 1)
#define EXPIO_INT_XUARTB_INT        (MXC_BOARD_IRQ_START + 2)

/*! This is System IRQ used by LAN9217 for interrupt generation taken
 * from platform.h
 */
#define LAN9217_IRQ              EXPIO_INT_ENET_INT

/*! This is base virtual address of debug board*/
extern unsigned int mx25_3stack_board_io;

#define MXC_BD_LED1             (1 << 0)
#define MXC_BD_LED2             (1 << 1)
#define MXC_BD_LED3             (1 << 2)
#define MXC_BD_LED4             (1 << 3)
#define MXC_BD_LED5             (1 << 4)
#define MXC_BD_LED6             (1 << 5)
#define MXC_BD_LED7             (1 << 6)
#define MXC_BD_LED8             (1 << 7)
#define MXC_BD_LED_ON(led)
#define MXC_BD_LED_OFF(led)

#define MXC_DEFAULT_INTENSITY	127
#define MXC_INTENSITY_OFF	0

/*! @} */

extern void mx25_3stack_gpio_init(void) __init;
extern int headphone_det_status(void);
extern void sgtl5000_enable_amp(void);
extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_write_protect(struct device *dev);
extern void gpio_can_active(int id);
extern void gpio_can_inactive(int id);
extern struct flexcan_platform_data flexcan_data[];
extern void mx2fb_set_brightness(uint8_t);
extern int __init mx25_3stack_init_mc34704(void);
extern void imx_adc_set_hsync(int on);

#endif				/* CONFIG_MACH_MX25_3DS */
#endif				/* __ASM_ARCH_MXC_BOARD_MX25_3STACK_H__ */
