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

#ifndef __ASM_ARCH_MXC_BOARD_MX35_3STACK_H__
#define __ASM_ARCH_MXC_BOARD_MX35_3STACK_H__

#ifdef CONFIG_MACH_MX35_3DS

/*!
 * @defgroup BRDCFG_MX35 Board Configuration Options
 * @ingroup MSL_MX35
 */

/*!
 * @file mach-mx35/board-mx35_3stack.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX35 3STACK Platform.
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
#define UART2_MODE              MODE_DTE
#define UART2_IR                NO_IRDA
#define UART2_ENABLED           1

/* UART 3 configuration */
#define UART3_MODE              MODE_DTE
#define UART3_IR                NO_IRDA
#define UART3_ENABLED           1

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#define MXC_PSEUDO_PARENT	MXC_INT_FORCE

enum {
	MCU_INT_HEADPHONE = 0,
	MCU_INT_GPS,
	MCU_INT_SD1_CD,
	MCU_INT_SD1_WP,
	MCU_INT_SD2_CD,
	MCU_INT_SD2_WP,
	MCU_INT_POWER_KEY,
	MCU_INT_RTC,
	MCU_INT_TS_ADC,
	MCU_INT_KEYPAD,
};

#define MXC_PSEUDO_IRQ_HEADPHONE	(MXC_PSEUDO_IO_BASE + MCU_INT_HEADPHONE)
#define MXC_PSEUDO_IRQ_GPS	(MXC_PSEUDO_IO_BASE + MCU_INT_GPS)
#define MXC_PSEUDO_IRQ_SD1_CD	(MXC_PSEUDO_IO_BASE + MCU_INT_SD1_CD)
#define MXC_PSEUDO_IRQ_SD1_WP	(MXC_PSEUDO_IO_BASE + MCU_INT_SD1_WP)
#define MXC_PSEUDO_IRQ_SD2_CD	(MXC_PSEUDO_IO_BASE + MCU_INT_SD2_CD)
#define MXC_PSEUDO_IRQ_SD2_WP	(MXC_PSEUDO_IO_BASE + MCU_INT_SD2_WP)
#define MXC_PSEUDO_IRQ_POWER_KEY	(MXC_PSEUDO_IO_BASE + MCU_INT_POWER_KEY)
#define MXC_PSEUDO_IRQ_KEYPAD	(MXC_PSEUDO_IO_BASE + MCU_INT_KEYPAD)
#define MXC_PSEUDO_IRQ_RTC	(MXC_PSEUDO_IO_BASE + MCU_INT_RTC)
#define MXC_PSEUDO_IRQ_TS_ADC	(MXC_PSEUDO_IO_BASE + MCU_INT_TS_ADC)

/*!
 * @name debug board parameters
 */
/*! @{ */
/*!
 * Base address of debug board
 */
#define DEBUG_BASE_ADDRESS      CS5_BASE_ADDR

/* External ethernet LAN9217 base address */
#define LAN9217_BASE_ADDR       DEBUG_BASE_ADDRESS

/* External UART */
#define UARTA_BASE_ADDR     	(DEBUG_BASE_ADDRESS + 0x08000)
#define UARTB_BASE_ADDR     	(DEBUG_BASE_ADDRESS + 0x10000)

#define BOARD_IO_ADDR		(DEBUG_BASE_ADDRESS + 0x20000)

/* LED switchs */
#define LED_SWITCH_REG        	0x00
/* buttons */
#define SWITCH_BUTTON_REG     	0x08
/* status, interrupt */
#define INTR_STATUS_REG       	0x10
#define INTR_RESET_REG        	0x20
/*CPLD configuration*/
#define CONFIG1_REG           	0x28
#define CONFIG2_REG           	0x30
/*interrupt mask */
#define INTR_MASK_REG        	0x38

/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG     	0x40
#define	MAGIC_NUMBER2_REG     	0x48
/* CPLD code version */
#define CPLD_CODE_VER_REG       0x50
/* magic word for debug CPLD */
#define MAGIC3_NUMBER3_REG     	0x58
/* module reset register*/
#define CONTROL_REG      	0x60
/* CPU ID and Personality ID*/
#define IDENT_REG         	0x68

/* For interrupts like xuart, enet etc */
#define EXPIO_PARENT_INT        MX35_PIN_GPIO1_1

#define EXPIO_INT_ENET_INT          (MXC_BOARD_IRQ_START + 0)
#define EXPIO_INT_XUARTA_INT        (MXC_BOARD_IRQ_START + 1)
#define EXPIO_INT_XUARTB_INT        (MXC_BOARD_IRQ_START + 2)
#define EXPIO_INT_BUTTONA_INT       (MXC_BOARD_IRQ_START + 3)
#define EXPIO_INT_BUTTONB_INT       (MXC_BOARD_IRQ_START + 4)

/*! This is System IRQ used by LAN9217 for interrupt generation taken
 * from platform.h
 */
#define LAN9217_IRQ              EXPIO_INT_ENET_INT

/*! This is base virtual address of debug board*/
extern unsigned int mx35_3stack_board_io;

#define MXC_BD_LED1             (1)
#define MXC_BD_LED2             (1 << 1)
#define MXC_BD_LED3             (1 << 2)
#define MXC_BD_LED4             (1 << 3)
#define MXC_BD_LED5             (1 << 4)
#define MXC_BD_LED6             (1 << 5)
#define MXC_BD_LED7             (1 << 6)
#define MXC_BD_LED8             (1 << 7)
#define MXC_BD_LED_ON(led)
#define MXC_BD_LED_OFF(led)

/*! @} */

#define AHB_FREQ                133000000
#define IPG_FREQ                66500000

extern void mx35_3stack_gpio_init(void) __init;
extern void gpio_tsc_active(void);
extern void gpio_tsc_inactive(void);
extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_write_protect(struct device *dev);
extern void gpio_can_active(int id);
extern void gpio_can_inactive(int id);
extern struct flexcan_platform_data flexcan_data[];
extern int __init mx35_3stack_init_mc13892(void);
extern int __init mx35_3stack_init_mc9s08dz60(void);

#endif				/* CONFIG_MACH_MX35_3DS */
#endif				/* __ASM_ARCH_MXC_BOARD_MX35_3STACK_H__ */
