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

#ifndef __ASM_ARCH_MXC_BOARD_MX31ADS_H__
#define __ASM_ARCH_MXC_BOARD_MX31ADS_H__

#ifdef CONFIG_MACH_MX31ADS
/*!
 * @defgroup BRDCFG_MX31 Board Configuration Options
 * @ingroup MSL_MX31
 */

/*!
 * @file mach-mx3/board-mx31ads.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX31 ADS Platform.
 *
 * @ingroup BRDCFG_MX31
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
#define UART2_IR                IRDA
#ifdef CONFIG_MXC_FIR_MODULE
#define UART2_ENABLED           0
#else
#define UART2_ENABLED           1
#endif
/* UART 3 configuration */
#define UART3_MODE              MODE_DTE
#define UART3_IR                NO_IRDA
#define UART3_ENABLED           1
/* UART 4 configuration */
#define UART4_MODE              MODE_DTE
#define UART4_IR                NO_IRDA
#define UART4_ENABLED           0	/* Disable UART 4 as its pins are shared with ATA */
/* UART 5 configuration */
#define UART5_MODE              MODE_DTE
#define UART5_IR                NO_IRDA
#define UART5_ENABLED           1

#define MXC_LL_EXTUART_PADDR	(CS4_BASE_ADDR + 0x10000)
#define MXC_LL_EXTUART_VADDR	CS4_IO_ADDRESS(MXC_LL_EXTUART_PADDR)
#undef  MXC_LL_EXTUART_16BIT_BUS

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

/*!
 * @name PBC Controller parameters
 */
/*! @{ */
/*!
 * Base address of PBC controller
 */
#define PBC_BASE_ADDRESS        IO_ADDRESS(CS4_BASE_ADDR)
/* Offsets for the PBC Controller register */
/*!
 * PBC Board status register offset
 */
#define PBC_BSTAT               0x000002
/*!
 * PBC Board control register 1 set address.
 */
#define PBC_BCTRL1_SET          0x000004
/*!
 * PBC Board control register 1 clear address.
 */
#define PBC_BCTRL1_CLEAR        0x000006
/*!
 * PBC Board control register 2 set address.
 */
#define PBC_BCTRL2_SET          0x000008
/*!
 * PBC Board control register 2 clear address.
 */
#define PBC_BCTRL2_CLEAR        0x00000A
/*!
 * PBC Board control register 3 set address.
 */
#define PBC_BCTRL3_SET          0x00000C
/*!
 * PBC Board control register 3 clear address.
 */
#define PBC_BCTRL3_CLEAR        0x00000E
/*!
 * PBC Board control register 4 set address.
 */
#define PBC_BCTRL4_SET          0x000010
/*!
 * PBC Board control register 4 clear address.
 */
#define PBC_BCTRL4_CLEAR        0x000012
/*!
 * PBC Board status register 1.
 */
#define PBC_BSTAT1              0x000014
/*!
 * PBC Board interrupt status register.
 */
#define PBC_INTSTATUS           0x000016
/*!
 * PBC Board interrupt current status register.
 */
#define PBC_INTCURR_STATUS      0x000018
/*!
 * PBC Interrupt mask register set address.
 */
#define PBC_INTMASK_SET         0x00001A
/*!
 * PBC Interrupt mask register clear address.
 */
#define PBC_INTMASK_CLEAR       0x00001C

/*!
 * External UART A.
 */
#define PBC_SC16C652_UARTA      0x010000
/*!
 * External UART B.
 */
#define PBC_SC16C652_UARTB      0x010010
/*!
 * Ethernet Controller IO base address.
 */
#define PBC_CS8900A_IOBASE      0x020000
/*!
 * Ethernet Controller Memory base address.
 */
#define PBC_CS8900A_MEMBASE     0x021000
/*!
 * Ethernet Controller DMA base address.
 */
#define PBC_CS8900A_DMABASE     0x022000
/*!
 * External chip select 0.
 */
#define PBC_XCS0                0x040000
/*!
 * LCD Display enable.
 */
#define PBC_LCD_EN_B            0x060000
/*!
 * Code test debug enable.
 */
#define PBC_CODE_B              0x070000
/*!
 * PSRAM memory select.
 */
#define PBC_PSRAM_B             0x5000000

/* PBC Board Status Register 1 bit definitions */
#define PBC_BSTAT1_NF_DET       0x0001	/* NAND flash card. 0 = connected */
#define PBC_BSTAT1_KP_ON        0x0002	/* KPP board. 0 = connected */
#define PBC_BSTAT1_LS           0x0004	/* KPP:LightSense signal */
#define PBC_BSTAT1_ATA_IOCS16   0x0008	/* ATA_IOCS16 signal */
#define PBC_BSTAT1_ATA_CBLID    0x0010	/* ATA_CBLID signal */
#define PBC_BSTAT1_ATA_DASP     0x0020	/* ATA_DASP signal */
#define PBC_BSTAT1_PWR_RDY      0x0040	/* MC13783 power. 1 = ready */
#define PBC_BSTAT1_SD1_WP       0x0080	/* 0 = SD1 card is write protected */
#define PBC_BSTAT1_SD2_WP       0x0100	/* 0 = SD2 card is write protected */
#define PBC_BSTAT1_FS1          0x0200	/* KPP:FlipSense1 signal */
#define PBC_BSTAT1_FS2          0x0400	/* KPP:FlipSense2 signal */
#define PBC_BSTAT1_PTT          0x0800	/* KPP:PTT signal */
#define PBC_BSTAT1_MC13783_IN   0x1000	/* MC13783 board. 0 = connected. */

/* PBC Board Control Register 1 bit definitions */
#define PBC_BCTRL1_ERST         0x0001	/* Ethernet Reset */
#define PBC_BCTRL1_URST         0x0002	/* Reset External UART controller */
#define PBC_BCTRL1_UENA         0x0004	/* Enable UART A transceiver */
#define PBC_BCTRL1_UENB         0x0008	/* Enable UART B transceiver */
#define PBC_BCTRL1_UENCE        0x0010	/* Enable UART CE transceiver */
#define PBC_BCTRL1_IREN         0x0020	/* Enable the IRDA transmitter */
#define PBC_BCTRL1_LED0         0x0040	/* Used to control LED 0 (green) */
#define PBC_BCTRL1_LED1         0x0080	/* Used to control LED 1 (yellow) */
#define PBC_BCTRL1_SENSOR1_ON	0x0600	/* Enable Sensor 1 */
#define PBC_BCTRL1_SENSOR2_ON	0x3000	/* Enable Sensor 2 */
#define PBC_BCTRL1_BEND         0x4000	/* Big Endian Select */
#define PBC_BCTRL1_LCDON        0x8000	/* Enable the LCD */

/* PBC Board Control Register 2 bit definitions */
#define PBC_BCTRL2_USELA	0x0001	/* UART A Select, 0 = UART1, 1 = UART5 */
#define PBC_BCTRL2_USELB	0x0002	/* UART B Select, 0 = UART3, 1 = UART5 */
#define PBC_BCTRL2_USELC	0x0004	/* UART C Select, 0 = UART2, 1 = UART1 */
#define PBC_BCTRL2_UMODENA	0x0008	/* UART A Modem Signals Enable, 0 = enabled */
#define PBC_BCTRL2_UMODENC	0x0008	/* UART C Modem Signals Enable, 0 = enabled */
#define PBC_BCTRL2_CSI_EN	0x0020	/* Enable the CSI interface, 0 = enabled */
#define PBC_BCTRL2_ATA_EN	0x0040	/* Enable the ATA interface, 0 = enabled */
#define PBC_BCTRL2_ATA_SEL	0x0080	/* ATA Select, 0 = group A, 1 = group B */
#define PBC_BCTRL2_IRDA_MOD	0x0100	/* IRDA Mode (see CPLD spec) */
#define PBC_BCTRL2_LDC_RST0	0x0200	/* LCD 0 Reset, 1 = reset signal asserted */
#define PBC_BCTRL2_LDC_RST1	0x0400	/* LCD 1 Reset, 1 = reset signal asserted */
#define PBC_BCTRL2_LDC_RST2	0x0800	/* LCD 2 Reset, 1 = reset signal asserted */
#define PBC_BCTRL2_LDCIO_EN	0x1000	/* LCD GPIO Enable, 0 = enabled */
#define PBC_BCTRL2_CT_CS	0x2000	/* Code Test Chip Select, = Code Test selected */
#define PBC_BCTRL2_VPP_EN	0x4000	/* PCMCIA VPP Enable, 1 = power on */
#define PBC_BCTRL2_VCC_EN	0x8000	/* PCMCIA VCC Enable, 1 = power on */

/* PBC Board Control Register 3 bit definitions */
#define PBC_BCTRL3_OTG_FS_SEL	0x0001	/* USB OTG Full Speed Select, 0 = PMIC, 1 = CPU */
#define PBC_BCTRL3_OTG_FS_EN	0x0002	/* USB OTG Full Speed Enable, 0 = enabled */
#define PBC_BCTRL3_FSH_SEL	0x0004	/* USB Full Speed Host Select, 0 = Group A, 1 = Group B */
#define PBC_BCTRL3_FSH_EN	0x0008	/* USB Full Speed Host Enable, 0 = enabled */
#define PBC_BCTRL3_HSH_SEL	0x0010	/* USB High Speed Host Select, 0 = Group A, 1 = Group B */
#define PBC_BCTRL3_HSH_EN	0x0020	/* USB High Speed Host Enable, 0 = enabled */
#define PBC_BCTRL3_FSH_MOD	0x0040	/* USB Full Speed Host Mode, 0 = Differential, 1 = Single ended */
#define PBC_BCTRL3_OTG_HS_EN	0x0080	/* USB OTG High Speed Enable, 0 = enabled */
#define PBC_BCTRL3_OTG_VBUS_EN	0x0100	/* USB OTG VBUS Regulator Enable, 0 = enabled */
#define PBC_BCTRL3_FSH_VBUS_EN	0x0200	/* USB Full Speed Host VBUS Regulator Enable, 0 = enabled */
#define PBC_BCTRL3_CARD1_SEL	0x0400	/* Card1 Select, 0 = SD1, 1 = MS1 */
#define PBC_BCTRL3_CARD2_SEL	0x0800	/* Card2 Select, 0 = PCMCIA & SD2, 1 = MS2 */
#define PBC_BCTRL3_SYNTH_RST	0x1000	/* Audio Synthesizer Reset, 0 = reset asserted */
#define PBC_BCTRL3_VSIM_EN	0x2000	/* VSIM Regulator Enable, 1 = enabled */
#define PBC_BCTRL3_VESIM_EN	0x4000	/* VESIM Regulator Enable, 1 = enabled */
#define PBC_BCTRL3_SPI3_RESET	0x8000	/* CSPI3 Connector Reset, 0 = reset asserted */

/* PBC Board Control Register 4 bit definitions */
#define PBC_BCTRL4_CSI_MSB_EN	0x0001	/* CSI MSB Enable, 0 = CSI_Data[3:0] enabled */
#define PBC_BCTRL4_REGEN_SEL	0x0002	/* Regulator Enable Select, 0 = enabled */
#define PBC_BCTRL4_USER_OFF	0x0004	/* User Off Indication, 1 = user off confirmation */
#define PBC_BCTRL4_VIB_EN	0x0008	/* Vibrator Enable, 1 = enabled */
#define PBC_BCTRL4_PCMCIA_EN	0x0010	/* PCMCIA Enable, 0 = buffer enabled */

#define CKIH_27MHZ_BIT_SET      (1 << 4)

#define PBC_INT_CS8900A         4
/*! @} */

#define PBC_INTSTATUS_REG	(PBC_INTSTATUS + PBC_BASE_ADDRESS)
#define PBC_INTCURR_STATUS_REG	(PBC_INTCURR_STATUS + PBC_BASE_ADDRESS)
#define PBC_INTMASK_SET_REG	(PBC_INTMASK_SET + PBC_BASE_ADDRESS)
#define PBC_INTMASK_CLEAR_REG	(PBC_INTMASK_CLEAR + PBC_BASE_ADDRESS)
#define EXPIO_PARENT_INT	IOMUX_TO_IRQ(MX31_PIN_GPIO1_4)

#define MXC_EXP_IO_BASE		(MXC_BOARD_IRQ_START)
#define EXPIO_INT_LOW_BAT	(MXC_EXP_IO_BASE + 0)
#define EXPIO_INT_PB_IRQ	(MXC_EXP_IO_BASE + 1)
#define EXPIO_INT_OTG_FS_OVR	(MXC_EXP_IO_BASE + 2)
#define EXPIO_INT_FSH_OVR	(MXC_EXP_IO_BASE + 3)
#define EXPIO_INT_RES4		(MXC_EXP_IO_BASE + 4)
#define EXPIO_INT_RES5		(MXC_EXP_IO_BASE + 5)
#define EXPIO_INT_RES6		(MXC_EXP_IO_BASE + 6)
#define EXPIO_INT_RES7		(MXC_EXP_IO_BASE + 7)
#define EXPIO_INT_ENET_INT	(MXC_EXP_IO_BASE + 8)
#define EXPIO_INT_OTG_FS_INT	(MXC_EXP_IO_BASE + 9)
#define EXPIO_INT_XUART_INTA	(MXC_EXP_IO_BASE + 10)
#define EXPIO_INT_XUART_INTB	(MXC_EXP_IO_BASE + 11)
#define EXPIO_INT_SYNTH_IRQ	(MXC_EXP_IO_BASE + 12)
#define EXPIO_INT_CE_INT1	(MXC_EXP_IO_BASE + 13)
#define EXPIO_INT_CE_INT2	(MXC_EXP_IO_BASE + 14)
#define EXPIO_INT_RES15		(MXC_EXP_IO_BASE + 15)

#define MXC_MAX_EXP_IO_LINES	16

/*!
 * @name  Defines Base address and IRQ used for CS8900A Ethernet Controller on MXC Boards
 */
/*! @{*/
/*! This is System IRQ used by CS8900A for interrupt generation taken from platform.h */
#define CS8900AIRQ              EXPIO_INT_ENET_INT
/*! This is I/O Base address used to access registers of CS8900A on MXC ADS */
#define CS8900A_BASE_ADDRESS    (PBC_BASE_ADDRESS + PBC_CS8900A_IOBASE + 0x300)
/*! @} */

#define MXC_PMIC_INT_LINE	IOMUX_TO_IRQ(MX31_PIN_GPIO1_3)

#define AHB_FREQ                133000000
#define IPG_FREQ                66500000

#define MXC_BD_LED1             (1 << 6)
#define MXC_BD_LED2             (1 << 7)
#define MXC_BD_LED_ON(led) \
        __raw_writew(led, PBC_BASE_ADDRESS + PBC_BCTRL1_SET)
#define MXC_BD_LED_OFF(led) \
        __raw_writew(led, PBC_BASE_ADDRESS + PBC_BCTRL1_CLEAR)

#endif				/* CONFIG_MACH_MX31ADS */
#endif				/* __ASM_ARCH_MXC_BOARD_MX31ADS_H__ */
