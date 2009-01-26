/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup MSL Machine Specific Layer (MSL)
 */

/*!
 * @defgroup System System-wide Misc Files for MSL
 * @ingroup MSL
 */

/*!
 * @file arch-mxc/mx21.h
 * @brief This file contains register definitions.
 *
 * @ingroup System
 */

#ifndef __ASM_ARCH_MXC_MX21_H__
#define __ASM_ARCH_MXC_MX21_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

/*!
 * defines the OS clock tick rate
 */
#define CLOCK_TICK_RATE         13300000

/*!
 * UART Chip level Configuration that a user may not have to edit. These
 * configuration vary depending on how the UART module is integrated with
 * the ARM core
 */
#define MXC_UART_NR 4
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive Irda data.
 */
#define MXC_UART_IR_RXDMUX      0x0004
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive UART data.
 */
#define MXC_UART_RXDMUX         0x0004

/*
 * IRAM
 */
#define IRAM_BASE_ADDR		0xFFFFD800	/* internal ram */
#define IRAM_BASE_ADDR_VIRT	0xD0000000
#define IRAM_SIZE		(SZ_4K + SZ_1K*2)

/*
 *  Register offests.
 */
#define AIPI_BASE_ADDR			0x10000000
#define AIPI_BASE_ADDR_VIRT		0xD4000000
#define AIPI_SIZE			SZ_1M

#define DMA_BASE_ADDR                 (AIPI_BASE_ADDR + 0x01000)
#define WDOG_BASE_ADDR               (AIPI_BASE_ADDR + 0x02000)
#define GPT1_BASE_ADDR              (AIPI_BASE_ADDR + 0x03000)
#define GPT2_BASE_ADDR              (AIPI_BASE_ADDR + 0x04000)
#define GPT3_BASE_ADDR              (AIPI_BASE_ADDR + 0x05000)
#define PWM_BASE_ADDR               (AIPI_BASE_ADDR + 0x06000)
#define RTC_BASE_ADDR               (AIPI_BASE_ADDR + 0x07000)
#define KPP_BASE_ADDR               (AIPI_BASE_ADDR + 0x08000)
#define OWIRE_BASE_ADDR             (AIPI_BASE_ADDR + 0x09000)
#define UART1_BASE_ADDR             (AIPI_BASE_ADDR + 0x0A000)
#define UART2_BASE_ADDR             (AIPI_BASE_ADDR + 0x0B000)
#define UART3_BASE_ADDR             (AIPI_BASE_ADDR + 0x0C000)
#define UART4_BASE_ADDR             (AIPI_BASE_ADDR + 0x0D000)
#define CSPI1_BASE_ADDR             (AIPI_BASE_ADDR + 0x0E000)
#define CSPI2_BASE_ADDR             (AIPI_BASE_ADDR + 0x0F000)
#define SSI1_BASE_ADDR              (AIPI_BASE_ADDR + 0x10000)
#define SSI2_BASE_ADDR              (AIPI_BASE_ADDR + 0x11000)
#define I2C_BASE_ADDR               (AIPI_BASE_ADDR + 0x12000)
#define SDHC1_BASE_ADDR             (AIPI_BASE_ADDR + 0x13000)
#define SDHC2_BASE_ADDR             (AIPI_BASE_ADDR + 0x14000)
#define GPIO_BASE_ADDR              (AIPI_BASE_ADDR + 0x15000)
#define AUDMUX_BASE_ADDR            (AIPI_BASE_ADDR + 0x16000)
#define CSPI3_BASE_ADDR             (AIPI_BASE_ADDR + 0x17000)
#define LCDC_BASE_ADDR              (AIPI_BASE_ADDR + 0x21000)
#define SLCDC_BASE_ADDR             (AIPI_BASE_ADDR + 0x22000)
#define USBOTG_BASE_ADDR            (AIPI_BASE_ADDR + 0x24000)
#define EMMA_BASE_ADDR              (AIPI_BASE_ADDR + 0x26000)
#define CCM_BASE_ADDR               (AIPI_BASE_ADDR + 0x27000)
#define FIRI_BASE_ADDR              (AIPI_BASE_ADDR + 0x28000)
#define JAM_BASE_ADDR               (AIPI_BASE_ADDR + 0x3E000)
#define MAX_BASE_ADDR               (AIPI_BASE_ADDR + 0x3F000)

/*
 * ROMP and AVIC
 */
#define ROMP_BASE_ADDR          0x10041000

#define AVIC_BASE_ADDR          0x10040000

#define SAHB1_BASE_ADDR         0x80000000
#define SAHB1_BASE_ADDR_VIRT    0xD4100000
#define SAHB1_SIZE              SZ_1M

#define CSI_BASE_ADDR           (SAHB1_BASE_ADDR + 0x0000)

/*
 * NAND, SDRAM, WEIM, M3IF, EMI controllers
 */
#define X_MEMC_BASE_ADDR	0xDF000000
#define X_MEMC_BASE_ADDR_VIRT	0xD4320000
#define X_MEMC_SIZE		SZ_64K

#define SDRAMC_BASE_ADDR	(X_MEMC_BASE_ADDR)
#define WEIM_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x1000)
#define PCMCIA_CTL_BASE_ADDR	(X_MEMC_BASE_ADDR + 0x2000)
#define NFC_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x3000)

/*
 * Memory regions and CS
 */
#define SDRAM_BASE_ADDR         0xC0000000
#define CSD1_BASE_ADDR          0xC4000000

#define CS0_BASE_ADDR           0xC8000000

#define CS1_BASE_ADDR           0xCC000000
#define CS1_BASE_ADDR_VIRT      0xEB000000
#define CS1_SIZE                SZ_16M

#define CS2_BASE_ADDR           0xD0000000
#define CS3_BASE_ADDR           0xD1000000

#define CS4_BASE_ADDR           0xD2000000

#define CS5_BASE_ADDR           0xD2000000
#define PCMCIA_MEM_BASE_ADDR    0xD4000000

/*!
 * This macro defines the physical to virtual address mapping for all the
 * peripheral modules. It is used by passing in the physical address as x
 * and returning the virtual address. If the physical address is not mapped,
 * it returns 0xDEADBEEF
 */
#define IO_ADDRESS(x)   \
        (((x >= AIPI_BASE_ADDR) && (x < (AIPI_BASE_ADDR + AIPI_SIZE))) ? AIPI_IO_ADDRESS(x):\
        ((x >= SAHB1_BASE_ADDR) && (x < (SAHB1_BASE_ADDR + SAHB1_SIZE))) ? SAHB1_IO_ADDRESS(x):\
        ((x >= CS1_BASE_ADDR) && (x < (CS1_BASE_ADDR + CS1_SIZE))) ? CS1_IO_ADDRESS(x):\
        ((x >= X_MEMC_BASE_ADDR) && (x < (X_MEMC_BASE_ADDR + X_MEMC_SIZE))) ? X_MEMC_IO_ADDRESS(x):\
        0xDEADBEEF)

/*
 * define the address mapping macros: in physical address order
 */

#define AIPI_IO_ADDRESS(x)  \
        (((x) - AIPI_BASE_ADDR) + AIPI_BASE_ADDR_VIRT)

#define AVIC_IO_ADDRESS(x)      AIPI_IO_ADDRESS(x)

#define SAHB1_IO_ADDRESS(x)  \
        (((x) - SAHB1_BASE_ADDR) + SAHB1_BASE_ADDR_VIRT)

#define CS1_IO_ADDRESS(x)  \
        (((x) - CS1_BASE_ADDR) + CS1_BASE_ADDR_VIRT)

#define X_MEMC_IO_ADDRESS(x)  \
        (((x) - X_MEMC_BASE_ADDR) + X_MEMC_BASE_ADDR_VIRT)

#define PCMCIA_IO_ADDRESS(x) \
	(((x) - X_MEMC_BASE_ADDR) + X_MEMC_BASE_ADDR_VIRT)

#define IS_MEM_DEVICE_NONSHARED(x)		0

/*
 *  MX21 ADS Interrupt numbers
 */
#define MXC_INT_CSPI3 	            6
#define MXC_INT_GPIO                    8
#define MXC_INT_FIRI                    9
#define MXC_INT_SDHC2                   10
#define MXC_INT_SDHC1                   11
#define MXC_INT_I2C                     12
#define MXC_INT_SSI2                    13
#define MXC_INT_SSI1                    14
#define MXC_INT_CSPI2                   15
#define MXC_INT_CSPI1              	    16
#define MXC_INT_UART4                   17
#define MXC_INT_UART3                   18
#define MXC_INT_UART2                   19
#define MXC_INT_UART1                   20
#define MXC_INT_KPP                     21
#define MXC_INT_RTC		            22
#define MXC_INT_PWM                     23
#define MXC_INT_GPT3                    24
#define MXC_INT_GPT2                    25
#define MXC_INT_GPT1                    26
#define MXC_INT_GPT                     INT_GPT1
#define MXC_INT_WDOG                    27
#define MXC_INT_PCMCIA                  28
#define MXC_INT_NANDFC                  29
#define MXC_INT_BMI                     30
#define MXC_INT_CSI                     31
#define MXC_INT_DMACH0                  32
#define MXC_INT_DMACH1                  33
#define MXC_INT_DMACH2                  34
#define MXC_INT_DMACH3                  35
#define MXC_INT_DMACH4                  36
#define MXC_INT_DMACH5                  37
#define MXC_INT_DMACH6                  38
#define MXC_INT_DMACH7                  39
#define MXC_INT_DMACH8                  40
#define MXC_INT_DMACH9                  41
#define MXC_INT_DMACH10                 42
#define MXC_INT_DMACH11                 43
#define MXC_INT_DMACH12                 44
#define MXC_INT_DMACH13                 45
#define MXC_INT_DMACH14                 46
#define MXC_INT_DMACH15                 47
#define MXC_INT_EMMAENC                 49
#define MXC_INT_EMMADEC                 50
#define MXC_INT_EMMAPRP                 51
#define MXC_INT_EMMAPP                  52
#define MXC_INT_USBWKUP                 53
#define MXC_INT_USBDMA                  54
#define MXC_INT_USBHOST                 55
#define MXC_INT_USBFUNC                 56
#define MXC_INT_USBHNP                  57
#define MXC_INT_USBCTRL                 58
#define MXC_INT_SAHARA                  59
#define MXC_INT_SLCDC                   60
#define MXC_INT_LCDC                    61

#define MXC_MAX_INT_LINES       64
#define MXC_MAX_EXT_LINES       0

#define MXC_MUX_GPIO_INTERRUPTS		1
#define MXC_GPIO_BASE			(MXC_MAX_INT_LINES)

/*!
 * Number of GPIO port as defined in the IC Spec
 */
#define GPIO_PORT_NUM           6
/*!
 * Number of GPIO pins per port
 */
#define GPIO_NUM_PIN            32

#define DMA_REQ_CSI_RX     31
#define DMA_REQ_CSI_STAT   30
#define DMA_REQ_BMI_RX     29
#define DMA_REQ_BMI_TX     28
#define DMA_REQ_UART1_TX   27
#define DMA_REQ_UART1_RX   26
#define DMA_REQ_UART2_TX   25
#define DMA_REQ_UART2_RX   24
#define DMA_REQ_UART3_TX   23
#define DMA_REQ_UART3_RX   22
#define DMA_REQ_UART4_TX   21
#define DMA_REQ_UART4_RX   20
#define DMA_REQ_CSPI1_TX   19
#define DMA_REQ_CSPI1_RX   18
#define DMA_REQ_CSPI2_TX   17
#define DMA_REQ_CSPI2_RX   16
#define DMA_REQ_SSI1_TX1   15
#define DMA_REQ_SSI1_RX1   14
#define DMA_REQ_SSI1_TX0   13
#define DMA_REQ_SSI1_RX0   12
#define DMA_REQ_SSI2_TX1   11
#define DMA_REQ_SSI2_RX1   10
#define DMA_REQ_SSI2_TX0   9
#define DMA_REQ_SSI2_RX0   8
#define DMA_REQ_SDHC1      7
#define DMA_REQ_SDHC2      6
#define DMA_FIRI_TX        5
#define DMA_FIRI_RX        4
#define DMA_EX             3
#define DMA_REQ_CSPI3_TX   2
#define DMA_REQ_CSPI3_RX   1

#define MXC_TIMER_GPT1          1
#define MXC_TIMER_GPT2          2
#define MXC_TIMER_GPT3          3

/*!
 * NFMS bit in FMCR register for pagesize of nandflash
 */
#define NFMS (*((volatile u32 *)IO_ADDRESS(CCM_BASE_ADDR+0x814)))

#define NFMS_BIT 5

#endif				/* __ASM_ARCH_MXC_MX21_H__ */
