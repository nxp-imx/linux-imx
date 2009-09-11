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
#ifndef __ASM_ARCH_MXC_MX35_H__
#define __ASM_ARCH_MXC_MX35_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

/*!
 * @file arch-mxc/mx35.h
 * @brief This file contains register definitions.
 *
 * @ingroup MSL_MX35
 */
/*!
 * defines the OS clock tick rate
 */
#define CLOCK_TICK_RATE         16625000

/*!
 * Register an interrupt handler for the SMN as well as the SCC.  In some
 * implementations, the SMN is not connected at all, and in others, it is
 * on the same interrupt line as the SCM. Comment this line out accordingly
 */
#define USE_SMN_INTERRUPT

/*
 * UART Chip level Configuration that a user may not have to edit. These
 * configuration vary depending on how the UART module is integrated with
 * the ARM core
 */
#define MXC_UART_NR 3
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

/*!
 * This option is used to set or clear the dspdma bit in the SDMA config
 * register.
 */
#define MXC_SDMA_DSPDMA         0

/*!
 * Define this option to specify we are using the newer SDMA module.
 */
#define MXC_SDMA_V2

/*
 * IRAM
 */
#define IRAM_BASE_ADDR		0x10000000	/* internal ram */
#define IRAM_BASE_ADDR_VIRT	0xF8400000
#define IRAM_SIZE		SZ_128K

#ifndef CONFIG_SDMA_IRAM
#define CONFIG_SDMA_IRAM_SIZE 0
#endif
#ifdef CONFIG_SND_MXC_SOC_IRAM
#define SND_RAM_SIZE 0x10000
#else
#define SND_RAM_SIZE 0
#endif

#define SND_RAM_BASE_ADDR       (IRAM_BASE_ADDR + CONFIG_SDMA_IRAM_SIZE)
#define MLB_IRAM_ADDR_OFFSET   CONFIG_SDMA_IRAM_SIZE + SND_RAM_SIZE

/*
 * L2CC
 */
#define L2CC_BASE_ADDR		0x30000000

/*
 * AIPS 1
 */
#define AIPS1_BASE_ADDR 	0x43F00000
#define AIPS1_BASE_ADDR_VIRT	0xF8500000
#define AIPS1_SIZE		SZ_1M

#define MAX_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00004000)
#define EVTMON_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00008000)
#define CLKCTL_BASE_ADDR	(AIPS1_BASE_ADDR + 0x0000C000)
#define ETB_SLOT4_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00010000)
#define ETB_SLOT5_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00014000)
#define ECT_CTIO_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00018000)
#define I2C_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00080000)
#define I2C3_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00084000)
#define UART1_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x00090000)
#define UART2_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x00094000)
#define I2C2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00098000)
#define OWIRE_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x0009C000)
#define SSI1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A0000)
#define CSPI1_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x000A4000)
#define KPP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A8000)
#define IOMUXC_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000AC000)
#define ECT_IP1_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000B8000)
#define ECT_IP2_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000BC000)

/*
 * SPBA global module enabled #0
 */
#define SPBA0_BASE_ADDR 	0x50000000
#define SPBA0_BASE_ADDR_VIRT	0xF8600000
#define SPBA0_SIZE		SZ_1M

#define UART3_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x0000C000)
#define CSPI2_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x00010000)
#define SSI2_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00014000)
#define ATA_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00020000)
#define ATA_DMA_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00020000)
#define MSHC1_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00024000)
#define SPDIF_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00028000)
#define ASRC_BASE_ADDR		(SPBA0_BASE_ADDR + 0x0002C000)
#define ESAI_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00034000)
#define FEC_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00038000)
#define SPBA_CTRL_BASE_ADDR	(SPBA0_BASE_ADDR + 0x0003C000)

/*!
 * defines for SPBA modules
 */
#define SPBA_UART3	0x0C
#define SPBA_CSPI2	0x10
#define SPBA_SSI2	0x14
#define SPBA_ATA	0x20
#define SPBA_MSHC	0x24
#define SPBA_SPDIR	0x28
#define SPBA_ASRC	0x2C
#define SPBA_ESAI	0x34
#define SPBA_FEC	0x38

/*!
 * Defines for modules using static and dynamic DMA channels
 */
#define MXC_DMA_CHANNEL_IRAM         30
#define MXC_DMA_CHANNEL_UART1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC1  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC2  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC3  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#ifdef CONFIG_SDMA_IRAM
#define MXC_DMA_CHANNEL_SSI1_TX  (MXC_DMA_CHANNEL_IRAM + 1)
#else
#define MXC_DMA_CHANNEL_SSI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#endif
#define MXC_DMA_CHANNEL_SSI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MEMORY  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SPDIF_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SPDIF_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ESAI_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ESAI_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCC_ESAI MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI1_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI1_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI2_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCA_SSI2_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI1_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI1_TX1 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI2_TX0 MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ASRCB_SSI2_TX1 MXC_DMA_DYNAMIC_CHANNEL

/*
 * AIPS 2
 */
#define AIPS2_BASE_ADDR		0x53F00000
#define AIPS2_BASE_ADDR_VIRT	0xF8700000
#define AIPS2_SIZE		SZ_1M
#define CCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00080000)
#define GPT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00090000)
#define EPIT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00094000)
#define EPIT2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00098000)
#define GPIO3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A4000)
#define SCC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AC000)
#define RNGC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000B0000)
#define MMC_SDHC1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B4000)
#define MMC_SDHC2_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B8000)
#define MMC_SDHC3_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000BC000)
#define IPU_CTRL_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C0000)
#define AUDMUX_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C4000)
#define GPIO1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000CC000)
#define GPIO2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D0000)
#define SDMA_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D4000)
#define RTC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D8000)
#define WDOG1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000DC000)
#define PWM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E0000)
#define CAN1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E4000)
#define CAN2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E8000)
#define RTIC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000EC000)
#define IIM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F0000)
#define OTG_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F4000)
#define MLB_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000F8000)

/*
 * ROMP and AVIC
 */
#define ROMP_BASE_ADDR		0x60000000
#define ROMP_BASE_ADDR_VIRT	0xF8800000
#define ROMP_SIZE		SZ_1M

#define AVIC_BASE_ADDR		0x68000000
#define AVIC_BASE_ADDR_VIRT	0xF8900000
#define AVIC_SIZE		SZ_1M

/*
 * SDRAM, WEIM, M3IF, EMI controllers
 */
#define X_MEMC_BASE_ADDR	0xB8000000
#define X_MEMC_BASE_ADDR_VIRT	0xF8A00000
#define X_MEMC_SIZE		SZ_1M

#define ESDCTL_BASE_ADDR	(X_MEMC_BASE_ADDR + 0x1000)
#define WEIM_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x2000)
#define M3IF_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x3000)
#define EMI_CTL_BASE_ADDR	(X_MEMC_BASE_ADDR + 0x4000)

/*
 * NFC controller
 */
#define NFC_BASE_ADDR		0xBB000000
#define NFC_BASE_ADDR_VIRT    	0xF8B00000
#define NFC_SIZE		SZ_1M

/*
 * Memory regions and CS
 */
/* MX35 ADS SDRAM is from 0x80000000, 64M */
#define IPU_MEM_BASE_ADDR       0x70000000
#define CSD0_BASE_ADDR          0x80000000
#define CSD1_BASE_ADDR          0x90000000

#define SDRAM_BASE_ADDR         CSD0_BASE_ADDR

#define CS0_BASE_ADDR           0xA0000000
#define CS1_BASE_ADDR           0xA8000000
#define CS2_BASE_ADDR           0xB0000000
#define CS3_BASE_ADDR           0xB2000000
#define CS4_BASE_ADDR           0xB4000000
#define CS5_BASE_ADDR           0xB6000000

/*!
 * This macro defines the physical to virtual address mapping for all the
 * peripheral modules. It is used by passing in the physical address as x
 * and returning the virtual address. If the physical address is not mapped,
 * it returns 0xDEADBEEF
 */
#define IO_ADDRESS(x)   \
	(((x >= IRAM_BASE_ADDR) && (x < (IRAM_BASE_ADDR + IRAM_SIZE))) ? IRAM_IO_ADDRESS(x):\
	((x >= AIPS1_BASE_ADDR) && (x < (AIPS1_BASE_ADDR + AIPS1_SIZE))) ? AIPS1_IO_ADDRESS(x):\
	((x >= SPBA0_BASE_ADDR) && (x < (SPBA0_BASE_ADDR + SPBA0_SIZE))) ? SPBA0_IO_ADDRESS(x):\
	((x >= AIPS2_BASE_ADDR) && (x < (AIPS2_BASE_ADDR + AIPS2_SIZE))) ? AIPS2_IO_ADDRESS(x):\
	((x >= ROMP_BASE_ADDR) && (x < (ROMP_BASE_ADDR + ROMP_SIZE))) ? ROMP_IO_ADDRESS(x):\
	((x >= AVIC_BASE_ADDR) && (x < (AVIC_BASE_ADDR + AVIC_SIZE))) ? AVIC_IO_ADDRESS(x):\
	((x >= NFC_BASE_ADDR) && (x < (NFC_BASE_ADDR + NFC_SIZE))) ? NFC_IO_ADDRESS(x):\
	((x >= X_MEMC_BASE_ADDR) && (x < (X_MEMC_BASE_ADDR + X_MEMC_SIZE))) ? X_MEMC_IO_ADDRESS(x):\
	0xDEADBEEF)

/*
 * define the address mapping macros: in physical address order
 */

#define IRAM_IO_ADDRESS(x)  \
	(((x) - IRAM_BASE_ADDR) + IRAM_BASE_ADDR_VIRT)

#define AIPS1_IO_ADDRESS(x)  \
	(((x) - AIPS1_BASE_ADDR) + AIPS1_BASE_ADDR_VIRT)

#define SPBA0_IO_ADDRESS(x)  \
	(((x) - SPBA0_BASE_ADDR) + SPBA0_BASE_ADDR_VIRT)

#define AIPS2_IO_ADDRESS(x)  \
	(((x) - AIPS2_BASE_ADDR) + AIPS2_BASE_ADDR_VIRT)

#define ROMP_IO_ADDRESS(x)  \
	(((x) - ROMP_BASE_ADDR) + ROMP_BASE_ADDR_VIRT)

#define AVIC_IO_ADDRESS(x)  \
	(((x) - AVIC_BASE_ADDR) + AVIC_BASE_ADDR_VIRT)

#define X_MEMC_IO_ADDRESS(x)  \
	(((x) - X_MEMC_BASE_ADDR) + X_MEMC_BASE_ADDR_VIRT)

#define NFC_IO_ADDRESS(x) \
	(((x) - NFC_BASE_ADDR) + NFC_BASE_ADDR_VIRT)

/*
 * DMA request assignments
 */
#define DMA_REQ_ASRC_DMA6  41
#define DMA_REQ_ASRC_DMA5  40
#define DMA_REQ_ASRC_DMA4  39
#define DMA_REQ_ASRC_DMA3  38
#define DMA_REQ_ASRC_DMA2  37
#define DMA_REQ_ASRC_DMA1  36
#define DMA_REQ_RSVD3      35
#define DMA_REQ_RSVD2      34
#define DMA_REQ_ESAI_TX    33
#define DMA_REQ_ESAI_RX    32
#define DMA_REQ_ECT        31
#define DMA_REQ_NFC        30
#define DMA_REQ_SSI1_TX1   29
#define DMA_REQ_SSI1_RX1   28
#define DMA_REQ_SSI1_TX2   27
#define DMA_REQ_SSI1_RX2   26
#define DMA_REQ_SSI2_TX1   25
#define DMA_REQ_SSI2_RX1   24
#define DMA_REQ_SSI2_TX2   23
#define DMA_REQ_SSI2_RX2   22
#define DMA_REQ_IPU        21
#define DMA_REQ_RSVD1      20
#define DMA_REQ_UART1_TX   19
#define DMA_REQ_UART1_RX   18
#define DMA_REQ_UART2_TX   17
#define DMA_REQ_UART2_RX   16
#define DMA_REQ_EXTREQ1    15
#define DMA_REQ_EXTREQ2    14
#define DMA_REQ_SPDIF_TX   13
#define DMA_REQ_SPDIF_RX   12
#define DMA_REQ_UART3_TX   11
#define DMA_REQ_UART3_RX   10
#define DMA_REQ_CSPI1_TX   9
#define DMA_REQ_CSPI1_RX   8
#define DMA_REQ_CSPI2_TX   7
#define DMA_REQ_CSPI2_RX   6
#define DMA_REQ_MSHC       5
#define DMA_REQ_ATA_RX     4
#define DMA_REQ_ATA_TX     3
#define DMA_REQ_ATA_TX_END 2
#define DMA_REQ_DPTC	   1
#define DMA_REQ_DVFS  	   1
#define DMA_REQ_EXTREQ0    0

/*
 * Interrupt numbers
 */
#define MXC_INT_BASE			0
#define MXC_INT_RESV0	            	0
#define MXC_INT_RESV1               	1
#define MXC_INT_OWIRE         	    	2
#define MXC_INT_I2C3                	3
#define MXC_INT_I2C2                	4
#define MXC_INT_RESV2	            	5
#define MXC_INT_RTIC                	6
#define MXC_INT_MMC_SDHC1           	7
#define MXC_INT_MMC_SDHC2           	8
#define MXC_INT_MMC_SDHC3           	9
#define MXC_INT_I2C                 	10
#define MXC_INT_SSI1                	11
#define MXC_INT_SSI2                	12
#define MXC_INT_CSPI2               	13
#define MXC_INT_CSPI1               	14
#define MXC_INT_ATA                 	15
#define MXC_INT_GPU2D               	16
#define MXC_INT_ASRC                	17
#define MXC_INT_UART3               	18
#define MXC_INT_IIM                 	19
#define MXC_INT_RESV20              	20
#define MXC_INT_RESV21              	21
#define MXC_INT_RNG                	22
#define MXC_INT_EVTMON              	23
#define MXC_INT_KPP                 	24
#define MXC_INT_RTC                 	25
#define MXC_INT_PWM                 	26
#define MXC_INT_EPIT2               	27
#define MXC_INT_EPIT1               	28
#define MXC_INT_GPT                 	29
#define MXC_INT_POWERFAIL           	30
#define MXC_INT_CCM                 	31
#define MXC_INT_UART2               	32
#define MXC_INT_NANDFC              	33
#define MXC_INT_SDMA                	34
#define MXC_INT_USB_HS              	35
#define MXC_INT_RESV36              	36
#define MXC_INT_USB_OTG             	37
#define MXC_INT_RESV38              	38
#define MXC_INT_MSHC1               	39
#define MXC_INT_ESAI                	40
#define MXC_INT_IPU_ERR             	41
#define MXC_INT_IPU_SYN             	42
#define MXC_INT_CAN1                	43
#define MXC_INT_CAN2                	44
#define MXC_INT_UART1               	45
#define MXC_INT_MLB                 	46
#define MXC_INT_SPDIF               	47
#define MXC_INT_ECT                 	48
#define MXC_INT_SCC_SCM             	49
#define MXC_INT_SCC_SMN             	50
#define MXC_INT_GPIO2               	51
#define MXC_INT_GPIO1               	52
#define MXC_INT_RESV53              	53
#define MXC_INT_RESV54              	54
#define MXC_INT_WDOG                	55
#define MXC_INT_GPIO3               	56
#define MXC_INT_FEC                 	57
#define MXC_INT_EXT_POWER           	58
#define MXC_INT_EXT_TEMPER          	59
#define MXC_INT_EXT_SENSOR60        	60
#define MXC_INT_EXT_SENSOR61        	61
#define MXC_INT_EXT_WDOG            	62
#define MXC_INT_EXT_TV              	63

#define MXC_MAX_INT_LINES       	64

#define MXC_INT_FORCE			MXC_INT_RESV0
/*!
 * Interrupt Number for ARM11 PMU
 */
#define ARM11_PMU_IRQ		MXC_INT_EVTMON
/* DVFS interrupt*/
#define MXC_INT_DVFS           	MXC_INT_CCM

#define	MXC_GPIO_INT_BASE	(MXC_MAX_INT_LINES)

/* gpio and gpio based interrupt handling */
#define GPIO_DR                 0x00
#define GPIO_GDIR               0x04
#define GPIO_PSR                0x08
#define GPIO_ICR1               0x0C
#define GPIO_ICR2               0x10
#define GPIO_IMR                0x14
#define GPIO_ISR                0x18
#define GPIO_INT_LOW_LEV        0x0
#define GPIO_INT_HIGH_LEV       0x1
#define GPIO_INT_RISE_EDGE      0x2
#define GPIO_INT_FALL_EDGE      0x3
#define GPIO_INT_NONE           0x4

/*!
 * Number of GPIO port as defined in the IC Spec
 */
#define GPIO_PORT_NUM           3
/*!
 * Number of GPIO pins per port
 */
#define GPIO_NUM_PIN            32

/*!
 * NFMS bit in RCSR register for pagesize of nandflash
 */
#define NFMS		(*((volatile u32 *)IO_ADDRESS(CCM_BASE_ADDR+0x18)))
#define NFMS_BIT		8
#define NFMS_NF_DWIDTH		14
#define NFMS_NF_PG_SZ		8

#endif				/*  __ASM_ARCH_MXC_MX35_H__ */
