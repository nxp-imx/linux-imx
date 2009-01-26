/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_MXC_MX37_H__
#define __ASM_ARCH_MXC_MX37_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

/*!
 * @file arch-mxc/mx37.h
 * @brief This file contains register definitions.
 *
 * @ingroup MSL_MX37
 */
/*!
 * defines the hardware clock tick rate
 */

#define CLOCK_TICK_RATE         8000000

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
#define IRAM_BASE_ADDR_VIRT	0xF8000000
#define IRAM_SIZE			(9*SZ_8K)	/* 72KB */

#if defined(CONFIG_MXC_SECURITY_SCC2) \
	|| defined(CONFIG_MXC_SECURITY_SCC2_MODULE)
#define SCC_IRAM_SIZE  SZ_16K
#else
#define SCC_IRAM_SIZE  0
#endif

/*#ifndef CONFIG_SDMA_IRAM
#define CONFIG_SDMA_IRAM_SIZE 0
#endif*/
#ifdef CONFIG_SDMA_IRAM
#define SDMA_IRAM_SIZE  CONFIG_SDMA_IRAM_SIZE
#else
#define SDMA_IRAM_SIZE  0
#endif

#ifdef CONFIG_SND_MXC_SOC_IRAM
#define SND_RAM_SIZE 0x6000
#else
#define SND_RAM_SIZE 0
#endif

#ifdef CONFIG_USB_STATIC_IRAM
#define USB_IRAM_SIZE	SZ_8K
#else
#define USB_IRAM_SIZE 0
#endif

#if (IRAM_SIZE < (SCC_IRAM_SIZE + SDMA_IRAM_SIZE + SND_RAM_SIZE + \
	USB_IRAM_SIZE))
#error "IRAM size exceeded"
#endif

#ifdef CONFIG_MXC_VPU_IRAM
#define VPU_IRAM_SIZE	(IRAM_BASE_ADDR + IRAM_SIZE - VPU_IRAM_BASE_ADDR)
#else
#define VPU_IRAM_SIZE 0
#endif

#define SCC_IRAM_BASE_ADDR (IRAM_BASE_ADDR + IRAM_SIZE - SCC_IRAM_SIZE)
#define SDMA_RAM_BASE_ADDR (IRAM_BASE_ADDR)
#define SND_RAM_BASE_ADDR	(IRAM_BASE_ADDR + SDMA_IRAM_SIZE)
#define USB_IRAM_BASE_ADDR	(SND_RAM_BASE_ADDR + SND_RAM_SIZE)
#define VPU_IRAM_BASE_ADDR	(USB_IRAM_BASE_ADDR + USB_IRAM_SIZE)

/*
 * NFC
 */
#define NFC_BASE_ADDR_AXI	0x7FFF0000	/* NAND flash AXI */
#define NFC_BASE_ADDR_AXI_VIRT	0xF9000000
#define NFC_AXI_SIZE		SZ_64K

/*
 * L2CC
 */
#define L2CC_BASE_ADDR		0xB0000000

#define PLATFORM_BASE_ADDR	0xB0400000
#define PLATFORM_BASE_ADDR_VIRT 0xFA000000
#define PLATFORM_SIZE		SZ_1M
#define EVTMON_BASE_ADDR	(PLATFORM_BASE_ADDR + 0x00000000)
#define ARM1176_BASE_ADDR	(PLATFORM_BASE_ADDR + 0x00004000)

#define TZIC_BASE_ADDR		0xB0800000
#define TZIC_BASE_ADDR_VIRT	0xFA100000
#define TZIC_SIZE		SZ_1M

#define DEBUG_BASE_ADDR		0xB0C00000
#define DEBUG_BASE_ADDR_VIRT	0xFA200000
#define DEBUG_SIZE		SZ_1M
#define ETB_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00001000)
#define ETM_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00002000)
#define TPIU_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00003000)
#define CTI0_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00004000)
#define CTI1_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00005000)
#define CTI2_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00006000)

/*
 * AIPS 1
 */
#define AIPS1_BASE_ADDR 	0xC3F00000
#define AIPS1_BASE_ADDR_VIRT	0xFC000000
#define AIPS1_SIZE		SZ_1M

#define MAX_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00080000)
#define GPIO1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00084000)
#define GPIO2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00088000)
#define GPIO3_BASE_ADDR		(AIPS1_BASE_ADDR + 0x0008C000)
#define KPP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00094000)
#define WDOG1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00098000)
#define WDOG2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x0009C000)
#define GPT1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A0000)
#define SRTC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A4000)
#define IOMUXC_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000A8000)
#define IIM_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000AC000)
#define CSU_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000B0000)
#define SDMA_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000B4000)
#define SCC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000BC000)
#define ROMCP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000C0000)
#define RTIC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000C4000)
#define VPU_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000D0000)
#define OTG_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000D4000)
#define ATA_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000D8000)
#define MSHC1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000E0000)
#define FEC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000E8000)
#define RNGC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000EC000)
#define TVE_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000F0000)

/*
 * SPBA global module enabled #0
 */
#define SPBA0_BASE_ADDR 	0xC0000000
#define SPBA0_BASE_ADDR_VIRT	0xFC100000
#define SPBA0_SIZE		SZ_1M

#define MMC_SDHC1_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00004000)
#define MMC_SDHC2_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00008000)
#define UART3_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x0000C000)
#define CSPI2_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x00010000)
#define SSI2_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00014000)
#define MMC_SDHC3_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00020000)
#define SPDIF_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00028000)
#define ATA_DMA_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00034000)
#define SPBA_CTRL_BASE_ADDR	(SPBA0_BASE_ADDR + 0x0003C000)

/*!
 * defines for SPBA modules
 */
#define SPBA_SDHC1	0x04
#define SPBA_SDHC2	0x08
#define SPBA_UART3	0x0C
#define SPBA_CSPI2	0x10
#define SPBA_SSI2	0x14
#define SPBA_SDHC3	0x20
#define SPBA_SPDIF	0x28
#define SPBA_ATA	0x34

/*!
 * Defines for modules using static and dynamic DMA channels
 */
#define MXC_DMA_CHANNEL_IRAM         30
#define MXC_DMA_CHANNEL_SPDIF_TX        MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC1		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC2		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_RX		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_TX		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_RX		MXC_DMA_DYNAMIC_CHANNEL
#ifdef CONFIG_SDMA_IRAM
#define MXC_DMA_CHANNEL_SSI2_TX  (MXC_DMA_CHANNEL_IRAM + 1)
#else				/*CONFIG_SDMA_IRAM */
#define MXC_DMA_CHANNEL_SSI2_TX		MXC_DMA_DYNAMIC_CHANNEL
#endif				/*CONFIG_SDMA_IRAM */
#define MXC_DMA_CHANNEL_CSPI1_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI3_RX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI3_TX	MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_RX		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_TX		MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MEMORY		MXC_DMA_DYNAMIC_CHANNEL

/*
 * AIPS 2
 */
#define AIPS2_BASE_ADDR		0xE3F00000
#define AIPS2_BASE_ADDR_VIRT	0xFC200000
#define AIPS2_SIZE		SZ_1M

#define PLL0_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00080000)
#define PLL1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00084000)
#define PLL2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00088000)
#define CCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0008C000)
#define GPC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00090000)
#define SRC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00094000)
#define EPIT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00098000)
#define EPIT2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0009C000)
#define PWM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A0000)
#define OWIRE_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000A4000)
#define CSPI3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A8000)
#define CSPI1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000AC000)
#define UART1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B0000)
#define UART2_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000BC000)
#define I2C3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C0000)
#define I2C2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C4000)
#define I2C_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C8000)
#define SSI1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000CC000)
#define AUDMUX_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000D0000)
#define EMI_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000DBF00)

#define M4IF_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D8000)
#define ESDCTL_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000D9000)
#define WEIM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000DA000)
#define NFC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000DB000)

/*
 * Memory regions and CS
 */
#define IPU_CTRL_BASE_ADDR	0x80000000
#define CSD0_BASE_ADDR          0x40000000
#define CSD1_BASE_ADDR          0x50000000

#define CS0_BASE_ADDR           0x60000000
#define CS1_BASE_ADDR           0x68000000
#define CS2_BASE_ADDR           0x70000000

/*!
 * This macro defines the physical to virtual address mapping for all the
 * peripheral modules. It is used by passing in the physical address as x
 * and returning the virtual address. If the physical address is not mapped,
 * it returns 0xDEADBEEF
 */
#define IO_ADDRESS(x)   \
        (((x >= (unsigned long)IRAM_BASE_ADDR) && (x < (unsigned long)IRAM_BASE_ADDR + IRAM_SIZE)) ? IRAM_IO_ADDRESS(x):\
        ((x >= (unsigned long)PLATFORM_BASE_ADDR) && (x < (unsigned long)PLATFORM_BASE_ADDR + PLATFORM_SIZE)) ? PLATFORM_IO_ADDRESS(x):\
        ((x >= (unsigned long)TZIC_BASE_ADDR) && (x < (unsigned long)TZIC_BASE_ADDR + TZIC_SIZE)) ? TZIC_IO_ADDRESS(x):\
        ((x >= (unsigned long)DEBUG_BASE_ADDR) && (x < (unsigned long)DEBUG_BASE_ADDR + DEBUG_SIZE)) ? DEBUG_IO_ADDRESS(x):\
        ((x >= (unsigned long)SPBA0_BASE_ADDR) && (x < (unsigned long)SPBA0_BASE_ADDR + SPBA0_SIZE)) ? SPBA0_IO_ADDRESS(x):\
        ((x >= (unsigned long)AIPS1_BASE_ADDR) && (x < (unsigned long)AIPS1_BASE_ADDR + AIPS1_SIZE)) ? AIPS1_IO_ADDRESS(x):\
        ((x >= (unsigned long)AIPS2_BASE_ADDR) && (x < (unsigned long)AIPS2_BASE_ADDR + AIPS2_SIZE)) ? AIPS2_IO_ADDRESS(x):\
        ((x >= (unsigned long)NFC_BASE_ADDR_AXI) && (x < (unsigned long)NFC_BASE_ADDR_AXI + NFC_AXI_SIZE)) ? NFC_BASE_ADDR_AXI_IO_ADDRESS(x):\
        0xDEADBEEF)

/*
 * define the address mapping macros: in physical address order
 */

#define IRAM_IO_ADDRESS(x)  \
        (((x) - IRAM_BASE_ADDR) + IRAM_BASE_ADDR_VIRT)

#define PLATFORM_IO_ADDRESS(x)  \
        (((x) - PLATFORM_BASE_ADDR) + PLATFORM_BASE_ADDR_VIRT)

#define TZIC_IO_ADDRESS(x)  \
        (((x) - TZIC_BASE_ADDR) + TZIC_BASE_ADDR_VIRT)

#define DEBUG_IO_ADDRESS(x)  \
        (((x) - DEBUG_BASE_ADDR) + DEBUG_BASE_ADDR_VIRT)

#define SPBA0_IO_ADDRESS(x)  \
        (((x) - SPBA0_BASE_ADDR) + SPBA0_BASE_ADDR_VIRT)

#define AIPS1_IO_ADDRESS(x)  \
        (((x) - AIPS1_BASE_ADDR) + AIPS1_BASE_ADDR_VIRT)

#define AIPS2_IO_ADDRESS(x)  \
        (((x) - AIPS2_BASE_ADDR) + AIPS2_BASE_ADDR_VIRT)

#define NFC_BASE_ADDR_AXI_IO_ADDRESS(x) \
        (((x) - NFC_BASE_ADDR_AXI) + NFC_BASE_ADDR_AXI_VIRT)

#define IS_MEM_DEVICE_NONSHARED(x)	((x) >= 0x80000000)

/*
 * DMA request assignments
 */
#define DMA_REQ_RESV47		47
#define DMA_REQ_VPU		46
#define DMA_REQ_SPDIF_TX	45
#define DMA_REQ_UART3_TX	44
#define DMA_REQ_UART3_RX	43
#define DMA_REQ_I2C2		42
#define DMA_REQ_I2C1		41
#define DMA_REQ_SDHC3		40
#define DMA_REQ_CSPI3_TX	39
#define DMA_REQ_CSPI3_RX	38
#define DMA_REQ_RESV37		37
#define DMA_REQ_IPU		36
#define DMA_REQ_RESV35		35
#define DMA_REQ_EPIT2		34
#define DMA_REQ_RESV33		33
#define DMA_REQ_RESV32		32
#define DMA_REQ_ECT		31
#define DMA_REQ_NFC		30
#define DMA_REQ_SSI1_TX1	29
#define DMA_REQ_SSI1_RX1	28
#define DMA_REQ_SSI1_TX2	27
#define DMA_REQ_SSI1_RX2	26
#define DMA_REQ_SSI2_TX1	25
#define DMA_REQ_SSI2_RX1	24
#define DMA_REQ_SSI2_TX2	23
#define DMA_REQ_SSI2_RX2	22
#define DMA_REQ_SDHC2		21
#define DMA_REQ_SDHC1		20
#define DMA_REQ_UART1_TX	19
#define DMA_REQ_UART1_RX	18
#define DMA_REQ_UART2_TX	17
#define DMA_REQ_UART2_RX	16
#define DMA_REQ_GPIO1_0		15
#define DMA_REQ_GPIO1_1		14
#define DMA_REQ_RESV13		13
#define DMA_REQ_RESV12		12
#define DMA_REQ_RESV11		11
#define DMA_REQ_RESV10		10
#define DMA_REQ_CSPI1_TX	9
#define DMA_REQ_CSPI1_RX	8
#define DMA_REQ_CSPI2_TX	7
#define DMA_REQ_CSPI2_RX	6
#define DMA_REQ_RESV5		5
#define DMA_REQ_ATA_TX_END	4
#define DMA_REQ_ATA_TX		3
#define DMA_REQ_ATA_RX		2
#define DMA_REQ_GPC		1
#define DMA_REQ_RESV0		0

/*
 * Interrupt numbers
 */
#define MXC_INT_BASE		0
#define MXC_INT_RESV0		0
#define MXC_INT_MMC_SDHC1	1
#define MXC_INT_MMC_SDHC2	2
#define MXC_INT_MMC_SDHC3	3
#define MXC_INT_RESV4		4
#define MXC_INT_RESV5		5
#define MXC_INT_SDMA		6
#define MXC_INT_IOMUX		7
#define MXC_INT_RESV8		8
#define MXC_INT_VPU		9
#define MXC_INT_IPU_ERR		10
#define MXC_INT_IPU_SYN		11
#define MXC_INT_RESV12		12
#define MXC_INT_RESV13		13
#define MXC_INT_RNG			14
#define MXC_INT_EMI		15
#define MXC_INT_RESV16		16
#define MXC_INT_RESV17		17
#define MXC_INT_USB_OTG		18
#define MXC_INT_RESV19		19
#define MXC_INT_RESV20		20
#define MXC_INT_SCC_SMN		21
#define MXC_INT_SCC_STZ		22
#define MXC_INT_SCC_SCM		23
#define MXC_INT_SRTC_NTZ	24
#define MXC_INT_SRTC_TZ		25
#define MXC_INT_RTIC		26
#define MXC_INT_CSU		27
#define MXC_INT_RESV28		28
#define MXC_INT_SSI1		29
#define MXC_INT_SSI2		30
#define MXC_INT_UART1		31
#define MXC_INT_UART2		32
#define MXC_INT_UART3		33
#define MXC_INT_RESV34		34
#define MXC_INT_RESV35		35
#define MXC_INT_CSPI1		36
#define MXC_INT_CSPI2		37
#define MXC_INT_CSPI3		38
#define MXC_INT_GPT		39
#define MXC_INT_EPIT1		40
#define MXC_INT_EPIT2		41
#define MXC_INT_GPIO1_INT7	42
#define MXC_INT_GPIO1_INT6	43
#define MXC_INT_GPIO1_INT5	44
#define MXC_INT_GPIO1_INT4	45
#define MXC_INT_GPIO1_INT3	46
#define MXC_INT_GPIO1_INT2	47
#define MXC_INT_GPIO1_INT1	48
#define MXC_INT_GPIO1_INT0	49
#define MXC_INT_GPIO1_LOW	50
#define MXC_INT_GPIO1_HIGH	51
#define MXC_INT_GPIO2_LOW	52
#define MXC_INT_GPIO2_HIGH	53
#define MXC_INT_GPIO3_LOW	54
#define MXC_INT_GPIO3_HIGH	55
#define MXC_INT_RESV56		56
#define MXC_INT_RESV57		57
#define MXC_INT_WDOG1		58
#define MXC_INT_WDOG2		59
#define MXC_INT_KPP		60
#define MXC_INT_PWM		61
#define MXC_INT_I2C		62
#define MXC_INT_I2C2		63
#define MXC_INT_I2C3		64
#define MXC_INT_MSHC1		65
#define MXC_INT_RESV66		66
#define MXC_INT_RESV67		67
#define MXC_INT_RESV68		68
#define MXC_INT_IIM		69
#define MXC_INT_ATA		70
#define MXC_INT_CCM1		71
#define MXC_INT_CCM2		72
#define MXC_INT_GPC1		73
#define MXC_INT_GPC2		74
#define MXC_INT_SRC		75
#define MXC_INT_EVTMON		76
#define MXC_INT_PER_MEASURE	77
#define MXC_INT_DECODE_ERR	78
#define MXC_INT_EVT_COUNT	79
#define MXC_INT_SLAVE_ERR	80
#define MXC_INT_RESV81		81
#define MXC_INT_RESV82		82
#define MXC_INT_RESV83		83
#define MXC_INT_RESV84		84
#define MXC_INT_RESV85		85
#define MXC_INT_RESV86		86
#define MXC_INT_FEC		87
#define MXC_INT_OWIRE		88
#define MXC_INT_CTI0		89
#define MXC_INT_CTM0		90
#define MXC_INT_SPDIF		91
#define MXC_INT_TVOUT		92

#define MXC_MAX_INT_LINES       128

/*!
 * Interrupt Number for ARM11 PMU
 */
#define ARM11_PMU_IRQ		MXC_INT_EVTMON

#define	MXC_GPIO_INT_BASE	(MXC_MAX_INT_LINES)

/*!
 * Number of GPIO port as defined in the IC Spec
 */
#define GPIO_PORT_NUM           3
/*!
 * Number of GPIO pins per port
 */
#define GPIO_NUM_PIN            32

#define MXC_GPIO_SPLIT_IRQ_2

/*!
 * Macro to convert elv, llv, ulv to a data which is used to set DCVR0, DCVR1,
 * DCVR2, or DCVR3.
 */
#define DCVR(elv, llv, ulv)     ((elv << 0) | (llv << 10) | (ulv << 21))

#endif				/*  __ASM_ARCH_MXC_MX37_H__ */
