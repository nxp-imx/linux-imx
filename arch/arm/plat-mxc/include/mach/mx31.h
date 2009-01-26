/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_MX31_H__
#define __ASM_ARCH_MXC_MX31_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

/*!
 * defines the hardware clock tick rate
 */
#define CLOCK_TICK_RATE		16625000

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
#define MXC_UART_NR 5
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

/*
 * MX31 memory map:
 *
 * Virt		Phys		Size	What
 * ---------------------------------------------------------------------------
 * F8000000	1FFC0000	16K	IRAM
 * F9000000	30000000	256M	L2CC
 * FC000000	43F00000	1M	AIPS 1
 * FC100000	50000000	1M	SPBA
 * FC200000	53F00000	1M	AIPS 2
 * FC500000	60000000	128M	ROMPATCH
 * FC400000	68000000	128M	AVIC
 *         	70000000	256M	IPU (MAX M2)
 *         	80000000	256M	CSD0 SDRAM/DDR
 *         	90000000	256M	CSD1 SDRAM/DDR
 *         	A0000000	128M	CS0 Flash
 *         	A8000000	128M	CS1 Flash
 *         	B0000000	32M	CS2
 *         	B2000000	32M	CS3
 * F4000000	B4000000	32M	CS4
 *         	B6000000	32M	CS5
 * FC320000	B8000000	64K	NAND, SDRAM, WEIM, M3IF, EMI controllers
 *         	C0000000	64M	PCMCIA/CF
 */
/*
 * IRAM
 */
#define IRAM_BASE_ADDR		0x1FFC0000	/* internal ram */
#define IRAM_BASE_ADDR_VIRT	0xF80C0000
#define IRAM_SIZE		SZ_16K

#define USB_IRAM_BASE_ADDR	(IRAM_BASE_ADDR)
#ifdef CONFIG_USB_STATIC_IRAM
#define USB_IRAM_SIZE	(2*SZ_8K)
#else
#define USB_IRAM_SIZE 0
#endif

/*
 * L2CC
 */
#define L2CC_BASE_ADDR		0x30000000

/*
 * AIPS 1
 */
#define AIPS1_BASE_ADDR		0x43F00000
#define AIPS1_BASE_ADDR_VIRT	0xFC000000
#define AIPS1_SIZE		SZ_1M

#define MAX_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00004000)
#define EVTMON_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00008000)
#define CLKCTL_BASE_ADDR	(AIPS1_BASE_ADDR + 0x0000C000)
#define ETB_SLOT4_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00010000)
#define ETB_SLOT5_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00014000)
#define ECT_CTIO_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00018000)
#define I2C_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00080000)
#define I2C3_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00084000)
#define OTG_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00088000)
#define ATA_BASE_ADDR		(AIPS1_BASE_ADDR + 0x0008C000)
#define UART1_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x00090000)
#define UART2_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x00094000)
#define I2C2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00098000)
#define OWIRE_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x0009C000)
#define SSI1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A0000)
#define CSPI1_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x000A4000)
#define KPP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A8000)
#define IOMUXC_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000AC000)
#define UART4_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x000B0000)
#define UART5_BASE_ADDR 	(AIPS1_BASE_ADDR + 0x000B4000)
#define ECT_IP1_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000B8000)
#define ECT_IP2_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000BC000)

/*
 * SPBA global module enabled #0
 */
#define SPBA0_BASE_ADDR 	0x50000000
#define SPBA0_BASE_ADDR_VIRT	0xFC100000
#define SPBA0_SIZE		SZ_1M

#define MMC_SDHC1_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00004000)
#define MMC_SDHC2_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00008000)
#define UART3_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x0000C000)
#define CSPI2_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x00010000)
#define SSI2_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00014000)
#define SIM1_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00018000)
#define IIM_BASE_ADDR		(SPBA0_BASE_ADDR + 0x0001C000)
#define ATA_DMA_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00020000)
#define MSHC1_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00024000)
#define MSHC2_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00024000)
#define SPBA_CTRL_BASE_ADDR	(SPBA0_BASE_ADDR + 0x0003C000)

/*!
 * defines for SPBA modules
 */
#define SPBA_SDHC1	0x04
#define SPBA_SDHC2	0x08
#define SPBA_UART3	0x0C
#define SPBA_CSPI2	0x10
#define SPBA_SSI2	0x14
#define SPBA_SIM	0x18
#define SPBA_IIM	0x1C
#define SPBA_ATA	0x20

/*!
 * Defines for modules using static and dynamic DMA channels
 */

#ifdef CONFIG_SDMA_IRAM
#define MXC_DMA_CHANNEL_IRAM         30
#endif				/*CONFIG_SDMA_IRAM */

#define MXC_DMA_CHANNEL_UART1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART4_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART4_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART5_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART5_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC1  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC2  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_RX  MXC_DMA_DYNAMIC_CHANNEL

#ifdef CONFIG_SDMA_IRAM
#define MXC_DMA_CHANNEL_SSI2_TX  (MXC_DMA_CHANNEL_IRAM + 1)
#else				/*CONFIG_SDMA_IRAM */
#define MXC_DMA_CHANNEL_SSI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#endif				/*CONFIG_SDMA_IRAM */

#define MXC_DMA_CHANNEL_FIR_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_FIR_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MEMORY  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_FIFO_MEMORY  MXC_DMA_DYNAMIC_CHANNEL

/*
 * AIPS 2
 */
#define AIPS2_BASE_ADDR		0x53F00000
#define AIPS2_BASE_ADDR_VIRT	0xFC200000
#define AIPS2_SIZE		SZ_1M
#define CCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00080000)
#define CSPI3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00084000)
#define FIRI_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0008C000)
#define GPT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00090000)
#define EPIT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00094000)
#define EPIT2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00098000)
#define GPIO3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A4000)
#define SCC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AC000)
#define SCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AE000)
#define SMN_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AF000)
#define RNGA_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000B0000)
#define IPU_CTRL_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C0000)
#define AUDMUX_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C4000)
#define MPEG4_ENC_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C8000)
#define VPU_BASE_ADDR		MPEG4_ENC_BASE_ADDR
#define GPIO1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000CC000)
#define GPIO2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D0000)
#define SDMA_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D4000)
#define RTC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D8000)
#define WDOG1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000DC000)
#define PWM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000E0000)
#define RTIC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000EC000)

/*
 * ROMP and AVIC
 */
#define ROMP_BASE_ADDR		0x60000000
#define ROMP_BASE_ADDR_VIRT	0xFC500000
#define ROMP_SIZE		SZ_1M

#define AVIC_BASE_ADDR		0x68000000
#define AVIC_BASE_ADDR_VIRT	0xFC400000
#define AVIC_SIZE		SZ_1M

/*
 * NAND, SDRAM, WEIM, M3IF, EMI controllers
 */
#define X_MEMC_BASE_ADDR	0xB8000000
#define X_MEMC_BASE_ADDR_VIRT	0xFC320000
#define X_MEMC_SIZE		SZ_64K

#define NFC_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x0000)
#define ESDCTL_BASE_ADDR	(X_MEMC_BASE_ADDR + 0x1000)
#define WEIM_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x2000)
#define M3IF_BASE_ADDR		(X_MEMC_BASE_ADDR + 0x3000)
#define EMI_CTL_BASE_ADDR	(X_MEMC_BASE_ADDR + 0x4000)
#define PCMCIA_CTL_BASE_ADDR	EMI_CTL_BASE_ADDR

/*
 * Memory regions and CS
 */
#define IPU_MEM_BASE_ADDR	0x70000000
#define CSD0_BASE_ADDR		0x80000000
#define CSD1_BASE_ADDR		0x90000000
#define CS0_BASE_ADDR		0xA0000000
#define CS1_BASE_ADDR		0xA8000000
#define CS2_BASE_ADDR		0xB0000000
#define CS3_BASE_ADDR		0xB2000000

#define CS4_BASE_ADDR		0xB4000000
#define CS4_BASE_ADDR_VIRT	0xF4000000
#define CS4_SIZE		SZ_32M

#define CS5_BASE_ADDR		0xB6000000
#define PCMCIA_MEM_BASE_ADDR	0xBC000000

/*
 * VL2CC for i.MX32
 */
#define VL2CC_BASE_ADDR		0xE0000000

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
	((x >= CS4_BASE_ADDR) && (x < (CS4_BASE_ADDR + CS4_SIZE))) ? CS4_IO_ADDRESS(x):\
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

#define CS4_IO_ADDRESS(x)  \
	(((x) - CS4_BASE_ADDR) + CS4_BASE_ADDR_VIRT)

#define X_MEMC_IO_ADDRESS(x)  \
	(((x) - X_MEMC_BASE_ADDR) + X_MEMC_BASE_ADDR_VIRT)

#define PCMCIA_IO_ADDRESS(x) \
	(((x) - X_MEMC_BASE_ADDR) + X_MEMC_BASE_ADDR_VIRT)

/*
 * DMA request assignments
 */
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
#define DMA_REQ_SDHC2      21
#define DMA_REQ_SDHC1      20
#define DMA_REQ_UART1_TX   19
#define DMA_REQ_UART1_RX   18
#define DMA_REQ_FIRI_TX    17
#define DMA_REQ_FIRI_RX    16
#define DMA_REQ_UART2_TX   17
#define DMA_REQ_UART2_RX   16
#define DMA_REQ_EXTREQ1    15
#define DMA_REQ_EXTREQ2    14
#define DMA_REQ_UART4_TX   13
#define DMA_REQ_UART4_RX   12
#define DMA_REQ_CSPI3_TX   11
#define DMA_REQ_CSPI3_RX   10
#define DMA_REQ_UART5_TX   11
#define DMA_REQ_UART5_RX   10
#define DMA_REQ_CSPI1_TX   9
#define DMA_REQ_CSPI1_RX   8
#define DMA_REQ_UART3_TX   9
#define DMA_REQ_UART3_RX   8
#define DMA_REQ_CSPI2_TX   7
#define DMA_REQ_CSPI2_RX   6
#define DMA_REQ_SIM        5
#define DMA_REQ_ATA_RX     4
#define DMA_REQ_ATA_TX     3
#define DMA_REQ_ATA_TX_END 2
#define DMA_REQ_CCM        1
#define DMA_REQ_EXTREQ0    0

/*
 * Interrupt numbers
 */
#define MXC_INT_PEN_ADS7843	0
#define MXC_INT_RESV1		1
#define MXC_INT_CS8900A		2
#define MXC_INT_I2C3		3
#define MXC_INT_I2C2		4
#define MXC_INT_MPEG4_ENC	5
#define MXC_INT_RTIC		6
#define MXC_INT_FIRI		7
#define MXC_INT_MMC_SDHC2	8
#define MXC_INT_MMC_SDHC1	9
#define MXC_INT_I2C		10
#define MXC_INT_SSI2		11
#define MXC_INT_SSI1		12
#define MXC_INT_CSPI2		13
#define MXC_INT_CSPI1		14
#define MXC_INT_ATA		15
#define MXC_INT_MBX		16
#define MXC_INT_VPU		MXC_INT_MBX
#define MXC_INT_CSPI3		17
#define MXC_INT_UART3		18
#define MXC_INT_IIM		19
#define MXC_INT_SIM2		20
#define MXC_INT_SIM1		21
#define MXC_INT_RNG		22
#define MXC_INT_EVTMON		23
#define MXC_INT_KPP		24
#define MXC_INT_RTC		25
#define MXC_INT_PWM		26
#define MXC_INT_EPIT2		27
#define MXC_INT_EPIT1		28
#define MXC_INT_GPT		29
#define MXC_INT_RESV30		30
#define MXC_INT_DVFS		31
#define MXC_INT_UART2		32
#define MXC_INT_NANDFC		33
#define MXC_INT_SDMA		34
#define MXC_INT_USB1		35
#define MXC_INT_USB2		36
#define MXC_INT_USB3		37
#define MXC_INT_USB4		38
#define MXC_INT_MSHC1		39
#define MXC_INT_MSHC2		40
#define MXC_INT_IPU_ERR		41
#define MXC_INT_IPU_SYN		42
#define MXC_INT_RESV43		43
#define MXC_INT_RESV44		44
#define MXC_INT_UART1		45
#define MXC_INT_UART4		46
#define MXC_INT_UART5		47
#define MXC_INT_ECT		48
#define MXC_INT_SCC_SCM		49
#define MXC_INT_SCC_SMN		50
#define MXC_INT_GPIO2		51
#define MXC_INT_GPIO1		52
#define MXC_INT_CCM		53
#define MXC_INT_PCMCIA		54
#define MXC_INT_WDOG		55
#define MXC_INT_GPIO3		56
#define MXC_INT_RESV57		57
#define MXC_INT_EXT_POWER	58
#define MXC_INT_EXT_TEMPER	59
#define MXC_INT_EXT_SENSOR60	60
#define MXC_INT_EXT_SENSOR61	61
#define MXC_INT_EXT_WDOG	62
#define MXC_INT_EXT_TV		63

#define MXC_MAX_INT_LINES	64

#define MXC_GPIO_INT_BASE	MXC_MAX_INT_LINES

/*!
 * Interrupt Number for ARM11 PMU
 */
#define ARM11_PMU_IRQ		MXC_INT_EVTMON

/*!
 * Number of GPIO port as defined in the IC Spec
 */
#define GPIO_PORT_NUM		3
/*!
 * Number of GPIO pins per port
 */
#define GPIO_NUM_PIN		32

/*!
 * NFMS bit in RCSR register for pagesize of nandflash
 */
#define NFMS (*((volatile u32 *)IO_ADDRESS(CCM_BASE_ADDR+0xc)))
#define NFMS_BIT 30
#define NFMS_NF_DWIDTH 		31
#define NFMS_NF_PG_SZ 		30

#endif /*  __ASM_ARCH_MXC_MX31_H__ */
