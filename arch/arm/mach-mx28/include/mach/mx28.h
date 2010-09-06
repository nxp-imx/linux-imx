/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MACH_MX28_H__
#define __ASM_ARCH_MACH_MX28_H__

#include <asm/sizes.h>

#define MX28_SOC_IO_PHYS_BASE	0x80000000
#define MX28_SOC_IO_VIRT_BASE	0xF0000000
#define MX28_SOC_IO_AREA_SIZE	SZ_1M

/* Virtual address where OCRAM is mapped */
#define MX28_OCRAM_PHBASE   0x00000000
#ifdef __ASSEMBLER__
#define MX28_OCRAM_BASE     0xf1000000
#else
#define MX28_OCRAM_BASE     (void __iomem *)0xf1000000
#endif
#define MX28_OCRAM_SIZE     (128 * SZ_1K)


#define ICOLL_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x000000)
#define HSADC_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x002000)
#define APBH_DMA_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x004000)
#define PERFMON_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x006000)
#define BCH_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x00A000)
#define GPMI_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x00C000)
#define SSP0_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x010000)
#define SSP1_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x012000)
#define SSP2_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x014000)
#define SSP3_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x016000)
#define PINCTRL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x018000)
#define DIGCTL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x01C000)
#define ETM_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x022000)
#define APBX_DMA_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x024000)
#define DCP_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x028000)
#define PXP_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x02A000)
#define OCOTP_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x02C000)
#define AXI_AHB0_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x02E000)
#define LCDIF_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x030000)
#define CAN0_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x032000)
#define CAN1_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x034000)
#define SIMDBG_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C000)
#define SIMGPMISEL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C200)
#define SIMSSPSEL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C300)
#define SIMMEMSEL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C400)
#define GPIOMON_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C500)
#define SIMENET_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C700)
#define ARMJTAG_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x03C800)
#define CLKCTRL_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x040000)
#define SAIF0_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x042000)
#define POWER_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x044000)
#define SAIF1_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x046000)
#define LRADC_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x050000)
#define SPDIF_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x054000)
#define RTC_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x056000)
#define I2C0_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x058000)
#define I2C1_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x05A000)
#define PWM_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x064000)
#define TIMROT_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x068000)
#define AUART0_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x06A000)
#define AUART1_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x06C000)
#define AUART2_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x06E000)
#define AUART3_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x070000)
#define AUART4_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x072000)
#define DUART_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x074000)
#define USBPHY0_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x07C000)
#define USBPHY1_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x07E000)
#define USBCTRL0_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x080000)
#define USBCTRL1_PHYS_ADDR	(MX28_SOC_IO_PHYS_BASE + 0x090000)
#define DFLPT_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x0C0000)
#define DRAM_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x0E0000)
#define ENET_PHYS_ADDR		(MX28_SOC_IO_PHYS_BASE + 0x0F0000)

/* IRQ Definitions */
#define IRQ_BATT_BRNOUT			0
#define IRQ_VDDD_BRNOUT			1
#define IRQ_VDDIO_BRNOUT		2
#define IRQ_VDDA_BRNOUT			3
#define IRQ_VDD5V_DROOP			4
#define IRQ_DCDC4P2_BRNOUT		5
#define IRQ_VDD5V			6
#define IRQ_RESV7			7
#define IRQ_CAN0			8
#define IRQ_CAN1			9
#define IRQ_LRADC_TOUCH			10
#define IRQ_RESV11			11
#define IRQ_RESV12			12
#define IRQ_HSADC			13
#define IRQ_IRADC_THRESH0		14
#define IRQ_IRADC_THRESH1		15
#define IRQ_LRADC_CH0			16
#define IRQ_LRADC_CH1			17
#define IRQ_LRADC_CH2			18
#define IRQ_LRADC_CH3			19
#define IRQ_LRADC_CH4			20
#define IRQ_LRADC_CH5			21
#define IRQ_LRADC_CH6			22
#define IRQ_LRADC_CH7			23
#define IRQ_LRADC_BUTTON0		24
#define IRQ_LRADC_BUTTON1		25
#define IRQ_RESV26			26
#define IRQ_PERFMON			27
#define IRQ_RTC_1MSEC			28
#define IRQ_RTC_ALARM			29
#define IRQ_RESV30			30
#define IRQ_COMMS			31
#define IRQ_EMI_ERR			32
#define IRQ_RESV33			33
#define IRQ_RESV34			34
#define IRQ_RESV35			35
#define IRQ_RESV36			36
#define IRQ_RESV37			37
#define IRQ_LCDIF			38
#define IRQ_PXP				39
#define IRQ_RESV40			40
#define IRQ_BCH				41
#define IRQ_GPMI			42
#define IRQ_RESV43			43
#define IRQ_RESV44			44
#define IRQ_SPDIF_ERROR			45
#define IRQ_RESV46			46
#define IRQ_DUART			47
#define IRQ_TIMER0			48
#define IRQ_TIMER1			49
#define IRQ_TIMER2			50
#define IRQ_TIMER3			51
#define IRQ_DCP_VMI			52
#define IRQ_DCP				53
#define IRQ_DCP_SECURE			54
#define IRQ_RESV55			55
#define IRQ_RESV56			56
#define IRQ_RESV57			57
#define IRQ_SAIF1			58
#define IRQ_SAIF0			59
#define IRQ_RESV60			60
#define IRQ_RESV61			61
#define IRQ_RESV62			62
#define IRQ_RESV63			63
#define IRQ_RESV64			64
#define IRQ_RESV65			65
#define IRQ_SPDIF_DMA			66
#define IRQ_RESV67			67
#define IRQ_I2C0_DMA			68
#define IRQ_I2C1_DMA			69
#define IRQ_AUART0_RX_DMA		70
#define IRQ_AUART0_TX_DMA		71
#define IRQ_AUART1_RX_DMA		72
#define IRQ_AUART1_TX_DMA		73
#define IRQ_AUART2_RX_DMA		74
#define IRQ_AUART2_TX_DMA		75
#define IRQ_AUART3_RX_DMA		76
#define IRQ_AUART3_TX_DMA		77
#define IRQ_AUART4_RX_DMA		78
#define IRQ_AUART4_TX_DMA		79
#define IRQ_SAIF0_DMA			80
#define IRQ_SAIF1_DMA			81
#define IRQ_SSP0_DMA			82
#define IRQ_SSP1_DMA			83
#define IRQ_SSP2_DMA			84
#define IRQ_SSP3_DMA			85
#define IRQ_LCDIF_DMA			86
#define IRQ_HSADC_DMA			87
#define IRQ_GPMI_DMA			88
#define IRQ_DIGCTL_DEBUG_TRAP		89
#define IRQ_RESV90			90
#define IRQ_RESV91			91
#define IRQ_USB1			92
#define IRQ_USB0			93
#define IRQ_USB1_WAKEUP			94
#define IRQ_USB0_WAKEUP			95
#define IRQ_SSP0			96
#define IRQ_SSP1			97
#define IRQ_SSP2			98
#define IRQ_SSP3			99
#define IRQ_ENET_SWI			100
#define IRQ_ENET_MAC0			101
#define IRQ_ENET_MAC1			102
#define IRQ_ENET_MAC0_1588		103
#define IRQ_ENET_MAC1_1588		104
#define IRQ_RESV105			105
#define IRQ_RESV106			106
#define IRQ_RESV107			107
#define IRQ_RESV108			108
#define IRQ_RESV109			109
#define IRQ_I2C1_ERROR			110
#define IRQ_I2C0_ERROR			111
#define IRQ_AUART0			112
#define IRQ_AUART1			113
#define IRQ_AUART2			114
#define IRQ_AUART3			115
#define IRQ_AUART4			116
#define IRQ_RESV117			117
#define IRQ_RESV118			118
#define IRQ_RESV119			119
#define IRQ_RESV120			120
#define IRQ_RESV121			121
#define IRQ_RESV122			122
#define IRQ_GPIO4			123
#define IRQ_GPIO3			124
#define IRQ_GPIO2			125
#define IRQ_GPIO1			126
#define IRQ_GPIO0			127

#define ARCH_NR_IRQS		128

/* On i.MX28, all interrupt sources can be configured as FIQ */
#define FIQ_START		IRQ_BATT_BRNOUT

#define MX28_SOC_IO_ADDRESS(x) \
	((x) - MX28_SOC_IO_PHYS_BASE + MX28_SOC_IO_VIRT_BASE)

#ifdef __ASSEMBLER__
#define IO_ADDRESS(x)		\
		MX28_SOC_IO_ADDRESS(x)
#else
#define IO_ADDRESS(x)		\
	(void __force __iomem *) 	\
	(((x) >= (unsigned long)MX28_SOC_IO_PHYS_BASE) && \
	 ((x) < (unsigned long)MX28_SOC_IO_PHYS_BASE + \
				MX28_SOC_IO_AREA_SIZE) ? \
		MX28_SOC_IO_ADDRESS(x) : 0xDEADBEEF)
#endif

#ifdef CONFIG_MXS_EARLY_CONSOLE
#define MXS_DEBUG_CONSOLE_PHYS DUART_PHYS_ADDR
#define MXS_DEBUG_CONSOLE_VIRT IO_ADDRESS(DUART_PHYS_ADDR)
#endif
#endif /* __ASM_ARCH_MACH_MX28_H__ */
