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

#ifndef __ASM_ARCH_MACH_MX23_H___
#define __ASM_ARCH_MACH_MX23_H___

#include <asm/sizes.h>
#include <mach/irqs.h>

#define MX23_SOC_IO_PHYS_BASE	0x80000000
#define MX23_SOC_IO_VIRT_BASE	0xF0000000
#define MX23_SOC_IO_AREA_SIZE	SZ_1M

/* Virtual address where OCRAM is mapped */
#define MX23_OCRAM_PHBASE   0x00000000
#ifdef __ASSEMBLER__
#define MX23_OCRAM_BASE     0xf1000000
#else
#define MX23_OCRAM_BASE     (void __iomem *)0xf1000000
#endif
#define MX23_OCRAM_SIZE     (32 * SZ_1K)

#define ICOLL_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x000000)
#define APBH_DMA_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x004000)
#define BCH_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x00A000)
#define GPMI_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x00C000)
#define SSP1_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x010000)
#define SSP2_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x034000)
#define PINCTRL_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x018000)
#define DIGCTL_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x01C000)
#define ETM_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x020000)
#define APBX_DMA_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x024000)
#define DCP_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x028000)
#define PXP_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x02A000)
#define OCOTP_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x02C000)
#define AXI_AHB0_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x02E000)
#define LCDIF_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x030000)
#define TVENC_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x038000)
#define CLKCTRL_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x040000)
#define SAIF0_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x042000)
#define POWER_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x044000)
#define SAIF1_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x046000)
#define AUDIOOUT_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x048000)
#define AUDIOIN_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x04c000)
#define LRADC_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x050000)
#define SPDIF_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x054000)
#define RTC_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x05c000)
#define I2C0_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x058000)
#define PWM_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x064000)
#define TIMROT_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x068000)
#define AUART1_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x06C000)
#define AUART2_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x06E000)
#define DUART_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x070000)
#define USBPHY_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x07C000)
#define USBCTRL_PHYS_ADDR	(MX23_SOC_IO_PHYS_BASE + 0x080000)
#define DRAM_PHYS_ADDR		(MX23_SOC_IO_PHYS_BASE + 0x0E0000)

#define MX23_SOC_IO_ADDRESS(x) \
	((x) - MX23_SOC_IO_PHYS_BASE + MX23_SOC_IO_VIRT_BASE)

#ifdef __ASSEMBLER__
#define IO_ADDRESS(x)		\
		MX23_SOC_IO_ADDRESS(x)
#else
#define IO_ADDRESS(x)		\
	(void __force __iomem *) 	\
	(((x) >= (unsigned long)MX23_SOC_IO_PHYS_BASE) && \
	 ((x) < (unsigned long)MX23_SOC_IO_PHYS_BASE + \
				MX23_SOC_IO_AREA_SIZE) ? \
		MX23_SOC_IO_ADDRESS(x) : 0xDEADBEEF)
#endif

#ifdef CONFIG_MXS_EARLY_CONSOLE
#define MXS_DEBUG_CONSOLE_PHYS DUART_PHYS_ADDR
#define MXS_DEBUG_CONSOLE_VIRT IO_ADDRESS(DUART_PHYS_ADDR)
#endif

#ifdef CONFIG_DEBUG_LL
#define MXS_LL_UART_PADDR DUART_PHYS_ADDR
#define MXS_LL_UART_VADDR MX23_SOC_IO_ADDRESS(DUART_PHYS_ADDR)
#endif

#endif /* __ASM_ARCH_MACH_MX23_H__ */
