/*
 *  Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_MXC_BOARD_MX35PDK_H__
#define __ASM_ARCH_MXC_BOARD_MX35PDK_H__

#include <mach/irqs.h>

#define MXC_PSEUDO_PARENT	MXC_INT_FORCE
#define EXPIO_PARENT_INT        IOMUX_TO_IRQ(MX35_GPIO1_1)

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
#define MXC_PSEUDO_IRQ_GPS		(MXC_PSEUDO_IO_BASE + MCU_INT_GPS)
#define MXC_PSEUDO_IRQ_SD1_CD		(MXC_PSEUDO_IO_BASE + MCU_INT_SD1_CD)
#define MXC_PSEUDO_IRQ_SD1_WP		(MXC_PSEUDO_IO_BASE + MCU_INT_SD1_WP)
#define MXC_PSEUDO_IRQ_SD2_CD		(MXC_PSEUDO_IO_BASE + MCU_INT_SD2_CD)
#define MXC_PSEUDO_IRQ_SD2_WP		(MXC_PSEUDO_IO_BASE + MCU_INT_SD2_WP)
#define MXC_PSEUDO_IRQ_POWER_KEY	(MXC_PSEUDO_IO_BASE + MCU_INT_POWER_KEY)
#define MXC_PSEUDO_IRQ_KEYPAD		(MXC_PSEUDO_IO_BASE + MCU_INT_KEYPAD)
#define MXC_PSEUDO_IRQ_RTC		(MXC_PSEUDO_IO_BASE + MCU_INT_RTC)
#define MXC_PSEUDO_IRQ_TS_ADC		(MXC_PSEUDO_IO_BASE + MCU_INT_TS_ADC)

extern int is_suspend_ops_started(void);
extern int __init mx35_3stack_init_mc13892(void);
extern int __init mx35_3stack_init_mc9s08dz60(void);

#endif /* __ASM_ARCH_MXC_BOARD_MX35PDK_H__ */
