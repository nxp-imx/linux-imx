/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * MTD primitives for XIP support. Architecture specific functions
 *
 * Do not include this file directly. It's included from linux/mtd/xip.h
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ARCH_MXC_MTD_XIP_H__
#define __ARCH_MXC_MTD_XIP_H__

#include <linux/clocksource.h>
#include <mach/hardware.h>
#include <mach/system.h>

#define xip_irqpending()        \
        ((__raw_readl(AVIC_NIVECSR) & __raw_readl(AVIC_FIVECSR)) != 0xFFFFFFFF)

extern struct clocksource *mtd_xip_clksrc;

#define xip_currtime()  (unsigned long)clocksource_read(mtd_xip_clksrc)

#if CLOCK_TICK_RATE > 1000000
#define NUMERATOR	1
#define DENOMINATOR	(CLOCK_TICK_RATE/1000000 + 1)
#else
#define NUMERATOR	(1000000/CLOCK_TICK_RATE)
#define DENOMINATOR	1
#endif

static inline unsigned long xip_elapsed_since(unsigned long x)
{
	return (((xip_currtime() - x) * NUMERATOR) / DENOMINATOR);
}

/*
 * Wait For Interrupt command for XIP kernel to put CPU in Idle mode
 */
#define xip_cpu_idle()  arch_idle()

#endif				/* __ARCH_MXC_MTD_XIP_H__ */
