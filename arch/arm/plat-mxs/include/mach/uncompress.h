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

#ifndef __ASM_PLAT_UNCOMPRESS_H
#define __ASM_PLAT_UNCOMPRESS_H

#include <mach/hardware.h>

/*
 * Register includes are for when the MMU enabled; we need to define our
 * own stuff here for pre-MMU use
 */
#define UART_PORT_BASE		DUART_PHYS_ADDR
#define UART(c)			(((volatile unsigned *)UART_PORT_BASE)[c])

/*
 * This does not append a newline
 */
static void putc(char c)
{
	/* Wait for TX fifo empty */
	while ((UART(6) & (1 << 7)) == 0)
		continue;

	/* Write byte */
	UART(0) = c;

	/* Wait for last bit to exit the UART */
	while (UART(6) & (1 << 3))
		continue;
}

#define flush()	do { } while (0)
/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif /* __ASM_PLAT_UNCOMPRESS_H */
