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

#ifndef __ASM_ARM_ARCH_HARDWARE_H
#define __ASM_ARM_ARCH_HARDWARE_H

#ifdef CONFIG_ARCH_MX28
# include <mach/mx28.h>
# define cpu_is_mx28() 1
# else
# define cpu_is_mx28() 0
#endif

#ifdef CONFIG_ARCH_MX23
# include <mach/mx23.h>
# define cpu_is_mx23() 1
# else
# define cpu_is_mx23() 0
#endif

#ifndef MXS_EXTEND_IRQS
#define MXS_EXTEND_IRQS	0
#endif

#ifndef MXS_ARCH_NR_GPIOS
#define MXS_ARCH_NR_GPIOS	160
#endif

#ifndef MXS_EXTEND_NR_GPIOS
#define MXS_EXTEND_NR_GPIOS	0
#endif

#define ARCH_NR_GPIOS	(MXS_ARCH_NR_GPIOS + MXS_EXTEND_NR_GPIOS)

#define MXS_GPIO_IRQ_START	ARCH_NR_IRQS
#define MXS_EXTEND_IRQ_START	(ARCH_NR_IRQS + ARCH_NR_GPIOS)

#endif /* __ASM_ARM_ARCH_HARDWARE_H */
