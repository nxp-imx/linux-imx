extern void mx51_cpu_op_init(void);
/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ARCH_ARM_MACH_MX53_WP_H__
#define  __ARCH_ARM_MACH_MX53_WP_H__
#include <linux/types.h>

/*!
 * @file mach-mx5/cpu_op-mx53.h
 *
 * @brief This file contains the information about MX53 CPU working points.
 *
 * @ingroup MSL_MX53
 */
enum mx53_cpu_part_number {
  IMX53_AEC,  /* automative and infotainment AP */
  IMX53_CEC,  /* Consumer AP, CPU freq is up to 1G */
  IMX53_CEC_1_2G, /* Consumer AP, CPU freq is up to 1.2GHZ */
};

void mx53_set_cpu_part_number(enum mx53_cpu_part_number part_num);
void mx53_cpu_op_init(void);

#endif /*__ARCH_ARM_MACH_MX53_WP_H__ */



