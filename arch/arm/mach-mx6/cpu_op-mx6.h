/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*The below value aligned with SPEED_GRADING bits in 0x440 fuse offset */
#define CPU_AT_800MHz		0
#define CPU_AT_1GHz		2
#define CPU_AT_1_2GHz		3

void mx6_cpu_op_init(void);
