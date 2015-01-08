/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier: GPL-2.0+ and/or BSD-3-Clause
 * The GPL-2.0+ license for this file can be found in the COPYING.GPL file
 * included with this distribution or at
 * http://www.gnu.org/licenses/gpl-2.0.html
 * The BSD-3-Clause License for this file can be found in the COPYING.BSD file
 * included with this distribution or at
 * http://opensource.org/licenses/BSD-3-Clause
 */

/*
 * Define the phiscal memory address on A9 and shared M4,
 * This definition should be aligned on both A9 and M4
 */
#define MCC_VECTOR_NUMBER_INVALID     (0)

enum {
	INT_CPU_TO_CPU_MU_A2M = 122,
	INT_CPU_TO_CPU_MU_M2A = 90,
};

/* Return core num. A9 0, M4 1 */
unsigned int _psp_core_num(void);
unsigned int _psp_node_num(void);

unsigned int mcc_get_cpu_to_cpu_vector(unsigned int);
/* Defined in MU driver */
void mcc_clear_cpu_to_cpu_interrupt(void);
int mcc_triger_cpu_to_cpu_interrupt(void);
int imx_mcc_bsp_int_disable(void);
int imx_mcc_bsp_int_enable(void);
