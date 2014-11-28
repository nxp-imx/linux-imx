/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All Rights Reserved.
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

#define MU_ATR0_OFFSET 0x0
#define MU_ARR0_OFFSET 0x10

#define MU_LPM_HANDSHAKE_INDEX		0
#define MU_LPM_BUS_HIGH_READY_FOR_M4	0xFFFF6666
#define MU_LPM_M4_FREQ_CHANGE_READY	0xFFFF7777
#define MU_LPM_M4_REQUEST_HIGH_BUS	0x2222CCCC
#define MU_LPM_M4_RELEASE_HIGH_BUS	0x2222BBBB
#define MU_LPM_M4_WAKEUP_SRC_VAL	0x55555000
#define MU_LPM_M4_WAKEUP_SRC_MASK	0xFFFFF000
#define MU_LPM_M4_WAKEUP_IRQ_MASK	0xFF0
#define MU_LPM_M4_WAKEUP_IRQ_SHIFT	0x4
#define MU_LPM_M4_WAKEUP_ENABLE_MASK	0xF
#define MU_LPM_M4_WAKEUP_ENABLE_SHIFT	0x0

enum {
	/* FIXME */
	INT_CPU_TO_CPU_MU_A2M = 122,
	INT_CPU_TO_CPU_MU_M2A = 90,

	MU_ASR = 0x20,
	MU_ACR = 0x24,
};

extern struct regmap *imx_mu_reg;

/* Return core num. A9 0, M4 1 */
unsigned int _psp_core_num(void);
unsigned int _psp_node_num(void);

unsigned int mcc_get_cpu_to_cpu_vector(unsigned int);
void mcc_clear_cpu_to_cpu_interrupt(unsigned int);
int mcc_triger_cpu_to_cpu_interrupt(void);
unsigned int mcc_get_mu_irq(void);
unsigned int mcc_handle_mu_receive_irq(void);
void mcc_handle_mu_send_irq(void);
void mcc_enable_receive_irq(unsigned int enable);
int imx_mcc_bsp_int_disable(unsigned int vector_number);
int imx_mcc_bsp_int_enable(unsigned int vector_number);
