/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * Prototypes for iMX6sx-specific MCC library functions.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Define the phiscal memory address on A9 and shared M4,
 * This definition should be aligned on both A9 and M4
 */
#define MCC_VECTOR_NUMBER_INVALID     (0)

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

unsigned int mcc_get_cpu_to_cpu_vector(unsigned int);
void mcc_clear_cpu_to_cpu_interrupt(unsigned int);
void mcc_triger_cpu_to_cpu_interrupt(void);
int imx_mcc_bsp_int_disable(unsigned int vector_number);
int imx_mcc_bsp_int_enable(unsigned int vector_number);
