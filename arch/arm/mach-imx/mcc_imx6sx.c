/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * iMX6sx-specific MCC library functions.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>

/*!
 * \brief This function returns the core number
 *
 * \return int
 */
unsigned int _psp_core_num(void)
{
    return 0;
}

/*
 * This field contains CPU-to-CPU interrupt vector numbers
 * for all device cores.
 */
static const unsigned int mcc_cpu_to_cpu_vectors[] = {INT_CPU_TO_CPU_MU_A2M,
	INT_CPU_TO_CPU_MU_M2A};

/*!
 * \brief This function gets the CPU-to-CPU vector num for the particular core.
 *
 * Platform-specific inter-CPU vector numbers for each core are defined in the
 * mcc_cpu_to_cpu_vectors[] field.
 *
 * \param[in] core Core number.
 *
 * \return vector number for the particular core
 * \return MCC_VECTOR_NUMBER_INVALID (vector number for the particular core
 * number not found)
 */
unsigned int mcc_get_cpu_to_cpu_vector(unsigned int core)
{
	u32 val;

	val = sizeof(mcc_cpu_to_cpu_vectors)/sizeof(mcc_cpu_to_cpu_vectors[0]);
	if (core <  val)
		return  mcc_cpu_to_cpu_vectors[core];

	return MCC_VECTOR_NUMBER_INVALID;
}

/*!
 * \brief This function clears the CPU-to-CPU int flag for the particular core.
 *
 * Implementation is platform-specific.
 *
 * \param[in] core Core number.
 */
void mcc_clear_cpu_to_cpu_interrupt(unsigned int core)
{
	u32 val;

	regmap_read(imx_mu_reg, MU_ASR, &val);
	/* write 1 to BIT31 to clear the bit31(GIP3) of MU_ASR */
	val = val | (1 << 31);
	regmap_write(imx_mu_reg, MU_ASR, val);
}

/*!
 * \brief This function triggers the CPU-to-CPU interrupt.
 *
 * Platform-specific software triggering the inter-CPU interrupts.
 */
void mcc_triger_cpu_to_cpu_interrupt(void)
{
	int i = 0;
	u32 val;

	regmap_read(imx_mu_reg, MU_ACR, &val);

	if ((val & BIT(19)) != 0) {
		do {
			regmap_read(imx_mu_reg, MU_ACR, &val);
			msleep(1);
		} while (((val & BIT(19)) > 0) && (i++ < 100));
	}

	if ((val & BIT(19)) == 0)
		/* Enable the bit19(GIR3) of MU_ACR */
		regmap_update_bits(imx_mu_reg, MU_ACR, BIT(19), BIT(19));
	else
		pr_info("mcc int still be triggered after %d ms polling!\n", i);
}

/*!
 * \brief This function disable the CPU-to-CPU interrupt.
 *
 * Platform-specific software disable the inter-CPU interrupts.
 */
int imx_mcc_bsp_int_disable(unsigned int vector_number)
{
	u32 val;

	if (vector_number == INT_CPU_TO_CPU_MU_A2M) {
		/* Disable the bit31(GIE3) of MU_ACR */
		regmap_update_bits(imx_mu_reg, MU_ACR, BIT(31), 0);
		/* flush */
		regmap_read(imx_mu_reg, MU_ACR, &val);
	} else
		pr_err("ERR:wrong vector number in mcc.\n");

	return 0;
}

/*!
 * \brief This function enable the CPU-to-CPU interrupt.
 *
 * Platform-specific software enable the inter-CPU interrupts.
 */
int imx_mcc_bsp_int_enable(unsigned int vector_number)
{
	u32 val;

	if (vector_number == INT_CPU_TO_CPU_MU_A2M) {
		/* Enable the bit31(GIE3) of MU_ACR */
		regmap_update_bits(imx_mu_reg, MU_ACR, BIT(31), BIT(31));
		/* flush */
		regmap_read(imx_mu_reg, MU_ACR, &val);
		return 0;
	} else {
		pr_err("ERR:wrong vector number in mcc.\n");
		return -EINVAL;
	}
}
