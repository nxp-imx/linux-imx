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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>

#include "mcc_config.h"

/*!
 * \brief This function returns the core number
 *
 * \return int
 */
unsigned int _psp_core_num(void)
{
#if (MCC_OS_USED == MCC_MQX)
    return 1;
#elif (MCC_OS_USED == MCC_LINUX)
    return 0;
#endif
}

/*!
 * \brief This function returns the node number
 *
 * \return unsigned int
 */
unsigned int _psp_node_num(void)
{
#if (MCC_OS_USED == MCC_MQX)
    return MCC_MQX_NODE_NUMBER ;
#elif (MCC_OS_USED == MCC_LINUX)
    return MCC_LINUX_NODE_NUMBER;
#endif
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

unsigned int mcc_get_mu_irq(void)
{
	u32 val;

	regmap_read(imx_mu_reg, MU_ASR, &val);

	return val;
}

void mcc_enable_receive_irq(unsigned int enable)
{
	u32 val = enable ? BIT(27) : 0;

	regmap_update_bits(imx_mu_reg, MU_ACR, BIT(27), val);
}

void mcc_send_via_mu_buffer(unsigned int index, unsigned int data)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	/* wait for transfer buffer empty */
	do {
		regmap_read(imx_mu_reg, MU_ASR, &val);
		if (time_after(jiffies, timeout)) {
			pr_err("Waiting MU transmit buffer empty timeout!\n");
			break;
		}
	} while ((val & (1 << (20 + index))) == 0);

	regmap_write(imx_mu_reg, index * 0x4 + MU_ATR0_OFFSET, data);
}

void mcc_receive_from_mu_buffer(unsigned int index, unsigned int *data)
{
	regmap_read(imx_mu_reg, index * 0x4 + MU_ARR0_OFFSET, data);
}

unsigned int mcc_handle_mu_receive_irq(void)
{
	u32 val;

	/* disable receive irq until last message is handled */
	mcc_enable_receive_irq(0);
	regmap_read(imx_mu_reg, MU_ARR0_OFFSET, &val);

	return val;
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
int mcc_triger_cpu_to_cpu_interrupt(void)
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

	if ((val & BIT(19)) == 0) {
		/* Enable the bit19(GIR3) of MU_ACR */
		regmap_update_bits(imx_mu_reg, MU_ACR, BIT(19), BIT(19));
		return 0;
	} else {
		pr_info("mcc int still be triggered after %d ms polling!\n", i);
		return -EIO;
	}
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
		regmap_update_bits(imx_mu_reg, MU_ACR, BIT(31) | BIT(27), 0);
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
		regmap_update_bits(imx_mu_reg, MU_ACR,
			BIT(31) | BIT(27), BIT(31) | BIT(27));
		/* flush */
		regmap_read(imx_mu_reg, MU_ACR, &val);
		return 0;
	} else {
		pr_err("ERR:wrong vector number in mcc.\n");
		return -EINVAL;
	}
}
