/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*!
 * @file pmic_external.c
 * @brief This file contains all external functions of PMIC drivers.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/errno.h>

#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include "pmic.h"

#define MXC_PMIC_FRAME_MASK		0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM		0x3F
#define MXC_PMIC_REG_NUM_SHIFT		0x19
#define MXC_PMIC_WRITE_BIT_SHIFT	31

struct mxc_pmic pmic_drv_data;
static unsigned int events_enabled0;
static unsigned int events_enabled1;

#ifdef CONFIG_MXC_PMIC_MC34704
extern int pmic_read(int reg_num, unsigned int *reg_val);
extern int pmic_write(int reg_num, const unsigned int reg_val);
#else
int pmic_spi_setup(struct spi_device *spi)
{
	/* Setup the SPI slave i.e.PMIC */
	pmic_drv_data.spi = spi;

	spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	spi->bits_per_word = 32;
	return spi_setup(spi);
}

int pmic_read(int reg_num, unsigned int *reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (pmic_drv_data.spi != NULL) {
		if (reg_num > MXC_PMIC_MAX_REG_NUM)
			return PMIC_ERROR;

		frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

		ret = spi_rw(pmic_drv_data.spi, (u8 *) &frame, 1);

		*reg_val = frame & MXC_PMIC_FRAME_MASK;
	} else {
		pr_err("SPI dev Not set\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

int pmic_write(int reg_num, const unsigned int reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (pmic_drv_data.spi != NULL) {
		if (reg_num > MXC_PMIC_MAX_REG_NUM)
			return PMIC_ERROR;

		frame |= (1 << MXC_PMIC_WRITE_BIT_SHIFT);

		frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

		frame |= reg_val & MXC_PMIC_FRAME_MASK;

		ret = spi_rw(pmic_drv_data.spi, (u8 *) &frame, 1);

		return ret;
	} else {
		pr_err("SPI dev Not set\n");
		return PMIC_ERROR;
	}
}
#endif

/*!
 * This function is called by PMIC clients to read a register on PMIC.
 *
 * @param        reg        number of register
 * @param        reg_value  return value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_read_reg(int reg, unsigned int *reg_value,
			  unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = pmic_read(reg, &temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}
	*reg_value = (temp & reg_mask);

	pr_debug("Read REG[ %d ] = 0x%x\n", reg, *reg_value);

	return ret;
}

/*!
 * This function is called by PMIC clients to write a register on PMIC.
 *
 * @param        reg        number of register
 * @param        reg_value  New value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_write_reg(int reg, unsigned int reg_value,
			   unsigned int reg_mask)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = pmic_read(reg, &temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}
	temp = (temp & (~reg_mask)) | reg_value;
#ifdef CONFIG_MXC_PMIC_MC13783
	if (reg == REG_POWER_MISCELLANEOUS)
		temp &= 0xFFFE7FFF;
#endif
	ret = pmic_write(reg, temp);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}

	pr_debug("Write REG[ %d ] = 0x%x\n", reg, reg_value);

	return ret;
}

#ifndef CONFIG_MXC_PMIC_MC34704
unsigned int pmic_get_active_events(unsigned int *active_events)
{
	unsigned int count = 0;
	unsigned int status0, status1;
	int bit_set;

	pmic_read(REG_INT_STATUS0, &status0);
	pmic_read(REG_INT_STATUS1, &status1);
	pmic_write(REG_INT_STATUS0, status0);
	pmic_write(REG_INT_STATUS1, status1);
	status0 &= events_enabled0;
	status1 &= events_enabled1;

	while (status0) {
		bit_set = ffs(status0) - 1;
		*(active_events + count) = bit_set;
		count++;
		status0 ^= (1 << bit_set);
	}
	while (status1) {
		bit_set = ffs(status1) - 1;
		*(active_events + count) = bit_set + 24;
		count++;
		status1 ^= (1 << bit_set);
	}

	return count;
}
EXPORT_SYMBOL(pmic_get_active_events);

#define EVENT_MASK_0			0x387fff
#define EVENT_MASK_1			0x1177ef

int pmic_event_unmask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_1HZI) {
		mask_reg = REG_INT_MASK0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 |= event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INT_MASK1;
		event_mask = EVENT_MASK_1;
		event_bit = (1 << event);
		events_enabled1 |= event_bit;
	}

	if ((event_bit & event_mask) == 0) {
		pr_debug("Error: unmasking a reserved/unused event\n");
		return PMIC_ERROR;
	}

	ret = pmic_write_reg(mask_reg, 0, event_bit);

	pr_debug("Enable Event : %d\n", event);

	return ret;
}
EXPORT_SYMBOL(pmic_event_unmask);

int pmic_event_mask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_1HZI) {
		mask_reg = REG_INT_MASK0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 &= ~event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INT_MASK1;
		event_mask = EVENT_MASK_1;
		event_bit = (1 << event);
		events_enabled1 &= ~event_bit;
	}

	if ((event_bit & event_mask) == 0) {
		pr_debug("Error: masking a reserved/unused event\n");
		return PMIC_ERROR;
	}

	ret = pmic_write_reg(mask_reg, event_bit, event_bit);

	pr_debug("Disable Event : %d\n", event);

	return ret;
}
EXPORT_SYMBOL(pmic_event_mask);
#endif

EXPORT_SYMBOL(pmic_read_reg);
EXPORT_SYMBOL(pmic_write_reg);
