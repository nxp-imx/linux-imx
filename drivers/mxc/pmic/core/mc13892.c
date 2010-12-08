/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file pmic/core/mc13892.c
 * @brief This file contains MC13892 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc13892/core.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>

#include "pmic.h"

/*
 * Defines
 */
#define MC13892_I2C_RETRY_TIMES 10
#define MXC_PMIC_FRAME_MASK		0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM		0x3F
#define MXC_PMIC_REG_NUM_SHIFT		0x19
#define MXC_PMIC_WRITE_BIT_SHIFT		31

static unsigned int events_enabled0;
static unsigned int events_enabled1;
static struct mxc_pmic pmic_drv_data;
#ifndef CONFIG_MXC_PMIC_I2C
struct i2c_client *mc13892_client;
#endif

int pmic_i2c_24bit_read(struct i2c_client *client, unsigned int reg_num,
			unsigned int *value)
{
	unsigned char buf[3];
	int ret;
	int i;

	memset(buf, 0, 3);
	for (i = 0; i < MC13892_I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_read_i2c_block_data(client, reg_num, 3, buf);
		if (ret == 3)
			break;
		msleep(1);
	}

	if (ret == 3) {
		*value = buf[0] << 16 | buf[1] << 8 | buf[2];
		return ret;
	} else {
		pr_err("24bit read error, ret = %d\n", ret);
		return -1;	/* return -1 on failure */
	}
}

int pmic_i2c_24bit_write(struct i2c_client *client,
			 unsigned int reg_num, unsigned int reg_val)
{
	char buf[3];
	int ret;
	int i;

	buf[0] = (reg_val >> 16) & 0xff;
	buf[1] = (reg_val >> 8) & 0xff;
	buf[2] = (reg_val) & 0xff;

	for (i = 0; i < MC13892_I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_write_i2c_block_data(client, reg_num, 3, buf);
		if (ret == 0)
			break;
		msleep(1);
	}
	if (i == MC13892_I2C_RETRY_TIMES)
		pr_err("24bit write error, ret = %d\n", ret);

	return ret;
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
		if (mc13892_client == NULL)
			return PMIC_ERROR;

		if (pmic_i2c_24bit_read(mc13892_client, reg_num, reg_val) == -1)
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
		if (mc13892_client == NULL)
			return PMIC_ERROR;

		return pmic_i2c_24bit_write(mc13892_client, reg_num, reg_val);
	}
}

void *pmic_alloc_data(struct device *dev)
{
	struct mc13892 *mc13892;

	mc13892 = kzalloc(sizeof(struct mc13892), GFP_KERNEL);
	if (mc13892 == NULL)
		return NULL;

	mc13892->dev = dev;

	return (void *)mc13892;
}

/*!
 * This function initializes the SPI device parameters for this PMIC.
 *
 * @param    spi	the SPI slave device(PMIC)
 *
 * @return   None
 */
int pmic_spi_setup(struct spi_device *spi)
{
	/* Setup the SPI slave i.e.PMIC */
	pmic_drv_data.spi = spi;

	spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	spi->bits_per_word = 32;

	return spi_setup(spi);
}

int pmic_init_registers(void)
{
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS1, 0xFFFFFF));
	/* disable auto charge */
	if (machine_is_mx51_3ds())
		CHECK_ERROR(pmic_write(REG_CHARGE, 0xB40003));

	pm_power_off = mc13892_power_off;

	return PMIC_SUCCESS;
}

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

/*!
 * This function returns the PMIC version in system.
 *
 * @param 	ver	pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */
void pmic_get_revision(pmic_version_t *ver)
{
	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;

	ver->id = PMIC_MC13892;
	pmic_read(REG_IDENTIFICATION, &rev_id);

	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;

	ver->revision = ((rev1 * 10) + rev2);
	printk(KERN_INFO "mc13892 Rev %d.%d FinVer %x detected\n", rev1,
	       rev2, finid);
}

void mc13892_power_off(void)
{
	unsigned int value;

	pmic_read_reg(REG_POWER_CTL0, &value, 0xffffff);

	value |= 0x000008;

	pmic_write_reg(REG_POWER_CTL0, value, 0xffffff);
}
