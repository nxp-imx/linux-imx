/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file pmic/core/mc13783.c
 * @brief This file contains MC13783 specific PMIC code. This implementaion
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
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/spi/spi.h>

#include <asm/uaccess.h>

#include "pmic.h"

/*
 * Defines
 */
#define EVENT_MASK_0			0x697fdf
#define EVENT_MASK_1			0x3efffb
#define MXC_PMIC_FRAME_MASK		0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM		0x3F
#define MXC_PMIC_REG_NUM_SHIFT		0x19
#define MXC_PMIC_WRITE_BIT_SHIFT	31

static unsigned int events_enabled0 = 0;
static unsigned int events_enabled1 = 0;
struct mxc_pmic pmic_drv_data;

/*!
 * This function is called to read a register on PMIC.
 *
 * @param        reg_num     number of the pmic register to be read
 * @param        reg_val   return value of register
 *
 * @return       Returns 0 on success -1 on failure.
 */
int pmic_read(unsigned int reg_num, unsigned int *reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (reg_num > MXC_PMIC_MAX_REG_NUM)
		return PMIC_ERROR;

	frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

	ret = spi_rw(pmic_drv_data.spi, (u8 *) & frame, 1);

	*reg_val = frame & MXC_PMIC_FRAME_MASK;

	return ret;
}

/*!
 * This function is called to write a value to the register on PMIC.
 *
 * @param        reg_num     number of the pmic register to be written
 * @param        reg_val   value to be written
 *
 * @return       Returns 0 on success -1 on failure.
 */
int pmic_write(int reg_num, const unsigned int reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (reg_num > MXC_PMIC_MAX_REG_NUM)
		return PMIC_ERROR;

	frame |= (1 << MXC_PMIC_WRITE_BIT_SHIFT);

	frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

	frame |= reg_val & MXC_PMIC_FRAME_MASK;

	ret = spi_rw(pmic_drv_data.spi, (u8 *) & frame, 1);

	return ret;
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

	spi->mode = SPI_MODE_2 | SPI_CS_HIGH;
	spi->bits_per_word = 32;

	return spi_setup(spi);
}

/*!
 * This function initializes the PMIC registers.
 *
 * @return   None
 */
int pmic_init_registers(void)
{
	CHECK_ERROR(pmic_write(REG_INTERRUPT_MASK_0, MXC_PMIC_FRAME_MASK));
	CHECK_ERROR(pmic_write(REG_INTERRUPT_MASK_1, MXC_PMIC_FRAME_MASK));
	CHECK_ERROR(pmic_write(REG_INTERRUPT_STATUS_0, MXC_PMIC_FRAME_MASK));
	CHECK_ERROR(pmic_write(REG_INTERRUPT_STATUS_1, MXC_PMIC_FRAME_MASK));
	return PMIC_SUCCESS;
}

/*!
 * This function returns the PMIC version in system.
 *
 * @param 	ver	pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */
void pmic_get_revision(pmic_version_t * ver)
{
	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;

	ver->id = PMIC_MC13783;
	pmic_read(REG_REVISION, &rev_id);

	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;

	/* Ver 0.2 is actually 3.2a.  Report as 3.2 */
	if ((rev1 == 0) && (rev2 == 2)) {
		rev1 = 3;
	}

	if (rev1 == 0 || icid != 2) {
		ver->revision = -1;
		printk(KERN_NOTICE
		       "mc13783: Not detected.\tAccess failed\t!!!\n");
	} else {
		ver->revision = ((rev1 * 10) + rev2);
		printk(KERN_INFO "mc13783 Rev %d.%d FinVer %x detected\n", rev1,
		       rev2, finid);
	}

	return;

}

/*!
 * This function reads the interrupt status registers of PMIC
 * and determine the current active events.
 *
 * @param 	active_events array pointer to be used to return active
 *		event numbers.
 *
 * @return       This function returns PMIC version.
 */
unsigned int pmic_get_active_events(unsigned int *active_events)
{
	unsigned int count = 0;
	unsigned int status0, status1;
	int bit_set;

	pmic_read(REG_INTERRUPT_STATUS_0, &status0);
	pmic_read(REG_INTERRUPT_STATUS_1, &status1);
	pmic_write(REG_INTERRUPT_STATUS_0, status0);
	pmic_write(REG_INTERRUPT_STATUS_1, status1);
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

/*!
 * This function unsets a bit in mask register of pmic to unmask an event IT.
 *
 * @param	event 	the event to be unmasked
 *
 * @return    This function returns PMIC_SUCCESS on SUCCESS, error on FAILURE.
 */
int pmic_event_unmask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_E1HZI) {
		mask_reg = REG_INTERRUPT_MASK_0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 |= event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INTERRUPT_MASK_1;
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

/*!
 * This function sets a bit in mask register of pmic to disable an event IT.
 *
 * @param	event 	the event to be masked
 *
 * @return     This function returns PMIC_SUCCESS on SUCCESS, error on FAILURE.
 */
int pmic_event_mask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_E1HZI) {
		mask_reg = REG_INTERRUPT_MASK_0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 &= ~event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INTERRUPT_MASK_1;
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
 * This function is called to read all sensor bits of PMIC.
 *
 * @param        sensor    Sensor to be checked.
 *
 * @return       This function returns true if the sensor bit is high;
 *               or returns false if the sensor bit is low.
 */
bool pmic_check_sensor(t_sensor sensor)
{
	unsigned int reg_val = 0;

	CHECK_ERROR(pmic_read_reg
		    (REG_INTERRUPT_SENSE_0, &reg_val, PMIC_ALL_BITS));

	if ((1 << sensor) & reg_val)
		return true;
	else
		return false;
}

/*!
 * This function checks one sensor of PMIC.
 *
 * @param        sensor_bits  structure of all sensor bits.
 *
 * @return    This function returns PMIC_SUCCESS on SUCCESS, error on FAILURE.
 */

PMIC_STATUS pmic_get_sensors(t_sensor_bits * sensor_bits)
{
	int sense_0 = 0;
	int sense_1 = 0;

	memset(sensor_bits, 0, sizeof(t_sensor_bits));

	pmic_read_reg(REG_INTERRUPT_SENSE_0, &sense_0, 0xffffff);
	pmic_read_reg(REG_INTERRUPT_SENSE_1, &sense_1, 0xffffff);

	sensor_bits->sense_chgdets = (sense_0 & (1 << 6)) ? true : false;
	sensor_bits->sense_chgovs = (sense_0 & (1 << 7)) ? true : false;
	sensor_bits->sense_chgrevs = (sense_0 & (1 << 8)) ? true : false;
	sensor_bits->sense_chgshorts = (sense_0 & (1 << 9)) ? true : false;
	sensor_bits->sense_cccvs = (sense_0 & (1 << 10)) ? true : false;
	sensor_bits->sense_chgcurrs = (sense_0 & (1 << 11)) ? true : false;
	sensor_bits->sense_bpons = (sense_0 & (1 << 12)) ? true : false;
	sensor_bits->sense_lobatls = (sense_0 & (1 << 13)) ? true : false;
	sensor_bits->sense_lobaths = (sense_0 & (1 << 14)) ? true : false;
	sensor_bits->sense_usb4v4s = (sense_0 & (1 << 16)) ? true : false;
	sensor_bits->sense_usb2v0s = (sense_0 & (1 << 17)) ? true : false;
	sensor_bits->sense_usb0v8s = (sense_0 & (1 << 18)) ? true : false;
	sensor_bits->sense_id_floats = (sense_0 & (1 << 19)) ? true : false;
	sensor_bits->sense_id_gnds = (sense_0 & (1 << 20)) ? true : false;
	sensor_bits->sense_se1s = (sense_0 & (1 << 21)) ? true : false;
	sensor_bits->sense_ckdets = (sense_0 & (1 << 22)) ? true : false;

	sensor_bits->sense_onofd1s = (sense_1 & (1 << 3)) ? true : false;
	sensor_bits->sense_onofd2s = (sense_1 & (1 << 4)) ? true : false;
	sensor_bits->sense_onofd3s = (sense_1 & (1 << 5)) ? true : false;
	sensor_bits->sense_pwrrdys = (sense_1 & (1 << 11)) ? true : false;
	sensor_bits->sense_thwarnhs = (sense_1 & (1 << 12)) ? true : false;
	sensor_bits->sense_thwarnls = (sense_1 & (1 << 13)) ? true : false;
	sensor_bits->sense_clks = (sense_1 & (1 << 14)) ? true : false;
	sensor_bits->sense_mc2bs = (sense_1 & (1 << 17)) ? true : false;
	sensor_bits->sense_hsdets = (sense_1 & (1 << 18)) ? true : false;
	sensor_bits->sense_hsls = (sense_1 & (1 << 19)) ? true : false;
	sensor_bits->sense_alspths = (sense_1 & (1 << 20)) ? true : false;
	sensor_bits->sense_ahsshorts = (sense_1 & (1 << 21)) ? true : false;
	return PMIC_SUCCESS;
}

EXPORT_SYMBOL(pmic_check_sensor);
EXPORT_SYMBOL(pmic_get_sensors);
