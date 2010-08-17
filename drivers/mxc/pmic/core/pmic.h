/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __PMIC_H__
#define __PMIC_H__

 /*!
  * @file pmic.h
  * @brief This file contains prototypes of all the functions to be
  * defined for each PMIC chip. The implementation of these may differ
  * from PMIC chip to PMIC chip.
  *
  * @ingroup PMIC_CORE
  */

#include <linux/spi/spi.h>

#define MAX_ACTIVE_EVENTS		10

/*!
 * This structure is a way for the PMIC core driver to define their own
 * \b spi_device structure. This structure includes the core \b spi_device
 * structure that is provided by Linux SPI Framework/driver as an
 * element and may contain other elements that are required by core driver.
 */
struct mxc_pmic {
	/*!
	 * Master side proxy for an SPI slave device(PMIC)
	 */
	struct spi_device *spi;
};

/*!
 * This function is called to transfer data to PMIC on SPI.
 *
 * @param    spi	the SPI slave device(PMIC)
 * @param    buf   	the pointer to the data buffer
 * @param    len    	the length of the data to be transferred
 *
 * @return   Returns 0 on success -1 on failure.
 */
static inline int spi_rw(struct spi_device *spi, u8 * buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = (const void *)buf,
		.rx_buf = buf,
		.len = len,
		.cs_change = 0,
		.delay_usecs = 0,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	if (spi_sync(spi, &m) != 0 || m.status != 0)
		return PMIC_ERROR;
	return len - m.actual_length;
}

/*!
 * This function returns the PMIC version in system.
 *
 * @param 	ver	pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */
void pmic_get_revision(pmic_version_t *ver);

/*!
 * This function initializes the SPI device parameters for this PMIC.
 *
 * @param    spi	the SPI slave device(PMIC)
 *
 * @return   None
 */
int pmic_spi_setup(struct spi_device *spi);

/*!
 * This function initializes the PMIC registers.
 *
 * @return   None
 */
int pmic_init_registers(void);

/*!
 * This function reads the interrupt status registers of PMIC
 * and determine the current active events.
 *
 * @param 	active_events array pointer to be used to return active
 *		event numbers.
 *
 * @return       This function returns PMIC version.
 */
unsigned int pmic_get_active_events(unsigned int *active_events);

/*!
 * This function sets a bit in mask register of pmic to disable an event IT.
 *
 * @param	event 	the event to be masked
 *
 * @return     This function returns PMIC_SUCCESS on SUCCESS, error on FAILURE.
 */
int pmic_event_mask(type_event event);

/*!
 * This function unsets a bit in mask register of pmic to unmask an event IT.
 *
 * @param	event 	the event to be unmasked
 *
 * @return    This function returns PMIC_SUCCESS on SUCCESS, error on FAILURE.
 */
int pmic_event_unmask(type_event event);

#ifdef CONFIG_MXC_PMIC_FIXARB
extern PMIC_STATUS pmic_fix_arbitration(struct spi_device *spi);
#else
static inline PMIC_STATUS pmic_fix_arbitration(struct spi_device *spi)
{
	return PMIC_SUCCESS;
}
#endif

void *pmic_alloc_data(struct device *dev);

int pmic_start_event_thread(int irq_num);

void pmic_stop_event_thread(void);

#endif				/* __PMIC_H__ */
