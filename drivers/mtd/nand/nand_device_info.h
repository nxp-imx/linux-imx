/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __DRIVERS_NAND_DEVICE_INFO_H
#define __DRIVERS_NAND_DEVICE_INFO_H

 /*
  * The number of ID bytes to read from the NAND Flash device and hand over to
  * the identification system.
  */

#define NAND_DEVICE_ID_BYTE_COUNT  (6)

 /*
  * The number of ID bytes to read from the NAND Flash device and hand over to
  * the identification system.
  */

enum nand_device_cell_technology {
	NAND_DEVICE_CELL_TECH_SLC = 0,
	NAND_DEVICE_CELL_TECH_MLC = 1,
};

/**
 * struct nand_device_info - Information about a single NAND Flash device.
 *
 * This structure contains all the *essential* information about a NAND Flash
 * device, derived from the device's data sheet. For each manufacturer, we have
 * an array of these structures.
 *
 * @end_of_table:              If true, marks the end of a table of device
 *                             information.
 * @manufacturer_code:         The manufacturer code (1st ID byte) reported by
 *                             the device.
 * @device_code:               The device code (2nd ID byte) reported by the
 *                             device.
 * @is_ddr_ok:                 Is this nand an ONFI nand or TOGGLE nand ?
 * @cell_technology:           The storage cell technology.
 * @chip_size_in_bytes:        The total size of the storage behind a single
 *                             chip select, in bytes. Notice that this is *not*
 *                             necessarily the total size of the storage in a
 *                             *package*, which may contain several chips.
 * @block_size_in_pages:       The number of pages in a block.
 * @page_total_size_in_bytes:  The total size of a page, in bytes, including
 *                             both the data and the OOB.
 * @ecc_strength_in_bits:      The strength of the ECC called for by the
 *                             manufacturer, in number of correctable bits.
 * @ecc_size_in_bytes:         The size of the data block over which the
 *                             manufacturer calls for the given ECC algorithm
 *                             and strength.
 * @data_setup_in_ns:          The data setup time, in nanoseconds. Usually the
 *                             maximum of tDS and tWP. A negative value
 *                             indicates this characteristic isn't known.
 * @data_hold_in_ns:           The data hold time, in nanoseconds. Usually the
 *                             maximum of tDH, tWH and tREH. A negative value
 *                             indicates this characteristic isn't known.
 * @address_setup_in_ns:       The address setup time, in nanoseconds. Usually
 *                             the maximum of tCLS, tCS and tALS. A negative
 *                             value indicates this characteristic isn't known.
 * @gpmi_sample_delay_in_ns:   A GPMI-specific timing parameter. A negative
 *                             value indicates this characteristic isn't known.
 * @tREA_in_ns:                tREA, in nanoseconds, from the data sheet. A
 *                             negative value indicates this characteristic
 *                             isn't known.
 * @tRLOH_in_ns:               tRLOH, in nanoseconds, from the data sheet. A
 *                             negative value indicates this characteristic
 *                             isn't known.
 * @tRHOH_in_ns:               tRHOH, in nanoseconds, from the data sheet. A
 *                             negative value indicates this characteristic
 *                             isn't known.
 */

struct nand_device_info {

	/* End of table marker */

	bool      end_of_table;

	/* Manufacturer and Device codes */

	uint8_t   manufacturer_code;
	uint8_t   device_code;

	/* Does the nand support DDR? (ONFI or TOGGLE) */
	bool	  is_ddr_ok;

	/* Technology */

	enum nand_device_cell_technology  cell_technology;

	/* Geometry */

	uint64_t  chip_size_in_bytes;
	uint32_t  block_size_in_pages;
	uint16_t  page_total_size_in_bytes;

	/* ECC */

	uint8_t   ecc_strength_in_bits;
	uint16_t  ecc_size_in_bytes;

	/* Timing */

	int8_t    data_setup_in_ns;
	int8_t    data_hold_in_ns;
	int8_t    address_setup_in_ns;
	int8_t    gpmi_sample_delay_in_ns;
	int8_t    tREA_in_ns;
	int8_t    tRLOH_in_ns;
	int8_t    tRHOH_in_ns;

	/* Description */

	const char  *description;

};

/**
 * nand_device_get_info - Get info about a device based on ID bytes.
 *
 * @id_bytes:  An array of NAND_DEVICE_ID_BYTE_COUNT ID bytes retrieved from the
 *             NAND Flash device.
 */

struct nand_device_info *nand_device_get_info(const uint8_t id_bytes[]);

/**
 * nand_device_print_info - Prints information about a NAND Flash device.
 *
 * @info  A pointer to a NAND Flash device information structure.
 */

void nand_device_print_info(struct nand_device_info *info);

/*
 * Check the NAND whether it supports the DDR mode.
 * Only the ONFI nand and TOGGLE nand support the DDR now.
 */
static inline bool is_ddr_nand(struct nand_device_info *info)
{
	return info->is_ddr_ok;
}
#endif
