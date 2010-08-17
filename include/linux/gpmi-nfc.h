/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __INCLUDE_LINUX_DEVICE_H
#define __INCLUDE_LINUX_DEVICE_H

/* The name that links together the GPMI NFC driver data structures. */

#define GPMI_NFC_DRIVER_NAME  "gpmi-nfc"

/* Resource names for the GPMI NFC driver. */

#define GPMI_NFC_GPMI_REGS_ADDR_RES_NAME  "GPMI NFC GPMI Registers"
#define GPMI_NFC_GPMI_INTERRUPT_RES_NAME  "GPMI NFC GPMI Interrupt"
#define GPMI_NFC_BCH_REGS_ADDR_RES_NAME   "GPMI NFC BCH Registers"
#define GPMI_NFC_BCH_INTERRUPT_RES_NAME   "GPMI NFC BCH Interrupt"
#define GPMI_NFC_DMA_CHANNELS_RES_NAME    "GPMI NFC DMA Channels"
#define GPMI_NFC_DMA_INTERRUPT_RES_NAME   "GPMI NFC DMA Interrupt"

/**
 * struct gpmi_nfc_platform_data - GPMI NFC driver platform data.
 *
 * This structure communicates platform-specific information to the GPMI NFC
 * driver that can't be expressed as resources.
 *
 * @nfc_version:             The version of the NFC hardware. This single number
 *                           represents a collection of NFC behaviors and
 *                           attributes the driver must comprehend. See the
 *                           driver code for details about what each version
 *                           means.
 * @boot_rom_version:        The version of the Boot ROM. This single number
 *                           represents a collection of Boot ROM behaviors and
 *                           attributes the driver must comprehend. See the
 *                           driver code for details about what each version
 *                           means.
 * @clock_name:              The name of the clock that drives the NFC.
 * @platform_init:           A pointer to a function the driver will call to
 *                           initialize the platform (e.g., set up the pin mux).
 *                           The max_chip_count parameter is the maximum number
 *                           of chips the driver is to support. If the platform
 *                           can't be configured to support this number, the
 *                           function should print a message and return a
 *                           non-zero value . The driver will only view this as
 *                           an indication of failure and will choose its own
 *                           error code to return.
 * @platform_exit:           A pointer to a function the driver will call to
 *                           exit the platform (e.g., free pins). The
 *                           max_chip_count parameter is same value passed to
 *                           the platform init function.
 * @min_prop_delay_in_ns:    Minimum propagation delay of GPMI signals to and
 *                           from the NAND Flash device, in nanoseconds.
 * @max_prop_delay_in_ns:    Maximum propagation delay of GPMI signals to and
 *                           from the NAND Flash device, in nanoseconds.
 * @max_chip_count:          The maximum number of chips for which the driver
 *                           should configure the hardware. This value most
 *                           likely reflects the number of pins that are
 *                           connected to a NAND Flash device. If this is
 *                           greater than the SoC hardware can support, the
 *                           driver will print a message and fail to initialize.
 * @boot_area_size_in_bytes: The amount of space reserved for each boot area.
 *                           Note that some Boot ROMs call for multiple boot
 *                           areas. If this value is zero, the driver will not
 *                           construct special partitions for boot areas.
 * @partition_source_types:  An array of strings that name sources of
 *                           partitioning information (e.g., the boot loader,
 *                           kernel command line, etc.). The function
 *                           parse_mtd_partitions() recognizes these names and
 *                           applies the appropriate "plugins" to discover
 *                           partitioning information. If any is found, it will
 *                           be applied to the "general use" MTD (it will NOT
 *                           override the boot area protection mechanism).
 * @partitions:              An optional pointer to an array of partition
 *                           descriptions. If the driver finds no other
 *                           partitioning information, it will apply these
 *                           partitions to the "general use" MTD (they do NOT
 *                           override the boot area protection mechanism).
 * @partition_count:         The number of elements in the partitions array.
 */

struct gpmi_nfc_platform_data {

	/* Version information. */

	unsigned int          nfc_version;
	unsigned int          boot_rom_version;

	/* SoC hardware information. */

	char                  *clock_name;
	int                   (*platform_init)(unsigned int max_chip_count);
	void                  (*platform_exit)(unsigned int max_chip_count);

	/* NAND Flash information. */

	unsigned int          min_prop_delay_in_ns;
	unsigned int          max_prop_delay_in_ns;
	unsigned int          max_chip_count;

	/* Medium information. */

	uint32_t              boot_area_size_in_bytes;
	const char            **partition_source_types;
	struct mtd_partition  *partitions;
	unsigned              partition_count;

};

#endif
