/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __INCLUDE_GPMI_NFC_H
#define __INCLUDE_GPMI_NFC_H

/* The size of the resource is fixed. */
#define RES_SIZE	6

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
 * @platform_init:           A pointer to a function the driver will call to
 *                           initialize the platform (e.g., set up the pin mux).
 * @platform_exit:           A pointer to a function the driver will call to
 *                           exit the platform (e.g., free pins).
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
 * @partitions:              An optional pointer to an array of partition
 *                           descriptions.
 * @partition_count:         The number of elements in the partitions array.
 */
struct gpmi_nfc_platform_data {
	/* SoC hardware information. */
	int		(*platform_init)(void);
	void		(*platform_exit)(void);

	/* NAND Flash information. */
	unsigned int	min_prop_delay_in_ns;
	unsigned int	max_prop_delay_in_ns;
	unsigned int	max_chip_count;

	/* Medium information. */
	struct mtd_partition *partitions;
	unsigned	partition_count;
};
#endif
