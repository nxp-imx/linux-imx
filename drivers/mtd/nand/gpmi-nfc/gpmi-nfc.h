/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc.
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

#ifndef __DRIVERS_MTD_NAND_GPMI_NFC_H
#define __DRIVERS_MTD_NAND_GPMI_NFC_H

#include "../nand_device_info.h"
#include "gpmi-nfc-v0-gpmi-regs.h"
#include "gpmi-nfc-v1-gpmi-regs.h"
#include "gpmi-nfc-v0-bch-regs.h"
#include "gpmi-nfc-v1-bch-regs.h"

/**
 * struct resources - The collection of resources the driver needs.
 *
 * @gpmi_regs:         A pointer to the GPMI registers.
 * @bch_regs:          A pointer to the BCH registers.
 * @bch_interrupt:     The BCH interrupt number.
 * @dma_low_channel:   The low  DMA channel.
 * @dma_high_channel:  The high DMA channel.
 * @dma_interrupt:     The DMA interrupt number.
 * @clock:             A pointer to the struct clk for the NFC's clock.
 */

struct resources {
	void          *gpmi_regs;
	void          *bch_regs;
	unsigned int  bch_interrupt;
	unsigned int  dma_low_channel;
	unsigned int  dma_high_channel;
	unsigned int  dma_interrupt;
	struct clk    *clock;
};

/**
 * struct mil - State for the MTD Interface Layer.
 *
 * @nand:                    The NAND Flash MTD data structure that represents
 *                           the NAND Flash medium.
 * @mtd:                     The MTD data structure that represents the NAND
 *                           Flash medium.
 * @oob_layout:              A structure that describes how bytes are laid out
 *                           in the OOB.
 * @general_use_mtd:         A pointer to an MTD we export for general use.
 *                           This *may* simply be a pointer to the mtd field, if
 *                           we've been instructed NOT to protect the boot
 *                           areas.
 * @partitions:              A pointer to a set of partitions collected from
 *                           information provided by the platform data. These
 *                           partitions are applied to the general use MTD.
 * @partition_count:         The number of partitions.
 * @current_chip:            The chip currently selected by the NAND Fash MTD
 *                           code. A negative value indicates that no chip is
 *                           selected.
 * @command_length:          The length of the command that appears in the
 *                           command buffer (see cmd_virt, below).
 * @inject_ecc_error:        Indicates the driver should inject a "fake" ECC
 *                           error into the next read operation that uses ECC.
 *                           User space programs can set this value through the
 *                           sysfs node of the same name. If this value is less
 *                           than zero, the driver will inject an uncorrectable
 *                           ECC error. If this value is greater than zero, the
 *                           driver will inject that number of correctable
 *                           errors, capped by the maximum possible number of
 *                           errors that could appear in a single read.
 * @ignore_bad_block_marks:  Indicates we are ignoring bad block marks.
 * @saved_bbt:               A saved pointer to the in-memory NAND Flash MTD bad
 *                           block table. See show_device_ignorebad() for more
 *                           details.
 * @raw_oob_mode:            Indicates the OOB is to be read/written in "raw"
 *                           mode. See mil_ecc_read_oob() for details.
 * @hooked_read_oob:         A pointer to the ecc.read_oob() function we
 *                           "hooked." See mil_ecc_read_oob() for details.
 * @hooked_write_oob:        A pointer to the ecc.write_oob() function pointer
 *                           we "hooked." See mil_ecc_read_oob() for details.
 * @marking_a_bad_block:     Indicates the caller is marking a bad block. See
 *                           mil_ecc_write_oob() for details.
 * @hooked_block_markbad:    A pointer to the block_markbad() function we
 *                           we "hooked." See mil_ecc_write_oob() for details.
 * @cmd_virt:                A pointer to a DMA-coherent buffer in which we
 *                           accumulate command bytes before we give them to the
 *                           NFC layer. See mil_cmd_ctrl() for more details.
 * @cmd_phys:                The physical address for the cmd_virt buffer.
 * @page_buffer_virt:        A pointer to a DMA-coherent buffer we use for
 *                           reading and writing pages. This buffer includes
 *                           space for both the payload data and the auxiliary
 *                           data (including status bytes, but not syndrome
 *                           bytes).
 * @page_buffer_phys:        The physical address for the page_buffer_virt
 *                           buffer.
 * @page_buffer_size:        The size of the page buffer.
 * @payload_virt:            A pointer to a location in the page buffer used
 *                           for payload bytes. The size of this buffer is
 *                           determined by struct nfc_geometry.
 * @payload_phys:            The physical address for payload_virt.
 * @payload_size:            The size of the payload area in the page buffer.
 * @auxiliary_virt:          A pointer to a location in the page buffer used
 *                           for auxiliary bytes. The size of this buffer is
 *                           determined by struct nfc_geometry.
 * @auxiliary_phys:          The physical address for auxiliary_virt.
 * @auxiliary_size:          The size of the auxiliary area in the page buffer.
 */

#define MIL_COMMAND_BUFFER_SIZE (10)

struct mil {

	/* MTD Data Structures */

	struct nand_chip       nand;
	struct mtd_info        mtd;
	struct nand_ecclayout  oob_layout;

	/* Partitioning and Boot Area Protection */

	struct mtd_info        *general_use_mtd;
	struct mtd_partition   *partitions;
	unsigned int           partition_count;

	/* General-use Variables */

	int                    current_chip;
	unsigned int           command_length;
	int                    inject_ecc_error;
	int                    ignore_bad_block_marks;
	void                   *saved_bbt;

	/* MTD Function Pointer Hooks */

	int                    raw_oob_mode;
	int                    (*hooked_read_oob)(struct mtd_info *mtd,
					loff_t from, struct mtd_oob_ops *ops);
	int                    (*hooked_write_oob)(struct mtd_info *mtd,
					loff_t to, struct mtd_oob_ops *ops);

	int                    marking_a_bad_block;
	int                    (*hooked_block_markbad)(struct mtd_info *mtd,
					loff_t ofs);

	/* DMA Buffers */

	char                   *cmd_virt;
	dma_addr_t             cmd_phys;

	void                   *page_buffer_virt;
	dma_addr_t             page_buffer_phys;
	unsigned int           page_buffer_size;

	void                   *payload_virt;
	dma_addr_t             payload_phys;

	void                   *auxiliary_virt;
	dma_addr_t             auxiliary_phys;

};

/**
 * struct physical_geometry - Physical geometry description.
 *
 * This structure describes the physical geometry of the medium.
 *
 * @chip_count:                    The number of chips in the medium.
 * @chip_size_in_bytes:            The size, in bytes, of a single chip
 *                                 (excluding the out-of-band bytes).
 * @block_size_in_bytes:           The size, in bytes, of a single block
 *                                 (excluding the out-of-band bytes).
 * @page_data_size_in_bytes:       The size, in bytes, of the data area in a
 *                                 page (excluding the out-of-band bytes).
 * @page_oob_size_in_bytes:        The size, in bytes, of the out-of-band area
 *                                 in a page.
 */

struct physical_geometry {
	unsigned int  chip_count;
	uint64_t      chip_size_in_bytes;
	unsigned int  block_size_in_bytes;
	unsigned int  page_data_size_in_bytes;
	unsigned int  page_oob_size_in_bytes;
};

/**
 * struct nfc_geometry - NFC geometry description.
 *
 * This structure describes the NFC's view of the medium geometry.
 *
 * @ecc_algorithm:            The human-readable name of the ECC algorithm
 *                            (e.g., "Reed-Solomon" or "BCH").
 * @ecc_strength:             A number that describes the strength of the ECC
 *                            algorithm.
 * @page_size_in_bytes:       The size, in bytes, of a physical page, including
 *                            both data and OOB.
 * @metadata_size_in_bytes:   The size, in bytes, of the metadata.
 * @ecc_chunk_size_in_bytes:  The size, in bytes, of a single ECC chunk. Note
 *                            the first chunk in the page includes both data and
 *                            metadata, so it's a bit larger than this value.
 * @ecc_chunk_count:          The number of ECC chunks in the page,
 * @payload_size_in_bytes:    The size, in bytes, of the payload buffer.
 * @auxiliary_size_in_bytes:  The size, in bytes, of the auxiliary buffer.
 * @auxiliary_status_offset:  The offset into the auxiliary buffer at which
 *                            the ECC status appears.
 * @block_mark_byte_offset:   The byte offset in the ECC-based page view at
 *                            which the underlying physical block mark appears.
 * @block_mark_bit_offset:    The bit offset into the ECC-based page view at
 *                            which the underlying physical block mark appears.
 */

struct nfc_geometry {
	char          *ecc_algorithm;
	unsigned int  ecc_strength;
	unsigned int  page_size_in_bytes;
	unsigned int  metadata_size_in_bytes;
	unsigned int  ecc_chunk_size_in_bytes;
	unsigned int  ecc_chunk_count;
	unsigned int  payload_size_in_bytes;
	unsigned int  auxiliary_size_in_bytes;
	unsigned int  auxiliary_status_offset;
	unsigned int  block_mark_byte_offset;
	unsigned int  block_mark_bit_offset;
};

/**
 * struct boot_rom_geometry - Boot ROM geometry description.
 *
 * This structure encapsulates decisions made by the Boot ROM Helper.
 *
 * @boot_area_count:             The number of boot areas. The first boot area
 *                               appears at the beginning of chip 0, the next
 *                               at the beginning of chip 1, etc.
 * @boot_area_size_in_bytes:     The size, in bytes, of each boot area.
 * @stride_size_in_pages:        The size of a boot block stride, in pages.
 * @search_area_stride_exponent: The logarithm to base 2 of the size of a
 *                               search area in boot block strides.
 */

struct boot_rom_geometry {
	unsigned int  boot_area_count;
	unsigned int  boot_area_size_in_bytes;
	unsigned int  stride_size_in_pages;
	unsigned int  search_area_stride_exponent;
};

/**
 * struct gpmi_nfc_data - i.MX NFC per-device data.
 *
 * Note that the "device" managed by this driver represents the NAND Flash
 * controller *and* the NAND Flash medium behind it. Thus, the per-device data
 * structure has information about the controller, the chips to which it is
 * connected, and properties of the medium as a whole.
 *
 * @dev:                 A pointer to the owning struct device.
 * @pdev:                A pointer to the owning struct platform_device.
 * @pdata:               A pointer to the device's platform data.
 * @resources:           Information about system resources used by this driver.
 * @device_info:         A structure that contains detailed information about
 *                       the NAND Flash device.
 * @physical_geometry:   A description of the medium's physical geometry.
 * @nfc:                 A pointer to a structure that represents the underlying
 *                       NFC hardware.
 * @nfc_geometry:        A description of the medium geometry as viewed by the
 *                       NFC.
 * @rom:                 A pointer to a structure that represents the underlying
 *                       Boot ROM.
 * @rom_geometry:        A description of the medium geometry as viewed by the
 *                       Boot ROM.
 * @mil:                 A collection of information used by the MTD Interface
 *                       Layer.
 */

struct gpmi_nfc_data {

	/* System Interface */
	struct device                  *dev;
	struct platform_device         *pdev;
	struct gpmi_nfc_platform_data  *pdata;

	/* Resources */
	struct resources               resources;

	/* Flash Hardware */
	struct nand_device_info        device_info;
	struct physical_geometry       physical_geometry;

	/* NFC HAL */
	struct nfc_hal                 *nfc;
	struct nfc_geometry            nfc_geometry;

	/* Boot ROM Helper */
	struct boot_rom_helper         *rom;
	struct boot_rom_geometry       rom_geometry;

	/* MTD Interface Layer */
	struct mil                     mil;

};

/**
 * struct nfc_hal - GPMI NFC HAL
 *
 * This structure embodies an abstract interface to the underlying NFC hardware.
 *
 * @version:          The NFC hardware version.
 * @description:      A pointer to a human-readable description of the NFC
 *                    hardware.
 * @max_chip_count:   The maximum number of chips the NFC can possibly support
 *                    (this value is a constant for each NFC version). This may
 *                    *not* be the actual number of chips connected.
 * @dma_descriptors:  A pool of DMA descriptors.
 * @isr_dma_channel:  The DMA channel with which the NFC HAL is working. We
 *                    record this here so the ISR knows which DMA channel to
 *                    acknowledge.
 * @dma_done:         The completion structure used for DMA interrupts.
 * @bch_done:         The completion structure used for BCH interrupts.
 * @init:             Initializes the NFC hardware and data structures. This
 *                    function will be called after everything has been set up
 *                    for communication with the NFC itself, but before the
 *                    platform has set up off-chip communication. Thus, this
 *                    function must not attempt to communicate with the NAND
 *                    Flash hardware.
 * @set_geometry:     Configures the NFC hardware and data structures to match
 *                    the physical NAND Flash geometry.
 * @exit:             Shuts down the NFC hardware and data structures. This
 *                    function will be called after the platform has shut down
 *                    off-chip communication but while communication with the
 *                    NFC itself still works.
 * @is_ready:         Returns true if the given chip is ready.
 * @send_command:     Sends the given buffer of command bytes.
 * @send_data:        Sends the given buffer of data bytes.
 * @read_data:        Reads data bytes into the given buffer.
 * @send_page:        Sends the given given data and OOB bytes, using the ECC
 *                    engine.
 * @read_page:        Reads a page through the ECC engine and delivers the data
 *                    and OOB bytes to the given buffers.
 */

#define  NFC_DMA_DESCRIPTOR_COUNT  (4)

struct nfc_hal {
	const unsigned int   version;
	const char           *description;
	const unsigned int   max_chip_count;
	struct mxs_dma_desc  *dma_descriptors[NFC_DMA_DESCRIPTOR_COUNT];
	int                  isr_dma_channel;
	struct completion    dma_done;
	struct completion    bch_done;
	int   (*init)        (struct gpmi_nfc_data *);
	int   (*set_geometry)(struct gpmi_nfc_data *);
	void  (*exit)        (struct gpmi_nfc_data *);
	int   (*is_ready)    (struct gpmi_nfc_data *, unsigned chip);
	int   (*send_command)(struct gpmi_nfc_data *, unsigned chip,
				dma_addr_t buffer, unsigned length);
	int   (*send_data)   (struct gpmi_nfc_data *, unsigned chip,
				dma_addr_t buffer, unsigned length);
	int   (*read_data)   (struct gpmi_nfc_data *, unsigned chip,
				dma_addr_t buffer, unsigned length);
	int   (*send_page)   (struct gpmi_nfc_data *, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary);
	int   (*read_page)   (struct gpmi_nfc_data *, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary);
};

/**
 * struct boot_rom_helper - Boot ROM Helper
 *
 * This structure embodies the interface to an object that assists the driver
 * in making decisions that relate to the Boot ROM.
 *
 * @version:                    The Boot ROM version.
 * @description:                A pointer to a human-readable description of the
 *                              Boot ROM.
 * @swap_block_mark:            Indicates that the Boot ROM will swap the block
 *                              mark with the first byte of the OOB.
 * @set_geometry:               Configures the Boot ROM geometry.
 * @check_transcription_stamp:  Checks for a transcription stamp. This pointer
 *                              is ignored if swap_block_mark is set.
 * @write_transcription_stamp:  Writes a transcription stamp. This pointer
 *                              is ignored if swap_block_mark is set.
 */

struct boot_rom_helper {
	const unsigned int  version;
	const char          *description;
	const int           swap_block_mark;
	int  (*set_geometry)             (struct gpmi_nfc_data *);
	int  (*check_transcription_stamp)(struct gpmi_nfc_data *);
	int  (*write_transcription_stamp)(struct gpmi_nfc_data *);
};

#endif
