/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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
 */
#ifndef __DRIVERS_MTD_NAND_GPMI_NFC_H
#define __DRIVERS_MTD_NAND_GPMI_NFC_H

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>
#include <linux/dmaengine.h>
#include <asm/sizes.h>

#include <mach/common.h>
#include <mach/dma.h>
#include <mach/gpmi-nfc.h>
#include <mach/system.h>
#include <mach/clock.h>

/**
 * struct resources - The collection of resources the driver needs.
 *
 * @gpmi_regs:         A pointer to the GPMI registers.
 * @bch_regs:          A pointer to the BCH registers.
 * @bch_interrupt:     The BCH interrupt number.
 * @dma_low_channel:   The low  DMA channel.
 * @dma_high_channel:  The high DMA channel.
 * @clock:             A pointer to the struct clk for the NFC's clock.
 */
struct resources {
	void          *gpmi_regs;
	void          *bch_regs;
	unsigned int  bch_low_interrupt;
	unsigned int  bch_high_interrupt;
	unsigned int  dma_low_channel;
	unsigned int  dma_high_channel;
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
 * @partitions:              A pointer to a set of partitions.
 * @partition_count:         The number of partitions.
 * @current_chip:            The chip currently selected by the NAND Fash MTD
 *                           code. A negative value indicates that no chip is
 *                           selected.
 * @command_length:          The length of the command that appears in the
 *                           command buffer (see cmd_virt, below).
 * @ignore_bad_block_marks:  Indicates we are ignoring bad block marks.
 * @saved_bbt:               A saved pointer to the in-memory NAND Flash MTD bad
 *                           block table. See show_device_ignorebad() for more
 *                           details.
 * @marking_a_bad_block:     Indicates the caller is marking a bad block. See
 *                           mil_ecc_write_oob() for details.
 * @hooked_block_markbad:    A pointer to the block_markbad() function we
 *                           we "hooked." See mil_ecc_write_oob() for details.
 * @upper_buf:               The buffer passed from upper layer.
 * @upper_len:               The buffer len passed from upper layer.
 * @direct_dma_map_ok:       Is the direct DMA map is good for the upper_buf?
 * @cmd_sgl/cmd_buffer:      For NAND command.
 * @data_sgl/data_buffer_dma:For NAND DATA ops.
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
 * @auxiliary_virt:          A pointer to a location in the page buffer used
 *                           for auxiliary bytes. The size of this buffer is
 *                           determined by struct nfc_geometry.
 * @auxiliary_phys:          The physical address for auxiliary_virt.
 */
struct mil {
	/* MTD Data Structures */
	struct nand_chip       nand;
	struct mtd_info        mtd;
	struct nand_ecclayout  oob_layout;

	/* Partitions*/
	struct mtd_partition   *partitions;
	unsigned int           partition_count;

	/* General-use Variables */
	int                    current_chip;
	unsigned int           command_length;
	int                    ignore_bad_block_marks;
	void                   *saved_bbt;

	/* MTD Function Pointer Hooks */
	int                    marking_a_bad_block;
	int                    (*hooked_block_markbad)(struct mtd_info *mtd,
					loff_t ofs);

	/* from upper layer */
	uint8_t			*upper_buf;
	int			upper_len;

	/* DMA */
	bool			direct_dma_map_ok;

	struct scatterlist	cmd_sgl;
	char			*cmd_buffer;

	struct scatterlist	data_sgl;
	char			*data_buffer_dma;

	void                   *page_buffer_virt;
	dma_addr_t             page_buffer_phys;
	unsigned int           page_buffer_size;

	void                   *payload_virt;
	dma_addr_t             payload_phys;

	void                   *auxiliary_virt;
	dma_addr_t             auxiliary_phys;
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
 * @stride_size_in_pages:        The size of a boot block stride, in pages.
 * @search_area_stride_exponent: The logarithm to base 2 of the size of a
 *                               search area in boot block strides.
 */
struct boot_rom_geometry {
	unsigned int  stride_size_in_pages;
	unsigned int  search_area_stride_exponent;
};

/* DMA operations types */
enum dma_ops_type {
	DMA_FOR_COMMAND = 1,
	DMA_FOR_READ_DATA,
	DMA_FOR_WRITE_DATA,
	DMA_FOR_READ_ECC_PAGE,
	DMA_FOR_WRITE_ECC_PAGE
};

/**
 * This structure contains the fundamental timing attributes for NAND.
 *
 * @data_setup_in_ns:         The data setup time, in nanoseconds. Usually the
 *                            maximum of tDS and tWP. A negative value
 *                            indicates this characteristic isn't known.
 * @data_hold_in_ns:          The data hold time, in nanoseconds. Usually the
 *                            maximum of tDH, tWH and tREH. A negative value
 *                            indicates this characteristic isn't known.
 * @address_setup_in_ns:      The address setup time, in nanoseconds. Usually
 *                            the maximum of tCLS, tCS and tALS. A negative
 *                            value indicates this characteristic isn't known.
 * @gpmi_sample_delay_in_ns:  A GPMI-specific timing parameter. A negative value
 *                            indicates this characteristic isn't known.
 * @tREA_in_ns:               tREA, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 * @tRLOH_in_ns:              tRLOH, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 * @tRHOH_in_ns:              tRHOH, in nanoseconds, from the data sheet. A
 *                            negative value indicates this characteristic isn't
 *                            known.
 */
struct nand_timing {
	int8_t  data_setup_in_ns;
	int8_t  data_hold_in_ns;
	int8_t  address_setup_in_ns;
	int8_t  gpmi_sample_delay_in_ns;
	int8_t  tREA_in_ns;
	int8_t  tRLOH_in_ns;
	int8_t  tRHOH_in_ns;
};

/**
 * struct gpmi_nfc_data - i.MX NFC per-device data.
 *
 * @dev:                 A pointer to the owning struct device.
 * @pdev:                A pointer to the owning struct platform_device.
 * @pdata:               A pointer to the device's platform data.
 * @resources:           Information about system resources used by this driver.
 * @device_info:         A structure that contains detailed information about
 *                       the NAND Flash device.
 * @nfc:                 A pointer to a structure that represents the underlying
 *                       NFC hardware.
 * @nfc_geometry:        A description of the medium geometry as viewed by the
 *                       NFC.
 * @swap_block_mark:     Does it support the swap-block-mark feature?
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
	struct nand_timing		timing;

	/* NFC HAL */
	struct nfc_hal                 *nfc;
	struct nfc_geometry            nfc_geometry;

	/* NAND Boot issue */
	bool				swap_block_mark;
	struct boot_rom_geometry       rom_geometry;

	/* MTD Interface Layer */
	struct mil                     mil;

	/* DMA channels */
#define DMA_CHANS			8
	struct dma_chan			*dma_chans[DMA_CHANS];
	struct mxs_dma_data		dma_data;
	enum dma_ops_type		dma_type;

	/* private */
	void				*private;
};

/**
 * struct gpmi_nfc_hardware_timing - GPMI NFC hardware timing parameters.
 *
 * This structure contains timing information expressed in a form directly
 * usable by the GPMI NFC hardware.
 *
 * @data_setup_in_cycles:      The data setup time, in cycles.
 * @data_hold_in_cycles:       The data hold time, in cycles.
 * @address_setup_in_cycles:   The address setup time, in cycles.
 * @use_half_periods:          Indicates the clock is running slowly, so the
 *                             NFC DLL should use half-periods.
 * @sample_delay_factor:       The sample delay factor.
 */
struct gpmi_nfc_hardware_timing {
	uint8_t  data_setup_in_cycles;
	uint8_t  data_hold_in_cycles;
	uint8_t  address_setup_in_cycles;
	bool     use_half_periods;
	uint8_t  sample_delay_factor;
};

/**
 * struct nfc_hal - GPMI NFC HAL
 *
 * @description:                 description.
 * @max_chip_count:              The maximum number of chips the NFC can
 *                               possibly support (this value is a constant for
 *                               each NFC version). This may *not* be the actual
 *                               number of chips connected.
 * @max_data_setup_cycles:       The maximum number of data setup cycles that
 *                               can be expressed in the hardware.
 * @internal_data_setup_in_ns:   The time, in ns, that the NFC hardware requires
 *                               for data read internal setup. In the Reference
 *                               Manual, see the chapter "High-Speed NAND
 *                               Timing" for more details.
 * @max_sample_delay_factor:     The maximum sample delay factor that can be
 *                               expressed in the hardware.
 * @max_dll_clock_period_in_ns:  The maximum period of the GPMI clock that the
 *                               sample delay DLL hardware can possibly work
 *                               with (the DLL is unusable with longer periods).
 *                               If the full-cycle period is greater than HALF
 *                               this value, the DLL must be configured to use
 *                               half-periods.
 * @max_dll_delay_in_ns:         The maximum amount of delay, in ns, that the
 *                               DLL can implement.
 * @dma_descriptors:             A pool of DMA descriptors.
 * @isr_dma_channel:             The DMA channel with which the NFC HAL is
 *                               working. We record this here so the ISR knows
 *                               which DMA channel to acknowledge.
 * @dma_done:                    The completion structure used for DMA
 *                               interrupts.
 * @bch_done:                    The completion structure used for BCH
 *                               interrupts.
 * @timing:                      The current timing configuration.
 * @clock_frequency_in_hz:       The clock frequency, in Hz, during the current
 *                               I/O transaction. If no I/O transaction is in
 *                               progress, this is the clock frequency during
 *                               the most recent I/O transaction.
 * @hardware_timing:             The hardware timing configuration in effect
 *                               during the current I/O transaction. If no I/O
 *                               transaction is in progress, this is the
 *                               hardware timing configuration during the most
 *                               recent I/O transaction.
 * @init:                        Initializes the NFC hardware and data
 *                               structures. This function will be called after
 *                               everything has been set up for communication
 *                               with the NFC itself, but before the platform
 *                               has set up off-chip communication. Thus, this
 *                               function must not attempt to communicate with
 *                               the NAND Flash hardware.
 * @set_geometry:                Configures the NFC hardware and data structures
 *                               to match the physical NAND Flash geometry.
 * @set_timing:                  Configures the NFC hardware and data structures
 *                               to match the given NAND Flash bus timing.
 * @get_timing:                  Returns the the clock frequency, in Hz, and
 *                               the hardware timing configuration during the
 *                               current I/O transaction. If no I/O transaction
 *                               is in progress, this is the timing state during
 *                               the most recent I/O transaction.
 * @exit:                        Shuts down the NFC hardware and data
 *                               structures. This function will be called after
 *                               the platform has shut down off-chip
 *                               communication but while communication with the
 *                               NFC itself still works.
 * @clear_bch:                   Clears a BCH interrupt (intended to be called
 *                               by a more general interrupt handler to do
 *                               device-specific clearing).
 * @is_ready:                    Returns true if the given chip is ready.
 * @begin:                       Begins an interaction with the NFC. This
 *                               function must be called before *any* of the
 *                               following functions so the NFC can prepare
 *                               itself.
 * @end:                         Ends interaction with the NFC. This function
 *                               should be called to give the NFC a chance to,
 *                               among other things, enter a lower-power state.
 * @send_command:                Sends the given buffer of command bytes.
 * @send_data:                   Sends the given buffer of data bytes.
 * @read_data:                   Reads data bytes into the given buffer.
 * @send_page:                   Sends the given given data and OOB bytes,
 *                               using the ECC engine.
 * @read_page:                   Reads a page through the ECC engine and
 *                               delivers the data and OOB bytes to the given
 *                               buffers.
 */
struct nfc_hal {
	/* Hardware attributes. */
	const char              *description;
	const unsigned int      max_chip_count;
	const unsigned int      max_data_setup_cycles;
	const unsigned int      internal_data_setup_in_ns;
	const unsigned int      max_sample_delay_factor;
	const unsigned int      max_dll_clock_period_in_ns;
	const unsigned int      max_dll_delay_in_ns;

	int                     isr_dma_channel;
	struct completion       dma_done;
	struct completion       bch_done;
	struct nand_timing      timing;
	unsigned long           clock_frequency_in_hz;

	/* Configuration functions. */
	int   (*init)        (struct gpmi_nfc_data *);
	int   (*extra_init)  (struct gpmi_nfc_data *);
	int   (*set_geometry)(struct gpmi_nfc_data *);
	int   (*set_timing)  (struct gpmi_nfc_data *,
					const struct nand_timing *);
	void  (*get_timing)  (struct gpmi_nfc_data *,
					unsigned long *clock_frequency_in_hz,
					struct gpmi_nfc_hardware_timing *);
	void  (*exit)        (struct gpmi_nfc_data *);

	/* Call these functions to begin and end I/O. */
	void  (*begin)       (struct gpmi_nfc_data *);
	void  (*end)         (struct gpmi_nfc_data *);

	/* Call these I/O functions only between begin() and end(). */
	void  (*clear_bch)   (struct gpmi_nfc_data *);
	int   (*is_ready)    (struct gpmi_nfc_data *, unsigned chip);
	int   (*send_command)(struct gpmi_nfc_data *);
	int   (*send_data)   (struct gpmi_nfc_data *);
	int   (*read_data)   (struct gpmi_nfc_data *);
	int   (*send_page)   (struct gpmi_nfc_data *,
				dma_addr_t payload, dma_addr_t auxiliary);
	int   (*read_page)   (struct gpmi_nfc_data *,
				dma_addr_t payload, dma_addr_t auxiliary);
};

/* NFC HAL Common Services */
extern int common_nfc_set_geometry(struct gpmi_nfc_data *this);
extern int gpmi_nfc_compute_hardware_timing(struct gpmi_nfc_data *this,
					struct gpmi_nfc_hardware_timing *hw);
extern struct dma_chan *get_dma_chan(struct gpmi_nfc_data *this);
extern void prepare_data_dma(struct gpmi_nfc_data *this,
				enum dma_data_direction dr);
extern int start_dma_without_bch_irq(struct gpmi_nfc_data *this,
					struct dma_async_tx_descriptor *desc);
extern int start_dma_with_bch_irq(struct gpmi_nfc_data *this,
					struct dma_async_tx_descriptor *desc);
/* NFC HAL Structures */
extern struct nfc_hal  gpmi_nfc_hal_imx23_imx28;
extern struct nfc_hal  gpmi_nfc_hal_mx50;

/* ONFI or TOGGLE nand */
bool is_ddr_nand(struct gpmi_nfc_data *);

/* for log */
extern int gpmi_debug;
#define GPMI_DEBUG_INIT		0x0001
#define GPMI_DEBUG_READ		0x0002
#define GPMI_DEBUG_WRITE	0x0004
#define GPMI_DEBUG_ECC_READ	0x0008
#define GPMI_DEBUG_ECC_WRITE	0x0010

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "[ %s : %.3d ] " fmt, __func__, __LINE__

#define logio(level)				\
		do {				\
			if (gpmi_debug & level)	\
				pr_info("\n");	\
		} while (0)

/* BCH : Status Block Completion Codes */
#define STATUS_GOOD		0x00
#define STATUS_ERASED		0xff
#define STATUS_UNCORRECTABLE	0xfe

/* Use the platform_id to distinguish different Archs. */
#define IS_MX23			0x1
#define IS_MX28			0x2
#define IS_MX50			0x4
#define IS_MX6Q			0x8
#define GPMI_IS_MX23(x)		((x)->pdev->id_entry->driver_data == IS_MX23)
#define GPMI_IS_MX28(x)		((x)->pdev->id_entry->driver_data == IS_MX28)
#define GPMI_IS_MX50(x)		((x)->pdev->id_entry->driver_data == IS_MX50)
#define GPMI_IS_MX6Q(x)		((x)->pdev->id_entry->driver_data == IS_MX6Q)
#endif
