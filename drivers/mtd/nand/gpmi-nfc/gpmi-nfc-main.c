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

#include "gpmi-nfc.h"

/*
 * This structure contains the "safe" GPMI timing that should succeed with any
 * NAND Flash device (although, with less-than-optimal performance).
 */

static struct gpmi_nfc_timing  safe_timing = {
	.data_setup_in_ns        = 80,
	.data_hold_in_ns         = 60,
	.address_setup_in_ns     = 25,
	.gpmi_sample_delay_in_ns =  6,
	.tREA_in_ns              = -1,
	.tRLOH_in_ns             = -1,
	.tRHOH_in_ns             = -1,
};

/*
 * This array has a pointer to every NFC HAL structure. The probing process will
 * find and install the one that matches the version given by the platform.
 */

static struct nfc_hal  *(nfc_hals[]) = {
	&gpmi_nfc_hal_v0,
	&gpmi_nfc_hal_v1,
};

/*
 * This array has a pointer to every Boot ROM Helper structure. The probing
 * process will find and install the one that matches the version given by the
 * platform.
 */

static struct boot_rom_helper  *(boot_rom_helpers[]) = {
	&gpmi_nfc_boot_rom_helper_v0,
	&gpmi_nfc_boot_rom_helper_v1,
};

/**
 * show_device_numchips() - Shows the number of physical chips.
 *
 * This node is made obsolete by the physical_geometry node, but we keep it for
 * backward compatibility (especially for kobs).
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_numchips(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this     = dev_get_drvdata(dev);
	struct physical_geometry  *physical = &this->physical_geometry;

	return sprintf(buf, "%d\n", physical->chip_count);

}

/**
 * show_device_physical_geometry() - Shows the physical Flash device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_physical_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this     = dev_get_drvdata(dev);
	struct physical_geometry  *physical = &this->physical_geometry;

	return sprintf(buf,
		"Chip Count             : %u\n"
		"Chip Size in Bytes     : %llu\n"
		"Block Size in Bytes    : %u\n"
		"Page Data Size in Bytes: %u\n"
		"Page OOB Size in Bytes : %u\n"
		,
		physical->chip_count,
		physical->chip_size_in_bytes,
		physical->block_size_in_bytes,
		physical->page_data_size_in_bytes,
		physical->page_oob_size_in_bytes
	);

}

/**
 * show_device_nfc_info() - Shows the NFC-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this      = dev_get_drvdata(dev);
	struct resources      *resources = &this->resources;
	struct nfc_hal        *nfc       =  this->nfc;
	struct clk            *parent_clock;
	unsigned long         parent_clock_rate_in_hz;
	unsigned long         clock_rate_in_hz;
	int                   clock_use_count;

	parent_clock            = clk_get_parent(resources->clock);
	parent_clock_rate_in_hz = clk_get_rate(parent_clock);
	clock_rate_in_hz        = clk_get_rate(resources->clock);
	clock_use_count         = clk_get_usecount(resources->clock);

	return sprintf(buf,
		"Version                : %u\n"
		"Description            : %s\n"
		"Max Chip Count         : %u\n"
		"Parent Clock Rate in Hz: %lu\n"
		"Clock Rate in Hz       : %lu\n"
		"Clock Use Count        : %u\n"
		"Data Setup in ns       : %d\n"
		"Data Hold in ns        : %d\n"
		"Address Setup in ns    : %d\n"
		"Sample Delay in ns     : %d\n"
		"tREA in ns             : %d\n"
		"tRLOH in ns            : %d\n"
		"tRHOH in ns            : %d\n"
		,
		nfc->version,
		nfc->description,
		nfc->max_chip_count,
		parent_clock_rate_in_hz,
		clock_rate_in_hz,
		clock_use_count,
		nfc->timing.data_setup_in_ns,
		nfc->timing.data_hold_in_ns,
		nfc->timing.address_setup_in_ns,
		nfc->timing.gpmi_sample_delay_in_ns,
		nfc->timing.tREA_in_ns,
		nfc->timing.tRLOH_in_ns,
		nfc->timing.tRHOH_in_ns
	);

}

/**
 * show_device_nfc_geometry() - Shows the NFC view of the device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct nfc_geometry   *nfc  = &this->nfc_geometry;

	return sprintf(buf,
		"ECC Algorithm          : %s\n"
		"ECC Strength           : %u\n"
		"Page Size in Bytes     : %u\n"
		"Metadata Size in Bytes : %u\n"
		"ECC Chunk Size in Bytes: %u\n"
		"ECC Chunk Count        : %u\n"
		"Payload Size in Bytes  : %u\n"
		"Auxiliary Size in Bytes: %u\n"
		"Auxiliary Status Offset: %u\n"
		"Block Mark Byte Offset : %u\n"
		"Block Mark Bit Offset  : %u\n"
		,
		nfc->ecc_algorithm,
		nfc->ecc_strength,
		nfc->page_size_in_bytes,
		nfc->metadata_size_in_bytes,
		nfc->ecc_chunk_size_in_bytes,
		nfc->ecc_chunk_count,
		nfc->payload_size_in_bytes,
		nfc->auxiliary_size_in_bytes,
		nfc->auxiliary_status_offset,
		nfc->block_mark_byte_offset,
		nfc->block_mark_bit_offset
	);

}

/**
 * show_device_rom_geometry() - Shows the Boot ROM Helper's geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_rom_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this = dev_get_drvdata(dev);
	struct boot_rom_geometry  *rom  = &this->rom_geometry;

	return sprintf(buf,
		"Boot Area Count           : %u\n"
		"Boot Area Size in Bytes   : %u\n"
		"Stride Size in Pages      : %u\n"
		"Seach Area Stride Exponent: %u\n"
		,
		rom->boot_area_count,
		rom->boot_area_size_in_bytes,
		rom->stride_size_in_pages,
		rom->search_area_stride_exponent
	);

}

/**
 * show_device_mtd_nand_info() - Shows the device's MTD NAND-specific info.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_nand_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct gpmi_nfc_data       *this = dev_get_drvdata(dev);
	struct mil                 *mil  = &this->mil;
	struct nand_chip           *nand = &mil->nand;

	o += sprintf(buf + o,
		"Options                       : 0x%08x\n"
		"Chip Count                    : %u\n"
		"Chip Size in Bytes            : %llu\n"
		"Minimum Writable Size in Bytes: %u\n"
		"Page Shift                    : %u\n"
		"Page Mask                     : 0x%x\n"
		"Block Shift                   : %u\n"
		"BBT Block Shift               : %u\n"
		"Chip Shift                    : %u\n"
		"Block Mark Offset             : %u\n"
		"Cached Page Number            : %d\n"
		,
		nand->options,
		nand->numchips,
		nand->chipsize,
		nand->subpagesize,
		nand->page_shift,
		nand->pagemask,
		nand->phys_erase_shift,
		nand->bbt_erase_shift,
		nand->chip_shift,
		nand->badblockpos,
		nand->pagebuf
	);

	o += sprintf(buf + o,
		"ECC Byte Count       : %u\n"
		,
		nand->ecc.layout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < nand->ecc.layout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= nand->ecc.layout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				nand->ecc.layout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes  : %u\n"
		,
		nand->ecc.layout->oobavail
	);

	j = 0;

	for (i = 0; j < nand->ecc.layout->oobavail; i++) {

		j += nand->ecc.layout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			nand->ecc.layout->oobfree[i].offset,
			nand->ecc.layout->oobfree[i].length
		);

	}

	return o;

}

/**
 * show_device_mtd_info() - Shows the device's MTD-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct gpmi_nfc_data       *this = dev_get_drvdata(dev);
	struct mil                 *mil  = &this->mil;
	struct mtd_info            *mtd  = &mil->mtd;

	o += sprintf(buf + o,
		"Name               : %s\n"
		"Type               : %u\n"
		"Flags              : 0x%08x\n"
		"Size in Bytes      : %llu\n"
		"Erase Region Count : %d\n"
		"Erase Size in Bytes: %u\n"
		"Write Size in Bytes: %u\n"
		"OOB Size in Bytes  : %u\n"
		"Errors Corrected   : %u\n"
		"Failed Reads       : %u\n"
		"Bad Block Count    : %u\n"
		"BBT Block Count    : %u\n"
		,
		mtd->name,
		mtd->type,
		mtd->flags,
		mtd->size,
		mtd->numeraseregions,
		mtd->erasesize,
		mtd->writesize,
		mtd->oobsize,
		mtd->ecc_stats.corrected,
		mtd->ecc_stats.failed,
		mtd->ecc_stats.badblocks,
		mtd->ecc_stats.bbtblocks
	);

	o += sprintf(buf + o,
		"ECC Byte Count     : %u\n"
		,
		mtd->ecclayout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < mtd->ecclayout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= mtd->ecclayout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				mtd->ecclayout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes: %u\n"
		,
		mtd->ecclayout->oobavail
	);

	j = 0;

	for (i = 0; j < mtd->ecclayout->oobavail; i++) {

		j += mtd->ecclayout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			mtd->ecclayout->oobfree[i].offset,
			mtd->ecclayout->oobfree[i].length
		);

	}

	return o;

}

/**
 * store_device_mark_block_bad() - Marks a block as bad.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_mark_block_bad(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	struct mtd_info       *mtd  = &mil->mtd;
	struct nand_chip      *nand = &mil->nand;
	unsigned long         block_number;
	loff_t                byte_address;
	int                   error;

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the block number. */

	if (strict_strtoul(buf, 0, &block_number))
		return -EINVAL;

	/* Compute the byte address of this block. */

	byte_address = block_number << nand->phys_erase_shift;

	/* Attempt to mark the block bad. */

	error = mtd->block_markbad(mtd, byte_address);

	if (error)
		return error;

	/* Return success. */

	return size;

}

/**
 * show_device_ignorebad() - Shows the value of the 'ignorebad' flag.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_ignorebad(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;

	return sprintf(buf, "%d\n", mil->ignore_bad_block_marks);
}

/**
 * store_device_ignorebad() - Sets the value of the 'ignorebad' flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_ignorebad(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	const char            *p = buf;
	unsigned long         v;

	/* Try to make sense of what arrived from user space. */

	if (strict_strtoul(p, 0, &v) < 0)
		return size;

	if (v > 0)
		v = 1;

	/* Only do something if the value is changing. */

	if (v != mil->ignore_bad_block_marks) {

		if (v) {

			/*
			 * If control arrives here, we want to begin ignoring
			 * bad block marks. Reach into the NAND Flash MTD data
			 * structures and set the in-memory BBT pointer to NULL.
			 * This will cause the NAND Flash MTD code to believe
			 * that it never created a BBT and force it to call our
			 * block_bad function.
			 *
			 * See mil_block_bad for more details.
			 */

			mil->saved_bbt = mil->nand.bbt;
			mil->nand.bbt  = 0;

		} else {

			/*
			 * If control arrives here, we want to stop ignoring
			 * bad block marks. Restore the NAND Flash MTD's pointer
			 * to its in-memory BBT.
			 */

			mil->nand.bbt = mil->saved_bbt;

		}

		mil->ignore_bad_block_marks = v;

	}

	return size;

}

/**
 * show_device_inject_ecc_error() - Shows the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_inject_ecc_error(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;

	return sprintf(buf, "%d\n", mil->inject_ecc_error);

}

/**
 * store_device_inject_ecc_error() - Sets the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_inject_ecc_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	long                  new_inject_ecc_error;

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the ECC error count. */

	if (strict_strtol(buf, 0, &new_inject_ecc_error))
		return -EINVAL;

	/* Store the value. */

	mil->inject_ecc_error = new_inject_ecc_error;

	/* Return success. */

	return size;

}

/**
 * store_device_invalidate_page_cache() - Invalidates the device's page cache.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_invalidate_page_cache(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);

	/* Invalidate the page cache. */

	this->mil.nand.pagebuf = -1;

	/* Return success. */

	return size;

}

/* Device attributes that appear in sysfs. */

static DEVICE_ATTR(numchips         , 0444, show_device_numchips         , 0);
static DEVICE_ATTR(physical_geometry, 0444, show_device_physical_geometry, 0);
static DEVICE_ATTR(nfc_info         , 0444, show_device_nfc_info         , 0);
static DEVICE_ATTR(nfc_geometry     , 0444, show_device_nfc_geometry     , 0);
static DEVICE_ATTR(rom_geometry     , 0444, show_device_rom_geometry     , 0);
static DEVICE_ATTR(mtd_nand_info    , 0444, show_device_mtd_nand_info    , 0);
static DEVICE_ATTR(mtd_info         , 0444, show_device_mtd_info         , 0);

static DEVICE_ATTR(mark_block_bad, 0200,
	0, store_device_mark_block_bad);

static DEVICE_ATTR(ignorebad, 0644,
	show_device_ignorebad, store_device_ignorebad);

static DEVICE_ATTR(inject_ecc_error, 0644,
	show_device_inject_ecc_error, store_device_inject_ecc_error);

static DEVICE_ATTR(invalidate_page_cache, 0644,
	0, store_device_invalidate_page_cache);

static struct device_attribute *device_attributes[] = {
	&dev_attr_numchips,
	&dev_attr_physical_geometry,
	&dev_attr_nfc_info,
	&dev_attr_nfc_geometry,
	&dev_attr_rom_geometry,
	&dev_attr_mtd_nand_info,
	&dev_attr_mtd_info,
	&dev_attr_mark_block_bad,
	&dev_attr_ignorebad,
	&dev_attr_inject_ecc_error,
	&dev_attr_invalidate_page_cache,
};

/**
 * validate_the_platform() - Validates information about the platform.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int validate_the_platform(struct platform_device *pdev)
{
	struct device                  *dev   = &pdev->dev;
	struct gpmi_nfc_platform_data  *pdata = pdev->dev.platform_data;

	/* Validate the clock name. */

	if (!pdata->clock_name) {
		dev_err(dev, "No clock name\n");
		return -ENXIO;
	}

	/* Validate the partitions. */

	if ((pdata->partitions && (!pdata->partition_count)) ||
			(!pdata->partitions && (pdata->partition_count))) {
		dev_err(dev, "Bad partition data\n");
		return -ENXIO;
	}

	/* Return success */

	return 0;

}

/**
 * acquire_register_block() - Tries to acquire and map a register block.
 *
 * @this:            Per-device data.
 * @resource_name:   The name of the resource.
 * @reg_block_base:  A pointer to a variable that will receive the address of
 *                   the mapped register block.
 */
static int acquire_register_block(struct gpmi_nfc_data *this,
			const char *resource_name, void **reg_block_base)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	void                    *p;
	struct resource         *r;

	/* Attempt to get information about the given resource. */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, resource_name);

	if (!r) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Attempt to remap the register block. */

	p = ioremap(r->start, r->end - r->start + 1);

	if (!p) {
		dev_err(dev, "Can't remap %s\n", resource_name);
		return -EIO;
	}

	/* If control arrives here, everything went fine. */

	*reg_block_base = p;

	return 0;

}

/**
 * release_register_block() - Releases a register block.
 *
 * @this:            Per-device data.
 * @reg_block_base:  A pointer to the mapped register block.
 */
static void release_register_block(struct gpmi_nfc_data *this,
							void *reg_block_base)
{
	iounmap(reg_block_base);
}

/**
 * acquire_interrupt() - Tries to acquire an interrupt.
 *
 * @this:              Per-device data.
 * @resource_name:     The name of the resource.
 * @interrupt_handler: A pointer to the function that will handle interrupts
 *                     from this interrupt number.
 * @interrupt_number:  A pointer to a variable that will receive the acquired
 *                     interrupt number.
 */
static int acquire_interrupt(
			struct gpmi_nfc_data *this, const char *resource_name,
			irq_handler_t interrupt_handler, int *interrupt_number)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	int                     error = 0;
	int                     i;

	/* Attempt to get information about the given resource. */

	i = platform_get_irq_byname(pdev, resource_name);

	if (i < 0) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Attempt to own the interrupt. */

	error = request_irq(i, interrupt_handler, 0, resource_name, this);

	if (error) {
		dev_err(dev, "Can't own %s\n", resource_name);
		return -EIO;
	}

	/* If control arrives here, everything went fine. */

	*interrupt_number = i;

	return 0;

}

/**
 * release_interrupt() - Releases an interrupt.
 *
 * @this:              Per-device data.
 * @interrupt_number:  The interrupt number.
 */
static void release_interrupt(struct gpmi_nfc_data *this, int interrupt_number)
{
	free_irq(interrupt_number, this);
}

/**
 * acquire_dma_channels() - Tries to acquire DMA channels.
 *
 * @this:           Per-device data.
 * @resource_name:  The name of the resource.
 * @low_channel:    A pointer to a variable that will receive the acquired
 *                  low DMA channel number.
 * @high_channel:   A pointer to a variable that will receive the acquired
 *                  high DMA channel number.
 */
static int acquire_dma_channels(
			struct gpmi_nfc_data *this, const char *resource_name,
			unsigned *low_channel, unsigned *high_channel)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	int                     error = 0;
	struct resource         *r;
	unsigned int            dma_channel;

	/* Attempt to get information about the given resource. */

	r = platform_get_resource_byname(pdev, IORESOURCE_DMA, resource_name);

	if (!r) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Loop over DMA channels, attempting to own them. */

	for (dma_channel = r->start; dma_channel <= r->end; dma_channel++) {

		/* Attempt to own the current channel. */

		error = mxs_dma_request(dma_channel, dev, resource_name);

		/* Check if we successfully acquired the current channel. */

		if (error) {

			dev_err(dev, "Can't acquire DMA channel %u\n",
								dma_channel);

			/* Free all the channels we've already acquired. */

			while (--dma_channel >= 0)
				mxs_dma_release(dma_channel, dev);

			return error;

		}

		/*
		 * If control arrives here, we successfully acquired the
		 * current channel. Continue initializing it.
		 */

		mxs_dma_reset(dma_channel);
		mxs_dma_ack_irq(dma_channel);

	}

	/* If control arrives here, all went well. */

	*low_channel  = r->start;
	*high_channel = r->end;

	return 0;

}

/**
 * release_dma_channels() - Releases DMA channels.
 *
 * @this:          Per-device data.
 * @low_channel:   The low DMA channel number.
 * @high_channel:  The high DMA channel number.
 */
static void release_dma_channels(struct gpmi_nfc_data *this,
				unsigned low_channel, unsigned high_channel)
{
	struct device  *dev = this->dev;
	unsigned int   i;

	for (i = low_channel; i <= high_channel; i++)
		mxs_dma_release(i, dev);
}

/**
 * acquire_clock() - Tries to acquire a clock.
 *
 * @this:           Per-device data.
 * @resource_name:  The name of the clock.
 * @high_channel:   A pointer to a variable that will receive the acquired
 *                  clock address.
 */
static int acquire_clock(struct gpmi_nfc_data *this,
				const char *clock_name, struct clk **clock)
{
	struct device  *dev  = this->dev;
	int            error = 0;
	struct clk     *c;

	/* Try to get the clock. */

	c = clk_get(dev, clock_name);

	if (IS_ERR(c)) {
		error = PTR_ERR(c);
		dev_err(dev, "Can't own clock %s\n", clock_name);
		return error;
	}

	/* If control arrives here, everything went fine. */

	*clock = c;

	return 0;

}

/**
 * release_clock() - Releases a clock.
 *
 * @this:   Per-device data.
 * @clock:  A pointer to the clock structure.
 */
static void release_clock(struct gpmi_nfc_data *this, struct clk *clock)
{
	clk_disable(clock);
	clk_put(clock);
}

/**
 * acquire_resources() - Tries to acquire resources.
 *
 * @this:  Per-device data.
 */
static int acquire_resources(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata     =  this->pdata;
	struct resources               *resources = &this->resources;
	int                            error      = 0;

	/* Attempt to acquire the GPMI register block. */

	error = acquire_register_block(this,
		GPMI_NFC_GPMI_REGS_ADDR_RES_NAME, &(resources->gpmi_regs));

	if (error)
		goto exit_gpmi_regs;

	/* Attempt to acquire the BCH register block. */

	error = acquire_register_block(this,
		GPMI_NFC_BCH_REGS_ADDR_RES_NAME, &(resources->bch_regs));

	if (error)
		goto exit_bch_regs;

	/* Attempt to acquire the BCH interrupt. */

	error = acquire_interrupt(this,
		GPMI_NFC_BCH_INTERRUPT_RES_NAME,
		gpmi_nfc_bch_isr, &(resources->bch_interrupt));

	if (error)
		goto exit_bch_interrupt;

	/* Attempt to acquire the DMA channels. */

	error = acquire_dma_channels(this,
		GPMI_NFC_DMA_CHANNELS_RES_NAME,
		&(resources->dma_low_channel), &(resources->dma_high_channel));

	if (error)
		goto exit_dma_channels;

	/* Attempt to acquire the DMA interrupt. */

	error = acquire_interrupt(this,
		GPMI_NFC_DMA_INTERRUPT_RES_NAME,
		gpmi_nfc_dma_isr, &(resources->dma_interrupt));

	if (error)
		goto exit_dma_interrupt;

	/* Attempt to acquire our clock. */

	error = acquire_clock(this, pdata->clock_name, &(resources->clock));

	if (error)
		goto exit_clock;

	/* If control arrives here, all went well. */

	return 0;

	/* Control arrives here if something went wrong. */

exit_clock:
	release_interrupt(this, resources->dma_interrupt);
exit_dma_interrupt:
	release_dma_channels(this,
		resources->dma_low_channel, resources->dma_high_channel);
exit_dma_channels:
	release_interrupt(this, resources->bch_interrupt);
exit_bch_interrupt:
	release_register_block(this, resources->bch_regs);
exit_bch_regs:
	release_register_block(this, resources->gpmi_regs);
exit_gpmi_regs:

	return error;

}

/**
 * release_resources() - Releases resources.
 *
 * @this:  Per-device data.
 */
static void release_resources(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	release_clock(this, resources->clock);
	release_register_block(this, resources->gpmi_regs);
	release_register_block(this, resources->bch_regs);
	release_interrupt(this, resources->bch_interrupt);
	release_dma_channels(this,
		resources->dma_low_channel, resources->dma_high_channel);
	release_interrupt(this, resources->dma_interrupt);
}

/**
 * set_up_nfc_hal() - Sets up the NFC HAL.
 *
 * @this:  Per-device data.
 */
static int set_up_nfc_hal(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata = this->pdata;
	struct device                  *dev   = this->dev;
	struct nfc_hal                 *nfc;
	int                            error = 0;
	unsigned int                   i;

	/* Attempt to find an NFC HAL that matches the given version. */

	for (i = 0; i < ARRAY_SIZE(nfc_hals); i++) {

		nfc = nfc_hals[i];

		if (nfc->version == pdata->nfc_version) {
			this->nfc = nfc;
			break;
		}

	}

	/* Check if we found a HAL. */

	if (i >= ARRAY_SIZE(nfc_hals)) {
		dev_err(dev, "Unkown NFC version %u\n", pdata->nfc_version);
		return -ENXIO;
	}

	pr_info("NFC: Version %u, %s\n", nfc->version, nfc->description);

	/*
	 * Check if we can handle the number of chips called for by the platform
	 * data.
	 */

	if (pdata->max_chip_count > nfc->max_chip_count) {
		dev_err(dev, "Platform data calls for %u chips "
				"but NFC supports a max of %u.\n",
				pdata->max_chip_count, nfc->max_chip_count);
		return -ENXIO;
	}

	/* Initialize the NFC HAL. */

	error = nfc->init(this);

	if (error)
		return error;

	/* Set up safe timing. */

	nfc->set_timing(this, &safe_timing);

	/*
	 * If control arrives here, all is well.
	 */

	return 0;

}

/**
 * set_up_boot_rom_helper() - Sets up the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int set_up_boot_rom_helper(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata = this->pdata;
	struct device                  *dev   = this->dev;
	unsigned int                   i;
	struct boot_rom_helper         *rom;

	/* Attempt to find a Boot ROM Helper that matches the given version. */

	for (i = 0; i < ARRAY_SIZE(boot_rom_helpers); i++) {

		rom = boot_rom_helpers[i];

		if (rom->version == pdata->boot_rom_version) {
			this->rom = rom;
			break;
		}

	}

	/* Check if we found a Boot ROM Helper. */

	if (i >= ARRAY_SIZE(boot_rom_helpers)) {
		dev_err(dev, "Unkown Boot ROM version %u\n",
						pdata->boot_rom_version);
		return -ENXIO;
	}

	pr_info("Boot ROM: Version %u, %s\n", rom->version, rom->description);

	/*
	 * If control arrives here, all is well.
	 */

	return 0;

}

/**
 * manage_sysfs_files() - Creates/removes sysfs files for this device.
 *
 * @this:  Per-device data.
 */
static void manage_sysfs_files(struct gpmi_nfc_data *this, int create)
{
	struct device            *dev = this->dev;
	int                      error;
	unsigned int             i;
	struct device_attribute  **attr;

	for (i = 0, attr = device_attributes;
			i < ARRAY_SIZE(device_attributes); i++, attr++) {

		if (create) {
			error = device_create_file(dev, *attr);
			if (error) {
				while (--attr >= device_attributes)
					device_remove_file(dev, *attr);
				return;
			}
		} else {
			device_remove_file(dev, *attr);
		}

	}

}

/**
 * gpmi_nfc_probe() - Probes for a device and, if possible, takes ownership.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int gpmi_nfc_probe(struct platform_device *pdev)
{
	int                            error  = 0;
	struct device                  *dev   = &pdev->dev;
	struct gpmi_nfc_platform_data  *pdata = pdev->dev.platform_data;
	struct gpmi_nfc_data           *this  = 0;

	/* Validate the platform device data. */

	error = validate_the_platform(pdev);

	if (error)
		goto exit_validate_platform;

	/* Allocate memory for the per-device data. */

	this = kzalloc(sizeof(*this), GFP_KERNEL);

	if (!this) {
		dev_err(dev, "Failed to allocate per-device memory\n");
		error = -ENOMEM;
		goto exit_allocate_this;
	}

	/* Set up our data structures. */

	platform_set_drvdata(pdev, this);

	this->pdev  = pdev;
	this->dev   = &pdev->dev;
	this->pdata = pdata;

	/* Acquire the resources we need. */

	error = acquire_resources(this);

	if (error)
		goto exit_acquire_resources;

	/* Set up the NFC. */

	error = set_up_nfc_hal(this);

	if (error)
		goto exit_nfc_init;

	/* Set up the platform. */

	if (pdata->platform_init)
		error = pdata->platform_init(pdata->max_chip_count);

	if (error)
		goto exit_platform_init;

	/* Set up the Boot ROM Helper. */

	error = set_up_boot_rom_helper(this);

	if (error)
		goto exit_boot_rom_helper_init;

	/* Initialize the MTD Interface Layer. */

	error = gpmi_nfc_mil_init(this);

	if (error)
		goto exit_mil_init;

	/* Create sysfs entries for this device. */

	manage_sysfs_files(this, true);

	/* Return success. */

	return 0;

	/* Error return paths begin here. */

exit_mil_init:
exit_boot_rom_helper_init:
	if (pdata->platform_exit)
		pdata->platform_exit(pdata->max_chip_count);
exit_platform_init:
	this->nfc->exit(this);
exit_nfc_init:
	release_resources(this);
exit_acquire_resources:
	platform_set_drvdata(pdev, NULL);
	kfree(this);
exit_allocate_this:
exit_validate_platform:
	return error;

}

/**
 * gpmi_nfc_remove() - Dissociates this driver from the given device.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int __exit gpmi_nfc_remove(struct platform_device *pdev)
{
	struct gpmi_nfc_data           *this = platform_get_drvdata(pdev);
	struct gpmi_nfc_platform_data  *pdata = this->pdata;

	manage_sysfs_files(this, false);
	gpmi_nfc_mil_exit(this);
	if (pdata->platform_exit)
		pdata->platform_exit(pdata->max_chip_count);
	this->nfc->exit(this);
	release_resources(this);
	platform_set_drvdata(pdev, NULL);
	kfree(this);

	return 0;
}

#ifdef CONFIG_PM

/**
 * gpmi_nfc_suspend() - Puts the NFC into a low power state.
 *
 * @pdev:  A pointer to the platform device data structure.
 * @state: The new power state.
 */
static int gpmi_nfc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

/**
 * gpmi_nfc_resume() - Brings the NFC back from a low power state.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int gpmi_nfc_resume(struct platform_device *pdev)
{
	return 0;
}

#else

#define suspend  NULL
#define resume   NULL

#endif  /* CONFIG_PM */

/*
 * This structure represents this driver to the platform management system.
 */
static struct platform_driver gpmi_nfc_driver = {
	.driver = {
		.name = GPMI_NFC_DRIVER_NAME,
	},
	.probe   = gpmi_nfc_probe,
	.remove  = __exit_p(gpmi_nfc_remove),
	.suspend = gpmi_nfc_suspend,
	.resume  = gpmi_nfc_resume,
};

/**
 * gpmi_nfc_init() - Initializes this module.
 */
static int __init gpmi_nfc_init(void)
{

	pr_info("i.MX GPMI NFC\n");

	/* Register this driver with the platform management system. */

	if (platform_driver_register(&gpmi_nfc_driver) != 0) {
		pr_err("i.MX GPMI NFC driver registration failed\n");
		return -ENODEV;
	}

	/* Return success. */

	return 0;

}

/**
 * gpmi_nfc_exit() - Deactivates this module.
 */
static void __exit gpmi_nfc_exit(void)
{
	platform_driver_unregister(&gpmi_nfc_driver);
}

module_init(gpmi_nfc_init);
module_exit(gpmi_nfc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX GPMI NAND Flash Controller Driver");
MODULE_LICENSE("GPL");
