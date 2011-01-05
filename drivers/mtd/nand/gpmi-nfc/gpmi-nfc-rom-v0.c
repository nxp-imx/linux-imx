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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "gpmi-nfc.h"

/*
 * Useful variables for Boot ROM Helper version 0.
 */

static const char  *fingerprint = "STMP";

/**
 * set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int set_geometry(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data	*pdata    =  this->pdata;
	struct boot_rom_geometry	*geometry = &this->rom_geometry;
	struct nand_chip		*nand     = &this->mil.nand;
	int                             error;

	/* Version-independent geometry. */
	error = gpmi_nfc_rom_helper_set_geometry(this);
	if (error)
		return error;

	/*
	 * Check if the platform data indicates we are to protect the boot area.
	 */
	if (!pdata->boot_area_size_in_bytes) {
		geometry->boot_area_count         = 0;
		geometry->boot_area_size_in_bytes = 0;
		return 0;
	}

	/*
	 * If control arrives here, we are supposed to set up partitions to
	 * protect the boot areas. In this version of the ROM, the number of
	 * boot areas and their size depends on the number of chips.
	 */
	if (nand->numchips == 1) {
		geometry->boot_area_count = 1;
		geometry->boot_area_size_in_bytes =
					pdata->boot_area_size_in_bytes * 2;
	} else {
		geometry->boot_area_count = 2;
		geometry->boot_area_size_in_bytes =
					pdata->boot_area_size_in_bytes;
	}
	return 0;
}

/**
 * check_transcription_stamp() - Checks for a transcription stamp.
 *
 * Returns 0 if a stamp is not found.
 *
 * @this:  Per-device data.
 */
static int check_transcription_stamp(struct gpmi_nfc_data *this)
{
	struct boot_rom_geometry  *rom_geo  = &this->rom_geometry;
	struct mil                *mil      = &this->mil;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_chip          *nand     = &mil->nand;
	unsigned int              search_area_size_in_strides;
	unsigned int              stride;
	unsigned int              page;
	loff_t                    byte;
	uint8_t                   *buffer = nand->buffers->databuf;
	int                       saved_chip_number;
	int                       found_an_ncb_fingerprint = false;

	/* Compute the number of strides in a search area. */
	search_area_size_in_strides = 1 << rom_geo->search_area_stride_exponent;

	/* Select chip 0. */
	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	/*
	 * Loop through the first search area, looking for the NCB fingerprint.
	 */
	pr_info("Scanning for an NCB fingerprint...\n");

	for (stride = 0; stride < search_area_size_in_strides; stride++) {
		/* Compute the page and byte addresses. */
		page = stride * rom_geo->stride_size_in_pages;
		byte = page   * mtd->writesize;

		pr_info("  Looking for a fingerprint in page 0x%x\n", page);

		/*
		 * Read the NCB fingerprint. The fingerprint is four bytes long
		 * and starts in the 12th byte of the page.
		 */
		nand->cmdfunc(mtd, NAND_CMD_READ0, 12, page);
		nand->read_buf(mtd, buffer, strlen(fingerprint));

		/* Look for the fingerprint. */
		if (!memcmp(buffer, fingerprint, strlen(fingerprint))) {
			found_an_ncb_fingerprint = true;
			break;
		}

	}

	/* Deselect chip 0. */
	nand->select_chip(mtd, saved_chip_number);

	if (found_an_ncb_fingerprint)
		pr_info("  Found a fingerprint\n");
	else
		pr_info("  No fingerprint found\n");
	return found_an_ncb_fingerprint;
}

/**
 * write_transcription_stamp() - Writes a transcription stamp.
 *
 * @this:  Per-device data.
 */
static int write_transcription_stamp(struct gpmi_nfc_data *this)
{
	struct device             *dev      =  this->dev;
	struct boot_rom_geometry  *rom_geo  = &this->rom_geometry;
	struct nand_device_info	  *info     = &this->device_info;
	struct mil                *mil      = &this->mil;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_chip          *nand     = &mil->nand;
	unsigned int              block_size_in_pages;
	unsigned int              search_area_size_in_strides;
	unsigned int              search_area_size_in_pages;
	unsigned int              search_area_size_in_blocks;
	unsigned int              block;
	unsigned int              stride;
	unsigned int              page;
	loff_t                    byte;
	uint8_t                   *buffer = nand->buffers->databuf;
	int                       saved_chip_number;
	int                       status;

	/* Compute the search area geometry. */
	block_size_in_pages = info->block_size_in_pages;
	search_area_size_in_strides = 1 << rom_geo->search_area_stride_exponent;
	search_area_size_in_pages = search_area_size_in_strides *
					rom_geo->stride_size_in_pages;
	search_area_size_in_blocks =
		  (search_area_size_in_pages + (block_size_in_pages - 1)) /
				    block_size_in_pages;

	#if defined(DETAILED_INFO)
	pr_info("--------------------\n");
	pr_info("Search Area Geometry\n");
	pr_info("--------------------\n");
	pr_info("Search Area Size in Blocks : %u", search_area_size_in_blocks);
	pr_info("Search Area Size in Strides: %u", search_area_size_in_strides);
	pr_info("Search Area Size in Pages  : %u", search_area_size_in_pages);
	#endif

	/* Select chip 0. */
	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	/* Loop over blocks in the first search area, erasing them. */
	pr_info("Erasing the search area...\n");

	for (block = 0; block < search_area_size_in_blocks; block++) {
		/* Compute the page address. */
		page = block * block_size_in_pages;

		/* Erase this block. */
		pr_info("  Erasing block 0x%x\n", block);
		nand->cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
		nand->cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1);

		/* Wait for the erase to finish. */
		status = nand->waitfunc(mtd, nand);
		if (status & NAND_STATUS_FAIL)
			dev_err(dev, "[%s] Erase failed.\n", __func__);
	}

	/* Write the NCB fingerprint into the page buffer. */
	memset(buffer, ~0, mtd->writesize);
	memset(nand->oob_poi, ~0, mtd->oobsize);
	memcpy(buffer + 12, fingerprint, strlen(fingerprint));

	/* Loop through the first search area, writing NCB fingerprints. */
	pr_info("Writing NCB fingerprints...\n");
	for (stride = 0; stride < search_area_size_in_strides; stride++) {
		/* Compute the page and byte addresses. */
		page = stride * rom_geo->stride_size_in_pages;
		byte = page   * mtd->writesize;

		/* Write the first page of the current stride. */
		pr_info("  Writing an NCB fingerprint in page 0x%x\n", page);
		nand->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
		nand->ecc.write_page_raw(mtd, nand, buffer);
		nand->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

		/* Wait for the write to finish. */
		status = nand->waitfunc(mtd, nand);
		if (status & NAND_STATUS_FAIL)
			dev_err(dev, "[%s] Write failed.\n", __func__);
	}

	/* Deselect chip 0. */
	nand->select_chip(mtd, saved_chip_number);
	return 0;
}

static int imx23_rom_extra_init(struct gpmi_nfc_data  *this)
{
	struct device             *dev      =  this->dev;
	struct mil                *mil      = &this->mil;
	struct nand_chip          *nand     = &mil->nand;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_device_info	  *info     = &this->device_info;
	unsigned int              block_count;
	unsigned int              block;
	int                       chip;
	int                       page;
	loff_t                    byte;
	uint8_t                   block_mark;
	int                       error = 0;

	/*
	 * If control arrives here, we can't use block mark swapping, which
	 * means we're forced to use transcription. First, scan for the
	 * transcription stamp. If we find it, then we don't have to do
	 * anything -- the block marks are already transcribed.
	 */
	if (check_transcription_stamp(this))
		return 0;

	/*
	 * If control arrives here, we couldn't find a transcription stamp, so
	 * so we presume the block marks are in the conventional location.
	 */
	pr_info("Transcribing bad block marks...\n");

	/* Compute the number of blocks in the entire medium. */
	block_count = info->chip_size_in_bytes >> nand->phys_erase_shift;

	/*
	 * Loop over all the blocks in the medium, transcribing block marks as
	 * we go.
	 */
	for (block = 0; block < block_count; block++) {
		/*
		 * Compute the chip, page and byte addresses for this block's
		 * conventional mark.
		 */
		chip = block >> (nand->chip_shift - nand->phys_erase_shift);
		page = block << (nand->phys_erase_shift - nand->page_shift);
		byte = block <<  nand->phys_erase_shift;

		/* Select the chip. */
		nand->select_chip(mtd, chip);

		/* Send the command to read the conventional block mark. */
		nand->cmdfunc(mtd, NAND_CMD_READ0, mtd->writesize, page);

		/* Read the conventional block mark. */
		block_mark = nand->read_byte(mtd);

		/*
		 * Check if the block is marked bad. If so, we need to mark it
		 * again, but this time the result will be a mark in the
		 * location where we transcribe block marks.
		 *
		 * Notice that we have to explicitly set the marking_a_bad_block
		 * member before we call through the block_markbad function
		 * pointer in the owning struct nand_chip. If we could call
		 * though the block_markbad function pointer in the owning
		 * struct mtd_info, which we have hooked, then this would be
		 * taken care of for us. Unfortunately, we can't because that
		 * higher-level code path will do things like consulting the
		 * in-memory bad block table -- which doesn't even exist yet!
		 * So, we have to call at a lower level and handle some details
		 * ourselves.
		 */
		if (block_mark != 0xff) {
			pr_info("Transcribing mark in block %u\n", block);
			mil->marking_a_bad_block = true;
			error = nand->block_markbad(mtd, byte);
			mil->marking_a_bad_block = false;
			if (error)
				dev_err(dev, "Failed to mark block bad with "
							"error %d\n", error);
		}

		/* Deselect the chip. */
		nand->select_chip(mtd, -1);
	}

	/* Write the stamp that indicates we've transcribed the block marks. */
	write_transcription_stamp(this);
	return 0;
}

/* This structure represents the Boot ROM Helper for this version. */
struct boot_rom_helper  gpmi_nfc_boot_rom_helper_v0 = {
	.version                   = 0,
	.description               = "Single/dual-chip boot area, "
						"no block mark swapping",
	.swap_block_mark           = false,
	.set_geometry              = set_geometry,
	.rom_extra_init		   = imx23_rom_extra_init,
};
