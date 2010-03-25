/*
 * Freescale i.MX28 GPMI (General-Purpose-Media-Interface)
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/ctype.h>
#include <mach/dma.h>
#include "gpmi.h"

/* Fingerprints for Boot Control Blocks. */

#define SIG_FCB		"FCB "
#define SIG_DBBT	"DBBT"
#define SIG_SIZE	4

/*
 * The equivalent of the BOOT_SEARCH_COUNT field in the OTP bits. That is, the
 * logarithm to base 2 of the number of strides in a search area (a stride is
 * 64 pages).
 */

static int boot_search_count;

module_param(boot_search_count, int, 0400);

/*
 * The size, in pages, of a search area stride.
 *
 * This number is dictated by the ROM, so it's not clear why it isn't at least
 * const, or perhaps a macro.
 */

static const int stride_size_in_pages = 64;

/*
 * read_page -
 *
 * @mtd:    The owning MTD.
 * @start:  The offset at which to begin reading.
 * @data:   A pointer to a buff that will receive the data. This pointer may be
 *          NULL, in which case this function will allocate a buffer.
 * @raw:    If true, indicates that the caller doesn't want to use ECC.
 */
static void *read_page(struct mtd_info *mtd, loff_t start, void *data, int raw)
{
	int ret;
	struct mtd_oob_ops ops;

	/* If the caller didn't send in his own buffer, allocate one. */

	if (!data)
		data = kzalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!data)
		return NULL;

	/* Check if the caller wants to use ECC. */

	if (raw)
		ops.mode = MTD_OOB_RAW;
	else
		ops.mode = MTD_OOB_PLACE;

	/*
	 * Call nand_do_read_ops() to do the dirty work.
	 */

	ops.datbuf = data;
	ops.len = mtd->writesize;
	ops.oobbuf = data + mtd->writesize;
	ops.ooblen = mtd->oobsize;
	ops.ooboffs = 0;
	ret = nand_do_read_ops(mtd, start, &ops);

	if (ret)
		return NULL;
	return data;
}

/*
 * gpmi_write_fcbs - Writes FCBs to the medium.
 *
 * @mtd:  The owning MTD.
 */
static void gpmi_write_fcbs(struct mtd_info *mtd)
{
	unsigned int  i;
	unsigned int  page_size_in_bytes;
	unsigned int  block_size_in_bytes;
	unsigned int  block_size_in_pages;
	unsigned int  search_area_size_in_strides;
	unsigned int  search_area_size_in_pages;
	unsigned int  search_area_size_in_blocks;
	void  *fcb;
	struct nand_chip *chip = mtd->priv;
	struct mtd_oob_ops ops;
	struct erase_info  instr;

	/* Compute some important facts. */

	page_size_in_bytes  = mtd->writesize;
	block_size_in_bytes = 1 << chip->phys_erase_shift;
	block_size_in_pages = 1 << (chip->phys_erase_shift - chip->page_shift);

	search_area_size_in_strides = (1 << boot_search_count) - 1;
	search_area_size_in_pages   =
			search_area_size_in_strides * stride_size_in_pages + 1;
	search_area_size_in_blocks  =
		(search_area_size_in_pages + (block_size_in_pages - 1)) /
							block_size_in_pages;

	/* Allocate an I/O buffer for the FCB page, with OOB. */

	fcb = kzalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!fcb)
		return;

	/* Write the FCB signature into the buffer. */

	memcpy(((uint8_t *) fcb) + 0x10, SIG_FCB, SIG_SIZE);

	/* Erase the entire search area. */

	for (i = 0; i < search_area_size_in_blocks; i++) {
		memset(&instr, 0, sizeof(instr));
		instr.mtd = mtd;
		instr.addr = i * block_size_in_bytes;
		instr.len = block_size_in_bytes;
		nand_erase_nand(mtd, &instr, 0);
	}

	/* Construct the data structure for the write operation. */

	ops.datbuf = (u8 *)fcb;
	ops.len = mtd->writesize;
	ops.oobbuf = (u8 *)fcb + mtd->writesize;
	ops.ooblen = mtd->oobsize;
	ops.ooboffs = 0;
	ops.mode = MTD_OOB_RAW;

	/* Loop over FCB locations in the search area. */

	for (i = 0; i <= search_area_size_in_strides; i++) {

		printk(KERN_NOTICE"Writing FCB in page 0x%08x\n",
				i * stride_size_in_pages);

		nand_do_write_ops(mtd,
			i * stride_size_in_pages * page_size_in_bytes, &ops);

	}

	/* Free our buffer. */

	kfree(fcb);

}

/*
 * gpmi_scan_for_fcb - Scans the medium for an FCB.
 *
 * @mtd:  The owning MTD.
 */
static int gpmi_scan_for_fcb(struct mtd_info *mtd)
{
	int result = 0;
	int page;
	u8 *pg;

	/* If the boot search count is 0, make it 2. */

	if (boot_search_count == 0)
		boot_search_count = 2;

	/* Loop through the medium, searching for the FCB. */

	printk(KERN_NOTICE"Scanning for FCB...\n");

	pg = NULL;

	for (page = 0;
		page < ((1 << boot_search_count) * stride_size_in_pages);
						page += stride_size_in_pages) {

		/* Read the current page. */

		pg = read_page(mtd, page * mtd->writesize, pg, !0);

		printk(KERN_NOTICE"Looking for FCB in page 0x%08x\n", page);

		/*
		 * A valid FCB page contains the following:
		 *
		 *     +------------+
		 *           .
		 *           .
		 *       Don't Care
		 *           .
		 *           .
		 *     +------------+ 1036
		 *     |            |
		 *     |  FCB ECC   |
		 *     |            |
		 *     +------------+  524
		 *     |            |
		 *     |    FCB     |
		 *     |            |
		 *     +------------+   12
		 *     | Don't Care |
		 *     +------------+    0
		 *
		 * Within the FCB, there is a "fingerprint":
		 *
		 *     +-----------+--------------------+
		 *     | Offset In |                    |
		 *     | FCB Page  |    Fingerprint     |
		 *     +-----------+--------------------+
		 *     |   0x10    | "FCB "  0x46434220 |
		 *     +-----------+--------------------+
		 */

		/* Check for the fingerprint. */

		if (memcmp(pg + 16, SIG_FCB, SIG_SIZE) != 0)
			continue;

		printk(KERN_NOTICE"Found FCB in page 0x%08X\n", page);

		result = !0;

		break;

	}

	if (!result)
		printk(KERN_NOTICE"No FCB found\n");

	/* Free the page buffer */

	kfree(pg);

	return result;

}

/**
 * gpmi_block_bad - Claims all blocks are good.
 *
 * @mtd:      The owning MTD.
 * @ofs:      The offset of the block.
 * @getchip:  ??
 *
 * In principle, this function is called when the NAND Flash MTD system isn't
 * allowed to keep an in-memory bad block table, so it must ask the driver
 * for bad block information.
 *
 * In fact, we permit the NAND Flash MTD system to have an in-memory BBT, so
 * this function is *only* called when we take it away.
 *
 * We take away the in-memory BBT when the user sets the "ignorebad" parameter,
 * which indicates that all blocks should be reported good.
 *
 * Thus, this function is only called when we want *all* blocks to look good,
 * so it need do nothing more than always return success.
 */
int gpmi_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	return 0;
}

/**
 * transcribe_block_mark - Transcribes a block mark.
 *
 * @mtd:  The owning MTD.
 * @ofs:  Identifies the block of interest.
 */
static void transcribe_block_mark(struct mtd_info *mtd, loff_t ofs)

{
	int page;
	struct nand_chip *chip = mtd->priv;
	int chipnr;

	/*
	 * Compute the position of the block mark within the OOB (this code
	 * appears to be wrong).
	 */

	int badblockpos = chip->ecc.steps * chip->ecc.bytes;

	/*
	 * Compute the page address of the first page in the block that contains
	 * the given offset.
	 */

	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	/*
	 * Compute the chip number that contains the given offset, and select
	 * it.
	 */

	chipnr = (int)(ofs >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* bad block marks still are on first byte of OOB */

	badblockpos = 0;

	/* Read the block mark. */

	chip->cmdfunc(mtd, NAND_CMD_READOOB, badblockpos, page);

	if (chip->read_byte(mtd) != 0xff) {
		printk(KERN_NOTICE"Transcribing block mark in block 0x%08x\n",
				(unsigned) (ofs >> chip->phys_erase_shift));
		chip->block_markbad(mtd, ofs);
	}

	/*
	 * Deselect the chip.
	 */

	chip->select_chip(mtd, -1);

}

/**
 * gpmi_scan_bbt - Sets up to manage bad blocks.
 *
 * @mtd:  The owning MTD.
 */
int gpmi_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	int r;
	int transcription_is_needed;
	unsigned int  search_area_size_in_strides;
	unsigned  page;
	unsigned  block;
	int numblocks, from, i;
	struct nand_chip *chip = mtd->priv;

	/* Search for the FCB. */

	transcription_is_needed = !gpmi_scan_for_fcb(mtd);

	/* Check if we found the FCB. */

	if (transcription_is_needed) {

		printk(KERN_NOTICE"Transcribing bad block marks...\n");

		/*
		 * If control arrives here, the medium has no FCB, so we
		 * presume it is in common format. This means we must transcribe
		 * the block marks.
		 *
		 * Compute the number of blocks in the entire medium.
		 */

		numblocks = this->chipsize >> this->bbt_erase_shift;

		/*
		 * Loop over all the blocks in the medium, transcribing block
		 * marks as we go.
		 */

		from = 0;
		for (i = 0; i < numblocks; i++) {
			/* Transcribe the mark in this block, if needed. */
			transcribe_block_mark(mtd, from);
			from += (1 << this->bbt_erase_shift);
		}

		/*
		 * Put an FCB in the medium to indicate the block marks have
		 * been transcribed.
		 */

		gpmi_write_fcbs(mtd);

	}

	/* Use the reference implementation's BBT scan. */

	r = nand_default_bbt(mtd);

	/* Mark all NCB blocks as good. */

	search_area_size_in_strides = (1 << boot_search_count) - 1;

	for (i = 0; i <= search_area_size_in_strides; i++) {

		/* Compute the current FCB page.*/

		page = i * stride_size_in_pages;

		/* Compute the block that contains the current FCB page.*/

		block = page >> (chip->phys_erase_shift - chip->page_shift);

		/* Mark the block good. */

		printk(KERN_NOTICE"Forcing good FCB block 0x%08x\n", block);

		gpmi_block_mark_as(this, block, 0x00);

	}

	return r;
}
