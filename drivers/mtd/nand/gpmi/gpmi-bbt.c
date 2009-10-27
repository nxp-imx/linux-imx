/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <mach/unique-id.h>
#include "gpmi.h"

/*
 * The equivalent of the BOOT_SEARCH_COUNT field in the OTP bits. That is, the
 * logarithm to base 2 of the number of strides in a search area (a stride is
 * 64 pages).
 */

static int boot_search_count;

/*
 * The size, in pages, of a search area stride.
 *
 * This number is dictated by the ROM, so it's not clear why it isn't at least
 * const, or perhaps a macro.
 */

static int stride = 64;

/*
 * Indicates how NCBs are to be comprehended.
 *
 * A value of 0 indicates a format that is easy to develop and test with, but
 *    which the ROM can't actually boot.
 *
 * A value of 1 corresponds to the TA-1 ROM.
 *
 * A value of 3 corresponds to the TA-3 ROM.
 */

static int ncb_version = 3;

module_param(boot_search_count, int, 0400);
module_param(ncb_version, int, 0400);

/*
 * gpmi_read_page -
 *
 * @mtd:    The owning MTD.
 * @start:  The offset at which to begin reading.
 * @data:   A pointer to a buff that will receive the data. This pointer may be
 *          NULL, in which case this function will allocate a buffer.
 * @raw:    If true, indicates that the caller doesn't want to use ECC.
 */
void *gpmi_read_page(struct mtd_info *mtd, loff_t start, void *data, int raw)
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
 * gpmi_write_ncb - Writes an NCB to the medium.
 *
 * @mtd:  The owning MTD.
 * @b:    Boot Control Block information.
 */
int gpmi_write_ncb(struct mtd_info *mtd, struct gpmi_bcb_info *b)
{
	struct gpmi_ncb *ncb = NULL, *unencoded_ncb = NULL;
	struct nand_chip *chip = mtd->priv;
	int err;
	loff_t start = 0;
	struct mtd_oob_ops ops;
	struct erase_info instr;
	int ncb_count;

	/* Allocate an I/O buffer for the NCB page, with OOB. */

	ncb = kzalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!ncb) {
		err = -ENOMEM;
		goto out;
	}

	/* Allocate a buffer within which we will construct the NCB. */

	unencoded_ncb = kzalloc(mtd->writesize, GFP_KERNEL);
	if (!unencoded_ncb) {
		err = -ENOMEM;
		goto out;
	}

	ops.mode = -1; /* if the value is not set in switch below,
			  this will cause BUG. Take care. */

	/*
	 * Check if we got any Boot Control Block information and, specifically,
	 * if it contains an NCB for use to use. If not, we need to construct
	 * our own.
	 */

	if (b && b->pre_ncb)
		memcpy(unencoded_ncb, b->pre_ncb, b->pre_ncb_size);
	else {
		memcpy(&unencoded_ncb->fingerprint1, SIG1, sizeof(u32));
		memcpy(&unencoded_ncb->fingerprint2, SIG_NCB, sizeof(u32));
		if (b)
			unencoded_ncb->timing = b->timing;
	}

	/* Construct the encoded NCB from the unencoded one. */

	switch (ncb_version) {
	case 0:
		ops.mode = MTD_OOB_AUTO;
		memcpy(ncb, unencoded_ncb, sizeof(*unencoded_ncb));
		break;
#ifdef CONFIG_MTD_NAND_GPMI_TA1
	case 1:
		ops.mode = MTD_OOB_RAW;
		gpmi_encode_hamming_ncb_22_16(unencoded_ncb,
			NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
			ncb, mtd->writesize + mtd->oobsize);
		break;
#endif
#ifdef CONFIG_MTD_NAND_GPMI_TA3
	case 3:
		ops.mode = MTD_OOB_RAW;
		gpmi_encode_hamming_ncb_13_8(unencoded_ncb,
			NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
			ncb, mtd->writesize + mtd->oobsize);
		break;
#endif

	default:
		printk(KERN_ERR"Incorrect ncb_version = %d\n", ncb_version);
		err = -EINVAL;
		goto out;
	}

	/* Construct data structures for the write operation. */

	ops.datbuf = (u8 *)ncb;
	ops.len = mtd->writesize;
	ops.oobbuf = (u8 *)ncb + mtd->writesize;
	ops.ooblen = mtd->oobsize;
	ops.ooboffs = 0;

	ncb_count = 0;
	do {
		printk(KERN_NOTICE"GPMI: Trying to store NCB at addr %lx\n",
				(unsigned long)start);

		/*
		 * Attempt to erase the block that will contain the current NCB.
		 */

		memset(&instr, 0, sizeof(instr));
		instr.mtd = mtd;
		instr.addr = start;
		instr.len = (1 << chip->phys_erase_shift);
		err = nand_erase_nand(mtd, &instr, 0);

		/*
		 * Check if the erase worked and, if so, write the NCB.
		 */

		if (err == 0) {
			printk(KERN_NOTICE"GPMI: Erased, storing\n");
			err = nand_do_write_ops(mtd, start, &ops);
			printk(KERN_NOTICE"GPMI: NCB update %s (%d).\n",
					err ? "failed" : "succeeded", err);
		}

		/*
		 * Move to the next block.
		 */

		start += (1 << chip->phys_erase_shift);
		ncb_count++;

	} while (err != 0 && ncb_count < 100);

	/* Tell the caller where we ended up putting the NCB. */

	if (b)
		b->ncbblock = start >> chip->bbt_erase_shift;

out:

	/* Free our buffers. */

	kfree(ncb);
	kfree(unencoded_ncb);

	return 0;
}

/*
 * gpmi_redundancy_check_one -
 *
 * @pg:
 * @dsize:
 * @esize:
 * @offset:
 * @o1:
 * @o2:
 */
static int gpmi_redundancy_check_one(u8 *pg, int dsize, int esize, int offset,
		int o1, int o2)
{
	int r;

	if (o1 == o2)
		return 0;

	r = memcmp(pg + o1 * dsize, pg + o2 * dsize, dsize);
	if (r) {
		pr_debug("DATA copies %d and %d are different: %d\n",
			o1, o2, r);
		return r;
	}

	r = memcmp(pg + o1 * esize + offset,
		   pg + o2 * esize + offset, esize);
	if (r) {
		pr_debug("ECC copies %d and %d are different: %d\n", o1, o2, r);
		return r;
	}
	pr_debug("Both DATA and ECC copies %d and %d are identical\n", o1, o2);
	return r;
}

/*
 * gpmi_redundancy_check -
 *
 * @pg:
 * @dsize:
 * @esize:
 * @ecc_offset:
 */
static int gpmi_redundancy_check(u8 *pg, int dsize, int esize, int ecc_offset)
{
	if (gpmi_redundancy_check_one(pg, dsize, esize, ecc_offset, 0, 1) == 0)
		return 0;
	if (gpmi_redundancy_check_one(pg, dsize, esize, ecc_offset, 0, 2) == 0)
		return 0;
	if (gpmi_redundancy_check_one(pg, dsize, esize, ecc_offset, 1, 2) == 0)
		return 1;
	return -1;
}

/*
 * gpmi_ncb1_redundancy_check_one -
 *
 * @pg:
 * @dsize:
 * @esize:
 * @offset:
 * @o1:
 * @o2:
 */
static inline int gpmi_ncb1_redundancy_check(u8 *pg)
{
	return gpmi_redundancy_check(pg,
		NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES,
		NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES,
		NAND_HC_ECC_OFFSET_FIRST_PARITY_COPY);
}

/*
 * gpmi_scan_sigmatel_bbt -
 *
 * @mtd:  The owning MTD.
 * @nfo:
 */
static int gpmi_scan_sigmatel_bbt(
	struct mtd_info *mtd, struct gpmi_bcb_info *nfo)
{
	int page, r;
	u8 *pg;
	struct gpmi_ncb *result = NULL;

	/*
	 * If the boot search count is 0, make it 1.
	 *
	 * This is old TA1 behavior. The correct answer is to map 0 to 2.
	 */

	if (boot_search_count == 0)
		boot_search_count = 1;

	/* Check for nonsense. */

	if (nfo == NULL)
		return -EINVAL;

	/*
	 * Loop through the medium, searching for the NCB.
	 *
	 * Note that this loop is wrong. Each stride is 64 pages, so it *should*
	 * be:
	 *
	 *	for (page = 0;
	 *     		page < ((1<<boot_search_count) * 64); page += stride) {
	 */

	pg = NULL;
	printk(KERN_NOTICE"Scanning for NCB...\n");
	for (page = 0; page < (1<<boot_search_count); page += stride) {

		/* Read the current page. */

		pg = gpmi_read_page(mtd, page * mtd->writesize,
				pg, ncb_version != 0);

		printk(KERN_NOTICE"GPMI: Checking page 0x%08X\n", page);

		/* Check for NCB version 0. */

		if (ncb_version == 0) {
			if (memcmp(pg, SIG1, SIG_SIZE) != 0)
				continue;
			printk(KERN_NOTICE"GPMI: Signature found at 0x%08X\n",
				 page);
			result = (struct gpmi_ncb *)pg;
		}

#ifdef CONFIG_MTD_NAND_GPMI_TA1
		if (ncb_version == 1) {
			void *dptr, *eccptr;

			if (memcmp(pg, SIG1, SIG_SIZE) != 0)
				continue;
			printk(KERN_NOTICE"GPMI: Signature found at 0x%08X\n",
				 page);

			r = gpmi_ncb1_redundancy_check(pg);

			if (r < 0) {
				printk(KERN_ERR"GPMI: Oops. All three "
					"copies of NCB are differrent!\n");
				continue;
			}

			dptr = pg + r * NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES;
			eccptr = pg + NAND_HC_ECC_OFFSET_FIRST_PARITY_COPY +
				r * NAND_HC_ECC_SIZEOF_PARITY_BLOCK_IN_BYTES;

			if (gpmi_verify_hamming_22_16(dptr, eccptr,
				NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES) < 0) {
				printk(KERN_ERR"Verification failed.\n");
				continue;
			}
			result = (struct gpmi_ncb *)pg;
		}
#endif

#ifdef CONFIG_MTD_NAND_GPMI_TA3

		/* Check for TA-3 NCB handling. */

		if (ncb_version == 3) {

			/*
			 * A valid NCB page contains the following:
			 *
			 *     +------------+
			 *           .
			 *           .
			 *       Don't Care
			 *           .
			 *           .
			 *     +------------+ 1036
			 *     |            |
			 *     |  NCB ECC   |
			 *     |            |
			 *     +------------+  524
			 *     |            |
			 *     |    NCB     |
			 *     |            |
			 *     +------------+   12
			 *     | Don't Care |
			 *     +------------+    0
			 *
			 * Within the NCB, there are three "fingerprints":
			 *
			 *     +-----------+--------------------+
			 *     | Offset In |                    |
			 *     | NCB Page  |    Fingerprint     |
			 *     +-----------+--------------------+
			 *     |   0x0c    | "STMP"  0x504d5453 |
			 *     |   0x38    | "NCB "  0x2042434E |
			 *     |   0x8c    | "RBIN"  0x4E494252 |
			 *     +-----------+--------------------+
			 */

			/* Check for the first signature. */

			if (memcmp(pg + 12, SIG1, SIG_SIZE) != 0)
				continue;

			printk(KERN_NOTICE"GPMI: Signature found at 0x%08X\n",
				 page);

			/* Validate the NCB against the ECC stored with it. */

			if (gpmi_verify_hamming_13_8(
				pg + 12,
				pg + 524,
				NAND_HC_ECC_SIZEOF_DATA_BLOCK_IN_BYTES) < 0) {
				printk(KERN_ERR"Verification failed.\n");
				continue;
			}

			/*
			 * If control arrives here, we found what we were
			 * looking for. We want to return the address of the
			 * NCB itself.
			 */

			result = (struct gpmi_ncb *)(pg + 12);

		}
#endif

		if (result) {
			printk(KERN_NOTICE"GPMI: Valid NCB found "
					"at 0x%08x\n", page);
			nfo->timing = result->timing;
			nfo->ncbblock = page * mtd->writesize;
			break;
		}

	}

	/* Free the page buffer */

	kfree(pg);

	return result != NULL;
}

/**
 * gpmi_scan_bbt - Sets up to manage bad blocks.
 *
 * @mtd:  The owning MTD.
 */
int gpmi_scan_bbt(struct mtd_info *mtd)
{
	struct gpmi_bcb_info stmp_bbt;
	struct nand_chip *this = mtd->priv;
	struct gpmi_nand_data *g = this->priv;
	int r;
	int numblocks, from, i, ign;

	memset(&stmp_bbt, 0, sizeof(stmp_bbt));
	g->transcribe_bbmark = 0;

	/*
	 * The following code assumes the NCB uses the full page. This was true
	 * in the TA1 revision of the boot ROM, but not later versions.
	 */

	/*
	 * Since NCB uses the full page, including BB pattern bits,
	 * driver has to ignore result of gpmi_block_bad when reading
	 * these pages.
	 */
	ign = g->ignorebad;

	g->ignorebad = true; 	/* strictly speaking, I'd have to hide
				 * the BBT too.
				 * But we still scanning it :)	*/

	r = gpmi_scan_sigmatel_bbt(mtd, &stmp_bbt);

	/* and then, driver has to restore the setting */
	g->ignorebad = ign;

	/* Check if we found the NCB. */

	if (r) {

		/*
		 * If control arrives here, the medium has an NCB and is
		 * therefore formatted for SigmaTel hardware. We want to use
		 * the timings from the NCB.
		 */

		printk(KERN_NOTICE"Setting discovered timings: %d:%d:%d:%d\n",
			stmp_bbt.timing.data_setup,
			stmp_bbt.timing.data_hold,
			stmp_bbt.timing.address_setup,
			stmp_bbt.timing.dsample_time);

		gpmi_set_timings(g->dev, &stmp_bbt.timing);
		g->timing = stmp_bbt.timing;

	} else {

		/*
		 * If control arrives here, the medium has no NCB, so we
		 * presume it is in common format. This means we must transcribe
		 * the bad block marks.
		 */

		g->transcribe_bbmark = !0;

		/*
		 * Compute the number of blocks in the entire medium.
		 */

		numblocks = this->chipsize >> this->bbt_erase_shift;

		/*
		 * Loop over all the blocks in the medium, transcribing block
		 * marks as we go.
		 */

		from = 0;
		printk(KERN_NOTICE"Checking BB on common-formatted flash\n");
		for (i = stmp_bbt.ncbblock + 1; i < numblocks; i++) {
			/* check the block and transcribe the bb if needed */
			gpmi_block_bad(mtd, from, 0);
			from += (1 << this->bbt_erase_shift);
		}
	}

	/* Use the reference implementation's BBT scan. */

	r = nand_default_bbt(mtd);

	/* Check if we were transcribing block marks. */

	if (g->transcribe_bbmark) {

		/*
		 * If control arrives here, we set the transcription flag
		 * because we didn't find an NCB. Now that we've
		 * transcribed all the bad block marks, we need to write an
		 * NCB to show that the medium is now formatted for SigmaTel
		 * Hardware.
		 */

		g->transcribe_bbmark = 0;

		/*
		 * Store the timings we discovered from the NCB in the
		 * per-device data structure.
		 */

		stmp_bbt.timing = gpmi_safe_timing;

		/* Write an NCB to the medium. */

		r = gpmi_write_ncb(mtd, &stmp_bbt);

	} else {
		/* NCB found, and its block should be marked as "good" */
		gpmi_block_mark_as(this, stmp_bbt.ncbblock, 0x00);
	}

	return r;
}

/**
 * gpmi_block_bad - Checks if a block is bad, and may transcribes its mark.
 *
 * @mtd:  The owning MTD.
 * @ofs:
 * @getchip:
 */
int gpmi_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)

{
	int page, res = 0;
	struct nand_chip *chip = mtd->priv;
	u16 bad;
	struct gpmi_nand_data *g = chip->priv;
	int chipnr;

	/*
	 * Compute the position of the block mark within the OOB (this code
	 * appears to be wrong).
	 */

	int badblockpos = chip->ecc.steps * chip->ecc.bytes;

	if (g->ignorebad)
		return 0;

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

	/*
	 * If we're transcribing block marks, set the position to that in a
	 * SigmaTel Hardware-format page.
	 */

	if (g->transcribe_bbmark)
		/* bad block marks still are on first byte of OOB */
		badblockpos = 0;

	/* Read the block mark. */

	if (chip->options & NAND_BUSWIDTH_16) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, badblockpos & 0xFE,
			      page);
		bad = cpu_to_le16(chip->read_word(mtd));
		if (badblockpos & 0x1)
			bad >>= 8;
		if ((bad & 0xFF) != 0xff)
			res = 1;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, badblockpos, page);
		if (chip->read_byte(mtd) != 0xff)
			res = 1;
	}

	/*
	 * If we successfully read the block mark, and we're supposed to
	 * transcribe it, then do so.
	 */

	if (g->transcribe_bbmark && res)
		chip->block_markbad(mtd, ofs);

	/*
	 * Deselect the chip.
	 */

	chip->select_chip(mtd, -1);

	return res;
}

#if defined(CONFIG_STMP3XXX_UNIQUE_ID)
/*
 * UID on NAND support
 */
const int uid_size = 256;

struct gpmi_uid_context {
	struct mtd_info *mtd;
	struct nand_chip *nand;
	u_int32_t start;
	u_int32_t size;
};

static int gpmi_read_uid(struct gpmi_uid_context *ctx, void *result)
{
	void *pg = NULL;
	int page, o;
	int status = -ENOENT;
	int h_size = gpmi_hamming_ecc_size_22_16(uid_size);

	for (page = ctx->start >> ctx->nand->page_shift;
	     page < (ctx->start + ctx->size) >> ctx->nand->page_shift;) {
		pr_debug("%s: reading page 0x%x\n", __func__, page);
		if (gpmi_block_bad(ctx->mtd, page * ctx->mtd->writesize, 0)) {
			pr_debug("%s: bad block %x, skipping it\n",
				__func__, page * ctx->mtd->writesize);
			page += (1 << ctx->nand->phys_erase_shift)
					>> ctx->nand->page_shift;
			continue;
		}
		pg = gpmi_read_page(ctx->mtd, page * ctx->mtd->writesize,
						pg, 0);
		if (pg)
			break;
		page++;
	}

	if (!pg)
		return status;

	o = gpmi_redundancy_check(pg, uid_size, h_size, 3 * uid_size);
	if (o >= 0) {
		if (gpmi_verify_hamming_22_16(
				pg + o * uid_size,
				pg + 3 * uid_size + h_size, uid_size) >= 0) {
			memcpy(result, pg + o * uid_size, uid_size);
			status = 0;
		}
	}
	kfree(pg);
	return status;
}

static int gpmi_write_uid(struct gpmi_uid_context *ctx, void *src)
{
	struct mtd_oob_ops ops;
	struct erase_info instr;
	u8 *data = kzalloc(ctx->mtd->writesize + ctx->mtd->oobsize, GFP_KERNEL);
	int h_size = gpmi_hamming_ecc_size_22_16(uid_size);
	char ecc[h_size];
	u_int32_t start;
	int i;
	int err;

	if (!data)
		return -ENOMEM;

	gpmi_encode_hamming_22_16(src, uid_size, ecc, h_size);
	for (i = 0; i < 3; i++) {
		memcpy(data + i * uid_size, src, uid_size);
		memcpy(data + 3 * uid_size + i * h_size, ecc, h_size);
	}

	ops.mode = MTD_OOB_AUTO;
	ops.datbuf = data;
	ops.len = ctx->mtd->writesize;
	ops.oobbuf = NULL;
	ops.ooblen = ctx->mtd->oobsize;
	ops.ooboffs = 0;

	start = ctx->start;

	do {
		memset(&instr, 0, sizeof(instr));
		instr.mtd = ctx->mtd;
		instr.addr = start;
		instr.len = (1 << ctx->nand->phys_erase_shift);
		err = nand_erase_nand(ctx->mtd, &instr, 0);
		if (err == 0)
			err = nand_do_write_ops(ctx->mtd, start, &ops);
		start += (1 << ctx->nand->phys_erase_shift);
		if (start > ctx->start + ctx->size)
			break;
	} while (err != 0);

	return err;
}

static ssize_t gpmi_uid_store(void *context, const char *page,
				size_t count, int ascii)
{
	u8 data[uid_size];

	memset(data, 0, sizeof(data));
	memcpy(data, page, uid_size < count ? uid_size : count);
	gpmi_write_uid(context, data);
	return count;
}

static ssize_t gpmi_uid_show(void *context, char *page, int ascii)
{
	u8 result[uid_size];
	int i;
	char *p = page;
	int r;

	r = gpmi_read_uid(context, result);
	if (r < 0)
		return r;

	if (ascii) {
		for (i = 0; i < uid_size; i++) {
			if (i % 16 == 0) {
				if (i)
					*p++ = '\n';
				sprintf(p, "%04X: ", i);
				p += strlen(p);
			}
			sprintf(p, "%02X ", result[i]);
			p += strlen(p);
		}
		*p++ = '\n';
		return p - page;

	} else {
		memcpy(page, result, uid_size);
		return uid_size;
	}
}

static struct uid_ops gpmi_uid_ops = {
	.id_show 	= gpmi_uid_show,
	.id_store 	= gpmi_uid_store,
};

static struct gpmi_uid_context gpmi_uid_context;

int  __init gpmi_uid_init(const char *name, struct mtd_info *mtd,
			u_int32_t start, u_int32_t size)
{
	gpmi_uid_context.mtd = mtd;
	gpmi_uid_context.nand = mtd->priv;
	gpmi_uid_context.start = start;
	gpmi_uid_context.size = size;
	return uid_provider_init(name, &gpmi_uid_ops, &gpmi_uid_context) ?
			 0 : -EFAULT;
}

void gpmi_uid_remove(const char *name)
{
	uid_provider_remove(name);
}
#endif
