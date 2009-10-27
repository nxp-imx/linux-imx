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
#ifndef __DRIVERS_GPMI_H
#define __DRIVERS_GPMI_H

#include <linux/mtd/partitions.h>
#include <linux/timer.h>
#include <mach/gpmi.h>
#include <mach/regs-gpmi.h>
#include <mach/regs-apbh.h>
#include <mach/dma.h>
#ifdef CONFIG_MTD_NAND_GPMI_BCH
#include <mach/regs-bch.h>
#endif
#include <mach/regs-ecc8.h>

#include "gpmi-hamming-22-16.h"

#define GPMI_ECC4_WR \
	(BM_GPMI_ECCCTRL_ENABLE_ECC | \
	BF(BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_4_BIT, GPMI_ECCCTRL_ECC_CMD))
#define GPMI_ECC4_RD \
	(BM_GPMI_ECCCTRL_ENABLE_ECC | \
	BF(BV_GPMI_ECCCTRL_ECC_CMD__DECODE_4_BIT, GPMI_ECCCTRL_ECC_CMD))
#define GPMI_ECC8_WR \
	(BM_GPMI_ECCCTRL_ENABLE_ECC | \
	BF(BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_8_BIT, GPMI_ECCCTRL_ECC_CMD))
#define GPMI_ECC8_RD \
	(BM_GPMI_ECCCTRL_ENABLE_ECC | \
	BF(BV_GPMI_ECCCTRL_ECC_CMD__DECODE_8_BIT, GPMI_ECCCTRL_ECC_CMD))

/* Fingerprints for Boot Control Blocks. */

#define SIG1		"STMP"
#define SIG_NCB		"NCB "
#define SIG_LDLB	"LDLB"
#define SIG_DBBT	"DBBT"
#define SIG_SIZE	4

/**
 * struct gpmi_nand_timing - NAND Flash timing parameters.
 *
 * This structure contains the four fundamental timing attributes for the
 * NAND Flash bus. These values are expressed in GPMI clock cycles or half
 * cycles, depending on how the GPMI clock is configured. See the hardware
 * reference manual for details.
 *
 * @data_setup:     The data setup time in nanoseconds.
 * @data_hold:      The data hold time in nanoseconds.
 * @address_setup:  The address setup time in nanoseconds.
 * @dsample_time:   The data sample delay in nanoseconds.
 */

struct gpmi_nand_timing {
	u8 data_setup;
	u8 data_hold;
	u8 address_setup;
	u8 dsample_time;
};

/**
 * struct gpmi_bcb_info - Information obtained from Boot Control Blocks.
 *
 * @timing:        Timing values extracted from an NCB.
 * @ncbblock:      The offset within the MTD at which the NCB was found.
 * @pre_ncb:
 * @pre_ncb_size:
 */

struct gpmi_bcb_info {
	struct gpmi_nand_timing timing;
	loff_t ncbblock;
	const void *pre_ncb;
	size_t pre_ncb_size;
};

struct gpmi_ncb;

int gpmi_erase(struct mtd_info *mtd, struct erase_info *instr);
int gpmi_block_markbad(struct mtd_info *mtd, loff_t ofs);
int gpmi_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip);
int gpmi_scan_bbt(struct mtd_info *mtd);
int gpmi_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip);
#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES
int gpmi_sysfs(struct platform_device *p, int create);
#endif
int gpmi_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			int page, int sndcmd);
void gpmi_set_timings(struct platform_device *pdev,
			struct gpmi_nand_timing *tm);
int gpmi_write_ncb(struct mtd_info *mtd, struct gpmi_bcb_info *b);

unsigned gpmi_hamming_ecc_size_22_16(int block_size);
void gpmi_encode_hamming_ncb_22_16(void *source_block, size_t source_size,
		void *target_block, size_t target_size);
void gpmi_encode_hamming_22_16(void *source_block, size_t src_size,
		void *source_ecc, size_t ecc_size);
int gpmi_verify_hamming_22_16(void *data, u8 *parity, size_t size);

unsigned gpmi_hamming_ecc_size_13_8(int block_size);
void gpmi_encode_hamming_ncb_13_8(void *source_block, size_t source_size,
		void *target_block, size_t target_size);
void gpmi_encode_hamming_13_8(void *source_block, size_t src_size,
		void *source_ecc, size_t ecc_size);
int gpmi_verify_hamming_13_8(void *data, u8 *parity, size_t size);

#define GPMI_DMA_MAX_CHAIN	20	/* max DMA commands in chain */

/*
 * Sizes of data buffers to exchange commands/data with NAND chip.
 * Default values cover 4K NAND page (4096 data bytes + 218 bytes OOB)
 */
#define GPMI_CMD_BUF_SZ		10
#define GPMI_DATA_BUF_SZ	NAND_MAX_PAGESIZE
#define GPMI_WRITE_BUF_SZ	NAND_MAX_PAGESIZE
#define GPMI_OOB_BUF_SZ		NAND_MAX_OOBSIZE

#define GPMI_MAX_CHIPS		10

/**
 * struct gpmi_ecc_descriptor - Abstract description of ECC.
 *
 * @name:   The name of the ECC represented by this structure.
 * @list:   Infrastructure for the list to which this structure belongs.
 * @setup:  A pointer to a function that prepares the ECC to function.
 * @reset:  A pointer to a function that resets the ECC to a known state. This
 *          pointer is currently never used, and probably should be removed.
 * @read:   A pointer to a function that fills in a given DMA chain such that
 *          a page read will pass through the owning ECC.
 * @write:  A pointer to a function that fills in a given DMA chain such that
 *          a page write will pass through the owning ECC.
 * @stat:   A pointer to a function that reports on ECC statistics for
 *          the preceding read operation.
 */

struct gpmi_ecc_descriptor {
	char name[40];
	struct list_head list;
	int (*setup)(void *ctx, int index, int writesize, int oobsize);
	int (*reset)(void *ctx, int index);
	int (*read)(void *ctx, int index,
			struct stmp3xxx_dma_descriptor *chain,
			dma_addr_t error,
			dma_addr_t page, dma_addr_t oob);
	int (*write)(void *ctx, int index,
			struct stmp3xxx_dma_descriptor *chain,
			dma_addr_t error,
			dma_addr_t page, dma_addr_t oob);
	int (*stat)(void *ctx, int index, struct mtd_ecc_stats *r);
};

/* ECC descriptor management. */

struct gpmi_ecc_descriptor *gpmi_ecc_find(char *name);
void   gpmi_ecc_add(struct gpmi_ecc_descriptor *chip);
void   gpmi_ecc_remove(struct gpmi_ecc_descriptor *chip);

/* Housecleaning functions for the ECC hardware blocks. */

int bch_init(void);
int ecc8_init(void);
void bch_exit(void);
void ecc8_exit(void);

/**
 * struct gpmi_nand_data -
 *
 * @io_base:              The base I/O address of of the GPMI registers.
 * @clk:                  A pointer to the structure that represents the GPMI
 *                        clock.
 * @irq:                  The GPMI interrupt request number.
 * @inactivity_timer:     A pointer to a timer the driver uses to shut itself
 *                        down after periods of inactivity.
 * @self_suspended:       Indicates the driver suspended itself, rather than
 *                        being suspended by higher layers of software. This is
 *                        important because it effects how the driver wakes
 *                        itself back up.
 * @use_count:            Used within the driver to hold off suspension until
 *                        all operations are complete.
 * @regulator:            A pointer to the structure that represents the
 *                        power regulator supplying power to the GPMI.
 * @reg_uA:               The GPMI current limit, in uA.
 * @ignorebad:            Forces the driver to report that all blocks are good.
 * @bbt:                  Used to save a pointer to the in-memory NAND Flash MTD
 *                        Bad Block Table if the "ignorebad" flag is turned on
 *                        through the corresponding sysfs node.
 * @nand:                 The data structure that represents this NAND Flash
 *                        medium to the MTD NAND Flash system.
 * @mtd:                  The data structure that represents this NAND Flash
 *                        medium to MTD.
 * @dev:                  A pointer to the owning struct device.
 * @nr_parts:             If the driver receives partition descriptions from an
 *                        external parser (command line, etc.), then this is
 *                        the number of discovered partitions.
 * @parts:                If the driver receives partition descriptions from an
 *                        external parser (command line, etc.), then this is a
 *                        pointer to the array of discovered partitions.
 * @done:                 A struct completion used to manage GPMI interrupts.
 * @cmd_buffer:
 * @cmd_buffer_handle:
 * @cmd_buffer_size:
 * @cmd_buffer_sz:        The number of command and address bytes queued up,
 *                        waiting for transmission to the NAND Flash.
 * @write_buffer:
 * @write_buffer_handle:
 * @write_buffer_size:
 * @read_buffer:
 * @read_buffer_handle:
 * @data_buffer:
 * @data_buffer_handle:
 * @data_buffer_size:
 * @oob_buffer:
 * @oob_buffer_handle:
 * @oob_buffer_size:
 * @verify_buffer:
 * @chips:                An array of data structures, one for each physical
 *                        chip.
 * @cchip:                A pointer to the element within the chips array that
 *                        represents the currently selected chip.
 * @selected_chip:        The currently selectd chip number, or -1 if no chip
 *                        is selected.
 * @hwecc_type_read:
 * @hwecc_type_write:
 * @hwecc:                Never used.
 * @ecc_oob_bytes:        The number of ECC bytes covering the OOB bytes alone.
 * @oob_free:             The total number of OOB bytes.
 * @transcribe_bbmark:    Used by the bad block management code to indicate
 *                        that the medium is in common format and the bad block
 *                        marks must be transcribed.
 * @timing:               The current timings installed in the hardware.
 * @saved_command:        Used to "hook" the NAND Flash MTD default
 *                        implementation for the cmdfunc fuction pointer.
 * @raw_oob_mode:
 * @saved_read_oob:       Used to "hook" the NAND Flash MTD interface function
 *                        for the MTD read_oob fuction pointer.
 * @saved_write_oob:      Used to "hook" the NAND Flash MTD interface function
 *                        for the MTD write_oob fuction pointer.
 * @hc:                   A pointer to a structure that represents the ECC
 *                        in use.
 * @numchips:             The number of physical chips.
 * @custom_partitions:    Indicates that the driver has applied driver-specific
 *                        logic in partitioning the medium. This is important
 *                        because similar logic must be applied in disassembling
 *                        the partitioning when the driver shuts down.
 * @chip_mtds:            An array with an element for each physical chip. If
 *                        there is only one physical chip, the first and only
 *                        element in this array will be a copy of the pointer in
 *                        the "mtd" field (that is, they will point to the same
 *                        structure). If there is more than one physical chip,
 *                        each element in this array is a pointer to a partition
 *                        MTD derived from the "mtd" field, and representing the
 *                        corresponding physical chip.
 * @chip_partitions:      An array of partition descriptions that each represent
 *                        one of the physical chips in the medium.
 * @n_concat:             The number of partitions to be concatenated (pointers
 *                        to the partition MTDs appear in the "concat" field).
 * @concat:               An array of pointers to partition MTDs to be
 *                        concatenated.
 * @concat_mtd:           If "n_concat" is zero, this is NULL. If "n_concat" is
 *                        one, this is a copy of the first and only element in
 *                        the "concat" array. If "n_concat" is greater than one,
 *                        this is a pointer to an MTD that represents the
 *                        concatenation of all the MTDs appearing in "concat".
 */

struct gpmi_nand_data {
	void __iomem *io_base;
	struct clk *clk;
	int irq;
	struct timer_list timer;
	int self_suspended;
	int use_count;
	struct regulator *regulator;
	int reg_uA;

	int ignorebad;
	void *bbt;

	struct nand_chip	chip;
	struct mtd_info		mtd;
	struct platform_device  *dev;

#ifdef CONFIG_MTD_PARTITIONS
	int			nr_parts;
	struct mtd_partition	*parts;
#endif

	struct completion	done;

	u8			*cmd_buffer;
	dma_addr_t		cmd_buffer_handle;
	int			cmd_buffer_size, cmd_buffer_sz;

	u8			*write_buffer;
	dma_addr_t		write_buffer_handle;
	int			write_buffer_size;
	u8			*read_buffer;	/* point in write_buffer */
	dma_addr_t		read_buffer_handle;

	u8			*data_buffer;
	dma_addr_t		data_buffer_handle;
	int			data_buffer_size;

	u8			*oob_buffer;
	dma_addr_t		oob_buffer_handle;
	int			oob_buffer_size;

	void			*verify_buffer;

	struct nchip {
		unsigned dma_ch;
		struct stmp37xx_circ_dma_chain chain;
		struct stmp3xxx_dma_descriptor d[GPMI_DMA_MAX_CHAIN];
		struct stmp3xxx_dma_descriptor error;
		int cs;
	} chips[GPMI_MAX_CHIPS];
	struct nchip *cchip;
	int selected_chip;

	unsigned		hwecc_type_read, hwecc_type_write;
	int			hwecc;

	int			ecc_oob_bytes, oob_free;

	int			transcribe_bbmark;
	struct gpmi_nand_timing timing;

	void (*saved_command)(struct mtd_info *mtd, unsigned int command,
			int column, int page_addr);

	int raw_oob_mode;
	int (*saved_read_oob)(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops);
	int (*saved_write_oob)(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops);

	struct gpmi_ecc_descriptor *hc;

	int numchips;
	int custom_partitions;
	struct mtd_info *chip_mtds[GPMI_MAX_CHIPS];
	struct mtd_partition chip_partitions[GPMI_MAX_CHIPS];
	int n_concat;
	struct mtd_info *concat[GPMI_MAX_CHIPS];
	struct mtd_info *concat_mtd;
};

extern struct gpmi_nand_timing gpmi_safe_timing;

/**
 * struct gpmi_ncb -
 *
 * @fingerprint1:
 * @timing:
 * @pagesize:
 * @page_plus_oob_size:
 * @sectors_per_block:
 * @sector_in_page_mask:
 * @sector_to_page_shift:
 * @num_nands:
 * @fingerprint2:
 */

struct gpmi_ncb {
	u32			fingerprint1;
	struct gpmi_nand_timing timing;
	u32			pagesize;
	u32			page_plus_oob_size;
	u32			sectors_per_block;
	u32			sector_in_page_mask;
	u32			sector_to_page_shift;
	u32			num_nands;
	u32			reserved[3];
	u32			fingerprint2;	  /* offset 0x2C     */
};

/**
 * struct gpmi_ldlb -
 *
 * @fingerprint1:
 * @major:
 * @minor:
 * @sub:
 * @nand_bitmap:
 * @fingerprint2:
 * @fw:
 * @fw_starting_nand:
 * @fw_starting_sector:
 * @fw_sector_stride:
 * @fw_sectors_total:
 * @fw_major:
 * @fw_minor:
 * @fw_sub:
 * @fw_reserved:
 * @bbt_blk:
 * @bbt_blk_backup:
 */

struct gpmi_ldlb {
	u32			fingerprint1;
	u16			major, minor, sub, reserved;
	u32			nand_bitmap;
	u32			reserved1[7];
	u32			fingerprint2;
	struct {
		u32			fw_starting_nand;
		u32			fw_starting_sector;
		u32			fw_sector_stride;
		u32			fw_sectors_total;
	} fw[2];
	u16			fw_major, fw_minor, fw_sub, fw_reserved;
	u32			bbt_blk;
	u32			bbt_blk_backup;
};

static inline void gpmi_block_mark_as(struct nand_chip *chip,
					int block, int mark)
{
	u32 o;
	int shift = (block & 0x03) << 1,
	    index = block >> 2;

	if (chip->bbt) {
		mark &= 0x03;

		o = chip->bbt[index];
		o &= ~(0x03 << shift);
		o |= (mark << shift);
		chip->bbt[index] = o;
	}
}

static inline int gpmi_block_badness(struct nand_chip *chip, int block)
{
	u32 o;
	int shift = (block & 0x03) << 1,
	    index = block >> 2;

	if (chip->bbt) {
		o = (chip->bbt[index] >> shift) & 0x03;
		pr_debug("%s: block = %d, o = %d\n", __func__, block, o);
		return o;
	}
	return -1;
}

#ifdef CONFIG_STMP3XXX_UNIQUE_ID
int __init gpmi_uid_init(const char *name, struct mtd_info *mtd,
		 u_int32_t start, u_int32_t size);
void gpmi_uid_remove(const char *name);
#else
#define gpmi_uid_init(name, mtd, start, size)
#define gpmi_uid_remove(name)
#endif

#endif
