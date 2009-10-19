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

/* fingerprints of BCB that can be found on STMP-formatted flash */
#define SIG1		"STMP"
#define SIG_NCB		"NCB "
#define SIG_LDLB	"LDLB"
#define SIG_DBBT	"DBBT"
#define SIG_SIZE	4

struct gpmi_nand_timing {
	u8 data_setup;
	u8 data_hold;
	u8 address_setup;
	u8 dsample_time;
};

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
 * Sizes of data buffers to exchange commands/data with NAND chip
 * Default values cover 4K NAND page (4096 data bytes + 218 bytes OOB)
 */
#define GPMI_CMD_BUF_SZ		10
#define GPMI_DATA_BUF_SZ	NAND_MAX_PAGESIZE
#define GPMI_WRITE_BUF_SZ	NAND_MAX_PAGESIZE
#define GPMI_OOB_BUF_SZ		NAND_MAX_OOBSIZE

#define GPMI_MAX_CHIPS		10

struct gpmi_hwecc_chip {
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

/* HWECC chips */
struct gpmi_hwecc_chip *gpmi_hwecc_chip_find(char *name);
void gpmi_hwecc_chip_add(struct gpmi_hwecc_chip *chip);
void gpmi_hwecc_chip_remove(struct gpmi_hwecc_chip *chip);
int bch_init(void);
int ecc8_init(void);
void bch_exit(void);
void ecc8_exit(void);


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
	struct mtd_partition
				*parts;
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

	struct gpmi_hwecc_chip *hc;

	int numchips;
	int custom_partitions;
	struct mtd_info *masters[GPMI_MAX_CHIPS];
	struct mtd_partition chip_partitions[GPMI_MAX_CHIPS];
	int n_concat;
	struct mtd_info *concat[GPMI_MAX_CHIPS];
	struct mtd_info *concat_mtd;
};

extern struct gpmi_nand_timing gpmi_safe_timing;

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

static inline int gpmi_block_badness(struct nand_chip *chip,
					int block)
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
