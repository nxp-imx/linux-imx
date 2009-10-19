/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
 *
 * STMP378X BCH hardware ECC engine
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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/dma-mapping.h>

#include <asm/dma.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/regs-gpmi.h>
#include "gpmi.h"

#define BCH_MAX_NANDS 4

static int bch_available(void *context);
static int bch_setup(void *context, int index, int writesize, int oobsize);
static int bch_read(void *context,
		int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob);
static int bch_stat(void *ctx, int index, struct mtd_ecc_stats *r);
static int bch_reset(void *context, int index);

struct bch_state_t {
	struct gpmi_hwecc_chip chip;
	struct {
		struct mtd_ecc_stats stat;
		struct completion done;
		u32 writesize, oobsize;
	} nands[BCH_MAX_NANDS];
};

static struct bch_state_t state = {
	.chip = {
		.name		= "bch",
		.setup		= bch_setup,
		.stat		= bch_stat,
		.read		= bch_read,
		.reset		= bch_reset,
	},
};

static int bch_reset(void *context, int index)
{
	stmp3xxx_reset_block(REGS_BCH_BASE, true);
	stmp3xxx_setl(BM_BCH_CTRL_COMPLETE_IRQ_EN, REGS_BCH_BASE + HW_BCH_CTRL);
	return 0;
}

static int bch_stat(void *context, int index, struct mtd_ecc_stats *r)
{
	struct bch_state_t *state = context;

	*r = state->nands[index].stat;
	state->nands[index].stat.failed = 0;
	state->nands[index].stat.corrected = 0;
	return 0;
}

static irqreturn_t bch_irq(int irq, void *context)
{
	u32 b0, s0;
	struct mtd_ecc_stats stat;
	int r;
	struct bch_state_t *state = context;

	s0 = __raw_readl(REGS_BCH_BASE + HW_BCH_STATUS0);
	r = (s0 & BM_BCH_STATUS0_COMPLETED_CE) >> 16;

	stat.corrected = stat.failed = 0;

	b0 = (s0 & BM_BCH_STATUS0_STATUS_BLK0) >> 8;
	if (b0 <= 4)
		stat.corrected += b0;
	if (b0 == 0xFE)
		stat.failed++;

	if (s0 & BM_BCH_STATUS0_CORRECTED)
		stat.corrected += (s0 & BM_BCH_STATUS0_CORRECTED);
	if (s0 & BM_BCH_STATUS0_UNCORRECTABLE)
		stat.failed++;

	stmp3xxx_clearl(BM_BCH_CTRL_COMPLETE_IRQ, REGS_BCH_BASE + HW_BCH_CTRL);

	pr_debug("%s: chip %d, failed %d, corrected %d\n",
			__func__, r,
			state->nands[r].stat.failed,
			state->nands[r].stat.corrected);
	state->nands[r].stat.corrected += stat.corrected;
	state->nands[r].stat.failed += stat.failed;
	complete(&state->nands[r].done);

	return IRQ_HANDLED;
}

static int bch_available(void *context)
{
	stmp3xxx_reset_block(REGS_BCH_BASE, 0);
	return __raw_readl(REGS_BCH_BASE + HW_BCH_BLOCKNAME) == 0x20484342;
}

static int bch_setup(void *context, int index, int writesize, int oobsize)
{
	struct bch_state_t *state = context;
	u32 layout = (u32)REGS_BCH_BASE + 0x80 + index * 0x20;
	u32 ecc0, eccN;
	u32 reg;
	int meta;

	switch (writesize) {
	case 2048:
		ecc0 = 4;
		eccN = 4;
		meta = 5;
		break;
	case 4096:
		ecc0 = 16;
		eccN = 14;
		meta = 10;
		break;
	default:
		printk(KERN_ERR"%s: cannot tune BCH for page size %d\n",
				__func__, writesize);
		return -EINVAL;
	}

	state->nands[index].oobsize = oobsize;
	state->nands[index].writesize = writesize;

	__raw_writel(BF(writesize/512, BCH_FLASH0LAYOUT0_NBLOCKS) |
		     BF(oobsize, BCH_FLASH0LAYOUT0_META_SIZE) |
		     BF(ecc0 >> 1, BCH_FLASH0LAYOUT0_ECC0) | /* for oob */
		     BF(0x00, BCH_FLASH0LAYOUT0_DATA0_SIZE), layout);
	__raw_writel(BF(writesize + oobsize, BCH_FLASH0LAYOUT1_PAGE_SIZE) |
		     BF(eccN >> 1, BCH_FLASH0LAYOUT1_ECCN) | /* for dblock */
		     BF(512, BCH_FLASH0LAYOUT1_DATAN_SIZE), layout + 0x10);

	/*
	 * since driver only supports CS 0..3, layouts are mapped 1:1 :
	 * 		FLASHnLAYOUT[1,2] => LAYOUTSELECT[n*2:n2*+1]
	 */
	reg = __raw_readl(REGS_BCH_BASE + HW_BCH_LAYOUTSELECT);
	reg &= ~(0x03 << (index * 2));
	reg |= index << (index * 2);
	__raw_writel(reg, REGS_BCH_BASE + HW_BCH_LAYOUTSELECT);

	bch_reset(context, index);

	printk(KERN_DEBUG"%s: CS = %d, LAYOUT = 0x%08X, layout_reg = "
			"0x%08x+0x%08x: 0x%08x+0x%08x\n",
			__func__,
			index, __raw_readl(REGS_BCH_BASE + HW_BCH_LAYOUTSELECT),
			layout, layout + 0x10,
			__raw_readl(layout),
			__raw_readl(layout+0x10));
	return 0;
}

static int bch_read(void *context,
		int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob)
{
	unsigned long readsize = 0;
	u32 bufmask = 0;
	struct bch_state_t *state = context;

	if (!dma_mapping_error(NULL, oob)) {
		bufmask |= BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;
		readsize += state->nands[index].oobsize;
	}
	if (!dma_mapping_error(NULL, page)) {
		bufmask |= (BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE
				& ~BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY);
		readsize += state->nands[index].writesize;
	}

	printk(KERN_DEBUG"readsize = %ld, bufmask = 0x%X\n", readsize, bufmask);
	bch_reset(context, index);

	/* wait for ready */
	chain->command->cmd =
		BF(1, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_NANDWAIT4READY	|
		BM_APBH_CHn_CMD_CHAIN		|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY,
			GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH				|
		BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA, GPMI_CTRL0_ADDRESS) |
		BF(index, GPMI_CTRL0_CS);
	chain->command->alternate = 0;
	chain++;

	/* enable BCH and read NAND data */
	chain->command->cmd =
		BF(6, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_NANDLOCK	|
		BM_APBH_CHn_CMD_CHAIN 		|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__READ, GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH	|
		BF(index, GPMI_CTRL0_CS)		|
		BF(readsize, GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	|
		BF(0x02, GPMI_ECCCTRL_ECC_CMD) 	|
		BF(bufmask, GPMI_ECCCTRL_BUFFER_MASK);
	chain->command->pio_words[3] = readsize;
	chain->command->pio_words[4] = !dma_mapping_error(NULL, page) ? page : 0;
	chain->command->pio_words[5] = !dma_mapping_error(NULL, oob) ? oob : 0;
	chain->command->alternate = 0;
	chain++;

	/* disable BCH block */
	chain->command->cmd =
		BF(3, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_NANDWAIT4READY	|
		BM_APBH_CHn_CMD_NANDLOCK	|
		BM_APBH_CHn_CMD_CHAIN 		|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY,
			GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH	|
		BF(index, GPMI_CTRL0_CS)		|
		BF(readsize, GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->alternate = 0;
	chain++;

	/* and deassert nand lock */
	chain->command->cmd =
		BF(0, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_IRQONCMPLT	|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->alternate = 0;

	return 0;
}

int __init bch_init(void)
{
	int err;

	if (!bch_available(&state.chip))
		return -ENXIO;
	gpmi_hwecc_chip_add(&state.chip);
	err = request_irq(IRQ_BCH, bch_irq, 0, state.chip.name, &state);
	if (err)
		return err;

	printk(KERN_DEBUG"%s: initialized\n", __func__);
	return 0;
}

void bch_exit(void)
{
	free_irq(IRQ_BCH, &state);
	gpmi_hwecc_chip_remove(&state.chip);
}
