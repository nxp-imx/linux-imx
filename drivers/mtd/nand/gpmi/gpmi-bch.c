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
static int bch_write(void *context,
		int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob);
static int bch_stat(void *ctx, int index, struct mtd_ecc_stats *r);
static int bch_reset(void *context, int index);

/**
 * bch_state_t - Describes the state of the BCH ECC.
 *
 * @chip:       A descriptor the GPMI driver uses to track this ECC.
 * @nands:      An array of elements, each of which represents a physical chip.
 * @stat:       Used by the interrupt level to communicate ECC statistics to the
 *              base level.
 * @done:       A struct completion used to manage ECC interrupts.
 * @writesize:  The page data size.
 * @oobsize:    The page OOB size.
 */

struct bch_state_t {
	struct gpmi_ecc_descriptor chip;
	struct {
		struct mtd_ecc_stats stat;
		struct completion done;
		u32 writesize, oobsize;
		u32 ecc0, eccn, metasize;
	} nands[BCH_MAX_NANDS];
};

/* The singleton struct bch_state_t for the BCH ECC. */

static struct bch_state_t state = {
	.chip = {
		.name		= "bch",
		.setup		= bch_setup,
		.stat		= bch_stat,
		.read		= bch_read,
		.write		= bch_write,
		.reset		= bch_reset,
	},
};

/**
 * bch_reset - Resets the BCH.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 * @index:    ??
 */
static int bch_reset(void *context, int index)
{
	stmp3xxx_reset_block(REGS_BCH_BASE, true);
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN,
		REGS_BCH_BASE + HW_BCH_CTRL_SET);
	return 0;
}

/**
 * bch_stat - Gather statistics and clean up after a read operation.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 * @index:    ??
 * @r:        A statistics structure that will receive the results of the most
 *            recent operation.
 */
static int bch_stat(void *context, int index, struct mtd_ecc_stats *r)
{
	struct bch_state_t *state = context;

	wait_for_completion(&state->nands[index].done);

	*r = state->nands[index].stat;
	state->nands[index].stat.failed = 0;
	state->nands[index].stat.corrected = 0;
	return 0;
}

/**
 * bch_irq - Interrupt handler for the BCH hardware.
 *
 * This function gains control when the BCH hardware interrupts. It acknowledges
 * the interrupt and gathers status information.
 *
 * @irq:      The interrupt number.
 * @context:  Context data -- a pointer to a struct bch_state_t.
 */
static irqreturn_t bch_irq(int irq, void *context)
{
	u32 b0, s0, ecc0;
	struct mtd_ecc_stats stat;
	int r;
	struct bch_state_t *state = context;

	s0 = __raw_readl(REGS_BCH_BASE + HW_BCH_STATUS0);
	r = (s0 & BM_BCH_STATUS0_COMPLETED_CE) >> 16;

	ecc0 = state->nands[r].ecc0;
	stat.corrected = stat.failed = 0;

	b0 = (s0 & BM_BCH_STATUS0_STATUS_BLK0) >> 8;
	if (b0 <= ecc0)
		stat.corrected += b0;
	if (b0 == 0xFE)
		stat.failed++;

	if (s0 & BM_BCH_STATUS0_UNCORRECTABLE)
		stat.failed++;

	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ, REGS_BCH_BASE + HW_BCH_CTRL_CLR);

	pr_debug("%s: chip %d, failed %d, corrected %d\n",
			__func__, r,
			state->nands[r].stat.failed,
			state->nands[r].stat.corrected);
	state->nands[r].stat.corrected += stat.corrected;
	state->nands[r].stat.failed += stat.failed;
	complete(&state->nands[r].done);

	return IRQ_HANDLED;
}

/**
 * bch_available - Returns whether the BCH hardware is available.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 */
static int bch_available(void *context)
{
	stmp3xxx_reset_block(REGS_BCH_BASE, true);
	return __raw_readl(REGS_BCH_BASE + HW_BCH_BLOCKNAME) == 0x20484342;
}

/**
 * bch_setup - Set up BCH for use.
 *
 * If the GPMI driver decides to use this ECC, it will call this function once,
 * before it starts any operations.
 *
 * @context:    Context data -- a pointer to a struct bch_state_t.
 * @index:      ??
 * @writesize:  The page data size.
 * @oobsize:    The page OOB size.
 */
static int bch_setup(void *context, int index, int writesize, int oobsize)
{
	struct bch_state_t *state = context;
	u32 layout = (u32)REGS_BCH_BASE + 0x80 + index * 0x20;
	u32 ecc0, eccn, metasize;
	u32 reg;

	switch (writesize) {
	case 2048:
		ecc0 = 8;
		eccn = 8;
		metasize = 10;
		break;
	case 4096:
		if (oobsize == 128) {
			ecc0 = 8;
			eccn = 8;
		} else {
			ecc0 = 16;
			eccn = 16;
		}

		metasize = 10;
		break;
	default:
		printk(KERN_ERR"%s: cannot tune BCH for page size %d\n",
				__func__, writesize);
		return -EINVAL;
	}

	state->nands[index].oobsize = oobsize;
	state->nands[index].writesize = writesize;
	state->nands[index].metasize = metasize;
	state->nands[index].ecc0 = ecc0;
	state->nands[index].eccn = eccn;

	__raw_writel(BF(writesize/512 - 1, BCH_FLASH0LAYOUT0_NBLOCKS) |
		     BF(metasize, BCH_FLASH0LAYOUT0_META_SIZE) |
		     BF(ecc0 >> 1, BCH_FLASH0LAYOUT0_ECC0) | /* for oob */
		     BF(512, BCH_FLASH0LAYOUT0_DATA0_SIZE), layout);
	__raw_writel(BF(writesize + oobsize, BCH_FLASH0LAYOUT1_PAGE_SIZE) |
		     BF(eccn >> 1, BCH_FLASH0LAYOUT1_ECCN) | /* for dblock */
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

/**
 * bch_read - Fill in a DMA chain to read a page.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 * @cs:       The chip number to read.
 * @chain:    The main descriptor of the DMA chain to fill.
 * @error:    ??
 * @page:     Physical address of the target page data buffer.
 * @oob:      Physical address of the target OOB data buffer.
 *
 * Return: status of operation -- 0 on success
 */
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
		BF(index, GPMI_CTRL0_CS)	|
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
		BM_GPMI_CTRL0_WORD_LENGTH	 |
		BF(index, GPMI_CTRL0_CS)	 |
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

	init_completion(&state->nands[index].done);
	return 0;
}

static int bch_write(void *context,
		int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob)
{
	unsigned long writesize = 0;
	u32 bufmask = 0;
	struct bch_state_t *state = context;

	if (!dma_mapping_error(NULL, oob)) {
		bufmask |= BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;
		writesize += state->nands[index].oobsize;
	}
	if (!dma_mapping_error(NULL, page)) {
		bufmask |= (BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE
				& ~BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY);
		writesize += state->nands[index].writesize;
	}

	pr_debug("writesize = %ld, bufmask = 0x%X\n", writesize, bufmask);
	bch_reset(context, index);

	/* enable BCH and write NAND data */
	chain->command->cmd =
		BF(6, APBH_CHn_CMD_CMDWORDS)    |
		BM_APBH_CHn_CMD_WAIT4ENDCMD     |
		BM_APBH_CHn_CMD_NANDLOCK        |
		BM_APBH_CHn_CMD_CHAIN           |
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__WRITE, GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH       |
		BF(index, GPMI_CTRL0_CS)        |
		BF(0, GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC      |
		BF(0x03, GPMI_ECCCTRL_ECC_CMD)  |
		BF(bufmask, GPMI_ECCCTRL_BUFFER_MASK);
	chain->command->pio_words[3] = writesize;
	chain->command->pio_words[4] =
		!dma_mapping_error(NULL, page) ? page : 0;
	chain->command->pio_words[5] =
		!dma_mapping_error(NULL, oob) ? oob : 0;
	chain->command->alternate = 0;
	chain++;

	/* emit IRQ */
	chain->command->cmd =
		BF(0, APBH_CHn_CMD_CMDWORDS)    |
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_IRQONCMPLT      |
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);

	init_completion(&state->nands[index].done);

	return 0;
}

/**
 * bch_init - Initialize and register ECC.
 *
 * The GPMI driver calls this function once, at the beginning of time, whether
 * or not it decides to use this ECC.
 */
int __init bch_init(void)
{
	int err;

	/* Check if the BCH hardware is available. */

	if (!bch_available(&state.chip))
		return -ENXIO;

	/* Give the GPMI driver a descriptor. */

	gpmi_ecc_add(&state.chip);

	/* Attempt to acquire the BCH interrupt. */

	err = request_irq(IRQ_BCH, bch_irq, 0, state.chip.name, &state);
	if (err)
		return err;

	printk(KERN_INFO"%s: initialized\n", __func__);
	return 0;
}

/**
 * bch_exit - Shut down and de-register ECC.
 */
void bch_exit(void)
{
	free_irq(IRQ_BCH, &state);
	gpmi_ecc_remove(&state.chip);
}
