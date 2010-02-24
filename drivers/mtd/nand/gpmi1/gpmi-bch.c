/*
 * Freescale i.MX28 GPMI (General-Purpose-Media-Interface)
 *
 * STMP378X BCH hardware ECC engine
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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/dma-mapping.h>

#include <asm/dma.h>
#include <mach/irqs.h>
#include <mach/system.h>
#include "regs-gpmi.h"
#include "gpmi.h"

#define BCH_MAX_NANDS 8

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

/**
 * bch_reset - Resets the BCH.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 * @index:    ??
 */
static int bch_reset(void *context, int index)
{
	mxs_reset_block(IO_ADDRESS(BCH_PHYS_ADDR), true);
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN,
		IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_CTRL_SET);
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

	s0 = __raw_readl(IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_STATUS0);
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

	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ,
				IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_CTRL_CLR);

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
	mxs_reset_block(IO_ADDRESS(BCH_PHYS_ADDR), true);
	return __raw_readl(IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_BLOCKNAME) ==
								0x20484342;
}

/**
 * bch_setup - Set up BCH for use.
 *
 * The GPMI driver calls this function for every chip.
 *
 * @context:    Context data -- a pointer to a struct bch_state_t.
 * @index:      ??
 * @writesize:  The page data size.
 * @oobsize:    The page OOB size.
 */
static int bch_setup(void *context, int index, int writesize, int oobsize)
{
	struct bch_state_t *state = context;
	u32 ecc0, eccn, metasize;

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

	state->nands[index].oobsize   = oobsize;
	state->nands[index].writesize = writesize;
	state->nands[index].metasize  = metasize;
	state->nands[index].ecc0      = ecc0;
	state->nands[index].eccn      = eccn;

	/* Configure layout 0. */

	__raw_writel(
		BF_BCH_FLASH0LAYOUT0_NBLOCKS(writesize/512 - 1) |
		BF_BCH_FLASH0LAYOUT0_META_SIZE(metasize)        |
		BF_BCH_FLASH0LAYOUT0_ECC0(ecc0 >> 1)            |
		BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(512)            ,
		IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_FLASH0LAYOUT0);

	__raw_writel(
		BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(writesize + oobsize) |
		BF_BCH_FLASH0LAYOUT1_ECCN(eccn >> 1)                |
		BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(512)                ,
		IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */

	__raw_writel(0, IO_ADDRESS(BCH_PHYS_ADDR) + HW_BCH_LAYOUTSELECT);

	bch_reset(context, index);

	return 0;
}

/**
 * bch_read - Fill in a DMA chain to read a page.
 *
 * @context:  Context data -- a pointer to a struct bch_state_t.
 * @cs:       The chip number to read.
 * @chain:    The main descriptor of the DMA chain to fill.
 * @page:     Physical address of the target page data buffer.
 * @oob:      Physical address of the target OOB data buffer.
 *
 * Return: status of operation -- 0 on success
 */
static int bch_read(void *context,
		int index,
		struct mxs_dma_desc **d, unsigned channel,
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

	bch_reset(context, index);

	/* Wait for the medium to report ready. */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(
				BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY)  |
		BM_GPMI_CTRL0_LOCK_CS                                        |
		BM_GPMI_CTRL0_WORD_LENGTH                                    |
		BF_GPMI_CTRL0_CS(index)                                      |
		BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)      |
		BF_GPMI_CTRL0_XFER_COUNT(0)                                  ;

	mxs_dma_desc_append(channel, (*d));
	d++;

	/* enable BCH and read NAND data */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BM_GPMI_CTRL0_LOCK_CS                                        |
		BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__READ) |
		BM_GPMI_CTRL0_WORD_LENGTH                                    |
		BF_GPMI_CTRL0_CS(index)                                      |
		BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)      |
		BF_GPMI_CTRL0_XFER_COUNT(readsize)                           ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	                         |
		BF_GPMI_ECCCTRL_ECC_CMD(BV_GPMI_ECCCTRL_ECC_CMD__DECODE) |
		BF_GPMI_ECCCTRL_BUFFER_MASK(bufmask)                     ;
	(*d)->cmd.pio_words[3] = readsize;
	(*d)->cmd.pio_words[4] = !dma_mapping_error(NULL, page) ? page : 0;
	(*d)->cmd.pio_words[5] = !dma_mapping_error(NULL, oob)  ? oob  : 0;

	mxs_dma_desc_append(channel, (*d));
	d++;

	/* disable BCH block */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(
				BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY)  |
		BM_GPMI_CTRL0_LOCK_CS                                        |
		BM_GPMI_CTRL0_WORD_LENGTH                                    |
		BF_GPMI_CTRL0_CS(index)                                      |
		BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)      |
		BF_GPMI_CTRL0_XFER_COUNT(readsize)                           ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(channel, (*d));
	d++;

	/* and deassert nand lock */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 0;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 0;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	mxs_dma_desc_append(channel, (*d));
	d++;

	init_completion(&state->nands[index].done);

	return 0;

}

static int bch_write(void *context,
		int index,
		struct mxs_dma_desc **d, unsigned channel,
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

	bch_reset(context, index);

	/* enable BCH and write NAND data */

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 6;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BM_GPMI_CTRL0_LOCK_CS                                        |
		BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE)|
		BM_GPMI_CTRL0_WORD_LENGTH                                    |
		BF_GPMI_CTRL0_CS(index)                                      |
		BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)      |
		BF_GPMI_CTRL0_XFER_COUNT(0)                                  ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	                         |
		BF_GPMI_ECCCTRL_ECC_CMD(BV_GPMI_ECCCTRL_ECC_CMD__ENCODE) |
		BF_GPMI_ECCCTRL_BUFFER_MASK(bufmask)                     ;

	(*d)->cmd.pio_words[3] = writesize;
	(*d)->cmd.pio_words[4] = !dma_mapping_error(NULL, page) ? page : 0;
	(*d)->cmd.pio_words[5] = !dma_mapping_error(NULL, oob)  ? oob  : 0;

	mxs_dma_desc_append(channel, (*d));
	d++;

	init_completion(&state->nands[index].done);

	return 0;
}

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
