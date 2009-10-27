/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
 *
 * STMP37XX/STMP378X ECC8 hardware ECC engine
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
#include <mach/irqs.h>
#include <mach/regs-gpmi.h>
#include <mach/regs-apbx.h>
#include <mach/regs-apbh.h>
#include <mach/platform.h>
#include "gpmi.h"

#define ECC8_MAX_NANDS 4

static int ecc8_available(void *context);
static int ecc8_setup(void *context, int index, int writesize, int oobsize);
static int ecc8_read(void *context, int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob);
static int ecc8_write(void *context, int index,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob);
static int ecc8_stat(void *ctx, int index, struct mtd_ecc_stats *r);
static int ecc8_reset(void *context, int index);

struct ecc8_nand {
};

/**
 * ecc8_state_t - Describes the state of the Reed-Solomon ECC.
 *
 * @chip:       A descriptor the GPMI driver uses to track this ECC.
 * @done:       A struct completion used to manage ECC interrupts.
 * @stat:       Used by the interrupt level to communicate ECC statistics to the
 *              base level.
 * @writesize:  The page data size.
 * @oobsize:    The page OOB size.
 * @ecc_page:   The number of ECC bytes associated with each 512-byte data
 *              block.
 * @ecc_oob:    The number of ECC bytes that cover the free bytes in the OOB.
 * @oob_free:   The number of free bytes in the OOB. That is, the number of
 *              OOB bytes that are *not* ECC bytes.
 * @r:          A bit mask that sets the ECC_CMD field of the GPMI_ECCCTRL
 *              register for reading.
 * @w:          A bit mask that sets the ECC_CMD field of the GPMI_ECCCTRL
 *              register for writing.
 * @bits:       The number of 512-byte data blocks in a page. By coincidence,
 *              this is also the strength of the ECC algorithm since the
 *              hardware is constrained to use ECC-4 with 2K pages and ECC-8
 *              with 4K pages. The ECC strength may be the original source of
 *              this field's name, but the code actually uses this value in the
 *              sense of the number of 512-byte blocks.
 * @s0_mask:    This is presumably the mask that should be applied to the
 *              ECC8_STATUS0 register before checking for ECC errors. In fact,
 *              it is assigned but never used.
 * @s1_mask:    This is presumably the mask that should be applied to the
 *              ECC8_STATUS1 register before checking for ECC errors. In fact,
 *              it is assigned but never used.
 */

struct ecc8_state_t {
	struct gpmi_ecc_descriptor chip;
	struct completion done;
	struct mtd_ecc_stats stat;
	u32 writesize, oobsize;
	u32 ecc_page, ecc_oob, oob_free;
	u32 r, w;
	int bits;
	u32 s0_mask, s1_mask;
};

/* The singleton struct ecc8_state_t for the ECC8. */

static struct ecc8_state_t state = {
	.chip = {
		.name	= "ecc8",
		.setup	= ecc8_setup,
		.stat	= ecc8_stat,
		.read	= ecc8_read,
		.write	= ecc8_write,
		.reset	= ecc8_reset,
	},
};

/**
 * ecc8_reset - Resets the ECC8.
 *
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 * @index:    ??
 */
static int  ecc8_reset(void *context, int index)
{
	stmp3xxx_reset_block(REGS_ECC8_BASE, false);
	while (__raw_readl(REGS_ECC8_BASE + HW_ECC8_CTRL) & BM_ECC8_CTRL_AHBM_SFTRST)
		stmp3xxx_clearl(BM_ECC8_CTRL_AHBM_SFTRST, REGS_ECC8_BASE + HW_ECC8_CTRL);
	stmp3xxx_setl(BM_ECC8_CTRL_COMPLETE_IRQ_EN, REGS_ECC8_BASE + HW_ECC8_CTRL);
	return 0;
}

/**
 * ecc8_stat - Gather statistics and clean up after a read operation.
 *
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 * @index:    ??
 * @r:        A statistics structure that will receive the results of the most
 *            recent operation.
 */
static int ecc8_stat(void *context, int index, struct mtd_ecc_stats *r)
{
	struct ecc8_state_t *state = context;

	wait_for_completion(&state->done);

	*r = state->stat;
	state->stat.failed = 0;
	state->stat.corrected = 0;
	return 0;
}

/**
 * ecc8_irq - Interrupt handler for the ECC8 hardware.
 *
 * This function gains control when the ECC8 hardware interrupts. It
 * acknowledges the interrupt and gathers status information.
 *
 * @irq:      The interrupt number.
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 */
static irqreturn_t ecc8_irq(int irq, void *context)
{
	int r;
	struct mtd_ecc_stats ecc_stats;
	struct ecc8_state_t *state = context;
	u32 corr;
	u32 s0 = __raw_readl(REGS_ECC8_BASE + HW_ECC8_STATUS0),
	    s1 = __raw_readl(REGS_ECC8_BASE + HW_ECC8_STATUS1);

	/* Get the physical chip number to which this operation applied. */

	r = (s0 & BM_ECC8_STATUS0_COMPLETED_CE) >> 16;

	/* Check if there were any errors. */

	if (s0 & BM_ECC8_STATUS0_CORRECTED ||
	    s0 & BM_ECC8_STATUS0_UNCORRECTABLE) {

		ecc_stats.failed = 0;
		ecc_stats.corrected = 0;

		/* Check for errors in the OOB bytes. */

		s0 = (s0 & BM_ECC8_STATUS0_STATUS_AUX) >> 8;
		if (s0 <= 4)
		       ecc_stats.corrected += s0;
		if (s0 == 0xE)
		       ecc_stats.failed++;

		/* Check for errors in the data bytes. */

		for ( ; s1 != 0; s1 >>= 4) {
		       corr = s1 & 0xF;
		       if (corr == 0x0C)
			      continue;
		       if (corr == 0xE)
			     ecc_stats.failed++;
		       if (corr <= 8)
			     ecc_stats.corrected += corr;
		       s1 >>= 4;
		}

		/* Accumulate statistics. */

		state->stat.corrected += ecc_stats.corrected;
		state->stat.failed += ecc_stats.failed;

	}

	/* Acknowledge the interrupt. */

	complete(&state->done);

	stmp3xxx_clearl(BM_ECC8_CTRL_COMPLETE_IRQ, REGS_ECC8_BASE + HW_ECC8_CTRL);
	return IRQ_HANDLED;
}

/**
 * ecc8_available - Returns whether the Reed-Solomon ECC hardware is available.
 *
 * Note that this function always returns true.
 *
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 */
static int ecc8_available(void *context)
{
	return 1;
}

/**
 * ecc8_setup - Set up ECC for use.
 *
 * If the GPMI driver decides to use this ECC, it will call this function once,
 * before it starts any operations.
 *
 * @context:    Context data -- a pointer to a struct ecc8_state_t.
 * @index:      ??
 * @writesize:  The page data size.
 * @oobsize:    The page OOB size.
 */
static int ecc8_setup(void *context, int index, int writesize, int oobsize)
{
	struct ecc8_state_t *state = context;

	switch (writesize) {
	case 2048:
		state->ecc_page = 9;
		state->ecc_oob = 9;
		state->bits = 4;
		state->r = BF(BV_GPMI_ECCCTRL_ECC_CMD__DECODE_4_BIT,
				GPMI_ECCCTRL_ECC_CMD);
		state->w = BF(BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_4_BIT,
				GPMI_ECCCTRL_ECC_CMD);
		break;
	case 4096:
		state->ecc_page = 18;
		state->ecc_oob = 9;
		state->bits = 8;
		state->r = BF(BV_GPMI_ECCCTRL_ECC_CMD__DECODE_8_BIT,
				GPMI_ECCCTRL_ECC_CMD);
		state->w = BF(BV_GPMI_ECCCTRL_ECC_CMD__ENCODE_8_BIT,
				GPMI_ECCCTRL_ECC_CMD);
		break;
	default:
		return -ENOTSUPP;
	}

	state->oob_free = oobsize -
		(state->bits * state->ecc_page) - state->ecc_oob;
	state->oobsize = oobsize;
	state->writesize = writesize;

	ecc8_reset(context, index);

	return 0;
}

/**
 * ecc8_read - Fill in a DMA chain to read a page.
 *
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 * @cs:       The chip number to read.
 * @chain:    The main descriptor of the DMA chain to fill.
 * @error:    ??
 * @page:     Physical address of the target page data buffer.
 * @oob:      Physical address of the target OOB data buffer.
 *
 * Return: status of operation -- 0 on success
 */
static int ecc8_read(void *context, int cs,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob)
{
	unsigned long readsize = 0;
	u32 bufmask = 0;
	struct ecc8_state_t *state = context;

	ecc8_reset(context, cs);

	state->s0_mask = state->s1_mask = 0;

	if (!dma_mapping_error(NULL, page)) {
		bufmask |= (1 << state->bits) - 1;
		readsize += (state->bits * state->ecc_page);
		readsize += state->writesize;
	}
	if (!dma_mapping_error(NULL, oob)) {
		bufmask |= BV_GPMI_ECCCTRL_BUFFER_MASK__AUXILIARY;
		readsize += state->oob_free + state->ecc_oob;
		state->s0_mask = BM_ECC8_STATUS0_STATUS_AUX;
		if (dma_mapping_error(NULL, page))
			page = oob;
	}

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
		BF(cs, GPMI_CTRL0_CS);
	chain->command->buf_ptr = 0;
	chain++;

	/* enable ECC and read NAND data */
	chain->command->cmd =
		BF(6, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_NANDLOCK	|
		BM_APBH_CHn_CMD_CHAIN 		|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__READ, GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH	|
		BF(cs, GPMI_CTRL0_CS)		|
		BF(readsize, GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	|
		state->r			|
		BF(bufmask, GPMI_ECCCTRL_BUFFER_MASK);
	chain->command->pio_words[3] = readsize;
	chain->command->pio_words[4] = !dma_mapping_error(NULL, page) ? page : 0;
	chain->command->pio_words[5] = !dma_mapping_error(NULL, oob) ? oob : 0;
	chain->command->buf_ptr = 0;
	chain++;

	/* disable ECC block */
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
		BF(cs, GPMI_CTRL0_CS);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->buf_ptr = 0;
	chain++;

	/* and deassert nand lock */
	chain->command->cmd =
		BF(0, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_IRQONCMPLT	|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->buf_ptr = 0;

	init_completion(&state->done);

	return 0;
}

/**
 * ecc8_write - Fill in a DMA chain to write a page.
 *
 * @context:  Context data -- a pointer to a struct ecc8_state_t.
 * @cs:       The chip number to read.
 * @chain:    The main descriptor of the DMA chain to fill.
 * @error:    ??
 * @page:     Physical address of the source page data buffer.
 * @oob:      Physical address of the source OOB data buffer.
 *
 * Return: status of operation -- 0 on success
 */
static int ecc8_write(void *context, int cs,
		struct stmp3xxx_dma_descriptor *chain,
		dma_addr_t error,
		dma_addr_t page, dma_addr_t oob)
{
	u32 bufmask = 0;
	struct ecc8_state_t *state = context;
	u32 data_w, data_w_ecc,
	    oob_w, oob_w_ecc;

	ecc8_reset(context, cs);

	data_w = data_w_ecc = oob_w = oob_w_ecc = 0;

	if (!dma_mapping_error(NULL, oob)) {
		bufmask |= BV_GPMI_ECCCTRL_BUFFER_MASK__AUXILIARY;
		oob_w = state->oob_free;
		oob_w_ecc = oob_w + state->ecc_oob;
	}
	if (!dma_mapping_error(NULL, page)) {
		bufmask |= (1 << state->bits) - 1;
		data_w = state->bits * 512;
		data_w_ecc = data_w + state->bits * state->ecc_page;
	}

	/* enable ECC and send NAND data (page only) */
	chain->command->cmd =
		BF(data_w ? data_w : oob_w, APBH_CHn_CMD_XFER_COUNT) |
		BF(4, APBH_CHn_CMD_CMDWORDS)	|
		BM_APBH_CHn_CMD_NANDLOCK	|
		BM_APBH_CHn_CMD_CHAIN 		|
		BF(BV_APBH_CHn_CMD_COMMAND__DMA_READ, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
		BF(BV_GPMI_CTRL0_COMMAND_MODE__WRITE, GPMI_CTRL0_COMMAND_MODE) |
		BM_GPMI_CTRL0_WORD_LENGTH	|
		BM_GPMI_CTRL0_LOCK_CS		|
		BF(cs, GPMI_CTRL0_CS)		|
		BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA, GPMI_CTRL0_ADDRESS) |
		BF(data_w + oob_w, GPMI_CTRL0_XFER_COUNT);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	|
		state->w				|
		BF(bufmask, GPMI_ECCCTRL_BUFFER_MASK);
	chain->command->pio_words[3] =
		data_w_ecc + oob_w_ecc;
	if (!dma_mapping_error(NULL, page))
		chain->command->buf_ptr = page;
	else
		chain->command->buf_ptr = oob;
	chain++;

	if (!dma_mapping_error(NULL, page) && !dma_mapping_error(NULL, oob)) {
		/* send NAND data (OOB only) */
		chain->command->cmd =
			BM_APBH_CHn_CMD_CHAIN |
			BF(oob_w, APBH_CHn_CMD_XFER_COUNT) |
			BF(0, APBH_CHn_CMD_CMDWORDS)	|
			BM_APBH_CHn_CMD_WAIT4ENDCMD	|
			BM_APBH_CHn_CMD_NANDLOCK	|
			BF(BV_APBH_CHn_CMD_COMMAND__DMA_READ,
				APBH_CHn_CMD_COMMAND);
		chain->command->buf_ptr = oob;	/* never dma_mapping_error() */
		chain++;
	}
	/* emit IRQ */
	chain->command->cmd =
		BF(0, APBH_CHn_CMD_CMDWORDS)	|
		BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER,
			APBH_CHn_CMD_COMMAND) |
		BM_APBH_CHn_CMD_WAIT4ENDCMD	|
		BM_APBH_CHn_CMD_IRQONCMPLT;

	return 0;
}

/**
 * ecc8_init - Initialize and register ECC.
 *
 * The GPMI driver calls this function once, at the beginning of time, whether
 * or not it decides to use this ECC.
 */
int __init ecc8_init(void)
{
	int err;

	/* Check if the ECC8 hardware is available. */

	if (!ecc8_available(&state))
		return -ENXIO;

	/* Give the GPMI driver a descriptor. */

	gpmi_ecc_add(&state.chip);

	/* Attempt to acquire the ECC interrupt. */

	err = request_irq(IRQ_ECC8_IRQ, ecc8_irq, 0, state.chip.name, &state);
	if (err)
		return err;

	printk(KERN_INFO"%s: initialized\n", __func__);
	return 0;
}

/**
 * ecc8_exit - Shut down and de-register ECC.
 */
void ecc8_exit(void)
{

	/* Relinquish the ECC interrupt. */

	free_irq(IRQ_ECC8_IRQ, &state);

	/* Remove the descriptor for this ECC from the GPMI's list. */

	gpmi_ecc_remove(&state.chip);
}
