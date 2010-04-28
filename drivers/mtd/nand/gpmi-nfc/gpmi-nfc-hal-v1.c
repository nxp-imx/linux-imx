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

#include "gpmi-nfc-gpmi-regs-v1.h"
#include "gpmi-nfc-bch-regs-v1.h"

/**
 * init() - Initializes the NFC hardware.
 *
 * @this:  Per-device data.
 */
static int init(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;
	int               error;

	/* Initialize DMA. */

	error = gpmi_nfc_dma_init(this);

	if (error)
		return error;

	/* Enable the clock. */

	clk_enable(resources->clock);

	/* Reset the GPMI block. */

	mxs_reset_block(resources->gpmi_regs + HW_GPMI_CTRL0, true);

	/* Choose NAND mode. */
	__raw_writel(BM_GPMI_CTRL1_GPMI_MODE,
				resources->gpmi_regs + HW_GPMI_CTRL1_CLR);

	/* Set the IRQ polarity. */
	__raw_writel(BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY,
				resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* Disable write protection. */
	__raw_writel(BM_GPMI_CTRL1_DEV_RESET,
				resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* Select BCH ECC. */
	__raw_writel(BM_GPMI_CTRL1_BCH_MODE,
				resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* Disable the clock. */

	clk_disable(resources->clock);

	/* If control arrives here, all is well. */

	return 0;

}

/**
 * set_geometry() - Configures the NFC geometry.
 *
 * @this:  Per-device data.
 */
static int set_geometry(struct gpmi_nfc_data *this)
{
	struct resources     *resources = &this->resources;
	struct nfc_geometry  *nfc       = &this->nfc_geometry;
	unsigned int         block_count;
	unsigned int         block_size;
	unsigned int         metadata_size;
	unsigned int         ecc_strength;
	unsigned int         page_size;

	/* We make the abstract choices in a common function. */

	if (gpmi_nfc_set_geometry(this))
		return !0;

	/* Translate the abstract choices into register fields. */

	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes;
	metadata_size = nfc->metadata_size_in_bytes;
	ecc_strength  = nfc->ecc_strength >> 1;
	page_size     = nfc->page_size_in_bytes;

	/* Enable the clock. */

	clk_enable(resources->clock);

	/*
	 * Reset the BCH block. Notice that we pass in true for the just_enable
	 * flag. This is because the soft reset for the version 0 BCH block
	 * doesn't work and the version 1 BCH block is similar enough that we
	 * suspect the same (though this has not been officially tested). If you
	 * try to soft reset a version 0 BCH block, it becomes unusable until
	 * the next hard reset.
	 */

	mxs_reset_block(resources->bch_regs, true);

	/* Configure layout 0. */

	__raw_writel(
		BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)     |
		BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size) |
		BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)       |
		BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size)   ,
		resources->bch_regs + HW_BCH_FLASH0LAYOUT0);

	__raw_writel(
		BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)   |
		BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)     |
		BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size) ,
		resources->bch_regs + HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */

	__raw_writel(0, resources->bch_regs + HW_BCH_LAYOUTSELECT);

	/* Enable interrupts. */

	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN,
				resources->bch_regs + HW_BCH_CTRL_SET);

	/* Disable the clock. */

	clk_disable(resources->clock);

	/* Return success. */

	return 0;

}

/**
 * set_set_timing() - Configures the NFC timing.
 *
 * @this:    Per-device data.
 * @timing:  The timing of interest.
 */
static int set_timing(struct gpmi_nfc_data *this,
					const struct gpmi_nfc_timing *timing)
{
	struct nfc_hal  *nfc = this->nfc;

	/* Accept the new timing. */

	nfc->timing = *timing;

	/* Return success. */

	return 0;

}

/**
 * exit() - Shuts down the NFC hardware.
 *
 * @this:  Per-device data.
 */
static void exit(struct gpmi_nfc_data *this)
{
	gpmi_nfc_dma_exit(this);
}

/**
 * begin() - Begin NFC I/O.
 *
 * @this:  Per-device data.
 */
static void begin(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	/* Enable the clock. */

	clk_enable(resources->clock);

}

/**
 * end() - End NFC I/O.
 *
 * @this:  Per-device data.
 */
static void end(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	/* Disable the clock. */

	clk_disable(resources->clock);

}

/**
 * clear_bch() - Clears a BCH interrupt.
 *
 * @this:  Per-device data.
 */
static void clear_bch(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ,
				resources->bch_regs + HW_BCH_CTRL_CLR);

}

/**
 * is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int is_ready(struct gpmi_nfc_data *this, unsigned chip)
{
	struct resources  *resources = &this->resources;
	uint32_t          mask;
	uint32_t          register_image;

	/* Extract and return the status. */

	mask = BF_GPMI_STAT_READY_BUSY(1 << chip);

	register_image = __raw_readl(resources->gpmi_regs + HW_GPMI_STAT);

	return !!(register_image & mask);

}

/**
 * send_command() - Sends a command and associated addresses.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the command bytes.
 * @length:  The number of bytes in the buffer.
 */
static int send_command(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct device        *dev       =  this->dev;
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that sends out the command. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_CLE;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_READ;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 3;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BM_GPMI_CTRL0_LOCK_CS                    |
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = gpmi_nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * send_data() - Sends data to the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the data.
 * @length:  The number of bytes in the buffer.
 */
static int send_data(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct device        *dev       =  this->dev;
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that writes a buffer out. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_READ;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BM_GPMI_CTRL0_LOCK_CS                    |
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = gpmi_nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * read_data() - Receives data from the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that will receive the data.
 * @length:  The number of bytes to read.
 */
static int read_data(struct gpmi_nfc_data *this, unsigned chip,
					dma_addr_t buffer, unsigned int length)
{
	struct device        *dev       =  this->dev;
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that reads the data. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = DMA_WRITE;
	(*d)->cmd.cmd.bits.chain             = 1;
	(*d)->cmd.cmd.bits.irq               = 0;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 0;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 1;
	(*d)->cmd.cmd.bits.bytes             = length;

	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BM_GPMI_CTRL0_LOCK_CS                    |
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/*
	 * A DMA descriptor that waits for the command to end and the chip to
	 * become ready.
	 *
	 * I think we actually should *not* be waiting for the chip to become
	 * ready because, after all, we don't care. I think the original code
	 * did that and no one has re-thought it yet.
	 */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	(*d)->cmd.cmd.data                   = 0;
	(*d)->cmd.cmd.bits.command           = NO_DMA_XFER;
	(*d)->cmd.cmd.bits.chain             = 0;
	(*d)->cmd.cmd.bits.irq               = 1;
	(*d)->cmd.cmd.bits.nand_lock         = 0;
	(*d)->cmd.cmd.bits.nand_wait_4_ready = 1;
	(*d)->cmd.cmd.bits.dec_sem           = 1;
	(*d)->cmd.cmd.bits.wait4end          = 1;
	(*d)->cmd.cmd.bits.halt_on_terminate = 0;
	(*d)->cmd.cmd.bits.terminate_flush   = 0;
	(*d)->cmd.cmd.bits.pio_words         = 4;
	(*d)->cmd.cmd.bits.bytes             = 0;

	(*d)->cmd.address = 0;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_LOCK_CS                    |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = gpmi_nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * send_page() - Sends a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int send_page(struct gpmi_nfc_data *this, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct device        *dev       =  this->dev;
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct nfc_geometry  *nfc_geo   = &this->nfc_geometry;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;
	uint32_t             ecc_command;
	uint32_t             buffer_mask;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that does an ECC page read. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__ENCODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		BM_GPMI_CTRL0_LOCK_CS                    |
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC               |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Prepare to receive an interrupt from the BCH block. */

	init_completion(&nfc->bch_done);

	/* Go! */

	error = gpmi_nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/**
 * read_page() - Reads a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int read_page(struct gpmi_nfc_data *this, unsigned chip,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct device        *dev       =  this->dev;
	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct nfc_geometry  *nfc_geo   = &this->nfc_geometry;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	int                  dma_channel;
	int                  error = 0;
	uint32_t             command_mode;
	uint32_t             address;
	uint32_t             ecc_command;
	uint32_t             buffer_mask;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* Wait for the chip to report ready. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_LOCK_CS                    |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Enable the BCH block and read. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__DECODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		BM_GPMI_CTRL0_LOCK_CS                                 |
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	            |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;
	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Disable the BCH block */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BM_GPMI_CTRL0_LOCK_CS                                 |
		BM_GPMI_CTRL0_WORD_LENGTH                             |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Deassert the NAND lock and interrupt. */

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

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Prepare to receive an interrupt from the BCH block. */

	init_completion(&nfc->bch_done);

	/* Go! */

	error = gpmi_nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/* This structure represents the NFC HAL for this version of the hardware. */

struct nfc_hal  gpmi_nfc_hal_v1 = {
	.version                      = 1,
	.description                  = "8-chip GPMI and BCH",
	.max_chip_count               = 8,
	.max_data_setup_cycles        = (BM_GPMI_TIMING0_DATA_SETUP >>
						BP_GPMI_TIMING0_DATA_SETUP),
	.max_data_sample_delay_cycles = (BM_GPMI_CTRL1_RDN_DELAY >>
						BP_GPMI_CTRL1_RDN_DELAY),
	.max_dll_clock_period_in_ns   = 32,
	.init                         = init,
	.set_geometry                 = set_geometry,
	.set_timing                   = set_timing,
	.exit                         = exit,
	.begin                        = begin,
	.end                          = end,
	.clear_bch                    = clear_bch,
	.is_ready                     = is_ready,
	.send_command                 = send_command,
	.send_data                    = send_data,
	.read_data                    = read_data,
	.send_page                    = send_page,
	.read_page                    = read_page,
};
