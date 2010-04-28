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

/**
 * gpmi_nfc_bch_isr - BCH interrupt service routine.
 *
 * @interrupt_number:  The interrupt number.
 * @cookie:            A cookie that contains a pointer to the owning device
 *                     data structure.
 */
irqreturn_t gpmi_nfc_bch_isr(int irq, void *cookie)
{
	struct gpmi_nfc_data  *this      = cookie;
	struct nfc_hal        *nfc       =  this->nfc;

	gpmi_nfc_add_event("> gpmi_nfc_bch_isr", 1);

	/* Clear the interrupt. */

	nfc->clear_bch(this);

	/* Release the base level. */

	complete(&(nfc->bch_done));

	/* Return success. */

	gpmi_nfc_add_event("< gpmi_nfc_bch_isr", -1);

	return IRQ_HANDLED;

}

/**
 * gpmi_nfc_dma_isr - DMA interrupt service routine.
 *
 * @interrupt_number:  The interrupt number.
 * @cookie:            A cookie that contains a pointer to the owning device
 *                     data structure.
 */
irqreturn_t gpmi_nfc_dma_isr(int irq, void *cookie)
{
	struct gpmi_nfc_data  *this = cookie;
	struct nfc_hal        *nfc  =  this->nfc;

	gpmi_nfc_add_event("> gpmi_nfc_dma_isr", 1);

	/* Acknowledge the DMA channel's interrupt. */

	mxs_dma_ack_irq(nfc->isr_dma_channel);

	/* Release the base level. */

	complete(&(nfc->dma_done));

	/* Return success. */

	gpmi_nfc_add_event("< gpmi_nfc_dma_isr", -1);

	return IRQ_HANDLED;

}

/**
 * gpmi_nfc_dma_init() - Initializes DMA.
 *
 * @this:  Per-device data.
 */
int gpmi_nfc_dma_init(struct gpmi_nfc_data *this)
{
	struct device   *dev = this->dev;
	struct nfc_hal  *nfc = this->nfc;
	int             i;
	int             error;

	/* Allocate the DMA descriptors. */

	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; i++) {
		nfc->dma_descriptors[i] = mxs_dma_alloc_desc();
		if (!nfc->dma_descriptors[i]) {
			dev_err(dev, "Cannot allocate all DMA descriptors.\n");
			error = -ENOMEM;
			goto exit_descriptor_allocation;
		}
	}

	/* If control arrives here, all is well. */

	return 0;

	/* Control arrives here when something has gone wrong. */

exit_descriptor_allocation:
	while (--i >= 0)
		mxs_dma_free_desc(this->nfc->dma_descriptors[i]);

	return error;

}

/**
 * gpmi_nfc_dma_exit() - Shuts down DMA.
 *
 * @this:  Per-device data.
 */
void gpmi_nfc_dma_exit(struct gpmi_nfc_data *this)
{
	struct nfc_hal  *nfc = this->nfc;
	int             i;

	/* Free the DMA descriptors. */

	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; i++)
		mxs_dma_free_desc(nfc->dma_descriptors[i]);

}

/**
 * gpmi_nfc_set_geometry() - Shared NFC geometry configuration.
 *
 * In principle, computing the NFC geometry is version-specific. However, at
 * this writing all, versions share the same page model, so this code can also
 * be shared.
 *
 * @this:  Per-device data.
 */
int gpmi_nfc_set_geometry(struct gpmi_nfc_data *this)
{
	struct device             *dev      = this->dev;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *geometry = &this->nfc_geometry;
	struct boot_rom_helper    *rom      =  this->rom;
	unsigned int              metadata_size;
	unsigned int              status_size;
	unsigned int              chunk_data_size_in_bits;
	unsigned int              chunk_ecc_size_in_bits;
	unsigned int              chunk_total_size_in_bits;
	unsigned int              block_mark_chunk_number;
	unsigned int              block_mark_chunk_bit_offset;
	unsigned int              block_mark_bit_offset;

	/* At this writing, we support only BCH. */

	geometry->ecc_algorithm = "BCH";

	/*
	 * We always choose a metadata size of 10. Don't try to make sense of
	 * it -- this is really only for historical compatibility.
	 */

	geometry->metadata_size_in_bytes = 10;

	/*
	 * At this writing, we always use 512-byte ECC chunks. Later hardware
	 * will be able to support larger chunks, which will cause this
	 * decision to move into version-specific code.
	 */

	geometry->ecc_chunk_size_in_bytes = 512;

	/* Compute the page size based on the physical geometry. */

	geometry->page_size_in_bytes =
			physical->page_data_size_in_bytes +
			physical->page_oob_size_in_bytes  ;

	/*
	 * Compute the total number of ECC chunks in a page. This includes the
	 * slightly larger chunk at the beginning of the page, which contains
	 * both data and metadata.
	 */

	geometry->ecc_chunk_count =
			  physical->page_data_size_in_bytes /
			/*---------------------------------*/
			  geometry->ecc_chunk_size_in_bytes;

	/*
	 * We use the same ECC strength for all chunks, including the first one.
	 * At this writing, we base our ECC strength choice entirely on the
	 * the physical page geometry. In the future, this should be changed to
	 * pay attention to the detailed device information we gathered earlier.
	 */

	geometry->ecc_strength = 0;

	switch (physical->page_data_size_in_bytes) {
	case 2048:
		geometry->ecc_strength = 8;
		break;
	case 4096:
		switch (physical->page_oob_size_in_bytes) {
		case 128:
			geometry->ecc_strength = 8;
			break;
		case 218:
			geometry->ecc_strength = 16;
			break;
		}
		break;
	}

	/* Check if we were able to figure out the ECC strength. */

	if (!geometry->ecc_strength) {
		dev_err(dev, "Unsupported page geometry: %u:%u\n",
			physical->page_data_size_in_bytes,
			physical->page_oob_size_in_bytes);
		return !0;
	}

	/*
	 * The payload buffer contains the data area of a page. The ECC engine
	 * only needs what's required to hold the data.
	 */

	geometry->payload_size_in_bytes = physical->page_data_size_in_bytes;

	/*
	 * In principle, computing the auxiliary buffer geometry is NFC
	 * version-specific. However, at this writing, all versions share the
	 * same model, so this code can also be shared.
	 *
	 * The auxiliary buffer contains the metadata and the ECC status. The
	 * metadata is padded to the nearest 32-bit boundary. The ECC status
	 * contains one byte for every ECC chunk, and is also padded to the
	 * nearest 32-bit boundary.
	 */

	metadata_size = (geometry->metadata_size_in_bytes + 0x3) & ~0x3;
	status_size   = (geometry->ecc_chunk_count        + 0x3) & ~0x3;

	geometry->auxiliary_size_in_bytes = metadata_size + status_size;
	geometry->auxiliary_status_offset = metadata_size;

	/* Check if we're going to do block mark swapping. */

	if (!rom->swap_block_mark)
		return 0;

	/*
	 * If control arrives here, we're doing block mark swapping, so we need
	 * to compute the byte and bit offsets of the physical block mark within
	 * the ECC-based view of the page data. In principle, this isn't a
	 * difficult computation -- but it's very important and it's easy to get
	 * it wrong, so we do it carefully.
	 *
	 * Note that this calculation is simpler because we use the same ECC
	 * strength for all chunks, including the zero'th one, which contains
	 * the metadata. The calculation would be slightly more complicated
	 * otherwise.
	 *
	 * We start by computing the physical bit offset of the block mark. We
	 * then subtract the number of metadata and ECC bits appearing before
	 * the mark to arrive at its bit offset within the data alone.
	 */

	/* Compute some important facts about chunk geometry. */

	chunk_data_size_in_bits = geometry->ecc_chunk_size_in_bytes * 8;
	chunk_ecc_size_in_bits  = geometry->ecc_strength * 13;

	chunk_total_size_in_bits =
			chunk_data_size_in_bits + chunk_ecc_size_in_bits;

	/* Compute the bit offset of the block mark within the physical page. */

	block_mark_bit_offset = physical->page_data_size_in_bytes * 8;

	/* Subtract the metadata bits. */

	block_mark_bit_offset -= geometry->metadata_size_in_bytes * 8;

	/*
	 * Compute the chunk number (starting at zero) in which the block mark
	 * appears.
	 */

	block_mark_chunk_number =
			block_mark_bit_offset / chunk_total_size_in_bits;

	/*
	 * Compute the bit offset of the block mark within its chunk, and
	 * validate it.
	 */

	block_mark_chunk_bit_offset =
		block_mark_bit_offset -
			(block_mark_chunk_number * chunk_total_size_in_bits);

	if (block_mark_chunk_bit_offset > chunk_data_size_in_bits) {

		/*
		 * If control arrives here, the block mark actually appears in
		 * the ECC bits of this chunk. This wont' work.
		 */

		dev_err(dev, "Unsupported page geometry "
					"(block mark in ECC): %u:%u\n",
					physical->page_data_size_in_bytes,
					physical->page_oob_size_in_bytes);
		return !0;

	}

	/*
	 * Now that we know the chunk number in which the block mark appears,
	 * we can subtract all the ECC bits that appear before it.
	 */

	block_mark_bit_offset -=
			block_mark_chunk_number * chunk_ecc_size_in_bits;

	/*
	 * We now know the absolute bit offset of the block mark within the
	 * ECC-based data. We can now compute the byte offset and the bit
	 * offset within the byte.
	 */

	geometry->block_mark_byte_offset = block_mark_bit_offset / 8;
	geometry->block_mark_bit_offset  = block_mark_bit_offset % 8;

	/* Return success. */

	return 0;

}

/*
 * This code is useful for debugging.
 */

/*#define DUMP_DMA_CONTEXT*/

#if (defined DUMP_DMA_CONTEXT)

int dump_dma_context_flag;

void dump_dma_context(struct gpmi_nfc_data *this, char *title)
{

	struct resources     *resources = &this->resources;
	struct nfc_hal       *nfc       =  this->nfc;
	struct mxs_dma_desc  **d        = nfc->dma_descriptors;
	void                 *q;
	uint32_t             *p;
	unsigned int         i;
	unsigned int         j;

	if (!dump_dma_context_flag)
		return;

	pr_info("%s\n", title);
	pr_info("======\n");
	pr_info("\n");

	/*--------------------------------------------------------------------*/

	pr_info("  Descriptors\n");
	pr_info("  -----------\n");
	{

	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; i++, d++) {
		pr_info("    #%u\n", i);
		pr_info("    --\n");
		pr_info("    Physical Address: 0x%08x\n" , (*d)->address);
		pr_info("    Next            : 0x%08lx\n", (*d)->cmd.next);
		pr_info("    Command         : 0x%08lx\n", (*d)->cmd.cmd.data);
		pr_info("    Buffer          : 0x%08x\n" , (*d)->cmd.address);
		for (j = 0; j < 6; j++)
			pr_info("    PIO[%u]          : 0x%08lx\n",
						j, (*d)->cmd.pio_words[j]);
	}

	}
	pr_info("\n");

	/*--------------------------------------------------------------------*/

	pr_info("  DMA\n");
	pr_info("  ---\n");
	{
	void  *DMA = IO_ADDRESS(APBH_DMA_PHYS_ADDR);

	p = q = DMA + 0x200;

	for (i = 0; i < 7; i++) {
		pr_info("    [0x%03x] 0x%08x\n", q - DMA, *p);
		q += 0x10;
		p = q;
	}

	}
	pr_info("\n");

	/*--------------------------------------------------------------------*/

	pr_info("  GPMI\n");
	pr_info("  ----\n");
	{
	void  *GPMI = resources->gpmi_regs;

	p = q = GPMI;

	for (i = 0; i < 33; i++) {
		pr_info("    [0x%03x] 0x%08x\n", q - GPMI, *p);
		q += 0x10;
		p = q;
	}

	}
	pr_info("\n");

	/*--------------------------------------------------------------------*/

	pr_info("  BCH\n");
	pr_info("  ---\n");
	{
	void  *BCH = resources->bch_regs;

	p = q = BCH;

	for (i = 0; i < 22; i++) {
		pr_info("    [0x%03x] 0x%08x\n", q - BCH, *p);
		q += 0x10;
		p = q;
	}

	}
	pr_info("\n");

}

#endif

/**
 * gpmi_nfc_dma_go - Run a DMA channel.
 *
 * @this:         Per-device data structure.
 * @dma_channel:  The DMA channel we're going to use.
 */
int gpmi_nfc_dma_go(struct gpmi_nfc_data *this, int  dma_channel)
{
	struct device     *dev       =  this->dev;
	struct resources  *resources = &this->resources;
	struct nfc_hal    *nfc       =  this->nfc;
	unsigned long     timeout;
	int               error;
	LIST_HEAD(tmp_desc_list);

	gpmi_nfc_add_event("> gpmi_nfc_dma_go", 1);

	/* Get ready... */

	nfc->isr_dma_channel = dma_channel;
	init_completion(&nfc->dma_done);
	mxs_dma_enable_irq(dma_channel, 1);

	/* Go! */

	#if defined(DUMP_DMA_CONTEXT)
		dump_dma_context(this, "BEFORE");
	#endif

	mxs_dma_enable(dma_channel);

	/* Wait for it to finish. */

	timeout = wait_for_completion_timeout(&nfc->dma_done,
							msecs_to_jiffies(1000));

	#if defined(DUMP_DMA_CONTEXT)
		dump_dma_context(this, "AFTER");
	#endif

	error = (!timeout) ? -ETIMEDOUT : 0;

	if (error) {
		dev_err(dev, "[%s] Chip: %u, DMA Channel: %d, Error %d\n",
			__func__, dma_channel - resources->dma_low_channel,
			dma_channel, error);
		gpmi_nfc_add_event("...DMA timed out", 0);
	} else
		gpmi_nfc_add_event("...Finished DMA successfully", 0);

	/* Clear out the descriptors we just ran. */

	mxs_dma_cooked(dma_channel, &tmp_desc_list);

	/* Shut the DMA channel down. */

	mxs_dma_reset(dma_channel);
	mxs_dma_enable_irq(dma_channel, 0);
	mxs_dma_disable(dma_channel);

	/* Return. */

	gpmi_nfc_add_event("< gpmi_nfc_dma_go", -1);

	return error;

}
