/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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

#include <linux/slab.h>
#include "gpmi-nfc.h"
#include "linux/slab.h"

/*
 * Indicates the driver should register the MTD that represents the entire
 * medium, thus making it visible.
 */

static int register_main_mtd;
module_param(register_main_mtd, int, 0400);

/*
 * Indicates the driver should attempt to perform DMA directly to/from buffers
 * passed into this driver. This is true by default. If false, the driver will
 * *always* copy incoming/outgoing data to/from its own DMA buffers.
 */

static int map_io_buffers = true;
module_param(map_io_buffers, int, 0600);

/* add our owner bbt descriptor */
static uint8_t scan_ff_pattern[] = { 0xff };
static struct nand_bbt_descr gpmi_bbt_descr = {
	.options	= 0,
	.offs		= 0,
	.len		= 1,
	.pattern	= scan_ff_pattern
};

/**
 * mil_outgoing_buffer_dma_begin() - Begins DMA on an outgoing buffer.
 *
 * @this:      Per-device data.
 * @source:    The source buffer.
 * @length:    The length of the data in the source buffer.
 * @alt_virt:  The virtual address of an alternate buffer which is ready to be
 *             used for DMA.
 * @alt_phys:  The physical address of an alternate buffer which is ready to be
 *             used for DMA.
 * @alt_size:  The size of the alternate buffer.
 * @use_virt:  A pointer to a variable that will receive the virtual address to
 *             use.
 * @use_phys:  A pointer to a variable that will receive the physical address to
 *             use.
 */
static int mil_outgoing_buffer_dma_begin(struct gpmi_nfc_data *this,
			const void *source, unsigned length,
			void *alt_virt, dma_addr_t alt_phys, unsigned alt_size,
			const void **use_virt, dma_addr_t *use_phys)
{
	struct device  *dev = this->dev;
	dma_addr_t     source_phys = ~0;

	/*
	 * If we can, we want to use the caller's buffer directly for DMA. Check
	 * if the system will let us map them.
	 */
	if (map_io_buffers && virt_addr_valid(source))
		source_phys =
			dma_map_single(dev,
				(void *) source, length, DMA_TO_DEVICE);

	if (dma_mapping_error(dev, source_phys)) {
		/*
		 * If control arrives here, we're not mapping the source buffer.
		 * Make sure the alternate is large enough.
		 */
		if (alt_size < length) {
			dev_err(dev, "Alternate buffer is too small "
							"for outgoing I/O\n");
			return -ENOMEM;
		}

		/*
		 * Copy the contents of the source buffer into the alternate
		 * buffer and set up the return values accordingly.
		 */
		memcpy(alt_virt, source, length);

		*use_virt = alt_virt;
		*use_phys = alt_phys;
	} else {
		/*
		 * If control arrives here, we're mapping the source buffer. Set
		 * up the return values accordingly.
		 */
		*use_virt = source;
		*use_phys = source_phys;
	}
	return 0;
}

/**
 * mil_outgoing_buffer_dma_end() - Ends DMA on an outgoing buffer.
 *
 * @this:       Per-device data.
 * @source:     The source buffer.
 * @length:     The length of the data in the source buffer.
 * @alt_virt:   The virtual address of an alternate buffer which was ready to be
 *              used for DMA.
 * @alt_phys:   The physical address of an alternate buffer which was ready to
 *              be used for DMA.
 * @alt_size:   The size of the alternate buffer.
 * @used_virt:  The virtual address that was used.
 * @used_phys:  The physical address that was used.
 */
static void mil_outgoing_buffer_dma_end(struct gpmi_nfc_data *this,
		const void *source, unsigned length,
		void *alt_virt, dma_addr_t alt_phys, unsigned alt_size,
		const void *used_virt, dma_addr_t used_phys)
{
	struct device  *dev = this->dev;

	/*
	 * Check if we used the source buffer, and it's not one of our own DMA
	 * buffers. If so, we need to unmap it.
	 */
	if (used_virt == source)
		dma_unmap_single(dev, used_phys, length, DMA_TO_DEVICE);
}

/**
 * mil_incoming_buffer_dma_begin() - Begins DMA on an incoming buffer.
 *
 * @this:         Per-device data.
 * @destination:  The destination buffer.
 * @length:       The length of the data that will arrive.
 * @alt_virt:     The virtual address of an alternate buffer which is ready
 *                to be used for DMA.
 * @alt_phys:     The physical address of an alternate buffer which is ready
 *                to be used for DMA.
 * @alt_size:     The size of the alternate buffer.
 * @use_virt:     A pointer to a variable that will receive the virtual address
 *                to use.
 * @use_phys:     A pointer to a variable that will receive the physical address
 *                to use.
 */
static int mil_incoming_buffer_dma_begin(struct gpmi_nfc_data *this,
			void *destination, unsigned length,
			void *alt_virt, dma_addr_t alt_phys, unsigned alt_size,
			void **use_virt, dma_addr_t *use_phys)
{
	struct device  *dev = this->dev;
	dma_addr_t     destination_phys = ~0;

	/*
	 * If we can, we want to use the caller's buffer directly for DMA. Check
	 * if the system will let us map them.
	 */
	if (map_io_buffers && virt_addr_valid(destination) &&
				!((int)destination & 0x3) && 0)
		destination_phys =
			dma_map_single(dev,
				(void *) destination, length, DMA_FROM_DEVICE);

	if (dma_mapping_error(dev, destination_phys)) {
		/*
		 * If control arrives here, we're not mapping the destination
		 * buffer. Make sure the alternate is large enough.
		 */
		if (alt_size < length) {
			dev_err(dev, "Alternate buffer is too small "
							"for incoming I/O\n");
			return -ENOMEM;
		}

		/* Set up the return values to use the alternate. */
		*use_virt = alt_virt;
		*use_phys = alt_phys;
	} else {
		/*
		 * If control arrives here, we're mapping the destination
		 * buffer. Set up the return values accordingly.
		 */
		*use_virt = destination;
		*use_phys = destination_phys;
	}
	return 0;
}

/**
 * mil_incoming_buffer_dma_end() - Ends DMA on an incoming buffer.
 *
 * @this:         Per-device data.
 * @destination:  The destination buffer.
 * @length:       The length of the data that arrived.
 * @alt_virt:     The virtual address of an alternate buffer which was ready to
 *                be used for DMA.
 * @alt_phys:     The physical address of an alternate buffer which was ready to
 *                be used for DMA.
 * @alt_size:     The size of the alternate buffer.
 * @used_virt:    The virtual address that was used.
 * @used_phys:    The physical address that was used.
 */
static void mil_incoming_buffer_dma_end(struct gpmi_nfc_data *this,
			void *destination, unsigned length,
			void *alt_virt, dma_addr_t alt_phys, unsigned alt_size,
			void *used_virt, dma_addr_t used_phys)
{
	struct device  *dev = this->dev;

	/*
	 * Check if we used the destination buffer, and it's not one of our own
	 * DMA buffers. If so, we need to unmap it.
	 */
	if (used_virt == destination)
		dma_unmap_single(dev, used_phys, length, DMA_FROM_DEVICE);
	else
		memcpy(destination, alt_virt, length);
}

/* Free all the DMA buffers */
static void mil_free_dma_buffer(struct gpmi_nfc_data *this)
{
	struct device        *dev      =  this->dev;
	struct mil           *mil      = &this->mil;

	/* Free the DMA buffer, if it has been allocated. */
	if (mil->page_buffer_virt && virt_addr_valid(mil->page_buffer_virt))
		dma_free_coherent(dev, mil->page_buffer_size,
					mil->page_buffer_virt,
					mil->page_buffer_phys);
	mil->page_buffer_size =  0;
	mil->page_buffer_virt =  0;
	mil->page_buffer_phys = ~0;

	/* Free the command buffer, if it's been allocated. */
	if (mil->cmd_virt)
		dma_free_coherent(dev, MIL_COMMAND_BUFFER_SIZE,
					mil->cmd_virt, mil->cmd_phys);
	mil->cmd_virt =  0;
	mil->cmd_phys = ~0;
}

/* Allocate the DMA command buffer and DMA page buffer */
static int mil_alloc_dma_buffer(struct gpmi_nfc_data *this)
{
	struct device        *dev      =  this->dev;
	struct nfc_geometry  *nfc_geo  = &this->nfc_geometry;
	struct mil           *mil      = &this->mil;

	/* Allocate a command buffer. */
	mil->cmd_virt = dma_alloc_coherent(dev, MIL_COMMAND_BUFFER_SIZE,
					&mil->cmd_phys, GFP_DMA);
	if (!mil->cmd_virt)
		goto error_alloc;

	/*
	 * Allocate the page buffer.
	 *
	 * Both the payload buffer and the auxiliary buffer must appear on
	 * 32-bit boundaries. We presume the size of the payload buffer is a
	 * power of two and is much larger than four, which guarantees the
	 * auxiliary buffer will appear on a 32-bit boundary.
	 */
	mil->page_buffer_size = nfc_geo->payload_size_in_bytes +
					nfc_geo->auxiliary_size_in_bytes;
	mil->page_buffer_virt = dma_alloc_coherent(dev, mil->page_buffer_size,
					&mil->page_buffer_phys, GFP_DMA);
	if (!mil->page_buffer_virt)
		goto error_alloc;

	/* Slice up the page buffer. */
	mil->payload_virt = mil->page_buffer_virt;
	mil->payload_phys = mil->page_buffer_phys;
	mil->auxiliary_virt = ((char *) mil->payload_virt) +
						nfc_geo->payload_size_in_bytes;
	mil->auxiliary_phys = mil->payload_phys +
						nfc_geo->payload_size_in_bytes;
	return 0;

error_alloc:
	printk(KERN_ERR "allocate DMA buffer error!!\n");
	mil_free_dma_buffer(this);
	dump_stack();
	return -ENOMEM;
}

/**
 * mil_cmd_ctrl - MTD Interface cmd_ctrl()
 *
 * This is the function that we install in the cmd_ctrl function pointer of the
 * owning struct nand_chip. The only functions in the reference implementation
 * that use these functions pointers are cmdfunc and select_chip.
 *
 * In this driver, we implement our own select_chip, so this function will only
 * be called by the reference implementation's cmdfunc. For this reason, we can
 * ignore the chip enable bit and concentrate only on sending bytes to the
 * NAND Flash.
 *
 * @mtd:   The owning MTD.
 * @data:  The value to push onto the data signals.
 * @ctrl:  The values to push onto the control signals.
 */
static void mil_cmd_ctrl(struct mtd_info *mtd, int data, unsigned int ctrl)
{
	struct nand_chip      *nand = mtd->priv;
	struct gpmi_nfc_data  *this = nand->priv;
	struct device         *dev  =  this->dev;
	struct mil            *mil  = &this->mil;
	struct nfc_hal        *nfc  =  this->nfc;
	int                   error;
#if defined(CONFIG_MTD_DEBUG)
	unsigned int          i;
	char                  display[MIL_COMMAND_BUFFER_SIZE * 5];
#endif

	/*
	 * Every operation begins with a command byte and a series of zero or
	 * more address bytes. These are distinguished by either the Address
	 * Latch Enable (ALE) or Command Latch Enable (CLE) signals being
	 * asserted. When MTD is ready to execute the command, it will deassert
	 * both latch enables.
	 *
	 * Rather than run a separate DMA operation for every single byte, we
	 * queue them up and run a single DMA operation for the entire series
	 * of command and data bytes.
	 */
	if ((ctrl & (NAND_ALE | NAND_CLE))) {
		if (data != NAND_CMD_NONE)
			mil->cmd_virt[mil->command_length++] = data;
		return;
	}

	/*
	 * If control arrives here, MTD has deasserted both the ALE and CLE,
	 * which means it's ready to run an operation. Check if we have any
	 * bytes to send.
	 */
	if (!mil->command_length)
		return;

	/* Hand the command over to the NFC. */
#if defined(CONFIG_MTD_DEBUG)
	display[0] = 0;
	for (i = 0; i < mil->command_length; i++)
		sprintf(display + strlen(display), " 0x%02x",
						mil->cmd_virt[i] & 0xff);
	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc cmd_ctrl] command: %s\n", display);
#endif

	error = nfc->send_command(this,
			mil->current_chip, mil->cmd_phys, mil->command_length);
	if (error) {
		dev_err(dev, "[%s] Chip: %u, Error %d\n",
					__func__, mil->current_chip, error);
		print_hex_dump(KERN_ERR,
			"    Command Bytes: ", DUMP_PREFIX_NONE, 16, 1,
					mil->cmd_virt, mil->command_length, 0);
	}

	/* Reset. */
	mil->command_length = 0;
}

/**
 * mil_dev_ready() - MTD Interface dev_ready()
 *
 * @mtd:   A pointer to the owning MTD.
 */
static int mil_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip      *nand = mtd->priv;
	struct gpmi_nfc_data  *this = nand->priv;
	struct nfc_hal        *nfc  = this->nfc;
	struct mil            *mil  = &this->mil;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc dev_ready]\n");

	return nfc->is_ready(this, mil->current_chip);
}

/**
 * mil_select_chip() - MTD Interface select_chip()
 *
 * @mtd:   A pointer to the owning MTD.
 * @chip:  The chip number to select, or -1 to select no chip.
 */
static void mil_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip      *nand  = mtd->priv;
	struct gpmi_nfc_data  *this  = nand->priv;
	struct mil            *mil   = &this->mil;
	struct nfc_hal        *nfc   =  this->nfc;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc select_chip] chip: %d\n", chip);

	/* Figure out what kind of transition this is. */
	if ((mil->current_chip < 0) && (chip >= 0)) {
		nfc->begin(this);
	} else if ((mil->current_chip >= 0) && (chip < 0)) {
		nfc->end(this);
	} else {
	}

	mil->current_chip = chip;
}

/**
 * mil_read_buf() - MTD Interface read_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The destination buffer.
 * @len:  The number of bytes to read.
 */
static void mil_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip      *nand     = mtd->priv;
	struct gpmi_nfc_data  *this     = nand->priv;
	struct device         *dev      =  this->dev;
	struct nfc_hal        *nfc      =  this->nfc;
	struct nfc_geometry   *nfc_geo  = &this->nfc_geometry;
	struct mil            *mil      = &this->mil;
	void                  *use_virt =  0;
	dma_addr_t            use_phys  = ~0;
	int                   error;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc readbuf] len: %d\n", len);

	/* Set up DMA. */
	error = mil_incoming_buffer_dma_begin(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					&use_virt, &use_phys);
	if (error) {
		dev_err(dev, "[%s] Inadequate DMA buffer\n", __func__);
		return;
	}

	/* Ask the NFC. */
	nfc->read_data(this, mil->current_chip, use_phys, len);

	/* Finish with DMA. */
	mil_incoming_buffer_dma_end(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					use_virt, use_phys);
}

/**
 * mil_write_buf() - MTD Interface write_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The source buffer.
 * @len:  The number of bytes to read.
 */
static void mil_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip      *nand     = mtd->priv;
	struct gpmi_nfc_data  *this     = nand->priv;
	struct device         *dev      =  this->dev;
	struct nfc_hal        *nfc      =  this->nfc;
	struct nfc_geometry   *nfc_geo  = &this->nfc_geometry;
	struct mil            *mil      = &this->mil;
	const void            *use_virt =  0;
	dma_addr_t            use_phys  = ~0;
	int                   error;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc writebuf] len: %d\n", len);
	/* Set up DMA. */
	error = mil_outgoing_buffer_dma_begin(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					&use_virt, &use_phys);
	if (error) {
		dev_err(dev, "[%s] Inadequate DMA buffer\n", __func__);
		return;
	}

	/* Ask the NFC. */
	nfc->send_data(this, mil->current_chip, use_phys, len);

	/* Finish with DMA. */
	mil_outgoing_buffer_dma_end(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					use_virt, use_phys);
}

/**
 * mil_read_byte() - MTD Interface read_byte().
 *
 * @mtd:  A pointer to the owning MTD.
 */
static uint8_t mil_read_byte(struct mtd_info *mtd)
{
	uint8_t  byte;

	mil_read_buf(mtd, (uint8_t *)&byte, 1);
	return byte;
}

/**
 * mil_handle_block_mark_swapping() - Handles block mark swapping.
 *
 * Note that, when this function is called, it doesn't know whether it's
 * swapping the block mark, or swapping it *back* -- but it doesn't matter
 * because the the operation is the same.
 *
 * @this:       Per-device data.
 * @payload:    A pointer to the payload buffer.
 * @auxiliary:  A pointer to the auxiliary buffer.
 */
static void mil_handle_block_mark_swapping(struct gpmi_nfc_data *this,
						void *payload, void *auxiliary)
{
	struct nfc_geometry     *nfc_geo = &this->nfc_geometry;
	struct boot_rom_helper  *rom     =  this->rom;
	unsigned char           *p;
	unsigned char           *a;
	unsigned int            bit;
	unsigned char           mask;
	unsigned char           from_data;
	unsigned char           from_oob;

	/* Check if we're doing block mark swapping. */
	if (!rom->swap_block_mark)
		return;

	/*
	 * If control arrives here, we're swapping. Make some convenience
	 * variables.
	 */
	bit = nfc_geo->block_mark_bit_offset;
	p   = ((unsigned char *) payload) + nfc_geo->block_mark_byte_offset;
	a   = auxiliary;

	/*
	 * Get the byte from the data area that overlays the block mark. Since
	 * the ECC engine applies its own view to the bits in the page, the
	 * physical block mark won't (in general) appear on a byte boundary in
	 * the data.
	 */
	from_data = (p[0] >> bit) | (p[1] << (8 - bit));

	/* Get the byte from the OOB. */
	from_oob = a[0];

	/* Swap them. */
	a[0] = from_data;

	mask = (0x1 << bit) - 1;
	p[0] = (p[0] & mask) | (from_oob << bit);

	mask = ~0 << bit;
	p[1] = (p[1] & mask) | (from_oob >> (8 - bit));
}

/**
 * mil_ecc_read_page() - MTD Interface ecc.read_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the destination buffer.
 */
static int mil_ecc_read_page(struct mtd_info *mtd, struct nand_chip *nand,
				uint8_t *buf, int page)
{
	struct gpmi_nfc_data    *this    = nand->priv;
	struct device           *dev     =  this->dev;
	struct nfc_hal          *nfc     =  this->nfc;
	struct nfc_geometry     *nfc_geo = &this->nfc_geometry;
	struct mil              *mil     = &this->mil;
	void                    *payload_virt   =  0;
	dma_addr_t              payload_phys    = ~0;
	void                    *auxiliary_virt =  0;
	dma_addr_t              auxiliary_phys  = ~0;
	unsigned int            i;
	unsigned char           *status;
	unsigned int            failed;
	unsigned int            corrected;
	int                     error = 0;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_read_page]\n");

	/*
	 * Set up DMA.
	 *
	 * Notice that we don't try to use the caller's buffer as the auxiliary.
	 * We need to do a lot of fiddling to deliver the OOB, so there's no
	 * point.
	 */
	error = mil_incoming_buffer_dma_begin(this, buf, mtd->writesize,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					&payload_virt, &payload_phys);
	if (error) {
		dev_err(dev, "[%s] Inadequate DMA buffer\n", __func__);
		error = -ENOMEM;
		goto exit_payload;
	}
	auxiliary_virt = mil->auxiliary_virt;
	auxiliary_phys = mil->auxiliary_phys;

	/* Ask the NFC. */
	error = nfc->read_page(this, mil->current_chip,
						payload_phys, auxiliary_phys);
	if (error) {
		dev_err(dev, "[%s] Error in ECC-based read: %d\n",
							__func__, error);
		goto exit_nfc;
	}

	/* Handle block mark swapping. */
	mil_handle_block_mark_swapping(this, payload_virt, auxiliary_virt);

	/* Loop over status bytes, accumulating ECC status. */
	failed		= 0;
	corrected	= 0;
	status		= ((unsigned char *) auxiliary_virt) +
					nfc_geo->auxiliary_status_offset;

	for (i = 0; i < nfc_geo->ecc_chunk_count; i++, status++) {
		if ((*status == 0x00) || (*status == 0xff))
			continue;

		if (*status == 0xfe) {
			failed++;
			continue;
		}
		corrected += *status;
	}

	/*
	 * Propagate ECC status to the owning MTD only when failed or
	 * corrected times nearly reaches our ECC correction threshold.
	 */
	if (failed || corrected >= (nfc_geo->ecc_strength - 1)) {
		mtd->ecc_stats.failed    += failed;
		mtd->ecc_stats.corrected += corrected;
	}

	/*
	 * It's time to deliver the OOB bytes. See mil_ecc_read_oob() for
	 * details about our policy for delivering the OOB.
	 *
	 * We fill the caller's buffer with set bits, and then copy the block
	 * mark to th caller's buffer. Note that, if block mark swapping was
	 * necessary, it has already been done, so we can rely on the first
	 * byte of the auxiliary buffer to contain the block mark.
	 */
	memset(nand->oob_poi, ~0, mtd->oobsize);
	nand->oob_poi[0] = ((uint8_t *) auxiliary_virt)[0];

exit_nfc:
	mil_incoming_buffer_dma_end(this, buf, mtd->writesize,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					payload_virt, payload_phys);
exit_payload:
	return error;
}

/**
 * mil_ecc_write_page() - MTD Interface ecc.write_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the source buffer.
 */
static void mil_ecc_write_page(struct mtd_info *mtd,
				struct nand_chip *nand, const uint8_t *buf)
{
	struct gpmi_nfc_data    *this    = nand->priv;
	struct device           *dev     =  this->dev;
	struct nfc_hal          *nfc     =  this->nfc;
	struct nfc_geometry     *nfc_geo = &this->nfc_geometry;
	struct boot_rom_helper  *rom     =  this->rom;
	struct mil              *mil     = &this->mil;
	const void              *payload_virt   =  0;
	dma_addr_t              payload_phys    = ~0;
	const void              *auxiliary_virt =  0;
	dma_addr_t              auxiliary_phys  = ~0;
	int                     error;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_write_page]\n");

	/* Set up DMA. */
	if (rom->swap_block_mark) {
		/*
		 * If control arrives here, we're doing block mark swapping.
		 * Since we can't modify the caller's buffers, we must copy them
		 * into our own.
		 */
		memcpy(mil->payload_virt, buf, mtd->writesize);
		payload_virt = mil->payload_virt;
		payload_phys = mil->payload_phys;

		memcpy(mil->auxiliary_virt, nand->oob_poi,
				nfc_geo->auxiliary_size_in_bytes);
		auxiliary_virt = mil->auxiliary_virt;
		auxiliary_phys = mil->auxiliary_phys;

		/* Handle block mark swapping. */
		mil_handle_block_mark_swapping(this,
				(void *) payload_virt, (void *) auxiliary_virt);
	} else {
		/*
		 * If control arrives here, we're not doing block mark swapping,
		 * so we can to try and use the caller's buffers.
		 */
		error = mil_outgoing_buffer_dma_begin(this,
				buf, mtd->writesize,
				mil->payload_virt, mil->payload_phys,
				nfc_geo->payload_size_in_bytes,
				&payload_virt, &payload_phys);
		if (error) {
			dev_err(dev, "[%s] Inadequate payload DMA buffer\n",
								__func__);
			return;
		}

		error = mil_outgoing_buffer_dma_begin(this,
				nand->oob_poi, mtd->oobsize,
				mil->auxiliary_virt, mil->auxiliary_phys,
				nfc_geo->auxiliary_size_in_bytes,
				&auxiliary_virt, &auxiliary_phys);
		if (error) {
			dev_err(dev, "[%s] Inadequate auxiliary DMA buffer\n",
								__func__);
			goto exit_auxiliary;
		}
	}

	/* Ask the NFC. */
	error = nfc->send_page(this, mil->current_chip,
						payload_phys, auxiliary_phys);
	if (error)
		dev_err(dev, "[%s] Error in ECC-based write: %d\n",
							__func__, error);

	/* Return. */
	if (!rom->swap_block_mark)
		mil_outgoing_buffer_dma_end(this, nand->oob_poi, mtd->oobsize,
				mil->auxiliary_virt, mil->auxiliary_phys,
				nfc_geo->auxiliary_size_in_bytes,
				auxiliary_virt, auxiliary_phys);
exit_auxiliary:
	if (!rom->swap_block_mark)
		mil_outgoing_buffer_dma_end(this, buf, mtd->writesize,
				mil->payload_virt, mil->payload_phys,
				nfc_geo->payload_size_in_bytes,
				payload_virt, payload_phys);
}

/**
 * mil_hook_block_markbad() - Hooked MTD Interface block_markbad().
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code. See the description of the marking_a_bad_block field
 * in struct mil for more information about this.
 *
 * @mtd:  A pointer to the MTD.
 * @ofs:  Byte address of the block to mark.
 */
static int mil_hook_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	register struct nand_chip  *chip = mtd->priv;
	struct gpmi_nfc_data       *this = chip->priv;
	struct mil                 *mil  = &this->mil;
	int                        ret;

	mil->marking_a_bad_block = true;
	ret = mil->hooked_block_markbad(mtd, ofs);
	mil->marking_a_bad_block = false;
	return ret;
}

/**
 * mil_ecc_read_oob() - MTD Interface ecc.read_oob().
 *
 * There are several places in this driver where we have to handle the OOB and
 * block marks. This is the function where things are the most complicated, so
 * this is where we try to explain it all. All the other places refer back to
 * here.
 *
 * These are the rules, in order of decreasing importance:
 *
 * 1) Nothing the caller does can be allowed to imperil the block mark, so all
 *    write operations take measures to protect it.
 *
 * 2) In read operations, the first byte of the OOB we return must reflect the
 *    true state of the block mark, no matter where that block mark appears in
 *    the physical page.
 *
 * 3) ECC-based read operations return an OOB full of set bits (since we never
 *    allow ECC-based writes to the OOB, it doesn't matter what ECC-based reads
 *    return).
 *
 * 4) "Raw" read operations return a direct view of the physical bytes in the
 *    page, using the conventional definition of which bytes are data and which
 *    are OOB. This gives the caller a way to see the actual, physical bytes
 *    in the page, without the distortions applied by our ECC engine.
 *
 *
 * What we do for this specific read operation depends on two questions:
 *
 * 1) Are we doing a "raw" read, or an ECC-based read?
 *
 * 2) Are we using block mark swapping or transcription?
 *
 * There are four cases, illustrated by the following Karnaugh map:
 *
 *                    |           Raw           |         ECC-based       |
 *       -------------+-------------------------+-------------------------+
 *                    | Read the conventional   |                         |
 *                    | OOB at the end of the   |                         |
 *       Swapping     | page and return it. It  |                         |
 *                    | contains exactly what   |                         |
 *                    | we want.                | Read the block mark and |
 *       -------------+-------------------------+ return it in a buffer   |
 *                    | Read the conventional   | full of set bits.       |
 *                    | OOB at the end of the   |                         |
 *                    | page and also the block |                         |
 *       Transcribing | mark in the metadata.   |                         |
 *                    | Copy the block mark     |                         |
 *                    | into the first byte of  |                         |
 *                    | the OOB.                |                         |
 *       -------------+-------------------------+-------------------------+
 *
 * Note that we break rule #4 in the Transcribing/Raw case because we're not
 * giving an accurate view of the actual, physical bytes in the page (we're
 * overwriting the block mark). That's OK because it's more important to follow
 * rule #2.
 *
 * It turns out that knowing whether we want an "ECC-based" or "raw" read is not
 * easy. When reading a page, for example, the NAND Flash MTD code calls our
 * ecc.read_page or ecc.read_page_raw function. Thus, the fact that MTD wants an
 * ECC-based or raw view of the page is implicit in which function it calls
 * (there is a similar pair of ECC-based/raw functions for writing).
 *
 * Since MTD assumes the OOB is not covered by ECC, there is no pair of
 * ECC-based/raw functions for reading or or writing the OOB. The fact that the
 * caller wants an ECC-based or raw view of the page is not propagated down to
 * this driver.
 *
 * @mtd:     A pointer to the owning MTD.
 * @nand:    A pointer to the owning NAND Flash MTD.
 * @page:    The page number to read.
 * @sndcmd:  Indicates this function should send a command to the chip before
 *           reading the out-of-band bytes. This is only false for small page
 *           chips that support auto-increment.
 */
static int mil_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *nand,
							int page, int sndcmd)
{
	struct gpmi_nfc_data      *this     = nand->priv;
	struct boot_rom_helper    *rom      =  this->rom;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_read_oob] "
		"page: 0x%06x, sndcmd: %s\n", page, sndcmd ? "Yes" : "No");

	/* clear the OOB buffer */
	memset(nand->oob_poi, ~0, mtd->oobsize);

	/* Read out the conventional OOB. */
	nand->cmdfunc(mtd, NAND_CMD_READ0, mtd->writesize, page);
	nand->read_buf(mtd, nand->oob_poi, mtd->oobsize);

	/*
	 * Now, we want to make sure the block mark is correct. In the
	 * Swapping/Raw case, we already have it. Otherwise, we need to
	 * explicitly read it.
	 */
	if (!rom->swap_block_mark) {
		/* Read the block mark into the first byte of the OOB buffer. */
		nand->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
		nand->oob_poi[0] = nand->read_byte(mtd);
	}

	/*
	 * Return true, indicating that the next call to this function must send
	 * a command.
	 */
	return true;
}

/**
 * mil_ecc_write_oob() - MTD Interface ecc.write_oob().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @page:  The page number to write.
 */
static int mil_ecc_write_oob(struct mtd_info *mtd,
					struct nand_chip *nand, int page)
{
	struct gpmi_nfc_data      *this     = nand->priv;
	struct device             *dev      =  this->dev;
	struct mil                *mil      = &this->mil;
	struct boot_rom_helper    *rom      =  this->rom;
	uint8_t                   block_mark = 0;
	int                       block_mark_column;
	int                       status;
	int                       error = 0;

	DEBUG(MTD_DEBUG_LEVEL2,
			"[gpmi_nfc ecc_write_oob] page: 0x%06x\n", page);
	/*
	 * There are fundamental incompatibilities between the i.MX GPMI NFC and
	 * the NAND Flash MTD model that make it essentially impossible to write
	 * the out-of-band bytes.
	 *
	 * We permit *ONE* exception. If the *intent* of writing the OOB is to
	 * mark a block bad, we can do that.
	 */
	if (!mil->marking_a_bad_block) {
		dev_emerg(dev, "This driver doesn't support writing the OOB\n");
		WARN_ON(1);
		error = -EIO;
		goto exit;
	}

	/*
	 * If control arrives here, we're marking a block bad. First, figure out
	 * where the block mark is.
	 *
	 * If we're using swapping, the block mark is in the conventional
	 * location. Otherwise, we're using transcription, and the block mark
	 * appears in the first byte of the page.
	 */
	if (rom->swap_block_mark)
		block_mark_column = mtd->writesize;
	else
		block_mark_column = 0;

	/* Write the block mark. */
	nand->cmdfunc(mtd, NAND_CMD_SEQIN, block_mark_column, page);
	nand->write_buf(mtd, &block_mark, 1);
	nand->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = nand->waitfunc(mtd, nand);

	/* Check if it worked. */
	if (status & NAND_STATUS_FAIL)
		error = -EIO;
exit:
	return error;
}

/**
 * mil_block_bad - Claims all blocks are good.
 *
 * In principle, this function is *only* called when the NAND Flash MTD system
 * isn't allowed to keep an in-memory bad block table, so it is forced to ask
 * the driver for bad block information.
 *
 * In fact, we permit the NAND Flash MTD system to have an in-memory BBT, so
 * this function is *only* called when we take it away.
 *
 * We take away the in-memory BBT when the user sets the "ignorebad" parameter,
 * which indicates that all blocks should be reported good.
 *
 * Thus, this function is only called when we want *all* blocks to look good,
 * so it *always* return success.
 *
 * @mtd:      Ignored.
 * @ofs:      Ignored.
 * @getchip:  Ignored.
 */
static int mil_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	return 0;
}


/**
 * mil_set_nfc_geometry() - Set up the NFC geometry.
 *
 * This function calls the NFC HAL to select an NFC geometry that is compatible
 * with the medium's physical geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_nfc_geometry(struct gpmi_nfc_data  *this)
{
	struct nfc_hal       *nfc =  this->nfc;
#if defined(DETAILED_INFO)
	struct nfc_geometry  *geo = &this->nfc_geometry;
#endif
	/* Set the NFC geometry. */
	if (nfc->set_geometry(this))
		return !0;

	#if defined(DETAILED_INFO)
	pr_info("------------\n");
	pr_info("NFC Geometry\n");
	pr_info("------------\n");
	pr_info("ECC Algorithm          : %s\n", geo->ecc_algorithm);
	pr_info("ECC Strength           : %u\n", geo->ecc_strength);
	pr_info("Page Size in Bytes     : %u\n", geo->page_size_in_bytes);
	pr_info("Metadata Size in Bytes : %u\n", geo->metadata_size_in_bytes);
	pr_info("ECC Chunk Size in Bytes: %u\n", geo->ecc_chunk_size_in_bytes);
	pr_info("ECC Chunk Count        : %u\n", geo->ecc_chunk_count);
	pr_info("Payload Size in Bytes  : %u\n", geo->payload_size_in_bytes);
	pr_info("Auxiliary Size in Bytes: %u\n", geo->auxiliary_size_in_bytes);
	pr_info("Auxiliary Status Offset: %u\n", geo->auxiliary_status_offset);
	pr_info("Block Mark Byte Offset : %u\n", geo->block_mark_byte_offset);
	pr_info("Block Mark Bit Offset  : %u\n", geo->block_mark_bit_offset);
	#endif

	return 0;
}

/**
 * mil_set_boot_rom_helper_geometry() - Set up the Boot ROM Helper geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_boot_rom_helper_geometry(struct gpmi_nfc_data  *this)
{
	struct boot_rom_helper    *rom =  this->rom;
#if defined(DETAILED_INFO)
	struct boot_rom_geometry  *geo = &this->rom_geometry;
#endif

	/* Set the Boot ROM Helper geometry. */
	if (rom->set_geometry(this))
		return !0;

	#if defined(DETAILED_INFO)
	pr_info("-----------------\n");
	pr_info("Boot ROM Geometry\n");
	pr_info("-----------------\n");
	pr_info("Boot Area Count            : %u\n", geo->boot_area_count);
	pr_info("Boot Area Size in Bytes    : %u (0x%x)\n",
		geo->boot_area_size_in_bytes, geo->boot_area_size_in_bytes);
	pr_info("Stride Size in Pages       : %u\n", geo->stride_size_in_pages);
	pr_info("Search Area Stride Exponent: %u\n",
					geo->search_area_stride_exponent);
	#endif
	return 0;
}

/**
 * mil_set_geometry() - Set up the medium geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_geometry(struct gpmi_nfc_data  *this)
{
	/* Free the temporary DMA memory for read ID case */
	mil_free_dma_buffer(this);

	/* Set up the NFC geometry. */
	if (mil_set_nfc_geometry(this))
		return -ENXIO;

	/* Alloc the new DMA buffers according to the pagesize and oobsize */
	return mil_alloc_dma_buffer(this);
}

/**
 * mil_pre_bbt_scan() - Prepare for the BBT scan.
 *
 * @this:  Per-device data.
 */
static int mil_pre_bbt_scan(struct gpmi_nfc_data  *this)
{
	struct boot_rom_helper	*rom	= this->rom;
	int			error	= 0;

	if (mil_set_boot_rom_helper_geometry(this))
		return -ENXIO;

	/* This is ROM arch-specific initilization before the BBT scanning. */
	if (rom->rom_extra_init)
		error = rom->rom_extra_init(this);
	return error;
}

/**
 * mil_scan_bbt() - MTD Interface scan_bbt().
 *
 * The HIL calls this function once, when it initializes the NAND Flash MTD.
 *
 * @mtd:  A pointer to the owning MTD.
 */
static int mil_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip         *nand = mtd->priv;
	struct gpmi_nfc_data     *this = nand->priv;
	int                      error;

	/*
	 * Tell MTD users that the out-of-band area can't be written.
	 *
	 * This flag is not part of the standard kernel source tree. It comes
	 * from a patch that touches both MTD and JFFS2.
	 *
	 * The problem is that, without this patch, JFFS2 believes it can write
	 * the data area and the out-of-band area separately. This is wrong for
	 * two reasons:
	 *
	 *     1)  Our NFC distributes out-of-band bytes throughout the page,
	 *         intermingled with the data, and covered by the same ECC.
	 *         Thus, it's not possible to write the out-of-band bytes and
	 *         data bytes separately.
	 *
	 *     2)  Large page (MLC) Flash chips don't support partial page
	 *         writes. You must write the entire page at a time. Thus, even
	 *         if our NFC didn't force you to write out-of-band and data
	 *         bytes together, it would *still* be a bad idea to do
	 *         otherwise.
	 */
	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/* Prepare for the BBT scan. */
	error = mil_pre_bbt_scan(this);
	if (error)
		return error;

	/* use the default BBT implementation */
	return nand_default_bbt(mtd);
}

/**
 * mil_boot_areas_init() - Initializes boot areas.
 *
 * @this:  Per-device data.
 */
static int mil_boot_areas_init(struct gpmi_nfc_data *this)
{
	struct device                  *dev      =  this->dev;
	struct boot_rom_geometry       *rom      = &this->rom_geometry;
	struct mil                     *mil      = &this->mil;
	struct mtd_info                *mtd      = &mil->mtd;
	struct nand_chip               *nand     = &mil->nand;
	struct nand_device_info		*info	 = &this->device_info;
	int                            mtd_support_is_adequate;
	unsigned int                   i;
	struct mtd_partition           partitions[4];
	struct mtd_info                *search_mtd;
	struct mtd_info                *chip_0_remainder_mtd = 0;
	struct mtd_info                *medium_remainder_mtd = 0;
	struct mtd_info                *concatenate[2];

	/*
	 * Here we declare the static strings we use to name partitions. We use
	 * static strings because, as of 2.6.31, the partitioning code *always*
	 * registers the partition MTDs it creates and leaves behind *no* other
	 * trace of its work. So, once we've created a partition, we must search
	 * the master MTD table to find the MTDs we created. Since we're using
	 * static strings, we can simply search the master table for an MTD with
	 * a name field pointing to a known address.
	 */
	static char  *chip_0_boot_name      = "gpmi-nfc-0-boot";
	static char  *chip_0_remainder_name = "gpmi-nfc-0-remainder";
	static char  *chip_1_boot_name      = "gpmi-nfc-1-boot";
	static char  *medium_remainder_name = "gpmi-nfc-remainder";
	static char  *general_use_name      = "gpmi-nfc-general-use";

	/* Check if we're protecting the boot areas.*/
	if (!rom->boot_area_count) {
		/*
		 * If control arrives here, we're not protecting the boot areas.
		 * In this case, there are not boot area partitons, and the main
		 * MTD is the general use MTD.
		 */
		mil->general_use_mtd = &mil->mtd;

		return 0;
	}

	/*
	 * If control arrives here, we're protecting the boot areas. Check if we
	 * have the MTD support we need.
	 */
	pr_info("Boot area protection is enabled.\n");

	if (rom->boot_area_count > 1) {
		/*
		 * If the Boot ROM wants more than one boot area, then we'll
		 * need to create partitions *and* concatenate them.
		 */
		#if defined(CONFIG_MTD_PARTITIONS) && defined(CONFIG_MTD_CONCAT)
			mtd_support_is_adequate = true;
		#else
			mtd_support_is_adequate = false;
		#endif
	} else if (rom->boot_area_count == 1) {
		/*
		 * If the Boot ROM wants only one boot area, then we only need
		 * to create partitions -- we don't need to concatenate them.
		 */
		#if defined(CONFIG_MTD_PARTITIONS)
			mtd_support_is_adequate = true;
		#else
			mtd_support_is_adequate = false;
		#endif
	} else {
		/*
		 * If control arrives here, we're protecting the boot area, but
		 * somehow the boot area count was set to zero. This doesn't
		 * make any sense.
		 */
		dev_err(dev, "Internal error: boot area count is "
						"incorrectly set to zero.");
		return -ENXIO;
	}

	if (!mtd_support_is_adequate) {
		dev_err(dev, "Configured MTD support is inadequate to "
						"protect the boot area(s).");
		return -ENXIO;
	}

	/*
	 * If control arrives here, we're protecting boot areas and we have
	 * everything we need to do so.
	 *
	 * We have special code to handle the case for one boot area.
	 *
	 * The code that handles "more than one" boot area actually only handles
	 * two. We *could* write the general case, but that would take a lot of
	 * time to both write and test -- and, right now, we don't have a chip
	 * that cares.
	 */

	/* Check if a boot area is larger than a single chip. */
	if (rom->boot_area_size_in_bytes > info->chip_size_in_bytes) {
		dev_emerg(dev, "Boot area size is larger than a chip");
		return -ENXIO;
	}

	if (rom->boot_area_count == 1) {
#if defined(CONFIG_MTD_PARTITIONS)
		/*
		 * We partition the medium like so:
		 *
		 * +------+----------------------------------------------------+
		 * | Boot |                    General Use                     |
		 * +------+----------------------------------------------------+
		 */

		/* Chip 0 Boot */
		partitions[0].name       = chip_0_boot_name;
		partitions[0].offset     = 0;
		partitions[0].size       = rom->boot_area_size_in_bytes;
		partitions[0].mask_flags = 0;

		/* General Use */
		partitions[1].name       = general_use_name;
		partitions[1].offset     = rom->boot_area_size_in_bytes;
		partitions[1].size       = MTDPART_SIZ_FULL;
		partitions[1].mask_flags = 0;

		/* Construct and register the partitions. */
		add_mtd_partitions(mtd, partitions, 2);

		/* Find the general use MTD. */
		i = 0;
		while ((search_mtd = get_mtd_device(0, i))) {
			/* Check if we got nonsense. */
			if ((!search_mtd) || (search_mtd == ERR_PTR(-ENODEV)))
				break;

			/* Check if the current MTD is one of our remainders. */
			if (search_mtd->name == general_use_name)
				mil->general_use_mtd = search_mtd;

			/* Put the MTD back. We only wanted a quick look. */
			put_mtd_device(search_mtd);

			i++;
		}

		if (!mil->general_use_mtd) {
			dev_emerg(dev, "Can't find general use MTD");
			BUG();
		}
#endif
	} else if (rom->boot_area_count == 2) {
#if defined(CONFIG_MTD_PARTITIONS) && defined(CONFIG_MTD_CONCAT)
		/*
		 * If control arrives here, there is more than one boot area.
		 * We partition the medium and concatenate the remainders like
		 * so:
		 *
		 *  --- Chip 0 ---   --- Chip 1 --- ... ------- Chip N -------
		 * /              \ /                                         \
		 * +----+----------+----+--------------- ... ------------------+
		 * |Boot|Remainder |Boot|              Remainder               |
		 * +----+----------+----+--------------- ... ------------------+
		 *      |          |   /                                      /
		 *      |          |  /                                      /
		 *      |          | /                                      /
		 *      |          |/                                      /
		 *      +----------+----------- ... ----------------------+
		 *      |                   General Use                   |
		 *      +---------------------- ... ----------------------+
		 *
		 * Notice that the results we leave in the master MTD table
		 * look like this:
		 *
		 *    * Chip 0 Boot Area
		 *    * Chip 1 Boot Area
		 *    * General Use
		 *
		 * Some user space programs expect the boot partitions to
		 * appear first. This is naive, but let's try not to cause
		 * any trouble, where we can avoid it.
		 */

		/* Chip 0 Boot */
		partitions[0].name       = chip_0_boot_name;
		partitions[0].offset     = 0;
		partitions[0].size       = rom->boot_area_size_in_bytes;
		partitions[0].mask_flags = 0;

		/* Chip 1 Boot */
		partitions[1].name       = chip_1_boot_name;
		partitions[1].offset     = nand->chipsize;
		partitions[1].size       = rom->boot_area_size_in_bytes;
		partitions[1].mask_flags = 0;

		/* Chip 0 Remainder */
		partitions[2].name       = chip_0_remainder_name;
		partitions[2].offset     = rom->boot_area_size_in_bytes;
		partitions[2].size       = nand->chipsize -
						rom->boot_area_size_in_bytes;
		partitions[2].mask_flags = 0;

		/* Medium Remainder */
		partitions[3].name       = medium_remainder_name;
		partitions[3].offset     = nand->chipsize +
						rom->boot_area_size_in_bytes;
		partitions[3].size       = MTDPART_SIZ_FULL;
		partitions[3].mask_flags = 0;

		/* Construct and register the partitions. */
		add_mtd_partitions(mtd, partitions, 4);

		/* Find the remainder partitions. */
		i = 0;
		while ((search_mtd = get_mtd_device(0, i))) {
			/* Check if we got nonsense. */
			if ((!search_mtd) || (search_mtd == ERR_PTR(-ENODEV)))
				break;

			/* Check if the current MTD is one of our remainders. */
			if (search_mtd->name == chip_0_remainder_name)
				chip_0_remainder_mtd = search_mtd;

			if (search_mtd->name == medium_remainder_name)
				medium_remainder_mtd = search_mtd;

			/* Put the MTD back. We only wanted a quick look. */
			put_mtd_device(search_mtd);

			i++;
		}

		if (!chip_0_remainder_mtd || !medium_remainder_mtd) {
			dev_emerg(dev, "Can't find remainder partitions");
			BUG();
		}

		/*
		 * Unregister the remainder MTDs. Note that we are *not*
		 * destroying these MTDs -- we're just removing from the
		 * globally-visible list. There's no need for anyone to see
		 * these.
		 */
		del_mtd_device(chip_0_remainder_mtd);
		del_mtd_device(medium_remainder_mtd);

		/* Concatenate the remainders and register the result. */
		concatenate[0] = chip_0_remainder_mtd;
		concatenate[1] = medium_remainder_mtd;

		mil->general_use_mtd = mtd_concat_create(concatenate,
							2, general_use_name);

		add_mtd_device(mil->general_use_mtd);
#endif

	} else {
		dev_err(dev, "Boot area count greater than two is "
							"unimplemented.\n");
		return -ENXIO;
	}

	return 0;
}

/**
 * mil_boot_areas_exit() - Shuts down boot areas.
 *
 * @this:  Per-device data.
 */
static void mil_boot_areas_exit(struct gpmi_nfc_data *this)
{
	struct boot_rom_geometry  *rom = &this->rom_geometry;
	struct mil                *mil = &this->mil;
	struct mtd_info           *mtd = &mil->mtd;

	/* Check if we're protecting the boot areas.*/
	if (!rom->boot_area_count) {
		/*
		 * If control arrives here, we're not protecting the boot areas.
		 * That means we never created any boot area partitions, and the
		 * general use MTD is just the main MTD.
		 */
		mil->general_use_mtd = 0;

		return;
	}

	/*
	 * If control arrives here, we're protecting the boot areas.
	 *
	 * Start by checking if there is more than one boot area. If so, then
	 * we both partitioned the medium and then concatenated some of the
	 * partitions to form the general use MTD. The first step is to get rid
	 * of the concatenation.
	 */
	#if defined(CONFIG_MTD_PARTITIONS) && defined(CONFIG_MTD_CONCAT)
		if (rom->boot_area_count > 1) {
				del_mtd_device(mil->general_use_mtd);
				mtd_concat_destroy(mil->general_use_mtd);
		}
	#endif

	/*
	 * At this point, we're left only with the partitions of the main MTD.
	 * Delete them.
	 */
	#if defined(CONFIG_MTD_PARTITIONS)
		del_mtd_partitions(mtd);
	#endif

	/* The general use MTD no longer exists. */
	mil->general_use_mtd = 0;
}

/**
 * mil_construct_ubi_partitions() - Constructs partitions for UBI.
 *
 * MTD uses a 64-bit value to express the size of MTDs, but UBI is still using
 * a 32-bit value. For this reason, UBI can't work on top of an MTD with size
 * greater than 2GiB. In this function, we examine the general use MTD and, if
 * it's larger than 2GiB, we construct a set of partitions for that MTD such
 * that none are too large for UBI to comprehend.
 *
 * @this:  Per-device data.
 */
static void mil_construct_ubi_partitions(struct gpmi_nfc_data *this)
{
#if defined(CONFIG_MTD_PARTITIONS)
	struct device                  *dev   =  this->dev;
	struct mil                     *mil   = &this->mil;
	unsigned int                   partition_count;
	struct mtd_partition           *partitions;
	unsigned int                   name_size;
	char                           *names;
	unsigned int                   memory_block_size;
	unsigned int                   i;

	static const char              *name_prefix = "gpmi-nfc-ubi-";

	/*
	 * If the general use MTD isn't larger than 2GiB, we have nothing to do.
	 */
	if (mil->general_use_mtd->size <= SZ_2G)
		return;

	/*
	 * If control arrives here, the general use MTD is larger than 2GiB. We
	 * need to split it up into some number of partitions. Find out how many
	 * 2GiB partitions we'll be creating.
	 */
	partition_count = mil->general_use_mtd->size >> 31;

	/*
	 * If the MTD size doesn't evenly divide by 2GiB, we'll need another
	 * partition to hold the extra.
	 */
	if (mil->general_use_mtd->size & ((1 << 30) - 1))
		partition_count++;

	/*
	 * We're going to allocate a single memory block to contain all the
	 * partition structures and their names. Calculate how large it must be.
	 */
	name_size = strlen(name_prefix) + 4;

	memory_block_size = (sizeof(*partitions) + name_size) * partition_count;

	/*
	 * Attempt to allocate the block.
	 */
	partitions = kzalloc(memory_block_size, GFP_KERNEL);
	if (!partitions) {
		dev_err(dev, "Could not allocate memory for UBI partitions.\n");
		return;
	}

	names = (char *)(partitions + partition_count);

	/* Loop over partitions, filling in the details. */
	for (i = 0; i < partition_count; i++) {
		partitions[i].name   = names;
		partitions[i].size   = SZ_2G;
		partitions[i].offset = MTDPART_OFS_NXTBLK;

		sprintf(names, "%s%u", name_prefix, i);
		names += name_size;
	}

	/* Adjust the last partition to take up the remainder. */
	partitions[i - 1].size = MTDPART_SIZ_FULL;

	/* Record everything in the device data structure.  */
	mil->partitions           = partitions;
	mil->partition_count      = partition_count;
	mil->ubi_partition_memory = partitions;
#endif
}

/**
 * mil_partitions_init() - Initializes partitions.
 *
 * @this:  Per-device data.
 */
static int mil_partitions_init(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata =  this->pdata;
	struct mil                     *mil   = &this->mil;
	struct mtd_info                *mtd   = &mil->mtd;
	int                            error;

	/*
	 * Set up the boot areas. When this function returns, if there has been
	 * no error, the boot area partitions (if any) will have been created
	 * and registered. Also, the general_use_mtd field will point to an MTD
	 * we can use.
	 */
	error = mil_boot_areas_init(this);
	if (error)
		return error;

	/*
	 * If we've been told to, register the MTD that represents the entire
	 * medium. Normally, we don't register the main MTD because we only want
	 * to expose the medium through the boot area partitions and the general
	 * use partition.
	 *
	 * We do this *after* setting up the boot areas because, for historical
	 * reasons, we like the lowest-numbered MTDs to be the boot areas.
	 */
	if (register_main_mtd) {
		pr_info("Registering the main MTD.\n");
		add_mtd_device(mtd);
	}

#if defined(CONFIG_MTD_PARTITIONS)
	/*
	 * If control arrives here, partitioning is available.
	 *
	 * There are three possible sets of partitions we might apply, in order
	 * of decreasing priority:
	 *
	 * 1) Partitions dynamically discovered from sources defined by the
	 *    platform. These can come from, for example, the command line or
	 *    a partition table.
	 *
	 * 2) Partitions attached to the platform data.
	 *
	 * 3) Partitions we generate to deal with limitations in UBI.
	 *
	 * Recall that the pointer to the general use MTD *may* just point to
	 * the main MTD.
	 */

	/*
	 * First, try to get partition information from the sources defined by
	 * the platform.
	 */
	if (pdata->partition_source_types)
		mil->partition_count =
			parse_mtd_partitions(mil->general_use_mtd,
						pdata->partition_source_types,
						&mil->partitions, 0);

	/*
	 * Check if we got anything. If not, then accept whatever partitions are
	 * attached to the platform data.
	 */
	if ((mil->partition_count <= 0) && (pdata->partitions)) {
		mil->partition_count = mil->partition_count;
		mil->partitions      = mil->partitions;
	}

	/*
	 * If we still don't have any partitions to apply, then we might want to
	 * apply some of our own, to account for UBI's limitations.
	 */
	if (!mil->partition_count)
		mil_construct_ubi_partitions(this);

	/* If we came up with any partitions, apply them. */
	if (mil->partition_count)
		add_mtd_partitions(mil->general_use_mtd,
					mil->partitions,
					mil->partition_count);
#endif
	return 0;
}

/**
 * mil_partitions_exit() - Shuts down partitions.
 *
 * @this:  Per-device data.
 */
static void mil_partitions_exit(struct gpmi_nfc_data *this)
{
	struct mil       *mil   = &this->mil;
	struct mtd_info  *mtd   = &mil->mtd;

	/* Check if we applied any partitions to the general use MTD. */
	#if defined(CONFIG_MTD_PARTITIONS)

		if (mil->partition_count)
			del_mtd_partitions(mil->general_use_mtd);

		kfree(mil->ubi_partition_memory);

	#endif

	/*
	 * If we were told to register the MTD that represents the entire
	 * medium, unregister it now. Note that this does *not* "destroy" the
	 * MTD - it merely unregisters it. That's important because all our
	 * other MTDs depend on this one.
	 */
	if (register_main_mtd)
		del_mtd_device(mtd);

	/* Tear down the boot areas. */
	mil_boot_areas_exit(this);
}

/*
 * This function is used to set the mtd->pagesize, mtd->oobsize,
 * mtd->erasesize. Yes, we also do some initialization.
 *
 * Return with the bus width. 0 for 8-bit, -1 for error.
 */
static int gpmi_init_size(struct mtd_info *mtd, struct nand_chip *nand,
				u8 *id_bytes)
{
	struct gpmi_nfc_data *this	= nand->priv;
	struct nfc_hal       *nfc	= this->nfc;
	struct mil           *mil	= &this->mil;
	struct nand_ecclayout *layout	= &mil->oob_layout;
	struct nand_device_info  *info;
	struct gpmi_nfc_timing   timing;
	int error;

	/* Look up this device in our database. */
	info = nand_device_get_info(id_bytes);
	if (!info) {
		pr_err("Unrecognized NAND Flash device.\n");
		return -1;
	}

	/* Display the information we discovered. */
	#if defined(DETAILED_INFO)
	pr_info("-----------------------------\n");
	pr_info("NAND Flash Device Information\n");
	pr_info("-----------------------------\n");
	nand_device_print_info(info);
	#endif

	/*
	 *  Init the right NAND/MTD parameters which will be used
	 *  in the following mil_set_geometry().
	 */
	mtd->writesize	= 1 << (fls(info->page_total_size_in_bytes) - 1);
	mtd->erasesize	= mtd->writesize * info->block_size_in_pages;
	mtd->oobsize	= info->page_total_size_in_bytes - mtd->writesize;
	nand->chipsize	= info->chip_size_in_bytes;

	/* Configure the struct nand_ecclayout. */
	layout->eccbytes          = 0;
	layout->oobavail          = mtd->oobsize;
	layout->oobfree[0].offset = 0;
	layout->oobfree[0].length = mtd->oobsize;

	nand->ecc.layout = layout;

	/*
	 * Copy the device info into the per-device data. We can't just keep
	 * the pointer because that storage is reclaimed after initialization.
	 */
	this->device_info = *info;
	this->device_info.description = kstrdup(info->description, GFP_KERNEL);

	/* Set up geometry. */
	error = mil_set_geometry(this);
	if (error)
		return -1;

	/* Set up timing. */
	timing.data_setup_in_ns        = info->data_setup_in_ns;
	timing.data_hold_in_ns         = info->data_hold_in_ns;
	timing.address_setup_in_ns     = info->address_setup_in_ns;
	timing.gpmi_sample_delay_in_ns = info->gpmi_sample_delay_in_ns;
	timing.tREA_in_ns              = info->tREA_in_ns;
	timing.tRLOH_in_ns             = info->tRLOH_in_ns;
	timing.tRHOH_in_ns             = info->tRHOH_in_ns;

	error = nfc->set_timing(this, &timing);
	if (error)
		return -1;

	if (nfc->extra_init) {
		error = nfc->extra_init(this);
		if (error != 0)
			return -1;
	}

	/* We only use 8-bit bus now, not 16-bit. */
	return 0;
}

/**
 * gpmi_nfc_mil_init() - Initializes the MTD Interface Layer.
 *
 * @this:  Per-device data.
 */
int gpmi_nfc_mil_init(struct gpmi_nfc_data *this)
{
	struct device                  *dev   =  this->dev;
	struct gpmi_nfc_platform_data  *pdata =  this->pdata;
	struct mil                     *mil   = &this->mil;
	struct mtd_info                *mtd   = &mil->mtd;
	struct nand_chip               *nand  = &mil->nand;
	int                            error = 0;

	/* Initialize MIL data. */
	mil->current_chip   = -1;
	mil->command_length =  0;

	mil->page_buffer_virt =  0;
	mil->page_buffer_phys = ~0;
	mil->page_buffer_size =  0;

	/* Initialize the MTD data structures. */
	mtd->priv  = nand;
	mtd->name  = "gpmi-nfc-main";
	mtd->owner = THIS_MODULE;
	nand->priv = this;

	/*
	 * Signal Control
	 */
	nand->cmd_ctrl = mil_cmd_ctrl;

	/*
	 * Chip Control
	 *
	 * We rely on the reference implementations of:
	 *     - cmdfunc
	 *     - waitfunc
	 */
	nand->dev_ready   = mil_dev_ready;
	nand->select_chip = mil_select_chip;

	/*
	 * Low-level I/O
	 *
	 * We don't support a 16-bit NAND Flash bus, so we don't implement
	 * read_word.
	 *
	 * We rely on the reference implentation of verify_buf.
	 */
	nand->read_byte = mil_read_byte;
	nand->read_buf  = mil_read_buf;
	nand->write_buf = mil_write_buf;

	/*
	 * ECC Control
	 *
	 * None of these functions are necessary for us:
	 *     - ecc.hwctl
	 *     - ecc.calculate
	 *     - ecc.correct
	 */

	/*
	 * ECC-aware I/O
	 *
	 * We rely on the reference implementations of:
	 *     - ecc.read_page_raw
	 *     - ecc.write_page_raw
	 */
	nand->ecc.read_page  = mil_ecc_read_page;
	nand->ecc.write_page = mil_ecc_write_page;

	/*
	 * High-level I/O
	 *
	 * We rely on the reference implementations of:
	 *     - write_page
	 *     - erase_cmd
	 */
	nand->ecc.read_oob  = mil_ecc_read_oob;
	nand->ecc.write_oob = mil_ecc_write_oob;

	/*
	 * Bad Block Management
	 *
	 * We rely on the reference implementations of:
	 *     - block_bad
	 *     - block_markbad
	 */
	nand->block_bad = mil_block_bad;
	nand->scan_bbt  = mil_scan_bbt;
	nand->init_size = gpmi_init_size;
	nand->badblock_pattern = &gpmi_bbt_descr;

	/*
	 * Error Recovery Functions
	 *
	 * We don't fill in the errstat function pointer because it's optional
	 * and we don't have a need for it.
	 */

	/*
	 * Set up NAND Flash options. Specifically:
	 *
	 *     - Disallow partial page writes.
	 */
	nand->options |= NAND_NO_SUBPAGE_WRITE;

	/*
	 * Tell the NAND Flash MTD system that we'll be handling ECC with our
	 * own hardware. It turns out that we still have to fill in the ECC size
	 * because the MTD code will divide by it -- even though it doesn't
	 * actually care.
	 */
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = 1;

	/* Allocate a temporary DMA buffer for reading ID in the nand_scan() */
	this->nfc_geometry.payload_size_in_bytes = 1024;
	this->nfc_geometry.auxiliary_size_in_bytes = 128;
	error = mil_alloc_dma_buffer(this);
	if (error)
		goto exit_dma_allocation;

	/*
	 * Ask the NAND Flash system to scan for chips.
	 *
	 * This will fill in reference implementations for all the members of
	 * the MTD structures that we didn't set, and will make the medium fully
	 * usable.
	 */
	pr_info("Scanning for NAND Flash chips...\n");
	error = nand_scan(mtd, pdata->max_chip_count);
	if (error) {
		dev_err(dev, "Chip scan failed\n");
		goto exit_nand_scan;
	}

	/*
	 * Hook some operations at the MTD level. See the descriptions of the
	 * saved function pointer fields for details about why we hook these.
	 */
	mil->hooked_block_markbad = mtd->block_markbad;
	mtd->block_markbad        = mil_hook_block_markbad;

	/* Construct partitions as necessary. */
	error = mil_partitions_init(this);
	if (error)
		goto exit_partitions;

	return 0;

exit_partitions:
	nand_release(&mil->mtd);
exit_nand_scan:
	mil_free_dma_buffer(this);
exit_dma_allocation:
	return error;
}

/**
 * gpmi_nfc_mil_exit() - Shuts down the MTD Interface Layer.
 *
 * @this:  Per-device data.
 */
void gpmi_nfc_mil_exit(struct gpmi_nfc_data *this)
{
	struct mil     *mil = &this->mil;

	/* Shut down partitions as necessary. */
	mil_partitions_exit(this);

	/* Get MTD to let go of our MTD. */
	nand_release(&mil->mtd);

	/* Free all the DMA buffer, if it's been allocated. */
	mil_free_dma_buffer(this);
}
