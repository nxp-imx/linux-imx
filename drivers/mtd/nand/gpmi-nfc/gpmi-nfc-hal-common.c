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

	/* Clear the interrupt. */
	nfc->clear_bch(this);

	/* Release the base level. */
	complete(&(nfc->bch_done));

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

	/* Acknowledge the DMA channel's interrupt. */
	mxs_dma_ack_irq(nfc->isr_dma_channel);

	/* Release the base level. */
	complete(&(nfc->dma_done));

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
	return 0;

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
	struct nfc_geometry       *geometry = &this->nfc_geometry;
	struct boot_rom_helper    *rom      =  this->rom;
	struct mtd_info		  *mtd	    = &this->mil.mtd;
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

	/* ECC chunks */
	geometry->ecc_chunk_size_in_bytes = 512;
	if (is_ddr_nand(&this->device_info))
		geometry->ecc_chunk_size_in_bytes = 1024;

	/* Compute the page size, include page and oob. */
	geometry->page_size_in_bytes = mtd->writesize + mtd->oobsize;

	/*
	 * Compute the total number of ECC chunks in a page. This includes the
	 * slightly larger chunk at the beginning of the page, which contains
	 * both data and metadata.
	 */
	geometry->ecc_chunk_count = mtd->writesize /
					  geometry->ecc_chunk_size_in_bytes;

	/*
	 * We use the same ECC strength for all chunks, including the first one.
	 * At this writing, we base our ECC strength choice entirely on the
	 * the physical page geometry. In the future, this should be changed to
	 * pay attention to the detailed device information we gathered earlier.
	 */
	geometry->ecc_strength = 0;

	switch (mtd->writesize) {
	case 2048:
		geometry->ecc_strength = 8;
		break;
	case 4096:
		switch (mtd->oobsize) {
		case 128:
			geometry->ecc_strength = 8;
			break;
		case 218:
			geometry->ecc_strength = 16;
			break;
		}
		break;
	case 8192:
		geometry->ecc_strength = 24;
		/*
		 * ONFI/TOGGLE nand needs GF14, so re-culculate DMA page size.
		 * The ONFI nand must do the reculation,
		 * else it will fail in DMA.
		 */
		if (is_ddr_nand(&this->device_info))
			geometry->page_size_in_bytes =
				mtd->writesize
				+ geometry->metadata_size_in_bytes +
				(geometry->ecc_strength * 14 * 8 /
					geometry->ecc_chunk_count);
		break;
	}

	if (!geometry->ecc_strength) {
		dev_err(dev, "Unsupported page geometry.\n");
		return !0;
	}

	/*
	 * The payload buffer contains the data area of a page. The ECC engine
	 * only needs what's required to hold the data.
	 */
	geometry->payload_size_in_bytes = mtd->writesize;

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

	/* ONFI nand needs GF14 */
	if (is_ddr_nand(&this->device_info))
		chunk_ecc_size_in_bits  = geometry->ecc_strength * 14;
	else
		chunk_ecc_size_in_bits  = geometry->ecc_strength * 13;

	chunk_total_size_in_bits =
			chunk_data_size_in_bits + chunk_ecc_size_in_bits;

	/* Compute the bit offset of the block mark within the physical page. */
	block_mark_bit_offset = mtd->writesize * 8;

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
					mtd->writesize, mtd->oobsize);
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

	return 0;
}

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

	/* Get ready... */
	nfc->isr_dma_channel = dma_channel;
	init_completion(&nfc->dma_done);
	mxs_dma_enable_irq(dma_channel, 1);

	/* Go! */
	mxs_dma_enable(dma_channel);

	/* Wait for it to finish. */
	timeout = wait_for_completion_timeout(&nfc->dma_done,
						msecs_to_jiffies(1000));
	error = (!timeout) ? -ETIMEDOUT : 0;
	if (error) {
		struct mxs_dma_info info;

		mxs_dma_get_info(dma_channel, &info);
		dev_err(dev, "[%s] Chip: %u, DMA Channel: %d, Error %d\n",
			__func__, dma_channel - resources->dma_low_channel,
			dma_channel, error);
	} else {
		/* Clear out the descriptors we just ran. */
		mxs_dma_cooked(dma_channel, &tmp_desc_list);
	}

	/* Shut the DMA channel down. */
	mxs_dma_reset(dma_channel);
	mxs_dma_enable_irq(dma_channel, 0);
	mxs_dma_disable(dma_channel);
	return error;
}

/*
 * This function is used in BCH reading or BCH writing.
 * It will wait for the BCH interrupt as long as one second.
 * Actually, we will wait for two interrupts, the DMA interrupt and
 * BCH interrupt.
 *
 * @this:        Per-device data structure.
 * @dma_channel: DMA channel
 *
 */
int start_dma_with_bch_irq(struct gpmi_nfc_data *this, int dma_channel)
{
	struct nfc_hal *nfc = this->nfc;
	int error = 0;

	/* Prepare to receive an interrupt from the BCH block. */
	init_completion(&nfc->bch_done);

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printk(KERN_ERR "[ %s ] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */
	error = wait_for_completion_timeout(&nfc->bch_done,
						msecs_to_jiffies(1000));
	error = (!error) ? -ETIMEDOUT : 0;
	if (error)
		printk(KERN_ERR "[ %s ] bch timeout!!!\n", __func__);
	return error;
}

/* This function is called in non-BCH DMA operations */
int start_dma_without_bch_irq(struct gpmi_nfc_data *this, int dma_channel)
{
	int error = 0;

	/* Go! */
	error = gpmi_nfc_dma_go(this, dma_channel);
	if (error)
		printk(KERN_ERR "[ %s ] DMA error\n", __func__);
	return error;
}

/**
 * ns_to_cycles - Converts time in nanoseconds to cycles.
 *
 * @ntime:   The time, in nanoseconds.
 * @period:  The cycle period, in nanoseconds.
 * @min:     The minimum allowable number of cycles.
 */
static unsigned int ns_to_cycles(unsigned int time,
					unsigned int period, unsigned int min)
{
	unsigned int k;

	/*
	 * Compute the minimum number of cycles that entirely contain the
	 * given time.
	 */
	k = (time + period - 1) / period;
	return max(k, min);
}

/**
 * gpmi_compute_hardware_timing - Apply timing to current hardware conditions.
 *
 * @this:             Per-device data.
 * @hardware_timing:  A pointer to a hardware timing structure that will receive
 *                    the results of our calculations.
 */
int gpmi_nfc_compute_hardware_timing(struct gpmi_nfc_data *this,
					struct gpmi_nfc_hardware_timing *hw)
{
	struct gpmi_nfc_platform_data  *pdata    =  this->pdata;
	struct nfc_hal                 *nfc      =  this->nfc;
	struct nand_chip		*nand	= &this->mil.nand;
	struct gpmi_nfc_timing         target    = nfc->timing;
	bool           improved_timing_is_available;
	unsigned long  clock_frequency_in_hz;
	unsigned int   clock_period_in_ns;
	bool           dll_use_half_periods;
	unsigned int   dll_delay_shift;
	unsigned int   max_sample_delay_in_ns;
	unsigned int   address_setup_in_cycles;
	unsigned int   data_setup_in_ns;
	unsigned int   data_setup_in_cycles;
	unsigned int   data_hold_in_cycles;
	int            ideal_sample_delay_in_ns;
	unsigned int   sample_delay_factor;
	int            tEYE;
	unsigned int   min_prop_delay_in_ns = pdata->min_prop_delay_in_ns;
	unsigned int   max_prop_delay_in_ns = pdata->max_prop_delay_in_ns;

	/*
	 * If there are multiple chips, we need to relax the timings to allow
	 * for signal distortion due to higher capacitance.
	 */
	if (nand->numchips > 2) {
		target.data_setup_in_ns    += 10;
		target.data_hold_in_ns     += 10;
		target.address_setup_in_ns += 10;
	} else if (nand->numchips > 1) {
		target.data_setup_in_ns    += 5;
		target.data_hold_in_ns     += 5;
		target.address_setup_in_ns += 5;
	}

	/* Check if improved timing information is available. */
	improved_timing_is_available =
		(target.tREA_in_ns  >= 0) &&
		(target.tRLOH_in_ns >= 0) &&
		(target.tRHOH_in_ns >= 0) ;

	/* Inspect the clock. */
	clock_frequency_in_hz = nfc->clock_frequency_in_hz;
	clock_period_in_ns    = 1000000000 / clock_frequency_in_hz;

	/*
	 * The NFC quantizes setup and hold parameters in terms of clock cycles.
	 * Here, we quantize the setup and hold timing parameters to the
	 * next-highest clock period to make sure we apply at least the
	 * specified times.
	 *
	 * For data setup and data hold, the hardware interprets a value of zero
	 * as the largest possible delay. This is not what's intended by a zero
	 * in the input parameter, so we impose a minimum of one cycle.
	 */
	data_setup_in_cycles    = ns_to_cycles(target.data_setup_in_ns,
							clock_period_in_ns, 1);
	data_hold_in_cycles     = ns_to_cycles(target.data_hold_in_ns,
							clock_period_in_ns, 1);
	address_setup_in_cycles = ns_to_cycles(target.address_setup_in_ns,
							clock_period_in_ns, 0);

	/*
	 * The clock's period affects the sample delay in a number of ways:
	 *
	 * (1) The NFC HAL tells us the maximum clock period the sample delay
	 *     DLL can tolerate. If the clock period is greater than half that
	 *     maximum, we must configure the DLL to be driven by half periods.
	 *
	 * (2) We need to convert from an ideal sample delay, in ns, to a
	 *     "sample delay factor," which the NFC uses. This factor depends on
	 *     whether we're driving the DLL with full or half periods.
	 *     Paraphrasing the reference manual:
	 *
	 *         AD = SDF x 0.125 x RP
	 *
	 * where:
	 *
	 *     AD   is the applied delay, in ns.
	 *     SDF  is the sample delay factor, which is dimensionless.
	 *     RP   is the reference period, in ns, which is a full clock period
	 *          if the DLL is being driven by full periods, or half that if
	 *          the DLL is being driven by half periods.
	 *
	 * Let's re-arrange this in a way that's more useful to us:
	 *
	 *                        8
	 *         SDF  =  AD x ----
	 *                       RP
	 *
	 * The reference period is either the clock period or half that, so this
	 * is:
	 *
	 *                        8       AD x DDF
	 *         SDF  =  AD x -----  =  --------
	 *                      f x P        P
	 *
	 * where:
	 *
	 *       f  is 1 or 1/2, depending on how we're driving the DLL.
	 *       P  is the clock period.
	 *     DDF  is the DLL Delay Factor, a dimensionless value that
	 *          incorporates all the constants in the conversion.
	 *
	 * DDF will be either 8 or 16, both of which are powers of two. We can
	 * reduce the cost of this conversion by using bit shifts instead of
	 * multiplication or division. Thus:
	 *
	 *                 AD << DDS
	 *         SDF  =  ---------
	 *                     P
	 *
	 *     or
	 *
	 *         AD  =  (SDF >> DDS) x P
	 *
	 * where:
	 *
	 *     DDS  is the DLL Delay Shift, the logarithm to base 2 of the DDF.
	 */
	if (clock_period_in_ns > (nfc->max_dll_clock_period_in_ns >> 1)) {
		dll_use_half_periods = true;
		dll_delay_shift      = 3 + 1;
	} else {
		dll_use_half_periods = false;
		dll_delay_shift      = 3;
	}

	/*
	 * Compute the maximum sample delay the NFC allows, under current
	 * conditions. If the clock is running too slowly, no sample delay is
	 * possible.
	 */
	if (clock_period_in_ns > nfc->max_dll_clock_period_in_ns)
		max_sample_delay_in_ns = 0;
	else {

		/*
		 * Compute the delay implied by the largest sample delay factor
		 * the NFC allows.
		 */

		max_sample_delay_in_ns =
			(nfc->max_sample_delay_factor * clock_period_in_ns) >>
								dll_delay_shift;

		/*
		 * Check if the implied sample delay larger than the NFC
		 * actually allows.
		 */

		if (max_sample_delay_in_ns > nfc->max_dll_delay_in_ns)
			max_sample_delay_in_ns = nfc->max_dll_delay_in_ns;

	}

	/*
	 * Check if improved timing information is available. If not, we have to
	 * use a less-sophisticated algorithm.
	 */

	if (!improved_timing_is_available) {

		/*
		 * Fold the read setup time required by the NFC into the ideal
		 * sample delay.
		 */

		ideal_sample_delay_in_ns = target.gpmi_sample_delay_in_ns +
						nfc->internal_data_setup_in_ns;

		/*
		 * The ideal sample delay may be greater than the maximum
		 * allowed by the NFC. If so, we can trade off sample delay time
		 * for more data setup time.
		 *
		 * In each iteration of the following loop, we add a cycle to
		 * the data setup time and subtract a corresponding amount from
		 * the sample delay until we've satisified the constraints or
		 * can't do any better.
		 */

		while ((ideal_sample_delay_in_ns > max_sample_delay_in_ns) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {

			data_setup_in_cycles++;
			ideal_sample_delay_in_ns -= clock_period_in_ns;

			if (ideal_sample_delay_in_ns < 0)
				ideal_sample_delay_in_ns = 0;

		}

		/*
		 * Compute the sample delay factor that corresponds most closely
		 * to the ideal sample delay. If the result is too large for the
		 * NFC, use the maximum value.
		 *
		 * Notice that we use the ns_to_cycles function to compute the
		 * sample delay factor. We do this because the form of the
		 * computation is the same as that for calculating cycles.
		 */

		sample_delay_factor =
			ns_to_cycles(
				ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

		if (sample_delay_factor > nfc->max_sample_delay_factor)
			sample_delay_factor = nfc->max_sample_delay_factor;

		/* Skip to the part where we return our results. */

		goto return_results;

	}

	/*
	 * If control arrives here, we have more detailed timing information,
	 * so we can use a better algorithm.
	 */

	/*
	 * Fold the read setup time required by the NFC into the maximum
	 * propagation delay.
	 */

	max_prop_delay_in_ns += nfc->internal_data_setup_in_ns;

	/*
	 * Earlier, we computed the number of clock cycles required to satisfy
	 * the data setup time. Now, we need to know the actual nanoseconds.
	 */

	data_setup_in_ns = clock_period_in_ns * data_setup_in_cycles;

	/*
	 * Compute tEYE, the width of the data eye when reading from the NAND
	 * Flash. The eye width is fundamentally determined by the data setup
	 * time, perturbed by propagation delays and some characteristics of the
	 * NAND Flash device.
	 *
	 * start of the eye = max_prop_delay + tREA
	 * end of the eye   = min_prop_delay + tRHOH + data_setup
	 */

	tEYE = (int)min_prop_delay_in_ns + (int)target.tRHOH_in_ns +
							(int)data_setup_in_ns;

	tEYE -= (int)max_prop_delay_in_ns + (int)target.tREA_in_ns;

	/*
	 * The eye must be open. If it's not, we can try to open it by
	 * increasing its main forcer, the data setup time.
	 *
	 * In each iteration of the following loop, we increase the data setup
	 * time by a single clock cycle. We do this until either the eye is
	 * open or we run into NFC limits.
	 */

	while ((tEYE <= 0) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {
		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;
	}

	/*
	 * When control arrives here, the eye is open. The ideal time to sample
	 * the data is in the center of the eye:
	 *
	 *     end of the eye + start of the eye
	 *     ---------------------------------  -  data_setup
	 *                    2
	 *
	 * After some algebra, this simplifies to the code immediately below.
	 */

	ideal_sample_delay_in_ns =
		((int)max_prop_delay_in_ns +
			(int)target.tREA_in_ns +
				(int)min_prop_delay_in_ns +
					(int)target.tRHOH_in_ns -
						(int)data_setup_in_ns) >> 1;

	/*
	 * The following figure illustrates some aspects of a NAND Flash read:
	 *
	 *
	 *           __                   _____________________________________
	 * RDN         \_________________/
	 *
	 *                                         <---- tEYE ----->
	 *                                        /-----------------\
	 * Read Data ----------------------------<                   >---------
	 *                                        \-----------------/
	 *             ^                 ^                 ^              ^
	 *             |                 |                 |              |
	 *             |<--Data Setup -->|<--Delay Time -->|              |
	 *             |                 |                 |              |
	 *             |                 |                                |
	 *             |                 |<--   Quantized Delay Time   -->|
	 *             |                 |                                |
	 *
	 *
	 * We have some issues we must now address:
	 *
	 * (1) The *ideal* sample delay time must not be negative. If it is, we
	 *     jam it to zero.
	 *
	 * (2) The *ideal* sample delay time must not be greater than that
	 *     allowed by the NFC. If it is, we can increase the data setup
	 *     time, which will reduce the delay between the end of the data
	 *     setup and the center of the eye. It will also make the eye
	 *     larger, which might help with the next issue...
	 *
	 * (3) The *quantized* sample delay time must not fall either before the
	 *     eye opens or after it closes (the latter is the problem
	 *     illustrated in the above figure).
	 */

	/* Jam a negative ideal sample delay to zero. */

	if (ideal_sample_delay_in_ns < 0)
		ideal_sample_delay_in_ns = 0;

	/*
	 * Extend the data setup as needed to reduce the ideal sample delay
	 * below the maximum permitted by the NFC.
	 */

	while ((ideal_sample_delay_in_ns > max_sample_delay_in_ns) &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {

		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;

		/*
		 * Decrease the ideal sample delay by one half cycle, to keep it
		 * in the middle of the eye.
		 */
		ideal_sample_delay_in_ns -= (clock_period_in_ns >> 1);

		/* Jam a negative ideal sample delay to zero. */
		if (ideal_sample_delay_in_ns < 0)
			ideal_sample_delay_in_ns = 0;

	}

	/*
	 * Compute the sample delay factor that corresponds to the ideal sample
	 * delay. If the result is too large, then use the maximum allowed
	 * value.
	 *
	 * Notice that we use the ns_to_cycles function to compute the sample
	 * delay factor. We do this because the form of the computation is the
	 * same as that for calculating cycles.
	 */

	sample_delay_factor =
		ns_to_cycles(ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

	if (sample_delay_factor > nfc->max_sample_delay_factor)
		sample_delay_factor = nfc->max_sample_delay_factor;

	/*
	 * These macros conveniently encapsulate a computation we'll use to
	 * continuously evaluate whether or not the data sample delay is inside
	 * the eye.
	 */

	#define IDEAL_DELAY  ((int) ideal_sample_delay_in_ns)

	#define QUANTIZED_DELAY  \
		((int) ((sample_delay_factor * clock_period_in_ns) >> \
							dll_delay_shift))

	#define DELAY_ERROR  (abs(QUANTIZED_DELAY - IDEAL_DELAY))

	#define SAMPLE_IS_NOT_WITHIN_THE_EYE  (DELAY_ERROR > (tEYE >> 1))

	/*
	 * While the quantized sample time falls outside the eye, reduce the
	 * sample delay or extend the data setup to move the sampling point back
	 * toward the eye. Do not allow the number of data setup cycles to
	 * exceed the maximum allowed by the NFC.
	 */

	while (SAMPLE_IS_NOT_WITHIN_THE_EYE &&
			(data_setup_in_cycles < nfc->max_data_setup_cycles)) {

		/*
		 * If control arrives here, the quantized sample delay falls
		 * outside the eye. Check if it's before the eye opens, or after
		 * the eye closes.
		 */

		if (QUANTIZED_DELAY > IDEAL_DELAY) {

			/*
			 * If control arrives here, the quantized sample delay
			 * falls after the eye closes. Decrease the quantized
			 * delay time and then go back to re-evaluate.
			 */

			if (sample_delay_factor != 0)
				sample_delay_factor--;

			continue;

		}

		/*
		 * If control arrives here, the quantized sample delay falls
		 * before the eye opens. Shift the sample point by increasing
		 * data setup time. This will also make the eye larger.
		 */

		/* Give a cycle to data setup. */
		data_setup_in_cycles++;
		/* Synchronize the data setup time with the cycles. */
		data_setup_in_ns += clock_period_in_ns;
		/* Adjust tEYE accordingly. */
		tEYE += clock_period_in_ns;

		/*
		 * Decrease the ideal sample delay by one half cycle, to keep it
		 * in the middle of the eye.
		 */
		ideal_sample_delay_in_ns -= (clock_period_in_ns >> 1);

		/* ...and one less period for the delay time. */
		ideal_sample_delay_in_ns -= clock_period_in_ns;

		/* Jam a negative ideal sample delay to zero. */
		if (ideal_sample_delay_in_ns < 0)
			ideal_sample_delay_in_ns = 0;

		/*
		 * We have a new ideal sample delay, so re-compute the quantized
		 * delay.
		 */

		sample_delay_factor =
			ns_to_cycles(
				ideal_sample_delay_in_ns << dll_delay_shift,
							clock_period_in_ns, 0);

		if (sample_delay_factor > nfc->max_sample_delay_factor)
			sample_delay_factor = nfc->max_sample_delay_factor;

	}

	/* Control arrives here when we're ready to return our results. */

return_results:
	hw->data_setup_in_cycles    = data_setup_in_cycles;
	hw->data_hold_in_cycles     = data_hold_in_cycles;
	hw->address_setup_in_cycles = address_setup_in_cycles;
	hw->use_half_periods        = dll_use_half_periods;
	hw->sample_delay_factor     = sample_delay_factor;

	/* Return success. */
	return 0;
}
