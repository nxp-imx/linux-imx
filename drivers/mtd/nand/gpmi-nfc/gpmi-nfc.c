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

/* Linux header files. */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>
#include <linux/gpmi-nfc.h>

/* Platform header files. */

#include <mach/system.h>
#include <mach/dmaengine.h>
#include <mach/device.h>

/* Driver header files. */

#include "gpmi-nfc.h"

/* Define this macro to enable detailed information messages. */

#define DETAILED_INFO

/* Define this macro to enable event reporting. */

/*#define EVENT_REPORTING*/

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

#ifdef EVENT_REPORTING

/*
 * This variable and module parameter controls whether the driver reports event
 * information by printing to the console.
 */

static int report_events;
module_param(report_events, int, 0600);

/**
 * struct event - A single record in the event trace.
 *
 * @time:         The time at which the event occurred.
 * @nesting:      Indicates function call nesting.
 * @description:  A description of the event.
 */

struct event {
	ktime_t       time;
	unsigned int  nesting;
	char          *description;
};

/**
 * The event trace.
 *
 * @overhead:  The delay to take a time stamp and nothing else.
 * @nesting:   The current nesting level.
 * @overflow:  Indicates the trace overflowed.
 * @next:      Index of the next event to write.
 * @events:    The array of events.
 */

#define MAX_EVENT_COUNT  (200)

static struct {
	ktime_t       overhead;
	int           nesting;
	int           overflow;
	unsigned int  next;
	struct event  events[MAX_EVENT_COUNT];
} event_trace;

/**
 * reset_event_trace() - Resets the event trace.
 */
static void reset_event_trace(void)
{
	event_trace.nesting  = 0;
	event_trace.overflow = false;
	event_trace.next     = 0;
}

/**
 * add_event() - Adds an event to the event trace.
 *
 * @description:  A description of the event.
 * @delta:        A delta to the nesting level for this event [-1, 0, 1].
 */
static inline void add_event(char *description, int delta)
{
	struct event  *event;

	if (!report_events)
		return;

	if (event_trace.overflow)
		return;

	if (event_trace.next >= MAX_EVENT_COUNT) {
		event_trace.overflow = true;
		return;
	}

	event = event_trace.events + event_trace.next;

	event->time = ktime_get();

	event->description = description;

	if (!delta)
		event->nesting = event_trace.nesting;
	else if (delta < 0) {
		event->nesting = event_trace.nesting - 1;
		event_trace.nesting -= 2;
	} else {
		event->nesting = event_trace.nesting + 1;
		event_trace.nesting += 2;
	}

	if (event_trace.nesting < 0)
		event_trace.nesting = 0;

	event_trace.next++;

}

/**
 * start_event_trace() - Starts an event trace and adds the first event.
 *
 * @description:  A description of the first event.
 */
static void start_event_trace(char *description)
{

	ktime_t  t0;
	ktime_t  t1;

	if (!report_events)
		return;

	reset_event_trace();

	t0 = ktime_get();
	t1 = ktime_get();

	event_trace.overhead = ktime_sub(t1, t0);

	add_event(description, 1);

}

/**
 * dump_event_trace() - Dumps the event trace.
 */
static void dump_event_trace(void)
{
	unsigned int  i;
	time_t        seconds;
	long          nanoseconds;
	char          line[100];
	int           o;
	struct event  *first_event;
	struct event  *last_event;
	struct event  *matching_event;
	struct event  *event;
	ktime_t       delta;

	/* Check if event reporting is turned off. */

	if (!report_events)
		return;

	/* Print important facts about this event trace. */

	printk(KERN_DEBUG "\n+--------------\n");

	printk(KERN_DEBUG "|  Overhead    : [%d:%d]\n",
				event_trace.overhead.tv.sec,
				event_trace.overhead.tv.nsec);

	if (!event_trace.next) {
		printk(KERN_DEBUG "|  No Events\n");
		return;
	}

	first_event = event_trace.events;
	last_event  = event_trace.events + (event_trace.next - 1);

	delta = ktime_sub(last_event->time, first_event->time);
	printk(KERN_DEBUG "|  Elapsed Time: [%d:%d]\n",
						delta.tv.sec, delta.tv.nsec);

	if (event_trace.overflow)
		printk(KERN_DEBUG "|  Overflow!\n");

	/* Print the events in this history. */

	for (i = 0, event = event_trace.events;
					i < event_trace.next; i++, event++) {

		/* Get the delta between this event and the previous event. */

		if (!i) {
			seconds     = 0;
			nanoseconds = 0;
		} else {
			delta = ktime_sub(event[0].time, event[-1].time);
			seconds     = delta.tv.sec;
			nanoseconds = delta.tv.nsec;
		}

		/* Print the current event. */

		o = 0;

		o = snprintf(line, sizeof(line) - o, "|  [%ld:% 10ld]%*s %s",
							seconds, nanoseconds,
							event->nesting, "",
							event->description);
		/* Check if this is the last event in a nested series. */

		if (i && (event[0].nesting < event[-1].nesting)) {

			for (matching_event = event - 1;; matching_event--) {

				if (matching_event < event_trace.events) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(event->time,
							matching_event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" <%d:%d]", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		/* Check if this is the first event in a nested series. */

		if ((i < event_trace.next - 1) &&
				(event[0].nesting < event[1].nesting)) {

			for (matching_event = event + 1;; matching_event++) {

				if (matching_event >=
					(event_trace.events+event_trace.next)) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(matching_event->time,
								event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" [%d:%d>", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		printk(KERN_DEBUG "%s\n", line);

	}

	printk(KERN_DEBUG "+--------------\n");

}

/**
 * stop_event_trace() - Stops an event trace.
 *
 * @description:  A description of the last event.
 */
static void stop_event_trace(char *description)
{
	struct event  *event;

	if (!report_events)
		return;

	/*
	 * We want the end of the trace, no matter what happens. If the trace
	 * has already overflowed, or is about to, just jam this event into the
	 * last spot. Otherwise, add this event like any other.
	 */

	if (event_trace.overflow || (event_trace.next >= MAX_EVENT_COUNT)) {
		event = event_trace.events + (MAX_EVENT_COUNT - 1);
		event->time = ktime_get();
		event->description = description;
		event->nesting     = 0;
	} else {
		add_event(description, -1);
	}

	dump_event_trace();
	reset_event_trace();

}

#else /* EVENT_REPORTING */

#define start_event_trace(description)                   do {} while (0)
#define add_event(description, delta)                    do {} while (0)
#define add_state_event_l(address, mask, zero, not_zero) do {} while (0)
#define stop_event_trace(description)                    do {} while (0)
#define dump_event_trace()                               do {} while (0)

#endif /* EVENT_REPORTING */

/*
 *------------------------------------------------------------------------------
 * NFC HAL
 *
 * The following functions implent the NFC HAL for various NFC hardware
 * versions.
 *------------------------------------------------------------------------------
 */

/**
 * nfc_bch_isr - BCH interrupt service routine.
 *
 * @interrupt_number:  The interrupt number.
 * @cookie:            A cookie that contains a pointer to the owning device
 *                     data structure.
 */
static irqreturn_t nfc_bch_isr(int irq, void *cookie)
{
	struct gpmi_nfc_data  *this      = cookie;
	struct resources      *resources = &this->resources;
	struct nfc_hal        *nfc       =  this->nfc;

	/* Clear the interrupt. */

	__raw_writel(V1_BM_BCH_CTRL_COMPLETE_IRQ,
				resources->bch_regs + V1_HW_BCH_CTRL_CLR);

	/* Release the base level. */

	complete(&(nfc->bch_done));

	/* Return success. */

	return IRQ_HANDLED;

}

/**
 * nfc_dma_isr - DMA interrupt service routine.
 *
 * @interrupt_number:  The interrupt number.
 * @cookie:            A cookie that contains a pointer to the owning device
 *                     data structure.
 */
static irqreturn_t nfc_dma_isr(int irq, void *cookie)
{
	struct gpmi_nfc_data  *this = cookie;
	struct nfc_hal        *nfc  =  this->nfc;

	add_event("=> nfc_dma_isr", 1);

	/* Acknowledge the DMA channel's interrupt. */

	mxs_dma_ack_irq(nfc->isr_dma_channel);

	/* Release the base level. */

	complete(&(nfc->dma_done));

	/* Return success. */

	add_event("<= nfc_dma_isr", -1);

	return IRQ_HANDLED;

}

/**
 * nfc_dma_init() - Initializes DMA.
 *
 * @this:  Per-device data.
 */
static int nfc_dma_init(struct gpmi_nfc_data *this)
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
 * nfc_dma_exit() - Shuts down DMA.
 *
 * @this:  Per-device data.
 */
static void nfc_dma_exit(struct gpmi_nfc_data *this)
{
	struct nfc_hal  *nfc = this->nfc;
	int             i;

	/* Free the DMA descriptors. */

	for (i = 0; i < NFC_DMA_DESCRIPTOR_COUNT; i++)
		mxs_dma_free_desc(nfc->dma_descriptors[i]);

}

/**
 * nfc_set_geometry() - Shared NFC geometry configuration.
 *
 * In principle, computing the NFC geometry is version-specific. However, at
 * this writing all, versions share the same page model, so this code can also
 * be shared.
 *
 * @this:  Per-device data.
 */
static int nfc_set_geometry(struct gpmi_nfc_data *this)
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

static void dump_dma_context(struct gpmi_nfc_data *this, char *title)
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
 * nfc_dma_go - Run a DMA channel.
 *
 * @this:         Per-device data structure.
 * @dma_channel:  The DMA channel we're going to use.
 */
static int nfc_dma_go(struct gpmi_nfc_data *this, int  dma_channel)
{
	struct device     *dev       =  this->dev;
	struct resources  *resources = &this->resources;
	struct nfc_hal    *nfc       =  this->nfc;
	unsigned long     timeout;
	int               error;
	LIST_HEAD(tmp_desc_list);

	add_event("=> nfc_dma_go", 1);

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
		add_event("...DMA timed out", 0);
	} else
		add_event("...Finished DMA successfully", 0);

	/* Clear out the descriptors we just ran. */

	mxs_dma_cooked(dma_channel, &tmp_desc_list);

	/* Shut the DMA channel down. */

	mxs_dma_reset(dma_channel);
	mxs_dma_enable_irq(dma_channel, 0);
	mxs_dma_disable(dma_channel);

	/* Return. */

	add_event("<= nfc_dma_go", -1);

	return error;

}

/**
 * nfc_v0_init() - Initializes the NFC hardware.
 *
 * @this:  Per-device data.
 */
static int nfc_v0_init(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;
	int               error;

	/* Initialize DMA. */

	error = nfc_dma_init(this);

	if (error)
		return error;

	/* Enable the clock. */

	clk_enable(resources->clock);

	/* Reset the GPMI block. */

	mxs_reset_block(resources->gpmi_regs + V0_HW_GPMI_CTRL0, true);

	/* Choose NAND mode. */
	__raw_writel(V0_BM_GPMI_CTRL1_GPMI_MODE,
				resources->gpmi_regs + V0_HW_GPMI_CTRL1_CLR);

	/* Set the IRQ polarity. */
	__raw_writel(V0_BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY,
				resources->gpmi_regs + V0_HW_GPMI_CTRL1_SET);

	/* Disable write protection. */
	__raw_writel(V0_BM_GPMI_CTRL1_DEV_RESET,
				resources->gpmi_regs + V0_HW_GPMI_CTRL1_SET);

	/* Select BCH ECC. */
	__raw_writel(V0_BM_GPMI_CTRL1_BCH_MODE,
				resources->gpmi_regs + V0_HW_GPMI_CTRL1_SET);

	/* If control arrives here, all is well. */

	return 0;

}

/**
 * nfc_v0_set_geometry() - Configures the NFC geometry.
 *
 * @this:  Per-device data.
 */
static int nfc_v0_set_geometry(struct gpmi_nfc_data *this)
{
	struct resources     *resources = &this->resources;
	struct nfc_geometry  *nfc       = &this->nfc_geometry;
	unsigned int         block_count;
	unsigned int         block_size;
	unsigned int         metadata_size;
	unsigned int         ecc_strength;
	unsigned int         page_size;

	/* We make the abstract choices in a common function. */

	if (nfc_set_geometry(this))
		return !0;

	/* Translate the abstract choices into register fields. */

	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes;
	metadata_size = nfc->metadata_size_in_bytes;
	ecc_strength  = nfc->ecc_strength >> 1;
	page_size     = nfc->page_size_in_bytes;

	/*
	 * Reset the BCH block. Notice that we pass in true for the just_enable
	 * flag. This is because the soft reset for the version 0 BCH block
	 * doesn't work. If you try to soft reset the BCH block, it becomes
	 * unusable until the next hard reset.
	 */

	mxs_reset_block(resources->bch_regs, true);

	/* Configure layout 0. */

	__raw_writel(
		V0_BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)     |
		V0_BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size) |
		V0_BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)       |
		V0_BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size)   ,
		resources->bch_regs + V0_HW_BCH_FLASH0LAYOUT0);

	__raw_writel(
		V0_BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)   |
		V0_BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)     |
		V0_BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size) ,
		resources->bch_regs + V0_HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */

	__raw_writel(0, resources->bch_regs + V0_HW_BCH_LAYOUTSELECT);

	/* Enable interrupts. */

	__raw_writel(V0_BM_BCH_CTRL_COMPLETE_IRQ_EN,
				resources->bch_regs + V0_HW_BCH_CTRL_SET);

	/* Return success. */

	return 0;

}

/**
 * nfc_v0_exit() - Shuts down the NFC hardware.
 *
 * @this:  Per-device data.
 */
static void nfc_v0_exit(struct gpmi_nfc_data *this)
{
	nfc_dma_exit(this);
}

/**
 * nfc_v0_is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int nfc_v0_is_ready(struct gpmi_nfc_data *this, unsigned chip)
{
	struct resources  *resources = &this->resources;
	uint32_t          mask;
	uint32_t          register_image;

	/* Extract and return the status. */

	mask = V0_BM_GPMI_DEBUG_READY0 << chip;

	register_image = __raw_readl(resources->gpmi_regs + V0_HW_GPMI_DEBUG);

	return !!(register_image & mask);

}

/**
 * nfc_v0_send_command() - Sends a command and associated addresses.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the command bytes.
 * @length:  The number of bytes in the buffer.
 */
static int nfc_v0_send_command(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_CLE;

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
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		V0_BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v0_send_data() - Sends data to the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the data.
 * @length:  The number of bytes in the buffer.
 */
static int nfc_v0_send_data(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BF_GPMI_CTRL0_XFER_COUNT(length)         ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v0_read_data() - Receives data from the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that will receive the data.
 * @length:  The number of bytes to read.
 */
static int nfc_v0_read_data(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BF_GPMI_CTRL0_XFER_COUNT(length)         ;

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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v0_send_page() - Sends a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int nfc_v0_send_page(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = V0_BV_GPMI_ECCCTRL_ECC_CMD__BCH_ENCODE;
	buffer_mask  = V0_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				V0_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		V0_BM_GPMI_ECCCTRL_ENABLE_ECC               |
		V0_BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		V0_BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Prepare to receive an interrupt from the BCH block. */

	init_completion(&nfc->bch_done);

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/**
 * nfc_v0_read_page() - Reads a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int nfc_v0_read_page(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V0_BM_GPMI_CTRL0_LOCK_CS                    |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                |
		V0_BF_GPMI_CTRL0_CS(chip)                   |
		V0_BF_GPMI_CTRL0_ADDRESS(address)           |
		V0_BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Enable the BCH block and read. */

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = V0_BV_GPMI_ECCCTRL_ECC_CMD__BCH_DECODE;
	buffer_mask  = V0_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				V0_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		V0_BM_GPMI_CTRL0_LOCK_CS                                 |
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                             |
		V0_BF_GPMI_CTRL0_CS(chip)                                |
		V0_BF_GPMI_CTRL0_ADDRESS(address)                        |
		V0_BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		V0_BM_GPMI_ECCCTRL_ENABLE_ECC 	            |
		V0_BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		V0_BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;
	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Disable the BCH block */

	command_mode = V0_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V0_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V0_BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		V0_BM_GPMI_CTRL0_LOCK_CS                                 |
		V0_BM_GPMI_CTRL0_WORD_LENGTH                             |
		V0_BF_GPMI_CTRL0_CS(chip)                                |
		V0_BF_GPMI_CTRL0_ADDRESS(address)                        |
		V0_BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

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

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/**
 * nfc_v1_init() - Initializes the NFC hardware.
 *
 * @this:  Per-device data.
 */
static int nfc_v1_init(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;
	int               error;

	/* Initialize DMA. */

	error = nfc_dma_init(this);

	if (error)
		return error;

	/* Enable the clock. */

	clk_enable(resources->clock);

	/* Reset the GPMI block. */

	mxs_reset_block(resources->gpmi_regs + V1_HW_GPMI_CTRL0, true);

	/* Choose NAND mode. */
	__raw_writel(V1_BM_GPMI_CTRL1_GPMI_MODE,
				resources->gpmi_regs + V1_HW_GPMI_CTRL1_CLR);

	/* Set the IRQ polarity. */
	__raw_writel(V1_BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY,
				resources->gpmi_regs + V1_HW_GPMI_CTRL1_SET);

	/* Disable write protection. */
	__raw_writel(V1_BM_GPMI_CTRL1_DEV_RESET,
				resources->gpmi_regs + V1_HW_GPMI_CTRL1_SET);

	/* Select BCH ECC. */
	__raw_writel(V1_BM_GPMI_CTRL1_BCH_MODE,
				resources->gpmi_regs + V1_HW_GPMI_CTRL1_SET);

	/* If control arrives here, all is well. */

	return 0;

}

/**
 * nfc_v1_set_geometry() - Configures the NFC geometry.
 *
 * @this:  Per-device data.
 */
static int nfc_v1_set_geometry(struct gpmi_nfc_data *this)
{
	struct resources     *resources = &this->resources;
	struct nfc_geometry  *nfc       = &this->nfc_geometry;
	unsigned int         block_count;
	unsigned int         block_size;
	unsigned int         metadata_size;
	unsigned int         ecc_strength;
	unsigned int         page_size;

	/* We make the abstract choices in a common function. */

	if (nfc_set_geometry(this))
		return !0;

	/* Translate the abstract choices into register fields. */

	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes;
	metadata_size = nfc->metadata_size_in_bytes;
	ecc_strength  = nfc->ecc_strength >> 1;
	page_size     = nfc->page_size_in_bytes;

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
		V1_BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)     |
		V1_BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size) |
		V1_BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)       |
		V1_BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size)   ,
		resources->bch_regs + V1_HW_BCH_FLASH0LAYOUT0);

	__raw_writel(
		V1_BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)   |
		V1_BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)     |
		V1_BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size) ,
		resources->bch_regs + V1_HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */

	__raw_writel(0, resources->bch_regs + V1_HW_BCH_LAYOUTSELECT);

	/* Enable interrupts. */

	__raw_writel(V1_BM_BCH_CTRL_COMPLETE_IRQ_EN,
				resources->bch_regs + V1_HW_BCH_CTRL_SET);

	/* Return success. */

	return 0;

}

/**
 * nfc_v1_exit() - Shuts down the NFC hardware.
 *
 * @this:  Per-device data.
 */
static void nfc_v1_exit(struct gpmi_nfc_data *this)
{
	nfc_dma_exit(this);
}

/**
 * nfc_v1_is_ready() - Returns the ready/busy status of the given chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip of interest.
 */
static int nfc_v1_is_ready(struct gpmi_nfc_data *this, unsigned chip)
{
	struct resources  *resources = &this->resources;
	uint32_t          mask;
	uint32_t          register_image;

	/* Extract and return the status. */

	mask = V1_BF_GPMI_STAT_READY_BUSY(1 << chip);

	register_image = __raw_readl(resources->gpmi_regs + V1_HW_GPMI_STAT);

	return !!(register_image & mask);

}

/**
 * nfc_v1_send_command() - Sends a command and associated addresses.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the command bytes.
 * @length:  The number of bytes in the buffer.
 */
static int nfc_v1_send_command(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_CLE;

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
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		V1_BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v1_send_data() - Sends data to the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that contains the data.
 * @length:  The number of bytes in the buffer.
 */
static int nfc_v1_send_data(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BF_GPMI_CTRL0_XFER_COUNT(length)         ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v1_read_data() - Receives data from the given chip.
 *
 * @this:    Per-device data.
 * @chip:    The chip of interest.
 * @buffer:  The physical address of a buffer that will receive the data.
 * @length:  The number of bytes to read.
 */
static int nfc_v1_read_data(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BF_GPMI_CTRL0_XFER_COUNT(length)         ;

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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;
	(*d)->cmd.pio_words[3] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Return success. */

	return error;

}

/**
 * nfc_v1_send_page() - Sends a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int nfc_v1_send_page(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = V1_BV_GPMI_ECCCTRL_ECC_CMD__ENCODE;
	buffer_mask  = V1_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				V1_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		V1_BM_GPMI_ECCCTRL_ENABLE_ECC               |
		V1_BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		V1_BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Prepare to receive an interrupt from the BCH block. */

	init_completion(&nfc->bch_done);

	/* Go! */

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/**
 * nfc_v1_read_page() - Reads a page, using ECC.
 *
 * @this:       Per-device data.
 * @chip:       The chip of interest.
 * @payload:    The physical address of the payload buffer.
 * @auxiliary:  The physical address of the auxiliary buffer.
 */
static int nfc_v1_read_page(struct gpmi_nfc_data *this, unsigned chip,
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

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		V1_BM_GPMI_CTRL0_LOCK_CS                    |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                |
		V1_BF_GPMI_CTRL0_CS(chip)                   |
		V1_BF_GPMI_CTRL0_ADDRESS(address)           |
		V1_BF_GPMI_CTRL0_XFER_COUNT(0)              ;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Enable the BCH block and read. */

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = V1_BV_GPMI_ECCCTRL_ECC_CMD__DECODE;
	buffer_mask  = V1_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				V1_BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

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
		V1_BM_GPMI_CTRL0_LOCK_CS                                 |
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                             |
		V1_BF_GPMI_CTRL0_CS(chip)                                |
		V1_BF_GPMI_CTRL0_ADDRESS(address)                        |
		V1_BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		V1_BM_GPMI_ECCCTRL_ENABLE_ECC 	            |
		V1_BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		V1_BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;
	(*d)->cmd.pio_words[3] = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Disable the BCH block */

	command_mode = V1_BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = V1_BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

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
		V1_BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		V1_BM_GPMI_CTRL0_LOCK_CS                                 |
		V1_BM_GPMI_CTRL0_WORD_LENGTH                             |
		V1_BF_GPMI_CTRL0_CS(chip)                                |
		V1_BF_GPMI_CTRL0_ADDRESS(address)                        |
		V1_BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes) ;

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

	error = nfc_dma_go(this, dma_channel);

	if (error)
		dev_err(dev, "[%s] DMA error\n", __func__);

	/* Wait for the interrupt from the BCH block. */

	wait_for_completion(&nfc->bch_done);

	/* Return success. */

	return error;

}

/*
 * At this point, we've defined all the version-specific primitives. We're now
 * ready to construct the NFC HAL structures for every version.
 */

struct nfc_hal  nfc_v0_hal = {
	.version        = 0,
	.description    = "4-chip GPMI and BCH",
	.max_chip_count = 4,
	.init           = nfc_v0_init,
	.set_geometry   = nfc_v0_set_geometry,
	.exit           = nfc_v0_exit,
	.is_ready       = nfc_v0_is_ready,
	.send_command   = nfc_v0_send_command,
	.send_data      = nfc_v0_send_data,
	.read_data      = nfc_v0_read_data,
	.send_page      = nfc_v0_send_page,
	.read_page      = nfc_v0_read_page,
};

struct nfc_hal  nfc_v1_hal = {
	.version        = 1,
	.description    = "8-chip GPMI and BCH",
	.max_chip_count = 8,
	.init           = nfc_v1_init,
	.set_geometry   = nfc_v1_set_geometry,
	.exit           = nfc_v1_exit,
	.is_ready       = nfc_v1_is_ready,
	.send_command   = nfc_v1_send_command,
	.send_data      = nfc_v1_send_data,
	.read_data      = nfc_v1_read_data,
	.send_page      = nfc_v1_send_page,
	.read_page      = nfc_v1_read_page,
};

/*
 * This array has a pointer to every NFC HAL structure. The probing process will
 * find and install the one that matches the version given by the platform.
 */

struct nfc_hal  *(nfc_hals[]) = {
	&nfc_v0_hal,
	&nfc_v1_hal,
};

/*
 *------------------------------------------------------------------------------
 * Boot ROM Helper
 *
 * The following functions implent Boot ROM helpers for various versions of the
 * Boot ROM.
 *------------------------------------------------------------------------------
 */

/**
 * rom_helper_set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int rom_helper_set_geometry(struct gpmi_nfc_data *this)
{
	struct boot_rom_geometry  *geometry = &this->rom_geometry;

	/*
	 * Set the boot block stride size.
	 *
	 * In principle, we should be reading this from the OTP bits, since
	 * that's where the ROM is going to get it. In fact, we don't have any
	 * way to read the OTP bits, so we go with the default and hope for the
	 * best.
	 */

	geometry->stride_size_in_pages = 64;

	/*
	 * Set the search area stride exponent.
	 *
	 * In principle, we should be reading this from the OTP bits, since
	 * that's where the ROM is going to get it. In fact, we don't have any
	 * way to read the OTP bits, so we go with the default and hope for the
	 * best.
	 */

	geometry->search_area_stride_exponent = 2;

	/* Return success. */

	return 0;

}

/*
 * Useful variables for Boot ROM Helper version 0.
 */

static const char  *rom_helper_v0_fingerprint = "STMP";

/**
 * rom_helper_v0_set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int rom_helper_v0_set_geometry(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata    =  this->pdata;
	struct physical_geometry       *physical = &this->physical_geometry;
	struct boot_rom_geometry       *geometry = &this->rom_geometry;
	int                             error;

	/* Version-independent geometry. */

	error = rom_helper_set_geometry(this);

	if (error)
		return error;

	/*
	 * Check if the platform data indicates we are to protect the boot area.
	 */

	if (!pdata->boot_area_size_in_bytes) {
		geometry->boot_area_count         = 0;
		geometry->boot_area_size_in_bytes = 0;
		return 0;
	}

	/*
	 * If control arrives here, we are supposed to set up partitions to
	 * protect the boot areas. In this version of the ROM, the number of
	 * boot areas and their size depends on the number of chips.
	 */

	if (physical->chip_count == 1) {
		geometry->boot_area_count = 1;
		geometry->boot_area_size_in_bytes =
					pdata->boot_area_size_in_bytes * 2;
	} else {
		geometry->boot_area_count = 2;
		geometry->boot_area_size_in_bytes =
					pdata->boot_area_size_in_bytes;
	}

	/* Return success. */

	return 0;

}

/**
 * rom_helper_v0_check_transcription_stamp() - Checks for a transcription stamp.
 *
 * Returns 0 if a stamp is not found.
 *
 * @this:  Per-device data.
 */
static int rom_helper_v0_check_transcription_stamp(struct gpmi_nfc_data *this)
{
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *nfc_geo  = &this->nfc_geometry;
	struct boot_rom_geometry  *rom_geo  = &this->rom_geometry;
	struct mil                *mil      = &this->mil;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_chip          *nand     = &mil->nand;
	unsigned int              search_area_size_in_strides;
	unsigned int              stride;
	unsigned int              page;
	loff_t                    byte;
	uint8_t                   *buffer = nand->buffers->databuf;
	int                       saved_chip_number;
	int                       found_an_ncb_fingerprint = false;

	/* Compute the number of strides in a search area. */

	search_area_size_in_strides = 1 << rom_geo->search_area_stride_exponent;

	/* Select chip 0. */

	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	/*
	 * Loop through the first search area, looking for the NCB fingerprint.
	 */

	pr_info("Scanning for an NCB fingerprint...\n");

	for (stride = 0; stride < search_area_size_in_strides; stride++) {

		/* Compute the page and byte addresses. */

		page = stride * rom_geo->stride_size_in_pages;
		byte = page   * physical->page_data_size_in_bytes;

		pr_info("  Looking for a fingerprint in page 0x%x\n", page);

		/*
		 * Read the NCB fingerprint. The fingerprint appears in the
		 * first four bytes of the page data, which is just past the
		 * metadata.
		 */

		nand->cmdfunc(mtd, NAND_CMD_READ0,
					nfc_geo->metadata_size_in_bytes, page);
		nand->read_buf(mtd, buffer, strlen(rom_helper_v0_fingerprint));

		/* Look for the fingerprint. */

		if (!memcmp(buffer, rom_helper_v0_fingerprint,
					strlen(rom_helper_v0_fingerprint))) {
			found_an_ncb_fingerprint = true;
			break;
		}

	}

	/* Deselect chip 0. */

	nand->select_chip(mtd, saved_chip_number);

	/* Return. */

	if (found_an_ncb_fingerprint)
		pr_info("  Found a fingerprint\n");
	else
		pr_info("  No fingerprint found\n");

	return found_an_ncb_fingerprint;

}

/**
 * rom_helper_v0_write_transcription_stamp() - Writes a transcription stamp.
 *
 * @this:  Per-device data.
 */
static int rom_helper_v0_write_transcription_stamp(struct gpmi_nfc_data *this)
{
	struct device             *dev      =  this->dev;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *nfc_geo  = &this->nfc_geometry;
	struct boot_rom_geometry  *rom_geo  = &this->rom_geometry;
	struct mil                *mil      = &this->mil;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_chip          *nand     = &mil->nand;
	unsigned int              block_size_in_pages;
	unsigned int              search_area_size_in_strides;
	unsigned int              search_area_size_in_pages;
	unsigned int              search_area_size_in_blocks;
	unsigned int              block;
	unsigned int              stride;
	unsigned int              page;
	loff_t                    byte;
	uint8_t                   *buffer = nand->buffers->databuf;
	int                       saved_chip_number;
	int                       status;

	/* Compute the search area geometry. */

	block_size_in_pages = physical->block_size_in_bytes >>
				(ffs(physical->page_data_size_in_bytes) - 1);

	search_area_size_in_strides = 1 << rom_geo->search_area_stride_exponent;

	search_area_size_in_pages = search_area_size_in_strides *
						rom_geo->stride_size_in_pages;

	search_area_size_in_blocks =
		  (search_area_size_in_pages + (block_size_in_pages - 1)) /
		/*-------------------------------------------------------*/
				    block_size_in_pages;

	#if defined(DETAILED_INFO)

	pr_info("--------------------\n");
	pr_info("Search Area Geometry\n");
	pr_info("--------------------\n");
	pr_info("Search Area Size in Blocks : %u", search_area_size_in_blocks);
	pr_info("Search Area Size in Strides: %u", search_area_size_in_strides);
	pr_info("Search Area Size in Pages  : %u", search_area_size_in_pages);

	#endif

	/* Select chip 0. */

	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	/* Loop over blocks in the first search area, erasing them. */

	pr_info("Erasing the search area...\n");

	for (block = 0; block < search_area_size_in_blocks; block++) {

		/* Compute the page address. */

		page = block * block_size_in_pages;

		/* Erase this block. */

		pr_info("  Erasing block 0x%x\n", block);

		nand->cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
		nand->cmdfunc(mtd, NAND_CMD_ERASE2, -1, -1);

		/* Wait for the erase to finish. */

		status = nand->waitfunc(mtd, nand);

		if (status & NAND_STATUS_FAIL)
			dev_err(dev, "[%s] Erase failed.\n", __func__);

	}

	/* Write the NCB fingerprint into the page buffer. */

	memset(buffer, ~0, mtd->writesize);
	memset(nand->oob_poi, ~0, mtd->oobsize);

	memcpy(buffer + nfc_geo->metadata_size_in_bytes,
		rom_helper_v0_fingerprint, strlen(rom_helper_v0_fingerprint));

	/* Loop through the first search area, writing NCB fingerprints. */

	pr_info("Writing NCB fingerprints...\n");

	for (stride = 0; stride < search_area_size_in_strides; stride++) {

		/* Compute the page and byte addresses. */

		page = stride * rom_geo->stride_size_in_pages;
		byte = page   * physical->page_data_size_in_bytes;

		/* Write the first page of the current stride. */

		pr_info("  Writing an NCB fingerprint in page 0x%x\n", page);

		nand->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
		nand->ecc.write_page_raw(mtd, nand, buffer);
		nand->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

		/* Wait for the write to finish. */

		status = nand->waitfunc(mtd, nand);

		if (status & NAND_STATUS_FAIL)
			dev_err(dev, "[%s] Write failed.\n", __func__);

	}

	/* Deselect chip 0. */

	nand->select_chip(mtd, saved_chip_number);

	/* Return success. */

	return 0;

}

/**
 * rom_helper_v1_set_geometry() - Sets geometry for the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int rom_helper_v1_set_geometry(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata    =  this->pdata;
	struct boot_rom_geometry       *geometry = &this->rom_geometry;
	int                            error;

	/* Version-independent geometry. */

	error = rom_helper_set_geometry(this);

	if (error)
		return error;

	/*
	 * Check if the platform data indicates we are to protect the boot area.
	 */

	if (!pdata->boot_area_size_in_bytes) {
		geometry->boot_area_count         = 0;
		geometry->boot_area_size_in_bytes = 0;
		return 0;
	}

	/*
	 * If control arrives here, we are supposed to set up partitions to
	 * protect the boot areas. In this version of the ROM, we support only
	 * one boot area.
	 */

	geometry->boot_area_count = 1;

	/*
	 * Use the platform's boot area size.
	 */

	geometry->boot_area_size_in_bytes = pdata->boot_area_size_in_bytes;

	/* Return success. */

	return 0;

}

/*
 * At this point, we've defined all the version-specific primitives. We're now
 * ready to construct the Boot ROM Helper structures for every version.
 */

struct boot_rom_helper  boot_rom_helper_0 = {
	.version                   = 0,
	.description               = "Single/dual-chip boot area, "
						"no block mark swapping",
	.swap_block_mark           = false,
	.set_geometry              = rom_helper_v0_set_geometry,
	.check_transcription_stamp = rom_helper_v0_check_transcription_stamp,
	.write_transcription_stamp = rom_helper_v0_write_transcription_stamp,
};

struct boot_rom_helper  boot_rom_helper_1 = {
	.version                   = 1,
	.description               = "Single-chip boot area, "
						"block mark swapping supported",
	.swap_block_mark           = true,
	.set_geometry              = rom_helper_v1_set_geometry,
	.check_transcription_stamp = 0,
	.write_transcription_stamp = 0,
};

/*
 * This array has a pointer to every Boot ROM Helper structure. The probing
 * process will find and install the one that matches the version given by the
 * platform.
 */

struct boot_rom_helper  *(boot_rom_helpers[]) = {
	&boot_rom_helper_0,
	&boot_rom_helper_1,
};

/*
 *------------------------------------------------------------------------------
 * MTD Interface Layer
 *
 * The following functions interface this driver to the NAND Flash MTD system.
 *------------------------------------------------------------------------------
 */

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

	/* If control arrives here, all is well. */

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

	if (map_io_buffers && virt_addr_valid(destination))
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

	/* If control arrives here, all is well. */

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

	add_event("mil_cmd_ctrl sending command...", 1);

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

	add_event("...Finished", -1);

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

	add_event("=> mil_dev_ready", 1);

	if (nfc->is_ready(this, mil->current_chip)) {
		add_event("<= mil_dev_ready - Returning ready", -1);
		return !0;
	} else {
		add_event("<= mil_dev_ready - Returning busy", -1);
		return 0;
	}

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
	struct clk            *clock = this->resources.clock;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc select_chip] chip: %d\n", chip);

	/* Figure out what kind of transition this is. */

	if ((mil->current_chip < 0) && (chip >= 0)) {
		start_event_trace("=> mil_select_chip");
		clk_enable(clock);
		add_event("<= mil_select_chip", -1);
	} else if ((mil->current_chip >= 0) && (chip < 0)) {
		add_event("=> mil_select_chip", 1);
		clk_disable(clock);
		stop_event_trace("<= mil_select_chip");
	} else {
		add_event("=> mil_select_chip", 1);
		add_event("<= mil_select_chip", -1);
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

	add_event("=> mil_read_buf", 1);

	/* Set up DMA. */

	error = mil_incoming_buffer_dma_begin(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					&use_virt, &use_phys);

	if (error) {
		dev_err(dev, "[%s] Inadequate DMA buffer\n", __func__);
		goto exit;
	}

	/* Ask the NFC. */

	nfc->read_data(this, mil->current_chip, use_phys, len);

	/* Finish with DMA. */

	mil_incoming_buffer_dma_end(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					use_virt, use_phys);

	/* Return. */

exit:

	add_event("<= mil_read_buf", -1);

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

	add_event("=> mil_write_buf", 1);

	/* Set up DMA. */

	error = mil_outgoing_buffer_dma_begin(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					&use_virt, &use_phys);

	if (error) {
		dev_err(dev, "[%s] Inadequate DMA buffer\n", __func__);
		goto exit;
	}

	/* Ask the NFC. */

	nfc->send_data(this, mil->current_chip, use_phys, len);

	/* Finish with DMA. */

	mil_outgoing_buffer_dma_end(this, buf, len,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					use_virt, use_phys);

	/* Return. */

exit:

	add_event("<= mil_write_buf", -1);

}

/**
 * mil_read_byte() - MTD Interface read_byte().
 *
 * @mtd:  A pointer to the owning MTD.
 */
static uint8_t mil_read_byte(struct mtd_info *mtd)
{
	uint8_t  byte;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc read_byte]\n");

	add_event("=> mil_read_byte", 1);

	mil_read_buf(mtd, (uint8_t *) &byte, 1);

	add_event("<= mil_read_byte", -1);

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc read_byte]: 0x%02x\n", byte);

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
static int mil_ecc_read_page(struct mtd_info *mtd,
					struct nand_chip *nand, uint8_t *buf)
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

	add_event("=> mil_ecc_read_page", 1);

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

	failed    = 0;
	corrected = 0;

	status = ((unsigned char *) auxiliary_virt) +
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

	/* Propagate ECC status to the owning MTD. */

	mtd->ecc_stats.failed    += failed;
	mtd->ecc_stats.corrected += corrected;

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

	/* Return. */

exit_nfc:
	mil_incoming_buffer_dma_end(this, buf, mtd->writesize,
					mil->payload_virt, mil->payload_phys,
					nfc_geo->payload_size_in_bytes,
					payload_virt, payload_phys);
exit_payload:

	add_event("<= mil_ecc_read_page", -1);

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

	add_event("=> mil_ecc_write_page", 1);

	/*
	 * Set up DMA.
	 *
	 * If we're doing block mark swapping, we *must* use our own buffers
	 * because we need to modify the incoming data, and we can't touch the
	 * caller's buffers.
	 *
	 * If we're not doing block mark swapping, we can try to map the
	 * caller's buffers.
	 */

	if (rom->swap_block_mark) {

		memcpy(mil->payload_virt, buf, mtd->writesize);
		payload_virt = mil->payload_virt;
		payload_phys = mil->payload_phys;

		memcpy(mil->auxiliary_virt, nand->oob_poi, mtd->oobsize);
		auxiliary_virt = mil->auxiliary_virt;
		auxiliary_phys = mil->auxiliary_phys;

	} else {

		error = mil_outgoing_buffer_dma_begin(this,
				buf, mtd->writesize,
				mil->payload_virt, mil->payload_phys,
				nfc_geo->payload_size_in_bytes,
				&payload_virt, &payload_phys);

		if (error) {
			dev_err(dev, "[%s] Inadequate payload DMA buffer\n",
								__func__);
			goto exit_payload;
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

	/* Handle block mark swapping before writing. */

	mil_handle_block_mark_swapping(this,
				(void *) payload_virt, (void *) auxiliary_virt);

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
exit_payload:

	add_event("<= mil_ecc_write_page", -1);

}

/**
 * mil_hook_read_oob() - Hooked MTD Interface read_oob().
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code. See the description of the raw_oob_mode field in
 * struct mil for more information about this.
 *
 * @mtd:   A pointer to the MTD.
 * @from:  The starting address to read.
 * @ops:   Describes the operation.
 */
static int mil_hook_read_oob(struct mtd_info *mtd,
					loff_t from, struct mtd_oob_ops *ops)
{
	register struct nand_chip  *chip = mtd->priv;
	struct gpmi_nfc_data       *this = chip->priv;
	struct mil                 *mil  = &this->mil;
	int                        ret;

	mil->raw_oob_mode = ops->mode == MTD_OOB_RAW;
	ret = mil->hooked_read_oob(mtd, from, ops);
	mil->raw_oob_mode = false;
	return ret;
}

/**
 * mil_hook_write_oob() - Hooked MTD Interface write_oob().
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code. See the description of the raw_oob_mode field in
 * struct mil for more information about this.
 *
 * @mtd:   A pointer to the MTD.
 * @to:    The starting address to write.
 * @ops:   Describes the operation.
 */
static int mil_hook_write_oob(struct mtd_info *mtd,
					loff_t to, struct mtd_oob_ops *ops)
{
	register struct nand_chip  *chip = mtd->priv;
	struct gpmi_nfc_data       *this = chip->priv;
	struct mil                 *mil  = &this->mil;
	int                        ret;

	mil->raw_oob_mode = ops->mode == MTD_OOB_RAW;
	ret = mil->hooked_write_oob(mtd, to, ops);
	mil->raw_oob_mode = false;
	return ret;
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
 * Since our OOB *is* covered by ECC, we need this information. So, we hook the
 * ecc.read_oob and ecc.write_oob function pointers in the owning
 * struct mtd_info with our own functions. These hook functions set the
 * raw_oob_mode field so that, when control finally arrives here, we'll know
 * what to do.
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
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct boot_rom_helper    *rom      =  this->rom;
	int                       block_mark_column;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc ecc_read_oob] "
		"page: 0x%06x, sndcmd: %s\n", page, sndcmd ? "Yes" : "No");

	add_event("=> mil_ecc_read_oob", 1);

	/*
	 * First, fill in the OOB buffer. If we're doing a raw read, we need to
	 * get the bytes from the physical page. If we're not doing a raw read,
	 * we need to fill the buffer with set bits.
	 */

	if (mil->raw_oob_mode) {

		/*
		 * If control arrives here, we're doing a "raw" read. Send the
		 * command to read the conventional OOB.
		 */

		nand->cmdfunc(mtd, NAND_CMD_READ0,
				physical->page_data_size_in_bytes, page);

		/* Read out the conventional OOB. */

		nand->read_buf(mtd, nand->oob_poi, mtd->oobsize);

	} else {

		/*
		 * If control arrives here, we're not doing a "raw" read. Fill
		 * the OOB buffer with set bits.
		 */

		memset(nand->oob_poi, ~0, mtd->oobsize);

	}

	/*
	 * Now, we want to make sure the block mark is correct. In the
	 * Swapping/Raw case, we already have it. Otherwise, we need to
	 * explicitly read it.
	 */

	if (!(rom->swap_block_mark && mil->raw_oob_mode)) {

		/* First, figure out where the block mark is. */

		if (rom->swap_block_mark)
			block_mark_column = physical->page_data_size_in_bytes;
		else
			block_mark_column = 0;

		/* Send the command to read the block mark. */

		nand->cmdfunc(mtd, NAND_CMD_READ0, block_mark_column, page);

		/* Read the block mark into the first byte of the OOB buffer. */

		nand->oob_poi[0] = nand->read_byte(mtd);

	}

	/*
	 * Return true, indicating that the next call to this function must send
	 * a command.
	 */

	add_event("<= mil_ecc_read_oob", -1);

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
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct boot_rom_helper    *rom      =  this->rom;
	uint8_t                   block_mark = 0;
	int                       block_mark_column;
	int                       status;
	int                       error = 0;

	DEBUG(MTD_DEBUG_LEVEL2,
			"[gpmi_nfc ecc_write_oob] page: 0x%06x\n", page);

	add_event("=> mil_ecc_write_oob", -1);

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
		block_mark_column = physical->page_data_size_in_bytes;
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

	/* Return. */

exit:

	add_event("<= mil_ecc_write_oob", -1);

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
int mil_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	return 0;
}

/**
 * mil_set_physical_geometry() - Set up the physical medium geometry.
 *
 * This function retrieves the physical geometry information discovered by
 * nand_scan(), corrects it, and records it in the per-device data structure.
 *
 * @this:  Per-device data.
 */
static int mil_set_physical_geometry(struct gpmi_nfc_data  *this)
{
	struct mil                *mil      = &this->mil;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mtd_info           *mtd      = &mil->mtd;
	struct nand_chip          *nand     = &mil->nand;
	struct nand_device_info   *info;
	int                       saved_chip_number;
	uint8_t                   id_bytes[NAND_DEVICE_ID_BYTE_COUNT];
	unsigned int              block_size_in_pages;
	unsigned int              chip_size_in_blocks;
	unsigned int              chip_size_in_pages;
	uint64_t                  medium_size_in_bytes;

	/* Read ID bytes from the first NAND Flash chip. */

	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	nand->cmdfunc(mtd, NAND_CMD_READID, 0, -1);
	nand->read_buf(mtd, id_bytes, NAND_DEVICE_ID_BYTE_COUNT);

	nand->select_chip(mtd, saved_chip_number);

	/* Look up information about this device, based on the ID bytes. */

	info = nand_device_get_info(id_bytes);

	/* Check if we understand this device. */

	if (!info) {
		printk(KERN_ERR "Unrecognized NAND Flash device.\n");
		return !0;
	}

	/* Display the information we discovered. */

	#if defined(DETAILED_INFO)
	pr_info("-----------------------------\n");
	pr_info("NAND Flash Device Information\n");
	pr_info("-----------------------------\n");
	nand_device_print_info(info);
	#endif

	/*
	 * Copy the device info into the per-device data. We can't just keep
	 * the pointer because that storage is reclaimed after initialization.
	 * Notice that we null out the pointer to the device description, which
	 * will also be reclaimed.
	 */

	this->device_info = *info;
	this->device_info.description = 0;

	/*
	 * Record the number of physical chips that MTD found.
	 */

	physical->chip_count = nand->numchips;

	/*
	 * We know the total size of a page. We need to break that down into the
	 * data size and OOB size. The data size is the largest power of two
	 * that will fit in the given page size. The OOB size is what's left
	 * over.
	 */

	physical->page_data_size_in_bytes =
				1 << (fls(info->page_total_size_in_bytes) - 1);

	physical->page_oob_size_in_bytes =
				info->page_total_size_in_bytes -
					physical->page_data_size_in_bytes;

	/*
	 * Now that we know the page data size, we can multiply this by the
	 * number of pages in a block to compute the block size.
	 */

	physical->block_size_in_bytes =
		physical->page_data_size_in_bytes * info->block_size_in_pages;

	/* Get the chip size. */

	physical->chip_size_in_bytes = info->chip_size_in_bytes;

	/* Compute some interesting facts. */

	block_size_in_pages  =
			physical->block_size_in_bytes >>
				(fls(physical->page_data_size_in_bytes) - 1);
	chip_size_in_pages   =
			physical->chip_size_in_bytes >>
				(fls(physical->page_data_size_in_bytes) - 1);
	chip_size_in_blocks  =
			physical->chip_size_in_bytes >>
				(fls(physical->block_size_in_bytes) - 1);
	medium_size_in_bytes =
			physical->chip_size_in_bytes * physical->chip_count;

	/* Report. */

	#if defined(DETAILED_INFO)

	pr_info("-----------------\n");
	pr_info("Physical Geometry\n");
	pr_info("-----------------\n");
	pr_info("Chip Count             : %d\n", physical->chip_count);
	pr_info("Page Data Size in Bytes: %u (0x%x)\n",
			physical->page_data_size_in_bytes,
			physical->page_data_size_in_bytes);
	pr_info("Page OOB Size in Bytes : %u\n",
			physical->page_oob_size_in_bytes);
	pr_info("Block Size in Bytes    : %u (0x%x)\n",
			physical->block_size_in_bytes,
			physical->block_size_in_bytes);
	pr_info("Block Size in Pages    : %u (0x%x)\n",
			block_size_in_pages,
			block_size_in_pages);
	pr_info("Chip Size in Bytes     : %llu (0x%llx)\n",
			physical->chip_size_in_bytes,
			physical->chip_size_in_bytes);
	pr_info("Chip Size in Pages     : %u (0x%x)\n",
			chip_size_in_pages, chip_size_in_pages);
	pr_info("Chip Size in Blocks    : %u (0x%x)\n",
			chip_size_in_blocks, chip_size_in_blocks);
	pr_info("Medium Size in Bytes   : %llu (0x%llx)\n",
			medium_size_in_bytes, medium_size_in_bytes);

	#endif

	/* Return success. */

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
	struct nfc_geometry  *geo = &this->nfc_geometry;

	/* Set the NFC geometry. */

	if (nfc->set_geometry(this))
		return !0;

	/* Report. */

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

	/* Return success. */

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
	struct boot_rom_geometry  *geo = &this->rom_geometry;

	/* Set the Boot ROM Helper geometry. */

	if (rom->set_geometry(this))
		return !0;

	/* Report. */

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

	/* Return success. */

	return 0;

}

/**
 * mil_set_mtd_geometry() - Set up the MTD geometry.
 *
 * This function adjusts the owning MTD data structures to match the logical
 * geometry we've chosen.
 *
 * @this:  Per-device data.
 */
static int mil_set_mtd_geometry(struct gpmi_nfc_data *this)
{
	struct physical_geometry  *physical = &this->physical_geometry;
	struct mil                *mil      = &this->mil;
	struct nand_ecclayout     *layout   = &mil->oob_layout;
	struct nand_chip          *nand     = &mil->nand;
	struct mtd_info           *mtd      = &mil->mtd;

	/* Configure the struct nand_ecclayout. */

	layout->eccbytes          = 0;
	layout->oobavail          = physical->page_oob_size_in_bytes;
	layout->oobfree[0].offset = 0;
	layout->oobfree[0].length = physical->page_oob_size_in_bytes;

	/* Configure the struct mtd_info. */

	mtd->size        = nand->numchips * physical->chip_size_in_bytes;
	mtd->erasesize   = physical->block_size_in_bytes;
	mtd->writesize   = physical->page_data_size_in_bytes;
	mtd->ecclayout   = layout;
	mtd->oobavail    = mtd->ecclayout->oobavail;
	mtd->oobsize     = mtd->ecclayout->oobavail + mtd->ecclayout->eccbytes;
	mtd->subpage_sft = 0; /* We don't support sub-page writing. */

	/* Configure the struct nand_chip. */

	nand->chipsize         = physical->chip_size_in_bytes;
	nand->page_shift       = ffs(mtd->writesize) - 1;
	nand->pagemask         = (nand->chipsize >> nand->page_shift) - 1;
	nand->subpagesize      = mtd->writesize >> mtd->subpage_sft;
	nand->phys_erase_shift = ffs(mtd->erasesize) - 1;
	nand->bbt_erase_shift  = nand->phys_erase_shift;
	nand->oob_poi          = nand->buffers->databuf + mtd->writesize;
	nand->ecc.layout       = layout;
	if (nand->chipsize & 0xffffffff)
		nand->chip_shift = ffs((unsigned) nand->chipsize) - 1;
	else
		nand->chip_shift =
				ffs((unsigned) (nand->chipsize >> 32)) + 32 - 1;

	/* Return success. */

	return 0;

}

/**
 * mil_set_geometry() - Set up the medium geometry.
 *
 * @this:  Per-device data.
 */
static int mil_set_geometry(struct gpmi_nfc_data  *this)
{
	struct device        *dev      =  this->dev;
	struct nfc_geometry  *nfc_geo  = &this->nfc_geometry;
	struct mil           *mil      = &this->mil;

	/* Set up the various layers of geometry, in this specific order. */

	if (mil_set_physical_geometry(this))
		return -ENXIO;

	if (mil_set_nfc_geometry(this))
		return -ENXIO;

	if (mil_set_boot_rom_helper_geometry(this))
		return -ENXIO;

	if (mil_set_mtd_geometry(this))
		return -ENXIO;

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

	mil->page_buffer_virt =
		dma_alloc_coherent(dev, mil->page_buffer_size,
					&mil->page_buffer_phys, GFP_DMA);

	if (!mil->page_buffer_virt)
		return -ENOMEM;

	/* Slice up the page buffer. */

	mil->payload_virt = mil->page_buffer_virt;
	mil->payload_phys = mil->page_buffer_phys;

	mil->auxiliary_virt = ((char *) mil->payload_virt) +
						nfc_geo->payload_size_in_bytes;
	mil->auxiliary_phys = mil->payload_phys +
						nfc_geo->payload_size_in_bytes;

	/* Return success. */

	return 0;

}

/**
 * mil_pre_bbt_scan() - Prepare for the BBT scan.
 *
 * @this:  Per-device data.
 */
static int mil_pre_bbt_scan(struct gpmi_nfc_data  *this)
{
	struct device             *dev      =  this->dev;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct boot_rom_helper    *rom      =  this->rom;
	struct mil                *mil      = &this->mil;
	struct nand_chip          *nand     = &mil->nand;
	struct mtd_info           *mtd      = &mil->mtd;
	unsigned int              block_count;
	unsigned int              block;
	int                       chip;
	int                       page;
	loff_t                    byte;
	uint8_t                   block_mark;
	int                       error;

	/*
	 * Check if we can use block mark swapping, which enables us to leave
	 * the block marks where they are. If so, we don't need to do anything
	 * at all.
	 */

	if (rom->swap_block_mark)
		return 0;

	/*
	 * If control arrives here, we can't use block mark swapping, which
	 * means we're forced to use transcription. First, scan for the
	 * transcription stamp. If we find it, then we don't have to do
	 * anything -- the block marks are already transcribed.
	 */

	if (rom->check_transcription_stamp(this))
		return 0;

	/*
	 * If control arrives here, we couldn't find a transcription stamp, so
	 * so we presume the block marks are in the conventional location.
	 */

	pr_info("Transcribing bad block marks...\n");

	/* Compute the number of blocks in the entire medium. */

	block_count =
		physical->chip_size_in_bytes >> nand->phys_erase_shift;

	/*
	 * Loop over all the blocks in the medium, transcribing block marks as
	 * we go.
	 */

	for (block = 0; block < block_count; block++) {

		/*
		 * Compute the chip, page and byte addresses for this block's
		 * conventional mark.
		 */

		chip = block >> (nand->chip_shift - nand->phys_erase_shift);
		page = block << (nand->phys_erase_shift - nand->page_shift);
		byte = block <<  nand->phys_erase_shift;

		/* Select the chip. */

		nand->select_chip(mtd, chip);

		/* Send the command to read the conventional block mark. */

		nand->cmdfunc(mtd, NAND_CMD_READ0,
				physical->page_data_size_in_bytes, page);

		/* Read the conventional block mark. */

		block_mark = nand->read_byte(mtd);

		/*
		 * Check if the block is marked bad. If so, we need to mark it
		 * again, but this time the result will be a mark in the
		 * location where we transcribe block marks.
		 *
		 * Notice that we have to explicitly set the marking_a_bad_block
		 * member before we call through the block_markbad function
		 * pointer in the owning struct nand_chip. If we could call
		 * though the block_markbad function pointer in the owning
		 * struct mtd_info, which we have hooked, then this would be
		 * taken care of for us. Unfortunately, we can't because that
		 * higher-level code path will do things like consulting the
		 * in-memory bad block table -- which doesn't even exist yet!
		 * So, we have to call at a lower level and handle some details
		 * ourselves.
		 */

		if (block_mark != 0xff) {
			pr_info("Transcribing block mark in block %u\n", block);
			mil->marking_a_bad_block = true;
			error = nand->block_markbad(mtd, byte);
			mil->marking_a_bad_block = false;
			if (error)
				dev_err(dev, "Failed to mark block bad with "
							"error %d\n", error);
		}

		/* Deselect the chip. */

		nand->select_chip(mtd, -1);

	}

	/* Write the stamp that indicates we've transcribed the block marks. */

	rom->write_transcription_stamp(this);

	/* Return success. */

	return 0;

}

/**
 * mil_scan_bbt() - MTD Interface scan_bbt().
 *
 * The HIL calls this function once, when it initializes the NAND Flash MTD.
 *
 * Nominally, the purpose of this function is to look for or create the bad
 * block table. In fact, since the HIL calls this function at the very end of
 * the initialization process started by nand_scan(), and the HIL doesn't have a
 * more formal mechanism, everyone "hooks" this function to continue the
 * initialization process.
 *
 * At this point, the physical NAND Flash chips have been identified and
 * counted, so we know the physical geometry. This enables us to make some
 * important configuration decisions.
 *
 * The return value of this function propogates directly back to this driver's
 * call to nand_scan(). Anything other than zero will cause this driver to
 * tear everything down and declare failure.
 *
 * @mtd:  A pointer to the owning MTD.
 */
static int mil_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip      *nand = mtd->priv;
	struct gpmi_nfc_data  *this = nand->priv;
	int                   error;

	DEBUG(MTD_DEBUG_LEVEL2, "[gpmi_nfc scan_bbt] \n");

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

	/* Set up geometry. */

	error = mil_set_geometry(this);

	if (error)
		return error;

	/* Prepare for the BBT scan. */

	error = mil_pre_bbt_scan(this);

	if (error)
		return error;

	/* We use the reference implementation for bad block management. */

	error = nand_default_bbt(mtd);

	if (error)
		return error;

	/* Return success. */

	return 0;

}

/**
 * mil_boot_areas_init() - Initializes boot areas.
 *
 * @this:  Per-device data.
 */
static int mil_boot_areas_init(struct gpmi_nfc_data *this)
{
	struct device                  *dev      =  this->dev;
	struct physical_geometry       *physical = &this->physical_geometry;
	struct boot_rom_geometry       *rom      = &this->rom_geometry;
	struct mil                     *mil      = &this->mil;
	struct mtd_info                *mtd      = &mil->mtd;
	struct nand_chip               *nand     = &mil->nand;
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

		dev_err(dev, "Boot area count is incorrectly set to zero.");
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
	 * time to both write and test -- and, at this writing, we don't have a
	 * chip that cares.
	 */

	/* Check if a boot area is larger than a single chip. */

	if (rom->boot_area_size_in_bytes > physical->chip_size_in_bytes) {
		dev_emerg(dev, "Boot area size is larger than a single chip");
		return -ENXIO;
	}

	if (rom->boot_area_count == 1) {

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

		for (i = 0; i < MAX_MTD_DEVICES; i++) {
			search_mtd = get_mtd_device(0, i);
			if (!search_mtd)
				continue;
			if (search_mtd == ERR_PTR(-ENODEV))
				continue;
			if (search_mtd->name == general_use_name)
				mil->general_use_mtd = search_mtd;
		}

		if (!mil->general_use_mtd) {
			dev_emerg(dev, "Can't find general use MTD");
			BUG();
		}

	} else if (rom->boot_area_count == 2) {

		/*
		 * If control arrives here, there is more than one chip. We
		 * partition the medium and concatenate the remainders like so:
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
		 *      +----------+----------- ... ----------------------+
		 *
		 * Notice that the results we leave in the master MTD table
		 * are a little bit goofy:
		 *
		 *    * Chip 0 Boot Area
		 *    * Chip 1 Boot Area
		 *    * Chip 0 Remainder
		 *    * Medium Remainder
		 *
		 * There are two reasons:
		 *
		 * 1) Since the remainder partitions aren't very useful, we'd
		 *    actually prefer to "hide" (create them but not register
		 *    them). This was possible before 2.6.31, but now the
		 *    partitioning code automatically registers anything it
		 *    creates (thanks :(). It might be possible to "unregister"
		 *    these MTDs after the fact, but I don't have time to look
		 *    into the other effects that might have.
		 *
		 * 2) Some user space programs expect the boot partitions to
		 *    appear first. This is naive, but let's try not to cause
		 *    any trouble, where we can avoid it.
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

		for (i = 0; i < MAX_MTD_DEVICES; i++) {
			search_mtd = get_mtd_device(0, i);
			if (!search_mtd)
				continue;
			if (search_mtd == ERR_PTR(-ENODEV))
				continue;
			if (search_mtd->name == chip_0_remainder_name)
				chip_0_remainder_mtd = search_mtd;
			if (search_mtd->name == medium_remainder_name)
				medium_remainder_mtd = search_mtd;
		}

		if (!chip_0_remainder_mtd || !medium_remainder_mtd) {
			dev_emerg(dev, "Can't find remainder partitions");
			BUG();
		}

		/* Concatenate the remainders and register the result. */

		concatenate[0] = chip_0_remainder_mtd;
		concatenate[1] = medium_remainder_mtd;

		mil->general_use_mtd = mtd_concat_create(concatenate,
							2, general_use_name);

		add_mtd_device(mil->general_use_mtd);

	} else {
		dev_err(dev, "Boot area count greater than two is "
							"unimplemented.\n");
		return -ENXIO;
	}

	/* Return success. */

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

	if (rom->boot_area_count > 1) {
		del_mtd_device(mil->general_use_mtd);
		mtd_concat_destroy(mil->general_use_mtd);
	}

	/*
	 * At this point, we're left only with the partitions of the main MTD.
	 * Delete them.
	 */

	del_mtd_partitions(mtd);

	/* The general use MTD no longer exists. */

	mil->general_use_mtd = 0;

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
	 * reasons that don't make particular sense, we like the lowest-numbered
	 * MTDs to be the boot areas.
	 */

	if (register_main_mtd) {
		pr_info("Registering the main MTD.\n");
		add_mtd_device(mtd);
	}

	/*
	 * If partitioning is available, we want to apply the user's partitions
	 * to the general use MTD. Recall that the general use MTD *can* be just
	 * a pointer to the main MTD.
	 */

	#if defined(CONFIG_MTD_PARTITIONS)

		/*
		 * Clear out the partition information.
		 */

		mil->partition_count = 0;
		mil->partitions      = 0;

		/*
		 * First, try to get partition information from the given
		 * partition sources.
		 */

		if (pdata->partition_source_types)
			mil->partition_count =
				parse_mtd_partitions(mtd,
					pdata->partition_source_types,
						&mil->partitions, 0);

		/*
		 * Check if we got anything from a partition source. If not,
		 * then accept whatever partitions are attached to the
		 * platform data.
		 */

		if ((mil->partition_count <= 0) && (pdata->partitions)) {
			mil->partition_count = mil->partition_count;
			mil->partitions      = mil->partitions;
		}

		/* If we came up with any partitions at all, apply them. */

		if (mil->partition_count)
			add_mtd_partitions(mil->general_use_mtd,
					mil->partitions, mil->partition_count);

	#endif

	/* Return success. */

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

	/*
	 * Check if we applied any partitions from either a partition source or
	 * the platform data.
	 */

	if (mil->partition_count)
		del_mtd_partitions(mil->general_use_mtd);

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

/**
 * mil_init() - Initializes the MTD Interface Layer.
 *
 * @this:  Per-device data.
 */
static int mil_init(struct gpmi_nfc_data *this)
{
	struct device                  *dev   =  this->dev;
	struct gpmi_nfc_platform_data  *pdata =  this->pdata;
	struct mil                     *mil   = &this->mil;
	struct mtd_info                *mtd   = &mil->mtd;
	struct nand_chip               *nand  = &mil->nand;
	static struct nand_ecclayout   fake_ecc_layout;
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

	/*
	 * Install a "fake" ECC layout.
	 *
	 * We'll be calling nand_scan() to do the final MTD setup. If we haven't
	 * already chosen an ECC layout, then nand_scan() will choose one based
	 * on the part geometry it discovers. Unfortunately, it doesn't make
	 * good choices. It would be best if we could install the correct ECC
	 * layout now, before we call nand_scan(). We can't do that because we
	 * don't know the medium geometry yet. Here, we install a "fake" ECC
	 * layout just to stop nand_scan() from trying to pick one for itself.
	 * Later, when we know the medium geometry, we'll install the correct
	 * one.
	 *
	 * Of course, this tactic depends critically on the MTD code not doing
	 * an I/O operation that depends on the ECC layout being sensible. This
	 * is in fact the case.
	 */

	memset(&fake_ecc_layout, 0, sizeof(fake_ecc_layout));

	nand->ecc.layout = &fake_ecc_layout;

	/* Allocate a command buffer. */

	mil->cmd_virt =
		dma_alloc_coherent(dev,
			MIL_COMMAND_BUFFER_SIZE, &mil->cmd_phys, GFP_DMA);

	if (!mil->cmd_virt)
		goto exit_cmd_allocation;

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

	mil->hooked_read_oob      = mtd->read_oob;
	mtd->read_oob             = mil_hook_read_oob;

	mil->hooked_write_oob     = mtd->write_oob;
	mtd->write_oob            = mil_hook_write_oob;

	mil->hooked_block_markbad = mtd->block_markbad;
	mtd->block_markbad        = mil_hook_block_markbad;

	/* Construct partitions as necessary. */

	error = mil_partitions_init(this);

	if (error)
		goto exit_partitions;

	/* Return success. */

	return 0;

	/* Control arrives here if something went wrong. */

exit_partitions:
	nand_release(&mil->mtd);
exit_nand_scan:
	dma_free_coherent(dev, MIL_COMMAND_BUFFER_SIZE,
						mil->cmd_virt, mil->cmd_phys);
	mil->cmd_virt =  0;
	mil->cmd_phys = ~0;
exit_cmd_allocation:

	return error;

}

/**
 * mil_exit() - Shuts down the MTD Interface Layer.
 *
 * @this:  Per-device data.
 */
static void mil_exit(struct gpmi_nfc_data *this)
{
	struct device  *dev = this->dev;
	struct mil     *mil = &this->mil;

	/* Shut down partitions as necessary. */

	mil_partitions_exit(this);

	/* Get MTD to let go of our MTD. */

	nand_release(&mil->mtd);

	/* Free the page buffer, if it's been allocated. */

	if (mil->page_buffer_virt)
		dma_free_coherent(dev, mil->page_buffer_size,
				mil->page_buffer_virt, mil->page_buffer_phys);

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

/*
 *------------------------------------------------------------------------------
 * System Interface
 *
 * The following functions interface this driver to the rest of the kernel.
 *------------------------------------------------------------------------------
 */

/**
 * show_device_numchips() - Shows the number of physical chips.
 *
 * This node is made obsolete by the physical_geometry node, but we keep it for
 * backward compatibility (especially for kobs).
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_numchips(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this     = dev_get_drvdata(dev);
	struct physical_geometry  *physical = &this->physical_geometry;

	return sprintf(buf, "%d\n", physical->chip_count);

}

/**
 * show_device_physical_geometry() - Shows the physical Flash device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_physical_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this     = dev_get_drvdata(dev);
	struct physical_geometry  *physical = &this->physical_geometry;

	return sprintf(buf,
		"Chip Count             : %u\n"
		"Chip Size in Bytes     : %llu\n"
		"Block Size in Bytes    : %u\n"
		"Page Data Size in Bytes: %u\n"
		"Page OOB Size in Bytes : %u\n"
		,
		physical->chip_count,
		physical->chip_size_in_bytes,
		physical->block_size_in_bytes,
		physical->page_data_size_in_bytes,
		physical->page_oob_size_in_bytes
	);

}

/**
 * show_device_nfc_info() - Shows the NFC-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this      = dev_get_drvdata(dev);
	struct resources      *resources = &this->resources;
	struct nfc_hal        *nfc       =  this->nfc;
	unsigned long         parent_clock_rate_in_hz;
	unsigned long         clock_rate_in_hz;
	struct clk            *parent_clock;

	parent_clock            = clk_get_parent(resources->clock);
	parent_clock_rate_in_hz = clk_get_rate(parent_clock);
	clock_rate_in_hz        = clk_get_rate(resources->clock);

	return sprintf(buf,
		"Version                 : %u\n"
		"Description             : %s\n"
		"Max Chip Count          : %u\n"
		"Parent Clock Rate in Hz : %lu\n"
		"Clock Rate in Hz        : %lu\n"
		,
		nfc->version,
		nfc->description,
		nfc->max_chip_count,
		parent_clock_rate_in_hz,
		clock_rate_in_hz
	);

}

/**
 * show_device_nfc_geometry() - Shows the NFC view of the device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct nfc_geometry   *nfc  = &this->nfc_geometry;

	return sprintf(buf,
		"ECC Algorithm          : %s\n"
		"ECC Strength           : %u\n"
		"Page Size in Bytes     : %u\n"
		"Metadata Size in Bytes : %u\n"
		"ECC Chunk Size in Bytes: %u\n"
		"ECC Chunk Count        : %u\n"
		"Payload Size in Bytes  : %u\n"
		"Auxiliary Size in Bytes: %u\n"
		"Auxiliary Status Offset: %u\n"
		"Block Mark Byte Offset : %u\n"
		"Block Mark Bit Offset  : %u\n"
		,
		nfc->ecc_algorithm,
		nfc->ecc_strength,
		nfc->page_size_in_bytes,
		nfc->metadata_size_in_bytes,
		nfc->ecc_chunk_size_in_bytes,
		nfc->ecc_chunk_count,
		nfc->payload_size_in_bytes,
		nfc->auxiliary_size_in_bytes,
		nfc->auxiliary_status_offset,
		nfc->block_mark_byte_offset,
		nfc->block_mark_bit_offset
	);

}

/**
 * show_device_rom_geometry() - Shows the Boot ROM Helper's geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_rom_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data      *this = dev_get_drvdata(dev);
	struct boot_rom_geometry  *rom  = &this->rom_geometry;

	return sprintf(buf,
		"Boot Area Count           : %u\n"
		"Boot Area Size in Bytes   : %u\n"
		"Stride Size in Pages      : %u\n"
		"Seach Area Stride Exponent: %u\n"
		,
		rom->boot_area_count,
		rom->boot_area_size_in_bytes,
		rom->stride_size_in_pages,
		rom->search_area_stride_exponent
	);

}

/**
 * show_device_mtd_nand_info() - Shows the device's MTD NAND-specific info.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_nand_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct gpmi_nfc_data       *this = dev_get_drvdata(dev);
	struct mil                 *mil  = &this->mil;
	struct nand_chip           *nand = &mil->nand;

	o += sprintf(buf + o,
		"Options                       : 0x%08x\n"
		"Chip Count                    : %u\n"
		"Chip Size in Bytes            : %llu\n"
		"Minimum Writable Size in Bytes: %u\n"
		"Page Shift                    : %u\n"
		"Page Mask                     : 0x%x\n"
		"Block Shift                   : %u\n"
		"BBT Block Shift               : %u\n"
		"Chip Shift                    : %u\n"
		"Block Mark Offset             : %u\n"
		"Cached Page Number            : %d\n"
		,
		nand->options,
		nand->numchips,
		nand->chipsize,
		nand->subpagesize,
		nand->page_shift,
		nand->pagemask,
		nand->phys_erase_shift,
		nand->bbt_erase_shift,
		nand->chip_shift,
		nand->badblockpos,
		nand->pagebuf
	);

	o += sprintf(buf + o,
		"ECC Byte Count       : %u\n"
		,
		nand->ecc.layout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < nand->ecc.layout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= nand->ecc.layout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				nand->ecc.layout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes  : %u\n"
		,
		nand->ecc.layout->oobavail
	);

	j = 0;

	for (i = 0; j < nand->ecc.layout->oobavail; i++) {

		j += nand->ecc.layout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			nand->ecc.layout->oobfree[i].offset,
			nand->ecc.layout->oobfree[i].length
		);

	}

	return o;

}

/**
 * show_device_mtd_info() - Shows the device's MTD-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct gpmi_nfc_data       *this = dev_get_drvdata(dev);
	struct mil                 *mil  = &this->mil;
	struct mtd_info            *mtd  = &mil->mtd;

	o += sprintf(buf + o,
		"Name               : %s\n"
		"Type               : %u\n"
		"Flags              : 0x%08x\n"
		"Size in Bytes      : %llu\n"
		"Erase Region Count : %d\n"
		"Erase Size in Bytes: %u\n"
		"Write Size in Bytes: %u\n"
		"OOB Size in Bytes  : %u\n"
		"Errors Corrected   : %u\n"
		"Failed Reads       : %u\n"
		"Bad Block Count    : %u\n"
		"BBT Block Count    : %u\n"
		,
		mtd->name,
		mtd->type,
		mtd->flags,
		mtd->size,
		mtd->numeraseregions,
		mtd->erasesize,
		mtd->writesize,
		mtd->oobsize,
		mtd->ecc_stats.corrected,
		mtd->ecc_stats.failed,
		mtd->ecc_stats.badblocks,
		mtd->ecc_stats.bbtblocks
	);

	o += sprintf(buf + o,
		"ECC Byte Count     : %u\n"
		,
		mtd->ecclayout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < mtd->ecclayout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= mtd->ecclayout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				mtd->ecclayout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes: %u\n"
		,
		mtd->ecclayout->oobavail
	);

	j = 0;

	for (i = 0; j < mtd->ecclayout->oobavail; i++) {

		j += mtd->ecclayout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			mtd->ecclayout->oobfree[i].offset,
			mtd->ecclayout->oobfree[i].length
		);

	}

	return o;

}

/**
 * store_device_mark_block_bad() - Marks a block as bad.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_mark_block_bad(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	struct mtd_info       *mtd  = &mil->mtd;
	struct nand_chip      *nand = &mil->nand;
	unsigned long         block_number;
	loff_t                byte_address;
	int                   error;

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the block number. */

	if (strict_strtoul(buf, 0, &block_number))
		return -EINVAL;

	/* Compute the byte address of this block. */

	byte_address = block_number << nand->phys_erase_shift;

	/* Attempt to mark the block bad. */

	error = mtd->block_markbad(mtd, byte_address);

	if (error)
		return error;

	/* Return success. */

	return size;

}

/**
 * show_device_ignorebad() - Shows the value of the 'ignorebad' flag.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_ignorebad(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;

	return sprintf(buf, "%d\n", mil->ignore_bad_block_marks);
}

/**
 * store_device_ignorebad() - Sets the value of the 'ignorebad' flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_ignorebad(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	const char            *p = buf;
	unsigned long         v;

	/* Try to make sense of what arrived from user space. */

	if (strict_strtoul(p, 0, &v) < 0)
		return size;

	if (v > 0)
		v = 1;

	/* Only do something if the value is changing. */

	if (v != mil->ignore_bad_block_marks) {

		if (v) {

			/*
			 * If control arrives here, we want to begin ignoring
			 * bad block marks. Reach into the NAND Flash MTD data
			 * structures and set the in-memory BBT pointer to NULL.
			 * This will cause the NAND Flash MTD code to believe
			 * that it never created a BBT and force it to call our
			 * block_bad function.
			 *
			 * See mil_block_bad for more details.
			 */

			mil->saved_bbt = mil->nand.bbt;
			mil->nand.bbt  = 0;

		} else {

			/*
			 * If control arrives here, we want to stop ignoring
			 * bad block marks. Restore the NAND Flash MTD's pointer
			 * to its in-memory BBT.
			 */

			mil->nand.bbt = mil->saved_bbt;

		}

		mil->ignore_bad_block_marks = v;

	}

	return size;

}

/**
 * show_device_inject_ecc_error() - Shows the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_inject_ecc_error(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;

	return sprintf(buf, "%d\n", mil->inject_ecc_error);

}

/**
 * store_device_inject_ecc_error() - Sets the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_inject_ecc_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);
	struct mil            *mil  = &this->mil;
	long                  new_inject_ecc_error;

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the ECC error count. */

	if (strict_strtol(buf, 0, &new_inject_ecc_error))
		return -EINVAL;

	/* Store the value. */

	mil->inject_ecc_error = new_inject_ecc_error;

	/* Return success. */

	return size;

}

/**
 * store_device_invalidate_page_cache() - Invalidates the device's page cache.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_invalidate_page_cache(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpmi_nfc_data  *this = dev_get_drvdata(dev);

	/* Invalidate the page cache. */

	this->mil.nand.pagebuf = -1;

	/* Return success. */

	return size;

}

/* Device attributes that appear in sysfs. */

static DEVICE_ATTR(numchips         , 0444, show_device_numchips         , 0);
static DEVICE_ATTR(physical_geometry, 0444, show_device_physical_geometry, 0);
static DEVICE_ATTR(nfc_info         , 0444, show_device_nfc_info         , 0);
static DEVICE_ATTR(nfc_geometry     , 0444, show_device_nfc_geometry     , 0);
static DEVICE_ATTR(rom_geometry     , 0444, show_device_rom_geometry     , 0);
static DEVICE_ATTR(mtd_nand_info    , 0444, show_device_mtd_nand_info    , 0);
static DEVICE_ATTR(mtd_info         , 0444, show_device_mtd_info         , 0);

static DEVICE_ATTR(mark_block_bad, 0200,
	0, store_device_mark_block_bad);

static DEVICE_ATTR(ignorebad, 0644,
	show_device_ignorebad, store_device_ignorebad);

static DEVICE_ATTR(inject_ecc_error, 0644,
	show_device_inject_ecc_error, store_device_inject_ecc_error);

static DEVICE_ATTR(invalidate_page_cache, 0644,
	0, store_device_invalidate_page_cache);

static struct device_attribute *device_attributes[] = {
	&dev_attr_numchips,
	&dev_attr_physical_geometry,
	&dev_attr_nfc_info,
	&dev_attr_nfc_geometry,
	&dev_attr_rom_geometry,
	&dev_attr_mtd_nand_info,
	&dev_attr_mtd_info,
	&dev_attr_mark_block_bad,
	&dev_attr_ignorebad,
	&dev_attr_inject_ecc_error,
	&dev_attr_invalidate_page_cache,
};

/**
 * validate_the_platform() - Validates information about the platform.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int validate_the_platform(struct platform_device *pdev)
{
	struct device                  *dev   = &pdev->dev;
	struct gpmi_nfc_platform_data  *pdata = pdev->dev.platform_data;

	/* Validate the clock name. */

	if (!pdata->clock_name) {
		dev_err(dev, "No clock name\n");
		return -ENXIO;
	}

	/* Validate the partitions. */

	if ((pdata->partitions && (!pdata->partition_count)) ||
			(!pdata->partitions && (pdata->partition_count))) {
		dev_err(dev, "Bad partition data\n");
		return -ENXIO;
	}

	/* Return success */

	return 0;

}

/**
 * acquire_register_block() - Tries to acquire and map a register block.
 *
 * @this:            Per-device data.
 * @resource_name:   The name of the resource.
 * @reg_block_base:  A pointer to a variable that will receive the address of
 *                   the mapped register block.
 */
static int acquire_register_block(struct gpmi_nfc_data *this,
			const char *resource_name, void **reg_block_base)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	void                    *p;
	struct resource         *r;

	/* Attempt to get information about the given resource. */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, resource_name);

	if (!r) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Attempt to remap the register block. */

	p = ioremap(r->start, r->end - r->start + 1);

	if (!p) {
		dev_err(dev, "Can't remap %s\n", resource_name);
		return -EIO;
	}

	/* If control arrives here, everything went fine. */

	*reg_block_base = p;

	return 0;

}

/**
 * release_register_block() - Releases a register block.
 *
 * @this:            Per-device data.
 * @reg_block_base:  A pointer to the mapped register block.
 */
static void release_register_block(struct gpmi_nfc_data *this,
							void *reg_block_base)
{
	iounmap(reg_block_base);
}

/**
 * acquire_interrupt() - Tries to acquire an interrupt.
 *
 * @this:              Per-device data.
 * @resource_name:     The name of the resource.
 * @interrupt_handler: A pointer to the function that will handle interrupts
 *                     from this interrupt number.
 * @interrupt_number:  A pointer to a variable that will receive the acquired
 *                     interrupt number.
 */
static int acquire_interrupt(
			struct gpmi_nfc_data *this, const char *resource_name,
			irq_handler_t interrupt_handler, int *interrupt_number)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	int                     error = 0;
	int                     i;

	/* Attempt to get information about the given resource. */

	i = platform_get_irq_byname(pdev, resource_name);

	if (i < 0) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Attempt to own the interrupt. */

	error = request_irq(i, interrupt_handler, 0, resource_name, this);

	if (error) {
		dev_err(dev, "Can't own %s\n", resource_name);
		return -EIO;
	}

	/* If control arrives here, everything went fine. */

	*interrupt_number = i;

	return 0;

}

/**
 * release_interrupt() - Releases an interrupt.
 *
 * @this:              Per-device data.
 * @interrupt_number:  The interrupt number.
 */
static void release_interrupt(struct gpmi_nfc_data *this, int interrupt_number)
{
	free_irq(interrupt_number, this);
}

/**
 * acquire_dma_channels() - Tries to acquire DMA channels.
 *
 * @this:           Per-device data.
 * @resource_name:  The name of the resource.
 * @low_channel:    A pointer to a variable that will receive the acquired
 *                  low DMA channel number.
 * @high_channel:   A pointer to a variable that will receive the acquired
 *                  high DMA channel number.
 */
static int acquire_dma_channels(
			struct gpmi_nfc_data *this, const char *resource_name,
			unsigned *low_channel, unsigned *high_channel)
{
	struct platform_device  *pdev = this->pdev;
	struct device           *dev  = this->dev;
	int                     error = 0;
	struct resource         *r;
	unsigned int            dma_channel;

	/* Attempt to get information about the given resource. */

	r = platform_get_resource_byname(pdev, IORESOURCE_DMA, resource_name);

	if (!r) {
		dev_err(dev, "Can't get resource information for '%s'\n",
								resource_name);
		return -ENXIO;
	}

	/* Loop over DMA channels, attempting to own them. */

	for (dma_channel = r->start; dma_channel <= r->end; dma_channel++) {

		/* Attempt to own the current channel. */

		error = mxs_dma_request(dma_channel, dev, resource_name);

		/* Check if we successfully acquired the current channel. */

		if (error) {

			dev_err(dev, "Can't acquire DMA channel %u\n",
								dma_channel);

			/* Free all the channels we've already acquired. */

			while (--dma_channel >= 0)
				mxs_dma_release(dma_channel, dev);

			return error;

		}

		/*
		 * If control arrives here, we successfully acquired the
		 * current channel. Continue initializing it.
		 */

		mxs_dma_reset(dma_channel);
		mxs_dma_ack_irq(dma_channel);

	}

	/* If control arrives here, all went well. */

	*low_channel  = r->start;
	*high_channel = r->end;

	return 0;

}

/**
 * release_dma_channels() - Releases DMA channels.
 *
 * @this:          Per-device data.
 * @low_channel:   The low DMA channel number.
 * @high_channel:  The high DMA channel number.
 */
static void release_dma_channels(struct gpmi_nfc_data *this,
				unsigned low_channel, unsigned high_channel)
{
	struct device  *dev = this->dev;
	unsigned int   i;

	for (i = low_channel; i <= high_channel; i++)
		mxs_dma_release(i, dev);
}

/**
 * acquire_clock() - Tries to acquire a clock.
 *
 * @this:           Per-device data.
 * @resource_name:  The name of the clock.
 * @high_channel:   A pointer to a variable that will receive the acquired
 *                  clock address.
 */
static int acquire_clock(struct gpmi_nfc_data *this,
				const char *clock_name, struct clk **clock)
{
	struct device  *dev  = this->dev;
	int            error = 0;
	struct clk     *c;

	/* Try to get the clock. */

	c = clk_get(dev, clock_name);

	if (IS_ERR(c)) {
		error = PTR_ERR(c);
		dev_err(dev, "Can't own clock %s\n", clock_name);
		return error;
	}

	/* If control arrives here, everything went fine. */

	*clock = c;

	return 0;

}

/**
 * release_clock() - Releases a clock.
 *
 * @this:   Per-device data.
 * @clock:  A pointer to the clock structure.
 */
static void release_clock(struct gpmi_nfc_data *this, struct clk *clock)
{
	clk_disable(clock);
	clk_put(clock);
}

/**
 * acquire_resources() - Tries to acquire resources.
 *
 * @this:  Per-device data.
 */
static int acquire_resources(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata     =  this->pdata;
	struct resources               *resources = &this->resources;
	int                            error      = 0;

	/* Attempt to acquire the GPMI register block. */

	error = acquire_register_block(this,
		GPMI_NFC_GPMI_REGS_ADDR_RES_NAME, &(resources->gpmi_regs));

	if (error)
		goto exit_gpmi_regs;

	/* Attempt to acquire the BCH register block. */

	error = acquire_register_block(this,
		GPMI_NFC_BCH_REGS_ADDR_RES_NAME, &(resources->bch_regs));

	if (error)
		goto exit_bch_regs;

	/* Attempt to acquire the BCH interrupt. */

	error = acquire_interrupt(this,
		GPMI_NFC_BCH_INTERRUPT_RES_NAME,
		nfc_bch_isr, &(resources->bch_interrupt));

	if (error)
		goto exit_bch_interrupt;

	/* Attempt to acquire the DMA channels. */

	error = acquire_dma_channels(this,
		GPMI_NFC_DMA_CHANNELS_RES_NAME,
		&(resources->dma_low_channel), &(resources->dma_high_channel));

	if (error)
		goto exit_dma_channels;

	/* Attempt to acquire the DMA interrupt. */

	error = acquire_interrupt(this,
		GPMI_NFC_DMA_INTERRUPT_RES_NAME,
		nfc_dma_isr, &(resources->dma_interrupt));

	if (error)
		goto exit_dma_interrupt;

	/* Attempt to acquire our clock. */

	error = acquire_clock(this, pdata->clock_name, &(resources->clock));

	if (error)
		goto exit_clock;

	/* If control arrives here, all went well. */

	return 0;

	/* Control arrives here if something went wrong. */

exit_clock:
	release_interrupt(this, resources->dma_interrupt);
exit_dma_interrupt:
	release_dma_channels(this,
		resources->dma_low_channel, resources->dma_high_channel);
exit_dma_channels:
	release_interrupt(this, resources->bch_interrupt);
exit_bch_interrupt:
	release_register_block(this, resources->bch_regs);
exit_bch_regs:
	release_register_block(this, resources->gpmi_regs);
exit_gpmi_regs:

	return error;

}

/**
 * release_resources() - Releases resources.
 *
 * @this:  Per-device data.
 */
static void release_resources(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	release_clock(this, resources->clock);
	release_register_block(this, resources->gpmi_regs);
	release_register_block(this, resources->bch_regs);
	release_interrupt(this, resources->bch_interrupt);
	release_dma_channels(this,
		resources->dma_low_channel, resources->dma_high_channel);
	release_interrupt(this, resources->dma_interrupt);
}

/**
 * set_up_nfc_hal() - Sets up the NFC HAL.
 *
 * @this:  Per-device data.
 */
static int set_up_nfc_hal(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata = this->pdata;
	struct device                  *dev   = this->dev;
	struct nfc_hal                 *nfc;
	int                            error = 0;
	unsigned int                   i;

	/* Attempt to find an NFC HAL that matches the given version. */

	for (i = 0; i < ARRAY_SIZE(nfc_hals); i++) {

		nfc = nfc_hals[i];

		if (nfc->version == pdata->nfc_version) {
			this->nfc = nfc;
			break;
		}

	}

	/* Check if we found a HAL. */

	if (i >= ARRAY_SIZE(nfc_hals)) {
		dev_err(dev, "Unkown NFC version %u\n", pdata->nfc_version);
		return -ENXIO;
	}

	pr_info("NFC: Version %u, %s\n", nfc->version, nfc->description);

	/*
	 * Check if we can handle the number of chips called for by the platform
	 * data.
	 */

	if (pdata->max_chip_count > nfc->max_chip_count) {
		dev_err(dev, "Platform data calls for %u chips "
				"but NFC supports a max of %u.\n",
				pdata->max_chip_count, nfc->max_chip_count);
		return -ENXIO;
	}

	/* Initialize the NFC HAL. */

	error = nfc->init(this);

	if (error)
		return error;

	/*
	 * If control arrives here, all is well.
	 */

	return 0;

}

/**
 * set_up_boot_rom_helper() - Sets up the Boot ROM Helper.
 *
 * @this:  Per-device data.
 */
static int set_up_boot_rom_helper(struct gpmi_nfc_data *this)
{
	struct gpmi_nfc_platform_data  *pdata = this->pdata;
	struct device                  *dev   = this->dev;
	unsigned int                   i;
	struct boot_rom_helper         *rom;

	/* Attempt to find a Boot ROM Helper that matches the given version. */

	for (i = 0; i < ARRAY_SIZE(boot_rom_helpers); i++) {

		rom = boot_rom_helpers[i];

		if (rom->version == pdata->boot_rom_version) {
			this->rom = rom;
			break;
		}

	}

	/* Check if we found a Boot ROM Helper. */

	if (i >= ARRAY_SIZE(boot_rom_helpers)) {
		dev_err(dev, "Unkown Boot ROM version %u\n",
						pdata->boot_rom_version);
		return -ENXIO;
	}

	pr_info("Boot ROM: Version %u, %s\n", rom->version, rom->description);

	/*
	 * If control arrives here, all is well.
	 */

	return 0;

}

/**
 * manage_sysfs_files() - Creates/removes sysfs files for this device.
 *
 * @this:  Per-device data.
 */
static void manage_sysfs_files(struct gpmi_nfc_data *this, int create)
{
	struct device            *dev = this->dev;
	int                      error;
	unsigned int             i;
	struct device_attribute  **attr;

	for (i = 0, attr = device_attributes;
			i < ARRAY_SIZE(device_attributes); i++, attr++) {

		if (create) {
			error = device_create_file(dev, *attr);
			if (error) {
				while (--attr >= device_attributes)
					device_remove_file(dev, *attr);
				return;
			}
		} else {
			device_remove_file(dev, *attr);
		}

	}

}

/**
 * gpmi_nfc_probe() - Probes for a device and, if possible, takes ownership.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int gpmi_nfc_probe(struct platform_device *pdev)
{
	int                            error  = 0;
	struct device                  *dev   = &pdev->dev;
	struct gpmi_nfc_platform_data  *pdata = pdev->dev.platform_data;
	struct gpmi_nfc_data           *this  = 0;

	/* Validate the platform device data. */

	error = validate_the_platform(pdev);

	if (error)
		goto exit_validate_platform;

	/* Allocate memory for the per-device data. */

	this = kzalloc(sizeof(*this), GFP_KERNEL);

	if (!this) {
		dev_err(dev, "Failed to allocate per-device memory\n");
		error = -ENOMEM;
		goto exit_allocate_this;
	}

	/* Set up our data structures. */

	platform_set_drvdata(pdev, this);

	this->pdev  = pdev;
	this->dev   = &pdev->dev;
	this->pdata = pdata;

	/* Acquire the resources we need. */

	error = acquire_resources(this);

	if (error)
		goto exit_acquire_resources;

	/* Set up the NFC. */

	error = set_up_nfc_hal(this);

	if (error)
		goto exit_nfc_init;

	/* Set up the platform. */

	if (pdata->platform_init)
		error = pdata->platform_init(pdata->max_chip_count);

	if (error)
		goto exit_platform_init;

	/* Set up the Boot ROM Helper. */

	error = set_up_boot_rom_helper(this);

	if (error)
		goto exit_boot_rom_helper_init;

	/* Initialize the MTD Interface Layer. */

	error = mil_init(this);

	if (error)
		goto exit_mil_init;

	/* Create sysfs entries for this device. */

	manage_sysfs_files(this, true);

	/* Return success. */

	return 0;

	/* Error return paths begin here. */

exit_mil_init:
exit_boot_rom_helper_init:
	if (pdata->platform_exit)
		pdata->platform_exit(pdata->max_chip_count);
exit_platform_init:
	this->nfc->exit(this);
exit_nfc_init:
	release_resources(this);
exit_acquire_resources:
	platform_set_drvdata(pdev, NULL);
	kfree(this);
exit_allocate_this:
exit_validate_platform:
	return error;

}

/**
 * gpmi_nfc_remove() - Dissociates this driver from the given device.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int __exit gpmi_nfc_remove(struct platform_device *pdev)
{
	struct gpmi_nfc_data           *this = platform_get_drvdata(pdev);
	struct gpmi_nfc_platform_data  *pdata = this->pdata;

	manage_sysfs_files(this, false);
	mil_exit(this);
	if (pdata->platform_exit)
		pdata->platform_exit(pdata->max_chip_count);
	this->nfc->exit(this);
	release_resources(this);
	platform_set_drvdata(pdev, NULL);
	kfree(this);

	return 0;
}

#ifdef CONFIG_PM

/**
 * gpmi_nfc_suspend() - Puts the NFC into a low power state.
 *
 * @pdev:  A pointer to the platform device data structure.
 * @state: The new power state.
 */
static int gpmi_nfc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

/**
 * gpmi_nfc_resume() - Brings the NFC back from a low power state.
 *
 * @pdev:  A pointer to the platform device data structure.
 */
static int gpmi_nfc_resume(struct platform_device *pdev)
{
	return 0;
}

#else

#define suspend  NULL
#define resume   NULL

#endif  /* CONFIG_PM */

/*
 * This structure represents this driver to the platform management system.
 */
static struct platform_driver gpmi_nfc_driver = {
	.driver = {
		.name = GPMI_NFC_DRIVER_NAME,
	},
	.probe   = gpmi_nfc_probe,
	.remove  = __exit_p(gpmi_nfc_remove),
	.suspend = gpmi_nfc_suspend,
	.resume  = gpmi_nfc_resume,
};

/**
 * gpmi_nfc_init() - Initializes this module.
 */
static int __init gpmi_nfc_init(void)
{

	pr_info("i.MX GPMI NFC\n");

	/* Register this driver with the platform management system. */

	if (platform_driver_register(&gpmi_nfc_driver) != 0) {
		pr_err("i.MX GPMI NFC driver registration failed\n");
		return -ENODEV;
	}

	/* Return success. */

	return 0;

}

/**
 * gpmi_nfc_exit() - Deactivates this module.
 */
static void __exit gpmi_nfc_exit(void)
{
	platform_driver_unregister(&gpmi_nfc_driver);
}

module_init(gpmi_nfc_init);
module_exit(gpmi_nfc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX GPMI NAND Flash Controller Driver");
MODULE_LICENSE("GPL");
