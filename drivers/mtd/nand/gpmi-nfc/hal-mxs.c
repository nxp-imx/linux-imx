/*
 * Freescale GPMI NFC NAND Flash Driver
 *
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc.
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
#include "gpmi-regs.h"
#include "bch-regs.h"

static int init_hal(struct gpmi_nfc_data *this)
{
	struct resources *resources = &this->resources;

	/* Enable the clock */
	clk_enable(resources->clock);

	/* Reset the GPMI block. */
	mxs_reset_block(resources->gpmi_regs);

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
	return 0;
}

/* Configures the NFC geometry for BCH.  */
static int set_geometry(struct gpmi_nfc_data *this)
{
	struct resources *resources = &this->resources;
	struct nfc_geometry *nfc = &this->nfc_geometry;
	unsigned int block_count;
	unsigned int block_size;
	unsigned int metadata_size;
	unsigned int ecc_strength;
	unsigned int page_size;

	if (common_nfc_set_geometry(this))
		return !0;

	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes;
	metadata_size = nfc->metadata_size_in_bytes;
	ecc_strength  = nfc->ecc_strength >> 1;
	page_size     = nfc->page_size_in_bytes;

	clk_enable(resources->clock);

	/*
	 * Reset the BCH block. Notice that we pass in true for the just_enable
	 * flag. This is because the soft reset for the version 0 BCH block
	 * doesn't work. If you try to soft reset the BCH block, it becomes
	 * unusable until the next hard reset.
	 */
	mxs_reset_block(resources->bch_regs);

	/* Configure layout 0. */
	__raw_writel(BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)
		| BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size)
		| BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)
		| BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size),
		resources->bch_regs + HW_BCH_FLASH0LAYOUT0);

	__raw_writel(BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)
		| BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)
		| BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size),
		resources->bch_regs + HW_BCH_FLASH0LAYOUT1);

	/* Set *all* chip selects to use layout 0. */
	__raw_writel(0, resources->bch_regs + HW_BCH_LAYOUTSELECT);

	/* Enable interrupts. */
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ_EN,
				resources->bch_regs + HW_BCH_CTRL_SET);

	clk_disable(resources->clock);
	return 0;
}

static int set_timing(struct gpmi_nfc_data *this,
			const struct nand_timing *timing)
{
	struct nfc_hal *nfc = this->nfc;

	nfc->timing = *timing;
	return 0;
}

/**
 * get_timing() - Retrieves the NFC hardware timing.
 *
 * @this:                    Per-device data.
 * @clock_frequency_in_hz:   The clock frequency, in Hz, during the current
 *                           I/O transaction. If no I/O transaction is in
 *                           progress, this is the clock frequency during the
 *                           most recent I/O transaction.
 * @hardware_timing:         The hardware timing configuration in effect during
 *                           the current I/O transaction. If no I/O transaction
 *                           is in progress, this is the hardware timing
 *                           configuration during the most recent I/O
 *                           transaction.
 */
static void get_timing(struct gpmi_nfc_data *this,
			unsigned long *clock_frequency_in_hz,
			struct gpmi_nfc_hardware_timing *hardware_timing)
{
	struct resources *resources = &this->resources;
	struct nfc_hal *nfc = this->nfc;
	unsigned char *gpmi_regs = resources->gpmi_regs;
	uint32_t register_image;

	/* Return the clock frequency. */
	*clock_frequency_in_hz = nfc->clock_frequency_in_hz;

	/* We'll be reading the hardware, so let's enable the clock. */
	clk_enable(resources->clock);

	/* Retrieve the hardware timing. */
	register_image = __raw_readl(gpmi_regs + HW_GPMI_TIMING0);

	hardware_timing->data_setup_in_cycles =
		(register_image & BM_GPMI_TIMING0_DATA_SETUP) >>
						BP_GPMI_TIMING0_DATA_SETUP;

	hardware_timing->data_hold_in_cycles =
		(register_image & BM_GPMI_TIMING0_DATA_HOLD) >>
						BP_GPMI_TIMING0_DATA_HOLD;

	hardware_timing->address_setup_in_cycles =
		(register_image & BM_GPMI_TIMING0_ADDRESS_SETUP) >>
						BP_GPMI_TIMING0_ADDRESS_SETUP;

	register_image = __raw_readl(gpmi_regs + HW_GPMI_CTRL1);

	hardware_timing->use_half_periods =
		(register_image & BM_GPMI_CTRL1_HALF_PERIOD) >>
						BP_GPMI_CTRL1_HALF_PERIOD;

	hardware_timing->sample_delay_factor =
		(register_image & BM_GPMI_CTRL1_RDN_DELAY) >>
						BP_GPMI_CTRL1_RDN_DELAY;

	/* We're done reading the hardware, so disable the clock. */
	clk_disable(resources->clock);
}

static void exit(struct gpmi_nfc_data *this)
{
}

/* Begin the I/O */
static void begin(struct gpmi_nfc_data *this)
{
	struct resources *resources = &this->resources;
	struct nfc_hal *nfc = this->nfc;
	unsigned char  *gpmi_regs = resources->gpmi_regs;
	unsigned int   clock_period_in_ns;
	uint32_t       register_image;
	unsigned int   dll_wait_time_in_us;
	struct gpmi_nfc_hardware_timing  hw;

	/* Enable the clock. */
	clk_enable(resources->clock);

	/* set the timing for imx23 */
	if (!GPMI_IS_MX23(this))
		return;

	/* Get the timing information we need. */
	nfc->clock_frequency_in_hz = clk_get_rate(resources->clock);
	clock_period_in_ns = 1000000000 / nfc->clock_frequency_in_hz;

	gpmi_nfc_compute_hardware_timing(this, &hw);

	/* Set up all the simple timing parameters. */
	register_image =
		BF_GPMI_TIMING0_ADDRESS_SETUP(hw.address_setup_in_cycles) |
		BF_GPMI_TIMING0_DATA_HOLD(hw.data_hold_in_cycles)         |
		BF_GPMI_TIMING0_DATA_SETUP(hw.data_setup_in_cycles)       ;

	__raw_writel(register_image, gpmi_regs + HW_GPMI_TIMING0);

	/*
	 * HEY - PAY ATTENTION!
	 *
	 * DLL_ENABLE must be set to zero when setting RDN_DELAY or HALF_PERIOD.
	 */
	__raw_writel(BM_GPMI_CTRL1_DLL_ENABLE, gpmi_regs + HW_GPMI_CTRL1_CLR);

	/* Clear out the DLL control fields. */
	__raw_writel(BM_GPMI_CTRL1_RDN_DELAY,   gpmi_regs + HW_GPMI_CTRL1_CLR);
	__raw_writel(BM_GPMI_CTRL1_HALF_PERIOD, gpmi_regs + HW_GPMI_CTRL1_CLR);

	/* If no sample delay is called for, return immediately. */
	if (!hw.sample_delay_factor)
		return;

	/* Configure the HALF_PERIOD flag. */
	if (hw.use_half_periods)
		__raw_writel(BM_GPMI_CTRL1_HALF_PERIOD,
						gpmi_regs + HW_GPMI_CTRL1_SET);

	/* Set the delay factor. */
	__raw_writel(BF_GPMI_CTRL1_RDN_DELAY(hw.sample_delay_factor),
						gpmi_regs + HW_GPMI_CTRL1_SET);

	/* Enable the DLL. */
	__raw_writel(BM_GPMI_CTRL1_DLL_ENABLE, gpmi_regs + HW_GPMI_CTRL1_SET);

	/*
	 * After we enable the GPMI DLL, we have to wait 64 clock cycles before
	 * we can use the GPMI.
	 *
	 * Calculate the amount of time we need to wait, in microseconds.
	 */
	dll_wait_time_in_us = (clock_period_in_ns * 64) / 1000;

	if (!dll_wait_time_in_us)
		dll_wait_time_in_us = 1;

	/* Wait for the DLL to settle. */
	udelay(dll_wait_time_in_us);
}

static void end(struct gpmi_nfc_data *this)
{
	struct resources *r = &this->resources;
	clk_disable(r->clock);
}

/* Clears a BCH interrupt. */
static void clear_bch(struct gpmi_nfc_data *this)
{
	struct resources *r = &this->resources;
	__raw_writel(BM_BCH_CTRL_COMPLETE_IRQ, r->bch_regs + HW_BCH_CTRL_CLR);
}

/* Returns the Ready/Busy status of the given chip. */
static int is_ready(struct gpmi_nfc_data *this, unsigned chip)
{
	struct resources *r = &this->resources;
	uint32_t mask;
	uint32_t reg;

	if (GPMI_IS_MX23(this)) {
		mask = MX23_BM_GPMI_DEBUG_READY0 << chip;
		reg = __raw_readl(r->gpmi_regs + HW_GPMI_DEBUG);
	} else if (GPMI_IS_MX28(this)) {
		mask = MX28_BF_GPMI_STAT_READY_BUSY(1 << chip);
		reg = __raw_readl(r->gpmi_regs + HW_GPMI_STAT);
	} else
		BUG();
	return !!(reg & mask);
}

static int send_command(struct gpmi_nfc_data *this)
{
	struct dma_chan *channel = get_dma_chan(this);
	struct mil *mil	= &this->mil;
	struct dma_async_tx_descriptor *desc;
	struct scatterlist *sgl;
	u32 pio[3];

	/* [1] send out the PIO words */
	pio[0] = BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__WRITE)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(mil->current_chip, this)
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_CLE)
		| BM_GPMI_CTRL0_ADDRESS_INCREMENT
		| BF_GPMI_CTRL0_XFER_COUNT(mil->command_length);
	pio[1] = pio[2] = 0;
	desc = channel->device->device_prep_slave_sg(channel,
					(struct scatterlist *)pio,
					ARRAY_SIZE(pio), DMA_NONE, 0);
	if (!desc) {
		pr_info("step 1 error\n");
		return -1;
	}

	/* [2] send out the COMMAND + ADDRESS string stored in @buffer */
	sgl = &mil->cmd_sgl;

	sg_init_one(sgl, mil->cmd_buffer, mil->command_length);
	dma_map_sg(this->dev, sgl, 1, DMA_TO_DEVICE);
	desc = channel->device->device_prep_slave_sg(channel,
					sgl, 1, DMA_TO_DEVICE, 1);
	if (!desc) {
		pr_info("step 2 error\n");
		return -1;
	}

	/* [3] submit the DMA */
	this->dma_type = DMA_FOR_COMMAND;
	start_dma_without_bch_irq(this, desc);
	return 0;
}

static int send_data(struct gpmi_nfc_data *this)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *channel = get_dma_chan(this);
	struct mil *mil	= &this->mil;
	uint32_t command_mode;
	uint32_t address;
	u32 pio[2];

	/* [1] PIO */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	pio[0] = BF_GPMI_CTRL0_COMMAND_MODE(command_mode)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(mil->current_chip, this)
		| BF_GPMI_CTRL0_ADDRESS(address)
		| BF_GPMI_CTRL0_XFER_COUNT(mil->upper_len);
	pio[1] = 0;
	desc = channel->device->device_prep_slave_sg(channel,
					(struct scatterlist *)pio,
					ARRAY_SIZE(pio), DMA_NONE, 0);
	if (!desc) {
		pr_info("step 1 error\n");
		return -1;
	}

	/* [2]  send DMA request */
	prepare_data_dma(this, DMA_TO_DEVICE);
	desc = channel->device->device_prep_slave_sg(channel, &mil->data_sgl,
						1, DMA_TO_DEVICE, 1);
	if (!desc) {
		pr_info("step 2 error\n");
		return -1;
	}
	/* [3] submit the DMA */
	this->dma_type = DMA_FOR_WRITE_DATA;
	start_dma_without_bch_irq(this, desc);
	return 0;
}

static int read_data(struct gpmi_nfc_data *this)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *channel = get_dma_chan(this);
	struct mil *mil = &this->mil;
	u32 pio[2];

	/* [1] : send PIO */
	pio[0] = BF_GPMI_CTRL0_COMMAND_MODE(BV_GPMI_CTRL0_COMMAND_MODE__READ)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(mil->current_chip, this)
		| BF_GPMI_CTRL0_ADDRESS(BV_GPMI_CTRL0_ADDRESS__NAND_DATA)
		| BF_GPMI_CTRL0_XFER_COUNT(mil->upper_len);
	pio[1] = 0;
	desc = channel->device->device_prep_slave_sg(channel,
					(struct scatterlist *)pio,
					ARRAY_SIZE(pio), DMA_NONE, 0);
	if (!desc) {
		pr_info("step 1 error\n");
		return -1;
	}

	/* [2] : send DMA request */
	prepare_data_dma(this, DMA_FROM_DEVICE);
	desc = channel->device->device_prep_slave_sg(channel, &mil->data_sgl,
						1, DMA_FROM_DEVICE, 1);
	if (!desc) {
		pr_info("step 2 error\n");
		return -1;
	}

	/* [3] : submit the DMA */
	this->dma_type = DMA_FOR_READ_DATA;
	start_dma_without_bch_irq(this, desc);
	return 0;
}

static int send_page(struct gpmi_nfc_data *this,
			dma_addr_t payload, dma_addr_t auxiliary)
{
	struct nfc_geometry *geo = &this->nfc_geometry;
	uint32_t command_mode;
	uint32_t address;
	uint32_t ecc_command;
	uint32_t buffer_mask;
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *channel = get_dma_chan(this);
	struct mil *mil = &this->mil;
	int chip = mil->current_chip;
	u32 pio[6];

	/* A DMA descriptor that does an ECC page read. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__BCH_ENCODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	pio[0] = BF_GPMI_CTRL0_COMMAND_MODE(command_mode)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(chip, this)
		| BF_GPMI_CTRL0_ADDRESS(address)
		| BF_GPMI_CTRL0_XFER_COUNT(0);
	pio[1] = 0;
	pio[2] = BM_GPMI_ECCCTRL_ENABLE_ECC
		| BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)
		| BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask);
	pio[3] = geo->page_size_in_bytes;
	pio[4] = payload;
	pio[5] = auxiliary;

	desc = channel->device->device_prep_slave_sg(channel,
					(struct scatterlist *)pio,
					ARRAY_SIZE(pio), DMA_NONE, 0);
	if (!desc) {
		pr_info("step 2 error\n");
		return -1;
	}
	this->dma_type = DMA_FOR_WRITE_ECC_PAGE;
	return start_dma_with_bch_irq(this, desc);
}

static int read_page(struct gpmi_nfc_data *this,
				dma_addr_t payload, dma_addr_t auxiliary)
{
	struct nfc_geometry *geo = &this->nfc_geometry;
	uint32_t command_mode;
	uint32_t address;
	uint32_t ecc_command;
	uint32_t buffer_mask;
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *channel = get_dma_chan(this);
	struct mil *mil = &this->mil;
	int chip = mil->current_chip;
	u32 pio[6];

	/* [1] Wait for the chip to report ready. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	pio[0] =  BF_GPMI_CTRL0_COMMAND_MODE(command_mode)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(chip, this)
		| BF_GPMI_CTRL0_ADDRESS(address)
		| BF_GPMI_CTRL0_XFER_COUNT(0);
	pio[1] = 0;
	desc = channel->device->device_prep_slave_sg(channel,
				(struct scatterlist *)pio, 2, DMA_NONE, 0);
	if (!desc) {
		pr_info("step 1 error\n");
		return -1;
	}

	/* [2] Enable the BCH block and read. */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__BCH_DECODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE
			| BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	pio[0] =  BF_GPMI_CTRL0_COMMAND_MODE(command_mode)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(chip, this)
		| BF_GPMI_CTRL0_ADDRESS(address)
		| BF_GPMI_CTRL0_XFER_COUNT(geo->page_size_in_bytes);

	pio[1] = 0;
	pio[2] =  BM_GPMI_ECCCTRL_ENABLE_ECC
		| BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)
		| BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask);
	pio[3] = geo->page_size_in_bytes;
	pio[4] = payload;
	pio[5] = auxiliary;
	desc = channel->device->device_prep_slave_sg(channel,
					(struct scatterlist *)pio,
					ARRAY_SIZE(pio), DMA_NONE, 1);
	if (!desc) {
		pr_info("step 2 error\n");
		return -1;
	}

	/* [3] Disable the BCH block */
	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	pio[0] = BF_GPMI_CTRL0_COMMAND_MODE(command_mode)
		| BM_GPMI_CTRL0_WORD_LENGTH
		| BF_GPMI_CTRL0_CS(chip, this)
		| BF_GPMI_CTRL0_ADDRESS(address)
		| BF_GPMI_CTRL0_XFER_COUNT(geo->page_size_in_bytes);
	pio[1] = 0;
	desc = channel->device->device_prep_slave_sg(channel,
				(struct scatterlist *)pio, 2, DMA_NONE, 1);
	if (!desc) {
		pr_info("step 3 error\n");
		return -1;
	}

	/* [4] submit the DMA */
	this->dma_type = DMA_FOR_READ_ECC_PAGE;
	return start_dma_with_bch_irq(this, desc);
}

struct nfc_hal  gpmi_nfc_hal_imx23_imx28 = {
	.description                 = "GPMI and BCH for IMX23/IMX28",
	.max_data_setup_cycles       = (BM_GPMI_TIMING0_DATA_SETUP >>
						BP_GPMI_TIMING0_DATA_SETUP),
	.internal_data_setup_in_ns   = 0,
	.max_sample_delay_factor     = (BM_GPMI_CTRL1_RDN_DELAY >>
						BP_GPMI_CTRL1_RDN_DELAY),
	.max_dll_clock_period_in_ns  = 32,
	.max_dll_delay_in_ns         = 16,
	.init                        = init_hal,
	.set_geometry                = set_geometry,
	.set_timing                  = set_timing,
	.get_timing                  = get_timing,
	.exit                        = exit,
	.begin                       = begin,
	.end                         = end,
	.clear_bch                   = clear_bch,
	.is_ready                    = is_ready,
	.send_command                = send_command,
	.read_data                   = read_data,
	.send_data                   = send_data,
	.read_page                   = read_page,
	.send_page                   = send_page,
};
