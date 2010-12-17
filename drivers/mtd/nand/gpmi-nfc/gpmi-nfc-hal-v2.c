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

#include "gpmi-nfc-gpmi-regs-v2.h"
#include "gpmi-nfc-bch-regs-v2.h"

#define FEATURE_SIZE		(4)	/* p1, p2, p3, p4 */

/*
 * In DDR mode of ONFI NAND, the data READ/WRITE will become 16-bit,
 * although the ALE/CLE are still use the 8-bit.
 */
static int onfi_ddr_mode;

/*
 * How many clocks do we need in low power mode?
 * We try to list them :
 *	GMPI		: gpmi_apb_clk, gpmi_io_clk
 *	BCH		: bch_clk, bch_apb_clk
 *	DMA(RAM)	: apbh_dma_clk, ddr_clk(RAM), ahb_max_clk(RAM)
 *			  (APBHDMA fetches DMA descriptors from DDR
 *			   through AHB-MAX/PL301)
 *	NAND		:
 *	ONFI NAND	: pll1_main_clk
 */
static struct clk *ddr_clk;
static struct clk *apbh_dma_clk;
static struct clk *ahb_max_clk;

static void setup_ddr_timing_onfi(struct gpmi_nfc_data *this)
{
	uint32_t value;
	struct resources  *resources = &this->resources;

	/* set timing 2 register */
	value = BF_GPMI_TIMING2_DATA_PAUSE(0x6)
		| BF_GPMI_TIMING2_CMDADD_PAUSE(0x4)
		| BF_GPMI_TIMING2_POSTAMBLE_DELAY(0x2)
		| BF_GPMI_TIMING2_PREAMBLE_DELAY(0x4)
		| BF_GPMI_TIMING2_CE_DELAY(0x2)
		| BF_GPMI_TIMING2_READ_LATENCY(0x2);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_TIMING2);

	/* set timing 1 register */
	__raw_writel(BF_GPMI_TIMING1_DEVICE_BUSY_TIMEOUT(0x500),
			resources->gpmi_regs + HW_GPMI_TIMING1);

	/* Put GPMI in NAND mode, disable device reset, and make certain
	   IRQRDY polarity is active high. */
	value = BV_GPMI_CTRL1_GPMI_MODE__NAND
		| BM_GPMI_CTRL1_GANGED_RDYBUSY
		| BF_GPMI_CTRL1_WRN_DLY_SEL(0x3)
		| (BV_GPMI_CTRL1_DEV_RESET__DISABLED << 3)
		| (BV_GPMI_CTRL1_ATA_IRQRDY_POLARITY__ACTIVEHIGH << 2);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_CTRL1_SET);
}

static int enable_ddr_onfi(struct gpmi_nfc_data *this)
{
	uint32_t value;
	struct resources  *resources = &this->resources;
	struct mil *mil	= &this->mil;
	struct nand_chip *nand = &this->mil.nand;
	struct mtd_info	 *mtd = &mil->mtd;
	int saved_chip_number = 0;
	uint8_t device_feature[FEATURE_SIZE];
	int mode = 0;/* there is 5 mode available, default is 0 */

	saved_chip_number = mil->current_chip;
	nand->select_chip(mtd, 0);

	/* [0] set proper timing */
	__raw_writel(BF_GPMI_TIMING0_ADDRESS_SETUP(0x1)
			| BF_GPMI_TIMING0_DATA_HOLD(0x3)
			| BF_GPMI_TIMING0_DATA_SETUP(0x3),
			resources->gpmi_regs + HW_GPMI_TIMING0);

	/* [1] send SET FEATURE commond to NAND */
	memset(device_feature, 0, sizeof(device_feature));
	device_feature[0] = (0x1 << 4) | (mode & 0x7);

	nand->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	nand->cmdfunc(mtd, NAND_CMD_SET_FEATURE, 1, -1);
	nand->write_buf(mtd, device_feature, FEATURE_SIZE);

	/* [2] set clk divider */
	__raw_writel(BM_GPMI_CTRL1_GPMI_CLK_DIV2_EN,
			resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* [3] about the clock, pay attention! */
	nand->select_chip(mtd, saved_chip_number);
	{
		struct clk *pll1;
		pll1 = clk_get(NULL, "pll1_main_clk");
		if (IS_ERR(pll1)) {
			printk(KERN_INFO "No PLL1 clock\n");
			return -EINVAL;
		}
		clk_set_parent(resources->clock, pll1);
		clk_set_rate(resources->clock, 20000000);
	}
	nand->select_chip(mtd, 0);

	/* [4] setup timing */
	setup_ddr_timing_onfi(this);

	/* [5] set to SYNC mode */
	__raw_writel(BM_GPMI_CTRL1_TOGGLE_MODE,
			    resources->gpmi_regs + HW_GPMI_CTRL1_CLR);
	__raw_writel(BM_GPMI_CTRL1_SSYNCMODE | BM_GPMI_CTRL1_GANGED_RDYBUSY,
			    resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* [6] enable both write & read DDR DLLs */
	value = BM_GPMI_READ_DDR_DLL_CTRL_REFCLK_ON |
		BM_GPMI_READ_DDR_DLL_CTRL_ENABLE |
		BF_GPMI_READ_DDR_DLL_CTRL_SLV_UPDATE_INT(0x2) |
		BF_GPMI_READ_DDR_DLL_CTRL_SLV_DLY_TARGET(0x7);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	/* [7] reset read */
	__raw_writel(value | BM_GPMI_READ_DDR_DLL_CTRL_RESET,
			resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	value = value & ~BM_GPMI_READ_DDR_DLL_CTRL_RESET;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = BM_GPMI_WRITE_DDR_DLL_CTRL_REFCLK_ON |
		BM_GPMI_WRITE_DDR_DLL_CTRL_ENABLE    |
		BF_GPMI_WRITE_DDR_DLL_CTRL_SLV_UPDATE_INT(0x2) |
		BF_GPMI_WRITE_DDR_DLL_CTRL_SLV_DLY_TARGET(0x7) ,

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [8] reset write */
	__raw_writel(value | BM_GPMI_WRITE_DDR_DLL_CTRL_RESET,
			resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [9] wait for locks for read and write  */
	do {
		uint32_t read_status, write_status;
		uint32_t r_mask, w_mask;

		read_status = __raw_readl(resources->gpmi_regs
					+ HW_GPMI_READ_DDR_DLL_STS);
		write_status = __raw_readl(resources->gpmi_regs
					+ HW_GPMI_WRITE_DDR_DLL_STS);

		r_mask = (BM_GPMI_READ_DDR_DLL_STS_REF_LOCK |
				BM_GPMI_READ_DDR_DLL_STS_SLV_LOCK);
		w_mask = (BM_GPMI_WRITE_DDR_DLL_STS_REF_LOCK |
				BM_GPMI_WRITE_DDR_DLL_STS_SLV_LOCK);

		if (((read_status & r_mask) == r_mask)
			&& ((write_status & w_mask) == w_mask))
				break;
	} while (1);

	/* [10] force update of read/write */
	value = __raw_readl(resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	__raw_writel(value | BM_GPMI_READ_DDR_DLL_CTRL_SLV_FORCE_UPD,
			resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = __raw_readl(resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value | BM_GPMI_WRITE_DDR_DLL_CTRL_SLV_FORCE_UPD,
			resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [11] set gate update */
	value = __raw_readl(resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	value |= BM_GPMI_READ_DDR_DLL_CTRL_GATE_UPDATE;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = __raw_readl(resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	value |= BM_GPMI_WRITE_DDR_DLL_CTRL_GATE_UPDATE;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	onfi_ddr_mode = 1;
	nand->select_chip(mtd, saved_chip_number);

	printk(KERN_INFO "Micron ONFI NAND enters synchronous mode %d\n", mode);
	return 0;
}

static void setup_ddr_timing_toggle(struct gpmi_nfc_data *this)
{
	uint32_t value;
	struct resources  *resources = &this->resources;

	/* set timing 2 register */
	value = BF_GPMI_TIMING2_DATA_PAUSE(0x6)
		| BF_GPMI_TIMING2_CMDADD_PAUSE(0x4)
		| BF_GPMI_TIMING2_POSTAMBLE_DELAY(0x3)
		| BF_GPMI_TIMING2_PREAMBLE_DELAY(0x2)
		| BF_GPMI_TIMING2_CE_DELAY(0x2)
		| BF_GPMI_TIMING2_READ_LATENCY(0x2);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_TIMING2);

	/* set timing 1 register */
	__raw_writel(BF_GPMI_TIMING1_DEVICE_BUSY_TIMEOUT(0x500),
			resources->gpmi_regs + HW_GPMI_TIMING1);

	/* Put GPMI in NAND mode, disable device reset, and make certain
	   IRQRDY polarity is active high. */
	value = BV_GPMI_CTRL1_GPMI_MODE__NAND
		| BM_GPMI_CTRL1_GANGED_RDYBUSY
		| (BV_GPMI_CTRL1_DEV_RESET__DISABLED << 3)
		| (BV_GPMI_CTRL1_ATA_IRQRDY_POLARITY__ACTIVEHIGH << 2);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_CTRL1_SET);
}

static int enable_ddr_toggle(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;
	struct mil *mil	= &this->mil;
	struct nand_chip *nand = &this->mil.nand;
	struct mtd_info	 *mtd = &mil->mtd;
	int saved_chip_number = mil->current_chip;
	uint32_t value;

	nand->select_chip(mtd, 0);

	/* [0] set proper timing */
	__raw_writel(BF_GPMI_TIMING0_ADDRESS_SETUP(0x5)
			| BF_GPMI_TIMING0_DATA_HOLD(0xa)
			| BF_GPMI_TIMING0_DATA_SETUP(0xa),
			resources->gpmi_regs + HW_GPMI_TIMING0);

	/* [2] set clk divider */
	__raw_writel(BM_GPMI_CTRL1_GPMI_CLK_DIV2_EN,
			resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* [3] about the clock, pay attention! */
	nand->select_chip(mtd, saved_chip_number);
	{
		struct clk *pll1;
		unsigned long rate;

		pll1 = clk_get(NULL, "pll1_main_clk");
		if (IS_ERR(pll1)) {
			printk(KERN_INFO "No PLL1 clock\n");
			return -EINVAL;
		}

		/* toggle nand : 133/66 MHz */
		rate = 33000000;
		clk_set_parent(resources->clock, pll1);
		clk_set_rate(resources->clock, rate);
	}
	nand->select_chip(mtd, 0);

	/* [4] setup timing */
	setup_ddr_timing_toggle(this);

	/* [5] set to TOGGLE mode */
	__raw_writel(BM_GPMI_CTRL1_SSYNCMODE,
			    resources->gpmi_regs + HW_GPMI_CTRL1_CLR);
	__raw_writel(BM_GPMI_CTRL1_TOGGLE_MODE | BM_GPMI_CTRL1_GANGED_RDYBUSY,
			    resources->gpmi_regs + HW_GPMI_CTRL1_SET);

	/* [6] enable both write & read DDR DLLs */
	value = BM_GPMI_READ_DDR_DLL_CTRL_REFCLK_ON |
		BM_GPMI_READ_DDR_DLL_CTRL_ENABLE |
		BF_GPMI_READ_DDR_DLL_CTRL_SLV_UPDATE_INT(0x2) |
		BF_GPMI_READ_DDR_DLL_CTRL_SLV_DLY_TARGET(0x7);

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	/* [7] reset read */
	__raw_writel(value | BM_GPMI_READ_DDR_DLL_CTRL_RESET,
			resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	value = value & ~BM_GPMI_READ_DDR_DLL_CTRL_RESET;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = BM_GPMI_WRITE_DDR_DLL_CTRL_REFCLK_ON |
		BM_GPMI_WRITE_DDR_DLL_CTRL_ENABLE    |
		BF_GPMI_WRITE_DDR_DLL_CTRL_SLV_UPDATE_INT(0x2) |
		BF_GPMI_WRITE_DDR_DLL_CTRL_SLV_DLY_TARGET(0x7) ,

	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [8] reset write */
	__raw_writel(value | BM_GPMI_WRITE_DDR_DLL_CTRL_RESET,
			resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [9] wait for locks for read and write  */
	do {
		uint32_t read_status, write_status;
		uint32_t r_mask, w_mask;

		read_status = __raw_readl(resources->gpmi_regs
					+ HW_GPMI_READ_DDR_DLL_STS);
		write_status = __raw_readl(resources->gpmi_regs
					+ HW_GPMI_WRITE_DDR_DLL_STS);

		r_mask = (BM_GPMI_READ_DDR_DLL_STS_REF_LOCK |
				BM_GPMI_READ_DDR_DLL_STS_SLV_LOCK);
		w_mask = (BM_GPMI_WRITE_DDR_DLL_STS_REF_LOCK |
				BM_GPMI_WRITE_DDR_DLL_STS_SLV_LOCK);

		if (((read_status & r_mask) == r_mask)
			&& ((write_status & w_mask) == w_mask))
				break;
	} while (1);

	/* [10] force update of read/write */
	value = __raw_readl(resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	__raw_writel(value | BM_GPMI_READ_DDR_DLL_CTRL_SLV_FORCE_UPD,
			resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = __raw_readl(resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value | BM_GPMI_WRITE_DDR_DLL_CTRL_SLV_FORCE_UPD,
			resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	/* [11] set gate update */
	value = __raw_readl(resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);
	value |= BM_GPMI_READ_DDR_DLL_CTRL_GATE_UPDATE;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_READ_DDR_DLL_CTRL);

	value = __raw_readl(resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);
	value |= BM_GPMI_WRITE_DDR_DLL_CTRL_GATE_UPDATE;
	__raw_writel(value, resources->gpmi_regs + HW_GPMI_WRITE_DDR_DLL_CTRL);

	onfi_ddr_mode = 1;
	nand->select_chip(mtd, saved_chip_number);

	printk(KERN_INFO "-- Sumsung TOGGLE NAND is enabled now. --\n");
	return 0;
}

/* To check if we need to initialize something else*/
static int extra_init(struct gpmi_nfc_data *this)
{
	ddr_clk = clk_get(NULL, "ddr_clk");
	if (IS_ERR(ddr_clk)) {
		printk(KERN_ERR "The ddr clock is gone!");
		ddr_clk = NULL;
		return -ENOENT;
	}
	apbh_dma_clk = clk_get(NULL, "apbh_dma_clk");
	if (IS_ERR(apbh_dma_clk)) {
		printk(KERN_ERR "The APBH_DMA clock is gone!");
		apbh_dma_clk = NULL;
		return -ENOENT;
	}
	ahb_max_clk = clk_get(NULL, "ahb_max_clk");
	if (IS_ERR(ahb_max_clk)) {
		printk(KERN_ERR "The APBH_DMA clock is gone!");
		ahb_max_clk = NULL;
		return -ENOENT;
	}

	if (is_ddr_nand(&this->device_info)) {
		switch (this->device_info.manufacturer_code) {
		case NAND_MFR_MICRON:
			return enable_ddr_onfi(this);
		case NAND_MFR_SAMSUNG:
			return enable_ddr_toggle(this);
		default:
			printk(KERN_ERR "You should not reach here.\n");
			BUG();
		}
	}
	return 0;
}

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
	uint32_t		value;

	/* We make the abstract choices in a common function. */

	if (gpmi_nfc_set_geometry(this))
		return !0;

	/* Translate the abstract choices into register fields. */

	block_count   = nfc->ecc_chunk_count - 1;
	block_size    = nfc->ecc_chunk_size_in_bytes >> 2;
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

	mxs_reset_block(resources->bch_regs, false);

	/* Configure layout 0. */

	value = BF_BCH_FLASH0LAYOUT0_NBLOCKS(block_count)     |
		BF_BCH_FLASH0LAYOUT0_META_SIZE(metadata_size) |
		BF_BCH_FLASH0LAYOUT0_ECC0(ecc_strength)       |
		BF_BCH_FLASH0LAYOUT0_DATA0_SIZE(block_size);
	if (is_ddr_nand(&this->device_info))
		value |= BM_BCH_FLASH0LAYOUT0_GF13_0_GF14_1;

	__raw_writel(value, resources->bch_regs + HW_BCH_FLASH0LAYOUT0);

	value = BF_BCH_FLASH0LAYOUT1_PAGE_SIZE(page_size)   |
		BF_BCH_FLASH0LAYOUT1_ECCN(ecc_strength)     |
		BF_BCH_FLASH0LAYOUT1_DATAN_SIZE(block_size);
	if (is_ddr_nand(&this->device_info))
		value |= BM_BCH_FLASH0LAYOUT1_GF13_0_GF14_1;

	__raw_writel(value, resources->bch_regs + HW_BCH_FLASH0LAYOUT1);

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
 * set_timing() - Configures the NFC timing.
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
	struct resources                 *resources = &this->resources;
	struct nfc_hal                   *nfc       =  this->nfc;
	unsigned char                    *gpmi_regs = resources->gpmi_regs;
	uint32_t                         register_image;

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
	struct resources                 *resources = &this->resources;
	struct nfc_hal                   *nfc       =  this->nfc;
	struct gpmi_nfc_hardware_timing  hw;

	/* Enable the clock. */

	if (ddr_clk)
		clk_enable(ddr_clk);
	if (apbh_dma_clk)
		clk_enable(apbh_dma_clk);
	if (ahb_max_clk)
		clk_enable(ahb_max_clk);
	clk_enable(resources->clock);

	/* Get the timing information we need. */

	nfc->clock_frequency_in_hz = clk_get_rate(resources->clock);
	gpmi_nfc_compute_hardware_timing(this, &hw);

	/* Apply the hardware timing. */

	/* Coming soon - the clock handling code isn't ready yet. */

}

/**
 * end() - End NFC I/O.
 *
 * @this:  Per-device data.
 */
static void end(struct gpmi_nfc_data *this)
{
	struct resources  *resources = &this->resources;

	clk_disable(resources->clock);
	if (ahb_max_clk)
		clk_disable(ahb_max_clk);
	if (apbh_dma_clk)
		clk_disable(apbh_dma_clk);
	if (ddr_clk)
		clk_disable(ddr_clk);
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

	mask = BF_GPMI_STAT_READY_BUSY(1 << 0);

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

	fill_dma_word1(&(*d)->cmd.cmd,
			DMA_READ, 0, 1, 1, 0, 1, 1, 1, 0, 3, length);
	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BM_GPMI_CTRL0_LOCK_CS			|
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BM_GPMI_CTRL0_ADDRESS_INCREMENT          |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = BV_GPMI_ECCCTRL_ENABLE_ECC__DISABLE;

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

	fill_dma_word1(&(*d)->cmd.cmd,
			DMA_READ, 0, 1, 0, 0, 1, 1, 0, 0, 4, length);
	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
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

	fill_dma_word1(&(*d)->cmd.cmd,
			DMA_WRITE, 0, 1, 0, 0, 1, 1, 1, 0, 1, length);
	(*d)->cmd.address = buffer;

	(*d)->cmd.pio_words[0] =
		BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BM_GPMI_CTRL0_WORD_LENGTH                |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(length)         ;

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
	uint32_t             value;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* A DMA descriptor that does an ECC page read. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WRITE;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__ENCODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	fill_dma_word1(&(*d)->cmd.cmd,
			NO_DMA_XFER, 0, 1, 0, 0, 1, 1, 0, 0, 6, 0);
	(*d)->cmd.address = 0;

	value = BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	if (onfi_ddr_mode == 0)
		value |= BM_GPMI_CTRL0_WORD_LENGTH;

	(*d)->cmd.pio_words[0] = value;

	(*d)->cmd.pio_words[1] = 0;

	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC               |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	if (onfi_ddr_mode)
		value = nfc_geo->page_size_in_bytes >> 1;
	else
		value = nfc_geo->page_size_in_bytes;

	(*d)->cmd.pio_words[3] = value;
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

	error = wait_for_completion_timeout(&nfc->bch_done,
							msecs_to_jiffies(1000));

	error = (!error) ? -ETIMEDOUT : 0;

	if (error)
		dev_err(dev, "[%s] bch timeout!!! \n", __func__);

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
	uint32_t             value;

	/* Compute the DMA channel. */

	dma_channel = resources->dma_low_channel + chip;

	/* Wait for the chip to report ready. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	fill_dma_word1(&(*d)->cmd.cmd,
			NO_DMA_XFER, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0);
	(*d)->cmd.address = 0;

	value = BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
		BF_GPMI_CTRL0_CS(chip)                   |
		BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(0)              ;
	if (onfi_ddr_mode == 0)
		value |= BM_GPMI_CTRL0_WORD_LENGTH;

	(*d)->cmd.pio_words[0] = value;


	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Enable the BCH block and read. */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__READ;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;
	ecc_command  = BV_GPMI_ECCCTRL_ECC_CMD__DECODE;
	buffer_mask  = BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_PAGE |
				BV_GPMI_ECCCTRL_BUFFER_MASK__BCH_AUXONLY;

	fill_dma_word1(&(*d)->cmd.cmd,
			NO_DMA_XFER, 1, 0, 0, 0, 0, 1, 0, 0, 6, 0);
	(*d)->cmd.address = 0;

	if (onfi_ddr_mode)
		value = BF_GPMI_CTRL0_COMMAND_MODE(command_mode) |
			BF_GPMI_CTRL0_CS(chip)                   |
			BF_GPMI_CTRL0_ADDRESS(address)           |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes >> 1) ;
	else
		value = BF_GPMI_CTRL0_COMMAND_MODE(command_mode)     |
			BM_GPMI_CTRL0_WORD_LENGTH                    |
			BF_GPMI_CTRL0_CS(chip)                       |
			BF_GPMI_CTRL0_ADDRESS(address)               |
		BF_GPMI_CTRL0_XFER_COUNT(nfc_geo->page_size_in_bytes);

	(*d)->cmd.pio_words[0] = value;

	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] =
		BM_GPMI_ECCCTRL_ENABLE_ECC 	         |
		BF_GPMI_ECCCTRL_ECC_CMD(ecc_command)     |
		BF_GPMI_ECCCTRL_BUFFER_MASK(buffer_mask) ;

	if (onfi_ddr_mode)
		value = nfc_geo->page_size_in_bytes >> 1;
	else
		value = nfc_geo->page_size_in_bytes;
	(*d)->cmd.pio_words[3] = value;

	(*d)->cmd.pio_words[4] = payload;
	(*d)->cmd.pio_words[5] = auxiliary;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Disable the BCH block */

	command_mode = BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY;
	address      = BV_GPMI_CTRL0_ADDRESS__NAND_DATA;

	fill_dma_word1(&(*d)->cmd.cmd,
			NO_DMA_XFER, 1, 0, 0, 1, 0, 1, 0, 0, 3, 0);
	(*d)->cmd.address = 0;

	value = BF_GPMI_CTRL0_COMMAND_MODE(command_mode)              |
		BF_GPMI_CTRL0_CS(chip)                                |
		BF_GPMI_CTRL0_ADDRESS(address)                        |
		BF_GPMI_CTRL0_XFER_COUNT(value) ;
	if (onfi_ddr_mode == 0)
		value |= BM_GPMI_CTRL0_WORD_LENGTH;

	(*d)->cmd.pio_words[0] = value;
	(*d)->cmd.pio_words[1] = 0;
	(*d)->cmd.pio_words[2] = 0;

	mxs_dma_desc_append(dma_channel, (*d));
	d++;

	/* Deassert the NAND lock and interrupt. */
	fill_dma_word1(&(*d)->cmd.cmd,
			NO_DMA_XFER, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0);
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

	error = wait_for_completion_timeout(&nfc->bch_done,
							msecs_to_jiffies(1000));

	error = (!error) ? -ETIMEDOUT : 0;

	if (error)
		dev_err(dev, "[%s] bch timeout!!! \n", __func__);

	/* Return success. */

	return error;

}

/* This structure represents the NFC HAL for this version of the hardware. */

struct nfc_hal  gpmi_nfc_hal_v2 = {
	.version                     = 2,
	.description                 = "8-chip GPMI and BCH",
	.max_chip_count              = 8,
	.max_data_setup_cycles       = (BM_GPMI_TIMING0_DATA_SETUP >>
						BP_GPMI_TIMING0_DATA_SETUP),
	.internal_data_setup_in_ns   = 0,
	.max_sample_delay_factor     = (BM_GPMI_CTRL1_RDN_DELAY >>
						BP_GPMI_CTRL1_RDN_DELAY),
	.max_dll_clock_period_in_ns  = 32,
	.max_dll_delay_in_ns         = 16,
	.init                        = init,
	.extra_init		     = extra_init,
	.set_geometry                = set_geometry,
	.set_timing                  = set_timing,
	.get_timing                  = get_timing,
	.exit                        = exit,
	.begin                       = begin,
	.end                         = end,
	.clear_bch                   = clear_bch,
	.is_ready                    = is_ready,
	.send_command                = send_command,
	.send_data                   = send_data,
	.read_data                   = read_data,
	.send_page                   = send_page,
	.read_page                   = read_page,
};
