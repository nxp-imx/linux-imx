/*
 * Freescale STMP37XX/STMP378X GPMI (General-Purpose-Media-Interface)
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
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>
#include <linux/dma-mapping.h>
#include <linux/ctype.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <asm/div64.h>

#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/regs-ecc8.h>
#include <mach/regs-gpmi.h>
#include <mach/dma.h>
#include "gpmi.h"

/*
 * Set this variable to a value greater than zero to see varying levels of
 * debugging output.
 */

static int debug;

/*
 * This variable counts the total number of times the driver has copied either
 * page data or OOB data from/to a DMA buffer.
 */

static int copies;

/*
 * Indicates that this driver should attempt to perform DMA directly to/from
 * buffers passed into this driver. If false, this driver will use its own
 * buffer for DMA and copy data between this buffer and the buffers that are
 * passed in.
 */

static int map_buffers = true;

static int ff_writes;

/*
 * Forces all OOB reads and writes to NOT use ECC.
 */

static int raw_mode;

/*
 * Indicates the driver should register an MTD that represents the entire
 * medium.
 */

static int add_mtd_entire;

/*
 * Indicates the driver should register a separate MTD for every physical chip.
 */

static int add_mtd_chip;

/*
 * Indicates the driver should report that *all* blocks are good.
 */

static int ignorebad;

/*
 * The maximum number of chips for which the NAND Flash MTD system is allowed to
 * scan.
 */

static int max_chips = 4;

/*
 *
 */

static long clk = -1;

/*
 * This variable is connected to the "bch" module parameter. If set, it
 * indicates the driver should use the BCH hardware block instead of the ECC8
 * hardware block for error correction.
 */

static int bch /* = 0 */ ;

/* Forward references. */

static int gpmi_nand_init_hw(struct platform_device *pdev, int request_pins);
static void gpmi_nand_release_hw(struct platform_device *pdev);
static int gpmi_dma_exchange(struct gpmi_nand_data *g,
			     struct stmp3xxx_dma_descriptor *dma);
static void gpmi_read_buf(struct mtd_info *mtd, uint8_t * buf, int len);

/*
 * This structure contains the "safe" GPMI timings that should succeed with any
 * NAND Flash device (although, with less-than-optimal performance).
 */

struct gpmi_nand_timing gpmi_safe_timing = {
	.address_setup = 25,
	.data_setup = 80,
	.data_hold = 60,
	.dsample_time = 6,
};

/*
 * ECC layout descriptions for various device geometries.
 */

static struct nand_ecclayout gpmi_oob_128 = {
	.oobfree = {
		    {
		     .offset = 2,
		     .length = 56,
		     }, {
			 .length = 0,
			 },
		    },
};

static struct nand_ecclayout gpmi_oob_64 = {
	.oobfree = {
		    {
		     .offset = 2,
		     .length = 16,
		     }, {
			 .length = 0,
			 },
		    },
};

/**
 * gpmi_cycles_ceil - Translates timings in nanoseconds to GPMI clock cycles.
 *
 * @ntime:   The time in nanoseconds.
 * @period:  The GPMI clock period.
 */
static inline u32 gpmi_cycles_ceil(u32 ntime, u32 period)
{
	int k;

	/*
	 * Compute the minimum number of clock periods that entirely contain the
	 * given time.
	 */

	k = (ntime + period - 1) / period;

	/*
	 * A cycle count of less than 1 can fatally confuse the hardware.
	 */

	if (k == 0)
		k++;

	return k;
}

/**
 * gpmi_timer_expiry - Inactivity timer expiration handler.
 */
static void gpmi_timer_expiry(unsigned long d)
{
#ifdef CONFIG_PM
	struct gpmi_nand_data *g = (struct gpmi_nand_data *)d;

	pr_debug("%s: timer expired\n", __func__);
	del_timer_sync(&g->timer);

	if (g->use_count ||
	    __raw_readl(REGS_GPMI_BASE + HW_GPMI_CTRL0) & BM_GPMI_CTRL0_RUN) {
		g->timer.expires = jiffies + 4 * HZ;
		add_timer(&g->timer);
	} else {
		stmp3xxx_setl(BM_GPMI_CTRL0_CLKGATE,
			      REGS_GPMI_BASE + HW_GPMI_CTRL0);
		clk_disable(g->clk);
		g->self_suspended = 1;
	}
#endif
}

/**
 * gpmi_self_wakeup - wakeup from self-pm light suspend
 */
static void gpmi_self_wakeup(struct gpmi_nand_data *g)
{
#ifdef CONFIG_PM
	int i = 1000;
	clk_enable(g->clk);
	stmp3xxx_clearl(BM_GPMI_CTRL0_CLKGATE, REGS_GPMI_BASE + HW_GPMI_CTRL0);
	while (i--
	       && __raw_readl(REGS_GPMI_BASE +
			      HW_GPMI_CTRL0) & BM_GPMI_CTRL0_CLKGATE) ;

	pr_debug("%s: i stopped at %d, data %p\n", __func__, i, g);
	g->self_suspended = 0;
	g->timer.expires = jiffies + 4 * HZ;
	add_timer(&g->timer);
#endif
}

/**
 * gpmi_set_timings - Set GPMI timings.
 *
 * @pdev: A pointer to the owning platform device.
 * @tm:   A pointer to the new timings.
 */
void gpmi_set_timings(struct platform_device *pdev, struct gpmi_nand_timing *tm)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	u32 period_ns = 1000000 / clk_get_rate(g->clk) + 1;
	u32 address_cycles, data_setup_cycles;
	u32 data_hold_cycles, data_sample_cycles;
	u32 busy_timeout;
	u32 t0;

	if (g->self_suspended)
		gpmi_self_wakeup(g);
	g->use_count++;

	g->timing = *tm;

	address_cycles = gpmi_cycles_ceil(tm->address_setup, period_ns);
	data_setup_cycles = gpmi_cycles_ceil(tm->data_setup, period_ns);
	data_hold_cycles = gpmi_cycles_ceil(tm->data_hold, period_ns);
	data_sample_cycles = gpmi_cycles_ceil(tm->dsample_time + period_ns / 4,
					      period_ns / 2);
	busy_timeout = gpmi_cycles_ceil(10000000 / 4096, period_ns);

	dev_dbg(&pdev->dev,
		"%s: ADDR %u, DSETUP %u, DH %u, DSAMPLE %u, BTO %u\n",
		__func__,
		address_cycles, data_setup_cycles, data_hold_cycles,
		data_sample_cycles, busy_timeout);

	t0 = BF(address_cycles, GPMI_TIMING0_ADDRESS_SETUP) |
	    BF(data_setup_cycles, GPMI_TIMING0_DATA_SETUP) |
	    BF(data_hold_cycles, GPMI_TIMING0_DATA_HOLD);
	__raw_writel(t0, REGS_GPMI_BASE + HW_GPMI_TIMING0);

	__raw_writel(BF(busy_timeout, GPMI_TIMING1_DEVICE_BUSY_TIMEOUT),
		     REGS_GPMI_BASE + HW_GPMI_TIMING1);

#ifdef CONFIG_ARCH_STMP378X
	stmp3xxx_clearl(BM_GPMI_CTRL1_RDN_DELAY,
			REGS_GPMI_BASE + HW_GPMI_CTRL1);
	stmp3xxx_setl(BF(data_sample_cycles, GPMI_CTRL1_RDN_DELAY),
		      REGS_GPMI_BASE + HW_GPMI_CTRL1);
#else
	stmp3xxx_clearl(BM_GPMI_CTRL1_DSAMPLE_TIME,
			REGS_GPMI_BASE + HW_GPMI_CTRL1);
	stmp3xxx_setl(BF(data_sample_cycles, GPMI_CTRL1_DSAMPLE_TIME),
		      REGS_GPMI_BASE + HW_GPMI_CTRL1);
#endif

	g->use_count--;
}

/**
 * bch_mode - Return a hardware register value that selects BCH.
 */
static inline u32 bch_mode(void)
{
	u32 c1 = 0;

#ifdef CONFIG_MTD_NAND_GPMI_BCH
	if (bch)
		c1 |= BM_GPMI_CTRL1_BCH_MODE;
#endif
	return c1;
}

/**
 * gpmi_nand_init_hw - Initialize the hardware.
 *
 * @pdev:          A pointer to the owning platform device.
 * @request_pins:  Indicates this function should request GPMI pins.
 *
 * Initialize GPMI hardware and set default (safe) timings for NAND access.
 * Returns error code or 0 on success
 */
static int gpmi_nand_init_hw(struct platform_device *pdev, int request_pins)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	struct gpmi_platform_data *gpd =
	    (struct gpmi_platform_data *)pdev->dev.platform_data;
	int err = 0;

	/* Try to get the GPMI clock. */

	g->clk = clk_get(NULL, "gpmi");
	if (IS_ERR(g->clk)) {
		err = PTR_ERR(g->clk);
		dev_err(&pdev->dev, "Can't get GPMI clock\n");
		goto out;
	}

	/* Turn on the GPMI clock. */

	clk_enable(g->clk);

	/*
	 * Check the clock rate setting. We don't allow this value to go below
	 * 24KHz because some chips don't work in that regime.
	 */

	if (clk <= 0)
		clk = 24000;

	/*
	 * Set the GPMI clock rate and record the value that was actually
	 * implemented.
	 */

	clk_set_rate(g->clk, clk);

	clk = clk_get_rate(g->clk);

	/* Check if we're supposed to ask for our pins. */

	if (request_pins)
		gpd->pinmux(1);

	/* Reset the GPMI block. */

	stmp3xxx_reset_block(HW_GPMI_CTRL0 + REGS_GPMI_BASE, 1);

	/* this CLEARS reset, despite of its name */
	stmp3xxx_setl(BM_GPMI_CTRL1_DEV_RESET, REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* IRQ polarity */
	stmp3xxx_setl(BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY,
		      REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/*
	 * Select the ECC to use. The bch_mode() function returns a value that
	 * selects whichever hardware is appropriate (q.v.).
	 */
	stmp3xxx_setl(bch_mode(), REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* Choose NAND mode (1 means ATA, 0 - NAND */
	stmp3xxx_clearl(BM_GPMI_CTRL1_GPMI_MODE,
			REGS_GPMI_BASE + HW_GPMI_CTRL1);

out:
	return err;
}

/**
 * gpmi_nand_release_hw - free the hardware
 *
 * @pdev: pointer to platform device
 *
 * In opposite to gpmi_nand_init_hw, release all acquired resources.
 */
static void gpmi_nand_release_hw(struct platform_device *pdev)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	struct gpmi_platform_data *gpd =
	    (struct gpmi_platform_data *)pdev->dev.platform_data;

	stmp3xxx_setl(BM_GPMI_CTRL0_SFTRST, REGS_GPMI_BASE + HW_GPMI_CTRL0);

	clk_disable(g->clk);
	clk_put(g->clk);
	gpd->pinmux(0);
}

/**
 * gpmi_dma_is_error -
 *
 * @g:  Per-device data structure.
 */
static int gpmi_dma_is_error(struct gpmi_nand_data *g)
{
	/* u32 n = __raw_readl(g->dma_ch); */

	/* CURrent DMA command */
	u32 c = __raw_readl(REGS_APBH_BASE +
			    HW_APBH_CHn_NXTCMDAR(g->cchip->dma_ch));

	if (c == g->cchip->error.handle) {
		pr_debug("%s: dma chain has reached error terminator\n",
			 __func__);
		return -EIO;
	}
	return 0;
}

/**
 * gpmi_dma_exchange - Run DMA to exchange with NAND chip
 *
 * @g:  Per-device data structure.
 * @d:  DMA descriptor.
 *
 * Run DMA and wait for completion
 */
static int gpmi_dma_exchange(struct gpmi_nand_data *g,
			     struct stmp3xxx_dma_descriptor *d)
{
	struct platform_device *pdev = g->dev;
	unsigned long timeout;
	int err;

	if (g->self_suspended)
		gpmi_self_wakeup(g);
	g->use_count++;

	if (!g->regulator) {
		g->regulator = regulator_get(&pdev->dev, "mmc_ssp-2");
		if (g->regulator && !IS_ERR(g->regulator))
			regulator_set_mode(g->regulator, REGULATOR_MODE_NORMAL);
		else
			g->regulator = NULL;
	}

	if (g->regulator)
		regulator_set_current_limit(g->regulator, g->reg_uA, g->reg_uA);

	init_completion(&g->done);
	stmp3xxx_dma_enable_interrupt(g->cchip->dma_ch);
	stmp3xxx_dma_go(g->cchip->dma_ch, d ? d : g->cchip->d, 1);

	timeout = wait_for_completion_timeout(&g->done, msecs_to_jiffies(1000));
	err = (timeout <= 0) ? -ETIMEDOUT : gpmi_dma_is_error(g);

	if (err)
		printk(KERN_ERR "%s: error %d, CS = %d, channel %d\n",
		       __func__, err, g->cchip->cs, g->cchip->dma_ch);

	stmp3xxx_dma_reset_channel(g->cchip->dma_ch);
	stmp3xxx_dma_clear_interrupt(g->cchip->dma_ch);

	if (g->regulator)
		regulator_set_current_limit(g->regulator, 0, 0);

	mod_timer(&g->timer, jiffies + 4 * HZ);
	g->use_count--;

	return err;
}

/**
 * gpmi_ecc_read_page - Replacement for nand_read_page
 *
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 */
static int gpmi_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t * buf)
{
	struct gpmi_nand_data *g = chip->priv;
	struct mtd_ecc_stats stats;
	dma_addr_t bufphys, oobphys;
	int err;

	bufphys = oobphys = ~0;

	if (map_buffers && virt_addr_valid(buf))
		bufphys = dma_map_single(&g->dev->dev, buf,
					 mtd->writesize, DMA_FROM_DEVICE);
	if (dma_mapping_error(&g->dev->dev, bufphys))
		bufphys = g->data_buffer_handle;

	if (map_buffers)
		oobphys = dma_map_single(&g->dev->dev, chip->oob_poi,
					 mtd->oobsize, DMA_FROM_DEVICE);
	if (dma_mapping_error(&g->dev->dev, oobphys))
		oobphys = g->oob_buffer_handle;

	/* ECC read */
	(void)g->hc->read(g->hc, g->selected_chip, g->cchip->d,
			  g->cchip->error.handle, bufphys, oobphys);

	err = gpmi_dma_exchange(g, NULL);

	g->hc->stat(g->hc, g->selected_chip, &stats);

	if (stats.failed || stats.corrected) {

		pr_debug("%s: ECC failed=%d, corrected=%d\n",
			 __func__, stats.failed, stats.corrected);

		g->mtd.ecc_stats.failed += stats.failed;
		g->mtd.ecc_stats.corrected += stats.corrected;
	}

	if (!dma_mapping_error(&g->dev->dev, oobphys)) {
		if (oobphys != g->oob_buffer_handle)
			dma_unmap_single(&g->dev->dev, oobphys,
					 mtd->oobsize, DMA_FROM_DEVICE);
		else {
			memcpy(chip->oob_poi, g->oob_buffer, mtd->oobsize);
			copies++;
		}
	}

	if (!dma_mapping_error(&g->dev->dev, bufphys)) {
		if (bufphys != g->data_buffer_handle)
			dma_unmap_single(&g->dev->dev, bufphys,
					 mtd->writesize, DMA_FROM_DEVICE);
		else {
			memcpy(buf, g->data_buffer, mtd->writesize);
			copies++;
		}
	}

	/* always fill the (possible ECC bytes with FF) */
	memset(chip->oob_poi + g->oob_free, 0xff, mtd->oobsize - g->oob_free);

	return err;
}

/**
 * is_ff - Checks if all the bits in a buffer are set.
 *
 * @buffer:  The buffer of interest.
 * @size:    The size of the buffer.
 */
static inline int is_ff(const u8 * buffer, size_t size)
{
	while (size--) {
		if (*buffer++ != 0xff)
			return 0;
	}
	return 1;
}

/**
 * gpmi_ecc_write_page - replacement for nand_write_page
 *
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void gpmi_ecc_write_page(struct mtd_info *mtd,
				struct nand_chip *chip, const uint8_t * buf)
{
	struct gpmi_nand_data *g = chip->priv;
	dma_addr_t bufphys, oobphys;
	int err;

	/* if we can't map it, copy it */
	bufphys = oobphys = ~0;

	if (map_buffers && virt_addr_valid(buf))
		bufphys = dma_map_single(&g->dev->dev,
					 (void *)buf, mtd->writesize,
					 DMA_TO_DEVICE);
	if (dma_mapping_error(&g->dev->dev, bufphys)) {
		bufphys = g->data_buffer_handle;
		memcpy(g->data_buffer, buf, mtd->writesize);
		copies++;
	}

	/* if OOB is all FF, leave it as such */
	if (!is_ff(chip->oob_poi, mtd->oobsize)) {
		if (map_buffers)
			oobphys = dma_map_single(&g->dev->dev, chip->oob_poi,
						 mtd->oobsize, DMA_TO_DEVICE);
		if (dma_mapping_error(&g->dev->dev, oobphys)) {
			oobphys = g->oob_buffer_handle;
			memcpy(g->oob_buffer, chip->oob_poi, mtd->oobsize);
			copies++;
		}
	} else
		ff_writes++;

	/* call ECC */
	g->hc->write(g->hc, g->selected_chip, g->cchip->d,
		     g->cchip->error.handle, bufphys, oobphys);

	err = gpmi_dma_exchange(g, NULL);
	if (err < 0)
		printk(KERN_ERR "%s: dma error\n", __func__);

	if (!dma_mapping_error(&g->dev->dev, oobphys)) {
		if (oobphys != g->oob_buffer_handle)
			dma_unmap_single(&g->dev->dev, oobphys, mtd->oobsize,
					 DMA_TO_DEVICE);
	}

	if (bufphys != g->data_buffer_handle)
		dma_unmap_single(&g->dev->dev, bufphys, mtd->writesize,
				 DMA_TO_DEVICE);
}

/**
 * gpmi_write_buf - replacement for nand_write_buf
 *
 * @mtd: MTD device
 * @buf: data buffer
 * @len: length of the data buffer
 */
static void gpmi_write_buf(struct mtd_info *mtd, const uint8_t * buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct stmp3xxx_dma_descriptor *chain = g->cchip->d;
	dma_addr_t phys;
	int err;

	BUG_ON(len > mtd->writesize);

	phys = ~0;

	if (map_buffers && virt_addr_valid(buf))
		phys = dma_map_single(&g->dev->dev,
				      (void *)buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&g->dev->dev, phys)) {
		phys = g->write_buffer_handle;
		memcpy(g->write_buffer, buf, len);
		copies++;
	}

	/* Write plain data */
	chain->command->cmd =
	    BF(len, APBH_CHn_CMD_XFER_COUNT) |
	    BF(4, APBH_CHn_CMD_CMDWORDS) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_NANDLOCK |
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBH_CHn_CMD_COMMAND__DMA_READ, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
	    BF(BV_GPMI_CTRL0_COMMAND_MODE__WRITE, GPMI_CTRL0_COMMAND_MODE) |
	    BM_GPMI_CTRL0_WORD_LENGTH |
	    BM_GPMI_CTRL0_LOCK_CS |
	    BF(g->selected_chip, GPMI_CTRL0_CS) |
	    BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA, GPMI_CTRL0_ADDRESS) |
	    BF(len, GPMI_CTRL0_XFER_COUNT);

	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = phys;

	err = gpmi_dma_exchange(g, NULL);
	if (err)
		printk(KERN_ERR "%s: dma error\n", __func__);

	if (phys != g->write_buffer_handle)
		dma_unmap_single(&g->dev->dev, phys, len, DMA_TO_DEVICE);

	if (debug >= 2)
		print_hex_dump_bytes("WBUF ", DUMP_PREFIX_OFFSET, buf, len);
}

/**
 * gpmi_read_buf - replacement for nand_read_buf
 *
 * @mtd: MTD device
 * @buf: pointer to the buffer
 * @len: size of the buffer
 */
static void gpmi_read_buf(struct mtd_info *mtd, uint8_t * buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct stmp3xxx_dma_descriptor *chain;
	dma_addr_t phys;
	int err;

	phys = ~0;

	if (map_buffers && virt_addr_valid(buf))
		phys = dma_map_single(&g->dev->dev, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(&g->dev->dev, phys))
		phys = g->read_buffer_handle;

	chain = g->cchip->d;

	/* read data */
	chain->command->cmd =
	    BF(len, APBH_CHn_CMD_XFER_COUNT) |
	    BF(1, APBH_CHn_CMD_CMDWORDS) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_CHAIN |
	    BF(BV_APBH_CHn_CMD_COMMAND__DMA_WRITE, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
	    BF(BV_GPMI_CTRL0_COMMAND_MODE__READ, GPMI_CTRL0_COMMAND_MODE) |
	    BM_GPMI_CTRL0_WORD_LENGTH |
	    BM_GPMI_CTRL0_LOCK_CS |
	    BF(g->selected_chip, GPMI_CTRL0_CS) |
	    BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA, GPMI_CTRL0_ADDRESS) |
	    BF(len, GPMI_CTRL0_XFER_COUNT);
	chain->command->buf_ptr = phys;
	chain++;

	chain->command->cmd =
	    BF(4, APBH_CHn_CMD_CMDWORDS) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_NANDWAIT4READY |
	    BM_APBH_CHn_CMD_NANDLOCK |
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
	    BF(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY,
	       GPMI_CTRL0_COMMAND_MODE) |
	    BM_GPMI_CTRL0_WORD_LENGTH |
	    BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA, GPMI_CTRL0_ADDRESS) |
	    BM_GPMI_CTRL0_LOCK_CS | BF(g->selected_chip, GPMI_CTRL0_CS);
	chain->command->pio_words[1] =
	    chain->command->pio_words[2] = chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = 0;

	err = gpmi_dma_exchange(g, NULL);
	if (err)
		printk(KERN_ERR "%s: dma error\n", __func__);

	if (phys != g->read_buffer_handle)
		dma_unmap_single(&g->dev->dev, phys, len, DMA_FROM_DEVICE);
	else {
		memcpy(buf, g->read_buffer, len);
		copies++;
	}

	if (debug >= 2)
		print_hex_dump_bytes("RBUF ", DUMP_PREFIX_OFFSET, buf, len);
}

/**
 * gpmi_read_byte - replacement for nand_read_byte
 * @mtd: MTD device
 *
 * Uses gpmi_read_buf to read 1 byte from device
 */
static u8 gpmi_read_byte(struct mtd_info *mtd)
{
	u8 b;

	gpmi_read_buf(mtd, (uint8_t *) & b, 1);
	return b;
}

/**
 * gpmi_read_word - replacement for nand_read_word
 * @mtd:  The owning MTD.
 *
 * Uses gpmi_read_buf to read 2 bytes from device
 */
static u16 gpmi_read_word(struct mtd_info *mtd)
{
	u16 w;

	gpmi_read_buf(mtd, (uint8_t *) & w, sizeof(u16));
	return w;
}

/**
 * gpmi_erase - Hook for erase operations at the MTD level.
 *
 * We install this function in the "erase" function pointer of the owning
 * struct mtd_info. Thus, this function will get called *instead* of the
 * function that the NAND Flash MTD system installed (see nand_erase()).
 *
 * We do this because, if an erase operation fails, then the block should be
 * marked bad. Unfortunately, the NAND Flash MTD code doesn't do this. Since
 * we've "hooked" the call, we can "override" the base NAND Flash MTD behavior
 * and make sure the proper marking gets done before we return to the
 * original caller.
 *
 * @mtd: MTD device
 * @instr: erase instruction
 */
int gpmi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int rc;
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct gpmi_nand_data *data = platform_get_drvdata(g->dev);

	if (g->self_suspended)
		gpmi_self_wakeup(data);
	g->use_count++;

	rc = nand_erase_nand(mtd, instr, 0);

	if (rc == -EIO)		/* block cannot be erased */
		gpmi_block_mark_as(chip,
				   (instr->addr >> chip->bbt_erase_shift),
				   0x01);

	mod_timer(&g->timer, jiffies + 4 * HZ);
	g->use_count--;
	return rc;
}

/**
 * gpmi_dev_ready - Wait until the medium is ready.
 *
 * This function is supposed to return the instantaneous state of the medium.
 * Instead, it actually waits for the medium to be ready. This is mostly
 * harmless, but isn't actually correct.
 *
 * @mtd:  The owning MTD.
 */
static int gpmi_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct stmp3xxx_dma_descriptor *chain = g->cchip->d;
	int ret;

	/* wait for ready */
	chain->command->cmd =
	    BF(4, APBH_CHn_CMD_CMDWORDS) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_NANDWAIT4READY |
	    BM_APBH_CHn_CMD_NANDLOCK |
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
	    BF(BV_GPMI_CTRL0_COMMAND_MODE__WAIT_FOR_READY,
	       GPMI_CTRL0_COMMAND_MODE) |
	    BM_GPMI_CTRL0_WORD_LENGTH |
	    BF(BV_GPMI_CTRL0_ADDRESS__NAND_DATA,
	       GPMI_CTRL0_ADDRESS) | BF(g->selected_chip, GPMI_CTRL0_CS);
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->pio_words[3] = 0;
	chain->command->buf_ptr = 0;
	chain++;

	ret = gpmi_dma_exchange(g, NULL);
	if (ret != 0)
		printk(KERN_ERR "gpmi: gpmi_dma_exchange() timeout!\n");
	return ret == 0;
}

/**
 * gpmi_hwcontrol - Send command/address byte to the NAND Flash.
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
 * @cmd:   The command byte.
 * @ctrl:  Control flags.
 */
static void gpmi_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct stmp3xxx_dma_descriptor *chain = g->cchip->d;
	int ret;

	/*
	 * Every operation begins with a series of command and address bytes,
	 * which are distinguished by either the Address Latch Enable (ALE) or
	 * Command Latch Enable (CLE) being asserted. Finally, when the caller
	 * is actually ready to execute the command, he will deassert both latch
	 * enables.
	 *
	 * Rather than run a separate DMA operation for every single byte, we
	 * queue them up and run a single DMA operation for the entire series
	 * of command and data bytes.
	 */

	if ((ctrl & (NAND_ALE | NAND_CLE))) {
		if (cmd != NAND_CMD_NONE)
			g->cmd_buffer[g->cmd_buffer_sz++] = cmd;
		return;
	}

	/*
	 * If control arrives here, the caller has deasserted both the ALE and
	 * CLE, which means he's ready to run an operation. Check if we actually
	 * have any bytes to send.
	 */

	if (g->cmd_buffer_sz == 0)
		return;

	/* output command */
	chain->command->cmd =
	    BF(g->cmd_buffer_sz, APBH_CHn_CMD_XFER_COUNT) |
	    BF(3, APBH_CHn_CMD_CMDWORDS) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD |
	    BM_APBH_CHn_CMD_NANDLOCK |
	    BM_APBH_CHn_CMD_CHAIN |
	    BF(BV_APBH_CHn_CMD_COMMAND__DMA_READ, APBH_CHn_CMD_COMMAND);
	chain->command->pio_words[0] =
	    BF(BV_GPMI_CTRL0_COMMAND_MODE__WRITE, GPMI_CTRL0_COMMAND_MODE) |
	    BM_GPMI_CTRL0_WORD_LENGTH |
	    BM_GPMI_CTRL0_LOCK_CS |
	    BF(g->selected_chip, GPMI_CTRL0_CS) |
	    BF(BV_GPMI_CTRL0_ADDRESS__NAND_CLE, GPMI_CTRL0_ADDRESS) |
	    BF(g->cmd_buffer_sz, GPMI_CTRL0_XFER_COUNT);
	if (g->cmd_buffer_sz > 0)
		chain->command->pio_words[0] |= BM_GPMI_CTRL0_ADDRESS_INCREMENT;
	chain->command->pio_words[1] = 0;
	chain->command->pio_words[2] = 0;
	chain->command->buf_ptr = g->cmd_buffer_handle;
	chain++;

	/* emit IRQ */
	chain->command->cmd =
	    BF(0, APBH_CHn_CMD_CMDWORDS) |
	    BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND) |
	    BM_APBH_CHn_CMD_WAIT4ENDCMD;
	chain++;

	/* last in chain get the irq bit set */
	chain[-1].command->cmd |= BM_APBH_CHn_CMD_IRQONCMPLT;

	if (debug >= 3)
		print_hex_dump(KERN_INFO, "CMD ", DUMP_PREFIX_OFFSET, 16, 1,
			       g->cmd_buffer, g->cmd_buffer_sz, 1);

	ret = gpmi_dma_exchange(g, NULL);
	if (ret != 0) {
		printk(KERN_ERR "%s: chip %d, dma error %d on the command:\n",
		       __func__, g->selected_chip, ret);
		print_hex_dump(KERN_INFO, "CMD ", DUMP_PREFIX_OFFSET, 16, 1,
			       g->cmd_buffer, g->cmd_buffer_sz, 1);
	}

	gpmi_dev_ready(mtd);

	g->cmd_buffer_sz = 0;
}

/**
 * gpmi_alloc_buffers - allocate DMA buffers for one chip
 *
 * @pdev:	GPMI platform device
 * @g:		pointer to structure associated with NAND chip
 *
 * Allocate buffer using dma_alloc_coherent
 */
static int gpmi_alloc_buffers(struct platform_device *pdev,
			      struct gpmi_nand_data *g)
{
	g->cmd_buffer = dma_alloc_coherent(&pdev->dev,
					   g->cmd_buffer_size,
					   &g->cmd_buffer_handle, GFP_DMA);
	if (!g->cmd_buffer)
		goto out1;

	g->write_buffer = dma_alloc_coherent(&pdev->dev,
					     g->write_buffer_size * 2,
					     &g->write_buffer_handle, GFP_DMA);
	if (!g->write_buffer)
		goto out2;

	g->read_buffer = g->write_buffer + g->write_buffer_size;
	g->read_buffer_handle = g->write_buffer_handle + g->write_buffer_size;

	g->data_buffer = dma_alloc_coherent(&pdev->dev,
					    g->data_buffer_size,
					    &g->data_buffer_handle, GFP_DMA);
	if (!g->data_buffer)
		goto out3;

	g->oob_buffer = dma_alloc_coherent(&pdev->dev,
					   g->oob_buffer_size,
					   &g->oob_buffer_handle, GFP_DMA);
	if (!g->oob_buffer)
		goto out4;

	g->verify_buffer = kzalloc(2 * (g->data_buffer_size +
					g->oob_buffer_size), GFP_KERNEL);
	if (!g->verify_buffer)
		goto out5;

	return 0;

out5:
	dma_free_coherent(&pdev->dev, g->oob_buffer_size,
			  g->oob_buffer, g->oob_buffer_handle);
out4:
	dma_free_coherent(&pdev->dev, g->data_buffer_size,
			  g->data_buffer, g->data_buffer_handle);
out3:
	dma_free_coherent(&pdev->dev, g->write_buffer_size * 2,
			  g->write_buffer, g->write_buffer_handle);
out2:
	dma_free_coherent(&pdev->dev, g->cmd_buffer_size,
			  g->cmd_buffer, g->cmd_buffer_handle);
out1:
	return -ENOMEM;
}

/**
 * gpmi_free_buffers - free buffers allocated by gpmi_alloc_buffers
 *
 * @pdev:	platform device
 * @g:		pointer to structure associated with NAND chip
 *
 * Deallocate buffers on exit
 */
static void gpmi_free_buffers(struct platform_device *pdev,
			      struct gpmi_nand_data *g)
{
	kfree(g->verify_buffer);
	dma_free_coherent(&pdev->dev, g->oob_buffer_size,
			  g->oob_buffer, g->oob_buffer_handle);
	dma_free_coherent(&pdev->dev, g->write_buffer_size * 2,
			  g->write_buffer, g->write_buffer_handle);
	dma_free_coherent(&pdev->dev, g->cmd_buffer_size,
			  g->cmd_buffer, g->cmd_buffer_handle);
	dma_free_coherent(&pdev->dev, g->data_buffer_size,
			  g->data_buffer, g->data_buffer_handle);
}

/* only used in SW-ECC or NO-ECC cases */
static int gpmi_verify_buf(struct mtd_info *mtd, const uint8_t * buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;

	chip->read_buf(mtd, g->verify_buffer, len);

	if (memcmp(buf, g->verify_buffer, len))
		return -EFAULT;

	return 0;
}

/**
 * gpmi_ecc_read_oob - replacement for nand_read_oob
 *
 * @mtd:	MTD device
 * @chip:	mtd->priv
 * @page:	page address
 * @sndcmd:	flag indicates that command should be sent
 */
int gpmi_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
		      int page, int sndcmd)
{
	struct gpmi_nand_data *g = chip->priv;
	loff_t oob_offset;
	struct mtd_ecc_stats stats;
	dma_addr_t oobphys;
	int ecc;
	int ret;

	ecc = g->raw_oob_mode == 0 && raw_mode == 0;

	if (sndcmd) {
		oob_offset = mtd->writesize;
		if (likely(ecc))
			oob_offset += chip->ecc.bytes * chip->ecc.steps;
		chip->cmdfunc(mtd, NAND_CMD_READ0, oob_offset, page);
		sndcmd = 0;
	}

	if (unlikely(!ecc)) {
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
		return 1;
	}

	oobphys = ~0;

	if (map_buffers)
		oobphys = dma_map_single(&g->dev->dev, chip->oob_poi,
					 mtd->oobsize, DMA_FROM_DEVICE);
	if (dma_mapping_error(&g->dev->dev, oobphys))
		oobphys = g->oob_buffer_handle;

	/* ECC read */
	(void)g->hc->read(g->hc, g->selected_chip, g->cchip->d,
			  g->cchip->error.handle, ~0, oobphys);

	ret = gpmi_dma_exchange(g, NULL);

	g->hc->stat(g->hc, g->selected_chip, &stats);

	if (stats.failed || stats.corrected) {

		printk(KERN_DEBUG "%s: ECC failed=%d, corrected=%d\n",
		       __func__, stats.failed, stats.corrected);

		g->mtd.ecc_stats.failed += stats.failed;
		g->mtd.ecc_stats.corrected += stats.corrected;
	}

	if (oobphys != g->oob_buffer_handle)
		dma_unmap_single(&g->dev->dev, oobphys, mtd->oobsize,
				 DMA_FROM_DEVICE);
	else {
		memcpy(chip->oob_poi, g->oob_buffer, mtd->oobsize);
		copies++;
	}

	/* fill rest with ff */
	memset(chip->oob_poi + g->oob_free, 0xff, mtd->oobsize - g->oob_free);

	return ret ? ret : 1;
}

/**
 * gpmi_ecc_write_oob - replacement for nand_write_oob
 *
 * @mtd:	MTD device
 * @chip:	mtd->priv
 * @page:	page address
 */
static int gpmi_ecc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	int status = 0;
	struct gpmi_nand_data *g = chip->priv;
	loff_t oob_offset;
	dma_addr_t oobphys;
	int ecc;
	int err = 0;

	/* if OOB is all FF, leave it as such */
	if (is_ff(chip->oob_poi, mtd->oobsize)) {
		ff_writes++;

		pr_debug("%s: Skipping an empty page 0x%x (0x%x)\n",
			 __func__, page, page << chip->page_shift);
		return 0;
	}

	ecc = g->raw_oob_mode == 0 && raw_mode == 0;

	/* Send command to start input data     */
	oob_offset = mtd->writesize;
	if (likely(ecc)) {
		oob_offset += chip->ecc.bytes * chip->ecc.steps;
		memset(chip->oob_poi + g->oob_free, 0xff,
		       mtd->oobsize - g->oob_free);
	}
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, oob_offset, page);

	/* call ECC */
	if (likely(ecc)) {

		oobphys = ~0;

		if (map_buffers)
			oobphys = dma_map_single(&g->dev->dev, chip->oob_poi,
						 mtd->oobsize, DMA_TO_DEVICE);
		if (dma_mapping_error(&g->dev->dev, oobphys)) {
			oobphys = g->oob_buffer_handle;
			memcpy(g->oob_buffer, chip->oob_poi, mtd->oobsize);
			copies++;
		}

		g->hc->write(g->hc, g->selected_chip, g->cchip->d,
			     g->cchip->error.handle, ~0, oobphys);

		err = gpmi_dma_exchange(g, NULL);
	} else
		chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	/* ..and wait for result                */
	status = chip->waitfunc(mtd, chip);

	if (likely(ecc)) {
		if (oobphys != g->oob_buffer_handle)
			dma_unmap_single(&g->dev->dev, oobphys, mtd->oobsize,
					 DMA_TO_DEVICE);
	}

	if (status & NAND_STATUS_FAIL) {
		pr_debug("%s: NAND_STATUS_FAIL\n", __func__);
		return -EIO;
	}

	return err;
}

/**
 * gpmi_irq - IRQ handler
 *
 * @irq:	Interrupt number.
 * @context:	IRQ context, pointer to gpmi_nand_data
 */
static irqreturn_t gpmi_irq(int irq, void *context)
{
	struct gpmi_nand_data *g = context;

	if (stmp3xxx_dma_is_interrupt(g->cchip->dma_ch)) {
		stmp3xxx_dma_clear_interrupt(g->cchip->dma_ch);
		complete(&g->done);
	}
	stmp3xxx_clearl(BM_GPMI_CTRL1_DEV_IRQ | BM_GPMI_CTRL1_TIMEOUT_IRQ,
			REGS_GPMI_BASE + HW_GPMI_CTRL1);
	return IRQ_HANDLED;
}

/**
 * gpmi_select_chip() - NAND Flash MTD Interface select_chip()
 *
 * @mtd:     A pointer to the owning MTD.
 * @chipnr:  The chip number to select, or -1 to select no chip.
 */
static void gpmi_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;

	if (chipnr == g->selected_chip)
		return;

	g->selected_chip = chipnr;
	g->cchip = NULL;

	if (chipnr == -1)
		return;

	g->cchip = g->chips + chipnr;
}

/**
 * gpmi_command() - NAND Flash MTD Interface cmdfunc()
 *
 * This function is a veneer that calls the function originally installed by the
 * NAND Flash MTD code.
 *
 * @mtd:       A pointer to the owning MTD.
 * @command:   The command code.
 * @column:    The column address associated with this command code, or -1 if
 *             no column address applies.
 * @page_addr: The page address associated with this command code, or -1 if no
 *             page address applies.
 */
static void gpmi_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;

	g->saved_command(mtd, command, column, page_addr);
}

/**
 * gpmi_read_oob() - MTD Interface read_oob().
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code.
 *
 * @mtd:   A pointer to the MTD.
 * @from:  The starting address to read.
 * @ops:   Describes the operation.
 */
static int gpmi_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
	register struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	int ret;

	g->raw_oob_mode = ops->mode == MTD_OOB_RAW;
	ret = g->saved_read_oob(mtd, from, ops);
	g->raw_oob_mode = 0;
	return ret;
}

/**
 * gpmi_read_oob() - MTD Interface write_oob().
 *
 * This function is a veneer that replaces the function originally installed by
 * the NAND Flash MTD code.
 *
 * @mtd:   A pointer to the MTD.
 * @to:    The starting address to write.
 * @ops:   Describes the operation.
 */
static int gpmi_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	register struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	int ret;

	g->raw_oob_mode = ops->mode == MTD_OOB_RAW;
	ret = g->saved_write_oob(mtd, to, ops);
	g->raw_oob_mode = 0;
	return ret;
}

/**
 * gpmi_scan_middle - Intermediate initialization.
 *
 * @g:  Per-device data structure.
 *
 * Rather than call nand_scan(), this function makes the same calls, but
 * inserts this function into the initialization pathway.
 */
static int gpmi_scan_middle(struct gpmi_nand_data *g)
{
	int oobsize = 0;

	/* Limit to 2G size due to Kernel larger 4G space support */
	if (g->mtd.size == 0) {
		g->mtd.size = 1 << 31;
		g->chip.chipsize = do_div(g->mtd.size, g->chip.numchips);
	}

	/*
	 * In all currently-supported geometries, the number of ECC bytes that
	 * apply to the OOB bytes is the same.
	 */

	g->ecc_oob_bytes = 9;

	/* Look at the page size and configure appropriately. */

	switch (g->mtd.writesize) {
	case 2048:		/* 2K page */
		g->chip.ecc.layout = &gpmi_oob_64;
		g->chip.ecc.bytes = 9;
		g->oob_free = 19;
		g->hwecc_type_read = GPMI_ECC4_RD;
		g->hwecc_type_write = GPMI_ECC4_WR;
		oobsize = 64;
		break;
	case 4096:
		g->chip.ecc.layout = &gpmi_oob_128;
		g->chip.ecc.bytes = 18;
		g->oob_free = 65;
		g->hwecc_type_read = GPMI_ECC8_RD;
		g->hwecc_type_write = GPMI_ECC8_WR;
		oobsize = 218;
		break;
	default:
		printk(KERN_ERR "Unsupported write_size %d.", g->mtd.writesize);
		break;
	}

	g->mtd.ecclayout = g->chip.ecc.layout;
	/* sanity check */
	if (oobsize > NAND_MAX_OOBSIZE || g->mtd.writesize > NAND_MAX_PAGESIZE) {
		printk(KERN_ERR "Internal error. Either page size "
		       "(%d) > max (%d) "
		       "or oob size (%d) > max(%d). Sorry.\n",
		       oobsize, NAND_MAX_OOBSIZE,
		       g->mtd.writesize, NAND_MAX_PAGESIZE);
		return -ERANGE;
	}

	/*
	 * Hook the command function provided by the reference implementation.
	 * This has to be done here, rather than at initialization time, because
	 * the NAND Flash MTD installed the reference implementation only just
	 * now.
	 */

	g->saved_command = g->chip.cmdfunc;
	g->chip.cmdfunc = gpmi_command;

	/* Install the ECC. */

	if (oobsize > 0) {
		g->mtd.oobsize = oobsize;
		/* otherwise error; oobsize should be set
		   in valid cases */
		g->hc = gpmi_ecc_find("ecc8");
		g->hc->setup(g->hc, 0, g->mtd.writesize, g->mtd.oobsize);
		return 0;
	}

	/* If control arrives here, something has gone wrong. */

	return -ENXIO;

}

/**
 * gpmi_write_page - [REPLACEABLE] write one page
 * @mtd:	MTD device structure
 * @chip:	NAND chip descriptor
 * @buf:	the data to write
 * @page:	page number to write
 * @cached:	cached programming
 * @raw:	use _raw version of write_page
 */
static int gpmi_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t * buf, int page, int cached, int raw)
{
	struct gpmi_nand_data *g = chip->priv;
	int status, empty_data, empty_oob;
	int oobsz;
#if defined(CONFIG_MTD_NAND_VERIFY_WRITE)
	void *vbuf, *obuf;
#if 0
	void *voob, *ooob;
#endif
#endif

	oobsz = likely(g->raw_oob_mode == 0 && raw_mode == 0) ?
	    g->oob_free : mtd->oobsize;

	empty_data = is_ff(buf, mtd->writesize);
	empty_oob = is_ff(buf, oobsz);

	if (empty_data && empty_oob) {
		ff_writes++;

		pr_debug("%s: Skipping an empty page 0x%x (0x%x)\n",
			 __func__, page, page << chip->page_shift);
		return 0;
	}

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	if (likely(raw == 0))
		chip->ecc.write_page(mtd, chip, buf);
	else
		chip->ecc.write_page_raw(mtd, chip, buf);

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

		status = chip->waitfunc(mtd, chip);

		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);

		if (status & NAND_STATUS_FAIL) {
			pr_debug("%s: NAND_STATUS_FAIL\n", __func__);
			return -EIO;
		}
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

#if defined(CONFIG_MTD_NAND_VERIFY_WRITE)
	if (empty_data)
		return 0;

	obuf = g->verify_buffer;
#if 1				/* make vbuf aligned by mtd->writesize */
	vbuf = obuf + mtd->writesize;
#else
	ooob = obuf + mtd->writesize;
	vbuf = ooob + mtd->oobsize;
	voob = vbuf + mtd->writesize;
#endif

	/* keep data around */
	memcpy(obuf, buf, mtd->writesize);
#if 0
	memcpy(ooob, chip->oob_poi, oobsz);
#endif
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	if (likely(raw == 0))
		chip->ecc.read_page(mtd, chip, vbuf);
	else
		chip->ecc.read_page_raw(mtd, chip, vbuf);

#if 0
	memcpy(voob, chip->oob_poi, oobsz);
#endif

	if (!empty_data && memcmp(obuf, vbuf, mtd->writesize) != 0)
		return -EIO;
#endif

	return 0;
}

/**
 * gpmi_read_page_raw - [Intern] read raw page data without ecc
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 */
static int gpmi_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t * buf)
{
	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	return 0;
}

/**
 * gpmi_write_page_raw - [Intern] raw page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void gpmi_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t * buf)
{
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

/**
 * gpmi_init_chip - Sets up the driver to control the given chip.
 *
 * @pdev:    A pointer to the owning struct platform_device.
 * @g:       Per-device data.
 * @n:       The chip number.
 * @dma_ch:  The DMA channel to use with this chip.
 */
static int gpmi_init_chip(struct platform_device *pdev,
			  struct gpmi_nand_data *g, int n, unsigned dma_ch)
{
	int err;

	g->chips[n].dma_ch = dma_ch;
	g->chips[n].cs = n;

	err = stmp3xxx_dma_request(dma_ch, NULL, dev_name(&pdev->dev));
	if (err) {
		dev_err(&pdev->dev, "can't request DMA channel 0x%x\n", dma_ch);
		goto out_all;
	}

	err = stmp3xxx_dma_make_chain(dma_ch,
				      &g->chips[n].chain,
				      g->chips[n].d, ARRAY_SIZE(g->chips[n].d));
	if (err) {
		dev_err(&pdev->dev, "can't setup DMA chain\n");
		goto out_all;
	}

	err = stmp3xxx_dma_allocate_command(dma_ch, &g->chips[n].error);
	if (err) {
		dev_err(&pdev->dev, "can't setup DMA chain\n");
		goto out_all;
	}

	g->chips[n].error.command->cmd =
	    BM_APBH_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBH_CHn_CMD_COMMAND__NO_DMA_XFER, APBH_CHn_CMD_COMMAND);
out_all:
	return err;
}

/**
 * gpmi_deinit_chip - Tears down this driver's control of the given chip.
 *
 * @pdev:    A pointer to the owning struct platform_device.
 * @g:       Per-device data.
 * @n:       The chip number.
 */
static void gpmi_deinit_chip(struct platform_device *pdev,
			     struct gpmi_nand_data *g, int n)
{
	int dma_ch;

	if (n < 0) {
		for (n = 0; n < ARRAY_SIZE(g->chips); n++)
			gpmi_deinit_chip(pdev, g, n);
		return;
	}

	if (g->chips[n].dma_ch <= 0)
		return;

	dma_ch = g->chips[n].dma_ch;

	stmp3xxx_dma_free_command(dma_ch, &g->chips[n].error);
	stmp3xxx_dma_free_chain(&g->chips[n].chain);
	stmp3xxx_dma_release(dma_ch);
}

/**
 * gpmi_create_partitions - Create platform-driven partitions.
 *
 * This function creates partitions based on the platform data.
 *
 * @g:         Per-device data.
 * @gpd:       Per-device platform data.
 * @chipsize:  The size of a single, physical chip in bytes.
 */
static void gpmi_create_partitions(struct gpmi_nand_data *g,
				   struct gpmi_platform_data *gpd,
				   uint64_t chipsize)
{

#ifdef CONFIG_MTD_PARTITIONS
	int chip, p;
	char chipname[20];

	/*
	 * We have a single MTD now that represents the entire medium. We want
	 * an MTD for each physical chip in the medium.
	 *
	 * If there's only one chip, then we can simply use the medium MTD.
	 *
	 * If there are multiple chips, we need to create partition MTDs that
	 * represent the subsets of the medium occupied by each physical chip.
	 */

	if (g->numchips == 1)
		g->chip_mtds[0] = &g->mtd;
	else {

		/*
		 * Construct an array of partition descriptions, one for each
		 * physical chip.
		 */

		for (chip = 0; chip < g->numchips; chip++) {
			memset(g->chip_partitions + chip,
			       0, sizeof(g->chip_partitions[chip]));
			snprintf(chipname, sizeof(chipname),
				 "gpmi-chip-%d", chip);
			g->chip_partitions[chip].name =
			    kstrdup(chipname, GFP_KERNEL);
			g->chip_partitions[chip].size = chipsize;
			g->chip_partitions[chip].offset = chipsize * chip;
			g->chip_partitions[chip].mask_flags = 0;
		}

		/*
		 * Derive a partition MTD for each physical chip.
		 *
		 * This function will register each partition MTD it creates
		 * that doesn't have the corresponding "mtdp" field set. Since
		 * we've explicitly set the "mtdp" field for each, *none* of
		 * these partitions will be registered.
		 */

		add_mtd_partitions(&g->mtd, g->chip_partitions, g->numchips);

	}

	g->n_concat = 0;
	memset(g->concat, 0, sizeof(g->concat));

	/*
	 * Loop over physical chips, handling sub-partitions for each in turn.
	 */

	for (chip = 0; chip < g->numchips; chip++) {

		/*
		 * If the module parameter "add_mtd_chip" is set, then we want
		 * to register the partition MTD we made for this chip. Among
		 * other things, this will make it visible to user space.
		 */

		if (add_mtd_chip) {
			printk(KERN_NOTICE "Registering an MTD for chip %d\n",
			       chip);
			add_mtd_device(g->chip_mtds[chip]);
		}

		/*
		 * Check if the platform data includes partition descriptions
		 * for this chip. If not, move to the next one.
		 */

		if (chip >= gpd->chip_count)
			continue;

		/*
		 * If control arrives here, the platform data includes
		 * partition descriptions for this chip. Loop over all the
		 * partition descriptions for this chip, checking if any appear
		 * on the list to be concatenated.
		 *
		 * For each partition that is to be concatenated, set its "mtdp"
		 * pointer such that the new MTD will be added to an array
		 * rather than registered in the public MTD list. Later, after
		 * we've concatenated all these MTDs, we will register the
		 * final result.
		 */

		add_mtd_partitions(g->chip_mtds[chip],
				   gpd->chip_partitions[chip].partitions,
				   gpd->chip_partitions[chip].nr_partitions);
	}
	if (g->n_concat > 0) {

#ifdef CONFIG_MTD_CONCAT
		if (g->n_concat == 1)
#endif
			for (p = 0; p < g->n_concat; p++)
				add_mtd_device(g->concat[p]);
#ifdef CONFIG_MTD_CONCAT
		if (g->n_concat > 1) {
			g->concat_mtd = mtd_concat_create(g->concat,
							  g->n_concat,
							  gpd->concat_name);
			if (g->concat_mtd)
				add_mtd_device(g->concat_mtd);
		}
#endif
	}

	/*
	 * Set a flag in the per-device data structure to show that that we
	 * created custom partitions. This is important because, when we need
	 * to disassemble them, we need to be aware of how we created them.
	 */

	g->custom_partitions = true;

#endif

}

/**
 * gpmi_delete_partitions - Remove the partitions created by this driver.
 *
 * @g:  The per-device data structure.
 */
static void gpmi_delete_partitions(struct gpmi_nand_data *g)
{

	/*
	 * First, check if MTD partitioning is even available. If not, then
	 * we don't have to do any special work.
	 */

#ifdef CONFIG_MTD_PARTITIONS
	int chip, p;

	/*
	 * If control arrives here, MTD partitioning is available. But, if we
	 * didn't construct any custom partitions, then we don't have to do any
	 * special work.
	 */

	if (!g->custom_partitions)
		return;

#ifdef CONFIG_MTD_CONCAT

	/*
	 * If control arrives here, we constructed some custom partitions, and
	 * we have to disassemble them carefully.
	 *
	 * If we concatenated any MTDs, delete the synthetic MTD first.
	 */

	if (g->concat_mtd)
		del_mtd_device(g->concat_mtd);

	/*
	 * Check if we concatenated any partitions, which now must be
	 * disassembled.
	 *
	 * This process is complicated by the fact that MTD concatenation may or
	 * may not be available. Here are the cases:
	 *
	 * * If concatenation is not available:
	 *
	 *     * De-register all the partition MTDs that would have been
	 *       concatenated.
	 *
	 * * If concatenation is available:
	 *
	 *     * If there is only one partition:
	 *
	 *         * De-register that one partition MTD.
	 */

	if (g->n_concat == 1)
#endif
		for (p = 0; p < g->n_concat; p++)
			del_mtd_device(g->concat[p]);

	/* Handle the partitions related to each physical chip. */

	for (chip = 0; chip < g->numchips; chip++) {

		/*
		 * De-register and destroy any partition MTDs that may have been
		 * derived from the MTD that represents this chip.
		 */

		del_mtd_partitions(g->chip_mtds[chip]);

		/*
		 * If the module variable 'add_mtd_chip' is set, then we
		 * registered the MTD that represents this chip. De-register it
		 * now.
		 */

		if (add_mtd_chip)
			del_mtd_device(g->chip_mtds[chip]);

		/*
		 * Free the memory we used to hold the name of the MTD that
		 * represented this chip.
		 */

		kfree(g->chip_partitions[chip].name);

	}
#endif
}

/**
 * gpmi_nand_probe - Probes for a GPMI device and, if possible, takes ownership.
 *
 * @pdev:  A pointer to the platform device.
 */
static int __init gpmi_nand_probe(struct platform_device *pdev)
{
	struct gpmi_nand_data *g;
	struct gpmi_platform_data *gpd;
	const char *part_type = 0;
	int err = 0;
	struct resource *r;
	int dma;
	unsigned long long chipsize;

	/* Allocate memory for the per-device structure (and zero it). */
	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g) {
		dev_err(&pdev->dev, "failed to allocate gpmi_nand_data\n");
		err = -ENOMEM;
		goto out1;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource\n");
		err = -ENXIO;
		goto out2;
	}
	g->io_base = ioremap(r->start, r->end - r->start + 1);
	if (!g->io_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -EIO;
		goto out2;
	}

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		err = -EIO;
		dev_err(&pdev->dev, "can't get IRQ resource\n");
		goto out3;
	}

	gpd = (struct gpmi_platform_data *)pdev->dev.platform_data;
	platform_set_drvdata(pdev, g);
	err = gpmi_nand_init_hw(pdev, 1);
	if (err)
		goto out3;

	init_timer(&g->timer);
	g->timer.data = (unsigned long)g;
	g->timer.function = gpmi_timer_expiry;
	g->timer.expires = jiffies + 4 * HZ;
	add_timer(&g->timer);
	dev_dbg(&pdev->dev, "%s: timer set to %ld\n",
		__func__, jiffies + 4 * HZ);

	g->reg_uA = gpd->io_uA;
	g->regulator = regulator_get(&pdev->dev, "mmc_ssp-2");
	if (g->regulator && !IS_ERR(g->regulator)) {
		regulator_set_mode(g->regulator, REGULATOR_MODE_NORMAL);
	} else
		g->regulator = NULL;

	gpmi_set_timings(pdev, &gpmi_safe_timing);

	g->irq = r->start;
	err = request_irq(g->irq, gpmi_irq, 0, dev_name(&pdev->dev), g);
	if (err) {
		dev_err(&pdev->dev, "can't request GPMI IRQ\n");
		goto out4;
	}

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get DMA resource\n");
		goto out5;
	}

	if (r->end - r->start > GPMI_MAX_CHIPS)
		dev_info(&pdev->dev, "too spread resource: max %d chips\n",
			 GPMI_MAX_CHIPS);

	for (dma = r->start;
	     dma < min_t(int, r->end, r->start + GPMI_MAX_CHIPS); dma++) {
		err = gpmi_init_chip(pdev, g, dma - r->start, dma);
		if (err)
			goto out6;
	}

	g->cmd_buffer_size = GPMI_CMD_BUF_SZ;
	g->write_buffer_size = GPMI_WRITE_BUF_SZ;
	g->data_buffer_size = GPMI_DATA_BUF_SZ;
	g->oob_buffer_size = GPMI_OOB_BUF_SZ;

	err = gpmi_alloc_buffers(pdev, g);
	if (err) {
		dev_err(&pdev->dev, "can't setup buffers\n");
		goto out6;
	}

	g->dev = pdev;
	g->chip.priv = g;
	g->timing = gpmi_safe_timing;
	g->selected_chip = -1;
	g->ignorebad = ignorebad;	/* copy global setting */

	/* Initialize the MTD object. */

	g->mtd.priv = &g->chip;
	g->mtd.name = dev_name(&pdev->dev);
	g->mtd.owner = THIS_MODULE;

	/*
	 * Signal Control
	 */

	g->chip.cmd_ctrl = gpmi_hwcontrol;

	/*
	 * Chip Control
	 *
	 * The cmdfunc pointer is assigned elsewhere.
	 * We use the reference implementation of waitfunc.
	 */

	g->chip.dev_ready   = gpmi_dev_ready;
	g->chip.select_chip = gpmi_select_chip;

	/*
	 * Low-level I/O
	 */

	g->chip.read_byte  = gpmi_read_byte;
	g->chip.read_word  = gpmi_read_word;
	g->chip.read_buf   = gpmi_read_buf;
	g->chip.write_buf  = gpmi_write_buf;
	g->chip.verify_buf = gpmi_verify_buf;

	/*
	 * ECC Control
	 *
	 * None of these functions are necessary:
	 *     - ecc.hwctl
	 *     - ecc.calculate
	 *     - ecc.correct
	 */

	/*
	 * ECC-aware I/O
	 */

	g->chip.ecc.read_page      = gpmi_ecc_read_page;
	g->chip.ecc.read_page_raw  = gpmi_read_page_raw;
	g->chip.ecc.write_page     = gpmi_ecc_write_page;
	g->chip.ecc.write_page_raw = gpmi_write_page_raw;

	/*
	 * High-level I/O
	 *
	 * This driver doesn't assign the erase_cmd pointer at the NAND Flash
	 * chip level. Instead, it intercepts the erase operation at the MTD
	 * level (see the assignment to mtd.erase below).
	 */

	g->chip.write_page    = gpmi_write_page;
	g->chip.ecc.read_oob  = gpmi_ecc_read_oob;
	g->chip.ecc.write_oob = gpmi_ecc_write_oob;

	/*
	 * Bad Block Management
	 *
	 * We use the reference implementation of block_markbad.
	 */

	g->chip.block_bad = gpmi_block_bad;
	g->chip.scan_bbt  = gpmi_scan_bbt;

	g->chip.ecc.mode  = NAND_ECC_HW_SYNDROME;
	g->chip.ecc.size  = 512;

	g->cmd_buffer_sz  = 0;

	/*
	 * At this point, most drivers would call nand_scan(). Instead, this
	 * driver directly performs most of the same operations nand_scan()
	 * would, and introduces more initialization work in the "middle."
	 */

	if (nand_scan_ident(&g->mtd, max_chips)
		    || gpmi_scan_middle(g)
		    || nand_scan_tail(&g->mtd)) {
		dev_err(&pdev->dev, "No NAND Flash chips found\n");
		/* errors found on some step */
		goto out7;
	}

	/* Completely disallow partial page writes. */

	g->chip.options     |= NAND_NO_SUBPAGE_WRITE;
	g->chip.subpagesize  = g->mtd.writesize;
	g->mtd.subpage_sft   = 0;

	/* Hook erase operations at the MTD level. */

	g->mtd.erase = gpmi_erase;

	/* Hook OOB read and write operations at the MTD level. */

	g->saved_read_oob  = g->mtd.read_oob;
	g->saved_write_oob = g->mtd.write_oob;
	g->mtd.read_oob    = gpmi_read_oob;
	g->mtd.write_oob   = gpmi_write_oob;

#ifdef CONFIG_MTD_PARTITIONS

	/*
	 * Check if we got any information about the platform. If not, then we
	 * have no guidance about how to set up MTD partitions, so we should
	 * just leave now.
	 */

	if (gpd == NULL)
		goto out_all;

	/*
	 * Check if the platform has specified a list of partition description
	 * parsers. If so, look for partitioning information from the parsers
	 * associated with the *first* partition set. We ignore any parsers
	 * attached to the second partition set (which makes having them rather
	 * silly, doesn't it?).
	 *
	 * Notice that any discovered partitions are recorded in the per-device
	 * data.
	 *
	 * Notice that, if the parsers find some partitions, we set the
	 * partition type to "command line". This isn't strictly accurate
	 * because we don't know which parsers were used, so we don't know if
	 * these partitions are *really* coming from a command line.
	 */

	if (gpd->chip_partitions[0].part_probe_types) {
		g->nr_parts = parse_mtd_partitions(&g->mtd,
						   gpd->chip_partitions[0].
						   part_probe_types,
						   &g->chip_partitions,
						   0);
		if (g->nr_parts > 0)
			part_type = "command line";
		else
			g->nr_parts = 0;
	}

	/*
	 * If no partition descriptions have been discovered, but the platform
	 * data has partitions for us, then adopt the partitions that came from
	 * the platform.
	 */

	if (g->nr_parts == 0 && gpd->chip_partitions[0].partitions) {
		g->parts = gpd->chip_partitions[0].partitions;
		g->nr_parts = gpd->chip_partitions[0].nr_partitions;
		part_type = "static";
	}

	/*
	 * At this point, we should have partition information either from the
	 * a parser or the platform. If we have nothing yet, then forget the
	 * whole partitioning thing.
	 */

	if (g->nr_parts == 0) {
		dev_err(&pdev->dev, "Neither part_probe_types nor "
			"partitions was specified in platform_data");
		goto out_all;
	}

	/*
	 * If control arrives here, we have some partitions to use. Announce
	 * what we've found.
	 */

	dev_info(&pdev->dev, "Using %s partition definition\n", part_type);

	/*
	 * Transcribe the number of chips discovered by the scan to the
	 * per-device data structure. This assignment should NOT happen here,
	 * because it's conditional on whether partitioning is on.
	 */

	g->numchips = g->chip.numchips;

	/*
	 * Compute the number of bytes per chip by taking the size of the entire
	 * medium and dividing by the number of chips.
	 */

	chipsize = g->mtd.size;
	do_div(chipsize, (unsigned long)g->numchips);

	/*
	 * Check if the partitions we're using came from a parser (in which case
	 * they should be used as-is), or came from the platform data (in which
	 * case we have some special processing we like to do).
	 */

	if (!strcmp(part_type, "command line"))
		add_mtd_partitions(&g->mtd, g->chip_partitions, g->nr_parts);
	else
		gpmi_create_partitions(g, gpd, chipsize);

	/*
	 * Check if we're supposed to register the MTD that represents the
	 * entire medium.
	 */

	if (add_mtd_entire) {
		printk(KERN_NOTICE "Adding MTD covering the whole flash\n");
		add_mtd_device(&g->mtd);
	}

#else

	/*
	 * If control arrives here, the MTD partitioning facility isn't
	 * available. We just register the MTD that represents the entire
	 * medium.
	 */

	add_mtd_device(&g->mtd);

#endif

	/* Initialize the Unique ID facility. */

	gpmi_uid_init("nand", &g->mtd, gpd->uid_offset, gpd->uid_size);

	/*
	 * Check if we should export sysfs entries.
	 *
	 * This configuration variable should be destroyed, and this driver
	 * should *always* create the sysfs entries.
	 */

#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES
	gpmi_sysfs(pdev, true);
#endif

	/* If control arrives here, everything worked. Return success. */

	return 0;

out_all:
	ecc8_exit();
	bch_exit();
out7:
	nand_release(&g->mtd);
	gpmi_free_buffers(pdev, g);
out6:
	gpmi_deinit_chip(pdev, g, -1);
out5:
	free_irq(g->irq, g);
out4:
	del_timer_sync(&g->timer);
	gpmi_nand_release_hw(pdev);
out3:
	platform_set_drvdata(pdev, NULL);
	iounmap(g->io_base);
out2:
	kfree(g);
out1:
	return err;
}

/**
 * gpmi_nand_remove - Dissociates this driver from the given device.
 *
 * @pdev:  A pointer to the platform device.
 */
static int __devexit gpmi_nand_remove(struct platform_device *pdev)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	int i = 0;
#ifdef CONFIG_MTD_PARTITIONS
	struct gpmi_platform_data *gpd = pdev->dev.platform_data;
	struct mtd_partition *platf_parts;
#endif

	gpmi_delete_partitions(g);
	del_timer_sync(&g->timer);
	gpmi_uid_remove("nand");

#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES
	gpmi_sysfs(pdev, false);
#endif

	nand_release(&g->mtd);
	gpmi_free_buffers(pdev, g);
	gpmi_deinit_chip(pdev, g, -1);
	gpmi_nand_release_hw(pdev);
	free_irq(g->irq, g);
	if (g->regulator)
		regulator_put(g->regulator);

#ifdef CONFIG_MTD_PARTITIONS
	if (i < gpd->chip_count && gpd->chip_partitions[i].partitions)
		platf_parts = gpd->chip_partitions[i].partitions;
	else
		platf_parts = NULL;
	if (g->chip_partitions && g->chip_partitions != platf_parts)
		kfree(g->chip_partitions);
#endif

	iounmap(g->io_base);
	kfree(g);

	return 0;
}

#ifdef CONFIG_PM

/**
 * gpmi_nand_suspend() - Suspends this driver.
 *
 * @pdev:  A pointer to the owning struct platform_device.
 * @pm:    For future use, currently unused.
 */
static int gpmi_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	int r = 0;

	/* If the driver suspended itself due to inactivity, wake it up. */

	if (g->self_suspended)
		gpmi_self_wakeup(g);

	/* Deactivate the inactivity timer. */

	del_timer_sync(&g->timer);

	/*
	 * Suspend MTD's use of this device and, if that works, then shut down
	 * the actual hardware.
	 */

	r = g->mtd.suspend(&g->mtd);
	if (r == 0)
		gpmi_nand_release_hw(pdev);

	return r;

}

/**
 * gpmi_nand_resume() - Resumes this driver from suspend.
 *
 * @pdev:  A pointer to the owning struct platform_device.
 */
static int gpmi_nand_resume(struct platform_device *pdev)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	int r;

	/*
	 * Spin up the hardware.
	 *
	 * Unfortunately, this code ignores the result of hardware
	 * initialization and spins up the driver unconditionally.
	 */

	r = gpmi_nand_init_hw(pdev, 1);
	gpmi_set_timings(pdev, &g->timing);

	/* Tell MTD it can use this device again. */

	g->mtd.resume(&g->mtd);

	/* Re-instate the inactivity timer. */

	g->timer.expires = jiffies + 4 * HZ;
	add_timer(&g->timer);

	return r;

}

#else
#define gpmi_nand_suspend	NULL
#define gpmi_nand_resume	NULL
#endif

#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES

/**
 * show_timings() - Shows the current NAND Flash timing.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_timings(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	struct gpmi_nand_timing *ptm;
	struct gpmi_nand_data *g = dev_get_drvdata(d);

	ptm = &g->timing;
	return sprintf(buf, "DATA_SETUP %d, DATA_HOLD %d, "
		       "ADDR_SETUP %d, DSAMPLE_TIME %d\n",
		       ptm->data_setup, ptm->data_hold,
		       ptm->address_setup, ptm->dsample_time);
}

/**
 * store_timings() - Sets the current NAND Flash timing.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_timings(struct device *d, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	const char *p, *end;
	struct gpmi_nand_timing t;
	struct gpmi_nand_data *g = dev_get_drvdata(d);
	char tmps[20];
	u8 *timings[] = {
		&t.data_setup,
		&t.data_hold,
		&t.address_setup,
		&t.dsample_time,
		NULL,
	};
	u8 **timing = timings;

	p = buf;

	/* parse values */
	while (*timing != NULL) {
		unsigned long t_long;

		end = strchr(p, ',');
		memset(tmps, 0, sizeof(tmps));
		if (end)
			strncpy(tmps, p, min_t(int, sizeof(tmps) - 1, end - p));
		else
			strncpy(tmps, p, sizeof(tmps) - 1);

		if (strict_strtoul(tmps, 0, &t_long) < 0)
			return -EINVAL;

		if (t_long > 255)
			return -EINVAL;

		**timing = (u8) t_long;
		timing++;

		if (!end && *timing)
			return -EINVAL;
		p = end + 1;
	}

	gpmi_set_timings(g->dev, &t);

	return size;
}

/**
 * show_stat() - Shows current statistics.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_stat(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "copies\t\t%dff pages\t%d\n", copies, ff_writes);
}

/**
 * show_chips() - Shows the number of physical chips that were discovered.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_chips(struct device *d, struct device_attribute *attr,
			  char *buf)
{
	struct gpmi_nand_data *g = dev_get_drvdata(d);
	return sprintf(buf, "%d\n", g->numchips);
}

/**
 * show_ignorebad() - Shows the value of the 'ignorebad' flag.
 *
 * @d:     The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_ignorebad(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	struct gpmi_nand_data *g = dev_get_drvdata(d);

	return sprintf(buf, "%d\n", g->ignorebad);
}

/**
 * store_ignorebad() - Sets the value of the 'ignorebad' flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_ignorebad(struct device *d, struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct gpmi_nand_data *g = dev_get_drvdata(d);
	const char *p = buf;
	unsigned long v;

	if (strict_strtoul(p, 0, &v) < 0)
		return size;
	if (v > 0)
		v = 1;
	if (v != g->ignorebad) {
		if (v) {
			g->bbt = g->chip.bbt;
			g->chip.bbt = NULL;
			g->ignorebad = 1;
		} else {
			g->chip.bbt = g->bbt;
			g->ignorebad = 0;
		}
	}
	return size;
}

static DEVICE_ATTR(timings, 0644, show_timings, store_timings);
static DEVICE_ATTR(stat, 0444, show_stat, NULL);
static DEVICE_ATTR(ignorebad, 0644, show_ignorebad, store_ignorebad);
static DEVICE_ATTR(numchips, 0444, show_chips, NULL);

static struct device_attribute *gpmi_attrs[] = {
	&dev_attr_timings,
	&dev_attr_stat,
	&dev_attr_ignorebad,
	&dev_attr_numchips,
	NULL,
};

/**
 * gpmi_sysfs() - Creates or removes sysfs nodes.
 *
 * @pdev:    A pointer to the owning platform device.
 * @create:  Indicates the nodes are to be created (otherwise, removed).
 */
int gpmi_sysfs(struct platform_device *pdev, int create)
{
	int err = 0;
	int i;

	if (create) {
		for (i = 0; gpmi_attrs[i]; i++) {
			err = device_create_file(&pdev->dev, gpmi_attrs[i]);
			if (err)
				break;
		}
		if (err)
			while (--i >= 0)
				device_remove_file(&pdev->dev, gpmi_attrs[i]);
	} else {
		for (i = 0; gpmi_attrs[i]; i++)
			device_remove_file(&pdev->dev, gpmi_attrs[i]);
	}
	return err;
}

#endif

/*
 * The global list of ECC descriptors.
 *
 * Each descriptor represents an ECC option that's available to the driver.
 */

static LIST_HEAD(gpmi_ecc_descriptor_list);

/**
 * gpmi_ecc_add() - Adds the given ECC descriptor.
 *
 * @name:  The name of interest.
 */
void gpmi_ecc_add(struct gpmi_ecc_descriptor *chip)
{
	list_add(&chip->list, &gpmi_ecc_descriptor_list);
}
EXPORT_SYMBOL_GPL(gpmi_ecc_add);

/**
 * gpmi_ecc_remove() - Removes an ECC descriptor with the given name.
 *
 * @name:  The name of interest.
 */
void gpmi_ecc_remove(struct gpmi_ecc_descriptor *chip)
{
	list_del(&chip->list);
}
EXPORT_SYMBOL_GPL(gpmi_ecc_remove);

/**
 * gpmi_ecc_find() - Tries to find an ECC descriptor with the given name.
 *
 * @name:  The name of interest.
 */
struct gpmi_ecc_descriptor *gpmi_ecc_find(char *name)
{
	struct gpmi_ecc_descriptor *c;

	list_for_each_entry(c, &gpmi_ecc_descriptor_list, list)
		if (strncmp(c->name, name, sizeof(c->name)) == 0)
			return c;

	return NULL;

}
EXPORT_SYMBOL_GPL(gpmi_ecc_find);

/*
 * This structure represents this driver to the platform management system.
 */

static struct platform_driver gpmi_nand_driver = {
	.probe = gpmi_nand_probe,
	.remove = __devexit_p(gpmi_nand_remove),
	.driver = {
		   .name = "gpmi",
		   .owner = THIS_MODULE,
		   },
	.suspend = gpmi_nand_suspend,
	.resume = gpmi_nand_resume,
};

static int __init gpmi_nand_init(void)
{
	int  return_value;

	pr_info("GPMI NAND Flash driver\n");

	/* Initialize the BCH and ECC8 hardware blocks. */

	bch_init();
	ecc8_init();

	/* Attempt to register this driver with the platform. */

	return_value = platform_driver_register(&gpmi_nand_driver);

	if (return_value)
		pr_err("GPMI NAND Flash driver registration failed\n");

	return return_value;

}

static void __exit gpmi_nand_exit(void)
{
	pr_info("GPMI NAND Flash driver exiting...\n");
	platform_driver_unregister(&gpmi_nand_driver);
}

module_init(gpmi_nand_init);
module_exit(gpmi_nand_exit);
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPMI NAND Flash driver");
module_param(max_chips, int, 0400);
module_param(clk, long, 0400);
module_param(bch, int, 0600);
module_param(map_buffers, int, 0600);
module_param(raw_mode, int, 0600);
module_param(debug, int, 0600);
module_param(add_mtd_entire, int, 0400);
module_param(add_mtd_chip, int, 0400);
module_param(ignorebad, int, 0400);
