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

static int debug;
static int copies;
static int map_buffers = true;
static int ff_writes;
static int raw_mode;
static int add_mtd_entire;
static int add_mtd_chip;
static int ignorebad;
static int max_chips = 4;
static long clk = -1;
static int bch /* = 0 */ ;

static int gpmi_nand_init_hw(struct platform_device *pdev, int request_pins);
static void gpmi_nand_release_hw(struct platform_device *pdev);
static int gpmi_dma_exchange(struct gpmi_nand_data *g,
			     struct stmp3xxx_dma_descriptor *dma);
static void gpmi_read_buf(struct mtd_info *mtd, uint8_t * buf, int len);

struct gpmi_nand_timing gpmi_safe_timing = {
	.address_setup = 25,
	.data_setup = 80,
	.data_hold = 60,
	.dsample_time = 6,
};

/*
 * define OOB placement schemes for 4k and 2k page devices
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

static inline u32 gpmi_cycles_ceil(u32 ntime, u32 period)
{
	int k;

	k = (ntime + period - 1) / period;
	if (k == 0)
		k++;
	return k;
}

/**
 * gpmi_timer_expiry - timer expiry handling
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
 * gpmi_set_timings - set GPMI timings
 * @pdev: pointer to GPMI platform device
 * @tm: pointer to structure &gpmi_nand_timing with new timings
 *
 * During initialization, GPMI uses safe sub-optimal timings, which
 * can be changed after reading boot control blocks
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
 * gpmi_nand_init_hw - initialize the hardware
 * @pdev: pointer to platform device
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

	g->clk = clk_get(NULL, "gpmi");
	if (IS_ERR(g->clk)) {
		err = PTR_ERR(g->clk);
		dev_err(&pdev->dev, "cannot set failsafe clockrate\n");
		goto out;
	}
	clk_enable(g->clk);
	if (clk <= 0)
		clk = 24000;	/* safe setting, some chips do not work on
				   speeds >= 24kHz */
	clk_set_rate(g->clk, clk);

	clk = clk_get_rate(g->clk);

	if (request_pins)
		gpd->pinmux(1);

	stmp3xxx_reset_block(HW_GPMI_CTRL0 + REGS_GPMI_BASE, 1);

	/* this CLEARS reset, despite of its name */
	stmp3xxx_setl(BM_GPMI_CTRL1_DEV_RESET, REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* IRQ polarity */
	stmp3xxx_setl(BM_GPMI_CTRL1_ATA_IRQRDY_POLARITY,
		      REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* ...and ECC module */
	stmp3xxx_setl(bch_mode(), REGS_GPMI_BASE + HW_GPMI_CTRL1);

	/* choose NAND mode (1 means ATA, 0 - NAND */
	stmp3xxx_clearl(BM_GPMI_CTRL1_GPMI_MODE,
			REGS_GPMI_BASE + HW_GPMI_CTRL1);

out:
	return err;
}

/**
 * gpmi_nand_release_hw - free the hardware
 * @pdev: pointer to platform device
 *
 * In opposite to gpmi_nand_init_hw, release all acquired resources
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
 * gpmi_dma_exchange - run DMA to exchange with NAND chip
 *
 * @g: structure associated with NAND chip
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
 * gpmi_ecc_read_page - replacement for nand_read_page
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

	/* write plain data */
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
 * @mtd: MTD device
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
 * gpmi_erase - erase a block and update BBT table
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
 * gpmi_dev_ready - poll the RDY pin
 *
 * @mtd: MTD device
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
 * gpmi_hwcontrol - set command/address byte to the device
 *
 * @mtd: MTD device
 * @cmd: command byte
 * @ctrl: control flags
 */
static void gpmi_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;
	struct stmp3xxx_dma_descriptor *chain = g->cchip->d;
	int ret;

	if ((ctrl & (NAND_ALE | NAND_CLE))) {
		if (cmd != NAND_CMD_NONE)
			g->cmd_buffer[g->cmd_buffer_sz++] = cmd;
		return;
	}

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
 * @irq:	irq no
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

static void gpmi_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	struct gpmi_nand_data *g = chip->priv;

	g->saved_command(mtd, command, column, page_addr);
}

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
 * perform the needed steps between nand_scan_ident and nand_scan_tail
 */
static int gpmi_scan_middle(struct gpmi_nand_data *g)
{
	int oobsize = 0;

	/* Limit to 2G size due to Kernel larger 4G space support */
	if (g->mtd.size == 0) {
		g->mtd.size = 1 << 31;
		g->chip.chipsize = do_div(g->mtd.size, g->chip.numchips);
	}

	g->ecc_oob_bytes = 9;
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

	g->saved_command = g->chip.cmdfunc;
	g->chip.cmdfunc = gpmi_command;

	if (oobsize > 0) {
		g->mtd.oobsize = oobsize;
		/* otherwise error; oobsize should be set
		   in valid cases */
		g->hc = gpmi_hwecc_chip_find("ecc8");
		g->hc->setup(g->hc, 0, g->mtd.writesize, g->mtd.oobsize);
		return 0;
	}

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

#if 0
static int gpmi_to_concat(struct mtd_partition *part, char **list)
{
	while (list && *list) {
		if (strcmp(part->name, *list) == 0) {
			pr_debug("Partition '%s' will be concatenated\n",
				 part->name);
			return true;
		}
		list++;
	}
	pr_debug("Partition '%s' is left as-is", part->name);
	return false;
}
#endif

static void gpmi_create_partitions(struct gpmi_nand_data *g,
				   struct gpmi_platform_data *gpd,
				   uint64_t chipsize)
{
#ifdef CONFIG_MTD_PARTITIONS
	int chip, p;
	char chipname[20];

	if (g->numchips == 1)
		g->masters[0] = &g->mtd;
	else {
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
		add_mtd_partitions(&g->mtd, g->chip_partitions, g->numchips);
	}
	g->n_concat = 0;
	memset(g->concat, 0, sizeof(g->concat));
	for (chip = 0; chip < g->numchips; chip++) {
		if (add_mtd_chip) {
			printk(KERN_NOTICE "Adding MTD for the chip %d\n",
			       chip);
			add_mtd_device(g->masters[chip]);
		}
		if (chip >= gpd->items)
			continue;
		add_mtd_partitions(g->masters[chip],
				   gpd->parts[chip].partitions,
				   gpd->parts[chip].nr_partitions);
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
	g->custom_partitions = true;
#endif
}

static void gpmi_delete_partitions(struct gpmi_nand_data *g)
{
#ifdef CONFIG_MTD_PARTITIONS
	int chip, p;

	if (!g->custom_partitions)
		return;
#ifdef CONFIG_MTD_CONCAT
	if (g->concat_mtd)
		del_mtd_device(g->concat_mtd);
	if (g->n_concat == 1)
#endif
		for (p = 0; p < g->n_concat; p++)
			del_mtd_device(g->concat[p]);

	for (chip = 0; chip < g->numchips; chip++) {
		del_mtd_partitions(g->masters[chip]);
		if (add_mtd_chip)
			del_mtd_device(g->masters[chip]);
		kfree(g->chip_partitions[chip].name);
	}
#endif
}

/**
 * gpmi_nand_probe - probe for the GPMI device
 *
 * Probe for GPMI device and discover NAND chips
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

	/* Allocate memory for the device structure (and zero it) */
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

	g->mtd.priv = &g->chip;
	g->mtd.name = dev_name(&pdev->dev);
	g->mtd.owner = THIS_MODULE;

	g->chip.cmd_ctrl = gpmi_hwcontrol;
	g->chip.read_word = gpmi_read_word;
	g->chip.read_byte = gpmi_read_byte;
	g->chip.read_buf = gpmi_read_buf;
	g->chip.write_buf = gpmi_write_buf;
	g->chip.select_chip = gpmi_select_chip;
	g->chip.verify_buf = gpmi_verify_buf;
	g->chip.dev_ready = gpmi_dev_ready;

	g->chip.ecc.mode = NAND_ECC_HW_SYNDROME;
	g->chip.ecc.write_oob = gpmi_ecc_write_oob;
	g->chip.ecc.read_oob = gpmi_ecc_read_oob;
	g->chip.ecc.write_page = gpmi_ecc_write_page;
	g->chip.ecc.read_page = gpmi_ecc_read_page;
	g->chip.ecc.read_page_raw = gpmi_read_page_raw;
	g->chip.ecc.write_page_raw = gpmi_write_page_raw;
	g->chip.ecc.size = 512;

	g->chip.write_page = gpmi_write_page;

	g->chip.scan_bbt = gpmi_scan_bbt;
	g->chip.block_bad = gpmi_block_bad;

	g->cmd_buffer_sz = 0;

	/* first scan to find the device and get the page size */
	if (nand_scan_ident(&g->mtd, max_chips)
	    || gpmi_scan_middle(g)
	    || nand_scan_tail(&g->mtd)) {
		dev_err(&pdev->dev, "No NAND found\n");
		/* errors found on some step */
		goto out7;
	}

	g->chip.options |= NAND_NO_SUBPAGE_WRITE;
	g->chip.subpagesize = g->mtd.writesize;
	g->mtd.subpage_sft = 0;

	g->mtd.erase = gpmi_erase;

	g->saved_read_oob = g->mtd.read_oob;
	g->saved_write_oob = g->mtd.write_oob;
	g->mtd.read_oob = gpmi_read_oob;
	g->mtd.write_oob = gpmi_write_oob;

#ifdef CONFIG_MTD_PARTITIONS
	if (gpd == NULL)
		goto out_all;

	if (gpd->parts[0].part_probe_types) {
		g->nr_parts = parse_mtd_partitions(&g->mtd,
						   gpd->parts[0].
						   part_probe_types, &g->parts,
						   0);
		if (g->nr_parts > 0)
			part_type = "command line";
		else
			g->nr_parts = 0;
	}

	if (g->nr_parts == 0 && gpd->parts[0].partitions) {
		g->parts = gpd->parts[0].partitions;
		g->nr_parts = gpd->parts[0].nr_partitions;
		part_type = "static";
	}

	if (g->nr_parts == 0) {
		dev_err(&pdev->dev, "Neither part_probe_types nor "
			"partitions was specified in platform_data");
		goto out_all;
	}

	dev_info(&pdev->dev, "Using %s partition definition\n", part_type);

	g->numchips = g->chip.numchips;
	chipsize = g->mtd.size;
	do_div(chipsize, (unsigned long)g->numchips);

	if (!strcmp(part_type, "command line"))
		add_mtd_partitions(&g->mtd, g->parts, g->nr_parts);
	else
		gpmi_create_partitions(g, gpd, chipsize);

	if (add_mtd_entire) {
		printk(KERN_NOTICE "Adding MTD covering the whole flash\n");
		add_mtd_device(&g->mtd);
	}
#else
	add_mtd_device(&g->mtd);
#endif
	gpmi_uid_init("nand", &g->mtd, gpd->uid_offset, gpd->uid_size);

#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES
	gpmi_sysfs(pdev, true);
#endif
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
 * gpmi_nand_remove - remove a GPMI device
 *
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
	if (i < gpd->items && gpd->parts[i].partitions)
		platf_parts = gpd->parts[i].partitions;
	else
		platf_parts = NULL;
	if (g->parts && g->parts != platf_parts)
		kfree(g->parts);
#endif
	iounmap(g->io_base);
	kfree(g);

	return 0;
}

#ifdef CONFIG_PM
static int gpmi_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	int r = 0;

	if (g->self_suspended)
		gpmi_self_wakeup(g);
	del_timer_sync(&g->timer);

	r = g->mtd.suspend(&g->mtd);
	if (r == 0)
		gpmi_nand_release_hw(pdev);

	return r;
}

static int gpmi_nand_resume(struct platform_device *pdev)
{
	struct gpmi_nand_data *g = platform_get_drvdata(pdev);
	int r;

	r = gpmi_nand_init_hw(pdev, 1);
	gpmi_set_timings(pdev, &g->timing);
	g->mtd.resume(&g->mtd);
	g->timer.expires = jiffies + 4 * HZ;
	add_timer(&g->timer);
	return r;
}
#else
#define gpmi_nand_suspend	NULL
#define gpmi_nand_resume	NULL
#endif

#ifdef CONFIG_MTD_NAND_GPMI_SYSFS_ENTRIES
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

static ssize_t show_stat(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "copies\t\t%dff pages\t%d\n", copies, ff_writes);
}

static ssize_t show_chips(struct device *d, struct device_attribute *attr,
			  char *buf)
{
	struct gpmi_nand_data *g = dev_get_drvdata(d);
	return sprintf(buf, "%d\n", g->numchips);
}

static ssize_t show_ignorebad(struct device *d, struct device_attribute *attr,
			      char *buf)
{
	struct gpmi_nand_data *g = dev_get_drvdata(d);

	return sprintf(buf, "%d\n", g->ignorebad);
}

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

static LIST_HEAD(gpmi_hwecc_chips);

void gpmi_hwecc_chip_add(struct gpmi_hwecc_chip *chip)
{
	list_add(&chip->list, &gpmi_hwecc_chips);
}

EXPORT_SYMBOL_GPL(gpmi_hwecc_chip_add);

void gpmi_hwecc_chip_remove(struct gpmi_hwecc_chip *chip)
{
	list_del(&chip->list);
}

EXPORT_SYMBOL_GPL(gpmi_hwecc_chip_remove);

struct gpmi_hwecc_chip *gpmi_hwecc_chip_find(char *name)
{
	struct gpmi_hwecc_chip *c;

	list_for_each_entry(c, &gpmi_hwecc_chips, list)
	    if (strncmp(c->name, name, sizeof(c->name)) == 0)
		return c;
	return NULL;
}

EXPORT_SYMBOL_GPL(gpmi_hwecc_chip_find);

static int __init gpmi_nand_init(void)
{
	bch_init();
	ecc8_init();
	return platform_driver_register(&gpmi_nand_driver);
}

static void __exit gpmi_nand_exit(void)
{
	platform_driver_unregister(&gpmi_nand_driver);
}

module_init(gpmi_nand_init);
module_exit(gpmi_nand_exit);
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPMI NAND driver");
module_param(max_chips, int, 0400);
module_param(clk, long, 0400);
module_param(bch, int, 0600);
module_param(map_buffers, int, 0600);
module_param(raw_mode, int, 0600);
module_param(debug, int, 0600);
module_param(add_mtd_entire, int, 0400);
module_param(add_mtd_chip, int, 0400);
module_param(ignorebad, int, 0400);
