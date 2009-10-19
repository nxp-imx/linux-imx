/*
 * Freescale STMP378X SPI master driver
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <asm/dma.h>

#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/regs-ssp.h>
#include <mach/regs-apbh.h>
#include "spi_stmp.h"

/* 0 means DMA modei(recommended, default), !0 - PIO mode */
static int pio /* = 0 */;
static int debug;

/**
 * stmp_spi_init_hw
 *
 * Initialize the SSP port
 */
static int stmp_spi_init_hw(struct stmp_spi *ss)
{
	int err;

	err = stmp37xx_spi_pins_request((char *)dev_name(ss->master_dev), ss->id);
	if (err)
		goto out;

	ss->clk = clk_get(NULL, "ssp");
	if (IS_ERR(ss->clk)) {
		err = PTR_ERR(ss->clk);
		goto out_free_pins;
	}
	clk_enable(ss->clk);

	stmp3xxx_reset_block((void *)ss->regs, 0);
	stmp3xxx_dma_reset_channel(ss->dma);

	return 0;

out_free_pins:
	stmp37xx_spi_pins_release((char *)dev_name(ss->master_dev), ss->id);
out:
	return err;
}

static void stmp_spi_release_hw(struct stmp_spi *ss)
{
	if (ss->clk && !IS_ERR(ss->clk)) {
		clk_disable(ss->clk);
		clk_put(ss->clk);
	}
	stmp37xx_spi_pins_release((char *)dev_name(ss->master_dev), ss->id);
}

static int stmp_spi_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{
	u8 bits_per_word;
	u32 hz;
	struct stmp_spi *ss /* = spi_master_get_devdata(spi->master) */;
	u16 rate;

	ss = spi_master_get_devdata(spi->master);

	bits_per_word = spi->bits_per_word;
	if (t && t->bits_per_word)
		bits_per_word = t->bits_per_word;

	/*
	  Calculate speed:
		- by default, use maximum speed from ssp clk
		- if device overrides it, use it
		- if transfer specifies other speed, use transfer's one
	 */
	hz = 1000 * ss->speed_khz / ss->divider;
	if (spi->max_speed_hz)
		hz = min(hz, spi->max_speed_hz);
	if (t && t->speed_hz)
		hz = min(hz, t->speed_hz);

	if (hz == 0) {
		dev_err(&spi->dev, "Cannot continue with zero clock\n");
		return -EINVAL;
	}

	if (bits_per_word != 8) {
		dev_err(&spi->dev, "%s, unsupported bits_per_word=%d\n",
			__func__, bits_per_word);
		return -EINVAL;
	}

	dev_dbg(&spi->dev, "Requested clk rate = %uHz, max = %uHz/%d = %uHz\n",
		hz, ss->speed_khz, ss->divider,
		ss->speed_khz * 1000 / ss->divider);

	if (ss->speed_khz * 1000 / ss->divider < hz) {
		dev_err(&spi->dev, "%s, unsupported clock rate %uHz\n",
			__func__, hz);
		return -EINVAL;
	}

	rate = 1000 * ss->speed_khz/ss->divider/hz;

	__raw_writel(BF(ss->divider, SSP_TIMING_CLOCK_DIVIDE) |
		BF(rate - 1, SSP_TIMING_CLOCK_RATE), ss->regs + HW_SSP_TIMING);

	__raw_writel(BF(BV_SSP_CTRL1_SSP_MODE__SPI, SSP_CTRL1_SSP_MODE) |
		BF(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS, SSP_CTRL1_WORD_LENGTH) |
		((spi->mode & SPI_CPOL) ? BM_SSP_CTRL1_POLARITY : 0) |
		((spi->mode & SPI_CPHA) ? BM_SSP_CTRL1_PHASE : 0) |
		(pio ? 0 : BM_SSP_CTRL1_DMA_ENABLE), ss->regs + HW_SSP_CTRL1);

	stmp3xxx_setl(0x00, ss->regs + HW_SSP_CMD0);

	return 0;
}


static void stmp_spi_cleanup(struct spi_device *spi)
{
	struct stmp37xx_spi_platform_data *pdata = spi->dev.platform_data;

	if (pdata && pdata->hw_release)
		pdata->hw_release(spi);
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA)
static int stmp_spi_setup(struct spi_device *spi)
{
	struct stmp37xx_spi_platform_data *pdata;
	struct stmp_spi *ss;
	int err = 0;

	ss = spi_master_get_devdata(spi->master);

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~MODEBITS) {
		dev_err(&spi->dev, "%s: unsupported mode bits %x\n",
			__func__, spi->mode & ~MODEBITS);
		err = -EINVAL;
		goto out;
	}

	dev_dbg(&spi->dev, "%s, mode %d, %u bits/w\n",
		__func__, spi->mode & MODEBITS, spi->bits_per_word);

	pdata = spi->dev.platform_data;

	if (pdata && pdata->hw_init) {
		err = pdata->hw_init(spi);
		if (err)
			goto out;
	}

	err = stmp_spi_setup_transfer(spi, NULL);
	if (err)
		goto out2;
	return 0;

out2:
	if (pdata)
		pdata->hw_release(spi);
out:
	dev_err(&spi->dev, "Failed to setup transfer, error = %d\n", err);
	return err;
}

static inline u32 stmp_spi_cs(unsigned cs)
{
	return  ((cs & 1) ? BM_SSP_CTRL0_WAIT_FOR_CMD : 0) |
		((cs & 2) ? BM_SSP_CTRL0_WAIT_FOR_IRQ : 0);
}

static int stmp_spi_txrx_dma(struct stmp_spi *ss, int cs,
		unsigned char *buf, dma_addr_t dma_buf, int len,
		int *first, int *last, int write)
{
	u32 c0 = 0;
	dma_addr_t spi_buf_dma = dma_buf;
	int count, status = 0;
	enum dma_data_direction dir = write ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	c0 |= (*first ? BM_SSP_CTRL0_LOCK_CS : 0);
	c0 |= (*last ? BM_SSP_CTRL0_IGNORE_CRC : 0);
	c0 |= (write ? 0 : BM_SSP_CTRL0_READ);
	c0 |= BM_SSP_CTRL0_DATA_XFER;

	c0 |= stmp_spi_cs(cs);

	c0 |= BF(len, SSP_CTRL0_XFER_COUNT);

	if (!dma_buf)
		spi_buf_dma = dma_map_single(ss->master_dev, buf, len, dir);

	ss->d.command->cmd =
		BF(len, APBH_CHn_CMD_XFER_COUNT) |
		BF(1, APBH_CHn_CMD_CMDWORDS) |
		BM_APBH_CHn_CMD_WAIT4ENDCMD |
		BM_APBH_CHn_CMD_IRQONCMPLT  |
		BF(write ? BV_APBH_CHn_CMD_COMMAND__DMA_READ :
			BV_APBH_CHn_CMD_COMMAND__DMA_WRITE,
			APBH_CHn_CMD_COMMAND);
	ss->d.command->pio_words[0] = c0;
	ss->d.command->buf_ptr = spi_buf_dma;

	stmp3xxx_dma_reset_channel(ss->dma);
	stmp3xxx_dma_clear_interrupt(ss->dma);
	stmp3xxx_dma_enable_interrupt(ss->dma);
	init_completion(&ss->done);
	stmp3xxx_dma_go(ss->dma, &ss->d, 1);
	wait_for_completion(&ss->done);
	count = 10000;
	while ((__raw_readl(ss->regs + HW_SSP_CTRL0) & BM_SSP_CTRL0_RUN) && count--)
		continue;
	if (count <= 0) {
		printk(KERN_ERR"%c: timeout on line %s:%d\n",
				write ? 'W':'C', __func__, __LINE__);
		status = -ETIMEDOUT;
	}

	if (!dma_buf)
		dma_unmap_single(ss->master_dev, spi_buf_dma, len, dir);

	return status;
}

static inline void stmp_spi_enable(struct stmp_spi *ss)
{
	stmp3xxx_setl(BM_SSP_CTRL0_LOCK_CS, ss->regs + HW_SSP_CTRL0);
	stmp3xxx_clearl(BM_SSP_CTRL0_IGNORE_CRC, ss->regs + HW_SSP_CTRL0);
}

static inline void stmp_spi_disable(struct stmp_spi *ss)
{
	stmp3xxx_clearl(BM_SSP_CTRL0_LOCK_CS, ss->regs + HW_SSP_CTRL0);
	stmp3xxx_setl(BM_SSP_CTRL0_IGNORE_CRC, ss->regs + HW_SSP_CTRL0);
}

static int stmp_spi_txrx_pio(struct stmp_spi *ss, int cs,
		unsigned char *buf, int len,
		int *first, int *last, int write)
{
	int count;

	if (*first) {
		stmp_spi_enable(ss);
		*first = 0;
	}

	stmp3xxx_setl(stmp_spi_cs(cs), ss->regs + HW_SSP_CTRL0);

	while (len--) {
		if (*last && len == 0) {
			stmp_spi_disable(ss);
			*last = 0;
		}
		stmp3xxx_clearl(BM_SSP_CTRL0_XFER_COUNT, ss->regs + HW_SSP_CTRL0);
		stmp3xxx_setl(1, ss->regs); /* byte-by-byte */

		if (write)
			stmp3xxx_clearl(BM_SSP_CTRL0_READ, ss->regs + HW_SSP_CTRL0);
		else
			stmp3xxx_setl(BM_SSP_CTRL0_READ, ss->regs + HW_SSP_CTRL0);

		/* Run! */
		stmp3xxx_setl(BM_SSP_CTRL0_RUN, ss->regs + HW_SSP_CTRL0);
		count = 10000;
		while (((__raw_readl(ss->regs + HW_SSP_CTRL0) & BM_SSP_CTRL0_RUN) == 0) && count--)
			continue;
		if (count <= 0) {
			printk(KERN_ERR"%c: timeout on line %s:%d\n",
					write ? 'W':'C', __func__, __LINE__);
			break;
		}

		if (write)
			__raw_writel(*buf, ss->regs + HW_SSP_DATA);

		/* Set TRANSFER */
		stmp3xxx_setl(BM_SSP_CTRL0_DATA_XFER, ss->regs + HW_SSP_CTRL0);

		if (!write) {
			count = 10000;
			while (count-- &&
				(__raw_readl(ss->regs + HW_SSP_STATUS) &
					BM_SSP_STATUS_FIFO_EMPTY))
				continue;
			if (count <= 0) {
				printk(KERN_ERR"%c: timeout on line %s:%d\n",
					write ? 'W':'C', __func__, __LINE__);
				break;
			}
			*buf = (__raw_readl(ss->regs + HW_SSP_DATA) & 0xFF);
		}

		count = 10000;
		while ((__raw_readl(ss->regs + HW_SSP_CTRL0) & BM_SSP_CTRL0_RUN) && count--)
			continue;
		if (count <= 0) {
			printk(KERN_ERR"%c: timeout on line %s:%d\n",
				write ? 'W':'C', __func__, __LINE__);
			break;
		}

		/* advance to the next byte */
		buf++;
	}
	return len < 0 ? 0 : -ETIMEDOUT;
}

static int stmp_spi_handle_message(struct stmp_spi *ss, struct spi_message *m)
{
	int first, last;
	struct spi_transfer *t, *tmp_t;
	int status = 0;
	int cs;

	first = last = 0;

	cs = m->spi->chip_select;

	list_for_each_entry_safe(t, tmp_t, &m->transfers, transfer_list) {

		stmp_spi_setup_transfer(m->spi, t);

		if (&t->transfer_list == m->transfers.next)
			first = !0;
		if (&t->transfer_list == m->transfers.prev)
			last = !0;
		if (t->rx_buf && t->tx_buf) {
			pr_debug("%s: cannot send and receive simultaneously\n",
				__func__);
			return -EINVAL;
		}

		/*
		  REVISIT:
		  here driver completely ignores setting of t->cs_change
		 */
		if (t->tx_buf) {
			status = pio ?
			   stmp_spi_txrx_pio(ss, cs, (void *)t->tx_buf,
				   t->len, &first, &last, 1) :
			   stmp_spi_txrx_dma(ss, cs, (void *)t->tx_buf,
				   t->tx_dma, t->len, &first, &last, 1);
			if (debug) {
				if (t->len < 0x10)
					print_hex_dump_bytes("Tx ",
						DUMP_PREFIX_OFFSET,
						t->tx_buf, t->len);
				else
					pr_debug("Tx: %d bytes\n", t->len);
			}
		}
		if (t->rx_buf) {
			status = pio ?
			   stmp_spi_txrx_pio(ss, cs, t->rx_buf,
				   t->len, &first, &last, 0):
			   stmp_spi_txrx_dma(ss, cs, t->rx_buf,
				   t->rx_dma, t->len, &first, &last, 0);
			if (debug) {
				if (t->len < 0x10)
					print_hex_dump_bytes("Rx ",
						DUMP_PREFIX_OFFSET,
						t->rx_buf, t->len);
				else
					pr_debug("Rx: %d bytes\n", t->len);
			}
		}

		if (status)
			break;

		first = last = 0;

	}
	return status;
}

/**
 * stmp_spi_handle
 *
 * The workhorse of the driver - it handles messages from the list
 *
 **/
static void stmp_spi_handle(struct work_struct *w)
{
	struct stmp_spi *ss = container_of(w, struct stmp_spi, work);
	unsigned long flags;
	struct spi_message *m;

	BUG_ON(w == NULL);

	spin_lock_irqsave(&ss->lock, flags);
	while (!list_empty(&ss->queue)) {
		m = list_entry(ss->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&ss->lock, flags);

		m->status = stmp_spi_handle_message(ss, m);
		if (m->complete)
			m->complete(m->context);

		spin_lock_irqsave(&ss->lock, flags);
	}
	spin_unlock_irqrestore(&ss->lock, flags);

	return;
}

/**
 * stmp_spi_transfer
 *
 * Called indirectly from spi_async, queues all the messages to
 * spi_handle_message
 *
 * @spi: spi device
 * @m: message to be queued
**/
static int stmp_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct stmp_spi *ss = spi_master_get_devdata(spi->master);
	unsigned long flags;

	m->status = -EINPROGRESS;
	spin_lock_irqsave(&ss->lock, flags);
	list_add_tail(&m->queue, &ss->queue);
	queue_work(ss->workqueue, &ss->work);
	spin_unlock_irqrestore(&ss->lock, flags);
	return 0;
}

static irqreturn_t stmp_spi_irq(int irq, void *dev_id)
{
	struct stmp_spi *ss = dev_id;

	stmp3xxx_dma_clear_interrupt(ss->dma);
	complete(&ss->done);
	return IRQ_HANDLED;
}

static irqreturn_t stmp_spi_irq_err(int irq, void *dev_id)
{
	struct stmp_spi *ss = dev_id;
	u32 c1, st;

	c1 = __raw_readl(ss->regs + HW_SSP_CTRL1);
	st = __raw_readl(ss->regs + HW_SSP_STATUS);
	printk(KERN_ERR"IRQ - ERROR!, status = 0x%08X, c1 = 0x%08X\n", st, c1);
	stmp3xxx_clearl(c1 & 0xCCCC0000, ss->regs + HW_SSP_CTRL1);

	return IRQ_HANDLED;
}

static int __init stmp_spi_probe(struct platform_device *dev)
{
	int err = 0;
	struct spi_master *master;
	struct stmp_spi *ss;
	struct resource *r;
	u32 mem;

	/* Get resources(memory, IRQ) associated with the device */
	master = spi_alloc_master(&dev->dev, sizeof(struct stmp_spi));

	if (master == NULL) {
		err = -ENOMEM;
		goto out0;
	}

	platform_set_drvdata(dev, master);

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		err = -ENODEV;
		goto out_put_master;
	}

	ss = spi_master_get_devdata(master);
	ss->master_dev = &dev->dev;
	ss->id = dev->id;

	INIT_WORK(&ss->work, stmp_spi_handle);
	INIT_LIST_HEAD(&ss->queue);
	spin_lock_init(&ss->lock);
	ss->workqueue = create_singlethread_workqueue(dev_name(&dev->dev));
	master->transfer = stmp_spi_transfer;
	master->setup = stmp_spi_setup;
	master->cleanup = stmp_spi_cleanup;

	if (!request_mem_region(r->start,
			r->end - r->start + 1, dev_name(&dev->dev))) {
		err = -ENXIO;
		goto out_put_master;
	}
	mem = r->start;

	ss->regs = r->start - STMP3XXX_REGS_PHBASE + STMP3XXX_REGS_BASE;

	ss->irq = platform_get_irq(dev, 0);
	if (ss->irq < 0) {
		err = -ENXIO;
		goto out_put_master;
	}

	r = platform_get_resource(dev, IORESOURCE_DMA, 0);
	if (r == NULL) {
		err = -ENODEV;
		goto out_put_master;
	}

	ss->dma = r->start;
	err = stmp3xxx_dma_request(ss->dma, &dev->dev, (char *)dev_name(&dev->dev));
	if (err)
		goto out_put_master;

	err = stmp3xxx_dma_allocate_command(ss->dma, &ss->d);
	if (err)
		goto out_free_dma;

	master->bus_num = dev->id;
	master->num_chipselect = 1;

	/* SPI controller initializations */
	err = stmp_spi_init_hw(ss);
	if (err) {
		dev_dbg(&dev->dev, "cannot initialize hardware\n");
		goto out_free_dma_desc;
	}

	clk_set_rate(ss->clk, 120000);
	ss->speed_khz = clk_get_rate(ss->clk);
	ss->divider = 2;
	dev_info(&dev->dev, "Max possible speed %d = %ld/%d kHz\n",
		ss->speed_khz, clk_get_rate(ss->clk), ss->divider);

	/* Register for SPI Interrupt */
	err = request_irq(ss->irq, stmp_spi_irq, 0,
			dev_name(&dev->dev), ss);
	if (err) {
		dev_dbg(&dev->dev, "request_irq failed, %d\n", err);
		goto out_release_hw;
	}
	err = request_irq(IRQ_SSP_ERROR, stmp_spi_irq_err, IRQF_SHARED,
			dev_name(&dev->dev), ss);
	if (err) {
		dev_dbg(&dev->dev, "request_irq(error) failed, %d\n", err);
		goto out_free_irq;
	}

	err = spi_register_master(master);
	if (err) {
		dev_dbg(&dev->dev, "cannot register spi master, %d\n", err);
		goto out_free_irq_2;
	}
	dev_info(&dev->dev, "at 0x%08X mapped to 0x%08X, irq=%d, bus %d, %s\n",
			mem, (u32)ss->regs, ss->irq,
			master->bus_num, pio ? "PIO" : "DMA");
	return 0;

out_free_irq_2:
	free_irq(IRQ_SSP_ERROR, ss);
out_free_irq:
	free_irq(ss->irq, ss);
out_free_dma_desc:
	stmp3xxx_dma_free_command(ss->dma, &ss->d);
out_free_dma:
	stmp3xxx_dma_release(ss->dma);
out_release_hw:
	stmp_spi_release_hw(ss);
out_put_master:
	spi_master_put(master);
out0:
	return err;
}

static int __devexit stmp_spi_remove(struct platform_device *dev)
{
	struct stmp_spi *ss;
	struct spi_master *master;

	master = platform_get_drvdata(dev);
	if (master == NULL)
		goto out0;
	ss = spi_master_get_devdata(master);
	if (ss == NULL)
		goto out1;
	free_irq(ss->irq, ss);
	if (ss->workqueue)
		destroy_workqueue(ss->workqueue);
	stmp3xxx_dma_free_command(ss->dma, &ss->d);
	stmp3xxx_dma_release(ss->dma);
	stmp_spi_release_hw(ss);
	platform_set_drvdata(dev, 0);
out1:
	spi_master_put(master);
out0:
	return 0;
}

#ifdef CONFIG_PM
static int stmp_spi_suspend(struct platform_device *pdev, pm_message_t pmsg)
{
	struct stmp_spi *ss;
	struct spi_master *master;

	master = platform_get_drvdata(pdev);
	ss = spi_master_get_devdata(master);

	ss->saved_timings = __raw_readl(ss->regs + HW_SSP_TIMING);
	clk_disable(ss->clk);

	return 0;
}

static int stmp_spi_resume(struct platform_device *pdev)
{
	struct stmp_spi *ss;
	struct spi_master *master;

	master = platform_get_drvdata(pdev);
	ss = spi_master_get_devdata(master);

	clk_enable(ss->clk);
	stmp3xxx_clearl(BM_SSP_CTRL0_SFTRST | BM_SSP_CTRL0_CLKGATE, ss->regs + HW_SSP_CTRL0);
	stmp3xxx_setl(ss->saved_timings, ss->regs + HW_SSP_TIMING);

	return 0;
}

#else
#define stmp_spi_suspend NULL
#define stmp_spi_resume  NULL
#endif

static struct platform_driver stmp_spi_driver = {
	.probe	= stmp_spi_probe,
	.remove	= __devexit_p(stmp_spi_remove),
	.driver = {
		.name = "stmp37xx_ssp",
		.owner = THIS_MODULE,
	},
	.suspend = stmp_spi_suspend,
	.resume  = stmp_spi_resume,
};

static int __init stmp_spi_init(void)
{
	return platform_driver_register(&stmp_spi_driver);
}

static void __exit stmp_spi_exit(void)
{
	platform_driver_unregister(&stmp_spi_driver);
}

module_init(stmp_spi_init);
module_exit(stmp_spi_exit);
module_param(pio, int, S_IRUGO);
module_param(debug, int, S_IRUGO);
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_DESCRIPTION("STMP37xx SPI/SSP");
MODULE_LICENSE("GPL");
