/*
 * Portions copyright (C) 2003 Russell King, PXA MMCI Driver
 * Portions copyright (C) 2004-2005 Pierre Ossman, W83L51xD SD/MMC driver
 *
 * Copyright 2008 Embedded Alley Solutions, Inc.
 * Copyright 2009-2011 Freescale Semiconductor, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <mach/hardware.h>
#include <mach/dmaengine.h>
#include <mach/regs-ssp.h>
#include <mach/device.h>
#include <mach/system.h>

#define DRIVER_NAME	"mxs-mmc"

/*
 * Card detect polling timeout
 */
#define MXS_MMC_DETECT_TIMEOUT		(HZ/2)

/* Max value supported for XFER_COUNT */
#define SSP_BUFFER_SIZE		(65535)

#ifndef BF
#define BF(value, field) (((value) << BP_##field) & BM_##field)
#endif

#ifndef HW_SSP_XFER_SIZE
#define HW_SSP_XFER_SIZE	(0xFFFFFFFF)
#endif
#ifndef HW_SSP_BLOCK_SIZE
#define HW_SSP_BLOCK_SIZE	(0xFFFFFFFF)
#endif

#ifndef BP_SSP_XFER_SIZE_XFER_COUNT
#define BP_SSP_XFER_SIZE_XFER_COUNT BP_SSP_CTRL0_XFER_COUNT
#endif
#ifndef BM_SSP_XFER_SIZE_XFER_COUNT
#define BM_SSP_XFER_SIZE_XFER_COUNT BM_SSP_CTRL0_XFER_COUNT
#endif
#ifndef BF_SSP_XFER_SIZE_XFER_COUNT
#define BF_SSP_XFER_SIZE_XFER_COUNT(v)  \
		(((v) << 0) & BM_SSP_CTRL0_XFER_COUNT)
#endif

#ifndef BP_SSP_BLOCK_SIZE_BLOCK_COUNT
#define BP_SSP_BLOCK_SIZE_BLOCK_COUNT	8
#endif
#ifndef BM_SSP_BLOCK_SIZE_BLOCK_COUNT
#define BM_SSP_BLOCK_SIZE_BLOCK_COUNT	0x0000FF00
#endif
#ifndef BF_SSP_BLOCK_SIZE_BLOCK_COUNT
#define BF_SSP_BLOCK_SIZE_BLOCK_COUNT(v)  \
		(((v) << 8) & BM_SSP_BLOCK_SIZE_BLOCK_COUNT)
#endif
#ifndef BP_SSP_BLOCK_SIZE_BLOCK_SIZE
#define BP_SSP_BLOCK_SIZE_BLOCK_SIZE	16
#endif
#ifndef BM_SSP_BLOCK_SIZE_BLOCK_SIZE
#define BM_SSP_BLOCK_SIZE_BLOCK_SIZE	0x000F0000
#endif
#ifndef BF_SSP_BLOCK_SIZE_BLOCK_SIZE
#define BF_SSP_BLOCK_SIZE_BLOCK_SIZE(v)  \
		(((v) << 16) & BM_SSP_BLOCK_SIZE_BLOCK_SIZE)
#endif
#ifndef BM_SSP_CMD0_DBL_DATA_RATE_EN
#define BM_SSP_CMD0_DBL_DATA_RATE_EN	0x02000000
#endif

struct mxs_mmc_host {
	struct device *dev;
	struct mmc_host *mmc;

	struct clk *clk;
	unsigned int clkrt;

	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;

	/*  data bus width 0:1bit, 1:4bit, 2:8bit */
	unsigned char	bus_width;

	/* Whether SD card is present */
	unsigned present:1;

	/* Polling timer */
	struct timer_list timer;

	/* SSP interface which MMC/SD card slot is attached to */
	void __iomem *ssp_base;

	/* DMA channel used for this host */
	unsigned int dmach;

	/* IRQs */
	int dmairq, errirq;

	/* DMA descriptor to transfer data over SSP interface */
	dma_addr_t dma_desc_phys;
	struct mxs_dma_desc *dma_desc;

	/* DMA buffer */
	dma_addr_t dma_buf_phys;
	char *dma_buf;

	struct completion dma_done;
	/* status on last interrupt */
	u32 status;
	int read_uA, write_uA;
	struct regulator *regulator;	/*! Regulator */

	spinlock_t lock;
	int sdio_irq_en;
	int fastpath_sz;
};

/* Return read only state of card */
static int mxs_mmc_get_ro(struct mmc_host *mmc)
{
	struct mxs_mmc_host *host = mmc_priv(mmc);
	struct mxs_mmc_platform_data *mmc_data = host->dev->platform_data;

	if (mmc_data && mmc_data->get_wp)
		return mmc_data->get_wp();

	return 0;
}

/* Detect if card is plugged */
static inline int mxs_mmc_is_plugged(struct mxs_mmc_host *host)
{
	u32 status = __raw_readl(host->ssp_base + HW_SSP_STATUS);
	return !(status & BM_SSP_STATUS_CARD_DETECT);
}

static void mxs_mmc_reset(struct mxs_mmc_host *host);
/* Card detection polling function */
static void mxs_mmc_detect_poll(unsigned long arg)
{
	struct mxs_mmc_host *host = (struct mxs_mmc_host *)arg;
	int card_status;

	card_status = mxs_mmc_is_plugged(host);
	if (card_status != host->present) {
		/* Reset MMC block */
		mxs_mmc_reset(host);
		host->present = card_status;
		mmc_detect_change(host->mmc, 0);
	}

	mod_timer(&host->timer, jiffies + MXS_MMC_DETECT_TIMEOUT);
}

#define MXS_MMC_ERR_IRQ_BITS  (BM_SSP_CTRL1_RESP_ERR_IRQ	| \
				BM_SSP_CTRL1_RESP_TIMEOUT_IRQ	| \
				BM_SSP_CTRL1_DATA_TIMEOUT_IRQ	| \
				BM_SSP_CTRL1_DATA_CRC_IRQ	| \
				BM_SSP_CTRL1_FIFO_UNDERRUN_IRQ	| \
				BM_SSP_CTRL1_RECV_TIMEOUT_IRQ   | \
				BM_SSP_CTRL1_FIFO_OVERRUN_IRQ)

#define MXS_MMC_IRQ_BITS  (BM_SSP_CTRL1_SDIO_IRQ		| \
				MXS_MMC_ERR_IRQ_BITS)

#define MXS_MMC_ERR_BITS (BM_SSP_CTRL1_RESP_ERR_IRQ       | \
				BM_SSP_CTRL1_RESP_TIMEOUT_IRQ   | \
				BM_SSP_CTRL1_DATA_TIMEOUT_IRQ   | \
				BM_SSP_CTRL1_DATA_CRC_IRQ       | \
				BM_SSP_CTRL1_RECV_TIMEOUT_IRQ)

/* SSP DMA interrupt handler */
static irqreturn_t mmc_irq_handler(int irq, void *dev_id)
{
	struct mxs_mmc_host *host = dev_id;
	u32 c1;
	LIST_HEAD(list);

	c1 = __raw_readl(host->ssp_base + HW_SSP_CTRL1);
	if (irq == host->dmairq)
		__raw_writel(c1 & MXS_MMC_ERR_IRQ_BITS,
				host->ssp_base + HW_SSP_CTRL1_CLR);
	else
		__raw_writel(c1 & MXS_MMC_IRQ_BITS,
				host->ssp_base + HW_SSP_CTRL1_CLR);
	if (irq == host->dmairq) {
		dev_dbg(host->dev, "dma irq 0x%x and stop DMA.\n", irq);
		mxs_dma_ack_irq(host->dmach);
		/* STOP the dma transfer here. */
		mxs_dma_cooked(host->dmach, &list);
	}

	if ((irq == host->dmairq) || (c1 & MXS_MMC_ERR_BITS))
		if (host->cmd) {
			host->status =
				__raw_readl(host->ssp_base + HW_SSP_STATUS);
			complete(&host->dma_done);
		}

	if ((c1 & BM_SSP_CTRL1_SDIO_IRQ) && (c1 & BM_SSP_CTRL1_SDIO_IRQ_EN))
		if (irq == host->errirq)
			mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}

/*
 * Check for MMC command errors
 * Returns error code or zerro if no errors
 */
static inline int mxs_mmc_cmd_error(u32 status)
{
	int err = 0;

	if (status & BM_SSP_STATUS_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_TIMEOUT)
		err = -ETIMEDOUT;
	else if (status & BM_SSP_STATUS_RESP_CRC_ERR)
		err = -EILSEQ;
	else if (status & BM_SSP_STATUS_RESP_ERR)
		err = -EIO;

	return err;
}

static void mxs_mmc_bc(struct mxs_mmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;
	u32 ssp_ctrl1;
	u32 c1;

	ssp_ctrl0 = BM_SSP_CTRL0_ENABLE |
	    BM_SSP_CTRL0_IGNORE_CRC;
	ssp_cmd0 = BF(cmd->opcode, SSP_CMD0_CMD) |
	    BM_SSP_CMD0_APPEND_8CYC;
	ssp_cmd1 = BF(cmd->arg, SSP_CMD1_CMD_ARG);
	ssp_ctrl1 =
	    BM_SSP_CTRL1_DMA_ENABLE |
	    BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_DATA_CRC_IRQ_EN |
	    BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_ERR_IRQ_EN;

	if (host->sdio_irq_en) {
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		ssp_cmd0 |= BM_SSP_CMD0_CONT_CLKING_EN \
			| BM_SSP_CMD0_SLOW_CLKING_EN;
	}

	/* following IO operations */
	__raw_writel(ssp_ctrl0, host->ssp_base + HW_SSP_CTRL0);
	__raw_writel(ssp_cmd0, host->ssp_base + HW_SSP_CMD0);
	__raw_writel(ssp_cmd1, host->ssp_base + HW_SSP_CMD1);
	/* clear these bits */
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_CLR);
	init_completion(&host->dma_done);
	__raw_writel(BM_SSP_CTRL0_RUN, host->ssp_base + HW_SSP_CTRL0_SET);

	while (__raw_readl(host->ssp_base + HW_SSP_CTRL0)
		       & BM_SSP_CTRL0_RUN)
		continue;
	while (__raw_readl(host->ssp_base + HW_SSP_STATUS)
		       & BM_SSP_STATUS_BUSY)
		continue;
	host->status =
		__raw_readl(host->ssp_base + HW_SSP_STATUS);
	c1 = __raw_readl(host->ssp_base + HW_SSP_CTRL1);
	__raw_writel(c1 & MXS_MMC_ERR_IRQ_BITS,
			host->ssp_base + HW_SSP_CTRL1_CLR);
	/* reenable these bits */
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_SET);
	/* end IO operations */

	cmd->error = mxs_mmc_cmd_error(host->status);

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
	}
}

/* Send the ac command to the device */
static void mxs_mmc_ac(struct mxs_mmc_host *host)
{
	struct mmc_command *cmd = host->cmd;
	u32 ignore_crc, resp, long_resp;
	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;
	u32 ssp_ctrl1;
	u32 c1;

	ignore_crc = (mmc_resp_type(cmd) & MMC_RSP_CRC) ?
	    0 : BM_SSP_CTRL0_IGNORE_CRC;
	resp = (mmc_resp_type(cmd) & MMC_RSP_PRESENT) ?
	    BM_SSP_CTRL0_GET_RESP : 0;
	long_resp = (mmc_resp_type(cmd) & MMC_RSP_136) ?
	    BM_SSP_CTRL0_LONG_RESP : 0;

	ssp_ctrl0 = BM_SSP_CTRL0_ENABLE | ignore_crc | long_resp | resp;
	ssp_cmd0 = BF(cmd->opcode, SSP_CMD0_CMD);
	ssp_cmd1 = BF(cmd->arg, SSP_CMD1_CMD_ARG);
	ssp_ctrl1 =
		BM_SSP_CTRL1_DMA_ENABLE |
		BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN |
		BM_SSP_CTRL1_DATA_CRC_IRQ_EN |
		BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN |
		BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN |
		BM_SSP_CTRL1_RESP_ERR_IRQ_EN;

	if (host->sdio_irq_en) {
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		ssp_cmd0 |= BM_SSP_CMD0_CONT_CLKING_EN \
			| BM_SSP_CMD0_SLOW_CLKING_EN;
	}

	/* following IO operations */
	__raw_writel(ssp_ctrl0, host->ssp_base + HW_SSP_CTRL0);
	__raw_writel(ssp_cmd0, host->ssp_base + HW_SSP_CMD0);
	__raw_writel(ssp_cmd1, host->ssp_base + HW_SSP_CMD1);
	/* clear these bits */
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_CLR);
	init_completion(&host->dma_done);
	__raw_writel(BM_SSP_CTRL0_RUN, host->ssp_base + HW_SSP_CTRL0_SET);

	while (__raw_readl(host->ssp_base + HW_SSP_CTRL0)
		       & BM_SSP_CTRL0_RUN)
		continue;
	while (__raw_readl(host->ssp_base + HW_SSP_STATUS)
		       & BM_SSP_STATUS_BUSY)
		continue;
	host->status =
		__raw_readl(host->ssp_base + HW_SSP_STATUS);
	c1 = __raw_readl(host->ssp_base + HW_SSP_CTRL1);
	__raw_writel(c1 & MXS_MMC_ERR_IRQ_BITS,
			host->ssp_base + HW_SSP_CTRL1_CLR);
	/* reenable these bits */
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_SET);
	/* end IO operations */

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		while (__raw_readl(host->ssp_base + HW_SSP_CTRL0)
		       & BM_SSP_CTRL0_RUN)
			continue;
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1B:
	case MMC_RSP_R3:
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		break;
	case MMC_RSP_R2:
		cmd->resp[3] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		cmd->resp[2] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP1);
		cmd->resp[1] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP2);
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP3);
		break;
	default:
		dev_warn(host->dev, "Unsupported response type 0x%x\n",
			 mmc_resp_type(cmd));
		BUG();
		break;
	}

	cmd->error = mxs_mmc_cmd_error(host->status);

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
	}
}

/* Copy data between sg list and dma buffer */
static unsigned int mxs_sg_dma_copy(struct mxs_mmc_host *host,
					 unsigned int size, int to_dma)
{
	struct mmc_data *data = host->cmd->data;
	unsigned int copy_size, bytes_copied = 0;
	struct scatterlist *sg;
	char *dmabuf = host->dma_buf;
	char *sgbuf;
	int len, i;

	sg = data->sg;
	len = data->sg_len;

	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */
	for (i = 0; i < len; i++) {
		sgbuf = kmap_atomic(sg_page(&sg[i]), KM_BIO_SRC_IRQ) +
		    sg[i].offset;
		copy_size = size < sg[i].length ? size : sg[i].length;
		if (to_dma)
			memcpy(dmabuf, sgbuf, copy_size);
		else
			memcpy(sgbuf, dmabuf, copy_size);
		kunmap_atomic(sgbuf, KM_BIO_SRC_IRQ);

		dmabuf += sg[i].length;

		bytes_copied += copy_size;
		size -= copy_size;

		if (size == 0)
			break;
	}

	return bytes_copied;
}

/* Convert ns to tick count according to the current sclk speed */
static unsigned short mxs_ns_to_ssp_ticks(unsigned clock_rate, unsigned ns)
{
	const unsigned int ssp_timeout_mul = 4096;
	/*
	 * Calculate ticks in ms since ns are large numbers
	 * and might overflow
	 */
	const unsigned int clock_per_ms = clock_rate / 1000;
	const unsigned int ms = ns / 1000;
	const unsigned int ticks = ms * clock_per_ms;
	const unsigned int ssp_ticks = ticks / ssp_timeout_mul;

	BUG_ON(ssp_ticks == 0);
	return ssp_ticks;
}

static void __init_reg(struct device *dev, struct regulator **pp_reg)
{
#if 0
	/* Up to now, there is not pwr ctrl. Just keep it for future usage. */
	struct regulator *reg = *pp_reg;

	if (!reg) {
		reg = regulator_get(NULL, "mmc_ssp-1");
		if (reg && !IS_ERR(reg))
			regulator_set_mode(reg, REGULATOR_MODE_NORMAL);
		else
			reg = NULL;
		*pp_reg = reg;
	}
#endif
}

static int mxs_mmc_map_sg(struct mxs_mmc_host *host, int is_reading)
{
	struct mxs_dma_desc *dma_desc = host->dma_desc;
	struct mmc_data *data = host->cmd->data;
	struct scatterlist *sg;
	unsigned int sg_len;
	int count = 0;

	BUG_ON(!data);

	sg_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, is_reading ? \
		DMA_FROM_DEVICE : DMA_TO_DEVICE);
	for_each_sg(data->sg, sg, sg_len, count) {
		if (count == PAGE_SIZE / sizeof(*dma_desc)) {
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, is_reading ? \
				DMA_FROM_DEVICE : DMA_TO_DEVICE);
			return 0;
		}

		dma_desc[count].cmd.next = 0;
		dma_desc[count].cmd.cmd.data = 0;

		dma_desc[count].cmd.address = (u32)sg->dma_address;
		dma_desc[count].address = host->dma_desc_phys + \
					sizeof(*dma_desc) * count;
		dma_desc[count].cmd.cmd.bits.command = is_reading ? \
						DMA_WRITE : DMA_READ;
		dma_desc[count].cmd.cmd.bits.halt_on_terminate = 1;

		if (count == 0)
			dma_desc[count].cmd.cmd.bits.pio_words = 3;

		dma_desc[count].cmd.cmd.bits.bytes = sg->length;
		dma_desc[count].cmd.cmd.bits.dec_sem = 1;
	}
	dma_desc[count - 1].cmd.cmd.bits.irq = 1;
	dma_desc[count - 1].cmd.cmd.bits.wait4end = 1;

	return count;
}

static void mxs_mmc_unmap_sg(struct mxs_mmc_host *host, int is_reading)
{
	struct mmc_data *data = host->cmd->data;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		is_reading ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
}

/* Send adtc command to the card */
static void mxs_mmc_adtc(struct mxs_mmc_host *host, int polling_mode)
{
	struct mmc_command *cmd = host->cmd;
	struct mxs_dma_desc *dma_desc = host->dma_desc;
	int ignore_crc, resp, long_resp;
	int is_reading = 0;
	unsigned int copy_size;
	unsigned int ssp_ver_major;
	int desc_count = 0;
	u32 ssp_ctrl0;
	u32 ssp_cmd0;
	u32 ssp_cmd1;
	u32 timeout;
	u32 val;

	u32 data_size = cmd->data->blksz * cmd->data->blocks;
	u32 log2_block_size;
	u32 transfer_size = data_size;

	ignore_crc = mmc_resp_type(cmd) & MMC_RSP_CRC ? 0 : 1;
	resp = mmc_resp_type(cmd) & MMC_RSP_PRESENT ? 1 : 0;
	long_resp = mmc_resp_type(cmd) & MMC_RSP_136 ? 1 : 0;

	dev_dbg(host->dev, "ADTC command:\n"
		"response: %d, ignore crc: %d\n"
		"data list: %u, blocksz: %u, blocks: %u, timeout: %uns %uclks, "
		"flags: 0x%x\n", resp, ignore_crc, cmd->data->sg_len,
		cmd->data->blksz, cmd->data->blocks, cmd->data->timeout_ns,
		cmd->data->timeout_clks, cmd->data->flags);

	if (cmd->data->flags & MMC_DATA_WRITE) {
		dev_dbg(host->dev, "Data Write\n");
		is_reading = 0;
		desc_count = mxs_mmc_map_sg(host, is_reading);

		if (desc_count == 0) {
			host->dma_desc->cmd.address = (u32) host->dma_buf_phys;
			if (!polling_mode)
				copy_size = mxs_sg_dma_copy(host, data_size, 1);
			BUG_ON(copy_size < data_size);
		}
		if (!host->regulator)
			__init_reg(host->dev, &host->regulator);
		if (host->regulator)
			regulator_set_current_limit(host->regulator,
						    host->write_uA,
						    host->write_uA);
	} else if (cmd->data->flags & MMC_DATA_READ) {
		dev_dbg(host->dev, "Data Read\n");
		is_reading = 1;
		desc_count = mxs_mmc_map_sg(host, is_reading);
		if (desc_count == 0)
			host->dma_desc->cmd.address = (u32) host->dma_buf_phys;
		if (!host->regulator)
			__init_reg(host->dev, &host->regulator);
		if (host->regulator)
			regulator_set_current_limit(host->regulator,
						    host->read_uA,
						    host->read_uA);
	} else {
		dev_warn(host->dev, "Unsuspported data mode, 0x%x\n",
			 cmd->data->flags);
		BUG();
	}

	BUG_ON(cmd->data->flags & MMC_DATA_STREAM);
	/* BUG_ON((data_size % 8) > 0); */

	if (polling_mode)
		goto skip_dma_setup1;

	/* when is_reading is set, DMA controller performs WRITE operation. */
	if (desc_count == 0) {
		dma_desc->cmd.cmd.bits.command = is_reading ? \
						DMA_WRITE : DMA_READ;
		dma_desc->cmd.cmd.bits.irq = 1;
		dma_desc->cmd.cmd.bits.dec_sem = 1;
		dma_desc->cmd.cmd.bits.wait4end = 1;
		dma_desc->cmd.cmd.bits.pio_words = 3;
		dma_desc->cmd.cmd.bits.bytes = data_size;
	}

skip_dma_setup1:
	ssp_ver_major = __raw_readl(host->ssp_base + HW_SSP_VERSION) >> 24;
	dev_dbg(host->dev, "ssp ver major is 0x%x\n", ssp_ver_major);
	if (ssp_ver_major > 3) {
		__raw_writel(data_size, host->ssp_base + HW_SSP_XFER_SIZE);
		ssp_ctrl0 = (ignore_crc ? BM_SSP_CTRL0_IGNORE_CRC : 0) |
			(resp ? BM_SSP_CTRL0_GET_RESP : 0) |
			(long_resp ? BM_SSP_CTRL0_LONG_RESP : 0) |
			(is_reading ? BM_SSP_CTRL0_READ : 0) |
			BM_SSP_CTRL0_DATA_XFER | BM_SSP_CTRL0_WAIT_FOR_IRQ |
			BM_SSP_CTRL0_ENABLE;
		if (host->bus_width == 2)
			ssp_ctrl0 |= BF(BV_SSP_CTRL0_BUS_WIDTH__EIGHT_BIT,
					     SSP_CTRL0_BUS_WIDTH);
		else if (host->bus_width == 1)
			ssp_ctrl0 |= BF(BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT,
					     SSP_CTRL0_BUS_WIDTH);
		else
			ssp_ctrl0 |= BF(BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT,
					     SSP_CTRL0_BUS_WIDTH);
	} else
		ssp_ctrl0 = (ignore_crc ? BM_SSP_CTRL0_IGNORE_CRC : 0) |
			(resp ? BM_SSP_CTRL0_GET_RESP : 0) |
			(long_resp ? BM_SSP_CTRL0_LONG_RESP : 0) |
			(is_reading ? BM_SSP_CTRL0_READ : 0) |
			BM_SSP_CTRL0_DATA_XFER | BM_SSP_CTRL0_WAIT_FOR_IRQ |
			BM_SSP_CTRL0_ENABLE |
			BF(data_size, SSP_XFER_SIZE_XFER_COUNT) |
			BF(host->bus_width ?
			BV_SSP_CTRL0_BUS_WIDTH__FOUR_BIT :
					     BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT,
					     SSP_CTRL0_BUS_WIDTH);

	/*
	 * We need to set the hardware register to the logarithm to base 2 of
	 * the block size.
	 */
	log2_block_size = ilog2(cmd->data->blksz);
	dev_dbg(host->dev, "%s blksz is 0x%x.\n", __func__, log2_block_size);

	if (ssp_ver_major > 3) {
		/* Configure the CMD0 */
		ssp_cmd0 = BF(cmd->opcode, SSP_CMD0_CMD);

		/* Configure the BLOCK SIZE and BLOCK COUNT */
		if ((1<<log2_block_size) != cmd->data->blksz) {
			BUG_ON(cmd->data->blocks > 1);
			__raw_writel(0, host->ssp_base + HW_SSP_BLOCK_SIZE);
		} else{
			val = BF(log2_block_size, SSP_BLOCK_SIZE_BLOCK_SIZE) |
			BF(cmd->data->blocks - 1, SSP_BLOCK_SIZE_BLOCK_COUNT);
			__raw_writel(val, host->ssp_base + HW_SSP_BLOCK_SIZE);
			if (host->mmc->ios.bus_width & MMC_BUS_WIDTH_DDR) {
				/*
				 * Enable the DDR mode
				 * Make sure the POLARITY bit is cleared
				 */
				__raw_writel(BM_SSP_CTRL1_POLARITY, \
					host->ssp_base + HW_SSP_CTRL1_CLR);
				ssp_cmd0 |= BM_SSP_CMD0_DBL_DATA_RATE_EN;
			} else {
				__raw_writel(BM_SSP_CTRL1_POLARITY, \
					host->ssp_base + HW_SSP_CTRL1_SET);
				ssp_cmd0 &= ~BM_SSP_CMD0_DBL_DATA_RATE_EN;
			}

		}
	} else {
		if ((1<<log2_block_size) != cmd->data->blksz) {
			BUG_ON(cmd->data->blocks > 1);
			ssp_cmd0 =
				BF(0, SSP_BLOCK_SIZE_BLOCK_SIZE) |
				BF(cmd->opcode, SSP_CMD0_CMD) |
				BF(0, SSP_BLOCK_SIZE_BLOCK_COUNT);
		} else
			ssp_cmd0 =
			BF(log2_block_size, SSP_BLOCK_SIZE_BLOCK_SIZE) |
			BF(cmd->opcode, SSP_CMD0_CMD) |
			BF(cmd->data->blocks - 1, SSP_BLOCK_SIZE_BLOCK_COUNT);
	}
	if (host->sdio_irq_en) {
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
		ssp_cmd0 |= BM_SSP_CMD0_CONT_CLKING_EN \
			| BM_SSP_CMD0_SLOW_CLKING_EN;
	}
	if ((cmd->opcode == 12) || (cmd->opcode == 53))
		ssp_cmd0 |= BM_SSP_CMD0_APPEND_8CYC;

	ssp_cmd1 = BF(cmd->arg, SSP_CMD1_CMD_ARG);

	if (!polling_mode) {
		dma_desc[0].cmd.pio_words[0] = ssp_ctrl0;
		dma_desc[0].cmd.pio_words[1] = ssp_cmd0;
		dma_desc[0].cmd.pio_words[2] = ssp_cmd1;

		/* Set the timeout count */
		timeout = mxs_ns_to_ssp_ticks(host->clkrt, \
			cmd->data->timeout_ns);
		val = __raw_readl(host->ssp_base + HW_SSP_TIMING);
		val &= ~(BM_SSP_TIMING_TIMEOUT);
		val |= BF(timeout, SSP_TIMING_TIMEOUT);
		__raw_writel(val, host->ssp_base + HW_SSP_TIMING);
		init_completion(&host->dma_done);
		mxs_dma_reset(host->dmach);
		if (desc_count != 0) {
			int i = 0;

			while (i != desc_count) {
				if (mxs_dma_desc_append(host->dmach, &host->dma_desc[i]) < 0)
					dev_err(host->dev, "desc_append failed\n");
				i++;
			}
		} else
			if (mxs_dma_desc_append(host->dmach, host->dma_desc) < 0)
				dev_err(host->dev, "desc_append failed\n");
		dev_dbg(host->dev, "%s start DMA.\n", __func__);
		if (mxs_dma_enable(host->dmach) < 0)
			dev_err(host->dev, "mmc_dma_enable failed\n");
		wait_for_completion(&host->dma_done);
		if (desc_count != 0)
			mxs_mmc_unmap_sg(host, is_reading);
	} else {
		int index = 0;
		int len;
		struct scatterlist *sg;
		int size;
		char *sgbuf;
		u8 *p;
		u32 data, status;
		u32 ssp_status = (u32)host->ssp_base + HW_SSP_STATUS;
		u32 ssp_data = (u32)host->ssp_base + HW_SSP_DATA;
		u32 ssp_ctl0 = (u32)host->ssp_base + HW_SSP_CTRL0;
		u32 ssp_ctrl1;
		u32 c1;

		sg = host->cmd->data->sg;
		len = host->cmd->data->sg_len;

		ssp_ctrl1 =
			BM_SSP_CTRL1_DMA_ENABLE |
			BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN |
			BM_SSP_CTRL1_DATA_CRC_IRQ_EN |
			BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN |
			BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN |
			BM_SSP_CTRL1_RESP_ERR_IRQ_EN;
		__raw_writel(ssp_ctrl0, host->ssp_base + HW_SSP_CTRL0);
		__raw_writel(ssp_cmd0, host->ssp_base + HW_SSP_CMD0);
		__raw_writel(ssp_cmd1, host->ssp_base + HW_SSP_CMD1);
		/* clear these bits */
		__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_CLR);
		init_completion(&host->dma_done);
		__raw_writel(BM_SSP_CTRL0_RUN, ssp_ctl0 + 4);

		while (__raw_readl(host->ssp_base + HW_SSP_CTRL0) & \
			BM_SSP_STATUS_CMD_BUSY)
			continue;

		while (transfer_size) {
			sgbuf = kmap_atomic(sg_page(&sg[index]), \
				KM_BIO_SRC_IRQ) + sg[index].offset;

			p = (u8 *)sgbuf;
			size = transfer_size < sg[index].length ? \
				transfer_size : sg[index].length;

			if (is_reading) {
				while (size) {
					status = __raw_readl(ssp_status);
					if (status & BM_SSP_STATUS_FIFO_EMPTY)
						continue;
					data = __raw_readl(ssp_data);
					if ((u32)p & 0x3) {
						*p++ = data & 0xff;
						*p++ = (data >> 8) & 0xff;
						*p++ = (data >> 16) & 0xff;
						*p++ = (data >> 24) & 0xff;
					} else {
						*(u32 *)p = data;
						p += 4;
					}
					transfer_size -= 4;
					size -= 4;
				}
			} else {
				while (size) {
					status = __raw_readl(ssp_status);
					if (status & BM_SSP_STATUS_FIFO_FULL)
						continue;
					if ((u32)p & 0x3)
						data = p[0] | \
							(p[1] << 8) | \
							(p[2] << 16) | \
							(p[3] << 24);
					else
						data = *(u32 *)p;

					__raw_writel(data, ssp_data);
					transfer_size -= 4;
					size -= 4;
					p += 4;
				}
			}
			kunmap_atomic(sgbuf, KM_BIO_SRC_IRQ);
			index++;
		}
		status = BM_SSP_STATUS_BUSY | \
			BM_SSP_STATUS_DATA_BUSY | \
			BM_SSP_STATUS_CMD_BUSY;
		while (__raw_readl(host->ssp_base + HW_SSP_STATUS) & status)
			continue;

		cmd->data->bytes_xfered = data_size;

		host->status =
			__raw_readl(host->ssp_base + HW_SSP_STATUS);
		c1 = __raw_readl(host->ssp_base + HW_SSP_CTRL1);
		__raw_writel(c1 & MXS_MMC_ERR_IRQ_BITS,
			host->ssp_base + HW_SSP_CTRL1_CLR);
		/* reenable these bits */
		__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1_SET);
	}
	if (host->regulator)
		regulator_set_current_limit(host->regulator, 0, 0);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R3:
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		break;
	case MMC_RSP_R2:
		cmd->resp[3] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP0);
		cmd->resp[2] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP1);
		cmd->resp[1] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP2);
		cmd->resp[0] =
		    __raw_readl(host->ssp_base + HW_SSP_SDRESP3);
		break;
	default:
		dev_warn(host->dev, "Unsupported response type 0x%x\n",
			 mmc_resp_type(cmd));
		BUG();
		break;
	}

	cmd->error = mxs_mmc_cmd_error(host->status);

	if (cmd->error) {
		dev_dbg(host->dev, "Command error 0x%x\n", cmd->error);
		if (polling_mode)
			return;
		mxs_dma_reset(host->dmach);
	} else {
		if (polling_mode)
			return;
		if (!desc_count && is_reading) {
			cmd->data->bytes_xfered =
			    mxs_sg_dma_copy(host, data_size, 0);
		} else
			cmd->data->bytes_xfered = data_size;

		dev_dbg(host->dev, "Transferred %u bytes\n",
			cmd->data->bytes_xfered);
	}
	mxs_dma_disable(host->dmach);
}

/* Begin sedning a command to the card */
static void mxs_mmc_start_cmd(struct mxs_mmc_host *host,
				   struct mmc_command *cmd)
{
	int data_size = 0;

	dev_dbg(host->dev, "MMC command:\n"
		"type: 0x%x opcode: %u, arg: %u, flags 0x%x retries: %u\n",
		mmc_cmd_type(cmd), cmd->opcode, cmd->arg, cmd->flags,
		cmd->retries);

	host->cmd = cmd;

	switch (mmc_cmd_type(cmd)) {
	case MMC_CMD_BC:
		mxs_mmc_bc(host);
		break;
	case MMC_CMD_BCR:
		mxs_mmc_ac(host);
		break;
	case MMC_CMD_AC:
		mxs_mmc_ac(host);
		break;
	case MMC_CMD_ADTC:
		data_size = host->cmd->data->blksz * host->cmd->data->blocks;
		mxs_mmc_adtc(host, data_size < host->fastpath_sz);
		break;
	default:
		dev_warn(host->dev, "Unknown MMC command\n");
		BUG();
		break;
	}

	dev_dbg(host->dev, "response: %u %u %u %u errors: %u\n",
		cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
		cmd->error);
}

/* Handle MMC request */
static void mxs_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mxs_mmc_host *host = mmc_priv(mmc);

	dev_dbg(host->dev, "MMC request\n");
	BUG_ON(host->mrq != NULL);
	host->mrq = mrq;

	mxs_mmc_start_cmd(host, mrq->cmd);

	if (mrq->data && mrq->data->stop) {
		dev_dbg(host->dev, "Stop opcode is %u\n",
			mrq->data->stop->opcode);
		mxs_mmc_start_cmd(host, mrq->data->stop);
	}

	host->mrq = NULL;
	mmc_request_done(mmc, mrq);
}

/*
 * Change divisors to reflect the rate of 'hz'. Note that we should not
 * play with clock rate, because the same source is used to clock both
 * SSP ports.
 */
static void
mxs_set_sclk_speed(struct mxs_mmc_host *host, unsigned int hz)
{
	unsigned long ssp, bus_clk = 0;
	u32 div1, div2;
	u32 val;
	struct mxs_mmc_platform_data *mmc_data = host->dev->platform_data;

	if (mmc_data && mmc_data->setclock) {
		/* using SSP1, no timeout, clock rate 1 */
		__raw_writel(BF(0xFFFF, SSP_TIMING_TIMEOUT) |
			     BF(2, SSP_TIMING_CLOCK_DIVIDE) |
			     BF(0, SSP_TIMING_CLOCK_RATE),
			     host->ssp_base + HW_SSP_TIMING);

		/*
		   if the SSP is buggy and platform provides callback...
		   well, let it be.
		 */
		host->clkrt = mmc_data->setclock(hz);
		dev_dbg(host->dev, "Setting clock rate to %d Hz"
			"(requested %d)\n",
			host->clkrt, hz);
		dev_dbg(host->dev, "source %ldk\n",
			clk_get_rate(host->clk));

		return;
	}

	/*
	   ...but the RightIdea(tm) is to set divisors to match
	   the requested clock.
	 */
	ssp = clk_get_rate(host->clk);

	for (div1 = 2; div1 <= 254; div1 += 2) {
		div2 = ssp / hz / div1;
		if (div2 <= 256)
			break;
	}
	if (div1 > 254) {
		dev_err(host->dev, "Cannot set clock to %dHz\n", hz);
		return;
	}

	if (div2 == 0)
		div2 = 1;

	bus_clk = ssp / div1 / div2;

	dev_dbg(host->dev, "Setting clock rate to %ld Hz [%x+%x] "
		"(requested %d), source %ldk\n",
		bus_clk, div1, div2, hz, ssp);

	val = __raw_readl(host->ssp_base + HW_SSP_TIMING);
	val &= ~(BM_SSP_TIMING_CLOCK_DIVIDE | BM_SSP_TIMING_CLOCK_RATE);
	val |= BF(div1, SSP_TIMING_CLOCK_DIVIDE) |
		      BF(div2 - 1, SSP_TIMING_CLOCK_RATE);
	__raw_writel(val, host->ssp_base + HW_SSP_TIMING);

	host->clkrt = bus_clk;
}

/* Configure card */
static void mxs_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mxs_mmc_host *host = mmc_priv(mmc);
	struct mxs_mmc_platform_data *mmc_data;

	dev_dbg(host->dev, "MMC set ios:\n"
		"Clock %u, vdd %u, bus_mode %u, chip_select %u, "
		"power mode %u, bus_width %u\n", ios->clock, ios->vdd,
		ios->bus_mode, ios->chip_select, ios->power_mode,
		ios->bus_width);

	mmc_data = host->dev->platform_data;

	if (mmc_data->cmd_pullup) {
		if (ios->bus_mode == MMC_BUSMODE_PUSHPULL)
			mmc_data->cmd_pullup(0);
		else
			mmc_data->cmd_pullup(1);
	} else
		dev_warn(host->dev,
			 "Platform does not support CMD pin pullup control\n");

	if ((ios->bus_width & ~MMC_BUS_WIDTH_DDR) == MMC_BUS_WIDTH_8)
		host->bus_width = 2;
	else if ((ios->bus_width & ~MMC_BUS_WIDTH_DDR) == MMC_BUS_WIDTH_4)
		host->bus_width = 1;
	else
		host->bus_width = 0;
	dev_dbg(host->dev, "MMC bus_width %u\n", host->bus_width);

	if (ios->clock > 0)
		mxs_set_sclk_speed(host, ios->clock);
}

static void mxs_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	unsigned long flags;
	struct mxs_mmc_host *host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	if (enable) {
		if (host->sdio_irq_en)
			goto exit;
		host->sdio_irq_en = 1;
		__raw_writel(BM_SSP_CTRL0_SDIO_IRQ_CHECK, host->ssp_base + \
			HW_SSP_CTRL0_SET);
		__raw_writel(BM_SSP_CTRL1_SDIO_IRQ_EN, host->ssp_base + \
			HW_SSP_CTRL1_SET);

		if (__raw_readl(host->ssp_base + \
		HW_SSP_STATUS) & BM_SSP_STATUS_SDIO_IRQ)
			mmc_signal_sdio_irq(host->mmc);

	} else {
		if (host->sdio_irq_en == 0)
			goto exit;
		host->sdio_irq_en = 0;
		__raw_writel(BM_SSP_CTRL0_SDIO_IRQ_CHECK, \
			host->ssp_base + HW_SSP_CTRL0_CLR);
		__raw_writel(BM_SSP_CTRL1_SDIO_IRQ_EN, \
			host->ssp_base + HW_SSP_CTRL1_CLR);
	}

exit:
	spin_unlock_irqrestore(&host->lock, flags);
	return;
}

static const struct mmc_host_ops mxs_mmc_ops = {
	.request = mxs_mmc_request,
	.get_ro = mxs_mmc_get_ro,
	.set_ios = mxs_mmc_set_ios,
	.enable_sdio_irq = mxs_mmc_enable_sdio_irq,
};

/*
 * MXS MMC/SD driver initialization
 */

/* Reset ssp peripheral to default values */
static void mxs_mmc_reset(struct mxs_mmc_host *host)
{
	u32 ssp_ctrl0;
	u32 ssp_ctrl1;

	mxs_reset_block(host->ssp_base, 0);

	/* Configure SSP Control Register 0 */
	ssp_ctrl0 =
	    BM_SSP_CTRL0_IGNORE_CRC |
	    BF(BV_SSP_CTRL0_BUS_WIDTH__ONE_BIT, SSP_CTRL0_BUS_WIDTH);

	/* Configure SSP Control Register 1 */
	ssp_ctrl1 =
	    BM_SSP_CTRL1_DMA_ENABLE |
	    BM_SSP_CTRL1_POLARITY |
	    BM_SSP_CTRL1_RECV_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_DATA_CRC_IRQ_EN |
	    BM_SSP_CTRL1_DATA_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_TIMEOUT_IRQ_EN |
	    BM_SSP_CTRL1_RESP_ERR_IRQ_EN |
	    BF(BV_SSP_CTRL1_WORD_LENGTH__EIGHT_BITS, SSP_CTRL1_WORD_LENGTH) |
	    BF(BV_SSP_CTRL1_SSP_MODE__SD_MMC, SSP_CTRL1_SSP_MODE);

	__raw_writel(BF(0xFFFF, SSP_TIMING_TIMEOUT) |
		     BF(2, SSP_TIMING_CLOCK_DIVIDE) |
		     BF(0, SSP_TIMING_CLOCK_RATE),
		     host->ssp_base + HW_SSP_TIMING);

	if (host->sdio_irq_en) {
		ssp_ctrl1 |= BM_SSP_CTRL1_SDIO_IRQ_EN;
		ssp_ctrl0 |= BM_SSP_CTRL0_SDIO_IRQ_CHECK;
       }

	/* Write the SSP Control Register 0 and 1 values out to the interface */
	__raw_writel(ssp_ctrl0, host->ssp_base + HW_SSP_CTRL0);
	__raw_writel(ssp_ctrl1, host->ssp_base + HW_SSP_CTRL1);
}

static void mxs_mmc_irq_release(struct mxs_mmc_host *host)
{
	free_irq(host->dmairq, host);
	free_irq(host->errirq, host);
}

static int __init mxs_mmc_irq_init(struct mxs_mmc_host *host)
{
	int ret;

	ret = request_irq(host->dmairq, mmc_irq_handler, 0,
			  DRIVER_NAME " dma", host);
	if (ret) {
		dev_err(host->dev, "Unable to set up DMA irq handler\n");
		goto out0;
	}

	ret = request_irq(host->errirq, mmc_irq_handler, IRQF_SHARED,
			  DRIVER_NAME " error", host);
	if (ret) {
		dev_err(host->dev, "Unable to set up SSP error irq handler\n");
		goto out1;
	}
	return 0;

out1:
	free_irq(host->dmairq, host);
out0:
	return ret;
}

/* Allocate and initialise the DMA chains */
static int mxs_mmc_dma_init(struct mxs_mmc_host *host, int reset)
{
	int ret = 0;

	if (!reset) {
		/* Allocate DMA channel */
		ret = mxs_dma_request(host->dmach,
					   host->dev, "MXS MMC/SD");
		if (ret) {
			dev_err(host->dev, "Unable to request DMA channel\n");
			return ret;
		}

		host->dma_buf = dma_alloc_coherent(host->dev, SSP_BUFFER_SIZE,
						   &host->dma_buf_phys,
						   GFP_DMA);
		if (host->dma_buf == NULL) {
			dev_err(host->dev, "Unable to allocate DMA memory\n");
			ret = -ENOMEM;
			goto out_mem;
		}

		host->dma_desc = dma_alloc_coherent(host->dev, PAGE_SIZE,
						   &host->dma_desc_phys,
						   GFP_KERNEL);
		if (host->dma_desc == NULL) {
			dev_err(host->dev,
				"Unable to allocate DMA descriptor\n");
			ret = -ENOMEM;
			goto out_cmd;
		}

		memset(host->dma_desc, 0, PAGE_SIZE);
		host->dma_desc->address = host->dma_desc_phys;
	}

	/* Reset DMA channel */
	mxs_dma_reset(host->dmach);

	/* Enable DMA interrupt */
	mxs_dma_ack_irq(host->dmach);
	mxs_dma_enable_irq(host->dmach, 1);

	return 0;

out_cmd:
	dma_free_coherent(host->dev, SSP_BUFFER_SIZE, host->dma_buf,
			  host->dma_buf_phys);
out_mem:
	mxs_dma_release(host->dmach, host->dev);

	return ret;
}

static void mxs_mmc_dma_release(struct mxs_mmc_host *host)
{
	mxs_dma_reset(host->dmach);

	mxs_dma_enable_irq(host->dmach, 0);
	mxs_dma_disable(host->dmach);
	mxs_dma_get_cooked(host->dmach, NULL);

	dma_free_coherent(host->dev, SSP_BUFFER_SIZE, host->dma_buf,
			  host->dma_buf_phys);

	dma_free_coherent(host->dev, PAGE_SIZE, host->dma_desc,
			  host->dma_desc_phys);
	mxs_dma_release(host->dmach, host->dev);
}

/* Probe peripheral for connected cards */
static int __init mxs_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxs_mmc_platform_data *mmc_data;
	struct mxs_mmc_host *host;
	struct mmc_host *mmc;
	struct resource *r;
	int err = 0;
	unsigned int ssp_ver_major;

	mmc_data = dev->platform_data;
	if (mmc_data == NULL) {
		err = -EINVAL;
		dev_err(dev, "Missing platform data\n");
		goto out;
	}

	/* Allocate main MMC host structure */
	mmc = mmc_alloc_host(sizeof(struct mxs_mmc_host), dev);
	if (!mmc) {
		dev_err(dev, "Unable to allocate MMC host\n");
		err = -ENOMEM;
		goto out;
	}
	host = mmc_priv(mmc);

	host->read_uA = mmc_data->read_uA;
	host->write_uA = mmc_data->write_uA;
	if (mmc_data->power_mmc != NULL)
		host->regulator = regulator_get(NULL, mmc_data->power_mmc);
	if (host->regulator && !IS_ERR(host->regulator))
		regulator_set_mode(host->regulator, REGULATOR_MODE_NORMAL);
	else
		host->regulator = NULL;

	/* get resources: */

	/*
	 *  1. io memory
	 */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_MEM\n");
		err = -ENXIO;
		goto out_res;
	}
	host->ssp_base = IO_ADDRESS(r->start);

	/*
	 *  2. DMA channel
	 */
	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_DMA\n");
		err = -ENXIO;
		goto out_res;
	}
	host->dmach = r->start;

	/*
	 * 3. two IRQs
	 */
	host->dmairq = platform_get_irq(pdev, 0);
	if (host->dmairq < 0) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_IRQ/0\n");
		err = host->dmairq;
		goto out_res;
	}

	host->errirq = platform_get_irq(pdev, 1);
	if (host->errirq < 0) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_IRQ/1\n");
		err = host->errirq;
		goto out_res;
	}

	/* Set up MMC pins */
	if (mmc_data->hw_init) {
		err = mmc_data->hw_init();
		if (err) {
			dev_err(dev, "MMC HW configuration failed\n");
			goto out_res;
		}
	}

	host->mmc = mmc;
	host->dev = dev;

	host->sdio_irq_en = 0;
	host->fastpath_sz = mmc_data->fastpath_sz;
	/* Set minimal clock rate */
	host->clk = clk_get(dev, mmc_data->clock_mmc);
	if (IS_ERR(host->clk)) {
		err = PTR_ERR(host->clk);
		dev_err(dev, "Clocks initialization failed\n");
		goto out_clk;
	}

	clk_enable(host->clk);
	mxs_set_sclk_speed(host, mmc_data->min_clk);

	/* Reset MMC block */
	mxs_mmc_reset(host);

	/* Enable DMA */
	err = mxs_mmc_dma_init(host, 0);
	if (err) {
		dev_err(dev, "DMA init failed\n");
		goto out_dma;
	}

	/* Set up interrupt handlers */
	err = mxs_mmc_irq_init(host);
	if (err) {
		dev_err(dev, "IRQ initialization failed\n");
		goto out_irq;
	}

	/* Get current card status for further cnanges tracking */
	host->present = mxs_mmc_is_plugged(host);

	/* Add a card detection polling timer */
	init_timer(&host->timer);
	host->timer.function = mxs_mmc_detect_poll;
	host->timer.data = (unsigned long)host;
	host->timer.expires = jiffies + MXS_MMC_DETECT_TIMEOUT;
	add_timer(&host->timer);

	mmc->ops = &mxs_mmc_ops;
	mmc->f_min = mmc_data->min_clk;
	mmc->f_max = mmc_data->max_clk;
	mmc->caps = mmc_data->caps;
	mmc->caps |= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;
	mmc->caps |= MMC_CAP_SDIO_IRQ;

	/* Maximum block count requests. */
	mmc->max_blk_size = 512;
	ssp_ver_major = __raw_readl(host->ssp_base + HW_SSP_VERSION) >> 24;
	dev_dbg(host->dev, "ssp ver major is 0x%x\n", ssp_ver_major);
	if (ssp_ver_major > 3) {
		mmc->max_blk_count = SSP_BUFFER_SIZE / 512;
		mmc->max_hw_segs = SSP_BUFFER_SIZE / 512;
		mmc->max_phys_segs = SSP_BUFFER_SIZE / 512;
		mmc->max_req_size = SSP_BUFFER_SIZE;
		mmc->max_seg_size = SSP_BUFFER_SIZE;
	} else {
		mmc->max_blk_count = SSP_BUFFER_SIZE / 512 - 1;
		mmc->max_hw_segs = SSP_BUFFER_SIZE / 512 - 1;
		mmc->max_phys_segs = SSP_BUFFER_SIZE / 512 - 1;
		mmc->max_req_size = SSP_BUFFER_SIZE - 512;
		mmc->max_seg_size = SSP_BUFFER_SIZE - 512;
	}

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	platform_set_drvdata(pdev, mmc);

	spin_lock_init(&host->lock);

	err = mmc_add_host(mmc);
	if (err) {
		dev_err(dev, "Oh God. mmc_add_host failed\n");
		goto out_all;
	}

	dev_info(&pdev->dev, "%s: MXS SSP MMC DMAIRQ %d ERRIRQ %d \n",
			mmc_hostname(mmc), host->dmairq, host->errirq);

	return err;

out_all:

out_irq:
	mxs_mmc_dma_release(host);
out_dma:
	clk_disable(host->clk);
out_clk:
	if (mmc_data->hw_release)
		mmc_data->hw_release();
out_res:
	mmc_free_host(mmc);
out:
	return err;
}

static int __exit mxs_mmc_remove(struct platform_device *pdev)
{
	struct mxs_mmc_host *host;
	struct mxs_mmc_platform_data *mmc_data;
	struct mmc_host *mmc;

	dev_info(&pdev->dev, "Removing\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	host = mmc_priv(mmc);
	mmc_remove_host(mmc);

	/* Disable SSP clock */
	clk_disable(host->clk);
	clk_put(host->clk);

	/* Release IRQs */
	mxs_mmc_irq_release(host);

	/* Delete card detection timer */
	del_timer(&host->timer);

	/* Release DMA */
	mxs_mmc_dma_release(host);
	if (host->regulator)
		regulator_put(host->regulator);

	mmc_free_host(mmc);

	if (mmc_data->hw_release)
		mmc_data->hw_release();

	return 0;
}

#ifdef CONFIG_PM
static int mxs_mmc_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct mxs_mmc_host *host;
	struct mxs_mmc_platform_data *mmc_data;
	struct mmc_host *mmc;
	int ret = 0;

	dev_dbg(&pdev->dev, "Suspending\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	ret = mmc_suspend_host(mmc);
	if (!ret) {
		if (mmc_data && mmc_data->hw_release)
			mmc_data->hw_release();
		clk_disable(host->clk);
	}
	return ret;
}

static int mxs_mmc_resume(struct platform_device *pdev)
{
	struct mxs_mmc_host *host;
	struct mxs_mmc_platform_data *mmc_data;
	struct mmc_host *mmc;

	dev_dbg(&pdev->dev, "Resuming\n");

	mmc_data = pdev->dev.platform_data;
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	clk_enable(host->clk);

	if (mmc_data->hw_init)
		mmc_data->hw_init();
	mxs_mmc_reset(host);
	mxs_mmc_dma_init(host, 1);

	return mmc_resume_host(mmc);
}
#else
#define mxs_mmc_suspend	NULL
#define mxs_mmc_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver mxs_mmc_driver = {
	.probe = mxs_mmc_probe,
	.remove = __exit_p(mxs_mmc_remove),
	.suspend = mxs_mmc_suspend,
	.resume = mxs_mmc_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init mxs_mmc_init(void)
{
	int ret = 0;

	printk(KERN_INFO DRIVER_NAME
	       ": MXS SSP Controller MMC Interface driver\n");
	ret = platform_driver_register(&mxs_mmc_driver);
	if (ret < 0)
		return ret;

	return ret;
}

static void __exit mxs_mmc_exit(void)
{
	platform_driver_unregister(&mxs_mmc_driver);
}

module_init(mxs_mmc_init);
module_exit(mxs_mmc_exit);

MODULE_DESCRIPTION("FREESCALE MXS MMC peripheral");
MODULE_LICENSE("GPL");
