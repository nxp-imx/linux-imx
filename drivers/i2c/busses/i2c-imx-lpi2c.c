// SPDX-License-Identifier: GPL-2.0+
/*
 * This is i.MX low power i2c controller driver.
 *
 * Copyright 2016 Freescale Semiconductor, Inc.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define DRIVER_NAME "imx-lpi2c"

#define LPI2C_PARAM	0x04	/* i2c RX/TX FIFO size */
#define LPI2C_MCR	0x10	/* i2c contrl register */
#define LPI2C_MSR	0x14	/* i2c status register */
#define LPI2C_MIER	0x18	/* i2c interrupt enable */
#define LPI2C_MDER	0x1C	/* i2c DMA enable */
#define LPI2C_MCFGR0	0x20	/* i2c master configuration */
#define LPI2C_MCFGR1	0x24	/* i2c master configuration */
#define LPI2C_MCFGR2	0x28	/* i2c master configuration */
#define LPI2C_MCFGR3	0x2C	/* i2c master configuration */
#define LPI2C_MCCR0	0x48	/* i2c master clk configuration */
#define LPI2C_MCCR1	0x50	/* i2c master clk configuration */
#define LPI2C_MFCR	0x58	/* i2c master FIFO control */
#define LPI2C_MFSR	0x5C	/* i2c master FIFO status */
#define LPI2C_MTDR	0x60	/* i2c master TX data register */
#define LPI2C_MRDR	0x70	/* i2c master RX data register */

/* i2c command */
#define TRAN_DATA	0X00
#define RECV_DATA	0X01
#define GEN_STOP	0X02
#define RECV_DISCARD	0X03
#define GEN_START	0X04
#define START_NACK	0X05
#define START_HIGH	0X06
#define START_HIGH_NACK	0X07

#define MCR_MEN		BIT(0)
#define MCR_RST		BIT(1)
#define MCR_DOZEN	BIT(2)
#define MCR_DBGEN	BIT(3)
#define MCR_RTF		BIT(8)
#define MCR_RRF		BIT(9)
#define MSR_TDF		BIT(0)
#define MSR_RDF		BIT(1)
#define MSR_SDF		BIT(9)
#define MSR_NDF		BIT(10)
#define MSR_ALF		BIT(11)
#define MSR_MBF		BIT(24)
#define MSR_BBF		BIT(25)
#define MIER_TDIE	BIT(0)
#define MIER_RDIE	BIT(1)
#define MIER_SDIE	BIT(9)
#define MIER_NDIE	BIT(10)
#define MCFGR1_AUTOSTOP	BIT(8)
#define MCFGR1_IGNACK	BIT(9)
#define MRDR_RXEMPTY	BIT(14)
#define MDER_TDDE	BIT(0)
#define MDER_RDDE	BIT(1)

#define I2C_CLK_RATIO	24 / 59
#define CHUNK_DATA	256

#define I2C_PM_TIMEOUT		1000 /* ms */
#define I2C_DMA_THRESHOLD	16 /* bytes */
#define I2C_USE_PIO		(-150)

enum lpi2c_imx_mode {
	STANDARD,	/* <=100Kbps */
	FAST,		/* <=400Kbps */
	FAST_PLUS,	/* <=1.0Mbps */
	HS,		/* <=3.4Mbps */
	ULTRA_FAST,	/* <=5.0Mbps */
};

enum lpi2c_imx_pincfg {
	TWO_PIN_OD,
	TWO_PIN_OO,
	TWO_PIN_PP,
	FOUR_PIN_PP,
};

struct lpi2c_imx_struct {
	struct i2c_adapter	adapter;
	resource_size_t		phy_addr;
	int			irq;
	struct clk		*clk_per;
	struct clk		*clk_ipg;
	void __iomem		*base;
	__u8			*rx_buf;
	__u8			*tx_buf;
	struct completion	complete;
	unsigned int		msglen;
	unsigned int		delivered;
	unsigned int		block_data;
	unsigned int		bitrate;
	unsigned int		txfifosize;
	unsigned int		rxfifosize;
	enum lpi2c_imx_mode	mode;

	struct i2c_bus_recovery_info rinfo;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_pins_default;
	struct pinctrl_state *pinctrl_pins_gpio;

	bool			can_use_dma;
	bool			using_dma;
	bool			xferred;
	struct i2c_msg		*msg;
	dma_addr_t		dma_addr;
	struct dma_chan		*dma_tx;
	struct dma_chan		*dma_rx;
	enum dma_data_direction dma_direction;
	u8			*dma_buf;
	unsigned int		dma_len;
};

static void lpi2c_imx_intctrl(struct lpi2c_imx_struct *lpi2c_imx,
			      unsigned int enable)
{
	writel(enable, lpi2c_imx->base + LPI2C_MIER);
}

static int lpi2c_imx_bus_busy(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned long orig_jiffies = jiffies;
	unsigned int temp;

	while (1) {
		temp = readl(lpi2c_imx->base + LPI2C_MSR);

		/* check for arbitration lost, clear if set */
		if (temp & MSR_ALF) {
			writel(temp, lpi2c_imx->base + LPI2C_MSR);
			return -EAGAIN;
		}

		if (temp & (MSR_BBF | MSR_MBF))
			break;

		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(500))) {
			dev_dbg(&lpi2c_imx->adapter.dev, "bus not work\n");
			if (lpi2c_imx->adapter.bus_recovery_info)
				i2c_recover_bus(&lpi2c_imx->adapter);
			return -ETIMEDOUT;
		}
		schedule();
	}

	return 0;
}

static void lpi2c_imx_set_mode(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned int bitrate = lpi2c_imx->bitrate;
	enum lpi2c_imx_mode mode;

	if (bitrate <= I2C_MAX_STANDARD_MODE_FREQ)
		mode = STANDARD;
	else if (bitrate <= I2C_MAX_FAST_MODE_FREQ)
		mode = FAST;
	else if (bitrate <= I2C_MAX_FAST_MODE_PLUS_FREQ)
		mode = FAST_PLUS;
	else if (bitrate <= I2C_MAX_HIGH_SPEED_MODE_FREQ)
		mode = HS;
	else
		mode = ULTRA_FAST;

	lpi2c_imx->mode = mode;
}

static int lpi2c_imx_start(struct lpi2c_imx_struct *lpi2c_imx,
			   struct i2c_msg *msgs)
{
	unsigned int temp;

	temp = readl(lpi2c_imx->base + LPI2C_MCR);
	temp |= MCR_RRF | MCR_RTF;
	writel(temp, lpi2c_imx->base + LPI2C_MCR);
	writel(0x7f00, lpi2c_imx->base + LPI2C_MSR);

	temp = i2c_8bit_addr_from_msg(msgs) | (GEN_START << 8);
	writel(temp, lpi2c_imx->base + LPI2C_MTDR);

	return lpi2c_imx_bus_busy(lpi2c_imx);
}

static void lpi2c_imx_stop(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned long orig_jiffies = jiffies;
	unsigned int temp;

	writel(GEN_STOP << 8, lpi2c_imx->base + LPI2C_MTDR);

	do {
		temp = readl(lpi2c_imx->base + LPI2C_MSR);
		if (temp & MSR_SDF)
			break;

		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(500))) {
			dev_dbg(&lpi2c_imx->adapter.dev, "stop timeout\n");
			if (lpi2c_imx->adapter.bus_recovery_info)
				i2c_recover_bus(&lpi2c_imx->adapter);
			break;
		}
		schedule();

	} while (1);
}

/* CLKLO = (1 - I2C_CLK_RATIO) * clk_cycle, SETHOLD = CLKHI, DATAVD = CLKHI/2
   CLKHI = I2C_CLK_RATIO * clk_cycle */
static int lpi2c_imx_config(struct lpi2c_imx_struct *lpi2c_imx)
{
	u8 prescale, filt, sethold, datavd;
	unsigned int clk_rate, clk_cycle, clkhi, clklo;
	enum lpi2c_imx_pincfg pincfg;
	unsigned int temp;

	lpi2c_imx_set_mode(lpi2c_imx);

	clk_rate = clk_get_rate(lpi2c_imx->clk_per);
	if (!clk_rate) {
		dev_dbg(&lpi2c_imx->adapter.dev, "clk_per rate is 0\n");
		return -EINVAL;
	}

	if (lpi2c_imx->mode == HS || lpi2c_imx->mode == ULTRA_FAST)
		filt = 0;
	else
		filt = 2;

	for (prescale = 0; prescale <= 7; prescale++) {
		clk_cycle = clk_rate / ((1 << prescale) * lpi2c_imx->bitrate)
			    - (2 + filt) / (1 << prescale);
		clkhi = clk_cycle * I2C_CLK_RATIO;
		clklo = clk_cycle - clkhi;
		if (clklo < 64)
			break;
	}

	if (prescale > 7)
		return -EINVAL;

	/* set MCFGR1: PINCFG, PRESCALE, IGNACK */
	if (lpi2c_imx->mode == ULTRA_FAST)
		pincfg = TWO_PIN_OO;
	else
		pincfg = TWO_PIN_OD;
	temp = prescale | pincfg << 24;

	if (lpi2c_imx->mode == ULTRA_FAST)
		temp |= MCFGR1_IGNACK;

	writel(temp, lpi2c_imx->base + LPI2C_MCFGR1);

	/* set MCFGR2: FILTSDA, FILTSCL */
	temp = (filt << 16) | (filt << 24);
	writel(temp, lpi2c_imx->base + LPI2C_MCFGR2);

	/* set MCCR: DATAVD, SETHOLD, CLKHI, CLKLO */
	sethold = clkhi;
	datavd = clkhi >> 1;
	temp = datavd << 24 | sethold << 16 | clkhi << 8 | clklo;

	if (lpi2c_imx->mode == HS)
		writel(temp, lpi2c_imx->base + LPI2C_MCCR1);
	else
		writel(temp, lpi2c_imx->base + LPI2C_MCCR0);

	return 0;
}

static int lpi2c_imx_master_enable(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned int temp;
	int ret;

	ret = pm_runtime_resume_and_get(lpi2c_imx->adapter.dev.parent);
	if (ret < 0)
		return ret;

	temp = MCR_RST;
	writel(temp, lpi2c_imx->base + LPI2C_MCR);
	writel(0, lpi2c_imx->base + LPI2C_MCR);

	ret = lpi2c_imx_config(lpi2c_imx);
	if (ret)
		goto rpm_put;

	if (lpi2c_imx->can_use_dma)
		writel(MDER_TDDE | MDER_RDDE, lpi2c_imx->base + LPI2C_MDER);

	temp = readl(lpi2c_imx->base + LPI2C_MCR);
	temp |= MCR_MEN;
	writel(temp, lpi2c_imx->base + LPI2C_MCR);

	return 0;

rpm_put:
	pm_runtime_mark_last_busy(lpi2c_imx->adapter.dev.parent);
	pm_runtime_put_autosuspend(lpi2c_imx->adapter.dev.parent);

	return ret;
}

static int lpi2c_imx_master_disable(struct lpi2c_imx_struct *lpi2c_imx)
{
	u32 temp;

	temp = readl(lpi2c_imx->base + LPI2C_MCR);
	temp &= ~MCR_MEN;
	writel(temp, lpi2c_imx->base + LPI2C_MCR);

	pm_runtime_mark_last_busy(lpi2c_imx->adapter.dev.parent);
	pm_runtime_put_autosuspend(lpi2c_imx->adapter.dev.parent);

	return 0;
}

static int lpi2c_imx_msg_complete(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned long timeout;

	timeout = wait_for_completion_timeout(&lpi2c_imx->complete, HZ);

	return timeout ? 0 : -ETIMEDOUT;
}

static int lpi2c_imx_txfifo_empty(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned long orig_jiffies = jiffies;
	u32 txcnt;

	do {
		txcnt = readl(lpi2c_imx->base + LPI2C_MFSR) & 0xff;

		if (readl(lpi2c_imx->base + LPI2C_MSR) & MSR_NDF) {
			dev_dbg(&lpi2c_imx->adapter.dev, "NDF detected\n");
			return -EIO;
		}

		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(500))) {
			dev_dbg(&lpi2c_imx->adapter.dev, "txfifo empty timeout\n");
			if (lpi2c_imx->adapter.bus_recovery_info)
				i2c_recover_bus(&lpi2c_imx->adapter);
			return -ETIMEDOUT;
		}
		schedule();

	} while (txcnt);

	return 0;
}

static void lpi2c_imx_set_tx_watermark(struct lpi2c_imx_struct *lpi2c_imx)
{
	writel(lpi2c_imx->txfifosize >> 1, lpi2c_imx->base + LPI2C_MFCR);
}

static void lpi2c_imx_set_rx_watermark(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned int temp, remaining;

	remaining = lpi2c_imx->msglen - lpi2c_imx->delivered;

	if (remaining > (lpi2c_imx->rxfifosize >> 1))
		temp = lpi2c_imx->rxfifosize >> 1;
	else
		temp = 0;

	writel(temp << 16, lpi2c_imx->base + LPI2C_MFCR);
}

static void lpi2c_imx_write_txfifo(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned int data, txcnt;

	txcnt = readl(lpi2c_imx->base + LPI2C_MFSR) & 0xff;

	while (txcnt < lpi2c_imx->txfifosize) {
		if (lpi2c_imx->delivered == lpi2c_imx->msglen)
			break;

		data = lpi2c_imx->tx_buf[lpi2c_imx->delivered++];
		writel(data, lpi2c_imx->base + LPI2C_MTDR);
		txcnt++;
	}

	if (lpi2c_imx->delivered < lpi2c_imx->msglen)
		lpi2c_imx_intctrl(lpi2c_imx, MIER_TDIE | MIER_NDIE);
	else
		complete(&lpi2c_imx->complete);
}

static void lpi2c_imx_read_rxfifo(struct lpi2c_imx_struct *lpi2c_imx)
{
	unsigned int blocklen, remaining;
	unsigned int temp, data;

	do {
		data = readl(lpi2c_imx->base + LPI2C_MRDR);
		if (data & MRDR_RXEMPTY)
			break;

		lpi2c_imx->rx_buf[lpi2c_imx->delivered++] = data & 0xff;
	} while (1);

	/*
	 * First byte is the length of remaining packet in the SMBus block
	 * data read. Add it to msgs->len.
	 */
	if (lpi2c_imx->block_data) {
		blocklen = lpi2c_imx->rx_buf[0];
		lpi2c_imx->msglen += blocklen;
	}

	remaining = lpi2c_imx->msglen - lpi2c_imx->delivered;

	if (!remaining) {
		complete(&lpi2c_imx->complete);
		return;
	}

	/* not finished, still waiting for rx data */
	lpi2c_imx_set_rx_watermark(lpi2c_imx);

	/* multiple receive commands */
	if (lpi2c_imx->block_data) {
		lpi2c_imx->block_data = 0;
		temp = remaining;
		temp |= (RECV_DATA << 8);
		writel(temp, lpi2c_imx->base + LPI2C_MTDR);
	} else if (!(lpi2c_imx->delivered & 0xff)) {
		temp = (remaining > CHUNK_DATA ? CHUNK_DATA : remaining) - 1;
		temp |= (RECV_DATA << 8);
		writel(temp, lpi2c_imx->base + LPI2C_MTDR);
	}

	lpi2c_imx_intctrl(lpi2c_imx, MIER_RDIE);
}

static void lpi2c_imx_write(struct lpi2c_imx_struct *lpi2c_imx,
			    struct i2c_msg *msgs)
{
	lpi2c_imx->tx_buf = msgs->buf;
	lpi2c_imx_set_tx_watermark(lpi2c_imx);
	lpi2c_imx_write_txfifo(lpi2c_imx);
}

static void lpi2c_imx_read(struct lpi2c_imx_struct *lpi2c_imx,
			   struct i2c_msg *msgs)
{
	unsigned int temp;

	lpi2c_imx->rx_buf = msgs->buf;
	lpi2c_imx->block_data = msgs->flags & I2C_M_RECV_LEN;

	lpi2c_imx_set_rx_watermark(lpi2c_imx);
	temp = msgs->len > CHUNK_DATA ? CHUNK_DATA - 1 : msgs->len - 1;
	temp |= (RECV_DATA << 8);
	writel(temp, lpi2c_imx->base + LPI2C_MTDR);

	lpi2c_imx_intctrl(lpi2c_imx, MIER_RDIE | MIER_NDIE);
}

static void lpi2c_dma_unmap(struct lpi2c_imx_struct *lpi2c_imx)
{
	struct dma_chan *chan = lpi2c_imx->dma_direction == DMA_FROM_DEVICE
				? lpi2c_imx->dma_rx : lpi2c_imx->dma_tx;

	dma_unmap_single(chan->device->dev, lpi2c_imx->dma_addr,
			 lpi2c_imx->dma_len, lpi2c_imx->dma_direction);

	lpi2c_imx->dma_direction = DMA_NONE;
}

static void lpi2c_cleanup_dma(struct lpi2c_imx_struct *lpi2c_imx)
{
	if (lpi2c_imx->dma_direction == DMA_NONE)
		return;
	else if (lpi2c_imx->dma_direction == DMA_FROM_DEVICE)
		dmaengine_terminate_all(lpi2c_imx->dma_rx);
	else if (lpi2c_imx->dma_direction == DMA_TO_DEVICE)
		dmaengine_terminate_all(lpi2c_imx->dma_tx);

	lpi2c_dma_unmap(lpi2c_imx);
}

static void lpi2c_dma_callback(void *data)
{
	struct lpi2c_imx_struct *lpi2c_imx = (struct lpi2c_imx_struct *)data;

	lpi2c_dma_unmap(lpi2c_imx);
	writel(GEN_STOP << 8, lpi2c_imx->base + LPI2C_MTDR);
	lpi2c_imx->xferred = true;

	complete(&lpi2c_imx->complete);
}

static int lpi2c_dma_submit(struct lpi2c_imx_struct *lpi2c_imx,
			   struct i2c_msg *msg)
{
	bool read = msg->flags & I2C_M_RD;
	enum dma_data_direction dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	struct dma_chan *chan = read ? lpi2c_imx->dma_rx : lpi2c_imx->dma_tx;
	struct dma_async_tx_descriptor *txdesc;
	dma_cookie_t cookie;

	lpi2c_imx->dma_len = read ? msg->len - 1 : msg->len;
	lpi2c_imx->msg = msg;
	lpi2c_imx->dma_direction = dir;

	if (IS_ERR(chan))
		return PTR_ERR(chan);

	lpi2c_imx->dma_addr = dma_map_single(chan->device->dev,
					     lpi2c_imx->dma_buf,
					     lpi2c_imx->dma_len, dir);
	if (dma_mapping_error(chan->device->dev, lpi2c_imx->dma_addr)) {
		dev_err(&lpi2c_imx->adapter.dev, "dma map failed, use pio\n");
		return -EINVAL;
	}

	txdesc = dmaengine_prep_slave_single(chan, lpi2c_imx->dma_addr,
					lpi2c_imx->dma_len, read ?
					DMA_DEV_TO_MEM : DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc) {
		dev_err(&lpi2c_imx->adapter.dev, "dma prep slave sg failed, use pio\n");
		lpi2c_cleanup_dma(lpi2c_imx);
		return -EINVAL;
	}

	reinit_completion(&lpi2c_imx->complete);
	txdesc->callback = lpi2c_dma_callback;
	txdesc->callback_param = (void *)lpi2c_imx;

	cookie = dmaengine_submit(txdesc);
	if (dma_submit_error(cookie)) {
		dev_err(&lpi2c_imx->adapter.dev, "submitting dma failed, use pio\n");
		lpi2c_cleanup_dma(lpi2c_imx);
		return -EINVAL;
	}

	lpi2c_imx_intctrl(lpi2c_imx, MIER_NDIE);

	dma_async_issue_pending(chan);

	return 0;
}

static bool is_use_dma(struct lpi2c_imx_struct *lpi2c_imx, struct i2c_msg *msg)
{
	if (!lpi2c_imx->can_use_dma)
		return false;

	if (msg->len < I2C_DMA_THRESHOLD)
		return false;

	return true;
}

static int lpi2c_imx_push_rx_cmd(struct lpi2c_imx_struct *lpi2c_imx,
				 struct i2c_msg *msg)
{
	unsigned int temp, rx_remain;
	unsigned long orig_jiffies = jiffies;

	if ((msg->flags & I2C_M_RD)) {
		rx_remain = msg->len;
		do {
			temp = rx_remain > CHUNK_DATA ?
				CHUNK_DATA - 1 : rx_remain - 1;
			temp |= (RECV_DATA << 8);
			while ((readl(lpi2c_imx->base + LPI2C_MFSR) & 0xff) > 2) {
				if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(1000))) {
					dev_dbg(&lpi2c_imx->adapter.dev, "txfifo empty timeout\n");
					if (lpi2c_imx->adapter.bus_recovery_info)
						i2c_recover_bus(&lpi2c_imx->adapter);
					return -ETIMEDOUT;
				}
				schedule();
			}
			writel(temp, lpi2c_imx->base + LPI2C_MTDR);
			rx_remain = rx_remain - (temp & 0xff) - 1;
		} while (rx_remain > 0);
	}

	return 0;
}

static int lpi2c_dma_xfer(struct lpi2c_imx_struct *lpi2c_imx,
			   struct i2c_msg *msg)
{
	int result;

	result = lpi2c_dma_submit(lpi2c_imx, msg);
	if (!result) {
		result = lpi2c_imx_push_rx_cmd(lpi2c_imx, msg);
		if (result)
			return result;
		result = lpi2c_imx_msg_complete(lpi2c_imx);
		return result;
	}

	/* DMA xfer failed, try to use PIO, clean up dma things */
	i2c_put_dma_safe_msg_buf(lpi2c_imx->dma_buf, lpi2c_imx->msg,
				 lpi2c_imx->xferred);
	lpi2c_cleanup_dma(lpi2c_imx);

	return I2C_USE_PIO;
}

static int lpi2c_imx_xfer(struct i2c_adapter *adapter,
			  struct i2c_msg *msgs, int num)
{
	struct lpi2c_imx_struct *lpi2c_imx = i2c_get_adapdata(adapter);
	unsigned int temp;
	int i, result;

	result = lpi2c_imx_master_enable(lpi2c_imx);
	if (result)
		return result;

	for (i = 0; i < num; i++) {
		lpi2c_imx->xferred = false;
		lpi2c_imx->using_dma = false;

		result = lpi2c_imx_start(lpi2c_imx, &msgs[i]);
		if (result)
			goto disable;

		/* quick smbus */
		if (num == 1 && msgs[0].len == 0)
			goto stop;

		if (is_use_dma(lpi2c_imx, &msgs[i])) {
			lpi2c_imx->using_dma = true;

			writel(0x1, lpi2c_imx->base + LPI2C_MFCR);

			lpi2c_imx->dma_buf = i2c_get_dma_safe_msg_buf(&msgs[i],
							    I2C_DMA_THRESHOLD);
			if (lpi2c_imx->dma_buf) {
				result = lpi2c_dma_xfer(lpi2c_imx, &msgs[i]);
				if (result != I2C_USE_PIO)
					goto stop;
			}
		}

		lpi2c_imx->using_dma = false;
		lpi2c_imx->delivered = 0;
		lpi2c_imx->msglen = msgs[i].len;
		reinit_completion(&lpi2c_imx->complete);

		if (msgs[i].flags & I2C_M_RD)
			lpi2c_imx_read(lpi2c_imx, &msgs[i]);
		else
			lpi2c_imx_write(lpi2c_imx, &msgs[i]);

		result = lpi2c_imx_msg_complete(lpi2c_imx);
		if (result)
			goto stop;

		if (!(msgs[i].flags & I2C_M_RD)) {
			result = lpi2c_imx_txfifo_empty(lpi2c_imx);
			if (result)
				goto stop;
		}
	}

stop:
	if (!lpi2c_imx->using_dma)
		lpi2c_imx_stop(lpi2c_imx);
	else {
		i2c_put_dma_safe_msg_buf(lpi2c_imx->dma_buf, lpi2c_imx->msg,
					 lpi2c_imx->xferred);
		if (result) {
			lpi2c_cleanup_dma(lpi2c_imx);
			writel(GEN_STOP << 8, lpi2c_imx->base + LPI2C_MTDR);
		}
	}

	temp = readl(lpi2c_imx->base + LPI2C_MSR);
	if ((temp & MSR_NDF) && !result)
		result = -EIO;

disable:
	lpi2c_imx_master_disable(lpi2c_imx);

	dev_dbg(&lpi2c_imx->adapter.dev, "<%s> exit with: %s: %d\n", __func__,
		(result < 0) ? "error" : "success msg",
		(result < 0) ? result : num);

	return (result < 0) ? result : num;
}

static irqreturn_t lpi2c_imx_isr(int irq, void *dev_id)
{
	struct lpi2c_imx_struct *lpi2c_imx = dev_id;
	unsigned int temp;

	lpi2c_imx_intctrl(lpi2c_imx, 0);
	temp = readl(lpi2c_imx->base + LPI2C_MSR);

	if (temp & MSR_NDF) {
		if (lpi2c_imx->using_dma) {
			lpi2c_cleanup_dma(lpi2c_imx);
			writel(GEN_STOP << 8, lpi2c_imx->base + LPI2C_MTDR);
		}
		complete(&lpi2c_imx->complete);
		goto ret;
	}

	if (temp & MSR_RDF)
		lpi2c_imx_read_rxfifo(lpi2c_imx);
	else if (temp & MSR_TDF)
		lpi2c_imx_write_txfifo(lpi2c_imx);

ret:
	return IRQ_HANDLED;
}

static void lpi2c_imx_prepare_recovery(struct i2c_adapter *adap)
{
	struct lpi2c_imx_struct *lpi2c_imx;

	lpi2c_imx = container_of(adap, struct lpi2c_imx_struct, adapter);

	pinctrl_select_state(lpi2c_imx->pinctrl, lpi2c_imx->pinctrl_pins_gpio);
}

static void lpi2c_imx_unprepare_recovery(struct i2c_adapter *adap)
{
	struct lpi2c_imx_struct *lpi2c_imx;

	lpi2c_imx = container_of(adap, struct lpi2c_imx_struct, adapter);

	pinctrl_select_state(lpi2c_imx->pinctrl, lpi2c_imx->pinctrl_pins_default);
}

/*
 * We switch SCL and SDA to their GPIO function and do some bitbanging
 * for bus recovery. These alternative pinmux settings can be
 * described in the device tree by a separate pinctrl state "gpio". If
 * this is missing this is not a big problem, the only implication is
 * that we can't do bus recovery.
 */
static int lpi2c_imx_init_recovery_info(struct lpi2c_imx_struct *lpi2c_imx,
		struct platform_device *pdev)
{
	struct i2c_bus_recovery_info *rinfo = &lpi2c_imx->rinfo;

	lpi2c_imx->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!lpi2c_imx->pinctrl || IS_ERR(lpi2c_imx->pinctrl)) {
		dev_info(&pdev->dev, "can't get pinctrl, bus recovery not supported\n");
		return PTR_ERR(lpi2c_imx->pinctrl);
	}

	lpi2c_imx->pinctrl_pins_default = pinctrl_lookup_state(lpi2c_imx->pinctrl,
			PINCTRL_STATE_DEFAULT);
	lpi2c_imx->pinctrl_pins_gpio = pinctrl_lookup_state(lpi2c_imx->pinctrl,
			"gpio");
	rinfo->sda_gpiod = devm_gpiod_get(&pdev->dev, "sda", GPIOD_IN);
	rinfo->scl_gpiod = devm_gpiod_get(&pdev->dev, "scl", GPIOD_OUT_HIGH_OPEN_DRAIN);

	if (PTR_ERR(rinfo->sda_gpiod) == -EPROBE_DEFER ||
	    PTR_ERR(rinfo->scl_gpiod) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(rinfo->sda_gpiod) ||
		   IS_ERR(rinfo->scl_gpiod) ||
		   IS_ERR(lpi2c_imx->pinctrl_pins_default) ||
		   IS_ERR(lpi2c_imx->pinctrl_pins_gpio)) {
		dev_dbg(&pdev->dev, "recovery information incomplete\n");
		return 0;
	}

	dev_info(&pdev->dev, "using scl%s for recovery\n",
		 rinfo->sda_gpiod ? ",sda" : "");

	rinfo->prepare_recovery = lpi2c_imx_prepare_recovery;
	rinfo->unprepare_recovery = lpi2c_imx_unprepare_recovery;
	rinfo->recover_bus = i2c_generic_scl_recovery;
	lpi2c_imx->adapter.bus_recovery_info = rinfo;

	return 0;
}

static u32 lpi2c_imx_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
		I2C_FUNC_SMBUS_READ_BLOCK_DATA;
}

static const struct i2c_algorithm lpi2c_imx_algo = {
	.master_xfer	= lpi2c_imx_xfer,
	.functionality	= lpi2c_imx_func,
};

static const struct of_device_id lpi2c_imx_of_match[] = {
	{ .compatible = "fsl,imx7ulp-lpi2c" },
	{ },
};
MODULE_DEVICE_TABLE(of, lpi2c_imx_of_match);

static void lpi2c_dma_exit(struct lpi2c_imx_struct *lpi2c_imx)
{
	if (lpi2c_imx->dma_rx) {
		dma_release_channel(lpi2c_imx->dma_rx);
		lpi2c_imx->dma_rx = NULL;
	}

	if (lpi2c_imx->dma_tx) {
		dma_release_channel(lpi2c_imx->dma_tx);
		lpi2c_imx->dma_tx = NULL;
	}
}

static int lpi2c_dma_init(struct device *dev,
			  struct lpi2c_imx_struct *lpi2c_imx)
{
	int ret;
	struct dma_slave_config dma_sconfig;

	/* Prepare for TX DMA: */
	lpi2c_imx->dma_tx = dma_request_chan(dev, "tx");
	if (IS_ERR(lpi2c_imx->dma_tx)) {
		ret = PTR_ERR(lpi2c_imx->dma_tx);
		dev_err(dev, "can't get the TX DMA channel, error %d!\n", ret);
		lpi2c_imx->dma_tx = NULL;
		goto err;
	}

	dma_sconfig.dst_addr = lpi2c_imx->phy_addr + LPI2C_MTDR;
	dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.dst_maxburst = 1;
	dma_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(lpi2c_imx->dma_tx, &dma_sconfig);
	if (ret < 0) {
		dev_err(dev, "can't configure tx channel (%d)\n", ret);
		goto fail_tx;
	}

	/* Prepare for RX DMA: */
	lpi2c_imx->dma_rx = dma_request_chan(dev, "rx");
	if (IS_ERR(lpi2c_imx->dma_rx)) {
		ret = PTR_ERR(lpi2c_imx->dma_rx);
		dev_err(dev, "can't get the RX DMA channel, error %d\n", ret);
		lpi2c_imx->dma_rx = NULL;
		goto fail_tx;
	}

	dma_sconfig.src_addr = lpi2c_imx->phy_addr + LPI2C_MRDR;
	dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_sconfig.src_maxburst = 1;
	dma_sconfig.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(lpi2c_imx->dma_rx, &dma_sconfig);
	if (ret < 0) {
		dev_err(dev, "can't configure rx channel (%d)\n", ret);
		goto fail_rx;
	}

	lpi2c_imx->can_use_dma = true;
	lpi2c_imx->using_dma = false;

	return 0;
fail_rx:
	dma_release_channel(lpi2c_imx->dma_rx);
fail_tx:
	dma_release_channel(lpi2c_imx->dma_tx);
err:
	lpi2c_dma_exit(lpi2c_imx);
	lpi2c_imx->can_use_dma = false;
	return ret;
}

static int lpi2c_imx_probe(struct platform_device *pdev)
{
	struct lpi2c_imx_struct *lpi2c_imx;
	unsigned int temp;
	int ret;
	struct resource *res;

	lpi2c_imx = devm_kzalloc(&pdev->dev, sizeof(*lpi2c_imx), GFP_KERNEL);
	if (!lpi2c_imx)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpi2c_imx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lpi2c_imx->base))
		return PTR_ERR(lpi2c_imx->base);

	lpi2c_imx->phy_addr = (dma_addr_t)res->start;
	lpi2c_imx->irq = platform_get_irq(pdev, 0);
	if (lpi2c_imx->irq < 0)
		return lpi2c_imx->irq;

	lpi2c_imx->adapter.owner	= THIS_MODULE;
	lpi2c_imx->adapter.algo		= &lpi2c_imx_algo;
	lpi2c_imx->adapter.dev.parent	= &pdev->dev;
	lpi2c_imx->adapter.dev.of_node	= pdev->dev.of_node;
	strlcpy(lpi2c_imx->adapter.name, pdev->name,
		sizeof(lpi2c_imx->adapter.name));

	lpi2c_imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(lpi2c_imx->clk_per)) {
		dev_err(&pdev->dev, "can't get I2C peripheral clock\n");
		return PTR_ERR(lpi2c_imx->clk_per);
	}

	lpi2c_imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(lpi2c_imx->clk_ipg)) {
		dev_err(&pdev->dev, "can't get I2C ipg clock\n");
		return PTR_ERR(lpi2c_imx->clk_ipg);
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "clock-frequency", &lpi2c_imx->bitrate);
	if (ret)
		lpi2c_imx->bitrate = I2C_MAX_STANDARD_MODE_FREQ;

	i2c_set_adapdata(&lpi2c_imx->adapter, lpi2c_imx);
	platform_set_drvdata(pdev, lpi2c_imx);

	pm_runtime_set_autosuspend_delay(&pdev->dev, I2C_PM_TIMEOUT);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);
	temp = readl(lpi2c_imx->base + LPI2C_PARAM);
	lpi2c_imx->txfifosize = 1 << (temp & 0x0f);
	lpi2c_imx->rxfifosize = 1 << ((temp >> 8) & 0x0f);

	/* Init optional bus recovery function */
	ret = lpi2c_imx_init_recovery_info(lpi2c_imx, pdev);
	/* Give it another chance if pinctrl used is not ready yet */
	if (ret == -EPROBE_DEFER)
		goto rpm_disable;

	/* Init DMA */
	lpi2c_imx->dma_direction = DMA_NONE;
	lpi2c_imx->dma_rx = lpi2c_imx->dma_tx = NULL;
	ret = lpi2c_dma_init(&pdev->dev, lpi2c_imx);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "dma setup error %d, use pio\n", ret);
		if (ret == -EPROBE_DEFER)
			goto rpm_disable;
	}

	init_completion(&lpi2c_imx->complete);

	ret = i2c_add_adapter(&lpi2c_imx->adapter);
	if (ret)
		goto rpm_disable;

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);

	dev_info(&lpi2c_imx->adapter.dev, "LPI2C adapter registered\n");

	return 0;

rpm_disable:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int lpi2c_imx_remove(struct platform_device *pdev)
{
	struct lpi2c_imx_struct *lpi2c_imx = platform_get_drvdata(pdev);

	i2c_del_adapter(&lpi2c_imx->adapter);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);

	return 0;
}

static int __maybe_unused lpi2c_runtime_suspend(struct device *dev)
{
	struct lpi2c_imx_struct *lpi2c_imx = dev_get_drvdata(dev);

	devm_free_irq(dev, lpi2c_imx->irq, lpi2c_imx);
	clk_disable_unprepare(lpi2c_imx->clk_ipg);
	clk_disable_unprepare(lpi2c_imx->clk_per);
	pinctrl_pm_select_idle_state(dev);

	return 0;
}

static int __maybe_unused lpi2c_runtime_resume(struct device *dev)
{
	struct lpi2c_imx_struct *lpi2c_imx = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);
	ret = clk_prepare_enable(lpi2c_imx->clk_per);
	if (ret) {
		dev_err(dev, "can't enable I2C per clock, ret=%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(lpi2c_imx->clk_ipg);
	if (ret) {
		clk_disable_unprepare(lpi2c_imx->clk_per);
		dev_err(dev, "can't enable I2C ipg clock, ret=%d\n", ret);
	}

	ret = devm_request_irq(dev, lpi2c_imx->irq, lpi2c_imx_isr,
			       IRQF_NO_SUSPEND,
			       dev_name(dev), lpi2c_imx);
	if (ret) {
		dev_err(dev, "can't claim irq %d\n", lpi2c_imx->irq);
		return ret;
	}

	return ret;
}

static int lpi2c_suspend_noirq(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		return ret;

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int lpi2c_resume_noirq(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops lpi2c_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(lpi2c_suspend_noirq,
				     lpi2c_resume_noirq)
	SET_RUNTIME_PM_OPS(lpi2c_runtime_suspend,
			   lpi2c_runtime_resume, NULL)
};

static struct platform_driver lpi2c_imx_driver = {
	.probe = lpi2c_imx_probe,
	.remove = lpi2c_imx_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = lpi2c_imx_of_match,
		.pm = &lpi2c_pm_ops,
	},
};

static int __init lpi2c_imx_init(void)
{
	return platform_driver_register(&lpi2c_imx_driver);
}
subsys_initcall(lpi2c_imx_init);

static void __exit lpi2c_imx_exit(void)
{
	platform_driver_unregister(&lpi2c_imx_driver);
}
module_exit(lpi2c_imx_exit);

MODULE_AUTHOR("Gao Pan <pandy.gao@nxp.com>");
MODULE_DESCRIPTION("I2C adapter driver for LPI2C bus");
MODULE_LICENSE("GPL");
