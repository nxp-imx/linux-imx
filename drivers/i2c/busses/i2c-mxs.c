/*
 * Freescale MX28 I2C bus driver
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/dmaengine.h>
#include <mach/device.h>
#include <mach/regs-i2c.h>
#include <mach/system.h>
#include <mach/hardware.h>

#include "i2c-mxs.h"

/* 2 for read, 1 for write */
#define	NR_DESC		3
static struct mxs_dma_desc *desc[NR_DESC];
static dma_addr_t i2c_buf_phys;
static u8 *i2c_buf_virt;

#define	CMD_I2C_SELECT	(BM_I2C_CTRL0_RETAIN_CLOCK |	\
			BM_I2C_CTRL0_PRE_SEND_START |	\
			BM_I2C_CTRL0_MASTER_MODE |	\
			BM_I2C_CTRL0_DIRECTION |	\
			BF_I2C_CTRL0_XFER_COUNT(1))
#define	CMD_I2C_WRITE	(BM_I2C_CTRL0_PRE_SEND_START |	\
			BM_I2C_CTRL0_MASTER_MODE |	\
			BM_I2C_CTRL0_DIRECTION)
#define	CMD_I2C_READ	(BM_I2C_CTRL0_SEND_NAK_ON_LAST |	\
			BM_I2C_CTRL0_MASTER_MODE)

/* Hack for platform which does not support PioQueue Mode */
#if !defined(HW_I2C_QUEUECMD) || 	\
    !defined(HW_I2C_QUEUEDATA) ||	\
    !defined(HW_I2C_QUEUECTRL_CLR) ||	\
    !defined(HW_I2C_QUEUECTRL_SET)
#warning "Pio Queue Mode *NOT* Support!"
#define HW_I2C_QUEUECMD		HW_I2C_VERSION
#define HW_I2C_QUEUEDATA	HW_I2C_VERSION
#define HW_I2C_QUEUECTRL_SET	HW_I2C_VERSION
#define HW_I2C_QUEUECTRL_CLR	HW_I2C_VERSION
#endif

static void hw_i2c_dmachan_reset(struct mxs_i2c_dev *dev)
{
	mxs_dma_disable(dev->dma_chan);
	mxs_dma_reset(dev->dma_chan);
	mxs_dma_ack_irq(dev->dma_chan);
}

static mxs_i2c_reset(struct mxs_i2c_dev *mxs_i2c)
{
	hw_i2c_dmachan_reset(mxs_i2c);
	mxs_dma_enable_irq(mxs_i2c->dma_chan, 1);
	mxs_reset_block((void __iomem *)mxs_i2c->regbase, 0);
	__raw_writel(0x0000FF00, mxs_i2c->regbase + HW_I2C_CTRL1_SET);
}

static int hw_i2c_dma_init(struct platform_device *pdev)
{
	struct mxs_i2c_dev *mxs_i2c = platform_get_drvdata(pdev);
	int i, ret;

	ret = mxs_dma_request(mxs_i2c->dma_chan, &pdev->dev, "i2c");
	if (ret)
		return ret;

	for (i = 0; i < NR_DESC; i++) {
		desc[i] = mxs_dma_alloc_desc();
		if (desc[i] == NULL)
			goto err;
	}

	i2c_buf_virt = dma_alloc_coherent(&pdev->dev,
					  PAGE_SIZE, &i2c_buf_phys, GFP_KERNEL);
	if (i2c_buf_virt == NULL)
		goto err;

	hw_i2c_dmachan_reset(mxs_i2c);
	mxs_dma_enable_irq(mxs_i2c->dma_chan, 1);

	return 0;

err:
	while (--i >= 0)
		mxs_dma_free_desc(desc[i]);

	return -ENOMEM;
}

static void hw_i2c_dma_uninit(struct platform_device *pdev)
{
	struct mxs_i2c_dev *mxs_i2c = platform_get_drvdata(pdev);
	int i;
	LIST_HEAD(list);

	mxs_dma_enable_irq(mxs_i2c->dma_chan, 0);
	mxs_dma_get_cooked(mxs_i2c->dma_chan, &list);
	mxs_dma_disable(mxs_i2c->dma_chan);

	for (i = 0; i < NR_DESC; i++)
		mxs_dma_free_desc(desc[i]);

	hw_i2c_dmachan_reset(mxs_i2c);

	dma_free_coherent(&pdev->dev, PAGE_SIZE, i2c_buf_virt, i2c_buf_phys);

	mxs_dma_release(mxs_i2c->dma_chan, &pdev->dev);
}

static void hw_i2c_pioq_setup_read(struct mxs_i2c_dev *dev,
				   u8 addr, void *buff, int len, int flags)
{
	u32 queuecmd;
	u32 queuedata;

	WARN_ONCE(len > 24, "choose DMA mode if xfer len > 24 bytes\n");

	/* fill queue cmd */
	queuecmd = CMD_I2C_SELECT;
	__raw_writel(queuecmd, dev->regbase + HW_I2C_QUEUECMD);

	/* fill data (slave addr) */
	queuedata = (addr << 1) | I2C_READ;
	__raw_writel(queuedata, dev->regbase + HW_I2C_DATA);

	/* fill queue cmd */
	queuecmd = CMD_I2C_READ | flags;
	queuecmd |= BF_I2C_CTRL0_XFER_COUNT(len) | flags;
	__raw_writel(queuecmd, dev->regbase + HW_I2C_QUEUECMD);

}

static void hw_i2c_dma_setup_read(u8 addr, void *buff, int len, int flags)
{
	if (len > (PAGE_SIZE - 4))
		BUG();

	memset(&desc[0]->cmd, 0, sizeof(desc[0]->cmd));
	memset(&desc[1]->cmd, 0, sizeof(desc[1]->cmd));

	desc[0]->cmd.cmd.bits.bytes = 1;
	desc[0]->cmd.cmd.bits.pio_words = 1;
	desc[0]->cmd.cmd.bits.wait4end = 1;
	desc[0]->cmd.cmd.bits.dec_sem = 1;
	desc[0]->cmd.cmd.bits.irq = 0;
	desc[0]->cmd.cmd.bits.chain = 1;
	desc[0]->cmd.cmd.bits.command = DMA_READ;
	desc[0]->cmd.address = i2c_buf_phys;
	desc[0]->cmd.pio_words[0] = CMD_I2C_SELECT;
	i2c_buf_virt[0] = (addr << 1) | I2C_READ;

	desc[1]->cmd.cmd.bits.bytes = len;
	desc[1]->cmd.cmd.bits.pio_words = 1;
	desc[1]->cmd.cmd.bits.wait4end = 1;
	desc[1]->cmd.cmd.bits.dec_sem = 1;
	desc[1]->cmd.cmd.bits.irq = 1;
	desc[1]->cmd.cmd.bits.command = DMA_WRITE;
	desc[1]->cmd.address = (u32) i2c_buf_phys + 1;
	desc[1]->cmd.pio_words[0] = CMD_I2C_READ;
	desc[1]->cmd.pio_words[0] |= BF_I2C_CTRL0_XFER_COUNT(len) | flags;
}

static void hw_i2c_pioq_setup_write(struct mxs_i2c_dev *dev,
				    u8 addr, void *buff, int len, int flags)
{
	int align_len, i;
	u8 slaveaddr;
	u32 queuecmd;
	u8 *buf1;
	u32 *buf2;

	WARN_ONCE(len > 24, "choose DMA mode if xfer len > 24 bytes\n");

	align_len = (len + 1 + 3) & ~3;

	buf1 = (u8 *) dev->buf;
	buf2 = (u32 *) dev->buf;

	/* fill queue cmd */
	queuecmd = CMD_I2C_WRITE;
	queuecmd |= BF_I2C_CTRL0_XFER_COUNT(len + 1) | flags;
	__raw_writel(queuecmd, dev->regbase + HW_I2C_QUEUECMD);

	/* fill data (slave addr) */
	slaveaddr = (addr << 1) | I2C_WRITE;
	memcpy(buf1, &slaveaddr, 1);

	memcpy(&buf1[1], buff, len);

	/* fill data */
	for (i = 0; i < align_len / 4; i++)
		__raw_writel(*buf2++, dev->regbase + HW_I2C_DATA);
}

static void hw_i2c_dma_setup_write(u8 addr, void *buff, int len, int flags)
{
	memset(&desc[2]->cmd, 0, sizeof(desc[2]->cmd));

	desc[2]->cmd.cmd.bits.bytes = len + 1;
	desc[2]->cmd.cmd.bits.pio_words = 1;
	desc[2]->cmd.cmd.bits.wait4end = 1;
	desc[2]->cmd.cmd.bits.dec_sem = 1;
	desc[2]->cmd.cmd.bits.irq = 1;
	desc[2]->cmd.cmd.bits.command = DMA_READ;
	desc[2]->cmd.address = i2c_buf_phys;
	desc[2]->cmd.pio_words[0] = CMD_I2C_WRITE;
	desc[2]->cmd.pio_words[0] |= BM_I2C_CTRL0_POST_SEND_STOP;
	desc[2]->cmd.pio_words[0] |= BF_I2C_CTRL0_XFER_COUNT(len + 1) | flags;

	i2c_buf_virt[0] = (addr << 1) | I2C_WRITE;
	memcpy(&i2c_buf_virt[1], buff, len);
}

static void hw_i2c_pioq_run(struct mxs_i2c_dev *dev)
{
	__raw_writel(0x20, dev->regbase + HW_I2C_QUEUECTRL_SET);
}

static void hw_i2c_dma_run(struct mxs_i2c_dev *dev, int dir)
{
	if (dir == I2C_READ) {
		mxs_dma_desc_append(dev->dma_chan, desc[0]);
		mxs_dma_desc_append(dev->dma_chan, desc[1]);
	} else
		mxs_dma_desc_append(dev->dma_chan, desc[2]);

	mxs_dma_enable(dev->dma_chan);
}

static void hw_i2c_pioq_stop(struct mxs_i2c_dev *dev)
{
	__raw_writel(0x20, dev->regbase + HW_I2C_QUEUECTRL_CLR);
}

static void hw_i2c_finish_read(struct mxs_i2c_dev *dev, void *buff, int len)
{
	int i, align_len;
	u8 *buf1;
	u32 *buf2;

	if (dev->flags & MXS_I2C_PIOQUEUE_MODE) {
		align_len = (len + 3) & ~3;

		buf1 = (u8 *) dev->buf;
		buf2 = (u32 *) dev->buf;

		for (i = 0; i < align_len / 4; i++)
			*buf2++ = __raw_readl(dev->regbase + HW_I2C_QUEUEDATA);

		memcpy(buff, buf1, len);
	} else
		memcpy(buff, &i2c_buf_virt[1], len);
}

/*
 * Low level master read/write transaction.
 */
static int mxs_i2c_xfer_msg(struct i2c_adapter *adap,
			    struct i2c_msg *msg, int stop)
{
	struct mxs_i2c_dev *dev = i2c_get_adapdata(adap);
	int err;
	int flags;

	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		msg->addr, msg->len, msg->flags, stop);

	if ((msg->len == 0) || (msg->len > (PAGE_SIZE - 1)))
		return -EINVAL;

	flags = stop ? BM_I2C_CTRL0_POST_SEND_STOP : 0;

	if (msg->flags & I2C_M_RD) {
		if (dev->flags & MXS_I2C_PIOQUEUE_MODE) {
			hw_i2c_pioq_setup_read(dev,
					       msg->addr,
					       msg->buf, msg->len, flags);
			hw_i2c_pioq_run(dev);
		} else {
			hw_i2c_dma_setup_read(msg->addr,
					      msg->buf, msg->len, flags);

			hw_i2c_dma_run(dev, I2C_READ);
		}
	} else {
		if (dev->flags & MXS_I2C_PIOQUEUE_MODE) {
			hw_i2c_pioq_setup_write(dev,
						msg->addr,
						msg->buf, msg->len, flags);
			hw_i2c_pioq_run(dev);
		} else {
			hw_i2c_dma_setup_write(msg->addr,
					       msg->buf, msg->len, flags);
			hw_i2c_dma_run(dev, I2C_WRITE);
		}
	}

	err = wait_for_completion_interruptible_timeout(&dev->cmd_complete,
							msecs_to_jiffies(1000)
	    );
	if (err <= 0) {
		mxs_i2c_reset(dev);
		dev_dbg(dev->dev, "controller is timed out\n");
		return -ETIMEDOUT;
	}

	if ((!dev->cmd_err) && (msg->flags & I2C_M_RD))
		hw_i2c_finish_read(dev, msg->buf, msg->len);

	dev_dbg(dev->dev, "Done with err=%d\n", dev->cmd_err);

	return dev->cmd_err;
}

static int
mxs_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int err;

	if (!msgs->len)
		return -EINVAL;

	for (i = 0; i < num; i++) {
		err = mxs_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
		if (err)
			break;
	}

	if (err == 0)
		err = num;

	return err;
}

static u32 mxs_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static irqreturn_t mxs_i2c_dma_isr(int this_irq, void *dev_id)
{
	struct mxs_i2c_dev *mxs_i2c = dev_id;

	LIST_HEAD(list);
	mxs_dma_ack_irq(mxs_i2c->dma_chan);
	mxs_dma_cooked(mxs_i2c->dma_chan, &list);

	complete(&mxs_i2c->cmd_complete);

	return IRQ_HANDLED;
}

static void mxs_i2c_task(struct work_struct *work)
{
	struct mxs_i2c_dev *mxs_i2c = container_of(work,
					struct mxs_i2c_dev, work);
	mxs_i2c_reset(mxs_i2c);
	complete(&mxs_i2c->cmd_complete);
}

#define I2C_IRQ_MASK 0x000000FF
static irqreturn_t mxs_i2c_isr(int this_irq, void *dev_id)
{
	struct mxs_i2c_dev *mxs_i2c = dev_id;
	u32 stat;
	u32 done_mask =
	    BM_I2C_CTRL1_DATA_ENGINE_CMPLT_IRQ | BM_I2C_CTRL1_BUS_FREE_IRQ;

	stat = __raw_readl(mxs_i2c->regbase + HW_I2C_CTRL1) & I2C_IRQ_MASK;
	if (!stat)
		return IRQ_NONE;

	if (stat & BM_I2C_CTRL1_NO_SLAVE_ACK_IRQ) {
		mxs_i2c->cmd_err = -EREMOTEIO;
		/* it takes long time to reset i2c */
		schedule_work(&mxs_i2c->work);
		goto done;
	}

	/* Don't care about BM_I2C_CTRL1_OVERSIZE_XFER_TERM_IRQ */
	if (stat & (BM_I2C_CTRL1_EARLY_TERM_IRQ |
		    BM_I2C_CTRL1_MASTER_LOSS_IRQ |
		    BM_I2C_CTRL1_SLAVE_STOP_IRQ | BM_I2C_CTRL1_SLAVE_IRQ)) {
		mxs_i2c->cmd_err = -EIO;
		complete(&mxs_i2c->cmd_complete);
		goto done;
	}

	if ((stat & done_mask) == done_mask &&
		(mxs_i2c->flags & MXS_I2C_PIOQUEUE_MODE))

		complete(&mxs_i2c->cmd_complete);

done:
	__raw_writel(stat, mxs_i2c->regbase + HW_I2C_CTRL1_CLR);
	return IRQ_HANDLED;
}

static const struct i2c_algorithm mxs_i2c_algo = {
	.master_xfer = mxs_i2c_xfer,
	.functionality = mxs_i2c_func,
};

static int mxs_i2c_probe(struct platform_device *pdev)
{
	struct mxs_i2c_dev *mxs_i2c;
	struct mxs_i2c_plat_data *pdata;
	struct i2c_adapter *adap;
	struct resource *res;
	int err = 0;

	mxs_i2c = kzalloc(sizeof(struct mxs_i2c_dev), GFP_KERNEL);
	if (!mxs_i2c) {
		dev_err(&pdev->dev, "no mem \n");
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no register base resource\n");
		err = -ENODEV;
		goto nores;
	}
	mxs_i2c->regbase = (unsigned long)IO_ADDRESS(res->start);

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res) {
		dev_err(&pdev->dev, "no dma channel resource\n");
		err = -ENODEV;
		goto nores;
	}
	mxs_i2c->dma_chan = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no err_irq resource\n");
		err = -ENODEV;
		goto nores;
	}
	mxs_i2c->irq_err = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(&pdev->dev, "no dma_irq resource\n");
		err = -ENODEV;
		goto nores;
	}
	mxs_i2c->irq_dma = res->start;

	mxs_i2c->dev = &pdev->dev;
	mxs_i2c->flags = pdata->pioqueue_mode ?
	    MXS_I2C_PIOQUEUE_MODE : MXS_I2C_DMA_MODE;

	err =
	    request_irq(mxs_i2c->irq_err, mxs_i2c_isr, 0, pdev->name, mxs_i2c);
	if (err) {
		dev_err(&pdev->dev, "Can't get IRQ\n");
		goto no_err_irq;
	}

	err =
	    request_irq(mxs_i2c->irq_dma, mxs_i2c_dma_isr, 0, pdev->name,
			mxs_i2c);
	if (err) {
		dev_err(&pdev->dev, "Can't get IRQ\n");
		goto no_dma_irq;
	}

	/* reset I2C module */
	mxs_reset_block((void __iomem *)mxs_i2c->regbase, 1);
	platform_set_drvdata(pdev, mxs_i2c);

	if (mxs_i2c->flags & MXS_I2C_PIOQUEUE_MODE)
		__raw_writel(0x04, mxs_i2c->regbase + HW_I2C_QUEUECTRL_SET);

	mxs_i2c->buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (mxs_i2c->buf == NULL) {
		dev_err(&pdev->dev, "HW Init failed\n");
		goto init_failed;
	} else {
		err = hw_i2c_dma_init(pdev);
		if (err) {
			dev_err(&pdev->dev, "HW Init failed\n");
			goto init_failed;
		}
	}

	/* Will catch all error (IRQ mask) */
	__raw_writel(0x0000FF00, mxs_i2c->regbase + HW_I2C_CTRL1_SET);

	adap = &mxs_i2c->adapter;
	strncpy(adap->name, "MXS I2C adapter", sizeof(adap->name));
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->algo = &mxs_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	i2c_set_adapdata(adap, mxs_i2c);
	err = i2c_add_numbered_adapter(adap);
	if (err) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		goto no_i2c_adapter;

	}

	INIT_WORK(&mxs_i2c->work, mxs_i2c_task);

	return 0;

no_i2c_adapter:
	__raw_writel(BM_I2C_CTRL0_SFTRST, mxs_i2c->regbase + HW_I2C_CTRL0_SET);

	if (mxs_i2c->flags & MXS_I2C_DMA_MODE)
		hw_i2c_dma_uninit(pdev);
	else
		kfree(mxs_i2c->buf);
init_failed:
	free_irq(mxs_i2c->irq_dma, mxs_i2c);
no_dma_irq:
	free_irq(mxs_i2c->irq_err, mxs_i2c);
no_err_irq:
nores:
	kfree(mxs_i2c);
	return err;
}

static int mxs_i2c_remove(struct platform_device *pdev)
{
	struct mxs_i2c_dev *mxs_i2c = platform_get_drvdata(pdev);
	int res;

	res = i2c_del_adapter(&mxs_i2c->adapter);
	if (res)
		return -EBUSY;

	__raw_writel(BM_I2C_CTRL0_SFTRST, mxs_i2c->regbase + HW_I2C_CTRL0_SET);

	if (mxs_i2c->flags & MXS_I2C_DMA_MODE)
		hw_i2c_dma_uninit(pdev);
	if (mxs_i2c->flags & MXS_I2C_PIOQUEUE_MODE)
		hw_i2c_pioq_stop(mxs_i2c);

	platform_set_drvdata(pdev, NULL);

	free_irq(mxs_i2c->irq_err, mxs_i2c);
	free_irq(mxs_i2c->irq_dma, mxs_i2c);

	kfree(mxs_i2c->buf);
	kfree(mxs_i2c);
	return 0;
}

static struct platform_driver mxs_i2c_driver = {
	.driver = {
		   .name = "mxs-i2c",
		   .owner = THIS_MODULE,
		   },
	.probe = mxs_i2c_probe,
	.remove = __devexit_p(mxs_i2c_remove),
};

static int __init mxs_i2c_init(void)
{
	return platform_driver_register(&mxs_i2c_driver);
}

subsys_initcall(mxs_i2c_init);

static void __exit mxs_i2c_exit(void)
{
	platform_driver_unregister(&mxs_i2c_driver);
}

module_exit(mxs_i2c_exit);

MODULE_AUTHOR("Embedded Alley Solutions, Inc/Freescale Inc");
MODULE_DESCRIPTION("MXS I2C Bus Driver");
MODULE_LICENSE("GPL");
