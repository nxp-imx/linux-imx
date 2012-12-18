/*
 * Copyright (C) 2009-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/err.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/dmaengine.h>

#include "regs-apbx.h"

static int mxs_dma_apbx_enable(struct mxs_dma_chan *pchan, unsigned int chan)
{
	unsigned int sem;
	struct mxs_dma_device *pdev = pchan->dma;
	struct mxs_dma_desc *pdesc;


	pdesc = list_first_entry(&pchan->active, struct mxs_dma_desc, node);
	if (pdesc == NULL)
		return -EFAULT;
	sem = __raw_readl(pdev->base + HW_APBX_CHn_SEMA(chan));
	sem = (sem & BM_APBX_CHn_SEMA_PHORE) >> BP_APBX_CHn_SEMA_PHORE;
	if (pchan->flags & MXS_DMA_FLAGS_BUSY) {
		if (pdesc->cmd.cmd.bits.chain == 0)
			return 0;
		if (sem < 2) {
			if (!sem)
				return 0;
			pdesc = list_entry(pdesc->node.next,
					   struct mxs_dma_desc, node);
			__raw_writel(mxs_dma_cmd_address(pdesc),
				     pdev->base + HW_APBX_CHn_NXTCMDAR(chan));
		}
		sem = pchan->pending_num;
		pchan->pending_num = 0;
		__raw_writel(BF_APBX_CHn_SEMA_INCREMENT_SEMA(sem),
			     pdev->base + HW_APBX_CHn_SEMA(chan));
		pchan->active_num += sem;
		return 0;
	}
	pchan->active_num += pchan->pending_num;
	pchan->pending_num = 0;
	__raw_writel(mxs_dma_cmd_address(pdesc),
		     pdev->base + HW_APBX_CHn_NXTCMDAR(chan));
	__raw_writel(pchan->active_num, pdev->base + HW_APBX_CHn_SEMA(chan));


	return 0;
}

static void mxs_dma_apbx_disable(struct mxs_dma_chan *pchan, unsigned int chan)
{
	struct mxs_dma_device *pdev = pchan->dma;
	__raw_writel(0, pdev->base + HW_APBX_CHn_SEMA(chan));
}

static void mxs_dma_apbx_reset(struct mxs_dma_device *pdev, unsigned int chan)
{
	__raw_writel(1 << (chan + BP_APBX_CHANNEL_CTRL_RESET_CHANNEL),
		     pdev->base + HW_APBX_CHANNEL_CTRL_SET);
}

static void mxs_dma_apbx_freeze(struct mxs_dma_device *pdev, unsigned int chan)
{
	__raw_writel(1 << chan, pdev->base + HW_APBX_CHANNEL_CTRL_SET);
}

static void
mxs_dma_apbx_unfreeze(struct mxs_dma_device *pdev, unsigned int chan)
{
	__raw_writel(1 << chan, pdev->base + HW_APBX_CHANNEL_CTRL_CLR);
}

static void mxs_dma_apbx_info(struct mxs_dma_device *pdev,
		unsigned int chan, struct mxs_dma_info *info)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBX_CTRL2);
	info->status = reg >> chan;
	info->buf_addr = __raw_readl(pdev->base + HW_APBX_CHn_BAR(chan));
	reg = __raw_readl(pdev->base + HW_APBX_CHn_CMD(chan));
	info->xfer_count = (reg & BM_APBX_CHn_CMD_XFER_COUNT) >> \
		BP_APBX_CHn_CMD_XFER_COUNT;
}

static int
mxs_dma_apbx_read_semaphore(struct mxs_dma_device *pdev, unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBX_CHn_SEMA(chan));
	return (reg & BM_APBX_CHn_SEMA_PHORE) >> BP_APBX_CHn_SEMA_PHORE;
}

static void
mxs_dma_apbx_enable_irq(struct mxs_dma_device *pdev,
			unsigned int chan, int enable)
{
	if (enable) {
		__raw_writel(1 << (chan + 16), pdev->base + HW_APBX_CTRL1_SET);
	} else {
		__raw_writel(1 << (chan + 16), pdev->base + HW_APBX_CTRL1_CLR);
	}
}

static int
mxs_dma_apbx_irq_is_pending(struct mxs_dma_device *pdev, unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBX_CTRL1);
	reg |= __raw_readl(pdev->base + HW_APBX_CTRL2);
	return reg & (1 << chan);
}

static void mxs_dma_apbx_ack_irq(struct mxs_dma_device *pdev, unsigned int chan)
{
	__raw_writel(1 << chan, pdev->base + HW_APBX_CTRL1_CLR);
	__raw_writel(1 << chan, pdev->base + HW_APBX_CTRL2_CLR);
}

static struct mxs_dma_device mxs_dma_apbx = {
	.name = "mxs-dma-apbx",
	.enable = mxs_dma_apbx_enable,
	.disable = mxs_dma_apbx_disable,
	.reset = mxs_dma_apbx_reset,
	.freeze = mxs_dma_apbx_freeze,
	.unfreeze = mxs_dma_apbx_unfreeze,
	.info = mxs_dma_apbx_info,
	.read_semaphore = mxs_dma_apbx_read_semaphore,
	.enable_irq = mxs_dma_apbx_enable_irq,
	.irq_is_pending = mxs_dma_apbx_irq_is_pending,
	.ack_irq = mxs_dma_apbx_ack_irq,
};

static int __devinit dma_apbx_probe(struct platform_device *pdev)
{
	int i;
	struct resource *res;
	struct mxs_dma_plat_data *plat;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;
	mxs_dma_apbx.base = IO_ADDRESS(res->start);
	__raw_writel(BM_APBX_CTRL0_SFTRST,
		     mxs_dma_apbx.base + HW_APBX_CTRL0_CLR);
	for (i = 0; i < 10000; i++) {
		if (!(__raw_readl(mxs_dma_apbx.base + HW_APBX_CTRL0_CLR) &
		      BM_APBX_CTRL0_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return -ETIME;
	__raw_writel(BM_APBX_CTRL0_CLKGATE,
		     mxs_dma_apbx.base + HW_APBX_CTRL0_CLR);

	plat = (struct mxs_dma_plat_data *)pdev->dev.platform_data;
	if (!plat)
		return -ENODEV;

	mxs_dma_apbx.pdev = pdev;
	mxs_dma_apbx.chan_base = plat->chan_base;
	mxs_dma_apbx.chan_num = plat->chan_num;
	platform_set_drvdata(pdev, &mxs_dma_apbx);
	return mxs_dma_device_register(&mxs_dma_apbx);
}

static int __devexit dma_apbx_remove(struct platform_device *pdev)
{
	return 0;
}

int dma_apbx_suspend(void)
{
	__raw_writel(BM_APBX_CTRL0_CLKGATE, IO_ADDRESS(APBX_DMA_PHYS_ADDR) + HW_APBX_CTRL0_SET);

  return 0;
}
EXPORT_SYMBOL(dma_apbx_suspend);

int dma_apbx_resume(void)
{
	int i;

	__raw_writel(BM_APBX_CTRL0_SFTRST, IO_ADDRESS(APBX_DMA_PHYS_ADDR) + HW_APBX_CTRL0_CLR);
	for (i = 0; i < 10000; i++) {
		if (!(__raw_readl(IO_ADDRESS(APBX_DMA_PHYS_ADDR) + HW_APBX_CTRL0_CLR) & BM_APBX_CTRL0_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return -ETIME;
	__raw_writel(BM_APBX_CTRL0_CLKGATE, IO_ADDRESS(APBX_DMA_PHYS_ADDR) + HW_APBX_CTRL0_CLR);

  return 0;
}
EXPORT_SYMBOL(dma_apbx_resume);


static struct platform_driver dma_apbx_driver = {
	.probe = dma_apbx_probe,
	.remove = __devexit_p(dma_apbx_remove),
	.driver = {
		   .name = "mxs-dma-apbx"},
};

static int __init mxs_dma_apbx_init(void)
{
	return platform_driver_register(&dma_apbx_driver);
}

fs_initcall(mxs_dma_apbx_init);
