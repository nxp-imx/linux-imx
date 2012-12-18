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

#include <mach/regs-apbh.h>
#ifndef BM_APBH_CTRL0_APB_BURST_EN
#define BM_APBH_CTRL0_APB_BURST_EN BM_APBH_CTRL0_APB_BURST4_EN
#endif

static int mxs_dma_apbh_enable(struct mxs_dma_chan *pchan, unsigned int chan)
{
	unsigned int sem;
	struct mxs_dma_device *pdev = pchan->dma;
	struct mxs_dma_desc *pdesc;

	pdesc = list_first_entry(&pchan->active, struct mxs_dma_desc, node);
	if (pdesc == NULL)
		return -EFAULT;

	sem = __raw_readl(pdev->base + HW_APBH_CHn_SEMA(chan));
	sem = (sem & BM_APBH_CHn_SEMA_PHORE) >> BP_APBH_CHn_SEMA_PHORE;
	if (pchan->flags & MXS_DMA_FLAGS_BUSY) {
		if (pdesc->cmd.cmd.bits.chain == 0)
			return 0;
		if (sem < 2) {
			if (!sem)
				return 0;
			pdesc = list_entry(pdesc->node.next,
					   struct mxs_dma_desc, node);
			__raw_writel(mxs_dma_cmd_address(pdesc),
				     pdev->base + HW_APBH_CHn_NXTCMDAR(chan));
		}
		sem = pchan->pending_num;
		pchan->pending_num = 0;
		__raw_writel(BF_APBH_CHn_SEMA_INCREMENT_SEMA(sem),
			     pdev->base + HW_APBH_CHn_SEMA(chan));
		pchan->active_num += sem;
		return 0;
	}
	pchan->active_num += pchan->pending_num;
	pchan->pending_num = 0;
	__raw_writel(mxs_dma_cmd_address(pdesc),
		     pdev->base + HW_APBH_CHn_NXTCMDAR(chan));
	__raw_writel(pchan->active_num, pdev->base + HW_APBH_CHn_SEMA(chan));
	__raw_writel(1 << chan, pdev->base + HW_APBH_CTRL0_CLR);
	return 0;
}

static void mxs_dma_apbh_disable(struct mxs_dma_chan *pchan, unsigned int chan)
{
	struct mxs_dma_device *pdev = pchan->dma;
	__raw_writel(1 << (chan + BP_APBH_CTRL0_CLKGATE_CHANNEL),
		pdev->base + HW_APBH_CTRL0_SET);
}

static void mxs_dma_apbh_reset(struct mxs_dma_device *pdev, unsigned int chan)
{
#ifdef CONFIG_ARCH_MX28
	__raw_writel(1 << (chan + BP_APBH_CHANNEL_CTRL_RESET_CHANNEL),
		     pdev->base + HW_APBH_CHANNEL_CTRL_SET);
#endif

#ifdef CONFIG_ARCH_MX23
	__raw_writel(1 << (chan + BP_APBH_CTRL0_RESET_CHANNEL),
				pdev->base + HW_APBH_CTRL0_SET);
#endif
}

static void mxs_dma_apbh_freeze(struct mxs_dma_device *pdev, unsigned int chan)
{
#ifdef CONFIG_ARCH_MX28
	__raw_writel(1 << chan, pdev->base + HW_APBH_CHANNEL_CTRL_SET);
#endif

#ifdef CONFIG_ARCH_MX23
	__raw_writel(1 << (chan + BP_APBH_CTRL0_FREEZE_CHANNEL),
				pdev->base + HW_APBH_CTRL0_SET);
#endif
}

static void
mxs_dma_apbh_unfreeze(struct mxs_dma_device *pdev, unsigned int chan)
{
#ifdef CONFIG_ARCH_MX28
	__raw_writel(1 << chan, pdev->base + HW_APBH_CHANNEL_CTRL_CLR);
#endif

#ifdef CONFIG_ARCH_MX23
	__raw_writel(1 << (chan + BP_APBH_CTRL0_FREEZE_CHANNEL),
				pdev->base + HW_APBH_CTRL0_CLR);
#endif

}

static void mxs_dma_apbh_info(struct mxs_dma_device *pdev,
		unsigned int chan, struct mxs_dma_info *info)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBH_CTRL2);
	info->status = reg >> chan;
	info->buf_addr = __raw_readl(pdev->base + HW_APBH_CHn_BAR(chan));
}

static int
mxs_dma_apbh_read_semaphore(struct mxs_dma_device *pdev, unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBH_CHn_SEMA(chan));
	return (reg & BM_APBH_CHn_SEMA_PHORE) >> BP_APBH_CHn_SEMA_PHORE;
}

static void
mxs_dma_apbh_enable_irq(struct mxs_dma_device *pdev,
			unsigned int chan, int enable)
{
	if (enable) {
		__raw_writel(1 << (chan + 16), pdev->base + HW_APBH_CTRL1_SET);
	} else {
		__raw_writel(1 << (chan + 16), pdev->base + HW_APBH_CTRL1_CLR);
	}
}

static int
mxs_dma_apbh_irq_is_pending(struct mxs_dma_device *pdev, unsigned int chan)
{
	unsigned int reg;
	reg = __raw_readl(pdev->base + HW_APBH_CTRL1);
	reg |= __raw_readl(pdev->base + HW_APBH_CTRL2);
	return reg & (1 << chan);
}

static void mxs_dma_apbh_ack_irq(struct mxs_dma_device *pdev, unsigned int chan)
{
	__raw_writel(1 << chan, pdev->base + HW_APBH_CTRL1_CLR);
	__raw_writel(1 << chan, pdev->base + HW_APBH_CTRL2_CLR);
}

static struct mxs_dma_device mxs_dma_apbh = {
	.name = "mxs-dma-apbh",
	.enable = mxs_dma_apbh_enable,
	.disable = mxs_dma_apbh_disable,
	.reset = mxs_dma_apbh_reset,
	.freeze = mxs_dma_apbh_freeze,
	.unfreeze = mxs_dma_apbh_unfreeze,
	.info = mxs_dma_apbh_info,
	.read_semaphore = mxs_dma_apbh_read_semaphore,
	.enable_irq = mxs_dma_apbh_enable_irq,
	.irq_is_pending = mxs_dma_apbh_irq_is_pending,
	.ack_irq = mxs_dma_apbh_ack_irq,
};

static int __devinit dma_apbh_probe(struct platform_device *pdev)
{
	int i;
	struct resource *res;
	struct mxs_dma_plat_data *plat;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;
	mxs_dma_apbh.base = IO_ADDRESS(res->start);
	__raw_writel(BM_APBH_CTRL0_SFTRST,
		     mxs_dma_apbh.base + HW_APBH_CTRL0_CLR);
	for (i = 0; i < 10000; i++) {
		if (!(__raw_readl(mxs_dma_apbh.base + HW_APBH_CTRL0_CLR) &
		      BM_APBH_CTRL0_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return -ETIME;
	__raw_writel(BM_APBH_CTRL0_CLKGATE,
		     mxs_dma_apbh.base + HW_APBH_CTRL0_CLR);

	plat = (struct mxs_dma_plat_data *)pdev->dev.platform_data;
	if (!plat)
		return -ENODEV;
	if (plat->burst8)
		__raw_writel(BM_APBH_CTRL0_AHB_BURST8_EN,
			     mxs_dma_apbh.base + HW_APBH_CTRL0_SET);
	else
		__raw_writel(BM_APBH_CTRL0_AHB_BURST8_EN,
			     mxs_dma_apbh.base + HW_APBH_CTRL0_CLR);

	if (plat->burst)
		__raw_writel(BM_APBH_CTRL0_APB_BURST_EN,
			     mxs_dma_apbh.base + HW_APBH_CTRL0_SET);
	else
		__raw_writel(BM_APBH_CTRL0_APB_BURST_EN,
			     mxs_dma_apbh.base + HW_APBH_CTRL0_CLR);

	mxs_dma_apbh.pdev = pdev;
	mxs_dma_apbh.chan_base = plat->chan_base;
	mxs_dma_apbh.chan_num = plat->chan_num;
	platform_set_drvdata(pdev, &mxs_dma_apbh);
	return mxs_dma_device_register(&mxs_dma_apbh);
}

static int __devexit dma_apbh_remove(struct platform_device *pdev)
{
	return 0;
}

int dma_apbh_suspend(void)
{
	__raw_writel(BM_APBH_CTRL0_CLKGATE, IO_ADDRESS(APBH_DMA_PHYS_ADDR)
			+ HW_APBH_CTRL0_SET);

  return 0;

}
EXPORT_SYMBOL(dma_apbh_suspend);

int dma_apbh_resume(void)
{
	int i;

	__raw_writel(BM_APBH_CTRL0_SFTRST,
			IO_ADDRESS(APBH_DMA_PHYS_ADDR) + HW_APBH_CTRL0_CLR);
	for (i = 0; i < 10000; i++) {
		if (!(__raw_readl(IO_ADDRESS(APBH_DMA_PHYS_ADDR) + HW_APBH_CTRL0_CLR) &
		      BM_APBH_CTRL0_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return -ETIME;

	__raw_writel(BM_APBH_CTRL0_CLKGATE, IO_ADDRESS(APBH_DMA_PHYS_ADDR) + HW_APBH_CTRL0_CLR);


	__raw_writel(BM_APBH_CTRL0_AHB_BURST8_EN, IO_ADDRESS(APBH_DMA_PHYS_ADDR) + HW_APBH_CTRL0_CLR);
	__raw_writel(BM_APBH_CTRL0_APB_BURST_EN,	IO_ADDRESS(APBH_DMA_PHYS_ADDR) + HW_APBH_CTRL0_CLR);

  return 0;
}
EXPORT_SYMBOL(dma_apbh_resume);


static struct platform_driver dma_apbh_driver = {
	.probe = dma_apbh_probe,
	.remove = __devexit_p(dma_apbh_remove),
	.driver = {
		   .name = "mxs-dma-apbh"},
};

static int __init mxs_dma_apbh_init(void)
{
	return platform_driver_register(&dma_apbh_driver);
}

fs_initcall(mxs_dma_apbh_init);
