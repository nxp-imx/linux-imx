/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file drv.c
 *
 * @brief Driver for Freescale CAN Controller FlexCAN.
 *
 * @ingroup can
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/random.h>
#include "flexcan.h"

static void flexcan_hw_start(struct flexcan_device *flexcan)
{
	unsigned int reg;
	if ((flexcan->maxmb + 1) > 32) {
		__raw_writel(0xFFFFFFFF, flexcan->io_base + CAN_HW_REG_IMASK1);
		reg = (1 << (flexcan->maxmb - 31)) - 1;
		__raw_writel(reg, flexcan->io_base + CAN_HW_REG_IMASK2);
	} else {
		reg = (1 << (flexcan->maxmb + 1)) - 1;
		__raw_writel(reg, flexcan->io_base + CAN_HW_REG_IMASK1);
		__raw_writel(0, flexcan->io_base + CAN_HW_REG_IMASK2);
	}

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR) & (~__MCR_HALT);
	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_MCR);
}

static void flexcan_hw_stop(struct flexcan_device *flexcan)
{
	unsigned int reg;
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	__raw_writel(reg | __MCR_HALT, flexcan->io_base + CAN_HW_REG_MCR);
}

static int flexcan_hw_reset(struct flexcan_device *flexcan)
{
	unsigned int reg;
	int timeout = 100000;

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	__raw_writel(reg | __MCR_MDIS, flexcan->io_base + CAN_HW_REG_MCR);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_CTRL);
	if (flexcan->br_clksrc)
		reg |= __CTRL_CLK_SRC;
	else
		reg &= ~__CTRL_CLK_SRC;
	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_CTRL);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR) & (~__MCR_MDIS);
	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_MCR);
	reg |= __MCR_SOFT_RST;
	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_MCR);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	while (reg & __MCR_SOFT_RST) {
		if (--timeout <= 0) {
			printk(KERN_ERR "Flexcan software Reset Timeouted\n");
			return -1;
		}
		udelay(10);
		reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	}
	return 0;
}

static inline void flexcan_mcr_setup(struct flexcan_device *flexcan)
{
	unsigned int reg;

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	reg &= ~(__MCR_MAX_MB_MASK | __MCR_WAK_MSK | __MCR_MAX_IDAM_MASK);

	if (flexcan->fifo)
		reg |= __MCR_FEN;
	else
		reg &= ~__MCR_FEN;

	if (flexcan->wakeup)
		reg |= __MCR_SLF_WAK | __MCR_WAK_MSK;
	else
		reg &= ~(__MCR_SLF_WAK | __MCR_WAK_MSK);

	if (flexcan->wak_src)
		reg |= __MCR_WAK_SRC;
	else
		reg &= ~__MCR_WAK_SRC;

	if (flexcan->srx_dis)
		reg |= __MCR_SRX_DIS;
	else
		reg &= ~__MCR_SRX_DIS;

	if (flexcan->bcc)
		reg |= __MCR_BCC;
	else
		reg &= ~__MCR_BCC;

	if (flexcan->lprio)
		reg |= __MCR_LPRIO_EN;
	else
		reg &= ~__MCR_LPRIO_EN;

	if (flexcan->abort)
		reg |= __MCR_AEN;
	else
		reg &= ~__MCR_AEN;

	reg |= (flexcan->maxmb << __MCR_MAX_MB_OFFSET);
	reg |= __MCR_DOZE | __MCR_MAX_IDAM_C;
	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_MCR);
}

static inline void flexcan_ctrl_setup(struct flexcan_device *flexcan)
{
	unsigned int reg;

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_CTRL);
	reg &= ~(__CTRL_PRESDIV_MASK | __CTRL_RJW_MASK | __CTRL_PSEG1_MASK |
		 __CTRL_PSEG2_MASK | __CTRL_PROPSEG_MASK);

	if (flexcan->loopback)
		reg |= __CTRL_LPB;
	else
		reg &= ~__CTRL_LPB;

	if (flexcan->smp)
		reg |= __CTRL_SMP;
	else
		reg &= ~__CTRL_SMP;

	if (flexcan->boff_rec)
		reg |= __CTRL_BOFF_REC;
	else
		reg &= ~__CTRL_BOFF_REC;

	if (flexcan->tsyn)
		reg |= __CTRL_TSYN;
	else
		reg &= ~__CTRL_TSYN;

	if (flexcan->listen)
		reg |= __CTRL_LOM;
	else
		reg &= ~__CTRL_LOM;

	reg |= (flexcan->br_presdiv << __CTRL_PRESDIV_OFFSET) |
	    (flexcan->br_rjw << __CTRL_RJW_OFFSET) |
	    (flexcan->br_pseg1 << __CTRL_PSEG1_OFFSET) |
	    (flexcan->br_pseg2 << __CTRL_PSEG2_OFFSET) |
	    (flexcan->br_propseg << __CTRL_PROPSEG_OFFSET);

	reg &= ~__CTRL_LBUF;

	reg |= __CTRL_TWRN_MSK | __CTRL_RWRN_MSK | __CTRL_BOFF_MSK |
	    __CTRL_ERR_MSK;

	__raw_writel(reg, flexcan->io_base + CAN_HW_REG_CTRL);
}

static int flexcan_hw_restart(struct net_device *dev)
{
	unsigned int reg;
	struct flexcan_device *flexcan = netdev_priv(dev);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	if (reg & __MCR_SOFT_RST)
		return 1;

	flexcan_mcr_setup(flexcan);

	__raw_writel(0, flexcan->io_base + CAN_HW_REG_IMASK2);
	__raw_writel(0, flexcan->io_base + CAN_HW_REG_IMASK1);

	__raw_writel(0xFFFFFFFF, flexcan->io_base + CAN_HW_REG_IFLAG2);
	__raw_writel(0xFFFFFFFF, flexcan->io_base + CAN_HW_REG_IFLAG1);

	__raw_writel(0, flexcan->io_base + CAN_HW_REG_ECR);

	flexcan_mbm_init(flexcan);
	netif_carrier_on(dev);
	flexcan_hw_start(flexcan);

	if (netif_queue_stopped(dev))
		netif_start_queue(dev);

	return 0;
}

static void flexcan_hw_watch(unsigned long data)
{
	unsigned int reg, ecr;
	struct net_device *dev = (struct net_device *)data;
	struct flexcan_device *flexcan = dev ? netdev_priv(dev) : NULL;

	BUG_ON(!flexcan);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	if (reg & __MCR_MDIS) {
		if (flexcan_hw_restart(dev))
			mod_timer(&flexcan->timer, HZ / 20);
		return;
	}
	ecr = __raw_readl(flexcan->io_base + CAN_HW_REG_ECR);
	if (flexcan->boff_rec) {
		if (((reg & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) > 1) {
			reg |= __MCR_SOFT_RST;
			__raw_writel(reg, flexcan->io_base + CAN_HW_REG_MCR);
			mod_timer(&flexcan->timer, HZ / 20);
			return;
		}
		netif_carrier_on(dev);
	}
}

static void flexcan_hw_busoff(struct net_device *dev)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	unsigned int reg;

	netif_carrier_off(dev);

	flexcan->timer.function = flexcan_hw_watch;
	flexcan->timer.data = (unsigned long)dev;

	if (flexcan->boff_rec) {
		mod_timer(&flexcan->timer, HZ / 10);
		return;
	}
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	__raw_writel(reg | __MCR_SOFT_RST, flexcan->io_base + CAN_HW_REG_MCR);
	mod_timer(&flexcan->timer, HZ / 20);
}

static int flexcan_hw_open(struct flexcan_device *flexcan)
{
	if (flexcan_hw_reset(flexcan))
		return -EFAULT;

	flexcan_mcr_setup(flexcan);
	flexcan_ctrl_setup(flexcan);

	__raw_writel(0, flexcan->io_base + CAN_HW_REG_IMASK2);
	__raw_writel(0, flexcan->io_base + CAN_HW_REG_IMASK1);

	__raw_writel(0xFFFFFFFF, flexcan->io_base + CAN_HW_REG_IFLAG2);
	__raw_writel(0xFFFFFFFF, flexcan->io_base + CAN_HW_REG_IFLAG1);

	__raw_writel(0, flexcan->io_base + CAN_HW_REG_ECR);
	return 0;
}

static void flexcan_err_handler(struct net_device *dev)
{
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct sk_buff *skb;
	struct can_frame *frame;
	unsigned int esr, ecr;

	esr = __raw_readl(flexcan->io_base + CAN_HW_REG_ESR);
	__raw_writel(esr & __ESR_INTERRUPTS, flexcan->io_base + CAN_HW_REG_ESR);

	if (esr & __ESR_WAK_INT)
		return;

	skb = dev_alloc_skb(sizeof(struct can_frame));
	if (!skb) {
		printk(KERN_ERR "%s: allocates skb fail in\n", __func__);
		return;
	}
	frame = (struct can_frame *)skb_put(skb, sizeof(*frame));
	memset(frame, 0, sizeof(*frame));
	frame->can_id = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;

	if (esr & __ESR_TWRN_INT)
		frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;

	if (esr & __ESR_RWRN_INT)
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;

	if (esr & __ESR_BOFF_INT)
		frame->can_id |= CAN_ERR_BUSOFF;

	if (esr & __ESR_ERR_INT) {
		if (esr & __ESR_BIT1_ERR)
			frame->data[2] |= CAN_ERR_PROT_BIT1;

		if (esr & __ESR_BIT0_ERR)
			frame->data[2] |= CAN_ERR_PROT_BIT0;

		if (esr & __ESR_ACK_ERR)
			frame->can_id |= CAN_ERR_ACK;

		/*TODO:// if (esr & __ESR_CRC_ERR) */

		if (esr & __ESR_FRM_ERR)
			frame->data[2] |= CAN_ERR_PROT_FORM;

		if (esr & __ESR_STF_ERR)
			frame->data[2] |= CAN_ERR_PROT_STUFF;

		ecr = __raw_readl(flexcan->io_base + CAN_HW_REG_ECR);
		switch ((esr & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) {
		case 0:
			if (__ECR_TX_ERR_COUNTER(ecr) >= __ECR_ACTIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
			if (__ECR_RX_ERR_COUNTER(ecr) >= __ECR_ACTIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
			break;
		case 1:
			if (__ECR_TX_ERR_COUNTER(ecr) >=
			    __ECR_PASSIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

			if (__ECR_RX_ERR_COUNTER(ecr) >=
			    __ECR_PASSIVE_THRESHOLD)
				frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
			break;
		default:
			frame->can_id |= CAN_ERR_BUSOFF;
		}
	}

	if (frame->can_id & CAN_ERR_BUSOFF)
		flexcan_hw_busoff(dev);

	skb->dev = dev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	netif_receive_skb(skb);
}

static irqreturn_t flexcan_irq_handler(int irq, void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct flexcan_device *flexcan = dev ? netdev_priv(dev) : NULL;
	unsigned int reg;

	BUG_ON(!flexcan);

	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_ESR);
	if (reg & __ESR_INTERRUPTS) {
		flexcan_err_handler(dev);
		return IRQ_HANDLED;
	}

	flexcan_mbm_isr(dev);
	return IRQ_HANDLED;
}

static int flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct flexcan_device *flexcan = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;

	BUG_ON(!flexcan);

	if (frame->can_dlc > 8)
		return -EINVAL;

	if (!flexcan_mbm_xmit(flexcan, frame)) {
		dev_kfree_skb(skb);
		stats->tx_bytes += frame->can_dlc;
		stats->tx_packets++;
		dev->trans_start = jiffies;
		return NETDEV_TX_OK;
	}
	netif_stop_queue(dev);
	return NETDEV_TX_BUSY;
}

static int flexcan_open(struct net_device *dev)
{
	struct flexcan_device *flexcan;
	struct platform_device *pdev;
	struct flexcan_platform_data *plat_data;

	flexcan = netdev_priv(dev);
	BUG_ON(!flexcan);

	pdev = flexcan->dev;
	plat_data = (pdev->dev).platform_data;
	if (plat_data && plat_data->active)
		plat_data->active(pdev->id);

	if (flexcan->clk)
		if (clk_enable(flexcan->clk))
			goto clk_err;

	if (flexcan->core_reg)
		if (regulator_enable(flexcan->core_reg))
			goto core_reg_err;

	if (flexcan->io_reg)
		if (regulator_enable(flexcan->io_reg))
			goto io_reg_err;

	if (plat_data && plat_data->xcvr_enable)
		plat_data->xcvr_enable(pdev->id, 1);

	if (request_irq(flexcan->irq, flexcan_irq_handler, IRQF_SHARED,
			dev->name, dev))
		goto irq_err;
	add_interrupt_randomness(flexcan->irq);

	if (flexcan_hw_open(flexcan))
		goto open_err;

	flexcan_mbm_init(flexcan);
	netif_carrier_on(dev);
	flexcan_hw_start(flexcan);
	return 0;
      open_err:
	free_irq(flexcan->irq, dev);
      irq_err:
	if (plat_data && plat_data->xcvr_enable)
		plat_data->xcvr_enable(pdev->id, 0);

	if (flexcan->io_reg)
		regulator_disable(flexcan->io_reg);
      io_reg_err:
	if (flexcan->core_reg)
		regulator_disable(flexcan->core_reg);
      core_reg_err:
	if (flexcan->clk)
		clk_disable(flexcan->clk);
      clk_err:
	if (plat_data && plat_data->inactive)
		plat_data->inactive(pdev->id);
	return -ENODEV;
}

static int flexcan_stop(struct net_device *dev)
{
	struct flexcan_device *flexcan;
	struct platform_device *pdev;
	struct flexcan_platform_data *plat_data;

	flexcan = netdev_priv(dev);

	BUG_ON(!flexcan);

	pdev = flexcan->dev;
	plat_data = (pdev->dev).platform_data;

	flexcan_hw_stop(flexcan);

	free_irq(flexcan->irq, dev);

	if (plat_data && plat_data->xcvr_enable)
		plat_data->xcvr_enable(pdev->id, 0);

	if (flexcan->io_reg)
		regulator_disable(flexcan->io_reg);
	if (flexcan->core_reg)
		regulator_disable(flexcan->core_reg);
	if (flexcan->clk)
		clk_disable(flexcan->clk);
	if (plat_data && plat_data->inactive)
		plat_data->inactive(pdev->id);
	return 0;
}

static struct net_device_ops flexcan_netdev_ops = {
	.ndo_open = flexcan_open,
	.ndo_stop = flexcan_stop,
	.ndo_start_xmit = flexcan_start_xmit,
};

static void flexcan_setup(struct net_device *dev)
{
	dev->type = ARPHRD_CAN;
	dev->mtu = sizeof(struct can_frame);
	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = FLEXCAN_MAX_MB;
	dev->flags = IFF_NOARP;
	dev->features = NETIF_F_NO_CSUM;

	dev->netdev_ops = &flexcan_netdev_ops;
}

static int flexcan_probe(struct platform_device *pdev)
{
	struct net_device *net;
	net = flexcan_device_alloc(pdev, flexcan_setup);
	if (!net)
		return -ENOMEM;

	if (register_netdev(net)) {
		flexcan_device_free(pdev);
		return -ENODEV;
	}
	return 0;
}

static int flexcan_remove(struct platform_device *pdev)
{
	flexcan_device_free(pdev);
	return 0;
}

static int flexcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *net;
	struct flexcan_device *flexcan;
	struct flexcan_platform_data *plat_data;
	net = (struct net_device *)dev_get_drvdata(&pdev->dev);
	flexcan = netdev_priv(net);

	BUG_ON(!flexcan);

	if (!(net->flags & IFF_UP))
		return 0;
	if (flexcan->wakeup)
		set_irq_wake(flexcan->irq, 1);
	else {
		plat_data = (pdev->dev).platform_data;

		if (plat_data && plat_data->xcvr_enable)
			plat_data->xcvr_enable(pdev->id, 0);

		if (flexcan->io_reg)
			regulator_disable(flexcan->io_reg);
		if (flexcan->core_reg)
			regulator_disable(flexcan->core_reg);
		if (flexcan->clk)
			clk_disable(flexcan->clk);
		if (plat_data && plat_data->inactive)
			plat_data->inactive(pdev->id);
	}
	return 0;
}

static int flexcan_resume(struct platform_device *pdev)
{
	struct net_device *net;
	struct flexcan_device *flexcan;
	struct flexcan_platform_data *plat_data;
	net = (struct net_device *)dev_get_drvdata(&pdev->dev);
	flexcan = netdev_priv(net);

	BUG_ON(!flexcan);

	if (!(net->flags & IFF_UP))
		return 0;

	if (flexcan->wakeup)
		set_irq_wake(flexcan->irq, 0);
	else {
		plat_data = (pdev->dev).platform_data;
		if (plat_data && plat_data->active)
			plat_data->active(pdev->id);

		if (flexcan->clk) {
			if (clk_enable(flexcan->clk))
				printk(KERN_ERR "%s:enable clock fail\n",
				       __func__);
		}

		if (flexcan->core_reg) {
			if (regulator_enable(flexcan->core_reg))
				printk(KERN_ERR "%s:enable core voltage\n",
				       __func__);
		}
		if (flexcan->io_reg) {
			if (regulator_enable(flexcan->io_reg))
				printk(KERN_ERR "%s:enable io voltage\n",
				       __func__);
		}

		if (plat_data && plat_data->xcvr_enable)
			plat_data->xcvr_enable(pdev->id, 1);
	}
	return 0;
}

static struct platform_driver flexcan_driver = {
	.driver = {
		   .name = FLEXCAN_DEVICE_NAME,
		   },
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.suspend = flexcan_suspend,
	.resume = flexcan_resume,
};

static __init int flexcan_init(void)
{
	pr_info("Freescale FlexCAN Driver \n");
	return platform_driver_register(&flexcan_driver);
}

static __exit void flexcan_exit(void)
{
	return platform_driver_unregister(&flexcan_driver);
}

module_init(flexcan_init);
module_exit(flexcan_exit);

MODULE_LICENSE("GPL");
