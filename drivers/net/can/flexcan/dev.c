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
 * @file dev.c
 *
 * @brief Driver for Freescale CAN Controller FlexCAN.
 *
 * @ingroup can
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>

#include <linux/module.h>
#ifdef CONFIG_ARCH_MXS
#include <mach/device.h>
#endif
#include "flexcan.h"

#define DEFAULT_BITRATE	    500000
#define TIME_SEGMENT_MIN    8
#define TIME_SEGMENT_MAX    25
#define TIME_SEGMENT_MID    ((TIME_SEGMENT_MIN + TIME_SEGMENT_MAX)/2)

struct time_segment {
	char propseg;
	char pseg1;
	char pseg2;
};

struct time_segment time_segments[] = {
    { /* total 8 timequanta */
	1, 2, 1
    },
    { /* total 9 timequanta */
	1, 2, 2
    },
    { /* total 10 timequanta */
	2, 2, 2
    },
    { /* total 11 timequanta */
	2, 2, 3
    },
    { /* total 12 timequanta */
	2, 3, 3
    },
    { /* total 13 timequanta */
	3, 3, 3
    },
    { /* total 14 timequanta */
	3, 3, 4
    },
    { /* total 15 timequanta */
	3, 4, 4
    },
    { /* total 16 timequanta */
	4, 4, 4
    },
    { /* total 17 timequanta */
	4, 4, 5
    },
    { /* total 18 timequanta */
	4, 5, 5
    },
    { /* total 19 timequanta */
	5, 5, 5
    },
    { /* total 20 timequanta */
	5, 5, 6
    },
    { /* total 21 timequanta */
	5, 6, 6
    },
    { /* total 22 timequanta */
	6, 6, 6
    },
    { /* total 23 timequanta */
	6, 6, 7
    },
    { /* total 24 timequanta */
	6, 7, 7
    },
    { /* total 25 timequanta */
	7, 7, 7
    },
};

enum {
	FLEXCAN_ATTR_STATE = 0,
	FLEXCAN_ATTR_BITRATE,
	FLEXCAN_ATTR_BR_PRESDIV,
	FLEXCAN_ATTR_BR_RJW,
	FLEXCAN_ATTR_BR_PROPSEG,
	FLEXCAN_ATTR_BR_PSEG1,
	FLEXCAN_ATTR_BR_PSEG2,
	FLEXCAN_ATTR_BR_CLKSRC,
	FLEXCAN_ATTR_MAXMB,
	FLEXCAN_ATTR_XMIT_MAXMB,
	FLEXCAN_ATTR_FIFO,
	FLEXCAN_ATTR_WAKEUP,
	FLEXCAN_ATTR_SRX_DIS,
	FLEXCAN_ATTR_WAK_SRC,
	FLEXCAN_ATTR_BCC,
	FLEXCAN_ATTR_LOCAL_PRIORITY,
	FLEXCAN_ATTR_ABORT,
	FLEXCAN_ATTR_LOOPBACK,
	FLEXCAN_ATTR_SMP,
	FLEXCAN_ATTR_BOFF_REC,
	FLEXCAN_ATTR_TSYN,
	FLEXCAN_ATTR_LISTEN,
	FLEXCAN_ATTR_EXTEND_MSG,
	FLEXCAN_ATTR_STANDARD_MSG,
#ifdef CONFIG_CAN_DEBUG_DEVICES
	FLEXCAN_ATTR_DUMP_REG,
	FLEXCAN_ATTR_DUMP_XMIT_MB,
	FLEXCAN_ATTR_DUMP_RX_MB,
#endif
	FLEXCAN_ATTR_MAX
};

static ssize_t flexcan_show_attr(struct device *dev,
				 struct device_attribute *attr, char *buf);
static ssize_t flexcan_set_attr(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count);

static struct device_attribute flexcan_dev_attr[FLEXCAN_ATTR_MAX] = {
	[FLEXCAN_ATTR_STATE] = __ATTR(state, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_BITRATE] =
	    __ATTR(bitrate, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PRESDIV] =
	    __ATTR(br_presdiv, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_RJW] =
	    __ATTR(br_rjw, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PROPSEG] =
	    __ATTR(br_propseg, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PSEG1] =
	    __ATTR(br_pseg1, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_PSEG2] =
	    __ATTR(br_pseg2, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BR_CLKSRC] =
	    __ATTR(br_clksrc, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_MAXMB] =
	    __ATTR(maxmb, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_XMIT_MAXMB] =
	    __ATTR(xmit_maxmb, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_FIFO] =
	    __ATTR(fifo, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_WAKEUP] =
	    __ATTR(wakeup, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_SRX_DIS] =
	    __ATTR(srx_dis, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_WAK_SRC] =
	    __ATTR(wak_src, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BCC] =
	    __ATTR(bcc, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LOCAL_PRIORITY] =
	    __ATTR(local_priority, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_ABORT] =
	    __ATTR(abort, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LOOPBACK] =
	    __ATTR(loopback, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_SMP] =
	    __ATTR(smp, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_BOFF_REC] =
	    __ATTR(boff_rec, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_TSYN] =
	    __ATTR(tsyn, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_LISTEN] =
	    __ATTR(listen, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_EXTEND_MSG] =
	    __ATTR(ext_msg, 0644, flexcan_show_attr, flexcan_set_attr),
	[FLEXCAN_ATTR_STANDARD_MSG] =
	    __ATTR(std_msg, 0644, flexcan_show_attr, flexcan_set_attr),
#ifdef CONFIG_CAN_DEBUG_DEVICES
	[FLEXCAN_ATTR_DUMP_REG] =
	    __ATTR(dump_reg, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_DUMP_XMIT_MB] =
	    __ATTR(dump_xmit_mb, 0444, flexcan_show_attr, NULL),
	[FLEXCAN_ATTR_DUMP_RX_MB] =
	    __ATTR(dump_rx_mb, 0444, flexcan_show_attr, NULL),
#endif
};

static void flexcan_set_bitrate(struct flexcan_device *flexcan, int bitrate)
{
	/* TODO:: implement in future
	 * based on the bitrate to get the timing of
	 * presdiv, pseg1, pseg2, propseg
	 */
	int i, rate, div;
	bool found = false;
	struct time_segment *segment;
	rate = clk_get_rate(flexcan->clk);

	if (!bitrate)
		bitrate = DEFAULT_BITRATE;

	if (rate % bitrate == 0) {
		div = rate / bitrate;
		for (i = TIME_SEGMENT_MID; i <= TIME_SEGMENT_MAX; i++) {
			if (div % i == 0) {
				found = true;
				break;
			}
		}
		if (!found) {
			for (i = TIME_SEGMENT_MID - 1;
					    i >= TIME_SEGMENT_MIN; i--) {
				if (div % i == 0) {
					found = true;
					break;
				}
			}

		}
	}

	if (found) {
		segment = &time_segments[i - TIME_SEGMENT_MIN];
		flexcan->br_presdiv = div/i - 1;
		flexcan->br_propseg = segment->propseg;
		flexcan->br_pseg1 = segment->pseg1;
		flexcan->br_pseg2 = segment->pseg2;
		flexcan->bitrate = bitrate;
	} else {
		pr_info("The bitrate %d can't supported with clock \
				    rate of %d \n", bitrate, rate);
	}
}

static void flexcan_update_bitrate(struct flexcan_device *flexcan)
{
	int rate, div;
	struct flexcan_platform_data *plat_data;
	plat_data = flexcan->dev->dev.platform_data;

	if (plat_data->root_clk_id)
		rate = clk_get_rate(flexcan->clk);
	else {
		if (flexcan->br_clksrc)
			rate = clk_get_rate(flexcan->clk);
		else {
			struct clk *clk;
			clk = clk_get(NULL, "ckih");
			if (!clk)
				return;
			rate = clk_get_rate(clk);
			clk_put(clk);
		}
	}
	if (!rate)
		return;

	div = (flexcan->br_presdiv + 1);
	div *=
	    (flexcan->br_propseg + flexcan->br_pseg1 + flexcan->br_pseg2 + 4);
	flexcan->bitrate = (rate + div - 1) / div;
}

#ifdef CONFIG_CAN_DEBUG_DEVICES
static int flexcan_dump_reg(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0;
	unsigned int reg;
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_MCR);
	ret += sprintf(buf + ret, "MCR::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_CTRL);
	ret += sprintf(buf + ret, "CTRL::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_RXGMASK);
	ret += sprintf(buf + ret, "RXGMASK::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_RX14MASK);
	ret += sprintf(buf + ret, "RX14MASK::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_RX15MASK);
	ret += sprintf(buf + ret, "RX15MASK::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_ECR);
	ret += sprintf(buf + ret, "ECR::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_ESR);
	ret += sprintf(buf + ret, "ESR::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_IMASK2);
	ret += sprintf(buf + ret, "IMASK2::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_IMASK1);
	ret += sprintf(buf + ret, "IMASK1::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_IFLAG2);
	ret += sprintf(buf + ret, "IFLAG2::0x%x\n", reg);
	reg = __raw_readl(flexcan->io_base + CAN_HW_REG_IFLAG1);
	ret += sprintf(buf + ret, "IFLAG1::0x%x\n", reg);
	return ret;
}

static int flexcan_dump_xmit_mb(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0, i;
	i = flexcan->xmit_maxmb + 1;
	for (; i <= flexcan->maxmb; i++)
		ret +=
		    sprintf(buf + ret,
			    "mb[%d]::CS:0x%x ID:0x%x DATA[1~2]:0x%02x,0x%02x\n",
			    i, flexcan->hwmb[i].mb_cs,
			    flexcan->hwmb[i].mb_id, flexcan->hwmb[i].mb_data[1],
			    flexcan->hwmb[i].mb_data[2]);
	return ret;
}

static int flexcan_dump_rx_mb(struct flexcan_device *flexcan, char *buf)
{
	int ret = 0, i;
	for (i = 0; i <= flexcan->xmit_maxmb; i++)
		ret +=
		    sprintf(buf + ret,
			    "mb[%d]::CS:0x%x ID:0x%x DATA[1~2]:0x%02x,0x%02x\n",
			    i, flexcan->hwmb[i].mb_cs,
			    flexcan->hwmb[i].mb_id, flexcan->hwmb[i].mb_data[1],
			    flexcan->hwmb[i].mb_data[2]);
	return ret;
}
#endif

static ssize_t flexcan_show_state(struct net_device *net, char *buf)
{
	int ret, esr;
	struct flexcan_device *flexcan = netdev_priv(net);
	ret = sprintf(buf, "%s::", netif_running(net) ? "Start" : "Stop");
	if (netif_carrier_ok(net)) {
		esr = __raw_readl(flexcan->io_base + CAN_HW_REG_ESR);
		switch ((esr & __ESR_FLT_CONF_MASK) >> __ESR_FLT_CONF_OFF) {
		case 0:
			ret += sprintf(buf + ret, "normal\n");
			break;
		case 1:
			ret += sprintf(buf + ret, "error passive\n");
			break;
		default:
			ret += sprintf(buf + ret, "bus off\n");
		}
	} else
		ret += sprintf(buf + ret, "bus off\n");
	return ret;
}

static ssize_t flexcan_show_attr(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int attr_id;
	struct net_device *net;
	struct flexcan_device *flexcan;

	net = dev_get_drvdata(dev);
	BUG_ON(!net);
	flexcan = netdev_priv(net);
	BUG_ON(!flexcan);

	attr_id = attr - flexcan_dev_attr;
	switch (attr_id) {
	case FLEXCAN_ATTR_STATE:
		return flexcan_show_state(net, buf);
	case FLEXCAN_ATTR_BITRATE:
		return sprintf(buf, "%d\n", flexcan->bitrate);
	case FLEXCAN_ATTR_BR_PRESDIV:
		return sprintf(buf, "%d\n", flexcan->br_presdiv + 1);
	case FLEXCAN_ATTR_BR_RJW:
		return sprintf(buf, "%d\n", flexcan->br_rjw);
	case FLEXCAN_ATTR_BR_PROPSEG:
		return sprintf(buf, "%d\n", flexcan->br_propseg + 1);
	case FLEXCAN_ATTR_BR_PSEG1:
		return sprintf(buf, "%d\n", flexcan->br_pseg1 + 1);
	case FLEXCAN_ATTR_BR_PSEG2:
		return sprintf(buf, "%d\n", flexcan->br_pseg2 + 1);
	case FLEXCAN_ATTR_BR_CLKSRC:
		return sprintf(buf, "%s\n", flexcan->br_clksrc ? "bus" : "osc");
	case FLEXCAN_ATTR_MAXMB:
		return sprintf(buf, "%d\n", flexcan->maxmb + 1);
	case FLEXCAN_ATTR_XMIT_MAXMB:
		return sprintf(buf, "%d\n", flexcan->xmit_maxmb + 1);
	case FLEXCAN_ATTR_FIFO:
		return sprintf(buf, "%d\n", flexcan->fifo);
	case FLEXCAN_ATTR_WAKEUP:
		return sprintf(buf, "%d\n", flexcan->wakeup);
	case FLEXCAN_ATTR_SRX_DIS:
		return sprintf(buf, "%d\n", flexcan->srx_dis);
	case FLEXCAN_ATTR_WAK_SRC:
		return sprintf(buf, "%d\n", flexcan->wak_src);
	case FLEXCAN_ATTR_BCC:
		return sprintf(buf, "%d\n", flexcan->bcc);
	case FLEXCAN_ATTR_LOCAL_PRIORITY:
		return sprintf(buf, "%d\n", flexcan->lprio);
	case FLEXCAN_ATTR_ABORT:
		return sprintf(buf, "%d\n", flexcan->abort);
	case FLEXCAN_ATTR_LOOPBACK:
		return sprintf(buf, "%d\n", flexcan->loopback);
	case FLEXCAN_ATTR_SMP:
		return sprintf(buf, "%d\n", flexcan->smp);
	case FLEXCAN_ATTR_BOFF_REC:
		return sprintf(buf, "%d\n", flexcan->boff_rec);
	case FLEXCAN_ATTR_TSYN:
		return sprintf(buf, "%d\n", flexcan->tsyn);
	case FLEXCAN_ATTR_LISTEN:
		return sprintf(buf, "%d\n", flexcan->listen);
	case FLEXCAN_ATTR_EXTEND_MSG:
		return sprintf(buf, "%d\n", flexcan->ext_msg);
	case FLEXCAN_ATTR_STANDARD_MSG:
		return sprintf(buf, "%d\n", flexcan->std_msg);
#ifdef CONFIG_CAN_DEBUG_DEVICES
	case FLEXCAN_ATTR_DUMP_REG:
		return flexcan_dump_reg(flexcan, buf);
	case FLEXCAN_ATTR_DUMP_XMIT_MB:
		return flexcan_dump_xmit_mb(flexcan, buf);
	case FLEXCAN_ATTR_DUMP_RX_MB:
		return flexcan_dump_rx_mb(flexcan, buf);
#endif
	default:
		return sprintf(buf, "%s:%p->%p\n", __func__, flexcan_dev_attr,
			       attr);
	}
}

static ssize_t flexcan_set_attr(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int attr_id, tmp;
	struct net_device *net;
	struct flexcan_device *flexcan;

	net = dev_get_drvdata(dev);
	BUG_ON(!net);
	flexcan = netdev_priv(net);
	BUG_ON(!flexcan);

	attr_id = attr - flexcan_dev_attr;

	if (mutex_lock_interruptible(&flexcan->mutex))
		return count;

	if (netif_running(net))
		goto set_finish;

	if (attr_id == FLEXCAN_ATTR_BR_CLKSRC) {
		if (!strncasecmp(buf, "bus", 3))
			flexcan->br_clksrc = 1;
		else if (!strncasecmp(buf, "osc", 3))
			flexcan->br_clksrc = 0;
		goto set_finish;
	}

	tmp = simple_strtoul(buf, NULL, 0);
	switch (attr_id) {
	case FLEXCAN_ATTR_BITRATE:
		flexcan_set_bitrate(flexcan, tmp);
		break;
	case FLEXCAN_ATTR_BR_PRESDIV:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PRESDIV)) {
			flexcan->br_presdiv = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_RJW:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_RJW))
			flexcan->br_rjw = tmp - 1;
		break;
	case FLEXCAN_ATTR_BR_PROPSEG:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PROPSEG)) {
			flexcan->br_propseg = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_PSEG1:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PSEG1)) {
			flexcan->br_pseg1 = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_BR_PSEG2:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_PSEG2)) {
			flexcan->br_pseg2 = tmp - 1;
			flexcan_update_bitrate(flexcan);
		}
		break;
	case FLEXCAN_ATTR_MAXMB:
		if ((tmp > 0) && (tmp <= FLEXCAN_MAX_MB)) {
			if (flexcan->maxmb != (tmp - 1)) {
				flexcan->maxmb = tmp - 1;
				if (flexcan->xmit_maxmb < flexcan->maxmb)
					flexcan->xmit_maxmb = flexcan->maxmb;
			}
		}
		break;
	case FLEXCAN_ATTR_XMIT_MAXMB:
		if ((tmp > 0) && (tmp <= (flexcan->maxmb + 1))) {
			if (flexcan->xmit_maxmb != (tmp - 1))
				flexcan->xmit_maxmb = tmp - 1;
		}
		break;
	case FLEXCAN_ATTR_FIFO:
		flexcan->fifo = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_WAKEUP:
		flexcan->wakeup = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_SRX_DIS:
		flexcan->srx_dis = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_WAK_SRC:
		flexcan->wak_src = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_BCC:
		flexcan->bcc = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_LOCAL_PRIORITY:
		flexcan->lprio = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_ABORT:
		flexcan->abort = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_LOOPBACK:
		flexcan->loopback = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_SMP:
		flexcan->smp = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_BOFF_REC:
		flexcan->boff_rec = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_TSYN:
		flexcan->tsyn = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_LISTEN:
		flexcan->listen = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_EXTEND_MSG:
		flexcan->ext_msg = tmp ? 1 : 0;
		break;
	case FLEXCAN_ATTR_STANDARD_MSG:
		flexcan->std_msg = tmp ? 1 : 0;
		break;
	}
      set_finish:
	mutex_unlock(&flexcan->mutex);
	return count;
}

static void flexcan_device_default(struct flexcan_device *dev)
{
	struct platform_device *pdev = dev->dev;
	struct flexcan_platform_data *plat_data = (pdev->dev).platform_data;
	dev->br_clksrc = plat_data->br_clksrc;
	dev->br_rjw = plat_data->br_rjw;
	dev->br_presdiv = plat_data->br_presdiv;
	dev->br_propseg = plat_data->br_propseg;
	dev->br_pseg1 = plat_data->br_pseg1;
	dev->br_pseg2 = plat_data->br_pseg2;

	dev->bcc = plat_data->bcc;
	dev->srx_dis = plat_data->srx_dis;
	dev->smp = plat_data->smp;
	dev->boff_rec = plat_data->boff_rec;

	dev->maxmb = FLEXCAN_MAX_MB - 1;
	dev->xmit_maxmb = (FLEXCAN_MAX_MB >> 1) - 1;
	dev->xmit_mb = dev->maxmb - dev->xmit_maxmb;

	dev->ext_msg = plat_data->ext_msg;
	dev->std_msg = plat_data->std_msg;
}

static int flexcan_device_attach(struct flexcan_device *flexcan)
{
	int ret;
	struct resource *res;
	struct platform_device *pdev = flexcan->dev;
	struct flexcan_platform_data *plat_data = (pdev->dev).platform_data;
	struct clk *can_root_clk;

	res = platform_get_resource(flexcan->dev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	flexcan->io_base = ioremap(res->start, res->end - res->start + 1);
	if (!flexcan->io_base)
		return -ENOMEM;

	flexcan->irq = platform_get_irq(flexcan->dev, 0);
	if (!flexcan->irq) {
		ret = -ENODEV;
		goto no_irq_err;
	}

	ret = -EINVAL;
	if (plat_data) {
		if (plat_data->core_reg) {
			flexcan->core_reg = regulator_get(&pdev->dev,
							  plat_data->core_reg);
			if (!flexcan->core_reg)
				goto plat_err;
		}

		if (plat_data->io_reg) {
			flexcan->io_reg = regulator_get(&pdev->dev,
							plat_data->io_reg);
			if (!flexcan->io_reg)
				goto plat_err;
		}
	}
	flexcan->clk = clk_get(&(flexcan->dev)->dev, "can_clk");
	if (plat_data->root_clk_id) {
		can_root_clk = clk_get(NULL, plat_data->root_clk_id);
		clk_set_parent(flexcan->clk, can_root_clk);
	}
	flexcan->hwmb = (struct can_hw_mb *)(flexcan->io_base + CAN_MB_BASE);
	flexcan->rx_mask = (unsigned int *)(flexcan->io_base + CAN_RXMASK_BASE);

	return 0;
      plat_err:
	if (flexcan->core_reg) {
		regulator_put(flexcan->core_reg);
		flexcan->core_reg = NULL;
	}
      no_irq_err:
	if (flexcan->io_base)
		iounmap(flexcan->io_base);
	return ret;
}

static void flexcan_device_detach(struct flexcan_device *flexcan)
{
	if (flexcan->clk) {
		clk_put(flexcan->clk);
		flexcan->clk = NULL;
	}

	if (flexcan->io_reg) {
		regulator_put(flexcan->io_reg);
		flexcan->io_reg = NULL;
	}

	if (flexcan->core_reg) {
		regulator_put(flexcan->core_reg);
		flexcan->core_reg = NULL;
	}

	if (flexcan->io_base)
		iounmap(flexcan->io_base);
}

/*!
 * @brief The function allocates can device.
 *
 * @param pdev	the pointer of platform device.
 * @param setup	the initial function pointer of network device.
 *
 * @return none
 */
struct net_device *flexcan_device_alloc(struct platform_device *pdev,
					void (*setup) (struct net_device *dev))
{
	struct flexcan_device *flexcan;
	struct net_device *net;
	int i, num;

	net = alloc_netdev(sizeof(*flexcan), "can%d", setup);
	if (net == NULL) {
		printk(KERN_ERR "Allocate netdevice for FlexCAN fail!\n");
		return NULL;
	}
	flexcan = netdev_priv(net);
	memset(flexcan, 0, sizeof(*flexcan));

	mutex_init(&flexcan->mutex);
	init_timer(&flexcan->timer);

	flexcan->dev = pdev;
	if (flexcan_device_attach(flexcan)) {
		printk(KERN_ERR "Attach FlexCAN fail!\n");
		free_netdev(net);
		return NULL;
	}
	flexcan_device_default(flexcan);
	flexcan_set_bitrate(flexcan, flexcan->bitrate);
	flexcan_update_bitrate(flexcan);

	num = ARRAY_SIZE(flexcan_dev_attr);

	for (i = 0; i < num; i++) {
		if (device_create_file(&pdev->dev, flexcan_dev_attr + i)) {
			printk(KERN_ERR "Create attribute file fail!\n");
			break;
		}
	}

	if (i != num) {
		for (; i >= 0; i--)
			device_remove_file(&pdev->dev, flexcan_dev_attr + i);
		free_netdev(net);
		return NULL;
	}
	dev_set_drvdata(&pdev->dev, net);
	return net;
}

/*!
 * @brief The function frees can device.
 *
 * @param pdev	the pointer of platform device.
 *
 * @return none
 */
void flexcan_device_free(struct platform_device *pdev)
{
	struct net_device *net;
	struct flexcan_device *flexcan;
	int i, num;
	net = (struct net_device *)dev_get_drvdata(&pdev->dev);

	unregister_netdev(net);
	flexcan = netdev_priv(net);
	del_timer(&flexcan->timer);

	num = ARRAY_SIZE(flexcan_dev_attr);

	for (i = 0; i < num; i++)
		device_remove_file(&pdev->dev, flexcan_dev_attr + i);

	flexcan_device_detach(netdev_priv(net));
	free_netdev(net);
}
