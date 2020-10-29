// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 */

#include <dt-bindings/reset/imx-hdmimix-reset.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset-controller.h>

#define IMX_HDMIMIX_RESET_CTL0_REG	0x20

#define IMX_HDMIMIX_RESET_CTL0_TX_TRNG_RESETN							(1 << 20)
#define IMX_HDMIMIX_RESET_CTL0_VID_LINK_SLV_RESETN						(1 << 22)
#define IMX_HDMIMIX_RESET_CTL0_PAI_RESETN								(1 << 18)
#define IMX_HDMIMIX_RESET_CTL0_IRQ_STEER_RESETN							(1 << 16)
#define IMX_HDMIMIX_RESET_CTL0_TX_KSV_MEM_RESETN						(1 << 13)
#define IMX_HDMIMIX_RESET_CTL0_TX_PHY_PRESETN							(1 << 12)
#define IMX_HDMIMIX_RESET_CTL0_TX_APBRSTZ								(1 << 11)
#define IMX_HDMIMIX_RESET_CTL0_TX_RSTZ									(1 << 10)
#define IMX_HDMIMIX_RESET_CTL0_FDCC_HDMI_RESETN							(1 << 7)
#define IMX_HDMIMIX_RESET_CTL0_FDCC_RESETN								(1 << 6)
#define IMX_HDMIMIX_RESET_CTL0_LCDIF_APB_RESETN							(1 << 5)
#define IMX_HDMIMIX_RESET_CTL0_LCDIF_ASYNC_RESETN						(1 << 4)
#define IMX_HDMIMIX_RESET_CTL0_NOC_RESETN								(1 << 0)

struct imx_hdmimix_reset_data {
	void __iomem *base;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
};

static int imx_hdmimix_reset_set(struct reset_controller_dev *rcdev,
			  unsigned long id, bool assert)
{
	struct imx_hdmimix_reset_data *drvdata = container_of(rcdev,
			struct imx_hdmimix_reset_data, rcdev);
	void __iomem *reg_addr = drvdata->base + IMX_HDMIMIX_RESET_CTL0_REG;
	unsigned long flags;
	unsigned int val;
	u32 reg;

	switch (id) {
	case IMX_HDMIMIX_HDMI_TX_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_APBRSTZ |
				IMX_HDMIMIX_RESET_CTL0_TX_RSTZ |
				IMX_HDMIMIX_RESET_CTL0_FDCC_HDMI_RESETN |
				IMX_HDMIMIX_RESET_CTL0_FDCC_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_PHY_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_PHY_PRESETN;
		break;
	case IMX_HDMIMIX_HDMI_PAI_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_PAI_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_PVI_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_VID_LINK_SLV_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_TRNG_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_TRNG_RESETN;
		break;
	case IMX_HDMIMIX_IRQ_STEER_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_IRQ_STEER_RESETN;
		break;
	case IMX_HDMIMIX_HDMI_HDCP_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_TX_KSV_MEM_RESETN;
		break;
	case IMX_HDMIMIX_LCDIF_RESET:
		val = IMX_HDMIMIX_RESET_CTL0_LCDIF_APB_RESETN |
				IMX_HDMIMIX_RESET_CTL0_LCDIF_ASYNC_RESETN;
		break;
	default:
		return -EINVAL;
	}

	if (assert) {
		pm_runtime_get_sync(rcdev->dev);
		spin_lock_irqsave(&drvdata->lock, flags);
		reg = readl(reg_addr);
		writel(reg & ~val, reg_addr);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	} else {
		spin_lock_irqsave(&drvdata->lock, flags);
		reg = readl(reg_addr);
		writel(reg | val, reg_addr);
		spin_unlock_irqrestore(&drvdata->lock, flags);
		pm_runtime_put(rcdev->dev);
	}
		reg = readl(reg_addr);

	return 0;
}

static int imx_hdmimix_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return imx_hdmimix_reset_set(rcdev, id, true);
}

static int imx_hdmimix_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return imx_hdmimix_reset_set(rcdev, id, false);
}

static int imx_hdmimix_reset(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	imx_hdmimix_reset_set(rcdev, id, true);
	return imx_hdmimix_reset_set(rcdev, id, false);
}

static const struct reset_control_ops imx_hdmimix_reset_ops = {
	.reset		= imx_hdmimix_reset,
	.assert		= imx_hdmimix_reset_assert,
	.deassert	= imx_hdmimix_reset_deassert,
};

static int imx_hdmimix_reset_probe(struct platform_device *pdev)
{
	struct imx_hdmimix_reset_data *drvdata;
	struct device *dev = &pdev->dev;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->base = dev_get_drvdata(dev->parent);

	platform_set_drvdata(pdev, drvdata);

	pm_runtime_enable(dev);

	spin_lock_init(&drvdata->lock);

	drvdata->rcdev.owner     = THIS_MODULE;
	drvdata->rcdev.nr_resets = IMX_HDMIMIX_RESET_NUM;
	drvdata->rcdev.ops       = &imx_hdmimix_reset_ops;
	drvdata->rcdev.of_node   = dev->of_node;
	drvdata->rcdev.dev	 = dev;

	return devm_reset_controller_register(dev, &drvdata->rcdev);
}

static const struct of_device_id imx_hdmimix_reset_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmimix-reset", },
	{ /* sentinel */ },
};

static struct platform_driver imx_hdmimix_reset_driver = {
	.probe	= imx_hdmimix_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= imx_hdmimix_reset_dt_ids,
	},
};
module_platform_driver(imx_hdmimix_reset_driver);
MODULE_LICENSE("GPL v2");
