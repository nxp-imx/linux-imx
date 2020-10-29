// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <dt-bindings/reset/imx-audiomix-reset.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset-controller.h>

#define IMX_AUDIOMIX_EARC_CTRL_REG	0x200

#define IMX_AUDIOMIX_EARC_RESET_BIT	0x0
#define IMX_AUDIOMIX_EARC_PHY_RESET_BIT	0x1

struct imx_audiomix_reset_data {
	void __iomem *base;
	struct reset_controller_dev rcdev;
	spinlock_t lock;
};

static int imx_audiomix_reset_set(struct reset_controller_dev *rcdev,
			  unsigned long id, bool assert)
{
	struct imx_audiomix_reset_data *drvdata = container_of(rcdev,
			struct imx_audiomix_reset_data, rcdev);
	void __iomem *reg_addr = drvdata->base;
	unsigned long flags;
	unsigned int offset;
	u32 reg;

	switch (id) {
	case IMX_AUDIOMIX_EARC_PHY_RESET:
		reg_addr += IMX_AUDIOMIX_EARC_CTRL_REG;
		offset = IMX_AUDIOMIX_EARC_PHY_RESET_BIT;
		break;
	case IMX_AUDIOMIX_EARC_RESET:
		reg_addr += IMX_AUDIOMIX_EARC_CTRL_REG;
		offset = IMX_AUDIOMIX_EARC_RESET_BIT;
		break;
	default:
		return -EINVAL;
	}

	if (assert) {
		pm_runtime_get_sync(rcdev->dev);
		spin_lock_irqsave(&drvdata->lock, flags);
		reg = readl(reg_addr);
		writel(reg & ~BIT(offset), reg_addr);
		spin_unlock_irqrestore(&drvdata->lock, flags);
	} else {
		spin_lock_irqsave(&drvdata->lock, flags);
		reg = readl(reg_addr);
		writel(reg | BIT(offset), reg_addr);
		spin_unlock_irqrestore(&drvdata->lock, flags);
		pm_runtime_put(rcdev->dev);
	}

	return 0;
}

static int imx_audiomix_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return imx_audiomix_reset_set(rcdev, id, true);
}

static int imx_audiomix_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return imx_audiomix_reset_set(rcdev, id, false);
}

static const struct reset_control_ops imx_audiomix_reset_ops = {
	.assert		= imx_audiomix_reset_assert,
	.deassert	= imx_audiomix_reset_deassert,
};

static int imx_audiomix_reset_probe(struct platform_device *pdev)
{
	struct imx_audiomix_reset_data *drvdata;
	struct device *dev = &pdev->dev;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->base = dev_get_drvdata(dev->parent);

	platform_set_drvdata(pdev, drvdata);

	pm_runtime_enable(dev);

	spin_lock_init(&drvdata->lock);

	drvdata->rcdev.owner     = THIS_MODULE;
	drvdata->rcdev.nr_resets = IMX_AUDIOMIX_RESET_NUM;
	drvdata->rcdev.ops       = &imx_audiomix_reset_ops;
	drvdata->rcdev.of_node   = dev->of_node;
	drvdata->rcdev.dev	 = dev;

	return devm_reset_controller_register(dev, &drvdata->rcdev);
}

static const struct of_device_id imx_audiomix_reset_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-audiomix-reset", },
	{ /* sentinel */ },
};

static struct platform_driver imx_audiomix_reset_driver = {
	.probe	= imx_audiomix_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= imx_audiomix_reset_dt_ids,
	},
};
module_platform_driver(imx_audiomix_reset_driver);
MODULE_LICENSE("GPL v2");
