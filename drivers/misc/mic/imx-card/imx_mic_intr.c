// SPDX-License-Identifier: GPL-2.0
// Copyright 2020 NXP

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/imx_mic_epf.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/pm_domain.h>
#include "imx_mic_boot.h"
#include "../common/imx_mic_common.h"

static struct imx_mic_intr *g_mintr;

void imx_mic_intr_enable(struct imx_mic_driver *mdrv, int index)
{
	struct imx_mic_intr *mintr = mdrv->mintr;
	u32 val;

	val = readl(mintr->base + MIC_INTR_MU_CR);
	val |= MIC_INTR_MU_ACR_GIEn(index);
	writel(val, mintr->base + MIC_INTR_MU_CR);
}
EXPORT_SYMBOL_GPL(imx_mic_intr_enable);

void imx_mic_intr_disable(struct imx_mic_driver *mdrv, int index)
{
	struct imx_mic_intr *mintr = mdrv->mintr;
	u32 val;

	val = readl(mintr->base + MIC_INTR_MU_CR);
	val &= ~MIC_INTR_MU_ACR_GIEn(index);
	writel(val, mintr->base + MIC_INTR_MU_CR);
}
EXPORT_SYMBOL_GPL(imx_mic_intr_disable);

void imx_mic_intr_ack(struct imx_mic_driver *mdrv, int index)
{
	struct imx_mic_intr *mintr = mdrv->mintr;
	u32 val;

	val = readl(mintr->base + MIC_INTR_MU_SR);
	/* clear interrupt: W1C*/
	val |= MIC_INTR_MU_ASR_GIPn(index);
	writel(val, mintr->base + MIC_INTR_MU_SR);
}
EXPORT_SYMBOL_GPL(imx_mic_intr_ack);

static irqreturn_t imx_mic_intr_handler(int irq, void *dev_id)
{
	struct imx_mic_intr *mintr = dev_id;
	u32 val;

	val = readl(mintr->base + MIC_INTR_MU_SR);

	if (val & MIC_INTR_MU_ASR_GIPn(MIC_CONFIG_DB))
		return mintr->config_intr(irq, mintr->config_intr_priv);
	else if (val & MIC_INTR_MU_ASR_GIPn(MIC_DATA_DB))
		return mintr->data_intr(irq, mintr->data_intr_priv);
	else {
		dev_err(mintr->dev, "invalid mu channel index\n");
		return IRQ_NONE;
	}
}

int imx_mic_intr_init(struct imx_mic_driver *mdrv)
{
	int ret, irq;

	mdrv->mintr = g_mintr;
	irq = mdrv->mintr->irq;

	ret = request_irq(irq, imx_mic_intr_handler, IRQF_SHARED,
			  mdrv->name, mdrv->mintr);
	if (ret) {
		dev_err(mdrv->dev, "Failed to request irq handler\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imx_mic_intr_init);

void imx_mic_intr_uninit(struct imx_mic_driver *mdrv)
{
	int irq;

	mdrv->mintr = g_mintr;
	irq = mdrv->mintr->irq;

	free_irq(irq, mdrv);
}
EXPORT_SYMBOL_GPL(imx_mic_intr_uninit);

void imx_mic_get_doorbell_info(u32 *doorbell_reg_base, u32 *doorbell_reg_size)
{
	struct imx_mic_intr *mintr = g_mintr;
	*doorbell_reg_base = mintr->doorbell_reg_base;
	*doorbell_reg_size = mintr->doorbell_reg_size;
}
EXPORT_SYMBOL_GPL(imx_mic_get_doorbell_info);

static int parse_doorbell_info(struct imx_mic_intr *mintr)
{
	struct device_node *np = mintr->dev->of_node;
	int ret;
	u32 val;

	ret = of_property_read_u32_index(np, "doorbell-reg", 0, &val);
	if (ret) {
		dev_err(mintr->dev, "can't get doorbell-reg address\n");
		return ret;
	}
	mintr->doorbell_reg_base = val;

	ret = of_property_read_u32_index(np, "doorbell-reg", 1, &val);
	if (ret) {
		dev_err(mintr->dev, "can't get doorbell-reg size\n");
		return ret;
	}
	mintr->doorbell_reg_size = val;

	return 0;
}

static int imx_mic_intr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_link *link;
	struct imx_mic_intr *intr;
	int ret;

	intr = devm_kzalloc(dev, sizeof(*intr), GFP_KERNEL);;
	if (!intr)
		return -ENOMEM;
	intr->dev = dev;
	g_mintr = intr;

	intr->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intr->base))
		return PTR_ERR(intr->base);

	intr->irq = platform_get_irq(pdev, 0);
	if (intr->irq < 0)
		return intr->irq;

	intr->clk_aside = devm_clk_get(dev, "clk_a");
	if (IS_ERR(intr->clk_aside)) {
		if (PTR_ERR(intr->clk_aside) != -ENOENT)
			return PTR_ERR(intr->clk_aside);

		intr->clk_aside = NULL;
	}

	intr->clk_bside = devm_clk_get(dev, "clk_b");
	if (IS_ERR(intr->clk_bside)) {
		if (PTR_ERR(intr->clk_bside) != -ENOENT)
			return PTR_ERR(intr->clk_bside);

		intr->clk_bside = NULL;
	}

	ret = clk_prepare_enable(intr->clk_aside);
	if (ret) {
		dev_err(dev, "Failed to enable clk_a clock\n");
		return ret;
	}

	ret = clk_prepare_enable(intr->clk_bside);
	if (ret) {
		dev_err(dev, "Failed to enable clk_b clock\n");
		return ret;
	}

	intr->pd_aside = dev_pm_domain_attach_by_name(dev, "pd_a");
	if (IS_ERR(intr->pd_aside)) {
		if (PTR_ERR(intr->pd_aside) != -ENOENT)
			return PTR_ERR(intr->pd_aside);

		intr->pd_aside = NULL;
	}
	link = device_link_add(dev, intr->pd_aside,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME |
			       DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(link)) {
		dev_err(dev, "Failed to add device_link to pd_a: %ld\n", PTR_ERR(link));
		return PTR_ERR(link);
	}

	intr->pd_bside = dev_pm_domain_attach_by_name(dev, "pd_b");
	if (IS_ERR(intr->pd_bside)) {
		if (PTR_ERR(intr->pd_bside) != -ENOENT)
			return PTR_ERR(intr->pd_bside);

		intr->pd_bside = NULL;
	}
	link = device_link_add(dev, intr->pd_bside,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME |
			       DL_FLAG_RPM_ACTIVE);
	if (IS_ERR(link)) {
		dev_err(dev, "Failed to add device_link to pd_b: %ld\n", PTR_ERR(link));
		return PTR_ERR(link);
	}

	parse_doorbell_info(intr);
	platform_set_drvdata(pdev, intr);

	return 0;
}

static int imx_mic_intr_remove(struct platform_device *pdev)
{
	struct imx_mic_intr *intr = platform_get_drvdata(pdev);

	clk_disable_unprepare(intr->clk_aside);
	clk_disable_unprepare(intr->clk_bside);

	return 0;
}

static const struct of_device_id imx_mic_intr_dt_ids[] = {
	{ .compatible = "fsl,imx-mic-intr" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_mic_intr_dt_ids);

static struct platform_driver imx_mic_intr_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "imx-mic-intr",
		   .of_match_table = imx_mic_intr_dt_ids,
		   },
	.probe = imx_mic_intr_probe,
	.remove = imx_mic_intr_remove,
};

module_platform_driver(imx_mic_intr_driver);

MODULE_AUTHOR("NXP Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX MIC Interrupt based on MU");
