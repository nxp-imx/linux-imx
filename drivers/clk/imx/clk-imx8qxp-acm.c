// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 *	Dong Aisheng <aisheng.dong@nxp.com>
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>

#include "clk.h"
#include "clk-scu.h"

#include <dt-bindings/clock/imx8-clock.h>

struct imx8qxp_acm_priv {
	void __iomem *reg;
	u32 regs[0x20];
};

static const char *aud_clk_sels[] = {
	"aud_rec_clk0_lpcg_clk",
	"aud_rec_clk1_lpcg_clk",
	"ext_aud_mclk0",
	"ext_aud_mclk1",
	"esai0_rx_clk",
	"esai0_rx_hf_clk",
	"esai0_tx_clk",
	"esai0_tx_hf_clk",
	"spdif0_rx",
	"sai0_rx_bclk",
	"sai0_tx_bclk",
	"sai1_rx_bclk",
	"sai1_tx_bclk",
	"sai2_rx_bclk",
	"sai3_rx_bclk",
};

static const char *mclk_out_sels[] = {
	"aud_rec_clk0_lpcg_clk",
	"aud_rec_clk1_lpcg_clk",
	"dummy",
	"dummy",
	"spdif0_rx",
	"dummy",
	"dummy",
	"sai4_rx_bclk",
};

static const char *sai_mclk_sels[] = {
	"aud_pll_div_clk0_lpcg_clk",
	"aud_pll_div_clk1_lpcg_clk",
	"acm_aud_clk0_sel",
	"acm_aud_clk1_sel",
};

static const char *esai_mclk_sels[] = {
	"aud_pll_div_clk0_lpcg_clk",
	"aud_pll_div_clk1_lpcg_clk",
	"acm_aud_clk0_sel",
	"acm_aud_clk1_sel",
};

static const char *spdif_mclk_sels[] = {
	"aud_pll_div_clk0_lpcg_clk",
	"aud_pll_div_clk1_lpcg_clk",
	"acm_aud_clk0_sel",
	"acm_aud_clk1_sel",
};

static const char *mqs_mclk_sels[] = {
	"aud_pll_div_clk0_lpcg_clk",
	"aud_pll_div_clk1_lpcg_clk",
	"acm_aud_clk0_sel",
	"acm_aud_clk1_sel",
};

static int imx8qxp_acm_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct clk_onecell_data *clk_data;
	struct imx8qxp_acm_priv *priv;
	struct resource *res;
	struct clk **clks;
	void __iomem *base;
	int num_domains;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = base;

	platform_set_drvdata(pdev, priv);

	clk_data = devm_kzalloc(&pdev->dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->clks = devm_kcalloc(&pdev->dev, IMX_ADMA_ACM_CLK_END,
					sizeof(*clk_data->clks), GFP_KERNEL);
	if (!clk_data->clks)
		return -ENOMEM;

	clk_data->clk_num = IMX_ADMA_ACM_CLK_END;

	clks = clk_data->clks;

	num_domains = of_count_phandle_with_args(dev->of_node, "power-domains",
						 "#power-domain-cells");
	for (i = 0; i < num_domains; i++) {
		struct device *pd_dev;
		struct device_link *link;

		pd_dev = dev_pm_domain_attach_by_id(&pdev->dev, i);
		if (IS_ERR(pd_dev))
			return PTR_ERR(pd_dev);

		link = device_link_add(&pdev->dev, pd_dev,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(link))
			return PTR_ERR(link);
	}

	clks[IMX_ADMA_EXT_AUD_MCLK0]     = imx_clk_fixed("ext_aud_mclk0", 0);
	clks[IMX_ADMA_EXT_AUD_MCLK1]     = imx_clk_fixed("ext_aud_mclk1", 0);
	clks[IMX_ADMA_ESAI0_RX_CLK]      = imx_clk_fixed("esai0_rx_clk", 0);
	clks[IMX_ADMA_ESAI0_RX_HF_CLK]   = imx_clk_fixed("esai0_rx_hf_clk", 0);
	clks[IMX_ADMA_ESAI0_TX_CLK]      = imx_clk_fixed("esai0_tx_clk", 0);
	clks[IMX_ADMA_ESAI0_TX_HF_CLK]   = imx_clk_fixed("esai0_tx_hf_clk", 0);
	clks[IMX_ADMA_SPDIF0_RX]         = imx_clk_fixed("spdif0_rx", 0);
	clks[IMX_ADMA_SAI0_RX_BCLK]      = imx_clk_fixed("sai0_rx_bclk", 0);
	clks[IMX_ADMA_SAI0_TX_BCLK]      = imx_clk_fixed("sai0_tx_bclk", 0);
	clks[IMX_ADMA_SAI1_RX_BCLK]      = imx_clk_fixed("sai1_rx_bclk", 0);
	clks[IMX_ADMA_SAI1_TX_BCLK]      = imx_clk_fixed("sai1_tx_bclk", 0);
	clks[IMX_ADMA_SAI2_RX_BCLK]      = imx_clk_fixed("sai2_rx_bclk", 0);
	clks[IMX_ADMA_SAI3_RX_BCLK]      = imx_clk_fixed("sai3_rx_bclk", 0);
	clks[IMX_ADMA_SAI4_RX_BCLK]      = imx_clk_fixed("sai4_rx_bclk", 0);


	clks[IMX_ADMA_ACM_AUD_CLK0_SEL] = imx_clk_mux("acm_aud_clk0_sel", base+0x000000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels));
	clks[IMX_ADMA_ACM_AUD_CLK1_CLK]	= imx_clk_mux("acm_aud_clk1_sel", base+0x010000, 0, 5, aud_clk_sels, ARRAY_SIZE(aud_clk_sels));

	clks[IMX_ADMA_ACM_MCLKOUT0_SEL]	= imx_clk_mux("acm_mclkout0_sel", base+0x020000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels));
	clks[IMX_ADMA_ACM_MCLKOUT1_SEL]	= imx_clk_mux("acm_mclkout1_sel", base+0x030000, 0, 3, mclk_out_sels, ARRAY_SIZE(mclk_out_sels));

	clks[IMX_ADMA_ACM_ESAI0_MCLK_SEL] = imx_clk_mux("acm_esai0_mclk_sel", base+0x060000, 0, 2, esai_mclk_sels, ARRAY_SIZE(esai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI0_MCLK_SEL] = imx_clk_mux("acm_sai0_mclk_sel", base+0x0E0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI1_MCLK_SEL] = imx_clk_mux("acm_sai1_mclk_sel", base+0x0F0000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI2_MCLK_SEL] = imx_clk_mux("acm_sai2_mclk_sel", base+0x100000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI3_MCLK_SEL] = imx_clk_mux("acm_sai3_mclk_sel", base+0x110000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI4_MCLK_SEL] = imx_clk_mux("acm_sai4_mclk_sel", base+0x140000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));
	clks[IMX_ADMA_ACM_SAI5_MCLK_SEL] = imx_clk_mux("acm_sai5_mclk_sel", base+0x150000, 0, 2, sai_mclk_sels, ARRAY_SIZE(sai_mclk_sels));

	clks[IMX_ADMA_ACM_SPDIF0_TX_CLK_SEL] = imx_clk_mux("acm_spdif0_mclk_sel", base+0x1A0000, 0, 2, spdif_mclk_sels, ARRAY_SIZE(spdif_mclk_sels));
	clks[IMX_ADMA_ACM_MQS_TX_CLK_SEL] = imx_clk_mux("acm_mqs_mclk_sel", base+0x1C0000, 0, 2, mqs_mclk_sels, ARRAY_SIZE(mqs_mclk_sels));

	for (i = 0; i < clk_data->clk_num; i++) {
		if (IS_ERR(clks[i]))
			pr_warn("i.MX clk %u: register failed with %ld\n",
				i, PTR_ERR(clks[i]));
	}

	return of_clk_add_provider(np, of_clk_src_onecell_get, clk_data);
}

static const struct of_device_id imx8qxp_acm_match[] = {
	{ .compatible = "nxp,imx8qxp-acm", },
	{ /* sentinel */ }
};

static int __maybe_unused imx8qxp_acm_suspend(struct device *dev)
{
	struct imx8qxp_acm_priv *priv = dev_get_drvdata(dev);

	priv->regs[0]  = readl_relaxed(priv->reg + 0x000000);
	priv->regs[1]  = readl_relaxed(priv->reg + 0x010000);
	priv->regs[2]  = readl_relaxed(priv->reg + 0x020000);
	priv->regs[3]  = readl_relaxed(priv->reg + 0x030000);
	priv->regs[6]  = readl_relaxed(priv->reg + 0x060000);
	priv->regs[14] = readl_relaxed(priv->reg + 0x0E0000);
	priv->regs[15] = readl_relaxed(priv->reg + 0x0F0000);
	priv->regs[16] = readl_relaxed(priv->reg + 0x100000);
	priv->regs[17] = readl_relaxed(priv->reg + 0x110000);
	priv->regs[20] = readl_relaxed(priv->reg + 0x140000);
	priv->regs[21] = readl_relaxed(priv->reg + 0x150000);
	priv->regs[26] = readl_relaxed(priv->reg + 0x1A0000);
	priv->regs[28] = readl_relaxed(priv->reg + 0x1C0000);

	return 0;
}

static int __maybe_unused imx8qxp_acm_resume(struct device *dev)
{
	struct imx8qxp_acm_priv *priv = dev_get_drvdata(dev);

	writel_relaxed(priv->regs[0],  priv->reg + 0x000000);
	writel_relaxed(priv->regs[1],  priv->reg + 0x010000);
	writel_relaxed(priv->regs[2],  priv->reg + 0x020000);
	writel_relaxed(priv->regs[3],  priv->reg + 0x030000);
	writel_relaxed(priv->regs[6],  priv->reg + 0x060000);
	writel_relaxed(priv->regs[14], priv->reg + 0x0E0000);
	writel_relaxed(priv->regs[15], priv->reg + 0x0F0000);
	writel_relaxed(priv->regs[16], priv->reg + 0x100000);
	writel_relaxed(priv->regs[17], priv->reg + 0x110000);
	writel_relaxed(priv->regs[20], priv->reg + 0x140000);
	writel_relaxed(priv->regs[21], priv->reg + 0x150000);
	writel_relaxed(priv->regs[26], priv->reg + 0x1A0000);
	writel_relaxed(priv->regs[28], priv->reg + 0x1C0000);

	return 0;
}

const struct dev_pm_ops imx8qxp_acm_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(imx8qxp_acm_suspend,
				      imx8qxp_acm_resume)
};

static struct platform_driver imx8qxp_acm_clk_driver = {
	.driver = {
		.name = "imx8qxp-acm",
		.of_match_table = imx8qxp_acm_match,
		.pm = &imx8qxp_acm_pm_ops,
		.suppress_bind_attrs = true,
	},
	.probe = imx8qxp_acm_clk_probe,
};

static int __init imx8qxp_acm_init(void)
{
	return platform_driver_register(&imx8qxp_acm_clk_driver);
}
fs_initcall(imx8qxp_acm_init);
