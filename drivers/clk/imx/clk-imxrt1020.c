/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2023
 * Author(s): Vladimir Skvortsov <vskvortsov@emcraft.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/imxrt1020-clock.h>

#include "clk.h"

static const char * const pll2_bypass_sels[] = {"pll2_sys", "osc", };
static const char * const pll3_bypass_sels[] = {"pll3_usb_otg", "osc", };

static const char *const pre_periph_sels[] = { "pll2_sys", "pll2_pfd3_297m", "pll3_pfd3_454_74m", "arm_podf", };
static const char *const periph_sels[] = { "pre_periph_sel", "todo", };
static const char *const usdhc_sels[] = { "pll2_pfd2_396m", "pll2_pfd0_352m", };
static const char *const lpuart_sels[] = { "pll3_80m", "osc", };
static const char *const semc_alt_sels[] = { "pll2_pfd2_396m", "pll3_pfd1_664_62m", };
static const char *const semc_sels[] = { "periph_sel", "semc_alt_sel", };

static const char * const perclk_sels[] = {"ipg", "osc", };

static struct clk_hw **hws;
static struct clk_hw_onecell_data *clk_hw_data;

static int imxrt1020_clk_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *anp;
	int ret;
	int i;

	clk_hw_data = kzalloc(struct_size(clk_hw_data, hws,
					  IMXRT1020_CLK_END), GFP_KERNEL);

	if (WARN_ON(!clk_hw_data))
		return -ENOMEM;

	clk_hw_data->num = IMXRT1020_CLK_END;
	hws = clk_hw_data->hws;

	hws[IMXRT1020_CLK_DUMMY] = imx_clk_hw_fixed("dummy", 0UL);

	hws[IMXRT1020_CLK_OSC] = imx_obtain_fixed_clk_hw(np, "osc");

	/* Anatop clocks */
	anp = of_find_compatible_node(NULL, NULL, "fsl,imxrt-anatop");
	base = of_iomap(anp, 0);
	of_node_put(anp);
	if (WARN_ON(!base))
		return -ENOMEM;

	hws[IMXRT1020_CLK_PLL2_SYS] = imx_clk_hw_pllv3(IMX_PLLV3_GENERIC, "pll2_sys", "osc",
		base + 0x30, 0x1);
	hws[IMXRT1020_CLK_PLL3_USB_OTG] = imx_clk_hw_pllv3(IMX_PLLV3_USB, "pll3_usb_otg", "osc",
		base + 0x10, 0x1);
	/* PLL bypass out */
	hws[IMXRT1020_CLK_PLL2_BYPASS] = imx_clk_hw_mux_flags("pll2_bypass",
		base + 0x30, 16, 1,
		pll2_bypass_sels, ARRAY_SIZE(pll2_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMXRT1020_CLK_PLL3_BYPASS] = imx_clk_hw_mux_flags("pll3_bypass",
		base + 0x10, 16, 1,
		pll3_bypass_sels, ARRAY_SIZE(pll3_bypass_sels), CLK_SET_RATE_PARENT);

	hws[IMXRT1020_CLK_PLL3_80M] = imx_clk_hw_fixed_factor("pll3_80m",  "pll3_usb_otg", 1, 6);

	hws[IMXRT1020_CLK_PLL2_PFD0_352M] = imx_clk_hw_pfd("pll2_pfd0_352m", "pll2_sys",
		base + 0x100, 0);
	hws[IMXRT1020_CLK_PLL2_PFD1_594M] = imx_clk_hw_pfd("pll2_pfd1_594m", "pll2_sys",
		base + 0x100, 1);
	hws[IMXRT1020_CLK_PLL2_PFD2_396M] = imx_clk_hw_pfd("pll2_pfd2_396m", "pll2_sys",
		base + 0x100, 2);
	hws[IMXRT1020_CLK_PLL2_PFD3_297M] = imx_clk_hw_pfd("pll2_pfd3_297m", "pll2_sys",
		base + 0x100, 3);
	hws[IMXRT1020_CLK_PLL3_PFD1_664_62M] = imx_clk_hw_pfd("pll3_pfd1_664_62m", "pll3_usb_otg",
		base + 0xf0, 1);
	hws[IMXRT1020_CLK_PLL3_PFD3_454_74M] = imx_clk_hw_pfd("pll3_pfd3_454_74m", "pll3_usb_otg",
		base + 0xf0, 3);

	/* CCM clocks */
	base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(base)))
		return PTR_ERR(base);

	hws[IMXRT1020_CLK_PRE_PERIPH_SEL] = imx_clk_hw_mux("pre_periph_sel",
		base + 0x18, 18, 2,
		pre_periph_sels, ARRAY_SIZE(pre_periph_sels));

	hws[IMXRT1020_CLK_PERIPH_SEL] = imx_clk_hw_mux("periph_sel",
		base + 0x14, 25, 1,
		periph_sels, ARRAY_SIZE(periph_sels));

	hws[IMXRT1020_CLK_USDHC1_SEL] = imx_clk_hw_mux("usdhc1_sel",
		base + 0x1c, 16, 1,
		usdhc_sels, ARRAY_SIZE(usdhc_sels));
	hws[IMXRT1020_CLK_USDHC2_SEL] = imx_clk_hw_mux("usdhc2_sel",
		base + 0x1c, 17, 1,
		usdhc_sels, ARRAY_SIZE(usdhc_sels));
	hws[IMXRT1020_CLK_LPUART_SEL] = imx_clk_hw_mux("lpuart_sel",
		base + 0x24, 6, 1,
		lpuart_sels, ARRAY_SIZE(lpuart_sels));
	hws[IMXRT1020_CLK_SEMC_ALT_SEL] = imx_clk_hw_mux("semc_alt_sel",
		base + 0x14, 7, 1,
		semc_alt_sels, ARRAY_SIZE(semc_alt_sels));
	hws[IMXRT1020_CLK_SEMC_SEL] = imx_clk_hw_mux_flags("semc_sel",
		base + 0x14, 6, 1,
		semc_sels, ARRAY_SIZE(semc_sels), CLK_IS_CRITICAL);

	hws[IMXRT1020_CLK_AHB_PODF] = imx_clk_hw_divider("ahb", "periph_sel",
		base + 0x14, 10, 3);
	hws[IMXRT1020_CLK_IPG_PODF] = imx_clk_hw_divider("ipg", "ahb",
		base + 0x14, 8, 2);

	hws[IMXRT1020_CLK_PER_CLK_SEL] = imx_clk_hw_mux("perclk_sel",
		base + 0x1c, 6, 1,
		perclk_sels, ARRAY_SIZE(perclk_sels));

	hws[IMXRT1020_CLK_PER_PODF] = imx_clk_hw_divider("per", "perclk_sel",
		base + 0x1c, 0, 5);

	hws[IMXRT1020_CLK_USDHC1_PODF] = imx_clk_hw_divider("usdhc1_podf", "usdhc1_sel",
		base + 0x24, 11, 3);
	hws[IMXRT1020_CLK_USDHC2_PODF] = imx_clk_hw_divider("usdhc2_podf", "usdhc2_sel",
		base + 0x24, 16, 3);
	hws[IMXRT1020_CLK_LPUART_PODF] = imx_clk_hw_divider("lpuart_podf", "lpuart_sel",
		base + 0x24, 0, 6);
	hws[IMXRT1020_CLK_SEMC_PODF] = imx_clk_hw_divider("semc_podf", "semc_sel",
		base + 0x14, 16, 3);

	hws[IMXRT1020_CLK_USDHC1] = imx_clk_hw_gate2("usdhc1", "usdhc1_podf", base + 0x80, 2);
	hws[IMXRT1020_CLK_USDHC2] = imx_clk_hw_gate2("usdhc2", "usdhc2_podf", base + 0x80, 4);
	hws[IMXRT1020_CLK_LPUART1] = imx_clk_hw_gate2("lpuart1", "lpuart_podf", base + 0x7c, 24);
	hws[IMXRT1020_CLK_SEMC] = imx_clk_hw_gate2("semc", "semc_podf", base + 0x74, 4);
	hws[IMXRT1020_CLK_USBOH3] = imx_clk_hw_gate2("usboh3", "ipg", base + 0x80, 0);

	imx_check_clk_hws(hws, IMXRT1020_CLK_END);

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_hw_data);
	if (ret < 0) {
		imx_unregister_hw_clocks(hws, IMXRT1020_CLK_END);
	}

	return 0;
}

static const struct of_device_id imxrt1020_clk_of_match[] = {
	{ .compatible = "fsl,imxrt1020-ccm" },
	{ /* Sentinel */ }
};

static struct platform_driver imxrt1020_clk_driver = {
	.driver = {
		.name = "imxrt1020-ccm",
		.of_match_table = imxrt1020_clk_of_match,
	},
	.probe = imxrt1020_clk_probe,
};
module_platform_driver(imxrt1020_clk_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP i.MXRT1020 clock driver");
MODULE_AUTHOR("Vladimir Skvortsov <vskvortsov@emcraft.com>");
