/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2023 Emcraft Systems
 * Author(s): Vladimir Skvortsov <vskvortsov@emcraft.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <dt-bindings/clock/imxrt1170-clock.h>

#include "clk.h"

#define IMXRT1170_CLK_SRC_COMMON "rcosc48M_div2", "osc", "rcosc400M", "rcosc16M"

static const char * const m7_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll_arm", "pll1_sys", "pll3_sys", "video_pll"};
static const char * const bus_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_sys", "pll1_div5", "pll2_sys", "pll2_pfd3"};
static const char * const bus_lpsr_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_pfd3", "pll3_sys", "pll2_sys", "pll1_div5"};
static const char * const lpuart1_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_div2", "pll1_div5", "pll2_sys", "pll2_pfd3"};
static const char * const gpt1_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_div2", "pll1_div5", "pll3_pfd2", "pll3_pfd3"};
static const char * const usdhc1_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll2_pfd2", "pll2_pfd0", "pll1_div5", "pll_arm"};
static const char * const semc_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll1_div5", "pll2_sys", "pll2_pfd1", "pll3_pfd0"};
static const char * const enet_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll1_div2", "audio_pll", "pll1_div5", "pll2_pfd1"};
static const char * const lpi2c1_4_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_div2", "pll1_div5", "pll2_sys", "pll2_pfd3"};
static const char * const lpi2c5_6_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_pfd3", "pll3_sys", "pll2_pfd3", "pll1_div5"};
static const char * const elcdif_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll2_sys", "pll2_pfd2", "pll3_pfd0", "video_pll"};
static const char * const mipi_dsi_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll2_sys", "pll2_pfd0", "pll3_pfd0", "video_pll"};
static const char * const flexspi_sels[] = {IMXRT1170_CLK_SRC_COMMON,
"pll3_pfd0", "pll2_sys", "pll2_pfd2", "pll3_sys"};

struct clk_hw *imxrt1170_clk_pll_div_out_composite(const char *name, const char *parent_name,
						void __iomem *reg, int div_factor, int gate_bit, unsigned long flags)
{
	struct clk_hw *hw = ERR_PTR(-ENOMEM);
	struct clk_fixed_factor *div = NULL;
	struct clk_gate *gate = NULL;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		goto fail;

	div->mult = 1;
	div->div = div_factor;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		goto fail;

	gate->reg = reg;
	gate->bit_idx = gate_bit;
	gate->flags = flags;

	hw = clk_hw_register_composite(NULL, name, &parent_name, 1,
					NULL, NULL,
					&div->hw, &clk_fixed_factor_ops,
					&gate->hw, &clk_gate_ops, flags);
	if (IS_ERR(hw))
		goto fail;

	return hw;

fail:
	kfree(gate);
	kfree(div);
	return ERR_CAST(hw);
}


struct imxrt1170_clk_root {
	u32 clk_id;
	char *name;
	const char * const *parent_names;
	u32 off;
	unsigned long flags;
};

static struct imxrt1170_clk_root clk_roots[] = {
	{ IMXRT1170_CLK_ROOT_M7, "m7_root", m7_sels, 0, CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_ROOT_BUS, "bus_root", bus_sels, (2 * 0x80), CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_ROOT_BUS_LPSR, "bus_lpsr_root", bus_lpsr_sels, (3 * 0x80), CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_ROOT_SEMC, "semc_root", semc_sels, (4 * 0x80), CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_ROOT_GPT1, "gpt1_root", gpt1_sels, (14 * 0x80), },
	{ IMXRT1170_CLK_ROOT_FLEXSPI1, "flexspi1_root", flexspi_sels, (20 * 0x80), },
	{ IMXRT1170_CLK_ROOT_FLEXSPI2, "flexspi2_root", flexspi_sels, (21 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPUART1, "lpuart1_root", lpuart1_sels, (25 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C1, "lpi2c1_root", lpi2c1_4_sels, (37 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C2, "lpi2c2_root", lpi2c1_4_sels, (38 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C3, "lpi2c3_root", lpi2c1_4_sels, (39 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C4, "lpi2c4_root", lpi2c1_4_sels, (40 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C5, "lpi2c5_root", lpi2c5_6_sels, (41 * 0x80), },
	{ IMXRT1170_CLK_ROOT_LPI2C6, "lpi2c6_root", lpi2c5_6_sels, (42 * 0x80), },
	{ IMXRT1170_CLK_ROOT_ENET1, "enet1_root", enet_sels, (51 * 0x80), },
	{ IMXRT1170_CLK_ROOT_ENET2, "enet2_root", enet_sels, (52 * 0x80), },
	{ IMXRT1170_CLK_ROOT_USDHC1, "usdhc1_root", usdhc1_sels, (58 * 0x80), },
	{ IMXRT1170_CLK_ROOT_ELCDIF, "elcdif_root", elcdif_sels, (69 * 0x80), },
	{ IMXRT1170_CLK_ROOT_MIPI_REF, "mipi_ref_root", mipi_dsi_sels, (71 * 0x80), },
	{ IMXRT1170_CLK_ROOT_MIPI_ESC, "mipi_esc_root", mipi_dsi_sels, (72 * 0x80), },
};

struct imxrt1170_clk_ccgr {
	u32 clk_id;
	char *name;
	char *parent_names;
	u32 off;
	unsigned long flags;
};

static struct imxrt1170_clk_ccgr clk_ccgrs[] = {
	{ IMXRT1170_CLK_M7, "m7", "m7_root", 0x6000, CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_EDMA, "edma", "bus_root", (0x6000 + (20 * 0x20)) },
	{ IMXRT1170_CLK_SEMC, "semc", "semc_root", (0x6000 + (33 * 0x20)), CLK_IS_CRITICAL },
	{ IMXRT1170_CLK_GPT1, "gpt1", "gpt1_root", (0x6000 + (64 * 0x20)), },
	{ IMXRT1170_CLK_FLEXSPI1, "flexspi1", "flexspi1_root", (0x6000 + (28 * 0x20)), },
	{ IMXRT1170_CLK_FLEXSPI2, "flexspi2", "flexspi2_root", (0x6000 + (29 * 0x20)), },
	{ IMXRT1170_CLK_LPUART1, "lpuart1", "lpuart1_root", (0x6000 + (86 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C1, "lpi2c1", "lpi2c1_root", (0x6000 + (98 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C2, "lpi2c2", "lpi2c2_root", (0x6000 + (99 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C3, "lpi2c3", "lpi2c3_root", (0x6000 + (100 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C4, "lpi2c4", "lpi2c4_root", (0x6000 + (101 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C5, "lpi2c5", "lpi2c5_root", (0x6000 + (102 * 0x20)), },
	{ IMXRT1170_CLK_LPI2C6, "lpi2c6", "lpi2c6_root", (0x6000 + (103 * 0x20)), },
	{ IMXRT1170_CLK_ENET1, "enet1", "enet1_root", (0x6000 + (112 * 0x20)), },
	{ IMXRT1170_CLK_ENET2, "enet2", "enet2_root", (0x6000 + (113 * 0x20)), },
	{ IMXRT1170_CLK_USB, "usb", "bus_root", (0x6000 + (115 * 0x20)), },
	{ IMXRT1170_CLK_USDHC1, "usdhc1", "usdhc1_root", (0x6000 + (117 * 0x20)), },
	{ IMXRT1170_CLK_ELCDIF, "elcdif", "elcdif_root", (0x6000 + (129 * 0x20)), },
	{ IMXRT1170_CLK_MIPI_DSI, "mipi_dsi", "mipi_ref_root", (0x6000 + (131 * 0x20)), },
};

static struct clk_hw **hws;
static struct clk_hw_onecell_data *clk_hw_data;

/* base address of the CLOCK_GROUPn_CONTROL register */
#define IMXRT1170_CCM_CLOCK_GROUP_CONTROL_SET(base, grp_id) (base + 0x4000 + ((grp_id) * 0x80))
#define CGC_DIV0_SHIFT		0
#define CGC_RSTDIV_SHIFT	16
#define CGC_OFF_SHIFT		24

static void __init imxrt1170_clocks_init(struct device_node *ccm_node)
{
	void __iomem *base;
	struct device_node *np;
	struct device_node *anp;
	struct imxrt1170_clk_root *root;
	struct imxrt1170_clk_ccgr *ccgr;
	int ret, i;
	u32 group_control;

	clk_hw_data = kzalloc(struct_size(clk_hw_data, hws,
					  IMXRT1170_CLK_END), GFP_KERNEL);

	WARN_ON(!clk_hw_data);

	clk_hw_data->num = IMXRT1170_CLK_END;
	hws = clk_hw_data->hws;

	hws[IMXRT1170_CLK_DUMMY] = imx_clk_hw_fixed("dummy", 0);
	hws[IMXRT1170_CLK_OSC] = imx_get_clk_hw_by_name(ccm_node, "osc");
	hws[IMXRT1170_CLK_RCOSC_16M] = imx_get_clk_hw_by_name(ccm_node, "rcosc16M");

	/* Anatop clocks */
	anp = of_find_compatible_node(NULL, NULL, "fsl,imxrt-anatop");
	base = of_iomap(anp, 0);
	of_node_put(anp);
	WARN_ON(!base);

	hws[IMXRT1170_CLK_RCOSC_48M] =
	       imx_clk_hw_fixed_factor("rcosc48M", "rcosc16M", 3, 1);
	hws[IMXRT1170_CLK_RCOSC_400M] =
	       imx_clk_hw_fixed_factor("rcosc400M",  "rcosc16M", 25, 1);
	hws[IMXRT1170_CLK_RCOSC_48M_DIV2] =
	       imx_clk_hw_fixed_factor("rcosc48M_div2",  "rcosc48M", 1, 2);

	hws[IMXRT1170_CLK_PLL_ARM] =
	       imx_clk_hw_pll_rt1170(IMXRT1170_PLLARM, "pll_arm", "osc", base + 0x200);

	hws[IMXRT1170_CLK_PLL3] =
	       imx_clk_hw_pll_rt1170(IMXRT1170_PLL3, "pll3_sys", "osc", base + 0x210);
	hws[IMXRT1170_CLK_PLL2] =
	       imx_clk_hw_pll_rt1170(IMXRT1170_PLL2, "pll2_sys", "osc", base + 0x240);
	hws[IMXRT1170_CLK_PLL1] =
	       imx_clk_hw_pll_rt1170(IMXRT1170_PLL1, "pll1_sys", "osc", base + 0x2c0);

	hws[IMXRT1170_CLK_PLL3_PFD0] =
	       imx_clk_hw_pfd("pll3_pfd0", "pll3_sys", base + 0x230, 0);
	hws[IMXRT1170_CLK_PLL3_PFD1] =
	       imx_clk_hw_pfd("pll3_pfd1", "pll3_sys", base + 0x230, 1);
	hws[IMXRT1170_CLK_PLL3_PFD2] =
	       imx_clk_hw_pfd("pll3_pfd2", "pll3_sys", base + 0x230, 2);
	hws[IMXRT1170_CLK_PLL3_PFD3] =
	       imx_clk_hw_pfd("pll3_pfd3", "pll3_sys", base + 0x230, 3);

	hws[IMXRT1170_CLK_PLL2_PFD0] =
	       imx_clk_hw_pfd("pll2_pfd0", "pll2_sys", base + 0x270, 0);
	hws[IMXRT1170_CLK_PLL2_PFD1] =
	       imx_clk_hw_pfd("pll2_pfd1", "pll2_sys", base + 0x270, 1);
	hws[IMXRT1170_CLK_PLL2_PFD2] =
	       imx_clk_hw_pfd("pll2_pfd2", "pll2_sys", base + 0x270, 2);
	hws[IMXRT1170_CLK_PLL2_PFD3] =
	       imx_clk_hw_pfd("pll2_pfd3", "pll2_sys", base + 0x270, 3);

	hws[IMXRT1170_CLK_PLL3_DIV2] =
	       imxrt1170_clk_pll_div_out_composite("pll3_div2", "pll3_sys", base + 0x210, 2, 3, 0);

	hws[IMXRT1170_CLK_PLL1_DIV2] =
	       imxrt1170_clk_pll_div_out_composite("pll1_div2", "pll1_sys", base + 0x2c0, 2, 25, 0);
	hws[IMXRT1170_CLK_PLL1_DIV5] =
	       imxrt1170_clk_pll_div_out_composite("pll1_div5", "pll1_sys", base + 0x2c0, 5, 26, 0);

	/* CCM clocks */
	np = ccm_node;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	for (i = 0; i < ARRAY_SIZE(clk_roots); i++) {
		root = &clk_roots[i];
		hws[root->clk_id] = imx93_clk_composite_flags(root->name,
							      root->parent_names,
							      8, base + root->off, 3,
							      root->flags);
	}

	for (i = 0; i < ARRAY_SIZE(clk_ccgrs); i++) {
		ccgr = &clk_ccgrs[i];
		hws[ccgr->clk_id] = imx_clk_hw_gate_flags(ccgr->name, ccgr->parent_names,
							  base + ccgr->off, 0, ccgr->flags);
	}

	/* Hardcode divisor = 2 for the MIPI DSI tx_esc clock in the clock group control #1 */
	group_control = (1 << CGC_DIV0_SHIFT) | (1 << CGC_RSTDIV_SHIFT) | (0 << CGC_OFF_SHIFT);
	writel_relaxed(group_control, IMXRT1170_CCM_CLOCK_GROUP_CONTROL_SET(base, 1));
	hws[IMXRT1170_CLK_MIPI_DSI_TX_ESC] = imx_clk_hw_fixed_factor("mipi_tx_esc",  "mipi_esc_root", 1, 2);

	imx_check_clk_hws(hws, IMXRT1170_CLK_END);

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_hw_data);
	if (ret < 0) {
		imx_unregister_hw_clocks(hws, IMXRT1170_CLK_END);
	}
}

CLK_OF_DECLARE(imxrt1170, "fsl,imxrt1170-ccm", imxrt1170_clocks_init);
