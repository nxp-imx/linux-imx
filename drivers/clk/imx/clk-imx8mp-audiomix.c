// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for i.MX8M Plus Audio BLK_CTRL
 *
 * Copyright (C) 2022 Marek Vasut <marex@denx.de>
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset-controller.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/imx8mp-clock.h>

#include "clk.h"

#define IMX8MP_DSP_NOC_OFFSET	0x208
#define IMX8MP_DSP_NOC_PRIORITY	0x80000404

#define IMX8MP_AUDIO_BLK_CTRL_EARC_RESET	0
#define IMX8MP_AUDIO_BLK_CTRL_EARC_PHY_RESET	1
#define IMX8MP_AUDIO_BLK_CTRL_RESET_NUM		2

#define CLKEN0			0x000
#define CLKEN1			0x004
#define SAI1_MCLK_SEL		0x300
#define SAI2_MCLK_SEL		0x304
#define SAI3_MCLK_SEL		0x308
#define SAI5_MCLK_SEL		0x30C
#define SAI6_MCLK_SEL		0x310
#define SAI7_MCLK_SEL		0x314
#define PDM_SEL			0x318
#define SAI_PLL_GNRL_CTL	0x400

#define AUDIOMIX_SAI1_IPG            0
#define AUDIOMIX_SAI1_MCLK1          1
#define AUDIOMIX_SAI1_MCLK2          2
#define AUDIOMIX_SAI1_MCLK3          3
#define AUDIOMIX_SAI2_IPG            4
#define AUDIOMIX_SAI2_MCLK1          5
#define AUDIOMIX_SAI2_MCLK2          6
#define AUDIOMIX_SAI2_MCLK3          7
#define AUDIOMIX_SAI3_IPG            8
#define AUDIOMIX_SAI3_MCLK1          9
#define AUDIOMIX_SAI3_MCLK2          10
#define AUDIOMIX_SAI3_MCLK3          11
#define AUDIOMIX_SAI5_IPG            12
#define AUDIOMIX_SAI5_MCLK1          13
#define AUDIOMIX_SAI5_MCLK2          14
#define AUDIOMIX_SAI5_MCLK3          15
#define AUDIOMIX_SAI6_IPG            16
#define AUDIOMIX_SAI6_MCLK1          17
#define AUDIOMIX_SAI6_MCLK2          18
#define AUDIOMIX_SAI6_MCLK3          19
#define AUDIOMIX_SAI7_IPG            20
#define AUDIOMIX_SAI7_MCLK1          21
#define AUDIOMIX_SAI7_MCLK2          22
#define AUDIOMIX_SAI7_MCLK3          23
#define AUDIOMIX_ASRC_IPG            24
#define AUDIOMIX_PDM_IPG             25
#define AUDIOMIX_SDMA2_ROOT          26
#define AUDIOMIX_SDMA3_ROOT          27
#define AUDIOMIX_SPBA2_ROOT          28
#define AUDIOMIX_DSP_ROOT            29
#define AUDIOMIX_DSPDBG_ROOT         30
#define AUDIOMIX_EARC_IPG            31
#define AUDIOMIX_OCRAMA_IPG          32
#define AUDIOMIX_AUD2HTX_IPG         33
#define AUDIOMIX_EDMA_ROOT           34
#define AUDIOMIX_AUDPLL_ROOT         35
#define AUDIOMIX_MU2_ROOT            36
#define AUDIOMIX_MU3_ROOT            37
#define AUDIOMIX_EARC_PHY            38

#define SAIn_MCLK1_PARENT(n)						\
static const struct clk_parent_data					\
clk_imx8mp_audiomix_sai##n##_mclk1_parents[] = {			\
	{								\
		.fw_name = "sai"__stringify(n),				\
		.name = "sai"__stringify(n)				\
	}, {								\
		.fw_name = "sai"__stringify(n)"_mclk",			\
		.name = "sai"__stringify(n)"_mclk"			\
	},								\
}

SAIn_MCLK1_PARENT(1);
SAIn_MCLK1_PARENT(2);
SAIn_MCLK1_PARENT(3);
SAIn_MCLK1_PARENT(5);
SAIn_MCLK1_PARENT(6);
SAIn_MCLK1_PARENT(7);

static const struct clk_parent_data clk_imx8mp_audiomix_sai_mclk2_parents[] = {
	{ .fw_name = "sai1", .name = "sai1" },
	{ .fw_name = "sai2", .name = "sai2" },
	{ .fw_name = "sai3", .name = "sai3" },
	{ .name = "dummy" },
	{ .fw_name = "sai5", .name = "sai5" },
	{ .fw_name = "sai6", .name = "sai6" },
	{ .fw_name = "sai7", .name = "sai7" },
	{ .fw_name = "sai1_mclk", .name = "sai1_mclk" },
	{ .fw_name = "sai2_mclk", .name = "sai2_mclk" },
	{ .fw_name = "sai3_mclk", .name = "sai3_mclk" },
	{ .name = "dummy" },
	{ .fw_name = "sai5_mclk", .name = "sai5_mclk" },
	{ .fw_name = "sai6_mclk", .name = "sai6_mclk" },
	{ .fw_name = "sai7_mclk", .name = "sai7_mclk" },
	{ .fw_name = "spdif_extclk", .name = "spdif_extclk" },
	{ .name = "dummy" },
};

static const struct clk_parent_data clk_imx8mp_audiomix_pdm_parents[] = {
	{ .fw_name = "pdm", .name = "pdm" },
	{ .name = "sai_pll_out_div2" },
	{ .fw_name = "sai1_mclk", .name = "sai1_mclk" },
	{ .name = "dummy" },
};


static const struct clk_parent_data clk_imx8mp_audiomix_pll_parents[] = {
	{ .fw_name = "osc_24m", .name = "osc_24m" },
	{ .name = "dummy" },
	{ .name = "dummy" },
	{ .name = "dummy" },
};

static const struct clk_parent_data clk_imx8mp_audiomix_pll_bypass_sels[] = {
	{ .fw_name = "sai_pll", .name = "sai_pll" },
	{ .fw_name = "sai_pll_ref_sel", .name = "sai_pll_ref_sel" },
};

static int shared_count_pdm;

#define CLK_GATE(gname, cname)						\
	{								\
		gname"_cg",						\
		IMX8MP_CLK_AUDIOMIX_##cname,				\
		{ .fw_name = "ahb", .name = "ahb" }, NULL, 1,		\
		CLKEN0 + 4 * !!(AUDIOMIX_##cname / 32),	\
		1, AUDIOMIX_##cname % 32			\
	}

#define CLK_GATE_SHARED(gname, cname, pname, reg, width, shift, shcount)\
	{								\
		gname,							\
		IMX8MP_CLK_AUDIOMIX_##cname,				\
		pname,							\
		reg, width, shift, shcount				\
	}

#define CLK_SAIn(n)							\
	{								\
		"sai"__stringify(n)"_mclk1_sel",			\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_MCLK1_SEL, {},		\
		clk_imx8mp_audiomix_sai##n##_mclk1_parents,		\
		ARRAY_SIZE(clk_imx8mp_audiomix_sai##n##_mclk1_parents), \
		SAI##n##_MCLK_SEL, 1, 0					\
	}, {								\
		"sai"__stringify(n)"_mclk2_sel",			\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_MCLK2_SEL, {},		\
		clk_imx8mp_audiomix_sai_mclk2_parents,			\
		ARRAY_SIZE(clk_imx8mp_audiomix_sai_mclk2_parents),	\
		SAI##n##_MCLK_SEL, 4, 1					\
	}, {								\
		"sai"__stringify(n)"_ipg_cg",				\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_IPG,			\
		{ .fw_name = "ahb", .name = "ahb" }, NULL, 1,		\
		CLKEN0, 1, AUDIOMIX_SAI##n##_IPG		\
	}, {								\
		"sai"__stringify(n)"_mclk1_cg",				\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_MCLK1,			\
		{							\
			.fw_name = "sai"__stringify(n)"_mclk1_sel",	\
			.name = "sai"__stringify(n)"_mclk1_sel"		\
		}, NULL, 1,						\
		CLKEN0, 1, AUDIOMIX_SAI##n##_MCLK1		\
	}, {								\
		"sai"__stringify(n)"_mclk2_cg",				\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_MCLK2,			\
		{							\
			.fw_name = "sai"__stringify(n)"_mclk2_sel",	\
			.name = "sai"__stringify(n)"_mclk2_sel"		\
		}, NULL, 1,						\
		CLKEN0, 1, AUDIOMIX_SAI##n##_MCLK2		\
	}, {								\
		"sai"__stringify(n)"_mclk3_cg",				\
		IMX8MP_CLK_AUDIOMIX_SAI##n##_MCLK3,			\
		{							\
			.fw_name = "sai_pll_out_div2",			\
			.name = "sai_pll_out_div2"			\
		}, NULL, 1,						\
		CLKEN0, 1, AUDIOMIX_SAI##n##_MCLK3		\
	}

#define CLK_PDM								\
	{								\
		"pdm_sel", IMX8MP_CLK_AUDIOMIX_PDM_SEL, {},		\
		clk_imx8mp_audiomix_pdm_parents,			\
		ARRAY_SIZE(clk_imx8mp_audiomix_pdm_parents),		\
		PDM_SEL, 2, 0						\
	}

#define CLK_GATE_PARENT(gname, cname, pname)						\
	{								\
		gname"_cg",						\
		IMX8MP_CLK_AUDIOMIX_##cname,				\
		{ .fw_name = pname, .name = pname }, NULL, 1,		\
		CLKEN0 + 4 * !!(IMX8MP_CLK_AUDIOMIX_##cname / 32),	\
		1, IMX8MP_CLK_AUDIOMIX_##cname % 32			\
	}

struct clk_imx8mp_audiomix_sel {
	const char			*name;
	int				clkid;
	const struct clk_parent_data	parent;		/* For gate */
	const struct clk_parent_data	*parents;	/* For mux */
	int				num_parents;
	u16				reg;
	u8				width;
	u8				shift;
};

static struct clk_imx8mp_audiomix_sel sels[] = {
	CLK_GATE("asrc", ASRC_IPG),
	CLK_GATE("earc", EARC_IPG),
	CLK_GATE("ocrama", OCRAMA_IPG),
	CLK_GATE("aud2htx", AUD2HTX_IPG),
	CLK_GATE_PARENT("earc_phy", EARC_PHY, "sai_pll_out_div2"),
	CLK_GATE("sdma3", SDMA3_ROOT),
	CLK_GATE("spba2", SPBA2_ROOT),
	CLK_GATE("dsp", DSP_ROOT),
	CLK_GATE("dspdbg", DSPDBG_ROOT),
	CLK_GATE("edma", EDMA_ROOT),
	CLK_GATE_PARENT("audpll", AUDPLL_ROOT, "osc_24m"),
	CLK_GATE("mu2", MU2_ROOT),
	CLK_GATE("mu3", MU3_ROOT),
	CLK_PDM,
	CLK_SAIn(1),
	CLK_SAIn(2),
	CLK_SAIn(3),
	CLK_SAIn(5),
	CLK_SAIn(6),
	CLK_SAIn(7)
};

struct clk_imx8mp_audiomix_shared_gate {
	const char			*name;
	int				clkid;
	const char			*parents;
	u16				reg;
	u8				width;
	u8				shift;
	int				*shcount;
};

static struct clk_imx8mp_audiomix_shared_gate pdms[] = {
	CLK_GATE_SHARED("pdm_ipg_clk", PDM_IPG, "audio_ahb_root", 0, 1, AUDIOMIX_PDM_IPG, &shared_count_pdm),
	CLK_GATE_SHARED("pdm_root_clk", PDM_ROOT, "pdm_sel", 0, 1, AUDIOMIX_PDM_IPG, &shared_count_pdm),
};

struct pm_safekeep_info {
	uint32_t *regs_values;
	uint32_t *regs_offsets;
	uint32_t regs_num;
};

struct reset_hw {
	u32 offset;
	u32 shift;
	u32 mask;
	unsigned long asserted;
};

struct imx_blk_ctrl_drvdata {
	void __iomem *base;
	struct reset_controller_dev rcdev;
	struct reset_hw *rst_hws;
	struct pm_safekeep_info pm_info;
	struct regmap *regmap;

	spinlock_t *lock;
};

struct imx_blk_ctrl_dev_data {
	u32 pm_runtime_saved_regs_num;
	u32 pm_runtime_saved_regs[];
};

static int imx_blk_ctrl_init_runtime_pm_safekeeping(struct device *dev)
{
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct pm_safekeep_info *pm_info = &drvdata->pm_info;
	u32 regs_num = dev_data->pm_runtime_saved_regs_num;
	const u32 *regs_offsets = dev_data->pm_runtime_saved_regs;

	if (!dev_data->pm_runtime_saved_regs_num)
		return 0;

	pm_info->regs_values = devm_kzalloc(dev,
					    sizeof(u32) * regs_num,
					    GFP_KERNEL);
	if (WARN_ON(IS_ERR(pm_info->regs_values)))
		return PTR_ERR(pm_info->regs_values);

	pm_info->regs_offsets = kmemdup(regs_offsets,
					regs_num * sizeof(u32), GFP_KERNEL);
	if (WARN_ON(IS_ERR(pm_info->regs_offsets)))
		return PTR_ERR(pm_info->regs_offsets);

	pm_info->regs_num = regs_num;

	return 0;
}

static int imx_blk_ctrl_reset_set(struct reset_controller_dev *rcdev,
				  unsigned long id, bool assert)
{
	struct imx_blk_ctrl_drvdata *drvdata = container_of(rcdev,
			struct imx_blk_ctrl_drvdata, rcdev);
	unsigned int offset = drvdata->rst_hws[id].offset;
	unsigned int shift = drvdata->rst_hws[id].shift;
	unsigned int mask = drvdata->rst_hws[id].mask;
	void __iomem *reg_addr = drvdata->base + offset;
	unsigned long flags;
	u32 reg;

	if (!assert && !test_bit(1, &drvdata->rst_hws[id].asserted))
		return -ENODEV;

	if (assert && !test_and_set_bit(1, &drvdata->rst_hws[id].asserted))
		pm_runtime_get_sync(rcdev->dev);

	spin_lock_irqsave(drvdata->lock, flags);

	reg = readl(reg_addr);
	if (assert)
		writel(reg & ~(mask << shift), reg_addr);
	else
		writel(reg | (mask << shift), reg_addr);

	spin_unlock_irqrestore(drvdata->lock, flags);

	if (!assert && test_and_clear_bit(1, &drvdata->rst_hws[id].asserted))
		pm_runtime_put_sync(rcdev->dev);

	return 0;
}

static int imx_blk_ctrl_reset_reset(struct reset_controller_dev *rcdev,
					   unsigned long id)
{
	imx_blk_ctrl_reset_set(rcdev, id, true);
	return imx_blk_ctrl_reset_set(rcdev, id, false);
}

static int imx_blk_ctrl_reset_assert(struct reset_controller_dev *rcdev,
					   unsigned long id)
{
	return imx_blk_ctrl_reset_set(rcdev, id, true);
}

static int imx_blk_ctrl_reset_deassert(struct reset_controller_dev *rcdev,
					     unsigned long id)
{
	return imx_blk_ctrl_reset_set(rcdev, id, false);
}

static const struct reset_control_ops imx_blk_ctrl_reset_ops = {
	.reset		= imx_blk_ctrl_reset_reset,
	.assert		= imx_blk_ctrl_reset_assert,
	.deassert	= imx_blk_ctrl_reset_deassert,
};

static int imx_blk_ctrl_register_reset_controller(struct device *dev)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct reset_hw *hws;

	drvdata->rcdev.owner     = THIS_MODULE;
	drvdata->rcdev.nr_resets = IMX8MP_AUDIO_BLK_CTRL_RESET_NUM;
	drvdata->rcdev.ops       = &imx_blk_ctrl_reset_ops;
	drvdata->rcdev.of_node   = dev->of_node;
	drvdata->rcdev.dev	 = dev;

	drvdata->rst_hws = devm_kzalloc(dev, sizeof(*hws) * IMX8MP_AUDIO_BLK_CTRL_RESET_NUM,
					GFP_KERNEL);
	hws = drvdata->rst_hws;

	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_RESET].offset = 0x200;
	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_RESET].shift = 0x0;
	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_RESET].mask = 0x1;

	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_PHY_RESET].offset = 0x200;
	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_PHY_RESET].shift = 0x1;
	hws[IMX8MP_AUDIO_BLK_CTRL_EARC_PHY_RESET].mask = 0x1;

	return devm_reset_controller_register(dev, &drvdata->rcdev);
}

static int clk_imx8mp_audiomix_probe(struct platform_device *pdev)
{
	struct imx_blk_ctrl_drvdata *drvdata;
	struct clk_hw_onecell_data *priv;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	struct clk_hw *hw;
	int i, ret;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	priv = devm_kzalloc(dev,
			    struct_size(priv, hws, IMX8MP_CLK_AUDIOMIX_END),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->num = IMX8MP_CLK_AUDIOMIX_END;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	drvdata->base = base;
	drvdata->lock = &imx_ccm_lock;
	drvdata->regmap = syscon_regmap_lookup_by_compatible("fsl,imx8m-noc");
	dev_set_drvdata(dev, drvdata);

	ret = imx_blk_ctrl_init_runtime_pm_safekeeping(dev);
	if (ret)
		return ret;
	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	for (i = 0; i < ARRAY_SIZE(sels); i++) {
		if (sels[i].num_parents == 1) {
			hw = devm_clk_hw_register_gate_parent_data(dev,
				sels[i].name, &sels[i].parent, CLK_SET_RATE_PARENT,
				base + sels[i].reg, sels[i].shift, 0, NULL);
		} else {
			hw = devm_clk_hw_register_mux_parent_data_table(dev,
				sels[i].name, sels[i].parents,
				sels[i].num_parents, CLK_SET_RATE_PARENT,
				base + sels[i].reg,
				sels[i].shift, sels[i].width,
				0, NULL, NULL);
		}

		if (IS_ERR(hw))
			return PTR_ERR(hw);

		priv->hws[sels[i].clkid] = hw;
	}

	for (i = 0; i < ARRAY_SIZE(pdms); i++) {
		hw = imx_dev_clk_hw_gate_shared(dev, pdms[i].name,
						pdms[i].parents,
						base + pdms[i].reg,
						pdms[i].shift,
						pdms[i].shcount);
		if (IS_ERR(hw))
			return PTR_ERR(hw);

		priv->hws[pdms[i].clkid] = hw;
	}

	/* SAI PLL */
	hw = devm_clk_hw_register_mux_parent_data_table(dev,
		"sai_pll_ref_sel", clk_imx8mp_audiomix_pll_parents,
		ARRAY_SIZE(clk_imx8mp_audiomix_pll_parents),
		CLK_SET_RATE_NO_REPARENT, base + SAI_PLL_GNRL_CTL,
		0, 2, 0, NULL, NULL);
	priv->hws[IMX8MP_CLK_AUDIOMIX_SAI_PLL_REF_SEL] = hw;

	hw = imx_dev_clk_hw_pll14xx(dev, "sai_pll", "sai_pll_ref_sel",
				    base + 0x400, &imx_1443x_pll);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	priv->hws[IMX8MP_CLK_AUDIOMIX_SAI_PLL] = hw;

	hw = devm_clk_hw_register_mux_parent_data_table(dev,
		"sai_pll_bypass", clk_imx8mp_audiomix_pll_bypass_sels,
		ARRAY_SIZE(clk_imx8mp_audiomix_pll_bypass_sels),
		CLK_SET_RATE_NO_REPARENT | CLK_SET_RATE_PARENT,
		base + SAI_PLL_GNRL_CTL, 16, 1, 0, NULL, NULL);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	priv->hws[IMX8MP_CLK_AUDIOMIX_SAI_PLL_BYPASS] = hw;

	hw = devm_clk_hw_register_gate(dev, "sai_pll_out", "sai_pll_bypass",
				       CLK_SET_RATE_PARENT, base + SAI_PLL_GNRL_CTL, 13,
				       0, NULL);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	priv->hws[IMX8MP_CLK_AUDIOMIX_SAI_PLL_OUT] = hw;

	hw = devm_clk_hw_register_fixed_factor(dev, "sai_pll_out_div2",
					       "sai_pll_out", CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	ret = devm_of_clk_add_hw_provider(&pdev->dev, of_clk_hw_onecell_get, priv);
	if (ret)
		return ret;

	ret = imx_blk_ctrl_register_reset_controller(dev);
	if (ret)
		return ret;

	pm_runtime_put(dev);

	return 0;
}

static void imx_blk_ctrl_read_write(struct device *dev, bool write)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct pm_safekeep_info *pm_info = &drvdata->pm_info;
	void __iomem *base = drvdata->base;
	unsigned long flags;
	int i;

	if (!pm_info->regs_num)
		return;

	spin_lock_irqsave(drvdata->lock, flags);

	for (i = 0; i < pm_info->regs_num; i++) {
		u32 offset = pm_info->regs_offsets[i];

		if (write)
			writel(pm_info->regs_values[i], base + offset);
		else
			pm_info->regs_values[i] = readl(base + offset);
	}

	spin_unlock_irqrestore(drvdata->lock, flags);

}

static int imx_blk_ctrl_runtime_suspend(struct device *dev)
{
	imx_blk_ctrl_read_write(dev, false);

	return 0;
}

static int imx_blk_ctrl_runtime_resume(struct device *dev)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct regmap *regmap = drvdata->regmap;

	imx_blk_ctrl_read_write(dev, true);

	if (!IS_ERR(regmap))
		regmap_write(regmap, IMX8MP_DSP_NOC_OFFSET, IMX8MP_DSP_NOC_PRIORITY);

	return 0;
}

static const struct dev_pm_ops imx_audiomix_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_blk_ctrl_runtime_suspend,
			   imx_blk_ctrl_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
			   pm_runtime_force_resume)
};

#define	IMX_AUDIO_BLK_CTRL_CLKEN0		0x0
#define	IMX_AUDIO_BLK_CTRL_CLKEN1		0x4
#define	IMX_AUDIO_BLK_CTRL_EARC			0x200
#define	IMX_AUDIO_BLK_CTRL_SAI1_MCLK_SEL	0x300
#define	IMX_AUDIO_BLK_CTRL_SAI2_MCLK_SEL	0x304
#define	IMX_AUDIO_BLK_CTRL_SAI3_MCLK_SEL	0x308
#define	IMX_AUDIO_BLK_CTRL_SAI5_MCLK_SEL	0x30C
#define	IMX_AUDIO_BLK_CTRL_SAI6_MCLK_SEL	0x310
#define	IMX_AUDIO_BLK_CTRL_SAI7_MCLK_SEL	0x314
#define	IMX_AUDIO_BLK_CTRL_PDM_CLK		0x318
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_GNRL_CTL	0x400
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL0	0x404
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL1	0x408
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_SSCG_CTL	0x40C
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_MNIT_CTL	0x410
#define	IMX_AUDIO_BLK_CTRL_IPG_LP_CTRL		0x504

#define IMX_MEDIA_BLK_CTRL_SFT_RSTN		0x0
#define IMX_MEDIA_BLK_CTRL_CLK_EN		0x4

const struct imx_blk_ctrl_dev_data imx8mp_audiomix_dev_data = {
	.pm_runtime_saved_regs_num = 16,
	.pm_runtime_saved_regs = {
		IMX_AUDIO_BLK_CTRL_CLKEN0,
		IMX_AUDIO_BLK_CTRL_CLKEN1,
		IMX_AUDIO_BLK_CTRL_EARC,
		IMX_AUDIO_BLK_CTRL_SAI1_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI2_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI3_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI5_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI6_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI7_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_PDM_CLK,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_GNRL_CTL,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL0,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL1,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_SSCG_CTL,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_MNIT_CTL,
		IMX_AUDIO_BLK_CTRL_IPG_LP_CTRL
	},
};

static const struct of_device_id clk_imx8mp_audiomix_of_match[] = {
	{ .compatible = "fsl,imx8mp-audio-blk-ctrl", .data = &imx8mp_audiomix_dev_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, clk_imx8mp_audiomix_of_match);

static struct platform_driver clk_imx8mp_audiomix_driver = {
	.probe	= clk_imx8mp_audiomix_probe,
	.driver = {
		.name = "imx8mp-audio-blk-ctrl",
		.of_match_table = clk_imx8mp_audiomix_of_match,
		.pm = &imx_audiomix_pm_ops,
	},
};

module_platform_driver(clk_imx8mp_audiomix_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("Freescale i.MX8MP Audio Block Controller driver");
MODULE_LICENSE("GPL");
