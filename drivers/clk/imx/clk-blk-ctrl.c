// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 NXP.
 */

#include <linux/clk.h>
#include <linux/reset-controller.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "clk.h"
#include "clk-blk-ctrl.h"

struct reset_hw {
	u32 offset;
	u32 shift;
	u32 mask;
	unsigned long asserted;
};

struct pm_safekeep_info {
	uint32_t *regs_values;
	uint32_t *regs_offsets;
	uint32_t regs_num;
};

struct imx_blk_ctrl_drvdata {
	void __iomem *base;
	struct reset_controller_dev rcdev;
	struct reset_hw *rst_hws;
	struct pm_safekeep_info pm_info;

	spinlock_t *lock;
};

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
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct reset_hw *hws;
	int max = dev_data->resets_max;
	int i;

	drvdata->lock = &imx_ccm_lock;

	drvdata->rcdev.owner     = THIS_MODULE;
	drvdata->rcdev.nr_resets = max;
	drvdata->rcdev.ops       = &imx_blk_ctrl_reset_ops;
	drvdata->rcdev.of_node   = dev->of_node;
	drvdata->rcdev.dev	 = dev;

	drvdata->rst_hws = devm_kzalloc(dev, sizeof(struct reset_hw) * max,
					GFP_KERNEL);
	hws = drvdata->rst_hws;

	for (i = 0; i < dev_data->hws_num; i++) {
		struct imx_blk_ctrl_hw *hw = &dev_data->hws[i];

		if (hw->type != BLK_CTRL_RESET)
			continue;

		hws[hw->id].offset = hw->offset;
		hws[hw->id].shift = hw->shift;
		hws[hw->id].mask = hw->mask;
	}

	return devm_reset_controller_register(dev, &drvdata->rcdev);
}
static struct clk_hw *imx_blk_ctrl_register_one_clock(struct device *dev,
						struct imx_blk_ctrl_hw *hw)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	void __iomem *base = drvdata->base;
	struct clk_hw *clk_hw;

	switch (hw->type) {
	case BLK_CTRL_CLK_MUX:
		clk_hw = imx_dev_clk_hw_mux_flags(dev, hw->name,
						  base + hw->offset,
						  hw->shift, hw->width,
						  hw->parents,
						  hw->parents_count,
						  hw->flags);
		break;
	case BLK_CTRL_CLK_GATE:
		clk_hw = imx_dev_clk_hw_gate(dev, hw->name, hw->parents,
					     base + hw->offset, hw->shift);
		break;
	case BLK_CTRL_CLK_SHARED_GATE:
		clk_hw = imx_dev_clk_hw_gate_shared(dev, hw->name,
						    hw->parents,
						    base + hw->offset,
						    hw->shift,
						    hw->shared_count);
		break;
	case BLK_CTRL_CLK_PLL14XX:
		clk_hw = imx_dev_clk_hw_pll14xx(dev, hw->name, hw->parents,
						base + hw->offset, hw->pll_tbl);
		break;
	default:
		clk_hw = NULL;
	};

	return clk_hw;
}

static int imx_blk_ctrl_register_clock_controller(struct device *dev)
{
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct clk_hw_onecell_data *clk_hw_data;
	struct clk_hw **hws;
	int i;

	clk_hw_data = devm_kzalloc(dev, struct_size(clk_hw_data, hws,
				dev_data->hws_num), GFP_KERNEL);
	if (WARN_ON(!clk_hw_data))
		return -ENOMEM;

	clk_hw_data->num = dev_data->clocks_max;
	hws = clk_hw_data->hws;

	for (i = 0; i < dev_data->hws_num; i++) {
		struct imx_blk_ctrl_hw *hw = &dev_data->hws[i];
		struct clk_hw *tmp = imx_blk_ctrl_register_one_clock(dev, hw);

		if (!tmp)
			continue;
		hws[hw->id] = tmp;
	}

	imx_check_clk_hws(hws, dev_data->clocks_max);

	return of_clk_add_hw_provider(dev->of_node, of_clk_hw_onecell_get,
					clk_hw_data);
}

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

static int imx_blk_ctrl_probe(struct platform_device *pdev)
{
	struct imx_blk_ctrl_drvdata *drvdata;
	struct device *dev = &pdev->dev;
	int ret;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (WARN_ON(!drvdata))
		return -ENOMEM;

	drvdata->base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(drvdata->base)))
		return PTR_ERR(drvdata->base);

	dev_set_drvdata(dev, drvdata);

	ret = imx_blk_ctrl_init_runtime_pm_safekeeping(dev);
	if (ret)
		return ret;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = imx_blk_ctrl_register_clock_controller(dev);
	if (ret) {
		pm_runtime_put(dev);
		return ret;
	}

	ret = imx_blk_ctrl_register_reset_controller(dev);

	pm_runtime_put(dev);

	return ret;
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
	imx_blk_ctrl_read_write(dev, true);

	return 0;
}

static const struct dev_pm_ops imx_blk_ctrl_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_blk_ctrl_runtime_suspend,
			   imx_blk_ctrl_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
			   pm_runtime_force_resume)
};

static const struct of_device_id imx_blk_ctrl_of_match[] = {
	{
		.compatible = "fsl,imx8mp-audio-blk-ctrl",
		.data = &imx8mp_audio_blk_ctrl_dev_data
	},
	{
		.compatible = "fsl,imx8mp-media-blk-ctrl",
		.data = &imx8mp_media_blk_ctrl_dev_data
	},
	{
		.compatible = "fsl,imx8mp-hdmi-blk-ctrl",
		.data = &imx8mp_hdmi_blk_ctrl_dev_data
	},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_blk_ctrl_of_match);

static struct platform_driver imx_blk_ctrl_driver = {
	.probe = imx_blk_ctrl_probe,
	.driver = {
		.name = "imx-blk-ctrl",
		.of_match_table = of_match_ptr(imx_blk_ctrl_of_match),
		.pm = &imx_blk_ctrl_pm_ops,
	},
};
module_platform_driver(imx_blk_ctrl_driver);
