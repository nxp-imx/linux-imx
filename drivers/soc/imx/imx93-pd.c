// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 NXP.
 */

#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <dt-bindings/power/imx93-power.h>

#define IMX93_SRC_MLMIX_OFF		0x1800
#define IMX93_SRC_MEDIAMIX_OFF		0x2400

#define MIX_SLICE_SW_CTRL_OFF		0x20
#define SLICE_SW_CTRL_PSW_CTRL_OFF_MASK	BIT(4)
#define SLICE_SW_CTRL_PDN_SOFT_MASK	BIT(31)

#define MIX_FUNC_STAT_OFF		0xB4

#define FUNC_STAT_PSW_STAT_MASK		BIT(0)
#define FUNC_STAT_RST_STAT_MASK		BIT(2)
#define FUNC_STAT_ISO_STAT_MASK		BIT(4)

struct imx93_slice_info {
	char *name;
	u32 mix_off;
};

struct imx93_plat_data {
	u32 num_slice;
	struct imx93_slice_info *slices;
};

struct imx93_power_domain {
	struct generic_pm_domain genpd;
	struct device *dev;
	void * __iomem base;
	const struct imx93_slice_info *slice_info;
	struct clk_bulk_data *clks;
	int num_clks;
};

#define to_imx93_pd(_genpd) container_of(_genpd, struct imx93_power_domain, genpd)

struct imx93_slice_info imx93_slice_infos[] = {
	[IMX93_POWER_DOMAIN_MEDIAMIX] = {
		.name      = "mediamix",
		.mix_off = IMX93_SRC_MEDIAMIX_OFF,
	},
	[IMX93_POWER_DOMAIN_MLMIX] = {
		.name      = "mlmix",
		.mix_off = IMX93_SRC_MLMIX_OFF,
	}
};

struct imx93_plat_data imx93_plat_data = {
	.num_slice = ARRAY_SIZE(imx93_slice_infos),
	.slices = imx93_slice_infos,
};

static int imx93_pd_on(struct generic_pm_domain *genpd)
{
	struct imx93_power_domain *domain = to_imx93_pd(genpd);
	const struct imx93_slice_info *slice_info =  domain->slice_info;
	void * __iomem addr = domain->base + slice_info->mix_off;
	u32 val;
	int ret;

	ret = clk_bulk_prepare_enable(domain->num_clks, domain->clks);
	if (ret) {
		dev_err(domain->dev, "failed to enable clocks for domain: %s\n", genpd->name);
		return ret;
	}

	val = readl(addr + MIX_SLICE_SW_CTRL_OFF);
	val &= ~SLICE_SW_CTRL_PDN_SOFT_MASK;
	writel(val, addr + MIX_SLICE_SW_CTRL_OFF);

	ret = readl_poll_timeout(addr + MIX_FUNC_STAT_OFF, val,
				 !(val & FUNC_STAT_ISO_STAT_MASK), 1, 10000);
	if (ret) {
		dev_err(domain->dev, "pd_on timeout: name: %s, stat: %x\n", genpd->name, val);
		return ret;
	}

	return 0;
}

static int imx93_pd_off(struct generic_pm_domain *genpd)
{
	struct imx93_power_domain *domain = to_imx93_pd(genpd);
	const struct imx93_slice_info *slice_info =  domain->slice_info;
	void * __iomem addr = domain->base + slice_info->mix_off;
	int ret;
	u32 val;

	/* Power off MIX */
	val = readl(addr + MIX_SLICE_SW_CTRL_OFF);
	val |= SLICE_SW_CTRL_PDN_SOFT_MASK;
	writel(val, addr + MIX_SLICE_SW_CTRL_OFF);

	ret = readl_poll_timeout(addr + MIX_FUNC_STAT_OFF, val,
				 val & FUNC_STAT_PSW_STAT_MASK, 1, 1000);
	if (ret) {
		dev_err(domain->dev, "pd_off timeout: name: %s, stat: %x\n", genpd->name, val);
		return ret;
	}

	clk_bulk_disable_unprepare(domain->num_clks, domain->clks);

	return 0;
};

static const struct of_device_id imx93_power_domain_ids[] = {
	{ .compatible = "fsl,imx93-src", .data = &imx93_plat_data, },
	{},
};

static int imx93_pd_remove(struct platform_device *pdev)
{
	struct imx93_power_domain *pd = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	const struct imx93_plat_data *data = of_device_get_match_data(dev);
	u32 num_domains = data->num_slice;
	struct device_node *slice_np, *np;
	int ret;

	slice_np = of_get_child_by_name(pdev->dev.of_node, "slice");

	for_each_child_of_node(slice_np, np) {
		struct imx93_power_domain *domain;
		u32 index;

		if (!of_device_is_available(np))
			continue;

		ret = of_property_read_u32(np, "reg", &index);
		if (ret) {
			dev_err(dev, "Failed to read 'reg' property\n");
			of_node_put(np);
			return ret;
		}

		if (index >= num_domains) {
			dev_warn(dev, "Domain index %d is out of bounds\n", index);
			continue;
		}

		domain = &pd[index];

		of_genpd_del_provider(np);

		pm_genpd_remove(&domain->genpd);
		clk_bulk_put_all(domain->num_clks, domain->clks);
	};

	return 0;
}

static int imx93_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct imx93_plat_data *data = of_device_get_match_data(dev);
	const struct imx93_slice_info *slice_info = data->slices;
	struct imx93_power_domain *pd;
	u32 num_domains = data->num_slice;
	struct device_node *slice_np, *np;
	void __iomem *base;
	bool is_off;
	int ret;

	slice_np = of_get_child_by_name(dev->of_node, "slice");
	if (!slice_np) {
		dev_err(dev, "No slices specified in DT\n");
		return -EINVAL;
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pd = devm_kcalloc(dev, num_domains, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	platform_set_drvdata(pdev, pd);

	for_each_child_of_node(slice_np, np) {
		struct imx93_power_domain *domain;
		u32 index;

		if (!of_device_is_available(np))
			continue;

		ret = of_property_read_u32(np, "reg", &index);
		if (ret) {
			dev_err(dev, "Failed to read 'reg' property\n");
			of_node_put(np);
			return ret;
		}

		if (index >= num_domains) {
			dev_warn(dev, "Domain index %d is out of bounds\n", index);
			continue;
		}

		domain = &pd[index];
		domain->dev = dev;

		domain->num_clks = of_clk_bulk_get_all(np, &domain->clks);
		if (domain->num_clks < 0) {
			return dev_err_probe(domain->dev, domain->num_clks,
					     "Failed to get %s's clocks\n",
					     slice_info[index].name);
		}

		domain->genpd.name = slice_info[index].name;
		domain->genpd.power_off = imx93_pd_off;
		domain->genpd.power_on = imx93_pd_on;
		domain->slice_info = &slice_info[index];
		domain->base = base;

		is_off = readl(domain->base + slice_info->mix_off + MIX_FUNC_STAT_OFF) &
			FUNC_STAT_ISO_STAT_MASK;
		/* Just to sync the status of hardware */
		if (!is_off) {
			ret = clk_bulk_prepare_enable(domain->num_clks, domain->clks);
			if (ret) {
				dev_err(domain->dev, "failed to enable clocks for domain: %s\n",
					domain->genpd.name);
				clk_bulk_put_all(domain->num_clks, domain->clks);
				return 0;
			}
		}

		dev_info(dev, "%s: state: %x\n", domain->genpd.name,
			 readl(domain->base + MIX_FUNC_STAT_OFF));
		ret = pm_genpd_init(&domain->genpd, NULL, is_off);
		if (ret) {
			dev_err(dev, "failed to init genpd\n");
			clk_bulk_put_all(domain->num_clks, domain->clks);
			return ret;
		}

		ret = of_genpd_add_provider_simple(np, &domain->genpd);
		if (ret) {
			clk_bulk_put_all(domain->num_clks, domain->clks);
			return ret;
		}
	}

	return 0;
}

static const struct of_device_id imx93_dt_ids[] = {
	{ .compatible = "fsl,imx93-src", .data = &imx93_plat_data, },
	{ }
};

static struct platform_driver imx93_power_domain_driver = {
	.driver = {
		.name	= "imx93_power_domain",
		.owner	= THIS_MODULE,
		.of_match_table = imx93_dt_ids,
	},
	.probe = imx93_pd_probe,
	.remove = imx93_pd_remove,
};
module_platform_driver(imx93_power_domain_driver);

MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX93 power domain driver");
MODULE_LICENSE("GPL v2");
