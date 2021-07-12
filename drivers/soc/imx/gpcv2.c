// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017 Impinj, Inc
 * Author: Andrey Smirnov <andrew.smirnov@gmail.com>
 *
 * Based on the code of analogus driver:
 *
 * Copyright 2015-2017 Pengutronix, Lucas Stach <kernel@pengutronix.de>
 */

#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <dt-bindings/power/imx7-power.h>
#include <dt-bindings/power/imx8mq-power.h>

#define GPC_LPCR_A_CORE_BSC			0x000

#define GPC_PGC_CPU_MAPPING		0x0ec

#define IMX7_USB_HSIC_PHY_A_CORE_DOMAIN		BIT(6)
#define IMX7_USB_OTG2_PHY_A_CORE_DOMAIN		BIT(5)
#define IMX7_USB_OTG1_PHY_A_CORE_DOMAIN		BIT(4)
#define IMX7_PCIE_PHY_A_CORE_DOMAIN		BIT(3)
#define IMX7_MIPI_PHY_A_CORE_DOMAIN		BIT(2)

#define IMX8M_PCIE2_A53_DOMAIN			BIT(15)
#define IMX8M_MIPI_CSI2_A53_DOMAIN		BIT(14)
#define IMX8M_MIPI_CSI1_A53_DOMAIN		BIT(13)
#define IMX8M_DISP_A53_DOMAIN			BIT(12)
#define IMX8M_HDMI_A53_DOMAIN			BIT(11)
#define IMX8M_VPU_A53_DOMAIN			BIT(10)
#define IMX8M_GPU_A53_DOMAIN			BIT(9)
#define IMX8M_DDR2_A53_DOMAIN			BIT(8)
#define IMX8M_DDR1_A53_DOMAIN			BIT(7)
#define IMX8M_OTG2_A53_DOMAIN			BIT(5)
#define IMX8M_OTG1_A53_DOMAIN			BIT(4)
#define IMX8M_PCIE1_A53_DOMAIN			BIT(3)
#define IMX8M_MIPI_A53_DOMAIN			BIT(2)

#define GPC_PU_PGC_SW_PUP_REQ		0x0f8
#define GPC_PU_PGC_SW_PDN_REQ		0x104

#define IMX7_USB_HSIC_PHY_SW_Pxx_REQ		BIT(4)
#define IMX7_USB_OTG2_PHY_SW_Pxx_REQ		BIT(3)
#define IMX7_USB_OTG1_PHY_SW_Pxx_REQ		BIT(2)
#define IMX7_PCIE_PHY_SW_Pxx_REQ		BIT(1)
#define IMX7_MIPI_PHY_SW_Pxx_REQ		BIT(0)

#define IMX8M_PCIE2_SW_Pxx_REQ			BIT(13)
#define IMX8M_MIPI_CSI2_SW_Pxx_REQ		BIT(12)
#define IMX8M_MIPI_CSI1_SW_Pxx_REQ		BIT(11)
#define IMX8M_DISP_SW_Pxx_REQ			BIT(10)
#define IMX8M_HDMI_SW_Pxx_REQ			BIT(9)
#define IMX8M_VPU_SW_Pxx_REQ			BIT(8)
#define IMX8M_GPU_SW_Pxx_REQ			BIT(7)
#define IMX8M_DDR2_SW_Pxx_REQ			BIT(6)
#define IMX8M_DDR1_SW_Pxx_REQ			BIT(5)
#define IMX8M_OTG2_SW_Pxx_REQ			BIT(3)
#define IMX8M_OTG1_SW_Pxx_REQ			BIT(2)
#define IMX8M_PCIE1_SW_Pxx_REQ			BIT(1)
#define IMX8M_MIPI_SW_Pxx_REQ			BIT(0)

#define GPC_M4_PU_PDN_FLG		0x1bc

#define GPC_PU_PWRHSK			0x1fc

#define IMX8M_GPU_HSK_PWRDNREQN			BIT(6)
#define IMX8M_VPU_HSK_PWRDNREQN			BIT(5)
#define IMX8M_DISP_HSK_PWRDNREQN		BIT(4)

/*
 * The PGC offset values in Reference Manual
 * (Rev. 1, 01/2018 and the older ones) GPC chapter's
 * GPC_PGC memory map are incorrect, below offset
 * values are from design RTL.
 */
#define IMX7_PGC_MIPI			16
#define IMX7_PGC_PCIE			17
#define IMX7_PGC_USB_HSIC		20

#define IMX8M_PGC_MIPI			16
#define IMX8M_PGC_PCIE1			17
#define IMX8M_PGC_OTG1			18
#define IMX8M_PGC_OTG2			19
#define IMX8M_PGC_DDR1			21
#define IMX8M_PGC_GPU			23
#define IMX8M_PGC_VPU			24
#define IMX8M_PGC_DISP			26
#define IMX8M_PGC_MIPI_CSI1		27
#define IMX8M_PGC_MIPI_CSI2		28
#define IMX8M_PGC_PCIE2			29

#define GPC_PGC_CTRL(n)			(0x800 + (n) * 0x40)
#define GPC_PGC_SR(n)			(GPC_PGC_CTRL(n) + 0xc)

#define GPC_PGC_CTRL_PCR		BIT(0)

#define GPC_CLK_MAX		6

struct imx_pgc_domain {
	struct generic_pm_domain genpd;
	struct regmap *regmap;
	struct regulator *regulator;
	struct clk *clk[GPC_CLK_MAX];
	int num_clks;

	unsigned int pgc;

	const struct {
		u32 pxx;
		u32 map;
		u32 hsk;
	} bits;

	const int voltage;
	struct device *dev;
};

struct imx_pgc_domain_data {
	const struct imx_pgc_domain *domains;
	size_t domains_num;
	const struct regmap_access_table *reg_access_table;
};

static int imx_pgc_get_clocks(struct imx_pgc_domain *domain)
{
	int i, ret;

	for (i = 0; ; i++) {
		struct clk *clk = of_clk_get(domain->dev->of_node, i);
		if (IS_ERR(clk))
			break;
		if (i >= GPC_CLK_MAX) {
			dev_err(domain->dev, "more than %d clocks\n",
				GPC_CLK_MAX);
			ret = -EINVAL;
			goto clk_err;
		}
		domain->clk[i] = clk;
	}
	domain->num_clks = i;

	return 0;

clk_err:
	while (i--)
		clk_put(domain->clk[i]);

	return ret;
}

static void imx_pgc_put_clocks(struct imx_pgc_domain *domain)
{
	int i;

	for (i = domain->num_clks - 1; i >= 0; i--)
		clk_put(domain->clk[i]);
}

static int imx_pgc_domain_probe(struct platform_device *pdev)
{
	struct imx_pgc_domain *domain = pdev->dev.platform_data;
	int ret;

	domain->dev = &pdev->dev;

	domain->regulator = devm_regulator_get_optional(domain->dev, "power");
	if (IS_ERR(domain->regulator)) {
		if (PTR_ERR(domain->regulator) != -ENODEV)
			return dev_err_probe(domain->dev, PTR_ERR(domain->regulator),
					     "Failed to get domain's regulator\n");
	} else if (domain->voltage) {
		regulator_set_voltage(domain->regulator,
				      domain->voltage, domain->voltage);
	}

	ret = imx_pgc_get_clocks(domain);
	if (ret)
		return dev_err_probe(domain->dev, ret, "Failed to get domain's clocks\n");

	ret = pm_genpd_init(&domain->genpd, NULL, true);
	if (ret) {
		dev_err(domain->dev, "Failed to init power domain\n");
		imx_pgc_put_clocks(domain);
		return ret;
	}

	ret = of_genpd_add_provider_simple(domain->dev->of_node,
					   &domain->genpd);
	if (ret) {
		dev_err(domain->dev, "Failed to add genpd provider\n");
		pm_genpd_remove(&domain->genpd);
		imx_pgc_put_clocks(domain);
	}

	return ret;
}

static int imx_pgc_domain_remove(struct platform_device *pdev)
{
	struct imx_pgc_domain *domain = pdev->dev.platform_data;

	of_genpd_del_provider(domain->dev->of_node);
	pm_genpd_remove(&domain->genpd);
	imx_pgc_put_clocks(domain);

	return 0;
}

static const struct platform_device_id imx_pgc_domain_id[] = {
	{ "imx-pgc-domain", },
	{ },
};

static struct platform_driver imx_pgc_domain_driver = {
	.driver = {
		.name = "imx-pgc",
	},
	.probe    = imx_pgc_domain_probe,
	.remove   = imx_pgc_domain_remove,
	.id_table = imx_pgc_domain_id,
};

#ifdef MODULE
module_platform_driver(imx_pgc_domain_driver);
#else
builtin_platform_driver(imx_pgc_domain_driver)
#endif

MODULE_LICENSE("GPL");
