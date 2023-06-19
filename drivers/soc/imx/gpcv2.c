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
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <soc/imx/gpcv2.h>
#include <dt-bindings/power/imx7-power.h>
#include <dt-bindings/power/imx8mq-power.h>
#include <dt-bindings/power/imx8mm-power.h>
#include <dt-bindings/power/imx8mn-power.h>
#include <dt-bindings/power/imx8mp-power.h>

#define GPC_LPCR_A_CORE_BSC			0x000

#define GPC_PGC_CPU_MAPPING		0x0ec
#define IMX8MP_GPC_PGC_CPU_MAPPING	0x1cc

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

#define IMX8MM_VPUH1_A53_DOMAIN			BIT(15)
#define IMX8MM_VPUG2_A53_DOMAIN			BIT(14)
#define IMX8MM_VPUG1_A53_DOMAIN			BIT(13)
#define IMX8MM_DISPMIX_A53_DOMAIN		BIT(12)
#define IMX8MM_VPUMIX_A53_DOMAIN		BIT(10)
#define IMX8MM_GPUMIX_A53_DOMAIN		BIT(9)
#define IMX8MM_GPU_A53_DOMAIN			(BIT(8) | BIT(11))
#define IMX8MM_DDR1_A53_DOMAIN			BIT(7)
#define IMX8MM_OTG2_A53_DOMAIN			BIT(5)
#define IMX8MM_OTG1_A53_DOMAIN			BIT(4)
#define IMX8MM_PCIE_A53_DOMAIN			BIT(3)
#define IMX8MM_MIPI_A53_DOMAIN			BIT(2)

#define IMX8MN_DISPMIX_A53_DOMAIN		BIT(12)
#define IMX8MN_GPUMIX_A53_DOMAIN		BIT(9)
#define IMX8MN_DDR1_A53_DOMAIN		BIT(7)
#define IMX8MN_OTG1_A53_DOMAIN		BIT(4)
#define IMX8MN_MIPI_A53_DOMAIN		BIT(2)

#define IMX8MP_MEDIA_ISPDWP_A53_DOMAIN	BIT(20)
#define IMX8MP_HSIOMIX_A53_DOMAIN		BIT(19)
#define IMX8MP_MIPI_PHY2_A53_DOMAIN		BIT(18)
#define IMX8MP_HDMI_PHY_A53_DOMAIN		BIT(17)
#define IMX8MP_HDMIMIX_A53_DOMAIN		BIT(16)
#define IMX8MP_VPU_VC8000E_A53_DOMAIN		BIT(15)
#define IMX8MP_VPU_G2_A53_DOMAIN		BIT(14)
#define IMX8MP_VPU_G1_A53_DOMAIN		BIT(13)
#define IMX8MP_MEDIAMIX_A53_DOMAIN		BIT(12)
#define IMX8MP_GPU3D_A53_DOMAIN			BIT(11)
#define IMX8MP_VPUMIX_A53_DOMAIN		BIT(10)
#define IMX8MP_GPUMIX_A53_DOMAIN		BIT(9)
#define IMX8MP_GPU2D_A53_DOMAIN			BIT(8)
#define IMX8MP_AUDIOMIX_A53_DOMAIN		BIT(7)
#define IMX8MP_MLMIX_A53_DOMAIN			BIT(6)
#define IMX8MP_USB2_PHY_A53_DOMAIN		BIT(5)
#define IMX8MP_USB1_PHY_A53_DOMAIN		BIT(4)
#define IMX8MP_PCIE_PHY_A53_DOMAIN		BIT(3)
#define IMX8MP_MIPI_PHY1_A53_DOMAIN		BIT(2)

#define IMX8MP_GPC_PU_PGC_SW_PUP_REQ	0x0d8
#define IMX8MP_GPC_PU_PGC_SW_PDN_REQ	0x0e4

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

#define IMX8MM_VPUH1_SW_Pxx_REQ			BIT(13)
#define IMX8MM_VPUG2_SW_Pxx_REQ			BIT(12)
#define IMX8MM_VPUG1_SW_Pxx_REQ			BIT(11)
#define IMX8MM_DISPMIX_SW_Pxx_REQ		BIT(10)
#define IMX8MM_VPUMIX_SW_Pxx_REQ		BIT(8)
#define IMX8MM_GPUMIX_SW_Pxx_REQ		BIT(7)
#define IMX8MM_GPU_SW_Pxx_REQ			(BIT(6) | BIT(9))
#define IMX8MM_DDR1_SW_Pxx_REQ			BIT(5)
#define IMX8MM_OTG2_SW_Pxx_REQ			BIT(3)
#define IMX8MM_OTG1_SW_Pxx_REQ			BIT(2)
#define IMX8MM_PCIE_SW_Pxx_REQ			BIT(1)
#define IMX8MM_MIPI_SW_Pxx_REQ			BIT(0)

#define IMX8MN_DISPMIX_SW_Pxx_REQ		BIT(10)
#define IMX8MN_GPUMIX_SW_Pxx_REQ		BIT(7)
#define IMX8MN_DDR1_SW_Pxx_REQ		BIT(5)
#define IMX8MN_OTG1_SW_Pxx_REQ		BIT(2)
#define IMX8MN_MIPI_SW_Pxx_REQ		BIT(0)

#define IMX8MP_DDRMIX_Pxx_REQ			BIT(19)
#define IMX8MP_MEDIA_ISP_DWP_Pxx_REQ		BIT(18)
#define IMX8MP_HSIOMIX_Pxx_REQ			BIT(17)
#define IMX8MP_MIPI_PHY2_Pxx_REQ		BIT(16)
#define IMX8MP_HDMI_PHY_Pxx_REQ			BIT(15)
#define IMX8MP_HDMIMIX_Pxx_REQ			BIT(14)
#define IMX8MP_VPU_VC8K_Pxx_REQ			BIT(13)
#define IMX8MP_VPU_G2_Pxx_REQ			BIT(12)
#define IMX8MP_VPU_G1_Pxx_REQ			BIT(11)
#define IMX8MP_MEDIMIX_Pxx_REQ			BIT(10)
#define IMX8MP_GPU_3D_Pxx_REQ			BIT(9)
#define IMX8MP_VPU_MIX_SHARE_LOGIC_Pxx_REQ	BIT(8)
#define IMX8MP_GPU_SHARE_LOGIC_Pxx_REQ		BIT(7)
#define IMX8MP_GPU_2D_Pxx_REQ			BIT(6)
#define IMX8MP_AUDIOMIX_Pxx_REQ			BIT(5)
#define IMX8MP_MLMIX_Pxx_REQ			BIT(4)
#define IMX8MP_USB2_PHY_Pxx_REQ			BIT(3)
#define IMX8MP_USB1_PHY_Pxx_REQ			BIT(2)
#define IMX8MP_PCIE_PHY_SW_Pxx_REQ		BIT(1)
#define IMX8MP_MIPI_PHY1_SW_Pxx_REQ		BIT(0)

#define GPC_M4_PU_PDN_FLG		0x1bc

#define IMX8MP_GPC_PU_PWRHSK		0x190
#define GPC_PU_PWRHSK			0x1fc

#define IMX8M_GPU_HSK_PWRDNACKN			BIT(26)
#define IMX8M_VPU_HSK_PWRDNACKN			BIT(25)
#define IMX8M_DISP_HSK_PWRDNACKN		BIT(24)
#define IMX8M_GPU_HSK_PWRDNREQN			BIT(6)
#define IMX8M_VPU_HSK_PWRDNREQN			BIT(5)
#define IMX8M_DISP_HSK_PWRDNREQN		BIT(4)

#define IMX8MM_GPUMIX_HSK_PWRDNACKN		BIT(29)
#define IMX8MM_GPU_HSK_PWRDNACKN		(BIT(27) | BIT(28))
#define IMX8MM_VPUMIX_HSK_PWRDNACKN		BIT(26)
#define IMX8MM_DISPMIX_HSK_PWRDNACKN		BIT(25)
#define IMX8MM_HSIO_HSK_PWRDNACKN		(BIT(23) | BIT(24))
#define IMX8MM_GPUMIX_HSK_PWRDNREQN		BIT(11)
#define IMX8MM_GPU_HSK_PWRDNREQN		(BIT(9) | BIT(10))
#define IMX8MM_VPUMIX_HSK_PWRDNREQN		BIT(8)
#define IMX8MM_DISPMIX_HSK_PWRDNREQN		BIT(7)
#define IMX8MM_HSIO_HSK_PWRDNREQN		(BIT(5) | BIT(6))

#define IMX8MN_GPUMIX_HSK_PWRDNACKN		(BIT(29) | BIT(27))
#define IMX8MN_DISPMIX_HSK_PWRDNACKN		BIT(25)
#define IMX8MN_HSIO_HSK_PWRDNACKN		BIT(23)
#define IMX8MN_GPUMIX_HSK_PWRDNREQN		(BIT(11) | BIT(9))
#define IMX8MN_DISPMIX_HSK_PWRDNREQN		BIT(7)
#define IMX8MN_HSIO_HSK_PWRDNREQN		BIT(5)

#define IMX8MP_MEDIAMIX_PWRDNACKN		BIT(30)
#define IMX8MP_HDMIMIX_PWRDNACKN		BIT(29)
#define IMX8MP_HSIOMIX_PWRDNACKN		BIT(28)
#define IMX8MP_VPUMIX_PWRDNACKN			BIT(26)
#define IMX8MP_GPUMIX_PWRDNACKN			BIT(25)
#define IMX8MP_MLMIX_PWRDNACKN			(BIT(23) | BIT(24))
#define IMX8MP_AUDIOMIX_PWRDNACKN		(BIT(20) | BIT(31))
#define IMX8MP_MEDIAMIX_PWRDNREQN		BIT(14)
#define IMX8MP_HDMIMIX_PWRDNREQN		BIT(13)
#define IMX8MP_HSIOMIX_PWRDNREQN		BIT(12)
#define IMX8MP_VPUMIX_PWRDNREQN			BIT(10)
#define IMX8MP_GPUMIX_PWRDNREQN			BIT(9)
#define IMX8MP_MLMIX_PWRDNREQN			(BIT(7) | BIT(8))
#define IMX8MP_AUDIOMIX_PWRDNREQN		(BIT(4) | BIT(15))

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

#define IMX8MM_PGC_MIPI			16
#define IMX8MM_PGC_PCIE			17
#define IMX8MM_PGC_OTG1			18
#define IMX8MM_PGC_OTG2			19
#define IMX8MM_PGC_DDR1			21
#define IMX8MM_PGC_GPU2D		22
#define IMX8MM_PGC_GPUMIX		23
#define IMX8MM_PGC_VPUMIX		24
#define IMX8MM_PGC_GPU3D		25
#define IMX8MM_PGC_DISPMIX		26
#define IMX8MM_PGC_VPUG1		27
#define IMX8MM_PGC_VPUG2		28
#define IMX8MM_PGC_VPUH1		29

#define IMX8MN_PGC_MIPI		16
#define IMX8MN_PGC_OTG1		18
#define IMX8MN_PGC_DDR1		21
#define IMX8MN_PGC_GPUMIX		23
#define IMX8MN_PGC_DISPMIX		26

#define IMX8MP_PGC_NOC			9
#define IMX8MP_PGC_MIPI1		12
#define IMX8MP_PGC_PCIE			13
#define IMX8MP_PGC_USB1			14
#define IMX8MP_PGC_USB2			15
#define IMX8MP_PGC_MLMIX		16
#define IMX8MP_PGC_AUDIOMIX		17
#define IMX8MP_PGC_GPU2D		18
#define IMX8MP_PGC_GPUMIX		19
#define IMX8MP_PGC_VPUMIX		20
#define IMX8MP_PGC_GPU3D		21
#define IMX8MP_PGC_MEDIAMIX		22
#define IMX8MP_PGC_VPU_G1		23
#define IMX8MP_PGC_VPU_G2		24
#define IMX8MP_PGC_VPU_VC8000E		25
#define IMX8MP_PGC_HDMIMIX		26
#define IMX8MP_PGC_HDMI			27
#define IMX8MP_PGC_MIPI2		28
#define IMX8MP_PGC_HSIOMIX		29
#define IMX8MP_PGC_MEDIA_ISP_DWP	30
#define IMX8MP_PGC_DDRMIX		31

#define GPC_PGC_CTRL(n)			(0x800 + (n) * 0x40)
#define GPC_PGC_SR(n)			(GPC_PGC_CTRL(n) + 0xc)

#define GPC_PGC_CTRL_PCR		BIT(0)

struct imx_pgc_regs {
	u16 map;
	u16 pup;
	u16 pdn;
	u16 hsk;
};

#define DOMAIN_MAX_NOC	6

struct imx_pgc_noc_data {
	u32 off;
	u32 priority;
	u32 mode;
	u32 extctrl;
};

struct imx_pgc_domain {
	struct generic_pm_domain genpd;
	struct regmap *regmap;
	struct regmap *noc_regmap;
	const struct imx_pgc_regs *regs;
	struct regulator *regulator;
	struct reset_control *reset;
	struct clk_bulk_data *clks;
	int num_clks;

	unsigned long pgc;

	const struct {
		u32 pxx;
		u32 map;
		u32 hskreq;
		u32 hskack;
	} bits;

	const int voltage;
	const bool keep_clocks;
	struct device *dev;

	unsigned int pgc_sw_pup_reg;
	unsigned int pgc_sw_pdn_reg;
	const struct imx_pgc_noc_data *noc_data[DOMAIN_MAX_NOC];
	u32 parent;
};

struct imx_pgc_domain_data {
	const struct imx_pgc_domain *domains;
	size_t domains_num;
	const struct regmap_access_table *reg_access_table;
	const struct imx_pgc_regs *pgc_regs;
};

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

	domain->num_clks = devm_clk_bulk_get_all(domain->dev, &domain->clks);
	if (domain->num_clks < 0)
		return dev_err_probe(domain->dev, domain->num_clks,
				     "Failed to get domain's clocks\n");

	domain->reset = devm_reset_control_array_get_optional_exclusive(domain->dev);
	if (IS_ERR(domain->reset))
		return dev_err_probe(domain->dev, PTR_ERR(domain->reset),
				     "Failed to get domain's resets\n");

	pm_runtime_enable(domain->dev);

	if (domain->bits.map)
		regmap_update_bits(domain->regmap, domain->regs->map,
				   domain->bits.map, domain->bits.map);

	ret = pm_genpd_init(&domain->genpd, NULL, true);
	if (ret) {
		dev_err(domain->dev, "Failed to init power domain\n");
		goto out_domain_unmap;
	}

	if (IS_ENABLED(CONFIG_LOCKDEP) &&
	    of_property_read_bool(domain->dev->of_node, "power-domains"))
		lockdep_set_subclass(&domain->genpd.mlock, 1);

	ret = of_genpd_add_provider_simple(domain->dev->of_node,
					   &domain->genpd);
	if (ret) {
		dev_err(domain->dev, "Failed to add genpd provider\n");
		goto out_genpd_remove;
	}

	if (domain->parent)
		pm_genpd_add_subdomain(pd_to_genpd(domain->dev->pm_domain), &domain->genpd);

	return 0;

out_genpd_remove:
	pm_genpd_remove(&domain->genpd);
out_domain_unmap:
	if (domain->bits.map)
		regmap_update_bits(domain->regmap, domain->regs->map,
				   domain->bits.map, 0);
	pm_runtime_disable(domain->dev);

	return ret;
}

static int imx_pgc_domain_remove(struct platform_device *pdev)
{
	struct imx_pgc_domain *domain = pdev->dev.platform_data;

	of_genpd_del_provider(domain->dev->of_node);
	pm_genpd_remove(&domain->genpd);

	if (domain->bits.map)
		regmap_update_bits(domain->regmap, domain->regs->map,
				   domain->bits.map, 0);

	pm_runtime_disable(domain->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_pgc_domain_suspend(struct device *dev)
{
	int ret;

	/*
	 * This may look strange, but is done so the generic PM_SLEEP code
	 * can power down our domain and more importantly power it up again
	 * after resume, without tripping over our usage of runtime PM to
	 * power up/down the nested domains.
	 */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	return 0;
}

static int imx_pgc_domain_resume(struct device *dev)
{
	return pm_runtime_put(dev);
}
#endif

static const struct dev_pm_ops imx_pgc_domain_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx_pgc_domain_suspend, imx_pgc_domain_resume)
};

static const struct platform_device_id imx_pgc_domain_id[] = {
	{ "imx-pgc-domain", },
	{ },
};

static struct platform_driver imx_pgc_domain_driver = {
	.driver = {
		.name = "imx-pgc",
		.pm = &imx_pgc_domain_pm_ops,
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
