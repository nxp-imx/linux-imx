// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define PHY_PLL_LOCK_WAIT_MAX_RETRIES	2000
#define IMX8MP_PCIE_PHY_FLAG_EXT_OSC	BIT(0)

#define IMX8MP_PCIE_PHY_CMN_REG020	0x80
#define  PLL_ANA_LPF_R_SEL_FINE_0_4	0x04
#define IMX8MP_PCIE_PHY_CMN_REG061	0x184
#define  ANA_PLL_CLK_OUT_TO_EXT_IO_EN	BIT(0)
#define IMX8MP_PCIE_PHY_CMN_REG062	0x188
#define  ANA_PLL_CLK_OUT_TO_EXT_IO_SEL	BIT(3)
#define IMX8MP_PCIE_PHY_CMN_REG063	0x18C
#define  AUX_PLL_REFCLK_SEL_SYS_PLL	GENMASK(7, 6)
#define IMX8MP_PCIE_PHY_CMN_REG064	0x190
#define  ANA_AUX_RX_TX_SEL_TX		BIT(7)
#define  ANA_AUX_RX_TERM_GND_EN		BIT(3)
#define  ANA_AUX_TX_TERM		BIT(2)
#define IMX8MP_PCIE_PHY_CMN_REG065	0x194
#define  ANA_AUX_RX_TERM		(BIT(7) | BIT(4))
#define  ANA_AUX_TX_LVL			GENMASK(3, 0)
#define IMX8MP_PCIE_PHY_CMN_REG076	0x200
#define  LANE_RESET_MUX_SEL		0x00
#define IMX8MP_PCIE_PHY_CMN_REG078	0x208
#define  LANE_TX_DATA_CLK_MUX_SEL	0x00

#define IMX8MP_PCIE_PHY_TRSV_REG001	0x404
#define  LN0_OVRD_TX_DRV_LVL		0x3F
#define IMX8MP_PCIE_PHY_TRSV_REG005	0x414
#define  LN0_OVRD_TX_DRV_PST_LVL_G1	0x2B
#define IMX8MP_PCIE_PHY_TRSV_REG006	0x418
#define  LN0_OVRD_TX_DRV_PST_LVL_G2	0x3
#define IMX8MP_PCIE_PHY_TRSV_REG007	0x41C
#define  LN0_OVRD_TX_DRV_PST_LVL_G3	0xA
#define IMX8MP_PCIE_PHY_TRSV_REG009	0x424
#define  LN0_OVRD_TX_DRV_PRE_LVL_G1	0x10
#define IMX8MP_PCIE_PHY_TRSV_REG059	0x4EC
#define  LN0_OVRD_RX_CTLE_RS1_G1	0x13
#define IMX8MP_PCIE_PHY_TRSV_REG060	0x4F0
#define  LN0_OVRD_RX_CTLE_RS1_G2_G3	0x25
#define IMX8MP_PCIE_PHY_TRSV_REG069	0x514
#define  LN0_ANA_RX_CTLE_IBLEED		0x7
#define IMX8MP_PCIE_PHY_TRSV_REG107	0x5AC
#define  LN0_OVRD_RX_RTERM_VCM_EN	0xB8
#define IMX8MP_PCIE_PHY_TRSV_REG109	0x5B4
#define  LN0_ANA_OVRD_RX_SQHS_DIFN_OC	0xD4
#define IMX8MP_PCIE_PHY_TRSV_REG110	0x5B8
#define  LN0_ANA_OVRD_RX_SQHS_DIFP_OC	0x6A
#define IMX8MP_PCIE_PHY_TRSV_REG158	0x678
#define  LN0_RX_CDR_FBB_FINE_G1_G2	0x55
#define IMX8MP_PCIE_PHY_TRSV_REG159	0x67C
#define  LN0_RX_CDR_FBB_FINE_G3_G4	0x53
#define IMX8MP_PCIE_PHY_TRSV_REG206	0x738
#define  LN0_TG_RX_SIGVAL_LBF_DELAY	0x4

static int imx8_pcie_phy_tuned;
struct imx8_pcie_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *base;
	u32 flags;
};

static int imx8_pcie_phy_init(struct phy *phy)
{
	u32 value, retries = 0;
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	/*
	 * SW workaround for ERR050442 of the iMX865 PCIe.
	 * Description:
	 * PCIE PHY can't support GEN3. GEN 1/2 are supported. Some
	 * buffer structures within the digital implementation in the
	 * PHY required for GEN3 operation are incorrect. This logic can
	 * be bypassed (optionally) in GEN 1/2 operation. This same
	 * implementation bug prevents SW form reading some status bits
	 * in all PCIe modes.
	 *
	 * Workaround:
	 * SW driver doesn't need to read the PHY status bits for proper
	 * PCIe operation. The buffer structure can be bypassed to
	 * completely support GEN1/2 operation. With some SW driver
	 * workarounds to read status from the PCIe PCS instead of the
	 * PHY, proper operation can be achieved for GEN1/2. Proper
	 * operation for GEN3 cannot be achieved with the SW workaround
	 * since the buffer structure cannot be bypassed in GEN3 mode.
	 */

	/* wait for pipe0_clk locked by checking status from PCS. */
	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES;
	     retries++) {
		value = readl(imx8_phy->base + 0x8188);
		if (value == BIT(1))
			break;
		udelay(10);
	}

	if (retries >= PHY_PLL_LOCK_WAIT_MAX_RETRIES) {
		pr_info("pcie phy pipe clk is not ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int imx8_pcie_phy_cal(struct phy *phy)
{
	u32 value;
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	/* export clock to ep when internal clock is used as PHY REF clock */
	if ((imx8_phy->flags & IMX8MP_PCIE_PHY_FLAG_EXT_OSC) == 0) {
		writel(ANA_PLL_CLK_OUT_TO_EXT_IO_EN,
		       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG061);
		writel(ANA_PLL_CLK_OUT_TO_EXT_IO_SEL,
		       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG062);
		writel(AUX_PLL_REFCLK_SEL_SYS_PLL,
		       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG063);
		value = ANA_AUX_RX_TX_SEL_TX | ANA_AUX_TX_TERM;
		writel(value | ANA_AUX_RX_TERM_GND_EN,
		       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG064);
		writel(ANA_AUX_RX_TERM | ANA_AUX_TX_LVL,
		       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG065);
	}

	/*
	 * Fine tune the parameters of the PHY, let PCIe link up to GEN3
	 * between two EVK boards in the EP/RC validation system.
	 */
	if (imx8_pcie_phy_tuned) {
		writel(LN0_OVRD_TX_DRV_LVL,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG001);
		writel(LN0_OVRD_TX_DRV_PST_LVL_G1,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG005);
		writel(LN0_OVRD_TX_DRV_PST_LVL_G2,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG006);
		writel(LN0_OVRD_TX_DRV_PST_LVL_G3,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG007);
		writel(LN0_OVRD_TX_DRV_PRE_LVL_G1,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG009);
		writel(LN0_OVRD_RX_CTLE_RS1_G1,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG059);
		writel(LN0_OVRD_RX_CTLE_RS1_G2_G3,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG060);
		writel(LN0_ANA_RX_CTLE_IBLEED,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG069);
		writel(LN0_OVRD_RX_RTERM_VCM_EN,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG107);
		writel(LN0_ANA_OVRD_RX_SQHS_DIFN_OC,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG109);
		writel(LN0_ANA_OVRD_RX_SQHS_DIFP_OC,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG110);
		writel(LN0_RX_CDR_FBB_FINE_G1_G2,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG158);
		writel(LN0_RX_CDR_FBB_FINE_G3_G4,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG159);
		writel(LN0_TG_RX_SIGVAL_LBF_DELAY,
		       imx8_phy->base + IMX8MP_PCIE_PHY_TRSV_REG206);
	}

	writel(PLL_ANA_LPF_R_SEL_FINE_0_4,
	       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG020);
	writel(LANE_RESET_MUX_SEL,
	       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG076);
	writel(LANE_TX_DATA_CLK_MUX_SEL,
	       imx8_phy->base + IMX8MP_PCIE_PHY_CMN_REG078);

	return 0;
}

static int imx8_pcie_phy_power_on(struct phy *phy)
{
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	return clk_prepare_enable(imx8_phy->clk);
}

static int imx8_pcie_phy_power_off(struct phy *phy)
{
	struct imx8_pcie_phy *imx8_phy = phy_get_drvdata(phy);

	clk_disable_unprepare(imx8_phy->clk);

	return 0;
}

static struct phy_ops imx8_pcie_phy_ops = {
	.init		= imx8_pcie_phy_init,
	.calibrate	= imx8_pcie_phy_cal,
	.power_on	= imx8_pcie_phy_power_on,
	.power_off	= imx8_pcie_phy_power_off,
	.owner		= THIS_MODULE,
};

static int __init imx8_pcie_phy_fine_tune(char *str)
{
	if (!strcmp(str, "yes")) {
		pr_info("i.MX PCIe PHY is fine tuned in EP/RC SYS.\n");
		imx8_pcie_phy_tuned = 1;
	}
	return 1;
}

__setup("pcie_phy_tuned=", imx8_pcie_phy_fine_tune);

static int imx8_pcie_phy_probe(struct platform_device *pdev)
{
	u32 val = 0;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx8_pcie_phy *imx8_phy;
	struct resource *res;

	imx8_phy = devm_kzalloc(dev, sizeof(*imx8_phy), GFP_KERNEL);
	if (!imx8_phy)
		return -ENOMEM;

	imx8_phy->flags &= ~IMX8MP_PCIE_PHY_FLAG_EXT_OSC;
	if (of_property_read_u32(np, "ext_osc", &val) < 0)
		/*
		 * Not specify ext_osc, use the external OSC as default
		 * CLK mode.
		 */
		imx8_phy->flags |= IMX8MP_PCIE_PHY_FLAG_EXT_OSC;
	if (val == 0)
		imx8_phy->flags &= ~IMX8MP_PCIE_PHY_FLAG_EXT_OSC;
	else if (val == 1)
		imx8_phy->flags |= IMX8MP_PCIE_PHY_FLAG_EXT_OSC;
	else
		dev_info(dev, "invalid clk mode %d.\n", val);

	imx8_phy->clk = devm_clk_get(dev, "phy");
	if (IS_ERR(imx8_phy->clk)) {
		dev_err(dev, "failed to get imx pcie phy clock\n");
		return PTR_ERR(imx8_phy->clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imx8_phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(imx8_phy->base))
		return PTR_ERR(imx8_phy->base);

	imx8_phy->phy = devm_phy_create(dev, NULL, &imx8_pcie_phy_ops);
	if (IS_ERR(imx8_phy->phy))
		return PTR_ERR(imx8_phy->phy);

	phy_set_drvdata(imx8_phy->phy, imx8_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id imx8_pcie_phy_of_match[] = {
	{.compatible = "fsl,imx8mp-pcie-phy",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx8_pcie_phy_of_match);

static struct platform_driver imx8_pcie_phy_driver = {
	.probe	= imx8_pcie_phy_probe,
	.driver = {
		.name	= "imx8-pcie-phy",
		.of_match_table	= imx8_pcie_phy_of_match,
	}
};
module_platform_driver(imx8_pcie_phy_driver);

MODULE_DESCRIPTION("FSL IMX8 PCIE PHY driver");
MODULE_LICENSE("GPL");
