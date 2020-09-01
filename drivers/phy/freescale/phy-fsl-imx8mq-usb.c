// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2017 NXP. */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>

#define PHY_CTRL0			0x0
#define PHY_CTRL0_REF_CLKDIV2		BIT(1)
#define PHY_CTRL0_REF_SSP_EN		BIT(2)
#define PHY_CTRL0_FSEL_MASK		GENMASK(10, 5)
#define PHY_CTRL0_FSEL_24M		0x2a
#define PHY_CTRL0_FSEL_100M		0x27
#define PHY_CTRL0_SSC_RANGE_MASK	GENMASK(23, 21)
#define PHY_CTRL0_SSC_RANGE_4003PPM	(0x2 << 21)

#define PHY_CTRL1			0x4
#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_COMMONONN		BIT(1)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_DCDENB		BIT(17)
#define PHY_CTRL1_CHRGSEL		BIT(18)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2			0x8
#define PHY_CTRL2_TXENABLEN0		BIT(8)
#define PHY_CTRL2_OTG_DISABLE		BIT(9)

#define PHY_CTRL5			0x14
#define PHY_CTRL5_DMPWD_OVERRIDE_SEL	BIT(23)
#define PHY_CTRL5_DMPWD_OVERRIDE	BIT(22)
#define PHY_CTRL5_DPPWD_OVERRIDE_SEL	BIT(21)
#define PHY_CTRL5_DPPWD_OVERRIDE	BIT(20)

#define PHY_CTRL6			0x18
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	BIT(29)
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

#define PHY_STS0			0x40
#define PHY_STS0_OTGSESSVLD		BIT(7)
#define PHY_STS0_CHGDET			BIT(4)
#define PHY_STS0_FSVPLUS		BIT(3)
#define PHY_STS0_FSVMINUS		BIT(2)

struct imx8mq_usb_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *base;
	struct regulator *vbus;
	struct notifier_block chg_det_nb;
	struct power_supply *vbus_power_supply;
	enum power_supply_usb_type chg_type;
};

static int imx8mq_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		   PHY_CTRL1_COMMONONN);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	value &= ~PHY_CTRL0_SSC_RANGE_MASK;
	value |= PHY_CTRL0_SSC_RANGE_4003PPM;
	writel(value, imx_phy->base + PHY_CTRL0);

	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0;
	writel(value, imx_phy->base + PHY_CTRL2);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	return 0;
}

static int imx8mp_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* USB3.0 PHY signal fsel for 24M ref */
	value = readl(imx_phy->base + PHY_CTRL0);
	value &= ~PHY_CTRL0_FSEL_MASK;
	value |= (PHY_CTRL0_FSEL_24M << 5);
	writel(value, imx_phy->base + PHY_CTRL0);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, imx_phy->base + PHY_CTRL6);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	writel(value, imx_phy->base + PHY_CTRL0);

	/* Disable OTG block */
	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	writel(value, imx_phy->base + PHY_CTRL2);

	udelay(10);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	return 0;
}

static int imx8m_usb_phy_init(struct phy *phy)
{
	if (of_device_is_compatible(phy->dev.parent->of_node,
				    "fsl,imx8mp-usb-phy"))
		return imx8mp_usb_phy_init(phy);
	else
		return imx8mq_usb_phy_init(phy);
}

static int imx8mq_phy_power_on(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;
	int ret;

	ret = regulator_enable(imx_phy->vbus);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx_phy->clk);
	if (ret)
		return ret;

	/* Disable rx term override */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	return 0;
}

static int imx8mq_phy_power_off(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* Override rx term to be 0 */
	value = readl(imx_phy->base + PHY_CTRL6);
	value |= PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	clk_disable_unprepare(imx_phy->clk);
	regulator_disable(imx_phy->vbus);

	return 0;
}

static int imx8mq_chg_data_contact_det(struct imx8mq_usb_phy *imx_phy)
{
	int i, data_pin_contact_count = 0;
	u32 val;

	/* Set DMPULLDOWN<#> = 1'b1 (to enable RDM_DWN) */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Set DPPULLDOWN<#> = 1'b0 */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Enable Data Contact Detect (DCD) per the USB BC 1.2 */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_DCDENB, imx_phy->base + PHY_CTRL1);

	for (i = 0; i < 100; i = i + 1) {
		val = readl(imx_phy->base + PHY_STS0);
		/* DP is low */
		if (!(val & PHY_STS0_FSVPLUS)) {
			if (data_pin_contact_count++ > 5)
				/* Data pin makes contact */
				break;
			usleep_range(5000, 10000);
		} else {
			data_pin_contact_count = 0;
			usleep_range(5000, 6000);
		}
	}

	/* Disable DCD after finished data contact check */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_DCDENB;
	writel(val, imx_phy->base + PHY_CTRL1);

	if (i == 100) {
		dev_err(&imx_phy->phy->dev,
			"VBUS is coming from a dedicated power supply.\n");

		/* disable override before finish */
		val = readl(imx_phy->base + PHY_CTRL5);
		val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
		writel(val, imx_phy->base + PHY_CTRL5);

		return -ENXIO;
	}

	/* Set DMPULLDOWN<#> to 1'b0 when DCD is completed */
	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~PHY_CTRL5_DMPWD_OVERRIDE_SEL;
	val |= PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	return 0;
}

static int imx8mq_chg_primary_detect(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDP_SRC is connected to D+ and IDM_SINK is connected to D- */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_CHRGSEL;
	val |= PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0;
	writel(val, imx_phy->base + PHY_CTRL1);

	usleep_range(1000, 2000);

	/* Check if D- is less than VDAT_REF to determine an SDP per BC 1.2 */
	val = readl(imx_phy->base + PHY_STS0);
	if (!(val & PHY_STS0_CHGDET)) {
		dev_dbg(&imx_phy->phy->dev, "It is a SDP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_SDP;
	}

	return 0;
}

static int imx8mq_phy_chg_secondary_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDM_SRC is connected to D- and IDP_SINK is connected to D+ */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		PHY_CTRL1_CHRGSEL, imx_phy->base + PHY_CTRL1);

	usleep_range(1000, 2000);

	/*
	 * Per BC 1.2, check voltage of D+:
	 * DCP: if greater than VDAT_REF;
	 * CDP: if less than VDAT0_REF.
	 */
	val = readl(imx_phy->base + PHY_STS0);
	if (val & PHY_STS0_CHGDET) {
		dev_dbg(&imx_phy->phy->dev, "It is a DCP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_DCP;
	} else {
		dev_dbg(&imx_phy->phy->dev, "It is a CDP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_CDP;
	}

	return 0;
}

static void imx8mq_phy_disable_chg_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
	writel(val, imx_phy->base + PHY_CTRL5);

	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~(PHY_CTRL1_DCDENB | PHY_CTRL1_VDATSRCENB0 |
		 PHY_CTRL1_VDATDETENB0 | PHY_CTRL1_CHRGSEL);
	writel(val, imx_phy->base + PHY_CTRL1);
}

static int imx8mq_phy_charger_detect(struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &imx_phy->phy->dev;
	struct device_node *np = dev->parent->of_node;
	union power_supply_propval propval;
	u32 value;
	int ret = 0;

	if (!np)
		return 0;

	imx_phy->vbus_power_supply = power_supply_get_by_phandle(np,
						"vbus-power-supply");
	if (IS_ERR_OR_NULL(imx_phy->vbus_power_supply))
		return 0;

	if (imx_phy->chg_type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
		goto put_psy;

	ret = power_supply_get_property(imx_phy->vbus_power_supply,
					POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret || propval.intval == 0) {
		dev_err(dev, "failed to get psy online infor\n");
		ret = -EINVAL;
		goto put_psy;
	}

	/* Check if vbus is valid */
	value = readl(imx_phy->base + PHY_STS0);
	if (!(value & PHY_STS0_OTGSESSVLD)) {
		dev_err(&imx_phy->phy->dev, "vbus is error\n");
		ret = -EINVAL;
		goto put_psy;
	}

	imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	ret = imx8mq_chg_data_contact_det(imx_phy);
	if (ret)
		goto put_psy;

	ret = imx8mq_chg_primary_detect(imx_phy);
	if (!ret && imx_phy->chg_type != POWER_SUPPLY_USB_TYPE_SDP)
		ret = imx8mq_phy_chg_secondary_det(imx_phy);

	imx8mq_phy_disable_chg_det(imx_phy);

	if (!ret) {
		propval.intval = imx_phy->chg_type;
		power_supply_set_property(imx_phy->vbus_power_supply,
					  POWER_SUPPLY_PROP_USB_TYPE,
					  &propval);
	}

put_psy:
	power_supply_put(imx_phy->vbus_power_supply);

	return ret;
}

static int imx8mq_phy_usb_vbus_notify(struct notifier_block *nb,
				      unsigned long val, void *v)
{
	struct imx8mq_usb_phy *imx_phy = container_of(nb, struct imx8mq_usb_phy,
						      chg_det_nb);
	struct device *dev = &imx_phy->phy->dev;
	struct device_node *np = dev->parent->of_node;
	union power_supply_propval propval;
	struct power_supply *psy = v;
	int ret;

	if (!np)
		return NOTIFY_DONE;

	imx_phy->vbus_power_supply = power_supply_get_by_phandle(np,
						"vbus-power-supply");
	if (IS_ERR_OR_NULL(imx_phy->vbus_power_supply)) {
		dev_err(dev, "failed to get power supply\n");
		return NOTIFY_DONE;
	}

	if (val == PSY_EVENT_PROP_CHANGED && psy == imx_phy->vbus_power_supply) {
		ret = power_supply_get_property(imx_phy->vbus_power_supply,
						POWER_SUPPLY_PROP_ONLINE,
						&propval);
		if (ret) {
			power_supply_put(imx_phy->vbus_power_supply);
			dev_err(dev, "failed to get psy online info\n");
			return NOTIFY_DONE;
		}

		if (propval.intval == 0)
			imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
	power_supply_put(imx_phy->vbus_power_supply);

	return NOTIFY_OK;
}

static int imx8mq_phy_set_mode(struct phy *phy, enum phy_mode mode,
			       int submode)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);

	if (mode == PHY_MODE_USB_DEVICE)
		return imx8mq_phy_charger_detect(imx_phy);

	return 0;
}

static struct phy_ops imx8mq_usb_phy_ops = {
	.init		= imx8m_usb_phy_init,
	.power_on	= imx8mq_phy_power_on,
	.power_off	= imx8mq_phy_power_off,
	.set_mode	= imx8mq_phy_set_mode,
	.owner		= THIS_MODULE,
};

static int imx8mq_usb_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct imx8mq_usb_phy *imx_phy;
	struct resource *res;

	imx_phy = devm_kzalloc(dev, sizeof(*imx_phy), GFP_KERNEL);
	if (!imx_phy)
		return -ENOMEM;

	imx_phy->clk = devm_clk_get(dev, "phy");
	if (IS_ERR(imx_phy->clk)) {
		dev_err(dev, "failed to get imx8mq usb phy clock\n");
		return PTR_ERR(imx_phy->clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imx_phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(imx_phy->base))
		return PTR_ERR(imx_phy->base);

	imx_phy->phy = devm_phy_create(dev, NULL, &imx8mq_usb_phy_ops);
	if (IS_ERR(imx_phy->phy))
		return PTR_ERR(imx_phy->phy);

	imx_phy->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(imx_phy->vbus))
		return PTR_ERR(imx_phy->vbus);

	phy_set_drvdata(imx_phy->phy, imx_phy);
	platform_set_drvdata(pdev, imx_phy);

	if (device_property_present(dev, "vbus-power-supply")) {
		imx_phy->chg_det_nb.notifier_call = imx8mq_phy_usb_vbus_notify;
		power_supply_reg_notifier(&imx_phy->chg_det_nb);
	}

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int imx8mq_usb_phy_remove(struct platform_device *pdev)
{
	struct imx8mq_usb_phy *imx_phy = platform_get_drvdata(pdev);

	if (device_property_present(&pdev->dev, "vbus-power-supply"))
		power_supply_unreg_notifier(&imx_phy->chg_det_nb);

	return 0;
}

static const struct of_device_id imx8mq_usb_phy_of_match[] = {
	{.compatible = "fsl,imx8mq-usb-phy",},
	{.compatible = "fsl,imx8mp-usb-phy",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx8mq_usb_phy_of_match);

static struct platform_driver imx8mq_usb_phy_driver = {
	.probe	= imx8mq_usb_phy_probe,
	.remove = imx8mq_usb_phy_remove,
	.driver = {
		.name	= "imx8mq-usb-phy",
		.of_match_table	= imx8mq_usb_phy_of_match,
	}
};
module_platform_driver(imx8mq_usb_phy_driver);

MODULE_DESCRIPTION("FSL IMX8MQ USB PHY driver");
MODULE_LICENSE("GPL");
