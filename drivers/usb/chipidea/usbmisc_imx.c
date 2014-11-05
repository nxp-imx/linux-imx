/*
 * Copyright 2012-2014 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "ci_hdrc_imx.h"

#define MX25_USB_PHY_CTRL_OFFSET	0x08
#define MX25_BM_EXTERNAL_VBUS_DIVIDER	BIT(23)

#define MX53_USB_OTG_PHY_CTRL_0_OFFSET	0x08
#define MX53_USB_OTG_PHY_CTRL_1_OFFSET	0x0c
#define MX53_USB_UH2_CTRL_OFFSET	0x14
#define MX53_USB_UH3_CTRL_OFFSET	0x18
#define MX53_BM_OVER_CUR_DIS_H1		BIT(5)
#define MX53_BM_OVER_CUR_DIS_OTG	BIT(8)
#define MX53_BM_OVER_CUR_DIS_UHx	BIT(30)
#define MX53_USB_PHYCTRL1_PLLDIV_MASK	0x3
#define MX53_USB_PLL_DIV_24_MHZ		0x01

#define MX6SX_USB_OTG1_PHY_CTRL		0x18
#define MX6SX_USB_OTG2_PHY_CTRL		0x1c
#define MX6SX_USB_VBUS_WAKEUP_SOURCE(v)	(v << 8)
#define MX6SX_USB_VBUS_WAKEUP_SOURCE_VBUS	MX6SX_USB_VBUS_WAKEUP_SOURCE(0)
#define MX6SX_USB_VBUS_WAKEUP_SOURCE_AVALID	MX6SX_USB_VBUS_WAKEUP_SOURCE(1)
#define MX6SX_USB_VBUS_WAKEUP_SOURCE_BVALID	MX6SX_USB_VBUS_WAKEUP_SOURCE(2)
#define MX6SX_USB_VBUS_WAKEUP_SOURCE_SESS_END	MX6SX_USB_VBUS_WAKEUP_SOURCE(3)

#define MX6_BM_UNBURST_SETTING		BIT(1)
#define MX6_BM_OVER_CUR_DIS		BIT(7)
#define MX6_BM_WAKEUP_ENABLE		BIT(10)
#define MX6_BM_UTMI_ON_CLOCK		BIT(13)
#define MX6_BM_ID_WAKEUP		BIT(16)
#define MX6_BM_VBUS_WAKEUP		BIT(17)
#define MX6SX_BM_DPDM_WAKEUP_EN		BIT(29)
#define MX6_BM_WAKEUP_INTR		BIT(31)

#define MX6_USB_HSIC_CTRL_OFFSET	0x10
/* Indicating whether HSIC clock is valid */
#define MX6_BM_HSIC_CLK_VLD		BIT(31)
/* set before portsc.suspendM = 1 */
#define MX6_BM_HSIC_DEV_CONN		BIT(21)
/* HSIC enable */
#define MX6_BM_HSIC_EN			BIT(12)
/* Force HSIC module 480M clock on, even when in Host is in suspend mode */
#define MX6_BM_HSIC_CLK_ON		BIT(11)
/* Send resume signal without 480Mhz PHY clock */
#define MX6SX_BM_HSIC_AUTO_RESUME	BIT(23)

#define ANADIG_ANA_MISC0		0x150
#define ANADIG_ANA_MISC0_SET		0x154
#define ANADIG_ANA_MISC0_CLK_DELAY(x)	((x >> 26) & 0x7)

#define ANADIG_REG_3P0_SET		0x124
#define ANADIG_REG_3P0_CLR		0x128
#define ANADIG_REG_3P0_ENABLE_ILIMIT	BIT(2)
#define ANADIG_REG_3P0_ENABLE_LINREG	BIT(0)

struct usbmisc_ops {
	/* It's called once when probe a usb device */
	int (*init)(struct imx_usbmisc_data *data);
	/* It's called once after adding a usb device */
	int (*post)(struct imx_usbmisc_data *data);
	/* It's called when we need to enable usb wakeup */
	int (*set_wakeup)(struct imx_usbmisc_data *data, bool enabled);
	/* It's called before setting portsc.suspendM */
	int (*hsic_set_connect)(struct imx_usbmisc_data *data);
	/* It's called during suspend/resume */
	int (*hsic_set_clk)(struct imx_usbmisc_data *data, bool enabled);
	/* It's called when system resume from usb power lost */
	int (*power_lost_check)(struct imx_usbmisc_data *data);
	/* It's called when the vbus event happenes */
	int (*vbus_handler)(struct imx_usbmisc_data *data, bool active);
};

struct imx_usbmisc {
	void __iomem *base;
	spinlock_t lock;
	const struct usbmisc_ops *ops;
};

static struct imx_usbmisc *usbmisc;
static struct regulator *vbus_wakeup_reg;

static int usbmisc_imx25_post(struct imx_usbmisc_data *data)
{
	void __iomem *reg;
	unsigned long flags;
	u32 val;

	if (data->index > 2)
		return -EINVAL;

	reg = usbmisc->base + MX25_USB_PHY_CTRL_OFFSET;

	if (data->evdo) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(reg);
		writel(val | MX25_BM_EXTERNAL_VBUS_DIVIDER, reg);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
		usleep_range(5000, 10000); /* needed to stabilize voltage */
	}

	return 0;
}

static int usbmisc_imx53_init(struct imx_usbmisc_data *data)
{
	void __iomem *reg = NULL;
	unsigned long flags;
	u32 val = 0;

	if (data->index > 3)
		return -EINVAL;

	/* Select a 24 MHz reference clock for the PHY  */
	reg = usbmisc->base + MX53_USB_OTG_PHY_CTRL_1_OFFSET;
	val = readl(reg);
	val &= ~MX53_USB_PHYCTRL1_PLLDIV_MASK;
	val |= MX53_USB_PLL_DIV_24_MHZ;
	writel(val, usbmisc->base + MX53_USB_OTG_PHY_CTRL_1_OFFSET);

	if (data->disable_oc) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		switch (data->index) {
		case 0:
			reg = usbmisc->base + MX53_USB_OTG_PHY_CTRL_0_OFFSET;
			val = readl(reg) | MX53_BM_OVER_CUR_DIS_OTG;
			break;
		case 1:
			reg = usbmisc->base + MX53_USB_OTG_PHY_CTRL_0_OFFSET;
			val = readl(reg) | MX53_BM_OVER_CUR_DIS_H1;
			break;
		case 2:
			reg = usbmisc->base + MX53_USB_UH2_CTRL_OFFSET;
			val = readl(reg) | MX53_BM_OVER_CUR_DIS_UHx;
			break;
		case 3:
			reg = usbmisc->base + MX53_USB_UH3_CTRL_OFFSET;
			val = readl(reg) | MX53_BM_OVER_CUR_DIS_UHx;
			break;
		}
		if (reg && val)
			writel(val, reg);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
	}

	return 0;
}

static void usbmisc_imx6_init(struct imx_usbmisc_data *data)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&usbmisc->lock, flags);

	if (data->disable_oc) {
		val = readl(usbmisc->base + data->index * 4);
		writel(val | MX6_BM_OVER_CUR_DIS,
			usbmisc->base + data->index * 4);
	}

	/* SoC unburst setting */
	val = readl(usbmisc->base + data->index * 4);
	writel(val | MX6_BM_UNBURST_SETTING,
		usbmisc->base + data->index * 4);

	spin_unlock_irqrestore(&usbmisc->lock, flags);
}

static int usbmisc_imx6q_init(struct imx_usbmisc_data *data)
{
	unsigned long flags;
	u32 val;

	if (data->index > 3)
		return -EINVAL;

	usbmisc_imx6_init(data);

	/* For HSIC controller */
	if (data->index == 2 || data->index == 3) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(usbmisc->base + data->index * 4);
		writel(val | MX6_BM_UTMI_ON_CLOCK,
			usbmisc->base + data->index * 4);
		val = readl(usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		val |= MX6_BM_HSIC_EN | MX6_BM_HSIC_CLK_ON;
		writel(val, usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);

		/*
		 * Need to add delay to wait 24M OSC to be stable,
		 * It is board specific.
		 */
		regmap_read(data->anatop, ANADIG_ANA_MISC0, &val);
		/* 0 <= data->osc_clkgate_delay <= 7 */
		if (data->osc_clkgate_delay > ANADIG_ANA_MISC0_CLK_DELAY(val))
			regmap_write(data->anatop, ANADIG_ANA_MISC0_SET,
				(data->osc_clkgate_delay) << 26);
	}

	return 0;
}
static int usbmisc_imx6sx_init(struct imx_usbmisc_data *data)
{
	unsigned long flags;
	u32 val;
	void __iomem *reg;

	if (data->index > 2)
		return -EINVAL;

	usbmisc_imx6_init(data);

	/* For HSIC controller */
	if (data->index == 2) {
		spin_lock_irqsave(&usbmisc->lock, flags);
		val = readl(usbmisc->base + data->index * 4);
		writel(val | MX6_BM_UTMI_ON_CLOCK,
			usbmisc->base + data->index * 4);
		val = readl(usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		val |= MX6_BM_HSIC_EN | MX6_BM_HSIC_CLK_ON
			| MX6SX_BM_HSIC_AUTO_RESUME;
		writel(val, usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);

		/*
		 * Need to add delay to wait 24M OSC to be stable,
		 * It is board specific.
		 */
		regmap_read(data->anatop, ANADIG_ANA_MISC0, &val);
		/* 0 <= data->osc_clkgate_delay <= 7 */
		if (data->osc_clkgate_delay > ANADIG_ANA_MISC0_CLK_DELAY(val))
			regmap_write(data->anatop, ANADIG_ANA_MISC0_SET,
				(data->osc_clkgate_delay) << 26);
	}

	if (data->index == 0 || data->index == 1) {
		reg = usbmisc->base + MX6SX_USB_OTG1_PHY_CTRL + data->index * 4;
		spin_lock_irqsave(&usbmisc->lock, flags);
		/* Set vbus wakeup source as bvalid */
		val = readl(reg);
		writel(val | MX6SX_USB_VBUS_WAKEUP_SOURCE_BVALID, reg);
		/*
		 * Disable dpdm wakeup in device mode when vbus is
		 * not there.
		 */
		val = readl(usbmisc->base + data->index * 4);
		writel(val & ~MX6SX_BM_DPDM_WAKEUP_EN,
			usbmisc->base + data->index * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
	}

	return 0;
}

static int usbmisc_imx6sx_power_lost_check(struct imx_usbmisc_data *data)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&usbmisc->lock, flags);
	val = readl(usbmisc->base + data->index * 4);
	spin_unlock_irqrestore(&usbmisc->lock, flags);
	/*
	 * Here use a power on reset value to judge
	 * if the controller experienced a power lost
	 */
	if (val == 0x30001000)
		return 1;
	else
		return 0;
}

static int usbmisc_imx6q_hsic_set_connect(struct imx_usbmisc_data *data)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&usbmisc->lock, flags);
	if (data->index == 2 || data->index == 3) {
		val = readl(usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		if (!(val & MX6_BM_HSIC_DEV_CONN))
			writel(val | MX6_BM_HSIC_DEV_CONN,
				usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
				+ (data->index - 2) * 4);
	}
	spin_unlock_irqrestore(&usbmisc->lock, flags);

	return 0;
}

static int usbmisc_imx6q_hsic_set_clk
	(struct imx_usbmisc_data *data, bool on)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&usbmisc->lock, flags);
	if (data->index == 2 || data->index == 3) {
		val = readl(usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
		val |= MX6_BM_HSIC_EN | MX6_BM_HSIC_CLK_ON;
		if (on)
			val |= MX6_BM_HSIC_CLK_ON;
		else
			val &= ~MX6_BM_HSIC_CLK_ON;
		writel(val, usbmisc->base + MX6_USB_HSIC_CTRL_OFFSET
			+ (data->index - 2) * 4);
	}
	spin_unlock_irqrestore(&usbmisc->lock, flags);

	return 0;
}

static u32 imx6q_finalize_wakeup_setting(struct imx_usbmisc_data *data)
{
	if (data->available_role == USB_DR_MODE_PERIPHERAL)
		return MX6_BM_VBUS_WAKEUP;
	else if (data->available_role == USB_DR_MODE_OTG)
		return MX6_BM_VBUS_WAKEUP | MX6_BM_ID_WAKEUP;

	return 0;
}

static int usbmisc_imx6q_set_wakeup
	(struct imx_usbmisc_data *data, bool enabled)
{
	unsigned long flags;
	u32 reg, val = MX6_BM_WAKEUP_ENABLE;
	int ret = 0;

	if (data->index > 3)
		return -EINVAL;

	spin_lock_irqsave(&usbmisc->lock, flags);
	reg = readl(usbmisc->base + data->index * 4);
	if (enabled) {
		val |= imx6q_finalize_wakeup_setting(data);
		writel(reg | val, usbmisc->base + data->index * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
		if (vbus_wakeup_reg)
			ret = regulator_enable(vbus_wakeup_reg);
	} else {
		if (reg & MX6_BM_WAKEUP_INTR)
			pr_debug("wakeup int at ci_hdrc.%d\n", data->index);
		val = MX6_BM_WAKEUP_ENABLE | MX6_BM_VBUS_WAKEUP
			| MX6_BM_ID_WAKEUP;
		writel(reg & ~val, usbmisc->base + data->index * 4);
		spin_unlock_irqrestore(&usbmisc->lock, flags);
		if (vbus_wakeup_reg && regulator_is_enabled(vbus_wakeup_reg))
			regulator_disable(vbus_wakeup_reg);
	}

	return ret;
}

static int usbmisc_imx6q_vbus_handler
	(struct imx_usbmisc_data *data, bool active)
{
	if (!data->anatop)
		return 0;

	if (active) {
		regmap_write(data->anatop, ANADIG_REG_3P0_SET,
			ANADIG_REG_3P0_ENABLE_LINREG);
		regmap_write(data->anatop, ANADIG_REG_3P0_CLR,
			ANADIG_REG_3P0_ENABLE_ILIMIT);
	} else {
		regmap_write(data->anatop, ANADIG_REG_3P0_CLR,
			ANADIG_REG_3P0_ENABLE_LINREG);
		regmap_write(data->anatop, ANADIG_REG_3P0_SET,
			ANADIG_REG_3P0_ENABLE_ILIMIT);
	}

	return 0;
}

static const struct usbmisc_ops imx25_usbmisc_ops = {
	.post = usbmisc_imx25_post,
};

static const struct usbmisc_ops imx53_usbmisc_ops = {
	.init = usbmisc_imx53_init,
};

static const struct usbmisc_ops imx6q_usbmisc_ops = {
	.init = usbmisc_imx6q_init,
	.set_wakeup = usbmisc_imx6q_set_wakeup,
	.hsic_set_connect = usbmisc_imx6q_hsic_set_connect,
	.hsic_set_clk	= usbmisc_imx6q_hsic_set_clk,
	.vbus_handler	= usbmisc_imx6q_vbus_handler,
};

static const struct usbmisc_ops imx6sx_usbmisc_ops = {
	.init = usbmisc_imx6sx_init,
	.set_wakeup = usbmisc_imx6q_set_wakeup,
	.hsic_set_connect = usbmisc_imx6q_hsic_set_connect,
	.hsic_set_clk	= usbmisc_imx6q_hsic_set_clk,
	.power_lost_check = usbmisc_imx6sx_power_lost_check,
	.vbus_handler	= usbmisc_imx6q_vbus_handler,
};

int imx_usbmisc_init(struct imx_usbmisc_data *data)
{
	if (!usbmisc)
		return -EPROBE_DEFER;
	if (!usbmisc->ops->init)
		return 0;
	return usbmisc->ops->init(data);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_init);

int imx_usbmisc_init_post(struct imx_usbmisc_data *data)
{
	if (!usbmisc)
		return -EPROBE_DEFER;
	if (!usbmisc->ops->post)
		return 0;
	return usbmisc->ops->post(data);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_init_post);

int imx_usbmisc_set_wakeup(struct imx_usbmisc_data *data, bool enabled)
{
	if (!usbmisc)
		return -ENODEV;
	if (!usbmisc->ops->set_wakeup)
		return 0;
	return usbmisc->ops->set_wakeup(data, enabled);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_set_wakeup);

int imx_usbmisc_hsic_set_connect(struct imx_usbmisc_data *data)
{
	if (!usbmisc)
		return -ENODEV;
	if (!usbmisc->ops->hsic_set_connect)
		return 0;
	return usbmisc->ops->hsic_set_connect(data);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_hsic_set_connect);

int imx_usbmisc_hsic_set_clk(struct imx_usbmisc_data *data, bool on)
{
	if (!usbmisc)
		return -ENODEV;
	if (!usbmisc->ops->hsic_set_clk)
		return 0;
	return usbmisc->ops->hsic_set_clk(data, on);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_hsic_set_clk);

int imx_usbmisc_power_lost_check(struct imx_usbmisc_data *data)
{
	if (!usbmisc)
		return -ENODEV;
	if (!usbmisc->ops->power_lost_check)
		return 0;
	return usbmisc->ops->power_lost_check(data);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_power_lost_check);

int imx_usbmisc_vbus_handler(struct imx_usbmisc_data *data, bool active)
{
	if (!usbmisc)
		return -ENODEV;
	if (!usbmisc->ops->vbus_handler)
		return 0;
	return usbmisc->ops->vbus_handler(data, active);
}
EXPORT_SYMBOL_GPL(imx_usbmisc_vbus_handler);

static const struct of_device_id usbmisc_imx_dt_ids[] = {
	{
		.compatible = "fsl,imx25-usbmisc",
		.data = &imx25_usbmisc_ops,
	},
	{
		.compatible = "fsl,imx53-usbmisc",
		.data = &imx53_usbmisc_ops,
	},
	{
		.compatible = "fsl,imx6q-usbmisc",
		.data = &imx6q_usbmisc_ops,
	},
	{
		.compatible = "fsl,imx6sx-usbmisc",
		.data = &imx6sx_usbmisc_ops,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usbmisc_imx_dt_ids);

static int usbmisc_imx_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct imx_usbmisc *data;
	struct of_device_id *tmp_dev;

	if (usbmisc)
		return -EBUSY;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	tmp_dev = (struct of_device_id *)
		of_match_device(usbmisc_imx_dt_ids, &pdev->dev);
	data->ops = (const struct usbmisc_ops *)tmp_dev->data;
	usbmisc = data;

	vbus_wakeup_reg = devm_regulator_get(&pdev->dev, "vbus-wakeup");
	if (PTR_ERR(vbus_wakeup_reg) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (PTR_ERR(vbus_wakeup_reg) == -ENODEV)
		/* no vbus regualator is needed */
		vbus_wakeup_reg = NULL;
	else if (IS_ERR(vbus_wakeup_reg)) {
		dev_err(&pdev->dev, "Getting regulator error: %ld\n",
			PTR_ERR(vbus_wakeup_reg));
		return PTR_ERR(vbus_wakeup_reg);
	}

	return 0;
}

static int usbmisc_imx_remove(struct platform_device *pdev)
{
	usbmisc = NULL;
	return 0;
}

static struct platform_driver usbmisc_imx_driver = {
	.probe = usbmisc_imx_probe,
	.remove = usbmisc_imx_remove,
	.driver = {
		.name = "usbmisc_imx",
		.owner = THIS_MODULE,
		.of_match_table = usbmisc_imx_dt_ids,
	 },
};

module_platform_driver(usbmisc_imx_driver);

MODULE_ALIAS("platform:usbmisc-imx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("driver for imx usb non-core registers");
MODULE_AUTHOR("Richard Zhao <richard.zhao@freescale.com>");
