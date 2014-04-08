/*
 * Copyright 2012-2014 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Marek Vasut <marex@denx.de>
 * on behalf of DENX Software Engineering GmbH
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
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/busfreq-imx6.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/power/imx6_usb_charger.h>
#include <linux/regulator/consumer.h>

#include "ci.h"
#include "ci_hdrc_imx.h"
#include "otg.h"
#include "bits.h"

#define CI_HDRC_IMX_IMX28_WRITE_FIX		BIT(0)
#define CI_HDRC_IMX_SUPPORT_RUNTIME_PM		BIT(1)
#define CI_HDRC_IMX_HOST_QUIRK			BIT(2)
#define CI_HDRC_IMX_HAS_HSIC			BIT(3)

struct ci_hdrc_imx_platform_flag {
	unsigned int flags;
};

static const struct ci_hdrc_imx_platform_flag imx27_usb_data = {
};

static const struct ci_hdrc_imx_platform_flag imx23_usb_data = {
	.flags = CI_HDRC_IMX_HOST_QUIRK,
};

static const struct ci_hdrc_imx_platform_flag imx28_usb_data = {
	.flags = CI_HDRC_IMX_IMX28_WRITE_FIX |
		CI_HDRC_IMX_HOST_QUIRK,
};

static const struct ci_hdrc_imx_platform_flag imx6q_usb_data = {
	.flags = CI_HDRC_IMX_SUPPORT_RUNTIME_PM |
		CI_HDRC_IMX_HOST_QUIRK |
		CI_HDRC_IMX_HAS_HSIC,
};

static const struct ci_hdrc_imx_platform_flag imx6sl_usb_data = {
	.flags = CI_HDRC_IMX_SUPPORT_RUNTIME_PM |
		CI_HDRC_IMX_HOST_QUIRK |
		CI_HDRC_IMX_HAS_HSIC,
};

static const struct ci_hdrc_imx_platform_flag imx6sx_usb_data = {
	.flags = CI_HDRC_IMX_SUPPORT_RUNTIME_PM |
		CI_HDRC_IMX_HOST_QUIRK |
		CI_HDRC_IMX_HAS_HSIC,
};

static const struct of_device_id ci_hdrc_imx_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-usb", .data = &imx6sx_usb_data},
	{ .compatible = "fsl,imx6sl-usb", .data = &imx6sl_usb_data},
	{ .compatible = "fsl,imx6q-usb", .data = &imx6q_usb_data},
	{ .compatible = "fsl,imx28-usb", .data = &imx28_usb_data},
	{ .compatible = "fsl,imx23-usb", .data = &imx23_usb_data},
	{ .compatible = "fsl,imx27-usb", .data = &imx27_usb_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ci_hdrc_imx_dt_ids);

struct ci_hdrc_imx_data {
	struct usb_phy *phy;
	struct platform_device *ci_pdev;
	struct clk *clk;
	struct imx_usbmisc_data *usbmisc_data;
	bool supports_runtime_pm;
	bool in_lpm;
	bool imx6_usb_charger_detection;
	struct usb_charger charger;
	struct regmap *anatop;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_hsic_active;
	struct regulator *hsic_pad_regulator;
};

/* Common functions shared by usbmisc drivers */

static struct imx_usbmisc_data *usbmisc_get_init_data(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	struct imx_usbmisc_data *data;
	int ret;

	/*
	 * In case the fsl,usbmisc property is not present this device doesn't
	 * need usbmisc. Return NULL (which is no error here)
	 */
	if (!of_get_property(np, "fsl,usbmisc", NULL))
		return NULL;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	ret = of_parse_phandle_with_args(np, "fsl,usbmisc", "#index-cells",
					0, &args);
	if (ret) {
		dev_err(dev, "Failed to parse property fsl,usbmisc, errno %d\n",
			ret);
		return ERR_PTR(ret);
	}

	data->index = args.args[0];
	of_node_put(args.np);

	if (of_find_property(np, "disable-over-current", NULL))
		data->disable_oc = 1;

	if (of_find_property(np, "external-vbus-divider", NULL))
		data->evdo = 1;

	if (of_find_property(np, "osc-clkgate-delay", NULL)) {
		ret = of_property_read_u32(np, "osc-clkgate-delay",
			&data->osc_clkgate_delay);
		if (ret) {
			dev_err(dev,
				"failed to get osc-clkgate-delay value\n");
			return ERR_PTR(ret);
		}
		/*
		 * 0 <= osc_clkgate_delay <=7
		 * - 0x0 (default) is 0.5ms,
		 * - 0x1-0x7: 1-7ms
		 */
		if (data->osc_clkgate_delay > 7) {
			dev_err(dev,
				"value of osc-clkgate-delay is incorrect\n");
			return ERR_PTR(-EINVAL);
		}
	}

	return data;
}

/* End of common functions shared by usbmisc drivers*/

static int ci_hdrc_imx_notify_event(struct ci_hdrc *ci, unsigned event)
{
	struct device *dev = ci->dev->parent;
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret = 0;

	switch (event) {
	case CI_HDRC_CONTROLLER_CHARGER_EVENT:
		if (!data->imx6_usb_charger_detection)
			return ret;
		if (ci->vbus_active) {
			ret = imx6_usb_vbus_connect(&data->charger);
			if (!ret && data->charger.psy.type
				!= POWER_SUPPLY_TYPE_USB)
				ret = CI_HDRC_NOTIFY_RET_DEFER_EVENT;
		} else {
			ret = imx6_usb_vbus_disconnect(&data->charger);
		}
		break;
	case CI_HDRC_CONTROLLER_CHARGER_POST_EVENT:
		if (!data->imx6_usb_charger_detection)
			return ret;
		imx6_usb_charger_detect_post(&data->charger);
		break;
	case CI_HDRC_IMX_HSIC_ACTIVE_EVENT:
		if (!IS_ERR(data->pinctrl) &&
			!IS_ERR(data->pinctrl_hsic_active)) {
			ret = pinctrl_select_state(data->pinctrl
				, data->pinctrl_hsic_active);
			if (ret)
				dev_err(dev,
					 "hsic_active select failed, err=%d\n",
					 ret);
			return ret;
		}
		break;
	case CI_HDRC_IMX_HSIC_SUSPEND_EVENT:
		if (data->usbmisc_data) {
			ret = imx_usbmisc_hsic_set_connect(data->usbmisc_data);
			if (ret)
				dev_err(dev,
					 "hsic_set_connect failed, err=%d\n",
					 ret);
			return ret;
		}
		break;
	default:
		dev_dbg(dev, "unknown event\n");
	}

	return ret;
}

static int ci_hdrc_imx_probe(struct platform_device *pdev)
{
	struct ci_hdrc_imx_data *data;
	struct ci_hdrc_platform_data pdata = {
		.name		= "ci_hdrc_imx",
		.capoffset	= DEF_CAPOFFSET,
		.flags		= CI_HDRC_REQUIRE_TRANSCEIVER |
				  CI_HDRC_DISABLE_STREAMING,
		.notify_event = ci_hdrc_imx_notify_event,
	};
	int ret;
	const struct of_device_id *of_id =
			of_match_device(ci_hdrc_imx_dt_ids, &pdev->dev);
	const struct ci_hdrc_imx_platform_flag *imx_platform_flag = of_id->data;
	struct device_node *np = pdev->dev.of_node;
	struct pinctrl_state *pinctrl_hsic_idle;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate ci_hdrc-imx data!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, data);

	data->usbmisc_data = usbmisc_get_init_data(&pdev->dev);
	if (IS_ERR(data->usbmisc_data))
		return PTR_ERR(data->usbmisc_data);

	data->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(data->pinctrl)) {
		dev_dbg(&pdev->dev, "pinctrl get failed, err=%ld\n",
			PTR_ERR(data->pinctrl));
	} else {
		pinctrl_hsic_idle = pinctrl_lookup_state(data->pinctrl, "idle");
		if (IS_ERR(pinctrl_hsic_idle)) {
			dev_dbg(&pdev->dev,
				 "pinctrl_hsic_idle lookup failed, err=%ld\n",
				 PTR_ERR(pinctrl_hsic_idle));
		} else {
			ret = pinctrl_select_state(data->pinctrl,
					pinctrl_hsic_idle);
			if (ret) {
				dev_err(&pdev->dev,
					 "hsic_idle select failed, err=%d\n",
					 ret);
				return ret;
			}
		}

		data->pinctrl_hsic_active = pinctrl_lookup_state
			(data->pinctrl, "active");
		if (IS_ERR(data->pinctrl_hsic_active))
			dev_dbg(&pdev->dev,
				 "pinctrl_hsic_active lookup failed, err=%ld\n",
				 PTR_ERR(data->pinctrl_hsic_active));
	}

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev,
			"Failed to get clock, err=%ld\n", PTR_ERR(data->clk));
		return PTR_ERR(data->clk);
	}

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(data->clk);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		dev_err(&pdev->dev,
			"Failed to prepare or enable clock, err=%d\n", ret);
		return ret;
	}

	data->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "fsl,usbphy", 0);
	if (IS_ERR(data->phy)) {
		ret = PTR_ERR(data->phy);
		goto err_clk;
	}

	pdata.phy = data->phy;

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (imx_platform_flag->flags & CI_HDRC_IMX_IMX28_WRITE_FIX)
		pdata.flags |= CI_HDRC_IMX28_WRITE_FIX;

	if (imx_platform_flag->flags & CI_HDRC_IMX_SUPPORT_RUNTIME_PM) {
		pdata.flags |= CI_HDRC_SUPPORTS_RUNTIME_PM;
		data->supports_runtime_pm = true;
	}

	if (imx_platform_flag->flags & CI_HDRC_IMX_HOST_QUIRK)
		pdata.flags |= CI_HDRC_IMX_EHCI_QUIRK;

	if (data->usbmisc_data && data->usbmisc_data->index > 1 &&
		(imx_platform_flag->flags & CI_HDRC_IMX_HAS_HSIC)) {
		pdata.flags |= CI_HDRC_IMX_IS_HSIC;

		data->hsic_pad_regulator =
			devm_regulator_get(&pdev->dev, "pad");
		if (PTR_ERR(data->hsic_pad_regulator) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto err_clk;
		} else if (PTR_ERR(data->hsic_pad_regulator) == -ENODEV) {
			/* no pad regualator is needed */
			data->hsic_pad_regulator = NULL;
		} else if (IS_ERR(data->hsic_pad_regulator)) {
			dev_err(&pdev->dev,
				"Get hsic pad regulator error: %ld\n",
				PTR_ERR(data->hsic_pad_regulator));
			ret = PTR_ERR(data->hsic_pad_regulator);
			goto err_clk;
		}

		if (data->hsic_pad_regulator) {
			ret = regulator_enable(data->hsic_pad_regulator);
			if (ret) {
				dev_err(&pdev->dev,
					"Fail to enable hsic pad regulator\n");
				goto err_clk;
			}
		}
	}

	if (of_find_property(np, "imx6-usb-charger-detection", NULL))
		data->imx6_usb_charger_detection = true;

	if (of_find_property(np, "fsl,anatop", NULL)) {
		data->anatop = syscon_regmap_lookup_by_phandle
			(np, "fsl,anatop");
		if (IS_ERR(data->anatop)) {
			dev_dbg(&pdev->dev,
				"failed to find regmap for anatop\n");
			ret = PTR_ERR(data->anatop);
			goto disable_hsic_regulator;
		}
		if (data->usbmisc_data)
			data->usbmisc_data->anatop = data->anatop;
		if (data->imx6_usb_charger_detection) {
			data->charger.anatop = data->anatop;
			data->charger.dev = &pdev->dev;
			ret = imx6_usb_create_charger(&data->charger,
				"imx6_usb_charger");
			if (ret && ret != -ENODEV)
				goto disable_hsic_regulator;
			if (!ret)
				dev_dbg(&pdev->dev,
					"USB Charger is created\n");
		}
	}

	if (data->usbmisc_data) {
		ret = imx_usbmisc_init(data->usbmisc_data);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc init failed, ret=%d\n",
					ret);
			goto remove_charger;
		}
	}

	data->ci_pdev = ci_hdrc_add_device(&pdev->dev,
				pdev->resource, pdev->num_resources,
				&pdata);
	if (IS_ERR(data->ci_pdev)) {
		ret = PTR_ERR(data->ci_pdev);
		dev_err(&pdev->dev,
			"Can't register ci_hdrc platform device, err=%d\n",
			ret);
		goto remove_charger;
	}

	/* usbmisc needs to know dr mode to choose wakeup setting */
	if (data->usbmisc_data)
		data->usbmisc_data->available_role =
			ci_hdrc_query_available_role(data->ci_pdev);

	if (data->usbmisc_data) {
		ret = imx_usbmisc_init_post(data->usbmisc_data);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc post failed, ret=%d\n",
					ret);
			goto disable_device;
		}
	}

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, false);
		if (ret) {
			dev_err(&pdev->dev, "usbmisc set_wakeup failed, ret=%d\n",
					ret);
			goto disable_device;
		}
	}

	device_set_wakeup_capable(&pdev->dev, true);

	if (data->supports_runtime_pm) {
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
	}

	return 0;

disable_device:
	ci_hdrc_remove_device(data->ci_pdev);
remove_charger:
	if (data->imx6_usb_charger_detection)
		imx6_usb_remove_charger(&data->charger);
disable_hsic_regulator:
	if (data->hsic_pad_regulator)
		ret = regulator_disable(data->hsic_pad_regulator);
err_clk:
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);
	return ret;
}

static int ci_hdrc_imx_remove(struct platform_device *pdev)
{
	struct ci_hdrc_imx_data *data = platform_get_drvdata(pdev);

	ci_hdrc_remove_device(data->ci_pdev);
	if (data->supports_runtime_pm) {
		pm_runtime_get_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		pm_runtime_put_noidle(&pdev->dev);
	}
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);
	if (data->imx6_usb_charger_detection)
		imx6_usb_remove_charger(&data->charger);
	if (data->hsic_pad_regulator)
		regulator_disable(data->hsic_pad_regulator);

	return 0;
}

#ifdef CONFIG_PM
static int imx_controller_suspend(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "at %s\n", __func__);

	if (data->in_lpm)
		return 0;

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, true);
		if (ret) {
			dev_err(dev,
				"usbmisc set_wakeup failed, ret=%d\n",
				ret);
			return ret;
		}
		ret = imx_usbmisc_hsic_set_clk(data->usbmisc_data, false);
		if (ret) {
			dev_err(dev,
				"usbmisc hsic_set_clk failed, ret=%d\n",
				ret);
			goto hsic_set_clk_fail;
		}
	}

	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);
	data->in_lpm = true;

	return 0;

hsic_set_clk_fail:
	imx_usbmisc_set_wakeup(data->usbmisc_data, false);

	return ret;
}

static int imx_controller_resume(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret = 0;

	dev_dbg(dev, "at %s\n", __func__);

	if (!data->in_lpm)
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(data->clk);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		return ret;
	}

	data->in_lpm = false;

	if (data->usbmisc_data) {
		ret = imx_usbmisc_set_wakeup(data->usbmisc_data, false);
		if (ret) {
			dev_err(dev,
				"usbmisc set_wakeup failed, ret=%d\n",
				ret);
			goto clk_disable;
		}
		ret = imx_usbmisc_hsic_set_clk(data->usbmisc_data, true);
		if (ret) {
			dev_err(dev,
				"usbmisc hsic_set_clk failed, ret=%d\n",
				ret);
			goto hsic_set_clk_fail;
		}
	}

	return 0;

hsic_set_clk_fail:
	imx_usbmisc_set_wakeup(data->usbmisc_data, true);
clk_disable:
	clk_disable_unprepare(data->clk);
	release_bus_freq(BUS_FREQ_HIGH);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int ci_hdrc_imx_suspend(struct device *dev)
{
	return imx_controller_suspend(dev);
}

static int ci_hdrc_imx_resume(struct device *dev)
{
	struct ci_hdrc_imx_data *data = dev_get_drvdata(dev);
	int ret;

	ret = imx_controller_resume(dev);
	if (!ret && data->supports_runtime_pm) {
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
	}

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int ci_hdrc_imx_runtime_suspend(struct device *dev)
{
	return imx_controller_suspend(dev);
}

static int ci_hdrc_imx_runtime_resume(struct device *dev)
{
	return imx_controller_resume(dev);
}
#endif /* CONFIG_PM_RUNTIME */

#endif /* CONFIG_PM */
static const struct dev_pm_ops ci_hdrc_imx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ci_hdrc_imx_suspend, ci_hdrc_imx_resume)
	SET_RUNTIME_PM_OPS(ci_hdrc_imx_runtime_suspend,
			ci_hdrc_imx_runtime_resume, NULL)
};

static struct platform_driver ci_hdrc_imx_driver = {
	.probe = ci_hdrc_imx_probe,
	.remove = ci_hdrc_imx_remove,
	.driver = {
		.name = "imx_usb",
		.owner = THIS_MODULE,
		.of_match_table = ci_hdrc_imx_dt_ids,
		.pm = &ci_hdrc_imx_pm_ops,
	 },
};

module_platform_driver(ci_hdrc_imx_driver);

MODULE_ALIAS("platform:imx-usb");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CI HDRC i.MX USB binding");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_AUTHOR("Richard Zhao <richard.zhao@freescale.com>");
