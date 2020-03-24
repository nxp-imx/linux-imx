// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-imx8mp.c - NXP imx8mp Specific Glue layer
 *
 * Copyright (c) 2020 NXP.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>

#include "core.h"

/* USB wakeup registers */
#define USB_WAKEUP_CTRL			0x00
#define USB_WAKEUP_STATUS		0x04

/* Global wakeup interrupt enable, also used to clear interrupt */
#define USB_WAKEUP_EN			BIT(31)
/* wakeup from connection or disconnection, only for superspeed */
#define USB_WAKEUP_SS_CONN		BIT(5)
/* 0 select vbus_valid, 1 select sessvld */
#define USB_WAKEUP_VBUS_SRC_SESS_VAL	BIT(4)
/* Enable signal for wake up from u3 state */
#define USB_WAKEUP_U3_EN		BIT(3)
/* Enable signal for wake up from id change */
#define USB_WAKEUP_ID_EN		BIT(2)
/* Enable signal for wake up from vbus change */
#define	USB_WAKEUP_VBUS_EN		BIT(1)
/* Enable signal for wake up from dp/dm change */
#define USB_WAKEUP_DPDM_EN		BIT(0)

#define USB_WAKEUP_EN_MASK		GENMASK(5, 0)

struct dwc3_imx8mp {
	struct device			*dev;
	struct platform_device		*dwc3;
	void __iomem			*glue_base;
	struct clk			*bus_clk;
	struct clk			*sleep_clk;
	int				irq;
	bool				pm_suspended;
};

static void dwc_imx8mp_wakeup_enable(struct dwc3_imx8mp *dwc_imx)
{
	struct dwc3	*dwc = platform_get_drvdata(dwc_imx->dwc3);
	u32		val;

	val = readl(dwc_imx->glue_base + USB_WAKEUP_CTRL);

	if (dwc->xhci)
		val |= USB_WAKEUP_EN | USB_WAKEUP_SS_CONN |
		       USB_WAKEUP_U3_EN | USB_WAKEUP_DPDM_EN;
	else if (dwc->current_dr_role == DWC3_GCTL_PRTCAP_DEVICE)
		val |= USB_WAKEUP_EN | USB_WAKEUP_VBUS_EN |
		       USB_WAKEUP_VBUS_SRC_SESS_VAL;

	writel(val, dwc_imx->glue_base + USB_WAKEUP_CTRL);
}

static void dwc_imx8mp_wakeup_disable(struct dwc3_imx8mp *dwc_imx)
{
	u32 val;

	val = readl(dwc_imx->glue_base + USB_WAKEUP_CTRL);
	val &= ~(USB_WAKEUP_EN | USB_WAKEUP_EN_MASK);
	writel(val, dwc_imx->glue_base + USB_WAKEUP_CTRL);
}

static irqreturn_t dwc3_imx8mp_interrupt(int irq, void *_dwc_imx)
{
	struct dwc3_imx8mp	*dwc_imx = _dwc_imx;
	struct dwc3		*dwc = platform_get_drvdata(dwc_imx->dwc3);

	if (!dwc_imx->pm_suspended)
		return IRQ_HANDLED;

	/* Clear wakeup irq */
	dwc_imx8mp_wakeup_disable(dwc_imx);

	/*
	 * runtime resume xhci or gadget, dwc3_imx8mp itself
	 * as parent device will be resumed firstly by pm core
	 */
	if ((dwc->current_dr_role == DWC3_GCTL_PRTCAP_HOST) && dwc->xhci)
		pm_runtime_resume(&dwc->xhci->dev);
	else if (dwc->current_dr_role == DWC3_GCTL_PRTCAP_DEVICE)
		pm_runtime_resume(dwc->dev);

	return IRQ_HANDLED;
}

static void dwc3_imx8mp_set_role_post(struct dwc3 *dwc, u32 role)
{
	switch (role) {
	case DWC3_GCTL_PRTCAP_HOST:
		/*
		 * For xhci host, we need disable dwc core auto
		 * suspend, because during this auto suspend delay(5s),
		 * xhci host RUN_STOP is cleared and wakeup is not
		 * enabled, if device is inserted, xhci host can't
		 * response the connection.
		 */
		pm_runtime_dont_use_autosuspend(dwc->dev);
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		pm_runtime_use_autosuspend(dwc->dev);
		break;
	default:
		break;
	}
}

static int dwc3_imx8mp_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*dwc3_np, *node = dev->of_node;
	struct dwc3_imx8mp	*dwc_imx;
	struct dwc3		*dwc;
	int			error, irq;

	if (!node) {
		dev_err(dev, "device node not found\n");
		return -EINVAL;
	}

	dwc_imx = devm_kzalloc(dev, sizeof(*dwc_imx), GFP_KERNEL);
	if (!dwc_imx)
		return -ENOMEM;

	platform_set_drvdata(pdev, dwc_imx);

	dwc_imx->dev = dev;

	dwc_imx->glue_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dwc_imx->glue_base))
		return PTR_ERR(dwc_imx->glue_base);

	dwc_imx->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(dwc_imx->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(dwc_imx->bus_clk);
	}

	dwc_imx->sleep_clk = devm_clk_get(dev, "sleep");
	if (IS_ERR(dwc_imx->sleep_clk)) {
		dev_err(dev, "failed to get sleep clock\n");
		return PTR_ERR(dwc_imx->sleep_clk);
	}

	error = clk_prepare_enable(dwc_imx->bus_clk);
	if (error)
		return error;

	error = clk_prepare_enable(dwc_imx->sleep_clk);
	if (error) {
		clk_disable_unprepare(dwc_imx->bus_clk);
		return error;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		error = irq;
		goto disable_clks;
	}
	dwc_imx->irq = irq;

	error = devm_request_threaded_irq(dev, irq, NULL, dwc3_imx8mp_interrupt,
					  IRQF_ONESHOT, dev_name(dev), dwc_imx);
	if (error) {
		dev_err(dev, "failed to request IRQ #%d --> %d\n",
				irq, error);
		goto disable_clks;
	}

	dwc3_np = of_get_child_by_name(node, "dwc3");
	if (!dwc3_np) {
		dev_err(dev, "failed to find dwc3 core child\n");
		goto disable_clks;
	}

	error = of_platform_populate(node, NULL, NULL, dev);
	if (error) {
		dev_err(&pdev->dev, "failed to create dwc3 core\n");
		goto disable_clks;
	}

	dwc_imx->dwc3 = of_find_device_by_node(dwc3_np);
	if (!dwc_imx->dwc3) {
		dev_err(dev, "failed to get dwc3 platform device\n");
		error = -ENODEV;
		goto depopulate;
	}

	dwc = platform_get_drvdata(dwc_imx->dwc3);
	if (!dwc) {
		error = -EPROBE_DEFER;
		goto depopulate;
	}

	dwc->priv_data = devm_kzalloc(dev, sizeof(struct dwc3_priv_data),
				      GFP_KERNEL);
	if (!dwc->priv_data) {
		error = -ENOMEM;
		goto depopulate;
	}
	dwc->priv_data->set_role_post = dwc3_imx8mp_set_role_post;

	device_init_wakeup(dev, 1);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

depopulate:
	of_platform_depopulate(dev);
disable_clks:
	clk_disable_unprepare(dwc_imx->bus_clk);
	clk_disable_unprepare(dwc_imx->sleep_clk);

	return error;
}

static int dwc3_imx8mp_remove(struct platform_device *pdev)
{
	struct dwc3_imx8mp *dwc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	pm_runtime_get_sync(dev);
	of_platform_depopulate(dev);

	clk_disable_unprepare(dwc->bus_clk);
	clk_disable_unprepare(dwc->sleep_clk);

	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int dwc3_imx8mp_suspend(struct dwc3_imx8mp *dwc, pm_message_t msg)
{
	if (dwc->pm_suspended)
		return 0;

	/* Wakeup enable */
	if (PMSG_IS_AUTO(msg) || device_may_wakeup(dwc->dev))
		dwc_imx8mp_wakeup_enable(dwc);

	clk_disable_unprepare(dwc->bus_clk);
	dwc->pm_suspended = true;

	return 0;
}

static int dwc3_imx8mp_resume(struct dwc3_imx8mp *dwc, pm_message_t msg)
{
	int ret = 0;

	if (!dwc->pm_suspended)
		return 0;

	dwc->pm_suspended = false;
	ret = clk_prepare_enable(dwc->bus_clk);

	/* Wakeup disable */
	dwc_imx8mp_wakeup_disable(dwc);

	return ret;
}

static int __maybe_unused dwc3_imx8mp_pm_suspend(struct device *dev)
{
	struct dwc3_imx8mp *dwc_imx = dev_get_drvdata(dev);

	if (device_may_wakeup(dwc_imx->dev))
		enable_irq_wake(dwc_imx->irq);

	return dwc3_imx8mp_suspend(dwc_imx, PMSG_SUSPEND);
}

static int __maybe_unused dwc3_imx8mp_pm_resume(struct device *dev)
{
	struct dwc3_imx8mp *dwc_imx = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dwc_imx->dev))
		disable_irq_wake(dwc_imx->irq);

	ret = dwc3_imx8mp_resume(dwc_imx, PMSG_RESUME);

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return ret;
}

static int __maybe_unused dwc3_imx8mp_runtime_suspend(struct device *dev)
{
	struct dwc3_imx8mp *dwc_imx = dev_get_drvdata(dev);

	dev_dbg(dev, "dwc3 imx8mp runtime suspend.\n");

	return dwc3_imx8mp_suspend(dwc_imx, PMSG_AUTO_SUSPEND);
}

static int __maybe_unused dwc3_imx8mp_runtime_resume(struct device *dev)
{
	struct dwc3_imx8mp *dwc_imx = dev_get_drvdata(dev);

	dev_dbg(dev, "dwc3 imx8mp runtime resume.\n");

	return dwc3_imx8mp_resume(dwc_imx, PMSG_AUTO_RESUME);
}

static const struct dev_pm_ops dwc3_imx8mp_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_imx8mp_pm_suspend, dwc3_imx8mp_pm_resume)
	SET_RUNTIME_PM_OPS(dwc3_imx8mp_runtime_suspend,
			   dwc3_imx8mp_runtime_resume, NULL)
};

static const struct of_device_id dwc_imx8mp_of_match[] = {
	{ .compatible = "fsl,imx8mp-dwc3", },
	{},
};
MODULE_DEVICE_TABLE(of, dwc_imx8mp_of_match);

static struct platform_driver dwc_imx8mp_driver = {
	.probe		= dwc3_imx8mp_probe,
	.remove		= dwc3_imx8mp_remove,
	.driver		= {
		.name	= "imx8mp-dwc3",
		.pm	= &dwc3_imx8mp_dev_pm_ops,
		.of_match_table	= dwc_imx8mp_of_match,
	},
};

module_platform_driver(dwc_imx8mp_driver);

MODULE_ALIAS("platform:imx8mp-dwc3");
MODULE_AUTHOR("jun.li@nxp.com");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 imx8mp Glue Layer");
