/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define	SNVS_LPSR_REG	0x4C	/* LP Status Register */
#define	SNVS_LPCR_REG	0x38	/* LP Control Register */
#define SNVS_HPSR_REG   0x14
#define SNVS_HPSR_BTN	(0x1 << 6)
#define SNVS_LPSR_SPO	(0x1 << 18)
#define SNVS_LPCR_DEP_EN (0x1 << 5)

struct pwrkey_drv_data {
	void __iomem *ioaddr;
	int irq;
	int keycode;
	int keystate;  /* 1:pressed */
	int wakeup;
	struct timer_list check_timer;
	struct input_dev *input;
};

static void imx_imx_snvs_check_for_events(unsigned long data)
{
	struct pwrkey_drv_data *pdata = (struct pwrkey_drv_data *) data;
	struct input_dev *input = pdata->input;
	void __iomem *ioaddr = pdata->ioaddr;
	u32 state;

	state = ((readl_relaxed(ioaddr + SNVS_HPSR_REG) & SNVS_HPSR_BTN) ?
		1 : 0);

	/* only report new event if status changed */
	if (state ^ pdata->keystate) {
		pdata->keystate = state;
		input_event(input, EV_KEY, pdata->keycode, state);
		input_sync(input);
	}

	/* repeat check if pressed long */
	if (state) {
		mod_timer(&pdata->check_timer,
			  jiffies + msecs_to_jiffies(60));
	}
}

static irqreturn_t imx_snvs_pwrkey_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_status;

	lp_status = readl_relaxed(ioaddr + SNVS_LPSR_REG);
	if (lp_status & SNVS_LPSR_SPO)
		mod_timer(&pdata->check_timer, jiffies + msecs_to_jiffies(2));

	/* clear SPO status */
	writel_relaxed(lp_status, ioaddr + SNVS_LPSR_REG);

	return IRQ_HANDLED;
}

static int imx_snvs_pwrkey_probe(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = NULL;
	struct input_dev *input = NULL;
	struct device_node *np;
	void __iomem *ioaddr;
	u32 lp_cr;
	int ret = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* Get SNVS register Page */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-snvs-pwrkey");
	if (!np)
		return -ENODEV;
	pdata->ioaddr = of_iomap(np, 0);
	if (IS_ERR(pdata->ioaddr))
		return PTR_ERR(pdata->ioaddr);

	if (of_property_read_u32(np, "fsl,keycode", &pdata->keycode)) {
		pdata->keycode = KEY_POWER;
		dev_warn(&pdev->dev, "KEY_POWER without setting in dts\n");
	}

	pdata->wakeup = !!of_get_property(np, "fsl,wakeup", NULL);

	ioaddr = pdata->ioaddr;
	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0) {
		dev_err(&pdev->dev, "no irq defined in platform data\n");
		return -EINVAL;
	}

	ioaddr = pdata->ioaddr;
	lp_cr = readl_relaxed(ioaddr + SNVS_LPCR_REG);
	lp_cr |= SNVS_LPCR_DEP_EN,
	writel_relaxed(lp_cr, ioaddr + SNVS_LPCR_REG);

	setup_timer(&pdata->check_timer,
		    imx_imx_snvs_check_for_events, (unsigned long) pdata);

	if (pdata->irq >= 0) {
		ret = devm_request_irq(&pdev->dev, pdata->irq,
					imx_snvs_pwrkey_interrupt,
					IRQF_TRIGGER_HIGH, pdev->name, pdev);
		if (ret) {
			dev_err(&pdev->dev, "interrupt not available.\n");
			return ret;
		}
	}

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		return -ENOMEM;
	}

	input->name = pdev->name;
	input->phys = "snvs-pwrkey/input0";
	input->id.bustype = BUS_HOST;
	input->evbit[0] = BIT_MASK(EV_KEY);

	input_set_capability(input, EV_KEY, pdata->keycode);

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		input_free_device(input);
		return ret;
	}

	pdata->input = input;
	platform_set_drvdata(pdev, pdata);

	device_init_wakeup(&pdev->dev, pdata->wakeup);

	dev_info(&pdev->dev, "i.MX snvs powerkey probed\n");

	return 0;
}

static int imx_snvs_pwrkey_remove(struct platform_device *pdev)
{
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	input_unregister_device(pdata->input);
	del_timer_sync(&pdata->check_timer);

	return 0;
}

static int imx_snvs_pwrkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(pdata->irq);

	return 0;
}

static int imx_snvs_pwrkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pwrkey_drv_data *pdata = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(pdata->irq);

	return 0;
}

static const struct of_device_id imx_snvs_pwrkey_ids[] = {
	{ .compatible = "fsl,imx6sx-snvs-pwrkey" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_snvs_pwrkey_ids);

static SIMPLE_DEV_PM_OPS(imx_snvs_pwrkey_pm_ops, imx_snvs_pwrkey_suspend,
				imx_snvs_pwrkey_resume);

static struct platform_driver imx_snvs_pwrkey_driver = {
	.driver = {
		.name = "snvs_pwrkey",
		.owner	= THIS_MODULE,
		.pm     = &imx_snvs_pwrkey_pm_ops,
		.of_match_table = imx_snvs_pwrkey_ids,
	},
	.probe = imx_snvs_pwrkey_probe,
	.remove = imx_snvs_pwrkey_remove,
};
module_platform_driver(imx_snvs_pwrkey_driver);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("i.MX snvs power key Driver");
MODULE_LICENSE("GPL");
