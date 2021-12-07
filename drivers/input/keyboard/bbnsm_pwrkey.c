// SPDX-License-Identifier: GPL-2.0+
//
// Copyright 2022 NXP.

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
#include <linux/pm_wakeirq.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#define BBNSM_CTRL		0x8
#define BBNSM_INT_EN		0x10
#define BBNSM_EVENTS		0x14
#define BBNSM_PAD_CTRL		0x24

#define BBNSM_BTN_PRESSED	BIT(7)
#define BBNSM_PWR_ON		BIT(6)
#define BBNSM_BTN_OFF		BIT(5)
#define BBNSM_EMG_OFF		BIT(4)
#define BBNSM_PWRKEY_EVENTS	(BBNSM_PWR_ON | BBNSM_BTN_OFF | BBNSM_EMG_OFF)
#define BBNSM_DP_EN		BIT(24)

#define DEBOUNCE_TIME		30
#define REPEAT_INTERVAL		60

struct bbnsm_pwrkey {
	struct regmap *regmap;
	int irq;
	int keycode;
	int keystate;  /* 1:pressed */
	int wakeup;
	struct timer_list check_timer;
	struct input_dev *input;
};

static void bbnsm_pwrkey_check_for_events(struct timer_list *t)
{
	struct bbnsm_pwrkey *bbnsm = from_timer(bbnsm, t, check_timer);
	struct input_dev *input = bbnsm->input;
	u32 state;

	regmap_read(bbnsm->regmap, BBNSM_EVENTS, &state);

	state = state & BBNSM_BTN_PRESSED ? 1 : 0;

	/* only report new event if status changed */
	if (state ^ bbnsm->keystate) {
		bbnsm->keystate = state;
		input_event(input, EV_KEY, bbnsm->keycode, state);
		input_sync(input);
		pm_relax(bbnsm->input->dev.parent);
	}

	/* repeat check if pressed long */
	if (state) {
		mod_timer(&bbnsm->check_timer,
			  jiffies + msecs_to_jiffies(REPEAT_INTERVAL));
	}
}

static irqreturn_t bbnsm_pwrkey_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct bbnsm_pwrkey *bbnsm = platform_get_drvdata(pdev);
	struct input_dev *input = bbnsm->input;
	u32 event;

	regmap_read(bbnsm->regmap, BBNSM_EVENTS, &event);
	if (event & BBNSM_BTN_OFF)
		mod_timer(&bbnsm->check_timer, jiffies + msecs_to_jiffies(DEBOUNCE_TIME));
	else
		return IRQ_NONE;

	pm_wakeup_event(input->dev.parent, 0);

	/* clear PWR OFF */
	regmap_write(bbnsm->regmap, BBNSM_EVENTS, BBNSM_BTN_OFF);

	return IRQ_HANDLED;
}

static void bbnsm_pwrkey_act(void *pdata)
{
	struct bbnsm_pwrkey *bbnsm = pdata;

	del_timer_sync(&bbnsm->check_timer);
}

static int bbnsm_pwrkey_probe(struct platform_device *pdev)
{
	struct bbnsm_pwrkey *bbnsm;
	struct input_dev *input;
	struct device_node *np = pdev->dev.of_node;
	int error;

	bbnsm = devm_kzalloc(&pdev->dev, sizeof(*bbnsm), GFP_KERNEL);
	if (!bbnsm)
		return -ENOMEM;

	bbnsm->regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "regmap");
	if (IS_ERR(bbnsm->regmap)) {
		dev_err(&pdev->dev, "bbnsm pwerkey get regmap failed\n");
		return PTR_ERR(bbnsm->regmap);
	}

	if (of_property_read_u32(np, "linux,keycode", &bbnsm->keycode)) {
		bbnsm->keycode = KEY_POWER;
		dev_warn(&pdev->dev, "KEY_POWER without setting in dts\n");
	}

	bbnsm->wakeup = of_property_read_bool(np, "wakeup-source");

	bbnsm->irq = platform_get_irq(pdev, 0);
	if (bbnsm ->irq < 0)
		return -EINVAL;

	/* config the BBNSM power related register */
	regmap_update_bits(bbnsm->regmap, BBNSM_CTRL, BBNSM_DP_EN, BBNSM_DP_EN);

	/* clear the unexpected interrupt before driver ready */
	regmap_write_bits(bbnsm->regmap, BBNSM_EVENTS, BBNSM_PWRKEY_EVENTS, BBNSM_PWRKEY_EVENTS);

	timer_setup(&bbnsm->check_timer, bbnsm_pwrkey_check_for_events, 0);

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		error = -ENOMEM;
		goto error_probe;
	}

	input->name = pdev->name;
	input->phys = "bbnsm-pwrkey/input0";
	input->id.bustype = BUS_HOST;

	input_set_capability(input, EV_KEY, bbnsm->keycode);

	/* input customer action to cancel release timer */
	error = devm_add_action(&pdev->dev, bbnsm_pwrkey_act, bbnsm);
	if (error) {
		dev_err(&pdev->dev, "failed to register remove action\n");
		goto error_probe;
	}

	bbnsm->input = input;
	platform_set_drvdata(pdev, bbnsm);

	error = devm_request_irq(&pdev->dev, bbnsm->irq, bbnsm_pwrkey_interrupt,
			       IRQF_SHARED, pdev->name, pdev);
	if (error) {
		dev_err(&pdev->dev, "interrupt not available.\n");
		goto error_probe;
	}

	error = input_register_device(input);
	if (error < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto error_probe;
	}

	device_init_wakeup(&pdev->dev, bbnsm->wakeup);
	error = dev_pm_set_wake_irq(&pdev->dev, bbnsm->irq);
	if (error)
		dev_err(&pdev->dev, "irq wake enable failed.\n");

	return 0;

error_probe:
	return error;
}

static const struct of_device_id bbnsm_pwrkey_ids[] = {
	{ .compatible = "nxp,bbnsm-pwrkey" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bbnsm_pwrkey_ids);

static struct platform_driver bbnsm_pwrkey_driver = {
	.driver = {
		.name = "bbnsm_pwrkey",
		.of_match_table = bbnsm_pwrkey_ids,
	},
	.probe = bbnsm_pwrkey_probe,
};
module_platform_driver(bbnsm_pwrkey_driver);

MODULE_AUTHOR("Jacky Bai <ping.bai@nxp.com>");
MODULE_DESCRIPTION("NXP bbnsm power key Driver");
MODULE_LICENSE("GPL v2");
