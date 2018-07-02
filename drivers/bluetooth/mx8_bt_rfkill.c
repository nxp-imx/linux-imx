/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file mx8_bt_rfkill.c
 *
 * @brief This driver is implement a rfkill control interface of bluetooth
 * chip on i.MX serial boards. Register the power regulator function and
 * reset function in platform data.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>

static int system_in_suspend;

struct mxc_bt_rfkill_data {
	int bt_power_gpio;
};

struct mxc_bt_rfkill_pdata {
};

static void mxc_bt_rfkill_reset(void *rfkdata)
{
	struct mxc_bt_rfkill_data *data = rfkdata;
	printk(KERN_INFO "mxc_bt_rfkill_reset\n");
	if (gpio_is_valid(data->bt_power_gpio)) {
		mdelay(500);
		gpio_set_value_cansleep(data->bt_power_gpio, 0);
		mdelay(500);
		gpio_set_value_cansleep(data->bt_power_gpio, 1);
		mdelay(500);
	}
}

static int mxc_bt_rfkill_power_change(void *rfkdata, int status)
{
	if (status)
		mxc_bt_rfkill_reset(rfkdata);
	return 0;
}
static int mxc_bt_set_block(void *rfkdata, bool blocked)
{
	int ret;

	/* Bluetooth stack will reset the bluetooth chip during
	 * resume, since we keep bluetooth's power during suspend,
	 * don't let rfkill to actually reset the chip. */
	if (system_in_suspend)
		return 0;
	pr_info("rfkill: BT RF going to : %s\n", blocked ? "off" : "on");
	if (!blocked)
		ret = mxc_bt_rfkill_power_change(rfkdata, 1);
	else
		ret = mxc_bt_rfkill_power_change(rfkdata, 0);

	return ret;
}

static const struct rfkill_ops mxc_bt_rfkill_ops = {
	.set_block = mxc_bt_set_block,
};

static int mxc_bt_power_event(struct notifier_block *this,
							unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		system_in_suspend = 1;
		/* going to suspend, don't reset chip */
		break;
	case PM_POST_SUSPEND:
		system_in_suspend = 0;
		/* System is resume, can reset chip */
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block mxc_bt_power_notifier = {
	.notifier_call = mxc_bt_power_event,
};

#if defined(CONFIG_OF)
static const struct of_device_id mxc_bt_rfkill_dt_ids[] = {
	{ .compatible = "fsl,mxc_bt_rfkill", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxc_bt_rfkill_dt_ids);

static struct mxc_bt_rfkill_pdata *mxc_bt_rfkill_of_populate_pdata(
		struct device *dev)
{
	struct device_node *of_node = dev->of_node;
	struct mxc_bt_rfkill_pdata *pdata = dev->platform_data;

	if (!of_node || pdata)
		return pdata;

	pdata = devm_kzalloc(dev, sizeof(struct mxc_bt_rfkill_pdata),
				GFP_KERNEL);
	if (!pdata)
		return pdata;

	return pdata;
}
#endif
static int mxc_bt_rfkill_probe(struct platform_device *pdev)
{
	int rc;
	struct rfkill *rfk;
	struct mxc_bt_rfkill_data *data;
	struct device *dev = &pdev->dev;
	struct mxc_bt_rfkill_pdata *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(dev, sizeof(struct mxc_bt_rfkill_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		pdata = mxc_bt_rfkill_of_populate_pdata(&pdev->dev);
		if (!pdata)
			return -EINVAL;
	}

	data->bt_power_gpio = of_get_named_gpio(np, "bt-power-gpios", 0);
	if (data->bt_power_gpio == -EPROBE_DEFER) {
		printk(KERN_INFO "mxc_bt_rfkill: gpio not ready, need defer\n");
		return -EPROBE_DEFER;
	}
	if (gpio_is_valid(data->bt_power_gpio)) {
		printk(KERN_INFO "bt power gpio is:%d\n", data->bt_power_gpio);
		rc = devm_gpio_request_one(&pdev->dev,
								data->bt_power_gpio,
								GPIOF_OUT_INIT_HIGH,
								"BT power enable");
		if (rc) {
			dev_err(&pdev->dev, "unable to get bt-power-gpios\n");
			goto error_request_gpio;
		}
	} else  {
		printk("bt power gpio not valid (%d)!\n", data->bt_power_gpio);
	}

	rc = register_pm_notifier(&mxc_bt_power_notifier);
	if (rc)
		goto error_check_func;

	rfk = rfkill_alloc("mxc-bt", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
					&mxc_bt_rfkill_ops, data);

	if (!rfk) {
		rc = -ENOMEM;
		goto error_rfk_alloc;
	}

	rc = rfkill_register(rfk);
	if (rc)
		goto error_rfkill;

	platform_set_drvdata(pdev, rfk);
	printk(KERN_INFO "mxc_bt_rfkill driver success loaded\n");
	return 0;

error_rfkill:
	rfkill_destroy(rfk);
error_request_gpio:
error_rfk_alloc:
error_check_func:
	kfree(data);
	return rc;
}

static int  mxc_bt_rfkill_remove(struct platform_device *pdev)
{
	struct mxc_bt_rfkill_data *data = platform_get_drvdata(pdev);
	struct rfkill *rfk = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (rfk) {
		rfkill_unregister(rfk);
		rfkill_destroy(rfk);
	}
	mxc_bt_rfkill_power_change(data, 0);
	kfree(data);
	return 0;
}

static struct platform_driver mxc_bt_rfkill_driver = {
	.probe	= mxc_bt_rfkill_probe,
	.remove	= mxc_bt_rfkill_remove,
	.driver = {
		.name	= "mxc_bt_rfkill",
		.owner	= THIS_MODULE,
		.of_match_table = mxc_bt_rfkill_dt_ids,
	},
};

module_platform_driver(mxc_bt_rfkill_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("RFKill control interface of BT on MX8 Platform");
