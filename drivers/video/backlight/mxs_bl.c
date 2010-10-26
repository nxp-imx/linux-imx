/*
 * Backlight Driver for Freescale MXS
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <mach/lcdif.h>
#include <mach/regulator.h>

struct mxs_bl_data {
	struct notifier_block nb;
	struct notifier_block reg_nb;
	struct notifier_block reg_init_nb;
	struct backlight_device *bd;
	struct mxs_platform_bl_data *pdata;
	int current_intensity;
	int saved_intensity;
	int mxsbl_suspended;
	int mxsbl_constrained;
};

static int mxsbl_do_probe(struct mxs_bl_data *data,
		struct mxs_platform_bl_data *pdata);
static int mxsbl_set_intensity(struct backlight_device *bd);
static inline void bl_register_reg(struct mxs_platform_bl_data *pdata,
				   struct mxs_bl_data *data);


/*
 * If we got here init is done
 */
static int bl_init_reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct mxs_bl_data *bdata;
	struct mxs_platform_bl_data *pdata;
	struct regulator *r = regulator_get(NULL, "mxs-bl-1");

	bdata = container_of(self, struct mxs_bl_data, reg_init_nb);
	pdata = bdata->pdata;

	if (r && !IS_ERR(r))
		regulator_put(r);
	else
		goto out;

	bl_register_reg(pdata, bdata);

	if (pdata->regulator) {

		printk(KERN_NOTICE"%s: setting intensity\n", __func__);

		bus_unregister_notifier(&platform_bus_type,
					&bdata->reg_init_nb);
		mutex_lock(&bdata->bd->ops_lock);
		mxsbl_set_intensity(bdata->bd);
		mutex_unlock(&bdata->bd->ops_lock);
	}

out:
	return 0;
}

static int bl_reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct mxs_bl_data *bdata;
	struct mxs_platform_bl_data *pdata;
	bdata = container_of(self, struct mxs_bl_data, reg_nb);
	pdata = bdata->pdata;

	mutex_lock(&bdata->bd->ops_lock);

	switch (event) {
	case MXS_REG5V_IS_USB:
		bdata->bd->props.max_brightness = pdata->bl_cons_intensity;
		bdata->bd->props.brightness = pdata->bl_cons_intensity;
		bdata->saved_intensity = bdata->current_intensity;
		bdata->mxsbl_constrained = 1;
		break;
	case MXS_REG5V_NOT_USB:
		bdata->bd->props.max_brightness = pdata->bl_max_intensity;
		bdata->bd->props.brightness = bdata->saved_intensity;
		bdata->mxsbl_constrained = 0;
		break;
	}

	mxsbl_set_intensity(bdata->bd);
	mutex_unlock(&bdata->bd->ops_lock);
	return 0;
}

static inline void bl_unregister_reg(struct mxs_platform_bl_data *pdata,
				  struct mxs_bl_data *data)
{
	if (!pdata)
		return;
	if (pdata->regulator)
		regulator_unregister_notifier(pdata->regulator,
					    &data->reg_nb);
	if (pdata->regulator)
		regulator_put(pdata->regulator);
	pdata->regulator = NULL;
}

static inline void bl_register_reg(struct mxs_platform_bl_data *pdata,
				   struct mxs_bl_data *data)
{
	pdata->regulator = regulator_get(NULL, "mxs-bl-1");
	if (pdata->regulator && !IS_ERR(pdata->regulator)) {
		regulator_set_mode(pdata->regulator, REGULATOR_MODE_FAST);
		if (pdata->regulator) {
			data->reg_nb.notifier_call = bl_reg_callback;
			regulator_register_notifier(pdata->regulator,
						    &data->reg_nb);
		}
	} else{
		printk(KERN_ERR "%s: failed to get regulator\n", __func__);
		pdata->regulator = NULL;
	}

}

static int bl_callback(struct notifier_block *self,
		       unsigned long event, void *data)
{
	struct mxs_platform_fb_entry *pentry = data;
	struct mxs_bl_data *bdata;
	struct mxs_platform_bl_data *pdata;

	switch (event) {
	case MXS_LCDIF_PANEL_INIT:
		bdata = container_of(self, struct mxs_bl_data, nb);
		pdata = pentry->bl_data;
		bdata->pdata = pdata;
		if (pdata) {
			bl_register_reg(pdata, bdata);
			if (!pdata->regulator) {
				/* wait for regulator to appear */
				bdata->reg_init_nb.notifier_call =
						bl_init_reg_callback;
				bus_register_notifier(&platform_bus_type,
						      &bdata->reg_init_nb);
			}
			return mxsbl_do_probe(bdata, pdata);
		}
		break;

	case MXS_LCDIF_PANEL_RELEASE:
		bdata = container_of(self, struct mxs_bl_data, nb);
		pdata = pentry->bl_data;
		if (pdata) {
			bus_unregister_notifier(&platform_bus_type,
						&bdata->reg_init_nb);
			bl_unregister_reg(pdata, bdata);
			pdata->free_bl(pdata);
		}
		bdata->pdata = NULL;
		break;
	}
	return 0;
}

#ifdef CONFIG_PM
static int mxsbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;

	data->mxsbl_suspended = 1;
	if (pdata) {
		dev_dbg(&pdev->dev, "real suspend\n");
		mxsbl_set_intensity(data->bd);
	}
	return 0;
}

static int mxsbl_resume(struct platform_device *pdev)
{
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;
	int ret = 0;

	data->mxsbl_suspended = 0;
	if (pdata) {
		dev_dbg(&pdev->dev, "real resume\n");
		pdata->free_bl(pdata);
		ret = pdata->init_bl(pdata);
		if (ret)
			goto out;
		mxsbl_set_intensity(data->bd);
	}
out:
	return ret;
}
#else
#define mxsbl_suspend	NULL
#define mxsbl_resume	NULL
#endif
/*
 *  This function should be called with bd->ops_lock held
 *  Suspend/resume ?
 */
static int mxsbl_set_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct mxs_platform_bl_data *pdata = data->pdata;

	if (pdata) {
		int ret;

		ret = pdata->set_bl_intensity(pdata, bd,
					      data->mxsbl_suspended);
		if (ret)
			bd->props.brightness = data->current_intensity;
		else
			data->current_intensity = bd->props.brightness;
		return ret;
	} else
		return -ENODEV;
}

static int mxsbl_get_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct mxs_bl_data *data = platform_get_drvdata(pdev);

	return data->current_intensity;
}

static struct backlight_ops mxsbl_ops = {
	.get_brightness = mxsbl_get_intensity,
	.update_status  = mxsbl_set_intensity,
};

static int mxsbl_do_probe(struct mxs_bl_data *data,
		struct mxs_platform_bl_data *pdata)
{
	int ret = pdata->init_bl(pdata);

	if (ret)
		goto out;

	data->bd->props.power = FB_BLANK_UNBLANK;
	data->bd->props.fb_blank = FB_BLANK_UNBLANK;

	if (!data->mxsbl_suspended)
		if (data->mxsbl_constrained) {
			data->bd->props.max_brightness = pdata->bl_cons_intensity;
			data->bd->props.brightness = pdata->bl_cons_intensity;
		} else {
			data->bd->props.max_brightness = pdata->bl_max_intensity;
			data->bd->props.brightness = pdata->bl_default_intensity;
		}

	data->pdata = pdata;
	mxsbl_set_intensity(data->bd);

out:
	return ret;
}

static int __init mxsbl_probe(struct platform_device *pdev)
{
	struct mxs_bl_data *data;
	struct mxs_platform_bl_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto out;
	}
	data->bd = backlight_device_register(pdev->name, &pdev->dev, pdev,
					&mxsbl_ops, NULL);
	if (IS_ERR(data->bd)) {
		ret = PTR_ERR(data->bd);
		goto out_1;
	}

	get_device(&pdev->dev);

	data->nb.notifier_call = bl_callback;
	mxs_lcdif_register_client(&data->nb);
	platform_set_drvdata(pdev, data);

	if (pdata) {
		ret = mxsbl_do_probe(data, pdata);
		if (ret)
			goto out_2;
	}

	goto out;

out_2:
	put_device(&pdev->dev);
out_1:
	kfree(data);
out:
	return ret;
}

static int mxsbl_remove(struct platform_device *pdev)
{
	struct mxs_platform_bl_data *pdata = pdev->dev.platform_data;
	struct mxs_bl_data *data = platform_get_drvdata(pdev);
	struct backlight_device *bd = data->bd;

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.fb_blank = FB_BLANK_POWERDOWN;
	bd->props.brightness = 0;
	data->current_intensity = bd->props.brightness;

	if (pdata) {
		pdata->set_bl_intensity(pdata, bd, data->mxsbl_suspended);
		if (pdata->free_bl)
			pdata->free_bl(pdata);
	}
	backlight_device_unregister(bd);
	if (pdata->regulator)
		regulator_put(pdata->regulator);
	put_device(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	mxs_lcdif_unregister_client(&data->nb);
	kfree(data);

	return 0;
}

static struct platform_driver mxsbl_driver = {
	.probe		= mxsbl_probe,
	.remove		= __devexit_p(mxsbl_remove),
	.suspend	= mxsbl_suspend,
	.resume		= mxsbl_resume,
	.driver		= {
		.name	= "mxs-bl",
		.owner	= THIS_MODULE,
	},
};

static int __init mxs_init(void)
{
	return platform_driver_register(&mxsbl_driver);
}

static void __exit mxs_exit(void)
{
	platform_driver_unregister(&mxsbl_driver);
}

module_init(mxs_init);
module_exit(mxs_exit);

MODULE_AUTHOR("Embedded Alley Solutions, Inc <sources@embeddedalley.com>");
MODULE_DESCRIPTION("MXS Backlight Driver");
MODULE_LICENSE("GPL");
