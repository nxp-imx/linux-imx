/*
 * Backlight Driver for Freescale STMP37XX/STMP378X
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
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

struct stmp3xxx_bl_data {
	struct notifier_block nb;
	struct notifier_block reg_nb;
	struct notifier_block reg_init_nb;
	struct backlight_device *bd;
	struct stmp3xxx_platform_bl_data *pdata;
	int current_intensity;
	int saved_intensity;
	int stmp3xxxbl_suspended;
	int stmp3xxxbl_constrained;
};

static int stmp3xxxbl_do_probe(struct stmp3xxx_bl_data *data,
		struct stmp3xxx_platform_bl_data *pdata);
static int stmp3xxxbl_set_intensity(struct backlight_device *bd);
static inline void bl_register_reg(struct stmp3xxx_platform_bl_data *pdata,
				   struct stmp3xxx_bl_data *data);


/*
 * If we got here init is done
 */
static int bl_init_reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct stmp3xxx_bl_data *bdata;
	struct stmp3xxx_platform_bl_data *pdata;
	struct regulator *r = regulator_get(NULL, "stmp3xxx-bl-1");

	bdata = container_of(self, struct stmp3xxx_bl_data, reg_init_nb);
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
		stmp3xxxbl_set_intensity(bdata->bd);
		mutex_unlock(&bdata->bd->ops_lock);
	}

out:
	return 0;
}

static int bl_reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct stmp3xxx_bl_data *bdata;
	struct stmp3xxx_platform_bl_data *pdata;
	bdata = container_of(self, struct stmp3xxx_bl_data, reg_nb);
	pdata = bdata->pdata;

	mutex_lock(&bdata->bd->ops_lock);

	switch (event) {
	case STMP3XXX_REG5V_IS_USB:
		bdata->bd->props.max_brightness = pdata->bl_cons_intensity;
		bdata->bd->props.brightness = pdata->bl_cons_intensity;
		bdata->saved_intensity = bdata->current_intensity;
		bdata->stmp3xxxbl_constrained = 1;
		break;
	case STMP3XXX_REG5V_NOT_USB:
		bdata->bd->props.max_brightness = pdata->bl_max_intensity;
		bdata->bd->props.brightness = bdata->saved_intensity;
		bdata->stmp3xxxbl_constrained = 0;
		break;
	}

	stmp3xxxbl_set_intensity(bdata->bd);
	mutex_unlock(&bdata->bd->ops_lock);
	return 0;
}

static inline void bl_unregister_reg(struct stmp3xxx_platform_bl_data *pdata,
				  struct stmp3xxx_bl_data *data)
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

static inline void bl_register_reg(struct stmp3xxx_platform_bl_data *pdata,
				   struct stmp3xxx_bl_data *data)
{
	pdata->regulator = regulator_get(NULL, "stmp3xxx-bl-1");
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
	struct stmp3xxx_platform_fb_entry *pentry = data;
	struct stmp3xxx_bl_data *bdata;
	struct stmp3xxx_platform_bl_data *pdata;

	switch (event) {
	case STMP3XXX_LCDIF_PANEL_INIT:
		bdata = container_of(self, struct stmp3xxx_bl_data, nb);
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
			return stmp3xxxbl_do_probe(bdata, pdata);
		}
		break;

	case STMP3XXX_LCDIF_PANEL_RELEASE:
		bdata = container_of(self, struct stmp3xxx_bl_data, nb);
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
static int stmp3xxxbl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct stmp3xxx_bl_data *data = platform_get_drvdata(pdev);
	struct stmp3xxx_platform_bl_data *pdata = data->pdata;

	data->stmp3xxxbl_suspended = 1;
	if (pdata) {
		dev_dbg(&pdev->dev, "real suspend\n");
		stmp3xxxbl_set_intensity(data->bd);
	}
	return 0;
}

static int stmp3xxxbl_resume(struct platform_device *pdev)
{
	struct stmp3xxx_bl_data *data = platform_get_drvdata(pdev);
	struct stmp3xxx_platform_bl_data *pdata = data->pdata;
	int ret = 0;

	data->stmp3xxxbl_suspended = 0;
	if (pdata) {
		dev_dbg(&pdev->dev, "real resume\n");
		pdata->free_bl(pdata);
		ret = pdata->init_bl(pdata);
		if (ret)
			goto out;
		stmp3xxxbl_set_intensity(data->bd);
	}
out:
	return ret;
}
#else
#define stmp3xxxbl_suspend	NULL
#define stmp3xxxbl_resume	NULL
#endif
/*
 *  This function should be called with bd->ops_lock held
 *  Suspend/resume ?
 */
static int stmp3xxxbl_set_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct stmp3xxx_bl_data *data = platform_get_drvdata(pdev);
	struct stmp3xxx_platform_bl_data *pdata = data->pdata;

	if (pdata) {
		int ret;

		ret = pdata->set_bl_intensity(pdata, bd,
					      data->stmp3xxxbl_suspended);
		if (ret)
			bd->props.brightness = data->current_intensity;
		else
			data->current_intensity = bd->props.brightness;
		return ret;
	} else
		return -ENODEV;
}

static int stmp3xxxbl_get_intensity(struct backlight_device *bd)
{
	struct platform_device *pdev = dev_get_drvdata(&bd->dev);
	struct stmp3xxx_bl_data *data = platform_get_drvdata(pdev);

	return data->current_intensity;
}

static struct backlight_ops stmp3xxxbl_ops = {
	.get_brightness = stmp3xxxbl_get_intensity,
	.update_status  = stmp3xxxbl_set_intensity,
};

static int stmp3xxxbl_do_probe(struct stmp3xxx_bl_data *data,
		struct stmp3xxx_platform_bl_data *pdata)
{
	int ret = pdata->init_bl(pdata);

	if (ret)
		goto out;

	data->bd->props.power = FB_BLANK_UNBLANK;
	data->bd->props.fb_blank = FB_BLANK_UNBLANK;
	if (data->stmp3xxxbl_constrained) {
		data->bd->props.max_brightness = pdata->bl_cons_intensity;
		data->bd->props.brightness = pdata->bl_cons_intensity;
	} else {
		data->bd->props.max_brightness = pdata->bl_max_intensity;
		data->bd->props.brightness = pdata->bl_default_intensity;
	}

	data->pdata = pdata;
	stmp3xxxbl_set_intensity(data->bd);

out:
	return ret;
}

static int __init stmp3xxxbl_probe(struct platform_device *pdev)
{
	struct stmp3xxx_bl_data *data;
	struct stmp3xxx_platform_bl_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto out;
	}
	data->bd = backlight_device_register(pdev->name, &pdev->dev, pdev,
					&stmp3xxxbl_ops);
	if (IS_ERR(data->bd)) {
		ret = PTR_ERR(data->bd);
		goto out_1;
	}

	get_device(&pdev->dev);

	data->nb.notifier_call = bl_callback;
	stmp3xxx_lcdif_register_client(&data->nb);
	platform_set_drvdata(pdev, data);

	if (pdata) {
		ret = stmp3xxxbl_do_probe(data, pdata);
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

static int stmp3xxxbl_remove(struct platform_device *pdev)
{
	struct stmp3xxx_platform_bl_data *pdata = pdev->dev.platform_data;
	struct stmp3xxx_bl_data *data = platform_get_drvdata(pdev);
	struct backlight_device *bd = data->bd;

	bd->props.power = FB_BLANK_POWERDOWN;
	bd->props.fb_blank = FB_BLANK_POWERDOWN;
	bd->props.brightness = 0;
	data->current_intensity = bd->props.brightness;

	if (pdata) {
		pdata->set_bl_intensity(pdata, bd, data->stmp3xxxbl_suspended);
		if (pdata->free_bl)
			pdata->free_bl(pdata);
	}
	backlight_device_unregister(bd);
	if (pdata->regulator)
		regulator_put(pdata->regulator);
	put_device(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	stmp3xxx_lcdif_unregister_client(&data->nb);
	kfree(data);

	return 0;
}

static struct platform_driver stmp3xxxbl_driver = {
	.probe		= stmp3xxxbl_probe,
	.remove		= __devexit_p(stmp3xxxbl_remove),
	.suspend	= stmp3xxxbl_suspend,
	.resume		= stmp3xxxbl_resume,
	.driver		= {
		.name	= "stmp3xxx-bl",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_init(void)
{
	return platform_driver_register(&stmp3xxxbl_driver);
}

static void __exit stmp3xxx_exit(void)
{
	platform_driver_unregister(&stmp3xxxbl_driver);
}

module_init(stmp3xxx_init);
module_exit(stmp3xxx_exit);

MODULE_AUTHOR("Embedded Alley Solutions, Inc <sources@embeddedalley.com>");
MODULE_DESCRIPTION("STMP3xxx Backlight Driver");
MODULE_LICENSE("GPL");
