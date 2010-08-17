/*
 * Freescale STMP378X voltage regulators
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
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
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <mach/power.h>
#include <mach/regulator.h>

static int mxs_set_voltage(struct regulator_dev *reg, int MiniV, int uv)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	if (mxs_reg->rdata->set_voltage)
		return mxs_reg->rdata->set_voltage(mxs_reg, uv);
	else
		return -ENOTSUPP;
}


static int mxs_get_voltage(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	if (mxs_reg->rdata->get_voltage)
		return mxs_reg->rdata->get_voltage(mxs_reg);
	else
		return -ENOTSUPP;
}

static int mxs_set_current(struct regulator_dev *reg, int min_uA, int uA)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	if (mxs_reg->rdata->set_current)
		return mxs_reg->rdata->set_current(mxs_reg, uA);
	else
		return -ENOTSUPP;
}

static int mxs_get_current(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	if (mxs_reg->rdata->get_current)
		return mxs_reg->rdata->get_current(mxs_reg);
	else
		return -ENOTSUPP;
}

static int mxs_enable(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	return mxs_reg->rdata->enable(mxs_reg);
}

static int mxs_disable(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	return mxs_reg->rdata->disable(mxs_reg);
}

static int mxs_is_enabled(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	return mxs_reg->rdata->is_enabled(mxs_reg);
}

static int mxs_set_mode(struct regulator_dev *reg, unsigned int mode)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	return mxs_reg->rdata->set_mode(mxs_reg, mode);
}

static unsigned int mxs_get_mode(struct regulator_dev *reg)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	return mxs_reg->rdata->get_mode(mxs_reg);
}

static unsigned int mxs_get_optimum_mode(struct regulator_dev *reg,
				int input_uV, int output_uV, int load_uA)
{
	struct mxs_regulator *mxs_reg = rdev_get_drvdata(reg);

	if (mxs_reg->rdata->get_optimum_mode)
		return mxs_reg->rdata->get_optimum_mode(mxs_reg, input_uV,
							 output_uV, load_uA);
	else
		return -ENOTSUPP;
}

static struct regulator_ops mxs_rops = {
	.set_voltage	= mxs_set_voltage,
	.get_voltage	= mxs_get_voltage,
	.set_current_limit	= mxs_set_current,
	.get_current_limit	= mxs_get_current,
	.enable		= mxs_enable,
	.disable	= mxs_disable,
	.is_enabled	= mxs_is_enabled,
	.set_mode	= mxs_set_mode,
	.get_mode	= mxs_get_mode,
	.get_optimum_mode = mxs_get_optimum_mode,
};

static struct regulator_desc mxs_reg_desc[] = {
	{
		.name = "vddd",
		.id = MXS_VDDD,
		.ops = &mxs_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vdda",
		.id = MXS_VDDA,
		.ops = &mxs_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddio",
		.id = MXS_VDDIO,
		.ops = &mxs_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddd_bo",
		.id = MXS_VDDDBO,
		.ops = &mxs_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "overall_current",
		.id = MXS_OVERALL_CUR,
		.ops = &mxs_rops,
		.irq = 0,
		.type = REGULATOR_CURRENT,
		.owner = THIS_MODULE
	},
};

static int reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	unsigned long flags;
	struct mxs_regulator *sreg =
		container_of(self, struct mxs_regulator , nb);

	switch (event) {
	case MXS_REG5V_IS_USB:
		spin_lock_irqsave(&sreg->lock, flags);
		sreg->rdata->max_current = 500000;
		spin_unlock_irqrestore(&sreg->lock, flags);
		break;
	case MXS_REG5V_NOT_USB:
		spin_lock_irqsave(&sreg->lock, flags);
		sreg->rdata->max_current = 0x7fffffff;
		spin_unlock_irqrestore(&sreg->lock, flags);
		break;
	}

	return 0;
}

int mxs_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct mxs_regulator *sreg;
	struct regulator_init_data *initdata;

	sreg = platform_get_drvdata(pdev);
	initdata = pdev->dev.platform_data;
	sreg->cur_current = 0;
	sreg->next_current = 0;
	sreg->cur_voltage = 0;

	init_waitqueue_head(&sreg->wait_q);
	spin_lock_init(&sreg->lock);

	if (pdev->id > MXS_OVERALL_CUR) {
		rdesc = kzalloc(sizeof(struct regulator_desc), GFP_KERNEL);
		memcpy(rdesc, &mxs_reg_desc[MXS_OVERALL_CUR],
			sizeof(struct regulator_desc));
		rdesc->name = kstrdup(sreg->rdata->name, GFP_KERNEL);
	} else
		rdesc = &mxs_reg_desc[pdev->id];

	pr_debug("probing regulator %s %s %d\n",
			sreg->rdata->name,
			rdesc->name,
			pdev->id);

	/* register regulator */
	rdev = regulator_register(rdesc, &pdev->dev,
				  initdata, sreg);

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			rdesc->name);
		return PTR_ERR(rdev);
	}

	if (sreg->rdata->max_current) {
		struct regulator *regu;
		regu = regulator_get(NULL, sreg->rdata->name);
		sreg->nb.notifier_call = reg_callback;
		regulator_register_notifier(regu, &sreg->nb);
	}

	return 0;
}


int mxs_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;

}

int mxs_register_regulator(
		struct mxs_regulator *reg_data, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc("mxs_reg", reg);
	if (!pdev)
		return -ENOMEM;

	pdev->dev.platform_data = initdata;

	platform_set_drvdata(pdev, reg_data);
	ret = platform_device_add(pdev);

	if (ret != 0) {
		pr_debug("Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
	}
	pr_debug("register regulator %s, %d: %d\n",
			reg_data->rdata->name, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(mxs_register_regulator);

struct platform_driver mxs_reg = {
	.driver = {
		.name	= "mxs_reg",
	},
	.probe	= mxs_regulator_probe,
	.remove	= mxs_regulator_remove,
};

int mxs_regulator_init(void)
{
	return platform_driver_register(&mxs_reg);
}

void mxs_regulator_exit(void)
{
	platform_driver_unregister(&mxs_reg);
}

postcore_initcall(mxs_regulator_init);
module_exit(mxs_regulator_exit);
