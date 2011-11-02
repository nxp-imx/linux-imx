/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/anatop-regulator.h>

static int anatop_set_voltage(struct regulator_dev *reg, int min_uV,
				  int max_uV, unsigned *selector)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	if (anatop_reg->rdata->set_voltage)
		return anatop_reg->rdata->set_voltage(anatop_reg, max_uV);
	else
		return -ENOTSUPP;
}

static int anatop_get_voltage(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	if (anatop_reg->rdata->get_voltage)
		return anatop_reg->rdata->get_voltage(anatop_reg);
	else
		return -ENOTSUPP;
}

static int anatop_enable(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	return anatop_reg->rdata->enable(anatop_reg);
}

static int anatop_disable(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	return anatop_reg->rdata->disable(anatop_reg);
}

static int anatop_is_enabled(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	return anatop_reg->rdata->is_enabled(anatop_reg);
}

static struct regulator_ops anatop_rops = {
	.set_voltage	= anatop_set_voltage,
	.get_voltage	= anatop_get_voltage,
	.enable		= anatop_enable,
	.disable	= anatop_disable,
	.is_enabled	= anatop_is_enabled,
};

static struct regulator_desc anatop_reg_desc[] = {
	{
		.name = "vddpu",
		.id = ANATOP_VDDPU,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddcore",
		.id = ANATOP_VDDCORE,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddsoc",
		.id = ANATOP_VDDSOC,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vdd2p5",
		.id = ANATOP_VDD2P5,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vdd1p1",
		.id = ANATOP_VDD1P1,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vdd3p0",
		.id = ANATOP_VDD3P0,
		.ops = &anatop_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
};

int anatop_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct anatop_regulator *sreg;
	struct regulator_init_data *initdata;

	sreg = platform_get_drvdata(pdev);
	initdata = pdev->dev.platform_data;
	sreg->cur_current = 0;
	sreg->next_current = 0;
	sreg->cur_voltage = 0;

	init_waitqueue_head(&sreg->wait_q);
	spin_lock_init(&sreg->lock);

	if (pdev->id > ANATOP_SUPPLY_NUM) {
		rdesc = kzalloc(sizeof(struct regulator_desc), GFP_KERNEL);
		memcpy(rdesc, &anatop_reg_desc[ANATOP_SUPPLY_NUM],
			sizeof(struct regulator_desc));
		rdesc->name = kstrdup(sreg->rdata->name, GFP_KERNEL);
	} else
		rdesc = &anatop_reg_desc[pdev->id];

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

	return 0;
}


int anatop_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;

}

int anatop_register_regulator(
		struct anatop_regulator *reg_data, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc("anatop_reg", reg);
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
EXPORT_SYMBOL_GPL(anatop_register_regulator);

struct platform_driver anatop_reg = {
	.driver = {
		.name	= "anatop_reg",
	},
	.probe	= anatop_regulator_probe,
	.remove	= anatop_regulator_remove,
};

int anatop_regulator_init(void)
{
	return platform_driver_register(&anatop_reg);
}

void anatop_regulator_exit(void)
{
	platform_driver_unregister(&anatop_reg);
}

postcore_initcall(anatop_regulator_init);
module_exit(anatop_regulator_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ANATOP Regulator driver");
MODULE_LICENSE("GPL");

