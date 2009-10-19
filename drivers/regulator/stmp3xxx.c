/*
 * Freescale STMP378X voltage regulators
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <mach/power.h>
#include <mach/regulator.h>

static int stmp3xxx_set_voltage(struct regulator_dev *reg, int MiniV, int uv)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	if (stmp_reg->rdata->set_voltage)
		return stmp_reg->rdata->set_voltage(stmp_reg, uv);
	else
		return -ENOTSUPP;
}


static int stmp3xxx_get_voltage(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	if (stmp_reg->rdata->get_voltage)
		return stmp_reg->rdata->get_voltage(stmp_reg);
	else
		return -ENOTSUPP;
}

static int stmp3xxx_set_current(struct regulator_dev *reg, int min_uA, int uA)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	if (stmp_reg->rdata->set_current)
		return stmp_reg->rdata->set_current(stmp_reg, uA);
	else
		return -ENOTSUPP;
}

static int stmp3xxx_get_current(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	if (stmp_reg->rdata->get_current)
		return stmp_reg->rdata->get_current(stmp_reg);
	else
		return -ENOTSUPP;
}

static int stmp3xxx_enable(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	return stmp_reg->rdata->enable(stmp_reg);
}

static int stmp3xxx_disable(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	return stmp_reg->rdata->disable(stmp_reg);
}

static int stmp3xxx_is_enabled(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	return stmp_reg->rdata->is_enabled(stmp_reg);
}

static int stmp3xxx_set_mode(struct regulator_dev *reg, unsigned int mode)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	return stmp_reg->rdata->set_mode(stmp_reg, mode);
}

static unsigned int stmp3xxx_get_mode(struct regulator_dev *reg)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	return stmp_reg->rdata->get_mode(stmp_reg);
}

static unsigned int stmp3xxx_get_optimum_mode(struct regulator_dev *reg,
				int input_uV, int output_uV, int load_uA)
{
	struct stmp3xxx_regulator *stmp_reg = rdev_get_drvdata(reg);

	if (stmp_reg->rdata->get_optimum_mode)
		return stmp_reg->rdata->get_optimum_mode(stmp_reg, input_uV,
							 output_uV, load_uA);
	else
		return -ENOTSUPP;
}

static struct regulator_ops stmp3xxx_rops = {
	.set_voltage	= stmp3xxx_set_voltage,
	.get_voltage	= stmp3xxx_get_voltage,
	.set_current_limit	= stmp3xxx_set_current,
	.get_current_limit	= stmp3xxx_get_current,
	.enable		= stmp3xxx_enable,
	.disable	= stmp3xxx_disable,
	.is_enabled	= stmp3xxx_is_enabled,
	.set_mode	= stmp3xxx_set_mode,
	.get_mode	= stmp3xxx_get_mode,
	.get_optimum_mode = stmp3xxx_get_optimum_mode,
};

static struct regulator_desc stmp3xxx_reg_desc[] = {
	{
		.name = "vddd",
		.id = STMP3XXX_VDDD,
		.ops = &stmp3xxx_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vdda",
		.id = STMP3XXX_VDDA,
		.ops = &stmp3xxx_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddio",
		.id = STMP3XXX_VDDIO,
		.ops = &stmp3xxx_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "vddd_bo",
		.id = STMP3XXX_VDDDBO,
		.ops = &stmp3xxx_rops,
		.irq = 0,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE
	},
	{
		.name = "overall_current",
		.id = STMP3XXX_OVERALL_CUR,
		.ops = &stmp3xxx_rops,
		.irq = 0,
		.type = REGULATOR_CURRENT,
		.owner = THIS_MODULE
	},
};

static int reg_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	unsigned long flags;
	struct stmp3xxx_regulator *sreg =
		container_of(self, struct stmp3xxx_regulator , nb);

	switch (event) {
	case STMP3XXX_REG5V_IS_USB:
		spin_lock_irqsave(&sreg->lock, flags);
		sreg->rdata->max_current = 500000;
		spin_unlock_irqrestore(&sreg->lock, flags);
		break;
	case STMP3XXX_REG5V_NOT_USB:
		spin_lock_irqsave(&sreg->lock, flags);
		sreg->rdata->max_current = 0x7fffffff;
		spin_unlock_irqrestore(&sreg->lock, flags);
		break;
	}

	return 0;
}

int stmp3xxx_regulator_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct stmp3xxx_regulator *sreg;
	struct regulator_init_data *initdata;

	sreg = platform_get_drvdata(pdev);
	initdata = pdev->dev.platform_data;
	sreg->cur_current = 0;
	sreg->next_current = 0;
	sreg->cur_voltage = 0;

	init_waitqueue_head(&sreg->wait_q);
	spin_lock_init(&sreg->lock);

	if (pdev->id > STMP3XXX_OVERALL_CUR) {
		rdesc = kzalloc(sizeof(struct regulator_desc), GFP_KERNEL);
		memcpy(rdesc, &stmp3xxx_reg_desc[STMP3XXX_OVERALL_CUR],
			sizeof(struct regulator_desc));
		rdesc->name = kstrdup(sreg->rdata->name, GFP_KERNEL);
	} else
		rdesc = &stmp3xxx_reg_desc[pdev->id];

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


int stmp3xxx_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;

}

int stmp3xxx_register_regulator(
		struct stmp3xxx_regulator *reg_data, int reg,
			      struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc("stmp3xxx_reg", reg);
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
EXPORT_SYMBOL_GPL(stmp3xxx_register_regulator);

struct platform_driver stmp3xxx_reg = {
	.driver = {
		.name	= "stmp3xxx_reg",
	},
	.probe	= stmp3xxx_regulator_probe,
	.remove	= stmp3xxx_regulator_remove,
};

int stmp3xxx_regulator_init(void)
{
	return platform_driver_register(&stmp3xxx_reg);
}

void stmp3xxx_regulator_exit(void)
{
	platform_driver_unregister(&stmp3xxx_reg);
}

postcore_initcall(stmp3xxx_regulator_init);
module_exit(stmp3xxx_regulator_exit);
