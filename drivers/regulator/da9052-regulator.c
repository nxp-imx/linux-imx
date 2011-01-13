/*
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * da9052-regulator.c: Regulator driver for DA9052
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/pm.h>

static struct regulator_ops da9052_ldo_buck_ops;


struct regulator {
	struct device *dev;
	struct list_head list;
	int uA_load;
	int min_uV;
	int max_uV;
	int enabled; /* client has called enabled */
	char *supply_name;
	struct device_attribute dev_attr;
	struct regulator_dev *rdev;
};




#define DA9052_LDO(_id, max, min, step_v, reg, mbits, cbits) \
{\
		.reg_desc	= {\
		.name		= #_id,\
		.ops		= &da9052_ldo_buck_ops,\
		.type		= REGULATOR_VOLTAGE,\
		.id			= _id,\
		.owner		= THIS_MODULE,\
	},\
	.reg_const = {\
		.max_uV		= (max) * 1000,\
		.min_uV		= (min) * 1000,\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE\
		| REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,\
		.valid_modes_mask = REGULATOR_MODE_NORMAL,\
	},\
		.step_uV		= (step_v) * 1000,\
		.reg_add		= (reg),\
		.mask_bits		= (mbits),\
		.en_bit_mask	= (cbits),\
}

struct regulator_info {
	struct regulator_desc reg_desc;
	struct regulation_constraints reg_const;
	int step_uV;
	unsigned char reg_add;
	unsigned char mask_bits;
	unsigned char en_bit_mask;
};

struct da9052_regulator_priv {
	struct da9052 *da9052;
	struct regulator_dev *regulators[];
};

struct regulator_info da9052_regulators[] = {
	/* LD01 - LDO10*/
	DA9052_LDO(DA9052_LDO1, DA9052_LDO1_VOLT_UPPER, DA9052_LDO1_VOLT_LOWER,
			DA9052_LDO1_VOLT_STEP, DA9052_LDO1_REG,
			DA9052_LDO1_VLDO1, DA9052_LDO1_LDO1EN),

	DA9052_LDO(DA9052_LDO2,
			DA9052_LDO2_VOLT_UPPER, DA9052_LDO2_VOLT_LOWER,
			DA9052_LDO2_VOLT_STEP, DA9052_LDO2_REG,
			DA9052_LDO2_VLDO2,
			DA9052_LDO2_LDO2EN),

	DA9052_LDO(DA9052_LDO3, DA9052_LDO34_VOLT_UPPER,
			DA9052_LDO34_VOLT_LOWER,
			DA9052_LDO34_VOLT_STEP, DA9052_LDO3_REG,
			DA9052_LDO3_VLDO3, DA9052_LDO3_LDO3EN),

	DA9052_LDO(DA9052_LDO4, DA9052_LDO34_VOLT_UPPER,
			DA9052_LDO34_VOLT_LOWER,
			DA9052_LDO34_VOLT_STEP, DA9052_LDO4_REG,
			DA9052_LDO4_VLDO4, DA9052_LDO4_LDO4EN),

	DA9052_LDO(DA9052_LDO5, DA9052_LDO567810_VOLT_UPPER,
			DA9052_LDO567810_VOLT_LOWER,
			DA9052_LDO567810_VOLT_STEP, DA9052_LDO5_REG,
			DA9052_LDO5_VLDO5, DA9052_LDO5_LDO5EN),

	DA9052_LDO(DA9052_LDO6, DA9052_LDO567810_VOLT_UPPER,
			DA9052_LDO567810_VOLT_LOWER,
			DA9052_LDO567810_VOLT_STEP, DA9052_LDO6_REG,
			DA9052_LDO6_VLDO6, DA9052_LDO6_LDO6EN),

	DA9052_LDO(DA9052_LDO7, DA9052_LDO567810_VOLT_UPPER,
			DA9052_LDO567810_VOLT_LOWER,
			DA9052_LDO567810_VOLT_STEP, DA9052_LDO7_REG,
			DA9052_LDO7_VLDO7, DA9052_LDO7_LDO7EN),

	DA9052_LDO(DA9052_LDO8, DA9052_LDO567810_VOLT_UPPER,
			DA9052_LDO567810_VOLT_LOWER,
			DA9052_LDO567810_VOLT_STEP, DA9052_LDO8_REG,
			DA9052_LDO8_VLDO8, DA9052_LDO8_LDO8EN),

	DA9052_LDO(DA9052_LDO9, DA9052_LDO9_VOLT_UPPER,
			DA9052_LDO9_VOLT_LOWER,
			DA9052_LDO9_VOLT_STEP,
			DA9052_LDO9_REG, DA9052_LDO9_VLDO9,
			DA9052_LDO9_LDO9EN),

	DA9052_LDO(DA9052_LDO10, DA9052_LDO567810_VOLT_UPPER,
			DA9052_LDO567810_VOLT_LOWER,
			DA9052_LDO567810_VOLT_STEP, DA9052_LDO10_REG,
			DA9052_LDO10_VLDO10, DA9052_LDO10_LDO10EN),

	/* BUCKS */
	DA9052_LDO(DA9052_BUCK_CORE, DA9052_BUCK_CORE_PRO_VOLT_UPPER,
			DA9052_BUCK_CORE_PRO_VOLT_LOWER,
			DA9052_BUCK_CORE_PRO_STEP, DA9052_BUCKCORE_REG,
			DA9052_BUCKCORE_VBCORE, DA9052_BUCKCORE_BCOREEN),

	DA9052_LDO(DA9052_BUCK_PRO, DA9052_BUCK_CORE_PRO_VOLT_UPPER,
			DA9052_BUCK_CORE_PRO_VOLT_LOWER,
			DA9052_BUCK_CORE_PRO_STEP, DA9052_BUCKPRO_REG,
			DA9052_BUCKPRO_VBPRO, DA9052_BUCKPRO_BPROEN),

	DA9052_LDO(DA9052_BUCK_MEM, DA9052_BUCK_MEM_VOLT_UPPER,
			DA9052_BUCK_MEM_VOLT_LOWER,
			DA9052_BUCK_MEM_STEP, DA9052_BUCKMEM_REG,
			DA9052_BUCKMEM_VBMEM, DA9052_BUCKMEM_BMEMEN),

	DA9052_LDO(DA9052_BUCK_PERI, DA9052_BUCK_PERI_VOLT_UPPER,
			DA9052_BUCK_PERI_VOLT_LOWER,
			DA9052_BUCK_PERI_STEP_BELOW_3000, DA9052_BUCKPERI_REG,
			DA9052_BUCKPERI_VBPERI, DA9052_BUCKPERI_BPERIEN),
};

int da9052_ldo_buck_enable(struct regulator_dev *rdev)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret = 0;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;

	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	ssc_msg.data = (ssc_msg.data | da9052_regulators[id].en_bit_mask);

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}
	da9052_unlock(priv->da9052);

	return 0;
}
EXPORT_SYMBOL_GPL(da9052_ldo_buck_enable);
/* Code added by KPIT to support additional attribure in sysfs - changestate */



int da9052_ldo_buck_disable(struct regulator_dev *rdev)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;

	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	ssc_msg.data = (ssc_msg.data & ~(da9052_regulators[id].en_bit_mask));

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}
	da9052_unlock(priv->da9052);

	return 0;
}
EXPORT_SYMBOL_GPL(da9052_ldo_buck_disable);
/* Code added by KPIT to support additional attribure in sysfs - changestate */

static int da9052_ldo_buck_is_enabled(struct regulator_dev *rdev)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int ret;
	struct da9052_ssc_msg ssc_msg;
	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;

	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}
	da9052_unlock(priv->da9052);
	return (ssc_msg.data & da9052_regulators[id].en_bit_mask) != 0;
}

int da9052_ldo_buck_set_voltage(struct regulator_dev *rdev,
					int min_uV, int max_uV)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct da9052_ssc_msg ssc_msg;
	int id = rdev_get_id(rdev);
	int ret;
	int ldo_volt = 0;

	/* KPIT - Below if condition is there for added setvoltage attribute
	in sysfs */
	if (0 == max_uV)
		max_uV = da9052_regulators[id].reg_const.max_uV;

	/* Compare voltage range */
	if (min_uV > max_uV)
		return -EINVAL;

	/* Check Minimum/ Maximum voltage range */
	if (min_uV < da9052_regulators[id].reg_const.min_uV ||
		min_uV > da9052_regulators[id].reg_const.max_uV)
		return -EINVAL;
	if (max_uV < da9052_regulators[id].reg_const.min_uV ||
		max_uV > da9052_regulators[id].reg_const.max_uV)
		return -EINVAL;

	/* Get the ldo register value */
	/* Varying step size for BUCK PERI */
	if ((da9052_regulators[id].reg_desc.id == DA9052_BUCK_PERI) &&
			(min_uV >= DA9052_BUCK_PERI_VALUES_3000)) {
		ldo_volt = (DA9052_BUCK_PERI_VALUES_3000 -
			da9052_regulators[id].reg_const.min_uV)/
			(da9052_regulators[id].step_uV);
		ldo_volt += (min_uV - DA9052_BUCK_PERI_VALUES_3000)/
			(DA9052_BUCK_PERI_STEP_ABOVE_3000);
	} else{
		ldo_volt = (min_uV - da9052_regulators[id].reg_const.min_uV)/
			(da9052_regulators[id].step_uV);
		/* Check for maximum value */
		if ((ldo_volt * da9052_regulators[id].step_uV) +
			da9052_regulators[id].reg_const.min_uV > max_uV)
			return -EINVAL;
	}

	/* Configure LDO Voltage, CONF bits */
	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;

	/* Read register */
	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	ssc_msg.data = (ssc_msg.data & ~(da9052_regulators[id].mask_bits));
	ssc_msg.data |= ldo_volt;

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	/* Set the GO LDO/BUCk bits so that the voltage changes */
	ssc_msg.addr = DA9052_SUPPLY_REG;
	ssc_msg.data = 0;

	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	switch (id) {
	case DA9052_LDO2:
		ssc_msg.data = (ssc_msg.data | DA9052_SUPPLY_VLDO2GO);
	break;
	case DA9052_LDO3:
		ssc_msg.data = (ssc_msg.data | DA9052_SUPPLY_VLDO3GO);
	break;
	case DA9052_BUCK_CORE:
		ssc_msg.data = (ssc_msg.data | DA9052_SUPPLY_VBCOREGO);
	break;
	case DA9052_BUCK_PRO:
		ssc_msg.data = (ssc_msg.data | DA9052_SUPPLY_VBPROGO);
	break;
	case DA9052_BUCK_MEM:
		ssc_msg.data = (ssc_msg.data | DA9052_SUPPLY_VBMEMGO);
	break;
	default:
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	da9052_unlock(priv->da9052);

	return 0;
}
EXPORT_SYMBOL_GPL(da9052_ldo_buck_set_voltage);
/* Code added by KPIT to support additional attributes in sysfs - setvoltage */


int da9052_ldo_buck_get_voltage(struct regulator_dev *rdev)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct da9052_ssc_msg ssc_msg;
	int id = rdev_get_id(rdev);
	int ldo_volt = 0;
	int ldo_volt_uV = 0;
	int ret;

	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;
	/* Read register */
	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}
	da9052_unlock(priv->da9052);

	ldo_volt = ssc_msg.data & da9052_regulators[id].mask_bits;
	if (da9052_regulators[id].reg_desc.id == DA9052_BUCK_PERI) {
		if (ldo_volt >= DA9052_BUCK_PERI_VALUES_UPTO_3000) {
			ldo_volt_uV = ((DA9052_BUCK_PERI_VALUES_UPTO_3000 *
				da9052_regulators[id].step_uV)
				+ da9052_regulators[id].reg_const.min_uV);
			ldo_volt_uV = (ldo_volt_uV +
				(ldo_volt - DA9052_BUCK_PERI_VALUES_UPTO_3000)
				* (DA9052_BUCK_PERI_STEP_ABOVE_3000));
		} else {
			ldo_volt_uV =
				(ldo_volt * da9052_regulators[id].step_uV)
				+ da9052_regulators[id].reg_const.min_uV;
		}
	} else {
		ldo_volt_uV = (ldo_volt * da9052_regulators[id].step_uV) +
				da9052_regulators[id].reg_const.min_uV;
	}
	return ldo_volt_uV;
}
EXPORT_SYMBOL_GPL(da9052_ldo_buck_get_voltage);
/* Code added by KPIT to support additional attributes in sysfs - setvoltage */

static int da9052_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct da9052_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct da9052_ssc_msg ssc_msg;
	int id = rdev_get_id(rdev);
	int ret;
	int ldo_volt = 0;


	/* Check Minimum/ Maximum voltage range */
	if (uV < da9052_regulators[id].reg_const.min_uV ||
		uV > da9052_regulators[id].reg_const.max_uV)
		return -EINVAL;

	/* Get the ldo register value */
	/* Varying step size for BUCK PERI */
	if ((da9052_regulators[id].reg_desc.id == DA9052_BUCK_PERI) &&
			(uV >= DA9052_BUCK_PERI_VALUES_3000)) {
		ldo_volt = (DA9052_BUCK_PERI_VALUES_3000 -
			da9052_regulators[id].reg_const.min_uV)/
			(da9052_regulators[id].step_uV);
		ldo_volt += (uV - DA9052_BUCK_PERI_VALUES_3000)/
			(DA9052_BUCK_PERI_STEP_ABOVE_3000);
	} else{
		ldo_volt = (uV - da9052_regulators[id].reg_const.min_uV)/
			(da9052_regulators[id].step_uV);
	}
	ldo_volt |= 0x80;
	dev_info(&rdev->dev, "preset to %d %x\n", uV, ldo_volt);

	/* Configure LDO Voltage, CONF bits */
	ssc_msg.addr = da9052_regulators[id].reg_add;
	ssc_msg.data = 0;

	/* Read register */
	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	ssc_msg.data = (ssc_msg.data & ~(da9052_regulators[id].mask_bits));
	ssc_msg.data |= ldo_volt;

	ret = priv->da9052->write(priv->da9052, &ssc_msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EIO;
	}

	da9052_unlock(priv->da9052);

	return 0;
}

static int da9052_ldo_buck_stby_enable(struct regulator_dev *reg)
{
	return 0;
}

static int da9052_ldo_buck_stby_disable(struct regulator_dev *reg)
{
	return 0;
}

static int da9052_ldo_buck_stby_set_mode(struct regulator_dev *reg,
					unsigned int mode)
{
	return 0;
}


static struct regulator_ops da9052_ldo_buck_ops = {
	.is_enabled = da9052_ldo_buck_is_enabled,
	.enable = da9052_ldo_buck_enable,
	.disable = da9052_ldo_buck_disable,
	.get_voltage = da9052_ldo_buck_get_voltage,
	.set_voltage = da9052_ldo_buck_set_voltage,
	.set_suspend_voltage = da9052_set_suspend_voltage,
	.set_suspend_enable = da9052_ldo_buck_stby_enable,
	.set_suspend_disable = da9052_ldo_buck_stby_disable,
	.set_suspend_mode = da9052_ldo_buck_stby_set_mode,
};


static int __devinit da9052_regulator_probe(struct platform_device *pdev)
{
	struct da9052_regulator_priv *priv;
	struct da9052_regulator_platform_data *pdata =
				(pdev->dev.platform_data);
	struct da9052 *da9052 = dev_get_drvdata(pdev->dev.parent);
	struct regulator_init_data  *init_data;
	int i, ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->da9052 = da9052;
	for (i = 0; i < 14; i++) {

		init_data = &pdata->regulators[i];
		init_data->driver_data = da9052;
		pdev->dev.platform_data = init_data;
		priv->regulators[i] = regulator_register(
				&da9052_regulators[i].reg_desc,
				&pdev->dev, init_data,
				priv);
		if (IS_ERR(priv->regulators[i])) {
			ret = PTR_ERR(priv->regulators[i]);
			goto err;
		}
	}
	platform_set_drvdata(pdev, priv);
	return 0;
err:
	while (--i >= 0)
		regulator_unregister(priv->regulators[i]);
	kfree(priv);
	return ret;
}

static int __devexit da9052_regulator_remove(struct platform_device *pdev)
{
	struct da9052_regulator_priv *priv = platform_get_drvdata(pdev);
	struct da9052_platform_data *pdata = pdev->dev.platform_data;
	int i;

	for (i = 0; i < pdata->num_regulators; i++)
		regulator_unregister(priv->regulators[i]);

	return 0;
}

static struct platform_driver da9052_regulator_driver = {
	.probe		= da9052_regulator_probe,
	.remove		= __devexit_p(da9052_regulator_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init da9052_regulator_init(void)
{
	return platform_driver_register(&da9052_regulator_driver);
}
subsys_initcall(da9052_regulator_init);

static void __exit da9052_regulator_exit(void)
{
	platform_driver_unregister(&da9052_regulator_driver);
}
module_exit(da9052_regulator_exit);

MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>");
MODULE_DESCRIPTION("Power Regulator driver for Dialog DA9052 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
