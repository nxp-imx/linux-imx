/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/mfd/syscon.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define LDO_RAMP_UP_UNIT_IN_CYCLES      64 /* 64 cycles per step */
#define LDO_RAMP_UP_FREQ_IN_MHZ         24 /* cycle based on 24M OSC */

#define REG_SET         0x4
#define REG_CLR         0x8
#define SOC_PU_FIELD_OFFSET   0x9
#define ANATOP_CORE_REG_OFFSET	0x140

/*
 * for CORE, SOC and PU regulator, the register field
 * has following definition: 00001 -- Target core voltage
 * = 0.725V, which means the lowest setting in this
 * field is 0.725V once the regulator is enabled. So
 * when these regulators are turned on from off status,
 * we need to count the voltage step of 0V to 0.7V, it will
 * need additional delay, so the additional step number is
 * 700mV / 25mV = 28.
 */
#define CORE_REG_ENABLE_STEP_ADD 28

struct anatop_regulator {
	const char *name;
	u32 control_reg;
	struct regmap *anatop;
	int vol_bit_shift;
	int vol_bit_width;
	u32 delay_reg;
	int delay_bit_shift;
	int delay_bit_width;
	int min_bit_val;
	int min_voltage;
	int max_voltage;
	struct regulator_desc rdesc;
	struct regulator_init_data *initdata;
	u32 enable_bit;
};

static int anatop_regmap_set_voltage_sel(struct regulator_dev *reg,
					unsigned selector)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	return regulator_set_voltage_sel_regmap(reg, selector);
}

static int anatop_regmap_set_voltage_time_sel(struct regulator_dev *reg,
	unsigned int old_sel,
	unsigned int new_sel)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);
	u32 val;
	int ret = 0;

	/* check whether need to care about LDO ramp up speed */
	if (anatop_reg->delay_bit_width && new_sel > old_sel) {
		/*
		 * the delay for LDO ramp up time is
		 * based on the register setting, we need
		 * to calculate how many steps LDO need to
		 * ramp up, and how much delay needed. (us)
		 */
		regmap_read(anatop_reg->anatop, anatop_reg->delay_reg, &val);
		val = (val >> anatop_reg->delay_bit_shift) &
			((1 << anatop_reg->delay_bit_width) - 1);
		ret = (new_sel - old_sel) * (LDO_RAMP_UP_UNIT_IN_CYCLES <<
			val) / LDO_RAMP_UP_FREQ_IN_MHZ + 1;
	}

	return ret;
}

static int anatop_regmap_get_voltage_sel(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	return regulator_get_voltage_sel_regmap(reg);
}

/*
 * currently on anatop regulators, only PU regulator supports
 * enable/disable function, and its voltage must be equal
 * to SOC voltage, so we need to get SOC voltage then set
 * into PU regulator. Other regulators are always on due
 * to hardware design, so enable/disable/is_enabled/enable_time
 * functions are only used by PU regulator.
 */
static int anatop_regmap_enable(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);
	u32 val;

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	regmap_read(anatop_reg->anatop, anatop_reg->control_reg, &val);
	if (anatop_reg->control_reg == ANATOP_CORE_REG_OFFSET) {
		val &= ((1 << anatop_reg->vol_bit_width) - 1) <<
			(anatop_reg->vol_bit_shift + SOC_PU_FIELD_OFFSET);
		val >>= SOC_PU_FIELD_OFFSET;
	} else {
		/* Only non-core regulator has enable bit */
		val = 1 << anatop_reg->enable_bit;
	}
	regmap_write(anatop_reg->anatop, anatop_reg->control_reg + REG_SET,
			val);

	return 0;
}

static int anatop_regmap_disable(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);
	u32 val;

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	if (anatop_reg->control_reg == ANATOP_CORE_REG_OFFSET)
		val = ((1 << anatop_reg->vol_bit_width) - 1)
			<< anatop_reg->vol_bit_shift;
	else
		val = 1 << anatop_reg->enable_bit;
	regmap_write(anatop_reg->anatop, anatop_reg->control_reg + REG_CLR,
			val);

	return 0;
}

static int anatop_regmap_is_enabled(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);
	u32 val;

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	regmap_read(anatop_reg->anatop, anatop_reg->control_reg, &val);
	if (anatop_reg->control_reg == ANATOP_CORE_REG_OFFSET) {
		val = (val >> anatop_reg->vol_bit_shift) &
			((1 << anatop_reg->vol_bit_width) - 1);
		/*
		 * look internal pu regulator as disabled
		 * in ldo-bypass mode(0x1f)
		 */
		if ((!strcmp(reg->desc->name, "vddpu")) && (val == 0x1f))
			val = 0;
		else
			val = val ? 1 : 0;
	} else {
		val = !!(val & (1 << anatop_reg->enable_bit));
	}

	return val;
}

static int anatop_regmap_enable_time(struct regulator_dev *reg)
{
	struct anatop_regulator *anatop_reg = rdev_get_drvdata(reg);
	u32 val, soc_val;

	if (!anatop_reg->control_reg)
		return -ENOTSUPP;

	regmap_read(anatop_reg->anatop, anatop_reg->control_reg, &val);
	soc_val = (val >> (anatop_reg->vol_bit_shift +
		SOC_PU_FIELD_OFFSET)) &
		((1 << anatop_reg->vol_bit_width) - 1);

	return anatop_regmap_set_voltage_time_sel(reg, 0,
		soc_val + CORE_REG_ENABLE_STEP_ADD);
}

static struct regulator_ops anatop_rops = {
	.set_voltage_sel = anatop_regmap_set_voltage_sel,
	.set_voltage_time_sel = anatop_regmap_set_voltage_time_sel,
	.get_voltage_sel = anatop_regmap_get_voltage_sel,
	.enable = anatop_regmap_enable,
	.disable = anatop_regmap_disable,
	.is_enabled = anatop_regmap_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.enable_time = anatop_regmap_enable_time,
};

static int anatop_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *anatop_np;
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct anatop_regulator *sreg;
	struct regulator_init_data *initdata;
	struct regulator_config config = { };
	int ret = 0;

	initdata = of_get_regulator_init_data(dev, np);
	sreg = devm_kzalloc(dev, sizeof(*sreg), GFP_KERNEL);
	if (!sreg)
		return -ENOMEM;
	sreg->initdata = initdata;
	sreg->name = kstrdup(of_get_property(np, "regulator-name", NULL),
			     GFP_KERNEL);
	rdesc = &sreg->rdesc;
	memset(rdesc, 0, sizeof(*rdesc));
	rdesc->name = sreg->name;
	rdesc->ops = &anatop_rops;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->owner = THIS_MODULE;

	anatop_np = of_get_parent(np);
	if (!anatop_np)
		return -ENODEV;
	sreg->anatop = syscon_node_to_regmap(anatop_np);
	of_node_put(anatop_np);
	if (IS_ERR(sreg->anatop))
		return PTR_ERR(sreg->anatop);

	ret = of_property_read_u32(np, "anatop-reg-offset",
				   &sreg->control_reg);
	if (ret) {
		dev_err(dev, "no anatop-reg-offset property set\n");
		goto anatop_probe_end;
	}
	ret = of_property_read_u32(np, "anatop-vol-bit-width",
				   &sreg->vol_bit_width);
	if (ret) {
		dev_err(dev, "no anatop-vol-bit-width property set\n");
		goto anatop_probe_end;
	}
	ret = of_property_read_u32(np, "anatop-vol-bit-shift",
				   &sreg->vol_bit_shift);
	if (ret) {
		dev_err(dev, "no anatop-vol-bit-shift property set\n");
		goto anatop_probe_end;
	}
	ret = of_property_read_u32(np, "anatop-min-bit-val",
				   &sreg->min_bit_val);
	if (ret) {
		dev_err(dev, "no anatop-min-bit-val property set\n");
		goto anatop_probe_end;
	}
	ret = of_property_read_u32(np, "anatop-min-voltage",
				   &sreg->min_voltage);
	if (ret) {
		dev_err(dev, "no anatop-min-voltage property set\n");
		goto anatop_probe_end;
	}
	ret = of_property_read_u32(np, "anatop-max-voltage",
				   &sreg->max_voltage);
	if (ret) {
		dev_err(dev, "no anatop-max-voltage property set\n");
		goto anatop_probe_end;
	}

	/* read LDO ramp up setting, only for core reg */
	of_property_read_u32(np, "anatop-delay-reg-offset",
			     &sreg->delay_reg);
	of_property_read_u32(np, "anatop-delay-bit-width",
			     &sreg->delay_bit_width);
	of_property_read_u32(np, "anatop-delay-bit-shift",
			     &sreg->delay_bit_shift);
	/* Only non-core regulator (3p0, 2p5, and 1p1) has enable bit */
	of_property_read_u32(np, "anatop-enable-bit", &sreg->enable_bit);

	rdesc->n_voltages = (sreg->max_voltage - sreg->min_voltage) / 25000 + 1
			    + sreg->min_bit_val;
	rdesc->min_uV = sreg->min_voltage;
	rdesc->uV_step = 25000;
	rdesc->linear_min_sel = sreg->min_bit_val;
	rdesc->vsel_reg = sreg->control_reg;
	rdesc->vsel_mask = ((1 << sreg->vol_bit_width) - 1) <<
			   sreg->vol_bit_shift;

	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = sreg;
	config.of_node = pdev->dev.of_node;
	config.regmap = sreg->anatop;

	/* register regulator */
	rdev = regulator_register(rdesc, &config);
	if (IS_ERR(rdev)) {
		dev_err(dev, "failed to register %s\n",
			rdesc->name);
		ret = PTR_ERR(rdev);
		goto anatop_probe_end;
	}

	platform_set_drvdata(pdev, rdev);

anatop_probe_end:
	if (ret)
		kfree(sreg->name);

	return ret;
}

static int anatop_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	struct anatop_regulator *sreg = rdev_get_drvdata(rdev);
	const char *name = sreg->name;

	regulator_unregister(rdev);
	kfree(name);

	return 0;
}

static struct of_device_id of_anatop_regulator_match_tbl[] = {
	{ .compatible = "fsl,anatop-regulator", },
	{ /* end */ }
};

static struct platform_driver anatop_regulator_driver = {
	.driver = {
		.name	= "anatop_regulator",
		.owner  = THIS_MODULE,
		.of_match_table = of_anatop_regulator_match_tbl,
	},
	.probe	= anatop_regulator_probe,
	.remove	= anatop_regulator_remove,
};

static int __init anatop_regulator_init(void)
{
	return platform_driver_register(&anatop_regulator_driver);
}
postcore_initcall(anatop_regulator_init);

static void __exit anatop_regulator_exit(void)
{
	platform_driver_unregister(&anatop_regulator_driver);
}
module_exit(anatop_regulator_exit);

MODULE_AUTHOR("Nancy Chen <Nancy.Chen@freescale.com>, "
	      "Ying-Chun Liu (PaulLiu) <paul.liu@linaro.org>");
MODULE_DESCRIPTION("ANATOP Regulator driver");
MODULE_LICENSE("GPL v2");
