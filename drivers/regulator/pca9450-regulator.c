/*
 * @file pca9450-regulator.c ROHM PCA9450MWV regulator driver
 *
 * Copyright 2019 NXP.
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/pca9450.h>
#include <linux/regulator/of_regulator.h>

#define PCA9450_DVS_BUCK_NUM	3 /* Buck 1/2/3 support DVS */
#define PCA9450_DVS0_1		2
#define PCA9450_DVS0		1

struct pca9450_buck_dvs {
	u32 voltage[PCA9450_DVS0_1];
};

/* @brief pca9450 regulator type */
struct pca9450_pmic {
	struct regulator_desc descs[PCA9450_REGULATOR_CNT];
	struct pca9450 *mfd;
	struct device *dev;
	struct regulator_dev *rdev[PCA9450_REGULATOR_CNT];
	struct pca9450_buck_dvs buck_dvs[PCA9450_DVS_BUCK_NUM];
	int	reg_index;
};

/*
 * BUCK1/2/3
 * BUCK1RAM[1:0] BUCK1 DVS ramp rate setting
 * 00: 25mV/1usec
 * 01: 25mV/2usec
 * 10: 25mV/4usec
 * 11: 25mV/8usec
 */
static int pca9450_buck123_set_ramp_delay(struct regulator_dev *rdev,
					  int ramp_delay)
{
	struct pca9450_pmic *pmic = rdev_get_drvdata(rdev);
	struct pca9450 *mfd = pmic->mfd;
	int id = rdev->desc->id;
	unsigned int ramp_value = BUCK1_RAMP_3P125MV;
	unsigned int buckctrl[3] = {PCA9450_BUCK1CTRL, PCA9450_BUCK2CTRL,
				    PCA9450_BUCK3CTRL};

	dev_dbg(pmic->dev, "Buck[%d] Set Ramp = %d\n", id + 1, ramp_delay);
	switch (ramp_delay) {
	case 1 ... 3125:
		ramp_value = BUCK1_RAMP_3P125MV;
		break;
	case 3126 ... 6250:
		ramp_value = BUCK1_RAMP_6P25MV;
		break;
	case 6251 ... 12500:
		ramp_value = BUCK1_RAMP_12P5MV;
		break;
	case 12501 ... 25000:
		ramp_value = BUCK1_RAMP_25MV;
		break;
	default:
		ramp_value = BUCK1_RAMP_25MV;
	}

	return regmap_update_bits(mfd->regmap, buckctrl[id],
			BUCK1_RAMP_MASK, ramp_value << 6);
}

static struct regulator_ops pca9450_ldo_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static struct regulator_ops pca9450_fixed_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
};

static struct regulator_ops pca9450_buck_regulator_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static struct regulator_ops pca9450_buck123_regulator_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = pca9450_buck123_set_ramp_delay,
};

/*
 * BUCK1/2/3
 * 0.60 to 2.1875V (12.5mV step)
 */
static const struct regulator_linear_range pca9450_buck123_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000,  0x00, 0x7F, 12500),
};

/*
 * BUCK4/5/6
 * 0.6V to 3.4V (25mV step)
 */
static const struct regulator_linear_range pca9450_buck456_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(600000, 0x00, 0x70, 25000),
	REGULATOR_LINEAR_RANGE(3400000, 0x71, 0x7F, 0),
};

/*
 * LDO1
 * 1.6 to 3.3V ()
 */
static const struct regulator_linear_range pca9450_ldo1_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1600000, 0x00, 0x03, 100000),
	REGULATOR_LINEAR_RANGE(3000000, 0x04, 0x07, 100000),
};

/*
 * LDO2
 * 0.8 to 1.15V (50mV step)
 */
static const struct regulator_linear_range pca9450_ldo2_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x07, 50000),
};

/*
 * LDO3
 * 0.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range pca9450_ldo34_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x19, 100000),
	REGULATOR_LINEAR_RANGE(3300000, 0x1A, 0x1F, 0),
};

/*
 * LDO5
 * 1.8 to 3.3V (100mV step)
 */
static const struct regulator_linear_range pca9450_ldo5_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(1800000,  0x00, 0x0F, 100000),
};

static const struct regulator_desc pca9450_regulators[] = {
	{
		.name = "BUCK1",
		.id = PCA9450_BUCK1,
		.ops = &pca9450_buck123_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK1_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck123_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck123_voltage_ranges),
		.vsel_reg = PCA9450_BUCK1OUT_DVS0,
		.vsel_mask = BUCK1OUT_DVS0_MASK,
		.enable_reg = PCA9450_BUCK1CTRL,
		.enable_mask = BUCK1_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK2",
		.id = PCA9450_BUCK2,
		.ops = &pca9450_buck123_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK2_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck123_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck123_voltage_ranges),
		.vsel_reg = PCA9450_BUCK2OUT_DVS0,
		.vsel_mask = BUCK2OUT_DVS0_MASK,
		.enable_reg = PCA9450_BUCK2CTRL,
		.enable_mask = BUCK2_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK3",
		.id = PCA9450_BUCK3,
		.ops = &pca9450_buck123_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK3_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck123_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck123_voltage_ranges),
		.vsel_reg = PCA9450_BUCK3OUT_DVS0,
		.vsel_mask = BUCK3OUT_DVS0_MASK,
		.enable_reg = PCA9450_BUCK3CTRL,
		.enable_mask = BUCK3_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK4",
		.id = PCA9450_BUCK4,
		.ops = &pca9450_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK4_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck456_voltage_ranges),
		.vsel_reg = PCA9450_BUCK4OUT,
		.vsel_mask = BUCK4OUT_MASK,
		.enable_reg = PCA9450_BUCK4CTRL,
		.enable_mask = BUCK4_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK5",
		.id = PCA9450_BUCK5,
		.ops = &pca9450_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK5_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck456_voltage_ranges),
		.vsel_reg = PCA9450_BUCK5OUT,
		.vsel_mask = BUCK5OUT_MASK,
		.enable_reg = PCA9450_BUCK5CTRL,
		.enable_mask = BUCK5_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "BUCK6",
		.id = PCA9450_BUCK6,
		.ops = &pca9450_buck_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_BUCK6_VOLTAGE_NUM,
		.linear_ranges = pca9450_buck456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_buck456_voltage_ranges),
		.vsel_reg = PCA9450_BUCK6OUT,
		.vsel_mask = BUCK6OUT_MASK,
		.enable_reg = PCA9450_BUCK6CTRL,
		.enable_mask = BUCK6_ENMODE_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO1",
		.id = PCA9450_LDO1,
		.ops = &pca9450_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_LDO1_VOLTAGE_NUM,
		.linear_ranges = pca9450_ldo1_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_ldo1_voltage_ranges),
		.vsel_reg = PCA9450_LDO1CTRL,
		.vsel_mask = LDO1OUT_MASK,
		.enable_reg = PCA9450_LDO1CTRL,
		.enable_mask = LDO1_EN_MASK,
		.owner = THIS_MODULE,
	},
	/*
	 * LDO2 0.9V
	 * Fixed voltage
	 */
	{
		.name = "LDO2",
		.id = PCA9450_LDO2,
		.ops = &pca9450_fixed_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_LDO2_VOLTAGE_NUM,
		.min_uV = 900000,
		.enable_reg = PCA9450_LDO2CTRL,
		.enable_mask = LDO2_EN_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO3",
		.id = PCA9450_LDO3,
		.ops = &pca9450_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_LDO3_VOLTAGE_NUM,
		.linear_ranges = pca9450_ldo34_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_ldo34_voltage_ranges),
		.vsel_reg = PCA9450_LDO3CTRL,
		.vsel_mask = LDO3OUT_MASK,
		.enable_reg = PCA9450_LDO3CTRL,
		.enable_mask = LDO3_EN_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO4",
		.id = PCA9450_LDO4,
		.ops = &pca9450_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_LDO4_VOLTAGE_NUM,
		.linear_ranges = pca9450_ldo34_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_ldo34_voltage_ranges),
		.vsel_reg = PCA9450_LDO4CTRL,
		.vsel_mask = LDO4OUT_MASK,
		.enable_reg = PCA9450_LDO4CTRL,
		.enable_mask = LDO4_EN_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "LDO5",
		.id = PCA9450_LDO5,
		.ops = &pca9450_ldo_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.n_voltages = PCA9450_LDO5_VOLTAGE_NUM,
		.linear_ranges = pca9450_ldo5_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pca9450_ldo5_voltage_ranges),
		.vsel_reg = PCA9450_LDO5CTRL_H,
		.vsel_mask = LDO5HOUT_MASK,
		.enable_reg = PCA9450_LDO5CTRL_H,
		.enable_mask = LDO5H_EN_MASK,
		.owner = THIS_MODULE,
	},
};

#ifdef CONFIG_OF

static struct of_regulator_match pca9450_matches[] = {
	{ .name = "buck1",	},
	{ .name = "buck2",	},
	{ .name = "buck3",	},
	{ .name = "buck4",	},
	{ .name = "buck5",	},
	{ .name = "buck6",	},
	{ .name = "ldo1",	},
	{ .name = "ldo2",	},
	{ .name = "ldo3",	},
	{ .name = "ldo4",	},
	{ .name = "ldo5",	},
};

/*
 * @brief parse pca9450 regulator device tree
 * @param pdev platform device of pca9450 regulator
 * @param pca9450_reg_matches return regualtor matches
 * @retval 0 parse success
 * @retval NULL parse fail
 */
static int pca9450_parse_dt_reg_data(
		struct platform_device *pdev,
		struct of_regulator_match **reg_matches)
{
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found\n");
		return -EINVAL;
	}

	count = ARRAY_SIZE(pca9450_matches);
	matches = pca9450_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static inline int pca9450_parse_dt_reg_data(
			struct platform_device *pdev,
			struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

/* @brief directly set raw value to chip register, format: 'register value' */
static ssize_t pca9450_sysfs_set_registers(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct pca9450_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret < 1) {
		pmic->reg_index = -1;
		dev_err(pmic->dev, "registers set: <reg> <value>\n");
		return count;
	}

	if (ret == 1 && reg < PCA9450_MAX_REGISTER) {
		pmic->reg_index = reg;
		dev_info(pmic->dev, "registers set: reg=0x%x\n", reg);
		return count;
	}

	if (reg >= PCA9450_MAX_REGISTER) {
		dev_err(pmic->dev, "reg=%d out of Max=%d\n", reg,
			PCA9450_MAX_REGISTER);
		return -EINVAL;
	}
	dev_info(pmic->dev, "registers set: reg=0x%x, val=0x%x\n", reg, val);
	ret = pca9450_reg_write(pmic->mfd, reg, val);
	if (ret < 0)
		return ret;
	return count;
}

/* @brief print value of chip register, format: 'register=value' */
static ssize_t pca9450_sysfs_print_reg(struct pca9450_pmic *pmic,
				       u8 reg,
				       char *buf)
{
	int ret = pca9450_reg_read(pmic->mfd, reg);

	if (ret < 0)
		return sprintf(buf, "%#.2x=error %d\n", reg, ret);
	return sprintf(buf, "[0x%.2X] = %.2X\n", reg, ret);
}

/*
 * @brief show all raw values of chip register, format per line:
 * 'register=value'
 */
static ssize_t pca9450_sysfs_show_registers(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct pca9450_pmic *pmic = dev_get_drvdata(dev);
	ssize_t ret = 0;
	int i;

	dev_info(pmic->dev, "register: index[0x%x]\n", pmic->reg_index);
	if (pmic->reg_index >= 0)
		ret += pca9450_sysfs_print_reg(pmic, pmic->reg_index,
					       buf + ret);
	else
		for (i = 0; i < PCA9450_MAX_REGISTER; i++)
			ret += pca9450_sysfs_print_reg(pmic, i, buf + ret);

	return ret;
}

static DEVICE_ATTR(registers, 0644,
		pca9450_sysfs_show_registers, pca9450_sysfs_set_registers);

/* @brief device sysfs attribute table, about o */
static struct attribute *clk_attributes[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group clk_attr_group = {
	.attrs	= clk_attributes,
};

/*----------------------------------------------------------------------*/
#ifdef CONFIG_OF
/*
 * @brief buck1/2 dvs enable/voltage from device tree
 * @param pdev platform device pointer
 * @param buck_dvs pointer
 * @return void
 */
static void of_pca9450_buck_dvs(struct platform_device *pdev,
				struct pca9450_buck_dvs *buck_dvs)
{
	struct device_node *pmic_np;

	pmic_np = of_node_get(pdev->dev.parent->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return;
	}
	if (of_get_property(pmic_np, "pca9450,pmic-buck1-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
					       "pca9450,pmic-buck1-dvs-voltage",
					       &buck_dvs[0].voltage[0],
					       PCA9450_DVS0_1)) {
			dev_err(&pdev->dev, "buck1 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "pca9450,pmic-buck2-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
					       "pca9450,pmic-buck2-dvs-voltage",
						&buck_dvs[1].voltage[0],
						PCA9450_DVS0_1)) {
			dev_err(&pdev->dev, "buck2 voltages not specified\n");
		}
	}

	if (of_get_property(pmic_np, "pca9450,pmic-buck3-uses-i2c-dvs", NULL)) {
		if (of_property_read_u32_array(pmic_np,
					       "pca9450,pmic-buck3-dvs-voltage",
					       &buck_dvs[2].voltage[0],
					       PCA9450_DVS0)) {
			dev_err(&pdev->dev, "buck3 voltages not specified\n");
		}
	}
}
#else
static void of_pca9450_buck_dvs(struct platform_device *pdev,
				struct pca9450_buck_dvs *buck_dvs)
{
	buck_dvs[0].voltage[0] = BUCK1OUT_DVS0_DEFAULT;
	buck_dvs[0].voltage[1] = BUCK1OUT_DVS1_DEFAULT;
	buck_dvs[1].voltage[0] = BUCK2OUT_DVS0_DEFAULT;
	buck_dvs[1].voltage[1] = BUCK2OUT_DVS1_DEFAULT;
	buck_dvs[2].voltage[0] = BUCK3OUT_DVS0_DEFAULT;
	buck_dvs[2].voltage[1] = 0; /* Not supported */
}
#endif

static int pca9450_buck123_dvs_init(struct pca9450_pmic *pmic)
{
	struct pca9450 *pca9450 = pmic->mfd;
	struct pca9450_buck_dvs *buck_dvs = &pmic->buck_dvs[0];
	int i, ret, val, selector = 0;
	u8 reg_dvs0, reg_dvs1;
	u8 reg_dvs0_msk, reg_dvs1_msk;

	for (i = 0; i < PCA9450_DVS_BUCK_NUM; i++, buck_dvs++) {
		switch (i) {
		case 0:
		default:
			reg_dvs0 = PCA9450_BUCK1OUT_DVS0;
			reg_dvs0_msk = BUCK1OUT_DVS0_MASK;
			reg_dvs1 = PCA9450_BUCK1OUT_DVS1;
			reg_dvs1_msk = BUCK1OUT_DVS1_MASK;
			break;
		case 1:
			reg_dvs0 = PCA9450_BUCK2OUT_DVS0;
			reg_dvs0_msk = BUCK2OUT_DVS0_MASK;
			reg_dvs1 = PCA9450_BUCK2OUT_DVS1;
			reg_dvs1_msk = BUCK2OUT_DVS1_MASK;
			break;
		case 2:
			reg_dvs0 = PCA9450_BUCK3OUT_DVS0;
			reg_dvs0_msk = BUCK3OUT_DVS0_MASK;
			reg_dvs1 = PCA9450_BUCK3OUT_DVS1;
			reg_dvs1_msk = BUCK3OUT_DVS1_MASK;
			break;
		}

		dev_dbg(pmic->dev, "Buck%d: DVS0-DVS1[%d - %d].\n", i+1,
			buck_dvs->voltage[0], buck_dvs->voltage[1]);
		if (reg_dvs0 > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i],
								 buck_dvs->voltage[0],
								 buck_dvs->voltage[0]);
			if (selector < 0) {
				dev_dbg(pmic->dev,
					"not found selector for DVS0 [%d]\n",
					buck_dvs->voltage[0]);
			} else {
				val = (selector & reg_dvs0_msk);
				ret = pca9450_reg_write(pca9450, reg_dvs0, val);
				if (ret < 0)
					return ret;
			}
		}
		if (reg_dvs1 > 0) {
			selector = regulator_map_voltage_iterate(pmic->rdev[i],
								 buck_dvs->voltage[1],
								 buck_dvs->voltage[1]);
			if (selector < 0) {
				dev_dbg(pmic->dev,
					"not found selector for DVS1 [%d]\n",
					buck_dvs->voltage[1]);
			} else {
				val = (selector & reg_dvs1_msk);
				ret = pca9450_reg_write(pca9450, reg_dvs1, val);
				if (ret < 0)
					return ret;
			}
		}
	}
	return 0;
}

/*
 * @brief pca9450 pmic interrupt
 * @param irq system irq
 * @param pwrsys pca9450 power device of system
 * @retval IRQ_HANDLED success
 * @retval IRQ_NONE error
 */
static irqreturn_t pca9450_pmic_interrupt(int irq, void *pwrsys)
{
	struct device *dev = pwrsys;
	struct pca9450 *mfd = dev_get_drvdata(dev->parent);
	int reg;

	reg = pca9450_reg_read(mfd, PCA9450_INT1);
	if (reg < 0)
		return IRQ_NONE;

	if (reg & IRQ_PWRON)
		dev_dbg(dev, "IRQ_PWRON\n");
	if (reg & IRQ_WDOGB)
		dev_dbg(dev, "IRQ_WDOGB\n");
	if (reg & IRQ_VR_FLT1)
		dev_dbg(dev, "IRQ_VR_FLT1\n");
	if (reg & IRQ_VR_FLT2)
		dev_dbg(dev, "IRQ_VR_FLT2\n");
	if (reg & IRQ_LOWVSYS)
		dev_dbg(dev, "IRQ_LOWVSYS\n");
	if (reg & IRQ_THERM_105)
		dev_dbg(dev, "IRQ_THERM_105\n");
	if (reg & IRQ_THERM_125)
		dev_dbg(dev, "IRQ_THERM_125\n");

	return IRQ_HANDLED;
}

/*
 * @brief probe pca9450 regulator device
 * @param pdev pca9450 regulator platform device
 * @retval 0 success
 * @retval negative fail
 */
static int pca9450_probe(struct platform_device *pdev)
{
	struct pca9450_pmic *pmic;
	struct pca9450_board *pdata;
	struct regulator_config config = {};
	struct pca9450 *pca9450 = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *matches = NULL;
	int i = 0, err, irq = 0, ret = 0;

	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	memcpy(pmic->descs, pca9450_regulators,	sizeof(pmic->descs));

	pmic->dev = &pdev->dev;
	pmic->mfd = pca9450;
	platform_set_drvdata(pdev, pmic);
	pdata = dev_get_platdata(pca9450->dev);

	if (!pdata && pca9450->dev->of_node) {
		pca9450_parse_dt_reg_data(pdev,	&matches);
		if (matches == NULL) {
			dev_err(&pdev->dev, "Platform data not found\n");
			return -EINVAL;
		}
	}

	/* Get buck dvs parameters */
	of_pca9450_buck_dvs(pdev, &pmic->buck_dvs[0]);

	for (i = 0; i < PCA9450_REGULATOR_CNT; i++) {
		struct regulator_init_data *init_data;
		struct regulator_desc *desc;
		struct regulator_dev *rdev;

		desc = &pmic->descs[i];
		desc->name = pca9450_matches[i].name;

		if (pdata)
			init_data = pdata->init_data[i];
		else
			init_data = pca9450_matches[i].init_data;

		config.dev = pmic->dev;
		config.init_data = init_data;
		config.driver_data = pmic;
		config.regmap = pca9450->regmap;
		if (matches)
			config.of_node = matches[i].of_node;
		dev_dbg(config.dev, "regulator register name '%s'\n",
			desc->name);

		rdev = regulator_register(desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(pca9450->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(rdev);
			goto err;
		}
		pmic->rdev[i] = rdev;
	}

	/* Init sysfs registers */
	pmic->reg_index = -1;

	err = sysfs_create_group(&pdev->dev.kobj, &clk_attr_group);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to create sysfs: %d\n", err);
		goto err;
	}

	/* Init Buck1/2/3 dvs */
	err = pca9450_buck123_dvs_init(pmic);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to buck123 dvs: %d\n", err);
		goto err;
	}

	/* Add Interrupt */
	irq  = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_warn(&pdev->dev, "platform irq error # %d\n", irq);
		return -ENXIO;
	}
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					pca9450_pmic_interrupt,
					IRQF_TRIGGER_LOW | IRQF_EARLY_RESUME,
					dev_name(&pdev->dev), &pdev->dev);
	if (ret < 0)
		dev_err(&pdev->dev, "IRQ %d is not free.\n", irq);

	/* Un-mask IRQ Interrupt */
	ret = pca9450_reg_write(pca9450, PCA9450_INT1_MSK, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Write 'PCA9450_REG_MIRQ': failed!\n");
		ret = -EIO;
		goto err;
	}

	return 0;

err:
	while (--i >= 0)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return err;
}

/*
 * @brief remove pca9450 regulator device
 * @param pdev pca9450 regulator platform device
 * @return 0
 */
static int __exit pca9450_remove(struct platform_device *pdev)
{
	struct pca9450_pmic *pmic = platform_get_drvdata(pdev);
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &clk_attr_group);

	for (i = 0; i < PCA9450_REGULATOR_CNT; i++)
		regulator_unregister(pmic->rdev[i]);

	kfree(pmic);
	return 0;
}

static struct platform_driver pca9450_driver = {
	.driver = {
		.name = "pca9450-pmic",
		.owner = THIS_MODULE,
	},
	.probe = pca9450_probe,
	.remove = pca9450_remove,
};
module_platform_driver(pca9450_driver);

MODULE_DESCRIPTION("PCA9450 voltage regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pca9450-pmic");
