/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/max17135.h>
#include <linux/gpio.h>

/*
 * Regulator definitions
 *   *_MIN_uV  - minimum microvolt for regulator
 *   *_MAX_uV  - maximum microvolt for regulator
 *   *_STEP_uV - microvolts between regulator output levels
 *   *_MIN_VAL - minimum register field value for regulator
 *   *_MAX_VAL - maximum register field value for regulator
 */
#define MAX17135_HVINP_MIN_uV    5000000
#define MAX17135_HVINP_MAX_uV   20000000
#define MAX17135_HVINP_STEP_uV   1000000
#define MAX17135_HVINP_MIN_VAL         0
#define MAX17135_HVINP_MAX_VAL         1

#define MAX17135_HVINN_MIN_uV    5000000
#define MAX17135_HVINN_MAX_uV   20000000
#define MAX17135_HVINN_STEP_uV   1000000
#define MAX17135_HVINN_MIN_VAL         0
#define MAX17135_HVINN_MAX_VAL         1

#define MAX17135_GVDD_MIN_uV    5000000
#define MAX17135_GVDD_MAX_uV   20000000
#define MAX17135_GVDD_STEP_uV   1000000
#define MAX17135_GVDD_MIN_VAL         0
#define MAX17135_GVDD_MAX_VAL         1

#define MAX17135_GVEE_MIN_uV    5000000
#define MAX17135_GVEE_MAX_uV   20000000
#define MAX17135_GVEE_STEP_uV   1000000
#define MAX17135_GVEE_MIN_VAL         0
#define MAX17135_GVEE_MAX_VAL         1

#define MAX17135_VCOM_MIN_VAL         0
#define MAX17135_VCOM_MAX_VAL       255

#define MAX17135_VNEG_MIN_uV    5000000
#define MAX17135_VNEG_MAX_uV   20000000
#define MAX17135_VNEG_STEP_uV   1000000
#define MAX17135_VNEG_MIN_VAL         0
#define MAX17135_VNEG_MAX_VAL         1

#define MAX17135_VPOS_MIN_uV    5000000
#define MAX17135_VPOS_MAX_uV   20000000
#define MAX17135_VPOS_STEP_uV   1000000
#define MAX17135_VPOS_MIN_VAL         0
#define MAX17135_VPOS_MAX_VAL         1

struct max17135_vcom_programming_data {
	int vcom_min_uV;
	int vcom_max_uV;
	int vcom_step_uV;
};

static long unsigned int max17135_pass_num = { 1 };
static int max17135_vcom = { -1250000 };

struct max17135_vcom_programming_data vcom_data[2] = {
	{
		-4325000,
		-500000,
		15000,
	},
	{
		-3050000,
		-500000,
		10000,
	},
};

static int max17135_is_power_good(struct max17135 *max17135);

/*
 * Regulator operations
 */
static int max17135_hvinp_set_voltage(struct regulator_dev *reg,
					int minuV, int uV, unsigned *selector)
{
	unsigned int reg_val;
	unsigned int fld_val;

	if ((uV >= MAX17135_HVINP_MIN_uV) &&
	    (uV <= MAX17135_HVINP_MAX_uV))
		fld_val = (uV - MAX17135_HVINP_MIN_uV) /
			MAX17135_HVINP_STEP_uV;
	else
		return -EINVAL;

	max17135_reg_read(REG_MAX17135_HVINP, &reg_val);

	reg_val &= ~BITFMASK(HVINP);
	reg_val |= BITFVAL(HVINP, fld_val); /* shift to correct bit */

	return max17135_reg_write(REG_MAX17135_HVINP, reg_val);
}

static int max17135_hvinp_get_voltage(struct regulator_dev *reg)
{
	unsigned int reg_val;
	unsigned int fld_val;
	int volt;

	max17135_reg_read(REG_MAX17135_HVINP, &reg_val);

	fld_val = (reg_val & BITFMASK(HVINP)) >> HVINP_LSH;

	if ((fld_val >= MAX17135_HVINP_MIN_VAL) &&
		(fld_val <= MAX17135_HVINP_MAX_VAL)) {
		volt = (fld_val * MAX17135_HVINP_STEP_uV) +
			MAX17135_HVINP_MIN_uV;
	} else {
		printk(KERN_ERR "MAX17135: HVINP voltage is out of range\n");
		volt = 0;
	}
	return volt;
}

static int max17135_hvinp_enable(struct regulator_dev *reg)
{
	return 0;
}

static int max17135_hvinp_disable(struct regulator_dev *reg)
{
	return 0;
}

/* Convert uV to the VCOM register bitfield setting */
static inline int vcom_uV_to_rs(int uV, int pass_num)
{
	return (vcom_data[pass_num].vcom_max_uV - uV)
		/ vcom_data[pass_num].vcom_step_uV;
}

/* Convert the VCOM register bitfield setting to uV */
static inline int vcom_rs_to_uV(int rs, int pass_num)
{
	return vcom_data[pass_num].vcom_max_uV
		- (vcom_data[pass_num].vcom_step_uV * rs);
}

static int max17135_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV, unsigned *selector)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	unsigned int reg_val;
	int vcom_read;

	if ((uV < vcom_data[max17135->pass_num-1].vcom_min_uV)
		|| (uV > vcom_data[max17135->pass_num-1].vcom_max_uV))
		return -EINVAL;

	max17135_reg_read(REG_MAX17135_DVR, &reg_val);

	/*
	 * Only program VCOM if it is not set to the desired value.
	 * Programming VCOM excessively degrades ability to keep
	 * DVR register value persistent.
	 */
	vcom_read = vcom_rs_to_uV(reg_val, max17135->pass_num-1);
	if (vcom_read != max17135->vcom_uV) {
		reg_val &= ~BITFMASK(DVR);
		reg_val |= BITFVAL(DVR, vcom_uV_to_rs(uV,
			max17135->pass_num-1));
		max17135_reg_write(REG_MAX17135_DVR, reg_val);

		reg_val = BITFVAL(CTRL_DVR, true); /* shift to correct bit */
		return max17135_reg_write(REG_MAX17135_PRGM_CTRL, reg_val);
	}

	return 0;
}

static int max17135_vcom_get_voltage(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	unsigned int reg_val;

	max17135_reg_read(REG_MAX17135_DVR, &reg_val);
	return vcom_rs_to_uV(BITFEXT(reg_val, DVR), max17135->pass_num-1);
}

static int max17135_vcom_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/*
	 * Check to see if we need to set the VCOM voltage.
	 * Should only be done one time. And, we can
	 * only change vcom voltage if we have been enabled.
	 */
	if (!max17135->vcom_setup && max17135_is_power_good(max17135)) {
		max17135_vcom_set_voltage(reg,
			max17135->vcom_uV,
			max17135->vcom_uV,
			NULL);
		max17135->vcom_setup = true;
	}

	/* enable VCOM regulator output */
	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 1);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(VCOM_ENABLE);
		reg_val |= BITFVAL(VCOM_ENABLE, 1); /* shift to correct bit */
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	return 0;
}

static int max17135_vcom_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 0);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(VCOM_ENABLE);
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	return 0;
}

static int max17135_vcom_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/* read VCOM regulator enable setting */
	if (max17135->pass_num == 1) {
		int gpio = gpio_get_value(max17135->gpio_pmic_vcom_ctrl);
		if (gpio == 0)
			return 0;
		else
			return 1;
	} else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= BITFMASK(VCOM_ENABLE);
		if (reg_val != 0)
			return 1;
		else
			return 0;
	}
}

static int max17135_is_power_good(struct max17135 *max17135)
{
    unsigned int reg_val;
    unsigned int fld_val;

    max17135_reg_read(REG_MAX17135_FAULT, &reg_val);
    fld_val = (reg_val & BITFMASK(FAULT_POK)) >> FAULT_POK_LSH;

    /* Check the POK bit */
    return fld_val;
}

static int max17135_wait_power_good(struct max17135 *max17135)
{
	int i;

	for (i = 0; i < max17135->max_wait * 3; i++) {
		if (max17135_is_power_good(max17135))
			return 0;

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int max17135_display_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/* The Pass 1 parts cannot turn on the PMIC via I2C. */
	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_wakeup, 1);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(ENABLE);
		reg_val |= BITFVAL(ENABLE, 1);
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	return max17135_wait_power_good(max17135);
}

static int max17135_display_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	if (max17135->pass_num == 1)
		gpio_set_value(max17135->gpio_pmic_wakeup, 0);
	else {
		unsigned int reg_val;

		max17135_reg_read(REG_MAX17135_ENABLE, &reg_val);
		reg_val &= ~BITFMASK(ENABLE);
		max17135_reg_write(REG_MAX17135_ENABLE, reg_val);
	}

	msleep(max17135->max_wait);

	return 0;
}

static int max17135_display_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(max17135->gpio_pmic_wakeup);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int max17135_v3p3_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	gpio_set_value(max17135->gpio_pmic_v3p3, 1);
	return 0;
}

static int max17135_v3p3_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	gpio_set_value(max17135->gpio_pmic_v3p3, 0);
	return 0;
}

static int max17135_v3p3_is_enabled(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(max17135->gpio_pmic_v3p3);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

/*
 * Regulator operations
 */

static struct regulator_ops max17135_display_ops = {
	.enable = max17135_display_enable,
	.disable = max17135_display_disable,
	.is_enabled = max17135_display_is_enabled,
};

static struct regulator_ops max17135_gvdd_ops = {
};

static struct regulator_ops max17135_gvee_ops = {
};

static struct regulator_ops max17135_hvinn_ops = {
};

static struct regulator_ops max17135_hvinp_ops = {
	.enable = max17135_hvinp_enable,
	.disable = max17135_hvinp_disable,
	.get_voltage = max17135_hvinp_get_voltage,
	.set_voltage = max17135_hvinp_set_voltage,
};

static struct regulator_ops max17135_vcom_ops = {
	.enable = max17135_vcom_enable,
	.disable = max17135_vcom_disable,
	.get_voltage = max17135_vcom_get_voltage,
	.set_voltage = max17135_vcom_set_voltage,
	.is_enabled = max17135_vcom_is_enabled,
};

static struct regulator_ops max17135_vneg_ops = {
};

static struct regulator_ops max17135_vpos_ops = {
};

static struct regulator_ops max17135_v3p3_ops = {
	.enable = max17135_v3p3_enable,
	.disable = max17135_v3p3_disable,
	.is_enabled = max17135_v3p3_is_enabled,
};


/*
 * Regulator descriptors
 */
static struct regulator_desc max17135_reg[MAX17135_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = MAX17135_DISPLAY,
	.ops = &max17135_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVDD",
	.id = MAX17135_GVDD,
	.ops = &max17135_gvdd_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "GVEE",
	.id = MAX17135_GVEE,
	.ops = &max17135_gvee_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "HVINN",
	.id = MAX17135_HVINN,
	.ops = &max17135_hvinn_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "HVINP",
	.id = MAX17135_HVINP,
	.ops = &max17135_hvinp_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = MAX17135_VCOM,
	.ops = &max17135_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VNEG",
	.id = MAX17135_VNEG,
	.ops = &max17135_vneg_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VPOS",
	.id = MAX17135_VPOS,
	.ops = &max17135_vpos_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "V3P3",
	.id = MAX17135_V3P3,
	.ops = &max17135_v3p3_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

static void max17135_setup_timings(struct max17135 *max17135)
{
	unsigned int reg_val;

	int timing1, timing2, timing3, timing4,
		timing5, timing6, timing7, timing8;

	max17135_reg_read(REG_MAX17135_TIMING1, &timing1);
	max17135_reg_read(REG_MAX17135_TIMING2, &timing2);
	max17135_reg_read(REG_MAX17135_TIMING3, &timing3);
	max17135_reg_read(REG_MAX17135_TIMING4, &timing4);
	max17135_reg_read(REG_MAX17135_TIMING5, &timing5);
	max17135_reg_read(REG_MAX17135_TIMING6, &timing6);
	max17135_reg_read(REG_MAX17135_TIMING7, &timing7);
	max17135_reg_read(REG_MAX17135_TIMING8, &timing8);

	if ((timing1 != max17135->gvee_pwrup) ||
		(timing2 != max17135->vneg_pwrup) ||
		(timing3 != max17135->vpos_pwrup) ||
		(timing4 != max17135->gvdd_pwrup) ||
		(timing5 != max17135->gvdd_pwrdn) ||
		(timing6 != max17135->vpos_pwrdn) ||
		(timing7 != max17135->vneg_pwrdn) ||
		(timing8 != max17135->gvee_pwrdn)) {
		max17135_reg_write(REG_MAX17135_TIMING1, max17135->gvee_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING2, max17135->vneg_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING3, max17135->vpos_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING4, max17135->gvdd_pwrup);
		max17135_reg_write(REG_MAX17135_TIMING5, max17135->gvdd_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING6, max17135->vpos_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING7, max17135->vneg_pwrdn);
		max17135_reg_write(REG_MAX17135_TIMING8, max17135->gvee_pwrdn);

		reg_val = BITFVAL(CTRL_TIMING, true); /* shift to correct bit */
		max17135_reg_write(REG_MAX17135_PRGM_CTRL, reg_val);
	}
}


/*
 * Regulator init/probing/exit functions
 */
static int max17135_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	rdev = regulator_register(&max17135_reg[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));

	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			max17135_reg[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int max17135_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver max17135_regulator_driver = {
	.probe = max17135_regulator_probe,
	.remove = max17135_regulator_remove,
	.driver = {
		.name = "max17135-reg",
	},
};

int max17135_register_regulator(struct max17135 *max17135, int reg,
				     struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	struct i2c_client *client = max17135->i2c_client;
	/* If we can't find PMIC via I2C, we should not register regulators */
	if (i2c_smbus_read_byte_data(client,
		REG_MAX17135_PRODUCT_REV) != 0) {
		dev_err(max17135->dev,
			"Max17135 PMIC not found!\n");
		return -ENXIO;
	}

	if (max17135->pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("max17135-reg", reg);
	if (!pdev)
		return -ENOMEM;

	max17135->pdev[reg] = pdev;

	initdata->driver_data = max17135;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = max17135->dev;
	platform_set_drvdata(pdev, max17135);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(max17135->dev,
		       "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		max17135->pdev[reg] = NULL;
	}

	if (!max17135->init_done) {
		max17135->pass_num = max17135_pass_num;
		max17135->vcom_uV = max17135_vcom;

		/*
		 * Set up PMIC timing values.
		 * Should only be done one time!  Timing values may only be
		 * changed a limited number of times according to spec.
		 */
		max17135_setup_timings(max17135);

		max17135->init_done = true;
	}

	return ret;
}

static int __init max17135_regulator_init(void)
{
	return platform_driver_register(&max17135_regulator_driver);
}
subsys_initcall(max17135_regulator_init);

static void __exit max17135_regulator_exit(void)
{
	platform_driver_unregister(&max17135_regulator_driver);
}
module_exit(max17135_regulator_exit);


/*
 * Parse user specified options (`max17135:')
 * example:
 *   max17135:pass=2,vcom=-1250000
 */
static int __init max17135_setup(char *options)
{
	int ret;
	char *opt;
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "pass=", 5)) {
			ret = strict_strtoul(opt + 5, 0, &max17135_pass_num);
			if (ret < 0)
				return ret;
		}
		if (!strncmp(opt, "vcom=", 5)) {
			int offs = 5;
			if (opt[5] == '-')
				offs = 6;
			ret = strict_strtoul(opt + offs, 0,
				(long *)&max17135_vcom);
			if (ret < 0)
				return ret;
			max17135_vcom = -max17135_vcom;
		}
	}

	return 1;
}

__setup("max17135:", max17135_setup);

/* Module information */
MODULE_DESCRIPTION("MAX17135 regulator driver");
MODULE_LICENSE("GPL");
