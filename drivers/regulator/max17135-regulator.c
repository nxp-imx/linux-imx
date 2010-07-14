/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/regulator/max17135.h>
#include <linux/gpio.h>

/*
 * Define this as 1 when using a Rev 1 MAX17135 part.  These parts have
 * some limitations, including an inability to turn on the PMIC via I2C.
 */
#define MAX17135_REV 1

/*
 * PMIC Register Addresses
 */
enum {
    REG_MAX17135_EXT_TEMP = 0x0,
    REG_MAX17135_CONFIG,
    REG_MAX17135_INT_TEMP = 0x4,
    REG_MAX17135_STATUS,
    REG_MAX17135_PRODUCT_REV,
    REG_MAX17135_PRODUCT_ID,
    REG_MAX17135_DVR,
    REG_MAX17135_ENABLE,
    REG_MAX17135_FAULT,  /*0x0A*/
    REG_MAX17135_HVINP,
    REG_MAX17135_PRGM_CTRL,
    REG_MAX17135_TIMING1 = 0x10,    /* Timing regs base address is 0x10 */
    REG_MAX17135_TIMING2,
    REG_MAX17135_TIMING3,
    REG_MAX17135_TIMING4,
    REG_MAX17135_TIMING5,
    REG_MAX17135_TIMING6,
    REG_MAX17135_TIMING7,
    REG_MAX17135_TIMING8,
};
#define MAX17135_REG_NUM        21
#define MAX17135_MAX_REGISTER   0xFF

/*
 * Bitfield macros that use rely on bitfield width/shift information.
 */
#define BITFMASK(field) (((1U << (field ## _WID)) - 1) << (field ## _LSH))
#define BITFVAL(field, val) ((val) << (field ## _LSH))
#define BITFEXT(var, bit) ((var & BITFMASK(bit)) >> (bit ## _LSH))

/*
 * Shift and width values for each register bitfield
 */
#define EXT_TEMP_LSH    7
#define EXT_TEMP_WID    9

#define THERMAL_SHUTDOWN_LSH    0
#define THERMAL_SHUTDOWN_WID    1

#define INT_TEMP_LSH    7
#define INT_TEMP_WID    9

#define STAT_BUSY_LSH   0
#define STAT_BUSY_WID   1
#define STAT_OPEN_LSH   1
#define STAT_OPEN_WID   1
#define STAT_SHRT_LSH   2
#define STAT_SHRT_WID   1

#define PROD_REV_LSH    0
#define PROD_REV_WID    8

#define PROD_ID_LSH     0
#define PROD_ID_WID     8

#define DVR_LSH         0
#define DVR_WID         8

#define ENABLE_LSH      0
#define ENABLE_WID      1
#define VCOM_ENABLE_LSH 1
#define VCOM_ENABLE_WID 1

#define FAULT_FBPG_LSH      0
#define FAULT_FBPG_WID      1
#define FAULT_HVINP_LSH     1
#define FAULT_HVINP_WID     1
#define FAULT_HVINN_LSH     2
#define FAULT_HVINN_WID     1
#define FAULT_FBNG_LSH      3
#define FAULT_FBNG_WID      1
#define FAULT_HVINPSC_LSH   4
#define FAULT_HVINPSC_WID   1
#define FAULT_HVINNSC_LSH   5
#define FAULT_HVINNSC_WID   1
#define FAULT_OT_LSH        6
#define FAULT_OT_WID        1
#define FAULT_POK_LSH       7
#define FAULT_POK_WID       1

#define HVINP_LSH           0
#define HVINP_WID           4

#define CTRL_DVR_LSH        0
#define CTRL_DVR_WID        1
#define CTRL_TIMING_LSH     1
#define CTRL_TIMING_WID     1

#define TIMING1_LSH         0
#define TIMING1_WID         8
#define TIMING2_LSH         0
#define TIMING2_WID         8
#define TIMING3_LSH         0
#define TIMING3_WID         8
#define TIMING4_LSH         0
#define TIMING4_WID         8
#define TIMING5_LSH         0
#define TIMING5_WID         8
#define TIMING6_LSH         0
#define TIMING6_WID         8
#define TIMING7_LSH         0
#define TIMING7_WID         8
#define TIMING8_LSH         0
#define TIMING8_WID         8

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

#if (MAX17135_REV == 1)
#define MAX17135_VCOM_MIN_uV   -4325000
#define MAX17135_VCOM_MAX_uV    -500000
#define MAX17135_VCOM_STEP_uV     15000
#define MAX17135_VCOM_MIN_VAL         0
#define MAX17135_VCOM_MAX_VAL       255
/* Required due to discrepancy between
 * observed VCOM programming and
 * what is suggested in the spec.
 */
#define MAX17135_VCOM_FUDGE_FACTOR 330000
#else
#define MAX17135_VCOM_MIN_uV   -3050000
#define MAX17135_VCOM_MAX_uV    -500000
#define MAX17135_VCOM_STEP_uV     10000
#define MAX17135_VCOM_MIN_VAL         0
#define MAX17135_VCOM_MAX_VAL       255
#define MAX17135_VCOM_FUDGE_FACTOR 330000
#endif

#define MAX17135_VCOM_VOLTAGE_DEFAULT -1250000

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

struct max17135 {
	/* chip revision */
	int rev;

	struct device *dev;

	/* Platform connection */
	struct i2c_client *i2c_client;

	/* Client devices */
	struct platform_device *pdev[MAX17135_REG_NUM];

	/* GPIOs */
	int gpio_pmic_pwrgood;
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	int gpio_pmic_intr;

	bool vcom_setup;

	int max_wait;
};

/*
 * Regulator operations
 */
static int max17135_hvinp_set_voltage(struct regulator_dev *reg,
					int minuV, int uV)
{
	unsigned int reg_val;
	unsigned int fld_val;
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	struct i2c_client *client = max17135->i2c_client;

	if ((uV >= MAX17135_HVINP_MIN_uV) &&
	    (uV <= MAX17135_HVINP_MAX_uV))
		fld_val = (uV - MAX17135_HVINP_MIN_uV) /
			MAX17135_HVINP_STEP_uV;
	else
		return -EINVAL;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_HVINP);

	reg_val &= ~BITFMASK(HVINP);
	reg_val |= BITFVAL(HVINP, fld_val); /* shift to correct bit */

	return i2c_smbus_write_byte_data(client, REG_MAX17135_HVINP, reg_val);
}

static int max17135_hvinp_get_voltage(struct regulator_dev *reg)
{
	unsigned int reg_val;
	unsigned int fld_val;
	int volt;
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	struct i2c_client *client = max17135->i2c_client;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_HVINP);

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
static inline int vcom_uV_to_rs(int uV)
{
	return (MAX17135_VCOM_MAX_uV - uV) / MAX17135_VCOM_STEP_uV;
}

/* Convert the VCOM register bitfield setting to uV */
static inline int vcom_rs_to_uV(int rs)
{
	return MAX17135_VCOM_MAX_uV - (MAX17135_VCOM_STEP_uV * rs);
}

static int max17135_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	struct i2c_client *client = max17135->i2c_client;
	unsigned int reg_val;
	int vcom_read;

	if ((uV < MAX17135_VCOM_MIN_uV) || (uV > MAX17135_VCOM_MAX_uV))
		return -EINVAL;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_DVR);

	/*
	 * Only program VCOM if it is not set to the desired value.
	 * Programming VCOM excessively degrades ability to keep
	 * DVR register value persistent.
	 */
	vcom_read = vcom_rs_to_uV(reg_val) - MAX17135_VCOM_FUDGE_FACTOR;
	if (vcom_read != MAX17135_VCOM_VOLTAGE_DEFAULT) {
		reg_val &= ~BITFMASK(DVR);
		reg_val |= BITFVAL(DVR,
			vcom_uV_to_rs(uV + MAX17135_VCOM_FUDGE_FACTOR));
		i2c_smbus_write_byte_data(client, REG_MAX17135_DVR, reg_val);

		reg_val = BITFVAL(CTRL_DVR, true); /* shift to correct bit */
		return i2c_smbus_write_byte_data(client,
			REG_MAX17135_PRGM_CTRL, reg_val);
	}
}

static int max17135_vcom_get_voltage(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
	struct i2c_client *client = max17135->i2c_client;
	unsigned int reg_val;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_DVR);
	return vcom_rs_to_uV(BITFEXT(reg_val, DVR));
}

static int max17135_vcom_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);

	/*
	 * Check to see if we need to set the VCOM voltage.
	 * Should only be done one time. And, we can
	 * only change vcom voltage if we have been enabled.
	 */
	if (!max17135->vcom_setup
		&& gpio_get_value(max17135->gpio_pmic_pwrgood)) {
		max17135_vcom_set_voltage(reg,
			MAX17135_VCOM_VOLTAGE_DEFAULT,
			MAX17135_VCOM_VOLTAGE_DEFAULT);
		max17135->vcom_setup = true;
	}

	/* enable VCOM regulator output */
#if (MAX17135_REV == 1)
	gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 1);
#else
	struct i2c_client *client = max17135->i2c_client;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_ENABLE);
	reg_val &= ~BITFMASK(VCOM_ENABLE);
	reg_val |= BITFVAL(VCOM_ENABLE, 1); /* shift to correct bit */
	i2c_smbus_write_byte_data(client, REG_MAX17135_ENABLE, reg_val);
#endif
	return 0;
}

static int max17135_vcom_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
#if (MAX17135_REV == 1)
	gpio_set_value(max17135->gpio_pmic_vcom_ctrl, 0);
#else
	struct i2c_client *client = max17135->i2c_client;
	unsigned int reg_val;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_ENABLE);
	reg_val &= ~BITFMASK(VCOM_ENABLE);
	i2c_smbus_write_byte_data(client, REG_MAX17135_ENABLE, reg_val);
#endif
	return 0;
}

static int max17135_wait_power_good(struct max17135 *max17135)
{
	int i;

	for (i = 0; i < max17135->max_wait * 3; i++) {
		if (gpio_get_value(max17135->gpio_pmic_pwrgood))
			return 0;

		msleep(1);
	}
	return -ETIMEDOUT;
}

static int max17135_display_enable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
#if (MAX17135_REV == 1)
	gpio_set_value(max17135->gpio_pmic_wakeup, 1);
#else
	struct i2c_client *client = max17135->i2c_client;
	unsigned int reg_val;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_ENABLE);
	reg_val &= ~BITFMASK(ENABLE);
	reg_val |= BITFVAL(ENABLE, 1);
	i2c_smbus_write_byte_data(client, REG_MAX17135_ENABLE, reg_val);
#endif

	return max17135_wait_power_good(max17135);
}

static int max17135_display_disable(struct regulator_dev *reg)
{
	struct max17135 *max17135 = rdev_get_drvdata(reg);
#if (MAX17135_REV == 1)
	gpio_set_value(max17135->gpio_pmic_wakeup, 0);
#else
	struct i2c_client *client = max17135->i2c_client;
	unsigned int reg_val;

	reg_val = i2c_smbus_read_byte_data(client, REG_MAX17135_ENABLE);
	reg_val &= ~BITFMASK(ENABLE);
	i2c_smbus_write_byte_data(client, REG_MAX17135_ENABLE, reg_val);
	msleep(PMIC_DISABLE__V3P3_DESERT/1000);
#endif
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
};

static struct regulator_ops max17135_vneg_ops = {
};

static struct regulator_ops max17135_vpos_ops = {
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
};

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

static int max17135_register_regulator(struct max17135 *max17135, int reg,
				     struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	struct i2c_client *client = max17135->i2c_client;
	/* If we can't find PMIC via I2C, we should not register regulators */
	if (i2c_smbus_read_byte_data(client,
		REG_MAX17135_PRODUCT_REV >= 0)) {
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

	return ret;
}

static int max17135_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int i;
	struct max17135 *max17135;
	struct max17135_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	if (!pdata || !pdata->regulator_init)
		return -ENODEV;

	/* Create the PMIC data structure */
	max17135 = kzalloc(sizeof(struct max17135), GFP_KERNEL);
	if (max17135 == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, max17135);
	max17135->dev = &client->dev;
	max17135->i2c_client = client;

	max17135->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	max17135->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	max17135->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	max17135->gpio_pmic_intr = pdata->gpio_pmic_intr;

	max17135->vcom_setup = false;

	ret = platform_driver_register(&max17135_regulator_driver);
	if (ret < 0)
		goto err;

	for (i = 0; i <= MAX17135_VPOS; i++) {
		ret = max17135_register_regulator(max17135, i, &pdata->regulator_init[i]);
		if (ret != 0) {
			dev_err(max17135->dev, "Platform init() failed: %d\n",
			ret);
		goto err;
		}
	}

	max17135->max_wait = pdata->vpos_pwrup + pdata->vneg_pwrup +
		pdata->gvdd_pwrup + pdata->gvee_pwrup;

	/* Initialize the PMIC device */
	dev_info(&client->dev, "PMIC MAX17135 for eInk display\n");

	return ret;
err:
	kfree(max17135);

	return ret;
}


static int max17135_i2c_remove(struct i2c_client *i2c)
{
	struct max17135 *max17135 = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < ARRAY_SIZE(max17135->pdev); i++)
		platform_device_unregister(max17135->pdev[i]);

	platform_driver_unregister(&max17135_regulator_driver);

	kfree(max17135);

	return 0;
}

static const struct i2c_device_id max17135_i2c_id[] = {
       { "max17135", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, max17135_i2c_id);


static struct i2c_driver max17135_i2c_driver = {
	.driver = {
		   .name = "max17135",
		   .owner = THIS_MODULE,
	},
	.probe = max17135_i2c_probe,
	.remove = max17135_i2c_remove,
	.id_table = max17135_i2c_id,
};

static int __init max17135_init(void)
{
	return i2c_add_driver(&max17135_i2c_driver);
}
module_init(max17135_init);

static void __exit max17135_exit(void)
{
	i2c_del_driver(&max17135_i2c_driver);
}
module_exit(max17135_exit);

/* Module information */
MODULE_DESCRIPTION("MAX17135 regulator driver");
MODULE_LICENSE("GPL");
