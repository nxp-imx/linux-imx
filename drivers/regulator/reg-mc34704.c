/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/ioctl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/mc34704/core.h>
#include <linux/platform_device.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

#define MC34704_ONOFFA	0x8
#define MC34704_ONOFFC	0x4
#define MC34704_ONOFFD	0x2
#define MC34704_ONOFFE	0x1

/* Private data for MC34704 regulators */

struct reg_mc34704_priv {
	short enable;		/* enable bit, if available */
	short v_default;	/* default regulator voltage in mV */
	int dvs_min;		/* minimum voltage change in units of 2.5% */
	int dvs_max;		/* maximum voltage change in units of 2.5% */
	char i2c_dvs;		/* i2c DVS register number */
	char i2c_stat;		/* i2c status register number */
};
struct reg_mc34704_priv mc34704_reg_priv[] = {
	{
	 .v_default = REG1_V_MV,
	 .dvs_min = REG1_DVS_MIN_PCT / 2.5,
	 .dvs_max = REG1_DVS_MAX_PCT / 2.5,
	 .i2c_dvs = 0x4,
	 .i2c_stat = 0x5,
	 .enable = MC34704_ONOFFA,
	 },
	{
	 .v_default = REG2_V_MV,
	 .dvs_min = REG2_DVS_MIN_PCT / 2.5,
	 .dvs_max = REG2_DVS_MAX_PCT / 2.5,
	 .i2c_dvs = 0x6,
	 .i2c_stat = 0x7,
	 },
	{
	 .v_default = REG3_V_MV,
	 .dvs_min = REG3_DVS_MIN_PCT / 2.5,
	 .dvs_max = REG3_DVS_MAX_PCT / 2.5,
	 .i2c_dvs = 0x8,
	 .i2c_stat = 0x9,
	 },
	{
	 .v_default = REG4_V_MV,
	 .dvs_min = REG4_DVS_MIN_PCT / 2.5,
	 .dvs_max = REG4_DVS_MAX_PCT / 2.5,
	 .i2c_dvs = 0xA,
	 .i2c_stat = 0xB,
	 },
	{
	 .v_default = REG5_V_MV,
	 .dvs_min = REG5_DVS_MIN_PCT / 2.5,
	 .dvs_max = REG5_DVS_MAX_PCT / 2.5,
	 .i2c_dvs = 0xC,
	 .i2c_stat = 0xE,
	 .enable = MC34704_ONOFFE,
	 },
};

static int mc34704_set_voltage(struct regulator_dev *reg, int MiniV, int uV)
{
	struct reg_mc34704_priv *priv = rdev_get_drvdata(reg);
	int mV = uV / 1000;
	int dV = mV - priv->v_default;

	/* compute dynamic voltage scaling value */
	int dvs = 1000 * dV / priv->v_default / 25;

	/* clip to regulator limits */
	if (dvs > priv->dvs_max)
		dvs = priv->dvs_max;
	if (dvs < priv->dvs_min)
		dvs = priv->dvs_min;

	return pmic_write_reg(priv->i2c_dvs, dvs << 1, 0x1E);
}

static int mc34704_get_voltage(struct regulator_dev *reg)
{
	int mV;
	struct reg_mc34704_priv *priv = rdev_get_drvdata(reg);
	int val, dvs;

	CHECK_ERROR(pmic_read_reg(priv->i2c_dvs, &val, 0xF));

	dvs = (val >> 1) & 0xF;

	/* dvs is 4-bit 2's complement; sign-extend it */
	if (dvs & 8)
		dvs |= -1 & ~0xF;

	/* Regulator voltage is adjusted by (dvs * 2.5%) */
	mV = priv->v_default * (1000 + 25 * dvs) / 1000;

	return 1000 * mV;
}

static int mc34704_enable_reg(struct regulator_dev *reg)
{
	struct reg_mc34704_priv *priv = rdev_get_drvdata(reg);

	if (priv->enable)
		return pmic_write_reg(REG_MC34704_GENERAL2, -1, priv->enable);

	return PMIC_ERROR;
}

static int mc34704_disable_reg(struct regulator_dev *reg)
{
	struct reg_mc34704_priv *priv = rdev_get_drvdata(reg);

	if (priv->enable)
		return pmic_write_reg(REG_MC34704_GENERAL2, 0, priv->enable);

	return PMIC_ERROR;
}

static int mc34704_is_reg_enabled(struct regulator_dev *reg)
{
	struct reg_mc34704_priv *priv = rdev_get_drvdata(reg);
	int val;

	if (priv->enable) {
		CHECK_ERROR(pmic_read_reg(REG_MC34704_GENERAL2, &val,
					  priv->enable));
		return val ? 1 : 0;
	} else {
		return PMIC_ERROR;
	}
}

static struct regulator_ops mc34704_full_ops = {
	.set_voltage = mc34704_set_voltage,
	.get_voltage = mc34704_get_voltage,
	.enable = mc34704_enable_reg,
	.disable = mc34704_disable_reg,
	.is_enabled = mc34704_is_reg_enabled,
};

static struct regulator_ops mc34704_partial_ops = {
	.set_voltage = mc34704_set_voltage,
	.get_voltage = mc34704_get_voltage,
};

static struct regulator_desc reg_mc34704[] = {
	{
	 .name = "REG1_BKLT",
	 .id = MC34704_BKLT,
	 .ops = &mc34704_full_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "REG2_CPU",
	 .id = MC34704_CPU,
	 .ops = &mc34704_partial_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "REG3_CORE",
	 .id = MC34704_CORE,
	 .ops = &mc34704_partial_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "REG4_DDR",
	 .id = MC34704_DDR,
	 .ops = &mc34704_partial_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
	{
	 .name = "REG5_PERS",
	 .id = MC34704_PERS,
	 .ops = &mc34704_full_ops,
	 .irq = 0,
	 .type = REGULATOR_VOLTAGE,
	 .owner = THIS_MODULE},
};

static int mc34704_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;

	/* register regulator */
	rdev = regulator_register(&reg_mc34704[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  (void *)&mc34704_reg_priv[pdev->id]);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			reg_mc34704[pdev->id].name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static int mc34704_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

int mc34704_register_regulator(struct mc34704 *mc34704, int reg,
			       struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;

	if (mc34704->pmic.pdev[reg])
		return -EBUSY;

	pdev = platform_device_alloc("mc34704-regulatr", reg);
	if (!pdev)
		return -ENOMEM;

	mc34704->pmic.pdev[reg] = pdev;

	initdata->driver_data = mc34704;

	pdev->dev.platform_data = initdata;

	pdev->dev.parent = mc34704->dev;
	platform_set_drvdata(pdev, mc34704);
	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(mc34704->dev, "Failed to register regulator %d: %d\n",
			reg, ret);
		platform_device_del(pdev);
		mc34704->pmic.pdev[reg] = NULL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mc34704_register_regulator);

static struct platform_driver mc34704_regulator_driver = {
	.probe = mc34704_regulator_probe,
	.remove = mc34704_regulator_remove,
	.driver = {
		   .name = "mc34704-regulatr",
		   },
};

static int __init mc34704_regulator_init(void)
{
	return platform_driver_register(&mc34704_regulator_driver);
}
subsys_initcall(mc34704_regulator_init);

static void __exit mc34704_regulator_exit(void)
{
	platform_driver_unregister(&mc34704_regulator_driver);
}
module_exit(mc34704_regulator_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MC34704 Regulator Driver");
MODULE_LICENSE("GPL");
