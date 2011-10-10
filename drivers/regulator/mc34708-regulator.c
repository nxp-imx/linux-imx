/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/mfd/mc-pmic.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/err.h>
#include "mc34708.h"

static const int mc34708_sw1A[] = {
	650000, 662500, 675000, 687500, 700000, 712500,
	725000, 737500, 750000, 762500, 775000, 787500,
	800000, 812500, 825000, 837500, 850000, 862500,
	875000, 887500, 900000, 912500, 925000, 937500,
	950000, 962500, 975000, 987500, 1000000, 1012500,
	1025000, 1037500, 1050000, 1062500, 1075000, 1087500,
	1100000, 1112500, 1125000, 1137500, 1150000, 1162500,
	1175000, 1187500, 1200000, 1212500, 1225000, 1237500,
	1250000, 1262500, 1275000, 1287500, 1300000, 1312500,
	1325000, 1337500, 1350000, 1362500, 1375000, 1387500,
	1400000, 1412500, 1425000, 1437500,
};


static const int mc34708_sw2[] = {
	650000, 662500, 675000, 687500, 700000, 712500,
	725000, 737500, 750000, 762500, 775000, 787500,
	800000, 812500, 825000, 837500, 850000, 862500,
	875000, 887500, 900000, 912500, 925000, 937500,
	950000, 962500, 975000, 987500, 1000000, 1012500,
	1025000, 1037500, 1050000, 1062500, 1075000, 1087500,
	1100000, 1112500, 1125000, 1137500, 1150000, 1162500,
	1175000, 1187500, 1200000, 1212500, 1225000, 1237500,
	1250000, 1262500, 1275000, 1287500, 1300000, 1312500,
	1325000, 1337500, 1350000, 1362500, 1375000, 1387500,
	1400000, 1412500, 1425000, 1437500,
};

static const int mc34708_sw3[] = {
	650000, 675000, 700000, 725000, 750000, 775000,
	800000, 825000, 850000, 875000, 900000, 925000,
	950000, 975000, 1000000, 1025000, 1050000, 1075000,
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000,
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000,
	1400000, 1425000,
};

static const int mc34708_sw4A[] = {
	1200000, 1225000, 1250000, 1275000, 1300000, 1325000,
	1350000, 1375000, 1400000, 1425000, 1450000, 1475000,
	1500000, 1525000, 1550000, 1575000, 1600000, 1625000,
	1650000, 1675000, 1700000, 1725000, 1750000, 1775000,
	1800000, 1825000, 1850000, 2500000, 3150000,
};


static const int mc34708_sw5[] = {
	1200000, 1225000, 1250000, 1275000, 1300000, 1325000,
	1350000, 1375000, 1400000, 1425000, 1450000, 1475000,
	1500000, 1525000, 1550000, 1575000, 1600000, 1625000,
	1650000, 1675000, 1700000, 1725000, 1750000, 1775000,
	1800000, 1825000, 1850000,
};

static const int mc34708_swbst[] = {
	5000000, 5050000, 5100000, 5150000,
};

static const int mc34708_vpll[] = {
	1200000, 1250000, 1500000, 1800000,
};

static const int mc34708_vrefddr[] = {
	600000,
};

static const int mc34708_vusb[] = {
	3300000,
};

static const int mc34708_vusb2[] = {
	2500000, 2600000, 2750000, 3000000,
};

static const int mc34708_vdac[] = {
	2500000, 2600000, 2750000, 2775000,
};

static const int mc34708_vgen1[] = {
	1200000, 1250000, 1300000, 1350000,
	1400000, 1450000, 1500000, 1550000,
};

static const int mc34708_vgen2[] = {
	2500000, 2700000, 2800000, 2900000,
	3000000, 3100000, 3150000, 3300000,
};

static struct regulator_ops mc34708_regulator_ops;
static struct regulator_ops mc34708_fixed_regulator_ops;
/* sw regulators need special care due to the "hi bit" */
static struct regulator_ops mc34708_sw_regulator_ops;
static struct regulator_ops mc34708_sw4_regulator_ops;

#define MC34708_FIXED_VOL_DEFINE(name, reg, voltages)		\
	MC34708_FIXED_DEFINE(MC34708_, name, reg, voltages,	\
			mc34708_fixed_regulator_ops)

#define MC34708_SW_DEFINE(name, reg, vsel_reg, voltages)	\
	MC34708_DEFINE(MC34708_, name, reg, vsel_reg, voltages, \
			mc34708_sw_regulator_ops)

#define MC34708_DEFINE_REGU(name, reg, vsel_reg, voltages)	\
	MC34708_DEFINE(MC34708_, name, reg, vsel_reg, voltages, \
			mc34708_regulator_ops)

#define MC34708_SW4_DEFINE(name, reg, vsel_reg, voltages)	\
	MC34708_DEFINE(MC34708_, name, reg, vsel_reg, voltages, \
			mc34708_sw4_regulator_ops)

#define MC34708_REVISION	7

#define MC34708_SW1ABVOL	24
#define MC34708_SW1ABVOL_SW1AVSEL	0
#define MC34708_SW1ABVOL_SW1AVSEL_M	(0x3f<<0)
#define MC34708_SW1ABVOL_SW1AEN	0
#define MC34708_SW1ABVOL_SW1BVSEL	0
#define MC34708_SW1ABVOL_SW1BVSEL_M	(0x3f<<0)
#define MC34708_SW1ABVOL_SW1BEN	0

#define MC34708_SW23VOL	25
#define MC34708_SW23VOL_SW2VSEL	0
#define MC34708_SW23VOL_SW2VSEL_M	(0x3f<<0)
#define MC34708_SW23VOL_SW2EN	0
#define MC34708_SW23VOL_SW3VSEL	12
#define MC34708_SW23VOL_SW3VSEL_M	(0x3f<<12)
#define MC34708_SW23VOL_SW3EN	0

#define MC34708_SW4ABVOL	26
#define MC34708_SW4ABVOL_SW4AVSEL	0
#define MC34708_SW4ABVOL_SW4AVSEL_M	(0x1f<<0)
#define MC34708_SW4ABVOL_SW4AHI	10
#define MC34708_SW4ABVOL_SW4AHI_M	(0x3<<10)
#define MC34708_SW4ABVOL_SW4AEN	0
#define	MC34708_SW4ABVOL_SW4BVSEL	12
#define MC34708_SW4ABVOL_SW4BVSEL_M	(0x1f<<12)
#define MC34708_SW4ABVOL_SW4BHI	22
#define MC34708_SW4ABVOL_SW4BHI_M	(0x3<<22)
#define MC34708_SW4ABVOL_SW4BEN	0

#define MC34708_SW5VOL	27
#define MC34708_SW5VOL_SW5VSEL	0
#define MC34708_SW5VOL_SW5VSEL_M	(0x1f<<0)
#define MC34708_SW5VOL_SW5EN	0

#define MC34708_SW12OP	28
#define MC34708_SW12OP_SW1AMODE_M	(0xf<<0)
#define MC34708_SW12OP_SW1AMODE_VALUE	(0xc<<0) /*Normal:APS,Standby:PFM */
#define MC34708_SW12OP_SW2MODE_M	(0xf<<14)
#define MC34708_SW12OP_SW2MODE_VALUE	(0xc<<14) /*Normal:APS,Standby:PFM */

#define MC34708_SW345OP	29
#define MC34708_SW345OP_SW3MODE_M	(0xf<<0)
#define MC34708_SW345OP_SW3MODE_VALUE	(0x0<<0) /*Normal:OFF,Standby:OFF */
#define MC34708_SW345OP_SW4AMODE_M	(0xf<<6)
#define MC34708_SW345OP_SW4AMODE_VALUE	(0xc<<6) /*Normal:APS,Standby:PFM */
#define MC34708_SW345OP_SW4BMODE_M	(0xf<<12)
#define MC34708_SW345OP_SW4BMODE_VALUE	(0xc<<12) /*Normal:APS,Standby:PFM */
#define MC34708_SW345OP_SW5MODE_M	(0xf<<18)
#define MC34708_SW345OP_SW5MODE_VALUE	(0xc<<18) /*Normal:APS,Standby:PFM */

#define MC34708_REGULATORSET0	30
#define MC34708_REGULATORSET0_VGEN1VSEL	0
#define MC34708_REGULATORSET0_VGEN1VSEL_M	(0x7<<0)
#define MC34708_REGULATORSET0_VDACVSEL	4
#define MC34708_REGULATORSET0_VDACVSEL_M	(0x3<<4)
#define MC34708_REGULATORSET0_VGEN2VSEL	6
#define MC34708_REGULATORSET0_VGEN2VSEL_M	(0x7<<6)
#define MC34708_REGULATORSET0_VPLLVSEL	9
#define MC34708_REGULATORSET0_VPLLVSEL_M	(0x3<<9)
#define MC34708_REGULATORSET0_VUSB2VSEL	11
#define MC34708_REGULATORSET0_VUSB2VSEL_M	(0x3<<9)

#define MC34708_SWBSTCONTROL	31
#define MC34708_SWBSTCONTROL_SWBSTVSEL	0
#define MC34708_SWBSTCONTROL_SWBSTVSEL_M	(0x3<<0)
#define MC34708_SWBSTCONTROL_SWBSTMODE_M	(0x3<<5)
#define MC34708_SWBSTCONTROL_SWBSTMODE_VALUE	(0x2<<5)	/*auto mode */
#define MC34708_SWBSTCONTROL_SWBSTEN	0

#define MC34708_REGULATORMODE0	32
#define MC34708_REGULATORMODE0_VGEN1EN	0
#define MC34708_REGULATORMODE0_VUSBEN	3
#define MC34708_REGULATORMODE0_VDACEN	4
#define MC34708_REGULATORMODE0_VREFDDREN	10
#define MC34708_REGULATORMODE0_VGEN2EN	12
#define MC34708_REGULATORMODE0_VPLLEN	15
#define MC34708_REGULATORMODE0_VUSB2EN	18

#define MC34708_USBCONTROL	39
#define MC34708_USBCONTROL_SWHOLD_M	(0x1<<12)
#define MC34708_USBCONTROL_SWHOLD_NORM	(0x0<<12)

static struct mc34708_regulator mc34708_regulators[] = {
	MC34708_SW_DEFINE(SW1A, SW1ABVOL, SW1ABVOL, mc34708_sw1A),
	MC34708_SW_DEFINE(SW1B, SW1ABVOL, SW1ABVOL, mc34708_sw1A),
	MC34708_SW_DEFINE(SW2, SW23VOL, SW23VOL, mc34708_sw2),
	MC34708_SW_DEFINE(SW3, SW23VOL, SW23VOL, mc34708_sw3),
	MC34708_SW4_DEFINE(SW4A, SW4ABVOL, SW4ABVOL, mc34708_sw4A),
	MC34708_SW4_DEFINE(SW4B, SW4ABVOL, SW4ABVOL, mc34708_sw4A),
	MC34708_SW_DEFINE(SW5, SW5VOL, SW5VOL, mc34708_sw5),
	MC34708_SW_DEFINE(SWBST, SWBSTCONTROL, SWBSTCONTROL, mc34708_swbst),
	MC34708_DEFINE_REGU(VPLL, REGULATORMODE0, REGULATORSET0, mc34708_vpll),
	MC34708_FIXED_VOL_DEFINE(VREFDDR, REGULATORMODE0, mc34708_vrefddr),
	MC34708_FIXED_VOL_DEFINE(VUSB, REGULATORMODE0, mc34708_vusb),
	MC34708_DEFINE_REGU(VUSB2, REGULATORMODE0, REGULATORSET0,
			    mc34708_vusb2),
	MC34708_DEFINE_REGU(VDAC, REGULATORMODE0, REGULATORSET0, mc34708_vdac),
	MC34708_DEFINE_REGU(VGEN1, REGULATORMODE0, REGULATORSET0,
			    mc34708_vgen1),
	MC34708_DEFINE_REGU(VGEN2, REGULATORMODE0, REGULATORSET0,
			    mc34708_vgen2),
};

static int mc34708_regulator_enable(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_rmw(priv->mc34708, mc34708_regulators[id].reg,
			      mc34708_regulators[id].enable_bit,
			      mc34708_regulators[id].enable_bit);
	mc_pmic_unlock(priv->mc34708);

	return ret;
}

static int mc34708_regulator_disable(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_rmw(priv->mc34708, mc34708_regulators[id].reg,
			      mc34708_regulators[id].enable_bit, 0);
	mc_pmic_unlock(priv->mc34708);

	return ret;
}

static int mc34708_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int ret, id = rdev_get_id(rdev);
	unsigned int val;

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_read(priv->mc34708, mc34708_regulators[id].reg, &val);
	mc_pmic_unlock(priv->mc34708);

	if (ret)
		return ret;

	return (val & mc34708_regulators[id].enable_bit) != 0;
}

int
mc34708_regulator_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int id = rdev_get_id(rdev);
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;

	if (selector >= mc34708_regulators[id].desc.n_voltages)
		return -EINVAL;

	return mc34708_regulators[id].voltages[selector];
}

EXPORT_SYMBOL_GPL(mc34708_regulator_list_voltage);

int
mc34708_get_best_voltage_index(struct regulator_dev *rdev,
			       int min_uV, int max_uV)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int reg_id = rdev_get_id(rdev);
	int i;
	int bestmatch;
	int bestindex;

	/*
	 * Locate the minimum voltage fitting the criteria on
	 * this regulator. The switchable voltages are not
	 * in strict falling order so we need to check them
	 * all for the best match.
	 */
	bestmatch = INT_MAX;
	bestindex = -1;
	for (i = 0; i < mc34708_regulators[reg_id].desc.n_voltages; i++) {
		if (mc34708_regulators[reg_id].voltages[i] >= min_uV &&
		    mc34708_regulators[reg_id].voltages[i] < bestmatch) {
			bestmatch = mc34708_regulators[reg_id].voltages[i];
			bestindex = i;
		}
	}

	if (bestindex < 0 || bestmatch > max_uV) {
		dev_warn(&rdev->dev, "no possible value for %d<=x<=%d uV\n",
			 min_uV, max_uV);
		return -EINVAL;
	}
	return bestindex;
}

EXPORT_SYMBOL_GPL(mc34708_get_best_voltage_index);

static int
mc34708_regulator_set_voltage(struct regulator_dev *rdev, int min_uV,
			      int max_uV, unsigned *selector)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int value, id = rdev_get_id(rdev);
	int ret;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d min_uV: %d max_uV: %d\n",
		__func__, id, min_uV, max_uV);

	/* Find the best index */
	value = mc34708_get_best_voltage_index(rdev, min_uV, max_uV);
	dev_dbg(rdev_get_dev(rdev), "%s best value: %d\n", __func__, value);
	if (value < 0)
		return value;

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_rmw(priv->mc34708, mc34708_regulators[id].vsel_reg,
			      mc34708_regulators[id].vsel_mask,
			      value << mc34708_regulators[id].vsel_shift);
	mc_pmic_unlock(priv->mc34708);

	return ret;
}

static int mc34708_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int ret, id = rdev_get_id(rdev);
	unsigned int val;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_read(priv->mc34708,
			       mc34708_regulators[id].vsel_reg, &val);
	mc_pmic_unlock(priv->mc34708);

	if (ret)
		return ret;

	val = (val & mc34708_regulators[id].vsel_mask)
	    >> mc34708_regulators[id].vsel_shift;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d val: %d\n", __func__, id, val);

	BUG_ON(val > mc34708_regulators[id].desc.n_voltages);

	return mc34708_regulators[id].voltages[val];
}

static struct regulator_ops mc34708_regulator_ops = {
	.enable = mc34708_regulator_enable,
	.disable = mc34708_regulator_disable,
	.is_enabled = mc34708_regulator_is_enabled,
	.list_voltage = mc34708_regulator_list_voltage,
	.set_voltage = mc34708_regulator_set_voltage,
	.get_voltage = mc34708_regulator_get_voltage,
};

EXPORT_SYMBOL_GPL(mc34708_regulator_ops);

int
mc34708_fixed_regulator_set_voltage(struct regulator_dev *rdev, int min_uV,
				    int max_uV, unsigned *selector)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int id = rdev_get_id(rdev);

	dev_dbg(rdev_get_dev(rdev), "%s id: %d min_uV: %d max_uV: %d\n",
		__func__, id, min_uV, max_uV);

	if (min_uV >= mc34708_regulators[id].voltages[0] &&
	    max_uV <= mc34708_regulators[id].voltages[0])
		return 0;
	else
		return -EINVAL;
}

EXPORT_SYMBOL_GPL(mc34708_fixed_regulator_set_voltage);

int mc34708_fixed_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int id = rdev_get_id(rdev);

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);

	return mc34708_regulators[id].voltages[0];
}

EXPORT_SYMBOL_GPL(mc34708_fixed_regulator_get_voltage);

static struct regulator_ops mc34708_fixed_regulator_ops = {
	.enable = mc34708_regulator_enable,
	.disable = mc34708_regulator_disable,
	.is_enabled = mc34708_regulator_is_enabled,
	.list_voltage = mc34708_regulator_list_voltage,
	.set_voltage = mc34708_fixed_regulator_set_voltage,
	.get_voltage = mc34708_fixed_regulator_get_voltage,
};

EXPORT_SYMBOL_GPL(mc34708_fixed_regulator_ops);

int mc34708_sw_regulator_is_enabled(struct regulator_dev *rdev)
{
	return 1;
}

EXPORT_SYMBOL_GPL(mc34708_sw_regulator_is_enabled);

static int mc34708_sw4_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int ret, id = rdev_get_id(rdev);
	unsigned int val, hi;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d\n", __func__, id);

	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_read(priv->mc34708,
			       mc34708_regulators[id].vsel_reg, &val);
	mc_pmic_unlock(priv->mc34708);

	if (ret)
		return ret;
	hi = (val & MC34708_SW4ABVOL_SW4BHI_M) >> MC34708_SW4ABVOL_SW4BHI;
	val = (val & mc34708_regulators[id].vsel_mask)
	    >> mc34708_regulators[id].vsel_shift;
	dev_dbg(rdev_get_dev(rdev), "%s id: %d val: %d\n", __func__, id, val);

	if (hi == 0x1)		/*2500000 */
		val = 27;
	else if (hi == 0x2)	/*3150000 */
		val = 28;

	return mc34708_regulators[id].voltages[val];
}

static int
mc34708_sw4_regulator_set_voltage(struct regulator_dev *rdev,
				  int min_uV, int max_uV, unsigned *selector)
{
	struct mc34708_regulator_priv *priv = rdev_get_drvdata(rdev);
	struct mc34708_regulator *mc34708_regulators = priv->mc34708_regulators;
	int value, id = rdev_get_id(rdev);
	int ret, hi;

	dev_dbg(rdev_get_dev(rdev), "%s id: %d min_uV: %d max_uV: %d\n",
		__func__, id, min_uV, max_uV);

	/* Find the best index */
	value = mc34708_get_best_voltage_index(rdev, min_uV, max_uV);
	dev_dbg(rdev_get_dev(rdev), "%s best value: %d\n", __func__, value);
	if (value < 0)
		return value;
	if (value <= 26)
		hi = 0x0;
	else if (value == 27)
		hi = 0x1;
	else
		hi = 0x2;
	mc_pmic_lock(priv->mc34708);
	ret = mc_pmic_reg_rmw(priv->mc34708, mc34708_regulators[id].vsel_reg,
			      mc34708_regulators[id].vsel_mask |
			      MC34708_SW4ABVOL_SW4BHI_M,
			      value << mc34708_regulators[id].vsel_shift |
			      (hi << MC34708_SW4ABVOL_SW4BHI));
	mc_pmic_unlock(priv->mc34708);

	return ret;
}

static struct regulator_ops mc34708_sw4_regulator_ops = {
	.is_enabled = mc34708_sw_regulator_is_enabled,
	.list_voltage = mc34708_regulator_list_voltage,
	.set_voltage = mc34708_sw4_regulator_set_voltage,
	.get_voltage = mc34708_sw4_regulator_get_voltage,
};

static struct regulator_ops mc34708_sw_regulator_ops = {
	.is_enabled = mc34708_sw_regulator_is_enabled,
	.list_voltage = mc34708_regulator_list_voltage,
	.set_voltage = mc34708_regulator_set_voltage,
	.get_voltage = mc34708_regulator_get_voltage,
};

static int __devinit mc34708_regulator_probe(struct platform_device *pdev)
{
	struct mc34708_regulator_priv *priv;
	struct mc_pmic *mc34708 = dev_get_drvdata(pdev->dev.parent);
	struct mc_pmic_regulator_platform_data *pdata =
	    dev_get_platdata(&pdev->dev);
	struct mc_pmic_regulator_init_data *init_data;
	int i, ret;
	u32 val = 0;

	priv = kzalloc(sizeof(*priv) +
		       pdata->num_regulators * sizeof(priv->regulators[0]),
		       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mc34708_regulators = mc34708_regulators;
	priv->mc34708 = mc34708;

	mc_pmic_lock(mc34708);
	ret = mc_pmic_reg_read(mc34708, MC34708_REVISION, &val);
	if (ret)
		goto err_free;
	ret = mc_pmic_reg_rmw(mc34708, MC34708_SW12OP,
			      MC34708_SW12OP_SW1AMODE_M |
			      MC34708_SW12OP_SW2MODE_M,
			      MC34708_SW12OP_SW1AMODE_VALUE |
			      MC34708_SW12OP_SW2MODE_VALUE);
	if (ret)
		goto err_free;
	ret = mc_pmic_reg_rmw(mc34708, MC34708_SW345OP,
			      MC34708_SW345OP_SW3MODE_M |
			      MC34708_SW345OP_SW4AMODE_M |
			      MC34708_SW345OP_SW4BMODE_M |
			      MC34708_SW345OP_SW5MODE_M,
			      MC34708_SW345OP_SW3MODE_VALUE |
			      MC34708_SW345OP_SW4AMODE_VALUE |
			      MC34708_SW345OP_SW4BMODE_VALUE |
			      MC34708_SW345OP_SW5MODE_VALUE);
	if (ret)
		goto err_free;
	ret = mc_pmic_reg_rmw(mc34708, MC34708_SWBSTCONTROL,
			      MC34708_SWBSTCONTROL_SWBSTMODE_M,
			      MC34708_SWBSTCONTROL_SWBSTMODE_VALUE);
	if (ret)
		goto err_free;
	ret = mc_pmic_reg_rmw(mc34708, MC34708_USBCONTROL,
			      MC34708_USBCONTROL_SWHOLD_M,
			      MC34708_USBCONTROL_SWHOLD_NORM);
	if (ret)
		goto err_free;
	mc_pmic_unlock(mc34708);
	dev_dbg(&pdev->dev, "PMIC MC34708 ID:0x%x\n", val);
	for (i = 0; i < pdata->num_regulators; i++) {
		init_data = &pdata->regulators[i];
		priv->regulators[i] =
		    regulator_register(&mc34708_regulators[init_data->id].desc,
				       &pdev->dev, init_data->init_data, priv);

		if (IS_ERR(priv->regulators[i])) {
			dev_err(&pdev->dev, "failed to register regulator %s\n",
				mc34708_regulators[i].desc.name);
			ret = PTR_ERR(priv->regulators[i]);
			goto err;
		}
	}

	platform_set_drvdata(pdev, priv);

	return 0;
 err:
	while (--i >= 0)
		regulator_unregister(priv->regulators[i]);

 err_free:
	mc_pmic_unlock(mc34708);
	kfree(priv);

	return ret;
}

static int __devexit mc34708_regulator_remove(struct platform_device *pdev)
{
	struct mc34708_regulator_priv *priv = platform_get_drvdata(pdev);
	struct mc_pmic_regulator_platform_data *pdata =
	    dev_get_platdata(&pdev->dev);
	int i;

	platform_set_drvdata(pdev, NULL);

	for (i = 0; i < pdata->num_regulators; i++)
		regulator_unregister(priv->regulators[i]);

	kfree(priv);
	return 0;
}

static struct platform_driver mc34708_regulator_driver = {
	.driver = {
		   .name = "mc34708-regulator",
		   .owner = THIS_MODULE,
		   },
	.remove = __devexit_p(mc34708_regulator_remove),
	.probe = mc34708_regulator_probe,
};

static int __init mc34708_regulator_init(void)
{
	return platform_driver_register(&mc34708_regulator_driver);
}

subsys_initcall(mc34708_regulator_init);

static void __exit mc34708_regulator_exit(void)
{
	platform_driver_unregister(&mc34708_regulator_driver);
}

module_exit(mc34708_regulator_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Regulator Driver for Freescale MC34708 PMIC");
MODULE_ALIAS("mc34708-regulator");
