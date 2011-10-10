/*
 * mc34708.h - regulators for the Freescale mc34708 PMIC
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc.
 *  based on:
 *  Copyright (C) 2010 Yong Shen <yong.shen@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_REGULATOR_MC34708_H
#define __LINUX_REGULATOR_MC34708_H

#include <linux/regulator/driver.h>

struct mc34708_regulator {
	struct regulator_desc desc;
	int reg;
	int enable_bit;
	int vsel_reg;
	int vsel_shift;
	int vsel_mask;
	int hi_bit;
	int const *voltages;
};

struct mc34708_regulator_priv {
	struct mc_pmic *mc34708;
	struct mc34708_regulator *mc34708_regulators;
	struct regulator_dev *regulators[];
};

int mc34708_sw_regulator(struct regulator_dev *rdev);
int mc34708_sw_regulator_is_enabled(struct regulator_dev *rdev);
int mc34708_get_best_voltage_index(struct regulator_dev *rdev,
				   int min_uV, int max_uV);
int mc34708_regulator_list_voltage(struct regulator_dev *rdev,
				   unsigned selector);
int mc34708_fixed_regulator_set_voltage(struct regulator_dev *rdev,
					int min_uV, int max_uV,
					unsigned *selector);
int mc34708_fixed_regulator_get_voltage(struct regulator_dev *rdev);

#define MC34708_DEFINE(prefix, _name, _reg, _vsel_reg, _voltages, _ops)	\
	[prefix ## _name] = {				\
		.desc = {						\
			.name = #prefix "_" #_name,			\
			.n_voltages = ARRAY_SIZE(_voltages),		\
			.ops = &_ops,			\
			.type = REGULATOR_VOLTAGE,			\
			.id = prefix ## _name,		\
			.owner = THIS_MODULE,				\
		},							\
		.reg = prefix ## _reg,				\
		.enable_bit = prefix ## _reg ## _ ## _name ## EN,	\
		.vsel_reg = prefix ## _vsel_reg,			\
		.vsel_shift = prefix ## _vsel_reg ## _ ## _name ## VSEL,\
		.vsel_mask = prefix ## _vsel_reg ## _ ## _name ## VSEL_M,\
		.voltages =  _voltages,					\
	}

#define MC34708_FIXED_DEFINE(prefix, _name, _reg, _voltages, _ops)	\
	[prefix ## _name] = {				\
		.desc = {						\
			.name = #prefix "_" #_name,			\
			.n_voltages = ARRAY_SIZE(_voltages),		\
			.ops = &_ops,		\
			.type = REGULATOR_VOLTAGE,			\
			.id = prefix ## _name,		\
			.owner = THIS_MODULE,				\
		},							\
		.reg = prefix ## _reg,				\
		.enable_bit = prefix ## _reg ## _ ## _name ## EN,	\
		.voltages =  _voltages,					\
	}

#endif
