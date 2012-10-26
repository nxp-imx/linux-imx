/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef __LINUX_REGULATOR_PFUZE_H
#define __LINUX_REGULATOR_PFUZE_H

#include <linux/regulator/driver.h>

struct pfuze_regulator {
	struct regulator_desc desc;
	unsigned int reg;
	unsigned int stby_reg;
	unsigned char enable_bit;
	unsigned char stby_bit;
	unsigned char vsel_shift;
	unsigned char vsel_mask;
	unsigned char stby_vsel_shift;
	unsigned char stby_vsel_mask;
	int const *voltages;
};

struct pfuze_regulator_priv {
	struct mc_pfuze *pfuze;
	struct pfuze_regulator *pfuze_regulators;
	struct regulator_dev *regulators[];
};

#define PFUZE_DEFINE(prefix, _name, _reg, _voltages, _ops)	\
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
		.enable_bit = prefix ## _reg ## _ ## EN,	\
		.stby_bit = prefix ## _reg ## _ ## STBY,	\
		.vsel_shift = prefix ## _reg ## _ ## VSEL,\
		.vsel_mask = prefix ## _reg ## _ ## VSEL_M,\
		.voltages =  _voltages,					\
	}
#define PFUZE_SW_DEFINE(prefix, _name, _reg, _voltages, _ops)	\
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
		.vsel_shift = prefix ## _reg ## _ ## VSEL,\
		.vsel_mask = prefix ## _reg ## _ ## VSEL_M,\
		.stby_reg = prefix ## _reg ## _ ## STBY,		\
		.stby_vsel_shift = prefix ## _reg ## _ ## STBY_VSEL,\
		.stby_vsel_mask = prefix ## _reg ## _ ## STBY_VSEL_M,\
		.voltages =  _voltages,					\
	}

#define PFUZE_SWBST_DEFINE(prefix, _name, _reg, _voltages, _ops)	\
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
		.vsel_shift = prefix ## _reg ## _ ## VSEL,\
		.vsel_mask = prefix ## _reg ## _ ## VSEL_M,\
		.voltages =  _voltages,					\
	}

#define PFUZE_FIXED_DEFINE(prefix, _name, _reg, _voltages, _ops)	\
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
		.enable_bit = prefix ## _reg ## _ ## EN,	\
		.voltages =  _voltages,					\
	}

#endif

