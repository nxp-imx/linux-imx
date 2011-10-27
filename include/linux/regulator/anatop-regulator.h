/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ANATOP_REGULATOR_H
#define __ANATOP_REGULATOR_H
#include <linux/regulator/driver.h>

/* regulator supplies for Anatop */
enum anatop_regulator_supplies {
	ANATOP_VDDPU,
	ANATOP_VDDCORE,
	ANATOP_VDDSOC,
	ANATOP_VDD2P5,
	ANATOP_VDD1P1,
	ANATOP_VDD3P0,
	ANATOP_SUPPLY_NUM
};

struct anatop_regulator {
	struct regulator_desc regulator;
	struct anatop_regulator *parent;
	struct anatop_regulator_data *rdata;
	struct completion done;

	spinlock_t         lock;
	wait_queue_head_t  wait_q;
	struct notifier_block nb;

	int mode;
	int cur_voltage;
	int cur_current;
	int next_current;
};


struct anatop_regulator_data {
	char name[80];
	char *parent_name;
	int (*reg_register)(struct anatop_regulator *sreg);
	int (*set_voltage)(struct anatop_regulator *sreg, int uv);
	int (*get_voltage)(struct anatop_regulator *sreg);
	int (*set_current)(struct anatop_regulator *sreg, int uA);
	int (*get_current)(struct anatop_regulator *sreg);
	int (*enable)(struct anatop_regulator *sreg);
	int (*disable)(struct anatop_regulator *sreg);
	int (*is_enabled)(struct anatop_regulator *sreg);
	int (*set_mode)(struct anatop_regulator *sreg, int mode);
	int (*get_mode)(struct anatop_regulator *sreg);
	int (*get_optimum_mode)(struct anatop_regulator *sreg,
			int input_uV, int output_uV, int load_uA);
	u32 control_reg;
	int vol_bit_shift;
	int vol_bit_mask;
	int min_bit_val;
	int min_voltage;
	int max_voltage;
	int max_current;
	struct regulation_constraints *cnstraints;
};

int anatop_register_regulator(
		struct anatop_regulator *reg_data, int reg,
		      struct regulator_init_data *initdata);

#endif /* __ANATOP_REGULATOR_H */
