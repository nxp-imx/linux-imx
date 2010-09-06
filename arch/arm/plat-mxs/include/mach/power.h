/*
 * Freescale MXS voltage regulator structure declarations
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __VOLTAGE_H
#define __VOLTAGE_H
#include <linux/completion.h>
#include <linux/regulator/driver.h>

struct mxs_regulator {
	struct regulator_desc regulator;
	struct mxs_regulator *parent;
	struct mxs_platform_regulator_data *rdata;
	struct completion done;

	spinlock_t         lock;
	wait_queue_head_t  wait_q;
	struct notifier_block nb;

	int mode;
	int cur_voltage;
	int cur_current;
	int next_current;
};


struct mxs_platform_regulator_data {
	char name[80];
	char *parent_name;
	int (*reg_register)(struct mxs_regulator *sreg);
	int (*set_voltage)(struct mxs_regulator *sreg, int uv);
	int (*get_voltage)(struct mxs_regulator *sreg);
	int (*set_current)(struct mxs_regulator *sreg, int uA);
	int (*get_current)(struct mxs_regulator *sreg);
	int (*enable)(struct mxs_regulator *sreg);
	int (*disable)(struct mxs_regulator *sreg);
	int (*is_enabled)(struct mxs_regulator *sreg);
	int (*set_mode)(struct mxs_regulator *sreg, int mode);
	int (*get_mode)(struct mxs_regulator *sreg);
	int (*get_optimum_mode)(struct mxs_regulator *sreg,
			int input_uV, int output_uV, int load_uA);
	u32 control_reg;
	int min_voltage;
	int max_voltage;
	int max_current;
	struct regulation_constraints *cnstraints;
};

int mxs_register_regulator(
		struct mxs_regulator *reg_data, int reg,
		      struct regulator_init_data *initdata);

#endif /* __VOLTAGE_H */
