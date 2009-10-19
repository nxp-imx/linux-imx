/*
 * Freescale STMP37XX/STMP378X voltage regulator structure declarations
 *
 * Embedded Alley Solutions, Inc <sources@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
//#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

struct stmp3xxx_regulator {
	struct regulator_desc regulator;
	struct stmp3xxx_regulator *parent;
	struct stmp3xxx_platform_regulator_data *rdata;
	struct completion done;

	spinlock_t         lock;
	wait_queue_head_t  wait_q;
	struct notifier_block nb;

	int mode;
	int cur_voltage;
	int cur_current;
	int next_current;
};


struct stmp3xxx_platform_regulator_data {
	char name[80];
	char *parent_name;
	int (*reg_register)(struct stmp3xxx_regulator *sreg);
	int (*set_voltage)(struct stmp3xxx_regulator *sreg, int uv);
	int (*get_voltage)(struct stmp3xxx_regulator *sreg);
	int (*set_current)(struct stmp3xxx_regulator *sreg, int uA);
	int (*get_current)(struct stmp3xxx_regulator *sreg);
	int (*enable)(struct stmp3xxx_regulator *sreg);
	int (*disable)(struct stmp3xxx_regulator *sreg);
	int (*is_enabled)(struct stmp3xxx_regulator *sreg);
	int (*set_mode)(struct stmp3xxx_regulator *sreg, int mode);
	int (*get_mode)(struct stmp3xxx_regulator *sreg);
	int (*get_optimum_mode)(struct stmp3xxx_regulator *sreg,
			int input_uV, int output_uV, int load_uA);
	u32 control_reg;
	int min_voltage;
	int max_voltage;
	int max_current;
	struct regulation_constraints *constraints;
};

int stmp3xxx_register_regulator(
		struct stmp3xxx_regulator *reg_data, int reg,
		      struct regulator_init_data *initdata);

#endif /* __VOLTAGE_H */
