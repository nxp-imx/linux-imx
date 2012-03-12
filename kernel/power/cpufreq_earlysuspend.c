/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/fs.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>

void cpufreq_save_default_governor(void);
void cpufreq_restore_default_governor(void);
void cpufreq_set_conservative_governor(void);
void cpufreq_set_conservative_governor_param(int up_th, int down_th);

#define SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD 95
#define SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD 50

static char cpufreq_gov_default[32];
static char *cpufreq_gov_conservative = "conservative";
static char *cpufreq_sysfs_place_holder = "/sys/devices/system/cpu/cpu%i/cpufreq/scaling_governor";
static char *cpufreq_gov_conservative_param = "/sys/devices/system/cpu/cpufreq/conservative/%s";

static void cpufreq_set_governor(char *governor)
{
	struct file *scaling_gov = NULL;
	char    buf[128];
	int i;
	loff_t offset = 0;

	if (governor == NULL)
		return;

	for_each_online_cpu(i) {
		sprintf(buf, cpufreq_sysfs_place_holder, i);
		scaling_gov = filp_open(buf, O_RDWR, 0);
		if (scaling_gov != NULL) {
			if (scaling_gov->f_op != NULL &&
				scaling_gov->f_op->write != NULL)
				scaling_gov->f_op->write(scaling_gov,
						governor,
						strlen(governor),
						&offset);
			else
				pr_err("f_op might be null\n");

			filp_close(scaling_gov, NULL);
		} else {
			pr_err("%s. Can't open %s\n", __func__, buf);
		}
	}
}

void cpufreq_save_default_governor(void)
{
	int ret;
	struct cpufreq_policy current_policy;
	ret = cpufreq_get_policy(&current_policy, 0);
	if (ret < 0)
		pr_err("%s: cpufreq_get_policy got error", __func__);
	memcpy(cpufreq_gov_default, current_policy.governor->name, 32);
}

void cpufreq_restore_default_governor(void)
{
	cpufreq_set_governor(cpufreq_gov_default);
}

void cpufreq_set_conservative_governor_param(int up_th, int down_th)
{
	struct file *gov_param = NULL;
	static char buf[128], parm[8];
	loff_t offset = 0;

	if (up_th <= down_th) {
		printk(KERN_ERR "%s: up_th(%d) is lesser than down_th(%d)\n",
			__func__, up_th, down_th);
		return;
	}

	sprintf(parm, "%d", up_th);
	sprintf(buf, cpufreq_gov_conservative_param , "up_threshold");
	gov_param = filp_open(buf, O_RDONLY, 0);
	if (gov_param != NULL) {
		if (gov_param->f_op != NULL &&
			gov_param->f_op->write != NULL)
			gov_param->f_op->write(gov_param,
					parm,
					strlen(parm),
					&offset);
		else
			pr_err("f_op might be null\n");

		filp_close(gov_param, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}

	sprintf(parm, "%d", down_th);
	sprintf(buf, cpufreq_gov_conservative_param , "down_threshold");
	gov_param = filp_open(buf, O_RDONLY, 0);
	if (gov_param != NULL) {
		if (gov_param->f_op != NULL &&
			gov_param->f_op->write != NULL)
			gov_param->f_op->write(gov_param,
					parm,
					strlen(parm),
					&offset);
		else
			pr_err("f_op might be null\n");

		filp_close(gov_param, NULL);
	} else {
		pr_err("%s. Can't open %s\n", __func__, buf);
	}
}

static void cpufreq_early_suspend(struct early_suspend *p)
{
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param(
		SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD,
		SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);
}

static void cpufreq_late_resume(struct early_suspend *p)
{
	cpufreq_restore_default_governor();
}

struct early_suspend cpufreq_earlysuspend = {
	.level = EARLY_SUSPEND_LEVEL_POST_DISABLE_FB,
	.suspend = cpufreq_early_suspend,
	.resume = cpufreq_late_resume,
};

void cpufreq_set_conservative_governor(void)
{
	cpufreq_set_governor(cpufreq_gov_conservative);
}

static int __init cpufreq_on_earlysuspend_init(void)
{
	register_early_suspend(&cpufreq_earlysuspend);
	return 0;
}

static void __exit cpufreq_on_earlysuspend_exit(void)
{
	unregister_early_suspend(&cpufreq_earlysuspend);
}

module_init(cpufreq_on_earlysuspend_init);
module_exit(cpufreq_on_earlysuspend_exit);
