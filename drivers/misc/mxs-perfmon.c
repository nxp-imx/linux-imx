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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <linux/fsl_devices.h>
#include <mach/system.h>
#include "regs-perfmon.h"

#define MONITOR		 "Monitor"

struct mxs_perfmon_cmd_config {
	int field;
	int val;
	const char *cmd;
};

static struct mxs_perfmon_cmd_config
common_perfmon_cmd_config[] = {
	{.val = 1,	.cmd = "start",	.field = BM_PERFMON_CTRL_RUN },
	{.val = 0,	.cmd = "stop",	.field = BM_PERFMON_CTRL_RUN },
	{.val = 1,	.cmd = "fetch",	.field = BM_PERFMON_CTRL_SNAP },
	{.val = 1,	.cmd = "clear",	.field = BM_PERFMON_CTRL_CLR },
	{.val = 1,	.cmd = "read",	.field = BM_PERFMON_CTRL_READ_EN },
	{.val = 0,	.cmd = "write",	.field = BM_PERFMON_CTRL_READ_EN }
};

static struct mxs_perfmon_bit_config
common_perfmon_bit_config[] = {
	{.reg = HW_PERFMON_CTRL,		.name = MONITOR,
	.field = ~0 },
	{.reg = HW_PERFMON_CTRL,		.name = "Trap-En",
	.field = BM_PERFMON_CTRL_TRAP_ENABLE },
	{.reg = HW_PERFMON_CTRL,		.name = "Trap-In-Range",
	.field = BM_PERFMON_CTRL_TRAP_IN_RANGE },
	{.reg = HW_PERFMON_CTRL,		.name = "Latency-En",
	.field = BM_PERFMON_CTRL_LATENCY_ENABLE },
	{.reg = HW_PERFMON_CTRL,		.name = "Trap-IRQ",
	.field = BM_PERFMON_CTRL_TRAP_IRQ },
	{.reg = HW_PERFMON_CTRL,		.name = "Latency-IRQ",
	.field = BM_PERFMON_CTRL_LATENCY_IRQ },
	{.reg = HW_PERFMON_TRAP_ADDR_LOW,	.name = "Trap-Low",
	.field = BM_PERFMON_TRAP_ADDR_LOW_ADDR },
	{.reg = HW_PERFMON_TRAP_ADDR_HIGH,	.name = "Trap-High",
	.field = BM_PERFMON_TRAP_ADDR_HIGH_ADDR },
	{.reg = HW_PERFMON_LAT_THRESHOLD,	.name = "Latency-Threshold",
	.field = BM_PERFMON_LAT_THRESHOLD_VALUE },
	{.reg = HW_PERFMON_ACTIVE_CYCLE,	.name = "Active-Cycle",
	.field = BM_PERFMON_ACTIVE_CYCLE_COUNT },
	{.reg = HW_PERFMON_TRANSFER_COUNT,	.name = "Transfer-count",
	.field = BM_PERFMON_TRANSFER_COUNT_VALUE },
	{.reg = HW_PERFMON_TOTAL_LATENCY,	.name = "Latency-count",
	.field = BM_PERFMON_TOTAL_LATENCY_COUNT },
	{.reg = HW_PERFMON_DATA_COUNT,		.name = "Data-count",
	.field = BM_PERFMON_DATA_COUNT_COUNT },
	{.reg = HW_PERFMON_MAX_LATENCY,		.name = "ABurst",
	.field = BM_PERFMON_MAX_LATENCY_ABURST },
	{.reg = HW_PERFMON_MAX_LATENCY,		.name = "ALen",
	.field = BM_PERFMON_MAX_LATENCY_ALEN },
	{.reg = HW_PERFMON_MAX_LATENCY,		.name = "ASize",
	.field = BM_PERFMON_MAX_LATENCY_ASIZE },
	{.reg = HW_PERFMON_MAX_LATENCY,		.name = "TAGID",
	.field = BM_PERFMON_MAX_LATENCY_TAGID },
	{.reg = HW_PERFMON_MAX_LATENCY,		.name = "Max-Count",
	.field = BM_PERFMON_MAX_LATENCY_COUNT }
};

static struct mxs_platform_perfmon_data common_perfmon_data = {
	.bit_config_tab = common_perfmon_bit_config,
	.bit_config_cnt = ARRAY_SIZE(common_perfmon_bit_config),
};

struct mxs_perfmon_data {
	struct device *dev;
	struct mxs_platform_perfmon_data *pdata;
	struct mxs_platform_perfmon_data *pdata_common;
	int count;
	struct attribute_group attr_group;
	unsigned int base;
	unsigned int initial;
	/* attribute ** follow */
	/* device_attribute follow */
};

#define pd_attribute_ptr(x) \
	((struct attribute **)((x) + 1))
#define pd_device_attribute_ptr(x) \
	((struct device_attribute *)(pd_attribute_ptr(x) + (x)->count + 1))

static inline u32 perfmon_reg_read(struct mxs_perfmon_data *pdata,
						int reg)
{
	return __raw_readl(pdata->base + reg);
}

static inline void perfmon_reg_write(struct mxs_perfmon_data *pdata,
				u32 val, int reg)
{
	__raw_writel(val, pdata->base + reg);
}

static int get_offset_form_field(int field)
{
	int offset = 0;

	if (!field)
		return offset;

	while (!(field & 0x1)) {
		field >>= 1;
		offset++;
	}

	return offset;
}

static ssize_t
perfmon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxs_perfmon_data *pd = platform_get_drvdata(pdev);
	struct device_attribute *devattr = pd_device_attribute_ptr(pd);
	struct mxs_perfmon_bit_config *pb;
	int idx;
	u32 val;
	ssize_t result = 0;

	idx = attr - devattr;
	if ((unsigned int)idx >= pd->count)
		return -EINVAL;

	if (idx < pd->pdata->bit_config_cnt) {
		pb = &pd->pdata->bit_config_tab[idx];
		pb->reg = HW_PERFMON_MASTER_EN;
	} else
		pb = &pd->pdata_common->bit_config_tab \
				[idx - pd->pdata->bit_config_cnt];

	if (!pd->initial) {
		mxs_reset_block((void *)pd->base, true);
		pd->initial = true;
	}

	if (!memcmp(pb->name, MONITOR, sizeof(MONITOR))) {
		/* cat monitor command, we return monitor status */
		val = perfmon_reg_read(pd, pb->reg);

		if (val & BV_PERFMON_CTRL_RUN__RUN)
			result += sprintf(buf, "Run mode\r\n");
		else
			result += sprintf(buf, "Stop mode\r\n");

		if (val & BM_PERFMON_CTRL_READ_EN)
			result += \
			sprintf(buf + result, "PM Read Activities\r\n");
		else
			result += \
			sprintf(buf + result, "PM Write Activities\r\n");

		return result;
	}

	/* read value and shift */
	val = perfmon_reg_read(pd, pb->reg);
	val &= pb->field;
	val >>= get_offset_form_field(pb->field);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t
perfmon_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxs_perfmon_data *pd = platform_get_drvdata(pdev);
	struct device_attribute *devattr = pd_device_attribute_ptr(pd);
	struct mxs_perfmon_bit_config *pb;
	int idx, r;
	unsigned long val, newval;

	idx = attr - devattr;
	if ((unsigned int)idx >= pd->count)
		return -EINVAL;

	if (!buf)
		return -EINVAL;

	if (idx < pd->pdata->bit_config_cnt) {
		pb = &pd->pdata->bit_config_tab[idx];
		pb->reg = HW_PERFMON_MASTER_EN;
	} else
		pb = &pd->pdata_common->bit_config_tab \
		[idx - pd->pdata->bit_config_cnt];

	if (!pd->initial) {
		mxs_reset_block((void *)pd->base, true);
		pd->initial = true;
	}

	if (!memcmp(pb->name, MONITOR, sizeof(MONITOR))) {
		/* it's a cmd */
		int scan, size;
		const struct mxs_perfmon_cmd_config *pcfg;

		size = ARRAY_SIZE(common_perfmon_cmd_config);
		for (scan = 0; scan < size; scan++) {
			pcfg = &common_perfmon_cmd_config[scan];
			if (!memcmp(buf, pcfg->cmd, strlen(pcfg->cmd))) {
				val = perfmon_reg_read(pd, HW_PERFMON_CTRL);
				val &= ~pcfg->field;
				val |= \
				pcfg->val << get_offset_form_field(pcfg->field);
				perfmon_reg_write(pd, val, HW_PERFMON_CTRL);

				return count;
			}
		}
		if (scan == ARRAY_SIZE(common_perfmon_cmd_config))
			return -EINVAL;
	}
	/* get value to write */
	if (buf && (count >= 2) && buf[0] == '0' && buf[1] == 'x')
		r = strict_strtoul(buf, 16, &val);
	else
		r = strict_strtoul(buf, 10, &val);

	if (r != 0)
		return r;

	/* verify it fits */
	if ((unsigned int)val > (pb->field >> get_offset_form_field(pb->field)))
		return -EINVAL;

	newval = perfmon_reg_read(pd, pb->reg);
	newval &= ~pb->field;
	newval |= val << get_offset_form_field(pb->field);
	perfmon_reg_write(pd, newval, pb->reg);

	return count;
}


static int __devinit mxs_perfmon_probe(struct platform_device *pdev)
{
	struct mxs_perfmon_data *pd;
	struct mxs_platform_perfmon_data *pdata;
	struct mxs_platform_perfmon_data *pdata_common;
	struct resource *res;
	struct mxs_perfmon_bit_config *pb;
	struct attribute **attr;
	struct device_attribute *devattr;
	int i, cnt, size;
	int err;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	pdata_common = &common_perfmon_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	cnt = pdata->bit_config_cnt + pdata_common->bit_config_cnt;
	size = sizeof(*pd) +
		 (cnt + 1) * sizeof(struct atrribute *) +
		 cnt * sizeof(struct device_attribute);
	pd = kzalloc(size, GFP_KERNEL);
	if (pd == NULL)
		return -ENOMEM;
	pd->dev = &pdev->dev;
	pd->pdata = pdata;
	pd->pdata_common = pdata_common;
	pd->base =  (unsigned int)ioremap(res->start, res->end - res->start);
	pd->initial = false;

	platform_set_drvdata(pdev, pd);
	pd->count = cnt;
	attr = pd_attribute_ptr(pd);
	devattr = pd_device_attribute_ptr(pd);

	/* build the attributes structures */
	pd->attr_group.attrs = attr;
	pb = pdata->bit_config_tab;
	for (i = 0; i < pdata->bit_config_cnt; i++) {
		devattr[i].attr.name = pb[i].name;
		devattr[i].attr.mode = S_IWUSR | S_IRUGO;
		devattr[i].show = perfmon_show;
		devattr[i].store = perfmon_store;
		attr[i] = &devattr[i].attr;
	}
	pb = pdata_common->bit_config_tab;
	for (i = 0; i < pdata_common->bit_config_cnt; i++) {
		devattr[i + pdata->bit_config_cnt].attr.name = pb[i].name;
		devattr[i + pdata->bit_config_cnt].attr.mode = \
			S_IWUSR | S_IRUGO;
		devattr[i + pdata->bit_config_cnt].show = perfmon_show;
		devattr[i + pdata->bit_config_cnt].store = perfmon_store;
		attr[i + pdata->bit_config_cnt] = \
			&devattr[i + pdata->bit_config_cnt].attr;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &pd->attr_group);
	if (err != 0) {
		platform_set_drvdata(pdev, NULL);
		kfree(pd);
		return err;
	}

	return 0;
}

static int __devexit mxs_perfmon_remove(struct platform_device *pdev)
{
	struct mxs_perfmon_data *pd;

	pd = platform_get_drvdata(pdev);
	sysfs_remove_group(&pdev->dev.kobj, &pd->attr_group);
	platform_set_drvdata(pdev, NULL);
	kfree(pd);

	return 0;
}

#ifdef CONFIG_PM
static int
mxs_perfmon_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mxs_perfmon_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define mxs_perfmon_suspend	NULL
#define	mxs_perfmon_resume	NULL
#endif

static struct platform_driver mxs_perfmon_driver = {
	.probe		= mxs_perfmon_probe,
	.remove		= __exit_p(mxs_perfmon_remove),
	.suspend	= mxs_perfmon_suspend,
	.resume		= mxs_perfmon_resume,
	.driver		= {
		.name   = "mxs-perfmon",
		.owner	= THIS_MODULE,
	},
};

static int __init mxs_perfmon_init(void)
{
	return platform_driver_register(&mxs_perfmon_driver);
}

static void __exit mxs_perfmon_exit(void)
{
	platform_driver_unregister(&mxs_perfmon_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Performance Monitor user-access driver");
MODULE_LICENSE("GPL");

module_init(mxs_perfmon_init);
module_exit(mxs_perfmon_exit);
