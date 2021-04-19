// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 */

#include <linux/export.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/pm-trace.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/suspend.h>
#include <linux/syscalls.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>

/**
 * lpa - control Low-Power Audio (LPA) mode.
 *
 * show() returns current LPA state, which may be either:
 *   - "0" (disabled), or
 *   - "1" (enabled).
 *
 * store() accepts one of those strings,
 * and enable or disable LPA flag in ATF accordingly.
 */

#define IMX_SRC_BASE   0x30390000
#define SRC_GPR9       0x00000098
#define LPA_MASK       0x00000F00
#define LPA_CLEAR_MASK 0xFFFFF0FF
#define LPA_ENABLED    0x00000500

void __iomem *src_gpr9_reg;

void lpa_enable(bool enable)
{
       uint32_t val;

       /* Retrieve SRC GPR9 register content */
       val = readl(src_gpr9_reg);
       /* Clear LPA Flag */
       val &= LPA_CLEAR_MASK;
       /* Update content */
       if (enable) {
               val |= LPA_ENABLED;
               pr_info("LPA: enabled\n");
       } else {
               pr_info("LPA: disabled\n");
       }
       writel(val, src_gpr9_reg);
}

bool lpa_is_enabled(void)
{
       uint32_t val;

       /* Retrieve SRC GPR9 register content */
       val = readl(src_gpr9_reg) & LPA_MASK;

       return (val == LPA_ENABLED);
}

static ssize_t lpa_show(struct kobject *kobj, struct kobj_attribute *attr,
                       char *buf)
{
       return sprintf(buf, "%u\n", lpa_is_enabled());
}

static ssize_t lpa_store(struct kobject *kobj, struct kobj_attribute *attr,
                        const char *buf, size_t n)
{
       unsigned long val;

       if (kstrtoul(buf, 10, &val))
               return -EINVAL;
       if (val > 1)
               return -EINVAL;

       lpa_enable(val);

       return n;
}

static struct kobj_attribute lpa_ctrl_attr=
	__ATTR(lpa, 0644, lpa_show, lpa_store);

static struct attribute * lpa_ctrl_attrbute[] = {
	&lpa_ctrl_attr.attr,
	NULL,
};

static const struct attribute_group lpa_ctrl_attr_group = {
	.attrs = lpa_ctrl_attrbute,
};

static const struct attribute_group *lpa_ctrl_attr_groups[] = {
	&lpa_ctrl_attr_group,
	NULL,
};

static int __init lpa_ctrl_init(void)
{
	int error;

	struct kobject * lpa_ctrl_kobj = kobject_create_and_add("lpa_ctrl", NULL);
	if (!lpa_ctrl_kobj)
		return -ENOMEM;
	error = sysfs_create_groups(lpa_ctrl_kobj, lpa_ctrl_attr_groups);
	if (error)
		return error;

    /* Map SRC GPR9 register */
    src_gpr9_reg = ioremap(IMX_SRC_BASE + SRC_GPR9, sizeof(uint32_t));

	return 0;
}

device_initcall(lpa_ctrl_init);
MODULE_LICENSE("GPL v2");
