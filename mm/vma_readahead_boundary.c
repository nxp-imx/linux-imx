// SPDX-License-Identifier: GPL-2.0
/*
 * Static key and sysfs file controlling VMA readahead boundary.
 *
 * Copyright (c) 2026, Google LLC.
 * Author: Frederick Mayle <fmayle@google.com>
 */

#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/kobject.h>
#include <linux/kstrtox.h>
#include <linux/mm.h>
#include <linux/sysfs.h>
#include <linux/vma_readahead_boundary.h>

DEFINE_STATIC_KEY_FALSE(vma_readahead_boundary_enabled);

static ssize_t enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sysfs_emit(buf, "%d\n", is_vma_readahead_boundary_enabled());
}

static ssize_t enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	bool enabled;

	if (kstrtobool(buf, &enabled))
		return -EINVAL;

	if (enabled)
		static_branch_enable(&vma_readahead_boundary_enabled);
	else
		static_branch_disable(&vma_readahead_boundary_enabled);

	return n;
}

static struct kobj_attribute enabled_attr = __ATTR_RW(enabled);

static struct attribute *vma_readahead_boundary_attrs[] = {
	&enabled_attr.attr,
	NULL
};

static struct attribute_group vma_readahead_boundary_attr_group = {
	.name = "vma_readahead_boundary",
	.attrs = vma_readahead_boundary_attrs,
};

/**
 * What:          /sys/kernel/mm/vma_readahead_boundary/enabled
 * Date:          April 2026
 * KernelVersion: v6.12+ (GKI kernels)
 * Contact:       Frederick Mayle <fmayle@google.com>
 * Description:   /sys/kernel/mm/vma_readahead_boundary/enabled
 *		  controls whether readahead triggered by mmap accesses will be
 *		  limited to the bounds of the accessed VMA.
 * Users:         n/a
 */
static int __init init_vma_readahead_boundary(void)
{
	if (sysfs_create_group(mm_kobj, &vma_readahead_boundary_attr_group))
		pr_err("vma_readahead_boundary: failed to create sysfs group\n");

	return 0;
};
late_initcall(init_vma_readahead_boundary);
