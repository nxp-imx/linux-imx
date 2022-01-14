// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 ARM Ltd.
 * Copyright (C) 2024 Google, Inc.
 */

#include <linux/kconfig.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/trusty/smcall.h>
#include <linux/arm_ffa.h>
#include <linux/trusty/trusty.h>

#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include "trusty-ffa.h"
#include "trusty-transport.h"

struct trusty_ffa_state {
	struct device *dev; /* ffa device */
};

int trusty_ffa_dev_share_or_lend_memory(struct device *dev, u64 *id,
					struct scatterlist *sglist,
					unsigned int nents, pgprot_t pgprot,
					u64 tag, bool lend, struct ns_mem_page_info *pg_inf)
{
	struct ffa_device *ffa_dev = to_ffa_dev(dev);
	int ret;
	struct ffa_mem_region_attributes ffa_mem_attr;
	struct ffa_mem_ops_args ffa_mem_args;

	ffa_mem_attr.receiver = ffa_dev->vm_id;
	ffa_mem_attr.attrs = pg_inf->ffa_mem_perm;

	ffa_mem_args.use_txbuf = 1;
	ffa_mem_args.tag = tag;
	ffa_mem_args.attrs = &ffa_mem_attr;
	ffa_mem_args.nattrs = 1;
	ffa_mem_args.sg = sglist;
	ffa_mem_args.flags = 0;

	if (lend)
		ret = ffa_dev->ops->mem_ops->memory_lend(&ffa_mem_args);
	else
		ret = ffa_dev->ops->mem_ops->memory_share(&ffa_mem_args);

	if (ret) {
		dev_err(dev, "memory %s failed %d", lend ? "lend" : "share", ret);
		return ret;
	}

	*id = ffa_mem_args.g_handle;
	return 0;
}
EXPORT_SYMBOL(trusty_ffa_dev_share_or_lend_memory);

int trusty_ffa_dev_reclaim_memory(struct device *dev, u64 id,
				  struct scatterlist *sglist,
				  unsigned int nents)
{
	struct ffa_device *ffa_dev = to_ffa_dev(dev);

	return ffa_dev->ops->mem_ops->memory_reclaim(id, 0);
}
EXPORT_SYMBOL(trusty_ffa_dev_reclaim_memory);

static int trusty_ffa_probe(struct ffa_device *ffa_dev)
{
	struct trusty_ffa_state *s;
	int ret;
	u32 ffa_drv_version;

	/* check ffa driver version compatibility */
	ffa_drv_version = ffa_dev->ops->info_ops->api_version_get();
	if (TO_TRUSTY_FFA_MAJOR(ffa_drv_version) != TRUSTY_FFA_VERSION_MAJOR ||
	    TO_TRUSTY_FFA_MINOR(ffa_drv_version) < TRUSTY_FFA_VERSION_MINOR) {
		ret = -EINVAL;
		goto err_ffa_version;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	s->dev = &ffa_dev->dev;
	ffa_dev_set_drvdata(ffa_dev, s);

	ffa_dev->ops->msg_ops->mode_32bit_set(ffa_dev);

	return 0;

err_alloc:
err_ffa_version:
	return ret;
}

static void trusty_ffa_remove(struct ffa_device *ffa_dev)
{
	struct trusty_ffa_state *s = ffa_dev_get_drvdata(ffa_dev);

	memzero_explicit(s, sizeof(struct trusty_ffa_state));
	kfree(s);
}

static const struct ffa_device_id trusty_ffa_device_id[] = {
	/*
	 * Trusty UUID: RFC-4122 compliant UUID version 4
	 * 40ee25f0-a2bc-304c-8c4ca173c57d8af1
	 */
	{ UUID_INIT(0x40ee25f0, 0xa2bc, 0x304c,
		    0x8c, 0x4c, 0xa1, 0x73, 0xc5, 0x7d, 0x8a, 0xf1) },
	{}
};

static int trusty_ffa_dev_match(struct device *dev, const void *uuid)
{
	struct ffa_device *ffa_dev;

	ffa_dev = to_ffa_dev(dev);
	if (uuid_equal(&ffa_dev->uuid, uuid))
		return 1;

	return 0;
}

struct device *trusty_ffa_find_device(void)
{
	struct device *ffa_dev;
	const struct bus_type *ffa_bus = symbol_get(ffa_bus_type);

	if (!ffa_bus)
		return ERR_PTR(-ENOENT);

	/* currently only one trusty instance is probed */
	ffa_dev = bus_find_device(ffa_bus, NULL, &trusty_ffa_device_id[0].uuid,
				  trusty_ffa_dev_match);

	if (ffa_dev) {
		if (bus_find_device(ffa_bus, ffa_dev, &trusty_ffa_device_id[0].uuid,
				    trusty_ffa_dev_match))
			dev_warn(ffa_dev, "multiple Trusty instances found, not supported yet");
	}

	symbol_put(ffa_bus_type);
	return ffa_dev ?: ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(trusty_ffa_find_device);

static struct ffa_driver trusty_ffa_driver = {
	.name = "trusty-ffa",
	.probe = trusty_ffa_probe,
	.remove = trusty_ffa_remove,
	.id_table = trusty_ffa_device_id,
};

int __init trusty_ffa_init(void)
{
	if (!IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return -ENODEV;

	return ffa_register(&trusty_ffa_driver);
}

void trusty_ffa_exit(void)
{
	if (!IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return;

	ffa_unregister(&trusty_ffa_driver);
}

subsys_initcall(trusty_ffa_init);
module_exit(trusty_ffa_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty FF-A transport driver");
