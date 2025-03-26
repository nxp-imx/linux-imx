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
#include <linux/version.h>

#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include "trusty-ffa.h"
#include "trusty-sched-share-api.h"
#include "trusty-transport.h"

static struct ffa_driver trusty_ffa_driver;

struct trusty_ffa_state {
	struct device *dev; /* ffa device */
	struct device *core_dev; /* the trusty-core device */
	struct trusty_transport transport;
	struct trusty_sched_share_state *sched_share_state;
};

/* partition property: Supports receipt of direct requests */
#define FFA_PARTITION_DIRECT_REQ_RECV	BIT(0)

static struct trusty_ffa_state *trusty_ffa_get_state(struct trusty_transport *tr)
{
	if (WARN_ON(!tr))
		return NULL;

	return container_of(tr, struct trusty_ffa_state, transport);
}

static unsigned long trusty_ffa_send_msg(struct trusty_ffa_state *s,
					 unsigned long msg, const char *msg_str,
					 unsigned long a0, unsigned long a1,
					 unsigned long a2, unsigned long a3)
{
	struct ffa_device *ffa_dev = to_ffa_dev(s->dev);
	int ret;
	struct ffa_send_direct_data ffa_msg = {
		.data0 = msg,
		.data1 = a0,
		.data2 = a1,
		.data3 = a2,
		.data4 = a3,
	};

	ret = ffa_dev->ops->msg_ops->sync_send_receive(ffa_dev, &ffa_msg);
	if (ret) {
		dev_err(&ffa_dev->dev, "%s: FF-A send %s error (%d)\n",
			__func__, msg_str, ret);
		/* TODO: convert from Linux to sm errors */
		return SM_ERR_INTERNAL_FAILURE;
	}

	return (s32)ffa_msg.data0;
}

static unsigned long trusty_ffa_run(struct trusty_ffa_state *s)
{
	struct ffa_device *ffa_dev = to_ffa_dev(s->dev);
	int this_cpu;
	int ret;

	preempt_disable();
	this_cpu = smp_processor_id();
	ret = ffa_dev->ops->cpu_ops->run(ffa_dev, this_cpu);
	preempt_enable();

	if (ret) {
		dev_err(s->dev, "%s: FF-A run error (%d)\n", __func__, ret);
		/* TODO: convert from Linux to sm errors */
		return SM_ERR_INTERNAL_FAILURE;
	}

	if (!s->sched_share_state) {
		dev_err(s->dev, "%s: no sched state available\n", __func__);
		return SM_ERR_INTERNAL_FAILURE;
	}

	if (trusty_is_idle(this_cpu, s->sched_share_state))
		return SM_ERR_NOP_DONE;
	else
		return SM_ERR_NOP_INTERRUPTED;
}

static unsigned long trusty_ffa_call(struct trusty_transport *tr,
				     unsigned long smcnr, unsigned long a0,
				     unsigned long a1, unsigned long a2)
{
	struct trusty_ffa_state *s = trusty_ffa_get_state(tr);
	int ret;

	if (WARN_ON(!s))
		return SM_ERR_INVALID_PARAMETERS;

	if (SMC_IS_FASTCALL(smcnr))
		return trusty_ffa_send_msg(s, TRUSTY_FFA_MSG_RUN_FASTCALL,
					   "FASTCALL", smcnr, a0, a1, a2);

	if (smcnr == SMC_SC_NOP) {
		if (!a0)
			return trusty_ffa_run(s);

		ret = trusty_ffa_send_msg(s, TRUSTY_FFA_MSG_RUN_NOPCALL,
					  "NOPCALL", a0, a1, a2, 0);
		/*
		 * Trusty should enqueue a NOP here by sending
		 * an IPI to the primary scheduler. The error code
		 * returned here is irrelevant, since the interrupt
		 * should be handled by the time dequeue_nop returns
		 * in nop_work_func, so do_nop there should always be true.
		 */
		return ret ?: SM_ERR_NOP_DONE;
	}

	if (smcnr != SMC_SC_RESTART_LAST) {
		ret = trusty_ffa_send_msg(s, TRUSTY_FFA_MSG_QUEUE_STDCALL,
					  "QUEUE_STDCALL", smcnr, a0, a1, a2);
		/*
		 * Trusty should enqueue a NOP here by sending
		 * an IPI to the primary scheduler. We are running with
		 * interrupts disabled so the handler has not run yet.
		 * Return all the way to trusty_std_call32 so the
		 * NOP thread gets to run. The next time we enter this
		 * function, smcnr will be SMC_SC_RESTART_LAST and
		 * we retrieve the stdcall return code then.
		 */
		return ret ?: SM_ERR_CPU_IDLE;
	}

	/* We got SMC_SC_RESTART_LAST, try to get the result of the last call */
	return trusty_ffa_send_msg(s, TRUSTY_FFA_MSG_GET_STDCALL_RET,
				   "GET_STDCALL_RET", 0, 0, 0, 0);
}

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

static int trusty_ffa_share_or_lend_memory(struct trusty_transport *tr, u64 *id,
					   struct scatterlist *sglist,
					   unsigned int nents, pgprot_t pgprot,
					   u64 tag, bool lend, struct ns_mem_page_info *pg_inf)
{
	struct trusty_ffa_state *s = trusty_ffa_get_state(tr);

	if (WARN_ON(!s))
		return -EINVAL;

	return trusty_ffa_dev_share_or_lend_memory(s->dev, id, sglist, nents,
						   pgprot, tag, lend, pg_inf);
}

static int trusty_ffa_reclaim_memory(struct trusty_transport *tr, u64 id,
				     struct scatterlist *sglist,
				     unsigned int nents)
{
	struct trusty_ffa_state *s = trusty_ffa_get_state(tr);

	if (WARN_ON(!s))
		return -EINVAL;

	return trusty_ffa_dev_reclaim_memory(s->dev, id, sglist, nents);
}

static void trusty_ffa_set_sched_share_state(struct trusty_transport *tr,
					     struct trusty_sched_share_state *tsss)
{
	struct trusty_ffa_state *s = trusty_ffa_get_state(tr);

	if (WARN_ON(!s))
		return;

	s->sched_share_state = tsss;
}

static const struct trusty_transport_ops trusty_ffa_transport_ops = {
	.call = &trusty_ffa_call,
	.share_or_lend_memory = &trusty_ffa_share_or_lend_memory,
	.reclaim_memory = &trusty_ffa_reclaim_memory,
	.set_sched_share_state = &trusty_ffa_set_sched_share_state,
};

static void trusty_ffa_sched_recv_cb(u16 vcpu, bool is_per_vcpu, void *cb_data)
{
	struct trusty_ffa_state *s = cb_data;

	if (!s->core_dev)
		return;

	if (is_per_vcpu)
		trusty_enqueue_nop_on_cpu(s->core_dev, NULL, vcpu);
	else
		trusty_enqueue_nop(s->core_dev, NULL);
}

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
	s->transport.magic = TRUSTY_TRANSPORT_MAGIC;
	s->transport.ops = &trusty_ffa_transport_ops;

	ffa_dev_set_drvdata(ffa_dev, s);

	ffa_dev->ops->msg_ops->mode_32bit_set(ffa_dev);

	ret = ffa_dev->ops->notifier_ops->sched_recv_cb_register(
		ffa_dev, trusty_ffa_sched_recv_cb, s);
	if (ret < 0) {
		dev_err(s->dev, "failed to register SRI callback (%d)\n",
			ret);

		/*
		 * For now, we log and continue if notifications are not
		 * supported because not all current SPMCs have them,
		 * e.g., EL3-SPMC.
		 */
		if (ret != -EOPNOTSUPP)
			goto err_ffa_sched_recv_cb;
	}

	return 0;

err_ffa_sched_recv_cb:
	ffa_dev_set_drvdata(ffa_dev, NULL);
	kfree(s);
err_alloc:
err_ffa_version:
	return ret;
}

static
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
int
#else
void
#endif
trusty_ffa_remove(struct ffa_device *ffa_dev)
{
	struct trusty_ffa_state *s = ffa_dev_get_drvdata(ffa_dev);

	/*
	 * The device links we add should guarantee that all
	 * consumers of this device will get removed first.
	 */
	memzero_explicit(s, sizeof(struct trusty_ffa_state));
	kfree(s);
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
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

	/* currently only one trusty instance is probed */
	ffa_dev = bus_find_device(&ffa_bus_type, NULL, &trusty_ffa_device_id[0].uuid,
				  trusty_ffa_dev_match);

	if (ffa_dev) {
		if (bus_find_device(&ffa_bus_type, ffa_dev, &trusty_ffa_device_id[0].uuid,
				    trusty_ffa_dev_match))
			dev_warn(ffa_dev, "multiple Trusty instances found, not supported yet");
	}

	return ffa_dev ?: ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(trusty_ffa_find_device);

static int trusty_ffa_remove_child(struct device *dev, void *data)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static struct ffa_driver trusty_ffa_driver = {
	.name = "trusty-ffa",
	.probe = trusty_ffa_probe,
	.remove = trusty_ffa_remove,
	.id_table = trusty_ffa_device_id,
};

static int trusty_ffa_platform_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev_ffa;
	struct ffa_device *ffa_dev;
	struct device_link *link;
	char trusty_uuid[UUID_STRING_LEN + 1];
	struct ffa_partition_info pinfo = { 0 };
	struct trusty_ffa_state *s;

	dev_ffa = trusty_ffa_find_device();
	if (IS_ERR(dev_ffa)) {
		dev_err(&pdev->dev, "FF-A device error (%ld)\n",
			PTR_ERR(dev_ffa));
		rc = PTR_ERR(dev_ffa);
		goto err_find_ffa_device;
	} else if (WARN_ON(!dev_ffa)) {
		rc = -EINVAL;
		goto err_find_ffa_device;
	}

	/*
	 * Add a link from our device to the FF-A one so
	 * the latter does not get removed while we're using it
	 */
	link = device_link_add(&pdev->dev, dev_ffa,
			       DL_FLAG_AUTOPROBE_CONSUMER);
	if (!link) {
		dev_err(&pdev->dev, "failed to add device link\n");
		rc = -EINVAL;
		goto err_link_device;
	}

	/* Supplier has not been probed yet */
	if (link->status == DL_STATE_DORMANT) {
		rc = -EPROBE_DEFER;
		goto err_defer_probe;
	}

	/* check if Trusty partition can support receipt of direct requests. */
	ffa_dev = to_ffa_dev(dev_ffa);
	snprintf(trusty_uuid, sizeof(trusty_uuid), "%pU",
		 &trusty_ffa_device_id[0].uuid);
	rc = ffa_dev->ops->info_ops->partition_info_get(trusty_uuid, &pinfo);
	if (rc < 0)
		goto err_get_partition_info;
	if (!(pinfo.properties & FFA_PARTITION_DIRECT_REQ_RECV)) {
		rc = -ENODEV;
		goto err_check_partition_info;
	}

	s = ffa_dev_get_drvdata(ffa_dev);
	if (WARN_ON(dev_ffa != s->dev)) {
		rc = -EINVAL;
		goto err_invalid_state;
	}
	if (WARN_ON(s->core_dev)) {
		rc = -EINVAL;
		goto err_invalid_state;
	}

	platform_set_drvdata(pdev, &s->transport);

	if (pdev->dev.of_node) {
		rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
		if (rc < 0) {
			dev_err(&pdev->dev, "Failed to add children: %d\n", rc);
			goto err_add_children;
		}
	} else {
		dev_warn(&pdev->dev, "of_node not found\n");
	}

	/*
	 * Depending on how it was created, the core device is either
	 * named trusty-ffa:trusty-core (from of_platform_populate)
	 * or just trusty-core if we create it ourselves
	 */
	s->core_dev = device_find_child_by_name(&pdev->dev, "trusty-ffa:trusty-core");
	if (!s->core_dev)
		s->core_dev = device_find_child_by_name(&pdev->dev, "trusty-core");

	return 0;

err_add_children:
err_invalid_state:
err_check_partition_info:
err_get_partition_info:
err_defer_probe:
err_link_device:
	put_device(dev_ffa);
err_find_ffa_device:
	return rc;
}

static void trusty_ffa_platform_remove(struct platform_device *pdev)
{
	struct trusty_ffa_state *s = trusty_ffa_get_state(platform_get_drvdata(pdev));

	device_for_each_child(&pdev->dev, NULL, trusty_ffa_remove_child);

	put_device(s->dev);
	put_device(s->core_dev);
	s->core_dev = NULL;
}

static const struct of_device_id trusty_ffa_of_match[] = {
	{ .compatible = "android,trusty-ffa-v1", },
	{},
};

MODULE_DEVICE_TABLE(trusty_ffa, trusty_ffa_of_match);

static struct platform_driver trusty_ffa_platform_driver = {
	.probe = trusty_ffa_platform_probe,
	.remove = trusty_ffa_platform_remove,
	.driver	= {
		.name = "trusty-ffa",
		.of_match_table = trusty_ffa_of_match,
	},
};

int __init trusty_ffa_init(void)
{
	int rc;

	if (!IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return -ENODEV;

	rc = ffa_register(&trusty_ffa_driver);
	if (rc < 0)
		goto err_register_ffa;

	rc = platform_driver_register(&trusty_ffa_platform_driver);
	if (rc < 0)
		goto err_register_driver;

	return 0;

err_register_driver:
	platform_driver_unregister(&trusty_ffa_platform_driver);
err_register_ffa:
	return rc;
}

void trusty_ffa_exit(void)
{
	if (!IS_REACHABLE(CONFIG_ARM_FFA_TRANSPORT))
		return;

	platform_driver_unregister(&trusty_ffa_platform_driver);
	ffa_unregister(&trusty_ffa_driver);
}

subsys_initcall(trusty_ffa_init);
module_exit(trusty_ffa_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty FF-A transport driver");
