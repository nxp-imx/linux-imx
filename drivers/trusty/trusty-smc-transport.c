// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Google, Inc.
 */

#include <linux/arm-smccc.h>
#include <linux/kconfig.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/trusty/arm_ffa.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>
#include <linux/version.h>

#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include "trusty-ffa.h"
#include "trusty-smc.h"
#include "trusty-transport.h"

static struct platform_driver trusty_smc_driver;

struct trusty_smc_state {
	struct device *dev;
	struct device_link *ffa_dev_link;
	struct trusty_transport transport;
	struct device *core_dev;
	void *ffa_tx;
	void *ffa_rx;
	u16 ffa_local_id;
	u16 ffa_remote_id;
	struct mutex share_memory_msg_lock; /* protects share_memory_msg */
};

static struct trusty_smc_state *trusty_smc_get_state(struct trusty_transport *tr)
{
	if (WARN_ON(!tr))
		return NULL;

	return container_of(tr, struct trusty_smc_state, transport);
}

static unsigned long trusty_smc_call(struct trusty_transport *tr, unsigned long smcnr,
				     unsigned long a0, unsigned long a1,
				     unsigned long a2)
{
	return trusty_smc8(smcnr, a0, a1, a2, 0, 0, 0, 0).r0;
}

static int trusty_smc_share_or_lend_memory(struct trusty_transport *tr, u64 *id,
					   struct scatterlist *sglist, unsigned int nents,
					   pgprot_t pgprot, u64 tag, bool lend,
					   struct ns_mem_page_info *pg_inf)
{
	struct trusty_smc_state *s = trusty_smc_get_state(tr);
	int ret = 0;
	struct scatterlist *sg;
	size_t count;
	size_t i;
	size_t len = 0;
	u64 ffa_handle = 0;
	size_t total_len;
	size_t endpoint_count = 1;
	struct ffa_mtd *mtd = s->ffa_tx;
	size_t comp_mrd_offset = offsetof(struct ffa_mtd, emad[endpoint_count]);
	struct ffa_comp_mrd *comp_mrd = s->ffa_tx + comp_mrd_offset;
	struct ffa_cons_mrd *cons_mrd = comp_mrd->address_range_array;
	size_t cons_mrd_offset = (void *)cons_mrd - s->ffa_tx;
	struct smc_ret8 smc_ret;
	u32 cookie_low;
	u32 cookie_high;

	if (WARN_ON(!s))
		return -EINVAL;

#if IS_ENABLED(CONFIG_TRUSTY_SMC_TRANSPORT_USE_FFA_TRANSPORT)
	if (s->ffa_dev_link)
		return trusty_ffa_dev_share_or_lend_memory(s->ffa_dev_link->supplier, id, sglist,
							   nents, pgprot, tag, lend, pg_inf);
#endif

	if (WARN_ON(!s->ffa_tx))
		return -EOPNOTSUPP;

	len = 0;
	for_each_sg(sglist, sg, nents, i)
		len += sg_dma_len(sg);

	mutex_lock(&s->share_memory_msg_lock);

	mtd->sender_id = s->ffa_local_id;
	mtd->memory_region_attributes = pg_inf->ffa_mem_attr;
	mtd->reserved_3 = 0;
	mtd->flags = 0;
	mtd->handle = 0;
	mtd->tag = tag;
	mtd->reserved_24_27 = 0;
	mtd->emad_count = endpoint_count;
	for (i = 0; i < endpoint_count; i++) {
		struct ffa_emad *emad = &mtd->emad[i];
		/* TODO: support stream ids */
		emad->mapd.endpoint_id = s->ffa_remote_id;
		emad->mapd.memory_access_permissions = pg_inf->ffa_mem_perm;
		emad->mapd.flags = 0;
		emad->comp_mrd_offset = comp_mrd_offset;
		emad->reserved_8_15 = 0;
	}
	comp_mrd->total_page_count = len / FFA_PAGE_SIZE;
	comp_mrd->address_range_count = nents;
	comp_mrd->reserved_8_15 = 0;

	total_len = cons_mrd_offset + nents * sizeof(*cons_mrd);
	sg = sglist;
	count = nents;
	while (count) {
		size_t lcount =
			min_t(size_t, count, (PAGE_SIZE - cons_mrd_offset) /
			      sizeof(*cons_mrd));
		size_t fragment_len = lcount * sizeof(*cons_mrd) +
				      cons_mrd_offset;

		for (i = 0; i < lcount; i++) {
			cons_mrd[i].address = sg_dma_address(sg);
			cons_mrd[i].page_count = sg_dma_len(sg) / FFA_PAGE_SIZE;
			cons_mrd[i].reserved_12_15 = 0;
			sg = sg_next(sg);
		}
		count -= lcount;
		if (cons_mrd_offset) {
			u32 smc = lend ? SMC_FC_FFA_MEM_LEND :
					 SMC_FC_FFA_MEM_SHARE;
			/* First fragment */
			smc_ret = trusty_smc8(smc, total_len,
					      fragment_len, 0, 0, 0, 0, 0);
		} else {
			smc_ret = trusty_smc8(SMC_FC_FFA_MEM_FRAG_TX,
					      cookie_low, cookie_high,
					      fragment_len, 0, 0, 0, 0);
		}
		if ((u32)smc_ret.r0 == SMC_FC_FFA_MEM_FRAG_RX) {
			cookie_low = smc_ret.r1;
			cookie_high = smc_ret.r2;
			dev_dbg(s->dev, "cookie %x %x", cookie_low,
				cookie_high);
			if (!count) {
				/*
				 * We have sent all our descriptors. Expected
				 * SMC_FC_FFA_SUCCESS, not a request to send
				 * another fragment.
				 */
				dev_err(s->dev, "%s: fragment_len %zd/%zd, unexpected SMC_FC_FFA_MEM_FRAG_RX\n",
					__func__, fragment_len, total_len);
				ret = -EIO;
				break;
			}
		} else if ((u32)smc_ret.r0 == SMC_FC_FFA_SUCCESS) {
			ffa_handle = (u64)(u32)smc_ret.r2 | (u64)(u32)smc_ret.r3 << 32;
			dev_dbg(s->dev, "%s: fragment_len %zu/%zu, got handle 0x%llx\n",
				__func__, fragment_len, total_len,
				ffa_handle);
			if (count) {
				/*
				 * We have not sent all our descriptors.
				 * Expected SMC_FC_FFA_MEM_FRAG_RX not
				 * SMC_FC_FFA_SUCCESS.
				 */
				dev_err(s->dev, "%s: fragment_len %zu/%zu, unexpected SMC_FC_FFA_SUCCESS, count %zu != 0\n",
					__func__, fragment_len, total_len,
					count);
				ret = -EIO;
				break;
			}
		} else {
			dev_err(s->dev, "%s: fragment_len %zu/%zu, SMC_FC_FFA_MEM_SHARE failed 0x%lx 0x%lx 0x%lx",
				__func__, fragment_len, total_len,
				smc_ret.r0, smc_ret.r1, smc_ret.r2);
			ret = -EIO;
			break;
		}

		cons_mrd = s->ffa_tx;
		cons_mrd_offset = 0;
	}

	mutex_unlock(&s->share_memory_msg_lock);

	if (!ret) {
		*id = ffa_handle;
		goto done;
	}

	dev_err(s->dev, "%s: failed %d", __func__, ret);

done:
	return ret;
}

static int trusty_smc_reclaim_memory(struct trusty_transport *tr, u64 id,
				     struct scatterlist *sglist, unsigned int nents)
{
	struct trusty_smc_state *s = trusty_smc_get_state(tr);
	int ret = 0;
	struct smc_ret8 smc_ret;

	if (WARN_ON(!s))
		return -EINVAL;

#if IS_ENABLED(CONFIG_TRUSTY_SMC_TRANSPORT_USE_FFA_TRANSPORT)
	if (s->ffa_dev_link)
		return trusty_ffa_dev_reclaim_memory(s->ffa_dev_link->supplier, id, sglist, nents);
#endif

	mutex_lock(&s->share_memory_msg_lock);

	smc_ret = trusty_smc8(SMC_FC_FFA_MEM_RECLAIM, (u32)id, id >> 32, 0, 0,
			      0, 0, 0);
	if ((u32)smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FC_FFA_MEM_RECLAIM failed 0x%lx 0x%lx 0x%lx",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		if ((u32)smc_ret.r0 == SMC_FC_FFA_ERROR &&
		    (s32)smc_ret.r2 == FFA_ERROR_DENIED)
			ret = -EBUSY;
		else
			ret = -EIO;
	}

	mutex_unlock(&s->share_memory_msg_lock);
	return ret;
}

static bool trusty_ret_is_ffa_not_supported(struct smc_ret8 *ret)
{
	/*
	 * The return code can be returned in X0 or W0. On ARM64, we always
	 * read the value from X0; calling a 32-bit SMC will put the lower
	 * 32 bits in that register, and leave the high 32 bits undefined.
	 * We convert to a 32-bit value explicitly since we're checking for -1.
	 * The same applies to X2/W2 for the FF-A error code.
	 */
	return (s32)ret->r0 == SMCCC_RET_NOT_SUPPORTED ||
	    ((u32)ret->r0 == SMC_FC_FFA_ERROR &&
	     (s32)ret->r2 == FFA_ERROR_NOT_SUPPORTED);
}

static int trusty_init_msg_buf(struct trusty_smc_state *s)
{
	phys_addr_t tx_paddr;
	phys_addr_t rx_paddr;
	int ret;
	struct smc_ret8 smc_ret;

	/* The FF-A driver owns the buffers */
	if (s->ffa_dev_link)
		return 0;

	/* Get supported FF-A version and check if it is compatible */
	smc_ret = trusty_smc8(SMC_FC_FFA_VERSION, FFA_CURRENT_VERSION, 0, 0,
			      0, 0, 0, 0);
	if (FFA_VERSION_TO_MAJOR(smc_ret.r0) != FFA_CURRENT_VERSION_MAJOR) {
		dev_err(s->dev,
			"%s: Unsupported FF-A version 0x%lx, expected 0x%x\n",
			__func__, smc_ret.r0, FFA_CURRENT_VERSION);
		ret = trusty_ret_is_ffa_not_supported(&smc_ret) ? 0 : -EIO;
		goto err_version;
	}

	/* Check that SMC_FC_FFA_MEM_SHARE is implemented */
	smc_ret = trusty_smc8(SMC_FC_FFA_FEATURES, SMC_FC_FFA_MEM_SHARE, 0, 0,
			      0, 0, 0, 0);
	if ((u32)smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev,
			"%s: SMC_FC_FFA_FEATURES(SMC_FC_FFA_MEM_SHARE) failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = trusty_ret_is_ffa_not_supported(&smc_ret) ? 0 : -EIO;
		goto err_features;
	}

	/*
	 * Set FF-A endpoint IDs.
	 *
	 * Hardcode 0x8000 for the secure os.
	 * TODO: Use FF-A call or device tree to configure this dynamically
	 */
	smc_ret = trusty_smc8(SMC_FC_FFA_ID_GET, 0, 0, 0, 0, 0, 0, 0);
	if ((u32)smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev,
			"%s: SMC_FC_FFA_ID_GET failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = -EIO;
		goto err_id_get;
	}

	s->ffa_local_id = smc_ret.r2;
	s->ffa_remote_id = 0x8000;

	/*
	 * The pKVM hypervisor uses the same page size as the host, including for
	 * stage-2 mappings. So the rx/tx buffers need to be page-sized multiple,
	 * and page-aligned.
	 *
	 * TODO: This can be made more generic by discovering the required size
	 * through SMC_FC_FFA_FEATURES later.
	 */
	s->ffa_tx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_tx) {
		ret = -ENOMEM;
		goto err_alloc_tx;
	}
	tx_paddr = virt_to_phys(s->ffa_tx);
	if (WARN_ON(tx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_unaligned_tx_buf;
	}

	s->ffa_rx = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!s->ffa_rx) {
		ret = -ENOMEM;
		goto err_alloc_rx;
	}
	rx_paddr = virt_to_phys(s->ffa_rx);
	if (WARN_ON(rx_paddr & (PAGE_SIZE - 1))) {
		ret = -EINVAL;
		goto err_unaligned_rx_buf;
	}

	smc_ret = trusty_smc8(SMC_FCZ_FFA_RXTX_MAP, tx_paddr, rx_paddr,
			      PAGE_SIZE / FFA_PAGE_SIZE, 0, 0, 0, 0);
	if ((u32)smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FCZ_FFA_RXTX_MAP failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
		ret = -EIO;
		goto err_rxtx_map;
	}

	return 0;

err_rxtx_map:
err_unaligned_rx_buf:
	kfree(s->ffa_rx);
	s->ffa_rx = NULL;
err_alloc_rx:
err_unaligned_tx_buf:
	kfree(s->ffa_tx);
	s->ffa_tx = NULL;
err_alloc_tx:
err_id_get:
err_features:
err_version:
	return ret;
}

static void trusty_free_msg_buf(struct trusty_smc_state *s)
{
	struct smc_ret8 smc_ret;

	/*
	 * If we got NOT_SUPPORTED earlier,
	 * or the FF-A driver owns the buffers,
	 * there is nothing to free here.
	 */
	if (!s->ffa_tx)
		return;

	smc_ret = trusty_smc8(SMC_FC_FFA_RXTX_UNMAP, 0, 0, 0, 0, 0, 0, 0);
	if ((u32)smc_ret.r0 != SMC_FC_FFA_SUCCESS) {
		dev_err(s->dev, "%s: SMC_FC_FFA_RXTX_UNMAP failed 0x%lx 0x%lx 0x%lx\n",
			__func__, smc_ret.r0, smc_ret.r1, smc_ret.r2);
	} else {
		kfree(s->ffa_rx);
		kfree(s->ffa_tx);
	}
}

static const struct trusty_transport_ops trusty_smc_transport_ops = {
	.call = &trusty_smc_call,
	.share_or_lend_memory = &trusty_smc_share_or_lend_memory,
	.reclaim_memory = &trusty_smc_reclaim_memory,
};

#if IS_ENABLED(CONFIG_TRUSTY_SMC_TRANSPORT_USE_FFA_TRANSPORT)
static struct device_link *trusty_smc_get_ffa_link(struct platform_device *pdev)
{
	struct device *ffa_dev;
	struct device_link *link;

	ffa_dev = trusty_ffa_find_device();
	if (ffa_dev == ERR_PTR(-ENOENT)) {
		/*
		 * ffa-core.ko is not loaded, fall back to builtin code.
		 * We would also check for ffa-module.ko but it does
		 * not export any symbols that we can check with symbol_get()
		 * so we assume that whoever loads ffa-core.ko will also
		 * load ffa-module.ko after that but before we get here.
		 */
		return NULL;
	} else if (IS_ERR(ffa_dev)) {
		dev_err(&pdev->dev, "Trusty FF-A partition error (%ld)\n",
			PTR_ERR(ffa_dev));
		return ERR_CAST(ffa_dev);
	} else if (WARN_ON(!ffa_dev)) {
		return ERR_PTR(-EINVAL);
	}

	/*
	 * The FF-A device was successfully created,
	 * since ffa_setup_partitions creates the devices
	 * from the ffa-module init function, but the new
	 * devices might not have been probed yet.
	 */
	link = device_link_add(&pdev->dev, ffa_dev,
			       DL_FLAG_AUTOPROBE_CONSUMER);
	put_device(ffa_dev);

	if (!link) {
		dev_err(&pdev->dev, "failed to add device link\n");
		return ERR_PTR(-EINVAL);
	}

	/* Supplier has not been probed yet */
	if (link->status == DL_STATE_DORMANT)
		return ERR_PTR(-EPROBE_DEFER);

	return link;
}
#else
static struct device_link *trusty_smc_get_ffa_link(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int trusty_smc_probe(struct platform_device *pdev)
{
	int ret;
	struct device_link *link;
	struct platform_device *core_pdev;
	struct trusty_smc_state *s;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		dev_err(&pdev->dev, "of_node required\n");
		return -EINVAL;
	}

	link = trusty_smc_get_ffa_link(pdev);
	if (IS_ERR(link)) {
		ret = PTR_ERR(link);
		goto err_ffa_link;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto err_allocate_state;
	}

	s->dev = &pdev->dev;
	s->ffa_dev_link = link;
	s->transport.magic = TRUSTY_TRANSPORT_MAGIC;
	s->transport.ops = &trusty_smc_transport_ops;

	mutex_init(&s->share_memory_msg_lock);

	platform_set_drvdata(pdev, &s->transport);

	ret = trusty_init_msg_buf(s);
	if (ret < 0) {
		dev_err(s->dev, "%s: trusty_init_msg_buf failed: %d\n",
			__func__, ret);
		goto err_init_msg_buf;
	}

	core_pdev = platform_device_register_data(&pdev->dev, "trusty-core",
						  PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(core_pdev)) {
		ret = PTR_ERR(core_pdev);
		goto err_register_device;
	}
	s->core_dev = &core_pdev->dev;

	return 0;

err_register_device:
	trusty_free_msg_buf(s);
err_init_msg_buf:
	mutex_destroy(&s->share_memory_msg_lock);
	kfree(s);
err_allocate_state:
err_ffa_link:
	return ret;
}

static
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
int
#else
void
#endif
trusty_smc_remove(struct platform_device *pdev)
{
	struct trusty_smc_state *s = trusty_smc_get_state(platform_get_drvdata(pdev));
	struct platform_device *core_pdev = to_platform_device(s->core_dev);

	platform_device_unregister(core_pdev);
	trusty_free_msg_buf(s);
	mutex_destroy(&s->share_memory_msg_lock);
	kfree(s);
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
}

static const struct of_device_id trusty_smc_of_match[] = {
	{ .compatible = "android,trusty-smc-v1", },
	{},
};

MODULE_DEVICE_TABLE(trusty_smc, trusty_smc_of_match);

static struct platform_driver trusty_smc_driver = {
	.probe = trusty_smc_probe,
	.remove = trusty_smc_remove,
	.driver	= {
		.name = "trusty-smc",
		.of_match_table = trusty_smc_of_match,
	},
};

int __init trusty_smc_init(void)
{
	return platform_driver_register(&trusty_smc_driver);
}

void trusty_smc_exit(void)
{
	platform_driver_unregister(&trusty_smc_driver);
}

subsys_initcall(trusty_smc_init);
module_exit(trusty_smc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty SMC transport driver");
