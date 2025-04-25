// SPDX-License-Identifier: GPL-2.0-only
/*
 * Trusty Virtio driver
 *
 * Copyright (C) 2015 Google, Inc.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/dma-map-ops.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>
#include <linux/trusty/trusty_ipc.h>

#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>

#include <linux/atomic.h>

#define  RSC_DESCR_VER  1

struct trusty_vdev;
static bool use_high_wq;
module_param(use_high_wq, bool, 0660);

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
static ulong default_vq_check_period_ms;
module_param(default_vq_check_period_ms, ulong, 0660);
#endif

struct trusty_ctx {
	struct device		*dev;
	void			*shared_va;
	struct scatterlist	shared_sg;
	trusty_shared_mem_id_t	shared_id;
	size_t			shared_sz;
	struct work_struct	check_vqs;
	struct work_struct	kick_vqs;
	struct notifier_block	call_notifier;
	struct list_head	vdev_list;
	struct mutex		mlock; /* protects vdev_list */
#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
	struct mutex		vq_timer_lock;
	u64			vq_check_period_ms;
	struct hrtimer		vq_timer;
#endif
	struct workqueue_struct	*kick_wq;
	struct workqueue_struct	*check_wq;
};

struct trusty_vring {
	void			*vaddr;
	struct scatterlist	sg;
	trusty_shared_mem_id_t	shared_mem_id;
	size_t			size;
	unsigned int		align;
	unsigned int		elem_num;
	u32			notifyid;
	atomic_t		needs_kick;
	struct fw_rsc_vdev_vring *vr_descr;
	struct virtqueue	*vq;
	struct trusty_vdev	*tvdev;
	struct trusty_nop	kick_nop;
};

struct trusty_vdev {
	struct list_head	node;
	struct virtio_device	vdev;
	struct trusty_ctx	*tctx;
	u32			notifyid;
	unsigned int		config_len;
	void			*config;
	struct fw_rsc_vdev	*vdev_descr;
	unsigned int		vring_num;
	struct trusty_vring	vrings[];
};

#define vdev_to_tvdev(vd)  container_of((vd), struct trusty_vdev, vdev)

static void check_all_vqs(struct work_struct *work)
{
	unsigned int i;
	struct trusty_ctx *tctx = container_of(work, struct trusty_ctx,
					       check_vqs);
	struct trusty_vdev *tvdev;

	list_for_each_entry(tvdev, &tctx->vdev_list, node) {
		for (i = 0; i < tvdev->vring_num; i++)
			if (tvdev->vrings[i].vq)
				vring_interrupt(0, tvdev->vrings[i].vq);
	}

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
	/*
	 * There are two ways we can get here: from trusty_call_notify
	 * or the timer callback; to cover both cases and because
	 * hrtimer_forward_now needs the timer to not be enqueued, we
	 * do this the hard way: cancel the timer if it's queued, then
	 * restart it
	 */
	mutex_lock(&tctx->vq_timer_lock);
	hrtimer_cancel(&tctx->vq_timer);
	if (tctx->vq_check_period_ms)
		hrtimer_start(&tctx->vq_timer,
			      ms_to_ktime(tctx->vq_check_period_ms),
			      HRTIMER_MODE_REL);
	mutex_unlock(&tctx->vq_timer_lock);
#endif
}

static int trusty_call_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct trusty_ctx *tctx;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	tctx = container_of(nb, struct trusty_ctx, call_notifier);
	queue_work(tctx->check_wq, &tctx->check_vqs);

	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
static enum hrtimer_restart vq_timer_fn(struct hrtimer *timer)
{
	struct trusty_ctx *tctx = container_of(timer, struct trusty_ctx,
					       vq_timer);

	queue_work(tctx->check_wq, &tctx->check_vqs);

	/* The timer callback will start the timer */
	return HRTIMER_NORESTART;
}

static ssize_t vq_check_period_ms_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	struct trusty_ctx *tctx = dev_get_drvdata(dev);

	return sprintf(buf, "%llu\n", tctx->vq_check_period_ms);
}

static ssize_t vq_check_period_ms_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct trusty_ctx *tctx = dev_get_drvdata(dev);
	u64 value;
	int rc;

	rc = kstrtou64(buf, 0, &value);
	if (rc)
		return -EINVAL;

	mutex_lock(&tctx->vq_timer_lock);
	hrtimer_cancel(&tctx->vq_timer);
	tctx->vq_check_period_ms = value;
	if (tctx->vq_check_period_ms)
		hrtimer_start(&tctx->vq_timer,
			      ms_to_ktime(tctx->vq_check_period_ms),
			      HRTIMER_MODE_REL);
	mutex_unlock(&tctx->vq_timer_lock);

	return count;
}
static DEVICE_ATTR_RW(vq_check_period_ms);
#endif

static struct attribute *trusty_virtio_attrs[] = {
#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
	&dev_attr_vq_check_period_ms.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(trusty_virtio);

static void kick_vq(struct trusty_ctx *tctx,
		    struct trusty_vdev *tvdev,
		    struct trusty_vring *tvr)
{
	int ret;

	dev_dbg(tctx->dev, "%s: vdev_id=%d: vq_id=%d\n",
		__func__, tvdev->notifyid, tvr->notifyid);

	ret = trusty_std_call32(tctx->dev->parent, SMC_SC_VDEV_KICK_VQ,
				tvdev->notifyid, tvr->notifyid, 0);
	if (ret) {
		dev_err(tctx->dev, "vq notify (%d, %d) returned %d\n",
			tvdev->notifyid, tvr->notifyid, ret);
	}
}

static void kick_vqs(struct work_struct *work)
{
	unsigned int i;
	struct trusty_vdev *tvdev;
	struct trusty_ctx *tctx = container_of(work, struct trusty_ctx,
					       kick_vqs);
	mutex_lock(&tctx->mlock);
	list_for_each_entry(tvdev, &tctx->vdev_list, node) {
		for (i = 0; i < tvdev->vring_num; i++) {
			struct trusty_vring *tvr = &tvdev->vrings[i];

			if (atomic_xchg(&tvr->needs_kick, 0))
				kick_vq(tctx, tvdev, tvr);
		}
	}
	mutex_unlock(&tctx->mlock);
}

static bool trusty_virtio_notify(struct virtqueue *vq)
{
	struct trusty_vring *tvr = vq->priv;
	struct trusty_vdev *tvdev = tvr->tvdev;
	struct trusty_ctx *tctx = tvdev->tctx;
	u32 api_ver = trusty_get_api_version(tctx->dev->parent);

	if (api_ver < TRUSTY_API_VERSION_SMP_NOP) {
		atomic_set(&tvr->needs_kick, 1);
		queue_work(tctx->kick_wq, &tctx->kick_vqs);
	} else {
		trusty_enqueue_nop(tctx->dev->parent, &tvr->kick_nop);
	}

	return true;
}

static int trusty_load_device_descr(struct trusty_ctx *tctx,
				    trusty_shared_mem_id_t id, size_t sz)
{
	int ret;

	dev_dbg(tctx->dev, "%s: %zu bytes @ id %llu\n", __func__, sz, id);

	ret = trusty_std_call32(tctx->dev->parent, SMC_SC_VIRTIO_GET_DESCR,
				(u32)id, id >> 32, sz);
	if (ret < 0) {
		dev_err(tctx->dev, "%s: virtio get descr returned (%d)\n",
			__func__, ret);
		return -ENODEV;
	}
	return ret;
}

static void trusty_virtio_stop(struct trusty_ctx *tctx,
			       trusty_shared_mem_id_t id, size_t sz)
{
	int ret;

	dev_dbg(tctx->dev, "%s: %zu bytes @ id %llu\n", __func__, sz, id);

	ret = trusty_std_call32(tctx->dev->parent, SMC_SC_VIRTIO_STOP,
				(u32)id, id >> 32, sz);
	if (ret) {
		dev_err(tctx->dev, "%s: virtio done returned (%d)\n",
			__func__, ret);
		return;
	}
}

static int trusty_virtio_start(struct trusty_ctx *tctx,
			       trusty_shared_mem_id_t id, size_t sz)
{
	int ret;

	dev_dbg(tctx->dev, "%s: %zu bytes @ id %llu\n", __func__, sz, id);

	ret = trusty_std_call32(tctx->dev->parent, SMC_SC_VIRTIO_START,
				(u32)id, id >> 32, sz);
	if (ret) {
		dev_err(tctx->dev, "%s: virtio start returned (%d)\n",
			__func__, ret);
		return -ENODEV;
	}
	return 0;
}

static void trusty_virtio_reset(struct virtio_device *vdev)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);
	struct trusty_ctx *tctx = tvdev->tctx;

	dev_dbg(&vdev->dev, "reset vdev_id=%d\n", tvdev->notifyid);
	trusty_std_call32(tctx->dev->parent, SMC_SC_VDEV_RESET,
			  tvdev->notifyid, 0, 0);
}

static u64 trusty_virtio_get_features(struct virtio_device *vdev)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);

	return tvdev->vdev_descr->dfeatures |
		(1ULL << VIRTIO_F_ACCESS_PLATFORM);
}

static int trusty_virtio_finalize_features(struct virtio_device *vdev)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);
	u64 features = vdev->features;

	/*
	 * We set VIRTIO_F_ACCESS_PLATFORM to enable the dma mapping hooks.
	 * The other side does not need to know.
	 */
	features &= ~(1ULL << VIRTIO_F_ACCESS_PLATFORM);

	/* Make sure we don't have any features > 32 bits! */
	if (WARN_ON((u32)vdev->features != features))
		return -EINVAL;

	tvdev->vdev_descr->gfeatures = vdev->features;
	return 0;
}

static void trusty_virtio_get_config(struct virtio_device *vdev,
				     unsigned int offset, void *buf,
				     unsigned int len)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);

	dev_dbg(&vdev->dev, "%s: %d bytes @ offset %d\n",
		__func__, len, offset);

	if (tvdev->config) {
		if (offset + len <= tvdev->config_len)
			memcpy(buf, tvdev->config + offset, len);
	}
}

static void trusty_virtio_set_config(struct virtio_device *vdev,
				     unsigned int offset, const void *buf,
				     unsigned int len)
{
}

static u8 trusty_virtio_get_status(struct virtio_device *vdev)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);

	return tvdev->vdev_descr->status;
}

static void trusty_virtio_set_status(struct virtio_device *vdev, u8 status)
{
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);

	tvdev->vdev_descr->status = status;
}

static void _del_vqs(struct virtio_device *vdev)
{
	unsigned int i;
	int ret;
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);
	struct trusty_vring *tvr = &tvdev->vrings[0];

	for (i = 0; i < tvdev->vring_num; i++, tvr++) {
		/* dequeue kick_nop */
		trusty_dequeue_nop(tvdev->tctx->dev->parent, &tvr->kick_nop);

		/* delete vq */
		if (tvr->vq) {
			vring_del_virtqueue(tvr->vq);
			tvr->vq = NULL;
		}
		/* delete vring */
		if (tvr->vaddr) {
			ret = trusty_reclaim_memory(tvdev->tctx->dev->parent,
						    tvr->shared_mem_id,
						    &tvr->sg, 1);
			if (WARN_ON(ret)) {
				dev_err(&vdev->dev,
					"trusty_revoke_memory failed: %d 0x%llx\n",
					ret, tvr->shared_mem_id);
				/*
				 * It is not safe to free this memory if
				 * trusty_revoke_memory fails. Leak it in that
				 * case.
				 */
			} else {
				free_pages_exact(tvr->vaddr, tvr->size);
			}
			tvr->vaddr = NULL;
		}
	}
}

static void trusty_virtio_del_vqs(struct virtio_device *vdev)
{
	_del_vqs(vdev);
}


static struct virtqueue *_find_vq(struct virtio_device *vdev,
				  unsigned int id,
				  void (*callback)(struct virtqueue *vq),
				  const char *name,
				  bool ctx)
{
	struct trusty_vring *tvr;
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);
	phys_addr_t pa;
	int ret;

	if (!name)
		return ERR_PTR(-EINVAL);

	if (id >= tvdev->vring_num)
		return ERR_PTR(-EINVAL);

	tvr = &tvdev->vrings[id];

	/* actual size of vring (in bytes) */
	tvr->size = PAGE_ALIGN(vring_size(tvr->elem_num, tvr->align));

	/* allocate memory for the vring. */
	tvr->vaddr = alloc_pages_exact(tvr->size, GFP_KERNEL | __GFP_ZERO);
	if (!tvr->vaddr) {
		dev_err(&vdev->dev, "vring alloc failed\n");
		return ERR_PTR(-ENOMEM);
	}

	sg_init_one(&tvr->sg, tvr->vaddr, tvr->size);
	ret = trusty_share_memory_compat(tvdev->tctx->dev->parent,
					 &tvr->shared_mem_id, &tvr->sg, 1,
					 PAGE_KERNEL);
	if (ret) {
		pa = virt_to_phys(tvr->vaddr);
		dev_err(&vdev->dev, "trusty_share_memory failed: %d %pa\n",
			ret, &pa);
		goto err_share_memory;
	}

	/* save vring address to shared structure */
	tvr->vr_descr->da = (u32)tvr->shared_mem_id;

	/* da field is only 32 bit wide. Use previously unused 'reserved' field
	 * to store top 32 bits of 64-bit shared_mem_id
	 */
	tvr->vr_descr->pa = (u32)(tvr->shared_mem_id >> 32);

	dev_info(&vdev->dev, "vring%d: va(id)  %p(%llx) qsz %d notifyid %d\n",
		 id, tvr->vaddr, (u64)tvr->shared_mem_id, tvr->elem_num,
		 tvr->notifyid);

	tvr->vq = vring_new_virtqueue(id, tvr->elem_num, tvr->align,
				      vdev, true, ctx, tvr->vaddr,
				      trusty_virtio_notify, callback, name);
	if (!tvr->vq) {
		dev_err(&vdev->dev, "vring_new_virtqueue %s failed\n",
			name);
		goto err_new_virtqueue;
	}

	tvr->vq->priv = tvr;

	return tvr->vq;

err_new_virtqueue:
	ret = trusty_reclaim_memory(tvdev->tctx->dev->parent,
				    tvr->shared_mem_id, &tvr->sg, 1);
	if (WARN_ON(ret)) {
		dev_err(&vdev->dev, "trusty_revoke_memory failed: %d 0x%llx\n",
			ret, tvr->shared_mem_id);
		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
	} else {
err_share_memory:
		free_pages_exact(tvr->vaddr, tvr->size);
	}
	tvr->vaddr = NULL;
	return ERR_PTR(-ENOMEM);
}

static int trusty_virtio_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
				  struct virtqueue *vqs[],
/*
 * TODO: change to 6.11. The version here should be 6.11, but android-mainline
 * is still on 6.10 with 6.11 some changes merged. Since there is no other
 * 6.10 gki kernel temporarily treat 6.10 as 6.11.
 */
#if (KERNEL_VERSION(6, 10, 0) <= LINUX_VERSION_CODE)
				  struct virtqueue_info vqs_info[],
#else
				  vq_callback_t *callbacks[],
				  const char * const names[],
				  const bool *ctxs,
#endif
				  struct irq_affinity *desc)
{
	unsigned int i;
	int ret;
	vq_callback_t *callback;
	const char *name;
	bool ctx = false;

	for (i = 0; i < nvqs; i++) {
/*
 * TODO: change to 6.11. The version here should be 6.11, but android-mainline
 * is still on 6.10 with 6.11 some changes merged. Since there is no other
 * 6.10 gki kernel temporarily treat 6.10 as 6.11.
 */
#if (KERNEL_VERSION(6, 10, 0) <= LINUX_VERSION_CODE)
		callback = vqs_info[i].callback;
		name = vqs_info[i].name;
		ctx = vqs_info[i].ctx;
#else
		callback = callbacks[i];
		name = names[i];
		ctx = false;
		if (ctxs)
			ctx = ctxs[i];
#endif
		vqs[i] = _find_vq(vdev, i, callback, name, ctx);
		if (IS_ERR(vqs[i])) {
			ret = PTR_ERR(vqs[i]);
			_del_vqs(vdev);
			return ret;
		}
	}
	return 0;
}

static const char *trusty_virtio_bus_name(struct virtio_device *vdev)
{
	return "trusty-virtio";
}

/* The ops structure which hooks everything together. */
static const struct virtio_config_ops trusty_virtio_config_ops = {
	.get_features = trusty_virtio_get_features,
	.finalize_features = trusty_virtio_finalize_features,
	.get = trusty_virtio_get_config,
	.set = trusty_virtio_set_config,
	.get_status = trusty_virtio_get_status,
	.set_status = trusty_virtio_set_status,
	.reset    = trusty_virtio_reset,
	.find_vqs = trusty_virtio_find_vqs,
	.del_vqs  = trusty_virtio_del_vqs,
	.bus_name = trusty_virtio_bus_name,
};

static void trusty_virtio_release_device(struct device *dev)
{
	struct virtio_device *vdev =
			container_of(dev, struct virtio_device, dev);
	struct trusty_vdev *tvdev = vdev_to_tvdev(vdev);

	kfree(tvdev);
}

static int trusty_virtio_add_device(struct trusty_ctx *tctx,
				    struct fw_rsc_vdev *vdev_descr,
				    struct fw_rsc_vdev_vring *vr_descr,
				    void *config)
{
	int i, ret;
	struct trusty_vdev *tvdev;

	tvdev = kzalloc(struct_size(tvdev, vrings, vdev_descr->num_of_vrings),
			GFP_KERNEL);
	if (!tvdev)
		return -ENOMEM;

	/* setup vdev */
	tvdev->tctx = tctx;
	tvdev->vdev.dev.parent = tctx->dev;
	tvdev->vdev.dev.release = trusty_virtio_release_device;
	tvdev->vdev.id.device  = vdev_descr->id;
	tvdev->vdev.config = &trusty_virtio_config_ops;
	tvdev->vdev_descr = vdev_descr;
	tvdev->notifyid = vdev_descr->notifyid;

	/* setup config */
	tvdev->config = config;
	tvdev->config_len = vdev_descr->config_len;

	/* setup vrings and vdev resource */
	tvdev->vring_num = vdev_descr->num_of_vrings;

	for (i = 0; i < tvdev->vring_num; i++, vr_descr++) {
		struct trusty_vring *tvr = &tvdev->vrings[i];

		tvr->tvdev    = tvdev;
		tvr->vr_descr = vr_descr;
		tvr->align    = vr_descr->align;
		tvr->elem_num = vr_descr->num;
		tvr->notifyid = vr_descr->notifyid;
		trusty_nop_init(&tvr->kick_nop, SMC_NC_VDEV_KICK_VQ,
				tvdev->notifyid, tvr->notifyid);
	}

	/* register device */
	ret = register_virtio_device(&tvdev->vdev);
	if (ret) {
		dev_err(tctx->dev,
			"Failed (%d) to register device dev type %u\n",
			ret, vdev_descr->id);
		goto err_register;
	}

	/* add it to tracking list */
	list_add_tail(&tvdev->node, &tctx->vdev_list);

	return 0;

err_register:
	/*
	 * Once we call register_virtio_device, we need to free
	 * the device using put_device(). Before that, we should
	 * call kfree instead.
	 */
	put_device(&tvdev->vdev.dev);
	return ret;
}

static int trusty_parse_device_descr(struct trusty_ctx *tctx,
				     void *descr_va, size_t descr_sz)
{
	u32 i;
	struct resource_table *descr = descr_va;

	if (descr_sz < sizeof(*descr)) {
		dev_err(tctx->dev, "descr table is too small (0x%x)\n",
			(int)descr_sz);
		return -ENODEV;
	}

	if (descr->ver != RSC_DESCR_VER) {
		dev_err(tctx->dev, "unexpected descr ver (0x%x)\n",
			(int)descr->ver);
		return -ENODEV;
	}

	if (descr_sz < (sizeof(*descr) + descr->num * sizeof(u32))) {
		dev_err(tctx->dev, "descr table is too small (0x%x)\n",
			(int)descr->ver);
		return -ENODEV;
	}

	for (i = 0; i < descr->num; i++) {
		struct fw_rsc_hdr *hdr;
		struct fw_rsc_vdev *vd;
		struct fw_rsc_vdev_vring *vr;
		void *cfg;
		size_t vd_sz;

		u32 offset = descr->offset[i];

		if (offset >= descr_sz) {
			dev_err(tctx->dev, "offset is out of bounds (%u)\n",
				offset);
			return -ENODEV;
		}

		/* check space for rsc header */
		if ((descr_sz - offset) < sizeof(struct fw_rsc_hdr)) {
			dev_err(tctx->dev, "no space for rsc header (%u)\n",
				offset);
			return -ENODEV;
		}
		hdr = (struct fw_rsc_hdr *)((u8 *)descr + offset);
		offset += sizeof(struct fw_rsc_hdr);

		/* check type */
		if (hdr->type != RSC_VDEV) {
			dev_err(tctx->dev, "unsupported rsc type (%u)\n",
				hdr->type);
			continue;
		}

		/* got vdev: check space for vdev */
		if ((descr_sz - offset) < sizeof(struct fw_rsc_vdev)) {
			dev_err(tctx->dev, "no space for vdev descr (%u)\n",
				offset);
			return -ENODEV;
		}
		vd = (struct fw_rsc_vdev *)((u8 *)descr + offset);

		/* check space for vrings and config area */
		vd_sz = sizeof(struct fw_rsc_vdev) +
			vd->num_of_vrings * sizeof(struct fw_rsc_vdev_vring) +
			vd->config_len;

		if ((descr_sz - offset) < vd_sz) {
			dev_err(tctx->dev, "no space for vdev (%u)\n", offset);
			return -ENODEV;
		}
		vr = (struct fw_rsc_vdev_vring *)vd->vring;
		cfg = (void *)(vr + vd->num_of_vrings);

		trusty_virtio_add_device(tctx, vd, vr, cfg);
	}

	return 0;
}

static void _remove_devices_locked(struct trusty_ctx *tctx)
{
	struct trusty_vdev *tvdev, *next;

	list_for_each_entry_safe(tvdev, next, &tctx->vdev_list, node) {
		list_del(&tvdev->node);
		unregister_virtio_device(&tvdev->vdev);
	}
}

static void trusty_virtio_remove_devices(struct trusty_ctx *tctx)
{
	mutex_lock(&tctx->mlock);
	_remove_devices_locked(tctx);
	mutex_unlock(&tctx->mlock);
}

static int trusty_virtio_add_devices(struct trusty_ctx *tctx)
{
	int ret;
	int ret_tmp;
	void *descr_va;
	trusty_shared_mem_id_t descr_id;
	size_t descr_sz;
	size_t descr_buf_sz;

	/* allocate buffer to load device descriptor into */
	descr_buf_sz = PAGE_SIZE;
	descr_va = alloc_pages_exact(descr_buf_sz, GFP_KERNEL | __GFP_ZERO);
	if (!descr_va) {
		dev_err(tctx->dev, "Failed to allocate shared area\n");
		return -ENOMEM;
	}

	sg_init_one(&tctx->shared_sg, descr_va, descr_buf_sz);
	ret = trusty_share_memory(tctx->dev->parent, &descr_id,
				  &tctx->shared_sg, 1, PAGE_KERNEL,
				  TRUSTY_DEFAULT_MEM_OBJ_TAG);
	if (ret) {
		dev_err(tctx->dev, "trusty_share_memory failed: %d\n", ret);
		goto err_share_memory;
	}

	/* load device descriptors */
	ret = trusty_load_device_descr(tctx, descr_id, descr_buf_sz);
	if (ret < 0) {
		dev_err(tctx->dev, "failed (%d) to load device descr\n", ret);
		goto err_load_descr;
	}

	descr_sz = (size_t)ret;

	mutex_lock(&tctx->mlock);

	/* parse device descriptor and add virtio devices */
	ret = trusty_parse_device_descr(tctx, descr_va, descr_sz);
	if (ret) {
		dev_err(tctx->dev, "failed (%d) to parse device descr\n", ret);
		goto err_parse_descr;
	}

	/* register call notifier */
	ret = trusty_call_notifier_register(tctx->dev->parent,
					    &tctx->call_notifier);
	if (ret) {
		dev_err(tctx->dev, "%s: failed (%d) to register notifier\n",
			__func__, ret);
		goto err_register_notifier;
	}

	/* start virtio */
	ret = trusty_virtio_start(tctx, descr_id, descr_sz);
	if (ret) {
		dev_err(tctx->dev, "failed (%d) to start virtio\n", ret);
		goto err_start_virtio;
	}

	/* attach shared area */
	tctx->shared_va = descr_va;
	tctx->shared_id = descr_id;
	tctx->shared_sz = descr_buf_sz;

	mutex_unlock(&tctx->mlock);

	return 0;

err_start_virtio:
	trusty_call_notifier_unregister(tctx->dev->parent,
					&tctx->call_notifier);
	cancel_work_sync(&tctx->check_vqs);
err_register_notifier:
err_parse_descr:
	_remove_devices_locked(tctx);
	mutex_unlock(&tctx->mlock);
	cancel_work_sync(&tctx->kick_vqs);
	trusty_virtio_stop(tctx, descr_id, descr_sz);
err_load_descr:
	ret_tmp = trusty_reclaim_memory(tctx->dev->parent, descr_id,
					&tctx->shared_sg, 1);
	if (WARN_ON(ret_tmp)) {
		dev_err(tctx->dev, "trusty_revoke_memory failed: %d 0x%llx\n",
			ret_tmp, tctx->shared_id);
		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
	} else {
err_share_memory:
		free_pages_exact(descr_va, descr_buf_sz);
	}
	return ret;
}

static dma_addr_t trusty_virtio_dma_map_page(struct device *dev,
					     struct page *page,
					     unsigned long offset, size_t size,
					     enum dma_data_direction dir,
					     unsigned long attrs)
{
	struct tipc_msg_buf *buf = page_to_virt(page) + offset;

	return buf->buf_id;
}

/* The kernel requires this to exist, but doesn't require it do anything */
static void trusty_virtio_dma_unmap_page(struct device *dev,
					 dma_addr_t dma_handle,
					 size_t size,
					 enum dma_data_direction dir,
					 unsigned long attrs)
{
}

/* The kernel requires this to exist, but doesn't require it do anything */
static int trusty_virtio_dma_map_sg(struct device *dev,
				    struct scatterlist *sg, int nents,
				    enum dma_data_direction dir,
				    unsigned long attrs)
{
	WARN_ON_ONCE(true);

#if (KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE)
	return 0;
#else
	return -EINVAL;
#endif
}

/* The kernel requires this to exist, but doesn't require it do anything */
static void trusty_virtio_dma_unmap_sg(struct device *dev,
				       struct scatterlist *sg, int nents,
				       enum dma_data_direction dir,
				       unsigned long attrs)
{
	WARN_ON_ONCE(true);
}

static const struct dma_map_ops trusty_virtio_dma_map_ops = {
	.map_page = trusty_virtio_dma_map_page,
	.unmap_page = trusty_virtio_dma_unmap_page,
	.map_sg = trusty_virtio_dma_map_sg,
	.unmap_sg = trusty_virtio_dma_unmap_sg,
};

#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
#if (KERNEL_VERSION(6, 13, 0) > LINUX_VERSION_CODE)
static void hrtimer_setup(struct hrtimer *timer, enum hrtimer_restart (*function)(struct hrtimer *),
			  clockid_t clock_id, enum hrtimer_mode mode)
{
	BUG_ON(!function);

	hrtimer_init(timer, clock_id, mode);
	timer->function = function;
}
#endif
#endif

static int trusty_virtio_probe(struct platform_device *pdev)
{
	int ret;
	struct trusty_ctx *tctx;

	tctx = kzalloc(sizeof(*tctx), GFP_KERNEL);
	if (!tctx)
		return -ENOMEM;

	tctx->dev = &pdev->dev;
	tctx->call_notifier.notifier_call = trusty_call_notify;
	mutex_init(&tctx->mlock);
	INIT_LIST_HEAD(&tctx->vdev_list);
#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
	mutex_init(&tctx->vq_timer_lock);
	hrtimer_setup(&tctx->vq_timer, vq_timer_fn, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tctx->vq_check_period_ms = default_vq_check_period_ms;
#endif
	INIT_WORK(&tctx->check_vqs, check_all_vqs);
	INIT_WORK(&tctx->kick_vqs, kick_vqs);
	platform_set_drvdata(pdev, tctx);

	set_dma_ops(&pdev->dev, &trusty_virtio_dma_map_ops);

	tctx->check_wq = alloc_workqueue("trusty-check-wq",
					 WQ_UNBOUND | WQ_HIGHPRI, 0);
	if (!tctx->check_wq) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "Failed create trusty-check-wq\n");
		goto err_create_check_wq;
	}

	tctx->kick_wq = alloc_workqueue("trusty-kick-wq",
					WQ_UNBOUND | WQ_CPU_INTENSIVE, 0);
	if (!tctx->kick_wq) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "Failed create trusty-kick-wq\n");
		goto err_create_kick_wq;
	}

	ret = trusty_virtio_add_devices(tctx);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add virtio devices\n");
		goto err_add_devices;
	}

	dev_info(&pdev->dev, "initializing done\n");
	return 0;

err_add_devices:
	destroy_workqueue(tctx->kick_wq);
err_create_kick_wq:
	destroy_workqueue(tctx->check_wq);
err_create_check_wq:
	kfree(tctx);
	return ret;
}

static
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
int
#else
void
#endif
trusty_virtio_remove(struct platform_device *pdev)
{
	struct trusty_ctx *tctx = platform_get_drvdata(pdev);
	int ret;

	/* unregister call notifier and wait until workqueue is done */
	trusty_call_notifier_unregister(tctx->dev->parent,
					&tctx->call_notifier);
	cancel_work_sync(&tctx->check_vqs);
#if IS_ENABLED(CONFIG_TRUSTY_VIRTIO_POLL_VQUEUES)
	hrtimer_cancel(&tctx->vq_timer);
#endif

	/* remove virtio devices */
	trusty_virtio_remove_devices(tctx);
	cancel_work_sync(&tctx->kick_vqs);

	/* destroy workqueues */
	destroy_workqueue(tctx->kick_wq);
	destroy_workqueue(tctx->check_wq);

	/* notify remote that shared area goes away */
	trusty_virtio_stop(tctx, tctx->shared_id, tctx->shared_sz);

	/* free shared area */
	ret = trusty_reclaim_memory(tctx->dev->parent, tctx->shared_id,
				    &tctx->shared_sg, 1);
	if (WARN_ON(ret)) {
		dev_err(tctx->dev, "trusty_revoke_memory failed: %d 0x%llx\n",
			ret, tctx->shared_id);
		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
	} else {
		free_pages_exact(tctx->shared_va, tctx->shared_sz);
	}

	/* free context */
	kfree(tctx);
#if (KERNEL_VERSION(6, 12, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
}

static const struct of_device_id trusty_of_match[] = {
	{
		.compatible = "android,trusty-virtio-v1",
	},
	{},
};

MODULE_DEVICE_TABLE(of, trusty_of_match);

static struct platform_driver trusty_virtio_driver = {
	.probe = trusty_virtio_probe,
	.remove = trusty_virtio_remove,
	.driver = {
		.name = "trusty-virtio",
		.of_match_table = trusty_of_match,
		.dev_groups = trusty_virtio_groups,
	},
};

module_platform_driver(trusty_virtio_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty virtio driver");
/*
 * TODO(b/168322325): trusty-virtio and trusty-ipc should be independent.
 * However, trusty-virtio is not completely generic and is aware of trusty-ipc.
 * See header includes. Particularly, trusty-virtio.ko can't be loaded before
 * trusty-ipc.ko.
 */
MODULE_SOFTDEP("pre: trusty-ipc");
