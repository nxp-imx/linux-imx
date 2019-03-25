/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * derived from the omap-rpmsg implementation.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/imx_rpmsg.h>
#include <linux/mx8_mu.h>
#ifdef CONFIG_ARCH_MXC_ARM64
#include <soc/imx8/sc/sci.h>
#include <soc/imx8/sc/svc/irq/api.h>
#endif
#include "rpmsg_internal.h"

enum imx_rpmsg_variants {
	IMX6SX,
	IMX7D,
	IMX7ULP,
	IMX8MQ,
	IMX8QXP,
	IMX8QM,
};

struct imx_virdev {
	struct virtio_device vdev;
	unsigned int vring[2];
	struct virtqueue *vq[2];
	int base_vq_id;
	int num_of_vqs;
	u32 vproc_id;
	struct notifier_block nb;
};

struct imx_rpmsg_vproc {
	char *rproc_name;
	struct mutex lock;
	struct clk *mu_clk;
	enum imx_rpmsg_variants variant;
	int vdev_nums;
	int first_notify;
	u32 none_suspend;
#define MAX_VDEV_NUMS  8
	struct imx_virdev *ivdev[MAX_VDEV_NUMS];
	void __iomem *mu_base;
	struct delayed_work rpmsg_work;
	struct blocking_notifier_head notifier;
#define MAX_NUM 10	/* enlarge it if overflow happen */
	u32 m4_message[MAX_NUM];
	u32 in_idx;
	u32 out_idx;
	u32 core_id;
	u32 mub_partition;
	struct notifier_block *pnotifier;
	spinlock_t mu_lock;
	struct platform_device *pdev;
};

/*
 * For now, allocate 256 buffers of 512 bytes for each side. each buffer
 * will then have 16B for the msg header and 496B for the payload.
 * This will require a total space of 256KB for the buffers themselves, and
 * 3 pages for every vring (the size of the vring depends on the number of
 * buffers it supports).
 */
#define RPMSG_NUM_BUFS		(512)
#define RPMSG_BUF_SIZE		(512)
#define RPMSG_BUFS_SPACE	(RPMSG_NUM_BUFS * RPMSG_BUF_SIZE)

/*
 * The alignment between the consumer and producer parts of the vring.
 * Note: this is part of the "wire" protocol. If you change this, you need
 * to update your BIOS image as well
 */
#define RPMSG_VRING_ALIGN	(4096)

/* With 256 buffers, our vring will occupy 3 pages */
#define RPMSG_RING_SIZE	((DIV_ROUND_UP(vring_size(RPMSG_NUM_BUFS / 2, \
				RPMSG_VRING_ALIGN), PAGE_SIZE)) * PAGE_SIZE)

#define to_imx_virdev(vd) container_of(vd, struct imx_virdev, vdev)

/* Flag 0 of ASR, 1 indicated that remote processor is ready */
#define REMOTE_IS_READY			BIT(0)
/*
 * The time consumption by remote ready is less than 1ms in the
 * evaluation. Set the max wait timeout as 50ms here.
 */
#define REMOTE_READY_WAIT_MAX_RETRIES	500

struct imx_rpmsg_vq_info {
	__u16 num;	/* number of entries in the virtio_ring */
	__u16 vq_id;	/* a globaly unique index of this virtqueue */
	void *addr;	/* address where we mapped the virtio ring */
	struct imx_rpmsg_vproc *rpdev;
};

static int imx_rpmsg_partion_notify0(struct notifier_block *nb,
				unsigned long event, void *group);
static int imx_rpmsg_partion_notify1(struct notifier_block *nb,
				unsigned long event, void *group);

static struct imx_rpmsg_vproc imx_rpmsg_vprocs[];

static u64 imx_rpmsg_get_features(struct virtio_device *vdev)
{
	/* VIRTIO_RPMSG_F_NS has been made private */
	return 1 << 0;
}

static int imx_rpmsg_finalize_features(struct virtio_device *vdev)
{
	/* Give virtio_ring a chance to accept features */
	vring_transport_features(vdev);
	return 0;
}

/* kick the remote processor, and let it know which virtqueue to poke at */
static bool imx_rpmsg_notify(struct virtqueue *vq)
{
	unsigned int mu_rpmsg = 0;
	struct imx_rpmsg_vq_info *rpvq = vq->priv;

	mu_rpmsg = rpvq->vq_id << 16;
	mutex_lock(&rpvq->rpdev->lock);
	/*
	 * Send the index of the triggered virtqueue as the mu payload.
	 * Use the timeout MU send message here.
	 * Since that M4 core may not be loaded, and the first MSG may
	 * not be handled by M4 when multi-vdev is enabled.
	 * To make sure that the message wound't be discarded when M4
	 * is running normally or in the suspend mode. Only use
	 * the timeout mechanism by the first notify when the vdev is
	 * registered.
	 * ~14ms is required by M4 ready to process the MU message from
	 * cold boot. Set the wait time 20ms here.
	 */
	if (unlikely(rpvq->rpdev->first_notify > 0)) {
		rpvq->rpdev->first_notify--;
		MU_SendMessageTimeout(rpvq->rpdev->mu_base, 1, mu_rpmsg, 2000);
	} else {
		MU_SendMessage(rpvq->rpdev->mu_base, 1, mu_rpmsg);
	}
	mutex_unlock(&rpvq->rpdev->lock);

	return true;
}

static int imx_mu_rpmsg_callback(struct notifier_block *this,
					unsigned long index, void *data)
{
	u32 mu_msg = (phys_addr_t) data;
	struct imx_virdev *virdev;

	virdev = container_of(this, struct imx_virdev, nb);

	pr_debug("%s mu_msg: 0x%x\n", __func__, mu_msg);
	/* ignore vq indices which are clearly not for us */
	mu_msg = mu_msg >> 16;
	if (mu_msg < virdev->base_vq_id || mu_msg > virdev->base_vq_id + 1) {
		pr_debug("mu_msg: 0x%x is invalid\n", mu_msg);
		return NOTIFY_DONE;
	}

	mu_msg -= virdev->base_vq_id;

	/*
	 * Currently both PENDING_MSG and explicit-virtqueue-index
	 * messaging are supported.
	 * Whatever approach is taken, at this point 'mu_msg' contains
	 * the index of the vring which was just triggered.
	 */
	if (mu_msg < virdev->num_of_vqs)
		vring_interrupt(mu_msg, virdev->vq[mu_msg]);

	return NOTIFY_DONE;
}

static int imx_mu_rpmsg_register_nb(struct imx_rpmsg_vproc *rpdev,
		struct notifier_block *nb)
{
	if ((rpdev == NULL) || (nb == NULL))
		return -EINVAL;

	blocking_notifier_chain_register(&(rpdev->notifier), nb);

	return 0;
}

static int imx_mu_rpmsg_unregister_nb(struct imx_rpmsg_vproc *rpdev,
		struct notifier_block *nb)
{
	if ((rpdev == NULL) || (nb == NULL))
		return -EINVAL;

	blocking_notifier_chain_unregister(&(rpdev->notifier), nb);

	return 0;
}

static struct virtqueue *rp_find_vq(struct virtio_device *vdev,
				    unsigned int index,
				    void (*callback)(struct virtqueue *vq),
				    const char *name,
				    bool ctx)
{
	struct imx_virdev *virdev = to_imx_virdev(vdev);
	int id = virdev->vproc_id;
	struct imx_rpmsg_vproc *rpdev = &imx_rpmsg_vprocs[id];
	struct imx_rpmsg_vq_info *rpvq;
	struct virtqueue *vq;
	int err;

	rpvq = kmalloc(sizeof(*rpvq), GFP_KERNEL);
	if (!rpvq)
		return ERR_PTR(-ENOMEM);

	/* ioremap'ing normal memory, so we cast away sparse's complaints */
	rpvq->addr = (__force void *) ioremap_nocache(virdev->vring[index],
							RPMSG_RING_SIZE);
	if (!rpvq->addr) {
		err = -ENOMEM;
		goto free_rpvq;
	}

	memset_io(rpvq->addr, 0, RPMSG_RING_SIZE);

	pr_debug("vring%d: phys 0x%x, virt 0x%p\n", index, virdev->vring[index],
					rpvq->addr);

	vq = vring_new_virtqueue(index, RPMSG_NUM_BUFS / 2, RPMSG_VRING_ALIGN,
			vdev, true, ctx,
			rpvq->addr,
			imx_rpmsg_notify, callback,
			name);
	if (!vq) {
		pr_err("vring_new_virtqueue failed\n");
		err = -ENOMEM;
		goto unmap_vring;
	}

	virdev->vq[index] = vq;
	vq->priv = rpvq;
	/* system-wide unique id for this virtqueue */
	rpvq->vq_id = virdev->base_vq_id + index;
	rpvq->rpdev = rpdev;
	mutex_init(&rpdev->lock);

	return vq;

unmap_vring:
	/* iounmap normal memory, so make sparse happy */
	iounmap((__force void __iomem *) rpvq->addr);
free_rpvq:
	kfree(rpvq);
	return ERR_PTR(err);
}

static void imx_rpmsg_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;
	struct imx_virdev *virdev = to_imx_virdev(vdev);
	int id = virdev->vproc_id;
	struct imx_rpmsg_vproc *rpdev = &imx_rpmsg_vprocs[id];

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		struct imx_rpmsg_vq_info *rpvq = vq->priv;

		iounmap(rpvq->addr);
		vring_del_virtqueue(vq);
		kfree(rpvq);
	}

	if (&virdev->nb)
		imx_mu_rpmsg_unregister_nb(rpdev, &virdev->nb);
}

static int imx_rpmsg_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
		       struct virtqueue *vqs[],
		       vq_callback_t *callbacks[],
		       const char * const names[],
		       const bool *ctx,
		       struct irq_affinity *desc)
{
	struct imx_virdev *virdev = to_imx_virdev(vdev);
	int id = virdev->vproc_id;
	struct imx_rpmsg_vproc *rpdev = &imx_rpmsg_vprocs[id];
	int i, err;

	/* we maintain two virtqueues per remote processor (for RX and TX) */
	if (nvqs != 2)
		return -EINVAL;

	for (i = 0; i < nvqs; ++i) {
		vqs[i] = rp_find_vq(vdev, i, callbacks[i], names[i],
				ctx ? ctx[i] : false);
		if (IS_ERR(vqs[i])) {
			err = PTR_ERR(vqs[i]);
			goto error;
		}
	}

	virdev->num_of_vqs = nvqs;

	virdev->nb.notifier_call = imx_mu_rpmsg_callback;
	imx_mu_rpmsg_register_nb(rpdev, &virdev->nb);

	return 0;

error:
	imx_rpmsg_del_vqs(vdev);
	return err;
}

static void imx_rpmsg_reset(struct virtio_device *vdev)
{
	dev_dbg(&vdev->dev, "reset !\n");
}

static u8 imx_rpmsg_get_status(struct virtio_device *vdev)
{
	return 0;
}

static void imx_rpmsg_set_status(struct virtio_device *vdev, u8 status)
{
	dev_dbg(&vdev->dev, "%s new status: %d\n", __func__, status);
}

static void imx_rpmsg_vproc_release(struct device *dev)
{
	/* this handler is provided so driver core doesn't yell at us */
}

static struct virtio_config_ops imx_rpmsg_config_ops = {
	.get_features	= imx_rpmsg_get_features,
	.finalize_features = imx_rpmsg_finalize_features,
	.find_vqs	= imx_rpmsg_find_vqs,
	.del_vqs	= imx_rpmsg_del_vqs,
	.reset		= imx_rpmsg_reset,
	.set_status	= imx_rpmsg_set_status,
	.get_status	= imx_rpmsg_get_status,
};

static struct notifier_block imx_rpmsg_partion_notifier[] = {
	{
		.notifier_call = imx_rpmsg_partion_notify0,
	},
	{
		.notifier_call = imx_rpmsg_partion_notify1,
	},
};

static struct imx_rpmsg_vproc imx_rpmsg_vprocs[] = {
	{
		.rproc_name	= "m4",
		.pnotifier	= &imx_rpmsg_partion_notifier[0],
	},
	{
		.rproc_name	= "m4",
		.pnotifier	= &imx_rpmsg_partion_notifier[1],
	},
};

static const struct of_device_id imx_rpmsg_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-rpmsg",  .data = (void *)IMX6SX, },
	{ .compatible = "fsl,imx7d-rpmsg",   .data = (void *)IMX7D, },
	{ .compatible = "fsl,imx7ulp-rpmsg", .data = (void *)IMX7ULP, },
	{ .compatible = "fsl,imx8mq-rpmsg", .data = (void *)IMX8MQ, },
	{ .compatible = "fsl,imx8qxp-rpmsg", .data = (void *)IMX8QXP, },
	{ .compatible = "fsl,imx8qm-rpmsg", .data = (void *)IMX8QM, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_dt_ids);

static int set_vring_phy_buf(struct platform_device *pdev,
		       struct imx_rpmsg_vproc *rpdev, int vdev_nums)
{
	struct resource *res;
	resource_size_t size;
	unsigned int start, end;
	int i, ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		size = resource_size(res);
		start = res->start;
		end = res->start + size;
		for (i = 0; i < vdev_nums; i++) {
			rpdev->ivdev[i] = kzalloc(sizeof(struct imx_virdev),
							GFP_KERNEL);
			if (!rpdev->ivdev[i])
				return -ENOMEM;

			rpdev->ivdev[i]->vring[0] = start;
			rpdev->ivdev[i]->vring[1] = start +
						   0x8000;
			start += 0x10000;
			if (start > end) {
				pr_err("Too small memory size %x!\n",
						(u32)size);
				ret = -EINVAL;
				break;
			}
		}
	} else {
		return -ENOMEM;
	}

	return ret;
}

static void rpmsg_work_handler(struct work_struct *work)
{
	u32 message;
	unsigned long flags;
	struct delayed_work *dwork = to_delayed_work(work);
	struct imx_rpmsg_vproc *rpdev = container_of(dwork,
			struct imx_rpmsg_vproc, rpmsg_work);

	spin_lock_irqsave(&rpdev->mu_lock, flags);
	/* handle all incoming mu message */
	while (rpdev->in_idx != rpdev->out_idx) {
		message = rpdev->m4_message[rpdev->out_idx % MAX_NUM];
		spin_unlock_irqrestore(&rpdev->mu_lock, flags);

		blocking_notifier_call_chain(&(rpdev->notifier), 4,
						(void *)(phys_addr_t)message);

		spin_lock_irqsave(&rpdev->mu_lock, flags);
		rpdev->m4_message[rpdev->out_idx % MAX_NUM] = 0;
		rpdev->out_idx++;
	}
	spin_unlock_irqrestore(&rpdev->mu_lock, flags);
}

static irqreturn_t imx_mu_rpmsg_isr(int irq, void *param)
{
	u32 irqs, message;
	unsigned long flags;
	struct imx_rpmsg_vproc *rpdev = (struct imx_rpmsg_vproc *)param;

	irqs = MU_ReadStatus(rpdev->mu_base);

	/* RPMSG */
	if (irqs & (1 << 26)) {
		spin_lock_irqsave(&rpdev->mu_lock, flags);
		/* get message from receive buffer */
		MU_ReceiveMsg(rpdev->mu_base, 1, &message);
		rpdev->m4_message[rpdev->in_idx % MAX_NUM] = message;
		rpdev->in_idx++;
		/*
		 * Too many mu message not be handled in timely, can enlarge
		 * MAX_NUM
		 */
		if (rpdev->in_idx == rpdev->out_idx) {
			spin_unlock_irqrestore(&rpdev->mu_lock, flags);
			pr_err("MU overflow!\n");
			return IRQ_HANDLED;
		}
		spin_unlock_irqrestore(&rpdev->mu_lock, flags);

		schedule_delayed_work(&(rpdev->rpmsg_work), 0);
	}

	return IRQ_HANDLED;
}

static int imx_rpmsg_mu_init(struct imx_rpmsg_vproc *rpdev)
{
	int ret = 0;

	/*
	 * bit26 is used by rpmsg channels.
	 * bit0 of MX7ULP_MU_CR used to let m4 to know MU is ready now
	 */
	MU_Init(rpdev->mu_base);
	if (rpdev->variant == IMX7ULP) {
		MU_EnableRxFullInt(rpdev->mu_base, 1);
		ret = MU_SetFn(rpdev->mu_base, 1);
	} else {
		MU_EnableRxFullInt(rpdev->mu_base, 1);
	}

	return ret;
}

void imx_rpmsg_restore(struct imx_rpmsg_vproc *rpdev)
{
	int i;
	u32 flags = 0;
	int vdev_nums = rpdev->vdev_nums;

	for (i = 0; i < vdev_nums; i++) {
		unregister_virtio_device(&rpdev->ivdev[i]->vdev);
		kfree(rpdev->ivdev[i]);
	}

	/* Make a double check that remote processor is ready or not */
	for (i = 0; i < REMOTE_READY_WAIT_MAX_RETRIES; i++) {
		if (rpdev->none_suspend)
			flags = MU_ReadStatus(rpdev->mu_base);
		if (flags & REMOTE_IS_READY)
			break;
		usleep_range(100, 200);
	}
	if (unlikely((flags & REMOTE_IS_READY) == 0)) {
		pr_err("Wait for remote ready timeout, assume it's dead.\n");
		/*
		 * In order to make the codes to be robust and back compatible.
		 * When wait remote ready timeout, use the MU_SendMessageTimeout
		 * to send the first kick-off message when register the vdev.
		 */
		rpdev->first_notify = rpdev->vdev_nums;
	}

	/* Allocate and setup ivdev again to register virtio devices */
	if (set_vring_phy_buf(rpdev->pdev, rpdev, rpdev->vdev_nums))
		pr_err("No vring buffer.\n");

	for (i = 0; i < vdev_nums; i++) {
		rpdev->ivdev[i]->vdev.id.device = VIRTIO_ID_RPMSG;
		rpdev->ivdev[i]->vdev.config = &imx_rpmsg_config_ops;
		rpdev->ivdev[i]->vdev.dev.parent = &rpdev->pdev->dev;
		rpdev->ivdev[i]->vdev.dev.release = imx_rpmsg_vproc_release;
		rpdev->ivdev[i]->base_vq_id = i * 2;
		rpdev->ivdev[i]->vproc_id = rpdev->core_id;

		if (register_virtio_device(&rpdev->ivdev[i]->vdev))
			pr_err("%s failed to register rpdev.\n", __func__);
	}
}

static int imx_rpmsg_partion_notify0(struct notifier_block *nb,
				      unsigned long event, void *group)
{
#ifdef CONFIG_ARCH_MXC_ARM64
	struct imx_rpmsg_vproc *rpdev = &imx_rpmsg_vprocs[0];

	/* Ignore other irqs */
	if (!((event & BIT(rpdev->mub_partition)) &&
		(*(sc_irq_group_t *)group == SC_IRQ_GROUP_REBOOTED)))
		return 0;

	imx_rpmsg_restore(rpdev);
	pr_info("Patition%d reset!\n", rpdev->mub_partition);
#endif
	return 0;
}

static int imx_rpmsg_partion_notify1(struct notifier_block *nb,
				      unsigned long event, void *group)
{
#ifdef CONFIG_ARCH_MXC_ARM64
	struct imx_rpmsg_vproc *rpdev = &imx_rpmsg_vprocs[1];

	/* Ignore other irqs */
	if (!((event & BIT(rpdev->mub_partition)) &&
		(*(sc_irq_group_t *)group == SC_IRQ_GROUP_REBOOTED)))
		return 0;

	imx_rpmsg_restore(rpdev);
	pr_info("Patition%d reset!\n", rpdev->mub_partition);
#endif
	return 0;
}

static int imx_rpmsg_probe(struct platform_device *pdev)
{
	int core_id, j, ret = 0;
	u32 irq;
	struct device_node *np_mu;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct imx_rpmsg_vproc *rpdev;

	if (of_property_read_u32(np, "multi-core-id", &core_id))
		core_id = 0;
	rpdev = &imx_rpmsg_vprocs[core_id];
	rpdev->core_id = core_id;
	rpdev->variant = (enum imx_rpmsg_variants)of_device_get_match_data(dev);

	/* Initialize the mu unit used by rpmsg */
	if (rpdev->core_id == 1)
		np_mu = of_find_compatible_node(NULL, NULL,
				"fsl,imx-mu-rpmsg1");
	else
		np_mu = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-mu");
	if (!np_mu) {
		pr_info("Cannot find MU-RPMSG entry in device tree\n");
		return -EINVAL;
	}
	rpdev->mu_base = of_iomap(np_mu, 0);
	WARN_ON(!rpdev->mu_base);

	spin_lock_init(&rpdev->mu_lock);

	if (rpdev->variant == IMX7ULP)
		irq = of_irq_get(np_mu, 1);
	else
		irq = of_irq_get(np_mu, 0);

	ret = request_irq(irq, imx_mu_rpmsg_isr,
			  IRQF_EARLY_RESUME | IRQF_SHARED,
			  "imx-mu-rpmsg", rpdev);
	if (ret) {
		pr_err("%s: register interrupt %d failed, rc %d\n",
			__func__, irq, ret);
		return ret;
	}

	if (rpdev->variant == IMX6SX || rpdev->variant == IMX7ULP) {
		rpdev->mu_clk = NULL;
	} else {
		rpdev->mu_clk = of_clk_get(np_mu, 0);
		if (IS_ERR(rpdev->mu_clk)) {
			pr_err("mu clock source missing or invalid\n");
			return PTR_ERR(rpdev->mu_clk);
		}
		ret = clk_prepare_enable(rpdev->mu_clk);
		if (ret) {
			pr_err("unable to enable mu clock\n");
			return ret;
		}
	}

	ret = imx_rpmsg_mu_init(rpdev);
	if (ret) {
		pr_err("unable to initialize mu module.\n");
		goto vdev_err_out;
	}
	INIT_DELAYED_WORK(&(rpdev->rpmsg_work), rpmsg_work_handler);
	BLOCKING_INIT_NOTIFIER_HEAD(&(rpdev->notifier));

	pr_info("MU is ready for cross core communication!\n");

	ret = of_property_read_u32(np, "vdev-nums", &rpdev->vdev_nums);
	if (ret)
		rpdev->vdev_nums = 1;
	if (rpdev->vdev_nums > MAX_VDEV_NUMS) {
		pr_err("vdev-nums exceed the max %d\n", MAX_VDEV_NUMS);
		ret = -EINVAL;
		goto vdev_err_out;
	}
	rpdev->first_notify = rpdev->vdev_nums;

	if (!strcmp(rpdev->rproc_name, "m4")) {
		ret = set_vring_phy_buf(pdev, rpdev,
					rpdev->vdev_nums);
		if (ret) {
			pr_err("No vring buffer.\n");
			ret = -ENOMEM;
			goto vdev_err_out;
		}
	} else {
		pr_err("No remote m4 processor.\n");
		ret = -ENODEV;
		goto vdev_err_out;
	}


	if (rpdev->variant == IMX8QM || rpdev->variant == IMX8QXP) {
		if (of_reserved_mem_device_init(&pdev->dev)) {
			dev_err(&pdev->dev,
			"dev doesn't have specific DMA pool.\n");
			ret = -ENOMEM;
			goto vdev_err_out;
		}
	}
	for (j = 0; j < rpdev->vdev_nums; j++) {
		pr_debug("%s rpdev%d vdev%d: vring0 0x%x, vring1 0x%x\n",
			 __func__, rpdev->core_id, rpdev->vdev_nums,
			 rpdev->ivdev[j]->vring[0],
			 rpdev->ivdev[j]->vring[1]);
		rpdev->ivdev[j]->vdev.id.device = VIRTIO_ID_RPMSG;
		rpdev->ivdev[j]->vdev.config = &imx_rpmsg_config_ops;
		rpdev->pdev = pdev;
		rpdev->ivdev[j]->vdev.dev.parent = &pdev->dev;
		rpdev->ivdev[j]->vdev.dev.release = imx_rpmsg_vproc_release;
		rpdev->ivdev[j]->base_vq_id = j * 2;
		rpdev->ivdev[j]->vproc_id = rpdev->core_id;

		ret = register_virtio_device(&rpdev->ivdev[j]->vdev);
		if (ret) {
			pr_err("%s failed to register rpdev: %d\n",
					__func__, ret);
			goto err_out;
		}
	}

	platform_set_drvdata(pdev, rpdev);

#ifdef CONFIG_ARCH_MXC_ARM64
	if (rpdev->variant == IMX8QXP || rpdev->variant == IMX8QM) {
		uint32_t mu_id;
		sc_err_t sciErr;
		static sc_ipc_t mu_ipchandle;
		uint32_t irq_status;

		/* Get muB partition id and enable irq in SCFW then */
		if (of_property_read_u32(np, "mub-partition",
					&rpdev->mub_partition))
			rpdev->mub_partition = 3; /* default partition 3 */

		sciErr = sc_ipc_getMuID(&mu_id);
		if (sciErr != SC_ERR_NONE) {
			pr_err("can't obtain mu id: %d\n", sciErr);
			return sciErr;
		}

		sciErr = sc_ipc_open(&mu_ipchandle, mu_id);

		if (sciErr != SC_ERR_NONE) {
			pr_err("can't get ipc handler: %d\n", sciErr);
			return sciErr;
		};

		/* Clear any pending partition reset interrupt during
		 * rpmsg probe.
		 */
		sciErr = sc_irq_status(mu_ipchandle, SC_R_MU_1A,
				       SC_IRQ_GROUP_REBOOTED,
				       &irq_status);
		if (sciErr != SC_ERR_NONE)
			pr_info("Cannot get partition reboot interrupt status\n");

		/* Request for the partition reset interrupt. */
		sciErr = sc_irq_enable(mu_ipchandle, SC_R_MU_1A,
				       SC_IRQ_GROUP_REBOOTED,
				       BIT(rpdev->mub_partition), true);
		if (sciErr)
			pr_info("Cannot request partition reset interrupt\n");

		return register_scu_notifier(rpdev->pnotifier);

	}
#endif

	return ret;

err_out:
	if (rpdev->variant == IMX8QM || rpdev->variant == IMX8QXP)
		of_reserved_mem_device_release(&pdev->dev);
vdev_err_out:
	if (rpdev->mu_clk)
		clk_disable_unprepare(rpdev->mu_clk);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int imx_rpmsg_noirq_suspend(struct device *dev)
{
	struct imx_rpmsg_vproc *rpdev = dev_get_drvdata(dev);

	rpdev->none_suspend = 0;
	if (rpdev->mu_clk)
		clk_disable_unprepare(rpdev->mu_clk);

	return 0;
}

static int imx_rpmsg_noirq_resume(struct device *dev)
{
	struct imx_rpmsg_vproc *rpdev = dev_get_drvdata(dev);
	int ret;

	if (rpdev->mu_clk) {
		ret = clk_prepare_enable(rpdev->mu_clk);
		if (ret) {
			pr_err("unable to enable mu clock\n");
			return ret;
		}
	}
	ret = imx_rpmsg_mu_init(rpdev);
	rpdev->none_suspend = 1;

	return ret;
}
#endif

static const struct dev_pm_ops imx_rpmsg_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(imx_rpmsg_noirq_suspend,
				      imx_rpmsg_noirq_resume)
};

static struct platform_driver imx_rpmsg_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "imx-rpmsg",
		   .of_match_table = imx_rpmsg_dt_ids,
		   .pm = &imx_rpmsg_pm_ops,
		   },
	.probe = imx_rpmsg_probe,
};

static int __init imx_rpmsg_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_rpmsg_driver);
	if (ret)
		pr_err("Unable to initialize rpmsg driver\n");
	else
		pr_info("imx rpmsg driver is registered.\n");

	return ret;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("iMX remote processor messaging virtio device");
MODULE_LICENSE("GPL v2");
arch_initcall(imx_rpmsg_init);
