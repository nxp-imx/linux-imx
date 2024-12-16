// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 * Copyright (C) 2022, STMicroelectronics
 * Copyright (c) 2016, Linaro Ltd.
 * Copyright (c) 2012, Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2012, PetaLogix
 * Copyright (c) 2011, Texas Instruments, Inc.
 * Copyright (c) 2011, Google, Inc.
 *
 * Based on rpmsg performance statistics driver by Michal Simek, which in turn
 * was based on TI & Google OMX rpmsg driver.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/rpmsg.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/rpmsg.h>

#include "rpmsg_char.h"
#include "rpmsg_internal.h"
#include <linux/imx_rpmsg.h>

#define RPMSG_DEV_MAX	(MINORMASK + 1)
#define CHRE_CATEGORY (0x0CU)
#define CHRE_VERSION (0x0100U)

#define CHRE_RPMSG_REQUEST          0x0
#define CHRE_RPMSG_RESPONSE         0x1
#define CHRE_RPMSG_NOTIFICATION     0x2
#define CHRE_RPMSG_PAYLOAD_BUFSIZE  256

static dev_t rpmsg_major;

static DEFINE_IDA(rpmsg_ept_ida);
static DEFINE_IDA(rpmsg_minor_ida);

#define dev_to_eptdev(dev) container_of(dev, struct rpmsg_eptdev, dev)
#define cdev_to_eptdev(i_cdev) container_of(i_cdev, struct rpmsg_eptdev, cdev)
const char chre_ack_message[10] = "CHRE";

struct host_to_chre_msg {
	struct imx_rpmsg_head header;
	uint8_t reserved;
	uint8_t data[CHRE_RPMSG_PAYLOAD_BUFSIZE];
} __attribute__ ((__packed__));

typedef enum
{
	CHRE_MESSAGE_DATA = 0U,
	CHRE_MESSAGE_FINISH,
	CHRE_ACK,
	CHRE_POWERMODE, // TODO
} chre_cmd_t;

struct chre_message_head {
	u8 cmd;
	u16 messagelen;
} __attribute__ ((__packed__));

/**
 * struct rpmsg_eptdev - endpoint device context
 * @dev:	endpoint device
 * @cdev:	cdev for the endpoint device
 * @rpdev:	underlaying rpmsg device
 * @chinfo:	info used to open the endpoint
 * @ept_lock:	synchronization of @ept modifications
 * @ept:	rpmsg endpoint reference, when open
 * @queue_lock:	synchronization of @queue operations
 * @queue:	incoming message queue
 * @readq:	wait object for incoming queue
 * @default_ept: set to channel default endpoint if the default endpoint should be re-used
 *              on device open to prevent endpoint address update.
 * remote_flow_restricted: to indicate if the remote has requested for flow to be limited
 * remote_flow_updated: to indicate if the flow control has been requested
 */

struct rpmsg_eptdev {
	struct device dev;
	struct cdev cdev;

	struct rpmsg_device *rpdev;
	struct rpmsg_channel_info chinfo;

	struct mutex ept_lock;
	struct rpmsg_endpoint *ept;
	struct rpmsg_endpoint *default_ept;

	spinlock_t queue_lock;
	struct sk_buff_head queue;
	wait_queue_head_t readq;

	bool remote_flow_restricted;
	bool remote_flow_updated;
};

static int rpmsg_chre_ept_cb(struct rpmsg_device *rpdev, void *buf, int len,
			void *priv, u32 addr)
{
	struct rpmsg_eptdev *eptdev = priv;
	struct device *dev = &eptdev->dev;
	u16 total_size = 0;
	/* The offset marks the position of each data from chre in the message. */
	static u16 offset = 0;
	/* The message_buf stored the full chre message */
	static u8* message_buf = NULL;

	/* collect all of message */
	struct chre_message_head *header = (struct chre_message_head *)buf;
	total_size = header->messagelen;
	len = len - sizeof(struct chre_message_head);
	if (header->cmd == CHRE_MESSAGE_DATA) {
		if (message_buf == NULL) {
			message_buf = kzalloc(total_size, GFP_KERNEL);
		}
		memcpy(message_buf + offset, buf + sizeof(struct chre_message_head), len);
		offset += len;
	} else if (header->cmd == CHRE_MESSAGE_FINISH) {
		struct sk_buff *skb;
		skb = alloc_skb(total_size, GFP_ATOMIC);
		if (!skb) {
			dev_err(dev, "skb allcate failed\n");
			return -ENOMEM;
		}
		/* Copy the previous content to skb */
		if (message_buf != NULL)
			skb_put_data(skb, message_buf, offset);
		skb_put_data(skb, buf + sizeof(struct chre_message_head), len);
		spin_lock(&eptdev->queue_lock);
		skb_queue_tail(&eptdev->queue, skb);
		spin_unlock(&eptdev->queue_lock);
		/* finish release memory */
		kfree(message_buf);
		message_buf = NULL;
		offset = 0;
		/* wake up any blocking processes, waiting for new data */
		wake_up_interruptible(&eptdev->readq);
	} else if (header->cmd == CHRE_ACK) {
		if (memcmp(chre_ack_message, buf + sizeof(struct chre_message_head), sizeof(chre_ack_message)) == 0) {
			dev_info(dev, "chre communicate with host successed\n");
		} else {
			dev_err(dev, "chre communicate with host failed\n");
		}
	} else {
		dev_err(dev, "command from chre is invalid\n");
	}

	return 0;
}

static int rpmsg_ept_flow_cb(struct rpmsg_device *rpdev, void *priv, bool enable)
{
	struct rpmsg_eptdev *eptdev = priv;

	eptdev->remote_flow_restricted = enable;
	eptdev->remote_flow_updated = true;

	wake_up_interruptible(&eptdev->readq);

	return 0;
}

static int rpmsg_eptdev_open(struct inode *inode, struct file *filp)
{
	struct rpmsg_eptdev *eptdev = cdev_to_eptdev(inode->i_cdev);
	struct rpmsg_endpoint *ept;
	struct rpmsg_device *rpdev = eptdev->rpdev;
	struct device *dev = &eptdev->dev;

	mutex_lock(&eptdev->ept_lock);
	if (eptdev->ept) {
		mutex_unlock(&eptdev->ept_lock);
		return -EBUSY;
	}

	if (!eptdev->rpdev) {
		mutex_unlock(&eptdev->ept_lock);
		return -ENETRESET;
	}

	get_device(dev);

	/*
	 * If the default_ept is set, the rpmsg device default endpoint is used.
	 * Else a new endpoint is created on open that will be destroyed on release.
	 */
	if (eptdev->default_ept)
		ept = eptdev->default_ept;
	else
		ept = rpmsg_create_ept(rpdev, rpmsg_chre_ept_cb, eptdev, eptdev->chinfo);

	if (!ept) {
		dev_err(dev, "failed to open %s\n", eptdev->chinfo.name);
		put_device(dev);
		mutex_unlock(&eptdev->ept_lock);
		return -EINVAL;
	}

	ept->flow_cb = rpmsg_ept_flow_cb;
	eptdev->ept = ept;
	filp->private_data = eptdev;
	mutex_unlock(&eptdev->ept_lock);

	return 0;
}

static int rpmsg_eptdev_release(struct inode *inode, struct file *filp)
{
	struct rpmsg_eptdev *eptdev = cdev_to_eptdev(inode->i_cdev);
	struct device *dev = &eptdev->dev;

	/* Close the endpoint, if it's not already destroyed by the parent */
	mutex_lock(&eptdev->ept_lock);
	if (eptdev->ept) {
		if (!eptdev->default_ept)
			rpmsg_destroy_ept(eptdev->ept);
		eptdev->ept = NULL;
	}
	mutex_unlock(&eptdev->ept_lock);
	eptdev->remote_flow_updated = false;

	/* Discard all SKBs */
	skb_queue_purge(&eptdev->queue);

	put_device(dev);

	return 0;
}

static ssize_t rpmsg_eptdev_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct file *filp = iocb->ki_filp;
	struct rpmsg_eptdev *eptdev = filp->private_data;
	unsigned long flags;
	struct sk_buff *skb;
	int use;

	if (!eptdev->ept)
		return -EPIPE;

	spin_lock_irqsave(&eptdev->queue_lock, flags);

	/* Wait for data in the queue */
	if (skb_queue_empty(&eptdev->queue)) {
		spin_unlock_irqrestore(&eptdev->queue_lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* Wait until we get data or the endpoint goes away */
		if (wait_event_interruptible(eptdev->readq,
					     !skb_queue_empty(&eptdev->queue) ||
					     !eptdev->ept))
			return -ERESTARTSYS;

		/* We lost the endpoint while waiting */
		if (!eptdev->ept)
			return -EPIPE;

		spin_lock_irqsave(&eptdev->queue_lock, flags);
	}

	skb = skb_dequeue(&eptdev->queue);
	spin_unlock_irqrestore(&eptdev->queue_lock, flags);
	if (!skb)
		return -EFAULT;

	use = min_t(size_t, iov_iter_count(to), skb->len);

	if (copy_to_iter(skb->data, use, to) != use)
		use = -EFAULT;

	kfree_skb(skb);

	return use;
}

static ssize_t rpmsg_eptdev_write_iter(struct kiocb *iocb,
				       struct iov_iter *from)
{
	struct file *filp = iocb->ki_filp;
	struct rpmsg_eptdev *eptdev = filp->private_data;
	size_t len = iov_iter_count(from);
	u16 ret;
	struct host_to_chre_msg msg = {};
	void *kbuf;
	/* Record the position of each sub-message in the full message */
	u16 cur_msg_pos = 0;

	if (mutex_lock_interruptible(&eptdev->ept_lock)) {
		ret = -ERESTARTSYS;
		goto free_kbuf;
	}

	if (!eptdev->ept) {
		ret = -EPIPE;
		goto unlock_eptdev;
	}

	kbuf = kzalloc(len, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	/* create rpmsg struct for chre */
	msg.header.cate = CHRE_CATEGORY;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = CHRE_RPMSG_REQUEST;
	msg.header.cmd = CHRE_MESSAGE_DATA;
	msg.header.reserved[0] = len & 0xff;
	msg.header.reserved[1] = (len & 0xff00) >> 8;
	/* copy full data into kernel */
	if (!copy_from_iter_full(kbuf, len, from)) {
		ret = -EFAULT;
		goto free_kbuf;
	}
	while (cur_msg_pos <= len) {
		if (((len - cur_msg_pos) < CHRE_RPMSG_PAYLOAD_BUFSIZE) && ((len - cur_msg_pos) >= 0)) {
			if (len - cur_msg_pos)
				memcpy(msg.data, kbuf + cur_msg_pos, len - cur_msg_pos);
			msg.header.cmd = CHRE_MESSAGE_FINISH;
			msg.header.reserved[2] = len - cur_msg_pos;
		} else {
			memcpy(msg.data, kbuf + cur_msg_pos, CHRE_RPMSG_PAYLOAD_BUFSIZE);
		}
		cur_msg_pos += CHRE_RPMSG_PAYLOAD_BUFSIZE;
		if (filp->f_flags & O_NONBLOCK) {
			ret = rpmsg_trysendto(eptdev->ept, &msg, sizeof(struct host_to_chre_msg), eptdev->chinfo.dst);
			if (ret == -ENOMEM) {
				ret = -EAGAIN;
			}
		} else {
			ret = rpmsg_sendto(eptdev->ept, &msg, sizeof(struct host_to_chre_msg), eptdev->chinfo.dst);
		}
	}
unlock_eptdev:
	mutex_unlock(&eptdev->ept_lock);

free_kbuf:
	kfree(kbuf);
	return ret < 0 ? ret : len;
}

static __poll_t rpmsg_eptdev_poll(struct file *filp, poll_table *wait)
{
	struct rpmsg_eptdev *eptdev = filp->private_data;
	__poll_t mask = 0;

	if (!eptdev->ept)
		return EPOLLERR;

	poll_wait(filp, &eptdev->readq, wait);

	if (!skb_queue_empty(&eptdev->queue))
		mask |= EPOLLIN | EPOLLRDNORM;

	if (eptdev->remote_flow_updated)
		mask |= EPOLLPRI;

	mutex_lock(&eptdev->ept_lock);
	mask |= rpmsg_poll(eptdev->ept, filp, wait);
	mutex_unlock(&eptdev->ept_lock);

	return mask;
}

static long rpmsg_eptdev_ioctl(struct file *fp, unsigned int cmd,
			       unsigned long arg)
{
	return 0;
}

static const struct file_operations rpmsg_eptdev_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_eptdev_open,
	.release = rpmsg_eptdev_release,
	.read_iter = rpmsg_eptdev_read_iter,
	.write_iter = rpmsg_eptdev_write_iter,
	.poll = rpmsg_eptdev_poll,
	.unlocked_ioctl = rpmsg_eptdev_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void rpmsg_eptdev_release_device(struct device *dev)
{
	struct rpmsg_eptdev *eptdev = dev_to_eptdev(dev);

	ida_simple_remove(&rpmsg_ept_ida, dev->id);
	ida_simple_remove(&rpmsg_minor_ida, MINOR(eptdev->dev.devt));
	kfree(eptdev);
}

static struct rpmsg_eptdev *rpmsg_chre_eptdev_alloc(struct rpmsg_device *rpdev,
						      struct device *parent)
{
	struct rpmsg_eptdev *eptdev;
	struct device *dev;

	eptdev = kzalloc(sizeof(*eptdev), GFP_KERNEL);
	if (!eptdev)
		return ERR_PTR(-ENOMEM);

	dev = &eptdev->dev;
	eptdev->rpdev = rpdev;

	mutex_init(&eptdev->ept_lock);
	spin_lock_init(&eptdev->queue_lock);
	skb_queue_head_init(&eptdev->queue);
	init_waitqueue_head(&eptdev->readq);

	device_initialize(dev);
	dev->class = &rpmsg_class;
	dev->parent = parent;
	dev_set_drvdata(dev, eptdev);

	cdev_init(&eptdev->cdev, &rpmsg_eptdev_fops);
	eptdev->cdev.owner = THIS_MODULE;

	return eptdev;
}

static int rpmsg_chre_eptdev_add(struct rpmsg_eptdev *eptdev, struct rpmsg_channel_info chinfo)
{
	struct device *dev = &eptdev->dev;
	int ret;

	eptdev->chinfo = chinfo;

	ret = ida_simple_get(&rpmsg_minor_ida, 0, RPMSG_DEV_MAX, GFP_KERNEL);
	if (ret < 0)
		goto free_eptdev;
	dev->devt = MKDEV(MAJOR(rpmsg_major), ret);

	ret = ida_simple_get(&rpmsg_ept_ida, 0, 0, GFP_KERNEL);
	if (ret < 0)
		goto free_minor_ida;
	dev->id = ret;
	dev_set_name(dev, "rpmsg_chre");

	ret = cdev_device_add(&eptdev->cdev, &eptdev->dev);
	if (ret)
		goto free_ept_ida;

	/* We can now rely on the release function for cleanup */
	dev->release = rpmsg_eptdev_release_device;

	return ret;

free_ept_ida:
	ida_simple_remove(&rpmsg_ept_ida, dev->id);
free_minor_ida:
	ida_simple_remove(&rpmsg_minor_ida, MINOR(dev->devt));
free_eptdev:
	put_device(dev);
	kfree(eptdev);

	return ret;
}

static int rpmsg_chre_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo;
	struct rpmsg_eptdev *eptdev;
	struct device *dev = &rpdev->dev;

	memcpy(chinfo.name, rpdev->id.name, RPMSG_NAME_SIZE);
	chinfo.src = rpdev->src;
	chinfo.dst = rpdev->dst;

	eptdev = rpmsg_chre_eptdev_alloc(rpdev, dev);
	if (IS_ERR(eptdev))
		return PTR_ERR(eptdev);

	/* Set the default_ept to the rpmsg device endpoint */
	eptdev->default_ept = rpdev->ept;

	/*
	 * The rpmsg_ept_cb uses *priv parameter to get its rpmsg_eptdev context.
	 * Storedit in default_ept *priv field.
	 */
	eptdev->default_ept->priv = eptdev;

	return rpmsg_chre_eptdev_add(eptdev, chinfo);
}

static void rpmsg_chre_remove(struct rpmsg_device *rpdev)
{
	int ret;

	ret = device_for_each_child(&rpdev->dev, NULL, rpmsg_chrdev_eptdev_destroy);
	if (ret)
		dev_warn(&rpdev->dev, "failed to destroy endpoints: %d\n", ret);
}

static struct rpmsg_device_id rpmsg_chre_id_table[] = {
	{ .name	= "rpmsg-chre" },
	{ },
};

static struct rpmsg_driver rpmsg_chre_driver = {
	.probe = rpmsg_chre_probe,
	.remove = rpmsg_chre_remove,
	.callback = rpmsg_chre_ept_cb,
	.id_table = rpmsg_chre_id_table,
	.drv.name = "rpmsg_chredev",
};

static int rpmsg_chre_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&rpmsg_major, 0, RPMSG_DEV_MAX, "rpmsg_chre");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		return ret;
	}

	ret = register_rpmsg_driver(&rpmsg_chre_driver);
	if (ret < 0) {
		pr_err("rpmsg: failed to register rpmsg chre driver\n");
		goto free_region;
	}

	return 0;

free_region:
	unregister_chrdev_region(rpmsg_major, RPMSG_DEV_MAX);

	return ret;
}
postcore_initcall(rpmsg_chre_init);

static void rpmsg_chrdev_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_chre_driver);
	unregister_chrdev_region(rpmsg_major, RPMSG_DEV_MAX);
}
module_exit(rpmsg_chrdev_exit);

MODULE_ALIAS("rpmsg:rpmsg_chre");
MODULE_LICENSE("GPL v2");
