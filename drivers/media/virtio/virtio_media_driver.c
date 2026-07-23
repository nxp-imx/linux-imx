// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0+

/*
 * Virtio-media driver.
 *
 * Copyright (c) 2024-2026 Google LLC.
 */

#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

#include "uapi/linux/virtio_media.h"
#include "virtio_media.h"

static void commandq_callback(struct virtqueue *vq)
{
}

static void eventq_callback(struct virtqueue *vq)
{
}

static const struct v4l2_file_operations virtio_media_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = v4l2_fh_release,
};

static int virtio_media_probe(struct virtio_device *virtio_dev)
{
	struct device *dev = &virtio_dev->dev;
	struct virtqueue *vqs[2];
	static struct virtqueue_info vq_info[2] = {
		{
			.name = "command",
			.callback = commandq_callback,
		},
		{
			.name = "event",
			.callback = eventq_callback,
		},
	};
	struct virtio_media *vv;
	struct video_device *vd;
	int ret;

	vv = devm_kzalloc(dev, sizeof(*vv), GFP_KERNEL);
	if (!vv)
		return -ENOMEM;

	INIT_LIST_HEAD(&vv->sessions);
	mutex_init(&vv->sessions_lock);
	mutex_init(&vv->events_lock);
	mutex_init(&vv->vlock);

	vv->virtio_dev = virtio_dev;
	virtio_dev->priv = vv;

	init_waitqueue_head(&vv->wq);

	ret = v4l2_device_register(dev, &vv->v4l2_dev);
	if (ret)
		return ret;

	ret = virtio_find_vqs(virtio_dev, 2, vqs, vq_info, NULL);
	if (ret)
		goto err_find_vqs;

	vv->commandq = vqs[0];
	vv->eventq = vqs[1];

	vd = &vv->video_dev;
	vd->v4l2_dev = &vv->v4l2_dev;
	vd->vfl_type = VFL_TYPE_VIDEO;
	vd->fops = &virtio_media_fops;
	vd->release = video_device_release_empty;
	strscpy(vd->name, "virtio-media", sizeof(vd->name));

	video_set_drvdata(vd, vv);

	vd->device_caps = virtio_cread32(virtio_dev, 0);
	if (vd->device_caps & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE))
		vd->vfl_dir = VFL_DIR_M2M;
	else if (vd->device_caps &
		 (V4L2_CAP_VIDEO_OUTPUT | V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE))
		vd->vfl_dir = VFL_DIR_TX;
	else
		vd->vfl_dir = VFL_DIR_RX;

	ret = video_register_device(vd, virtio_cread32(virtio_dev, 4), 0);
	if (ret)
		goto err_register_device;

	virtio_device_ready(virtio_dev);

	return 0;

err_register_device:
	virtio_dev->config->del_vqs(virtio_dev);
err_find_vqs:
	v4l2_device_unregister(&vv->v4l2_dev);
	return ret;
}

static void virtio_media_remove(struct virtio_device *virtio_dev)
{
	struct virtio_media *vv = virtio_dev->priv;

	virtio_reset_device(virtio_dev);
	v4l2_device_unregister(&vv->v4l2_dev);
	virtio_dev->config->del_vqs(virtio_dev);
	video_unregister_device(&vv->video_dev);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_MEDIA, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {};

static struct virtio_driver virtio_media_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name = VIRTIO_MEDIA_DEFAULT_DRIVER_NAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.probe = virtio_media_probe,
	.remove = virtio_media_remove,
};

module_virtio_driver(virtio_media_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("virtio media driver");
MODULE_AUTHOR("Alexandre Courbot <gnurou@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
