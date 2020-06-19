// SPDX-License-Identifier: GPL-2.0
/*
 * Management Complex (MC) userspace support
 *
 * Copyright 2018 NXP
 *
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include "fsl-mc-private.h"

struct uapi_priv_data {
	struct fsl_mc_uapi *uapi;
	struct fsl_mc_io *mc_io;
};

static int fsl_mc_uapi_send_command(unsigned long arg,
				    struct fsl_mc_io *mc_io)
{
	struct fsl_mc_command mc_cmd;
	int error;

	error = copy_from_user(&mc_cmd, (void __user *)arg, sizeof(mc_cmd));
	if (error)
		return -EFAULT;

	error = mc_send_command(mc_io, &mc_cmd);
	if (error)
		return error;

	error = copy_to_user((void __user *)arg, &mc_cmd, sizeof(mc_cmd));
	if (error)
		return -EFAULT;

	return 0;
}

static int fsl_mc_uapi_dev_open(struct inode *inode, struct file *filep)
{
	struct fsl_mc_device *root_mc_device;
	struct uapi_priv_data *priv_data;
	struct fsl_mc_io *dynamic_mc_io;
	struct fsl_mc_uapi *mc_uapi;
	struct fsl_mc_bus *mc_bus;
	int error;

	priv_data = kzalloc(sizeof(*priv_data), GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;

	mc_uapi = container_of(filep->private_data, struct fsl_mc_uapi, misc);
	mc_bus = container_of(mc_uapi, struct fsl_mc_bus, uapi_misc);
	root_mc_device = &mc_bus->mc_dev;

	mutex_lock(&mc_uapi->mutex);

	if (!mc_uapi->local_instance_in_use) {
		priv_data->mc_io = mc_uapi->static_mc_io;
		mc_uapi->local_instance_in_use = 1;
	} else {
		error = fsl_mc_portal_allocate(root_mc_device, 0,
					       &dynamic_mc_io);
		if (error) {
			dev_dbg(&root_mc_device->dev,
				"Could not allocate MC portal\n");
			goto error_portal_allocate;
		}

		priv_data->mc_io = dynamic_mc_io;
	}
	priv_data->uapi = mc_uapi;
	filep->private_data = priv_data;

	mutex_unlock(&mc_uapi->mutex);

	return 0;

error_portal_allocate:
	mutex_unlock(&mc_uapi->mutex);
	kfree(priv_data);

	return error;
}

static int fsl_mc_uapi_dev_release(struct inode *inode, struct file *filep)
{
	struct uapi_priv_data *priv_data;
	struct fsl_mc_uapi *mc_uapi;
	struct fsl_mc_io *mc_io;

	priv_data = filep->private_data;
	mc_uapi = priv_data->uapi;
	mc_io = priv_data->mc_io;

	mutex_lock(&mc_uapi->mutex);

	if (mc_io == mc_uapi->static_mc_io)
		mc_uapi->local_instance_in_use = 0;
	else
		fsl_mc_portal_free(mc_io);

	kfree(filep->private_data);
	filep->private_data =  NULL;

	mutex_unlock(&mc_uapi->mutex);

	return 0;
}

static long fsl_mc_uapi_dev_ioctl(struct file *file,
				  unsigned int cmd,
				  unsigned long arg)
{
	struct uapi_priv_data *priv_data = file->private_data;
	struct fsl_mc_device *root_mc_device;
	struct fsl_mc_bus *mc_bus;
	int error;

	mc_bus = container_of(priv_data->uapi, struct fsl_mc_bus, uapi_misc);
	root_mc_device = &mc_bus->mc_dev;

	switch (cmd) {
	case FSL_MC_SEND_MC_COMMAND:
		error = fsl_mc_uapi_send_command(arg, priv_data->mc_io);
		break;
	default:
		dev_dbg(&root_mc_device->dev, "unexpected ioctl call number\n");
		error = -EINVAL;
	}

	return error;
}

static const struct file_operations fsl_mc_uapi_dev_fops = {
	.owner = THIS_MODULE,
	.open = fsl_mc_uapi_dev_open,
	.release = fsl_mc_uapi_dev_release,
	.unlocked_ioctl = fsl_mc_uapi_dev_ioctl,
};

int fsl_mc_uapi_create_device_file(struct fsl_mc_bus *mc_bus)
{
	struct fsl_mc_device *mc_dev = &mc_bus->mc_dev;
	struct fsl_mc_uapi *mc_uapi = &mc_bus->uapi_misc;
	int error;

	mc_uapi->misc.minor = MISC_DYNAMIC_MINOR;
	mc_uapi->misc.name = dev_name(&mc_dev->dev);
	mc_uapi->misc.fops = &fsl_mc_uapi_dev_fops;

	error = misc_register(&mc_uapi->misc);
	if (error)
		return error;

	mc_uapi->static_mc_io = mc_bus->mc_dev.mc_io;

	mutex_init(&mc_uapi->mutex);

	return 0;
}

void fsl_mc_uapi_remove_device_file(struct fsl_mc_bus *mc_bus)
{
	misc_deregister(&mc_bus->uapi_misc.misc);
}
