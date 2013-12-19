/*
 * Copyright (C) 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/ir.h>

#define IR_MINOR 255

static DEFINE_MUTEX(ir_mutex);

static int ir_open_cnt;	/* #times opened */

static int ir_open(struct inode *inode, struct file *file)
{
    int ret;
    mutex_lock(&ir_mutex);
	if (0 == ir_open_cnt) {
		ret = 0;
		ir_open_cnt++;
	} else {
		ret = -1;
	}
	mutex_unlock(&ir_mutex);
	return ret;
}

static int ir_release(struct inode *inode, struct file *file)
{
	mutex_lock(&ir_mutex);
	ir_open_cnt--;
	mutex_unlock(&ir_mutex);
	return 0;
}


static long ir_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *c;
	void *devdata;
	int ret = 0;

	mutex_lock(&ir_mutex);
	c = file->private_data;
	if (!c) {
		mutex_unlock(&ir_mutex);
		return -EINVAL;
	}
	devdata = dev_get_drvdata(c->this_device);
	if (!devdata) {
		mutex_unlock(&ir_mutex);
		return -EINVAL;
	}

	switch (cmd) {
	case IR_GET_CARRIER_FREQS:
	{
		struct ir_carrier_freq ir_carrier;
		int id;

		if (copy_from_user(&ir_carrier,
							arg,
							sizeof(struct ir_carrier_freq))) {
			ret = -EINVAL;
		} else {
			id = ir_carrier.id;
			ret = ir_get_carrier_range(id,
										&ir_carrier.min,
										&ir_carrier.max);
			if (0 == ret) {
				if (copy_to_user(arg, &ir_carrier, sizeof(ir_carrier)))
					ret = -EINVAL;
			}
		}
		break;
	}
	case IR_GET_CARRIER_FREQS_NUM:
	{
		int num = ir_get_num_carrier_freqs();
		if (copy_to_user(arg, &num, 1))
			ret = -EINVAL;
		break;
	}
	case IR_CFG_CARRIER:
	{
		int carrier = (int)(arg);
		ir_config(devdata, carrier);
		break;
	}
	case IR_DATA_TRANSMIT:
	{
		struct ir_data_pattern p;
		int *pattern;

		if (!arg) {
			mutex_unlock(&ir_mutex);
			return -EINVAL;
		}

		if (copy_from_user(&p, arg, sizeof(struct ir_data_pattern))) {
			mutex_unlock(&ir_mutex);
			return -EINVAL;
		}

		pattern = kzalloc(p.len*sizeof(int), GFP_KERNEL);
		if (!pattern) {
			mutex_unlock(&ir_mutex);
			printk(KERN_ERR "memory allocate for IR pattern error.\n");
			return -EINVAL;
		}

		if (copy_from_user(pattern, p.pattern, p.len*sizeof(int))) {
			printk(KERN_ERR "memory allocate for IR pattern error.\n");
			ret = -EINVAL;
		} else {
			ir_transmit(devdata, p.len, pattern, 1);
		}
		kfree(pattern);
		break;
	}
	default:
		break;
	}
	mutex_unlock(&ir_mutex);
	return ret;
}

static const struct file_operations ir_fops = {
	.owner		= THIS_MODULE,
	.open       = ir_open,
	.release    = ir_release,
	.unlocked_ioctl	= ir_ioctl,
};

static struct miscdevice ir_miscdev = {
	.minor = IR_MINOR,
	.name = "ir",
	.fops = &ir_fops,
};

void ir_device_register(const char *name, struct device *parent, void *devdata)
{
    int ret;
	ret = misc_register(&ir_miscdev);
	dev_set_drvdata(ir_miscdev.this_device, devdata);
	return ret;
}
EXPORT_SYMBOL(ir_device_register);

void ir_device_unregister(void)
{
	misc_deregister(&ir_miscdev);
}
EXPORT_SYMBOL(ir_device_unregister);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IR Low Abstract Layer");
MODULE_LICENSE("GPL");
