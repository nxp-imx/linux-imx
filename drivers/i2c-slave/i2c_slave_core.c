/*
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include "i2c_slave_device.h"

int i2c_slave_major;

static ssize_t i2c_slave_read(struct file *fd, char __user *buf, size_t len,
			      loff_t *ptr)
{
	i2c_slave_device_t *dev;
	void *kbuf;
	int ret;

	if (len == 0)
		return 0;

	kbuf = kmalloc(len, GFP_KERNEL);
	if (!kbuf) {
		ret = -ENOMEM;
		goto error0;
	}

	dev = (i2c_slave_device_t *) fd->private_data;
	if (!dev) {
		ret = -ENODEV;
		goto error1;
	}

	ret = i2c_slave_device_read(dev, len, kbuf);
	if (ret <= 0 || copy_to_user(buf, kbuf, len)) {
		ret = -EFAULT;
	}

      error1:
	kfree(kbuf);
      error0:
	return ret;
}

static ssize_t i2c_slave_write(struct file *fd, const char __user *buf,
			       size_t len, loff_t *ptr)
{
	i2c_slave_device_t *dev;
	void *kbuf;
	int ret;

	if (len == 0)
		return 0;

	kbuf = kmalloc(len, GFP_KERNEL);
	if (!kbuf) {
		ret = -ENOMEM;
		goto error0;
	}
	if (copy_from_user(kbuf, buf, len)) {
		ret = -EFAULT;
		goto error1;
	}

	dev = (i2c_slave_device_t *) fd->private_data;
	if (!dev) {
		ret = -ENODEV;
		goto error1;
	}

	ret = i2c_slave_device_write(dev, len, (u8 *) kbuf);

      error1:
	kfree(kbuf);
      error0:
	return ret;
}

static int i2c_slave_ioctl(struct inode *inode, struct file *fd,
			   unsigned code, unsigned long value)
{
	/*todo */
	return 0;
}

static int i2c_slave_open(struct inode *inode, struct file *fd)
{
	int ret;
	unsigned int id;
	i2c_slave_device_t *dev;
	id = iminor(inode);

	if (id >= I2C_SLAVE_DEVICE_MAX) {
		ret = -ENODEV;
		goto error;
	}

	dev = i2c_slave_device_find(id);
	if (!dev) {
		ret = -ENODEV;
		goto error;
	}

	i2c_slave_rb_clear(dev->receive_buffer);
	i2c_slave_rb_clear(dev->send_buffer);

	if (i2c_slave_device_start(dev)) {
		ret = -EBUSY;
		goto error;
	}

	fd->private_data = (void *)dev;
	ret = 0;

      error:
	return ret;
}

static int i2c_slave_release(struct inode *inode, struct file *fd)
{
	int ret;
	unsigned int id;
	i2c_slave_device_t *dev;
	id = iminor(inode);

	if (id >= I2C_SLAVE_DEVICE_MAX) {
		ret = -ENODEV;
		goto error;
	}

	dev = i2c_slave_device_find(id);
	if (!dev) {
		ret = -ENODEV;
		goto error;
	}

	if (i2c_slave_device_stop(dev)) {
		ret = -EBUSY;
		goto error;
	}

	ret = 0;

      error:
	return ret;
}

static const struct file_operations i2c_slave_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = i2c_slave_read,
	.write = i2c_slave_write,
	.ioctl = i2c_slave_ioctl,
	.open = i2c_slave_open,
	.release = i2c_slave_release,
};

static int i2c_slave_bus_match(struct device *dev, struct device_driver *drv)
{
	return 0;
}

/*
static int i2c_slave_bus_probe(struct device *dev)
{
	struct device_driver *driver = dev->driver;

	if (driver) {
		return driver->probe(dev);
	} else {
		dev_err(dev, "%s: no driver\n", __func__);
		return -ENODEV;
	}
}
*/

static int i2c_slave_bus_remove(struct device *dev)
{
	struct device_driver *driver = dev->driver;

	if (driver) {
		if (!driver->remove) {
			return 0;
		} else {
			return driver->remove(dev);
		}
	} else {

		dev_err(dev, "%s: no driver\n", __func__);
		return -ENODEV;
	}
}

static void i2c_slave_bus_shutdown(struct device *dev)
{
	struct device_driver *driver = dev->driver;

	if (driver) {

		driver->shutdown(dev);
	} else {

		dev_err(dev, "%s: no driver\n", __func__);
		return;
	}
}
static int i2c_slave_bus_suspend(struct device *dev, pm_message_t state)
{
	struct device_driver *driver = dev->driver;

	if (driver) {

		if (!driver->suspend) {
			return 0;
		} else {
			return driver->suspend(dev, state);
		}
	} else {

		dev_err(dev, "%s: no driver\n", __func__);
		return -ENODEV;
	}
}

static int i2c_slave_bus_resume(struct device *dev)
{
	struct device_driver *driver = dev->driver;

	if (driver) {

		if (!driver->resume) {
			return 0;
		} else {
			return driver->resume(dev);
		}
	} else {

		dev_err(dev, "%s: no driver\n", __func__);
		return -ENODEV;
	}
}

struct bus_type i2c_slave_bus_type = {
	.name = "i2c-slave",
	.match = i2c_slave_bus_match,
	.remove = i2c_slave_bus_remove,
	.shutdown = i2c_slave_bus_shutdown,
	.suspend = i2c_slave_bus_suspend,
	.resume = i2c_slave_bus_resume,
};

EXPORT_SYMBOL_GPL(i2c_slave_bus_type);

static int i2c_slave_driver_probe(struct device *dev)
{
	return 0;
}

static int i2c_slave_driver_remove(struct device *dev)
{
	return 0;
}

static int i2c_slave_driver_shutdown(struct device *dev)
{
	return 0;
}

static int i2c_slave_driver_suspend(struct device *dev, pm_message_t state)
{
	return 0;
}

static int i2c_slave_driver_resume(struct device *dev)
{
	return 0;
}

extern struct class *i2c_slave_class;

static struct device_driver i2c_slave_driver = {
	.name = "i2c-slave",
	.owner = THIS_MODULE,
	.bus = &i2c_slave_bus_type,
	.probe = i2c_slave_driver_probe,
	.remove = i2c_slave_driver_remove,
	.shutdown = i2c_slave_driver_shutdown,
	.suspend = i2c_slave_driver_suspend,
	.resume = i2c_slave_driver_resume,
};

static int __init i2c_slave_dev_init(void)
{
	int ret;

	printk(KERN_INFO "i2c slave /dev entries driver\n");

	ret = bus_register(&i2c_slave_bus_type);
	if (ret) {
		printk(KERN_ERR "%s: bus_register error\n", __func__);
		goto out;
	}

	i2c_slave_class = class_create(THIS_MODULE, "i2c-slave");
	if (IS_ERR(i2c_slave_class)) {
		pr_err("%s: class_create error\n", __func__);
		goto out_unreg_bus;
	}

	i2c_slave_major = register_chrdev(0, "i2c-slave", &i2c_slave_fops);
	if (i2c_slave_major <= 0) {
		pr_err("%s: register_chrdev error\n", __func__);
		goto out_unreg_class;
	}

	ret = driver_register(&i2c_slave_driver);
	if (ret) {
		pr_err("%s: driver_register error\n", __func__);
		goto out_unreg_chrdev;
	}

	return 0;

      out_unreg_chrdev:
	unregister_chrdev(i2c_slave_major, "i2c-slave");
      out_unreg_class:
	class_destroy(i2c_slave_class);
      out_unreg_bus:
	bus_unregister(&i2c_slave_bus_type);
      out:
	pr_err("%s: init error\n", __func__);
	return ret;
}

static void __exit i2c_dev_exit(void)
{
	driver_unregister(&i2c_slave_driver);
	class_destroy(i2c_slave_class);
	unregister_chrdev(i2c_slave_major, "i2c-slave");
	bus_unregister(&i2c_slave_bus_type);
}

module_init(i2c_slave_dev_init);
module_exit(i2c_dev_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("I2C Slave Driver Core");
MODULE_LICENSE("GPL");
