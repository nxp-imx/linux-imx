/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file agpsgpiodev.c
 *
 * @brief Main file for GPIO kernel module. Contains driver entry/exit
 *
 */

#include <linux/module.h>
#include <linux/fs.h>		/* Async notification */
#include <linux/uaccess.h>	/* for get_user, put_user, access_ok */
#include <linux/sched.h>	/* jiffies */
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include "agpsgpiodev.h"

extern void gpio_gps_active(void);
extern void gpio_gps_inactive(void);
extern int gpio_gps_access(int para);

struct mxc_gps_platform_data *mxc_gps_ioctrl_data;
static int Device_Open;		/* Only allow a single user of this device */
static struct cdev mxc_gps_cdev;
static dev_t agps_gpio_dev;
static struct class *gps_class;
static struct device *gps_class_dev;

/* Write GPIO from user space */
static int ioctl_writegpio(int arg)
{

	/* Bit 0 of arg identifies the GPIO pin to write:
	   0 = GPS_RESET_GPIO, 1 = GPS_POWER_GPIO.
	   Bit 1 of arg identifies the value to write (0 or 1). */

	/* Bit 2 should be 0 to show this access is write */
	return gpio_gps_access(arg & (~0x4));
}

/* Read GPIO from user space */
static int ioctl_readgpio(int arg)
{
	/* Bit 0 of arg identifies the GPIO pin to read:
	   0 = GPS_RESET_GPIO. 1 = GPS_POWER_GPIO
	   Bit 2 should be 1 to show this access is read */
	return gpio_gps_access(arg | 0x4);
}

static int device_open(struct inode *inode, struct file *fp)
{
	/* We don't want to talk to two processes at the same time. */
	if (Device_Open) {
		printk(KERN_DEBUG "device_open() - Returning EBUSY. \
			Device already open... \n");
		return -EBUSY;
	}
	Device_Open++;		/* BUGBUG : Not protected! */
	try_module_get(THIS_MODULE);

	return 0;
}

static int device_release(struct inode *inode, struct file *fp)
{
	/* We're now ready for our next caller */
	Device_Open--;
	module_put(THIS_MODULE);

	return 0;
}

static int device_ioctl(struct inode *inode, struct file *fp,
			unsigned int cmd, unsigned long arg)
{
	int err = 0;

	/* Extract the type and number bitfields, and don't decode wrong cmds.
	   Return ENOTTY (inappropriate ioctl) before access_ok() */
	if (_IOC_TYPE(cmd) != MAJOR_NUM) {
		printk(KERN_ERR
		       "device_ioctl() - Error! IOC_TYPE = %d. Expected %d\n",
		       _IOC_TYPE(cmd), MAJOR_NUM);
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > IOCTL_MAXNUMBER) {
		printk(KERN_ERR
		       "device_ioctl() - Error!"
		       "IOC_NR = %d greater than max supported(%d)\n",
		       _IOC_NR(cmd), IOCTL_MAXNUMBER);
		return -ENOTTY;
	}

	/* The direction is a bitmask, and VERIFY_WRITE catches R/W transfers.
	   `Type' is user-oriented, while access_ok is kernel-oriented, so the
	   concept of "read" and "write" is reversed. I think this is primarily
	   for good coding practice. You can easily do any kind of R/W access
	   without these checks and IOCTL code can be implemented "randomly"! */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));

	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		printk(KERN_ERR
		       "device_ioctl() - Error! User arg not valid"
		       "for selected access (R/W/RW). Cmd %d\n",
		       _IOC_TYPE(cmd));
		return -EFAULT;
	}

	/* Note: Read and writing data to user buffer can be done using regular
	   pointer stuff but we may also use get_user() or put_user() */

	/* Cmd and arg has been verified... */
	switch (cmd) {
	case IOCTL_WRITEGPIO:
		return ioctl_writegpio((int)arg);
	case IOCTL_READGPIO:
		return ioctl_readgpio((int)arg);
	default:
		printk(KERN_ERR "device_ioctl() - Invalid IOCTL (0x%x)\n", cmd);
		return EINVAL;
	}
	return 0;
}

struct file_operations Fops = {
	.ioctl = device_ioctl,
	.open = device_open,
	.release = device_release,
};

/* Initialize the module - Register the character device */
int init_chrdev(struct device *dev)
{
	int ret, gps_major;

	ret = alloc_chrdev_region(&agps_gpio_dev, 1, 1,	"agps_gpio");
	gps_major = MAJOR(agps_gpio_dev);
	if (ret < 0) {
		dev_err(dev, "can't get major %d\n", gps_major);
		goto err3;
	}

	cdev_init(&mxc_gps_cdev, &Fops);
	mxc_gps_cdev.owner = THIS_MODULE;

	ret = cdev_add(&mxc_gps_cdev, agps_gpio_dev, 1);
	if (ret) {
		dev_err(dev, "can't add cdev\n");
		goto err2;
	}

	/* create class and device for udev information */
	gps_class = class_create(THIS_MODULE, "gps");
	if (IS_ERR(gps_class)) {
		dev_err(dev, "failed to create gps class\n");
		ret = -ENOMEM;
		goto err1;
	}

	gps_class_dev = device_create(gps_class, NULL, MKDEV(gps_major, 1), NULL,
					AGPSGPIO_DEVICE_FILE_NAME);
	if (IS_ERR(gps_class_dev)) {
		dev_err(dev, "failed to create gps gpio class device\n");
		ret = -ENOMEM;
		goto err0;
	}

	return 0;
err0:
	class_destroy(gps_class);
err1:
	cdev_del(&mxc_gps_cdev);
err2:
	unregister_chrdev_region(agps_gpio_dev, 1);
err3:
	return ret;
}

/* Cleanup - unregister the appropriate file from /proc. */
void cleanup_chrdev(void)
{
	/* destroy gps device class */
	device_destroy(gps_class, MKDEV(MAJOR(agps_gpio_dev), 1));
	class_destroy(gps_class);

	/* Unregister the device */
	cdev_del(&mxc_gps_cdev);
	unregister_chrdev_region(agps_gpio_dev, 1);
}

/*!
 * This function initializes the driver in terms of memory of the soundcard
 * and some basic HW clock settings.
 *
 * @return              0 on success, -1 otherwise.
 */
static int __init gps_ioctrl_probe(struct platform_device *pdev)
{
	struct regulator *gps_regu;

	mxc_gps_ioctrl_data =
	    (struct mxc_gps_platform_data *)pdev->dev.platform_data;

	/* open GPS GPO3 1v8 for GL gps support */
	if (mxc_gps_ioctrl_data->core_reg != NULL) {
		mxc_gps_ioctrl_data->gps_regu_core =
		    regulator_get(&(pdev->dev), mxc_gps_ioctrl_data->core_reg);
		gps_regu = mxc_gps_ioctrl_data->gps_regu_core;
		if (!IS_ERR_VALUE((u32)gps_regu)) {
			regulator_set_voltage(gps_regu, 1800000, 1800000);
			regulator_enable(gps_regu);
		} else {
			return -1;
		}
	}
	/* open GPS GPO1 2v8 for GL gps support */
	if (mxc_gps_ioctrl_data->analog_reg != NULL) {
		mxc_gps_ioctrl_data->gps_regu_analog =
		    regulator_get(&(pdev->dev),
				  mxc_gps_ioctrl_data->analog_reg);
		gps_regu = mxc_gps_ioctrl_data->gps_regu_analog;
		if (!IS_ERR_VALUE((u32)gps_regu)) {
			regulator_set_voltage(gps_regu, 2800000, 2800000);
			regulator_enable(gps_regu);
		} else {
			return -1;
		}
	}
	gpio_gps_active();

	/* Register character device */
	init_chrdev(&(pdev->dev));
	return 0;
}

static int gps_ioctrl_remove(struct platform_device *pdev)
{
	struct regulator *gps_regu;

	mxc_gps_ioctrl_data =
	    (struct mxc_gps_platform_data *)pdev->dev.platform_data;

	/* Character device cleanup.. */
	cleanup_chrdev();
	gpio_gps_inactive();

	/* close GPS GPO3 1v8 for GL gps */
	gps_regu = mxc_gps_ioctrl_data->gps_regu_core;
	if (mxc_gps_ioctrl_data->core_reg != NULL) {
		regulator_disable(gps_regu);
		regulator_put(gps_regu);
	}
	/* close GPS GPO1 2v8 for GL gps */
	gps_regu = mxc_gps_ioctrl_data->gps_regu_analog;
	if (mxc_gps_ioctrl_data->analog_reg != NULL) {
		regulator_disable(gps_regu);
		regulator_put(gps_regu);
	}

	return 0;
}

static int gps_ioctrl_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* PowerEn toggle off */
	ioctl_writegpio(0x1);
	return 0;
}

static int gps_ioctrl_resume(struct platform_device *pdev)
{
	/* PowerEn pull up */
	ioctl_writegpio(0x3);
	return 0;
}

static struct platform_driver gps_ioctrl_driver = {
	.probe = gps_ioctrl_probe,
	.remove = gps_ioctrl_remove,
	.suspend = gps_ioctrl_suspend,
	.resume = gps_ioctrl_resume,
	.driver = {
		   .name = "gps_ioctrl",
		   },
};

/*!
 * Entry point for GPS ioctrl module.
 *
 */
static int __init gps_ioctrl_init(void)
{
	return platform_driver_register(&gps_ioctrl_driver);
}

/*!
 * unloading module.
 *
 */
static void __exit gps_ioctrl_exit(void)
{
	platform_driver_unregister(&gps_ioctrl_driver);
}

module_init(gps_ioctrl_init);
module_exit(gps_ioctrl_exit);
MODULE_DESCRIPTION("GPIO DEVICE DRIVER");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_LICENSE("GPL");
