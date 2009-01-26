/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/uaccess.h>	/* for get_user, put_user, access_ok */
#include <linux/sched.h>	/* jiffies */
#include <linux/poll.h>
#include <linux/regulator/regulator.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "agpsgpiodev.h"

extern void gpio_gps_active(void);
extern void gpio_gps_inactive(void);
extern int gpio_gps_access(int para);

struct mxc_gps_platform_data *mxc_gps_ioctrl_data;
static int Device_Open;		/* Only allow a single user of this device */

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
int init_chrdev(void)
{
	/* NOTE : THIS IS THE OLD-SCHOOL WAY TO REGISTER A CHAR DEVICE.
	   THE RECOMMENDED APPROACH IS TO USE cdev_alloc, cdev_init, cdev_add,
	   cdev_del. REFER TO CHAPTER 3 IN THE DEVICE DRIVER BOOK! */

	/* Register the character device (at least try) */
	int ret_val =
	    register_chrdev(MAJOR_NUM, AGPSGPIO_DEVICE_FILE_NAME, &Fops);

	/* Negative values signify an error */
	if (ret_val < 0) {
		printk(KERN_ERR
		       "init_chrdev() - Failed to register"
		       "char device (error %d)\n", ret_val);
		return ret_val;
	}

	return 0;
}

/* Cleanup - unregister the appropriate file from /proc. */
void cleanup_chrdev(void)
{
	/* Unregister the device
	   int ret = unregister_chrdev(MAJOR_NUM, AGPSGPIO_DEVICE_FILE_NAME);
	   change for 2.6.24 since its declarationis as below:
	   extern void unregister_chrdev(unsigned int, const char *); */
	unregister_chrdev(MAJOR_NUM, AGPSGPIO_DEVICE_FILE_NAME);
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
		gps_regu =
		    regulator_get(&(pdev->dev), mxc_gps_ioctrl_data->core_reg);
		if (!IS_ERR_VALUE((u32)gps_regu)) {
			regulator_set_voltage(gps_regu, 1800000);
			regulator_enable(gps_regu);
			regulator_put(gps_regu, &(pdev->dev));
		} else {
			return -1;
		}
	}
	/* open GPS GPO1 2v8 for GL gps support */
	if (mxc_gps_ioctrl_data->analog_reg != NULL) {
		gps_regu =
		    regulator_get(&(pdev->dev),
				  mxc_gps_ioctrl_data->analog_reg);
		if (!IS_ERR_VALUE((u32)gps_regu)) {
			regulator_set_voltage(gps_regu, 2800000);
			regulator_enable(gps_regu);
			regulator_put(gps_regu, &(pdev->dev));
		} else {
			return -1;
		}
	}
	gpio_gps_active();

	/* Register character device */
	init_chrdev();
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
	if (mxc_gps_ioctrl_data->core_reg != NULL) {
		gps_regu =
		    regulator_get(&(pdev->dev), mxc_gps_ioctrl_data->core_reg);
		regulator_disable(gps_regu);
		regulator_put(gps_regu, &(pdev->dev));
	}
	/* close GPS GPO1 2v8 for GL gps */
	if (mxc_gps_ioctrl_data->analog_reg != NULL) {
		gps_regu =
		    regulator_get(&(pdev->dev),
				  mxc_gps_ioctrl_data->analog_reg);
		regulator_disable(gps_regu);
		regulator_put(gps_regu, &(pdev->dev));
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
