/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13783/pmic_rtc.c
 * @brief This is the main file of PMIC(mc13783) RTC driver.
 *
 * @ingroup PMIC_RTC
 */

/*
 * Includes
 */
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/pmic_rtc.h>
#include <linux/pmic_status.h>

#include "pmic_rtc_defs.h"

#define PMIC_LOAD_ERROR_MSG		\
"PMIC card was not correctly detected. Stop loading PMIC RTC driver\n"

/*
 * Global variables
 */
static int pmic_rtc_major;
static void callback_alarm_asynchronous(void *);
static void callback_alarm_synchronous(void *);
static unsigned int pmic_rtc_poll(struct file *file, poll_table * wait);
static DECLARE_WAIT_QUEUE_HEAD(queue_alarm);
static DECLARE_WAIT_QUEUE_HEAD(pmic_rtc_wait);
static pmic_event_callback_t alarm_callback;
static pmic_event_callback_t rtc_callback;
static bool pmic_rtc_done;
static struct class *pmic_rtc_class;

static DECLARE_MUTEX(mutex);

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_rtc_set_time);
EXPORT_SYMBOL(pmic_rtc_get_time);
EXPORT_SYMBOL(pmic_rtc_set_time_alarm);
EXPORT_SYMBOL(pmic_rtc_get_time_alarm);
EXPORT_SYMBOL(pmic_rtc_wait_alarm);
EXPORT_SYMBOL(pmic_rtc_event_sub);
EXPORT_SYMBOL(pmic_rtc_event_unsub);

/*
 * Real Time Clock Pmic API
 */

/*!
 * This is the callback function called on TSI Pmic event, used in asynchronous
 * call.
 */
static void callback_alarm_asynchronous(void *unused)
{
	pmic_rtc_done = true;
}

/*!
 * This is the callback function is used in test code for (un)sub.
 */
static void callback_test_sub(void)
{
	printk(KERN_INFO "*****************************************\n");
	printk(KERN_INFO "***** PMIC RTC 'Alarm IT CallBack' ******\n");
	printk(KERN_INFO "*****************************************\n");
}

/*!
 * This is the callback function called on TSI Pmic event, used in synchronous
 * call.
 */
static void callback_alarm_synchronous(void *unused)
{
	printk(KERN_INFO "*** Alarm IT Pmic ***\n");
	wake_up(&queue_alarm);
}

/*!
 * This function wait the Alarm event
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_wait_alarm(void)
{
	DEFINE_WAIT(wait);
	alarm_callback.func = callback_alarm_synchronous;
	alarm_callback.param = NULL;
	CHECK_ERROR(pmic_event_subscribe(EVENT_TODAI, alarm_callback));
	prepare_to_wait(&queue_alarm, &wait, TASK_UNINTERRUPTIBLE);
	schedule();
	finish_wait(&queue_alarm, &wait);
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_TODAI, alarm_callback));
	return PMIC_SUCCESS;
}

/*!
 * This function set the real time clock of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time(struct timeval *pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	tod_reg_val = pmic_time->tv_sec % 86400;
	day_reg_val = pmic_time->tv_sec / 86400;

	mask = BITFMASK(MC13783_RTCTIME_TIME);
	value = BITFVAL(MC13783_RTCTIME_TIME, tod_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_TIME, value, mask));

	mask = BITFMASK(MC13783_RTCDAY_DAY);
	value = BITFVAL(MC13783_RTCDAY_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function get the real time clock of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_get_time(struct timeval *pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	mask = BITFMASK(MC13783_RTCTIME_TIME);
	CHECK_ERROR(pmic_read_reg(REG_RTC_TIME, &value, mask));
	tod_reg_val = BITFEXT(value, MC13783_RTCTIME_TIME);

	mask = BITFMASK(MC13783_RTCDAY_DAY);
	CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
	day_reg_val = BITFEXT(value, MC13783_RTCDAY_DAY);

	pmic_time->tv_sec = (unsigned long)((unsigned long)(tod_reg_val &
							    0x0001FFFF) +
					    (unsigned long)(day_reg_val *
							    86400));
	return PMIC_SUCCESS;
}

/*!
 * This function set the real time clock alarm of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time_alarm(struct timeval *pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	if (down_interruptible(&mutex) < 0)
		return ret;

	tod_reg_val = pmic_time->tv_sec % 86400;
	day_reg_val = pmic_time->tv_sec / 86400;

	mask = BITFMASK(MC13783_RTCALARM_TIME);
	value = BITFVAL(MC13783_RTCALARM_TIME, tod_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, value, mask));

	mask = BITFMASK(MC13783_RTCALARM_DAY);
	value = BITFVAL(MC13783_RTCALARM_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY_ALARM, value, mask));
	up(&mutex);
	return PMIC_SUCCESS;
}

/*!
 * This function get the real time clock alarm of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_get_time_alarm(struct timeval *pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	mask = BITFMASK(MC13783_RTCALARM_TIME);
	CHECK_ERROR(pmic_read_reg(REG_RTC_ALARM, &value, mask));
	tod_reg_val = BITFEXT(value, MC13783_RTCALARM_TIME);

	mask = BITFMASK(MC13783_RTCALARM_DAY);
	CHECK_ERROR(pmic_read_reg(REG_RTC_DAY_ALARM, &value, mask));
	day_reg_val = BITFEXT(value, MC13783_RTCALARM_DAY);

	pmic_time->tv_sec = (unsigned long)((unsigned long)(tod_reg_val &
							    0x0001FFFF) +
					    (unsigned long)(day_reg_val *
							    86400));

	return PMIC_SUCCESS;
}

/*!
 * This function is used to un/subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 * @param        sub      	define if Un/subscribe event.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event(t_rtc_int event, void *callback, bool sub)
{
	type_event rtc_event;
	if (callback == NULL) {
		return PMIC_ERROR;
	} else {
		rtc_callback.func = callback;
		rtc_callback.param = NULL;
	}
	switch (event) {
	case RTC_IT_ALARM:
		rtc_event = EVENT_TODAI;
		break;
	case RTC_IT_1HZ:
		rtc_event = EVENT_E1HZI;
		break;
	case RTC_IT_RST:
		rtc_event = EVENT_RTCRSTI;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	if (sub == true) {
		CHECK_ERROR(pmic_event_subscribe(rtc_event, rtc_callback));
	} else {
		CHECK_ERROR(pmic_event_unsubscribe(rtc_event, rtc_callback));
	}
	return PMIC_SUCCESS;
}

/*!
 * This function is used to subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event_sub(t_rtc_int event, void *callback)
{
	CHECK_ERROR(pmic_rtc_event(event, callback, true));
	return PMIC_SUCCESS;
}

/*!
 * This function is used to un subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event_unsub(t_rtc_int event, void *callback)
{
	CHECK_ERROR(pmic_rtc_event(event, callback, false));
	return PMIC_SUCCESS;
}

/* Called without the kernel lock - fine */
static unsigned int pmic_rtc_poll(struct file *file, poll_table * wait)
{
	/*poll_wait(file, &pmic_rtc_wait, wait); */

	if (pmic_rtc_done)
		return POLLIN | POLLRDNORM;
	return 0;
}

/*!
 * This function implements IOCTL controls on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int pmic_rtc_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct timeval *pmic_time = NULL;

	if (_IOC_TYPE(cmd) != 'p')
		return -ENOTTY;

	if (arg) {
		pmic_time = kmalloc(sizeof(struct timeval), GFP_KERNEL);
		if (pmic_time == NULL)
			return -ENOMEM;

		/*      if (copy_from_user(pmic_time, (struct timeval *)arg,
		   sizeof(struct timeval))) {
		   return -EFAULT;
		   } */
	}

	switch (cmd) {
	case PMIC_RTC_SET_TIME:
		if (copy_from_user(pmic_time, (struct timeval *)arg,
				   sizeof(struct timeval))) {
			return -EFAULT;
		}
		pr_debug("SET RTC\n");
		CHECK_ERROR(pmic_rtc_set_time(pmic_time));
		break;
	case PMIC_RTC_GET_TIME:
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval))) {
			return -EFAULT;
		}
		pr_debug("GET RTC\n");
		CHECK_ERROR(pmic_rtc_get_time(pmic_time));
		break;
	case PMIC_RTC_SET_ALARM:
		if (copy_from_user(pmic_time, (struct timeval *)arg,
				   sizeof(struct timeval))) {
			return -EFAULT;
		}
		pr_debug("SET RTC ALARM\n");
		CHECK_ERROR(pmic_rtc_set_time_alarm(pmic_time));
		break;
	case PMIC_RTC_GET_ALARM:
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval))) {
			return -EFAULT;
		}
		pr_debug("GET RTC ALARM\n");
		CHECK_ERROR(pmic_rtc_get_time_alarm(pmic_time));
		break;
	case PMIC_RTC_WAIT_ALARM:
		printk(KERN_INFO "WAIT ALARM...\n");
		CHECK_ERROR(pmic_rtc_event_sub(RTC_IT_ALARM,
					       callback_test_sub));
		CHECK_ERROR(pmic_rtc_wait_alarm());
		printk(KERN_INFO "ALARM DONE\n");
		CHECK_ERROR(pmic_rtc_event_unsub(RTC_IT_ALARM,
						 callback_test_sub));
		break;
	case PMIC_RTC_ALARM_REGISTER:
		printk(KERN_INFO "PMIC RTC ALARM REGISTER\n");
		alarm_callback.func = callback_alarm_asynchronous;
		alarm_callback.param = NULL;
		CHECK_ERROR(pmic_event_subscribe(EVENT_TODAI, alarm_callback));
		break;
	case PMIC_RTC_ALARM_UNREGISTER:
		printk(KERN_INFO "PMIC RTC ALARM UNREGISTER\n");
		alarm_callback.func = callback_alarm_asynchronous;
		alarm_callback.param = NULL;
		CHECK_ERROR(pmic_event_unsubscribe
			    (EVENT_TODAI, alarm_callback));
		pmic_rtc_done = false;
		break;
	default:
		pr_debug("%d unsupported ioctl command\n", (int)cmd);
		return -EINVAL;
	}

	if (arg) {
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval))) {
			return -EFAULT;
		}
		kfree(pmic_time);
	}

	return 0;
}

/*!
 * This function implements the open method on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_rtc_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * This function implements the release method on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_rtc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * This function is called to put the RTC in a low power state.
 * There is no need for power handlers for the RTC device.
 * The RTC cannot be suspended.
 *
 * @param   pdev  the device structure used to give information on which RTC
 *                device (0 through 3 channels) to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int pmic_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

/*!
 * This function is called to resume the RTC from a low power state.
 *
 * @param   pdev  the device structure used to give information on which RTC
 *                device (0 through 3 channels) to suspend
 *
 * @return  The function always returns 0.
 */
static int pmic_rtc_resume(struct platform_device *pdev)
{
	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */

static struct file_operations pmic_rtc_fops = {
	.owner = THIS_MODULE,
	.ioctl = pmic_rtc_ioctl,
	.poll = pmic_rtc_poll,
	.open = pmic_rtc_open,
	.release = pmic_rtc_release,
};

static int pmic_rtc_remove(struct platform_device *pdev)
{
	device_destroy(pmic_rtc_class, MKDEV(pmic_rtc_major, 0));
	class_destroy(pmic_rtc_class);
	unregister_chrdev(pmic_rtc_major, "pmic_rtc");
	return 0;
}

static int pmic_rtc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	pmic_rtc_major = register_chrdev(0, "pmic_rtc", &pmic_rtc_fops);
	if (pmic_rtc_major < 0) {
		printk(KERN_ERR "Unable to get a major for pmic_rtc\n");
		return pmic_rtc_major;
	}

	pmic_rtc_class = class_create(THIS_MODULE, "pmic_rtc");
	if (IS_ERR(pmic_rtc_class)) {
		printk(KERN_ERR "Error creating pmic rtc class.\n");
		ret = PTR_ERR(pmic_rtc_class);
		goto err_out1;
	}

	temp_class = device_create(pmic_rtc_class, NULL,
				   MKDEV(pmic_rtc_major, 0), NULL,
				   "pmic_rtc");
	if (IS_ERR(temp_class)) {
		printk(KERN_ERR "Error creating pmic rtc class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	printk(KERN_INFO "PMIC RTC successfully probed\n");
	return ret;

      err_out2:
	class_destroy(pmic_rtc_class);
      err_out1:
	unregister_chrdev(pmic_rtc_major, "pmic_rtc");
	return ret;
}

static struct platform_driver pmic_rtc_driver_ldm = {
	.driver = {
		   .name = "pmic_rtc",
		   .owner = THIS_MODULE,
		   },
	.suspend = pmic_rtc_suspend,
	.resume = pmic_rtc_resume,
	.probe = pmic_rtc_probe,
	.remove = pmic_rtc_remove,
};

static int __init pmic_rtc_init(void)
{
	pr_debug("PMIC RTC driver loading...\n");
	return platform_driver_register(&pmic_rtc_driver_ldm);
}
static void __exit pmic_rtc_exit(void)
{
	platform_driver_unregister(&pmic_rtc_driver_ldm);
	pr_debug("PMIC RTC driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall(pmic_rtc_init);
module_exit(pmic_rtc_exit);

MODULE_DESCRIPTION("Pmic_rtc driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
