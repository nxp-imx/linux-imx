/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc9s08dz60keyb.c
 *
 * @brief Driver for the Freescale Semiconductor MXC keypad port.
 *
 * The keypad driver is designed as a standard Input driver which interacts
 * with low level keypad port hardware. Upon opening, the Keypad driver
 * initializes the keypad port. When the keypad interrupt happens the driver
 * calles keypad polling timer and scans the keypad matrix for key
 * press/release. If all key press/release happened it comes out of timer and
 * waits for key press interrupt. The scancode for key press and release events
 * are passed to Input subsytem.
 *
 * @ingroup keypad
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/kbd_kern.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <asm/mach/keypad.h>

#define MOD_NAME "mc9s08dz60-keyb"
/*
 * Module header file
 */

/*! Input device structure. */
static struct input_dev *mc9s08dz60kbd_dev;
static unsigned int key_status;
static int keypad_irq;
static unsigned int key_code_map[8] = {
	KEY_LEFT,
	KEY_DOWN,
	0,
	0,
	KEY_UP,
	KEY_RIGHT,
	0,
	0,
};
static unsigned int keycodes_size = 8;

static void read_key_handler(struct work_struct *work);
static DECLARE_WORK(key_pad_event, read_key_handler);


static void read_key_handler(struct work_struct *work)
{
	unsigned int val1, val2;
	int pre_val, curr_val, i;
	val1 = val2 = 0xff;
	mcu_pmic_read_reg(REG_MCU_KPD_1, &val1, 0xff);
	mcu_pmic_read_reg(REG_MCU_KPD_2, &val2, 0xff);
	pr_debug("key pressed, 0x%02x%02x\n", val2, val1);
	for (i = 0; i < 8; i++) {
		curr_val = (val1 >> i) & 0x1;
		if (curr_val > 0)
			input_event(mc9s08dz60kbd_dev, EV_KEY,
			key_code_map[i], 1);
		else {
			pre_val = (key_status >> i) & 0x1;
			if (pre_val > 0)
				input_event(mc9s08dz60kbd_dev, EV_KEY,
				key_code_map[i], 0);
		}
	}
	key_status = val1;

}

static irqreturn_t mc9s08dz60kpp_interrupt(int irq, void *dev_id)
{
	schedule_work(&key_pad_event);
	return IRQ_RETVAL(1);
}

/*!
 * This function is called when the keypad driver is opened.
 * Since keypad initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always return 0
 */
static int mc9s08dz60kpp_open(struct input_dev *dev)
{
	return 0;
}

/*!
 * This function is called close the keypad device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void mc9s08dz60kpp_close(struct input_dev *dev)
{
}


/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions.
 *
 * @return  The function returns 0 on successful registration. Otherwise returns
 *          specific error code.
 */
static int mc9s08dz60kpp_probe(struct platform_device *pdev)
{
	int i, irq;
	int retval;

	retval = mcu_pmic_write_reg(REG_MCU_KPD_CONTROL, 0x1, 0x1);
	if (retval != 0) {
		pr_info("mc9s08dz60 keypad: mcu not detected!\n");
		return retval;
	}

	irq = platform_get_irq(pdev, 0);
	retval = request_irq(irq, mc9s08dz60kpp_interrupt,
			0, MOD_NAME, MOD_NAME);
	if (retval) {
		pr_debug("KPP: request_irq(%d) returned error %d\n",
			 irq, retval);
		return retval;
	}
	keypad_irq = irq;

	mc9s08dz60kbd_dev = input_allocate_device();
	if (!mc9s08dz60kbd_dev) {
		pr_info(KERN_ERR "mc9s08dz60kbd_dev: \
		not enough memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	mc9s08dz60kbd_dev->name = "mc9s08dz60kpd";
	mc9s08dz60kbd_dev->id.bustype = BUS_HOST;
	mc9s08dz60kbd_dev->open = mc9s08dz60kpp_open;
	mc9s08dz60kbd_dev->close = mc9s08dz60kpp_close;

	retval = input_register_device(mc9s08dz60kbd_dev);
	if (retval < 0) {
		pr_info(KERN_ERR
		       "mc9s08dz60kbd_dev: failed to register input device\n");
		goto err2;
	}

	__set_bit(EV_KEY, mc9s08dz60kbd_dev->evbit);

	for (i = 0; i < keycodes_size; i++)
		__set_bit(key_code_map[i], mc9s08dz60kbd_dev->keybit);

	device_init_wakeup(&pdev->dev, 1);

	pr_info("mc9s08dz60 keypad probed\n");

	return 0;

err2:
	input_free_device(mc9s08dz60kbd_dev);
err1:
	free_irq(irq, MOD_NAME);
	return retval;
}

/*!
 * Dissociates the driver from the kpp device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mc9s08dz60kpp_remove(struct platform_device *pdev)
{
	free_irq(keypad_irq, MOD_NAME);
	input_unregister_device(mc9s08dz60kbd_dev);

	if (mc9s08dz60kbd_dev)
		input_free_device(mc9s08dz60kbd_dev);

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mc9s08dz60kpd_driver = {
	.driver = {
		   .name = "mc9s08dz60keypad",
		   .bus = &platform_bus_type,
		   },
	.probe = mc9s08dz60kpp_probe,
	.remove = mc9s08dz60kpp_remove
};

static int __init mc9s08dz60kpp_init(void)
{
	pr_info(KERN_INFO "mc9s08dz60 keypad loaded\n");
	platform_driver_register(&mc9s08dz60kpd_driver);
	return 0;
}

static void __exit mc9s08dz60kpp_cleanup(void)
{
	platform_driver_unregister(&mc9s08dz60kpd_driver);
}

module_init(mc9s08dz60kpp_init);
module_exit(mc9s08dz60kpp_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Keypad Controller Driver");
MODULE_LICENSE("GPL");
