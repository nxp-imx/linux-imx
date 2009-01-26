/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/major.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include "i2c_slave_device.h"
static i2c_slave_device_t *i2c_slave_devices[I2C_SLAVE_DEVICE_MAX];
struct class *i2c_slave_class;
static int i2c_slave_device_get_id(void)
{
	int i;
	for (i = 0; i < I2C_SLAVE_DEVICE_MAX; i++) {
		if (!i2c_slave_devices[i])
			return i;
	}
	return -1;
}

i2c_slave_device_t *i2c_slave_device_find(int id)
{
	if (id >= 0 && id < I2C_SLAVE_DEVICE_MAX)
		return i2c_slave_devices[id];

	else
		return NULL;
}
void i2c_slave_device_set_name(i2c_slave_device_t *device, char *name)
{
	device->name = name;
}

void i2c_slave_device_set_address(i2c_slave_device_t *device, u8 address)
{
	device->address = address;
}

u8 i2c_slave_device_get_addr(i2c_slave_device_t *device)
{
	return device->address;
}

int i2c_slave_device_set_freq(i2c_slave_device_t *device, u32 freq)
{
	/*TODO: freq check */
	device->scl_freq = freq;
	return 0;
}

u32 i2c_slave_device_get_freq(i2c_slave_device_t *device)
{
	return device->scl_freq;
}

/*used by the specific i2c device to register itself to the core.*/
i2c_slave_device_t *i2c_slave_device_alloc(void)
{
	int id;
	i2c_slave_device_t *device;
	id = i2c_slave_device_get_id();
	if (id < 0) {
		goto error;
	}
	device =
	    (i2c_slave_device_t *) kzalloc(sizeof(i2c_slave_device_t),
					   GFP_KERNEL);
	if (!device) {
		printk(KERN_ERR "%s: alloc device error\n", __func__);
		goto error_device;
	}
	device->receive_buffer = i2c_slave_rb_alloc(PAGE_SIZE);
	if (!device->receive_buffer) {
		printk(KERN_ERR "%s: alloc receive buffer error\n", __func__);
		goto error_receive_buffer;
	}
	device->send_buffer = i2c_slave_rb_alloc(PAGE_SIZE);
	if (!device->send_buffer) {
		printk(KERN_ERR "%s: alloc send buffer error\n", __func__);
		goto error_send_buffer;
	}
	device->id = id;
	return device;

      error_send_buffer:
	kfree(device->receive_buffer);
      error_receive_buffer:
	kfree((void *)device);
      error_device:
	pr_debug(KERN_ERR "%s: no memory\n", __func__);
      error:
	return 0;
}

void i2c_slave_device_free(i2c_slave_device_t *dev)
{
	i2c_slave_rb_release(dev->receive_buffer);
	i2c_slave_rb_release(dev->send_buffer);
	kfree(dev);
}

int i2c_slave_device_register(i2c_slave_device_t *device)
{
	device->dev = device_create(i2c_slave_class, NULL,
				    MKDEV(i2c_slave_major, device->id),
				    "slave-i2c-%d", device->id);
	if (!device->dev) {
		return -1;
	}
	i2c_slave_devices[device->id] = device;
	return 0;
}

void i2c_slave_device_unregister(i2c_slave_device_t *device)
{
	device_destroy(i2c_slave_class, MKDEV(i2c_slave_major, device->id));
	i2c_slave_devices[device->id] = 0;
	i2c_slave_device_free(device);
}

/*
	this two functions are used by i2c slave core to start or stop the specific i2c device.
*/
int i2c_slave_device_start(i2c_slave_device_t *device)
{
	return device->start(device);
}

int i2c_slave_device_stop(i2c_slave_device_t *device)
{
	return device->stop(device);
}

/*
	this two functions are used by i2c slave core to get data by the specific i2c slave device
	or send data to it to feed i2c master's need.

	@mod: async(1) or sync(0) mode.
*/
int i2c_slave_device_read(i2c_slave_device_t *device, int num, u8 *data)
{
	int read_num, read_total = 0;
	int step = 1000;
	u8 *read_buf = data;
	printk(KERN_INFO "%s: device id=%d, num=%d\n", __func__, device->id,
	       num);
	read_num = i2c_slave_rb_consume(device->receive_buffer, num, read_buf);
	read_total += read_num;
	read_buf += read_num;
	step--;
	while ((read_total < num) && step) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 10);
		if (!signal_pending(current)) {
		} else {
			 /*TODO*/ break;
		}
		read_num =
		    i2c_slave_rb_consume(device->receive_buffer,
					 num - read_total, read_buf);
		num -= read_num;
		read_buf += read_num;
		step--;
	}
	return read_total;
}
int i2c_slave_device_write(i2c_slave_device_t *device, int num, u8 *data)
{
	int write_num, write_total = 0;
	int step = 1000;
	u8 *buf_index = data;
	write_num = i2c_slave_rb_produce(device->send_buffer, num, buf_index);
	write_total += write_num;
	buf_index += write_num;
	step--;
	while (write_total < num && step) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 10);
		if (!signal_pending(current)) {
		} else {
			 /*TODO*/ step = 0;
			break;
		}
		write_num =
		    i2c_slave_rb_produce(device->send_buffer, num - write_total,
					 buf_index);
		write_total += write_num;
		buf_index += write_num;
		step--;
	}
	while (step && i2c_slave_rb_num(device->send_buffer)) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 10);
		if (!signal_pending(current)) {
			step--;
		} else {
			 /*TODO*/ step = 0;
			break;
		}
	}
	if (!step) {
		write_total -= i2c_slave_rb_num(device->send_buffer);
		i2c_slave_rb_clear(device->send_buffer);
	}
	return write_total;
}

/*
 * this 2 functions used by the specific i2c slave device when they got data from master(produce),
 * or is request by master(consume).
 */
int i2c_slave_device_produce(i2c_slave_device_t *device, int num, u8 *data)
{
	int ret;
	ret = i2c_slave_rb_produce(device->receive_buffer, num, data);
	return ret;
}
int i2c_slave_device_consume(i2c_slave_device_t *device, int num, u8 *data)
{
	return i2c_slave_rb_consume(device->send_buffer, num, data);
}

EXPORT_SYMBOL(i2c_slave_device_set_name);
EXPORT_SYMBOL(i2c_slave_device_set_address);
EXPORT_SYMBOL(i2c_slave_device_get_addr);
EXPORT_SYMBOL(i2c_slave_device_find);
EXPORT_SYMBOL(i2c_slave_device_set_freq);
EXPORT_SYMBOL(i2c_slave_device_get_freq);

/*
* used by the specific i2c device to register itself to the core.
*/
EXPORT_SYMBOL(i2c_slave_device_alloc);
EXPORT_SYMBOL(i2c_slave_device_free);
EXPORT_SYMBOL(i2c_slave_device_register);
EXPORT_SYMBOL(i2c_slave_device_unregister);

/*
	this two functions are used by i2c slave core to start or stop the specific i2c device.
*/
EXPORT_SYMBOL(i2c_slave_device_start);
EXPORT_SYMBOL(i2c_slave_device_stop);

/*
	this two functions are used by i2c slave core to get data by the specific i2c slave device
	or send data to it for it to feed i2c master's need.

	@mod: async(1) or sync(0) mode.
*/
EXPORT_SYMBOL(i2c_slave_device_read);
EXPORT_SYMBOL(i2c_slave_device_write);

/*
* this 2 functions used by the specific i2c slave device when they got data from master,
* or is request by master.
*/
EXPORT_SYMBOL(i2c_slave_device_produce);
EXPORT_SYMBOL(i2c_slave_device_consume);
