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

#ifndef __I2C_SLAVE_H__
#define __I2C_SLAVE_H__

#include <linux/list.h>
#include "i2c_slave_ring_buffer.h"

#define I2C_SLAVE_DEVICE_MAX 256
extern int i2c_slave_major;

typedef struct i2c_slave_device {
	struct list_head list;
	u8 address;
	u32 scl_freq;
	char *name;
	i2c_slave_ring_buffer_t *receive_buffer;
	i2c_slave_ring_buffer_t *send_buffer;
	int (*start) (struct i2c_slave_device *);
	int (*stop) (struct i2c_slave_device *);
	/*int (*set_freq)(struct i2c_slave_device*);
	int (*set_addr)(struct i2c_slave_device*);*/
	void *private_data;
	struct device *dev;
	int id;
} i2c_slave_device_t;

/*
	used by the specific device to set some infomations.
*/
void i2c_slave_device_set_name(i2c_slave_device_t *device, char *name);
void i2c_slave_device_set_address(i2c_slave_device_t *device, u8 address);
i2c_slave_device_t *i2c_slave_device_find(int id);
u8 i2c_slave_device_get_addr(i2c_slave_device_t *device);
int i2c_slave_device_set_freq(i2c_slave_device_t *device, u32 freq);
u32 i2c_slave_device_get_freq(i2c_slave_device_t *device);

/*
**used by the specific i2c device to register itself to the core.
*/
i2c_slave_device_t *i2c_slave_device_alloc(void);
void i2c_slave_device_free(i2c_slave_device_t *);
int i2c_slave_device_register(i2c_slave_device_t *device);
void i2c_slave_device_unregister(i2c_slave_device_t *device);

/*
	this two functions are used by i2c slave core to start or stop the specific i2c device.
*/
int i2c_slave_device_start(i2c_slave_device_t *device);
int i2c_slave_device_stop(i2c_slave_device_t *device);

/*
	this two functions are used by i2c slave core to get data by the specific i2c slave device
	or send data to it for it to feed i2c master's need.

	@mod: async(1) or sync(0) mode.
*/
int i2c_slave_device_read(i2c_slave_device_t *device, int num, u8 *data);
int i2c_slave_device_write(i2c_slave_device_t *device, int num, u8 *data);

/*
*this 2 functions used by the specific i2c slave device when they got data from master,
*or is requested by master.
*/
int i2c_slave_device_produce(i2c_slave_device_t *device, int num, u8 *data);
int i2c_slave_device_consume(i2c_slave_device_t *device, int num, u8 *data);

#endif
