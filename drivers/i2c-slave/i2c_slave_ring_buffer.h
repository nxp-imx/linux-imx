#ifndef __BUFFER_MANAGER_H__
#define __BUFFER_MANAGER_H__
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
#include <linux/types.h>
#include <linux/spinlock.h>

typedef struct i2c_slave_ring_buffer {
	int start;
	int end;
	int total;
	bool full;
	char *buffer;
	spinlock_t lock;
} i2c_slave_ring_buffer_t;

i2c_slave_ring_buffer_t *i2c_slave_rb_alloc(int size);

void i2c_slave_rb_release(i2c_slave_ring_buffer_t *ring);

int i2c_slave_rb_produce(i2c_slave_ring_buffer_t *ring, int num, char *data);

int i2c_slave_rb_consume(i2c_slave_ring_buffer_t *ring, int num, char *data);

int i2c_slave_rb_num(i2c_slave_ring_buffer_t *ring);

void i2c_slave_rb_clear(i2c_slave_ring_buffer_t *ring);

#endif
