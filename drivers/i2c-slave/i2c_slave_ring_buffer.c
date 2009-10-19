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
#include <linux/slab.h>
#include <linux/hardirq.h>
#include "i2c_slave_ring_buffer.h"

i2c_slave_ring_buffer_t *i2c_slave_rb_alloc(int size)
{
	i2c_slave_ring_buffer_t *ring_buf;

	ring_buf =
	    (i2c_slave_ring_buffer_t *) kzalloc(sizeof(i2c_slave_ring_buffer_t),
						GFP_KERNEL);
	if (!ring_buf) {
		pr_debug("%s: alloc ring_buf error\n", __func__);
		goto error;
	}

	ring_buf->buffer = kmalloc(size, GFP_KERNEL);
	if (!ring_buf->buffer) {
		pr_debug("%s: alloc buffer error\n", __func__);
		goto error1;
	}

	ring_buf->total = size;

	ring_buf->lock = __SPIN_LOCK_UNLOCKED(ring_buf->lock);
	return ring_buf;

      error1:
	kfree(ring_buf);
      error:
	return NULL;
}

void i2c_slave_rb_release(i2c_slave_ring_buffer_t *ring)
{
	unsigned long flags;

	spin_lock_irqsave(ring->lock, flags);
	kfree(ring->buffer);
	spin_unlock_irqrestore(ring->lock, flags);
	kfree(ring);
}

int i2c_slave_rb_produce(i2c_slave_ring_buffer_t *ring, int num, char *data)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(ring->lock, flags);

	if (ring->start < ring->end) {
		if ((ring->start + num) < ring->end) {	/*have enough space */
			memcpy(&ring->buffer[ring->start], data, num);
			ring->start += num;
			ret = num;
		} else {	/*space not enough, just copy part of it. */
			ret = ring->end - ring->start;
			memcpy(&ring->buffer[ring->start], data, ret);
			ring->start += ret;
			ring->full = 1;
		}
	} else if (ring->start >= ring->end && !ring->full) {
		if (ring->start + num <= ring->total) {	/*space enough */
			ret = num;
			memcpy(&ring->buffer[ring->start], data, ret);
			ring->start += ret;
		} else {	/*turn ring->start around */
			ret = ring->total - ring->start;
			memcpy(&ring->buffer[ring->start], data, ret);
			ring->start = 0;
			num -= ret;
			data += ret;
			if (num < ring->end) {	/*space enough */
				ret += num;
				memcpy(&ring->buffer[ring->start], data, num);
			} else {	/*space not enough, just copy part of it. */
				ret += ring->end;
				memcpy(&ring->buffer[ring->start], data,
				       ring->end);
				ring->start = ring->end;
				ring->full = 1;
			}
		}
	} else {		/*(ring->data == ring->end) && ring->full ) : full */
		ret = 0;
	}

	spin_unlock_irqrestore(ring->lock, flags);
	return ret;
}

int i2c_slave_rb_consume(i2c_slave_ring_buffer_t *ring, int num, char *data)
{
	int ret;
	unsigned long flags;
	spin_lock_irqsave(ring->lock, flags);
	if (num <= 0) {
		ret = 0;
		goto out;
	}

	if (ring->start > ring->end) {
		if (num <= ring->start - ring->end) {	/*enough */
			ret = num;
			memcpy(data, &ring->buffer[ring->end], ret);
			ring->end += ret;
		} else {	/*not enough */
			ret = ring->start - ring->end;
			memcpy(data, &ring->buffer[ring->end], ret);
			ring->end += ret;
		}
	} else if (ring->start < ring->end || ring->full) {
		if (num <= ring->total - ring->end) {
			ret = ring->total - ring->end;
			memcpy(data, &ring->buffer[ring->end], ret);
			ring->end += ret;
		} else if (num <= ring->total - ring->end + ring->start) {
			ret = ring->total - ring->end;
			memcpy(data, &ring->buffer[ring->end], ret);
			ring->end = 0;
			data += ret;
			num -= ret;
			memcpy(data, &ring->buffer[ring->end], num);
			ring->end = num;
			ret += num;
		} else {
			ret = ring->total - ring->end;
			memcpy(data, &ring->buffer[ring->end], ret);
			ring->end = 0;
			data += ret;
			num -= ret;
			memcpy(data, &ring->buffer[ring->end], ring->start);
			ring->end = ring->start;
			ret += ring->start;
		}
		ring->full = 0;
	} else {		/*empty */
		ret = 0;
	}

      out:
	spin_unlock_irqrestore(ring->lock, flags);

	return ret;
}

int i2c_slave_rb_num(i2c_slave_ring_buffer_t *ring)
{
	int ret;
	unsigned long flags;
	spin_lock_irqsave(ring->lock, flags);
	if (ring->start > ring->end) {
		ret = ring->start - ring->end;
	} else if (ring->start < ring->end) {
		ret = ring->total - ring->end + ring->start;
	} else if (ring->full) {
		ret = ring->total;
	} else {
		ret = 0;
	}
	spin_unlock_irqrestore(ring->lock, flags);
	return ret;
}

void i2c_slave_rb_clear(i2c_slave_ring_buffer_t *ring)
{
	unsigned long flags;
	spin_lock_irqsave(ring->lock, flags);

	ring->start = ring->end = 0;
	ring->full = 0;
	spin_unlock_irqrestore(ring->lock, flags);
}
