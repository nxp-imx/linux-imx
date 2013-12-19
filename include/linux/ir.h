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
#ifndef _LINUX_IR_H
#define _LINUX_IR_H

#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/kgdb.h>
#endif /* __KERNEL__ */

#define IR_MAGIC 'I'

#define IR_GET_CARRIER_FREQS		_IO(IR_MAGIC, 0x01)
#define IR_GET_CARRIER_FREQS_NUM	_IO(IR_MAGIC, 0x02)
#define IR_DATA_TRANSMIT		_IO(IR_MAGIC, 0x03)
#define IR_CFG_CARRIER			_IO(IR_MAGIC, 0x04)

struct ir_data_pattern {
	int len;
	int *pattern;
};

struct ir_carrier_freq {
	/* carrier frequence id */
	int id;
	/* ir support min carrier freq */
	int min;
	/* ir support max carrier freq */
	int max;
};
#ifdef __KERNEL__
extern void ir_device_register(const char *name, struct device *parent, void *devdata);
extern void ir_device_unregister(void);
extern int ir_config(void *dev, int carry_freq);
extern int ir_get_num_carrier_freqs(void);
extern int ir_get_carrier_range(int id, int *min, int *max);
extern int ir_transmit(void *dev, int len, int *pattern, unsigned char start_level);
#endif
#endif /* _LINUX_IR_H */
