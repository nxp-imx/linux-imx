/*
 * da9052 TSI filter module declarations.
  *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_TSI_FILTER_H
#define __LINUX_MFD_DA9052_TSI_FILTER_H

#include <linux/mfd/da9052/tsi_cfg.h>
#include <linux/semaphore.h>

struct da9052_tsi_data {
 	s16	x;
	s16	y;
	s16	z;
};

struct da9052_tsi_raw_fifo {
	struct semaphore 	lock;
	s32		 	head;
	s32 			tail;
	struct da9052_tsi_data	data[TSI_RAW_DATA_BUF_SIZE];
};

struct tsi_thread_type {
	u8 			pid;
	u8 			state;
	struct completion 	notifier;
	struct task_struct	*thread_task;
} ;

/* State for TSI thread */
#define	ACTIVE		0
#define	INACTIVE	1


extern u32 da9052_tsi_get_input_dev(u8 off);

ssize_t da9052_tsi_get_calib_display_point(struct da9052_tsi_data *displayPtr);

#endif	/* __LINUX_MFD_DA9052_TSI_FILTER_H */
