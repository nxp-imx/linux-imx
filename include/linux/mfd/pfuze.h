/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __LINUX_MFD_PFUZE_H
#define __LINUX_MFD_PFUZE_H

#include <linux/interrupt.h>

struct mc_pfuze;
void pfuze_lock(struct mc_pfuze *mc_pfuze);
void pfuze_unlock(struct mc_pfuze *mc_pfuze);
int pfuze_reg_read(struct mc_pfuze *mc_pfuze, unsigned int offset,
		   unsigned char *val);
int pfuze_reg_write(struct mc_pfuze *mc_pfuze, unsigned int offset,
		    unsigned char val);
int pfuze_reg_rmw(struct mc_pfuze *mc_pfuze, unsigned int offset,
		  unsigned char mask, unsigned char val);
int pfuze_irq_mask(struct mc_pfuze *mc_pfuze, int irq);
int pfuze_irq_unmask(struct mc_pfuze *mc_pfuze, int irq);
int pfuze_irq_status(struct mc_pfuze *mc_pfuze, int irq, int *enabled,
		     int *pending);
int pfuze_irq_status(struct mc_pfuze *mc_pfuze, int irq, int *enabled,
		     int *pending);
int pfuze_irq_ack(struct mc_pfuze *mc_pfuze, int irq);
int pfuze_irq_request_nounmask(struct mc_pfuze *mc_pfuze, int irq,
			       irq_handler_t handler, const char *name,
			       void *dev);
int pfuze_irq_request(struct mc_pfuze *mc_pfuze, int irq, irq_handler_t handler,
		      const char *name, void *dev);
int pfuze_irq_free(struct mc_pfuze *mc_pfuze, int irq, void *dev);
unsigned int pfuze_get_flags(struct mc_pfuze *mc_pfuze);

#define PFUZE_I2C_RETRY_TIMES   10
#define PFUZE_NUM_IRQ		56
#define PFUZE_NUMREGS		128
#define PFUZE_IRQ_PWRONI	0
#define PFUZE_IRQ_LOWBATT	1
#define PFUZE_IRQ_THERM110	2
#define PFUZE_IRQ_THERM120	3
#define PFUZE_IRQ_THERM125	4
#define PFUZE_IRQ_THERM130	5
#define PFUZE_IRQ_SW1AFAULT	8
#define PFUZE_IRQ_SW1BFAULT	9
#define PFUZE_IRQ_SW1CFAULT	10
#define PFUZE_IRQ_SW2FAULT	11
#define PFUZE_IRQ_SW3AFAULT	12
#define PFUZE_IRQ_SW3BFAULT	13
#define PFUZE_IRQ_SW4FAULT	14
#define PFUZE_IRQ_SWBSTFAULT	24
#define PFUZE_IRQ_OTPECC	31
#define PFUZE_IRQ_VGEN1FAULT	32
#define PFUZE_IRQ_VGEN2FAULT	33
#define PFUZE_IRQ_VGEN3FAULT	34
#define PFUZE_IRQ_VGEN4FAULT	35
#define PFUZE_IRQ_VGEN5FAULT	36
#define PFUZE_IRQ_VGEN6FAULT	37

#define PFUZE100_SW1A		0
#define PFUZE100_SW1B		1
#define PFUZE100_SW1C		2
#define PFUZE100_SW2		3
#define PFUZE100_SW3A		4
#define PFUZE100_SW3B		5
#define PFUZE100_SW4		6
#define PFUZE100_SWBST		7
#define PFUZE100_VSNVS		8
#define PFUZE100_VREFDDR	9
#define PFUZE100_VGEN1		10
#define PFUZE100_VGEN2		11
#define PFUZE100_VGEN3		12
#define PFUZE100_VGEN4		13
#define PFUZE100_VGEN5		14
#define PFUZE100_VGEN6		15
#define PFUZE100_SW2_VOL6	1
#define PFUZE100_SW3_VOL6	0
#define PFUZE100_SW4_VOL6	1
/*#define PFUZE100_FIRST_VERSION*/

struct regulator_init_data;
struct pfuze_regulator_init_data {
	int id;
	struct regulator_init_data *init_data;
};
struct pfuze_regulator_platform_data {
	int num_regulators;
	struct pfuze_regulator_init_data *regulators;
	int (*pfuze_init)(struct mc_pfuze *pfuze);
};

struct pfuze_platform_data {
#define PFUZE_USE_REGULATOR	(1 << 0)
	unsigned int flags;
	int num_regulators;
	struct pfuze_regulator_init_data *regulators;
	int (*pfuze_init)(struct mc_pfuze *pfuze);
};
#endif
