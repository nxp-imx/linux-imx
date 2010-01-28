/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARM_ARCH_DEVICE_H
#define __ASM_ARM_ARCH_DEVICE_H

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>

#include <asm/mach/time.h>

#define MXS_MAX_DEVICES 128

struct mxs_sys_timer {
	struct sys_timer timer;
	unsigned char id;
	unsigned char clk_sel;
	unsigned char resv[2];
	int irq;
	struct clk *clk;
	void __iomem *base;
};

struct mxs_dev_lookup {
	char *name;
	unsigned long lock;
	int size;
	struct platform_device *pdev;
};

/* Define the Platform special structure for each device type*/
struct mxs_dma_plat_data {
	unsigned int burst8:1;
	unsigned int burst:1;
	unsigned int chan_base;
	unsigned int chan_num;
};

struct mxs_i2c_plat_data {
	unsigned int pioqueue_mode:1;
};

struct mxs_lradc_plat_data {
	unsigned int vddio_voltage;
	unsigned int battery_voltage;
};

struct mxskbd_keypair {
	int raw;
	int kcode;
};

struct mxs_kbd_plat_data {
	struct mxskbd_keypair *keypair;
	int channel;
};

extern void mxs_timer_init(struct mxs_sys_timer *timer);

extern void mxs_nop_release(struct device *dev);
extern int mxs_add_devices(struct platform_device *, int num, int level);
extern int mxs_add_device(struct platform_device *, int level);
extern struct platform_device *mxs_get_device(char *name, int id);
extern struct mxs_dev_lookup *mxs_get_devices(char *name);

/* mxs ssp sd/mmc data definitons */
struct mxs_mmc_platform_data {
	int (*hw_init)(void);
	void (*hw_release)(void);
	void (*cmd_pullup)(int enable);
	int (*get_wp)(void);
	unsigned long (*setclock)(unsigned long hz);
	unsigned int caps;
	unsigned int min_clk;
	unsigned int max_clk;
	int read_uA;
	int write_uA;
	char *power_mmc;
	char *clock_mmc;
};
/* end of mxs ssp sd/mmc data definitions */

#ifdef CONFIG_MXS_ICOLL
extern void __init avic_init_irq(void __iomem *base, int nr_irqs);
#endif

#endif
