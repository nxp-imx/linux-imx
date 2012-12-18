/*
 * Copyright (C) 2009-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/leds.h>

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

struct fsl_otp_data {
	char 		**fuse_name;
	char		*regulator_name;
	unsigned int 	fuse_num;
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
	unsigned int btn_enable; /* detect enable bits */
	unsigned int btn_irq_stat; /* detect irq status bits */
	unsigned int btn_irq_ctrl; /* detect irq enable bits */
};

struct mxs_touchscreen_plat_data {
	u8 x_plus_chan;
	u8 x_minus_chan;
	u8 y_plus_chan;
	u8 y_minus_chan;
	unsigned int x_plus_val;
	unsigned int x_minus_val;
	unsigned int y_plus_val;
	unsigned int y_minus_val;
	unsigned int x_plus_mask;
	unsigned int x_minus_mask;
	unsigned int y_plus_mask;
	unsigned int y_minus_mask;
};

struct mxs_auart_plat_data {
	unsigned int fifo_size:6;
	unsigned int dma_mode:1;
	unsigned int timeout;
	unsigned int dma_rx_buffer_size;
	const char *clk;
};

struct mxs_pwm_led {
	struct led_classdev dev;
	const char *name;
	unsigned int pwm;
};

struct mxs_pwm_leds_plat_data {
	unsigned int num;
	struct mxs_pwm_led *leds;
};

struct mxs_mma7450_platform_data {
	char *reg_dvdd_io;
	char *reg_avdd;
	void (*gpio_pin_get) (void);
	void (*gpio_pin_put) (void);
	int int1;
	int int2;
};

struct mxs_spi_platform_data {
	int (*hw_pin_init)(void);
	int (*hw_pin_release)(void);

	int slave_mode;
	char *clk;
};

struct mxs_audio_platform_data {
	int intr_id_hp;
	int ext_ram;
	struct clk *saif_mclock;

	int hp_irq;
	int (*hp_status) (void);

	int sysclk;

	int (*init) (void);	/* board specific init */
	int (*amp_enable) (int enable);
	int (*finit) (void);	/* board specific finit */
	void *priv;		/* used by board specific functions */
  int inited;
};

struct mxs_persistent_bit_config {
	int reg;
	int start;
	int width;
	const char *name;
};

struct mxs_platform_persistent_data {
	const struct mxs_persistent_bit_config *bit_config_tab;
	int bit_config_cnt;
};

extern void mxs_timer_init(struct mxs_sys_timer *timer);
extern void mxs_nomatch_timer_init(struct mxs_sys_timer *timer);

extern void mxs_nop_release(struct device *dev);
extern int mxs_add_devices(struct platform_device *, int num, int level);
extern int mxs_add_device(struct platform_device *, int level);
extern struct platform_device *mxs_get_device(char *name, int id);
extern struct mxs_dev_lookup *mxs_get_devices(char *name);

extern int iram_init(unsigned long base, unsigned long size);

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
	int fastpath_sz;
};
/* end of mxs ssp sd/mmc data definitions */

#ifdef CONFIG_MXS_ICOLL
extern void __init avic_init_irq(void __iomem *base, int nr_irqs);
#endif

#endif
