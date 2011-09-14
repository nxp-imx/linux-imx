/*
 * da9052 declarations.
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

#ifndef __LINUX_MFD_DA9052_DA9052_H
#define __LINUX_MFD_DA9052_DA9052_H

#include <linux/slab.h>
#include <linux/mfd/core.h>

#include <linux/mfd/da9052/eh.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/led.h>


#define SPI 1
#define I2C 2

#define DA9052_SSC_DEVICE_NAME		"da9052_ssc"
#define DA9052_EH_DEVICE_NAME		"da9052_eh"

#define DA9052_IRQ			S3C_EINT(9)

/* Module specific error codes */
#define INVALID_REGISTER		2
#define INVALID_READ			3
#define INVALID_PAGE			4

/* Defines for Volatile and Non Volatile register types */
#define VOLATILE			0
#define NON_VOLATILE			1

/* Defines for cache state */
#define VALID				0
#define INVALID				1

/* Total number of registers in DA9057 */
#define DA9052_REG_CNT			DA9052_PAGE1_REG_END

/* Maximum number of registers that can be read/written by a singe request */
#define	MAX_READ_WRITE_CNT		16


#define DA9052_SSC_SPI_DEVICE_NAME	"da9052_ssc_spi"
#define PAGE_0_START			1
#define PAGE_0_END			127
#define PAGE_1_START			128
#define PAGE_1_END			255
#define ACTIVE_PAGE_0			0
#define ACTIVE_PAGE_1			1
#define PAGECON_0			0
#define PAGECON_128			128
#define RW_POL				1

#define DA9052_SSC_I2C_DEVICE_NAME		"da9052_ssc_i2c"
#define	DA9052_I2C_ADDR				0x90
#define	DA9052_SSC_I2C_PAGE_WRITE_MODE		0
#define DA9052_SSC_I2C_REPEAT_WRITE_MODE	1
#define DA9052_SSC_I2C_WRITE_MODE		DA9052_SSC_I2C_REPEAT_WRITE_MODE

struct da9052_ssc_msg {
	unsigned char	data;
	unsigned char	addr;
};

struct ssc_cache_entry{
	 unsigned char	val;
	 unsigned char	type:4;
	 unsigned char	status:4;
};

struct da9052_eh_nb{
	struct list_head nb_list;
	unsigned char	eve_type;
	void (*call_back)(struct da9052_eh_nb *, unsigned int);
};

struct da9052_regulator_init_data {
	struct regulator_init_data *init_data;
	int id;

};

struct da9052_regulator_platform_data {
	struct regulator_init_data *regulators;
};

struct da9052_tsi_platform_data {
	u32	pen_up_interval;
	u16	tsi_delay_bit_shift;
	u16	tsi_skip_bit_shift;
	u16	num_gpio_tsi_register;
	u16	tsi_supply_voltage;
	u16	tsi_ref_source;
	u16	max_tsi_delay;
	u16	max_tsi_skip_slot;
};


struct da9052 {
	struct mutex ssc_lock;
	struct mutex eve_nb_lock;
	struct mutex manconv_lock;
	struct work_struct eh_isr_work;
	struct ssc_cache_entry ssc_cache[DA9052_REG_CNT];
	int (*read) (struct da9052 *da9052, struct da9052_ssc_msg *sscmsg);
	int (*write) (struct da9052 *da9052, struct da9052_ssc_msg *sscmsg);
	int (*read_many) (struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int cnt);
	int (*write_many)(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int cnt);
	int (*register_event_notifier)(struct da9052 *da9052,
		struct da9052_eh_nb *nb);
	int (*unregister_event_notifier)(struct da9052 *da9052,
		struct da9052_eh_nb *nb);
	int num_regulators;
	int connecting_device;
	int irq;
	struct		spi_device *spi_dev;
	unsigned int	spi_active_page;
	unsigned char	rw_pol;
	unsigned char	*spi_rx_buf;
	unsigned char	*spi_tx_buf;

	struct i2c_client *i2c_client;
	struct device *dev;
	struct i2c_adapter *adapter;
	unsigned char	slave_addr;
};


struct da9052_platform_data {
	int (*init)(struct da9052 *da9052);
	int	irq_high;
	int	irq_base;
	int	gpio_base;
	int	num_regulators;
	struct da9052 *da9052;
	struct regulator_init_data *regulators;
	struct da9052_leds_platform_data *led_data;
	struct da9052_tsi_platform_data *tsi_data;
};

struct da9052_ssc_ops {
	int (*write)(struct da9052 *da9052, struct da9052_ssc_msg *msg);
	int (*read)(struct da9052 *da9052, struct da9052_ssc_msg *msg);
	int (*write_many)(struct da9052 *da9052,
	struct da9052_ssc_msg *sscmsg, int msg_no);
	int (*read_many)(struct da9052 *da9052,
	struct da9052_ssc_msg *sscmsg, int msg_no);
	int (*device_register)(struct da9052 *da9052);
	void (*device_unregister)(void);
};

int da9052_ssc_write(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg);
int da9052_ssc_read(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg);
int da9052_ssc_write_many(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int cnt);
int da9052_ssc_read_many(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int cnt);

int da9052_spi_write(struct da9052 *da9052,
		struct da9052_ssc_msg *msg);
int da9052_spi_read(struct da9052 *da9052,
		struct da9052_ssc_msg *msg);

int da9052_spi_write_many(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg,
		int msg_no);
int da9052_spi_read_many(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg,
		int msg_no);

void da9052_ssc_exit(struct da9052 *da9052);
int da9052_ssc_init(struct da9052 *da9052);

/* I2C specific Functions */
int da9052_i2c_write(struct da9052 *da9052, struct da9052_ssc_msg *msg);
int da9052_i2c_read(struct da9052 *da9052, struct da9052_ssc_msg *msg);
int da9052_i2c_write_many(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int msg_no);
int da9052_i2c_read_many(struct da9052 *da9052,
		struct da9052_ssc_msg *sscmsg, int msg_no);

void da9052_lock(struct da9052 *da9052);
void da9052_unlock(struct da9052 *da9052);
int eh_register_nb(struct da9052 *da9052, struct da9052_eh_nb *nb);
int eh_unregister_nb(struct da9052 *da9052, struct da9052_eh_nb *nb);
int da9052_manual_read(struct da9052 *da9052,
		unsigned char channel);
#endif /* __LINUX_MFD_DA9052_DA9052_H */
