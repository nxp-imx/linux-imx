/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Implementation based on w1_ds2433.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#ifdef CONFIG_W1_F51_CRC
#include <linux/crc16.h>

#define CRC16_INIT		0
#define CRC16_VALID		0xb001

#endif

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"

#define W1_EEPROM_SIZE		32
#define W1_PAGE_SIZE		32
#define W1_PAGE_BITS		5
#define W1_PAGE_MASK		0x1F

#define W1_F51_TIME		300

#define W1_F51_READ_EEPROM	0xB8
#define W1_F51_WRITE_SCRATCH	0x6C
#define W1_F51_READ_SCRATCH	0x69
#define W1_F51_COPY_SCRATCH	0x48
#define W1_STATUS_OFFSET        0x0001
#define W1_EEPROM_OFFSET        0x0007
#define W1_SPECIAL_OFFSET       0x0008
#define W1_EEPROM_BLOCK_0       0x0020
#define W1_EEPROM_BLOCK_1       0x0030
#define W1_SRAM                 0x0080
struct w1_f51_data {
	u8 memory[W1_EEPROM_SIZE];
	u32 validcrc;
};

/**
 * Check the file size bounds and adjusts count as needed.
 * This would not be needed if the file size didn't reset to 0 after a write.
 */
static inline size_t w1_f51_fix_count(loff_t off, size_t count, size_t size)
{
	if (off > size)
		return 0;

	if ((off + count) > size)
		return (size - off);

	return count;
}

#ifdef CONFIG_W1_F51_CRC
static int w1_f51_refresh_block(struct w1_slave *sl, struct w1_f51_data *data,
				int block)
{
	u8 wrbuf[3];
	int off = block * W1_PAGE_SIZE;
	if (data->validcrc & (1 << block))
		return 0;

	if (w1_reset_select_slave(sl)) {
		data->validcrc = 0;
		return -EIO;
	}
	wrbuf[0] = W1_F51_READ_EEPROM;
	wrbuf[1] = off & 0xff;
	wrbuf[2] = off >> 8;
	w1_write_block(sl->master, wrbuf, 3);
	w1_read_block(sl->master, &data->memory[off], W1_PAGE_SIZE);

	/* cache the block if the CRC is valid */
	if (crc16(CRC16_INIT, &data->memory[off], W1_PAGE_SIZE) == CRC16_VALID)
		data->validcrc |= (1 << block);

	return 0;
}
#endif				/* CONFIG_W1_F51_CRC */

static ssize_t w1_f51_read_bin(struct kobject *kobj, char *buf, loff_t off,
			       size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
#ifdef CONFIG_W1_F51_CRC
	struct w1_f51_data *data = sl->family_data;
	int i, min_page, max_page;
#else
	u8 wrbuf[3];
#endif

	if ((count = w1_f51_fix_count(off, count, W1_EEPROM_SIZE)) == 0) {
		return 0;
	}
	mutex_lock(&sl->master->mutex);
#ifdef CONFIG_W1_F51_CRC
	min_page = (off >> W1_PAGE_BITS);
	max_page = (off + count - 1) >> W1_PAGE_BITS;
	for (i = min_page; i <= max_page; i++) {
		if (w1_f51_refresh_block(sl, data, i)) {
			count = -EIO;
			goto out_up;
		}
	}
	memcpy(buf, &data->memory[off], count);

#else				/* CONFIG_W1_F51_CRC */

	/* read directly from the EEPROM */
	if (w1_reset_select_slave(sl)) {
		count = -EIO;
		goto out_up;
	}
	off = (loff_t) W1_EEPROM_BLOCK_0;
	wrbuf[0] = W1_F51_READ_EEPROM;
	wrbuf[1] = off & 0xff;
	wrbuf[2] = off >> 8;
	w1_write_block(sl->master, wrbuf, 3);
	if (w1_reset_select_slave(sl)) {
		count = -EIO;
		goto out_up;
	}

	wrbuf[0] = W1_F51_READ_SCRATCH;
	wrbuf[1] = off & 0xff;
	wrbuf[2] = off >> 8;
	w1_write_block(sl->master, wrbuf, 3);
	w1_read_block(sl->master, buf, count);

#endif				/* CONFIG_W1_F51_CRC */

      out_up:
	mutex_unlock(&sl->master->mutex);
	return count;
}

/**
 * Writes to the scratchpad and reads it back for verification.
 * Then copies the scratchpad to EEPROM.
 * The data must be on one page.
 * The master must be locked.
 *
 * @param sl	The slave structure
 * @param addr	Address for the write
 * @param len   length must be <= (W1_PAGE_SIZE - (addr & W1_PAGE_MASK))
 * @param data	The data to write
 * @return	0=Success -1=failure
 */
static int w1_f51_write(struct w1_slave *sl, int addr, int len, const u8 * data)
{
	u8 wrbuf[4];
	u8 rdbuf[W1_EEPROM_SIZE + 3];
	u8 es = (addr + len - 1) & 0x1f;
	/* Write the data to the scratchpad */
	if (w1_reset_select_slave(sl))
		return -1;
	wrbuf[0] = W1_F51_WRITE_SCRATCH;
	wrbuf[1] = addr & 0xff;
	wrbuf[2] = addr >> 8;

	w1_write_block(sl->master, wrbuf, 3);
	w1_write_block(sl->master, data, len);
	/* Read the scratchpad and verify */
	if (w1_reset_select_slave(sl))
		return -1;
	wrbuf[0] = W1_F51_READ_SCRATCH;
	w1_write_block(sl->master, wrbuf, 3);
	w1_read_block(sl->master, rdbuf, len + 3);
	/* Compare what was read against the data written */
	if (memcmp(data, &rdbuf[0], len) != 0) {
		printk("Error reading the scratch Pad\n");
		return -1;
	}
	/* Copy the scratchpad to EEPROM */
	if (w1_reset_select_slave(sl))
		return -1;
	wrbuf[0] = W1_F51_COPY_SCRATCH;
	wrbuf[3] = es;
	w1_write_block(sl->master, wrbuf, 4);
	/* Sleep for 5 ms to wait for the write to complete */
	msleep(5);

	/* Reset the bus to wake up the EEPROM (this may not be needed) */
	w1_reset_bus(sl->master);

	return 0;
}

static ssize_t w1_f51_write_bin(struct kobject *kobj, char *buf, loff_t off,
				size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	int addr;

	if ((count = w1_f51_fix_count(off, count, W1_EEPROM_SIZE)) == 0)
		return 0;
	off = (loff_t) 0x0020;
#ifdef CONFIG_W1_F51_CRC
	/* can only write full blocks in cached mode */
	if ((off & W1_PAGE_MASK) || (count & W1_PAGE_MASK)) {
		dev_err(&sl->dev, "invalid offset/count off=%d cnt=%zd\n",
			(int)off, count);
		return -EINVAL;
	}

	/* make sure the block CRCs are valid */
	for (idx = 0; idx < count; idx += W1_PAGE_SIZE) {
		if (crc16(CRC16_INIT, &buf[idx], W1_PAGE_SIZE) != CRC16_VALID) {
			dev_err(&sl->dev, "bad CRC at offset %d\n", (int)off);
			return -EINVAL;
		}
	}
#endif				/* CONFIG_W1_F51_CRC */

	mutex_lock(&sl->master->mutex);

	/* Can only write data to one page at a time */
	addr = off;
	if (w1_f51_write(sl, addr, count, buf) < 0) {
		count = -EIO;
		goto out_up;
	}

      out_up:
	mutex_unlock(&sl->master->mutex);

	return count;
}

static struct bin_attribute w1_f51_bin_attr = {
	.attr = {
		 .name = "eeprom",
		 .mode = S_IRUGO | S_IWUSR,
		 .owner = THIS_MODULE,
		 },
	.size = W1_EEPROM_SIZE,
	.read = w1_f51_read_bin,
	.write = w1_f51_write_bin,
};

static int w1_f51_add_slave(struct w1_slave *sl)
{
	int err;
#ifdef CONFIG_W1_F51_CRC
	struct w1_f51_data *data;
	data = kmalloc(sizeof(struct w1_f51_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	memset(data, 0, sizeof(struct w1_f51_data));
	sl->family_data = data;

#endif				/* CONFIG_W1_F51_CRC */

	err = sysfs_create_bin_file(&sl->dev.kobj, &w1_f51_bin_attr);

#ifdef CONFIG_W1_F51_CRC
	if (err)
		kfree(data);
#endif				/* CONFIG_W1_F51_CRC */

	return err;
}

static void w1_f51_remove_slave(struct w1_slave *sl)
{
#ifdef CONFIG_W1_F51_CRC
	kfree(sl->family_data);
	sl->family_data = NULL;
#endif				/* CONFIG_W1_F51_CRC */
	sysfs_remove_bin_file(&sl->dev.kobj, &w1_f51_bin_attr);
}

static struct w1_family_ops w1_f51_fops = {
	.add_slave = w1_f51_add_slave,
	.remove_slave = w1_f51_remove_slave,
};

static struct w1_family w1_family_51 = {
	.fid = W1_EEPROM_DS2751,
	.fops = &w1_f51_fops,
};

static int __init w1_f51_init(void)
{
	return w1_register_family(&w1_family_51);
}

static void __exit w1_f51_fini(void)
{
	w1_unregister_family(&w1_family_51);
}

module_init(w1_f51_init);
module_exit(w1_f51_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductors Inc");
MODULE_DESCRIPTION
    ("w1 family 51 driver for DS2751, Battery Level Sensing Device");
