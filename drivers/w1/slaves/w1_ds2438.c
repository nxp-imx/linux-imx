/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/idr.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"
#include "w1_ds2438.h"

/* W1 slave DS2438 famliy operations */
int w1_ds2438_read_page(struct device *dev, u8 page, u8 *buf)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);
	if ((page >= DS2438_PAGE_NUM) || (buf == NULL))
		return -EINVAL;

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, W1_READ_SCRATCHPAD);
		w1_write_8(slave->master, page);
		w1_read_block(slave->master, buf, DS2438_PAGE_SIZE);
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}
EXPORT_SYMBOL(w1_ds2438_read_page);

int w1_ds2438_write_page(struct device *dev, u8 page, u8 *buf)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);
	if ((page >= DS2438_PAGE_NUM) || (buf == NULL))
		return -EINVAL;

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, DS2438_WRITE_SCRATCHPAD);
		w1_write_8(slave->master, page);
		w1_write_block(slave->master, buf, DS2438_PAGE_SIZE);
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}
EXPORT_SYMBOL(w1_ds2438_write_page);

int w1_ds2438_command(struct device *dev, u8 command, u8 data)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, command);
		switch (command) {
		case DS2438_COPY_SCRATCHPAD:
		case DS2438_RECALL_MEMORY:
			w1_write_8(slave->master, data);
		}
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}
EXPORT_SYMBOL(w1_ds2438_command);

int w1_ds2438_drain_sram(struct device *dev, u8 page)
{
	return w1_ds2438_command(dev, DS2438_COPY_SCRATCHPAD, page);
}
EXPORT_SYMBOL(w1_ds2438_drain_sram);

int w1_ds2438_load_sram(struct device *dev, u8 page)
{
	return w1_ds2438_command(dev, DS2438_RECALL_MEMORY, page);
}
EXPORT_SYMBOL(w1_ds2438_load_sram);

static DEFINE_IDR(bat_idr);
static DEFINE_MUTEX(bat_idr_lock);

static int new_bat_id(void)
{
	int ret;

	while (1) {
		int id;

		ret = idr_pre_get(&bat_idr, GFP_KERNEL);
		if (ret == 0)
			return -ENOMEM;

		mutex_lock(&bat_idr_lock);
		ret = idr_get_new(&bat_idr, NULL, &id);
		mutex_unlock(&bat_idr_lock);

		if (ret == 0) {
			ret = id & MAX_ID_MASK;
			break;
		} else if (ret == -EAGAIN) {
			continue;
		} else {
			break;
		}
	}

	return ret;
}

static void release_bat_id(int id)
{
	mutex_lock(&bat_idr_lock);
	idr_remove(&bat_idr, id);
	mutex_unlock(&bat_idr_lock);
}

static int ds2438_add_slave(struct w1_slave *slave)
{
	int ret;
	int id;
	struct platform_device *pdev;

	id = new_bat_id();
	if (id < 0) {
		ret = id;
		goto noid;
	}

	pdev = platform_device_alloc(DS2438_DEV_NAME, id);
	if (!pdev) {
		ret = -ENOMEM;
		goto pdev_alloc_failed;
	}
	pdev->dev.parent = &slave->dev;

	ret = platform_device_add(pdev);
	if (ret)
		goto pdev_add_failed;

	dev_set_drvdata(&slave->dev, pdev);
		goto success;

pdev_add_failed:
	platform_device_unregister(pdev);
pdev_alloc_failed:
		release_bat_id(id);
noid:
success:
		return ret;
}

static void ds2438_remove_slave(struct w1_slave *slave)
{
	struct platform_device *pdev = dev_get_drvdata(&slave->dev);
	int id = pdev->id;

	platform_device_unregister(pdev);
	release_bat_id(id);
}

static struct w1_family_ops w1_ds2438_fops = {
	.add_slave = ds2438_add_slave,
	.remove_slave = ds2438_remove_slave,
};

static struct w1_family w1_family_ds2438 = {
	.fid = W1_FAMILY_DS2438,
	.fops = &w1_ds2438_fops,
};

static int __init w1_ds2438_init(void)
{
	pr_info("1-wire driver for the DS2438 smart battery monitor\n");
	idr_init(&bat_idr);
	return w1_register_family(&w1_family_ds2438);
}

static void __exit w1_ds2438_fini(void)
{
	w1_unregister_family(&w1_family_ds2438);
	idr_destroy(&bat_idr);
}

module_init(w1_ds2438_init);
module_exit(w1_ds2438_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductors Inc");
MODULE_DESCRIPTION("1-wire DS2438 family, smart battery monitor.");
