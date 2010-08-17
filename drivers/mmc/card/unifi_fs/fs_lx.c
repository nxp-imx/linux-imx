/*
 * fs_lx.c - Freescale SDIO glue module for UniFi.
 *
 * Copyright (C) 2008 Cambridge Silicon Radio Ltd.
 *
 *  Important:
 *      This module does not support more than one device driver instances.
 *
 */
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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <linux/scatterlist.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <linux/clk.h>
#include <linux/err.h>

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>

#include <mach/mmc.h>
#include <mach/gpio.h>

#include "fs_sdio_api.h"

struct regulator_unifi {
	struct regulator *reg_gpo1;
	struct regulator *reg_gpo2;
	struct regulator *reg_1v5_ana_bb;
	struct regulator *reg_vdd_vpa;
	struct regulator *reg_1v5_dd;
};

static struct sdio_driver sdio_unifi_driver;

static int fs_sdio_probe(struct sdio_func *func,
			 const struct sdio_device_id *id);
static void fs_sdio_remove(struct sdio_func *func);
static void fs_sdio_irq(struct sdio_func *func);
static int fs_sdio_suspend(struct device *dev, pm_message_t state);
static int fs_sdio_resume(struct device *dev);

/* Globals to store the context to this module and the device driver */
static struct sdio_dev *available_sdio_dev;
static struct fs_driver *available_driver;
struct mxc_unifi_platform_data *plat_data;

extern void mxc_mmc_force_detect(int id);

enum sdio_cmd_direction {
	CMD_READ,
	CMD_WRITE,
};

static int fsl_io_rw_direct(struct mmc_card *card, int write, unsigned fn,
	unsigned addr, u8 in, u8 *out)
{
	struct mmc_command cmd;
	int err;

	BUG_ON(!card);
	BUG_ON(fn > 7);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg = write ? 0x80000000 : 0x00000000;
	cmd.arg |= fn << 28;
	cmd.arg |= (write && out) ? 0x08000000 : 0x00000000;
	cmd.arg |= addr << 9;
	cmd.arg |= in;
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	if (mmc_host_is_spi(card->host)) {
		/* host driver already reported errors */
	} else {
		if (cmd.resp[0] & R5_ERROR)
			return -EIO;
		if (cmd.resp[0] & R5_FUNCTION_NUMBER)
			return -EINVAL;
		if (cmd.resp[0] & R5_OUT_OF_RANGE)
			return -ERANGE;
	}

	if (out) {
		if (mmc_host_is_spi(card->host))
			*out = (cmd.resp[0] >> 8) & 0xFF;
		else
			*out = cmd.resp[0] & 0xFF;
	}

	return 0;
}


int fs_sdio_readb(struct sdio_dev *fdev, int funcnum, unsigned long addr,
		  unsigned char *pdata)
{
	int err;
	char val;

	sdio_claim_host(fdev->func);
	if (funcnum == 0)
		val = sdio_f0_readb(fdev->func, (unsigned int)addr, &err);
	else
		val = sdio_readb(fdev->func, (unsigned int)addr, &err);
	sdio_release_host(fdev->func);
	if (!err)
		*pdata = val;
	else
		printk(KERN_ERR "fs_lx: readb error,fun=%d,addr=%d,data=%d,"
		       "err=%d\n", funcnum, (int)addr, *pdata, err);

	return err;
}
EXPORT_SYMBOL(fs_sdio_readb);

int fs_sdio_writeb(struct sdio_dev *fdev, int funcnum, unsigned long addr,
		   unsigned char data)
{
	int err;

	sdio_claim_host(fdev->func);
	if (funcnum == 0)
		err = fsl_io_rw_direct(fdev->func->card, 1, 0, addr,
				       data, NULL);
	else
		sdio_writeb(fdev->func, data, (unsigned int)addr, &err);
	sdio_release_host(fdev->func);

	if (err)
		printk(KERN_ERR "fs_lx: writeb error,fun=%d,addr=%d,data=%d,"
		       "err=%d\n", funcnum, (int)addr, data, err);
	return err;
}
EXPORT_SYMBOL(fs_sdio_writeb);

int fs_sdio_block_rw(struct sdio_dev *fdev, int funcnum, unsigned long addr,
		     unsigned char *pdata, unsigned int count, int direction)
{
	int err;

	sdio_claim_host(fdev->func);
	if (direction == CMD_READ)
		err = sdio_memcpy_fromio(fdev->func, pdata, addr, count);
	else
		err = sdio_memcpy_toio(fdev->func, addr, pdata, count);
	sdio_release_host(fdev->func);

	return err;
}
EXPORT_SYMBOL(fs_sdio_block_rw);

int fs_sdio_enable_interrupt(struct sdio_dev *fdev, int enable)
{
	struct mmc_host *host = fdev->func->card->host;
	unsigned long flags;

	spin_lock_irqsave(&fdev->lock, flags);
	if (enable) {
		if (!fdev->int_enabled) {
		fdev->int_enabled = 1;
		host->ops->enable_sdio_irq(host, 1);
		}
	} else {
		if (fdev->int_enabled) {
		host->ops->enable_sdio_irq(host, 0);
		fdev->int_enabled = 0;
		}
	}
	spin_unlock_irqrestore(&fdev->lock, flags);

	return 0;
}
EXPORT_SYMBOL(fs_sdio_enable_interrupt);

int fs_sdio_disable(struct sdio_dev *fdev)
{
	int err;
	sdio_claim_host(fdev->func);
	err = sdio_disable_func(fdev->func);
	sdio_release_host(fdev->func);
	if (err)
		printk(KERN_ERR "fs_lx:fs_sdio_disable error,err=%d\n", err);
	return err;
}
EXPORT_SYMBOL(fs_sdio_disable);

int fs_sdio_enable(struct sdio_dev *fdev)
{
	int err = 0;

	sdio_claim_host(fdev->func);
	err = sdio_disable_func(fdev->func);
	err = sdio_enable_func(fdev->func);
	sdio_release_host(fdev->func);
	if (err)
		printk(KERN_ERR "fs_lx:fs_sdio_enable error,err=%d\n", err);
	return err;
}
EXPORT_SYMBOL(fs_sdio_enable);

int fs_sdio_set_max_clock_speed(struct sdio_dev *fdev, int max_khz)
{
	struct mmc_card *card = fdev->func->card;

	/* Respect the host controller's min-max. */
	max_khz *= 1000;
	if (max_khz < card->host->f_min)
		max_khz = card->host->f_min;
	if (max_khz > card->host->f_max)
		max_khz = card->host->f_max;

	card->host->ios.clock = max_khz;
	card->host->ops->set_ios(card->host, &card->host->ios);

	return max_khz / 1000;
}
EXPORT_SYMBOL(fs_sdio_set_max_clock_speed);

int fs_sdio_set_block_size(struct sdio_dev *fdev, int blksz)
{
	return 0;
}
EXPORT_SYMBOL(fs_sdio_set_block_size);

/*
 * ---------------------------------------------------------------------------
 *
 * Turn on the power of WIFI card
 *
 * ---------------------------------------------------------------------------
 */
static void fs_unifi_power_on(void)
{
	struct regulator_unifi *reg_unifi;
	unsigned int tmp;

	reg_unifi = plat_data->priv;

	if (reg_unifi->reg_gpo1)
		regulator_enable(reg_unifi->reg_gpo1);
	if (reg_unifi->reg_gpo2)
		regulator_enable(reg_unifi->reg_gpo2);

	if (plat_data->enable)
		plat_data->enable(1);

	if (reg_unifi->reg_1v5_ana_bb) {
		regulator_set_voltage(reg_unifi->reg_1v5_ana_bb,
					1500000, 1500000);
		regulator_enable(reg_unifi->reg_1v5_ana_bb);
	}
	if (reg_unifi->reg_vdd_vpa) {
		tmp = regulator_get_voltage(reg_unifi->reg_vdd_vpa);
		if (tmp < 3000000 || tmp > 3600000)
			regulator_set_voltage(reg_unifi->reg_vdd_vpa,
					      3000000, 3000000);
		regulator_enable(reg_unifi->reg_vdd_vpa);
	}
	/* WL_1V5DD should come on last, 10ms after other supplies */
	msleep(10);
	if (reg_unifi->reg_1v5_dd) {
		regulator_set_voltage(reg_unifi->reg_1v5_dd,
					1500000, 1500000);
		regulator_enable(reg_unifi->reg_1v5_dd);
	}
	msleep(10);
}

/*
 * ---------------------------------------------------------------------------
 *
 * Turn off the power of WIFI card
 *
 * ---------------------------------------------------------------------------
 */
static void fs_unifi_power_off(void)
{
	struct regulator_unifi *reg_unifi;

	reg_unifi = plat_data->priv;
	if (reg_unifi->reg_1v5_dd)
		regulator_disable(reg_unifi->reg_1v5_dd);
	if (reg_unifi->reg_vdd_vpa)
		regulator_disable(reg_unifi->reg_vdd_vpa);

	if (reg_unifi->reg_1v5_ana_bb)
		regulator_disable(reg_unifi->reg_1v5_ana_bb);

	if (plat_data->enable)
		plat_data->enable(0);

	if (reg_unifi->reg_gpo2)
		regulator_disable(reg_unifi->reg_gpo2);

	if (reg_unifi->reg_gpo1)
		regulator_disable(reg_unifi->reg_gpo1);
}

/* This should be made conditional on being slot 2 too - so we can
 * use a plug in card in slot 1
 */
int fs_sdio_hard_reset(struct sdio_dev *fdev)
{
	return 0;
}
EXPORT_SYMBOL(fs_sdio_hard_reset);

static const struct sdio_device_id fs_sdio_ids[] = {
	{SDIO_DEVICE(0x032a, 0x0001)},
	{ /* end: all zeroes */	},
};

static struct sdio_driver sdio_unifi_driver = {
	.name = "fs_unifi",
	.probe = fs_sdio_probe,
	.remove = fs_sdio_remove,
	.id_table = fs_sdio_ids,
	.drv = {
		.suspend = fs_sdio_suspend,
		.resume	= fs_sdio_resume,
	}
};

int fs_sdio_register_driver(struct fs_driver *driver)
{
	int ret, retry;

	/* Switch us on, sdio device may exist if power is on by default. */
	plat_data->hardreset(0);
	if (available_sdio_dev)
		mxc_mmc_force_detect(plat_data->host_id);
	/* Wait for card removed */
	for (retry = 0; retry < 100; retry++) {
		if (!available_sdio_dev)
			break;
		msleep(100);
	}
	if (retry == 100)
		printk(KERN_ERR "fs_sdio_register_driver: sdio device exists, "
				"timeout for card removed");
	fs_unifi_power_on();
	plat_data->hardreset(1);
	msleep(500);
	mxc_mmc_force_detect(plat_data->host_id);
	for (retry = 0; retry < 100; retry++) {
		if (available_sdio_dev)
			break;
		msleep(50);
	}
	if (retry == 100)
		printk(KERN_ERR "fs_sdio_register_driver: Timeout waiting"
				" for card added\n");
	/* Store the context to the device driver to the global */
	available_driver = driver;

	/*
	 * If available_sdio_dev is not NULL, probe has been called,
	 * so pass the probe to the registered driver
	 */
	if (available_sdio_dev) {
		/* Store the context to the new device driver */
		available_sdio_dev->driver = driver;

		printk(KERN_INFO "fs_sdio_register_driver: Glue exists, add "
		       "device driver and register IRQ\n");
		driver->probe(available_sdio_dev);

		/* Register the IRQ handler to the SDIO IRQ. */
		sdio_claim_host(available_sdio_dev->func);
		ret = sdio_claim_irq(available_sdio_dev->func, fs_sdio_irq);
		sdio_release_host(available_sdio_dev->func);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(fs_sdio_register_driver);

void fs_sdio_unregister_driver(struct fs_driver *driver)
{
	/*
	 * If available_sdio_dev is not NULL, probe has been called,
	 * so pass the remove to the registered driver to clean up.
	 */
	if (available_sdio_dev) {
		struct mmc_host *host = available_sdio_dev->func->card->host;

		printk(KERN_INFO "fs_sdio_unregister_driver: Glue exists, "
		       "unregister IRQ and remove device driver\n");

		/* Unregister the IRQ handler first. */
		sdio_claim_host(available_sdio_dev->func);
		sdio_release_irq(available_sdio_dev->func);
		sdio_release_host(available_sdio_dev->func);

		driver->remove(available_sdio_dev);

		if (!available_sdio_dev->int_enabled) {
			available_sdio_dev->int_enabled = 1;
			host->ops->enable_sdio_irq(host, 1);
		}

		/* Invalidate the context to the device driver */
		available_sdio_dev->driver = NULL;
	}

	/* invalidate the context to the device driver to the global */
	available_driver = NULL;
	/* Power down the UniFi */
	fs_unifi_power_off();

}
EXPORT_SYMBOL(fs_sdio_unregister_driver);

static void fs_sdio_irq(struct sdio_func *func)
{
	struct sdio_dev *fdev = (struct sdio_dev *)sdio_get_drvdata(func);
	if (fdev->driver) {
		if (fdev->driver->card_int_handler)
			fdev->driver->card_int_handler(fdev);
	}
}

#ifdef CONFIG_PM
static int fs_sdio_suspend(struct device *dev, pm_message_t state)
{
	struct sdio_dev *fdev = available_sdio_dev;

	/* Pass event to the registered driver. */
	if (fdev->driver)
		if (fdev->driver->suspend)
			fdev->driver->suspend(fdev, state);

	return 0;
}

static int fs_sdio_resume(struct device *dev)
{
	struct sdio_dev *fdev = available_sdio_dev;

	/* Pass event to the registered driver. */
	if (fdev->driver)
		if (fdev->driver->resume)
			fdev->driver->resume(fdev);

	return 0;
}
#else
#define fs_sdio_suspend NULL
#define fs_sdio_resume  NULL
#endif

static int fs_sdio_probe(struct sdio_func *func,
			 const struct sdio_device_id *id)
{
	struct sdio_dev *fdev;

	/* Allocate our private context */
	fdev = kmalloc(sizeof(struct sdio_dev), GFP_KERNEL);
	if (!fdev)
		return -ENOMEM;
	available_sdio_dev = fdev;
	memset(fdev, 0, sizeof(struct sdio_dev));
	fdev->func = func;
	fdev->vendor_id = id->vendor;
	fdev->device_id = id->device;
	fdev->max_blocksize = func->max_blksize;
	fdev->int_enabled = 1;
	spin_lock_init(&fdev->lock);

	/* Store our context in the MMC driver */
	printk(KERN_INFO "fs_sdio_probe: Add glue driver\n");
	sdio_set_drvdata(func, fdev);

	return 0;
}

static void fs_sdio_remove(struct sdio_func *func)
{
	struct sdio_dev *fdev = (struct sdio_dev *)sdio_get_drvdata(func);
	struct mmc_host *host = func->card->host;

	/* If there is a registered device driver, pass on the remove */
	if (fdev->driver) {
		printk(KERN_INFO "fs_sdio_remove: Free IRQ and remove device "
		       "driver\n");
		/* Unregister the IRQ handler first. */
		sdio_claim_host(fdev->func);
		sdio_release_irq(func);
		sdio_release_host(fdev->func);

		fdev->driver->remove(fdev);

		if (!fdev->int_enabled) {
			fdev->int_enabled = 1;
			host->ops->enable_sdio_irq(host, 1);
		}
	}

	/* Unregister the card context from the MMC driver. */
	sdio_set_drvdata(func, NULL);

	/* Invalidate the global to our context. */
	available_sdio_dev = NULL;
	kfree(fdev);
}

static int fs_unifi_init(void)
{
	struct regulator_unifi *reg_unifi;
	struct regulator *reg;
	int err = 0;

	plat_data = get_unifi_plat_data();

	if (!plat_data)
		return -ENOENT;

	reg_unifi = kzalloc(sizeof(struct regulator_unifi), GFP_KERNEL);
	if (!reg_unifi)
		return -ENOMEM;

	if (plat_data->reg_gpo1) {
		reg = regulator_get(NULL, plat_data->reg_gpo1);
		if (!IS_ERR(reg))
			reg_unifi->reg_gpo1 = reg;
		else {
			err = -EINVAL;
			goto err_reg_gpo1;
		}
	}

	if (plat_data->reg_gpo2) {
		reg = regulator_get(NULL, plat_data->reg_gpo2);
		if (!IS_ERR(reg))
			reg_unifi->reg_gpo2 = reg;
		else {
			err = -EINVAL;
			goto err_reg_gpo2;
		}
	}

	if (plat_data->reg_1v5_ana_bb) {
		reg = regulator_get(NULL, plat_data->reg_1v5_ana_bb);
		if (!IS_ERR(reg))
			reg_unifi->reg_1v5_ana_bb = reg;
		else {
			err = -EINVAL;
			goto err_reg_1v5_ana_bb;
		}
	}

	if (plat_data->reg_vdd_vpa) {
		reg = regulator_get(NULL, plat_data->reg_vdd_vpa);
		if (!IS_ERR(reg))
			reg_unifi->reg_vdd_vpa = reg;
		else {
			err = -EINVAL;
			goto err_reg_vdd_vpa;
		}
	}

	if (plat_data->reg_1v5_dd) {
		reg = regulator_get(NULL, plat_data->reg_1v5_dd);
		if (!IS_ERR(reg))
			reg_unifi->reg_1v5_dd = reg;
		else {
			err = -EINVAL;
			goto err_reg_1v5_dd;
		}
	}
	plat_data->priv = reg_unifi;
	return 0;

err_reg_1v5_dd:
	if (reg_unifi->reg_vdd_vpa)
		regulator_put(reg_unifi->reg_vdd_vpa);
err_reg_vdd_vpa:
	if (reg_unifi->reg_1v5_ana_bb)
		regulator_put(reg_unifi->reg_1v5_ana_bb);
err_reg_1v5_ana_bb:
	if (reg_unifi->reg_gpo2)
		regulator_put(reg_unifi->reg_gpo2);
err_reg_gpo2:
	if (reg_unifi->reg_gpo1)
		regulator_put(reg_unifi->reg_gpo1);
err_reg_gpo1:
	kfree(reg_unifi);
	return err;
}

int fs_unifi_remove(void)
{
	struct regulator_unifi *reg_unifi;

	reg_unifi = plat_data->priv;
	plat_data->priv = NULL;

	if (reg_unifi->reg_1v5_dd)
		regulator_put(reg_unifi->reg_1v5_dd);
	if (reg_unifi->reg_vdd_vpa)
		regulator_put(reg_unifi->reg_vdd_vpa);

	if (reg_unifi->reg_1v5_ana_bb)
		regulator_put(reg_unifi->reg_1v5_ana_bb);

	if (reg_unifi->reg_gpo2)
		regulator_put(reg_unifi->reg_gpo2);

	if (reg_unifi->reg_gpo1)
		regulator_put(reg_unifi->reg_gpo1);

	kfree(reg_unifi);
	return 0;
}

/* Module init and exit, register and unregister to the SDIO/MMC driver */
static int __init fs_sdio_init(void)
{
	int err;

	printk(KERN_INFO "Freescale: Register to MMC/SDIO driver\n");
	/* Sleep a bit - otherwise if the mmc subsystem has just started, it
	 * will allow us to register, then immediatly remove us!
	 */
	msleep(10);
	err = fs_unifi_init();
	if (err) {
		printk(KERN_ERR "Error: fs_unifi_init failed!\n");
		return err;
	}
	err = sdio_register_driver(&sdio_unifi_driver);
	if (err) {
		printk(KERN_ERR "Error: register sdio_unifi_driver failed!\n");
		fs_unifi_remove();
	}
	return err;
}

module_init(fs_sdio_init);

static void __exit fs_sdio_exit(void)
{
	printk(KERN_INFO "Freescale: Unregister from MMC/SDIO driver\n");
	sdio_unregister_driver(&sdio_unifi_driver);
	fs_unifi_remove();
}

module_exit(fs_sdio_exit);

MODULE_DESCRIPTION("Freescale SDIO glue driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
