/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file pmic/core/mc34704.c
 * @brief This file contains MC34704 specific PMIC code.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/mfd/mc34704/core.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include "pmic.h"

/*
 * Globals
 */
static pmic_version_t mxc_pmic_version = {
	.id = PMIC_MC34704,
	.revision = 0,
};
static unsigned int events_enabled;
unsigned int active_events[MAX_ACTIVE_EVENTS];
struct i2c_client *mc34704_client;
static void pmic_trigger_poll(void);

#define MAX_MC34704_REG 0x59
static unsigned int mc34704_reg_readonly[MAX_MC34704_REG / 32 + 1] = {
	(1 << 0x03) || (1 << 0x05) || (1 << 0x07) || (1 << 0x09) ||
	    (1 << 0x0B) || (1 << 0x0E) || (1 << 0x11) || (1 << 0x14) ||
	    (1 << 0x17) || (1 << 0x18),
	0,
};
static unsigned int mc34704_reg_written[MAX_MC34704_REG / 32 + 1];
static unsigned char mc34704_shadow_regs[MAX_MC34704_REG - 1];
#define IS_READONLY(r) ((1 << ((r) % 32)) & mc34704_reg_readonly[(r) / 32])
#define WAS_WRITTEN(r) ((1 << ((r) % 32)) & mc34704_reg_written[(r) / 32])
#define MARK_WRITTEN(r) do { \
	mc34704_reg_written[(r) / 32] |= (1 << ((r) % 32)); \
} while (0)

int pmic_read(int reg_nr, unsigned int *reg_val)
{
	int c;

	/*
	 * Use the shadow register if we've written to it
	 */
	if (WAS_WRITTEN(reg_nr)) {
		*reg_val = mc34704_shadow_regs[reg_nr];
		return PMIC_SUCCESS;
	}

	/*
	 * Otherwise, actually read the real register.
	 * Write-only registers will read as zero.
	 */
	c = i2c_smbus_read_byte_data(mc34704_client, reg_nr);
	if (c == -1) {
		pr_debug("mc34704: error reading register 0x%02x\n", reg_nr);
		return PMIC_ERROR;
	} else {
		*reg_val = c;
		return PMIC_SUCCESS;
	}
}

int pmic_write(int reg_nr, const unsigned int reg_val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(mc34704_client, reg_nr, reg_val);
	if (ret == -1) {
		return PMIC_ERROR;
	} else {
		/*
		 * Update our software copy of the register since you
		 * can't read what you wrote.
		 */
		if (!IS_READONLY(reg_nr)) {
			mc34704_shadow_regs[reg_nr] = reg_val;
			MARK_WRITTEN(reg_nr);
		}
		return PMIC_SUCCESS;
	}
}

unsigned int pmic_get_active_events(unsigned int *active_events)
{
	unsigned int count = 0;
	unsigned int faults;
	int bit_set;

	/* Check for any relevant PMIC faults */
	pmic_read(REG_MC34704_FAULTS, &faults);
	faults &= events_enabled;

	/*
	 * Mask all active events, because there is no way to acknowledge
	 * or dismiss them in the PMIC -- they're sticky.
	 */
	events_enabled &= ~faults;

	/* Account for all unmasked faults */
	while (faults) {
		bit_set = ffs(faults) - 1;
		*(active_events + count) = bit_set;
		count++;
		faults ^= (1 << bit_set);
	}
	return count;
}

int pmic_event_unmask(type_event event)
{
	unsigned int event_bit = 0;
	unsigned int prior_events = events_enabled;

	event_bit = (1 << event);
	events_enabled |= event_bit;

	pr_debug("Enable Event : %d\n", event);

	/* start the polling task as needed */
	if (events_enabled && prior_events == 0)
		pmic_trigger_poll();

	return 0;
}

int pmic_event_mask(type_event event)
{
	unsigned int event_bit = 0;

	event_bit = (1 << event);
	events_enabled &= ~event_bit;

	pr_debug("Disable Event : %d\n", event);

	return 0;
}

/*!
 * PMIC event polling task.  This task is called periodically to poll
 * for possible MC34704 events (No interrupt supplied by the hardware).
 */
static void pmic_event_task(struct work_struct *work);
DECLARE_DELAYED_WORK(pmic_ws, pmic_event_task);

static void pmic_trigger_poll(void)
{
	schedule_delayed_work(&pmic_ws, HZ / 10);
}

static void pmic_event_task(struct work_struct *work)
{
	unsigned int count = 0;
	int i;

	count = pmic_get_active_events(active_events);
	pr_debug("active events number %d\n", count);

	/* call handlers for all active events */
	for (i = 0; i < count; i++)
		pmic_event_callback(active_events[i]);

	/* re-trigger this task, but only if somebody is watching */
	if (events_enabled)
		pmic_trigger_poll();

	return;
}

pmic_version_t pmic_get_version(void)
{
	return mxc_pmic_version;
}
EXPORT_SYMBOL(pmic_get_version);

int __devinit pmic_init_registers(void)
{
	/*
	 * Set some registers to what they should be,
	 * if for no other reason than to initialize our
	 * software register copies.
	 */
	CHECK_ERROR(pmic_write(REG_MC34704_GENERAL2, 0x09));
	CHECK_ERROR(pmic_write(REG_MC34704_VGSET1, 0));
	CHECK_ERROR(pmic_write(REG_MC34704_REG2SET1, 0));
	CHECK_ERROR(pmic_write(REG_MC34704_REG3SET1, 0));
	CHECK_ERROR(pmic_write(REG_MC34704_REG4SET1, 0));
	CHECK_ERROR(pmic_write(REG_MC34704_REG5SET1, 0));

	return PMIC_SUCCESS;
}

static int __devinit is_chip_onboard(struct i2c_client *client)
{
	int val;

	/*
	 * This PMIC has no version or ID register, so just see
	 * if it ACK's and returns 0 on some write-only register as
	 * evidence of its presence.
	 */
	val = i2c_smbus_read_byte_data(client, REG_MC34704_GENERAL2);
	if (val != 0)
		return -1;

	return 0;
}

static int __devinit pmic_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct mc34704 *mc34704;
	struct mc34704_platform_data *plat_data = client->dev.platform_data;

	if (!plat_data || !plat_data->init)
		return -ENODEV;

	ret = is_chip_onboard(client);

	if (ret == -1)
		return -ENODEV;

	mc34704 = kzalloc(sizeof(struct mc34704), GFP_KERNEL);
	if (mc34704 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, mc34704);
	mc34704->dev = &client->dev;
	mc34704->i2c_client = client;

	mc34704_client = client;

	/* Initialize the PMIC event handling */
	pmic_event_list_init();

	/* Initialize PMI registers */
	if (pmic_init_registers() != PMIC_SUCCESS)
		return PMIC_ERROR;

	ret = plat_data->init(mc34704);
	if (ret != 0)
		return PMIC_ERROR;

	dev_info(&client->dev, "Loaded\n");

	return PMIC_SUCCESS;
}

static int pmic_remove(struct i2c_client *client)
{
	return 0;
}

static int pmic_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int pmic_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id mc34704_id[] = {
	{"mc34704", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mc34704_id);

static struct i2c_driver pmic_driver = {
	.driver = {
		   .name = "mc34704",
		   .bus = NULL,
		   },
	.probe = pmic_probe,
	.remove = pmic_remove,
	.suspend = pmic_suspend,
	.resume = pmic_resume,
	.id_table = mc34704_id,
};

static int __init pmic_init(void)
{
	return i2c_add_driver(&pmic_driver);
}

static void __exit pmic_exit(void)
{
	i2c_del_driver(&pmic_driver);
}

/*
 * Module entry points
 */
subsys_initcall_sync(pmic_init);
module_exit(pmic_exit);

MODULE_DESCRIPTION("MC34704 PMIC driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
