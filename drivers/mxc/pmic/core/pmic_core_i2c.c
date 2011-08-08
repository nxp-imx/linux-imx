/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*!
 * @file pmic_core_i2c.c
 * @brief This is the main file for the PMIC Core/Protocol driver. i2c
 * should be providing the interface between the PMIC and the MCU.
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
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/mfd/mc13892/core.h>
#include <linux/mfd/mc34708/core.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>

#include "pmic.h"

#define MC13892_GENERATION_ID_LSH	6
#define MC13892_IC_ID_LSH		    13

#define MC13892_GENERATION_ID_WID	3
#define MC13892_IC_ID_WID		    6

#define MC13892_GEN_ID_VALUE	    0x7
#define MC13892_IC_ID_VALUE		    1

/*
 * Global variables
 */
extern pmic_version_t mxc_pmic_version;
extern irqreturn_t pmic_irq_handler(int irq, void *dev_id);
/*
 * Platform device structure for PMIC client drivers
 */
static struct platform_device adc_ldm = {
	.name = "pmic_adc",
	.id = 1,
};
static struct platform_device battery_ldm = {
	.name = "pmic_battery",
	.id = 1,
};
static struct platform_device power_ldm = {
	.name = "pmic_power",
	.id = 1,
};
static struct platform_device rtc_ldm = {
	.name = "pmic_rtc",
	.id = 1,
};
static struct platform_device light_ldm = {
	.name = "pmic_light",
	.id = 1,
};
static struct platform_device rleds_ldm = {
	.name = "pmic_leds",
	.id = 'r',
};
static struct platform_device gleds_ldm = {
	.name = "pmic_leds",
	.id = 'g',
};
static struct platform_device bleds_ldm = {
	.name = "pmic_leds",
	.id = 'b',
};
static struct platform_device pwm1_ldm = {
	.name = "pmic_pwm",
	.id = 0,
};
static struct platform_device pwm2_ldm = {
	.name = "pmic_pwm",
	.id = 1,
};

enum pmic_id {
#if defined(CONFIG_MXC_PMIC_MC13892) || defined(CONFIG_MXC_PMIC_MC13892_MODULE)
	PMIC_ID_MC13892,
#endif
#if defined(CONFIG_MXC_PMIC_MC34708) || defined(CONFIG_MXC_PMIC_MC34708_MODULE)
	PMIC_ID_MC34708,
#endif
	PMIC_ID_INVALID,
};

static struct pmic_internal pmic_internal[] = {
#if defined(CONFIG_MXC_PMIC_MC13892) || defined(CONFIG_MXC_PMIC_MC13892_MODULE)
	[PMIC_ID_MC13892] = _PMIC_INTERNAL_INITIALIZER(mc13892),
#endif
#if defined(CONFIG_MXC_PMIC_MC34708) || defined(CONFIG_MXC_PMIC_MC34708_MODULE)
	[PMIC_ID_MC34708] = _PMIC_INTERNAL_INITIALIZER(mc34708),
#endif
};

static int get_index_pmic_internal(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pmic_internal); i++)
		if (!strcmp(name, pmic_internal[i].name))
			return i;

	return PMIC_ID_INVALID;
}

static const char *get_client_device_name(const char *name, const char *format)
{
	char buf[30];
	const char *client_devname;

	if (snprintf(buf, sizeof(buf), format, name) > sizeof(buf))
		return NULL;

	client_devname = kmemdup(buf, strlen(buf) + 1, GFP_KERNEL);
	if (!client_devname)
		return NULL;

	return client_devname;
}

static const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}

static const struct i2c_device_id *i2c_get_device_id(
		const struct i2c_client *idev)
{
	const struct i2c_driver *idrv = to_i2c_driver(idev->dev.driver);

	return i2c_match_id(idrv->id_table, idev);
}

static const char *get_chipname(struct i2c_client *idev)
{
	const struct i2c_device_id *devid =
		i2c_get_device_id(idev);

	if (!devid)
		return NULL;

	return devid->name;
}

static void pmic_pdev_register(struct device *dev)
{
	platform_device_register(&adc_ldm);

	if (!cpu_is_mx53())
		platform_device_register(&battery_ldm);

	platform_device_register(&rtc_ldm);
	platform_device_register(&power_ldm);
	platform_device_register(&light_ldm);
	platform_device_register(&rleds_ldm);
	platform_device_register(&gleds_ldm);
	platform_device_register(&bleds_ldm);
	platform_device_register(&pwm1_ldm);
	platform_device_register(&pwm2_ldm);
}

/*!
 * This function unregisters platform device structures for
 * PMIC client drivers.
 */
static void pmic_pdev_unregister(void)
{
	platform_device_unregister(&adc_ldm);
	platform_device_unregister(&battery_ldm);
	platform_device_unregister(&rtc_ldm);
	platform_device_unregister(&power_ldm);
	platform_device_unregister(&light_ldm);
	platform_device_unregister(&pwm1_ldm);
	platform_device_unregister(&pwm2_ldm);
}

static int __devinit is_chip_onboard(struct i2c_client *client)
{
	unsigned int ret = 0;

	/*bind the right device to the driver */
	if (pmic_i2c_24bit_read(client, REG_IDENTIFICATION, &ret) == -1)
		return -1;

	return 0;
}

static ssize_t pmic_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int i, value;
	int offset = (REG_TEST4 + 1) / 4;

	for (i = 0; i < offset; i++) {
		pmic_read(i, &value);
		pr_info("reg%02d: %06x\t\t", i, value);
		pmic_read(i + offset, &value);
		pr_info("reg%02d: %06x\t\t", i + offset, value);
		pmic_read(i + offset * 2, &value);
		pr_info("reg%02d: %06x\t\t", i + offset * 2, value);
		pmic_read(i + offset * 3, &value);
		pr_info("reg%02d: %06x\n", i + offset * 3, value);
	}

	return 0;
}

static ssize_t pmic_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	int reg, value, ret;
	char *p;

	reg = simple_strtoul(buf, NULL, 10);

	p = NULL;
	p = memchr(buf, ' ', count);

	if (p == NULL) {
		pmic_read(reg, &value);
		pr_debug("reg%02d: %06x\n", reg, value);
		return count;
	}

	p += 1;

	value = simple_strtoul(p, NULL, 16);

	ret = pmic_write(reg, value);
	if (ret == 0)
		pr_debug("write reg%02d: %06x\n", reg, value);
	else
		pr_debug("register update failed\n");

	return count;
}

static struct device_attribute pmic_dev_attr = {
	.attr = {
		 .name = "pmic_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = pmic_show,
	.store = pmic_store,
};

static int __devinit pmic_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	int pmic_irq;
	struct pmic_platform_data *plat_data = client->dev.platform_data;
	const char *name;
	int pmic_index;

	ret = is_chip_onboard(client);
	if (ret == -1)
		return -ENODEV;

	name = get_chipname(client);
	if (!name)
		return PMIC_ERROR;
	pmic_index = get_index_pmic_internal(name);
	if (pmic_index == PMIC_ID_INVALID)
		return PMIC_ERROR;

	adc_ldm.name = get_client_device_name(name, "%s_adc");
	battery_ldm.name = get_client_device_name(name, "%s_battery");
	light_ldm.name = get_client_device_name(name, "%s_light");
	rtc_ldm.name = get_client_device_name(name, "%s_rtc");
	pwm1_ldm.name = get_client_device_name(name, "%s_pwm");
	pwm2_ldm.name = get_client_device_name(name, "%s_pwm");

	i2c_set_clientdata(client,
		pmic_internal[pmic_index].pmic_alloc_data(&client->dev));

	/* so far, we got matched chip on board */

	pmic_i2c_setup(client);

	/* Initialize the PMIC event handling */
	pmic_event_list_init();

	/* Initialize GPIO for PMIC Interrupt */
	gpio_pmic_active();

	/* Get the PMIC Version */
	pmic_internal[pmic_index].pmic_get_revision(&mxc_pmic_version);
	if (mxc_pmic_version.revision < 0) {
		dev_err((struct device *)client,
			"PMIC not detected!!! Access Failed\n");
		return -ENODEV;
	} else {
		dev_dbg((struct device *)client,
			"Detected pmic core IC version number is %d\n",
			mxc_pmic_version.revision);
	}

	/* Initialize the PMIC parameters */
	ret = pmic_internal[pmic_index].pmic_init_registers();
	if (ret != PMIC_SUCCESS)
		return PMIC_ERROR;

	pmic_irq = (int)(client->irq);
	if (pmic_irq == 0)
		return PMIC_ERROR;

	ret = pmic_start_event_thread(pmic_irq);
	if (ret) {
		pr_err("pmic driver init: \
			fail to start event thread\n");
		return PMIC_ERROR;
	}

	/* Set and install PMIC IRQ handler */

	set_irq_type(pmic_irq, IRQF_TRIGGER_HIGH);

	ret =
	    request_irq(pmic_irq, pmic_irq_handler, 0, "PMIC_IRQ",
			0);

	if (ret) {
		dev_err(&client->dev, "request irq %d error!\n", pmic_irq);
		return ret;
	}
	enable_irq_wake(pmic_irq);

	if (plat_data && plat_data->init) {
		ret = plat_data->init(i2c_get_clientdata(client));
		if (ret != 0)
			return PMIC_ERROR;
	}

	ret = device_create_file(&client->dev, &pmic_dev_attr);
	if (ret)
		dev_err(&client->dev, "create device file failed!\n");

	pmic_pdev_register(&client->dev);

	dev_info(&client->dev, "Loaded\n");

	return PMIC_SUCCESS;
}

static int pmic_remove(struct i2c_client *client)
{
	int pmic_irq = (int)(client->irq);

	pmic_stop_event_thread();
	free_irq(pmic_irq, 0);
	pmic_pdev_unregister();
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

static const struct i2c_device_id pmic_id[] = {
	{"mc13892", 0},
	{"mc34708", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, pmic_id);

static struct i2c_driver pmic_driver = {
	.driver = {
		   .name = "pmic",
		   .bus = NULL,
		   },
	.probe = pmic_probe,
	.remove = pmic_remove,
	.suspend = pmic_suspend,
	.resume = pmic_resume,
	.id_table = pmic_id,
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

MODULE_DESCRIPTION("Core/Protocol driver for PMIC");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
