/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include <asm/uaccess.h>

#include "pmic.h"

#define MC13892_GENERATION_ID_LSH	6
#define MC13892_IC_ID_LSH		13

#define MC13892_GENERATION_ID_WID	3
#define MC13892_IC_ID_WID		6

#define MC13892_GEN_ID_VALUE	0x7
#define MC13892_IC_ID_VALUE		1

/*
 * Global variables
 */
static pmic_version_t mxc_pmic_version;
unsigned int active_events[MAX_ACTIVE_EVENTS];
struct i2c_client *mc13892_client;

static struct workqueue_struct *pmic_event_wq;

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

static void pmic_pdev_register(void)
{
	platform_device_register(&adc_ldm);
	platform_device_register(&battery_ldm);
	platform_device_register(&rtc_ldm);
	platform_device_register(&power_ldm);
	platform_device_register(&light_ldm);
	platform_device_register(&rleds_ldm);
	platform_device_register(&gleds_ldm);
	platform_device_register(&bleds_ldm);
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
}

void pmic_bh_handler(struct work_struct *work);

/*!
 * Bottom half handler of PMIC event handling.
 */
DECLARE_WORK(pmic_ws, pmic_bh_handler);

/*!
 * This function is called when pmic interrupt occurs on the processor.
 * It is the interrupt handler for the pmic module.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 *
 * @return       The function returns IRQ_HANDLED when handled.
 */
static irqreturn_t pmic_irq_handler(int irq, void *dev_id)
{
	/* prepare a task */
	queue_work(pmic_event_wq, &pmic_ws);

	return IRQ_HANDLED;
}

/*!
 * This function is the bottom half handler of the PMIC interrupt.
 * It checks for active events and launches callback for the
 * active events.
 */
void pmic_bh_handler(struct work_struct *work)
{
	unsigned int loop;
	unsigned int count = 0;

	count = pmic_get_active_events(active_events);
	pr_debug("active events number %d\n", count);

	for (loop = 0; loop < count; loop++)
		pmic_event_callback(active_events[loop]);

	return;
}

pmic_version_t pmic_get_version(void)
{
	return mxc_pmic_version;
}

EXPORT_SYMBOL(pmic_get_version);

static int __devinit is_chip_onboard(struct i2c_client *client)
{
	unsigned int ret = 0;

	/*bind the right device to the driver */
	if (pmic_i2c_24bit_read(client, REG_IDENTIFICATION, &ret) == -1)
		return -1;

	if (MC13892_GEN_ID_VALUE != BITFEXT(ret, MC13892_GENERATION_ID)) {
		/*compare the address value */
		dev_err(&client->dev,
			"read generation ID 0x%x is not equal to 0x%x!\n",
			BITFEXT(ret, MC13892_GENERATION_ID),
			MC13892_GEN_ID_VALUE);
		return -1;
	}

	return 0;
}

static ssize_t mc13892_show(struct device *dev,
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

static ssize_t mc13892_store(struct device *dev,
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

static struct device_attribute mc13892_dev_attr = {
	.attr = {
		 .name = "mc13892_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = mc13892_show,
	.store = mc13892_store,
};

static int __devinit pmic_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	int pmic_irq;

	ret = is_chip_onboard(client);

	if (ret == -1)
		return -ENODEV;

	/* so far, we got matched chip on board */

	mc13892_client = client;

	/* Initialize the PMIC event handling */
	pmic_event_list_init();

	/* Initialize GPIO for PMIC Interrupt */
	gpio_pmic_active();

	/* Initialize the PMIC parameters */
	ret = pmic_init_registers();
	if (ret != PMIC_SUCCESS)
		return PMIC_ERROR;

	/* Set and install PMIC IRQ handler */
	pmic_irq = (int)(client->dev.platform_data);
	if (pmic_irq == 0)
		return PMIC_ERROR;

	set_irq_type(IOMUX_TO_IRQ(pmic_irq), IRQF_TRIGGER_RISING);
	ret =
	    request_irq(IOMUX_TO_IRQ(pmic_irq), pmic_irq_handler, 0, "PMIC_IRQ",
			0);

	if (ret) {
		dev_err(&client->dev, "request irq %d error!\n", pmic_irq);
		return ret;
	}
	enable_irq_wake(IOMUX_TO_IRQ(pmic_irq));

	reg_mc13892_probe();

	ret = device_create_file(&client->dev, &mc13892_dev_attr);
	if (ret)
		dev_err(&client->dev, "create device file failed!\n");

	pmic_pdev_register();

	dev_info(&client->dev, "Loaded\n");

	return PMIC_SUCCESS;
}

static int pmic_remove(struct i2c_client *client)
{
	int pmic_irq = (int)(client->dev.platform_data);

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

static const struct i2c_device_id mc13892_id[] = {
	{"mc13892", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mc13892_id);

static struct i2c_driver pmic_driver = {
	.driver = {
		   .name = "mc13892",
		   .bus = NULL,
		   },
	.probe = pmic_probe,
	.remove = pmic_remove,
	.suspend = pmic_suspend,
	.resume = pmic_resume,
	.id_table = mc13892_id,
};

static int __init pmic_init(void)
{
	pmic_event_wq = create_workqueue(pmic_driver.driver.name);
	if (!pmic_event_wq) {
		pr_err("mc13892 pmic driver init: fail to create work queue");
		return -EFAULT;
	}
	return i2c_add_driver(&pmic_driver);
}

static void __exit pmic_exit(void)
{
	if (pmic_event_wq)
		destroy_workqueue(pmic_event_wq);

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
