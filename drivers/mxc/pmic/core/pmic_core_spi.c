/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file pmic_core_spi.c
 * @brief This is the main file for the PMIC Core/Protocol driver. SPI
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
#include <linux/spi/spi.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>

#include <asm/uaccess.h>

#include <linux/mfd/mc13892/core.h>
#include <linux/mfd/mc34708/core.h>

#include "pmic.h"

/*
 * Static functions
 */
static void pmic_pdev_register(void);
static void pmic_pdev_unregister(void);

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
	PMIC_ID_MC13892,
	PMIC_ID_MC34708,
	PMIC_ID_INVALID,
};

struct pmic_internal pmic_internal[] = {
	[PMIC_ID_MC13892] = _PMIC_INTERNAL_INITIALIZER(mc13892),
	[PMIC_ID_MC34708] = _PMIC_INTERNAL_INITIALIZER(mc34708),
};

/*
 * External functions
 */
extern void pmic_event_list_init(void);
extern void pmic_event_callback(type_event event);
extern void gpio_pmic_active(void);
extern irqreturn_t pmic_irq_handler(int irq, void *dev_id);
extern pmic_version_t mxc_pmic_version;

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

static const char *get_chipname(struct spi_device *spidev)
{
	const struct spi_device_id *devid =
		spi_get_device_id(spidev);

	if (!devid)
		return NULL;

	return devid->name;
}

/*!
 * This function registers platform device structures for
 * PMIC client drivers.
 */
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

/*!
 * This function puts the SPI slave device in low-power mode/state.
 *
 * @param	spi	the SPI slave device
 * @param	message	the power state to enter
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int pmic_suspend(struct spi_device *spi, pm_message_t message)
{
	return PMIC_SUCCESS;
}

/*!
 * This function brings the SPI slave device back from low-power mode/state.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int pmic_resume(struct spi_device *spi)
{
	return PMIC_SUCCESS;
}

static struct spi_driver pmic_driver;

/*!
 * This function is called whenever the SPI slave device is detected.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int __devinit pmic_probe(struct spi_device *spi)
{
	int ret = 0;
	struct pmic_platform_data *plat_data = spi->dev.platform_data;
	const char *name;
	int pmic_index;

	/* Initialize the PMIC parameters */
	ret = pmic_spi_setup(spi);
	if (ret != PMIC_SUCCESS) {
		return PMIC_ERROR;
	}

	name = get_chipname(spi);
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

	/* Initialize the PMIC event handling */
	pmic_event_list_init();

	/* Initialize GPIO for PMIC Interrupt */
	gpio_pmic_active();

	/* Get the PMIC Version */
	pmic_internal[pmic_index].pmic_get_revision(&mxc_pmic_version);
	if (mxc_pmic_version.revision < 0) {
		dev_err((struct device *)spi,
			"PMIC not detected!!! Access Failed\n");
		return -ENODEV;
	} else {
		dev_dbg((struct device *)spi,
			"Detected pmic core IC version number is %d\n",
			mxc_pmic_version.revision);
	}

	spi_set_drvdata(spi,
			pmic_internal[pmic_index].pmic_alloc_data(&(spi->dev)));

	/* Initialize the PMIC parameters */
	ret = pmic_internal[pmic_index].pmic_init_registers();
	if (ret != PMIC_SUCCESS) {
		kfree(spi_get_drvdata(spi));
		spi_set_drvdata(spi, NULL);
		return PMIC_ERROR;
	}

	ret = pmic_start_event_thread(spi->irq);
	if (ret) {
		pr_err("pmic driver init: \
			fail to start event thread\n");
		kfree(spi_get_drvdata(spi));
		spi_set_drvdata(spi, NULL);
		return PMIC_ERROR;
	}

	/* Set and install PMIC IRQ handler */
	set_irq_type(spi->irq, IRQF_TRIGGER_HIGH);
	ret = request_irq(spi->irq, pmic_irq_handler, 0, "PMIC_IRQ", 0);
	if (ret) {
		kfree(spi_get_drvdata(spi));
		spi_set_drvdata(spi, NULL);
		dev_err((struct device *)spi, "gpio1: irq%d error.", spi->irq);
		return ret;
	}

	enable_irq_wake(spi->irq);

	if (plat_data && plat_data->init) {
		ret = plat_data->init(spi_get_drvdata(spi));
		if (ret != 0) {
			kfree(spi_get_drvdata(spi));
			spi_set_drvdata(spi, NULL);
			return PMIC_ERROR;
		}
	}

	power_ldm.dev.platform_data = spi->dev.platform_data;

	pmic_pdev_register();

	printk(KERN_INFO "Device %s probed\n", dev_name(&spi->dev));

	return PMIC_SUCCESS;
}

/*!
 * This function is called whenever the SPI slave device is removed.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int __devexit pmic_remove(struct spi_device *spi)
{
	pmic_stop_event_thread();
	free_irq(spi->irq, 0);

	pmic_pdev_unregister();

	printk(KERN_INFO "Device %s removed\n", dev_name(&spi->dev));

	return PMIC_SUCCESS;
}

static const struct spi_device_id pmic_device_id[] = {
	{
		.name = "mc13892",
	}, {
		.name = "mc34708",
	}, {
		/* sentinel */
	}
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct spi_driver pmic_driver = {
	.id_table = pmic_device_id,
	.driver = {
		   .name = "pmic_spi",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = pmic_probe,
	.remove = __devexit_p(pmic_remove),
	.suspend = pmic_suspend,
	.resume = pmic_resume,
};

/*
 * Initialization and Exit
 */

/*!
 * This function implements the init function of the PMIC device.
 * This function is called when the module is loaded. It registers
 * the PMIC Protocol driver.
 *
 * @return       This function returns 0.
 */
static int __init pmic_init(void)
{
	return spi_register_driver(&pmic_driver);
}

/*!
 * This function implements the exit function of the PMIC device.
 * This function is called when the module is unloaded. It unregisters
 * the PMIC Protocol driver.
 *
 */
static void __exit pmic_exit(void)
{
	pr_debug("Unregistering the PMIC Protocol Driver\n");
	spi_unregister_driver(&pmic_driver);
}

/*
 * Module entry points
 */
subsys_initcall_sync(pmic_init);
module_exit(pmic_exit);

MODULE_DESCRIPTION("Core/Protocol driver for PMIC");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
