// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2011 bct electronic GmbH
 * Copyright 2013 Qtechnology/AS
 * Copyright 2022 NXP
 *
 *
 * LED driver for PCA995x I2C LED drivers
 *
 * Supported devices:
 *
 * PCA9955BTW
 * PCA9952TW
 * PCA9955TW
 *
 * Note that hardware blinking violates the leds infrastructure driver
 * interface since the hardware only supports blinking all LEDs with the
 * same delay_on/delay_off rates.  That is, only the LEDs that are set to
 * blink will actually blink but all LEDs that are set to blink will blink
 * in identical fashion.  The delay_on/delay_off values of the last LED
 * that is set to blink will be used for all of the blinking LEDs.
 * Hardware blinking is disabled by default but can be enabled by setting
 * the 'blink_type' member in the platform_data struct to 'PCA995X_HW_BLINK'
 * or by adding the 'nxp,hw-blink' property to the DTS.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>

#include <dt-bindings/leds/leds-pca995x.h>

 /* LED select registers determine the source that drives LED outputs */
#define PCA995X_LED_OFF		0x0	/* LED driver off */
#define PCA995X_LED_ON		0x1	/* LED driver on */
#define PCA995X_LED_PWM		0x2	/* Controlled through PWM */
#define PCA995X_LED_GRP_PWM	0x3	/* Controlled through PWM/GRPPWM */
#define PCA995X_LDRX_MASK	0x3	/* 2 bits per output state control */


#define PCA995X_MODE_1_CFG 0x00		/* Auto-increment disabled. Normal mode */
#define PCA995X_IREFALL_CFG 0x7F	/* Half of max current gain multiplier*/

#define PCA995X_MODE2_DMBLNK	0x20	/* Enable blinking */
#define PCA995X_MAX_OUTPUTS		16		/* Supported outputs */

#define PCA995X_MODE1	0x00
#define PCA995X_MODE2	0x01

enum pca995x_type {
	pca9955btw,	/* PCA9955BTW */
	pca995xtw,	/* PCA9952TW/PCA9955TW */
};

struct pca995x_chipdef {
	u8			grppwm;
	u8			grpfreq;
	u8			ledout_base;
	u8			irefall;
	u8			pwmbase;
};

static struct pca995x_chipdef pca995x_chipdefs[] = {
	[pca9955btw] = {
		.grppwm = 0x06,
		.grpfreq = 0x07,
		.ledout_base = 0x02,
		.irefall = 0x45,
		.pwmbase = 0x08,
		},
	[pca995xtw] = {
		.grppwm = 0x08,
		.grpfreq = 0x09,
		.ledout_base = 0x02,
		.irefall = 0x43,
		.pwmbase = 0x0A,
		},
};

/* Total blink period in milliseconds. Consider GFRQ = {0,255}  */
#define PCA995X_BLINK_PERIOD_MIN	66
#define PCA995X_BLINK_PERIOD_MAX	16775

static const struct i2c_device_id pca995x_id[] = {
	{ "pca9955btw", pca9955btw },
	{ "pca995xtw", pca995xtw },
	{}
};

MODULE_DEVICE_TABLE(i2c, pca995x_id);

struct pca995x_led;

struct pca995x {
	struct pca995x_chipdef *chipdef;
	struct mutex mutex;
	struct i2c_client *client;
	struct pca995x_led *leds;
	u8	n_leds;
};

struct pca995x_led {
	struct pca995x *chip;
	struct led_classdev led_cdev;
	u8 led_num; /* 0 .. 15 potentially */
	char name[32];
};

static int pca995x_led_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct pca995x_led *pca995x;
	u8 ledout_addr, pwmout_addr;
	u8 ledout, shift, mask;
	int ret;

	pca995x = container_of(led_cdev, struct pca995x_led, led_cdev);
	pwmout_addr = pca995x->chip->chipdef->pwmbase;
	ledout_addr = pca995x->chip->chipdef->ledout_base
		+ (pca995x->led_num / 4);
	shift = 2 * (pca995x->led_num % 4);
	mask = PCA995X_LDRX_MASK << shift;

	mutex_lock(&pca995x->chip->mutex);
	ledout = i2c_smbus_read_byte_data(pca995x->chip->client, ledout_addr);
	switch (value) {
	case LED_FULL:
		ret = i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA995X_LED_ON << shift));
		break;
	case LED_OFF:
		ret = i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			ledout & ~mask);
		break;
	default:
		/*	Adjust brightness as per user input by changing individual pwm */
		ret = i2c_smbus_write_byte_data(pca995x->chip->client,
			pwmout_addr + pca995x->led_num, value);
		if (ret)
			goto out;
		/*	Change LDRx configuration to individual brightness via PWM.
		 *	Led will stop blinking if it's doing so
		 */
		ret = i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA995X_LED_PWM << shift));
		break;
	}
out:
	mutex_unlock(&pca995x->chip->mutex);
	return ret;
}

static int pca995x_blink_set(struct led_classdev *led_cdev,	unsigned long *delay_on,
							unsigned long *delay_off)
{
	struct pca995x_led *pca995x;
	u32 time_on, time_off, period;
	u8 gdc, gfrq, mode2;
	u8 ledout_addr;
	u8 ledout, shift, mask;

	pca995x = container_of(led_cdev, struct pca995x_led, led_cdev);

	shift = 2 * (pca995x->led_num % 4);
	mask = PCA995X_LDRX_MASK << shift;

	ledout_addr = pca995x->chip->chipdef->ledout_base +
		(pca995x->led_num / 4);

	time_on = *delay_on;
	time_off = *delay_off;

	/* If both zero, pick reasonable defaults of 500ms each */
	if (!time_on && !time_off) {
		time_on = 500;
		time_off = 500;
	}

	period = time_on + time_off;

	/* If period not supported by hardware, default to someting sane. */
	if ((period < PCA995X_BLINK_PERIOD_MIN) ||
		(period > PCA995X_BLINK_PERIOD_MAX)) {
		time_on = 500;
		time_off = 500;
		period = time_on + time_off;
	}

	/*
	 * From manual: duty cycle = (GDC / 256) ->
	 *	(time_on / period) = (GDC / 256) ->
	 *		GDC = ((time_on * 256) / period)
	 */
	gdc = (time_on * 256) / period;

	/*
	 * From manual: period = ((GFRQ + 1) / 15.26) in seconds.
	 * So, period (in ms) = (((GFRQ + 1) / 15.26) * 1000) ->
	 *		GFRQ = ((period * 736 / 50000) - 1)
	 */
	gfrq = (period * 736 / 50000) - 1;

	*delay_on = time_on;
	*delay_off = time_off;

	mutex_lock(&pca995x->chip->mutex);
	mode2 = i2c_smbus_read_byte_data(pca995x->chip->client, PCA995X_MODE2);

	i2c_smbus_write_byte_data(pca995x->chip->client,
		pca995x->chip->chipdef->grppwm, gdc);

	i2c_smbus_write_byte_data(pca995x->chip->client,
		pca995x->chip->chipdef->grpfreq, gfrq);

	/* Check if group control is already set to blinking*/
	if (!(mode2 & PCA995X_MODE2_DMBLNK))
		i2c_smbus_write_byte_data(pca995x->chip->client, PCA995X_MODE2,
			mode2 | PCA995X_MODE2_DMBLNK);
	ledout = i2c_smbus_read_byte_data(pca995x->chip->client, ledout_addr);
	/* Check if LDRx is already configured for group blinking*/
	if ((ledout & mask) != (PCA995X_LED_GRP_PWM << shift))
		i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA995X_LED_GRP_PWM << shift));
	mutex_unlock(&pca995x->chip->mutex);

	return 0;
}

static struct pca995x_platform_data*
pca995x_dt_init(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node, *child;
	struct pca995x_platform_data *pdata;
	struct led_info *pca995x_leds;
	int count;

	count = of_get_child_count(np);
	if (!count || count > PCA995X_MAX_OUTPUTS)
		return ERR_PTR(-ENODEV);

	pca995x_leds = devm_kzalloc(&client->dev,
		sizeof(struct led_info) * count, GFP_KERNEL);

	if (!pca995x_leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		struct led_info led = {};
		u32 reg;
		int res;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= PCA995X_MAX_OUTPUTS))
			continue;

		led.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		pca995x_leds[reg] = led;
	}
	pdata = devm_kzalloc(&client->dev,
		sizeof(struct pca995x_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	/* default to software blinking unless hardware blinking is specified */
	if (of_property_read_bool(np, "nxp,hw-blink"))
		pdata->blink_type = PCA995X_HW_BLINK;
	else
		pdata->blink_type = PCA995X_SW_BLINK;

	pdata->leds.leds = pca995x_leds;
	pdata->leds.num_leds = count;

	return pdata;
}

static const struct of_device_id of_pca995x_match[] = {
	{.compatible = "nxp,pca9955btw", .data = (void *) pca9955btw},
	{.compatible = "nxp,pca995xtw",  .data = (void *) pca995xtw},
	{},
};

static int pca995x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pca995x *pca995x_chip;
	struct pca995x_led *pca995x;
	struct pca995x_platform_data *pdata;
	struct pca995x_chipdef *chip;
	struct gpio_desc *rst_pin;
	int i, err;

	chip = &pca995x_chipdefs[id->driver_data];
	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
		pdata = pca995x_dt_init(client);
		if (IS_ERR(pdata)) {
			dev_warn(&client->dev, "could not parse configuration\n");
			pdata = NULL;
		}
	}
	if (pdata && (pdata->leds.num_leds < 1 || pdata->leds.num_leds > PCA995X_MAX_OUTPUTS)) {
		dev_err(&client->dev, "board info must claim 1-%d LEDs",
			PCA995X_MAX_OUTPUTS);
		return -EINVAL;
	}
	pca995x_chip = devm_kzalloc(&client->dev, sizeof(*pca995x_chip),
		GFP_KERNEL);
	if (!pca995x_chip)
		return -ENOMEM;

	pca995x = devm_kzalloc(&client->dev, pdata->leds.num_leds * sizeof(*pca995x),
		GFP_KERNEL);
	if (!pca995x)
		return -ENOMEM;

	rst_pin = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rst_pin))
		dev_warn(&client->dev, "no pin available, reset line not controlled\n");

	i2c_set_clientdata(client, pca995x_chip);

	mutex_init(&pca995x_chip->mutex);
	pca995x_chip->chipdef = chip;
	pca995x_chip->client = client;
	pca995x_chip->leds = pca995x;
	pca995x_chip->n_leds = pdata->leds.num_leds;
	/* Turn off LEDs by default*/
	for (i = 0; i < pdata->leds.num_leds / 4; i++)
		i2c_smbus_write_byte_data(client, chip->ledout_base + i, 0x00);

	for (i = 0; i < pdata->leds.num_leds; i++) {
		pca995x[i].led_num = i;
		pca995x[i].chip = pca995x_chip;

		/* Platform data can specify LED names and default triggers */
		if (pdata && i < pdata->leds.num_leds) {
			if (pdata->leds.leds[i].name)
				snprintf(pca995x[i].name,
					sizeof(pca995x[i].name), "pca995x:%s",
					pdata->leds.leds[i].name);
			if (pdata->leds.leds[i].default_trigger)
				pca995x[i].led_cdev.default_trigger =
				pdata->leds.leds[i].default_trigger;
		}
		if (!pdata || i >= pdata->leds.num_leds ||
			!pdata->leds.leds[i].name) {
			snprintf(pca995x[i].name, sizeof(pca995x[i].name),
				"pca995x:%d:%.2x:%d", client->adapter->nr,
				client->addr, i);
		}

		pca995x[i].led_cdev.name = pca995x[i].name;
		pca995x[i].led_cdev.brightness_set_blocking = pca995x_led_set;

		if (pdata && pdata->blink_type == PCA995X_HW_BLINK)
			pca995x[i].led_cdev.blink_set = pca995x_blink_set;

		err = led_classdev_register(&client->dev, &pca995x[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	/* Disable LED all-call address and set normal mode */
	i2c_smbus_write_byte_data(client, PCA995X_MODE1, PCA995X_MODE_1_CFG);
	/* IREF Output current value for all LEDn outputs */
	i2c_smbus_write_byte_data(client, chip->irefall, PCA995X_IREFALL_CFG);

	return 0;

exit:
	while (i--)
		led_classdev_unregister(&pca995x[i].led_cdev);

	return err;
}

static int pca995x_remove(struct i2c_client *client)
{
	struct pca995x *pca995x = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < pca995x->n_leds; i++)
		led_classdev_unregister(&pca995x->leds[i].led_cdev);
	return 0;
}

static struct i2c_driver pca995x_driver = {
	.driver = {
		.name = "leds-pca995x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_pca995x_match),
	},
	.probe = pca995x_probe,
	.remove = pca995x_remove,
	.id_table = pca995x_id,
};

module_i2c_driver(pca995x_driver);

MODULE_AUTHOR("Isai Gaspar <isaiezequiel.gaspar@nxp.com>");
MODULE_DESCRIPTION("PCA995x LED driver");
MODULE_LICENSE("GPL v2");
