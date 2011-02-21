/*
 * da9052-adc.c  --  ADC Driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/platform_device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/adc.h>

#define DRIVER_NAME "da9052-adc"

static const char *input_names[] = {
	[DA9052_ADC_VDDOUT]	=	"VDDOUT",
	[DA9052_ADC_ICH]	=	"CHARGING CURRENT",
	[DA9052_ADC_TBAT]	=	"BATTERY TEMP",
	[DA9052_ADC_VBAT]	=	"BATTERY VOLTAGE",
	[DA9052_ADC_ADCIN4]	=	"ADC INPUT 4",
	[DA9052_ADC_ADCIN5]	=	"ADC INPUT 5",
	[DA9052_ADC_ADCIN6]	=	"ADC INPUT 6",
	[DA9052_ADC_TSI]	=	"TSI",
	[DA9052_ADC_TJUNC]	=	"BATTERY JUNCTION TEMP",
	[DA9052_ADC_VBBAT]	=	"BACK-UP BATTERY TEMP",
};

struct da9052 *da9052_local;

int da9052_adc_read(unsigned char channel)
{
	if (da9052_local != NULL)
		return da9052_manual_read(da9052_local, channel);
	return -1;
}
EXPORT_SYMBOL(da9052_adc_read);

int da9052_manual_read(struct da9052 *da9052,
			unsigned char channel)
{
	unsigned char man_timeout_cnt = DA9052_ADC_MAX_MANCONV_RETRY_COUNT;
	struct da9052_ssc_msg msg;
	unsigned short calc_data;
	unsigned int ret;
	u16 data = 0;

	msg.addr = DA9052_ADCMAN_REG;
	msg.data = channel;
	msg.data =  (msg.data | DA9052_ADCMAN_MANCONV);

	mutex_lock(&da9052->manconv_lock);
	da9052_lock(da9052);

	ret = da9052->write(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);

	/* Wait for the event */
	do {
		msg.addr = DA9052_ADCCONT_REG;
		msg.data = 0;
		da9052_lock(da9052);
		ret = da9052->read(da9052, &msg);
		if (ret)
			goto err_ssc_comm;
		da9052_unlock(da9052);

		if (DA9052_ADCCONT_ADCMODE & msg.data)
			msleep(1);
		else
			msleep(10);

		msg.addr = DA9052_ADCMAN_REG;
		msg.data = 0;
		da9052_lock(da9052);
		ret = da9052->read(da9052, &msg);
		if (ret)
			goto err_ssc_comm;
		da9052_unlock(da9052);

		/* Counter to avoid endless while loop */
		man_timeout_cnt--;
		if (man_timeout_cnt == 1) {
			if (!(msg.data & DA9052_ADCMAN_MANCONV))
				break;
			else
				goto err_ssc_comm;
		}
	/* Wait until the MAN_CONV bit is cleared to zero */
	} while (msg.data & DA9052_ADCMAN_MANCONV);

	msg.addr = DA9052_ADCRESH_REG;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);

	calc_data = (unsigned short)msg.data;
	data = (calc_data << 2);

	msg.addr = DA9052_ADCRESL_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);

	/* Clear first 14 bits before ORing */
	calc_data = (unsigned short)msg.data & 0x0003;
	data |= calc_data;

	mutex_unlock(&da9052->manconv_lock);

	return data;
err_ssc_comm:
	mutex_unlock(&da9052->manconv_lock);
	da9052_unlock(da9052);
	return -EIO;
}
EXPORT_SYMBOL(da9052_manual_read);

int da9052_read_tjunc(struct da9052 *da9052, char *buf)
{
	struct da9052_ssc_msg msg;
	unsigned char temp;
	int ret;

	msg.addr = DA9052_TJUNCRES_REG;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;

	temp = msg.data;

	msg.addr = DA9052_TOFFSET_REG;
	msg.data = 0;
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);
	/* Calculate Junction temperature */
	temp = (temp - msg.data);
	*buf = temp;
	return 0;
err_ssc_comm:
	da9052_unlock(da9052);
	return -EIO;
}
EXPORT_SYMBOL(da9052_read_tjunc);

int da9052_read_tbat_ich(struct da9052 *da9052, char *data, int channel_no)
{
	struct da9052_ssc_msg msg;
	int ret;

	/* Read TBAT conversion result */
	switch (channel_no) {
	case DA9052_ADC_TBAT:
		msg.addr = DA9052_TBATRES_REG;
	break;
	case DA9052_ADC_ICH:
		msg.addr = DA9052_ICHGAV_REG;
	break;
	default:
		return -EINVAL;
	}
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);
	*data = msg.data;
	printk(KERN_INFO"msg.data 1= %d\n", msg.data);
	msg.data = 28;
	da9052_lock(da9052);
	ret = da9052->write(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);
	printk(KERN_INFO"msg.data2 = %d\n", msg.data);
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;
	da9052_unlock(da9052);
	printk(KERN_INFO"msg.data3 = %d\n", msg.data);
	return 0;

err_ssc_comm:
	da9052_unlock(da9052);
	return ret;
}
EXPORT_SYMBOL(da9052_read_tbat_ich);

static int da9052_start_adc(struct da9052 *da9052, unsigned channel)
{
	struct da9052_ssc_msg msg;
	int ret;

	msg.addr = DA9052_ADCCONT_REG;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;

	if (channel == DA9052_ADC_VDDOUT)
		msg.data = (msg.data | DA9052_ADCCONT_AUTOVDDEN);
	else if (channel == DA9052_ADC_ADCIN4)
		msg.data = (msg.data | DA9052_ADCCONT_AUTOAD4EN);
	else if (channel == DA9052_ADC_ADCIN5)
		msg.data = (msg.data | DA9052_ADCCONT_AUTOAD5EN);
	else if (channel == DA9052_ADC_ADCIN6)
		msg.data = (msg.data | DA9052_ADCCONT_AUTOAD6EN);
	else
		return -EINVAL;

	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);
	return 0;

err_ssc_comm:
	da9052_unlock(da9052);
	return -EIO;
}

static int da9052_stop_adc(struct da9052 *da9052, unsigned channel)
{
	int ret;
	struct da9052_ssc_msg msg;

	msg.addr = DA9052_ADCCONT_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;

	if (channel == DA9052_ADC_VDDOUT)
		msg.data = (msg.data & ~(DA9052_ADCCONT_AUTOVDDEN));
	else if (channel == DA9052_ADC_ADCIN4)
		msg.data = (msg.data & ~(DA9052_ADCCONT_AUTOAD4EN));
	else if (channel == DA9052_ADC_ADCIN5)
		msg.data = (msg.data & ~(DA9052_ADCCONT_AUTOAD5EN));
	else if (channel == DA9052_ADC_ADCIN6)
		msg.data =  (msg.data & ~(DA9052_ADCCONT_AUTOAD6EN));
	else
		return -EINVAL;

	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);

	return 0;
err_ssc_comm:
	da9052_unlock(da9052);
	return -EIO;
}

static ssize_t da9052_adc_read_start_stop(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	struct da9052_ssc_msg msg;
	int channel = to_sensor_dev_attr(devattr)->index;
	int ret;

	ret = da9052_start_adc(priv->da9052, channel);
	if (ret < 0)
		return ret;

	/* Read the ADC converted value */
	switch (channel) {
	case DA9052_ADC_VDDOUT:
		msg.addr = DA9052_VDDRES_REG;
	break;
#if (DA9052_ADC_CONF_ADC4 == 1)
	case DA9052_ADC_ADCIN4:
		msg.addr = DA9052_ADCIN4RES_REG;
	break;
#endif
#if (DA9052_ADC_CONF_ADC5 == 1)
	case DA9052_ADC_ADCIN5:
		msg.addr = DA9052_ADCIN5RES_REG;
	break;
#endif
#if (DA9052_ADC_CONF_ADC6 == 1)
	case DA9052_ADC_ADCIN6:
		msg.addr = DA9052_ADCIN6RES_REG;
	break;
#endif
	default:
		return -EINVAL;
	}
	msg.data = 0;
	da9052_lock(priv->da9052);
	ret = priv->da9052->read(priv->da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(priv->da9052);

	ret = da9052_stop_adc(priv->da9052, channel);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", msg.data);

err_ssc_comm:
	da9052_unlock(priv->da9052);
	return ret;
}

static ssize_t da9052_adc_read_ich(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	int ret;

	ret = da9052_read_tbat_ich(priv->da9052, buf, DA9052_ADC_ICH);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", *buf);
}

static ssize_t da9052_adc_read_tbat(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	int ret;

	ret = da9052_read_tbat_ich(priv->da9052, buf, DA9052_ADC_TBAT);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", *buf);
}

static ssize_t da9052_adc_read_vbat(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	s32 ret;

	ret = da9052_manual_read(priv->da9052, DA9052_ADC_VBAT);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", ret);
}

static ssize_t da9052_adc_read_tjunc(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	int ret;
	ret = da9052_read_tjunc(priv->da9052, buf);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", *buf);
}

static ssize_t da9052_adc_read_vbbat(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);
	s32 ret;

	ret = da9052_manual_read(priv->da9052, DA9052_ADC_VBBAT);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", ret);
}

static int da9052_adc_hw_init(struct da9052 *da9052)
{
	struct da9052_ssc_msg msg;
	int ret;

	/* ADC channel 4 and 5 are by default enabled */
#if (DA9052_ADC_CONF_ADC4 == 1)
	msg.addr = DA9052_GPIO0001_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;

	msg.data = (msg.data & ~(DA9052_GPIO0001_GPIO0PIN));
	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);
#endif

#if (DA9052_ADC_CONF_ADC5 == 1)
	msg.addr = DA9052_GPIO0001_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;

	msg.data = (msg.data & ~(DA9052_GPIO0001_GPIO0PIN));
	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);
#endif

#if (DA9052_ADC_CONF_ADC6 == 1)
	msg.addr = DA9052_GPIO0203_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto err_ssc_comm;

	msg.data = (msg.data & ~(DA9052_GPIO0203_GPIO2PIN));
	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);
#endif
#if 0
	/* By default configure the Measurement sequence interval to 10ms */
	msg.addr = DA9052_ADCCONT_REG;
	msg.data = 0;
	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;

	/* Set the ADC MODE bit for 10msec sampling timer */
	msg.data = (msg.data & ~(DA9052_ADCCONT_ADCMODE));
	ret = da9052->write(da9052, &msg);
	if (ret != 0)
		goto err_ssc_comm;
	da9052_unlock(da9052);
#endif
	return 0;
err_ssc_comm:
	da9052_unlock(da9052);
	return -EIO;
}

static ssize_t da9052_adc_show_name(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "da9052-adc\n");
}

static ssize_t show_label(struct device *dev,
			  struct device_attribute *devattr, char *buf)
{
	int channel = to_sensor_dev_attr(devattr)->index;

	return sprintf(buf, "%s\n", input_names[channel]);
}
#define DA9052_ADC_CHANNELS(id, name) \
	static SENSOR_DEVICE_ATTR(in##id##_label, S_IRUGO, show_label, \
				  NULL, name)

DA9052_ADC_CHANNELS(0, DA9052_ADC_VDDOUT);
DA9052_ADC_CHANNELS(1, DA9052_ADC_ICH);
DA9052_ADC_CHANNELS(2, DA9052_ADC_TBAT);
DA9052_ADC_CHANNELS(3, DA9052_ADC_VBAT);
#if (DA9052_ADC_CONF_ADC4 == 1)
DA9052_ADC_CHANNELS(4, DA9052_ADC_ADCIN4);
#endif
#if (DA9052_ADC_CONF_ADC5 == 1)
DA9052_ADC_CHANNELS(5, DA9052_ADC_ADCIN5);
#endif
#if (DA9052_ADC_CONF_ADC6 == 1)
DA9052_ADC_CHANNELS(6, DA9052_ADC_ADCIN6);
#endif
DA9052_ADC_CHANNELS(7, DA9052_ADC_TSI);
DA9052_ADC_CHANNELS(8, DA9052_ADC_TJUNC);
DA9052_ADC_CHANNELS(9, DA9052_ADC_VBBAT);


static DEVICE_ATTR(name, S_IRUGO, da9052_adc_show_name, NULL);
static SENSOR_DEVICE_ATTR(read_vddout, S_IRUGO,
				da9052_adc_read_start_stop, NULL,
				DA9052_ADC_VDDOUT);
static SENSOR_DEVICE_ATTR(read_ich, S_IRUGO, da9052_adc_read_ich, NULL,
				DA9052_ADC_ICH);
static SENSOR_DEVICE_ATTR(read_tbat, S_IRUGO, da9052_adc_read_tbat, NULL,
				DA9052_ADC_TBAT);
static SENSOR_DEVICE_ATTR(read_vbat, S_IRUGO, da9052_adc_read_vbat, NULL,
				DA9052_ADC_VBAT);
#if (DA9052_ADC_CONF_ADC4 == 1)
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, da9052_adc_read_start_stop, NULL,
				DA9052_ADC_ADCIN4);
#endif
#if (DA9052_ADC_CONF_ADC5 == 1)
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, da9052_adc_read_start_stop, NULL,
				DA9052_ADC_ADCIN5);
#endif
#if (DA9052_ADC_CONF_ADC6 == 1)
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, da9052_adc_read_start_stop, NULL,
				DA9052_ADC_ADCIN6);
#endif
static SENSOR_DEVICE_ATTR(read_tjunc, S_IRUGO, da9052_adc_read_tjunc, NULL,
				DA9052_ADC_TJUNC);
static SENSOR_DEVICE_ATTR(read_vbbat, S_IRUGO, da9052_adc_read_vbbat, NULL,
				DA9052_ADC_VBBAT);

static struct attribute *da9052_attr[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_read_vddout.dev_attr.attr,
	&sensor_dev_attr_in0_label.dev_attr.attr,
	&sensor_dev_attr_read_ich.dev_attr.attr,
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_read_tbat.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_read_vbat.dev_attr.attr,
	&sensor_dev_attr_in3_label.dev_attr.attr,
#if (DA9052_ADC_CONF_ADC4 == 1)
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
#endif
#if (DA9052_ADC_CONF_ADC5 == 1)
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in5_label.dev_attr.attr,
#endif
#if (DA9052_ADC_CONF_ADC6 == 1)
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in6_label.dev_attr.attr,
#endif
	&sensor_dev_attr_in7_label.dev_attr.attr,
	&sensor_dev_attr_read_tjunc.dev_attr.attr,
	&sensor_dev_attr_in8_label.dev_attr.attr,
	&sensor_dev_attr_read_vbbat.dev_attr.attr,
	&sensor_dev_attr_in9_label.dev_attr.attr,
	NULL
};

static const struct attribute_group da9052_group = {
	.attrs = da9052_attr,
};

static int __init da9052_adc_probe(struct platform_device *pdev)
{
	struct da9052_adc_priv *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->da9052 = dev_get_drvdata(pdev->dev.parent);

	platform_set_drvdata(pdev, priv);

	/* Register sysfs hooks */
	ret = sysfs_create_group(&pdev->dev.kobj, &da9052_group);
	if (ret)
		goto out_err_create1;

	priv->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(priv->hwmon_dev)) {
		ret = PTR_ERR(priv->hwmon_dev);
		goto out_err_create2;
	}
	/* Initializes the hardware for ADC module */
	da9052_adc_hw_init(priv->da9052);

	/* Initialize mutex required for ADC Manual read */
	mutex_init(&priv->da9052->manconv_lock);

	da9052_local = priv->da9052;
	return 0;

out_err_create2:
	sysfs_remove_group(&pdev->dev.kobj, &da9052_group);
out_err_create1:
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return ret;
}

static int __devexit da9052_adc_remove(struct platform_device *pdev)
{
	struct da9052_adc_priv *priv = platform_get_drvdata(pdev);

	mutex_destroy(&priv->da9052->manconv_lock);

	hwmon_device_unregister(priv->hwmon_dev);

	sysfs_remove_group(&pdev->dev.kobj, &da9052_group);

	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver da9052_adc_driver = {
	.remove		= __devexit_p(da9052_adc_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
};

static int __init da9052_adc_init(void)
{
	return platform_driver_probe(&da9052_adc_driver, da9052_adc_probe);
}
module_init(da9052_adc_init);

static void __exit da9052_adc_exit(void)
{
	platform_driver_unregister(&da9052_adc_driver);
}
module_exit(da9052_adc_exit);

MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>")
MODULE_DESCRIPTION("DA9052 ADC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
