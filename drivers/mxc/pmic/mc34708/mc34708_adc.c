/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/device.h>

#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc34708/mc34708.h>

#include "../core/pmic.h"

#define ADSEL0_LSH	0
#define ADSEL0_WID	4
#define ADSEL1_LSH	4
#define ADSEL1_WID	4
#define ADSEL2_LSH	8
#define ADSEL2_WID	4
#define ADSEL3_LSH	12
#define ADSEL3_WID	4
#define ADSEL4_LSH	16
#define ADSEL4_WID	4
#define ADSEL5_LSH	20
#define ADSEL5_WID	4

#define ADSEL6_LSH	0
#define ADSEL6_WID	4
#define ADSEL7_LSH	4
#define ADSEL7_WID	4

#define ADRESULT0_LSH 2
#define ADRESULT0_WID 10

#define ADRESULT1_LSH 14
#define ADRESULT1_WID 10

#define ADEN_LSH 0
#define ADEN_WID 1

#define ADSTART_LSH 1
#define ADSTART_WID 1

#define ADSTOP_LSH 4
#define ADSTOP_WID 3

#define TSEN	(1 << 12)
#define TSSTART	(1 << 13)
#define	TSCONT	(1 << 14)
#define	TSHOLD	(1 << 15)
#define	TSSTOP0	(1 << 16)
#define	TSSTOP1	(1 << 17)
#define	TSSTOP2	(1 << 18)
#define	TSPENDETEN	(1 << 20)

#define	DUMMY		0
#define	X_POS		1
#define	Y_POS		2
#define	CONTACT_RES	3

#define	DELTA_Y_MAX	50
#define	DELTA_X_MAX	50

#define ADC_MAX_CHANNEL 7

/* internal function */
static void callback_tspendet(void *);
static void callback_tsdone(void *);
static void callback_adcbisdone(void *);

static int suspend_flag;

static DECLARE_COMPLETION(adcdone_it);
static DECLARE_COMPLETION(tsdone_int);
static DECLARE_COMPLETION(tspendet_int);
static pmic_event_callback_t tspendet_event;
static pmic_event_callback_t event_tsdone;
static pmic_event_callback_t event_adc;
static DECLARE_MUTEX(convert_mutex);

u32 value[8];

static bool pmic_adc_ready;

int is_mc34708_adc_ready()
{
	return pmic_adc_ready;
}

EXPORT_SYMBOL(is_mc34708_adc_ready);

static int mc34708_pmic_adc_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	suspend_flag = 1;
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_ADC0, BITFVAL(ADEN, 0), BITFMASK(ADEN)));

	return 0;
};

static int mc34708_pmic_adc_resume(struct platform_device *pdev)
{
	suspend_flag = 0;
	CHECK_ERROR(pmic_write_reg
		    (MC34708_REG_ADC0, BITFVAL(ADEN, 1), BITFMASK(ADEN)));

	return 0;
};

static void callback_tspendet(void *unused)
{
	pr_debug("*** TSI IT mc34708 PMIC_ADC_GET_TOUCH_SAMPLE ***\n");
	complete(&tspendet_int);
	pmic_event_mask(MC34708_EVENT_TSPENDET);
}

static void callback_tsdone(void *unused)
{
	complete(&tsdone_int);
}

static void callback_adcdone(void *unused)
{
	complete(&adcdone_it);
}

int mc34708_pmic_adc_init(void)
{
	unsigned int reg_value = 0, i = 0;

	if (suspend_flag == 1)
		return -EBUSY;

	/* sub to ADCDone IT */
	event_adc.param = NULL;
	event_adc.func = callback_adcdone;
	CHECK_ERROR(pmic_event_subscribe(MC34708_EVENT_ADCDONEI, event_adc));

	/* sub to TSDONE INT */
	event_tsdone.param = NULL;
	event_tsdone.func = callback_tsdone;
	CHECK_ERROR(pmic_event_subscribe(MC34708_EVENT_TSDONEI, event_tsdone));

	/* sub to TS Pen Detect INT */
	tspendet_event.param = NULL;
	tspendet_event.func = callback_tspendet;
	CHECK_ERROR(pmic_event_subscribe
		    (MC34708_EVENT_TSPENDET, tspendet_event));

	/* enable adc */
	pmic_write_reg(MC34708_REG_ADC0, 0x170000, PMIC_ALL_BITS);
	pmic_write_reg(MC34708_REG_ADC1, 0xFFF000, PMIC_ALL_BITS);
	pmic_write_reg(MC34708_REG_ADC2, 0x000000, PMIC_ALL_BITS);
	pmic_write_reg(MC34708_REG_ADC3, 0xF28500, PMIC_ALL_BITS);

	return PMIC_SUCCESS;
}

PMIC_STATUS mc34708_pmic_adc_deinit(void)
{
	CHECK_ERROR(pmic_event_unsubscribe
		    (MC34708_EVENT_ADCDONEI, event_tsdone));
	CHECK_ERROR(pmic_event_unsubscribe
		    (MC34708_EVENT_TSPENDET, tspendet_event));

	return PMIC_SUCCESS;
}

PMIC_STATUS mc34708_pmic_adc_convert(t_channel channel, unsigned short *result)
{
	PMIC_STATUS ret;
	unsigned int register_val = 0, register_mask = 0;
	unsigned int register1;

	register1 = MC34708_REG_ADC2;

	if (suspend_flag == 1)
		return -EBUSY;
	down(&convert_mutex);

	if (channel <= BATTERY_CURRENT) {
		INIT_COMPLETION(adcdone_it);
		ret = pmic_write_reg(MC34708_REG_ADC2,
					   BITFVAL(ADSEL0, BATTERY_VOLTAGE),
					   BITFMASK(ADSEL0));
		if (PMIC_SUCCESS != ret)
			goto error1;

		ret = pmic_write_reg(MC34708_REG_ADC2,
					   BITFVAL(ADSEL1, BATTERY_CURRENT),
					   BITFMASK(ADSEL1));
		if (PMIC_SUCCESS != ret)
			goto error1;

		ret = pmic_write_reg
			    (MC34708_REG_ADC0, BITFVAL(ADEN, 1),
			     BITFMASK(ADEN));

		if (PMIC_SUCCESS != ret)
			goto error1;
		ret = pmic_write_reg(MC34708_REG_ADC0,
				     BITFVAL(ADSTOP, ADC_MAX_CHANNEL),
				     BITFMASK(ADSTOP));
		if (PMIC_SUCCESS != ret)
			goto error1;
		ret = pmic_write_reg(MC34708_REG_ADC0,
				     BITFVAL(ADSTART, 1), BITFMASK(ADSTART));
		if (PMIC_SUCCESS != ret)
			goto error1;

		pr_debug("wait adc done.\n");
		msleep(100);
		wait_for_completion_interruptible(&adcdone_it);
		ret = pmic_write_reg(MC34708_REG_ADC0,
				     BITFVAL(ADSTART, 0), BITFMASK(ADSTART));

		ret =
		    pmic_read_reg(MC34708_REG_ADC4, &register_val,
				  PMIC_ALL_BITS);

		if (PMIC_SUCCESS != ret)
			goto error1;

		switch (channel) {
		case BATTERY_VOLTAGE:
			*result = BITFEXT(register_val, ADRESULT0);
			break;
		case BATTERY_CURRENT:
			*result = BITFEXT(register_val, ADRESULT1);
			break;
		default:
			*result = BITFEXT(register_val, ADRESULT0);
			break;
		}

		pmic_write_reg
			    (MC34708_REG_ADC0, BITFVAL(ADEN, 0),
			     BITFMASK(ADEN));
	} else {

		INIT_COMPLETION(tsdone_int);
		pr_debug("wait adc done.\n");
		wait_for_completion_interruptible(&tsdone_int);
		ret =
		    pmic_read_reg(MC34708_REG_ADC4, &register_val,
				  PMIC_ALL_BITS);
		if (PMIC_SUCCESS != ret)
			goto error1;
		*result = BITFEXT(register_val, ADRESULT0);
	}
error1:
	up(&convert_mutex);

	return ret;
}

EXPORT_SYMBOL(mc34708_pmic_adc_convert);

static int pmic_adc_filter(t_touch_screen *ts_curr)
{
	unsigned int ydiff, xdiff;
	unsigned int sample_sumx, sample_sumy;

	if (ts_curr->contact_resistance == 0) {
		ts_curr->x_position = 0;
		ts_curr->y_position = 0;
		return 0;
	}

	ydiff = abs(ts_curr->y_position1 - ts_curr->y_position2);
	if (ydiff > DELTA_Y_MAX) {
		pr_debug("pmic_adc_filter: Ret pos y\n");
		return -1;
	}

	xdiff = abs(ts_curr->x_position1 - ts_curr->x_position2);
	if (xdiff > DELTA_X_MAX) {
		pr_debug("pmic_adc_filter: Ret pos x\n");
		return -1;
	}

	sample_sumx = ts_curr->x_position1 + ts_curr->x_position2;
	sample_sumy = ts_curr->y_position1 + ts_curr->y_position2;

	ts_curr->y_position = sample_sumy / 2;
	ts_curr->x_position = sample_sumx / 2;

	return 0;
}

PMIC_STATUS mc34708_adc_read_ts(t_touch_screen *ts_value, int wait_tsi)
{
	pr_debug("mc34708_adc : mc34708_adc_read_ts\n");

	if (wait_tsi) {
		INIT_COMPLETION(tspendet_int);

		pmic_event_unmask(MC34708_EVENT_TSPENDET);
		pmic_write_reg(MC34708_REG_ADC0, 0x170000, PMIC_ALL_BITS);
		pmic_write_reg(MC34708_REG_ADC1, 0xFFF000, PMIC_ALL_BITS);
		pmic_write_reg(MC34708_REG_ADC2, 0x000000, PMIC_ALL_BITS);
		pmic_write_reg(MC34708_REG_ADC3, 0xF28500, PMIC_ALL_BITS);

		wait_for_completion_interruptible(&tspendet_int);
	}

	INIT_COMPLETION(tsdone_int);

	int adc3 = X_POS << 8 | X_POS << 10 | DUMMY << 12 |
	    Y_POS << 14 | Y_POS << 16 | DUMMY << 18 |
	    CONTACT_RES << 20 | CONTACT_RES << 22;

	pmic_write_reg(MC34708_REG_ADC1, 0xFFF000, PMIC_ALL_BITS);
	pmic_write_reg(MC34708_REG_ADC2, 0x000000, PMIC_ALL_BITS);
	pmic_write_reg(MC34708_REG_ADC3, adc3, PMIC_ALL_BITS);
	/* TSSTOP=0b111, TSCONT=1, TSSTART=1, TSEN=1 */
	pmic_write_reg(MC34708_REG_ADC0, 0x177000, PMIC_ALL_BITS);

	wait_for_completion_interruptible(&tsdone_int);

	int i;
	for (i = 0; i < 4; i++) {
		int reg = MC34708_REG_ADC4 + i;
		int result, ret;
		ret = pmic_read_reg(reg, &result, PMIC_ALL_BITS);
		if (ret != PMIC_SUCCESS)
			pr_err("pmic_write_reg err\n");

		value[i * 2] = (result & (0x3FF << 2)) >> 2;
		value[i * 2 + 1] = (result & (0x3FF << 14)) >> 14;
	}

	if (value[6] < 1000) {
		ts_value->x_position = value[0];
		ts_value->x_position1 = value[0];
		ts_value->x_position2 = value[1];

		ts_value->y_position = value[3];
		ts_value->y_position1 = value[3];
		ts_value->y_position2 = value[4];

		ts_value->contact_resistance = value[6];
	} else {
		ts_value->x_position = 0;
		ts_value->y_position = 0;
		ts_value->contact_resistance = 0;
	}

	return PMIC_SUCCESS;
}

PMIC_STATUS mc34708_adc_get_touch_sample(t_touch_screen *touch_sample, int wait)
{
	if (mc34708_adc_read_ts(touch_sample, wait) != 0)
		return PMIC_ERROR;
	if (0 == pmic_adc_filter(touch_sample))
		return PMIC_SUCCESS;
	else
		return PMIC_ERROR;
}

EXPORT_SYMBOL(mc34708_adc_get_touch_sample);

#ifdef DEBUG
static t_adc_param adc_param_db;

static ssize_t adc_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int *value = adc_param_db.value;

	pr_debug("adc_info\n");

	pr_debug("ch0\t\t%d\n", adc_param_db.channel_0);
	pr_debug("ch1\t\t%d\n", adc_param_db.channel_1);
	pr_debug("d5\t\t%d\n", adc_param_db.chrgraw_devide_5);
	pr_debug("conv delay\t%d\n", adc_param_db.conv_delay);
	pr_debug("delay\t\t%d\n", adc_param_db.delay);
	pr_debug("read mode\t%d\n", adc_param_db.read_mode);
	pr_debug("read ts\t\t%d\n", adc_param_db.read_ts);
	pr_debug("single ch\t%d\n", adc_param_db.single_channel);
	pr_debug("wait ts int\t%d\n", adc_param_db.wait_tsi);
	pr_debug("value0-3:\t%d\t%d\t%d\t%d\n", value[0], value[1],
		 value[2], value[3]);
	pr_debug("value4-7:\t%d\t%d\t%d\t%d\n", value[4], value[5],
		 value[6], value[7]);

	return 0;
}

enum {
	ADC_SET_CH0 = 0,
	ADC_SET_CH1,
	ADC_SET_DV5,
	ADC_SET_CON_DELAY,
	ADC_SET_DELAY,
	ADC_SET_RM,
	ADC_SET_RT,
	ADC_SET_S_CH,
	ADC_SET_WAIT_TS,
	ADC_INIT_P,
	ADC_START,
	ADC_TS,
	ADC_TS_READ,
	ADC_TS_CAL,
	ADC_CMD_MAX
};

static const char *const adc_cmd[ADC_CMD_MAX] = {
	[ADC_SET_CH0] = "ch0",
	[ADC_SET_CH1] = "ch1",
	[ADC_SET_DV5] = "dv5",
	[ADC_SET_CON_DELAY] = "cd",
	[ADC_SET_DELAY] = "dl",
	[ADC_SET_RM] = "rm",
	[ADC_SET_RT] = "rt",
	[ADC_SET_S_CH] = "sch",
	[ADC_SET_WAIT_TS] = "wt",
	[ADC_INIT_P] = "init",
	[ADC_START] = "start",
	[ADC_TS] = "touch",
	[ADC_TS_READ] = "touchr",
	[ADC_TS_CAL] = "cal"
};

static int cmd(unsigned int index, int value)
{
	t_touch_screen ts;

	switch (index) {
	case ADC_SET_CH0:
		adc_param_db.channel_0 = value;
		break;
	case ADC_SET_CH1:
		adc_param_db.channel_1 = value;
		break;
	case ADC_SET_DV5:
		adc_param_db.chrgraw_devide_5 = value;
		break;
	case ADC_SET_CON_DELAY:
		adc_param_db.conv_delay = value;
		break;
	case ADC_SET_RM:
		adc_param_db.read_mode = value;
		break;
	case ADC_SET_RT:
		adc_param_db.read_ts = value;
		break;
	case ADC_SET_S_CH:
		adc_param_db.single_channel = value;
		break;
	case ADC_SET_WAIT_TS:
		adc_param_db.wait_tsi = value;
		break;
	case ADC_TS:
		mc34708_pmic_adc_get_touch_sample(&ts, 1);
		pr_debug("x = %d\n", ts.x_position);
		pr_debug("y = %d\n", ts.y_position);
		pr_debug("p = %d\n", ts.contact_resistance);
		break;
	case ADC_TS_READ:
		mc34708_pmic_adc_get_touch_sample(&ts, 0);
		pr_debug("x = %d\n", ts.x_position);
		pr_debug("y = %d\n", ts.y_position);
		pr_debug("p = %d\n", ts.contact_resistance);
		break;
	case ADC_TS_CAL:
		break;
	default:
		pr_debug("error command\n");
		break;
	}
	return 0;
}

static ssize_t adc_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int state = 0;
	const char *const *s;
	char *p, *q;
	int error;
	int len, value = 0;

	pr_debug("adc_ctl\n");

	q = NULL;
	q = memchr(buf, ' ', count);

	if (q != NULL) {
		len = q - buf;
		q += 1;
		value = simple_strtoul(q, NULL, 10);
	} else {
		p = memchr(buf, '\n', count);
		len = p ? p - buf : count;
	}

	for (s = &adc_cmd[state]; state < ADC_CMD_MAX; s++, state++) {
		if (*s && !strncmp(buf, *s, len))
			break;
	}
	if (state < ADC_CMD_MAX && *s)
		error = cmd(state, value);
	else
		error = -EINVAL;

	return count;
}

#else
static ssize_t adc_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return 0;
}

static ssize_t adc_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	return count;
}

#endif

static DEVICE_ATTR(adc, 0644, adc_info, adc_ctl);

static struct pmic_adc_api pmic_adc_api = {
	.is_pmic_adc_ready = is_mc34708_adc_ready,
	.pmic_adc_get_touch_sample = mc34708_adc_get_touch_sample,
};

static int mc34708_pmic_adc_module_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("PMIC ADC start probe\n");
	ret = device_create_file(&(pdev->dev), &dev_attr_adc);
	if (ret) {
		pr_debug("Can't create device file!\n");
		return -ENODEV;
	}

	ret = mc34708_pmic_adc_init();
	if (ret != PMIC_SUCCESS) {
		pr_info("Error in mc34708_pmic_adc_init.\n");
		goto rm_dev_file;
	}

	pmic_adc_ready = 1;
	register_adc_apis(&pmic_adc_api);
	pr_debug("PMIC ADC successfully probed\n");
	return 0;

rm_dev_file:
	device_remove_file(&(pdev->dev), &dev_attr_adc);
	return ret;
}

static int mc34708_pmic_adc_module_remove(struct platform_device *pdev)
{
	mc34708_pmic_adc_deinit();
	pmic_adc_ready = 0;
	pr_debug("PMIC ADC successfully removed\n");
	return 0;
}

static struct platform_driver mc34708_pmic_adc_driver_ldm = {
	.driver = {
		   .name = "mc34708_adc",
		   },
	.suspend = mc34708_pmic_adc_suspend,
	.resume = mc34708_pmic_adc_resume,
	.probe = mc34708_pmic_adc_module_probe,
	.remove = mc34708_pmic_adc_module_remove,
};

static int __init mc34708_pmic_adc_module_init(void)
{
	pr_info("MC34708 PMIC ADC driver loading...\n");
	return platform_driver_register(&mc34708_pmic_adc_driver_ldm);
}

static void __exit mc34708_pmic_adc_module_exit(void)
{
	platform_driver_unregister(&mc34708_pmic_adc_driver_ldm);
	pr_debug("MC34708 PMIC ADC driver successfully unloaded\n");
}

module_init(mc34708_pmic_adc_module_init);
module_exit(mc34708_pmic_adc_module_exit);

MODULE_DESCRIPTION("MC34708 PMIC ADC device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
