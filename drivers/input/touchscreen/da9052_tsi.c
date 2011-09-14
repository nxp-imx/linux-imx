/*
 * da9052_tsi.c  --  TSI driver for Dialog DA9052
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
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/tsi_cfg.h>
#include <linux/mfd/da9052/tsi.h>
#include <linux/mfd/da9052/gpio.h>
#include <linux/mfd/da9052/adc.h>

#define WAIT_FOR_PEN_DOWN	0
#define WAIT_FOR_SAMPLING	1
#define SAMPLING_ACTIVE		2

static ssize_t __init da9052_tsi_create_input_dev(struct input_dev **ip_dev,
					u8 n);
static ssize_t read_da9052_reg(struct da9052 *da9052, u8 reg_addr);
static ssize_t write_da9052_reg(struct da9052 *da9052, u8 reg_addr, u8 data);

static void da9052_tsi_reg_pendwn_event(struct da9052_ts_priv *priv);
static void da9052_tsi_reg_datardy_event(struct da9052_ts_priv *priv);
static ssize_t da9052_tsi_config_delay(struct da9052_ts_priv *priv,
					enum TSI_DELAY delay);
static ssize_t da9052_tsi_config_measure_seq(struct da9052_ts_priv *priv,
					enum TSI_MEASURE_SEQ seq);
static ssize_t da9052_tsi_config_state(struct da9052_ts_priv *ts,
					enum TSI_STATE state);
static ssize_t da9052_tsi_set_sampling_mode(struct da9052_ts_priv *priv,
					u8 interval);
static ssize_t da9052_tsi_config_skip_slots(struct da9052_ts_priv *priv,
					enum TSI_SLOT_SKIP skip);
static ssize_t da9052_tsi_config_pen_detect(struct da9052_ts_priv *priv,
					u8 flag);
static ssize_t da9052_tsi_disable_irq(struct da9052_ts_priv *priv,
					enum TSI_IRQ tsi_irq);
static ssize_t da9052_tsi_enable_irq(struct da9052_ts_priv *priv,
					enum TSI_IRQ tsi_irq);
static ssize_t da9052_tsi_config_manual_mode(struct da9052_ts_priv *priv,
					u8 coordinate);
static ssize_t da9052_tsi_config_auto_mode(struct da9052_ts_priv *priv,
					u8 state);
static ssize_t da9052_tsi_config_gpio(struct da9052_ts_priv *priv);
static ssize_t da9052_tsi_config_power_supply(struct da9052_ts_priv *priv,
					u8 state);
static struct da9052_tsi_info *get_tsi_drvdata(void);
static void da9052_tsi_penup_event(struct da9052_ts_priv *priv);
static s32 da9052_tsi_get_rawdata(struct da9052_tsi_reg *buf, u8 cnt);
static ssize_t da9052_tsi_reg_proc_thread(void *ptr);
static ssize_t da9052_tsi_resume(struct platform_device *dev);
static ssize_t da9052_tsi_suspend(struct platform_device *dev,
					pm_message_t state);
struct da9052_tsi tsi_reg;
struct da9052_tsi_info gda9052_tsi_info;

static ssize_t write_da9052_reg(struct da9052 *da9052, u8 reg_addr, u8 data)
{
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr =  reg_addr;
	ssc_msg.data =  data;
	ret = da9052->write(da9052, &ssc_msg);
	if (ret) {
		DA9052_DEBUG("%s: ",__FUNCTION__);
		DA9052_DEBUG("da9052_ssc_write Failed %d\n",ret );
	}

	return ret;
}

static ssize_t read_da9052_reg(struct da9052 *da9052, u8 reg_addr)
{
	 ssize_t ret = 0;
	 struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr =  reg_addr;
	ssc_msg.data =  0;
	ret = da9052->read(da9052, &ssc_msg);
	if (ret) {
		DA9052_DEBUG("%s: ",__FUNCTION__);
		DA9052_DEBUG("da9052_ssc_read Failed => %d\n" ,ret);
		return -ret;
	}
	return ssc_msg.data;
}

static struct da9052_tsi_info *get_tsi_drvdata(void)
{
	return &gda9052_tsi_info;
}

static ssize_t da9052_tsi_config_measure_seq(struct da9052_ts_priv *priv,
						enum TSI_MEASURE_SEQ seq)
{
	ssize_t ret = 0;
	u8 data = 0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (seq > 1)
		return -EINVAL;

	da9052_lock(priv->da9052);
	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTA_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = (u8)ret;

	if (seq == XYZP_MODE)
		data = enable_xyzp_mode(data);
	else if (seq == XP_MODE)
		data = enable_xp_mode(data);
	else {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("Invalid Value passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTA_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	ts->tsi_conf.auto_cont.da9052_tsi_cont_a = data;

	return 0;
}

static ssize_t da9052_tsi_set_sampling_mode(struct da9052_ts_priv *priv,
					u8 mode)
{
	u8 data = 0;
	ssize_t ret = 0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	da9052_lock(priv->da9052);

	ret = read_da9052_reg(priv->da9052, DA9052_ADCCONT_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	data = (u8)ret;

	if (mode == ECONOMY_MODE)
		data = adc_mode_economy_mode(data);
	else if (mode == FAST_MODE)
		data = adc_mode_fast_mode(data);
	else {
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("Invalid interval passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	ret = write_da9052_reg(priv->da9052, DA9052_ADCCONT_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	switch (mode) {
	case ECONOMY_MODE:
		priv->tsi_reg_data_poll_interval =
			TSI_ECO_MODE_REG_DATA_PROCESSING_INTERVAL;
		priv->tsi_raw_data_poll_interval =
			TSI_ECO_MODE_RAW_DATA_PROCESSING_INTERVAL;
		break;
	case FAST_MODE:
		priv->tsi_reg_data_poll_interval =
			TSI_FAST_MODE_REG_DATA_PROCESSING_INTERVAL;
		priv->tsi_raw_data_poll_interval =
			TSI_FAST_MODE_RAW_DATA_PROCESSING_INTERVAL;
		break;
	default:
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("Invalid interval passed \n" );
		return -EINVAL;
	}

	ts->tsi_penup_count =
		(u32)priv->tsi_pdata->pen_up_interval /
		priv->tsi_reg_data_poll_interval;

	return 0;
}

static ssize_t da9052_tsi_config_delay(struct da9052_ts_priv *priv,
					enum TSI_DELAY delay)
{
	ssize_t ret = 0;
	u8 data = 0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (delay > priv->tsi_pdata->max_tsi_delay) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" invalid value for tsi delay!!!\n" );
		return -EINVAL;
	}

	da9052_lock(priv->da9052);

	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTA_REG);
	if(ret < 0) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = clear_bits((u8)ret, DA9052_TSICONTA_TSIDELAY);

	data = set_bits(data, (delay << priv->tsi_pdata->tsi_delay_bit_shift));

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTA_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	ts->tsi_conf.auto_cont.da9052_tsi_cont_a = data;

	return 0;
}

ssize_t da9052_tsi_config_skip_slots(struct da9052_ts_priv *priv,
					enum TSI_SLOT_SKIP skip)
{
	ssize_t ret = 0;
	u8 data = 0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (skip > priv->tsi_pdata->max_tsi_skip_slot) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" invalid value for tsi skip slots!!!\n" );
		return -EINVAL;
	}

	da9052_lock(priv->da9052);

	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTA_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = clear_bits((u8)ret, DA9052_TSICONTA_TSISKIP);

	data = set_bits(data, (skip << priv->tsi_pdata->tsi_skip_bit_shift));

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTA_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_config_skip_slots:");
		DA9052_DEBUG(" write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	ts->tsi_conf.auto_cont.da9052_tsi_cont_a = data;

	return 0;
}

static ssize_t da9052_tsi_config_state(struct da9052_ts_priv *priv,
					enum TSI_STATE state)
{
	s32 ret;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (ts->tsi_conf.state == state)
		return 0;

	switch (state) {
	case TSI_AUTO_MODE:
		ts->tsi_zero_data_cnt = 0;
		priv->early_data_flag = TRUE;
		priv->debounce_over = FALSE;
		priv->win_reference_valid = FALSE;

		clean_tsi_fifos(priv);

		ret = da9052_tsi_config_auto_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_manual_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_power_supply(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_enable_irq(priv, TSI_PEN_DWN);
		if (ret)
			return ret;
		ts->tsi_conf.tsi_pendown_irq_mask = RESET;

		ret = da9052_tsi_disable_irq(priv, TSI_DATA_RDY);
		if (ret)
			return ret;
		ts->tsi_conf.tsi_ready_irq_mask	  = SET;

		da9052_tsi_reg_pendwn_event(priv);
		da9052_tsi_reg_datardy_event(priv);

		ret = da9052_tsi_config_pen_detect(priv, ENABLE);
		if (ret)
			return ret;
		break;

	case TSI_IDLE:
		ts->pen_dwn_event = RESET;

		ret = da9052_tsi_config_pen_detect(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_auto_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_manual_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_power_supply(priv, DISABLE);
		if (ret)
			return ret;

		if (ts->pd_reg_status) {
			priv->da9052->unregister_event_notifier(priv->da9052,
								&priv->pd_nb);
		ts->pd_reg_status = RESET;
		}
		break;

	default:
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Invalid state passed");
		return -EINVAL;
	}

	ts->tsi_conf.state = state;

	return 0;
}

static void da9052_tsi_reg_pendwn_event(struct da9052_ts_priv *priv)
{
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = get_tsi_drvdata();

	if (ts->pd_reg_status) {
		DA9052_DEBUG("%s: Pen down ",__FUNCTION__);
		DA9052_DEBUG("Registeration is already done \n");
		return;
	}

	priv->pd_nb.eve_type = PEN_DOWN_EVE;
	priv->pd_nb.call_back = &da9052_tsi_pen_down_handler;

	ret = priv->da9052->register_event_notifier(priv->da9052, &priv->pd_nb);
	if (ret) {
		DA9052_DEBUG("%s: EH Registeration",__FUNCTION__);
		DA9052_DEBUG(" Failed: ret = %d\n",ret );
		ts->pd_reg_status = RESET;
	} else
		ts->pd_reg_status = SET;

	priv->os_data_cnt = 0;
	priv->raw_data_cnt = 0;

	return;
}

static void da9052_tsi_reg_datardy_event(struct da9052_ts_priv *priv)
{
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = get_tsi_drvdata();

	if(ts->datardy_reg_status)
	{
		DA9052_DEBUG("%s: Data Ready ",__FUNCTION__);
		DA9052_DEBUG("Registeration is already done \n");
		return;
	}

	priv->datardy_nb.eve_type = TSI_READY_EVE;
	priv->datardy_nb.call_back = &da9052_tsi_data_ready_handler;

	ret = priv->da9052->register_event_notifier(priv->da9052,
						&priv->datardy_nb);

	if(ret)
	{
		DA9052_DEBUG("%s: EH Registeration",__FUNCTION__);
		DA9052_DEBUG(" Failed: ret = %d\n",ret );
		ts->datardy_reg_status = RESET;
	} else
		ts->datardy_reg_status = SET;

	return;
}

static ssize_t __init da9052_tsi_create_input_dev(struct input_dev **ip_dev,
							u8 n)
{
	u8 i;
	s32 ret;
	struct input_dev *dev = NULL;

	if (!n)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		dev = input_allocate_device();
		if (!dev) {
			DA9052_DEBUG(KERN_ERR "%s:%s():memory allocation for \
					inputdevice failed\n", __FILE__,
								__FUNCTION__);
			return -ENOMEM;
		}

		ip_dev[i] = dev;
		switch (i) {
		case TSI_INPUT_DEVICE_OFF:
			dev->name = DA9052_TSI_INPUT_DEV;
			dev->phys = "input(tsi)";
			break;
		default:
			break;
		}
	}
	dev->id.vendor = DA9052_VENDOR_ID;
	dev->id.product = DA9052_PRODUCT_ID;
	dev->id.bustype = BUS_RS232;
	dev->id.version = TSI_VERSION;
	dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	dev->evbit[0] = (BIT_MASK(EV_SYN) |
			BIT_MASK(EV_KEY) |
			BIT_MASK(EV_ABS));

	input_set_abs_params(dev, ABS_X, 0, DA9052_DISPLAY_X_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, DA9052_DISPLAY_Y_MAX, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, DA9052_TOUCH_PRESSURE_MAX,
				0, 0);

	ret = input_register_device(dev);
	if (ret) {
		DA9052_DEBUG(KERN_ERR "%s: Could ", __FUNCTION__);
		DA9052_DEBUG("not register input device(touchscreen)!\n");
		ret = -EIO;
		goto fail;
	}
	return 0;

fail:
	for (; i-- != 0; )
		input_free_device(ip_dev[i]);
	return -EINVAL;
}

static ssize_t __init da9052_tsi_init_drv(struct da9052_ts_priv *priv)
{
	u8 cnt = 0;
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = get_tsi_drvdata();

	if ((DA9052_GPIO_PIN_3 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_4 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_5 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_6 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_7 != DA9052_GPIO_CONFIG_TSI)) {
		printk(KERN_ERR"DA9052_TSI: Configure DA9052 GPIO ");
		printk(KERN_ERR"pins for TSI\n");
		return -EINVAL;
	}

	ret = da9052_tsi_config_gpio(priv);

	ret = da9052_tsi_config_state(priv, TSI_IDLE);
	ts->tsi_conf.state = TSI_IDLE;

	da9052_tsi_config_measure_seq(priv, TSI_MODE_VALUE);

	da9052_tsi_config_skip_slots(priv, TSI_SLOT_SKIP_VALUE);

	da9052_tsi_config_delay(priv, TSI_DELAY_VALUE);

	da9052_tsi_set_sampling_mode(priv, DEFAULT_TSI_SAMPLING_MODE);

	ts->tsi_calib = get_calib_config();

	ret = da9052_tsi_create_input_dev(ts->input_devs, NUM_INPUT_DEVS);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("da9052_tsi_create_input_dev Failed \n" );
		return ret;
	}

	da9052_init_tsi_fifos(priv);

	init_completion(&priv->tsi_reg_proc_thread.notifier);
	priv->tsi_reg_proc_thread.state = ACTIVE;
	priv->tsi_reg_proc_thread.pid =
				kernel_thread(da9052_tsi_reg_proc_thread,
					priv, CLONE_KERNEL | SIGCHLD);

	init_completion(&priv->tsi_raw_proc_thread.notifier);
	priv->tsi_raw_proc_thread.state = ACTIVE;
	priv->tsi_raw_proc_thread.pid =
				kernel_thread(da9052_tsi_raw_proc_thread,
					priv, CLONE_KERNEL | SIGCHLD);

	ret = da9052_tsi_config_state(priv, DEFAULT_TSI_STATE);
	if (ret) {
		for (cnt = 0; cnt < NUM_INPUT_DEVS; cnt++) {
			if (ts->input_devs[cnt] != NULL)
				input_free_device(ts->input_devs[cnt]);
		}
	}

	return 0;
}

u32 da9052_tsi_get_input_dev(u8 off)
{
	struct da9052_tsi_info   *ts = get_tsi_drvdata();

	if (off > NUM_INPUT_DEVS-1)
		return -EINVAL;

	return (u32)ts->input_devs[off];
}

static ssize_t da9052_tsi_config_pen_detect(struct da9052_ts_priv *priv,
						u8 flag)
{
	u8 data;
	u32 ret;
	struct da9052_tsi_info  *ts = get_tsi_drvdata();

	da9052_lock(priv->da9052);
	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTA_REG);
	if (ret < 0) {
		DA9052_DEBUG("%s:", __FUNCTION__);
		DA9052_DEBUG(" read_da9052_reg	Failed\n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	if (flag == ENABLE)
		data = set_bits((u8)ret, DA9052_TSICONTA_PENDETEN);
	else if (flag == DISABLE)
		data = clear_bits((u8)ret, DA9052_TSICONTA_PENDETEN);
	else {
		DA9052_DEBUG("%s:", __FUNCTION__);
		DA9052_DEBUG(" Invalid flag passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTA_REG, data);
	if (ret < 0) {
		DA9052_DEBUG("%s:", __FUNCTION__);
		DA9052_DEBUG(" write_da9052_reg	Failed\n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	ts->tsi_conf.auto_cont.da9052_tsi_cont_a = data;
	return 0;
}

static ssize_t da9052_tsi_disable_irq(struct da9052_ts_priv *priv,
					enum TSI_IRQ tsi_irq)
{
	u8 data = 0;
	ssize_t ret =0;
	struct da9052_tsi_info	*ts = get_tsi_drvdata();

	da9052_lock(priv->da9052);
	ret = read_da9052_reg(priv->da9052, DA9052_IRQMASKB_REG);
	if (ret < 0)	{
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_disable_irq:");
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	data = ret;
	switch (tsi_irq) {
	case TSI_PEN_DWN:
		data = mask_pendwn_irq(data);
	break;
	case TSI_DATA_RDY:
		data = mask_tsi_rdy_irq(data);
	break;
	default:
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_disable_irq:");
		DA9052_DEBUG("Invalid IRQ passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}
	ret = write_da9052_reg(priv->da9052, DA9052_IRQMASKB_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_disable_irq:");
		DA9052_DEBUG("write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);
	switch (tsi_irq) {
	case TSI_PEN_DWN:
		ts->tsi_conf.tsi_pendown_irq_mask = SET;
	break;
	case TSI_DATA_RDY:
		ts->tsi_conf.tsi_ready_irq_mask = SET;
	break;
	default:
		return -EINVAL;
	}

	return 0; 

}

static ssize_t da9052_tsi_enable_irq(struct da9052_ts_priv *priv,
					enum TSI_IRQ tsi_irq)
{
	u8 data =0;
	ssize_t ret =0;
	struct da9052_tsi_info  *ts = get_tsi_drvdata();

	da9052_lock(priv->da9052);
	ret = read_da9052_reg(priv->da9052, DA9052_IRQMASKB_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_enable_irq:");
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = ret;
	switch (tsi_irq) {
	case TSI_PEN_DWN:
		data = unmask_pendwn_irq(data);
	break;
	case TSI_DATA_RDY:
		data = unmask_tsi_rdy_irq(data);
	break;
	default:
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_enable_irq:");
		DA9052_DEBUG("Invalid IRQ passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}
	ret = write_da9052_reg(priv->da9052, DA9052_IRQMASKB_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI:da9052_tsi_enable_irq:");
		DA9052_DEBUG("write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);
	switch (tsi_irq) {
	case TSI_PEN_DWN:
		ts->tsi_conf.tsi_pendown_irq_mask = RESET;
	break;
	case TSI_DATA_RDY:
		ts->tsi_conf.tsi_ready_irq_mask = RESET;
	break;
	default:
		return -EINVAL;
	}

	return 0;
 }

static ssize_t da9052_tsi_config_gpio(struct da9052_ts_priv *priv)
{
	u8 idx = 0;
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg[priv->tsi_pdata->num_gpio_tsi_register];

	ssc_msg[idx++].addr  = DA9052_GPIO0203_REG;
	ssc_msg[idx++].addr  = DA9052_GPIO0405_REG;
	ssc_msg[idx++].addr  = DA9052_GPIO0607_REG;

	da9052_lock(priv->da9052);
	ret = priv->da9052->read_many(priv->da9052, ssc_msg,idx);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("da9052_ssc_read_many Failed\n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	idx = 0;
	ssc_msg[idx].data = clear_bits(ssc_msg[idx].data,
						DA9052_GPIO0203_GPIO3PIN);
	idx++;
	ssc_msg[idx].data = clear_bits(ssc_msg[idx].data,
		(DA9052_GPIO0405_GPIO4PIN | DA9052_GPIO0405_GPIO5PIN));
	idx++;
	ssc_msg[idx].data = clear_bits(ssc_msg[idx].data,
		(DA9052_GPIO0607_GPIO6PIN | DA9052_GPIO0607_GPIO7PIN));
	idx++;

	ret = priv->da9052->write_many(priv->da9052, ssc_msg,idx);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("da9052_ssc_read_many Failed\n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	return 0;
}

s32 da9052_pm_configure_ldo(struct da9052_ts_priv *priv,
				struct da9052_ldo_config ldo_config)
{
	struct da9052_ssc_msg msg;
	u8 reg_num;
	u8 ldo_volt;
	u8 ldo_volt_bit = 0;
	u8 ldo_conf_bit = 0;
	u8 ldo_en_bit = 0;
	s8 ldo_pd_bit = -1;
	s32 ret = 0;

	if (validate_ldo9_mV(ldo_config.ldo_volt))
		return INVALID_LDO9_VOLT_VALUE;

	ldo_volt = ldo9_mV_to_reg(ldo_config.ldo_volt);

	reg_num = DA9052_LDO9_REG;
	ldo_volt_bit = DA9052_LDO9_VLDO9;
	ldo_conf_bit = DA9052_LDO9_LDO9CONF;
	ldo_en_bit = DA9052_LDO9_LDO9EN;

	da9052_lock(priv->da9052);

	msg.addr = reg_num;

	ret = priv->da9052->read(priv->da9052, &msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}
	msg.data = ldo_volt |
		(ldo_config.ldo_conf ? ldo_conf_bit : 0) |
		(msg.data & ldo_en_bit);

	ret = priv->da9052->write(priv->da9052, &msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	if (-1 != ldo_pd_bit) {
		msg.addr = DA9052_PULLDOWN_REG;
		ret = priv->da9052->read(priv->da9052, &msg);
		if (ret) {
			da9052_unlock(priv->da9052);
			return -EINVAL;
		}

		msg.data = (ldo_config.ldo_pd ?
				set_bits(msg.data, ldo_pd_bit) :
				clear_bits(msg.data, ldo_pd_bit));

		ret = priv->da9052->write(priv->da9052, &msg);
		if (ret) {
			da9052_unlock(priv->da9052);
			return -EINVAL;
		}

	}
	da9052_unlock(priv->da9052);

	return 0;
}


s32 da9052_pm_set_ldo(struct da9052_ts_priv *priv, u8 ldo_num, u8 flag)
{
	struct da9052_ssc_msg msg;
	u8 reg_num = 0;
	u8 value = 0;
	s32 ret = 0;

	DA9052_DEBUG("I am in function: %s\n", __FUNCTION__);

	reg_num = DA9052_LDO9_REG;
	value =	DA9052_LDO9_LDO9EN;
	da9052_lock(priv->da9052);

	msg.addr = reg_num;

	ret = priv->da9052->read(priv->da9052, &msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	msg.data = flag ?
		set_bits(msg.data, value) :
		clear_bits(msg.data, value);

	ret = priv->da9052->write(priv->da9052, &msg);
	if (ret) {
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	da9052_unlock(priv->da9052);

	return 0;
}

static ssize_t da9052_tsi_config_power_supply(struct da9052_ts_priv *priv,
						u8 state)
{
	struct da9052_ldo_config ldo_config;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (state != ENABLE && state != DISABLE) {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("Invalid state Passed\n" );
		return -EINVAL;
	}

	ldo_config.ldo_volt = priv->tsi_pdata->tsi_supply_voltage;
	ldo_config.ldo_num =  priv->tsi_pdata->tsi_ref_source;
	ldo_config.ldo_conf = RESET;

	if (da9052_pm_configure_ldo(priv, ldo_config))
		return -EINVAL;

	if (da9052_pm_set_ldo(priv, priv->tsi_pdata->tsi_ref_source, state))
		return -EINVAL;

	if (state == ENABLE)
		ts->tsi_conf.ldo9_en = SET;
	else
		ts->tsi_conf.ldo9_en = RESET;

	return 0;
}

static ssize_t da9052_tsi_config_auto_mode(struct da9052_ts_priv *priv,
						u8 state)
{
	u8 data;
	s32 ret = 0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (state != ENABLE && state != DISABLE)
		return -EINVAL;

	da9052_lock(priv->da9052);

	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTA_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = (u8)ret;

	if (state == ENABLE)
		data = set_auto_tsi_en(data);
	else if (state == DISABLE)
		data = reset_auto_tsi_en(data);
	else {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("Invalid Parameter Passed \n" );
		da9052_unlock(priv->da9052);
		return -EINVAL;
	}

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTA_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Failed to configure Auto TSI mode\n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);
	ts->tsi_conf.auto_cont.da9052_tsi_cont_a = data;
	return 0;
}

static ssize_t da9052_tsi_config_manual_mode(struct da9052_ts_priv *priv,
						u8 state)
{
	u8 data = 0;
	ssize_t ret=0;
	struct da9052_tsi_info *ts = get_tsi_drvdata();

	if (state != ENABLE && state != DISABLE) {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("Invalid state Passed\n" );
		return -EINVAL;
	}

	da9052_lock(priv->da9052);

	ret = read_da9052_reg(priv->da9052, DA9052_TSICONTB_REG);
	if (ret < 0) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("read_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	data = (u8)ret;
	if (state == DISABLE)
		data = disable_tsi_manual_mode(data);
	else
		data = enable_tsi_manual_mode(data);

	ret = write_da9052_reg(priv->da9052, DA9052_TSICONTB_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("write_da9052_reg Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}

	if (state == DISABLE)
		ts->tsi_conf.man_cont.tsi_cont_b.tsi_man = RESET;
	else
		ts->tsi_conf.man_cont.tsi_cont_b.tsi_man = SET;

	data = 0;
	data = set_bits(data, DA9052_ADC_TSI);

	ret = write_da9052_reg(priv->da9052, DA9052_ADCMAN_REG, data);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("ADC write Failed \n" );
		da9052_unlock(priv->da9052);
		return ret;
	}
	da9052_unlock(priv->da9052);

	return 0;
}

static u32 da9052_tsi_get_reg_data(struct da9052_ts_priv *priv)
{
	u32 free_cnt, copy_cnt, cnt;

	if (down_interruptible(&priv->tsi_reg_fifo.lock))
		return 0;

	copy_cnt = 0;

	if ((priv->tsi_reg_fifo.head - priv->tsi_reg_fifo.tail) > 1) {
		free_cnt = get_reg_free_space_cnt(priv);
		if (free_cnt > TSI_POLL_SAMPLE_CNT)
			free_cnt = TSI_POLL_SAMPLE_CNT;

		cnt = da9052_tsi_get_rawdata(
			&priv->tsi_reg_fifo.data[priv->tsi_reg_fifo.tail],
			free_cnt);

		if (cnt > free_cnt) {
			DA9052_DEBUG("EH copied more data");
			return -EINVAL;
		}

		copy_cnt = cnt;

		while (cnt--)
			incr_with_wrap_reg_fifo(priv->tsi_reg_fifo.tail);

	} else if ((priv->tsi_reg_fifo.head - priv->tsi_reg_fifo.tail) <= 0) {

		free_cnt = (TSI_REG_DATA_BUF_SIZE - priv->tsi_reg_fifo.tail);
		if (free_cnt > TSI_POLL_SAMPLE_CNT) {
			free_cnt = TSI_POLL_SAMPLE_CNT;

			cnt = da9052_tsi_get_rawdata(
			&priv->tsi_reg_fifo.data[priv->tsi_reg_fifo.tail],
			free_cnt);
			if (cnt > free_cnt) {
				DA9052_DEBUG("EH copied more data");
				return -EINVAL;
			}
			copy_cnt = cnt;

			while (cnt--)
				incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
		} else {
			if (free_cnt) {
				cnt = da9052_tsi_get_rawdata(
					&priv->
					tsi_reg_fifo.data[priv->
					tsi_reg_fifo.tail],
					free_cnt
					);
				if (cnt > free_cnt) {
					DA9052_DEBUG("EH copied more data");
					return -EINVAL;
				}
				copy_cnt = cnt;
				while (cnt--)
					incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
			}
			free_cnt = priv->tsi_reg_fifo.head;
			if (free_cnt > TSI_POLL_SAMPLE_CNT - copy_cnt)
				free_cnt = TSI_POLL_SAMPLE_CNT - copy_cnt;
			if (free_cnt) {
				cnt = da9052_tsi_get_rawdata(
					&priv->tsi_reg_fifo.data[priv->
					tsi_reg_fifo.tail], free_cnt
					);
				if (cnt > free_cnt) {
					DA9052_DEBUG("EH copied more data");
					return -EINVAL;
				}

				copy_cnt += cnt;

				while (cnt--)
					incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
			}
		}
	} else
		copy_cnt = 0;

	up(&priv->tsi_reg_fifo.lock);

	return copy_cnt;
}


static ssize_t da9052_tsi_reg_proc_thread(void *ptr)
{
	u32 data_cnt;
	ssize_t ret = 0;
	struct da9052_tsi_info *ts;
	struct da9052_ts_priv *priv = (struct da9052_ts_priv *)ptr;

	set_freezable();

	while (priv->tsi_reg_proc_thread.state == ACTIVE) {

		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(priv->
					tsi_reg_data_poll_interval));

		ts = get_tsi_drvdata();

		if (!ts->pen_dwn_event)
			continue;

		data_cnt = da9052_tsi_get_reg_data(priv);

		da9052_tsi_process_reg_data(priv);

		if (data_cnt)
			ts->tsi_zero_data_cnt = 0;
		else {
			if ((++(ts->tsi_zero_data_cnt)) >
						     ts->tsi_penup_count) {
				ts->pen_dwn_event = RESET;
				da9052_tsi_penup_event(priv);
			}
		}
	}

	complete_and_exit(&priv->tsi_reg_proc_thread.notifier, 0);
	return 0;
}


static void da9052_tsi_penup_event(struct da9052_ts_priv *priv)
{

	struct da9052_tsi_info *ts = get_tsi_drvdata();
	struct input_dev *ip_dev =
		(struct input_dev *)da9052_tsi_get_input_dev(
		(u8)TSI_INPUT_DEVICE_OFF);

	if (da9052_tsi_config_auto_mode(priv, DISABLE))
		goto exit;
	ts->tsi_conf.auto_cont.tsi_cont_a.auto_tsi_en = RESET;

	if (da9052_tsi_config_power_supply(priv, ENABLE))
		goto exit;

	ts->tsi_conf.ldo9_en = RESET;

	if (da9052_tsi_enable_irq(priv, TSI_PEN_DWN))
		goto exit;
	ts->tsi_conf.tsi_pendown_irq_mask = RESET;
	tsi_reg.tsi_state =  WAIT_FOR_PEN_DOWN;

	if (da9052_tsi_disable_irq(priv, TSI_DATA_RDY))
		goto exit;

	ts->tsi_zero_data_cnt = 0;
	priv->early_data_flag = TRUE;
	priv->debounce_over = FALSE;
	priv->win_reference_valid = FALSE;

	printk(KERN_INFO "The raw data count is %d \n", priv->raw_data_cnt);
	printk(KERN_INFO "The OS data count is %d \n", priv->os_data_cnt);
	printk(KERN_INFO "PEN UP DECLARED \n");
	input_report_abs(ip_dev, BTN_TOUCH, 0);
	input_sync(ip_dev);
	priv->os_data_cnt = 0;
	priv->raw_data_cnt = 0;

exit:
	clean_tsi_fifos(priv);
	return;
}

void da9052_tsi_pen_down_handler(struct da9052_eh_nb *eh_data, u32 event)
{
	ssize_t ret = 0;
	struct da9052_ts_priv *priv =
		container_of(eh_data, struct da9052_ts_priv, pd_nb);
	struct da9052_tsi_info *ts = get_tsi_drvdata();
	struct input_dev *ip_dev = 
		(struct input_dev*)da9052_tsi_get_input_dev(
		(u8)TSI_INPUT_DEVICE_OFF);

	if (tsi_reg.tsi_state !=  WAIT_FOR_PEN_DOWN)
		return;

	tsi_reg.tsi_state = WAIT_FOR_SAMPLING;

	if (ts->tsi_conf.state != TSI_AUTO_MODE) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Configure TSI to auto mode.\n" );
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Then call this API.\n" );
		goto fail;
	}

	if (da9052_tsi_config_power_supply(priv, ENABLE))
		goto fail;

	if (da9052_tsi_disable_irq(priv, TSI_PEN_DWN))
		goto fail;

	if (da9052_tsi_config_auto_mode(priv, ENABLE))
		goto fail;
	ts->tsi_conf.auto_cont.tsi_cont_a.auto_tsi_en = SET;

	if (da9052_tsi_enable_irq(priv, TSI_DATA_RDY))
		goto fail;

	input_sync(ip_dev);

	ts->tsi_rdy_event = (DA9052_EVENTB_ETSIREADY & (event>>8));
	ts->pen_dwn_event = (DA9052_EVENTB_EPENDOWN & (event>>8));

	tsi_reg.tsi_state =  SAMPLING_ACTIVE;

	goto success;

fail:
	if (ts->pd_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
						&priv->pd_nb);
		ts->pd_reg_status = RESET;

		priv->da9052->register_event_notifier(priv->da9052,
					&priv->datardy_nb);
		da9052_tsi_reg_pendwn_event(priv);
	}

success:
	ret = 0;
	printk(KERN_INFO "Exiting PEN DOWN HANDLER \n");
}

void da9052_tsi_data_ready_handler(struct da9052_eh_nb *eh_data, u32 event)
{
	struct da9052_ssc_msg tsi_data[4];
	s32 ret;
	struct da9052_ts_priv *priv =
		container_of(eh_data, struct da9052_ts_priv, datardy_nb);

	if (tsi_reg.tsi_state !=  SAMPLING_ACTIVE)
		return;

	tsi_data[0].addr  = DA9052_TSIXMSB_REG;
	tsi_data[1].addr  = DA9052_TSIYMSB_REG;
	tsi_data[2].addr  = DA9052_TSILSB_REG;
	tsi_data[3].addr  = DA9052_TSIZMSB_REG;

	tsi_data[0].data  = 0;
	tsi_data[1].data  = 0;
	tsi_data[2].data  = 0;
	tsi_data[3].data  = 0;

	da9052_lock(priv->da9052);

	ret = priv->da9052->read_many(priv->da9052, tsi_data, 4);
	if (ret) {
		DA9052_DEBUG("Error in reading TSI data \n" );
		da9052_unlock(priv->da9052);
		return;
	}
	da9052_unlock(priv->da9052);

#if 1
	mutex_lock(&tsi_reg.tsi_fifo_lock);

	tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_end].x_msb = tsi_data[0].data;
	tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_end].y_msb = tsi_data[1].data;
	tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_end].lsb   = tsi_data[2].data;
	tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_end].z_msb = tsi_data[3].data;
	incr_with_wrap(tsi_reg.tsi_fifo_end);

	if (tsi_reg.tsi_fifo_end == tsi_reg.tsi_fifo_start)
		tsi_reg.tsi_fifo_start++;

	mutex_unlock(&tsi_reg.tsi_fifo_lock);
#endif
//	printk(KERN_INFO "Exiting Data ready handler \n");
}

static s32 da9052_tsi_get_rawdata(struct da9052_tsi_reg *buf, u8 cnt) {
	u32 data_cnt = 0;
	u32 rem_data_cnt = 0;

	mutex_lock(&tsi_reg.tsi_fifo_lock);

	if (tsi_reg.tsi_fifo_start < tsi_reg.tsi_fifo_end) {
		data_cnt = (tsi_reg.tsi_fifo_end - tsi_reg.tsi_fifo_start);

		if (cnt < data_cnt)
			data_cnt = cnt;

		memcpy(buf, &tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_start],
				sizeof(struct da9052_tsi_reg) * data_cnt);

		tsi_reg.tsi_fifo_start += data_cnt;

		if (tsi_reg.tsi_fifo_start == tsi_reg.tsi_fifo_end) {
			tsi_reg.tsi_fifo_start = 0;
			tsi_reg.tsi_fifo_end = 0;
		}
	} else if (tsi_reg.tsi_fifo_start > tsi_reg.tsi_fifo_end) {
		data_cnt = ((TSI_FIFO_SIZE - tsi_reg.tsi_fifo_start)
		+ tsi_reg.tsi_fifo_end);

		if (cnt < data_cnt)
			data_cnt = cnt;

		if (data_cnt <= (TSI_FIFO_SIZE - tsi_reg.tsi_fifo_start)) {
			memcpy(buf, &tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_start],
				sizeof(struct da9052_tsi_reg) * data_cnt);

			tsi_reg.tsi_fifo_start += data_cnt;
			if (tsi_reg.tsi_fifo_start >= TSI_FIFO_SIZE)
				tsi_reg.tsi_fifo_start = 0;
		} else {
			memcpy(buf, &tsi_reg.tsi_fifo[tsi_reg.tsi_fifo_start],
				sizeof(struct da9052_tsi_reg)
				* (TSI_FIFO_SIZE - tsi_reg.tsi_fifo_start));

			rem_data_cnt = (data_cnt -
				(TSI_FIFO_SIZE - tsi_reg.tsi_fifo_start));

			memcpy(buf, &tsi_reg.tsi_fifo[0],
				sizeof(struct da9052_tsi_reg) * rem_data_cnt);

			tsi_reg.tsi_fifo_start = rem_data_cnt;
		}

		if (tsi_reg.tsi_fifo_start == tsi_reg.tsi_fifo_end) {
				tsi_reg.tsi_fifo_start = 0;
			tsi_reg.tsi_fifo_end = 0;
		}
	} else
		data_cnt = 0;

	mutex_unlock(&tsi_reg.tsi_fifo_lock);

	return data_cnt;
}

static ssize_t da9052_tsi_suspend(struct platform_device *dev, 
							pm_message_t state)
{
	printk(KERN_INFO "%s: called\n", __FUNCTION__);
	return 0;
}

static ssize_t da9052_tsi_resume(struct platform_device *dev)
{
	printk(KERN_INFO "%s: called\n", __FUNCTION__);
	return 0;
}

static s32 __devinit da9052_tsi_probe(struct platform_device *pdev)
{

	struct da9052_ts_priv *priv;
	struct da9052_tsi_platform_data *pdata = pdev->dev.platform_data;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, priv);

	priv->tsi_pdata = pdata;

	if (da9052_tsi_init_drv(priv))
			return -EFAULT;

	mutex_init(&tsi_reg.tsi_fifo_lock);
	tsi_reg.tsi_state = WAIT_FOR_PEN_DOWN;

	printk(KERN_INFO "TSI Drv Successfully Inserted %s\n",
					DA9052_TSI_DEVICE_NAME);
	return 0;
}

static int __devexit da9052_tsi_remove(struct platform_device *pdev)
{
	struct da9052_ts_priv *priv = platform_get_drvdata(pdev);
	struct da9052_tsi_info *ts = get_tsi_drvdata();
	s32 ret = 0, i = 0;

	ret = da9052_tsi_config_state(priv, TSI_IDLE);
	if (!ret)
		return -EINVAL;

	if (ts->pd_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
							&priv->pd_nb);
		ts->pd_reg_status = RESET;
	}

	if (ts->datardy_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
							&priv->datardy_nb);
		ts->datardy_reg_status = RESET;
	}

	mutex_destroy(&tsi_reg.tsi_fifo_lock);

	priv->tsi_reg_proc_thread.state = INACTIVE;
	wait_for_completion(&priv->tsi_reg_proc_thread.notifier);

	priv->tsi_raw_proc_thread.state = INACTIVE;
	wait_for_completion(&priv->tsi_raw_proc_thread.notifier);

	for (i = 0; i < NUM_INPUT_DEVS; i++) {
		input_unregister_device(ts->input_devs[i]);
	}

	platform_set_drvdata(pdev, NULL);
	DA9052_DEBUG(KERN_DEBUG "Removing %s \n", DA9052_TSI_DEVICE_NAME);

	return 0;
}

static struct platform_driver da9052_tsi_driver = {
	.probe		= da9052_tsi_probe,
	.remove		= __devexit_p(da9052_tsi_remove),
	.suspend	= da9052_tsi_suspend,
	.resume		= da9052_tsi_resume,
	.driver		= {
				.name	= DA9052_TSI_DEVICE_NAME,
				.owner	= THIS_MODULE,
			},
};

static int __init da9052_tsi_init(void)
{
	printk("DA9052 TSI Device Driver, v1.0\n");
	return platform_driver_register(&da9052_tsi_driver);
}
module_init(da9052_tsi_init);

static void __exit da9052_tsi_exit(void)
{
	printk(KERN_ERR "TSI Driver %s Successfully Removed \n",
					DA9052_TSI_DEVICE_NAME);
	return;

}
module_exit(da9052_tsi_exit);

MODULE_DESCRIPTION("Touchscreen driver for Dialog Semiconductor DA9052");
MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
