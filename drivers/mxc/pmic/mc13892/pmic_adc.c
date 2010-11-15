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
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/device.h>

#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>

#include "../core/pmic.h"

#define         DEF_ADC_0     0x008000
#define         DEF_ADC_3     0x0001c0

#define ADC_NB_AVAILABLE        2

#define MAX_CHANNEL             7

#define MC13892_ADC0_TS_M_LSH	14
#define MC13892_ADC0_TS_M_WID	3

/*
 * Maximun allowed variation in the three X/Y co-ordinates acquired from
 * touch-screen
 */
#define DELTA_Y_MAX             50
#define DELTA_X_MAX             50

/*
 * ADC 0
 */
#define ADC_CHRGICON		0x000002
#define ADC_WAIT_TSI_0		0x001400

#define ADC_INC                 0x030000
#define ADC_BIS                 0x800000
#define ADC_CHRGRAW_D5          0x008000

/*
 * ADC 1
 */

#define ADC_EN                  0x000001
#define ADC_SGL_CH              0x000002
#define ADC_ADCCAL		0x000004
#define ADC_ADSEL               0x000008
#define ADC_TRIGMASK		0x000010
#define ADC_CH_0_POS            5
#define ADC_CH_0_MASK           0x0000E0
#define ADC_CH_1_POS            8
#define ADC_CH_1_MASK           0x000700
#define ADC_DELAY_POS           11
#define ADC_DELAY_MASK          0x07F800
#define ADC_ATO                 0x080000
#define ASC_ADC                 0x100000
#define ADC_WAIT_TSI_1		0x200001
#define ADC_NO_ADTRIG           0x200000

/*
 * ADC 2 - 4
 */
#define ADD1_RESULT_MASK        0x00000FFC
#define ADD2_RESULT_MASK        0x00FFC000
#define ADC_TS_MASK             0x00FFCFFC

#define ADC_WCOMP               0x040000
#define ADC_WCOMP_H_POS         0
#define ADC_WCOMP_L_POS         9
#define ADC_WCOMP_H_MASK        0x00003F
#define ADC_WCOMP_L_MASK        0x007E00

#define ADC_MODE_MASK           0x00003F

#define ADC_INT_BISDONEI        0x02
#define ADC_TSMODE_MASK 0x007000

typedef enum adc_state {
	ADC_FREE,
	ADC_USED,
	ADC_MONITORING,
} t_adc_state;

typedef enum reading_mode {
	/*!
	 * Enables lithium cell reading
	 */
	M_LITHIUM_CELL = 0x000001,
	/*!
	 * Enables charge current reading
	 */
	M_CHARGE_CURRENT = 0x000002,
	/*!
	 * Enables battery current reading
	 */
	M_BATTERY_CURRENT = 0x000004,
} t_reading_mode;

typedef struct {
	/*!
	 * Delay before first conversion
	 */
	unsigned int delay;
	/*!
	 * sets the ATX bit for delay on all conversion
	 */
	bool conv_delay;
	/*!
	 * Sets the single channel mode
	 */
	bool single_channel;
	/*!
	 * Channel selection 1
	 */
	t_channel channel_0;
	/*!
	 * Channel selection 2
	 */
	t_channel channel_1;
	/*!
	 * Used to configure ADC mode with t_reading_mode
	 */
	t_reading_mode read_mode;
	/*!
	 * Sets the Touch screen mode
	 */
	bool read_ts;
	/*!
	 * Wait TSI event before touch screen reading
	 */
	bool wait_tsi;
	/*!
	 * Sets CHRGRAW scaling to divide by 5
	 * Only supported on 2.0 and higher
	 */
	bool chrgraw_devide_5;
	/*!
	 * Return ADC values
	 */
	unsigned int value[8];
	/*!
	 * Return touch screen values
	 */
	t_touch_screen ts_value;
} t_adc_param;

static int pmic_adc_filter(t_touch_screen *ts_curr);
int mc13892_adc_request(bool read_ts);
int mc13892_adc_release(int adc_index);
t_reading_mode mc13892_set_read_mode(t_channel channel);
PMIC_STATUS mc13892_adc_read_ts(t_touch_screen *touch_sample, int wait_tsi);

/* internal function */
static void callback_tsi(void *);
static void callback_adcdone(void *);
static void callback_adcbisdone(void *);

static int swait;

static int suspend_flag;

static wait_queue_head_t suspendq;

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_adc_init);
EXPORT_SYMBOL(pmic_adc_deinit);
EXPORT_SYMBOL(pmic_adc_convert);
EXPORT_SYMBOL(pmic_adc_convert_8x);
EXPORT_SYMBOL(pmic_adc_set_touch_mode);
EXPORT_SYMBOL(pmic_adc_get_touch_mode);
EXPORT_SYMBOL(pmic_adc_get_touch_sample);

static DECLARE_COMPLETION(adcdone_it);
static DECLARE_COMPLETION(adcbisdone_it);
static DECLARE_COMPLETION(adc_tsi);
static pmic_event_callback_t tsi_event;
static pmic_event_callback_t event_adc;
static pmic_event_callback_t event_adc_bis;
static bool data_ready_adc_1;
static bool data_ready_adc_2;
static bool adc_ts;
static bool wait_ts;
static bool monitor_en;
static bool monitor_adc;
static DECLARE_MUTEX(convert_mutex);

static DECLARE_WAIT_QUEUE_HEAD(queue_adc_busy);
static t_adc_state adc_dev[2];

static unsigned channel_num[] = {
	0,
	1,
	3,
	4,
	2,
	0,
	1,
	3,
	4,
	-1,
	5,
	6,
	7,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1,
	-1
};

static bool pmic_adc_ready;

int is_pmic_adc_ready()
{
	return pmic_adc_ready;
}
EXPORT_SYMBOL(is_pmic_adc_ready);


static int pmic_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	suspend_flag = 1;
	CHECK_ERROR(pmic_write_reg(REG_ADC0, DEF_ADC_0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC1, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC2, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC3, DEF_ADC_3, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC4, 0, PMIC_ALL_BITS));

	return 0;
};

static int pmic_adc_resume(struct platform_device *pdev)
{
	/* nothing for mc13892 adc */
	unsigned int adc_0_reg, adc_1_reg, reg_mask;
	suspend_flag = 0;

	/* let interrupt of TSI again */
	adc_0_reg = ADC_WAIT_TSI_0;
	reg_mask = ADC_WAIT_TSI_0;
	CHECK_ERROR(pmic_write_reg(REG_ADC0, adc_0_reg, reg_mask));
	adc_1_reg = ADC_WAIT_TSI_1 | (ADC_BIS * adc_ts);
	CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg, PMIC_ALL_BITS));

	while (swait > 0) {
		swait--;
		wake_up_interruptible(&suspendq);
	}

	return 0;
};

static void callback_tsi(void *unused)
{
	pr_debug("*** TSI IT mc13892 PMIC_ADC_GET_TOUCH_SAMPLE ***\n");
	if (wait_ts) {
		complete(&adc_tsi);
		pmic_event_mask(EVENT_TSI);
	}
}

static void callback_adcdone(void *unused)
{
	if (data_ready_adc_1)
		complete(&adcdone_it);
}

static void callback_adcbisdone(void *unused)
{
	pr_debug("* adcdone bis it callback *\n");
	if (data_ready_adc_2)
		complete(&adcbisdone_it);
}

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
		pr_debug("mc13892_adc_filter: Ret pos x\n");
		return -1;
	}

	sample_sumx = ts_curr->x_position1 + ts_curr->x_position2;
	sample_sumy = ts_curr->y_position1 + ts_curr->y_position2;

	ts_curr->y_position = sample_sumy / 2;
	ts_curr->x_position = sample_sumx / 2;

	return 0;
}

int pmic_adc_init(void)
{
	unsigned int reg_value = 0, i = 0;

	if (suspend_flag == 1)
		return -EBUSY;

	for (i = 0; i < ADC_NB_AVAILABLE; i++)
		adc_dev[i] = ADC_FREE;

	CHECK_ERROR(pmic_write_reg(REG_ADC0, DEF_ADC_0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC1, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC2, 0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC3, DEF_ADC_3, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC4, 0, PMIC_ALL_BITS));
	reg_value = 0x001000;

	data_ready_adc_1 = false;
	data_ready_adc_2 = false;
	adc_ts = false;
	wait_ts = false;
	monitor_en = false;
	monitor_adc = false;

	/* sub to ADCDone IT */
	event_adc.param = NULL;
	event_adc.func = callback_adcdone;
	CHECK_ERROR(pmic_event_subscribe(EVENT_ADCDONEI, event_adc));

	/* sub to ADCDoneBis IT */
	event_adc_bis.param = NULL;
	event_adc_bis.func = callback_adcbisdone;
	CHECK_ERROR(pmic_event_subscribe(EVENT_ADCBISDONEI, event_adc_bis));

	/* sub to Touch Screen IT */
	tsi_event.param = NULL;
	tsi_event.func = callback_tsi;
	CHECK_ERROR(pmic_event_subscribe(EVENT_TSI, tsi_event));

	return PMIC_SUCCESS;
}

PMIC_STATUS pmic_adc_deinit(void)
{
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_ADCDONEI, event_adc));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_ADCBISDONEI, event_adc_bis));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_TSI, tsi_event));

	return PMIC_SUCCESS;
}

int mc13892_adc_init_param(t_adc_param *adc_param)
{
	int i = 0;

	if (suspend_flag == 1)
		return -EBUSY;

	adc_param->delay = 0;
	adc_param->conv_delay = false;
	adc_param->single_channel = false;
	adc_param->channel_0 = BATTERY_VOLTAGE;
	adc_param->channel_1 = BATTERY_VOLTAGE;
	adc_param->read_mode = 0;
	adc_param->wait_tsi = 0;
	adc_param->chrgraw_devide_5 = true;
	adc_param->read_ts = false;
	adc_param->ts_value.x_position = 0;
	adc_param->ts_value.y_position = 0;
	adc_param->ts_value.contact_resistance = 0;
	for (i = 0; i <= MAX_CHANNEL; i++)
		adc_param->value[i] = 0;

	return 0;
}

PMIC_STATUS mc13892_adc_convert(t_adc_param *adc_param)
{
	bool use_bis = false;
	unsigned int adc_0_reg = 0, adc_1_reg = 0, result_reg = 0, i = 0;
	unsigned int result = 0, temp = 0;
	pmic_version_t mc13892_ver;
	int ret;

	pr_debug("mc13892 ADC - mc13892_adc_convert ....\n");
	if (suspend_flag == 1)
		return -EBUSY;

	if (adc_param->wait_tsi) {
		/* configure adc to wait tsi interrupt */
		INIT_COMPLETION(adc_tsi);

		/*for ts don't use bis */
		/*put ts in interrupt mode */
		/* still kep reference? */
		adc_0_reg = 0x001400 | (ADC_BIS * 0);
		pmic_event_unmask(EVENT_TSI);
		CHECK_ERROR(pmic_write_reg(REG_ADC0, adc_0_reg, PMIC_ALL_BITS));
		/*for ts don't use bis */
		adc_1_reg = 0x200001 | (ADC_BIS * 0);
		CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg, PMIC_ALL_BITS));
		pr_debug("wait tsi ....\n");
		wait_ts = true;
		wait_for_completion_interruptible(&adc_tsi);
		wait_ts = false;
	}
	down(&convert_mutex);
	use_bis = mc13892_adc_request(adc_param->read_ts);
	if (use_bis < 0) {
		pr_debug("process has received a signal and got interrupted\n");
		ret = -EINTR;
		goto out_up_convert_mutex;
	}

	/* CONFIGURE ADC REG 0 */
	adc_0_reg = 0;
	adc_1_reg = 0;
	if (adc_param->read_ts == false) {
		adc_0_reg = adc_param->read_mode & 0x00003F;
		/* add auto inc */
		adc_0_reg |= ADC_INC;
		if (use_bis) {
			/* add adc bis */
			adc_0_reg |= ADC_BIS;
		}
		mc13892_ver = pmic_get_version();
		if (mc13892_ver.revision >= 20)
			if (adc_param->chrgraw_devide_5)
				adc_0_reg |= ADC_CHRGRAW_D5;

		if (adc_param->single_channel)
			adc_1_reg |= ADC_SGL_CH;

		if (adc_param->conv_delay)
			adc_1_reg |= ADC_ATO;

		if (adc_param->single_channel)
			adc_1_reg |= ADC_SGL_CH;

		adc_1_reg |= (adc_param->channel_0 << ADC_CH_0_POS) &
		    ADC_CH_0_MASK;
		adc_1_reg |= (adc_param->channel_1 << ADC_CH_1_POS) &
		    ADC_CH_1_MASK;
	} else {
		adc_0_reg = 0x002400 | (ADC_BIS * use_bis) | ADC_INC;
	}

	if (adc_param->channel_0 == channel_num[CHARGE_CURRENT])
		adc_0_reg |= ADC_CHRGICON;

	/*Change has been made here */
	ret = pmic_write_reg(REG_ADC0, adc_0_reg,
		ADC_INC | ADC_BIS | ADC_CHRGRAW_D5 | 0xfff00ff);
	if (ret != PMIC_SUCCESS) {
		pr_debug("pmic_write_reg");
		goto out_mc13892_adc_release;
	}

	/* CONFIGURE ADC REG 1 */
	if (adc_param->read_ts == false) {
		adc_1_reg |= ADC_NO_ADTRIG;
		adc_1_reg |= ADC_EN;
		adc_1_reg |= (adc_param->delay << ADC_DELAY_POS) &
		    ADC_DELAY_MASK;
		if (use_bis)
			adc_1_reg |= ADC_BIS;
	} else {
		/* configure and start convert to read x and y position */
		/* configure to read 2 value in channel selection 1 & 2 */
		adc_1_reg = 0x100409 | (ADC_BIS * use_bis) | ADC_NO_ADTRIG;
		/* set ATOx = 5, it could be better for ts ADC */
		adc_1_reg |= 0x002800;
	}

	if (adc_param->channel_0 == channel_num[CHARGE_CURRENT]) {
		adc_param->channel_1 = channel_num[CHARGE_CURRENT];
		adc_1_reg &= ~(ADC_CH_0_MASK | ADC_CH_1_MASK | ADC_NO_ADTRIG |
			       ADC_TRIGMASK | ADC_EN | ADC_SGL_CH | ADC_ADCCAL);
		adc_1_reg |= ((adc_param->channel_0 << ADC_CH_0_POS) |
			      (adc_param->channel_1 << ADC_CH_1_POS));
		adc_1_reg |= (ADC_EN | ADC_SGL_CH | ADC_ADCCAL);

		if (use_bis == 0) {
			CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg,
						   0xFFFFFF));
		} else {
			CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg,
						   0xFFFFFF));
			temp = 0x800000;
			CHECK_ERROR(pmic_write_reg(REG_ADC3, temp, 0xFFFFFF));
		}

		adc_1_reg &= ~(ADC_NO_ADTRIG | ASC_ADC | ADC_ADCCAL);
		adc_1_reg |= (ADC_NO_ADTRIG | ASC_ADC);
		if (use_bis == 0) {
			data_ready_adc_1 = true;
			INIT_COMPLETION(adcdone_it);
			CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg,
						   0xFFFFFF));
			pr_debug("wait adc done\n");
			wait_for_completion_interruptible(&adcdone_it);
			data_ready_adc_1 = false;
		} else {
			data_ready_adc_2 = true;
			INIT_COMPLETION(adcbisdone_it);
			CHECK_ERROR(pmic_write_reg(REG_ADC1, adc_1_reg,
						   0xFFFFFF));
			temp = 0x800000;
			CHECK_ERROR(pmic_write_reg(REG_ADC3, temp, 0xFFFFFF));
			pr_debug("wait adc done bis\n");
			wait_for_completion_interruptible(&adcbisdone_it);
			data_ready_adc_2 = false;
		}
	} else {
	if (use_bis == 0) {
		data_ready_adc_1 = false;
		adc_1_reg |= ASC_ADC;
		data_ready_adc_1 = true;
		pr_debug("Write Reg %i = %x\n", REG_ADC1, adc_1_reg);
		INIT_COMPLETION(adcdone_it);
		ret = pmic_write_reg(REG_ADC1, adc_1_reg,
						   ADC_SGL_CH | ADC_ATO |
						   ADC_ADSEL | ADC_CH_0_MASK |
						   ADC_CH_1_MASK |
					   ADC_NO_ADTRIG | ADC_EN |
						   ADC_DELAY_MASK | ASC_ADC |
						   ADC_BIS);
		if (ret != PMIC_SUCCESS) {
			pr_debug("pmic_write_reg");
			goto out_mc13892_adc_release;
		}

		pr_debug("wait adc done\n");
		wait_for_completion_interruptible(&adcdone_it);
		data_ready_adc_1 = false;
	} else {
		data_ready_adc_2 = false;
		adc_1_reg |= ASC_ADC;
		data_ready_adc_2 = true;
		INIT_COMPLETION(adcbisdone_it);
		ret = pmic_write_reg(REG_ADC1, adc_1_reg, 0xFFFFFF);
		if (ret != PMIC_SUCCESS) {
			pr_debug("pmic_write_reg");
			goto out_mc13892_adc_release;
		}

		temp = 0x800000;
		ret = pmic_write_reg(REG_ADC3, temp, 0xFFFFFF);
		if (ret != PMIC_SUCCESS) {
			pr_info("pmic_write_reg");
			goto out_mc13892_adc_release;
		}

		pr_debug("wait adc done bis\n");
		wait_for_completion_interruptible(&adcbisdone_it);
		data_ready_adc_2 = false;
	}
	}

	/* read result and store in adc_param */
	result = 0;
	if (use_bis == 0)
		result_reg = REG_ADC2;
	else
		result_reg = REG_ADC4;

	ret = pmic_write_reg(REG_ADC1, 4 << ADC_CH_1_POS,
		ADC_CH_0_MASK | ADC_CH_1_MASK);
	if (ret != PMIC_SUCCESS) {
		pr_debug("pmic_write_reg");
		goto out_mc13892_adc_release;
	}

	for (i = 0; i <= 3; i++) {
		ret = pmic_read_reg(result_reg, &result, PMIC_ALL_BITS);
		if (ret != PMIC_SUCCESS) {
			pr_debug("pmic_write_reg");
			goto out_mc13892_adc_release;
		}

		adc_param->value[i] = ((result & ADD1_RESULT_MASK) >> 2);
		adc_param->value[i + 4] = ((result & ADD2_RESULT_MASK) >> 14);
		pr_debug("value[%d] = %d, value[%d] = %d\n",
			 i, adc_param->value[i],
			 i + 4, adc_param->value[i + 4]);
	}
	if (adc_param->read_ts) {
		adc_param->ts_value.x_position = adc_param->value[0];
		adc_param->ts_value.x_position1 = adc_param->value[0];
		adc_param->ts_value.x_position2 = adc_param->value[1];
		adc_param->ts_value.y_position = adc_param->value[3];
		adc_param->ts_value.y_position1 = adc_param->value[3];
		adc_param->ts_value.y_position2 = adc_param->value[4];
		adc_param->ts_value.contact_resistance = adc_param->value[6];
		ret = pmic_write_reg(REG_ADC0, 0x0, ADC_TSMODE_MASK);
		if (ret != PMIC_SUCCESS) {
			pr_debug("pmic_write_reg");
			goto out_mc13892_adc_release;
		}
	}

	/*if (adc_param->read_ts) {
	   adc_param->ts_value.x_position = adc_param->value[2];
	   adc_param->ts_value.y_position = adc_param->value[5];
	   adc_param->ts_value.contact_resistance = adc_param->value[6];
	   } */
	ret = PMIC_SUCCESS;
out_mc13892_adc_release:
	mc13892_adc_release(use_bis);
out_up_convert_mutex:
	up(&convert_mutex);

	return ret;
}

t_reading_mode mc13892_set_read_mode(t_channel channel)
{
	t_reading_mode read_mode = 0;

	switch (channel) {
	case CHARGE_CURRENT:
		read_mode = M_CHARGE_CURRENT;
		break;
	case BATTERY_CURRENT:
		read_mode = M_BATTERY_CURRENT;
		break;
	default:
		read_mode = 0;
	}

	return read_mode;
}

PMIC_STATUS pmic_adc_convert(t_channel channel, unsigned short *result)
{
	t_adc_param adc_param;
	PMIC_STATUS ret;
	unsigned int i;

	if (suspend_flag == 1)
		return -EBUSY;

	channel = channel_num[channel];
	if (channel == -1) {
		pr_debug("Wrong channel ID\n");
		return PMIC_PARAMETER_ERROR;
	}
	mc13892_adc_init_param(&adc_param);
	pr_debug("pmic_adc_convert\n");
	adc_param.read_ts = false;
	adc_param.single_channel = true;
	adc_param.read_mode = mc13892_set_read_mode(channel);

	/* Find the group */
	if (channel <= 7)
		adc_param.channel_0 = channel;
	else
		return PMIC_PARAMETER_ERROR;

	ret = mc13892_adc_convert(&adc_param);
	for (i = 0; i <= 7; i++)
		result[i] = adc_param.value[i];

	return ret;
}

PMIC_STATUS pmic_adc_convert_8x(t_channel channel, unsigned short *result)
{
	t_adc_param adc_param;
	int i;
	PMIC_STATUS ret;
	if (suspend_flag == 1)
		return -EBUSY;

	channel = channel_num[channel];

	if (channel == -1) {
		pr_debug("Wrong channel ID\n");
		return PMIC_PARAMETER_ERROR;
	}
	mc13892_adc_init_param(&adc_param);
	pr_debug("pmic_adc_convert_8x\n");
	adc_param.read_ts = false;
	adc_param.single_channel = true;
	adc_param.read_mode = mc13892_set_read_mode(channel);

	if (channel <= 7) {
		adc_param.channel_0 = channel;
		adc_param.channel_1 = channel;
	} else
		return PMIC_PARAMETER_ERROR;

	ret = mc13892_adc_convert(&adc_param);
	for (i = 0; i <= 7; i++)
		result[i] = adc_param.value[i];

	return ret;
}

PMIC_STATUS pmic_adc_set_touch_mode(t_touch_mode touch_mode)
{
	if (suspend_flag == 1)
		return -EBUSY;

	CHECK_ERROR(pmic_write_reg(REG_ADC0,
				   BITFVAL(MC13892_ADC0_TS_M, touch_mode),
				   BITFMASK(MC13892_ADC0_TS_M)));
	return PMIC_SUCCESS;
}

PMIC_STATUS pmic_adc_get_touch_mode(t_touch_mode *touch_mode)
{
	unsigned int value;
	if (suspend_flag == 1)
		return -EBUSY;

	CHECK_ERROR(pmic_read_reg(REG_ADC0, &value, PMIC_ALL_BITS));

	*touch_mode = BITFEXT(value, MC13892_ADC0_TS_M);

	return PMIC_SUCCESS;
}

PMIC_STATUS pmic_adc_get_touch_sample(t_touch_screen *touch_sample, int wait)
{
	if (mc13892_adc_read_ts(touch_sample, wait) != 0)
		return PMIC_ERROR;
	if (0 == pmic_adc_filter(touch_sample))
		return PMIC_SUCCESS;
	else
		return PMIC_ERROR;
}

PMIC_STATUS mc13892_adc_read_ts(t_touch_screen *ts_value, int wait_tsi)
{
	t_adc_param param;
	pr_debug("mc13892_adc : mc13892_adc_read_ts\n");
	if (suspend_flag == 1)
		return -EBUSY;

	if (wait_ts) {
		pr_debug("mc13892_adc : error TS busy \n");
		return PMIC_ERROR;
	}
	mc13892_adc_init_param(&param);
	param.wait_tsi = wait_tsi;
	param.read_ts = true;
	if (mc13892_adc_convert(&param) != 0)
		return PMIC_ERROR;
	/* check if x-y is ok */
	if (param.ts_value.contact_resistance < 1000) {
		ts_value->x_position = param.ts_value.x_position;
		ts_value->x_position1 = param.ts_value.x_position1;
		ts_value->x_position2 = param.ts_value.x_position2;

		ts_value->y_position = param.ts_value.y_position;
		ts_value->y_position1 = param.ts_value.y_position1;
		ts_value->y_position2 = param.ts_value.y_position2;

		ts_value->contact_resistance =
		    param.ts_value.contact_resistance + 1;

	} else {
		ts_value->x_position = 0;
		ts_value->y_position = 0;
		ts_value->contact_resistance = 0;

	}
	return PMIC_SUCCESS;
}

int mc13892_adc_request(bool read_ts)
{
	int adc_index = -1;
	if (read_ts != 0) {
		/*for ts we use bis=0 */
		if (adc_dev[0] == ADC_USED)
			return -1;
		/*no wait here */
		adc_dev[0] = ADC_USED;
		adc_index = 0;
	} else {
		/*for other adc use bis = 1 */
		if (adc_dev[1] == ADC_USED) {
			return -1;
			/*no wait here */
		}
		adc_dev[1] = ADC_USED;
		adc_index = 1;
	}
	pr_debug("mc13892_adc : request ADC %d\n", adc_index);
	return adc_index;
}

int mc13892_adc_release(int adc_index)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0)))
			return -ERESTARTSYS;
	}

	pr_debug("mc13892_adc : release ADC %d\n", adc_index);
	if ((adc_dev[adc_index] == ADC_MONITORING) ||
	    (adc_dev[adc_index] == ADC_USED)) {
		adc_dev[adc_index] = ADC_FREE;
		wake_up(&queue_adc_busy);
		return 0;
	}
	return -1;
}

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
	case ADC_INIT_P:
		mc13892_adc_init_param(&adc_param_db);
		break;
	case ADC_START:
		mc13892_adc_convert(&adc_param_db);
		break;
	case ADC_TS:
		pmic_adc_get_touch_sample(&ts, 1);
		pr_debug("x = %d\n", ts.x_position);
		pr_debug("y = %d\n", ts.y_position);
		pr_debug("p = %d\n", ts.contact_resistance);
		break;
	case ADC_TS_READ:
		pmic_adc_get_touch_sample(&ts, 0);
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

static int pmic_adc_module_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_debug("PMIC ADC start probe\n");
	ret = device_create_file(&(pdev->dev), &dev_attr_adc);
	if (ret) {
		pr_debug("Can't create device file!\n");
		return -ENODEV;
	}

	init_waitqueue_head(&suspendq);

	ret = pmic_adc_init();
	if (ret != PMIC_SUCCESS) {
		pr_debug("Error in pmic_adc_init.\n");
		goto rm_dev_file;
	}

	pmic_adc_ready = 1;
	pr_debug("PMIC ADC successfully probed\n");
	return 0;

rm_dev_file:
	device_remove_file(&(pdev->dev), &dev_attr_adc);
	return ret;
}

static int pmic_adc_module_remove(struct platform_device *pdev)
{
	pmic_adc_deinit();
	pmic_adc_ready = 0;
	pr_debug("PMIC ADC successfully removed\n");
	return 0;
}

static struct platform_driver pmic_adc_driver_ldm = {
	.driver = {
		   .name = "pmic_adc",
		   },
	.suspend = pmic_adc_suspend,
	.resume = pmic_adc_resume,
	.probe = pmic_adc_module_probe,
	.remove = pmic_adc_module_remove,
};

static int __init pmic_adc_module_init(void)
{
	pr_debug("PMIC ADC driver loading...\n");
	return platform_driver_register(&pmic_adc_driver_ldm);
}

static void __exit pmic_adc_module_exit(void)
{
	platform_driver_unregister(&pmic_adc_driver_ldm);
	pr_debug("PMIC ADC driver successfully unloaded\n");
}

module_init(pmic_adc_module_init);
module_exit(pmic_adc_module_exit);

MODULE_DESCRIPTION("PMIC ADC device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
