/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13783/pmic_adc.c
 * @brief This is the main file of PMIC(mc13783) ADC driver.
 *
 * @ingroup PMIC_ADC
 */

/*
 * Includes
 */

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>

#include "../core/pmic.h"
#include "pmic_adc_defs.h"

#define NB_ADC_REG      5

static int pmic_adc_major;

/* internal function */
static void callback_tsi(void *);
static void callback_adcdone(void *);
static void callback_adcbisdone(void *);
static void callback_adc_comp_high(void *);

/*!
 * Number of users waiting in suspendq
 */
static int swait;

/*!
 * To indicate whether any of the adc devices are suspending
 */
static int suspend_flag;

/*!
 * The suspendq is used by blocking application calls
 */
static wait_queue_head_t suspendq;

static struct class *pmic_adc_class;

/*
 * ADC mc13783 API
 */
/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(pmic_adc_init);
EXPORT_SYMBOL(pmic_adc_deinit);
EXPORT_SYMBOL(pmic_adc_convert);
EXPORT_SYMBOL(pmic_adc_convert_8x);
EXPORT_SYMBOL(pmic_adc_convert_multichnnel);
EXPORT_SYMBOL(pmic_adc_set_touch_mode);
EXPORT_SYMBOL(pmic_adc_get_touch_mode);
EXPORT_SYMBOL(pmic_adc_get_touch_sample);
EXPORT_SYMBOL(pmic_adc_get_battery_current);
EXPORT_SYMBOL(pmic_adc_active_comparator);
EXPORT_SYMBOL(pmic_adc_deactive_comparator);

static DECLARE_COMPLETION(adcdone_it);
static DECLARE_COMPLETION(adcbisdone_it);
static DECLARE_COMPLETION(adc_tsi);
static pmic_event_callback_t tsi_event;
static pmic_event_callback_t event_adc;
static pmic_event_callback_t event_adc_bis;
static pmic_event_callback_t adc_comp_h;
static bool data_ready_adc_1;
static bool data_ready_adc_2;
static bool adc_ts;
static bool wait_ts;
static bool monitor_en;
static bool monitor_adc;
static t_check_mode wcomp_mode;
static DECLARE_MUTEX(convert_mutex);

void (*monitoring_cb) (void);	/*call back to be called when event is detected. */

static DECLARE_WAIT_QUEUE_HEAD(queue_adc_busy);
static t_adc_state adc_dev[2];

static unsigned channel_num[] = {
	0,
	1,
	3,
	4,
	2,
	12,
	13,
	14,
	15,
	-1,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	7,
	6,
	-1,
	-1,
	-1,
	-1,
	5,
	7
};

static bool pmic_adc_ready;

int is_pmic_adc_ready()
{
	return pmic_adc_ready;
}
EXPORT_SYMBOL(is_pmic_adc_ready);


/*!
 * This is the suspend of power management for the mc13783 ADC API.
 * It supports SAVE and POWER_DOWN state.
 *
 * @param        pdev           the device
 * @param        state          the state
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	unsigned int reg_value = 0;
	suspend_flag = 1;
	CHECK_ERROR(pmic_write_reg(REG_ADC_0, DEF_ADC_0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_2, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_3, DEF_ADC_3, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_4, reg_value, PMIC_ALL_BITS));

	return 0;
};

/*!
 * This is the resume of power management for the mc13783 adc API.
 * It supports RESTORE state.
 *
 * @param        pdev           the device
 *
 * @return       This function returns 0 if successful.
 */
static int pmic_adc_resume(struct platform_device *pdev)
{
	/* nothing for mc13783 adc */
	unsigned int adc_0_reg, adc_1_reg;
	suspend_flag = 0;

	/* let interrupt of TSI again */
	adc_0_reg = ADC_WAIT_TSI_0;
	CHECK_ERROR(pmic_write_reg(REG_ADC_0, adc_0_reg, PMIC_ALL_BITS));
	adc_1_reg = ADC_WAIT_TSI_1 | (ADC_BIS * adc_ts);
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, adc_1_reg, PMIC_ALL_BITS));

	while (swait > 0) {
		swait--;
		wake_up_interruptible(&suspendq);
	}

	return 0;
};

/*
 * Call back functions
 */

/*!
 * This is the callback function called on TSI mc13783 event, used in synchronous call.
 */
static void callback_tsi(void *unused)
{
	pr_debug("*** TSI IT mc13783 PMIC_ADC_GET_TOUCH_SAMPLE ***\n");
	if (wait_ts) {
		complete(&adc_tsi);
		pmic_event_mask(EVENT_TSI);
	}
}

/*!
 * This is the callback function called on ADCDone mc13783 event.
 */
static void callback_adcdone(void *unused)
{
	if (data_ready_adc_1) {
		complete(&adcdone_it);
	}
}

/*!
 * This is the callback function called on ADCDone mc13783 event.
 */
static void callback_adcbisdone(void *unused)
{
	pr_debug("* adcdone bis it callback *\n");
	if (data_ready_adc_2) {
		complete(&adcbisdone_it);
	}
}

/*!
 * This is the callback function called on mc13783 event.
 */
static void callback_adc_comp_high(void *unused)
{
	pr_debug("* adc comp it high *\n");
	if (wcomp_mode == CHECK_HIGH || wcomp_mode == CHECK_LOW_OR_HIGH) {
		/* launch callback */
		if (monitoring_cb != NULL) {
			monitoring_cb();
		}
	}
}

/*!
 * This function performs filtering and rejection of excessive noise prone
 * samples.
 *
 * @param        ts_curr     Touch screen value
 *
 * @return       This function returns 0 on success, -1 otherwise.
 */
static int pmic_adc_filter(t_touch_screen *ts_curr)
{
	unsigned int ydiff1, ydiff2, ydiff3, xdiff1, xdiff2, xdiff3;
	unsigned int sample_sumx, sample_sumy;
	static unsigned int prev_x[FILTLEN], prev_y[FILTLEN];
	int index = 0;
	unsigned int y_curr, x_curr;
	static int filt_count;
	/* Added a variable filt_type to decide filtering at run-time */
	unsigned int filt_type = 0;

	if (ts_curr->contact_resistance == 0) {
		ts_curr->x_position = 0;
		ts_curr->y_position = 0;
		filt_count = 0;
		return 0;
	}

	ydiff1 = abs(ts_curr->y_position1 - ts_curr->y_position2);
	ydiff2 = abs(ts_curr->y_position2 - ts_curr->y_position3);
	ydiff3 = abs(ts_curr->y_position1 - ts_curr->y_position3);
	if ((ydiff1 > DELTA_Y_MAX) ||
	    (ydiff2 > DELTA_Y_MAX) || (ydiff3 > DELTA_Y_MAX)) {
		pr_debug("pmic_adc_filter: Ret pos 1\n");
		return -1;
	}

	xdiff1 = abs(ts_curr->x_position1 - ts_curr->x_position2);
	xdiff2 = abs(ts_curr->x_position2 - ts_curr->x_position3);
	xdiff3 = abs(ts_curr->x_position1 - ts_curr->x_position3);

	if ((xdiff1 > DELTA_X_MAX) ||
	    (xdiff2 > DELTA_X_MAX) || (xdiff3 > DELTA_X_MAX)) {
		pr_debug("mc13783_adc_filter: Ret pos 2\n");
		return -1;
	}
	/* Compute two closer values among the three available Y readouts */

	if (ydiff1 < ydiff2) {
		if (ydiff1 < ydiff3) {
			/* Sample 0 & 1 closest together */
			sample_sumy = ts_curr->y_position1 +
			    ts_curr->y_position2;
		} else {
			/* Sample 0 & 2 closest together */
			sample_sumy = ts_curr->y_position1 +
			    ts_curr->y_position3;
		}
	} else {
		if (ydiff2 < ydiff3) {
			/* Sample 1 & 2 closest together */
			sample_sumy = ts_curr->y_position2 +
			    ts_curr->y_position3;
		} else {
			/* Sample 0 & 2 closest together */
			sample_sumy = ts_curr->y_position1 +
			    ts_curr->y_position3;
		}
	}

	/*
	 * Compute two closer values among the three available X
	 * readouts
	 */
	if (xdiff1 < xdiff2) {
		if (xdiff1 < xdiff3) {
			/* Sample 0 & 1 closest together */
			sample_sumx = ts_curr->x_position1 +
			    ts_curr->x_position2;
		} else {
			/* Sample 0 & 2 closest together */
			sample_sumx = ts_curr->x_position1 +
			    ts_curr->x_position3;
		}
	} else {
		if (xdiff2 < xdiff3) {
			/* Sample 1 & 2 closest together */
			sample_sumx = ts_curr->x_position2 +
			    ts_curr->x_position3;
		} else {
			/* Sample 0 & 2 closest together */
			sample_sumx = ts_curr->x_position1 +
			    ts_curr->x_position3;
		}
	}
	/*
	 * Wait FILTER_MIN_DELAY number of samples to restart
	 * filtering
	 */
	if (filt_count < FILTER_MIN_DELAY) {
		/*
		 * Current output is the average of the two closer
		 * values and no filtering is used
		 */
		y_curr = (sample_sumy / 2);
		x_curr = (sample_sumx / 2);
		ts_curr->y_position = y_curr;
		ts_curr->x_position = x_curr;
		filt_count++;
	} else {
		if (abs(sample_sumx - (prev_x[0] + prev_x[1])) >
		    (DELTA_X_MAX * 16)) {
			pr_debug("pmic_adc_filter: : Ret pos 3\n");
			return -1;
		}
		if (abs(sample_sumy - (prev_y[0] + prev_y[1])) >
		    (DELTA_Y_MAX * 16)) {
			return -1;
		}
		sample_sumy /= 2;
		sample_sumx /= 2;
		/* Use hard filtering if the sample difference < 10 */
		if ((abs(sample_sumy - prev_y[0]) > 10) ||
		    (abs(sample_sumx - prev_x[0]) > 10)) {
			filt_type = 1;
		}

		/*
		 * Current outputs are the average of three previous
		 * values and the present readout
		 */
		y_curr = sample_sumy;
		for (index = 0; index < FILTLEN; index++) {
			if (filt_type == 0) {
				y_curr = y_curr + (prev_y[index]);
			} else {
				y_curr = y_curr + (prev_y[index] / 3);
			}
		}
		if (filt_type == 0) {
			y_curr = y_curr >> 2;
		} else {
			y_curr = y_curr >> 1;
		}
		ts_curr->y_position = y_curr;

		x_curr = sample_sumx;
		for (index = 0; index < FILTLEN; index++) {
			if (filt_type == 0) {
				x_curr = x_curr + (prev_x[index]);
			} else {
				x_curr = x_curr + (prev_x[index] / 3);
			}
		}
		if (filt_type == 0) {
			x_curr = x_curr >> 2;
		} else {
			x_curr = x_curr >> 1;
		}
		ts_curr->x_position = x_curr;

	}

	/* Update previous X and Y values */
	for (index = (FILTLEN - 1); index > 0; index--) {
		prev_x[index] = prev_x[index - 1];
		prev_y[index] = prev_y[index - 1];
	}

	/*
	 * Current output will be the most recent past for the
	 * next sample
	 */
	prev_y[0] = y_curr;
	prev_x[0] = x_curr;

	return 0;
}

/*!
 * This function implements the open method on a MC13783 ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_adc_open(struct inode *inode, struct file *file)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}
	pr_debug("mc13783_adc : mc13783_adc_open()\n");
	return 0;
}

/*!
 * This function implements the release method on a MC13783 ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_adc_free(struct inode *inode, struct file *file)
{
	pr_debug("mc13783_adc : mc13783_adc_free()\n");
	return 0;
}

/*!
 * This function initializes all ADC registers with default values. This
 * function also registers the interrupt events.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
int pmic_adc_init(void)
{
	unsigned int reg_value = 0, i = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	for (i = 0; i < ADC_NB_AVAILABLE; i++) {
		adc_dev[i] = ADC_FREE;
	}
	CHECK_ERROR(pmic_write_reg(REG_ADC_0, DEF_ADC_0, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_2, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_3, DEF_ADC_3, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_4, reg_value, PMIC_ALL_BITS));
	reg_value = 0x001000;
	CHECK_ERROR(pmic_write_reg(REG_ARBITRATION_PERIPHERAL_AUDIO, reg_value,
				   0xFFFFFF));

	data_ready_adc_1 = false;
	data_ready_adc_2 = false;
	adc_ts = false;
	wait_ts = false;
	monitor_en = false;
	monitor_adc = false;
	wcomp_mode = CHECK_LOW;
	monitoring_cb = NULL;
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

	/* ADC reading above high limit */
	adc_comp_h.param = NULL;
	adc_comp_h.func = callback_adc_comp_high;
	CHECK_ERROR(pmic_event_subscribe(EVENT_WHIGHI, adc_comp_h));

	return PMIC_SUCCESS;
}

/*!
 * This function disables the ADC, de-registers the interrupt events.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_deinit(void)
{
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_ADCDONEI, event_adc));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_ADCBISDONEI, event_adc_bis));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_TSI, tsi_event));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_WHIGHI, adc_comp_h));

	return PMIC_SUCCESS;
}

/*!
 * This function initializes adc_param structure.
 *
 * @param        adc_param     Structure to be initialized.
 *
 * @return       This function returns 0 if successful.
 */
int mc13783_adc_init_param(t_adc_param *adc_param)
{
	int i = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	adc_param->delay = 0;
	adc_param->conv_delay = false;
	adc_param->single_channel = false;
	adc_param->group = false;
	adc_param->channel_0 = BATTERY_VOLTAGE;
	adc_param->channel_1 = BATTERY_VOLTAGE;
	adc_param->read_mode = 0;
	adc_param->wait_tsi = 0;
	adc_param->chrgraw_devide_5 = true;
	adc_param->read_ts = false;
	adc_param->ts_value.x_position = 0;
	adc_param->ts_value.y_position = 0;
	adc_param->ts_value.contact_resistance = 0;
	for (i = 0; i <= MAX_CHANNEL; i++) {
		adc_param->value[i] = 0;
	}
	return 0;
}

/*!
 * This function starts the convert.
 *
 * @param        adc_param      contains all adc configuration and return value.
 *
 * @return       This function returns 0 if successful.
 */
PMIC_STATUS mc13783_adc_convert(t_adc_param *adc_param)
{
	bool use_bis = false;
	unsigned int adc_0_reg = 0, adc_1_reg = 0, reg_1 = 0, result_reg =
	    0, i = 0;
	unsigned int result = 0, temp = 0;
	pmic_version_t mc13783_ver;
	pr_debug("mc13783 ADC - mc13783_adc_convert ....\n");
	if (suspend_flag == 1) {
		return -EBUSY;
	}

	if (adc_param->wait_tsi) {
		/* we need to set ADCEN 1 for TSI interrupt on mc13783 1.x */
		/* configure adc to wait tsi interrupt */
		INIT_COMPLETION(adc_tsi);
		pr_debug("mc13783 ADC - pmic_write_reg ....\n");
		/*for ts don't use bis */
		adc_0_reg = 0x001c00 | (ADC_BIS * 0);
		pmic_event_unmask(EVENT_TSI);
		CHECK_ERROR(pmic_write_reg
			    (REG_ADC_0, adc_0_reg, PMIC_ALL_BITS));
		/*for ts don't use bis */
		adc_1_reg = 0x200001 | (ADC_BIS * 0);
		CHECK_ERROR(pmic_write_reg
			    (REG_ADC_1, adc_1_reg, PMIC_ALL_BITS));
		pr_debug("wait tsi ....\n");
		wait_ts = true;
		wait_for_completion_interruptible(&adc_tsi);
		wait_ts = false;
	}
	if (adc_param->read_ts == false)
		down(&convert_mutex);
	use_bis = mc13783_adc_request(adc_param->read_ts);
	if (use_bis < 0) {
		pr_debug("process has received a signal and got interrupted\n");
		return -EINTR;
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
		mc13783_ver = pmic_get_version();
		if (mc13783_ver.revision >= 20) {
			if (adc_param->chrgraw_devide_5) {
				adc_0_reg |= ADC_CHRGRAW_D5;
			}
		}
		if (adc_param->single_channel) {
			adc_1_reg |= ADC_SGL_CH;
		}

		if (adc_param->conv_delay) {
			adc_1_reg |= ADC_ATO;
		}

		if (adc_param->group) {
			adc_1_reg |= ADC_ADSEL;
		}

		if (adc_param->single_channel) {
			adc_1_reg |= ADC_SGL_CH;
		}

		adc_1_reg |= (adc_param->channel_0 << ADC_CH_0_POS) &
		    ADC_CH_0_MASK;
		adc_1_reg |= (adc_param->channel_1 << ADC_CH_1_POS) &
		    ADC_CH_1_MASK;
	} else {
		adc_0_reg = 0x003c00 | (ADC_BIS * use_bis) | ADC_INC;
	}
	pr_debug("Write Reg %i = %x\n", REG_ADC_0, adc_0_reg);
	/*Change has been made here */
	CHECK_ERROR(pmic_write_reg(REG_ADC_0, adc_0_reg,
				   ADC_INC | ADC_BIS | ADC_CHRGRAW_D5 |
				   0xfff00ff));
	/* CONFIGURE ADC REG 1 */
	if (adc_param->read_ts == false) {
		adc_1_reg |= ADC_NO_ADTRIG;
		adc_1_reg |= ADC_EN;
		adc_1_reg |= (adc_param->delay << ADC_DELAY_POS) &
		    ADC_DELAY_MASK;
		if (use_bis) {
			adc_1_reg |= ADC_BIS;
		}
	} else {
		/* configure and start convert to read x and y position */
		/* configure to read 2 value in channel selection 1 & 2 */
		adc_1_reg = 0x100409 | (ADC_BIS * use_bis) | ADC_NO_ADTRIG;
	}
	reg_1 = adc_1_reg;
	if (use_bis == 0) {
		data_ready_adc_1 = false;
		adc_1_reg |= ASC_ADC;
		data_ready_adc_1 = true;
		pr_debug("Write Reg %i = %x\n", REG_ADC_1, adc_1_reg);
		INIT_COMPLETION(adcdone_it);
		CHECK_ERROR(pmic_write_reg(REG_ADC_1, adc_1_reg,
					   ADC_SGL_CH | ADC_ATO | ADC_ADSEL
					   | ADC_CH_0_MASK | ADC_CH_1_MASK |
					   ADC_NO_ADTRIG | ADC_EN |
					   ADC_DELAY_MASK | ASC_ADC | ADC_BIS));
		pr_debug("wait adc done \n");
		wait_for_completion_interruptible(&adcdone_it);
		data_ready_adc_1 = false;
	} else {
		data_ready_adc_2 = false;
		adc_1_reg |= ASC_ADC;
		data_ready_adc_2 = true;
		INIT_COMPLETION(adcbisdone_it);
		CHECK_ERROR(pmic_write_reg(REG_ADC_1, adc_1_reg, 0xFFFFFF));
		temp = 0x800000;
		CHECK_ERROR(pmic_write_reg(REG_ADC_3, temp, 0xFFFFFF));
		temp = 0x001000;
		pmic_write_reg(REG_ARBITRATION_PERIPHERAL_AUDIO, temp,
			       0xFFFFFF);
		pr_debug("wait adc done bis\n");
		wait_for_completion_interruptible(&adcbisdone_it);
		data_ready_adc_2 = false;
	}
	/* read result and store in adc_param */
	result = 0;
	if (use_bis == 0) {
		result_reg = REG_ADC_2;
	} else {
		result_reg = REG_ADC_4;
	}
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, 4 << ADC_CH_1_POS,
				   ADC_CH_0_MASK | ADC_CH_1_MASK));

	for (i = 0; i <= 3; i++) {
		CHECK_ERROR(pmic_read_reg(result_reg, &result, PMIC_ALL_BITS));
		pr_debug("result %i = %x\n", result_reg, result);
		adc_param->value[i] = ((result & ADD1_RESULT_MASK) >> 2);
		adc_param->value[i + 4] = ((result & ADD2_RESULT_MASK) >> 14);
	}
	if (adc_param->read_ts) {
		adc_param->ts_value.x_position = adc_param->value[2];
		adc_param->ts_value.x_position1 = adc_param->value[0];
		adc_param->ts_value.x_position2 = adc_param->value[1];
		adc_param->ts_value.x_position3 = adc_param->value[2];
		adc_param->ts_value.y_position1 = adc_param->value[3];
		adc_param->ts_value.y_position2 = adc_param->value[4];
		adc_param->ts_value.y_position3 = adc_param->value[5];
		adc_param->ts_value.y_position = adc_param->value[5];
		adc_param->ts_value.contact_resistance = adc_param->value[6];

	}

	/*if (adc_param->read_ts) {
	   adc_param->ts_value.x_position = adc_param->value[2];
	   adc_param->ts_value.y_position = adc_param->value[5];
	   adc_param->ts_value.contact_resistance = adc_param->value[6];
	   } */
	mc13783_adc_release(use_bis);
	if (adc_param->read_ts == false)
		up(&convert_mutex);

	return PMIC_SUCCESS;
}

/*!
 * This function select the required read_mode for a specific channel.
 *
 * @param        channel   The channel to be sampled
 *
 * @return       This function returns the requires read_mode
 */
t_reading_mode mc13783_set_read_mode(t_channel channel)
{
	t_reading_mode read_mode = 0;

	switch (channel) {
	case LICELL:
		read_mode = M_LITHIUM_CELL;
		break;
	case CHARGE_CURRENT:
		read_mode = M_CHARGE_CURRENT;
		break;
	case BATTERY_CURRENT:
		read_mode = M_BATTERY_CURRENT;
		break;
	case THERMISTOR:
		read_mode = M_THERMISTOR;
		break;
	case DIE_TEMP:
		read_mode = M_DIE_TEMPERATURE;
		break;
	case USB_ID:
		read_mode = M_UID;
		break;
	default:
		read_mode = 0;
	}

	return read_mode;
}

/*!
 * This function triggers a conversion and returns one sampling result of one
 * channel.
 *
 * @param        channel   The channel to be sampled
 * @param        result    The pointer to the conversion result. The memory
 *                         should be allocated by the caller of this function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_convert(t_channel channel, unsigned short *result)
{
	t_adc_param adc_param;
	PMIC_STATUS ret;

	if (suspend_flag == 1) {
		return -EBUSY;
	}

	channel = channel_num[channel];
	if (channel == -1) {
		pr_debug("Wrong channel ID\n");
		return PMIC_PARAMETER_ERROR;
	}
	mc13783_adc_init_param(&adc_param);
	pr_debug("pmic_adc_convert\n");
	adc_param.read_ts = false;
	adc_param.read_mode = mc13783_set_read_mode(channel);

	adc_param.single_channel = true;
	/* Find the group */
	if ((channel >= 0) && (channel <= 7)) {
		adc_param.channel_0 = channel;
		adc_param.group = false;
	} else if ((channel >= 8) && (channel <= 15)) {
		adc_param.channel_0 = channel & 0x07;
		adc_param.group = true;
	} else {
		return PMIC_PARAMETER_ERROR;
	}
	ret = mc13783_adc_convert(&adc_param);
	*result = adc_param.value[0];
	return ret;
}

/*!
 * This function triggers a conversion and returns eight sampling results of
 * one channel.
 *
 * @param        channel   The channel to be sampled
 * @param        result    The pointer to array to store eight sampling results.
 *                         The memory should be allocated by the caller of this
 *                         function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_convert_8x(t_channel channel, unsigned short *result)
{
	t_adc_param adc_param;
	int i;
	PMIC_STATUS ret;
	if (suspend_flag == 1) {
		return -EBUSY;
	}

	channel = channel_num[channel];

	if (channel == -1) {
		pr_debug("Wrong channel ID\n");
		return PMIC_PARAMETER_ERROR;
	}
	mc13783_adc_init_param(&adc_param);
	pr_debug("pmic_adc_convert_8x\n");
	adc_param.read_ts = false;
	adc_param.single_channel = true;
	adc_param.read_mode = mc13783_set_read_mode(channel);
	if ((channel >= 0) && (channel <= 7)) {
		adc_param.channel_0 = channel;
		adc_param.channel_1 = channel;
		adc_param.group = false;
	} else if ((channel >= 8) && (channel <= 15)) {
		adc_param.channel_0 = channel & 0x07;
		adc_param.channel_1 = channel & 0x07;
		adc_param.group = true;
	} else {
		return PMIC_PARAMETER_ERROR;
	}

	ret = mc13783_adc_convert(&adc_param);
	for (i = 0; i <= 7; i++) {
		result[i] = adc_param.value[i];
	}
	return ret;
}

/*!
 * This function triggers a conversion and returns sampling results of each
 * specified channel.
 *
 * @param        channels  This input parameter is bitmap to specify channels
 *                         to be sampled.
 * @param        result    The pointer to array to store sampling results.
 *                         The memory should be allocated by the caller of this
 *                         function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_convert_multichnnel(t_channel channels,
					 unsigned short *result)
{
	t_adc_param adc_param;
	int i;
	PMIC_STATUS ret;
	if (suspend_flag == 1) {
		return -EBUSY;
	}
	mc13783_adc_init_param(&adc_param);
	pr_debug("pmic_adc_convert_multichnnel\n");

	channels = channel_num[channels];

	if (channels == -1) {
		pr_debug("Wrong channel ID\n");
		return PMIC_PARAMETER_ERROR;
	}

	adc_param.read_ts = false;
	adc_param.single_channel = false;
	if ((channels >= 0) && (channels <= 7)) {
		adc_param.channel_0 = channels;
		adc_param.channel_1 = ((channels + 4) % 4) + 4;
		adc_param.group = false;
	} else if ((channels >= 8) && (channels <= 15)) {
		channels = channels & 0x07;
		adc_param.channel_0 = channels;
		adc_param.channel_1 = ((channels + 4) % 4) + 4;
		adc_param.group = true;
	} else {
		return PMIC_PARAMETER_ERROR;
	}
	adc_param.read_mode = 0x00003f;
	adc_param.read_ts = false;
	ret = mc13783_adc_convert(&adc_param);

	for (i = 0; i <= 7; i++) {
		result[i] = adc_param.value[i];
	}
	return ret;
}

/*!
 * This function sets touch screen operation mode.
 *
 * @param        touch_mode   Touch screen operation mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_set_touch_mode(t_touch_mode touch_mode)
{
	if (suspend_flag == 1) {
		return -EBUSY;
	}
	CHECK_ERROR(pmic_write_reg(REG_ADC_0,
				   BITFVAL(MC13783_ADC0_TS_M, touch_mode),
				   BITFMASK(MC13783_ADC0_TS_M)));
	return PMIC_SUCCESS;
}

/*!
 * This function retrieves the current touch screen operation mode.
 *
 * @param        touch_mode   Pointer to the retrieved touch screen operation
 *                            mode.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_get_touch_mode(t_touch_mode *touch_mode)
{
	unsigned int value;
	if (suspend_flag == 1) {
		return -EBUSY;
	}
	CHECK_ERROR(pmic_read_reg(REG_ADC_0, &value, PMIC_ALL_BITS));

	*touch_mode = BITFEXT(value, MC13783_ADC0_TS_M);

	return PMIC_SUCCESS;
}

/*!
 * This function retrieves the current touch screen (X,Y) coordinates.
 *
 * @param        touch_sample Pointer to touch sample.
 * @param        wait	indicates whether this call must block or not.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_get_touch_sample(t_touch_screen *touch_sample, int wait)
{
	if (mc13783_adc_read_ts(touch_sample, wait) != 0)
		return PMIC_ERROR;
	if (0 == pmic_adc_filter(touch_sample))
		return PMIC_SUCCESS;
	else
		return PMIC_ERROR;
}

/*!
 * This function read the touch screen value.
 *
 * @param        ts_value    return value of touch screen
 * @param        wait_tsi    if true, this function is synchronous (wait in TSI event).
 *
 * @return       This function returns 0.
 */
PMIC_STATUS mc13783_adc_read_ts(t_touch_screen *ts_value, int wait_tsi)
{
	t_adc_param param;
	pr_debug("mc13783_adc : mc13783_adc_read_ts\n");
	if (suspend_flag == 1) {
		return -EBUSY;
	}
	if (wait_ts) {
		pr_debug("mc13783_adc : error TS busy \n");
		return PMIC_ERROR;
	}
	mc13783_adc_init_param(&param);
	param.wait_tsi = wait_tsi;
	param.read_ts = true;
	if (mc13783_adc_convert(&param) != 0)
		return PMIC_ERROR;
	/* check if x-y is ok */
	if ((param.ts_value.x_position1 < TS_X_MAX) &&
	    (param.ts_value.x_position1 >= TS_X_MIN) &&
	    (param.ts_value.y_position1 < TS_Y_MAX) &&
	    (param.ts_value.y_position1 >= TS_Y_MIN) &&
	    (param.ts_value.x_position2 < TS_X_MAX) &&
	    (param.ts_value.x_position2 >= TS_X_MIN) &&
	    (param.ts_value.y_position2 < TS_Y_MAX) &&
	    (param.ts_value.y_position2 >= TS_Y_MIN) &&
	    (param.ts_value.x_position3 < TS_X_MAX) &&
	    (param.ts_value.x_position3 >= TS_X_MIN) &&
	    (param.ts_value.y_position3 < TS_Y_MAX) &&
	    (param.ts_value.y_position3 >= TS_Y_MIN)) {
		ts_value->x_position = param.ts_value.x_position;
		ts_value->x_position1 = param.ts_value.x_position1;
		ts_value->x_position2 = param.ts_value.x_position2;
		ts_value->x_position3 = param.ts_value.x_position3;
		ts_value->y_position = param.ts_value.y_position;
		ts_value->y_position1 = param.ts_value.y_position1;
		ts_value->y_position2 = param.ts_value.y_position2;
		ts_value->y_position3 = param.ts_value.y_position3;
		ts_value->contact_resistance =
		    param.ts_value.contact_resistance + 1;

	} else {
		ts_value->x_position = 0;
		ts_value->y_position = 0;
		ts_value->contact_resistance = 0;

	}
	return PMIC_SUCCESS;
}

/*!
 * This function starts a Battery Current mode conversion.
 *
 * @param        mode      Conversion mode.
 * @param        result    Battery Current measurement result.
 *                         if \a mode = ADC_8CHAN_1X, the result is \n
 *                             result[0] = (BATTP - BATT_I) \n
 *                         if \a mode = ADC_1CHAN_8X, the result is \n
 *                             result[0] = BATTP \n
 *                             result[1] = BATT_I \n
 *                             result[2] = BATTP \n
 *                             result[3] = BATT_I \n
 *                             result[4] = BATTP \n
 *                             result[5] = BATT_I \n
 *                             result[6] = BATTP \n
 *                             result[7] = BATT_I
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_get_battery_current(t_conversion_mode mode,
					 unsigned short *result)
{
	PMIC_STATUS ret;
	t_channel channel;
	if (suspend_flag == 1) {
		return -EBUSY;
	}
	channel = BATTERY_CURRENT;
	if (mode == ADC_8CHAN_1X) {
		ret = pmic_adc_convert(channel, result);
	} else {
		ret = pmic_adc_convert_8x(channel, result);
	}
	return ret;
}

/*!
 * This function request a ADC.
 *
 * @return      This function returns index of ADC to be used (0 or 1) if successful.
 * return -1 if error.
 */
int mc13783_adc_request(bool read_ts)
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
	pr_debug("mc13783_adc : request ADC %d\n", adc_index);
	return adc_index;
}

/*!
 * This function release an ADC.
 *
 * @param        adc_index     index of ADC to be released.
 *
 * @return       This function returns 0 if successful.
 */
int mc13783_adc_release(int adc_index)
{
	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}

	pr_debug("mc13783_adc : release ADC %d\n", adc_index);
	if ((adc_dev[adc_index] == ADC_MONITORING) ||
	    (adc_dev[adc_index] == ADC_USED)) {
		adc_dev[adc_index] = ADC_FREE;
		wake_up(&queue_adc_busy);
		return 0;
	}
	return -1;
}

/*!
 * This function initializes monitoring structure.
 *
 * @param        monitor     Structure to be initialized.
 *
 * @return       This function returns 0 if successful.
 */
int mc13783_adc_init_monitor_param(t_monitoring_param *monitor)
{
	pr_debug("mc13783_adc : init monitor\n");
	monitor->delay = 0;
	monitor->conv_delay = false;
	monitor->channel = BATTERY_VOLTAGE;
	monitor->read_mode = 0;
	monitor->comp_low = 0;
	monitor->comp_high = 0;
	monitor->group = 0;
	monitor->check_mode = CHECK_LOW_OR_HIGH;
	monitor->callback = NULL;
	return 0;
}

/*!
 * This function actives the comparator.  When comparator is active and ADC
 * is enabled, the 8th converted value will be digitally compared against the
 * window defined by WLOW and WHIGH registers.
 *
 * @param        low      Comparison window low threshold (WLOW).
 * @param        high     Comparison window high threshold (WHIGH).
 * @param        channel  The channel to be sampled
 * @param        callback Callback function to be called when the converted
 *                        value is beyond the comparison window.  The callback
 *                        function will pass a parameter of type
 *                        \b t_comp_expection to indicate the reason of
 *                        comparator exception.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_active_comparator(unsigned char low,
				       unsigned char high,
				       t_channel channel,
				       t_comparator_cb callback)
{
	bool use_bis = false;
	unsigned int adc_0_reg = 0, adc_1_reg = 0, adc_3_reg = 0;
	t_monitoring_param monitoring;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	if (monitor_en) {
		pr_debug("mc13783_adc : monitoring already configured\n");
		return PMIC_ERROR;
	}
	monitor_en = true;
	mc13783_adc_init_monitor_param(&monitoring);
	monitoring.comp_low = low;
	monitoring.comp_high = high;
	monitoring.channel = channel;
	monitoring.callback = (void *)callback;

	use_bis = mc13783_adc_request(false);
	if (use_bis < 0) {
		pr_debug("mc13783_adc : request error\n");
		return PMIC_ERROR;
	}
	monitor_adc = use_bis;

	adc_0_reg = 0;

	/* TO DO ADOUT CONFIGURE */
	adc_0_reg = monitoring.read_mode & ADC_MODE_MASK;
	if (use_bis) {
		/* add adc bis */
		adc_0_reg |= ADC_BIS;
	}
	adc_0_reg |= ADC_WCOMP;

	/* CONFIGURE ADC REG 1 */
	adc_1_reg = 0;
	adc_1_reg |= ADC_EN;
	if (monitoring.conv_delay) {
		adc_1_reg |= ADC_ATO;
	}
	if (monitoring.group) {
		adc_1_reg |= ADC_ADSEL;
	}
	adc_1_reg |= (monitoring.channel << ADC_CH_0_POS) & ADC_CH_0_MASK;
	adc_1_reg |= (monitoring.delay << ADC_DELAY_POS) & ADC_DELAY_MASK;
	if (use_bis) {
		adc_1_reg |= ADC_BIS;
	}

	adc_3_reg |= (monitoring.comp_high << ADC_WCOMP_H_POS) &
	    ADC_WCOMP_H_MASK;
	adc_3_reg |= (monitoring.comp_low << ADC_WCOMP_L_POS) &
	    ADC_WCOMP_L_MASK;
	if (use_bis) {
		adc_3_reg |= ADC_BIS;
	}

	wcomp_mode = monitoring.check_mode;
	/* call back to be called when event is detected. */
	monitoring_cb = monitoring.callback;

	CHECK_ERROR(pmic_write_reg(REG_ADC_0, adc_0_reg, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, adc_1_reg, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_3, adc_3_reg, PMIC_ALL_BITS));
	return PMIC_SUCCESS;
}

/*!
 * This function deactivates the comparator.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_adc_deactive_comparator(void)
{
	unsigned int reg_value = 0;

	if (suspend_flag == 1) {
		return -EBUSY;
	}
	if (!monitor_en) {
		pr_debug("mc13783_adc : adc monitoring free\n");
		return PMIC_ERROR;
	}

	if (monitor_en) {
		reg_value = ADC_BIS;
	}

	/* clear all reg value */
	CHECK_ERROR(pmic_write_reg(REG_ADC_0, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_1, reg_value, PMIC_ALL_BITS));
	CHECK_ERROR(pmic_write_reg(REG_ADC_3, reg_value, PMIC_ALL_BITS));

	reg_value = 0;

	if (monitor_adc) {
		CHECK_ERROR(pmic_write_reg
			    (REG_ADC_4, reg_value, PMIC_ALL_BITS));
	} else {
		CHECK_ERROR(pmic_write_reg
			    (REG_ADC_2, reg_value, PMIC_ALL_BITS));
	}

	mc13783_adc_release(monitor_adc);
	monitor_en = false;
	return PMIC_SUCCESS;
}

/*!
 * This function implements IOCTL controls on a MC13783 ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int pmic_adc_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	t_adc_convert_param *convert_param;
	t_touch_mode touch_mode;
	t_touch_screen touch_sample;
	unsigned short b_current;
	t_adc_comp_param *comp_param;
	if ((_IOC_TYPE(cmd) != 'p') && (_IOC_TYPE(cmd) != 'D'))
		return -ENOTTY;

	while (suspend_flag == 1) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, (suspend_flag == 0))) {
			return -ERESTARTSYS;
		}
	}

	convert_param = kmalloc(sizeof(t_adc_convert_param), GFP_KERNEL);
	comp_param = kmalloc(sizeof(t_adc_comp_param), GFP_KERNEL);
	switch (cmd) {
	case PMIC_ADC_INIT:
		pr_debug("init adc\n");
		CHECK_ERROR(pmic_adc_init());
		break;

	case PMIC_ADC_DEINIT:
		pr_debug("deinit adc\n");
		CHECK_ERROR(pmic_adc_deinit());
		break;

	case PMIC_ADC_CONVERT:
		if (convert_param  == NULL)
			return -ENOMEM;

		if (copy_from_user(convert_param, (t_adc_convert_param *) arg,
				   sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_adc_convert(convert_param->channel,
						   convert_param->result),
				  (kfree(convert_param)));

		if (copy_to_user((t_adc_convert_param *) arg, convert_param,
				 sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		kfree(convert_param);
		break;

	case PMIC_ADC_CONVERT_8X:
		if (convert_param == NULL)
			return -ENOMEM;

		if (copy_from_user(convert_param, (t_adc_convert_param *) arg,
				   sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_adc_convert_8x(convert_param->channel,
						      convert_param->result),
				  (kfree(convert_param)));

		if (copy_to_user((t_adc_convert_param *) arg, convert_param,
				 sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		kfree(convert_param);
		break;

	case PMIC_ADC_CONVERT_MULTICHANNEL:
		if (convert_param == NULL)
			return -ENOMEM;

		if (copy_from_user(convert_param, (t_adc_convert_param *) arg,
				   sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}

		CHECK_ERROR_KFREE(pmic_adc_convert_multichnnel
				  (convert_param->channel,
				   convert_param->result),
				  (kfree(convert_param)));

		if (copy_to_user((t_adc_convert_param *) arg, convert_param,
				 sizeof(t_adc_convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		kfree(convert_param);
		break;

	case PMIC_ADC_SET_TOUCH_MODE:
		CHECK_ERROR(pmic_adc_set_touch_mode((t_touch_mode) arg));
		break;

	case PMIC_ADC_GET_TOUCH_MODE:
		CHECK_ERROR(pmic_adc_get_touch_mode(&touch_mode));
		if (copy_to_user((t_touch_mode *) arg, &touch_mode,
				 sizeof(t_touch_mode))) {
			return -EFAULT;
		}
		break;

	case PMIC_ADC_GET_TOUCH_SAMPLE:
		pr_debug("pmic_adc_ioctl: " "PMIC_ADC_GET_TOUCH_SAMPLE\n");
		CHECK_ERROR(pmic_adc_get_touch_sample(&touch_sample, 1));
		if (copy_to_user((t_touch_screen *) arg, &touch_sample,
				 sizeof(t_touch_screen))) {
			return -EFAULT;
		}
		break;

	case PMIC_ADC_GET_BATTERY_CURRENT:
		CHECK_ERROR(pmic_adc_get_battery_current(ADC_8CHAN_1X,
							 &b_current));
		if (copy_to_user((unsigned short *)arg, &b_current,
				 sizeof(unsigned short))) {

			return -EFAULT;
		}
		break;

	case PMIC_ADC_ACTIVATE_COMPARATOR:
		if (comp_param == NULL)
			return -ENOMEM;

		if (copy_from_user(comp_param, (t_adc_comp_param *) arg,
				   sizeof(t_adc_comp_param))) {
			kfree(comp_param);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(pmic_adc_active_comparator(comp_param->wlow,
							     comp_param->whigh,
							     comp_param->
							     channel,
							     comp_param->
							     callback),
				  (kfree(comp_param)));
		break;

	case PMIC_ADC_DEACTIVE_COMPARATOR:
		CHECK_ERROR(pmic_adc_deactive_comparator());
		break;

	default:
		pr_debug("pmic_adc_ioctl: unsupported ioctl command 0x%x\n",
			 cmd);
		return -EINVAL;
	}
	return 0;
}

static struct file_operations mc13783_adc_fops = {
	.owner = THIS_MODULE,
	.ioctl = pmic_adc_ioctl,
	.open = pmic_adc_open,
	.release = pmic_adc_free,
};

static int pmic_adc_module_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	pmic_adc_major = register_chrdev(0, "pmic_adc", &mc13783_adc_fops);

	if (pmic_adc_major < 0) {
		pr_debug(KERN_ERR "Unable to get a major for pmic_adc\n");
		return pmic_adc_major;
	}
	init_waitqueue_head(&suspendq);

	pmic_adc_class = class_create(THIS_MODULE, "pmic_adc");
	if (IS_ERR(pmic_adc_class)) {
		pr_debug(KERN_ERR "Error creating pmic_adc class.\n");
		ret = PTR_ERR(pmic_adc_class);
		goto err_out1;
	}

	temp_class = device_create(pmic_adc_class, NULL,
				   MKDEV(pmic_adc_major, 0), NULL, "pmic_adc");
	if (IS_ERR(temp_class)) {
		pr_debug(KERN_ERR "Error creating pmic_adc class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	ret = pmic_adc_init();
	if (ret != PMIC_SUCCESS) {
		pr_debug(KERN_ERR "Error in pmic_adc_init.\n");
		goto err_out4;
	}

	pmic_adc_ready = 1;
	pr_debug(KERN_INFO "PMIC ADC successfully probed\n");
	return ret;

      err_out4:
	device_destroy(pmic_adc_class, MKDEV(pmic_adc_major, 0));
      err_out2:
	class_destroy(pmic_adc_class);
      err_out1:
	unregister_chrdev(pmic_adc_major, "pmic_adc");
	return ret;
}

static int pmic_adc_module_remove(struct platform_device *pdev)
{
	pmic_adc_ready = 0;
	pmic_adc_deinit();
	device_destroy(pmic_adc_class, MKDEV(pmic_adc_major, 0));
	class_destroy(pmic_adc_class);
	unregister_chrdev(pmic_adc_major, "pmic_adc");
	pr_debug(KERN_INFO "PMIC ADC successfully removed\n");
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

/*
 * Initialization and Exit
 */
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

/*
 * Module entry points
 */

module_init(pmic_adc_module_init);
module_exit(pmic_adc_module_exit);

MODULE_DESCRIPTION("PMIC ADC device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
