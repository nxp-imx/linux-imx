/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file adc/imx_adc.c
 * @brief This is the main file of i.MX ADC driver.
 *
 * @ingroup IMX_ADC
 */

/*
 * Includes
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/imx_adc.h>
#include "imx_adc_reg.h"

static int imx_adc_major;

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
static wait_queue_head_t tsq;

static bool imx_adc_ready;
static bool ts_data_ready;
static int tsi_data = TSI_DATA;
static unsigned short ts_data_buf[16];

static struct class *imx_adc_class;
static struct imx_adc_data *adc_data;

static DECLARE_MUTEX(general_convert_mutex);
static DECLARE_MUTEX(ts_convert_mutex);

unsigned long tsc_base;

int is_imx_adc_ready(void)
{
	return imx_adc_ready;
}
EXPORT_SYMBOL(is_imx_adc_ready);

void tsc_clk_enable(void)
{
	unsigned long reg;

	clk_enable(adc_data->adc_clk);

	reg = __raw_readl(tsc_base + TGCR);
	reg |= TGCR_IPG_CLK_EN;
	__raw_writel(reg, tsc_base + TGCR);
}

void tsc_clk_disable(void)
{
	unsigned long reg;

	clk_disable(adc_data->adc_clk);

	reg = __raw_readl(tsc_base + TGCR);
	reg &= ~TGCR_IPG_CLK_EN;
	__raw_writel(reg, tsc_base + TGCR);
}

void tsc_self_reset(void)
{
	unsigned long reg;

	reg = __raw_readl(tsc_base + TGCR);
	reg |= TGCR_TSC_RST;
	__raw_writel(reg, tsc_base + TGCR);

	while (__raw_readl(tsc_base + TGCR) & TGCR_TSC_RST)
		continue;
}

/* Internal reference */
void tsc_intref_enable(void)
{
	unsigned long reg;

	reg = __raw_readl(tsc_base + TGCR);
	reg |= TGCR_INTREFEN;
	__raw_writel(reg, tsc_base + TGCR);
}

/* initialize touchscreen */
void imx_tsc_init(void)
{
	unsigned long reg;
	int lastitemid;

	/* Level sense */
	reg = __raw_readl(tsc_base + TCQCR);
	reg &= ~CQCR_PD_CFG;  /* edge sensitive */
	reg |= (0xf << CQCR_FIFOWATERMARK_SHIFT);  /* watermark */
	__raw_writel(reg, tsc_base + TCQCR);

	/* Configure 4-wire */
	reg = TSC_4WIRE_PRECHARGE;
	reg |= CC_IGS;
	__raw_writel(reg, tsc_base + TCC0);

	reg = TSC_4WIRE_TOUCH_DETECT;
	reg |= 3 << CC_NOS_SHIFT;	/* 4 samples */
	reg |= 32 << CC_SETTLING_TIME_SHIFT;	/* it's important! */
	__raw_writel(reg, tsc_base + TCC1);

	reg = TSC_4WIRE_X_MEASUMENT;
	reg |= 3 << CC_NOS_SHIFT;	/* 4 samples */
	reg |= 16 << CC_SETTLING_TIME_SHIFT;	/* settling time */
	__raw_writel(reg, tsc_base + TCC2);

	reg = TSC_4WIRE_Y_MEASUMENT;
	reg |= 3 << CC_NOS_SHIFT;	/* 4 samples */
	reg |= 16 << CC_SETTLING_TIME_SHIFT;	/* settling time */
	__raw_writel(reg, tsc_base + TCC3);

	reg = (TCQ_ITEM_TCC0 << TCQ_ITEM7_SHIFT) |
	      (TCQ_ITEM_TCC0 << TCQ_ITEM6_SHIFT) |
	      (TCQ_ITEM_TCC1 << TCQ_ITEM5_SHIFT) |
	      (TCQ_ITEM_TCC0 << TCQ_ITEM4_SHIFT) |
	      (TCQ_ITEM_TCC3 << TCQ_ITEM3_SHIFT) |
	      (TCQ_ITEM_TCC2 << TCQ_ITEM2_SHIFT) |
	      (TCQ_ITEM_TCC1 << TCQ_ITEM1_SHIFT) |
	      (TCQ_ITEM_TCC0 << TCQ_ITEM0_SHIFT);
	__raw_writel(reg, tsc_base + TCQ_ITEM_7_0);

	lastitemid = 5;
	reg = __raw_readl(tsc_base + TCQCR);
	reg = (reg & ~CQCR_LAST_ITEM_ID_MASK) |
	      (lastitemid << CQCR_LAST_ITEM_ID_SHIFT);
	__raw_writel(reg, tsc_base + TCQCR);

	/* Config idle for 4-wire */
	reg = TSC_4WIRE_PRECHARGE;
	__raw_writel(reg, tsc_base + TICR);

	reg = TSC_4WIRE_TOUCH_DETECT;
	__raw_writel(reg, tsc_base + TICR);

	/* pen down mask */
	reg = __raw_readl(tsc_base + TCQCR);
	reg &= ~CQCR_PD_MSK;
	__raw_writel(reg, tsc_base + TCQCR);
	reg = __raw_readl(tsc_base + TCQMR);
	reg &= ~TCQMR_PD_IRQ_MSK;
	__raw_writel(reg, tsc_base + TCQMR);

	/* Debounce time = dbtime*8 adc clock cycles */
	reg = __raw_readl(tsc_base + TGCR);
	reg &= ~TGCR_PDBTIME_MASK;
	reg |= TGCR_PDBTIME128 | TGCR_HSYNC_EN;
	__raw_writel(reg, tsc_base + TGCR);

	/* pen down enable */
	reg = __raw_readl(tsc_base + TGCR);
	reg |= TGCR_PDB_EN;
	__raw_writel(reg, tsc_base + TGCR);
	reg |= TGCR_PD_EN;
	__raw_writel(reg, tsc_base + TGCR);
}

static irqreturn_t imx_adc_interrupt(int irq, void *dev_id)
{
	unsigned long reg;

	if (__raw_readl(tsc_base + TGSR) & 0x4) {
		/* deep sleep wakeup interrupt */
		/* clear tgsr */
		__raw_writel(0,  tsc_base + TGSR);
		/* clear deep sleep wakeup irq */
		reg = __raw_readl(tsc_base + TGCR);
		reg &= ~TGCR_SLPC;
		__raw_writel(reg, tsc_base + TGCR);
		/* un-mask pen down and pen down irq */
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_PD_MSK;
		__raw_writel(reg, tsc_base + TCQCR);
		reg = __raw_readl(tsc_base + TCQMR);
		reg &= ~TCQMR_PD_IRQ_MSK;
		__raw_writel(reg, tsc_base + TCQMR);
	} else if ((__raw_readl(tsc_base + TGSR) & 0x1) &&
		   (__raw_readl(tsc_base + TCQSR) & 0x1)) {

		/* mask pen down detect irq */
		reg = __raw_readl(tsc_base + TCQMR);
		reg |= TCQMR_PD_IRQ_MSK;
		__raw_writel(reg, tsc_base + TCQMR);

		ts_data_ready = 1;
		wake_up_interruptible(&tsq);
	}
	return IRQ_HANDLED;
}

enum IMX_ADC_STATUS imx_adc_read_general(unsigned short *result)
{
	unsigned long reg;
	unsigned int data_num = 0;

	reg = __raw_readl(tsc_base + GCQCR);
	reg |= CQCR_FQS;
	__raw_writel(reg, tsc_base + GCQCR);

	while (!(__raw_readl(tsc_base + GCQSR) & CQSR_EOQ))
		continue;
	reg = __raw_readl(tsc_base + GCQCR);
	reg &= ~CQCR_FQS;
	__raw_writel(reg, tsc_base + GCQCR);
	reg = __raw_readl(tsc_base + GCQSR);
	reg |= CQSR_EOQ;
	__raw_writel(reg, tsc_base + GCQSR);

	while (!(__raw_readl(tsc_base + GCQSR) & CQSR_EMPT)) {
		result[data_num] = __raw_readl(tsc_base + GCQFIFO) >>
				 GCQFIFO_ADCOUT_SHIFT;
		data_num++;
	}
	return IMX_ADC_SUCCESS;
}

/*!
 * This function will get raw (X,Y) value by converting the voltage
 * @param        touch_sample Pointer to touch sample
 *
 * return        This funciton returns 0 if successful.
 *
 *
 */
enum IMX_ADC_STATUS imx_adc_read_ts(struct t_touch_screen *touch_sample,
				    int wait_tsi)
{
	unsigned long reg;
	int data_num = 0;
	int detect_sample1, detect_sample2;

	memset(ts_data_buf, 0, sizeof ts_data_buf);
	touch_sample->valid_flag = 1;

	if (wait_tsi) {
		/* Config idle for 4-wire */
		reg = TSC_4WIRE_TOUCH_DETECT;
		__raw_writel(reg, tsc_base + TICR);

		/* Pen interrupt starts new conversion queue */
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_QSM_MASK;
		reg |= CQCR_QSM_PEN;
		__raw_writel(reg, tsc_base + TCQCR);

		/* unmask pen down detect irq */
		reg = __raw_readl(tsc_base + TCQMR);
		reg &= ~TCQMR_PD_IRQ_MSK;
		__raw_writel(reg, tsc_base + TCQMR);

		wait_event_interruptible(tsq, ts_data_ready);
		while (!(__raw_readl(tsc_base + TCQSR) & CQSR_EOQ))
			continue;

		/* stop the conversion */
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_QSM_MASK;
		__raw_writel(reg, tsc_base + TCQCR);
		reg = CQSR_PD | CQSR_EOQ;
		__raw_writel(reg, tsc_base + TCQSR);

		/* change configuration for FQS mode */
		tsi_data = TSI_DATA;
		reg = (0x1 << CC_YPLLSW_SHIFT) | (0x1 << CC_XNURSW_SHIFT) |
		      CC_XPULSW;
		__raw_writel(reg, tsc_base + TICR);
	} else {
		/* FQS semaphore */
		down(&ts_convert_mutex);

		reg = (0x1 << CC_YPLLSW_SHIFT) | (0x1 << CC_XNURSW_SHIFT) |
		      CC_XPULSW;
		__raw_writel(reg, tsc_base + TICR);

		/* FQS */
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_QSM_MASK;
		reg |= CQCR_QSM_FQS;
		__raw_writel(reg, tsc_base + TCQCR);
		reg = __raw_readl(tsc_base + TCQCR);
		reg |= CQCR_FQS;
		__raw_writel(reg, tsc_base + TCQCR);
		while (!(__raw_readl(tsc_base + TCQSR) & CQSR_EOQ))
			continue;

		/* stop FQS */
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_QSM_MASK;
		__raw_writel(reg, tsc_base + TCQCR);
		reg = __raw_readl(tsc_base + TCQCR);
		reg &= ~CQCR_FQS;
		__raw_writel(reg, tsc_base + TCQCR);

		/* clear status bit */
		reg = __raw_readl(tsc_base + TCQSR);
		reg |= CQSR_EOQ;
		__raw_writel(reg, tsc_base + TCQSR);
		tsi_data = FQS_DATA;

		/* Config idle for 4-wire */
		reg = TSC_4WIRE_PRECHARGE;
		__raw_writel(reg, tsc_base + TICR);

		reg = TSC_4WIRE_TOUCH_DETECT;
		__raw_writel(reg, tsc_base + TICR);

	}

	while (!(__raw_readl(tsc_base + TCQSR) & CQSR_EMPT)) {
		reg = __raw_readl(tsc_base + TCQFIFO);
		ts_data_buf[data_num] = reg;
		data_num++;
	}

	touch_sample->x_position1 = ts_data_buf[4] >> 4;
	touch_sample->x_position2 = ts_data_buf[5] >> 4;
	touch_sample->x_position3 = ts_data_buf[6] >> 4;
	touch_sample->y_position1 = ts_data_buf[9] >> 4;
	touch_sample->y_position2 = ts_data_buf[10] >> 4;
	touch_sample->y_position3 = ts_data_buf[11] >> 4;

	detect_sample1 = ts_data_buf[0];
	detect_sample2 = ts_data_buf[12];

	if ((detect_sample1 > 0x6000) || (detect_sample2 > 0x6000))
		touch_sample->valid_flag = 0;

	ts_data_ready = 0;

	if (!(touch_sample->x_position1 ||
	      touch_sample->x_position2 || touch_sample->x_position3))
		touch_sample->contact_resistance = 0;
	else
		touch_sample->contact_resistance = 1;

	if (tsi_data == FQS_DATA)
		up(&ts_convert_mutex);
	return IMX_ADC_SUCCESS;
}

/*!
 * This function performs filtering and rejection of excessive noise prone
 * sampl.
 *
 * @param        ts_curr     Touch screen value
 *
 * @return       This function returns 0 on success, -1 otherwise.
 */
static int imx_adc_filter(struct t_touch_screen *ts_curr)
{

	unsigned int ydiff1, ydiff2, ydiff3, xdiff1, xdiff2, xdiff3;
	unsigned int sample_sumx, sample_sumy;
	static unsigned int prev_x[FILTLEN], prev_y[FILTLEN];
	int index = 0;
	unsigned int y_curr, x_curr;
	static int filt_count;
	/* Added a variable filt_type to decide filtering at run-time */
	unsigned int filt_type = 0;

	/* ignore the data converted when pen down and up */
	if ((ts_curr->contact_resistance == 0) || tsi_data == TSI_DATA) {
		ts_curr->x_position = 0;
		ts_curr->y_position = 0;
		filt_count = 0;
		return 0;
	}
	/* ignore the data valid */
	if (ts_curr->valid_flag == 0)
		return -1;

	ydiff1 = abs(ts_curr->y_position1 - ts_curr->y_position2);
	ydiff2 = abs(ts_curr->y_position2 - ts_curr->y_position3);
	ydiff3 = abs(ts_curr->y_position1 - ts_curr->y_position3);
	if ((ydiff1 > DELTA_Y_MAX) ||
	    (ydiff2 > DELTA_Y_MAX) || (ydiff3 > DELTA_Y_MAX)) {
		pr_debug("imx_adc_filter: Ret pos 1\n");
		return -1;
	}

	xdiff1 = abs(ts_curr->x_position1 - ts_curr->x_position2);
	xdiff2 = abs(ts_curr->x_position2 - ts_curr->x_position3);
	xdiff3 = abs(ts_curr->x_position1 - ts_curr->x_position3);

	if ((xdiff1 > DELTA_X_MAX) ||
	    (xdiff2 > DELTA_X_MAX) || (xdiff3 > DELTA_X_MAX)) {
		pr_debug("imx_adc_filter: Ret pos 2\n");
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
			pr_debug("imx_adc_filter: : Ret pos 3\n");
			return -1;
		}
		if (abs(sample_sumy - (prev_y[0] + prev_y[1])) >
		    (DELTA_Y_MAX * 16)) {
			pr_debug("imx_adc_filter: : Ret pos 4\n");
			return -1;
		}
		sample_sumy /= 2;
		sample_sumx /= 2;
		/* Use hard filtering if the sample difference < 10 */
		if ((abs(sample_sumy - prev_y[0]) > 10) ||
		    (abs(sample_sumx - prev_x[0]) > 10))
			filt_type = 1;

		/*
		 * Current outputs are the average of three previous
		 * values and the present readout
		 */
		y_curr = sample_sumy;
		for (index = 0; index < FILTLEN; index++) {
			if (filt_type == 0)
				y_curr = y_curr + (prev_y[index]);
			else
				y_curr = y_curr + (prev_y[index] / 3);
		}
		if (filt_type == 0)
			y_curr = y_curr >> 2;
		else
			y_curr = y_curr >> 1;
		ts_curr->y_position = y_curr;

		x_curr = sample_sumx;
		for (index = 0; index < FILTLEN; index++) {
			if (filt_type == 0)
				x_curr = x_curr + (prev_x[index]);
			else
				x_curr = x_curr + (prev_x[index] / 3);
		}
		if (filt_type == 0)
			x_curr = x_curr >> 2;
		else
			x_curr = x_curr >> 1;
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
 * This function retrieves the current touch screen (X,Y) coordinates.
 *
 * @param        touch_sample Pointer to touch sample.
 *
 * @return       This function returns IMX_ADC_SUCCESS if successful.
 */
enum IMX_ADC_STATUS imx_adc_get_touch_sample(struct t_touch_screen
					     *touch_sample, int wait_tsi)
{
	if (imx_adc_read_ts(touch_sample, wait_tsi))
		return IMX_ADC_ERROR;
	if (!imx_adc_filter(touch_sample))
		return IMX_ADC_SUCCESS;
	else
		return IMX_ADC_ERROR;
}
EXPORT_SYMBOL(imx_adc_get_touch_sample);

void imx_adc_set_hsync(int on)
{
	unsigned long reg;
	if (imx_adc_ready) {
		reg = __raw_readl(tsc_base + TGCR);
		if (on)
			reg |= TGCR_HSYNC_EN;
		else
			reg &= ~TGCR_HSYNC_EN;
		__raw_writel(reg, tsc_base + TGCR);
	}
}
EXPORT_SYMBOL(imx_adc_set_hsync);

/*!
 * This is the suspend of power management for the i.MX ADC API.
 * It supports SAVE and POWER_DOWN state.
 *
 * @param        pdev           the device
 * @param        state          the state
 *
 * @return       This function returns 0 if successful.
 */
static int imx_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	unsigned long reg;

	/* Config idle for 4-wire */
	reg = TSC_4WIRE_PRECHARGE;
	__raw_writel(reg, tsc_base + TICR);

	reg = TSC_4WIRE_TOUCH_DETECT;
	__raw_writel(reg, tsc_base + TICR);

	/* enable deep sleep wake up */
	reg = __raw_readl(tsc_base + TGCR);
	reg |= TGCR_SLPC;
	__raw_writel(reg, tsc_base + TGCR);

	/* mask pen down and pen down irq */
	reg = __raw_readl(tsc_base + TCQCR);
	reg |= CQCR_PD_MSK;
	__raw_writel(reg, tsc_base + TCQCR);
	reg = __raw_readl(tsc_base + TCQMR);
	reg |= TCQMR_PD_IRQ_MSK;
	__raw_writel(reg, tsc_base + TCQMR);

	/* Set power mode to off */
	reg = __raw_readl(tsc_base + TGCR) & ~TGCR_POWER_MASK;
	reg |= TGCR_POWER_OFF;
	__raw_writel(reg, tsc_base + TGCR);

	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(adc_data->irq);
	} else {
		suspend_flag = 1;
		tsc_clk_disable();
	}
	return 0;
};

/*!
 * This is the resume of power management for the i.MX adc API.
 * It supports RESTORE state.
 *
 * @param        pdev           the device
 *
 * @return       This function returns 0 if successful.
 */
static int imx_adc_resume(struct platform_device *pdev)
{
	unsigned long reg;

	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(adc_data->irq);
	} else {
		suspend_flag = 0;
		tsc_clk_enable();
		while (swait > 0) {
			swait--;
			wake_up_interruptible(&suspendq);
		}
	}

	/* recover power mode */
	reg = __raw_readl(tsc_base + TGCR) & ~TGCR_POWER_MASK;
	reg |= TGCR_POWER_SAVE;
	__raw_writel(reg, tsc_base + TGCR);

	return 0;
}

/*!
 * This function implements the open method on an i.MX ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int imx_adc_open(struct inode *inode, struct file *file)
{
	while (suspend_flag) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, !suspend_flag))
			return -ERESTARTSYS;
	}
	pr_debug("imx_adc : imx_adc_open()\n");
	return 0;
}

/*!
 * This function implements the release method on an i.MX ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int imx_adc_free(struct inode *inode, struct file *file)
{
	pr_debug("imx_adc : imx_adc_free()\n");
	return 0;
}

/*!
 * This function initializes all ADC registers with default values. This
 * function also registers the interrupt events.
 *
 * @return       This function returns IMX_ADC_SUCCESS if successful.
 */
int imx_adc_init(void)
{
	unsigned long reg;

	pr_debug("imx_adc_init()\n");

	if (suspend_flag)
		return -EBUSY;

	tsc_clk_enable();

	/* Reset */
	tsc_self_reset();

	/* Internal reference */
	tsc_intref_enable();

	/* Set power mode */
	reg = __raw_readl(tsc_base + TGCR) & ~TGCR_POWER_MASK;
	reg |= TGCR_POWER_SAVE;
	__raw_writel(reg, tsc_base + TGCR);

	imx_tsc_init();

	return IMX_ADC_SUCCESS;
}
EXPORT_SYMBOL(imx_adc_init);

/*!
 * This function disables the ADC, de-registers the interrupt events.
 *
 * @return       This function returns IMX_ADC_SUCCESS if successful.
 */
enum IMX_ADC_STATUS imx_adc_deinit(void)
{
	pr_debug("imx_adc_deinit()\n");

	return IMX_ADC_SUCCESS;
}
EXPORT_SYMBOL(imx_adc_deinit);

/*!
 * This function triggers a conversion and returns one sampling result of one
 * channel.
 *
 * @param        channel   The channel to be sampled
 * @param        result    The pointer to the conversion result. The memory
 *                         should be allocated by the caller of this function.
 *
 * @return       This function returns IMX_ADC_SUCCESS if successful.
 */
enum IMX_ADC_STATUS imx_adc_convert(enum t_channel channel,
				    unsigned short *result)
{
	unsigned long reg;
	int lastitemid;
	struct t_touch_screen touch_sample;

	switch (channel) {

	case TS_X_POS:
		imx_adc_get_touch_sample(&touch_sample, 0);
		result[0] = touch_sample.x_position;

		/* if no pen down ,recover the register configuration */
		if (touch_sample.contact_resistance == 0) {
			reg = __raw_readl(tsc_base + TCQCR);
			reg &= ~CQCR_QSM_MASK;
			reg |= CQCR_QSM_PEN;
			__raw_writel(reg, tsc_base + TCQCR);

			reg = __raw_readl(tsc_base + TCQMR);
			reg &= ~TCQMR_PD_IRQ_MSK;
			__raw_writel(reg, tsc_base + TCQMR);
		}
		break;

	case TS_Y_POS:
		imx_adc_get_touch_sample(&touch_sample, 0);
		result[1] = touch_sample.y_position;

		/* if no pen down ,recover the register configuration */
		if (touch_sample.contact_resistance == 0) {
			reg = __raw_readl(tsc_base + TCQCR);
			reg &= ~CQCR_QSM_MASK;
			reg |= CQCR_QSM_PEN;
			__raw_writel(reg, tsc_base + TCQCR);

			reg = __raw_readl(tsc_base + TCQMR);
			reg &= ~TCQMR_PD_IRQ_MSK;
			__raw_writel(reg, tsc_base + TCQMR);
		}
		break;

	case GER_PURPOSE_ADC0:
		down(&general_convert_mutex);

		lastitemid = 0;
		reg = (0xf << CQCR_FIFOWATERMARK_SHIFT) |
		      (lastitemid << CQCR_LAST_ITEM_ID_SHIFT) | CQCR_QSM_FQS;
		__raw_writel(reg, tsc_base + GCQCR);

		reg = TSC_GENERAL_ADC_GCC0;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC0);

		imx_adc_read_general(result);
		up(&general_convert_mutex);
		break;

	case GER_PURPOSE_ADC1:
		down(&general_convert_mutex);

		lastitemid = 0;
		reg = (0xf << CQCR_FIFOWATERMARK_SHIFT) |
		      (lastitemid << CQCR_LAST_ITEM_ID_SHIFT) | CQCR_QSM_FQS;
		__raw_writel(reg, tsc_base + GCQCR);

		reg = TSC_GENERAL_ADC_GCC1;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC0);

		imx_adc_read_general(result);
		up(&general_convert_mutex);
		break;

	case GER_PURPOSE_ADC2:
		down(&general_convert_mutex);

		lastitemid = 0;
		reg = (0xf << CQCR_FIFOWATERMARK_SHIFT) |
		      (lastitemid << CQCR_LAST_ITEM_ID_SHIFT) | CQCR_QSM_FQS;
		__raw_writel(reg, tsc_base + GCQCR);

		reg = TSC_GENERAL_ADC_GCC2;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC0);

		imx_adc_read_general(result);
		up(&general_convert_mutex);
		break;

	case GER_PURPOSE_MULTICHNNEL:
		down(&general_convert_mutex);

		reg = TSC_GENERAL_ADC_GCC0;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC0);

		reg = TSC_GENERAL_ADC_GCC1;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC1);

		reg = TSC_GENERAL_ADC_GCC2;
		reg |= (3 << CC_NOS_SHIFT) | (16 << CC_SETTLING_TIME_SHIFT);
		__raw_writel(reg, tsc_base + GCC2);

		reg = (GCQ_ITEM_GCC2 << GCQ_ITEM2_SHIFT) |
		      (GCQ_ITEM_GCC1 << GCQ_ITEM1_SHIFT) |
		      (GCQ_ITEM_GCC0 << GCQ_ITEM0_SHIFT);
		__raw_writel(reg, tsc_base + GCQ_ITEM_7_0);

		lastitemid = 2;
		reg = (0xf << CQCR_FIFOWATERMARK_SHIFT) |
		      (lastitemid << CQCR_LAST_ITEM_ID_SHIFT) | CQCR_QSM_FQS;
		__raw_writel(reg, tsc_base + GCQCR);

		imx_adc_read_general(result);
		up(&general_convert_mutex);
		break;
	default:
		pr_debug("%s: bad channel number\n", __func__);
		return IMX_ADC_ERROR;
	}

	return IMX_ADC_SUCCESS;
}
EXPORT_SYMBOL(imx_adc_convert);

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
 * @return       This function returns IMX_ADC_SUCCESS if successful.
 */
enum IMX_ADC_STATUS imx_adc_convert_multichnnel(enum t_channel channels,
						unsigned short *result)
{
	imx_adc_convert(GER_PURPOSE_MULTICHNNEL, result);
	return IMX_ADC_SUCCESS;
}
EXPORT_SYMBOL(imx_adc_convert_multichnnel);

/*!
 * This function implements IOCTL controls on an i.MX ADC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int imx_adc_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct t_adc_convert_param *convert_param;

	if ((_IOC_TYPE(cmd) != 'p') && (_IOC_TYPE(cmd) != 'D'))
		return -ENOTTY;

	while (suspend_flag) {
		swait++;
		/* Block if the device is suspended */
		if (wait_event_interruptible(suspendq, !suspend_flag))
			return -ERESTARTSYS;
	}

	switch (cmd) {
	case IMX_ADC_INIT:
		pr_debug("init adc\n");
		CHECK_ERROR(imx_adc_init());
		break;

	case IMX_ADC_DEINIT:
		pr_debug("deinit adc\n");
		CHECK_ERROR(imx_adc_deinit());
		break;

	case IMX_ADC_CONVERT:
		convert_param = kmalloc(sizeof(*convert_param), GFP_KERNEL);
		if (convert_param == NULL)
			return -ENOMEM;
		if (copy_from_user(convert_param,
				   (struct t_adc_convert_param *)arg,
				   sizeof(*convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(imx_adc_convert(convert_param->channel,
						  convert_param->result),
				  (kfree(convert_param)));

		if (copy_to_user((struct t_adc_convert_param *)arg,
				 convert_param, sizeof(*convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		kfree(convert_param);
		break;

	case IMX_ADC_CONVERT_MULTICHANNEL:
		convert_param = kmalloc(sizeof(*convert_param), GFP_KERNEL);
		if (convert_param == NULL)
			return -ENOMEM;
		if (copy_from_user(convert_param,
				   (struct t_adc_convert_param *)arg,
				   sizeof(*convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		CHECK_ERROR_KFREE(imx_adc_convert_multichnnel
				  (convert_param->channel,
				   convert_param->result),
				  (kfree(convert_param)));

		if (copy_to_user((struct t_adc_convert_param *)arg,
				 convert_param, sizeof(*convert_param))) {
			kfree(convert_param);
			return -EFAULT;
		}
		kfree(convert_param);
		break;

	default:
		pr_debug("imx_adc_ioctl: unsupported ioctl command 0x%x\n",
			 cmd);
		return -EINVAL;
	}
	return 0;
}

static struct file_operations imx_adc_fops = {
	.owner = THIS_MODULE,
	.ioctl = imx_adc_ioctl,
	.open = imx_adc_open,
	.release = imx_adc_free,
};

static int imx_adc_module_probe(struct platform_device *pdev)
{
	int ret = 0;
	int retval;
	struct device *temp_class;
	struct resource *res;
	void __iomem *base;

	/* ioremap the base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No TSC base address provided\n");
		goto err_out0;
	}
	base = ioremap(res->start, res->end - res->start);
	if (base == NULL) {
		dev_err(&pdev->dev, "failed to rebase TSC base address\n");
		goto err_out0;
	}
	tsc_base = (unsigned long)base;

	/* create the chrdev */
	imx_adc_major = register_chrdev(0, "imx_adc", &imx_adc_fops);

	if (imx_adc_major < 0) {
		dev_err(&pdev->dev, "Unable to get a major for imx_adc\n");
		return imx_adc_major;
	}
	init_waitqueue_head(&suspendq);
	init_waitqueue_head(&tsq);

	imx_adc_class = class_create(THIS_MODULE, "imx_adc");
	if (IS_ERR(imx_adc_class)) {
		dev_err(&pdev->dev, "Error creating imx_adc class.\n");
		ret = PTR_ERR(imx_adc_class);
		goto err_out1;
	}

	temp_class = device_create(imx_adc_class, NULL,
				   MKDEV(imx_adc_major, 0), NULL, "imx_adc");
	if (IS_ERR(temp_class)) {
		dev_err(&pdev->dev, "Error creating imx_adc class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	adc_data = kmalloc(sizeof(struct imx_adc_data), GFP_KERNEL);
	if (adc_data == NULL)
		return -ENOMEM;
	adc_data->irq = platform_get_irq(pdev, 0);
	retval = request_irq(adc_data->irq, imx_adc_interrupt,
			     0, MOD_NAME, MOD_NAME);
	if (retval) {
		return retval;
	}
	adc_data->adc_clk = clk_get(&pdev->dev, "tchscrn_clk");

	ret = imx_adc_init();

	if (ret != IMX_ADC_SUCCESS) {
		dev_err(&pdev->dev, "Error in imx_adc_init.\n");
		goto err_out4;
	}
	imx_adc_ready = 1;

	/* By default, devices should wakeup if they can */
	/* So TouchScreen is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	pr_info("i.MX ADC at 0x%x irq %d\n", (unsigned int)res->start,
		adc_data->irq);
	return ret;

err_out4:
	device_destroy(imx_adc_class, MKDEV(imx_adc_major, 0));
err_out2:
	class_destroy(imx_adc_class);
err_out1:
	unregister_chrdev(imx_adc_major, "imx_adc");
err_out0:
	return ret;
}

static int imx_adc_module_remove(struct platform_device *pdev)
{
	imx_adc_ready = 0;
	imx_adc_deinit();
	device_destroy(imx_adc_class, MKDEV(imx_adc_major, 0));
	class_destroy(imx_adc_class);
	unregister_chrdev(imx_adc_major, "imx_adc");
	free_irq(adc_data->irq, MOD_NAME);
	kfree(adc_data);
	pr_debug("i.MX ADC successfully removed\n");
	return 0;
}

static struct platform_driver imx_adc_driver = {
	.driver = {
		   .name = "imx_adc",
		   },
	.suspend = imx_adc_suspend,
	.resume = imx_adc_resume,
	.probe = imx_adc_module_probe,
	.remove = imx_adc_module_remove,
};

/*
 * Initialization and Exit
 */
static int __init imx_adc_module_init(void)
{
	pr_debug("i.MX ADC driver loading...\n");
	return platform_driver_register(&imx_adc_driver);
}

static void __exit imx_adc_module_exit(void)
{
	platform_driver_unregister(&imx_adc_driver);
	pr_debug("i.MX ADC driver successfully unloaded\n");
}

/*
 * Module entry points
 */

module_init(imx_adc_module_init);
module_exit(imx_adc_module_exit);

MODULE_DESCRIPTION("i.MX ADC device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
