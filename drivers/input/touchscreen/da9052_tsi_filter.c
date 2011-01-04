/*
 * da9052_tsi_filter.c  --  TSI filter driver for Dialog DA9052
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

#include <linux/input.h>
#include <linux/freezer.h>
#include <linux/mfd/da9052/tsi.h>

#define get_abs(x)	(x < 0 ? ((-1) * x) : (x))

#define WITHIN_WINDOW(x, y)	((get_abs(x) <= TSI_X_WINDOW_SIZE) && \
				(get_abs(y) <= TSI_Y_WINDOW_SIZE))

#define FIRST_SAMPLE					0

#define incr_with_wrap_raw_fifo(x)			\
		if (++x >= TSI_RAW_DATA_BUF_SIZE)	\
			x = 0

#define decr_with_wrap_raw_fifo(x)			\
		if (--x < 0)				\
			x = (TSI_RAW_DATA_BUF_SIZE-1)

static u32 get_raw_data_cnt(struct da9052_ts_priv *priv);
static void da9052_tsi_convert_reg_to_coord(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *raw_data);
static inline void clean_tsi_reg_fifo(struct da9052_ts_priv *priv);
static inline void clean_tsi_raw_fifo(struct da9052_ts_priv *priv);

#if (ENABLE_AVERAGE_FILTER)
static void da9052_tsi_avrg_filter(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *tsi_avg_data);

#endif
#if (ENABLE_TSI_DEBOUNCE)
static s32 da9052_tsi_calc_debounce_data(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *raw_data);

#endif
# if (ENABLE_WINDOW_FILTER)
static s32 diff_within_window(struct da9052_tsi_data *prev_raw_data,
			      struct da9052_tsi_data *cur_raw_data);
#endif
static s32 da9052_tsi_window_filter(struct da9052_ts_priv *ts,
					struct da9052_tsi_data *raw_data);

void clean_tsi_fifos(struct da9052_ts_priv *priv)
{
	clean_tsi_raw_fifo(priv);
	clean_tsi_reg_fifo(priv);
}

void __init da9052_init_tsi_fifos(struct da9052_ts_priv *priv)
{
	init_MUTEX(&priv->tsi_raw_fifo.lock);
	init_MUTEX(&priv->tsi_reg_fifo.lock);

	clean_tsi_raw_fifo(priv);
	clean_tsi_reg_fifo(priv);
}

u32 get_reg_data_cnt(struct da9052_ts_priv *priv)
{
	u8 reg_data_cnt;

	if (priv->tsi_reg_fifo.head <= priv->tsi_reg_fifo.tail) {
		reg_data_cnt = (priv->tsi_reg_fifo.tail -
				priv->tsi_reg_fifo.head);
	} else {
		reg_data_cnt = (priv->tsi_reg_fifo.tail +
				(TSI_REG_DATA_BUF_SIZE -
				priv->tsi_reg_fifo.head));
	}

	return reg_data_cnt;
}

u32 get_reg_free_space_cnt(struct da9052_ts_priv *priv)
{
	u32 free_cnt;

	if (priv->tsi_reg_fifo.head <= priv->tsi_reg_fifo.tail) {
		free_cnt = ((TSI_REG_DATA_BUF_SIZE - 1) -
			(priv->tsi_reg_fifo.tail - priv->tsi_reg_fifo.head));
	} else
		free_cnt = ((priv->tsi_reg_fifo.head - priv->tsi_reg_fifo.tail)
				- 1);

	return free_cnt;
}

void da9052_tsi_process_reg_data(struct da9052_ts_priv *priv)
{
	s32 ret;
	struct da9052_tsi_data tmp_raw_data;
	u32 reg_data_cnt;

	if (down_interruptible(&priv->tsi_reg_fifo.lock))
		return;

	reg_data_cnt = get_reg_data_cnt(priv);

	while (reg_data_cnt-- > 0) {

		ret = 0;

		if (get_raw_data_cnt(priv) >= (TSI_RAW_DATA_BUF_SIZE - 1)) {
			DA9052_DEBUG("%s: RAW data FIFO is full\n",
							__FUNCTION__);
			break;
		}

		da9052_tsi_convert_reg_to_coord(priv, &tmp_raw_data);

		if ((tmp_raw_data.x < TS_X_MIN) ||
			(tmp_raw_data.x > TS_X_MAX) ||
			(tmp_raw_data.y < TS_Y_MIN) ||
			(tmp_raw_data.y > TS_Y_MAX)) {
			DA9052_DEBUG("%s: ", __FUNCTION__);
			DA9052_DEBUG("sample beyond touchscreen panel ");
			DA9052_DEBUG("dimensions\n");
			continue;
		}

#if (ENABLE_TSI_DEBOUNCE)
		if (debounce_over == FALSE) {
			ret =
			da9052_tsi_calc_debounce_data(priv, &tmp_raw_data);
			if (ret != SUCCESS)
				continue;
		}
#endif

# if (ENABLE_WINDOW_FILTER)
		ret = da9052_tsi_window_filter(priv, &tmp_raw_data);
		if (ret != SUCCESS)
			continue;
#endif
		priv->early_data_flag = FALSE;

		if (down_interruptible(&priv->tsi_raw_fifo.lock)) {
			DA9052_DEBUG("%s: Failed to ", __FUNCTION__);
			DA9052_DEBUG("acquire RAW FIFO Lock!\n");

			up(&priv->tsi_reg_fifo.lock);
			return;
		}

		priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.tail] = tmp_raw_data;
		incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.tail);

		up(&priv->tsi_raw_fifo.lock);
	}


	up(&priv->tsi_reg_fifo.lock);

	return;
}

static u32 get_raw_data_cnt(struct da9052_ts_priv *priv)
{
	u32 raw_data_cnt;

	if (priv->tsi_raw_fifo.head <= priv->tsi_raw_fifo.tail)
		raw_data_cnt =
			(priv->tsi_raw_fifo.tail - priv->tsi_raw_fifo.head);
	else
		raw_data_cnt =
			(priv->tsi_raw_fifo.tail + (TSI_RAW_DATA_BUF_SIZE -
			priv->tsi_raw_fifo.head));

	return raw_data_cnt;
}

static void da9052_tsi_convert_reg_to_coord(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *raw_data)
{

	struct da9052_tsi_reg *src;
	struct da9052_tsi_data *dst = raw_data;

	src = &priv->tsi_reg_fifo.data[priv->tsi_reg_fifo.head];

	dst->x = (src->x_msb << X_MSB_SHIFT);
	dst->x |= (src->lsb & X_LSB_MASK) >> X_LSB_SHIFT;

	dst->y = (src->y_msb << Y_MSB_SHIFT);
	dst->y |= (src->lsb & Y_LSB_MASK) >> Y_LSB_SHIFT;

	dst->z = (src->z_msb << Z_MSB_SHIFT);
	dst->z |= (src->lsb & Z_LSB_MASK) >> Z_LSB_SHIFT;

#if DA9052_TSI_RAW_DATA_PROFILING
	printk("R\tX\t%4d\tY\t%4d\tZ\t%4d\n",
					(u16)dst->x,
					(u16)dst->y,
					(u16)dst->z);
#endif
	priv->raw_data_cnt++;
	incr_with_wrap_reg_fifo(priv->tsi_reg_fifo.head);
}

#if (ENABLE_AVERAGE_FILTER)
static void da9052_tsi_avrg_filter(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *tsi_avg_data)
{
	u8 cnt;

	if (down_interruptible(&priv->tsi_raw_fifo.lock)) {
		printk("%s: No RAW Lock !\n", __FUNCTION__);
		return;
	}

	(*tsi_avg_data) = priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head];
	incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.head);

	for (cnt = 2; cnt <= TSI_AVERAGE_FILTER_SIZE; cnt++) {

		tsi_avg_data->x +=
			priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].x;
		tsi_avg_data->y +=
			priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].y;
		tsi_avg_data->z +=
			priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].z;

		incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.head);
	}

	up(&priv->tsi_raw_fifo.lock);

	tsi_avg_data->x /= TSI_AVERAGE_FILTER_SIZE;
	tsi_avg_data->y /= TSI_AVERAGE_FILTER_SIZE;
	tsi_avg_data->z /= TSI_AVERAGE_FILTER_SIZE;

#if DA9052_TSI_AVG_FLT_DATA_PROFILING
	printk("A\tX\t%4d\tY\t%4d\tZ\t%4d\n",
			(u16)tsi_avg_data->x,
			(u16)tsi_avg_data->y,
			(u16)tsi_avg_data->z);
#endif

	return;
}
#endif

s32 da9052_tsi_raw_proc_thread(void *ptr)
{
	struct da9052_tsi_data coord;
	u8 calib_ok, range_ok;
	struct calib_cfg_t *tsi_calib = get_calib_config();
	struct input_dev *ip_dev = (struct input_dev *)
				da9052_tsi_get_input_dev(
						(u8)TSI_INPUT_DEVICE_OFF);
	struct da9052_ts_priv *priv = (struct da9052_ts_priv *)ptr;

	set_freezable();

	while (priv->tsi_raw_proc_thread.state == ACTIVE) {

		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(
			msecs_to_jiffies(priv->tsi_raw_data_poll_interval));

		if (priv->early_data_flag || (get_raw_data_cnt(priv) == 0))
			continue;

		calib_ok = TRUE;
		range_ok = TRUE;

#if (ENABLE_AVERAGE_FILTER)

		if (get_raw_data_cnt(priv) < TSI_AVERAGE_FILTER_SIZE)
			continue;

		da9052_tsi_avrg_filter(priv, &coord);

#else

		if (down_interruptible(&priv->tsi_raw_fifo.lock))
			continue;

		coord = priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head];
		incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.head);

		up(&priv->tsi_raw_fifo.lock);

#endif

		if (tsi_calib->calibrate_flag) {
			calib_ok = da9052_tsi_get_calib_display_point(&coord);

			if ((coord.x < DISPLAY_X_MIN) ||
				(coord.x > DISPLAY_X_MAX) ||
				(coord.y < DISPLAY_Y_MIN) ||
						(coord.y > DISPLAY_Y_MAX))
				range_ok = FALSE;
		}

		if (calib_ok && range_ok) {
			input_report_abs(ip_dev, BTN_TOUCH, 1);
			input_report_abs(ip_dev, ABS_X, coord.x);
			input_report_abs(ip_dev, ABS_Y, coord.y);
			input_sync(ip_dev);

			priv->os_data_cnt++;

#if DA9052_TSI_OS_DATA_PROFILING
			printk("O\tX\t%4d\tY\t%4d\tZ\t%4d\n",  (u16)coord.x,
						(u16)coord.y, (u16)coord.z);
#endif
		} else {
			if (!calib_ok) {
				DA9052_DEBUG("%s: ", __FUNCTION__);
				DA9052_DEBUG("calibration Failed\n");
			}
			if (!range_ok) {
				DA9052_DEBUG("%s: ", __FUNCTION__);
				DA9052_DEBUG("sample beyond display ");
				DA9052_DEBUG("panel dimension\n");
			}
		}
	}
	priv->tsi_raw_proc_thread.thread_task = NULL;
	complete_and_exit(&priv->tsi_raw_proc_thread.notifier, 0);
	return 0;

}

#if (ENABLE_TSI_DEBOUNCE)
static s32 da9052_tsi_calc_debounce_data(struct da9052_tsi_data *raw_data)
{
#if (TSI_DEBOUNCE_DATA_CNT)
	u8 cnt;
	struct da9052_tsi_data	temp = {.x = 0, .y = 0, .z = 0};

	priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.tail] = (*raw_data);
	incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.tail);

	if (get_raw_data_cnt(priv) <= TSI_DEBOUNCE_DATA_CNT)
		return -FAILURE;

	for (cnt = 1; cnt <= TSI_DEBOUNCE_DATA_CNT; cnt++) {
		temp.x += priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].x;
		temp.y += priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].y;
		temp.z += priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.head].z;

		incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.head);
	}

	temp.x /= TSI_DEBOUNCE_DATA_CNT;
	temp.y /= TSI_DEBOUNCE_DATA_CNT;
	temp.z /= TSI_DEBOUNCE_DATA_CNT;

	priv->tsi_raw_fifo.tail = priv->tsi_raw_fifo.head;
	priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.tail] = temp;
	incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.tail);

#if DA9052_TSI_PRINT_DEBOUNCED_DATA
	printk("D: X: %d Y: %d Z: %d\n",
	       (u16)temp.x, (u16)temp.y, (u16)temp.z);
#endif

#endif
	priv->debounce_over = TRUE;

	return 0;
}
#endif

# if (ENABLE_WINDOW_FILTER)
static s32 diff_within_window(struct da9052_tsi_data *prev_raw_data,
				struct da9052_tsi_data *cur_raw_data)
{
	s32 ret = -EINVAL;
	s32 x, y;
	x = ((cur_raw_data->x) - (prev_raw_data->x));
	y = ((cur_raw_data->y) - (prev_raw_data->y));

	if (WITHIN_WINDOW(x, y)) {

#if DA9052_TSI_WIN_FLT_DATA_PROFILING
		printk("W\tX\t%4d\tY\t%4d\tZ\t%4d\n",
					(u16)cur_raw_data->x,
					(u16)cur_raw_data->y,
					(u16)cur_raw_data->z);
#endif
		ret = 0;
	}
	return ret;
}

static s32 da9052_tsi_window_filter(struct da9052_ts_priv *priv,
					struct da9052_tsi_data *raw_data)
{
	u8 ref_found;
	u32 cur, next;
	s32 ret = -EINVAL;
	static struct da9052_tsi_data prev_raw_data;

	if (priv->win_reference_valid == TRUE) {

#if DA9052_TSI_PRINT_PREVIOUS_DATA
		printk("P: X: %d Y: %d Z: %d\n",
		(u16)prev_raw_data.x, (u16)prev_raw_data.y,
		(u16)prev_raw_data.z);
#endif
		ret = diff_within_window(&prev_raw_data, raw_data);
		if (!ret)
			prev_raw_data = (*raw_data);
	} else {
		priv->tsi_raw_fifo.data[priv->tsi_raw_fifo.tail] = (*raw_data);
		incr_with_wrap_raw_fifo(priv->tsi_raw_fifo.tail);

		if (get_raw_data_cnt(priv) == SAMPLE_CNT_FOR_WIN_REF) {

			ref_found = FALSE;

			next = cur = priv->tsi_raw_fifo.head;
			incr_with_wrap_raw_fifo(next);

			while (next <= priv->tsi_raw_fifo.tail) {
				ret = diff_within_window(
						&priv->tsi_raw_fifo.data[cur],
						&priv->tsi_raw_fifo.data[next]
						);
				if (ret == SUCCESS) {
					ref_found = TRUE;
					break;
				}
				incr_with_wrap_raw_fifo(cur);
				incr_with_wrap_raw_fifo(next);

			}

			if (ref_found == FALSE)
				priv->tsi_raw_fifo.tail =
					priv->tsi_raw_fifo.head;
			else {
				prev_raw_data = priv->tsi_raw_fifo.data[cur];

				prev_raw_data.x +=
					priv->tsi_raw_fifo.data[next].x;
				prev_raw_data.y +=
					priv->tsi_raw_fifo.data[next].y;
				prev_raw_data.z +=
					priv->tsi_raw_fifo.data[next].z;

				prev_raw_data.x = prev_raw_data.x / 2;
				prev_raw_data.y = prev_raw_data.y / 2;
				prev_raw_data.z = prev_raw_data.z / 2;

				(*raw_data) = prev_raw_data;

				priv->tsi_raw_fifo.tail =
						priv->tsi_raw_fifo.head;

				priv->win_reference_valid = TRUE;
				ret = SUCCESS;
			}
		}
	}
	return ret;
}
#endif
static inline void clean_tsi_reg_fifo(struct da9052_ts_priv *priv)
{
	priv->tsi_reg_fifo.head = FIRST_SAMPLE;
	priv->tsi_reg_fifo.tail = FIRST_SAMPLE;
}

static inline void clean_tsi_raw_fifo(struct da9052_ts_priv *priv)
{
	priv->tsi_raw_fifo.head = FIRST_SAMPLE;
	priv->tsi_raw_fifo.tail = FIRST_SAMPLE;
}

