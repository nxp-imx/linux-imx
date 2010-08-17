/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mc13783/pmic_adc_defs.h
 * @brief This header contains all defines for PMIC(mc13783) ADC driver.
 *
 * @ingroup PMIC_ADC
 */

#ifndef __MC13783_ADC__DEFS_H__
#define __MC13783_ADC__DEFS_H__

#define MC13783_ADC_DEVICE "/dev/mc13783_adc"

#define         DEF_ADC_0     0x008000
#define         DEF_ADC_3     0x000080

#define ADC_NB_AVAILABLE        2

#define MAX_CHANNEL             7

/*
 * Maximun allowed variation in the three X/Y co-ordinates acquired from
 * touch-screen
 */
#define DELTA_Y_MAX             50
#define DELTA_X_MAX             50

/* Upon clearing the filter, this is the delay in restarting the filter */
#define FILTER_MIN_DELAY        4

/* Length of X and Y Touch screen filters */
#define FILTLEN                 3

#define TS_X_MAX                1000
#define TS_Y_MAX                1000

#define TS_X_MIN                80
#define TS_Y_MIN                80

#define MC13783_ADC0_TS_M_LSH	14
#define MC13783_ADC0_TS_M_WID	3
/*
 * ADC 0
 */
#define ADC_WAIT_TSI_0		0x001C00

/*
 * ADC 1
 */

#define ADC_EN                  0x000001
#define ADC_SGL_CH              0x000002
#define ADC_ADSEL               0x000008
#define ADC_CH_0_POS            5
#define ADC_CH_0_MASK           0x0000E0
#define ADC_CH_1_POS            8
#define ADC_CH_1_MASK           0x000700
#define ADC_DELAY_POS           11
#define ADC_DELAY_MASK          0x07F800
#define ADC_ATO                 0x080000
#define ASC_ADC                 0x100000
#define ADC_WAIT_TSI_1		0x300001
#define ADC_CHRGRAW_D5          0x008000

/*
 * ADC 2 - 4
 */
#define ADD1_RESULT_MASK        0x00000FFC
#define ADD2_RESULT_MASK        0x00FFC000
#define ADC_TS_MASK             0x00FFCFFC

/*
 * ADC 3
 */
#define ADC_INC                 0x030000
#define ADC_BIS                 0x800000

/*
 * ADC 3
 */
#define ADC_NO_ADTRIG           0x200000
#define ADC_WCOMP               0x040000
#define ADC_WCOMP_H_POS         0
#define ADC_WCOMP_L_POS         9
#define ADC_WCOMP_H_MASK        0x00003F
#define ADC_WCOMP_L_MASK        0x007E00

#define ADC_MODE_MASK           0x00003F

/*
 * Interrupt Status 0
 */
#define ADC_INT_BISDONEI        0x02

/*!
 * Define state mode of ADC.
 */
typedef enum adc_state {
	/*!
	 * Free.
	 */
	ADC_FREE,
	/*!
	 * Used.
	 */
	ADC_USED,
	/*!
	 * Monitoring
	 */
	ADC_MONITORING,
} t_adc_state;

/*!
 * This enumeration, is used to configure the mode of ADC.
 */
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
	/*!
	 * Enables thermistor reading
	 */
	M_THERMISTOR = 0x000008,
	/*!
	 * Enables die temperature reading
	 */
	M_DIE_TEMPERATURE = 0x000010,
	/*!
	 * Enables UID reading
	 */
	M_UID = 0x000020,
} t_reading_mode;

/*!
 * This enumeration, is used to configure the monitoring mode.
 */
typedef enum check_mode {
	/*!
	 * Comparator low level
	 */
	CHECK_LOW,
	/*!
	 * Comparator high level
	 */
	CHECK_HIGH,
	/*!
	 * Comparator low or high level
	 */
	CHECK_LOW_OR_HIGH,
} t_check_mode;

/*!
 * This structure is used to configure and report adc value.
 */
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
	 * Selects the set of inputs
	 */
	bool group;
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

/*!
 * This structure is used to configure the monitoring mode of ADC.
 */
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
	 * Channel selection 1
	 */
	t_channel channel;
	/*!
	 * Selects the set of inputs
	 */
	bool group;
	/*!
	 * Used to configure ADC mode with t_reading_mode
	 */
	unsigned int read_mode;
	/*!
	 * Comparator low level in WCOMP mode
	 */
	unsigned int comp_low;
	/*!
	 * Comparator high level in WCOMP mode
	 */
	unsigned int comp_high;
	/*!
	 * Sets type of monitoring (low, high or both)
	 */
	t_check_mode check_mode;
	/*!
	 * Callback to be launched when event is detected
	 */
	void (*callback) (void);
} t_monitoring_param;

/*!
 * This function performs filtering and rejection of excessive noise prone
 * samples.
 *
 * @param        ts_curr     Touch screen value
 *
 * @return       This function returns 0 on success, -1 otherwise.
 */
static int pmic_adc_filter(t_touch_screen *ts_curr);

/*!
 * This function request a ADC.
 *
 * @return      This function returns index of ADC to be used (0 or 1) if successful.
 * return -1 if error.
 */
int mc13783_adc_request(bool read_ts);

/*!
 * This function is used to update buffer of touch screen value in read mode.
 */
void update_buffer(void);

/*!
 * This function release an ADC.
 *
 * @param        adc_index     index of ADC to be released.
 *
 * @return       This function returns 0 if successful.
 */
int mc13783_adc_release(int adc_index);

/*!
 * This function select the required read_mode for a specific channel.
 *
 * @param        channel   The channel to be sampled
 *
 * @return       This function returns the requires read_mode
 */
t_reading_mode mc13783_set_read_mode(t_channel channel);

/*!
 * This function read the touch screen value.
 *
 * @param        touch_sample    return value of touch screen
 * @param        wait_tsi    if true, this function is synchronous (wait in TSI event).
 *
 * @return       This function returns 0.
 */
PMIC_STATUS mc13783_adc_read_ts(t_touch_screen *touch_sample, int wait_tsi);

#endif				/* __MC13783_ADC__DEFS_H__ */
