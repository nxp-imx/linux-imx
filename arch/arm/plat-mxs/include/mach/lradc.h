/*
 * Freescale STMP37XX/STMP378X LRADC helper interface
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_PLAT_LRADC_H
#define __ASM_PLAT_LRADC_H

int hw_lradc_use_channel(int);
int hw_lradc_unuse_channel(int);
extern u32 hw_lradc_vddio(void);
void hw_lradc_set_delay_trigger_kick(int trigger, int value);
void hw_lradc_configure_channel(int channel, int enable_div2,
		int enable_acc, int samples);
int hw_lradc_present(int channel);
int hw_lradc_init_ladder(int channel, int trigger, unsigned sampling);
int hw_lradc_stop_ladder(int channel, int trigger);
void hw_lradc_set_delay_trigger(int trigger, u32 trigger_lradc,
		u32 delay_triggers, u32 loops, u32 delays);
void hw_lradc_clear_delay_trigger(int trigger, u32 trigger_lradc,
		u32 delay_triggers);


#define LRADC_CH0		0
#define LRADC_CH1		1
#define LRADC_CH2		2
#define LRADC_CH3		3
#define LRADC_CH4		4
#define LRADC_CH5		5
#define LRADC_CH6		6
#define LRADC_CH7		7
#define LRADC_TOUCH_X_PLUS	LRADC_CH2
#define LRADC_TOUCH_Y_PLUS	LRADC_CH3
#define LRADC_TOUCH_X_MINUS	LRADC_CH4
#define LRADC_TOUCH_Y_MINUS	LRADC_CH5
#define VDDIO_VOLTAGE_CH	LRADC_CH6
#define BATTERY_VOLTAGE_CH	LRADC_CH7

#define LRADC_CLOCK_6MHZ	0
#define LRADC_CLOCK_4MHZ	1
#define LRADC_CLOCK_3MHZ	2
#define LRADC_CLOCK_2MHZ	3

#define LRADC_DELAY_TRIGGER_BUTTON	0
#define LRADC_DELAY_TRIGGER_BATTERY	1
#define LRADC_DELAY_TRIGGER_TOUCHSCREEN	2
#define LRADC_DELAY_TRIGGER_DIE		3

#endif /* __ASM_PLAT_LRADC_H */
