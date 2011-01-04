/*
 * da9052 BAT module declarations.
  *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_MFD_DA9052_BAT_H
#define __LINUX_MFD_DA9052_BAT_H

#include <linux/power_supply.h>

/* STATIC CONFIGURATION */
#define BAT_MANUFACTURER			"Samsung"
#define BAT_TYPE				POWER_SUPPLY_TECHNOLOGY_LION
#define DA9052_LOOK_UP_TABLE_SIZE		68
#define DA9052_NO_OF_LOOKUP_TABLE		3
#define CURRENT_MONITORING_WINDOW		10
#define FILTER_SIZE				4
#define DA9052_NUMBER_OF_STORE_CURENT_READING	4
#define DA9052_ILLEGAL_BATTERY_DETECT		1
#define DA9052_BAT_FILTER_HYS			0


static const u16 temperature_lookup_ref
			[DA9052_NO_OF_LOOKUP_TABLE] = {10, 25, 40};
static u32 const vbat_vs_capacity_look_up[DA9052_NO_OF_LOOKUP_TABLE]
					[DA9052_LOOK_UP_TABLE_SIZE][2] = {
	/* For temperature 10 degree celisus*/
	{
	{4082, 100}, {4036, 98},
	{4020, 96}, {4008, 95},
	{3997, 93}, {3983, 91},
	{3964, 90}, {3943, 88},
	{3926, 87}, {3912, 85},
	{3900, 84}, {3890, 82},
	{3881, 80}, {3873, 79},
	{3865, 77}, {3857, 76},
	{3848, 74}, {3839, 73},
	{3829, 71}, {3820, 70},
	{3811, 68}, {3802, 67},
	{3794, 65}, {3785, 64},
	{3778, 62}, {3770, 61},
	{3763, 59}, {3756, 58},
	{3750, 56}, {3744, 55},
	{3738, 53}, {3732, 52},
	{3727, 50}, {3722, 49},
	{3717, 47}, {3712, 46},
	{3708, 44}, {3703, 43},
	{3700, 41}, {3696, 40},
	{3693, 38}, {3691, 37},
	{3688, 35}, {3686, 34},
	{3683, 32}, {3681, 31},
	{3678, 29}, {3675, 28},
	{3672, 26}, {3669, 25},
	{3665, 23}, {3661, 22},
	{3656, 21}, {3651, 19},
	{3645, 18}, {3639, 16},
	{3631, 15}, {3622, 13},
	{3611, 12}, {3600, 10},
	{3587, 9}, {3572, 7},
	{3548, 6}, {3503, 5},
	{3420, 3}, {3268, 2},
	{2992, 1}, {2746, 0}
	},
	/* For temperature 25 degree celisus */
	{
	{4102, 100}, {4065, 98},
	{4048, 96}, {4034, 95},
	{4021, 93}, {4011, 92},
	{4001, 90}, {3986, 88},
	{3968, 87}, {3952, 85},
	{3938, 84}, {3926, 82},
	{3916, 81}, {3908, 79},
	{3900, 77}, {3892, 76},
	{3883, 74}, {3874, 73},
	{3864, 71}, {3855, 70},
	{3846, 68}, {3836, 67},
	{3827, 65}, {3819, 64},
	{3810, 62}, {3801, 61},
	{3793, 59}, {3786, 58},
	{3778, 56}, {3772, 55},
	{3765, 53}, {3759, 52},
	{3754, 50}, {3748, 49},
	{3743, 47}, {3738, 46},
	{3733, 44}, {3728, 43},
	{3724, 41}, {3720, 40},
	{3716, 38}, {3712, 37},
	{3709, 35}, {3706, 34},
	{3703, 33}, {3701, 31},
	{3698, 30}, {3696, 28},
	{3693, 27}, {3690, 25},
	{3687, 24}, {3683, 22},
	{3680, 21}, {3675, 19},
	{3671, 18}, {3666, 17},
	{3660, 15}, {3654, 14},
	{3647, 12}, {3639, 11},
	{3630, 9}, {3621, 8},
	{3613, 6}, {3606, 5},
	{3597, 4}, {3582, 2},
	{3546, 1}, {2747, 0}
	},
	/* For temperature 40 degree celisus*/
	{
	{4114, 100}, {4081, 98},
	{4065, 96}, {4050, 95},
	{4036, 93}, {4024, 92},
	{4013, 90}, {4002, 88},
	{3990, 87}, {3976, 85},
	{3962, 84}, {3950, 82},
	{3939, 81}, {3930, 79},
	{3921, 77}, {3912, 76},
	{3902, 74}, {3893, 73},
	{3883, 71}, {3874, 70},
	{3865, 68}, {3856, 67},
	{3847, 65}, {3838, 64},
	{3829, 62}, {3820, 61},
	{3812, 59}, {3803, 58},
	{3795, 56}, {3787, 55},
	{3780, 53}, {3773, 52},
	{3767, 50}, {3761, 49},
	{3756, 47}, {3751, 46},
	{3746, 44}, {3741, 43},
	{3736, 41}, {3732, 40},
	{3728, 38}, {3724, 37},
	{3720, 35}, {3716, 34},
	{3713, 33}, {3710, 31},
	{3707, 30}, {3704, 28},
	{3701, 27}, {3698, 25},
	{3695, 24}, {3691, 22},
	{3686, 21}, {3681, 19},
	{3676, 18}, {3671, 17},
	{3666, 15}, {3661, 14},
	{3655, 12}, {3648, 11},
	{3640, 9}, {3632, 8},
	{3622, 6}, {3616, 5},
	{3611, 4}, {3604, 2},
	{3594, 1}, {2747, 0}
	}
};

enum charge_status_enum {
	DA9052_NONE = 1,
	DA9052_CHARGING,
	DA9052_DISCHARGING_WITH_CHARGER,
	DA9052_DISCHARGING_WITHOUT_CHARGER,
	DA9052_PRECHARGING,
	DA9052_LINEARCHARGING,
	DA9052_CHARGEEND
};

enum charger_type_enum {
	DA9052_NOCHARGER = 1,
	DA9052_USB_HUB,
	DA9052_USB_CHARGER,
	DA9052_WALL_CHARGER
};

enum precharge_enum {
	PRE_CHARGE_0MA = 0,
	PRE_CHARGE_20MA = 20,
	PRE_CHARGE_40MA = 40,
	PRE_CHARGE_60MA = 60
};

struct da9052_bat_event_registration {
	u8	da9052_event_vddlow:1;
	u8	da9052_event_tbat:1;
};

struct da9052_bat_threshold {
	u16	vddout_mon;
	u16	ichg_thr;
	u16	tbat_thr_min;
	u16	tbat_thr_max;
	u16	tbat_thr_highns;
	u16	tbat_thr_limit;
	u16	tjunc_thr_limit;
	u16	ichg_av_thr_min;
	u16	ichg_av_thr_max;
};

struct da9052_bat_hysteresis {
	u16	bat_volt_arr[3];
	u16	array_hys_batvoltage[2];
	u16	upper_limit;
	u16	lower_limit;
	u8	index;
	u8	hys_flag;
};

struct da9052_bat_status {
	u8	cal_capacity;
	u8	charging_mode;
	u8	charger_type;
	u8	health;
	u8	status;
	u8	illegalbattery;
};

struct monitoring_state {
	u16	vddout_value;
	u16	bat_temp_value;
	u16	current_value;
	u16	junc_temp_value;
	u8	bat_level;
	u8	vddout_status:1;
	u8	bat_temp_status:1;
	u8	junc_temp_status:1;
	u8	current_status:1;
	u8	bat_level_status:1;
};

struct da9052_bat_device {
	char	manufacture[32];
	u16	chg_current_raw[DA9052_NUMBER_OF_STORE_CURENT_READING];
	u16	chg_current;
	u16	chg_junc_temp;
	u16	bat_voltage;
	u16	backup_bat_voltage;
	u16	bat_temp;
	u16	vddout;
};

/* Below macro is for testing purpose only */
#define DA9052_BAT_STATUS  1

#if (DA9052_BAT_STATUS == 1)
struct bat_thread_type {
	u8 			pid;
	u8 			state;
	struct completion 	notifier;
	struct task_struct	*thread_task;
};
#endif

struct da9052_charger_device {
	struct da9052_bat_threshold	threshold;
	struct da9052			*da9052;
	struct da9052_bat_platform_data	*bat_pdata;
	struct delayed_work		work;
#if (DA9052_BAT_STATUS == 1)
	struct bat_thread_type	print_bat_status;
#endif
	struct power_supply		psy;
	struct da9052_eh_nb		vddlow_eh_data;
	struct da9052_eh_nb		tbat_eh_data;
	u16				monitoring_interval;
	u8				hys_flag;
	u16				charger_voltage_drop;
	u16				bat_target_voltage;
	u16				voltage_threshold;
	u16				dcin_current;
	u16				vbus_current;
	u16				usb_charger_current;;
	u16				chg_end_current;
	u16				precharging_current;
	u16				charging_time;
	u8				timer_mode:1;
	u8				charger_buck_lp:1;
	u8				usb_charger_det:1;
	u8				ichg_low_cntr:1;
	u8				sw_temp_cntr:1;
	u8				auto_temp_cntr:1;
};

static inline  u8 bat_temp_reg_to_C(u16 value) { return (55 - value); }
static inline  u8 bat_mV_to_reg(u16 value) { return (((value-4100)/100)<<4); }
static inline  u8 bat_drop_mV_to_reg(u16 value)
		{ return (((value-100)/100)<<6); }
static inline  u16 bat_reg_to_mV(u8 value) { return ((value*100) + 4100); }
static inline  u16 bat_drop_reg_to_mV(u8 value) { return ((value*100)+100); }
static inline  u8 vch_thr_mV_to_reg(u16 value) { return ((value-3700)/100); }
static inline  u8 precharge_mA_to_reg(u8 value) { return ((value/20)<<6); }
static inline  u8 vddout_mon_mV_to_reg(u16 value)
		{ return (((value-2500)*128)/1000); }
static inline  u16 vddout_reg_to_mV(u8 value)
		{ return ((value*1000)/128)+2500; }
static inline  u16 volt_reg_to_mV(u16 value)
		{ return ((value*1000)/512)+2500; }
static inline  u8 ichg_mA_to_reg(u16 value) { return (value/4); }
static inline  u16 ichg_reg_to_mA(u8 value) { return ((value*3900)/1000); }
static inline u8 iset_mA_to_reg(u16 iset_value)
		{\
		if ((70 <= iset_value) && (iset_value <= 120)) \
			return (iset_value-70)/10; \
		else if ((400 <= iset_value) && (iset_value <= 700)) \
			return ((iset_value-400)/50)+6; \
		else if ((900 <= iset_value) && (iset_value <= 1300)) \
			return ((iset_value-900)/200)+13; else return 0;
		}

#define DA9052_BAT_DEBUG 		0

#define DA9052_BAT_PROFILE		0
#define SUCCESS				0
#define FAILURE				1

#define TRUE				1
#define FALSE				0

#define set_bits(value, mask)		(value | mask)
#define clear_bits(value, mask)		(value & ~(mask))

#undef DA9052_DEBUG
#if DA9052_BAT_DEBUG
#define DA9052_DEBUG(fmt, args...) printk(KERN_CRIT "" fmt, ##args)
#else
#define DA9052_DEBUG(fmt, args...)
#endif


/* SSC Read or Write Error */
#define DA9052_SSC_FAIL			150

/* To enable debug output for your module, set this to 1 */
#define		DA9052_SSC_DEBUG	0

#undef DA9052_DEBUG
#if DA9052_SSC_DEBUG
#define DA9052_DEBUG(fmt, args...) printk(KERN_CRIT "" fmt, ##args)
#else
#define DA9052_DEBUG(fmt, args...)
#endif


#endif /* __LINUX_MFD_DA9052_BAT_H */
