/*
 * da9052 TSI module declarations.
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

#ifndef __LINUX_MFD_DA9052_TSI_H
#define __LINUX_MFD_DA9052_TSI_H

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/tsi_filter.h>
#include <linux/mfd/da9052/tsi_calibrate.h>
#include <linux/mfd/da9052/pm.h>

#define DA9052_TSI_DEVICE_NAME		"da9052-tsi"
#define DA9052_TSI_INPUT_DEV		DA9052_TSI_DEVICE_NAME

#define TSI_VERSION 			0x0101
#define DA9052_VENDOR_ID		0x15B6
#define DA9052_PRODUCT_ID		0x9052

#define TSI_INPUT_DEVICE_OFF		0
#define NUM_INPUT_DEVS			1

#define DA9052_DISPLAY_X_MAX 		0x3FF
#define DA9052_DISPLAY_Y_MAX 		0x3FF
#define DA9052_TOUCH_PRESSURE_MAX	0x3FF

#define DA9052_TCA_AUTO_TSI_ENABLE				(1<<0)
#define DA9052_TCA_PEN_DET_ENABLE				(1<<1)
#define DA9052_TCA_TSI_XP_MODE_ENABLE				(1<<2)

#define DA9052_TCA_TSI_DELAY_0SLOTS				(0<<6)
#define DA9052_TCA_TSI_DELAY_2SLOTS				(2<<6)
#define DA9052_TCA_TSI_DELAY_4SLOTS				(3<<6)

#define DA9052_TCA_TSI_SEL_XPLUS				(1<<0)
#define DA9052_TCA_TSI_SEL_XMINUS				(1<<1)
#define DA9052_TCA_TSI_SEL_YPLUS				(1<<2)
#define DA9052_TCA_TSI_SEL_YMINUS				(1<<3)

#define DA9052_TCA_TSI_MUX_XPLUS_ROUTED_ADCIN7			(0<<4)
#define DA9052_TCA_TSI_MUX_YPLUS_ROUTED_ADCIN7			(1<<4)
#define DA9052_TCA_TSI_MUX_XMINUS_ROUTED_ADCIN7			(2<<4)
#define DA9052_TCA_TSI_MUX_YMINUS_ROUTED_ADCIN7			(3<<4)

#define DA9052_TCA_TSI_MAN_ENABLE				(1<<6)
#define DA9052_TCA_TSI_SET_TSIREF				(0<<7)
#define DA9052_TCA_TSI_SET_XY_REF				(1<<7)

#define DA9052_EVETN_B_E_PEN_DOWN				(1<<6)
#define DA9052_EVENT_B_E_TSI_READY				(1<<7)

#define DA9052_IRQMASK_B_PENDOWN_MASK   			(1<<6)
#define DA9052_IRQMASK_B_TSI_READY_MASK 			(1<<7)

#define X_LSB_SHIFT 	(0)
#define Y_LSB_SHIFT 	(2)
#define Z_LSB_SHIFT 	(4)
#define PEN_DET_SHIFT 	(6)
#define X_MSB_SHIFT	(2)
#define Y_MSB_SHIFT	(2)
#define Z_MSB_SHIFT	(2)
#define X_LSB_MASK	(11 << X_LSB_SHIFT)
#define Y_LSB_MASK	(11 << Y_LSB_SHIFT)
#define Z_LSB_MASK	(11 << Z_LSB_SHIFT)
#define PEN_DET_MASK	(11 << PEN_DET_SHIFT)

#define TSI_FIFO_SIZE		16

#define INVALID_LDO9_VOLT_VALUE			17

#define set_bits(value, mask)		(value | mask)
#define clear_bits(value, mask)		(value & ~(mask))

#define SUCCESS		0
#define FAILURE		1

#define SET		1
#define	RESET		0
#define	CLEAR		0

#define ENABLE		1
#define DISABLE		0

#define TRUE		1
#define FALSE		0

#define incr_with_wrap_reg_fifo(x)			\
		if(++x >= TSI_REG_DATA_BUF_SIZE)	\
			x = 0

#define incr_with_wrap(x)	\
		if(++x >= TSI_FIFO_SIZE)	\
			x = 0

#undef DA9052_DEBUG
#if DA9052_TSI_DEBUG
#define DA9052_DEBUG( fmt, args... ) printk( KERN_CRIT "" fmt, ##args )
#else
#define DA9052_DEBUG( fmt, args... )
#endif

enum ADC_MODE {

	ECONOMY_MODE = 0,
	FAST_MODE = 1
};

enum TSI_DELAY {
	TSI_DELAY_0SLOTS = 0,
	TSI_DELAY_1SLOTS = 1,
	TSI_DELAY_2SLOTS = 2,
	TSI_DELAY_4SLOTS = 3
};

enum TSI_SLOT_SKIP{
	TSI_SKIP_0SLOTS = 0,
	TSI_SKIP_2SLOTS = 1,
	TSI_SKIP_5SLOTS = 2,
	TSI_SKIP_10SLOTS = 3,
	TSI_SKIP_30SLOTS = 4,
	TSI_SKIP_80SLOTS = 5,
	TSI_SKIP_130SLOTS = 6, 
	TSI_SKIP_330SLOTS = 7 
};


enum TSI_MUX_SEL
{
	TSI_MUX_XPLUS	= 0,
	TSI_MUX_YPLUS 	= 1,
	TSI_MUX_XMINUS	= 2,
	TSI_MUX_YMINUS	= 3
};


enum TSI_IRQ{
	TSI_PEN_DWN,
	TSI_DATA_RDY
};


enum TSI_COORDINATE{
	X_COORDINATE,
	Y_COORDINATE,
	Z_COORDINATE
};

enum TSI_MEASURE_SEQ{
	XYZP_MODE,
	XP_MODE
};

enum TSI_STATE {
	TSI_AUTO_MODE,
	TSI_MANUAL_COORD_X,
	TSI_MANUAL_COORD_Y,
	TSI_MANUAL_COORD_Z,
	TSI_MANUAL_SET,
	TSI_IDLE
};

union da9052_tsi_cont_reg {
	u8 da9052_tsi_cont_a;
	struct{
		u8 auto_tsi_en:1;
		u8 pen_det_en:1;
		u8 tsi_mode:1;
		u8 tsi_skip:3;
		u8 tsi_delay:2;
	}tsi_cont_a;
};

union da9052_tsi_man_cont_reg {
	u8 da9052_tsi_cont_b;
	struct{
		u8 tsi_sel_0:1;
		u8 tsi_sel_1:1;
		u8 tsi_sel_2:1;
		u8 tsi_sel_3:1;
		u8 tsi_mux:2;
		u8 tsi_man:1;
		u8 tsi_adc_ref:1;
	}tsi_cont_b;
};

struct da9052_tsi_conf {
	union da9052_tsi_cont_reg	auto_cont;
	union da9052_tsi_man_cont_reg	man_cont;
	u8 				tsi_adc_sample_intervel:1;
	enum TSI_STATE 			state;
	u8 				ldo9_en:1;
	u8 				ldo9_conf:1;
	u8 				tsi_ready_irq_mask:1;
	u8 				tsi_pendown_irq_mask:1;
};


struct da9052_tsi_reg {
 	u8	x_msb;
	u8	y_msb;
	u8	z_msb;
	u8	lsb;
 };


struct da9052_tsi_reg_fifo {
	struct semaphore	lock;
	s32			head;
	s32 			tail;
	struct da9052_tsi_reg	data[TSI_REG_DATA_BUF_SIZE];
};

struct da9052_tsi_info {
	struct  da9052_tsi_conf  tsi_conf;
	struct input_dev	*input_devs[NUM_INPUT_DEVS];
	struct calib_cfg_t	*tsi_calib;
	u32 			tsi_data_poll_interval;
	u32			tsi_penup_count;
	u32			tsi_zero_data_cnt;
	u8			pen_dwn_event;
	u8			tsi_rdy_event;
	u8			pd_reg_status;	
	u8			datardy_reg_status;
}; 
 
struct da9052_tsi {
	struct da9052_tsi_reg tsi_fifo[TSI_FIFO_SIZE];
	struct mutex tsi_fifo_lock;
	u8 tsi_sampling;
	u8 tsi_state;
	u32 tsi_fifo_start;
	u32 tsi_fifo_end;
};
 
 struct da9052_ts_priv {
	struct da9052	*da9052;
	struct da9052_eh_nb pd_nb;
	struct da9052_eh_nb datardy_nb;

	struct tsi_thread_type	tsi_reg_proc_thread;
	struct tsi_thread_type tsi_raw_proc_thread;

	struct da9052_tsi_platform_data *tsi_pdata;

	struct da9052_tsi_reg_fifo	tsi_reg_fifo;
	struct da9052_tsi_raw_fifo 	tsi_raw_fifo;

	u32 tsi_reg_data_poll_interval;
	u32 tsi_raw_data_poll_interval;

	u8 early_data_flag;
	u8 debounce_over;
	u8 win_reference_valid;

	int os_data_cnt;
	int raw_data_cnt;
};

static inline u8  mask_pendwn_irq(u8 val)
{ 
	return (val |= DA9052_IRQMASKB_MPENDOWN);
}

static inline u8  unmask_pendwn_irq(u8 val)
{ 
	return (val &= ~DA9052_IRQMASKB_MPENDOWN);
}

static inline u8  mask_tsi_rdy_irq(u8 val) 
{
	return (val |=DA9052_IRQMASKB_MTSIREADY);
}

static inline u8  unmask_tsi_rdy_irq(u8 val)
{ 
	return (val &= ~DA9052_IRQMASKB_MTSIREADY);
}

static inline u8  enable_ldo9(u8 val) 
{
	return (val |=DA9052_LDO9_LDO9EN);
}

static inline u8  disable_ldo9(u8 val)
{
	return (val &= ~DA9052_LDO9_LDO9EN);
} 

static inline u8  set_auto_tsi_en(u8 val)
{
	return (val |=DA9052_TSICONTA_AUTOTSIEN);
}

static inline u8  reset_auto_tsi_en(u8 val)
{
	return (val &=~DA9052_TSICONTA_AUTOTSIEN);
}

static inline u8  enable_pen_detect(u8 val)
{
	return (val |=DA9052_TSICONTA_PENDETEN);
}

static inline u8  disable_pen_detect(u8 val) 
{
	return (val &=~DA9052_TSICONTA_PENDETEN);
}

static inline u8  enable_xyzp_mode(u8 val)
{
	return (val &= ~DA9052_TSICONTA_TSIMODE);
}

static inline u8  enable_xp_mode(u8 val)
{
	return(val |= DA9052_TSICONTA_TSIMODE);
}

static inline u8  enable_tsi_manual_mode(u8 val)
{
	return(val |= DA9052_TSICONTB_TSIMAN);
}

static inline u8  disable_tsi_manual_mode(u8 val)
{
	return(val &= ~DA9052_TSICONTB_TSIMAN);
}

static inline u8 tsi_sel_xplus_close(u8 val)
{
	return(val |= DA9052_TSICONTB_TSISEL0);
}

static inline u8 tsi_sel_xplus_open(u8 val)
{
	return(val &= ~DA9052_TSICONTB_TSISEL0);
}

static inline u8 tsi_sel_xminus_close(u8 val)
{
	return(val |= DA9052_TSICONTB_TSISEL1);
}

static inline u8 tsi_sel_xminus_open(u8 val)
{
	return(val &= ~DA9052_TSICONTB_TSISEL1);
}

static inline u8 tsi_sel_yplus_close(u8 val)
{
	return(val |= DA9052_TSICONTB_TSISEL2);
}

static inline u8 tsi_sel_yplus_open(u8 val)
{
	return(val &= ~DA9052_TSICONTB_TSISEL2);
}

static inline u8 tsi_sel_yminus_close(u8 val)
{
	return(val |= DA9052_TSICONTB_TSISEL3);
}

static inline u8 tsi_sel_yminus_open(u8 val)
{
	return(val &= ~DA9052_TSICONTB_TSISEL3);
}

static inline u8 adc_mode_economy_mode(u8 val)
{
	return(val &= ~DA9052_ADCCONT_ADCMODE);
}

static inline u8 adc_mode_fast_mode(u8 val)
{
	return(val |= DA9052_ADCCONT_ADCMODE);
}
int da9052_tsi_get_calib_display_point(struct da9052_tsi_data *display);

struct da9052_ldo_config {
	u16 		ldo_volt;
	u8	 	ldo_num;
	u8 		ldo_conf:1;
	u8 		ldo_pd:1;
};

static inline  u8 ldo9_mV_to_reg(u16 value)
{
	return ((value - DA9052_LDO9_VOLT_LOWER)/DA9052_LDO9_VOLT_STEP);
}

static inline  u8 validate_ldo9_mV(u16 value)
{
	if ((value >= DA9052_LDO9_VOLT_LOWER) && \
					(value <= DA9052_LDO9_VOLT_UPPER))
		return 
			(((value - DA9052_LDO9_VOLT_LOWER) % DA9052_LDO9_VOLT_STEP > 0) ? -1 : 0);
	return FAILURE;
}

s32 da9052_tsi_raw_proc_thread (void *ptr);
void __init da9052_init_tsi_fifos (struct da9052_ts_priv *priv);
void clean_tsi_fifos(struct da9052_ts_priv *priv);
u32 get_reg_data_cnt (struct da9052_ts_priv *priv);
u32 get_reg_free_space_cnt(struct da9052_ts_priv *priv);
void da9052_tsi_process_reg_data(struct da9052_ts_priv *priv);
void da9052_tsi_pen_down_handler(struct da9052_eh_nb *eh_data, u32 event);
void da9052_tsi_data_ready_handler(struct da9052_eh_nb *eh_data, u32 event);

#endif /* __LINUX_MFD_DA9052_TSI_H */
