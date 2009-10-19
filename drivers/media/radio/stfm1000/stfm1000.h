/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef STFM1000_H
#define STFM1000_H

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/irq.h>
#include <media/videobuf-dma-sg.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <mach/dma.h>

#include "stfm1000-regs.h"

#include "stfm1000-filter.h"
#include "stfm1000-rds.h"

struct stfm1000 {
	struct list_head devlist;
	int idx;

	struct i2c_client *client;
	struct video_device radio;

	/* alsa */
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	struct stmp3xxx_dma_descriptor *dma;
	int desc_num;
	int dma_ch;
	int dma_irq;
	int attn_irq;

	struct mutex state_lock;
	int read_count;
	int read_offset;
	int blocks;
	int blksize;
	int bufsize;

	struct mutex deffered_work_lock;
	struct execute_work snd_capture_start_work;
	struct execute_work snd_capture_stop_work;

	int now_recording;
	int alsa_initialized;
	int stopping_recording;

	/* actual DRI buffer */
	dma_addr_t dri_phys;
	void *dri_buf;
	int dri_bufsz;

	/* various */
	u16 curvol;
	int users;
	int removed;
	struct mutex xfer_lock;
	u8 revid;

	unsigned int dbgflg;

	/* shadow registers */
	u32 shadow_regs[STFM1000_NUM_REGS];
	u32 reg_rw_set[(STFM1000_NUM_REGS + 31) / 32];
	u32 reg_ra_set[(STFM1000_NUM_REGS + 31) / 32];
	u32 reg_dirty_set[(STFM1000_NUM_REGS + 31) / 32];

	/* tuning parameters (not everything is used for now) */
	u16 tune_rssi_th;		/* sd_ctl_TuneRssiTh_u16 */
	u16 tune_mpx_dc_th;		/* sd_ctl_TuneMpxDcTh_u16 */
	u16 adj_chan_th;		/* sd_ctl_AdjChanTh_u16 */
	u16 pilot_est_th;		/* sd_ctl_PilotEstTh_u16 */
	u16 coef_lna_turn_off_th;	/* sd_ctl_pCoefLnaTurnOffTh_u16 */
	u16 coef_lna_turn_on_th;	/* sd_ctl_pCoefLnaTurnOnTh_u16 */
	u16 reg_agc_ref_lna_off;	/* sd_ctl_pRegAgcRefLnaOff_u16 */
	u16 reg_agc_ref_lna_on;		/* sd_ctl_pRegAgcRefLnaOn_u16 */

	u32 sdnominal_pivot;		/* sd_ctl_SdnominalData_u32 */

	/* jiffies of the next monitor cycle */
	unsigned long next_quality_monitor;
	unsigned long next_agc_monitor;

	unsigned int mute : 1;			/* XXX */
	unsigned int lna_driving : 1;		/* sd_ctl_LnaDriving_u1 */
	unsigned int weak_signal : 1;		/* sd_ctl_WeakSignal_u1 */
	unsigned int is_station : 1;		/* XXX */
	unsigned int force_mono : 1;		/* XXX */
	unsigned int signal_indicator : 1;	/* XXX */
	unsigned int stereo_indicator : 1;	/* XXX */
	unsigned int agc_monitor : 1;		/* XXX */
	unsigned int quality_monitor : 1;	/* XXX */
	unsigned int pilot_present : 1;		/* sd_ctl_PilotPresent_u1 */
	unsigned int prev_pilot_present : 1;	/* XXX */
	unsigned int stereo : 1;
	unsigned int active : 1;		/* set when audio enabled */
	unsigned int rds_enable : 1;		/* set when rds is enabled */
	unsigned int rds_present : 1;		/* RDS info present */
	unsigned int rds_sync : 1;		/* RDS force sync */
	unsigned int rds_demod_running : 1;	/* RDS demod is running ATM */
	unsigned int rds_sdnominal_adapt : 1;	/* adapt for better recept. */
	unsigned int rds_phase_pop : 1;		/* enable phase pop */
	unsigned int rds_info : 1;		/* print debugging info RDS */
	unsigned int tuning_grid_50KHz : 1;	/* tuning grid of 50Khz */
	u32 rssi;				/* rssi last decoded frame */
	u16 rssi_dc_est_log;
	u16 signal_strength;			/* is rssi_dc_est_log */
	u16 rds_signal_th;			/* RDS threshold */
	s16 mpx_dc;				/* sd_ctl_ShadowToneData_i16 */

	u32 tune_cap_a_f;			/* float! sd_ctl_TuneCapA_f */
	u32 tune_cap_b_f;			/* float! sd_ctl_TuneCapB_f */

	int monitor_period;			/* period of the monitor */
	int quality_monitor_period;		/* update period in ms */
	int agc_monitor_period;			/* update period in ms */

	int georegion;				/* current graphical region */

	/* last tuned frequency */
	int freq;				/* 88.0 = 8800 */

	/* weak signal processing filter state */
	struct stfm1000_filter_parms filter_parms;

	/* state of rds */
	spinlock_t rds_lock;
	struct stfm1000_rds_state rds_state;
	unsigned int rds_pkt_bad;
	unsigned int rds_pkt_good;
	unsigned int rds_pkt_recovered;
	unsigned int rds_pkt_lost_sync;
	unsigned int rds_bit_overruns;

	/* monitor thread */
	wait_queue_head_t thread_wait;
	unsigned long thread_events;
	struct task_struct *thread;
};

#define EVENT_RDS_BITS		0
#define EVENT_RDS_MIXFILT	1
#define EVENT_RDS_SDNOMINAL	2
#define EVENT_RDS_RESET		3

#define STFM1000_DBGFLG_I2C	(1 << 0)

static inline struct stfm1000 *stfm1000_from_file(struct file *file)
{
	return container_of(video_devdata(file), struct stfm1000, radio);
}

/* in stfm1000-i2c.c */

/* setup reg set */
void stfm1000_setup_reg_set(struct stfm1000 *stfm1000);

/* direct access to registers bypassing the shadow register set */
int stfm1000_raw_read(struct stfm1000 *stfm1000, int reg, u32 *value);
int stfm1000_raw_write(struct stfm1000 *stfm1000, int reg, u32 value);

/* access using the shadow register set */
int stfm1000_write(struct stfm1000 *stfm1000, int reg, u32 value);
int stfm1000_read(struct stfm1000 *stfm1000, int reg, u32 *value);
int stfm1000_write_masked(struct stfm1000 *stfm1000, int reg, u32 value,
		u32 mask);
int stfm1000_set_bits(struct stfm1000 *stfm1000, int reg, u32 value);
int stfm1000_clear_bits(struct stfm1000 *stfm1000, int reg, u32 value);

struct stfm1000_reg {
	unsigned int regno;
	u32 value;
};

#define STFM1000_REG_END	-1
#define STFM1000_REG_DELAY	-2

#define STFM1000_REG_SET_BITS_MASK	0x1000
#define STFM1000_REG_CLEAR_BITS_MASK	0x2000

#define STFM1000_REG(r, v) \
	{ .regno = STFM1000_ ## r , .value = (v) }

#define STFM1000_END \
	{ .regno = STFM1000_REG_END }

#define STFM1000_DELAY(x) \
	{ .regno = STFM1000_REG_DELAY, .value = (x) }

#define STFM1000_REG_SETBITS(r, v) \
	{ .regno = STFM1000_ ## r | STFM1000_REG_SET_BITS_MASK, \
		.value = (v) }

#define STFM1000_REG_CLRBITS(r, v) \
	{ .regno = STFM1000_ ## r | STFM1000_REG_CLEAR_BITS_MASK, \
		.value = (v) }

int stfm1000_write_regs(struct stfm1000 *stfm1000,
		const struct stfm1000_reg *reg);

/* in stfm1000-precalc.c */
extern const struct stfm1000_tune1
stfm1000_tune1_table[STFM1000_FREQUENCY_100KHZ_RANGE];

/* exported for use by alsa driver */

struct stfm1000_dri_sample {
	/* L+R */
	u16 l_plus_r;
	/* L-R */
	u16 l_minus_r;
	/* Rx signal strength channel */
	u16 rssi;
	/* Radio data service channel */
	u16 rds;
};

struct stfm1000_alsa_ops {
	int  (*init)(struct stfm1000 *stfm1000);
	void  (*release)(struct stfm1000 *stfm1000);
	void (*dma_irq)(struct stfm1000 *stfm1000);
	void (*attn_irq)(struct stfm1000 *stfm1000);
};

extern struct list_head stfm1000_devlist;
extern struct stfm1000_alsa_ops *stfm1000_alsa_ops;

/* needed for setting the interrupt handlers from alsa */
irqreturn_t stfm1000_dri_attn_irq(int irq, void *dev_id);
irqreturn_t stfm1000_dri_dma_irq(int irq, void *dev_id);
void stfm1000_decode_block(struct stfm1000 *stfm1000, const s16 *src, s16 *dst, int count);
void stfm1000_take_down(struct stfm1000 *stfm1000);
void stfm1000_bring_up(struct stfm1000 *stfm1000);
void stfm1000_tune_current(struct stfm1000 *stfm1000);

void stfm1000_monitor_signal(struct stfm1000 *stfm1000, int bit);

#endif
