// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC Timer driver
 * Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/fsl/netc_global.h>

#define NETC_TMR_CTRL			0x0080
#define  TMR_CTRL_CK_SEL		GENMASK(1, 0)
#define  TMR_CTRL_TE			BIT(2)
#define  TMR_ETEP1			BIT(8)
#define  TMR_ETEP2			BIT(9)
#define  TMR_COMP_MODE			BIT(15)
#define  TMR_CTRL_TCLK_PERIOD		GENMASK(25, 16)
#define  TMR_CTRL_PP2L			BIT(26)
#define  TMR_CTRL_PP1L			BIT(27)
#define  TMR_CTRL_FS			BIT(28)
#define  TMR_ALARM1P			BIT(31)

#define NETC_TMR_TEVENT			0x0084
#define  TMR_TEVNET_PPEN(a)		BIT(7 - (a))
#define  TMR_TEVENT_PPEN_ALL		GENMASK(7, 5)
#define  TMR_TEVENT_ALM1EN		BIT(16)
#define  TMR_TEVENT_ALM2EN		BIT(17)
#define  TMR_TEVENT_ETS1_THREN		BIT(20)
#define  TMR_TEVENT_ETS2_THREN		BIT(21)
#define  TMR_TEVENT_ETS1EN		BIT(24)
#define  TMR_TEVENT_ETS2EN		BIT(25)
#define  TMR_TEVENT_ETS1_OVEN		BIT(28)
#define  TMR_TEVENT_ETS2_OVEN		BIT(29)
#define  TMR_TEVENT_ETS1		(TMR_TEVENT_ETS1_THREN | \
					 TMR_TEVENT_ETS1EN | TMR_TEVENT_ETS1_OVEN)
#define  TMR_TEVENT_ETS2		(TMR_TEVENT_ETS2_THREN | \
					 TMR_TEVENT_ETS2EN | TMR_TEVENT_ETS2_OVEN)

#define NETC_TMR_TEMASK			0x0088
#define NETC_TMR_STAT			0x0094
#define  TMR_STAT_ETS1_VLD		BIT(24)
#define  TMR_STAT_ETS2_VLD		BIT(25)

#define NETC_TMR_CNT_L			0x0098
#define NETC_TMR_CNT_H			0x009c
#define NETC_TMR_ADD			0x00a0
#define NETC_TMR_ACC			0x00a4
#define NETC_TMR_PRSC			0x00a8
#define NETC_TMR_ECTRL			0x00ac
#define NETC_TMR_OFF_L			0x00b0
#define NETC_TMR_OFF_H			0x00b4

/* a = 0 or 1, a = 0 indicates TMR_ALARM1, a = 1 indicates TMR_ALARM2 */
#define NETC_TMR_ALARM_L(a)		(0x00b8 + (a) * 8)
#define NETC_TMR_ALARM_H(a)		(0x00bc + (a) * 8)

#define NETC_TMR_ALARM_CTRL		0x00cc
#define  ALARM_CTRL_PW(a)		(GENMASK(4, 0) << (a) * 8)
#define  ALARM_CTRL_PG(a)		(BIT(7) << (a) * 8)

/* a = 0, 1, 2. a = 0 indicates TMR_FIPER1, a = 1 indicates TMR_FIPER2,
 * a = 2 indicates TMR_FIPER3.
 */
#define NETC_TMR_FIPER(a)		(0x00d0 + (a) * 4)

#define NETC_TMR_FIPER_CTRL		0x00dc
#define  FIPER_CTRL_PW(a)		(GENMASK(4, 0) << (a) * 8)
#define  FIPER_CTRL_SET_PW(a, w)	((w) << 8 * (a))
#define  FIPER_CTRL_FS_ALARM(a)		(BIT(5) << (a) * 8)
#define  FIPER_CTRL_PG(a)		(BIT(6) << (a) * 8)
#define  FIPER_CTRL_DIS(a)		(BIT(7) << (a) * 8)

#define NETC_TMR_ETTS1_L		0x00e0
#define NETC_TMR_ETTS1_H		0x00e4
#define NETC_TMR_ETTS2_L		0x00e8
#define NETC_TMR_ETTS2_H		0x00ec
#define NETC_TMR_CUR_TIME_L		0x00f0
#define NETC_TMR_CUR_TIME_H		0x00f4
#define NETC_TMR_PARAM			0x00f8

#define NETC_TMR_REGS_BAR		0
#define NETC_TMR_FIPER_NUM		3
#define NETC_TMR_DEFAULT_PRSC		2
#define NETC_TMR_DEFAULT_ALARM		0xffffffffffffffffULL
#define NETC_TMR_DEFAULT_FIPER		0xffffffff
#define NETC_TMR_PRSC_OCK_MAX		0xfffe

/* 1588 timer reference clock source select */
#define NETC_TMR_CCM_TIMER1		0 /* enet_timer1_clk_root, from CCM */
#define NETC_TMR_SYSTEM_CLK		1 /* enet_clk_root/2, from CCM */
#define NETC_TMR_EXT_OSC		2 /* tmr_1588_clk, from IO pins */

#define NETC_TMR_SYSCLK_RATE		333333333UL

#define NETC_TMR_FIPER_PW		0x1f
#define NETC_TMR_ETTS_NUM		2
#define NETC_TMR_ALARM_NUM		2
#define NETC_TMR_DEFAULT_ETTF_THR	7
#define NETC_TMR_DEFAULT_PPS_FIPER	0

#define NETC_GLOBAL_OFFSET		0x10000
#define NETC_GLOBAL_IPBRR0		0xbf8
#define  IPBRR0_IP_REV			GENMASK(15, 0)
#define NETC_REV_4_1			0x0401

#define netc_timer_rd(p, o)		netc_read((p)->base + (o))
#define netc_timer_wr(p, o, v)		netc_write((p)->base + (o), v)

enum netc_pp_type {
	NETC_PP_PPS = 1,
	NETC_PP_PEROUT,
};

struct netc_pp {
	enum netc_pp_type type;
	bool enabled;
	int alarm_id;
	u32 period; /* pulse period, ns */
	u64 stime; /* start time, ns */
};

struct netc_timer {
	void __iomem *base;
	struct device *dev;
	struct pci_dev *pci_dev;
	int irq;
	char irq_name[64];
	spinlock_t lock; /* protect regs */

	struct ptp_clock *clock;
	struct ptp_clock_info caps;
	int phc_index;
	struct clk *src_clk;
	u32 clk_select;
	u32 clk_freq;
	u32 period_int;
	/* fractional part of clock period * BIT(32) */
	u32 period_frac;
	/* High 32 bits are the integer part, low 32 bits
	 * are the fractional part
	 */
	u64 base_period;
	u32 oclk_prsc; /* must be an even value */
	struct netc_pp pp[NETC_TMR_FIPER_NUM]; /* periodic pulse */

	u8 pps_channel;
	u8 alarm_bitmap;
	u8 alarm_num;
	struct dentry *debugfs_root;
};

#define ptp_to_netc_timer(ptp)		container_of((ptp), struct netc_timer, caps)

static u64 netc_timer_cnt_read(struct netc_timer *priv)
{
	u32 tmr_cnt_l, tmr_cnt_h;
	u64 ns;

	/* The user must read the TMR_CNC_L register first to get
	 * correct 64-bit TMR_CNT_H/L counter values.
	 */
	tmr_cnt_l = netc_timer_rd(priv, NETC_TMR_CNT_L);
	tmr_cnt_h = netc_timer_rd(priv, NETC_TMR_CNT_H);
	ns = (((u64)tmr_cnt_h) << 32) | tmr_cnt_l;

	return ns;
}

static u64 netc_timer_cur_time_read(struct netc_timer *priv)
{
	u32 time_h, time_l;
	u64 ns;

	time_l = netc_timer_rd(priv, NETC_TMR_CUR_TIME_L);
	time_h = netc_timer_rd(priv, NETC_TMR_CUR_TIME_H);
	ns = (u64)time_h << 32 | time_l;

	return ns;
}

static u64 netc_timer_offset_read(struct netc_timer *priv)
{
	u32 tmr_off_l, tmr_off_h;
	u64 offset;

	tmr_off_l = netc_timer_rd(priv, NETC_TMR_OFF_L);
	tmr_off_h = netc_timer_rd(priv, NETC_TMR_OFF_H);
	offset = (((u64)tmr_off_h) << 32) | tmr_off_l;

	return offset;
}

static void netc_timer_offset_write(struct netc_timer *priv, u64 offset)
{
	u32 tmr_off_h = upper_32_bits(offset);
	u32 tmr_off_l = lower_32_bits(offset);

	netc_timer_wr(priv, NETC_TMR_OFF_L, tmr_off_l);
	netc_timer_wr(priv, NETC_TMR_OFF_H, tmr_off_h);
}

u64 netc_timer_get_current_time(struct pci_dev *timer_dev)
{
	struct netc_timer *priv;
	u64 cur_time;

	if (!timer_dev)
		return 0;

	priv = pci_get_drvdata(timer_dev);
	if (!priv)
		return 0;

	guard(spinlock_irqsave)(&priv->lock);
	cur_time = netc_timer_cur_time_read(priv);

	return cur_time;
}
EXPORT_SYMBOL_GPL(netc_timer_get_current_time);

static void netc_timer_cnt_write(struct netc_timer *priv, u64 ns)
{
	u32 tmr_cnt_h = upper_32_bits(ns);
	u32 tmr_cnt_l = lower_32_bits(ns);

	/* The user must write to TMR_CNT_L register first. */
	netc_timer_wr(priv, NETC_TMR_CNT_L, tmr_cnt_l);
	netc_timer_wr(priv, NETC_TMR_CNT_H, tmr_cnt_h);
}

static void netc_timer_alarm_write(struct netc_timer *priv,
				   u64 alarm, int index)
{
	u32 alarm_h = upper_32_bits(alarm);
	u32 alarm_l = lower_32_bits(alarm);

	netc_timer_wr(priv, NETC_TMR_ALARM_L(index), alarm_l);
	netc_timer_wr(priv, NETC_TMR_ALARM_H(index), alarm_h);
}

static int netc_timer_get_alarm_id(struct netc_timer *priv)
{
	int i;

	for (i = 0; i < priv->alarm_num; i++) {
		if (!(priv->alarm_bitmap & BIT(i))) {
			priv->alarm_bitmap |= BIT(i);
			break;
		}
	}

	return i;
}

static void netc_timer_set_pps_alarm(struct netc_timer *priv, int channel)
{
	struct netc_pp *pp = &priv->pp[channel];
	u64 alarm;

	if (pp->type != NETC_PP_PPS || !pp->enabled)
		return;

	/* Get the alarm value */
	alarm = netc_timer_cur_time_read(priv) +  NSEC_PER_MSEC;
	alarm = roundup_u64(alarm, NSEC_PER_SEC);
	alarm = roundup_u64(alarm, priv->period_int);

	netc_timer_alarm_write(priv, alarm, pp->alarm_id);
}

static void netc_timer_set_perout_alarm(struct netc_timer *priv, int channel)
{
	u64 cur_time = netc_timer_cur_time_read(priv);
	struct netc_pp *pp = &priv->pp[channel];
	u64 alarm, delta, min_time;
	u32 period = pp->period;
	u64 stime = pp->stime;

	min_time = cur_time + NSEC_PER_MSEC + period;
	if (stime < min_time) {
		delta = min_time - stime;
		stime += roundup_u64(delta, period);
	}

	alarm = roundup_u64(stime - period, priv->period_int);
	netc_timer_alarm_write(priv, alarm, pp->alarm_id);
}

static void netc_timer_disable_fiper(struct netc_timer *priv)
{
	u32 fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	int i;

	for (i = 0; i < NETC_TMR_FIPER_NUM; i++) {
		struct netc_pp *pp = &priv->pp[i];

		if (!pp->enabled)
			continue;

		fiper_ctrl |= FIPER_CTRL_DIS(i);
	}

	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);
}

static void netc_timer_enable_fiper(struct netc_timer *priv)
{
	u32 fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	int i;

	for (i = 0; i < NETC_TMR_FIPER_NUM; i++) {
		struct netc_pp *pp = &priv->pp[i];

		if (!pp->enabled)
			continue;

		fiper_ctrl &= ~FIPER_CTRL_DIS(i);

		if (pp->type == NETC_PP_PPS)
			netc_timer_set_pps_alarm(priv, i);
		else if (pp->type == NETC_PP_PEROUT)
			netc_timer_set_perout_alarm(priv, i);
	}

	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);
}

static u64 netc_timer_get_gclk_period(struct netc_timer *priv)
{
	u64 dividend = (u64)NSEC_PER_SEC * priv->oclk_prsc;

	/* TMR_GCLK_freq = (clk_freq / oclk_prsc) Hz.
	 * TMR_GCLK_period = NSEC_PER_SEC / TMR_GCLK_freq.
	 * TMR_GCLK_period = (NSEC_PER_SEC * oclk_prsc) / clk_freq
	 */

	return div_u64(dividend, priv->clk_freq);
}

static u32 netc_timer_calculate_fiper_pulse_width(struct netc_timer *priv,
						  u32 fiper)
{
	u64 pw;

	/* Set the FIPER pulse width to half FIPER interval by default.
	 * pulse_width = (fiper / 2) / TMR_GCLK_period,
	 * TMR_GCLK_period = NSEC_PER_SEC / TMR_GCLK_freq,
	 * TMR_GCLK_freq = (clk_freq / oclk_prsc) Hz,
	 * so pulse_width = fiper * clk_freq / (2 * NSEC_PER_SEC * oclk_prsc).
	 */
	pw = (u64)fiper * priv->clk_freq;
	pw = div64_u64(pw, 2000000000UL * priv->oclk_prsc);

	/* The FIPER_PW field only has 5 bits, need to update oclk_prsc */
	if (pw > NETC_TMR_FIPER_PW)
		pw = NETC_TMR_FIPER_PW;

	return pw;
}

static void netc_timer_adjust_period(struct netc_timer *priv, u64 period)
{
	u32 period_frac = lower_32_bits(period);
	u32 period_int = upper_32_bits(period);
	u32 tmr_ctrl, old_tmr_ctrl;

	guard(spinlock_irqsave)(&priv->lock);
	old_tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
	tmr_ctrl = u32_replace_bits(old_tmr_ctrl, period_int,
				    TMR_CTRL_TCLK_PERIOD);
	if (tmr_ctrl != old_tmr_ctrl)
		netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);

	netc_timer_wr(priv, NETC_TMR_ADD, period_frac);
}

static void netc_timer_handle_etts_event(struct netc_timer *priv, int index,
					 bool update_event)
{
	u32 regoff_l, regoff_h, etts_l, etts_h, ets_vld;
	struct ptp_clock_event event;

	switch (index) {
	case 0:
		ets_vld = TMR_STAT_ETS1_VLD;
		regoff_l = NETC_TMR_ETTS1_L;
		regoff_h = NETC_TMR_ETTS1_H;
		break;
	case 1:
		ets_vld = TMR_STAT_ETS2_VLD;
		regoff_l = NETC_TMR_ETTS2_L;
		regoff_h = NETC_TMR_ETTS2_H;
		break;
	default:
		return;
	}

	if (!(netc_timer_rd(priv, NETC_TMR_STAT) & ets_vld))
		return;

	do {
		etts_l = netc_timer_rd(priv, regoff_l);
		etts_h = netc_timer_rd(priv, regoff_h);
	} while (netc_timer_rd(priv, NETC_TMR_STAT) & ets_vld);

	if (update_event) {
		event.type = PTP_CLOCK_EXTTS;
		event.index = index;
		event.timestamp = ((u64) etts_h) << 32;
		event.timestamp |= etts_l;
		ptp_clock_event(priv->clock, &event);
	}
}

static irqreturn_t netc_timer_isr(int irq, void *data)
{
	struct netc_timer *priv = data;
	struct ptp_clock_event event;
	u32 tmr_event, tmr_emask;

	guard(spinlock_irqsave)(&priv->lock);

	tmr_event = netc_timer_rd(priv, NETC_TMR_TEVENT);
	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);

	tmr_event &= tmr_emask;
	if (tmr_event & TMR_TEVENT_PPEN_ALL) {
		event.type = PTP_CLOCK_PPS;
		ptp_clock_event(priv->clock, &event);
	}

	if (tmr_event & TMR_TEVENT_ALM1EN)
		netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 0);

	if (tmr_event & TMR_TEVENT_ALM2EN)
		netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 1);

	if (tmr_event & TMR_TEVENT_ETS1)
		netc_timer_handle_etts_event(priv, 0, true);

	if (tmr_event & TMR_TEVENT_ETS2)
		netc_timer_handle_etts_event(priv, 1, true);

	/* Clear interrupts status */
	netc_timer_wr(priv, NETC_TMR_TEVENT, tmr_event);

	return IRQ_HANDLED;
}

/* ppm: parts per million, ppb: parts per billion */
static int netc_timer_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 new_period;

	if (!scaled_ppm)
		return 0;

	new_period = adjust_by_scaled_ppm(priv->base_period, scaled_ppm);
	netc_timer_adjust_period(priv, new_period);

	return 0;
}

static int netc_timer_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 tmr_cnt, tmr_off;

	guard(spinlock_irqsave)(&priv->lock);

	netc_timer_disable_fiper(priv);

	tmr_off = netc_timer_offset_read(priv);
	if (delta < 0 && tmr_off < abs(delta)) {
		delta += tmr_off;
		if (!tmr_off)
			netc_timer_offset_write(priv, 0);

		tmr_cnt = netc_timer_cnt_read(priv);
		tmr_cnt += delta;
		netc_timer_cnt_write(priv, tmr_cnt);
	} else {
		tmr_off += delta;
		netc_timer_offset_write(priv, tmr_off);
	}

	netc_timer_enable_fiper(priv);

	return 0;
}

static int netc_timer_gettimex64(struct ptp_clock_info *ptp, struct timespec64 *ts,
				 struct ptp_system_timestamp *sts)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 ns;

	scoped_guard(spinlock_irqsave, &priv->lock) {
		ptp_read_system_prets(sts);
		ns = netc_timer_cur_time_read(priv);
		ptp_read_system_postts(sts);
	}

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int netc_timer_settime64(struct ptp_clock_info *ptp,
				const struct timespec64 *ts)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);
	u64 ns = timespec64_to_ns(ts);

	guard(spinlock_irqsave)(&priv->lock);

	netc_timer_disable_fiper(priv);
	netc_timer_offset_write(priv, 0);
	netc_timer_cnt_write(priv, ns);
	netc_timer_enable_fiper(priv);

	return 0;
}

static int netc_timer_enable_pps(struct netc_timer *priv,
				 struct ptp_clock_request *rq, int on)
{
	u32 tmr_emask, fiper, fiper_ctrl, fiper_pw;
	u8 channel = priv->pps_channel;
	struct netc_pp *pp;
	int alarm_id;

	guard(spinlock_irqsave)(&priv->lock);

	pp = &priv->pp[channel];
	if (pp->type == NETC_PP_PEROUT) {
		dev_err(priv->dev,
			"FIPER%u is being used for PEROUT\n", channel);
		return -EBUSY;
	}

	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);

	if (on) {
		if (pp->enabled)
			return 0;

		alarm_id = netc_timer_get_alarm_id(priv);
		if (alarm_id == priv->alarm_num) {
			dev_err(priv->dev, "No available ALARMs\n");
			return -EBUSY;
		}

		pp->enabled = true;
		pp->type = NETC_PP_PPS;
		pp->alarm_id = alarm_id;

		fiper = NSEC_PER_SEC - priv->period_int;
		fiper_pw = netc_timer_calculate_fiper_pulse_width(priv, fiper);
		fiper_ctrl &= ~(FIPER_CTRL_DIS(channel) | FIPER_CTRL_PW(channel) |
				FIPER_CTRL_FS_ALARM(channel));
		fiper_ctrl |= FIPER_CTRL_SET_PW(channel, fiper_pw);
		fiper_ctrl |= alarm_id ? FIPER_CTRL_FS_ALARM(channel) : 0;
		tmr_emask |= TMR_TEVNET_PPEN(channel);
		netc_timer_set_pps_alarm(priv, channel);
	} else {
		if (!pp->enabled)
			return 0;

		priv->alarm_bitmap &= ~BIT(pp->alarm_id);
		memset(pp, 0, sizeof(*pp));

		fiper = NETC_TMR_DEFAULT_FIPER;
		tmr_emask &= ~TMR_TEVNET_PPEN(channel);
		fiper_ctrl |= FIPER_CTRL_DIS(channel);
	}

	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);
	netc_timer_wr(priv, NETC_TMR_FIPER(channel), fiper);
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);

	return 0;
}

static int net_timer_enable_perout(struct netc_timer *priv,
				   struct ptp_clock_request *rq, int on)
{
	u64 alarm, period_ns, gclk_period, max_period, min_period;
	u32 tmr_emask, fiper, fiper_ctrl;
	struct timespec64 period, stime;
	u32 channel, fiper_pw;
	struct netc_pp *pp;
	int alarm_id;

	if (rq->perout.flags)
		return -EOPNOTSUPP;

	channel = rq->perout.index;
	if (channel >= NETC_TMR_FIPER_NUM)
		return -EINVAL;

	guard(spinlock_irqsave)(&priv->lock);

	pp = &priv->pp[channel];
	if (pp->type == NETC_PP_PPS) {
		dev_err(priv->dev,
			"FIPER%u is being used for PPS\n", channel);
		return -EBUSY;
	}

	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	if (!on) {
		if (!pp->enabled)
			return 0;

		tmr_emask &= ~TMR_TEVNET_PPEN(channel);
		alarm = NETC_TMR_DEFAULT_ALARM;
		fiper = NETC_TMR_DEFAULT_FIPER;
		fiper_ctrl |= FIPER_CTRL_DIS(channel);

		alarm_id = pp->alarm_id;
		netc_timer_alarm_write(priv, alarm, alarm_id);
		priv->alarm_bitmap &= ~BIT(alarm_id);
		memset(pp, 0, sizeof(*pp));
	} else {
		period.tv_sec = rq->perout.period.sec;
		period.tv_nsec = rq->perout.period.nsec;
		period_ns = timespec64_to_ns(&period);

		max_period = (u64)NETC_TMR_DEFAULT_FIPER + priv->period_int;
		gclk_period = netc_timer_get_gclk_period(priv);
		min_period = gclk_period * 4 + priv->period_int;
		if (period_ns > max_period || period_ns < min_period) {
			dev_err(priv->dev, "The period range is %llu ~ %llu\n",
				min_period, max_period);
			return -EINVAL;
		}

		stime.tv_sec = rq->perout.start.sec;
		stime.tv_nsec = rq->perout.start.nsec;

		tmr_emask |= TMR_TEVNET_PPEN(channel);

		/* Set to desired FIPER interval in ns - TCLK_PERIOD */
		fiper = period_ns - priv->period_int;
		fiper_pw = netc_timer_calculate_fiper_pulse_width(priv, fiper);

		if (pp->enabled) {
			alarm_id = pp->alarm_id;
		} else {
			alarm_id = netc_timer_get_alarm_id(priv);
			if (alarm_id == priv->alarm_num) {
				dev_err(priv->dev, "No available ALARMs\n");
				return -EBUSY;
			}

			pp->type = NETC_PP_PEROUT;
			pp->enabled = true;
			pp->alarm_id = alarm_id;
		}

		pp->stime = timespec64_to_ns(&stime);
		pp->period = period_ns;

		fiper_ctrl &= ~(FIPER_CTRL_DIS(channel) | FIPER_CTRL_PW(channel) |
				FIPER_CTRL_FS_ALARM(channel));
		fiper_ctrl |= FIPER_CTRL_SET_PW(channel, fiper_pw);
		fiper_ctrl |= alarm_id ? FIPER_CTRL_FS_ALARM(channel) : 0;

		netc_timer_set_perout_alarm(priv, channel);
	}

	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);
	netc_timer_wr(priv, NETC_TMR_FIPER(channel), fiper);
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);

	return 0;
}

static int netc_timer_enable_extts(struct netc_timer *priv,
				   struct ptp_clock_request *rq, int on)
{
	u32 ets_emask, tmr_emask, tmr_ctrl, ettp_bit;
	unsigned long flags;

	/* Reject requests to enable time stamping on both edges */
	if ((rq->extts.flags & PTP_ENABLE_FEATURE) &&
	    (rq->extts.flags & PTP_STRICT_FLAGS) &&
	    (rq->extts.flags & PTP_EXTTS_EDGES) == PTP_EXTTS_EDGES)
		return -EOPNOTSUPP;

	switch (rq->extts.index) {
	case 0:
		ettp_bit = TMR_ETEP1;
		ets_emask = TMR_TEVENT_ETS1;
		break;
	case 1:
		ettp_bit = TMR_ETEP2;
		ets_emask = TMR_TEVENT_ETS2;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);

	netc_timer_handle_etts_event(priv, rq->extts.index, false);
	tmr_emask = netc_timer_rd(priv, NETC_TMR_TEMASK);
	if (on) {
		tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
		if (rq->extts.flags & PTP_FALLING_EDGE)
			tmr_ctrl |= ettp_bit;
		else
			tmr_ctrl &= ~ettp_bit;

		netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);
		tmr_emask |= ets_emask;
	} else {
		tmr_emask &= ~ets_emask;
	}

	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int netc_timer_enable(struct ptp_clock_info *ptp,
			     struct ptp_clock_request *rq, int on)
{
	struct netc_timer *priv = ptp_to_netc_timer(ptp);

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		return net_timer_enable_perout(priv, rq, on);
	case PTP_CLK_REQ_PPS:
		return netc_timer_enable_pps(priv, rq, on);
	case PTP_CLK_REQ_EXTTS:
		return netc_timer_enable_extts(priv, rq, on);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct ptp_clock_info netc_timer_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "NETC Timer PTP clock",
	.max_adj	= 500000000,
	.n_alarm	= 2,
	.n_ext_ts	= 2,
	.n_per_out	= 3,
	.n_pins		= 0,
	.pps		= 1,
	.adjfine	= netc_timer_adjfine,
	.adjtime	= netc_timer_adjtime,
	.gettimex64	= netc_timer_gettimex64,
	.settime64	= netc_timer_settime64,
	.enable		= netc_timer_enable,
};

static int netc_timer_get_source_clk(struct netc_timer *priv)
{
	struct device_node *node = priv->dev->of_node;
	struct device *dev = priv->dev;
	const char *clk_name = NULL;
	u64 ns = NSEC_PER_SEC;

	of_property_read_string(node, "clock-names", &clk_name);
	if (clk_name) {
		priv->src_clk = devm_clk_get_optional(dev, clk_name);
		if (IS_ERR_OR_NULL(priv->src_clk)) {
			dev_warn(dev, "Failed to get source clock\n");
			priv->src_clk = NULL;
			goto default_to_system_clk;
		}

		priv->clk_freq = clk_get_rate(priv->src_clk);
		if (!strcmp(clk_name, "netc_clk_root")) {
			/* The system clock should be divided by 2 */
			priv->clk_freq /= 2;
			priv->clk_select = NETC_TMR_SYSTEM_CLK;
		} else if (!strcmp(clk_name, "ccm_timer1_clk")) {
			priv->clk_select = NETC_TMR_CCM_TIMER1;
		} else if (!strcmp(clk_name, "tmr_1588_clk")) {
			priv->clk_select = NETC_TMR_EXT_OSC;
		} else {
			goto default_to_system_clk;
		}

		goto cal_clk_period;
	}

default_to_system_clk:
	priv->clk_select = NETC_TMR_SYSTEM_CLK;
	priv->clk_freq = NETC_TMR_SYSCLK_RATE;

cal_clk_period:
	priv->base_period = div_u64(ns << 32, priv->clk_freq);
	priv->period_int = upper_32_bits(priv->base_period);
	priv->period_frac = lower_32_bits(priv->base_period);

	return 0;
}

int netc_timer_get_phc_index(struct pci_dev *timer_pdev)
{
	struct netc_timer *priv;

	if (!timer_pdev)
		return -ENODEV;

	priv = pci_get_drvdata(timer_pdev);
	if (!priv)
		return -EINVAL;

	return priv->phc_index;
}
EXPORT_SYMBOL_GPL(netc_timer_get_phc_index);

static int netc_timer_get_global_ip_rev(struct netc_timer *priv)
{
	u32 val;

	val = netc_timer_rd(priv, NETC_GLOBAL_OFFSET + NETC_GLOBAL_IPBRR0);

	return val & IPBRR0_IP_REV;
}

static int netc_timer_init(struct netc_timer *priv)
{
	u32 tmr_emask = TMR_TEVENT_ALM1EN | TMR_TEVENT_ALM2EN;
	u32 tmr_ctrl, fiper_ctrl;
	struct timespec64 now;
	u64 ns;
	int i;

	priv->caps = netc_timer_ptp_caps;
	priv->oclk_prsc = NETC_TMR_DEFAULT_PRSC;
	priv->alarm_num = NETC_TMR_ALARM_NUM;

	if (netc_timer_get_global_ip_rev(priv) == NETC_REV_4_1)
		priv->alarm_num = 1;

	spin_lock_init(&priv->lock);

	guard(spinlock_irqsave)(&priv->lock);
	/* Software must enable timer first and the clock selected must be
	 * active, otherwise, the registers which are in the timer clock
	 * domain are not accesdible.
	 */
	tmr_ctrl = (priv->clk_select & TMR_CTRL_CK_SEL) | TMR_CTRL_TE;
	netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);

	/* Output FIPER pulse clock (TMR_GCLK) is generated by dividing the
	 * input clock of Timer by priv->oclk_prsc. For example, if input
	 * clock of Timer is 200MHz, and priv->oclk_prsc is 2, then TMR_GCLK
	 * is 100MHz.
	 */
	netc_timer_wr(priv, NETC_TMR_PRSC, priv->oclk_prsc);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	for (i = 0; i < NETC_TMR_FIPER_NUM; i++) {
		fiper_ctrl |= FIPER_CTRL_DIS(i);
		fiper_ctrl &= ~FIPER_CTRL_PG(i);
	}
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);
	netc_timer_wr(priv, NETC_TMR_ECTRL, NETC_TMR_DEFAULT_ETTF_THR);

	ktime_get_real_ts64(&now);
	ns = timespec64_to_ns(&now);
	netc_timer_cnt_write(priv, ns);

	/* Allow atomic writes to TCLK_PERIOD and TMR_ADD,  An update
	 * to TCLK_PERIOD doesn't take effect until TMR_ADD is written.
	 */
	tmr_ctrl |= ((priv->period_int << 16) & TMR_CTRL_TCLK_PERIOD) |
		    TMR_COMP_MODE | TMR_CTRL_FS;
	netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);
	netc_timer_wr(priv, NETC_TMR_ADD, priv->period_frac);
	netc_timer_wr(priv, NETC_TMR_TEMASK, tmr_emask);

	return 0;
}

static void netc_timer_deinit(struct netc_timer *priv)
{
	u32 fiper_ctrl;
	int i;

	guard(spinlock_irqsave)(&priv->lock);

	netc_timer_wr(priv, NETC_TMR_TEMASK, 0);
	netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 0);
	netc_timer_alarm_write(priv, NETC_TMR_DEFAULT_ALARM, 1);
	fiper_ctrl = netc_timer_rd(priv, NETC_TMR_FIPER_CTRL);
	for (i = 0; i < NETC_TMR_FIPER_NUM; i++) {
		netc_timer_wr(priv, NETC_TMR_FIPER(i),
				     NETC_TMR_DEFAULT_FIPER);
		fiper_ctrl |= FIPER_CTRL_DIS(i);
	}
	netc_timer_wr(priv, NETC_TMR_FIPER_CTRL, fiper_ctrl);
}

static int netc_timer_parse_dt(struct netc_timer *priv)
{
	struct device_node *node = priv->dev->of_node;

	if (!node || of_property_read_u8(node, "nxp,pps-channel",
					 &priv->pps_channel))
		priv->pps_channel = NETC_TMR_DEFAULT_PPS_FIPER;

	if (priv->pps_channel >= NETC_TMR_FIPER_NUM) {
		dev_err(priv->dev, "pps_channel is %u, greater than %d\n",
			priv->pps_channel, NETC_TMR_FIPER_NUM);

		return -EINVAL;
	}

	netc_timer_get_source_clk(priv);

	return 0;
}

static int netc_timer_get_fiper_loopback(struct netc_timer *priv,
					 int fiper, u64 *val)
{
	unsigned long flags;
	u32 tmr_ctrl;

	spin_lock_irqsave(&priv->lock, flags);
	tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);

	switch (fiper) {
	case 0:
		*val = tmr_ctrl & TMR_CTRL_PP1L ? 1 : 0;
		break;
	case 1:
		*val = tmr_ctrl & TMR_CTRL_PP2L ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int netc_timer_set_fiper_loopback(struct netc_timer *priv,
					 int fiper, u64 val)
{
	unsigned long flags;
	u32 tmr_ctrl;
	int err = 0;

	spin_lock_irqsave(&priv->lock, flags);

	tmr_ctrl = netc_timer_rd(priv, NETC_TMR_CTRL);
	switch (fiper) {
	case 0:
		tmr_ctrl = u32_replace_bits(tmr_ctrl, val ? 1 : 0,
					    TMR_CTRL_PP1L);
		break;
	case 1:
		tmr_ctrl = u32_replace_bits(tmr_ctrl, val ? 1 : 0,
					    TMR_CTRL_PP2L);
		break;
	default:
		err = -EINVAL;
	}

	if (!err)
		netc_timer_wr(priv, NETC_TMR_CTRL, tmr_ctrl);

	spin_unlock_irqrestore(&priv->lock, flags);

	return err;
}

static int netc_timer_get_fiper1_loopback(void *data, u64 *val)
{
	struct netc_timer *priv = data;

	return netc_timer_get_fiper_loopback(priv, 0, val);
}

static int netc_timer_set_fiper1_loopback(void *data, u64 val)
{
	struct netc_timer *priv = data;

	return netc_timer_set_fiper_loopback(priv, 0, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(netc_timer_fiper1_fops, netc_timer_get_fiper1_loopback,
			 netc_timer_set_fiper1_loopback, "%llu\n");

static int netc_timer_get_fiper2_loopback(void *data, u64 *val)
{
	struct netc_timer *priv = data;

	return netc_timer_get_fiper_loopback(priv, 1, val);
}

static int netc_timer_set_fiper2_loopback(void *data, u64 val)
{
	struct netc_timer *priv = data;

	return netc_timer_set_fiper_loopback(priv, 1, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(netc_timer_fiper2_fops, netc_timer_get_fiper2_loopback,
			 netc_timer_set_fiper2_loopback, "%llu\n");

static void netc_timer_create_debugfs(struct netc_timer *priv)
{
	char debugfs_name[24];
	struct dentry *root;

	snprintf(debugfs_name, sizeof(debugfs_name),
		 "netc_timer%d", priv->phc_index);
	root = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(root))
		return;

	priv->debugfs_root = root;
	debugfs_create_file("fiper1-loopback", 0600, root, priv,
			    &netc_timer_fiper1_fops);
	debugfs_create_file("fiper2-loopback", 0600, root, priv,
			    &netc_timer_fiper2_fops);
}

static void netc_timer_remove_debugfs(struct netc_timer *priv)
{
	debugfs_remove_recursive(priv->debugfs_root);
	priv->debugfs_root = NULL;
}

static int netc_timer_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct netc_timer *priv;
	int err, len, n;

	err = pci_enable_device_mem(pdev);
	if (err)
		return dev_err_probe(dev, err, "device enable failed\n");

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(dev, "DMA configuration failed: 0x%x\n", err);
		goto disable_dev;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(dev, "pci_request_regions failed err=%d\n", err);
		goto disable_dev;
	}

	pci_set_master(pdev);
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto release_mem_regions;
	}
	priv->dev = dev;
	priv->pci_dev = pdev;
	priv->phc_index = -1; /* initialize it as an invalid index */

	len = pci_resource_len(pdev, NETC_TMR_REGS_BAR);
	priv->base = ioremap(pci_resource_start(pdev, NETC_TMR_REGS_BAR), len);
	if (!priv->base) {
		err = -ENXIO;
		dev_err(dev, "ioremap() failed\n");
		goto free_priv;
	}

	n = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
	if (n != 1) {
		err = -EPERM;
		goto unmap_resource;
	}
	priv->irq = pci_irq_vector(pdev, 0);
	snprintf(priv->irq_name, sizeof(priv->irq_name),
		"ptp-netc %s", pci_name(pdev));
	err = request_irq(priv->irq, netc_timer_isr, 0, priv->irq_name, priv);
	if (err) {
		dev_err(dev, "request_irq() failed!\n");
		goto free_irq_vectors;
	}

	err = netc_timer_parse_dt(priv);
	if (err) {
		dev_err(dev, "Parse DT node failed!\n");
		goto free_irq;
	}

	err = clk_prepare_enable(priv->src_clk);
	if (err) {
		dev_err(dev, "Enable timer source clock failed!\n");
		goto free_irq;
	}

	err = netc_timer_init(priv);
	if (err) {
		dev_err(dev, "NETC Timer initialization failed\n");
		goto disable_clk;
	}

	priv->clock = ptp_clock_register(&priv->caps, priv->dev);
	if (IS_ERR(priv->clock)) {
		err = PTR_ERR(priv->clock);
		goto deinit_timer;
	}

	priv->phc_index = ptp_clock_index(priv->clock);

	pci_set_drvdata(pdev, priv);
	netc_timer_create_debugfs(priv);

	return 0;

deinit_timer:
	netc_timer_deinit(priv);
disable_clk:
	clk_disable_unprepare(priv->src_clk);
free_irq:
	free_irq(priv->irq, priv);
free_irq_vectors:
	pci_free_irq_vectors(pdev);
unmap_resource:
	iounmap(priv->base);
free_priv:
	kfree(priv);
release_mem_regions:
	pci_release_mem_regions(pdev);
disable_dev:
	pci_disable_device(pdev);

	return err;
}

static void netc_timer_remove(struct pci_dev *pdev)
{
	struct netc_timer *priv = pci_get_drvdata(pdev);

	netc_timer_remove_debugfs(priv);
	ptp_clock_unregister(priv->clock);
	netc_timer_deinit(priv);
	clk_disable_unprepare(priv->src_clk);

	disable_irq(priv->irq);
	irq_set_affinity_hint(priv->irq, NULL);
	free_irq(priv->irq, priv);
	pci_free_irq_vectors(pdev);

	iounmap(priv->base);
	kfree(priv);

	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id netc_timer_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_NXP2, PCI_DEVICE_ID_NXP2_NETC_TIMER) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, netc_timer_id_table);

static void ptp_netc_shutdown(struct netc_timer *priv)
{
	struct pci_dev *pdev = priv->pci_dev;

	netc_timer_deinit(priv);
	clk_disable_unprepare(priv->src_clk);
	disable_irq(priv->irq);
	irq_set_affinity_hint(priv->irq, NULL);
	free_irq(priv->irq, priv);
	pci_free_irq_vectors(pdev);

	pci_save_state(pdev);
	pci_disable_device(priv->pci_dev);

	return;
}

static int ptp_netc_powerup(struct netc_timer *priv)
{
	struct pci_dev *pdev = priv->pci_dev;
	int err, n;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}
	pci_restore_state(pdev);

	pci_set_master(pdev);

	n = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
	if (n != 1) {
		err = -EPERM;
		goto disable_dev;
	}
	priv->irq = pci_irq_vector(pdev, 0);
	err = request_irq(priv->irq, netc_timer_isr, 0, priv->irq_name, priv);
	if (err) {
		dev_err(&pdev->dev, "request_irq() failed!\n");
		goto free_irq_vectors;
	}

	err = clk_prepare_enable(priv->src_clk);
	if (err) {
		dev_err(&pdev->dev, "Enable timer source clock failed!\n");
		goto free_irq;
	}

	err = netc_timer_init(priv);
	if (err) {
		dev_err(&pdev->dev, "NETC Timer initialization failed, err=%d\n", err);
		goto disable_clk;
	}

	return 0;

disable_clk:
	clk_disable_unprepare(priv->src_clk);
free_irq:
	free_irq(priv->irq, priv);
free_irq_vectors:
	pci_free_irq_vectors(pdev);
disable_dev:
	pci_disable_device(pdev);

	return err;
}

static int ptp_netc_suspend_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_timer *priv;

	priv = pci_get_drvdata(pdev);

	if (netc_ierb_may_wakeonlan())
		return 0;
	ptp_netc_shutdown(priv);

	return 0;
}

static int ptp_netc_resume_noirq(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct netc_timer *priv;
	int err;

	priv = pci_get_drvdata(pdev);

	if (netc_ierb_may_wakeonlan())
		return 0;

	err = ptp_netc_powerup(priv);
	if (err) {
		dev_err(dev, "NETC Timer powerup failed\n");
		return err;
	}

	return err;
}

static const struct dev_pm_ops __maybe_unused ptp_netc_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(ptp_netc_suspend_noirq,
				      ptp_netc_resume_noirq)
};

static struct pci_driver netc_timer_driver = {
	.name = KBUILD_MODNAME,
	.id_table = netc_timer_id_table,
	.probe = netc_timer_probe,
	.remove = netc_timer_remove,
	.driver.pm = &ptp_netc_pm_ops,
};
module_pci_driver(netc_timer_driver);

MODULE_AUTHOR("Wei Fang <wei.fang@nxp.com>");
MODULE_DESCRIPTION("NXP NETC Timer Driver");
MODULE_LICENSE("Dual BSD/GPL");
