/*
 * drivers/net/imx_ptp.c
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Description: IEEE 1588 driver supporting imx5 Fast Ethernet Controller.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

#include "fec.h"
#include "fec_1588.h"
#include "imx_ptp.h"

#ifdef PTP_DEBUG
#define VDBG(fmt, args...)	printk(KERN_DEBUG "[%s]  " fmt "\n", \
				__func__, ## args)
#else
#define VDBG(fmt, args...)	do {} while (0)
#endif

static DECLARE_WAIT_QUEUE_HEAD(ptp_rx_ts_wait);
static DECLARE_WAIT_QUEUE_HEAD(ptp_tx_ts_wait);
#define PTP_GET_RX_TIMEOUT	(HZ/10)
#define PTP_GET_TX_TIMEOUT      (HZ/100)

static struct fec_ptp_private *ptp_private;
static void ptp_rtc_get_current_time(struct ptp *p_ptp,
					struct ptp_time *p_time);
static struct ptp *ptp_dev;

/* The ring resource create and manage */
static int fec_ptp_init_circ(struct circ_buf *ptp_buf, int size)
{
	ptp_buf->buf = vmalloc(size * sizeof(struct fec_ptp_data_t));

	if (!ptp_buf->buf)
		return 1;
	ptp_buf->head = 0;
	ptp_buf->tail = 0;

	return 0;
}

static inline int fec_ptp_calc_index(int size, int curr_index, int offset)
{
	return (curr_index + offset) % size;
}

static int fec_ptp_is_empty(struct circ_buf *buf)
{
	return (buf->head == buf->tail);
}

static int fec_ptp_nelems(struct circ_buf *buf, int size)
{
	const int front = buf->head;
	const int end = buf->tail;
	int n_items;

	if (end > front)
		n_items = end - front;
	else if (end < front)
		n_items = size - (front - end);
	else
		n_items = 0;

	return n_items;
}

static int fec_ptp_is_full(struct circ_buf *buf, int size)
{
	if (fec_ptp_nelems(buf, size) == (size - 1))
		return 1;
	else
		return 0;
}

static int fec_ptp_insert(struct circ_buf *ptp_buf,
			  struct fec_ptp_data_t *data,
			  struct fec_ptp_private *priv,
			  int size)
{
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_full(ptp_buf, size))
		return 1;

	spin_lock(&priv->ptp_lock);
	tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + ptp_buf->tail;

	tmp->key = data->key;
	memcpy(tmp->spid, data->spid, 10);
	tmp->ts_time.sec = data->ts_time.sec;
	tmp->ts_time.nsec = data->ts_time.nsec;

	ptp_buf->tail = fec_ptp_calc_index(size, ptp_buf->tail, 1);
	spin_unlock(&priv->ptp_lock);

	return 0;
}

static int fec_ptp_find_and_remove(struct circ_buf *ptp_buf,
				   struct fec_ptp_data_t *data,
				   struct fec_ptp_private *priv,
				   int size)
{
	int i;
	int end = ptp_buf->tail;
	unsigned long flags;
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_empty(ptp_buf))
		return 1;

	i = ptp_buf->head;
	tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + i;
	while (i != end) {
		tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + i;
		if (tmp->key == data->key &&
				!memcmp(tmp->spid, data->spid, 10))
			break;
		i = fec_ptp_calc_index(size, i, 1);
	}

	spin_lock_irqsave(&priv->ptp_lock, flags);
	if (i == end) {
		ptp_buf->head = end;
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		return 1;
	}

	data->ts_time.sec = tmp->ts_time.sec;
	data->ts_time.nsec = tmp->ts_time.nsec;

	ptp_buf->head = fec_ptp_calc_index(size, i, 1);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/* ptp and rtc param configuration */
static void rtc_default_param(struct ptp_rtc *rtc)
{
	struct ptp_rtc_driver_param *drv_param = rtc->driver_param;
	int i;

	rtc->bypass_compensation = DEFAULT_BYPASS_COMPENSATION;
	rtc->output_clock_divisor = DEFAULT_OUTPUT_CLOCK_DIVISOR;

	drv_param->src_clock = DEFAULT_SRC_CLOCK;
	drv_param->src_clock_freq_hz = clk_get_rate(rtc->clk);

	drv_param->invert_input_clk_phase = DEFAULT_INVERT_INPUT_CLK_PHASE;
	drv_param->invert_output_clk_phase = DEFAULT_INVERT_OUTPUT_CLK_PHASE;
	drv_param->pulse_start_mode = DEFAULT_PULSE_START_MODE;
	drv_param->events_mask = DEFAULT_EVENTS_RTC_MASK;

	for (i = 0; i < PTP_RTC_NUM_OF_ALARMS; i++)
		drv_param->alarm_polarity[i] = DEFAULT_ALARM_POLARITY ;

	for (i = 0; i < PTP_RTC_NUM_OF_TRIGGERS; i++)
		drv_param->trigger_polarity[i] = DEFAULT_TRIGGER_POLARITY;
}

static int ptp_rtc_config(struct ptp_rtc *rtc)
{
	/*allocate memory for RTC driver parameter*/
	rtc->driver_param = kzalloc(sizeof(struct ptp_rtc_driver_param),
					GFP_KERNEL);
	if (!rtc->driver_param) {
		printk(KERN_ERR "allocate memory failed\n");
		return -ENOMEM;
	}

	/* expected RTC input clk frequency */
	rtc->driver_param->rtc_freq_hz = PTP_RTC_FREQ * MHZ;

	/*set default RTC configuration parameters*/
	rtc_default_param(rtc);

	return 0;
}

static void ptp_param_config(struct ptp *p_ptp)
{
	struct ptp_driver_param *drv_param;

	p_ptp->driver_param = kzalloc(sizeof(struct ptp_driver_param),
					GFP_KERNEL);
	if (!p_ptp->driver_param) {
		printk(KERN_ERR	"allocate memory failed for "
				"PTP driver parameters\n");
		return;
	}

	drv_param = p_ptp->driver_param;
	/*set the default configuration parameters*/
	drv_param->eth_type_value = ETH_TYPE_VALUE;
	drv_param->vlan_type_value = VLAN_TYPE_VALUE;
	drv_param->udp_general_port = UDP_GENERAL_PORT;
	drv_param->udp_event_port = UDP_EVENT_PORT;
	drv_param->ip_type_value = IP_TYPE_VALUE;
	drv_param->eth_type_offset = ETH_TYPE_OFFSET;
	drv_param->ip_type_offset = IP_TYPE_OFFSET;
	drv_param->udp_dest_port_offset = UDP_DEST_PORT_OFFSET;
	drv_param->ptp_type_offset = PTP_TYPE_OFFSET;

	drv_param->ptp_msg_codes[e_PTP_MSG_SYNC] = DEFAULT_MSG_SYNC;
	drv_param->ptp_msg_codes[e_PTP_MSG_DELAY_REQ] = DEFAULT_MSG_DELAY_REQ;
	drv_param->ptp_msg_codes[e_PTP_MSG_FOLLOW_UP] = DEFAULT_MSG_FOLLOW_UP;
	drv_param->ptp_msg_codes[e_PTP_MSG_DELAY_RESP] = DEFAULT_MSG_DELAY_RESP;
	drv_param->ptp_msg_codes[e_PTP_MSG_MANAGEMENT] = DEFAULT_MSG_MANAGEMENT;
}

/* 64 bits operation */
static u32 div64_oper(u64 dividend, u32 divisor, u32 *quotient)
{
	u32 time_h, time_l;
	u32 result;
	u64 tmp_dividend;
	int i;

	time_h = (u32)(dividend >> 32);
	time_l = (u32)dividend;
	time_h = time_h % divisor;
	for (i = 1; i <= 32; i++) {
		tmp_dividend = (((u64)time_h << 32) | (u64)time_l);
		tmp_dividend = (tmp_dividend << 1);
		time_h = (u32)(tmp_dividend >> 32);
		time_l = (u32)tmp_dividend;
		result = time_h / divisor;
		time_h = time_h % divisor;
		*quotient += (result << (32 - i));
	}

	return time_h;
}

/*64 bites add and return the result*/
static u64 add64_oper(u64 addend, u64 augend)
{
	u64 result = 0;
	u32 addendh, addendl, augendl, augendh;

	addendh = (u32)(addend >> 32);
	addendl = (u32)addend;

	augendh = (u32)(augend>>32);
	augendl = (u32)augend;

	__asm__(
	"adds %0,%2,%3\n"
	"adc %1,%4,%5"
	: "=r" (addendl), "=r" (addendh)
	: "r" (addendl), "r" (augendl), "r" (addendh), "r" (augendh)
	);

	udelay(1);
	result = (((u64)addendh << 32) | (u64)addendl);

	return result;
}

/*64 bits multiplication and return the result*/
static u64 multi64_oper(u32 multiplier, u32 multiplicand)
{
	u64 result = 0;
	u64 tmp_ret = 0;
	u32 tmp_multi = multiplicand;
	int i;

	for (i = 0; i < 32; i++) {
		if (tmp_multi & 0x1) {
			tmp_ret = ((u64)multiplier << i);
			result = add64_oper(result, tmp_ret);
		}
		tmp_multi = (tmp_multi >> 1);
	}

	VDBG("multi 64 low result is 0x%x\n", result);
	VDBG("multi 64 high result is 0x%x\n", (u32)(result>>32));

	return result;
}

/*convert the 64 bites time stamp to second and nanosecond*/
static void convert_rtc_time(u64 *rtc_time, struct ptp_time *p_time)
{
	u32 time_h;
	u32 time_sec = 0;

	time_h = div64_oper(*rtc_time, NANOSEC_IN_SEC, &time_sec);

	p_time->sec = time_sec;
	p_time->nsec = time_h;
}

/* convert rtc time to 64 bites timestamp */
static u64 convert_unsigned_time(struct ptp_time *ptime)
{
	return add64_oper(multi64_oper(ptime->sec, NANOSEC_IN_SEC),
			(u64)ptime->nsec);
}

/*RTC interrupt handler*/
static irqreturn_t ptp_rtc_interrupt(int irq, void *_ptp)
{
	struct ptp *p_ptp = (struct ptp *)_ptp;
	struct ptp_rtc *rtc = p_ptp->rtc;
	struct ptp_time time;
	register u32 events;

	/*get valid events*/
	events = readl(rtc->mem_map + PTP_TMR_TEVENT);

	/*clear event bits*/
	writel(events, rtc->mem_map + PTP_TMR_TEVENT);

	/*get the current time as quickly as possible*/
	ptp_rtc_get_current_time(p_ptp, &time);

	if (events & RTC_TEVENT_ALARM_1) {
		p_ptp->alarm_counters[0]++;
		VDBG("PTP Alarm 1 event, time = %2d:%09d[sec:nsec]\n",
			time.sec, time.nsec);
	}
	if (events & RTC_TEVENT_ALARM_2) {
		p_ptp->alarm_counters[1]++;
		VDBG("PTP Alarm 2 event, time = %2d:%09d[sec:nsec]\n",
			time.sec, time.nsec);
	}
	if (events & RTC_TEVENT_PERIODIC_PULSE_1) {
		p_ptp->pulse_counters[0]++;
		VDBG("PTP Pulse 1 event, time = %2d:%09d[sec:nsec]\n",
			time.sec, time.nsec);
	}
	if (events & RTC_TEVENT_PERIODIC_PULSE_2) {
		p_ptp->pulse_counters[1]++;
		VDBG("PTP Pulse 2 event, time = %2d:%09d[sec:nsec]\n",
			time.sec, time.nsec);
	}
	if (events & RTC_TEVENT_PERIODIC_PULSE_3) {
		p_ptp->pulse_counters[2]++;
		VDBG("PTP Pulse 3 event, time = %2d:%09d[sec:nsec]\n",
			time.sec, time.nsec);
	}

	return IRQ_HANDLED;
}

static int ptp_rtc_init(struct ptp *p_ptp)
{
	struct ptp_rtc *rtc = p_ptp->rtc;
	struct ptp_rtc_driver_param *rtc_drv_param = rtc->driver_param;
	void __iomem *rtc_mem = rtc->mem_map;
	u32 freq_compensation = 0;
	u32 tmr_ctrl = 0;
	int ret = 0;
	int i;

	rtc = p_ptp->rtc;
	rtc_drv_param = rtc->driver_param;
	rtc_mem = rtc->mem_map;

	if (!rtc->bypass_compensation)
		rtc->clock_period_nansec = NANOSEC_PER_ONE_HZ_TICK /
				rtc_drv_param->rtc_freq_hz;
	else {
		/*In bypass mode,the RTC clock equals the source clock*/
		rtc->clock_period_nansec = NANOSEC_PER_ONE_HZ_TICK /
				rtc_drv_param->src_clock_freq_hz;
		tmr_ctrl |= RTC_TMR_CTRL_BYP;
	}

	tmr_ctrl |= ((rtc->clock_period_nansec <<
			RTC_TMR_CTRL_TCLK_PERIOD_SHIFT) &
			RTC_TMR_CTRL_TCLK_PERIOD_MSK);

	if (rtc_drv_param->invert_input_clk_phase)
		tmr_ctrl |= RTC_TMR_CTRL_CIPH;
	if (rtc_drv_param->invert_output_clk_phase)
		tmr_ctrl |= RTC_TMR_CTRL_COPH;
	if (rtc_drv_param->pulse_start_mode == e_PTP_RTC_PULSE_START_ON_ALARM) {
		tmr_ctrl |= RTC_TMR_CTRL_FS;
		rtc->start_pulse_on_alarm = TRUE;
	}

	for (i = 0; i < PTP_RTC_NUM_OF_ALARMS; i++) {
		if (rtc_drv_param->alarm_polarity[i] ==
			e_PTP_RTC_ALARM_POLARITY_ACTIVE_LOW)
			tmr_ctrl |= (RTC_TMR_CTRL_ALMP1 >> i);

	}

	/*clear TMR_ALARM registers*/
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_ALARM1_L);
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_ALARM1_H);
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_ALARM2_L);
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_ALARM2_H);

	for (i = 0; i < PTP_RTC_NUM_OF_TRIGGERS; i++) {
		if (rtc_drv_param->trigger_polarity[i] ==
			e_PTP_RTC_TRIGGER_ON_FALLING_EDGE)
			tmr_ctrl |= (RTC_TMR_CTRL_ETEP1 << i);
	}

	/*clear TMR_FIPER registers*/
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_FIPER1);
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_FIPER2);
	writel(0xFFFFFFFF, rtc_mem + PTP_TMR_FIPER3);

	/*set the source clock*/
	/*use a clock from the QE bank of clocks*/
	tmr_ctrl |= RTC_TMR_CTRL_CKSEL_EXT_CLK;

	/*write register and perform software reset*/
	writel((tmr_ctrl | RTC_TMR_CTRL_TMSR), rtc_mem + PTP_TMR_CTRL);
	writel(tmr_ctrl, rtc_mem + PTP_TMR_CTRL);

	/*clear TMR_TEVEMT*/
	writel(RTC_EVENT_ALL, rtc_mem + PTP_TMR_TEVENT);

	/*initialize TMR_TEMASK*/
	writel(rtc_drv_param->events_mask, rtc_mem + PTP_TMR_TEMASK);

	/*initialize TMR_ADD with the initial frequency compensation value:
	 freq_compensation = (2^32 / frequency ratio)*/
	div64_oper(((u64)rtc_drv_param->rtc_freq_hz << 32),
			rtc_drv_param->src_clock_freq_hz, &freq_compensation);
	p_ptp->orig_freq_comp = freq_compensation;
	writel(freq_compensation, rtc_mem + PTP_TMR_ADD);

	/*initialize TMR_PRSC*/
	writel(rtc->output_clock_divisor, rtc_mem + PTP_TMR_PRSC);

	/*initialize TMR_OFF*/
	writel(0, rtc_mem + PTP_TMR_OFF_L);
	writel(0, rtc_mem + PTP_TMR_OFF_H);

	return ret;
}

static void init_ptp_parser(struct ptp *p_ptp)
{
	void __iomem *mem_map = p_ptp->mem_map;
	struct ptp_driver_param *drv_param = p_ptp->driver_param;
	u32 reg32;

	/*initialzie PTP TSPDR1*/
	reg32 = ((drv_param->eth_type_value << PTP_TSPDR1_ETT_SHIFT) &
			PTP_TSPDR1_ETT_MASK);
	reg32 |= ((drv_param->ip_type_value << PTP_TSPDR1_IPT_SHIFT) &
			PTP_TSPDR1_IPT_MASK);
	writel(reg32, mem_map + PTP_TSPDR1);

	/*initialize PTP TSPDR2*/
	reg32 = ((drv_param->udp_general_port << PTP_TSPDR2_DPNGE_SHIFT) &
			PTP_TSPDR2_DPNGE_MASK);
	reg32 |= (drv_param->udp_event_port & PTP_TSPDR2_DPNEV_MASK);
	writel(reg32, mem_map + PTP_TSPDR2);

	/*initialize PTP TSPDR3*/
	reg32 = ((drv_param->ptp_msg_codes[e_PTP_MSG_SYNC] <<
			PTP_TSPDR3_SYCTL_SHIFT) & PTP_TSPDR3_SYCTL_MASK);
	reg32 |= ((drv_param->ptp_msg_codes[e_PTP_MSG_DELAY_REQ] <<
			PTP_TSPDR3_DRCTL_SHIFT) & PTP_TSPDR3_DRCTL_MASK);
	reg32 |= ((drv_param->ptp_msg_codes[e_PTP_MSG_DELAY_RESP] <<
			PTP_TSPDR3_DRPCTL_SHIFT) & PTP_TSPDR3_DRPCTL_MASK);
	reg32 |= (drv_param->ptp_msg_codes[e_PTP_MSG_FOLLOW_UP] &
			PTP_TSPDR3_FUCTL_MASK);
	writel(reg32, mem_map + PTP_TSPDR3);

	/*initialzie PTP TSPDR4*/
	reg32 = ((drv_param->ptp_msg_codes[e_PTP_MSG_MANAGEMENT] <<
			PTP_TSPDR4_MACTL_SHIFT) & PTP_TSPDR4_MACTL_MASK);
	reg32 |= (drv_param->vlan_type_value & PTP_TSPDR4_VLAN_MASK);
	writel(reg32, mem_map + PTP_TSPDR4);

	/*initialize PTP TSPOV*/
	reg32 = ((drv_param->eth_type_offset << PTP_TSPOV_ETTOF_SHIFT) &
			PTP_TSPOV_ETTOF_MASK);
	reg32 |= ((drv_param->ip_type_offset << PTP_TSPOV_IPTOF_SHIFT) &
			PTP_TSPOV_IPTOF_MASK);
	reg32 |= ((drv_param->udp_dest_port_offset << PTP_TSPOV_UDOF_SHIFT) &
			PTP_TSPOV_UDOF_MASK);
	reg32 |= (drv_param->ptp_type_offset & PTP_TSPOV_PTOF_MASK);
	writel(reg32, mem_map + PTP_TSPOV);
}

/* compatible with MXS 1588 */
#ifdef CONFIG_IN_BAND
void fec_ptp_store_txstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	int msg_type, seq_id, control;
	struct fec_ptp_data_t tmp_tx_time, tmp;
	struct fec_ptp_private *fpp = priv;
	struct ptp *p_ptp = ptp_dev;
	int flag;
	unsigned char *sp_id;
	unsigned short portnum;
	u64 timestamp;

	/* Check for PTP Event */
	if ((bdp->cbd_sc & BD_ENET_TX_PTP) == 0)
		return;

	/* Get ts from tx ts queue */
	memset(&tmp, 0, sizeof(struct fec_ptp_data_t));
	tmp.key = SEQ_ID_OUT_OF_BAND;
	flag = fec_ptp_find_and_remove(&(priv->txstamp), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
	if (!flag) {
		tmp_tx_time.ts_time.sec = tmp.ts_time.sec;
		tmp_tx_time.ts_time.nsec = tmp.ts_time.nsec;
	} else {
		/*read timestamp from register*/
		timestamp = ((u64)readl(p_ptp->mem_map + PTP_TMR_TXTS_H)
				<< 32) |
			(readl(p_ptp->mem_map + PTP_TMR_TXTS_L));
		convert_rtc_time(&timestamp, &(tmp_tx_time.ts_time));
	}

	seq_id = *((u16 *)(skb->data + FEC_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + FEC_PTP_CTRL_OFFS));
	sp_id = skb->data + FEC_PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_tx_time.key = ntohs(seq_id);
	memcpy(tmp_tx_time.spid, sp_id, 8);
	memcpy(tmp_tx_time.spid + 8, (unsigned char *)&portnum, 2);

	switch (control) {

	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->tx_time_sync), &tmp_tx_time, priv,
				DEFAULT_PTP_TX_BUF_SZ);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->tx_time_del_req), &tmp_tx_time, priv,
				DEFAULT_PTP_TX_BUF_SZ);
		break;

	/* clear transportSpecific field*/
	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data +
				FEC_PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			fec_ptp_insert(&(priv->tx_time_pdel_req), &tmp_tx_time,
					priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			fec_ptp_insert(&(priv->tx_time_pdel_resp), &tmp_tx_time,
					priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_tx_ts_wait);
}
#else
void fec_ptp_store_txstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
}
#endif

static void ptp_store_txstamp(struct fec_ptp_private *priv,
				struct ptp *p_ptp,
				struct ptp_time *pts,
				u32 events)
{
	struct fec_ptp_data_t tmp_tx_time;
	u16 seq_id;

	seq_id = SEQ_ID_OUT_OF_BAND;

	memset(&tmp_tx_time, 0, sizeof(struct fec_ptp_data_t));
	tmp_tx_time.key = ntohs(seq_id);
	tmp_tx_time.ts_time.sec = pts->sec;
	tmp_tx_time.ts_time.nsec = pts->nsec;
	fec_ptp_insert(&(priv->txstamp), &tmp_tx_time,
			priv, DEFAULT_PTP_TX_BUF_SZ);
}

/* out-of-band rx ts store */
static void ptp_store_rxstamp(struct fec_ptp_private *priv,
			      struct ptp *p_ptp,
			      struct ptp_time *pts,
			      u32 events)
{
	int control = PTP_MSG_ALL_OTHER;
	u16 seq_id;
	struct fec_ptp_data_t tmp_rx_time;

	/* out-of-band mode can't get seq_id */
	seq_id = SEQ_ID_OUT_OF_BAND;

	memset(&tmp_rx_time, 0, sizeof(struct fec_ptp_data_t));
	tmp_rx_time.key = ntohs(seq_id);
	tmp_rx_time.ts_time.sec = pts->sec;
	tmp_rx_time.ts_time.nsec = pts->nsec;
	if (events & PTP_TS_RX_SYNC1)
		control = PTP_MSG_SYNC;
	else if (events & PTP_TS_RX_DELAY_REQ1)
		control = PTP_MSG_DEL_REQ;
	else if (events & PTP_TS_PDRQRE1)
		control = PTP_MSG_P_DEL_REQ;
	else if (events & PTP_TS_PDRSRE1)
		control = PTP_MSG_DEL_RESP;

	switch (control) {
	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->rx_time_sync), &tmp_rx_time,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->rx_time_del_req), &tmp_rx_time,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_REQ:
		fec_ptp_insert(&(priv->rx_time_pdel_req), &tmp_rx_time,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_RESP:
		fec_ptp_insert(&(priv->rx_time_pdel_resp), &tmp_rx_time,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	default:
		break;
	}

	wake_up_interruptible(&ptp_rx_ts_wait);

}

/* in-band rx ts store */
#ifdef CONFIG_IN_BAND
void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	int msg_type, seq_id, control;
	struct fec_ptp_data_t tmp_rx_time;
	struct fec_ptp_private *fpp = priv;
	u64 timestamp;
	unsigned char *sp_id;
	unsigned short portnum;

	/* Check for PTP Event */
	if ((bdp->cbd_sc & BD_ENET_RX_PTP) == 0) {
		skb_pull(skb, 8);
		return;
	}

	/* Get ts from skb data */
	timestamp = *((u64 *)(skb->data));
	convert_rtc_time(&timestamp, &(tmp_rx_time.ts_time));
	skb_pull(skb, 8);

	seq_id = *((u16 *)(skb->data + FEC_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + FEC_PTP_CTRL_OFFS));
	sp_id = skb->data + FEC_PTP_SPORT_ID_OFFS;
	portnum = ntohs(*((unsigned short *)(sp_id + 8)));

	tmp_rx_time.key = ntohs(seq_id);
	memcpy(tmp_rx_time.spid, sp_id, 8);
	memcpy(tmp_rx_time.spid + 8, (unsigned char *)&portnum, 2);

	switch (control) {

	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->rx_time_sync), &tmp_rx_time, priv,
				DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->rx_time_del_req), &tmp_rx_time, priv,
				DEFAULT_PTP_RX_BUF_SZ);
		break;

	/* clear transportSpecific field*/
	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data +
				FEC_PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			fec_ptp_insert(&(priv->rx_time_pdel_req), &tmp_rx_time,
					priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			fec_ptp_insert(&(priv->rx_time_pdel_resp), &tmp_rx_time,
					priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_rx_ts_wait);
}
#else
void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
}
#endif

/*PTP interrupt handler*/
static irqreturn_t ptp_interrupt(int irq, void *dev_id)
{
	struct ptp *p_ptp = (struct ptp *)dev_id;
	void __iomem *mem_map = p_ptp->mem_map;
	struct ptp_time ps;
	u64 timestamp;
	u32 events, orig_events;

	/*get valid events*/
	events = readl(mem_map + PTP_TMR_PEVENT);
	while (events) {
		if (events & PTP_TS_TX_FRAME1) {
			/*read timestamp from register*/
			timestamp = ((u64)readl(mem_map + PTP_TMR_TXTS_H)
					<< 32) |
				(readl(mem_map + PTP_TMR_TXTS_L));

			/*clear event ASAP,hoping to prevent overrun*/
			writel((u32)PTP_TS_TX_FRAME1,
					mem_map + PTP_TMR_PEVENT);
			/*check for overrun(which incalidates last timestamp)*/
			events = readl(mem_map + PTP_TMR_PEVENT);

			if (events & PTP_TS_TX_OVR1) {
				/*lost synchronization with TX timestamps*/
				/*clear overrun event*/
				writel(PTP_TS_TX_OVR1,
						mem_map + PTP_TMR_PEVENT);

				p_ptp->tx_time_stamps_overrun++;
			} else {
				/*insert the Tx timestamps into the queue*/
				convert_rtc_time(&timestamp, &ps);
				ptp_store_txstamp(ptp_private, p_ptp,
						&ps, orig_events);

				/*this event is never really masked,
				 *but it should be reported only
				 *if includeed in usre events mask*/
				if (p_ptp->events_mask & PTP_TS_TX_FRAME1)
					p_ptp->tx_time_stamps++;
				VDBG("tx interrupt\n");
			}
		}

		/*typically only one of these events is relevant,
		 *depending on whether the device is PTP master or slave*/
		if (events & PTP_TS_RX_ALL) {
			/*out-of-band mode:read timestamp
			 *from registers*/
			timestamp = ((u64)readl(mem_map + PTP_TMR_RXTS_H)
					<< 32) |
					(readl(mem_map + PTP_TMR_RXTS_L));

			/*clear event ASAP,hoping to prevent overrun*/
			orig_events = events;
			writel((u32)(PTP_TS_RX_ALL),
				mem_map + PTP_TMR_PEVENT);

			/*check for overrun (which invalidates
			 *last tiemstamp)*/
			events = readl(mem_map + PTP_TMR_PEVENT);

			if (events & PTP_TS_RX_OVR1) {
				/*lost synchronization with Rx timestamp*/
				/*clear overrun event. clear the
				 *timestamp event as well, because
				 *it may have arrived after it was
				 *cleared above,but still it is not
				 *synchronized with received frames*/
				writel((u32)(PTP_TS_RX_ALL |
					PTP_TS_RX_OVR1),
					mem_map + PTP_TMR_RXTS_H);
				p_ptp->rx_time_stamps_overrun++;
			} else {
				/*insert Rx timestamp into the queue*/
				convert_rtc_time(&timestamp, &ps);
				ptp_store_rxstamp(ptp_private, p_ptp,
						&ps, orig_events);

				/*the Rx TS event is never masked in
				 *out-of-ban mode,but it should be
				 *reported only if included in user's
				 *event's mask*/
				if (p_ptp->events_mask &
					(PTP_TS_RX_SYNC1 |
					PTP_TS_RX_DELAY_REQ1))
					p_ptp->rx_time_stamps++;
			}
		}

		writel(~PTP_TMR_PEVENT_VALID, mem_map + PTP_TMR_PEVENT);
		events = readl(mem_map + PTP_TMR_PEVENT);
	}

	return IRQ_HANDLED;
}

static void init_ptp_tsu(struct ptp *p_ptp)
{
	struct ptp_driver_param *drv_param = p_ptp->driver_param;
	void __iomem *mem_map;
	u32 tsmr, pemask, events_mask;

	mem_map = p_ptp->mem_map;

	/*Tx timestamp events are required in all modes*/
	events_mask = PTP_TS_TX_FRAME1 | PTP_TS_TX_OVR1;

	/*read current values of TSU registers*/
	tsmr = readl(mem_map + PTP_TSMR);
	pemask = readl(mem_map + PTP_TMR_PEMASK);

	if (drv_param->delivery_mode ==	e_PTP_TSU_DELIVERY_IN_BAND) {
		tsmr |= PTP_TSMR_OPMODE1_IN_BAND;
		events_mask &= ~(PTP_TS_TX_OVR1);
	} else
		/*rx timestamp events are required for out of band mode*/
		events_mask |= (PTP_TS_RX_SYNC1 | PTP_TS_RX_DELAY_REQ1 |
				PTP_TS_RX_OVR1);

	pemask |= events_mask;

	/*update TSU register*/
	writel(tsmr, mem_map + PTP_TSMR);
	writel(pemask, mem_map + PTP_TMR_PEMASK);
}

/* ptp module init */
static void ptp_tsu_init(struct ptp *p_ptp)
{
	void __iomem *mem_map = p_ptp->mem_map;

	/*initialization of registered PTP modules*/
	init_ptp_parser(p_ptp);

	/*reset timestamp*/
	writel(0, mem_map + PTP_TSMR);
	writel(0, mem_map + PTP_TMR_PEMASK);
	writel(PTP_TMR_PEVENT_ALL, mem_map + PTP_TMR_PEVENT);

}

/* TSU configure function */
static u32 ptp_tsu_enable(struct ptp *p_ptp)
{
	void __iomem *mem_map;
	u32 tsmr;

	/*enable the TSU*/
	mem_map = p_ptp->mem_map;

	/*set the TSU enable bit*/
	tsmr = readl(mem_map + PTP_TSMR);
	tsmr |= PTP_TSMR_EN1;

	writel(tsmr, mem_map + PTP_TSMR);

	return 0;
}

static u32 ptp_tsu_disable(struct ptp *p_ptp)
{
	void __iomem *mem_map;
	u32 tsmr;

	mem_map = p_ptp->mem_map;

	tsmr = readl(mem_map + PTP_TSMR);
	tsmr &= ~(PTP_TSMR_EN1);
	writel(tsmr, mem_map + PTP_TSMR);

	return 0;
}

static int ptp_tsu_config_events_mask(struct ptp *p_ptp,
					u32 events_mask)
{

	p_ptp->events_mask = events_mask;

	return 0;
}

/* rtc configure function */
static u32 rtc_enable(struct ptp_rtc *rtc, bool reset_clock)
{
	u32 tmr_ctrl;

	tmr_ctrl = readl(rtc->mem_map + PTP_TMR_CTRL);
	if (reset_clock) {
		writel((tmr_ctrl | RTC_TMR_CTRL_TMSR),
				rtc->mem_map + PTP_TMR_CTRL);

		/*clear TMR_OFF*/
		writel(0, rtc->mem_map + PTP_TMR_OFF_L);
		writel(0, rtc->mem_map + PTP_TMR_OFF_H);
	}

	writel((tmr_ctrl | RTC_TMR_CTRL_TE),
			rtc->mem_map + PTP_TMR_CTRL);

	return 0;
}

static u32 rtc_disable(struct ptp_rtc *rtc)
{
	u32 tmr_ctrl;

	tmr_ctrl = readl(rtc->mem_map + PTP_TMR_CTRL);
	writel((tmr_ctrl & ~RTC_TMR_CTRL_TE),
			rtc->mem_map + PTP_TMR_CTRL);

	return 0;
}

static u32 rtc_set_periodic_pulse(
	struct ptp_rtc *rtc,
	enum e_ptp_rtc_pulse_id pulse_ID,
	u32 pulse_periodic)
{
	u32 factor;

	if (rtc->start_pulse_on_alarm) {
		/*from the spec:the ratio between the prescale register value
		 *and the fiper value should be decisable by the clock period
		 *FIPER_VALUE = (prescale_value * tclk_per * N) - tclk_per*/
		 factor = (u32)((pulse_periodic + rtc->clock_period_nansec) /
			(rtc->clock_period_nansec * rtc->output_clock_divisor));

		if ((factor * rtc->clock_period_nansec *
			rtc->output_clock_divisor) <
			(pulse_periodic + rtc->clock_period_nansec))
			pulse_periodic = ((factor * rtc->clock_period_nansec *
				rtc->output_clock_divisor) -
				rtc->clock_period_nansec);
	}

	/* Decrease it to fix PPS question (frequecy error)*/
	pulse_periodic -= rtc->clock_period_nansec;

	writel((u32)pulse_periodic, rtc->mem_map + PTP_TMR_FIPER1 +
			(pulse_ID * 4));
	return 0;
}

static u32 ptp_rtc_set_periodic_pulse(
	struct ptp *p_ptp,
	enum e_ptp_rtc_pulse_id pulse_ID,
	struct ptp_time *ptime)
{
	u32 ret;
	u64 pulse_periodic;

	if (pulse_ID >= PTP_RTC_NUM_OF_PULSES)
		return -1;
	if (ptime->nsec < 0)
		return -1;

	pulse_periodic = convert_unsigned_time(ptime);
	if (pulse_periodic > 0xFFFFFFFF)
		return -1;

	ret = rtc_set_periodic_pulse(p_ptp->rtc, pulse_ID, (u32)pulse_periodic);

	return ret;
}

static u32 rtc_set_alarm(
	struct ptp_rtc *rtc,
	enum e_ptp_rtc_alarm_id alarm_ID,
	u64 alarm_time)
{
	u32 fiper;
	int i;

	if ((alarm_ID == e_PTP_RTC_ALARM_1) && rtc->start_pulse_on_alarm)
		alarm_time -= (3 * rtc->clock_period_nansec);

	/*TMR_ALARM_L must be written first*/
	writel((u32)alarm_time, rtc->mem_map + PTP_TMR_ALARM1_L +
			(alarm_ID * 4));
	writel((u32)(alarm_time >> 32),
			rtc->mem_map + PTP_TMR_ALARM1_H + (alarm_ID * 4));

	if ((alarm_ID == e_PTP_RTC_ALARM_1) && rtc->start_pulse_on_alarm) {
		/*we must write the TMR_FIPER register again(hardware
		 *constraint),From the spec:in order to keep tracking
		 *the prescale output clock each tiem before enabling
		 *the fiper,the user must reset the fiper by writing
		 *a new value to the reigster*/
		for (i = 0; i < PTP_RTC_NUM_OF_PULSES; i++) {
			fiper = readl(rtc->mem_map + PTP_TMR_FIPER1 +
					(i * 4));
			writel(fiper, rtc->mem_map + PTP_TMR_FIPER1 +
					(i * 4));
		}
	}

	return 0;
}

static u32 ptp_rtc_set_alarm(
	struct ptp *p_ptp,
	enum e_ptp_rtc_alarm_id alarm_ID,
	struct ptp_time *ptime)
{
	u32 ret;
	u64 alarm_time;

	if (alarm_ID >= PTP_RTC_NUM_OF_ALARMS)
		return -1;
	if (ptime->nsec < 0)
		return -1;

	alarm_time = convert_unsigned_time(ptime);

	ret = rtc_set_alarm(p_ptp->rtc, alarm_ID, alarm_time);

	return ret;
}

/* rtc ioctl function */
/*get the current time from RTC time counter register*/
static void ptp_rtc_get_current_time(struct ptp *p_ptp,
					struct ptp_time *p_time)
{
	u64 times;
	struct ptp_rtc *rtc = p_ptp->rtc;

	/*TMR_CNT_L must be read first to get an accurate value*/
	times = (u64)readl(rtc->mem_map + PTP_TMR_CNT_L);
	times |= ((u64)readl(rtc->mem_map + PTP_TMR_CNT_H)) << 32;

	/*convert RTC time*/
	convert_rtc_time(&times, p_time);
}

static void ptp_rtc_reset_counter(struct ptp *p_ptp, struct ptp_time *p_time)
{
	u64 times;
	struct ptp_rtc *rtc = p_ptp->rtc;

	times = convert_unsigned_time(p_time);
	writel((u32)times, rtc->mem_map + PTP_TMR_CNT_L);
	writel((u32)(times >> 32), rtc->mem_map + PTP_TMR_CNT_H);

}

static void rtc_modify_frequency_compensation(
	struct ptp_rtc *rtc,
	u32 freq_compensation)
{
	writel(freq_compensation, rtc->mem_map + PTP_TMR_ADD);
}

/* Set the BD to ptp */
int fec_ptp_do_txstamp(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct udphdr *udph;

	if (skb->len > 44) {
		/* Check if port is 319 for PTP Event, and check for UDP */
		iph = ip_hdr(skb);
		if (iph == NULL || iph->protocol != FEC_PACKET_TYPE_UDP)
			return 0;

		udph = udp_hdr(skb);
		if (udph != NULL && ntohs(udph->dest) == 319)
			return 1;
	}

	return 0;
}

static int fec_get_tx_timestamp(struct fec_ptp_private *priv,
				 struct ptp_ts_data *pts,
				 struct ptp_time *tx_time)
{
	struct fec_ptp_data_t tmp;
	int flag;

#ifdef CONFIG_IN_BAND
	u8 mode;
	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);
	mode = pts->message_type;

	switch (mode) {
	case PTP_MSG_SYNC:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_sync), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
		break;
	case PTP_MSG_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_del_req), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_pdel_req), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = fec_ptp_find_and_remove(&(priv->tx_time_pdel_resp),
				&tmp, priv, DEFAULT_PTP_TX_BUF_SZ);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		tx_time->sec = tmp.ts_time.sec;
		tx_time->nsec = tmp.ts_time.nsec;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_tx_ts_wait, 0,
					PTP_GET_TX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = fec_ptp_find_and_remove(&(priv->tx_time_sync),
				&tmp, priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->tx_time_del_req), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->tx_time_pdel_req), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = fec_ptp_find_and_remove(
				&(priv->tx_time_pdel_resp), &tmp,
				priv, DEFAULT_PTP_TX_BUF_SZ);
			break;
		}

		if (flag == 0) {
			tx_time->sec = tmp.ts_time.sec;
			tx_time->nsec = tmp.ts_time.nsec;
			return 0;
		}

		return -1;
	}

#else
	memset(tmp.spid, 0, 10);
	tmp.key = SEQ_ID_OUT_OF_BAND;

	flag = fec_ptp_find_and_remove(&(priv->txstamp), &tmp,
			priv, DEFAULT_PTP_TX_BUF_SZ);
	tx_time->sec = tmp.ts_time.sec;
	tx_time->nsec = tmp.ts_time.nsec;
	return 0;
#endif
}

static uint8_t fec_get_rx_timestamp(struct fec_ptp_private *priv,
				    struct ptp_ts_data *pts,
				    struct ptp_time *rx_time)
{
	struct fec_ptp_data_t tmp;
	int flag;
	u8 mode;

#ifdef CONFIG_IN_BAND
	tmp.key = pts->seq_id;
	memcpy(tmp.spid, pts->spid, 10);
#else
	memset(tmp.spid, 0, 10);
	tmp.key = SEQ_ID_OUT_OF_BAND;
#endif
	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_sync), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;
	case PTP_MSG_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_del_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_resp),
				&tmp, priv, DEFAULT_PTP_RX_BUF_SZ);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		rx_time->sec = tmp.ts_time.sec;
		rx_time->nsec = tmp.ts_time.nsec;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_rx_ts_wait, 0,
					PTP_GET_RX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = fec_ptp_find_and_remove(&(priv->rx_time_sync),
				&tmp, priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_del_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_req), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_resp), &tmp,
				priv, DEFAULT_PTP_RX_BUF_SZ);
			break;
		}

		if (flag == 0) {
			rx_time->sec = tmp.ts_time.sec;
			rx_time->nsec = tmp.ts_time.nsec;
			return 0;
		}

		return -1;
	}
}

/* 1588 Module start */
int fec_ptp_start(struct fec_ptp_private *priv)
{
	struct ptp *p_ptp = ptp_dev;

	/* Enable TSU clk */
	clk_enable(p_ptp->clk);

	/*initialize the TSU using the register function*/
	init_ptp_tsu(p_ptp);

	/* start counter */
	p_ptp->fpp = ptp_private;
	ptp_tsu_enable(p_ptp);

	return 0;
}

/* Cleanup routine for 1588 module.
 * When PTP is disabled this routing is called */
void fec_ptp_stop(struct fec_ptp_private *priv)
{
	struct ptp *p_ptp = ptp_dev;

	/* stop counter */
	ptp_tsu_disable(p_ptp);
	clk_disable(p_ptp->clk);

	return;
}

static int ptp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ptp_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* ptp device ioctl function */
static int ptp_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	struct ptp_rtc_time *cnt;
	struct ptp_rtc_time curr_time;
	struct ptp_time rx_time, tx_time;
	struct ptp_ts_data *p_ts;
	struct ptp_set_comp *p_comp;
	struct fec_ptp_private *priv;
	int retval = 0;

	priv = (struct fec_ptp_private *) ptp_private;
	switch (cmd) {
	case PTP_GET_RX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		retval = fec_get_rx_timestamp(priv, p_ts, &rx_time);
		if (retval == 0)
			retval = copy_to_user((void __user *)(&(p_ts->ts)),
					&rx_time, sizeof(rx_time));
		break;
	case PTP_GET_TX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		fec_get_tx_timestamp(priv, p_ts, &tx_time);
		retval = copy_to_user((void __user *)(&(p_ts->ts)),
				&tx_time, sizeof(tx_time));
		break;
	case PTP_GET_CURRENT_TIME:
		ptp_rtc_get_current_time(ptp_dev, &(curr_time.rtc_time));
		retval = copy_to_user((void __user *)arg, &curr_time,
				sizeof(curr_time));
		break;
	case PTP_SET_RTC_TIME:
		cnt = (struct ptp_rtc_time *)arg;
		ptp_rtc_reset_counter(ptp_dev, &(cnt->rtc_time));
		break;
	case PTP_FLUSH_TIMESTAMP:
		/* reset sync buffer */
		priv->rx_time_sync.head = 0;
		priv->rx_time_sync.tail = 0;
		/* reset delay_req buffer */
		priv->rx_time_del_req.head = 0;
		priv->rx_time_del_req.tail = 0;
		/* reset pdelay_req buffer */
		priv->rx_time_pdel_req.head = 0;
		priv->rx_time_pdel_req.tail = 0;
		/* reset pdelay_resp buffer */
		priv->rx_time_pdel_resp.head = 0;
		priv->rx_time_pdel_resp.tail = 0;
		/* reset sync buffer */
		priv->tx_time_sync.head = 0;
		priv->tx_time_sync.tail = 0;
		/* reset delay_req buffer */
		priv->tx_time_del_req.head = 0;
		priv->tx_time_del_req.tail = 0;
		/* reset pdelay_req buffer */
		priv->tx_time_pdel_req.head = 0;
		priv->tx_time_pdel_req.tail = 0;
		/* reset pdelay_resp buffer */
		priv->tx_time_pdel_resp.head = 0;
		priv->tx_time_pdel_resp.tail = 0;
		priv->txstamp.head = 0;
		priv->txstamp.tail = 0;
		break;
	case PTP_SET_COMPENSATION:
		p_comp = (struct ptp_set_comp *)arg;
		rtc_modify_frequency_compensation(ptp_dev->rtc,
				p_comp->freq_compensation);
		break;
	case PTP_GET_ORIG_COMP:
		((struct ptp_get_comp *)arg)->dw_origcomp =
					ptp_dev->orig_freq_comp;
		break;
	default:
		return -EINVAL;
	}
	return retval;
}

static const struct file_operations ptp_fops = {
	.owner	= THIS_MODULE,
	.llseek	= NULL,
	.read	= NULL,
	.write	= NULL,
	.ioctl	= ptp_ioctl,
	.open	= ptp_open,
	.release = ptp_release,
};

static int init_ptp_driver(struct ptp *p_ptp)
{
	struct ptp_time ptime;
	int ret;

	/* configure RTC param */
	ret = ptp_rtc_config(p_ptp->rtc);
	if (ret)
		return -1;

	/* initialize RTC register */
	ptp_rtc_init(p_ptp);

	/* initialize PTP TSU param */
	ptp_param_config(p_ptp);

	/* set TSU configuration parameters */
#ifdef CONFIG_IN_BAND
	p_ptp->driver_param->delivery_mode = e_PTP_TSU_DELIVERY_IN_BAND;
#else
	p_ptp->driver_param->delivery_mode = e_PTP_TSU_DELIVERY_OUT_OF_BAND;
#endif

	if (ptp_tsu_config_events_mask(p_ptp, DEFAULT_events_PTP_Mask))
			goto end;

	/* initialize PTP TSU register */
	ptp_tsu_init(p_ptp);

	/* set periodic pulses */
	ptime.sec = USE_CASE_PULSE_1_PERIOD / NANOSEC_IN_SEC;
	ptime.nsec = USE_CASE_PULSE_1_PERIOD % NANOSEC_IN_SEC;
	ret = ptp_rtc_set_periodic_pulse(p_ptp, e_PTP_RTC_PULSE_1, &ptime);
	if (ret)
		goto end;

	ptime.sec = USE_CASE_PULSE_2_PERIOD / NANOSEC_IN_SEC;
	ptime.nsec = USE_CASE_PULSE_2_PERIOD % NANOSEC_IN_SEC;
	ret = ptp_rtc_set_periodic_pulse(p_ptp, e_PTP_RTC_PULSE_2, &ptime);
	if (ret)
		goto end;

	ptime.sec = USE_CASE_PULSE_3_PERIOD / NANOSEC_IN_SEC;
	ptime.nsec = USE_CASE_PULSE_3_PERIOD % NANOSEC_IN_SEC;
	ret = ptp_rtc_set_periodic_pulse(p_ptp, e_PTP_RTC_PULSE_3, &ptime);
	if (ret)
		goto end;

	/* set alarm */
	ptime.sec = (USE_CASE_ALARM_1_TIME / NANOSEC_IN_SEC);
	ptime.nsec = (USE_CASE_ALARM_1_TIME % NANOSEC_IN_SEC);
	ret = ptp_rtc_set_alarm(p_ptp, e_PTP_RTC_ALARM_1, &ptime);
	if (ret)
		goto end;

	ptime.sec = (USE_CASE_ALARM_2_TIME / NANOSEC_IN_SEC);
	ptime.nsec = (USE_CASE_ALARM_2_TIME % NANOSEC_IN_SEC);
	ret = ptp_rtc_set_alarm(p_ptp, e_PTP_RTC_ALARM_2, &ptime);
	if (ret)
		goto end;

	/* enable the RTC */
	ret = rtc_enable(p_ptp->rtc, FALSE);
	if (ret)
		goto end;

	udelay(10);
	ptp_rtc_get_current_time(p_ptp, &ptime);
	if (ptime.nsec == 0) {
		printk(KERN_ERR "PTP RTC is not running\n");
		goto end;
	}

	if (register_chrdev(PTP_MAJOR, "ptp", &ptp_fops))
		printk(KERN_ERR "Unable to register PTP device as char\n");
	else
		printk(KERN_INFO "Register PTP as char device\n");

end:
	return ret;
}

static void ptp_free(void)
{
	rtc_disable(ptp_dev->rtc);
	/*unregister the PTP device*/
	unregister_chrdev(PTP_MAJOR, "ptp");
}

/*
 * Resource required for accessing 1588 Timer Registers.
 */
int fec_ptp_init(struct fec_ptp_private *priv, int id)
{
	fec_ptp_init_circ(&(priv->rx_time_sync), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_del_req), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_pdel_req), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->rx_time_pdel_resp), DEFAULT_PTP_RX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_sync), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_del_req), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_pdel_req), DEFAULT_PTP_TX_BUF_SZ);
	fec_ptp_init_circ(&(priv->tx_time_pdel_resp), DEFAULT_PTP_TX_BUF_SZ);

	fec_ptp_init_circ(&(priv->txstamp), DEFAULT_PTP_TX_BUF_SZ);

	spin_lock_init(&priv->ptp_lock);
	spin_lock_init(&priv->cnt_lock);
	ptp_private = priv;

	return 0;
}
EXPORT_SYMBOL(fec_ptp_init);

void fec_ptp_cleanup(struct fec_ptp_private *priv)
{
	if (priv->rx_time_sync.buf)
		vfree(priv->rx_time_sync.buf);
	if (priv->rx_time_del_req.buf)
		vfree(priv->rx_time_del_req.buf);
	if (priv->rx_time_pdel_req.buf)
		vfree(priv->rx_time_pdel_req.buf);
	if (priv->rx_time_pdel_resp.buf)
		vfree(priv->rx_time_pdel_resp.buf);
	if (priv->tx_time_sync.buf)
		vfree(priv->tx_time_sync.buf);
	if (priv->tx_time_del_req.buf)
		vfree(priv->tx_time_del_req.buf);
	if (priv->tx_time_pdel_req.buf)
		vfree(priv->tx_time_pdel_req.buf);
	if (priv->tx_time_pdel_resp.buf)
		vfree(priv->tx_time_pdel_resp.buf);
	if (priv->txstamp.buf)
		vfree(priv->txstamp.buf);

	ptp_free();
}
EXPORT_SYMBOL(fec_ptp_cleanup);

/* probe just register memory and irq */
static int __devinit
ptp_probe(struct platform_device *pdev)
{
	int i, irq, ret = 0;
	struct resource *r;

	/* setup board info structure */
	ptp_dev = kzalloc(sizeof(struct ptp), GFP_KERNEL);
	if (!ptp_dev) {
		ret = -ENOMEM;
		goto err1;
	}
	ptp_dev->rtc = kzalloc(sizeof(struct ptp_rtc),
				GFP_KERNEL);
	if (!ptp_dev->rtc) {
		ret = -ENOMEM;
		goto err2;
	}

	/* PTP register memory */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto err3;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r) {
		ret = -EBUSY;
		goto err3;
	}

	ptp_dev->mem_map = ioremap(r->start, resource_size(r));
	if (!ptp_dev->mem_map) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	/* RTC register memory */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!r) {
		ret = -ENXIO;
		goto err4;
	}

	r = request_mem_region(r->start, resource_size(r), "PTP_RTC");
	if (!r) {
		ret = -EBUSY;
		goto err4;
	}

	ptp_dev->rtc->mem_map = ioremap(r->start, resource_size(r));

	if (!ptp_dev->rtc->mem_map) {
		ret = -ENOMEM;
		goto failed_ioremap1;
	}

	/* This device has up to two irqs on some platforms */
	for (i = 0; i < 2; i++) {
		irq = platform_get_irq(pdev, i);
		if (i && irq < 0)
			break;
		if (i == 0)
			ret = request_irq(irq, ptp_interrupt,
					IRQF_DISABLED, pdev->name, ptp_dev);
		else
			ret = request_irq(irq, ptp_rtc_interrupt,
					IRQF_DISABLED, "ptp_rtc", ptp_dev);
		if (ret) {
			while (i >= 0) {
				irq = platform_get_irq(pdev, i);
				free_irq(irq, ptp_dev);
				i--;
			}
			goto failed_irq;
		}
	}

	ptp_dev->rtc->clk = clk_get(NULL, "ieee_rtc_clk");
	if (IS_ERR(ptp_dev->rtc->clk)) {
		ret = PTR_ERR(ptp_dev->rtc->clk);
		goto failed_clk1;
	}

	ptp_dev->clk = clk_get(&pdev->dev, "ieee_1588_clk");
	if (IS_ERR(ptp_dev->clk)) {
		ret = PTR_ERR(ptp_dev->clk);
		goto failed_clk2;
	}

	clk_enable(ptp_dev->clk);

	init_ptp_driver(ptp_dev);
	clk_disable(ptp_dev->clk);

	return 0;

failed_clk2:
	clk_put(ptp_dev->rtc->clk);
failed_clk1:
	for (i = 0; i < 2; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq > 0)
			free_irq(irq, ptp_dev);
	}
failed_irq:
	iounmap((void __iomem *)ptp_dev->rtc->mem_map);
failed_ioremap1:
err4:
	iounmap((void __iomem *)ptp_dev->mem_map);
failed_ioremap:
err3:
	kfree(ptp_dev->rtc);
err2:
	kfree(ptp_dev);
err1:
	return ret;
}

static int __devexit
ptp_drv_remove(struct platform_device *pdev)
{
	clk_disable(ptp_dev->clk);
	clk_put(ptp_dev->clk);
	clk_put(ptp_dev->rtc->clk);
	iounmap((void __iomem *)ptp_dev->rtc->mem_map);
	iounmap((void __iomem *)ptp_dev->mem_map);
	kfree(ptp_dev->rtc->driver_param);
	kfree(ptp_dev->rtc);
	kfree(ptp_dev->driver_param);
	kfree(ptp_dev);
	return 0;
}

static struct platform_driver ptp_driver = {
	.driver	= {
		.name    = "ptp",
		.owner	 = THIS_MODULE,
	},
	.probe   = ptp_probe,
	.remove  = __devexit_p(ptp_drv_remove),
};

static int __init
ptp_module_init(void)
{
	printk(KERN_INFO "iMX PTP Driver\n");

	return platform_driver_register(&ptp_driver);
}

static void __exit
ptp_cleanup(void)
{
	platform_driver_unregister(&ptp_driver);
}

module_exit(ptp_cleanup);
module_init(ptp_module_init);

MODULE_LICENSE("GPL");
