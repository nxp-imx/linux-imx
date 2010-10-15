/*
 * drivers/net/imx_ptp.h
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

#ifndef _PTP_H_
#define _PTP_H_
#ifndef TRUE
#define TRUE	1
#endif
#ifndef FALSE
#define FALSE	0
#endif

#define PTP_NUM_OF_PORTS	2
#define SEQ_ID_OUT_OF_BAND	0xFFFF

/* PTP message types */
enum e_ptp_message {
	e_PTP_MSG_SYNC = 0,	/*Sync message*/
	e_PTP_MSG_DELAY_REQ,	/*Dealy_req message*/
	e_PTP_MSG_FOLLOW_UP,	/*Follow_up message*/
	e_PTP_MSG_DELAY_RESP,	/*Delay_resp message*/
	e_PTP_MSG_MANAGEMENT,	/*Management message*/
	e_PTP_MSG_DUMMY_LAST
};

/* PTP time stamp delivery mode*/
enum e_ptp_tsu_delivery_mode {
	e_PTP_TSU_DELIVERY_IN_BAND,	/*in-band time-sttamp delivery mode*/
	e_PTP_TSU_DELIVERY_OUT_OF_BAND	/*out-of-band time stamp delivery mode*/
};

/*PTP RTC Alarm Polarity Options*/
enum e_ptp_rtc_alarm_polarity {
	e_PTP_RTC_ALARM_POLARITY_ACTIVE_HIGH,	/*active-high output polarity*/
	e_PTP_RTC_ALARM_POLARITY_ACTIVE_LOW	/*active-low output polarity*/
};

/*PTP RTC Trigger Polarity Options*/
enum e_ptp_rtc_trig_polarity {
	e_PTP_RTC_TRIGGER_ON_RISING_EDGE,	/*trigger on rising edge*/
	e_PTP_RTC_TRIGGER_ON_FALLING_EDGE	/*trigger on falling edge*/
};

/*PTP RTC Periodic Pulse Start Mode*/
enum e_ptp_rtc_pulse_start_mode {
	e_PTP_RTC_PULSE_START_AUTO,	/*start pulse when RTC is enabled*/
	e_PTP_RTC_PULSE_START_ON_ALARM	/*start pulse on alarm 1 event*/
};

/*PTP RTC Alarm ID*/
enum e_ptp_rtc_alarm_id {
	e_PTP_RTC_ALARM_1 = 0,	/*alarm signal 1*/
	e_PTP_RTC_ALARM_2,	/*slarm signal 2*/
	e_PTP_RTC_ALARM_DUMMY_LAST
};
#define PTP_RTC_NUM_OF_ALARMS e_PTP_RTC_ALARM_DUMMY_LAST

/*PTP RTC Periodic Pulse ID*/
enum e_ptp_rtc_pulse_id {
	e_PTP_RTC_PULSE_1 = 0,	/*periodic pulse 1*/
	e_PTP_RTC_PULSE_2,	/*periodic pulse 2*/
	e_PTP_RTC_PULSE_3,	/*periodic pulse 3*/
	e_PTP_RTC_PULSE_DUMMY_LASR
};
#define PTP_RTC_NUM_OF_PULSES e_PTP_RTC_PULSE_DUMMY_LASR

/*PTP RTC External trigger ID*/
enum e_ptp_rtc_trigger_id {
	e_PTP_RTC_TRIGGER_1 = 0,	/*External trigger 1*/
	e_PTP_RTC_TRIGGER_2,		/*External trigger 2*/
	e_PTP_RTC_TRIGGER_DUMMY_LAST
};
#define PTP_RTC_NUM_OF_TRIGGERS e_PTP_RTC_TRIGGER_DUMMY_LAST

/* PTP register definition */
#define PTP_TSPDR1		0x0
#define PTP_TSPDR2		0x4
#define PTP_TSPDR3		0x8
#define PTP_TSPDR4		0xc
#define PTP_TSPOV		0x10
#define PTP_TSMR		0x14
#define PTP_TMR_PEVENT		0x18
#define PTP_TMR_PEMASK		0x1c
#define PTP_TMR_RXTS_H		0x20
#define PTP_TMR_RXTS_L		0x30
#define PTP_TMR_TXTS_H		0x40
#define PTP_TMR_TXTS_L		0x50
#define PTP_TSPDR5		0x60
#define PTP_TSPDR6		0x64
#define PTP_TSPDR7		0x68

/* RTC register definition */
#define PTP_TMR_CTRL		0x0
#define PTP_TMR_TEVENT		0x4
#define PTP_TMR_TEMASK		0x8
#define PTP_TMR_CNT_L		0xc
#define PTP_TMR_CNT_H		0x10
#define PTP_TMR_ADD		0x14
#define PTP_TMR_ACC		0x18
#define PTP_TMR_PRSC		0x1c
#define PTP_TMR_OFF_L		0x20
#define PTP_TMR_OFF_H		0x24
#define PTP_TMR_ALARM1_L	0x28
#define PTP_TMR_ALARM1_H	0x2c
#define PTP_TMR_ALARM2_L	0x30
#define PTP_TMR_ALARM2_H	0x34
#define PTP_TMR_FIPER1		0x38
#define PTP_TMR_FIPER2		0x3c
#define PTP_TMR_FIPER3		0x40
#define PTP_TMR_ETTS1_L		0x44
#define PTP_TMR_ETTS1_H		0x48
#define PTP_TMR_ETTS2_L		0x4c
#define PTP_TMR_ETTS2_H		0x50
#define PTP_TMR_FSV_L		0x54
#define PTP_TMR_FSV_H		0x58

/* PTP parser registers*/
#define PTP_TSPDR1_ETT_MASK	0xFFFF0000
#define PTP_TSPDR1_IPT_MASK	0x0000FF00
#define PTP_TSPDR1_ETT_SHIFT	16
#define PTP_TSPDR1_IPT_SHIFT	8

#define PTP_TSPDR2_DPNGE_MASK	0xFFFF0000
#define PTP_TSPDR2_DPNEV_MASK	0x0000FFFF
#define PTP_TSPDR2_DPNGE_SHIFT	16

#define PTP_TSPDR3_SYCTL_MASK	0xFF000000
#define PTP_TSPDR3_DRCTL_MASK	0x00FF0000
#define PTP_TSPDR3_DRPCTL_MASK	0x0000FF00
#define PTP_TSPDR3_FUCTL_MASK	0x000000FF
#define PTP_TSPDR3_SYCTL_SHIFT	24
#define PTP_TSPDR3_DRCTL_SHIFT	16
#define PTP_TSPDR3_DRPCTL_SHIFT	8

#define PTP_TSPDR4_MACTL_MASK	0xFF000000
#define PTP_TSPDR4_VLAN_MASK	0x0000FFFF
#define PTP_TSPDR4_MACTL_SHIFT	24

/*PTP Parsing Offset Values*/
#define PTP_TSPOV_ETTOF_MASK	0xFF000000
#define PTP_TSPOV_IPTOF_MASK	0x00FF0000
#define PTP_TSPOV_UDOF_MASK	0x0000FF00
#define PTP_TSPOV_PTOF_MASK	0x000000FF
#define PTP_TSPOV_ETTOF_SHIFT	24
#define PTP_TSPOV_IPTOF_SHIFT	16
#define PTP_TSPOV_UDOF_SHIFT	8

/*PTP Mode register*/
#define PTP_TSMR_OPMODE1_IN_BAND	0x00080000
#define PTP_TSMR_OPMODE2_IN_BAND	0x00040000
#define PTP_TSMR_OPMODE3_IN_BAND	0x00020000
#define PTP_TSMR_OPMODE4_IN_BAND	0x00010000
#define PTP_TSMR_EN1			0x00000008
#define PTP_TSMR_EN2			0x00000004
#define PTP_TSMR_EN3			0x00000002
#define PTP_TSMR_EN4			0x00000001

/*ptp tsu event register*/
#define PTP_TS_EXR		0x80000000	/*rx, EX to receiver */
#define PTP_TS_RX_OVR1		0x40000000	/*rx,overrun */
#define PTP_TS_TX_OVR1		0x20000000	/*tx,overrun */
#define PTP_TS_RX_SYNC1		0x10000000	/*rx,Sync Frame */
#define PTP_TS_RX_DELAY_REQ1	0x08000000	/*rx,dealy_req frame */
#define PTP_TS_TX_FRAME1	0x04000000	/*tx,PTP frame */
#define PTP_TS_PDRQRE1		0x02000000	/*rx,Pdelay_Req frame */
#define PTP_TS_PDRSRE1		0x01000000	/*rx,Pdelay_Resp frame */
#define PTP_TS_EXT		0x00800000	/*tx, EX to transmitter */
#define DEFAULT_events_PTP_Mask	0xFF800000
#define PTP_TMR_PEVENT_ALL	DEFAULT_events_PTP_Mask
#define PTP_TMR_PEVENT_VALID	0x7F000000

#define PTP_TS_RX_ALL			 \
		(PTP_TS_RX_SYNC1	|\
		 PTP_TS_RX_DELAY_REQ1	|\
		 PTP_TS_PDRQRE1		|\
		 PTP_TS_PDRSRE1)

/*RTC timer control register*/
#define RTC_TMR_CTRL_ALMP1		0x80000000	/*active low output*/
#define RTC_TMR_CTRL_ALMP2		0x40000000	/*active low output*/
#define RTC_TMR_CTRL_FS			0x10000000
#define RTC_TMR_CTRL_TCLK_PERIOD_MSK	0x03FF0000
#define RTC_TMR_CTRL_ETEP2		0x00000200
#define RTC_TMR_CTRL_ETEP1		0x00000100
#define RTC_TMR_CTRL_COPH		0x00000080
#define RTC_TMR_CTRL_CIPH		0x00000040
#define RTC_TMR_CTRL_TMSR		0x00000020
#define RTC_TMR_CTRL_DBG		0x00000010
#define RTC_TMR_CTRL_BYP		0x00000008
#define RTC_TMR_CTRL_TE			0x00000004
#define RTC_TMR_CTRL_CKSEL_TX_CLK	0x00000002
#define RTC_TMR_CTRL_CKSEL_QE_CLK	0x00000001
#define RTC_TMR_CTRL_CKSEL_EXT_CLK	0x00000000
#define RTC_TMR_CTRL_TCLK_PERIOD_SHIFT	16

/*RTC event register*/
#define RTC_TEVENT_EXT_TRIGGER_2_TS	0x02000000	/*External trigger2 TS*/
#define RTC_TEVENT_EXT_TRIGGER_1_TS	0x01000000	/*External trigger1 TS*/
#define RTC_TEVENT_ALARM_2		0x00020000	/*Alarm 2*/
#define RTC_TEVENT_ALARM_1		0x00010000	/*Alarm 1*/
#define RTC_TEVENT_PERIODIC_PULSE_1	0x00000080	/*Periodic pulse 1*/
#define RTC_TEVENT_PERIODIC_PULSE_2	0x00000040	/*Periodic pulse 2*/
#define RTC_TEVENT_PERIODIC_PULSE_3	0x00000020	/*Periodic pulse 3*/

#define RTC_EVENT_ALL				 \
		(RTC_TEVENT_EXT_TRIGGER_2_TS	|\
		RTC_TEVENT_EXT_TRIGGER_1_TS	|\
		RTC_TEVENT_ALARM_2		|\
		RTC_TEVENT_ALARM_1		|\
		RTC_TEVENT_PERIODIC_PULSE_1	|\
		RTC_TEVENT_PERIODIC_PULSE_2	|\
		RTC_TEVENT_PERIODIC_PULSE_3)

#define OFFSET_RTC	0x0000
#define OFFSET_PTP1	0x0080
#define OFFSET_PTP2	0x0100

#define NUM_OF_MODULE	2

/*General definitions*/
#define NANOSEC_PER_ONE_HZ_TICK	1000000000
#define NANOSEC_IN_SEC		NANOSEC_PER_ONE_HZ_TICK
#define MIN_RTC_CLK_FREQ_HZ	1000
#define MHZ			1000000
#define PTP_RTC_FREQ		50	/*MHz*/

/*PTP driver's default values*/
#define ETH_TYPE_VALUE		0x0800	/*IP frame*/
#define VLAN_TYPE_VALUE		0x8100
#define IP_TYPE_VALUE		0x11	/*UDP frame*/
#define UDP_GENERAL_PORT	320
#define UDP_EVENT_PORT		319
#define ETH_TYPE_OFFSET		12
#define IP_TYPE_OFFSET		23
#define UDP_DEST_PORT_OFFSET	36
#define PTP_TYPE_OFFSET		74
#define PTP_SEQUENCE_OFFSET	72
#define LENGTH_OF_TS		8

#define PTP_TX				0x00800000
#define PTP_RX				0x02000000

/*RTC default values*/
#define DEFAULT_SRC_CLOCK		0	/*external clock source*/
#define DEFAULT_BYPASS_COMPENSATION	FALSE
#define DEFAULT_INVERT_INPUT_CLK_PHASE	FALSE
#define DEFAULT_INVERT_OUTPUT_CLK_PHASE	FALSE
#define DEFAULT_OUTPUT_CLOCK_DIVISOR	0x0001
#define DEFAULT_ALARM_POLARITY		e_PTP_RTC_ALARM_POLARITY_ACTIVE_HIGH
#define DEFAULT_TRIGGER_POLARITY	e_PTP_RTC_TRIGGER_ON_RISING_EDGE
#define DEFAULT_PULSE_START_MODE	e_PTP_RTC_PULSE_START_AUTO
#define DEFAULT_EVENTS_RTC_MASK		RTC_EVENT_ALL

/*PTP default message type*/
#define DEFAULT_MSG_SYNC		e_PTP_MSG_SYNC
#define DEFAULT_MSG_DELAY_REQ		e_PTP_MSG_DELAY_REQ
#define DEFAULT_MSG_FOLLOW_UP		e_PTP_MSG_FOLLOW_UP
#define DEFAULT_MSG_DELAY_RESP		e_PTP_MSG_DELAY_RESP
#define DEFAULT_MSG_MANAGEMENT		e_PTP_MSG_MANAGEMENT

#define USE_CASE_PULSE_1_PERIOD		(NANOSEC_IN_SEC)
#define USE_CASE_PULSE_2_PERIOD		(NANOSEC_IN_SEC / 2)
#define USE_CASE_PULSE_3_PERIOD		(NANOSEC_IN_SEC / 4)

#define USE_CASE_ALARM_1_TIME		(NANOSEC_IN_SEC)
#define USE_CASE_ALARM_2_TIME		(NANOSEC_IN_SEC * 2)

#define UCC_PTP_ENABLE			0x40000000

struct ptp_rtc_driver_param {
	u32 src_clock;
	u32 src_clock_freq_hz;
	u32 rtc_freq_hz;
	bool invert_input_clk_phase;
	bool invert_output_clk_phase;
	u32 events_mask;
	enum e_ptp_rtc_pulse_start_mode pulse_start_mode;
	enum e_ptp_rtc_alarm_polarity alarm_polarity[PTP_RTC_NUM_OF_ALARMS];
	enum e_ptp_rtc_trig_polarity trigger_polarity[PTP_RTC_NUM_OF_TRIGGERS];
};

struct ptp_rtc {
	void __iomem *mem_map;		/*pointer to RTC mem*/
	bool bypass_compensation;			/*is bypass?*/
	bool start_pulse_on_alarm;			/*start on alarm 1*/
	u32 clock_period_nansec;			/*clock periodic in ns*/
	u16 output_clock_divisor;			/*clock divisor*/
	struct ptp_rtc_driver_param *driver_param;	/*driver parameters*/
	u32 rtc_irq;
	struct clk *clk;
	struct {
		void *ext_trig_timestamp_queue;
	} ext_trig_ts[2];				/*external trigger ts*/
};

/*PTP driver parameters*/
struct ptp_driver_param {
	u16 eth_type_value;
	u16 vlan_type_value;
	u16 udp_general_port;
	u16 udp_event_port;
	u8 ip_type_value;
	u8 eth_type_offset;
	u8 ip_type_offset;
	u8 udp_dest_port_offset;	/*offset of UDP destination port*/
	u8 ptp_type_offset;
	u8 ptp_msg_codes[e_PTP_MSG_DUMMY_LAST];

	enum e_ptp_tsu_delivery_mode delivery_mode;
};

/*PTP control structure*/
struct ptp {
	spinlock_t lock;
	void __iomem *mem_map;

	/*TSU*/
	u32 events_mask;
	struct fec_ptp_private *fpp;
	struct clk *clk;

	/*RTC*/
	struct ptp_rtc *rtc;		/*pointer to RTC control structure*/
	u32 orig_freq_comp;		/*the initial frequency compensation*/

	/*driver parameters*/
	struct ptp_driver_param *driver_param;

	u32 tx_time_stamps;
	u32 rx_time_stamps;
	u32 tx_time_stamps_overrun;
	u32 rx_time_stamps_overrun;
	u32 alarm_counters[PTP_RTC_NUM_OF_ALARMS];
	u32 pulse_counters[PTP_RTC_NUM_OF_PULSES];
};

#endif
