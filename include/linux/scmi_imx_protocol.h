/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SCMI Message Protocol driver NXP extension header
 *
 * Copyright 2024 NXP.
 */

#ifndef _LINUX_SCMI_NXP_PROTOCOL_H
#define _LINUX_SCMI_NXP_PROTOCOL_H

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/scmi_protocol.h>
#include <linux/types.h>

enum scmi_nxp_protocol {
	SCMI_PROTOCOL_IMX_LMM = 0x80,
	SCMI_PROTOCOL_IMX_BBM = 0x81,
	SCMI_PROTOCOL_IMX_CPU = 0x82,
	SCMI_PROTOCOL_IMX_MISC = 0x84,
};

struct scmi_imx_bbm_proto_ops {
	int (*rtc_time_set)(const struct scmi_protocol_handle *ph, u32 id,
			    uint64_t sec);
	int (*rtc_time_get)(const struct scmi_protocol_handle *ph, u32 id,
			    u64 *val);
	int (*rtc_alarm_set)(const struct scmi_protocol_handle *ph, u32 id,
			     bool enable, u64 sec);
	int (*button_get)(const struct scmi_protocol_handle *ph, u32 *state);
};

enum scmi_nxp_notification_events {
	SCMI_EVENT_IMX_BBM_RTC = 0x0,
	SCMI_EVENT_IMX_BBM_BUTTON = 0x1,
	SCMI_EVENT_IMX_MISC_CONTROL = 0x0,
};

struct scmi_imx_bbm_notif_report {
	bool			is_rtc;
	bool			is_button;
	ktime_t			timestamp;
	unsigned int		rtc_id;
	unsigned int		rtc_evt;
};

struct scmi_imx_misc_ctrl_notify_report {
	ktime_t			timestamp;
	unsigned int		ctrl_id;
	unsigned int		flags;
};

#define MISC_MAX_BUILDDATE	16
#define MISC_MAX_BUILDTIME	16
#define MISC_MAX_CFGNAME	16
#define MISC_MAX_SINAME		16
#define MISC_MAX_BRDNAME	16

struct scmi_imx_misc_system_info {
	u32 buildnum;
	u32 buildcommit;
	u8 date[MISC_MAX_BUILDDATE];
	u8 time[MISC_MAX_BUILDTIME];
	u32 msel;
	u8 cfgname[MISC_MAX_CFGNAME];
	/* silicon */
	u32 deviceid;
	u32 sirev;
	u32 partnum;
	u8 siname[MISC_MAX_SINAME];
	u32 brd_attributes;
	u8 brdname[MISC_MAX_BRDNAME];
};

struct scmi_imx_misc_sys_sleep_rec {
	u32 sleepentryusec;
	u32 sleepexitusec;
	u32 sleepcnt;
	u32 wakesource;
	u32 mixpwrstat;
	u32 mempwrstat;
	u32 pllpwrstat;
	u32 syssleepmode;
	u32 syssleepflags;
};

struct scmi_imx_misc_syslog {
	struct scmi_imx_misc_sys_sleep_rec syssleeprecord;
	uint32_t deverrlog;
};

struct scmi_imx_misc_proto_ops {
	int (*misc_board_info)(const struct scmi_protocol_handle *ph,
			       struct scmi_imx_misc_system_info *info);
	int (*misc_cfg_info)(const struct scmi_protocol_handle *ph,
			     struct scmi_imx_misc_system_info *info);
	int (*misc_ctrl_set)(const struct scmi_protocol_handle *ph, u32 id,
			     u32 num, u32 *val);
	int (*misc_ctrl_get)(const struct scmi_protocol_handle *ph, u32 id,
			     u32 *num, u32 *val);
	int (*misc_ctrl_req_notify)(const struct scmi_protocol_handle *ph,
				    u32 ctrl_id, u32 evt_id, u32 flags);
	int (*misc_discover_build_info)(const struct scmi_protocol_handle *ph,
					struct scmi_imx_misc_system_info *info);
	int (*misc_silicon_info)(const struct scmi_protocol_handle *ph,
				 struct scmi_imx_misc_system_info *info);
	int (*misc_syslog)(const struct scmi_protocol_handle *ph, u16 size,
			  void *array);
};

#define	LMM_ID_DISCOVER	0xFFFFFFFFU
#define	LMM_MAX_NAME	16

enum scmi_imx_lmm_state {
	LMM_STATE_LM_OFF,
	LMM_STATE_LM_ON,
	LMM_STATE_LM_SUSPEND,
	LMM_STATE_LM_POWERED,
};

struct scmi_imx_lmm_info {
	u32 lmid;
	enum scmi_imx_lmm_state state;
	u32 errstatus;
	u8 name[LMM_MAX_NAME];
};

struct scmi_imx_lmm_proto_ops {
	int (*lmm_boot)(const struct scmi_protocol_handle *ph, u32 lmid);
	int (*lmm_info)(const struct scmi_protocol_handle *ph, u32 lmid,
			struct scmi_imx_lmm_info *info);
	int (*lmm_power_on)(const struct scmi_protocol_handle *ph, u32 lmid);
	int (*lmm_reset_vector_set)(const struct scmi_protocol_handle *ph,
				    u32 lmid, u32 cpuid, u64 vector);
	int (*lmm_shutdown)(const struct scmi_protocol_handle *ph, u32 lmid,
			    u32 flags);
};

struct scmi_imx_cpu_proto_ops {
	int (*cpu_reset_vector_set)(const struct scmi_protocol_handle *ph,
				    u32 cpuid, u64 vector, bool start,
				    bool boot, bool resume);
	int (*cpu_start)(const struct scmi_protocol_handle *ph, u32 cpuid);
	int (*cpu_started)(const struct scmi_protocol_handle *ph, u32 cpuid,
			   bool *started);
	int (*cpu_stop)(const struct scmi_protocol_handle *ph, u32 cpuid);
};
#endif
