/** @file  mlanroamagent.h
  *
  * @brief This files contains mlanutl roamagent command handling.
  *
  *
  * Copyright 2014-2020 NXP
  *
  * This software file (the File) is distributed by NXP
  * under the terms of the GNU General Public License Version 2, June 1991
  * (the License).  You may use, redistribute and/or modify the File in
  * accordance with the terms and conditions of the License, a copy of which
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
  * this warranty disclaimer.
  *
  */
/************************************************************************
Change log:
     08/11/2009: initial version
************************************************************************/

#ifndef _MLANROAMAGENT_H_
#define _MLANROAMAGENT_H_

/** Bit definitions */
#ifndef BIT
#define BIT(x)  (1UL << (x))
#endif

/* Define actionsd for HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE */
/** Blacklist */
#define HostCmd_ACT_ADD_TO_BLACKLIST            0x0001
/** Flushing blacklisted entry */
#define HostCmd_ACT_REMOVE_FROM_BLACKLIST       0x0002

/** Roaming scanmode: discovery */
#define DISCOVERY_MODE          1
/** Roaming scanmode: tracking */
#define TRACKING_MODE           2

/** Threshold configuration: RSSI */
#define RSSI_THRESHOLD          1
/** Threshold configuration: SNR */
#define SNR_THRESHOLD           2

#ifdef BIG_ENDIAN_SUPPORT
/** Bit values for Qualified Neighbor RSSI Entry */
#define BIT_NEIGHFLAG_RSSI  0x80000000
/** Bit values for Qualified Neighbor AGE Entry */
#define BIT_NEIGHFLAG_AGE   0x40000000
/** Bit values for Qualified Neighbor Blacklist Entry */
#define BIT_NEIGHFLAG_BLACKLIST 0x20000000
/** Bit values for Qualified Neighbor Admission Capacity */
#define BIT_NEIGHFLAG_ADMISSION_CAP     0x10000000
/** Bit values for Qualified Neighbor Uplink RSSI */
#define BIT_NEIGHFLAG_UPLINK_RSSI    0x08000000
#else
/** Bit values for Qualified Neighbor RSSI Entry */
#define BIT_NEIGHFLAG_RSSI  0x01
/** Bit values for Qualified Neighbor AGE Entry */
#define BIT_NEIGHFLAG_AGE   0x02
/** Bit values for Qualified Neighbor Blacklist Entry */
#define BIT_NEIGHFLAG_BLACKLIST 0x04
/** Bit values for Qualified Neighbor Admission Capacity */
#define BIT_NEIGHFLAG_ADMISSION_CAP     0x08
/** Bit values for Qualified Neighbor Uplink RSSI */
#define BIT_NEIGHFLAG_UPLINK_RSSI    0x10
#endif

/** milliseconds time conversion data */
typedef struct exactTime {
	t_u16 hrs;   /**< Number of hours */
	t_u16 mins;  /**< Number of minutes */
	t_u16 secs;  /**< Number of seconds */
	t_u16 msecs; /**< Number of milliseconds left */
} ExactTime_t;

/** ROAMAGENT HostEvent bitmasks */
typedef enum {
	HOST_EVENT_NBOR_DISABLE = 6,	/* reset bit 0 */
	HOST_EVENT_NBOR_ENABLE = 1,	/* set bit 0   */
	HOST_EVENT_ROAM_DISABLE = 5,	/* reset bit 1 */
	HOST_EVENT_ROAM_ENABLE = 2,	/* set bit 1   */
	HOST_EVENT_STATE_DISABLE = 3,	/* reset bit 2 */
	HOST_EVENT_STATE_ENABLE = 4,	/* reset bit 2 */
} __ATTRIB_PACK__ HostEvent_e;

/** ROAMAGENT_CONTROL command identifiers */
typedef enum {
	ROAM_CONTROL_DISABLE = 6,	/* reset bit 0 */
	ROAM_CONTROL_ENABLE = 1,	/* set bit 0   */
	ROAM_CONTROL_RESUME = 5,	/* reset bit 1 */
	ROAM_CONTROL_SUSPEND = 2,	/* set bit 1   */
	CROSSBAND_DISABLE = 3,	/* reset bit 2 */
	CROSSBAND_ENABLE = 4	/* set bit 2   */
} __ATTRIB_PACK__ RoamControl_e;

/*
 * Definitions of roaming state and other constants
 */
/** Enum Definitations: Roaming agent state */
typedef enum {
	STATE_DISCONNECTED,
	STATE_STABLE,
	STATE_DEGRADING,
	STATE_UNACCEPTABLE,
	STATE_HARDROAM,
	STATE_LINKLOSS,
	STATE_SOFTROAM,
	STATE_SUSPEND,
	STATE_CMD_SUSPEND,
	STATE_ASYNCASSOC_SUSPEND
} RoamingAgentState;

/** statistics threshold High RSSI */
typedef struct {
    /** Header */
	MrvlIEtypesHeader_t Header;
    /** RSSI threshold (dBm) */
	t_u8 Value;
    /** reporting frequency */
	t_u8 Frequency;
} __ATTRIB_PACK__ MrvlIEtypes_BeaconHighRssiThreshold_t,
	MrvlIEtypes_BeaconLowRssiThreshold_t,
	MrvlIEtypes_BeaconHighSnrThreshold_t,
	MrvlIEtypes_BeaconLowSnrThreshold_t;

/** HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD */
typedef struct {
    /** Action */
	t_u16 action;
    /** roaming state */
	t_u8 State;
    /** active/configured user */
	t_u8 Profile;
    /** TLV buffer */
	t_u8 TlvBuffer[1];
	/* MrvlIEtypes_BeaconHighRssiThreshold_t BeaconHighRssiThreshold;
	 * MrvlIEtypes_BeaconLowRssiThreshold_t  BeaconLowRssiThreshold;
	 * MrvlIEtypes_BeaconHighSnrThreshold_t  BeaconHighSnrThreshold;
	 * MrvlIEtypes_BeaconLowSnrThreshold_t   BeaconLowSnrThreshold;
	 * MrvlIEtypes_BeaconsMissed_t           PreBeaconMissed;
	 * MrvlIEtypes_FailureCount_t            FailureCnt;
	 */
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD;

typedef struct {
    /**  */
	signed char RssiHighLevel;
    /**  */
	signed char RssiLowLevel;
    /** */
	signed char RssiNborDiff;

} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_ROAM_THRESHOLD;

#define ROAM_THRESH_MAX  4

/** HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT */
typedef struct {
    /** Action */
	t_u16 action;
    /** configured qualifying snr */
	signed char ConfQualSignalStrength;
    /** active qualifying snr */
	signed char ActiveQualSignalStrength;
    /** qualifying neighbor count */
	t_u16 QualifyingNumNeighbor;
    /** inactivity in # scans */
	t_u16 StaleCount;
    /** inactivity in time (ms) */
	t_u32 StalePeriod;
    /** blacklist duration in ms due to minor failures */
	t_u32 ShortBlacklistPeriod;
    /** blacklist duration in ms due to severe failures */
	t_u32 LongBlacklistPeriod;

	HostCmd_DS_CMD_ROAMAGENT_ROAM_THRESHOLD RoamThresh[ROAM_THRESH_MAX];

} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT;

/** HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST */
typedef struct {
    /** Action */
	t_u16 action;
    /** Reserved */
	t_u16 Reserved;
    /** TLV buffer */
	t_u8 TlvBuffer[1];
	/* MrvlIEtypes_NeighborEntry_t Neighbors[MRVL_ROAM_MAX_NEIGHBORS];
	 * MRVL_ROAM_MAX_NEIGHBORS = 5
	 */
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST;

/** neighbor entry details roaming agent */
typedef struct {
    /** Header */
	MrvlIEtypesHeader_t Header;
    /** bssid of neighbor */
	t_u8 Bssid[ETH_ALEN];
    /** Reserved */
	t_u16 Reserved;
    /** neighbor snr */
	t_s16 SignalStrength;
    /** neighbor age */
	t_u16 Age;
    /** bit map for qualified neighbor */
	t_u32 QualifiedNeighborBitmap;
    /** blacklist duration in ms */
	t_u32 BlackListDuration;
} __ATTRIB_PACK__ MrvlIEtypes_NeighborEntry_t;

/** HostCmd_DS_ROAMAGENT_ADV_METRIC_THRESHOLD */
typedef struct {
    /** Action */
	t_u16 action;
    /** Beacon RSSI Metrics,Data RSSI Metrics or PER Metrics */
	t_u16 Metrics;
    /** Percentage FER Threshold value to exceed for making a roam decision */
	t_u8 UcFerThresholdValue;
    /** Percentage PER Threshold value to exceed for making a roam decision */
	t_u8 UcPerThresholdValue;
    /** Reserved for later use */
	t_u8 Reserved[2];
    /** Time (ms) for which FER should prevail in stable state */
	t_u32 StableFERPeriod_ms;
    /** Time (ms) for which FER should prevail in degrading  state */
	t_u32 DegradingFERPeriod_ms;
    /** Time (ms) for which FER should prevail in unacceptable state */
	t_u32 UnacceptableFERPeriod_ms;
    /** Time (ms) for which FER should prevail in stable state */
	t_u32 StablePERPeriod_ms;
    /** Time (ms) for which PER should prevail in degrading  state */
	t_u32 DegradingPERPeriod_ms;
    /** Time (ms) for which PER should prevail in unacceptable state */
	t_u32 UnacceptablePERPeriod_ms;
    /** Number of TX packets to exceed in period_ms ms for the FER for Roam */
	t_u32 UiFerPktThreshold;
    /** Number of TX packets to exceed in period_ms ms for the PER for Roam */
	t_u32 UiPerPktThreshold;
    /** Time in ms for which inactivity should prevail for state transition */
	t_u32 InactivityPeriodThreshold_ms;
    /** With Data RSSI Metrics, Roam only when RX packets in period_ms ms exceeds this */
	t_u32 UiRxPktThreshold;
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD;

/** scan period for each search mode and state for roaming agent */
typedef struct {
    /** Header */
	MrvlIEtypesHeader_t Header;
    /** search mode */
	t_u16 SearchMode;
    /** roaming state */
	t_u16 State;
    /** scan period value */
	t_u32 ScanPeriod;
} __ATTRIB_PACK__ MrvlIEtypes_NeighborScanPeriod_t;

/** HostCmd_DS_CMD_ROAMAGENT_CONTROL */
typedef struct {
    /** Action */
	t_u16 action;
    /** enable control */
	t_u8 Control;
    /** host event control */
	t_u8 HostEvent;
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_CONTROL;

/** HostCmd_DS_CMD_ROAMAGENT_BACKOFF */
typedef struct {
    /** Action */
	t_u16 action;
    /** minimum scans */
	t_u16 Scans;
    /** backoff period */
	t_u32 Period;
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_BACKOFF;

/** HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD */
typedef struct {
    /** Action */
	t_u16 action;
    /** Reserved */
	t_u16 Reserved;
    /** scanPeriod TLV */
	MrvlIEtypes_NeighborScanPeriod_t scanPeriod;
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD;

/** HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD_RSP */
typedef struct {
    /** Action */
	t_u16 action;
    /** Reserved */
	t_u16 Reserved;
    /** TLV buffer */
	t_u8 TlvBuffer[1];
	/* MrvlIEtypes_NeighborScanPeriod_t scanPeriod[MRVL_ROAM_SCAN_PERIODS];
	 * MRVL_ROAM_SCAN_PERIODS  = 6
	 */
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD_RSP;

/** HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE */
typedef struct {
    /** Action */
	t_u16 action;
    /** BSSID */
	t_u8 BSSID[ETH_ALEN];
} __ATTRIB_PACK__ HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE;

#endif /* _MLANROAMAGENT_H_ */
