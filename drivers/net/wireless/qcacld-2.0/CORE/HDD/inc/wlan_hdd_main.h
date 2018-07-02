/*
 * Copyright (c) 2012-2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#if !defined( WLAN_HDD_MAIN_H )
#define WLAN_HDD_MAIN_H
/**===========================================================================

  \file  WLAN_HDD_MAIN_H.h

  \brief Linux HDD Adapter Type

  ==========================================================================*/

/*---------------------------------------------------------------------------
  Include files
  -------------------------------------------------------------------------*/

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <net/cfg80211.h>
#include <vos_list.h>
#include <vos_types.h>
#include "sirMacProtDef.h"
#include "csrApi.h"
#include <wlan_hdd_assoc.h>
#include <wlan_hdd_dp_utils.h>
#include <wlan_hdd_wmm.h>
#include <wlan_hdd_cfg.h>
#include <linux/spinlock.h>
#include <wlan_hdd_ftm.h>
#ifdef FEATURE_WLAN_TDLS
#include "wlan_hdd_tdls.h"
#endif
#include "wlan_hdd_cfg80211.h"
#include <adf_os_defer.h>
#ifdef WLAN_FEATURE_MBSSID
#include "sapApi.h"
#endif
#include "wlan_hdd_nan_datapath.h"
#include "tl_shim.h"

/*---------------------------------------------------------------------------
  Preprocessor definitions and constants
  -------------------------------------------------------------------------*/
/** Number of Tx Queues */
#define NUM_TX_QUEUES 4
/** HDD's internal Tx Queue Length. Needs to be a power of 2 */
#define HDD_TX_QUEUE_MAX_LEN 128
/** HDD internal Tx Queue Low Watermark. Net Device TX queue is disabled
 *  when HDD queue becomes full. This Low watermark is used to enable
 *  the Net Device queue again */
#define HDD_TX_QUEUE_LOW_WATER_MARK (HDD_TX_QUEUE_MAX_LEN*3/4)

/** Length of the TX queue for the netdev */
#define HDD_NETDEV_TX_QUEUE_LEN (3000)

/** Hdd Tx Time out value */
#ifdef LIBRA_LINUX_PC
#define HDD_TX_TIMEOUT          (8000)
#else
#define HDD_TX_TIMEOUT          msecs_to_jiffies(5000)
#endif
/** Hdd Default MTU */
#define HDD_DEFAULT_MTU         (1500)

#ifdef QCA_CONFIG_SMP
#define NUM_CPUS NR_CPUS
#else
#define NUM_CPUS 1
#endif

/**event flags registered net device*/
#define NET_DEVICE_REGISTERED  (0)
#define SME_SESSION_OPENED     (1)
#define INIT_TX_RX_SUCCESS     (2)
#define WMM_INIT_DONE          (3)
#define SOFTAP_BSS_STARTED     (4)
#define DEVICE_IFACE_OPENED    (5)
#define TDLS_INIT_DONE         (6)
#define ACS_PENDING            (7)
#define SOFTAP_INIT_DONE       (8)

/* HDD global event flags */
#define ACS_IN_PROGRESS        (0)

/** Maximum time(ms)to wait for disconnect to complete **/
#define WLAN_WAIT_TIME_DISCONNECT  5000
#define WLAN_WAIT_DISCONNECT_ALREADY_IN_PROGRESS  1000
#define WLAN_WAIT_TIME_STATS       800
#define WLAN_WAIT_TIME_POWER       800
#define WLAN_WAIT_TIME_COUNTRY     1000
#define WLAN_WAIT_TIME_LINK_STATUS 800
/* Amount of time to wait for sme close session callback.
   This value should be larger than the timeout used by WDI to wait for
   a response from WCNSS */
#define WLAN_WAIT_TIME_SESSIONOPENCLOSE  15000
#define WLAN_WAIT_TIME_ABORTSCAN  2000
#define WLAN_WAIT_TIME_EXTSCAN  1000
#define WLAN_WAIT_TIME_LL_STATS 800
#define WLAN_WAIT_TIME_POWER_STATS 800

#define WLAN_WAIT_SMPS_FORCE_MODE  500

/** Maximum time(ms) to wait for mc thread suspend **/
#define WLAN_WAIT_TIME_MCTHREAD_SUSPEND  1200

/** Maximum time(ms) to wait for target to be ready for suspend **/
#define WLAN_WAIT_TIME_READY_TO_SUSPEND  2000


/** Maximum time(ms) to wait for tdls add sta to complete **/
#define WAIT_TIME_TDLS_ADD_STA      1500

/** Maximum time(ms) to wait for tdls del sta to complete **/
#define WAIT_TIME_TDLS_DEL_STA      1500

/** Maximum time(ms) to wait for Link Establish Req to complete **/
#define WAIT_TIME_TDLS_LINK_ESTABLISH_REQ      1500

/** Maximum time(ms) to wait for tdls mgmt to complete **/
#define WAIT_TIME_TDLS_MGMT         11000

/** Maximum time(ms) to wait for tdls initiator to start direct communication **/
#define WAIT_TIME_TDLS_INITIATOR    600

/* Scan Req Timeout */
#define WLAN_WAIT_TIME_SCAN_REQ 100

#define WLAN_WAIT_TIME_BPF     1000

#define WLAN_WAIT_TIME_CHAIN_RSSI  1000

#define WLAN_WAIT_TIME_SET_RND 100

#define MAX_NUMBER_OF_ADAPTERS 4

#define MAX_CFG_STRING_LEN  255

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
/** Mac Address string **/
#define MAC_ADDRESS_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDRESS_STR_LEN 18 /* Including null terminator */
#define MAX_GENIE_LEN 512

#define WLAN_CHIP_VERSION   "WCNSS"

#define hddLog(level, args...) VOS_TRACE( VOS_MODULE_ID_HDD, level, ## args)
#define ENTER() VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO, "Enter:%s", __func__)
#define EXIT()  VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO, "Exit:%s", __func__)

#define WLAN_HDD_GET_PRIV_PTR(__dev__) (hdd_adapter_t*)(netdev_priv((__dev__)))

#define MAX_EXIT_ATTEMPTS_DURING_LOGP 20

#define MAX_NO_OF_2_4_CHANNELS 14

#define WLAN_HDD_PUBLIC_ACTION_FRAME 4
#define WLAN_HDD_PUBLIC_ACTION_FRAME_OFFSET 24
#define WLAN_HDD_PUBLIC_ACTION_FRAME_BODY_OFFSET 24
#define WLAN_HDD_PUBLIC_ACTION_FRAME_TYPE_OFFSET 30
#define WLAN_HDD_PUBLIC_ACTION_FRAME_CATEGORY_OFFSET 0
#define WLAN_HDD_PUBLIC_ACTION_FRAME_ACTION_OFFSET 1
#define WLAN_HDD_PUBLIC_ACTION_FRAME_OUI_OFFSET 2
#define WLAN_HDD_PUBLIC_ACTION_FRAME_OUI_TYPE_OFFSET 5
#define WLAN_HDD_VENDOR_SPECIFIC_ACTION 0x09
#define WLAN_HDD_WFA_OUI   0x506F9A
#define WLAN_HDD_WFA_P2P_OUI_TYPE 0x09
#define WLAN_HDD_P2P_SOCIAL_CHANNELS 3
#define WLAN_HDD_P2P_SINGLE_CHANNEL_SCAN 1
#define WLAN_HDD_PUBLIC_ACTION_FRAME_SUB_TYPE_OFFSET 6

#define WLAN_HDD_IS_SOCIAL_CHANNEL(center_freq) \
(((center_freq) == 2412) || ((center_freq) == 2437) || ((center_freq) == 2462))

#define WLAN_HDD_CHANNEL_IN_UNII_1_BAND(center_freq) \
(((center_freq) == 5180 ) || ((center_freq) == 5200) \
|| ((center_freq) == 5220) || ((center_freq) == 5240))

#ifdef WLAN_FEATURE_11W
#define WLAN_HDD_SA_QUERY_ACTION_FRAME 8
#endif

#define WLAN_HDD_PUBLIC_ACTION_TDLS_DISC_RESP 14
#define WLAN_HDD_TDLS_ACTION_FRAME 12

#define WLAN_HDD_QOS_ACTION_FRAME 1
#define WLAN_HDD_QOS_MAP_CONFIGURE 4
#define HDD_SAP_WAKE_LOCK_DURATION 10000 //in msecs

#if defined(CONFIG_HL_SUPPORT)
#define HDD_MOD_EXIT_SSR_MAX_RETRIES 300
#else
#define HDD_MOD_EXIT_SSR_MAX_RETRIES 75
#endif

/* Maximum number of interfaces allowed(STA, P2P Device, P2P Interfaces) */
#ifndef WLAN_OPEN_P2P_INTERFACE
#ifdef WLAN_4SAP_CONCURRENCY
#define WLAN_MAX_INTERFACES 4
#else
#define WLAN_MAX_INTERFACES 3
#endif
#else
#define WLAN_MAX_INTERFACES 4
#endif

#ifdef WLAN_FEATURE_GTK_OFFLOAD
#define GTK_OFFLOAD_ENABLE  0
#define GTK_OFFLOAD_DISABLE 1
#endif

#define MAX_USER_COMMAND_SIZE 4096

#define HDD_MAC_ADDR_LEN    6
#define HDD_SESSION_ID_ANY  50 //This should be same as CSR_SESSION_ID_ANY
/* This should be same as CSR_ROAM_SESSION_MAX */
#define HDD_SESSION_MAX  5


#define HDD_MIN_TX_POWER (-100) // minimum tx power
#define HDD_MAX_TX_POWER (+100)  // maximum tx power

/* FW expects burst duration in 1020*ms */
#define SIFS_BURST_DUR_MULTIPLIER 1020
#define SIFS_BURST_DUR_MAX        12240

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0))
#ifdef CONFIG_CNSS
#define cfg80211_vendor_cmd_reply(skb) vos_vendor_cmd_reply(skb)
#endif
#endif

/*
 * NET_NAME_UNKNOWN is only introduced after Kernel 3.17, to have a macro
 * here if the Kernel version is less than 3.17 to avoid the interleave
 * conditional compilation.
 */
#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0))
#define NET_NAME_UNKNOWN	0
#endif

typedef v_U8_t tWlanHddMacAddr[HDD_MAC_ADDR_LEN];

#define HDD_BW_GET_DIFF(_x, _y) (unsigned long)((ULONG_MAX - (_y)) + (_x) + 1)

#define MAX_PROBE_REQ_OUIS 16

#define SCAN_REJECT_THRESHOLD_TIME 300000 /* Time is in msec, equal to 5 mins */
#define SCAN_REJECT_THRESHOLD 15


/*
 * @eHDD_SCAN_REJECT_DEFAULT: default value
 * @eHDD_CONNECTION_IN_PROGRESS: connection is in progress
 * @eHDD_REASSOC_IN_PROGRESS: reassociation is in progress
 * @eHDD_EAPOL_IN_PROGRESS: STA/P2P-CLI is in middle of EAPOL/WPS exchange
 * @eHDD_SAP_EAPOL_IN_PROGRESS: SAP/P2P-GO is in middle of EAPOL/WPS exchange
 */
typedef enum
{
   eHDD_SCAN_REJECT_DEFAULT = 0,
   eHDD_CONNECTION_IN_PROGRESS,
   eHDD_REASSOC_IN_PROGRESS,
   eHDD_EAPOL_IN_PROGRESS,
   eHDD_SAP_EAPOL_IN_PROGRESS,
} scan_reject_states;

/*
 * Maximum no.of random mac addresses supported by firmware
 * for transmitting management action frames
 */
#define MAX_RANDOM_MAC_ADDRS 16

/*
 * Generic asynchronous request/response support
 *
 * Many of the APIs supported by HDD require a call to SME to
 * perform an action or to retrieve some data.  In most cases SME
 * performs the operation asynchronously, and will execute a provided
 * callback function when the request has completed.  In order to
 * synchronize this the HDD API allocates a context which is then
 * passed to SME, and which is then, in turn, passed back to the
 * callback function when the operation completes.  The callback
 * function then sets a completion variable inside the context which
 * the HDD API is waiting on.  In an ideal world the HDD API would
 * wait forever (or at least for a long time) for the response to be
 * received and for the completion variable to be set.  However in
 * most cases these HDD APIs are being invoked in the context of a
 * user space thread which has invoked either a cfg80211 API or a
 * wireless extensions ioctl and which has taken the kernel rtnl_lock.
 * Since this lock is used to synchronize many of the kernel tasks, we
 * do not want to hold it for a long time.  In addition we do not want
 * to block user space threads (such as the wpa supplicant's main
 * thread) for an extended time.  Therefore we only block for a short
 * time waiting for the response before we timeout.  This means that
 * it is possible for the HDD API to timeout, and for the callback to
 * be invoked afterwards.  In order for the callback function to
 * determine if the HDD API is still waiting, a magic value is also
 * stored in the shared context.  Only if the context has a valid
 * magic will the callback routine do any work.  In order to further
 * synchronize these activities a spinlock is used so that if any HDD
 * API timeout coincides with its callback, the operations of the two
 * threads will be serialized.
 */

struct statsContext
{
   struct completion completion;
   hdd_adapter_t *pAdapter;
   unsigned int magic;
   union iwreq_data *wrqu;
   char *extra;
};

struct linkspeedContext
{
   struct completion completion;
   hdd_adapter_t *pAdapter;
   unsigned int magic;
};

/**
 * struct random_mac_context - Context used with hdd_random_mac_callback
 * @random_mac_completion: Event on which hdd_set_random_mac will wait
 * @adapter: Pointer to adapter
 * @magic: For valid context this is set to ACTION_FRAME_RANDOM_CONTEXT_MAGIC
 * @set_random_addr: Status of random filter set
 */
struct random_mac_context {
	struct completion random_mac_completion;
	hdd_adapter_t *adapter;
	unsigned int magic;
	bool set_random_addr;
};

extern spinlock_t hdd_context_lock;

#define STATS_CONTEXT_MAGIC 0x53544154   //STAT
#define PEER_INFO_CONTEXT_MAGIC  0x52535349   /* PEER_INFO */
#define POWER_CONTEXT_MAGIC 0x504F5752   //POWR
#define SNR_CONTEXT_MAGIC   0x534E5200   //SNR
#define LINK_CONTEXT_MAGIC  0x4C494E4B   //LINKSPEED
#define LINK_STATUS_MAGIC   0x4C4B5354   //LINKSTATUS(LNST)
#define TEMP_CONTEXT_MAGIC 0x74656d70   // TEMP (temperature)
#define FW_STATUS_MAGIC 0x46575354 /* FWSTATUS(FWST) */
#define POWER_STATS_MAGIC 0x14111990
#define BPF_CONTEXT_MAGIC 0x4575354    /* BPF */
#define ACTION_FRAME_RANDOM_CONTEXT_MAGIC 0x87878787
#define ISOLATION_CONTEXT_MAGIC 0x48575354 //Antenna Isolation

#ifdef QCA_LL_TX_FLOW_CT
/* MAX OS Q block time value in msec
 * Prevent from permanent stall, resume OS Q if timer expired */
#define WLAN_HDD_TX_FLOW_CONTROL_OS_Q_BLOCK_TIME 1000
#define WLAN_SAP_HDD_TX_FLOW_CONTROL_OS_Q_BLOCK_TIME 100
#define WLAN_HDD_TX_FLOW_CONTROL_MAX_24BAND_CH   14
#endif /* QCA_LL_TX_FLOW_CT */

#define NUM_TX_RX_HISTOGRAM 1024
#define NUM_TX_RX_HISTOGRAM_MASK (NUM_TX_RX_HISTOGRAM - 1)

/**
 * struct hdd_tx_rx_histogram - structure to keep track of tx and rx packets
                                received over 100ms intervals
 * @interval_rx        # of rx packets received in the last 100ms interval
 * @interval_tx        # of tx packets received in the last 100ms interval
 * @total_rx           # of total rx packets received on interface
 * @total_rx           # of total tx packets received on interface
 * @next_vote_level    cnss_bus_width_type voting level (high or low) determined
                       on the basis of total tx and rx packets received in the
                       last 100ms interval
 * @next_rx_level      cnss_bus_width_type voting level (high or low) determined
                       on the basis of rx packets received in the last 100ms
                       interval
 * @next_tx_level      cnss_bus_width_type voting level (high or low) determined
                       on the basis of tx packets received in the last 100ms
                       interval

 * The structure keeps track of throughput requirements of wlan driver in 100ms
 * intervals for later analysis.
 */
struct hdd_tx_rx_histogram
{
   uint64_t interval_rx;
   uint64_t interval_tx;
   uint64_t total_rx;
   uint64_t total_tx;
   uint32_t next_vote_level;
   uint32_t next_rx_level;
   uint32_t next_tx_level;
};

typedef struct hdd_tx_rx_stats_s
{
   // start_xmit stats
   __u32    txXmitCalled;
   __u32    txXmitDropped;
   __u32    txXmitClassifiedAC[NUM_TX_QUEUES];
   __u32    txXmitDroppedAC[NUM_TX_QUEUES];
   // complete_cbk_stats
   __u32    txCompleted;
   // rx stats
   __u32    rxPackets[NUM_CPUS];
   __u32    rxDropped[NUM_CPUS];
   __u32    rxDelivered[NUM_CPUS];
   __u32    rxRefused[NUM_CPUS];

   bool     is_txflow_paused;
   __u32    txflow_pause_cnt;
   __u32    txflow_unpause_cnt;
   __u32    txflow_timer_cnt;
} hdd_tx_rx_stats_t;

#ifdef WLAN_FEATURE_11W
typedef struct hdd_pmf_stats_s
{
   uint8    numUnprotDeauthRx;
   uint8    numUnprotDisassocRx;
} hdd_pmf_stats_t;
#endif

typedef struct hdd_stats_s
{
   tCsrSummaryStatsInfo       summary_stat;
   tCsrGlobalClassAStatsInfo  ClassA_stat;
   tCsrGlobalClassBStatsInfo  ClassB_stat;
   tCsrGlobalClassCStatsInfo  ClassC_stat;
   tCsrGlobalClassDStatsInfo  ClassD_stat;
   tCsrPerStaStatsInfo        perStaStats;
   struct csr_per_chain_rssi_stats_info  per_chain_rssi_stats;
   hdd_tx_rx_stats_t          hddTxRxStats;
#ifdef WLAN_FEATURE_11W
   hdd_pmf_stats_t            hddPmfStats;
#endif
} hdd_stats_t;

typedef enum
{
   HDD_ROAM_STATE_NONE,

   // Issuing a disconnect due to transition into low power states.
   HDD_ROAM_STATE_DISCONNECTING_POWER,

   // move to this state when HDD sets a key with SME/CSR.  Note this is
   // an important state to get right because we will get calls into our SME
   // callback routine for SetKey activity that we did not initiate!
   HDD_ROAM_STATE_SETTING_KEY,
} HDD_ROAM_STATE;

typedef enum
{
   eHDD_SUSPEND_NONE = 0,
   eHDD_SUSPEND_DEEP_SLEEP,
   eHDD_SUSPEND_STANDBY,
} hdd_ps_state_t;

typedef struct roaming_info_s
{
   HDD_ROAM_STATE roamingState;
   vos_event_t roamingEvent;

   tWlanHddMacAddr bssid;
   tWlanHddMacAddr peerMac;
   tANI_U32 roamId;
   eRoamCmdStatus roamStatus;
   v_BOOL_t deferKeyComplete;

} roaming_info_t;

#ifdef FEATURE_WLAN_WAPI
/* Define WAPI macros for Length, BKID count etc*/
#define MAX_WPI_KEY_LENGTH    16
#define MAX_NUM_PN            16
#define MAC_ADDR_LEN           6
#define MAX_ADDR_INDEX        12
#define MAX_NUM_AKM_SUITES    16
#define MAX_NUM_UNI_SUITES    16
#define MAX_NUM_BKIDS         16

/** WAPI AUTH mode definition */
enum _WAPIAuthMode
{
   WAPI_AUTH_MODE_OPEN = 0,
   WAPI_AUTH_MODE_PSK = 1,
   WAPI_AUTH_MODE_CERT
} __packed;
typedef enum _WAPIAuthMode WAPIAuthMode;

/** WAPI Work mode structure definition */
#define   WZC_ORIGINAL      0
#define   WAPI_EXTENTION    1

struct _WAPI_FUNCTION_MODE
{
   unsigned char wapiMode;
}__packed;

typedef struct _WAPI_FUNCTION_MODE WAPI_FUNCTION_MODE;

typedef struct _WAPI_BKID
{
   v_U8_t   bkid[16];
}WAPI_BKID, *pWAPI_BKID;

/** WAPI Association information structure definition */
struct _WAPI_AssocInfo
{
   v_U8_t      elementID;
   v_U8_t      length;
   v_U16_t     version;
   v_U16_t     akmSuiteCount;
   v_U32_t     akmSuite[MAX_NUM_AKM_SUITES];
   v_U16_t     unicastSuiteCount;
   v_U32_t     unicastSuite[MAX_NUM_UNI_SUITES];
   v_U32_t     multicastSuite;
   v_U16_t     wapiCability;
   v_U16_t     bkidCount;
   WAPI_BKID   bkidList[MAX_NUM_BKIDS];
} __packed;

typedef struct _WAPI_AssocInfo WAPI_AssocInfo;
typedef struct _WAPI_AssocInfo *pWAPI_IEAssocInfo;

/** WAPI KEY Type definition */
enum _WAPIKeyType
{
   PAIRWISE_KEY, //0
   GROUP_KEY     //1
}__packed;
typedef enum _WAPIKeyType WAPIKeyType;

/** WAPI KEY Direction definition */
enum _KEY_DIRECTION
{
   None,
   Rx,
   Tx,
   Rx_Tx
}__packed;

typedef enum _KEY_DIRECTION WAPI_KEY_DIRECTION;

/* WAPI KEY structure definition */
struct WLAN_WAPI_KEY
{
   WAPIKeyType     keyType;
   WAPI_KEY_DIRECTION   keyDirection;  /*reserved for future use*/
   v_U8_t          keyId;
   v_U8_t          addrIndex[MAX_ADDR_INDEX]; /*reserved for future use*/
   int             wpiekLen;
   v_U8_t          wpiek[MAX_WPI_KEY_LENGTH];
   int             wpickLen;
   v_U8_t          wpick[MAX_WPI_KEY_LENGTH];
   v_U8_t          pn[MAX_NUM_PN];        /*reserved for future use*/
}__packed;

typedef struct WLAN_WAPI_KEY WLAN_WAPI_KEY;
typedef struct WLAN_WAPI_KEY *pWLAN_WAPI_KEY;

#define WPA_GET_LE16(a) ((u16) (((a)[1] << 8) | (a)[0]))
#define WPA_GET_BE24(a) ((u32) ( (a[0] << 16) | (a[1] << 8) | a[2]))
#define WLAN_EID_WAPI 68
#define WAPI_PSK_AKM_SUITE  0x02721400
#define WAPI_CERT_AKM_SUITE 0x01721400

/* WAPI BKID List structure definition */
struct _WLAN_BKID_LIST
{
   v_U32_t          length;
   v_U32_t          BKIDCount;
   WAPI_BKID        BKID[1];
}__packed;

typedef struct _WLAN_BKID_LIST WLAN_BKID_LIST;
typedef struct _WLAN_BKID_LIST *pWLAN_BKID_LIST;


/* WAPI Information structure definition */
struct hdd_wapi_info_s
{
   v_U32_t     nWapiMode;
   v_BOOL_t    fIsWapiSta;
   v_MACADDR_t cachedMacAddr;
   v_UCHAR_t   wapiAuthMode;
}__packed;
typedef struct hdd_wapi_info_s hdd_wapi_info_t;
#endif /* FEATURE_WLAN_WAPI */

typedef struct beacon_data_s {
    u8 *head;
    u8 *tail;
    u8 *proberesp_ies;
    u8 *assocresp_ies;
    int head_len;
    int tail_len;
    int proberesp_ies_len;
    int assocresp_ies_len;
    int dtim_period;
} beacon_data_t;

typedef enum device_mode
{  /* MAINTAIN 1 - 1 CORRESPONDENCE WITH tVOS_CON_MODE*/
   WLAN_HDD_INFRA_STATION,
   WLAN_HDD_SOFTAP,
   WLAN_HDD_P2P_CLIENT,
   WLAN_HDD_P2P_GO,
   WLAN_HDD_MONITOR,
   WLAN_HDD_FTM,
   WLAN_HDD_IBSS,
   WLAN_HDD_P2P_DEVICE,
   WLAN_HDD_OCB,
   WLAN_HDD_NDI
} device_mode_t;

#define WLAN_HDD_VDEV_STA_MAX 2

typedef enum rem_on_channel_request_type
{
   REMAIN_ON_CHANNEL_REQUEST,
   OFF_CHANNEL_ACTION_TX,
}rem_on_channel_request_type_t;

/* Thermal mitigation Level Enum Type */
typedef enum
{
   WLAN_HDD_TM_LEVEL_0,
   WLAN_HDD_TM_LEVEL_1,
   WLAN_HDD_TM_LEVEL_2,
   WLAN_HDD_TM_LEVEL_3,
   WLAN_HDD_TM_LEVEL_4,
   WLAN_HDD_TM_LEVEL_MAX
} WLAN_TmLevelEnumType;

/* Driver Action based on thermal mitigation level structure */
typedef struct
{
   v_BOOL_t  ampduEnable;
   v_BOOL_t  enterImps;
   v_U32_t   txSleepDuration;
   v_U32_t   txOperationDuration;
   v_U32_t   txBlockFrameCountThreshold;
} hdd_tmLevelAction_t;

/* Thermal Mitigation control context structure */
typedef struct
{
   WLAN_TmLevelEnumType currentTmLevel;
   hdd_tmLevelAction_t  tmAction;
   vos_timer_t          txSleepTimer;
   struct mutex         tmOperationLock;
   vos_event_t          setTmDoneEvent;
   v_U32_t              txFrameCount;
   v_TIME_t             lastblockTs;
   v_TIME_t             lastOpenTs;
   struct netdev_queue *blockedQueue;
   v_BOOL_t             qBlocked;
} hdd_thermal_mitigation_info_t;

typedef struct action_pkt_buffer
{
   tANI_U8* frame_ptr;
   tANI_U32 frame_length;
   tANI_U16 freq;
}action_pkt_buffer_t;

typedef struct hdd_remain_on_chan_ctx
{
  struct net_device *dev;
  struct ieee80211_channel chan;
  enum nl80211_channel_type chan_type;
  unsigned int duration;
  u64 cookie;
  rem_on_channel_request_type_t rem_on_chan_request;
  v_U32_t p2pRemOnChanTimeStamp;
  vos_timer_t hdd_remain_on_chan_timer;
  action_pkt_buffer_t action_pkt_buff;
  v_BOOL_t hdd_remain_on_chan_cancel_in_progress;
}hdd_remain_on_chan_ctx_t;

/* RoC Request entry */
typedef struct hdd_roc_req
{
    hdd_list_node_t node; /* MUST be first element */
    hdd_adapter_t *pAdapter;
    hdd_remain_on_chan_ctx_t *pRemainChanCtx;
}hdd_roc_req_t;

typedef enum{
    HDD_IDLE,
    HDD_PD_REQ_ACK_PENDING,
    HDD_GO_NEG_REQ_ACK_PENDING,
    HDD_INVALID_STATE,
}eP2PActionFrameState;

typedef enum {
    WLAN_HDD_GO_NEG_REQ,
    WLAN_HDD_GO_NEG_RESP,
    WLAN_HDD_GO_NEG_CNF,
    WLAN_HDD_INVITATION_REQ,
    WLAN_HDD_INVITATION_RESP,
    WLAN_HDD_DEV_DIS_REQ,
    WLAN_HDD_DEV_DIS_RESP,
    WLAN_HDD_PROV_DIS_REQ,
    WLAN_HDD_PROV_DIS_RESP,
    WLAN_HDD_ACTION_FRM_TYPE_MAX = 255,
}tActionFrmType;

typedef struct hdd_cfg80211_state_s
{
  tANI_U16 current_freq;
  u64 action_cookie;
  tANI_U8 *buf;
  size_t len;
  hdd_remain_on_chan_ctx_t* remain_on_chan_ctx;
  struct mutex remain_on_chan_ctx_lock;
  eP2PActionFrameState actionFrmState;
  /* is_go_neg_ack_received flag is set to 1 when
   * the pending ack for GO negotiation req is
   * received.
   */
  v_BOOL_t is_go_neg_ack_received;
}hdd_cfg80211_state_t;


typedef enum{
    HDD_SSR_NOT_REQUIRED,
    HDD_SSR_REQUIRED,
    HDD_SSR_DISABLED,
}e_hdd_ssr_required;

/**
 * struct hdd_mon_set_ch_info - Holds monitor mode channel switch params
 * @channel: Channel number.
 * @cb_mode: Channel bonding
 * @channel_width: Channel width 0/1/2 for 20/40/80MHz respectively.
 * @phy_mode: PHY mode
 */
struct hdd_mon_set_ch_info {
	uint8_t channel;
	uint8_t cb_mode;
	uint32_t channel_width;
	eCsrPhyMode phy_mode;
};

/**
 * struct action_frame_cookie - Action frame cookie item in cookie list
 * @cookie_node: List item
 * @cookie: Cookie value
 */
struct action_frame_cookie {
	struct list_head cookie_node;
	uint64_t cookie;
};

/**
 * struct action_frame_random_mac - Action Frame random mac addr & related attrs
 * @in_use: Checks whether random mac is in use
 * @addr: Contains random mac addr
 * @cookie_list: List of cookies tied with random mac
 */
struct action_frame_random_mac {
	bool in_use;
	uint8_t addr[VOS_MAC_ADDR_SIZE];
	struct list_head cookie_list;
};

struct hdd_station_ctx
{
  /** Handle to the Wireless Extension State */
   hdd_wext_state_t WextState;

#ifdef FEATURE_WLAN_TDLS
   tdlsCtx_t *pHddTdlsCtx;
#endif


   /**Connection information*/
   connection_info_t conn_info;

   roaming_info_t roam_info;

#if  defined (WLAN_FEATURE_VOWIFI_11R) || defined (FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_LFR)
   int     ft_carrier_on;
#endif

#ifdef WLAN_FEATURE_GTK_OFFLOAD
   tSirGtkOffloadParams gtkOffloadReqParams;
#endif
   /*Increment whenever ibss New peer joins and departs the network */
   int ibss_sta_generation;

   /* Indication of wep/wpa-none keys installation */
   v_BOOL_t ibss_enc_key_installed;

   /*Save the wep/wpa-none keys*/
   tCsrRoamSetKey ibss_enc_key;
   tSirPeerInfoRspParams ibss_peer_info;

   v_BOOL_t hdd_ReassocScenario;

   /* STA ctx debug variables */
   int staDebugState;

   struct hdd_mon_set_ch_info ch_info;
#ifdef WLAN_FEATURE_NAN_DATAPATH
   struct nan_datapath_ctx ndp_ctx;
#endif

   uint8_t broadcast_staid;
};

#define BSS_STOP    0
#define BSS_START   1
typedef struct hdd_hostapd_state_s
{
    int bssState;
    vos_event_t vosEvent;
    vos_event_t stop_bss_event;
    vos_event_t sta_disassoc_event;
    VOS_STATUS vosStatus;
    v_BOOL_t bCommit;

} hdd_hostapd_state_t;

/**
 * enum bss_stop_reason - reasons why a BSS is stopped.
 * @BSS_STOP_REASON_INVALID: no reason specified explicitly.
 * @BSS_STOP_DUE_TO_MCC_SCC_SWITCH: BSS stopped due to host
 *  driver is trying to switch AP role to a different channel
 *  to maintain SCC mode with the STA role on the same card.
 *  this usually happens when STA is connected to an external
 *  AP that runs on a different channel
 */
enum bss_stop_reason
{
	BSS_STOP_REASON_INVALID = 0,
	BSS_STOP_DUE_TO_MCC_SCC_SWITCH = 1,
};

/*
 * Per station structure kept in HDD for multiple station support for SoftAP
*/
typedef struct {
    /** The station entry is used or not  */
    v_BOOL_t isUsed;

    /** Station ID reported back from HAL (through SAP). Broadcast
     *  uses station ID zero by default in both libra and volans. */
    v_U8_t ucSTAId;

    /** MAC address of the station */
    v_MACADDR_t macAddrSTA;

    /** Current Station state so HDD knows how to deal with packet
     *  queue. Most recent states used to change TL STA state. */
    WLANTL_STAStateType tlSTAState;

   /** Transmit queues for each AC (VO,VI,BE etc). */
   hdd_list_t wmm_tx_queue[NUM_TX_QUEUES];

   /** Might need to differentiate queue depth in contention case */
   v_U16_t aTxQueueDepth[NUM_TX_QUEUES];

   /**Track whether OS TX queue has been disabled.*/
   v_BOOL_t txSuspended[NUM_TX_QUEUES];

   /**Track whether 3/4th of resources are used */
   v_BOOL_t vosLowResource;

   /** Track QoS status of station */
   v_BOOL_t isQosEnabled;

   /** The station entry for which Deauth is in progress  */
   v_BOOL_t isDeauthInProgress;

   /** Number of spatial streams supported */
   uint8_t   nss;

   /** Rate Flags for this connection */
   uint32_t  rate_flags;

   /** SUB 20 Bandwidth Flags */
   uint8_t   sub20_dynamic_channelwidth;
   /** Extended CSA capabilities */
   uint8_t   ecsa_capable;

   /** Max phy rate */
   uint32_t max_phy_rate;

   /** Tx packets */
   uint32_t tx_packets;

   /** Tx bytes */
   uint64_t tx_bytes;

   /** Rx packets */
   uint32_t rx_packets;

   /** Rx bytes */
   uint64_t rx_bytes;

   /** Last tx/rx timestamp */
   adf_os_time_t last_tx_rx_ts;

   /** Assoc timestamp */
   adf_os_time_t assoc_ts;

   /** Tx Rate */
   uint32_t tx_rate;

   /** Rx Rate */
   uint32_t rx_rate;

   /** Ampdu */
   bool ampdu;

   /** Short GI */
   bool sgi_enable;

   /** Tx stbc */
   bool tx_stbc;

   /** Rx stbc */
   bool rx_stbc;

   /** Channel Width */
   uint8_t ch_width;

   /** Mode */
   uint8_t mode;

   /** Max supported idx */
   uint8_t max_supp_idx;

   /** Max extended idx */
   uint8_t max_ext_idx;

   /** HT max mcs idx */
   uint8_t max_mcs_idx;

   /** VHT rx mcs map */
   uint8_t rx_mcs_map;

   /** VHT tx mcs map */
   uint8_t tx_mcs_map;
} hdd_station_info_t;

/**
 * struct hdd_rate_info - rate_info in HDD
 * @rate: tx/rx rate (kbps)
 * @mode: 0->11abg legacy, 1->HT, 2->VHT (refer to sir_sme_phy_mode)
 * @nss: number of streams
 * @mcs: mcs index for HT/VHT mode
 * @rate_flags: rate flags for last tx/rx
 *
 * rate info in HDD
 */
struct hdd_rate_info {
	uint32_t rate;
	uint8_t mode;
	uint8_t nss;
	uint8_t mcs;
	uint8_t rate_flags;
};

/**
 * struct hdd_fw_txrx_stats - fw txrx status in HDD
 *                            (refer to station_info struct in Kernel)
 * @tx_packets: packets transmitted to this station
 * @tx_bytes: bytes transmitted to this station
 * @rx_packets: packets received from this station
 * @rx_bytes: bytes received from this station
 * @rx_retries: cumulative retry counts
 * @tx_failed: number of failed transmissions
 * @rssi: The signal strength (dbm)
 * @tx_rate: last used tx rate info
 * @rx_rate: last used rx rate info
 *
 * fw txrx status in HDD
 */
struct hdd_fw_txrx_stats {
	uint32_t tx_packets;
	uint64_t tx_bytes;
	uint32_t rx_packets;
	uint64_t rx_bytes;
	uint32_t tx_retries;
	uint32_t tx_failed;
	int8_t rssi;
	struct hdd_rate_info tx_rate;
	struct hdd_rate_info rx_rate;
};

struct hdd_ap_ctx_s
{
   hdd_hostapd_state_t HostapdState;

   // Memory differentiation mode is enabled
   //v_U16_t uMemoryDiffThreshold;
   //v_U8_t uNumActiveAC;
   //v_U8_t uActiveACMask;
   //v_U8_t aTxQueueLimit[NUM_TX_QUEUES];

   /** Packet Count to update uNumActiveAC and uActiveACMask */
   //v_U16_t uUpdatePktCount;

   /** Station ID assigned after BSS starts */
   v_U8_t uBCStaId;

   v_U8_t uPrivacy;  // The privacy bits of configuration

   tSirWPSPBCProbeReq WPSPBCProbeReq;

   tsap_Config_t sapConfig;

   struct semaphore semWpsPBCOverlapInd;

   v_BOOL_t apDisableIntraBssFwd;

   vos_timer_t hdd_ap_inactivity_timer;

   v_U8_t   operatingChannel;

   v_BOOL_t uIsAuthenticated;

   eCsrEncryptionType ucEncryptType;

   //This will point to group key data, if it is received before start bss.
   tCsrRoamSetKey groupKey;
   // This will have WEP key data, if it is received before start bss
   tCsrRoamSetKey wepKey[CSR_MAX_NUM_KEY];

   /* WEP default key index */
   uint8_t wep_def_key_idx;

   beacon_data_t *beacon;

   v_BOOL_t bApActive;
#ifdef WLAN_FEATURE_MBSSID
   /* SAP Context */
   v_PVOID_t sapContext;
#endif
   v_BOOL_t dfs_cac_block_tx;
   enum bss_stop_reason bss_stop_reason;
   /* Fw txrx stats info */
   struct hdd_fw_txrx_stats txrx_stats;
};

typedef struct hdd_scaninfo_s
{
   /* The scan id  */
   v_U32_t scanId;

   /* The scan pending  */
   v_U32_t mScanPending;

  /* Counter for mScanPending so that the scan pending
     error log is not printed for more than 5 times    */
   v_U32_t mScanPendingCounter;

   /* Client Wait Scan Result */
   v_U32_t waitScanResult;

   /* Additional IE for scan */
   tSirAddie scanAddIE;

   /* Scan mode*/
   tSirScanType scan_mode;

   /* Scan Completion Event */
   struct completion scan_req_completion_event;

   /* completion variable for abortscan */
   struct completion abortscan_event_var;

   vos_event_t scan_finished_event;

   hdd_scan_pending_option_e scan_pending_option;

}hdd_scaninfo_t;

#ifdef WLAN_FEATURE_PACKET_FILTERING
typedef struct multicast_addr_list
{
   v_U8_t isFilterApplied;
   v_U8_t mc_cnt;
   v_U8_t *addr;
} t_multicast_add_list;
#endif

/*
 * WLAN_HDD_ADAPTER_MAGIC is a magic number used to identify net devices
 * belonging to this driver from net devices belonging to other devices.
 * Therefore, the magic number must be unique relative to the numbers for
 * other drivers in the system. If WLAN_HDD_ADAPTER_MAGIC is already defined
 * (e.g. by compiler argument), then use that. If it's not already defined,
 * then use the first 4 characters of MULTI_IF_NAME to construct the magic
 * number. If MULTI_IF_NAME is not defined, then use a default magic number.
 */
#ifndef WLAN_HDD_ADAPTER_MAGIC
#ifdef MULTI_IF_NAME
#define WLAN_HDD_ADAPTER_MAGIC						\
	(MULTI_IF_NAME[0] == 0 ? 0x574c414e :				\
	(MULTI_IF_NAME[1] == 0 ? (MULTI_IF_NAME[0] << 24) :		\
	(MULTI_IF_NAME[2] == 0 ? (MULTI_IF_NAME[0] << 24) |		\
		(MULTI_IF_NAME[1] << 16) :				\
	(MULTI_IF_NAME[0] << 24) | (MULTI_IF_NAME[1] << 16) |		\
		(MULTI_IF_NAME[2] << 8) | MULTI_IF_NAME[3])))
#else
#define WLAN_HDD_ADAPTER_MAGIC 0x574c414e //ASCII "WLAN"
#endif
#endif

/**
 * struct hdd_runtime_pm_context - context to prevent/allow runtime pm
 * @scan: scan context to prevent/allow runtime pm
 * @roc : remain on channel runtime pm context
 * @dfs : Dynamic frequency selection runtime pm context
 * @obss: Obss protection runtime pm context
 *
 * Prevent Runtime PM for scan, roc and dfs.
 */
struct hdd_runtime_pm_context {
	void *scan;
	void *roc;
	void *dfs;
	void *obss;
};

/**
 * struct hdd_adapter_pm_context - Context/Adapter to prevent/allow runtime pm
 * @connect : Connect context per adapter
 *
 * Structure to hold runtime pm contexts for each adapter
 */
struct hdd_adapter_pm_context {
	void *connect;
};

#define WLAN_HDD_MAX_HISTORY_ENTRY     10

/**
 * struct hdd_netif_queue_stats - netif queue operation statistics
 * @pause_count - pause counter
 * @unpause_count - unpause counter
 */
struct hdd_netif_queue_stats {
	uint16_t pause_count;
	uint16_t unpause_count;
};

/**
 * struct hdd_netif_queue_history - netif queue operation history
 * @time: timestamp
 * @netif_action: action type
 * @netif_reason: reason type
 * @pause_map: pause map
 */
struct hdd_netif_queue_history {
	vos_time_t time;
	uint16_t netif_action;
	uint16_t netif_reason;
	uint32_t pause_map;
};


struct hdd_adapter_s
{
   /* Magic cookie for adapter sanity verification.  Note that this
    * needs to be at the beginning of the private data structure so
    * that it will exists at the beginning of dev->priv and hence
    * will always be in mapped memory
    */
   v_U32_t magic;

   void *pHddCtx;

   /** Handle to the network device */
   struct net_device *dev;

   device_mode_t device_mode;

   /** IPv4 notifier callback for handling ARP offload on change in IP */
   struct work_struct  ipv4NotifierWorkQueue;
#ifdef WLAN_NS_OFFLOAD
   /** IPv6 notifier callback for handling NS offload on change in IP */
   struct work_struct  ipv6NotifierWorkQueue;
#endif

   //TODO Move this to sta Ctx
   struct wireless_dev wdev ;
   struct cfg80211_scan_request *request ;

   /** ops checks if Opportunistic Power Save is Enable or Not
    * ctw stores ctWindow value once we receive Opps command from
    * wpa_supplicant then using ctWindow value we need to Enable
    * Opportunistic Power Save
    */
    tANI_U8  ops;
    tANI_U32 ctw;

   /** Current MAC Address for the adapter  */
   v_MACADDR_t macAddressCurrent;

   /**Event Flags*/
   unsigned long event_flags;

   /**Device TX/RX statistics*/
   struct net_device_stats stats;
   /** HDD statistics*/
   hdd_stats_t hdd_stats;
   /** linkspeed statistics */
   tSirLinkSpeedInfo ls_stats;
   /**Mib information*/
   sHddMib_t  hdd_mib;

   tANI_U8 sessionId;
#ifdef FEATURE_WLAN_THERMAL_SHUTDOWN
   bool netif_carrier_on;
#endif

   /* Completion variable for session close */
   struct completion session_close_comp_var;

   /* Completion variable for session open */
   struct completion session_open_comp_var;

   /* Completion variable for smps force mode command */
   struct completion smps_force_mode_comp_var;
   int8_t smps_force_mode_status;

   //TODO: move these to sta ctx. These may not be used in AP
   /** completion variable for disconnect callback */
   struct completion disconnect_comp_var;

   /** Completion of change country code */
   struct completion change_country_code;

   /* completion variable for Linkup Event */
   struct completion linkup_event_var;

   /* completion variable for cancel remain on channel Event */
   struct completion cancel_rem_on_chan_var;

   /* completion variable for off channel  remain on channel Event */
   struct completion offchannel_tx_event;
   /* Completion variable for action frame */
   struct completion tx_action_cnf_event;
   /* Completion variable for remain on channel ready */
   struct completion rem_on_chan_ready_event;

   /* Completion variable for Upper Layer Authentication */
   struct completion ula_complete;

#ifdef FEATURE_WLAN_TDLS
   struct completion tdls_add_station_comp;
   struct completion tdls_del_station_comp;
   struct completion tdls_mgmt_comp;
   struct completion tdls_link_establish_req_comp;
   eHalStatus tdlsAddStaStatus;
#endif

   struct completion ibss_peer_info_comp;

   /* Track whether the linkup handling is needed  */
   v_BOOL_t isLinkUpSvcNeeded;

   /* Mgmt Frames TX completion status code */
   tANI_U32 mgmtTxCompletionStatus;

/*************************************************************
 *  Tx Queues
 */
   /** Transmit queues for each AC (VO,VI,BE etc) */
   hdd_list_t wmm_tx_queue[NUM_TX_QUEUES];
   /**Track whether VOS is in a low resource state*/
   v_BOOL_t isVosOutOfResource;

   /**Track whether 3/4th of resources are used */
   v_BOOL_t isVosLowResource;

   /**Track whether OS TX queue has been disabled.*/
   v_BOOL_t isTxSuspended[NUM_TX_QUEUES];

   /** WMM Status */
   hdd_wmm_status_t hddWmmStatus;
/*************************************************************
 */
/*************************************************************
 * TODO - Remove it later
 */
    /** Multiple station supports */
   /** Per-station structure */
   spinlock_t staInfo_lock; //To protect access to station Info
   hdd_station_info_t aStaInfo[WLAN_MAX_STA_COUNT];
   //v_U8_t uNumActiveStation;

   v_U16_t aTxQueueLimit[NUM_TX_QUEUES];
/*************************************************************
 */

#ifdef FEATURE_WLAN_WAPI
   hdd_wapi_info_t wapi_info;
#endif

   v_S7_t rssi;
   int8_t rssi_on_disconnect;
#ifdef WLAN_FEATURE_LPSS
   v_BOOL_t rssi_send;
#endif

   tANI_U8 snr;

   struct work_struct  monTxWorkQueue;
   struct sk_buff *skb_to_tx;

   union {
      hdd_station_ctx_t station;
      hdd_ap_ctx_t  ap;
   }sessionCtx;

#ifdef WLAN_FEATURE_TSF
   /* tsf value get from firmware */
   uint64_t cur_target_time;
   vos_timer_t host_capture_req_timer;
#ifdef WLAN_FEATURE_TSF_PLUS
   /* spin lock for read/write timestamps */
   spinlock_t host_target_sync_lock;
   vos_timer_t host_target_sync_timer;
   uint64_t cur_host_time;
   uint64_t last_host_time;
   uint64_t last_target_time;
   /* to store the count of continuous invalid tstamp-pair */
   int continuous_error_count;
   /* to indicate whether tsf_sync has been initialized */
   adf_os_atomic_t tsf_sync_ready_flag;
#endif /* WLAN_FEATURE_TSF_PLUS */
#endif

   hdd_cfg80211_state_t cfg80211State;

#ifdef WLAN_FEATURE_PACKET_FILTERING
   t_multicast_add_list mc_addr_list;
#endif
   uint8_t addr_filter_pattern;

   /* to store the time of last bug report generated in HDD */
   uint64_t last_tx_jiffies;
   /* stores how many times timeout happens since last bug report generation */
   uint8_t bug_report_count;

   v_BOOL_t higherDtimTransition;
   v_BOOL_t survey_idx;

   hdd_scaninfo_t scan_info;
#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
   tAniTrafStrmMetrics tsmStats;
#endif
   /* Flag to ensure PSB is configured through framework */
   v_U8_t psbChanged;
   /* UAPSD psb value configured through framework */
   v_U8_t configuredPsb;
#ifdef IPA_OFFLOAD
    void *ipa_context;
#endif
#ifdef WLAN_FEATURE_MBSSID
    /* this need to be adapter struct since adapter type can be dyn changed */
    mbssid_sap_dyn_ini_config_t sap_dyn_ini_cfg;
#endif
    struct work_struct scan_block_work;
    /* Using delayed work for ACS for Primary AP Startup to complete
     * since CSR Config is same for both AP */
    struct delayed_work acs_pending_work;
#ifdef FEATURE_BUS_BANDWIDTH
    unsigned long prev_rx_packets;
    unsigned long prev_tx_packets;
    unsigned long prev_fwd_tx_packets;
    unsigned long prev_fwd_rx_packets;
    unsigned long prev_tx_bytes;
    int connection;
#endif
    v_BOOL_t is_roc_inprogress;

#ifdef QCA_LL_TX_FLOW_CT
    vos_timer_t  tx_flow_control_timer;
    v_BOOL_t tx_flow_timer_initialized;
    unsigned int tx_flow_low_watermark;
    unsigned int tx_flow_high_watermark_offset;
#endif /* QCA_LL_TX_FLOW_CT */
    v_BOOL_t offloads_configured;

    /* DSCP to UP QoS Mapping */
    sme_QosWmmUpType hddWmmDscpToUpMap[WLAN_HDD_MAX_DSCP+1];

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
   v_BOOL_t isLinkLayerStatsSet;
#endif
   v_U8_t linkStatus;

   /* variable for temperature in Celsius */
   int temperature;

    /* Time stamp for last completed RoC request */
    v_TIME_t lastRocTs;

    /* Time stamp for start RoC request */
    v_TIME_t startRocTs;

    /* State for synchronous OCB requests to WMI */
    struct sir_ocb_set_config_response ocb_set_config_resp;
    struct sir_ocb_get_tsf_timer_response ocb_get_tsf_timer_resp;
    struct sir_dcc_get_stats_response *dcc_get_stats_resp;
    struct sir_dcc_update_ndl_response dcc_update_ndl_resp;
    struct dsrc_radio_chan_stats_ctxt dsrc_chan_stats;
#ifdef WLAN_FEATURE_DSRC
    /* MAC addresses used for OCB interfaces */
    tSirMacAddr ocb_mac_address[VOS_MAX_CONCURRENCY_PERSONA];
    int ocb_mac_addr_count;
#endif
    struct hdd_adapter_pm_context runtime_context;
    struct mib_stats_metrics mib_stats;

    /* BITMAP indicating pause reason */
    uint32_t pause_map;
    spinlock_t pause_map_lock;

    adf_os_time_t start_time;
    adf_os_time_t last_time;
    adf_os_time_t total_pause_time;
    adf_os_time_t total_unpause_time;

    uint8_t history_index;
    struct hdd_netif_queue_history
            queue_oper_history[WLAN_HDD_MAX_HISTORY_ENTRY];
    struct hdd_netif_queue_stats queue_oper_stats[WLAN_REASON_TYPE_MAX];
    struct power_stats_response *chip_power_stats;

    /* random address management for management action frames */
    spinlock_t random_mac_lock;
    struct action_frame_random_mac random_mac[MAX_RANDOM_MAC_ADDRS];
};

#define WLAN_HDD_GET_STATION_CTX_PTR(pAdapter) (&(pAdapter)->sessionCtx.station)
#define WLAN_HDD_GET_AP_CTX_PTR(pAdapter) (&(pAdapter)->sessionCtx.ap)
#define WLAN_HDD_GET_WEXT_STATE_PTR(pAdapter)  (&(pAdapter)->sessionCtx.station.WextState)
#define WLAN_HDD_GET_CTX(pAdapter) ((hdd_context_t*)pAdapter->pHddCtx)
#define WLAN_HDD_GET_HAL_CTX(pAdapter)  (((hdd_context_t*)(pAdapter->pHddCtx))->hHal)
#define WLAN_HDD_GET_HOSTAP_STATE_PTR(pAdapter) (&(pAdapter)->sessionCtx.ap.HostapdState)
#define WLAN_HDD_GET_CFG_STATE_PTR(pAdapter)  (&(pAdapter)->cfg80211State)
#ifdef WLAN_FEATURE_MBSSID
#define WLAN_HDD_GET_SAP_CTX_PTR(pAdapter) (pAdapter->sessionCtx.ap.sapContext)
#endif
#ifdef FEATURE_WLAN_TDLS
#define WLAN_HDD_IS_TDLS_SUPPORTED_ADAPTER(pAdapter) \
        (((WLAN_HDD_INFRA_STATION != pAdapter->device_mode) && \
        (WLAN_HDD_P2P_CLIENT != pAdapter->device_mode)) ? 0 : 1)
#define WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter) \
        ((WLAN_HDD_IS_TDLS_SUPPORTED_ADAPTER(pAdapter)) ? \
        (tdlsCtx_t*)(pAdapter)->sessionCtx.station.pHddTdlsCtx : NULL)
#endif
#ifdef WLAN_FEATURE_NAN_DATAPATH
#define WLAN_HDD_GET_NDP_CTX_PTR(adapter) (&(adapter)->sessionCtx.station.ndp_ctx)
#define WLAN_HDD_IS_NDP_ENABLED(hdd_ctx) ((hdd_ctx)->nan_datapath_enabled)
#else
/* WLAN_HDD_GET_NDP_CTX_PTR and WLAN_HDD_GET_NDP_WEXT_STATE_PTR are not defined
 * intentionally so that all references to these must be within NDP code.
 * non-NDP code can call WLAN_HDD_IS_NDP_ENABLED(), and when it is enabled,
 * invoke NDP code to do all work.
 */
#define WLAN_HDD_IS_NDP_ENABLED(hdd_ctx) (false)
#endif

/* Set mac address locally administered bit */
#define WLAN_HDD_RESET_LOCALLY_ADMINISTERED_BIT(macaddr) (macaddr[0] &= 0xFD)

#define HDD_DEFAULT_MCC_P2P_QUOTA    70
#define HDD_RESET_MCC_P2P_QUOTA      50

typedef struct hdd_adapter_list_node
{
   hdd_list_node_t node;     // MUST be first element
   hdd_adapter_t *pAdapter;
}hdd_adapter_list_node_t;

typedef struct hdd_priv_data_s
{
   tANI_U8 *buf;
   int used_len;
   int total_len;
}hdd_priv_data_t;

#ifdef FEATURE_GREEN_AP

#define GREEN_AP_PS_ON_TIME        (0)
#define GREEN_AP_PS_DELAY_TIME     (20)

/*
 * Green-AP power save state
 */
typedef enum
{
    GREEN_AP_PS_IDLE_STATE = 1,
    GREEN_AP_PS_OFF_STATE,
    GREEN_AP_PS_WAIT_STATE,
    GREEN_AP_PS_ON_STATE,
}hdd_green_ap_ps_state_t;

typedef enum
{
    GREEN_AP_PS_START_EVENT = 1,
    GREEN_AP_PS_STOP_EVENT,
    GREEN_AP_ADD_STA_EVENT,
    GREEN_AP_DEL_STA_EVENT,
    GREEN_AP_PS_ON_EVENT,
    GREEN_AP_PS_WAIT_EVENT,
}hdd_green_ap_event_t;

typedef struct
{
    uint64_t ps_on_count;
    v_TIME_t ps_on_prev_ticks;
    v_TIME_t ps_on_ticks;

    uint64_t ps_off_count;
    v_TIME_t ps_off_prev_ticks;
    v_TIME_t ps_off_ticks;

}hdd_green_ap_stats;

/*
 * Green-AP context
 */
typedef struct
{
    v_CONTEXT_t pHddContext;

    v_U8_t ps_enable;
    v_U32_t ps_on_time;
    v_U32_t ps_delay_time;
    v_U32_t num_nodes;

    hdd_green_ap_ps_state_t ps_state;
    hdd_green_ap_event_t ps_event;

    vos_timer_t ps_timer;

    hdd_green_ap_stats stats;

    bool egap_support;

}hdd_green_ap_ctx_t;
#endif /* FEATURE_GREEN_AP */

#define  MAX_MOD_LOGLEVEL 10
typedef struct
{
    v_U8_t enable;
    v_U8_t dl_type;
    v_U8_t dl_report;
    v_U8_t dl_loglevel;
    v_U8_t index;
    v_U32_t dl_mod_loglevel[MAX_MOD_LOGLEVEL];

}fw_log_info;

/**
 * enum antenna_mode - number of TX/RX chains
 * @HDD_ANTENNA_MODE_INVALID: Invalid mode place holder
 * @HDD_ANTENNA_MODE_1X1: Number of TX/RX chains equals 1
 * @HDD_ANTENNA_MODE_2X2: Number of TX/RX chains equals 2
 * @HDD_ANTENNA_MODE_MAX: Place holder for max mode
 */
enum antenna_mode {
	HDD_ANTENNA_MODE_INVALID,
	HDD_ANTENNA_MODE_1X1,
	HDD_ANTENNA_MODE_2X2,
	HDD_ANTENNA_MODE_MAX
};

/**
 * enum smps_mode - SM power save mode
 * @HDD_SMPS_MODE_STATIC: Static power save
 * @HDD_SMPS_MODE_DYNAMIC: Dynamic power save
 * @HDD_SMPS_MODE_RESERVED: Reserved
 * @HDD_SMPS_MODE_DISABLED: Disable power save
 * @HDD_SMPS_MODE_MAX: Place holder for max mode
 */
enum smps_mode {
	HDD_SMPS_MODE_STATIC,
	HDD_SMPS_MODE_DYNAMIC,
	HDD_SMPS_MODE_RESERVED,
	HDD_SMPS_MODE_DISABLED,
	HDD_SMPS_MODE_MAX
};

#ifdef FEATURE_WLAN_EXTSCAN
/**
 * struct hdd_ext_scan_context - hdd ext scan context
 *
 * @request_id: userspace-assigned ID associated with the request
 * @response_event: Ext scan wait event
 * @response_status: Status returned by FW in response to a request
 * @ignore_cached_results: Flag to ignore cached results or not
 * @capability_response: Ext scan capability response data from target
 * @buckets_scanned: bitmask of buckets scanned in extscan cycle
 */
struct hdd_ext_scan_context {
	uint32_t request_id;
	int response_status;
	bool ignore_cached_results;
	struct completion response_event;
	struct ext_scan_capabilities_response capability_response;
	uint32_t buckets_scanned;
};
#endif /* End of FEATURE_WLAN_EXTSCAN */

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
/**
 * struct hdd_ll_stats_context - hdd link layer stats context
 *
 * @request_id: userspace-assigned link layer stats request id
 * @request_bitmap: userspace-assigned link layer stats request bitmap
 * @response_event: LL stats request wait event
 */
struct hdd_ll_stats_context {
	uint32_t request_id;
	uint32_t request_bitmap;
	struct completion response_event;
};
#endif /* End of WLAN_FEATURE_LINK_LAYER_STATS */

/**
 * struct hdd_chain_rssi_context - hdd chain rssi context
 * @response_event: chain rssi request wait event
 * @ignore_result: Flag to ignore the result or not
 * @chain_rssi: chain rssi array
 */
struct hdd_chain_rssi_context {
	struct completion response_event;
	bool ignore_result;
	struct chain_rssi_result result;
};

#ifdef WLAN_FEATURE_OFFLOAD_PACKETS
/**
 * struct hdd_offloaded_packets - request id to pattern id mapping
 * @request_id: request id
 * @pattern_id: pattern id
 *
 */
struct hdd_offloaded_packets {
	uint32_t request_id;
	uint8_t  pattern_id;
};

/**
 * struct hdd_offloaded_packets_ctx - offloaded packets context
 * @op_table: request id to pattern id table
 * @op_lock: mutex lock
 */
struct hdd_offloaded_packets_ctx {
	struct hdd_offloaded_packets op_table[MAXNUM_PERIODIC_TX_PTRNS];
	struct mutex op_lock;
};
#endif

/**
 * struct hdd_bpf_context - hdd Context for bpf
 * @magic: magic number
 * @completion: Completion variable for BPF Get Capability
 * @capability_response: capabilities response received from fw
 */
struct hdd_bpf_context {
	unsigned int magic;
	struct completion completion;
	struct sir_bpf_get_offload capability_response;
};

/**
 * struct acs_dfs_policy - Define ACS policies
 * @acs_dfs_mode: Dfs mode enabled/disabled.
 * @acs_channel: pre defined channel to avoid ACS.
 */
struct acs_dfs_policy {
	enum dfs_mode acs_dfs_mode;
	uint8_t acs_channel;
};

/**
 * struct hdd_scan_chan_info - channel info
 * @freq: radio frequence
 * @cmd flag: cmd flag
 * @noise_floor: noise floor
 * @cycle_count: cycle count
 * @rx_clear_count: rx clear count
 * @tx_frame_count: TX frame count
 * @delta_cycle_count: delta of cc
 * @delta_rx_clear_count: delta of rcc
 * @delta_tx_frame_count: delta of tfc
 * @clock_freq: clock frequence MHZ
 */
struct hdd_scan_chan_info {
	uint32_t freq;
	uint32_t cmd_flag;
	uint32_t noise_floor;
	uint32_t cycle_count;
	uint32_t rx_clear_count;
	uint32_t tx_frame_count;
	uint32_t delta_cycle_count;
	uint32_t delta_rx_clear_count;
	uint32_t delta_tx_frame_count;
	uint32_t clock_freq;
};

#ifdef WLAN_FEATURE_SAP_TO_FOLLOW_STA_CHAN
typedef struct sap_ch_switch_with_csa_ctx
{
    v_BOOL_t is_ch_sw_through_sta_csa;
    u_int8_t tbtt_count;
    v_U8_t csa_to_channel; //channel on which SAP will move after sending CSA
    v_U8_t sap_chan_sw_pending; //SAP channel switch pending after STA disconnect
    vos_timer_t hdd_ap_chan_switch_timer; //timer to init SAP chan switch
    v_BOOL_t chan_sw_timer_initialized;
    v_U8_t def_csa_channel_on_disc;
    v_BOOL_t scan_only_dfs_channels;
    struct mutex sap_ch_sw_lock; //Synchronize access to sap_chan_sw_pending
}sap_ch_switch_ctx;
#endif

/** Adapter stucture definition */

struct hdd_context_s
{
   /** Global VOS context  */
   v_CONTEXT_t pvosContext;

   /** HAL handle...*/
   tHalHandle hHal;

   struct wiphy *wiphy ;
   //TODO Remove this from here.

   hdd_list_t hddAdapters; //List of adapters

   /* One per STA: 1 for BCMC_STA_ID, 1 for each SAP_SELF_STA_ID, 1 for WDS_STAID */
   hdd_adapter_t *sta_to_adapter[WLAN_MAX_STA_COUNT + VOS_MAX_NO_OF_SAP_MODE + 2]; //One per sta. For quick reference.

   /** Pointer for firmware image data */
   const struct firmware *fw;

   /** Pointer for configuration data */
   const struct firmware *cfg;

   /** Pointer for nv data */
   const struct firmware *nv;

   /** Pointer to the parent device */
   struct device *parent_dev;

   /** Config values read from qcom_cfg.ini file */
   hdd_config_t *cfg_ini;
   wlan_hdd_ftm_status_t ftm;
   /** completion variable for full power callback */
   struct completion full_pwr_comp_var;
   /** completion variable for Request BMPS callback */
   struct completion req_bmps_comp_var;

   /** completion variable for standby callback */
   struct completion standby_comp_var;

   /* Completion  variable to indicate Rx Thread Suspended */
   struct completion rx_sus_event_var;

   /* Completion  variable to indicate Tx Thread Suspended */
   struct completion tx_sus_event_var;

   /* Completion  variable to indicate Mc Thread Suspended */
   struct completion mc_sus_event_var;

   struct completion reg_init;

   v_BOOL_t isWlanSuspended;

   v_BOOL_t isTxThreadSuspended;

   v_BOOL_t isMcThreadSuspended;

   v_BOOL_t isRxThreadSuspended;

#ifdef QCA_CONFIG_SMP
   v_BOOL_t isTlshimRxThreadSuspended;
#endif

   volatile v_BOOL_t isLogpInProgress;

   v_BOOL_t isLoadInProgress;

   v_BOOL_t isUnloadInProgress;
#ifdef FEATURE_WLAN_THERMAL_SHUTDOWN
   bool system_suspended;
   volatile int thermal_suspend_state;
   spinlock_t thermal_suspend_lock;
   struct workqueue_struct *thermal_suspend_wq;
   struct delayed_work thermal_suspend_work;
#endif

   /**Track whether driver has been suspended.*/
   hdd_ps_state_t hdd_ps_state;

   /* Track whether Mcast/Bcast Filter is enabled.*/
   v_BOOL_t hdd_mcastbcast_filter_set;

   /* Track whether ignore DTIM is enabled*/
   v_BOOL_t hdd_ignore_dtim_enabled;
   v_U32_t hdd_actual_ignore_DTIM_value;
   v_U32_t hdd_actual_LI_value;


   v_BOOL_t hdd_wlan_suspended;
   v_BOOL_t suspended;

   spinlock_t filter_lock;

   /* Lock to avoid race condition during start/stop bss */
   struct mutex sap_lock;

   /** ptt Process ID*/
   v_SINT_t ptt_pid;
#ifdef WLAN_KD_READY_NOTIFIER
   v_BOOL_t kd_nl_init;
#endif /* WLAN_KD_READY_NOTIFIER */

#ifdef FEATURE_OEM_DATA_SUPPORT
   /* OEM App registered or not */
   v_BOOL_t oem_app_registered;

   /* OEM App Process ID */
   v_SINT_t oem_pid;
#endif

   v_U8_t change_iface;

   /** Concurrency Parameters*/
   tVOS_CONCURRENCY_MODE concurrency_mode;

   v_U8_t no_of_open_sessions[VOS_MAX_NO_OF_MODE];
   v_U8_t no_of_active_sessions[VOS_MAX_NO_OF_MODE];

   /* Number of times riva restarted */
   v_U32_t  hddRivaResetStats;

   /* Can we allow AMP connection right now*/
   v_BOOL_t isAmpAllowed;

   /** P2P Device MAC Address for the adapter  */
   v_MACADDR_t p2pDeviceAddress;

   /* Thermal mitigation information */
   hdd_thermal_mitigation_info_t tmInfo;

   vos_wake_lock_t rx_wake_lock;

   /*
    * Framework initiated driver restarting
    *    hdd_reload_timer   : Restart retry timer
    *    isRestartInProgress: Restart in progress
    *    hdd_restart_retries: Restart retries
    *
    */
   vos_timer_t hdd_restart_timer;
   atomic_t isRestartInProgress;
   u_int8_t hdd_restart_retries;

   vos_wake_lock_t sap_wake_lock;

#ifdef FEATURE_WLAN_TDLS
    eTDLSSupportMode tdls_mode;
    eTDLSSupportMode tdls_mode_last;
    tdlsConnInfo_t tdlsConnInfo[HDD_MAX_NUM_TDLS_STA];
    /* maximum TDLS station number allowed upon runtime condition */
    tANI_U16 max_num_tdls_sta;
    /* TDLS peer connected count */
    tANI_U16 connected_peer_count;
    tdls_scan_context_t tdls_scan_ctxt;
    /* Lock to avoid race condition during TDLS operations*/
    struct mutex tdls_lock;
    tANI_U8      tdls_off_channel;
    tANI_U16     tdls_channel_offset;
    int32_t      tdls_fw_off_chan_mode;
    bool         tdls_nss_switch_in_progress;
    bool         tdls_nss_teardown_complete;
    int32_t      tdls_nss_transition_mode;
    int32_t      tdls_teardown_peers_cnt;
#endif

#ifdef IPA_OFFLOAD
    void *hdd_ipa;
#ifdef IPA_UC_OFFLOAD
    /* CE resources */
    v_U32_t ce_sr_base_paddr;
    v_U32_t ce_sr_ring_size;
    v_U32_t ce_reg_paddr;

    /* WLAN TX:IPA->WLAN */
    v_U32_t tx_comp_ring_base_paddr;
    v_U32_t tx_comp_ring_size;
    v_U32_t tx_num_alloc_buffer;

    /* WLAN RX:WLAN->IPA */
    v_U32_t rx_rdy_ring_base_paddr;
    v_U32_t rx_rdy_ring_size;
    v_U32_t rx_proc_done_idx_paddr;

    /* IPA UC doorbell registers paddr */
    v_U32_t tx_comp_doorbell_paddr;
    v_U32_t rx_ready_doorbell_paddr;
#endif /* IPA_UC_OFFLOAD */
#endif
    /* MC/BC Filter state variable
     * This always contains the value that is currently
     * configured
     * */
    v_U8_t configuredMcastBcastFilter;

    v_U8_t sus_res_mcastbcast_filter;

    v_BOOL_t sus_res_mcastbcast_filter_valid;

    /* debugfs entry */
    struct dentry *debugfs_phy;

#ifdef WLAN_POWER_DEBUGFS
    /* mutex lock to block concurrent access */
    struct mutex power_stats_lock;
#endif

    /* Use below lock to protect access to isSchedScanUpdatePending
     * since it will be accessed in two different contexts.
     */
    spinlock_t schedScan_lock;

    // Flag keeps track of wiphy suspend/resume
    v_BOOL_t isWiphySuspended;

    // Indicates about pending sched_scan results
    v_BOOL_t isSchedScanUpdatePending;

#ifdef FEATURE_BUS_BANDWIDTH
    /* DDR bus bandwidth compute timer */
    vos_timer_t    bus_bw_timer;
    int            cur_vote_level;
    spinlock_t     bus_bw_lock;
    int            cur_rx_level;
    uint64_t       prev_rx;
    int            cur_tx_level;
    uint64_t       prev_tx;
#endif

    /* VHT80 allowed*/
    v_BOOL_t isVHT80Allowed;

    struct completion ready_to_suspend;
    /* defining the solution type */
    v_U32_t target_type;

    /* defining the firmware version */
    v_U32_t target_fw_version;
    v_U32_t dfs_radar_found;

    /* defining the chip/rom version */
    v_U32_t target_hw_version;
    /* defining the chip/rom revision */
    v_U32_t target_hw_revision;
    /* chip/rom name */
    const char *target_hw_name;
    struct regulatory reg;
#ifdef FEATURE_WLAN_CH_AVOID
    v_U16_t unsafe_channel_count;
    v_U16_t unsafe_channel_list[NUM_20MHZ_RF_CHANNELS];
#endif /* FEATURE_WLAN_CH_AVOID */

    v_U8_t max_intf_count;
    v_U8_t current_intf_count;
#ifdef WLAN_FEATURE_LPSS
    v_U8_t lpss_support;
#endif
    uint8_t ap_arpns_support;
    tSirScanType ioctl_scan_mode;

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    adf_os_work_t  sta_ap_intf_check_work;
#endif

    struct work_struct  sap_start_work;
    bool is_sap_restart_required;
    bool is_ch_avoid_in_progress;

    bool is_sta_connection_pending;
    spinlock_t sap_update_info_lock;
    spinlock_t sta_update_info_lock;

    v_U8_t dev_dfs_cac_status;

    v_BOOL_t btCoexModeSet;
#ifdef FEATURE_GREEN_AP
    hdd_green_ap_ctx_t *green_ap_ctx;
#endif
    fw_log_info fw_log_settings;
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
    vos_timer_t skip_acs_scan_timer;
    v_U8_t skip_acs_scan_status;
    uint8_t *last_acs_channel_list;
    uint8_t num_of_channels;
    spinlock_t acs_skip_lock;
#endif

    vos_wake_lock_t sap_dfs_wakelock;
    atomic_t sap_dfs_ref_cnt;

#ifdef WLAN_FEATURE_EXTWOW_SUPPORT
    v_BOOL_t is_extwow_app_type1_param_set;
    v_BOOL_t is_extwow_app_type2_param_set;
    v_BOOL_t ext_wow_should_suspend;
    struct completion ready_to_extwow;
#endif

    /* Time since boot up to extscan start (in micro seconds) */
    v_U64_t ext_scan_start_since_boot;

    /* RoC request queue and work */
    struct delayed_work rocReqWork;
    hdd_list_t hdd_roc_req_q;
    bool mcc_mode;
    unsigned long g_event_flags;
    uint8_t miracast_value;
    /* Dfs lock used to syncronize on sap channel switch during
     * radar found indication and application triggered channel
     * switch
     */
    spinlock_t dfs_lock;

#ifdef FEATURE_WLAN_EXTSCAN
    struct hdd_ext_scan_context ext_scan_context;
#endif /* FEATURE_WLAN_EXTSCAN */

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
    struct hdd_ll_stats_context ll_stats_context;
#endif /* End of WLAN_FEATURE_LINK_LAYER_STATS */

    struct hdd_chain_rssi_context chain_rssi_context;

    struct mutex memdump_lock;
    uint16_t driver_dump_size;
    uint8_t *driver_dump_mem;

    /* number of rf chains supported by target */
    uint32_t  num_rf_chains;

    /* Is htTxSTBC supported by target */
    uint8_t   ht_tx_stbc_supported;

    bool ns_offload_enable;
#ifdef WLAN_NS_OFFLOAD
    /* IPv6 notifier callback for handling NS offload on change in IP */
    struct notifier_block ipv6_notifier;
#endif
    /* IPv4 notifier callback for handling ARP offload on change in IP */
    struct notifier_block ipv4_notifier;

#ifdef WLAN_FEATURE_OFFLOAD_PACKETS
    struct hdd_offloaded_packets_ctx op_ctx;
#endif
    bool per_band_chainmask_supp;
    uint16_t hdd_txrx_hist_idx;
    struct hdd_tx_rx_histogram *hdd_txrx_hist;
    struct hdd_runtime_pm_context runtime_context;
    bool hbw_requested;
#ifdef IPA_OFFLOAD
    struct completion ipa_ready;
#endif
    uint8_t supp_2g_chain_mask;
    uint8_t supp_5g_chain_mask;
    /* Current number of TX X RX chains being used */
    enum antenna_mode current_antenna_mode;
    bool bpf_enabled;
    uint16_t wmi_max_len;
    /*
     * place to store FTM capab of target. This allows changing of FTM capab
     * at runtime and intersecting it with target capab before updating.
     */
    uint32_t fine_time_meas_cap_target;
    uint32_t rx_high_ind_cnt;

    int radio_index;

#ifdef WLAN_FEATURE_NAN_DATAPATH
    bool nan_datapath_enabled;
#endif
    unsigned int last_scan_bug_report_timestamp;
    bool driver_being_stopped; /* Track if DRIVER STOP cmd is sent */
    uint8_t max_mc_addr_list;
    struct acs_dfs_policy acs_policy;
    uint8_t max_peers;
    /* bit map to set/reset TDLS by different sources */
    unsigned long tdls_source_bitmap;
    /* tdls source timer to enable/disable TDLS on p2p listen */
    vos_timer_t tdls_source_timer;
    struct hdd_scan_chan_info *chan_info;
    struct mutex chan_info_lock;

    uint32_t no_of_probe_req_ouis;
    struct vendor_oui *probe_req_voui;
    v_U8_t last_scan_reject_session_id;
    scan_reject_states last_scan_reject_reason;
    v_TIME_t last_scan_reject_timestamp;
    uint8_t scan_reject_cnt;
    uint8_t hdd_dfs_regdomain;
#ifdef WLAN_FEATURE_TSF
    /* indicate whether tsf has been initialized */
    adf_os_atomic_t tsf_ready_flag;
    /* indicate whether it's now capturing tsf(updating tstamp-pair) */
    adf_os_atomic_t cap_tsf_flag;
    /* the context that is capturing tsf */
    hdd_adapter_t *cap_tsf_context;
#endif
    /* flag to show whether moniotr mode is enabled */
    bool is_mon_enable;
    v_MACADDR_t hw_macaddr;
#ifdef WLAN_FEATURE_SAP_TO_FOLLOW_STA_CHAN
    sap_ch_switch_ctx  ch_switch_ctx;
#endif//#ifdef WLAN_FEATURE_SAP_TO_FOLLOW_CHAN
};

/*---------------------------------------------------------------------------
  Function declarations and documentation
  -------------------------------------------------------------------------*/
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
void wlan_hdd_check_sta_ap_concurrent_ch_intf(void *sta_pAdapter);
#endif

const char* hdd_device_mode_to_string(uint8_t device_mode);

VOS_STATUS hdd_get_front_adapter( hdd_context_t *pHddCtx,
                                  hdd_adapter_list_node_t** ppAdapterNode);

VOS_STATUS hdd_get_next_adapter( hdd_context_t *pHddCtx,
                                 hdd_adapter_list_node_t* pAdapterNode,
                                 hdd_adapter_list_node_t** pNextAdapterNode);

VOS_STATUS hdd_remove_adapter( hdd_context_t *pHddCtx,
                               hdd_adapter_list_node_t* pAdapterNode);

VOS_STATUS hdd_remove_front_adapter( hdd_context_t *pHddCtx,
                                     hdd_adapter_list_node_t** ppAdapterNode);

VOS_STATUS hdd_add_adapter_back( hdd_context_t *pHddCtx,
                                 hdd_adapter_list_node_t* pAdapterNode);

VOS_STATUS hdd_add_adapter_front( hdd_context_t *pHddCtx,
                                  hdd_adapter_list_node_t* pAdapterNode);

/**
 * hdd_get_current_vdev_sta_count() - get currnet vdev sta count
 * @hdd_ctx: hdd context
 *
 * Traverse the adapter list to get the vdev sta count
 *
 * Return: vdev sta count
 */
uint32_t hdd_get_current_vdev_sta_count(hdd_context_t *hdd_ctx);

hdd_adapter_t* hdd_open_adapter( hdd_context_t *pHddCtx, tANI_U8 session_type,
                                 const char* name, tSirMacAddr macAddr,
                                 unsigned char name_assign_type,
                                 tANI_U8 rtnl_held );
VOS_STATUS hdd_close_adapter( hdd_context_t *pHddCtx, hdd_adapter_t *pAdapter, tANI_U8 rtnl_held );
VOS_STATUS hdd_close_all_adapters( hdd_context_t *pHddCtx );
VOS_STATUS hdd_stop_all_adapters( hdd_context_t *pHddCtx );
VOS_STATUS hdd_reset_all_adapters( hdd_context_t *pHddCtx );
VOS_STATUS hdd_start_all_adapters( hdd_context_t *pHddCtx );
VOS_STATUS hdd_reconnect_all_adapters( hdd_context_t *pHddCtx );
void hdd_dump_concurrency_info(hdd_context_t *pHddCtx);
hdd_adapter_t * hdd_get_adapter_by_name( hdd_context_t *pHddCtx, tANI_U8 *name );
hdd_adapter_t * hdd_get_adapter_by_vdev( hdd_context_t *pHddCtx,
                                         tANI_U32 vdev_id );
hdd_adapter_t * hdd_get_adapter_by_macaddr( hdd_context_t *pHddCtx, tSirMacAddr macAddr );

/**
 * hdd_get_adapter_by_rand_macaddr() - Get adapter using random DA of MA frms
 * @hdd_ctx: Pointer to hdd context
 * @mac_addr: random mac address
 *
 * This function is used to get adapter from randomized destination mac
 * address present in management action frames
 *
 * Return: If randomized addr is present then return pointer to adapter
 *         else NULL
 */
hdd_adapter_t * hdd_get_adapter_by_rand_macaddr(hdd_context_t *hdd_ctx, tSirMacAddr mac_addr);

VOS_STATUS hdd_init_station_mode( hdd_adapter_t *pAdapter );
hdd_adapter_t * hdd_get_adapter( hdd_context_t *pHddCtx, device_mode_t mode );
void hdd_deinit_adapter(hdd_context_t *pHddCtx, hdd_adapter_t *pAdapter,
                        bool rtnl_held);
VOS_STATUS hdd_stop_adapter( hdd_context_t *pHddCtx, hdd_adapter_t *pAdapter,
                             const v_BOOL_t bCloseSession);
void hdd_set_station_ops( struct net_device *pWlanDev );
tANI_U8* wlan_hdd_get_intf_addr(hdd_context_t* pHddCtx);
void wlan_hdd_release_intf_addr(hdd_context_t* pHddCtx, tANI_U8* releaseAddr);
v_U8_t hdd_get_operating_channel( hdd_context_t *pHddCtx, device_mode_t mode );

void hdd_set_conparam ( v_UINT_t newParam );
tVOS_CON_MODE hdd_get_conparam( void );

void wlan_hdd_enable_deepsleep(v_VOID_t * pVosContext);
v_BOOL_t hdd_is_suspend_notify_allowed(hdd_context_t* pHddCtx);
void hdd_abort_mac_scan(hdd_context_t* pHddCtx, tANI_U8 sessionId,
                        eCsrAbortReason reason);
void hdd_cleanup_actionframe( hdd_context_t *pHddCtx, hdd_adapter_t *pAdapter );

void crda_regulatory_entry_default(v_U8_t *countryCode, int domain_id);
void wlan_hdd_set_concurrency_mode(hdd_context_t *pHddCtx,
                                   tVOS_CON_MODE mode);
void wlan_hdd_clear_concurrency_mode(hdd_context_t *pHddCtx,
                                     tVOS_CON_MODE mode);
void wlan_hdd_incr_active_session(hdd_context_t *pHddCtx,
                                  tVOS_CON_MODE mode);
void wlan_hdd_decr_active_session(hdd_context_t *pHddCtx,
                                  tVOS_CON_MODE mode);
uint8_t wlan_hdd_get_active_session_count(hdd_context_t *pHddCtx);
void wlan_hdd_reset_prob_rspies(hdd_adapter_t* pHostapdAdapter);
void hdd_prevent_suspend(uint32_t reason);
void hdd_allow_suspend(uint32_t reason);
void hdd_prevent_suspend_timeout(v_U32_t timeout, uint32_t reason);
void hdd_wlan_wakelock_create (void);
void hdd_wlan_wakelock_destroy(void);
void wlan_hdd_wakelocks_destroy(hdd_context_t *hdd_ctx);
void wlan_hdd_netdev_notifiers_cleanup(hdd_context_t * hdd_ctx);

bool hdd_is_ssr_required(void);
void hdd_set_ssr_required(e_hdd_ssr_required value);

VOS_STATUS hdd_enable_bmps_imps(hdd_context_t *pHddCtx);
VOS_STATUS hdd_disable_bmps_imps(hdd_context_t *pHddCtx, tANI_U8 session_type);

/**
 * hdd_thermal_suspend_queue_work() - Queue a thermal suspend work
 * @hdd_ctx:     Pointer to hdd_context_t
 * @ms: Delay time in milliseconds to execute the work
 *
 * Queue thermal suspend work on the workqueue after delay
 *
 * Return: false if work was already on a queue, true otherwise.
 */
bool hdd_thermal_suspend_queue_work(hdd_context_t *hdd_ctx, unsigned long ms);

void wlan_hdd_cfg80211_update_wiphy_caps(struct wiphy *wiphy);
VOS_STATUS hdd_setIbssPowerSaveParams(hdd_adapter_t *pAdapter);
VOS_STATUS wlan_hdd_restart_driver(hdd_context_t *pHddCtx);
void hdd_exchange_version_and_caps(hdd_context_t *pHddCtx);
void hdd_set_pwrparams(hdd_context_t *pHddCtx);
void hdd_reset_pwrparams(hdd_context_t *pHddCtx);
int wlan_hdd_validate_context(hdd_context_t *pHddCtx);
v_BOOL_t hdd_is_valid_mac_address(const tANI_U8* pMacAddr);
VOS_STATUS hdd_issta_p2p_clientconnected(hdd_context_t *pHddCtx);
void hdd_ipv4_notifier_work_queue(struct work_struct *work);
bool hdd_isConnectionInProgress(hdd_context_t *pHddCtx, v_U8_t *session_id,
				scan_reject_states *reason);
#ifdef WLAN_FEATURE_PACKET_FILTERING
int wlan_hdd_setIPv6Filter(hdd_context_t *pHddCtx, tANI_U8 filterType, tANI_U8 sessionId);
#endif

#ifdef WLAN_NS_OFFLOAD
void hdd_ipv6_notifier_work_queue(struct work_struct *work);
#endif

v_MACADDR_t* hdd_wlan_get_ibss_mac_addr_from_staid(hdd_adapter_t *pAdapter, v_U8_t staIdx);

void hdd_checkandupdate_phymode( hdd_context_t *pHddCtx);

int hdd_wmmps_helper(hdd_adapter_t *pAdapter, tANI_U8 *ptr);
int wlan_hdd_set_mc_rate(hdd_adapter_t *pAdapter, int targetRate);
#ifdef FEATURE_BUS_BANDWIDTH
void hdd_start_bus_bw_compute_timer(hdd_adapter_t *pAdapter);
void hdd_stop_bus_bw_compute_timer(hdd_adapter_t *pAdapter);
#else
static inline void hdd_start_bus_bw_compute_timer(hdd_adapter_t *pAdapter)
{
    return;
}

static inline void hdd_stop_bus_bw_computer_timer(hdd_adapter_t *pAdapter)
{
    return;
}
#endif

int hdd_wlan_startup(struct device *dev, void *hif_sc);
void __hdd_wlan_exit(void);
int hdd_wlan_notify_modem_power_state(int state);
#ifdef QCA_HT_2040_COEX
int hdd_wlan_set_ht2040_mode(hdd_adapter_t *pAdapter, v_U16_t staId,
                             v_MACADDR_t macAddrSTA, int width);
#endif

VOS_STATUS hdd_abort_mac_scan_all_adapters(hdd_context_t *pHddCtx);


#ifdef WLAN_FEATURE_LPSS
void wlan_hdd_send_status_pkg(hdd_adapter_t *pAdapter,
                              hdd_station_ctx_t *pHddStaCtx,
                              v_U8_t is_on,
                              v_U8_t is_connected);
void wlan_hdd_send_version_pkg(v_U32_t fw_version,
                               v_U32_t chip_id,
                               const char *chip_name);
void wlan_hdd_send_all_scan_intf_info(hdd_context_t *pHddCtx);
#endif
void wlan_hdd_send_svc_nlink_msg(int radio, int type, void *data, int len);
#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
void wlan_hdd_auto_shutdown_enable(hdd_context_t *hdd_ctx, v_U8_t enable);
#endif

hdd_adapter_t *hdd_get_con_sap_adapter(hdd_adapter_t *this_sap_adapter,
                                       bool check_start_bss);

boolean hdd_is_5g_supported(hdd_context_t * pHddCtx);

int wlan_hdd_scan_abort(hdd_adapter_t *pAdapter);

#ifdef FEATURE_GREEN_AP
void hdd_wlan_green_ap_mc(hdd_context_t *pHddCtx,
        hdd_green_ap_event_t event);
void hdd_wlan_green_ap_init(struct hdd_context_s *hdd_ctx);
void hdd_wlan_green_ap_deinit(struct hdd_context_s *hdd_ctx);
void hdd_wlan_green_ap_start_bss(hdd_context_t *hdd_ctx);
void hdd_wlan_green_ap_stop_bss(struct hdd_context_s *hdd_ctx);
void hdd_wlan_green_ap_add_sta(struct hdd_context_s *hdd_ctx);
void hdd_wlan_green_ap_del_sta(struct hdd_context_s *hdd_ctx);
int hdd_wlan_enable_egap(struct hdd_context_s *hdd_ctx);
#else
static inline void hdd_wlan_green_ap_init(struct hdd_context_s *hdd_ctx) {}
static inline void hdd_wlan_green_ap_deinit(struct hdd_context_s *hdd_ctx) {}
static inline void hdd_wlan_green_ap_start_bss(hdd_context_t *hdd_ctx) {}
static inline void hdd_wlan_green_ap_stop_bss(struct hdd_context_s *hdd_ctx) {}
static inline void hdd_wlan_green_ap_add_sta(struct hdd_context_s *hdd_ctx) {}
static inline void hdd_wlan_green_ap_del_sta(struct hdd_context_s *hdd_ctx) {}
static inline int hdd_wlan_enable_egap(struct hdd_context_s *hdd_ctx) {
    return -EINVAL;
}
#endif

#ifdef WLAN_FEATURE_STATS_EXT
void wlan_hdd_cfg80211_stats_ext_init(hdd_context_t *pHddCtx);
#endif

void hdd_update_macaddr(hdd_config_t *cfg_ini, v_MACADDR_t hw_macaddr);
#if defined(FEATURE_WLAN_LFR) && defined(WLAN_FEATURE_ROAM_SCAN_OFFLOAD)
void wlan_hdd_disable_roaming(hdd_adapter_t *pAdapter);
void wlan_hdd_enable_roaming(hdd_adapter_t *pAdapter);
#endif
int hdd_set_miracast_mode(hdd_adapter_t *pAdapter, tANI_U8 *command);
VOS_STATUS wlan_hdd_check_custom_con_channel_rules(hdd_adapter_t *sta_adapter,
                                              hdd_adapter_t *ap_adapter,
                                              tCsrRoamProfile *roam_profile,
                                              tScanResultHandle *scan_cache,
                                              bool *concurrent_chnl_same);
#ifdef WLAN_FEATURE_MBSSID
void wlan_hdd_stop_sap(hdd_adapter_t *ap_adapter);
void wlan_hdd_start_sap(hdd_adapter_t *ap_adapter, bool reinit);
#else
static inline void wlan_hdd_stop_sap(hdd_adapter_t *ap_adapter) {}
static inline void wlan_hdd_start_sap(hdd_adapter_t *ap_adapter, bool reinit) {}
#endif
int wlan_hdd_get_link_speed(hdd_adapter_t *sta_adapter, uint32_t *link_speed);
int hdd_wlan_go_set_mcc_p2p_quota(hdd_adapter_t *hostapd_adapter,
					uint32_t set_value);
int hdd_wlan_set_mcc_p2p_quota(hdd_adapter_t *hostapd_adapter,
					uint32_t set_value);
int hdd_set_mas(hdd_adapter_t *hostapd_adapter, uint8_t filter_type);
uint8_t hdd_is_mcc_in_24G(hdd_context_t *hdd_ctx);
bool wlan_hdd_get_fw_state(hdd_adapter_t *adapter);

#ifdef WLAN_FEATURE_LINK_LAYER_STATS
static inline bool hdd_link_layer_stats_supported(void)
{
	return true;
}

/**
 * hdd_init_ll_stats_ctx() - initialize link layer stats context
 * @hdd_ctx: Pointer to hdd context
 *
 * Return: none
 */
static inline void hdd_init_ll_stats_ctx(hdd_context_t *hdd_ctx)
{
	init_completion(&hdd_ctx->ll_stats_context.response_event);
	hdd_ctx->ll_stats_context.request_bitmap = 0;

	return;
}

/**
 * wlan_hdd_cfg80211_link_layer_stats_init() - Initialize llstats callbacks
 * @pHddCtx: HDD context
 *
 * Return: none
 */
void wlan_hdd_cfg80211_link_layer_stats_init(hdd_context_t *pHddCtx);

#else
static inline bool hdd_link_layer_stats_supported(void)
{
	return false;
}

static inline void hdd_init_ll_stats_ctx(hdd_context_t *hdd_ctx)
{
	return;
}

void wlan_hdd_cfg80211_link_layer_stats_init(hdd_context_t *pHddCtx)
{
	return;
}
#endif /* WLAN_FEATURE_LINK_LAYER_STATS */

#ifdef FEATURE_WLAN_LFR
static inline bool hdd_driver_roaming_supported(hdd_context_t *hdd_ctx)
{
	return hdd_ctx->cfg_ini->isFastRoamIniFeatureEnabled;
}
#else
static inline bool hdd_driver_roaming_supported(hdd_context_t *hdd_ctx)
{
	return false;
}
#endif

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
static inline bool hdd_firmware_roaming_supported(hdd_context_t *hdd_ctx)
{
	return hdd_ctx->cfg_ini->isRoamOffloadScanEnabled;
}
#else
static inline bool hdd_firmware_roaming_supported(hdd_context_t *hdd_ctx)
{
	return false;
}
#endif

static inline bool hdd_roaming_supported(hdd_context_t *hdd_ctx)
{
	bool val;

	val = hdd_driver_roaming_supported(hdd_ctx) ||
		hdd_firmware_roaming_supported(hdd_ctx);

	return val;
}

#ifdef CFG80211_SCAN_RANDOM_MAC_ADDR
static inline bool hdd_scan_random_mac_addr_supported(void)
{
	return true;
}
#else
static inline bool hdd_scan_random_mac_addr_supported(void)
{
	return false;
}
#endif

void hdd_get_fw_version(hdd_context_t *hdd_ctx,
			uint32_t *major_spid, uint32_t *minor_spid,
			uint32_t *siid, uint32_t *crmid);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28))
static inline void
hdd_set_needed_headroom(struct net_device *wlan_dev, uint16_t len)
{
	wlan_dev->needed_headroom = len;
}
#else
static inline void
hdd_set_needed_headroom(struct net_device *wlan_dev, uint16_t len)
{
	/* no-op */
}
#endif /* LINUX_VERSION_CODE */

#ifdef QCA_CONFIG_SMP
int wlan_hdd_get_cpu(void);
#else
static inline int wlan_hdd_get_cpu(void)
{
	return 0;
}
#endif

const char *hdd_get_fwpath(void);

uint8_t wlan_hdd_find_opclass(tHalHandle hal, uint8_t channel,
			uint8_t bw_offset);

#ifdef QCA_LL_TX_FLOW_CT
void wlan_hdd_clean_tx_flow_control_timer(hdd_context_t *hddctx,
					hdd_adapter_t *adapter);
#else
static inline void
wlan_hdd_clean_tx_flow_control_timer(hdd_context_t *hddctx,
				hdd_adapter_t *adapter)
{
}
#endif

struct cfg80211_bss *hdd_cfg80211_get_bss(struct wiphy *wiphy,
	struct ieee80211_channel *channel,
	const u8 *bssid,
	const u8 *ssid, size_t ssid_len);

void hdd_connect_result(struct net_device *dev, const u8 *bssid,
			tCsrRoamInfo *roam_info, const u8 *req_ie,
			size_t req_ie_len, const u8 * resp_ie,
			size_t resp_ie_len, u16 status, gfp_t gfp,
			bool connect_timeout,
			tSirResultCodes timeout_reason);

int wlan_hdd_init_tx_rx_histogram(hdd_context_t *pHddCtx);
void wlan_hdd_deinit_tx_rx_histogram(hdd_context_t *pHddCtx);
void wlan_hdd_display_tx_rx_histogram(hdd_context_t *pHddCtx);
void wlan_hdd_clear_tx_rx_histogram(hdd_context_t *pHddCtx);
void wlan_hdd_display_netif_queue_history(hdd_context_t *hdd_ctx);
void wlan_hdd_clear_netif_queue_history(hdd_context_t *hdd_ctx);

void hdd_runtime_suspend_init(hdd_context_t *);
void hdd_runtime_suspend_deinit(hdd_context_t *);
void hdd_indicate_mgmt_frame(tSirSmeMgmtFrameInd *frame_ind);
hdd_adapter_t *hdd_get_adapter_by_sme_session_id(hdd_context_t *hdd_ctx,
						uint32_t sme_session_id);
#ifdef FEATURE_GREEN_AP
void wlan_hdd_set_egap_support(hdd_context_t *hdd_ctx, struct hdd_tgt_cfg *cfg);
#else
static inline void wlan_hdd_set_egap_support(hdd_context_t *hdd_ctx,
					     struct hdd_tgt_cfg *cfg) {}
#endif

int wlan_hdd_update_txrx_chain_mask(hdd_context_t *hdd_ctx,
				    uint8_t chain_mask);
void
hdd_get_ibss_peer_info_cb(v_VOID_t *pUserData,
                                    tSirPeerInfoRspParams *pPeerInfo);

eHalStatus hdd_smeCloseSessionCallback(void *pContext);

int hdd_enable_disable_ca_event(hdd_context_t *hddctx,
				tANI_U8 set_value);

void wlan_hdd_undo_acs(hdd_adapter_t *adapter);
void hdd_decide_dynamic_chain_mask(hdd_context_t *hdd_ctx,
				enum antenna_mode forced);
#ifdef CONFIG_CNSS_LOGGER
/**
 * wlan_hdd_nl_init() - wrapper function to CNSS_LOGGER case
 * @hdd_ctx:	the hdd context pointer
 *
 * The nl_srv_init() will call to cnss_logger_device_register() and
 * expect to get a radio_index from cnss_logger module and assign to
 * hdd_ctx->radio_index, then to maintain the consistency to original
 * design, adding the radio_index check here, then return the error
 * code if radio_index is not assigned correctly, which means the nl_init
 * from cnss_logger is failed.
 *
 * Return: 0 if successfully, otherwise error code
 */
static inline int wlan_hdd_nl_init(hdd_context_t *hdd_ctx)
{
	hdd_ctx->radio_index = nl_srv_init(hdd_ctx->wiphy);

	/* radio_index is assigned from 0, so only >=0 will be valid index  */
	if (hdd_ctx->radio_index >= 0)
		return 0;
	else
		return -EINVAL;
}
#else
/**
 * wlan_hdd_nl_init() - wrapper function to non CNSS_LOGGER case
 * @hdd_ctx:	the hdd context pointer
 *
 * In case of non CNSS_LOGGER case, the nl_srv_init() will initialize
 * the netlink socket and return the success or not.
 *
 * Return: the return value from  nl_srv_init()
 */
static inline int wlan_hdd_nl_init(hdd_context_t *hdd_ctx)
{
	return nl_srv_init(hdd_ctx->wiphy);
}
#endif

#ifdef WLAN_FEATURE_PACKET_FILTERING
int hdd_init_packet_filtering(hdd_context_t *hdd_ctx,
					hdd_adapter_t *adapter);
void hdd_deinit_packet_filtering(hdd_adapter_t *adapter);
#else
static inline int hdd_init_packet_filtering(hdd_context_t *hdd_ctx,
						hdd_adapter_t *adapter)
{
	return 0;
}
static inline void hdd_deinit_packet_filtering(hdd_adapter_t *adapter)
{
}
#endif
enum  sap_acs_dfs_mode wlan_hdd_get_dfs_mode(enum dfs_mode mode);
void hdd_ch_avoid_cb(void *hdd_context, void *indi_param);
uint8_t hdd_find_prefd_safe_chnl(hdd_context_t *hdd_ctxt,
		hdd_adapter_t *ap_adapter);
void hdd_unsafe_channel_restart_sap(hdd_context_t *hdd_ctx);

#if defined (FEATURE_WLAN_MCC_TO_SCC_SWITCH) || \
	defined (FEATURE_WLAN_STA_AP_MODE_DFS_DISABLE)
void wlan_hdd_restart_sap(hdd_adapter_t *ap_adapter);
#else
static inline void wlan_hdd_restart_sap(hdd_adapter_t *ap_adapter)
{
}
#endif

#ifdef QCA_SUPPORT_TXRX_DRIVER_TCP_DEL_ACK
static inline
void hdd_set_driver_del_ack_enable(uint16_t session_id, hdd_context_t *hdd_ctx,
				   uint64_t rx_packets)
{
	tlshim_set_driver_del_ack_enable(session_id, rx_packets,
		hdd_ctx->cfg_ini->busBandwidthComputeInterval,
		hdd_ctx->cfg_ini->del_ack_threshold_high,
		hdd_ctx->cfg_ini->del_ack_threshold_low);

}
#else
static inline
void hdd_set_driver_del_ack_enable(uint16_t session_id, hdd_context_t *hdd_ctx,
				   uint64_t rx_packets)
{
}
#endif



int hdd_reassoc(hdd_adapter_t *pAdapter, const tANI_U8 *bssid,
		const tANI_U8 channel, const handoff_src src);

void hdd_sap_restart_handle(struct work_struct *work);

void hdd_set_rps_cpu_mask(hdd_context_t *hdd_ctx);
void hdd_initialize_adapter_common(hdd_adapter_t *adapter);
void hdd_svc_fw_shutdown_ind(struct device *dev);
void wlan_hdd_stop_enter_lowpower(hdd_context_t *hdd_ctx);
void wlan_hdd_init_chan_info(hdd_context_t *hdd_ctx);
void wlan_hdd_deinit_chan_info(hdd_context_t *hdd_ctx);

void hdd_chip_pwr_save_fail_detected_cb(void *hddctx,
				struct chip_pwr_save_fail_detected_params
				*data);

/**
 * hdd_drv_cmd_validate() - Validates for space in hdd driver command
 * @command: pointer to input data (its a NULL terminated string)
 * @len: length of command name
 *
 * This function checks for space after command name and if no space
 * is found returns error.
 *
 * Return: 0 for success non-zero for failure
 */
int hdd_drv_cmd_validate(tANI_U8 *command, int len);

/**
 * wlan_hdd_monitor_mode_enable() - function to enable/disable monitor mode
 * @hdd_ctx: pointer to HDD context
 * @enable: 0 - disable, 1 - enable
 *
 * Return: 0 for success and error number for failure
 */
int wlan_hdd_monitor_mode_enable(hdd_context_t *hdd_ctx, bool enable);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0))
static inline int
hdd_wlan_nla_put_u64(struct sk_buff *skb, int attrtype, u64 value)
{
	return nla_put_u64(skb, attrtype, value);
}
#else
static inline int
hdd_wlan_nla_put_u64(struct sk_buff *skb, int attrtype, u64 value)
{
	return nla_put_u64_64bit(skb, attrtype, value,
				 QCA_WLAN_VENDOR_ATTR_LL_STATS_PAD);
}
#endif
#endif    // end #if !defined( WLAN_HDD_MAIN_H )
