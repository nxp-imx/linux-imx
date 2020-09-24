/** @file moal_main.h
 *
 * @brief This file contains wlan driver specific defines etc.
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

/********************************************************
Change log:
    10/21/2008: initial version
********************************************************/

#ifndef _MOAL_MAIN_H
#define _MOAL_MAIN_H

/* warnfix for FS redefination if any? */
#ifdef FS
#undef FS
#endif

/* Linux header files */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/ptrace.h>
#include <linux/string.h>
#include <linux/irqreturn.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
#include <linux/namei.h>
#include <linux/fs.h>
#endif
#include <linux/of.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/config.h>
#endif

#ifdef USB
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
#include <linux/freezer.h>
#endif
#include <linux/usb.h>
#endif /* USB */

/* ASM files */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#include <asm/switch_to.h>
#else
#include <asm/system.h>
#endif

#include <linux/spinlock.h>

/* Net header files */
#include <linux/netdevice.h>
#include <linux/net.h>
#include <linux/inet.h>
#include <linux/ip.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <net/sock.h>
#include <net/arp.h>
#include <linux/rtnetlink.h>
#include <linux/inetdevice.h>

#include <linux/firmware.h>

#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
#include <linux/pm_wakeup.h>
#include <linux/device.h>
#else
#include <linux/wakelock.h>
#endif
#endif

#include "mlan.h"
#include "moal_shim.h"
/* Wireless header */
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#include <net/lib80211.h>
#include <net/cfg80211.h>
#include <net/ieee80211_radiotap.h>
#endif
#if defined(STA_WEXT) || defined(UAP_WEXT)
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include "moal_wext.h"
#endif
#ifdef STA_WEXT
#include "moal_priv.h"
#endif

#ifndef MIN
/** Find minimum */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/** Find maximum */
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define COMPAT_VERSION_CODE KERNEL_VERSION(0, 0, 0)
#define CFG80211_VERSION_CODE MAX(LINUX_VERSION_CODE, COMPAT_VERSION_CODE)

/**
 * Reason Code 3: STA is leaving (or has left) IBSS or ESS
 */
#define DEF_DEAUTH_REASON_CODE (0x3)

/**
 * 802.1 Local Experimental 1.
 */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 24)
#define REFDATA __refdata
#else
#define REFDATA
#endif

/**
 * Linux Kernels later 3.9 use CONFIG_PM_RUNTIME instead of
 * CONFIG_USB_SUSPEND
 * Linux Kernels later 3.19 use CONFIG_PM instead of
 * CONFIG_PM_RUNTIME
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
#ifdef CONFIG_PM
#ifndef CONFIG_USB_SUSPEND
#define CONFIG_USB_SUSPEND
#endif
#ifndef CONFIG_PM_RUNTIME
#define CONFIG_PM_RUNTIME
#endif
#endif
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0) */
#ifdef CONFIG_PM_RUNTIME
#ifndef CONFIG_USB_SUSPEND
#define CONFIG_USB_SUSPEND
#endif
#endif
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0) */
#endif

/**
 * Linux kernel later 3.10 use strncasecmp instead of strnicmp
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
#define strnicmp strncasecmp
#endif

/**
 * Linux kernel later 4.7 use nl80211_band instead of ieee80211_band
 * Linux kernel later 4.7 use new macro
 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
#define ieee80211_band nl80211_band
#define IEEE80211_BAND_2GHZ NL80211_BAND_2GHZ
#define IEEE80211_BAND_5GHZ NL80211_BAND_5GHZ
#define IEEE80211_NUM_BANDS NUM_NL80211_BANDS
#endif

/**
 * interface name
 */
#define default_mlan_name "mlan%d"
#define default_uap_name "uap%d"
#define default_wfd_name "wfd%d"
#define default_nan_name "nan%d"
#define default_mpl_name "mpl%d"
#define default_11p_name "ocb%d"
#define mwiphy_name "mwiphy%d"

/** Define BOOLEAN */
typedef t_u8 BOOLEAN;

#define INTF_CARDTYPE "---------%s-MXM"

#define KERN_VERSION "4X"

#define V15 "15"
#define V16 "16"
#define V17 "17"

/** Chip Magic Value */
#define CHIP_MAGIC_VALUE 0x24
/** card type SD_UART */
#define CARD_TYPE_SD_UART 0
/** card type SD_SD */
#define CARD_TYPE_SD_SD 1
/** card type PCIE_PCIE */
#define CARD_TYPE_PCIE_PCIE 2
/** card type PCIE_UART */
#define CARD_TYPE_PCIE_UART 3
/** card type USB_UART */
#define CARD_TYPE_USB_UART 4
/** card type USB_USB */
#define CARD_TYPE_USB_USB 6
/** card type PCIE_USB */
#define CARD_TYPE_PCIE_USB 7

/** Driver version */
extern char driver_version[];

/** Private structure for MOAL */
typedef struct _moal_private moal_private, *pmoal_private;
/** Handle data structure for MOAL  */
typedef struct _moal_handle moal_handle, *pmoal_handle;

/** Hardware status codes */
typedef enum _MOAL_HARDWARE_STATUS {
	HardwareStatusReady,
	HardwareStatusInitializing,
	HardwareStatusFwReady,
	HardwareStatusReset,
	HardwareStatusClosing,
	HardwareStatusNotReady
} MOAL_HARDWARE_STATUS;

/** fw cap info 11p */
#define FW_CAPINFO_80211P MBIT(24)
/** fw cap info disable nan */
#define FW_CAPINFO_DISABLE_NAN MBIT(29)
/** fw cap info BGA */
#define FW_CAPINFO_80211BGA (MBIT(8) | MBIT(9) | MBIT(10))

/** moal_wait_option */
enum { MOAL_NO_WAIT, MOAL_IOCTL_WAIT, MOAL_IOCTL_WAIT_TIMEOUT };

/** moal_main_state */
enum { MOAL_STATE_IDLE,
       MOAL_RECV_INT,
       MOAL_ENTER_WORK_QUEUE,
       MOAL_START_MAIN_PROCESS,
       MOAL_END_MAIN_PROCESS };

/** HostCmd_Header */
typedef struct _HostCmd_Header {
	/** Command */
	t_u16 command;
	/** Size */
	t_u16 size;
} HostCmd_Header;

/*
 * OS timer specific
 */

/** Timer structure */
typedef struct _moal_drv_timer {
	/** Timer list */
	struct timer_list tl;
	/** Timer function */
	void (*timer_function)(void *context);
	/** Timer function context */
	void *function_context;
	/** Time period */
	t_u32 time_period;
	/** Is timer periodic ? */
	t_u32 timer_is_periodic;
	/** Is timer cancelled ? */
	t_u32 timer_is_canceled;
} moal_drv_timer, *pmoal_drv_timer;

/**
 *  @brief Timer handler
 *
 *  @param fcontext	Timer context
 *
 *  @return		N/A
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
static inline void woal_timer_handler(struct timer_list *t)
{
	pmoal_drv_timer timer = from_timer(timer, t, tl);
#else
static inline void woal_timer_handler(unsigned long fcontext)
{
	pmoal_drv_timer timer = (pmoal_drv_timer)fcontext;
#endif

	timer->timer_function(timer->function_context);

	if (timer->timer_is_periodic == MTRUE) {
		mod_timer(&timer->tl,
			  jiffies + ((timer->time_period * HZ) / 1000));
	} else {
		timer->timer_is_canceled = MTRUE;
		timer->time_period = 0;
	}
}

/**
 *  @brief Initialize timer
 *
 *  @param timer		Timer structure
 *  @param TimerFunction	Timer function
 *  @param FunctionContext	Timer function context
 *
 *  @return			N/A
 */
static inline void woal_initialize_timer(pmoal_drv_timer timer,
					 void (*TimerFunction)(void *context),
					 void *FunctionContext)
{
	/* First, setup the timer to trigger the wlan_timer_handler proxy */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
	timer_setup(&timer->tl, woal_timer_handler, 0);
#else
	init_timer(&timer->tl);
	timer->tl.function = woal_timer_handler;
	timer->tl.data = (t_ptr)timer;
#endif

	/* Then tell the proxy which function to call and what to pass it */
	timer->timer_function = TimerFunction;
	timer->function_context = FunctionContext;
	timer->timer_is_canceled = MTRUE;
	timer->time_period = 0;
	timer->timer_is_periodic = MFALSE;
}

/**
 *  @brief Modify timer
 *
 *  @param timer		Timer structure
 *  @param millisecondperiod	Time period in millisecond
 *
 *  @return			N/A
 */
static inline void woal_mod_timer(pmoal_drv_timer timer,
				  t_u32 millisecondperiod)
{
	timer->time_period = millisecondperiod;
	mod_timer(&timer->tl, jiffies + (millisecondperiod * HZ) / 1000);
	timer->timer_is_canceled = MFALSE;
}

/**
 *  @brief Cancel timer
 *
 *  @param timer	Timer structure
 *
 *  @return		N/A
 */
static inline void woal_cancel_timer(moal_drv_timer *timer)
{
	if (timer->timer_is_periodic || in_atomic() || irqs_disabled())
		del_timer(&timer->tl);
	else
		del_timer_sync(&timer->tl);
	timer->timer_is_canceled = MTRUE;
	timer->time_period = 0;
}

#ifdef REASSOCIATION
/*
 * OS Thread Specific
 */

#include <linux/kthread.h>

/** Kernel thread structure */
typedef struct _moal_thread {
	/** Task control structrue */
	struct task_struct *task;
	/** Pointer to wait_queue_head */
	wait_queue_head_t wait_q;
	/** PID */
	pid_t pid;
	/** Pointer to moal_handle */
	void *handle;
} moal_thread;

/**
 *  @brief Activate thread
 *
 *  @param thr			Thread structure
 *  @return			N/A
 */
static inline void woal_activate_thread(moal_thread *thr)
{
	/** Initialize the wait queue */
	init_waitqueue_head(&thr->wait_q);

	/** Record the thread pid */
	thr->pid = current->pid;
}

/**
 *  @brief De-activate thread
 *
 *  @param thr			Thread structure
 *  @return			N/A
 */
static inline void woal_deactivate_thread(moal_thread *thr)
{
	/* Reset the pid */
	thr->pid = 0;
}

/**
 *  @brief Create and run the thread
 *
 *  @param threadfunc		Thread function
 *  @param thr			Thread structure
 *  @param name			Thread name
 *  @return			N/A
 */
static inline void woal_create_thread(int (*threadfunc)(void *),
				      moal_thread *thr, char *name)
{
	/* Create and run the thread */
	thr->task = kthread_run(threadfunc, thr, "%s", name);
}
#endif /* REASSOCIATION */

/* The following macros are neccessary to retain compatibility
 * around the workqueue chenges happened in kernels >= 2.6.20:
 * - INIT_WORK changed to take 2 arguments and let the work function
 *   get its own data through the container_of macro
 * - delayed works have been split from normal works to save some
 *   memory usage in struct work_struct
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
/** Work_queue work initialization */
#define MLAN_INIT_WORK(_work, _fun)                                            \
	INIT_WORK(_work, ((void (*)(void *))_fun), _work)
/** Work_queue delayed work initialization */
#define MLAN_INIT_DELAYED_WORK(_work, _fun)                                    \
	INIT_WORK(_work, ((void (*)(void *))_fun), _work)
/** Work_queue container parameter */
#define MLAN_DELAYED_CONTAINER_OF(_ptr, _type, _m) container_of(_ptr, _type, _m)
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */
/** Work_queue work initialization */
#define MLAN_INIT_WORK INIT_WORK
/** Work_queue delayed work initialization */
#define MLAN_INIT_DELAYED_WORK INIT_DELAYED_WORK
/** Work_queue container parameter */
#define MLAN_DELAYED_CONTAINER_OF(_ptr, _type, _m)                             \
	container_of(_ptr, _type, _m.work)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */

/**
 *  @brief Schedule timeout
 *
 *  @param millisec	Timeout duration in milli second
 *
 *  @return		N/A
 */
static inline void woal_sched_timeout(t_u32 millisec)
{
	set_current_state(TASK_INTERRUPTIBLE);

	schedule_timeout((millisec * HZ) / 1000);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
#define IN6PTON_XDIGIT 0x00010000
#define IN6PTON_DIGIT 0x00020000
#define IN6PTON_COLON_MASK 0x00700000
#define IN6PTON_COLON_1 0x00100000 /* single : requested */
#define IN6PTON_COLON_2 0x00200000 /* second : requested */
#define IN6PTON_COLON_1_2 0x00400000 /* :: requested */
#define IN6PTON_DOT 0x00800000 /* . */
#define IN6PTON_DELIM 0x10000000
#define IN6PTON_NULL 0x20000000 /* first/tail */
#define IN6PTON_UNKNOWN 0x40000000

static inline int xdigit2bin(char c, int delim)
{
	if (c == delim || c == '\0')
		return IN6PTON_DELIM;
	if (c == ':')
		return IN6PTON_COLON_MASK;
	if (c == '.')
		return IN6PTON_DOT;
	if (c >= '0' && c <= '9')
		return IN6PTON_XDIGIT | IN6PTON_DIGIT | (c - '0');
	if (c >= 'a' && c <= 'f')
		return IN6PTON_XDIGIT | (c - 'a' + 10);
	if (c >= 'A' && c <= 'F')
		return IN6PTON_XDIGIT | (c - 'A' + 10);
	if (delim == -1)
		return IN6PTON_DELIM;
	return IN6PTON_UNKNOWN;
}

static inline int in4_pton(const char *src, int srclen, u8 *dst, int delim,
			   const char **end)
{
	const char *s;
	u8 *d;
	u8 dbuf[4];
	int ret = 0;
	int i;
	int w = 0;

	if (srclen < 0)
		srclen = strlen(src);
	s = src;
	d = dbuf;
	i = 0;
	while (1) {
		int c;
		c = xdigit2bin(srclen > 0 ? *s : '\0', delim);
		if (!(c & (IN6PTON_DIGIT | IN6PTON_DOT | IN6PTON_DELIM |
			   IN6PTON_COLON_MASK))) {
			goto out;
		}
		if (c & (IN6PTON_DOT | IN6PTON_DELIM | IN6PTON_COLON_MASK)) {
			if (w == 0)
				goto out;
			*d++ = w & 0xff;
			w = 0;
			i++;
			if (c & (IN6PTON_DELIM | IN6PTON_COLON_MASK)) {
				if (i != 4)
					goto out;
				break;
			}
			goto cont;
		}
		w = (w * 10) + c;
		if ((w & 0xffff) > 255)
			goto out;
	cont:
		if (i >= 4)
			goto out;
		s++;
		srclen--;
	}
	ret = 1;
	moal_memcpy_ext(NULL, dst, dbuf, sizeof(dbuf), sizeof(dbuf));
out:
	if (end)
		*end = s;
	return ret;
}
#endif /* < 2.6.19 */

#ifndef __ATTRIB_ALIGN__
#define __ATTRIB_ALIGN__ __attribute__((aligned(4)))
#endif

#ifndef __ATTRIB_PACK__
#define __ATTRIB_PACK__ __attribute__((packed))
#endif

/** Get module */
#define MODULE_GET try_module_get(THIS_MODULE)
/** Put module */
#define MODULE_PUT module_put(THIS_MODULE)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE(x) init_MUTEX(x)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE_LOCKED(x) init_MUTEX_LOCKED(x)
#else
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE(x) sema_init(x, 1)
/** Initialize semaphore */
#define MOAL_INIT_SEMAPHORE_LOCKED(x) sema_init(x, 0)
#endif

/** Acquire semaphore and with blocking */
#define MOAL_ACQ_SEMAPHORE_BLOCK(x) down_interruptible(x)
/** Acquire semaphore without blocking */
#define MOAL_ACQ_SEMAPHORE_NOBLOCK(x) down_trylock(x)
/** Release semaphore */
#define MOAL_REL_SEMAPHORE(x) up(x)

/** Request FW timeout in second */
#define REQUEST_FW_TIMEOUT 30

#if defined(USB) || defined(SYSKT)
/** Max loop count (* 100ms) for waiting device ready at init time */
#define MAX_WAIT_DEVICE_READY_COUNT 50
#endif

/** Default watchdog timeout */
#define MRVDRV_DEFAULT_WATCHDOG_TIMEOUT (10 * HZ)

#ifdef UAP_SUPPORT
/** Default watchdog timeout
    Increase the value to avoid kernel Tx timeout message in case
    station in PS mode or left.
    The default value of PS station ageout timer is 40 seconds.
    Hence, the watchdog timer is set to a value higher than it.
*/
#define MRVDRV_DEFAULT_UAP_WATCHDOG_TIMEOUT (41 * HZ)
#endif

/* IOCTL Timeout */
#define MOAL_IOCTL_TIMEOUT (20 * HZ)

#ifdef ANDROID_KERNEL
/** Wake lock timeout in msec */
#define WAKE_LOCK_TIMEOUT 3000
/** Roaming Wake lock timeout in msec */
#define ROAMING_WAKE_LOCK_TIMEOUT 10000
#endif

/** Threshold value of number of times the Tx timeout happened */
/* WAR For EDMAC Test */
#define NUM_TX_TIMEOUT_THRESHOLD 10
/** Custom event : DRIVER HANG */
#define CUS_EVT_DRIVER_HANG "EVENT=DRIVER_HANG"

/** AP connected event */
#define CUS_EVT_AP_CONNECTED "EVENT=AP_CONNECTED"

/** Custom event : BW changed */
#define CUS_EVT_BW_CHANGED "EVENT=BW_CHANGED"
/** Custom event : OBSS scan parameter */
#define CUS_EVT_OBSS_SCAN_PARAM "EVENT=OBSS_SCAN_PARAM"

/** Custom event : AdHoc link sensed */
#define CUS_EVT_ADHOC_LINK_SENSED "EVENT=ADHOC_LINK_SENSED"
/** Custom event : AdHoc link lost */
#define CUS_EVT_ADHOC_LINK_LOST "EVENT=ADHOC_LINK_LOST"
/** Custom event : MIC failure, unicast */
#define CUS_EVT_MLME_MIC_ERR_UNI "MLME-MICHAELMICFAILURE.indication unicast"
/** Custom event : MIC failure, multicast */
#define CUS_EVT_MLME_MIC_ERR_MUL "MLME-MICHAELMICFAILURE.indication multicast"
/** Custom event : Beacon RSSI low */
#define CUS_EVT_BEACON_RSSI_LOW "EVENT=BEACON_RSSI_LOW"
/** Custom event : Beacon SNR low */
#define CUS_EVT_BEACON_SNR_LOW "EVENT=BEACON_SNR_LOW"
/** Custom event : Beacon RSSI high */
#define CUS_EVT_BEACON_RSSI_HIGH "EVENT=BEACON_RSSI_HIGH"
/** Custom event : Beacon SNR high */
#define CUS_EVT_BEACON_SNR_HIGH "EVENT=BEACON_SNR_HIGH"
/** Custom event : Max fail */
#define CUS_EVT_MAX_FAIL "EVENT=MAX_FAIL"
/** Custom event : Data RSSI low */
#define CUS_EVT_DATA_RSSI_LOW "EVENT=DATA_RSSI_LOW"
/** Custom event : Data SNR low */
#define CUS_EVT_DATA_SNR_LOW "EVENT=DATA_SNR_LOW"
/** Custom event : Data RSSI high */
#define CUS_EVT_DATA_RSSI_HIGH "EVENT=DATA_RSSI_HIGH"
/** Custom event : Data SNR high */
#define CUS_EVT_DATA_SNR_HIGH "EVENT=DATA_SNR_HIGH"
/** Custom event : Link Quality */
#define CUS_EVT_LINK_QUALITY "EVENT=LINK_QUALITY"
/** Custom event : Port Release */
#define CUS_EVT_PORT_RELEASE "EVENT=PORT_RELEASE"
/** Custom event : Pre-Beacon Lost */
#define CUS_EVT_PRE_BEACON_LOST "EVENT=PRE_BEACON_LOST"

/** Custom event : Deep Sleep awake */
#define CUS_EVT_DEEP_SLEEP_AWAKE "EVENT=DS_AWAKE"

/** Custom event : Host Sleep activated */
#define CUS_EVT_HS_ACTIVATED "HS_ACTIVATED"
/** Custom event : Host Sleep deactivated */
#define CUS_EVT_HS_DEACTIVATED "HS_DEACTIVATED"
/** Custom event : Host Sleep wakeup */
#define CUS_EVT_HS_WAKEUP "HS_WAKEUP"

/** Wakeup Reason */
typedef enum {
	NO_HSWAKEUP_REASON = 0, // 0.unknown
	BCAST_DATA_MATCHED, // 1. Broadcast data matched
	MCAST_DATA_MATCHED, // 2. Multicast data matched
	UCAST_DATA_MATCHED, // 3. Unicast data matched
	MASKTABLE_EVENT_MATCHED, // 4. Maskable event matched
	NON_MASKABLE_EVENT_MATCHED, // 5. Non-maskable event matched
	NON_MASKABLE_CONDITION_MATCHED, // 6. Non-maskable condition matched
					// (EAPoL rekey)
	MAGIC_PATTERN_MATCHED, // 7. Magic pattern matched
	CONTROL_FRAME_MATCHED, // 8. Control frame matched
	MANAGEMENT_FRAME_MATCHED, // 9. Management frame matched
	GTK_REKEY_FAILURE, // 10. GTK rekey failure
	RESERVED // Others: reserved
} HSWakeupReason_t;

/** Custom event : WEP ICV error */
#define CUS_EVT_WEP_ICV_ERR "EVENT=WEP_ICV_ERR"

/** Custom event : Channel Switch Announcment */
#define CUS_EVT_CHANNEL_SWITCH_ANN "EVENT=CHANNEL_SWITCH_ANN"

/** Custom indiciation message sent to the application layer for WMM changes */
#define WMM_CONFIG_CHANGE_INDICATION "WMM_CONFIG_CHANGE.indication"

#ifdef UAP_SUPPORT
/** Custom event : STA connected */
#define CUS_EVT_STA_CONNECTED "EVENT=STA_CONNECTED"
/** Custom event : STA disconnected */
#define CUS_EVT_STA_DISCONNECTED "EVENT=STA_DISCONNECTED"
#endif
#define FW_DEBUG_INFO "EVENT=FW_DEBUG_INFO"

/** 10 seconds */
#define MOAL_TIMER_10S 10000
/** 5 seconds */
#define MOAL_TIMER_5S 5000
/** 1 second */
#define MOAL_TIMER_1S 1000
/** 1 milisecond */
#define MOAL_TIMER_1MS 1

/** passive scan time */
#define PASSIVE_SCAN_CHAN_TIME 110
/** active scan time */
#define ACTIVE_SCAN_CHAN_TIME 110
/** specific scan time */
#define SPECIFIC_SCAN_CHAN_TIME 110
/** passive scan time */
#define INIT_PASSIVE_SCAN_CHAN_TIME 80
/** active scan time */
#define INIT_ACTIVE_SCAN_CHAN_TIME 80
/** specific scan time */
#define INIT_SPECIFIC_SCAN_CHAN_TIME 80
/** specific scan time after connected */
#define MIN_SPECIFIC_SCAN_CHAN_TIME 40

/** Default value of re-assoc timer */
#define REASSOC_TIMER_DEFAULT 500

/** Netlink protocol number */
#define NETLINK_NXP (MAX_LINKS - 1)
/** Netlink maximum payload size */
#define NL_MAX_PAYLOAD 1024
/** Netlink multicast group number */
#define NL_MULTICAST_GROUP 1

#define MAX_RX_PENDING_THRHLD 50

/** high rx pending packets */
#define HIGH_RX_PENDING 100
/** low rx pending packets */
#define LOW_RX_PENDING 80

/** MAX Tx Pending count */
#define MAX_TX_PENDING 400

/** LOW Tx Pending count */
#define LOW_TX_PENDING 380

/** Offset for subcommand */
#define SUBCMD_OFFSET 4

/** default scan channel gap  */
#define DEF_SCAN_CHAN_GAP 50
/** default scan time per channel in miracast mode */
#define DEF_MIRACAST_SCAN_TIME 20
/** GAP value is optional */
#define GAP_FLAG_OPTIONAL MBIT(15)

/** Macro to extract the TOS field from a skb */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
#define SKB_TOS(skb) (ip_hdr(skb)->tos)
#else
#define SKB_TOS(skb) (skb->nh.iph->tos)
#endif
#define SKB_TIDV6(skb) (ipv6_get_dsfield(ipv6_hdr(skb)))
#define IS_SKB_MAGIC_VLAN(skb) (skb->priority >= 256 && skb->priority <= 263)
#define GET_VLAN_PRIO(skb) (skb->priority - 256)

/** Offset for TOS field in the IP header */
#define IPTOS_OFFSET 5

/** Offset for DSCP in the tos field */
#define DSCP_OFFSET 2

/** max retry count for wait_event_interupptible_xx while loop */
#define MAX_RETRY_CNT 100
/** wait_queue structure */
typedef struct _wait_queue {
	/** wait_queue_head */
	wait_queue_head_t wait;
	/** Wait condition */
	BOOLEAN condition;
	/** Start time */
	long start_time;
	/** Status from MLAN */
	mlan_status status;
	/** flag for wait_timeout */
	t_u8 wait_timeout;
	/** retry count */
	t_u8 retry;
} wait_queue, *pwait_queue;

/** Auto Rate */
#define AUTO_RATE 0xFF

#define STA_WEXT_MASK MBIT(0)
#define UAP_WEXT_MASK MBIT(1)
#define STA_CFG80211_MASK MBIT(2)
#define UAP_CFG80211_MASK MBIT(3)
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
/** Is STA CFG80211 enabled in module param */
#define IS_STA_CFG80211(x) (x & STA_CFG80211_MASK)
#endif
#endif
#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
/** Is UAP CFG80211 enabled in module param */
#define IS_UAP_CFG80211(x) (x & UAP_CFG80211_MASK)
#endif
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/** Is UAP or STA CFG80211 enabled in module param */
#define IS_STA_OR_UAP_CFG80211(x) (x & (STA_CFG80211_MASK | UAP_CFG80211_MASK))
#endif

#ifdef STA_WEXT
/** Is STA WEXT enabled in module param */
#define IS_STA_WEXT(x) (x & STA_WEXT_MASK)
#endif /* STA_WEXT */
#ifdef UAP_WEXT
/** Is UAP WEXT enabled in module param */
#define IS_UAP_WEXT(x) (x & UAP_WEXT_MASK)
#endif /* UAP_WEXT */
#if defined(STA_WEXT) || defined(UAP_WEXT)
/** Is UAP or STA WEXT enabled in module param */
#define IS_STA_OR_UAP_WEXT(x) (x & (STA_WEXT_MASK | UAP_WEXT_MASK))
#endif

#ifdef STA_SUPPORT
/** Driver mode STA bit */
#define DRV_MODE_STA MBIT(0)
/** Maximum STA BSS */
#define MAX_STA_BSS 1
/** Default STA BSS */
#define DEF_STA_BSS 1
#endif
#ifdef UAP_SUPPORT
/** Driver mode uAP bit */
#define DRV_MODE_UAP MBIT(1)
/** Maximum uAP BSS */
#define MAX_UAP_BSS 1
/** Default uAP BSS */
#define DEF_UAP_BSS 1
#endif
#ifdef WIFI_DIRECT_SUPPORT
/** Driver mode WIFIDIRECT bit */
#define DRV_MODE_WIFIDIRECT MBIT(2)
/** Maximum WIFIDIRECT BSS */
#define MAX_WIFIDIRECT_BSS 2
/** Default WIFIDIRECT BSS */
#define DEF_WIFIDIRECT_BSS 1
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#define DEF_VIRTUAL_BSS 0
#endif
#endif /* WIFI_DIRECT_SUPPORT */

#define DRV_MODE_WLAN (MBIT(0) | MBIT(1) | MBIT(2) | MBIT(3) | MBIT(4))

/**
 * the maximum number of adapter supported
 **/
#define MAX_MLAN_ADAPTER 3

typedef struct _moal_drv_mode {
	/** driver mode */
	t_u16 drv_mode;
	/** total number of interfaces */
	t_u16 intf_num;
	/** attribute of bss */
	mlan_bss_attr *bss_attr;
	/** name of firmware image */
	char *fw_name;
} moal_drv_mode;

/** Indicate if handle->info's address */
#define INFO_ADDR BIT(0)
#define IS_INFO_ADDR(attr) (attr & INFO_ADDR)
/** Indicate if handle's address */
#define HANDLE_ADDR BIT(1)
#define IS_HANDLE_ADDR(attr) (attr & HANDLE_ADDR)
/** Indicate if card's address */
#define CARD_ADDR BIT(2)
#define IS_CARD_ADDR(attr) (attr & CARD_ADDR)
/** indicate if priv's address */
#define PRIV_ADDR BIT(3)
#define IS_PRIV_ADDR(attr) (attr & PRIV_ADDR)

/** Debug data */
struct debug_data {
	/** Name */
	char name[32];
	/** Size */
	t_u32 size;
	/** Address */
	t_ptr addr;
	/** Attribute:
	0-7bit: start address for addr to add to, 0 means common(no specific)
	8-15bit: interface type, 0 means common(no interface specific)
	other: unused
	*/
	t_u32 attr;
};

/** Private debug data */
struct debug_data_priv {
	/** moal_private handle */
	moal_private *priv;
	/** Debug items */
	struct debug_data *items;
	/** numbre of item */
	int num_of_items;
};

/** Maximum IP address buffer length */
#define IPADDR_MAX_BUF 20
/** IP address operation: Remove */
#define IPADDR_OP_REMOVE 0

#define DROP_TCP_ACK 1
#define HOLD_TCP_ACK 2
struct tcp_sess {
	struct list_head link;
	/** tcp session info */
	t_u32 src_ip_addr;
	t_u32 dst_ip_addr;
	t_u16 src_tcp_port;
	t_u16 dst_tcp_port;
	/** tx ack packet info */
	t_u32 ack_seq;
	/** tcp ack buffer */
	void *ack_skb;
	/** priv structure */
	void *priv;
	/** pmbuf */
	void *pmbuf;
	/** timer for ack */
	moal_drv_timer ack_timer __ATTRIB_ALIGN__;
	/** timer is set */
	BOOLEAN is_timer_set;
};

struct tx_status_info {
	struct list_head link;
	/** cookie */
	t_u64 tx_cookie;
	/** seq_num */
	t_u8 tx_seq_num;
	/** cancel remain on channel when receive tx status */
	t_u8 cancel_remain_on_channel;
	/**          skb */
	void *tx_skb;
};

/** woal event type */
enum woal_event_type {
	WOAL_EVENT_CHAN_SWITCH,
	WOAL_EVENT_BGSCAN_STOP,
	WOAL_EVENT_DEAUTH,
	WOAL_EVENT_ASSOC_RESP,
};

/** woal event */
struct woal_event {
	/*list head */
	struct list_head link;
	/** type */
	enum woal_event_type type;
	/** priv pointer */
	void *priv;
	union {
		chan_band_info chan_info;
		mlan_ds_misc_assoc_rsp assoc_resp;
		int reason_code;
	};
};

#define MAX_NUM_ETHER_TYPE 8
typedef struct {
	/** number of protocols in protocol array*/
	t_u8 protocol_num;
	/** protocols supported */
	t_u16 protocols[MAX_NUM_ETHER_TYPE];
} __ATTRIB_PACK__ dot11_protocol;
typedef struct {
	/** Data rate in unit of 0.5Mbps */
	t_u16 datarate;
	/** Channel number to transmit the frame */
	t_u8 channel;
	/** Bandwidth to transmit the frame */
	t_u8 bw;
	/** Power to be used for transmission */
	t_u8 power;
	/** Priority of the packet to be transmitted */
	t_u8 priority;
	/** retry time of tx transmission*/
	t_u8 retry_limit;
	/** Reserved fields*/
	t_u8 reserved[1];
} __ATTRIB_PACK__ dot11_txcontrol;

typedef struct {
	/** Data rate of received paccket*/
	t_u16 datarate;
	/** Channel on which packet was received*/
	t_u8 channel;
	/** Rx antenna*/
	t_u8 antenna;
	/** RSSI */
	t_u8 rssi;
	/** Reserved */
	t_u8 reserved[3];
} __ATTRIB_PACK__ dot11_rxcontrol;

#define OKC_WAIT_TARGET_PMKSA_TIMEOUT (4 * HZ / 1000)
#define PMKID_LEN 16
struct pmksa_entry {
	struct list_head link;
	u8 bssid[ETH_ALEN];
	u8 pmkid[PMKID_LEN];
};

struct rf_test_mode_data {
	/* tx antenna num */
	t_u32 tx_antenna;
	/* rx antenna num */
	t_u32 rx_antenna;
	/* RF band */
	t_u32 band;
	/* RF bandwidth */
	t_u32 bandwidth;
	/* RF channel */
	t_u32 channel;
	/* Total Rx ucast/mcast/bcast pkt count */
	t_u32 rx_tot_pkt_count;
	/* Rx mcast/bcast pkt count */
	t_u32 rx_mcast_bcast_pkt_count;
	/* Rx fcs error count */
	t_u32 rx_pkt_fcs_err_count;
	/* Tx power config values */
	t_u32 tx_power_data[3];
	/* Tx continuous config values */
	t_u32 tx_cont_data[6];
	/* Tx frame config values */
	t_u32 tx_frame_data[13];
	/* BSSID */
	t_u8 bssid[ETH_ALEN];
};

/** Number of samples in histogram (/proc/mwlan/adapterX/mlan0/histogram).*/
#define HIST_MAX_SAMPLES 1048576
#define RX_RATE_MAX 196

/** SRN MAX  */
#define SNR_MAX 256
/** NOISE FLR MAX  */
#define NOISE_FLR_MAX 256
/** SIG STRENTGH MAX */
#define SIG_STRENGTH_MAX 256
/** historgram data */
typedef struct _hgm_data {
	/** snr */
	atomic_t snr[SNR_MAX];
	/** noise flr */
	atomic_t noise_flr[NOISE_FLR_MAX];
	/** sig_str */
	atomic_t sig_str[SIG_STRENGTH_MAX];
	/** num sample */
	atomic_t num_samples;
	/** rx rate */
	atomic_t rx_rate[];
} hgm_data, *phgm_data;

/** max antenna number */
#define MAX_ANTENNA_NUM 4

/* wlan_hist_proc_data */
typedef struct _wlan_hist_proc_data {
	/** antenna */
	u8 ant_idx;
	/** Private structure */
	struct _moal_private *priv;
} wlan_hist_proc_data;

enum ring_id {
	VERBOSE_RING_ID,
	EVENT_RING_ID,
	RING_ID_MAX,
};

/** Private structure for MOAL */
struct _moal_private {
	/** Handle structure */
	moal_handle *phandle;
	/** Tx timeout count */
	t_u32 num_tx_timeout;
	/** BSS index */
	t_u8 bss_index;
	/** BSS type */
	t_u8 bss_type;
	/** BSS role */
	t_u8 bss_role;
	/** bss virtual flag */
	t_u8 bss_virtual;
	/** MAC address information */
	t_u8 current_addr[ETH_ALEN];
	/** Media connection status */
	BOOLEAN media_connected;
	/** Statistics of tcp ack tx dropped */
	t_u32 tcp_ack_drop_cnt;
	/** Statistics of tcp ack tx in total from kernel */
	t_u32 tcp_ack_cnt;
#ifdef UAP_SUPPORT
	/** uAP started or not */
	BOOLEAN bss_started;
	/** host based uap flag */
	BOOLEAN uap_host_based;
	/** uAP skip CAC*/
	BOOLEAN skip_cac;
	/** tx block flag */
	BOOLEAN uap_tx_blocked;
	/** user cac period */
	t_u32 user_cac_period_msec;
	/** channel under nop */
	BOOLEAN chan_under_nop;
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** current working channel */
	struct cfg80211_chan_def chan;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	/** switch channel */
	struct cfg80211_chan_def csa_chan;
	/** beacon after channel switch */
	struct cfg80211_beacon_data beacon_after;
	/** CSA work queue */
	struct workqueue_struct *csa_workqueue;
	/** csa work */
	struct delayed_work csa_work;
#endif
#endif
#endif
#ifdef STA_SUPPORT
	/** scan type */
	t_u8 scan_type;
	/** extended capabilities */
	ExtCap_t extended_capabilities;
	/** bg_scan_start */
	t_u8 bg_scan_start;
	/** bg_scan reported */
	t_u8 bg_scan_reported;
	/** bg_scan config */
	wlan_bgscan_cfg scan_cfg;
	/** sched scaning flag */
	t_u8 sched_scanning;
	/** bgscan request id */
	t_u64 bg_scan_reqid;
#ifdef STA_CFG80211
	/** roaming enabled flag */
	t_u8 roaming_enabled;
	/** roaming required flag */
	t_u8 roaming_required;
#endif
#ifdef STA_CFG80211
	/** rssi low threshold */
	int rssi_low;
	/** channel for connect */
	struct ieee80211_channel conn_chan;
	/** bssid for connect */
	t_u8 conn_bssid[ETH_ALEN];
	/** ssid for connect */
	t_u8 conn_ssid[MLAN_MAX_SSID_LENGTH];
	/** length of ssid for connect */
	t_u8 conn_ssid_len;
	/** key data */
	t_u8 conn_wep_key[MAX_WEP_KEY_SIZE];
	/** connection param */
	struct cfg80211_connect_params sme_current;
#endif
	t_u8 wait_target_ap_pmkid;
	wait_queue_head_t okc_wait_q __ATTRIB_ALIGN__;
	struct list_head pmksa_cache_list;
	spinlock_t pmksa_list_lock;
	struct pmksa_entry *target_ap_pmksa;
	t_u8 okc_ie_len;
	t_u8 *okc_roaming_ie;
#endif
	/** Net device pointer */
	struct net_device *netdev;
	/** Net device statistics structure */
	struct net_device_stats stats;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	/** Wireless device pointer */
	struct wireless_dev *wdev;
	/** Wireless device */
	struct wireless_dev w_dev;
	/** Net device pointer */
	struct net_device *pa_netdev;
	/** channel parameter for UAP/GO */
	t_u16 channel;
#ifdef UAP_SUPPORT
	/** wep key */
	wep_key uap_wep_key[4];
	/** cipher */
	t_u32 cipher;
#endif
	/** beacon ie index */
	t_u16 beacon_index;
	/** proberesp ie index */
	t_u16 proberesp_index;
	/** proberesp_p2p_index */
	t_u16 proberesp_p2p_index;
	/** assocresp ie index */
	t_u16 assocresp_index;
	/** assocresp qos map ie index */
	t_u16 assocresp_qos_map_index;
	/** probereq index for mgmt ie */
	t_u16 probereq_index;
	/** mgmt_subtype_mask */
	t_u32 mgmt_subtype_mask;
	/** beacon wps index for mgmt ie */
	t_u16 beacon_wps_index;
	/** beacon/proberesp vendor ie index */
	t_u16 beacon_vendor_index;
#endif
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
	/** CFG80211 association description */
	t_u8 cfg_bssid[ETH_ALEN];
	/** Disconnect request from CFG80211 */
	bool cfg_disconnect;
	/** connect request from CFG80211 */
	bool cfg_connect;
	/** lock for cfg connect */
	spinlock_t connect_lock;
	/** assoc status */
	t_u32 assoc_status;
	/** rssi_threshold */
	s32 cqm_rssi_thold;
	/** rssi_high_threshold */
	s32 cqm_rssi_high_thold;
	/** rssi hysteresis */
	u32 cqm_rssi_hyst;
	/** last rssi_low */
	u8 last_rssi_low;
	/** last rssi_high */
	u8 last_rssi_high;
	/** mrvl rssi threshold */
	u8 mrvl_rssi_low;
	/** last event */
	u32 last_event;
	/** fake scan flag */
	u8 fake_scan_complete;
	/**ft ie*/
	t_u8 ft_ie[MAX_IE_SIZE];
	/**ft ie len*/
	t_u8 ft_ie_len;
	/**ft ie*/
	t_u8 pre_ft_ie[MAX_IE_SIZE];
	/**ft ie len*/
	t_u8 pre_ft_ie_len;
	/**mobility domain value*/
	t_u16 ft_md;
	/**ft capability*/
	t_u8 ft_cap;
	/** set true when receive ft auth or action ft roaming */
	t_bool ft_pre_connect;
	/**ft roaming triggered by driver or not*/
	t_bool ft_roaming_triggered_by_driver;
	/**target ap mac address for Fast Transition*/
	t_u8 target_ap_bssid[ETH_ALEN];
	/** IOCTL wait queue for FT*/
	wait_queue_head_t ft_wait_q __ATTRIB_ALIGN__;
	/** ft wait condition */
	t_bool ft_wait_condition;
#endif /* STA_SUPPORT */
#endif /* STA_CFG80211 */
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** flag for host_mlme */
	t_u8 host_mlme;
	/** flag for auth */
	t_u8 auth_flag;
#endif
#ifdef CONFIG_PROC_FS
	/** Proc entry */
	struct proc_dir_entry *proc_entry;
	/** Proc entry name */
	char proc_entry_name[IFNAMSIZ];
	/** proc entry for hist */
	struct proc_dir_entry *hist_entry;
	/** ant_hist_proc_data */
	wlan_hist_proc_data hist_proc[MAX_ANTENNA_NUM];
#endif /* CONFIG_PROC_FS */
#ifdef STA_SUPPORT
	/** Nickname */
	t_u8 nick_name[16];
	/** AdHoc link sensed flag */
	BOOLEAN is_adhoc_link_sensed;
	/** Current WEP key index */
	t_u16 current_key_index;
#ifdef REASSOCIATION
	mlan_ssid_bssid prev_ssid_bssid;
	/** Re-association required */
	BOOLEAN reassoc_required;
	/** Flag of re-association on/off */
	BOOLEAN reassoc_on;
	/** Set asynced essid flag */
	BOOLEAN set_asynced_essid_flag;
#endif /* REASSOCIATION */
	/** Report scan result */
	t_u8 report_scan_result;
	/** wpa_version */
	t_u8 wpa_version;
	/** key mgmt */
	t_u8 key_mgmt;
	/** rx_filter */
	t_u8 rx_filter;
#endif /* STA_SUPPORT */
	/** Rate index */
	t_u16 rate_index;
#if defined(STA_WEXT) || defined(UAP_WEXT)
	/** IW statistics */
	struct iw_statistics w_stats;
#endif
#ifdef UAP_WEXT
	/** Pairwise Cipher used for WPA/WPA2 mode */
	t_u16 pairwise_cipher;
	/** Group Cipher */
	t_u16 group_cipher;
	/** Protocol stored during uap wext configuratoin */
	t_u16 uap_protocol;
	/** Key Mgmt whether PSK or 1x */
	t_u16 uap_key_mgmt;
	/** Beacon IE length from hostapd */
	t_u16 bcn_ie_len;
	/** Beacon IE buffer from hostapd */
	t_u8 bcn_ie_buf[MAX_IE_SIZE];
#endif

	/** dscp mapping */
	t_u8 dscp_map[64];
	/** MLAN debug info */
	struct debug_data_priv items_priv;

	/** tcp session queue */
	struct list_head tcp_sess_queue;
	/** TCP Ack enhance flag */
	t_u8 enable_tcp_ack_enh;
	/** TCP session spin lock */
	spinlock_t tcp_sess_lock;
#if CFG80211_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	atomic_t wmm_tx_pending[4];
#endif
	/** per interface extra headroom */
	t_u16 extra_tx_head_len;
	/** TX status spin lock */
	spinlock_t tx_stat_lock;
	/** tx_seq_num */
	t_u8 tx_seq_num;
	/** tx status queue */
	struct list_head tx_stat_queue;
	/** rx hgm data */
	phgm_data hist_data[MAX_ANTENNA_NUM];
	t_u8 random_mac[MLAN_MAC_ADDR_LENGTH];
	BOOLEAN assoc_with_mac;
	t_u8 gtk_data_ready;
	mlan_ds_misc_gtk_rekey_data gtk_rekey_data;
	dot11_protocol tx_protocols;
	dot11_protocol rx_protocols;
#if defined(DRV_EMBEDDED_AUTHENTICATOR) || defined(DRV_EMBEDDED_SUPPLICANT)
	/** hostcmd_wait_q */
	wait_queue_head_t hostcmd_wait_q __ATTRIB_ALIGN__;
	/** hostcmd_wait_condition */
	t_bool hostcmd_wait_condition;
#endif
	void *rings[RING_ID_MAX];
	t_u8 pkt_fate_monitor_enable;
	void *packet_filter;
};

#ifdef SDIO
#define DUMP_FW_SDIO_V2 2
#define DUMP_FW_SDIO_V3 3

#define DUMP_REG_MAX 13

typedef struct _dump_reg_t {
	t_u8 reg_table[DUMP_REG_MAX];
	t_u8 reg_table_size;
} dump_reg_t;
#endif

#define FW_NAMW_MAX_LEN 64

/** card info */
typedef struct _card_info {
	/** support embeded supp */
	t_bool embedded_supp;
	/** support drcs */
	t_bool drcs;
	/** support Go NOA*/
	t_bool go_noa;
	/** support V16_FW_API*/
	t_bool v16_fw_api;
	/** support V17_FW_API*/
	t_bool v17_fw_api;
	/** support pmic */
	t_bool pmic;
	/** support antcfg */
	t_bool antcfg;
	/** support cal_data_cfg */
	t_bool cal_data_cfg;
	/** support WLAN_LOW_POWER_ENABLE */
	t_bool low_power_enable;
	/** rx_rate_max for hist_data: 11N: 76  11AC:196 11AX: 412 */
	t_u16 rx_rate_max;
	t_u8 histogram_table_num;
	/* feature_control */
	t_u32 feature_control;
	/* Revision id register */
	t_u32 rev_id_reg;
	/* host interface selection reg*/
	t_u32 host_strap_reg;
	/* Chip Magic Register */
	t_u32 magic_reg;
	/* FW Name */
	char fw_name[FW_NAMW_MAX_LEN];
	char fw_name_wlan[FW_NAMW_MAX_LEN];
#ifdef SDIO
	t_u8 dump_fw_info;
	t_u8 dump_fw_ctrl_reg;
	t_u8 dump_fw_start_reg;
	t_u8 dump_fw_end_reg;
	t_u8 dump_fw_host_ready;
	dump_reg_t dump_reg;
	t_u8 scratch_reg;
	t_u8 func1_reg_start;
	t_u8 func1_reg_end;
	t_u32 fw_reset_reg;
	t_u8 fw_reset_val;
	t_u32 slew_rate_reg;
	t_u8 slew_rate_bit_offset;
#endif
	t_u8 per_pkt_cfg_support;
} card_info;

#define GTK_REKEY_OFFLOAD_DISABLE 0
#define GTK_REKEY_OFFLOAD_ENABLE 1
#define GTK_REKEY_OFFLOAD_SUSPEND 2

#define MAX_KEEP_ALIVE_ID 4

/** Operation data structure for MOAL bus interfaces */
typedef struct _moal_if_ops {
	mlan_status (*register_dev)(moal_handle *handle);
	void (*unregister_dev)(moal_handle *handle);
	mlan_status (*read_reg)(moal_handle *handle, t_u32 reg, t_u32 *data);
	mlan_status (*write_reg)(moal_handle *handle, t_u32 reg, t_u32 data);
	mlan_status (*read_data_sync)(moal_handle *handle, mlan_buffer *pmbuf,
				      t_u32 port, t_u32 timeout);
	mlan_status (*write_data_sync)(moal_handle *handle, mlan_buffer *pmbuf,
				       t_u32 port, t_u32 timeout);
	mlan_status (*get_fw_name)(moal_handle *handle);
	void (*dump_fw_info)(moal_handle *handle);
	int (*dump_reg_info)(moal_handle *handle, t_u8 *buf);
	void (*reg_dbg)(moal_handle *handle);
	t_u8 (*is_second_mac)(moal_handle *handle);
} moal_if_ops;

#define WIFI_DIRECT_KERNEL_VERSION KERNEL_VERSION(2, 6, 39)

/**  Extended flags */
enum ext_mod_params {
	EXT_HW_TEST,
#ifdef CONFIG_OF
	EXT_DTS_ENABLE,
#endif
	EXT_REQ_FW_NOWAIT,
	EXT_FW_SERIAL,
#ifdef SDIO
	EXT_INTMODE,
#ifdef SDIO_SUSPEND_RESUME
	EXT_PM_KEEP_POWER,
	EXT_SHUTDOWN_HS,
#endif
#endif
	EXT_CNTRY_TXPWR,
#if defined(USB)
	EXT_SKIP_FWDNLD,
#endif
	EXT_AGGR_CTRL,
	EXT_LOW_PW_MODE,
#ifdef SDIO
	EXT_SDIO_RX_AGGR,
#endif
	EXT_PMIC,
	EXT_DISCONNECT_ON_SUSPEND,
	EXT_HS_MIMO_SWITCH,
	EXT_FIX_BCN_BUF,
	EXT_NAPI,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	EXT_DFS_OFFLOAD,
#endif
	EXT_DISABLE_REGD_BY_DRIVER,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	EXT_COUNTRY_IE_IGNORE,
	EXT_BEACON_HINTS,
#endif
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
	EXT_HOST_MLME,
#endif
#endif
	EXT_MAX_PARAM,
};

/** Module parameter data structure for MOAL */
typedef struct _moal_mod_para {
	t_u8 ext_flgs[DIV_ROUND_UP(EXT_MAX_PARAM, 8)];
	/* BIT24 ~ BIT32 reserved */
	t_u8 flag;
	char *fw_name;
	int fw_reload;
	char *mac_addr;
#ifdef MFG_CMD_SUPPORT
	int mfg_mode;
#endif /* MFG_CMD_SUPPORT */
	int drv_mode;
#ifdef STA_SUPPORT
	int max_sta_bss;
	char *sta_name;
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
	int max_uap_bss;
	char *uap_name;
	int uap_max_sta;
#endif /* UAP_SUPPORT */
#ifdef WIFI_DIRECT_SUPPORT
	int max_wfd_bss;
	char *wfd_name;
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
	int max_vir_bss;
#endif
#endif /* WIFI_DIRECT_SUPPORT */
	int auto_ds;
	int ps_mode;
	int p2a_scan;
	/** scan chan gap */
	int scan_chan_gap;
	int max_tx_buf;
#if defined(SDIO)
	int gpiopin;
#endif
#if defined(STA_SUPPORT)
	int cfg_11d;
#endif
#if defined(SDIO)
	int slew_rate;
#endif
	char *dpd_data_cfg;
	char *init_cfg;
	char *cal_data_cfg;
	char *txpwrlimit_cfg;
	char *init_hostcmd_cfg;
	char *band_steer_cfg;
	int cfg80211_wext;
	int wq_sched_prio;
	int wq_sched_policy;
	int rx_work;
#ifdef USB
	int usb_aggr;
#endif
#ifdef PCIE
	int pcie_int_mode;
#endif /* PCIE */
#ifdef ANDROID_KERNEL
	int wakelock_timeout;
#endif
	unsigned int dev_cap_mask;
#if defined(SD8997) || defined(PCIE8997) || defined(USB8997) ||                \
	defined(SD8977) || defined(SD8987) || defined(SD9098) ||               \
	defined(USB9098) || defined(PCIE9098) || defined(SD9097) ||            \
	defined(USB9097) || defined(PCIE9097) || defined(SD8978) ||            \
	defined(USB8978)
	int pmic;
#endif
	int antcfg;
	unsigned int uap_oper_ctrl;
	int hs_wake_interval;
	int indication_gpio;
	int indrstcfg;
#ifdef WIFI_DIRECT_SUPPORT
	int GoAgeoutTime;
#endif
	int gtk_rekey_offload;
	t_u16 multi_dtim;
	t_u16 inact_tmo;
	char *reg_alpha2;
	int dfs53cfg;
} moal_mod_para;

/** Handle data structure for MOAL */
struct _moal_handle {
	/** MLAN adapter structure */
	t_void *pmlan_adapter;
	/** Private pointer */
	moal_private *priv[MLAN_MAX_BSS_NUM];
	/** Priv number */
	t_u8 priv_num;
	/** Bss attr */
	moal_drv_mode drv_mode;

	/** set mac address flag */
	t_u8 set_mac_addr;
	/** MAC address */
	t_u8 mac_addr[ETH_ALEN];
#ifdef CONFIG_PROC_FS
	/** Proc top level directory entry */
	struct proc_dir_entry *proc_wlan;
	/** Proc top level directory entry name */
	char proc_wlan_name[32];
#endif
#ifdef USB
	/** Firmware download skip flag */
	t_u8 skip_fw_dnld;
#endif /* USB */
	/** Firmware */
	const struct firmware *firmware;
	/** Firmware request start time */
	struct timeval req_fw_time;
	/** Init config file */
	const struct firmware *init_cfg_data;
	/** Init config file */
	const struct firmware *user_data;
	/** Init user configure wait queue token */
	t_u16 init_user_conf_wait_flag;
	/** Init user configure file wait queue */
	wait_queue_head_t init_user_conf_wait_q __ATTRIB_ALIGN__;
	/** dpd config file */
	const struct firmware *dpd_data;
	/** txpwr data file */
	const struct firmware *txpwr_data;
	/** Hotplug device */
	struct device *hotplug_device;
	/** STATUS variables */
	MOAL_HARDWARE_STATUS hardware_status;
	BOOLEAN fw_reload;
	/** POWER MANAGEMENT AND PnP SUPPORT */
	BOOLEAN surprise_removed;
	/** Firmware release number */
	t_u32 fw_release_number;
	/** ECSA support */
	t_u8 fw_ecsa_enable;
	/** Getlog support */
	t_u8 fw_getlog_enable;
	/** Init wait queue token */
	t_u16 init_wait_q_woken;
	/** Init wait queue */
	wait_queue_head_t init_wait_q __ATTRIB_ALIGN__;
	/** Device suspend flag */
	BOOLEAN is_suspended;
#ifdef SDIO_SUSPEND_RESUME
	/** suspend notify flag */
	BOOLEAN suspend_notify_req;
	/** hs_shutdown in process flag  */
	BOOLEAN shutdown_hs_in_process;
#endif
	/** Suspend wait queue token */
	t_u16 suspend_wait_q_woken;
	/** Suspend wait queue */
	wait_queue_head_t suspend_wait_q __ATTRIB_ALIGN__;
	/** Host Sleep activated flag */
	t_u8 hs_activated;
	/** Host Sleep activated event wait queue token */
	t_u16 hs_activate_wait_q_woken;
	/** Host Sleep activated event wait queue */
	wait_queue_head_t hs_activate_wait_q __ATTRIB_ALIGN__;
	/** auto_arp and ipv6 offload enable/disable flag */
	t_u8 hs_auto_arp;
	/** Card pointer */
	t_void *card;
	/** Rx pending in MLAN */
	atomic_t rx_pending;
	/** Tx packet pending count in mlan */
	atomic_t tx_pending;
	/** IOCTL pending count in mlan */
	atomic_t ioctl_pending;
	/** lock count */
	atomic_t lock_count;
	/** Malloc count */
	atomic_t malloc_count;
	/** vmalloc count */
	atomic_t vmalloc_count;
	/** mlan buffer alloc count */
	atomic_t mbufalloc_count;
#ifdef PCIE
	/** Malloc consistent count */
	atomic_t malloc_cons_count;
#endif
	/** hs skip count */
	t_u32 hs_skip_count;
	/** hs force count */
	t_u32 hs_force_count;
	/** suspend_fail flag */
	BOOLEAN suspend_fail;
#ifdef REASSOCIATION
	/** Re-association thread */
	moal_thread reassoc_thread;
	/** Re-association timer set flag */
	BOOLEAN is_reassoc_timer_set;
	/** Re-association timer */
	moal_drv_timer reassoc_timer __ATTRIB_ALIGN__;
	/**  */
	struct semaphore reassoc_sem;
	/** Bitmap for re-association on/off */
	t_u8 reassoc_on;
#endif /* REASSOCIATION */
	/** Driver workqueue */
	struct workqueue_struct *workqueue;
	/** main work */
	struct work_struct main_work;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	/** host_mlme_priv */
	moal_private *host_mlme_priv;
	/** Host Mlme Work struct**/
	struct work_struct host_mlme_work;
#endif
	/** Driver workqueue */
	struct workqueue_struct *rx_workqueue;
	/** main work */
	struct work_struct rx_work;
	/** Driver event workqueue */
	struct workqueue_struct *evt_workqueue;
	/** event  work */
	struct work_struct evt_work;
	/** event spin lock */
	spinlock_t evt_lock;
	/** event queue */
	struct list_head evt_queue;
	/** remain on channel flag */
	t_u8 remain_on_channel;
	/** bss index for remain on channel */
	t_u8 remain_bss_index;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	struct wiphy *wiphy;
	/** Country code for regulatory domain */
	t_u8 country_code[COUNTRY_CODE_LEN];
	/** band */
	enum ieee80211_band band;
	/** first scan done flag */
	t_u8 first_scan_done;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	/** remain_on_channel timer set flag */
	BOOLEAN is_remain_timer_set;
	/** remani_on_channel_timer */
	moal_drv_timer remain_timer __ATTRIB_ALIGN__;
	/** ieee802_11_channel */
	struct ieee80211_channel chan;
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
	/** channel type */
	enum nl80211_channel_type channel_type;
#endif
	/** cookie */
	t_u64 cookie;
#endif

#ifdef WIFI_DIRECT_SUPPORT
	/** NoA duration */
	t_u32 noa_duration;
	/** NoA interval */
	t_u32 noa_interval;
	/** miracast mode */
	t_u8 miracast_mode;
	/** scan time in miracast mode */
	t_u16 miracast_scan_time;
	/** GO timer set flag */
	BOOLEAN is_go_timer_set;
	/** GO timer */
	moal_drv_timer go_timer __ATTRIB_ALIGN__;
#endif
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	/** cfg80211_suspend status */
	t_u8 cfg80211_suspend;
#endif
#endif
	/** FW debug flag */
	t_u8 fw_dbg;
	/** reg debug flag */
	t_u8 reg_dbg;
#ifdef SDIO
#endif /* SDIO */
	/** Netlink kernel socket */
	struct sock *nl_sk;
	/** Netlink kernel socket number */
	t_u32 netlink_num;
	/** w_stats wait queue token */
	BOOLEAN meas_wait_q_woken;
	/** w_stats wait queue */
	wait_queue_head_t meas_wait_q __ATTRIB_ALIGN__;
	/** Measurement start jiffes */
	long meas_start_jiffies;
	/** CAC checking period flag */
	BOOLEAN cac_period;
	/** CAC timer jiffes */
	long cac_timer_jiffies;
	/** User NOP Period in sec */
	t_u16 usr_nop_period_sec;
	/** BSS START command delay executing flag */
	BOOLEAN delay_bss_start;
	/** SSID,BSSID parameter of delay executing */
	mlan_ssid_bssid delay_ssid_bssid;
#ifdef UAP_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	/* CAC channel info */
	struct cfg80211_chan_def dfs_channel;
	/* time set flag*/
	BOOLEAN is_cac_timer_set;
	/** cac_timer */
	moal_drv_timer cac_timer __ATTRIB_ALIGN__;
	/** cac bss index */
	t_u8 cac_bss_index;
#endif
#endif
#if defined(UAP_SUPPORT)
	/** channel switch wait queue token */
	BOOLEAN chsw_wait_q_woken;
	/** channel switch wait queue */
	wait_queue_head_t chsw_wait_q __ATTRIB_ALIGN__;
#endif
	/** cac period length, valid only when dfs testing is enabled */
	long cac_period_jiffies;
	/** handle index - for multiple card supports */
	t_u8 handle_idx;
#if defined(USB)
	/** Flag to indicate boot state */
	t_u8 boot_state;
#endif /* USB_NEW_FW_DNLD */
#ifdef SDIO_MMC_DEBUG
	/** cmd53 write state */
	u8 cmd53w;
	/** cmd53 read state */
	u8 cmd53r;
#endif
#ifdef STA_SUPPORT
	/** Scan pending on blocked flag */
	t_u8 scan_pending_on_block;
	/** Scan Private pointer */
	moal_private *scan_priv;
	/** Async scan semaphore */
	struct semaphore async_sem;
	/** scan channel gap */
	t_u16 scan_chan_gap;
#ifdef STA_CFG80211
	/** CFG80211 scan request description */
	struct cfg80211_scan_request *scan_request;
#endif
#endif
	/** main state */
	t_u8 main_state;
	/** driver status */
	t_u8 driver_status;
	/** driver state */
	t_u8 driver_state;
	/** ioctl timeout */
	t_u8 ioctl_timeout;
	/** FW dump state */
	t_u8 fw_dump;
	/** FW dump full name */
	t_u8 firmware_dump_file[128];
#ifdef SDIO
	/** cmd52 function */
	t_u8 cmd52_func;
	/** cmd52 register */
	t_u8 cmd52_reg;
	/** cmd52 value */
	t_u8 cmd52_val;
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	/** spinlock to stop_queue/wake_queue*/
	spinlock_t queue_lock;
#endif
	/** Driver spin lock */
	spinlock_t driver_lock;
	/** Driver ioctl spin lock */
	spinlock_t ioctl_lock;
	/** lock for scan_request */
	spinlock_t scan_req_lock;
	/** Interface type: 1 byte,  Card type: 1 byte */
	t_u16 card_type;
	/** card revsion */
	t_u8 card_rev;
	/** card info */
	card_info *card_info;
	/** Card specific driver version */
	t_s8 driver_version[MLAN_MAX_VER_STR_LEN];
	char *fwdump_fname;
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	struct wakeup_source ws;
#else
	struct wake_lock wake_lock;
#endif
#endif
	t_u16 dfs_repeater_mode;
	/* feature_control */
	t_u32 feature_control;
	struct notifier_block woal_notifier;
	mlan_ds_misc_keep_alive keep_alive[MAX_KEEP_ALIVE_ID];
	struct net_device napi_dev;
	struct napi_struct napi_rx;
	/* bus interface operations */
	moal_if_ops ops;
	/* module parameter data */
	const struct firmware *param_data;
	/* wrapped module parameters */
	moal_mod_para params;
	/* debug info */
	mlan_debug_info debug_info;
	/* block id in module param config file */
	int blk_id;
	/** time when FW is active, time is get from boot time, in Nanosecond */
	t_u64 on_time;
	/** tx time, in usecs */
	t_u64 tx_time;
	/** systime when tx start */
	wifi_timeval tx_time_start;
	/** systime when tx end */
	wifi_timeval tx_time_end;
	/** rx time, in usecs */
	t_u64 rx_time;
	/** scan time, in usecs */
	t_u64 scan_time;
	/** systime when scan cmd response return success */
	wifi_timeval scan_time_start;
	/** systime when scan event has no more event */
	wifi_timeval scan_time_end;
	/** seecond mac flag */
	t_u8 second_mac;
	/** moal handle for another mac */
	void *pref_mac;
	/** RF test mode status */
	t_u8 rf_test_mode;
	/** pointer to rf test mode data struct */
	struct rf_test_mode_data *rf_data;
};

/**
 *  @brief set extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			N/A
 */
static inline void moal_extflg_set(moal_handle *handle, enum ext_mod_params idx)
{
	t_u8 *ext_fbyte;
	ext_fbyte = &handle->params.ext_flgs[idx / 8];
	*ext_fbyte |= MBIT(idx % 8);
}

/**
 *  @brief clear extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			N/A
 */
static inline void moal_extflg_clear(moal_handle *handle,
				     enum ext_mod_params idx)
{
	t_u8 *ext_fbyte;
	ext_fbyte = &handle->params.ext_flgs[idx / 8];
	*ext_fbyte &= ~MBIT(idx % 8);
}

/**
 *  @brief check value of extended flag in bitmap
 *
 *  @param dev		A pointer to moal_handle structure
 *  @param idx      extended flags id
 *  @return			value of extended flag
 */
static inline t_u8 moal_extflg_isset(moal_handle *handle,
				     enum ext_mod_params idx)
{
	t_u8 ext_fbyte;
	ext_fbyte = handle->params.ext_flgs[idx / 8];
	return (ext_fbyte & MBIT(idx % 8)) != 0;
}

/**
 *  @brief set trans_start for each TX queue.
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_set_trans_start(struct net_device *dev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
	unsigned int i;
	for (i = 0; i < dev->num_tx_queues; i++)
		netdev_get_tx_queue(dev, i)->trans_start = jiffies;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
	dev->trans_start = jiffies;
#else
	netif_trans_update(dev);
#endif
}

/**
 *  @brief Start queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_start_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 29)
	netif_start_queue(dev);
#else
	if (dev->reg_state == NETREG_REGISTERED)
		netif_tx_wake_all_queues(dev);
	else
		netif_tx_start_all_queues(dev);
#endif
}

/**
 *  @brief Stop queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_stop_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	unsigned long flags;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	spin_lock_irqsave(&priv->phandle->queue_lock, flags);
	woal_set_trans_start(dev);
	if (!netif_queue_stopped(dev))
		netif_tx_stop_all_queues(dev);
	spin_unlock_irqrestore(&priv->phandle->queue_lock, flags);
#else
	woal_set_trans_start(dev);
	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);
#endif
}

/**
 *  @brief wake queue
 *
 *  @param dev		A pointer to net_device structure
 *
 *  @return			N/A
 */
static inline void woal_wake_queue(struct net_device *dev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	unsigned long flags;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	spin_lock_irqsave(&priv->phandle->queue_lock, flags);
	if (netif_queue_stopped(dev))
		netif_tx_wake_all_queues(dev);
	spin_unlock_irqrestore(&priv->phandle->queue_lock, flags);
#else
	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
#endif
}

/** Debug Macro definition*/
#ifdef DEBUG_LEVEL1
extern t_u32 drvdbg;

#define LOG_CTRL(level) (0)

#ifdef DEBUG_LEVEL2
#define PRINTM_MINFO(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MINFO)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MWARN(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MWARN)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MENTRY(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MENTRY)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#else
#define PRINTM_MINFO(level, msg...)                                            \
	do {                                                                   \
	} while (0)
#define PRINTM_MWARN(level, msg...)                                            \
	do {                                                                   \
	} while (0)
#define PRINTM_MENTRY(level, msg...)                                           \
	do {                                                                   \
	} while (0)
#endif /* DEBUG_LEVEL2 */

#define PRINTM_MFW_D(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MFW_D)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MCMD_D(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MCMD_D)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MDAT_D(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MDAT_D)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MIF_D(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MIF_D)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)

#define PRINTM_MIOCTL(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MIOCTL)                                           \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MINTR(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MINTR)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MEVENT(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MEVENT)                                           \
			printk(msg);                                           \
	} while (0)
#define PRINTM_MCMND(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MCMND)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MDATA(level, msg...)                                            \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MDATA)                                            \
			printk(KERN_DEBUG msg);                                \
	} while (0)
#define PRINTM_MERROR(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MERROR)                                           \
			printk(KERN_ERR msg);                                  \
	} while (0)
#define PRINTM_MFATAL(level, msg...)                                           \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MFATAL)                                           \
			printk(KERN_ERR msg);                                  \
	} while (0)
#define PRINTM_MMSG(level, msg...)                                             \
	do {                                                                   \
		woal_print(level, msg);                                        \
		if (drvdbg & MMSG)                                             \
			printk(KERN_ALERT msg);                                \
	} while (0)

static inline void woal_print(t_u32 level, char *fmt, ...)
{
}

#define PRINTM(level, msg...) PRINTM_##level(level, msg)

#else

#define PRINTM(level, msg...)                                                  \
	do {                                                                   \
	} while (0)

#endif /* DEBUG_LEVEL1 */

/** Wait until a condition becomes true */
#define MASSERT(cond)                                                          \
	do {                                                                   \
		if (!(cond)) {                                                 \
			PRINTM(MFATAL, "ASSERT: %s: %i\n", __func__,           \
			       __LINE__);                                      \
			panic("Assert failed: Panic!");                        \
		}                                                              \
	} while (0)

/** Log entry point for debugging */
#define ENTER() PRINTM(MENTRY, "Enter: %s\n", __func__)
/** Log exit point for debugging */
#define LEAVE() PRINTM(MENTRY, "Leave: %s\n", __func__)

#ifdef DEBUG_LEVEL1
#define DBG_DUMP_BUF_LEN 64
#define MAX_DUMP_PER_LINE 16

static inline void hexdump(t_u32 level, char *prompt, t_u8 *buf, int len)
{
	int i;
	char dbgdumpbuf[DBG_DUMP_BUF_LEN];
	char *ptr = dbgdumpbuf;

	if (drvdbg & level)
		printk(KERN_DEBUG "%s:\n", prompt);
	for (i = 1; i <= len; i++) {
		ptr += snprintf(ptr, 4, "%02x ", *buf);
		buf++;
		if (i % MAX_DUMP_PER_LINE == 0) {
			*ptr = 0;
			if (drvdbg & level)
				printk(KERN_DEBUG "%s\n", dbgdumpbuf);
			ptr = dbgdumpbuf;
		}
	}
	if (len % MAX_DUMP_PER_LINE) {
		*ptr = 0;
		if (drvdbg & level)
			printk(KERN_DEBUG "%s\n", dbgdumpbuf);
	}
}

#define DBG_HEXDUMP_MERROR(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MERROR) || LOG_CTRL(MERROR))                     \
			hexdump(MERROR, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MCMD_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MCMD_D) || LOG_CTRL(MCMD_D))                     \
			hexdump(MCMD_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MDAT_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MDAT_D) || LOG_CTRL(MDAT_D))                     \
			hexdump(MDAT_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MIF_D(x, y, z)                                             \
	do {                                                                   \
		if ((drvdbg & MIF_D) || LOG_CTRL(MIF_D))                       \
			hexdump(MIF_D, x, y, z);                               \
	} while (0)
#define DBG_HEXDUMP_MEVT_D(x, y, z)                                            \
	do {                                                                   \
		if ((drvdbg & MEVT_D) || LOG_CTRL(MEVT_D))                     \
			hexdump(MEVT_D, x, y, z);                              \
	} while (0)
#define DBG_HEXDUMP_MFW_D(x, y, z)                                             \
	do {                                                                   \
		if ((drvdbg & MFW_D) || LOG_CTRL(MFW_D))                       \
			hexdump(MFW_D, x, y, z);                               \
	} while (0)
#define DBG_HEXDUMP(level, x, y, z) DBG_HEXDUMP_##level(x, y, z)

#else
/** Do nothing since debugging is not turned on */
#define DBG_HEXDUMP(level, x, y, z)                                            \
	do {                                                                   \
	} while (0)
#endif

#ifdef DEBUG_LEVEL2
#define HEXDUMP(x, y, z)                                                       \
	do {                                                                   \
		if ((drvdbg & MINFO) || LOG_CTRL(MINFO))                       \
			hexdump(MINFO, x, y, z);                               \
	} while (0)
#else
/** Do nothing since debugging is not turned on */
#define HEXDUMP(x, y, z)                                                       \
	do {                                                                   \
	} while (0)
#endif

#ifdef BIG_ENDIAN_SUPPORT
/** Convert from 16 bit little endian format to CPU format */
#define woal_le16_to_cpu(x) le16_to_cpu(x)
/** Convert from 32 bit little endian format to CPU format */
#define woal_le32_to_cpu(x) le32_to_cpu(x)
/** Convert from 64 bit little endian format to CPU format */
#define woal_le64_to_cpu(x) le64_to_cpu(x)
/** Convert to 16 bit little endian format from CPU format */
#define woal_cpu_to_le16(x) cpu_to_le16(x)
/** Convert to 32 bit little endian format from CPU format */
#define woal_cpu_to_le32(x) cpu_to_le32(x)
/** Convert to 64 bit little endian format from CPU format */
#define woal_cpu_to_le64(x) cpu_to_le64(x)
#else
/** Do nothing */
#define woal_le16_to_cpu(x) x
/** Do nothing */
#define woal_le32_to_cpu(x) x
/** Do nothing */
#define woal_le64_to_cpu(x) x
/** Do nothing */
#define woal_cpu_to_le16(x) x
/** Do nothing */
#define woal_cpu_to_le32(x) x
/** Do nothing */
#define woal_cpu_to_le64(x) x
#endif

/**
 *  @brief This function returns first available priv
 *  based on the BSS role
 *
 *  @param handle    A pointer to moal_handle
 *  @param bss_role  BSS role or MLAN_BSS_ROLE_ANY
 *
 *  @return          Pointer to moal_private
 */
static inline moal_private *woal_get_priv(moal_handle *handle,
					  mlan_bss_role bss_role)
{
	int i;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (bss_role == MLAN_BSS_ROLE_ANY ||
			    GET_BSS_ROLE(handle->priv[i]) == bss_role)
				return handle->priv[i];
		}
	}
	return NULL;
}

/**
 *  @brief This function returns first available priv
 *  based on the BSS type
 *
 *  @param handle    A pointer to moal_handle
 *  @param bss_type  BSS type or MLAN_BSS_TYPE_ANY
 *
 *  @return          Pointer to moal_private
 */
static inline moal_private *woal_get_priv_bss_type(moal_handle *handle,
						   mlan_bss_type bss_type)
{
	int i;

	for (i = 0; i < MIN(handle->priv_num, MLAN_MAX_BSS_NUM); i++) {
		if (handle->priv[i]) {
			if (bss_type == MLAN_BSS_TYPE_ANY ||
			    handle->priv[i]->bss_type == bss_type)
				return handle->priv[i];
		}
	}
	return NULL;
}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#endif

static inline void woal_get_monotonic_time(struct timeval *tv)
{
	struct timespec ts;

	getrawmonotonic(&ts);
	if (tv) {
		tv->tv_sec = ts.tv_sec;
		tv->tv_usec = ts.tv_nsec / 1000;
	}
}

/* CAC Measure report default time 60 seconds */
#define MEAS_REPORT_TIME (60 * HZ)

/** Max line length allowed in init config file */
#define MAX_LINE_LEN 256
/** Max MAC address string length allowed */
#define MAX_MAC_ADDR_LEN 18
/** Max register type/offset/value etc. parameter length allowed */
#define MAX_PARAM_LEN 12

/** HostCmd_CMD_CFG_DATA for CAL data */
#define HostCmd_CMD_CFG_DATA 0x008f
/** HostCmd action set */
#define HostCmd_ACT_GEN_SET 0x0001
/** HostCmd CAL data header length */
#define CFG_DATA_HEADER_LEN 6

typedef struct _HostCmd_DS_GEN {
	t_u16 command;
	t_u16 size;
	t_u16 seq_num;
	t_u16 result;
} HostCmd_DS_GEN;

typedef struct _HostCmd_DS_802_11_CFG_DATA {
	/** Action */
	t_u16 action;
	/** Type */
	t_u16 type;
	/** Data length */
	t_u16 data_len;
	/** Data */
	t_u8 data[1];
} __ATTRIB_PACK__ HostCmd_DS_802_11_CFG_DATA;

/** combo scan header */
#define WEXT_CSCAN_HEADER "CSCAN S\x01\x00\x00S\x00"
/** combo scan header size */
#define WEXT_CSCAN_HEADER_SIZE 12
/** combo scan ssid section */
#define WEXT_CSCAN_SSID_SECTION 'S'
/** commbo scan channel section */
#define WEXT_CSCAN_CHANNEL_SECTION 'C'
/** commbo scan passive dwell section */
#define WEXT_CSCAN_PASV_DWELL_SECTION 'P'
/** commbo scan home dwell section */
#define WEXT_CSCAN_HOME_DWELL_SECTION 'H'
/** BGSCAN RSSI section */
#define WEXT_BGSCAN_RSSI_SECTION 'R'
/** BGSCAN SCAN INTERVAL SECTION */
#define WEXT_BGSCAN_INTERVAL_SECTION 'T'
/** BGSCAN REPEAT SECTION */
#define WEXT_BGSCAN_REPEAT_SECTION 'E'
/** Min BGSCAN interval 30 second */
#define MIN_BGSCAN_INTERVAL 30000
/** default repeat count */
#define DEF_REPEAT_COUNT 6

/** default rssi low threshold */
#define DEFAULT_RSSI_LOW_THRESHOLD 70
/** RSSI HYSTERSIS */
#define RSSI_HYSTERESIS 6
/** lowest rssi threshold */
#define LOWEST_RSSI_THRESHOLD 82
/** delta rssi */
#define DELTA_RSSI 10

/** NL80211 scan configuration header */
#define NL80211_SCANCFG_HEADER "SCAN-CFG "
/** NL80211 scan configuration header length */
#define NL80211_SCANCFG_HEADER_SIZE 9
/** NL80211 scan configuration active scan section */
#define NL80211_SCANCFG_ACTV_DWELL_SECTION 'A'
/** NL80211 scan configuration passive scan section */
#define NL80211_SCANCFG_PASV_DWELL_SECTION 'P'
/** NL80211 scan configuration specific scan section */
#define NL80211_SCANCFG_SPCF_DWELL_SECTION 'S'

/** band AUTO */
#define WIFI_FREQUENCY_BAND_AUTO 0
/** band 5G */
#define WIFI_FREQUENCY_BAND_5GHZ 1
/** band 2G */
#define WIFI_FREQUENCY_BAND_2GHZ 2
/** All band */
#define WIFI_FREQUENCY_ALL_BAND 3

/** Rx filter: IPV4 multicast */
#define RX_FILTER_IPV4_MULTICAST 1
/** Rx filter: broadcast */
#define RX_FILTER_BROADCAST 2
/** Rx filter: unicast */
#define RX_FILTER_UNICAST 4
/** Rx filter: IPV6 multicast */
#define RX_FILTER_IPV6_MULTICAST 8

/**  Convert ASCII string to hex value */
int woal_ascii2hex(t_u8 *d, char *s, t_u32 dlen);
/** parse ie */
const t_u8 *woal_parse_ie_tlv(const t_u8 *ie, int len, t_u8 id);
/** parse extension ie */
const t_u8 *woal_parse_ext_ie_tlv(const t_u8 *ie, int len, t_u8 ext_id);
/**  Convert mac address from string to t_u8 buffer */
void woal_mac2u8(t_u8 *mac_addr, char *buf);
/**  Extract token from string */
char *woal_strsep(char **s, char delim, char esc);
/** Return int value of a given ASCII string */
mlan_status woal_atoi(int *data, char *a);
/** Return hex value of a given ASCII string */
int woal_atox(char *a);
/** Allocate buffer */
pmlan_buffer woal_alloc_mlan_buffer(moal_handle *handle, int size);
/** Allocate IOCTL request buffer */
pmlan_ioctl_req woal_alloc_mlan_ioctl_req(int size);
/** Free buffer */
void woal_free_mlan_buffer(moal_handle *handle, pmlan_buffer pmbuf);
/** Get private structure of a BSS by index */
moal_private *woal_bss_index_to_priv(moal_handle *handle, t_u8 bss_index);
/* Functions in init module */
/** init module parameters */
mlan_status woal_init_module_param(moal_handle *handle);
/** free module parameters */
void woal_free_module_param(moal_handle *handle);
/** init module parameters from device tree */
void woal_init_from_dev_tree(void);
/** initializes software */
mlan_status woal_init_sw(moal_handle *handle);
/** update the default firmware name */
void woal_update_firmware_name(moal_handle *handle);
/** cancel all works in the queue */
void woal_terminate_workqueue(moal_handle *handle);
/** initializes firmware */
mlan_status woal_init_fw(moal_handle *handle);
/** frees the structure of moal_handle */
void woal_free_moal_handle(moal_handle *handle);
/** shutdown fw */
mlan_status woal_shutdown_fw(moal_private *priv, t_u8 wait_option);
/* Functions in interface module */
#ifdef ANDROID_KERNEL
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static inline void wakeup_source_init(struct wakeup_source *ws,
				      const char *name)
{
	ENTER();

	if (ws) {
		memset(ws, 0, sizeof(*ws));
		ws->name = name;
	}
	wakeup_source_add(ws);

	LEAVE();
}

static inline void wakeup_source_trash(struct wakeup_source *ws)
{
	ENTER();

	if (!ws) {
		PRINTM(MERROR, "ws is null!\n");
		return;
	}
	wakeup_source_remove(ws);
	__pm_relax(ws);

	LEAVE();
}
#endif
#endif
/** Add card */
moal_handle *woal_add_card(void *card, struct device *dev, moal_if_ops *if_ops,
			   t_u16 card_type);
/** Remove card */
mlan_status woal_remove_card(void *card);
/** broadcast event */
mlan_status woal_broadcast_event(moal_private *priv, t_u8 *payload, t_u32 len);
#ifdef CONFIG_PROC_FS
/** switch driver mode */
mlan_status woal_switch_drv_mode(moal_handle *handle, t_u32 mode);
#endif

/** check if any interface is up */
t_u8 woal_is_any_interface_active(moal_handle *handle);
/** Get version */
void woal_get_version(moal_handle *handle, char *version, int maxlen);
/** Get Driver Version */
int woal_get_driver_version(moal_private *priv, struct ifreq *req);
/** Get extended driver version */
int woal_get_driver_verext(moal_private *priv, struct ifreq *ireq);
/** check driver status */
t_u8 woal_check_driver_status(moal_handle *handle);
/** Mgmt frame forward registration */
int woal_reg_rx_mgmt_ind(moal_private *priv, t_u16 action,
			 t_u32 *pmgmt_subtype_mask, t_u8 wait_option);
#ifdef DEBUG_LEVEL1
/** Set driver debug bit masks */
int woal_set_drvdbg(moal_private *priv, t_u32 drvdbg);
#endif

mlan_status woal_set_get_tx_bf_cap(moal_private *priv, t_u16 action,
				   t_u32 *tx_bf_cap);
/** Set/Get TX beamforming configurations */
mlan_status woal_set_get_tx_bf_cfg(moal_private *priv, t_u16 action,
				   mlan_ds_11n_tx_bf_cfg *bf_cfg);
/** Request MAC address setting */
mlan_status woal_request_set_mac_address(moal_private *priv, t_u8 wait_option);
/** Request multicast list setting */
void woal_request_set_multicast_list(moal_private *priv,
				     struct net_device *dev);
/** Request IOCTL action */
mlan_status woal_request_ioctl(moal_private *priv, mlan_ioctl_req *req,
			       t_u8 wait_option);
/** Set/Get generic element */
mlan_status woal_set_get_gen_ie(moal_private *priv, t_u32 action, t_u8 *ie,
				int *ie_len, t_u8 wait_option);
#ifdef CONFIG_PROC_FS
mlan_status woal_request_soft_reset(moal_handle *handle);
#endif
void woal_request_fw_reload(moal_handle *phandle, t_u8 mode);

/** Get debug information */
mlan_status woal_get_debug_info(moal_private *priv, t_u8 wait_option,
				mlan_debug_info *debug_info);
/** Set debug information */
mlan_status woal_set_debug_info(moal_private *priv, t_u8 wait_option,
				mlan_debug_info *debug_info);
/** Disconnect */
mlan_status woal_disconnect(moal_private *priv, t_u8 wait_option, t_u8 *mac,
			    t_u16 reason_code);
/** associate */
mlan_status woal_bss_start(moal_private *priv, t_u8 wait_option,
			   mlan_ssid_bssid *ssid_bssid);
/** Request firmware information */
mlan_status woal_request_get_fw_info(moal_private *priv, t_u8 wait_option,
				     mlan_fw_info *fw_info);
/** Get channel of active intf */
mlan_status woal_get_active_intf_channel(moal_private *priv,
					 chan_band_info *channel);
#ifdef STA_SUPPORT
/** Request Exented Capability information */
int woal_request_extcap(moal_private *priv, t_u8 *buf, t_u8 len);
#endif
mlan_status woal_set_get_dtim_period(moal_private *priv, t_u32 action,
				     t_u8 wait_option, t_u8 *value);
/** Set/get Host Sleep parameters */
mlan_status woal_set_get_hs_params(moal_private *priv, t_u16 action,
				   t_u8 wait_option, mlan_ds_hs_cfg *hscfg);
/** Cancel Host Sleep configuration */
mlan_status woal_cancel_hs(moal_private *priv, t_u8 wait_option);
/** Enable Host Sleep configuration */
int woal_enable_hs(moal_private *priv);
/** hs active timeout 2 second */
#define HS_ACTIVE_TIMEOUT (2 * HZ)
/** Get wakeup reason */
mlan_status woal_get_wakeup_reason(moal_private *priv,
				   mlan_ds_hs_wakeup_reason *wakeup_reason);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
void woal_create_dump_dir(moal_handle *phandle, char *dir_buf, int buf_size);
#endif
mlan_status woal_save_dump_info_to_file(char *dir_name, char *file_name,
					t_u8 *buf, t_u32 buf_len);
void woal_dump_drv_info(moal_handle *phandle, t_u8 *dir_name);

#define FW_DUMP_TYPE_ENDED 0x002
#define FW_DUMP_TYPE_MEM_ITCM 0x004
#define FW_DUMP_TYPE_MEM_DTCM 0x005
#define FW_DUMP_TYPE_MEM_SQRAM 0x006
#define FW_DUMP_TYPE_MEM_IRAM 0x007
#define FW_DUMP_TYPE_REG_MAC 0x009
#define FW_DUMP_TYPE_REG_CIU 0x00E
#define FW_DUMP_TYPE_REG_APU 0x00F
#define FW_DUMP_TYPE_REG_ICU 0x014
#ifdef SDIO_MMC
void woal_dump_firmware_info(moal_handle *phandle);
void woal_dump_firmware_info_v2(moal_handle *phandle);
void woal_dump_firmware_info_v3(moal_handle *phandle);
#endif /* SDIO_MMC */
/* Store the FW dumps received from events in a file */
void woal_store_firmware_dump(moal_handle *phandle, pmlan_event pmevent);

#if defined(PCIE)
void woal_store_ssu_dump(moal_handle *phandle, pmlan_event pmevent);
#endif /* SSU_SUPPORT */

int woal_pre_warmreset(moal_private *priv);
int woal_warmreset(moal_private *priv);

/** get deep sleep */
int woal_get_deep_sleep(moal_private *priv, t_u32 *data);
/** set deep sleep */
int woal_set_deep_sleep(moal_private *priv, t_u8 wait_option,
			BOOLEAN bdeep_sleep, t_u16 idletime);
/** process hang */
void woal_process_hang(moal_handle *handle);
/** Get BSS information */
mlan_status woal_get_bss_info(moal_private *priv, t_u8 wait_option,
			      mlan_bss_info *bss_info);
void woal_process_ioctl_resp(moal_private *priv, mlan_ioctl_req *req);
char *region_code_2_string(t_u8 region_code);
t_bool woal_is_etsi_country(t_u8 *country_code);
t_u8 woal_is_valid_alpha2(char *alpha2);
#ifdef STA_SUPPORT
void woal_send_disconnect_to_system(moal_private *priv, t_u16 reason_code);
void woal_send_mic_error_event(moal_private *priv, t_u32 event);
void woal_ioctl_get_bss_resp(moal_private *priv, mlan_ds_bss *bss);
void woal_ioctl_get_info_resp(moal_private *priv, mlan_ds_get_info *info);
mlan_status woal_get_assoc_rsp(moal_private *priv,
			       mlan_ds_misc_assoc_rsp *assoc_rsp,
			       t_u8 wait_option);
/** Get signal information */
mlan_status woal_get_signal_info(moal_private *priv, t_u8 wait_option,
				 mlan_ds_get_signal *signal);
/** Get mode */
t_u32 woal_get_mode(moal_private *priv, t_u8 wait_option);
mlan_status woal_get_sta_channel(moal_private *priv, t_u8 wait_option,
				 chan_band_info *channel);
#ifdef STA_WEXT
/** Get data rates */
mlan_status woal_get_data_rates(moal_private *priv, t_u8 wait_option,
				pmoal_802_11_rates m_rates);
void woal_send_iwevcustom_event(moal_private *priv, char *str);
/** Get channel list */
mlan_status woal_get_channel_list(moal_private *priv, t_u8 wait_option,
				  mlan_chan_list *chanlist);
mlan_status woal_11d_check_ap_channel(moal_private *priv, t_u8 wait_option,
				      mlan_ssid_bssid *ssid_bssid);
#endif
/** Set/Get retry count */
mlan_status woal_set_get_retry(moal_private *priv, t_u32 action,
			       t_u8 wait_option, int *value);
/** Set/Get RTS threshold */
mlan_status woal_set_get_rts(moal_private *priv, t_u32 action, t_u8 wait_option,
			     int *value);
/** Set/Get fragment threshold */
mlan_status woal_set_get_frag(moal_private *priv, t_u32 action,
			      t_u8 wait_option, int *value);
/** Set/Get TX power */
mlan_status woal_set_get_tx_power(moal_private *priv, t_u32 action,
				  mlan_power_cfg_t *pwr);
/** Set/Get power IEEE management */
mlan_status woal_set_get_power_mgmt(moal_private *priv, t_u32 action,
				    int *disabled, int type, t_u8 wait_option);
/** Get data rate */
mlan_status woal_set_get_data_rate(moal_private *priv, t_u8 action,
				   mlan_rate_cfg_t *datarate);
/** Request a network scan */
mlan_status woal_request_scan(moal_private *priv, t_u8 wait_option,
			      mlan_802_11_ssid *req_ssid);
/** Set radio on/off */
int woal_set_radio(moal_private *priv, t_u8 option);
/** Set region code */
mlan_status woal_set_region_code(moal_private *priv, char *region);
/** Set authentication mode */
mlan_status woal_set_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 auth_mode);
/** Set encryption mode */
mlan_status woal_set_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 encrypt_mode);
/** Enable wep key */
mlan_status woal_enable_wep_key(moal_private *priv, t_u8 wait_option);
/** Set WPA enable */
mlan_status woal_set_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 enable);
/** cancel scan command */
mlan_status woal_cancel_scan(moal_private *priv, t_u8 wait_option);
/** Find best network to connect */
mlan_status woal_find_best_network(moal_private *priv, t_u8 wait_option,
				   mlan_ssid_bssid *ssid_bssid);
/** Set Ad-Hoc channel */
mlan_status woal_change_adhoc_chan(moal_private *priv, int channel,
				   t_u8 wait_option);

/** Get scan table */
mlan_status woal_get_scan_table(moal_private *priv, t_u8 wait_option,
				mlan_scan_resp *scanresp);
/** Get authentication mode */
mlan_status woal_get_auth_mode(moal_private *priv, t_u8 wait_option,
			       t_u32 *auth_mode);
/** Get encryption mode */
mlan_status woal_get_encrypt_mode(moal_private *priv, t_u8 wait_option,
				  t_u32 *encrypt_mode);
/** Get WPA state */
mlan_status woal_get_wpa_enable(moal_private *priv, t_u8 wait_option,
				t_u32 *enable);
#endif /**STA_SUPPORT */

mlan_status woal_set_11d(moal_private *priv, t_u8 wait_option, t_u8 enable);

mlan_status woal_process_rf_test_mode(moal_handle *handle, t_u32 mode);
mlan_status woal_process_rf_test_mode_cmd(moal_handle *handle, t_u32 cmd,
					  const char *buffer, size_t len,
					  t_u32 action, t_u32 val);
#if defined(STA_SUPPORT) || defined(UAP_SUPPORT)
/** Get statistics information */
mlan_status woal_get_stats_info(moal_private *priv, t_u8 wait_option,
				pmlan_ds_get_stats stats);
#endif /**STA_SUPPORT||UAP_SUPPORT*/

mlan_status woal_set_wapi_enable(moal_private *priv, t_u8 wait_option,
				 t_u32 enable);

/** Initialize priv */
void woal_init_priv(moal_private *priv, t_u8 wait_option);
/** Reset interface(s) */
int woal_reset_intf(moal_private *priv, t_u8 wait_option, int all_intf);
#define TLV_TYPE_MGMT_IE (0x169)
#define MGMT_MASK_ASSOC_REQ 0x01
#define MGMT_MASK_REASSOC_REQ 0x04
#define MGMT_MASK_ASSOC_RESP 0x02
#define MGMT_MASK_REASSOC_RESP 0x08
#define MGMT_MASK_PROBE_REQ 0x10
#define MGMT_MASK_PROBE_RESP 0x20
#define MGMT_MASK_BEACON 0x100
#define MGMT_MASK_ASSOC_RESP_QOS_MAP 0x4000
#define MGMT_MASK_BEACON_WPS_P2P 0x8000
#define MLAN_CUSTOM_IE_DELETE_MASK 0x0
/** common ioctl for uap, station */
int woal_custom_ie_ioctl(struct net_device *dev, struct ifreq *req);
#ifdef UAP_SUPPORT
int woal_priv_get_nonglobal_operclass_by_bw_channel(moal_private *priv,
						    t_u8 bandwidth,
						    t_u8 channel,
						    t_u8 *oper_class);
#endif
int woal_send_host_packet(struct net_device *dev, struct ifreq *req);
/** Private command ID to pass mgmt frame */
#define WOAL_MGMT_FRAME_TX_IOCTL (SIOCDEVPRIVATE + 12)

int woal_get_bss_type(struct net_device *dev, struct ifreq *req);
#if defined(STA_WEXT) || defined(UAP_WEXT)
int woal_host_command(moal_private *priv, struct iwreq *wrq);
#endif
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
mlan_status woal_bss_role_cfg(moal_private *priv, t_u8 action, t_u8 wait_option,
			      t_u8 *bss_role);
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
void woal_go_timer_func(void *context);
#endif
#if defined(STA_WEXT) || defined(UAP_WEXT)
int woal_set_get_bss_role(moal_private *priv, struct iwreq *wrq);
#endif
#endif
#if defined(WIFI_DIRECT_SUPPORT) || defined(UAP_SUPPORT)
/** hostcmd ioctl for uap, wifidirect */
int woal_hostcmd_ioctl(struct net_device *dev, struct ifreq *req);
#endif

mlan_status woal_set_remain_channel_ioctl(moal_private *priv, t_u8 wait_option,
					  pmlan_ds_remain_chan pchan);
void woal_remain_timer_func(void *context);
#ifdef WIFI_DIRECT_SUPPORT
mlan_status woal_wifi_direct_mode_cfg(moal_private *priv, t_u16 action,
				      t_u16 *mode);
mlan_status woal_p2p_config(moal_private *priv, t_u32 action,
			    mlan_ds_wifi_direct_config *p2p_config);
#endif /* WIFI_DIRECT_SUPPORT */

int woal_11h_cancel_chan_report_ioctl(moal_private *priv, t_u8 wait_option);

#ifdef CONFIG_PROC_FS
/** Initialize /proc/mwlan */
mlan_status woal_root_proc_init(void);
/** Remove /proc/mwlan */
void woal_root_proc_remove(void);
/** Initialize proc fs */
void woal_proc_init(moal_handle *handle);
/** Clean up proc fs */
void woal_proc_exit(moal_handle *handle);
/** Create proc entry */
void woal_create_proc_entry(moal_private *priv);
/** Remove proc entry */
void woal_proc_remove(moal_private *priv);
/** string to number */
int woal_string_to_number(char *s);
#endif

/** Create debug proc fs */
void woal_debug_entry(moal_private *priv);
/** Remove debug proc fs */
void woal_debug_remove(moal_private *priv);

/** check pm info */
mlan_status woal_get_pm_info(moal_private *priv, mlan_ds_ps_info *pm_info);
/** get mlan debug info */
void woal_mlan_debug_info(moal_private *priv);

#ifdef USB
#ifdef CONFIG_USB_SUSPEND
/** Enter USB Suspend */
int woal_enter_usb_suspend(moal_handle *handle);
/** Exit from USB Suspend */
int woal_exit_usb_suspend(moal_handle *handle);
#endif /* CONFIG_USB_SUSPEND */
#endif

#ifdef REASSOCIATION
int woal_reassociation_thread(void *data);
void woal_reassoc_timer_func(void *context);
#endif /* REASSOCIATION */

t_void woal_main_work_queue(struct work_struct *work);
t_void woal_rx_work_queue(struct work_struct *work);
t_void woal_evt_work_queue(struct work_struct *work);

netdev_tx_t woal_hard_start_xmit(struct sk_buff *skb, struct net_device *dev);
#ifdef STA_SUPPORT
mlan_status woal_init_sta_dev(struct net_device *dev, moal_private *priv);
#endif
#ifdef UAP_SUPPORT
mlan_status woal_init_uap_dev(struct net_device *dev, moal_private *priv);
#endif
mlan_status woal_update_drv_tbl(moal_handle *handle, int drv_mode_local);
moal_private *woal_add_interface(moal_handle *handle, t_u8 bss_num,
				 t_u8 bss_type);
void woal_remove_interface(moal_handle *handle, t_u8 bss_index);
void woal_set_multicast_list(struct net_device *dev);
mlan_status woal_request_fw(moal_handle *handle);
int woal_11h_channel_check_ioctl(moal_private *priv, t_u8 wait_option);
void woal_cancel_cac_block(moal_private *priv);
void woal_moal_debug_info(moal_private *priv, moal_handle *handle, u8 flag);

#ifdef STA_SUPPORT
mlan_status woal_get_powermode(moal_private *priv, int *powermode);
mlan_status woal_set_scan_type(moal_private *priv, t_u32 scan_type);
mlan_status woal_get_scan_config(moal_private *priv, mlan_scan_cfg *scan_cfg);
mlan_status woal_enable_ext_scan(moal_private *priv, t_u8 enable);
mlan_status woal_set_powermode(moal_private *priv, char *powermode);
int woal_find_essid(moal_private *priv, mlan_ssid_bssid *ssid_bssid,
		    t_u8 wait_option);
mlan_status woal_find_bssid(moal_private *priv, mlan_802_11_mac_addr bssid);
mlan_status woal_request_userscan(moal_private *priv, t_u8 wait_option,
				  wlan_user_scan_cfg *scan_cfg);
mlan_status woal_do_scan(moal_private *priv, wlan_user_scan_cfg *scan_cfg);
int woal_set_combo_scan(moal_private *priv, char *buf, int length);
mlan_status woal_set_scan_time(moal_private *priv, t_u16 active_scan_time,
			       t_u16 passive_scan_time,
			       t_u16 specific_scan_time);
mlan_status woal_get_band(moal_private *priv, int *band);
mlan_status woal_set_band(moal_private *priv, char *pband);
mlan_status woal_add_rxfilter(moal_private *priv, char *rxfilter);
mlan_status woal_remove_rxfilter(moal_private *priv, char *rxfilter);
mlan_status woal_priv_qos_cfg(moal_private *priv, t_u32 action, char *qos_cfg);
mlan_status woal_set_sleeppd(moal_private *priv, char *psleeppd);
int woal_set_scan_cfg(moal_private *priv, char *buf, int length);
void woal_update_dscp_mapping(moal_private *priv);

/* EVENT: BCN_RSSI_LOW */
#define EVENT_BCN_RSSI_LOW 0x0001
/* EVENT: PRE_BCN_LOST */
#define EVENT_PRE_BCN_LOST 0x0002
mlan_status woal_set_rssi_low_threshold(moal_private *priv, char *rssi,
					t_u8 wait_option);
mlan_status woal_set_rssi_threshold(moal_private *priv, t_u32 event_id,
				    t_u8 wait_option);
/* EVENT: BG_SCAN_REPORT */
#define EVENT_BG_SCAN_REPORT 0x0004
mlan_status woal_set_bg_scan(moal_private *priv, char *buf, int length);
mlan_status woal_stop_bg_scan(moal_private *priv, t_u8 wait_option);
void woal_reconfig_bgscan(moal_handle *handle);
#ifdef STA_CFG80211
void woal_config_bgscan_and_rssi(moal_private *priv, t_u8 set_rssi);
void woal_start_roaming(moal_private *priv);
#endif
mlan_status woal_request_bgscan(moal_private *priv, t_u8 wait_option,
				wlan_bgscan_cfg *scan_cfg);
#endif
#ifdef STA_CFG80211
void woal_save_conn_params(moal_private *priv,
			   struct cfg80211_connect_params *sme);
void woal_clear_conn_params(moal_private *priv);
#endif

void woal_flush_tcp_sess_queue(moal_private *priv);
void wlan_scan_create_brief_table_entry(t_u8 **ppbuffer,
					BSSDescriptor_t *pbss_desc);
int wlan_get_scan_table_ret_entry(BSSDescriptor_t *pbss_desc, t_u8 **ppbuffer,
				  int *pspace_left);
BOOLEAN woal_ssid_valid(mlan_802_11_ssid *pssid);
int woal_is_connected(moal_private *priv, mlan_ssid_bssid *ssid_bssid);
int woal_priv_hostcmd(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen,
		      t_u8 wait_option);
void woal_flush_tx_stat_queue(moal_private *priv);
struct tx_status_info *woal_get_tx_info(moal_private *priv, t_u8 tx_seq_num);
void woal_remove_tx_info(moal_private *priv, t_u8 tx_seq_num);

mlan_status woal_request_country_power_table(moal_private *priv, char *region);
#ifdef RX_PACKET_COALESCE
mlan_status woal_rx_pkt_coalesce_cfg(moal_private *priv, t_u16 *enable,
				     t_u8 wait_option, t_u8 action);
#endif
mlan_status woal_set_low_pwr_mode(moal_handle *handle, t_u8 wait_option);
int woal_hexval(char chr);
mlan_status woal_pmic_configure(moal_handle *handle, t_u8 wait_option);
mlan_status woal_set_user_antcfg(moal_handle *handle, t_u8 wait_option);
void woal_hist_data_reset(moal_private *priv);
void woal_hist_do_reset(moal_private *priv, void *data);
void woal_hist_reset_table(moal_private *priv, t_u8 antenna);
void woal_hist_data_add(moal_private *priv, t_u16 rx_rate, t_s8 snr, t_s8 nflr,
			t_u8 antenna);
mlan_status woal_set_hotspotcfg(moal_private *priv, t_u8 wait_option,
				t_u32 hotspotcfg);
mlan_status woal_set_get_wowlan_config(moal_private *priv, t_u16 action,
				       t_u8 wait_option,
				       mlan_ds_misc_mef_flt_cfg *mefcfg);
mlan_status woal_set_auto_arp_ext(moal_handle *handle, t_u8 enable);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
mlan_status woal_do_flr(moal_handle *handle, bool prepare);
#endif
mlan_status woal_delba_all(moal_private *priv, t_u8 wait_option);
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
int woal_mkeep_alive_vendor_event(moal_private *priv,
				  pmlan_ds_misc_keep_alive mkeep_alive);
#endif
#endif
int woal_start_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id,
			   t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac,
			   t_u8 *dst_mac, t_u32 period_msec,
			   t_u32 retry_interval, t_u8 retry_cnt);
int woal_stop_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id, t_u8 reset,
			  t_u8 *ip_pkt, t_u8 *pkt_len);
int woal_priv_save_cloud_keep_alive_params(
	moal_private *priv, t_u8 mkeep_alive_id, t_u8 enable, t_u16 ether_type,
	t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac, t_u8 *dst_mac,
	t_u32 period_msec, t_u32 retry_interval, t_u8 retry_cnt);
mlan_status woal_init_aggr_ctrl(moal_handle *handle, t_u8 wait_option);

#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)
mlan_status woal_set_rekey_data(moal_private *priv,
				mlan_ds_misc_gtk_rekey_data *gtk_rekey,
				t_u8 action, t_u8 wait_option);
#endif
#endif

mlan_status woal_vdll_req_fw(moal_handle *handle);

void woal_ioctl_get_misc_conf(moal_private *priv, mlan_ds_misc_cfg *info);
t_u8 woal_get_second_channel_offset(int chan);
#endif /* _MOAL_MAIN_H */
