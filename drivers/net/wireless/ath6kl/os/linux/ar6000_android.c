/*
 *
 * Copyright (c) 2004-2010 Atheros Communications Inc.
 * All rights reserved.
 *
 * 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
 *
 */
#include "ar6000_drv.h"
#include "htc.h"
#include <linux/vmalloc.h>

#include <linux/fs.h>
#ifdef CONFIG_PM

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/wakelock.h>
#endif 
enum {
    WLAN_PWR_CTRL_UP = 0,
    WLAN_PWR_CTRL_CUT_PWR,
    WLAN_PWR_CTRL_DEEP_SLEEP,
    WLAN_PWR_CTRL_WOW
};
#include <linux/platform_device.h>
#include <linux/inetdevice.h>

#define IS_MAC_NULL(mac) (mac[0]==0 && mac[1]==0 && mac[2]==0 && mac[3]==0 && mac[4]==0 && mac[5]==0)
#define MAX_BUF (8*1024)

#ifdef DEBUG

#define  ATH_DEBUG_DBG_LOG       ATH_DEBUG_MAKE_MODULE_MASK(0)

static ATH_DEBUG_MASK_DESCRIPTION android_debug_desc[] = {
    { ATH_DEBUG_DBG_LOG      , "Android Debug Logs"},
};

ATH_DEBUG_INSTANTIATE_MODULE_VAR(android,
                                 "android",
                                 "Android Driver Interface",
                                 ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_DBG_LOG,
                                 ATH_DEBUG_DESCRIPTION_COUNT(android_debug_desc),
                                 android_debug_desc);
                                 
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
char fwpath[256] = "/lib/firmware/ath6k/AR6102";
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0) */
int buspm = WLAN_PWR_CTRL_CUT_PWR;
int wow2mode = WLAN_PWR_CTRL_CUT_PWR;
#define HAVE_WLAN_PWR_IMPL 0 
#if HAVE_WLAN_PWR_IMPL
/**   @brief Disable SDIO clock source and mask all interrupt  */
extern void plat_disable_wlan_slot(void);
/**   @brief Enable SDIO clock source and unmask all interrupt */
extern void plat_enable_wlan_slot(void);
#else
#define plat_disable_wlan_slot()
#define plat_enable_wlan_slot()
#endif 

#endif /* CONFIG_PM */
extern int bmienable;
extern int wlaninitmode;
extern unsigned int wmitimeout;
extern wait_queue_head_t arEvent;
extern struct net_device *ar6000_devices[];
#ifdef CONFIG_HOST_TCMD_SUPPORT
extern unsigned int testmode;
#endif
extern char ifname[];

const char def_ifname[] = "wlan0";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
module_param_string(fwpath, fwpath, sizeof(fwpath), 0644);
module_param(buspm, int, 0644);
#else
#define __user
/* for linux 2.4 and lower */
MODULE_PARM(buspm,"i");
#endif 

struct wake_lock ar6k_init_wake_lock;
struct wake_lock ar6k_wow_wake_lock;
static A_STATUS (*ar6000_avail_ev_p)(void *, void *);

extern int ar6000_init(struct net_device *dev);
extern A_STATUS ar6000_configure_target(AR_SOFTC_T *ar);
extern void ar6000_stop_endpoint(struct net_device *dev, A_BOOL keepprofile);
extern A_STATUS ar6000_sysfs_bmi_get_config(AR_SOFTC_T *ar, A_UINT32 mode);
extern void ar6000_destroy(struct net_device *dev, unsigned int unregister);

static void ar6000_enable_mmchost_detect_change(int enable);
static void ar6000_restart_endpoint(struct net_device *dev);

#if defined(CONFIG_PM)
static A_STATUS ar6000_suspend_ev(void *context);

static A_STATUS ar6000_resume_ev(void *context);
#endif

int android_request_firmware(const struct firmware **firmware_p, const char *name,
                     struct device *device)
{
    struct file *filp = (struct file *)-ENOENT;
    int ret = 0;
    mm_segment_t    oldfs;
    struct firmware *firmware;
	char filename[2048];
#if 0 
    const char *raw_filename = strrchr(name, '/');
#else
    const char *raw_filename = NULL;
#endif 
	*firmware_p = firmware = kzalloc(sizeof(*firmware), GFP_KERNEL);
    if (!firmware) 
		return -ENOMEM;
    if (raw_filename) 
        ++raw_filename;
    else
        raw_filename = name;
	sprintf(filename, "%s/%s", fwpath, raw_filename);
    // Open file
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    do {
        size_t length, bufsize, bmisize;
        struct inode    *inode;
        filp = filp_open(filename, O_RDONLY, S_IRUSR);
        if (IS_ERR(filp) || !filp->f_op) {
            printk("%s: file %s filp_open error\n", __FUNCTION__, filename);
            ret = -1;
            break;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
        inode = filp->f_path.dentry->d_inode;
#else
        inode = filp->f_dentry->d_inode;
#endif
		if (!inode) {
            printk("%s: Get inode from filp failed\n", __FUNCTION__);
            ret = -1;
            break;
        }
        length = i_size_read(inode->i_mapping->host);
	    bufsize = ALIGN(length, PAGE_SIZE);
        bmisize = A_ROUND_UP(length, 4);
        bufsize = max(bmisize, bufsize);
	    firmware->data = vmalloc(bufsize);
        firmware->size = bmisize;
        if (!firmware->data) {
            printk("Cannot allocate buffer for firmware\n");
            ret = -ENOMEM;
            break;
        }
        if (filp->f_op->read(filp, (char*)firmware->data, length, &filp->f_pos) != length) {
            printk("%s: file read error, remaining=%d\n", __FUNCTION__, length);
            ret = -1;
            break;
        }        
    } while (0);

    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
    set_fs(oldfs);

    if (ret!=0) {
		if (firmware) {
            if (firmware->data)
                vfree(firmware->data);
            kfree(firmware);
        }
        *firmware_p = NULL;
    }
    return ret;    
}

void android_release_firmware(const struct firmware *firmware)
{
	if (firmware) {
        if (firmware->data)
            vfree(firmware->data);
        kfree(firmware);
    }
}

#if defined(CONFIG_PM)
static void ar6k_send_asleep_event_to_app(AR_SOFTC_T *ar, A_BOOL asleep)
{
    char buf[128];
    union iwreq_data wrqu;

    snprintf(buf, sizeof(buf), "HOST_ASLEEP=%s", asleep ? "asleep" : "awake");
    A_MEMZERO(&wrqu, sizeof(wrqu));
    wrqu.data.length = strlen(buf);
    wireless_send_event(ar->arNetDev, IWEVCUSTOM, &wrqu, buf);
}

static void ar6000_wow_resume(AR_SOFTC_T *ar)
{
    if (ar->arWowState) {
        WMI_SET_HOST_SLEEP_MODE_CMD hostSleepMode = {TRUE, FALSE};
        ar->arWowState = 0;
        wmi_set_host_sleep_mode_cmd(ar->arWmi, &hostSleepMode);
        wmi_scanparams_cmd(ar->arWmi, 0,0,60,0,0,0,0,0,0,0);
        //wmi_set_keepalive_cmd(ar->arWmi, 0);

#if 1 /* we don't do it if the power consumption is already good enough. */
        if (wmi_listeninterval_cmd(ar->arWmi, ar->arListenInterval, 0) == A_OK) {
        }
#endif
        ar6k_send_asleep_event_to_app(ar, FALSE);
        if (ar->arTxPending[ar->arControlEp]) {
            long timeleft = wait_event_interruptible_timeout(arEvent,
            ar->arTxPending[ar->arControlEp] == 0, wmitimeout * HZ);
            if (!timeleft || signal_pending(current)) {
                printk("Failed to Resume Wow!!!!!!!!!!!!!!!!!!!!\n");
            } else {
                printk("Resume WoW successfully\n");
            }
        }
    } else {
        printk("WoW does not invoked. skip resume");
    }
}

static void ar6000_wow_suspend(AR_SOFTC_T *ar)
{
#define ANDROID_WOW_LIST_ID 1
    if (ar->arNetworkType != AP_NETWORK) {
        /* Setup WoW for unicast & Aarp request for our own IP
        disable background scan. Set listen interval into 1000 TUs
        Enable keepliave for 110 seconds
        */
        struct in_ifaddr **ifap = NULL;
        struct in_ifaddr *ifa = NULL;
        struct in_device *in_dev;
        A_UINT8 macMask[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
        A_STATUS status;
        WMI_ADD_WOW_PATTERN_CMD addWowCmd = { .filter = { 0 } };
        WMI_DEL_WOW_PATTERN_CMD delWowCmd;
        WMI_SET_HOST_SLEEP_MODE_CMD hostSleepMode = {FALSE, TRUE};
        WMI_SET_WOW_MODE_CMD wowMode = { .enable_wow = TRUE };

        ar6000_TxDataCleanup(ar); /* IMPORTANT, otherwise there will be 11mA after listen interval as 1000*/

#if 1 /* we don't do it if the power consumption is already good enough. */
        if (wmi_listeninterval_cmd(ar->arWmi, A_MAX_WOW_LISTEN_INTERVAL, 0) == A_OK) {
        }
#endif

//        wmi_set_keepalive_cmd(ar->arWmi, 110); /* keepalive otherwise, we will be disconnected*/
        status = wmi_scanparams_cmd(ar->arWmi, 0,0,0xffff,0,0,0,0,0,0,0);
        wmi_set_wow_mode_cmd(ar->arWmi, &wowMode);

        /* clear up our WoW pattern first */
        delWowCmd.filter_list_id = ANDROID_WOW_LIST_ID;
        delWowCmd.filter_id = 0;
        wmi_del_wow_pattern_cmd(ar->arWmi, &delWowCmd);

        /* setup unicast packet pattern for WoW */
        if (ar->arNetDev->dev_addr[1]) {
            addWowCmd.filter_list_id = ANDROID_WOW_LIST_ID;
            addWowCmd.filter_size = 6; /* MAC address */
            addWowCmd.filter_offset = 2;
            status = wmi_add_wow_pattern_cmd(ar->arWmi, &addWowCmd, ar->arNetDev->dev_addr, macMask, addWowCmd.filter_size);
        }
            /* setup ARP request for our own IP */
        if ((in_dev = __in_dev_get_rtnl(ar->arNetDev)) != NULL) {
            for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL; ifap = &ifa->ifa_next) {
                if (!strcmp(ar->arNetDev->name, ifa->ifa_label)) {
                    break; /* found */
                }
            }
        }
        if (ifa && ifa->ifa_local) {
            WMI_SET_IP_CMD ipCmd;
            memset(&ipCmd, 0, sizeof(ipCmd));
            ipCmd.ips[0] = ifa->ifa_local;
            status = wmi_set_ip_cmd(ar->arWmi, &ipCmd);
        }
        ar6k_send_asleep_event_to_app(ar, TRUE);
        wmi_set_host_sleep_mode_cmd(ar->arWmi, &hostSleepMode);
        if (ar->arTxPending[ar->arControlEp]) {
            long timeleft = wait_event_interruptible_timeout(arEvent,
            ar->arTxPending[ar->arControlEp] == 0, wmitimeout * HZ);
            if (!timeleft || signal_pending(current)) {
               /* what can I do? wow resume at once */
                printk("Fail to setup WoW\n");
            } else {
                ar->arWowState = 1;
                printk("Setup WoW successfully\n");
            }
        }
        mdelay(10);
    } else {
        printk("Not allowed to go to WOW at this moment.\n");
    }
}

static void ar6000_pwr_on(AR_SOFTC_T *ar)
{
    if (ar == NULL) {
        /* turn on for all cards */
    }
    printk("%s --enter\n", __func__);

}

static void ar6000_pwr_down(AR_SOFTC_T *ar)
{
    if (ar == NULL) {
        /* shutdown for all cards */
    }
    printk("%s --enter\n", __func__);

}

static A_STATUS ar6000_suspend_ev(void *context)
{
    int pmmode = buspm;
    AR_SOFTC_T *ar = (AR_SOFTC_T *)context;
    printk("%s: enter ar %p devices %p\n", __func__, ar, ar6000_devices[0]? netdev_priv(ar6000_devices[0]) : NULL);

wow_not_connected:

    switch (pmmode) {
    case WLAN_PWR_CTRL_DEEP_SLEEP:
        ar6000_set_wlan_state(ar, WLAN_DISABLED);
        ar->arOsPowerCtrl = WLAN_PWR_CTRL_DEEP_SLEEP;
        return A_EBUSY;
    case WLAN_PWR_CTRL_WOW:
        if (ar->arWmiReady && ar->arWlanState==WLAN_ENABLED && ar->arConnected) {
            ar->arOsPowerCtrl = WLAN_PWR_CTRL_WOW;
            /* leave for pm_device to setup wow */
            return A_EBUSY;
        } else {
            pmmode = wow2mode;
            goto wow_not_connected;
        }
        break;
    case WLAN_PWR_CTRL_CUT_PWR:
        /* fall through */
    default:
        ar->arOsPowerCtrl = WLAN_PWR_CTRL_CUT_PWR;
        ar6000_stop_endpoint(ar->arNetDev, TRUE);
        ar->arWlanState = WLAN_DISABLED;
        break;
    }
    return A_OK;
}

static A_STATUS ar6000_resume_ev(void *context)
{
    AR_SOFTC_T *ar = (AR_SOFTC_T *)context;
    A_UINT16 powerCtrl = ar->arOsPowerCtrl;
    wake_lock(&ar6k_init_wake_lock);
    printk("%s: enter\n", __func__);
    ar->arOsPowerCtrl = WLAN_PWR_CTRL_UP;
    switch (powerCtrl) {
    case WLAN_PWR_CTRL_WOW:
        printk("Warning! resume but osPowerCtl is not clear\n");
        break;
    case WLAN_PWR_CTRL_CUT_PWR:
        ar6000_restart_endpoint(ar->arNetDev);
        break;
    case WLAN_PWR_CTRL_DEEP_SLEEP:
        ar6000_set_wlan_state(ar, WLAN_ENABLED);
        break;
    default:
        printk("Strange SDIO bus power mode!!\n");
        break; 
    }
    wake_unlock(&ar6k_init_wake_lock);
    return A_OK;
}

static A_STATUS ar6000_android_avail_ev(void *context, void *hif_handle)
{
	A_STATUS ret;    
    wake_lock(&ar6k_init_wake_lock);
	ret = ar6000_avail_ev_p(context, hif_handle);
    wake_unlock(&ar6k_init_wake_lock);
    return ret;
}


static int ar6000_pm_suspend(struct platform_device *dev, pm_message_t state)
{
    int i;
    for (i = 0; i < MAX_AR6000; i++) {
        AR_SOFTC_T *ar;

        if (ar6000_devices[i] == NULL)
            continue;
        ar = (AR_SOFTC_T*)netdev_priv(ar6000_devices[i]);
        printk("%s: enter\n", __func__);
        switch (ar->arOsPowerCtrl) {
        case WLAN_PWR_CTRL_CUT_PWR:
            ar6000_pwr_down(ar);
            break;
        case WLAN_PWR_CTRL_WOW:
            ar6000_wow_suspend(ar);
            plat_disable_wlan_slot();
            break;
        case WLAN_PWR_CTRL_DEEP_SLEEP:
            /* nothing to do. keep the power on */
            break;
        default:
            printk("Something is strange for ar6000_pm_suspend %d\n", ar->arOsPowerCtrl);
            break;
       }
    }
    return 0;
}

static int ar6000_pm_resume(struct platform_device *dev)
{
    int i;
    for (i = 0; i < MAX_AR6000; i++) {
        AR_SOFTC_T *ar;

        if (ar6000_devices[i] == NULL)
            continue;
        printk("%s: enter\n", __func__);
        ar = (AR_SOFTC_T*)netdev_priv(ar6000_devices[i]);
        switch (ar->arOsPowerCtrl) {
        case WLAN_PWR_CTRL_CUT_PWR:
            ar6000_pwr_on(ar);
            break;
        case WLAN_PWR_CTRL_WOW:
            wake_lock_timeout(&ar6k_wow_wake_lock, 3*HZ);
            plat_enable_wlan_slot();
            ar6000_wow_resume(ar);
            ar->arOsPowerCtrl = WLAN_PWR_CTRL_UP;
            break;
        case WLAN_PWR_CTRL_DEEP_SLEEP:
            /* nothing to do. keep the power on */
            break;
        default:
            printk("Something is strange for ar6000_pm_resume %d\n", ar->arOsPowerCtrl);
            break;
       }
    }
    return 0;
}

static int ar6000_pm_probe(struct platform_device *pdev)
{
	ar6000_pwr_on(NULL);
	return 0;
}

static int ar6000_pm_remove(struct platform_device *pdev)
{
	ar6000_pwr_down(NULL);
	return 0;
}

static struct platform_driver ar6000_pm_device = {
	.probe		= ar6000_pm_probe,
	.remove		= ar6000_pm_remove,
	.suspend	= ar6000_pm_suspend,
	.resume		= ar6000_pm_resume,
	.driver		= {
			.name = "wlan_ar6000_pm_dev",
	},
};
#endif /* CONFIG_PM */

/* Useful for qualcom platform to detect our wlan card for mmc stack */
static void ar6000_enable_mmchost_detect_change(int enable)
{
#ifdef CONFIG_MMC_MSM 
    mm_segment_t        oldfs;
    struct file     *filp = (struct file*)-ENOENT;
    int         length;
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    do {
        char buf[3];
        filp = filp_open("/sys/devices/platform/msm_sdcc.2/detect_change", O_RDWR, S_IRUSR);
        if (IS_ERR(filp) || !filp->f_op)
            break;
        length = snprintf(buf, sizeof(buf), "%d\n", enable ? 1 : 0);
        if (filp->f_op->write(filp, buf, length, &filp->f_pos) != length) {
            break;
        }
    } while (0);
    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
    set_fs(oldfs);
#endif
}

static void
ar6000_restart_endpoint(struct net_device *dev)
{
    A_STATUS status = A_OK;
    AR_SOFTC_T *ar = (AR_SOFTC_T*)netdev_priv(dev);
    if (down_interruptible(&ar->arSem)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s(): down_interruptible failed \n", __func__));
        return ;
    }
    if (ar->bIsDestroyProgress) {
        up(&ar->arSem);
        return;
    }
    BMIInit();
    do {        
        A_BOOL rtnl_lock_held_on_entry; 
        if ( (status=ar6000_configure_target(ar))!=A_OK)
            break;
		if ( (status=ar6000_sysfs_bmi_get_config(ar, wlaninitmode)) != A_OK)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("ar6000_avail: ar6000_sysfs_bmi_get_config failed\n"));
			break;
		}     
        rtnl_lock_held_on_entry = rtnl_trylock();
        status = (ar6000_init(dev)==0) ? A_OK : A_ERROR;
        if (rtnl_lock_held_on_entry) {
            rtnl_unlock();
        }
        if (status!=A_OK) {
            break;
        }   
        ar->arWlanState = WLAN_ENABLED;
        if (ar->arSsidLen) {
            ar6000_connect_to_ap(ar);
        }
    } while (0);

    up(&ar->arSem);    
    if (status==A_OK) {
        return;
    }

    ar6000_devices[ar->arDeviceIndex] = NULL;
    ar6000_destroy(ar->arNetDev, 1);
}

void android_module_init(OSDRV_CALLBACKS *osdrvCallbacks)
{
    ar6000_enable_mmchost_detect_change(1);
    bmienable = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (ifname[0] == '\0')
        strcpy(ifname, def_ifname);
#endif 
    if (wow2mode!=WLAN_PWR_CTRL_CUT_PWR && wow2mode!=WLAN_PWR_CTRL_DEEP_SLEEP) {
        wow2mode=WLAN_PWR_CTRL_CUT_PWR;
    }

    wake_lock_init(&ar6k_init_wake_lock, WAKE_LOCK_SUSPEND, "ar6k_init");
    wake_lock_init(&ar6k_wow_wake_lock, WAKE_LOCK_SUSPEND, "ar6k_wow");

#if defined(CONFIG_PM)
    osdrvCallbacks->deviceSuspendHandler = ar6000_suspend_ev;
    osdrvCallbacks->deviceResumeHandler = ar6000_resume_ev;
#endif
    ar6000_avail_ev_p = osdrvCallbacks->deviceInsertedHandler;
    osdrvCallbacks->deviceInsertedHandler = ar6000_android_avail_ev;

#if defined(CONFIG_PM)
    /* Register ar6000_pm_device into system.
     * We should also add platform_device into the first item of array devices[] in
     * file arch/xxx/mach-xxx/board-xxxx.c
     * Otherwise, WoW may not work properly since we may trigger WoW GPIO before system suspend
     */
    if (platform_driver_register(&ar6000_pm_device))
        printk("ar6000: fail to register the power control driver.\n");
#endif
}

void android_module_exit(void)
{
    wake_lock_destroy(&ar6k_wow_wake_lock);
    wake_lock_destroy(&ar6k_init_wake_lock);

#ifdef CONFIG_PM
    platform_driver_unregister(&ar6000_pm_device);
#endif
    ar6000_enable_mmchost_detect_change(1);
}

A_BOOL android_ar6k_endpoint_is_stop(AR_SOFTC_T *ar)
{
#ifdef CONFIG_PM
    return ar->arOsPowerCtrl == WLAN_PWR_CTRL_CUT_PWR;
#else
    return FALSE;
#endif 
}

void android_ar6k_check_wow_status(AR_SOFTC_T *ar)
{
#ifdef CONFIG_PM
	if (ar->arWowState) {
		ar6000_wow_resume(ar);
	}
#endif /* CONFIG_PM */
}

A_STATUS android_ar6k_start(AR_SOFTC_T *ar)
{
    return A_OK;
}
