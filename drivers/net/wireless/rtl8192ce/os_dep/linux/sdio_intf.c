/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _HCI_INTF_C_

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <recv_osdep.h>
#include <xmit_osdep.h>
#include <rtw_version.h>

#ifndef CONFIG_SDIO_HCI
#error "CONFIG_SDIO_HCI shall be on!\n"
#endif

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#ifdef CONFIG_RTL8723A
#include <rtl8723a_hal.h>
#endif

#include <hal_init.h>
#include <sdio_hal.h>
#include <sdio_ops.h>


static const struct sdio_device_id sdio_ids[] = {
	{ SDIO_DEVICE(0x024c, 0x8723) },
//	{ SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN)		},
//	{ /* end: all zeroes */				},
};

struct sdio_drv_priv {
	struct sdio_driver r871xs_drv;

	_mutex hw_init_mutex;
};

static void sd_sync_int_hdl(struct sdio_func *func)
{
	struct dvobj_priv *psdpriv;


	psdpriv = sdio_get_drvdata(func);
	sd_int_hdl(psdpriv->padapter);
}

static u32 sdio_init(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	PSDIO_DATA psdio_data;
	struct sdio_func *func;
	int err;

_func_enter_;

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+sdio_init\n"));
	if (padapter == NULL) {
		printk(KERN_ERR "%s: padapter is NULL!\n", __func__);
		err = -1;
		goto exit;
	}

	psddev = &padapter->dvobjpriv;
	psdio_data = &psddev->intf_data;
	func = psdio_data->func;

	//3 1. init SDIO bus
	sdio_claim_host(func);

	err = sdio_enable_func(func);
	if (err) {
		printk(KERN_CRIT "%s: sdio_enable_func FAIL(%d)!\n", __func__, err);
		goto release;
	}

	err = sdio_set_block_size(func, 512);
	if (err) {
		printk(KERN_CRIT "%s: sdio_set_block_size FAIL(%d)!\n", __func__, err);
		goto release;
	}
	psdio_data->block_transfer_len = 512;
	psdio_data->tx_block_mode = 1;
	psdio_data->rx_block_mode = 1;

release:
	sdio_release_host(func);

exit:
_func_exit_;

	if (err) return _FAIL;
	return _SUCCESS;
}

static void sdio_deinit(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	struct sdio_func *func;
	int err;


	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+sdio_deinit\n"));

	if (padapter == NULL) return;
	psddev = &padapter->dvobjpriv;
	func = psddev->intf_data.func;

	if (func) {
		sdio_claim_host(func);
		err = sdio_disable_func(func);
		if (err)
			printk(KERN_ERR "%s: sdio_disable_func(%d)\n", __func__, err);
		sdio_release_host(func);
	}
}

thread_return rtw_xmit_thread(thread_context context)
{
	s32 err;
	PADAPTER padapter;


	err = _SUCCESS;
	padapter = (PADAPTER)context;

#if 0
	thread_enter(padapter->pnetdev);
#else
//	daemonize("%s", padapter->pnetdev->name);
	daemonize("%s", "RTW_XMIT_THREAD");
	allow_signal(SIGTERM);
#endif

	do {
		err = hal_xmit_handler(padapter);
		if (signal_pending(current)) {
			flush_signals(current);
		}
	} while (_SUCCESS == err);

	_rtw_up_sema(&padapter->xmitpriv.terminate_xmitthread_sema);

	thread_exit();
}

static void decide_chip_type_by_device_id(PADAPTER padapter, u32 id)
{
	padapter->chip_type = NULL_CHIP_TYPE;

	switch (id)
	{
		case 0x8723:
			padapter->chip_type = RTL8188C_8192C;
			padapter->HardwareType = HARDWARE_TYPE_RTL8723AS;
			break;
	}
}

static void sd_intf_start(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	struct sdio_func *func;
	int err;

	if (padapter == NULL) goto exit;
	
	psddev = &padapter->dvobjpriv;
	func = psddev->intf_data.func;

	//os/intf dep
	if (func) {		
		sdio_claim_host(func);

		//according to practice, this is needed...
		err = sdio_set_block_size(func, 512);
		if (err) {
			printk(KERN_CRIT "%s: sdio_set_block_size FAIL(%d)!\n", __func__, err);
			goto release_host;
		}	
		err = sdio_claim_irq(func, &sd_sync_int_hdl);
		if (err) {
			printk(KERN_CRIT "%s: sdio_claim_irq FAIL(%d)!\n", __func__, err);
			goto release_host;
		}
	
release_host:
		sdio_release_host(func);
	}

	//hal dep
	if (padapter->HalFunc.enable_interrupt)
		padapter->HalFunc.enable_interrupt(padapter);
	else {
		DBG_871X("%s HalFunc.enable_interrupt is %p\n", __FUNCTION__, padapter->HalFunc.enable_interrupt);
		goto exit;
	}

exit:
	return;
}

static void sd_intf_stop(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	struct sdio_func *func;
	int err;

	if (padapter == NULL) goto exit;
	
	psddev = &padapter->dvobjpriv;
	func = psddev->intf_data.func;

	//hal dep
	if (padapter->HalFunc.disable_interrupt)
		padapter->HalFunc.disable_interrupt(padapter);
	else
	{
		DBG_871X("%s HalFunc.disable_interrupt is %p\n", __FUNCTION__, padapter->HalFunc.disable_interrupt);
		goto exit;
	}

	//os/intf dep
	if (func) {
		sdio_claim_host(func);

		err = sdio_release_irq(func);
		if (err)
			printk(KERN_ERR "%s: sdio_release_irq(%d)\n", __func__, err);

		sdio_release_host(func);
	}

exit:
	return;
}

/*
 * drv_init() - a device potentially for us
 *
 * notes: drv_init() is called when the bus driver has located a card for us to support.
 *        We accept the new device by returning 0.
 */
static int rtw_drv_init(
	struct sdio_func *func,
	const struct sdio_device_id *id)
{
	struct net_device *pnetdev;
	PADAPTER padapter;
	struct dvobj_priv *pdvobjpriv;
	PSDIO_DATA psdio;


	RT_TRACE(_module_hci_intfs_c_, _drv_info_,
		 ("+rtw_drv_init: vendor=0x%04x device=0x%04x class=0x%02x\n",
		  func->vendor, func->device, func->class));

	//3 1. init network device data
	pnetdev = rtw_init_netdev(NULL);
	if (!pnetdev) goto error;

	SET_NETDEV_DEV(pnetdev, &func->dev);

	padapter = rtw_netdev_priv(pnetdev);
	pdvobjpriv = &padapter->dvobjpriv;
	pdvobjpriv->padapter = padapter;
	psdio = &pdvobjpriv->intf_data;
	psdio->func = func;

	//3 2. set interface private data
	sdio_set_drvdata(func, pdvobjpriv);

	//3 3. init driver special setting, interface, OS and hardware relative
	// set interface_type to sdio
	padapter->interface_type = RTW_SDIO;
	decide_chip_type_by_device_id(padapter, (u32)func->device);

	//4 3.1 set hardware operation functions
	padapter->HalData = rtw_zmalloc(sizeof(HAL_DATA_TYPE));
	if (padapter->HalData == NULL) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: can't alloc memory for HAL DATA\n"));
		goto error;
	}
	set_hal_ops(padapter);

	//3 4. interface init
	if (sdio_init(padapter) != _SUCCESS) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: initialize device object priv Failed!\n"));
		goto error;
	}
	padapter->intf_start = &sd_intf_start;
	padapter->intf_stop = &sd_intf_stop;

	//3 5. register I/O operations
	if (rtw_init_io_priv(padapter, sdio_set_intf_ops) == _FAIL)
	{
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			("rtw_drv_init: Can't init io_priv\n"));
		goto deinit;
	}

	//3 6.
	intf_read_chip_version(padapter);

	//3 7.
	intf_chip_configure(padapter);

	//3 8. read efuse/eeprom data
	intf_read_chip_info(padapter);

	//3 9. init driver common data
	if (rtw_init_drv_sw(padapter) == _FAIL) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: Initialize driver software resource Failed!\n"));
		goto deinit;
	}

	//3 10. get WLan MAC address
	// alloc dev name after read efuse.
	rtw_init_netdev_name(pnetdev, padapter->registrypriv.ifname);

	rtw_macaddr_cfg(padapter->eeprompriv.mac_addr);
	_rtw_memcpy(pnetdev->dev_addr, padapter->eeprompriv.mac_addr, ETH_ALEN);

	//3 11. Tell the network stack we exist
	if (register_netdev(pnetdev) != 0) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: register_netdev() failed\n"));
		goto deinit;
	}


#ifdef CONFIG_PROC_DEBUG
#ifdef RTK_DMP_PLATFORM
	rtw_proc_init_one(pnetdev);
#endif
#endif

#ifdef CONFIG_HOSTAPD_MLME
	hostapd_mode_init(padapter);
#endif

	padapter->hw_init_mutex = &sdio_drvpriv.hw_init_mutex;
#ifdef CONFIG_CONCURRENT_MODE

	//set global variable to primary adapter
	padapter->ph2c_fwcmd_mutex = &sdio_drvpriv.h2c_fwcmd_mutex;
	padapter->psetch_mutex = &sdio_drvpriv.setch_mutex;
	padapter->psetbw_mutex = &sdio_drvpriv.setbw_mutex;	
	if(rtw_drv_if2_init(padapter, NULL)==NULL)
	{
		goto error;
	}	
#endif

#ifdef CONFIG_PLATFORM_RTD2880B
	DBG_871X("wlan link up\n");
	rtd2885_wlan_netlink_sendMsg("linkup", "8712");
#endif


#ifdef CONFIG_GLOBAL_UI_PID
	if(ui_pid[1]!=0) {
		DBG_871X("ui_pid[1]:%d\n",ui_pid[1]);
		rtw_signal_process(ui_pid[1], SIGUSR2);
	}
#endif

	DBG_871X("bDriverStopped:%d, bSurpriseRemoved:%d, bup:%d, hw_init_completed:%d\n"
		,padapter->bDriverStopped
		,padapter->bSurpriseRemoved
		,padapter->bup
		,padapter->hw_init_completed
	);


	RT_TRACE(_module_hci_intfs_c_, _drv_info_,
		 ("-rtw_drv_init: Success. bDriverStopped=%d bSurpriseRemoved=%d\n",
		  padapter->bDriverStopped, padapter->bSurpriseRemoved));

	return 0;

deinit:
	sdio_deinit(padapter);

error:
	if (padapter)
	{
		if (padapter->HalData) {
			rtw_mfree(padapter->HalData, sizeof(HAL_DATA_TYPE));
			padapter->HalData = NULL;
		}
	}

	if (pnetdev)
		rtw_free_netdev(pnetdev);

	RT_TRACE(_module_hci_intfs_c_, _drv_crit_, ("-rtw_drv_init: FAIL!\n"));

	return -1;
}

/*
 * Do deinit job corresponding to netdev_open()
 */
static void rtw_dev_unload(PADAPTER padapter)
{
	struct net_device *pnetdev = (struct net_device*)padapter->pnetdev;


	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_dev_unload\n"));

	padapter->bDriverStopped = _TRUE;

	if (padapter->bup == _TRUE)
	{
		// stop TX
//		val8 = 0xFF;
//		padapter->HalFunc.SetHwRegHandler(padapter, HW_VAR_TXPAUSE,&val8);

#if 0
		if (padapter->intf_stop)
			padapter->intf_stop(padapter);
#else
		sd_intf_stop(padapter);
#endif
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: stop intf complete!\n"));

		if (!padapter->pwrctrlpriv.bInternalAutoSuspend)
			rtw_stop_drv_threads(padapter);
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: stop thread complete!\n"));

		if (padapter->bSurpriseRemoved == _FALSE)
		{
#ifdef CONFIG_WOWLAN
			if (padapter->pwrctrlpriv.bSupportWakeOnWlan == _TRUE) {
				printk("%s bSupportWakeOnWlan==_TRUE  do not run rtw_hal_deinit()\n",__FUNCTION__);
			}
			else
#endif
			{
				rtw_hal_deinit(padapter);
			}
			padapter->bSurpriseRemoved = _TRUE;
		}
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: deinit hal complelt!\n"));

		padapter->bup = _FALSE;
	}
	else {
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_unload: bup==_FALSE\n"));
	}

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_dev_unload\n"));
}

static void rtw_dev_remove(struct sdio_func *func)
{
	PADAPTER padapter;
	struct net_device *pnetdev;

_func_enter_;

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_dev_remove\n"));

	padapter = ((struct dvobj_priv*)sdio_get_drvdata(func))->padapter;

#if defined(CONFIG_HAS_EARLYSUSPEND ) || defined(CONFIG_ANDROID_POWER)
	rtw_unregister_early_suspend(&padapter->pwrctrlpriv);
#endif

	if (padapter->bSurpriseRemoved == _FALSE)
	{
		// test surprise remove
		int err;

		sdio_claim_host(func);
		sdio_readb(func, 0, &err);
		sdio_release_host(func);
		if (err == -ENOMEDIUM) {
			padapter->bSurpriseRemoved = _TRUE;
			printk(KERN_NOTICE "%s: device had been removed!\n", __func__);
		}
	}

#ifdef CONFIG_HOSTAPD_MLME
	hostapd_mode_unload(padapter);
#endif
	LeaveAllPowerSaveMode(padapter);

	pnetdev = (struct net_device*)padapter->pnetdev;
	if (pnetdev) {
		unregister_netdev(pnetdev); //will call netdev_close()
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_remove: unregister netdev\n"));
#ifdef CONFIG_PROC_DEBUG
		rtw_proc_remove_one(pnetdev);
#endif
	} else {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_, ("rtw_dev_remove: NO padapter->pnetdev!\n"));
	}

	rtw_cancel_all_timer(padapter);

	rtw_dev_unload(padapter);

	// interface deinit
	sdio_deinit(padapter);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_remove: deinit intf complete!\n"));

	rtw_free_drv_sw(padapter);

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_dev_remove\n"));

_func_exit_;
}

static int rtw_sdio_suspend(struct device *dev)
{
	struct sdio_func *func =dev_to_sdio_func(dev);
	struct dvobj_priv *psdpriv = sdio_get_drvdata(func);
	_adapter *padapter = psdpriv->padapter;
	struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct net_device *pnetdev = padapter->pnetdev;
	int ret = 0;
	u32 start_time = rtw_get_current_time();
	
	_func_enter_;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	if((!padapter->bup) || (padapter->bDriverStopped)||(padapter->bSurpriseRemoved))
	{
		DBG_871X("%s bup=%d bDriverStopped=%d bSurpriseRemoved = %d\n", __FUNCTION__
			,padapter->bup, padapter->bDriverStopped,padapter->bSurpriseRemoved);
		goto exit;
	}
	
	pwrpriv->bInSuspend = _TRUE;		
	rtw_cancel_all_timer(padapter);		
	LeaveAllPowerSaveMode(padapter);
	
	//padapter->net_closed = _TRUE;
	//s1.
	if(pnetdev)
	{
		netif_carrier_off(pnetdev);
		rtw_netif_stop_queue(pnetdev);
	}
#ifdef CONFIG_WOWLAN
	padapter->pwrctrlpriv.bSupportWakeOnWlan=_TRUE;
#else		
	//s2.
	//s2-1.  issue rtw_disassoc_cmd to fw
	disconnect_hdl(padapter, NULL);
	//rtw_disassoc_cmd(padapter);
#endif

#ifdef CONFIG_LAYER2_ROAMING_RESUME
	if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) && check_fwstate(pmlmepriv, _FW_LINKED) )
	{
		DBG_871X("%s %s(" MAC_FMT "), length:%d assoc_ssid.length:%d\n",__FUNCTION__,
				pmlmepriv->cur_network.network.Ssid.Ssid,
				MAC_ARG(pmlmepriv->cur_network.network.MacAddress),
				pmlmepriv->cur_network.network.Ssid.SsidLength,
				pmlmepriv->assoc_ssid.SsidLength);
		
		pmlmepriv->to_roaming = 1;
	}
#endif
	//s2-2.  indicate disconnect to os
	rtw_indicate_disconnect(padapter);
	//s2-3.
	rtw_free_assoc_resources(padapter, 1);

	//s2-4.
	rtw_free_network_queue(padapter, _TRUE);

	rtw_led_control(padapter, LED_CTL_POWER_OFF);

	rtw_dev_unload(padapter);

exit:
	DBG_871X("<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	_func_exit_;
	return ret;
}

extern int pm_netdev_open(struct net_device *pnetdev,u8 bnormal);
int rtw_resume_process(_adapter *padapter)
{
	struct net_device *pnetdev;
	struct pwrctrl_priv *pwrpriv;
	u8 is_pwrlock_hold_by_caller;
	u8 is_directly_called_by_auto_resume;
	int ret = 0;
	u32 start_time = rtw_get_current_time();
	
	_func_enter_;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	if(padapter) {
		pnetdev= padapter->pnetdev;
		pwrpriv = &padapter->pwrctrlpriv;
	} else {
		ret =-1;
		goto exit;
	}
	
	rtw_reset_drv_sw(padapter);
	pwrpriv->bkeepfwalive = _FALSE;

	DBG_871X("bkeepfwalive(%x)\n",pwrpriv->bkeepfwalive);
	if(pm_netdev_open(pnetdev,_TRUE) != 0) {
		ret = -1;
		goto exit;
	}

	netif_device_attach(pnetdev);	
	netif_carrier_on(pnetdev);

	if( padapter->pid[1]!=0) {
		DBG_871X("pid[1]:%d\n",padapter->pid[1]);
		rtw_signal_process(padapter->pid[1], SIGUSR2);
	}	

	#ifdef CONFIG_LAYER2_ROAMING_RESUME
	rtw_roaming(padapter, NULL);
	#endif	
	
	#ifdef CONFIG_RESUME_IN_WORKQUEUE
	rtw_unlock_suspend();
	#endif //CONFIG_RESUME_IN_WORKQUEUE

exit:
	DBG_871X("<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	_func_exit_;
	
	return ret;
}

static int rtw_sdio_resume(struct device *dev)
{
	struct sdio_func *func =dev_to_sdio_func(dev);
	struct dvobj_priv *psdpriv = sdio_get_drvdata(func);
	_adapter *padapter = psdpriv->padapter;
	struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
	 int ret = 0;
 
	if(pwrpriv->bInternalAutoSuspend ){
 		ret = rtw_resume_process(padapter);
	} else {
#ifdef CONFIG_RESUME_IN_WORKQUEUE
		rtw_resume_in_workqueue(pwrpriv);
#elif defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_ANDROID_POWER)
		if(rtw_is_earlysuspend_registered(pwrpriv)) {
			//jeff: bypass resume here, do in late_resume
			pwrpriv->do_late_resume = _TRUE;
		} else {
			ret = rtw_resume_process(padapter);
		}
#else // Normal resume process
		ret = rtw_resume_process(padapter);
#endif //CONFIG_RESUME_IN_WORKQUEUE
	}
	
	return ret;

}

static const struct dev_pm_ops rtw_sdio_pm_ops = {
	.suspend	= rtw_sdio_suspend,
	.resume	= rtw_sdio_resume,
};


static struct sdio_drv_priv sdio_drvpriv = {
	.r871xs_drv.probe = rtw_drv_init,
	.r871xs_drv.remove = rtw_dev_remove,
	.r871xs_drv.name = (char*)DRV_NAME,
	.r871xs_drv.id_table = sdio_ids,

	.r871xs_drv.drv = {
		.pm = &rtw_sdio_pm_ops,
	}
};

static int __init rtw_drv_entry(void)
{
	int ret;


//	printk(KERN_INFO "+%s", __func__);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_drv_entry\n"));
	printk(KERN_NOTICE DRV_NAME " driver version " DRIVERVERSION "\n");

	_rtw_mutex_init(&sdio_drvpriv.hw_init_mutex);
	ret = sdio_register_driver(&sdio_drvpriv.r871xs_drv);
//	printk(KERN_INFO "-%s: ret=%d", __func__, ret);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_drv_entry: ret=%d\n", ret));

	return ret;
}

static void __exit rtw_drv_halt(void)
{
//	printk(KERN_INFO "+%s", __func__);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_drv_halt\n"));
	sdio_unregister_driver(&sdio_drvpriv.r871xs_drv);
//	printk(KERN_INFO "-%s", __func__);
	_rtw_mutex_free(&sdio_drvpriv.hw_init_mutex);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_drv_halt\n"));
}


module_init(rtw_drv_entry);
module_exit(rtw_drv_halt);
