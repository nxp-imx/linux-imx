/**  @file moal_proc.c
 *
 * @brief This file contains functions for proc file.
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

#include "moal_main.h"
#ifdef UAP_SUPPORT
#include "moal_uap.h"
#endif
#ifdef SDIO
#include "moal_sdio.h"
#endif

/********************************************************
		Local Variables
********************************************************/
#ifdef CONFIG_PROC_FS
#define STATUS_PROC "wifi_status"
#define MWLAN_PROC "mwlan"
#define WLAN_PROC "adapter%d"
/** Proc mwlan directory entry */
struct proc_dir_entry *proc_mwlan;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#define PROC_DIR NULL
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#define PROC_DIR (&proc_root)
#else
#define PROC_DIR proc_net
#endif

#ifdef STA_SUPPORT
static char *szModes[] = {
	"Unknown",
	"Managed",
	"Ad-hoc",
	"Auto",
};
#endif

/********************************************************
		Global Variables
********************************************************/
int wifi_status;

/********************************************************
		Local Functions
********************************************************/
/**
 *  @brief Proc read function for info
 *
 *  @param sfp      pointer to seq_file structure
 *  @param data
 *
 *  @return         Number of output data
 */
static int woal_info_proc_read(struct seq_file *sfp, void *data)
{
	struct net_device *netdev = (struct net_device *)sfp->private;
	char fmt[MLAN_MAX_VER_STR_LEN];
	moal_private *priv = (moal_private *)netdev_priv(netdev);
#ifdef STA_SUPPORT
	int i = 0;
	moal_handle *handle = NULL;
	mlan_bss_info info;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	struct dev_mc_list *mcptr = netdev->mc_list;
	int mc_count = netdev->mc_count;
#else
	struct netdev_hw_addr *mcptr = NULL;
	int mc_count = netdev_mc_count(netdev);
#endif /* < 2.6.35 */
#else
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	int i = 0;
#endif /* >= 2.6.29 */
#endif
#ifdef UAP_SUPPORT
	mlan_ds_uap_stats ustats;
#endif
	union {
		t_u32 l;
		t_u8 c[4];
	} ver;

	ENTER();

	if (priv == NULL)
		goto exit;
#ifdef STA_SUPPORT
	handle = priv->phandle;
	if (handle == NULL)
		goto exit;
#endif

	if (!MODULE_GET) {
		LEAVE();
		return 0;
	}

	memset(fmt, 0, sizeof(fmt));
#ifdef UAP_SUPPORT
	memset(&ustats, 0, sizeof(ustats));
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		seq_printf(sfp, "driver_name = "
				"\"uap\"\n");
		woal_uap_get_version(priv, fmt, sizeof(fmt) - 1);
		if (MLAN_STATUS_SUCCESS !=
		    woal_uap_get_stats(priv, MOAL_IOCTL_WAIT, &ustats)) {
			MODULE_PUT;
			LEAVE();
			return -EFAULT;
		}
	}
#endif /* UAP_SUPPORT*/
#ifdef STA_SUPPORT
	memset(&info, 0, sizeof(info));
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		woal_get_version(handle, fmt, sizeof(fmt) - 1);
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &info)) {
			MODULE_PUT;
			LEAVE();
			return -EFAULT;
		}
		seq_printf(sfp, "driver_name = "
				"\"wlan\"\n");
	}
#endif
	seq_printf(sfp, "driver_version = %s", fmt);
	seq_printf(sfp, "\ninterface_name=\"%s\"\n", netdev->name);
	ver.l = handle->fw_release_number;
	seq_printf(sfp, "firmware_major_version=%u.%u.%u\n", ver.c[2], ver.c[1],
		   ver.c[0]);
#ifdef WIFI_DIRECT_SUPPORT
	if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA)
			seq_printf(sfp, "bss_mode = \"WIFIDIRECT-Client\"\n");
		else
			seq_printf(sfp, "bss_mode = \"WIFIDIRECT-GO\"\n");
	}
#endif
#ifdef STA_SUPPORT
	if (priv->bss_type == MLAN_BSS_TYPE_STA)
		seq_printf(sfp, "bss_mode =\"%s\"\n", szModes[info.bss_mode]);
#endif
	seq_printf(sfp, "media_state=\"%s\"\n",
		   ((priv->media_connected == MFALSE) ? "Disconnected" :
							"Connected"));
	seq_printf(sfp, "mac_address=\"%02x:%02x:%02x:%02x:%02x:%02x\"\n",
		   netdev->dev_addr[0], netdev->dev_addr[1],
		   netdev->dev_addr[2], netdev->dev_addr[3],
		   netdev->dev_addr[4], netdev->dev_addr[5]);
#ifdef STA_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		seq_printf(sfp, "multicast_count=\"%d\"\n", mc_count);
		seq_printf(sfp, "essid=\"%s\"\n", info.ssid.ssid);
		seq_printf(sfp, "bssid=\"%02x:%02x:%02x:%02x:%02x:%02x\"\n",
			   info.bssid[0], info.bssid[1], info.bssid[2],
			   info.bssid[3], info.bssid[4], info.bssid[5]);
		seq_printf(sfp, "channel=\"%d\"\n", (int)info.bss_chan);
		seq_printf(sfp, "region_code = \"%02x\"\n",
			   (t_u8)info.region_code);

		/*
		 * Put out the multicast list
		 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
		for (i = 0; i < netdev->mc_count; i++) {
			seq_printf(
				sfp,
				"multicast_address[%d]=\"%02x:%02x:%02x:%02x:%02x:%02x\"\n",
				i, mcptr->dmi_addr[0], mcptr->dmi_addr[1],
				mcptr->dmi_addr[2], mcptr->dmi_addr[3],
				mcptr->dmi_addr[4], mcptr->dmi_addr[5]);

			mcptr = mcptr->next;
		}
#else
		netdev_for_each_mc_addr (mcptr, netdev)
			seq_printf(
				sfp,
				"multicast_address[%d]=\"%02x:%02x:%02x:%02x:%02x:%02x\"\n",
				i++, mcptr->addr[0], mcptr->addr[1],
				mcptr->addr[2], mcptr->addr[3], mcptr->addr[4],
				mcptr->addr[5]);
#endif /* < 2.6.35 */
	}
#endif
	seq_printf(sfp, "num_tx_bytes = %lu\n", priv->stats.tx_bytes);
	seq_printf(sfp, "num_rx_bytes = %lu\n", priv->stats.rx_bytes);
	seq_printf(sfp, "num_tx_pkts = %lu\n", priv->stats.tx_packets);
	seq_printf(sfp, "num_rx_pkts = %lu\n", priv->stats.rx_packets);
	seq_printf(sfp, "num_tx_pkts_dropped = %lu\n", priv->stats.tx_dropped);
	seq_printf(sfp, "num_rx_pkts_dropped = %lu\n", priv->stats.rx_dropped);
	seq_printf(sfp, "num_tx_pkts_err = %lu\n", priv->stats.tx_errors);
	seq_printf(sfp, "num_rx_pkts_err = %lu\n", priv->stats.rx_errors);
	seq_printf(sfp, "carrier %s\n",
		   ((netif_carrier_ok(priv->netdev)) ? "on" : "off"));
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
	for (i = 0; i < netdev->num_tx_queues; i++) {
		seq_printf(sfp, "tx queue %d:  %s\n", i,
			   ((netif_tx_queue_stopped(
				    netdev_get_tx_queue(netdev, 0))) ?
				    "stopped" :
				    "started"));
	}
#else
	seq_printf(sfp, "tx queue %s\n",
		   ((netif_queue_stopped(priv->netdev)) ? "stopped" :
							  "started"));
#endif
#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		seq_printf(sfp, "tkip_mic_failures = %u\n",
			   ustats.tkip_mic_failures);
		seq_printf(sfp, "ccmp_decrypt_errors = %u\n",
			   ustats.ccmp_decrypt_errors);
		seq_printf(sfp, "wep_undecryptable_count = %u\n",
			   ustats.wep_undecryptable_count);
		seq_printf(sfp, "wep_icv_error_count = %u\n",
			   ustats.wep_icv_error_count);
		seq_printf(sfp, "decrypt_failure_count = %u\n",
			   ustats.decrypt_failure_count);
		seq_printf(sfp, "mcast_tx_count = %u\n", ustats.mcast_tx_count);
		seq_printf(sfp, "failed_count = %u\n", ustats.failed_count);
		seq_printf(sfp, "retry_count = %u\n", ustats.retry_count);
		seq_printf(sfp, "multiple_retry_count = %u\n",
			   ustats.multi_retry_count);
		seq_printf(sfp, "frame_duplicate_count = %u\n",
			   ustats.frame_dup_count);
		seq_printf(sfp, "rts_success_count = %u\n",
			   ustats.rts_success_count);
		seq_printf(sfp, "rts_failure_count = %u\n",
			   ustats.rts_failure_count);
		seq_printf(sfp, "ack_failure_count = %u\n",
			   ustats.ack_failure_count);
		seq_printf(sfp, "rx_fragment_count = %u\n",
			   ustats.rx_fragment_count);
		seq_printf(sfp, "mcast_rx_frame_count = %u\n",
			   ustats.mcast_rx_frame_count);
		seq_printf(sfp, "fcs_error_count = %u\n",
			   ustats.fcs_error_count);
		seq_printf(sfp, "tx_frame_count = %u\n", ustats.tx_frame_count);
		seq_printf(sfp, "rsna_tkip_cm_invoked = %u\n",
			   ustats.rsna_tkip_cm_invoked);
		seq_printf(sfp, "rsna_4way_hshk_failures = %u\n",
			   ustats.rsna_4way_hshk_failures);
	}
#endif /* UAP_SUPPORT */
	seq_printf(sfp, "=== tp_acnt.on:%d drop_point:%d ===\n",
		   handle->tp_acnt.on, handle->tp_acnt.drop_point);
	seq_printf(sfp, "====Tx accounting====\n");
	for (i = 0; i < MAX_TP_ACCOUNT_DROP_POINT_NUM; i++) {
		seq_printf(sfp, "[%d] Tx packets     : %lu\n", i,
			   handle->tp_acnt.tx_packets[i]);
		seq_printf(sfp, "[%d] Tx packets last: %lu\n", i,
			   handle->tp_acnt.tx_packets_last[i]);
		seq_printf(sfp, "[%d] Tx packets rate: %lu\n", i,
			   handle->tp_acnt.tx_packets_rate[i]);
		seq_printf(sfp, "[%d] Tx bytes       : %lu\n", i,
			   handle->tp_acnt.tx_bytes[i]);
		seq_printf(sfp, "[%d] Tx bytes last  : %lu\n", i,
			   handle->tp_acnt.tx_bytes_last[i]);
		seq_printf(sfp, "[%d] Tx bytes rate  : %luMbps\n", i,
			   handle->tp_acnt.tx_bytes_rate[i] * 8 / 1024 / 1024);
	}
	seq_printf(sfp, "Tx intr cnt    		: %lu\n",
		   handle->tp_acnt.tx_intr_cnt);
	seq_printf(sfp, "Tx intr last        : %lu\n",
		   handle->tp_acnt.tx_intr_last);
	seq_printf(sfp, "Tx intr rate        : %lu\n",
		   handle->tp_acnt.tx_intr_rate);
	seq_printf(sfp, "Tx pending          : %lu\n",
		   handle->tp_acnt.tx_pending);
	seq_printf(sfp, "====Rx accounting====\n");
	for (i = 0; i < MAX_TP_ACCOUNT_DROP_POINT_NUM; i++) {
		seq_printf(sfp, "[%d] Rx packets     : %lu\n", i,
			   handle->tp_acnt.rx_packets[i]);
		seq_printf(sfp, "[%d] Rx packets last: %lu\n", i,
			   handle->tp_acnt.rx_packets_last[i]);
		seq_printf(sfp, "[%d] Rx packets rate: %lu\n", i,
			   handle->tp_acnt.rx_packets_rate[i]);
		seq_printf(sfp, "[%d] Rx bytes       : %lu\n", i,
			   handle->tp_acnt.rx_bytes[i]);
		seq_printf(sfp, "[%d] Rx bytes last  : %lu\n", i,
			   handle->tp_acnt.rx_bytes_last[i]);
		seq_printf(sfp, "[%d] Rx bytes rate  : %luMbps\n", i,
			   handle->tp_acnt.rx_bytes_rate[i] * 8 / 1024 / 1024);
	}
	seq_printf(sfp, "Rx intr cnt    	 : %lu\n",
		   handle->tp_acnt.rx_intr_cnt);
	seq_printf(sfp, "Rx intr last        : %lu\n",
		   handle->tp_acnt.rx_intr_last);
	seq_printf(sfp, "Rx intr rate        : %lu\n",
		   handle->tp_acnt.rx_intr_rate);
	seq_printf(sfp, "Rx pending          : %lu\n",
		   handle->tp_acnt.rx_pending);
	seq_printf(sfp, "Rx pause            : %lu\n",
		   handle->tp_acnt.rx_paused_cnt);
exit:
	LEAVE();
	MODULE_PUT;
	return 0;
}

static int woal_info_proc_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	return single_open(file, woal_info_proc_read, PDE_DATA(inode));
#else
	return single_open(file, woal_info_proc_read, PDE(inode)->data);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops info_proc_fops = {
	.proc_open = woal_info_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations info_proc_fops = {
	.owner = THIS_MODULE,
	.open = woal_info_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

#ifdef SDIO
#define CMD52_STR_LEN 50
/*
 *  @brief Parse cmd52 string
 *
 *  @param buffer   A pointer user buffer
 *  @param len      Length user buffer
 *  @param func     Parsed func number
 *  @param reg      Parsed reg value
 *  @param val      Parsed value to set
 *  @return         BT_STATUS_SUCCESS
 */
static int parse_cmd52_string(const char *buffer, size_t len, int *func,
			      int *reg, int *val)
{
	int ret = MLAN_STATUS_SUCCESS;
	char *string = NULL;
	char *pos = NULL;
	gfp_t flag;

	ENTER();
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	string = kzalloc(CMD52_STR_LEN, flag);
	if (string == NULL)
		return -ENOMEM;

	moal_memcpy_ext(NULL, string, buffer + strlen("sdcmd52rw="),
			len - strlen("sdcmd52rw="), CMD52_STR_LEN - 1);
	string = strstrip(string);

	*func = -1;
	*reg = -1;
	*val = -1;

	/* Get func */
	pos = strsep(&string, " \t");
	if (pos)
		*func = woal_string_to_number(pos);

	/* Get reg */
	pos = strsep(&string, " \t");
	if (pos)
		*reg = woal_string_to_number(pos);

	/* Get val (optional) */
	pos = strsep(&string, " \t");
	if (pos)
		*val = woal_string_to_number(pos);
	kfree(string);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief config proc write function
 *
 *  @param f        file pointer
 *  @param buf      pointer to data buffer
 *  @param count    data number to write
 *  @param off      Offset
 *
 *  @return         number of data
 */
static ssize_t woal_config_write(struct file *f, const char __user *buf,
				 size_t count, loff_t *off)
{
	char databuf[101];
	char *line = NULL;
	t_u32 config_data = 0;
	struct seq_file *sfp = f->private_data;
	moal_handle *handle = (moal_handle *)sfp->private;

#ifdef SDIO
	int func = 0, reg = 0, val = 0;
#endif
	moal_handle *ref_handle = NULL;
	t_u32 cmd = 0;
	int copy_len;
	moal_private *priv = NULL;

	ENTER();
	if (!MODULE_GET) {
		LEAVE();
		return 0;
	}

	if (count >= sizeof(databuf)) {
		MODULE_PUT;
		LEAVE();
		return (int)count;
	}
	memset(databuf, 0, sizeof(databuf));
	copy_len = MIN((sizeof(databuf) - 1), count);
	if (copy_from_user(databuf, buf, copy_len)) {
		MODULE_PUT;
		LEAVE();
		return 0;
	}
	line = databuf;
	if (!strncmp(databuf, "soft_reset", strlen("soft_reset"))) {
		line += strlen("soft_reset") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		PRINTM(MINFO, "soft_reset: %d\n", (int)config_data);
		if (woal_request_soft_reset(handle) == MLAN_STATUS_SUCCESS)
			handle->hardware_status = HardwareStatusReset;
		else
			PRINTM(MERROR, "Could not perform soft reset\n");
	}
	if (!strncmp(databuf, "drv_mode", strlen("drv_mode"))) {
		line += strlen("drv_mode") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		PRINTM(MINFO, "drv_mode: %d\n", (int)config_data);
		if (config_data != (t_u32)handle->params.drv_mode)
			if (woal_switch_drv_mode(handle, config_data) !=
			    MLAN_STATUS_SUCCESS) {
				PRINTM(MERROR, "Could not switch drv mode\n");
			}
	}
#ifdef SDIO
	if (IS_SD(handle->card_type)) {
		if (!strncmp(databuf, "sdcmd52rw=", strlen("sdcmd52rw=")) &&
		    count > strlen("sdcmd52rw=")) {
			parse_cmd52_string((const char *)databuf, (size_t)count,
					   &func, &reg, &val);
			woal_sdio_read_write_cmd52(handle, func, reg, val);
		}
	}
#endif /* SD */
	if (!strncmp(databuf, "debug_dump", strlen("debug_dump"))) {
		ref_handle = (moal_handle *)handle->pref_mac;
		if (ref_handle) {
			priv = woal_get_priv(ref_handle, MLAN_BSS_ROLE_ANY);
			if (priv) {
#ifdef DEBUG_LEVEL1
				drvdbg &= ~MFW_D;
#endif
				woal_mlan_debug_info(priv);
				woal_moal_debug_info(priv, NULL, MFALSE);
			}
		}
		priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
		if (priv) {
			PRINTM(MERROR, "Recevie debug_dump command\n");
#ifdef DEBUG_LEVEL1
			drvdbg &= ~MFW_D;
#endif
			woal_mlan_debug_info(priv);
			woal_moal_debug_info(priv, NULL, MFALSE);
			handle->ops.dump_fw_info(handle);
		}
	}

	if (!strncmp(databuf, "fwdump_file=", strlen("fwdump_file="))) {
		int len = copy_len - strlen("fwdump_file=");
		gfp_t flag;
		if (len) {
			kfree(handle->fwdump_fname);
			flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC :
								  GFP_KERNEL;
			handle->fwdump_fname = kzalloc(len, flag);
			if (handle->fwdump_fname)
				moal_memcpy_ext(handle, handle->fwdump_fname,
						databuf +
							strlen("fwdump_file="),
						len - 1, len - 1);
		}
	}
	if (!strncmp(databuf, "fw_reload", strlen("fw_reload"))) {
		if (!strncmp(databuf, "fw_reload=", strlen("fw_reload="))) {
			line += strlen("fw_reload") + 1;
			config_data = (t_u32)woal_string_to_number(line);
		}
#ifdef SDIO_MMC
		else if (IS_SD(handle->card_type))
			config_data = FW_RELOAD_SDIO_INBAND_RESET;
#endif
		PRINTM(MMSG, "Request fw_reload=%d\n", config_data);
		woal_request_fw_reload(handle, config_data);
	}
	if (!strncmp(databuf, "drop_point=", strlen("drop_point="))) {
		line += strlen("drop_point") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		if (config_data) {
			handle->tp_acnt.on = 1;
			handle->tp_acnt.drop_point = config_data;
			if (handle->is_tp_acnt_timer_set == MFALSE) {
				woal_initialize_timer(&handle->tp_acnt.timer,
						      woal_tp_acnt_timer_func,
						      handle);
				handle->is_tp_acnt_timer_set = MTRUE;
				woal_mod_timer(&handle->tp_acnt.timer, 1000);
			}
		} else {
			if (handle->is_tp_acnt_timer_set) {
				woal_cancel_timer(&handle->tp_acnt.timer);
				handle->is_tp_acnt_timer_set = MFALSE;
			}
			memset((void *)&handle->tp_acnt, 0,
			       sizeof(moal_tp_acnt_t));
		}
		priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
		if (priv)
			woal_set_tp_state(priv);
		PRINTM(MMSG, "on=%d drop_point=%d\n", handle->tp_acnt.on,
		       handle->tp_acnt.drop_point);
	}
	if (!strncmp(databuf, "rf_test_mode", strlen("rf_test_mode"))) {
		line += strlen("rf_test_mode") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		PRINTM(MINFO, "RF test mode: %d\n", (int)config_data);
		if (config_data != (t_u32)handle->rf_test_mode)
			if (woal_process_rf_test_mode(handle, config_data) !=
			    MLAN_STATUS_SUCCESS)
				PRINTM(MERROR, "Could not set RF test mode\n");
	}
	if (!strncmp(databuf, "tx_antenna", strlen("tx_antenna"))) {
		line += strlen("tx_antenna") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		cmd = MFG_CMD_TX_ANT;
	}
	if (!strncmp(databuf, "rx_antenna", strlen("rx_antenna"))) {
		line += strlen("rx_antenna") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		cmd = MFG_CMD_RX_ANT;
	}
	if (!strncmp(databuf, "channel", strlen("channel"))) {
		line += strlen("channel") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		cmd = MFG_CMD_RF_CHAN;
	}
	if (!strncmp(databuf, "band", strlen("band"))) {
		line += strlen("band") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		cmd = MFG_CMD_RF_BAND_AG;
	}
	if (!strncmp(databuf, "bw", strlen("bw"))) {
		line += strlen("bw") + 1;
		config_data = (t_u32)woal_string_to_number(line);
		cmd = MFG_CMD_RF_CHANNELBW;
	}
	if (!strncmp(databuf, "get_and_reset_per", strlen("get_and_reset_per")))
		cmd = MFG_CMD_CLR_RX_ERR;
	if (!strncmp(databuf, "tx_power=", strlen("tx_power=")) &&
	    count > strlen("tx_power="))
		cmd = MFG_CMD_RFPWR;
	if (!strncmp(databuf, "tx_frame=", strlen("tx_frame=")) &&
	    count > strlen("tx_frame="))
		cmd = MFG_CMD_TX_FRAME;
	if (!strncmp(databuf, "tx_continuous=", strlen("tx_continuous=")) &&
	    count > strlen("tx_continuous="))
		cmd = MFG_CMD_TX_CONT;

	if (cmd && handle->rf_test_mode &&
	    (woal_process_rf_test_mode_cmd(
		     handle, cmd, (const char *)databuf, (size_t)count,
		     MLAN_ACT_SET, config_data) != MLAN_STATUS_SUCCESS)) {
		PRINTM(MERROR, "RF test mode cmd error\n");
	}
	if (cmd && !handle->rf_test_mode)
		PRINTM(MERROR, "RF test mode is disabled\n");
	MODULE_PUT;
	LEAVE();
	return (int)count;
}

/**
 *  @brief config proc read function
 *
 *  @param sfp      pointer to seq_file structure
 *  @param data
 *
 *  @return         number of output data
 */
static int woal_config_read(struct seq_file *sfp, void *data)
{
	moal_handle *handle = (moal_handle *)sfp->private;
	int i;

	ENTER();

	if (!MODULE_GET) {
		LEAVE();
		return 0;
	}

	seq_printf(sfp, "hardware_status=%d\n", (int)handle->hardware_status);
	seq_printf(sfp, "netlink_num=%d\n", (int)handle->netlink_num);
	seq_printf(sfp, "drv_mode=%d\n", (int)handle->params.drv_mode);
#ifdef SDIO
	if (IS_SD(handle->card_type)) {
		seq_printf(sfp, "sdcmd52rw=%d 0x%0x 0x%02X\n",
			   handle->cmd52_func, handle->cmd52_reg,
			   handle->cmd52_val);
	}
#endif /* SD */
	seq_printf(sfp, "rf_test_mode=%u\n", handle->rf_test_mode);
	if (handle->rf_test_mode && handle->rf_data) {
		seq_printf(sfp, "tx_antenna=%u\n", handle->rf_data->tx_antenna);
		seq_printf(sfp, "rx_antenna=%u\n", handle->rf_data->rx_antenna);
		seq_printf(sfp, "band=%u\n", handle->rf_data->band);
		seq_printf(sfp, "bw=%u\n", handle->rf_data->bandwidth);
		if (handle->rf_data->channel)
			seq_printf(sfp, "channel=%u\n",
				   handle->rf_data->channel);
		else
			seq_printf(sfp, "channel=\n");
		seq_printf(sfp, "total rx pkt count=%u\n",
			   handle->rf_data->rx_tot_pkt_count);
		seq_printf(sfp, "rx multicast/broadcast pkt count=%u\n",
			   handle->rf_data->rx_mcast_bcast_pkt_count);
		seq_printf(sfp, "rx fcs error pkt count=%u\n",
			   handle->rf_data->rx_pkt_fcs_err_count);
		if (handle->rf_data->tx_power_data[0]) {
			seq_printf(sfp, "tx_power=%u",
				   handle->rf_data->tx_power_data[0]);
			seq_printf(sfp, " %u",
				   handle->rf_data->tx_power_data[1]);
			seq_printf(sfp, " %u\n",
				   handle->rf_data->tx_power_data[2]);
		} else
			seq_printf(sfp, "tx_power=\n");
		seq_printf(sfp, "tx_continuous=%u",
			   handle->rf_data->tx_cont_data[0]);
		if (handle->rf_data->tx_cont_data[0] == MTRUE) {
			seq_printf(sfp, " %u",
				   handle->rf_data->tx_cont_data[1]);
			seq_printf(sfp, " 0x%x",
				   handle->rf_data->tx_cont_data[2]);
			for (i = 3; i < 6; i++)
				seq_printf(sfp, " %u",
					   handle->rf_data->tx_cont_data[i]);
		}
		seq_printf(sfp, "\n");
		seq_printf(sfp, "tx_frame=%u",
			   handle->rf_data->tx_frame_data[0]);
		if (handle->rf_data->tx_frame_data[0] == MTRUE) {
			seq_printf(sfp, " %u",
				   handle->rf_data->tx_frame_data[1]);
			seq_printf(sfp, " 0x%x",
				   handle->rf_data->tx_frame_data[2]);
			for (i = 3; i < 13; i++)
				seq_printf(sfp, " %u",
					   handle->rf_data->tx_frame_data[i]);
			seq_printf(sfp, " %02x:%02x:%02x:%02x:%02x:%02x",
				   handle->rf_data->bssid[0],
				   handle->rf_data->bssid[1],
				   handle->rf_data->bssid[2],
				   handle->rf_data->bssid[3],
				   handle->rf_data->bssid[4],
				   handle->rf_data->bssid[5]);
		}
		seq_printf(sfp, "\n");
	}
	MODULE_PUT;
	LEAVE();
	return 0;
}

static int woal_config_proc_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	return single_open(file, woal_config_read, PDE_DATA(inode));
#else
	return single_open(file, woal_config_read, PDE(inode)->data);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops config_proc_fops = {
	.proc_open = woal_config_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = woal_config_write,
};
#else
static const struct file_operations config_proc_fops = {
	.owner = THIS_MODULE,
	.open = woal_config_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = woal_config_write,
};
#endif

/**
 *  @brief wifi status proc read function
 *
 *  @param sfp      pointer to seq_file structure
 *  @param data
 *
 *  @return         number of output data
 */
static int woal_wifi_status_read(struct seq_file *sfp, void *data)
{
	ENTER();

	if (!MODULE_GET) {
		LEAVE();
		return 0;
	}

	seq_printf(sfp, "%d\n", wifi_status);

	MODULE_PUT;
	LEAVE();
	return 0;
}

static int woal_wifi_status_proc_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	return single_open(file, woal_wifi_status_read, PDE_DATA(inode));
#else
	return single_open(file, woal_wifi_status_read, PDE(inode)->data);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops wifi_status_proc_fops = {
	.proc_open = woal_wifi_status_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations wifi_status_proc_fops = {
	.owner = THIS_MODULE,
	.open = woal_wifi_status_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

/********************************************************
		Global Functions
********************************************************/
/**
 *  @brief Convert string to number
 *
 *  @param s        Pointer to numbered string
 *
 *  @return         Converted number from string s
 */
int woal_string_to_number(char *s)
{
	int r = 0;
	int base = 0;
	int pn = 1;

	if (!strncmp(s, "-", 1)) {
		pn = -1;
		s++;
	}
	if (!strncmp(s, "0x", 2) || !strncmp(s, "0X", 2)) {
		base = 16;
		s += 2;
	} else
		base = 10;

	for (s = s; *s; s++) {
		if ((*s >= '0') && (*s <= '9'))
			r = (r * base) + (*s - '0');
		else if ((*s >= 'A') && (*s <= 'F'))
			r = (r * base) + (*s - 'A' + 10);
		else if ((*s >= 'a') && (*s <= 'f'))
			r = (r * base) + (*s - 'a' + 10);
		else
			break;
	}

	return r * pn;
}

/**
 *  @brief This function creates proc mwlan directory
 *  directory structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_root_proc_init(void)
{
	ENTER();

	PRINTM(MINFO, "Create /proc/mwlan directory\n");

	proc_mwlan = proc_mkdir(MWLAN_PROC, PROC_DIR);
	if (!proc_mwlan) {
		PRINTM(MERROR,
		       "woal_root_proc_init: Cannot create /proc/mwlan\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* create /proc/mwlan/wifi_status */
	proc_create_data(STATUS_PROC, 0644, proc_mwlan, &wifi_status_proc_fops,
			 NULL);

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function removes proc mwlan directory
 *  directory structure
 *
 *  @return         N/A
 */
void woal_root_proc_remove(void)
{
	ENTER();

	remove_proc_entry(STATUS_PROC, proc_mwlan);

	remove_proc_entry(MWLAN_PROC, PROC_DIR);
	proc_mwlan = NULL;

	LEAVE();
}

/**
 *  @brief Create the top level proc directory
 *
 *  @param handle   Pointer to woal_handle
 *
 *  @return         N/A
 */
void woal_proc_init(moal_handle *handle)
{
	struct proc_dir_entry *r;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	struct proc_dir_entry *pde = proc_mwlan;
#endif
	char config_proc_dir[20];

	ENTER();

	if (handle->proc_wlan) {
		PRINTM(MMSG, "woal_proc_init: proc_wlan is already exist %s\n",
		       handle->proc_wlan_name);
		goto done;
	}

	snprintf(handle->proc_wlan_name, sizeof(handle->proc_wlan_name),
		 WLAN_PROC, handle->handle_idx);
	PRINTM(MINFO, "Create Proc Interface %s\n", handle->proc_wlan_name);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	/* Check if directory already exists */
	for (pde = pde->subdir; pde; pde = pde->next) {
		if (pde->namelen &&
		    !strcmp(handle->proc_wlan_name, pde->name)) {
			/* Directory exists */
			PRINTM(MWARN, "proc interface already exists!\n");
			handle->proc_wlan = pde;
			break;
		}
	}
	if (pde == NULL) {
		handle->proc_wlan =
			proc_mkdir(handle->proc_wlan_name, proc_mwlan);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
		if (handle->proc_wlan)
			atomic_set(&handle->proc_wlan->count, 1);
#endif
	}
#else
	handle->proc_wlan = proc_mkdir(handle->proc_wlan_name, proc_mwlan);
#endif
	if (!handle->proc_wlan) {
		PRINTM(MERROR, "Cannot create proc interface %s!\n",
		       handle->proc_wlan_name);
		goto done;
	}

	strcpy(config_proc_dir, "config");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	r = proc_create_data(config_proc_dir, 0644, handle->proc_wlan,
			     &config_proc_fops, handle);
#else
	r = create_proc_entry(config_proc_dir, 0644, handle->proc_wlan);
	if (r) {
		r->data = handle;
		r->proc_fops = &config_proc_fops;
	}
#endif
	if (!r)
		PRINTM(MERROR, "Fail to create proc config\n");

done:
	LEAVE();
}

/**
 *  @brief Remove the top level proc directory
 *
 *  @param handle   pointer moal_handle
 *
 *  @return         N/A
 */
void woal_proc_exit(moal_handle *handle)
{
	char config_proc_dir[20];

	ENTER();

	PRINTM(MINFO, "Remove Proc Interface %s\n", handle->proc_wlan_name);
	if (handle->proc_wlan) {
		strcpy(config_proc_dir, "config");
		remove_proc_entry(config_proc_dir, handle->proc_wlan);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
		/* Remove only if we are the only instance using this */
		if (atomic_read(&(handle->proc_wlan->count)) > 1) {
			PRINTM(MWARN, "More than one interface using proc!\n");
		} else {
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
			atomic_dec(&(handle->proc_wlan->count));
#endif
			remove_proc_entry(handle->proc_wlan_name, proc_mwlan);

			handle->proc_wlan = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
		}
#endif
	}
	LEAVE();
}

/**
 *  @brief Create proc file for interface
 *
 *  @param priv     pointer moal_private
 *
 *  @return         N/A
 */
void woal_create_proc_entry(moal_private *priv)
{
	struct proc_dir_entry *r;
	struct net_device *dev = priv->netdev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	char proc_dir_name[22];
#endif

	ENTER();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	if (!priv->proc_entry) {
		memset(proc_dir_name, 0, sizeof(proc_dir_name));
		strncpy(proc_dir_name, priv->phandle->proc_wlan_name,
			sizeof(proc_dir_name) - 2);
		proc_dir_name[strlen(proc_dir_name)] = '/';

		if (strlen(dev->name) >
		    ((sizeof(proc_dir_name) - 1) - (strlen(proc_dir_name)))) {
			PRINTM(MERROR,
			       "Failed to create proc entry, device name is too long\n");
			LEAVE();
			return;
		}
		strcat(proc_dir_name, dev->name);
		/* Try to create adapterX/dev_name directory first under
		 * /proc/mwlan/ */
		priv->proc_entry = proc_mkdir(proc_dir_name, proc_mwlan);
		if (priv->proc_entry) {
			/* Success. Continue normally */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
			if (!priv->phandle->proc_wlan) {
				priv->phandle->proc_wlan =
					priv->proc_entry->parent;
			}
			atomic_inc(&(priv->phandle->proc_wlan->count));
#endif
		} else {
			/* Failure. adapterX/ may not exist. Try to create that
			 * first */
			priv->phandle->proc_wlan = proc_mkdir(
				priv->phandle->proc_wlan_name, proc_mwlan);
			if (!priv->phandle->proc_wlan) {
				/* Failure. Something broken */
				LEAVE();
				return;
			} else {
				/* Success. Now retry creating mlanX */
				priv->proc_entry =
					proc_mkdir(proc_dir_name, proc_mwlan);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
				atomic_inc(&(priv->phandle->proc_wlan->count));
#endif
			}
		}
#else
	if (priv->phandle->proc_wlan && !priv->proc_entry) {
		priv->proc_entry =
			proc_mkdir(dev->name, priv->phandle->proc_wlan);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
		atomic_inc(&(priv->phandle->proc_wlan->count));
#endif /* < 3.10.0 */
#endif /* < 2.6.26 */
		strcpy(priv->proc_entry_name, dev->name);
		if (priv->proc_entry) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
			r = proc_create_data("info", 0, priv->proc_entry,
					     &info_proc_fops, dev);
#else
			r = create_proc_entry("info", 0, priv->proc_entry);
			if (r) {
				r->data = dev;
				r->proc_fops = &info_proc_fops;
			}
#endif
			if (!r)
				PRINTM(MMSG, "Fail to create proc info\n");
		}
	}

	LEAVE();
}

/**
 *  @brief Remove proc file
 *
 *  @param priv     Pointer moal_private
 *
 *  @return         N/A
 */
void woal_proc_remove(moal_private *priv)
{
	ENTER();
	if (priv->phandle->proc_wlan && priv->proc_entry) {
		remove_proc_entry("info", priv->proc_entry);
		remove_proc_entry(priv->proc_entry_name,
				  priv->phandle->proc_wlan);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
		atomic_dec(&(priv->phandle->proc_wlan->count));
#endif
		priv->proc_entry = NULL;
	}
	LEAVE();
}
#endif
