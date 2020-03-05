/** @file moal_cfgvendor.c
  *
  * @brief This file contains the functions for CFG80211 vendor.
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

#include    "moal_cfgvendor.h"
#include    "moal_cfg80211.h"

/********************************************************
				Local Variables
********************************************************/

/********************************************************
				Global Variables
********************************************************/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
extern int dfs_offload;
#endif
extern int roamoffload_in_hs;
extern int disable_regd_by_driver;
/********************************************************
				Local Functions
********************************************************/

/********************************************************
				Global Functions
********************************************************/

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
/**nxp vendor command and event*/
#define MRVL_VENDOR_ID  0x005043
/** vendor events */
const struct nl80211_vendor_cmd_info vendor_events[] = {
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_hang,},	/*event_id 0 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_rssi_monitor,},	/*event_id 0x1501 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_set_key_mgmt_offload,},	/*event_id 0x10001 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_fw_roam_success,},	/*event_id 0x10002 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_dfs_radar_detected,},	/*event_id 0x10004 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_dfs_cac_started,},	/*event_id 0x10005 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_dfs_cac_finished,},	/*event_id 0x10006 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_dfs_cac_aborted,},	/*event_id 0x10007 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_dfs_nop_finished,},	/*event_id 0x10008 */
	{.vendor_id = MRVL_VENDOR_ID,.subcmd =
	 event_wifi_logger_ring_buffer_data,},
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_wifi_logger_alert,},
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_packet_fate_monitor,},
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_wake_reason_report,},
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_nan_cb,},
	/**add vendor event here*/
	{.vendor_id = MRVL_VENDOR_ID,.subcmd = event_rtt_result,},	/*event_id ??? */
};

/**
 * @brief get the event id of the events array
 *
 * @param event     vendor event
 *
 * @return    index of events array
 */
int
woal_get_event_id(int event)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(vendor_events); i++) {
		if (vendor_events[i].subcmd == event)
			return i;
	}

	return event_max;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv       A pointer to moal_private
 * @param event    vendor event
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
int
woal_cfg80211_vendor_event(IN moal_private *priv,
			   IN int event, IN t_u8 *data, IN int len)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_id = 0;
	t_u8 *pos = NULL;
	int ret = 0;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		LEAVE();
		return ret;
	}
	wiphy = priv->wdev->wiphy;
	PRINTM(MEVENT, "vendor event :0x%x\n", event);
	event_id = woal_get_event_id(event);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = 1;
		LEAVE();
		return ret;
	}

	/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev, len, event_id,
					  GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy, len, event_id, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = 1;
		LEAVE();
		return ret;
	}
	pos = skb_put(skb, len);
	memcpy(pos, data, len);
	/**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

	LEAVE();
	return ret;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv       A pointer to moal_private
 * @param event    vendor event
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
struct sk_buff *
woal_cfg80211_alloc_vendor_event(IN moal_private *priv,
				 IN int event, IN int len)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_id = 0;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		goto done;
	}
	wiphy = priv->wdev->wiphy;
	PRINTM(MEVENT, "vendor event :0x%x\n", event);
	event_id = woal_get_event_id(event);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		goto done;
	}

	/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev, len, event_id,
					  GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy, len, event_id, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		goto done;
	}

done:
	LEAVE();
	return skb;
}

/**
 * @brief send dfs vendor event to kernel
 *
 * @param priv       A pointer to moal_private
 * @param event      dfs vendor event
 * @param chandef    a pointer to struct cfg80211_chan_def
 *
 * @return      N/A
 */
void
woal_cfg80211_dfs_vendor_event(moal_private *priv, int event,
			       struct cfg80211_chan_def *chandef)
{
	dfs_event evt;
	ENTER();
	if (!chandef) {
		LEAVE();
		return;
	}
	memset(&evt, 0, sizeof(dfs_event));
	evt.freq = chandef->chan->center_freq;
	evt.chan_width = chandef->width;
	evt.cf1 = chandef->center_freq1;
	evt.cf2 = chandef->center_freq2;
	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
		evt.ht_enabled = 0;
		break;
	case NL80211_CHAN_WIDTH_20:
		evt.ht_enabled = 1;
		break;
	case NL80211_CHAN_WIDTH_40:
		evt.ht_enabled = 1;
		if (chandef->center_freq1 < chandef->chan->center_freq)
			evt.chan_offset = -1;
		else
			evt.chan_offset = 1;
		break;
	case NL80211_CHAN_WIDTH_80:
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_160:
		evt.ht_enabled = 1;
		break;
	default:
		break;
	}
	woal_cfg80211_vendor_event(priv, event, (t_u8 *)&evt,
				   sizeof(dfs_event));
	LEAVE();
	return;
}

/**
 * @brief vendor command to set drvdbg
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_set_drvdbg(struct wiphy *wiphy,
				struct wireless_dev *wdev,
				const void *data, int data_len)
{
#ifdef DEBUG_LEVEL1
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u8 *pos = NULL;
#endif
	int ret = 1;

	ENTER();
#ifdef DEBUG_LEVEL1
	/**handle this sub command*/
	DBG_HEXDUMP(MCMD_D, "Vendor drvdbg", (t_u8 *)data, data_len);

	if (data_len) {
		/* Get the driver debug bit masks from user */
		drvdbg = *((t_u32 *)data);
		PRINTM(MIOCTL, "new drvdbg %x\n", drvdbg);
		/* Set the driver debug bit masks into mlan */
		if (woal_set_drvdbg(priv, drvdbg)) {
			PRINTM(MERROR, "Set drvdbg failed!\n");
			ret = 1;
		}
	}
	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(drvdbg));
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = 1;
		LEAVE();
		return ret;
	}
	pos = skb_put(skb, sizeof(drvdbg));
	memcpy(pos, &drvdbg, sizeof(drvdbg));
	ret = cfg80211_vendor_cmd_reply(skb);
#endif
	LEAVE();
	return ret;
}

/**
 * @brief process one channel in bucket
 *
 * @param priv       A pointer to moal_private struct
 *
 * @param channel     a pointer to channel
 *
 * @return      0: success  other: fail
 */
static mlan_status
woal_band_to_valid_channels(moal_private *priv, wifi_band w_band, int channel[],
			    t_u32 *nchannel)
{
	int band = 0;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	int i = 0;
	t_u8 cnt = 0;
	int *ch_ptr = channel;

	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		if (!priv->wdev->wiphy->bands[band])
			continue;
		if ((band == IEEE80211_BAND_2GHZ) && !(w_band & WIFI_BAND_BG))
			continue;
		if ((band == IEEE80211_BAND_5GHZ) &&
		    !((w_band & WIFI_BAND_A) || (w_band & WIFI_BAND_A_DFS)))
			continue;
		sband = priv->wdev->wiphy->bands[band];
		for (i = 0; (i < sband->n_channels); i++) {
			ch = &sband->channels[i];
			if (ch->flags & IEEE80211_CHAN_DISABLED) {
				PRINTM(MERROR, "Skip DISABLED channel %d\n",
				       ch->center_freq);
				continue;
			}
			if ((band == IEEE80211_BAND_5GHZ)) {
				if (((ch->flags & IEEE80211_CHAN_RADAR) &&
				     !(w_band & WIFI_BAND_A_DFS)) ||
				    (!(ch->flags & IEEE80211_CHAN_RADAR) &&
				     !(w_band & WIFI_BAND_A)))
					continue;
			}
			if (cnt >= *nchannel) {
				PRINTM(MERROR,
				       "cnt=%d is exceed %d, cur ch=%d %dMHz\n",
				       cnt, *nchannel, ch->hw_value,
				       ch->center_freq);
				break;
			}
			*ch_ptr = ch->center_freq;
			ch_ptr++;
			cnt++;
		}
	}

	PRINTM(MCMND, "w_band=%d cnt=%d\n", w_band, cnt);
	*nchannel = cnt;
	return MLAN_STATUS_SUCCESS;
}

/**
 * @brief GSCAN subcmd - enable full scan results
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 *
 * @param data     a pointer to data
 * @param  data_len     data length
 *
 * @return      0: success  other: fail
 */
static int
woal_cfg80211_subcmd_get_valid_channels(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct nlattr *tb[ATTR_WIFI_MAX];
	t_u32 band = 0;
	int ch_out[MAX_CHANNEL_NUM];
	t_u32 nchannel = 0;
	t_u32 mem_needed = 0;
	struct sk_buff *skb = NULL;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_get_valid_channels\n");

	err = nla_parse(tb, ATTR_WIFI_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

	if (!tb[ATTR_CHANNELS_BAND]) {
		PRINTM(MERROR, "%s: null attr: tb[ATTR_GET_CH]=%p\n",
		       __FUNCTION__, tb[ATTR_CHANNELS_BAND]);
		err = -EINVAL;
		goto done;
	}
	band = nla_get_u32(tb[ATTR_CHANNELS_BAND]);
	if (band > WIFI_BAND_MAX) {
		PRINTM(MERROR, "%s: invalid band=%d\n", __FUNCTION__, band);
		err = -EINVAL;
		goto done;
	}

	memset(ch_out, 0x00, sizeof(ch_out));
	nchannel = MAX_CHANNEL_NUM;
	if (woal_band_to_valid_channels(priv, band, ch_out, &nchannel) !=
	    MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR,
		       "get_channel_list: woal_band_to_valid_channels fail\n");
		return -EFAULT;
	}

	mem_needed =
		nla_total_size(nchannel * sizeof(ch_out[0])) +
		nla_total_size(sizeof(nchannel))
		+ VENDOR_REPLY_OVERHEAD;
	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, mem_needed);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed");
		err = -ENOMEM;
		goto done;
	}

	nla_put_u32(skb, ATTR_NUM_CHANNELS, nchannel);
	nla_put(skb, ATTR_CHANNEL_LIST, nchannel * sizeof(ch_out[0]), ch_out);
	err = cfg80211_vendor_cmd_reply(skb);
	if (err) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
		goto done;
	}

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to get driver version
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_drv_version(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     const void *data, int data_len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;
	t_u32 drv_len = 0;
	char drv_version[MLAN_MAX_VER_STR_LEN] = { 0 };
	char *pos;

	ENTER();
	memcpy(drv_version, &priv->phandle->driver_version,
	       MLAN_MAX_VER_STR_LEN);
	drv_len = strlen(drv_version);
	pos = strstr(drv_version, "%s");
	/* remove 3 char "-%s" in driver_version string */
	if (pos != NULL)
		memcpy(pos, pos + 3, strlen(pos) - 3);

	reply_len = strlen(drv_version) + 1;
	drv_len -= 3;
	drv_version[drv_len] = '\0';

	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_NAME, reply_len,
		(t_u8 *)drv_version);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get firmware version
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_fw_version(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int data_len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	char end_c = '\0';
	int ret = 0;
	char fw_ver[32] = { 0 };
	union {
		t_u32 l;
		t_u8 c[4];
	} ver;

	ENTER();

	ver.l = priv->phandle->fw_release_number;
	snprintf(fw_ver, sizeof(fw_ver), "%u.%u.%u.p%u%c",
		 ver.c[2], ver.c[1], ver.c[0], ver.c[3], end_c);
	reply_len = strlen(fw_ver) + 1;

	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_NAME, reply_len, (t_u8 *)fw_ver);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get firmware memory dump
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_fw_dump(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 const void *data, int data_len)
{
	struct net_device *dev = NULL;
	moal_private *priv = NULL;
	moal_handle *handle = NULL;
	int ret = MLAN_STATUS_SUCCESS;
	int length = 0;
	struct sk_buff *skb = NULL;

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}
	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);
	handle = priv->phandle;
	if (handle->firmware_dump_file) {
		memset(handle->firmware_dump_file, 0,
		       sizeof(handle->firmware_dump_file));
	}
	length = sizeof(handle->firmware_dump_file);
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, length);
	if (!skb) {
		PRINTM(MERROR, "Failed to allocate memory for skb\n");
		ret = -ENOMEM;
		goto done;
	}
	nla_put(skb, ATTR_FW_DUMP_PATH, sizeof(handle->firmware_dump_file),
		handle->firmware_dump_file);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get driver memory dump
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_drv_dump(struct wiphy *wiphy,
				  struct wireless_dev *wdev,
				  const void *data, int data_len)
{
	struct net_device *dev = NULL;
	moal_private *priv = NULL;
	moal_handle *handle = NULL;
	int ret = MLAN_STATUS_SUCCESS;
	int length = 0;
	char driver_dump_file[128];
	struct sk_buff *skb = NULL;

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}
	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);
	handle = priv->phandle;
	memset(driver_dump_file, 0, sizeof(driver_dump_file));
	sprintf(driver_dump_file, "/proc/mwlan/");
	if (handle->handle_idx)
		sprintf(driver_dump_file, "drv_dump%d", handle->handle_idx);
	else
		sprintf(driver_dump_file, "drv_dump");
	PRINTM(MMSG, "driver dump file is %s\n", driver_dump_file);
	length = sizeof(driver_dump_file);
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, length);
	if (!skb) {
		PRINTM(MERROR, "Failed to allocate memory for skb\n");
		ret = -ENOMEM;
		goto done;
	}
	nla_put_string(skb, ATTR_DRV_DUMP_PATH, driver_dump_file);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get supported feature set
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_supp_feature_set(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int data_len)
{
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;
	t_u32 supp_feature_set = 0;
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_fw_info fw_info;
	ENTER();

	supp_feature_set = WLAN_FEATURE_INFRA
#if defined(UAP_SUPPORT) && defined(STA_SUPPORT)
		| WLAN_FEATURE_AP_STA
#endif
		| WLAN_FEATURE_LINK_LAYER_STATS
		| WLAN_FEATURE_LOGGER
		| WLAN_FEATURE_RSSI_MONITOR
		| WLAN_FEATURE_CONFIG_NDO
		| WLAN_FEATURE_CONTROL_ROAMING | WLAN_FEATURE_SCAN_RAND;

	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if (fw_info.fw_bands & BAND_A)
		supp_feature_set |= WLAN_FEATURE_INFRA_5G;

	reply_len = sizeof(supp_feature_set);
	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}
	nla_put_u32(skb, ATTR_FEATURE_SET, supp_feature_set);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to set country code
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_set_country_code(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int data_len)
{
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0, rem, type;
	const struct nlattr *iter;
	char country[COUNTRY_CODE_LEN] = { 0 };

	ENTER();

	nla_for_each_attr(iter, data, data_len, rem) {
		type = nla_type(iter);
		switch (type) {
		case ATTR_COUNTRY_CODE:
			strncpy(country, nla_data(iter),
				MIN(sizeof(country) - 1, nla_len(iter)));
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			return ret;
		}
	}

	if (!disable_regd_by_driver)
		regulatory_hint(wiphy, country);

	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get supported feature set
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_wifi_logger_supp_feature_set(struct wiphy *wiphy,
						      struct wireless_dev *wdev,
						      const void *data,
						      int data_len)
{
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;
	t_u32 supp_feature_set = 0;

	ENTER();

	supp_feature_set = WIFI_LOGGER_CONNECT_EVENT_SUPPORTED;
	reply_len = sizeof(supp_feature_set);
    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}
	nla_put_u32(skb, ATTR_WIFI_LOGGER_FEATURE_SET, supp_feature_set);
	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief get ring status
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_id    ring buffer id
 * @param status     a pointer to wifi_ring_buffer_status
 *
 * @return    void
 */
static void
woal_get_ring_status(moal_private *priv, int ring_id,
		     wifi_ring_buffer_status * status)
{
	int id = 0;
	wifi_ring_buffer *ring;

	ENTER();
	for (id = 0; id < RING_ID_MAX; id++) {
		ring = (wifi_ring_buffer *) priv->rings[id];
		if (ring && VALID_RING(ring->ring_id) &&
		    ring_id == ring->ring_id) {
			strncpy(status->name, ring->name,
				sizeof(status->name) - 1);
			status->ring_id = ring->ring_id;
			status->ring_buffer_byte_size = ring->ring_size;
			status->written_bytes = ring->ctrl.written_bytes;
			status->written_records = ring->ctrl.written_records;
			status->read_bytes = ring->ctrl.read_bytes;
			status->verbose_level = ring->log_level;
			PRINTM(MINFO,
			       "%s, name: %s, ring_id: %d, ring_size : %d, written_bytes: %d, written_records: "
			       "%d, read_bytes: %d \n",
			       __FUNCTION__, status->name, status->ring_id,
			       status->ring_buffer_byte_size,
			       status->written_bytes, status->written_records,
			       status->read_bytes);
			break;
		}
	}
	LEAVE();
	return;
}

/**
 * @brief vendor command to get ring buffer status
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_ring_buff_status(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int data_len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	t_u8 ringIdx, ring_cnt = 0;
	int ret = 0;
	wifi_ring_buffer_status status[RING_ID_MAX];
	wifi_ring_buffer_status ring_status;

	ENTER();
	reply_len =
		RING_ID_MAX * sizeof(wifi_ring_buffer_status) + sizeof(t_u32);
    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	/* shoud sync with HAL side to decide payload layout.
	   just ring buffer status or ring buffer num + ring buffer status
	   currently only have ring buffer status
	 */
	for (ringIdx = 0; ringIdx < RING_ID_MAX; ringIdx++) {
		memset(&ring_status, 0, sizeof(wifi_ring_buffer_status));
		woal_get_ring_status(priv, ringIdx, &ring_status);
		memcpy(&status[ring_cnt++], &ring_status,
		       sizeof(wifi_ring_buffer_status));
	}

	nla_put_u32(skb, ATTR_NUM_RINGS, ring_cnt);
	nla_put(skb, ATTR_RING_BUFFER_STATUS,
		sizeof(wifi_ring_buffer_status) * ring_cnt, status);

	ret = cfg80211_vendor_cmd_reply(skb);

	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief return ring_id based on ring_name
 *
 * @param priv          A pointer to moal_private struct
 * @param ring_name     A pointer to ring_name
 *
 * @return      An invalid ring id for failure or valid ring id on success.
 */
int
woal_get_ring_id_by_name(moal_private *priv, char *ring_name)
{
	int id;
	wifi_ring_buffer *ring;

	ENTER();

	for (id = 0; id < RING_ID_MAX; id++) {
		ring = (wifi_ring_buffer *) priv->rings[id];
		if (ring &&
		    !strncmp(ring->name, ring_name, sizeof(ring->name) - 1))
			break;
	}

	LEAVE();
	return id;
}

/**
 * @brief start logger
 *
 * @param priv          A pointer to moal_private struct
 * @param ring_name     string ring_name
 * @param log_level     log level to record
 * @param flags         reserved
 * @param time_intval   interval to report log to HAL
 * @param threshold     buffer threshold to report log to HAL
 *
 * @return      0: success  1: fail
 */
int
woal_start_logging(moal_private *priv, char *ring_name, int log_level,
		   int flags, int time_intval, int threshold)
{
	int ret = 0;
	int ring_id;
	wifi_ring_buffer *ring_buffer;
	unsigned long lock_flags;

	ENTER();

	ring_id = woal_get_ring_id_by_name(priv, ring_name);
	if (!VALID_RING(ring_id)) {
		ret = -EINVAL;
		goto done;
	}

	PRINTM(MCMND,
	       "%s , log_level : %d, time_intval : %d, threshod %d Bytes\n",
	       __FUNCTION__, log_level, time_intval, threshold);

	ring_buffer = (wifi_ring_buffer *) priv->rings[ring_id];
	if (!ring_buffer || ring_buffer->state == RING_STOP) {
		PRINTM(MERROR, "Ring is stopped!\n");
		ret = -EAGAIN;
		goto done;
	}

	spin_lock_irqsave(&ring_buffer->lock, lock_flags);
	ring_buffer->log_level = log_level;
	ring_buffer->threshold = threshold;
	if (log_level == 0)
		ring_buffer->state = RING_SUSPEND;
	else
		ring_buffer->state = RING_ACTIVE;
	ring_buffer->interval = msecs_to_jiffies(time_intval * MSEC_PER_SEC);
	spin_unlock_irqrestore(&ring_buffer->lock, lock_flags);

	if (log_level == 0) {
		cancel_delayed_work_sync(&ring_buffer->work);
	} else {
		if (ring_buffer->interval)
			schedule_delayed_work(&ring_buffer->work,
					      ring_buffer->interval);
	}

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to start logging
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_start_logging(struct wiphy *wiphy,
				   struct wireless_dev *wdev, const void *data,
				   int len)
{
	int ret = 0, rem, type;
	char ring_name[RING_NAME_MAX] = { 0 };
	int log_level = 0, flags = 0, time_intval = 0, threshold = 0;
	const struct nlattr *iter;
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case ATTR_WIFI_LOGGER_RING_ID:
			strncpy(ring_name, nla_data(iter),
				MIN(sizeof(ring_name) - 1, nla_len(iter)));
			break;
		case ATTR_WIFI_LOGGER_VERBOSE_LEVEL:
			log_level = nla_get_u32(iter);
			break;
		case ATTR_WIFI_LOGGER_FLAGS:
			flags = nla_get_u32(iter);
			break;
		case ATTR_WIFI_LOGGER_MAX_INTERVAL_SEC:
			time_intval = nla_get_u32(iter);
			break;
		case ATTR_WIFI_LOGGER_MIN_DATA_SIZE:
			threshold = nla_get_u32(iter);
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	ret = woal_start_logging(priv, ring_name, log_level, flags, time_intval,
				 threshold);
	if (ret < 0) {
		PRINTM(MERROR, "Start_logging is failed ret: %d\n", ret);
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief get log data in ring buffer
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_name  ring name string
 *
 * @return      0: success  1: fail
 */
static int
woal_trigger_get_ring_data(moal_private *priv, char *ring_name)
{
	int ret = 0;
	int ring_id;
	wifi_ring_buffer *ring_buffer;

	ENTER();

	ring_id = woal_get_ring_id_by_name(priv, ring_name);
	if (!VALID_RING(ring_id)) {
		PRINTM(MERROR, "invalid ring_id \n");
		ret = -EINVAL;
		goto done;
	}

	ring_buffer = (wifi_ring_buffer *) priv->rings[ring_id];
	if (!ring_buffer) {
		PRINTM(MERROR, "invalid ring_buffer \n");
		ret = -EINVAL;
		goto done;
	}
	if (ring_buffer->interval)
		cancel_delayed_work_sync(&ring_buffer->work);
	schedule_delayed_work(&ring_buffer->work, 0);

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get ring buffer data
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_ring_data(struct wiphy *wiphy,
				   struct wireless_dev *wdev, const void *data,
				   int len)
{
	int ret = 0, rem, type;
	char ring_name[RING_NAME_MAX] = { 0 };
	const struct nlattr *iter;
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case ATTR_WIFI_LOGGER_RING_ID:
			strncpy(ring_name, nla_data(iter),
				MIN(sizeof(ring_name) - 1, nla_len(iter)));
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	ret = woal_trigger_get_ring_data(priv, ring_name);
	if (ret < 0) {
		PRINTM(MERROR, "trigger_get_data failed ret:%d\n", ret);
	}

done:
	LEAVE();
	return ret;
}

/**
 * @brief ring reset
 *
 * @param priv       A pointer to moal_private struct
 * @param ring    A pointer to wifi_ring_buffer
 *
 * @return    void
 */
static void
woal_ring_reset(wifi_ring_buffer * ring)
{
	ENTER();
	ring->wp = 0;
	ring->rp = 0;
	memset(&(ring->ctrl), 0x00, sizeof(ring->ctrl));
	memset(ring->ring_buf, 0, ring->ring_size);

	LEAVE();
	return;
}

/**
 * @brief get ring buffer entry
 *
 * @param ring       A pointer to wifi_ring_buffer struct
 * @param offset     entry offset
 *
 * @return     A pointer to wifi_ring_buffer_entry struct
 */
static wifi_ring_buffer_entry *
woal_get_ring_entry(wifi_ring_buffer * ring, t_u32 offset)
{
	wifi_ring_buffer_entry *entry =
		(wifi_ring_buffer_entry *) (ring->ring_buf + offset);

	ENTER();

	if (!entry->entry_size)
		return (wifi_ring_buffer_entry *) ring->ring_buf;

	LEAVE();
	return entry;
}

/**
 * @brief get next ring buffer entry
 *
 * @param ring       A pointer to wifi_ring_buffer struct
 * @param offset     next entry offset
 *
 * @return     offset of next entry
 */
static t_u32
woal_get_ring_next_entry(wifi_ring_buffer * ring, t_u32 offset)
{
	wifi_ring_buffer_entry *entry =
		(wifi_ring_buffer_entry *) (ring->ring_buf + offset);
	wifi_ring_buffer_entry *next_entry = NULL;

	ENTER();

	if (!entry->entry_size) {
		entry = (wifi_ring_buffer_entry *) ring->ring_buf;
		LEAVE();
		return ENTRY_LENGTH(entry);
	}
	if ((offset + ENTRY_LENGTH(entry)) >= ring->ring_size) {
		/* move to head */
		LEAVE();
		return 0;
	}
	next_entry =
		(wifi_ring_buffer_entry *) (ring->ring_buf + offset +
					    ENTRY_LENGTH(entry));
	if (!next_entry->entry_size) {
		/* move to head */
		LEAVE();
		return 0;
	}
	LEAVE();
	return offset + ENTRY_LENGTH(entry);
}

/**
 * @brief prepare log data to send to HAL
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_id    ring ID
 * @param data       A pointer to data buffer
 * @param buf_len    log data length
 *
 * @return      data length
 */
int
woal_ring_pull_data(moal_private *priv, int ring_id, void *data, t_s32 buf_len)
{
	t_s32 r_len = 0;
	wifi_ring_buffer *ring;
	wifi_ring_buffer_entry *hdr;
	t_s32 buf_left = buf_len;

	ENTER();

	ring = (wifi_ring_buffer *) priv->rings[ring_id];

	/* get a fresh pending length */
	while (buf_left > 0) {
		hdr = woal_get_ring_entry(ring, ring->rp);
		if (buf_left < ENTRY_LENGTH(hdr))
			break;
		memcpy(data, hdr, ENTRY_LENGTH(hdr));
		r_len += ENTRY_LENGTH(hdr);
		data += ENTRY_LENGTH(hdr);
		buf_left -= ENTRY_LENGTH(hdr);
		ring->ctrl.read_bytes += ENTRY_LENGTH(hdr);
		if (!buf_left) {
			ring->rp += ENTRY_LENGTH(hdr);
			break;
		}
		ring->rp = woal_get_ring_next_entry(ring, ring->rp);
	}
	PRINTM(MDATA, "Ring pull data: [wp=%d rp=%d] r_len=%d buf_len=%d\n",
	       ring->wp, ring->rp, r_len, buf_len);

	LEAVE();
	return r_len;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv       A pointer to moal_private
 * @param event    vendor event
 * @param data     a pointer to data
 * @param  len     data length
 * @param ring_status    A pointer to wifi_ring_buffer status struct
 *
 * @return      0: success  1: fail
 */
int
woal_ring_buffer_data_vendor_event(IN moal_private *priv, IN int ring_id,
				   IN t_u8 *data, IN int len,
				   wifi_ring_buffer_status * ring_status)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_id = 0;
	int ret = 0;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "priv is null \n");
		ret = -EINVAL;
		goto done;
	}
	wiphy = priv->wdev->wiphy;
	PRINTM(MEVENT, "woal_ring_buffer_data_vendor_event ring_id:%d\n",
	       ring_id);
	event_id = woal_get_event_id(event_wifi_logger_ring_buffer_data);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = -EINVAL;
		goto done;
	}

/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev,
					  len + sizeof(wifi_ring_buffer_status),
					  event_id, GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy,
					  len + sizeof(wifi_ring_buffer_status),
					  event_id, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = -ENOMEM;
		goto done;
	}
	nla_put(skb, ATTR_RING_BUFFER_STATUS, sizeof(wifi_ring_buffer_status),
		ring_status);
	DBG_HEXDUMP(MEVT_D, "ring_buffer_data", data, len);
	nla_put(skb, ATTR_RING_BUFFER, len, data);

    /**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

done:
	LEAVE();
	return ret;
}

/**
 * @brief send log data to HAL
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_id    ring ID
 * @param data       A pointer to data buffer
 * @param len    log data length
 * @param ring_status    A pointer to wifi_ring_buffer status struct
 *
 * @return void
 */
static void
woal_ring_data_send(moal_private *priv, int ring_id, const void *data,
		    const t_u32 len, wifi_ring_buffer_status * ring_status)
{
	struct net_device *ndev = priv->netdev;
	ENTER();

	if (!ndev)
		goto done;
	woal_ring_buffer_data_vendor_event(priv, ring_id, (t_u8 *)data, len,
					   ring_status);

done:
	LEAVE();
	return;
}

/**
 * @brief main thread to send log data
 *
 * @param work       A pointer to work_struct struct
 *
 * @return void
 */
void
woal_ring_poll_worker(struct work_struct *work)
{
	struct delayed_work *d_work = to_delayed_work(work);
	wifi_ring_buffer *ring_info =
		container_of(d_work, wifi_ring_buffer, work);
	moal_private *priv = ring_info->priv;
	int ringid = ring_info->ring_id;
	wifi_ring_buffer_status ring_status;
	void *buf = NULL;
	wifi_ring_buffer_entry *hdr;
	t_s32 buflen, rlen;
	unsigned long flags;

	ENTER();
	if (ring_info->state != RING_ACTIVE) {
		PRINTM(MERROR, "Ring is not active!\n");
		goto exit;
	}
	buf = kmalloc(ring_info->ring_size, GFP_KERNEL);
	if (!buf) {
		PRINTM(MERROR, "Fail to allocate buffer for poll work\n");
		goto exit;
	}
	spin_lock_irqsave(&ring_info->lock, flags);
	memset(&ring_status, 0, sizeof(ring_status));
	woal_get_ring_status(priv, ringid, &ring_status);
	PRINTM(MDATA, "Ring work: ringid %d write %d, read %d, size %d\n",
	       ringid, ring_status.written_bytes, ring_status.read_bytes,
	       ring_status.ring_buffer_byte_size);
	if (ring_status.written_bytes > ring_status.read_bytes) {
		buflen = ring_status.written_bytes - ring_status.read_bytes;
		if (buflen > ring_info->ring_size) {
			PRINTM(MERROR, "Pending data bigger than ring size\n");
			spin_unlock_irqrestore(&ring_info->lock, flags);
			goto exit;
		}
	} else {
		spin_unlock_irqrestore(&ring_info->lock, flags);
		PRINTM(MINFO, "No new records\n");
		goto exit;
	}

	rlen = woal_ring_pull_data(priv, ringid, buf, buflen);
	spin_unlock_irqrestore(&ring_info->lock, flags);
	hdr = (wifi_ring_buffer_entry *) buf;
	while (rlen > 0) {
		ring_status.read_bytes += ENTRY_LENGTH(hdr);
		woal_ring_data_send(priv, ringid, hdr, ENTRY_LENGTH(hdr),
				    &ring_status);
		rlen -= ENTRY_LENGTH(hdr);
		hdr = (wifi_ring_buffer_entry *) ((void *)hdr +
						  ENTRY_LENGTH(hdr));
	}
exit:
	kfree(buf);
	if (ring_info->interval)
		schedule_delayed_work(&ring_info->work, ring_info->interval);

	LEAVE();
	return;
}

/**
 * @brief add log data to ring buffer
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_id    ring ID
 * @param hdr        A pointer to wifi_ring_buffer_entry struct
 * @param data       A pointer to data buffer
 *
 * @return      0: success  -1: fail
 */
int
woal_ring_push_data(moal_private *priv, int ring_id,
		    wifi_ring_buffer_entry * hdr, void *data)
{
	unsigned long flags;
	t_u32 w_len;
	wifi_ring_buffer *ring;
	wifi_ring_buffer_entry *w_entry;
	int ret = 0;

	ENTER();

	ring = (wifi_ring_buffer *) priv->rings[ring_id];

	if (!ring || ring->state != RING_ACTIVE) {
		PRINTM(MERROR, "Ring is not active\n");
		ret = -EINVAL;
		goto done;
	}

	w_len = ENTRY_LENGTH(hdr);
	if (w_len > ring->ring_size) {
		PRINTM(MERROR, "event size too big =%d\n", w_len);
		ret = -EINVAL;
		goto done;
	}
	spin_lock_irqsave(&ring->lock, flags);
	PRINTM(MDATA, "Ring push data: rp=%d  wp=%d, w_len=%d\n", ring->rp,
	       ring->wp, w_len);
	/* prep the space */
	do {
		if (ring->rp == ring->wp) {
			if (ring->ctrl.read_bytes == ring->ctrl.written_bytes) {
				if (ring->wp)
					woal_ring_reset(ring);
				break;
			} else {
				/* full, we should drop one entry */
				w_entry =
					(wifi_ring_buffer_entry *) (ring->
								    ring_buf +
								    ring->rp);
				ring->rp =
					woal_get_ring_next_entry(ring,
								 ring->rp);
				ring->ctrl.written_bytes -=
					ENTRY_LENGTH(w_entry);
				memset((u8 *)w_entry, 0, ENTRY_LENGTH(w_entry));
				ring->ctrl.written_records--;
				continue;
			}
		}
		if (ring->rp < ring->wp) {
			if (ring->ring_size - ring->wp >= w_len) {
				break;
			} else if (ring->ring_size - ring->wp < w_len) {
				if (ring->rp == 0) {
		    /** drop one entry */
					w_entry =
						(wifi_ring_buffer_entry
						 *) (ring->ring_buf + ring->rp);
					ring->rp =
						woal_get_ring_next_entry(ring,
									 ring->
									 rp);
					ring->ctrl.written_bytes -=
						ENTRY_LENGTH(w_entry);
					memset((u8 *)w_entry, 0,
					       ENTRY_LENGTH(w_entry));
					ring->ctrl.written_records--;
				}
				ring->wp = 0;
				continue;
			}
		}
		if (ring->rp > ring->wp) {
			if (ring->rp - ring->wp < w_len) {
		/** drop one entry */
				w_entry =
					(wifi_ring_buffer_entry *) (ring->
								    ring_buf +
								    ring->rp);
				ring->rp =
					woal_get_ring_next_entry(ring,
								 ring->rp);
				ring->ctrl.written_bytes -=
					ENTRY_LENGTH(w_entry);
				memset((u8 *)w_entry, 0, ENTRY_LENGTH(w_entry));
				ring->ctrl.written_records--;
				continue;
			} else {
				break;
			}
		}
	} while (1);

	if ((ring->wp + w_len) > ring->ring_size ||
	    (ring->ctrl.written_bytes + w_len) > ring->ring_size) {
		PRINTM(MERROR,
		       "Ring push buffer overflow: rp=%d  wp=%d, write_bytes=%d \n",
		       ring->rp, ring->wp, ring->ctrl.written_bytes);
		goto done;
	}

	w_entry = (wifi_ring_buffer_entry *) (ring->ring_buf + ring->wp);
	/* header */
	memcpy(w_entry, hdr, RING_ENTRY_SIZE);
	/* payload */
	memcpy((char *)w_entry + RING_ENTRY_SIZE, data, w_entry->entry_size);
	/* update write pointer */
	ring->wp += w_len;
	/* update statistics */
	ring->ctrl.written_records++;
	ring->ctrl.written_bytes += w_len;
	spin_unlock_irqrestore(&ring->lock, flags);

	PRINTM(MDATA,
	       "Ring push data: wp=%d rp=%d  write_bytes=%d, read_bytes=%d\n",
	       ring->wp, ring->rp, ring->ctrl.written_bytes,
	       ring->ctrl.read_bytes);

	/* if the current pending size is bigger than threshold */
	if (ring->threshold && (READ_AVAIL_SPACE(ring) >= ring->threshold)) {
		PRINTM(MDATA, "Ring threshold reached\n");
		if (ring->interval)
			cancel_delayed_work_sync(&ring->work);
		schedule_delayed_work(&ring->work, 0);
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function will init wifi logger ring buffer
 *
 *  @param priv          A pointer to moal_private structure
 *  @param ring_buff     A pointer to wifi_ring_buffer structure
 *  @param ringIdx       Ring buffer ID
 *  @param name          Ring buffer name
 *  @param ring_sz       Ring buffer size
 *
 *  @return        0 - success
 */
static int
woal_init_ring_buffer_internal(moal_private *priv, void **ring, t_u16 ringIdx,
			       t_u8 *name, t_u32 ring_sz)
{
	unsigned long flags;
	wifi_ring_buffer *ring_buff;
	ENTER();

	*ring = vmalloc(sizeof(wifi_ring_buffer));
	if (!unlikely(*ring)) {
		PRINTM(MERROR, "WiFi Logger: ring alloc failed\n");
		goto done;
	}
	memset(*ring, 0, sizeof(wifi_ring_buffer));
	ring_buff = (wifi_ring_buffer *) * ring;

	ring_buff->ring_buf = vmalloc(ring_sz);
	if (!unlikely(ring_buff->ring_buf)) {
		PRINTM(MERROR, "WiFi Logger: ring buffer data alloc failed\n");
		vfree(ring_buff);
		*ring = NULL;
		goto done;
	}
	memset(ring_buff->ring_buf, 0, ring_sz);
	spin_lock_init(&ring_buff->lock);
	spin_lock_irqsave(&ring_buff->lock, flags);
	ring_buff->rp = ring_buff->wp = 0;
	INIT_DELAYED_WORK(&ring_buff->work, woal_ring_poll_worker);
	memcpy(ring_buff->name, name, MIN(strlen(name), RING_NAME_MAX));
	ring_buff->name[RING_NAME_MAX - 1] = 0;
	ring_buff->ring_id = ringIdx;
	ring_buff->ring_size = ring_sz;
	ring_buff->state = RING_SUSPEND;
	ring_buff->threshold = ring_buff->ring_size / 2;
	ring_buff->priv = priv;
	spin_unlock_irqrestore(&ring_buff->lock, flags);
done:
	LEAVE();
	return 0;
}

/**
 * @brief init each ring in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      data length
 */
static int
woal_init_ring_buffer(moal_private *priv)
{
	ENTER();
	woal_init_ring_buffer_internal(priv, &priv->rings[VERBOSE_RING_ID],
				       VERBOSE_RING_ID, VERBOSE_RING_NAME,
				       DEFAULT_RING_BUFFER_SIZE);
	woal_init_ring_buffer_internal(priv, &priv->rings[EVENT_RING_ID],
				       EVENT_RING_ID, EVENT_RING_NAME,
				       DEFAULT_RING_BUFFER_SIZE);
	LEAVE();
	return 0;
}

/**
 * @brief deinit each ring in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      data length
 */
static int
woal_deinit_ring_buffer(moal_private *priv)
{
	int i;
	enum ring_state ring_state = RING_STOP;
	unsigned long lock_flags = 0;
	wifi_ring_buffer *ring_buff;
	ENTER();

	for (i = 0; i < RING_ID_MAX - 1; i++) {
		ring_buff = (wifi_ring_buffer *) priv->rings[i];
		if (!ring_buff)
			continue;
		spin_lock_irqsave(&ring_buff->lock, lock_flags);
		ring_state = ring_buff->state;
		if (ring_state == RING_ACTIVE) {
			ring_buff->state = RING_STOP;
		}
		spin_unlock_irqrestore(&ring_buff->lock, lock_flags);
		if (ring_state == RING_ACTIVE)
			cancel_delayed_work_sync(&ring_buff->work);
		vfree(ring_buff->ring_buf);
		ring_buff->ring_buf = NULL;
		vfree(ring_buff);
		priv->rings[i] = NULL;
	}

	LEAVE();
	return 0;
}

/**
 * @brief add log data to ring buffer
 *
 * @param priv       A pointer to moal_private struct
 * @param ring_id    ring ID
 * @param hdr        A pointer to wifi_ring_buffer_entry struct
 * @param data       A pointer to data buffer
 *
 * @return      0: success  -1: fail
 */
int
woal_ring_event_logger(moal_private *priv, int ring_id, pmlan_event pmevent)
{
	t_u8 event_buf[100] = { 0 };
	wifi_ring_buffer_driver_connectivity_event *connectivity_event;
	tlv_log *tlv;
	t_u8 *pos;
	wifi_ring_buffer_entry msg_hdr;
	wifi_ring_buffer *ring;

	ENTER();
	ring = (wifi_ring_buffer *) priv->rings[ring_id];

	if (!ring || ring->state != RING_ACTIVE) {
		PRINTM(MINFO, "Ring is not active\n");
		goto done;
	}

	switch (pmevent->event_id) {
	case MLAN_EVENT_ID_DRV_ASSOC_SUCC_LOGGER:
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
			assoc_logger_data *pbss_desc =
				(assoc_logger_data *) pmevent->event_buf;
			memset(&msg_hdr, 0, sizeof(msg_hdr));
			msg_hdr.flags |= RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
			msg_hdr.type = ENTRY_TYPE_CONNECT_EVENT;
			connectivity_event =
				(wifi_ring_buffer_driver_connectivity_event *)
				event_buf;
			connectivity_event->event = WIFI_EVENT_ASSOC_COMPLETE;
			pos = (t_u8 *)connectivity_event->tlvs;
			if (pbss_desc->oui) {
				tlv = (tlv_log *) pos;
				tlv->tag = WIFI_TAG_VENDOR_SPECIFIC;
				tlv->length = MLAN_MAC_ADDR_LENGTH / 2;
				memcpy(tlv->value, pbss_desc->oui, tlv->length);
				msg_hdr.entry_size +=
					tlv->length + TLV_LOG_HEADER_LEN;
				pos = pos + tlv->length + TLV_LOG_HEADER_LEN;
			}
			if (pbss_desc->bssid) {
				tlv = (tlv_log *) pos;
				tlv->tag = WIFI_TAG_BSSID;
				tlv->length = sizeof(pbss_desc->bssid);
				memcpy(tlv->value, pbss_desc->bssid,
				       sizeof(pbss_desc->bssid));
				msg_hdr.entry_size +=
					tlv->length + TLV_LOG_HEADER_LEN;
				pos = pos + tlv->length + TLV_LOG_HEADER_LEN;
			}
			if (pbss_desc->ssid) {
				tlv = (tlv_log *) pos;
				tlv->tag = WIFI_TAG_SSID;
				tlv->length = strlen(pbss_desc->ssid);
				memcpy(tlv->value, pbss_desc->ssid,
				       tlv->length);
				msg_hdr.entry_size +=
					tlv->length + TLV_LOG_HEADER_LEN;
				pos = pos + tlv->length + TLV_LOG_HEADER_LEN;
			}
			if (pbss_desc->rssi) {
				tlv = (tlv_log *) pos;
				tlv->tag = WIFI_TAG_RSSI;
				tlv->length = sizeof(pbss_desc->rssi);
				memcpy(tlv->value, &pbss_desc->rssi,
				       tlv->length);
				msg_hdr.entry_size +=
					tlv->length + TLV_LOG_HEADER_LEN;
				pos = pos + tlv->length + TLV_LOG_HEADER_LEN;
			}
			if (pbss_desc->channel) {
				tlv = (tlv_log *) pos;
				tlv->tag = WIFI_TAG_CHANNEL;
				tlv->length = sizeof(pbss_desc->channel);
				memcpy(tlv->value, &pbss_desc->channel,
				       sizeof(pbss_desc->channel));
				msg_hdr.entry_size +=
					tlv->length + TLV_LOG_HEADER_LEN;
			}
			msg_hdr.entry_size += sizeof(connectivity_event->event);
			DBG_HEXDUMP(MCMD_D, "connectivity_event",
				    (t_u8 *)connectivity_event,
				    msg_hdr.entry_size);
			woal_ring_push_data(priv, ring_id, &msg_hdr,
					    connectivity_event);
		}
		break;
	case MLAN_EVENT_ID_DRV_ASSOC_FAILURE_LOGGER:
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
			int status_code = *(int *)pmevent->event_buf;
			memset(&msg_hdr, 0, sizeof(msg_hdr));
			msg_hdr.flags |= RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
			msg_hdr.type = ENTRY_TYPE_CONNECT_EVENT;
			connectivity_event =
				(wifi_ring_buffer_driver_connectivity_event *)
				event_buf;
			connectivity_event->event = WIFI_EVENT_ASSOC_COMPLETE;
			pos = (t_u8 *)connectivity_event->tlvs;
			tlv = (tlv_log *) pos;
			tlv->tag = WIFI_TAG_STATUS;
			tlv->length = sizeof(status_code);
			memcpy(tlv->value, &status_code, sizeof(status_code));
			msg_hdr.entry_size += tlv->length + 4;
			msg_hdr.entry_size += sizeof(connectivity_event->event);
			woal_ring_push_data(priv, ring_id, &msg_hdr,
					    connectivity_event);
		}
		break;
	case MLAN_EVENT_ID_DRV_DISCONNECT_LOGGER:
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
			t_u16 reason_code = *(t_u16 *)pmevent->event_buf;
			memset(&msg_hdr, 0, sizeof(msg_hdr));
			msg_hdr.flags |= RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
			msg_hdr.type = ENTRY_TYPE_CONNECT_EVENT;
			connectivity_event =
				(wifi_ring_buffer_driver_connectivity_event *)
				event_buf;
			connectivity_event->event = WIFI_EVENT_ASSOC_COMPLETE;
			pos = (t_u8 *)connectivity_event->tlvs;
			tlv = (tlv_log *) pos;
			tlv->tag = WIFI_TAG_REASON_CODE;
			tlv->length = sizeof(reason_code);
			memcpy(tlv->value, &reason_code, sizeof(reason_code));
			msg_hdr.entry_size += tlv->length + 4;
			msg_hdr.entry_size += sizeof(connectivity_event->event);
			woal_ring_push_data(priv, ring_id, &msg_hdr,
					    connectivity_event);
		}
		break;
	default:
		break;
	}
done:
	LEAVE();
	return 0;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv               A pointer to moal_private struct
 * @param wake_reason        wake_reason
 *
 * @return      0: success  1: fail
 */
int
woal_wake_reason_vendor_event(IN moal_private *priv,
			      IN mlan_ds_hs_wakeup_reason wake_reason)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_id = 0;
	int ret = 0;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "priv is null \n");
		ret = -EINVAL;
		goto done;
	}
	wiphy = priv->wdev->wiphy;

	event_id = woal_get_event_id(event_wake_reason_report);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev,
					  sizeof(wake_reason.hs_wakeup_reason),
					  event_id, GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy,
					  sizeof(wake_reason.hs_wakeup_reason),
					  event_id, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = -ENOMEM;
		goto done;
	}

	PRINTM(MINFO, "wake_reason.hs_wakeup_reason = %d\n",
	       wake_reason.hs_wakeup_reason);
	nla_put_u16(skb, ATTR_WAKE_REASON_STAT, wake_reason.hs_wakeup_reason);
    /**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

done:
	PRINTM(MINFO, "wake reason vendor event ret %d\n", ret);
	LEAVE();
	return ret;
}

/**
 * @brief log wake reason
 *
 * @param priv       A pointer to moal_private struct
 * @param wake_reason    wake_reason
 *
 * @return      0: success  -1: fail
 */
int
woal_wake_reason_logger(moal_private *priv,
			mlan_ds_hs_wakeup_reason wake_reason)
{
	int ret = 0;
	ENTER();

	ret = woal_wake_reason_vendor_event(priv, wake_reason);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to start packet fate monitor
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_start_packet_fate_monitor(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data, int data_len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;

	ENTER();

    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	priv->pkt_fate_monitor_enable = MTRUE;

	ret = cfg80211_vendor_cmd_reply(skb);

	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv               A pointer to moal_private struct
 * @param pkt_type           tx or rx
 * @param fate               packet fate
 * @param payload_type       type of payload
 * @param drv_ts_usec        driver time in usec
 * @param fw_ts_usec         firmware time in usec
 * @param data               frame data
 * @param len                frame data len
 *
 * @return      0: success  1: fail
 */
int
woal_packet_fate_vendor_event(IN moal_private *priv,
			      IN packet_fate_packet_type pkt_type, IN t_u8 fate,
			      IN frame_type payload_type, IN t_u32 drv_ts_usec,
			      IN t_u32 fw_ts_usec, IN t_u8 *data, IN t_u32 len)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int event_id = 0;
	int ret = 0;
	PACKET_FATE_REPORT fate_report;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "priv is null \n");
		ret = -EINVAL;
		goto done;
	}
	wiphy = priv->wdev->wiphy;

	event_id = woal_get_event_id(event_packet_fate_monitor);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev,
					  len + sizeof(PACKET_FATE_REPORT),
					  event_id, GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy,
					  len + sizeof(PACKET_FATE_REPORT),
					  event_id, GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = -ENOMEM;
		goto done;
	}

	memset(&fate_report, 0, sizeof(PACKET_FATE_REPORT));
	if (pkt_type == PACKET_TYPE_TX) {
		fate_report.u.tx_report_i.fate = fate;
		fate_report.u.tx_report_i.frame_inf.payload_type = payload_type;
		fate_report.u.tx_report_i.frame_inf.driver_timestamp_usec =
			drv_ts_usec;
		fate_report.u.tx_report_i.frame_inf.firmware_timestamp_usec =
			fw_ts_usec;
		fate_report.u.tx_report_i.frame_inf.frame_len = len;
		nla_put(skb, ATTR_PACKET_FATE_TX, sizeof(PACKET_FATE_REPORT),
			&fate_report);
	} else {
		fate_report.u.rx_report_i.fate = fate;
		fate_report.u.rx_report_i.frame_inf.payload_type = payload_type;
		fate_report.u.rx_report_i.frame_inf.driver_timestamp_usec =
			drv_ts_usec;
		fate_report.u.rx_report_i.frame_inf.firmware_timestamp_usec =
			fw_ts_usec;
		fate_report.u.rx_report_i.frame_inf.frame_len = len;
		nla_put(skb, ATTR_PACKET_FATE_RX, sizeof(PACKET_FATE_REPORT),
			&fate_report);
	}
	DBG_HEXDUMP(MCMD_D, "fate_report", (t_u8 *)&fate_report,
		    sizeof(PACKET_FATE_REPORT));

	nla_put(skb, ATTR_PACKET_FATE_DATA, len, data);
	DBG_HEXDUMP(MCMD_D, "packet_fate_data", data, len);

    /**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

done:
	PRINTM(MINFO, "packet fate vendor event ret %d\n", ret);
	LEAVE();
	return ret;
}

/**
 * @brief log packet fate
 *
 * @param priv               A pointer to moal_private struct
 * @param pkt_type           tx or rx
 * @param fate               packet fate
 * @param payload_type       type of payload
 * @param drv_ts_usec        driver time in usec
 * @param fw_ts_usec         firmware time in usec
 * @param data               frame data
 * @param len                frame data len
 *
 * @return      0: success  -1: fail
 */
int
woal_packet_fate_monitor(moal_private *priv, packet_fate_packet_type pkt_type,
			 t_u8 fate, frame_type payload_type, t_u32 drv_ts_usec,
			 t_u32 fw_ts_usec, t_u8 *data, t_u32 len)
{
	int ret = 0;

	ENTER();

	if (priv->pkt_fate_monitor_enable)
		ret = woal_packet_fate_vendor_event(priv, pkt_type, fate,
						    payload_type, drv_ts_usec,
						    fw_ts_usec, data, len);

	PRINTM(MINFO, "packet fate monitor ret %d\n", ret);
	LEAVE();
	return ret;
}

/**
 * @brief init packet_filter in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  -1: fail
 */
static int
woal_init_packet_filter(moal_private *priv)
{
	int ret = 0;
	packet_filter *pkt_filter = NULL;

	ENTER();

	pkt_filter = vmalloc(sizeof(pkt_filter));
	if (!unlikely(pkt_filter)) {
		PRINTM(MERROR, "WiFi Logger: packet_filter alloc failed\n");
		ret = -ENOMEM;
		goto done;
	}

	memset(pkt_filter, 0, sizeof(packet_filter));

	spin_lock_init(&pkt_filter->lock);
	pkt_filter->packet_filter_max_len = PACKET_FILTER_MAX_LEN;
	pkt_filter->packet_filter_len = 0;
	pkt_filter->packet_filter_version = APF_VERSION;
	pkt_filter->state = PACKET_FILTER_STATE_STOP;

	priv->packet_filter = pkt_filter;

done:
	LEAVE();
	return ret;
}

/**
 * @brief deinit packet_filter in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  -1: fail
 */
static int
woal_deinit_packet_filter(moal_private *priv)
{
	int ret = 0;
	packet_filter *pkt_filter = NULL;
	unsigned long flags;

	ENTER();

	pkt_filter = priv->packet_filter;

	if (!unlikely(pkt_filter)) {
		goto done;
	}

	spin_lock_irqsave(&pkt_filter->lock, flags);
	pkt_filter->state = PACKET_FILTER_STATE_INIT;
	spin_unlock_irqrestore(&pkt_filter->lock, flags);

	vfree(pkt_filter);
	priv->packet_filter = NULL;

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to set packet filter
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_set_packet_filter(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0, rem, type;
	const struct nlattr *iter;
	packet_filter *pkt_filter = NULL;
	t_u32 packet_filter_len = 0;
	unsigned long flags;

	ENTER();
	pkt_filter = priv->packet_filter;
	if (!unlikely(pkt_filter) ||
	    pkt_filter->state == PACKET_FILTER_STATE_INIT) {
		PRINTM(MERROR, "packet_filter not init\n");
		ret = -EINVAL;
		goto done;
	}

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case ATTR_PACKET_FILTER_TOTAL_LENGTH:
			packet_filter_len = nla_get_u32(iter);
			if (packet_filter_len >
			    pkt_filter->packet_filter_max_len) {
				PRINTM(MERROR,
				       "packet_filter_len exceed max\n");
				ret = -EINVAL;
				goto done;
			}
			break;
		case ATTR_PACKET_FILTER_PROGRAM:
			spin_lock_irqsave(&pkt_filter->lock, flags);
			strncpy(pkt_filter->packet_filter_program,
				nla_data(iter), MIN(packet_filter_len,
						    nla_len(iter)));
			pkt_filter->packet_filter_len =
				MIN(packet_filter_len, nla_len(iter));
			pkt_filter->state = PACKET_FILTER_STATE_START;
			spin_unlock_irqrestore(&pkt_filter->lock, flags);
			DBG_HEXDUMP(MCMD_D, "packet_filter_program",
				    pkt_filter->packet_filter_program,
				    pkt_filter->packet_filter_len);
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get packet filter capability
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_get_packet_filter_capability(struct wiphy *wiphy,
						  struct wireless_dev *wdev,
						  const void *data,
						  int data_len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;
	packet_filter *pkt_filter;

	ENTER();
	pkt_filter = priv->packet_filter;
	if (!unlikely(pkt_filter)) {
		PRINTM(MERROR, "packet_filter not init\n");
		ret = -EINVAL;
		goto done;
	}

	reply_len =
		sizeof(pkt_filter->packet_filter_version) +
		sizeof(pkt_filter->packet_filter_max_len);
    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	nla_put_u32(skb, ATTR_PACKET_FILTER_VERSION,
		    pkt_filter->packet_filter_version);
	nla_put_u32(skb, ATTR_PACKET_FILTER_MAX_LEN,
		    pkt_filter->packet_filter_max_len);
	ret = cfg80211_vendor_cmd_reply(skb);

	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);
done:
	LEAVE();
	return ret;
}

/**
 * Perform a packet filtering program over a packet.
 *
 * @param program the program bytecode.
 * @param program_len the length of {@code apf_program} in bytes.
 * @param packet the packet bytes, starting from the 802.3 header and not
 *               including any CRC bytes at the end.
 * @param pkt_len the length of {@code packet} in bytes.
 * @param filter_age the number of seconds since the filter was programmed.
 *
 * @return non-zero if packet should be passed to AP, zero if
 *         packet should be dropped.
 */
int
process_packet(const t_u8 *program, t_u32 program_len, const t_u8 *pkt,
	       t_u32 pkt_len, t_u32 filter_age)
{
	/* Program counter */
	t_u32 pc = 0;
	/* Memory values */
	t_u32 mem[MEM_ITEMS] = { };
	/* Register values */
	t_u32 reg[2] = { };
	/* Number of instructions remaining to execute. This is done to make sure
	 * there is upper bound on execution time. It should never be hit and is only for
	 * safety. Initialize to the number of bytes in the program which is an
	 * upper bound on the number of instructions in the program. */
	t_u32 instructions_left = program_len;

	t_u8 bytecode;
	t_u32 opcode;
	t_u32 reg_num;
	t_u32 len_field;
	t_u32 imm_len;
	t_u32 end_offs;
	t_u32 val = 0;
	t_u32 last_pkt_offs;
	t_u32 imm = 0;
	int32_t sign_imm = 0;
	t_u32 offs = 0;
	t_u32 i;
	t_u32 load_size;

/* Is offset within program bounds? */
#define WITHIN_PROGRAM_BOUNDS(p) (ENFORCE_UNSIGNED(p) && (p) < program_len)
/* Is offset within packet bounds? */
#define WITHIN_PKT_BOUNDS(p) (ENFORCE_UNSIGNED(p) && (p) < pkt_len)
/* Verify an internal condition and accept packet if it fails. */
#define ASSERT_RETURN(c) if (!(c)) return PASS_PKT
/* If not within program bounds, Accept the packet */
#define ASSERT_WITHIN_PROGRAM_BOUNDS(p) ASSERT_RETURN(WITHIN_PROGRAM_BOUNDS(p))
/* If not within packet bounds, Accept the packet */
#define ASSERT_WITHIN_PKT_BOUNDS(p) ASSERT_RETURN(WITHIN_PKT_BOUNDS(p))
/* If not within program or not ahead of program counter, Accept the packet */
#define ASSERT_FORWARD_WITHIN_PROGRAM(p) ASSERT_RETURN(WITHIN_PROGRAM_BOUNDS(p) && (p) >= pc)

	/* Fill in pre-filled memory slot values. */
	mem[MEM_OFFSET_PKT_SIZE] = pkt_len;
	mem[MEM_OFFSET_FILTER_AGE] = filter_age;
	ASSERT_WITHIN_PKT_BOUNDS(APF_FRAME_HEADER_SIZE);

	/* Only populate if IP version is IPv4. */
	if ((pkt[APF_FRAME_HEADER_SIZE] & 0xf0) == 0x40) {
		mem[MEM_OFFSET_IPV4_HEADER_SIZE] =
			(pkt[APF_FRAME_HEADER_SIZE] & 15) * 4;
	}

	do {
		if (pc == program_len) {
			return PASS_PKT;
		} else if (pc == (program_len + 1)) {
			return DROP_PKT;
		}
		ASSERT_WITHIN_PROGRAM_BOUNDS(pc);
		bytecode = program[pc++];
		opcode = GET_OPCODE(bytecode);
		reg_num = GET_REGISTER(bytecode);

		/* All instructions have immediate fields, load them here now */
		len_field = GET_IMM_LENGTH(bytecode);
		imm = 0;
		sign_imm = 0;

#define REG (reg[reg_num])
#define OTHER_REG (reg[reg_num ^ 1])

		if (len_field) {
			imm_len = 1 << (len_field - 1);
			ASSERT_FORWARD_WITHIN_PROGRAM(pc + imm_len - 1);
			for (i = 0; i < imm_len; i++)
				imm = (imm << 8) | program[pc++];
			/* Sign extend imm into signed_imm. */
			sign_imm = imm << ((4 - imm_len) * 8);
			sign_imm >>= (4 - imm_len) * 8;
		}
		switch (opcode) {
		case NXP_LDB_OPCODE:
		case NXP_LDH_OPCODE:
		case NXP_LDW_OPCODE:
		case NXP_LDBX_OPCODE:
		case NXP_LDHX_OPCODE:
		case NXP_LDWX_OPCODE:{
				offs = imm;
				if (opcode >= NXP_LDBX_OPCODE) {
					/* Note: this can overflow and actually decrease offs. */
					offs += reg[1];
				}
				ASSERT_WITHIN_PKT_BOUNDS(offs);
				switch (opcode) {
				case NXP_LDB_OPCODE:
				case NXP_LDBX_OPCODE:
					load_size = 1;
					break;
				case NXP_LDH_OPCODE:
				case NXP_LDHX_OPCODE:
					load_size = 2;
					break;
				case NXP_LDW_OPCODE:
				case NXP_LDWX_OPCODE:
					load_size = 4;
					break;
					/* Immediately enclosing switch statement guarantees
					 * opcode cannot be any other value. */
				}
				end_offs = offs + (load_size - 1);
				/* Catch overflow/wrap-around. */
				ASSERT_RETURN(end_offs >= offs);
				ASSERT_WITHIN_PKT_BOUNDS(end_offs);
				val = 0;
				while (load_size--)
					val = (val << 8) | pkt[offs++];
				REG = val;
				break;
			}
		case NXP_JMP_OPCODE:
			/* This can jump backwards. Infinite looping prevented by instructions_left. */
			pc += imm;
			break;
		case NXP_JEQ_OPCODE:
		case NXP_JNE_OPCODE:
		case NXP_JGT_OPCODE:
		case NXP_JLT_OPCODE:
		case NXP_JSET_OPCODE:
		case NXP_JNEBS_OPCODE:{
				/* Load second immediate field. */
				t_u32 cmp_imm = 0;
				if (reg_num == 1) {
					cmp_imm = reg[1];
				} else if (len_field != 0) {
					t_u32 cmp_imm_len =
						1 << (len_field - 1);
					ASSERT_FORWARD_WITHIN_PROGRAM(pc +
								      cmp_imm_len
								      - 1);
					for (i = 0; i < cmp_imm_len; i++)
						cmp_imm =
							(cmp_imm << 8) |
							program[pc++];
				}
				switch (opcode) {
				case NXP_JEQ_OPCODE:
					if (reg[0] == cmp_imm)
						pc += imm;
					break;
				case NXP_JNE_OPCODE:
					if (reg[0] != cmp_imm)
						pc += imm;
					break;
				case NXP_JGT_OPCODE:
					if (reg[0] > cmp_imm)
						pc += imm;
					break;
				case NXP_JLT_OPCODE:
					if (reg[0] < cmp_imm)
						pc += imm;
					break;
				case NXP_JSET_OPCODE:
					if (reg[0] & cmp_imm)
						pc += imm;
					break;
				case NXP_JNEBS_OPCODE:{
						/* cmp_imm is size in bytes of data to compare.
						 * pc is offset of program bytes to compare.
						 * imm is jump target offset.
						 * REG is offset of pkt bytes to compare. */
						ASSERT_FORWARD_WITHIN_PROGRAM(pc
									      +
									      cmp_imm
									      -
									      1);
						ASSERT_WITHIN_PKT_BOUNDS(REG);
						last_pkt_offs =
							REG + cmp_imm - 1;
						ASSERT_RETURN(last_pkt_offs >=
							      REG);
						ASSERT_WITHIN_PKT_BOUNDS
							(last_pkt_offs);
						if (memcmp
						    (program + pc, pkt + REG,
						     cmp_imm))
							pc += imm;
						/* skip past comparison bytes */
						pc += cmp_imm;
						break;
					}
				}
				break;
			}
		case NXP_ADD_OPCODE:
			reg[0] += reg_num ? reg[1] : imm;
			break;
		case NXP_MUL_OPCODE:
			reg[0] *= reg_num ? reg[1] : imm;
			break;
		case NXP_DIV_OPCODE:{
				const t_u32 div_operand =
					reg_num ? reg[1] : imm;
				ASSERT_RETURN(div_operand);
				reg[0] /= div_operand;
				break;
			}
		case NXP_AND_OPCODE:
			reg[0] &= reg_num ? reg[1] : imm;
			break;
		case NXP_OR_OPCODE:
			reg[0] |= reg_num ? reg[1] : imm;
			break;
		case NXP_SH_OPCODE:{
				const int32_t shift_val =
					reg_num ? (int32_t) reg[1] : sign_imm;
				if (shift_val > 0)
					reg[0] <<= shift_val;
				else
					reg[0] >>= -shift_val;
				break;
			}
		case NXP_LI_OPCODE:
			REG = sign_imm;
			break;
		case NXP_EXT_OPCODE:
			if (
/* If LDM_EXT_OPCODE is 0 and imm is compared with it, a compiler error will result,
 * instead just enforce that imm is unsigned (so it's always greater or equal to 0). */
#if NXP_LDM_EXT_OPCODE == 0
				   ENFORCE_UNSIGNED(imm) &&
#else
				   imm >= NXP_LDM_EXT_OPCODE &&
#endif
				   imm < (NXP_LDM_EXT_OPCODE + MEM_ITEMS)) {
				REG = mem[imm - NXP_LDM_EXT_OPCODE];
			} else if (imm >= NXP_STM_EXT_OPCODE &&
				   imm < (NXP_STM_EXT_OPCODE + MEM_ITEMS)) {
				mem[imm - NXP_STM_EXT_OPCODE] = REG;
			} else
				switch (imm) {
				case NXP_NOT_EXT_OPCODE:
					REG = ~REG;
					break;
				case NXP_NEG_EXT_OPCODE:
					REG = -REG;
					break;
				case NXP_SWAP_EXT_OPCODE:{
						t_u32 tmp = REG;
						REG = OTHER_REG;
						OTHER_REG = tmp;
						break;
					}
				case NXP_MOV_EXT_OPCODE:
					REG = OTHER_REG;
					break;
					/* Unknown extended opcode */
				default:
					/* Bail out */
					return PASS_PKT;
				}
			break;
			/* Unknown opcode */
		default:
			/* Bail out */
			return PASS_PKT;
		}
	} while (instructions_left--);
	return PASS_PKT;
}

/**
 * @brief filter packet
 *
 * @param priv          A pointer to moal_private struct
 * @param data          packet data
 * @param len           packet len
 * @param filter_age    filter age
 * @return non-zero if packet should be passed to AP, zero if
 *         packet should be dropped.
 */
int
woal_filter_packet(moal_private *priv, t_u8 *data, t_u32 len, t_u32 filter_age)
{
	packet_filter *pkt_filter = NULL;
	int ret = PASS_PKT;
	unsigned long flags;

	ENTER();
	pkt_filter = priv->packet_filter;
	if (!unlikely(pkt_filter)) {
		PRINTM(MINFO, "packet_filter not init\n");
		goto done;
	}

	if (pkt_filter->state != PACKET_FILTER_STATE_START)
		goto done;

	DBG_HEXDUMP(MCMD_D, "packet_filter_program",
		    pkt_filter->packet_filter_program,
		    pkt_filter->packet_filter_len);
	DBG_HEXDUMP(MCMD_D, "packet_filter_data", data, len);
	spin_lock_irqsave(&pkt_filter->lock, flags);
	ret = process_packet(pkt_filter->packet_filter_program,
			     pkt_filter->packet_filter_len, data, len,
			     filter_age);
	spin_unlock_irqrestore(&pkt_filter->lock, flags);

done:
	PRINTM(MINFO, "packet filter ret %d\n", ret);
	LEAVE();
	return ret;
}

/**
 * @brief send vendor event to kernel
 *
 * @param priv     A pointer to moal_private
 * @param cmd      nan_cmd
 *
 * @return      0: success  1: fail
 */
int
woal_nan_vendor_event(IN moal_private *priv, nan_cmd cmd)
{
	struct wiphy *wiphy = NULL;
	struct sk_buff *skb = NULL;
	int ret = 0;
	int event_id = 0;

	ENTER();

	if (!priv || !priv->wdev || !priv->wdev->wiphy) {
		PRINTM(MERROR, "priv is null \n");
		ret = -EINVAL;
		goto done;
	}
	wiphy = priv->wdev->wiphy;
	PRINTM(MEVENT, "woal_nan_worker\n");
	event_id = woal_get_event_id(event_nan_cb);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = -EINVAL;
		goto done;
	}

/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev, sizeof(nan_cmd),
					  event_id, GFP_ATOMIC);
#else
	skb = cfg80211_vendor_event_alloc(wiphy, sizeof(nan_cmd), event_id,
					  GFP_ATOMIC);
#endif

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = -ENOMEM;
		goto done;
	}

	nla_put(skb, ATTR_NAN_FAKE, sizeof(NanHeader_Ext), &cmd.nan_header_ext);

	if (cmd.indicate_enable) {
		nla_put_u32(skb, ATTR_NAN_IND, cmd.indicate_type);
	}

	PRINTM(MCMND, "woal_nan_vendor_event %d, %d, %d,%d\n",
	       cmd.nan_header_ext.nan_header.MsgId,
	       cmd.nan_header_ext.nan_header.transactionId, cmd.indicate_enable,
	       cmd.indicate_type);

    /**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

done:
	LEAVE();
	return ret;
}

/**
 * @brief worker to send vendor event
 *
 * @param work       A pointer to work_struct struct
 *
 * @return void
 */
void
woal_nan_worker(struct work_struct *work)
{
	struct delayed_work *d_work = to_delayed_work(work);
	nan_cb *nan = container_of(d_work, nan_cb, response_work);
	moal_private *priv = nan->priv;
	nan_cmd cmd;
	int ret;
	ENTER();

	PRINTM(MCMND, "fifo len: %u\n", kfifo_len(&nan->cmd_fifo));

	ret = kfifo_out_peek(&nan->cmd_fifo, &cmd, sizeof(nan_cmd));
	PRINTM(MCMND, "fifo ret: %u\n", ret);

	if (ret) {
		ret = kfifo_out(&nan->cmd_fifo, &cmd, sizeof(nan_cmd));
	} else {
		PRINTM(MERROR, "not enough element:\n");
		goto done;
	}

	woal_nan_vendor_event(priv, cmd);

	ret = kfifo_out_peek(&nan->cmd_fifo, &cmd, sizeof(nan_cmd));

	if (ret)
		schedule_delayed_work(&nan->response_work, 0);

done:
	LEAVE();
	return;
}

/**
 * @brief init nan in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  -1: fail
 */
static int
woal_init_nan(moal_private *priv)
{
	int ret = 0;
	nan_cb *nan;
	moal_handle *phandle = priv->phandle;
	ENTER();

	nan = vmalloc(sizeof(nan_cb));
	if (!unlikely(nan)) {
		PRINTM(MERROR, "WiFi Logger: packet_filter alloc failed\n");
		ret = -ENOMEM;
		goto done;
	}

	if (!unlikely(phandle)) {
		PRINTM(MERROR, "WiFi Logger: phandle is null\n");
		ret = -ENOMEM;
		goto done;
	}

	memset(nan, 0, sizeof(nan_cb));
	INIT_DELAYED_WORK(&nan->response_work, woal_nan_worker);
	nan->priv = priv;

	ret = kfifo_alloc(&nan->cmd_fifo, CMD_FIFO_SIZE * sizeof(nan_cmd),
			  GFP_KERNEL);
	if (ret) {
		PRINTM(MERROR, "error kfifo_alloc\n");
		goto done;
	}

	priv->nan_cb = nan;
	LEAVE();
	return ret;

done:
	if (nan)
		vfree(nan);
	LEAVE();
	return ret;
}

/**
 * @brief deinit nan in moal_private
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  -1: fail
 */
static int
woal_deinit_nan(moal_private *priv)
{
	int ret = 0;
	nan_cb *nan = NULL;

	ENTER();

	nan = priv->nan_cb;

	if (!unlikely(nan)) {
		goto done;
	}

	cancel_delayed_work_sync(&nan->response_work);

	kfifo_free(&nan->cmd_fifo);

	vfree(nan);
	priv->nan_cb = NULL;

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to enable nan
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
nan_handler(struct wiphy *wiphy, struct wireless_dev *wdev,
	    const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	t_u32 reply_len = 0;
	int ret = 0;
	int rem, type;

	const struct nlattr *iter;
	nan_cb *nan = priv->nan_cb;
	nan_cmd cmd;
	t_u32 indicate_number_total = 0;
	t_u32 indicate_array[MAX_INDICATE_ARRAY_SIZE] = { 0 };
	int i = 0;

	ENTER();

	memset(&cmd, 0, sizeof(nan_cmd));
    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = -ENOMEM;
		goto done;
	}

	if (!unlikely(nan)) {
		PRINTM(MERROR, "WiFi hal: nan not init\n");
		ret = -EINVAL;
		goto done;
	}

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case ATTR_NAN_FAKE:
			memcpy(&cmd, nla_data(iter), sizeof(NanHeader));
			PRINTM(MCMND, "ATTR_NAN_FAKE %d, %d\n",
			       cmd.nan_header_ext.nan_header.MsgId,
			       cmd.nan_header_ext.nan_header.transactionId);
			break;
		case ATTR_NAN_IND:
			indicate_array[indicate_number_total] =
				nla_get_u32(iter);
			indicate_number_total++;
			PRINTM(MCMND, "ATTR_NAN_IND %d, %d\n",
			       indicate_array[indicate_number_total - 1],
			       indicate_number_total);
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	cmd.indicate_enable = 0;
	kfifo_in(&nan->cmd_fifo, &cmd, sizeof(nan_cmd));
	PRINTM(MCMND, "fifo len: %u\n", kfifo_len(&nan->cmd_fifo));

	for (i = 0; i < MAX_INDICATE_ARRAY_SIZE && i < indicate_number_total;
	     i++) {
		cmd.indicate_enable = 1;
		cmd.indicate_type = indicate_array[i];
		kfifo_in(&nan->cmd_fifo, &cmd, sizeof(nan_cmd));
		PRINTM(MCMND, "fifo len: %u\n", kfifo_len(&nan->cmd_fifo));
		DBG_HEXDUMP(MCMD_D, "nan_cmd", (t_u8 *)&cmd, sizeof(nan_cmd));
	}

	ret = cfg80211_vendor_cmd_reply(skb);

	schedule_delayed_work(&nan->response_work, 0);

	if (ret)
		PRINTM(MERROR, "Vendor command reply failed ret = %d\n", ret);

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to enable nan
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_enable_req(struct wiphy *wiphy,
				    struct wireless_dev *wdev, const void *data,
				    int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to disable nan
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_disable_req(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to nan publish req
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_publish_req(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to cancel nan publish
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_publish_cancel(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to nan subscribe req
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_subscribe_req(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to cancel nan subscribe
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_subscribe_cancel(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to trasmit followup
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_trasmit_followup(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to req nan stats
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_stats_req(struct wiphy *wiphy,
				   struct wireless_dev *wdev, const void *data,
				   int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to config nan
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_config_req(struct wiphy *wiphy,
				    struct wireless_dev *wdev, const void *data,
				    int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to nan tca req
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_tca_req(struct wiphy *wiphy, struct wireless_dev *wdev,
				 const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to beacon sdf payload
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_beacon_sdf_payload(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get nan version
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_get_version(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get nan capability
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_get_capability(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to data if create
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_data_if_create(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to data if delete
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_data_if_delete(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to data req initor
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_data_req_initor(struct wiphy *wiphy,
					 struct wireless_dev *wdev,
					 const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to data indication resp
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_data_indi_resp(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to date end
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_nan_data_end(struct wiphy *wiphy,
				  struct wireless_dev *wdev, const void *data,
				  int len)
{
	int ret = 0;
	ENTER();

	ret = nan_handler(wiphy, wdev, data, len);

	LEAVE();
	return ret;
}

/**
 * @brief init wifi hal
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  1: fail
 */
int
woal_init_wifi_hal(moal_private *priv)
{
	ENTER();
	woal_init_ring_buffer(priv);
	priv->pkt_fate_monitor_enable = MFALSE;
	woal_init_packet_filter(priv);
	woal_init_nan(priv);
	LEAVE();
	return 0;
}

/**
 * @brief deinit wifi hal
 *
 * @param priv       A pointer to moal_private struct
 *
 * @return      0: success  1: fail
 */
int
woal_deinit_wifi_hal(moal_private *priv)
{
	ENTER();
	woal_deinit_ring_buffer(priv);
	priv->pkt_fate_monitor_enable = MFALSE;
	woal_deinit_packet_filter(priv);
	woal_deinit_nan(priv);
	LEAVE();
	return 0;
}

/**
 * @brief vendor command to get correlated HW and System time
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_get_correlated_time(struct wiphy *wiphy,
					 struct wireless_dev *wdev,
					 const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_get_correlated_time *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int err = -1;
	int length = 0;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		PRINTM(MERROR, "Could not allocate mlan ioctl request!\n");
		return -ENOMEM;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	misc->sub_command = MLAN_OID_MISC_GET_CORRELATED_TIME;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "get correleted time fail\n");
		goto done;
	}

	length = sizeof(mlan_ds_get_correlated_time);
	info = (mlan_ds_get_correlated_time *) (&misc->param.host_clock);

	DBG_HEXDUMP(MCMD_D, "get_correlated_time", (t_u8 *)info, length);

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, length);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		goto done;
	}

	/* Push the data to the skb */
	nla_put_nohdr(skb, length, info);

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return err;
}

/**
 * @brief vendor command to get link layer statistic
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_link_statistic_get(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct sk_buff *skb = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	wifi_radio_stat *radio_stat = NULL;
	wifi_radio_stat *radio_stat_tmp = NULL;
	wifi_iface_stat *iface_stat = NULL;
	t_u32 num_radio = 0, iface_stat_len = 0, radio_stat_len = 0;
	int err = -1, length = 0, i;
	char *ioctl_link_stats_buf = NULL;
	mlan_ds_get_stats stats;
	t_u64 cur_time = 0;
	t_u64 inter_msec = 0;
	t_u64 max_msec = (t_u64)24 * (t_u64)24 * (t_u64)3600 * (t_u64)1000;
	moal_handle *handle = priv->phandle;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(t_u32) + BUF_MAXLEN);
	if (req == NULL) {
		PRINTM(MERROR, "Could not allocate mlan ioctl request!\n");
		return -ENOMEM;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_LINK_STATS;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "get link layer statistic fail\n");
		goto done;
	}

	/* Get Log from the firmware */
	memset(&stats, 0, sizeof(mlan_ds_get_stats));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_stats_info(priv, MOAL_IOCTL_WAIT, &stats)) {
		PRINTM(MERROR, "Error getting stats information\n");
		goto done;
	}

	ioctl_link_stats_buf = info->param.link_statistic;
	num_radio = *((t_u32 *)info->param.link_statistic);

	radio_stat =
		(wifi_radio_stat *) (info->param.link_statistic +
				     sizeof(num_radio));
	radio_stat_len = num_radio * sizeof(wifi_radio_stat);

	/* Re-write on_time/tx_time/rx_time/on_time_scan from moal handle */
	PRINTM(MINFO, "handle->on_time=%llu\n", handle->on_time);
	if (handle->on_time) {
		moal_get_boot_ktime(handle, &cur_time);
		inter_msec = moal_do_div(cur_time - handle->on_time, 1000000);
		PRINTM(MINFO, "cur_time=%llu inter_msec=%llu max_msec=%llu\n",
		       cur_time, inter_msec, max_msec);
		/* When we report the time up, u32 is not big enough(represent max 49days) and might out of range, make the max value to 24days. */
		if (inter_msec > max_msec) {
			PRINTM(MMSG,
			       "Out of range, set inter_msec=%llu to max_msec=%llu\n",
			       inter_msec, max_msec);
			inter_msec = max_msec;
		}
	}
	PRINTM(MINFO, "handle->tx_time=%llu\n", handle->tx_time);
	PRINTM(MINFO, "handle->rx_time=%llu\n", handle->rx_time);
	PRINTM(MINFO, "handle->scan_time=%llu\n", handle->scan_time);
	radio_stat_tmp = radio_stat;
	for (i = 0; i < num_radio; i++) {
		radio_stat_tmp->on_time = (t_u32)inter_msec;
		radio_stat_tmp->tx_time =
			(t_u32)moal_do_div(handle->tx_time, 1000);
		radio_stat_tmp->rx_time =
			(t_u32)moal_do_div(handle->rx_time, 1000);
		radio_stat_tmp->on_time_scan =
			(t_u32)moal_do_div(handle->scan_time, 1000);
		radio_stat_tmp++;
	}

	iface_stat =
		(wifi_iface_stat *) (info->param.link_statistic +
				     sizeof(num_radio) + radio_stat_len);
	iface_stat_len = sizeof(wifi_iface_stat);
	/* Fill some fileds */
	iface_stat->beacon_rx = stats.bcn_rcv_cnt;

	/* could get peer info with seperate cmd */
	for (i = 0; i < iface_stat->num_peers; i++) {
		/* no need copy, just increase iface_stat length */
		iface_stat_len +=
			sizeof(wifi_peer_info) +
			sizeof(wifi_rate_stat) *
			iface_stat->peer_info[i].num_rate;
	}

	/* Here the length doesn't contain addition 2 attribute header length */
	length = NLA_HDRLEN * 2 + sizeof(num_radio) + radio_stat_len +
		iface_stat_len;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, length);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		goto done;
	}

	if (nla_put_u32(skb, ATTR_LL_STATS_NUM_RADIO, num_radio) ||
	    nla_put(skb, ATTR_LL_STATS_RADIO, radio_stat_len, radio_stat) ||
	    nla_put(skb, ATTR_LL_STATS_IFACE, iface_stat_len, iface_stat)) {
		PRINTM(MERROR, "nla_put failed!\n");
		kfree(skb);
		goto done;
	}
	PRINTM(MCMD_D, "%s: <<< Start DUMP\n", __FUNCTION__);
	PRINTM(MCMD_D, "sizeof(wifi_radio_stat)=%zu\n",
	       sizeof(wifi_radio_stat));
	DBG_HEXDUMP(MCMD_D, "radio_stat", (t_u8 *)radio_stat, radio_stat_len);
	PRINTM(MCMD_D, "sizeof(wifi_channel_stat)=%zu\n",
	       sizeof(wifi_channel_stat));
	DBG_HEXDUMP(MCMD_D, "iface_stat", (t_u8 *)iface_stat, iface_stat_len);
	PRINTM(MCMD_D, "num_radio=%d\n", num_radio);
	radio_stat_tmp = radio_stat;
	for (i = 0; i < num_radio; i++) {
		PRINTM(MCMD_D, "--radio_stat[%d]--\n", i);
		PRINTM(MCMD_D, "radio=%d\n", radio_stat_tmp->radio);
		PRINTM(MCMD_D, "on_time=%d\n", radio_stat_tmp->on_time);
		PRINTM(MCMD_D, "tx_time=%d\n", radio_stat_tmp->tx_time);
		PRINTM(MCMD_D, "reserved0=%d\n", radio_stat_tmp->reserved0);
		PRINTM(MCMD_D, "rx_time=%d\n", radio_stat_tmp->rx_time);
		PRINTM(MCMD_D, "on_time_scan=%d\n",
		       radio_stat_tmp->on_time_scan);
		PRINTM(MCMD_D, "on_time_nbd=%d\n", radio_stat_tmp->on_time_nbd);
		PRINTM(MCMD_D, "on_time_gscan=%d\n",
		       radio_stat_tmp->on_time_gscan);
		PRINTM(MCMD_D, "on_time_roam_scan=%d\n",
		       radio_stat_tmp->on_time_roam_scan);
		PRINTM(MCMD_D, "on_time_pno_scan=%d\n",
		       radio_stat_tmp->on_time_pno_scan);
		PRINTM(MCMD_D, "on_time_hs20=%d\n",
		       radio_stat_tmp->on_time_hs20);
		PRINTM(MCMD_D, "num_channels=%d\n",
		       radio_stat_tmp->num_channels);
		radio_stat_tmp++;
	}
	PRINTM(MCMD_D, "%s: >>> End DUMP\n", __FUNCTION__);

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return err;
}

/**
 * @brief API to trigger the link layer statistics collection.
 *   Unless his API is invoked - link layer statistics will not be collected.
 *   Radio statistics (once started) do not stop or get reset unless
 *   wifi_clear_link_stats is invoked, Interface statistics (once started)
 *   reset and start afresh after each connection.
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_link_statistic_set(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(wdev->netdev);
	struct nlattr *tb[ATTR_LL_STATS_MAX + 1];
	wifi_link_layer_params ll_params;
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int err = 0;

	priv->phandle->link_layer_en = MTRUE;

	err = nla_parse(tb, ATTR_LL_STATS_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err)
		return err;

	if (!tb[ATTR_LL_STATS_MPDU_SIZE_THRESHOLD] ||
	    !tb[ATTR_LL_STATS_AGGRSSIVE_STATS_GATHERING])
		return -EINVAL;

	ll_params.mpdu_size_threshold =
		nla_get_u32(tb[ATTR_LL_STATS_MPDU_SIZE_THRESHOLD]);
	ll_params.aggressive_statistics_gathering =
		nla_get_u32(tb[ATTR_LL_STATS_AGGRSSIVE_STATS_GATHERING]);

	PRINTM(MEVENT,
	       "link layer params mpdu_size_threshold = 0x%x, aggressive_statistics_gathering = 0x%x\n",
	       ll_params.mpdu_size_threshold,
	       ll_params.aggressive_statistics_gathering);

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(t_u32) +
					sizeof(mlan_ds_get_info));
	if (req == NULL) {
		PRINTM(MERROR, "Could not allocate mlan ioctl request!\n");
		return -ENOMEM;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_LINK_STATS;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_SET;

	/* Configure parameter to firmware */
	memcpy(info->param.link_statistic, &ll_params, sizeof(ll_params));

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status == MLAN_STATUS_SUCCESS) {
		PRINTM(MMSG, "enable link layer statistic successfully\n");
	}

	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return 0;
}

/**
 * @brief clear function should download command to fimrware,
 * 	so that firmware could cleanup per peer statistic number
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_link_statistic_clr(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(wdev->netdev);
	struct nlattr *tb[ATTR_LL_STATS_MAX + 1];
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	u32 stats_clear_req_mask = 0x0, stats_clear_rsp_mask = 0x0;
	u8 stop_req = 0x0, stop_rsp = 0x0;
	struct sk_buff *skb = NULL;
	int err = 0, length = 0;

	err = nla_parse(tb, ATTR_LL_STATS_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err)
		return err;

	if (!tb[ATTR_LL_STATS_CLEAR_REQ_MASK])
		return -EINVAL;
	else
		stats_clear_req_mask =
			nla_get_u32(tb[ATTR_LL_STATS_CLEAR_REQ_MASK]);

	if (!tb[ATTR_LL_STATS_STOP_REQ])
		return -EINVAL;
	else
		stop_req = nla_get_u8(tb[ATTR_LL_STATS_STOP_REQ]);

	PRINTM(MEVENT,
	       "link layer clear stats_clear_req_mask = 0x%x, stop_req = 0x%x\n",
	       stats_clear_req_mask, stop_req);

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(t_u32) +
					sizeof(mlan_ds_get_info));
	if (req == NULL) {
		PRINTM(MERROR, "Could not allocate mlan ioctl request!\n");
		return -ENOMEM;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_LINK_STATS;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_CLEAR;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status == MLAN_STATUS_SUCCESS) {
		PRINTM(MMSG, "enable link layer statistic successfully\n");
	}

	length = NLA_HDRLEN + sizeof(stats_clear_rsp_mask) + sizeof(stop_rsp);
	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, length);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		err = -EINVAL;
		goto exit;
	}

	/*  clear api to reset statistics, stats_clear_rsp_mask identifies what stats have been cleared
	 *  stop_req = 1 will imply whether to stop the statistics collection.
	 *  stop_rsp = 1 will imply that stop_req was honored and statistics collection was stopped.
	 */
	stats_clear_rsp_mask = WIFI_STATS_RADIO | WIFI_STATS_IFACE;
	stop_rsp = 1;
	if (nla_put_u32(skb, ATTR_LL_STATS_CLEAR_RSP_MASK, stats_clear_rsp_mask)
	    || nla_put_u8(skb, ATTR_LL_STATS_STOP_RSP, stop_rsp)) {
		PRINTM(MERROR, "nla_put failed!\n");
		kfree(skb);
		err = -EINVAL;
		goto exit;
	}

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
		goto exit;;
	}

exit:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return err;
}

#ifdef STA_CFG80211
#define RSSI_MONOTOR_START 1
#define RSSI_MONOTOR_STOP 0

/**
 * @brief vendor command to control rssi monitor
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rssi_monitor(struct wiphy *wiphy,
				  struct wireless_dev *wdev, const void *data,
				  int len)
{
	struct nlattr *tb[ATTR_RSSI_MONITOR_MAX + 1];
	moal_private *priv = (moal_private *)woal_get_netdev_priv(wdev->netdev);
	u32 rssi_monitor_control = 0x0;
	s8 rssi_min = 0, rssi_max = 0;
	int err = 0;
	t_u8 *pos = NULL;
	struct sk_buff *skb = NULL;
	int ret = 0;

	ENTER();

	if (!priv->media_connected) {
		ret = -EINVAL;
		goto done;
	}

	ret = nla_parse(tb, ATTR_RSSI_MONITOR_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (ret)
		goto done;

	if (!tb[ATTR_RSSI_MONITOR_CONTROL]) {
		ret = -EINVAL;
		goto done;
	}
	rssi_monitor_control = nla_get_u32(tb[ATTR_RSSI_MONITOR_CONTROL]);

	if (rssi_monitor_control == RSSI_MONOTOR_START) {
		if ((!tb[ATTR_RSSI_MONITOR_MIN_RSSI]) ||
		    (!tb[ATTR_RSSI_MONITOR_MAX_RSSI])) {
			ret = -EINVAL;
			goto done;
		}

		rssi_min = nla_get_s8(tb[ATTR_RSSI_MONITOR_MIN_RSSI]);
		rssi_max = nla_get_s8(tb[ATTR_RSSI_MONITOR_MAX_RSSI]);

		PRINTM(MEVENT,
		       "start rssi monitor rssi_min = %d, rssi_max= %d\n",
		       rssi_min, rssi_max);

		/* set rssi low/high threshold */
		priv->cqm_rssi_high_thold = rssi_max;
		priv->cqm_rssi_thold = rssi_min;
		priv->cqm_rssi_hyst = 4;
		woal_set_rssi_threshold(priv, 0, MOAL_IOCTL_WAIT);
	} else if (rssi_monitor_control == RSSI_MONOTOR_STOP) {
		/* stop rssi monitor */
		PRINTM(MEVENT, "stop rssi monitor\n");
		/* set both rssi_thold/hyst to 0, will trigger subscribe event clear */
		priv->cqm_rssi_high_thold = 0;
		priv->cqm_rssi_thold = 0;
		priv->cqm_rssi_hyst = 0;
		woal_set_rssi_threshold(priv, 0, MOAL_IOCTL_WAIT);
	} else {
		PRINTM(MERROR, "invalid rssi_monitor control request\n");
		ret = -EINVAL;
		goto done;
	}

	/* Alloc the SKB for cmd reply */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, len);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		ret = -EINVAL;
		goto done;
	}
	pos = skb_put(skb, len);
	memcpy(pos, data, len);
	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
		ret = err;
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief send rssi event to kernel
 *
 * @param priv       A pointer to moal_private
 * @param rssi       current rssi value
 *
 * @return      N/A
 */
void
woal_cfg80211_rssi_monitor_event(moal_private *priv, t_s16 rssi)
{
	struct sk_buff *skb = NULL;
	t_s8 rssi_value = 0;

	ENTER();

	skb = dev_alloc_skb(NLA_HDRLEN * 2 + ETH_ALEN + sizeof(t_s8));
	if (!skb)
		goto done;
	/* convert t_s16 to t_s8 */
	rssi_value = -abs(rssi);
	if (nla_put
	    (skb, ATTR_RSSI_MONITOR_CUR_BSSID, ETH_ALEN, priv->conn_bssid) ||
	    nla_put_s8(skb, ATTR_RSSI_MONITOR_CUR_RSSI, rssi_value)) {
		PRINTM(MERROR, "nla_put failed!\n");
		kfree(skb);
		goto done;
	}
	woal_cfg80211_vendor_event(priv, event_rssi_monitor, (t_u8 *)skb->data,
				   skb->len);
	kfree(skb);
done:
	LEAVE();
}
#endif

/**
 * @brief vendor command to key_mgmt_set_key
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  fail otherwise
 */
static int
woal_cfg80211_subcmd_set_roaming_offload_key(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     const void *data, int data_len)
{
	moal_private *priv;
	struct net_device *dev;
	struct sk_buff *skb = NULL;
	t_u8 *pos = (t_u8 *)data;
	int ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (data)
		DBG_HEXDUMP(MCMD_D, "Vendor pmk", (t_u8 *)data, data_len);

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}

	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);
	if (!priv || !pos) {
		LEAVE();
		return -EFAULT;
	}

	if (data_len > MLAN_MAX_KEY_LENGTH) {
		memcpy(&priv->pmk.pmk_r0, pos, MLAN_MAX_KEY_LENGTH);
		pos += MLAN_MAX_KEY_LENGTH;
		memcpy(&priv->pmk.pmk_r0_name, pos,
		       MIN(MLAN_MAX_PMKR0_NAME_LENGTH,
			   data_len - MLAN_MAX_KEY_LENGTH));
	} else {
		memcpy(&priv->pmk.pmk, data,
		       MIN(MLAN_MAX_KEY_LENGTH, data_len));
	}
	priv->pmk_saved = MTRUE;

    /** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, data_len);
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		LEAVE();
		return -EFAULT;
	}
	pos = skb_put(skb, data_len);
	memcpy(pos, data, data_len);
	ret = cfg80211_vendor_cmd_reply(skb);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to supplicant to update AP info
 *
 * @param priv     A pointer to moal_private
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
int
woal_roam_ap_info(IN moal_private *priv, IN t_u8 *data, IN int len)
{
	struct wiphy *wiphy = priv->wdev->wiphy;
	struct sk_buff *skb = NULL;
	int ret = MLAN_STATUS_SUCCESS;
	key_info *pkey = NULL;
	apinfo *pinfo = NULL;
	apinfo *req_tlv = NULL;
	MrvlIEtypesHeader_t *tlv = NULL;
	t_u16 tlv_type = 0, tlv_len = 0, tlv_buf_left = 0;
	int event_id = 0;
	t_u8 authorized = 1;

	ENTER();

	event_id = woal_get_event_id(event_fw_roam_success);
	if (event_max == event_id) {
		PRINTM(MERROR, "Not find this event %d \n", event_id);
		ret = 1;
		LEAVE();
		return ret;
	}
	/**allocate skb*/
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	skb = cfg80211_vendor_event_alloc(wiphy, priv->wdev, len + 50,
#else
	skb = cfg80211_vendor_event_alloc(wiphy, len + 50,
#endif
					  event_id, GFP_ATOMIC);

	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor event\n");
		ret = 1;
		LEAVE();
		return ret;
	}

	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID,
		MLAN_MAC_ADDR_LENGTH, (t_u8 *)data);
	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED,
		sizeof(authorized), &authorized);
	tlv = (MrvlIEtypesHeader_t *)(data + MLAN_MAC_ADDR_LENGTH);
	tlv_buf_left = len - MLAN_MAC_ADDR_LENGTH;
	while (tlv_buf_left >= sizeof(MrvlIEtypesHeader_t)) {
		tlv_type = woal_le16_to_cpu(tlv->type);
		tlv_len = woal_le16_to_cpu(tlv->len);

		if (tlv_buf_left < (tlv_len + sizeof(MrvlIEtypesHeader_t))) {
			PRINTM(MERROR,
			       "Error processing firmware roam success TLVs, bytes left < TLV length\n");
			break;
		}

		switch (tlv_type) {
		case TLV_TYPE_APINFO:
			pinfo = (apinfo *) tlv;
			nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE,
				pinfo->header.len, pinfo->rsp_ie);
			break;

		case TLV_TYPE_ASSOC_REQ_IE:
			req_tlv = (apinfo *) tlv;
			nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE,
				req_tlv->header.len, req_tlv->rsp_ie);
			break;

		case TLV_TYPE_KEYINFO:
			pkey = (key_info *) tlv;
			nla_put(skb,
				MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_KEY_REPLAY_CTR,
				MLAN_REPLAY_CTR_LEN, pkey->key.replay_ctr);
			nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KCK,
				MLAN_KCK_LEN, pkey->key.kck);
			nla_put(skb, MRVL_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KEK,
				MLAN_KEK_LEN, pkey->key.kek);
			break;
		default:
			break;
		}
		tlv_buf_left -= tlv_len + sizeof(MrvlIEtypesHeader_t);
		tlv = (MrvlIEtypesHeader_t *)((t_u8 *)tlv + tlv_len +
					      sizeof(MrvlIEtypesHeader_t));
	}

	/**send event*/
	cfg80211_vendor_event(skb, GFP_ATOMIC);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get fw roaming capability
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  fail otherwise
 */
static int
woal_cfg80211_subcmd_get_roaming_capability(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int len)
{
	int ret = MLAN_STATUS_SUCCESS;
	wifi_roaming_capabilities capa;
	struct sk_buff *skb = NULL;
	int err = 0;

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}

	capa.max_blacklist_size = MAX_AP_LIST;
	capa.max_whitelist_size = MAX_SSID_NUM;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
						  sizeof
						  (wifi_roaming_capabilities) +
						  50);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		goto done;
	}

	/* Push the data to the skb */
	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CAPA,
		sizeof(wifi_roaming_capabilities), (t_u8 *)&capa);

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
	}

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to enable/disable fw roaming
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  fail otherwise
 */
static int
woal_cfg80211_subcmd_fw_roaming_enable(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int len)
{
	moal_private *priv;
	struct net_device *dev;
	int ret = MLAN_STATUS_SUCCESS;
	struct sk_buff *skb = NULL;
	const struct nlattr *iter;
	int type, rem, err;
	t_u32 fw_roaming_enable = 0;
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	t_u8 enable = 0;
#endif
#endif

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}

	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);
	if (!priv || !priv->phandle) {
		LEAVE();
		return -EFAULT;
	}

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONTROL:
			fw_roaming_enable = nla_get_u32(iter);
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	PRINTM(MMSG, "FW roaming set enable=%d from wifi hal.\n",
	       fw_roaming_enable);
	ret = woal_enable_fw_roaming(priv, fw_roaming_enable);
	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(t_u32) + 50);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed\n");
		goto done;
	}

	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONTROL, sizeof(t_u32),
		&fw_roaming_enable);
	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err)) {
		PRINTM(MERROR, "Vendor Command reply failed ret:%d \n", err);
	}
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (!fw_roaming_enable) {
		woal_cfg80211_vendor_event(priv, event_set_key_mgmt_offload,
					   &enable, sizeof(enable));
	}
#endif
#endif

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to config blacklist and whitelist for fw roaming
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  fail otherwise
 */
static int
woal_cfg80211_subcmd_fw_roaming_config(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int len)
{
	moal_private *priv;
	struct net_device *dev;
	int ret = MLAN_STATUS_SUCCESS;
	const struct nlattr *iter;
	int type, rem;
	woal_roam_offload_cfg *roam_offload_cfg = NULL;
	wifi_bssid_params blacklist;
	wifi_ssid_params whitelist;

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}

	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);
	if (!priv || !priv->phandle) {
		LEAVE();
		return -EFAULT;
	}

	memset((char *)&blacklist, 0, sizeof(wifi_bssid_params));
	memset((char *)&whitelist, 0, sizeof(wifi_ssid_params));
	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONFIG_BSSID:
			memcpy((t_u8 *)&blacklist, nla_data(iter),
			       MIN(nla_len(iter), sizeof(wifi_bssid_params)));
			break;
		case MRVL_WLAN_VENDOR_ATTR_FW_ROAMING_CONFIG_SSID:
			memcpy((t_u8 *)&whitelist, nla_data(iter),
			       MIN(nla_len(iter), sizeof(wifi_ssid_params)));
			break;
		default:
			PRINTM(MERROR, "Unknown type: %d\n", type);
			ret = -EINVAL;
			goto done;
		}
	}

	if (roamoffload_in_hs) {
		/*save blacklist and whitelist in driver */
		priv->phandle->fw_roam_params.black_list.ap_num =
			blacklist.num_bssid;
		memcpy((t_u8 *)priv->phandle->fw_roam_params.black_list.ap_mac,
		       (t_u8 *)blacklist.mac_addr,
		       sizeof(wifi_bssid_params) - sizeof(blacklist.num_bssid));
		priv->phandle->fw_roam_params.ssid_list.ssid_num =
			whitelist.num_ssid;
		memcpy((t_u8 *)priv->phandle->fw_roam_params.ssid_list.ssids,
		       (t_u8 *)whitelist.whitelist_ssid,
		       sizeof(wifi_ssid_params) - sizeof(whitelist.num_ssid));
	} else {
		roam_offload_cfg =
			(woal_roam_offload_cfg *)
			kmalloc(sizeof(woal_roam_offload_cfg), GFP_KERNEL);
		if (!roam_offload_cfg) {
			PRINTM(MERROR, "kmalloc failed!\n");
			ret = -ENOMEM;
			goto done;
		}
		/*download parameters directly to fw */
		memset((char *)roam_offload_cfg, 0,
		       sizeof(woal_roam_offload_cfg));
		roam_offload_cfg->black_list.ap_num = blacklist.num_bssid;
		memcpy((t_u8 *)&roam_offload_cfg->black_list.ap_mac,
		       (t_u8 *)blacklist.mac_addr,
		       sizeof(wifi_bssid_params) - sizeof(blacklist.num_bssid));
		roam_offload_cfg->ssid_list.ssid_num = whitelist.num_ssid;
		memcpy((t_u8 *)&roam_offload_cfg->ssid_list.ssids,
		       (t_u8 *)whitelist.whitelist_ssid,
		       sizeof(wifi_ssid_params) - sizeof(whitelist.num_ssid));
		woal_config_fw_roaming(priv, ROAM_OFFLOAD_PARAM_CFG,
				       roam_offload_cfg);
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to enable/disable 11k
 *
 * @param wiphy         A pointer to wiphy struct
 * @param wdev          A pointer to wireless_dev struct
 * @param data           a pointer to data
 * @param data_len     data length
 *
 * @return      0: success  <0: fail
 */
static int
woal_cfg80211_subcmd_11k_cfg(struct wiphy *wiphy,
			     struct wireless_dev *wdev,
			     const void *data, int data_len)
{
	struct net_device *dev = NULL;
	moal_private *priv = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11k_cfg *pcfg_11k = NULL;
	struct nlattr *tb_vendor[ATTR_ND_OFFLOAD_MAX + 1];
	int ret = 0;
	int status = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}

	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);

	nla_parse(tb_vendor, ATTR_ND_OFFLOAD_MAX,
		  (struct nlattr *)data, data_len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
		  , NULL
#endif
		);
	if (!tb_vendor[ATTR_ND_OFFLOAD_CONTROL]) {
		PRINTM(MINFO, "%s: ATTR_ND_OFFLOAD not found\n", __FUNCTION__);
		ret = -EFAULT;
		goto done;
	}
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11k_cfg));
	if (req == NULL) {
		PRINTM(MERROR, "Could not allocate mlan ioctl request!\n");
		ret = -EFAULT;
		goto done;
	}
	/* Fill request buffer */
	pcfg_11k = (mlan_ds_11k_cfg *) req->pbuf;
	pcfg_11k->sub_command = MLAN_OID_11K_CFG_ENABLE;
	req->req_id = MLAN_IOCTL_11K_CFG;
	req->action = MLAN_ACT_SET;
	if (nla_get_u32(tb_vendor[ATTR_ND_OFFLOAD_CONTROL]))
		pcfg_11k->param.enable_11k = MTRUE;
	else
		pcfg_11k->param.enable_11k = MFALSE;
	PRINTM(MCMND, "11k enable = %d\n", pcfg_11k->param.enable_11k);
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;

}

/**
 * @brief vendor command to set scan mac oui
 *
 * @param wiphy         A pointer to wiphy struct
 * @param wdev          A pointer to wireless_dev struct
 * @param data           a pointer to data
 * @param data_len     data length
 *
 * @return      0: success  <0: fail
 */
static int
woal_cfg80211_subcmd_set_scan_mac_oui(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int data_len)
{
	struct net_device *dev = NULL;
	moal_private *priv = NULL;
	struct nlattr *tb_vendor[ATTR_WIFI_MAX + 1];
	t_u8 mac_oui[3];
	int ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!wdev || !wdev->netdev) {
		LEAVE();
		return -EFAULT;
	}
	dev = wdev->netdev;
	priv = (moal_private *)woal_get_netdev_priv(dev);

	nla_parse(tb_vendor, ATTR_WIFI_MAX,
		  (struct nlattr *)data, data_len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
		  , NULL
#endif
		);
	if (!tb_vendor[ATTR_SCAN_MAC_OUI_SET]) {
		PRINTM(MINFO, "%s: ATTR_SCAN_MAC_OUI_SET not found\n",
		       __FUNCTION__);
		ret = -EFAULT;
		goto done;
	}
	memcpy(mac_oui, nla_data(tb_vendor[ATTR_SCAN_MAC_OUI_SET]), 3);
	memcpy(priv->random_mac, priv->current_addr, 6);
	memcpy(priv->random_mac, mac_oui, 3);
	PRINTM(MCMND, "random_mac is " FULL_MACSTR "\n",
	       FULL_MAC2STR(priv->random_mac));
done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to set enable/disable dfs offload
 *
 * @param wiphy       A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  1: fail
 */
static int
woal_cfg80211_subcmd_set_dfs_offload(struct wiphy *wiphy,
				     struct wireless_dev *wdev,
				     const void *data, int data_len)
{
	struct sk_buff *skb = NULL;
	int ret = 1;

	ENTER();

	/** Allocate skb for cmd reply*/
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(dfs_offload));
	if (!skb) {
		PRINTM(MERROR, "allocate memory fail for vendor cmd\n");
		ret = 1;
		LEAVE();
		return ret;
	}
	nla_put(skb, MRVL_WLAN_VENDOR_ATTR_DFS, sizeof(t_u32), &dfs_offload);
	ret = cfg80211_vendor_cmd_reply(skb);

	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get rtt capability
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_get_capa(struct wiphy *wiphy,
				  struct wireless_dev *wdev, const void *data,
				  int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	moal_handle *handle = priv->phandle;
	struct sk_buff *skb = NULL;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "CfgVendor: cfg80211_subcmd_rtt_get_capa\n");

	DBG_HEXDUMP(MCMD_D, "input data", (t_u8 *)data, len);

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
						  nla_total_size(sizeof
								 (handle->
								  rtt_capa)) +
						  VENDOR_REPLY_OVERHEAD);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed in %s\n", __FUNCTION__);
		goto done;
	}

	/* Put the attribute to the skb */
	nla_put(skb, ATTR_RTT_CAPA, sizeof(handle->rtt_capa),
		&(handle->rtt_capa));

	PRINTM(MCMND, "NL80211_CMD_VENDOR=0x%x\n", NL80211_CMD_VENDOR);
	PRINTM(MCMND, "NL80211_ATTR_WIPHY=0x%x\n", NL80211_ATTR_WIPHY);
	PRINTM(MCMND, "NL80211_ATTR_VENDOR_ID=0x%x\n", NL80211_ATTR_VENDOR_ID);
	PRINTM(MCMND, "NL80211_ATTR_VENDOR_SUBCMD=0x%x\n",
	       NL80211_ATTR_VENDOR_SUBCMD);
	PRINTM(MCMND, "NL80211_ATTR_VENDOR_DATA=0x%x\n",
	       NL80211_ATTR_VENDOR_DATA);
	PRINTM(MCMND, "NL80211_ATTR_VENDOR_EVENTS=0x%x\n",
	       NL80211_ATTR_VENDOR_EVENTS);

	DBG_HEXDUMP(MCMD_D, "output data skb->head", (t_u8 *)skb->head, 50);
	DBG_HEXDUMP(MCMD_D, "output data skb->data", (t_u8 *)skb->data, 50);
	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err))
		PRINTM(MERROR, "Vendor Command reply failed err:%d \n", err);

done:
	LEAVE();
	return err;
}

static void
woal_dump_rtt_params(wifi_rtt_config_params_t * rtt_params)
{
	int i = 0;

	PRINTM(MMSG, "===== Start DUMP RTT Params =====\n");
	PRINTM(MMSG, "rtt_config_num=%d\n\n", rtt_params->rtt_config_num);

	for (i = 0; i < rtt_params->rtt_config_num; i++) {
		PRINTM(MMSG, "----------[%d]----------\n", i);
		PRINTM(MMSG, "rtt_config[%d].addr=" MACSTR "\n", i,
		       MAC2STR(rtt_params->rtt_config[i].addr));
		PRINTM(MMSG, "rtt_config[%d].type=%d\n", i,
		       rtt_params->rtt_config[i].type);
		PRINTM(MMSG, "rtt_config[%d].peer=%d\n", i,
		       rtt_params->rtt_config[i].peer);
		PRINTM(MMSG, "rtt_config[%d].channel=[%d %d %d %d]\n", i,
		       rtt_params->rtt_config[i].channel.width,
		       rtt_params->rtt_config[i].channel.center_freq,
		       rtt_params->rtt_config[i].channel.center_freq0,
		       rtt_params->rtt_config[i].channel.center_freq1);
		PRINTM(MMSG, "rtt_config[%d].burst_period=%d\n", i,
		       rtt_params->rtt_config[i].burst_period);
		PRINTM(MMSG, "rtt_config[%d].num_burst=%d\n", i,
		       rtt_params->rtt_config[i].num_burst);
		PRINTM(MMSG, "rtt_config[%d].num_frames_per_burst=%d\n", i,
		       rtt_params->rtt_config[i].num_frames_per_burst);
		PRINTM(MMSG, "rtt_config[%d].num_retries_per_rtt_frame=%d\n", i,
		       rtt_params->rtt_config[i].num_retries_per_rtt_frame);
		PRINTM(MMSG, "rtt_config[%d].num_retries_per_ftmr=%d\n", i,
		       rtt_params->rtt_config[i].num_retries_per_ftmr);
		PRINTM(MMSG, "rtt_config[%d].LCI_request=%d\n", i,
		       rtt_params->rtt_config[i].LCI_request);
		PRINTM(MMSG, "rtt_config[%d].LCR_request=%d\n", i,
		       rtt_params->rtt_config[i].LCR_request);
		PRINTM(MMSG, "rtt_config[%d].burst_duration=%d\n", i,
		       rtt_params->rtt_config[i].burst_duration);
		PRINTM(MMSG, "rtt_config[%d].preamble=%d\n", i,
		       rtt_params->rtt_config[i].preamble);
		PRINTM(MMSG, "rtt_config[%d].bw=%d\n", i,
		       rtt_params->rtt_config[i].bw);
		PRINTM(MMSG, "\n");
	}
}

/**
 * @brief vendor command to request rtt range
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_range_request(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	moal_handle *handle = priv->phandle;
	struct nlattr *tb[ATTR_RTT_MAX];
	t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = { 0 };
	t_u8 rtt_config_num = 0;
	wifi_rtt_config *rtt_config = NULL;
	t_u8 i = 0, j = 0;
	wifi_rtt_config_params_t rtt_params;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_range_request()\n");

	err = nla_parse(tb, ATTR_RTT_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		err = -EFAULT;
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		goto done;
	}

	if (!tb[ATTR_RTT_TARGET_NUM] || !tb[ATTR_RTT_TARGET_CONFIG]) {
		PRINTM(MERROR,
		       "%s: null attr: tb[ATTR_RTT_TARGET_NUM]=%p tb[ATTR_RTT_TARGET_CONFIG]=%p\n",
		       __FUNCTION__, tb[ATTR_RTT_TARGET_NUM],
		       tb[ATTR_RTT_TARGET_CONFIG]);
		err = -EINVAL;
		goto done;
	}

	rtt_config_num = nla_get_u8(tb[ATTR_RTT_TARGET_NUM]);

	if ((rtt_config_num == 0) ||
	    ((handle->rtt_params.rtt_config_num + rtt_config_num) >
	     MAX_RTT_CONFIG_NUM)) {
		PRINTM(MERROR, "%s: invalid num=%d  num in handle=%d  MAX=%d\n",
		       __FUNCTION__, rtt_config_num,
		       handle->rtt_params.rtt_config_num, MAX_RTT_CONFIG_NUM);
		err = -EINVAL;
		goto done;
	}
	if (nla_len(tb[ATTR_RTT_TARGET_CONFIG]) !=
	    sizeof(rtt_params.rtt_config[0]) * rtt_config_num) {
		PRINTM(MERROR, "%s: invalid %d(total) != %d(num) * %lu(each)\n",
		       __FUNCTION__, nla_len(tb[ATTR_RTT_TARGET_CONFIG]),
		       rtt_config_num, sizeof(rtt_params.rtt_config[0]));
		err = -EINVAL;
		goto done;
	}

	rtt_config = (wifi_rtt_config *) nla_data(tb[ATTR_RTT_TARGET_CONFIG]);
	memset(&rtt_params, 0, sizeof(rtt_params));
	/** Strip the zero mac config */
	for (i = 0; i < rtt_config_num; i++) {
		if (!memcmp
		    (rtt_config[i].addr, zero_mac, sizeof(rtt_config[i].addr)))
			continue;
		else {
			memcpy(&rtt_params.
			       rtt_config[rtt_params.rtt_config_num],
			       &rtt_config[i],
			       sizeof(rtt_params.
				      rtt_config[rtt_params.rtt_config_num]));
			rtt_params.rtt_config_num++;
		}
	}
	if (!rtt_params.rtt_config_num) {
		PRINTM(MERROR, "%s: no valid mac addr\n", __FUNCTION__);
		goto done;
	}
	woal_dump_rtt_params(&rtt_params);

	ret = woal_config_rtt(priv, MOAL_IOCTL_WAIT, &rtt_params);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_config_rtt() failed\n", __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

	for (i = 0; i < rtt_params.rtt_config_num; i++) {
		for (j = 0; j < handle->rtt_params.rtt_config_num; j++) {
			if (!memcmp
			    (handle->rtt_params.rtt_config[j].addr,
			     rtt_params.rtt_config[i].addr,
			     sizeof(handle->rtt_params.rtt_config[j].addr)))
				break;
		}
		memcpy(&(handle->rtt_params.rtt_config[j]),
		       &(rtt_params.rtt_config[i]),
		       sizeof(handle->rtt_params.rtt_config[j]));
		if (j == handle->rtt_params.rtt_config_num)
			handle->rtt_params.rtt_config_num++;
	}

	woal_dump_rtt_params(&(handle->rtt_params));

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to cancel rtt range
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_range_cancel(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	moal_handle *handle = priv->phandle;
	t_u8 rtt_config_num = handle->rtt_params.rtt_config_num;
	struct nlattr *tb[ATTR_RTT_MAX];
	t_u32 target_num = 0;
	t_u8 addr[MAX_RTT_CONFIG_NUM][MLAN_MAC_ADDR_LENGTH];
	int i = 0, j = 0;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_range_cancel()\n");

	err = nla_parse(tb, ATTR_RTT_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		goto done;
	}

	if (!tb[ATTR_RTT_TARGET_NUM] || !tb[ATTR_RTT_TARGET_ADDR]) {
		PRINTM(MERROR,
		       "%s: null attr: tb[ATTR_RTT_TARGET_NUM]=%p tb[ATTR_RTT_TARGET_ADDR]=%p\n",
		       __FUNCTION__, tb[ATTR_RTT_TARGET_NUM],
		       tb[ATTR_RTT_TARGET_ADDR]);
		err = -EINVAL;
		goto done;
	}

	target_num = nla_get_u8(tb[ATTR_RTT_TARGET_NUM]);

	if ((target_num <= 0 || target_num > MAX_RTT_CONFIG_NUM) ||
	    (nla_len(tb[ATTR_RTT_TARGET_ADDR]) !=
	     sizeof(t_u8) * MLAN_MAC_ADDR_LENGTH * target_num)) {
		PRINTM(MERROR, "%s: Check if %din[1-%d] or %d*%lu=%d\n",
		       __FUNCTION__, target_num, MAX_RTT_CONFIG_NUM, target_num,
		       sizeof(t_u8) * MLAN_MAC_ADDR_LENGTH,
		       nla_len(tb[ATTR_RTT_TARGET_ADDR]));
		err = -EINVAL;
		goto done;
	}
	woal_dump_rtt_params(&(handle->rtt_params));

	memcpy(addr, nla_data(tb[ATTR_RTT_TARGET_ADDR]),
	       nla_len(tb[ATTR_RTT_TARGET_ADDR]));

	for (i = 0; i < target_num; i++)
		PRINTM(MMSG, "cancel[%d].addr=" MACSTR "\n", i,
		       MAC2STR(addr[i]));

	for (i = 0; i < target_num; i++) {
		for (j = 0; j < handle->rtt_params.rtt_config_num; j++) {
			if (!memcmp
			    (addr[i], handle->rtt_params.rtt_config[j].addr,
			     sizeof(addr[0]))) {
				memset(&(handle->rtt_params.rtt_config[j]),
				       0x00,
				       sizeof(handle->rtt_params.
					      rtt_config[0]));
				if ((j + 1) < handle->rtt_params.rtt_config_num) {
					memmove(&
						(handle->rtt_params.
						 rtt_config[j]),
						&(handle->rtt_params.
						  rtt_config[j + 1]),
						sizeof(handle->rtt_params.
						       rtt_config[0]) *
						(handle->rtt_params.
						 rtt_config_num - (j + 1)));
					memset(&
					       (handle->rtt_params.
						rtt_config[handle->rtt_params.
							   rtt_config_num - 1]),
					       0x00,
					       sizeof(handle->rtt_params.
						      rtt_config[0]));
				}
				handle->rtt_params.rtt_config_num--;
				continue;
			}
		}
	}

	if (handle->rtt_params.rtt_config_num >= rtt_config_num) {
		PRINTM(MERROR, "%s: No matched mac addr in rtt_config\n",
		       __FUNCTION__);
		goto done;
	}

	ret = woal_cancel_rtt(priv, MOAL_IOCTL_WAIT, target_num, addr);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_cancel_rtt() failed\n", __FUNCTION__);
		err = -EFAULT;
		goto done;
	}
	woal_dump_rtt_params(&(handle->rtt_params));

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor event to report RTT Results
 *
 * @param priv     A pointer to moal_private
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      mlan_status
 */
mlan_status
woal_cfg80211_event_rtt_result(IN moal_private *priv, IN t_u8 *data, IN int len)
{
	//moal_handle *handle = priv->phandle;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 *pos = data;
	t_u32 event_left_len = len;
	struct sk_buff *skb = NULL;
	t_u32 vdr_event_len = 0;
	t_u32 complete = 0;
	wifi_rtt_result_element *rtt_result_elem = NULL;
	t_u32 num_results = 0;

	ENTER();

	PRINTM(MEVENT, "Enter woal_cfg80211_event_rtt_result()\n");

	vdr_event_len = nla_total_size(sizeof(complete)) +
		nla_total_size(sizeof(num_results)) +
		nla_total_size(len) + NLA_ALIGNTO * num_results
		+ VENDOR_REPLY_OVERHEAD;
	PRINTM(MEVENT, "vdr_event_len = %d\n", vdr_event_len);
	skb = woal_cfg80211_alloc_vendor_event(priv, event_rtt_result,
					       vdr_event_len);
	if (!skb)
		goto done;

	complete = *pos;
	nla_put(skb, ATTR_RTT_RESULT_COMPLETE, sizeof(complete), &complete);
	pos++;
	event_left_len--;

	while (event_left_len > sizeof(wifi_rtt_result_element)) {
		rtt_result_elem = (wifi_rtt_result_element *) pos;

		nla_put(skb, ATTR_RTT_RESULT_FULL, rtt_result_elem->len,
			rtt_result_elem->data);
		num_results++;

		pos += sizeof(*rtt_result_elem) + rtt_result_elem->len;
		event_left_len -=
			sizeof(*rtt_result_elem) + rtt_result_elem->len;
	}

	nla_put(skb, ATTR_RTT_RESULT_NUM, sizeof(num_results), &num_results);

	DBG_HEXDUMP(MEVT_D, "output data skb->data", (t_u8 *)skb->data,
		    skb->len);
	/**send event*/
	cfg80211_vendor_event(skb, GFP_KERNEL);

done:
	LEAVE();
	return ret;
}

/**
 * @brief vendor command to get rtt responder info
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_get_responder_info(struct wiphy *wiphy,
					    struct wireless_dev *wdev,
					    const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_rtt_responder rtt_rsp_cfg;
	struct sk_buff *skb = NULL;
	wifi_rtt_responder rtt_rsp;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_get_responder_info()\n");

	memset(&rtt_rsp_cfg, 0x00, sizeof(rtt_rsp_cfg));
	rtt_rsp_cfg.action = RTT_GET_RESPONDER_INFO;
	ret = woal_rtt_responder_cfg(priv, MOAL_IOCTL_WAIT, &rtt_rsp_cfg);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_rtt_responder_cfg() failed\n",
		       __FUNCTION__);
		err = -EFAULT;
		goto done;
	}
	PRINTM(MCMD_D,
	       "mlan_rtt_responder from FW: channel=%d bandcfg=%d %d %d %d preamble=%d\n",
	       rtt_rsp_cfg.u.info.channel, rtt_rsp_cfg.u.info.bandcfg.chanBand,
	       rtt_rsp_cfg.u.info.bandcfg.chanWidth,
	       rtt_rsp_cfg.u.info.bandcfg.chan2Offset,
	       rtt_rsp_cfg.u.info.bandcfg.scanMode,
	       rtt_rsp_cfg.u.info.preamble);

	memset(&rtt_rsp, 0x00, sizeof(rtt_rsp));
	woal_bandcfg_to_channel_info(priv, &(rtt_rsp_cfg.u.info.bandcfg),
				     rtt_rsp_cfg.u.info.channel,
				     &(rtt_rsp.channel));
	rtt_rsp.preamble = rtt_rsp_cfg.u.info.preamble;
	PRINTM(MCMD_D, "wifi_rtt_responder report to HAL:\n");
	PRINTM(MCMD_D,
	       "channel: width=%d center_freq=%d center_freq0=%d center_freq1=%d\n",
	       rtt_rsp.channel.width, rtt_rsp.channel.center_freq,
	       rtt_rsp.channel.center_freq0, rtt_rsp.channel.center_freq1);
	PRINTM(MCMD_D, "preamble=%d\n", rtt_rsp.preamble);

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
						  nla_total_size(sizeof
								 (rtt_rsp)) +
						  VENDOR_REPLY_OVERHEAD);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed in %s\n", __FUNCTION__);
		goto done;
	}

	/* Put the attribute to the skb */
	nla_put(skb, ATTR_RTT_CHANNEL_INFO, sizeof(rtt_rsp.channel),
		&(rtt_rsp.channel));
	nla_put(skb, ATTR_RTT_PREAMBLE, sizeof(rtt_rsp.preamble),
		&(rtt_rsp.preamble));
	DBG_HEXDUMP(MCMD_D, "output data skb->data", (t_u8 *)skb->data,
		    skb->len);

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err))
		PRINTM(MERROR, "Vendor Command reply failed err:%d \n", err);

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to enable rtt responder
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_enable_responder(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct nlattr *tb[ATTR_RTT_MAX];
	wifi_channel_info *ch_info = NULL;
	t_u32 max_dur_sec = 0;
	mlan_rtt_responder rtt_rsp_cfg;
	wifi_rtt_responder rtt_rsp;
	struct sk_buff *skb = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_enable_responder()\n");

	err = nla_parse(tb, ATTR_RTT_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		err = -EFAULT;
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		goto done;
	}

	if (!tb[ATTR_RTT_CHANNEL_INFO] || !tb[ATTR_RTT_MAX_DUR_SEC]) {
		PRINTM(MERROR,
		       "%s: null attr: tb[ATTR_RTT_TARGET_NUM]=%p tb[ATTR_RTT_TARGET_CONFIG]=%p\n",
		       __FUNCTION__, tb[ATTR_RTT_CHANNEL_INFO],
		       tb[ATTR_RTT_MAX_DUR_SEC]);
		err = -EINVAL;
		goto done;
	}
	ch_info = (wifi_channel_info *) nla_data(tb[ATTR_RTT_CHANNEL_INFO]);
	max_dur_sec = nla_get_u32(tb[ATTR_RTT_MAX_DUR_SEC]);
	PRINTM(MCMD_D, "HAL input: \n");
	PRINTM(MCMD_D,
	       "wifi_channel_info: width=%d center_freq=%d center_freq0=%d center_freq1=%d\n",
	       ch_info->width, ch_info->center_freq, ch_info->center_freq0,
	       ch_info->center_freq1);
	PRINTM(MCMD_D, "max_dur_sec=%d\n", max_dur_sec);

	memset(&rtt_rsp_cfg, 0x00, sizeof(rtt_rsp_cfg));
	rtt_rsp_cfg.action = RTT_SET_RESPONDER_ENABLE;
	rtt_rsp_cfg.u.encfg.channel =
		ieee80211_frequency_to_channel(ch_info->center_freq);
	woal_channel_info_to_bandcfg(priv, ch_info,
				     &(rtt_rsp_cfg.u.encfg.bandcfg));
	rtt_rsp_cfg.u.encfg.max_dur_sec = max_dur_sec;
	PRINTM(MCMD_D, "HAL input to rtt_responder_encfg: \n");
	PRINTM(MCMD_D,
	       "channel=%d bandcfg=[chanBand=%d chanWidth=%d chan2Offset=%d scanMode=%d]\n",
	       rtt_rsp_cfg.u.encfg.channel,
	       rtt_rsp_cfg.u.encfg.bandcfg.chanBand,
	       rtt_rsp_cfg.u.encfg.bandcfg.chanWidth,
	       rtt_rsp_cfg.u.encfg.bandcfg.chan2Offset,
	       rtt_rsp_cfg.u.encfg.bandcfg.scanMode);
	PRINTM(MCMD_D, "max_dur_sec=%d\n", rtt_rsp_cfg.u.encfg.max_dur_sec);
	ret = woal_rtt_responder_cfg(priv, MOAL_IOCTL_WAIT, &rtt_rsp_cfg);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_rtt_responder_cfg() failed\n",
		       __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

	memset(&rtt_rsp, 0x00, sizeof(rtt_rsp));
	woal_bandcfg_to_channel_info(priv, &(rtt_rsp_cfg.u.info.bandcfg),
				     rtt_rsp_cfg.u.info.channel,
				     &(rtt_rsp.channel));
	rtt_rsp.preamble = rtt_rsp_cfg.u.info.preamble;
	PRINTM(MCMD_D, "wifi_rtt_responder report to HAL:\n");
	PRINTM(MCMD_D,
	       "channel: width=%d center_freq=%d center_freq0=%d center_freq1=%d\n",
	       rtt_rsp.channel.width, rtt_rsp.channel.center_freq,
	       rtt_rsp.channel.center_freq0, rtt_rsp.channel.center_freq1);
	PRINTM(MCMD_D, "preamble=%d\n", rtt_rsp.preamble);

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
						  nla_total_size(sizeof
								 (rtt_rsp)) +
						  VENDOR_REPLY_OVERHEAD);
	if (unlikely(!skb)) {
		PRINTM(MERROR, "skb alloc failed in %s\n", __FUNCTION__);
		goto done;
	}

	/* Put the attribute to the skb */
	nla_put(skb, ATTR_RTT_CHANNEL_INFO, sizeof(rtt_rsp.channel),
		&(rtt_rsp.channel));
	nla_put(skb, ATTR_RTT_PREAMBLE, sizeof(rtt_rsp.preamble),
		&(rtt_rsp.preamble));
	DBG_HEXDUMP(MCMD_D, "output data skb->data", (t_u8 *)skb->data,
		    skb->len);

	err = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(err))
		PRINTM(MERROR, "Vendor Command reply failed err:%d \n", err);

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to disable rtt responder
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_disable_responder(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data, int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_rtt_responder rtt_rsp_cfg;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_disable_responder()\n");

	memset(&rtt_rsp_cfg, 0x00, sizeof(rtt_rsp_cfg));
	rtt_rsp_cfg.action = RTT_SET_RESPONDER_DISABLE;
	ret = woal_rtt_responder_cfg(priv, MOAL_IOCTL_WAIT, &rtt_rsp_cfg);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_rtt_responder_cfg() failed\n",
		       __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to set rtt lci
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_set_lci(struct wiphy *wiphy,
				 struct wireless_dev *wdev, const void *data,
				 int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct nlattr *tb[ATTR_RTT_MAX];
	mlan_rtt_responder rtt_rsp_cfg;
	wifi_lci_information *lci_info;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_set_lci()\n");

	err = nla_parse(tb, ATTR_RTT_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		err = -EFAULT;
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		goto done;
	}

	if (!tb[ATTR_RTT_LCI_INFO]) {
		PRINTM(MERROR, "%s: null attr: tb[ATTR_RTT_LCI_INFO]=%p\n",
		       __FUNCTION__, tb[ATTR_RTT_LCI_INFO]);
		err = -EINVAL;
		goto done;
	}
	lci_info = (wifi_lci_information *) nla_data(tb[ATTR_RTT_LCI_INFO]);
	PRINTM(MCMD_D, "HAL input: \n");
	PRINTM(MCMD_D,
	       "wifi_lci_information: latitude=%lu longitude=%lu altitude=%d latitude_unc=%d longitude_unc=%d altitude_unc=%d \n",
	       lci_info->latitude, lci_info->longitude, lci_info->altitude,
	       lci_info->latitude_unc, lci_info->longitude_unc,
	       lci_info->altitude_unc);
	PRINTM(MCMD_D,
	       "wifi_lci_information: motion_pattern=%d floor=%d height_above_floor=%d height_unc=%d\n",
	       lci_info->motion_pattern, lci_info->floor,
	       lci_info->height_above_floor, lci_info->height_unc);

	memset(&rtt_rsp_cfg, 0x00, sizeof(rtt_rsp_cfg));
	rtt_rsp_cfg.action = RTT_SET_RESPONDER_LCI;
	memcpy(&(rtt_rsp_cfg.u.lci), lci_info, sizeof(rtt_rsp_cfg.u.lci));
	ret = woal_rtt_responder_cfg(priv, MOAL_IOCTL_WAIT, &rtt_rsp_cfg);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_rtt_responder_cfg() failed\n",
		       __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return err;
}

/**
 * @brief vendor command to set rtt lcr
 *
 * @param wiphy    A pointer to wiphy struct
 * @param wdev     A pointer to wireless_dev struct
 * @param data     a pointer to data
 * @param  len     data length
 *
 * @return      0: success  -1: fail
 */
static int
woal_cfg80211_subcmd_rtt_set_lcr(struct wiphy *wiphy,
				 struct wireless_dev *wdev, const void *data,
				 int len)
{
	struct net_device *dev = wdev->netdev;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct nlattr *tb[ATTR_RTT_MAX];
	mlan_rtt_responder rtt_rsp_cfg;
	wifi_lcr_information *lcr_info;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	int err = 0;

	ENTER();
	PRINTM(MCMND, "Enter woal_cfg80211_subcmd_rtt_set_lcr()\n");

	err = nla_parse(tb, ATTR_RTT_MAX, data, len, NULL
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
			, NULL
#endif
		);
	if (err) {
		err = -EFAULT;
		PRINTM(MERROR, "%s: nla_parse fail\n", __FUNCTION__);
		goto done;
	}

	if (!tb[ATTR_RTT_LCR_INFO]) {
		PRINTM(MERROR, "%s: null attr: tb[ATTR_RTT_LCR_INFO]=%p\n",
		       __FUNCTION__, tb[ATTR_RTT_LCR_INFO]);
		err = -EINVAL;
		goto done;
	}
	lcr_info = (wifi_lcr_information *) nla_data(tb[ATTR_RTT_LCR_INFO]);
	PRINTM(MCMD_D, "HAL input: \n");
	PRINTM(MCMD_D, "wifi_lcr_information: country_code='%c' '%c'\n",
	       lcr_info->country_code[0], lcr_info->country_code[1]);
	PRINTM(MCMD_D, "wifi_lci_information: length=%d civic_info=%s\n",
	       lcr_info->length, lcr_info->civic_info);

	memset(&rtt_rsp_cfg, 0x00, sizeof(rtt_rsp_cfg));
	rtt_rsp_cfg.action = RTT_SET_RESPONDER_LCR;
	memcpy(&(rtt_rsp_cfg.u.lcr), lcr_info, sizeof(rtt_rsp_cfg.u.lcr));
	ret = woal_rtt_responder_cfg(priv, MOAL_IOCTL_WAIT, &rtt_rsp_cfg);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: woal_rtt_responder_cfg() failed\n",
		       __FUNCTION__);
		err = -EFAULT;
		goto done;
	}

done:
	LEAVE();
	return err;
}

const struct wiphy_vendor_command vendor_commands[] = {
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = sub_cmd_set_drvdbg,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_set_drvdbg,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_valid_channels,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_valid_channels,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_set_scan_mac_oui,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_set_scan_mac_oui,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_link_statistic_set,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_link_statistic_set,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_link_statistic_get,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_link_statistic_get,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_link_statistic_clr,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_link_statistic_clr,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
#ifdef STA_CFG80211
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = sub_cmd_rssi_monitor,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rssi_monitor,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
#endif
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_set_roaming_offload_key,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_set_roaming_offload_key,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_roaming_capability,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_roaming_capability,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_fw_roaming_enable,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_fw_roaming_enable,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_fw_roaming_config,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_fw_roaming_config,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },

	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_dfs_capability,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_set_dfs_offload,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },

	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_correlated_time,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_correlated_time,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },

	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = SUBCMD_RTT_GET_CAPA,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_get_capa,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  SUBCMD_RTT_RANGE_REQUEST,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_range_request,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  SUBCMD_RTT_RANGE_CANCEL,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_range_cancel,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  SUBCMD_RTT_GET_RESPONDER_INFO,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_get_responder_info,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  SUBCMD_RTT_ENABLE_RESPONDER,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_enable_responder,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  SUBCMD_RTT_DISABLE_RESPONDER,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_disable_responder,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = SUBCMD_RTT_SET_LCI,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_set_lci,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = SUBCMD_RTT_SET_LCR,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_rtt_set_lcr,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },

	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = sub_cmd_nd_offload},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_11k_cfg,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_drv_version,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_drv_version,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_fw_version,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_fw_version,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_wifi_supp_feature_set,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_supp_feature_set,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_set_country_code,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_set_country_code,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },

	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_wifi_logger_supp_feature_set,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
	 .doit = woal_cfg80211_subcmd_get_wifi_logger_supp_feature_set,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_ring_buff_status,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_get_ring_buff_status,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = sub_cmd_start_logging,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_start_logging,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_ring_buff_data,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_get_ring_data,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_start_packet_fate_monitor,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_start_packet_fate_monitor,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_fw_mem_dump,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_get_fw_dump,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_drv_mem_dump,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_get_drv_dump,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_set_packet_filter,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_set_packet_filter,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  sub_cmd_get_packet_filter_capability,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_get_packet_filter_capability,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_enable_req,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_enable_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_disable_req,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_disable_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_publish_req,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_publish_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_publish_cancel,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_publish_cancel,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_subscribe_req,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_subscribe_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_subscribe_cancel,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_subscribe_cancel,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_trasmit_followup,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_trasmit_followup,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_stats_req,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_stats_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_config_req,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_config_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_tca_req,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_tca_req,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_beacon_sdf_payload,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_beacon_sdf_payload,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_get_version,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_get_version,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_get_capa,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_get_capability,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_data_if_create,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_data_if_create,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_data_if_delete,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_data_if_delete,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_data_req_initor,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_data_req_initor,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd =
		  subcmd_nan_data_indi_resp,},
	 .flags =
	 WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV |
	 WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_data_indi_resp,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
	{
	 .info = {.vendor_id = MRVL_VENDOR_ID,.subcmd = subcmd_nan_data_end,},
	 .flags = WIPHY_VENDOR_CMD_NEED_WDEV |
	 WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	 .doit = woal_cfg80211_subcmd_nan_data_end,
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	 .policy = VENDOR_CMD_RAW_DATA,
#endif
	 },
};

/**
 * @brief register vendor commands and events
 *
 * @param wiphy       A pointer to wiphy struct
 *
 * @return
 */
void
woal_register_cfg80211_vendor_command(struct wiphy *wiphy)
{
	ENTER();
	wiphy->vendor_commands = vendor_commands;
	wiphy->n_vendor_commands = ARRAY_SIZE(vendor_commands);
	wiphy->vendor_events = vendor_events;
	wiphy->n_vendor_events = ARRAY_SIZE(vendor_events);
	LEAVE();
}
#endif
