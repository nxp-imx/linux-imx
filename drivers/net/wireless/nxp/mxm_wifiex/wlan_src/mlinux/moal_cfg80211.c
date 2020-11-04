/** @file moal_cfg80211.c
 *
 * @brief This file contains the functions for CFG80211.
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

#include "moal_cfg80211.h"
#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
#include "moal_uap.h"
#endif
#endif

/********************************************************
 *				Local Variables
 ********************************************************/
/** Supported rates to be advertised to the cfg80211 */
static struct ieee80211_rate cfg80211_rates[] = {
	{
		.bitrate = 10,
		.hw_value = 2,
	},
	{
		.bitrate = 20,
		.hw_value = 4,
	},
	{
		.bitrate = 55,
		.hw_value = 11,
	},
	{
		.bitrate = 110,
		.hw_value = 22,
	},
	{
		.bitrate = 60,
		.hw_value = 12,
	},
	{
		.bitrate = 90,
		.hw_value = 18,
	},
	{
		.bitrate = 120,
		.hw_value = 24,
	},
	{
		.bitrate = 180,
		.hw_value = 36,
	},
	{
		.bitrate = 240,
		.hw_value = 48,
	},
	{
		.bitrate = 360,
		.hw_value = 72,
	},
	{
		.bitrate = 480,
		.hw_value = 96,
	},
	{
		.bitrate = 540,
		.hw_value = 108,
	},
};

/** Channel definitions for 2 GHz to be advertised to cfg80211 */
static struct ieee80211_channel cfg80211_channels_2ghz[] = {
	{.center_freq = 2412, .hw_value = 1, .max_power = 20},
	{.center_freq = 2417, .hw_value = 2, .max_power = 20},
	{.center_freq = 2422, .hw_value = 3, .max_power = 20},
	{.center_freq = 2427, .hw_value = 4, .max_power = 20},
	{.center_freq = 2432, .hw_value = 5, .max_power = 20},
	{.center_freq = 2437, .hw_value = 6, .max_power = 20},
	{.center_freq = 2442, .hw_value = 7, .max_power = 20},
	{.center_freq = 2447, .hw_value = 8, .max_power = 20},
	{.center_freq = 2452, .hw_value = 9, .max_power = 20},
	{.center_freq = 2457, .hw_value = 10, .max_power = 20},
	{.center_freq = 2462, .hw_value = 11, .max_power = 20},
	{.center_freq = 2467, .hw_value = 12, .max_power = 20},
	{.center_freq = 2472, .hw_value = 13, .max_power = 20},
	{.center_freq = 2484, .hw_value = 14, .max_power = 20},
};

/** Channel definitions for 5 GHz to be advertised to cfg80211 */
static struct ieee80211_channel cfg80211_channels_5ghz[] = {
	{.center_freq = 5180, .hw_value = 36, .max_power = 20},
	{.center_freq = 5200, .hw_value = 40, .max_power = 20},
	{.center_freq = 5220, .hw_value = 44, .max_power = 20},
	{.center_freq = 5240, .hw_value = 48, .max_power = 20},
	{.center_freq = 5260, .hw_value = 52, .max_power = 20},
	{.center_freq = 5280, .hw_value = 56, .max_power = 20},
	{.center_freq = 5300, .hw_value = 60, .max_power = 20},
	{.center_freq = 5320, .hw_value = 64, .max_power = 20},
	{.center_freq = 5500, .hw_value = 100, .max_power = 20},
	{.center_freq = 5520, .hw_value = 104, .max_power = 20},
	{.center_freq = 5540, .hw_value = 108, .max_power = 20},
	{.center_freq = 5560, .hw_value = 112, .max_power = 20},
	{.center_freq = 5580, .hw_value = 116, .max_power = 20},
	{.center_freq = 5600, .hw_value = 120, .max_power = 20},
	{.center_freq = 5620, .hw_value = 124, .max_power = 20},
	{.center_freq = 5640, .hw_value = 128, .max_power = 20},
	{.center_freq = 5660, .hw_value = 132, .max_power = 20},
	{.center_freq = 5680, .hw_value = 136, .max_power = 20},
	{.center_freq = 5700, .hw_value = 140, .max_power = 20},
	{.center_freq = 5720, .hw_value = 144, .max_power = 20},
	{.center_freq = 5745, .hw_value = 149, .max_power = 20},
	{.center_freq = 5765, .hw_value = 153, .max_power = 20},
	{.center_freq = 5785, .hw_value = 157, .max_power = 20},
	{.center_freq = 5805, .hw_value = 161, .max_power = 20},
	{.center_freq = 5825, .hw_value = 165, .max_power = 20},
};

struct ieee80211_supported_band cfg80211_band_2ghz = {
	.channels = cfg80211_channels_2ghz,
	.band = IEEE80211_BAND_2GHZ,
	.n_channels = ARRAY_SIZE(cfg80211_channels_2ghz),
	.bitrates = cfg80211_rates,
	.n_bitrates = ARRAY_SIZE(cfg80211_rates),
};

struct ieee80211_supported_band cfg80211_band_5ghz = {
	.channels = cfg80211_channels_5ghz,
	.band = IEEE80211_BAND_5GHZ,
	.n_channels = ARRAY_SIZE(cfg80211_channels_5ghz),
	.bitrates = cfg80211_rates + 4,
	.n_bitrates = ARRAY_SIZE(cfg80211_rates) - 4,
};

/** Channel definitions for 5 GHz to be advertised to cfg80211 */
static struct ieee80211_channel mac1_cfg80211_channels_5ghz[] = {
	{.center_freq = 5180, .hw_value = 36, .max_power = 20},
	{.center_freq = 5200, .hw_value = 40, .max_power = 20},
	{.center_freq = 5220, .hw_value = 44, .max_power = 20},
	{.center_freq = 5240, .hw_value = 48, .max_power = 20},
	{.center_freq = 5260, .hw_value = 52, .max_power = 20},
	{.center_freq = 5280, .hw_value = 56, .max_power = 20},
	{.center_freq = 5300, .hw_value = 60, .max_power = 20},
	{.center_freq = 5320, .hw_value = 64, .max_power = 20},
	{.center_freq = 5500, .hw_value = 100, .max_power = 20},
	{.center_freq = 5520, .hw_value = 104, .max_power = 20},
	{.center_freq = 5540, .hw_value = 108, .max_power = 20},
	{.center_freq = 5560, .hw_value = 112, .max_power = 20},
	{.center_freq = 5580, .hw_value = 116, .max_power = 20},
	{.center_freq = 5600, .hw_value = 120, .max_power = 20},
	{.center_freq = 5620, .hw_value = 124, .max_power = 20},
	{.center_freq = 5640, .hw_value = 128, .max_power = 20},
	{.center_freq = 5660, .hw_value = 132, .max_power = 20},
	{.center_freq = 5680, .hw_value = 136, .max_power = 20},
	{.center_freq = 5700, .hw_value = 140, .max_power = 20},
	{.center_freq = 5720, .hw_value = 144, .max_power = 20},
	{.center_freq = 5745, .hw_value = 149, .max_power = 20},
	{.center_freq = 5765, .hw_value = 153, .max_power = 20},
	{.center_freq = 5785, .hw_value = 157, .max_power = 20},
	{.center_freq = 5805, .hw_value = 161, .max_power = 20},
	{.center_freq = 5825, .hw_value = 165, .max_power = 20},
};

struct ieee80211_supported_band mac1_cfg80211_band_2ghz = {
	.channels = cfg80211_channels_2ghz,
	.band = IEEE80211_BAND_2GHZ,
	.n_channels = ARRAY_SIZE(cfg80211_channels_2ghz),
	.bitrates = cfg80211_rates,
	.n_bitrates = ARRAY_SIZE(cfg80211_rates),
};

struct ieee80211_supported_band mac1_cfg80211_band_5ghz = {
	.channels = mac1_cfg80211_channels_5ghz,
	.band = IEEE80211_BAND_5GHZ,
	.n_channels = ARRAY_SIZE(mac1_cfg80211_channels_5ghz),
	.bitrates = cfg80211_rates + 4,
	.n_bitrates = ARRAY_SIZE(cfg80211_rates) - 4,
};

#if KERNEL_VERSION(2, 6, 29) < LINUX_VERSION_CODE
#ifdef UAP_SUPPORT
/** Network device handlers for uAP */
extern const struct net_device_ops woal_uap_netdev_ops;
#endif
#ifdef STA_SUPPORT
/** Network device handlers for STA */
extern const struct net_device_ops woal_netdev_ops;
#endif
#endif
/********************************************************
 *				Local Functions
 ********************************************************/

/********************************************************
 *				Global Functions
 ********************************************************/

/**
 * @brief Get the private structure from wiphy
 *
 * @param wiphy     A pointer to wiphy structure
 *
 * @return          Pointer to moal_private
 */
void *woal_get_wiphy_priv(struct wiphy *wiphy)
{
	return (void *)(*(unsigned long *)wiphy_priv(wiphy));
}

/**
 * @brief Get the private structure from net device
 *
 * @param dev       A pointer to net_device structure
 *
 * @return          Pointer to moal_private
 */
void *woal_get_netdev_priv(struct net_device *dev)
{
	return (void *)netdev_priv(dev);
}

/**
 *  @brief Get current frequency of active interface
 *
 *  @param priv        A pointer to moal_private
 *
 *  @return              channel frequency
 */
int woal_get_active_intf_freq(moal_private *priv)
{
	moal_handle *handle = priv->phandle;
	int i;

	if (priv->media_connected == MTRUE
#ifdef UAP_SUPPORT
	    || priv->bss_started == MTRUE
#endif
	)
		return ieee80211_channel_to_frequency(
			priv->channel
#if KERNEL_VERSION(2, 6, 39) <= CFG80211_VERSION_CODE
			,
			(priv->channel <= 14 ? IEEE80211_BAND_2GHZ :
					       IEEE80211_BAND_5GHZ)
#endif
		);

	for (i = 0; i < handle->priv_num; i++) {
#ifdef STA_SUPPORT
		if (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_STA) {
			if (handle->priv[i]->media_connected == MTRUE)
				return ieee80211_channel_to_frequency(
					handle->priv[i]->channel
#if KERNEL_VERSION(2, 6, 39) <= CFG80211_VERSION_CODE
					,
					(handle->priv[i]->channel <= 14 ?
						 IEEE80211_BAND_2GHZ :
						 IEEE80211_BAND_5GHZ)
#endif
				);
		}
#endif
#ifdef UAP_SUPPORT
		if (GET_BSS_ROLE(handle->priv[i]) == MLAN_BSS_ROLE_UAP) {
			if (handle->priv[i]->bss_started == MTRUE)
				return ieee80211_channel_to_frequency(
					handle->priv[i]->channel
#if KERNEL_VERSION(2, 6, 39) <= CFG80211_VERSION_CODE
					,
					(handle->priv[i]->channel <= 14 ?
						 IEEE80211_BAND_2GHZ :
						 IEEE80211_BAND_5GHZ)
#endif
				);
		}
#endif
	}
	return 0;
}

/**
 *  @brief Convert driver band configuration to IEEE band type
 *
 *  @param band     Driver band configuration
 *
 *  @return         IEEE band type
 */
t_u8 woal_band_cfg_to_ieee_band(t_u32 band)
{
	t_u8 ret_radio_type;

	ENTER();

	switch (band) {
	case BAND_A:
	case BAND_AN:
	case BAND_A | BAND_AN:
		ret_radio_type = IEEE80211_BAND_5GHZ;
		break;
	case BAND_B:
	case BAND_G:
	case BAND_B | BAND_G:
	case BAND_GN:
	case BAND_B | BAND_GN:
	/* Fall Through */
	default:
		ret_radio_type = IEEE80211_BAND_2GHZ;
		break;
	}

	LEAVE();
	return ret_radio_type;
}

/**
 *  @brief Set/Enable encryption key
 *
 *  @param priv             A pointer to moal_private structure
 *  @param is_enable_wep    Enable WEP default key
 *  @param cipher           Cipher suite selector
 *  @param key              A pointer to key
 *  @param key_len          Key length
 *  @param seq              A pointer to sequence
 *  @param seq_len          Sequence length
 *  @param key_index        Key index
 *  @param addr             Mac for which key is to be set
 *  @param disable          Key disabled or not
 *  @param wait_option      wait option
 *
 *  @return                 MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_cfg80211_set_key(moal_private *priv, t_u8 is_enable_wep,
				  t_u32 cipher, const t_u8 *key, int key_len,
				  const t_u8 *seq, int seq_len, t_u8 key_index,
				  const t_u8 *addr, int disable,
				  t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 bcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	ENTER();

#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		if (is_enable_wep) {
			PRINTM(MIOCTL, "Enable UAP default key=%d\n",
			       key_index);
			priv->uap_wep_key[key_index].is_default = MTRUE;
			goto done;
		}
		if (key && key_len &&
		    ((cipher == WLAN_CIPHER_SUITE_WEP40) ||
		     (cipher == WLAN_CIPHER_SUITE_WEP104))) {
			priv->uap_wep_key[key_index].length = key_len;
			moal_memcpy_ext(
				priv->phandle, priv->uap_wep_key[key_index].key,
				key, key_len,
				sizeof(priv->uap_wep_key[key_index].key));
			priv->cipher = cipher;
			priv->uap_wep_key[key_index].key_index = key_index;
			priv->uap_wep_key[key_index].is_default = MFALSE;
			PRINTM(MIOCTL, "Set UAP WEP key: key_index=%d len=%d\n",
			       key_index, key_len);
			goto done;
		}
	}
#endif
#endif

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ENCRYPT_KEY;
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_SET;

	if (is_enable_wep) {
		sec->param.encrypt_key.key_index = key_index;
		sec->param.encrypt_key.is_current_wep_key = MTRUE;
	} else if (!disable) {
		if (cipher != WLAN_CIPHER_SUITE_WEP40 &&
		    cipher != WLAN_CIPHER_SUITE_WEP104 &&
		    cipher != WLAN_CIPHER_SUITE_TKIP &&
		    cipher != WLAN_CIPHER_SUITE_SMS4 &&
		    cipher != WLAN_CIPHER_SUITE_AES_CMAC &&
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
		    cipher != WLAN_CIPHER_SUITE_CCMP_256 &&
#endif
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
		    cipher != WLAN_CIPHER_SUITE_GCMP &&
#endif
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
		    cipher != WLAN_CIPHER_SUITE_BIP_GMAC_256 &&
		    cipher != WLAN_CIPHER_SUITE_GCMP_256 &&
#endif
		    cipher != WLAN_CIPHER_SUITE_CCMP) {
			PRINTM(MERROR, "Invalid cipher suite specified\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		sec->param.encrypt_key.key_index = key_index;
		if (key && key_len) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.key_material,
					key, key_len, MLAN_MAX_KEY_LENGTH);
			sec->param.encrypt_key.key_len = key_len;
		}
		/* Set WAPI key */
		if (cipher == WLAN_CIPHER_SUITE_SMS4) {
			sec->param.encrypt_key.is_wapi_key = MTRUE;
			if (seq_len) {
				moal_memcpy_ext(priv->phandle,
						sec->param.encrypt_key.pn, seq,
						PN_SIZE, PN_SIZE);
				DBG_HEXDUMP(MCMD_D, "WAPI PN",
					    sec->param.encrypt_key.pn, seq_len);
			}
		}
		if (addr) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr, addr,
					ETH_ALEN, MLAN_MAC_ADDR_LENGTH);
			if (memcmp(sec->param.encrypt_key.mac_addr, bcast_addr,
				   ETH_ALEN) == 0)
				sec->param.encrypt_key.key_flags =
					KEY_FLAG_GROUP_KEY;
			else
				sec->param.encrypt_key.key_flags =
					KEY_FLAG_SET_TX_KEY;
		} else {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr,
					bcast_addr, ETH_ALEN,
					MLAN_MAC_ADDR_LENGTH);
			sec->param.encrypt_key.key_flags =
				KEY_FLAG_GROUP_KEY | KEY_FLAG_SET_TX_KEY;
		}
		if (seq && seq_len) {
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.pn, seq, seq_len,
					PN_SIZE);
			sec->param.encrypt_key.key_flags |=
				KEY_FLAG_RX_SEQ_VALID;
		}

#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
		if (cipher == WLAN_CIPHER_SUITE_GCMP)
			sec->param.encrypt_key.key_flags |= KEY_FLAG_GCMP;
#endif
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
		else if (cipher == WLAN_CIPHER_SUITE_GCMP_256)
			sec->param.encrypt_key.key_flags |= KEY_FLAG_GCMP_256;
#endif
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
		if (cipher == WLAN_CIPHER_SUITE_CCMP_256)
			sec->param.encrypt_key.key_flags |= KEY_FLAG_CCMP_256;
#endif

		if (cipher == WLAN_CIPHER_SUITE_AES_CMAC
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
		    || cipher == WLAN_CIPHER_SUITE_BIP_GMAC_256
#endif
		) {
			sec->param.encrypt_key.key_flags |=
				KEY_FLAG_AES_MCAST_IGTK;
		}
	} else {
		if (key_index == KEY_INDEX_CLEAR_ALL)
			sec->param.encrypt_key.key_disable = MTRUE;
		else {
			sec->param.encrypt_key.key_remove = MTRUE;
			sec->param.encrypt_key.key_index = key_index;
		}
		sec->param.encrypt_key.key_flags = KEY_FLAG_REMOVE_KEY;
		if (addr)
			moal_memcpy_ext(priv->phandle,
					sec->param.encrypt_key.mac_addr, addr,
					ETH_ALEN, MLAN_MAC_ADDR_LENGTH);
	}

	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, wait_option);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Enable the WEP key to driver
 *
 * @param priv      A pointer to moal_private structure
 * @param key       A pointer to key data
 * @param key_len   Length of the key data
 * @param index     Key index
 * @param wait_option wait_option
 *
 * @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_cfg80211_set_wep_keys(moal_private *priv, const t_u8 *key,
				       int key_len, t_u8 index,
				       t_u8 wait_option)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 cipher = 0;

	ENTER();

	if (key_len) {
		if (key_len == 5)
			cipher = WLAN_CIPHER_SUITE_WEP40;
		else
			cipher = WLAN_CIPHER_SUITE_WEP104;
		ret = woal_cfg80211_set_key(priv, 0, cipher, key, key_len, NULL,
					    0, index, NULL, 0, wait_option);
	} else {
		/* No key provided so it is enable key. We
		 * want to just set the transmit key index
		 */
		woal_cfg80211_set_key(priv, 1, cipher, key, key_len, NULL, 0,
				      index, NULL, 0, wait_option);
	}

	LEAVE();
	return ret;
}

/**
 * @brief clear all mgmt ies
 *
 * @param priv              A pointer to moal private structure
 * @param wait_option       wait_option
 * @return                  N/A
 */
void woal_clear_all_mgmt_ies(moal_private *priv, t_u8 wait_option)
{
	t_u16 mask = 0;
	/* clear BEACON WPS/P2P IE */
	if (priv->beacon_wps_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		PRINTM(MCMND, "Clear BEACON WPS ie\n");
		woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0, NULL, 0,
					    NULL, 0, MGMT_MASK_BEACON_WPS_P2P,
					    wait_option);
		priv->beacon_wps_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	}
	if (priv->assocresp_qos_map_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		PRINTM(MCMND, "Clear associate response QOS map ie\n");
		woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0, NULL, 0,
					    NULL, 0,
					    MGMT_MASK_ASSOC_RESP_QOS_MAP,
					    wait_option);
		priv->assocresp_qos_map_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	}
	/* clear mgmt frame ies */
	if (priv->probereq_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_PROBE_REQ;
	if (priv->beacon_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_BEACON;
	if (priv->proberesp_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_PROBE_RESP;
	if (priv->assocresp_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_ASSOC_RESP;
	if (priv->proberesp_p2p_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_PROBE_RESP;
	if (priv->beacon_vendor_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK)
		mask |= MGMT_MASK_BEACON;
	if (mask) {
		PRINTM(MCMND, "Clear IES: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		       priv->beacon_index, priv->probereq_index,
		       priv->proberesp_index, priv->assocresp_index,
		       priv->proberesp_p2p_index, priv->beacon_vendor_index);
		woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0, NULL, 0,
					    NULL, 0, mask, wait_option);
	}
	priv->probereq_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	priv->beacon_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	priv->proberesp_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	priv->assocresp_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	priv->proberesp_p2p_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
	priv->beacon_vendor_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
}

#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
/**
 * @brief set bss role
 *
 * @param priv              A pointer to moal private structure
 * @param action            Action: set or get
 * @param role              A pointer to bss role
 *
 * @return                  0 -- success, otherwise fail
 */
int woal_cfg80211_bss_role_cfg(moal_private *priv, t_u16 action, t_u8 *bss_role)
{
	int ret = 0;

	ENTER();

	if (action == MLAN_ACT_SET) {
		/* Reset interface */
		woal_reset_intf(priv, MOAL_IOCTL_WAIT, MFALSE);
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_role_cfg(priv, action, MOAL_IOCTL_WAIT, bss_role)) {
		ret = -EFAULT;
		goto done;
	}

	if (action == MLAN_ACT_SET) {
		/* set back the mac address */
		woal_request_set_mac_address(priv, MOAL_IOCTL_WAIT);
		/* clear the mgmt ies */
		woal_clear_all_mgmt_ies(priv, MOAL_IOCTL_WAIT);
		/* Initialize private structures */
		woal_init_priv(priv, MOAL_IOCTL_WAIT);

		/* Enable interfaces */
		netif_device_attach(priv->netdev);
		woal_start_queue(priv->netdev);
	}

done:
	LEAVE();
	return ret;
}
#endif

#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
/**
 *  @brief This function display P2P public action frame type
 *
 *  @param  buf      A buffer to a management frame
 *  @param  len      buffer len
 *  @param chan      the channel
 *  @param flag      Tx/Rx flag. Tx:flag = 1;Rx:flag = 0;
 *
 *  @return          N/A
 */
void woal_cfg80211_display_p2p_actframe(const t_u8 *buf, int len,
					struct ieee80211_channel *chan,
					const t_u8 flag)
{
	const t_u8 p2p_oui[] = {0x50, 0x6f, 0x9a, 0x09};
	t_u8 subtype;

	ENTER();

	if (!buf || len < (P2P_ACT_FRAME_OUI_SUBTYPE_OFFSET + 1)) {
		LEAVE();
		return;
	}

	if (((struct ieee80211_mgmt *)buf)->u.action.category ==
		    P2P_ACT_FRAME_CATEGORY &&
	    !memcmp(buf + P2P_ACT_FRAME_OUI_OFFSET, p2p_oui, sizeof(p2p_oui))) {
		subtype = *(buf + P2P_ACT_FRAME_OUI_SUBTYPE_OFFSET);
		switch (subtype) {
		case P2P_GO_NEG_REQ:
			PRINTM(MMSG,
			       "wlan: %s P2P Group Owner Negotiation Req Frame, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_GO_NEG_RSP:
			PRINTM(MMSG,
			       "wlan: %s P2P Group Owner Negotiation Rsp Frame, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_GO_NEG_CONF:
			PRINTM(MMSG,
			       "wlan: %s P2P Group Owner Negotiation Confirm Frame, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_INVITE_REQ:
			PRINTM(MMSG,
			       "wlan: %s P2P Invitation Request, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_INVITE_RSP:
			PRINTM(MMSG,
			       "wlan: %s P2P Invitation Response, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_DEVDIS_REQ:
			PRINTM(MMSG,
			       "wlan: %s P2P Device Discoverability Request, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_DEVDIS_RSP:
			PRINTM(MIOCTL,
			       "wlan: %s P2P Device Discoverability Response, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_PROVDIS_REQ:
			PRINTM(MMSG,
			       "wlan: %s P2P Provision Discovery Request, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		case P2P_PROVDIS_RSP:
			PRINTM(MMSG,
			       "wlan: %s P2P Provision Discovery Response, channel=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0);
			break;
		default:
			PRINTM(MMSG,
			       "wlan: %s Unknown P2P Action Frame, channel=%d, subtype=%d\n",
			       (flag) ? "TX" : "RX",
			       (chan) ? chan->hw_value : 0, subtype);
			break;
		}
	}

	LEAVE();
}

/**
 * @brief initialize p2p client for wpa_supplicant
 *
 * @param priv			A pointer to moal private structure
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_init_p2p_client(moal_private *priv)
{
	int ret = MLAN_STATUS_SUCCESS;
	t_u16 wifi_direct_mode = WIFI_DIRECT_MODE_DISABLE;
	t_u8 bss_role;

	ENTER();

	/* bss type check */
	if (priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
		PRINTM(MERROR, "Unexpected bss type when init p2p client\n");
		ret = -EFAULT;
		goto done;
	}

	/* get the bss role */
	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_GET, &bss_role)) {
		ret = -EFAULT;
		goto done;
	}

	if (bss_role != MLAN_BSS_ROLE_STA) {
		bss_role = MLAN_BSS_ROLE_STA;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_SET, &bss_role)) {
			ret = -EFAULT;
			goto done;
		}
	}

	wifi_direct_mode = WIFI_DIRECT_MODE_DISABLE;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}

	/* first, init wifi direct to listen mode */
	wifi_direct_mode = WIFI_DIRECT_MODE_LISTEN;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}

	/* second, init wifi direct client  */
	wifi_direct_mode = WIFI_DIRECT_MODE_CLIENT;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief initialize p2p GO for wpa_supplicant
 *
 * @param priv			A pointer to moal private structure
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_init_p2p_go(moal_private *priv)
{
	int ret = MLAN_STATUS_SUCCESS;
	t_u16 wifi_direct_mode;
	t_u8 bss_role;
	mlan_ds_wifi_direct_config p2p_config;
	mlan_ds_ps_mgmt ps_mgmt;

	ENTER();

	/* bss type check */
	if (priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
		PRINTM(MERROR, "Unexpected bss type when init p2p GO\n");
		ret = -EFAULT;
		goto done;
	}

	wifi_direct_mode = WIFI_DIRECT_MODE_DISABLE;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}

	/* first, init wifi direct to listen mode */
	wifi_direct_mode = WIFI_DIRECT_MODE_LISTEN;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}

	/* second, init wifi direct to GO mode  */
	wifi_direct_mode = WIFI_DIRECT_MODE_GO;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}

	/* get the bss role, and set it to uAP */
	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_GET, &bss_role)) {
		ret = -EFAULT;
		goto done;
	}

	if (bss_role != MLAN_BSS_ROLE_UAP) {
		bss_role = MLAN_BSS_ROLE_UAP;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_SET, &bss_role)) {
			ret = -EFAULT;
			goto done;
		}
	}
/* NoA:-- Interval = 100TUs and Duration= 50TUs, count=255*/
#define DEF_NOA_COUNT 255
	if (priv->phandle->noa_duration && priv->phandle->card_info->go_noa) {
		memset(&p2p_config, 0, sizeof(p2p_config));
		p2p_config.noa_enable = MTRUE;
		p2p_config.index = 0;
		p2p_config.noa_count = DEF_NOA_COUNT;
		p2p_config.noa_duration = priv->phandle->noa_duration;
		p2p_config.noa_interval = priv->phandle->noa_interval;
		p2p_config.flags = WIFI_DIRECT_NOA;
		woal_p2p_config(priv, MLAN_ACT_SET, &p2p_config);
		memset(&ps_mgmt, 0, sizeof(ps_mgmt));
		ps_mgmt.flags = PS_FLAG_PS_MODE;
		ps_mgmt.ps_mode = PS_MODE_INACTIVITY;
		woal_set_get_uap_power_mode(priv, MLAN_ACT_SET, &ps_mgmt);
		PRINTM(MMSG, "Enable NOA: duration=%d, interval=%d\n",
		       priv->phandle->noa_duration,
		       priv->phandle->noa_interval);
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief reset bss role and wifi direct mode for wpa_supplicant
 *
 * @param priv			A pointer to moal private structure
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_deinit_p2p(moal_private *priv)
{
	int ret = MLAN_STATUS_SUCCESS;
	t_u16 wifi_direct_mode;
	t_u8 bss_role;
	t_u8 channel_status;
	moal_private *remain_priv = NULL;
	mlan_ds_ps_mgmt ps_mgmt;

	ENTER();

	/* bss type check */
	if (priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
		PRINTM(MERROR, "Unexpected bss type when deinit p2p\n");
		ret = -EFAULT;
		goto done;
	}
	/* unregister mgmt frame from FW */
	if (priv->mgmt_subtype_mask) {
		priv->mgmt_subtype_mask = 0;
		if (woal_reg_rx_mgmt_ind(priv, MLAN_ACT_SET,
					 &priv->mgmt_subtype_mask,
					 MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR,
			       "deinit_p2p: fail to unregister mgmt frame\n");
			ret = -EFAULT;
			goto done;
		}
	}

	/* cancel previous remain on channel */
	if (priv->phandle->remain_on_channel) {
		remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (!remain_priv) {
			PRINTM(MERROR,
			       "deinit_p2p: wrong remain_bss_index=%d\n",
			       priv->phandle->remain_bss_index);
			ret = -EFAULT;
			goto done;
		}
		if (woal_cfg80211_remain_on_channel_cfg(
			    remain_priv, MOAL_IOCTL_WAIT, MTRUE,
			    &channel_status, NULL, 0, 0)) {
			PRINTM(MERROR,
			       "deinit_p2p: Fail to cancel remain on channel\n");
			ret = -EFAULT;
			goto done;
		}
		if (priv->phandle->cookie) {
			cfg80211_remain_on_channel_expired(
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
				remain_priv->netdev,
#else
				remain_priv->wdev,
#endif
				priv->phandle->cookie, &priv->phandle->chan,
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
				priv->phandle->channel_type,
#endif
				GFP_ATOMIC);
			priv->phandle->cookie = 0;
		}
		priv->phandle->remain_on_channel = MFALSE;
	}

	/* get the bss role */
	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_GET, &bss_role)) {
		ret = -EFAULT;
		goto done;
	}

	/* reset bss role */
	if (bss_role != MLAN_BSS_ROLE_STA) {
		memset(&ps_mgmt, 0, sizeof(ps_mgmt));
		ps_mgmt.flags = PS_FLAG_PS_MODE;
		ps_mgmt.ps_mode = PS_MODE_DISABLE;
		woal_set_get_uap_power_mode(priv, MLAN_ACT_SET, &ps_mgmt);
		bss_role = MLAN_BSS_ROLE_STA;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_SET, &bss_role)) {
			ret = -EFAULT;
			goto done;
		}
	}

	wifi_direct_mode = WIFI_DIRECT_MODE_DISABLE;
	if (MLAN_STATUS_SUCCESS !=
	    woal_wifi_direct_mode_cfg(priv, MLAN_ACT_SET, &wifi_direct_mode)) {
		ret = -EFAULT;
		goto done;
	}
done:
	LEAVE();
	return ret;
}
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */

/**
 * @brief Request the driver to change the interface type
 *
 * @param wiphy         A pointer to wiphy structure
 * @param dev           A pointer to net_device structure
 * @param type          Virtual interface types
 * @param flags         Flags
 * @param params        A pointer to vif_params structure
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_change_virtual_intf(struct wiphy *wiphy,
				      struct net_device *dev,
				      enum nl80211_iftype type,
#if KERNEL_VERSION(4, 12, 0) > CFG80211_VERSION_CODE
				      u32 *flags,
#endif
				      struct vif_params *params)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
	t_u8 bss_role;
#endif
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (priv->wdev->iftype == type) {
		PRINTM(MINFO, "Already set to required type\n");
		goto done;
	}
#ifdef UAP_SUPPORT
	if ((priv->bss_type == MLAN_BSS_TYPE_UAP) && (priv->bss_index > 0)) {
		priv->wdev->iftype = type;
		PRINTM(MMSG, "%s: Skip change virtual intf on uap: type=%d\n",
		       dev->name, type);
		goto done;
	}
#endif

	PRINTM(MIOCTL, "%s: change virturl intf=%d\n", dev->name, type);
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	/** cancel previous remain on channel to avoid firmware hang */
	if (priv->phandle->remain_on_channel) {
		t_u8 channel_status;
		moal_private *remain_priv = NULL;

		remain_priv =
			priv->phandle->priv[priv->phandle->remain_bss_index];
		if (!remain_priv) {
			PRINTM(MERROR,
			       "change_virtual_intf:wrong remain_bss_index=%d\n",
			       priv->phandle->remain_bss_index);
			ret = -EFAULT;
			goto done;
		}
		if (woal_cfg80211_remain_on_channel_cfg(
			    remain_priv, MOAL_IOCTL_WAIT, MTRUE,
			    &channel_status, NULL, 0, 0)) {
			PRINTM(MERROR,
			       "change_virtual_intf: Fail to cancel remain on channel\n");
			ret = -EFAULT;
			goto done;
		}
		if (priv->phandle->cookie) {
			cfg80211_remain_on_channel_expired(
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
				remain_priv->netdev,
#else
				remain_priv->wdev,
#endif
				priv->phandle->cookie, &priv->phandle->chan,
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
				priv->phandle->channel_type,
#endif
				GFP_ATOMIC);
			priv->phandle->cookie = 0;
		}
		priv->phandle->remain_on_channel = MFALSE;
	}
#endif
#endif

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MODE;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	switch (type) {
	case NL80211_IFTYPE_STATION:
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
		if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT &&
		    (priv->wdev->iftype == NL80211_IFTYPE_AP ||
		     priv->wdev->iftype == NL80211_IFTYPE_P2P_GO ||
		     priv->wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)) {
			if (priv->phandle->is_go_timer_set) {
				woal_cancel_timer(&priv->phandle->go_timer);
				priv->phandle->is_go_timer_set = MFALSE;
			}
			/* if we support wifi direct && priv->bss_type ==
			 * wifi_direct, and currently the interface type is AP
			 * or GO or client, that means wpa_supplicant deinit()
			 * wifi direct interface, so we should deinit bss_role
			 * and wifi direct mode, for other bss_type, we should
			 * not update bss_role and wifi direct mode
			 */

			if (MLAN_STATUS_SUCCESS !=
			    woal_cfg80211_deinit_p2p(priv)) {
				ret = -EFAULT;
				goto done;
			}
		}
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
		if (priv->bss_type == MLAN_BSS_TYPE_UAP) {
			woal_cfg80211_del_beacon(wiphy, dev);
			bss_role = MLAN_BSS_ROLE_STA;
			woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_SET,
						   &bss_role);
			PRINTM(MIOCTL, "set bss role for STA\n");
		}
#endif
		bss->param.bss_mode = MLAN_BSS_MODE_INFRA;
		priv->wdev->iftype = NL80211_IFTYPE_STATION;
		PRINTM(MINFO, "Setting interface type to managed\n");
		break;
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	case NL80211_IFTYPE_P2P_CLIENT:
		if (priv->phandle->is_go_timer_set) {
			woal_cancel_timer(&priv->phandle->go_timer);
			priv->phandle->is_go_timer_set = MFALSE;
		}

		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_init_p2p_client(priv)) {
			ret = -EFAULT;
			goto done;
		}

		bss->param.bss_mode = MLAN_BSS_MODE_INFRA;
		priv->wdev->iftype = NL80211_IFTYPE_P2P_CLIENT;
		PRINTM(MINFO, "Setting interface type to P2P client\n");

		break;
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */
	case NL80211_IFTYPE_AP:
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
	/* Fall Through */
	case NL80211_IFTYPE_P2P_GO:
		if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_cfg80211_init_p2p_go(priv)) {
				ret = -EFAULT;
				goto done;
			}
			priv->phandle->is_go_timer_set = MTRUE;
			woal_mod_timer(&priv->phandle->go_timer,
				       MOAL_TIMER_10S);
		}
		if (type == NL80211_IFTYPE_P2P_GO)
			priv->wdev->iftype = NL80211_IFTYPE_P2P_GO;
#endif
#endif
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
		if (priv->bss_type == MLAN_BSS_TYPE_STA) {
#ifdef STA_CFG80211
			/** cancel pending scan */
			woal_cancel_scan(priv, MOAL_IOCTL_WAIT);
#endif
			if (priv->probereq_index !=
			    MLAN_CUSTOM_IE_AUTO_IDX_MASK)
				woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL,
							    0, NULL, 0, NULL, 0,
							    MGMT_MASK_PROBE_REQ,
							    MOAL_IOCTL_WAIT);
			bss_role = MLAN_BSS_ROLE_UAP;
			woal_cfg80211_bss_role_cfg(priv, MLAN_ACT_SET,
						   &bss_role);
			PRINTM(MIOCTL, "set bss role for AP\n");
		}
#endif
		if (type == NL80211_IFTYPE_AP)
			priv->wdev->iftype = NL80211_IFTYPE_AP;
		PRINTM(MINFO, "Setting interface type to P2P GO\n");

		/* there is no need for P2P GO to set bss_mode */
		goto done;

		break;

	case NL80211_IFTYPE_UNSPECIFIED:
		bss->param.bss_mode = MLAN_BSS_MODE_AUTO;
		priv->wdev->iftype = NL80211_IFTYPE_STATION;
		PRINTM(MINFO, "Setting interface type to auto\n");
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret)
		goto done;

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
 * @brief Request the driver to change the value of fragment
 * threshold or rts threshold or retry limit
 *
 * @param wiphy         A pointer to wiphy structure
 * @param changed       Change flags
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	moal_private *priv = NULL;
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
	pmlan_uap_bss_param sys_cfg = NULL;
#endif
#endif
	int frag_thr = wiphy->frag_threshold;
	int rts_thr = wiphy->rts_threshold;
	int retry = wiphy->retry_long;

	ENTER();

	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		LEAVE();
		return -EFAULT;
	}
	if (rts_thr == MLAN_FRAG_RTS_DISABLED)
		rts_thr = MLAN_RTS_MAX_VALUE;
	if (frag_thr == MLAN_FRAG_RTS_DISABLED)
		frag_thr = MLAN_FRAG_MAX_VALUE;

#ifdef UAP_CFG80211
#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		sys_cfg = kzalloc(sizeof(mlan_uap_bss_param), GFP_ATOMIC);
		if (!sys_cfg) {
			PRINTM(MERROR,
			       "Fail to alloc memory for mlan_uap_bss_param\n");
			LEAVE();
			return -EFAULT;
		}
		/* Initialize the invalid values so that the correct
		 * values below are downloaded to firmware
		 */
		woal_set_sys_config_invalid_data(sys_cfg);
		sys_cfg->frag_threshold = frag_thr;
		sys_cfg->rts_threshold = rts_thr;
		sys_cfg->retry_limit = retry;

		if ((changed & WIPHY_PARAM_RTS_THRESHOLD) ||
		    (changed & WIPHY_PARAM_FRAG_THRESHOLD) ||
		    (changed &
		     (WIPHY_PARAM_RETRY_LONG | WIPHY_PARAM_RETRY_SHORT))) {
			if (woal_set_get_sys_config(priv, MLAN_ACT_SET,
						    MOAL_IOCTL_WAIT, sys_cfg)) {
				kfree(sys_cfg);
				goto fail;
			}
		}
		kfree(sys_cfg);
	}
#endif
#endif

#ifdef STA_CFG80211
#ifdef STA_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
			if (woal_set_get_rts(priv, MLAN_ACT_SET,
					     MOAL_IOCTL_WAIT, &rts_thr))
				goto fail;
		}
		if (changed & WIPHY_PARAM_FRAG_THRESHOLD) {
			if (woal_set_get_frag(priv, MLAN_ACT_SET,
					      MOAL_IOCTL_WAIT, &frag_thr))
				goto fail;
		}
		if (changed &
		    (WIPHY_PARAM_RETRY_LONG | WIPHY_PARAM_RETRY_SHORT))
			if (woal_set_get_retry(priv, MLAN_ACT_SET,
					       MOAL_IOCTL_WAIT, &retry))
				goto fail;
	}
#endif
#endif
	LEAVE();
	return 0;

fail:
	PRINTM(MERROR, "Failed to change wiphy params %x\n", changed);
	LEAVE();
	return -EFAULT;
}

#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
/**
 * @brief Request the driver to add a key
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 * @param pairwise      Flag to indicate pairwise or group (for kernel > 2.6.36)
 * @param mac_addr      MAC address (NULL for group key)
 * @param params        A pointer to key_params structure
 *
 * @return              0 -- success, otherwise fail
 */
#else
/**
 * @brief Request the driver to add a key
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 * @param mac_addr      MAC address (NULL for group key)
 * @param params        A pointer to key_params structure
 *
 * @return              0 -- success, otherwise fail
 */
#endif
int woal_cfg80211_add_key(struct wiphy *wiphy, struct net_device *netdev,
			  t_u8 key_index,
#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
			  bool pairwise,
#endif
			  const t_u8 *mac_addr, struct key_params *params)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(netdev);

	ENTER();
	if (priv->ft_pre_connect) {
		PRINTM(MINFO, "Skip set keys during ft connecting\n");
		return -EFAULT;
	}
	if (woal_cfg80211_set_key(priv, 0, params->cipher, params->key,
				  params->key_len, params->seq, params->seq_len,
				  key_index, mac_addr, 0, MOAL_IOCTL_WAIT)) {
		PRINTM(MERROR, "Error adding the crypto keys\n");
		LEAVE();
		return -EFAULT;
	}

	PRINTM(MINFO, "Crypto keys added\n");

	LEAVE();
	return 0;
}

#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
/**
 * @brief Request the driver to delete a key
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 * @param pairwise      Flag to indicate pairwise or group (for kernel > 2.6.36)
 * @param mac_addr      MAC address (NULL for group key)
 *
 * @return              0 -- success, otherwise fail
 */
#else
/**
 * @brief Request the driver to delete a key
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 * @param mac_addr      MAC address (NULL for group key)
 *
 * @return              0 -- success, otherwise fail
 */
#endif
int woal_cfg80211_del_key(struct wiphy *wiphy, struct net_device *netdev,
			  t_u8 key_index,
#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
			  bool pairwise,
#endif
			  const t_u8 *mac_addr)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(netdev);

	ENTER();
	if (priv->phandle->driver_status) {
		PRINTM(MERROR, "Block %s in abnormal driver state\n", __func__);
		LEAVE();
		return -EFAULT;
	}
	/* del_key will be trigger from cfg80211_rx_mlme_mgmt funtion
	 * where we receive deauth/disassoicate packet in rx_work
	 * use MOAL_NO_WAIT to avoid dead lock
	 */
	if (MLAN_STATUS_FAILURE ==
	    woal_cfg80211_set_key(priv, 0, 0, NULL, 0, NULL, 0, key_index,
				  mac_addr, 1, MOAL_NO_WAIT)) {
		PRINTM(MERROR, "Error deleting the crypto keys\n");
		LEAVE();
		return -EFAULT;
	}

	PRINTM(MINFO, "Crypto keys deleted\n");
	LEAVE();
	return 0;
}

#if KERNEL_VERSION(2, 6, 37) < CFG80211_VERSION_CODE
/**
 * @brief Request to enable WEP key to driver
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 * @param ucast         Unicast flag (for kernel > 2.6.37)
 * @param mcast         Multicast flag (for kernel > 2.6.37)
 *
 * @return              0 -- success, otherwise fail
 */
#else
/**
 * @brief Request to enable WEP key to driver
 *
 * @param wiphy         A pointer to wiphy structure
 * @param netdev        A pointer to net_device structure
 * @param key_index     Key index
 *
 * @return              0 -- success, otherwise fail
 */
#endif
int woal_cfg80211_set_default_key(struct wiphy *wiphy,
				  struct net_device *netdev, t_u8 key_index
#if KERNEL_VERSION(2, 6, 37) < CFG80211_VERSION_CODE
				  ,
				  bool ucast, bool mcast
#endif
)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(netdev);
	mlan_bss_info bss_info;

	ENTER();
	memset(&bss_info, 0, sizeof(mlan_bss_info));
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
		if (!bss_info.wep_status) {
			LEAVE();
			return ret;
		}
	}
	if (MLAN_STATUS_SUCCESS !=
	    woal_cfg80211_set_wep_keys(priv, NULL, 0, key_index,
				       MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
	}
	LEAVE();
	return ret;
}

#if KERNEL_VERSION(2, 6, 30) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_default_mgmt_key(struct wiphy *wiphy,
				       struct net_device *netdev,
				       t_u8 key_index)
{
	PRINTM(MINFO, "set default mgmt key, key index=%d\n", key_index);

	return 0;
}
#endif

#if KERNEL_VERSION(3, 1, 0) <= CFG80211_VERSION_CODE
/**
 *  @brief  Set GTK rekey data to driver
 *
 *  @param priv         A pointer to moal_private structure
 *  @param gtk_rekey     A pointer to mlan_ds_misc_gtk_rekey_data structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *
 *  @return             0 --success, otherwise fail
 */
mlan_status woal_set_rekey_data(moal_private *priv,
				mlan_ds_misc_gtk_rekey_data *gtk_rekey,
				t_u8 action, t_u8 wait_option)
{
	mlan_ioctl_req *req;
	mlan_ds_misc_cfg *misc_cfg;
	int ret = 0;
	mlan_status status;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));

	if (req == NULL) {
		ret = -ENOMEM;
	} else {
		misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
		misc_cfg->sub_command = MLAN_OID_MISC_GTK_REKEY_OFFLOAD;
		req->req_id = MLAN_IOCTL_MISC_CFG;

		req->action = action;
		if (action == MLAN_ACT_SET)
			moal_memcpy_ext(priv->phandle,
					&misc_cfg->param.gtk_rekey, gtk_rekey,
					sizeof(mlan_ds_misc_gtk_rekey_data),
					sizeof(mlan_ds_misc_gtk_rekey_data));

		status = woal_request_ioctl(priv, req, wait_option);
		if (status != MLAN_STATUS_SUCCESS)
			ret = -EFAULT;
		if (status != MLAN_STATUS_PENDING)
			kfree(req);
	}

	LEAVE();
	return ret;
}

/**
 * @brief Give the data necessary for GTK rekeying to the driver
 *
 * @param wiphy         A pointer to wiphy structure
 * @param dev           A pointer to net_device structure
 * @param data        A pointer to cfg80211_gtk_rekey_data structure
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_set_rekey_data(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_gtk_rekey_data *data)
{
	int ret = 0;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_ds_misc_gtk_rekey_data rekey;
	mlan_fw_info fw_info;

	ENTER();

	if (priv->phandle->params.gtk_rekey_offload ==
	    GTK_REKEY_OFFLOAD_DISABLE) {
		PRINTM(MMSG, "%s return: gtk_rekey_offload is DISABLE\n",
		       __func__);
		LEAVE();
		return ret;
	}

	memset(&fw_info, 0, sizeof(mlan_fw_info));
	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if (!fw_info.fw_supplicant_support) {
		LEAVE();
		return -1;
	}

	moal_memcpy_ext(priv->phandle, rekey.kek, data->kek, MLAN_KEK_LEN,
			MLAN_KEK_LEN);
	moal_memcpy_ext(priv->phandle, rekey.kck, data->kck, MLAN_KCK_LEN,
			MLAN_KCK_LEN);
	moal_memcpy_ext(priv->phandle, rekey.replay_ctr, data->replay_ctr,
			MLAN_REPLAY_CTR_LEN, MLAN_REPLAY_CTR_LEN);

	moal_memcpy_ext(priv->phandle, &priv->gtk_rekey_data, &rekey,
			sizeof(mlan_ds_misc_gtk_rekey_data),
			sizeof(mlan_ds_misc_gtk_rekey_data));
	if (priv->phandle->params.gtk_rekey_offload ==
	    GTK_REKEY_OFFLOAD_SUSPEND) {
		priv->gtk_data_ready = MTRUE;
		LEAVE();
		return ret;
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_rekey_data(priv, &rekey, MLAN_ACT_SET, MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
	}

	LEAVE();
	return ret;
}
#endif

#ifdef STA_SUPPORT
/* Opportunistic Key Caching APIs functions support
 *
 * this function get pmksa entry list in private structure
 * @param priv         A pointer to moal_private structure
 * @param bssid        A pointer to bssid
 * @return             pointer to target entry or NULL
 */
struct pmksa_entry *woal_get_pmksa_entry(moal_private *priv, const u8 *bssid)
{
	struct pmksa_entry *entry = NULL;
	unsigned long flags;

	if (!priv || priv->bss_type != MLAN_BSS_TYPE_STA) {
		PRINTM(MERROR, "Invalid interface structure\n");
		return NULL;
	}

	spin_lock_irqsave(&priv->pmksa_list_lock, flags);
	list_for_each_entry (entry, &priv->pmksa_cache_list, link) {
		if (!memcmp(entry->bssid, bssid, ETH_ALEN)) {
			spin_unlock_irqrestore(&priv->pmksa_list_lock, flags);
			return entry;
		}
	}
	spin_unlock_irqrestore(&priv->pmksa_list_lock, flags);

	return NULL;
}

/**
 * This function flush pmksa entry list in private structure
 * @param priv         A pointer to moal_private structure
 * @return             success of failure
 */
int woal_flush_pmksa_list(moal_private *priv)
{
	struct pmksa_entry *entry, *tmp;
	unsigned long flags;

	if (!priv || priv->bss_type != MLAN_BSS_TYPE_STA) {
		PRINTM(MERROR, "Invalid interface structure\n");
		return -1;
	}

	spin_lock_irqsave(&priv->pmksa_list_lock, flags);
	list_for_each_entry_safe (entry, tmp, &priv->pmksa_cache_list, link) {
		list_del(&entry->link);
		kfree(entry);
	}
	INIT_LIST_HEAD(&priv->pmksa_cache_list);
	spin_unlock_irqrestore(&priv->pmksa_list_lock, flags);

	return 0;
}

/**
 *  This function add new pmksa entry to list
 *  @param wiphy        A pointer to struct wiphy
 *  @param dev          A pointer to net_device structure
 *  @param pmksa        A pointer to cfg80211_pmksa structure
 *  @return             success of failure
 */
int woal_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_pmksa *pmksa)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct pmksa_entry *entry = NULL;
	unsigned long flags;
	int ret = 0;

	ENTER();

	if (!priv || priv->bss_type != MLAN_BSS_TYPE_STA) {
		PRINTM(MERROR, "Invalid interface structure\n");
		ret = -1;
		goto done;
	}

	PRINTM(MIOCTL, "Set pmksa entry: bssid=" MACSTR "\n",
	       MAC2STR(pmksa->bssid));
	entry = woal_get_pmksa_entry(priv, pmksa->bssid);
	if (!entry) {
		entry = kzalloc(sizeof(struct pmksa_entry), GFP_ATOMIC);
		if (!entry) {
			PRINTM(MERROR, "Fail to allocate pmksa entry\n");
			goto done;
		}
		INIT_LIST_HEAD(&entry->link);
		moal_memcpy_ext(priv->phandle, entry->bssid, pmksa->bssid,
				ETH_ALEN, ETH_ALEN);
		moal_memcpy_ext(priv->phandle, entry->pmkid, pmksa->pmkid,
				PMKID_LEN, PMKID_LEN);
		spin_lock_irqsave(&priv->pmksa_list_lock, flags);
		list_add_tail(&entry->link, &priv->pmksa_cache_list);
		spin_unlock_irqrestore(&priv->pmksa_list_lock, flags);
	} else {
		/** pmkid is differnt from previous value? */
		memset(entry->pmkid, 0, PMKID_LEN);
		moal_memcpy_ext(priv->phandle, entry->pmkid, pmksa->pmkid,
				PMKID_LEN, PMKID_LEN);
	}

	/** Check if current roaming is going and received target pmkid */
	if (priv->wait_target_ap_pmkid) {
		struct cfg80211_connect_params *param = &priv->sme_current;

		if (param && !memcmp(pmksa->bssid, param->bssid, ETH_ALEN)) {
			PRINTM(MIOCTL,
			       "Current roaming target bssid=" MACSTR "\n",
			       MAC2STR(param->bssid));
			priv->target_ap_pmksa = entry;
			priv->wait_target_ap_pmkid = MFALSE;
			wake_up_interruptible(&priv->okc_wait_q);
		}
	}

done:
	LEAVE();
	return ret;
}

/**
 *  This function delete pmksa entry
 *  @param wiphy        A pointer to struct wiphy
 *  @param dev          A pointer to net_device structure
 *  @param pmksa        A pointer to cfg80211_pmksa structure
 *  @return             success of failure
 */
int woal_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_pmksa *pmksa)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	struct pmksa_entry *entry, *tmp;
	unsigned long flags;

	ENTER();

	if (!priv || priv->bss_type != MLAN_BSS_TYPE_STA) {
		PRINTM(MERROR, "Invalid interface structure\n");
		LEAVE();
		return -1;
	}

	PRINTM(MIOCTL, "Delete pmksa: bssid=" MACSTR "\n",
	       MAC2STR(pmksa->bssid));
	spin_lock_irqsave(&priv->pmksa_list_lock, flags);
	list_for_each_entry_safe (entry, tmp, &priv->pmksa_cache_list, link) {
		if (!memcmp(entry->bssid, pmksa->bssid, ETH_ALEN)) {
			list_del(&entry->link);
			kfree(entry);
		}
	}
	spin_unlock_irqrestore(&priv->pmksa_list_lock, flags);

	LEAVE();
	return 0;
}

/**
 *  This function flush pmksa entry list
 *  @param wiphy        A pointer to struct wiphy
 *  @param dev          A pointer to net_device structure
 *  @return             success of failure
 */
int woal_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	if (!priv || priv->bss_type != MLAN_BSS_TYPE_STA)
		return -1;

	PRINTM(MIOCTL, "Flush pmksa list.\n");
	return woal_flush_pmksa_list(priv);
}
#endif

#if KERNEL_VERSION(3, 5, 0) > CFG80211_VERSION_CODE
#if KERNEL_VERSION(2, 6, 34) < CFG80211_VERSION_CODE
/**
 * @brief Request the driver to change the channel
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param chan            A pointer to ieee80211_channel structure
 * @param channel_type    Channel type of nl80211_channel_type
 *
 * @return                0 -- success, otherwise fail
 */
#else
/**
 * @brief Request the driver to change the channel
 *
 * @param wiphy           A pointer to wiphy structure
 * @param chan            A pointer to ieee80211_channel structure
 * @param channel_type    Channel type of nl80211_channel_type
 *
 * @return                0 -- success, otherwise fail
 */
#endif
int woal_cfg80211_set_channel(struct wiphy *wiphy,
#if KERNEL_VERSION(2, 6, 34) < CFG80211_VERSION_CODE
			      struct net_device *dev,
#endif
			      struct ieee80211_channel *chan,
			      enum nl80211_channel_type channel_type)
{
	int ret = 0;
	moal_private *priv = NULL;

	ENTER();

#if KERNEL_VERSION(3, 5, 0) > CFG80211_VERSION_CODE
#if KERNEL_VERSION(2, 6, 34) < CFG80211_VERSION_CODE
	if (dev)
		priv = woal_get_netdev_priv(dev);
#endif
#endif
	if (!priv) {
		moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);

		if (handle)
			priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	}
	if (priv) {
#ifdef STA_CFG80211
#ifdef STA_SUPPORT
		if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
			if (priv->media_connected == MTRUE) {
				PRINTM(MERROR,
				       "This configuration is valid only when station is not connected\n");
				LEAVE();
				return -EINVAL;
			}
			ret = woal_set_rf_channel(priv, chan, channel_type,
						  MOAL_IOCTL_WAIT);
		}
#endif
#endif
		priv->channel =
			ieee80211_frequency_to_channel(chan->center_freq);
	}
	/* set monitor channel support */

	LEAVE();
	return ret;
}
#endif

#if KERNEL_VERSION(3, 12, 0) <= CFG80211_VERSION_CODE
static bool woal_is_pattern_supported(struct cfg80211_pkt_pattern *pat,
				      t_u8 *byte_seq, t_u8 max_byte_seq)
{
	int j, k, valid_byte_cnt = 0;
	bool dont_care_byte = false;

	for (j = 0; j < DIV_ROUND_UP(pat->pattern_len, 8); j++) {
		for (k = 0; k < 8; k++) {
			if (pat->mask[j] & 1 << k) {
				moal_memcpy_ext(NULL, byte_seq + valid_byte_cnt,
						&pat->pattern[j * 8 + k], 1,
						(t_u32)max_byte_seq -
							(t_u32)valid_byte_cnt);
				valid_byte_cnt++;
				if (dont_care_byte)
					return false;
			} else {
				if (valid_byte_cnt)
					dont_care_byte = true;
			}

			if (valid_byte_cnt > max_byte_seq)
				return false;
		}
	}

	byte_seq[max_byte_seq] = valid_byte_cnt;

	return true;
}

static int woal_get_coalesce_pkt_type(t_u8 *byte_seq)
{
	const t_u8 ipv4_mc_mac[] = {0x33, 0x33};
	const t_u8 ipv6_mc_mac[] = {0x01, 0x00, 0x5e};
	const t_u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff};

	if ((byte_seq[0] & 0x01) && (byte_seq[COALESCE_MAX_BYTESEQ] == 1))
		return PACKET_TYPE_UNICAST;
	else if (!memcmp(byte_seq, bc_mac, 4))
		return PACKET_TYPE_BROADCAST;
	else if ((!memcmp(byte_seq, ipv4_mc_mac, 2) &&
		  byte_seq[COALESCE_MAX_BYTESEQ] == 2) ||
		 (!memcmp(byte_seq, ipv6_mc_mac, 3) &&
		  byte_seq[COALESCE_MAX_BYTESEQ] == 3))
		return PACKET_TYPE_MULTICAST;

	return 0;
}

static int woal_fill_coalesce_rule_info(struct cfg80211_coalesce_rules *crule,
					struct coalesce_rule *mrule)
{
	t_u8 byte_seq[COALESCE_MAX_BYTESEQ + 1];
	struct filt_field_param *param;
	int i;

	mrule->max_coalescing_delay = crule->delay;

	param = mrule->params;

	for (i = 0; i < crule->n_patterns; i++) {
		memset(byte_seq, 0, sizeof(byte_seq));
		if (!woal_is_pattern_supported(&crule->patterns[i], byte_seq,
					       COALESCE_MAX_BYTESEQ)) {
			PRINTM(MERROR, "Pattern not supported\n");
			return -EOPNOTSUPP;
		}

		if (!crule->patterns[i].pkt_offset) {
			u8 pkt_type;

			pkt_type = woal_get_coalesce_pkt_type(byte_seq);
			if (pkt_type && mrule->pkt_type) {
				PRINTM(MERROR,
				       "Multiple packet types not allowed\n");
				return -EOPNOTSUPP;
			} else if (pkt_type) {
				mrule->pkt_type = pkt_type;
				continue;
			}
		}

		if (crule->condition == NL80211_COALESCE_CONDITION_MATCH)
			param->operation = RECV_FILTER_MATCH_TYPE_EQ;
		else
			param->operation = RECV_FILTER_MATCH_TYPE_NE;

		param->operand_len = byte_seq[COALESCE_MAX_BYTESEQ];
		moal_memcpy_ext(NULL, param->operand_byte_stream, byte_seq,
				param->operand_len, COALESCE_MAX_BYTESEQ);
		param->offset = crule->patterns[i].pkt_offset;
		param++;

		mrule->num_of_fields++;
	}

	if (!mrule->pkt_type) {
		PRINTM(MERROR, "Packet type can not be determined\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

/**
 *  @brief Set coalesce parameter
 *
 *  @param priv             A pointer to moal_private structure
 *  @param action           MLAN_ACT_SET or MLAN_ACT_GET
 *  @param coalesce_cfg     A pointer to coalesce structure
 *
 *  @return                 MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_set_coalesce(moal_private *priv, t_u16 action,
			      mlan_ds_coalesce_cfg *coalesce_cfg)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_ioctl_req *req = NULL;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_COALESCE_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = action;

	moal_memcpy_ext(priv->phandle, &misc_cfg->param.coalesce_cfg,
			coalesce_cfg, sizeof(mlan_ds_coalesce_cfg),
			sizeof(mlan_ds_coalesce_cfg));

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Request the driver to set the coalesce
 *
 * @param wiphy           A pointer to wiphy structure
 * @param coalesce        A pointer to cfg80211_coalesce structure
 *
 * @return                0 -- success, otherwise fail
 */
int woal_cfg80211_set_coalesce(struct wiphy *wiphy,
			       struct cfg80211_coalesce *coalesce)
{
	int ret = 0;
	int i;
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv = NULL;
	mlan_ds_coalesce_cfg coalesce_cfg;

	ENTER();

	if (!handle) {
		PRINTM(MFATAL, "Unable to get handle\n");
		ret = -EINVAL;
		goto done;
	}
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		ret = -EINVAL;
		goto done;
	}

	memset(&coalesce_cfg, 0, sizeof(coalesce_cfg));
	if (!coalesce) {
		PRINTM(MMSG, "Disable coalesce and reset all previous rules\n");
	} else {
		coalesce_cfg.num_of_rules = coalesce->n_rules;
		for (i = 0; i < coalesce->n_rules; i++) {
			ret = woal_fill_coalesce_rule_info(
				&coalesce->rules[i], &coalesce_cfg.rule[i]);
			if (ret) {
				PRINTM(MERROR,
				       "Recheck the patterns provided for rule %d\n",
				       i + 1);
				return ret;
			}
		}
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_coalesce(priv, MLAN_ACT_SET, &coalesce_cfg)) {
		PRINTM(MERROR, "wlan: Fail to set coalesce\n");
		ret = -EFAULT;
	}

done:
	LEAVE();
	return ret;
}
#endif

/**
 * @brief Request the driver to set the bitrate
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param peer            A pointer to peer address
 * @param mask            A pointer to cfg80211_bitrate_mask structure
 *
 * @return                0 -- success, otherwise fail
 */
int woal_cfg80211_set_bitrate_mask(struct wiphy *wiphy, struct net_device *dev,
				   const u8 *peer,
				   const struct cfg80211_bitrate_mask *mask)
{
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	mlan_bss_info bss_info;
	enum ieee80211_band band;
	mlan_ioctl_req *req = NULL;
	mlan_ds_rate *rate = NULL;
	mlan_rate_cfg_t *rate_cfg = NULL;

	ENTER();

	if (priv->media_connected == MFALSE) {
		PRINTM(MERROR, "Can not set data rate in disconnected state\n");
		ret = -EINVAL;
		goto done;
	}

	status = woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
	if (status)
		goto done;
	band = woal_band_cfg_to_ieee_band(bss_info.bss_band);

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	rate = (mlan_ds_rate *)req->pbuf;
	rate_cfg = &rate->param.rate_cfg;
	rate->sub_command = MLAN_OID_RATE_CFG;
	req->req_id = MLAN_IOCTL_RATE;
	req->action = MLAN_ACT_SET;
	rate_cfg->rate_type = MLAN_RATE_BITMAP;

	/* Fill HR/DSSS rates. */
	if (band == IEEE80211_BAND_2GHZ)
		rate_cfg->bitmap_rates[0] = mask->control[band].legacy & 0x000f;

	/* Fill OFDM rates */
	if (band == IEEE80211_BAND_2GHZ)
		rate_cfg->bitmap_rates[1] =
			(mask->control[band].legacy & 0x0ff0) >> 4;
	else
		rate_cfg->bitmap_rates[1] = mask->control[band].legacy;

#if KERNEL_VERSION(3, 4, 0) <= CFG80211_VERSION_CODE
		/* Fill MCS rates */
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
	rate_cfg->bitmap_rates[2] = mask->control[band].ht_mcs[0];
#else
	rate_cfg->bitmap_rates[2] = mask->control[band].mcs[0];
#endif
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
	rate_cfg->bitmap_rates[2] |= mask->control[band].ht_mcs[1] << 8;
#else
	rate_cfg->bitmap_rates[2] |= mask->control[band].mcs[1] << 8;
#endif
#endif

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

#if KERNEL_VERSION(2, 6, 38) <= CFG80211_VERSION_CODE
/**
 * @brief Request the driver to get antenna configuration
 *
 * @param wiphy           A pointer to wiphy structure
 * @param tx_ant          Bitmaps of allowed antennas to use for TX
 * @param rx_ant          Bitmaps of allowed antennas to use for RX
 *
 * @return                0 -- success, otherwise fail
 */
int woal_cfg80211_get_antenna(struct wiphy *wiphy, u32 *tx_ant, u32 *rx_ant)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv = NULL;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	if (!handle) {
		PRINTM(MFATAL, "Unable to get handle\n");
		ret = -EINVAL;
		goto done;
	}
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_ANT_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (handle->feature_control & FEATURE_CTRL_STREAM_2X2) {
		*tx_ant = radio->param.ant_cfg.tx_antenna;
		*rx_ant = radio->param.ant_cfg.rx_antenna;
	} else {
		*tx_ant = radio->param.ant_cfg_1x1.antenna;
		*rx_ant = radio->param.ant_cfg_1x1.antenna;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	/* Driver must return -EINVAL to cfg80211 */
	if (ret)
		ret = -EINVAL;
	LEAVE();
	return ret;
}

/**
 * @brief Request the driver to set antenna configuration
 *
 * @param wiphy           A pointer to wiphy structure
 * @param tx_ant          Bitmaps of allowed antennas to use for TX
 * @param rx_ant          Bitmaps of allowed antennas to use for RX
 *
 * @return                0 -- success, otherwise fail
 */
int woal_cfg80211_set_antenna(struct wiphy *wiphy, u32 tx_ant, u32 rx_ant)
{
	moal_handle *handle = (moal_handle *)woal_get_wiphy_priv(wiphy);
	moal_private *priv = NULL;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	if (!handle) {
		PRINTM(MFATAL, "Unable to get handle\n");
		ret = -EINVAL;
		goto done;
	}
	priv = woal_get_priv(handle, MLAN_BSS_ROLE_ANY);
	if (!priv) {
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_ANT_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;
	req->action = MLAN_ACT_SET;
	if (handle->feature_control & FEATURE_CTRL_STREAM_2X2) {
		radio->param.ant_cfg.tx_antenna = tx_ant;
		radio->param.ant_cfg.rx_antenna = rx_ant;
	} else {
		radio->param.ant_cfg_1x1.antenna = tx_ant;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	/* Driver must return -EINVAL to cfg80211 */
	if (ret)
		ret = -EINVAL;
	LEAVE();
	return ret;
}
#endif
/**
 * @brief register/unregister mgmt frame forwarding
 *
 * @param priv             A pointer to moal_private structure
 * @param frame_type      Bit mask for mgmt frame type
 * @param reg             Register or unregister
 *
 * @return                0 -- success, otherwise fail
 */
void woal_mgmt_frame_register(moal_private *priv, u16 frame_type, bool reg)
{
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u32 mgmt_subtype_mask = 0x0;
	t_u32 last_mgmt_subtype_mask = priv->mgmt_subtype_mask;

	ENTER();

#ifdef SDIO_SUSPEND_RESUME
	if (priv->phandle->shutdown_hs_in_process) {
		LEAVE();
		return;
	}
#endif

	if (reg == MTRUE) {
		/* set mgmt_subtype_mask based on origin value */
		mgmt_subtype_mask =
			last_mgmt_subtype_mask | BIT(frame_type >> 4);
	} else {
		/* clear mgmt_subtype_mask */
		mgmt_subtype_mask =
			last_mgmt_subtype_mask & ~BIT(frame_type >> 4);
	}
	PRINTM(MIOCTL,
	       "%s: frame_type=0x%x mgmt_subtype_mask=0x%x last_mgmt_subtype_mask=0x%x\n",
	       priv->netdev->name, frame_type, mgmt_subtype_mask,
	       last_mgmt_subtype_mask);
	if (mgmt_subtype_mask != last_mgmt_subtype_mask) {
		last_mgmt_subtype_mask = mgmt_subtype_mask;
		/* Notify driver that a mgmt frame type was registered.
		 * Note that this callback may not sleep, and cannot run
		 * concurrently with itself.
		 */
		status = woal_reg_rx_mgmt_ind(priv, MLAN_ACT_SET,
					      &mgmt_subtype_mask, MOAL_NO_WAIT);
		priv->mgmt_subtype_mask = last_mgmt_subtype_mask;
	}

	LEAVE();
}
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
/**
 * @brief register/unregister mgmt frame forwarding
 *
 * @param wiphy           A pointer to wiphy structure
 * @param dev             A pointer to net_device structure
 * @param frame_type      Bit mask for mgmt frame type
 * @param reg             Register or unregister
 *
 * @return                0 -- success, otherwise fail
 */
void woal_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
				       struct net_device *dev, u16 frame_type,
				       bool reg)
#else
#if KERNEL_VERSION(5, 8, 0) <= CFG80211_VERSION_CODE
void woal_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       struct mgmt_frame_regs *upd)
#else
/**
 * @brief register/unregister mgmt frame forwarding
 *
 * @param wiphy           A pointer to wiphy structure
 * @param wdev            A pointer to wireless_dev structure
 * @param frame_type      Bit mask for mgmt frame type
 * @param reg             Register or unregister
 *
 * @return                0 -- success, otherwise fail
 */
void woal_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
				       struct wireless_dev *wdev,
				       u16 frame_type, bool reg)
#endif
#endif
{
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
	struct net_device *dev = wdev->netdev;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);

	ENTER();

#if KERNEL_VERSION(5, 8, 0) <= CFG80211_VERSION_CODE
	if ((upd->interface_stypes & BIT(IEEE80211_STYPE_AUTH >> 4))
	    /** Supplicant 2.8 always register auth, FW will handle auth when
	     *  host_mlme=0
	     */
	    && !moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
		upd->interface_stypes &= ~BIT(IEEE80211_STYPE_AUTH >> 4);
	woal_reg_rx_mgmt_ind(priv, MLAN_ACT_SET, &upd->interface_stypes,
			     MOAL_NO_WAIT);
#else
	if (frame_type == IEEE80211_STYPE_AUTH
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	    /** Supplicant 2.8 always register auth, FW will handle auth when
	     *  host_mlme=0
	     */
	    && !moal_extflg_isset(priv->phandle, EXT_HOST_MLME)
#endif
	) {
		LEAVE();
		return;
	}
	woal_mgmt_frame_register(priv, frame_type, reg);
#endif
	LEAVE();
}

#if KERNEL_VERSION(3, 2, 0) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 3, 0) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param params                A pointer to cfg80211_mgmt_tx_params structure
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param offchan               Off channel or not
 * @param wait                  Duration to wait
 * @param buf                   Frame buffer
 * @param len                   Frame length
 * @param no_cck                No CCK check
 * @param dont_wait_for_ack     Do not wait for ACK
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#endif
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param wdev                  A pointer to wireless_dev structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param offchan               Off channel or not
 * @param channel_type          Channel type
 * @param channel_type_valid    Is channel type valid or not
 * @param wait                  Duration to wait
 * @param buf                   Frame buffer
 * @param len                   Frame length
 * @param no_cck                No CCK check
 * @param dont_wait_for_ack     Do not wait for ACK
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#endif
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param offchan               Off channel or not
 * @param channel_type          Channel type
 * @param channel_type_valid    Is channel type valid or not
 * @param wait                  Duration to wait
 * @param buf                   Frame buffer
 * @param len                   Frame length
 * @param no_cck                No CCK check
 * @param dont_wait_for_ack     Do not wait for ACK
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#endif
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param offchan               Off channel or not
 * @param channel_type          Channel type
 * @param channel_type_valid    Is channel type valid or not
 * @param wait                  Duration to wait
 * @param buf                   Frame buffer
 * @param len                   Frame length
 * @param no_cck                No CCK check
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#endif
#else
/**
 * @brief tx mgmt frame
 *
 * @param wiphy                 A pointer to wiphy structure
 * @param dev                   A pointer to net_device structure
 * @param chan                  A pointer to ieee80211_channel structure
 * @param offchan               Off channel or not
 * @param channel_type          Channel type
 * @param channel_type_valid    Is channel type valid or not
 * @param wait                  Duration to wait
 * @param buf                   Frame buffer
 * @param len                   Frame length
 * @param cookie                A pointer to frame cookie
 *
 * @return                0 -- success, otherwise fail
 */
#endif
int woal_cfg80211_mgmt_tx(struct wiphy *wiphy,
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
			  struct net_device *dev,
#else
			  struct wireless_dev *wdev,
#endif
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
			  struct cfg80211_mgmt_tx_params *params,
#else
			  struct ieee80211_channel *chan, bool offchan,
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
			  enum nl80211_channel_type channel_type,
			  bool channel_type_valid,
#endif
			  unsigned int wait, const u8 *buf, size_t len,
#if KERNEL_VERSION(3, 2, 0) <= CFG80211_VERSION_CODE
			  bool no_cck,
#endif
#if KERNEL_VERSION(3, 3, 0) <= CFG80211_VERSION_CODE
			  bool dont_wait_for_ack,
#endif
#endif
			  u64 *cookie)
{
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
	struct net_device *dev = wdev->netdev;
#endif
#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
	struct ieee80211_channel *chan = params->chan;
	unsigned int wait = params->wait;
	const u8 *buf = params->buf;
	size_t len = params->len;
#endif
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int ret = 0;
	pmlan_buffer pmbuf = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u16 packet_len = 0;
	t_u8 addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	t_u32 pkt_type;
	t_u32 tx_control;
#if KERNEL_VERSION(2, 6, 39) <= CFG80211_VERSION_CODE
	t_u8 channel_status;
	t_u32 duration;
	moal_private *remain_priv = NULL;
#endif

	unsigned long flags;
	struct sk_buff *skb = NULL;
	struct tx_status_info *tx_info = NULL;
	t_u32 remain_len = 0;
	t_u16 fc, type, stype;

	ENTER();

	if (buf == NULL || len == 0) {
		PRINTM(MERROR, "%s: corrupt data\n", __func__);
		LEAVE();
		return -EFAULT;
	}

	/* If the packet is probe response, that means we are in listen phase,
	 * so we should not call remain_on_channel_cfg because
	 * remain_on_channl already handled it. If the packet if action, that
	 * means we are in PD/GO negotiation, so we should call
	 * remain_on_channel_cfg in order to receive action frame from peer
	 * device
	 */
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		if (ieee80211_is_probe_resp(
			    ((struct ieee80211_mgmt *)buf)->frame_control)) {
			PRINTM(MIOCTL, "Skip send probe_resp in GO/UAP mode\n");
			goto done;
		}
		fc = le16_to_cpu(((struct ieee80211_mgmt *)buf)->frame_control);
		type = fc & IEEE80211_FCTL_FTYPE;
		stype = fc & IEEE80211_FCTL_STYPE;
		if (type == IEEE80211_FTYPE_MGMT) {
			switch (stype) {
			case IEEE80211_STYPE_AUTH:
				PRINTM(MMSG, "wlan: HostMlme %s send Auth\n",
				       priv->netdev->name);
				break;
			case IEEE80211_STYPE_DEAUTH:
			case IEEE80211_STYPE_DISASSOC:
				PRINTM(MMSG,
				       "wlan: HostMlme %s send deauth/disassoc\n",
				       priv->netdev->name);
				break;
			case IEEE80211_STYPE_ASSOC_RESP:
			case IEEE80211_STYPE_REASSOC_RESP:
				PRINTM(MMSG,
				       "wlan: HostMlme %s send assoc/reassoc resp\n",
				       priv->netdev->name);
				break;
			default:
				break;
			}
		}
	}
#if KERNEL_VERSION(2, 6, 39) <= CFG80211_VERSION_CODE
	if ((ieee80211_is_action(((struct ieee80211_mgmt *)buf)->frame_control))
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	    || moal_extflg_isset(priv->phandle, EXT_HOST_MLME)
#endif
	) {
#ifdef WIFI_DIRECT_SUPPORT
		if (priv->bss_type == MLAN_BSS_TYPE_WIFIDIRECT)
			woal_cfg80211_display_p2p_actframe(buf, len, chan,
							   MTRUE);
		if (priv->phandle->is_go_timer_set) {
			woal_cancel_timer(&priv->phandle->go_timer);
			priv->phandle->is_go_timer_set = MFALSE;
		}
#endif
		if (priv->phandle->is_remain_timer_set) {
			woal_cancel_timer(&priv->phandle->remain_timer);
			woal_remain_timer_func(priv->phandle);
		}
		/* With sd8777 We have difficulty to receive response packet in
		 * 500ms
		 */
#define MGMT_TX_DEFAULT_WAIT_TIME 1500
		if (priv->phandle->remain_on_channel)
			remain_priv =
				priv->phandle
					->priv[priv->phandle->remain_bss_index];
		/** cancel previous remain on channel */
		if (priv->phandle->remain_on_channel && remain_priv) {
			if (woal_cfg80211_remain_on_channel_cfg(
				    remain_priv, MOAL_IOCTL_WAIT, MTRUE,
				    &channel_status, NULL, 0, 0))
				PRINTM(MERROR,
				       "mgmt_tx:Fail to cancel remain on channel\n");
			if (priv->phandle->cookie) {
				cfg80211_remain_on_channel_expired(
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
					remain_priv->netdev,
#else
					remain_priv->wdev,
#endif
					priv->phandle->cookie,
					&priv->phandle->chan,
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
					priv->phandle->channel_type,
#endif
					GFP_ATOMIC);
				priv->phandle->cookie = 0;
			}
			priv->phandle->remain_on_channel = MFALSE;
		}
#ifdef STA_CFG80211
		/** cancel pending scan */
		woal_cancel_scan(priv, MOAL_IOCTL_WAIT);
#endif

		if (chan) {
			duration = (wait > MGMT_TX_DEFAULT_WAIT_TIME) ?
					   wait :
					   MGMT_TX_DEFAULT_WAIT_TIME;
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
			if (channel_type_valid)
				ret = woal_cfg80211_remain_on_channel_cfg(
					priv, MOAL_IOCTL_WAIT, MFALSE,
					&channel_status, chan, channel_type,
					duration);
			else
#endif
				ret = woal_cfg80211_remain_on_channel_cfg(
					priv, MOAL_IOCTL_WAIT, MFALSE,
					&channel_status, chan, 0, duration);
			if (ret) {
				/* Return fail will cause p2p connection fail
				 */
				woal_sched_timeout(2);
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
				if (channel_type_valid)
					ret = woal_cfg80211_remain_on_channel_cfg(
						priv, MOAL_IOCTL_WAIT, MFALSE,
						&channel_status, chan,
						channel_type, duration);
				else
#endif
					ret = woal_cfg80211_remain_on_channel_cfg(
						priv, MOAL_IOCTL_WAIT, MFALSE,
						&channel_status, chan, 0,
						duration);
				PRINTM(MERROR,
				       "Try configure remain on channel again, ret=%d\n",
				       ret);
				ret = 0;
			} else {
				priv->phandle->remain_on_channel = MTRUE;
				priv->phandle->remain_bss_index =
					priv->bss_index;
#if KERNEL_VERSION(3, 8, 0) > CFG80211_VERSION_CODE
				priv->phandle->channel_type = channel_type;
#endif
				moal_memcpy_ext(
					priv->phandle, &priv->phandle->chan,
					chan, sizeof(struct ieee80211_channel),
					sizeof(struct ieee80211_channel));
				PRINTM(MIOCTL,
				       "%s: Mgmt Tx: Set remain channel=%d duration=%d\n",
				       dev->name,
				       ieee80211_frequency_to_channel(
					       chan->center_freq),
				       duration);
			}
		}
	}
#endif

	/* pkt_type + tx_control */
#define HEADER_SIZE 8
	packet_len = (t_u16)len + MLAN_MAC_ADDR_LENGTH;
	pmbuf = woal_alloc_mlan_buffer(priv->phandle,
				       MLAN_MIN_DATA_HEADER_LEN + HEADER_SIZE +
					       packet_len + sizeof(packet_len));
	if (!pmbuf) {
		PRINTM(MERROR, "Fail to allocate mlan_buffer\n");
		ret = -ENOMEM;
		goto done;
	}
#if KERNEL_VERSION(3, 8, 0) > LINUX_VERSION_CODE
	*cookie = random32() | 1;
#else
	*cookie = prandom_u32() | 1;
#endif
	pmbuf->data_offset = MLAN_MIN_DATA_HEADER_LEN;
	pkt_type = MRVL_PKT_TYPE_MGMT_FRAME;
	tx_control = 0;
	remain_len = HEADER_SIZE + packet_len + sizeof(packet_len);
	/* Add pkt_type and tx_control */
	moal_memcpy_ext(priv->phandle, pmbuf->pbuf + pmbuf->data_offset,
			&pkt_type, sizeof(pkt_type), remain_len);
	remain_len -= sizeof(pkt_type);
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + sizeof(pkt_type),
			&tx_control, sizeof(tx_control), remain_len);
	remain_len -= sizeof(tx_control);
	/* frmctl + durationid + addr1 + addr2 + addr3 + seqctl */
#define PACKET_ADDR4_POS (2 + 2 + 6 + 6 + 6 + 2)
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE,
			&packet_len, sizeof(packet_len), remain_len);
	remain_len -= sizeof(packet_len);
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE +
				sizeof(packet_len),
			buf, PACKET_ADDR4_POS, remain_len);
	remain_len -= PACKET_ADDR4_POS;
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE +
				sizeof(packet_len) + PACKET_ADDR4_POS,
			addr, MLAN_MAC_ADDR_LENGTH, remain_len);
	remain_len -= MLAN_MAC_ADDR_LENGTH;
	moal_memcpy_ext(priv->phandle,
			pmbuf->pbuf + pmbuf->data_offset + HEADER_SIZE +
				sizeof(packet_len) + PACKET_ADDR4_POS +
				MLAN_MAC_ADDR_LENGTH,
			buf + PACKET_ADDR4_POS, len - PACKET_ADDR4_POS,
			remain_len);

	pmbuf->data_len = HEADER_SIZE + packet_len + sizeof(packet_len);
	pmbuf->buf_type = MLAN_BUF_TYPE_RAW_DATA;
	pmbuf->bss_index = priv->bss_index;
	if ((ieee80211_is_action(((struct ieee80211_mgmt *)buf)->frame_control))
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	    || moal_extflg_isset(priv->phandle, EXT_HOST_MLME)
#endif
	) {
		pmbuf->flags = MLAN_BUF_FLAG_TX_STATUS;
		if (!priv->tx_seq_num)
			priv->tx_seq_num++;
		pmbuf->tx_seq_num = priv->tx_seq_num++;
		tx_info = kzalloc(sizeof(struct tx_status_info), GFP_ATOMIC);
		if (tx_info) {
			skb = alloc_skb(len, GFP_ATOMIC);
			if (skb) {
				moal_memcpy_ext(priv->phandle, skb->data, buf,
						len, len);
				skb_put(skb, len);
				spin_lock_irqsave(&priv->tx_stat_lock, flags);
				tx_info->tx_cookie = *cookie;
				tx_info->tx_skb = skb;
				tx_info->tx_seq_num = pmbuf->tx_seq_num;
				if ((priv->bss_role == MLAN_BSS_ROLE_UAP) &&
				    (priv->phandle->remain_on_channel && !wait))
					tx_info->cancel_remain_on_channel =
						MTRUE;
				INIT_LIST_HEAD(&tx_info->link);
				list_add_tail(&tx_info->link,
					      &priv->tx_stat_queue);
				spin_unlock_irqrestore(&priv->tx_stat_lock,
						       flags);
			} else {
				kfree(tx_info);
				tx_info = NULL;
			}
		}
	}

	status = mlan_send_packet(priv->phandle->pmlan_adapter, pmbuf);

	switch (status) {
	case MLAN_STATUS_PENDING:
		atomic_inc(&priv->phandle->tx_pending);
		queue_work(priv->phandle->workqueue, &priv->phandle->main_work);

		/* Delay 30ms to guarantee the packet has been already tx'ed,
		 * because if we call cfg80211_mgmt_tx_status() immediately,
		 * then wpa_supplicant will call cancel_remain_on_channel(),
		 * which may affect the mgmt frame tx. Meanwhile it is only
		 * necessary for P2P action handshake to wait 30ms.
		 */
		if ((ieee80211_is_action(
			    ((struct ieee80211_mgmt *)buf)->frame_control))
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
		    || moal_extflg_isset(priv->phandle, EXT_HOST_MLME)
#endif
		) {
			if (tx_info)
				break;
			else
				woal_sched_timeout(30);
		}
		/* Notify the mgmt tx status */
#if KERNEL_VERSION(2, 6, 37) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 6, 0) > CFG80211_VERSION_CODE
		cfg80211_mgmt_tx_status(dev, *cookie, buf, len, true,
					GFP_ATOMIC);
#else
		cfg80211_mgmt_tx_status(priv->wdev, *cookie, buf, len, true,
					GFP_ATOMIC);
#endif
#endif
		break;
	case MLAN_STATUS_SUCCESS:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		break;
	case MLAN_STATUS_FAILURE:
	default:
		woal_free_mlan_buffer(priv->phandle, pmbuf);
		ret = -EFAULT;
		break;
	}

done:

	if (status != MLAN_STATUS_PENDING) {
		if (tx_info)
			woal_remove_tx_info(priv, tx_info->tx_seq_num);
	}

	LEAVE();
	return ret;
}

/**
 * @brief Add custom ie to mgmt frames.
 *
 * @param priv                  A pointer to moal private structure
 * @param beacon_ies_data       Beacon ie
 * @param beacon_index          The index for beacon when auto index
 * @param proberesp_ies_data    Probe resp ie
 * @param proberesp_index       The index for probe resp when auto index
 * @param assocresp_ies_data    Assoc resp ie
 * @param assocresp_index       The index for assoc resp when auto index
 * @param probereq_ies_data     Probe req ie
 * @param probereq_index        The index for probe req when auto index
 * @param wait_option           wait option
 *
 * @return              0 -- success, otherwise fail
 */
static mlan_status
woal_cfg80211_custom_ie(moal_private *priv, custom_ie *beacon_ies_data,
			t_u16 *beacon_index, custom_ie *proberesp_ies_data,
			t_u16 *proberesp_index, custom_ie *assocresp_ies_data,
			t_u16 *assocresp_index, custom_ie *probereq_ies_data,
			t_u16 *probereq_index, t_u8 wait_option)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_custom_ie *custom_ie = NULL;
	t_u8 *pos = NULL;
	t_u16 len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u32 remain_len = 0;

	ENTER();

	custom_ie = kzalloc(sizeof(mlan_ds_misc_custom_ie), GFP_KERNEL);
	if (!custom_ie) {
		PRINTM(MERROR, "Fail to allocate custome_ie\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	custom_ie->type = TLV_TYPE_MGMT_IE;

	pos = (t_u8 *)custom_ie->ie_data_list;
	remain_len = sizeof(custom_ie->ie_data_list);
	if (beacon_ies_data) {
		len = sizeof(*beacon_ies_data) - MAX_IE_SIZE +
		      beacon_ies_data->ie_length;
		moal_memcpy_ext(priv->phandle, pos, beacon_ies_data, len,
				remain_len);
		pos += len;
		remain_len -= len;
		custom_ie->len += len;
	}

	if (proberesp_ies_data) {
		len = sizeof(*proberesp_ies_data) - MAX_IE_SIZE +
		      proberesp_ies_data->ie_length;
		moal_memcpy_ext(priv->phandle, pos, proberesp_ies_data, len,
				remain_len);
		pos += len;
		remain_len -= len;
		custom_ie->len += len;
	}

	if (assocresp_ies_data) {
		len = sizeof(*assocresp_ies_data) - MAX_IE_SIZE +
		      assocresp_ies_data->ie_length;
		moal_memcpy_ext(priv->phandle, pos, assocresp_ies_data, len,
				remain_len);
		pos += len;
		remain_len -= len;
		custom_ie->len += len;
	}

	if (probereq_ies_data) {
		len = sizeof(*probereq_ies_data) - MAX_IE_SIZE +
		      probereq_ies_data->ie_length;
		moal_memcpy_ext(priv->phandle, pos, probereq_ies_data, len,
				remain_len);
		pos += len;
		remain_len -= len;
		custom_ie->len += len;
	}
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		PRINTM(MERROR, "Fail to allocate ioctl_req\n");
		status = MLAN_STATUS_FAILURE;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &misc->param.cust_ie, custom_ie,
			sizeof(mlan_ds_misc_custom_ie),
			sizeof(mlan_ds_misc_custom_ie));

	status = woal_request_ioctl(priv, ioctl_req, wait_option);
	if (status != MLAN_STATUS_SUCCESS)
		goto done;

	/* get the assigned index */
	pos = (t_u8 *)(&misc->param.cust_ie.ie_data_list[0].ie_index);
	if (beacon_ies_data && beacon_ies_data->ie_length &&
	    beacon_ies_data->ie_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		/* save beacon ie index after auto-indexing */
		*beacon_index = misc->param.cust_ie.ie_data_list[0].ie_index;
		len = sizeof(*beacon_ies_data) - MAX_IE_SIZE +
		      beacon_ies_data->ie_length;
		pos += len;
	}

	if (proberesp_ies_data && proberesp_ies_data->ie_length &&
	    proberesp_ies_data->ie_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		/* save probe resp ie index after auto-indexing */
		*proberesp_index = *((t_u16 *)pos);
		len = sizeof(*proberesp_ies_data) - MAX_IE_SIZE +
		      proberesp_ies_data->ie_length;
		pos += len;
	}

	if (assocresp_ies_data && assocresp_ies_data->ie_length &&
	    assocresp_ies_data->ie_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		/* save assoc resp ie index after auto-indexing */
		*assocresp_index = *((t_u16 *)pos);
		len = sizeof(*assocresp_ies_data) - MAX_IE_SIZE +
		      assocresp_ies_data->ie_length;
		pos += len;
	}
	if (probereq_ies_data && probereq_ies_data->ie_length &&
	    probereq_ies_data->ie_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
		/* save probe resp ie index after auto-indexing */
		*probereq_index = *((t_u16 *)pos);
		len = sizeof(*probereq_ies_data) - MAX_IE_SIZE +
		      probereq_ies_data->ie_length;
		pos += len;
	}
	// TODO why we check status_code at end
	if (ioctl_req->status_code == MLAN_ERROR_IOCTL_FAIL)
		status = MLAN_STATUS_FAILURE;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	kfree(custom_ie);
	LEAVE();
	return status;
}

#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
/**
 * @brief set Qos map
 *
 * @param wiphy         A pointer to wiphy structure
 * @param dev           A pointer to net_device structure
 * @param qos_map       A pointer to cfg80211_qos_map structure
 *
 *
 * @return              0 -- success, otherwise fail
 */
int woal_cfg80211_set_qos_map(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_qos_map *qos_map)
{
	moal_private *priv = (moal_private *)woal_get_netdev_priv(dev);
	int i, j, ret = 0;

	ENTER();
	/**clear dscp map*/
	if (!qos_map) {
		memset(priv->dscp_map, 0xFF, sizeof(priv->dscp_map));
		goto done;
	}

	/**update dscp map*/
	for (i = 0; i < MAX_NUM_TID; i++) {
		PRINTM(MINFO, "TID %d: dscp_low=%d, dscp_high=%d\n", i,
		       qos_map->up[i].low, qos_map->up[i].high);
		if (qos_map->up[i].low != 0xff && qos_map->up[i].high != 0xff &&
		    qos_map->up[i].high <= 63) {
			for (j = qos_map->up[i].low; j <= qos_map->up[i].high;
			     j++)
				priv->dscp_map[j] = i;
		}
	}

	for (i = 0; i < qos_map->num_des; i++) {
		if ((qos_map->dscp_exception[i].dscp <= 63) &&
		    (qos_map->dscp_exception[i].up <= 7)) {
			PRINTM(MINFO, "dscp excpt: value=%d priority=%d\n",
			       qos_map->dscp_exception[i].dscp,
			       qos_map->dscp_exception[i].up);
			priv->dscp_map[qos_map->dscp_exception[i].dscp] =
				qos_map->dscp_exception[i].up;
		}
	}

	/**UAP update (re)associate response*/
	if (priv->bss_type == MLAN_BSS_TYPE_UAP) {
		IEEEtypes_Generic_t qos_map_ie;
		t_u16 qos_map_ies_len;

		qos_map_ie.ieee_hdr.element_id = QOS_MAPPING;
		qos_map_ie.ieee_hdr.len =
			2 * qos_map->num_des + sizeof(qos_map->up);
		qos_map_ies_len =
			qos_map_ie.ieee_hdr.len + sizeof(qos_map_ie.ieee_hdr);

		if (qos_map_ies_len > sizeof(qos_map_ie.data)) {
			PRINTM(MERROR,
			       "QoS MAP IE size exceeds the buffer len\n");
			goto done;
		}
		moal_memcpy_ext(priv->phandle, qos_map_ie.data,
				(t_u8 *)qos_map->dscp_exception,
				2 * qos_map->num_des, sizeof(qos_map_ie.data));
		moal_memcpy_ext(priv->phandle,
				&qos_map_ie.data[2 * qos_map->num_des],
				(t_u8 *)qos_map->up, sizeof(qos_map->up),
				sizeof(qos_map_ie.data) - 2 * qos_map->num_des);

		/* set the assoc response ies */
		ret = woal_cfg80211_mgmt_frame_ie(priv, NULL, 0, NULL, 0,
						  (t_u8 *)&qos_map_ie,
						  qos_map_ies_len, NULL, 0,
						  MGMT_MASK_ASSOC_RESP_QOS_MAP,
						  MOAL_IOCTL_WAIT);
		if (ret) {
			PRINTM(MERROR, "Failed to set beacon wps/p2p ie\n");
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}
#endif

/**
 * @brief get specific ie
 *
 * @param ie              Pointer to IEs
 * @param len             Total length of ie
 * @param ie_out          Pointer to out IE buf
 * @param ie_out_len    Total length of ie_out
 * @param mask            IE mask
 *
 * @return                out IE length
 */
static t_u16 woal_get_specific_ie(const t_u8 *ie, int len, t_u8 *ie_out,
				  t_u32 ie_out_len, t_u16 mask)
{
	int left_len = len;
	const t_u8 *pos = ie;
	int length;
	t_u8 id = 0;
	t_u16 out_len = 0;
	IEEEtypes_VendorSpecific_t *pvendor_ie = NULL;
	const u8 wps_oui[4] = {0x00, 0x50, 0xf2, 0x04};
	const u8 p2p_oui[4] = {0x50, 0x6f, 0x9a, 0x09};
	const u8 wfd_oui[4] = {0x50, 0x6f, 0x9a, 0x0a};
	const t_u8 wmm_oui[4] = {0x00, 0x50, 0xf2, 0x02};

	ENTER();
	while (left_len >= 2) {
		length = *(pos + 1);
		id = *pos;
		if ((length + 2) > left_len)
			break;
		if (id == VENDOR_SPECIFIC_221) {
			pvendor_ie = (IEEEtypes_VendorSpecific_t *)pos;
			if (!memcmp(pvendor_ie->vend_hdr.oui, wmm_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    pvendor_ie->vend_hdr.oui_type == wmm_oui[3]) {
				PRINTM(MIOCTL, "find WMM IE\n");
			} else if (!memcmp(pvendor_ie->vend_hdr.oui, p2p_oui,
					   sizeof(pvendor_ie->vend_hdr.oui)) &&
				   pvendor_ie->vend_hdr.oui_type ==
					   p2p_oui[3]) {
				if (mask & IE_MASK_P2P) {
					/** only get first p2p ie here */
					moal_memcpy_ext(NULL, ie_out + out_len,
							pos, length + 2,
							ie_out_len - out_len);
					out_len += length + 2;
					break;
				}
			} else if (!memcmp(pvendor_ie->vend_hdr.oui, wps_oui,
					   sizeof(pvendor_ie->vend_hdr.oui)) &&
				   pvendor_ie->vend_hdr.oui_type ==
					   wps_oui[3]) {
				if (mask & IE_MASK_WPS) {
					if ((out_len + length + 2) <
					    ie_out_len) {
						moal_memcpy_ext(
							NULL, ie_out + out_len,
							pos, length + 2,
							ie_out_len - out_len);
						out_len += length + 2;
					} else {
						PRINTM(MERROR,
						       "get_specific_ie: IE too big, fail copy WPS IE\n");
						break;
					}
				}
			} else if (!memcmp(pvendor_ie->vend_hdr.oui, wfd_oui,
					   sizeof(pvendor_ie->vend_hdr.oui)) &&
				   pvendor_ie->vend_hdr.oui_type ==
					   wfd_oui[3]) {
				if (mask & IE_MASK_WFD) {
					if ((out_len + length + 2) <
					    ie_out_len) {
						moal_memcpy_ext(
							NULL, ie_out + out_len,
							pos, length + 2,
							ie_out_len - out_len);
						out_len += length + 2;
					} else {
						PRINTM(MERROR,
						       "get_specific_ie: IE too big, fail copy WFD IE\n");
						break;
					}
				}
			} else if (mask & IE_MASK_VENDOR) {
				if ((out_len + length + 2) < ie_out_len) {
					moal_memcpy_ext(NULL, ie_out + out_len,
							pos, length + 2,
							ie_out_len - out_len);
					out_len += length + 2;
				} else {
					PRINTM(MERROR,
					       "get_specific_ie:IE too big, fail copy VENDOR IE\n");
					break;
				}
			}
		}
		pos += (length + 2);
		left_len -= (length + 2);
	}
	LEAVE();
	return out_len;
}

/**
 * @brief Find specific IE from IE buffer
 *
 * @param ie              Pointer to IEs
 * @param len             Total length of ie
 * @param spec_ie         Pointer to specific IE buffer
 * @param spec_len        Total length of specific IE
 *
 * @return                out IE length
 */
static t_u8 woal_find_ie(const t_u8 *ie, int len, const t_u8 *spec_ie,
			 int spec_len)
{
	int left_len = len;
	const t_u8 *pos = ie;
	int length;
	t_u8 id = 0;

	while (left_len >= 2) {
		length = *(pos + 1);
		id = *pos;
		if ((length + 2) > left_len)
			break;
		if ((length + 2) == spec_len) {
			if (!memcmp(pos, spec_ie, spec_len))
				return MTRUE;
		}
		pos += (length + 2);
		left_len -= (length + 2);
	}
	return MFALSE;
}

/**
 * @brief Filter specific IE in ie buf
 *
 * @param priv            pointer to moal private structure
 * @param ie              Pointer to IEs
 * @param len             Total length of ie
 * @param ie_out		  Pointer to out IE buf
 * @param ie_out_len      Total length of ie_out
 * @param wps_flag	      flag for wps/p2p
 * @param dup_ie          Pointer to duplicate ie
 * @param dup_ie_len	  duplicate IE len
 *
 * @return                out IE length
 */
static t_u16 woal_filter_beacon_ies(moal_private *priv, const t_u8 *ie, int len,
				    t_u8 *ie_out, t_u32 ie_out_len,
				    t_u16 wps_flag, const t_u8 *dup_ie,
				    int dup_ie_len)
{
	int left_len = len;
	const t_u8 *pos = ie;
	int length;
	t_u8 id = 0;
	t_u16 out_len = 0;
	IEEEtypes_VendorSpecific_t *pvendor_ie = NULL;
	const u8 wps_oui[4] = {0x00, 0x50, 0xf2, 0x04};
	const u8 p2p_oui[4] = {0x50, 0x6f, 0x9a, 0x09};
	const u8 wfd_oui[4] = {0x50, 0x6f, 0x9a, 0x0a};
	const t_u8 wmm_oui[4] = {0x00, 0x50, 0xf2, 0x02};
	t_u8 find_p2p_ie = MFALSE;
	t_u8 enable_11d = MFALSE;
	t_u8 ext_id = 0;
	int ie_len;

	/* ERP_INFO/EXTENDED_SUPPORT_RATES/HT_CAPABILITY/HT_OPERATION/WMM
	 * and WPS/P2P/WFD IE will be fileter out
	 */
	while (left_len >= 2) {
		length = *(pos + 1);
		id = *pos;
		if ((length + 2) > left_len)
			break;
		if (dup_ie && dup_ie_len &&
		    woal_find_ie(dup_ie, dup_ie_len, pos, length + 2)) {
			PRINTM(MIOCTL, "skip duplicate IE\n");
			pos += (length + 2);
			left_len -= (length + 2);
			continue;
		}
		switch (id) {
		case COUNTRY_INFO:
			enable_11d = MTRUE;
			if ((out_len + length + 2) < ie_out_len) {
				moal_memcpy_ext(priv->phandle, ie_out + out_len,
						pos, length + 2,
						ie_out_len - out_len);
				out_len += length + 2;
			} else {
				PRINTM(MERROR,
				       "IE too big, fail copy COUNTRY INFO IE\n");
			}
			break;
		case HT_CAPABILITY:
		case HT_OPERATION:
		case VHT_CAPABILITY:
		case VHT_OPERATION:
			if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME)) {
				if ((out_len + length + 2) < ie_out_len) {
					moal_memcpy_ext(priv->phandle,
							ie_out + out_len, pos,
							length + 2,
							ie_out_len - out_len);
					out_len += length + 2;
				} else {
					PRINTM(MERROR,
					       "IE too big, fail copy COUNTRY INFO IE\n");
				}
			}
			break;
		case EXTENDED_SUPPORTED_RATES:
		case WLAN_EID_ERP_INFO:
		/* Fall Through */
		case REGULATORY_CLASS:
		/* Fall Through */
		case OVERLAPBSSSCANPARAM:
		/* Fall Through */
		case WAPI_IE:
			break;
		case EXTENSION:
			ext_id = *(pos + 2);
			if ((ext_id == HE_CAPABILITY ||
			     ext_id == HE_OPERATION) &&
			    !moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
				break;
			else {
				if ((out_len + length + 2) < ie_out_len) {
					moal_memcpy_ext(priv->phandle,
							ie_out + out_len, pos,
							length + 2,
							ie_out_len - out_len);
					out_len += length + 2;
				} else {
					PRINTM(MERROR,
					       "IE too big, fail copy EXTENSION IE\n");
				}
				break;
			}
		case EXT_CAPABILITY:
			/* filter out EXTCAP */
			if (wps_flag & IE_MASK_EXTCAP) {
				ie_len = length + 2;
				if (MLAN_STATUS_SUCCESS !=
				    woal_set_get_gen_ie(priv, MLAN_ACT_SET,
							(t_u8 *)pos, &ie_len,
							MOAL_IOCTL_WAIT))
					PRINTM(MERROR,
					       "Fail to set EXTCAP IE\n");
				break;
			}
			if ((out_len + length + 2) < ie_out_len) {
				moal_memcpy_ext(priv->phandle, ie_out + out_len,
						pos, length + 2,
						ie_out_len - out_len);
				out_len += length + 2;
			} else {
				PRINTM(MERROR,
				       "IE too big, fail copy EXTCAP IE\n");
			}
			break;
		case VENDOR_SPECIFIC_221:
			/* filter out wmm ie */
			pvendor_ie = (IEEEtypes_VendorSpecific_t *)pos;
			if (!memcmp(pvendor_ie->vend_hdr.oui, wmm_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    pvendor_ie->vend_hdr.oui_type == wmm_oui[3]) {
				break;
			}
			/* filter out wps ie */
			else if (!memcmp(pvendor_ie->vend_hdr.oui, wps_oui,
					 sizeof(pvendor_ie->vend_hdr.oui)) &&
				 pvendor_ie->vend_hdr.oui_type == wps_oui[3]) {
				if (wps_flag & IE_MASK_WPS)
					break;
			}
			/* filter out first p2p ie */
			else if (!memcmp(pvendor_ie->vend_hdr.oui, p2p_oui,
					 sizeof(pvendor_ie->vend_hdr.oui)) &&
				 pvendor_ie->vend_hdr.oui_type == p2p_oui[3]) {
				if (!find_p2p_ie && (wps_flag & IE_MASK_P2P)) {
					find_p2p_ie = MTRUE;
					break;
				}
			}
			/* filter out wfd ie */
			else if (!memcmp(pvendor_ie->vend_hdr.oui, wfd_oui,
					 sizeof(pvendor_ie->vend_hdr.oui)) &&
				 pvendor_ie->vend_hdr.oui_type == wfd_oui[3]) {
				if (wps_flag & IE_MASK_WFD)
					break;
			} else if (wps_flag & IE_MASK_VENDOR) {
				// filter out vendor IE
				break;
			}
			if ((out_len + length + 2) < ie_out_len) {
				moal_memcpy_ext(priv->phandle, ie_out + out_len,
						pos, length + 2,
						ie_out_len - out_len);
				out_len += length + 2;
			} else {
				PRINTM(MERROR,
				       "IE too big, fail copy VENDOR_SPECIFIC_221 IE\n");
			}
			break;
		default:
			if ((out_len + length + 2) < ie_out_len) {
				moal_memcpy_ext(priv->phandle, ie_out + out_len,
						pos, length + 2,
						ie_out_len - out_len);
				out_len += length + 2;
			} else {
				PRINTM(MERROR, "IE too big, fail copy %d IE\n",
				       id);
			}
			break;
		}
		pos += (length + 2);
		left_len -= (length + 2);
	}

	if (enable_11d)
		woal_set_11d(priv, MOAL_IOCTL_WAIT, MTRUE);
	return out_len;
}

#ifdef WIFI_DIRECT_SUPPORT
/**
 * @brief Check if selected_registrar_on in wps_ie
 *
 * @param ie              Pointer to IEs
 * @param len             Total length of ie
 *
 * @return                MTRUE/MFALSE
 */
t_u8 is_selected_registrar_on(const t_u8 *ie, int len)
{
#define WPS_IE_FIX_LEN 6
#define TLV_ID_SELECTED_REGISTRAR 0x1041
	int left_len = len - WPS_IE_FIX_LEN;

	TLV_Generic_t *tlv = (TLV_Generic_t *)(ie + WPS_IE_FIX_LEN);
	u16 tlv_type, tlv_len;
	u8 *pos = NULL;

	while (left_len > sizeof(TLV_Generic_t)) {
		tlv_type = ntohs(tlv->type);
		tlv_len = ntohs(tlv->len);
		if (tlv_type == TLV_ID_SELECTED_REGISTRAR) {
			PRINTM(MIOCTL, "Selected Registrar found !");
			pos = (u8 *)tlv + sizeof(TLV_Generic_t);
			if (*pos == 1)
				return MTRUE;
			else
				return MFALSE;
		}
		tlv = (TLV_Generic_t *)((u8 *)tlv + tlv_len +
					sizeof(TLV_Generic_t));
		left_len -= tlv_len + sizeof(TLV_Generic_t);
	}
	return MFALSE;
}

/**
 * @brief Check if selected_registrar_on in ies
 *
 * @param ie              Pointer to IEs
 * @param len             Total length of ie
 *
 *
 * @return                MTRUE/MFALSE
 */
static t_u16 woal_is_selected_registrar_on(const t_u8 *ie, int len)
{
	int left_len = len;
	const t_u8 *pos = ie;
	int length;
	t_u8 id = 0;
	IEEEtypes_VendorSpecific_t *pvendor_ie = NULL;
	const u8 wps_oui[4] = {0x00, 0x50, 0xf2, 0x04};

	while (left_len >= 2) {
		length = *(pos + 1);
		id = *pos;
		if ((length + 2) > left_len)
			break;
		switch (id) {
		case VENDOR_SPECIFIC_221:
			pvendor_ie = (IEEEtypes_VendorSpecific_t *)pos;
			if (!memcmp(pvendor_ie->vend_hdr.oui, wps_oui,
				    sizeof(pvendor_ie->vend_hdr.oui)) &&
			    pvendor_ie->vend_hdr.oui_type == wps_oui[3]) {
				PRINTM(MIOCTL, "Find WPS ie\n");
				return is_selected_registrar_on(pos,
								length + 2);
			}
			break;
		default:
			break;
		}
		pos += (length + 2);
		left_len -= (length + 2);
	}
	return MFALSE;
}
#endif

/**
 * @brief config AP or GO for mgmt frame ies.
 *
 * @param priv                  A pointer to moal private structure
 * @param beacon_ies            A pointer to beacon ies
 * @param beacon_ies_len        Beacon ies length
 * @param proberesp_ies         A pointer to probe resp ies
 * @param proberesp_ies_len     Probe resp ies length
 * @param assocresp_ies         A pointer to probe resp ies
 * @param assocresp_ies_len     Assoc resp ies length
 * @param probereq_ies          A pointer to probe req ies
 * @param probereq_ies_len      Probe req ies length *
 * @param mask					Mgmt frame mask
 * @param wait_option           wait_option
 *
 * @return                      0 -- success, otherwise fail
 */
int woal_cfg80211_mgmt_frame_ie(
	moal_private *priv, const t_u8 *beacon_ies, size_t beacon_ies_len,
	const t_u8 *proberesp_ies, size_t proberesp_ies_len,
	const t_u8 *assocresp_ies, size_t assocresp_ies_len,
	const t_u8 *probereq_ies, size_t probereq_ies_len, t_u16 mask,
	t_u8 wait_option)
{
	int ret = 0;
	t_u8 *pos = NULL;
	custom_ie *beacon_ies_data = NULL;
	custom_ie *proberesp_ies_data = NULL;
	custom_ie *assocresp_ies_data = NULL;
	custom_ie *probereq_ies_data = NULL;

	/* static variables for mgmt frame ie auto-indexing */
	t_u16 beacon_index = priv->beacon_index;
	t_u16 proberesp_index = priv->proberesp_index;
	t_u16 assocresp_index = priv->assocresp_index;
	t_u16 probereq_index = priv->probereq_index;
	t_u16 beacon_wps_index = priv->beacon_wps_index;
	t_u16 proberesp_p2p_index = priv->proberesp_p2p_index;
	t_u16 assocrep_qos_map_index = priv->assocresp_qos_map_index;
	t_u16 beacon_vendor_index = priv->beacon_vendor_index;

	ENTER();

	/* we need remove vendor IE from beacon extra IE, vendor IE will be
	 * configure through proberesp_vendor_index
	 */
	if (mask & MGMT_MASK_BEACON_WPS_P2P) {
		beacon_ies_data = kzalloc(sizeof(custom_ie), GFP_KERNEL);
		if (!beacon_ies_data) {
			ret = -ENOMEM;
			goto done;
		}
		if (beacon_ies && beacon_ies_len) {
#ifdef WIFI_DIRECT_SUPPORT
			if (woal_is_selected_registrar_on(beacon_ies,
							  beacon_ies_len)) {
				PRINTM(MIOCTL, "selected_registrar is on\n");
				priv->phandle->is_go_timer_set = MTRUE;
				woal_mod_timer(&priv->phandle->go_timer,
					       MOAL_TIMER_10S);
			} else
				PRINTM(MIOCTL, "selected_registrar is off\n");
#endif
			beacon_ies_data->ie_index = beacon_wps_index;
			beacon_ies_data->mgmt_subtype_mask = MGMT_MASK_BEACON;
			beacon_ies_data->ie_length = woal_filter_beacon_ies(
				priv, beacon_ies, beacon_ies_len,
				beacon_ies_data->ie_buffer, MAX_IE_SIZE,
				IE_MASK_VENDOR, NULL, 0);
			DBG_HEXDUMP(MCMD_D, "beacon extra ie",
				    beacon_ies_data->ie_buffer,
				    beacon_ies_data->ie_length);
		} else {
			/* clear the beacon wps ies */
			if (beacon_wps_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid beacon wps index for mgmt frame ie.\n");
				goto done;
			}

			beacon_ies_data->ie_index = beacon_wps_index;
			beacon_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			beacon_ies_data->ie_length = 0;
			beacon_wps_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
		if ((beacon_ies && beacon_ies_len &&
		     beacon_ies_data->ie_length) ||
		    (beacon_ies_data->mgmt_subtype_mask ==
		     MLAN_CUSTOM_IE_DELETE_MASK)) {
			if (MLAN_STATUS_FAILURE ==
			    woal_cfg80211_custom_ie(
				    priv, beacon_ies_data, &beacon_wps_index,
				    proberesp_ies_data, &proberesp_index,
				    assocresp_ies_data, &assocresp_index,
				    probereq_ies_data, &probereq_index,
				    wait_option)) {
				PRINTM(MERROR, "Fail to set beacon wps IE\n");
				ret = -EFAULT;
			}
			priv->beacon_wps_index = beacon_wps_index;
			PRINTM(MCMND, "beacon_wps_index=0x%x len=%d\n",
			       beacon_wps_index, beacon_ies_data->ie_length);
			goto done;
		}
		kfree(beacon_ies_data); // Further allocation of beacon_ies_data
					// is happening, so need to free here.
		beacon_ies_data = NULL;
	}

	if (mask & MGMT_MASK_ASSOC_RESP_QOS_MAP) {
		assocresp_ies_data = kzalloc(sizeof(custom_ie), GFP_KERNEL);
		if (!assocresp_ies_data) {
			ret = -ENOMEM;
			goto done;
		}
		if (assocresp_ies && assocresp_ies_len) {
			/* set the assoc response qos map ies */
			assocresp_ies_data->ie_index = assocrep_qos_map_index;
			assocresp_ies_data->mgmt_subtype_mask =
				MGMT_MASK_ASSOC_RESP;
			if (MLAN_CUSTOM_IE_AUTO_IDX_MASK ==
			    assocrep_qos_map_index)
				assocresp_ies_data->mgmt_subtype_mask |=
					MLAN_CUSTOM_IE_NEW_MASK;
			if (assocresp_ies_len > MAX_IE_SIZE) {
				PRINTM(MERROR,
				       "IE too big: assocresp_ies_len=%d\n",
				       (int)assocresp_ies_len);
				goto done;
			}
			assocresp_ies_data->ie_length = assocresp_ies_len;
			pos = assocresp_ies_data->ie_buffer;
			moal_memcpy_ext(priv->phandle, pos, assocresp_ies,
					assocresp_ies_len, MAX_IE_SIZE);
			DBG_HEXDUMP(MCMD_D, "Qos Map",
				    assocresp_ies_data->ie_buffer,
				    assocresp_ies_data->ie_length);
		} else {
			/* clear the assoc response qos map ie */
			if (assocrep_qos_map_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid Qos map index for mgmt frame ie.\n");
				goto done;
			}

			assocresp_ies_data->ie_index = assocrep_qos_map_index;
			assocresp_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			assocresp_ies_data->ie_length = 0;
			assocrep_qos_map_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
		if (MLAN_STATUS_FAILURE ==
		    woal_cfg80211_custom_ie(priv, NULL, &beacon_wps_index, NULL,
					    &proberesp_index,
					    assocresp_ies_data,
					    &assocrep_qos_map_index, NULL,
					    &probereq_index, wait_option)) {
			PRINTM(MERROR, "Fail to set Qos map IE\n");
			ret = -EFAULT;
		}
		priv->assocresp_qos_map_index = assocrep_qos_map_index;
		PRINTM(MCMND, "qos map ie index=0x%x len=%d\n",
		       assocrep_qos_map_index, assocresp_ies_data->ie_length);
		goto done;
	}

	if (mask & MGMT_MASK_BEACON) {
		beacon_ies_data = kzalloc(sizeof(custom_ie), GFP_KERNEL);
		if (!beacon_ies_data) {
			ret = -ENOMEM;
			goto done;
		}
	}

	if (mask & MGMT_MASK_PROBE_RESP) {
		/** set or clear proberesp ie */
		if (proberesp_ies_len ||
		    (!proberesp_ies_len && !beacon_ies_len)) {
			proberesp_ies_data =
				kzalloc(sizeof(custom_ie), GFP_KERNEL);
			if (!proberesp_ies_data) {
				ret = -ENOMEM;
				goto done;
			}
		}
	}

	if (mask & MGMT_MASK_ASSOC_RESP) {
		/** set or clear assocresp ie */
		if (assocresp_ies_len ||
		    (!assocresp_ies_len && !beacon_ies_len)) {
			assocresp_ies_data =
				kzalloc(sizeof(custom_ie), GFP_KERNEL);
			if (!assocresp_ies_data) {
				ret = -ENOMEM;
				goto done;
			}
		}
	}
	if (mask & MGMT_MASK_PROBE_REQ) {
		probereq_ies_data = kzalloc(sizeof(custom_ie), GFP_KERNEL);
		if (!probereq_ies_data) {
			ret = -ENOMEM;
			goto done;
		}
	}

	if (beacon_ies_data) {
		if (beacon_ies && beacon_ies_len) {
			/* set the probe response/beacon vendor ies which
			 * includes wpa IE
			 */
			beacon_ies_data->ie_index = beacon_vendor_index;
			beacon_ies_data->mgmt_subtype_mask =
				MGMT_MASK_PROBE_RESP | MGMT_MASK_BEACON;
			if (beacon_vendor_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK)
				beacon_ies_data->mgmt_subtype_mask |=
					MLAN_CUSTOM_IE_NEW_MASK;
			beacon_ies_data->ie_length =
				woal_get_specific_ie(beacon_ies, beacon_ies_len,
						     beacon_ies_data->ie_buffer,
						     MAX_IE_SIZE,
						     IE_MASK_VENDOR);
			DBG_HEXDUMP(MCMD_D, "beacon vendor IE",
				    beacon_ies_data->ie_buffer,
				    beacon_ies_data->ie_length);
		}
		if (beacon_vendor_index != MLAN_CUSTOM_IE_AUTO_IDX_MASK &&
		    !beacon_ies_data->ie_length) {
			/* clear the beacon vendor ies */
			if (beacon_vendor_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid beacon_vendor_index for mgmt frame ie.\n");
				goto done;
			}
			beacon_ies_data->ie_index = beacon_vendor_index;
			beacon_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			beacon_ies_data->ie_length = 0;
			beacon_vendor_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
		if ((beacon_ies && beacon_ies_len &&
		     beacon_ies_data->ie_length) ||
		    (beacon_ies_data->mgmt_subtype_mask ==
		     MLAN_CUSTOM_IE_DELETE_MASK)) {
			if (MLAN_STATUS_FAILURE ==
			    woal_cfg80211_custom_ie(
				    priv, beacon_ies_data, &beacon_vendor_index,
				    NULL, &proberesp_index, NULL,
				    &assocresp_index, NULL, &probereq_index,
				    wait_option)) {
				PRINTM(MERROR,
				       "Fail to set beacon vendor IE\n");
				ret = -EFAULT;
				goto done;
			}
			priv->beacon_vendor_index = beacon_vendor_index;
			PRINTM(MCMND, "beacon_vendor=0x%x len=%d\n",
			       beacon_vendor_index, beacon_ies_data->ie_length);
		}
		memset(beacon_ies_data, 0x00, sizeof(custom_ie));
		if (beacon_ies && beacon_ies_len) {
			/* set the beacon ies */
			/* we need remove vendor IE from beacon tail, vendor/wpa
			 * IE will be configure through beacon_vendor_index
			 */
			beacon_ies_data->ie_index = beacon_index;
			beacon_ies_data->mgmt_subtype_mask =
				MGMT_MASK_BEACON | MGMT_MASK_ASSOC_RESP |
				MGMT_MASK_PROBE_RESP;
			beacon_ies_data->ie_length = woal_filter_beacon_ies(
				priv, beacon_ies, beacon_ies_len,
				beacon_ies_data->ie_buffer, MAX_IE_SIZE,
				IE_MASK_WPS | IE_MASK_WFD | IE_MASK_P2P |
					IE_MASK_VENDOR,
				proberesp_ies, proberesp_ies_len);
			if (beacon_ies_data->ie_length)
				DBG_HEXDUMP(MCMD_D, "beacon ie",
					    beacon_ies_data->ie_buffer,
					    beacon_ies_data->ie_length);
			else {
				kfree(beacon_ies_data);
				beacon_ies_data = NULL;
			}
		} else {
			/* clear the beacon ies */
			if (beacon_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MINFO,
				       "Invalid beacon index for mgmt frame ie.\n");
				goto done;
			}

			beacon_ies_data->ie_index = beacon_index;
			beacon_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			beacon_ies_data->ie_length = 0;
			beacon_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
	}

	if (proberesp_ies_data) {
		if (proberesp_ies && proberesp_ies_len) {
			/* set the probe response p2p ies */
			proberesp_ies_data->ie_index = proberesp_p2p_index;
			proberesp_ies_data->mgmt_subtype_mask =
				MGMT_MASK_PROBE_RESP;
			proberesp_ies_data->ie_length = woal_get_specific_ie(
				proberesp_ies, proberesp_ies_len,
				proberesp_ies_data->ie_buffer, MAX_IE_SIZE,
				IE_MASK_P2P);
			DBG_HEXDUMP(MCMD_D, "proberesp p2p ie",
				    proberesp_ies_data->ie_buffer,
				    proberesp_ies_data->ie_length);
		} else if (proberesp_p2p_index !=
			   MLAN_CUSTOM_IE_AUTO_IDX_MASK) {
			/* clear the probe response p2p ies */
			if (proberesp_p2p_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid proberesp_p2p_index for mgmt frame ie.\n");
				goto done;
			}
			proberesp_ies_data->ie_index = proberesp_p2p_index;
			proberesp_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			proberesp_ies_data->ie_length = 0;
			proberesp_p2p_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
		if ((proberesp_ies && proberesp_ies_len &&
		     proberesp_ies_data->ie_length) ||
		    (proberesp_ies_data->mgmt_subtype_mask ==
		     MLAN_CUSTOM_IE_DELETE_MASK)) {
			if (MLAN_STATUS_FAILURE ==
			    woal_cfg80211_custom_ie(
				    priv, NULL, &beacon_index,
				    proberesp_ies_data, &proberesp_p2p_index,
				    NULL, &assocresp_index, NULL,
				    &probereq_index, wait_option)) {
				PRINTM(MERROR,
				       "Fail to set proberesp p2p IE\n");
				ret = -EFAULT;
				goto done;
			}
			priv->proberesp_p2p_index = proberesp_p2p_index;
			PRINTM(MCMND, "proberesp_p2p=0x%x len=%d\n",
			       proberesp_p2p_index,
			       proberesp_ies_data->ie_length);
		}
		memset(proberesp_ies_data, 0x00, sizeof(custom_ie));
		if (proberesp_ies && proberesp_ies_len) {
			/* set the probe response ies */
			proberesp_ies_data->ie_index = proberesp_index;
			proberesp_ies_data->mgmt_subtype_mask =
				MGMT_MASK_PROBE_RESP;
			if (proberesp_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK)
				proberesp_ies_data->mgmt_subtype_mask |=
					MLAN_CUSTOM_IE_NEW_MASK;
			proberesp_ies_data->ie_length = woal_filter_beacon_ies(
				priv, proberesp_ies, proberesp_ies_len,
				proberesp_ies_data->ie_buffer, MAX_IE_SIZE,
				IE_MASK_P2P | IE_MASK_VENDOR, NULL, 0);
			if (proberesp_ies_data->ie_length) {
				DBG_HEXDUMP(MCMD_D, "proberesp ie",
					    proberesp_ies_data->ie_buffer,
					    proberesp_ies_data->ie_length);
			} else {
				kfree(proberesp_ies_data);
				proberesp_ies_data = NULL;
			}
		} else {
			/* clear the probe response ies */
			if (proberesp_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid probe resp index for mgmt frame ie.\n");
				goto done;
			}
			proberesp_ies_data->ie_index = proberesp_index;
			proberesp_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			proberesp_ies_data->ie_length = 0;
			proberesp_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
	}
	if (assocresp_ies_data) {
		if (assocresp_ies && assocresp_ies_len) {
			/* set the assoc response ies */
			assocresp_ies_data->ie_index = assocresp_index;
			assocresp_ies_data->mgmt_subtype_mask =
				MGMT_MASK_ASSOC_RESP;
			if (assocresp_index == MLAN_CUSTOM_IE_AUTO_IDX_MASK)
				assocresp_ies_data->mgmt_subtype_mask |=
					MLAN_CUSTOM_IE_NEW_MASK;
			if (assocresp_ies_len > MAX_IE_SIZE) {
				PRINTM(MERROR,
				       "IE too big, assocresp_ies_len=%d\n",
				       (int)assocresp_ies_len);
				goto done;
			}
			assocresp_ies_data->ie_length = assocresp_ies_len;
			pos = assocresp_ies_data->ie_buffer;
			moal_memcpy_ext(priv->phandle, pos, assocresp_ies,
					assocresp_ies_len, MAX_IE_SIZE);
			DBG_HEXDUMP(MCMD_D, "assocresp ie",
				    assocresp_ies_data->ie_buffer,
				    assocresp_ies_data->ie_length);
		} else {
			/* clear the assoc response ies */
			if (assocresp_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid assoc resp index for mgmt frame ie.\n");
				goto done;
			}

			assocresp_ies_data->ie_index = assocresp_index;
			assocresp_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			assocresp_ies_data->ie_length = 0;
			assocresp_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
	}

	if (probereq_ies_data) {
		if (probereq_ies && probereq_ies_len) {
			/* set the probe req ies */
			probereq_ies_data->ie_index = probereq_index;
			probereq_ies_data->mgmt_subtype_mask =
				MGMT_MASK_PROBE_REQ;
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
			if (priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
				/* filter out P2P/WFD ie/EXT_CAP ie */
				probereq_ies_data->ie_length =
					woal_filter_beacon_ies(
						priv, probereq_ies,
						probereq_ies_len,
						probereq_ies_data->ie_buffer,
						MAX_IE_SIZE,
						IE_MASK_P2P | IE_MASK_WFD |
							IE_MASK_EXTCAP,
						NULL, 0);
			} else {
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */
				if (probereq_ies_len > MAX_IE_SIZE) {
					PRINTM(MERROR,
					       "IE too big, probereq_ies_len=%d\n",
					       (int)probereq_ies_len);
					goto done;
				}
				probereq_ies_data->ie_length = probereq_ies_len;
				pos = probereq_ies_data->ie_buffer;
				moal_memcpy_ext(priv->phandle, pos,
						probereq_ies, probereq_ies_len,
						MAX_IE_SIZE);
#ifdef WIFI_DIRECT_SUPPORT
#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION
			}
#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */
			if (probereq_ies_data->ie_length)
				DBG_HEXDUMP(MCMD_D, "probereq ie",
					    probereq_ies_data->ie_buffer,
					    probereq_ies_data->ie_length);
			else {
				kfree(probereq_ies_data);
				probereq_ies_data = NULL;
			}
		} else {
			/* clear the probe req ies */
			if (probereq_index > MAX_MGMT_IE_INDEX) {
				PRINTM(MERROR,
				       "Invalid probe req index for mgmt frame ie.\n");
				goto done;
			}
			probereq_ies_data->ie_index = probereq_index;
			probereq_ies_data->mgmt_subtype_mask =
				MLAN_CUSTOM_IE_DELETE_MASK;
			probereq_ies_data->ie_length = 0;
			probereq_index = MLAN_CUSTOM_IE_AUTO_IDX_MASK;
		}
	}

	if (beacon_ies_data || proberesp_ies_data || assocresp_ies_data ||
	    probereq_ies_data) {
		if (MLAN_STATUS_FAILURE ==
		    woal_cfg80211_custom_ie(
			    priv, beacon_ies_data, &beacon_index,
			    proberesp_ies_data, &proberesp_index,
			    assocresp_ies_data, &assocresp_index,
			    probereq_ies_data, &probereq_index, wait_option)) {
			PRINTM(MERROR,
			       "Fail to set beacon proberesp assoc probereq IES\n");
			ret = -EFAULT;
			goto done;
		}
	}
	if (beacon_ies_data) {
		priv->beacon_index = beacon_index;
		PRINTM(MCMND, "beacon ie length = %d\n",
		       beacon_ies_data->ie_length);
	}
	if (assocresp_ies_data) {
		priv->assocresp_index = assocresp_index;
		PRINTM(MCMND, "assocresp ie length = %d\n",
		       assocresp_ies_data->ie_length);
	}
	if (proberesp_ies_data) {
		priv->proberesp_index = proberesp_index;
		PRINTM(MCMND, "proberesp ie length = %d\n",
		       proberesp_ies_data->ie_length);
	}
	if (probereq_ies_data) {
		priv->probereq_index = probereq_index;
		PRINTM(MCMND, "probereq ie length = %d\n",
		       probereq_ies_data->ie_length);
	}
	PRINTM(MCMND, "beacon=%x assocresp=%x proberesp=%x probereq=%x\n",
	       beacon_index, assocresp_index, proberesp_index, probereq_index);
done:
	kfree(beacon_ies_data);
	kfree(proberesp_ies_data);
	kfree(assocresp_ies_data);
	kfree(probereq_ies_data);

	LEAVE();

	return ret;
}

/**
 *  @brief Sets up the CFG802.11 specific HT capability fields
 *  with default values
 *
 *  @param ht_info      A pointer to ieee80211_sta_ht_cap structure
 *  @param dev_cap      Device capability information
 *  @param mcs_set      Device MCS sets
 *
 *  @return             N/A
 */
void woal_cfg80211_setup_ht_cap(struct ieee80211_sta_ht_cap *ht_info,
				t_u32 dev_cap, t_u8 *mcs_set)
{
	ENTER();

	ht_info->ht_supported = true;
	ht_info->ampdu_factor = 0x3;
	ht_info->ampdu_density = 0;

	memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));
	ht_info->cap = 0;
	if (mcs_set)
		moal_memcpy_ext(NULL, ht_info->mcs.rx_mask, mcs_set,
				sizeof(ht_info->mcs.rx_mask),
				sizeof(ht_info->mcs.rx_mask));
	if (dev_cap & MBIT(8)) /* 40Mhz intolarance enabled */
		ht_info->cap |= IEEE80211_HT_CAP_40MHZ_INTOLERANT;
	if (dev_cap & MBIT(17)) /* Channel width 20/40Mhz support */
		ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
	if ((dev_cap >> 20) & 0x03) /* Delayed ACK supported */
		ht_info->cap |= IEEE80211_HT_CAP_DELAY_BA;
	if (dev_cap & MBIT(22)) /* Rx LDPC supported */
		ht_info->cap |= IEEE80211_HT_CAP_LDPC_CODING;
	if (dev_cap & MBIT(23)) /* Short GI @ 20Mhz supported */
		ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
	if (dev_cap & MBIT(24)) /* Short GI @ 40Mhz supported */
		ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
	if (dev_cap & MBIT(25)) /* Tx STBC supported */
		ht_info->cap |= IEEE80211_HT_CAP_TX_STBC;
	if (dev_cap & MBIT(26)) /* Rx STBC supported */
		ht_info->cap |= IEEE80211_HT_CAP_RX_STBC;
	if (dev_cap & MBIT(27)) /* MIMO PS supported */
		ht_info->cap |= 0; /* WLAN_HT_CAP_SM_PS_STATIC */
	else /* Disable HT SM PS */
		ht_info->cap |= IEEE80211_HT_CAP_SM_PS;
	if (dev_cap & MBIT(29)) /* Green field supported */
		ht_info->cap |= IEEE80211_HT_CAP_GRN_FLD;
	if (dev_cap & MBIT(31)) /* MAX AMSDU supported */
		ht_info->cap |= IEEE80211_HT_CAP_MAX_AMSDU;
	/* DSSS/CCK in 40Mhz supported*/
	ht_info->cap |= IEEE80211_HT_CAP_DSSSCCK40;
	ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;

	LEAVE();
}

#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
/**
 *  @brief Sets up the CFG802.11 specific VHT capability fields
 *  with default values
 *
 * @param priv         A pointer to moal private structure
 *  @param vht_cap      A pointer to ieee80211_sta_vht_cap structure
 *
 *  @return             N/A
 */
void woal_cfg80211_setup_vht_cap(moal_private *priv,
				 struct ieee80211_sta_vht_cap *vht_cap)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_status status;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (req == NULL) {
		status = MLAN_STATUS_FAILURE;
		PRINTM(MERROR, "Fail to allocate buf for setup vht_cap\n");
		goto done;
	}
	cfg_11ac = (mlan_ds_11ac_cfg *)req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_VHT_CFG;
	req->req_id = MLAN_IOCTL_11AC_CFG;
	req->action = MLAN_ACT_GET;
	cfg_11ac->param.vht_cfg.band = BAND_SELECT_A;
	cfg_11ac->param.vht_cfg.txrx = MLAN_RADIO_RX;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Fail to get vht_cfg\n");
		goto done;
	}
	vht_cap->vht_supported = true;
	vht_cap->cap = cfg_11ac->param.vht_cfg.vht_cap_info;
	vht_cap->vht_mcs.rx_mcs_map = (t_u16)cfg_11ac->param.vht_cfg.vht_rx_mcs;
	vht_cap->vht_mcs.rx_highest =
		(t_u16)cfg_11ac->param.vht_cfg.vht_rx_max_rate;
	vht_cap->vht_mcs.tx_mcs_map = (t_u16)cfg_11ac->param.vht_cfg.vht_tx_mcs;
	vht_cap->vht_mcs.tx_highest =
		(t_u16)cfg_11ac->param.vht_cfg.vht_tx_max_rate;
	PRINTM(MCMND,
	       "vht_cap=0x%x rx_mcs_map=0x%x rx_max=0x%x tx_mcs_map=0x%x tx_max=0x%x\n",
	       vht_cap->cap, vht_cap->vht_mcs.rx_mcs_map,
	       vht_cap->vht_mcs.rx_highest, vht_cap->vht_mcs.tx_mcs_map,
	       vht_cap->vht_mcs.tx_highest);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
}
#endif

#if KERNEL_VERSION(4, 20, 0) <= CFG80211_VERSION_CODE
/*
===============
11AX CAP for uAP
===============
Note: bits not mentioned below are set to 0.

5G
===
HE MAC Cap:
Bit0:  1  (+HTC HE Support)
Bit25: 1  (OM Control Support. But uAP does not support
	   Tx OM received from the STA, as it does not support UL OFDMA)

HE PHY Cap:
Bit1-7: 0x2 (Supported Channel Width Set.
	     Note it would be changed after 80+80 MHz is supported)
Bit8-11: 0x3 (Punctured Preamble Rx.
	      Note: it would be changed after 80+80 MHz is supported)
Bit12: 0x0 (Device Class)
Bit13: 0x1 (LDPC coding in Payload)
Bit17: 0x1 (NDP with 4xHE-LTF+3.2usGI)
Bit18: 0x1 (STBC Tx <= 80 MHz)
Bit19: 0x1 (STBC Rx <= 80 MHz)
Bit20: 0x1 (Doppler Tx)
Bit21: 0x1 (Doppler Rx)
Bit27-28: 0x1 (DCM Max Constellation Rx)
Bit31: 0x1 (SU Beamformer)
Bit32: 0x1 (SU BeamFormee)
Bit34-36: 0x7 (Beamformee STS <= 80 MHz)
Bit40-42: 0x1 (Number of Sounding Dimentions <= 80 MHz)
Bit53: 0x1 (Partial Bandwidth Extended Range)
Bit55: 0x1 (PPE Threshold Present.
	    Note: PPE threshold may have some changes later)
Bit58: 0x1 (HE SU PPDU and HE MU PPDU with 4xHE-LTF+0.8usGI)
Bit59-61: 0x1 (Max Nc)
Bit75: 0x1 (Rx 1024-QAM Support < 242-tone RU)
*/

#define UAP_HE_MAC_CAP0_MASK 0x00
#define UAP_HE_MAC_CAP1_MASK 0x00
#define UAP_HE_MAC_CAP2_MASK 0x00
#define UAP_HE_MAC_CAP3_MASK 0x00
#define UAP_HE_MAC_CAP4_MASK 0x02
#define UAP_HE_MAC_CAP5_MASK 0x00
#define UAP_HE_PHY_CAP0_MASK 0x04
#define UAP_HE_PHY_CAP1_MASK 0x23
#define UAP_HE_PHY_CAP2_MASK 0x3E
#define UAP_HE_PHY_CAP3_MASK 0x88
#define UAP_HE_PHY_CAP4_MASK 0x1D
#define UAP_HE_PHY_CAP5_MASK 0x01
#define UAP_HE_PHY_CAP6_MASK 0xA0
#define UAP_HE_PHY_CAP7_MASK 0x0C
#define UAP_HE_PHY_CAP8_MASK 0x00
#define UAP_HE_PHY_CAP9_MASK 0x08
#define UAP_HE_PHY_CAP10_MASK 0x00

/*
2G
===
HE MAC Cap:
Bit0:   1  (+HTC HE Support)
Bit25: 1  (OM Control Support. Note: uAP does not support
	Tx OM received from the STA, as it does not support UL OFDMA)

HE PHY Cap:
Bit1-7: 0x1 (Supported Channel Width Set)
Bit8-11: 0x0 (Punctured Preamble Rx)
Bit12: 0x0 (Device Class)
Bit13: 0x1 (LDPC coding in Payload)
Bit17: 0x1 (NDP with 4xLTF+3.2usGI)
Bit18: 0x1 (STBC Tx <= 80 MHz)
Bit19: 0x1 (STBC Rx <= 80 MHz)
Bit20: 0x1 (Doppler Tx)
Bit21: 0x1 (Doppler Rx)
Bit27-28: 0x1 (DCM Max Constellation Rx)
Bit31: 0x1 (SU Beamformer)
Bit32: 0x1 (SU BeamFormee)
Bit34-36: 0x7 (Beamformee STS <= 80 MHz)
Bit40-42: 0x1 (Number of Sounding Dimentions <= 80 MHz)
Bit53: 0x1 (Partial Bandwidth Extended Range)
Bit55: 0x1 (PPE Threshold Present.
	    Note: PPE threshold may have some changes later)
Bit58: 0x1 (HE SU PPDU and HE MU PPDU with 4xHE-LTF+0.8usGI)
Bit59-61: 0x1 (Max Nc)
Bit75: 0x1 (Rx 1024-QAM Support < 242-tone RU)
*/
#define UAP_HE_2G_MAC_CAP0_MASK 0x00
#define UAP_HE_2G_MAC_CAP1_MASK 0x00
#define UAP_HE_2G_MAC_CAP2_MASK 0x00
#define UAP_HE_2G_MAC_CAP3_MASK 0x00
#define UAP_HE_2G_MAC_CAP4_MASK 0x02
#define UAP_HE_2G_MAC_CAP5_MASK 0x00
#define UAP_HE_2G_PHY_CAP0_MASK 0x04
#define UAP_HE_2G_PHY_CAP1_MASK 0x20
#define UAP_HE_2G_PHY_CAP2_MASK 0x3E
#define UAP_HE_2G_PHY_CAP3_MASK 0x88
#define UAP_HE_2G_PHY_CAP4_MASK 0x1D
#define UAP_HE_2G_PHY_CAP5_MASK 0x01
#define UAP_HE_2G_PHY_CAP6_MASK 0xA0
#define UAP_HE_2G_PHY_CAP7_MASK 0x0C
#define UAP_HE_2G_PHY_CAP8_MASK 0x00
#define UAP_HE_2G_PHY_CAP9_MASK 0x08
#define UAP_HE_2G_PHY_CAP10_MASK 0x00

/**
 *  @brief update 11ax ie for AP mode *
 *  @param band     band config
 *  @hecap_ie      a pointer to mlan_ds_11ax_he_capa
 *
 *  @return         0--success, otherwise failure
 */
void woal_uap_update_11ax_ie(t_u8 band, mlan_ds_11ax_he_capa *hecap_ie)
{
	if (band == BAND_5GHZ) {
		hecap_ie->he_mac_cap[0] &= UAP_HE_MAC_CAP0_MASK;
		hecap_ie->he_mac_cap[1] &= UAP_HE_MAC_CAP1_MASK;
		hecap_ie->he_mac_cap[2] &= UAP_HE_MAC_CAP2_MASK;
		hecap_ie->he_mac_cap[3] &= UAP_HE_MAC_CAP3_MASK;
		hecap_ie->he_mac_cap[4] &= UAP_HE_MAC_CAP4_MASK;
		hecap_ie->he_mac_cap[5] &= UAP_HE_MAC_CAP5_MASK;
		hecap_ie->he_phy_cap[0] &= UAP_HE_PHY_CAP0_MASK;
		hecap_ie->he_phy_cap[1] &= UAP_HE_PHY_CAP1_MASK;
		hecap_ie->he_phy_cap[2] &= UAP_HE_PHY_CAP2_MASK;
		hecap_ie->he_phy_cap[3] &= UAP_HE_PHY_CAP3_MASK;
		hecap_ie->he_phy_cap[4] &= UAP_HE_PHY_CAP4_MASK;
		hecap_ie->he_phy_cap[5] &= UAP_HE_PHY_CAP5_MASK;
		hecap_ie->he_phy_cap[6] &= UAP_HE_PHY_CAP6_MASK;
		hecap_ie->he_phy_cap[7] &= UAP_HE_PHY_CAP7_MASK;
		hecap_ie->he_phy_cap[8] &= UAP_HE_PHY_CAP8_MASK;
		hecap_ie->he_phy_cap[9] &= UAP_HE_PHY_CAP9_MASK;
		hecap_ie->he_phy_cap[10] &= UAP_HE_PHY_CAP10_MASK;
	} else {
		hecap_ie->he_mac_cap[0] &= UAP_HE_2G_MAC_CAP0_MASK;
		hecap_ie->he_mac_cap[1] &= UAP_HE_2G_MAC_CAP1_MASK;
		hecap_ie->he_mac_cap[2] &= UAP_HE_2G_MAC_CAP2_MASK;
		hecap_ie->he_mac_cap[3] &= UAP_HE_2G_MAC_CAP3_MASK;
		hecap_ie->he_mac_cap[4] &= UAP_HE_2G_MAC_CAP4_MASK;
		hecap_ie->he_mac_cap[5] &= UAP_HE_2G_MAC_CAP5_MASK;
		hecap_ie->he_phy_cap[0] &= UAP_HE_2G_PHY_CAP0_MASK;
		hecap_ie->he_phy_cap[1] &= UAP_HE_2G_PHY_CAP1_MASK;
		hecap_ie->he_phy_cap[2] &= UAP_HE_2G_PHY_CAP2_MASK;
		hecap_ie->he_phy_cap[3] &= UAP_HE_2G_PHY_CAP3_MASK;
		hecap_ie->he_phy_cap[4] &= UAP_HE_2G_PHY_CAP4_MASK;
		hecap_ie->he_phy_cap[5] &= UAP_HE_2G_PHY_CAP5_MASK;
		hecap_ie->he_phy_cap[6] &= UAP_HE_2G_PHY_CAP6_MASK;
		hecap_ie->he_phy_cap[7] &= UAP_HE_2G_PHY_CAP7_MASK;
		hecap_ie->he_phy_cap[8] &= UAP_HE_2G_PHY_CAP8_MASK;
		hecap_ie->he_phy_cap[9] &= UAP_HE_2G_PHY_CAP9_MASK;
		hecap_ie->he_phy_cap[10] &= UAP_HE_2G_PHY_CAP10_MASK;
	}
	return;
}

/**
 *  @brief Sets up the CFG802.11 specific HE capability fields *  with default
 * values
 *
 *  @param priv         A pointer to moal private structure
 *  @param iftype_data  A pointer to ieee80211_sband_iftype_data structure
 *
 *  @return             N/A
 */
void woal_cfg80211_setup_he_cap(moal_private *priv,
				struct ieee80211_supported_band *band)
{
	mlan_fw_info fw_info;
	struct ieee80211_sband_iftype_data *iftype_data = NULL;
	t_u8 extra_mcs_size = 0;
	int ppe_threshold_len = 0;
	mlan_ds_11ax_he_capa *phe_cap = NULL;
	t_u8 hw_hecap_len;

	woal_request_get_fw_info(priv, MOAL_IOCTL_WAIT, &fw_info);
	if (band->band == NL80211_BAND_5GHZ) {
		phe_cap = (mlan_ds_11ax_he_capa *)fw_info.hw_he_cap;
		hw_hecap_len = fw_info.hw_hecap_len;
		woal_uap_update_11ax_ie(BAND_5GHZ, phe_cap);
	} else {
		phe_cap = (mlan_ds_11ax_he_capa *)fw_info.hw_2g_he_cap;
		hw_hecap_len = fw_info.hw_2g_hecap_len;
		woal_uap_update_11ax_ie(BAND_2GHZ, phe_cap);
	}

	if (!hw_hecap_len)
		return;
	DBG_HEXDUMP(MCMD_D, "Setup HECAP", (u8 *)phe_cap, hw_hecap_len);
	iftype_data =
		kmalloc(sizeof(struct ieee80211_sband_iftype_data), GFP_KERNEL);
	if (!iftype_data) {
		PRINTM(MERROR, "Fail to allocate iftype data\n");
		goto done;
	}
	iftype_data->types_mask =
		MBIT(NL80211_IFTYPE_STATION) | MBIT(NL80211_IFTYPE_AP) |
		MBIT(NL80211_IFTYPE_P2P_CLIENT) | MBIT(NL80211_IFTYPE_P2P_GO);
	iftype_data->he_cap.has_he = true;
	moal_memcpy_ext(priv->phandle,
			iftype_data->he_cap.he_cap_elem.mac_cap_info,
			phe_cap->he_mac_cap, sizeof(phe_cap->he_mac_cap),
			sizeof(iftype_data->he_cap.he_cap_elem.mac_cap_info));
	moal_memcpy_ext(priv->phandle,
			iftype_data->he_cap.he_cap_elem.phy_cap_info,
			phe_cap->he_phy_cap, sizeof(phe_cap->he_phy_cap),
			sizeof(iftype_data->he_cap.he_cap_elem.phy_cap_info));
	memset(&iftype_data->he_cap.he_mcs_nss_supp, 0xff,
	       sizeof(struct ieee80211_he_mcs_nss_supp));
	moal_memcpy_ext(priv->phandle, &iftype_data->he_cap.he_mcs_nss_supp,
			phe_cap->he_txrx_mcs_support,
			sizeof(phe_cap->he_txrx_mcs_support),
			sizeof(struct ieee80211_he_mcs_nss_supp));
	// Support 160Mhz
	if (phe_cap->he_phy_cap[0] & MBIT(3))
		extra_mcs_size += 4;

	// Support 80+80
	if (phe_cap->he_phy_cap[0] & MBIT(4))
		extra_mcs_size += 4;
	if (extra_mcs_size)
		moal_memcpy_ext(
			priv->phandle,
			(t_u8 *)&iftype_data->he_cap.he_mcs_nss_supp.rx_mcs_160,
			phe_cap->val, extra_mcs_size,
			sizeof(struct ieee80211_he_mcs_nss_supp) - 4);

#define HE_CAP_FIX_SIZE 22
	// Support PPE threshold
	ppe_threshold_len = phe_cap->len - HE_CAP_FIX_SIZE - extra_mcs_size;
	if (phe_cap->he_phy_cap[6] & MBIT(7) && ppe_threshold_len) {
		moal_memcpy_ext(priv->phandle, iftype_data->he_cap.ppe_thres,
				&phe_cap->val[extra_mcs_size],
				ppe_threshold_len,
				sizeof(iftype_data->he_cap.ppe_thres));
	} else {
		iftype_data->he_cap.he_cap_elem.phy_cap_info[6] &= ~MBIT(7);
		PRINTM(MCMND, "Clear PPE threshold 0x%x\n",
		       iftype_data->he_cap.he_cap_elem.phy_cap_info[7]);
	}
	band->n_iftype_data = 1;
	band->iftype_data = iftype_data;
done:
	LEAVE();
}

/**
 *  @brief free iftype_data
 *
 *  @param wiphy        A pointer to struct wiphy
 *
 *
 *  @return             N/A
 */
void woal_cfg80211_free_iftype_data(struct wiphy *wiphy)
{
	enum nl80211_band band;

	for (band = NL80211_BAND_2GHZ; band < NUM_NL80211_BANDS; ++band) {
		if (!wiphy->bands[band])
			continue;
		if (!wiphy->bands[band]->iftype_data)
			continue;
		kfree(wiphy->bands[band]->iftype_data);
		wiphy->bands[band]->n_iftype_data = 0;
	}
}
#endif

/*
 * @brief  prepare and send fake deauth packet to cfg80211 to
 *         notify wpa_supplicant about disconnection
 *	       <host_mlme, wiphy suspend case>
 *
 * @param priv           A pointer moal_private structure
 * @param reason_code    disconnect reason code
 *
 * @return          N/A
 */
void woal_deauth_event(moal_private *priv, int reason_code)
{
	struct woal_event *evt;
	unsigned long flags;
	moal_handle *handle = priv->phandle;

	evt = kzalloc(sizeof(struct woal_event), GFP_ATOMIC);
	if (!evt) {
		PRINTM(MERROR, "Fail to alloc memory for deauth event\n");
		LEAVE();
		return;
	}
	evt->priv = priv;
	evt->type = WOAL_EVENT_DEAUTH;
	evt->reason_code = reason_code;
	INIT_LIST_HEAD(&evt->link);
	spin_lock_irqsave(&handle->evt_lock, flags);
	list_add_tail(&evt->link, &handle->evt_queue);
	spin_unlock_irqrestore(&handle->evt_lock, flags);
	queue_work(handle->evt_workqueue, &handle->evt_work);
}

#ifdef STA_CFG80211
#if KERNEL_VERSION(3, 2, 0) <= CFG80211_VERSION_CODE
/**
 * @brief   prepare woal_bgscan_stop event
 *
 * @param priv          A pointer moal_private structure
 * @param pchan_info    A pointer to chan_band structure
 *
 * @return          N/A
 */
void woal_bgscan_stop_event(moal_private *priv)
{
	struct woal_event *evt;
	unsigned long flags;
	moal_handle *handle = priv->phandle;

	evt = kzalloc(sizeof(struct woal_event), GFP_ATOMIC);
	if (evt) {
		evt->priv = priv;
		evt->type = WOAL_EVENT_BGSCAN_STOP;
		INIT_LIST_HEAD(&evt->link);
		spin_lock_irqsave(&handle->evt_lock, flags);
		list_add_tail(&evt->link, &handle->evt_queue);
		spin_unlock_irqrestore(&handle->evt_lock, flags);
		queue_work(handle->evt_workqueue, &handle->evt_work);
	}
}

/**
 * @brief Notify cfg80211 schedule scan stopped
 *
 * @param priv          A pointer moal_private structure
 *
 * @return          N/A
 */
void woal_cfg80211_notify_sched_scan_stop(moal_private *priv)
{
	cfg80211_sched_scan_stopped(priv->wdev->wiphy
#if KERNEL_VERSION(4, 12, 0) <= CFG80211_VERSION_CODE
				    ,
				    0
#endif
	);
	priv->sched_scanning = MFALSE;
	PRINTM(MEVENT, "Sched_Scan stopped\n");
}
#endif
#endif

#if KERNEL_VERSION(3, 5, 0) <= CFG80211_VERSION_CODE
/**
 * @brief   Handle woal_channel_switch event
 *
 * @param priv          A pointer moal_private structure
 * @param pchan_info    A pointer to chan_band structure
 *
 * @return          N/A
 */
void woal_channel_switch_event(moal_private *priv, chan_band_info *pchan_info)
{
	struct woal_event *evt;
	unsigned long flags;
	moal_handle *handle = priv->phandle;

	evt = kzalloc(sizeof(struct woal_event), GFP_ATOMIC);
	if (evt) {
		evt->priv = priv;
		evt->type = WOAL_EVENT_CHAN_SWITCH;
		moal_memcpy_ext(priv->phandle, &evt->chan_info, pchan_info,
				sizeof(chan_band_info), sizeof(chan_band_info));
		INIT_LIST_HEAD(&evt->link);
		spin_lock_irqsave(&handle->evt_lock, flags);
		list_add_tail(&evt->link, &handle->evt_queue);
		spin_unlock_irqrestore(&handle->evt_lock, flags);
		queue_work(handle->evt_workqueue, &handle->evt_work);
	}
}

/**
 * @brief Notify cfg80211 supplicant channel changed
 *
 * @param priv          A pointer moal_private structure
 * @param pchan_info    A pointer to chan_band structure
 *
 * @return          N/A
 */
void woal_cfg80211_notify_channel(moal_private *priv,
				  chan_band_info *pchan_info)
{
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	struct cfg80211_chan_def chandef;
#else
#if KERNEL_VERSION(3, 5, 0) <= CFG80211_VERSION_CODE
	enum nl80211_channel_type type;
	enum ieee80211_band band;
	int freq = 0;
#endif
#endif
	ENTER();

#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
	if (MLAN_STATUS_SUCCESS ==
	    woal_chandef_create(priv, &chandef, pchan_info)) {
		cfg80211_ch_switch_notify(priv->netdev, &chandef);
		priv->channel = pchan_info->channel;
#ifdef UAP_CFG80211
		moal_memcpy_ext(priv->phandle, &priv->chan, &chandef,
				sizeof(struct cfg80211_chan_def),
				sizeof(struct cfg80211_chan_def));
#endif
	}
#else
#if KERNEL_VERSION(3, 5, 0) <= CFG80211_VERSION_CODE
	if (pchan_info->bandcfg.chanBand == BAND_2GHZ)
		band = IEEE80211_BAND_2GHZ;
	else if (pchan_info->bandcfg.chanBand == BAND_5GHZ)
		band = IEEE80211_BAND_5GHZ;
	else {
		LEAVE();
		return;
	}
	priv->channel = pchan_info->channel;
	freq = ieee80211_channel_to_frequency(pchan_info->channel, band);
	switch (pchan_info->bandcfg.chanWidth) {
	case CHAN_BW_20MHZ:
		if (pchan_info->is_11n_enabled)
			type = NL80211_CHAN_HT20;
		else
			type = NL80211_CHAN_NO_HT;
		break;
	default:
		if (pchan_info->bandcfg.chan2Offset == SEC_CHAN_ABOVE)
			type = NL80211_CHAN_HT40PLUS;
		else if (pchan_info->bandcfg.chan2Offset == SEC_CHAN_BELOW)
			type = NL80211_CHAN_HT40MINUS;
		else
			type = NL80211_CHAN_HT20;
		break;
	}
	cfg80211_ch_switch_notify(priv->netdev, freq, type);
#endif
#endif
	LEAVE();
}
#endif

#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
/**
 * @brief create cfg80211_chan_def structure based on chan_band info
 *
 * @param priv          A pointer moal_private structure
 * @param chandef       A pointer to cfg80211_chan_def structure
 * @param pchan_info    A pointer to chan_band_info structure
 *
 * @return              MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE
 */
mlan_status woal_chandef_create(moal_private *priv,
				struct cfg80211_chan_def *chandef,
				chan_band_info *pchan_info)
{
	enum ieee80211_band band = IEEE80211_BAND_2GHZ;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	memset(chandef, 0, sizeof(struct cfg80211_chan_def));
	chandef->center_freq2 = 0;
	if (pchan_info->bandcfg.chanBand == BAND_2GHZ)
		band = IEEE80211_BAND_2GHZ;
	else if (pchan_info->bandcfg.chanBand == BAND_5GHZ)
		band = IEEE80211_BAND_5GHZ;
	chandef->chan = ieee80211_get_channel(
		priv->wdev->wiphy,
		ieee80211_channel_to_frequency(pchan_info->channel, band));
	if (chandef->chan == NULL) {
		PRINTM(MERROR,
		       "Fail on ieee80211_get_channel, channel=%d, band=%d\n",
		       pchan_info->channel, band);
		status = MLAN_STATUS_FAILURE;
		goto done;
	}
	switch (pchan_info->bandcfg.chanWidth) {
	case CHAN_BW_20MHZ:
		if (pchan_info->is_11n_enabled)
			chandef->width = NL80211_CHAN_WIDTH_20;
		else
			chandef->width = NL80211_CHAN_WIDTH_20_NOHT;
		chandef->center_freq1 = chandef->chan->center_freq;
		break;
	case CHAN_BW_40MHZ:
		chandef->width = NL80211_CHAN_WIDTH_40;
		if (pchan_info->bandcfg.chan2Offset == SEC_CHAN_ABOVE)
			chandef->center_freq1 = chandef->chan->center_freq + 10;
		else if (pchan_info->bandcfg.chan2Offset == SEC_CHAN_BELOW)
			chandef->center_freq1 = chandef->chan->center_freq - 10;
		break;
	case CHAN_BW_80MHZ:
		chandef->width = NL80211_CHAN_WIDTH_80;
		chandef->center_freq1 = ieee80211_channel_to_frequency(
			pchan_info->center_chan, band);
		break;
	default:
		break;
	}
done:
	LEAVE();
	return status;
}
#endif
