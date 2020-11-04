/** @file moal_cfg80211.h
 *
 * @brief This file contains the CFG80211 specific defines.
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

#ifndef _MOAL_CFG80211_H_
#define _MOAL_CFG80211_H_

#include "moal_main.h"

#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
#define IEEE80211_CHAN_PASSIVE_SCAN IEEE80211_CHAN_NO_IR
#define IEEE80211_CHAN_NO_IBSS IEEE80211_CHAN_NO_IR
#endif

#if KERNEL_VERSION(3, 16, 0) <= CFG80211_VERSION_CODE
#define MAX_CSA_COUNTERS_NUM 2
#endif

/* Clear all key indexes */
#define KEY_INDEX_CLEAR_ALL (0x0000000F)

/** RTS/FRAG disabled value */
#define MLAN_FRAG_RTS_DISABLED (0xFFFFFFFF)

#ifndef WLAN_CIPHER_SUITE_SMS4
#define WLAN_CIPHER_SUITE_SMS4 0x00000020
#endif

#ifndef WLAN_CIPHER_SUITE_AES_CMAC
#define WLAN_CIPHER_SUITE_AES_CMAC 0x000FAC06
#endif
#if KERNEL_VERSION(4, 0, 0) <= CFG80211_VERSION_CODE
#ifndef WLAN_CIPHER_SUITE_BIP_GMAC_256
#define WLAN_CIPHER_SUITE_BIP_GMAC_256 0x000FAC0C
#endif
#endif

/* define for custom ie operation */
#define MLAN_CUSTOM_IE_AUTO_IDX_MASK 0xffff
#define MLAN_CUSTOM_IE_NEW_MASK 0x8000
#define IE_MASK_WPS 0x0001
#define IE_MASK_P2P 0x0002
#define IE_MASK_WFD 0x0004
#define IE_MASK_VENDOR 0x0008
#define IE_MASK_EXTCAP 0x0010

#define MRVL_PKT_TYPE_MGMT_FRAME 0xE5

/**
 * If multiple wiphys are registered e.g. a regular netdev with
 * assigned ieee80211_ptr and you won't know whether it points
 * to a wiphy your driver has registered or not. Assign this to
 * something global to your driver to help determine whether
 * you own this wiphy or not.
 */
static const void *const mrvl_wiphy_privid = &mrvl_wiphy_privid;

/* Get the private structure from wiphy */
void *woal_get_wiphy_priv(struct wiphy *wiphy);

/* Get the private structure from net device */
void *woal_get_netdev_priv(struct net_device *dev);
#ifdef STA_SUPPORT
/** get scan interface */
pmoal_private woal_get_scan_interface(pmoal_handle handle);
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
/** AUTH pending flag */
#define HOST_MLME_AUTH_PENDING MBIT(0)
/** AUTH complete flag */
#define HOST_MLME_AUTH_DONE MBIT(1)
#define HOST_MLME_ASSOC_PENDING MBIT(2)
#define HOST_MLME_ASSOC_DONE MBIT(3)
void woal_host_mlme_disconnect(pmoal_private priv, u16 reason_code, u8 *sa);
void woal_host_mlme_work_queue(struct work_struct *work);
void woal_host_mlme_process_assoc_resp(moal_private *priv,
				       mlan_ds_misc_assoc_rsp *assoc_rsp);
#endif
#endif

t_u8 woal_band_cfg_to_ieee_band(t_u32 band);

int woal_cfg80211_change_virtual_intf(struct wiphy *wiphy,
				      struct net_device *dev,
				      enum nl80211_iftype type,
#if KERNEL_VERSION(4, 12, 0) > CFG80211_VERSION_CODE
				      u32 *flags,
#endif
				      struct vif_params *params);

int woal_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed);

int woal_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
			  t_u8 key_index,
#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
			  bool pairwise,
#endif
			  const t_u8 *mac_addr, struct key_params *params);

int woal_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
			  t_u8 key_index,
#if KERNEL_VERSION(2, 6, 36) < CFG80211_VERSION_CODE
			  bool pairwise,
#endif
			  const t_u8 *mac_addr);
#ifdef STA_SUPPORT
/** Opportunistic Key Caching APIs support */
struct pmksa_entry *woal_get_pmksa_entry(pmoal_private priv, const u8 *bssid);

int woal_flush_pmksa_list(moal_private *priv);

int woal_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_pmksa *pmksa);

int woal_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
			    struct cfg80211_pmksa *pmksa);

int woal_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev);
#endif

int woal_cfg80211_set_bitrate_mask(struct wiphy *wiphy, struct net_device *dev,
				   const u8 *peer,
				   const struct cfg80211_bitrate_mask *mask);
#if KERNEL_VERSION(2, 6, 38) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_antenna(struct wiphy *wiphy, u32 tx_ant, u32 rx_ant);
int woal_cfg80211_get_antenna(struct wiphy *wiphy, u32 *tx_ant, u32 *rx_ant);
#endif

#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_qos_map(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_qos_map *qos_map);
#endif

#ifdef STA_CFG80211
#ifdef STA_SUPPORT
int woal_set_rf_channel(moal_private *priv, struct ieee80211_channel *chan,
			enum nl80211_channel_type channel_type,
			t_u8 wait_option);

static inline int woal_cfg80211_scan_done(struct cfg80211_scan_request *request,
					  bool aborted)
{
#if KERNEL_VERSION(4, 8, 0) <= CFG80211_VERSION_CODE
	struct cfg80211_scan_info info;

	info.aborted = aborted;
	cfg80211_scan_done(request, &info);
#else
	cfg80211_scan_done(request, aborted);
#endif
	return 0;
}
mlan_status woal_inform_bss_from_scan_result(moal_private *priv,
					     pmlan_ssid_bssid ssid_bssid,
					     t_u8 wait_option);
#endif
#endif

#if KERNEL_VERSION(3, 5, 0) > CFG80211_VERSION_CODE
int woal_cfg80211_set_channel(struct wiphy *wiphy,
#if KERNEL_VERSION(2, 6, 34) < CFG80211_VERSION_CODE
			      struct net_device *dev,
#endif
			      struct ieee80211_channel *chan,
			      enum nl80211_channel_type channel_type);
#endif

#if KERNEL_VERSION(2, 6, 37) < CFG80211_VERSION_CODE
int woal_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *dev,
				  t_u8 key_index, bool ucast, bool mcast);
#else
int woal_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *dev,
				  t_u8 key_index);
#endif

#if KERNEL_VERSION(2, 6, 30) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_default_mgmt_key(struct wiphy *wiphy,
				       struct net_device *netdev,
				       t_u8 key_index);
#endif

#if KERNEL_VERSION(3, 1, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_rekey_data(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_gtk_rekey_data *data);
#endif
void woal_mgmt_frame_register(moal_private *priv, u16 frame_type, bool reg);
void woal_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
				       struct wireless_dev *wdev,
#else
				       struct net_device *dev,
#endif
#if KERNEL_VERSION(5, 8, 0) <= CFG80211_VERSION_CODE
				       struct mgmt_frame_regs *upd
#else
				       t_u16 frame_type, bool reg
#endif
);

int woal_cfg80211_mgmt_tx(struct wiphy *wiphy,
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
			  struct wireless_dev *wdev,
#else
			  struct net_device *dev,
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
			  u64 *cookie);

#if KERNEL_VERSION(3, 14, 0) <= CFG80211_VERSION_CODE
void woal_update_radar_chans_dfs_state(struct wiphy *wiphy);
#endif

mlan_status woal_register_cfg80211(moal_private *priv);

extern struct ieee80211_supported_band cfg80211_band_2ghz;
extern struct ieee80211_supported_band cfg80211_band_5ghz;
extern struct ieee80211_supported_band mac1_cfg80211_band_2ghz;
extern struct ieee80211_supported_band mac1_cfg80211_band_5ghz;

#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
int woal_cfg80211_bss_role_cfg(moal_private *priv, t_u16 action,
			       t_u8 *bss_role);
#endif
#if KERNEL_VERSION(4, 1, 0) <= CFG80211_VERSION_CODE
struct wireless_dev *
woal_cfg80211_add_virtual_intf(struct wiphy *wiphy, const char *name,
			       unsigned char name_assign_type,
			       enum nl80211_iftype type,
#if KERNEL_VERSION(4, 12, 0) > CFG80211_VERSION_CODE
			       u32 *flags,
#endif
			       struct vif_params *params);
#else
#if KERNEL_VERSION(3, 7, 0) <= CFG80211_VERSION_CODE
struct wireless_dev *woal_cfg80211_add_virtual_intf(struct wiphy *wiphy,
						    const char *name,
						    enum nl80211_iftype type,
						    u32 *flags,
						    struct vif_params *params);
#else
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
struct wireless_dev *woal_cfg80211_add_virtual_intf(struct wiphy *wiphy,
						    char *name,
						    enum nl80211_iftype type,
						    u32 *flags,
						    struct vif_params *params);
#else
#if KERNEL_VERSION(2, 6, 37) < CFG80211_VERSION_CODE
struct net_device *woal_cfg80211_add_virtual_intf(struct wiphy *wiphy,
						  char *name,
						  enum nl80211_iftype type,
						  u32 *flags,
						  struct vif_params *params);
#else
int woal_cfg80211_add_virtual_intf(struct wiphy *wiphy, char *name,
				   enum nl80211_iftype type, u32 *flags,
				   struct vif_params *params);
#endif
#endif
#endif
#endif
int woal_cfg80211_del_virt_if(struct wiphy *wiphy, struct net_device *dev);
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_del_virtual_intf(struct wiphy *wiphy,
				   struct wireless_dev *wdev);
#else
int woal_cfg80211_del_virtual_intf(struct wiphy *wiphy, struct net_device *dev);
#endif

#ifdef WIFI_DIRECT_SUPPORT
/* Group Owner Negotiation Req */
#define P2P_GO_NEG_REQ 0
/* Group Owner Negotiation Rsp */
#define P2P_GO_NEG_RSP 1
/* Group Owner Negotiation Confirm */
#define P2P_GO_NEG_CONF 2
/* P2P Invitation Request */
#define P2P_INVITE_REQ 3
/* P2P Invitation Response */
#define P2P_INVITE_RSP 4
/* Device Discoverability Request */
#define P2P_DEVDIS_REQ 5
/* Device Discoverability Response */
#define P2P_DEVDIS_RSP 6
/* Provision Discovery Request */
#define P2P_PROVDIS_REQ 7
/* Provision Discovery Response */
#define P2P_PROVDIS_RSP 8
/** P2P category */
#define P2P_ACT_FRAME_CATEGORY 0x04
/** P2P oui offset */
#define P2P_ACT_FRAME_OUI_OFFSET 26
/** P2P subtype offset */
#define P2P_ACT_FRAME_OUI_SUBTYPE_OFFSET 30
void woal_cfg80211_display_p2p_actframe(const t_u8 *buf, int len,
					struct ieee80211_channel *chan,
					const t_u8 flag);

/** Define kernel version for wifi direct */
#define WIFI_DIRECT_KERNEL_VERSION KERNEL_VERSION(2, 6, 39)

#if CFG80211_VERSION_CODE >= WIFI_DIRECT_KERNEL_VERSION

int woal_cfg80211_init_p2p_client(moal_private *priv);

int woal_cfg80211_init_p2p_go(moal_private *priv);

int woal_cfg80211_deinit_p2p(moal_private *priv);

void woal_remove_virtual_interface(moal_handle *handle);

#endif /* KERNEL_VERSION */
#endif /* WIFI_DIRECT_SUPPORT */

/** Define for remain on channel duration timer */
#define MAX_REMAIN_ON_CHANNEL_DURATION (1000)

int woal_cfg80211_remain_on_channel_cfg(moal_private *priv, t_u8 wait_option,
					t_u8 remove, t_u8 *status,
					struct ieee80211_channel *chan,
					enum mlan_channel_type channel_type,
					t_u32 duration);

#ifdef UAP_CFG80211
int woal_uap_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
#if KERNEL_VERSION(3, 16, 0) <= CFG80211_VERSION_CODE
				  const u8 *mac,
#else
				  u8 *mac,
#endif
				  struct station_info *stainfo);

int woal_uap_cfg80211_dump_station(struct wiphy *wiphy, struct net_device *dev,
				   int idx, t_u8 *mac,
				   struct station_info *sinfo);
#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_change_bss(struct wiphy *wiphy, struct net_device *dev,
			     struct bss_parameters *params);
#endif
#if KERNEL_VERSION(3, 9, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_mac_acl(struct wiphy *wiphy, struct net_device *dev,
			      const struct cfg80211_acl_data *params);
#endif
#if KERNEL_VERSION(3, 1, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_txq_params(struct wiphy *wiphy, struct net_device *dev,
				 struct ieee80211_txq_params *params);
#endif

#if KERNEL_VERSION(3, 12, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_set_coalesce(struct wiphy *wiphy,
			       struct cfg80211_coalesce *coalesce);
#endif

#if KERNEL_VERSION(3, 4, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_add_beacon(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_ap_settings *params);

int woal_cfg80211_set_beacon(struct wiphy *wiphy, struct net_device *dev,
			     struct cfg80211_beacon_data *params);
#else
int woal_cfg80211_add_beacon(struct wiphy *wiphy, struct net_device *dev,
			     struct beacon_parameters *params);

int woal_cfg80211_set_beacon(struct wiphy *wiphy, struct net_device *dev,
			     struct beacon_parameters *params);
#endif

int woal_cfg80211_del_beacon(struct wiphy *wiphy, struct net_device *dev);
int woal_cfg80211_del_station(struct wiphy *wiphy, struct net_device *dev,
#if KERNEL_VERSION(3, 19, 0) <= CFG80211_VERSION_CODE
			      struct station_del_parameters *param);
#else
#if KERNEL_VERSION(3, 16, 0) <= CFG80211_VERSION_CODE
			      const u8 *mac_addr);
#else
			      u8 *mac_addr);
#endif
#endif
#if KERNEL_VERSION(3, 12, 0) <= CFG80211_VERSION_CODE
#if KERNEL_VERSION(3, 15, 0) <= CFG80211_VERSION_CODE
int woal_cfg80211_start_radar_detection(struct wiphy *wiphy,
					struct net_device *dev,
					struct cfg80211_chan_def *chandef,
					u32 cac_time_ms);
#else
int woal_cfg80211_start_radar_detection(struct wiphy *wiphy,
					struct net_device *dev,
					struct cfg80211_chan_def *chandef);
#endif

int woal_cfg80211_channel_switch(struct wiphy *wiphy, struct net_device *dev,
				 struct cfg80211_csa_settings *params);

void woal_cac_timer_func(void *context);
void woal_csa_work_queue(struct work_struct *work);
#endif
#endif /* UAP_CFG80211 */
#if defined(UAP_CFG80211) || defined(STA_CFG80211)
#if KERNEL_VERSION(3, 5, 0) <= CFG80211_VERSION_CODE
void woal_cfg80211_notify_channel(moal_private *priv,
				  pchan_band_info pchan_info);
void woal_channel_switch_event(moal_private *priv, chan_band_info *pchan_info);
#endif
#endif

#ifdef STA_CFG80211
#if KERNEL_VERSION(3, 2, 0) <= CFG80211_VERSION_CODE
void woal_bgscan_stop_event(moal_private *priv);
void woal_cfg80211_notify_sched_scan_stop(moal_private *priv);
#endif
#endif

void woal_deauth_event(moal_private *priv, int reason_code);

#if KERNEL_VERSION(3, 8, 0) <= CFG80211_VERSION_CODE
mlan_status woal_chandef_create(moal_private *priv,
				struct cfg80211_chan_def *chandef,
				chan_band_info *pchan_info);
#endif

#if KERNEL_VERSION(4, 20, 0) <= CFG80211_VERSION_CODE
void woal_cfg80211_setup_he_cap(moal_private *priv,
				struct ieee80211_supported_band *band);
void woal_cfg80211_free_iftype_data(struct wiphy *wiphy);
#endif

void woal_clear_all_mgmt_ies(moal_private *priv, t_u8 wait_option);
int woal_cfg80211_mgmt_frame_ie(
	moal_private *priv, const t_u8 *beacon_ies, size_t beacon_ies_len,
	const t_u8 *proberesp_ies, size_t proberesp_ies_len,
	const t_u8 *assocresp_ies, size_t assocresp_ies_len,
	const t_u8 *probereq_ies, size_t probereq_ies_len, t_u16 mask,
	t_u8 wait_option);

int woal_get_active_intf_freq(moal_private *priv);

void woal_cfg80211_setup_ht_cap(struct ieee80211_sta_ht_cap *ht_info,
				t_u32 dev_cap, t_u8 *mcs_set);
#if KERNEL_VERSION(3, 6, 0) <= CFG80211_VERSION_CODE
void woal_cfg80211_setup_vht_cap(moal_private *priv,
				 struct ieee80211_sta_vht_cap *vht_cap);
#endif
int woal_cfg80211_assoc(moal_private *priv, void *sme, t_u8 wait_option,
			pmlan_ds_misc_assoc_rsp assoc_rsp);

#endif /* _MOAL_CFG80211_H_ */
