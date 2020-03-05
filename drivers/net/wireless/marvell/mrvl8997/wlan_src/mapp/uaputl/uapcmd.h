/** @file  uapcmd.h
  *
  * @brief This file contains declaration referring to
  * functions defined in uapcmd.c
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
    03/01/08: Initial creation
************************************************************************/

#ifndef _UAPCMD_H
#define _UAPCMD_H

/** Function Prototype Declaration */
int apcmd_sys_cfg_ap_mac_address(int argc, char *argv[]);
int apcmd_sys_cfg_ssid(int argc, char *argv[]);
int apcmd_sys_cfg_beacon_period(int argc, char *argv[]);
int apcmd_sys_cfg_dtim_period(int argc, char *argv[]);
int apcmd_sys_cfg_bss_status(int argc, char *argv[]);
int apcmd_sys_cfg_channel(int argc, char *argv[]);
int apcmd_sys_cfg_channel_ext(int argc, char *argv[]);
int apcmd_sys_cfg_scan_channels(int argc, char *argv[]);
int apcmd_sys_cfg_rates(int argc, char *argv[]);
int apcmd_sys_cfg_rates_ext(int argc, char *argv[]);
int apcmd_sys_cfg_tx_power(int argc, char *argv[]);
int apcmd_sys_cfg_bcast_ssid_ctl(int argc, char *argv[]);
int apcmd_sys_cfg_preamble_ctl(int argc, char *argv[]);
int apcmd_sys_cfg_rts_threshold(int argc, char *argv[]);
int apcmd_sys_cfg_frag_threshold(int argc, char *argv[]);
int apcmd_sys_cfg_radio_ctl(int argc, char *argv[]);
int apcmd_sys_cfg_rsn_replay_prot(int argc, char *argv[]);
int apcmd_sys_cfg_tx_beacon_rate(int argc, char *argv[]);
int apcmd_sys_cfg_mcbc_data_rate(int argc, char *argv[]);
int apcmd_sys_cfg_pkt_fwd_ctl(int argc, char *argv[]);
int apcmd_sys_cfg_sta_ageout_timer(int argc, char *argv[]);
int apcmd_sys_cfg_ps_sta_ageout_timer(int argc, char *argv[]);
int apcmd_sys_cfg_auth(int argc, char *argv[]);
int apcmd_sys_cfg_protocol(int argc, char *argv[]);
int apcmd_sys_cfg_wep_key(int argc, char *argv[]);
int apcmd_sys_cfg_cipher(int argc, char *argv[]);
int apcmd_sys_cfg_pwk_cipher(int argc, char *argv[]);
int apcmd_sys_cfg_gwk_cipher(int argc, char *argv[]);
int apcmd_sys_cfg_wpa_passphrase(int argc, char *argv[]);
int apcmd_sys_cfg_group_rekey_timer(int argc, char *argv[]);
int apcmd_sta_filter_table(int argc, char *argv[]);
int apcmd_sys_cfg_max_sta_num(int argc, char *argv[]);
int apcmd_sys_cfg_retry_limit(int argc, char *argv[]);
int apcmd_sys_cfg_sticky_tim_config(int argc, char *argv[]);
int apcmd_sys_cfg_sticky_tim_sta_mac_addr(int argc, char *argv[]);
int apcmd_sys_cfg_eapol_pwk_hsk(int argc, char *argv[]);
int apcmd_sys_cfg_eapol_gwk_hsk(int argc, char *argv[]);
int apcmd_cfg_data(int argc, char *argv[]);
int apcmd_sys_cfg_custom_ie(int argc, char *argv[]);
int apcmd_sys_cfg_wmm(int argc, char *argv[]);
int apcmd_sys_cfg_ap_wmm(int argc, char *argv[]);
int apcmd_sys_cfg_11n(int argc, char *argv[]);
#ifdef RX_PACKET_COALESCE
int apcmd_rx_pkt_coalesce(int argc, char *argv[]);
void print_rx_packet_coalesc_help(void);
#endif
int apcmd_sys_cfg_2040_coex(int argc, char *argv[]);
int apcmd_sys_cfg_restrict_client_mode(int argc, char *argv[]);
#endif /* _UAP_H */
