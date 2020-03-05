/** @file  uapcmd.c
  *
  * @brief This file contains the handling of command.
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
/****************************************************************************
Change log:
    03/01/08: Initial creation
****************************************************************************/

/****************************************************************************
        Header files
****************************************************************************/
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <stdio.h>
#include <getopt.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "uaputl.h"
#include "uapcmd.h"

extern struct option cmd_options[];
/****************************************************************************
        Local functions
****************************************************************************/
/**
 *  @brief Show usage information for the sys_cfg_ap_mac_address
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_ap_mac_address_usage(void)
{
	printf("\nUsage : sys_cfg_ap_mac_address [AP_MAC_ADDRESS]\n");
	printf("\nIf AP_MAC_ADDRESS is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_ssid command
 *
 *  $return         N/A
 */
void
print_sys_cfg_ssid_usage(void)
{
	printf("\nUsage : sys_cfg_ssid [SSID]\n");
	printf("\nIf SSID is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_beacon_period
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_beacon_period_usage(void)
{
	printf("\nUsage : sys_cfg_beacon_period [BEACON_PERIOD]\n");
	printf("\nIf BEACON_PERIOD is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_dtim_period
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_dtim_period_usage(void)
{
	printf("\nUsage : sys_cfg_dtim_period [DTIM_PERIOD]\n");
	printf("\nIf DTIM_PERIOD is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the bss_status
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_bss_status_usage(void)
{
	printf("\nUsage : sys_cfg_bss_status\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_channel
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_channel_usage(void)
{
	printf("\nUsage : sys_cfg_channel [CHANNEL] [MODE]\n");
	printf("\nIf CHANNEL is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\n MODE:  band config mode ");
	printf("\n        Bit 0: ACS mode enable/disable");
	printf("\n        Bit 1: secondary channel is above primary channel");
	printf("\n        Bit 2: secondary channel is below primary channel");
	printf("\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_channel
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_channel_ext_usage(void)
{
	printf("\nUsage : sys_cfg_channel_ext [CHANNEL] [BAND] [MODE]\n");
	printf("\nIf CHANNEL is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\n BAND:");
	printf("\n        0 : 2.4GHz operation");
	printf("\n        1 : 5GHz operation");
	printf("\n MODE:  band config mode ");
	printf("\n        Bit 0: ACS mode enable/disable");
	printf("\n        Bit 1: secondary channel is above primary channel");
	printf("\n        Bit 2: secondary channel is below primary channel");
	printf("\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_scan_channels
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_scan_channels_usage(void)
{
	printf("\nUsage : sys_cfg_scan_channels [CHANNEL[.BAND]]\n");
	printf("\nIf CHANNELS and BANDS are provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("\n BAND : band of operation");
	printf("\n        0 : 2.4GHZ");
	printf("\n        1 : 5GHZ\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_rates_ext command
 *
 *  $return         N/A
 */
void
print_sys_cfg_rates_ext_usage(void)
{
	printf("\nUsage : sys_cfg_rates_ext [rates RATES] [mbrate RATE]\n");
	printf("\nIf 'Rate' provided, a 'set' is performed else a 'get' is performed");
	printf("\nRATES is provided as a set of data rates, in unit of 500 kilobits");
	printf("\nA rate with MSB bit is basic rate, i.e 0x82 is basic rate.\n");
	printf("\nFollowing is the list of supported rates in units of 500 Kbps:");
	printf("\nDecimal: (2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108)");
	printf("\nHex: (0x02, 0x04, 0x0b, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c)");
	printf("\nBasic rates: (0x82, 0x84, 0x8b, 0x96, 0x8C, 0x92, 0x98, 0xA4, 0xB0, 0xC8, 0xE0, 0xEc)\n");
	printf("\nRates 2, 4, 11 and 22 (in units of 500 Kbps) must be present in either of basic or");
	printf("\nnon-basic rates. If OFDM rates are enabled then 12, 24 and 48 (in units of 500 Kbps)");
	printf("\nmust be present in either basic or non-basic rates");
	printf("\nEach rate must be separated by a space.");
	printf("\nrates followed by RATES for setting operational rates.");
	printf("\nmbrate followed by RATE for setting multicast and broadcast rate.");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_rates command
 *
 *  $return         N/A
 */
void
print_sys_cfg_rates_usage(void)
{
	printf("\nUsage : sys_cfg_rates [RATES]\n");
	printf("\n[RATES] is set of data rates in unit of 500 kbps and each rate can be");
	printf("\nentered in hexadecimal or decimal format. Rates must be separated by");
	printf("\nspace. Duplicate Rate  fields are not allowed");
	printf("\nA rate with MSB bit is basic rate, i.e 0x82 is basic rate.");
	printf("\nFollowing is the list of supported rates in units of 500 Kbps:");
	printf("\nDecimal: (2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108)");
	printf("\nHex: (0x02, 0x04, 0x0b, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c)");
	printf("\nBasic rates: (0x82, 0x84, 0x8b, 0x96, 0x8C, 0x92, 0x98, 0xA4, 0xB0, 0xC8, 0xE0, 0xEc)\n");
	return;
}

/**
 *   @brief Show usage information for the sys_cfg_tx_power
 *    command
 *
 *  $return         N/A
 */
void
print_sys_cfg_tx_power_usage(void)
{
	printf("\nUsage : sys_cfg_tx_power [TX_POWER]\n");
	printf("\nIf TX_POWER is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\nTX_POWER is represented in dBm.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_bcast_ssid_ctl
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_bcast_ssid_ctl_usage(void)
{
	printf("\nUsage : sys_cfg_bcast_ssid_ctl [0|1]\n");
	printf("\nOptions: 0     - send empty SSID (length=0) in beacon");
	printf("\n         1     - Enable SSID broadcast");
	printf("\n         2     - clear SSID (ACSII 0) in beacon, but keep the original length");
	printf("\n         empty - Get current SSID broadcast setting\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_rsn_replay_prot
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_rsn_replay_prot_usage(void)
{
	printf("\nUsage : sys_cfg_rsn_replay_prot [0|1]\n");
	printf("\nOptions: 0     - Disable RSN replay protection");
	printf("\n         1     - Enable  RSN replay protection");
	printf("\n         empty - Get current RSN replay protection setting\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_preamble_ctl
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_preamble_ctl_usage(void)
{
	printf("\nUsage : sys_cfg_preamble_ctl\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_antenna_ctl
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_antenna_ctl_usage(void)
{
	printf("\nUsage : sys_cfg_antenna_ctl <ANTENNA> [MODE]\n");
	printf("\nOptions: ANTENNA : 0 - Rx antenna");
	printf("\n                   1 - Tx antenna");
	printf("\n         MODE    : 0       - Antenna A");
	printf("\n                   1       - Antenna B");
	printf("\n                   empty   - Get current antenna settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_rts_threshold
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_rts_threshold_usage(void)
{
	printf("\nUsage : sys_cfg_rts_threshold [RTS_THRESHOLD]\n");
	printf("\nIf RTS_THRESHOLD is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_frag_threshold
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_frag_threshold_usage(void)
{
	printf("\nUsage : sys_cfg_frag_threshold [FRAG_THRESHOLD]\n");
	printf("\nIf FRAG_THRESHOLD is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\nFragment threshold should between 256 and 2346.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_tx_beacon_rate
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_tx_beacon_rates_usage(void)
{
	printf("\nUsage : sys_cfg_tx_beacon_rate [TX_DATA_RATE]\n");
	printf("\nOptions: 0     - Auto rate");
	printf("\n         >0    - Set specified beacon rate");
	printf("\n         empty - Get current beacon rate");
	printf("\nFollowing is the list of supported rates in units of 500 Kbps");
	printf("\nDecimal: (2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108)");
	printf("\nHex: (0x02, 0x04, 0x0b, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c)");
	printf("\nOnly zero or rates currently configured are allowed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_mcbc_data_rate
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_mcbc_data_rates_usage(void)
{
	printf("\nUsage : sys_cfg_mcbc_data_rate [MCBC_DATA_RATE]\n");
	printf("\nOptions: 0     - Auto rate");
	printf("\n         >0    - Set specified MCBC data rate");
	printf("\n         empty - Get current MCBC data rate");
	printf("\nFollowing is the list of supported rates in units of 500 Kbps");
	printf("\nDecimal: (2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108)");
	printf("\nHex: (0x02, 0x04, 0x0b, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c)");
	printf("\nOnly zero or one of the basic rates currently configured are allowed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_auth command
 *
 *  $return         N/A
 */
void
print_sys_cfg_auth_usage(void)
{
	printf("\nUsage : sys_cfg_auth [AUTHMODE]\n");
	printf("\nOptions: AUTHMODE :     0 - Open authentication");
	printf("\n                        1 - Shared key authentication");
	printf("\n                        255 - Auto (Open and Shared key) authentication");
	printf("\n         empty - Get current authenticaton mode\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_pkt_fwd_ctl command
 *
 *  $return         N/A
 */
void
print_sys_cfg_pkt_fwd_ctl_usage(void)
{
	printf("\nUsage : sys_cfg_pkt_fwd_ctl [PKT_FWD_CTRL]\n");
	printf("\nPKT_FWD_CTRL: bit 0 -- Packet forwarding handled by Host (0) or Firmware (1)");
	printf("\n         bit 1 -- Intra-BSS broadcast packets are allowed (0) or denied (1)");
	printf("\n         bit 2 -- Intra-BSS unicast packets are allowed (0) or denied (1)");
	printf("\n         bit 3 -- Inter-BSS unicast packets are allowed (0) or denied (1)");
	printf("\n         empty - Get current packet forwarding setting\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_sta_ageout_timer
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_sta_ageout_timer_usage(void)
{
	printf("\nUsage : sys_cfg_sta_ageout_timer [STA_AGEOUT_TIMER]\n");
	printf("\nIf STA_AGEOUT_TIMER is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\nSTA_AGEOUT_TIMER is represented in units of 100 ms.");
	printf("\nValue of 0 will mean that stations will never be aged out.");
	printf("\nThe value should be between 100 and 864000.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_ps_sta_ageout_timer
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_ps_sta_ageout_timer_usage(void)
{
	printf("\nUsage : sys_cfg_ps_sta_ageout_timer [PS_STA_AGEOUT_TIMER]\n");
	printf("\nIf PS_STA_AGEOUT_TIMER is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\nPS_STA_AGEOUT_TIMER is represented in units of 100 ms.");
	printf("\nValue of 0 will mean that stations will never be aged out.");
	printf("\nThe value should be between 100 and 864000.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_protocol command
 *
 *  $return         N/A
 */
void
print_sys_cfg_protocol_usage(void)
{
	printf("\nUsage : sys_cfg_protocol [PROTOCOL] [AKM_SUITE]\n");
	printf("\nOptions: PROTOCOL:        1 - No RSN");
	printf("\n              2 - WEP Static");
	printf("\n              8 - WPA");
	printf("\n              32 - WPA2");
	printf("\n              40 - WPA2 Mixed");
#ifdef EMBDEDDED_OWE_SUPPORT
	printf("\n              512 - OWE");
#endif
	printf("\n              empty - Get current protocol");
	printf("\n          AKM_SUITE: bit 0 - KEY_MGMT_EAP");
	printf("\n              bit 1 - KEY_MGMT_PSK");
	printf("\n              bit 2 - KEY_MGMT_NONE");
	printf("\n              bit 8 - KEY_MGMT_PSK_SHA256\n");
#ifdef EMBDEDDED_OWE_SUPPORT
	printf("\n              bit 9 - KEY_MGMT_OWE\n");
#endif
	printf("\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_wep_key
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_wep_key_usage(void)
{
	printf("\nUsage : sys_cfg_wep_key ");
	printf("[INDEX_0 IS_DEFAULT KEY_0] [INDEX_1 IS_DEFAULT KEY_1] [INDEX_2 IS_DEFAULT KEY_2] [INDEX_3 IS_DEFAULT KEY_3]\n");
	printf("[Index_0] [Index_1] [Index_2] [Index_3]\n");
	printf("\nOptions: INDEX_* :      0 - KeyIndex is 0");
	printf("\n                        1 - KeyIndex is 1");
	printf("\n                        2 - KeyIndex is 2");
	printf("\n                        3 - KeyIndex is 3");
	printf("\n         IS_DEFAULT :   0 - KeyIndex is not the default");
	printf("\n                        1 - KeyIndex is the default transmit key");
	printf("\n         KEY_* :        Key value");
	printf("\n         Index_*:       0 - Get key 0 setting");
	printf("\n                        1 - Get key 1 setting");
	printf("\n                        2 - Get key 2 setting");
	printf("\n                        3 - Get key 3 setting");
	printf("\n         empty - Get current WEP key settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_cipher
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_cipher_usage(void)
{
	printf("\nUsage : sys_cfg_cipher [PAIRWISE_CIPHER GROUP_CIPHER]\n");
	printf("\nOptions: PAIRWISE_CIPHER:  0 - NONE");
	printf("\n                           4 - TKIP");
	printf("\n                           8 - AES CCMP");
	printf("\n                           12 - AES CCMP + TKIP");
	printf("\n         GROUP_CIPHER :    0 - NONE");
	printf("\n                           4 - TKIP");
	printf("\n                           8 - AES CCMP");
	printf("\n         empty - Get current cipher settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_pwk_cipher
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_pwk_cipher_usage(void)
{
	printf("\nUsage : sys_cfg_pwk_cipher [<PROTOCOL>] [PAIRWISE_CIPHER]\n");
	printf("\nOptions: PROTOCOL :");
	printf("\n                   8 - WPA");
	printf("\n                   32 - WPA2");
	printf("\n         PAIRWISE_CIPHER : ");
	printf("\n                          4 - TKIP");
	printf("\n                          8 - AES CCMP");
	printf("\n                          12 - AES CCMP + TKIP");
	printf("\n         WPA/TKIP cannot be used when uAP operates in 802.11n mode.\n");
	printf("\n         If only PROTOCOL is given, pairwise cipher for that protocol is displayed\n");
	printf("\n         empty - Get current protocol and pairwise cipher settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_gwk_cipher
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_gwk_cipher_usage(void)
{
	printf("\nUsage : sys_cfg_gwk_cipher [GROUP_CIPHER]\n");
	printf("\n         GROUP_CIPHER :");
	printf("\n                       4 - TKIP");
	printf("\n                       8 - AES CCMP");
	printf("\n         empty - Get current group cipher settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_group_rekey_timer command
 *
 *  $return         N/A
 */
void
print_sys_cfg_group_rekey_timer_usage(void)
{
	printf("\nUsage : sys_cfg_group_rekey_timer [GROUP_REKEY_TIMER]\n");
	printf("\nOptions: GROUP_REKEY_TIME is represented in seconds");
	printf("\n         empty - Get current group re-key time\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_wpa_passphrase
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_wpa_passphrase_usage(void)
{
	printf("\nUsage : sys_cfg_wpa_passphrase [PASSPHRASE]\n");
	printf("\nIf PASSPHRASE is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sta_filter_table command
 *
 *  $return         N/A
 */
void
print_sta_filter_table_usage(void)
{
	printf("\nUsage : sta_filter_table <FILTERMODE> <MACADDRESS_LIST>\n");
	printf("\nOptions: FILTERMODE : 0 - Disable filter table");
	printf("\n                      1 - allow MAC addresses specified in the allowed list");
	printf("\n                      2 - block MAC addresses specified in the banned list");
	printf("\n         MACADDRESS_LIST is the list of MAC addresses to be acted upon. Each");
	printf("\n                      MAC address must be separated with a space. Maximum of");
	printf("\n                      16 MAC addresses are supported.");
	printf("\n         empty - Get current mac filter settings\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_max_sta_num command
 *
 *  $return         N/A
 */
void
print_sys_cfg_max_sta_num_usage(void)
{
	printf("\nUsage : sys_cfg_max_sta_num [STA_NUM]\n");
	printf("\nIf STA_NUM is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_retry_limit command
 *
 *  $return         N/A
 */
void
print_sys_cfg_retry_limit_usage(void)
{
	printf("\nUsage : sys_cfg_retry_limit [RETRY_LIMIT]\n");
	printf("\nIf RETRY_LIMIT is provided, a 'set' is performed, else a 'get' is performed.");
	printf("\nRETRY_LIMIT should be greater than or equal to zero and less than or equal to 14\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_sticky_tim_config command
 *
 *  $return         N/A
 */
void
print_sys_cfg_sticky_tim_config_usage(void)
{
	printf("\nUsage : sys_cfg_sticky_tim_config [ENABLE] [<DURATION> <STICKY_BITMASK>]\n");
	printf("\nOptions: ENABLE  0 - disable");
	printf("\n                 1 - enable");
	printf("\n                 2 - enable with previous values of DURATION and STICKY_BITMASK");
	printf("\n         DURATION: duration for sticky TIM");
	printf("\n         STICKY_BITMASK: Bitmask for sticky TIM configuartion");
	printf("\n         empty - Get current sticky TIM configuration\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_sticky_tim_mac_addr command
 *
 *  $return         N/A
 */
void
print_sys_cfg_sticky_tim_sta_mac_addr_usage(void)
{
	printf("\nUsage : sys_cfg_sticky_tim_sta_mac_addr [CONTROL] [STA_MAC_ADDRESS]\n");
	printf("\nOptions: CONTROL: sticky TIM config for given station");
	printf("\n         STA_MAC_ADDRESS: station MAC address");
	printf("\n         if only STA_MAC_ADDRESS is given - Get sticky TIM configuration for that station\n");
	printf("\n         empty - Get current sticky TIM configuration for all associated stations\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_2040_coex command
 *
 *  $return         N/A
 */
void
print_sys_cfg_2040_coex_usage(void)
{
	printf("\nUsage : sys_cfg_2040_coex [ENABLE]\n");
	printf("\nOptions: ENABLE  0 - disable");
	printf("\n                 1 - enable");
	printf("\n         empty - Get current 20/40 BSS coexistence configuration\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_eapol_pwk_hsk command
 *
 *  $return         N/A
 */
void
print_sys_cfg_eapol_pwk_hsk_usage(void)
{
	printf("\nUsage : sys_cfg_eapol_pwk_hsk [<TIMEOUT> <RETRIES>]\n");
	printf("\nIf TIMEOUT and number of RETRIES are both provided, a 'set' is performed.");
	printf("\nIf no parameter is provided,a 'get' is performed.");
	printf("\nTIMEOUT and number of RETRIES should be greater than or equal to zero\n");
	return;
}

/**
 *  @brief Show usage information for the sys_cfg_eapol_gwk_hsk command
 *
 *  $return         N/A
 */
void
print_sys_cfg_eapol_gwk_hsk_usage(void)
{
	printf("\nUsage : sys_cfg_eapol_gwk_hsk [<TIMEOUT> <RETRIES>]\n");
	printf("\nIf TIMEOUT and number of RETRIES are both provided, a 'set' is performed.");
	printf("\nIf no parameter is provided,a 'get' is performed.");
	printf("\nTIMEOUT and number of RETRIES should be greater than or equal to zero\n");
	return;
}

/**
 *  @brief Show usage information for the cfg_data command
 *
 *  $return         N/A
 */
void
print_cfg_data_usage(void)
{
	printf("\nUsage : cfg_data <type> [*.conf]\n");
	printf("\n        type : 2 -- cal data");
	printf("\n        *.conf : file contain configuration data");
	printf("\n                 empty - get current configuration data\n");
	return;
}

/**
 *  @brief  Get configured operational rates.
 *
 *  @param  rates   Operational rates allowed are
 *                  stored at this pointer
 *  @return         Number of basic rates allowed.
 *                  -1 if a failure
 */
int
get_sys_cfg_rates(t_u8 *rates)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_rates *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len;
	int ret = UAP_FAILURE;
	int i = 0;
	int rate_cnt = 0;
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_rates) +
		MAX_DATA_RATES;
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(cmd_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return -1;
	}
	memset(buffer, 0, cmd_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_rates *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RATES_TLV_ID;
	cmd_buf->action = ACTION_GET;
	tlv->length = MAX_DATA_RATES;

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_RATES_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return -1;
		}

		/* Copy response */
		if (cmd_buf->result == CMD_SUCCESS) {
			for (i = 0; i < tlv->length; i++) {
				if (tlv->operational_rates[i] != 0) {
					rates[rate_cnt++] =
						tlv->operational_rates[i];
				}
			}
		} else {
			printf("ERR:Could not get operational rates!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return rate_cnt;
}

/**
 *  @brief Check rate is valid or not.
 *
 *  @param  rate   Rate for check
 *
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_tx_rate_valid(t_u8 rate)
{
	int rate_cnt = 0;
	int i;
	t_u8 rates[MAX_DATA_RATES];

	rate_cnt = get_sys_cfg_rates((t_u8 *)&rates);
	if (rate_cnt > 0) {
		for (i = 0; i < rate_cnt; i++) {
			if (rate == (rates[i] & ~BASIC_RATE_SET_BIT)) {
				return UAP_SUCCESS;
			}
		}
	}
	return UAP_FAILURE;
}

/**
 *  @brief Check mcbc rate is valid or not.
 *
 *  @param  rate   Rate for check
 *
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_mcbc_rate_valid(t_u8 rate)
{
	int rate_cnt = 0;
	int i;
	t_u8 rates[MAX_DATA_RATES];

	rate_cnt = get_sys_cfg_rates((t_u8 *)&rates);
	if (rate_cnt > 0) {
		for (i = 0; i < rate_cnt; i++) {
			if (rates[i] & BASIC_RATE_SET_BIT) {
				if (rate == (rates[i] & ~BASIC_RATE_SET_BIT)) {
					return UAP_SUCCESS;
				}
			}
		}
	}
	return UAP_FAILURE;
}

/****************************************************************************
        Global functions
****************************************************************************/
/**
 *  @brief Creates a sys_cfg request for AP MAC address
 *   and sends to the driver
 *
 *  Usage: "sys_cfg_ap_mac_address [AP_MAC_ADDRESS]"
 *          if AP_MAC_ADDRESS is provided, a 'set' is performed,
 *          else a 'get' is performed.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_ap_mac_address(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_ap_mac_address *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_ap_mac_address_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_ap_mac_address_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_ap_mac_address);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_ap_mac_address *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_AP_MAC_ADDRESS_TLV_ID;
	tlv->length = ETH_ALEN;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		if ((ret = mac2raw(argv[0], tlv->ap_mac_addr)) != UAP_SUCCESS) {
			printf("ERR: %s Address \n",
			       ret == UAP_FAILURE ? "Invalid MAC" : ret ==
			       UAP_RET_MAC_BROADCAST ? "Broadcast" :
			       "Multicast");
			free(buffer);
			return UAP_FAILURE;
		}
	}
	endian_convert_tlv_header_out(tlv);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_AP_MAC_ADDRESS_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("AP MAC address = ");
				print_mac(tlv->ap_mac_addr);
				printf("\n");
			} else {
				printf("AP MAC address setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get AP MAC address!\n");
			} else {
				printf("ERR:Could not set AP MAC address!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for SSID
 *   and sends to the driver
 *
 *  Usage: "sys_cfg_ssid [SSID]"
 *          if SSID is provided, a 'set' is performed
 *          else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_ssid(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_ssid *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	t_u8 ssid[33];

	argc--;
	argv++;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_ssid_usage();
		return UAP_FAILURE;
	}

	if (argc == 0) {
		/* Initialize the command length */
		cmd_len =
			sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_ssid) +
			MAX_SSID_LENGTH;
	} else {
		if (strlen(argv[0]) > MAX_SSID_LENGTH) {
			printf("ERR:SSID too long.\n");
			return UAP_FAILURE;
		}
		/* Initialize the command length */
		if (argv[0][1] == '"') {
			argv[0]++;
		}
		if (argv[0][strlen(argv[0])] == '"') {
			argv[0][strlen(argv[0])] = '\0';
		}
		if (!strlen(argv[0])) {
			printf("ERR:NULL SSID not allowed.\n");
			return UAP_FAILURE;
		}
		cmd_len =
			sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_ssid) +
			strlen(argv[0]);
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_ssid *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_SSID_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = MAX_SSID_LENGTH;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = strlen(argv[0]);
		memcpy(tlv->ssid, argv[0], tlv->length);
	}

	endian_convert_tlv_header_out(tlv);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_SSID_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				memset(ssid, 0, sizeof(ssid));
				strncpy((char *)ssid, (char *)tlv->ssid,
					sizeof(ssid) - 1);
				printf("SSID = %s\n", ssid);
			} else {
				printf("SSID setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get SSID!\n");
			} else {
				printf("ERR:Could not set SSID!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for beacon period
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_beacon_period [BEACON_PERIOD]"
 *           if BEACON_PERIOD is provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *           BEACON_PERIOD is represented in ms
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_beacon_period(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_beacon_period *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_beacon_period_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && is_input_valid(BEACONPERIOD, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_beacon_period_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_beacon_period);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_beacon_period *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_BEACON_PERIOD_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->beacon_period_ms = (t_u16)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->beacon_period_ms = uap_cpu_to_le16(tlv->beacon_period_ms);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->beacon_period_ms = uap_le16_to_cpu(tlv->beacon_period_ms);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_BEACON_PERIOD_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Beacon period = %d\n",
				       tlv->beacon_period_ms);
			} else {
				printf("Beacon period setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get beacon period!\n");
			} else {
				printf("ERR:Could not set beacon period!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for DTIM period
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_dtim_period [DTIM_PERIOD]"
 *           if DTIM_PERIOD is provided, a 'set' is performed
 *           else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_dtim_period(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_dtim_period *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_dtim_period_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(DTIMPERIOD, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_dtim_period_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_dtim_period);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_dtim_period *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_DTIM_PERIOD_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->dtim_period = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_DTIM_PERIOD_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("DTIM period = %d\n", tlv->dtim_period);
			} else {
				printf("DTIM period setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get DTIM period!\n");
			} else {
				printf("ERR:Could not set DTIM period!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for bss_status
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_bss_status"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_bss_status(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_bss_status *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_bss_status_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_bss_status_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_bss_status);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_bss_status *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_BSS_STATUS_TLV_ID;
	tlv->length = 2;
	cmd_buf->action = ACTION_GET;

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->bss_status = uap_le16_to_cpu(tlv->bss_status);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_BSS_STATUS_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("BSS status = %s\n",
			       (tlv->bss_status == 0) ? "stopped" : "started");
		} else {
			printf("ERR:Could not get BSS status!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/** DFS Radar starting channel FCC */
#define DFS_FCC_RADAR_CHANNEL_START      52
/** DFS Radar ending channel FCC */
#define DFS_FCC_RADAR_CHANNEL_END       144

/** DFS Radar starting channel ETSI */
#define DFS_ETSI_RADAR_CHANNEL_START     52
/** DFS Radar ending channel ETSI */
#define DFS_ETSI_RADAR_CHANNEL_END      140

/** DFS Radar starting channel JAPAN */
#define DFS_JAPAN_RADAR_CHANNEL_START    52
/** DFS Radar ending channel JAPAN */
#define DFS_JAPAN_RADAR_CHANNEL_END     140

/** list of RegDomain values */
typedef enum {
	RegDomain_Null = 0x00,
	RegDomain_FCC = 0x01,
	RegDomain_ETSI = 0x02,
	RegDomain_MIC = 0x03,
	RegDomain_Other = 0xFF,
} regdomain_e;

/** mapping of region and domain code */
typedef struct {
    /** region */
	t_u8 region[COUNTRY_CODE_LEN];
    /** domain code */
	regdomain_e code;
} region_code_mapping_t;

static region_code_mapping_t region_code_mapping[] = {
	{"US ", RegDomain_FCC},	/* US FCC      0x10     */
	{"SG ", RegDomain_FCC},	/* Singapore   0x10   */
	{"EU ", RegDomain_ETSI},	/* ETSI        0x30      */
	{"AU ", RegDomain_ETSI},	/* Australia   0x30  */
	{"KR ", RegDomain_ETSI},	/* Republic Of Korea 0x30 */
	{"JP ", RegDomain_MIC},	/* Japan       0x40 */
	{"J1 ", RegDomain_MIC},	/* Japan1      0x41 */
};

/**
 *  @brief This function converts region string to region code
 *
 *  @param region_string         Region string
 *
 *  @return     Region code
 */
regdomain_e
region_string_2_region_code(char *region_string)
{
	t_u8 i;
	t_u8 size = sizeof(region_code_mapping) / sizeof(region_code_mapping_t);

	for (i = 0; i < COUNTRY_CODE_LEN && region_string[i]; i++) {
		region_string[i] = toupper(region_string[i]);
	}

	for (i = 0; i < size; i++) {
		if (!memcmp(region_string,
			    region_code_mapping[i].region, COUNTRY_CODE_LEN)) {
			return (region_code_mapping[i].code);
		}
	}

	/* default is US */
	return (region_code_mapping[0].code);
}

/**
 *  @brief Checks if channel is dfs channel and allow only if 11h is enabled.
 *
 *  @param channel  Channel
 *  @param country  Country
 *  @return         UAP_FAILURE/UAP_SUCCESS
 */
int
is_dfs_channel(int channel, char *country)
{
	regdomain_e region;
	t_u8 DFS_RadarDetectChannelStart = DFS_FCC_RADAR_CHANNEL_START;
	t_u8 DFS_RadarDetectChannelEnd = DFS_FCC_RADAR_CHANNEL_END;

	region = region_string_2_region_code(country);
	/* set radar start channel and end channel */
	switch (region) {
	default:		/* use FCC as default setting */
	case RegDomain_FCC:
		DFS_RadarDetectChannelStart = DFS_FCC_RADAR_CHANNEL_START;
		DFS_RadarDetectChannelEnd = DFS_FCC_RADAR_CHANNEL_END;
		break;
	case RegDomain_ETSI:
		DFS_RadarDetectChannelStart = DFS_ETSI_RADAR_CHANNEL_START;
		DFS_RadarDetectChannelEnd = DFS_ETSI_RADAR_CHANNEL_END;
		break;
	case RegDomain_MIC:
		DFS_RadarDetectChannelStart = DFS_JAPAN_RADAR_CHANNEL_START;
		DFS_RadarDetectChannelEnd = DFS_JAPAN_RADAR_CHANNEL_END;
		break;
	}
	if (channel < DFS_RadarDetectChannelStart
	    || channel > DFS_RadarDetectChannelEnd) {
		return UAP_FAILURE;
	}

	return UAP_SUCCESS;
}

/**
 *  @brief Checks if channel is valid for given 11d domain.
 *
 *  @param channel  Channel
 *  @param band     Band
 *  @param set_domain Flag to indicate update domain info with this channel
 *  @return         UAP_FAILURE/UAP_SUCCESS
 */
int
check_channel_validity_11d(int channel, int band, int set_domain)
{
	apcmdbuf_cfg_80211d *cmd_buf = NULL;
	ieeetypes_subband_set_t sub_bands[MAX_SUB_BANDS];
	t_u8 *buf = NULL;
	t_u16 cmd_len;
	t_u16 buf_len = 0;
	int ret = UAP_FAILURE;
	int i, j, found = 0;
	t_u8 no_of_sub_band = 0;
	char country[4] = { ' ', ' ', 0, 0 };

	buf_len = sizeof(apcmdbuf_cfg_80211d);
	buf_len += MAX_SUB_BANDS * sizeof(ieeetypes_subband_set_t);
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)buf;
	cmd_len = (sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));

	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->action = ACTION_GET;
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			/* Check channel against US if country code is not set */
			if (!
			    (cmd_buf->domain.country_code[0] ||
			     cmd_buf->domain.country_code[1] ||
			     cmd_buf->domain.country_code[2])) {
				country[0] = 'U';
				country[1] = 'S';
				country[2] = '\0';
				/* Do not send this country code to FW though */
				set_domain = 0;
			} else {
				country[0] = cmd_buf->domain.country_code[0];
				country[1] = cmd_buf->domain.country_code[1];
			}
			if (cmd_buf->domain.country_code[2] == 'I') {
				if ((channel != 36) && (channel != 40) &&
				    (channel != 44) && (channel != 48)) {
					printf("ERR: Channel not valid for indoor operation!\n");
					ret = UAP_FAILURE;
					goto done;
				}
			}
			if (is_dfs_channel(channel, country) == UAP_SUCCESS) {
				printf("DFS channel selected.\n");
			}
			no_of_sub_band =
				parse_domain_file(country, band, sub_bands,
						  NULL);
			/* copy third character of country code here as domain file cannot handle it */
			country[2] = cmd_buf->domain.country_code[2];
			if (no_of_sub_band == UAP_FAILURE) {
				printf("Parsing of domain configuration file failed\n");
				ret = UAP_FAILURE;
				goto done;
			}

			for (i = 0; i < no_of_sub_band; i++) {
				for (j = 0; j < sub_bands[i].no_of_chan; j++) {
					if (channel ==
					    (sub_bands[i].first_chan + j)) {
						found = 1;
						break;
					}
				}
				if (found)
					break;
			}

			if (!found) {
				printf("ERR:Invalid channel %d for given domain!\n", channel);
				ret = UAP_FAILURE;
				goto done;
			}
		} else {
			printf("ERR:Command Response incorrect!\n");
			ret = UAP_FAILURE;
			goto done;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		goto done;
	}
	if (found && set_domain) {
		/*  If channel is valid then set domain_info with new band */
		memset(buf, 0, buf_len);
		buf_len =
			sizeof(apcmdbuf_cfg_80211d) +
			no_of_sub_band * sizeof(ieeetypes_subband_set_t);
		cmd_buf = (apcmdbuf_cfg_80211d *)buf;
		cmd_len = buf_len;
		cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
		cmd_buf->result = 0;
		cmd_buf->seq_num = 0;
		cmd_buf->action = ACTION_SET;
		cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
		cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

		cmd_buf->domain.tag = uap_cpu_to_le16(TLV_TYPE_DOMAIN);
		cmd_buf->domain.length = uap_cpu_to_le16(sizeof(domain_param_t)
							 - BUF_HEADER_SIZE +
							 (no_of_sub_band *
							  sizeof
							  (ieeetypes_subband_set_t)));

		memset(cmd_buf->domain.country_code, ' ',
		       sizeof(cmd_buf->domain.country_code));
		memcpy(cmd_buf->domain.country_code, country, strlen(country));
		memcpy(cmd_buf->domain.subband, sub_bands,
		       no_of_sub_band * sizeof(ieeetypes_subband_set_t));

		ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

		if (ret == UAP_SUCCESS) {
			if (cmd_buf->result == CMD_SUCCESS) {
				ret = UAP_SUCCESS;
			} else {
				printf("ERR:Command Response incorrect!\n");
				ret = UAP_FAILURE;
				goto done;
			}
		} else {
			printf("ERR:Command sending failed!\n");
			ret = UAP_FAILURE;
			goto done;
		}
	}
done:
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for channel
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_channel [CHANNEL] [MODE]"
 *           if CHANNEL is provided, a 'set' is performed
 *           else a 'get' is performed
 *           if MODE is provided, a 'set' is performed with
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_channel(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_channel_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	int mode = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_channel_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && is_input_valid(CHANNEL, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_channel_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_channel_config);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_channel_config *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CHANNELCONFIG_TLV_ID;
	tlv->length = sizeof(tlvbuf_channel_config) - TLVHEADER_LEN;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		if (argc == 1) {
			tlv->chan_number = (t_u8)atoi(argv[0]);
			memset(&(tlv->bandcfg), 0, sizeof(tlv->bandcfg));
		} else {
			mode = atoi(argv[1]);
			memset(&(tlv->bandcfg), 0, sizeof(tlv->bandcfg));
			if (mode & BITMAP_ACS_MODE) {
				int mode;
				if (uap_ioctl_dfs_repeater_mode(&mode) ==
				    UAP_SUCCESS) {
					if (mode) {
						printf("ERR: ACS in DFS Repeater mode" " is not allowed\n");
						ret = UAP_FAILURE;
						goto done;
					}
				}
				tlv->bandcfg.scanMode = SCAN_MODE_ACS;
			}
			if (mode & BITMAP_CHANNEL_ABOVE)
				tlv->bandcfg.chan2Offset = SEC_CHAN_ABOVE;
			if (mode & BITMAP_CHANNEL_BELOW)
				tlv->bandcfg.chan2Offset = SEC_CHAN_BELOW;
			tlv->chan_number = (t_u8)atoi(argv[0]);
		}
		if (atoi(argv[0]) > MAX_CHANNELS_BG) {
			tlv->bandcfg.chanBand = BAND_5GHZ;
		}
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CHANNELCONFIG_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Mode    = %s\n",
				       (tlv->bandcfg.scanMode ==
					SCAN_MODE_ACS) ? "ACS" : "Manual");
				printf("Channel = %d\n", tlv->chan_number);
				if (tlv->bandcfg.chan2Offset == SEC_CHAN_NONE)
					printf("no secondary channel\n");
				else if (tlv->bandcfg.chan2Offset ==
					 SEC_CHAN_ABOVE)
					printf("secondary channel is above primary channel\n");
				else if (tlv->bandcfg.chan2Offset ==
					 SEC_CHAN_BELOW)
					printf("secondary channel is below primary channel\n");
			} else {
				printf("Channel setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get channel!\n");
			} else {
				printf("ERR:Could not set channel!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for channel
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_channel_ext [CHANNEL] [BAND] [MODE]"
 *           if CHANNEL is provided, a 'set' is performed
 *           else a 'get' is performed
 *           if BAND is provided, a 'set' is performed with band info(2.4GHz/5GHz)
 *           if MODE is provided, a 'set' is performed with mode (ACS/secondary channel)
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_channel_ext(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_channel_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	int mode = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_channel_ext_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && is_input_valid(CHANNEL_EXT, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_channel_ext_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_channel_config);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_channel_config *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CHANNELCONFIG_TLV_ID;
	tlv->length = sizeof(tlvbuf_channel_config) - TLVHEADER_LEN;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->chan_number = (t_u8)atoi(argv[0]);
		if (argc == 1) {
			if (atoi(argv[0]) > MAX_CHANNELS_BG)
				tlv->bandcfg.chanBand = BAND_5GHZ;
		} else {
			if (atoi(argv[1]) == 0) {
				tlv->bandcfg.chanBand = BAND_2GHZ;
			} else {
				tlv->bandcfg.chanBand = BAND_5GHZ;
			}
			if (argc == 3) {
				mode = atoi(argv[2]);
				if (mode & BITMAP_ACS_MODE) {
					int mode;
					if (uap_ioctl_dfs_repeater_mode(&mode)
					    == UAP_SUCCESS) {
						if (mode) {
							printf("ERR: ACS in DFS Repeater mode" " is not allowed\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
					tlv->bandcfg.scanMode = SCAN_MODE_ACS;
				}
				if (mode & BITMAP_CHANNEL_ABOVE)
					tlv->bandcfg.chan2Offset =
						SEC_CHAN_ABOVE;
				if (mode & BITMAP_CHANNEL_BELOW)
					tlv->bandcfg.chan2Offset =
						SEC_CHAN_BELOW;
			}
		}
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CHANNELCONFIG_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Mode    = %s\n",
				       (tlv->bandcfg.scanMode ==
					SCAN_MODE_ACS) ? "ACS" : "Manual");
				printf("Channel = %d\n", tlv->chan_number);
				printf("Band    = %s\n",
				       (tlv->bandcfg.chanBand ==
					BAND_5GHZ) ? "5GHz" : "2.4GHz");
				if (tlv->bandcfg.chan2Offset == SEC_CHAN_NONE)
					printf("no secondary channel\n");
				else if (tlv->bandcfg.chan2Offset ==
					 SEC_CHAN_ABOVE)
					printf("secondary channel is above primary channel\n");
				else if (tlv->bandcfg.chan2Offset ==
					 SEC_CHAN_BELOW)
					printf("secondary channel is below primary channel\n");
			} else {
				printf("Channel setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get channel!\n");
			} else {
				printf("ERR:Could not set channel!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for channel list
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_scan_channels [CHANNEL[.BAND]]"
 *           if CHANNEL and BAND are provided, a 'set' is performed
 *           else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_scan_channels(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_channel_list *tlv = NULL;
	channel_list *pchan_list = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_FAILURE;
	int opt;
	int i;
	int scan_channels_band;
	int chan_number = 0;
	int band_flag = -1;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_scan_channels_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && is_input_valid(SCANCHANNELS, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_scan_channels_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	if (argc == 0)
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_channel_list) +
			sizeof(channel_list) * MAX_CHANNELS;
	else
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_channel_list) +
			sizeof(channel_list) * argc;

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* LOCATE HEADERS */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_channel_list *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CHANNELLIST_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = sizeof(channel_list) * MAX_CHANNELS;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = sizeof(channel_list) * argc;
		pchan_list = tlv->chan_list;
		for (i = 0; i < argc; i++) {
			band_flag = -1;
			sscanf(argv[i], "%d.%d", &chan_number, &band_flag);
			pchan_list->chan_number = chan_number;
			pchan_list->bandcfg.chanBand = BAND_2GHZ;
			if (((band_flag != -1) && (band_flag)) ||
			    (chan_number > MAX_CHANNELS_BG)) {
				pchan_list->bandcfg.chanBand = BAND_5GHZ;
			}
			scan_channels_band = BAND_B | BAND_G;
			if ((scan_channels_band != BAND_A) &&
			    (pchan_list->bandcfg.chanBand == BAND_5GHZ)) {
				scan_channels_band = BAND_A;
			}
			if (check_channel_validity_11d
			    (pchan_list->chan_number, scan_channels_band,
			     0) == UAP_FAILURE) {
				free(buffer);
				return UAP_FAILURE;
			}
			pchan_list++;
		}
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CHANNELLIST_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Channels List = ");
				if (tlv->length % sizeof(channel_list)) {
					printf("Error: Length mismatch\n");
					free(buffer);
					return UAP_FAILURE;
				}
				pchan_list = tlv->chan_list;
				for (i = 0;
				     (unsigned int)i <
				     (tlv->length / sizeof(channel_list));
				     i++) {
					printf("\n%d\t%sGHz",
					       pchan_list->chan_number,
					       (pchan_list->bandcfg.chanBand ==
						BAND_5GHZ) ? "5" : "2.4");
					pchan_list++;
				}
				printf("\n");
			} else {
				printf("Scan Channel List setting successful\n");
			}
		} else {
			printf("ERR:Could not %s scan channel list!\n",
			       argc ? "SET" : "GET");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Parser for sys_cfg_rates_ext input
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @param output   Stores indexes for "rates, mbrate and urate"
 *                  arguments
 *
 *                  e.g.,
 *
 *                  "rates 0x82 4 16 22 0x30 mbrate 2 urate 16"
 *
 *                  will have output array as
 *
 *                          start_index end_index
 *                  rates       0           5
 *                  mbrate      6           7
 *                  urate       8           9
 *
 *  @return         NA
 *
 */
void
parse_input(int argc, char **argv, int output[3][2])
{
	int i, j, k = 0;
	char *keywords[3] = { "rates", "mbrate", "urate" };

	for (i = 0; i < 3; i++)
		output[i][0] = -1;

	for (i = 0; i < argc; i++) {
		for (j = 0; j < 3; j++) {
			if (strcmp(argv[i], keywords[j]) == 0) {
				output[j][1] = output[j][0] = i;
				k = j;
				break;
			}
		}
		output[k][1] += 1;
	}
}

/**
 *  @brief Creates a sys_cfg request for setting data_rates, MCBC and
 *   TX data rates.
 *
 *   Usage: "sys_cfg_rates_ext  [RATES]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_rates_ext(int argc, char *argv[])
{
	int i, j = 0, found = 0;
	int opt;
	int rflag = 0, mflag = 0, uflag = 0;
	char *argv_rate[14];
	int argc_rate = 0;
	char *argv_mrate[1];
	int mrate_found = UAP_FAILURE;
	t_u8 *tlv_buf = NULL;
	tlvbuf_mcbc_data_rate *tlv_mrate = NULL;
	tlvbuf_rates *tlv_rate = NULL;
	t_u8 *buffer = NULL;
	apcmdbuf_sys_configure *cmd_buf = NULL;
	int ret = UAP_SUCCESS;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int output[3][2];
	char *keywords[3] = { "rates", "mbrate" };
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_rates_ext_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc) {
		/*
		 * SET
		 */
		parse_input(argc, argv, output);

		for (i = 0; i < 3; i++) {
			if (output[i][0] == -1) {
				for (j = 0; j < 3; j++) {
					if (output[j][0] == 0)
						found = 1;
				}
				if (!found) {
					printf("%s keyword not specified!\n",
					       keywords[i]);
					ret = UAP_FAILURE;
					goto done;
				}
			}
		}
		/*
		 * Rate
		 */
		if ((output[0][0] != -1) && (output[0][1] > output[0][0])) {
			rflag = 1;
			i = 0;
			for (j = (output[0][0] + 1); j < output[0][1]; j++) {
				argv_rate[i] =
					(char *)malloc(sizeof(char) *
						       (strlen(argv[j]) + 1));
				memset(argv_rate[i], 0,
				       sizeof(char) * (strlen(argv[j]) + 1));
				strncpy(argv_rate[i], argv[j], strlen(argv[j]));
				argc_rate = ++i;
			}
		}

		/*
		 * mrate
		 */

		if ((output[1][0] != -1) && (output[1][1] > output[1][0])) {
			if ((output[1][1] - output[1][0]) != 2) {
				printf("\nERR: Invalid mrate input");
				print_sys_cfg_rates_ext_usage();
				ret = UAP_FAILURE;
				goto done;
			}
			mflag = 1;
			argv_mrate[0] =
				(char *)malloc(sizeof(char) *
					       (strlen(argv[j]) + 1));
			memset(argv_mrate[0], 0,
			       sizeof(char) * (strlen(argv[j]) + 1));
			strncpy(argv_mrate[0], argv[output[1][0] + 1],
				strlen(argv[j]));
		}
		if (!rflag && !mflag & !uflag) {
			printf("ERR: Invalid input\n");
			print_sys_cfg_rates_ext_usage();
			ret = UAP_FAILURE;
			goto done;
		}

		if (rflag &&
		    is_input_valid(RATE, argc_rate, argv_rate) != UAP_SUCCESS) {
			printf("ERR: Invalid RATE\n");
			print_sys_cfg_rates_ext_usage();
			ret = UAP_FAILURE;
			goto done;
		}

		if (mflag &&
		    is_input_valid(MCBCDATARATE, 1,
				   argv_mrate) != UAP_SUCCESS) {
			printf("ERR: Invalid MCBC RATE\n");
			print_sys_cfg_rates_ext_usage();
			ret = UAP_FAILURE;
			goto done;
		}

		if (!rflag && (mflag || uflag)) {
			/*
			 * Check mrate and urate wrt old Rates
			 */
			if (mflag && A2HEXDECIMAL(argv_mrate[0]) &&
			    is_mcbc_rate_valid(A2HEXDECIMAL(argv_mrate[0])) !=
			    UAP_SUCCESS) {
				printf("ERR: invalid MCBC data rate.");
				print_sys_cfg_rates_ext_usage();
				ret = UAP_FAILURE;
				goto done;
			}
		} else if (rflag && (mflag || uflag)) {
			/*
			 * Check mrate and urate wrt new Rates
			 */
			for (i = 0; i < argc_rate; i++) {
				/*
				 * MCBC rate must be from Basic rates
				 */
				if (mflag && !mrate_found &&
				    A2HEXDECIMAL(argv_rate[i]) &
				    BASIC_RATE_SET_BIT) {
					if (A2HEXDECIMAL(argv_mrate[0]) ==
					    (A2HEXDECIMAL(argv_rate[i]) &
					     ~BASIC_RATE_SET_BIT)) {
						mrate_found = UAP_SUCCESS;
					}
				}
			}

			if (mflag && A2HEXDECIMAL(argv_mrate[0]) &&
			    !(mrate_found == UAP_SUCCESS)) {
				printf("ERR: mrate not valid\n");
				ret = UAP_FAILURE;
				goto done;
			}
		}
		/* Post-parsing */
		cmd_len = sizeof(apcmdbuf_sys_configure);
		if (rflag)
			cmd_len += sizeof(tlvbuf_rates) + argc_rate;
		if (mflag)
			cmd_len += sizeof(tlvbuf_mcbc_data_rate);
	} else {
		/* GET */
		cmd_len = sizeof(apcmdbuf_sys_configure)
			+ sizeof(tlvbuf_rates) + MAX_RATES +
			sizeof(tlvbuf_mcbc_data_rate);
	}

	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Fill the command buffer */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = argc ? ACTION_SET : ACTION_GET;
	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);
	/* Locate headers */
	if (rflag || (!argc)) {
		tlv_rate =
			(tlvbuf_rates *)(buffer +
					 sizeof(apcmdbuf_sys_configure));
		tlv_rate->tag = MRVL_RATES_TLV_ID;
		tlv_rate->length = argc ? argc_rate : MAX_RATES;
		for (i = 0; i < argc_rate; i++) {
			tlv_rate->operational_rates[i] =
				(t_u8)A2HEXDECIMAL(argv_rate[i]);
		}
		tlv_buf += tlv_rate->length + sizeof(tlvbuf_rates);
		endian_convert_tlv_header_out(tlv_rate);
	}
	if (mflag || (!argc)) {
		tlv_mrate = (tlvbuf_mcbc_data_rate *)tlv_buf;
		tlv_mrate->tag = MRVL_MCBC_DATA_RATE_TLV_ID;
		tlv_mrate->length = 2;
		tlv_mrate->mcbc_datarate = 0;
		if (mflag) {
			tlv_mrate->mcbc_datarate =
				(t_u16)A2HEXDECIMAL(argv_mrate[0])
				& ~BASIC_RATE_SET_BIT;
			tlv_mrate->mcbc_datarate =
				uap_cpu_to_le16(tlv_mrate->mcbc_datarate);
		}
		tlv_buf += sizeof(tlvbuf_mcbc_data_rate);
		endian_convert_tlv_header_out(tlv_mrate);
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);

	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x\n",
			       cmd_buf->cmd_code);
			ret = UAP_FAILURE;
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("Rates setting successful\n");
			} else {
				print_tlv((t_u8 *)tlv_buf,
					  cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}
		} else {
			printf("ERR:Could not %s operational rates!\n",
			       argc ? "set" : "get");
			if (argc)
				printf("operational rates only allow to set before bss start.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (rflag) {
		for (i = 0; i < argc_rate; i++) {
			free(argv_rate[i]);
		}
	}
	if (mflag)
		free(argv_mrate[0]);
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for data rates
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_rates  [RATES]"
 *
 *           RATES is provided as a set of data rates, in
 *           unit of 500 kilobits/s.
 *           Maximum 12 rates can be provided.
 *
 *           if no RATE is provided, then it gets configured rates
 *
 *           Each rate must be separated by a space
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_rates(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_rates *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int i = 0;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_rates_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && is_input_valid(RATE, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_rates_usage();
		return UAP_FAILURE;
	}
	if (argc == 0) {
		/* Initialize the command length */
		cmd_len =
			sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_rates) +
			MAX_RATES;
	} else {
		/* Initialize the command length */
		cmd_len =
			sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_rates) +
			argc;
	}
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_rates *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RATES_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = MAX_RATES;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = argc;
		for (i = 0; i < tlv->length; i++) {
			tlv->operational_rates[i] = (t_u8)A2HEXDECIMAL(argv[i]);
		}
	}

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (uap_le16_to_cpu(tlv->tag) != MRVL_RATES_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, uap_le16_to_cpu(tlv->tag));
			free(buffer);
			return UAP_FAILURE;
		}

		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				print_rate(tlv);
			} else {
				printf("Rates setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get operational rates!\n");
			} else {
				printf("ERR:Could not set operational rates!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Checks if tx power is valid for given 11d domain.
 *
 *  @param tx_pwr  tx power
 *  @return         UAP_FAILURE/UAP_SUCCESS
 */
int
check_tx_pwr_validity_11d(t_u8 tx_pwr)
{
	apcmdbuf_sys_configure *cmd_buf1 = NULL;
	apcmdbuf_cfg_80211d *cmd_buf2 = NULL;
	tlvbuf_channel_config *tlv = NULL;
	ieeetypes_subband_set_t sub_bands[MAX_SUB_BANDS];
	t_u8 *buf = NULL;
	t_u16 cmd_len;
	t_u16 buf_len = 0;
	int ret = UAP_FAILURE;
	int i, found = 0, band = BAND_B | BAND_G;
	t_u8 no_of_sub_band = 0, channel = 0;
	char country[4] = { ' ', ' ', 0, 0 };

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_channel_config);

	/* Initialize the command buffer */
	buf = (t_u8 *)malloc(cmd_len);

	if (!buf) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, cmd_len);

	/* Locate headers */
	cmd_buf1 = (apcmdbuf_sys_configure *)buf;
	tlv = (tlvbuf_channel_config *)(buf + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf1->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf1->size = cmd_len;
	cmd_buf1->seq_num = 0;
	cmd_buf1->result = 0;
	tlv->tag = MRVL_CHANNELCONFIG_TLV_ID;
	tlv->length = sizeof(tlvbuf_channel_config) - TLVHEADER_LEN;
	cmd_buf1->action = ACTION_GET;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf1, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf1->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CHANNELCONFIG_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf1->cmd_code, tlv->tag);
			free(buf);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf1->result == CMD_SUCCESS) {
			channel = tlv->chan_number;
			printf("channel=%d\n", channel);
		} else {
			printf("ERR:Could not get channel!\n");
			free(buf);
			return UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		free(buf);
		return UAP_FAILURE;
	}
	/* free current buffer */
	free(buf);
	if (channel > MAX_CHANNELS_BG)
		band = BAND_A;
	/* initialize second command length and buffer */
	buf_len = sizeof(apcmdbuf_cfg_80211d);
	buf_len += MAX_SUB_BANDS * sizeof(ieeetypes_subband_set_t);
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	/* Locate headers */
	cmd_buf2 = (apcmdbuf_cfg_80211d *)buf;
	cmd_len = (sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));

	cmd_buf2->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf2->result = 0;
	cmd_buf2->seq_num = 0;
	cmd_buf2->action = ACTION_GET;
	cmd_buf2->action = uap_cpu_to_le16(cmd_buf2->action);
	cmd_buf2->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

	ret = uap_ioctl((t_u8 *)cmd_buf2, &cmd_len, buf_len);

	if (ret == UAP_SUCCESS) {
		if (cmd_buf2->result == CMD_SUCCESS) {
			if (cmd_buf2->domain.country_code[0] ||
			    cmd_buf2->domain.country_code[1] ||
			    cmd_buf2->domain.country_code[2]) {
				country[0] = cmd_buf2->domain.country_code[0];
				country[1] = cmd_buf2->domain.country_code[1];
				if (cmd_buf2->domain.country_code[2] == 'I') {
					if (tx_pwr > MAX_TX_PWR_INDOOR) {
						printf("Transmit power invalid for indoor operation!\n");
						ret = UAP_FAILURE;
						goto done;
					}
				}
				no_of_sub_band =
					parse_domain_file(country, band,
							  sub_bands, NULL);
				if (no_of_sub_band == UAP_FAILURE) {
					printf("Parsing of domain configuration file failed\n");
					ret = UAP_FAILURE;
					goto done;
				}

				for (i = 0; i < no_of_sub_band; i++) {
					if (tx_pwr <= (sub_bands[i].max_tx_pwr)) {
						found = 1;
						break;
					}
				}

				if (!found) {
					printf("ERR:Invalid transmit power for given domain!\n");
					ret = UAP_FAILURE;
					goto done;
				}
			}
		} else {
			printf("ERR:Command Response incorrect!\n");
			ret = UAP_FAILURE;
			goto done;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for Tx power
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_tx_power [TX_POWER]"
 *           if TX_POWER is provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *           TX_POWER is represented in dBm
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_tx_power(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_tx_power *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	t_u8 state = 0;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_tx_power_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && is_input_valid(TXPOWER, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_tx_power_usage();
		return UAP_FAILURE;
	}
	/* Check if tx power is valid for given domain. */
	if (argc) {
		ret = sg_snmp_mib(ACTION_GET, OID_80211D_ENABLE, sizeof(state),
				  &state);
		if (state) {
			ret = check_tx_pwr_validity_11d(atoi(argv[0]));
			if (ret == UAP_FAILURE) {
				print_sys_cfg_tx_power_usage();
				return UAP_FAILURE;
			}
		}
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tx_power);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_tx_power *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_TX_POWER_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		printf("Please check power calibration for board to see if this power\n" "setting is within calibrated range. Firmware may over-ride\n " "this setting if it is not within calibrated range, which can\n" "vary from board to board.\n");
		cmd_buf->action = ACTION_SET;
		tlv->tx_power_dbm = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_TX_POWER_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Tx power = %d dBm\n",
				       tlv->tx_power_dbm);
			} else {
				printf("Tx power setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get tx power!\n");
			} else {
				printf("ERR:Could not set tx power!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for SSID broadcast
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_bcast_ssid_ctl [0|1]"
 *
 *   Options: 0     - Disable SSID broadcast
 *            1     - Enable SSID broadcast
 *            empty - Get current SSID broadcast setting
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_bcast_ssid_ctl(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_bcast_ssid_ctl *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_bcast_ssid_ctl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && is_input_valid(BROADCASTSSID, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_bcast_ssid_ctl_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_bcast_ssid_ctl);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_bcast_ssid_ctl *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_BCAST_SSID_CTL_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->bcast_ssid_ctl = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_BCAST_SSID_CTL_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("SSID broadcast is %s\n",
				       (tlv->bcast_ssid_ctl ==
					1) ? "enabled" : "disabled");
			} else {
				printf("SSID broadcast setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get SSID broadcast!\n");
			} else {
				printf("ERR:Could not set SSID broadcast!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for preamble settings
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_preamble_ctl"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_preamble_ctl(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_preamble_ctl *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_preamble_ctl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_preamble_ctl_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_preamble_ctl);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_preamble_ctl *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_PREAMBLE_CTL_TLV_ID;
	tlv->length = 1;
	cmd_buf->action = ACTION_GET;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_PREAMBLE_CTL_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS)
			printf("Preamble type is %s\n",
			       (tlv->preamble_type ==
				0) ? "auto" : ((tlv->preamble_type ==
						1) ? "short" : "long"));
		else {
			printf("ERR:Could not get preamble type!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for RTS threshold
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_rts_threshold [RTS_THRESHOLD]"
 *           if RTS_THRESHOLD is provided, a 'set' is performed
 *           else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_rts_threshold(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_rts_threshold *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_rts_threshold_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(RTSTHRESH, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_rts_threshold_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_rts_threshold);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_rts_threshold *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RTS_THRESHOLD_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->rts_threshold = (t_u16)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->rts_threshold = uap_cpu_to_le16(tlv->rts_threshold);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->rts_threshold = uap_le16_to_cpu(tlv->rts_threshold);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_RTS_THRESHOLD_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("RTS threshold = %d\n",
				       tlv->rts_threshold);
			} else {
				printf("RTS threshold setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get RTS threshold!\n");
			} else {
				printf("ERR:Could not set RTS threshold!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for Fragmentation threshold
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_frag_threshold [FRAG_THRESHOLD]"
 *           if FRAG_THRESHOLD is provided, a 'set' is performed
 *           else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_frag_threshold(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_frag_threshold *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_frag_threshold_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(FRAGTHRESH, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_frag_threshold_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_frag_threshold);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_frag_threshold *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_FRAG_THRESHOLD_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->frag_threshold = (t_u16)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->frag_threshold = uap_cpu_to_le16(tlv->frag_threshold);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->frag_threshold = uap_le16_to_cpu(tlv->frag_threshold);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_FRAG_THRESHOLD_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Fragmentation threshold = %d\n",
				       tlv->frag_threshold);
			} else {
				printf("Fragmentation threshold setting successful\n");
			}
		} else {
			if (argc == 1) {
				printf("ERR:Could not set Fragmentation threshold!\n");
			} else {
				printf("ERR:Could not get Fragmentation threshold!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for RSN replay protection
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_rsn_replay_prot [0|1]"
 *
 *   Options: 0     - Disable RSN replay protection
 *            1     - Enable  RSN replay protection
 *            empty - Get current RSN replay protection setting
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_rsn_replay_prot(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_rsn_replay_prot *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_rsn_replay_prot_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && is_input_valid(RSNREPLAYPROT, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_rsn_replay_prot_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_rsn_replay_prot);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_rsn_replay_prot *)(buffer +
					 sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RSN_REPLAY_PROT_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->rsn_replay_prot = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_RSN_REPLAY_PROT_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->Tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("RSN replay protection is %s\n",
				       (tlv->rsn_replay_prot ==
					1) ? "enabled" : "disabled");
			} else {
				printf("RSN replay protection setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get RSN replay protection !\n");
			} else {
				printf("ERR:Could not set RSN replay protection !\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for MCBC data rates
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_mcbc_data_rate [MCBC_DATA_RATE]"
 *
 *   Options: 0     - Auto rate
 *            >0    - Set specified MCBC data rate
 *            empty - Get current MCBC data rate
 *
 *           MCBC_DATA_RATE is represented in units of 500 kbps
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_mcbc_data_rate(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_mcbc_data_rate *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_mcbc_data_rates_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if (is_input_valid(MCBCDATARATE, argc, argv) != UAP_SUCCESS) {
			printf("ERR: Invalid input\n");
			print_sys_cfg_mcbc_data_rates_usage();
			return UAP_FAILURE;
		}
		if ((A2HEXDECIMAL(argv[0]) != 0) &&
		    (is_mcbc_rate_valid(A2HEXDECIMAL(argv[0])) !=
		     UAP_SUCCESS)) {
			printf("ERR: invalid MCBC data rate.");
			print_sys_cfg_mcbc_data_rates_usage();
			return UAP_FAILURE;
		}
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_mcbc_data_rate);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_mcbc_data_rate *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_MCBC_DATA_RATE_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->mcbc_datarate = (t_u16)A2HEXDECIMAL(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->mcbc_datarate = uap_cpu_to_le16(tlv->mcbc_datarate);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->mcbc_datarate = uap_le16_to_cpu(tlv->mcbc_datarate);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_MCBC_DATA_RATE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				if (tlv->mcbc_datarate == 0) {
					printf("MCBC data rate is auto\n");
				} else {
					printf("MCBC data rate = 0x%x\n",
					       tlv->mcbc_datarate);
				}
			} else {
				printf("MCBC data rate setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get MCBC data rate!\n");
			} else {
				printf("ERR:Could not set MCBC data rate!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for Tx beacon rates
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_tx_beacon_rate [TX_BEACON_RATE]"
 *
 *   Options: 0     - Auto rate
 *            >0    - Set specified data rate
 *            empty - Get current data rate
 *
 *           TX_BEACON_RATE is represented in units of 500 kbps
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_tx_beacon_rate(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_tx_data_rate *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_tx_beacon_rates_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if (is_input_valid(TXBEACONRATE, argc, argv) != UAP_SUCCESS) {
			printf("ERR: Invalid input\n");
			return UAP_FAILURE;
		} else if ((A2HEXDECIMAL(argv[0]) != 0) &&
			   (is_tx_rate_valid(A2HEXDECIMAL(argv[0])) !=
			    UAP_SUCCESS)) {
			printf("ERR: invalid tx beacon rate.");
			return UAP_FAILURE;
		}
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tx_data_rate);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_tx_data_rate *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_TX_BEACON_RATE_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->tx_data_rate = (t_u16)A2HEXDECIMAL(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->tx_data_rate = uap_cpu_to_le16(tlv->tx_data_rate);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->tx_data_rate = uap_le16_to_cpu(tlv->tx_data_rate);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_TX_BEACON_RATE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				if (tlv->tx_data_rate == 0) {
					printf("Tx beacon rate is auto\n");
				} else {
					printf("Tx beacon rate = 0x%x\n",
					       tlv->tx_data_rate);
				}
			} else {
				printf("Tx beacon rate setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get tx beacon rate!\n");
			} else {
				printf("ERR:Could not set tx beacon rate!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for packet forwarding
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_pkt_fwd_ctl [PKT_FWD_CTRL]"
 *
 *   PKT_FWD_CTRL: bit 0 -- Packet forwarding handled by Host (0) or Firmware (1)
 *            bit 1 -- Intra-BSS broadcast packets are allowed (0) or denied (1)
 *            bit 2 -- Intra-BSS unicast packets are allowed (0) or denied (1)
 *            bit 3 -- Inter-BSS unicast packets are allowed (0) or denied (1)
 *            empty - Get current packet forwarding setting
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_pkt_fwd_ctl(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_pkt_fwd_ctl *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_pkt_fwd_ctl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(PKTFWD, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_pkt_fwd_ctl_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_pkt_fwd_ctl);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_pkt_fwd_ctl *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_PKT_FWD_CTL_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->pkt_fwd_ctl = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_PKT_FWD_CTL_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("%s handles packet forwarding -\n",
				       ((tlv->pkt_fwd_ctl & PKT_FWD_FW_BIT) ==
					0) ? "Host" : "Firmware");
				printf("\tIntra-BSS broadcast packets are %s\n",
				       ((tlv->
					 pkt_fwd_ctl & PKT_FWD_INTRA_BCAST) ==
					0) ? "allowed" : "denied");
				printf("\tIntra-BSS unicast packets are %s\n",
				       ((tlv->
					 pkt_fwd_ctl & PKT_FWD_INTRA_UCAST) ==
					0) ? "allowed" : "denied");
				printf("\tInter-BSS unicast packets are %s\n",
				       ((tlv->
					 pkt_fwd_ctl & PKT_FWD_INTER_UCAST) ==
					0) ? "allowed" : "denied");
			} else {
				printf("Packet control logic setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get packet control logic!\n");
			} else {
				printf("ERR:Could not set packet control logic!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for STA ageout timer
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_sta_ageout_timer [STA_AGEOUT_TIMER]"
 *           if STA_AGEOUT_TIMER is provided, a 'set' is performed
 *           else a 'get' is performed.
 *           The value should between 100 and 864000
 *
 *           STA_AGEOUT_TIMER is represented in units of 100 ms
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_sta_ageout_timer(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_sta_ageout_timer *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_sta_ageout_timer_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(STAAGEOUTTIMER, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_sta_ageout_timer_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_sta_ageout_timer);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_sta_ageout_timer *)(buffer +
					  sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_STA_AGEOUT_TIMER_TLV_ID;
	tlv->length = 4;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->sta_ageout_timer_ms = (t_u32)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->sta_ageout_timer_ms = uap_cpu_to_le32(tlv->sta_ageout_timer_ms);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->sta_ageout_timer_ms = uap_le32_to_cpu(tlv->sta_ageout_timer_ms);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_STA_AGEOUT_TIMER_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("STA ageout timer value = %d\n",
				       (int)tlv->sta_ageout_timer_ms);
			} else {
				printf("STA ageout timer setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get STA ageout timer!\n");
			} else {
				printf("ERR:Could not set STA ageout timer!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for PS STA ageout timer
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_ps_sta_ageout_timer [PS_STA_AGEOUT_TIMER]"
 *           if PS_STA_AGEOUT_TIMER is provided, a 'set' is performed
 *           else a 'get' is performed.
 *           The value should between 100 and 864000
 *
 *           PS_STA_AGEOUT_TIMER is represented in units of 100 ms
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_ps_sta_ageout_timer(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_ps_sta_ageout_timer *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_ps_sta_ageout_timer_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc &&
	    (is_input_valid(PSSTAAGEOUTTIMER, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_ps_sta_ageout_timer_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_ps_sta_ageout_timer);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_ps_sta_ageout_timer *)(buffer +
					     sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_PS_STA_AGEOUT_TIMER_TLV_ID;
	tlv->length = 4;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->ps_sta_ageout_timer_ms = (t_u32)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->ps_sta_ageout_timer_ms =
		uap_cpu_to_le32(tlv->ps_sta_ageout_timer_ms);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->ps_sta_ageout_timer_ms =
		uap_le32_to_cpu(tlv->ps_sta_ageout_timer_ms);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_PS_STA_AGEOUT_TIMER_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("PS STA ageout timer value = %d\n",
				       (int)tlv->ps_sta_ageout_timer_ms);
			} else {
				printf("PS STA ageout timer setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get PS STA ageout timer!\n");
			} else {
				printf("ERR:Could not set PS STA ageout timer!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for authentication mode
 *   and sends to the driver
 *
 *   Usage: "Usage : sys_cfg_auth [AUTHMODE]"
 *
 *   Options: AUTHMODE :     0 - Open authentication
 *                           1 - Shared key authentication
 *            empty - Get current authentication mode
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_auth(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_auth_mode *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_auth_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(AUTHMODE, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_auth_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_auth_mode);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_auth_mode *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_AUTH_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->auth_mode = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_AUTH_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				print_auth(tlv);
			} else {
				printf("authentication mode setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get authentication mode!\n");
			} else {
				printf("ERR:Could not set authentication mode!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for encryption protocol
 *   and sends to the driver
 *
 *   Usage: "Usage : sys_cfg_protocol [PROTOCOL]"
 *
 *   Options: PROTOCOL    	     Bit 0 - No RSN
 *		                     Bit 1 - WEP Static
 *		                     Bit 3 - WPA
 *	                             Bit 3 - WPA2
 *            empty - Get current protocol
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_protocol(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_protocol *tlv = NULL;
	tlvbuf_akmp *akmp_tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	t_u16 cmd_len = 0;
	int ret = UAP_SUCCESS;
	int opt;
	int get_akm_val = 0;
	int resend = 0;
	t_u16 tlv_val_16 = 0;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_protocol_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc && (is_input_valid(PROTOCOL, argc, argv) != UAP_SUCCESS ||
		     is_input_valid(AKM_SUITE, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_protocol_usage();
		return UAP_FAILURE;
	}

	if ((argc == 1) &&
	    ((atoi(argv[0]) == PROTOCOL_WPA) ||
	     (atoi(argv[0]) == PROTOCOL_WPA2) ||
	     (atoi(argv[0]) == PROTOCOL_WPA2_MIXED)
	    )) {
		get_akm_val = 1;
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}

	/* case such as ./uaputl.exe sys_cfg_protocol 8/32/40 */
	/* Don't assume PSK in such cases, query FW and then set  */
	do {
		memset(buffer, 0, buf_len);

		/* Initialize the command length */
		if ((argc == 1) &&
		    ((atoi(argv[0]) == PROTOCOL_NO_SECURITY) ||
		     (atoi(argv[0]) == PROTOCOL_STATIC_WEP))) {
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_protocol);
		} else {
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_protocol) + sizeof(tlvbuf_akmp);
		}

		/* Locate headers */
		cmd_buf = (apcmdbuf_sys_configure *)buffer;
		tlv = (tlvbuf_protocol *)(buffer +
					  sizeof(apcmdbuf_sys_configure));
		akmp_tlv =
			(tlvbuf_akmp *)(buffer +
					sizeof(apcmdbuf_sys_configure) +
					sizeof(tlvbuf_protocol));
		/* Fill the command buffer */
		cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
		cmd_buf->size = cmd_len;
		cmd_buf->seq_num = 0;
		cmd_buf->result = 0;
		tlv->tag = MRVL_PROTOCOL_TLV_ID;
		tlv->length = 2;
		if (argc == 0) {
			cmd_buf->action = ACTION_GET;
			akmp_tlv->tag = MRVL_AKMP_TLV_ID;
			akmp_tlv->length = 4;	/* sizeof(tlvbuf_akmp) - TLVHEADER_LEN */
		} else {
			cmd_buf->action = ACTION_SET;
			tlv->protocol = (t_u16)atoi(argv[0]);
			printf("protocol: 0x%x\n", tlv->protocol);
			if (tlv->protocol & (PROTOCOL_WPA | PROTOCOL_WPA2)) {
				akmp_tlv->tag = MRVL_AKMP_TLV_ID;
				akmp_tlv->length = 4;	/* sizeof(tlvbuf_akmp) - TLVHEADER_LEN */
				if (argc == 2) {
					akmp_tlv->key_mgmt =
						(t_u16)A2HEXDECIMAL(argv[1]);
				} else {
					akmp_tlv->key_mgmt = tlv_val_16;	// AKM SUITE from FW in 2nd iteration
					if (get_akm_val) {
						/* Get AKM SUITE val from FW */
						cmd_buf->action = ACTION_GET;
						resend = 1;
					} else
						resend = 0;
				}
				printf("KeyMgmt: 0x%x\n", akmp_tlv->key_mgmt);
				akmp_tlv->key_mgmt =
					uap_cpu_to_le16(akmp_tlv->key_mgmt);
			}
		}
		endian_convert_tlv_header_out(tlv);
		if (cmd_len ==
		    (sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_protocol) +
		     sizeof(tlvbuf_akmp))) {
			endian_convert_tlv_header_out(akmp_tlv);
		}
		tlv->protocol = uap_cpu_to_le16(tlv->protocol);

		/* Send the command */
		ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

		/* Process response */
		if (ret == UAP_SUCCESS) {
			/* Verify response */
			if (cmd_buf->cmd_code !=
			    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
				printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, uap_le16_to_cpu(tlv->tag));
				free(buffer);
				return UAP_FAILURE;
			}
			/* Print response */
			if (cmd_buf->result == CMD_SUCCESS) {
				if (argc == 0) {
					print_tlv((t_u8 *)tlv,
						  cmd_buf->size -
						  sizeof(apcmdbuf_sys_configure)
						  + BUF_HEADER_SIZE);
				} else {
					if (get_akm_val) {
						get_akm_val = 0;
						/* AKM Suite value from FW */
						tlv_val_16 =
							*(t_u8 *)&akmp_tlv->
							key_mgmt;
						tlv_val_16 |=
							(*
							 ((t_u8 *)&akmp_tlv->
							  key_mgmt + 1) << 8);
					} else
						printf("protocol setting successful\n");
				}
			} else {
				if (argc == 0) {
					printf("ERR:Could not get protocol!\n");
				} else {
					printf("ERR:Could not set protocol!\n");
				}
				ret = UAP_FAILURE;
			}
		} else {
			printf("ERR:Command sending failed!\n");
		}
	} while (resend && (ret == UAP_SUCCESS));

	if (buffer)
		free(buffer);

	return ret;
}

/**
 *  @brief Creates a sys_cfg request for WEP keys settings
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_wep_key [INDEX_0 IS_DEFAULT KEY_0] [INDEX_1 IS_DEFAULT KEY_1] [INDEX_2 IS_DEFAULT KEY_2] [INDEX_3 IS_DEFAULT KEY_3]"
 *
 *   Options: INDEX_* :      0 - KeyIndex is 0
 *                           1 - KeyIndex is 1
 *                           2 - KeyIndex is 2
 *                           3 - KeyIndex is 3
 *            IS_DEFAUL :    0 - KeyIndex is not the default
 *                           1 - KeyIndex is the default transmit key
 *            KEY_* :        Key value
 *            empty - Get current WEP key settings
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_wep_key(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_wep_key *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u8 *tmp_buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int key_len = -1;
	int length = 0;
	int number_of_keys = 0;
	int i = 0;
	int keyindex = -1;
	int is_default = -1;
	char *key = NULL;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_wep_key_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 12) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_wep_key_usage();
		return UAP_FAILURE;
	} else if ((argc > 1) && ((argc % 3) != 0)) {
		printf("ERR:Illegal number of parameters.\n");
		print_sys_cfg_wep_key_usage();
		return UAP_FAILURE;
	} else if (argc > 1) {

		/* Find number of keys provided */
		number_of_keys = argc / 3;
		for (i = 0; i < number_of_keys; i++) {
			if ((ISDIGIT(argv[(3 * i)]) == 0) ||
			    (atoi(argv[(3 * i)]) < 0) ||
			    (atoi(argv[(3 * i)]) > 3)) {
				printf("ERR:Illegal INDEX %s. Must be either '0', '1', '2', or '3'.\n", argv[(3 * i)]);
				print_sys_cfg_wep_key_usage();
				return UAP_FAILURE;
			}
			if ((ISDIGIT(argv[(3 * i) + 1]) == 0) ||
			    (atoi(argv[(3 * i) + 1]) < 0) ||
			    (atoi(argv[(3 * i) + 1]) > 1)) {
				printf("ERR:Illegal IS_DEFAULT %s. Must be either '0', or '1'.\n", argv[(3 * i) + 1]);
				print_sys_cfg_wep_key_usage();
				return UAP_FAILURE;
			}
			if ((strlen(argv[(3 * i) + 2]) != 5) &&
			    (strlen(argv[(3 * i) + 2]) != 10)
			    && (strlen(argv[(3 * i) + 2]) != 13) &&
			    (strlen(argv[(3 * i) + 2]) != 26)) {
				printf("ERR:Incorrect KEY_%d length %d\n",
				       atoi(argv[(3 * i)]),
				       (t_u32)strlen(argv[(3 * i) + 2]));
				print_sys_cfg_wep_key_usage();
				return UAP_FAILURE;
			}
			if ((strlen(argv[(3 * i) + 2]) == 10) ||
			    (strlen(argv[(3 * i) + 2]) == 26)) {
				if (UAP_FAILURE ==
				    ishexstring(argv[(3 * i) + 2])) {
					printf("ERR:Only hex digits are allowed when key length is 10 or 26\n");
					print_sys_cfg_wep_key_usage();
					return UAP_FAILURE;
				}
			}
		}
	} else if (argc == 1) {
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
		    (atoi(argv[0]) > 3)) {
			printf("ERR:Illegal INDEX %s. Must be either '0', '1', '2', or '3'.\n", argv[0]);
			print_sys_cfg_wep_key_usage();
			return UAP_FAILURE;
		}
	}

	/* Initialize the command length */
	if (argc == 0 || argc == 1) {
		if (argc == 0)
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_header);
		else
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_wep_key) - 1;
	} else {
		cmd_len = sizeof(apcmdbuf_sys_configure);
	}
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	if (argc == 0 || argc == 1) {
		cmd_buf->action = ACTION_GET;
		tlv = (tlvbuf_wep_key *)(buffer +
					 sizeof(apcmdbuf_sys_configure));
		tlv->tag = MRVL_WEP_KEY_TLV_ID;
		if (argc == 0)
			tlv->length = 0;
		else {
			tlv->length = 1;
			tlv->key_index = atoi(argv[0]);
		}
		endian_convert_tlv_header_out(tlv);
	} else {
		cmd_buf->action = ACTION_SET;
	}
	/* Add key TLVs */
	for (i = 0; i < number_of_keys; i++) {
		keyindex = atoi(argv[(3 * i)]);
		is_default = atoi(argv[(3 * i) + 1]);
		key = argv[(3 * i) + 2];
		length = strlen(key);
		switch (length) {
		case 5:
		case 10:
			key_len = 5;
			break;
		case 13:
		case 26:
			key_len = 13;
			break;
		default:
			key_len = 0;
			break;
		}
		/* Adjust command buffer */
		tmp_buffer =
			realloc(buffer,
				(cmd_len + sizeof(tlvbuf_wep_key) + key_len));
		if (!tmp_buffer) {
			printf("ERR:Cannot append WEP key configurations TLV!\n");
			free(buffer);
			return UAP_FAILURE;
		} else {
			buffer = tmp_buffer;
			tmp_buffer = NULL;
		}
		/* Locate headers */
		cmd_buf = (apcmdbuf_sys_configure *)buffer;
		tlv = (tlvbuf_wep_key *)(buffer + cmd_len);
		/* Adjust command length */
		cmd_len += (sizeof(tlvbuf_wep_key) + key_len);
		/* Set TLV fields */
		tlv->tag = MRVL_WEP_KEY_TLV_ID;
		tlv->length = 2 + key_len;
		tlv->key_index = (t_u8)keyindex;
		tlv->is_default = (t_u8)is_default;
		/* Check if string or raw */
		switch (length) {
		case 5:
		case 13:
			memcpy(tlv->key, key, length);
			break;
		case 10:
		case 26:
			string2raw(key, tlv->key);
			break;
		default:
			break;
		}
		endian_convert_tlv_header_out(tlv);
	}

	/* Update command length */
	cmd_buf->size = cmd_len;
	if ((argc != 0) && (argc != 1))
		buf_len = cmd_len;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if ((argc != 0) && (argc != 1)) {
				printf("WEP key setting successful\n");
			} else {
				printf("query WEP key setting successful\n");
				tlv = (tlvbuf_wep_key *)(buffer +
							 sizeof
							 (apcmdbuf_sys_configure));
				print_tlv((t_u8 *)tlv,
					  cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}
		} else {
			if ((argc != 0) && (argc != 1))
				printf("ERR:Could not set WEP keys!\n");
			else
				printf("ERR:Could not get WEP keys!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for cipher configurations
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_cipher [PAIRWISE_CIPHER GROUP_CIPHER]"
 *
 *   Options: PAIRWISE_CIPHER : Bit 2 - TKIP
 *                              Bit 3 - AES CCMP
 *            GROUP_CIPHER :    Bit 2 - TKIP
 *                              Bit 3 - AES CCMP
 *            empty - Get current cipher settings
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_cipher(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_cipher *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_cipher_usage();
			return UAP_FAILURE;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc != 0) && (argc != 2)) {
		printf("ERR:wrong arguments.\n");
		print_sys_cfg_cipher_usage();
		return UAP_FAILURE;
	}
	if (argc == 2) {
		if ((ISDIGIT(argv[0]) == 0) || (ISDIGIT(argv[1]) == 0)) {
			print_sys_cfg_cipher_usage();
			return UAP_FAILURE;
		}
		if (atoi(argv[0]) & ~CIPHER_BITMAP) {
			printf("ERR:Illegal PAIRWISE_CIPHER parameter %s.\n",
			       argv[0]);
			print_sys_cfg_cipher_usage();
			return UAP_FAILURE;
		}
		if (atoi(argv[1]) & ~CIPHER_BITMAP) {
			printf("ERR:Illegal GROUP_CIPHER parameter %s.\n",
			       argv[1]);
			print_sys_cfg_cipher_usage();
			return UAP_FAILURE;
		}
		if (is_cipher_valid(atoi(argv[0]), atoi(argv[1])) !=
		    UAP_SUCCESS) {
			printf("ERR:Wrong group and pairwise cipher combination!\n");
			print_sys_cfg_cipher_usage();
			return UAP_FAILURE;
		}
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_cipher);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_cipher *)(buffer + sizeof(apcmdbuf_sys_configure));
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CIPHER_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->pairwise_cipher = (t_u8)atoi(argv[0]);
		tlv->group_cipher = (t_u8)atoi(argv[1]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CIPHER_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				print_cipher(tlv);
			} else {
				printf("cipher setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get cipher parameters!\n");
			} else {
				printf("ERR:Could not set cipher parameters!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for pairwise cipher configurations
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_pwk_cipher [<PROTOCOL>] [PAIRWISE_CIPHER]"
 *
 *   Options: PROTOCOL : Bit 3 - WPA
 *                       Bit 5 - WPA2
 *            PAIRWISE_CIPHER : Bit 2 - TKIP
 *                              Bit 3 - AES CCMP
 *            empty - Get current pairwise cipher settings
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_pwk_cipher(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_pwk_cipher *tlv = NULL;
	HTCap_t htcap;
	t_u16 proto = 0;
	t_u8 *buffer = NULL;
	t_u16 tlv_get_len = 0, protocol = 0, cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_pwk_cipher_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if (is_input_valid(PWK_CIPHER, argc, argv) != UAP_SUCCESS) {
			print_sys_cfg_pwk_cipher_usage();
			return UAP_FAILURE;
		}
		if ((argc == 2) && (atoi(argv[1]) == CIPHER_TKIP)) {
			if (UAP_SUCCESS == get_sys_cfg_protocol(&proto)) {
				/* Ok to have TKIP in mixed mode */
				if (proto != PROTOCOL_WPA2_MIXED) {
					memset(&htcap, 0, sizeof(htcap));
					if (UAP_SUCCESS ==
					    get_sys_cfg_11n(&htcap)) {
						if (htcap.supported_mcs_set[0]) {
							printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
							print_sys_cfg_pwk_cipher_usage
								();
							return UAP_FAILURE;
						}
					}
				}
			}
		}
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_pwk_cipher);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_pwk_cipher *)(buffer + sizeof(apcmdbuf_sys_configure));
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CIPHER_PWK_TLV_ID;
	if (argc < 2) {
		cmd_buf->action = ACTION_GET;
		tlv->length = 0;
		/* Adjust cmd_len for 0 payload */
		cmd_len = cmd_len - (sizeof(tlvbuf_pwk_cipher) - TLVHEADER_LEN);
		if (argc == 1)
			protocol = atoi(argv[0]);
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = sizeof(tlvbuf_pwk_cipher) - TLVHEADER_LEN;
		tlv->protocol = (t_u16)atoi(argv[0]);
		tlv->pairwise_cipher = (t_u8)atoi(argv[1]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->protocol = uap_cpu_to_le16(tlv->protocol);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CIPHER_PWK_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc < 2) {
				tlv_get_len =
					cmd_buf->size -
					sizeof(apcmdbuf_sys_configure) +
					BUF_HEADER_SIZE;
				if ((!tlv_get_len) ||
				    (tlv_get_len % sizeof(tlvbuf_pwk_cipher)) ||
				    (tlv_get_len >
				     (3 * sizeof(tlvbuf_pwk_cipher)))) {
					printf("ERR:Corrupted response!\n");
					free(buffer);
					return UAP_FAILURE;
				}
				if (!proto)
					get_sys_cfg_protocol(&proto);
				if (proto & PROTOCOL_WPA ||
				    proto & PROTOCOL_WPA2)
					printf("Active Protocol = %s\n",
					       (proto == PROTOCOL_WPA)
					       ? "WPA" : (proto ==
							  PROTOCOL_WPA2) ?
					       "WPA2" : "WPA | WPA2");
				while (tlv_get_len > 0) {
					endian_convert_tlv_header_in(tlv);
					tlv->protocol =
						uap_le16_to_cpu(tlv->protocol);
					if ((argc == 1) &&
					    (protocol != tlv->protocol)) {
						tlv_get_len -=
							sizeof
							(tlvbuf_pwk_cipher);
						tlv++;
						continue;
					}
					print_pwk_cipher(tlv);
					tlv_get_len -=
						sizeof(tlvbuf_pwk_cipher);
					tlv++;
				}
			} else {
				printf("protocol and pairwise cipher setting successful\n");
			}
		} else {
			if (argc < 2) {
				printf("ERR:Could not get pairwise cipher parameters!\n");
			} else {
				printf("ERR:Could not set pairwise cipher parameters!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for group cipher configurations
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_gwk_cipher [GROUP_CIPHER]"
 *
 *   Options: GROUP_CIPHER : Bit 2 - TKIP
 *                           Bit 3 - AES CCMP
 *            empty - Get current group cipher settings
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_gwk_cipher(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_gwk_cipher *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_gwk_cipher_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if (is_input_valid(GWK_CIPHER, argc, argv) != UAP_SUCCESS) {
			print_sys_cfg_gwk_cipher_usage();
			return UAP_FAILURE;
		}
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_gwk_cipher);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_gwk_cipher *)(buffer + sizeof(apcmdbuf_sys_configure));
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CIPHER_GWK_TLV_ID;
	tlv->length = 2;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->group_cipher = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CIPHER_GWK_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				print_gwk_cipher(tlv);
			} else {
				printf("group cipher setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get group cipher parameters!\n");
			} else {
				printf("ERR:Could not set group cipher parameters!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for group re-key timer
 *   and sends to the driver
 *
 *   Usage: "Usage : sys_cfg_group_rekey_timer [GROUP_REKEY_TIMER]"
 *
 *   Options: GROUP_REKEY_TIME is represented in seconds
 *            Get current group re-key timer
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_group_rekey_timer(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_group_rekey_timer *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_group_rekey_timer_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc &&
	    (is_input_valid(GROUPREKEYTIMER, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_group_rekey_timer_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_group_rekey_timer);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_group_rekey_timer *)(buffer +
					   sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_GRP_REKEY_TIME_TLV_ID;
	tlv->length = 4;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->group_rekey_time_sec = (t_u32)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->group_rekey_time_sec = uap_cpu_to_le32(tlv->group_rekey_time_sec);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->group_rekey_time_sec = uap_le32_to_cpu(tlv->group_rekey_time_sec);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_GRP_REKEY_TIME_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				if (tlv->group_rekey_time_sec > 0)
					printf("Group rekey time is %d s\n",
					       tlv->group_rekey_time_sec);
				else
					printf("Group rekey time is disabled\n");
			} else {
				printf("group re-key time setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get group re-key time!\n");
			} else {
				printf("ERR:Could not set group re-key time!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for WPA passphrase
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_wpa_passphrase [PASSPHRASE]"
 *           if PASSPHRASE is provided, a 'set' is performed
 *           else a 'get' is performed
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_wpa_passphrase(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_wpa_passphrase *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;

	argc--;
	argv++;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_wpa_passphrase_usage();
		return UAP_FAILURE;
	}
	if ((argc == 1) && (strlen(argv[0]) > MAX_WPA_PASSPHRASE_LENGTH)) {
		printf("ERR:PASSPHRASE too long.\n");
		return UAP_FAILURE;
	}
	if ((argc == 1) && (strlen(argv[0]) < MIN_WPA_PASSPHRASE_LENGTH)) {
		printf("ERR:PASSPHRASE too short.\n");
		return UAP_FAILURE;
	}
	if ((argc == 1) && (strlen(argv[0]) == MAX_WPA_PASSPHRASE_LENGTH)) {
		if (UAP_FAILURE == ishexstring(argv[0])) {
			printf("ERR:Only hex digits are allowed when passphrase's length is 64\n");
			return UAP_FAILURE;
		}
	}
	/* Initialize the command length */
	if (argc == 0)
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_wpa_passphrase) +
			MAX_WPA_PASSPHRASE_LENGTH;
	else
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_wpa_passphrase) + strlen(argv[0]);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_wpa_passphrase *)(buffer +
					sizeof(apcmdbuf_sys_configure));
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_WPA_PASSPHRASE_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = MAX_WPA_PASSPHRASE_LENGTH;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = strlen(argv[0]);
		memcpy(tlv->passphrase, argv[0], tlv->length);
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_WPA_PASSPHRASE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				if (tlv->length > 0)
					printf("WPA passphrase = %s\n",
					       tlv->passphrase);
				else
					printf("WPA passphrase: None\n");
			} else {
				printf("WPA passphrase setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get WPA passphrase!\n");
			} else {
				printf("ERR:Could not set WPA passphrase!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a STA filter request and sends to the driver
 *
 *   Usage: "sta_filter_table <FILTERMODE> <MACADDRESS_LIST>"
 *
 *   Options: FILTERMODE : 0 - Disable filter table
 *                         1 - Allow mac address specified in the allwed list
 *		           2 - Block MAC addresses specified in the  banned list
 *            MACADDRESS_LIST is the list of MAC addresses to be acted upon. Each
 *                         MAC address must be separated with a space. Maximum of
 *                         16 MAC addresses are supported.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sta_filter_table(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_sta_mac_addr_filter *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int i = 0;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sta_filter_table_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc > (MAX_MAC_ONESHOT_FILTER + 1)) {
		printf("ERR:Too many arguments.\n");
		print_sta_filter_table_usage();
		return UAP_FAILURE;
	}
	if (argc > 0) {
		if ((ISDIGIT(argv[0]) == 0) ||
		    ((atoi(argv[0]) < 0) || (atoi(argv[0]) > 2))) {
			printf("ERR:Illegal FILTERMODE parameter %s. Must be either '0', '1', or '2'.\n", argv[1]);
			print_sta_filter_table_usage();
			return UAP_FAILURE;
		}
	}
	/* Initialize the command length */
	if (argc == 0) {
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_sta_mac_addr_filter) +
			(MAX_MAC_ONESHOT_FILTER * ETH_ALEN);
	} else {
		if (argc == 1) {
			cmd_len = sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_sta_mac_addr_filter) +
				(MAX_MAC_ONESHOT_FILTER * ETH_ALEN);
		} else {
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_sta_mac_addr_filter) + (argc -
								      1) *
				ETH_ALEN;
		}
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_sta_mac_addr_filter *)(buffer +
					     sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_STA_MAC_ADDR_FILTER_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->count = MAX_MAC_ONESHOT_FILTER;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->filter_mode = atoi(argv[0]);
		tlv->count = argc - 1;
		if (tlv->count) {
			for (i = 0; i < tlv->count; i++) {
				if ((ret =
				     mac2raw(argv[i + 1],
					     &tlv->mac_address[i *
							       ETH_ALEN])) !=
				    UAP_SUCCESS) {
					printf("ERR: %s Address\n",
					       ret ==
					       UAP_FAILURE ? "Invalid MAC" : ret
					       ==
					       UAP_RET_MAC_BROADCAST ?
					       "Broadcast" : "Multicast");
					print_sta_filter_table_usage();
					free(buffer);
					return UAP_FAILURE;
				}
			}
		} else {
			memset(tlv->mac_address, 0,
			       MAX_MAC_ONESHOT_FILTER * ETH_ALEN);
		}
	}
	if (tlv->count) {
		tlv->length = tlv->count * ETH_ALEN + 2;
	} else {
		tlv->length = MAX_MAC_ONESHOT_FILTER * ETH_ALEN + 2;
	}
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_STA_MAC_ADDR_FILTER_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				print_mac_filter(tlv);
			} else {
				printf("MAC address filter table setting successful!\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get MAC address filter table settings!\n");
			} else {
				printf("ERR:Could not set MAC address filter table settings!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for max station number
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_max_sta_num [STA_NUM]"
 *           if STA_NUM is provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *           STA_NUM should not bigger than 8
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_max_sta_num(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_max_sta_num *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	t_u16 max_sta_num_supported = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_max_sta_num_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && (is_input_valid(MAXSTANUM, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_max_sta_num_usage();
		return UAP_FAILURE;
	}
	if (argc) {
		if (get_max_sta_num_supported(&max_sta_num_supported) ==
		    UAP_FAILURE) {
			return UAP_FAILURE;
		}
		if (atoi(argv[0]) > max_sta_num_supported) {
			printf("ERR: MAX_STA_NUM must be less than %d\n",
			       max_sta_num_supported);
			print_sys_cfg_max_sta_num_usage();
			return UAP_FAILURE;
		}
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_max_sta_num);
	if (argc) {
		cmd_len -= sizeof(tlv->max_sta_num_supported);
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_max_sta_num *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_MAX_STA_CNT_TLV_ID;
	if (argc == 0) {
		tlv->length = 4;
		cmd_buf->action = ACTION_GET;
	} else {
		tlv->length = 2;
		cmd_buf->action = ACTION_SET;
		tlv->max_sta_num_configured = (t_u16)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);
	tlv->max_sta_num_configured =
		uap_cpu_to_le16(tlv->max_sta_num_configured);
	tlv->max_sta_num_supported =
		uap_cpu_to_le16(tlv->max_sta_num_supported);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->max_sta_num_configured =
		uap_le16_to_cpu(tlv->max_sta_num_configured);
	tlv->max_sta_num_supported =
		uap_le16_to_cpu(tlv->max_sta_num_supported);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_MAX_STA_CNT_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("configured max station count = %d\n",
				       tlv->max_sta_num_configured);
				if (tlv->length == 4) {
					printf("max number of stations supported = %d\n", tlv->max_sta_num_supported);
				}
			} else {
				printf("max station number setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get configured max station count and supported max station count!\n");
			} else {
				printf("ERR:Could not set max station number!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for retry limit
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_retry_limit [RETRY_LIMIT]"
 *           if RETRY_LIMIT is provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *           RETRY_LIMIT should not bigger than 14
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_retry_limit(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_retry_limit *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_retry_limit_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && (is_input_valid(RETRYLIMIT, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_retry_limit_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_retry_limit);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_retry_limit *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RETRY_LIMIT_TLV_ID;
	tlv->length = 1;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->retry_limit = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_RETRY_LIMIT_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("retry limit = %d\n", tlv->retry_limit);
			} else {
				printf("retry limit setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get retry limit!\n");
			} else {
				printf("ERR:Could not set retry limit!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for sticky TIM configuration
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_sticky_tim_config [ENABLE] [<DURATION> <STICKY_BIT_MASK>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_sticky_tim_config(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_sticky_tim_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_sticky_tim_config_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc &&
	    (is_input_valid(STICKYTIMCONFIG, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_sticky_tim_config_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_sticky_tim_config);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_sticky_tim_config *)(buffer +
					   sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_STICKY_TIM_CONFIG_TLV_ID;
	tlv->length = sizeof(tlvbuf_sticky_tim_config) - TLVHEADER_LEN;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->enable = (t_u16)atoi(argv[0]);
		if (argc > 1) {
			tlv->duration = (t_u16)atoi(argv[1]);
			tlv->sticky_bitmask = (t_u16)atoi(argv[2]);
		}
	}
	endian_convert_tlv_header_out(tlv);
	tlv->enable = uap_cpu_to_le16(tlv->enable);
	if (argc > 1) {
		tlv->duration = uap_cpu_to_le16(tlv->duration);
		tlv->sticky_bitmask = uap_cpu_to_le16(tlv->sticky_bitmask);
	}
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->enable = uap_le16_to_cpu(tlv->enable);
	if (argc > 1) {
		tlv->duration = uap_le16_to_cpu(tlv->duration);
		tlv->sticky_bitmask = uap_le16_to_cpu(tlv->sticky_bitmask);
	}

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_STICKY_TIM_CONFIG_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("Sticky TIM is %s\n",
				       (tlv->enable ==
					0) ? "disabled" : "enabled");
				printf("Duration = %d beacons\n",
				       tlv->duration);
				printf("Sticky Bitmask = %x\n",
				       tlv->sticky_bitmask);
			} else {
				printf("sticky TIM configuration successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get sticky TIM configuration!\n");
			} else {
				printf("ERR:Could not set sticky TIM configuration!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for sticky TIM station MAC address
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_sticky_tim_sta_mac_addr [CONTROL] [STA_MAC_ADDRESS]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_sticky_tim_sta_mac_addr(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_sticky_tim_sta_mac_addr *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int tlv_get_len = 0;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_sticky_tim_sta_mac_addr_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc &&
	    (is_input_valid(STICKYTIMSTAMACADDR, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_sticky_tim_sta_mac_addr_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_sticky_tim_sta_mac_addr);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_sticky_tim_sta_mac_addr *)(buffer +
						 sizeof
						 (apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_STICKY_TIM_STA_MAC_ADDR_TLV_ID;
	tlv->length = sizeof(tlvbuf_sticky_tim_sta_mac_addr) - TLVHEADER_LEN;
	if (argc < 2) {
		cmd_buf->action = ACTION_GET;
		if (argc == 0) {
			tlv->length = 0;
			cmd_len -=
				sizeof(tlvbuf_sticky_tim_sta_mac_addr) -
				TLVHEADER_LEN;
		}
		if ((argc == 1) &&
		    mac2raw(argv[0], tlv->sta_mac_address) != UAP_SUCCESS) {
			printf("ERR: Invalid MAC address %s \n", argv[0]);
			free(buffer);
			return UAP_FAILURE;
		}
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->control = (t_u16)atoi(argv[0]);
		if (mac2raw(argv[1], tlv->sta_mac_address) != UAP_SUCCESS) {
			printf("ERR: Invalid MAC address %s \n", argv[1]);
			free(buffer);
			return UAP_FAILURE;
		}
	}
	endian_convert_tlv_header_out(tlv);
	tlv->control = uap_cpu_to_le16(tlv->control);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->control = uap_le16_to_cpu(tlv->control);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_STICKY_TIM_STA_MAC_ADDR_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc < 2) {
				tlv_get_len =
					cmd_buf->size -
					sizeof(apcmdbuf_sys_configure) +
					BUF_HEADER_SIZE;
				while (tlv_get_len > 0) {
					printf("station MAC address = ");
					print_mac(tlv->sta_mac_address);
					printf("\ncontrol = %x\n",
					       tlv->control);
					tlv++;
					tlv_get_len -=
						sizeof
						(tlvbuf_sticky_tim_sta_mac_addr);
				}
			} else {
				printf("sticky TIM configuration for given station is successful\n");
			}
		} else {
			if (argc < 2) {
				printf("ERR:Could not get sticky TIM configuration for station(s)!\n");
			} else {
				printf("ERR:Could not set sticky TIM configuration for given station!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for 20/40 coex configuration
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_2040_coex [ENABLE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_2040_coex(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_2040_coex *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_2040_coex_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc && (is_input_valid(COEX2040CONFIG, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_2040_coex_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_2040_coex);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_2040_coex *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_2040_BSS_COEX_CONTROL_TLV_ID;
	tlv->length = sizeof(tlvbuf_2040_coex) - TLVHEADER_LEN;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->enable = (t_u8)atoi(argv[0]);
	}
	endian_convert_tlv_header_out(tlv);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_2040_BSS_COEX_CONTROL_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("20/40 BSS Co-existence is %s\n",
				       (tlv->enable ==
					0) ? "disabled" : "enabled");
			} else {
				printf("20/40 coex configuration successful\n");
			}
		} else {
			printf("ERR:Could not %s 2040 coex configuration!\n",
			       argc ? "set" : "get");
			if (argc)
				printf("20/40 coex configuration is allowed to set only before bss start.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for setting pairwise key handshake timeout
 *  and number of retries and sends to the driver
 *
 *   Usage: "sys_cfg_eapol_pwk_hsk [<TIMEOUT> <RETRIES>]"
 *           If both TIMEOUT value and number of RETRIES are provided,
 *           a 'set' is performed else a 'get' is performed.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_eapol_pwk_hsk(int argc, char *argv[])
{
	int opt;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	t_u8 *tlv_buf = NULL;
	t_u8 *buffer = NULL;
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_eapol_pwk_hsk_timeout *timeout_tlv = NULL;
	tlvbuf_eapol_pwk_hsk_retries *retries_tlv = NULL;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_eapol_pwk_hsk_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check Arguments */
	if (argc && (is_input_valid(EAPOL_PWK_HSK, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_eapol_pwk_hsk_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_eapol_pwk_hsk_timeout)
		+ sizeof(tlvbuf_eapol_pwk_hsk_retries);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Fill the command buffer */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = argc ? ACTION_SET : ACTION_GET;
	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);

	/* Fill Timeout tlv */
	timeout_tlv =
		(tlvbuf_eapol_pwk_hsk_timeout *)(buffer +
						 sizeof
						 (apcmdbuf_sys_configure));
	timeout_tlv->tag = MRVL_EAPOL_PWK_HSK_TIMEOUT_TLV_ID;
	timeout_tlv->length = 4;
	if (argv[0])
		timeout_tlv->pairwise_update_timeout = (t_u32)atoi(argv[0]);
	tlv_buf += sizeof(tlvbuf_eapol_pwk_hsk_timeout);
	endian_convert_tlv_header_out(timeout_tlv);
	timeout_tlv->pairwise_update_timeout =
		uap_cpu_to_le32(timeout_tlv->pairwise_update_timeout);

	/* Fill retries tlv */
	retries_tlv = (tlvbuf_eapol_pwk_hsk_retries *)(tlv_buf);
	retries_tlv->tag = MRVL_EAPOL_PWK_HSK_RETRIES_TLV_ID;
	retries_tlv->length = 4;
	if (argv[1])
		retries_tlv->pwk_retries = (t_u32)atoi(argv[1]);
	endian_convert_tlv_header_out(retries_tlv);
	retries_tlv->pwk_retries = uap_cpu_to_le32(retries_tlv->pwk_retries);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x\n",
			       cmd_buf->cmd_code);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("PWK update timeout and retries setting successful\n");
			} else {
				print_tlv((t_u8 *)tlv_buf,
					  cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}
		} else {
			printf("ERR:Could not %s pwk timeout and retries value!\n", argc ? "set" : "get");
			if (argc)
				printf("PWK timeout and retries are allowed to set only before bss start.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for setting groupwise key handshake timeout
 *  and number of retries and sends to the driver
 *
 *   Usage: "sys_cfg_eapol_gwk_hsk [<TIMEOUT> <RETRIES>]"
 *           If both TIMEOUT value and number of RETRIES are provided,
 *           a 'set' is performed else a 'get' is performed.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_eapol_gwk_hsk(int argc, char *argv[])
{
	int opt;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	t_u8 *tlv_buf = NULL;
	t_u8 *buffer = NULL;
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_eapol_gwk_hsk_timeout *timeout_tlv = NULL;
	tlvbuf_eapol_gwk_hsk_retries *retries_tlv = NULL;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_eapol_gwk_hsk_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check Arguments */
	if (argc && (is_input_valid(EAPOL_GWK_HSK, argc, argv) != UAP_SUCCESS)) {
		print_sys_cfg_eapol_gwk_hsk_usage();
		return UAP_FAILURE;
	}
	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_eapol_gwk_hsk_timeout) +
		sizeof(tlvbuf_eapol_gwk_hsk_retries);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Fill the command buffer */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = argc ? ACTION_SET : ACTION_GET;
	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);

	/* Fill Timeout tlv */
	timeout_tlv =
		(tlvbuf_eapol_gwk_hsk_timeout *)(buffer +
						 sizeof
						 (apcmdbuf_sys_configure));
	timeout_tlv->tag = MRVL_EAPOL_GWK_HSK_TIMEOUT_TLV_ID;
	timeout_tlv->length = 4;
	if (argv[0])
		timeout_tlv->groupwise_update_timeout = (t_u32)atoi(argv[0]);
	tlv_buf += sizeof(tlvbuf_eapol_gwk_hsk_timeout);
	endian_convert_tlv_header_out(timeout_tlv);
	timeout_tlv->groupwise_update_timeout =
		uap_cpu_to_le32(timeout_tlv->groupwise_update_timeout);

	/* Fill retries tlv */
	retries_tlv = (tlvbuf_eapol_gwk_hsk_retries *)(tlv_buf);
	retries_tlv->tag = MRVL_EAPOL_GWK_HSK_RETRIES_TLV_ID;
	retries_tlv->length = 4;
	if (argv[1])
		retries_tlv->gwk_retries = (t_u32)atoi(argv[1]);
	endian_convert_tlv_header_out(retries_tlv);
	retries_tlv->gwk_retries = uap_cpu_to_le32(retries_tlv->gwk_retries);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	tlv_buf = buffer + sizeof(apcmdbuf_sys_configure);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x\n",
			       cmd_buf->cmd_code);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("GWK update timeout and retries setting successful\n");
			} else {
				print_tlv((t_u8 *)tlv_buf,
					  cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}
		} else {
			printf("ERR:Could not %s gwk timeout and retries value!\n", argc ? "set" : "get");
			if (argc)
				printf("GWK timeout and retries are allowed to set only before bss start.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Convert string to integer
 *
 *  @param ptr		A pointer to data buffer
 *  @param chr 		A pointer to return integer
 *  @return      	A pointer to next data field
 */
char *
convert2hex(char *ptr, t_u8 *chr)
{
	t_u8 val;

	for (val = 0; *ptr && isxdigit(*ptr); ptr++) {
		val = (val * 16) + hexc2bin(*ptr);
	}

	*chr = val;

	return ptr;
}

/**
 *  @brief Parse hex data
 *  @param fp 		A pointer to FILE stream
 *  @param dst		A pointer to receive hex data
 *  @return            	length of hex data
 */
int
fparse_for_hex(FILE * fp, t_u8 *dst)
{
	char *ptr;
	t_u8 *dptr;
	char buf[256];

	dptr = dst;
	while (fgets(buf, sizeof(buf), fp)) {
		ptr = buf;

		while (*ptr) {
			/* Skip leading spaces */
			while (*ptr && (isspace(*ptr) || *ptr == '\t'))
				ptr++;

			/* Skip blank lines and lines beginning with '#' */
			if (*ptr == '\0' || *ptr == '#')
				break;

			if (isxdigit(*ptr)) {
				ptr = convert2hex(ptr, dptr++);
			} else {
				/* Invalid character on data line */
				ptr++;
			}
		}
	}

	return (dptr - dst);
}

/**
 *  @brief Creates a cfg_data request
 *   and sends to the driver
 *
 *   Usage: "cfg_data <cfg_data.conf>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_cfg_data(int argc, char *argv[])
{
	apcmdbuf_cfg_data *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	FILE *fp = NULL;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_cfg_data_usage();
			return UAP_FAILURE;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc == 0) || (argc > 2)) {
		printf("ERR:wrong arguments.\n");
		print_cfg_data_usage();
		return UAP_FAILURE;
	} else {
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) != 2)) {
			printf("ERR:Illegal type parameter %s. Must be '2'.\n",
			       argv[0]);
			print_cfg_data_usage();
			return UAP_FAILURE;
		}
	}
	buf = (t_u8 *)malloc(buf_len);
	if (buf == NULL) {
		printf("Error: allocate memory for hostcmd failed\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	cmd_buf = (apcmdbuf_cfg_data *)buf;
	if (argc == 2) {
		/* Check if file exists */
		fp = fopen(argv[1], "r");
		if (fp == NULL) {
			printf("\nERR:Config file can not open %s.\n", argv[1]);
			free(buf);
			return UAP_FAILURE;
		}
		cmd_buf->action = ACTION_SET;
		cmd_buf->data_len = fparse_for_hex(fp, cmd_buf->data);
		fclose(fp);
		if (cmd_buf->data_len > MAX_CFG_DATA_SIZE) {
			printf("ERR: Config file is too big %d\n",
			       cmd_buf->data_len);
			free(buf);
			return UAP_FAILURE;
		}
	} else {
		cmd_buf->action = ACTION_GET;
		cmd_buf->data_len = 0;
	}

	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->type = atoi(argv[0]);
	cmd_buf->type = uap_cpu_to_le16(cmd_buf->type);
	cmd_buf->data_len = uap_cpu_to_le16(cmd_buf->data_len);

	/* Fill the command buffer */
	cmd_len =
		uap_le16_to_cpu(cmd_buf->data_len) + sizeof(apcmdbuf_cfg_data);
	cmd_buf->cmd_code = HostCmd_CMD_CFG_DATA;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		cmd_buf->action = uap_le16_to_cpu(cmd_buf->action);
		cmd_buf->data_len = uap_le16_to_cpu(cmd_buf->data_len);
		if (cmd_buf->action == ACTION_GET) {
			hexdump_data("cfg_data", cmd_buf->data,
				     cmd_buf->data_len, ' ');
		} else
			printf("download cfg data successful\n");
	}
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Show usage information for the sys_cfg_11n
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_11n_usage(void)
{
	printf("\nUsage : sys_cfg_11n [ENABLE] [HTCAP] [AMPDU] [TXBFCAP] [HT_MCS_MAP]\n");
	printf("\nOptions: ENABLE  0 - disable");
	printf("\n                 1 - enable");
	printf("\n         HTCAP: HT Capabilities info");
	printf("\n         AMPDU: A-MPDU Parameter");
	printf("\n         TXBFCAP: TX Beamforming capabilities info");
	printf("\n         HT_MCS_MAP: Bitmap for supported MCS rates\n");
	printf("\n         empty - Get current 802.11n parameters\n");
	return;
}

/**
 *  @brief  Get Ht capability Info from firmware
 *
 *  @param  pHtCap  A pointer to HTCap_t structure
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
get_sys_cfg_11n(HTCap_t *pHtCap)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_htcap_t *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len;
	int ret = UAP_FAILURE;
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_htcap_t);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(cmd_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return -1;
	}
	memset(buffer, 0, cmd_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_htcap_t *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = HT_CAPABILITY_TLV_ID;
	cmd_buf->action = ACTION_GET;
	tlv->length = sizeof(HTCap_t);

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != HT_CAPABILITY_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		/* Copy response */
		if (cmd_buf->result == CMD_SUCCESS) {
			memcpy(pHtCap, &tlv->ht_cap, sizeof(HTCap_t));
			pHtCap->ht_cap_info =
				uap_le16_to_cpu(pHtCap->ht_cap_info);
			pHtCap->tx_bf_cap = uap_le32_to_cpu(pHtCap->tx_bf_cap);
			ret = UAP_SUCCESS;
		} else {
			ret = UAP_FAILURE;
			printf("ERR:Could not get HT capability!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
		ret = UAP_FAILURE;
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief 		Print rx packet coalesc help
 *
 *  @return    	N/A
 */

#ifdef RX_PACKET_COALESCE
void
print_rx_packet_coalesc_help()
{
	printf("\nUSAGE: rxpktcoal_cfg [PKT-THRESHOLD] [TIMEOUT]\n\n");
	printf("OPTIONS:");
	printf("PKT-THRESHOLD: count after which packets would be sent to host. Valid values 1-7\n");
	printf("\tTIMEOUT: Time after which packets would be sent to host Valid values 1-4\n");
	printf("\tCoalescing is disabled if both or either of PKT-THRESHOLD or TIMEOUT is zero\n\n");
	printf("\tEmpty - Get current packet coalescing settings\n");
}

/**
 *  @brief          apcmd rx packet coalesce
 *
 *  @param  argc    no. of arguments,
 *  @param  argv    pointer to arguments
 *
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_rx_pkt_coalesce(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_rx_pkt_coal_t *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_rx_packet_coalesc_help();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if (argc != 2) {
			printf("ERR: Invalid no. of arguments.\n");
			print_rx_packet_coalesc_help();
			return UAP_FAILURE;
		}
		if ((ISDIGIT(argv[0]) == 0) || (ISDIGIT(argv[1]) == 0)) {
			printf("ERR: Invalid arguments %s/%s. Both params must be digits.\n", argv[0], argv[1]);
			print_rx_packet_coalesc_help();
			return UAP_FAILURE;
		}
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_rx_pkt_coal_t *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Initialize the command length */
	if (argc) {
		cmd_len =
			sizeof(apcmdbuf_sys_configure) +
			sizeof(tlvbuf_rx_pkt_coal_t);
		cmd_buf->action = ACTION_SET;
	} else {
		cmd_len = sizeof(apcmdbuf_sys_configure) + TLVHEADER_LEN;
		cmd_buf->action = ACTION_GET;
	}

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	if (!argc) {
		tlv->tag = MRVL_RX_PKT_COAL_TLV_ID;
		tlv->length = 0;
		//endian_convert_tlv_header_out(tlv);
	} else {
		tlv->length = sizeof(tlvbuf_rx_pkt_coal_t) - TLVHEADER_LEN;
		tlv->tag = MRVL_RX_PKT_COAL_TLV_ID;
		tlv->rx_pkt_count = uap_cpu_to_le32(atoi(argv[0]));
		tlv->delay = uap_cpu_to_le16(atoi(argv[1]));
	}

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x\n",
			       cmd_buf->cmd_code);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("Configure RX packet coalesce parameters successful\n");
			} else {

				print_tlv((t_u8 *)tlv, cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}

		} else {
			if (argc)
				printf("ERR:Could not set RX packet coalesce parameters!\n");
			else
				printf("ERR: Could not get RX packet coalesce parameters!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;

}
#endif

/**
 *  @brief Creates a sys_cfg request for 11n parameters
 *   and sends to the driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_11n(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_htcap_t *tlv = NULL;
	tlvbuf_htinfo_t *ht_info_tlv = NULL;
	HTCap_t htcap;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;
	t_u32 supported_mcs_set = 0;
	fw_info fw;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_11n_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc) {
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
		    (atoi(argv[0]) > 1)) {
			printf("ERR: Illegal ENABLE parameter %s. Must be either '0' or '1'.\n", argv[0]);
			print_sys_cfg_11n_usage();
			return UAP_FAILURE;
		}
		if (argc > 5) {
			printf("ERR:wrong arguments.\n");
			print_sys_cfg_11n_usage();
			return UAP_FAILURE;
		}
		if (argc > 1) {
			if (IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
			if ((((t_u16)A2HEXDECIMAL(argv[1])) &
			     (~HT_CAP_CONFIG_MASK)) != HT_CAP_CHECK_MASK) {
				printf("ERR: Invalid HTCAP value!\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
		}
		if (argc > 2) {
			if (IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
			if ((A2HEXDECIMAL(argv[2])) > AMPDU_CONFIG_MASK) {
				printf("ERR: Invalid AMPDU value!\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
		}
		if (argc > 3) {
			if (IS_HEX_OR_DIGIT(argv[3]) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
		}
		if (argc > 4) {
			if (IS_HEX_OR_DIGIT(argv[4]) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				print_sys_cfg_11n_usage();
				return UAP_FAILURE;
			}
			if (0 == get_fw_info(&fw)) {
				/* Check upper nibble of MCS support value
				 * and block MCS_SET_1 when 2X2 is not supported
				 * by the underlying hardware */
				if (((fw.hw_dev_mcs_support & 0xf0) <
				     STREAM_2X2_MASK) &&
				    (A2HEXDECIMAL(argv[4]) & MCS_SET_1_MASK)) {
					printf("ERR: Invalid HT_MCS_MAP\n");
					return UAP_FAILURE;
				}
			}
		}
		memset(&htcap, 0, sizeof(htcap));
		if (UAP_FAILURE == get_sys_cfg_11n(&htcap)) {
			printf("Fail to get 11n parameters from firmware\n");
			return UAP_FAILURE;
		}
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_htcap_t *)(buffer + sizeof(apcmdbuf_sys_configure));
	tlv->tag = HT_CAPABILITY_TLV_ID;

	/* Initialize the command length */
	if (argc) {
		cmd_len =
			sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_htcap_t);
		cmd_buf->action = ACTION_SET;
	} else {
		cmd_len = sizeof(apcmdbuf_sys_configure) + 2 * TLVHEADER_LEN;
		cmd_buf->action = ACTION_GET;
	}

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	if (!argc) {
		tlv->length = 0;
		/* Append HT_INFO TLV only for GET */
		ht_info_tlv =
			(tlvbuf_htinfo_t *)(buffer +
					    sizeof(apcmdbuf_sys_configure) +
					    TLVHEADER_LEN);
		ht_info_tlv->tag = HT_INFO_TLV_ID;
		ht_info_tlv->length = 0;
		endian_convert_tlv_header_out(ht_info_tlv);
	} else {
		tlv->length = sizeof(HTCap_t);
		/* disable 802.11n */
		if (atoi(argv[0]) == 0) {
			if (argc > 1) {
				printf("ERR:wrong arguments.\n");
				print_sys_cfg_11n_usage();
				free(buffer);
				return UAP_FAILURE;
			}
			memcpy(&tlv->ht_cap, &htcap, sizeof(HTCap_t));
			/* disable mcs rate */
			tlv->ht_cap.supported_mcs_set[0] = 0;
			tlv->ht_cap.supported_mcs_set[4] = 0;
			tlv->ht_cap.supported_mcs_set[1] = 0;
		} else {
			t_u8 is_40MHz_supported = 0;
			/* enable 802.11n */
			memcpy(&tlv->ht_cap, &htcap, sizeof(HTCap_t));
			if (0 == get_fw_info(&fw)) {
				if ((fw.hw_dev_mcs_support & 0x0f) >= 2)
					tlv->ht_cap.supported_mcs_set[1] =
						DEFAULT_MCS_SET_1;
			}
			if (argc >= 2) {
				tlv->ht_cap.ht_cap_info =
					DEFAULT_HT_CAP_VALUE &
					~HT_CAP_CONFIG_MASK;
				tlv->ht_cap.ht_cap_info |=
					(t_u16)A2HEXDECIMAL(argv[1]) &
					HT_CAP_CONFIG_MASK;
				tlv->ht_cap.ht_cap_info =
					uap_cpu_to_le16(tlv->ht_cap.
							ht_cap_info);
				is_40MHz_supported =
					tlv->ht_cap.ht_cap_info & MBIT(1);
			}
			/* enable mcs rate */
			tlv->ht_cap.supported_mcs_set[0] = DEFAULT_MCS_SET_0;
			/* enable MCS32 only in case of both 20/40 MHz support */
			if (is_40MHz_supported)
				tlv->ht_cap.supported_mcs_set[4] =
					DEFAULT_MCS_SET_4;
			else
				tlv->ht_cap.supported_mcs_set[4] = 0;

			if (argc >= 3)
				tlv->ht_cap.ampdu_param =
					(t_u8)A2HEXDECIMAL(argv[2]) &
					AMPDU_CONFIG_MASK;
			if (argc >= 4) {
				tlv->ht_cap.tx_bf_cap =
					(t_u32)A2HEXDECIMAL(argv[3]);
				tlv->ht_cap.tx_bf_cap =
					uap_cpu_to_le32(tlv->ht_cap.tx_bf_cap);
			}
			if (argc == 5) {
				supported_mcs_set =
					(t_u32)A2HEXDECIMAL(argv[4]);
				supported_mcs_set =
					uap_cpu_to_le32(supported_mcs_set);
				memcpy(tlv->ht_cap.supported_mcs_set,
				       &supported_mcs_set, sizeof(t_u32));
			}
		}
	}

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x\n",
			       cmd_buf->cmd_code);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("Configure 802.11n parameters successful\n");
			} else {
				print_tlv((t_u8 *)tlv, cmd_buf->size -
					  sizeof(apcmdbuf_sys_configure) +
					  BUF_HEADER_SIZE);
			}

		} else {
			if (argc)
				printf("ERR:Could not set 802.11n parameters!\n");
			else
				printf("ERR: Could not get 802.11n parameters!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Show usage information for the sys_cfg_wmm
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_wmm_usage(void)
{
	printf("\nUsage : sys_cfg_wmm  [qosinfo=<qos_info>] [AC_BE AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_BK AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_VI AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_VO AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\nOptions: qosinfo - 0x80 or 0x00");
	printf("\n         AC_BE - 0, AC_BK - 1");
	printf("\n         AC_VI - 2, AC_VO - 3");
	printf("\n         AIFSN - AIFSN value");
	printf("\n         ECW_MAX: ECW max");
	printf("\n         ECW_MIN: ECW min");
	printf("\n         TX_OP: TXOP Limit");
	printf("\n         empty - Get current wmm parameters\n");
	return;
}

 /**
@brief Show usage information for the sys_cfg_ap_wmm
*   command
*
*  $return         N/A
*/
void
print_sys_cfg_ap_wmm_usage(void)
{
	printf("\nUsage : sys_cfg_wmm  [AC_BE AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_BK AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_VI AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\n                     [AC_VO AIFSN ECW_MAX ECW_MIN TX_OP]\n");
	printf("\nOptions: AC_BE - 0, AC_BK - 1");
	printf("\n         AC_VI - 2, AC_VO - 3");
	printf("\n         AIFSN - AIFSN value");
	printf("\n         ECW_MAX: ECW max");
	printf("\n         ECW_MIN: ECW min");
	printf("\n         TX_OP: TXOP Limit");
	printf("\n         empty - Get current wmm parameters\n");
	return;
}

/**
 *  @brief  Get WMM parameters from firmware
 *
 *  @param  pWmm    A pointer to WmmParameter_t structure
 *  @param  tlv_tag TLV ID
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
get_wmm_parameters(WmmParameter_t *pWmm, t_u16 tlv_tag)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_wmm_para_t *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len;
	t_u8 oui_type[4] = { 0x00, 0x50, 0xF2, 0x02 };

	int ret = UAP_FAILURE;
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_wmm_para_t);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(cmd_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return -1;
	}
	memset(buffer, 0, cmd_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_wmm_para_t *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = tlv_tag;
	cmd_buf->action = ACTION_GET;
	tlv->length = sizeof(WmmParameter_t);
	memcpy(tlv->wmm_para.ouitype, oui_type, sizeof(oui_type));
	tlv->wmm_para.ouisubtype = 1;
	tlv->wmm_para.version = 1;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != tlv_tag) ||
		    (tlv->length != sizeof(WmmParameter_t))) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		/* Copy response */
		if (cmd_buf->result == CMD_SUCCESS) {
			memcpy(pWmm, &tlv->wmm_para, sizeof(WmmParameter_t));
			ret = UAP_SUCCESS;
		} else {
			ret = UAP_FAILURE;
			printf("ERR:Could not get wmm parameters!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
		ret = UAP_FAILURE;
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for wmm parameters
 *   and sends to the driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_wmm(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_wmm_para_t *tlv = NULL;
	WmmParameter_t wmm_para;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int i;
	int ret = UAP_SUCCESS;
	int opt;
	t_u8 ac = 0;
	t_u8 wmm_disable = 1;
	t_u8 qos_info = 0;
	t_u8 flag = 0;
	char str[13];
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_wmm_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	memset(&wmm_para, 0, sizeof(wmm_para));
	memset(str, 0, sizeof(str));
	if (UAP_FAILURE ==
	    get_wmm_parameters(&wmm_para, VENDOR_SPECIFIC_IE_TLV_ID)) {
		printf("Fail to get wmm parameters from firmware\n");
		return UAP_FAILURE;
	}

	if (!argc) {
		printf("wmm parameters:\n");
		printf("\tqos_info = 0x%x\n", wmm_para.qos_info);
		printf("\tBE: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_BE].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_BE].ecw.ecw_max,
		       wmm_para.ac_params[AC_BE].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_BE].tx_op_limit));
		printf("\tBK: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_BK].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_BK].ecw.ecw_max,
		       wmm_para.ac_params[AC_BK].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_BK].tx_op_limit));
		printf("\tVI: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_VI].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_VI].ecw.ecw_max,
		       wmm_para.ac_params[AC_VI].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_VI].tx_op_limit));
		printf("\tVO: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_VO].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_VO].ecw.ecw_max,
		       wmm_para.ac_params[AC_VO].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_VO].tx_op_limit));
		return UAP_SUCCESS;
	}

	if (strlen(argv[0]) > 8) {
		if (strncmp(argv[0], "qosinfo=", 8) == 0) {
			strncpy(str, argv[0], sizeof(str) - 1);
			strncpy(str, strchr(str, '=') + 1, 4);
			qos_info = A2HEXDECIMAL(str);
			if ((qos_info != ENABLE_WMM_PS) &&
			    (qos_info != DISABLE_WMM_PS)) {
				printf("ERR:qos_info must be either 0x80 or 0x00\n");
				print_sys_cfg_wmm_usage();
				return UAP_FAILURE;
			}
			argc -= 1;
			argv += 1;
			flag = 1;
		} else {
			printf("ERR:Invalid argument!\n");
			print_sys_cfg_wmm_usage();
			return UAP_FAILURE;
		}
	}
	/* Check arguments */
	if ((argc != 1) && ((argc > 20) || ((argc > 0) && ((argc % 5) != 0)))) {
		printf("ERR:Illegal number of parameters.\n");
		print_sys_cfg_wmm_usage();
		return UAP_FAILURE;
	}
	for (i = 0; i < argc; i++) {
		if (IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE) {
			printf("ERR: Only Number values are allowed\n");
			print_sys_cfg_wmm_usage();
			return UAP_FAILURE;
		}
		if (A2HEXDECIMAL(argv[i]))
			wmm_disable = 0;
	}
	if ((argc == 1) && A2HEXDECIMAL(argv[0])) {
		printf("ERR: Only 0 is allowed to disable WMM using single parameter.\n");
		print_sys_cfg_wmm_usage();
		return UAP_FAILURE;
	}
	if (argc != 20 && argc != 1)
		wmm_disable = 0;

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_wmm_para_t);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_wmm_para_t *)(buffer + sizeof(apcmdbuf_sys_configure));
	tlv->tag = VENDOR_SPECIFIC_IE_TLV_ID;
	tlv->length = sizeof(WmmParameter_t);

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = ACTION_SET;
	endian_convert_tlv_header_out(tlv);

	memcpy(&tlv->wmm_para, &wmm_para, sizeof(WmmParameter_t));
	if (wmm_disable) {
		/* clear AC parameters to disalbe wmm */
		memset((t_u8 *)tlv->wmm_para.ac_params, 0,
		       sizeof(IEEEtypes_WmmAcParameters_t) * MAX_AC_QUEUES);
	} else {
		if (flag) {
			if (qos_info)
				tlv->wmm_para.qos_info =
					wmm_para.qos_info | qos_info;
			else
				tlv->wmm_para.qos_info =
					wmm_para.qos_info & WMM_PS_MASK;
		}
		for (i = 0; i < (argc / 5); i++) {
			ac = (t_u8)A2HEXDECIMAL(argv[i * 5]);
			if (ac > AC_VO) {
				printf("ERR: Invalid AC queue index, Only support AC_BE, AC_BK, AC_VI, AC_VO\n");
				print_sys_cfg_wmm_usage();
				free(buffer);
				return UAP_FAILURE;
			}
			tlv->wmm_para.ac_params[ac].aci_aifsn.aifsn =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 1]);
			tlv->wmm_para.ac_params[ac].aci_aifsn.aci = (t_u8)ac;
			tlv->wmm_para.ac_params[ac].ecw.ecw_max =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 2]);
			tlv->wmm_para.ac_params[ac].ecw.ecw_min =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 3]);
			tlv->wmm_para.ac_params[ac].tx_op_limit =
				uap_cpu_to_le16((t_u16)
						A2HEXDECIMAL(argv[i * 5 + 4]));
		}
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != VENDOR_SPECIFIC_IE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Configure wmm parameters successful\n");

		} else {
			printf("ERR:Could not set wmm parameters!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
*  @param argc     Number of arguments
*  @param argv     Pointer to the arguments
*  @return         UAP_SUCCESS/UAP_FAILURE
*/
int
apcmd_sys_cfg_ap_wmm(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_wmm_para_t *tlv = NULL;
	WmmParameter_t wmm_para;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int i;
	int opt;
	t_u8 ac = 0;
	t_u8 wmm_reset = 1;
	int ret = UAP_SUCCESS;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_ap_wmm_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;
	memset(&wmm_para, 0, sizeof(wmm_para));
	if (UAP_FAILURE ==
	    get_wmm_parameters(&wmm_para, MRVL_AP_WMM_PARAM_TLV_ID)) {
		printf("Fail to get ap wmm parameters from firmware\n");
		return UAP_FAILURE;
	}

	if (!argc) {
		printf("ap wmm parameters:\n");
		printf("\tBE: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_BE].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_BE].ecw.ecw_max,
		       wmm_para.ac_params[AC_BE].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_BE].tx_op_limit));
		printf("\tBK: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_BK].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_BK].ecw.ecw_max,
		       wmm_para.ac_params[AC_BK].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_BK].tx_op_limit));
		printf("\tVI: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_VI].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_VI].ecw.ecw_max,
		       wmm_para.ac_params[AC_VI].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_VI].tx_op_limit));
		printf("\tVO: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
		       wmm_para.ac_params[AC_VO].aci_aifsn.aifsn,
		       wmm_para.ac_params[AC_VO].ecw.ecw_max,
		       wmm_para.ac_params[AC_VO].ecw.ecw_min,
		       uap_le16_to_cpu(wmm_para.ac_params[AC_VO].tx_op_limit));
		return UAP_SUCCESS;
	}

	/* Check arguments */
	if ((argc != 1) && ((argc > 20) || ((argc > 0) && ((argc % 5) != 0)))) {
		printf("ERR:Illegal number of parameters.\n");
		print_sys_cfg_ap_wmm_usage();
		return UAP_FAILURE;
	}
	for (i = 0; i < argc; i++) {
		if (IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE) {
			printf("ERR: Only Number values are allowed\n");
			print_sys_cfg_ap_wmm_usage();
			return UAP_FAILURE;
		}
		if (A2HEXDECIMAL(argv[i]))
			wmm_reset = 0;
	}
	if ((argc == 1) && A2HEXDECIMAL(argv[0])) {
		printf("ERR: Only 0 is allowed to reset AP WMM using single parameter.\n");
		print_sys_cfg_ap_wmm_usage();
		return UAP_FAILURE;
	}
	if (argc != 20 && argc != 1)
		wmm_reset = 0;

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_wmm_para_t);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_wmm_para_t *)(buffer + sizeof(apcmdbuf_sys_configure));
	tlv->tag = MRVL_AP_WMM_PARAM_TLV_ID;
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = ACTION_SET;
	endian_convert_tlv_header_out(tlv);

	memcpy(&tlv->wmm_para, &wmm_para, sizeof(WmmParameter_t));
	if (wmm_reset) {
		/* clear AC parameters to reset ap wmm */
		memset((t_u8 *)tlv->wmm_para.ac_params, 0,
		       sizeof(IEEEtypes_WmmAcParameters_t) * MAX_AC_QUEUES);
	} else {
		for (i = 0; i < (argc / 5); i++) {
			ac = (t_u8)A2HEXDECIMAL(argv[i * 5]);
			if (ac > AC_VO) {
				printf("ERR: Invalid AC queue index, Only support AC_BE, AC_BK, AC_VI, AC_VO\n");
				print_sys_cfg_ap_wmm_usage();
				free(buffer);
				return UAP_FAILURE;
			}
			tlv->wmm_para.ac_params[ac].aci_aifsn.aifsn =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 1]);
			tlv->wmm_para.ac_params[ac].aci_aifsn.aci = (t_u8)ac;
			tlv->wmm_para.ac_params[ac].ecw.ecw_max =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 2]);
			tlv->wmm_para.ac_params[ac].ecw.ecw_min =
				(t_u8)A2HEXDECIMAL(argv[i * 5 + 3]);
			tlv->wmm_para.ac_params[ac].tx_op_limit =
				uap_cpu_to_le16((t_u16)
						A2HEXDECIMAL(argv[i * 5 + 4]));
		}
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_AP_WMM_PARAM_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Configure ap wmm parameters successful\n");
		} else {
			printf("ERR:Could not set ap wmm parameters!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Show usage information for the sys_cfg_restrict_client_mode
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_restrict_client_mode_usage(void)
{
	printf("\nUsage : sys_cfg_restrict_client_mode [<ENABLE> [MODE_CONFIG]]\n");
	printf("\nOptions:");
	printf("\n        Bit 0: 1 â€?enable restricted client mode");
	printf("\n               0 â€?disable restricted client mode");
	printf("\n        Bits [1-7] : set to 0");
	printf("\n        Bits [8:12]:");
	printf("\n               Bit 8: B only Mode");
	printf("\n               Bit 9: A only Mode");
	printf("\n               Bit 10: G only Mode");
	printf("\n               Bit 11: N only Mode");
	printf("\n               Bit 12: AC only Mode");
	printf("\n        Bits [13:15]: set to 0");
	printf("\n");
	printf("\n        Empty - Get current restricted client mode setting.\n");
	return;
}

/**
 *  @brief Creates a sys_cfg request to Set/Get restricted client mode
 *  and sends to the driver
 *
 *   Usage: "sys_cfg_restrict_client_mode [<ENABLE> [MODE_CONFIG]]"
 *           If arguments are provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_restrict_client_mode(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_restrict_client_mode *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_restrict_client_mode_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc &&
	    is_input_valid(RESTRICT_CLIENT_MODE, argc, argv) != UAP_SUCCESS) {
		print_sys_cfg_restrict_client_mode_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_restrict_client_mode *)(buffer +
					      sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_RESTRICT_CLIENT_MODE_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = 0;
	} else {
		cmd_buf->action = ACTION_SET;
		tlv->length = sizeof(t_u16);
		tlv->mode_config =
			(t_u16)((tlv->
				 mode_config | RESTRICT_CLIENT_MODE_ENABLE_MASK)
				& atoi(argv[0]));
		if (argc == 2) {
			tlv->mode_config |= (t_u16)(A2HEXDECIMAL(argv[1]) << 8);
		}
		tlv->mode_config = uap_cpu_to_le16(tlv->mode_config);
	}
	cmd_buf->size =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_restrict_client_mode);
	cmd_len =
		sizeof(apcmdbuf_sys_configure) +
		sizeof(tlvbuf_restrict_client_mode);
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_RESTRICT_CLIENT_MODE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				tlv->mode_config =
					uap_le16_to_cpu(tlv->mode_config);
				printf("Restricted Client Mode: %s\n",
				       (tlv->
					mode_config &
					RESTRICT_CLIENT_MODE_ENABLE_MASK) ?
				       "Enabled" : "Disabled");
				if (tlv->
				    mode_config &
				    RESTRICT_CLIENT_MODE_ENABLE_MASK) {
					printf("Current client mode : ");
					if (tlv->mode_config & B_ONLY_MASK)
						printf("B Only\n");
					else if (tlv->mode_config & A_ONLY_MASK)
						printf("A Only\n");
					else if (tlv->mode_config & G_ONLY_MASK)
						printf("G Only\n");
					else if (tlv->mode_config & N_ONLY_MASK)
						printf("N Only\n");
					else if (tlv->
						 mode_config & AC_ONLY_MASK)
						printf("AC Only\n");
				}
			} else {
				printf("Restricted client mode setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get restrict client mode setting!\n");
			} else {
				printf("ERR:Could not set restrict client mode setting!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}
