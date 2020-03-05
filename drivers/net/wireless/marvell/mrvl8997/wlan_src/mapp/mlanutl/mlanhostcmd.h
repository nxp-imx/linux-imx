/** @file  mlanhostcmd.h
  *
  * @brief This file contains command structures for mlanutl application
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
     11/26/2008: initial version
************************************************************************/
#ifndef _MLANHOSTCMD_H_
#define _MLANHOSTCMD_H_

/** Find number of elements */
#define NELEMENTS(x) (sizeof(x)/sizeof(x[0]))

/** Size of command buffer */
#define MRVDRV_SIZE_OF_CMD_BUFFER		     (4 * 1024)

/** Host Command ID : Memory access */
#define HostCmd_CMD_MEM_ACCESS                0x0086

/** Pre-Authenticate - 11r only */
#define HostCmd_CMD_802_11_AUTHENTICATE       0x0011

/** Read/Write Mac register */
#define HostCmd_CMD_MAC_REG_ACCESS            0x0019
/** Read/Write BBP register */
#define HostCmd_CMD_BBP_REG_ACCESS            0x001a
/** Read/Write RF register */
#define HostCmd_CMD_RF_REG_ACCESS             0x001b
/** Get TX Power data */
#define HostCmd_CMD_802_11_RF_TX_POWER        0x001e
/** Host Command ID : CAU register access */
#define HostCmd_CMD_CAU_REG_ACCESS            0x00ed

/** Host Command ID : 802.11 BG scan configuration */
#define HostCmd_CMD_802_11_BG_SCAN_CONFIG     0x006b
/** Host Command ID : Configuration data */
#define HostCmd_CMD_CFG_DATA                  0x008f
/** Host Command ID : 802.11 TPC adapt req */
#define HostCmd_CMD_802_11_TPC_ADAPT_REQ      0x0060
/** Host Command ID : 802.11 crypto */
#define HostCmd_CMD_802_11_CRYPTO             0x0078
/** Host Command ID : 802.11 auto Tx */
#define HostCmd_CMD_802_11_AUTO_TX      	0x0082

/** Host Command ID : 802.11 subscribe event */
#define HostCmd_CMD_802_11_SUBSCRIBE_EVENT       0x0075

/** Host Command ID : Channel TRPC config */
#define HostCmd_CMD_CHAN_TRPC_CONFIG                0x00fb

/** TLV  type ID definition */
#define PROPRIETARY_TLV_BASE_ID     0x0100
/** TLV type : Beacon RSSI low */
#define TLV_TYPE_RSSI_LOW           (PROPRIETARY_TLV_BASE_ID + 0x04)	/* 0x0104 */
/** TLV type : Beacon SNR low */
#define TLV_TYPE_SNR_LOW            (PROPRIETARY_TLV_BASE_ID + 0x05)	/* 0x0105 */
/** TLV type : Fail count */
#define TLV_TYPE_FAILCOUNT          (PROPRIETARY_TLV_BASE_ID + 0x06)	/* 0x0106 */
/** TLV type : BCN miss */
#define TLV_TYPE_BCNMISS            (PROPRIETARY_TLV_BASE_ID + 0x07)	/* 0x0107 */
/** TLV type : Beacon RSSI high */
#define TLV_TYPE_RSSI_HIGH          (PROPRIETARY_TLV_BASE_ID + 0x16)	/* 0x0116 */
/** TLV type : Beacon SNR high */
#define TLV_TYPE_SNR_HIGH           (PROPRIETARY_TLV_BASE_ID + 0x17)	/* 0x0117 */
/** TLV type : Auto Tx */
#define TLV_TYPE_AUTO_TX            (PROPRIETARY_TLV_BASE_ID + 0x18)	/* 0x0118 */
/** TLV type :Link Quality */
#define TLV_TYPE_LINK_QUALITY       (PROPRIETARY_TLV_BASE_ID + 0x24)	/* 0x0124 */
/** TLV type : Data RSSI low */
#define TLV_TYPE_RSSI_LOW_DATA      (PROPRIETARY_TLV_BASE_ID + 0x26)	/* 0x0126 */
/** TLV type : Data SNR low */
#define TLV_TYPE_SNR_LOW_DATA       (PROPRIETARY_TLV_BASE_ID + 0x27)	/* 0x0127 */
/** TLV type : Data RSSI high */
#define TLV_TYPE_RSSI_HIGH_DATA     (PROPRIETARY_TLV_BASE_ID + 0x28)	/* 0x0128 */
/** TLV type : Data SNR high */
#define TLV_TYPE_SNR_HIGH_DATA      (PROPRIETARY_TLV_BASE_ID + 0x29)	/* 0x0129 */
/** TLV type: Pre-Beacon Lost */
#define TLV_TYPE_PRE_BEACON_LOST    (PROPRIETARY_TLV_BASE_ID + 0x49)	/* 0x0149 */

/** TLV type : Channel TRPC */
#define TLV_TYPE_CHAN_TRPC              (PROPRIETARY_TLV_BASE_ID + 0x89)	/* 0x0189 */

/** mlan_ioctl_11h_tpc_resp */
typedef struct {
	int status_code;
		     /**< Firmware command result status code */
	int tx_power;/**< Reported TX Power from the TPC Report */
	int link_margin;
		     /**< Reported Link margin from the TPC Report */
	int rssi;    /**< RSSI of the received TPC Report frame */
} __ATTRIB_PACK__ mlan_ioctl_11h_tpc_resp;

/* Define general hostcmd data structure */

/** Convert String to integer */
t_u32 a2hex_or_atoi(char *value);
char *mlan_config_get_line(FILE * fp, char *str, t_s32 size, int *lineno);

int prepare_host_cmd_buffer(FILE * fp, char *cmd_name, t_u8 *buf);
int prepare_hostcmd_regrdwr(t_u32 type, t_u32 offset, t_u32 *value, t_u8 *buf);

#endif /* _MLANHOSTCMD_H_ */
