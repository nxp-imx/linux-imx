
/** @file  moal_eth_ioctl.c
  *
  * @brief This file contains private ioctl functions

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
    01/05/2012: initial version
************************************************************************/

#include "moal_main.h"
#include "moal_eth_ioctl.h"
#include "mlan_ioctl.h"
#if defined(STA_WEXT) || defined(UAP_WEXT)
#include "moal_priv.h"
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#include "moal_cfg80211.h"
#endif
#ifdef UAP_SUPPORT
#include "moal_uap.h"
#endif
#ifdef USB
#include "moal_usb.h"
#endif
#ifdef SDIO
#include "moal_sdio.h"
#endif
#ifdef PCIE
#include "moal_pcie.h"
#endif
#ifdef STA_CFG80211
#include "moal_sta_cfg80211.h"
#endif
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#include "moal_cfg80211_util.h"
#endif
#endif

/********************************************************
			Local Variables
********************************************************/

/** Bands supported in Infra mode */
static t_u16 SupportedInfraBand[] = {
	BAND_B,
	BAND_B | BAND_G,
	BAND_G,
	BAND_GN,
	BAND_B | BAND_G | BAND_GN,
	BAND_G | BAND_GN,
	BAND_A,
	BAND_B | BAND_A,
	BAND_B | BAND_G | BAND_A,
	BAND_G | BAND_A,
	BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN,
	BAND_A | BAND_G | BAND_AN | BAND_GN,
	BAND_A | BAND_AN,
	BAND_GN | BAND_GAC,
	BAND_B | BAND_G | BAND_GN | BAND_GAC,
	BAND_G | BAND_GN | BAND_GAC,
	BAND_GN | BAND_GAC | BAND_GAX,
	BAND_B | BAND_G | BAND_GN | BAND_GAC | BAND_GAX,
	BAND_G | BAND_GN | BAND_GAC | BAND_GAX,
	BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN | BAND_AAC,
	BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN | BAND_AAC | BAND_GAC,
	BAND_A | BAND_G | BAND_AN | BAND_GN | BAND_AAC,
	BAND_A | BAND_AN | BAND_AAC,
	BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN | BAND_AAC | BAND_AAX,
	BAND_A | BAND_B | BAND_G | BAND_AN | BAND_GN | BAND_AAC | BAND_GAC |
		BAND_AAX,
	BAND_A | BAND_G | BAND_AN | BAND_GN | BAND_AAC | BAND_AAX,
	BAND_A | BAND_AN | BAND_AAC | BAND_AAX,
};

/** Bands supported in Ad-Hoc mode */
static t_u8 SupportedAdhocBand[] = {
	BAND_B,
	BAND_B | BAND_G,
	BAND_G,
	BAND_A,
};

/********************************************************
			Global Variables
********************************************************/
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
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
			Local Functions
********************************************************/
/**
 * @brief Parse a string to extract numerical arguments
 *
 * @param pos           Pointer to the arguments string
 * @param data          Pointer to the arguments buffer
 * @param datalen       Length of the arguments buffer
 * @param user_data_len Pointer to the number of arguments extracted
 *
 * @return              MLAN_STATUS_SUCCESS
 */
mlan_status parse_arguments(t_u8 *pos, int *data, int datalen,
			    int *user_data_len)
{
	unsigned int i, j, k;
	char cdata[10];
	int is_hex = 0;

	if (strlen(pos) == 0) {
		*user_data_len = 0;
		return MLAN_STATUS_SUCCESS;
	}

	memset(cdata, 0, sizeof(cdata));
	for (i = 0, j = 0, k = 0; i <= strlen(pos); i++) {
		if ((k == 0) && (i <= (strlen(pos) - 2))) {
			if ((pos[i] == '0') && (pos[i + 1] == 'x')) {
				is_hex = 1;
				i = i + 2;
			}
		}
		if (pos[i] == '\0' || pos[i] == ' ') {
			if (j >= datalen) {
				j++;
				break;
			}
			if (is_hex) {
				data[j] = woal_atox(cdata);
				is_hex = 0;
			} else {
				woal_atoi(&data[j], cdata);
			}
			j++;
			k = 0;
			memset(cdata, 0, sizeof(cdata));
			if (pos[i] == '\0')
				break;
		} else {
			if (k >= sizeof(cdata)) {
				PRINTM(MERROR, "Invalid numerical arguments\n");
				break;
			}
			cdata[k] = pos[i];
			k++;
		}
	}

	*user_data_len = j;
	return MLAN_STATUS_SUCCESS;
}

/** Convert character to integer */
#define CHAR2INT(x) (((x) >= 'A') ? ((x) - 'A' + 10) : ((x) - '0'))
/**
 * @brief Converts a string to hex value
 *
 * @param str      A pointer to the string
 * @param raw      A pointer to the raw data buffer
 * @param raw_size      raw data buffer size
 * @return         Number of bytes read
 **/
int string2raw(unsigned char *str, unsigned char *raw, int raw_size)
{
	int len = (strlen(str) + 1) / 2;
	int i = 0;

	do {
		if (strlen(str) < 2)
			return -1;
		if (!isxdigit(*str) || !isxdigit(*(str + 1)))
			return -1;
		*str = toupper(*str);
		*raw = CHAR2INT(*str) << 4;
		++str;
		*str = toupper(*str);
		*raw |= CHAR2INT(*str);
		++raw;
		i++;
	} while (*++str != '\0' && i < raw_size);
	return len;
}

#if defined(STA_CFG80211) && defined(UAP_CFG80211)
/**
 *  @brief Set wps & p2p ie in AP mode
 *
 *  @param priv         Pointer to priv stucture
 *  @param ie           Pointer to ies data
 *  @param len          Length of data
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_ap_wps_p2p_ie(moal_private *priv, t_u8 *ie, size_t len)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 *pos = ie;
	int ie_len;

	ENTER();

	ie_len = len - 2;
	if (ie_len <= 0) {
		PRINTM(MERROR, "IE len error: %d\n", ie_len);
		ret = -EFAULT;
		goto done;
	}

	/* Android cmd format:
	 * "SET_AP_WPS_P2P_IE 1"  -- beacon IE
	 * "SET_AP_WPS_P2P_IE 2"  -- proberesp IE
	 * "SET_AP_WPS_P2P_IE 4"  -- assocresp IE
	 */
	if (*pos == '1') {
		/* set the beacon wps/p2p ies */
		pos += 2;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_mgmt_frame_ie(
			    priv, pos, ie_len, NULL, 0, NULL, 0, NULL, 0,
			    MGMT_MASK_BEACON_WPS_P2P, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Failed to set beacon wps/p2p ie\n");
			ret = -EFAULT;
			goto done;
		}
	} else if (*pos == '2') {
		/* set the probe resp ies */
		pos += 2;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_mgmt_frame_ie(
			    priv, NULL, 0, pos, ie_len, NULL, 0, NULL, 0,
			    MGMT_MASK_PROBE_RESP, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Failed to set probe resp ie\n");
			ret = -EFAULT;
			goto done;
		}
	} else if (*pos == '4') {
		/* set the assoc resp ies */
		pos += 2;
		if (MLAN_STATUS_SUCCESS !=
		    woal_cfg80211_mgmt_frame_ie(
			    priv, NULL, 0, NULL, 0, pos, ie_len, NULL, 0,
			    MGMT_MASK_ASSOC_RESP, MOAL_IOCTL_WAIT)) {
			PRINTM(MERROR, "Failed to set assoc resp ie\n");
			ret = -EFAULT;
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}
#endif

#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/**
 *  @brief Set miracast mode
 *
 *  @param priv         Pointer to priv stucture
 *  @param pdata        Pointer to cmd buffer
 *  @param len          Length of data
 *
 *  @return             MLAN_STATUS_SUCCESS -- success, otherwise fail
 */
mlan_status woal_set_miracast_mode(moal_private *priv, t_u8 *pdata, size_t len)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u8 *pos = pdata;

	ENTER();
	if (!pos || (len == 0)) {
		PRINTM(MERROR, "%s: Null buf!\n", __func__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	while (!isdigit(*pos) && --len > 0)
		pos++;
	switch (*pos) {
	case '0':
		/* disable miracast mode */
		priv->phandle->miracast_mode = 0;
		break;
	case '1':
		/* Source */
		priv->phandle->miracast_mode = 1;
		break;
	case '2':
		/* Sink */
		priv->phandle->miracast_mode = 2;
		break;
	default:
		PRINTM(MERROR, "%s: Unknown miracast mode (%c)\n",
		       priv->netdev->name, *pos);
		ret = MLAN_STATUS_FAILURE;
		break;
	}
done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 *  @brief Get Driver Version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_get_priv_driver_version(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int len = 0, ret = -1;
	char buf[MLAN_MAX_VER_STR_LEN];

	ENTER();

	if (!respbuf) {
		LEAVE();
		return 0;
	}

	memset(buf, 0, sizeof(buf));

	/* Get version string to local buffer */
	woal_get_version(priv->phandle, buf, sizeof(buf) - 1);
	len = strlen(buf);

	if (len) {
		/* Copy back the retrieved version string */
		PRINTM(MINFO, "MOAL VERSION: %s\n", buf);
		ret = MIN(len, (respbuflen - 1));
		moal_memcpy_ext(priv->phandle, respbuf, buf, ret,
				respbuflen - 1);
	} else {
		ret = -1;
		PRINTM(MERROR, "Get version failed!\n");
	}

	LEAVE();
	return ret;
}
/**
 *  @brief Hostcmd interface from application
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *  @param wait_option  Wait option
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_hostcmd(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen,
		      t_u8 wait_option)
{
	int ret = 0;
	t_u8 *data_ptr;
	t_u32 buf_len = 0;
	HostCmd_Header cmd_header;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_HOSTCMD));
	buf_len = *((t_u32 *)data_ptr);
	moal_memcpy_ext(priv->phandle, &cmd_header, data_ptr + sizeof(buf_len),
			sizeof(HostCmd_Header), sizeof(HostCmd_Header));

	PRINTM(MINFO, "Host command len = %d\n",
	       woal_le16_to_cpu(cmd_header.size));
	if (woal_le16_to_cpu(cmd_header.size) > MRVDRV_SIZE_OF_CMD_BUFFER) {
		LEAVE();
		return -EINVAL;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto error;
	}
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_HOST_CMD;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;
	misc_cfg->param.hostcmd.len = woal_le16_to_cpu(cmd_header.size);
	/* get the whole command */
	moal_memcpy_ext(priv->phandle, misc_cfg->param.hostcmd.cmd,
			data_ptr + sizeof(buf_len), misc_cfg->param.hostcmd.len,
			MRVDRV_SIZE_OF_CMD_BUFFER);

	status = woal_request_ioctl(priv, req, wait_option);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto error;
	}
	ret = misc_cfg->param.hostcmd.len + sizeof(buf_len) + strlen(CMD_NXP) +
	      strlen(PRIV_CMD_HOSTCMD);
	if (ret > respbuflen) {
		ret = -EFAULT;
		goto error;
	}
	moal_memcpy_ext(
		priv->phandle, data_ptr + sizeof(buf_len),
		misc_cfg->param.hostcmd.cmd, misc_cfg->param.hostcmd.len,
		respbuflen - (strlen(CMD_NXP) + strlen(PRIV_CMD_HOSTCMD) +
			      sizeof(buf_len)));
	moal_memcpy_ext(priv->phandle, data_ptr,
			(t_u8 *)&misc_cfg->param.hostcmd.len, sizeof(t_u32),
			sizeof(t_u32));

error:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               configure 11ax HE capability or HE operation
 *
 *
 *  @param priv    Pointer to the mlan_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param len          length used
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_setget_priv_11axcmdcfg(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen, t_u8 wait_option)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ax_cmd_cfg *cfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int header_len = 0, user_data_len = 0;
	int data[3] = {0};
	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ax_cmd_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_11AX_CFG;
	req->action = MLAN_ACT_SET;

	cfg = (mlan_ds_11ax_cmd_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_11AX_CMD_CFG;

	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	PRINTM(MINFO, "data_len=%d,data=%d,%d,%d\n", user_data_len, data[0],
	       data[1], data[2]);

	if (user_data_len > 3 || user_data_len == 0) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EFAULT;
		goto done;
	} else if (user_data_len == 1) {
		req->action = MLAN_ACT_GET;
	}

	switch (data[0]) {
	case MLAN_11AXCMD_CFG_ID_SR_OBSS_PD_OFFSET:
		cfg->sub_id = MLAN_11AXCMD_SR_SUBID;
		cfg->param.sr_cfg.type = MRVL_DOT11AX_OBSS_PD_OFFSET_TLV_ID;
		cfg->param.sr_cfg.len = sizeof(mlan_11axcmdcfg_obss_pd_offset);
		cfg->param.sr_cfg.param.obss_pd_offset.offset[0] = data[1];
		cfg->param.sr_cfg.param.obss_pd_offset.offset[1] = data[2];
		break;
	case MLAN_11AXCMD_CFG_ID_SR_ENABLE:
		cfg->sub_id = MLAN_11AXCMD_SR_SUBID;
		cfg->param.sr_cfg.type = MRVL_DOT11AX_ENABLE_SR_TLV_ID;
		cfg->param.sr_cfg.len = sizeof(mlan_11axcmdcfg_sr_control);
		cfg->param.sr_cfg.param.sr_control.control = data[1];
		break;
	case MLAN_11AXCMD_CFG_ID_BEAM_CHANGE:
		cfg->sub_id = MLAN_11AXCMD_BEAM_SUBID;
		cfg->param.beam_cfg.value = data[1];
		break;
	case MLAN_11AXCMD_CFG_ID_HTC_ENABLE:
		cfg->sub_id = MLAN_11AXCMD_HTC_SUBID;
		cfg->param.htc_cfg.value = data[1];
		break;
	case MLAN_11AXCMD_CFG_ID_TXOP_RTS:
		cfg->sub_id = MLAN_11AXCMD_TXOPRTS_SUBID;
		cfg->param.txop_cfg.rts_thres = data[1];
		break;
	case MLAN_11AXCMD_CFG_ID_TX_OMI:
		cfg->sub_id = MLAN_11AXCMD_TXOMI_SUBID;
		cfg->param.txomi_cfg.omi = data[1];
		break;
	case MLAN_11AXCMD_CFG_ID_OBSSNBRU_TOLTIME:
		cfg->sub_id = MLAN_11AXCMD_OBSS_TOLTIME_SUBID;
		cfg->param.toltime_cfg.tol_time = data[1];
		break;
	default:
		PRINTM(MERROR, "unknown 11axcmd\n");
		ret = -EFAULT;
		goto done;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, respbuf, &req->action,
			sizeof(req->action), sizeof(req->action));
	respbuf += sizeof(req->action);

	cfg = (mlan_ds_11ax_cmd_cfg *)respbuf;
	moal_memcpy_ext(priv->phandle, cfg, req->pbuf,
			sizeof(mlan_ds_11ax_cmd_cfg), respbuflen);

	ret = sizeof(req->action) + sizeof(mlan_ds_11ax_cmd_cfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief             Set/get range ext mode
 *
 *
 *  @param priv    Pointer to the mlan_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param len          length used
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_setget_priv_range_ext(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[1];
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RANGE_EXT);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RANGE_EXT;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid Parameter\n");
			ret = -EFAULT;
			goto done;
		}
		if (data[0] < 0 || data[0] > 2) {
			PRINTM(MERROR,
			       "Invalid Parameter: range_ext mode 0-2\n");
			ret = -EFAULT;
			goto done;
		}
		misc->param.range_ext_mode = (t_u8)data[0];
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.range_ext_mode;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u32 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Custom IE setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_customie(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	t_u8 *data_ptr;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_custom_ie *custom_ie = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_CUSTOMIE));

	custom_ie = (mlan_ds_misc_custom_ie *)data_ptr;
	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	if ((custom_ie->len == 0) ||
	    (custom_ie->len == sizeof(custom_ie->ie_data_list[0].ie_index)))
		ioctl_req->action = MLAN_ACT_GET;
	else
		ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &misc->param.cust_ie, custom_ie,
			sizeof(mlan_ds_misc_custom_ie),
			sizeof(mlan_ds_misc_custom_ie));

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	custom_ie = (mlan_ds_misc_custom_ie *)data_ptr;
	moal_memcpy_ext(priv->phandle, custom_ie, &misc->param.cust_ie,
			sizeof(mlan_ds_misc_custom_ie),
			respbuflen -
				(strlen(CMD_NXP) + strlen(PRIV_CMD_CUSTOMIE)));
	ret = sizeof(mlan_ds_misc_custom_ie);
	if (ioctl_req->status_code == MLAN_ERROR_IOCTL_FAIL) {
		/* send a separate error code to indicate error from driver */
		ret = EFAULT;
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Band and Adhoc-band setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_bandcfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int ret = 0;
	unsigned int i;
	int data[3];
	int user_data_len = 0;
	t_u32 infra_band = 0;
	t_u32 adhoc_band = 0;
	t_u32 adhoc_channel = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_radio_cfg *radio_cfg = NULL;
	mlan_ds_band_cfg *band_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_BANDCFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_BANDCFG),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		LEAVE();
		return -EINVAL;
	}

	if (user_data_len > 0) {
		if (priv->media_connected == MTRUE) {
			LEAVE();
			return -EOPNOTSUPP;
		}
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto error;
	}
	radio_cfg = (mlan_ds_radio_cfg *)req->pbuf;
	radio_cfg->sub_command = MLAN_OID_BAND_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;

	if (user_data_len == 0) {
		/* Get config_bands, adhoc_start_band and adhoc_channel values
		 * from MLAN
		 */
		req->action = MLAN_ACT_GET;
	} else {
		/* To support only <b/bg/bgn/n/aac/gac> */
		infra_band = data[0];
		for (i = 0; i < (sizeof(SupportedInfraBand) /
				 sizeof(SupportedInfraBand[0]));
		     i++)
			if (infra_band == SupportedInfraBand[i])
				break;
		if (i == sizeof(SupportedInfraBand)) {
			ret = -EINVAL;
			goto error;
		}

		/* Set Adhoc band */
		if (user_data_len >= 2) {
			adhoc_band = data[1];
			for (i = 0; i < sizeof(SupportedAdhocBand); i++)
				if (adhoc_band == SupportedAdhocBand[i])
					break;
			if (i == sizeof(SupportedAdhocBand)) {
				ret = -EINVAL;
				goto error;
			}
		}

		/* Set Adhoc channel */
		if (user_data_len >= 3) {
			adhoc_channel = data[2];
			if (adhoc_channel == 0) {
				/* Check if specified adhoc channel is non-zero
				 */
				ret = -EINVAL;
				goto error;
			}
		}
		/* Set config_bands and adhoc_start_band values to MLAN */
		req->action = MLAN_ACT_SET;
		radio_cfg->param.band_cfg.config_bands = infra_band;
		radio_cfg->param.band_cfg.adhoc_start_band = adhoc_band;
		radio_cfg->param.band_cfg.adhoc_channel = adhoc_channel;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto error;
	}

	band_cfg = (mlan_ds_band_cfg *)respbuf;

	moal_memcpy_ext(priv->phandle, band_cfg, &radio_cfg->param.band_cfg,
			sizeof(mlan_ds_band_cfg), respbuflen);

	ret = sizeof(mlan_ds_band_cfg);

error:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11n configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_httxcfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	t_u32 data[2];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_HTTXCFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_HTTXCFG),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_TX;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get 11n tx parameters from MLAN */
		req->action = MLAN_ACT_GET;
		cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_BG;
	} else {
		cfg_11n->param.tx_cfg.httxcap = data[0];
		PRINTM(MINFO, "SET: httxcap:0x%x\n", data[0]);
		cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_BOTH;
		if (user_data_len == 2) {
			if (data[1] != BAND_SELECT_BG &&
			    data[1] != BAND_SELECT_A &&
			    data[1] != BAND_SELECT_BOTH) {
				PRINTM(MERROR, "Invalid band selection\n");
				ret = -EINVAL;
				goto done;
			}
			cfg_11n->param.tx_cfg.misc_cfg = data[1];
			PRINTM(MINFO, "SET: httxcap band:0x%x\n", data[1]);
		}
		/* Update 11n tx parameters in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	data[0] = cfg_11n->param.tx_cfg.httxcap;
	PRINTM(MINFO, "GET: httxcap:0x%x\n", data[0]);

	if (req->action == MLAN_ACT_GET) {
		cfg_11n->param.tx_cfg.httxcap = 0;
		cfg_11n->param.tx_cfg.misc_cfg = BAND_SELECT_A;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
		data[1] = cfg_11n->param.tx_cfg.httxcap;
		PRINTM(MINFO, "GET: httxcap for 5GHz:0x%x\n", data[1]);
	}

	moal_memcpy_ext(priv->phandle, respbuf, data, sizeof(data), respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11n capability information
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_htcapinfo(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int data[2];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	woal_ht_cap_info *ht_cap = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_HTCAPINFO))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_HTCAPINFO),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_HTCAP_CFG;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get 11n tx parameters from MLAN */
		req->action = MLAN_ACT_GET;
		cfg_11n->param.htcap_cfg.misc_cfg = BAND_SELECT_BG;
	} else {
		cfg_11n->param.htcap_cfg.htcap = data[0];
		PRINTM(MINFO, "SET: htcapinfo:0x%x\n", data[0]);
		cfg_11n->param.htcap_cfg.misc_cfg = BAND_SELECT_BOTH;
		if (user_data_len == 2) {
			if (data[1] != BAND_SELECT_BG &&
			    data[1] != BAND_SELECT_A &&
			    data[1] != BAND_SELECT_BOTH) {
				PRINTM(MERROR, "Invalid band selection\n");
				ret = -EINVAL;
				goto done;
			}
			cfg_11n->param.htcap_cfg.misc_cfg = data[1];
			PRINTM(MINFO, "SET: htcapinfo band:0x%x\n", data[1]);
		}
		/* Update 11n tx parameters in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	data[0] = cfg_11n->param.htcap_cfg.htcap;
	PRINTM(MINFO, "GET: htcapinfo for 2.4GHz:0x%x\n", data[0]);

	if (req->action == MLAN_ACT_GET) {
		cfg_11n->param.htcap_cfg.htcap = 0;
		cfg_11n->param.htcap_cfg.misc_cfg = BAND_SELECT_A;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
		data[1] = cfg_11n->param.htcap_cfg.htcap;
		PRINTM(MINFO, "GET: htcapinfo for 5GHz:0x%x\n", data[1]);
	}

	ht_cap = (woal_ht_cap_info *)respbuf;
	ht_cap->ht_cap_info_bg = data[0];
	ht_cap->ht_cap_info_a = data[1];
	ret = sizeof(woal_ht_cap_info);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get add BA parameters
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_addbapara(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int data[5];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	woal_addba *addba = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_ADDBAPARA))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_ADDBAPARA),
				data, ARRAY_SIZE(data), &user_data_len);

		if (user_data_len != ARRAY_SIZE(data)) {
			PRINTM(MERROR, "Invalid number of arguments\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] < 0 || data[0] > MLAN_DEFAULT_BLOCK_ACK_TIMEOUT) {
			PRINTM(MERROR, "Incorrect addba timeout value.\n");
			ret = -EFAULT;
			goto done;
		}
		if (data[1] <= 0 || data[1] > MLAN_AMPDU_MAX_TXWINSIZE ||
		    data[2] <= 0 || data[2] > MLAN_AMPDU_MAX_RXWINSIZE) {
			PRINTM(MERROR, "Incorrect Tx/Rx window size.\n");
			ret = -EFAULT;
			goto done;
		}
		if (data[3] < 0 || data[3] > 1 || data[4] < 0 || data[4] > 1) {
			PRINTM(MERROR, "Incorrect Tx/Rx amsdu.\n");
			ret = -EFAULT;
			goto done;
		}
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_PARAM;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get add BA parameters from MLAN */
		req->action = MLAN_ACT_GET;
	} else {
		cfg_11n->param.addba_param.timeout = data[0];
		cfg_11n->param.addba_param.txwinsize = data[1];
		cfg_11n->param.addba_param.rxwinsize = data[2];
		cfg_11n->param.addba_param.txamsdu = data[3];
		cfg_11n->param.addba_param.rxamsdu = data[4];
		PRINTM(MINFO,
		       "SET: timeout:%d txwinsize:%d rxwinsize:%d txamsdu=%d rxamsdu=%d\n",
		       data[0], data[1], data[2], data[3], data[4]);
		/* Update add BA parameters in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	addba = (woal_addba *)respbuf;

	addba->time_out = cfg_11n->param.addba_param.timeout;
	addba->tx_win_size = cfg_11n->param.addba_param.txwinsize;
	addba->rx_win_size = cfg_11n->param.addba_param.rxwinsize;
	addba->tx_amsdu = cfg_11n->param.addba_param.txamsdu;
	addba->rx_amsdu = cfg_11n->param.addba_param.rxamsdu;
	PRINTM(MINFO,
	       "GET: timeout:%d txwinsize:%d rxwinsize:%d txamsdu=%d, rxamsdu=%d\n",
	       addba->time_out, addba->tx_win_size, addba->rx_win_size,
	       addba->tx_amsdu, addba->rx_amsdu);

	ret = sizeof(woal_addba);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Delete selective BA based on parameters
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_delba(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	t_u32 data[2] = {0xFF, 0xFF};
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ds_11n_delba *del_ba = NULL;
	int ret = 0;
	int user_data_len = 0;
	int header_len = 0;
	t_u8 *mac_pos = NULL;
	t_u8 peer_mac[ETH_ALEN] = {0};
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DELBA);

	if (strlen(respbuf) == header_len) {
		/* Incorrect number of arguments */
		PRINTM(MERROR, "%d: Invalid arguments\n", __LINE__);
		ret = -EINVAL;
		goto done;
	}

	mac_pos = strstr(respbuf + header_len, " ");
	if (mac_pos)
		mac_pos = strstr(mac_pos + 1, " ");
	if (mac_pos) {
#define MAC_STRING_LENGTH 17
		if (strlen(mac_pos + 1) != MAC_STRING_LENGTH) {
			PRINTM(MERROR, "%d: Invalid arguments\n", __LINE__);
			ret = -EINVAL;
			goto done;
		}
		woal_mac2u8(peer_mac, mac_pos + 1);
		*mac_pos = '\0';
	}

	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);

	if (mac_pos)
		user_data_len++;

	if (user_data_len > 3 || (!(data[0] & (DELBA_TX | DELBA_RX))) ||
	    (data[1] != DELBA_ALL_TIDS && !(data[1] <= 7))) {
		/* Incorrect number of arguments */
		PRINTM(MERROR, "%d: Invalid arguments\n", __LINE__);
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	req->req_id = MLAN_IOCTL_11N_CFG;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_DELBA;

	del_ba = &cfg_11n->param.del_ba;
	memset(del_ba, 0, sizeof(mlan_ds_11n_delba));
	del_ba->direction = (t_u8)data[0];
	del_ba->tid = DELBA_ALL_TIDS;
	if (user_data_len > 1)
		del_ba->tid = (t_u8)data[1];
	if (user_data_len > 2)
		moal_memcpy_ext(priv->phandle, del_ba->peer_mac_addr, peer_mac,
				ETH_ALEN, MLAN_MAC_ADDR_LENGTH);

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);

	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	ret = sprintf(respbuf, "OK. BA deleted successfully.\n") + 1;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get the reject addba requst conditions
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_rejectaddbareq(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	t_u32 data[1];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_REJECTADDBAREQ))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_REJECTADDBAREQ),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_REJECT_ADDBA_REQ;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get the reject addba req conditions*/
		req->action = MLAN_ACT_GET;
	} else {
		/* Set the reject addba req conditions */
		cfg_11n->param.reject_addba_req.conditions = data[0];
		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (req->action == MLAN_ACT_GET) {
		sprintf(respbuf, "0x%x",
			cfg_11n->param.reject_addba_req.conditions);
		ret = strlen(respbuf) + 1;
	} else {
		ret = sprintf(respbuf, "OK\n") + 1;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get the addba reject setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action set or get
 *  @param addba_reject A pointer to addba_reject array.
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_ioctl_addba_reject(moal_private *priv, t_u32 action,
				    t_u8 *addba_reject)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_REJECT;
	req->req_id = MLAN_IOCTL_11N_CFG;

	req->action = action;
	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, cfg_11n->param.addba_reject,
				addba_reject,
				sizeof(cfg_11n->param.addba_reject),
				sizeof(cfg_11n->param.addba_reject));
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, addba_reject,
				cfg_11n->param.addba_reject,
				sizeof(cfg_11n->param.addba_reject),
				MAX_NUM_TID);
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get addba prio_tbl
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action set or get
 *  @param aggr_prio_tbl  A pointer to mlan_ds_11n_aggr_prio_tbl.
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_ioctl_aggr_prio_tbl(moal_private *priv, t_u32 action,
				     mlan_ds_11n_aggr_prio_tbl *aggr_prio_tbl)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_AGGR_PRIO_TBL;
	req->req_id = MLAN_IOCTL_11N_CFG;

	req->action = action;
	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &cfg_11n->param.aggr_prio_tbl,
				aggr_prio_tbl,
				sizeof(mlan_ds_11n_aggr_prio_tbl),
				sizeof(mlan_ds_11n_aggr_prio_tbl));
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, aggr_prio_tbl,
				&cfg_11n->param.aggr_prio_tbl,
				sizeof(mlan_ds_11n_aggr_prio_tbl),
				sizeof(mlan_ds_11n_aggr_prio_tbl));
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get addba_param
 *
 *  @param priv         A pointer to moal_private structure
 *  @param action       Action set or get
 *  @param addba_param  A pointer to mlan_ds_11n_addba_param.
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_ioctl_addba_param(moal_private *priv, t_u32 action,
				   mlan_ds_11n_addba_param *addba_param)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_PARAM;
	req->req_id = MLAN_IOCTL_11N_CFG;

	req->action = action;
	if (action == MLAN_ACT_SET)
		moal_memcpy_ext(priv->phandle, &cfg_11n->param.addba_param,
				addba_param, sizeof(mlan_ds_11n_addba_param),
				sizeof(mlan_ds_11n_addba_param));
	/* Send IOCTL request to MLAN */
	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS)
		goto done;
	if (action == MLAN_ACT_GET)
		moal_memcpy_ext(priv->phandle, addba_param,
				&cfg_11n->param.addba_param,
				sizeof(mlan_ds_11n_addba_param),
				sizeof(mlan_ds_11n_addba_param));
done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *   @brief Configuring rx block-ack window size
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise failure
 */
int woal_set_rx_ba_winsize(moal_private *priv, t_u8 *respbuf, int respbuflen)
{
	int data[2];
	t_u8 addba_reject[MAX_NUM_TID];
	mlan_ds_11n_addba_param addba_param;
	int ret = 0;
	int user_data_len = 0;

	ENTER();

	memset((char *)data, 0, sizeof(data));
	if (respbuf && strlen(respbuf) > 0)
		parse_arguments(respbuf, data, ARRAY_SIZE(data),
				&user_data_len);

	if (user_data_len != 2) {
		PRINTM(MERROR, "Invalid arguments for ba_winsize command\n");
		ret = -EINVAL;
		goto done;
	}
	if (data[0] > 7 || data[0] < 0) {
		PRINTM(MERROR, "Invalid tid %d\n", data[0]);
		ret = -EINVAL;
		goto done;
	}
	if (data[1] < 0) {
		PRINTM(MERROR, "Invalid winsize %d\n", data[1]);
		ret = -EINVAL;
		goto done;
	}
	memset(addba_reject, 0, sizeof(addba_reject));
	if (MLAN_STATUS_SUCCESS !=
	    woal_ioctl_addba_reject(priv, MLAN_ACT_GET, addba_reject)) {
		ret = -EFAULT;
		goto done;
	}
	/* disable tx ba */
	if (data[1] == 0) {
		addba_reject[data[0]] = MTRUE;
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_addba_reject(priv, MLAN_ACT_SET, addba_reject))
			ret = -EFAULT;
	} else {
		if (addba_reject[data[0]] == MTRUE) {
			addba_reject[data[0]] = MFALSE;
			if (MLAN_STATUS_SUCCESS !=
			    woal_ioctl_addba_reject(priv, MLAN_ACT_SET,
						    addba_reject)) {
				ret = -EFAULT;
				goto done;
			}
		}
		memset(&addba_param, 0, sizeof(addba_param));
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_addba_param(priv, MLAN_ACT_GET, &addba_param)) {
			ret = -EFAULT;
			goto done;
		}
		if (data[1] != addba_param.rxwinsize) {
			addba_param.rxwinsize = data[1];
			if (MLAN_STATUS_SUCCESS !=
			    woal_ioctl_addba_param(priv, MLAN_ACT_SET,
						   &addba_param))
				ret = -EFAULT;
		}
	}
done:
	LEAVE();
	return ret;
}
/**
 *   @brief Configuring trx block-ack window size
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise failure
 */
int woal_set_tx_ba_winsize(moal_private *priv, t_u8 *respbuf, int respbuflen)
{
	int data[2];
	mlan_ds_11n_aggr_prio_tbl aggr_prio_tbl;
	mlan_ds_11n_addba_param addba_param;
	t_u8 tos_to_tid_inv[] = {0x02, 0x00, 0x01, 0x03,
				 0x04, 0x05, 0x06, 0x07};
	int ret = 0;
	int user_data_len = 0;

	ENTER();

	memset((char *)data, 0, sizeof(data));
	if (respbuf && strlen(respbuf) > 0)
		parse_arguments(respbuf, data, ARRAY_SIZE(data),
				&user_data_len);

	if (user_data_len != 2) {
		PRINTM(MERROR, "Invalid arguments for ba_winsize command\n");
		ret = -EINVAL;
		goto done;
	}
	if (data[0] > 7 || data[0] < 0) {
		PRINTM(MERROR, "Invalid tid %d\n", data[0]);
		ret = -EINVAL;
		goto done;
	}
	if (data[1] < 0) {
		PRINTM(MERROR, "Invalid winsize %d\n", data[1]);
		ret = -EINVAL;
		goto done;
	}
	memset(&aggr_prio_tbl, 0, sizeof(aggr_prio_tbl));
	if (MLAN_STATUS_SUCCESS !=
	    woal_ioctl_aggr_prio_tbl(priv, MLAN_ACT_GET, &aggr_prio_tbl)) {
		ret = -EFAULT;
		goto done;
	}
	/* disable tx ba */
	if (data[1] == 0) {
		if (aggr_prio_tbl.ampdu[data[0]] != 0xff) {
			aggr_prio_tbl.ampdu[data[0]] = 0xff;
			if (MLAN_STATUS_SUCCESS !=
			    woal_ioctl_aggr_prio_tbl(priv, MLAN_ACT_SET,
						     &aggr_prio_tbl))
				ret = -EFAULT;
		}
	} else {
		if (aggr_prio_tbl.ampdu[data[0]] == 0xff) {
			aggr_prio_tbl.ampdu[data[0]] = tos_to_tid_inv[data[0]];
			if (MLAN_STATUS_SUCCESS !=
			    woal_ioctl_aggr_prio_tbl(priv, MLAN_ACT_SET,
						     &aggr_prio_tbl)) {
				ret = -EFAULT;
				goto done;
			}
		}
		memset(&addba_param, 0, sizeof(addba_param));
		if (MLAN_STATUS_SUCCESS !=
		    woal_ioctl_addba_param(priv, MLAN_ACT_GET, &addba_param)) {
			ret = -EFAULT;
			goto done;
		}
		if (data[1] != addba_param.txwinsize) {
			addba_param.txwinsize = data[1];
			if (MLAN_STATUS_SUCCESS !=
			    woal_ioctl_addba_param(priv, MLAN_ACT_SET,
						   &addba_param))
				ret = -EFAULT;
		}
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get aggregation priority table configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_aggrpriotbl(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int data[MAX_NUM_TID * 2], i, j;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_AGGRPRIOTBL))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_AGGRPRIOTBL),
				data, ARRAY_SIZE(data), &user_data_len);

		if (user_data_len != ARRAY_SIZE(data)) {
			PRINTM(MERROR, "Invalid number of arguments\n");
			ret = -EINVAL;
			goto done;
		}
		for (i = 0, j = 0; i < user_data_len; i = i + 2, ++j) {
			if ((data[i] > 7 && data[i] != 0xff) ||
			    (data[i + 1] > 7 && data[i + 1] != 0xff)) {
				PRINTM(MERROR,
				       "Invalid priority, valid value 0-7 or 0xff.\n");
				ret = -EFAULT;
				goto done;
			}
		}
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_AGGR_PRIO_TBL;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get aggr priority table from MLAN */
		req->action = MLAN_ACT_GET;
	} else {
		for (i = 0, j = 0; i < user_data_len; i = i + 2, ++j) {
			cfg_11n->param.aggr_prio_tbl.ampdu[j] = data[i];
			cfg_11n->param.aggr_prio_tbl.amsdu[j] = data[i + 1];
		}
		/* Update aggr priority table in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	for (i = 0, j = 0; i < (MAX_NUM_TID * 2); i = i + 2, ++j) {
		respbuf[i] = cfg_11n->param.aggr_prio_tbl.ampdu[j];
		respbuf[i + 1] = cfg_11n->param.aggr_prio_tbl.amsdu[j];
	}

	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Add BA reject configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_addbareject(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int data[MAX_NUM_TID], i;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_ADDBAREJECT))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_ADDBAREJECT),
				data, ARRAY_SIZE(data), &user_data_len);

		if (user_data_len != ARRAY_SIZE(data)) {
			PRINTM(MERROR, "Invalid number of arguments\n");
			ret = -EINVAL;
			goto done;
		}
		for (i = 0; i < user_data_len; i++) {
			if (data[i] != 0 && data[i] != 1) {
				PRINTM(MERROR,
				       "addba reject only takes argument as 0 or 1\n");
				ret = -EFAULT;
				goto done;
			}
		}
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_ADDBA_REJECT;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (user_data_len == 0) {
		/* Get add BA reject configuration from MLAN */
		req->action = MLAN_ACT_GET;
	} else {
		for (i = 0; i < user_data_len; i++)
			cfg_11n->param.addba_reject[i] = data[i];
		/* Update add BA reject configuration in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	for (i = 0; i < MAX_NUM_TID; i++)
		respbuf[i] = cfg_11n->param.addba_reject[i];

	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11AC configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_vhtcfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data[6];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_ds_11ac_vht_cfg *vhtcfg = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_VHTCFG))) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + strlen(CMD_NXP) + strlen(PRIV_CMD_VHTCFG),
			data, ARRAY_SIZE(data), &user_data_len);

	if ((user_data_len > 6) || (user_data_len < 2)) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11ac = (mlan_ds_11ac_cfg *)req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_VHT_CFG;
	req->req_id = MLAN_IOCTL_11AC_CFG;

	/* Band */
	if ((data[0] < 0) || (data[0] > 2)) {
		PRINTM(MERROR, "Invalid band selection\n");
		ret = -EINVAL;
		goto done;
	} else {
		if (data[0] == BAND_SELECT_BOTH) {
			cfg_11ac->param.vht_cfg.band =
				(BAND_SELECT_BG | BAND_SELECT_A);
		} else {
			cfg_11ac->param.vht_cfg.band = data[0];
		}
		PRINTM(MINFO, "GET/SET: vhtcfg band: 0x%x\n", data[0]);
	}

	/* Tx/Rx */
	if ((data[1] <= 0) || (data[1] > 3)) {
		PRINTM(MERROR, "Invalid Tx/Rx selection\n");
		ret = -EINVAL;
		goto done;
	} else {
		cfg_11ac->param.vht_cfg.txrx = data[1];
		PRINTM(MINFO, "GET/SET: vhtcfg txrx: 0x%x\n", data[1]);
	}

	if (user_data_len == 2) {
		/* GET operation */
		if (data[0] == BAND_SELECT_BOTH) {
			/* if get both bands, get BG first */
			cfg_11ac->param.vht_cfg.band = BAND_SELECT_BG;
		}
		if (priv->bss_role == MLAN_BSS_ROLE_UAP)
			cfg_11ac->param.vht_cfg.txrx = MLAN_RADIO_RX;

		req->action = MLAN_ACT_GET;
	} else {
		if (user_data_len == 3) {
			PRINTM(MERROR, "Invalid number of arguments\n");
			ret = -EINVAL;
			goto done;
		}
		if (user_data_len >= 4) {
			/* BW cfg */
			if ((data[2] < 0) || (data[2] > 1) ||
			    ((data[2] == 1) && (data[0] & BAND_SELECT_BG))) {
				PRINTM(MERROR, "Invalid BW cfg selection\n");
				ret = -EINVAL;
				goto done;
			} else {
				cfg_11ac->param.vht_cfg.bwcfg = data[2];
				PRINTM(MINFO, "SET: vhtcfg bw cfg:0x%x\n",
				       data[2]);
			}

			cfg_11ac->param.vht_cfg.vht_cap_info = data[3];
			PRINTM(MINFO, "SET: vhtcfg vht_cap_info:0x%x\n",
			       data[3]);
		}
		if (user_data_len == 4) {
			data[4] = 0xffffffff;
			data[5] = 0xffffffff;
		}
		if (user_data_len == 5) {
			PRINTM(MERROR, "Invalid number of arguments\n");
			ret = -EINVAL;
			goto done;
		}
		if (user_data_len >= 4) {
			cfg_11ac->param.vht_cfg.vht_tx_mcs = data[4];
			cfg_11ac->param.vht_cfg.vht_rx_mcs = data[5];
		}
		/* Update 11AC parameters in MLAN */
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/* number of vhtcfg entries */
	*respbuf = 1;
	vhtcfg = (mlan_ds_11ac_vht_cfg *)(respbuf + 1);
	moal_memcpy_ext(priv->phandle, vhtcfg, &cfg_11ac->param.vht_cfg,
			sizeof(mlan_ds_11ac_vht_cfg), respbuflen - 1);
	ret = 1 + sizeof(mlan_ds_11ac_vht_cfg);

	if ((req->action == MLAN_ACT_GET) && (data[0] == BAND_SELECT_BOTH)) {
		cfg_11ac->param.vht_cfg.band = BAND_SELECT_A;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
		/* number of vhtcfg entries */
		*respbuf = 2;
		vhtcfg++;
		moal_memcpy_ext(priv->phandle, vhtcfg, &cfg_11ac->param.vht_cfg,
				sizeof(mlan_ds_11ac_vht_cfg),
				respbuflen - 1 - sizeof(mlan_ds_11ac_vht_cfg));
		ret += sizeof(mlan_ds_11ac_vht_cfg);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11AC Operating Mode Notification configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_opermodecfg(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int data[2];
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ac_cfg *cfg_11ac = NULL;
	mlan_ds_11ac_opermode_cfg *opermodecfg = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_OPERMODECFG),
			data, ARRAY_SIZE(data), &user_data_len);

	if ((user_data_len != 0) && (user_data_len != 2)) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	if (user_data_len == 2) {
		/* Channel width */
		if ((data[0] < 1) || (data[0] > 4)) {
			PRINTM(MERROR, "Invalid channel width: 0x%x\n",
			       data[0]);
			ret = -EINVAL;
			goto done;
		}
		/* nss */
		if ((data[1] < 1) || (data[1] > 8)) {
			PRINTM(MERROR, "Invalid nss: 0x%x\n", data[1]);
			ret = -EINVAL;
			goto done;
		}
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ac_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11ac = (mlan_ds_11ac_cfg *)req->pbuf;
	cfg_11ac->sub_command = MLAN_OID_11AC_OPERMODE_CFG;
	req->req_id = MLAN_IOCTL_11AC_CFG;

	if (user_data_len == 0) {
		req->action = MLAN_ACT_GET;
	} else {
		req->action = MLAN_ACT_SET;
		cfg_11ac->param.opermode_cfg.bw = data[0];
		cfg_11ac->param.opermode_cfg.nss = data[1];
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	opermodecfg = (mlan_ds_11ac_opermode_cfg *)respbuf;
	moal_memcpy_ext(priv->phandle, opermodecfg,
			&(cfg_11ac->param.opermode_cfg), sizeof(*opermodecfg),
			respbuflen);
	ret = sizeof(*opermodecfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get 11AC configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_get_priv_datarate(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_rate *rate = NULL;
	mlan_data_rate *data_rate = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	rate = (mlan_ds_rate *)req->pbuf;
	rate->sub_command = MLAN_OID_GET_DATA_RATE;
	req->req_id = MLAN_IOCTL_RATE;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data_rate = (mlan_data_rate *)respbuf;

	moal_memcpy_ext(priv->phandle, data_rate, &rate->param.data_rate,
			sizeof(mlan_data_rate), respbuflen);

	ret = sizeof(mlan_data_rate);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get tx rate configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_txratecfg(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	t_u32 data[4];
	mlan_ioctl_req *req = NULL;
	mlan_ds_rate *rate = NULL;
	woal_tx_rate_cfg *ratecfg = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	txrate_setting *rate_setting = NULL;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_TXRATECFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_TXRATECFG),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 4) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_rate));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_RATE;
	rate = (mlan_ds_rate *)req->pbuf;
	rate->sub_command = MLAN_OID_RATE_CFG;
	rate->param.rate_cfg.rate_type = MLAN_RATE_INDEX;

	if (user_data_len == 0) {
		/* Get operation */
		req->action = MLAN_ACT_GET;
	} else {
		/* Set operation */
		req->action = MLAN_ACT_SET;
		/* format */
		if ((data[0] != AUTO_RATE) && (data[0] >= 4)) {
			PRINTM(MERROR, "Invalid format selection\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] == AUTO_RATE) {
			/* auto */
			rate->param.rate_cfg.is_rate_auto = 1;
		} else {
			/* fixed rate */
			PRINTM(MINFO, "SET: txratefg format: 0x%x\n", data[0]);
			if ((data[0] != AUTO_RATE) &&
			    (data[0] > MLAN_RATE_FORMAT_HE)) {
				PRINTM(MERROR, "Invalid format selection\n");
				ret = -EINVAL;
				goto done;
			}
			rate->param.rate_cfg.rate_format = data[0];

			if (user_data_len >= 2) {
				PRINTM(MINFO, "SET: txratefg index: 0x%x\n",
				       data[1]);
				/* sanity check */
				if (((data[0] == MLAN_RATE_FORMAT_LG) &&
				     (data[1] > MLAN_RATE_INDEX_OFDM7)) ||
				    ((data[0] == MLAN_RATE_FORMAT_HT) &&
				     (data[1] != 32) && (data[1] > 15)) ||
				    ((data[0] == MLAN_RATE_FORMAT_VHT) &&
				     (data[1] > MLAN_RATE_INDEX_MCS9)) ||
				    ((data[0] == MLAN_RATE_FORMAT_HE) &&
				     (data[1] > MLAN_RATE_INDEX_MCS11))) {
					PRINTM(MERROR,
					       "Invalid index selection\n");
					ret = -EINVAL;
					goto done;
				}

				PRINTM(MINFO, "SET: txratefg index: 0x%x\n",
				       data[1]);
				rate->param.rate_cfg.rate = data[1];
			}

			if (data[0] == 2 || data[0] == 3) {
				PRINTM(MINFO, "SET: txratefg nss: 0x%x\n",
				       data[2]);
				/* NSS is supported up to 2 */
				if ((data[2] <= 0) || (data[2] >= 3)) {
					PRINTM(MERROR,
					       "Invalid nss selection\n");
					ret = -EINVAL;
					goto done;
				}
				rate->param.rate_cfg.nss = data[2];
			}
			if (user_data_len == 4) {
				rate->param.rate_cfg.rate_setting =
					data[3] & ~0x0C00;
				PRINTM(MIOCTL,
				       "SET: txratefg HE Rate Setting: 0x%x\n",
				       data[3]);

/* HE Preamble type */
#define HE_SU_PREAMBLE 0
#define HE_ER_PREAMBLE 1

/* HE ER SU Type */
#define HE_ER_SU_BANDWIDTH_TONE242 0
#define HE_ER_SU_BANDWIDTH_TONE106 1

				rate_setting = (txrate_setting *)&data[3];

				if (data[0] == MLAN_RATE_FORMAT_HE) {
					if (rate_setting->preamble ==
					    HE_ER_PREAMBLE) {
						if (rate_setting->bandwidth ==
						    HE_ER_SU_BANDWIDTH_TONE242) {
							if ((data[1] >
							     MLAN_RATE_INDEX_MCS4) ||
							    data[2] >
								    MLAN_RATE_NSS1) {
								PRINTM(MERROR,
								       "Invalid rate and MCS or NSS configuration for 242 tone\n");
								ret = -EINVAL;
								goto done;
							}
						} else if (rate_setting
								   ->bandwidth ==
							   HE_ER_SU_BANDWIDTH_TONE106) {
							if ((data[1] !=
							     MLAN_RATE_INDEX_MCS0) ||
							    data[2] !=
								    MLAN_RATE_NSS1) {
								PRINTM(MERROR,
								       "Invalid rate and MCS or NSS configuration\n for 106 tone");
								ret = -EINVAL;
								goto done;
							}
						} else {
							PRINTM(MERROR,
							       "Invalid Bandwidth for HE ER Preamble\n");
							ret = -EINVAL;
							goto done;
						}
					}
					if (rate_setting->dcm) {
						if ((data[1] ==
						     MLAN_RATE_INDEX_MCS2) ||
						    (data[1] >
						     MLAN_RATE_INDEX_MCS4)) {
							PRINTM(MERROR,
							       "Invalid MCS configuration if DCM is supported\n");
							ret = -EINVAL;
							goto done;
						}
					}
				}
			} else {
				rate->param.rate_cfg.rate_setting = 0xffff;
			}
		}
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	ratecfg = (woal_tx_rate_cfg *)respbuf;
	if (rate->param.rate_cfg.is_rate_auto == MTRUE) {
		ratecfg->rate_format = 0xFF;
	} else {
		/* fixed rate */
		ratecfg->rate_format = rate->param.rate_cfg.rate_format;
		ratecfg->rate_index = rate->param.rate_cfg.rate;
		if (rate->param.rate_cfg.rate_format == MLAN_RATE_FORMAT_VHT ||
		    rate->param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HE)
			ratecfg->nss = rate->param.rate_cfg.nss;
		ratecfg->rate_setting = rate->param.rate_cfg.rate_setting;
	}

	ret = sizeof(woal_tx_rate_cfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_SUPPORT) || defined(UAP_SUPPORT)
/**
 *  @brief Get statistics information
 *
 *  @param priv         A pointer to moal_private structure
 *  @param wait_option  Wait option
 *  @param stats        A pointer to mlan_ds_get_stats structure
 *
 *  @return             MLAN_STATUS_SUCCESS/MLAN_STATUS_PENDING -- success,
 * otherwise fail
 */
mlan_status woal_get_stats_info(moal_private *priv, t_u8 wait_option,
				mlan_ds_get_stats *stats)
{
	int ret = 0;
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA)
		info->sub_command = MLAN_OID_GET_STATS;
	else if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP)
		info->sub_command = MLAN_OID_GET_UAP_STATS_LOG;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, wait_option);
	if (status == MLAN_STATUS_SUCCESS) {
		if (stats)
			moal_memcpy_ext(priv->phandle, stats,
					&info->param.stats,
					sizeof(mlan_ds_get_stats),
					sizeof(mlan_ds_get_stats));
#if defined(STA_WEXT) || defined(UAP_WEXT)
		priv->w_stats.discard.fragment = info->param.stats.fcs_error;
		priv->w_stats.discard.retries = info->param.stats.retry;
		priv->w_stats.discard.misc = info->param.stats.ack_failure;
#endif
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return status;
}

/**
 *  @brief Get wireless stats information
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_get_priv_getlog(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_get_stats *stats;
	ENTER();

	if (respbuflen < sizeof(*stats)) {
		PRINTM(MERROR, "Get log: respbuflen (%d) too small!",
		       (int)respbuflen);
		ret = -EFAULT;
		goto done;
	}
	stats = (mlan_ds_get_stats *)respbuf;
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_stats_info(priv, MOAL_IOCTL_WAIT, stats)) {
		PRINTM(MERROR, "Get log: Failed to get stats info!");
		ret = -EFAULT;
		goto done;
	}

	if (priv->phandle->fw_getlog_enable)
		ret = sizeof(mlan_ds_get_stats);
	else
		ret = sizeof(mlan_ds_get_stats_org);

done:
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Set/Get esupplicant mode configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_esuppmode(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	t_u32 data[3];
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	woal_esuppmode_cfg *esupp_mode = NULL;
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->phandle->card_info->embedded_supp) {
		PRINTM(MERROR, "Not supported cmd on this card\n");
		ret = -EOPNOTSUPP;
		goto done;
	}
	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_ESUPPMODE))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_ESUPPMODE),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len >= 4 || user_data_len == 1 || user_data_len == 2) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_SEC_CFG;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_ESUPP_MODE;

	if (user_data_len == 0) {
		/* Get operation */
		req->action = MLAN_ACT_GET;
	} else {
		/* Set operation */
		req->action = MLAN_ACT_SET;
		/* RSN mode */
		sec->param.esupp_mode.rsn_mode = data[0];
		/* Pairwise cipher */
		sec->param.esupp_mode.act_paircipher = (data[1] & 0xFF);
		/* Group cipher */
		sec->param.esupp_mode.act_groupcipher = (data[2] & 0xFF);
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	esupp_mode = (woal_esuppmode_cfg *)respbuf;
	esupp_mode->rsn_mode =
		(t_u16)((sec->param.esupp_mode.rsn_mode) & 0xFFFF);
	esupp_mode->pairwise_cipher =
		(t_u8)((sec->param.esupp_mode.act_paircipher) & 0xFF);
	esupp_mode->group_cipher =
		(t_u8)((sec->param.esupp_mode.act_groupcipher) & 0xFF);

	ret = sizeof(woal_esuppmode_cfg);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get esupplicant passphrase configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_setget_priv_passphrase(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	int ret = 0, action = -1, i = 0;
	char *begin, *end, *opt;
	t_u16 len = 0;
	t_u8 zero_mac[] = {0, 0, 0, 0, 0, 0};
	t_u8 *mac = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->phandle->card_info->embedded_supp) {
		PRINTM(MERROR, "Not supported cmd on this card\n");
		ret = -EOPNOTSUPP;
		goto done;
	}

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_PASSPHRASE))) {
		PRINTM(MERROR, "No arguments provided\n");
		ret = -EINVAL;
		goto done;
	}

	/* Parse the buf to get the cmd_action */
	begin = respbuf + strlen(CMD_NXP) + strlen(PRIV_CMD_PASSPHRASE);
	end = woal_strsep(&begin, ';', '/');
	if (end)
		action = woal_atox(end);
	if (action < 0 || action > 2 || end[1] != '\0') {
		PRINTM(MERROR, "Invalid action argument %s\n", end);
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_SEC_CFG;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_PASSPHRASE;
	if (action == 0)
		req->action = MLAN_ACT_GET;
	else
		req->action = MLAN_ACT_SET;

	while (begin) {
		end = woal_strsep(&begin, ';', '/');
		opt = woal_strsep(&end, '=', '/');
		if (!opt || !end || !end[0]) {
			PRINTM(MERROR, "Invalid option\n");
			ret = -EINVAL;
			break;
		} else if (!strnicmp(opt, "ssid", strlen(opt))) {
			if (strlen(end) > MLAN_MAX_SSID_LENGTH) {
				PRINTM(MERROR,
				       "SSID length exceeds max length\n");
				ret = -EFAULT;
				break;
			}
			sec->param.passphrase.ssid.ssid_len = strlen(end);
			moal_memcpy_ext(priv->phandle,
					sec->param.passphrase.ssid.ssid, end,
					strlen(end), MLAN_MAX_SSID_LENGTH);
			PRINTM(MINFO, "ssid=%s, len=%d\n",
			       sec->param.passphrase.ssid.ssid,
			       (int)sec->param.passphrase.ssid.ssid_len);
		} else if (!strnicmp(opt, "bssid", strlen(opt))) {
			woal_mac2u8((t_u8 *)&sec->param.passphrase.bssid, end);
		} else if (!strnicmp(opt, "psk", strlen(opt)) &&
			   req->action == MLAN_ACT_SET) {
			if (strlen(end) != MLAN_PMK_HEXSTR_LENGTH) {
				PRINTM(MERROR, "Invalid PMK length\n");
				ret = -EINVAL;
				break;
			}
			woal_ascii2hex(
				(t_u8 *)(sec->param.passphrase.psk.pmk.pmk),
				end, MLAN_PMK_HEXSTR_LENGTH / 2);
			sec->param.passphrase.psk_type = MLAN_PSK_PMK;
		} else if (!strnicmp(opt, "passphrase", strlen(opt)) &&
			   req->action == MLAN_ACT_SET) {
			if (strlen(end) < MLAN_MIN_PASSPHRASE_LENGTH ||
			    strlen(end) > MLAN_MAX_PASSPHRASE_LENGTH) {
				PRINTM(MERROR,
				       "Invalid length for passphrase\n");
				ret = -EINVAL;
				break;
			}
			sec->param.passphrase.psk_type = MLAN_PSK_PASSPHRASE;
			moal_memcpy_ext(
				priv->phandle,
				sec->param.passphrase.psk.passphrase.passphrase,
				end,
				sizeof(sec->param.passphrase.psk.passphrase
					       .passphrase),
				sizeof(sec->param.passphrase.psk.passphrase
					       .passphrase));
			sec->param.passphrase.psk.passphrase.passphrase_len =
				strlen(end);
			PRINTM(MINFO, "passphrase=%s, len=%d\n",
			       sec->param.passphrase.psk.passphrase.passphrase,
			       (int)sec->param.passphrase.psk.passphrase
				       .passphrase_len);
		} else if (!strnicmp(opt, "sae_password", strlen(opt)) &&
			   req->action == MLAN_ACT_SET) {
			if (strlen(end) < MLAN_MIN_SAE_PASSWORD_LENGTH ||
			    strlen(end) > MLAN_MAX_SAE_PASSWORD_LENGTH) {
				PRINTM(MERROR,
				       "Invalid length for sae password\n");
				ret = -EINVAL;
				break;
			}
			sec->param.passphrase.psk_type = MLAN_PSK_SAE_PASSWORD;
			moal_memcpy_ext(
				priv->phandle,
				sec->param.passphrase.psk.sae_password
					.sae_password,
				end,
				sizeof(sec->param.passphrase.psk.sae_password
					       .sae_password),
				sizeof(sec->param.passphrase.psk.sae_password
					       .sae_password));
			sec->param.passphrase.psk.sae_password.sae_password_len =
				strlen(end);
			PRINTM(MINFO, "sae_password=%s, len=%d\n",
			       sec->param.passphrase.psk.sae_password
				       .sae_password,
			       (int)sec->param.passphrase.psk.sae_password
				       .sae_password_len);
		} else {
			PRINTM(MERROR, "Invalid option %s\n", opt);
			ret = -EINVAL;
			break;
		}
	}
	if (ret)
		goto done;

	if (action == 2)
		sec->param.passphrase.psk_type = MLAN_PSK_CLEAR;
	else if (action == 0)
		sec->param.passphrase.psk_type = MLAN_PSK_QUERY;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	memset(respbuf, 0, respbuflen);
	if (sec->param.passphrase.ssid.ssid_len) {
		len += sprintf(respbuf + len, "ssid:");
		moal_memcpy_ext(priv->phandle, respbuf + len,
				sec->param.passphrase.ssid.ssid,
				sec->param.passphrase.ssid.ssid_len,
				respbuflen - len);
		len += sec->param.passphrase.ssid.ssid_len;
		len += sprintf(respbuf + len, " ");
	}
	if (memcmp(&sec->param.passphrase.bssid, zero_mac, sizeof(zero_mac))) {
		mac = (t_u8 *)&sec->param.passphrase.bssid;
		len += sprintf(respbuf + len, "bssid:");
		for (i = 0; i < ETH_ALEN - 1; ++i)
			len += sprintf(respbuf + len, "%02x:", mac[i]);
		len += sprintf(respbuf + len, "%02x ", mac[i]);
	}
	if (sec->param.passphrase.psk_type == MLAN_PSK_PMK) {
		len += sprintf(respbuf + len, "psk:");
		for (i = 0; i < MLAN_MAX_KEY_LENGTH; ++i)
			len += sprintf(respbuf + len, "%02x",
				       sec->param.passphrase.psk.pmk.pmk[i]);
		len += sprintf(respbuf + len, "\n");
	}
	if (sec->param.passphrase.psk_type == MLAN_PSK_PASSPHRASE)
		len += sprintf(respbuf + len, "passphrase:%s\n",
			       sec->param.passphrase.psk.passphrase.passphrase);
	if (sec->param.passphrase.psk_type == MLAN_PSK_SAE_PASSWORD)
		len += sprintf(
			respbuf + len, "sae_password:%s\n",
			sec->param.passphrase.psk.sae_password.sae_password);

	ret = len;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Deauthenticate
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_deauth(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	t_u8 mac[ETH_ALEN];

	ENTER();

	if (strlen(respbuf) > (strlen(CMD_NXP) + strlen(PRIV_CMD_DEAUTH))) {
		/* Deauth mentioned BSSID */
		woal_mac2u8(mac, respbuf + strlen(CMD_NXP) +
					 strlen(PRIV_CMD_DEAUTH));
		if (MLAN_STATUS_SUCCESS !=
		    woal_disconnect(priv, MOAL_IOCTL_WAIT, mac,
				    DEF_DEAUTH_REASON_CODE)) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		if (MLAN_STATUS_SUCCESS !=
		    woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL,
				    DEF_DEAUTH_REASON_CODE))
			ret = -EFAULT;
	}

done:
	LEAVE();
	return ret;
}

#ifdef UAP_SUPPORT
/**
 *  @brief uap station deauth ioctl handler
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_ap_deauth(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	t_u8 *data_ptr;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_deauth_param deauth_param;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_AP_DEAUTH));
	memset(&deauth_param, 0, sizeof(mlan_deauth_param));
	moal_memcpy_ext(priv->phandle, &deauth_param, data_ptr,
			sizeof(mlan_deauth_param), sizeof(mlan_deauth_param));

	PRINTM(MIOCTL, "ioctl deauth station: " MACSTR ", reason=%d\n",
	       MAC2STR(deauth_param.mac_addr), deauth_param.reason_code);

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)ioctl_req->pbuf;

	bss->sub_command = MLAN_OID_UAP_DEAUTH_STA;
	ioctl_req->req_id = MLAN_IOCTL_BSS;
	ioctl_req->action = MLAN_ACT_SET;

	moal_memcpy_ext(priv->phandle, &bss->param.deauth_param, &deauth_param,
			sizeof(mlan_deauth_param), sizeof(mlan_deauth_param));
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(
		priv->phandle, data_ptr, &ioctl_req->status_code, sizeof(t_u32),
		respbuflen - (strlen(CMD_NXP) + strlen(PRIV_CMD_AP_DEAUTH)));
	ret = sizeof(t_u32);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief uap get station list handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_priv_get_sta_list(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_get_info *info = NULL;
	mlan_ds_sta_list *sta_list = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Allocate an IOCTL request buffer */
	ioctl_req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_get_info));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	info = (mlan_ds_get_info *)ioctl_req->pbuf;
	info->sub_command = MLAN_OID_UAP_STA_LIST;
	ioctl_req->req_id = MLAN_IOCTL_GET_INFO;
	ioctl_req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	sta_list = (mlan_ds_sta_list *)(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_GET_STA_LIST));
	moal_memcpy_ext(
		priv->phandle, sta_list, &info->param.sta_list,
		sizeof(mlan_ds_sta_list),
		respbuflen - (strlen(CMD_NXP) + strlen(PRIV_CMD_GET_STA_LIST)));
	ret = sizeof(mlan_ds_sta_list);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}

/**
 *  @brief uap bss_config handler
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @return         0 --success, otherwise fail
 */
static int woal_priv_bss_config(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	t_u32 action = 0;
	int offset = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	offset = strlen(CMD_NXP) + strlen(PRIV_CMD_BSS_CONFIG);
	moal_memcpy_ext(priv->phandle, (u8 *)&action, respbuf + offset,
			sizeof(action), sizeof(action));
	offset += sizeof(action);

	/* Allocate an IOCTL request buffer */
	ioctl_req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(
		sizeof(mlan_ds_bss));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	bss = (mlan_ds_bss *)ioctl_req->pbuf;
	bss->sub_command = MLAN_OID_UAP_BSS_CONFIG;
	ioctl_req->req_id = MLAN_IOCTL_BSS;
	if (action == 1) {
		ioctl_req->action = MLAN_ACT_SET;
		/* Get the BSS config from user */
		moal_memcpy_ext(priv->phandle, &bss->param.bss_config,
				respbuf + offset, sizeof(mlan_uap_bss_param),
				sizeof(mlan_uap_bss_param));
	} else {
		ioctl_req->action = MLAN_ACT_GET;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (ioctl_req->action == MLAN_ACT_GET) {
		moal_memcpy_ext(priv->phandle, respbuf + offset,
				&bss->param.bss_config,
				sizeof(mlan_uap_bss_param),
				respbuflen - (t_u32)offset);
	}
	ret = sizeof(mlan_uap_bss_param);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);
	LEAVE();
	return ret;
}
#endif

#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
/**
 *  @brief Set/Get BSS role
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_bssrole(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	t_u32 data[1];
	int ret = 0;
	int user_data_len = 0;
	t_u8 action = MLAN_ACT_GET;

	ENTER();

	memset((char *)data, 0, sizeof(data));
	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_BSSROLE))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_BSSROLE),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len >= 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto error;
	}

	if (user_data_len == 0) {
		action = MLAN_ACT_GET;
	} else {
		if ((data[0] != MLAN_BSS_ROLE_STA &&
		     data[0] != MLAN_BSS_ROLE_UAP) ||
		    priv->bss_type != MLAN_BSS_TYPE_WIFIDIRECT) {
			PRINTM(MWARN, "Invalid BSS role\n");
			ret = -EINVAL;
			goto error;
		}
		if (data[0] == GET_BSS_ROLE(priv)) {
			PRINTM(MWARN, "Already BSS is in desired role\n");
			goto done;
		}
		action = MLAN_ACT_SET;
		/* Reset interface */
		woal_reset_intf(priv, MOAL_IOCTL_WAIT, MFALSE);
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_role_cfg(priv, action, MOAL_IOCTL_WAIT, (t_u8 *)data)) {
		ret = -EFAULT;
		goto error;
	}

	if (user_data_len) {
		/* Initialize private structures */
		woal_init_priv(priv, MOAL_IOCTL_WAIT);
		/* Enable interfaces */
		netif_device_attach(priv->netdev);
		woal_start_queue(priv->netdev);
	}

done:
	memset(respbuf, 0, respbuflen);
	respbuf[0] = (t_u8)data[0];
	ret = 1;

error:
	LEAVE();
	return ret;
}
#endif /* STA_SUPPORT && UAP_SUPPORT */
#endif /* WIFI_DIRECT_SUPPORT */

#ifdef STA_SUPPORT
/**
 *  @brief Set user scan
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_setuserscan(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	wlan_user_scan_cfg scan_cfg;
	int ret = 0;

	ENTER();

	/* Create the scan_cfg structure */
	memset(&scan_cfg, 0, sizeof(scan_cfg));

	/* We expect the scan_cfg structure to be passed in respbuf */
	moal_memcpy_ext(priv->phandle, (char *)&scan_cfg,
			respbuf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_SETUSERSCAN),
			sizeof(wlan_user_scan_cfg), sizeof(wlan_user_scan_cfg));
	moal_memcpy_ext(priv->phandle, scan_cfg.random_mac, priv->random_mac,
			ETH_ALEN, MLAN_MAC_ADDR_LENGTH);
	/* Call for scan */
	if (MLAN_STATUS_FAILURE == woal_do_scan(priv, &scan_cfg))
		ret = -EFAULT;

	LEAVE();
	return ret;
}

/**
 *  @brief Get channel statistics
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_get_chanstats(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_scan_resp scan_resp;
	chan_stats *stats = NULL;
	int ret = 0;
	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	memset(&scan_resp, 0, sizeof(scan_resp));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_scan_table(priv, MOAL_IOCTL_WAIT, &scan_resp)) {
		ret = -EFAULT;
		goto done;
	}
	memset(respbuf, 0, respbuflen);
	stats = (chan_stats *)respbuf;
	stats->num_in_chan_stats = scan_resp.num_in_chan_stats;
	ret = sizeof(ChanStatistics_t) * stats->num_in_chan_stats;
	moal_memcpy_ext(priv->phandle, (t_u8 *)stats->stats,
			(t_u8 *)scan_resp.pchan_stats, ret, respbuflen);
	ret += sizeof(stats->num_in_chan_stats);
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Retrieve the scan response/beacon table
 *
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *  @param scan_resp    A pointer to mlan_scan_resp structure
 *  @param scan_start   Argument
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
int moal_ret_get_scan_table_ioctl(t_u8 *respbuf, t_u32 respbuflen,
				  mlan_scan_resp *scan_resp, t_u32 scan_start)
{
	pBSSDescriptor_t pbss_desc, scan_table;
	wlan_ioctl_get_scan_table_info *prsp_info;
	int ret_code;
	int ret_len;
	int space_left;
	t_u8 *pcurrent;
	t_u8 *pbuffer_end;
	t_u32 num_scans_done;

	ENTER();

	num_scans_done = 0;
	ret_code = MLAN_STATUS_SUCCESS;

	prsp_info = (wlan_ioctl_get_scan_table_info *)respbuf;
	pcurrent = (t_u8 *)prsp_info->scan_table_entry_buf;

	pbuffer_end = respbuf + respbuflen - 1;
	space_left = pbuffer_end - pcurrent;
	scan_table = (BSSDescriptor_t *)(scan_resp->pscan_table);

	PRINTM(MINFO, "GetScanTable: scan_start req = %d\n", scan_start);
	PRINTM(MINFO, "GetScanTable: length avail = %d\n", respbuflen);

	if (!scan_start) {
		PRINTM(MINFO, "GetScanTable: get current BSS Descriptor\n");

		/* Use to get current association saved descriptor */
		pbss_desc = scan_table;

		ret_code = wlan_get_scan_table_ret_entry(pbss_desc, &pcurrent,
							 &space_left);

		if (ret_code == MLAN_STATUS_SUCCESS)
			num_scans_done = 1;

	} else {
		scan_start--;

		while (space_left &&
		       (scan_start + num_scans_done <
			scan_resp->num_in_scan_table) &&
		       (ret_code == MLAN_STATUS_SUCCESS)) {
			pbss_desc =
				(scan_table + (scan_start + num_scans_done));

			PRINTM(MINFO,
			       "GetScanTable: get current BSS Descriptor [%d]\n",
			       scan_start + num_scans_done);

			ret_code = wlan_get_scan_table_ret_entry(
				pbss_desc, &pcurrent, &space_left);

			if (ret_code == MLAN_STATUS_SUCCESS)
				num_scans_done++;
		}
	}

	prsp_info->scan_number = num_scans_done;
	ret_len = pcurrent - respbuf;

	LEAVE();
	return ret_len;
}

/**
 *  @brief Get scan table
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_getscantable(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_scan *scan = NULL;
	t_u32 scan_start;
	mlan_status status = MLAN_STATUS_SUCCESS;
	moal_handle *handle = priv->phandle;

	ENTER();

	/* First make sure scanning is not in progress */
	if (handle->scan_pending_on_block == MTRUE) {
		ret = -EAGAIN;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	scan = (mlan_ds_scan *)req->pbuf;
	req->req_id = MLAN_IOCTL_SCAN;
	req->action = MLAN_ACT_GET;

	/* Get the whole command from user */
	moal_memcpy_ext(handle, &scan_start,
			respbuf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_GETSCANTABLE),
			sizeof(scan_start), sizeof(scan_start));
	if (scan_start)
		scan->sub_command = MLAN_OID_SCAN_NORMAL;
	else
		scan->sub_command = MLAN_OID_SCAN_GET_CURRENT_BSS;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status == MLAN_STATUS_SUCCESS) {
		ret = moal_ret_get_scan_table_ioctl(respbuf, respbuflen,
						    &scan->param.scan_resp,
						    scan_start);
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
/**
 *  @brief Extended capabilities configuration
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_extcapcfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret, header;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	IEEEtypes_Header_t *ie;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		LEAVE();
		return 0;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_EXT_CAP_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	header = strlen(CMD_NXP) + strlen(PRIV_CMD_EXTCAPCFG);
	if (strlen(respbuf) == header)
		/* GET operation */
		req->action = MLAN_ACT_GET;
	else {
		/* SET operation */
		ie = (IEEEtypes_Header_t *)(respbuf + header);
		if (ie->len > sizeof(ExtCap_t)) {
			PRINTM(MERROR,
			       "Extended Capability lenth is invalid\n");
			ret = -EFAULT;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		memset(&cfg->param.ext_cap, 0, sizeof(ExtCap_t));
		moal_memcpy_ext(priv->phandle, &cfg->param.ext_cap, ie + 1,
				ie->len, sizeof(ExtCap_t));
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	memset(respbuf, 0, respbuflen);
	ie = (IEEEtypes_Header_t *)respbuf;
	ie->element_id = EXT_CAPABILITY;
	ie->len = sizeof(ExtCap_t);
	moal_memcpy_ext(priv->phandle, ie + 1, &cfg->param.ext_cap,
			sizeof(ExtCap_t),
			respbuflen - sizeof(IEEEtypes_Header_t));

	ret = sizeof(IEEEtypes_Header_t) + ie->len;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Set/Get deep sleep mode configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_setgetdeepsleep(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	t_u32 data[2];
	int ret = 0;
	int user_data_len = 0;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_DEEPSLEEP))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_DEEPSLEEP),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len >= 3) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len == 0) {
		if (MLAN_STATUS_SUCCESS != woal_get_deep_sleep(priv, data)) {
			ret = -EFAULT;
			goto done;
		}
		sprintf(respbuf, "%d %d", data[0], data[1]);
		ret = strlen(respbuf) + 1;
	} else {
		if (data[0] == DEEP_SLEEP_OFF) {
			PRINTM(MINFO, "Exit Deep Sleep Mode\n");
			ret = woal_set_deep_sleep(priv, MOAL_IOCTL_WAIT, MFALSE,
						  0);
			if (ret != MLAN_STATUS_SUCCESS) {
				ret = -EINVAL;
				goto done;
			}
		} else if (data[0] == DEEP_SLEEP_ON) {
			PRINTM(MINFO, "Enter Deep Sleep Mode\n");
			if (user_data_len != 2)
				data[1] = 0;
			ret = woal_set_deep_sleep(priv, MOAL_IOCTL_WAIT, MTRUE,
						  data[1]);
			if (ret != MLAN_STATUS_SUCCESS) {
				ret = -EINVAL;
				goto done;
			}
		} else {
			PRINTM(MERROR, "Unknown option = %u\n", data[0]);
			ret = -EINVAL;
			goto done;
		}
		ret = sprintf(respbuf, "OK\n") + 1;
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get IP address configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_setgetipaddr(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0, op_code = 0, data_length = 0, header = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (priv->bss_type != MLAN_BSS_TYPE_STA) {
		PRINTM(MIOCTL, "Bss type[%d]: Not STA, ignore it\n",
		       priv->bss_type);
		ret = sprintf(respbuf, "OK\n") + 1;
		goto done;
	}

	header = strlen(CMD_NXP) + strlen(PRIV_CMD_IPADDR);
	data_length = strlen(respbuf) - header;

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;

	if (data_length < 1) { /* GET */
		req->action = MLAN_ACT_GET;
	} else {
		/* Make sure we have the operation argument */
		if (data_length > 2 && respbuf[header + 1] != ';') {
			PRINTM(MERROR,
			       "No operation argument. Separate with ';'\n");
			ret = -EINVAL;
			goto done;
		} else {
			respbuf[header + 1] = '\0';
		}
		req->action = MLAN_ACT_SET;

		/* Only one IP is supported in current firmware */
		memset(misc->param.ipaddr_cfg.ip_addr[0], 0, IPADDR_LEN);
		if (data_length > 2)
			in4_pton(&respbuf[header + 2],
				 MIN((IPADDR_MAX_BUF - 3), (data_length - 2)),
				 misc->param.ipaddr_cfg.ip_addr[0], ' ', NULL);
		misc->param.ipaddr_cfg.ip_addr_num = 1;
		misc->param.ipaddr_cfg.ip_addr_type = IPADDR_TYPE_IPV4;

		if (woal_atoi(&op_code, &respbuf[header]) !=
		    MLAN_STATUS_SUCCESS) {
			ret = -EINVAL;
			goto done;
		}
		misc->param.ipaddr_cfg.op_code = (t_u32)op_code;
	}

	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->sub_command = MLAN_OID_MISC_IP_ADDR;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		snprintf(respbuf, IPADDR_MAX_BUF, "%d;%d.%d.%d.%d",
			 misc->param.ipaddr_cfg.op_code,
			 misc->param.ipaddr_cfg.ip_addr[0][0],
			 misc->param.ipaddr_cfg.ip_addr[0][1],
			 misc->param.ipaddr_cfg.ip_addr[0][2],
			 misc->param.ipaddr_cfg.ip_addr[0][3]);
		ret = IPADDR_MAX_BUF + 1;
	} else {
		ret = sprintf(respbuf, "OK\n") + 1;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get WPS session configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_setwpssession(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wps_cfg *pwps = NULL;
	t_u32 data[1];
	int ret = 0;
	int user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset((char *)data, 0, sizeof(data));
	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_WPSSESSION))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_WPSSESSION),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wps_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pwps = (mlan_ds_wps_cfg *)req->pbuf;

	req->req_id = MLAN_IOCTL_WPS_CFG;
	req->action = MLAN_ACT_SET;
	pwps->sub_command = MLAN_OID_WPS_CFG_SESSION;

	if (data[0] == 1)
		pwps->param.wps_session = MLAN_WPS_CFG_SESSION_START;
	else
		pwps->param.wps_session = MLAN_WPS_CFG_SESSION_END;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	ret = sprintf(respbuf, "OK\n") + 1;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get OTP user data
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_otpuserdata(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data[1];
	int user_data_len = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_otp_user_data *otp = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_OTPUSERDATA))) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_OTPUSERDATA),
			data, ARRAY_SIZE(data), &user_data_len);

	if (user_data_len != 1) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	req->action = MLAN_ACT_GET;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_OTP_USER_DATA;
	misc->param.otp_user_data.user_data_length = data[0];

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	otp = (mlan_ds_misc_otp_user_data *)req->pbuf;

	if (req->action == MLAN_ACT_GET) {
		ret = MIN(otp->user_data_length, data[0]);
		moal_memcpy_ext(priv->phandle, respbuf, otp->user_data, ret,
				respbuflen);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set / Get country code
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_countrycode(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	/* char data[COUNTRY_CODE_LEN] = {0, 0, 0}; */
	int header = 0, data_length = 0; /* wrq->u.data.length; */
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	mlan_ds_misc_country_code *country_code = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header = strlen(CMD_NXP) + strlen(PRIV_CMD_COUNTRYCODE);
	data_length = strlen(respbuf) - header;

	if (data_length > COUNTRY_CODE_LEN) {
		PRINTM(MERROR, "Invalid argument!\n");
		ret = -EINVAL;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	country_code = &pcfg_misc->param.country_code;
	pcfg_misc->sub_command = MLAN_OID_MISC_COUNTRY_CODE;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (data_length <= 1) {
		req->action = MLAN_ACT_GET;
	} else {
		memset(country_code->country_code, 0, COUNTRY_CODE_LEN);
		moal_memcpy_ext(priv->phandle, country_code->country_code,
				respbuf + header, COUNTRY_CODE_LEN,
				COUNTRY_CODE_LEN);
		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		ret = data_length = COUNTRY_CODE_LEN;
		memset(respbuf + header, 0, COUNTRY_CODE_LEN);
		moal_memcpy_ext(priv->phandle, respbuf,
				country_code->country_code, COUNTRY_CODE_LEN,
				respbuflen);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Get cfp information
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_get_cfpinfo(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfp_misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int header = 0, data_length = 0;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header = strlen(CMD_NXP) + strlen(PRIV_CMD_CFPINFO);
	data_length = strlen(respbuf) - header;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	cfp_misc = (mlan_ds_misc_cfg *)req->pbuf;
	cfp_misc->sub_command = MLAN_OID_MISC_CFP_INFO;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (respbuflen < req->data_read_written) {
		PRINTM(MERROR, "response buffer length is too short!\n");
		ret = -EINVAL;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)req->pbuf,
			req->data_read_written, respbuflen);
	ret = req->data_read_written;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get TCP Ack enhancement configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_setgettcpackenh(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	t_u32 data[1];
	int ret = 0;
	int user_data_len = 0;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_TCPACKENH))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_TCPACKENH),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len >= 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len == 0) {
		/* get operation */
		respbuf[0] = priv->enable_tcp_ack_enh;
	} else {
		/* set operation */
		if (data[0] == MTRUE) {
			PRINTM(MINFO, "Enabling TCP Ack enhancement\n");
			priv->enable_tcp_ack_enh = MTRUE;
		} else if (data[0] == MFALSE) {
			PRINTM(MINFO, "Disabling TCP Ack enhancement\n");
			priv->enable_tcp_ack_enh = MFALSE;
			/* release the tcp sessions if any */
			woal_flush_tcp_sess_queue(priv);
		} else {
			PRINTM(MERROR, "Unknown option = %u\n", data[0]);
			ret = -EINVAL;
			goto done;
		}
		respbuf[0] = priv->enable_tcp_ack_enh;
	}
	ret = 1;

done:
	LEAVE();
	return ret;
}

#ifdef REASSOCIATION
/**
 *  @brief Set Asynced ESSID
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *  @param bBSSID       A variable that bssid is set or not
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_assocessid(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen,
			 t_u8 bBSSID)
{
	mlan_ssid_bssid ssid_bssid;
	moal_handle *handle = priv->phandle;
	int ret = 0;
	int header_len = 0;
	int copy_len = 0;
	char buf[64];
	t_u8 buflen = 0;
	t_u8 i = 0;
	t_u8 mac_idx = 0;

	ENTER();

	if (bBSSID)
		header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ASSOCBSSID);
	else
		header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ASSOCESSID);

	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "No argument, invalid operation!\n");
		ret = -EINVAL;
		LEAVE();
		return ret;
	}
	copy_len = strlen(respbuf) - header_len;
	buflen = MIN(copy_len, (sizeof(buf) - 1));
	memset(buf, 0, sizeof(buf));
	memset(&ssid_bssid, 0, sizeof(mlan_ssid_bssid));
	moal_memcpy_ext(handle, buf, respbuf + header_len, buflen, sizeof(buf));
	priv->assoc_with_mac = MFALSE;

	/* check if has parameter BSSID */
	if (bBSSID) {
		if (buflen < (3 * ETH_ALEN) + 2) {
			PRINTM(MERROR,
			       "Associate: Insufficient length in IOCTL input\n");
			/* buffer should be at least 3 characters per BSSID
			 *octet "00:"
			 **   plus a space separater and at least 1 char in the
			 *SSID
			 */
			ret = -EINVAL;
			goto setessid_ret;
		}
		for (; (i < buflen) && (mac_idx < ETH_ALEN) && (buf[i] != ' ');
		     i++) {
			if (buf[i] == ':') {
				mac_idx++;
			} else {
				ssid_bssid.bssid[mac_idx] =
					(t_u8)woal_atox(buf + i);
				while ((i < buflen) && isxdigit(buf[i + 1]))
					/* Skip entire hex value */
					i++;
			}
		}
		/* Skip one space between the BSSID and start of the SSID */
		i++;
		PRINTM(MMSG, "Trying to associate AP BSSID = [" MACSTR "]\n",
		       MAC2STR(ssid_bssid.bssid));
		priv->assoc_with_mac = MTRUE;
	}

	ssid_bssid.ssid.ssid_len = buflen - i;
	/* Check the size of the ssid_len */
	if (ssid_bssid.ssid.ssid_len > MLAN_MAX_SSID_LENGTH + 1) {
		PRINTM(MERROR, "ssid_bssid.ssid.ssid_len = %d\n",
		       ssid_bssid.ssid.ssid_len);
		ret = -E2BIG;
		goto setessid_ret;
	}

	/* Copy the SSID */
	moal_memcpy_ext(handle, ssid_bssid.ssid.ssid, buf + i,
			ssid_bssid.ssid.ssid_len, MLAN_MAX_SSID_LENGTH);

	if (!ssid_bssid.ssid.ssid_len ||
	    (MFALSE == woal_ssid_valid(&ssid_bssid.ssid))) {
		PRINTM(MERROR, "Invalid SSID - aborting set_essid\n");
		ret = -EINVAL;
		goto setessid_ret;
	}

	PRINTM(MMSG, "Trying to associate AP SSID = %s\n",
	       (char *)ssid_bssid.ssid.ssid);

	/* Cancel re-association */
	priv->reassoc_required = MFALSE;

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->reassoc_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_set_essid\n");
		ret = -EBUSY;
		LEAVE();
		return ret;
	}

	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE);

	if (MTRUE == woal_is_connected(priv, &ssid_bssid)) {
		PRINTM(MIOCTL, "Already connect to the network\n");
		ret = sprintf(respbuf,
			      "Has already connected to this ESSID!\n") +
		      1;
		goto setessid_ret;
	}
	moal_memcpy_ext(handle, &priv->prev_ssid_bssid, &ssid_bssid,
			sizeof(mlan_ssid_bssid), sizeof(mlan_ssid_bssid));
	/* disconnect before driver assoc */
	woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL, DEF_DEAUTH_REASON_CODE);
	priv->set_asynced_essid_flag = MTRUE;
	priv->reassoc_required = MTRUE;
	priv->phandle->is_reassoc_timer_set = MTRUE;
	woal_mod_timer(&priv->phandle->reassoc_timer, 0);
	ret = sprintf(respbuf, "%s\n", buf) + 1;

setessid_ret:
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE);
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Get wakeup reason
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_getwakeupreason(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	t_u32 data;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_WAKEUPREASON))) {
		/* GET operation */
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
		if (req == NULL) {
			LEAVE();
			return -ENOMEM;
		}

		pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
		pm_cfg->sub_command = MLAN_OID_PM_HS_WAKEUP_REASON;
		req->req_id = MLAN_IOCTL_PM_CFG;
		req->action = MLAN_ACT_GET;

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			if (status != MLAN_STATUS_PENDING)
				kfree(req);
			goto done;
		} else {
			data = pm_cfg->param.wakeup_reason.hs_wakeup_reason;
			sprintf(respbuf, " %d", data);
			ret = strlen(respbuf) + 1;
			kfree(req);
		}
	} else {
		PRINTM(MERROR, "Not need argument, invalid operation!\n");
		ret = -EINVAL;
		goto done;
	}

done:
	LEAVE();
	return ret;
}

#ifdef STA_SUPPORT
/**
 *  @brief Set / Get listen interval
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_listeninterval(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	int data[1];
	int user_data_len = 0;
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *pcfg_bss = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_LISTENINTERVAL))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_LISTENINTERVAL),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pcfg_bss = (mlan_ds_bss *)req->pbuf;
	pcfg_bss->sub_command = MLAN_OID_BSS_LISTEN_INTERVAL;
	req->req_id = MLAN_IOCTL_BSS;

	if (user_data_len) {
		pcfg_bss->param.listen_interval = (t_u16)data[0];
		req->action = MLAN_ACT_SET;
	} else {
		req->action = MLAN_ACT_GET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (req->action == MLAN_ACT_GET) {
		sprintf(respbuf, "%d", pcfg_bss->param.listen_interval);
		ret = strlen(respbuf) + 1;
	} else {
		ret = sprintf(respbuf, "OK\n") + 1;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

#ifdef DEBUG_LEVEL1
/**
 *  @brief Set / Get driver debug level
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_drvdbg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data[4];
	int user_data_len = 0;
	int ret = 0;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_DRVDBG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_DRVDBG),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len) {
		/* Get the driver debug bit masks from user */
		drvdbg = data[0];
		/* Set the driver debug bit masks into mlan */
		if (woal_set_drvdbg(priv, drvdbg)) {
			PRINTM(MERROR, "Set drvdbg failed!\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

	ret = sizeof(drvdbg);

	moal_memcpy_ext(priv->phandle, respbuf, &drvdbg, sizeof(drvdbg),
			respbuflen);

	printk(KERN_ALERT "drvdbg = 0x%08x\n", drvdbg);
#ifdef DEBUG_LEVEL2
	printk(KERN_ALERT "MINFO  (%08x) %s\n", MINFO,
	       (drvdbg & MINFO) ? "X" : "");
	printk(KERN_ALERT "MWARN  (%08x) %s\n", MWARN,
	       (drvdbg & MWARN) ? "X" : "");
	printk(KERN_ALERT "MENTRY (%08x) %s\n", MENTRY,
	       (drvdbg & MENTRY) ? "X" : "");
#endif
	printk(KERN_ALERT "MMPA_D (%08x) %s\n", MMPA_D,
	       (drvdbg & MMPA_D) ? "X" : "");
	printk(KERN_ALERT "MIF_D  (%08x) %s\n", MIF_D,
	       (drvdbg & MIF_D) ? "X" : "");
	printk(KERN_ALERT "MFW_D  (%08x) %s\n", MFW_D,
	       (drvdbg & MFW_D) ? "X" : "");
	printk(KERN_ALERT "MEVT_D (%08x) %s\n", MEVT_D,
	       (drvdbg & MEVT_D) ? "X" : "");
	printk(KERN_ALERT "MCMD_D (%08x) %s\n", MCMD_D,
	       (drvdbg & MCMD_D) ? "X" : "");
	printk(KERN_ALERT "MDAT_D (%08x) %s\n", MDAT_D,
	       (drvdbg & MDAT_D) ? "X" : "");
	printk(KERN_ALERT "MREG_D (%08x) %s\n", MREG_D,
	       (drvdbg & MREG_D) ? "X" : "");
	printk(KERN_ALERT "MIOCTL (%08x) %s\n", MIOCTL,
	       (drvdbg & MIOCTL) ? "X" : "");
	printk(KERN_ALERT "MINTR  (%08x) %s\n", MINTR,
	       (drvdbg & MINTR) ? "X" : "");
	printk(KERN_ALERT "MEVENT (%08x) %s\n", MEVENT,
	       (drvdbg & MEVENT) ? "X" : "");
	printk(KERN_ALERT "MCMND  (%08x) %s\n", MCMND,
	       (drvdbg & MCMND) ? "X" : "");
	printk(KERN_ALERT "MDATA  (%08x) %s\n", MDATA,
	       (drvdbg & MDATA) ? "X" : "");
	printk(KERN_ALERT "MERROR (%08x) %s\n", MERROR,
	       (drvdbg & MERROR) ? "X" : "");
	printk(KERN_ALERT "MFATAL (%08x) %s\n", MFATAL,
	       (drvdbg & MFATAL) ? "X" : "");
	printk(KERN_ALERT "MMSG   (%08x) %s\n", MMSG,
	       (drvdbg & MMSG) ? "X" : "");

done:
	LEAVE();
	return ret;
}

#endif

/**
 *  @brief management frame filter wakeup config
 *
 *  @param priv             A pointer to moal_private structure
 *  @param respbuf          A pointer to response buffer
 *  @param respbuflen       Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_mgmt_filter(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int header_len = 0, data_len = 0;
	int ret = 0;
	t_u16 action;
	t_u8 *argument;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	pm_cfg = (mlan_ds_pm_cfg *)ioctl_req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_MGMT_FILTER;
	ioctl_req->req_id = MLAN_IOCTL_PM_CFG;

	header_len = strlen(PRIV_CMD_MGMT_FILTER) + strlen(CMD_NXP);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		argument = (t_u8 *)(respbuf + header_len);
		data_len = respbuflen - header_len;
		if (data_len >
		    MAX_MGMT_FRAME_FILTER * sizeof(mlan_mgmt_frame_wakeup)) {
			PRINTM(MERROR, "%d: Invalid arguments\n", __LINE__);
			ret = -EINVAL;
			goto done;
		}
		moal_memcpy_ext(priv->phandle,
				(t_u8 *)pm_cfg->param.mgmt_filter, argument,
				data_len, sizeof(pm_cfg->param.mgmt_filter));
		action = MLAN_ACT_SET;
	}

	ioctl_req->action = action;
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

#define PARAMETER_GPIO_INDICATION 1
#define PARAMETER_EXTEND_HSCFG 2
#define PARAMETER_HS_WAKEUP_INTERVAL 3
/**
 *  @brief Set/Get Host Sleep configuration
 *
 *  @param priv             A pointer to moal_private structure
 *  @param respbuf          A pointer to response buffer
 *  @param respbuflen       Available length of response buffer
 *  @param invoke_hostcmd   MTRUE --invoke HostCmd, otherwise MFALSE
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_hscfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen,
		    BOOLEAN invoke_hostcmd)
{
	int data[13] = {0};
	int *temp_data, type;
	int user_data_len = 0;
	int ret = 0;
	mlan_ds_hs_cfg hscfg, hscfg_temp;
	t_u16 action;
	mlan_bss_info bss_info;
	int is_negative = MFALSE;
	t_u8 *arguments = NULL;

	ENTER();

	memset(data, 0, sizeof(data));
	memset(&hscfg, 0, sizeof(mlan_ds_hs_cfg));
	memset(&hscfg_temp, 0, sizeof(mlan_ds_hs_cfg));

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_HSCFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		arguments = respbuf + strlen(CMD_NXP) + strlen(PRIV_CMD_HSCFG);
		if (*arguments == '-') {
			is_negative = MTRUE;
			arguments += 1;
		}
		parse_arguments(arguments, data, ARRAY_SIZE(data),
				&user_data_len);

		if (is_negative == MTRUE) {
			if (data[0] == 1) {
				data[0] = -1;
			} else {
				PRINTM(MERROR, "Invalid arguments\n");
				ret = -EINVAL;
				goto done;
			}
		}
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		LEAVE();
		return -EINVAL;
	}

	if (user_data_len == 0) {
		action = MLAN_ACT_GET;
	} else {
		if (user_data_len >= 1 && user_data_len <= 13) {
			action = MLAN_ACT_SET;
		} else {
			PRINTM(MERROR, "Invalid arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}

	/* HS config is blocked if HS is already activated */
	if (user_data_len &&
	    (data[0] != HOST_SLEEP_CFG_CANCEL || invoke_hostcmd == MFALSE)) {
		memset(&bss_info, 0, sizeof(bss_info));
		woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
		if (bss_info.is_hs_configured) {
			PRINTM(MERROR, "HS already configured\n");
			ret = -EFAULT;
			goto done;
		}
	}

	/* Do a GET first if some arguments are not provided */
	if (user_data_len >= 1 && user_data_len < 11) {
		woal_set_get_hs_params(priv, MLAN_ACT_GET, MOAL_IOCTL_WAIT,
				       &hscfg_temp);
	}
	hscfg.conditions = hscfg_temp.conditions;
	hscfg.gpio = hscfg_temp.gpio;
	hscfg.gap = hscfg_temp.gap;

	if (user_data_len)
		hscfg.conditions = data[0];
	if (user_data_len >= 2)
		hscfg.gpio = data[1];
	if (user_data_len >= 3)
		hscfg.gap = data[2];
	user_data_len = user_data_len - 3;
	if (user_data_len > 0) {
		temp_data = data + 3;
		while ((user_data_len > 0) && temp_data) {
			type = *temp_data;
			switch (type) {
			case PARAMETER_GPIO_INDICATION:
				if (user_data_len >= 2)
					hscfg.ind_gpio = *(++temp_data);
				else {
					PRINTM(MERROR,
					       "Invaild number of parameters\n");
					ret = -EINVAL;
					goto done;
				}
				if (user_data_len >= 3) {
					hscfg.level = *(++temp_data);
					if (hscfg.level != 0 &&
					    hscfg.level != 1) {
						PRINTM(MERROR,
						       "Invalid indication gpio arguments\n");
						ret = -EINVAL;
						goto done;
					}
				}
				hscfg.param_type_ind = type;
				user_data_len = user_data_len - 3;
				temp_data++;
				break;
			case PARAMETER_EXTEND_HSCFG:
				if (user_data_len >= 4) {
					hscfg.event_force_ignore =
						*(++temp_data);
					hscfg.event_use_ext_gap =
						*(++temp_data);
					hscfg.ext_gap = *(++temp_data);
					hscfg.gpio_wave = *(++temp_data);
				} else {
					PRINTM(MERROR,
					       "Invaild number of parameters\n");
					ret = -EINVAL;
					goto done;
				}
				/* Force_ignore_gpio and ext_gap_gpio should not
				 * set the same bit(s)*/
				if ((hscfg.event_force_ignore &
				     hscfg.event_use_ext_gap) ||
				    (hscfg.gpio_wave != 1 &&
				     hscfg.gpio_wave != 0)) {
					PRINTM(MERROR,
					       "Invalid arguments for extend hscfg\n");
					ret = -EINVAL;
					goto done;
				}
				hscfg.param_type_ext = type;
				user_data_len = user_data_len - 5;
				temp_data++;
				break;
			case PARAMETER_HS_WAKEUP_INTERVAL:
				if (user_data_len >= 2)
					hscfg.hs_wake_interval = *(++temp_data);
				else {
					PRINTM(MERROR,
					       "Invaild number of parameters\n");
					ret = -EINVAL;
					goto done;
				}
				user_data_len = user_data_len - 2;
				temp_data++;
				break;
			default:
				PRINTM(MERROR, "Unsupported type\n");
				ret = -EINVAL;
				goto done;
			}
		}
	}

	if ((invoke_hostcmd == MTRUE) && (action == MLAN_ACT_SET)) {
		/* Need to issue an extra IOCTL first to set up parameters */
		hscfg.is_invoke_hostcmd = MFALSE;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_hs_params(priv, MLAN_ACT_SET, MOAL_IOCTL_WAIT,
					   &hscfg)) {
			ret = -EFAULT;
			goto done;
		}
	}
	hscfg.is_invoke_hostcmd = invoke_hostcmd;
	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_hs_params(priv, action, MOAL_IOCTL_WAIT, &hscfg)) {
		ret = -EFAULT;
		goto done;
	}

	if (action == MLAN_ACT_GET) {
		/* Return the current driver host sleep configurations */
		moal_memcpy_ext(priv->phandle, respbuf, &hscfg,
				sizeof(mlan_ds_hs_cfg), respbuflen);
		ret = sizeof(mlan_ds_hs_cfg);
	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set Host Sleep parameters
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_hssetpara(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data[13] = {0};
	int user_data_len = 0;
	int ret = 0;

	ENTER();

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_HSSETPARA))) {
		PRINTM(MERROR, "Invalid arguments\n");
		ret = -EINVAL;
		goto done;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_HSSETPARA),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		LEAVE();
		return -EINVAL;
	}

	if (user_data_len >= 1 && user_data_len <= 13) {
		sprintf(respbuf, "%s%s%s", CMD_NXP, PRIV_CMD_HSCFG,
			respbuf +
				(strlen(CMD_NXP) + strlen(PRIV_CMD_HSSETPARA)));
		respbuflen = strlen(respbuf);
		ret = woal_priv_hscfg(priv, respbuf, respbuflen, MFALSE);
		goto done;
	}
done:
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get scan configuration parameters
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
int woal_priv_set_get_scancfg(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0;
	int data[9];
	mlan_ds_scan *scan = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(data, 0, sizeof(data));
	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_SCANCFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_SCANCFG),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		LEAVE();
		return -EINVAL;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_scan));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	scan = (mlan_ds_scan *)req->pbuf;
	scan->sub_command = MLAN_OID_SCAN_CONFIG;
	req->req_id = MLAN_IOCTL_SCAN;

	if (user_data_len) {
		if ((data[0] < 0) || (data[0] > MLAN_SCAN_TYPE_PASSIVE)) {
			PRINTM(MERROR, "Invalid argument for scan type\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[1] < 0) || (data[1] > MLAN_SCAN_MODE_ANY)) {
			PRINTM(MERROR, "Invalid argument for scan mode\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[2] < 0) || (data[2] > MAX_PROBES)) {
			PRINTM(MERROR, "Invalid argument for scan probes\n");
			ret = -EINVAL;
			goto done;
		}
		if (((data[3] < 0) ||
		     (data[3] > MRVDRV_MAX_ACTIVE_SCAN_CHAN_TIME)) ||
		    ((data[4] < 0) ||
		     (data[4] > MRVDRV_MAX_ACTIVE_SCAN_CHAN_TIME)) ||
		    ((data[5] < 0) ||
		     (data[5] > MRVDRV_MAX_PASSIVE_SCAN_CHAN_TIME))) {
			PRINTM(MERROR, "Invalid argument for scan time\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[6] < 0) || (data[6] > MLAN_PASS_TO_ACT_SCAN_DIS)) {
			PRINTM(MERROR,
			       "Invalid argument for Passive to Active Scan\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[7] < 0) || (data[7] > 3)) {
			PRINTM(MERROR, "Invalid argument for extended scan\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[8] < 0) ||
		    (data[8] > MRVDRV_MAX_SCAN_CHAN_GAP_TIME)) {
			PRINTM(MERROR,
			       "Invalid argument for scan channel gap\n");
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		moal_memcpy_ext(priv->phandle, &scan->param.scan_cfg, data,
				sizeof(data), sizeof(scan->param.scan_cfg));
	} else
		req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, respbuf, &scan->param.scan_cfg,
			sizeof(mlan_scan_cfg), respbuflen);
	ret = sizeof(mlan_scan_cfg);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Get Netlink Number
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written, negative for failure.
 */
int woal_priv_getnlnum(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	int data = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null in %s\n", __FUNCTION__);
		ret = -EFAULT;
		goto done;
	}

	data = priv->phandle->netlink_num;
	moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set / Get packet aggregation control
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_aggrctrl(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int data[1];
	int user_data_len = 0;
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	moal_handle *handle = priv->phandle;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!handle || !handle->card) {
		PRINTM(MERROR, "Handle or card is null\n");
		ret = -EFAULT;
		goto done;
	}

	memset((char *)data, 0, sizeof(data));
	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_AGGRCTRL))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		if (woal_is_any_interface_active(priv->phandle)) {
			PRINTM(MERROR,
			       "aggrctrl are not allowed to change after BSS active!\n");
			ret = -EFAULT;
			goto done;
		}
		/* SET operation */
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_AGGRCTRL),
				data, ARRAY_SIZE(data), &user_data_len);

		if (sizeof(int) * user_data_len > sizeof(data)) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	pcfg_misc->sub_command = MLAN_OID_MISC_AGGR_CTRL;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	/* Get the values first, then modify these values if user had modified
	 * them */
	if (user_data_len == 0)
		req->action = MLAN_ACT_GET;
	else {
		req->action = MLAN_ACT_SET;
		pcfg_misc->param.aggr_params.tx.enable = (t_u16)data[0];
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		/* MLAN will return CMD_INVALID if FW does not support this
		 * feature */
		if (MLAN_ERROR_CMD_INVALID == req->status_code)
			ret = -EOPNOTSUPP;
		else
			ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(handle, respbuf, (t_u8 *)&pcfg_misc->param.aggr_params,
			sizeof(mlan_ds_misc_aggr_ctrl), respbuflen);
	ret = sizeof(mlan_ds_misc_aggr_ctrl);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

#ifdef USB
/**
 *  @brief Set / Get USB packet aggregation control
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_usbaggrctrl(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int data[8];
	int user_data_len = 0;
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg_misc = NULL;
	moal_handle *handle = priv->phandle;
	struct usb_card_rec *cardp = NULL;
	int i = 0, usb_resubmit_urbs = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!handle || !handle->card) {
		PRINTM(MERROR, "Handle or card is null\n");
		ret = -EFAULT;
		goto done;
	}
	cardp = (struct usb_card_rec *)handle->card;

	memset((char *)data, 0, sizeof(data));
	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_USBAGGRCTRL))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_USBAGGRCTRL),
				data, ARRAY_SIZE(data), &user_data_len);

		if (sizeof(int) * user_data_len > sizeof(data)) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	pcfg_misc = (mlan_ds_misc_cfg *)req->pbuf;
	pcfg_misc->sub_command = MLAN_OID_MISC_USB_AGGR_CTRL;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	/* Get the values first, then modify these values if user had modified
	 * them */
	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		/* MLAN will return CMD_INVALID if FW does not support this
		 * feature */
		if (MLAN_ERROR_CMD_INVALID == req->status_code)
			ret = -EOPNOTSUPP;
		else
			ret = -EFAULT;
		goto done;
	}

	if (user_data_len == 0) {
		moal_memcpy_ext(handle, respbuf,
				(t_u8 *)&pcfg_misc->param.usb_aggr_params,
				sizeof(mlan_ds_misc_usb_aggr_ctrl), respbuflen);
		ret = sizeof(mlan_ds_misc_usb_aggr_ctrl);
		goto done;
	}

	switch (user_data_len) {
	case 8:
		if (data[7] < 0) {
			PRINTM(MERROR, "Invalid Rx timeout value (%d)\n",
			       data[7]);
			ret = -EINVAL;
			goto done;
		}
		pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.aggr_tmo =
			(t_u16)data[7];
		/* fall through */
	case 7:
		if (data[6] < 0 || (data[6] > 10000 &&
				    data[6] != MLAN_USB_TX_AGGR_TIMEOUT_DYN)) {
			PRINTM(MERROR, "Invalid Tx timeout value (%d)\n",
			       data[6]);
			ret = -EINVAL;
			goto done;
		}
		pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.aggr_tmo =
			(t_u16)data[6];
		/* fall through */
	case 6:
		if ((data[5] < 512) || ((data[5] % 512) != 0)) {
			PRINTM(MERROR, "Invalid Rx alignment value (%d)\n",
			       data[5]);
			ret = -EINVAL;
			goto done;
		}
		if (cardp->rx_deaggr_ctrl.enable &&
		    pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.aggr_align !=
			    (t_u16)data[5])
			usb_resubmit_urbs = 1;
		pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.aggr_align =
			(t_u16)data[5];
		/* fall through */
	case 5:
		if ((data[4] < 2048) || ((data[4] % 2048) != 0)) {
			PRINTM(MERROR, "Invalid Tx alignment value (%d)\n",
			       data[4]);
			ret = -EINVAL;
			goto done;
		}
		pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.aggr_align =
			(t_u16)data[4];
		/* fall through */
	case 4:
		if ((data[3] == 2) || (data[3] == 4) || (data[3] == 8) ||
		    (data[3] == 16)) {
			pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl
				.aggr_mode = MLAN_USB_AGGR_MODE_NUM;
		} else if ((data[3] == 4096) || (data[3] == 8192) ||
			   (data[3] == 16384) || (data[3] == 32768)) {
			pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl
				.aggr_mode = MLAN_USB_AGGR_MODE_LEN;
		} else {
			PRINTM(MERROR, "Invalid Rx max size/num value (%d)\n",
			       data[3]);
			ret = -EINVAL;
			goto done;
		}
		if (cardp->rx_deaggr_ctrl.enable &&
		    pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.aggr_max !=
			    (t_u16)data[3])
			usb_resubmit_urbs = 1;
		pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.aggr_max =
			(t_u16)data[3];
		/* fall through */
	case 3:
		if ((data[2] == 2) || (data[2] == 4) || (data[2] == 8) ||
		    (data[2] == 16)) {
			pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.aggr_mode =
				MLAN_USB_AGGR_MODE_NUM;
		} else if ((data[2] == 4096) || (data[2] == 8192) ||
			   (data[2] == 16384) || (data[2] == 32768)) {
			pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.aggr_mode =
				MLAN_USB_AGGR_MODE_LEN;
		} else {
			PRINTM(MERROR, "Invalid Tx max size/num value (%d)\n",
			       data[2]);
			ret = -EINVAL;
			goto done;
		}
		pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.aggr_max =
			(t_u16)data[2];
		/* fall through */
	case 2:
		if ((data[1] != 0) && (data[1] != 1)) {
			PRINTM(MERROR, "Invalid Rx enable value (%d)\n",
			       data[1]);
			ret = -EINVAL;
			goto done;
		}
		if (pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.enable !=
		    (t_u16)data[1])
			usb_resubmit_urbs = 1;
		pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl.enable =
			(t_u16)data[1];
		/* fall through */
	case 1:
		if ((data[0] != 0) && (data[0] != 1)) {
			PRINTM(MERROR, "Invalid Tx enable value (%d)\n",
			       data[0]);
			ret = -EINVAL;
			goto done;
		}
		pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl.enable =
			(t_u16)data[0];
	default:
		break;
	}

	pcfg_misc->sub_command = MLAN_OID_MISC_USB_AGGR_CTRL;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_SET;

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(handle, respbuf,
			(t_u8 *)&pcfg_misc->param.usb_aggr_params,
			sizeof(mlan_ds_misc_usb_aggr_ctrl), respbuflen);
	ret = sizeof(mlan_ds_misc_usb_aggr_ctrl);

	/* Keep a copy of the latest Tx aggregation parameters in MOAL */
	moal_memcpy_ext(handle, &cardp->tx_aggr_ctrl,
			&pcfg_misc->param.usb_aggr_params.tx_aggr_ctrl,
			sizeof(usb_aggr_ctrl), sizeof(usb_aggr_ctrl));

	if (usb_resubmit_urbs) {
		/* Indicate resubmition from here */
		cardp->resubmit_urbs = 1;
		/* Rx SG parameters has changed or disabled, kill the URBs, they
		   will be resubmitted after saving the parameters to USB card
		 */
		if (atomic_read(&cardp->rx_data_urb_pending)) {
			for (i = 0; i < MVUSB_RX_DATA_URB; i++) {
				if (cardp->rx_data_list[i].urb) {
					usb_kill_urb(
						cardp->rx_data_list[i].urb);
					usb_init_urb(
						cardp->rx_data_list[i].urb);
				}
			}
		}
	}

	/* Keep a copy of the latest Rx deaggregation parameters in MOAL */
	moal_memcpy_ext(handle, &cardp->rx_deaggr_ctrl,
			&pcfg_misc->param.usb_aggr_params.rx_deaggr_ctrl,
			sizeof(usb_aggr_ctrl), sizeof(usb_aggr_ctrl));

	if (usb_resubmit_urbs) {
		/* Ensure the next data URBs will use the modified parameters */
		if (!atomic_read(&cardp->rx_data_urb_pending)) {
			/* Submit multiple Rx data URBs */
			woal_usb_submit_rx_data_urbs(handle);
		}
		cardp->resubmit_urbs = 0;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}
#endif

#ifdef STA_SUPPORT
/**
 * @brief Set AP settings
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful, otherwise fail
 */
static int woal_priv_set_ap(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	t_u8 *data_ptr;
	const t_u8 bcast[MLAN_MAC_ADDR_LENGTH] = {255, 255, 255, 255, 255, 255};
	const t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0, 0, 0, 0, 0, 0};
	mlan_ssid_bssid ssid_bssid;
	mlan_bss_info bss_info;
	struct mwreq *mwr;
	struct sockaddr *awrq;

	ENTER();
	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_SET_AP));

	mwr = (struct mwreq *)data_ptr;

	if (mwr->u.ap_addr.sa_family != ARPHRD_ETHER) {
		ret = -EINVAL;
		goto done;
	}

	awrq = (struct sockaddr *)&(mwr->u.ap_addr);

	PRINTM(MINFO, "ASSOC: WAP: sa_data: " MACSTR "\n",
	       MAC2STR((t_u8 *)awrq->sa_data));

	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}

#ifdef REASSOCIATION
	/* Cancel re-association */
	priv->reassoc_required = MFALSE;
#endif

	/* zero_mac means disconnect */
	if (!memcmp(zero_mac, awrq->sa_data, MLAN_MAC_ADDR_LENGTH)) {
		woal_disconnect(priv, MOAL_IOCTL_WAIT, NULL,
				DEF_DEAUTH_REASON_CODE);
		goto done;
	}

	/* Broadcast MAC means search for best network */
	memset(&ssid_bssid, 0, sizeof(mlan_ssid_bssid));

	if (memcmp(bcast, awrq->sa_data, MLAN_MAC_ADDR_LENGTH)) {
		/* Check if we are already assoicated to the AP */
		if (bss_info.media_connected == MTRUE) {
			if (!memcmp(awrq->sa_data, &bss_info.bssid, ETH_ALEN))
				goto done;
		}
		moal_memcpy_ext(priv->phandle, &ssid_bssid.bssid, awrq->sa_data,
				ETH_ALEN, sizeof(mlan_802_11_mac_addr));
	}

	if (MLAN_STATUS_SUCCESS !=
	    woal_find_best_network(priv, MOAL_IOCTL_WAIT, &ssid_bssid)) {
		PRINTM(MERROR,
		       "ASSOC: WAP: MAC address not found in BSSID List\n");
		ret = -ENETUNREACH;
		goto done;
	}
	/* Zero SSID implies use BSSID to connect */
	memset(&ssid_bssid.ssid, 0, sizeof(mlan_802_11_ssid));
	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT, &ssid_bssid)) {
		ret = -EFAULT;
		goto done;
	}

#ifdef REASSOCIATION
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.ssid,
			&bss_info.ssid, sizeof(mlan_802_11_ssid),
			sizeof(mlan_802_11_ssid));
	moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.bssid,
			&bss_info.bssid, MLAN_MAC_ADDR_LENGTH,
			sizeof(mlan_802_11_mac_addr));
#endif /* REASSOCIATION */

done:
	LEAVE();
	return ret;
}
#endif

/**
 * @brief Set BSS mode
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful, otherwise fail
 */
static int woal_priv_set_bss_mode(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	struct mwreq *mwr;
	t_u8 *data_ptr;
	t_u32 mode;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_SET_BSS_MODE));

	mwr = (struct mwreq *)data_ptr;
	mode = mwr->u.mode;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MODE;
	req->req_id = MLAN_IOCTL_BSS;
	req->action = MLAN_ACT_SET;

	switch (mode) {
	case MW_MODE_INFRA:
		bss->param.bss_mode = MLAN_BSS_MODE_INFRA;
		break;
	case MW_MODE_ADHOC:
		bss->param.bss_mode = MLAN_BSS_MODE_IBSS;
		break;
	case MW_MODE_AUTO:
		bss->param.bss_mode = MLAN_BSS_MODE_AUTO;
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

#ifdef STA_SUPPORT
/**
 * @brief Set power management
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful, otherwise fail
 */
static int woal_priv_set_power(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	struct mwreq *mwr;
	t_u8 *data_ptr;
	int ret = 0, disabled;

	ENTER();

	if (moal_extflg_isset(priv->phandle, EXT_HW_TEST)) {
		PRINTM(MIOCTL, "block set power in hw_test mode\n");
		LEAVE();
		return ret;
	}

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_SET_POWER));

	mwr = (struct mwreq *)data_ptr;
	disabled = mwr->u.power.disabled;

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_power_mgmt(priv, MLAN_ACT_SET, &disabled,
				    mwr->u.power.flags, MOAL_IOCTL_WAIT)) {
		return -EFAULT;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Set essid
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful, otherwise fail
 */
static int woal_priv_set_essid(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	mlan_802_11_ssid req_ssid;
	mlan_ssid_bssid ssid_bssid;
#ifdef REASSOCIATION
	moal_handle *handle = priv->phandle;
	mlan_bss_info bss_info;
#endif
	int ret = 0;
	t_u32 mode = 0;
	struct mwreq *mwr;
	t_u8 *data_ptr;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_SET_ESSID));

	mwr = (struct mwreq *)data_ptr;

#ifdef REASSOCIATION
	/* Cancel re-association */
	priv->reassoc_required = MFALSE;

	if (MOAL_ACQ_SEMAPHORE_BLOCK(&handle->reassoc_sem)) {
		PRINTM(MERROR, "Acquire semaphore error, woal_set_essid\n");
		LEAVE();
		return -EBUSY;
	}
#endif /* REASSOCIATION */

	/* Check the size of the string */
	if (mwr->u.essid.length > MW_ESSID_MAX_SIZE - 1) {
		ret = -E2BIG;
		goto setessid_ret;
	}
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE);
	memset(&req_ssid, 0, sizeof(mlan_802_11_ssid));
	memset(&ssid_bssid, 0, sizeof(mlan_ssid_bssid));

	req_ssid.ssid_len = mwr->u.essid.length;

	/* Check if we asked for 'any' or 'particular' */
	if (!mwr->u.essid.flags) {
#ifdef REASSOCIATION
		if (!req_ssid.ssid_len) {
			memset(&priv->prev_ssid_bssid.ssid, 0x00,
			       sizeof(mlan_802_11_ssid));
			memset(&priv->prev_ssid_bssid.bssid, 0x00,
			       MLAN_MAC_ADDR_LENGTH);
			goto setessid_ret;
		}
#endif
		/* Do normal SSID scanning */
		if (MLAN_STATUS_SUCCESS !=
		    woal_request_scan(priv, MOAL_IOCTL_WAIT, NULL)) {
			ret = -EFAULT;
			goto setessid_ret;
		}
	} else {
		/* Set the SSID */
		moal_memcpy_ext(handle, req_ssid.ssid, mwr->u.essid.pointer,
				req_ssid.ssid_len, MLAN_MAX_SSID_LENGTH);
		if (!req_ssid.ssid_len ||
		    (MFALSE == woal_ssid_valid(&req_ssid))) {
			PRINTM(MERROR, "Invalid SSID - aborting set_essid\n");
			ret = -EINVAL;
			goto setessid_ret;
		}

		PRINTM(MINFO, "Requested new SSID = %s\n",
		       (char *)req_ssid.ssid);
		moal_memcpy_ext(handle, &ssid_bssid.ssid, &req_ssid,
				sizeof(mlan_802_11_ssid),
				sizeof(mlan_802_11_ssid));
		if (MTRUE == woal_is_connected(priv, &ssid_bssid)) {
			PRINTM(MIOCTL, "Already connect to the network\n");
			goto setessid_ret;
		}

		if (mwr->u.essid.flags != 0xFFFF) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_find_essid(priv, &ssid_bssid,
					    MOAL_IOCTL_WAIT)) {
				/* Do specific SSID scanning */
				if (MLAN_STATUS_SUCCESS !=
				    woal_request_scan(priv, MOAL_IOCTL_WAIT,
						      &req_ssid)) {
					ret = -EFAULT;
					goto setessid_ret;
				}
			}
		}
	}

	mode = woal_get_mode(priv, MOAL_IOCTL_WAIT);

	if (mode != MW_MODE_ADHOC) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_find_best_network(priv, MOAL_IOCTL_WAIT,
					   &ssid_bssid)) {
			ret = -EFAULT;
			goto setessid_ret;
		}
	}
#ifdef UAP_SUPPORT
	else if (MLAN_STATUS_SUCCESS !=
		 woal_find_best_network(priv, MOAL_IOCTL_WAIT, &ssid_bssid))
		/* Adhoc start, Check the channel command */
		woal_11h_channel_check_ioctl(priv, MOAL_IOCTL_WAIT);
#endif

	/* Connect to BSS by ESSID */
	memset(&ssid_bssid.bssid, 0, MLAN_MAC_ADDR_LENGTH);

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT, &ssid_bssid)) {
		ret = -EFAULT;
		goto setessid_ret;
	}

#ifdef REASSOCIATION
	memset(&bss_info, 0, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		ret = -EFAULT;
		goto setessid_ret;
	}
	moal_memcpy_ext(handle, &priv->prev_ssid_bssid.ssid, &bss_info.ssid,
			sizeof(mlan_802_11_ssid), sizeof(mlan_802_11_ssid));
	moal_memcpy_ext(handle, &priv->prev_ssid_bssid.bssid, &bss_info.bssid,
			MLAN_MAC_ADDR_LENGTH, sizeof(mlan_802_11_mac_addr));
#endif /* REASSOCIATION */

setessid_ret:
	if (priv->scan_type == MLAN_SCAN_TYPE_PASSIVE)
		woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE);
#ifdef REASSOCIATION
	MOAL_REL_SEMAPHORE(&handle->reassoc_sem);
#endif
	LEAVE();
	return ret;
}

/**
 *  @brief Set authentication mode parameters
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful, otherwise fail
 */
static int woal_priv_set_auth(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	struct mwreq *mwr;
	t_u8 *data_ptr;
	int ret = 0;
	t_u32 auth_mode = 0;
	t_u32 encrypt_mode = 0;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_SET_AUTH));

	mwr = (struct mwreq *)data_ptr;

	switch (mwr->u.param.flags & MW_AUTH_INDEX) {
	case MW_AUTH_CIPHER_PAIRWISE:
	case MW_AUTH_CIPHER_GROUP:
		if (mwr->u.param.value & MW_AUTH_CIPHER_NONE)
			encrypt_mode = MLAN_ENCRYPTION_MODE_NONE;
		else if (mwr->u.param.value & MW_AUTH_CIPHER_WEP40)
			encrypt_mode = MLAN_ENCRYPTION_MODE_WEP40;
		else if (mwr->u.param.value & MW_AUTH_CIPHER_WEP104)
			encrypt_mode = MLAN_ENCRYPTION_MODE_WEP104;
		else if (mwr->u.param.value & MW_AUTH_CIPHER_TKIP)
			encrypt_mode = MLAN_ENCRYPTION_MODE_TKIP;
		else if (mwr->u.param.value & MW_AUTH_CIPHER_CCMP)
			encrypt_mode = MLAN_ENCRYPTION_MODE_CCMP;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_encrypt_mode(priv, MOAL_IOCTL_WAIT, encrypt_mode))
			ret = -EFAULT;
		break;
	case MW_AUTH_80211_AUTH_ALG:
		switch (mwr->u.param.value) {
		case MW_AUTH_ALG_SHARED_KEY:
			PRINTM(MINFO, "Auth mode shared key!\n");
			auth_mode = MLAN_AUTH_MODE_SHARED;
			break;
		case MW_AUTH_ALG_LEAP:
			PRINTM(MINFO, "Auth mode LEAP!\n");
			auth_mode = MLAN_AUTH_MODE_NETWORKEAP;
			break;
		case MW_AUTH_ALG_OPEN_SYSTEM:
			PRINTM(MINFO, "Auth mode open!\n");
			auth_mode = MLAN_AUTH_MODE_OPEN;
			break;
		case MW_AUTH_ALG_SHARED_KEY | MW_AUTH_ALG_OPEN_SYSTEM:
		default:
			PRINTM(MINFO, "Auth mode auto!\n");
			auth_mode = MLAN_AUTH_MODE_AUTO;
			break;
		}
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_auth_mode(priv, MOAL_IOCTL_WAIT, auth_mode))
			ret = -EFAULT;
		break;
	case MW_AUTH_WPA_ENABLED:
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wpa_enable(priv, MOAL_IOCTL_WAIT,
					mwr->u.param.value))
			ret = -EFAULT;
		break;
#define MW_AUTH_WAPI_ENABLED 0x20
	case MW_AUTH_WAPI_ENABLED:
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_wapi_enable(priv, MOAL_IOCTL_WAIT,
					 mwr->u.param.value))
			ret = -EFAULT;
		break;
	case MW_AUTH_WPA_VERSION:
		/* set WPA_VERSION_DISABLED/VERSION_WPA/VERSION_WP2 */
		priv->wpa_version = mwr->u.param.value;
		break;
	case MW_AUTH_KEY_MGMT:
		/* set KEY_MGMT_802_1X/KEY_MGMT_PSK */
		priv->key_mgmt = mwr->u.param.value;
		break;
	case MW_AUTH_TKIP_COUNTERMEASURES:
	case MW_AUTH_DROP_UNENCRYPTED:
	case MW_AUTH_RX_UNENCRYPTED_EAPOL:
	case MW_AUTH_ROAMING_CONTROL:
	case MW_AUTH_PRIVACY_INVOKED:
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief Get current BSSID
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful else negative value
 */
static int woal_priv_get_ap(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	struct mwreq *mwr;
	t_u8 *data_ptr;
	int ret = 0;
	mlan_bss_info bss_info;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_GET_AP));

	mwr = (struct mwreq *)data_ptr;

	memset(&bss_info, 0, sizeof(bss_info));

	ret = woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
	if (ret != MLAN_STATUS_SUCCESS)
		return -EFAULT;

	if (bss_info.media_connected == MTRUE) {
		moal_memcpy_ext(priv->phandle, mwr->u.ap_addr.sa_data,
				&bss_info.bssid, MLAN_MAC_ADDR_LENGTH,
				sizeof(mwr->u.ap_addr.sa_data));
	} else {
		memset(mwr->u.ap_addr.sa_data, 0, MLAN_MAC_ADDR_LENGTH);
	}
	mwr->u.ap_addr.sa_family = ARPHRD_ETHER;
	ret = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_AP) + sizeof(struct mwreq);

	LEAVE();
	return ret;
}

/**
 *  @brief  Get power management
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful else negative value
 */
static int woal_priv_get_power(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	struct mwreq *mwr;
	t_u8 *data_ptr;
	int ret = 0, ps_mode;

	ENTER();

	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_GET_POWER));

	mwr = (struct mwreq *)data_ptr;

	if (MLAN_STATUS_SUCCESS != woal_set_get_power_mgmt(priv, MLAN_ACT_GET,
							   &ps_mode, 0,
							   MOAL_IOCTL_WAIT)) {
		return -EFAULT;
	}

	if (ps_mode)
		mwr->u.power.disabled = 0;
	else
		mwr->u.power.disabled = 1;

	mwr->u.power.value = 0;
	ret = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_POWER) +
	      sizeof(struct mwreq);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get power save mode
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_set_get_psmode(moal_private *priv, t_u8 *respbuf,
				    t_u32 respbuflen)
{
	int ret = 0;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	t_u32 action = MLAN_ACT_GET;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PSMODE);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		action = MLAN_ACT_SET;
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	/* Flip the value */
	data = !data;

	if (MLAN_STATUS_SUCCESS !=
	    woal_set_get_power_mgmt(priv, action, &data, 0, MOAL_IOCTL_WAIT)) {
		ret = -EFAULT;
		goto done;
	}

	if (action == MLAN_ACT_SET)
		data = !data;

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	LEAVE();
	return ret;
}
#endif /* STA_SUPPORT */

/**
 * @brief Performs warm reset
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         Number of bytes written if successful else negative value
 */
static int woal_priv_warmreset(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int ret = 0;
	moal_handle *handle = priv->phandle;
	moal_handle *ref_handle;
	moal_private *ref_priv;
	ENTER();
	ret = woal_pre_warmreset(priv);
	if (ret)
		goto done;
	ref_handle = (moal_handle *)handle->pref_mac;
	if (ref_handle) {
		ref_priv = woal_get_priv(ref_handle, MLAN_BSS_ROLE_ANY);
		if (ref_priv) {
			ret = woal_pre_warmreset(ref_priv);
			if (ret)
				goto done;
			ret = woal_warmreset(ref_priv);
			if (ret)
				goto done;
		}
	}
	ret = woal_warmreset(priv);
done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get TX power configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         0 --success, otherwise fail
 */
static int woal_priv_txpowercfg(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int data[5];
	int user_data_len;
	int ret = 0;
	mlan_bss_info bss_info;
	mlan_ds_power_cfg *pcfg = NULL;
	mlan_ioctl_req *req = NULL;
	t_u8 *arguments = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	memset(data, 0, sizeof(data));
	memset(&bss_info, 0, sizeof(bss_info));
	woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);

	if (strlen(respbuf) ==
	    (strlen(CMD_NXP) + strlen(PRIV_CMD_TXPOWERCFG))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		arguments =
			respbuf + strlen(CMD_NXP) + strlen(PRIV_CMD_TXPOWERCFG);
		parse_arguments(arguments, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_power_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pcfg = (mlan_ds_power_cfg *)req->pbuf;
	pcfg->sub_command = MLAN_OID_POWER_CFG_EXT;
	req->req_id = MLAN_IOCTL_POWER_CFG;

	if (!user_data_len)
		req->action = MLAN_ACT_GET;
	else {
		/* SET operation */
		req->action = MLAN_ACT_SET;
		if (sizeof(int) * user_data_len > sizeof(data)) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
		switch (user_data_len) {
		case 1:
			if (data[0] == 0xFF)
				pcfg->param.power_ext.power_group[0]
					.rate_format = TX_PWR_CFG_AUTO_CTRL_OFF;
			else
				ret = -EINVAL;
			break;
		case 3:
		case 5:
			switch (data[0]) {
			case 0: /* LG */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_LG;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW20;
				break;
			case 1: /* 20 MHz HT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_HT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW20;
				break;
			case 2: /* 40 MHz HT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_HT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW40;
				break;
			case 3: /* 1 NSS 20 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW20;
				pcfg->param.power_ext.power_group[0].nss = 1;
				break;
			case 4: /* 2 NSS 20 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW20;
				pcfg->param.power_ext.power_group[0].nss = 2;
				break;
			case 5: /* 1 NSS 40 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW40;
				pcfg->param.power_ext.power_group[0].nss = 1;
				break;
			case 6: /* 2 NSS 40 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_HT_BW40;
				pcfg->param.power_ext.power_group[0].nss = 2;
				break;
			case 7: /* 1 NSS 80 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_VHT_BW80;
				pcfg->param.power_ext.power_group[0].nss = 1;
				break;
			case 8: /* 2 NSS 80 MHZ VHT */
				pcfg->param.power_ext.power_group[0]
					.rate_format = MLAN_RATE_FORMAT_VHT;
				pcfg->param.power_ext.power_group[0].bandwidth =
					MLAN_VHT_BW80;
				pcfg->param.power_ext.power_group[0].nss = 2;
				break;
			default:
				ret = -EINVAL;
				break;
			}
			pcfg->param.power_ext.power_group[0].first_rate_ind =
				data[1];
			pcfg->param.power_ext.power_group[0].last_rate_ind =
				data[1];
			if (data[2] < bss_info.min_power_level) {
				PRINTM(MERROR,
				       "The set powercfg rate value %d dBm is out of range (%d dBm-%d dBm)!\n",
				       data[2], (int)bss_info.min_power_level,
				       (int)bss_info.max_power_level);
				ret = -EINVAL;
				break;
			}
			if (data[2] > bss_info.max_power_level) {
				PRINTM(MERROR,
				       "The set powercfg rate value %d dBm is out of range (%d dBm-%d dBm)!\n",
				       data[2], (int)bss_info.min_power_level,
				       (int)bss_info.max_power_level);
				ret = -EINVAL;
				break;
			}
			pcfg->param.power_ext.power_group[0].power_min =
				data[2];
			pcfg->param.power_ext.power_group[0].power_max =
				data[2];
			pcfg->param.power_ext.power_group[0].power_step = 0;
			pcfg->param.power_ext.num_pwr_grp = 1;

			break;
		default:
			ret = -EINVAL;
			break;
		}
		if (ret)
			goto done;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (!user_data_len) {
		/* GET operation */
		moal_memcpy_ext(priv->phandle, respbuf,
				(t_u8 *)&pcfg->param.power_ext,
				sizeof(pcfg->param.power_ext.num_pwr_grp) +
					(MIN(pcfg->param.power_ext.num_pwr_grp,
					     MAX_POWER_GROUP) *
					 sizeof(mlan_power_group)),
				respbuflen);
		ret = sizeof(pcfg->param.power_ext.num_pwr_grp) +
		      (MIN(pcfg->param.power_ext.num_pwr_grp, MAX_POWER_GROUP) *
		       sizeof(mlan_power_group));
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get PS configuration parameters
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_pscfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data[7] = {0}, ret = 0;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int allowed = 3;
	int i = 3;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	allowed++; /* For beacon missing timeout parameter */
	allowed += 2; /* For delay to PS and PS mode parameters */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PSCFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	if (user_data_len && user_data_len > allowed) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_PS_CFG;
	req->req_id = MLAN_IOCTL_PM_CFG;
	if (user_data_len) {
		if ((data[0] < PS_NULL_DISABLE)) {
			PRINTM(MERROR,
			       "Invalid argument for PS null interval\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[1] != MRVDRV_IGNORE_MULTIPLE_DTIM) &&
		    (data[1] != MRVDRV_MATCH_CLOSEST_DTIM) &&
		    ((data[1] < MRVDRV_MIN_MULTIPLE_DTIM) ||
		     (data[1] > MRVDRV_MAX_MULTIPLE_DTIM))) {
			PRINTM(MERROR, "Invalid argument for multiple DTIM\n");
			ret = -EINVAL;
			goto done;
		}

		if ((data[2] < MRVDRV_MIN_LISTEN_INTERVAL) &&
		    (data[2] != MRVDRV_LISTEN_INTERVAL_DISABLE)) {
			PRINTM(MERROR,
			       "Invalid argument for listen interval\n");
			ret = -EINVAL;
			goto done;
		}

		if ((data[i] != DISABLE_BCN_MISS_TO) &&
		    ((data[i] < MIN_BCN_MISS_TO) ||
		     (data[i] > MAX_BCN_MISS_TO))) {
			PRINTM(MERROR,
			       "Invalid argument for beacon miss timeout\n");
			ret = -EINVAL;
			goto done;
		}
		i++;
		if (user_data_len < allowed - 1)
			data[i] = DELAY_TO_PS_UNCHANGED;
		else if ((data[i] < MIN_DELAY_TO_PS) ||
			 (data[i] > MAX_DELAY_TO_PS)) {
			PRINTM(MERROR, "Invalid argument for delay to PS\n");
			ret = -EINVAL;
			goto done;
		}
		i++;
		if ((data[i] != PS_MODE_UNCHANGED) &&
		    (data[i] != PS_MODE_AUTO) && (data[i] != PS_MODE_POLL) &&
		    (data[i] != PS_MODE_NULL)) {
			PRINTM(MERROR, "Invalid argument for PS mode\n");
			ret = -EINVAL;
			goto done;
		}
		i++;
		req->action = MLAN_ACT_SET;
		moal_memcpy_ext(priv->phandle, &pm_cfg->param.ps_cfg, data,
				sizeof(data), sizeof(pm_cfg->param.ps_cfg));
	} else
		req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, data, &pm_cfg->param.ps_cfg,
			MIN((sizeof(int) * allowed),
			    sizeof(pm_cfg->param.ps_cfg)),
			sizeof(data));
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
			sizeof(int) * allowed, respbuflen);
	ret = sizeof(int) * allowed;
	if (req->action == MLAN_ACT_SET) {
		pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
		pm_cfg->sub_command = MLAN_OID_PM_CFG_IEEE_PS;
		pm_cfg->param.ps_mode = 1;
		req->req_id = MLAN_IOCTL_PM_CFG;
		req->action = MLAN_ACT_SET;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get PS configuration parameters
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_bcntimeoutcfg(moal_private *priv, t_u8 *respbuf,
				   t_u32 respbuflen)
{
	int data[4] = {0}, ret = 0;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int allowed = 4;
	int i = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_BCNTIMEOUTCFG);

	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	if (user_data_len != allowed) {
		PRINTM(MERROR, "Invalid args num: input=%d allowed=%d\n",
		       user_data_len, allowed);
		ret = -EINVAL;
		goto done;
	}
	for (i = 0; i < allowed; i++) {
		if (data[i] <= 0) {
			PRINTM(MERROR, "Invalid data[%d]=%d\n", i, data[i]);
			ret = -EINVAL;
			goto done;
		}
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_BCN_TIMEOUT;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = MLAN_ACT_SET;
	pm_cfg->param.bcn_timeout.bcn_miss_tmo_window = (t_u16)data[0];
	pm_cfg->param.bcn_timeout.bcn_miss_tmo_period = (t_u16)data[1];
	pm_cfg->param.bcn_timeout.bcn_rq_tmo_window = (t_u16)data[2];
	pm_cfg->param.bcn_timeout.bcn_rq_tmo_period = (t_u16)data[3];

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
 * @brief Set/Get sleep period
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_sleeppd(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_pm_cfg *pm_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pm_cfg = (mlan_ds_pm_cfg *)req->pbuf;
	pm_cfg->sub_command = MLAN_OID_PM_CFG_SLEEP_PD;
	req->req_id = MLAN_IOCTL_PM_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SLEEPPD);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len) {
		if ((data <= MAX_SLEEP_PERIOD && data >= MIN_SLEEP_PERIOD) ||
		    (data == 0) || (data == SLEEP_PERIOD_RESERVED_FF)) {
			req->action = MLAN_ACT_SET;
			pm_cfg->param.sleep_period = data;
		} else {
			ret = -EINVAL;
			goto done;
		}
	} else
		req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (!user_data_len) {
		data = pm_cfg->param.sleep_period;
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Set/Get Tx control flag
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_txcontrol(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_TXCONTROL;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TXCONTROL);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len) {
		req->action = MLAN_ACT_SET;
		misc_cfg->param.tx_control = (t_u32)data;
	} else {
		req->action = MLAN_ACT_GET;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (!user_data_len) {
		data = misc_cfg->param.tx_control;
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Read/Write adapter registers value
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_regrdwr(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data[3];
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_reg_mem *reg_mem = NULL;
	int user_data_len = 0, header_len = 0;
	t_u8 *arguments = NULL, *space_ind = NULL;
	t_u32 is_negative_val = MFALSE;
	mlan_status status = MLAN_STATUS_SUCCESS;
	gfp_t flag;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_reg_mem));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	reg_mem = (mlan_ds_reg_mem *)req->pbuf;
	reg_mem->sub_command = MLAN_OID_REG_RW;
	req->req_id = MLAN_IOCTL_REG_MEM;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_REGRDWR);

	if (strlen(respbuf) == header_len) {
		ret = -EINVAL;
		goto done;
	}
	/* SET operation */
	memset((char *)data, 0, sizeof(data));
	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	arguments = kzalloc(strlen(respbuf) * sizeof(char), flag);
	if (arguments == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	strcpy(arguments, respbuf + header_len);
	space_ind = strstr((char *)arguments, " ");
	if (space_ind)
		space_ind = strstr(space_ind + 1, " ");
	if (space_ind) {
		if (*(char *)(space_ind + 1) == '-') {
			is_negative_val = MTRUE;
			arguments[space_ind + 1 - arguments] = '\0';
			strcat(arguments, space_ind + 2);
		}
	}
	parse_arguments(arguments, data, ARRAY_SIZE(data), &user_data_len);
	if (is_negative_val == MTRUE)
		data[2] *= -1;

	if (user_data_len == 2) {
		req->action = MLAN_ACT_GET;
	} else if (user_data_len == 3) {
		req->action = MLAN_ACT_SET;
	} else {
		ret = -EINVAL;
		goto done;
	}

	reg_mem->param.reg_rw.type = (t_u32)data[0];
	reg_mem->param.reg_rw.offset = (t_u32)data[1];
	if (user_data_len == 3)
		reg_mem->param.reg_rw.value = (t_u32)data[2];

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		moal_memcpy_ext(priv->phandle, respbuf, &reg_mem->param.reg_rw,
				sizeof(reg_mem->param.reg_rw), respbuflen);
		ret = sizeof(reg_mem->param.reg_rw);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	kfree(arguments);
	LEAVE();
	return ret;
}

/**
 * @brief Read the EEPROM contents of the card
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_rdeeprom(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	int data[2];
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_reg_mem *reg_mem = NULL;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_reg_mem));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	reg_mem = (mlan_ds_reg_mem *)req->pbuf;
	reg_mem->sub_command = MLAN_OID_EEPROM_RD;
	req->req_id = MLAN_IOCTL_REG_MEM;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RDEEPROM);

	if (strlen(respbuf) == header_len) {
		ret = -EINVAL;
		goto done;
	}
	/* SET operation */
	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);

	if (user_data_len == 2) {
		req->action = MLAN_ACT_GET;
	} else {
		ret = -EINVAL;
		goto done;
	}

	reg_mem->param.rd_eeprom.offset = (t_u16)data[0];
	reg_mem->param.rd_eeprom.byte_count = (t_u16)data[1];

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		moal_memcpy_ext(priv->phandle, respbuf,
				&reg_mem->param.rd_eeprom,
				sizeof(reg_mem->param.rd_eeprom), respbuflen);
		ret = sizeof(reg_mem->param.rd_eeprom);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief Read/Write device memory value
 *
 * @param priv         A pointer to moal_private structure
 * @param respbuf      A pointer to response buffer
 * @param respbuflen   Available length of response buffer
 *
 * @return         0 --success, otherwise fail
 */
static int woal_priv_memrdwr(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data[2];
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_reg_mem *reg_mem = NULL;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_reg_mem));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	reg_mem = (mlan_ds_reg_mem *)req->pbuf;
	reg_mem->sub_command = MLAN_OID_MEM_RW;
	req->req_id = MLAN_IOCTL_REG_MEM;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MEMRDWR);

	if (strlen(respbuf) == header_len) {
		ret = -EINVAL;
		goto done;
	}
	/* SET operation */
	memset((char *)data, 0, sizeof(data));
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);

	if (user_data_len == 1) {
		PRINTM(MINFO, "MEM_RW: GET\n");
		req->action = MLAN_ACT_GET;
	} else if (user_data_len == 2) {
		PRINTM(MINFO, "MEM_RW: SET\n");
		req->action = MLAN_ACT_SET;
	} else {
		ret = -EINVAL;
		goto done;
	}

	reg_mem->param.mem_rw.addr = (t_u32)data[0];
	if (user_data_len == 2)
		reg_mem->param.mem_rw.value = (t_u32)data[1];

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		moal_memcpy_ext(priv->phandle, respbuf, &reg_mem->param.mem_rw,
				sizeof(reg_mem->param.mem_rw), respbuflen);
		ret = sizeof(reg_mem->param.mem_rw);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef SDIO
/**
 *  @brief Cmd52 read/write register
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             MLAN_STATUS_SUCCESS --success, otherwise fail
 */
static int woal_priv_sdcmd52rw(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	t_u8 rw = 0, func, data = 0;
	int buf[3], reg, ret = MLAN_STATUS_SUCCESS;
	int user_data_len = 0, header_len = 0;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SDCMD52RW);
	memset((t_u8 *)buf, 0, sizeof(buf));

	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	parse_arguments(respbuf + header_len, buf, ARRAY_SIZE(buf),
			&user_data_len);

	if (user_data_len < 2 || user_data_len > 3) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}

	func = (t_u8)buf[0];
	if (func > 7) {
		PRINTM(MERROR, "Invalid function number!\n");
		ret = -EINVAL;
		goto done;
	}
	reg = (t_u32)buf[1];
	if (user_data_len == 2) {
		rw = 0; /* CMD52 read */
		PRINTM(MINFO, "Cmd52 read, func=%d, reg=0x%08X\n", func, reg);
	}
	if (user_data_len == 3) {
		rw = 1; /* CMD52 write */
		data = (t_u8)buf[2];
		PRINTM(MINFO, "Cmd52 write, func=%d, reg=0x%08X, data=0x%02X\n",
		       func, reg, data);
	}

	if (!rw) {
#ifdef SDIO_MMC
		sdio_claim_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (func)
			data = sdio_readb(
				((struct sdio_mmc_card *)priv->phandle->card)
					->func,
				reg, &ret);
		else
			data = sdio_f0_readb(
				((struct sdio_mmc_card *)priv->phandle->card)
					->func,
				reg, &ret);
		sdio_release_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (ret) {
			PRINTM(MERROR,
			       "sdio_readb: reading register 0x%X failed\n",
			       reg);
			goto done;
		}
#else
		if (sdio_read_ioreg(priv->phandle->card, func, reg, &data) <
		    0) {
			PRINTM(MERROR,
			       "sdio_read_ioreg: reading register 0x%X failed\n",
			       reg);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#endif /* SDIO_MMC */
	} else {
#ifdef SDIO_MMC
		sdio_claim_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (func)
			sdio_writeb(
				((struct sdio_mmc_card *)priv->phandle->card)
					->func,
				data, reg, &ret);
		else
			sdio_f0_writeb(
				((struct sdio_mmc_card *)priv->phandle->card)
					->func,
				data, reg, &ret);
		sdio_release_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (ret) {
			PRINTM(MERROR,
			       "sdio_writeb: writing register 0x%X failed\n",
			       reg);
			goto done;
		}
#else
		if (sdio_write_ioreg(priv->phandle->card, func, reg, data) <
		    0) {
			PRINTM(MERROR,
			       "sdio_write_ioreg: writing register 0x%X failed\n",
			       reg);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
#endif /* SDIO_MMC */
	}

	/* Action = GET */
	buf[0] = data;

	moal_memcpy_ext(priv->phandle, respbuf, &buf, sizeof(int), respbuflen);
	ret = sizeof(int);

done:
	LEAVE();
	return ret;
}
#endif /* SDIO */

/**
 *  @brief Set / Get Auto ARP Response configuration
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_get_auto_arp(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int data[4];
	int user_data_len = 0;
	int ret = 0;
	moal_handle *handle = NULL;

	ENTER();

	if (priv == NULL) {
		PRINTM(MERROR, "Invalid priv\n");
		goto done;
	}
	handle = priv->phandle;

	if (strlen(respbuf) == (strlen(CMD_NXP) + strlen(PRIV_CMD_AUTO_ARP))) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_AUTO_ARP),
				data, ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len) {
		/* Get the enable/disable value from user */
		handle->hs_auto_arp = data[0];
		PRINTM(MIOCTL, "Auto ARP : %s\n",
		       handle->hs_auto_arp ? "enable" : "disable");
	}

	moal_memcpy_ext(handle, respbuf, &handle->hs_auto_arp,
			sizeof(handle->hs_auto_arp), respbuflen);
	ret = sizeof(handle->hs_auto_arp);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Get/Set deauth control
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_deauth_ctrl(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	mlan_ds_snmp_mib *cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0, header_len = 0, data = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_snmp_mib));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_snmp_mib *)req->pbuf;
	cfg->sub_command = MLAN_OID_SNMP_MIB_CTRL_DEAUTH;
	req->req_id = MLAN_IOCTL_SNMP_MIB;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DEAUTH_CTRL);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of args! %d\n",
			       user_data_len);
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		cfg->param.deauthctrl = (t_u8)data;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data = (int)cfg->param.deauthctrl;
	moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#define MRVL_TLV_HEADER_SIZE 4
/**
 *  @brief              Get/Set per packet Txctl and Rxinfo configuration
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_per_pkt_cfg(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	t_u8 *pos = NULL;
	int left_len, header_len = 0;
	mlan_per_pkt_cfg *perpkt = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PER_PKT_CFG);
	pos = respbuf + header_len;
	left_len = respbuflen - header_len;

	if (priv->phandle->card_info->per_pkt_cfg_support == 0) {
		PRINTM(MERROR, "Device not support per packet configuration\n");
		ret = -EFAULT;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_PER_PKT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (*pos == 0) {
		/* GET operation */
		pos++;
		if (priv->tx_protocols.protocol_num) {
			perpkt = (mlan_per_pkt_cfg *)pos;
			perpkt->type = TLV_TYPE_PER_PKT_CFG;
			perpkt->tx_rx_control = TX_PKT_CTRL;
			perpkt->proto_type_num =
				priv->tx_protocols.protocol_num;
			moal_memcpy_ext(priv->phandle, perpkt->ether_type,
					priv->tx_protocols.protocols,
					perpkt->proto_type_num * sizeof(t_u16),
					MAX_NUM_ETHER_TYPE * sizeof(t_u16));
			perpkt->len =
				(perpkt->proto_type_num + 1) * sizeof(t_u16);
			pos += perpkt->len + MRVL_TLV_HEADER_SIZE;
		}
		if (priv->rx_protocols.protocol_num) {
			perpkt = (mlan_per_pkt_cfg *)pos;
			perpkt->type = TLV_TYPE_PER_PKT_CFG;
			perpkt->tx_rx_control = RX_PKT_INFO;
			perpkt->proto_type_num =
				priv->rx_protocols.protocol_num;
			moal_memcpy_ext(priv->phandle, perpkt->ether_type,
					priv->rx_protocols.protocols,
					perpkt->proto_type_num * sizeof(t_u16),
					MAX_NUM_ETHER_TYPE * sizeof(t_u16));
			perpkt->len =
				(perpkt->proto_type_num + 1) * sizeof(t_u16);
			pos += perpkt->len + MRVL_TLV_HEADER_SIZE;
		}
		ret = pos - respbuf;
		goto done;
	} else if (*pos == 1) {
		/* SET operation */
		req->action = MLAN_ACT_SET;
		pos++;
		left_len--;
		while (*pos == TLV_TYPE_PER_PKT_CFG && (left_len > 2)) {
			perpkt = (mlan_per_pkt_cfg *)pos;
			if (perpkt->tx_rx_control & TX_PKT_CTRL) {
				priv->tx_protocols.protocol_num =
					perpkt->proto_type_num;
				if (perpkt->proto_type_num <=
				    MAX_NUM_ETHER_TYPE)
					moal_memcpy_ext(
						priv->phandle,
						priv->tx_protocols.protocols,
						perpkt->ether_type,
						perpkt->proto_type_num *
							sizeof(t_u16),
						MAX_NUM_ETHER_TYPE *
							sizeof(t_u16));
			}
			if (perpkt->tx_rx_control & RX_PKT_INFO) {
				priv->rx_protocols.protocol_num =
					perpkt->proto_type_num;
				if (perpkt->proto_type_num <=
				    MAX_NUM_ETHER_TYPE)
					moal_memcpy_ext(
						priv->phandle,
						priv->rx_protocols.protocols,
						perpkt->ether_type,
						perpkt->proto_type_num *
							sizeof(t_u16),
						MAX_NUM_ETHER_TYPE *
							sizeof(t_u16));
			}
			if (!perpkt->tx_rx_control) {
				memset(&priv->tx_protocols, 0,
				       sizeof(dot11_protocol));
				memset(&priv->rx_protocols, 0,
				       sizeof(dot11_protocol));
			}
			pos += perpkt->len + MRVL_TLV_HEADER_SIZE;
			left_len -= (perpkt->len + MRVL_TLV_HEADER_SIZE);
		}
	} else
		goto done;

	if (perpkt != NULL)
		misc->param.txrx_pkt_ctrl = perpkt->tx_rx_control;
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
 *  @brief              Get Region Channel Power
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_chnrgpwr(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int header_len = 0;
	t_u8 *pos = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	header_len = strlen(PRIV_CMD_GET_CHNRGPWR);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_REGIONPWR_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	ret = header_len + sizeof(t_u16) + misc->param.rgchnpwr_cfg.length;
	pos = respbuf + header_len;
	moal_memcpy_ext(priv->phandle, pos, &misc->param.rgchnpwr_cfg,
			sizeof(t_u16) + misc->param.rgchnpwr_cfg.length,
			respbuflen - header_len);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief              Get TX/RX histogram statistic
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_txpwrlimit(moal_private *priv, t_u8 *respbuf,
				    t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ds_misc_chan_trpc_cfg *trpc_cfg = NULL;
	int header_len = 0;
	t_u8 *pos = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	header_len = strlen(PRIV_CMD_GET_TXPWR_LIMIT);
	trpc_cfg = (mlan_ds_misc_chan_trpc_cfg *)(respbuf + header_len);
	if ((trpc_cfg->sub_band != 0) && (trpc_cfg->sub_band != 0x10) &&
	    (trpc_cfg->sub_band != 0x11) && (trpc_cfg->sub_band != 0x12)) {
		PRINTM(MERROR, "Invalid subband=0x%x\n", trpc_cfg->sub_band);
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_CHAN_TRPC_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	misc->param.trpc_cfg.sub_band = trpc_cfg->sub_band;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	ret = header_len + sizeof(t_u16) + sizeof(t_u16) +
	      misc->param.trpc_cfg.length;
	pos = respbuf + header_len;
	moal_memcpy_ext(priv->phandle, pos, &misc->param.trpc_cfg,
			sizeof(t_u16) + sizeof(t_u16) +
				misc->param.trpc_cfg.length,
			respbuflen - header_len);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/**
 *  @brief              Get TX/RX histogram statistic
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_getcfgchanlist(moal_private *priv, t_u8 *respbuf,
				    t_u32 respbuflen)
{
	int ret = 0;
	int num_chan = 0;
	wlan_ieee80211_chan_list *plist = NULL;
	struct ieee80211_supported_band *sband;
	struct wiphy *wiphy = NULL;
	int i;

	ENTER();
	if (priv && priv->wdev)
		wiphy = priv->wdev->wiphy;
	if (!wiphy) {
		PRINTM(MERROR, "wiphy is NULL\n");
		ret = -EFAULT;
		goto done;
	}
	plist = (wlan_ieee80211_chan_list *)respbuf;
	sband = wiphy->bands[NL80211_BAND_2GHZ];
	if (sband) {
		num_chan += sband->n_channels;
		for (i = 0; i < sband->n_channels; i++) {
			plist->chan_list[i].center_freq =
				sband->channels[i].center_freq;
			plist->chan_list[i].hw_value =
				sband->channels[i].hw_value;
			plist->chan_list[i].flags = sband->channels[i].flags;
			plist->chan_list[i].max_power =
				sband->channels[i].max_power;
		}
	}
	sband = wiphy->bands[NL80211_BAND_5GHZ];
	if (sband) {
		for (i = 0; i < sband->n_channels; i++) {
			plist->chan_list[i + num_chan].center_freq =
				sband->channels[i].center_freq;
			plist->chan_list[i + num_chan].hw_value =
				sband->channels[i].hw_value;
			plist->chan_list[i + num_chan].flags =
				sband->channels[i].flags;
			plist->chan_list[i + num_chan].max_power =
				sband->channels[i].max_power;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 13)
			plist->chan_list[i + num_chan].dfs_state =
				sband->channels[i].dfs_state;
#endif
		}
		num_chan += sband->n_channels;
	}
	plist->num_chan = num_chan;
	ret = sizeof(wlan_ieee80211_chan_list) +
	      sizeof(wlan_ieee80211_chan) * num_chan;
done:
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief              Get TX/RX histogram statistic
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_rx_tx_histogram(moal_private *priv, t_u8 *respbuf,
					 t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	tx_rx_histogram *tx_rx_info = NULL;
	int header_len = 0;
	t_u8 *pos = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	header_len = strlen(PRIV_CMD_TX_RX_HISTOGRAM);
	tx_rx_info = (tx_rx_histogram *)(respbuf + header_len);
	if (tx_rx_info->enable > 2 ||
	    (tx_rx_info->enable == GET_TX_RX_HISTOGRAM &&
	     (tx_rx_info->action > 3 || tx_rx_info->action <= 0))) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_GET_TX_RX_HISTOGRAM;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (tx_rx_info->enable == GET_TX_RX_HISTOGRAM) {
		misc->param.tx_rx_histogram.enable = ENABLE_TX_RX_HISTOGRAM;
		misc->param.tx_rx_histogram.action = (t_u16)tx_rx_info->action;
	} else {
		misc->param.tx_rx_histogram.enable = tx_rx_info->enable;
		misc->param.tx_rx_histogram.action |=
			FLAG_TX_HISTOGRAM | FLAG_RX_HISTOGRAM;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	ret = header_len + 2 * sizeof(t_u8);
	if (tx_rx_info->enable & GET_TX_RX_HISTOGRAM) {
		pos = respbuf + header_len + 2 * sizeof(t_u8);
		/* Save tx/rx histogram size */
		moal_memcpy_ext(priv->phandle, pos,
				&misc->param.tx_rx_histogram.size,
				sizeof(misc->param.tx_rx_histogram.size),
				respbuflen - (header_len + 2 * sizeof(t_u8)));
		ret += sizeof(misc->param.tx_rx_histogram.size);
		pos += sizeof(misc->param.tx_rx_histogram.size);
		moal_memcpy_ext(
			priv->phandle, pos, &misc->param.tx_rx_histogram.value,
			misc->param.tx_rx_histogram.size,
			respbuflen - (header_len + 2 * sizeof(t_u8)) -
				sizeof(misc->param.tx_rx_histogram.size));
		ret += misc->param.tx_rx_histogram.size;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get hotspot mode configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         0 --success, otherwise fail
 */
int woal_priv_hotspotcfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_HOTSPOTCFG);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
	}
	if (user_data_len >= 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	} else {
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
		if (req == NULL) {
			ret = -ENOMEM;
			goto done;
		}

		cfg = (mlan_ds_misc_cfg *)req->pbuf;
		if (user_data_len == 0) {
			req->action = MLAN_ACT_GET;
		} else {
			cfg->param.hotspot_cfg = data;
			req->action = MLAN_ACT_SET;
		}
	}
	cfg->sub_command = MLAN_OID_MISC_HOTSPOT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	data = cfg->param.hotspot_cfg;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Mgmt Frame passthru mask
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         0 --success, otherwise fail
 */
int woal_priv_mgmt_frame_passthru_ctrl(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen)
{
	int ret = 0;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *mgmt_cfg = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MGMT_FRAME_CTRL);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
	}

	if (user_data_len >= 2) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	} else {
		req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
		if (req == NULL) {
			ret = -ENOMEM;
			goto done;
		}
		mgmt_cfg = (mlan_ds_misc_cfg *)req->pbuf;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		mgmt_cfg->sub_command = MLAN_OID_MISC_RX_MGMT_IND;

		if (user_data_len == 0) { /* Get */
			req->action = MLAN_ACT_GET;
		} else { /* Set */
			mgmt_cfg->param.mgmt_subtype_mask = data;
			req->action = MLAN_ACT_SET;
		}
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data = mgmt_cfg->param.mgmt_subtype_mask;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Private IOCTL entry to send an ADDTS TSPEC
 *
 *  Receive a ADDTS command from the application.  The command structure
 *    contains a TSPEC and timeout in milliseconds.  The timeout is performed
 *    in the firmware after the ADDTS command frame is sent.
 *
 *  The TSPEC is received in the API as an opaque block. The firmware will
 *    send the entire data block, including the bytes after the TSPEC.  This
 *    is done to allow extra IEs to be packaged with the TSPEC in the ADDTS
 *    action frame.
 *
 *  The IOCTL structure contains two return fields:
 *    - The firmware command result, which indicates failure and timeouts
 *    - The IEEE Status code which contains the corresponding value from
 *      any ADDTS response frame received.
 *
 *  In addition, the opaque TSPEC data block passed in is replaced with the
 *    TSPEC received in the ADDTS response frame.  In case of failure, the
 *    AP may modify the TSPEC on return and in the case of success, the
 *    medium time is returned as calculated by the AP.  Along with the TSPEC,
 *    any IEs that are sent in the ADDTS response are also returned and can be
 *    parsed using the IOCTL length as an indicator of extra elements.
 *
 *  The return value to the application layer indicates a driver execution
 *    success or failure.  A successful return could still indicate a firmware
 *    failure or AP negotiation failure via the commandResult field copied
 *    back to the application.
 *
 *  @param priv    Pointer to the mlan_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_priv_wmm_addts_req_ioctl(moal_private *priv, t_u8 *respbuf,
					 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *cfg = NULL;
	wlan_ioctl_wmm_addts_req_t addts_ioctl;
	int ret = 0, header_len = 0, copy_len = sizeof(addts_ioctl);
	t_u8 *data_ptr;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ADDTS);
	data_ptr = respbuf + header_len;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_WMM_CFG;
	cfg = (mlan_ds_wmm_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_WMM_CFG_ADDTS;

	memset(&addts_ioctl, 0x00, sizeof(addts_ioctl));

	moal_memcpy_ext(priv->phandle, (t_u8 *)&addts_ioctl, data_ptr,
			sizeof(addts_ioctl), sizeof(addts_ioctl));

	cfg->param.addts.timeout = addts_ioctl.timeout_ms;
	cfg->param.addts.ie_data_len = addts_ioctl.ie_data_len;

	moal_memcpy_ext(priv->phandle, cfg->param.addts.ie_data,
			addts_ioctl.ie_data, cfg->param.addts.ie_data_len,
			MLAN_WMM_TSPEC_SIZE + MLAN_WMM_ADDTS_EXTRA_IE_BYTES);

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	addts_ioctl.cmd_result = cfg->param.addts.result;
	addts_ioctl.ieee_status_code = (t_u8)cfg->param.addts.status_code;
	addts_ioctl.ie_data_len = cfg->param.addts.ie_data_len;

	moal_memcpy_ext(priv->phandle, addts_ioctl.ie_data,
			cfg->param.addts.ie_data, cfg->param.addts.ie_data_len,
			MLAN_WMM_TSPEC_SIZE + MLAN_WMM_ADDTS_EXTRA_IE_BYTES);

	copy_len = (sizeof(addts_ioctl) - sizeof(addts_ioctl.ie_data) +
		    cfg->param.addts.ie_data_len);

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&addts_ioctl, copy_len,
			respbuflen);
	ret = copy_len;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Private IOCTL entry to send a DELTS TSPEC
 *
 *  Receive a DELTS command from the application.  The command structure
 *    contains a TSPEC and reason code along with space for a command result
 *    to be returned.  The information is packaged is sent to the wlan_cmd.c
 *    firmware command prep and send routines for execution in the firmware.
 *
 *  The reason code is not used for WMM implementations but is indicated in
 *    the 802.11e specification.
 *
 *  The return value to the application layer indicates a driver execution
 *    success or failure.  A successful return could still indicate a firmware
 *    failure via the cmd_result field copied back to the application.
 *
 *  @param priv    Pointer to the mlan_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_priv_wmm_delts_req_ioctl(moal_private *priv, t_u8 *respbuf,
					 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *cfg = NULL;
	wlan_ioctl_wmm_delts_req_t delts_ioctl;
	int ret = 0, header_len = 0, copy_len = 0;
	t_u8 *data_ptr;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DELTS);
	data_ptr = respbuf + header_len;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_WMM_CFG;
	cfg = (mlan_ds_wmm_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_WMM_CFG_DELTS;

	memset(&delts_ioctl, 0x00, sizeof(delts_ioctl));

	if (strlen(respbuf) > header_len) {
		copy_len = MIN(strlen(data_ptr), sizeof(delts_ioctl));
		moal_memcpy_ext(priv->phandle, (t_u8 *)&delts_ioctl, data_ptr,
				copy_len, sizeof(delts_ioctl));

		cfg->param.delts.status_code =
			(t_u32)delts_ioctl.ieee_reason_code;
		cfg->param.delts.ie_data_len = (t_u8)delts_ioctl.ie_data_len;

		moal_memcpy_ext(priv->phandle, cfg->param.delts.ie_data,
				delts_ioctl.ie_data,
				cfg->param.delts.ie_data_len,
				MLAN_WMM_TSPEC_SIZE);

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}

		/* Return the firmware command result back to the application
		 * layer */
		delts_ioctl.cmd_result = cfg->param.delts.result;
		copy_len = sizeof(delts_ioctl);
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&delts_ioctl,
				copy_len, respbuflen);
		ret = copy_len;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Private IOCTL entry to get/set a specified AC Queue's parameters
 *
 *  Receive a AC Queue configuration command which is used to get, set, or
 *    default the parameters associated with a specific WMM AC Queue.
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         0 --success, otherwise fail
 */
static int woal_priv_qconfig(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *pwmm = NULL;
	mlan_ds_wmm_queue_config *pqcfg = NULL;
	wlan_ioctl_wmm_queue_config_t qcfg_ioctl;
	t_u8 *data_ptr;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_WMM_CFG;
	pwmm = (mlan_ds_wmm_cfg *)req->pbuf;
	pwmm->sub_command = MLAN_OID_WMM_CFG_QUEUE_CONFIG;

	memset(&qcfg_ioctl, 0x00, sizeof(qcfg_ioctl));
	pqcfg = (mlan_ds_wmm_queue_config *)&pwmm->param.q_cfg;
	data_ptr = respbuf + (strlen(CMD_NXP) + strlen(PRIV_CMD_QCONFIG));

	moal_memcpy_ext(priv->phandle, (t_u8 *)&qcfg_ioctl, data_ptr,
			sizeof(qcfg_ioctl), sizeof(qcfg_ioctl));
	pqcfg->action = qcfg_ioctl.action;
	pqcfg->access_category = qcfg_ioctl.access_category;
	pqcfg->msdu_lifetime_expiry = qcfg_ioctl.msdu_lifetime_expiry;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	memset(&qcfg_ioctl, 0x00, sizeof(qcfg_ioctl));
	qcfg_ioctl.action = pqcfg->action;
	qcfg_ioctl.access_category = pqcfg->access_category;
	qcfg_ioctl.msdu_lifetime_expiry = pqcfg->msdu_lifetime_expiry;
	moal_memcpy_ext(priv->phandle, data_ptr, (t_u8 *)&qcfg_ioctl,
			sizeof(qcfg_ioctl),
			respbuflen -
				(strlen(CMD_NXP) + strlen(PRIV_CMD_QCONFIG)));
	ret = strlen(CMD_NXP) + strlen(PRIV_CMD_QCONFIG) + sizeof(qcfg_ioctl);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Private IOCTL entry to get the status of the WMM queues
 *
 *  Return the following information for each WMM AC:
 *        - WMM IE Acm Required
 *        - Firmware Flow Required
 *        - Firmware Flow Established
 *        - Firmware Queue Enabled
 *        - Firmware Delivery Enabled
 *        - Firmware Trigger Enabled
 *
 *  @param priv    Pointer to the moal_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_priv_wmm_queue_status_ioctl(moal_private *priv, t_u8 *respbuf,
					    t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *pwmm = NULL;
	wlan_ioctl_wmm_queue_status_t qstatus_ioctl;
	int ret = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_QSTATUS);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_WMM_CFG;
	pwmm = (mlan_ds_wmm_cfg *)req->pbuf;
	pwmm->sub_command = MLAN_OID_WMM_CFG_QUEUE_STATUS;

	if (strlen(respbuf) == header_len) {
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}

		memset(&qstatus_ioctl, 0x00, sizeof(qstatus_ioctl));
		moal_memcpy_ext(priv->phandle, (void *)&qstatus_ioctl,
				(void *)&pwmm->param.q_status,
				sizeof(qstatus_ioctl), sizeof(qstatus_ioctl));
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&qstatus_ioctl,
				sizeof(qstatus_ioctl), respbuflen);
		ret = sizeof(qstatus_ioctl);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Private IOCTL entry to get the status of the WMM Traffic Streams
 *
 *  @param priv    Pointer to the moal_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_priv_wmm_ts_status_ioctl(moal_private *priv, t_u8 *respbuf,
					 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_wmm_cfg *pwmm = NULL;
	wlan_ioctl_wmm_ts_status_t ts_status_ioctl;
	int ret = 0, header_len = 0;
	t_u8 *data_ptr;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TS_STATUS);
	data_ptr = respbuf + header_len;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_WMM_CFG;
	pwmm = (mlan_ds_wmm_cfg *)req->pbuf;
	pwmm->sub_command = MLAN_OID_WMM_CFG_TS_STATUS;

	memset(&ts_status_ioctl, 0x00, sizeof(ts_status_ioctl));

	moal_memcpy_ext(priv->phandle, (t_u8 *)&ts_status_ioctl, data_ptr,
			sizeof(ts_status_ioctl), sizeof(ts_status_ioctl));

	memset(&pwmm->param.ts_status, 0x00, sizeof(ts_status_ioctl));
	pwmm->param.ts_status.tid = ts_status_ioctl.tid;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	memset(&ts_status_ioctl, 0x00, sizeof(ts_status_ioctl));
	moal_memcpy_ext(priv->phandle, (void *)&ts_status_ioctl,
			(void *)&pwmm->param.ts_status, sizeof(ts_status_ioctl),
			sizeof(ts_status_ioctl));
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&ts_status_ioctl,
			sizeof(ts_status_ioctl), respbuflen);
	ret = sizeof(ts_status_ioctl);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get MAC control
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_macctrl(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MAC_CTRL);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_MAC_CONTROL;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (user_data_len == 0)
		req->action = MLAN_ACT_GET;
	else {
		cfg->param.mac_ctrl = (t_u32)data;
		req->action = MLAN_ACT_SET;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&cfg->param.mac_ctrl,
			sizeof(data), respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get connection status
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_getwap(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int ret = 0;
#ifdef STA_SUPPORT
	mlan_bss_info bss_info;
#endif

	ENTER();

#ifdef STA_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA) {
		memset(&bss_info, 0, sizeof(bss_info));

		woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);

		if (bss_info.media_connected == MTRUE) {
			moal_memcpy_ext(priv->phandle, respbuf,
					(t_u8 *)&bss_info.bssid,
					MLAN_MAC_ADDR_LENGTH, respbuflen);
		} else {
			memset(respbuf, 0, MLAN_MAC_ADDR_LENGTH);
		}
	}
#endif
#ifdef UAP_SUPPORT
	if (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_UAP) {
		if (priv->bss_started) {
			moal_memcpy_ext(priv->phandle, respbuf,
					priv->current_addr,
					MLAN_MAC_ADDR_LENGTH, respbuflen);
		} else {
			memset(respbuf, 0, MLAN_MAC_ADDR_LENGTH);
		}
	}
#endif
	ret = MLAN_MAC_ADDR_LENGTH;
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Region Code
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_region_code(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_REGION_CODE);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_REGION;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (user_data_len == 0)
		req->action = MLAN_ACT_GET;
	else {
		cfg->param.region_code = (t_u32)data;
		req->action = MLAN_ACT_SET;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&cfg->param.region_code,
			sizeof(data), respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef RX_PACKET_COALESCE
/**
 *  @brief Set/Get RX packet coalesceing setting
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_rx_pkt_coalesce_cfg(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	t_u32 data[2];
	int user_data_len = 0, header_len = 0;
	mlan_ds_misc_cfg *cfg = NULL;
	t_u8 *data_ptr;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	data_ptr = respbuf + strlen(CMD_NXP) + strlen(PRIV_CMD_RX_COAL_CFG);
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RX_COAL_CFG);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if ((user_data_len != 0) && (user_data_len != 2)) {
		PRINTM(MERROR, "Invalid arguments\n");
		ret = -EINVAL;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_RX_PACKET_COALESCE;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (user_data_len == 0) {
		req->action = MLAN_ACT_GET;
	} else {
		req->action = MLAN_ACT_SET;
		cfg->param.rx_coalesce.packet_threshold = data[0];
		cfg->param.rx_coalesce.delay = data[1];
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(
		priv->phandle, respbuf,
		(mlan_ds_misc_rx_packet_coalesce *)&cfg->param.rx_coalesce,
		req->buf_len, respbuflen);
	ret = req->buf_len;

done:
	LEAVE();
	return ret;
}
#endif
/**
 *  @brief Set/Get FW side mac address
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_fwmacaddr(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	t_u8 data[ETH_ALEN];
	int ret = 0;
	int header_len = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_bss *bss = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_FWMACADDR);

	/* Allocate an IOCTL request buffer */
	req = (mlan_ioctl_req *)woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	/* Fill request buffer */
	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_BSS_MAC_ADDR;
	req->req_id = MLAN_IOCTL_BSS;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		req->action = MLAN_ACT_SET;
		memset(data, 0, sizeof(data));
		woal_mac2u8(data, respbuf + header_len);
		moal_memcpy_ext(priv->phandle, bss->param.mac_addr, data,
				ETH_ALEN, sizeof(mlan_802_11_mac_addr));
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, bss->param.mac_addr,
			sizeof(data), respbuflen);
	ret = sizeof(data);
	HEXDUMP("FW MAC Addr:", respbuf, ETH_ALEN);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
/**
 *  @brief Set offchannel
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_offchannel(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int data[4];
	int ret = 0;
	t_u8 status = 1;
	t_u8 chan_type = CHAN_NO_HT;
	int user_data_len = 0, header_len = 0;

	ENTER();

	memset(data, 0, sizeof(data));

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_OFFCHANNEL);

	if (header_len == strlen(respbuf)) {
		/* Query current remain on channel status */
		if (priv->phandle->remain_on_channel)
			ret = sprintf(respbuf,
				      "There is pending remain on channel from bss %d\n",
				      priv->phandle->remain_bss_index) +
			      1;
		else
			ret = sprintf(respbuf,
				      "There is no pending remain on channel\n") +
			      1;
		goto done;
	} else
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}

	if (user_data_len >= 1) {
		if ((data[0] != 0) && (data[0] != 1)) {
			PRINTM(MERROR, "action (%d) must be either 0 or 1\n",
			       data[0]);
			ret = -EINVAL;
			goto done;
		}
	}
	if (user_data_len == 2) {
		if (data[0] == 1) {
			PRINTM(MERROR,
			       "channel and duration must both the mentioned\n");
			ret = -EINVAL;
			goto done;
		} else {
			PRINTM(MWARN,
			       "extra arguments are ignored since action is 'cancel'\n");
		}
	}
	if (user_data_len >= 3) {
		if (data[0] == 1) {
			if (data[1] < 0) {
				PRINTM(MERROR, "channel cannot be negative\n");
				ret = -EINVAL;
				goto done;
			}
			if (data[2] < 0) {
				PRINTM(MERROR, "duration cannot be negative\n");
				ret = -EINVAL;
				goto done;
			}
			if (user_data_len == 4) {
				if (data[3] &&
				    (data[3] != CHANNEL_BW_40MHZ_ABOVE) &&
				    (data[3] != CHANNEL_BW_40MHZ_BELOW) &&
				    (data[3] != CHANNEL_BW_80MHZ)) {
					PRINTM(MERROR, "invalid bandwidth");
					ret = -EINVAL;
					goto done;
				}
				switch (data[3]) {
				case CHANNEL_BW_40MHZ_ABOVE:
					chan_type = CHAN_HT40PLUS;
					break;
				case CHANNEL_BW_40MHZ_BELOW:
					chan_type = CHAN_HT40MINUS;
					break;
				case CHANNEL_BW_80MHZ:
					chan_type = CHAN_VHT80;
					break;
				default:
					break;
				}
			}
		}
	}

	if (data[0] == 0) {
		if (!priv->phandle->remain_on_channel) {
			ret = sprintf(respbuf,
				      "There is no pending remain on channel to be canceled\n") +
			      1;
			goto done;
		}
		if (woal_cfg80211_remain_on_channel_cfg(priv, MOAL_IOCTL_WAIT,
							MTRUE, &status, NULL, 0,
							0)) {
			PRINTM(MERROR, "remain_on_channel: Failed to cancel\n");
			ret = -EFAULT;
			goto done;
		}
		if (status == MLAN_STATUS_SUCCESS)
			priv->phandle->remain_on_channel = MFALSE;
	} else if (data[0] == 1) {
		if (woal_cfg80211_remain_on_channel_cfg(
			    priv, MOAL_IOCTL_WAIT, MFALSE, &status,
			    ieee80211_get_channel(
				    priv->wdev->wiphy,
				    ieee80211_channel_to_frequency(
					    data[1]
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
					    ,
					    (data[1] <= 14 ?
						     IEEE80211_BAND_2GHZ :
						     IEEE80211_BAND_5GHZ)
#endif
						    )),
			    chan_type, (t_u32)data[2])) {
			PRINTM(MERROR, "remain_on_channel: Failed to start\n");
			ret = -EFAULT;
			goto done;
		}
		if (status == MLAN_STATUS_SUCCESS) {
			priv->phandle->remain_on_channel = MTRUE;
			priv->phandle->remain_bss_index = priv->bss_index;
		}
	}

	if (status != MLAN_STATUS_SUCCESS)
		ret = -EFAULT;
	else
		ret = sprintf(respbuf, "OK\n") + 1;

done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 *  @brief Set/Get dscp map
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
int woal_priv_set_get_dscp_map(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int ret = MLAN_STATUS_SUCCESS;
	t_u8 *pos = NULL;
	int copy_size = 0, header_len = 0;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DSCP_MAP);
	if (strlen(respbuf) != header_len) {
		/* SET operation */
		pos = respbuf + header_len;
		moal_memcpy_ext(priv->phandle, priv->dscp_map, pos,
				sizeof(priv->dscp_map), sizeof(priv->dscp_map));
	}

	copy_size = MIN(sizeof(priv->dscp_map), respbuflen);
	moal_memcpy_ext(priv->phandle, respbuf, priv->dscp_map, copy_size,
			respbuflen);
	ret = copy_size;

	LEAVE();
	return ret;
}

/**
 *  @brief Get extended driver version
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_get_driver_verext(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_get_info *info = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int copy_size = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_VER_EXT;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_VEREXT);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	info->param.ver_ext.version_str_sel = data;
	if (((t_s32)(info->param.ver_ext.version_str_sel)) < 0) {
		PRINTM(MERROR, "Invalid arguments!\n");
		ret = -EINVAL;
		goto done;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	/*
	 * Set the amount to copy back to the application as the minimum of the
	 *   available assoc resp data or the buffer provided by the application
	 */
	copy_size = MIN(strlen(info->param.ver_ext.version_str), respbuflen);
	moal_memcpy_ext(priv->phandle, respbuf, info->param.ver_ext.version_str,
			copy_size, respbuflen);
	ret = copy_size;
	PRINTM(MINFO, "MOAL EXTENDED VERSION: %s\n",
	       info->param.ver_ext.version_str);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef USB
#ifdef CONFIG_USB_SUSPEND
/**
 *  @brief This function makes USB device to suspend.
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_enter_usb_suspend(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen)
{
	int ret = 0;

	ENTER();
	ret = woal_enter_usb_suspend(priv->phandle);
	moal_memcpy_ext(priv->phandle, respbuf, &ret, sizeof(int), respbuflen);
	ret = sizeof(int);

	LEAVE();
	return ret;
}

/**
 *  @brief This function makes USB device to resume.
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_exit_usb_suspend(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	int ret = 0;

	ENTER();
	ret = woal_exit_usb_suspend(priv->phandle);
	moal_memcpy_ext(priv->phandle, respbuf, &ret, sizeof(int), respbuflen);
	ret = sizeof(int);

	LEAVE();
	return ret;
}
#endif /* CONFIG_USB_SUSPEND */
#endif

#if defined(STA_SUPPORT) && defined(STA_WEXT)
/**
 *  @brief SET/Get radio
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_radio_ctrl(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int ret = 0, option = 0;
	int user_data_len = 0, header_len = 0;
	mlan_bss_info bss_info;

	ENTER();

	memset(&bss_info, 0, sizeof(bss_info));

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RADIO_CTRL);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &option, 1,
				&user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	if (user_data_len == 1) {
		/* Set radio */
		if (option < 0 || option > 1) {
			PRINTM(MERROR, "Invalid arguments!\n");
			ret = -EINVAL;
			goto done;
		}
		if (MLAN_STATUS_SUCCESS != woal_set_radio(priv, (t_u8)option))
			ret = -EFAULT;
		goto done;
	} else {
		/* Get radio status */
		woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info);
		moal_memcpy_ext(priv->phandle, respbuf, &bss_info.radio_on,
				sizeof(bss_info.radio_on), respbuflen);
		ret = sizeof(bss_info.radio_on);
	}
done:
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Implement WMM enable command
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_wmm_cfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_wmm_cfg *wmm = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_wmm_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	wmm = (mlan_ds_wmm_cfg *)req->pbuf;
	wmm->sub_command = MLAN_OID_WMM_CFG_ENABLE;
	req->req_id = MLAN_IOCTL_WMM_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_WMM_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			/* Set wmm */
			if ((data < CMD_DISABLED) || (data > CMD_ENABLED)) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			if (data == CMD_DISABLED)
				wmm->param.wmm_enable = MFALSE;
			else
				wmm->param.wmm_enable = MTRUE;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, &wmm->param.wmm_enable,
			sizeof(wmm->param.wmm_enable), respbuflen);
	ret = sizeof(wmm->param.wmm_enable);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Implement Mininum BA Threshold cfg command
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_min_ba_threshold_cfg(moal_private *priv, t_u8 *respbuf,
					  t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_MIN_BA_THRESHOLD;
	req->req_id = MLAN_IOCTL_11N_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MIN_BA_THRESH_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			/* Set minimum BA Threshold */
			if ((data < 0) || (data > 16)) {
				PRINTM(MERROR,
				       "Error: Valid minimum BA threshold range (0-16)!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			cfg_11n->param.min_ba_threshold = data;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf,
			&cfg_11n->param.min_ba_threshold,
			sizeof(cfg_11n->param.min_ba_threshold), respbuflen);
	ret = sizeof(cfg_11n->param.min_ba_threshold);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(STA_SUPPORT)
/**
 *  @brief Implement 802.11D enable command
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_11d_cfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_11d_cfg *pcfg_11d = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11d_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	pcfg_11d = (mlan_ds_11d_cfg *)req->pbuf;
	pcfg_11d->sub_command = MLAN_OID_11D_CFG_ENABLE;
	req->req_id = MLAN_IOCTL_11D_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_11D_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			if ((data < CMD_DISABLED) || (data > CMD_ENABLED)) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			if (data == CMD_DISABLED)
				pcfg_11d->param.enable_11d = MFALSE;
			else
				pcfg_11d->param.enable_11d = MTRUE;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, &pcfg_11d->param.enable_11d,
			sizeof(pcfg_11d->param.enable_11d), respbuflen);
	ret = sizeof(pcfg_11d->param.enable_11d);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Implement 802.11D clear chan table command
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise fail
 */
static int woal_priv_11d_clr_chan_tbl(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	mlan_ds_11d_cfg *pcfg_11d = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11d_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_11D_CLR_TBL);

	if (strlen(respbuf) != header_len) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	pcfg_11d = (mlan_ds_11d_cfg *)req->pbuf;
	pcfg_11d->sub_command = MLAN_OID_11D_CLR_CHAN_TABLE;
	req->req_id = MLAN_IOCTL_11D_CFG;
	req->action = MLAN_ACT_SET;

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
#endif

#ifndef OPCHAN
/**
 *  @brief Set/Get WWS mode
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_wws_cfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_misc_cfg *wws = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	wws = (mlan_ds_misc_cfg *)req->pbuf;
	wws->sub_command = MLAN_OID_MISC_WWS;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_WWS_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			if ((data < CMD_DISABLED) || (data > CMD_ENABLED)) {
				PRINTM(MERROR,
				       "Invalid arguments, WWS config not changed!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			wws->param.wws_cfg = (t_u16)data;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, &wws->param.wws_cfg,
			sizeof(wws->param.wws_cfg), respbuflen);
	ret = sizeof(wws->param.wws_cfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

#if defined(REASSOCIATION)
/**
 *  @brief Set/Get reassociation settings
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_set_get_reassoc(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	moal_handle *handle = priv->phandle;
	int data = 0;
	int ret = 0;
	int user_data_len = 0, header_len = 0;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_REASSOCTRL);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		data = (int)(priv->reassoc_on);
		moal_memcpy_ext(handle, respbuf, &data, sizeof(data),
				respbuflen);
		ret = sizeof(data);
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			if (data == 0) {
				handle->reassoc_on &= ~MBIT(priv->bss_index);
				priv->reassoc_on = MFALSE;
				priv->reassoc_required = MFALSE;
				if (!handle->reassoc_on &&
				    handle->is_reassoc_timer_set == MTRUE) {
					woal_cancel_timer(
						&handle->reassoc_timer);
					handle->is_reassoc_timer_set = MFALSE;
				}
			} else if (data == 1) {
				handle->reassoc_on |= MBIT(priv->bss_index);
				priv->reassoc_on = MTRUE;
			} else {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
			}
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
		}
	}

	LEAVE();
	return ret;
}
#endif /* REASSOCIATION */

/**
 *  @brief Get Transmit buffer size
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_txbuf_cfg(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int buf_size = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_MAX_TX_BUF_SIZE;
	req->req_id = MLAN_IOCTL_11N_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TXBUF_CFG);

	if (strlen(respbuf) != header_len) {
		PRINTM(MERROR,
		       "Don't support set Tx buffer size after driver loaded!\n");
		ret = -EINVAL;
		goto done;
	} else {
		/* Get Tx buffer size from MLAN */
		req->action = MLAN_ACT_GET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	buf_size = cfg_11n->param.tx_buf_size;
	moal_memcpy_ext(priv->phandle, respbuf, &buf_size, sizeof(buf_size),
			respbuflen);
	ret = sizeof(buf_size);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef STA_SUPPORT
/**
 *  @brief Set/Get auth type
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_auth_type(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int auth_type = 0;
	t_u32 auth_mode;
	int ret = 0;
	int user_data_len = 0, header_len = 0;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_AUTH_TYPE);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_auth_mode(priv, MOAL_IOCTL_WAIT, &auth_mode)) {
			ret = -EFAULT;
			goto done;
		}
		user_data_len = 0;
		auth_type = auth_mode;
		moal_memcpy_ext(priv->phandle, respbuf, &auth_type,
				sizeof(auth_type), respbuflen);
		ret = sizeof(auth_type);
		goto done;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &auth_type, 1,
				&user_data_len);
		if (user_data_len == 1) {
			PRINTM(MINFO, "SET: auth_type %d\n", auth_type);
			if (((auth_type < MLAN_AUTH_MODE_OPEN) ||
			     (auth_type > MLAN_AUTH_MODE_SAE)) &&
			    (auth_type != MLAN_AUTH_MODE_AUTO)) {
				ret = -EINVAL;
				goto done;
			}
			auth_mode = auth_type;
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_auth_mode(priv, MOAL_IOCTL_WAIT,
					       auth_mode)) {
				ret = -EFAULT;
				goto done;
			}
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
		}
	}

done:
	LEAVE();
	return ret;
}
#endif

/**
 *  @brief Set/get user provisioned local power constraint
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_11h_local_pwr_constraint(moal_private *priv, t_u8 *respbuf,
					      t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;
	ds_11hcfg->sub_command = MLAN_OID_11H_LOCAL_POWER_CONSTRAINT;
	req->req_id = MLAN_IOCTL_11H_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_POWER_CONS);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			req->action = MLAN_ACT_SET;
			ds_11hcfg->param.usr_local_power_constraint =
				(t_s8)data;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (req->action == MLAN_ACT_GET) {
		data = (int)ds_11hcfg->param.usr_local_power_constraint;
		moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
				respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/get HT stream configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_ht_stream_cfg(moal_private *priv, t_u8 *respbuf,
				   t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_11n_cfg *cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_11n_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_11N_CFG_STREAM_CFG;
	req->req_id = MLAN_IOCTL_11N_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_HT_STREAM_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			if (data != HT_STREAM_MODE_1X1 &&
			    data != HT_STREAM_MODE_2X2) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			cfg->param.stream_cfg = data;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data = ((mlan_ds_11n_cfg *)req->pbuf)->param.stream_cfg;
	moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set mimo switch configurations
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_mimo_switch(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int data[2] = {0};
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_MIMO_SWITCH;
	req->req_id = MLAN_IOCTL_RADIO_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MIMO_SWITCH);

	if (strlen(respbuf) > header_len) {
		/* SET operation */
		req->action = MLAN_ACT_SET;
		parse_arguments(respbuf + header_len, data, 2, &user_data_len);
		if (user_data_len == 2) {
			radio->param.mimo_switch_cfg.txpath_antmode = data[0];
			radio->param.mimo_switch_cfg.rxpath_antmode = data[1];
		} else {
			PRINTM(MERROR, "Invalid arguments!\n");
			ret = -EINVAL;
			goto done;
		}
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		PRINTM(MERROR, "Invalid arguments!\n");
		ret = -EINVAL;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get thermal reading
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_thermal(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0, header_len = 0, data = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_THERMAL;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_THERMAL);

	if (strlen(respbuf) != header_len) {
		PRINTM(MERROR, "Set is not supported for this command\n");
		ret = -EINVAL;
		goto done;
	}
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data = (int)cfg->param.thermal;
	moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get beacon interval
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_beacon_interval(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	int data = 0;
	mlan_ds_bss *bss = NULL;
	mlan_ioctl_req *req = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_bss));
	if (req == NULL) {
		ret = ENOMEM;
		goto done;
	}

	bss = (mlan_ds_bss *)req->pbuf;
	bss->sub_command = MLAN_OID_IBSS_BCN_INTERVAL;
	req->req_id = MLAN_IOCTL_BSS;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_BCN_INTERVAL);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		req->action = MLAN_ACT_GET;
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);
		if (user_data_len == 1) {
			if ((data < MLAN_MIN_BEACON_INTERVAL) ||
			    (data > MLAN_MAX_BEACON_INTERVAL)) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
			req->action = MLAN_ACT_SET;
			bss->param.bcn_interval = data;
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data = ((mlan_ds_bss *)req->pbuf)->param.bcn_interval;
	moal_memcpy_ext(priv->phandle, respbuf, &data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef STA_SUPPORT
/**
 *  @brief Get signal
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_signal(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
/** Input data size */
#define IN_DATA_SIZE 2
/** Output data size */
#define OUT_DATA_SIZE 12
	int ret = 0;
	int in_data[IN_DATA_SIZE];
	int out_data[OUT_DATA_SIZE];
	mlan_ds_get_signal signal;
	int data_length = 0;
	int buflen = 0;
	int user_data_len = 0, header_len = 0;

	ENTER();

	memset(in_data, 0, sizeof(in_data));
	memset(out_data, 0, sizeof(out_data));

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_SIGNAL);

	if (strlen(respbuf) != header_len)
		parse_arguments(respbuf + header_len, in_data, IN_DATA_SIZE,
				&user_data_len);
	buflen = MIN(user_data_len, IN_DATA_SIZE);

	if (priv->media_connected == MFALSE) {
		PRINTM(MERROR, "Can not get RSSI in disconnected state\n");
		ret = -ENOTSUPP;
		goto done;
	}

	if (user_data_len) {
		if (user_data_len > IN_DATA_SIZE) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}

	switch (user_data_len) {
	case 0: /* No checking, get everything */
		break;
	case 2: /* Check subtype range */
		if (in_data[1] < 1 || in_data[1] > 4) {
			ret = -EINVAL;
			goto done;
		}
		/* Fall through */
	case 1: /* Check type range */
		if (in_data[0] < 1 || in_data[0] > 3) {
			ret = -EINVAL;
			goto done;
		}
		break;
	default:
		ret = -EINVAL;
		goto done;
	}

	memset(&signal, 0, sizeof(mlan_ds_get_signal));
	if (MLAN_STATUS_SUCCESS !=
	    woal_get_signal_info(priv, MOAL_IOCTL_WAIT, &signal)) {
		ret = -EFAULT;
		goto done;
	}
	PRINTM(MINFO, "RSSI Beacon Last   : %d\n", (int)signal.bcn_rssi_last);
	PRINTM(MINFO, "RSSI Beacon Average: %d\n", (int)signal.bcn_rssi_avg);
	PRINTM(MINFO, "RSSI Data Last     : %d\n", (int)signal.data_rssi_last);
	PRINTM(MINFO, "RSSI Data Average  : %d\n", (int)signal.data_rssi_avg);
	PRINTM(MINFO, "SNR Beacon Last    : %d\n", (int)signal.bcn_snr_last);
	PRINTM(MINFO, "SNR Beacon Average : %d\n", (int)signal.bcn_snr_avg);
	PRINTM(MINFO, "SNR Data Last      : %d\n", (int)signal.data_snr_last);
	PRINTM(MINFO, "SNR Data Average   : %d\n", (int)signal.data_snr_avg);
	PRINTM(MINFO, "NF Beacon Last     : %d\n", (int)signal.bcn_nf_last);
	PRINTM(MINFO, "NF Beacon Average  : %d\n", (int)signal.bcn_nf_avg);
	PRINTM(MINFO, "NF Data Last       : %d\n", (int)signal.data_nf_last);
	PRINTM(MINFO, "NF Data Average    : %d\n", (int)signal.data_nf_avg);

	/* Check type */
	switch (in_data[0]) {
	case 0: /* Send everything */
		out_data[data_length++] = signal.bcn_rssi_last;
		out_data[data_length++] = signal.bcn_rssi_avg;
		out_data[data_length++] = signal.data_rssi_last;
		out_data[data_length++] = signal.data_rssi_avg;
		out_data[data_length++] = signal.bcn_snr_last;
		out_data[data_length++] = signal.bcn_snr_avg;
		out_data[data_length++] = signal.data_snr_last;
		out_data[data_length++] = signal.data_snr_avg;
		out_data[data_length++] = signal.bcn_nf_last;
		out_data[data_length++] = signal.bcn_nf_avg;
		out_data[data_length++] = signal.data_nf_last;
		out_data[data_length++] = signal.data_nf_avg;
		break;
	case 1: /* RSSI */
		/* Check subtype */
		switch (in_data[1]) {
		case 0: /* Everything */
			out_data[data_length++] = signal.bcn_rssi_last;
			out_data[data_length++] = signal.bcn_rssi_avg;
			out_data[data_length++] = signal.data_rssi_last;
			out_data[data_length++] = signal.data_rssi_avg;
			break;
		case 1: /* bcn last */
			out_data[data_length++] = signal.bcn_rssi_last;
			break;
		case 2: /* bcn avg */
			out_data[data_length++] = signal.bcn_rssi_avg;
			break;
		case 3: /* data last */
			out_data[data_length++] = signal.data_rssi_last;
			break;
		case 4: /* data avg */
			out_data[data_length++] = signal.data_rssi_avg;
			break;
		default:
			break;
		}
		break;
	case 2: /* SNR */
		/* Check subtype */
		switch (in_data[1]) {
		case 0: /* Everything */
			out_data[data_length++] = signal.bcn_snr_last;
			out_data[data_length++] = signal.bcn_snr_avg;
			out_data[data_length++] = signal.data_snr_last;
			out_data[data_length++] = signal.data_snr_avg;
			break;
		case 1: /* bcn last */
			out_data[data_length++] = signal.bcn_snr_last;
			break;
		case 2: /* bcn avg */
			out_data[data_length++] = signal.bcn_snr_avg;
			break;
		case 3: /* data last */
			out_data[data_length++] = signal.data_snr_last;
			break;
		case 4: /* data avg */
			out_data[data_length++] = signal.data_snr_avg;
			break;
		default:
			break;
		}
		break;
	case 3: /* NF */
		/* Check subtype */
		switch (in_data[1]) {
		case 0: /* Everything */
			out_data[data_length++] = signal.bcn_nf_last;
			out_data[data_length++] = signal.bcn_nf_avg;
			out_data[data_length++] = signal.data_nf_last;
			out_data[data_length++] = signal.data_nf_avg;
			break;
		case 1: /* bcn last */
			out_data[data_length++] = signal.bcn_nf_last;
			break;
		case 2: /* bcn avg */
			out_data[data_length++] = signal.bcn_nf_avg;
			break;
		case 3: /* data last */
			out_data[data_length++] = signal.data_nf_last;
			break;
		case 4: /* data avg */
			out_data[data_length++] = signal.data_nf_avg;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	moal_memcpy_ext(priv->phandle, respbuf, out_data,
			(data_length * sizeof(int)), respbuflen);
	ret = data_length * sizeof(int);

done:
	LEAVE();
	return ret;
}

static int woal_signal_ext_enable(moal_private *priv, t_u8 enable)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_snmp_mib *snmp = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	snmp = (mlan_ds_snmp_mib *)req->pbuf;
	snmp->sub_command = MLAN_OID_SNMP_MIB_SIGNALEXT_ENABLE;
	req->req_id = MLAN_IOCTL_SNMP_MIB;
	req->action = MLAN_ACT_SET;
	snmp->param.signalext_enable = enable;

	/* Send IOCTL request to MLAN */
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
 *  @brief Get signal
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_signal_ext(moal_private *priv, t_u8 *respbuf,
				    t_u32 respbuflen)
{
#define PATH_SIZE 13
	int ret = 0;
	int data = 0, path = 0, data_len = 0;
	int user_data_len = 0, header_len = 0;
	int out_data[PATH_SIZE * MAX_PATH_NUM] = {0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_ds_get_signal signal_get[MAX_PATH_NUM];
	mlan_status status = MLAN_STATUS_SUCCESS;
	int path_num;
	t_u8 enable = 1;

	ENTER();

	if (priv->media_connected == MFALSE) {
		PRINTM(MERROR, "Can not get RSSI in disconnected state\n");
		ret = -ENOTSUPP;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_SIGNAL_EXT);

	if (strlen(respbuf) != header_len) {
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	if (data < PATH_ALL || data > PATH_AB) {
		PRINTM(MERROR, "Wrong arguments\n");
		ret = -EINVAL;
		goto done;
	}

	/** Enable signalext feature in firmware */
	if (MLAN_STATUS_SUCCESS != woal_signal_ext_enable(priv, enable)) {
		ret = -EFAULT;
		goto done;
	}
	woal_sched_timeout(1000);
	enable = 0;

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_SIGNAL_EXT;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;
	info->param.path_id = (t_u16)data;

	/* Send IOCTL request to MLAN */
	if (MLAN_STATUS_SUCCESS !=
	    woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
		woal_signal_ext_enable(priv, enable);
		ret = -EFAULT;
		goto done;
	}
	if (MLAN_STATUS_SUCCESS != woal_signal_ext_enable(priv, enable)) {
		ret = -EFAULT;
		goto done;
	}
	path_num = 1;
	if (data == PATH_ALL) {
		moal_memcpy_ext(priv->phandle, signal_get,
				info->param.signal_ext, sizeof(signal_get),
				sizeof(signal_get));
		path_num = MAX_PATH_NUM;
	} else
		moal_memcpy_ext(priv->phandle, signal_get,
				info->param.signal_ext,
				sizeof(mlan_ds_get_signal), sizeof(signal_get));

	for (path = 0; path < path_num; path++) {
		if (signal_get[path].selector == PATH_AB)
			PRINTM(MINFO, "PATH A+B:\n");
		else if (signal_get[path].selector == PATH_A)
			PRINTM(MINFO, "PATH A:\n");
		else if (signal_get[path].selector == PATH_B)
			PRINTM(MINFO, "PATH B:\n");
		PRINTM(MINFO, "RSSI Beacon Last   : %d\n",
		       (int)signal_get[path].bcn_rssi_last);
		PRINTM(MINFO, "RSSI Beacon Average: %d\n",
		       (int)signal_get[path].bcn_rssi_avg);
		PRINTM(MINFO, "RSSI Data Last     : %d\n",
		       (int)signal_get[path].data_rssi_last);
		PRINTM(MINFO, "RSSI Data Average  : %d\n",
		       (int)signal_get[path].data_rssi_avg);
		PRINTM(MINFO, "SNR Beacon Last    : %d\n",
		       (int)signal_get[path].bcn_snr_last);
		PRINTM(MINFO, "SNR Beacon Average : %d\n",
		       (int)signal_get[path].bcn_snr_avg);
		PRINTM(MINFO, "SNR Data Last      : %d\n",
		       (int)signal_get[path].data_snr_last);
		PRINTM(MINFO, "SNR Data Average   : %d\n",
		       (int)signal_get[path].data_snr_avg);
		PRINTM(MINFO, "NF Beacon Last     : %d\n",
		       (int)signal_get[path].bcn_nf_last);
		PRINTM(MINFO, "NF Beacon Average  : %d\n",
		       (int)signal_get[path].bcn_nf_avg);
		PRINTM(MINFO, "NF Data Last       : %d\n",
		       (int)signal_get[path].data_nf_last);
		PRINTM(MINFO, "NF Data Average    : %d\n",
		       (int)signal_get[path].data_nf_avg);
		out_data[data_len++] = (int)signal_get[path].selector;
		out_data[data_len++] = (int)signal_get[path].bcn_rssi_last;
		out_data[data_len++] = (int)signal_get[path].bcn_rssi_avg;
		out_data[data_len++] = (int)signal_get[path].data_rssi_last;
		out_data[data_len++] = (int)signal_get[path].data_rssi_avg;
		out_data[data_len++] = (int)signal_get[path].bcn_snr_last;
		out_data[data_len++] = (int)signal_get[path].bcn_snr_avg;
		out_data[data_len++] = (int)signal_get[path].data_snr_last;
		out_data[data_len++] = (int)signal_get[path].data_snr_avg;
		out_data[data_len++] = (int)signal_get[path].bcn_nf_last;
		out_data[data_len++] = (int)signal_get[path].bcn_nf_avg;
		out_data[data_len++] = (int)signal_get[path].data_nf_last;
		out_data[data_len++] = (int)signal_get[path].data_nf_avg;
	}
	moal_memcpy_ext(
		priv->phandle, respbuf, out_data,
		(MIN((PATH_SIZE * MAX_PATH_NUM), data_len) * sizeof(int)),
		respbuflen);
	ret = data_len * sizeof(int);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Get signalext v2
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_signal_ext_v2(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen)
{
#define PATH_SIZE 13
	int ret = 0;
	int data = 0, path = 0, data_len = 0;
	int user_data_len = 0, header_len = 0;
	int out_data[PATH_SIZE * MAX_PATH_NUM] = {0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_get_info *info = NULL;
	mlan_ds_get_signal signal_get[MAX_PATH_NUM];
	mlan_status status = MLAN_STATUS_SUCCESS;
	int path_num;

	ENTER();

	if (priv->media_connected == MFALSE) {
		PRINTM(MERROR, "Can not get RSSI in disconnected state\n");
		ret = -ENOTSUPP;
		goto done;
	}

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_get_info));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_SIGNAL_EXT_V2);
	if (strlen(respbuf) != header_len) {
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
	}

	if (user_data_len > 1) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		goto done;
	}
	if (data < PATH_ALL || data > PATH_AB) {
		PRINTM(MERROR, "Wrong arguments\n");
		ret = -EINVAL;
		goto done;
	}

	/* Fill request buffer */
	info = (mlan_ds_get_info *)req->pbuf;
	info->sub_command = MLAN_OID_GET_SIGNAL_EXT;
	req->req_id = MLAN_IOCTL_GET_INFO;
	req->action = MLAN_ACT_GET;
	info->param.path_id = (t_u16)data;

	/* Send IOCTL request to MLAN */
	if (MLAN_STATUS_SUCCESS !=
	    woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT)) {
		PRINTM(MERROR,
		       "Enable signalextcfg: mlanutl mlanX signalextcfg 1"
		       " before issuing this command\n");
		ret = -EFAULT;
		goto done;
	}
	path_num = 1;

	if (data == PATH_ALL) {
		moal_memcpy_ext(priv->phandle, signal_get,
				info->param.signal_ext, sizeof(signal_get),
				sizeof(signal_get));
		path_num = MAX_PATH_NUM;
	} else
		moal_memcpy_ext(priv->phandle, signal_get,
				info->param.signal_ext,
				sizeof(mlan_ds_get_signal), sizeof(signal_get));

	PRINTM(MMSG, "data=%d path_num=%d\n", data, path_num);

	for (path = 0; path < path_num; path++) {
		if (signal_get[path].selector == PATH_AB)
			PRINTM(MINFO, "PATH A+B:\n");
		else if (signal_get[path].selector == PATH_A)
			PRINTM(MINFO, "PATH A:\n");
		else if (signal_get[path].selector == PATH_B)
			PRINTM(MINFO, "PATH B:\n");
		PRINTM(MINFO, "RSSI Beacon Last   : %d\n",
		       (int)signal_get[path].bcn_rssi_last);
		PRINTM(MINFO, "RSSI Beacon Average: %d\n",
		       (int)signal_get[path].bcn_rssi_avg);
		PRINTM(MINFO, "RSSI Data Last     : %d\n",
		       (int)signal_get[path].data_rssi_last);
		PRINTM(MINFO, "RSSI Data Average  : %d\n",
		       (int)signal_get[path].data_rssi_avg);
		PRINTM(MINFO, "SNR Beacon Last    : %d\n",
		       (int)signal_get[path].bcn_snr_last);
		PRINTM(MINFO, "SNR Beacon Average : %d\n",
		       (int)signal_get[path].bcn_snr_avg);
		PRINTM(MINFO, "SNR Data Last      : %d\n",
		       (int)signal_get[path].data_snr_last);
		PRINTM(MINFO, "SNR Data Average   : %d\n",
		       (int)signal_get[path].data_snr_avg);
		PRINTM(MINFO, "NF Beacon Last     : %d\n",
		       (int)signal_get[path].bcn_nf_last);
		PRINTM(MINFO, "NF Beacon Average  : %d\n",
		       (int)signal_get[path].bcn_nf_avg);
		PRINTM(MINFO, "NF Data Last       : %d\n",
		       (int)signal_get[path].data_nf_last);
		PRINTM(MINFO, "NF Data Average    : %d\n",
		       (int)signal_get[path].data_nf_avg);
		out_data[data_len++] = (int)signal_get[path].selector;
		out_data[data_len++] = (int)signal_get[path].bcn_rssi_last;
		out_data[data_len++] = (int)signal_get[path].bcn_rssi_avg;
		out_data[data_len++] = (int)signal_get[path].data_rssi_last;
		out_data[data_len++] = (int)signal_get[path].data_rssi_avg;
		out_data[data_len++] = (int)signal_get[path].bcn_snr_last;
		out_data[data_len++] = (int)signal_get[path].bcn_snr_avg;
		out_data[data_len++] = (int)signal_get[path].data_snr_last;
		out_data[data_len++] = (int)signal_get[path].data_snr_avg;
		out_data[data_len++] = (int)signal_get[path].bcn_nf_last;
		out_data[data_len++] = (int)signal_get[path].bcn_nf_avg;
		out_data[data_len++] = (int)signal_get[path].data_nf_last;
		out_data[data_len++] = (int)signal_get[path].data_nf_avg;
	}
	moal_memcpy_ext(
		priv->phandle, respbuf, out_data,
		(MIN((PATH_SIZE * MAX_PATH_NUM), data_len) * sizeof(int)),
		respbuflen);
	ret = data_len * sizeof(int);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set signalext cfg
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             The result of this processing.
 */
static int woal_priv_signalext_cfg(moal_private *priv, t_u8 *respbuf,
				   t_u32 respbuflen)
{
	int enable = 0;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	ENTER();
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SIGNALEXT_CFG);
	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid arguments!\n");
		ret = -EINVAL;
		goto done;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &enable, 1,
				&user_data_len);
		if (user_data_len == 1) {
			if (enable != 0x0 && enable != 0x1) {
				PRINTM(MERROR, "Invalid arguments!\n");
				ret = -EINVAL;
				goto done;
			}
			ret = woal_signal_ext_enable(priv, enable);
		} else {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
	}
done:
	LEAVE();
	return ret;
}
#endif /* #ifdef STA_SUPPORT */

#if defined(STA_SUPPORT)
/**
 * @brief               Make PMF bit required/optional
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              0 -- success, otherwise fail
 */
int woal_priv_set_get_pmfcfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int data[2] = {0, 0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	mlan_ds_misc_pmfcfg *pmfcfg;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv->phandle->card_info->embedded_supp) {
		PRINTM(MERROR, "Not supported cmd on this card\n");
		ret = -EOPNOTSUPP;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PMFCFG);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	if (user_data_len > 2) {
		PRINTM(MERROR, "Invalid number of arguments\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	pmfcfg = (mlan_ds_misc_pmfcfg *)&cfg->param.pmfcfg;
	cfg->sub_command = MLAN_OID_MISC_PMFCFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (user_data_len == 0)
		req->action = MLAN_ACT_GET;
	else {
		pmfcfg->mfpc = (t_u8)data[0];
		pmfcfg->mfpr = (t_u8)data[1];
		req->action = MLAN_ACT_SET;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&cfg->param.pmfcfg,
			sizeof(mlan_ds_misc_pmfcfg), respbuflen);
	ret = sizeof(mlan_ds_misc_pmfcfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
#endif

/**
 * @brief               Get/Set inactivity timeout extend
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_inactivity_timeout_ext(moal_private *priv, t_u8 *respbuf,
					    t_u32 respbuflen)
{
	int data[4];
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pmcfg = NULL;
	pmlan_ds_inactivity_to inac_to = NULL;
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_INACTIVITYTO);
	memset(data, 0, sizeof(data));
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	if (user_data_len != 0 && user_data_len != 3 && user_data_len != 4) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	pmcfg = (mlan_ds_pm_cfg *)req->pbuf;
	inac_to = &pmcfg->param.inactivity_to;
	pmcfg->sub_command = MLAN_OID_PM_CFG_INACTIVITY_TO;
	req->req_id = MLAN_IOCTL_PM_CFG;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (user_data_len) {
		inac_to->timeout_unit = data[0];
		inac_to->unicast_timeout = data[1];
		inac_to->mcast_timeout = data[2];
		if (user_data_len == 4)
			inac_to->ps_entry_timeout = data[3];
		req->action = MLAN_ACT_SET;

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		data[0] = inac_to->timeout_unit;
		data[1] = inac_to->unicast_timeout;
		data[2] = inac_to->mcast_timeout;
		data[3] = inac_to->ps_entry_timeout;

		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief               Enable/Disable amsdu_aggr_ctrl
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_11n_amsdu_aggr_ctrl(moal_private *priv, t_u8 *respbuf,
					 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	int ret = 0, data[2] = {0};
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_AMSDU_AGGR_CTRL);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_AMSDU_AGGR_CTRL;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, 1, &user_data_len);

		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		cfg_11n->param.amsdu_aggr_ctrl.enable = data[0];
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = cfg_11n->param.amsdu_aggr_ctrl.enable;
	data[1] = cfg_11n->param.amsdu_aggr_ctrl.curr_buf_size;

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get Transmit beamforming capabilities
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_tx_bf_cap_ioctl(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *bf_cfg = NULL;
	int ret = 0, bf_cap = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TX_BF_CAP);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	bf_cfg = (mlan_ds_11n_cfg *)req->pbuf;
	bf_cfg->sub_command = MLAN_OID_11N_CFG_TX_BF_CAP;
	req->req_id = MLAN_IOCTL_11N_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &bf_cap, 1,
				&user_data_len);

		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		bf_cfg->param.tx_bf_cap = bf_cap;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	bf_cap = bf_cfg->param.tx_bf_cap;

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&bf_cap, sizeof(bf_cap),
			respbuflen);
	ret = sizeof(bf_cap);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#ifdef SDIO
/**
 * @brief               Turn on/off the sdio clock
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              Number of bytes written, negative for failure.
 */
static int woal_priv_sdio_clock_ioctl(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	int ret = 0;
	int data = 2;
	int user_data_len = 0, header_len = 0;
	/* Initialize the clock state as on */
	static int clock_state = 1;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SDIO_CLOCK);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&clock_state,
				sizeof(clock_state), respbuflen);
		ret = sizeof(clock_state);
		goto done;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data, 1, &user_data_len);

		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
	}
	switch (data) {
	case CMD_DISABLED:
		PRINTM(MINFO, "SDIO clock is turned off\n");
		ret = woal_sdio_set_bus_clock(priv->phandle, MFALSE);
		clock_state = data;
		break;
	case CMD_ENABLED:
		PRINTM(MINFO, "SDIO clock is turned on\n");
		ret = woal_sdio_set_bus_clock(priv->phandle, MTRUE);
		clock_state = data;
		break;
	default:
		ret = -EINVAL;
		PRINTM(MINFO, "sdioclock: wrong parameter\n");
		break;
	}
done:
	LEAVE();
	return ret;
}
#endif

#ifdef SDIO
/**
 * @brief               Set SDIO Multi-point aggregation control parameters
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_sdio_mpa_ctrl(moal_private *priv, t_u8 *respbuf,
				   t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0, data[6];
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(data, 0, sizeof(data));
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MPA_CTRL);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);

		if (user_data_len > 6) {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_SDIO_MPA_CTRL;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;
	/* Get the values first, then modify these values if
	 * user had modified them */

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "woal_request_ioctl returned %d\n", ret);
		ret = -EFAULT;
		goto done;
	}

	if (user_data_len == 0) {
		data[0] = misc->param.mpa_ctrl.tx_enable;
		data[1] = misc->param.mpa_ctrl.rx_enable;
		data[2] = misc->param.mpa_ctrl.tx_buf_size;
		data[3] = misc->param.mpa_ctrl.rx_buf_size;
		data[4] = misc->param.mpa_ctrl.tx_max_ports;
		data[5] = misc->param.mpa_ctrl.rx_max_ports;

		PRINTM(MINFO, "Get Param: %d %d %d %d %d %d\n", data[0],
		       data[1], data[2], data[3], data[4], data[5]);

		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
		goto done;
	}

	switch (user_data_len) {
	case 6:
		misc->param.mpa_ctrl.rx_max_ports = data[5];
		/* fall through */
	case 5:
		misc->param.mpa_ctrl.tx_max_ports = data[4];
		/* fall through */
	case 4:
		misc->param.mpa_ctrl.rx_buf_size = data[3];
		/* fall through */
	case 3:
		misc->param.mpa_ctrl.tx_buf_size = data[2];
		/* fall through */
	case 2:
		misc->param.mpa_ctrl.rx_enable = data[1];
		/* fall through */
	case 1:
		/* Set cmd */
		req->action = MLAN_ACT_SET;

		PRINTM(MINFO, "Set Param: %d %d %d %d %d %d\n", data[0],
		       data[1], data[2], data[3], data[4], data[5]);

		misc->param.mpa_ctrl.tx_enable = data[0];
		break;
	default:
		PRINTM(MERROR, "Default case error\n");
		ret = -EINVAL;
		goto done;
	}

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
#endif

/**
 * @brief               Configure sleep parameters
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_sleep_params_ioctl(moal_private *priv, t_u8 *respbuf,
					t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_pm_cfg *pm = NULL;
	mlan_ds_sleep_params *psleep_params = NULL;
	int data[6] = {0}, i;
	int user_data_len = 0, header_len = 0;
#ifdef DEBUG_LEVEL1
	char err_str[][36] = {{"sleep clock error in ppm"},
			      {"wakeup offset in usec"},
			      {"clock stabilization time in usec"},
			      {"control periodic calibration(0-2)"},
			      {"control of external sleepClock(0-2)"},
			      {"value of reserved for debug"}};
#endif
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	memset(data, 0, sizeof(data));
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_pm_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	pm = (mlan_ds_pm_cfg *)req->pbuf;
	pm->sub_command = MLAN_OID_PM_CFG_SLEEP_PARAMS;
	req->req_id = MLAN_IOCTL_PM_CFG;
	psleep_params = (pmlan_ds_sleep_params)&pm->param.sleep_params;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SLEEP_PARAMS);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != 6) {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
#define MIN_VAL 0x0000
#define MAX_VAL 0xFFFF
		for (i = 0; i < 6; i++) {
			if ((i == 3) || (i == 4)) {
				/* These two cases are handled below the loop */
				continue;
			}
			if (data[i] < MIN_VAL || data[i] > MAX_VAL) {
				PRINTM(MERROR, "Invalid %s (0-65535)!\n",
				       err_str[i]);
				ret = -EINVAL;
				goto done;
			}
		}
		if (data[3] < 0 || data[3] > 2) {
			PRINTM(MERROR,
			       "Invalid control periodic calibration (0-2)!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[4] < 0 || data[4] > 2) {
			PRINTM(MERROR,
			       "Invalid control of external sleep clock (0-2)!\n");
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		psleep_params->error = data[0];
		psleep_params->offset = data[1];
		psleep_params->stable_time = data[2];
		psleep_params->cal_control = data[3];
		psleep_params->ext_sleep_clk = data[4];
		psleep_params->reserved = data[5];
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = psleep_params->error;
	data[1] = psleep_params->offset;
	data[2] = psleep_params->stable_time;
	data[3] = psleep_params->cal_control;
	data[4] = psleep_params->ext_sleep_clk;
	data[5] = psleep_params->reserved;

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get DFS Testing settings
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_dfs_testing(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	int ret = 0;
	int data[5] = {0};
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DFS_TESTING);
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;
	ds_11hcfg->sub_command = MLAN_OID_11H_DFS_TESTING;
	req->req_id = MLAN_IOCTL_11H_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != 5) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if ((unsigned)data[0] > 1800) {
			PRINTM(MERROR,
			       "The maximum user CAC is 1800 seconds (30 mins).\n");
			ret = -EINVAL;
			goto done;
		}
		if ((unsigned)data[1] > 0xFFFF) {
			PRINTM(MERROR, "The maximum user NOP is 65535 sec.\n");
			ret = -EINVAL;
			goto done;
		}
		if ((unsigned)data[3] > 0xFF) {
			PRINTM(MERROR,
			       "The maximum user fixed channel is 255.\n");
			ret = -EINVAL;
			goto done;
		}
		if ((unsigned)data[4] != 0 && ((unsigned)data[4] != 1)) {
			PRINTM(MERROR, "CAC restart should be 0/1\n");
			ret = -EINVAL;
			goto done;
		}

		ds_11hcfg->param.dfs_testing.usr_cac_period_msec =
			(t_u32)data[0] * 1000;
		ds_11hcfg->param.dfs_testing.usr_nop_period_sec =
			(t_u16)data[1];
		ds_11hcfg->param.dfs_testing.usr_no_chan_change =
			data[2] ? 1 : 0;
		ds_11hcfg->param.dfs_testing.usr_fixed_new_chan = (t_u8)data[3];
		ds_11hcfg->param.dfs_testing.usr_cac_restart = (t_u8)data[4];
		priv->phandle->cac_restart = (t_u8)data[4];
		priv->phandle->cac_period_jiffies = (t_u32)data[0] * HZ / 1000;
		priv->phandle->usr_nop_period_sec = (t_u16)data[1];
		req->action = MLAN_ACT_SET;
#ifdef UAP_SUPPORT
		priv->user_cac_period_msec =
			ds_11hcfg->param.dfs_testing.usr_cac_period_msec;
#endif
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		data[0] =
			ds_11hcfg->param.dfs_testing.usr_cac_period_msec / 1000;
		data[1] = ds_11hcfg->param.dfs_testing.usr_nop_period_sec;
		data[2] = ds_11hcfg->param.dfs_testing.usr_no_chan_change;
		data[3] = ds_11hcfg->param.dfs_testing.usr_fixed_new_chan;
		data[4] = ds_11hcfg->param.dfs_testing.usr_cac_restart;
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get DFS W53 settings
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_dfs53cfg(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11h_cfg *ds_11hcfg = NULL;
	int ret = 0;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DFS53_CFG);
	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11h_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	ds_11hcfg = (mlan_ds_11h_cfg *)req->pbuf;
	ds_11hcfg->sub_command = MLAN_OID_11H_DFS_W53_CFG;
	req->req_id = MLAN_IOCTL_11H_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of args !\n");
			ret = -EINVAL;
			goto done;
		}
		if (data > DFS_W53_OLD) {
			PRINTM(MERROR, "Invalid config !\n");
			ret = -EINVAL;
			goto done;
		}

		ds_11hcfg->param.dfs_w53_cfg.dfs53cfg = (t_u8)data;
		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		moal_memcpy_ext(priv->phandle, respbuf,
				(t_u8 *)&ds_11hcfg->param.dfs_w53_cfg.dfs53cfg,
				sizeof(ds_11hcfg->param.dfs_w53_cfg.dfs53cfg),
				respbuflen);
		ret = sizeof(t_u8);
	}

	PRINTM(MIOCTL, "dfs w53 cfg %d\n",
	       ds_11hcfg->param.dfs_w53_cfg.dfs53cfg);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief determine the center frquency center index for bandwidth
 *         of 80 MHz and 160 MHz
 *
 ** @param priv          Pointer to moal_private structure
 *  @param band         band
 *  @param pri_chan     primary channel
 *  @param chan_bw      channel bandwidth
 *
 *  @return             channel center frequency center, if found; O, otherwise
 */

t_u8 woal_get_center_freq_idx(moal_private *priv, t_u8 band, t_u32 pri_chan,
			      t_u8 chan_bw)
{
	t_u8 center_freq_idx = 0;

	if (band & BAND_AAC) {
		switch (pri_chan) {
		case 36:
		case 40:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 38;
				break;
			}
			/* fall through */
		case 44:
		case 48:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 46;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 42;
				break;
			}
			/* fall through */
		case 52:
		case 56:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 54;
				break;
			}
			/* fall through */
		case 60:
		case 64:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 62;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 58;
				break;
			} else if (chan_bw == CHANNEL_BW_160MHZ) {
				center_freq_idx = 50;
				break;
			}
			/* fall through */
		case 68:
		case 72:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 70;
				break;
			}
			/* fall through */
		case 76:
		case 80:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 78;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 74;
				break;
			}
			/* fall through */
		case 84:
		case 88:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 86;
				break;
			}
			/* fall through */
		case 92:
		case 96:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 94;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 90;
				break;
			}
			/* fall through */
		case 100:
		case 104:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 102;
				break;
			}
			/* fall through */
		case 108:
		case 112:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 110;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 106;
				break;
			}
			/* fall through */
		case 116:
		case 120:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 118;
				break;
			}
			/* fall through */
		case 124:
		case 128:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 126;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 122;
			} else if (chan_bw == CHANNEL_BW_160MHZ) {
				center_freq_idx = 114;
			}
			break;
		case 132:
		case 136:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 134;
				break;
			}
			/* fall through */
		case 140:
		case 144:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 126;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 138;
			}
			break;
		case 149:
		case 153:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 151;
				break;
			}
			/* fall through */
		case 157:
		case 161:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 159;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 155;
				break;
			}
			/* fall through */
		case 165:
		case 169:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 167;
				break;
			}
			/* fall through */
		case 173:
		case 177:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 175;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 171;
				break;
			}
			/* fall through */
		case 184:
		case 188:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 186;
				break;
			}
			/* fall through */
		case 192:
		case 196:
			if (chan_bw == CHANNEL_BW_40MHZ_ABOVE ||
			    chan_bw == CHANNEL_BW_40MHZ_BELOW) {
				center_freq_idx = 194;
				break;
			} else if (chan_bw == CHANNEL_BW_80MHZ) {
				center_freq_idx = 190;
				break;
			}
			/* fall through */
		default: /* error. go to the default */
			center_freq_idx = 42;
		}
	}
	return center_freq_idx;
}

/**
 *  @brief determine the center frquency center index for bandwidth
 *         of 80 MHz and 160 MHz
 *
 ** @param priv          Pointer to moal_private structure
 *  @param block_tx      0-no need block traffic 1- need block traffic
 *  @param oper_class    oper_class
 *  @param channel       channel
 *  @param switch count  how many csa/ecsa beacon will send out
 *  @param band_width    1-40Mhz above, 3-40Mhz below, 4-80Mhz, 5-160Mhz
 *  @param ecsa          MTRUE/MFALSE;
 *
 *  @return             channel center frequency center, if found; O, otherwise
 */

static int woal_channel_switch(moal_private *priv, t_u8 block_tx,
			       t_u8 oper_class, t_u8 channel, t_u8 switch_count,
			       t_u8 band_width, t_u8 ecsa)
{
	IEEEtypes_ExtChanSwitchAnn_t *ext_chan_switch = NULL;
	IEEEtypes_ChanSwitchAnn_t *chan_switch = NULL;
	custom_ie *pcust_chansw_ie = NULL;
	t_u8 center_freq_idx = 0;
	IEEEtypes_Header_t *pChanSwWrap_ie = NULL;
	IEEEtypes_WideBWChanSwitch_t *pbwchansw_ie = NULL;
	IEEEtypes_VhtTpcEnvelope_t *pvhttpcEnv_ie = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	t_u8 bw;
	t_u8 new_oper_class = oper_class;
	int ret = 0;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	misc->param.cust_ie.type = TLV_TYPE_MGMT_IE;
	misc->param.cust_ie.len = (sizeof(custom_ie) - MAX_IE_SIZE);

	pcust_chansw_ie = (custom_ie *)&misc->param.cust_ie.ie_data_list[0];
	pcust_chansw_ie->ie_index = 0xffff; /*Auto index */
	pcust_chansw_ie->ie_length = sizeof(IEEEtypes_ChanSwitchAnn_t);
	pcust_chansw_ie->mgmt_subtype_mask =
		MGMT_MASK_BEACON | MGMT_MASK_PROBE_RESP; /*Add IE for
							    BEACON/probe resp*/
	chan_switch = (IEEEtypes_ChanSwitchAnn_t *)pcust_chansw_ie->ie_buffer;
	chan_switch->element_id = CHANNEL_SWITCH_ANN;
	chan_switch->len = 3;
	chan_switch->chan_switch_mode = block_tx;
	chan_switch->new_channel_num = channel;
	chan_switch->chan_switch_count = switch_count;
	DBG_HEXDUMP(MCMD_D, "CSA IE", (t_u8 *)pcust_chansw_ie->ie_buffer,
		    pcust_chansw_ie->ie_length);
	switch (band_width) {
	case CHANNEL_BW_40MHZ_ABOVE:
	case CHANNEL_BW_40MHZ_BELOW:
		bw = 40;
		break;
	case CHANNEL_BW_80MHZ:
		bw = 80;
		break;
	case CHANNEL_BW_160MHZ:
		bw = 160;
		break;
	default:
		bw = 20;
		break;
	}
	if (!new_oper_class && ecsa)
		woal_priv_get_nonglobal_operclass_by_bw_channel(
			priv, bw, channel, &new_oper_class);
	if (new_oper_class) {
		pcust_chansw_ie->ie_length +=
			sizeof(IEEEtypes_ExtChanSwitchAnn_t);
		ext_chan_switch =
			(IEEEtypes_ExtChanSwitchAnn_t
				 *)(pcust_chansw_ie->ie_buffer +
				    sizeof(IEEEtypes_ChanSwitchAnn_t));
		ext_chan_switch->element_id = EXTEND_CHANNEL_SWITCH_ANN;
		ext_chan_switch->len = 4;
		ext_chan_switch->chan_switch_mode = block_tx;
		ext_chan_switch->new_oper_class = new_oper_class;
		ext_chan_switch->new_channel_num = channel;
		ext_chan_switch->chan_switch_count = switch_count;
		DBG_HEXDUMP(MCMD_D, "ECSA IE",
			    (t_u8 *)(pcust_chansw_ie->ie_buffer +
				     sizeof(IEEEtypes_ChanSwitchAnn_t)),
			    pcust_chansw_ie->ie_length -
				    sizeof(IEEEtypes_ChanSwitchAnn_t));
	}
	/* bandwidth 40/80/160 should set channel switch wrapper ie for 11ac 5G
	 * channel*/
	if (band_width && channel > 14) {
		pChanSwWrap_ie =
			(IEEEtypes_Header_t *)(pcust_chansw_ie->ie_buffer +
					       pcust_chansw_ie->ie_length);
		pChanSwWrap_ie->element_id = EXT_POWER_CONSTR;
		pChanSwWrap_ie->len = sizeof(IEEEtypes_WideBWChanSwitch_t);

		pbwchansw_ie = (IEEEtypes_WideBWChanSwitch_t
					*)((t_u8 *)pChanSwWrap_ie +
					   sizeof(IEEEtypes_Header_t));
		pbwchansw_ie->ieee_hdr.element_id = BW_CHANNEL_SWITCH;
		pbwchansw_ie->ieee_hdr.len =
			sizeof(IEEEtypes_WideBWChanSwitch_t) -
			sizeof(IEEEtypes_Header_t);

		center_freq_idx = woal_get_center_freq_idx(priv, BAND_AAC,
							   channel, band_width);
		if (band_width == CHANNEL_BW_40MHZ_ABOVE ||
		    band_width == CHANNEL_BW_40MHZ_BELOW) {
			pbwchansw_ie->new_channel_width = 0;
			pbwchansw_ie->new_channel_center_freq0 =
				center_freq_idx;
		} else if (band_width == CHANNEL_BW_80MHZ) {
			pbwchansw_ie->new_channel_width = 1;
			pbwchansw_ie->new_channel_center_freq0 =
				center_freq_idx - 4;
			pbwchansw_ie->new_channel_center_freq1 =
				center_freq_idx + 4;
		} else if (band_width == CHANNEL_BW_160MHZ) {
			pbwchansw_ie->new_channel_width = 2;
			pbwchansw_ie->new_channel_center_freq0 =
				center_freq_idx - 8;
			pbwchansw_ie->new_channel_center_freq1 =
				center_freq_idx + 8;
		} else
			PRINTM(MERROR,
			       "Invalid bandwidth.Support value 1/3/4/5 for 40+/40-/80/160MHZ\n");

		/*prepare the VHT Transmit Power Envelope IE*/
		pvhttpcEnv_ie =
			(IEEEtypes_VhtTpcEnvelope_t
				 *)((t_u8 *)pChanSwWrap_ie +
				    sizeof(IEEEtypes_Header_t) +
				    sizeof(IEEEtypes_WideBWChanSwitch_t));
		pvhttpcEnv_ie->ieee_hdr.element_id = VHT_TX_POWER_ENV;
		pvhttpcEnv_ie->ieee_hdr.len =
			sizeof(IEEEtypes_VhtTpcEnvelope_t) -
			sizeof(IEEEtypes_Header_t);
		/* Local Max TX Power Count= 3,
		 * Local TX Power Unit Inter=EIP(0) */
		pvhttpcEnv_ie->tpc_info = 3;
		pvhttpcEnv_ie->local_max_tp_20mhz = 0xff;
		pvhttpcEnv_ie->local_max_tp_40mhz = 0xff;
		pvhttpcEnv_ie->local_max_tp_80mhz = 0xff;
		pChanSwWrap_ie->len += sizeof(IEEEtypes_VhtTpcEnvelope_t);
		pcust_chansw_ie->ie_length +=
			pChanSwWrap_ie->len + sizeof(IEEEtypes_Header_t);
		DBG_HEXDUMP(MCMD_D, "Channel switch wrapper IE",
			    (t_u8 *)pChanSwWrap_ie,
			    pChanSwWrap_ie->len + sizeof(IEEEtypes_Header_t));
	}
	if (block_tx) {
		if (netif_carrier_ok(priv->netdev))
			netif_carrier_off(priv->netdev);
		woal_stop_queue(priv->netdev);
		priv->uap_tx_blocked = MTRUE;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to set ECSA IE\n");
		ret = -EFAULT;
		goto done;
	}

	priv->phandle->chsw_wait_q_woken = MFALSE;
	/* wait for channel switch to complete  */
	wait_event_interruptible_timeout(
		priv->phandle->chsw_wait_q, priv->phandle->chsw_wait_q_woken,
		(u32)HZ * (switch_count + 2) * 110 / 1000);

	pcust_chansw_ie->ie_index = 0xffff; /*Auto index */
	pcust_chansw_ie->mgmt_subtype_mask = 0;
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to clear ECSA IE\n");
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get CFP table codes
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_cfp_code(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	int data[2] = {0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_ds_misc_cfp_code *cfp_code = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_CFP_CODE);

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfp_code = &misc_cfg->param.cfp_code;
	misc_cfg->sub_command = MLAN_OID_MISC_CFP_CODE;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 2) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		cfp_code->cfp_code_bg = data[0];
		if (user_data_len == 2)
			cfp_code->cfp_code_a = data[1];
		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		data[0] = cfp_code->cfp_code_bg;
		data[1] = cfp_code->cfp_code_a;
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get Tx/Rx antenna
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_set_get_tx_rx_ant(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_ds_radio_cfg *radio = NULL;
	mlan_ioctl_req *req = NULL;
	int data[3] = {0};
	mlan_status status = MLAN_STATUS_SUCCESS;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	struct wiphy *wiphy = priv->phandle->wiphy;
#endif

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ANT_CFG);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_radio_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	radio = (mlan_ds_radio_cfg *)req->pbuf;
	radio->sub_command = MLAN_OID_ANT_CFG;
	req->req_id = MLAN_IOCTL_RADIO_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 2) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (priv->phandle->feature_control & FEATURE_CTRL_STREAM_2X2) {
			radio->param.ant_cfg.tx_antenna = data[0];
			radio->param.ant_cfg.rx_antenna = data[0];
			if (user_data_len == 2)
				radio->param.ant_cfg.rx_antenna = data[1];
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
			if (IS_CARD9098(priv->phandle->card_type) ||
			    IS_CARD9097(priv->phandle->card_type)) {
				if (IS_STA_OR_UAP_CFG80211(
					    priv->phandle->params
						    .cfg80211_wext) &&
				    wiphy) {
					if (wiphy->bands[IEEE80211_BAND_2GHZ]) {
						if (((radio->param.ant_cfg
							      .tx_antenna &
						      0xFF) != 3 &&
						     (radio->param.ant_cfg
							      .tx_antenna &
						      0xFF) != 0) ||
						    ((radio->param.ant_cfg
							      .rx_antenna &
						      0xFF) != 3 &&
						     (radio->param.ant_cfg
							      .rx_antenna &
						      0xFF) != 0))
							wiphy->bands[IEEE80211_BAND_2GHZ]
								->ht_cap.mcs
								.rx_mask[1] = 0;
						else if ((radio->param.ant_cfg
								  .tx_antenna &
							  0xFF) == 3 ||
							 (radio->param.ant_cfg
								  .rx_antenna &
							  0xFF) == 3)
							wiphy->bands[IEEE80211_BAND_2GHZ]
								->ht_cap.mcs
								.rx_mask[1] =
								0xff;
						wiphy->bands[IEEE80211_BAND_2GHZ]
							->ht_cap.mcs.rx_mask[4] =
							0;
					}
					if (wiphy->bands[IEEE80211_BAND_5GHZ]) {
						if (((radio->param.ant_cfg
							      .tx_antenna &
						      0xFF00) != 0x300 &&
						     (radio->param.ant_cfg
							      .tx_antenna &
						      0xFF00) != 0) ||
						    ((radio->param.ant_cfg
							      .rx_antenna &
						      0xFF00) != 0x300 &&
						     (radio->param.ant_cfg
							      .rx_antenna &
						      0xFF00) != 0)) {
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->ht_cap.mcs
								.rx_mask[1] = 0;
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->vht_cap
								.vht_mcs
								.rx_mcs_map =
								0xfffe;
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->vht_cap
								.vht_mcs
								.tx_mcs_map =
								0xfffe;
						} else if ((radio->param.ant_cfg
								    .tx_antenna &
							    0xFF00) == 0x300 ||
							   (radio->param.ant_cfg
								    .rx_antenna &
							    0xFF00) == 0x300) {
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->ht_cap.mcs
								.rx_mask[1] =
								0xff;
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->vht_cap
								.vht_mcs
								.rx_mcs_map =
								0xfffa;
							wiphy->bands[IEEE80211_BAND_5GHZ]
								->vht_cap
								.vht_mcs
								.tx_mcs_map =
								0xfffa;
						}
					}
				}
			}
#endif
		} else {
			radio->param.ant_cfg_1x1.antenna = data[0];
			if (user_data_len == 2)
				radio->param.ant_cfg_1x1.evaluate_time =
					data[1];
		}
		req->action = MLAN_ACT_SET;
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (!user_data_len) {
		if (priv->phandle->feature_control & FEATURE_CTRL_STREAM_2X2) {
			data[0] = radio->param.ant_cfg.tx_antenna;
			data[1] = radio->param.ant_cfg.rx_antenna;
			if (data[0] && data[1])
				ret = sizeof(int) * 2;
			else
				ret = sizeof(int) * 1;
			moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
					sizeof(data), respbuflen);
		} else {
			data[0] = (int)radio->param.ant_cfg_1x1.antenna;
			data[1] = (int)radio->param.ant_cfg_1x1.evaluate_time;
			data[2] = (int)radio->param.ant_cfg_1x1.current_antenna;
			moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
					sizeof(data), respbuflen);
			ret = sizeof(data);
		}
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/*
 * @brief               Set/Get CWMode
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_set_get_cwmode(moal_private *priv, t_u8 *respbuf,
				    t_u32 respbuflen)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_cw_mode_ctrl *cwmode;
	int ret = 0;
	int header_len = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CWMODE_CTRL;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_CWMODE);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		ioctl_req->action = MLAN_ACT_SET;

		cwmode = (mlan_ds_cw_mode_ctrl *)(respbuf + header_len +
						  sizeof(t_u8));
		misc->param.cwmode.mode = cwmode->mode;
		misc->param.cwmode.txPower = cwmode->txPower;
		misc->param.cwmode.rateInfo = cwmode->rateInfo;
		misc->param.cwmode.channel = cwmode->channel;
		misc->param.cwmode.chanInfo = cwmode->chanInfo;
		misc->param.cwmode.pktLength = cwmode->pktLength;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&misc->param.cwmode,
			sizeof(misc->param.cwmode), respbuflen);
	ret = sizeof(misc->param.cwmode);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get out band independent reset
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return             Number of bytes written, negative for failure.
 */
static int woal_priv_ind_rst_cfg(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_ioctl_req *req = NULL;
	int data[2] = {0};
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_IND_RST_CFG);
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	memset(misc, 0, sizeof(mlan_ds_misc_cfg));
	misc->sub_command = MLAN_OID_MISC_IND_RST_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 2) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}

		if ((user_data_len == 1) || (user_data_len == 2)) {
			req->action = MLAN_ACT_SET;

			/* ir_mode */
			if (data[0] < 0 || data[0] > 2) {
				PRINTM(MERROR, "Invalid ir mode parameter!\n");
				ret = -EINVAL;
				goto done;
			}
			misc->param.ind_rst_cfg.ir_mode = data[0];

			/* gpio_pin */
			if (user_data_len == 2) {
				if ((data[1] != 0xFF) && (data[1] < 0)) {
					PRINTM(MERROR,
					       "Invalid gpio pin no!\n");
					ret = -EINVAL;
					goto done;
				}
				misc->param.ind_rst_cfg.gpio_pin = data[1];
			}
		}
	}
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = (int)misc->param.ind_rst_cfg.ir_mode;
	data[1] = (int)misc->param.ind_rst_cfg.gpio_pin;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief               Get/Set system clock
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_sysclock(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	int data[65];
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *cfg = NULL;
	int ret = 0, i = 0;
	int user_data_len = 0, header_len = 0;
	int data_length = 0, length_index = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SYSCLOCK);
	memset(data, 0, sizeof(data));
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
	}

	if (user_data_len > MLAN_MAX_CLK_NUM) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	cfg = (mlan_ds_misc_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_MISC_SYS_CLOCK;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	if (user_data_len) {
		/* SET operation */
		req->action = MLAN_ACT_SET;

		/* Set configurable clocks */
		cfg->param.sys_clock.sys_clk_type = MLAN_CLK_CONFIGURABLE;
		cfg->param.sys_clock.sys_clk_num =
			MIN(MLAN_MAX_CLK_NUM, user_data_len);
		for (i = 0; i < cfg->param.sys_clock.sys_clk_num; i++)
			cfg->param.sys_clock.sys_clk[i] = (t_u16)data[i];

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		/* GET operation */
		req->action = MLAN_ACT_GET;

		/* Get configurable clocks */
		cfg->param.sys_clock.sys_clk_type = MLAN_CLK_CONFIGURABLE;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}

		/* Current system clock */
		data[1] = (int)cfg->param.sys_clock.cur_sys_clk;
		data_length = 1;

		length_index =
			MIN(cfg->param.sys_clock.sys_clk_num, MLAN_MAX_CLK_NUM);

		/* Configurable clocks */
		for (i = 1; i <= length_index; i++)
			data[i + data_length] =
				(int)cfg->param.sys_clock.sys_clk[i - 1];

		data_length += length_index;

		/* Get supported clocks */
		cfg->param.sys_clock.sys_clk_type = MLAN_CLK_SUPPORTED;
		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}

		length_index =
			MIN(cfg->param.sys_clock.sys_clk_num, MLAN_MAX_CLK_NUM);

		/* Supported clocks */
		for (i = 1; i <= length_index; i++)
			data[i + data_length] =
				(int)cfg->param.sys_clock.sys_clk[i - 1];

		data_length += length_index;

		/* Send length as first element */
		data[0] = data_length;
		data_length++;

		moal_memcpy_ext(priv->phandle, respbuf, data,
				sizeof(int) * data_length, respbuflen);
		ret = data_length * sizeof(int);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief               Get GTK/PTK
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_get_key(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int ret = 0, copy_len = 0;
	int header_len = 0;
	unsigned int i;
	t_u8 key_ascii[256];
	t_u8 *tmp;
	mlan_ds_sec_cfg *sec = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_GET_KEY);
	if (strlen(respbuf) != header_len) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}
	memset(key_ascii, 0x00, sizeof(key_ascii));
	tmp = key_ascii;

	if (priv->media_connected == MFALSE) {
		PRINTM(MERROR, "Can't get key in un-associated state\n");
		ret = -EFAULT;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Get Unicast Key */
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_QUERY_KEY;
	sec->param.encrypt_key.key_index = 0;
	sec->param.encrypt_key.key_flags = 0;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (sec->param.encrypt_key.key_len) {
		sprintf((char *)tmp, "\n%s", "PTK: ");
		tmp += 5;
		for (i = 0; i < sec->param.encrypt_key.key_len; i++)
			tmp += sprintf((char *)tmp, "%02x",
				       sec->param.encrypt_key.key_material[i]);
	}

	/* Get Multicase Key */
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_QUERY_KEY;
	sec->param.encrypt_key.key_index = 0;
	sec->param.encrypt_key.key_flags = KEY_FLAG_GROUP_KEY;
	memset(sec->param.encrypt_key.mac_addr, 0x0, MLAN_MAC_ADDR_LENGTH);
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (sec->param.encrypt_key.key_len) {
		sprintf((char *)tmp, "\n%s", "GTK: ");
		tmp += 5;
		for (i = 0; i < sec->param.encrypt_key.key_len; i++)
			tmp += sprintf((char *)tmp, "%02x",
				       sec->param.encrypt_key.key_material[i]);
	}

	/* Get IGTK Key */
	req->req_id = MLAN_IOCTL_SEC_CFG;
	req->action = MLAN_ACT_GET;
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_QUERY_KEY;
	sec->param.encrypt_key.key_index = 0;
	sec->param.encrypt_key.key_flags = KEY_FLAG_AES_MCAST_IGTK;
	memset(sec->param.encrypt_key.mac_addr, 0x0, MLAN_MAC_ADDR_LENGTH);
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (sec->param.encrypt_key.key_len) {
		sprintf((char *)tmp, "\n%s", "IGTK: ");
		tmp += 6;
		for (i = 0; i < sec->param.encrypt_key.key_len; i++)
			tmp += sprintf((char *)tmp, "%02x",
				       sec->param.encrypt_key.key_material[i]);
	}

	copy_len = tmp - key_ascii;
	moal_memcpy_ext(priv->phandle, respbuf, &key_ascii, copy_len,
			respbuflen);
	ret = copy_len;
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               Associate to a specific indexed entry in the ScanTable
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_associate_ssid_bssid(moal_private *priv, t_u8 *respbuf,
					  t_u32 respbuflen)
{
	int ret = 0, copy_len = 0;
	int header_len = 0;
	mlan_ssid_bssid ssid_bssid;
#ifdef REASSOCIATION
	mlan_bss_info bss_info;
#endif
	char buf[64];
	t_u8 buflen;
	t_u8 mac_idx;
	t_u8 i;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ASSOCIATE);
	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}
	copy_len = strlen(respbuf) - header_len;
	mac_idx = 0;
	buflen = MIN(copy_len, (sizeof(buf) - 1));
	memset(buf, 0, sizeof(buf));
	memset(&ssid_bssid, 0, sizeof(ssid_bssid));

	if (buflen < (3 * ETH_ALEN) + 2) {
		PRINTM(MERROR,
		       "Associate: Insufficient length in IOCTL input\n");

		/* buffer should be at least 3 characters per BSSID octet "00:"
		**   plus a space separater and at least 1 char in the SSID
		*/
		ret = -EINVAL;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, buf, respbuf + header_len, buflen,
			sizeof(buf));

	/* Skip white space */
	for (i = 0; (i < buflen) && (buf[i] == ' '); i++)
		;

	/* Copy/Convert the BSSID */
	for (; (i < buflen) && (mac_idx < ETH_ALEN) && (buf[i] != ' '); i++) {
		if (buf[i] == ':') {
			mac_idx++;
		} else {
			ssid_bssid.bssid[mac_idx] = (t_u8)woal_atox(buf + i);

			while (((i < buflen) && isxdigit(buf[i + 1])))
				/* Skip entire hex value */
				i++;
		}
	}

	/* Skip one space between the BSSID and start of the SSID */
	i++;

	/* Copy the SSID */
	ssid_bssid.ssid.ssid_len = buflen - i;
	moal_memcpy_ext(priv->phandle, ssid_bssid.ssid.ssid, buf + i,
			sizeof(ssid_bssid.ssid.ssid),
			sizeof(ssid_bssid.ssid.ssid));

	PRINTM(MCMND, "iwpriv assoc: AP=[" MACSTR "], ssid(%d)=[%s]\n",
	       MAC2STR(ssid_bssid.bssid), (int)ssid_bssid.ssid.ssid_len,
	       ssid_bssid.ssid.ssid);

	if (MLAN_STATUS_SUCCESS !=
	    woal_bss_start(priv, MOAL_IOCTL_WAIT, &ssid_bssid)) {
		ret = -EFAULT;
		goto done;
	}

#ifdef REASSOCIATION
	memset(&bss_info, 0x00, sizeof(bss_info));
	if (MLAN_STATUS_SUCCESS ==
	    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
		moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.ssid,
				&bss_info.ssid, sizeof(mlan_802_11_ssid),
				sizeof(mlan_802_11_ssid));
		moal_memcpy_ext(priv->phandle, &priv->prev_ssid_bssid.bssid,
				&bss_info.bssid, MLAN_MAC_ADDR_LENGTH,
				sizeof(mlan_802_11_mac_addr));
	}
#endif /* REASSOCIATION */

done:
	LEAVE();
	return 0;
}

/* Maximum input output characters in group WOAL_SET_GET_256_CHAR */
#define MAX_IN_OUT_CHAR 256
/** Tx BF Global conf argument index */
#define BF_ENABLE_PARAM 1
#define SOUND_ENABLE_PARAM 2
#define FB_TYPE_PARAM 3
#define SNR_THRESHOLD_PARAM 4
#define SOUND_INTVL_PARAM 5
#define BF_MODE_PARAM 6
#define MAX_TX_BF_GLOBAL_ARGS 6
#define BF_CFG_ACT_GET 0
#define BF_CFG_ACT_SET 1

/**
 * @brief               Set/Get Transmit beamforming configuration
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_tx_bf_cfg(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int header_len = 0;
	int ret = 0, copy_len = 0;
	int bf_action = 0, interval = 0;
	int snr = 0, i, tmp_val = 0;
	t_u8 buf[MAX_IN_OUT_CHAR], char_count = 0;
	t_u8 *str, *token, *pos;
	t_u16 action = 0;

	mlan_ds_11n_tx_bf_cfg bf_cfg;
	mlan_trigger_sound_args *bf_sound = NULL;
	mlan_tx_bf_peer_args *tx_bf_peer = NULL;
	mlan_snr_thr_args *bf_snr = NULL;
	mlan_bf_periodicity_args *bf_periodicity = NULL;
	mlan_bf_global_cfg_args *bf_global = NULL;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TX_BF_CFG);
	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}
	memset(&bf_cfg, 0, sizeof(bf_cfg));
	/* Pointer to corresponding buffer */
	bf_sound = bf_cfg.body.bf_sound;
	tx_bf_peer = bf_cfg.body.tx_bf_peer;
	bf_snr = bf_cfg.body.bf_snr;
	bf_periodicity = bf_cfg.body.bf_periodicity;
	bf_global = &bf_cfg.body.bf_global_cfg;

	/* Total characters in buffer */
	char_count = strlen(respbuf) - header_len;
	copy_len = char_count;
	memset(buf, 0, sizeof(buf));
	if (char_count) {
		if (copy_len > sizeof(buf)) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
		moal_memcpy_ext(priv->phandle, buf, respbuf + header_len,
				copy_len, sizeof(buf));

		if (char_count > 1 && buf[1] != ';') {
			PRINTM(MERROR,
			       "No action argument. Separate with ';'\n");
			ret = -EINVAL;
			goto done;
		}
		/* Replace ';' with NULL in the string to separate args */
		for (i = 0; i < char_count; i++) {
			if (buf[i] == ';')
				buf[i] = '\0';
		}
		/* The first byte represents the beamforming action */
		if (woal_atoi(&bf_action, &buf[0]) != MLAN_STATUS_SUCCESS) {
			ret = -EINVAL;
			goto done;
		}
		switch (bf_action) {
		case BF_GLOBAL_CONFIGURATION:
			if (char_count == 1) {
				action = MLAN_ACT_GET;
				bf_cfg.action = BF_CFG_ACT_GET;
			} else {
				action = MLAN_ACT_SET;
				bf_cfg.action = BF_CFG_ACT_SET;
				/* Eliminate action field */
				token = &buf[2];
				for (i = 1, str = &buf[2]; token != NULL; i++) {
					token = strstr(str, " ");
					pos = str;
					if (token != NULL) {
						*token = '\0';
						str = token + 1;
					}
					woal_atoi(&tmp_val, pos);
					switch (i) {
					case BF_ENABLE_PARAM:
						bf_global->bf_enbl =
							(t_u8)tmp_val;
						break;
					case SOUND_ENABLE_PARAM:
						bf_global->sounding_enbl =
							(t_u8)tmp_val;
						break;
					case FB_TYPE_PARAM:
						bf_global->fb_type =
							(t_u8)tmp_val;
						break;
					case SNR_THRESHOLD_PARAM:
						bf_global->snr_threshold =
							(t_u8)tmp_val;
						break;
					case SOUND_INTVL_PARAM:
						bf_global->sounding_interval =
							(t_u16)tmp_val;
						break;
					case BF_MODE_PARAM:
						bf_global->bf_mode =
							(t_u8)tmp_val;
						break;
					default:
						PRINTM(MERROR,
						       "Invalid Argument\n");
						ret = -EINVAL;
						goto done;
					}
				}
			}
			break;
		case TRIGGER_SOUNDING_FOR_PEER:
			/* First arg  = 2   BfAction
			 * Second arg = 17  MAC "00:50:43:20:BF:64" */
			if (char_count != 19) {
				PRINTM(MERROR, "Invalid argument\n");
				ret = -EINVAL;
				goto done;
			}
			woal_mac2u8(bf_sound->peer_mac, &buf[2]);
			action = MLAN_ACT_SET;
			bf_cfg.action = BF_CFG_ACT_SET;
			break;
		case SET_GET_BF_PERIODICITY:
			/* First arg  = 2   BfAction
			 * Second arg = 18  MAC "00:50:43:20:BF:64;"
			 * Third arg =  1  (min char)  TX BF interval
			 *              10 (max char)  u32 maximum value
			 * 4294967295 */
			if (char_count < 19 || char_count > 30) {
				PRINTM(MERROR, "Invalid argument\n");
				ret = -EINVAL;
				goto done;
			}

			woal_mac2u8(bf_periodicity->peer_mac, &buf[2]);
			if (char_count == 19) {
				action = MLAN_ACT_GET;
				bf_cfg.action = BF_CFG_ACT_GET;
			} else {
				action = MLAN_ACT_SET;
				bf_cfg.action = BF_CFG_ACT_SET;
				if (woal_atoi(&interval, &buf[20]) !=
				    MLAN_STATUS_SUCCESS) {
					ret = -EINVAL;
					goto done;
				}
				bf_periodicity->interval = interval;
			}
			break;
		case TX_BF_FOR_PEER_ENBL:
			/* Handle only SET operation here
			 * First arg  = 2   BfAction
			 * Second arg = 18  MAC "00:50:43:20:BF:64;"
			 * Third arg  = 2   enable/disable bf
			 * Fourth arg = 2   enable/disable sounding
			 * Fifth arg  = 1   FB Type */
			if (char_count != 25 && char_count != 1) {
				PRINTM(MERROR, "Invalid argument\n");
				ret = -EINVAL;
				goto done;
			}
			if (char_count == 1) {
				action = MLAN_ACT_GET;
				bf_cfg.action = BF_CFG_ACT_GET;
			} else {
				woal_mac2u8(tx_bf_peer->peer_mac, &buf[2]);
				woal_atoi(&tmp_val, &buf[20]);
				tx_bf_peer->bf_enbl = (t_u8)tmp_val;
				woal_atoi(&tmp_val, &buf[22]);
				tx_bf_peer->sounding_enbl = (t_u8)tmp_val;
				woal_atoi(&tmp_val, &buf[24]);
				tx_bf_peer->fb_type = (t_u8)tmp_val;
				action = MLAN_ACT_SET;
				bf_cfg.action = BF_CFG_ACT_SET;
			}
			break;
		case SET_SNR_THR_PEER:
			/* First arg  = 2   BfAction
			 * Second arg = 18  MAC "00:50:43:20:BF:64;"
			 * Third arg  = 1/2 SNR u8 - can be 1/2 charerters */
			if (char_count != 1 &&
			    !(char_count == 21 || char_count == 22)) {
				PRINTM(MERROR, "Invalid argument\n");
				ret = -EINVAL;
				goto done;
			}
			if (char_count == 1) {
				action = MLAN_ACT_GET;
				bf_cfg.action = BF_CFG_ACT_GET;
			} else {
				woal_mac2u8(bf_snr->peer_mac, &buf[2]);
				if (woal_atoi(&snr, &buf[20]) !=
				    MLAN_STATUS_SUCCESS) {
					ret = -EINVAL;
					goto done;
				}
				bf_snr->snr = snr;
				action = MLAN_ACT_SET;
				bf_cfg.action = BF_CFG_ACT_SET;
			}
			break;
		default:
			ret = -EINVAL;
			goto done;
		}

		/* Save the value */
		bf_cfg.bf_action = bf_action;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_tx_bf_cfg(priv, action, &bf_cfg)) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		ret = -EINVAL;
		goto done;
	}

	switch (bf_action) {
	case BF_GLOBAL_CONFIGURATION:
		moal_memcpy_ext(priv->phandle, respbuf, bf_global,
				sizeof(mlan_bf_global_cfg_args), respbuflen);
		ret = sizeof(mlan_bf_global_cfg_args);
		break;
	case TRIGGER_SOUNDING_FOR_PEER:
		moal_memcpy_ext(priv->phandle, respbuf, bf_sound,
				sizeof(mlan_bf_global_cfg_args), respbuflen);
		ret = sizeof(mlan_bf_global_cfg_args);
		break;
	case SET_GET_BF_PERIODICITY:
		moal_memcpy_ext(priv->phandle, respbuf, bf_periodicity,
				sizeof(mlan_bf_periodicity_args), respbuflen);
		ret = sizeof(mlan_bf_periodicity_args);
		break;
	case TX_BF_FOR_PEER_ENBL:
		moal_memcpy_ext(priv->phandle, respbuf, tx_bf_peer,
				sizeof(mlan_tx_bf_peer_args), respbuflen);
		ret = sizeof(mlan_tx_bf_peer_args);
		break;
	case SET_SNR_THR_PEER:
		moal_memcpy_ext(priv->phandle, respbuf, bf_snr,
				sizeof(mlan_snr_thr_args), respbuflen);
		ret = sizeof(mlan_snr_thr_args);
		break;
	default:
		ret = 0;
	}

done:
	LEAVE();
	return ret;
}

#ifdef SDIO
/**
 * @brief               Cmd53 read/write register
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_cmd53rdwr(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int header_len = 0;
	int ret = 0;
	t_u8 *buf = NULL;
	t_u8 *data = NULL;
	t_u8 rw, func, mode;
	t_u16 blklen = 0, blknum = 0;
	int reg = 0;
	t_u32 pattern_len = 0, total_len = 0;
	t_u16 cmd_len;
	gfp_t flag;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_SD_CMD53_RW);

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	data = kzalloc(WOAL_2K_BYTES, flag);
	if (!data) {
		PRINTM(MERROR, "Cannot allocate buffer for command!\n");
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, &cmd_len, respbuf + header_len,
			sizeof(cmd_len), sizeof(cmd_len));
	buf = respbuf + header_len + sizeof(cmd_len);

	rw = buf[0]; /* read/write (0/1) */
	func = buf[1]; /* func (0/1/2) */
	reg = buf[5]; /* address */
	reg = (reg << 8) | buf[4];
	reg = (reg << 8) | buf[3];
	reg = (reg << 8) | buf[2];
	mode = buf[6]; /* byte mode/block mode (0/1) */
	blklen = buf[8]; /* block size */
	blklen = (blklen << 8) | buf[7];
	blknum = buf[10]; /* block number or byte number */
	blknum = (blknum << 8) | buf[9];

	if (mode == BYTE_MODE)
		blklen = 1;
	else
		mode = BLOCK_MODE;

	total_len = (mode == BLOCK_MODE) ? blknum * blklen : blknum;
	if (total_len > WOAL_2K_BYTES) {
		PRINTM(MERROR, "Total data length is too large!\n");
		ret = -EINVAL;
		goto done;
	}
	PRINTM(MINFO,
	       "CMD53 read/write, func = %d, addr = %#x, mode = %d, "
	       "block size = %d, block(byte) number = %d\n",
	       func, reg, mode, blklen, blknum);

	if (!rw) {
		sdio_claim_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (sdio_readsb(
			    ((struct sdio_mmc_card *)priv->phandle->card)->func,
			    respbuf, reg, total_len)) {
			PRINTM(MERROR,
			       "sdio_readsb: reading memory 0x%x failed\n",
			       reg);
			goto done;
		}
		sdio_release_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		ret = total_len;
	} else {
		int pos = 0;
		pattern_len = cmd_len - 11;
		if (pattern_len > total_len)
			pattern_len = total_len;

		/* Copy/duplicate the pattern to data buffer */
		for (pos = 0; pos < total_len; pos++)
			data[pos] = buf[11 + (pos % pattern_len)];
		sdio_claim_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
		if (sdio_writesb(
			    ((struct sdio_mmc_card *)priv->phandle->card)->func,
			    reg, data, total_len))
			PRINTM(MERROR,
			       "sdio_writesb: writing memory 0x%x failed\n",
			       reg);
		sdio_release_host(
			((struct sdio_mmc_card *)priv->phandle->card)->func);
	}

done:
	kfree(data);
	LEAVE();
	return ret;
}
#endif /* SDIO */

/**
 * @brief               Set/Get Port Control mode
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_port_ctrl(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int header_len = 0, user_data_len = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_sec_cfg *sec = NULL;
	int ret = 0, data = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;
	ENTER();

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_sec_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	sec = (mlan_ds_sec_cfg *)req->pbuf;
	sec->sub_command = MLAN_OID_SEC_CFG_PORT_CTRL_ENABLED;
	req->req_id = MLAN_IOCTL_SEC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PORT_CTRL);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len == 1) {
			sec->param.port_ctrl_enabled = data;
			req->action = MLAN_ACT_SET;
		} else {
			PRINTM(MERROR, "Invalid number of parameters\n");
			ret = -EINVAL;
			goto done;
		}
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		moal_memcpy_ext(priv->phandle, respbuf,
				&sec->param.port_ctrl_enabled, sizeof(int),
				respbuflen);
		ret = sizeof(int);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               Private IOCTL entry to get the By-passed TX packet from
 * upper layer
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_bypassed_packet(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	int header_len = 0;
	int ret = 0;
	struct sk_buff *skb = NULL;
	struct ethhdr *eth;
	t_u16 moreLen = 0, copyLen = 0;
	ENTER();

#define MLAN_BYPASS_PKT_EXTRA_OFFSET (4)

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PB_BYPASS);
	copyLen = strlen(respbuf) - header_len;
	moreLen = MLAN_MIN_DATA_HEADER_LEN + MLAN_BYPASS_PKT_EXTRA_OFFSET +
		  sizeof(mlan_buffer);

	skb = alloc_skb(copyLen + moreLen, GFP_KERNEL);
	if (skb == NULL) {
		PRINTM(MERROR, "kmalloc no memory !!\n");
		LEAVE();
		return -ENOMEM;
	}

	skb_reserve(skb, moreLen);

	moal_memcpy_ext(priv->phandle, skb_put(skb, copyLen),
			respbuf + header_len, copyLen, copyLen);

	eth = (struct ethhdr *)skb->data;
	eth->h_proto = __constant_htons(eth->h_proto);
	skb->dev = priv->netdev;

	HEXDUMP("Bypass TX Data", skb->data, MIN(skb->len, 100));

	woal_hard_start_xmit(skb, priv->netdev);

	LEAVE();
	return ret;
}

/**
 * @brief               Set Robustcoex gpiocfg
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_robustcoex(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int header_len = 0, user_data_len = 0;
	int ret = 0, data[3] = {0};
	mlan_ds_misc_cfg *robust_coex_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	robust_coex_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	while (respbuf[0] == ' ') {
		/** skip space */
		respbuf++;
	}

	if (strncmp(respbuf, "gpiocfg", strlen("gpiocfg")) == 0) {
		header_len = strlen("gpiocfg") + 1;
		parse_arguments(respbuf + header_len, data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len > 3) {
			PRINTM(MERROR, "Invalid parameter number\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] != ROBUSTCOEX_GPIOCFG_ENABLE &&
		    data[0] != ROBUSTCOEX_GPIOCFG_DISABLE) {
			PRINTM(MERROR, "Invalid parameter number\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] == ROBUSTCOEX_GPIOCFG_ENABLE) {
			if (user_data_len != 3) {
				PRINTM(MMSG,
				       "Please provide gpio num and gpio polarity for ROBUSTCOEX_GPIOCFG_ENABLE\n");
				ret = -EINVAL;
				goto done;
			}
			robust_coex_cfg->param.robustcoexparams.method =
				ROBUSTCOEX_GPIO_CFG;
			robust_coex_cfg->param.robustcoexparams.enable =
				ROBUSTCOEX_GPIOCFG_ENABLE;
			robust_coex_cfg->param.robustcoexparams.gpio_num =
				data[1];
			robust_coex_cfg->param.robustcoexparams.gpio_polarity =
				data[2];
		} else {
			robust_coex_cfg->param.robustcoexparams.method =
				ROBUSTCOEX_GPIO_CFG;
			robust_coex_cfg->param.robustcoexparams.enable =
				ROBUSTCOEX_GPIOCFG_DISABLE;
			robust_coex_cfg->param.robustcoexparams.gpio_num = 0;
			robust_coex_cfg->param.robustcoexparams.gpio_polarity =
				0;
		}
		req->action = MLAN_ACT_SET;
		req->req_id = MLAN_IOCTL_MISC_CFG;
		robust_coex_cfg->sub_command = MLAN_OID_MISC_ROBUSTCOEX;
	} else {
		PRINTM(MERROR, "Invalid parameter\n");
		ret = -EFAULT;
		goto done;
	}
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
 * @brief               Set DMCS mapping policy or get DMCS status
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_dmcs(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int header_len = 0, user_data_len = 0;
	int ret = 0, data[2] = {0};
	mlan_ds_misc_cfg *dmcs_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DMCS);
	dmcs_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	if (user_data_len > 2) {
		PRINTM(MERROR, "Invalid number of args! %d\n", user_data_len);
		ret = -EINVAL;
		goto done;
	}
	req->req_id = MLAN_IOCTL_MISC_CFG;
	dmcs_cfg->sub_command = MLAN_OID_MISC_DMCS_CONFIG;
	dmcs_cfg->param.dmcs_policy.subcmd = data[0];
	switch (data[0]) {
	case 0:
		if (user_data_len != 2) {
			PRINTM(MERROR, "Please provide mapping policy\n");
			ret = -EINVAL;
			goto done;
		}
		req->action = MLAN_ACT_SET;
		dmcs_cfg->param.dmcs_policy.mapping_policy = data[1];
		break;
	case 1:
		req->action = MLAN_ACT_GET;
		break;
	default:
		break;
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	if (req->action == MLAN_ACT_GET) {
		moal_memcpy_ext(priv->phandle, respbuf,
				&dmcs_cfg->param.dmcs_status,
				sizeof(mlan_ds_misc_dmcs_status), respbuflen);
	}
	ret = sizeof(mlan_ds_misc_dmcs_status);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 * @brief               Set and get boot sleep configure
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_bootsleep(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	int ret = MLAN_STATUS_SUCCESS;
	int user_data_len = 0;
	int header_len = 0;
	int allowed = 1;
	int data[1] = {0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		LEAVE();
		return -ENOMEM;
	}

	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_BOOT_SLEEP;
	req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_BOOTSLEEP);

	if (strlen(respbuf) == header_len) {
		req->action = MLAN_ACT_GET;
	} else {
		req->action = MLAN_ACT_SET;
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != allowed) {
			PRINTM(MERROR, "Invalid number of args! %d\n",
			       user_data_len);
			ret = -EINVAL;
			goto done;
		}
		misc->param.boot_sleep = data[0] ? 1 : 0;
		PRINTM(MIOCTL, "boot sleep cfg:%u\n", misc->param.boot_sleep);
	}

	ret = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT_TIMEOUT);
	if (ret != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, &misc->param.boot_sleep,
			sizeof(misc->param.boot_sleep), respbuflen);
	ret = sizeof(misc->param.boot_sleep);

	PRINTM(MIOCTL, "boot sleep cfg: %u\n", misc->param.boot_sleep);

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if defined(PCIE)
/**
 * @brief               Enable SSU support
 * @param priv          Pointer to moal_private structure
 * @param used_len 		used length
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_ssu_cmd(moal_private *priv, t_u8 used_len, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int ret = 0;
	mlan_ds_misc_cfg *ssu_cfg = NULL;
	mlan_ioctl_req *req = NULL;
	ssu_params_cfg *ssu_params;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	ssu_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	memset(&ssu_cfg->param.ssu_params, 0, sizeof(mlan_ds_ssu_params));
	if (!used_len) {
		req->action = MLAN_ACT_SET;
		ssu_cfg->param.ssu_params.nskip = 0;
		ssu_cfg->param.ssu_params.nsel = 1;
		ssu_cfg->param.ssu_params.adcdownsample = 3;
		ssu_cfg->param.ssu_params.mask_adc_pkt = 0;
		ssu_cfg->param.ssu_params.out_16bits = 1;
	} else {
		ssu_params = (ssu_params_cfg *)respbuf;
		DBG_HEXDUMP(MCMD_D, "User SSU params:", respbuf,
			    sizeof(mlan_ds_ssu_params));
		if (ssu_params->ssu_mode == 2)
			req->action = MLAN_ACT_DEFAULT;
		else {
			req->action = MLAN_ACT_SET;
			ssu_cfg->param.ssu_params.nskip = ssu_params->nskip;
			ssu_cfg->param.ssu_params.nsel = ssu_params->nsel;
			ssu_cfg->param.ssu_params.adcdownsample =
				ssu_params->adcdownsample;
			ssu_cfg->param.ssu_params.mask_adc_pkt =
				ssu_params->mask_adc_pkt;
			ssu_cfg->param.ssu_params.out_16bits =
				ssu_params->out_16bits;
			ssu_cfg->param.ssu_params.spec_pwr_enable =
				ssu_params->spec_pwr_enable;
			ssu_cfg->param.ssu_params.rate_deduction =
				ssu_params->rate_deduction;
			ssu_cfg->param.ssu_params.n_pkt_avg =
				ssu_params->n_pkt_avg;
		}
	}
	req->req_id = MLAN_IOCTL_MISC_CFG;
	ssu_cfg->sub_command = MLAN_OID_MISC_SSU;

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
#endif

/**
 * @brief               configure 11ax HE capability or HE operation
 *
 *
 *  @param priv    Pointer to the mlan_private driver data struct
 *  @param respbuf      A pointer to response buffer
 *  @param len          length used
 *  @param respbuflen   Available length of response buffer
 *
 *  @return         Number of bytes written if successful else negative value
 */
static int woal_priv_11axcfg_cmd(moal_private *priv, t_u8 *respbuf, t_u8 len,
				 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_11ax_cfg *cfg = NULL;
	mlan_ds_11ax_he_cfg *data_ptr = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	data_ptr = (mlan_ds_11ax_he_cfg *)respbuf;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11ax_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_11AX_CFG;
	req->action = MLAN_ACT_SET;
	cfg = (mlan_ds_11ax_cfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_11AX_HE_CFG;
	if (len)
		moal_memcpy_ext(priv->phandle, (t_u8 *)&cfg->param.he_cfg,
				respbuf, len, sizeof(mlan_ds_11ax_he_cfg));
	else
		req->action = MLAN_ACT_GET;
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&cfg->param.he_cfg,
			sizeof(mlan_ds_11ax_he_cfg), respbuflen);
	ret = sizeof(mlan_ds_11ax_he_cfg);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}
/**
 * @brief               Configure TWT Setup parameters
 *
 * @param priv          Pointer to the mlan_private driver data struct
 * @param respbuf       A pointer to response buffer
 * @param len           Length used
 * @param respbuflen    Available length of response buffer
 *
 * @return              Number of bytes written if successful else negative
 * value
 */
static int woal_priv_twt_setup(moal_private *priv, t_u8 *respbuf, t_u8 len,
			       t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_twtcfg *cfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_twtcfg));
	if (req == NULL) {
		PRINTM(MERROR, "Failed to allocate ioctl_req!\n");
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_11AX_CFG;
	req->action = MLAN_ACT_SET;
	cfg = (mlan_ds_twtcfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_11AX_TWT_CFG;
	cfg->sub_id = MLAN_11AX_TWT_SETUP_SUBID;

	if (len) {
		moal_memcpy_ext(priv->phandle, (t_u8 *)&cfg->param.twt_setup,
				respbuf, len, sizeof(mlan_ds_twt_setup));
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "woal_request_ioctl failed!\n");
		ret = -EFAULT;
		goto done;
	}

	ret = sizeof(mlan_ds_twt_setup);
done:
	if (status != MLAN_STATUS_PENDING) {
		kfree(req);
	}
	LEAVE();
	return ret;
}

/**
 * @brief               Configure TWT Tear down parameters
 *
 * @param priv          Pointer to the mlan_private driver data struct
 * @param respbuf       A pointer to response buffer
 * @param len           Length used
 * @param respbuflen    Available length of response buffer
 *
 * @return              Number of bytes written if successful else negative
 * value
 */
static int woal_priv_twt_teardown(moal_private *priv, t_u8 *respbuf, t_u8 len,
				  t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_twtcfg *cfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_twtcfg));
	if (req == NULL) {
		PRINTM(MERROR, "Failed to allocate ioctl_req!\n");
		ret = -ENOMEM;
		goto done;
	}

	req->req_id = MLAN_IOCTL_11AX_CFG;
	req->action = MLAN_ACT_SET;
	cfg = (mlan_ds_twtcfg *)req->pbuf;
	cfg->sub_command = MLAN_OID_11AX_TWT_CFG;
	cfg->sub_id = MLAN_11AX_TWT_TEARDOWN_SUBID;

	if (len) {
		moal_memcpy_ext(priv->phandle, (t_u8 *)&cfg->param.twt_teardown,
				respbuf, len, sizeof(mlan_ds_twt_teardown));
	}

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "woal_request_ioctl failed!\n");
		ret = -EFAULT;
		goto done;
	}

	ret = sizeof(mlan_ds_twt_teardown);
done:
	if (status != MLAN_STATUS_PENDING) {
		kfree(req);
	}
	LEAVE();
	return ret;
}

#ifdef WIFI_DIRECT_SUPPORT
#if defined(UAP_CFG80211)
/**
 * @brief               Set/Get P2P NoA (Notice of Absence) parameters
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_cfg_noa(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	int header_len = 0, user_data_len = 0;
	int ret = 0, data[7];
	mlan_ds_wifi_direct_config noa_cfg;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_CFG_NOA);
	memset(&noa_cfg, 0, sizeof(noa_cfg));

	memset(data, 0, sizeof(data));
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);

	if (user_data_len > 5) {
		PRINTM(MERROR, "invalid parameters\n");
		ret = -EINVAL;
		goto done;
	}

	noa_cfg.flags |= WIFI_DIRECT_NOA;

	if (woal_p2p_config(priv, MLAN_ACT_GET, &noa_cfg) !=
	    MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Could not get P2P noa config\n");
		ret = -EINVAL;
		goto done;
	}

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		moal_memcpy_ext(priv->phandle, respbuf, &noa_cfg,
				sizeof(noa_cfg), respbuflen);
		ret = sizeof(noa_cfg);
	} else {
		switch (user_data_len) {
		case 5:
			noa_cfg.noa_interval = (t_u32)data[4];
			/* fall through */
		case 4:
			noa_cfg.noa_duration = (t_u32)data[3];
			/* fall through */
		case 3:
			if (data[2] < 1 || data[2] > 255) {
				PRINTM(MERROR,
				       "Invalid number of absence intervals\n");
				ret = -EINVAL;
				goto done;
			}
			noa_cfg.noa_count = (t_u8)data[2];
			/* fall through */
		case 2:
			if (data[1] < 0 || data[1] > 255) {
				PRINTM(MERROR, "Invalid Index\n");
				ret = -EINVAL;
				goto done;
			}
			noa_cfg.index = (t_u16)data[1];
			/* fall through */
		case 1:
			if (data[0] < 0 || data[0] > 1) {
				PRINTM(MERROR, "Invalid noa enable\n");
				ret = -EINVAL;
				goto done;
			}
			noa_cfg.noa_enable = (t_u8)data[0];
			noa_cfg.flags |= WIFI_DIRECT_NOA;
			break;
		default:
			break;
		}
		woal_p2p_config(priv, MLAN_ACT_SET, &noa_cfg);
	}

done:

	LEAVE();
	return ret;
}

/**
 * @brief               Set/Get P2P OPP-PS parameters
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_cfg_opp_ps(moal_private *priv, t_u8 *respbuf,
				t_u32 respbuflen)
{
	int header_len = 0, user_data_len = 0;
	int ret = 0, data[7];
	mlan_ds_wifi_direct_config opp_ps_cfg;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_CFG_OPP_PS);
	memset(&opp_ps_cfg, 0, sizeof(opp_ps_cfg));

	memset(data, 0, sizeof(data));
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);

	if (user_data_len > 2) {
		PRINTM(MERROR, "invalid parameters\n");
		ret = -EINVAL;
		goto done;
	}

	opp_ps_cfg.flags |= WIFI_DIRECT_OPP_PS;

	if (woal_p2p_config(priv, MLAN_ACT_GET, &opp_ps_cfg) !=
	    MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Could not get P2P opp ps config\n");
		ret = -EINVAL;
		goto done;
	}

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		moal_memcpy_ext(priv->phandle, respbuf, &opp_ps_cfg,
				sizeof(opp_ps_cfg), respbuflen);
		ret = sizeof(opp_ps_cfg);
	} else {
		switch (user_data_len) {
		case 2:
			opp_ps_cfg.ct_window = (t_u8)data[1];
			/* fall through */
		case 1:
			if (data[0] < 0 || data[0] > 1) {
				PRINTM(MERROR, "Invalid ps enable\n");
				ret = -EINVAL;
				goto done;
			}
			opp_ps_cfg.opp_ps_enable = (t_u8)data[0];
			opp_ps_cfg.flags |= WIFI_DIRECT_OPP_PS;
			/* fall through */
		default:
			break;
		}
		woal_p2p_config(priv, MLAN_ACT_SET, &opp_ps_cfg);
	}

done:

	LEAVE();
	return ret;
}
#endif
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#ifdef WIFI_DIRECT_SUPPORT
#define DEF_NOA_INTERVAL 100
/**
 ** @brief               Set/Get P2P NoA (Notice of Absence) parameters
 ** @param priv          Pointer to moal_private structure
 ** @param respbuf       Pointer to response buffer
 ** @param resplen       Response buffer length
 **
 **  @return             Number of bytes written, negative for failure.
 **/
static int woal_p2p_ps_cfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	int user_data_len = 0;
	int ret = 0, data[2];
	u32 duration = priv->phandle->noa_duration;
	u32 interval = 0;

	ENTER();
	if (strlen(respbuf) > strlen("P2P_PERIODIC_SLEEP")) {
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen("P2P_PERIODIC_SLEEP") + 1,
				data, ARRAY_SIZE(data), &user_data_len);
	}
	if ((user_data_len != 1) && (user_data_len != 2)) {
		PRINTM(MERROR,
		       " Invalid parameter number for P2P_PERIODIC_SLEEP");
		ret = -EINVAL;
		goto done;
	}
	if (data[0] < DEF_NOA_INTERVAL)
		interval = DEF_NOA_INTERVAL;
	else
		interval = (data[0] + DEF_NOA_INTERVAL - 1) / DEF_NOA_INTERVAL *
			   DEF_NOA_INTERVAL;

	if (user_data_len == 2)
		duration = data[1];
	if (duration >= interval) {
		PRINTM(MERROR,
		       " Invalid noa duration/interval! duration=%d interval=%d\n",
		       duration, interval);
		ret = -EINVAL;
		goto done;
	}
	priv->phandle->noa_interval = interval;
	priv->phandle->noa_duration = duration;
	PRINTM(MIOCTL, "configure noa interval=%d, duration=%d\n",
	       priv->phandle->noa_interval, priv->phandle->noa_duration);
done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 * @brief               Set/Get DFS repeater mode
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_dfs_repeater_cfg(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0, data[1] = {0};
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc_cfg = NULL;
	mlan_ds_misc_dfs_repeater *dfs_repeater = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DFS_REPEATER_CFG);

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc_cfg = (mlan_ds_misc_cfg *)req->pbuf;
	misc_cfg->sub_command = MLAN_OID_MISC_DFS_REAPTER_MODE;
	dfs_repeater =
		(mlan_ds_misc_dfs_repeater *)&misc_cfg->param.dfs_repeater;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of args! %d\n",
			       user_data_len);
			ret = -EINVAL;
			goto done;
		}
		if ((data[0] != MTRUE) && (data[0] != MFALSE)) {
			PRINTM(MERROR, "Invalid DFS repeater mode %d\n",
			       data[0]);
			ret = -EINVAL;
			goto done;
		}
		dfs_repeater->mode = (t_u16)data[0];

		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)dfs_repeater,
				sizeof(mlan_ds_misc_dfs_repeater), respbuflen);
		ret = sizeof(mlan_ds_misc_dfs_repeater);
	}

	/* Store current value of DFS repeater mode for futher references. eg.,
	 * for avoiding CAC timers
	 */
	priv->phandle->dfs_repeater_mode = dfs_repeater->mode;

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/**
 * @brief               Set/Get MIRACAST configuration parameters
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_miracast_cfg(moal_private *priv, t_u8 *respbuf,
				  t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0, data[3] = {0, 0, 0};

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_MIRACAST_CFG);

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		data[0] = priv->phandle->miracast_mode;
		data[1] = priv->phandle->miracast_scan_time;
		data[2] = priv->phandle->scan_chan_gap;

		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	} else {
		/* SET operation */
		memset(data, 0, sizeof(data));
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);

		if (user_data_len > 3) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] < 0 || data[0] > 2 || data[1] < 0 || data[2] < 0) {
			PRINTM(MERROR, "Invalid argument\n");
			ret = -EINVAL;
			goto done;
		}
	}

	if (user_data_len >= 1)
		priv->phandle->miracast_mode = (t_u8)data[0];
	if (user_data_len >= 2)
		priv->phandle->miracast_scan_time = (t_u16)data[1];
	if (user_data_len == 3)
		priv->phandle->scan_chan_gap = (t_u16)data[2];

done:
	LEAVE();
	return ret;
}

/**
 *   @brief Configuring scan gap for miracast mode
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             0 --success, otherwise failure
 */
int woal_set_scan_chan_gap(moal_private *priv, t_u8 *respbuf, int respbuflen)
{
	t_u32 data[2];
	int ret = 0;
	int user_data_len = 0;

	ENTER();

	if (strlen(respbuf) > strlen("SCAN_TIMING")) {
		memset((char *)data, 0, sizeof(data));
		parse_arguments(respbuf + strlen("SCAN_TIMING") + 1, data,
				ARRAY_SIZE(data), &user_data_len);
	}

	if (user_data_len != 2) {
		PRINTM(MERROR, "Invalid arguments for scan timing\n");
		ret = -EINVAL;
		goto done;
	}
	priv->phandle->miracast_scan_time = (t_u16)data[0];
	priv->phandle->scan_chan_gap = (t_u16)data[1];
done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 * @brief               Set/Get control to coex RX window size configuration
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_coex_rx_winsize(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0, data = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_11n_cfg *cfg_11n = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_COEX_RX_WINSIZE);

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_11n_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	req->req_id = MLAN_IOCTL_11N_CFG;
	cfg_11n = (mlan_ds_11n_cfg *)req->pbuf;
	cfg_11n->sub_command = MLAN_OID_11N_CFG_COEX_RX_WINSIZE;

	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of args! %d\n",
			       user_data_len);
			ret = -EINVAL;
			goto done;
		}
		if ((data != MTRUE) && (data != MFALSE)) {
			PRINTM(MERROR,
			       "Invalid coex RX window size parameter %d\n",
			       data);
			ret = -EINVAL;
			goto done;
		}
		cfg_11n->param.coex_rx_winsize = data;
		req->action = MLAN_ACT_SET;
	}

	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!user_data_len) {
		moal_memcpy_ext(priv->phandle, respbuf,
				(t_u8 *)&cfg_11n->param.coex_rx_winsize,
				sizeof(t_u32), respbuflen);
		ret = sizeof(t_u32);
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

#ifdef PCIE
/**
 * @brief               Read/Write PCIE register
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              Number of bytes written, negative for failure.
 */
static int woal_priv_pcie_reg_rw(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	moal_handle *handle = priv->phandle;
	int data[3];
	t_u32 reg;
	t_u32 value;
	int ret = MLAN_STATUS_SUCCESS;
	int user_data_len = 0, header_len = 0;

	ENTER();

	memset(data, 0, sizeof(data));
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PCIE_REG_RW);
	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}

	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	if ((user_data_len != 1) && (user_data_len != 2)) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}

	reg = (t_u32)data[0];
	if (user_data_len == 1) {
		if (moal_read_reg(handle, reg, &value)) {
			ret = -EFAULT;
			goto done;
		}
		data[1] = value;
	} else {
		value = data[1];
		if (moal_write_reg(handle, reg, value)) {
			ret = -EFAULT;
			goto done;
		}
	}
	moal_memcpy_ext(handle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	LEAVE();
	return ret;
}

/**
 * @brief               Read/Write PCIE register/memory from BAR0
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              Number of bytes written, negative for failure.
 */
static int woal_priv_pcie_bar0_reg_rw(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	moal_handle *handle = priv->phandle;
	pcie_service_card *card = (pcie_service_card *)handle->card;
	int data[3];
	t_u32 reg;
	t_u32 value;
	int ret = MLAN_STATUS_SUCCESS;
	int user_data_len = 0, header_len = 0;

	ENTER();

	memset(data, 0, sizeof(data));
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_PCIE_BAR0_REG_RW);
	if (strlen(respbuf) == header_len) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}

	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	if ((user_data_len != 1) && (user_data_len != 2)) {
		PRINTM(MERROR, "Invalid number of parameters\n");
		ret = -EINVAL;
		goto done;
	}

	reg = (t_u32)data[0];
	if (user_data_len == 1) {
		value = ioread32(card->pci_mmap + reg);
		if (value == MLAN_STATUS_FAILURE) {
			ret = -EFAULT;
			goto done;
		}
		data[1] = value;
	} else {
		value = data[1];
		iowrite32(value, card->pci_mmap + reg);
	}
	moal_memcpy_ext(handle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	LEAVE();
	return ret;
}
#endif

/**
 * @brief               Get SOC temperature
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              Number of bytes written, negative for failure.
 */
static int woal_priv_get_sensor_temp(moal_private *priv, t_u8 *respbuf,
				     t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *pcfg = NULL;
	int ret = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	pcfg = (mlan_ds_misc_cfg *)req->pbuf;
	pcfg->sub_command = MLAN_OID_MISC_GET_SENSOR_TEMP;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	req->action = MLAN_ACT_GET;

	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	memset(respbuf, 0, respbuflen);
	moal_memcpy_ext(priv->phandle, respbuf,
			&pcfg->param.sensor_temp.temperature, sizeof(t_u32),
			respbuflen);

	ret = sizeof(t_u32);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
/**
 * @brief               Enable/disable DFS offload
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length
 *
 * @return              Number of bytes written, negative for failure.
 */
static int woal_priv_dfs_offload_enable(moal_private *priv, t_u8 *respbuf,
					t_u32 respbuflen)
{
	struct wiphy *wiphy = NULL;
	int ret = 0, dfs_offload_en = 0, user_data_len = 0, header_len = 0,
	    dfs_offload;

	ENTER();

	if (priv && priv->wdev)
		wiphy = priv->wdev->wiphy;
	if (!wiphy) {
		PRINTM(MERROR, "wiphy is NULL\n");
		ret = -EFAULT;
		goto done;
	}
	dfs_offload = moal_extflg_isset(priv->phandle, EXT_DFS_OFFLOAD);
	if (woal_is_any_interface_active(priv->phandle)) {
		PRINTM(MERROR,
		       "DFS offload enable/disable do not allowed after BSS started!\n");
		ret = -EFAULT;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DFS_OFFLOAD);
	parse_arguments(respbuf + header_len, &dfs_offload_en,
			sizeof(dfs_offload_en) / sizeof(int), &user_data_len);
	if (user_data_len != 1) {
		PRINTM(MERROR, "Invalid number of args! %d\n", user_data_len);
		ret = -EINVAL;
		goto done;
	}

	if (dfs_offload_en != 0 && dfs_offload_en != 1) {
		PRINTM(MERROR, "Invalid args!\n");
		ret = -EINVAL;
		goto done;
	}
	if (dfs_offload != dfs_offload_en) {
		dfs_offload = dfs_offload_en;
		if (dfs_offload)
			moal_extflg_set(priv->phandle, EXT_DFS_OFFLOAD);
		else
			moal_extflg_clear(priv->phandle, EXT_DFS_OFFLOAD);
		woal_update_radar_chans_dfs_state(wiphy);
	}
done:
	LEAVE();
	return ret;
}
#endif
#endif

/**
 * @brief               Set/Get dynamic bandwidth
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length

 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_config_dyn_bw(moal_private *priv, t_u8 *respbuf,
				   t_u32 respbuflen)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;
	int user_data_len = 0, header_len = 0, data = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_DYN_BW;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DYN_BW);
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		ioctl_req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, &data,
				sizeof(data) / sizeof(int), &user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid number of args! %d\n",
			       user_data_len);
			ret = -EINVAL;
			goto done;
		}
		ioctl_req->action = MLAN_ACT_SET;
		misc->param.dyn_bw = (t_u16)data;
	}

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&misc->param.dyn_bw,
			sizeof(t_u16), respbuflen);
	ret = sizeof(t_u16);

	PRINTM(MIOCTL, "Dynamic bandwidth %d\n", misc->param.dyn_bw);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

#if defined(UAP_SUPPORT)
/**
 * @brief               Check validation of channel and oper class
 *
 * @param priv          Pointer to moal_private structure
 * @param channel       channel
 * @param oper_class    oper_class

 *  @return             SUCCESS/FAIL
 */
static int woal_check_valid_channel_operclass(moal_private *priv, int channel,
					      int oper_class)
{
	int ret = 0;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_OPER_CLASS_CHECK;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_GET;
	misc->param.bw_chan_oper.oper_class = (t_u8)oper_class;
	misc->param.bw_chan_oper.channel = (t_u8)channel;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 ** @brief               Set extended channel switch ie
 **
 ** @param priv          Pointer to moal_private structure
 ** @param respbuf       Pointer to response buffer
 ** @param resplen       Response buffer length
 **
 ** @return             Number of bytes written, negative for failure.
 **/
static int woal_priv_extend_channel_switch(moal_private *priv, t_u8 *respbuf,
					   t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0;
	int data[5] = {0};
	ENTER();

	if (!priv || !priv->phandle || (priv->bss_role != MLAN_BSS_ROLE_UAP) ||
	    (priv->bss_started != MTRUE)) {
		PRINTM(MERROR,
		       "priv or handle is null or interface is not AP/GO"
		       "or AP is not started\n");
		ret = -EFAULT;
		LEAVE();
		return ret;
	}

	parse_arguments(respbuf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_EXTEND_CHAN_SWITCH),
			data, ARRAY_SIZE(data), &user_data_len);

	if (sizeof(int) * user_data_len > sizeof(data)) {
		PRINTM(MERROR, "Too many arguments\n");
		ret = -EINVAL;
		LEAVE();
		return ret;
	}

	if (data[1]) {
		if (woal_check_valid_channel_operclass(priv, data[2],
						       data[1])) {
			PRINTM(MERROR, "Wrong channel switch parameters!\n");
			ret = -EINVAL;
			goto done;
		}
	}
	woal_channel_switch(priv, data[0], data[1], data[2], data[3], data[4],
			    MFALSE);
done:
	LEAVE();
	return ret;
}

/**
 * @brief               P2P extended channel switch
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length

 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_p2p_ecsa(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	int ret = 0;
	int user_data_len = 0, header_len = 0;
	int data[2] = {0};
	t_u8 bw = 0, oper_class = 0, channel = 0;
	IEEEtypes_ExtChanSwitchAnn_t *ext_chan_switch = NULL;
	custom_ie *pcust_chansw_ie = NULL;
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	if (priv->bss_role != MLAN_BSS_ROLE_UAP) {
		PRINTM(MERROR,
		       "Extended Channel Switch is only allowed for AP/GO mode\n");
		ret = -EFAULT;
		goto done;
	}

	if (priv->bss_started != MTRUE) {
		PRINTM(MERROR, "AP is not started!\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CUSTOM_IE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	ioctl_req->action = MLAN_ACT_SET;
	misc->param.cust_ie.type = TLV_TYPE_MGMT_IE;
	misc->param.cust_ie.len = (sizeof(custom_ie) - MAX_IE_SIZE);

	pcust_chansw_ie = (custom_ie *)&misc->param.cust_ie.ie_data_list[0];
	pcust_chansw_ie->ie_index = 0xffff; /*Auto index */
	pcust_chansw_ie->ie_length = sizeof(IEEEtypes_ExtChanSwitchAnn_t);
	pcust_chansw_ie->mgmt_subtype_mask =
		MGMT_MASK_BEACON | MGMT_MASK_PROBE_RESP; /*Add IE for
							    BEACON/probe resp*/
	ext_chan_switch =
		(IEEEtypes_ExtChanSwitchAnn_t *)pcust_chansw_ie->ie_buffer;

	header_len = strlen("P2P_ECSA");
	parse_arguments(respbuf + header_len + 1, data, ARRAY_SIZE(data),
			&user_data_len);

	if (user_data_len != 2) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EFAULT;
		goto done;
	}

	channel = data[0];
	/* bandwidth 20:20M 40:40M 80:80M*/
	bw = data[1];
	if (bw != 20 && bw != 40 && bw != 80) {
		PRINTM(MERROR, "Unsupported bandwidth\n");
		ret = -EINVAL;
		goto done;
	}
	if (channel >= 52 && channel <= 144) {
		PRINTM(MERROR, "Switch to DFS channel is not allowed!\n");
		ret = -EINVAL;
		goto done;
	}

	woal_priv_get_nonglobal_operclass_by_bw_channel(priv, bw, channel,
							&oper_class);
	if (oper_class == 0) {
		PRINTM(MERROR, "Wrong parameters!\n");
		ret = -EFAULT;
		goto done;
	}
	ext_chan_switch->element_id = EXTEND_CHANNEL_SWITCH_ANN;
	ext_chan_switch->len = 4;
	ext_chan_switch->chan_switch_mode = 1;
	ext_chan_switch->new_oper_class = oper_class;
	ext_chan_switch->new_channel_num = channel;
	ext_chan_switch->chan_switch_count = DEF_CHAN_SWITCH_COUNT;

	if (ext_chan_switch->chan_switch_mode) {
		if (netif_carrier_ok(priv->netdev))
			netif_carrier_off(priv->netdev);
		woal_stop_queue(priv->netdev);
		priv->uap_tx_blocked = MTRUE;
	}

	DBG_HEXDUMP(MCMD_D, "ECSA IE", (t_u8 *)pcust_chansw_ie->ie_buffer,
		    pcust_chansw_ie->ie_length);

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	priv->phandle->chsw_wait_q_woken = MFALSE;
	/* wait for channel switch to complete  */
	wait_event_interruptible_timeout(
		priv->phandle->chsw_wait_q, priv->phandle->chsw_wait_q_woken,
		(u32)HZ * (ext_chan_switch->chan_switch_count + 2) * 110 /
			1000);

	pcust_chansw_ie->ie_index = 0xffff; /*Auto index */
	pcust_chansw_ie->mgmt_subtype_mask = 0;
	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Failed to clear ECSA IE\n");
	}
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}
#endif

/**
* @brief               Set random mac configure value (ON/OFF)
*
* @param priv          Pointer to moal_private structure
* @param respbuf       Pointer to response buffer
* @param resplen       Response buffer length

*  @return             Number of bytes written, negative for failure.
*/
static int woal_priv_config_random_mac(moal_private *priv, t_u8 *respbuf,
				       t_u32 respbuflen)
{
	int ret = 0;
	int header_len = 0, space_len = 0, i;
	t_u8 rand_data[3];
	const t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0, 0, 0, 0, 0, 0};
	char fakemac_sts[16];

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	header_len = strlen("FAKEMAC");
	if (strlen(respbuf) >= header_len) {
		for (i = 0; i < (strlen(respbuf) - header_len - 1); i++) {
			if (respbuf[header_len + 1 + i] != ' ')
				break;
		}
		space_len = i;

		if (strncmp(respbuf + header_len + 1 + space_len, "On",
			    strlen("On")) == 0) {
			if (memcmp(priv->random_mac, zero_mac,
				   MLAN_MAC_ADDR_LENGTH)) {
				ret = sprintf(respbuf,
					      "FAKEMAC has been On\n") +
				      1;
				goto done;
			}
			moal_memcpy_ext(priv->phandle, priv->random_mac,
					priv->current_addr, ETH_ALEN,
					MLAN_MAC_ADDR_LENGTH);
			get_random_bytes(rand_data, 3);
			moal_memcpy_ext(priv->phandle, priv->random_mac + 3,
					rand_data, 3, 3);
		} else if (strncmp(respbuf + header_len + 1 + space_len, "Off",
				   strlen("Off")) == 0) {
			memset(priv->random_mac, 0, ETH_ALEN);
		} else {
			PRINTM(MERROR, "Invalid parameter!\n");
			ret = -EINVAL;
			goto done;
		}
	} else {
		PRINTM(MERROR, "Invalid parameter!\n");
		ret = -EINVAL;
		goto done;
	}

	memset(fakemac_sts, 0, sizeof(fakemac_sts));
	snprintf(fakemac_sts, sizeof(fakemac_sts), "%s",
		 (respbuf + header_len + 1 + space_len));
	ret = snprintf(respbuf, respbuflen, "FAKEMAC parameter is %s\n",
		       fakemac_sts) +
	      1;
	PRINTM(MMSG, "FAKEMAC parameter is %s\n",
	       (t_u8 *)(respbuf + header_len + 1 + space_len));

done:
	LEAVE();
	return ret;
}

/**
 * @brief               Download start keep alive parameters
 *
 * @param priv          Pointer to moal_private structure
 * @param mkeep_alive_id       keep alive ID number
 * @param ip_pke        IP packet from host
 * @param ip_pke_len    IP packet length from host
 * @param src_mac       Source MAC address
 * @param dst_mac       Destination MAC address
 * @param period_msec   Send keep alive packet interval

 * @return      0: success  fail otherwise
 */
int woal_start_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id,
			   t_u8 *ip_pkt, t_u16 ip_pkt_len, t_u8 *src_mac,
			   t_u8 *dst_mac, t_u32 period_msec,
			   t_u32 retry_interval, t_u8 retry_cnt)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	int ret = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CLOUD_KEEP_ALIVE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	if (mkeep_alive_id >= MAX_KEEP_ALIVE_ID) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EFAULT;
		goto done;
	}

	/* SET operation */
	ioctl_req->action = MLAN_ACT_SET;
	misc->param.keep_alive.mkeep_alive_id = mkeep_alive_id;
	misc->param.keep_alive.enable = true;
	misc->param.keep_alive.send_interval = period_msec;
	misc->param.keep_alive.retry_interval = retry_interval;
	misc->param.keep_alive.retry_count = retry_cnt;
	moal_memcpy_ext(priv->phandle, misc->param.keep_alive.dst_mac, dst_mac,
			MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	moal_memcpy_ext(priv->phandle, misc->param.keep_alive.src_mac, src_mac,
			MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
	misc->param.keep_alive.pkt_len =
		MIN(ip_pkt_len, MKEEP_ALIVE_IP_PKT_MAX);
	moal_memcpy_ext(priv->phandle, misc->param.keep_alive.packet, ip_pkt,
			ip_pkt_len, MKEEP_ALIVE_IP_PKT_MAX);

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 * @brief               Download stop keep alive parameters
 *
 * @param priv          Pointer to moal_private structure
 * @param mkeep_alive_id       keep alive ID number
 * @param ip_pkt        Last packet
 * @param ip_pkt_len    Last packet length

 * @return      0: success  fail otherwise
 */
int woal_stop_mkeep_alive(moal_private *priv, t_u8 mkeep_alive_id, t_u8 reset,
			  t_u8 *ip_pkt, t_u8 *pkt_len)
{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	mlan_status status = MLAN_STATUS_SUCCESS;
	mlan_ds_misc_keep_alive *misc_keep_alive = NULL;
	int ret = 0;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CLOUD_KEEP_ALIVE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;
	misc_keep_alive = &misc->param.keep_alive;

	if (mkeep_alive_id >= MAX_KEEP_ALIVE_ID) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EFAULT;
		goto done;
	}

	/* GET operation */
	ioctl_req->action = MLAN_ACT_GET;
	misc_keep_alive->mkeep_alive_id = mkeep_alive_id;
	misc_keep_alive->enable = false;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (!misc_keep_alive->enable) {
		PRINTM(MERROR, "ID %d is already stop\n", mkeep_alive_id);
		goto done;
	}

	if (reset)
		ioctl_req->action = MLAN_ACT_RESET;
	else
		/* SET operation */
		ioctl_req->action = MLAN_ACT_SET;
	misc_keep_alive->mkeep_alive_id = mkeep_alive_id;
	misc_keep_alive->enable = false;

	status = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (IS_STA_CFG80211(priv->phandle->params.cfg80211_wext)) {
		ret = woal_mkeep_alive_vendor_event(priv,
						    &misc->param.keep_alive);
		if (ret)
			PRINTM(MERROR,
			       "Keep alive vendor event upload failed\n");
	}
#endif
#endif
	if (pkt_len) {
		*pkt_len = MIN(misc_keep_alive->pkt_len,
			       (MKEEP_ALIVE_IP_PKT_MAX - 1));
		PRINTM(MINFO, "keep alive stop pkt_len is %d\n", *pkt_len);
	}
	if (*pkt_len && ip_pkt)
		moal_memcpy_ext(priv->phandle, ip_pkt, misc_keep_alive->packet,
				*pkt_len, *pkt_len);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 * @brief               Save cloud keep alive params in driver handle
 *
 * @param priv          Pointer to moal_private structure
 * @params              Other params for keep alive

 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_save_cloud_keep_alive_params(moal_private *priv,
					   t_u8 mkeep_alive_id, t_u8 enable,
					   t_u16 ether_type, t_u8 *ip_pkt,
					   t_u16 ip_pkt_len, t_u8 *src_mac,
					   t_u8 *dst_mac, t_u32 period_msec,
					   t_u32 retry_interval, t_u8 retry_cnt)

{
	mlan_ioctl_req *ioctl_req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	mlan_ds_misc_keep_alive *keep_alive = NULL;
	moal_handle *phandle = NULL;

	ENTER();

	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is null\n");
		ret = -EFAULT;
		goto done;
	}
	phandle = priv->phandle;

	ioctl_req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (ioctl_req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	misc = (mlan_ds_misc_cfg *)ioctl_req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CLOUD_KEEP_ALIVE;
	ioctl_req->req_id = MLAN_IOCTL_MISC_CFG;

	if (mkeep_alive_id >= MAX_KEEP_ALIVE_ID) {
		PRINTM(MERROR, "Invalid parameters\n");
		ret = -EINVAL;
		goto done;
	}

	/* GET operation */
	ioctl_req->action = MLAN_ACT_GET;
	misc->param.keep_alive.mkeep_alive_id = mkeep_alive_id;
	misc->param.keep_alive.enable = true;

	ret = woal_request_ioctl(priv, ioctl_req, MOAL_IOCTL_WAIT);
	if (ret != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	if (misc->param.keep_alive.enable) {
		PRINTM(MERROR, "ID %d is in use\n", mkeep_alive_id);
		ret = -EINVAL;
		goto done;
	}

	keep_alive = &phandle->keep_alive[mkeep_alive_id];
	keep_alive->mkeep_alive_id = mkeep_alive_id;
	keep_alive->enable = enable;
	if (enable) {
		keep_alive->cached = true;
		keep_alive->send_interval = period_msec;
		keep_alive->retry_interval = retry_interval;
		keep_alive->retry_count = retry_cnt;
		moal_memcpy_ext(phandle, keep_alive->dst_mac, dst_mac,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		moal_memcpy_ext(phandle, keep_alive->src_mac, src_mac,
				MLAN_MAC_ADDR_LENGTH, MLAN_MAC_ADDR_LENGTH);
		keep_alive->pkt_len = MIN(ip_pkt_len, MKEEP_ALIVE_IP_PKT_MAX);
		moal_memcpy_ext(phandle, keep_alive->packet, ip_pkt, ip_pkt_len,
				MKEEP_ALIVE_IP_PKT_MAX);
		if (ether_type)
			keep_alive->ether_type = ether_type;
		else
			keep_alive->ether_type = 0;
	}

done:
	if (ret != MLAN_STATUS_PENDING)
		kfree(ioctl_req);

	LEAVE();
	return ret;
}

/**
 * @brief               Cloud keep alive feature
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length

 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_cloud_keep_alive(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	int ret = 0;
	cloud_keep_alive *keep_alive = NULL;
	int header_len = 0;

	ENTER();

	header_len = strlen(PRIV_CMD_CLOUD_KEEP_ALIVE);

	keep_alive = (cloud_keep_alive *)(respbuf + header_len);

	if (keep_alive->enable) {
		ret = woal_priv_save_cloud_keep_alive_params(
			priv, keep_alive->mkeep_alive_id, keep_alive->enable, 0,
			keep_alive->pkt, keep_alive->pkt_len,
			keep_alive->src_mac, keep_alive->dst_mac,
			keep_alive->sendInterval, keep_alive->retryInterval,
			keep_alive->retryCount);
	} else {
		if (0 != woal_stop_mkeep_alive(priv, keep_alive->mkeep_alive_id,
					       keep_alive->reset,
					       keep_alive->pkt,
					       &keep_alive->pkt_len)) {
			ret = -EFAULT;
			return ret;
		}
		ret = respbuflen;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get static rx abort config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_rx_abort_cfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[2] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RX_ABORT_CFG);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RX_ABORT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 2 ||
		    (data[0] == MTRUE && user_data_len != 2)) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] == MTRUE && data[1] > 0x7f) {
			PRINTM(MERROR, "Invalid threshold value\n");
			ret = -EINVAL;
			goto done;
		}
		misc->param.rx_abort_cfg.enable = (t_u8)data[0];
		if (user_data_len == 2)
			misc->param.rx_abort_cfg.rssi_threshold = (t_s8)data[1];
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.rx_abort_cfg.enable;
	data[1] = misc->param.rx_abort_cfg.rssi_threshold;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get dynamic rx abort config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_rx_abort_cfg_ext(moal_private *priv, t_u8 *respbuf,
			       t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[3] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RX_ABORT_CFG_EXT);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RX_ABORT_CFG_EXT;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 3 ||
		    (data[0] == MTRUE && user_data_len != 3)) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] == MTRUE) {
			if (data[1] > 0x7f) {
				PRINTM(MERROR, "Invalid margin value\n");
				ret = -EINVAL;
				goto done;
			}
			if (data[2] > 0x7f) {
				PRINTM(MERROR,
				       "Invalid ceil threshold value\n");
				ret = -EINVAL;
				goto done;
			}
		}
		misc->param.rx_abort_cfg_ext.enable = (t_u8)data[0];
		if (user_data_len > 1) {
			misc->param.rx_abort_cfg_ext.rssi_margin =
				(t_s8)data[1];
			misc->param.rx_abort_cfg_ext.ceil_rssi_threshold =
				(t_s8)data[2];
		}
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.rx_abort_cfg_ext.enable;
	data[1] = misc->param.rx_abort_cfg_ext.rssi_margin;
	data[2] = misc->param.rx_abort_cfg_ext.ceil_rssi_threshold;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Enable/Disable Un-associated Dot11mc FTM Frame exchanges
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_dot11mc_unassoc_ftm_cfg(moal_private *priv, t_u8 *respbuf,
				      t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[1] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_DOT11MC_UNASSOC_FTM_CFG);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_DOT11MC_UNASSOC_FTM_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 1) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[0] != MTRUE) && (data[0] != MFALSE)) {
			PRINTM(MERROR, "Invalid state for unassoc ftm\n");
			ret = -EINVAL;
			goto done;
		}
		misc->param.dot11mc_unassoc_ftm_cfg.state = (t_u16)data[0];
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.dot11mc_unassoc_ftm_cfg.state;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Tx  AMPDU protection mode
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_tx_ampdu_prot_mode(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[1] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TX_AMPDU_PROT_MODE);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TX_AMPDU_PROT_MODE;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 1) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] > TX_AMPDU_DYNAMIC_RTS_CTS) {
			PRINTM(MERROR, "Invalid protection mode\n");
			ret = -EINVAL;
			goto done;
		}
		misc->param.tx_ampdu_prot_mode.mode = (t_u16)data[0];
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.tx_ampdu_prot_mode.mode;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get Tx rate adapt config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_rate_adapt_cfg(moal_private *priv, t_u8 *respbuf,
			     t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[4] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_RATE_ADAPT_CFG);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_RATE_ADAPT_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len < 1) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] > RATEADAPT_ALGO_SR) {
			PRINTM(MERROR, "Invalid Rateadapt Algorithm\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] == RATEADAPT_ALGO_SR && user_data_len > 1) {
			if ((data[1] & data[2]) == 0xff) {
				/* dynamic CCA noise based rate adapation enable
				 * request nothing to do here
				 */
			} else if (data[1] > 100 || data[2] > 100) {
				PRINTM(MERROR,
				       "Invalid success rate threshold value\n");
				ret = -EINVAL;
				goto done;
			}
			if (data[3] < 10 || data[3] > 0xffff) {
				PRINTM(MERROR, "Invalid interval value\n");
				ret = -EINVAL;
				goto done;
			}
		}
		misc->param.rate_adapt_cfg.sr_rateadapt = (t_u8)data[0];
		if (data[0] == RATEADAPT_ALGO_SR && user_data_len > 1) {
			misc->param.rate_adapt_cfg.ra_low_thresh =
				(t_u8)data[1];
			misc->param.rate_adapt_cfg.ra_high_thresh =
				(t_u8)data[2];
			misc->param.rate_adapt_cfg.ra_interval = (t_u16)data[3];
		}
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}
	data[0] = misc->param.rate_adapt_cfg.sr_rateadapt;
	data[1] = misc->param.rate_adapt_cfg.ra_low_thresh;
	data[2] = misc->param.rate_adapt_cfg.ra_high_thresh;
	data[3] = misc->param.rate_adapt_cfg.ra_interval;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get global cck desense config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_cck_desense_cfg(moal_private *priv, t_u8 *respbuf,
			      t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[5] = {0};
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_CCK_DESENSE_CFG);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_CCK_DESENSE_CFG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len > 5) {
			PRINTM(MERROR, "Invalid number of args!\n");
			ret = -EINVAL;
			goto done;
		}
		if (data[0] > CCK_DESENSE_MODE_DYN_ENH) {
			PRINTM(MERROR, "Invalid cck desense mode\n");
			ret = -EINVAL;
			goto done;
		}
		if ((data[0] == CCK_DESENSE_MODE_DISABLED &&
		     user_data_len > 1) ||
		    (data[0] == CCK_DESENSE_MODE_DYNAMIC &&
		     user_data_len != 3) ||
		    (data[0] == CCK_DESENSE_MODE_DYN_ENH &&
		     (user_data_len < 3 || user_data_len == 4))) {
			PRINTM(MERROR,
			       "Invalid number of args for requested mode\n");
			ret = -EINVAL;
			goto done;
		}
		if (user_data_len > 1) {
			if (data[1] > 0x7f) {
				PRINTM(MERROR, "Invalid margin value\n");
				ret = -EINVAL;
				goto done;
			}
			if (data[2] > 0x7f) {
				PRINTM(MERROR,
				       "Invalid ceil threshold value\n");
				ret = -EINVAL;
				goto done;
			}
		}
		if (user_data_len > 3) {
			if (data[3] > 0xff || data[4] > 0xff) {
				PRINTM(MERROR,
				       "Invalid ON/OFF intervals value\n");
				ret = -EINVAL;
				goto done;
			}
		}
		misc->param.cck_desense_cfg.mode = (t_u8)data[0];
		if (user_data_len > 1) {
			misc->param.cck_desense_cfg.margin = (t_s8)data[1];
			misc->param.cck_desense_cfg.ceil_thresh = (t_s8)data[2];
		}
		if (data[0] == CCK_DESENSE_MODE_DYN_ENH) {
			if (user_data_len > 3) {
				misc->param.cck_desense_cfg.num_on_intervals =
					(t_u8)data[3];
				misc->param.cck_desense_cfg.num_off_intervals =
					(t_u8)data[4];
			} else {
				/* set these to 0xff. This will indicate the FW
				 * to use previously set values.
				 */
				misc->param.cck_desense_cfg.num_on_intervals =
					0xff;
				misc->param.cck_desense_cfg.num_off_intervals =
					0xff;
			}
		}
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.cck_desense_cfg.mode;
	data[1] = misc->param.cck_desense_cfg.margin;
	data[2] = misc->param.cck_desense_cfg.ceil_thresh;
	data[3] = misc->param.cck_desense_cfg.num_on_intervals;
	data[4] = misc->param.cck_desense_cfg.num_off_intervals;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 * @brief               set/get low power mode
 *
 * @param priv          Pointer to moal_private structure
 * @param respbuf       Pointer to response buffer
 * @param resplen       Response buffer length

 *  @return             Number of bytes written, negative for failure.
 */
static int woal_priv_set_get_lpm(moal_private *priv, t_u8 *respbuf,
				 t_u32 respbuflen)
{
	int ret = 0;
	mlan_ioctl_req *req = NULL;
	mlan_ds_power_cfg *cfg = NULL;
	int data = 0;
	int user_data_len = 0, header_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (IS_CARD9098(priv->phandle->card_type) ||
	    IS_CARD9097(priv->phandle->card_type)) {
		header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_LPM);
		if (strlen(respbuf) == header_len) {
			/* GET operation */
			user_data_len = 0;
		} else {
			/* SET operation */
			parse_arguments(respbuf + header_len, &data, 1,
					&user_data_len);
		}
		if (user_data_len >= 2) {
			PRINTM(MERROR, "Too many arguments\n");
			ret = -EINVAL;
			goto done;
		} else {
			req = woal_alloc_mlan_ioctl_req(
				sizeof(mlan_ds_power_cfg));
			if (req == NULL) {
				ret = -ENOMEM;
				goto done;
			}

			cfg = (mlan_ds_power_cfg *)req->pbuf;
			if (user_data_len == 0) {
				req->action = MLAN_ACT_GET;
			} else {
				cfg->param.lpm = data;
				req->action = MLAN_ACT_SET;
			}
		}
		cfg->sub_command = MLAN_OID_POWER_LOW_POWER_MODE;
		req->req_id = MLAN_IOCTL_POWER_CFG;

		status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
		if (status != MLAN_STATUS_SUCCESS) {
			ret = -EFAULT;
			goto done;
		}

		data = cfg->param.lpm;
		moal_memcpy_ext(priv->phandle, respbuf, (t_u8 *)&data,
				sizeof(data), respbuflen);
		ret = sizeof(data);
	} else
		PRINTM(MERROR, "Low power mode command is not supported!\n");

done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	LEAVE();
	return ret;
}

/**
 *  @brief Set/Get HW ARB config
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_arbcfg(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	int ret = 0;
	int data[1];
	int header_len = 0, user_data_len = 0;
	mlan_status status = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_ARB_CFG);
	user_data_len = strlen(respbuf) - header_len;

	/* Allocate an IOCTL request buffer */
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_ARB_CONFIG;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	if (strlen(respbuf) == header_len) {
		/* GET operation */
		user_data_len = 0;
		req->action = MLAN_ACT_GET;
	} else {
		/* SET operation */
		parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
				&user_data_len);
		if (user_data_len != 1) {
			PRINTM(MERROR, "Invalid Parameter\n");
			ret = -EFAULT;
			goto done;
		}
		if (data[0] < 0 || data[0] > 4) {
			PRINTM(MERROR, "Invalid Parameter: arb mode 0-4\n");
			ret = -EFAULT;
			goto done;
		}
		misc->param.arb_cfg.arb_mode = (t_u32)data[0];
		req->action = MLAN_ACT_SET;
	}
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_SUCCESS) {
		ret = -EFAULT;
		goto done;
	}

	data[0] = misc->param.arb_cfg.arb_mode;
	moal_memcpy_ext(priv->phandle, respbuf, (t_u32 *)data, sizeof(data),
			respbuflen);
	ret = sizeof(data);
done:
	if (status != MLAN_STATUS_PENDING)
		kfree(req);

	LEAVE();
	return ret;
}

/**
 *  @brief      Timer function for TP state command.
 *
 *  @param data pointer to a buffer
 *
 *  @return     N/A
 */
void woal_tp_acnt_timer_func(void *context)
{
	moal_handle *phandle = (moal_handle *)context;
	int i = 0;

	if (phandle == NULL)
		return;
	PRINTM(MDATA, "####### CPU%d: tp acnt timer\n", smp_processor_id());
	/* Tx TP accounting */
	for (i = 0; i < MAX_TP_ACCOUNT_DROP_POINT_NUM; i++) {
		phandle->tp_acnt.tx_bytes_rate[i] =
			phandle->tp_acnt.tx_bytes[i] -
			phandle->tp_acnt.tx_bytes_last[i];
		phandle->tp_acnt.tx_bytes_last[i] =
			phandle->tp_acnt.tx_bytes[i];
	}
	phandle->tp_acnt.tx_pending = atomic_read(&phandle->tx_pending);
	/* Tx Interrupt accounting */
	phandle->tp_acnt.tx_intr_rate =
		phandle->tp_acnt.tx_intr_cnt - phandle->tp_acnt.tx_intr_last;
	phandle->tp_acnt.tx_intr_last = phandle->tp_acnt.tx_intr_cnt;

	/* Rx TP accounting */
	for (i = 0; i < MAX_TP_ACCOUNT_DROP_POINT_NUM; i++) {
		phandle->tp_acnt.rx_bytes_rate[i] =
			phandle->tp_acnt.rx_bytes[i] -
			phandle->tp_acnt.rx_bytes_last[i];
		phandle->tp_acnt.rx_bytes_last[i] =
			phandle->tp_acnt.rx_bytes[i];
	}
	phandle->tp_acnt.rx_pending = atomic_read(&phandle->rx_pending);
	// Interrupt accounting, RX
	phandle->tp_acnt.rx_intr_rate =
		phandle->tp_acnt.rx_intr_cnt - phandle->tp_acnt.rx_intr_last;
	phandle->tp_acnt.rx_intr_last = phandle->tp_acnt.rx_intr_cnt;

	/* re-arm timer */
	woal_mod_timer(&phandle->tp_acnt.timer, 1000);
}

/**
 *  @brief      set tp state to mlan
 *
 *  @param priv  pointer to moal_private
 *
 *  @return     N/A
 */
void woal_set_tp_state(moal_private *priv)
{
	mlan_ioctl_req *req = NULL;
	mlan_ds_misc_cfg *misc = NULL;
	moal_handle *handle = priv->phandle;
	mlan_status status = MLAN_STATUS_SUCCESS;
	req = woal_alloc_mlan_ioctl_req(sizeof(mlan_ds_misc_cfg));
	if (req == NULL)
		return;
	/* Fill request buffer */
	misc = (mlan_ds_misc_cfg *)req->pbuf;
	misc->sub_command = MLAN_OID_MISC_TP_STATE;
	req->req_id = MLAN_IOCTL_MISC_CFG;
	misc->param.tp_state.on = handle->tp_acnt.on;
	misc->param.tp_state.drop_point = handle->tp_acnt.drop_point;
	req->action = MLAN_ACT_SET;
	/* Send IOCTL request to MLAN */
	status = woal_request_ioctl(priv, req, MOAL_IOCTL_WAIT);
	if (status != MLAN_STATUS_PENDING)
		kfree(req);
	return;
}

/**
 *  @brief Set/Get TP statistics.
 *
 *  @param priv         A pointer to moal_private structure
 *  @param respbuf      A pointer to response buffer
 *  @param respbuflen   Available length of response buffer
 *
 *  @return             Number of bytes written, negative for failure.
 */
int woal_priv_set_tp_state(moal_private *priv, t_u8 *respbuf, t_u32 respbuflen)
{
	moal_handle *handle = priv->phandle;
	int ret = 0;
	int data[2];
	int header_len = 0, user_data_len = 0;

	ENTER();

	if (!respbuf) {
		PRINTM(MERROR, "response buffer is not available!\n");
		ret = -EINVAL;
		goto done;
	}
	header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_TP_STATE);
	user_data_len = strlen(respbuf) - header_len;
	parse_arguments(respbuf + header_len, data, ARRAY_SIZE(data),
			&user_data_len);
	if (user_data_len > 2) {
		PRINTM(MERROR, "Invalid number of args!\n");
		ret = -EINVAL;
		goto done;
	}
	if (user_data_len) {
		handle->tp_acnt.on = data[0];
		/* Enable TP statistics collection */
		if (data[0] == 1) {
			handle->tp_acnt.drop_point = data[1];
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
		woal_set_tp_state(priv);
	}
	/* Get command results */
	if (user_data_len == 0) {
		moal_memcpy_ext(handle, respbuf, (t_u8 *)(&handle->tp_acnt),
				sizeof(handle->tp_acnt), respbuflen);
		ret = sizeof(handle->tp_acnt);
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief Set priv command for Android
 *  @param dev          A pointer to net_device structure
 *  @param req          A pointer to ifreq structure
 *
 *  @return             0 --success, otherwise fail
 */
int woal_android_priv_cmd(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	android_wifi_priv_cmd priv_cmd;
	moal_private *priv = (moal_private *)netdev_priv(dev);
	char *buf = NULL;
	char *pdata;
#ifdef STA_SUPPORT
	int power_mode = 0;
	int band = 0;
	char *pband = NULL;
	mlan_bss_info bss_info;
	mlan_ds_get_signal signal;
	mlan_rate_cfg_t rate;
	t_u8 country_code[COUNTRY_CODE_LEN];
	int copy_len = 0;
#endif
	int len = 0;
	gfp_t flag;
	char *cmd_buf = NULL;
	int cfg80211_wext;

	ENTER();
	if (!priv || !priv->phandle) {
		PRINTM(MERROR, "priv or handle is NULL\n");
		ret = -EFAULT;
		goto done;
	}
	cfg80211_wext = priv->phandle->params.cfg80211_wext;
	if (copy_from_user(&priv_cmd, req->ifr_data,
			   sizeof(android_wifi_priv_cmd))) {
		ret = -EFAULT;
		goto done;
	}
#define CMD_BUF_LEN 3072
	if (priv_cmd.used_len < 0 || priv_cmd.total_len <= 0 ||
	    priv_cmd.used_len > priv_cmd.total_len) {
		PRINTM(MERROR,
		       "Invalid Android priv cmd len. used_len: %d, total_len: %d\n",
		       priv_cmd.used_len, priv_cmd.total_len);
		ret = -EINVAL;
		goto done;
	}
	if (priv_cmd.total_len + 1 > CMD_BUF_LEN)
		priv_cmd.total_len = CMD_BUF_LEN - 1;

	flag = (in_atomic() || irqs_disabled()) ? GFP_ATOMIC : GFP_KERNEL;
	buf = kzalloc(CMD_BUF_LEN, flag);
	if (!buf) {
		PRINTM(MERROR, "%s: failed to allocate memory\n", __FUNCTION__);
		ret = -ENOMEM;
		goto done;
	}
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	moal_memcpy_ext(priv->phandle, &cmd_buf, &priv_cmd.buf, sizeof(cmd_buf),
			sizeof(cmd_buf));
#else
	cmd_buf = priv_cmd.buf;
#endif
	if (copy_from_user(buf, cmd_buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto done;
	}
	buf[CMD_BUF_LEN - 1] = '\0';

	PRINTM(MIOCTL, "Android priv cmd: [%s] on [%s]\n", buf, req->ifr_name);

	if (strncmp(buf, CMD_NXP, strlen(CMD_NXP)) &&
	    woal_check_driver_status(priv->phandle)) {
		PRINTM(MERROR, "%s fail when driver hang\n", buf);
		ret = -EFAULT;
		goto done;
	}

	if (strncmp(buf, CMD_NXP, strlen(CMD_NXP)) == 0) {
		/* This command has come from mlanutl app */

		/* Check command */
		if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_VERSION,
			     strlen(PRIV_CMD_VERSION)) == 0) {
			/* Get version */
			len = woal_get_priv_driver_version(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_BANDCFG,
				    strlen(PRIV_CMD_BANDCFG)) == 0) {
			/* Set/Get band configuration */
			len = woal_setget_priv_bandcfg(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HOSTCMD,
				    strlen(PRIV_CMD_HOSTCMD)) == 0) {
			/* hostcmd configuration */
			len = woal_priv_hostcmd(priv, buf, priv_cmd.total_len,
						MOAL_IOCTL_WAIT);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_11AXCMDCFG,
				    strlen(PRIV_CMD_11AXCMDCFG)) == 0) {
			/* 11ax command */
			pdata = buf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_11AXCMDCFG);
			len = priv_cmd.total_len - strlen(CMD_NXP) +
			      strlen(PRIV_CMD_11AXCMDCFG);
			len = woal_setget_priv_11axcmdcfg(priv, pdata, len,
							  MOAL_IOCTL_WAIT);
			len += strlen(CMD_NXP) + strlen(PRIV_CMD_11AXCMDCFG);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_RANGE_EXT,
				    strlen(PRIV_CMD_RANGE_EXT)) == 0) {
			len = woal_setget_priv_range_ext(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HTTXCFG,
				    strlen(PRIV_CMD_HTTXCFG)) == 0) {
			/* Set/Get HT Tx configuration */
			len = woal_setget_priv_httxcfg(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HTCAPINFO,
				    strlen(PRIV_CMD_HTCAPINFO)) == 0) {
			/* Set/Get HT Capability information */
			len = woal_setget_priv_htcapinfo(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ADDBAPARA,
				    strlen(PRIV_CMD_ADDBAPARA)) == 0) {
			/* Set/Get Add BA parameters */
			len = woal_setget_priv_addbapara(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_AGGRPRIOTBL,
				    strlen(PRIV_CMD_AGGRPRIOTBL)) == 0) {
			/* Set/Get Aggregation priority table parameters */
			len = woal_setget_priv_aggrpriotbl(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ADDBAREJECT,
				    strlen(PRIV_CMD_ADDBAREJECT)) == 0) {
			/* Set/Get Add BA reject parameters */
			len = woal_setget_priv_addbareject(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DELBA,
				    strlen(PRIV_CMD_DELBA)) == 0) {
			/* Delete selective BA based on parameters */
			len = woal_priv_delba(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_REJECTADDBAREQ,
				    strlen(PRIV_CMD_REJECTADDBAREQ)) == 0) {
			/* Set/Get the reject addba requst conditions*/
			len = woal_priv_rejectaddbareq(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_VHTCFG,
				    strlen(PRIV_CMD_VHTCFG)) == 0) {
			/* Set/Get 11AC configuration */
			len = woal_setget_priv_vhtcfg(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_OPERMODECFG,
				    strlen(PRIV_CMD_OPERMODECFG)) == 0) {
			/* Set/Get 11AC configuration */
			len = woal_setget_priv_opermodecfg(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DATARATE,
				    strlen(PRIV_CMD_DATARATE)) == 0) {
			/* Get data rate */
			len = woal_get_priv_datarate(priv, buf,
						     priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TXRATECFG,
				    strlen(PRIV_CMD_TXRATECFG)) == 0) {
			/* Set/Get tx rate cfg */
			len = woal_setget_priv_txratecfg(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#if defined(STA_SUPPORT) || defined(UAP_SUPPORT)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GETLOG,
				    strlen(PRIV_CMD_GETLOG)) == 0) {
			/* Get wireless stats information */
			len = woal_get_priv_getlog(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CUSTOMIE,
				    strlen(PRIV_CMD_CUSTOMIE)) == 0) {
			/* Custom IE configuration */
			len = woal_priv_customie(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ESUPPMODE,
				    strlen(PRIV_CMD_ESUPPMODE)) == 0) {
			/* Esupplicant mode configuration */
			len = woal_setget_priv_esuppmode(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PASSPHRASE,
				    strlen(PRIV_CMD_PASSPHRASE)) == 0) {
			/* Esupplicant passphrase configuration */
			len = woal_setget_priv_passphrase(priv, buf,
							  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DEAUTH,
				    strlen(PRIV_CMD_DEAUTH)) == 0) {
			/* Deauth */
			len = woal_priv_deauth(priv, buf, priv_cmd.total_len);
			goto handled;
#ifdef UAP_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_AP_DEAUTH,
				    strlen(PRIV_CMD_AP_DEAUTH)) == 0) {
			/* AP Deauth */
			len = woal_priv_ap_deauth(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GET_STA_LIST,
				    strlen(PRIV_CMD_GET_STA_LIST)) == 0) {
			/* Get STA list */
			len = woal_priv_get_sta_list(priv, buf,
						     priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_BSS_CONFIG,
				    strlen(PRIV_CMD_BSS_CONFIG)) == 0) {
			/* BSS config */
			len = woal_priv_bss_config(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#endif
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_BSSROLE,
				    strlen(PRIV_CMD_BSSROLE)) == 0) {
			/* BSS Role */
			len = woal_priv_bssrole(priv, buf,
						(t_u32)priv_cmd.total_len);
			goto handled;
#endif
#endif
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SETUSERSCAN,
				    strlen(PRIV_CMD_SETUSERSCAN)) == 0) {
			/* Set user scan */
			len = woal_priv_setuserscan(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GETSCANTABLE,
				    strlen(PRIV_CMD_GETSCANTABLE)) == 0) {
			/* Get scan table */
			len = woal_priv_getscantable(priv, buf,
						     priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GETCHANSTATS,
				    strlen(PRIV_CMD_GETCHANSTATS)) == 0) {
			/* Get channel statistics */
			len = woal_priv_get_chanstats(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_EXTCAPCFG,
				    strlen(PRIV_CMD_EXTCAPCFG)) == 0) {
			/* Extended capabilities configure */
			len = woal_priv_extcapcfg(priv, buf,
						  (t_u32)priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CANCELSCAN,
				    strlen(PRIV_CMD_CANCELSCAN)) == 0) {
			/* Cancel scan */
			len = woal_cancel_scan(priv, MOAL_IOCTL_WAIT);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DEEPSLEEP,
				    strlen(PRIV_CMD_DEEPSLEEP)) == 0) {
			/* Deep sleep */
			len = woal_priv_setgetdeepsleep(priv, buf,
							priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_IPADDR,
				    strlen(PRIV_CMD_IPADDR)) == 0) {
			/* IP address */
			len = woal_priv_setgetipaddr(priv, buf,
						     (t_u32)priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_WPSSESSION,
				    strlen(PRIV_CMD_WPSSESSION)) == 0) {
			/* WPS Session */
			len = woal_priv_setwpssession(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_OTPUSERDATA,
				    strlen(PRIV_CMD_OTPUSERDATA)) == 0) {
			/* OTP user data */
			len = woal_priv_otpuserdata(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_COUNTRYCODE,
				    strlen(PRIV_CMD_COUNTRYCODE)) == 0) {
			/* Country code */
			len = woal_priv_set_get_countrycode(priv, buf,
							    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CFPINFO,
				    strlen(PRIV_CMD_CFPINFO)) == 0) {
			/* CFP info */
			len = woal_priv_get_cfpinfo(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TCPACKENH,
				    strlen(PRIV_CMD_TCPACKENH)) == 0) {
			/* TCP ack enhancement */
			len = woal_priv_setgettcpackenh(priv, buf,
							priv_cmd.total_len);
			goto handled;
#ifdef REASSOCIATION
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ASSOCBSSID,
				    strlen(PRIV_CMD_ASSOCBSSID)) == 0) {
			/* Associate to essid */
			len = woal_priv_assocessid(priv, buf,
						   priv_cmd.total_len, 1);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ASSOCESSID,
				    strlen(PRIV_CMD_ASSOCESSID)) == 0) {
			/* Associate to essid */
			len = woal_priv_assocessid(priv, buf,
						   priv_cmd.total_len, 0);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_WAKEUPREASON,
				    strlen(PRIV_CMD_WAKEUPREASON)) == 0) {
			/* wakeup reason */
			len = woal_priv_getwakeupreason(priv, buf,
							priv_cmd.total_len);
			goto handled;
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_LISTENINTERVAL,
				    strlen(PRIV_CMD_LISTENINTERVAL)) == 0) {
			/* Listen Interval */
			len = woal_priv_set_get_listeninterval(
				priv, buf, priv_cmd.total_len);
			goto handled;
#endif
#ifdef DEBUG_LEVEL1
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DRVDBG,
				    strlen(PRIV_CMD_DRVDBG)) == 0) {
			/* Driver debug bit mask */
			len = woal_priv_set_get_drvdbg(priv, buf,
						       priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HSCFG,
				    strlen(PRIV_CMD_HSCFG)) == 0) {
			/* HS configuration */
			len = woal_priv_hscfg(priv, buf, priv_cmd.total_len,
					      MTRUE);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HSSETPARA,
				    strlen(PRIV_CMD_HSSETPARA)) == 0) {
			/* Set HS parameter */
			len = woal_priv_hssetpara(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_MGMT_FILTER,
				    strlen(PRIV_CMD_MGMT_FILTER)) == 0) {
			/* Management frame filter wakeup */
			len = woal_priv_mgmt_filter(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SCANCFG,
				    strlen(PRIV_CMD_SCANCFG)) == 0) {
			/* Scan configuration */
			len = woal_priv_set_get_scancfg(priv, buf,
							priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GETNLNUM,
				    strlen(PRIV_CMD_GETNLNUM)) == 0) {
			/* Scan configuration */
			len = woal_priv_getnlnum(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_AGGRCTRL,
				    strlen(PRIV_CMD_AGGRCTRL)) == 0) {
			/* aggregation control */
			len = woal_priv_set_get_aggrctrl(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#ifdef USB
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_USBAGGRCTRL,
				    strlen(PRIV_CMD_USBAGGRCTRL)) == 0) {
			/* USB aggregation control */
			len = woal_priv_set_get_usbaggrctrl(priv, buf,
							    priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_SET_BSS_MODE,
				    strlen(PRIV_CMD_SET_BSS_MODE)) == 0) {
			/* Set bss mode */
			len = woal_priv_set_bss_mode(priv, buf,
						     priv_cmd.total_len);
			goto handled;
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SET_AP,
				    strlen(PRIV_CMD_SET_AP)) == 0) {
			/* Set AP */
			len = woal_priv_set_ap(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SET_POWER,
				    strlen(PRIV_CMD_SET_POWER)) == 0) {
			/* Set power management parameters */
			len = woal_priv_set_power(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SET_ESSID,
				    strlen(PRIV_CMD_SET_ESSID)) == 0) {
			/* Set essid */
			len = woal_priv_set_essid(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SET_AUTH,
				    strlen(PRIV_CMD_SET_AUTH)) == 0) {
			/* Set authentication mode parameters */
			len = woal_priv_set_auth(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GET_AP,
				    strlen(PRIV_CMD_GET_AP)) == 0) {
			/* Get AP */
			len = woal_priv_get_ap(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GET_POWER,
				    strlen(PRIV_CMD_GET_POWER)) == 0) {
			/* Get power management parameters */
			len = woal_priv_get_power(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PSMODE,
				    strlen(PRIV_CMD_PSMODE)) == 0) {
			/* Set/Get PS mode */
			len = woal_priv_set_get_psmode(priv, buf,
						       priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_WARMRESET,
				    strlen(PRIV_CMD_WARMRESET)) == 0) {
			/* Performs warm reset */
			len = woal_priv_warmreset(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TXPOWERCFG,
				    strlen(PRIV_CMD_TXPOWERCFG)) == 0) {
			/* TX power configurations */
			len = woal_priv_txpowercfg(priv, buf,
						   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_RX_ABORT_CFG_EXT,
				    strlen(PRIV_CMD_RX_ABORT_CFG_EXT)) == 0) {
			/* dynamic Rx Abort config */
			len = woal_priv_rx_abort_cfg_ext(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_RX_ABORT_CFG,
				    strlen(PRIV_CMD_RX_ABORT_CFG)) == 0) {
			/* static Rx Abort config */
			len = woal_priv_rx_abort_cfg(priv, buf,
						     priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_TX_AMPDU_PROT_MODE,
				    strlen(PRIV_CMD_TX_AMPDU_PROT_MODE)) == 0) {
			/* tx ampdu protection mode setting */
			len = woal_priv_tx_ampdu_prot_mode(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_DOT11MC_UNASSOC_FTM_CFG,
				    strlen(PRIV_CMD_DOT11MC_UNASSOC_FTM_CFG)) ==
			   0) {
			/* setting for dot11mc un-associated case FTM frame
			 * exchange */
			len = woal_priv_dot11mc_unassoc_ftm_cfg(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_RATE_ADAPT_CFG,
				    strlen(PRIV_CMD_RATE_ADAPT_CFG)) == 0) {
			/* rate adapt config */
			len = woal_priv_rate_adapt_cfg(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_CCK_DESENSE_CFG,
				    strlen(PRIV_CMD_CCK_DESENSE_CFG)) == 0) {
			/* cck desense config */
			len = woal_priv_cck_desense_cfg(priv, buf,
							priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PSCFG,
				    strlen(PRIV_CMD_PSCFG)) == 0) {
			/* PS configurations */
			len = woal_priv_pscfg(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_BCNTIMEOUTCFG,
				    strlen(PRIV_CMD_BCNTIMEOUTCFG)) == 0) {
			/* Beacon timeout configurations */
			len = woal_priv_bcntimeoutcfg(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SLEEPPD,
				    strlen(PRIV_CMD_SLEEPPD)) == 0) {
			/* Sleep period */
			len = woal_priv_sleeppd(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TXCONTROL,
				    strlen(PRIV_CMD_TXCONTROL)) == 0) {
			/* Tx control */
			len = woal_priv_txcontrol(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_REGRDWR,
				    strlen(PRIV_CMD_REGRDWR)) == 0) {
			/* Register Read/Write */
			len = woal_priv_regrdwr(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_RDEEPROM,
				    strlen(PRIV_CMD_RDEEPROM)) == 0) {
			/* Read the EEPROM contents of the card */
			len = woal_priv_rdeeprom(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_MEMRDWR,
				    strlen(PRIV_CMD_MEMRDWR)) == 0) {
			/* Memory Read/Write */
			len = woal_priv_memrdwr(priv, buf, priv_cmd.total_len);
			goto handled;
#ifdef SDIO
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SDCMD52RW,
				    strlen(PRIV_CMD_SDCMD52RW)) == 0) {
			/* Cmd52 read/write register */
			len = woal_priv_sdcmd52rw(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SDIO_CLOCK,
				    strlen(PRIV_CMD_SDIO_CLOCK)) == 0) {
			/* Turn on/off the sdio clock */
			len = woal_priv_sdio_clock_ioctl(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_MPA_CTRL,
				    strlen(PRIV_CMD_MPA_CTRL)) == 0) {
			/* Set SDIO Multi-point aggregation
			 * control parameters */
			len = woal_priv_sdio_mpa_ctrl(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SD_CMD53_RW,
				    strlen(PRIV_CMD_SD_CMD53_RW)) == 0) {
			/* Cmd53 read/write register */
			len = woal_priv_cmd53rdwr(priv, buf,
						  priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ROBUSTCOEX,
				    strlen(PRIV_CMD_ROBUSTCOEX)) == 0) {
			/* Set Robustcoex GPIOcfg */
			pdata = buf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_ROBUSTCOEX);
			len = priv_cmd.total_len - strlen(PRIV_CMD_ROBUSTCOEX) -
			      strlen(CMD_NXP);
			len = woal_priv_robustcoex(priv, pdata, len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DMCS,
				    strlen(PRIV_CMD_DMCS)) == 0) {
			/* Set/Get DMCS config */
			len = woal_priv_dmcs(priv, buf, priv_cmd.total_len);
			goto handled;
#if defined(PCIE)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SSU,
				    strlen(PRIV_CMD_SSU)) == 0) {
			/* Set SSU config */
			pdata = buf + strlen(CMD_NXP) + strlen(PRIV_CMD_SSU);
			len = priv_cmd.used_len - strlen(PRIV_CMD_SSU) -
			      strlen(CMD_NXP);
			len = woal_priv_ssu_cmd(priv, len, pdata,
						priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_AUTO_ARP,
				    strlen(PRIV_CMD_AUTO_ARP)) == 0) {
			/* Auto ARP enable/disable */
			len = woal_priv_set_get_auto_arp(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_HOTSPOTCFG,
				    strlen(PRIV_CMD_HOTSPOTCFG)) == 0) {
			/* Hotspot CFG */
			len = woal_priv_hotspotcfg(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#ifdef RX_PACKET_COALESCE
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_RX_COAL_CFG,
				    strlen(PRIV_CMD_RX_COAL_CFG)) == 0) {
			/* RX packet coalescing Configuration */
			len = woal_priv_rx_pkt_coalesce_cfg(priv, buf,
							    priv_cmd.total_len);
			goto handled;
#endif

		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_MGMT_FRAME_CTRL,
				    strlen(PRIV_CMD_MGMT_FRAME_CTRL)) == 0) {
			/* Mgmt Frame Passthrough Ctrl */
			len = woal_priv_mgmt_frame_passthru_ctrl(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_QCONFIG,
				    strlen(PRIV_CMD_QCONFIG)) == 0) {
			/* Queue config */
			len = woal_priv_qconfig(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ADDTS,
				    strlen(PRIV_CMD_ADDTS)) == 0) {
			/* Send an ADDTS TSPEC */
			len = woal_priv_wmm_addts_req_ioctl(priv, buf,
							    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DELTS,
				    strlen(PRIV_CMD_DELTS)) == 0) {
			/* Send a DELTS TSPE */
			len = woal_priv_wmm_delts_req_ioctl(priv, buf,
							    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_QSTATUS,
				    strlen(PRIV_CMD_QSTATUS)) == 0) {
			/* Get the status of the WMM queues */
			len = woal_priv_wmm_queue_status_ioctl(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TS_STATUS,
				    strlen(PRIV_CMD_TS_STATUS)) == 0) {
			/* Get the status of the WMM Traffic Streams */
			len = woal_priv_wmm_ts_status_ioctl(priv, buf,
							    priv_cmd.total_len);
			goto handled;
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_QOS_CFG,
				    strlen(PRIV_CMD_QOS_CFG)) == 0) {
			t_u32 action = MLAN_ACT_GET;
			if (strlen(buf) ==
			    strlen(CMD_NXP) + strlen(PRIV_CMD_QOS_CFG)) {
				pdata = buf; /* GET operation */
			} else {
				pdata = buf + strlen(CMD_NXP) +
					strlen(PRIV_CMD_QOS_CFG);
				action = MLAN_ACT_SET; /* SET operation */
			}
			if (MLAN_STATUS_SUCCESS !=
			    woal_priv_qos_cfg(priv, action, pdata)) {
				ret = -EFAULT;
				goto done;
			}
			if (action == MLAN_ACT_GET)
				len = sizeof(t_u8);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_MAC_CTRL,
				    strlen(PRIV_CMD_MAC_CTRL)) == 0) {
			/* MAC CTRL */
			len = woal_priv_macctrl(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GETWAP,
				    strlen(PRIV_CMD_GETWAP)) == 0) {
			/* Get WAP */
			len = woal_priv_getwap(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_REGION_CODE,
				    strlen(PRIV_CMD_REGION_CODE)) == 0) {
			/* Region Code */
			len = woal_priv_region_code(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_FWMACADDR,
				    strlen(PRIV_CMD_FWMACADDR)) == 0) {
			/* Set FW MAC address */
			len = woal_priv_fwmacaddr(priv, buf,
						  priv_cmd.total_len);
			goto handled;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_OFFCHANNEL,
				    strlen(PRIV_CMD_OFFCHANNEL)) == 0) {
			if (IS_STA_CFG80211(cfg80211_wext)) {
				/* Set offchannel */
				len = woal_priv_offchannel(priv, buf,
							   priv_cmd.total_len);
			} else
				len = sprintf(buf,
					      "CFG80211 is not enabled\n") +
				      1;
			goto handled;
#endif
#endif

		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DSCP_MAP,
				    strlen(PRIV_CMD_DSCP_MAP)) == 0) {
			/* Set/Get DSCP Map */
			len = woal_priv_set_get_dscp_map(priv, buf,
							 priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_VEREXT,
				    strlen(PRIV_CMD_VEREXT)) == 0) {
			/* Get Extended version */
			len = woal_priv_get_driver_verext(priv, buf,
							  priv_cmd.total_len);
			goto handled;
#ifdef USB
#ifdef CONFIG_USB_SUSPEND
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_USB_SUSPEND,
				    strlen(PRIV_CMD_USB_SUSPEND)) == 0) {
			/* Makes USB device to suspend */
			len = woal_priv_enter_usb_suspend(priv, buf,
							  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_USB_RESUME,
				    strlen(PRIV_CMD_USB_RESUME)) == 0) {
			/* Makes USB device to resume */
			len = woal_priv_exit_usb_suspend(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#endif /* CONFIG_USB_SUSPEND */
#endif
#if defined(STA_SUPPORT) && defined(STA_WEXT)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_RADIO_CTRL,
				    strlen(PRIV_CMD_RADIO_CTRL)) == 0) {
			/* Set/Get radio */
			len = woal_priv_radio_ctrl(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_WMM_CFG,
				    strlen(PRIV_CMD_WMM_CFG)) == 0) {
			/* Implement WMM enable command */
			len = woal_priv_wmm_cfg(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_MIN_BA_THRESH_CFG,
				    strlen(PRIV_CMD_MIN_BA_THRESH_CFG)) == 0) {
			/* Implement Minimum BA threshold configuration command
			 */
			len = woal_priv_min_ba_threshold_cfg(
				priv, buf, priv_cmd.total_len);
			goto handled;
#if defined(STA_SUPPORT)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_11D_CFG,
				    strlen(PRIV_CMD_11D_CFG)) == 0) {
			/* Implement 802.11D enable command */
			len = woal_priv_11d_cfg(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_11D_CLR_TBL,
				    strlen(PRIV_CMD_11D_CLR_TBL)) == 0) {
			/* Implement 802.11D clear chan table command */
			len = woal_priv_11d_clr_chan_tbl(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#endif
#ifndef OPCHAN
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_WWS_CFG,
				    strlen(PRIV_CMD_WWS_CFG)) == 0) {
			/* Set/Get WWS configuration */
			len = woal_priv_wws_cfg(priv, buf, priv_cmd.total_len);
			goto handled;
#endif
#if defined(REASSOCIATION)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_REASSOCTRL,
				    strlen(PRIV_CMD_REASSOCTRL)) == 0) {
			/* Set/Get reassociation settings */
			len = woal_priv_set_get_reassoc(priv, buf,
							priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TXBUF_CFG,
				    strlen(PRIV_CMD_TXBUF_CFG)) == 0) {
			/* Get Transmit buffer size */
			len = woal_priv_txbuf_cfg(priv, buf,
						  priv_cmd.total_len);
			goto handled;
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_AUTH_TYPE,
				    strlen(PRIV_CMD_AUTH_TYPE)) == 0) {
			/* Set/Get auth type */
			len = woal_priv_auth_type(priv, buf,
						  priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_POWER_CONS,
				    strlen(PRIV_CMD_POWER_CONS)) == 0) {
			/* Set/get user provisioned local power constraint */
			len = woal_priv_11h_local_pwr_constraint(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_HT_STREAM_CFG,
				    strlen(PRIV_CMD_HT_STREAM_CFG)) == 0) {
			/* Set/get HT stream configurations */
			len = woal_priv_ht_stream_cfg(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_MIMO_SWITCH,
				    strlen(PRIV_CMD_MIMO_SWITCH)) == 0) {
			/* Set mimo switch configurations */
			len = woal_priv_mimo_switch(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_THERMAL,
				    strlen(PRIV_CMD_THERMAL)) == 0) {
			/* Get thermal reading */
			len = woal_priv_thermal(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_BCN_INTERVAL,
				    strlen(PRIV_CMD_BCN_INTERVAL)) == 0) {
			/* Set/Get beacon interval */
			len = woal_priv_beacon_interval(priv, buf,
							priv_cmd.total_len);
			goto handled;
#ifdef STA_SUPPORT
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_SIGNALEXT_CFG,
				    strlen(PRIV_CMD_SIGNALEXT_CFG)) == 0) {
			/* Set signalext flag */
			len = woal_priv_signalext_cfg(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GET_SIGNAL_EXT_V2,
				    strlen(PRIV_CMD_GET_SIGNAL_EXT_V2)) == 0) {
			/* Get signal info */
			len = woal_priv_get_signal_ext_v2(priv, buf,
							  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GET_SIGNAL_EXT,
				    strlen(PRIV_CMD_GET_SIGNAL_EXT)) == 0) {
			/* Get signal info */
			len = woal_priv_get_signal_ext(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GET_SIGNAL,
				    strlen(PRIV_CMD_GET_SIGNAL)) == 0) {
			/* Get signal */
			len = woal_priv_get_signal(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#endif
#if defined(STA_SUPPORT)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PMFCFG,
				    strlen(PRIV_CMD_PMFCFG)) == 0) {
			/* Configure PMF */
			len = woal_priv_set_get_pmfcfg(priv, buf,
						       priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_INACTIVITYTO,
				    strlen(PRIV_CMD_INACTIVITYTO)) == 0) {
			/* Get/Set inactivity timeout extend */
			len = woal_priv_inactivity_timeout_ext(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_AMSDU_AGGR_CTRL,
				    strlen(PRIV_CMD_AMSDU_AGGR_CTRL)) == 0) {
			/* Enable/Disable amsdu_aggr_ctrl */
			len = woal_priv_11n_amsdu_aggr_ctrl(priv, buf,
							    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TX_BF_CAP,
				    strlen(PRIV_CMD_TX_BF_CAP)) == 0) {
			/* Set/Get Transmit beamforming capabilities */
			len = woal_priv_tx_bf_cap_ioctl(priv, buf,
							priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_SLEEP_PARAMS,
				    strlen(PRIV_CMD_SLEEP_PARAMS)) == 0) {
			/* Configure sleep parameters */
			len = woal_priv_sleep_params_ioctl(priv, buf,
							   priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DFS_TESTING,
				    strlen(PRIV_CMD_DFS_TESTING)) == 0) {
			/* Set/Get DFS Testing settings */
			len = woal_priv_dfs_testing(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DFS53_CFG,
				    strlen(PRIV_CMD_DFS53_CFG)) == 0) {
			/* Set/Get DFS W53 settings */
			len = woal_priv_dfs53cfg(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ARB_CFG,
				    strlen(PRIV_CMD_ARB_CFG)) == 0) {
			/* Set/Get CFP table codes */
			len = woal_priv_arbcfg(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CFP_CODE,
				    strlen(PRIV_CMD_CFP_CODE)) == 0) {
			/* Set/Get CFP table codes */
			len = woal_priv_cfp_code(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CWMODE,
				    strlen(PRIV_CMD_CWMODE)) == 0) {
			/* Set/Get Tx CWMode */
			len = woal_priv_set_get_cwmode(priv, buf,
						       priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ANT_CFG,
				    strlen(PRIV_CMD_ANT_CFG)) == 0) {
			/* Set/Get Tx/Rx antenna */
			len = woal_priv_set_get_tx_rx_ant(priv, buf,
							  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_SYSCLOCK,
				    strlen(PRIV_CMD_SYSCLOCK)) == 0) {
			/* Get/Set system clock */
			len = woal_priv_sysclock(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_GET_KEY,
				    strlen(PRIV_CMD_GET_KEY)) == 0) {
			/* Get GTK/PTK */
			len = woal_priv_get_key(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_ASSOCIATE,
				    strlen(PRIV_CMD_ASSOCIATE)) == 0) {
			/* Associate to a specific indexed entry in the
			 * ScanTable */
			len = woal_priv_associate_ssid_bssid(
				priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TX_BF_CFG,
				    strlen(PRIV_CMD_TX_BF_CFG)) == 0) {
			/* Set/Get Transmit beamforming configuration */
			len = woal_priv_tx_bf_cfg(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_BOOTSLEEP,
				    strlen(PRIV_CMD_BOOTSLEEP)) == 0) {
			len = woal_priv_bootsleep(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PORT_CTRL,
				    strlen(PRIV_CMD_PORT_CTRL)) == 0) {
			/* Set/Get Port Control mode */
			len = woal_priv_port_ctrl(priv, buf,
						  priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PB_BYPASS,
				    strlen(PRIV_CMD_PB_BYPASS)) == 0) {
			/* Private IOCTL entry to get the By-passed TX packet
			 * from upper layer */
			len = woal_priv_bypassed_packet(priv, buf,
							priv_cmd.total_len);
			goto handled;
#ifdef WIFI_DIRECT_SUPPORT
#if defined(UAP_CFG80211)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CFG_NOA,
				    strlen(PRIV_CMD_CFG_NOA)) == 0) {
			/* Set/Get P2P NoA (Notice of Absence) parameters */
			len = woal_priv_cfg_noa(priv, buf, priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_CFG_OPP_PS,
				    strlen(PRIV_CMD_CFG_OPP_PS)) == 0) {
			/* Set/Get P2P OPP-PS parameters */
			len = woal_priv_cfg_opp_ps(priv, buf,
						   priv_cmd.total_len);
			goto handled;
#endif
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_DFS_REPEATER_CFG,
				    strlen(PRIV_CMD_DFS_REPEATER_CFG)) == 0) {
			/* Set/Get DFS_REPEATER mode */
			len = woal_priv_dfs_repeater_cfg(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_MIRACAST_CFG,
				    strlen(PRIV_CMD_MIRACAST_CFG)) == 0) {
			/* Set/Get MIRACAST configuration parameters */
			len = woal_priv_miracast_cfg(priv, buf,
						     priv_cmd.total_len);
			goto handled;
#endif
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_COEX_RX_WINSIZE,
				    strlen(PRIV_CMD_COEX_RX_WINSIZE)) == 0) {
			/* Set/Get control to coex RX window size */
			len = woal_priv_coex_rx_winsize(priv, buf,
							priv_cmd.total_len);
			goto handled;
#ifdef PCIE
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PCIE_REG_RW,
				    strlen(PRIV_CMD_PCIE_REG_RW)) == 0) {
			/* Read/Write PCIE register */
			len = woal_priv_pcie_reg_rw(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_PCIE_BAR0_REG_RW,
				    strlen(PRIV_CMD_PCIE_BAR0_REG_RW)) == 0) {
			/* Read/Write PCIE register/memory from BAR0 */
			len = woal_priv_pcie_bar0_reg_rw(priv, buf,
							 priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GET_SENSOR_TEMP,
				    strlen(PRIV_CMD_GET_SENSOR_TEMP)) == 0) {
			/* Get SOC temperature */
			len = woal_priv_get_sensor_temp(priv, buf,
							priv_cmd.total_len);
			goto handled;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DFS_OFFLOAD,
				    strlen(PRIV_CMD_DFS_OFFLOAD)) == 0) {
			/* Enable/disable DFS offload */
			if (IS_STA_OR_UAP_CFG80211(cfg80211_wext))
				len = woal_priv_dfs_offload_enable(
					priv, buf, priv_cmd.total_len);
			else
				len = sprintf(buf,
					      "CFG80211 is not enabled\n") +
				      1;
			goto handled;
#endif
#endif
#if defined(UAP_SUPPORT)
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_EXTEND_CHAN_SWITCH,
				    strlen(PRIV_CMD_EXTEND_CHAN_SWITCH)) == 0) {
			/* Extended channel switch */
			len = woal_priv_extend_channel_switch(
				priv, buf, priv_cmd.total_len);
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DYN_BW,
				    strlen(PRIV_CMD_DYN_BW)) == 0) {
			/* Set/Get dynamic bandwidth */
			len = woal_priv_config_dyn_bw(priv, buf,
						      priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_IND_RST_CFG,
				    strlen(PRIV_CMD_IND_RST_CFG)) == 0) {
			/* Set/Get out band independent reset */
			len = woal_priv_ind_rst_cfg(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_PER_PKT_CFG,
				    strlen(PRIV_CMD_PER_PKT_CFG)) == 0) {
			/* Get/Set per packet Txctl and Rxinfo configuration */
			len = woal_priv_per_pkt_cfg(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_DEAUTH_CTRL,
				    strlen(PRIV_CMD_DEAUTH_CTRL)) == 0) {
			len = woal_priv_deauth_ctrl(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_11AXCFG,
				    strlen(PRIV_CMD_11AXCFG)) == 0) {
			pdata = buf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_11AXCFG);
			len = priv_cmd.used_len - strlen(PRIV_CMD_11AXCFG) -
			      strlen(CMD_NXP);
			len = woal_priv_11axcfg_cmd(priv, pdata, len,
						    priv_cmd.total_len);
			len += strlen(PRIV_CMD_11AXCFG) + strlen(CMD_NXP);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TWT_SETUP,
				    strlen(PRIV_CMD_TWT_SETUP)) == 0) {
			pdata = buf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_TWT_SETUP);
			len = priv_cmd.used_len - strlen(PRIV_CMD_TWT_SETUP) -
			      strlen(CMD_NXP);
			len = woal_priv_twt_setup(priv, pdata, len,
						  priv_cmd.total_len);
			len += strlen(PRIV_CMD_TWT_SETUP) + strlen(CMD_NXP);
			goto handled;

		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_TWT_TEARDOWN,
				    strlen(PRIV_CMD_TWT_TEARDOWN)) == 0) {
			pdata = buf + strlen(CMD_NXP) +
				strlen(PRIV_CMD_TWT_TEARDOWN);
			len = priv_cmd.used_len -
			      strlen(PRIV_CMD_TWT_TEARDOWN) - strlen(CMD_NXP);
			len = woal_priv_twt_teardown(priv, pdata, len,
						     priv_cmd.total_len);
			len += strlen(PRIV_CMD_TWT_TEARDOWN) + strlen(CMD_NXP);
			goto handled;
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		} else if (strnicmp(buf + strlen(CMD_NXP),
				    PRIV_CMD_GET_CFG_CHAN_LIST,
				    strlen(PRIV_CMD_GET_CFG_CHAN_LIST)) == 0) {
			/* Get txpwrlimit */
			if (IS_STA_OR_UAP_CFG80211(cfg80211_wext))
				len = woal_priv_getcfgchanlist(
					priv, buf, priv_cmd.total_len);
			else
				len = sprintf(buf,
					      "CFG80211 is not enabled\n") +
				      1;
			goto handled;
#endif
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_LPM,
				    strlen(PRIV_CMD_LPM)) == 0) {
			/* Set/Get low power mode */
			len = woal_priv_set_get_lpm(priv, buf,
						    priv_cmd.total_len);
			goto handled;
		} else if (strnicmp(buf + strlen(CMD_NXP), PRIV_CMD_TP_STATE,
				    strlen(PRIV_CMD_TP_STATE)) == 0) {
			/* Set/Get TP accounting state */
			len = woal_priv_set_tp_state(priv, buf,
						     priv_cmd.total_len);
			goto handled;
		} else {
			PRINTM(MERROR,
			       "Unknown NXP PRIVATE command %s, ignored\n",
			       buf);
			ret = -EFAULT;
			goto done;
		}
	}
#ifdef STA_SUPPORT
	if (strncmp(buf, "RSSILOW-THRESHOLD", strlen("RSSILOW-THRESHOLD")) ==
	    0) {
		pdata = buf + strlen("RSSILOW-THRESHOLD") + 1;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_rssi_low_threshold(priv, pdata, MOAL_IOCTL_WAIT)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SCAN-CFG", strlen("SCAN-CFG")) == 0) {
		PRINTM(MIOCTL, "Set SCAN CFG\n");
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_cfg(priv, buf, priv_cmd.total_len)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "RSSI", strlen("RSSI")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_bss_info(priv, MOAL_IOCTL_WAIT, &bss_info)) {
			ret = -EFAULT;
			goto done;
		}
		if (bss_info.media_connected) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_get_signal_info(priv, MOAL_IOCTL_WAIT,
						 &signal)) {
				ret = -EFAULT;
				goto done;
			}
			len = sprintf(buf, "%.32s rssi %d\n",
				      bss_info.ssid.ssid, signal.bcn_rssi_avg) +
			      1;
		} else {
			len = sprintf(buf, "OK\n") + 1;
		}
	} else if (strncmp(buf, "LINKSPEED", strlen("LINKSPEED")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_get_data_rate(priv, MLAN_ACT_GET, &rate)) {
			ret = -EFAULT;
			goto done;
		}
		PRINTM(MIOCTL, "tx rate=%d\n", (int)rate.rate);
		len = sprintf(buf, "LinkSpeed %d\n",
			      (int)(rate.rate * 500000 / 1000000)) +
		      1;
	} else
#endif
		if (strncmp(buf, "MACADDR", strlen("MACADDR")) == 0) {
		len = sprintf(buf, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
			      priv->current_addr[0], priv->current_addr[1],
			      priv->current_addr[2], priv->current_addr[3],
			      priv->current_addr[4], priv->current_addr[5]) +
		      1;
	}
#ifdef STA_SUPPORT
	else if (strncmp(buf, "GETPOWER", strlen("GETPOWER")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_get_powermode(priv, &power_mode)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "powermode = %d\n", power_mode) + 1;
	} else if (strncmp(buf, "SCAN-ACTIVE", strlen("SCAN-ACTIVE")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_type(priv, MLAN_SCAN_TYPE_ACTIVE)) {
			ret = -EFAULT;
			goto done;
		}
		priv->scan_type = MLAN_SCAN_TYPE_ACTIVE;
		PRINTM(MIOCTL, "Set Active Scan\n");
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SCAN-PASSIVE", strlen("SCAN-PASSIVE")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_type(priv, MLAN_SCAN_TYPE_PASSIVE)) {
			ret = -EFAULT;
			goto done;
		}
		priv->scan_type = MLAN_SCAN_TYPE_PASSIVE;
		PRINTM(MIOCTL, "Set Passive Scan\n");
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "POWERMODE", strlen("POWERMODE")) == 0) {
		pdata = buf + strlen("POWERMODE") + 1;
		if (!moal_extflg_isset(priv->phandle, EXT_HW_TEST)) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_set_powermode(priv, pdata)) {
				ret = -EFAULT;
				goto done;
			}
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SETROAMING", strlen("SETROAMING")) == 0) {
		pdata = buf + strlen("SETROAMING") + 1;
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
			goto done;
#endif
#endif
#ifdef STA_CFG80211
		if (*pdata == '1') {
			priv->roaming_enabled = MTRUE;
			PRINTM(MIOCTL, "Roaming enabled\n");
		} else if (*pdata == '0') {
			priv->roaming_enabled = MFALSE;
			PRINTM(MIOCTL, "Roaming disabled\n");
		}
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "ROAM", strlen("ROAM")) == 0) {
		pdata = buf + strlen("ROAM") + 1;
#ifdef STA_CFG80211
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		if (moal_extflg_isset(priv->phandle, EXT_HOST_MLME))
			goto done;
#endif
#endif
#ifdef STA_CFG80211
		if (*pdata == '1') {
			priv->roaming_enabled = MTRUE;
			PRINTM(MIOCTL, "Roaming enabled\n");
		} else if (*pdata == '0') {
			priv->roaming_enabled = MFALSE;
			PRINTM(MIOCTL, "Roaming disabled\n");
		}
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "COUNTRY", strlen("COUNTRY")) == 0) {
		copy_len = strlen(buf) - strlen("COUNTRY") - 1;
		if (copy_len > COUNTRY_CODE_LEN || copy_len <= 0) {
			PRINTM(MERROR, "Invalid country length\n");
			ret = -EFAULT;
			goto done;
		}
		memset(country_code, 0, sizeof(country_code));
		moal_memcpy_ext(priv->phandle, country_code,
				buf + strlen("COUNTRY") + 1, copy_len,
				COUNTRY_CODE_LEN);
		PRINTM(MIOCTL, "Set COUNTRY %s\n", country_code);
		if (moal_extflg_isset(priv->phandle, EXT_CNTRY_TXPWR)) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_request_country_power_table(priv,
							     country_code)) {
				ret = -EFAULT;
				goto done;
			}
		}
#ifdef STA_CFG80211
		if (IS_STA_CFG80211(cfg80211_wext)) {
			PRINTM(MIOCTL, "Notify country code=%s\n",
			       country_code);
			if (!moal_extflg_isset(priv->phandle,
					       EXT_DISABLE_REGD_BY_DRIVER))
				regulatory_hint(priv->wdev->wiphy,
						country_code);
			len = sprintf(buf, "OK\n") + 1;
			goto done;
		}
#endif
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_region_code(priv, country_code)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (memcmp(buf, WEXT_CSCAN_HEADER, WEXT_CSCAN_HEADER_SIZE) ==
		   0) {
		PRINTM(MIOCTL, "Set Combo Scan\n");
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_combo_scan(priv, buf, priv_cmd.total_len)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "GETBAND", strlen("GETBAND")) == 0) {
		if (MLAN_STATUS_SUCCESS != woal_get_band(priv, &band)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "Band %d\n", band) + 1;
	} else if (strncmp(buf, "SETBAND", strlen("SETBAND")) == 0) {
		pband = buf + strlen("SETBAND") + 1;
		if (MLAN_STATUS_SUCCESS != woal_set_band(priv, pband)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	}
#endif
	else if (strncmp(buf, "START", strlen("START")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "STOP", strlen("STOP")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	}
#ifdef UAP_SUPPORT
	else if (strncmp(buf, "AP_BSS_START", strlen("AP_BSS_START")) == 0) {
		ret = woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_START);
		if (ret)
			goto done;
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "AP_BSS_STOP", strlen("AP_BSS_STOP")) == 0) {
		ret = woal_uap_bss_ctrl(priv, MOAL_IOCTL_WAIT, UAP_BSS_STOP);
		if (ret)
			goto done;
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "AP_SET_CFG", strlen("AP_SET_CFG")) == 0) {
		if (priv_cmd.total_len <= strlen("AP_SET_CFG") + 1)
			goto done;
		pdata = buf + strlen("AP_SET_CFG") + 1;
		ret = woal_uap_set_ap_cfg(priv, pdata,
					  priv_cmd.total_len -
						  strlen("AP_SET_CFG") - 1);
		if (ret)
			goto done;
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "WL_FW_RELOAD", strlen("WL_FW_RELOAD")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "AP_GET_STA_LIST", strlen("AP_GET_STA_LIST")) ==
		   0) {
		/* TODO Add STA list support */
		len = sprintf(buf, "OK\n") + 1;
	}
#endif
	else if (strncmp(buf, "SETSUSPENDOPT", strlen("SETSUSPENDOPT")) == 0) {
		/* it will be done by GUI */
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SETSUSPENDMODE", strlen("SETSUSPENDMODE")) ==
		   0) {
		/* it will be done by GUI */
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXMODE", strlen("BTCOEXMODE")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXSCAN-START",
			   strlen("BTCOEXSCAN-START")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "BTCOEXSCAN-STOP", strlen("BTCOEXSCAN-STOP")) ==
		   0) {
		len = sprintf(buf, "OK\n") + 1;
	}
#ifdef STA_SUPPORT
	else if (strncmp(buf, "BGSCAN-START", strlen("BGSCAN-START")) == 0) {
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "BGSCAN-CONFIG", strlen("BGSCAN-CONFIG")) ==
		   0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_bg_scan(priv, buf, priv_cmd.total_len)) {
			ret = -EFAULT;
			goto done;
		}
		priv->bg_scan_start = MTRUE;
		priv->bg_scan_reported = MFALSE;
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "BGSCAN-STOP", strlen("BGSCAN-STOP")) == 0) {
		if (priv->bg_scan_start && !priv->scan_cfg.rssi_threshold) {
			if (MLAN_STATUS_SUCCESS !=
			    woal_stop_bg_scan(priv, MOAL_NO_WAIT)) {
				ret = -EFAULT;
				goto done;
			}
			priv->bg_scan_start = MFALSE;
			priv->bg_scan_reported = MFALSE;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-START", strlen("RXFILTER-START")) ==
		   0) {
#ifdef MEF_CFG_RX_FILTER
		ret = woal_set_rxfilter(priv, MTRUE);
		if (ret)
			goto done;
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-STOP", strlen("RXFILTER-STOP")) ==
		   0) {
#ifdef MEF_CFG_RX_FILTER
		ret = woal_set_rxfilter(priv, MFALSE);
		if (ret)
			goto done;
#endif
		len = sprintf(buf, "OK\n") + 1;
	}
#ifdef STA_CFG80211
	else if (strncmp(buf, "GET_EVENT", strlen("GET_EVENT")) == 0) {
		if (IS_STA_CFG80211(cfg80211_wext)) {
			if (priv->last_event & EVENT_BG_SCAN_REPORT)
				woal_inform_bss_from_scan_result(
					priv, NULL, MOAL_IOCTL_WAIT);
		}
		len = sprintf(buf, "EVENT=%d\n", priv->last_event) + 1;
		priv->last_event = 0;
	} else if (strncmp(buf, "GET_802_11W", strlen("GET_802_11W")) == 0) {
		len = sprintf(buf, "802_11W=ENABLED\n") + 1;
	}
#endif /* STA_CFG80211 */
	else if (strncmp(buf, "RXFILTER-ADD", strlen("RXFILTER-ADD")) == 0) {
		pdata = buf + strlen("RXFILTER-ADD") + 1;
		if (MLAN_STATUS_SUCCESS != woal_add_rxfilter(priv, pdata)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "RXFILTER-REMOVE", strlen("RXFILTER-REMOVE")) ==
		   0) {
		pdata = buf + strlen("RXFILTER-REMOVE") + 1;
		if (MLAN_STATUS_SUCCESS != woal_remove_rxfilter(priv, pdata)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "QOSINFO", strlen("QOSINFO")) == 0) {
		pdata = buf + strlen("QOSINFO") + 1;
#ifdef STA_SUPPORT
		if (MLAN_STATUS_SUCCESS !=
		    woal_priv_qos_cfg(priv, MLAN_ACT_SET, pdata)) {
			ret = -EFAULT;
			goto done;
		}
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SLEEPPD", strlen("SLEEPPD")) == 0) {
		pdata = buf + strlen("SLEEPPD") + 1;
		if (MLAN_STATUS_SUCCESS != woal_set_sleeppd(priv, pdata)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "SET_AP_WPS_P2P_IE",
			   strlen("SET_AP_WPS_P2P_IE")) == 0) {
		pdata = buf + strlen("SET_AP_WPS_P2P_IE") + 1;
		/* Android cmd format:
		 * "SET_AP_WPS_P2P_IE 1"  -- beacon IE
		 * "SET_AP_WPS_P2P_IE 2"  -- proberesp IE
		 * "SET_AP_WPS_P2P_IE 4"  -- assocresp IE
		 */
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_ap_wps_p2p_ie(priv, (t_u8 *)pdata,
					   priv_cmd.used_len -
						   strlen("SET_AP_WPS_P2P_IE") -
						   1)) {
			ret = -EFAULT;
			goto done;
		}
#endif
#endif
		len = sprintf(buf, "OK\n") + 1;
	}
#endif
	else if (strncmp(buf, "P2P_DEV_ADDR", strlen("P2P_DEV_ADDR")) == 0) {
		memset(buf, 0x0, (size_t)priv_cmd.total_len);
		moal_memcpy_ext(priv->phandle, buf, priv->current_addr,
				ETH_ALEN, (t_u32)priv_cmd.total_len);
		len = ETH_ALEN;
	} else if (strncmp(buf, ("P2P_GET_NOA"), strlen("P2P_GET_NOA")) == 0) {
		/* TODO
		 * Just return '\0'
		 */
		memset(buf, 0x0, (size_t)priv_cmd.total_len);
		*buf = 0;
		len = 1;
	} else if (strnicmp(buf, "MIRACAST", strlen("MIRACAST")) == 0) {
		pdata = buf + strlen("MIRACAST");
		/* Android cmd format:
		 * "MIRACAST 0"  -- disabled
		 * "MIRACAST 1"  -- operating as source
		 * "MIRACAST 2"  -- operating as sink
		 */
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_miracast_mode(priv, (t_u8 *)pdata,
					   priv_cmd.used_len -
						   strlen("MIRACAST"))) {
			ret = -EFAULT;
			goto done;
		}
#endif
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strnicmp(buf, "SCAN_TIMING", strlen("SCAN_TIMING")) == 0) {
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_scan_chan_gap(priv, buf, priv_cmd.total_len)) {
			ret = -EFAULT;
			goto done;
		}
#endif
#endif
		len = sprintf(buf, "OK\n") + 1;
	} else if (strnicmp(buf, "BA_WSIZE_RX", strlen("BA_WSIZE_RX")) == 0) {
		pdata = buf + strlen("BA_WSIZE_RX") + 1;
		len = priv_cmd.total_len - strlen("BA_WSIZE_RX") - 1;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_rx_ba_winsize(priv, pdata, len)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strnicmp(buf, "BA_WSIZE_TX", strlen("BA_WSIZE_TX")) == 0) {
		pdata = buf + strlen("BA_WSIZE_TX") + 1;
		len = priv_cmd.total_len - strlen("BA_WSIZE_TX") - 1;
		if (MLAN_STATUS_SUCCESS !=
		    woal_set_tx_ba_winsize(priv, pdata, len)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	} else if (strncmp(buf, "FAKE_SCAN_COMPLETE",
			   strlen("FAKE_SCAN_COMPLETE")) == 0) {
		pdata = buf + strlen("FAKE_SCAN_COMPLETE") + 1;
#ifdef STA_CFG80211
		if (*pdata == '1') {
			priv->fake_scan_complete = MTRUE;
			PRINTM(MIOCTL, "fake scan complete enabled\n");
		} else if (*pdata == '0') {
			priv->fake_scan_complete = MFALSE;
			PRINTM(MIOCTL, "fake scan complete disabled\n");
		}
#endif
		len = sprintf(buf, "OK\n") + 1;
	}
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#ifdef WIFI_DIRECT_SUPPORT
	else if (strncmp(buf, "P2P_PERIODIC_SLEEP",
			 strlen("P2P_PERIODIC_SLEEP")) == 0) {
		if (MLAN_STATUS_SUCCESS !=
		    woal_p2p_ps_cfg(priv, buf, priv_cmd.total_len)) {
			ret = -EFAULT;
			goto done;
		}
		len = sprintf(buf, "OK\n") + 1;
	}
#endif
#endif
	else if (strncmp(buf, "WLS_BATCHING", strlen("WLS_BATCHING")) == 0) {
		/* TODO */
		len = sprintf(buf, "OK\n") + 1;
	}
#if defined(UAP_SUPPORT)
	else if (strncmp(buf, "P2P_ECSA", strlen("P2P_ECSA")) == 0) {
		len = woal_priv_p2p_ecsa(priv, buf, priv_cmd.total_len);
	}
#endif
	else if (strncmp(buf, "FAKEMAC", strlen("FAKEMAC")) == 0) {
		len = woal_priv_config_random_mac(priv, buf,
						  priv_cmd.total_len);
	} else if (strncmp(buf, PRIV_CMD_CLOUD_KEEP_ALIVE,
			   strlen(PRIV_CMD_CLOUD_KEEP_ALIVE)) == 0) {
		len = woal_priv_cloud_keep_alive(priv, buf, priv_cmd.total_len);
	} else if (strnicmp(buf, PRIV_CMD_TX_RX_HISTOGRAM,
			    strlen(PRIV_CMD_TX_RX_HISTOGRAM)) == 0) {
		/* Get TX/RX histogram statistic */
		len = woal_priv_get_rx_tx_histogram(priv, buf,
						    priv_cmd.total_len);
		goto handled;
	} else if (strnicmp(buf, PRIV_CMD_GET_CHNRGPWR,
			    strlen(PRIV_CMD_GET_CHNRGPWR)) == 0) {
		/* Get chnrgpwr */
		len = woal_priv_get_chnrgpwr(priv, buf, priv_cmd.total_len);
		goto handled;
	} else if (strnicmp(buf, PRIV_CMD_GET_TXPWR_LIMIT,
			    strlen(PRIV_CMD_GET_TXPWR_LIMIT)) == 0) {
		/* Get txpwrlimit */
		len = woal_priv_get_txpwrlimit(priv, buf, priv_cmd.total_len);
		goto handled;
	} else {
		PRINTM(MIOCTL, "Unknown PRIVATE command: %s, ignored\n", buf);
		ret = -EFAULT;
		goto done;
	}

handled:
	PRINTM(MIOCTL, "PRIV Command return: %s, length=%d\n", buf, len);

	if (len > 0) {
		priv_cmd.used_len = len;
		if (priv_cmd.used_len <= priv_cmd.total_len) {
			memset(buf + priv_cmd.used_len, 0,
			       (size_t)(CMD_BUF_LEN - priv_cmd.used_len));
			if (copy_to_user(cmd_buf, buf, priv_cmd.total_len)) {
				PRINTM(MERROR,
				       "%s: failed to copy data to user buffer\n",
				       __FUNCTION__);
				ret = -EFAULT;
				goto done;
			}
			if (copy_to_user(req->ifr_data, &priv_cmd,
					 sizeof(android_wifi_priv_cmd))) {
				PRINTM(MERROR,
				       "%s: failed to copy command header to user buffer\n",
				       __FUNCTION__);
				ret = -EFAULT;
			}
		} else {
			PRINTM(MERROR,
			       "%s: the buffer supplied by appl is too small (supplied: %d, used: %d)\n",
			       __FUNCTION__, priv_cmd.total_len,
			       priv_cmd.used_len);
			ret = -EFAULT;
		}
	} else {
		ret = len;
	}

done:
	kfree(buf);
	LEAVE();
	return ret;
}

/********************************************************
			Global Functions
********************************************************/
/**
 *  @brief Create a brief scan resp to relay basic BSS info to the app layer
 *
 *  When the beacon/probe response has not been buffered, use the saved BSS
 *    information available to provide a minimum response for the application
 *    ioctl retrieval routines.  Include:
 *        - Timestamp
 *        - Beacon Period
 *        - Capabilities (including WMM Element if available)
 *        - SSID
 *
 *  @param ppbuffer  Output parameter: Buffer used to create basic scan rsp
 *  @param pbss_desc Pointer to a BSS entry in the scan table to create
 *                   scan response from for delivery to the application layer
 *
 *  @return          N/A
 */
void wlan_scan_create_brief_table_entry(t_u8 **ppbuffer,
					BSSDescriptor_t *pbss_desc)
{
	t_u8 *ptmp_buf = *ppbuffer;
	t_u8 tmp_ssid_hdr[2];
	t_u8 ie_len = 0;

	ENTER();

	moal_memcpy_ext(NULL, ptmp_buf, pbss_desc->time_stamp,
			sizeof(pbss_desc->time_stamp),
			sizeof(pbss_desc->time_stamp));
	ptmp_buf += sizeof(pbss_desc->time_stamp);

	moal_memcpy_ext(NULL, ptmp_buf, &pbss_desc->beacon_period,
			sizeof(pbss_desc->beacon_period),
			sizeof(pbss_desc->beacon_period));
	ptmp_buf += sizeof(pbss_desc->beacon_period);

	moal_memcpy_ext(NULL, ptmp_buf, &pbss_desc->cap_info,
			sizeof(pbss_desc->cap_info),
			sizeof(pbss_desc->cap_info));
	ptmp_buf += sizeof(pbss_desc->cap_info);

	tmp_ssid_hdr[0] = 0; /* Element ID for SSID is zero */
	tmp_ssid_hdr[1] = pbss_desc->ssid.ssid_len;
	moal_memcpy_ext(NULL, ptmp_buf, tmp_ssid_hdr, sizeof(tmp_ssid_hdr),
			sizeof(tmp_ssid_hdr));
	ptmp_buf += sizeof(tmp_ssid_hdr);

	moal_memcpy_ext(NULL, ptmp_buf, pbss_desc->ssid.ssid,
			pbss_desc->ssid.ssid_len, pbss_desc->ssid.ssid_len);
	ptmp_buf += pbss_desc->ssid.ssid_len;

	if (pbss_desc->wmm_ie.vend_hdr.element_id == WMM_IE) {
		ie_len = sizeof(IEEEtypes_Header_t) +
			 pbss_desc->wmm_ie.vend_hdr.len;
		moal_memcpy_ext(NULL, ptmp_buf, &pbss_desc->wmm_ie, ie_len,
				ie_len);
		ptmp_buf += ie_len;
	}

	if (pbss_desc->pwpa_ie) {
		if ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id == WPA_IE) {
			ie_len = sizeof(IEEEtypes_Header_t) +
				 (*(pbss_desc->pwpa_ie)).vend_hdr.len;
			moal_memcpy_ext(NULL, ptmp_buf, pbss_desc->pwpa_ie,
					ie_len, ie_len);
		}

		ptmp_buf += ie_len;
	}
	if (pbss_desc->prsn_ie) {
		if ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id == RSN_IE) {
			ie_len = sizeof(IEEEtypes_Header_t) +
				 (*(pbss_desc->prsn_ie)).ieee_hdr.len;
			moal_memcpy_ext(NULL, ptmp_buf, pbss_desc->prsn_ie,
					ie_len, ie_len);
		}

		ptmp_buf += ie_len;
	}
	*ppbuffer = ptmp_buf;
	LEAVE();
}

/**
 *  @brief Create a wlan_ioctl_get_scan_table_entry for a given BSS
 *         Descriptor for inclusion in the ioctl response to the user space
 *         application.
 *
 *
 *  @param pbss_desc   Pointer to a BSS entry in the scan table to form
 *                     scan response from for delivery to the application layer
 *  @param ppbuffer    Output parameter: Buffer used to output scan return
 * struct
 *  @param pspace_left Output parameter: Number of bytes available in the
 *                     response buffer.
 *
 *  @return MLAN_STATUS_SUCCESS, or < 0 with IOCTL error code
 */
int wlan_get_scan_table_ret_entry(BSSDescriptor_t *pbss_desc, t_u8 **ppbuffer,
				  int *pspace_left)
{
	wlan_ioctl_get_scan_table_entry *prsp_entry;
	wlan_ioctl_get_scan_table_entry tmp_rsp_entry;
	int space_needed;
	t_u8 *pcurrent;
	int variable_size;

	const int fixed_size = sizeof(wlan_ioctl_get_scan_table_entry);

	ENTER();

	pcurrent = *ppbuffer;

	/* The variable size returned is the stored beacon size */
	variable_size = pbss_desc->beacon_buf_size;

	/* If we stored a beacon and its size was zero, set the variable
	 *  size return value to the size of the brief scan response
	 *  wlan_scan_create_brief_table_entry creates.  Also used if
	 *  we are not configured to store beacons in the first place
	 */
	if (!variable_size) {
		variable_size = pbss_desc->ssid.ssid_len + 2;
		variable_size += (sizeof(pbss_desc->beacon_period) +
				  sizeof(pbss_desc->time_stamp) +
				  sizeof(pbss_desc->cap_info));
		if (pbss_desc->wmm_ie.vend_hdr.element_id == WMM_IE) {
			variable_size += (sizeof(IEEEtypes_Header_t) +
					  pbss_desc->wmm_ie.vend_hdr.len);
		}

		if (pbss_desc->pwpa_ie) {
			if ((*(pbss_desc->pwpa_ie)).vend_hdr.element_id ==
			    WPA_IE) {
				variable_size +=
					(sizeof(IEEEtypes_Header_t) +
					 (*(pbss_desc->pwpa_ie)).vend_hdr.len);
			}
		}

		if (pbss_desc->prsn_ie) {
			if ((*(pbss_desc->prsn_ie)).ieee_hdr.element_id ==
			    RSN_IE) {
				variable_size +=
					(sizeof(IEEEtypes_Header_t) +
					 (*(pbss_desc->prsn_ie)).ieee_hdr.len);
			}
		}
	}

	space_needed = fixed_size + variable_size;

	PRINTM(MINFO, "GetScanTable: need(%d), left(%d)\n", space_needed,
	       *pspace_left);

	if (space_needed >= *pspace_left) {
		*pspace_left = 0;
		LEAVE();
		return -E2BIG;
	}

	*pspace_left -= space_needed;

	tmp_rsp_entry.fixed_field_length =
		(sizeof(tmp_rsp_entry) -
		 sizeof(tmp_rsp_entry.fixed_field_length) -
		 sizeof(tmp_rsp_entry.bss_info_length));

	moal_memcpy_ext(NULL, tmp_rsp_entry.fixed_fields.bssid,
			pbss_desc->mac_address,
			sizeof(prsp_entry->fixed_fields.bssid),
			sizeof(prsp_entry->fixed_fields.bssid));

	tmp_rsp_entry.fixed_fields.rssi = pbss_desc->rssi;
	tmp_rsp_entry.fixed_fields.channel = pbss_desc->channel;
	tmp_rsp_entry.fixed_fields.chan_load = pbss_desc->chan_load;
	tmp_rsp_entry.fixed_fields.network_tsf = pbss_desc->network_tsf;
	tmp_rsp_entry.bss_info_length = variable_size;

	/*
	 *  Copy fixed fields to user space
	 */
	moal_memcpy_ext(NULL, pcurrent, &tmp_rsp_entry, fixed_size, fixed_size);
	pcurrent += fixed_size;

	if (pbss_desc->pbeacon_buf) {
		/*
		 *  Copy variable length elements to user space
		 */
		moal_memcpy_ext(NULL, pcurrent, pbss_desc->pbeacon_buf,
				pbss_desc->beacon_buf_size,
				pbss_desc->beacon_buf_size);

		pcurrent += pbss_desc->beacon_buf_size;
	} else {
		wlan_scan_create_brief_table_entry(&pcurrent, pbss_desc);
	}

	*ppbuffer = pcurrent;

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief ioctl function - entry point
 *
 *  @param dev      A pointer to net_device structure
 *  @param req      A pointer to ifreq structure
 *  @param cmd      Command
 *
 *  @return          0 --success, otherwise fail
 */
int woal_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	int ret = 0;

	ENTER();

	PRINTM(MINFO, "woal_do_ioctl: ioctl cmd = 0x%x\n", cmd);
	switch (cmd) {
	case WOAL_ANDROID_DEF_CMD:
		/** android default ioctl ID is SIOCDEVPRIVATE + 1 */
		ret = woal_android_priv_cmd(dev, req);
		break;
	case WOAL_CUSTOM_IE_CFG:
		ret = woal_custom_ie_ioctl(dev, req);
		break;
	case WOAL_MGMT_FRAME_TX:
		ret = woal_send_host_packet(dev, req);
		break;
	case WOAL_ANDROID_PRIV_CMD:
		ret = woal_android_priv_cmd(dev, req);
		break;
	case WOAL_GET_BSS_TYPE:
		ret = woal_get_bss_type(dev, req);
		break;
	default:
#if defined(STA_WEXT)
#ifdef STA_SUPPORT
		ret = woal_wext_do_ioctl(dev, req, cmd);
#else
		ret = -EINVAL;
#endif
#else
		ret = -EINVAL;
#endif
		break;
	}

	LEAVE();
	return ret;
}
