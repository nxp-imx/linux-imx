/** @file moal_init.c
 *
 * @brief This file contains the major functions in WLAN
 * driver.
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
#include "moal_main.h"

/** Global moal_handle array */
extern pmoal_handle m_handle[];

/** Firmware name */
char *fw_name;
int req_fw_nowait;
int fw_reload;

/** MAC address */
char *mac_addr;
/** Module param cfg file */
char *mod_para;

#ifdef MFG_CMD_SUPPORT
/** Mfg mode */
int mfg_mode;
#endif

#if defined(SDIO)
/** SDIO interrupt mode (0: INT_MODE_SDIO, 1: INT_MODE_GPIO) */
int intmode = INT_MODE_SDIO;
/** GPIO interrupt pin number */
int gpiopin;
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
int disable_regd_by_driver = 1;
/** Region alpha2 string */
char *reg_alpha2;
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
int country_ie_ignore;
int beacon_hints;
#endif
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
int host_mlme;
#endif
#endif

/** Auto deep sleep */
int auto_ds;

/** IEEE PS mode */
int ps_mode;
/** passive to active scan */
int p2a_scan;
/** scan chan gap */
int scan_chan_gap;
/** Max Tx buffer size */
int max_tx_buf;

#ifdef STA_SUPPORT
/** Max STA interfaces */
int max_sta_bss = DEF_STA_BSS;
/** STA interface name */
char *sta_name;
#endif

#ifdef UAP_SUPPORT
/** Max uAP interfaces */
int max_uap_bss = DEF_UAP_BSS;
/** uAP interface name */
char *uap_name;
/** Max uAP station number */
int uap_max_sta;
#endif

#ifdef WIFI_DIRECT_SUPPORT
/** Max WIFIDIRECT interfaces */
int max_wfd_bss = DEF_WIFIDIRECT_BSS;
/** WIFIDIRECT interface name */
char *wfd_name;
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
/** max VIRTUAL bss */
int max_vir_bss = DEF_VIRTUAL_BSS;
#endif
#endif

#ifdef SDIO_SUSPEND_RESUME
/** PM keep power */
int pm_keep_power = 1;
/** HS when shutdown */
int shutdown_hs;
#endif

#if defined(SDIO)
/** SDIO slew rate */
int slew_rate = 3;
#endif
int tx_work = 0;

#if defined(STA_SUPPORT)
/** 802.11d configuration */
int cfg_11d;
#endif

/** fw serial download check */
int fw_serial = 1;

/** napi support*/
int napi;

/** DPD data config file */
char *dpd_data_cfg;

/** CAL data config file */
char *cal_data_cfg;
/** Init config file (MAC address, register etc.) */
char *init_cfg;

/** Set configuration data of Tx power limitation */
char *txpwrlimit_cfg;
/** Allow setting tx power table of country */
int cntry_txpwr;

/** Init hostcmd file */
char *init_hostcmd_cfg;
char *band_steer_cfg;

#if defined(STA_WEXT) || defined(UAP_WEXT)
/** CFG80211 and WEXT mode */
int cfg80211_wext = STA_WEXT_MASK | UAP_WEXT_MASK;
#else
/** CFG80211 mode */
int cfg80211_wext = STA_CFG80211_MASK | UAP_CFG80211_MASK;
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
int fw_region = 1;
#endif
#endif

/** Work queue priority */
int wq_sched_prio;
/** Work queue scheduling policy */
int wq_sched_policy = SCHED_NORMAL;
/** rx_work flag */
int rx_work;

#if defined(USB)
int skip_fwdnld;
#endif

/* Enable/disable aggrctrl */
int aggrctrl;

#ifdef USB
/* Enable/disable USB aggregation feature */
int usb_aggr;
#endif

#ifdef PCIE
/* Enable/disable Message Signaled Interrupt (MSI) */
int pcie_int_mode = PCIE_INT_MODE_MSI;
#endif /* PCIE */

int low_power_mode_enable;

int hw_test;

#ifdef CONFIG_OF
int dts_enable = 1;
#endif

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
int dfs_offload;
#endif

#ifdef ANDROID_KERNEL
int wakelock_timeout = WAKE_LOCK_TIMEOUT;
#endif

#if defined(STA_SUPPORT) && defined(UAP_SUPPORT)
#ifdef WIFI_DIRECT_SUPPORT
int drv_mode = (DRV_MODE_STA | DRV_MODE_UAP | DRV_MODE_WIFIDIRECT);
#else
int drv_mode = (DRV_MODE_STA | DRV_MODE_UAP);
#endif /* WIFI_DIRECT_SUPPORT */
#else
#ifdef STA_SUPPORT
int drv_mode = DRV_MODE_STA;
#else
int drv_mode = DRV_MODE_UAP;
#endif /* STA_SUPPORT */
#endif /* STA_SUPPORT & UAP_SUPPORT */

int gtk_rekey_offload = GTK_REKEY_OFFLOAD_DISABLE;

int pmic;

int antcfg;

t_u32 uap_oper_ctrl;

int hs_wake_interval = 400;
int indication_gpio = 0xff;
int disconnect_on_suspend;
int hs_mimo_switch;

int indrstcfg = 0xffffffff;

/** all the feature are enabled */
#define DEFAULT_DEV_CAP_MASK 0xffffffff
t_u32 dev_cap_mask = DEFAULT_DEV_CAP_MASK;
#ifdef SDIO
int sdio_rx_aggr = MTRUE;
#endif

/** The global variable of scan beacon buffer **/
int fixed_beacon_buffer;

#ifdef WIFI_DIRECT_SUPPORT
int GoAgeoutTime;
#endif

t_u16 multi_dtim;

t_u16 inact_tmo;

#ifdef DEBUG_LEVEL1
#ifdef DEBUG_LEVEL2
#define DEFAULT_DEBUG_MASK (0xffffffff)
#else
#define DEFAULT_DEBUG_MASK (MMSG | MFATAL | MERROR | MREG_D)
#endif /* DEBUG_LEVEL2 */
t_u32 drvdbg = DEFAULT_DEBUG_MASK;

#endif /* DEBUG_LEVEL1 */

static card_type_entry card_type_map_tbl[] = {
#ifdef SD8887
	{CARD_TYPE_SD8887, 0, CARD_SD8887},
#endif
#ifdef SD8897
	{CARD_TYPE_SD8897, 0, CARD_SD8897},
#endif
#ifdef SD8977
	{CARD_TYPE_SD8977, 0, CARD_SD8977},
#endif
#ifdef SD8978
	{CARD_TYPE_SD8978, 0, CARD_SD8978},
#endif
#ifdef SD8997
	{CARD_TYPE_SD8997, 0, CARD_SD8997},
#endif
#ifdef SD8987
	{CARD_TYPE_SD8987, 0, CARD_SD8987},
#endif
#ifdef SD9097
	{CARD_TYPE_SD9097, 0, CARD_SD9097},
#endif
#ifdef SD9098
	{CARD_TYPE_SD9098, 0, CARD_SD9098},
#endif
#ifdef PCIE8897
	{CARD_TYPE_PCIE8897, 0, CARD_PCIE8897},
#endif
#ifdef PCIE8997
	{CARD_TYPE_PCIE8997, 0, CARD_PCIE8997},
#endif
#ifdef PCIE9097
	{CARD_TYPE_PCIE9097, 0, CARD_PCIE9097},
#endif
#ifdef PCIE9098
	{CARD_TYPE_PCIE9098, 0, CARD_PCIE9098},
#endif
#ifdef USB8897
	{CARD_TYPE_USB8897, 0, CARD_USB8897},
#endif
#ifdef USB8997
	{CARD_TYPE_USB8997, 0, CARD_USB8997},
#endif
#ifdef USB8978
	{CARD_TYPE_USB8978, 0, CARD_USB8978},
#endif
#ifdef USB9098
	{CARD_TYPE_USB9098, 0, CARD_USB9098},
#endif
#ifdef USB9097
	{CARD_TYPE_USB9097, 0, CARD_USB9097},
#endif
};

int dfs53cfg = DFS_W53_DEFAULT_FW;

/**
 *  @brief This function read a line in module parameter file
 *
 *  @param data     A pointer to module parameter data buffer
 *  @param size     module parameter file size
 *  @param line_pos A pointer to offset of current line
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static t_size parse_cfg_get_line(t_u8 *data, t_size size, t_u8 *line_pos)
{
	t_u8 *src, *dest;
	static t_s32 pos;

	ENTER();

	if (pos >= size) { /* reach the end */
		pos = 0; /* Reset position for rfkill */
		LEAVE();
		return -1;
	}
	memset(line_pos, 0, MAX_LINE_LEN);
	src = data + pos;
	dest = line_pos;

	while (pos < size && *src != '\x0A' && *src != '\0') {
		if (*src != ' ' && *src != '\t') /* parse space */
			*dest++ = *src++;
		else
			src++;
		pos++;
	}
	/* parse new line */
	pos++;
	*dest = '\0';
	LEAVE();
	return strlen(line_pos);
}

/**
 *  @brief This function duplicate a string
 *
 *  @param dst   A pointer to destination string
 *  @param src   A pointer to source string
 *
 *  @return      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static void woal_dup_string(char **dst, char *src)
{
	size_t len = 0;
	if (src) {
		len = strlen(src);
		if (len != 0) {
			if (*dst != NULL)
				kfree(*dst);
			*dst = kzalloc(len + 1, GFP_KERNEL);
			if (*dst == NULL) {
				PRINTM(MERROR,
				       "Failed to alloc mem for param: %s\n",
				       src);
				return;
			}
			moal_memcpy_ext(NULL, *dst, src, len, len);
		}
	}
}

/**
 *  @brief This function read an integer value in module parameter file
 *
 *  @param line     A pointer to a line
 *  @param out_data A pointer to parsed integer value
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status parse_line_read_int(t_u8 *line, int *out_data)
{
	t_u8 *p = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	if (line == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	p = strstr(line, "=");
	if (p == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	p++;
	ret = woal_atoi(out_data, p);
out:
	if (ret != MLAN_STATUS_SUCCESS)
		*out_data = 0;
	return ret;
}

/**
 *  @brief This function read a string in module parameter file
 *
 *  @param line     A pointer to a line
 *  @param out_str  A pointer to parsed string
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status parse_line_read_string(t_u8 *line, char **out_str)
{
	t_u8 *p = NULL, *pstr = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	if (line == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	p = strstr(line, "=");
	if (p == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	p++;
	pstr = p;
	while (*pstr) {
		if (*pstr == '\"')
			*pstr = '\0';
		pstr++;
	}
	if (*p == '\0')
		p++;
	*out_str = p;
out:
	return ret;
}

/**
 *  @brief This function read card info in module parameter file
 *
 *  @param line     A pointer to a line
 *  @param type     A pointer to card type
 *  @param if_id    A pointer to interface id
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status parse_line_read_card_info(t_u8 *line, char **type,
					     char **if_id)
{
	t_u8 *p = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	if (line == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}

	p = strstr(line, "=");
	if (p == NULL) {
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	*p = '\0';

	p = strstr(line, "_");
	if (p != NULL) {
		*p++ = '\0';
		*if_id = p;
	} else {
		*if_id = NULL;
	}
	*type = line;
out:
	return ret;
}

/**
 *  @brief This function read blocks in module parameter file
 *
 *  @param data     A pointer to a line
 *  @param size     line size
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status parse_cfg_read_block(t_u8 *data, t_u32 size,
					moal_handle *handle)
{
	int out_data = 0, end = 0;
	char *out_str = NULL;
	t_u8 line[MAX_LINE_LEN];
	moal_mod_para *params = &handle->params;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	while (parse_cfg_get_line(data, size, line) != -1) {
		if (strncmp(line, "}", strlen("}")) == 0) {
			end = 1;
			break;
		}
		if (end == 0 && strstr(line, "{") != 0)
			break;
		if (strncmp(line, "hw_test", strlen("hw_test")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_HW_TEST);
			else
				moal_extflg_clear(handle, EXT_HW_TEST);
			PRINTM(MMSG, "hw_test %s\n",
			       moal_extflg_isset(handle, EXT_HW_TEST) ? "on" :
									"off");
		}
#ifdef CONFIG_OF
		else if (strncmp(line, "dts_enable", strlen("dts_enable")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_DTS_ENABLE);
			else
				moal_extflg_clear(handle, EXT_DTS_ENABLE);
			PRINTM(MMSG, "dts_enable %s\n",
			       moal_extflg_isset(handle, EXT_DTS_ENABLE) ?
				       "on" :
				       "off");
		}
#endif
		else if (strncmp(line, "fw_name", strlen("fw_name")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->fw_name, out_str);
			PRINTM(MMSG, "fw_name=%s\n", params->fw_name);
		} else if (strncmp(line, "req_fw_nowait",
				   strlen("req_fw_nowait")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_REQ_FW_NOWAIT);
			else
				moal_extflg_clear(handle, EXT_REQ_FW_NOWAIT);
			PRINTM(MMSG, "req fw nowait %s\n",
			       moal_extflg_isset(handle, EXT_REQ_FW_NOWAIT) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "fw_reload", strlen("fw_reload")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->fw_reload = out_data;
			PRINTM(MMSG, "fw_reload %d\n", params->fw_reload);
		} else if (strncmp(line, "fw_serial", strlen("fw_serial")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_FW_SERIAL);
			else
				moal_extflg_clear(handle, EXT_FW_SERIAL);
			PRINTM(MMSG, "fw_serial %s\n",
			       moal_extflg_isset(handle, EXT_FW_SERIAL) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "mac_addr", strlen("mac_addr")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->mac_addr, out_str);
			PRINTM(MMSG, "mac_addr=%s\n", params->mac_addr);
		}
#ifdef MFG_CMD_SUPPORT
		else if (strncmp(line, "mfg_mode", strlen("mfg_mode")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->mfg_mode = out_data;
			PRINTM(MMSG, "mfg_mode = %d\n", params->mfg_mode);
		}
#endif
		else if (strncmp(line, "drv_mode", strlen("drv_mode")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->drv_mode = out_data;
			PRINTM(MMSG, "drv_mode = %d\n", params->drv_mode);
		}
#ifdef STA_SUPPORT
		else if (strncmp(line, "max_sta_bss", strlen("max_sta_bss")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->max_sta_bss = out_data;
			PRINTM(MMSG, "max_sta_bss = %d\n", params->max_sta_bss);
		} else if (strncmp(line, "sta_name", strlen("sta_name")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->sta_name, out_str);
			PRINTM(MMSG, "sta_name=%s\n", params->sta_name);
		}
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
		else if (strncmp(line, "max_uap_bss", strlen("max_uap_bss")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->max_uap_bss = out_data;
			PRINTM(MMSG, "max_uap_bss = %d\n", params->max_uap_bss);
		} else if (strncmp(line, "uap_name", strlen("uap_name")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->uap_name, out_str);
			PRINTM(MMSG, "uap_name=%s\n", params->uap_name);
		}
#endif /* UAP_SUPPORT */
#ifdef WIFI_DIRECT_SUPPORT
		else if (strncmp(line, "wfd_name", strlen("wfd_name")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->wfd_name, out_str);
			PRINTM(MMSG, "wfd_name=%s\n", params->wfd_name);
		}
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
		else if (strncmp(line, "max_vir_bss", strlen("max_vir_bss")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->max_vir_bss = out_data;
			PRINTM(MMSG, "max_vir_bss=%d\n", params->max_vir_bss);
		}
#endif
#endif
		else if (strncmp(line, "auto_ds", strlen("auto_ds")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->auto_ds = out_data;
			PRINTM(MMSG, "auto_ds = %d\n", params->auto_ds);
		} else if (strncmp(line, "ps_mode", strlen("ps_mode")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->ps_mode = out_data;
			PRINTM(MMSG, "ps_mode = %d\n", params->ps_mode);
		} else if (strncmp(line, "p2a_scan", strlen("p2a_scan")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->p2a_scan = out_data;
			PRINTM(MMSG, "p2a_scan = %d\n", params->p2a_scan);
		} else if (strncmp(line, "scan_chan_gap",
				   strlen("scan_chan_gap")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->scan_chan_gap = out_data;
			PRINTM(MMSG, "scan_chan_gap = %d\n",
			       params->scan_chan_gap);
		} else if (strncmp(line, "max_tx_buf", strlen("max_tx_buf")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->max_tx_buf = out_data;
			PRINTM(MMSG, "max_tx_buf = %d\n", params->max_tx_buf);
		}
#if defined(SDIO)
		else if (strncmp(line, "intmode", strlen("intmode")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_INTMODE);
			else
				moal_extflg_clear(handle, EXT_INTMODE);
			PRINTM(MMSG, "intmode %s\n",
			       moal_extflg_isset(handle, EXT_INTMODE) ? "on" :
									"off");
		} else if (strncmp(line, "gpiopin", strlen("gpiopin")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->gpiopin = out_data;
			PRINTM(MMSG, "gpiopin = %d\n", params->gpiopin);
		}
#endif
#if defined(SDIO) && defined(SDIO_SUSPEND_RESUME)
		else if (strncmp(line, "pm_keep_power",
				 strlen("pm_keep_power")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_PM_KEEP_POWER);
			else
				moal_extflg_clear(handle, EXT_PM_KEEP_POWER);
			PRINTM(MMSG, "pm_keep_power %s\n",
			       moal_extflg_isset(handle, EXT_PM_KEEP_POWER) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "shutdown_hs",
				   strlen("shutdown_hs")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_SHUTDOWN_HS);
			else
				moal_extflg_clear(handle, EXT_SHUTDOWN_HS);
			PRINTM(MMSG, "shutdown_hs %s\n",
			       moal_extflg_isset(handle, EXT_SHUTDOWN_HS) ?
				       "on" :
				       "off");
		}
#endif
#if defined(STA_SUPPORT)
		else if (strncmp(line, "cfg_11d", strlen("cfg_11d")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->cfg_11d = out_data;
			PRINTM(MMSG, "cfg_11d = %d\n", params->cfg_11d);
		}
#endif
#if defined(SDIO)
		else if (strncmp(line, "slew_rate", strlen("slew_rate")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->slew_rate = out_data;
			PRINTM(MMSG, "slew_rate = %d\n", params->slew_rate);
		}
#endif
		else if (strncmp(line, "dpd_data_cfg",
				 strlen("dpd_data_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->dpd_data_cfg, out_str);
			PRINTM(MMSG, "dpd_data_cfg=%s\n", params->dpd_data_cfg);
		} else if (strncmp(line, "init_cfg", strlen("init_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->init_cfg, out_str);
			PRINTM(MMSG, "init_cfg=%s\n", params->init_cfg);
		} else if (strncmp(line, "cal_data_cfg",
				   strlen("cal_data_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->cal_data_cfg, out_str);
			PRINTM(MMSG, "cal_data_cfg=%s\n", params->cal_data_cfg);
		} else if (strncmp(line, "txpwrlimit_cfg",
				   strlen("txpwrlimit_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->txpwrlimit_cfg, out_str);
			PRINTM(MMSG, "txpwrlimit_cfg=%s\n",
			       params->txpwrlimit_cfg);
		} else if (strncmp(line, "cntry_txpwr",
				   strlen("cntry_txpwr")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_CNTRY_TXPWR);
			else
				moal_extflg_clear(handle, EXT_CNTRY_TXPWR);
			PRINTM(MMSG, "cntry_txpwr %s\n",
			       moal_extflg_isset(handle, EXT_CNTRY_TXPWR) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "init_hostcmd_cfg",
				   strlen("init_hostcmd_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->init_hostcmd_cfg, out_str);
			PRINTM(MMSG, "init_hostcmd_cfg=%s\n",
			       params->init_hostcmd_cfg);
		} else if (strncmp(line, "band_steer_cfg",
				   strlen("band_steer_cfg")) == 0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->band_steer_cfg, out_str);
			PRINTM(MMSG, "band_steer_cfg=%s\n",
			       params->band_steer_cfg);
		} else if (strncmp(line, "cfg80211_wext",
				   strlen("cfg80211_wext")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->cfg80211_wext = out_data;
			PRINTM(MMSG, "cfg80211_wext=0x%x\n",
			       params->cfg80211_wext);
		}
#if defined(USB)
		else if (IS_USB(handle->card_type) &&
			 strncmp(line, "skip_fwdnld", strlen("skip_fwdnld")) ==
				 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_SKIP_FWDNLD);
			else
				moal_extflg_clear(handle, EXT_SKIP_FWDNLD);
			PRINTM(MMSG, "skip_fwdnld %s\n",
			       moal_extflg_isset(handle, EXT_SKIP_FWDNLD) ?
				       "on" :
				       "off");
		}
#endif
		else if (strncmp(line, "wq_sched_prio",
				 strlen("wq_sched_prio")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->wq_sched_prio = out_data;
			PRINTM(MMSG, "wq_sched_prio=0x%x\n",
			       params->wq_sched_prio);
		} else if (strncmp(line, "wq_sched_policy",
				   strlen("wq_sched_policy")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->wq_sched_policy = out_data;
			PRINTM(MMSG, "wq_sched_policy=0x%x\n",
			       params->wq_sched_policy);
		} else if (strncmp(line, "rx_work", strlen("rx_work")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->rx_work = out_data;
			PRINTM(MMSG, "rx_work=0x%x\n", params->rx_work);
		} else if (strncmp(line, "aggrctrl", strlen("aggrctrl")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_AGGR_CTRL);
			else
				moal_extflg_clear(handle, EXT_AGGR_CTRL);
			PRINTM(MMSG, "aggrctrl %s\n",
			       moal_extflg_isset(handle, EXT_AGGR_CTRL) ?
				       "on" :
				       "off");
		}
#ifdef USB
		else if (IS_USB(handle->card_type) &&
			 strncmp(line, "usb_aggr", strlen("usb_aggr")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->usb_aggr = out_data;
			PRINTM(MMSG, "usb_aggr=0x%x\n", params->usb_aggr);
		}
#endif
#ifdef PCIE
		else if (IS_PCIE(handle->card_type) &&
			 strncmp(line, "pcie_int_mode",
				 strlen("pcie_int_mode")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->pcie_int_mode = out_data;
			PRINTM(MMSG, "pcie_int_mode=%d\n",
			       params->pcie_int_mode);
		}
#endif
		else if (strncmp(line, "low_power_mode_enable",
				 strlen("low_power_mode_enable")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_LOW_PW_MODE);
			else
				moal_extflg_clear(handle, EXT_LOW_PW_MODE);
			PRINTM(MMSG, "low_power_mode_enable %s\n",
			       moal_extflg_isset(handle, EXT_LOW_PW_MODE) ?
				       "on" :
				       "off");
		}
#ifdef ANDROID_KERNEL
		else if (strncmp(line, "wakelock_timeout",
				 strlen("wakelock_timeout")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->wakelock_timeout = out_data;
			PRINTM(MMSG, "wakelock_timeout=%d\n",
			       params->wakelock_timeout);
		}
#endif
		else if (strncmp(line, "dev_cap_mask",
				 strlen("dev_cap_mask")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->dev_cap_mask = out_data;
			PRINTM(MMSG, "dev_cap_mask=%d\n", params->dev_cap_mask);
		}
#ifdef SDIO
		else if (strncmp(line, "sdio_rx_aggr",
				 strlen("sdio_rx_aggr")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_SDIO_RX_AGGR);
			else
				moal_extflg_clear(handle, EXT_SDIO_RX_AGGR);
			PRINTM(MMSG, "sdio_rx_aggr %s\n",
			       moal_extflg_isset(handle, EXT_SDIO_RX_AGGR) ?
				       "on" :
				       "off");
		}
#endif
#if defined(SD8997) || defined(PCIE8997) || defined(USB8997) ||                \
	defined(SD8977) || defined(SD8987) || defined(SD9098) ||               \
	defined(USB9098) || defined(PCIE9098) || defined(SD9097) ||            \
	defined(USB9097) || defined(PCIE9097) || defined(SD8978) ||            \
	defined(USB8978)
		else if (strncmp(line, "pmic", strlen("pmic")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_PMIC);
			else
				moal_extflg_clear(handle, EXT_PMIC);
			PRINTM(MMSG, "pmic %s\n",
			       moal_extflg_isset(handle, EXT_PMIC) ? "on" :
								     "off");
		}
#endif
		else if (strncmp(line, "antcfg", strlen("antcfg")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->antcfg = out_data;
			PRINTM(MMSG, "antcfg=%d\n", params->antcfg);
		} else if (strncmp(line, "uap_oper_ctrl",
				   strlen("uap_oper_ctrl")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->uap_oper_ctrl = out_data;
			PRINTM(MMSG, "uap_oper_ctrl=%d\n",
			       params->uap_oper_ctrl);
		} else if (strncmp(line, "hs_wake_interval",
				   strlen("hs_wake_interval")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->hs_wake_interval = out_data;
			PRINTM(MMSG, "hs_wake_interval=%d\n",
			       params->hs_wake_interval);
		} else if (strncmp(line, "indication_gpio",
				   strlen("indication_gpio")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->indication_gpio = out_data;
			PRINTM(MMSG, "indication_gpio=%d\n",
			       params->indication_gpio);
		} else if (strncmp(line, "disconnect_on_suspend",
				   strlen("disconnect_on_suspend")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle,
						EXT_DISCONNECT_ON_SUSPEND);
			else
				moal_extflg_clear(handle,
						  EXT_DISCONNECT_ON_SUSPEND);
			PRINTM(MMSG, "disconnect_on_suspend %s\n",
			       moal_extflg_isset(handle,
						 EXT_DISCONNECT_ON_SUSPEND) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "hs_mimo_switch",
				   strlen("hs_mimo_switch")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_HS_MIMO_SWITCH);
			else
				moal_extflg_clear(handle, EXT_HS_MIMO_SWITCH);
			PRINTM(MMSG, "hs_mimo_switch %s\n",
			       moal_extflg_isset(handle, EXT_HS_MIMO_SWITCH) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "indrstcfg", strlen("indrstcfg")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->indrstcfg = out_data;
			PRINTM(MMSG, "indrstcfg=%d\n", params->indrstcfg);
		} else if (strncmp(line, "fixed_beacon_buffer",
				   strlen("fixed_beacon_buffer")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_FIX_BCN_BUF);
			else
				moal_extflg_clear(handle, EXT_FIX_BCN_BUF);
			PRINTM(MMSG, "fixed_beacon_buffer %s\n",
			       moal_extflg_isset(handle, EXT_FIX_BCN_BUF) ?
				       "on" :
				       "off");
		}
#ifdef WIFI_DIRECT_SUPPORT
		else if (strncmp(line, "GoAgeoutTime",
				 strlen("GoAgeoutTime")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->GoAgeoutTime = out_data;
			PRINTM(MMSG, "GoAgeoutTime=%d\n", params->GoAgeoutTime);
		}
#endif
		else if (strncmp(line, "gtk_rekey_offload",
				 strlen("gtk_rekey_offload")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->gtk_rekey_offload = out_data;
			PRINTM(MMSG, "gtk_rekey_offload=%d\n",
			       params->gtk_rekey_offload);
		} else if (strncmp(line, "multi_dtim", strlen("multi_dtim")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->multi_dtim = out_data;
			PRINTM(MMSG, "multi_dtim=%d\n", params->multi_dtim);
		} else if (strncmp(line, "inact_tmo", strlen("inact_tmo")) ==
			   0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->inact_tmo = out_data;
			PRINTM(MMSG, "inact_tmo=%d\n", params->inact_tmo);
		} else if (strncmp(line, "napi", strlen("napi")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_NAPI);
			else
				moal_extflg_clear(handle, EXT_NAPI);
			PRINTM(MMSG, "napi %s\n",
			       moal_extflg_isset(handle, EXT_NAPI) ? "on" :
								     "off");
		} else if (strncmp(line, "tx_work", strlen("tx_work")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_TX_WORK);
			else
				moal_extflg_clear(handle, EXT_TX_WORK);
			PRINTM(MMSG, "tx_work %s\n",
			       moal_extflg_isset(handle, EXT_TX_WORK) ? "on" :
									"off");
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		else if (strncmp(line, "dfs_offload", strlen("dfs_offload")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_DFS_OFFLOAD);
			else
				moal_extflg_clear(handle, EXT_DFS_OFFLOAD);
			PRINTM(MMSG, "dfs_offload %s\n",
			       moal_extflg_isset(handle, EXT_DFS_OFFLOAD) ?
				       "on" :
				       "off");
		}
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		else if (strncmp(line, "disable_regd_by_driver",
				 strlen("disable_regd_by_driver")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle,
						EXT_DISABLE_REGD_BY_DRIVER);
			else
				moal_extflg_clear(handle,
						  EXT_DISABLE_REGD_BY_DRIVER);
			PRINTM(MMSG, "reg domain set by driver=%s\n",
			       moal_extflg_isset(handle,
						 EXT_DISABLE_REGD_BY_DRIVER) ?
				       "enable" :
				       "disable");
		} else if (strncmp(line, "reg_alpha2", strlen("reg_alpha2")) ==
			   0) {
			if (parse_line_read_string(line, &out_str) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			woal_dup_string(&params->reg_alpha2, out_str);
			PRINTM(MMSG, "reg_alpha2=%s\n", params->reg_alpha2);
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		else if (strncmp(line, "country_ie_ignore",
				 strlen("country_ie_ignore")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_COUNTRY_IE_IGNORE);
			else
				moal_extflg_clear(handle,
						  EXT_COUNTRY_IE_IGNORE);
			PRINTM(MMSG, "country_ie_ignore=%s\n",
			       moal_extflg_isset(handle,
						 EXT_COUNTRY_IE_IGNORE) ?
				       "on" :
				       "off");
		} else if (strncmp(line, "beacon_hints",
				   strlen("beacon_hints")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_BEACON_HINTS);
			else
				moal_extflg_clear(handle, EXT_BEACON_HINTS);
			PRINTM(MMSG, "beacon_hints=%s\n",
			       moal_extflg_isset(handle, EXT_BEACON_HINTS) ?
				       "enable" :
				       "disable");
		}
#endif
#endif
#ifdef UAP_SUPPORT
		else if (strncmp(line, "uap_max_sta", strlen("uap_max_sta")) ==
			 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->uap_max_sta = out_data;
			PRINTM(MMSG, "uap_max_sta=%d\n", params->uap_max_sta);
		}
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		else if (strncmp(line, "host_mlme", strlen("host_mlme")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			if (out_data)
				moal_extflg_set(handle, EXT_HOST_MLME);
			else
				moal_extflg_clear(handle, EXT_HOST_MLME);
			PRINTM(MMSG, "host_mlme=%s\n",
			       moal_extflg_isset(handle, EXT_HOST_MLME) ?
				       "enable" :
				       "disable");
		}
#endif
#endif
		else if (strncmp(line, "dfs53cfg", strlen("dfs53cfg")) == 0) {
			if (parse_line_read_int(line, &out_data) !=
			    MLAN_STATUS_SUCCESS)
				goto err;
			params->dfs53cfg = out_data;
			PRINTM(MMSG, "dfs53cfg= %d\n", params->dfs53cfg);
		}
	}
	if (end)
		return ret;
err:
	PRINTM(MMSG, "Invalid line: %s\n", line);
	ret = MLAN_STATUS_FAILURE;
	return ret;
}

/**
 *  @brief This function initialize module parameter
 *
 *  @param handle   A pointer to moal_handle structure
 *  @param params   A pointer to moal_mod_para structure
 *
 *  @return         N/A
 */
static void woal_setup_module_param(moal_handle *handle, moal_mod_para *params)
{
	if (hw_test)
		moal_extflg_set(handle, EXT_HW_TEST);
#ifdef CONFIG_OF
	if (dts_enable)
		moal_extflg_set(handle, EXT_DTS_ENABLE);
#endif
	woal_dup_string(&handle->params.fw_name, fw_name);
	if (params && params->fw_name)
		woal_dup_string(&handle->params.fw_name, params->fw_name);
	if (req_fw_nowait)
		moal_extflg_set(handle, EXT_REQ_FW_NOWAIT);
	handle->params.fw_reload = fw_reload;
	if (fw_reload == FW_RELOAD_WITH_EMULATION) {
		if (!IS_USB(handle->card_type))
			handle->params.fw_reload = 0;
		else
			fw_reload = 0;
	}
	if (params)
		handle->params.fw_reload = params->fw_reload;
	if (fw_serial)
		moal_extflg_set(handle, EXT_FW_SERIAL);
	woal_dup_string(&handle->params.mac_addr, mac_addr);
	if (params && params->mac_addr)
		woal_dup_string(&handle->params.mac_addr, params->mac_addr);
#ifdef MFG_CMD_SUPPORT
	handle->params.mfg_mode = mfg_mode;
	if (params)
		handle->params.mfg_mode = params->mfg_mode;
#endif
	handle->params.drv_mode = drv_mode;
	if (params)
		handle->params.drv_mode = params->drv_mode;
#ifdef STA_SUPPORT
	handle->params.max_sta_bss = max_sta_bss;
	woal_dup_string(&handle->params.sta_name, sta_name);
	if (params) {
		handle->params.max_sta_bss = params->max_sta_bss;
		woal_dup_string(&handle->params.sta_name, params->sta_name);
	}
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
	handle->params.max_uap_bss = max_uap_bss;
	woal_dup_string(&handle->params.uap_name, uap_name);
	handle->params.uap_max_sta = uap_max_sta;
	if (params) {
		handle->params.max_uap_bss = params->max_uap_bss;
		woal_dup_string(&handle->params.uap_name, params->uap_name);
		handle->params.uap_max_sta = params->uap_max_sta;
	}
#endif /* UAP_SUPPORT */
#ifdef WIFI_DIRECT_SUPPORT
	handle->params.max_wfd_bss = max_wfd_bss;
	woal_dup_string(&handle->params.wfd_name, wfd_name);
	if (params) {
		handle->params.max_wfd_bss = params->max_wfd_bss;
		woal_dup_string(&handle->params.wfd_name, params->wfd_name);
	}
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
	handle->params.max_vir_bss = max_vir_bss;
	if (params)
		handle->params.max_vir_bss = params->max_vir_bss;
#endif
#endif /* WIFI_DIRECT_SUPPORT */
	handle->params.auto_ds = auto_ds;
	if (params)
		handle->params.auto_ds = params->auto_ds;
	handle->params.ps_mode = ps_mode;
	handle->params.p2a_scan = p2a_scan;
	handle->params.scan_chan_gap = scan_chan_gap;
	handle->params.max_tx_buf = max_tx_buf;
	if (params) {
		handle->params.ps_mode = params->ps_mode;
		handle->params.max_tx_buf = params->max_tx_buf;
		handle->params.p2a_scan = params->p2a_scan;
		handle->params.scan_chan_gap = params->scan_chan_gap;
	}
#if defined(SDIO)
	if (intmode)
		moal_extflg_set(handle, EXT_INTMODE);
	handle->params.gpiopin = gpiopin;
	if (params)
		handle->params.gpiopin = params->gpiopin;
#endif
#if defined(SDIO) && defined(SDIO_SUSPEND_RESUME)
	if (pm_keep_power)
		moal_extflg_set(handle, EXT_PM_KEEP_POWER);
	if (shutdown_hs)
		moal_extflg_set(handle, EXT_SHUTDOWN_HS);
#endif
#if defined(STA_SUPPORT)
	handle->params.cfg_11d = cfg_11d;
	if (params)
		handle->params.cfg_11d = params->cfg_11d;
#endif
#if defined(SDIO)
	handle->params.slew_rate = slew_rate;
#endif
	woal_dup_string(&handle->params.dpd_data_cfg, dpd_data_cfg);
	if (params)
		woal_dup_string(&handle->params.dpd_data_cfg,
				params->dpd_data_cfg);
	woal_dup_string(&handle->params.init_cfg, init_cfg);
	woal_dup_string(&handle->params.cal_data_cfg, cal_data_cfg);
	if (params) {
		woal_dup_string(&handle->params.init_cfg, params->init_cfg);
		woal_dup_string(&handle->params.cal_data_cfg,
				params->cal_data_cfg);
	}
	woal_dup_string(&handle->params.txpwrlimit_cfg, txpwrlimit_cfg);
	if (params)
		woal_dup_string(&handle->params.txpwrlimit_cfg,
				params->txpwrlimit_cfg);
	if (cntry_txpwr)
		moal_extflg_set(handle, EXT_CNTRY_TXPWR);
	woal_dup_string(&handle->params.init_hostcmd_cfg, init_hostcmd_cfg);
	if (params)
		woal_dup_string(&handle->params.init_hostcmd_cfg,
				params->init_hostcmd_cfg);

	woal_dup_string(&handle->params.band_steer_cfg, band_steer_cfg);
	if (params)
		woal_dup_string(&handle->params.band_steer_cfg,
				params->band_steer_cfg);
	handle->params.cfg80211_wext = cfg80211_wext;
	if (params)
		handle->params.cfg80211_wext = params->cfg80211_wext;
#if defined(USB)
	if (skip_fwdnld)
		moal_extflg_set(handle, EXT_SKIP_FWDNLD);
#endif
	handle->params.wq_sched_prio = wq_sched_prio;
	handle->params.wq_sched_policy = wq_sched_policy;
	handle->params.rx_work = rx_work;
	if (params) {
		handle->params.wq_sched_prio = params->wq_sched_prio;
		handle->params.wq_sched_policy = params->wq_sched_policy;
		handle->params.rx_work = params->rx_work;
	}
	if (aggrctrl)
		moal_extflg_set(handle, EXT_AGGR_CTRL);
#ifdef USB
	handle->params.usb_aggr = usb_aggr;
	if (params)
		handle->params.usb_aggr = params->usb_aggr;
#endif
#ifdef PCIE
	handle->params.pcie_int_mode = pcie_int_mode;
	if (params)
		handle->params.pcie_int_mode = params->pcie_int_mode;
#endif /* PCIE */
	if (low_power_mode_enable)
		moal_extflg_set(handle, EXT_LOW_PW_MODE);

#ifdef ANDROID_KERNEL
	handle->params.wakelock_timeout = wakelock_timeout;
	if (params)
		handle->params.wakelock_timeout = params->wakelock_timeout;
#endif
	handle->params.dev_cap_mask = dev_cap_mask;
	if (params)
		handle->params.dev_cap_mask = params->dev_cap_mask;
#ifdef SDIO
	if (sdio_rx_aggr)
		moal_extflg_set(handle, EXT_SDIO_RX_AGGR);
#endif
#if defined(SD8997) || defined(PCIE8997) || defined(USB8997) ||                \
	defined(SD8977) || defined(SD8987) || defined(SD9098) ||               \
	defined(USB9098) || defined(PCIE9098) || defined(SD9097) ||            \
	defined(USB9097) || defined(PCIE9097) || defined(SD8978) ||            \
	defined(USB8978)
	if (pmic)
		moal_extflg_set(handle, EXT_PMIC);
#endif
	handle->params.antcfg = antcfg;
	if (params)
		handle->params.antcfg = params->antcfg;
	handle->params.uap_oper_ctrl = uap_oper_ctrl;
	if (params)
		handle->params.uap_oper_ctrl = params->uap_oper_ctrl;
	handle->params.hs_wake_interval = hs_wake_interval;
	handle->params.indication_gpio = indication_gpio;
	if (params) {
		handle->params.hs_wake_interval = params->hs_wake_interval;
		handle->params.indication_gpio = params->indication_gpio;
	}
	if (disconnect_on_suspend)
		moal_extflg_set(handle, EXT_DISCONNECT_ON_SUSPEND);
	if (hs_mimo_switch)
		moal_extflg_set(handle, EXT_HS_MIMO_SWITCH);
	handle->params.indrstcfg = indrstcfg;
	if (params)
		handle->params.indrstcfg = params->indrstcfg;
	if (fixed_beacon_buffer)
		moal_extflg_set(handle, EXT_FIX_BCN_BUF);
#ifdef WIFI_DIRECT_SUPPORT
	handle->params.GoAgeoutTime = GoAgeoutTime;
	if (params)
		handle->params.GoAgeoutTime = params->GoAgeoutTime;
#endif
	handle->params.gtk_rekey_offload = gtk_rekey_offload;
	if (params)
		handle->params.gtk_rekey_offload = params->gtk_rekey_offload;
	handle->params.multi_dtim = multi_dtim;
	handle->params.inact_tmo = inact_tmo;
	if (params) {
		handle->params.multi_dtim = params->multi_dtim;
		handle->params.inact_tmo = params->inact_tmo;
	}
	if (napi)
		moal_extflg_set(handle, EXT_NAPI);
	if (tx_work)
		moal_extflg_set(handle, EXT_TX_WORK);

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (dfs_offload)
		moal_extflg_set(handle, EXT_DFS_OFFLOAD);
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	if (host_mlme)
		moal_extflg_set(handle, EXT_HOST_MLME);
#endif
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
	if (disable_regd_by_driver)
		moal_extflg_set(handle, EXT_DISABLE_REGD_BY_DRIVER);
	if (reg_alpha2)
		woal_dup_string(&handle->params.reg_alpha2, reg_alpha2);
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
	if (country_ie_ignore)
		moal_extflg_set(handle, EXT_COUNTRY_IE_IGNORE);
	if (beacon_hints)
		moal_extflg_set(handle, EXT_BEACON_HINTS);
#endif
#endif
	if (params && params->reg_alpha2)
		woal_dup_string(&handle->params.reg_alpha2, params->reg_alpha2);
	if (params)
		moal_memcpy_ext(handle, handle->params.ext_flgs,
				params->ext_flgs, sizeof(params->ext_flgs),
				sizeof(handle->params.ext_flgs));

		/* do some special handle for MFG mode */
#ifdef MFG_CMD_SUPPORT
	if (handle->params.mfg_mode) {
#if defined(STA_WEXT) || defined(UAP_WEXT)
		handle->params.cfg80211_wext = STA_WEXT_MASK | UAP_WEXT_MASK;
#else
		handle->params.cfg80211_wext = 0;
#endif
		handle->params.drv_mode = DRV_MODE_STA;
	}
#endif
	if (dfs53cfg > DFS_W53_OLD) {
		PRINTM(MERROR, "Invalid value for dfs53cfg !\n");
		handle->params.dfs53cfg = DFS_W53_DEFAULT_FW;
	} else {
		handle->params.dfs53cfg = dfs53cfg;
		if (params)
			handle->params.dfs53cfg = params->dfs53cfg;
	}
}

/**
 *  @brief This function free module parameter memory
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         N/A
 */
void woal_free_module_param(moal_handle *handle)
{
	moal_mod_para *params = &handle->params;
	PRINTM(MMSG, "Free module params\n");
	if (params->fw_name) {
		kfree(params->fw_name);
		params->fw_name = NULL;
	}
	if (params->mac_addr) {
		kfree(params->mac_addr);
		params->mac_addr = NULL;
	}
#ifdef STA_SUPPORT
	if (params->sta_name) {
		kfree(params->sta_name);
		params->sta_name = NULL;
	}
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
	if (params->uap_name) {
		kfree(params->uap_name);
		params->uap_name = NULL;
	}
#endif /* UAP_SUPPORT */
#ifdef WIFI_DIRECT_SUPPORT
	if (params->wfd_name) {
		kfree(params->wfd_name);
		params->wfd_name = NULL;
	}
#endif /* WIFI_DIRECT_SUPPORT */
	if (params->dpd_data_cfg) {
		kfree(params->dpd_data_cfg);
		params->dpd_data_cfg = NULL;
	}
	if (params->init_cfg) {
		kfree(params->init_cfg);
		params->init_cfg = NULL;
	}
	if (params->cal_data_cfg) {
		kfree(params->cal_data_cfg);
		params->cal_data_cfg = NULL;
	}
	if (params->txpwrlimit_cfg) {
		kfree(params->txpwrlimit_cfg);
		params->txpwrlimit_cfg = NULL;
	}
	if (params->init_hostcmd_cfg) {
		kfree(params->init_hostcmd_cfg);
		params->init_hostcmd_cfg = NULL;
	}
	if (params->band_steer_cfg) {
		kfree(params->band_steer_cfg);
		params->band_steer_cfg = NULL;
	}
	if (params->reg_alpha2) {
		kfree(params->reg_alpha2);
		params->reg_alpha2 = NULL;
	}
}

/**
 *  @brief This function request module parameter data from user space
 *
 *  @param handle   A pointer to moal_handle structure
 *  @param mod_file A pointer to module parameter file path
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status woal_req_mod_param(moal_handle *handle, char *mod_file)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	struct device *dev = handle->hotplug_device;

	if (dev == NULL) {
		PRINTM(MERROR, "No device attached\n");
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}

	ret = request_firmware(&handle->param_data, mod_file, dev);
	if (ret < 0)
		PRINTM(MERROR, "Request firmware: %s failed, error: %d\n",
		       mod_file, ret);
out:
	return ret;
}

#ifdef CONFIG_OF
/**
 *  @brief This function read the initial parameter from device tress
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         N/A
 */
void woal_init_from_dev_tree(void)
{
	struct device_node *dt_node = NULL;
	struct property *prop;
	t_u32 data;
	const char *string_data;

	ENTER();

	if (!dts_enable) {
		PRINTM(MIOCTL, "DTS is disabled!");
		return;
	}

	dt_node = of_find_node_by_name(NULL, "sdxxx-wlan");
	if (!dt_node) {
		LEAVE();
		return;
	}
	for_each_property_of_node (dt_node, prop) {
		if (!strncmp(prop->name, "drv_mode", strlen("drv_mode"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "drv_mode=0x%x\n", data);
				drv_mode = data;
			}
		}
#ifdef DEBUG_LEVEL1
		else if (!strncmp(prop->name, "drvdbg", strlen("drvdbg"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "drvdbg=0x%x\n", data);
				drvdbg = data;
			}
		}
#endif
		else if (!strncmp(prop->name, "dev_cap_mask",
				  strlen("dev_cap_mask"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "dev_cap_mask=0x%x\n", data);
				dev_cap_mask = data;
			}
		} else if (!strncmp(prop->name, "hw_test", strlen("hw_test"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "hw_test=0x%x\n", data);
				hw_test = data;
			}
		}
#if defined(SDIO)
		else if (!strncmp(prop->name, "slew_rate",
				  strlen("slew_rate"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "slew_rate=0x%x\n", data);
				slew_rate = data;
			}
		}
#endif
		else if (!strncmp(prop->name, "tx_work", strlen("tx_work"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "tx_work=0x%x\n", data);
				tx_work = data;
			}
		}
#ifdef MFG_CMD_SUPPORT
		else if (!strncmp(prop->name, "mfg_mode", strlen("mfg_mode"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "mfg_mode=0x%x\n", data);
				mfg_mode = data;
			}
		}
#endif
		else if (!strncmp(prop->name, "mac_addr", strlen("mac_addr"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				mac_addr = (char *)string_data;
				PRINTM(MIOCTL, "mac_addr=%s\n", mac_addr);
			}
		} else if (!strncmp(prop->name, "fw_name", strlen("fw_name"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				fw_name = (char *)string_data;
				PRINTM(MIOCTL, "fw_name=%s\n", fw_name);
			}
		}
#if defined(STA_WEXT) || defined(UAP_WEXT)
		else if (!strncmp(prop->name, "cfg80211_wext",
				  strlen("cfg80211_wext"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "cfg80211_wext=0x%x\n", data);
				cfg80211_wext = data;
			}
		}
#endif
#ifdef STA_SUPPORT
		else if (!strncmp(prop->name, "sta_name", strlen("sta_name"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				sta_name = (char *)string_data;
				PRINTM(MIOCTL, "sta_name=%s\n", sta_name);
			}
		}
#endif
#ifdef WIFI_DIRECT_SUPPORT
		else if (!strncmp(prop->name, "wfd_name", strlen("wfd_name"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				wfd_name = (char *)string_data;
				PRINTM(MIOCTL, "wfd_name=%s\n", wfd_name);
			}
		}
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
		else if (!strncmp(prop->name, "disable_regd_by_driver",
				  strlen("disable_regd_by_driver"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "disable_regd_by_driver=0x%x\n",
				       data);
				disable_regd_by_driver = data;
			}
		} else if (!strncmp(prop->name, "reg_alpha2",
				    strlen("reg_alpha2"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				reg_alpha2 = (char *)string_data;
				PRINTM(MIOCTL, "reg_alpha2=%s\n", reg_alpha2);
			}
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		else if (!strncmp(prop->name, "country_ie_ignore",
				  strlen("country_ie_ignore"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "country_ie_ignore=0x%x\n",
				       data);
				country_ie_ignore = data;
			}
		} else if (!strncmp(prop->name, "beacon_hints",
				    strlen("beacon_hints"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "beacon_hints=0x%x\n", data);
				beacon_hints = data;
			}
		}
#endif
#endif
#ifdef WIFI_DIRECT_SUPPORT
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
		else if (!strncmp(prop->name, "max_vir_bss",
				  strlen("max_vir_bss"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "max_vir_bss=0x%x\n", data);
				max_vir_bss = data;
			}
		}
#endif
#endif
		else if (!strncmp(prop->name, "dpd_data_cfg",
				  strlen("dpd_data_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				dpd_data_cfg = (char *)string_data;
				PRINTM(MIOCTL, "dpd_data_cfg=%s\n",
				       dpd_data_cfg);
			}
		} else if (!strncmp(prop->name, "init_cfg",
				    strlen("init_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				init_cfg = (char *)string_data;
				PRINTM(MIOCTL, "init_cfg=%s\n", init_cfg);
			}
		} else if (!strncmp(prop->name, "cal_data_cfg",
				    strlen("cal_data_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				cal_data_cfg = (char *)string_data;
				PRINTM(MIOCTL, "cal_data_cfg=%s\n",
				       cal_data_cfg);
			}
		} else if (!strncmp(prop->name, "txpwrlimit_cfg",
				    strlen("txpwrlimit_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				txpwrlimit_cfg = (char *)string_data;
				PRINTM(MIOCTL, "txpwrlimit_cfg=%s\n",
				       txpwrlimit_cfg);
			}
		} else if (!strncmp(prop->name, "cntry_txpwr",
				    strlen("cntry_txpwr"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				cntry_txpwr = data;
				PRINTM(MIOCTL, "cntry_txpwr=%d\n", cntry_txpwr);
			}
		} else if (!strncmp(prop->name, "init_hostcmd_cfg",
				    strlen("init_hostcmd_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				init_hostcmd_cfg = (char *)string_data;
				PRINTM(MIOCTL, "init_hostcmd_cfg=%s\n",
				       init_hostcmd_cfg);
			}
		} else if (!strncmp(prop->name, "band_steer_cfg",
				    strlen("band_steer_cfg"))) {
			if (!of_property_read_string(dt_node, prop->name,
						     &string_data)) {
				band_steer_cfg = (char *)string_data;
				PRINTM(MIOCTL, "band_steer_cfg=%s\n",
				       band_steer_cfg);
			}
		} else if (!strncmp(prop->name, "pmic", strlen("pmic"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				pmic = data;
				PRINTM(MIOCTL, "pmic=%d\n", pmic);
			}
		} else if (!strncmp(prop->name, "antcfg", strlen("antcfg"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				antcfg = data;
				PRINTM(MIOCTL, "antcfg=%d\n", antcfg);
			}
		} else if (!strncmp(prop->name, "hs_wake_interval",
				    strlen("hs_wake_interval"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				hs_wake_interval = data;
				PRINTM(MIOCTL, "hs_wake_interval=%d\n",
				       hs_wake_interval);
			}
		} else if (!strncmp(prop->name, "indication_gpio",
				    strlen("indication_gpio"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				indication_gpio = (t_u8)data;
				PRINTM(MIOCTL, "indication_gpio=%d\n",
				       indication_gpio);
			}
		} else if (!strncmp(prop->name, "hs_mimo_switch",
				    strlen("hs_mimo_switch"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				hs_mimo_switch = data;
				PRINTM(MIOCTL, "hs_mimo_switch=%d\n",
				       hs_mimo_switch);
			}
		}
#ifdef WIFI_DIRECT_SUPPORT
		else if (!strncmp(prop->name, "GoAgeoutTime",
				  strlen("GoAgeoutTime"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				GoAgeoutTime = data;
				PRINTM(MIOCTL, "GoAgeoutTime=%d\n",
				       GoAgeoutTime);
			}
		}
#endif
		else if (!strncmp(prop->name, "indrstcfg",
				  strlen("indrstcfg"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				indrstcfg = data;
				PRINTM(MIOCTL, "indrstcfg=%d\n", indrstcfg);
			}
		} else if (!strncmp(prop->name, "fixed_beacon_buffer",
				    strlen("fixed_beacon_buffer"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				fixed_beacon_buffer = data;
				PRINTM(MIOCTL, "fixed_beacon_buffer=%d\n",
				       fixed_beacon_buffer);
			}
		} else if (!strncmp(prop->name, "multi_dtim",
				    strlen("multi_dtim"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				multi_dtim = data;
				PRINTM(MIOCTL, "multi_dtim=%d\n", multi_dtim);
			}
		} else if (!strncmp(prop->name, "inact_tmo",
				    strlen("inact_tmo"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				inact_tmo = data;
				PRINTM(MIOCTL, "inact_tmo=%d\n", inact_tmo);
			}
		}
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
		else if (!strncmp(prop->name, "dfs_offload",
				  strlen("dfs_offload"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				dfs_offload = data;
				PRINTM(MIOCTL, "dfs_offload=%d\n", dfs_offload);
			}
		}
#endif
		else if (!strncmp(prop->name, "gtk_rekey_offload",
				  strlen("gtk_rekey_offload"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				gtk_rekey_offload = data;
				PRINTM(MIOCTL, "gtk_rekey_offload=%d\n",
				       gtk_rekey_offload);
			}
		}
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
		else if (!strncmp(prop->name, "host_mlme",
				  strlen("host_mlme"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MIOCTL, "host_mlme=0x%x\n", data);
				host_mlme = data;
			}
		}
#endif
#endif
#ifdef UAP_SUPPORT
		else if (!strncmp(prop->name, "uap_max_sta",
				  strlen("uap_max_sta"))) {
			if (!of_property_read_u32(dt_node, prop->name, &data)) {
				PRINTM(MERROR, "uap_max_sta=0x%x\n", data);
				uap_max_sta = data;
			}
		}
#endif
	}
	LEAVE();
	return;
}
#endif

/**
 *  @brief This function check if configuration block id could be used
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_validate_cfg_id(moal_handle *handle)
{
	int i;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	for (i = 0; i < MAX_MLAN_ADAPTER; i++) {
		if (m_handle[i] == NULL || m_handle[i] == handle)
			continue;
		if (m_handle[i]->card_type == handle->card_type) {
			if (m_handle[i]->blk_id == handle->blk_id) {
				ret = MLAN_STATUS_FAILURE;
			}
		}
	}
	return ret;
}

/**
 *  @brief This function skip current configuration block
 *
 *  @param data   A pointer to buffer of module configuration file
 *  @param size   Size of module configuration file
 *
 *  @return       MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status parse_skip_cfg_block(t_u8 *data, t_u32 size)
{
	int end = 0;
	t_u8 line[MAX_LINE_LEN];
	while (parse_cfg_get_line(data, size, line) != -1) {
		if (strncmp(line, "}", strlen("}")) == 0) {
			end = 1;
			break;
		}
		if (end == 0 && strstr(line, "{") != 0)
			break;
	}
	return (end == 1) ? MLAN_STATUS_SUCCESS : MLAN_STATUS_FAILURE;
}

/**
 *  @brief This function handle fallback processing for invalid
 *  block id with same card type
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return       MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_cfg_fallback_process(moal_handle *handle)
{
	int i, blk_id = 0x7fffffff, idx = -1;
	mlan_status ret = MLAN_STATUS_FAILURE;
	PRINTM(MMSG, "Configuration block, fallback processing\n");
	for (i = 0; i < MAX_MLAN_ADAPTER; i++) {
		if (m_handle[i] == NULL || m_handle[i] == handle ||
		    m_handle[i]->card_type != handle->card_type)
			continue;
		/* use configuratino with lowest blk_id value */
		if (m_handle[i]->blk_id >= 0 && m_handle[i]->blk_id <= blk_id) {
			idx = i;
			blk_id = m_handle[i]->blk_id;
		}
	}
	if (idx >= 0 && idx < MAX_MLAN_ADAPTER) {
		ret = MLAN_STATUS_SUCCESS;
		handle->blk_id = m_handle[idx]->blk_id;
		PRINTM(MMSG,
		       "Configuration fallback to, card_type: 0x%x, blk_id: 0x%x\n",
		       handle->card_type, handle->blk_id);
		woal_setup_module_param(handle, &m_handle[idx]->params);
	}
	return ret;
}

/**
 *  @brief This function parse and initialize module parameters
 *
 *  @param handle   A pointer to moal_handle structure
 *
 *  @return         MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status woal_init_module_param(moal_handle *handle)
{
	int no_match = 1;
	t_u32 size, i, tbl_size;
	t_u8 line[MAX_LINE_LEN], *data = NULL;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	char *card_type = NULL, *blk_id = NULL;

	memset(line, 0, MAX_LINE_LEN);
	woal_setup_module_param(handle, NULL);
	if (mod_para == NULL) {
		PRINTM(MMSG, "No module param cfg file specified\n");
		goto out;
	}
	if (woal_req_mod_param(handle, mod_para)) {
		PRINTM(MERROR, "Failed to get module param file\n");
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	tbl_size = sizeof(card_type_map_tbl) / sizeof(card_type_map_tbl[0]);
	for (i = 0; i < tbl_size; i++)
		if (handle->card_type == card_type_map_tbl[i].card_type)
			break;
	if (i >= tbl_size) {
		PRINTM(MERROR, "No card type entry found for card type: 0x%x\n",
		       handle->card_type);
		ret = MLAN_STATUS_FAILURE;
		goto out;
	}
	PRINTM(MMSG, "%s: init module param from usr cfg\n",
	       card_type_map_tbl[i].name);
	size = handle->param_data->size;
	data = (t_u8 *)handle->param_data->data;
	while (parse_cfg_get_line(data, size, line) != -1) {
		if (line[0] == '#')
			continue;
		if (strstr(line, "={")) {
			ret = parse_line_read_card_info(line, &card_type,
							&blk_id);
			if (ret != MLAN_STATUS_SUCCESS)
				goto out;
			PRINTM(MINFO,
			       "Traverse, card_type: %s, config block: %s\n",
			       card_type, blk_id);
			if (strcmp(card_type_map_tbl[i].name, card_type) == 0) {
				/* parse config block id */
				if (blk_id == NULL)
					handle->blk_id = 0;
				else
					woal_atoi(&handle->blk_id, blk_id);
				PRINTM(MINFO,
				       "Validation check, %s, config block: %d\n",
				       card_type, handle->blk_id);
				/* check validation of config id */
				if (woal_validate_cfg_id(handle) !=
				    MLAN_STATUS_SUCCESS) {
					ret = parse_skip_cfg_block(data, size);
					if (ret != MLAN_STATUS_SUCCESS) {
						PRINTM(MMSG,
						       "failed to skip block\n");
						goto out;
					}
					continue;
				}
				no_match = 0;
				PRINTM(MMSG,
				       "card_type: %s, config block: %d\n",
				       card_type, handle->blk_id);
				/* parse config block */
				ret = parse_cfg_read_block(data, size, handle);
				if (ret != MLAN_STATUS_SUCCESS)
					goto out;
				break;
			}
		}
	}
	if (no_match)
		ret = woal_cfg_fallback_process(handle);
out:
	if (handle->param_data) {
		release_firmware(handle->param_data);
		/* rewind pos */
		parse_cfg_get_line(NULL, 0, NULL);
	}
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "Invalid block: %s\n", line);
		woal_free_module_param(handle);
		woal_setup_module_param(handle, NULL);
	}
	return ret;
}

module_param(mod_para, charp, 0);
MODULE_PARM_DESC(mod_para, "Module parameters configuration file");
module_param(hw_test, int, 0660);
MODULE_PARM_DESC(hw_test, "0: Disable hardware test; 1: Enable hardware test");
#ifdef CONFIG_OF
module_param(dts_enable, int, 0);
MODULE_PARM_DESC(dts_enable, "0: Disable DTS; 1: Enable DTS");
#endif
module_param(fw_name, charp, 0660);
MODULE_PARM_DESC(fw_name, "Firmware name");
module_param(req_fw_nowait, int, 0);
MODULE_PARM_DESC(
	req_fw_nowait,
	"0: Use request_firmware API; 1: Use request_firmware_nowait API");
module_param(fw_reload, int, 0);
MODULE_PARM_DESC(fw_reload,
		 "0: disable fw_reload; 1: enable fw reload feature");
module_param(fw_serial, int, 0);
MODULE_PARM_DESC(
	fw_serial,
	"0: support parallel download FW; 1: support serial download FW");
module_param(mac_addr, charp, 0660);
MODULE_PARM_DESC(mac_addr, "MAC address");
#ifdef MFG_CMD_SUPPORT
module_param(mfg_mode, int, 0660);
MODULE_PARM_DESC(mfg_mode,
		 "0: Download normal firmware; 1: Download MFG firmware");
#endif /* MFG_CMD_SUPPORT */
module_param(drv_mode, int, 0660);
MODULE_PARM_DESC(drv_mode, "Bit 0: STA; Bit 1: uAP; Bit 2: WIFIDIRECT");

#ifdef STA_SUPPORT
module_param(max_sta_bss, int, 0);
MODULE_PARM_DESC(max_sta_bss, "Number of STA interfaces (1)");
module_param(sta_name, charp, 0);
MODULE_PARM_DESC(sta_name, "STA interface name");
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
module_param(max_uap_bss, int, 0);
MODULE_PARM_DESC(max_uap_bss, "Number of uAP interfaces (1)");
module_param(uap_name, charp, 0);
MODULE_PARM_DESC(uap_name, "uAP interface name");
#endif /* UAP_SUPPORT */
#ifdef WIFI_DIRECT_SUPPORT
module_param(max_wfd_bss, int, 0);
MODULE_PARM_DESC(max_wfd_bss, "Number of WIFIDIRECT interfaces (1)");
module_param(wfd_name, charp, 0);
MODULE_PARM_DESC(wfd_name, "WIFIDIRECT interface name");
#if defined(STA_CFG80211) && defined(UAP_CFG80211)
module_param(max_vir_bss, int, 0);
MODULE_PARM_DESC(max_vir_bss, "Number of Virtual interfaces (0)");
#endif
#endif /* WIFI_DIRECT_SUPPORT */
#ifdef DEBUG_LEVEL1
module_param(drvdbg, uint, 0660);
MODULE_PARM_DESC(drvdbg, "Driver debug");
#endif /* DEBUG_LEVEL1 */
module_param(auto_ds, int, 0660);
MODULE_PARM_DESC(
	auto_ds,
	"0: MLAN default; 1: Enable auto deep sleep; 2: Disable auto deep sleep");
module_param(ps_mode, int, 0660);
MODULE_PARM_DESC(
	ps_mode,
	"0: MLAN default; 1: Enable IEEE PS mode; 2: Disable IEEE PS mode");
module_param(p2a_scan, int, 0660);
MODULE_PARM_DESC(
	p2a_scan,
	"0: MLAN default; 1: Enable passive to active scan for DFS channel; 2: Disable passive to active scan for DFS channel");
module_param(scan_chan_gap, int, 0660);
MODULE_PARM_DESC(
	scan_chan_gap,
	"Time gap between two scans in milliseconds when connected to AP(max value 500ms)");
module_param(max_tx_buf, int, 0);
MODULE_PARM_DESC(max_tx_buf, "Maximum Tx buffer size (2048/4096/8192)");

#if defined(SDIO)
module_param(intmode, int, 0);
MODULE_PARM_DESC(intmode, "0: INT_MODE_SDIO, 1: INT_MODE_GPIO");
module_param(gpiopin, int, 0);
MODULE_PARM_DESC(gpiopin, "255:new GPIO int mode, other vlue: gpio pin number");
#endif

#ifdef SDIO_SUSPEND_RESUME
module_param(pm_keep_power, int, 0);
MODULE_PARM_DESC(pm_keep_power, "1: PM keep power; 0: PM no power");
module_param(shutdown_hs, int, 0);
MODULE_PARM_DESC(shutdown_hs,
		 "1: Enable HS when shutdown; 0: No HS when shutdown");
#endif
#if defined(STA_SUPPORT)
module_param(cfg_11d, int, 0);
MODULE_PARM_DESC(cfg_11d,
		 "0: MLAN default; 1: Enable 802.11d; 2: Disable 802.11d");
#endif
#if defined(SDIO)
module_param(slew_rate, int, 0);
MODULE_PARM_DESC(
	slew_rate,
	"0:has the slowest slew rate, then 01, then 02, and 03 has the highest slew rate");
#endif
module_param(tx_work, uint, 0660);
MODULE_PARM_DESC(tx_work, "1: Enable tx_work; 0: Disable tx_work");
module_param(dpd_data_cfg, charp, 0);
MODULE_PARM_DESC(dpd_data_cfg, "DPD data file name");
module_param(init_cfg, charp, 0);
MODULE_PARM_DESC(init_cfg, "Init config file name");
module_param(cal_data_cfg, charp, 0);
MODULE_PARM_DESC(cal_data_cfg, "Calibration data file name");
module_param(txpwrlimit_cfg, charp, 0);
MODULE_PARM_DESC(txpwrlimit_cfg,
		 "Set configuration data of Tx power limitation");
module_param(cntry_txpwr, int, 0);
MODULE_PARM_DESC(
	cntry_txpwr,
	"Allow setting tx power table of country; 0: disable (default), 1: enable.");
module_param(init_hostcmd_cfg, charp, 0);
MODULE_PARM_DESC(init_hostcmd_cfg, "Init hostcmd file name");
module_param(band_steer_cfg, charp, 0);
MODULE_PARM_DESC(band_steer_cfg, "band steer cfg file name");
module_param(cfg80211_wext, int, 0660);
MODULE_PARM_DESC(cfg80211_wext,
#ifdef STA_WEXT
		 "Bit 0: STA WEXT; "
#endif
#ifdef UAP_WEXT
		 "Bit 1: UAP WEXT; "
#endif
#ifdef STA_CFG80211
		 "Bit 2: STA CFG80211; "
#endif
#ifdef UAP_CFG80211
		 "Bit 3: UAP CFG80211;"
#endif
);
#if defined(USB)
module_param(skip_fwdnld, int, 0);
MODULE_PARM_DESC(skip_fwdnld, "0: Enable FW download; 1: Disable FW download");
#endif
module_param(wq_sched_prio, int, 0);
module_param(wq_sched_policy, int, 0);
MODULE_PARM_DESC(wq_sched_prio, "Priority of work queue");
MODULE_PARM_DESC(
	wq_sched_policy,
	"0: SCHED_NORMAL; 1: SCHED_FIFO; 2: SCHED_RR; 3: SCHED_BATCH; 5: SCHED_IDLE");
module_param(rx_work, int, 0);
MODULE_PARM_DESC(
	rx_work,
	"0: default; 1: Enable rx_work_queue; 2: Disable rx_work_queue");
module_param(aggrctrl, int, 0);
MODULE_PARM_DESC(aggrctrl,
		 "1: Enable Tx aggregation; 0: Disable Tx aggregation");
#ifdef USB
module_param(usb_aggr, int, 0);
MODULE_PARM_DESC(usb_aggr,
		 "0: MLAN default; 1: Enable USB aggr; 2: Disable USB aggr");
#endif
#ifdef PCIE
module_param(pcie_int_mode, int, 0);
MODULE_PARM_DESC(pcie_int_mode, "0: Legacy mode; 1: MSI mode; 2: MSI-X mode");
#endif /* PCIE */
module_param(low_power_mode_enable, int, 0);
MODULE_PARM_DESC(low_power_mode_enable, "0/1: Disable/Enable Low Power Mode");

#ifdef ANDROID_KERNEL
module_param(wakelock_timeout, int, 0);
MODULE_PARM_DESC(wakelock_timeout, "set wakelock_timeout value (ms)");
#endif

module_param(dev_cap_mask, uint, 0);
MODULE_PARM_DESC(dev_cap_mask, "Device capability mask");

#ifdef SDIO
module_param(sdio_rx_aggr, int, 0);
MODULE_PARM_DESC(sdio_rx_aggr,
		 "1: Enable SDIO rx aggr; 0: Disable SDIO rx aggr");
#endif

module_param(pmic, int, 0);
MODULE_PARM_DESC(
	pmic,
	"1: Send pmic configure cmd to firmware; 0: No pmic configure cmd sent to firmware");

module_param(antcfg, int, 0660);
MODULE_PARM_DESC(
	antcfg,
	"0:default; SD8887/SD8987-[1:Tx/Rx antenna 1, 2:Tx/Rx antenna 2, 0xffff:enable antenna diversity];SD8897/SD8997-[Bit0:Rx Path A, Bit1:Rx Path B, Bit 4:Tx Path A, Bit 5:Tx Path B];9098/9097-[Bit 0: 2G Tx/Rx path A, Bit 1: 2G Tx/Rx path B,Bit 8: 5G Tx/Rx path A, Bit 9: 5G Tx/Rx path B]");

module_param(uap_oper_ctrl, uint, 0);
MODULE_PARM_DESC(uap_oper_ctrl, "0:default; 0x20001:uap restarts on channel 6");

module_param(hs_wake_interval, int, 0660);
MODULE_PARM_DESC(
	hs_wake_interval,
	"Host sleep wakeup interval,it will round to nearest multiple dtim*beacon_period in fw");
module_param(indication_gpio, int, 0);
MODULE_PARM_DESC(
	indication_gpio,
	"GPIO to indicate wakeup source; high four bits: level for normal wakeup; low four bits: GPIO pin number.");
module_param(disconnect_on_suspend, int, 0);
MODULE_PARM_DESC(
	disconnect_on_suspend,
	"1: Enable disconnect wifi on suspend; 0: Disable disconnect wifi on suspend");
module_param(hs_mimo_switch, int, 0660);
MODULE_PARM_DESC(
	hs_mimo_switch,
	"Dynamic MIMO-SISO switch during host sleep; 0: disable (default), 1: enable");

module_param(indrstcfg, int, 0);
MODULE_PARM_DESC(
	indrstcfg,
	"Independent reset configuration; high byte: GPIO pin number; low byte: IR mode");

module_param(fixed_beacon_buffer, int, 0);
MODULE_PARM_DESC(
	fixed_beacon_buffer,
	"0: allocate default buffer size; 1: allocate max buffer size.");

#ifdef WIFI_DIRECT_SUPPORT
module_param(GoAgeoutTime, int, 0);
MODULE_PARM_DESC(GoAgeoutTime,
		 "0: use default ageout time; set Go age out time (TU 100ms)");
#endif

module_param(gtk_rekey_offload, int, 0);
MODULE_PARM_DESC(
	gtk_rekey_offload,
	"0: disable gtk_rekey_offload; 1: enable gtk_rekey_offload (default); 2: enable gtk_rekey_offload in suspend mode only;");

module_param(multi_dtim, ushort, 0);
MODULE_PARM_DESC(multi_dtim, "DTIM interval");

module_param(inact_tmo, ushort, 0);
MODULE_PARM_DESC(inact_tmo, "IEEE ps inactivity timout value");

module_param(napi, int, 0);
MODULE_PARM_DESC(napi, "1: enable napi api; 0: disable napi");

#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
module_param(dfs_offload, int, 0);
MODULE_PARM_DESC(dfs_offload, "1: enable dfs offload; 0: disable dfs offload.");
#endif

#ifdef UAP_SUPPORT
module_param(uap_max_sta, int, 0);
MODULE_PARM_DESC(uap_max_sta, "Maximum station number for UAP/GO.");
#endif
#if defined(STA_CFG80211) || defined(UAP_CFG80211)
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
module_param(host_mlme, int, 0);
MODULE_PARM_DESC(host_mlme,
		 "1: Enable Host MLME Support; 0: Disable Host MLME support");
#endif
#endif

#if defined(STA_CFG80211) || defined(UAP_CFG80211)
module_param(disable_regd_by_driver, int, 0);
MODULE_PARM_DESC(
	disable_regd_by_driver,
	"0: reg domain set by driver enable(default); 1: reg domain set by driver disable");
module_param(reg_alpha2, charp, 0660);
MODULE_PARM_DESC(reg_alpha2, "Regulatory alpha2");
#if CFG80211_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
module_param(country_ie_ignore, int, 0);
MODULE_PARM_DESC(
	country_ie_ignore,
	"0: Follow countryIE from AP and beacon hint enable; 1: Ignore countryIE from AP and beacon hint disable");
module_param(beacon_hints, int, 0);
MODULE_PARM_DESC(beacon_hints,
		 "0: enable beacon hints(default); 1: disable beacon hints");
#endif
#endif

module_param(dfs53cfg, int, 0);
MODULE_PARM_DESC(dfs53cfg, "0: fw default; 1: new w53 dfs; 2: old w53 dfs");
