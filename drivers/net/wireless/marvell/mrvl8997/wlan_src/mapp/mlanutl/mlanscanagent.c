/** @file  mlanscanagent.c
  *
  * @brief This files contains mlanutl scanagent command handling.
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
     08/11/2009: initial version
************************************************************************/

#include    "mlanutl.h"
#include    "mlanhostcmd.h"
#include    "mlanoffload.h"
#include    "mlanscanagent.h"

/********************************************************
		Local Variables
********************************************************/

/********************************************************
		Global Variables
********************************************************/

/********************************************************
		Local Functions
********************************************************/

/**
 *  @brief scanagent configure scan table
 *
 *  @param age_limit    age limit
 *  @param hold_limit   hold limit
 *
 *  @return             MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_cfg_scan_table_limits(t_u32 age_limit, t_u32 hold_limit)
{
	int ret = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_SCANAGENT_SCAN_TABLE_LIMITS *scan_table_limits = NULL;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);
	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_SCANAGENT_SCAN_TABLE_LIMITS);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return -ENOMEM;
	}

	/* Fill up buffer */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	memset(cmd, 0, sizeof(struct eth_priv_cmd));
	memcpy(&cmd->buf, &buffer, sizeof(buffer));
#else
	cmd->buf = buffer;
#endif
	cmd->used_len = 0;
	cmd->total_len = BUFFER_LENGTH;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD, strlen(HOSTCMD));

	/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
	hostcmd = (HostCmd_DS_GEN *)(buffer + cmd_header_len + sizeof(t_u32));
	hostcmd->command = cpu_to_le16(HostCmd_CMD_SCANAGENT_SCAN_TABLE_LIMITS);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	scan_table_limits = (HostCmd_DS_SCANAGENT_SCAN_TABLE_LIMITS *)pos;
	scan_table_limits->table_age_limit = cpu_to_le16(age_limit);
	scan_table_limits->table_hold_limit = cpu_to_le16(hold_limit);

	/* 0 set values are ignored by firmware */
	scan_table_limits->action = cpu_to_le16(HostCmd_ACT_GEN_SET);

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[scanAgentIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	printf("\nAge limit  = %7d seconds\n",
	       le16_to_cpu(scan_table_limits->table_age_limit));
	printf("Hold limit = %7d seconds\n\n",
	       le16_to_cpu(scan_table_limits->table_hold_limit));

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Set scanagent age limit
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_age_limit(int argc, char *argv[])
{
	t_u32 age_limit = 0;

	if (argc) {
		age_limit = atoi(argv[0]);
	}

	return scanagent_cfg_scan_table_limits(age_limit, 0);
}

/**
 *  @brief Set scanagent hold limit
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_hold_limit(int argc, char *argv[])
{
	t_u32 hold_limit = 0;

	if (argc) {
		hold_limit = atoi(argv[0]);
	}

	return scanagent_cfg_scan_table_limits(0, hold_limit);
}

/**
 *  @brief Set scanagent scan timing
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_timing(int argc, char *argv[])
{
	int ret = 0;
	struct ifreq ifr;
	int idx;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len = 0, sel = 0;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_SCANAGENT_CONFIG_TIMING *cfg_timing_cmd = NULL;
	MrvlIEtypes_ConfigScanTiming_t *cfg_timing_tlv = NULL;
	timing_sel_t sel_str[] = { {"disconnected", 1},
	{"adhoc", 1},
	{"fullpower", 1},
	{"ieeeps", 1},
	{"periodic", 1}
	};

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);
	cmd_len = S_DS_GEN + sizeof(t_u16);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return -ENOMEM;
	}

	/* Fill up buffer */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	memset(cmd, 0, sizeof(struct eth_priv_cmd));
	memcpy(&cmd->buf, &buffer, sizeof(buffer));
#else
	cmd->buf = buffer;
#endif
	cmd->used_len = 0;
	cmd->total_len = BUFFER_LENGTH;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD, strlen(HOSTCMD));

	/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
	hostcmd = (HostCmd_DS_GEN *)(buffer + cmd_header_len + sizeof(t_u32));
	hostcmd->command = cpu_to_le16(HostCmd_CMD_SCANAGENT_SCAN_TIMING);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cfg_timing_cmd = (HostCmd_DS_SCANAGENT_CONFIG_TIMING *)pos;
	cfg_timing_cmd->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

	cfg_timing_tlv
		= (MrvlIEtypes_ConfigScanTiming_t *)cfg_timing_cmd->tlv_buffer;

	if (argc == 5) {
		cfg_timing_cmd->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
		cfg_timing_tlv->header.type = cpu_to_le16(TLV_TYPE_SCAN_TIMING);
		cfg_timing_tlv->header.len =
			cpu_to_le16(sizeof(MrvlIEtypes_ConfigScanTiming_t)
				    - sizeof(cfg_timing_tlv->header));

		for (idx = 0; (unsigned int)idx < NELEMENTS(sel_str); idx++) {
			if (strncmp(argv[0],
				    sel_str[idx].str,
				    sel_str[idx].match_len) == 0) {
				sel = idx + 1;
				break;
			}
		}

		if (idx == NELEMENTS(sel_str)) {
			printf("Wrong argument for mode selected \"%s\"\n",
			       argv[0]);
			ret = -EINVAL;
			goto done;
		}

		/*
		 *  HostCmd_DS_ScanagentTimingMode_e;
		 *     TIMING_MODE_INVALID        = 0,
		 *     TIMING_MODE_DISCONNECTED   = 1,
		 *     TIMING_MODE_ADHOC          = 2,
		 *     TIMING_MODE_FULL_POWER     = 3,
		 *     TIMING_MODE_IEEE_PS        = 4,
		 *     TIMING_MODE_PERIODIC_PS    = 5,
		 */
		cfg_timing_tlv->mode = cpu_to_le32(sel);
		cfg_timing_tlv->dwell = cpu_to_le32(atoi(argv[1]));
		cfg_timing_tlv->max_off = cpu_to_le32(atoi(argv[2]));
		cfg_timing_tlv->min_link = cpu_to_le32(atoi(argv[3]));
		cfg_timing_tlv->rsp_timeout = cpu_to_le32(atoi(argv[4]));

		cmd_len += sizeof(MrvlIEtypes_ConfigScanTiming_t);
	}

	hostcmd->size = cpu_to_le16(cmd_len);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[scanAgentIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	ret = process_host_cmd_resp(HOSTCMD, buffer);

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Set scanagent profile scan period
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_profile_period(int argc, char *argv[])
{
	int ret = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len = 0;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_SCANAGENT_CONFIG_PROFILE_SCAN *cfg_profile_scan = NULL;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);
	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_SCANAGENT_CONFIG_PROFILE_SCAN);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return -ENOMEM;
	}

	/* Fill up buffer */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	memset(cmd, 0, sizeof(struct eth_priv_cmd));
	memcpy(&cmd->buf, &buffer, sizeof(buffer));
#else
	cmd->buf = buffer;
#endif
	cmd->used_len = 0;
	cmd->total_len = BUFFER_LENGTH;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD, strlen(HOSTCMD));

	/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
	hostcmd = (HostCmd_DS_GEN *)(buffer + cmd_header_len + sizeof(t_u32));
	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_SCANAGENT_CONFIG_PROFILE_SCAN);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cfg_profile_scan = (HostCmd_DS_SCANAGENT_CONFIG_PROFILE_SCAN *)pos;
	if (argc == 1) {
		cfg_profile_scan->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
		cfg_profile_scan->scan_interval = cpu_to_le16(atoi(argv[0]));
	} else {
		cfg_profile_scan->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[scanAgentIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	cfg_profile_scan->scan_interval =
		le16_to_cpu(cfg_profile_scan->scan_interval);
	if ((int)cfg_profile_scan->scan_interval == 0)
		printf("\nProfile Scan interval: <disabled>\n\n");
	else
		printf("\nProfile Scan interval: %d seconds\n\n",
		       (int)cfg_profile_scan->scan_interval);

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief scanagent parse entry selection
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param psel     A pointer to scanagent entry selection
 *
 *  @return         None
 */
static void
scanAgentParseEntrySel(int argc, char *argv[],
		       HostCmd_DS_SCANAGENT_TABLE_MAINTENANCE *psel,
		       int *cmd_len)
{
	int arg_idx, tmp_idx;
	t_u8 *tlv_pos;
	MrvlIEtypes_SsIdParamSet_t *ssid;
	MrvlIEtypes_Bssid_List_t *bssid;
	unsigned int mac[ETH_ALEN];

	tlv_pos = (t_u8 *)psel->tlv_buffer;

	for (arg_idx = 0; arg_idx < argc; arg_idx++) {
		if (strncmp(argv[arg_idx], "ssid=", strlen("ssid=")) == 0) {
			ssid = (MrvlIEtypes_SsIdParamSet_t *)tlv_pos;
			ssid->header.type = cpu_to_le16(TLV_TYPE_SSID);
			ssid->header.len =
				strlen(argv[arg_idx]) - strlen("ssid=");
			strncpy((char *)ssid->ssid,
				(argv[arg_idx] + strlen("ssid=")),
				ssid->header.len);
			tlv_pos +=
				ssid->header.len + sizeof(MrvlIEtypesHeader_t);
			ssid->header.len = cpu_to_le16(ssid->header.len);

		} else if (strncmp(argv[arg_idx], "bssid=", strlen("bssid=")) ==
			   0) {
			bssid = (MrvlIEtypes_Bssid_List_t *)tlv_pos;
			bssid->header.type = cpu_to_le16(TLV_TYPE_BSSID);
			bssid->header.len = ETH_ALEN;
			/*
			 *  "bssid" token string handler
			 */
			sscanf(argv[arg_idx] + strlen("bssid="),
			       "%2x:%2x:%2x:%2x:%2x:%2x", mac + 0, mac + 1,
			       mac + 2, mac + 3, mac + 4, mac + 5);
			for (tmp_idx = 0;
			     (unsigned int)tmp_idx < NELEMENTS(mac);
			     tmp_idx++) {
				bssid->bssid[tmp_idx] = (t_u8)mac[tmp_idx];
			}
			tlv_pos +=
				bssid->header.len + sizeof(MrvlIEtypesHeader_t);
			bssid->header.len = cpu_to_le16(bssid->header.len);

		} else if (strncmp(argv[arg_idx], "age=", strlen("age=")) == 0) {
			psel->age =
				cpu_to_le32(atoi
					    (argv[arg_idx] + strlen("age=")));

		} else if (strncmp(argv[arg_idx], "id=", strlen("id=")) == 0) {
			psel->scan_request_id =
				cpu_to_le32(atoi
					    (argv[arg_idx] + strlen("id=")));
		}
	}

	*cmd_len += (tlv_pos - psel->tlv_buffer);
}

/**
 *  @brief scanagent execute scan
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_exec_scan(int argc, char *argv[])
{
	int ret = 0;
	struct ifreq ifr;
	int arg_idx, tmp_idx;
	t_u32 cmd_len = 0, cmd_header_len = 0;
	t_u8 *buffer = NULL, *pos = NULL, *tlv_pos = NULL;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_SCANAGENT_SCAN_EXEC *scan_exec = NULL;
	MrvlIEtypes_SsIdParamSet_t *ssid = NULL;
	MrvlIEtypes_Bssid_List_t *bssid = NULL;
	MrvlIEtypes_ConfigScanTiming_t *cfg_timing_tlv = NULL;
	unsigned int mac[ETH_ALEN];

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);
	cmd_len = (S_DS_GEN + sizeof(HostCmd_DS_SCANAGENT_SCAN_EXEC)
		   - sizeof(scan_exec->tlv_buffer));

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return -ENOMEM;
	}

	/* Fill up buffer */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	memset(cmd, 0, sizeof(struct eth_priv_cmd));
	memcpy(&cmd->buf, &buffer, sizeof(buffer));
#else
	cmd->buf = buffer;
#endif
	cmd->used_len = 0;
	cmd->total_len = BUFFER_LENGTH;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD, strlen(HOSTCMD));

	/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
	hostcmd = (HostCmd_DS_GEN *)(buffer + cmd_header_len + sizeof(t_u32));
	hostcmd->command = cpu_to_le16(HostCmd_CMD_SCANAGENT_SCAN_EXEC);
	hostcmd->size = 0;
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	scan_exec = (HostCmd_DS_SCANAGENT_SCAN_EXEC *)pos;
	tlv_pos = scan_exec->tlv_buffer;

	for (arg_idx = 0; arg_idx < argc; arg_idx++) {
		if (strncmp(argv[arg_idx], "ssid=", strlen("ssid=")) == 0) {
			/*
			 *  "ssid" token string handler
			 */
			ssid = (MrvlIEtypes_SsIdParamSet_t *)tlv_pos;
			ssid->header.type = cpu_to_le16(TLV_TYPE_SSID);
			ssid->header.len =
				strlen(argv[arg_idx]) - strlen("ssid=");
			strncpy((char *)ssid->ssid,
				argv[arg_idx] + strlen("ssid="),
				ssid->header.len);
			tlv_pos +=
				ssid->header.len + sizeof(MrvlIEtypesHeader_t);
			ssid->header.len = cpu_to_le16(ssid->header.len);
		} else if (strncmp(argv[arg_idx], "bssid=", strlen("bssid=")) ==
			   0) {
			bssid = (MrvlIEtypes_Bssid_List_t *)tlv_pos;
			bssid->header.type = cpu_to_le16(TLV_TYPE_BSSID);
			bssid->header.len = ETH_ALEN;
			/*
			 *  "bssid" token string handler
			 */
			sscanf(argv[arg_idx] + strlen("bssid="),
			       "%2x:%2x:%2x:%2x:%2x:%2x", mac + 0, mac + 1,
			       mac + 2, mac + 3, mac + 4, mac + 5);
			for (tmp_idx = 0;
			     (unsigned int)tmp_idx < NELEMENTS(mac);
			     tmp_idx++) {
				bssid->bssid[tmp_idx] = (t_u8)mac[tmp_idx];
			}
			tlv_pos +=
				bssid->header.len + sizeof(MrvlIEtypesHeader_t);
			bssid->header.len = cpu_to_le16(bssid->header.len);
		} else if (strncmp(argv[arg_idx], "type=", strlen("type=")) ==
			   0) {
			/*
			   if (strcmp(argv[arg_idx] + strlen("type="), "prof") == 0) {
			   scan_exec->scan_type = CONFIG_PROFILE;
			   } else {
			   scan_exec->scan_type = CONFIG_SITE_SURVEY;
			   }
			 */
			scan_exec->scan_type = CONFIG_SITE_SURVEY;
			scan_exec->scan_type =
				cpu_to_le16(scan_exec->scan_type);
		} else if (strncmp(argv[arg_idx], "group=", strlen("group=")) ==
			   0) {
			sscanf(argv[arg_idx] + strlen("group="), "0x%x",
			       &tmp_idx);
			scan_exec->chan_group = cpu_to_le32(tmp_idx);
		} else if (strncmp(argv[arg_idx], "delay=", strlen("delay=")) ==
			   0) {
			/*
			 *  "delay" token string handler
			 */
			sscanf(argv[arg_idx] + strlen("delay="),
			       "%d", (int *)&scan_exec->delay);
			scan_exec->delay = cpu_to_le32(scan_exec->delay);
		} else if (strncmp(argv[arg_idx], "timing=", strlen("timing="))
			   == 0) {
			cfg_timing_tlv =
				(MrvlIEtypes_ConfigScanTiming_t *)tlv_pos;
			cfg_timing_tlv->header.type =
				cpu_to_le16(TLV_TYPE_SCAN_TIMING);
			cfg_timing_tlv->header.len = ((sizeof(cfg_timing_tlv)
						       -
						       sizeof(cfg_timing_tlv->
							      header)));
			/*
			 *  "timing" token string handler
			 */
			sscanf(argv[arg_idx] + strlen("timing="), "%d,%d,%d,%d",
			       (int *)&cfg_timing_tlv->dwell,
			       (int *)&cfg_timing_tlv->max_off,
			       (int *)&cfg_timing_tlv->min_link,
			       (int *)&cfg_timing_tlv->rsp_timeout);

			cfg_timing_tlv->mode = 0;
			cfg_timing_tlv->dwell =
				cpu_to_le32(cfg_timing_tlv->dwell);
			cfg_timing_tlv->max_off =
				cpu_to_le32(cfg_timing_tlv->max_off);
			cfg_timing_tlv->min_link =
				cpu_to_le32(cfg_timing_tlv->min_link);
			cfg_timing_tlv->rsp_timeout =
				cpu_to_le32(cfg_timing_tlv->rsp_timeout);

			tlv_pos += sizeof(MrvlIEtypesHeader_t);
			tlv_pos += cfg_timing_tlv->header.len;
			cfg_timing_tlv->header.len =
				cpu_to_le16(cfg_timing_tlv->header.len);
		}
	}

	cmd_len += (tlv_pos - scan_exec->tlv_buffer);
	hostcmd->size = cpu_to_le16(cmd_len);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[scanAgentIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	} else {
		printf("\nScan Scheduled, ID = %d\n\n",
		       (int)le32_to_cpu(scan_exec->scan_req_id_out));
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a scanagent cmd_type subcommand
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param cmd_type command type
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_table_entry_sub_cmd(int argc, char *argv[],
			      HostCmd_DS_ScanagentTableMaintenance_e cmd_type)
{
	int ret = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len = 0;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_SCANAGENT_TABLE_MAINTENANCE *table_maintenance = NULL;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);
	cmd_len = (S_DS_GEN + sizeof(HostCmd_DS_SCANAGENT_TABLE_MAINTENANCE)
		   - sizeof(table_maintenance->tlv_buffer));

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return -ENOMEM;
	}

	/* Fill up buffer */
#ifdef USERSPACE_32BIT_OVER_KERNEL_64BIT
	memset(cmd, 0, sizeof(struct eth_priv_cmd));
	memcpy(&cmd->buf, &buffer, sizeof(buffer));
#else
	cmd->buf = buffer;
#endif
	cmd->used_len = 0;
	cmd->total_len = BUFFER_LENGTH;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD, strlen(HOSTCMD));

	/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
	hostcmd = (HostCmd_DS_GEN *)(buffer + cmd_header_len + sizeof(t_u32));
	hostcmd->command = cpu_to_le16(HostCmd_CMD_SCANAGENT_TABLE_MAINTENANCE);
	hostcmd->size = 0;
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	table_maintenance = (HostCmd_DS_SCANAGENT_TABLE_MAINTENANCE *)pos;
	table_maintenance->action = cpu_to_le16((t_u16)cmd_type);

	scanAgentParseEntrySel(argc, argv, table_maintenance, (int *)&cmd_len);

	hostcmd->size = cpu_to_le16(cmd_len);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[scanAgentIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a scanagent table lock subcommand
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_table_lock(int argc, char *argv[])
{
	return scanagent_table_entry_sub_cmd(argc, argv, SCAN_TABLE_OP_LOCK);
}

/**
 *  @brief Issue a scanagent table unlock subcommand
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_table_unlock(int argc, char *argv[])
{
	return scanagent_table_entry_sub_cmd(argc, argv, SCAN_TABLE_OP_UNLOCK);
}

/**
 *  @brief Issue a scanagent table purge subcommand
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS on success, otherwise error code
 */
static int
scanagent_table_purge(int argc, char *argv[])
{
	return scanagent_table_entry_sub_cmd(argc, argv, SCAN_TABLE_OP_PURGE);
}

/**
 *  @brief Issue a scanagent command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_scanagent(int argc, char *argv[])
{
	sub_cmd_exec_t sub_cmd[] = {
		{"timing", 2, 1, scanagent_timing},
		{"scan", 2, 1, scanagent_exec_scan},
		{"lock", 2, 1, scanagent_table_lock},
		{"unlock", 2, 1, scanagent_table_unlock},
		{"purge", 2, 1, scanagent_table_purge},
		{"profile", 2, 1, scanagent_profile_period},
		{"holdlimit", 2, 1, scanagent_hold_limit},
		{"agelimit", 2, 1, scanagent_age_limit}
	};

	return process_sub_cmd(sub_cmd, NELEMENTS(sub_cmd), argc, argv);
}
