/** @file  mlanoffload.c
 *
 * @brief This files contains mlanutl offload command handling.
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

/********************************************************
				Local Variables
********************************************************/

t_void hexdump(char *prompt, t_void *p, t_s32 len, t_s8 delim);

/********************************************************
				Global Variables
********************************************************/

/********************************************************
				Local Functions
********************************************************/

/**
 *  @brief Remove unwanted spaces, tabs from a line
 *
 *  @param data     A pointer to the starting of the line
 *  @return         NA
 */
static void
profile_param_polish(char *data)
{
	t_u8 i, j, len = 0;
	char *ptr;
	ptr = strrchr(data, '\r');
	if (ptr == NULL) {
		ptr = strrchr(data, '\n');
		if (ptr == NULL) {
			return;
		}
	}
	len = ptr - data;
	for (i = 0; i < len; i++) {
		if ((*(data + i) == ' ') || (*(data + i) == '\t')) {
			for (j = i; j < len; j++) {
				data[j] = data[j + 1];
			}
			i--;
			len--;
		}
	}
}

static int
ascii_value(char letter)
{
	if (letter >= '0' && letter <= '9')
		return letter - '0';
	if (letter >= 'a' && letter <= 'f')
		return letter - 'a' + 10;
	if (letter >= 'A' && letter <= 'F')
		return letter - 'A' + 10;
	return -1;
}

static int
twodigit_ascii(const char *nibble)
{
	int a, b;
	a = ascii_value(*nibble++);
	if (a < 0)
		return -1;
	b = ascii_value(*nibble++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}

/**
 *  @brief Read a network block from the profile configuration file
 *
 *  @param fp       file pointer of the configuration file
 *  @param p_head   profile head
 *  @return         MLAN_STATUS_SUCCESS
 */
static int
profile_read_block(FILE * fp, profile_entry_t *p_head)
{
	char line[0x100];
	char *ptr, *eptr;
	t_u8 key_cnt = 0;
	t_u8 i, wep_len;
	int byte;
	int tmpIdx;
	unsigned int mac[ETH_ALEN];

	while (fgets(line, sizeof(line), fp)) {
		/* call function to remove spaces, tabs */
		profile_param_polish(line);

		if (strstr(line, "}") != NULL) {
			ptr = strstr(line, "}");
			/* end of network */
			break;

		} else if (line[0] == '#') {
			/* comments go ahead */
			continue;

		} else if (strstr(line, "bssid=") != NULL) {
			ptr = strstr(line, "bssid=");
			ptr = ptr + strlen("bssid=");
			sscanf(ptr, "%2x:%2x:%2x:%2x:%2x:%2x",
			       mac + 0, mac + 1, mac + 2, mac + 3, mac + 4,
			       mac + 5);
			for (tmpIdx = 0; (unsigned int)tmpIdx < NELEMENTS(mac);
			     tmpIdx++) {
				p_head->bssid[tmpIdx] = (t_u8)mac[tmpIdx];
			}

		} else if (strstr(line, "ssid=") != NULL) {

			ptr = strstr(line, "ssid=");
			ptr = ptr + strlen("ssid=");
			eptr = strrchr(ptr + 1, '"');

			if ((*ptr != '"') || (strrchr(ptr + 1, '"') == NULL)) {

				fprintf(stderr, "ssid not within quotes\n");
				break;
			}

			p_head->ssid_len =
				MIN(IW_ESSID_MAX_SIZE, eptr - ptr - 1);
			strncpy((char *)p_head->ssid, ptr + 1,
				p_head->ssid_len);
			p_head->ssid[p_head->ssid_len] = '\0';

		} else if (strstr(line, "psk=") != NULL) {
			ptr = strstr(line, "psk=");
			ptr = ptr + strlen("psk=");
			if (*ptr != '"') {
				p_head->psk_config = 1;
				strncpy((char *)p_head->psk, ptr, KEY_LEN);
			} else {
				eptr = strrchr(ptr + 1, '"');
				if (eptr == NULL) {
					fprintf(stderr,
						"passphrase not within quotes.\n");
					break;
				}
				p_head->passphrase_len =
					MIN(PHRASE_LEN, eptr - ptr - 1);
				strncpy((char *)p_head->passphrase, ptr + 1,
					p_head->passphrase_len);
			}
		} else if (strstr(line, "wep_key") != NULL) {
			ptr = strstr(line, "wep_key");
			ptr = ptr + strlen("wep_key");
			key_cnt = atoi(ptr);
			ptr++;
			if (*ptr != '=') {
				fprintf(stderr,
					"invalid wep_key, missing =.\n");
				break;
			}
			eptr = strrchr(ptr + 1, '\r');
			if (eptr == NULL) {
				eptr = strrchr(ptr + 1, '\n');
				if (eptr == NULL) {
					fprintf(stderr,
						"missing EOL from the wep_key config\n");
					break;
				}
			}
			ptr++;
			if (*ptr == '"') {
				eptr = strrchr(ptr + 1, '"');
				if (eptr == NULL) {
					fprintf(stderr,
						"wep key does not end with quote.\n");
					break;
				}
				*eptr = '\0';
				p_head->wep_key_len[key_cnt] = eptr - ptr - 1;
				strncpy((char *)p_head->wep_key[key_cnt],
					ptr + 1, p_head->wep_key_len[key_cnt]);
			} else {
				while (*eptr == '\r' || *eptr == '\n')
					eptr--;
				*(eptr + 1) = '\0';
				wep_len = strlen(ptr);
				if (wep_len & 0x01) {
					fprintf(stderr,
						"incorrect wep key %s.\n", ptr);
					break;
				}
				p_head->wep_key_len[key_cnt] = wep_len / 2;
				for (i = 0; i < wep_len / 2; i++) {
					byte = twodigit_ascii(ptr);
					if (byte == -1) {
						fprintf(stderr,
							"incorrect wep key %s.\n",
							ptr);
						break;
					}
					*(p_head->wep_key[key_cnt] + i) =
						(t_u8)byte;
					ptr += 2;
				}
			}
		} else if (strstr(line, "key_mgmt=") != NULL) {
			ptr = strstr(line, "key_mgmt=");
			ptr = ptr + strlen("key_mgmt=");
			eptr = strstr(ptr, "WPA-EAP");
			if (eptr != NULL) {
				p_head->key_mgmt |=
					PROFILE_DB_KEY_MGMT_IEEE8021X;
			}
			eptr = strstr(ptr, "WPA-PSK");
			if (eptr != NULL) {
				p_head->key_mgmt |= PROFILE_DB_KEY_MGMT_PSK;
			}
			eptr = strstr(ptr, "FT-EAP");
			if (eptr != NULL) {
				p_head->key_mgmt |=
					PROFILE_DB_KEY_MGMT_FT_IEEE8021X;
			}
			eptr = strstr(ptr, "FT-PSK");
			if (eptr != NULL) {
				p_head->key_mgmt |= PROFILE_DB_KEY_MGMT_FT_PSK;
			}
			eptr = strstr(ptr, "WPA-EAP-SHA256");
			if (eptr != NULL) {
				p_head->key_mgmt |=
					PROFILE_DB_KEY_MGMT_SHA256_IEEE8021X;
			}
			eptr = strstr(ptr, "WPA-PSK-SHA256");
			if (eptr != NULL) {
				p_head->key_mgmt |=
					PROFILE_DB_KEY_MGMT_SHA256_PSK;
			}
			eptr = strstr(ptr, "CCKM");
			if (eptr != NULL) {
				p_head->key_mgmt |= PROFILE_DB_KEY_MGMT_CCKM;
			}
			eptr = strstr(ptr, "NONE");
			if (eptr != NULL) {
				p_head->key_mgmt |= PROFILE_DB_KEY_MGMT_NONE;
			}
		} else if (strstr(line, "proto=") != NULL) {
			ptr = strstr(line, "proto=");
			ptr = ptr + strlen("proto=");
			eptr = strstr(ptr, "WPA");
			if (eptr != NULL) {
				p_head->protocol |= PROFILE_DB_PROTO_WPA;
			}

			eptr = strstr(ptr, "RSN");
			if (eptr != NULL) {
				p_head->protocol |= PROFILE_DB_PROTO_WPA2;

			}
		} else if (strstr(line, "pairwise=") != NULL) {
			ptr = strstr(line, "pairwise=");
			ptr = ptr + strlen("pairwise=");
			eptr = strstr(ptr, "CCMP");
			if (eptr != NULL) {
				p_head->pairwise_cipher |=
					PROFILE_DB_CIPHER_CCMP;
			}
			eptr = strstr(ptr, "TKIP");
			if (eptr != NULL) {
				p_head->pairwise_cipher |=
					PROFILE_DB_CIPHER_TKIP;
			}
		} else if (strstr(line, "groupwise=") != NULL) {
			ptr = strstr(line, "groupwise=");
			ptr = ptr + strlen("groupwise=");
			eptr = strstr(ptr, "CCMP");
			if (eptr != NULL) {
				p_head->groupwise_cipher |=
					PROFILE_DB_CIPHER_CCMP;
			}
			eptr = strstr(ptr, "TKIP");
			if (eptr != NULL) {
				p_head->groupwise_cipher |=
					PROFILE_DB_CIPHER_TKIP;
			}
		} else if (strstr(line, "wep_tx_keyidx=") != NULL) {
			ptr = strstr(line, "wep_tx_keyidx=");
			ptr = ptr + strlen("wep_tx_keyidx=");
			p_head->wep_key_idx = atoi(ptr);
		} else if (strstr(line, "roaming=") != NULL) {
			ptr = strstr(line, "roaming=");
			ptr = ptr + strlen("roaming=");
			p_head->roaming = atoi(ptr);
		} else if (strstr(line, "ccx=") != NULL) {
			ptr = strstr(line, "ccx=");
			ptr = ptr + strlen("ccx=");
			p_head->ccx = atoi(ptr);
		} else if (strstr(line, "mode=") != NULL) {
			ptr = strstr(line, "mode=");
			ptr = ptr + strlen("mode=");
			p_head->mode = atoi(ptr);
		}
	}
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Issue profile command to add new profile to FW
 *
 *  @param filename     Name of profile file
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
profile_read_download(char *filename)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	int i = 0;
	t_u16 temp, tempc;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	profile_entry_t *p_head = NULL;
	FILE *fp;
	char line[0x100];

	fp = fopen(filename, "r");
	if (fp == NULL) {
		perror("fopen");
		fprintf(stderr, "Cannot open file %s\n", filename);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		ret = ENOMEM;
		goto done;
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

	pos = (t_u8 *)hostcmd;
	while (fgets(line, sizeof(line), fp)) {
		/* call function to remove spaces, tabs */
		profile_param_polish(line);
		if ((strstr(line, "network={") == NULL) || (line[0] == '#')) {
			continue;
		}
		/*
		 * Memory allocation of every network block
		 */
		p_head = (profile_entry_t *)malloc(sizeof(profile_entry_t));
		if (p_head == NULL) {
			fprintf(stderr, "Memory error.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		memset(p_head, 0x00, sizeof(profile_entry_t));

		ret = profile_read_block(fp, p_head);
		if (ret || p_head->ssid_len == 0) {
			free(p_head);
			continue;
		}

		/*
		 * Put all the ssid parameters in the buffer
		 */
		memset(pos, 0,
		       (BUFFER_LENGTH - cmd_header_len - sizeof(t_u32)));

		/* Cmd Header : Command */
		hostcmd->command = cpu_to_le16(HostCmd_CMD_PROFILE_DB);
		cmd_len = sizeof(HostCmd_DS_GEN);

		/* set action as set */
		tempc = cpu_to_le16(HostCmd_ACT_GEN_SET);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;

		/* ssid */
		tempc = cpu_to_le16(TLV_TYPE_SSID);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = strlen((char *)p_head->ssid);
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		memcpy((void *)(pos + cmd_len), p_head->ssid, temp);
		cmd_len += temp;

		if (memcmp(p_head->bssid, "\x00\x00\x00\x00\x00\x00", ETH_ALEN)) {
			/* bssid */
			tempc = cpu_to_le16(TLV_TYPE_BSSID);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			temp = ETH_ALEN;
			tempc = cpu_to_le16(temp);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			memcpy((void *)(pos + cmd_len), p_head->bssid, temp);
			cmd_len += temp;
		}

		/* proto */
		if (p_head->protocol == 0) {
			p_head->protocol = 0xFFFF;
		}

		tempc = cpu_to_le16(TLV_TYPE_PROTO);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = 2;
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		memcpy((pos + cmd_len), &(p_head->protocol), temp);
		cmd_len += temp;

		/* key_mgmt */
		if (p_head->key_mgmt == 0) {
			p_head->key_mgmt = 0xFFFF;
		}

		tempc = cpu_to_le16(TLV_TYPE_AKMP);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = 2;
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		memcpy((pos + cmd_len), &(p_head->key_mgmt), temp);
		cmd_len += temp;

		/* pairwise */
		if (p_head->pairwise_cipher == 0) {
			p_head->pairwise_cipher = 0xFF;
		}

		/* groupwise */
		if (p_head->groupwise_cipher == 0) {
			p_head->groupwise_cipher = 0xFF;
		}

		tempc = cpu_to_le16(TLV_TYPE_CIPHER);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = 2;
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		pos[cmd_len] = p_head->pairwise_cipher;
		cmd_len += 1;
		pos[cmd_len] = p_head->groupwise_cipher;
		cmd_len += 1;

		if (p_head->passphrase_len) {
			/* passphrase */
			tempc = cpu_to_le16(TLV_TYPE_PASSPHRASE);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			temp = p_head->passphrase_len;
			tempc = cpu_to_le16(temp);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			memcpy((void *)(pos + cmd_len), p_head->passphrase,
			       temp);
			cmd_len += temp;
		}

		if (p_head->psk_config) {
			/* psk method */
			tempc = cpu_to_le16(TLV_TYPE_PMK);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			temp = 32;
			tempc = cpu_to_le16(temp);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			memcpy((void *)(pos + cmd_len), p_head->psk, temp);
			cmd_len += temp;
		}

		for (i = 0; i < WEP_KEY_CNT; i++) {
			if (p_head->wep_key_len[i]) {
				/* TAG_WEP_KEY */
				tempc = cpu_to_le16(TLV_TYPE_WEP_KEY);
				memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
				cmd_len += 2;
				/* wep_key_len + sizeof(keyIndex) + sizeof(IsDefault) */
				tempc = cpu_to_le16(p_head->wep_key_len[i] + 1 +
						    1);
				memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
				cmd_len += 2;
				*(pos + cmd_len) = i;
				cmd_len += 1;
				*(pos + cmd_len) = (i == p_head->wep_key_idx);
				cmd_len += 1;
				temp = p_head->wep_key_len[i];
				memcpy((void *)(pos + cmd_len),
				       p_head->wep_key[i], temp);
				cmd_len += temp;
			}
		}

		if (p_head->roaming | p_head->ccx) {
			tempc = cpu_to_le16(TLV_TYPE_OFFLOAD_ENABLE);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			temp = 2;
			tempc = cpu_to_le16(temp);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += 2;
			tempc = 0;
			if (p_head->roaming)
				tempc |= PROFILE_DB_FEATURE_ROAMING;
			if (p_head->ccx)
				tempc |= PROFILE_DB_FEATURE_CCX;
			tempc = cpu_to_le16(tempc);
			memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
			cmd_len += temp;
		}

		/* Put buffer length */
		memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));
		hostcmd->size = cpu_to_le16(cmd_len);

		fprintf(stdout, "Downloading profile: %s ... ", p_head->ssid);
		fflush(stdout);

		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
		ifr.ifr_ifru.ifru_data = (void *)cmd;
		/* Perform ioctl */
		if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
			perror("ioctl[profiledb ioctl]");
			printf("ERR:Command sending failed!\n");
			ret = -EFAULT;
			goto done;
		} else {
			hostcmd->result = le16_to_cpu(hostcmd->result);
			if (hostcmd->result != 0) {
				printf("hostcmd : profiledb ioctl failure, code %d\n", hostcmd->result);
				ret = -EFAULT;
				goto done;
			}
		}

		fprintf(stdout, "done.\n");

		if (p_head)
			free(p_head);
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	if (fp)
		fclose(fp);
	return ret;
}

/********************************************************
				Global Functions
********************************************************/

/**
 *  @brief Process sub command
 *
 *  @param sub_cmd      Sub command
 *  @param num_sub_cmds Number of subcommands
 *  @param argc         Number of arguments
 *  @param argv         A pointer to arguments array
 *
 *  @return             MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_sub_cmd(sub_cmd_exec_t *sub_cmd, int num_sub_cmds,
		int argc, char *argv[])
{
	int idx;
	boolean invalid_cmd = TRUE;
	int ret = MLAN_STATUS_FAILURE;

	if (argv[3]) {
		for (idx = 0; idx < num_sub_cmds; idx++) {
			if (strncmp(argv[3],
				    sub_cmd[idx].str,
				    sub_cmd[idx].match_len) == 0) {
				invalid_cmd = FALSE;
				ret = sub_cmd[idx].callback(argc - 4, argv + 4);
				break;
			}
		}
	}

	if (invalid_cmd) {
		printf("\nUnknown %s command. Valid subcmds:\n", argv[2]);
		for (idx = 0; idx < num_sub_cmds; idx++) {
			if (sub_cmd[idx].display) {
				printf("  - %s\n", sub_cmd[idx].str);
			}
		}
		printf("\n");
	}

	return ret;
}

/**
 *  @brief  select the table's regclass
 *
 *  @param table_str  Reg channel table type
 *  @param pTable     Pointer to the Reg channel table
 *
 *  @return           TRUE if success otherwise FALSE
 */
boolean
reg_class_table_select(char *table_str, reg_chan_table_e *pTable)
{
	boolean retval = TRUE;

	if (strcmp(table_str, "user") == 0) {
		*pTable = REGTABLE_USER;
	} else if ((strcmp(table_str, "md") == 0) ||
		   (strncmp(table_str, "multidomain", 5) == 0)) {
		*pTable = REGTABLE_MULTIDOMAIN;
	} else if (strcmp(table_str, "ess") == 0) {
		*pTable = REGTABLE_ESS;
	} else if (strcmp(table_str, "default") == 0) {
		*pTable = REGTABLE_DEFAULT;
	} else {		/* If no option/wrong option set to default */
		*pTable = REGTABLE_DEFAULT;
	}

	return retval;
}

/**
 *  @brief Issue a measurement timing command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_measurement(int argc, char *argv[])
{
	int ret = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;
	HostCmd_DS_MEASUREMENT_Timing *timing_cmd = NULL;
	MrvlIETypes_MeasTiming_t *timing_tlv = NULL;
	int idx, rsp_len;
	t_u8 sel = 0;
	t_u16 tlv_len = 0;
	timing_sel_t sel_str[] = { {"disconnected", 1},
	{"adhoc", 1},
	{"fullpower", 1},
	{"ieeeps", 1},
	{"periodic", 1}
	};

	if ((argc < 4) || strncmp(argv[3], "timing",
				  MAX(strlen("timing"), strlen(argv[3])))) {
		printf("\nUnknown %s command. Valid subcmd: timing \n",
		       argv[2]);
		return MLAN_STATUS_FAILURE;
	}

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
	hostcmd->command = cpu_to_le16(HostCmd_CMD_MEASUREMENT_TIMING_CONFIG);
	hostcmd->size = cmd_len;
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	timing_cmd = (HostCmd_DS_MEASUREMENT_Timing *)pos;
	timing_cmd->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	timing_tlv = (MrvlIETypes_MeasTiming_t *)timing_cmd->tlv_buffer;

	if (argc == 7) {
		timing_cmd->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
		timing_tlv->header.type =
			cpu_to_le16(TLV_TYPE_MEASUREMENT_TIMING);
		timing_tlv->header.len =
			cpu_to_le16(sizeof(MrvlIETypes_MeasTiming_t)
				    - sizeof(timing_tlv->header));

		for (idx = 1; (unsigned int)idx < NELEMENTS(sel_str); idx++) {
			if (strncmp
			    (argv[4], sel_str[idx].str,
			     sel_str[idx].match_len) == 0) {
				sel = idx + 1;
				break;
			}
		}

		if (idx == NELEMENTS(sel_str)) {
			printf("Wrong argument for mode selected \"%s\"\n",
			       argv[4]);
			ret = -EINVAL;
			goto done;
		}

		timing_tlv->mode = cpu_to_le32(sel);
		timing_tlv->max_off_channel = cpu_to_le32(atoi(argv[5]));
		timing_tlv->max_on_channel = cpu_to_le32(atoi(argv[6]));
		cmd_len += sizeof(MrvlIETypes_MeasTiming_t);
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));
	hostcmd->size = cpu_to_le16(cmd_len);

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[measurement timing ioctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	printf("--------------------------------------------------\n");
	printf("%44s\n", "Measurement Timing Profiles (in ms)");
	printf("--------------------------------------------------\n");
	printf("     Profile    |  MaxOffChannel |  MaxOnChannel\n");
	printf("--------------------------------------------------\n");

	/* Changed to TLV parsing */
	rsp_len = le16_to_cpu(hostcmd->size);
	rsp_len -= (S_DS_GEN + sizeof(t_u16));
	pos = (t_u8 *)hostcmd + S_DS_GEN + sizeof(t_u16);
	while ((unsigned int)rsp_len > sizeof(MrvlIEtypesHeader_t)) {
		switch (le16_to_cpu(*(t_u16 *)(pos))) {
		case TLV_TYPE_MEASUREMENT_TIMING:
			timing_tlv = (MrvlIETypes_MeasTiming_t *)pos;
			tlv_len = le16_to_cpu(timing_tlv->header.len);
			printf("%15s | %14d | %13d\n",
			       sel_str[le32_to_cpu(timing_tlv->mode) - 1].str,
			       (int)le32_to_cpu(timing_tlv->max_off_channel),
			       (int)le32_to_cpu(timing_tlv->max_on_channel));
			break;
		}
		pos += tlv_len + sizeof(MrvlIEtypesHeader_t);
		rsp_len -= tlv_len + sizeof(MrvlIEtypesHeader_t);
		rsp_len = (rsp_len > 0) ? rsp_len : 0;
	}
	printf("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a profile command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_profile_entry(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	unsigned int mac[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	int idx;
	t_u16 temp, tempc;
	char *ssid = NULL;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd = NULL;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	if (argc < 4) {
		fprintf(stderr, "Invalid number of argument!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (!strncmp(argv[3], "delete", sizeof("delete"))) {
		if (argc > 4) {
			if (strncmp(argv[4], "bssid=", strlen("bssid=")) == 0) {
				/* "bssid" token string handler */
				sscanf(argv[4] + strlen("bssid="),
				       "%2x:%2x:%2x:%2x:%2x:%2x", mac + 0,
				       mac + 1, mac + 2, mac + 3, mac + 4,
				       mac + 5);
			} else if (strncmp(argv[4], "ssid=", strlen("ssid=")) ==
				   0) {
				/* "ssid" token string handler */
				ssid = argv[4] + strlen("ssid=");
			} else {
				printf("Error: missing required option for command (ssid, bssid)\n");
				ret = -ENOMEM;
				goto done;
			}
			printf("Driver profile delete request\n");
		} else {
			printf("Error: missing required option for command (ssid, bssid)\n");
			ret = -ENOMEM;
			goto done;
		}
	} else if (!strncmp(argv[3], "flush", sizeof("flush"))) {
		printf("Driver profile flush request\n");
	} else {
		ret = profile_read_download(argv[3]);
		goto done;
	}

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = -ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = -ENOMEM;
		goto done;
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
	hostcmd->command = cpu_to_le16(HostCmd_CMD_PROFILE_DB);
	hostcmd->size = 0;
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd;
	cmd_len = S_DS_GEN;

	/* set action as del */
	tempc = cpu_to_le16(HostCmd_ACT_GEN_REMOVE);
	memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
	cmd_len += 2;

	/* ssid */
	if (ssid) {
		printf("For ssid %s\n", ssid);
		tempc = cpu_to_le16(TLV_TYPE_SSID);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = strlen((char *)ssid);
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		memcpy((void *)(pos + cmd_len), ssid, temp);
		cmd_len += temp;
	} else {
		/* bssid */
		if (mac[0] != 0xFF) {
			printf("For bssid %02x:%02x:%02x:%02x:%02x:%02x\n",
			       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		}
		tempc = cpu_to_le16(TLV_TYPE_BSSID);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		temp = ETH_ALEN;
		tempc = cpu_to_le16(temp);
		memcpy((pos + cmd_len), &tempc, sizeof(t_u16));
		cmd_len += 2;
		for (idx = 0; (unsigned int)idx < NELEMENTS(mac); idx++) {
			pos[cmd_len + idx] = (t_u8)mac[idx];
		}
		cmd_len += temp;
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));
	hostcmd->size = cpu_to_le16(cmd_len);

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[profiledb ioctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	} else {
		hostcmd->result = le16_to_cpu(hostcmd->result);
		if (hostcmd->result != 0) {
			printf("hostcmd : profiledb ioctl failure, code %d\n",
			       hostcmd->result);
			ret = -EFAULT;
			goto done;
		}
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a chan report command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_chanrpt(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int respLen;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	t_u8 *pByte;
	t_u8 numBins;
	t_u8 idx;
	MrvlIEtypes_Data_t *pTlvHdr;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CHAN_RPT_RSP *pChanRptRsp;
	HostCmd_DS_CHAN_RPT_REQ *pChanRptReq;

	MrvlIEtypes_ChanRptBcn_t *pBcnRpt;
	MrvlIEtypes_ChanRptChanLoad_t *pLoadRpt;
	MrvlIEtypes_ChanRptNoiseHist_t *pNoiseRpt;
	MrvlIEtypes_ChanRpt11hBasic_t *pBasicRpt;
	MrvlIEtypes_ChanRptFrame_t *pFrameRpt;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_CHAN_RPT_REQ);

	hostcmd->command = cpu_to_le16(HostCmd_CMD_CHAN_REPORT_REQUEST);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	pChanRptReq = (HostCmd_DS_CHAN_RPT_REQ *)pos;
	pChanRptRsp = (HostCmd_DS_CHAN_RPT_RSP *)pos;

	memset((void *)pChanRptReq, 0x00, sizeof(HostCmd_DS_CHAN_RPT_REQ));

	if ((argc != 5) && (argc != 6)) {
		printf("\nchanrpt syntax: chanrpt <chan#> <millisecs> [sFreq]\n\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	pChanRptReq->chanDesc.chanNum = atoi(argv[3]);
	pChanRptReq->millisecDwellTime = cpu_to_le32(atoi(argv[4]));

	if (argc == 6) {
		pChanRptReq->chanDesc.startFreq = cpu_to_le16(atoi(argv[5]));
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[chanrpt hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	/* TSF is a t_u64, some formatted printing libs have
	 *   trouble printing long longs, so cast and dump as bytes
	 */
	pByte = (t_u8 *)&pChanRptRsp->startTsf;

	printf("\n");
	printf("[%03d]      TSF: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
	       atoi(argv[3]),
	       pByte[7], pByte[6], pByte[5], pByte[4],
	       pByte[3], pByte[2], pByte[1], pByte[0]);
	printf("[%03d]    Dwell: %u us\n",
	       atoi(argv[3]), (unsigned int)le32_to_cpu(pChanRptRsp->duration));

	pByte = pChanRptRsp->tlvBuffer;

	respLen = le16_to_cpu(hostcmd->size) - sizeof(HostCmd_DS_GEN);

	respLen -= sizeof(pChanRptRsp->commandResult);
	respLen -= sizeof(pChanRptRsp->startTsf);
	respLen -= sizeof(pChanRptRsp->duration);

	pByte = pChanRptRsp->tlvBuffer;

	while ((unsigned int)respLen >= sizeof(pTlvHdr->header)) {
		pTlvHdr = (MrvlIEtypes_Data_t *)pByte;
		pTlvHdr->header.len = le16_to_cpu(pTlvHdr->header.len);

		switch (le16_to_cpu(pTlvHdr->header.type)) {
		case TLV_TYPE_CHANRPT_BCN:
			pBcnRpt = (MrvlIEtypes_ChanRptBcn_t *)pTlvHdr;
			printf("[%03d]   Beacon: scanReqId = %d\n",
			       atoi(argv[3]), pBcnRpt->scanReqId);

			break;

		case TLV_TYPE_CHANRPT_CHAN_LOAD:
			pLoadRpt = (MrvlIEtypes_ChanRptChanLoad_t *)pTlvHdr;
			printf("[%03d] ChanLoad: %d%%\n",
			       atoi(argv[3]),
			       (pLoadRpt->ccaBusyFraction * 100) / 255);
			break;

		case TLV_TYPE_CHANRPT_NOISE_HIST:
			pNoiseRpt = (MrvlIEtypes_ChanRptNoiseHist_t *)pTlvHdr;
			numBins =
				pNoiseRpt->header.len - sizeof(pNoiseRpt->anpi);
			printf("[%03d]     ANPI: %d dB\n", atoi(argv[3]),
			       le16_to_cpu(pNoiseRpt->anpi));
			printf("[%03d] NoiseHst:", atoi(argv[3]));
			for (idx = 0; idx < numBins; idx++) {
				printf(" %03d", pNoiseRpt->rpiDensities[idx]);
			}
			printf("\n");
			break;

		case TLV_TYPE_CHANRPT_11H_BASIC:
			pBasicRpt = (MrvlIEtypes_ChanRpt11hBasic_t *)pTlvHdr;
			printf("[%03d] 11hBasic: BSS(%d), OFDM(%d), UnId(%d), Radar(%d): " "[0x%02x]\n", atoi(argv[3]), pBasicRpt->map.BSS, pBasicRpt->map.OFDM_Preamble, pBasicRpt->map.Unidentified, pBasicRpt->map.Radar, *(t_u8 *)&pBasicRpt->map);
			break;

		case TLV_TYPE_CHANRPT_FRAME:
			pFrameRpt = (MrvlIEtypes_ChanRptFrame_t *)pTlvHdr;
			printf("[%03d]    Frame: %02x:%02x:%02x:%02x:%02x:%02x  " "%02x:%02x:%02x:%02x:%02x:%02x  %3d   %02d\n", atoi(argv[3]), pFrameRpt->sourceAddr[0], pFrameRpt->sourceAddr[1], pFrameRpt->sourceAddr[2], pFrameRpt->sourceAddr[3], pFrameRpt->sourceAddr[4], pFrameRpt->sourceAddr[5], pFrameRpt->bssid[0], pFrameRpt->bssid[1], pFrameRpt->bssid[2], pFrameRpt->bssid[3], pFrameRpt->bssid[4], pFrameRpt->bssid[5], pFrameRpt->rssi, pFrameRpt->frameCnt);
			break;

		default:
			printf("[%03d] Other: Id=0x%x, Size = %d\n",
			       atoi(argv[3]),
			       pTlvHdr->header.type, pTlvHdr->header.len);

			break;
		}

		pByte += (pTlvHdr->header.len + sizeof(pTlvHdr->header));
		respLen -= (pTlvHdr->header.len + sizeof(pTlvHdr->header));
		respLen = (respLen > 0) ? respLen : 0;
	}

	printf("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a assoc timing command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_assoc_timing(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_AssociationTiming_t *assoctiming;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_AssociationTiming_t);

	hostcmd->command = cpu_to_le16(HostCmd_CMD_ASSOCIATION_TIMING);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	assoctiming = (HostCmd_DS_AssociationTiming_t *)pos;
	assoctiming->Action = cpu_to_le16(HostCmd_ACT_GEN_GET);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (argc > 3) {
		assoctiming->Action = cpu_to_le16(HostCmd_ACT_GEN_SET);
		switch (argc) {
		case 9:
			assoctiming->ReassocDiscMax =
				cpu_to_le16(atoi(argv[8]));
			/* No break, do everything below as well */
		case 8:
			assoctiming->PriorApDeauthDelay =
				cpu_to_le16(atoi(argv[7]));
			/* No break, do everything below as well */
		case 7:
			assoctiming->FrameExchangeTimeout =
				cpu_to_le16(atoi(argv[6]));
			/* No break, do everything below as well */
		case 6:
			assoctiming->HandShakeTimeout =
				cpu_to_le16(atoi(argv[5]));
			/* No break, do everything below as well */
		case 5:
			assoctiming->ReassocTimeout =
				cpu_to_le16(atoi(argv[4]));
			/* No break, do everything below as well */
		case 4:
			assoctiming->AssocTimeout = cpu_to_le16(atoi(argv[3]));
			break;
		}
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	puts("");
	printf("------------------------------------------------\n");
	printf("        Association Timing Parameters\n");
	printf("------------------------------------------------\n");

	printf("Association Timeout     %5u ms\n"
	       "Reassociation Timeout   %5u ms\n"
	       "Handshake Timeout       %5u ms\n"
	       "Frame Exchange Timeout  %5u ms\n"
	       "Prior AP Deauth Delay   %5u ms\n"
	       "Reassoc Disconnect Max  %5u ms\n",
	       le16_to_cpu(assoctiming->AssocTimeout),
	       le16_to_cpu(assoctiming->ReassocTimeout),
	       le16_to_cpu(assoctiming->HandShakeTimeout),
	       le16_to_cpu(assoctiming->FrameExchangeTimeout),
	       le16_to_cpu(assoctiming->PriorApDeauthDelay),
	       le16_to_cpu(assoctiming->ReassocDiscMax));
	puts("");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Retrieve the association response from the driver
 *
 *  Retrieve the buffered (re)association management frame from the driver.
 *    The response is identical to the one received from the AP and conforms
 *    to the IEEE specification.
 *
 *  @return MLAN_STATUS_SUCCESS or ioctl error code
 */
int
process_get_assocrsp(int argc, char *argv[])
{
	int ret = 0;
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	IEEEtypes_AssocRsp_t *pAssocRsp = NULL;

	/* Initialize buffer */
	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return MLAN_STATUS_FAILURE;
	}

	pAssocRsp = (IEEEtypes_AssocRsp_t *)buffer;

	/* buffer = MRVL_CMD<cmd> */
	strncpy((char *)buffer, CMD_NXP, strlen(CMD_NXP));
	strncpy((char *)buffer + strlen(CMD_NXP), argv[2], strlen(argv[2]));

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		free(buffer);
		return MLAN_STATUS_FAILURE;
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

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlanutl");
		fprintf(stderr, "mlanutl: version fail\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (cmd->used_len) {
		printf("getassocrsp: Status[%d], Cap[0x%04x]:\n",
		       pAssocRsp->StatusCode,
		       le16_to_cpu(*(t_u16 *)&pAssocRsp->Capability));
		hexdump(NULL, buffer, cmd->used_len, ' ');
	} else {
		printf("getassocrsp: <empty>\n");
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);

	return ret;
}

/*
**  Process mlanutl fcontrol command:
**
**    mlanutl mlanX fcontrol %d [0xAA 0xBB... ]
**
**  Sets and/or retrieves the feature control settings for a specific
**    control set (argv[3] decimal argument).
**
*/
int
process_fcontrol(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	t_u8 idx;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_OFFLOAD_FEATURE_CONTROL *pFcontrol;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	pFcontrol = (HostCmd_DS_OFFLOAD_FEATURE_CONTROL *)pos;

	if (argc < 4) {
		printf("Wrong number of arguments\n");
		ret = -EINVAL;
		goto done;
	}

	pFcontrol->controlSelect = atoi(argv[3]);
	cmd_len = S_DS_GEN + sizeof(pFcontrol->controlSelect);

	for (idx = 4; idx < argc; idx++) {
		pFcontrol->controlBitmap[idx - 4] = a2hex_or_atoi(argv[idx]);
		cmd_len++;
	}

	hostcmd->command = cpu_to_le16(HostCmd_CMD_OFFLOAD_FEATURE_CONTROL);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[fcontrol hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	cmd_len = (le16_to_cpu(hostcmd->size) - sizeof(HostCmd_DS_GEN));

	printf("Control[%d]", pFcontrol->controlSelect);
	cmd_len--;

	for (idx = 0; idx < cmd_len; idx++) {
		printf("\t0x%02x", pFcontrol->controlBitmap[idx]);
	}

	printf("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/*
**  Process mlanutl iapp command:
**
**    mlanutl mlanX iapp <timeout> 0xAA 0xBB [0x... 0x.. ]
**
**    0xAA = IAPP type
**    0xBB = IAPP subtype
**    0x.. = Remaning bytes are iapp data
**
*/
int
process_iapp(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	t_u8 idx;
	t_u8 fixlen;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_IAPP_PROXY *pIappProxy;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_IAPP_PROXY);

	pIappProxy = (HostCmd_DS_IAPP_PROXY *)pos;

	if (argc < 6) {
		printf("Wrong number of arguments\n");
		ret = -EINVAL;
	}

	memset(pIappProxy, 0x00, sizeof(HostCmd_DS_IAPP_PROXY));

	pIappProxy->iappType = a2hex_or_atoi(argv[4]);
	pIappProxy->iappSubType = a2hex_or_atoi(argv[5]);

	/* Fixed len portions of command */
	fixlen = (S_DS_GEN + sizeof(HostCmd_DS_IAPP_PROXY)
		  - sizeof(pIappProxy->iappData));

	pIappProxy->timeout_ms = cpu_to_le32(a2hex_or_atoi(argv[3]));

	for (idx = 6; idx < argc; idx++) {
		pIappProxy->iappData[idx - 6] = a2hex_or_atoi(argv[idx]);
		pIappProxy->iappDataLen++;
	}

	hostcmd->command = cpu_to_le16(HostCmd_CMD_IAPP_PROXY);
	hostcmd->size = cpu_to_le16(fixlen + pIappProxy->iappDataLen);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	pIappProxy->iappDataLen = cpu_to_le32(pIappProxy->iappDataLen);
	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[iapp hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	printf("\nResult:   %d\n", le32_to_cpu(pIappProxy->commandResult));
	printf("Type:     0x%02x\n", pIappProxy->iappType);
	printf("SubType:  0x%02x\n", pIappProxy->iappSubType);

	printf("IappData: ");
	hexdump(NULL, pIappProxy->iappData,
		le32_to_cpu(pIappProxy->iappDataLen), ' ');
	printf("\n\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a rf tx power command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_rf_tx_power(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_802_11_RF_TX_POWER *pRfTxPower;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_802_11_RF_TX_POWER);

	pRfTxPower = (HostCmd_DS_802_11_RF_TX_POWER *)pos;

	memset(pRfTxPower, 0x00, sizeof(HostCmd_DS_802_11_RF_TX_POWER));

	hostcmd->command = cpu_to_le16(HostCmd_CMD_802_11_RF_TX_POWER);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	printf("\n");
	printf("  MinPower:  %2d\n", pRfTxPower->min_power);
	printf("  MaxPower:  %2d\n", pRfTxPower->max_power);
	printf("  Current:   %2d\n", le16_to_cpu(pRfTxPower->current_level));
	printf("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a authenticate command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_authenticate(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_802_11_AUTHENTICATE *pAuth;
	unsigned int mac[ETH_ALEN];
	int tmpIdx;

	if (argc != 4) {
		printf("Wrong number of arguments\n");
		ret = -EINVAL;
		goto done;
	}

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_802_11_AUTHENTICATE);

	pAuth = (HostCmd_DS_802_11_AUTHENTICATE *)pos;

	memset(pAuth, 0x00, sizeof(HostCmd_DS_802_11_AUTHENTICATE));

	sscanf(argv[3],
	       "%2x:%2x:%2x:%2x:%2x:%2x",
	       mac + 0, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5);

	for (tmpIdx = 0; (unsigned int)tmpIdx < NELEMENTS(mac); tmpIdx++) {
		pAuth->MacAddr[tmpIdx] = (t_u8)mac[tmpIdx];
	}

	hostcmd->command = cpu_to_le16(HostCmd_CMD_802_11_AUTHENTICATE);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[hostcmd]");
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

static void
display_channel(void)
{
	FILE *tmpfile;
	char result[200];
	char cmdStr[50];
	int ghz, mhz, chan;

	puts("\n");

	sprintf(cmdStr, "iwlist %s chan", dev_name);

	tmpfile = popen(cmdStr, "r");

	if (tmpfile == NULL) {
		perror("mlanutl: iwlist failed to get current channel");
	} else {
		while (fgets(result, sizeof(result), tmpfile)) {
			if ((sscanf
			     (result, " Current Frequency=%d.%d ", &ghz,
			      &mhz) == 2) ||
			    (sscanf
			     (result, " Current Frequency:%d.%d ", &ghz,
			      &mhz) == 2)) {
				if (mhz < 10) {
					mhz *= 100;
				} else if (mhz < 100) {
					mhz *= 10;
				}

				chan = ghz * 1000 + mhz;
				if (chan > 5000) {
					chan -= 5000;
					chan /= 5;
				} else if (chan == 2484) {
					chan = 14;
				} else {
					chan -= 2407;
					chan /= 5;
				}
				printf("   Channel: %3d [%d.%d GHz]\n", chan,
				       ghz, mhz);
			}
		}
		pclose(tmpfile);
	}
}

static char *
get_ratestr(int txRate)
{
	char *pStr;

	switch (txRate) {
	case 0:
		pStr = "1";
		break;
	case 1:
		pStr = "2";
		break;
	case 2:
		pStr = "5.5";
		break;
	case 3:
		pStr = "11";
		break;
	case 4:
		pStr = "6";
		break;
	case 5:
		pStr = "9";
		break;
	case 6:
		pStr = "12";
		break;
	case 7:
		pStr = "18";
		break;
	case 8:
		pStr = "24";
		break;
	case 9:
		pStr = "36";
		break;
	case 10:
		pStr = "48";
		break;
	case 11:
		pStr = "54";
		break;
	case 12:
		pStr = "MCS0";
		break;
	case 13:
		pStr = "MCS1";
		break;
	case 14:
		pStr = "MCS2";
		break;
	case 15:
		pStr = "MCS3";
		break;
	case 16:
		pStr = "MCS4";
		break;
	case 17:
		pStr = "MCS5";
		break;
	case 18:
		pStr = "MCS6";
		break;
	case 19:
		pStr = "MCS7";
		break;

	case 140:
		pStr = "MCS0";
		break;
	case 141:
		pStr = "MCS1";
		break;
	case 142:
		pStr = "MCS2";
		break;
	case 143:
		pStr = "MCS3";
		break;
	case 144:
		pStr = "MCS4";
		break;
	case 145:
		pStr = "MCS5";
		break;
	case 146:
		pStr = "MCS6";
		break;
	case 147:
		pStr = "MCS7";
		break;

	default:
		pStr = "Unkn";
		break;
	}

	return pStr;
}

typedef struct {
	int rate;
	int min;
	int max;

} RatePower_t;

static int
get_txpwrcfg(RatePower_t ratePower[])
{
	FILE *tmpfile;
	char result[300];
	char cmdStr[50];
	int counter = 0;
	char *pBuf;
	int r1 = 0, r2 = 0, min = 0, max = 0, rate = 0;
	int rateIdx = 0;

	sprintf(cmdStr, "iwpriv %s txpowercfg", dev_name);

	tmpfile = popen(cmdStr, "r");

	if (tmpfile == NULL) {
		perror("mlanutl: iwpriv failed to get txpowercfg");
	} else {
		while (fgets(result, sizeof(result), tmpfile)) {
			pBuf = strtok(result, ": ");

			while (pBuf != NULL) {
				switch (counter % 5) {
				case 0:
					r1 = atoi(pBuf);
					break;

				case 1:
					r2 = atoi(pBuf);
					break;

				case 2:
					min = atoi(pBuf);
					break;

				case 3:
					max = atoi(pBuf);
					break;

				case 4:
					for (rate = r1; rate <= r2; rate++) {
						ratePower[rateIdx].rate = rate;
						ratePower[rateIdx].min = min;
						ratePower[rateIdx].max = max;
						rateIdx++;
					}
					break;
				}

				if (isdigit(*pBuf)) {
					counter++;
				}
				pBuf = strtok(NULL, ": ");
			}
		}
		pclose(tmpfile);
	}

	return rateIdx;
}

static void
rateSort(RatePower_t rateList[], int numRates)
{
	int inc, i, j;
	RatePower_t tmp;

	inc = 3;

	while (inc > 0) {
		for (i = 0; i < numRates; i++) {
			j = i;
			memcpy(&tmp, &rateList[i], sizeof(RatePower_t));

			while ((j >= inc) &&
			       (rateList[j - inc].rate > tmp.rate)) {
				memcpy(&rateList[j], &rateList[j - inc],
				       sizeof(RatePower_t));
				j -= inc;
			}

			memcpy(&rateList[j], &tmp, sizeof(RatePower_t));
		}

		if (inc >> 1) {
			inc >>= 1;
		} else if (inc == 1) {
			inc = 0;
		} else {
			inc = 1;
		}
	}
}

typedef struct {
	int rate;
	int modGroup;

} RateModPair_t;

/*
**
** ModulationGroups
**    0: CCK (1,2,5.5,11 Mbps)
**    1: OFDM (6,9,12,18 Mbps)
**    2: OFDM (24,36 Mbps)
**    3: OFDM (48,54 Mbps)
**    4: HT20 (0,1,2)
**    5: HT20 (3,4)
**    6: HT20 (5,6,7)
**    7: HT40 (0,1,2)
**    8: HT40 (3,4)
**    9: HT40 (5,6,7)
*/

static RateModPair_t rateModPairs[] = {
	{0, 0},			/* 1 */
	{1, 0},			/* 2 */
	{2, 0},			/* 5.5 */
	{3, 0},			/* 11 */
	{4, 1},			/* 6 */
	{5, 1},			/* 9 */
	{6, 1},			/* 12 */
	{7, 1},			/* 18 */
	{8, 2},			/* 24 */
	{9, 2},			/* 36 */
	{10, 3},		/* 48 */
	{11, 3},		/* 54 */
	{12, 4},		/* MCS0 */
	{13, 4},		/* MCS1 */
	{14, 4},		/* MCS2 */
	{15, 5},		/* MCS3 */
	{16, 5},		/* MCS4 */
	{17, 6},		/* MCS5 */
	{18, 6},		/* MCS6 */
	{19, 6},		/* MCS7 */

	{140, 7},		/* MCS0 */
	{141, 7},		/* MCS1 */
	{142, 7},		/* MCS2 */
	{143, 8},		/* MCS3 */
	{144, 8},		/* MCS4 */
	{145, 9},		/* MCS5 */
	{146, 9},		/* MCS6 */
	{147, 9},		/* MCS7 */
};

int
process_chantrpcdisp(int startRate, int endRate)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CHAN_TRPC_CONFIG *pChanTrpc;
	MrvlIEtypes_ChanTrpcCfg_t *pChanTrpcTlv;
	int totalTlvBytes = 0;

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	pChanTrpc = (HostCmd_DS_CHAN_TRPC_CONFIG *)pos;
	pChanTrpc->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

	cmd_len = S_DS_GEN + sizeof(pChanTrpc->action);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_CHAN_TRPC_CONFIG);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[chantrpc hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	totalTlvBytes = (le16_to_cpu(hostcmd->size)
			 - sizeof(HostCmd_DS_GEN)
			 - sizeof(pChanTrpc->action)
			 - sizeof(pChanTrpc->reserved));

	pChanTrpcTlv = (MrvlIEtypes_ChanTrpcCfg_t *)pChanTrpc->tlv_buffer;

	while (totalTlvBytes) {
		int tlvSize, numModGroups, idx, modIdx, numOut;

		/* Switch to TLV parsing */
		printf("%4d.%-3d ",
		       le16_to_cpu(pChanTrpcTlv->chanDesc.startFreq),
		       pChanTrpcTlv->chanDesc.chanNum);

		numOut = 0;

		tlvSize = (le16_to_cpu(pChanTrpcTlv->header.len)
			   + sizeof(pChanTrpcTlv->header));

		numModGroups = (le16_to_cpu(pChanTrpcTlv->header.len)
				- sizeof(pChanTrpcTlv->chanDesc));
		numModGroups /= sizeof(pChanTrpcTlv->chanTrpcEntry[0]);

		for (idx = 0; idx < NELEMENTS(rateModPairs); idx++) {
			if ((rateModPairs[idx].rate >= startRate) &&
			    (rateModPairs[idx].rate <= endRate)) {
				for (modIdx = 0; modIdx < numModGroups;
				     modIdx++) {
					if (rateModPairs[idx].modGroup ==
					    pChanTrpcTlv->chanTrpcEntry[modIdx].
					    modGroup) {
						printf("%*d",
						       (numOut == 0) ? 3 : 6,
						       pChanTrpcTlv->
						       chanTrpcEntry[modIdx].
						       txPower);
						numOut++;
					}
				}

				if (numOut == 0) {
					printf(" --   ");
				}
			}
		}

		puts("");

		pChanTrpcTlv =
			(MrvlIEtypes_ChanTrpcCfg_t *)((t_u8 *)pChanTrpcTlv +
						      tlvSize);
		totalTlvBytes -= tlvSize;
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a tx power display command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_txpowdisp(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int rateIdx, rates;
	int connected;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_802_11_RF_TX_POWER *pRfTxPower;
	RatePower_t ratePower[50];

	cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

	buffer = (t_u8 *)malloc(BUFFER_LENGTH);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot alloc memory\n");
		ret = ENOMEM;
		goto done;
	}
	memset(buffer, 0, BUFFER_LENGTH);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = ENOMEM;
		goto done;
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_802_11_RF_TX_POWER);

	pRfTxPower = (HostCmd_DS_802_11_RF_TX_POWER *)pos;

	memset(pRfTxPower, 0x00, sizeof(HostCmd_DS_802_11_RF_TX_POWER));

	hostcmd->command = cpu_to_le16(HostCmd_CMD_802_11_RF_TX_POWER);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[hostcmd]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	connected = le16_to_cpu(pRfTxPower->current_level) ? TRUE : FALSE;

	if (connected) {
		display_channel();

		printf("\n");
		printf("  MinPower:  %2d\n", pRfTxPower->min_power);
		printf("  MaxPower:  %2d\n", pRfTxPower->max_power);
		printf("  Current:   %2d\n",
		       le16_to_cpu(pRfTxPower->current_level));
		printf("\n");
	}

	rates = get_txpwrcfg(ratePower);

	puts("");

	rateSort(ratePower, rates);

	printf("20MHz:");

	for (rateIdx = 0; rateIdx < 12; rateIdx++) {
		printf("%6s", get_ratestr(ratePower[rateIdx].rate));
	}

	printf("\n---------------------------------------"
	       "----------------------------------------\n%s",
	       connected ? "Active" : "Max   ");

	for (rateIdx = 0; rateIdx < 12; rateIdx++) {
		printf("%6d", ratePower[rateIdx].max);
	}

	if (!connected) {
		printf("\n---------------------------------------"
		       "----------------------------------------\n");

		process_chantrpcdisp(ratePower[0].rate, ratePower[12 - 1].rate);
	}

	puts("\n");

	/*
	 ** MCS0 -> MCS7
	 */

	printf("20MHz:");

	for (rateIdx = 12; rateIdx < 20; rateIdx++) {
		printf("%6s", get_ratestr(ratePower[rateIdx].rate));
	}

	printf("\n---------------------------------------"
	       "----------------------------------------\n%s",
	       connected ? "Active" : "Max   ");

	for (rateIdx = 12; rateIdx < 20; rateIdx++) {
		printf("%6d", ratePower[rateIdx].max);
	}

	if (!connected) {
		printf("\n---------------------------------------"
		       "----------------------------------------\n");

		process_chantrpcdisp(ratePower[12].rate,
				     ratePower[20 - 1].rate);
	}

	puts("\n");

	/*
	 ** MCS0 -> MCS7 @ 40MHz
	 */

	printf("40MHz:");

	for (rateIdx = 20; rateIdx < rates; rateIdx++) {
		printf("%6s", get_ratestr(ratePower[rateIdx].rate));
	}

	printf("\n---------------------------------------"
	       "----------------------------------------\n%s",
	       connected ? "Active" : "Max   ");

	for (rateIdx = 20; rateIdx < rates; rateIdx++) {
		printf("%6d", ratePower[rateIdx].max);
	}

	if (!connected) {
		printf("\n---------------------------------------"
		       "----------------------------------------\n");

		process_chantrpcdisp(ratePower[20].rate,
				     ratePower[rates - 1].rate);
	}

	puts("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}
