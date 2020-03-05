/** @file  mlanregclass.c
  *
  * @brief This files contains mlanutl regclass command handling.
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
#include    "mlanregclass.h"

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
 *  @brief Convert reg domain number to string
 *
 *  @param reg_domain Reg Domain
 *
 *  @return           Reg Domain type
 */
static char *
reg_domain_to_str(reg_domain_e reg_domain)
{
	switch (reg_domain) {
	case REGDOMAIN_FCC:
		return "FCC";

	case REGDOMAIN_ETSI:
		return "ETSI";

	case REGDOMAIN_MIC:
		return "MIC";

	case REGDOMAIN_OTHER:
		return "MULTI";

	default:
		break;
	}

	return "UNKN";
}

/**
 *  @brief Convert reg channel table number to string
 *
 *  @param table_select Reg channel table
 *
 *  @return             Reg channel table type
 */
static char *
table_num_to_str(reg_chan_table_e table_select)
{
	switch (table_select) {
	case REGTABLE_USER:
		return "User";

	case REGTABLE_MULTIDOMAIN:
		return "MultiDomain";

	case REGTABLE_ESS:
		return "ESS";

	case REGTABLE_DEFAULT:
		return "Default";

	default:
		break;
	}

	return "UNKN";
}

/**
 *  @brief      Regclass dump channel table
 *
 *  @param argc     Number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
reg_class_dump_chan_table(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_REGCLASS_GET_CHAN_TABLE *get_table;
	int idx;
	t_u16 regLimits;
	boolean invalid_cmd = FALSE;

	printf("ERR:Cannot allocate buffer for command!\n");
	if (argv[0] == NULL) {
		invalid_cmd = TRUE;
	} else {

		cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

		buffer = (t_u8 *)malloc(BUFFER_LENGTH);
		if (buffer == NULL) {
			fprintf(stderr, "Cannot alloc memory\n");
			ret = ENOMEM;
			goto done;
		}
		memset(buffer, 0, BUFFER_LENGTH);

		cmd = (struct eth_priv_cmd *)
			malloc(sizeof(struct eth_priv_cmd));
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
		strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD,
			strlen(HOSTCMD));

		/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
		hostcmd =
			(HostCmd_DS_GEN *)(buffer + cmd_header_len +
					   sizeof(t_u32));

		/* Point after host command header */
		pos = (t_u8 *)hostcmd + S_DS_GEN;

		cmd_len = S_DS_GEN + sizeof(HostCmd_DS_REGCLASS_GET_CHAN_TABLE);
		hostcmd->command = cpu_to_le16(HostCmd_CMD_REGCLASS_CHAN_TABLE);
		hostcmd->size = cpu_to_le16(cmd_len);
		hostcmd->seq_num = 0;
		hostcmd->result = 0;

		get_table = (HostCmd_DS_REGCLASS_GET_CHAN_TABLE *)pos;
		get_table->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

		if (reg_class_table_select(argv[0], (reg_chan_table_e *)
					   &get_table->table_select) == FALSE) {
			invalid_cmd = TRUE;
		}
	}

	if (invalid_cmd) {
		printf("\nValid tables table; valid [user, md, ess, default]\n\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	get_table->table_select = cpu_to_le16((t_u16)(get_table->table_select));

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[regClassIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (!le16_to_cpu(hostcmd->result)) {
		printf("HOSTCMD_RESP: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("HOSTCMD failed: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	}

	get_table->table_select = le16_to_cpu(get_table->table_select);
	get_table->chan = le32_to_cpu(get_table->chan);

	printf("---------------------------------------");
	printf("---------------------------------------\n");
	printf("%35s: %s [%d]\n", "Channel Table",
	       table_num_to_str(get_table->table_select), (int)get_table->chan);
	printf("---------------------------------------");
	printf("---------------------------------------\n");
	printf(" chn | freq | sfrq | sp |  class   | maxP | behavior limits\n");
	printf("---------------------------------------");
	printf("---------------------------------------\n");

	for (idx = 0; (unsigned int)idx < get_table->chan; idx++) {
		char regDisp[8];

		sprintf(regDisp, "%4s-%02u",
			reg_domain_to_str(get_table->chan_entry[idx].
					  reg_domain),
			get_table->chan_entry[idx].regulatory_class);

		printf(" %03u | %04u | %04u | %02u | %-8s |  %02u  |",
		       get_table->chan_entry[idx].chan_num,
		       (get_table->chan_entry[idx].start_freq +
			(get_table->chan_entry[idx].chan_num * 5)),
		       le16_to_cpu(get_table->chan_entry[idx].start_freq),
		       le16_to_cpu(get_table->chan_entry[idx].chan_spacing),
		       regDisp, get_table->chan_entry[idx].max_tx_power);

		regLimits = le16_to_cpu(get_table->chan_entry[idx].reg_limits);

		if (regLimits & BLIMIT_NOMADIC)
			printf(" nomadic");
		if (regLimits & BLIMIT_INDOOR_ONLY)
			printf(" indoor");
		if (regLimits & BLIMIT_TPC)
			printf(" tpc");
		if (regLimits & BLIMIT_DFS)
			printf(" dfs");
		if (regLimits & BLIMIT_IBSS_PROHIBIT)
			printf(" no_ibss");
		if (regLimits & BLIMIT_FOUR_MS_CS)
			printf(" 4ms_cs");
		if (regLimits & BLIMIT_LIC_BASE_STA)
			printf(" base_sta");
		if (regLimits & BLIMIT_MOBILE_STA)
			printf(" mobile");
		if (regLimits & BLIMIT_PUBLIC_SAFETY)
			printf(" safety");
		if (regLimits & BLIMIT_ISM_BANDS)
			printf(" ism");

		printf("\n");
	}
	printf("---------------------------------------");
	printf("---------------------------------------\n");
	printf("\n");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief      Regclass configure user table
 *
 *  @param argc     Number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
reg_class_config_user_table(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_REGCLASS_CONFIG_USER_TABLE *cfg_user_table;

	if (argv[0] == NULL) {
		printf("\nCountry string not specified\n");
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

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_REGCLASS_CONFIG_USER_TABLE);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_REGCLASS_CONFIG_USER_TABLE);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	cfg_user_table = (HostCmd_DS_REGCLASS_CONFIG_USER_TABLE *)pos;
	cfg_user_table->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	memcpy(cfg_user_table->regulatory_str,
	       argv[0],
	       MIN(strlen(argv[0]), sizeof(cfg_user_table->regulatory_str)));

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[regClassIoctl]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (!le16_to_cpu(hostcmd->result)) {
		printf("HOSTCMD_RESP: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("HOSTCMD failed: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief      Issue regclass multi-domain command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
reg_class_multidomain(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_REGCLASS_MULTIDOMAIN_CONTROL *multidomain_ctrl;
	boolean invalid_cmd = FALSE;

	if (argv[0] == NULL) {
		invalid_cmd = TRUE;
	} else {
		cmd_header_len = strlen(CMD_NXP) + strlen(HOSTCMD);

		buffer = (t_u8 *)malloc(BUFFER_LENGTH);
		if (buffer == NULL) {
			fprintf(stderr, "Cannot alloc memory\n");
			ret = ENOMEM;
			goto done;
		}
		memset(buffer, 0, BUFFER_LENGTH);

		cmd = (struct eth_priv_cmd *)
			malloc(sizeof(struct eth_priv_cmd));
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
		strncpy((char *)buffer + strlen(CMD_NXP), HOSTCMD,
			strlen(HOSTCMD));

		/* buffer = MRVL_CMD<cmd><hostcmd_size><HostCmd_DS_GEN><CMD_DS> */
		hostcmd =
			(HostCmd_DS_GEN *)(buffer + cmd_header_len +
					   sizeof(t_u32));

		/* Point after host command header */
		pos = (t_u8 *)hostcmd + S_DS_GEN;

		cmd_len =
			S_DS_GEN +
			sizeof(HostCmd_DS_REGCLASS_MULTIDOMAIN_CONTROL);
		hostcmd->command =
			cpu_to_le16(HostCmd_CMD_REGCLASS_MULTIDOMAIN_CONTROL);
		hostcmd->size = cpu_to_le16(cmd_len);
		hostcmd->seq_num = 0;
		hostcmd->result = 0;

		multidomain_ctrl =
			(HostCmd_DS_REGCLASS_MULTIDOMAIN_CONTROL *)pos;
		if (strcmp(argv[0], "on") == 0) {
			multidomain_ctrl->multidomain_enable = 1;
		} else if (strcmp(argv[0], "off") == 0) {
			multidomain_ctrl->multidomain_enable = 0;
		} else {
			invalid_cmd = TRUE;
		}
	}

	if (invalid_cmd) {
		printf("\nUnknown multiDomain command; valid [on, off]\n\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	multidomain_ctrl->multidomain_enable =
		cpu_to_le32(multidomain_ctrl->multidomain_enable);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[regClass]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	} else {
		printf("\nMultiDomain: %s\n",
		       le32_to_cpu(multidomain_ctrl->multidomain_enable) ?
		       "Enabled" : "Disabled");
	}

	if (!le16_to_cpu(hostcmd->result)) {
		printf("HOSTCMD_RESP: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("HOSTCMD failed: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief Issue a regclass command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_regclass(int argc, char *argv[])
{
	sub_cmd_exec_t sub_cmd[] = { {"table", 1, 1, reg_class_dump_chan_table},
	{"multidomain", 1, 1, reg_class_multidomain},
	{"country", 1, 1, reg_class_config_user_table}
	};

	return process_sub_cmd(sub_cmd, NELEMENTS(sub_cmd), argc, argv);
}
