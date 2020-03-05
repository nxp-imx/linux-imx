/** @file  mlanroamagent.c
 *
 * @brief This files contains mlanutl roamagent command handling.
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
#include    "mlanroamagent.h"

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
 *  @brief          Issue getra failcnt command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetFailureCount(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int failCount, failTimeThresh, state, i;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_FailureCount_t *pFailureCount;
	MrvlIEtypesHeader_t *pTlvHdr;
	t_u8 *tlvptr;
	const char *states[] =
		{ "Stable", "Degrading", "Unacceptable", "Hardroam" };

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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	cmd_len =
		(S_DS_GEN +
		 sizeof(HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD)
		 - sizeof(statsthreshold->TlvBuffer));

	/* Can be extended to all states later */
	for (state = STATE_HARDROAM - 1; state < STATE_HARDROAM; state++) {
		statsthreshold->State = state + 1;
		hostcmd->command =
			cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
		hostcmd->size = cpu_to_le16(cmd_len);
		hostcmd->seq_num = 0;
		hostcmd->result = 0;
		statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

		/* Put buffer length */
		memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
		ifr.ifr_ifru.ifru_data = (void *)cmd;
		/* Perform ioctl */
		if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
			perror("ioctl[roamstatistics]");
			printf("ERR:Command sending failed!\n");
			ret = -EFAULT;
			goto done;
		}

		i = le16_to_cpu(hostcmd->size);
		i -= cmd_len;
		tlvptr = statsthreshold->TlvBuffer;

		while (i > 2) {
			pTlvHdr = (MrvlIEtypesHeader_t *)tlvptr;

			switch (le16_to_cpu(pTlvHdr->type)) {
			case TLV_TYPE_FAILCOUNT:
				pFailureCount =
					(MrvlIEtypes_FailureCount_t *)pTlvHdr;
				failCount = pFailureCount->fail_value;
				failTimeThresh =
					le16_to_cpu(pFailureCount->
						    fail_min_thresh_time_millisecs);
				break;
			}

			tlvptr += (le16_to_cpu(pTlvHdr->len)
				   + sizeof(MrvlIEtypesHeader_t));
			i -= (le16_to_cpu(pTlvHdr->len)
			      + sizeof(MrvlIEtypesHeader_t));
		}

		printf("State: %-8s. FailCount = %d, FailTimeThresh(ms) = %d\n",
		       states[state], failCount, failTimeThresh);
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue getra bcnmiss command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetPreBeaconMiss(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int bcnmiss, state = STATE_HARDROAM, i;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_BeaconsMissed_t *pBeaconMissed;
	MrvlIEtypesHeader_t *pTlvHdr;
	t_u8 *tlvptr;
	const char *states[] =
		{ "Stable", "Degrading", "Unacceptable", "Hardroam" };

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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	cmd_len =
		(S_DS_GEN +
		 sizeof(HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD)
		 - sizeof(statsthreshold->TlvBuffer));

	/* Can be extended to all states later */
	for (state = STATE_HARDROAM - 1; state < STATE_HARDROAM; state++) {
		statsthreshold->State = state + 1;
		hostcmd->command =
			cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
		hostcmd->size = cpu_to_le16(cmd_len);
		hostcmd->seq_num = 0;
		hostcmd->result = 0;
		statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

		/* Put buffer length */
		memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
		ifr.ifr_ifru.ifru_data = (void *)cmd;
		/* Perform ioctl */
		if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
			perror("ioctl[beacon miss]");
			printf("ERR:Command sending failed!\n");
			ret = -EFAULT;
			goto done;
		}

		i = le16_to_cpu(hostcmd->size);
		i -= cmd_len;
		tlvptr = statsthreshold->TlvBuffer;

		while (i > 2) {
			pTlvHdr = (MrvlIEtypesHeader_t *)tlvptr;

			switch (le16_to_cpu(pTlvHdr->type)) {

			case TLV_TYPE_PRE_BEACON_LOST:
				pBeaconMissed =
					(MrvlIEtypes_BeaconsMissed_t *)pTlvHdr;
				bcnmiss = pBeaconMissed->beacon_missed;
			}

			tlvptr += (le16_to_cpu(pTlvHdr->len)
				   + sizeof(MrvlIEtypesHeader_t));
			i -= (le16_to_cpu(pTlvHdr->len)
			      + sizeof(MrvlIEtypesHeader_t));
		}

		printf("State: %-8s. Pre Beacon missed threshold %d\n",
		       states[state], bcnmiss);
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue getra rssi/snr command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param type     RSSI/SNR threshold type
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetStatsThreshold(int argc, char *argv[], int type)
{
	int ret = MLAN_STATUS_SUCCESS;
	int state = 0, i = 0, profile = 1;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_BeaconHighRssiThreshold_t *pHighRssiThreshold;
	MrvlIEtypes_BeaconLowRssiThreshold_t *pLowRssiThreshold;
	MrvlIEtypes_BeaconHighSnrThreshold_t *pHighSnrThreshold;
	MrvlIEtypes_BeaconLowSnrThreshold_t *pLowSnrThreshold;
	t_s8 high, low;
	t_u8 *tlvptr;
	t_u16 tlv;
	const char *states[] = { "Stable", "Degrading", "Unacceptable" };

	if (argc) {
		if (strncmp(argv[0], "configured", strlen("config")) == 0) {
			profile = 0;
		} else if (strncmp(argv[0], "active", strlen("active"))) {
			printf("\nIncorrect parameter %s for getra command\n\n",
			       argv[0]);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	cmd_len =
		(S_DS_GEN +
		 sizeof(HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD)
		 - sizeof(statsthreshold->TlvBuffer));

	printf("---------------------------------------------\n");
	if (type == RSSI_THRESHOLD) {
		printf("               RSSI Thresholds\n");
	} else {
		printf("                SNR Thresholds\n");
	}
	printf("---------------------------------------------\n");

	for (state = STATE_STABLE - 1; state < STATE_UNACCEPTABLE; state++) {
		statsthreshold->State = state + 1;
		statsthreshold->Profile = profile;
		hostcmd->command =
			cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
		hostcmd->size = cpu_to_le16(cmd_len);
		hostcmd->seq_num = 0;
		hostcmd->result = 0;
		statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

		/* Put buffer length */
		memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
		ifr.ifr_ifru.ifru_data = (void *)cmd;
		/* Perform ioctl */
		if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
			perror("ioctl[roamstatistics]");
			printf("ERR:Command sending failed!\n");
			ret = -EFAULT;
			goto done;
		}

		if (le16_to_cpu(hostcmd->result)) {
			printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n", le16_to_cpu(hostcmd->command), le16_to_cpu(hostcmd->result));
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		i = le16_to_cpu(hostcmd->size);
		i -= cmd_len;
		tlvptr = statsthreshold->TlvBuffer;
		while (i > 2) {
			/*
			 * ENDIANNESS for Response
			 */
			tlv = le16_to_cpu(*((t_u16 *)(tlvptr)));

			switch (tlv) {
			case TLV_TYPE_RSSI_HIGH:
				high = *(t_s8 *)(tlvptr + sizeof(t_u16) +
						 sizeof(t_u16));
				pHighRssiThreshold =
					(MrvlIEtypes_BeaconHighRssiThreshold_t
					 *)(tlvptr);
				tlvptr +=
					(le16_to_cpu
					 (pHighRssiThreshold->Header.len) +
					 sizeof(MrvlIEtypesHeader_t));
				i -= (le16_to_cpu
				      (pHighRssiThreshold->Header.len) +
				      sizeof(MrvlIEtypesHeader_t));
				break;

			case TLV_TYPE_RSSI_LOW:
				low = *(t_s8 *)(tlvptr + sizeof(t_u16) +
						sizeof(t_u16));
				pLowRssiThreshold =
					(MrvlIEtypes_BeaconLowRssiThreshold_t
					 *)(tlvptr);
				tlvptr +=
					(le16_to_cpu
					 (pLowRssiThreshold->Header.len) +
					 sizeof(MrvlIEtypesHeader_t));
				i -= (le16_to_cpu(pLowRssiThreshold->Header.len)
				      + sizeof(MrvlIEtypesHeader_t));
				break;

			case TLV_TYPE_SNR_HIGH:
				high = *(t_s8 *)(tlvptr + sizeof(t_u16) +
						 sizeof(t_u16));
				pHighSnrThreshold =
					(MrvlIEtypes_BeaconHighSnrThreshold_t
					 *)(tlvptr);
				tlvptr +=
					(le16_to_cpu
					 (pHighSnrThreshold->Header.len) +
					 sizeof(MrvlIEtypesHeader_t));
				i -= (le16_to_cpu(pHighSnrThreshold->Header.len)
				      + sizeof(MrvlIEtypesHeader_t));
				break;

			case TLV_TYPE_SNR_LOW:
				low = *(t_s8 *)(tlvptr + sizeof(t_u16) +
						sizeof(t_u16));
				pLowSnrThreshold =
					(MrvlIEtypes_BeaconLowSnrThreshold_t
					 *)(tlvptr);
				tlvptr +=
					(le16_to_cpu
					 (pLowSnrThreshold->Header.len) +
					 sizeof(MrvlIEtypesHeader_t));
				i -= (le16_to_cpu(pLowSnrThreshold->Header.len)
				      + sizeof(MrvlIEtypesHeader_t));
				break;
			}
		}

		printf("%-13s|  High = %4d  |  Low = %4d  |\n",
		       states[state], high, low);
	}
	puts("");

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue getra rssi/snr command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetRssiStatsThreshold(int argc, char *argv[])
{
	return roamGetStatsThreshold(argc, argv, RSSI_THRESHOLD);
}

/**
 *  @brief          Coverts ms to exactTime
 *
 *  @param ms       number of milliseconds
 *  @param t        converted into this structure
 *
 *  @return         none
 */
static void
ms2exactTime(t_u32 ms, ExactTime_t *t)
{
	memset(t, 0, sizeof(ExactTime_t));

	t->msecs = ms % 1000;
	if (ms >= 1000) {
		ms = (ms - t->msecs) / 1000;
		t->secs = ms % 60;
		if (ms >= 60) {
			ms = (ms - t->secs) / 60;
			t->mins = ms % 60;
			if (ms >= 60) {
				ms = (ms - t->mins) / 60;
				t->hrs = ms;
			}
		}
	}
	return;
}

/**
 *  @brief          Issue getra neighbor assessment command
 *
 *  @pNborAssessment neighbour assessment struct
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
getNborAssessment(HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT
		  *pNborAssessmentParam)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT *pNborAssessment;

	/*
	 * NEIGHBOR_ASSESSMENT
	 */
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	pNborAssessment = (HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT *)pos;
	cmd_len = S_DS_GEN + sizeof(pNborAssessment->action);
	pNborAssessment->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT);
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
		perror("ioctl[GetNeibhorAssesinfo]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}
	/*
	 * ENDIANNESS for Response
	 */
	pNborAssessment->QualifyingNumNeighbor =
		le16_to_cpu(pNborAssessment->QualifyingNumNeighbor);
	pNborAssessment->ShortBlacklistPeriod =
		le32_to_cpu(pNborAssessment->ShortBlacklistPeriod);
	pNborAssessment->LongBlacklistPeriod =
		le32_to_cpu(pNborAssessment->LongBlacklistPeriod);
	pNborAssessment->StaleCount = le16_to_cpu(pNborAssessment->StaleCount);
	pNborAssessment->StalePeriod =
		le32_to_cpu(pNborAssessment->StalePeriod);

	memcpy((void *)pNborAssessmentParam, (void *)pNborAssessment,
	       sizeof(HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT));

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Display exactTime structure elements
 *
 *  @param t        ExactTime_t struct
 *
 *  @return         None
 */
static void
printExactTime2stdout(ExactTime_t *t)
{
	int flag = 0;
	if (t->hrs) {
		printf("%dh ", t->hrs);
		flag = 1;
	}
	if (t->mins) {
		printf("%dm ", t->mins);
		flag = 1;
	}
	if (t->secs) {
		printf("%ds ", t->secs);
		flag = 1;
	}
	if (t->msecs) {
		printf("%dms", t->msecs);
		flag = 1;
	}
	if (!flag) {
		printf(" 0");
	}
}

static int
printNeighborAssessmentConfig(void)
{
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT nborAssessment;
	int idx;
	int threshDisp = FALSE;

	if (getNborAssessment(&nborAssessment) != MLAN_STATUS_SUCCESS) {
		return MLAN_STATUS_FAILURE;
	}

	printf("----------------------------------------------------------\n");
	printf("             Neighbor Threshold Parameters\n");
	printf("----------------------------------------------------------\n");
	printf(" Neighbors needed for tracking mode = %d\n",
	       nborAssessment.QualifyingNumNeighbor);
	printf(" Config RSSI qualification offset   = %d dB\n",
	       nborAssessment.ConfQualSignalStrength);
	printf(" Active RSSI qualification offset   = %d dB\n",
	       nborAssessment.ActiveQualSignalStrength);
	printf(" Short black list period            = %d ms\n",
	       (int)nborAssessment.ShortBlacklistPeriod);
	printf(" Long black list period             = %d ms\n",
	       (int)nborAssessment.LongBlacklistPeriod);
	printf(" Stale count                        = %d\n",
	       (int)nborAssessment.StaleCount);
	printf(" Stale period                       = %d ms\n",
	       (int)nborAssessment.StalePeriod);
	printf(" Proactive Roaming Thresholds       =");
	for (idx = 0; idx < NELEMENTS(nborAssessment.RoamThresh); idx++) {
		if (nborAssessment.RoamThresh[idx].RssiNborDiff) {
			if (threshDisp) {
				printf("                                     ");
			}

			threshDisp = TRUE;
			printf("%3d to %4d [%d]\n",
			       nborAssessment.RoamThresh[idx].RssiHighLevel,
			       nborAssessment.RoamThresh[idx].RssiLowLevel,
			       nborAssessment.RoamThresh[idx].RssiNborDiff);
		}
	}

	if (!threshDisp) {
		puts(" < None >");
	}
	puts("");

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief          Issue getra neighbors command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetNborList(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int cmdresplen, i;
	struct ifreq ifr;
	char neighflag[6];
	ExactTime_t t;
	t_u16 tlv;
	t_u8 *buffer = NULL, *pos = NULL, *tlvptr = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST *pNborList;
	MrvlIEtypes_NeighborEntry_t *pNeighbor;

	/*
	 * NEIGHBOR_LIST
	 */
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	pNborList = (HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST *)pos;
	cmd_len = S_DS_GEN + sizeof(pNborList->action);
	pNborList->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBORLIST);
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
		perror("ioctl[roamneighborlist]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	cmdresplen = le16_to_cpu(hostcmd->size);
	cmdresplen -= cmd_len + sizeof(pNborList->Reserved);
	tlvptr = pNborList->TlvBuffer;

	printf("----------------------------------------------------"
	       "--------------\n");
	printf("       BSSID       | RSSI |  Age  | Qualified |"
	       " Blacklist Duration\n");
	printf("----------------------------------------------------"
	       "--------------\n");

	i = 0;

	while (cmdresplen > 2) {
		i++;
		/*
		 * ENDIANNESS for Response
		 */
		tlv = le16_to_cpu(*((t_u16 *)(tlvptr)));
		switch (tlv) {
		case TLV_TYPE_NEIGHBOR_ENTRY:
			pNeighbor = (MrvlIEtypes_NeighborEntry_t *)(tlvptr);
			pNeighbor->SignalStrength =
				le16_to_cpu(pNeighbor->SignalStrength);
			pNeighbor->Age = le16_to_cpu(pNeighbor->Age);
			pNeighbor->QualifiedNeighborBitmap =
				le32_to_cpu(pNeighbor->QualifiedNeighborBitmap);

			pNeighbor->BlackListDuration =
				le32_to_cpu(pNeighbor->BlackListDuration);
			ms2exactTime(pNeighbor->BlackListDuration, &t);
			neighflag[0] = '\0';

			if ((pNeighbor->QualifiedNeighborBitmap
			     & (BIT_NEIGHFLAG_RSSI
				| BIT_NEIGHFLAG_AGE
				| BIT_NEIGHFLAG_BLACKLIST
				| BIT_NEIGHFLAG_ADMISSION_CAP
				| BIT_NEIGHFLAG_UPLINK_RSSI))
			    == (BIT_NEIGHFLAG_RSSI
				| BIT_NEIGHFLAG_AGE
				| BIT_NEIGHFLAG_BLACKLIST
				| BIT_NEIGHFLAG_ADMISSION_CAP
				| BIT_NEIGHFLAG_UPLINK_RSSI)) {
				strcat(neighflag, "Yes");
			} else {
				strcat(neighflag, "No: ");
				if (!
				    (pNeighbor->
				     QualifiedNeighborBitmap &
				     BIT_NEIGHFLAG_RSSI)) {
					strcat(neighflag, "R");
				}
				if (!
				    (pNeighbor->
				     QualifiedNeighborBitmap &
				     BIT_NEIGHFLAG_AGE)) {
					strcat(neighflag, "S");
				}
				if (!(pNeighbor->QualifiedNeighborBitmap
				      & BIT_NEIGHFLAG_BLACKLIST)) {
					strcat(neighflag, "B");
				}
				if (!(pNeighbor->QualifiedNeighborBitmap
				      & BIT_NEIGHFLAG_ADMISSION_CAP)) {
					strcat(neighflag, "A");
				}
				if (!(pNeighbor->QualifiedNeighborBitmap
				      & BIT_NEIGHFLAG_UPLINK_RSSI)) {
					strcat(neighflag, "U");
				}
			}
			printf(" %02x:%02x:%02x:%02x:%02x:%02x | %3d  | %5d | %9s | ", pNeighbor->Bssid[0], pNeighbor->Bssid[1], pNeighbor->Bssid[2], pNeighbor->Bssid[3], pNeighbor->Bssid[4], pNeighbor->Bssid[5], pNeighbor->SignalStrength, pNeighbor->Age, neighflag);

			if (pNeighbor->BlackListDuration) {
				printExactTime2stdout(&t);
			} else {
				printf("Not Blacklisted");
			}
			printf("\n");

			tlvptr += (le16_to_cpu(pNeighbor->Header.len) +
				   sizeof(MrvlIEtypesHeader_t));
			cmdresplen -= (le16_to_cpu(pNeighbor->Header.len) +
				       sizeof(MrvlIEtypesHeader_t));
			break;

		default:
			printf("\nIncorrect response.\n\n");
			break;
		}
	}

	if (i == 0) {
		printf("< Empty >\n");
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
 *  @brief          Issue getra neighbor params command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetNborParams(int argc, char *argv[])
{
	printNeighborAssessmentConfig();

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief          Issue getra metrics command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetMetrics(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int i;
	t_u16 Metrics = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD *metricscmd;

	const char *metricslist[] = { "beacon", "data", "per", "fer" };

	if (argc != 0) {
		puts("\nIncorrect number of arguments.\n");
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	metricscmd = (HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD *)pos;
	cmd_len = S_DS_GEN + sizeof(metricscmd->action);
	metricscmd->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD);
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
		perror("ioctl[roam set matrics]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	Metrics = le16_to_cpu(metricscmd->Metrics);

	if (le16_to_cpu(hostcmd->result) == MLAN_STATUS_SUCCESS) {
		printf("\n Metrics Activated: ");
		for (i = 0; (unsigned int)i < NELEMENTS(metricslist); i++) {
			if (Metrics & BIT(i)) {
				printf(" %s ", metricslist[i]);
			}
		}
		printf("\n");

		if (Metrics & BIT(3)) {
			printf("FER Threshold : %u %% \n",
			       metricscmd->UcFerThresholdValue);
			printf("FER Packet Threshold: %d \n",
			       le32_to_cpu(metricscmd->UiFerPktThreshold));
			printf("FER period. Stable : %d ms, Degrading : %d ms, "
			       "Unacceptable : %d ms\n",
			       le32_to_cpu(metricscmd->StableFERPeriod_ms),
			       le32_to_cpu(metricscmd->DegradingFERPeriod_ms),
			       le32_to_cpu(metricscmd->
					   UnacceptableFERPeriod_ms));
		}

		if (Metrics & BIT(2)) {
			printf("PER Threshold : %u %% \n",
			       metricscmd->UcPerThresholdValue);
			printf("PER Packet Threshold: %d \n",
			       le32_to_cpu(metricscmd->UiPerPktThreshold));
			printf("PER period. Stable : %d ms, Degrading : %d ms, "
			       "Unacceptable : %d ms\n",
			       le32_to_cpu(metricscmd->StablePERPeriod_ms),
			       le32_to_cpu(metricscmd->DegradingPERPeriod_ms),
			       le32_to_cpu(metricscmd->
					   UnacceptablePERPeriod_ms));
		}

		if (Metrics & BIT(1)) {
			printf("Data Packet Threshold: %d \n",
			       le32_to_cpu(metricscmd->UiRxPktThreshold));
		}

		if ((Metrics & BIT(1)) || (Metrics & BIT(2)) ||
		    (Metrics & BIT(3))) {

			printf("Inactivity Period: %d ms \n",
			       le32_to_cpu(metricscmd->
					   InactivityPeriodThreshold_ms));
		}
	} else {
		printf("command response failure %d.\n",
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
 *  @brief          Issue getra scanperiod command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetScanPeriod(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int state = 0, scanmode = 0, cmdresplen;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL, *tlvptr = NULL;
	t_u16 tlv;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD_RSP *scanPeriodInfo;
	MrvlIEtypes_NeighborScanPeriod_t *pscanperiod;
	const char *states[] = { "Stable", "Degrading", "Unacceptable" };
	const char *scanmodes[] = { "Discovery", "Tracking" };
	/* scanperiodValues[state][scanmode] */
	t_u32 values[3][2];

	/*
	 * NEIGHBOR_SCANPERIOD
	 */
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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	scanPeriodInfo =
		(HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD_RSP *)pos;
	cmd_len = S_DS_GEN + sizeof(scanPeriodInfo->action);
	scanPeriodInfo->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBOR_SCAN_PERIOD);
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
		perror("ioctl[roamscanperiod]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	cmdresplen = le16_to_cpu(hostcmd->size);
	cmdresplen -= cmd_len + sizeof(scanPeriodInfo->Reserved);
	tlvptr = scanPeriodInfo->TlvBuffer;
	while (cmdresplen > 2) {
		/*
		 * ENDIANNESS for Response
		 */
		tlv = le16_to_cpu(*((t_u16 *)(tlvptr)));
		switch (tlv) {
		case TLV_TYPE_NEIGHBOR_SCANPERIOD:
			pscanperiod =
				(MrvlIEtypes_NeighborScanPeriod_t *)(tlvptr);
			pscanperiod->SearchMode =
				le16_to_cpu(pscanperiod->SearchMode);
			pscanperiod->State = le16_to_cpu(pscanperiod->State);
			pscanperiod->ScanPeriod =
				le32_to_cpu(pscanperiod->ScanPeriod);
			state = pscanperiod->State;
			scanmode = pscanperiod->SearchMode;
			if ((state < STATE_STABLE) ||
			    (state > STATE_UNACCEPTABLE)) {
				puts("\nIncorrect state in response.\n");
			}
			if ((scanmode < DISCOVERY_MODE) ||
			    (scanmode > TRACKING_MODE)) {
				puts("\nIncorrect scanmode in response.\n");
			}
			values[state - 1][scanmode - 1] =
				pscanperiod->ScanPeriod;
			tlvptr +=
				(le16_to_cpu(pscanperiod->Header.len) +
				 sizeof(MrvlIEtypesHeader_t));
			cmdresplen -=
				(le16_to_cpu(pscanperiod->Header.len) +
				 sizeof(MrvlIEtypesHeader_t));
			break;

		default:
			puts("\nIncorrect response.\n");
			break;
		}
	}

	for (state = STATE_STABLE - 1; state < STATE_UNACCEPTABLE; state++) {
		printf("\nState: %-14s  ", states[state]);
		for (scanmode = DISCOVERY_MODE - 1;
		     scanmode < TRACKING_MODE; scanmode++) {
			printf("%s = %6d ms\t",
			       scanmodes[scanmode],
			       (int)values[state][scanmode]);
		}
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
 *  @brief          Issue getra command
 *
 *  @param pcontrol control struct to return
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
getControl(HostCmd_DS_CMD_ROAMAGENT_CONTROL *pcontrol)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_CONTROL *roamcontrolcmd = NULL;

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
	cmd_len = S_DS_GEN + sizeof(roamcontrolcmd->action);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_ROAMAGENT_CONTROL);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	roamcontrolcmd = (HostCmd_DS_CMD_ROAMAGENT_CONTROL *)pos;

	roamcontrolcmd->action = cpu_to_le16(HostCmd_ACT_GEN_GET);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[roamcontrol]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	memcpy((void *)pcontrol,
	       (void *)roamcontrolcmd,
	       sizeof(HostCmd_DS_CMD_ROAMAGENT_CONTROL));

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue getra command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param control  roma control indicator value
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetEventBitmap(int argc, char *argv[])
{
	t_u8 hostevent;
	HostCmd_DS_CMD_ROAMAGENT_CONTROL roamcontrolcmd;

	if (getControl(&roamcontrolcmd) != MLAN_STATUS_SUCCESS) {
		return MLAN_STATUS_FAILURE;
	}

	hostevent = roamcontrolcmd.HostEvent;
	if (!(hostevent | 0)) {
		puts("\nHost events are disabled.\n");
		return MLAN_STATUS_SUCCESS;
	}
	printf("\nHost events enabled: ");
	if (hostevent & HOST_EVENT_NBOR_ENABLE)
		printf("neighbor ");

	if (hostevent & HOST_EVENT_ROAM_ENABLE)
		printf("roam ");

	if (hostevent & HOST_EVENT_STATE_ENABLE)
		printf("state");

	printf("\n");
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief          Issue getra command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param control  roma control indicator value
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetControl(int argc, char *argv[])
{
	t_u8 control;
	HostCmd_DS_CMD_ROAMAGENT_CONTROL roamcontrolcmd;

	if (getControl(&roamcontrolcmd) != MLAN_STATUS_SUCCESS) {
		return MLAN_STATUS_FAILURE;
	}

	control = roamcontrolcmd.Control;
	printf("\nGlobal roaming agent state: ");
	if (control & ROAM_CONTROL_ENABLE) {
		printf("enabled, ");
		if (control & ROAM_CONTROL_SUSPEND)
			printf("suspend.\n");
		else
			printf("resume.\n");
	} else
		printf("disabled.\n");

	printf("Crossband: ");
	if (control & CROSSBAND_ENABLE)
		printf("enabled.\n");
	else
		printf("disabled.\n");

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief                    Internal funtion to issue getra backoff
 *
 *  @param roambackoffcmd     Backoff command structure
 *  @return                   MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
getBackOff(HostCmd_DS_CMD_ROAMAGENT_BACKOFF *roambackoffcmdParam)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_BACKOFF *roambackoffcmd;

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

	/* Point after host command header */
	pos = (t_u8 *)hostcmd + S_DS_GEN;

	roambackoffcmd = (HostCmd_DS_CMD_ROAMAGENT_BACKOFF *)pos;
	cmd_len = S_DS_GEN + sizeof(roambackoffcmd->action);
	roambackoffcmd->action = cpu_to_le16(HostCmd_ACT_GEN_GET);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_ROAMAGENT_BACKOFF);
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
		perror("ioctl[roambackoff]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	/*
	 * ENDIANNESS for Response
	 */
	roambackoffcmd->Scans = le16_to_cpu(roambackoffcmd->Scans);
	roambackoffcmd->Period = le32_to_cpu(roambackoffcmd->Period);

	memcpy((void *)roambackoffcmdParam, (void *)roambackoffcmd,
	       sizeof(HostCmd_DS_CMD_ROAMAGENT_BACKOFF));

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue getra backoff command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param control  roma control indicator value
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamGetBackOff(int argc, char *argv[])
{
	HostCmd_DS_CMD_ROAMAGENT_BACKOFF backoffcmd;

	if (getBackOff(&backoffcmd) != MLAN_STATUS_SUCCESS) {
		return MLAN_STATUS_FAILURE;
	}

	puts("");
	printf("----------------------------------------------------\n");
	printf("           Scanning Backoff Parameters              \n");
	printf("----------------------------------------------------\n");
	printf(" Backoff period (max time in tracking) = %d ms\n",
	       backoffcmd.Period);
	printf(" # of discovery scans before backoff   = %d\n",
	       backoffcmd.Scans);
	puts("");

	if (backoffcmd.Scans) {
		printf(" - Discovery backoff mode is enabled.\n"
		       "   After %d discovery scans without a change in the number\n"
		       "   of neighbors, the RA will only track the existing\n"
		       "   neighbors until the backoff period expires.\n",
		       backoffcmd.Scans);
	} else {
		printf(" - Discovery backoff mode is disabled.\n"
		       "   The RA will only move to tracking mode if a minimum\n"
		       "   number of good neighbors have been found. See the\n"
		       "   getra neighbor command for the current min setting.\n");
	}
	puts("");

	return MLAN_STATUS_SUCCESS;
}

static int
printGetNeighborDeprecation(int argc, char *argv[])
{
	printf("\nInfo: getra neighbor replaced with:\n"
	       "      - getra nlist   (display neighbor list)\n"
	       "      - getra nparams (display neighbor assessment params)\n\n");

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Get the Roaming agent configuration parameters from FW.
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_get_ra_config(int argc, char *argv[])
{
	if (argc == 3) {
		return roamGetControl(argc, argv);
	}

	sub_cmd_exec_t subCmds[] = {
		{"rssi", 1, 1, roamGetRssiStatsThreshold},
		{"prebcnmiss", 1, 1, roamGetPreBeaconMiss},
		{"failcnt", 1, 1, roamGetFailureCount},
		{"backoff", 1, 1, roamGetBackOff},
		{"neighbor", 2, 0, printGetNeighborDeprecation},
		{"nlist", 2, 1, roamGetNborList},
		{"nparams", 2, 1, roamGetNborParams},
		{"scanperiod", 1, 1, roamGetScanPeriod},
		{"metrics", 1, 1, roamGetMetrics},
		{"event", 1, 1, roamGetEventBitmap}
	};

	return process_sub_cmd(subCmds, NELEMENTS(subCmds), argc, argv);
}

/**
 *  @brief          Issue setra bcnmiss command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetPreBeaconLoss(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int bcnmiss;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_BeaconsMissed_t *beaconmiss;

	if (argc != 1) {
		puts("\nIncorrect number of arguments.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	bcnmiss = atoi(argv[0]);

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

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	cmd_len = sizeof(HostCmd_DS_GEN) + sizeof(statsthreshold->action) +
		sizeof(statsthreshold->State) +
		sizeof(statsthreshold->Profile) +
		sizeof(MrvlIEtypes_BeaconsMissed_t);

	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	statsthreshold->State = STATE_HARDROAM;

	beaconmiss = (MrvlIEtypes_BeaconsMissed_t *)&statsthreshold->TlvBuffer;
	beaconmiss->beacon_missed = bcnmiss;
	beaconmiss->header.type = cpu_to_le16(TLV_TYPE_PRE_BEACON_LOST);
	beaconmiss->header.len =
		cpu_to_le16(sizeof(MrvlIEtypes_BeaconsMissed_t) -
			    sizeof(MrvlIEtypesHeader_t));

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[setra]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		roamGetPreBeaconMiss(argc, argv);
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue setra failurecnt command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetFailureCount(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int failCount, failTimeThresh;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_FailureCount_t *failcnt;

	if (argc != 2) {
		puts("\n2 arguments required: FailCnt, FailTimeThresh(ms)\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	failCount = atoi(argv[0]);
	failTimeThresh = atoi(argv[1]);

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

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;

	cmd_len = (S_DS_GEN +
		   sizeof(statsthreshold->action) +
		   sizeof(statsthreshold->State) +
		   sizeof(statsthreshold->Profile) +
		   sizeof(MrvlIEtypes_FailureCount_t));

	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;
	statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	statsthreshold->State = STATE_HARDROAM;

	failcnt = (MrvlIEtypes_FailureCount_t *)&statsthreshold->TlvBuffer;
	failcnt->fail_value = failCount;
	failcnt->fail_min_thresh_time_millisecs = cpu_to_le16(failTimeThresh);
	failcnt->header.type = cpu_to_le16(TLV_TYPE_FAILCOUNT);
	failcnt->header.len = cpu_to_le16(sizeof(MrvlIEtypes_FailureCount_t)
					  - sizeof(MrvlIEtypesHeader_t));

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[setra]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		roamGetFailureCount(argc, argv);
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue setra rssi/snr command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param type     RSSI/SNR threshold type
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetStatsThreshold(int argc, char *argv[], int type)
{
	int ret = MLAN_STATUS_SUCCESS;
	int i, state = 0, low = 0, high = 0, lowval, highval;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *statsthreshold;
	MrvlIEtypes_BeaconHighRssiThreshold_t *bcnhighrssi;
	MrvlIEtypes_BeaconLowRssiThreshold_t *bcnlowrssi;
	MrvlIEtypes_BeaconHighSnrThreshold_t *bcnhighsnr;
	MrvlIEtypes_BeaconLowSnrThreshold_t *bcnlowsnr;

	if (argv[0] == NULL) {
		puts("\nInsufficient arguments.. \n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argv[0] && strncmp(argv[0], "stable", strlen("stable")) == 0) {
		state = STATE_STABLE;
		/* degrad[ing] */
	} else if (argv[0] && strncmp(argv[0], "degrad", strlen("degrad")) == 0) {
		state = STATE_DEGRADING;
		/* unaccep[table] */
	} else if (argv[0]
		   && strncmp(argv[0], "unaccep", strlen("unaccep")) == 0) {
		state = STATE_UNACCEPTABLE;
	} else {
		puts("\nUnknown state. Use stable/degrading/unacceptable\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (argc != 3 && argc != 5) {
		puts("\nIncorrect number of arguments.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (argv[1] && strncmp(argv[1], "low", strlen("low")) == 0) {
		low = 1;
		lowval = atoi(argv[2]);
		/* TODO validation check on lowval */
	}
	if (argv[1 + (2 * low)]
	    && strncmp(argv[1 + (2 * low)], "high", strlen("high")) == 0) {
		high = 1;
		highval = atoi(argv[2 + (2 * low)]);
		/* TODO validation check on highval */
	}
	/* check if low is specified after high */
	if ((low == 0) && (argc == 5)) {
		if (argv[1 + (2 * high)]
		    && strncmp(argv[1 + (2 * high)], "low", strlen("low")) == 0) {
			low = 1;
			lowval = atoi(argv[2 + 2 * (high)]);
			/* TODO validation check on lowval */
		}
	}

	if (!low && !high) {
		puts("\nIncorrect argument. Use low <low val>/high <high val>\n");
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

	statsthreshold = (HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD *)pos;
	cmd_len =
		(S_DS_GEN +
		 sizeof(HostCmd_DS_CMD_ROAMAGENT_STATISTICS_THRESHOLD)
		 - sizeof(statsthreshold->TlvBuffer));
	if (type == RSSI_THRESHOLD) {
		if (high) {
			cmd_len +=
				sizeof(MrvlIEtypes_BeaconHighRssiThreshold_t);
		}
		if (low) {
			cmd_len += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
		}
	} else {
		if (high) {
			cmd_len += sizeof(MrvlIEtypes_BeaconHighSnrThreshold_t);
		}
		if (low) {
			cmd_len += sizeof(MrvlIEtypes_BeaconLowSnrThreshold_t);
		}
	}

	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_STATISTICS_THRESHOLD);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	statsthreshold->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	statsthreshold->State = state;

	/*
	 * TLV buffer start pointer initialization
	 */
	i = 0;
	if (type == RSSI_THRESHOLD) {
		if (high) {
			bcnhighrssi = (MrvlIEtypes_BeaconHighRssiThreshold_t *)
				(((t_u8 *)&statsthreshold->TlvBuffer) + i);
			bcnhighrssi->Header.type = TLV_TYPE_RSSI_HIGH;
			bcnhighrssi->Header.len =
				(sizeof(MrvlIEtypes_BeaconHighRssiThreshold_t)
				 - sizeof(MrvlIEtypesHeader_t));
			bcnhighrssi->Value = highval;
			i += sizeof(MrvlIEtypes_BeaconHighRssiThreshold_t);
			/*
			 * ENDIANNESS
			 */
			bcnhighrssi->Header.type =
				cpu_to_le16(bcnhighrssi->Header.type);
			bcnhighrssi->Header.len =
				cpu_to_le16(bcnhighrssi->Header.len);
		}

		if (low) {
			bcnlowrssi = (MrvlIEtypes_BeaconLowRssiThreshold_t *)
				(((t_u8 *)&statsthreshold->TlvBuffer) + i);
			bcnlowrssi->Header.type = TLV_TYPE_RSSI_LOW;
			bcnlowrssi->Header.len =
				(sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t)
				 - sizeof(MrvlIEtypesHeader_t));
			bcnlowrssi->Value = lowval;
			i += sizeof(MrvlIEtypes_BeaconLowRssiThreshold_t);
			/*
			 * ENDIANNESS
			 */
			bcnlowrssi->Header.type =
				cpu_to_le16(bcnlowrssi->Header.type);
			bcnlowrssi->Header.len =
				cpu_to_le16(bcnlowrssi->Header.len);
		}
	} else {
		if (high) {
			bcnhighsnr = (MrvlIEtypes_BeaconHighSnrThreshold_t *)
				(((t_u8 *)&statsthreshold->TlvBuffer) + i);
			bcnhighsnr->Header.type = TLV_TYPE_SNR_HIGH;
			bcnhighsnr->Header.len =
				(sizeof(MrvlIEtypes_BeaconHighSnrThreshold_t)
				 - sizeof(MrvlIEtypesHeader_t));
			bcnhighsnr->Value = highval;
			i += sizeof(MrvlIEtypes_BeaconHighSnrThreshold_t);
			/*
			 * ENDIANNESS
			 */
			bcnhighsnr->Header.type =
				cpu_to_le16(bcnhighsnr->Header.type);
			bcnhighsnr->Header.len =
				cpu_to_le16(bcnhighsnr->Header.len);
		}
		if (low) {
			bcnlowsnr = (MrvlIEtypes_BeaconLowSnrThreshold_t *)
				(((t_u8 *)&statsthreshold->TlvBuffer) + i);
			bcnlowsnr->Header.type = TLV_TYPE_SNR_LOW;
			bcnlowsnr->Header.len =
				(sizeof(MrvlIEtypes_BeaconLowSnrThreshold_t)
				 - sizeof(MrvlIEtypesHeader_t));
			bcnlowsnr->Value = lowval;
			i += sizeof(MrvlIEtypes_BeaconLowSnrThreshold_t);
			/*
			 * ENDIANNESS
			 */
			bcnlowsnr->Header.type =
				cpu_to_le16(bcnlowsnr->Header.type);
			bcnlowsnr->Header.len =
				cpu_to_le16(bcnlowsnr->Header.len);
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
		perror("ioctl[setra]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 *  @brief          Issue setra rssi command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetRssiStatsThreshold(int argc, char *argv[])
{
	if (roamSetStatsThreshold(argc, argv, RSSI_THRESHOLD)
	    == MLAN_STATUS_SUCCESS) {
		roamGetStatsThreshold(0, argv, RSSI_THRESHOLD);
	}

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief          Issue setra scanperiod command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetScanPeriod(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int scanmode = 0, period = 0, state = 0;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD *scanPeriodInfo;

	if (argv[0] == NULL) {
		puts("\nInsufficient arguments.. \n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* scanperiod */
	if (argc != 3) {
		puts("\nIncorrect number of arguments.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argv[0] && strncmp(argv[0], "stable", strlen("stable")) == 0) {
		state = STATE_STABLE;
		/* degrad[ing] */
	} else if (argv[0] && strncmp(argv[0], "degrad", strlen("degrad")) == 0) {
		state = STATE_DEGRADING;
		/* unaccep[table] */
	} else if (argv[0]
		   && strncmp(argv[0], "unaccep", strlen("unaccep")) == 0) {
		state = STATE_UNACCEPTABLE;
	} else {
		puts("\nUnknown state. Use stable/degrading/unacceptable\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argv[1] && strncmp(argv[1], "disc", strlen("disc")) == 0) {
		scanmode = DISCOVERY_MODE;
		/* track[ing] */
	} else if (argv[1] && strncmp(argv[1], "track", strlen("track")) == 0) {
		scanmode = TRACKING_MODE;
	} else {
		puts("\nUnknown scamode. Use discovery/ tracking\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	period = atoi(argv[2]);
	/* TODO validation check on period */

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

	scanPeriodInfo = (HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD *)pos;
	cmd_len =
		S_DS_GEN + sizeof(HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_SCANPERIOD);
	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBOR_SCAN_PERIOD);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;
	scanPeriodInfo->action = HostCmd_ACT_GEN_SET;
	scanPeriodInfo->scanPeriod.Header.type = TLV_TYPE_NEIGHBOR_SCANPERIOD;
	scanPeriodInfo->scanPeriod.Header.len =
		sizeof(MrvlIEtypes_NeighborScanPeriod_t) -
		sizeof(MrvlIEtypesHeader_t);
	scanPeriodInfo->scanPeriod.SearchMode = scanmode;
	scanPeriodInfo->scanPeriod.State = state;
	scanPeriodInfo->scanPeriod.ScanPeriod = period;

	/*
	 * ENDIANNESS
	 */
	scanPeriodInfo->action = cpu_to_le16(scanPeriodInfo->action);
	scanPeriodInfo->scanPeriod.Header.type = cpu_to_le16
		(scanPeriodInfo->scanPeriod.Header.type);
	scanPeriodInfo->scanPeriod.Header.len = cpu_to_le16
		(scanPeriodInfo->scanPeriod.Header.len);
	scanPeriodInfo->scanPeriod.SearchMode = cpu_to_le16
		(scanPeriodInfo->scanPeriod.SearchMode);
	scanPeriodInfo->scanPeriod.State = cpu_to_le16
		(scanPeriodInfo->scanPeriod.State);
	scanPeriodInfo->scanPeriod.ScanPeriod = cpu_to_le32
		(scanPeriodInfo->scanPeriod.ScanPeriod);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[neighborlist maintain]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 *  @brief          Check it string contains digits
 *  @param str      string to check for digits
 *  @return         same as isdigit(char)
 */
static inline int
isdigitstr(char *str)
{
	unsigned int i = 0;
	for (i = 0; i < strlen(str); i++) {
		if (!isdigit((str)[i]))
			return 0;
	}
	return 1;
}

/**
 *  @brief          Process sub command
 *
 *  @param valid_cmds   valid commands array
 *  @param count        number of  valid commands
 *  @param argc         number of arguments
 *  @param argv         A pointer to arguments array
 *  @param param        argc parameter counter
 *  @param value        values to update back
 *
 *  @return             command index--success, otherwise--fail
 */
static int
process_subcmd(char *valid_cmds[], int count, int argc, char *argv[],
	       int *param, int *value)
{
	int ret = 0;
	int j = *param;
	int k;
	while (1) {
		for (k = 0; k < count; k++)
			if (argv[j] &&
			    !strncmp(valid_cmds[k], argv[j],
				     strlen(valid_cmds[k])))
				break;
		if (k >= count) {
			break;
		} else {
			/* special case */
			if (!strncmp
			    (valid_cmds[k], "perperiod", strlen("perperiod")) ||
			    !strncmp(valid_cmds[k], "ferperiod",
				     strlen("ferperiod"))) {
				*param = j;
				return ret | 1 << k;
			}
			if (!argv[j + 1] || !isdigitstr(argv[j + 1]))
				return -1;
			value[k] = atoi(argv[j + 1]);
			j = j + 2;
			ret |= (1 << k);
		}

		if (j >= argc) {
			break;
		}
	}

	if (*param == j) {
		return -1;
	}

	*param = j;
	return ret;
}

static int
printSetNeighborDeprecation(int argc, char *argv[])
{
	printf("\nInfo: setra neighbor replaced with:\n"
	       "      - setra nparams (config neighbor assessment params)\n\n");

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief          Issue setra neighbor command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetNborParams(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	int trackcount = 0, i = 0, j = 0;
	int blist[2] = { 0, 0 };	/* blacklist short and long:  0-short 1-long */
	int qualoffset[1] = { 0 };	/* rssi */
	int stalecount = ~0;
	int staleperiod = ~0;
	signed char temp;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	char *valid_cmds[] = { "trackcount", "qualoffset", "blacklistdur",
		"stalecount", "staleperiod", "roamthresh"
	};

    /** blacklistdur sub commands */
	char *valid_blcmds[] = { "short", "long" };

    /** qualoffset sub commands */
	char *valid_bfcmds[] = { "rssi" };
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT *pNborAssessment;

	if (argv[0] == NULL) {
		puts("\nUnknown setra nparams subcmd. Valid subcmds:");
		for (i = 0; i < NELEMENTS(valid_cmds); i++) {
			printf("  - %s\n", valid_cmds[i]);
		}
		printf("\n");

		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argc < 2 || argc > 18) {
		puts("\nIncorrect number of arguments.\n");
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

	pNborAssessment = (HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT *)pos;
	cmd_len =
		S_DS_GEN + sizeof(HostCmd_DS_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT);

	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBOR_ASSESSMENT);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	/* Get parameters unspecified on command line */
	if (getNborAssessment(pNborAssessment) != MLAN_STATUS_SUCCESS) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	while (argc - j) {
		for (i = 0; (unsigned int)i < NELEMENTS(valid_cmds); i++) {
			if (!strncmp
			    (valid_cmds[i], argv[j], strlen(valid_cmds[i])))
				break;
		}

		if ((unsigned int)i >= NELEMENTS(valid_cmds)) {
			printf("\nInvalid argument to \"%s\"\n\n", argv[0]);
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		switch (i) {

		case 0:
		/** trackcount */
			trackcount = atoi(argv[j + 1]);
			pNborAssessment->QualifyingNumNeighbor = trackcount;
			j = j + 2;
			break;

		case 1:
		/** qualoffset */
			j++;
			ret = process_subcmd(valid_bfcmds,
					     NELEMENTS(valid_bfcmds), argc,
					     argv, &j, qualoffset);
			if (ret == -1) {
				printf("\nInvalid argument to \"%s\"\n\n",
				       argv[j - 1]);
				ret = ret;
				goto done;
			}
			pNborAssessment->ConfQualSignalStrength = qualoffset[0];
			break;

		case 2:
		/** blacklistdur */
			j++;
			ret = process_subcmd(valid_blcmds,
					     NELEMENTS(valid_blcmds), argc,
					     argv, &j, blist);
			if (ret == -1) {
				printf("\nInvalid argument to \"%s\"\n\n",
				       argv[j - 1]);
				ret = ret;
				goto done;
			} else {
				if (ret & 0x01) {
					pNborAssessment->ShortBlacklistPeriod =
						blist[0];
				}
				if (ret & 0x02) {
					pNborAssessment->LongBlacklistPeriod =
						blist[1];
				}
			}
			break;

		case 3:
		/** stalecount */
			stalecount = atoi(argv[j + 1]);
			pNborAssessment->StaleCount = stalecount;
			j = j + 2;
			break;

		case 4:
		/** staleperiod */
			staleperiod = atoi(argv[j + 1]);
			pNborAssessment->StalePeriod = staleperiod;
			j = j + 2;
			break;

		case 5:
		/** roamthresh */
			j++;
			temp = 0;
			for (i = 0; j + i < argc; i++) {
				if (isdigit(*argv[j + i]) ||
				    (*argv[j + i] == '-')) {
					temp++;
				} else {
					break;
				}
			}

			if ((temp % 3 != 0) || (temp > 12)) {
				puts("");
				printf("Error: %d numeric arguments detected for roamthresh.\n" "       Roam threhold values must be specified in\n" "       multiples of 3 (low, high, diff triplets) up\n" "       to a maximum of 4 sets (12 numbers max).\n", temp);
				puts("");

				ret = MLAN_STATUS_FAILURE;
				goto done;
			}

			for (i = 0; (i < 12); i++) {
				if ((j >= argc) ||
				    (!isdigit(*argv[j]) && *argv[j] != '-')) {
					temp = 0;
				} else {
					temp = atoi(argv[j]);
					j++;
				}

				switch (i % 3) {
				case 0:
					pNborAssessment->RoamThresh[i /
								    3].
						RssiHighLevel = temp;
					break;

				case 1:
					pNborAssessment->RoamThresh[i /
								    3].
						RssiLowLevel = temp;
					break;

				case 2:
					pNborAssessment->RoamThresh[i /
								    3].
						RssiNborDiff = temp;
					break;
				}
			}
			break;
		}
	}

	pNborAssessment->action = cpu_to_le16(HostCmd_ACT_GEN_SET);

	/* Both stalecount, staleperiod can not be zero */
	if ((pNborAssessment->StaleCount == 0) &&
	    (pNborAssessment->StalePeriod == 0)) {

		puts("\nstalecount and staleperiod both can not be zero\n");
		ret = (MLAN_STATUS_FAILURE);
		goto done;
	}

	/*
	 * ENDIANNESS
	 */
	pNborAssessment->QualifyingNumNeighbor =
		cpu_to_le16(pNborAssessment->QualifyingNumNeighbor);
	pNborAssessment->ShortBlacklistPeriod =
		cpu_to_le32(pNborAssessment->ShortBlacklistPeriod);
	pNborAssessment->LongBlacklistPeriod =
		cpu_to_le32(pNborAssessment->LongBlacklistPeriod);
	pNborAssessment->StaleCount = cpu_to_le16(pNborAssessment->StaleCount);
	pNborAssessment->StalePeriod =
		cpu_to_le16(pNborAssessment->StalePeriod);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[roam neighbor assessment]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printNeighborAssessmentConfig();
	}

done:
	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);
	return ret;
}

/**
 *  @brief          Issue setra metrics command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetMetrics(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD *metricscmd;

	int perlist[3] = { 0, 0, 0 };	/* 0=inactivity,
					   1=perthreshold,
					   2=packetthreshold */
	int ferlist[3] = { 0, 0, 0 };	/* 0=inactivity,
					   1=ferthreshold,
					   2=framethreshold */
	int perstate[3] = { 0, 0, 0 };	/* 0=stable, 1=degrading, 2=unacceptable */
	int ferstate[3] = { 0, 0, 0 };	/* 0=stable, 1=degrading, 2=unacceptable */
	int datalist[2] = { 0, 0 };	/* 0=inactivity, 1=datathreshold */
	char *valid_cmds[] = { "Beacon", "Data", "PER", "FER" };

    /** PER sub commands */
	char *valid_percmds[] = { "inactivity", "perthreshold",
		"packetthreshold", "perperiod"
	};

	char *valid_fercmds[] = { "inactivity", "ferthreshold",
		"framethreshold", "ferperiod"
	};

    /** PER period states */
	char *per_states[] = { "stable", "degrading", "unacceptable" };

    /** Data sub commands */
	char *valid_datacmds[] = { "inactivity", "datathreshold" };

	int i = 0, j = 0;

	if (argv[0] == NULL) {
		puts("\nInsufficient arguments.. \n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argc < 1 || argc > 38) {
		puts("\nIncorrect number of arguments.\n");
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

	cmd_len =
		S_DS_GEN +
		sizeof(HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD);

	hostcmd->command =
		cpu_to_le16(HostCmd_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	metricscmd = (HostCmd_DS_CMD_ROAMAGENT_ADV_METRIC_THRESHOLD *)pos;
	metricscmd->action = cpu_to_le16(HostCmd_ACT_GEN_SET);

	/* clear [ Beacon/Data/PER ] */
	if (!strncmp("clear", argv[j], strlen("clear"))) {
		if (argc == 1) {
			puts("\nInvalid number of argument");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		while (1) {
			j++;
			for (i = 0; (unsigned int)i < NELEMENTS(valid_cmds);
			     i++) {
				if (!strncmp
				    (valid_cmds[i], argv[j],
				     strlen(valid_cmds[i])))
					break;
			}
			if ((unsigned int)i >= NELEMENTS(valid_cmds)) {
				printf("\nInvalid argument \"%s\"\n\n",
				       argv[j]);
				ret = MLAN_STATUS_FAILURE;
				goto done;
			} else {
				metricscmd->Metrics |= BIT(i);
			}

			if (j >= (argc - 1)) {
				break;
			}
		}
		metricscmd->action = HostCmd_ACT_GEN_REMOVE;
	} else {		/* set [ Beacon/Data/PER ] */

		while (argc - j) {
			for (i = 0; (unsigned int)i < NELEMENTS(valid_cmds);
			     i++) {
				if (!strncmp
				    (valid_cmds[i], argv[j],
				     strlen(valid_cmds[i])))
					break;
			}
			if ((unsigned int)i >= NELEMENTS(valid_cmds)) {
				printf("\nInvalid argument to \"%s\"\n\n",
				       argv[j]);
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}

			switch (i) {
			case 0:
		    /** Beacon */
				j++;
				metricscmd->Metrics |= 0x01;
				break;

			case 1:
		    /** Data*/
				j++;
				ret = process_subcmd(valid_datacmds,
						     NELEMENTS(valid_datacmds),
						     argc, argv, &j, datalist);
				if (ret > 0) {
					if (ret & 0x01) {
						metricscmd->
							InactivityPeriodThreshold_ms
							= datalist[0];
					}
					if (ret & 0x02) {
						metricscmd->UiRxPktThreshold =
							datalist[1];
					}
				}
				metricscmd->Metrics |= 0x02;
				break;

			case 2:
		    /** PER */
				j++;
				ret = process_subcmd(valid_percmds,
						     NELEMENTS(valid_percmds),
						     argc, argv, &j, perlist);
				if (ret > 0) {
					if (ret & 0x01) {
						metricscmd->
							InactivityPeriodThreshold_ms
							= perlist[0];
					}
					if (ret & 0x02) {
						metricscmd->
							UcPerThresholdValue =
							perlist[1];
					}
					if (ret & 0x04) {
						metricscmd->UiPerPktThreshold =
							perlist[2];
					}
					if (ret & 0x08) {
						j++;
						ret = process_subcmd(per_states,
								     NELEMENTS
								     (per_states),
								     argc, argv,
								     &j,
								     perstate);
						if (ret & 0x01) {
							metricscmd->
								StablePERPeriod_ms
								= perstate[0];
						}
						if (ret & 0x02) {
							metricscmd->
								DegradingPERPeriod_ms
								= perstate[1];
						}
						if (ret & 0x04) {
							metricscmd->
								UnacceptablePERPeriod_ms
								= perstate[2];
						}
						if (j < argc) {
							ret = process_subcmd
								(valid_percmds,
								 NELEMENTS
								 (valid_percmds),
								 argc, argv, &j,
								 perlist);
							if (ret > 0) {
								if (ret & 0x01) {
									metricscmd->
										InactivityPeriodThreshold_ms
										=
										perlist
										[0];
								}
								if (ret & 0x02) {
									metricscmd->
										UcPerThresholdValue
										=
										perlist
										[1];
								}
								if (ret & 0x04) {
									metricscmd->
										UiPerPktThreshold
										=
										perlist
										[2];
								}
							}
						}
					}
				}
				metricscmd->Metrics |= 0x04;
				break;

			case 3:
		    /** FER */
				j++;
				ret = process_subcmd(valid_fercmds,
						     NELEMENTS(valid_fercmds),
						     argc, argv, &j, ferlist);
				if (ret > 0) {
					if (ret & 0x01) {
						metricscmd->
							InactivityPeriodThreshold_ms
							= ferlist[0];
					}
					if (ret & 0x02) {
						metricscmd->
							UcFerThresholdValue =
							ferlist[1];
					}
					if (ret & 0x04) {
						metricscmd->UiFerPktThreshold =
							ferlist[2];
					}
					if (ret & 0x08) {
						j++;
						ret = process_subcmd(per_states,
								     NELEMENTS
								     (per_states),
								     argc, argv,
								     &j,
								     ferstate);
						if (ret & 0x01) {
							metricscmd->
								StableFERPeriod_ms
								= ferstate[0];
						}
						if (ret & 0x02) {
							metricscmd->
								DegradingFERPeriod_ms
								= ferstate[1];
						}
						if (ret & 0x04) {
							metricscmd->
								UnacceptableFERPeriod_ms
								= ferstate[2];
						}
						if (j < argc) {
							ret = process_subcmd
								(valid_fercmds,
								 NELEMENTS
								 (valid_fercmds),
								 argc, argv, &j,
								 ferlist);
							if (ret > 0) {
								if (ret & 0x01) {
									metricscmd->
										InactivityPeriodThreshold_ms
										=
										ferlist
										[0];
								}
								if (ret & 0x02) {
									metricscmd->
										UcFerThresholdValue
										=
										ferlist
										[1];
								}
								if (ret & 0x04) {
									metricscmd->
										UiFerPktThreshold
										=
										ferlist
										[2];
								}
							}
						}
					}
				}
				metricscmd->Metrics |= 0x08;
				break;
			}
			metricscmd->action = HostCmd_ACT_GEN_SET;
		}
	}

	/*
	 * ENDIANNESS
	 */
	metricscmd->action = cpu_to_le16(metricscmd->action);
	metricscmd->Metrics = cpu_to_le16(metricscmd->Metrics);
	metricscmd->UiPerPktThreshold =
		cpu_to_le32(metricscmd->UiPerPktThreshold);
	metricscmd->StablePERPeriod_ms =
		cpu_to_le32(metricscmd->StablePERPeriod_ms);
	metricscmd->DegradingPERPeriod_ms =
		cpu_to_le32(metricscmd->DegradingPERPeriod_ms);
	metricscmd->UnacceptablePERPeriod_ms =
		cpu_to_le32(metricscmd->UnacceptablePERPeriod_ms);
	metricscmd->InactivityPeriodThreshold_ms =
		cpu_to_le32(metricscmd->InactivityPeriodThreshold_ms);
	metricscmd->UiRxPktThreshold =
		cpu_to_le32(metricscmd->UiRxPktThreshold);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[neighborlist maintain]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 * @brief       Process maintenance of Neighbor list
 *
 * @param       argc    # arguments
 * @param       argv    A pointer to arguments array
 *
 */
static int
roamSetNborMaintenance(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	char *valid_cmds[] = { "blacklist", "clear" };
	int i = 0, j = 0, k = 0;
	unsigned int mac[ETH_ALEN];

	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE *pNborMaintainance;

	if (argv[0] == NULL) {
		puts("\nInsufficient arguments.. \n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	if (argc != 2) {
		puts("\nIncorrect number of arguments.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	for (i = 0; (unsigned int)i < NELEMENTS(valid_cmds); i++)
		if (!strncmp(valid_cmds[i], argv[j], strlen(valid_cmds[i])))
			break;
	if ((unsigned int)i >= NELEMENTS(valid_cmds)) {
		printf("\nInvalid argument \"%s\"\n\n", argv[0]);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	switch (i) {
	case 0:
	    /** blacklist */
		j++;
		sscanf(argv[j], "%2x:%2x:%2x:%2x:%2x:%2x",
		       mac, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5);
		for (k = 0; k < ETH_ALEN; k++) {
			if (*(mac + k) != 0xFF) {
				break;
			}
		}

		if (k == ETH_ALEN) {
			puts("\nBlacklisting a Broadcast address is not allowed");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		break;

	case 1:
	    /** clear */
		j++;
		if (!strncmp("all", argv[j], strlen("all"))) {
			for (k = 0; k < ETH_ALEN; k++) {
				*(mac + k) = 0xFF;
			}
		} else {
			sscanf(argv[j], "%2x:%2x:%2x:%2x:%2x:%2x",
			       mac, mac + 1, mac + 2, mac + 3, mac + 4,
			       mac + 5);
		}
		break;
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

	pNborMaintainance
		= (HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE *)pos;

	cmd_len = S_DS_GEN +
		sizeof(HostCmd_DS_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE);

	hostcmd->command
		= cpu_to_le16(HostCmd_CMD_ROAMAGENT_NEIGHBORLIST_MAINTENANCE);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;
	pNborMaintainance->action = ((i == 0) ? HostCmd_ACT_ADD_TO_BLACKLIST :
				     HostCmd_ACT_REMOVE_FROM_BLACKLIST);
	pNborMaintainance->action = cpu_to_le16(pNborMaintainance->action);
	for (k = 0; k < ETH_ALEN; k++) {
		pNborMaintainance->BSSID[k] = *(mac + k);
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[neighborlist maintain]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 *  @brief          Issue setra backoff command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetBackOff(int argc, char *argv[])
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	int spflag[2] = { 0, 0 };	/*scan,priod */
	t_u16 minscan;
	t_u32 bperiod;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_BACKOFF *roambackoffcmd;
	int i;

	if (argv[0] == NULL) {
		puts("\nInsufficient arguments.. \n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* BackOff */
	if ((argc != 2) && (argc != 4)) {
		puts("\nIncorrect number of arguments.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	for (i = 0; i < 4; i++) {
		if (argv[i + 0] &&
		    strncmp(argv[i + 0], "scans", strlen("scans")) == 0) {
			minscan = atoi(argv[i + 1]);
			i++;
			spflag[0] = 1;
			/* TODO validation check on minscan */
		}
		if (argv[i + 0] &&
		    strncmp(argv[i + 0], "period", strlen("period")) == 0) {
			bperiod = atoi(argv[i + 1]);
			i++;
			spflag[1] = 1;
			/* TODO validation check on bperiod */
		}
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

	roambackoffcmd = (HostCmd_DS_CMD_ROAMAGENT_BACKOFF *)pos;
	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_CMD_ROAMAGENT_BACKOFF);
	hostcmd->command = cpu_to_le16(HostCmd_CMD_ROAMAGENT_BACKOFF);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	switch (spflag[0] + spflag[1]) {
	case 0:
		/* error */
		puts("\nIncorrect arguments for setra backoff command.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;

	case 1:
		/* get missing parameter */
		if (getBackOff(roambackoffcmd) != MLAN_STATUS_SUCCESS) {
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		if (spflag[0] == 0) {
			minscan = roambackoffcmd->Scans;
		} else {
			bperiod = roambackoffcmd->Period;
		}
		break;

	case 2:
	default:
		break;
	}

	roambackoffcmd->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	roambackoffcmd->Scans = cpu_to_le16(minscan);
	roambackoffcmd->Period = cpu_to_le16(bperiod);

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[roambackoff]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 *  @brief          Issue setra enable/disable/resume/suspend command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *  @param set/reset set/reset flag
 *  @param value    roam control/ host event bitmap value
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetControl(int argc, char *argv[], int setreset, t_u8 value)
{
	int ret = MLAN_STATUS_SUCCESS;
	struct ifreq ifr;
	t_u8 *buffer = NULL, *pos = NULL;
	t_u32 cmd_len = 0, cmd_header_len;
	struct eth_priv_cmd *cmd = NULL;
	HostCmd_DS_GEN *hostcmd;
	HostCmd_DS_CMD_ROAMAGENT_CONTROL *roamcontrolcmd;

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

	cmd_len = S_DS_GEN + sizeof(HostCmd_DS_CMD_ROAMAGENT_CONTROL);

	hostcmd->command = cpu_to_le16(HostCmd_CMD_ROAMAGENT_CONTROL);
	hostcmd->size = cpu_to_le16(cmd_len);
	hostcmd->seq_num = 0;
	hostcmd->result = 0;

	roamcontrolcmd = (HostCmd_DS_CMD_ROAMAGENT_CONTROL *)pos;

	/* get current value */
	if (getControl(roamcontrolcmd) != MLAN_STATUS_SUCCESS) {
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	roamcontrolcmd->action = cpu_to_le16(HostCmd_ACT_GEN_SET);
	switch (setreset) {
	case 0:
		roamcontrolcmd->Control &= value;
		break;
	case 1:
		roamcontrolcmd->Control |= value;
		break;
	case 2:
		roamcontrolcmd->HostEvent &= value;
		break;
	case 3:
		roamcontrolcmd->HostEvent |= value;
		break;
	}

	/* Put buffer length */
	memcpy(buffer + cmd_header_len, &cmd_len, sizeof(t_u32));

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;
	/* Perform ioctl */
	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("ioctl[roamcontrol]");
		printf("ERR:Command sending failed!\n");
		ret = -EFAULT;
		goto done;
	}

	if (le16_to_cpu(hostcmd->result)) {
		printf("\nHostCmd Error: ReturnCode=%#04x, Result=%#04x\n\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
	} else {
		printf("\nHostCmd Success: ReturnCode=%#04x, Result=%#04x\n\n",
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
 *  @brief          Issue setra enable command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetEnableControl(int argc, char *argv[])
{
	return roamSetControl(argc, argv, 1, ROAM_CONTROL_ENABLE);
}

/**
 *  @brief          Issue setra disable command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetDisableControl(int argc, char *argv[])
{
	return roamSetControl(argc, argv, 0, ROAM_CONTROL_DISABLE);
}

/**
 *  @brief          Issue setra suspend command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetSuspendControl(int argc, char *argv[])
{
	return roamSetControl(argc, argv, 1, ROAM_CONTROL_SUSPEND);
}

/**
 *  @brief          Issue setra resume command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetResumeControl(int argc, char *argv[])
{
	return roamSetControl(argc, argv, 0, ROAM_CONTROL_RESUME);
}

/**
 *  @brief          Issue setra crossband command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to argument array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetCrossband(int argc, char *argv[])
{
	if (argc != 1) {
		printf("Invalid Usage \n");
		return MLAN_STATUS_FAILURE;
	}
	if (strncmp(argv[0], "enable", strlen("enable")) == 0) {
		return roamSetControl(argc, argv, 1, CROSSBAND_ENABLE);
	} else if (strncmp(argv[0], "disable", strlen("disable")) == 0) {
		return roamSetControl(argc, argv, 0, CROSSBAND_DISABLE);
	}

	printf("Invalid Usage \n");
	return MLAN_STATUS_FAILURE;
}

/**
 *  @brief          Issue setra event command
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to argument array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
roamSetEventBitmap(int argc, char *argv[])
{
	int i, enableFlag, found = 0;
	t_u8 value;
	if (argc > 4 || argc < 2) {
		printf("Invalid Usage \n");
		return MLAN_STATUS_FAILURE;
	}
	if (strncmp(argv[0], "enable", strlen("enable")) == 0) {
		enableFlag = 3;
		value = HOST_EVENT_NBOR_DISABLE & HOST_EVENT_ROAM_DISABLE
			& HOST_EVENT_STATE_DISABLE;
	} else if (strncmp(argv[0], "disable", strlen("disable")) == 0) {
		enableFlag = 2;
		value = HOST_EVENT_NBOR_ENABLE | HOST_EVENT_ROAM_ENABLE
			| HOST_EVENT_STATE_ENABLE;
	} else {
		printf("Invalid parameter %s \n", argv[0]);
		return MLAN_STATUS_FAILURE;
	}

	for (i = 1; i < argc; i++) {
		found = 0;
		if (strncmp(argv[i], "neighbor", strlen("neighbor")) == 0) {
			found = 1;
			if (enableFlag == 3) {
				value |= HOST_EVENT_NBOR_ENABLE;
			} else {
				value &= HOST_EVENT_NBOR_DISABLE;
			}
		}
		if (strncmp(argv[i], "roam", strlen("roam")) == 0) {
			found = 1;
			if (enableFlag == 3) {
				value |= HOST_EVENT_ROAM_ENABLE;
			} else {
				value &= HOST_EVENT_ROAM_DISABLE;
			}
		}
		if (strncmp(argv[i], "state", strlen("state")) == 0) {
			found = 1;
			if (enableFlag == 3) {
				value |= HOST_EVENT_STATE_ENABLE;
			} else {
				value &= HOST_EVENT_STATE_DISABLE;
			}
		}
		if (found == 0) {
			printf("Invalid parameter %s \n", argv[i]);
			return MLAN_STATUS_FAILURE;
		}
	}

	return roamSetControl(argc, argv, enableFlag, value);
}

/**
 *  @brief Set the Roaming agent configuration parameters to FW.
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_set_ra_config(int argc, char *argv[])
{
	sub_cmd_exec_t subCmds[] = {
		{"rssi", 2, 1, roamSetRssiStatsThreshold},
		{"prebcnmiss", 2, 1, roamSetPreBeaconLoss},
		{"failcnt", 2, 1, roamSetFailureCount},
		{"neighbor", 2, 0, printSetNeighborDeprecation},
		{"nparams", 2, 1, roamSetNborParams},
		{"maintain", 2, 1, roamSetNborMaintenance},
		{"scanperiod", 2, 1, roamSetScanPeriod},
		{"backoff", 2, 1, roamSetBackOff},
		{"enable", 2, 1, roamSetEnableControl},
		{"disable", 2, 1, roamSetDisableControl},
		{"suspend", 2, 1, roamSetSuspendControl},
		{"resume", 2, 1, roamSetResumeControl},
		{"metrics", 2, 1, roamSetMetrics},
		{"crossband", 2, 1, roamSetCrossband},
		{"event", 2, 1, roamSetEventBitmap}
	};

	return process_sub_cmd(subCmds, NELEMENTS(subCmds), argc, argv);
}
