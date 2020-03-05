/** @file  mlan2040coex.c
  *
  * @brief 11n 20/40 MHz Coex application
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
     06/24/2009: initial version
************************************************************************/

#include    <stdio.h>
#include    <ctype.h>
#include    <unistd.h>
#include    <string.h>
#include    <signal.h>
#include    <fcntl.h>
#include    <stdlib.h>
#include    <errno.h>
#include    <sys/socket.h>
#include    <sys/ioctl.h>
#include    <linux/if.h>
#include    <linux/wireless.h>
#include    <linux/netlink.h>
#include    <linux/rtnetlink.h>
#ifdef ANDROID
#include    <net/if_ether.h>
#else
#include    <net/ethernet.h>
#endif
#include    "mlan2040coex.h"
#include    "mlan2040misc.h"

/** coex application's version number */
#define COEX_VER "M2.0"

/** Initial number of total private ioctl calls */
#define IW_INIT_PRIV_NUM    128
/** Maximum number of total private ioctl calls supported */
#define IW_MAX_PRIV_NUM     1024

/********************************************************
		Local Variables
********************************************************/

static char *usage[] = {
	"Usage: ",
	"	mlan2040coex [-i <intfname>] [-hvB] ",
	"	-h = help",
	"	-v = version",
	"	-B = run the process in background.",
	"	(if intfname not present then mlan0 assumed)"
};

t_s32 sockfd = 0;  /**< socket descriptor */
char dev_name[IFNAMSIZ + 1];   /**< device name */

/** Flag: is 2040coex command required */
int coex_cmd_req_flag = FALSE;
/** Flag: is associated */
int assoc_flag = FALSE;
/** Flag: is HT AP */
int is_ht_ap = FALSE;
/** terminate flag */
int terminate_flag = FALSE;

/********************************************************
		Global Variables
********************************************************/
/** OBSS scan parameter of associated AP */
OBSSScanParam_t cur_obss_scan_param;

/********************************************************
		Local Functions
********************************************************/

/**
 *  @brief Prepare command buffer
 *  @param buffer   Command buffer to be filled
 *  @param cmd      Command id
 *  @param num      Number of arguments
 *  @param args     Arguments list
 *  @return         MLAN_STATUS_SUCCESS
 */
static int
prepare_buffer(t_u8 *buffer, char *cmd, t_u32 num, char *args[])
{
	t_u8 *pos = NULL;
	unsigned int i = 0;

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	/* Flag it for our use */
	pos = buffer;
	strncpy((char *)pos, CMD_NXP, strlen(CMD_NXP));
	pos += (strlen(CMD_NXP));

	/* Insert command */
	strncpy((char *)pos, (char *)cmd, strlen(cmd));
	pos += (strlen(cmd));

	/* Insert arguments */
	for (i = 0; i < num; i++) {
		strncpy((char *)pos, args[i], strlen(args[i]));
		pos += strlen(args[i]);
		if (i < (num - 1)) {
			strncpy((char *)pos, " ", strlen(" "));
			pos += 1;
		}
	}

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Process OBSS scan table
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_scantable(void)
{
	char ssid[MRVDRV_MAX_SSID_LENGTH + 1] = { 0 };
	unsigned int scan_start;
	int idx, i = 0, j, already_listed, ssid_len = 0, ssid_idx;

	t_u8 *pcurrent;
	t_u8 *pnext;
	IEEEtypes_ElementId_e *pelement_id;
	t_u8 *pelement_len, ht_cap_present, intol_bit_is_set;
	int ret = MLAN_STATUS_SUCCESS;
	t_s32 bss_info_len = 0;
	t_u32 fixed_field_length = 0;

	IEEEtypes_CapInfo_t cap_info;
	t_u8 tsf[8];
	t_u16 beacon_interval;
	IEEEtypes_HTCap_t *pht_cap;

	wlan_ioctl_get_scan_table_info *prsp_info;
	wlan_get_scan_table_fixed fixed_fields;

	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;

	/* Start preparing the buffer */
	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	memset(&cap_info, 0x00, sizeof(cap_info));
	memset(leg_ap_chan_list, 0, sizeof(leg_ap_chan_list));
	num_leg_ap_chan = 0;

	scan_start = 1;

	do {
		/* buffer = CMD_NXP + <cmd_string> */
		prepare_buffer(buffer, "getscantable", 0, NULL);
		prsp_info =
			(wlan_ioctl_get_scan_table_info *)(buffer +
							   strlen(CMD_NXP) +
							   strlen
							   ("getscantable"));

		prsp_info->scan_number = scan_start;

		/* Perform IOCTL */
		memset(&ifr, 0, sizeof(struct ifreq));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
		ifr.ifr_ifru.ifru_data = (void *)cmd;

		if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
			if (errno == EAGAIN) {
				ret = -EAGAIN;
			} else {
				perror("mlan2040coex");
				fprintf(stderr,
					"mlan2040coex: getscantable fail\n");
				ret = MLAN_STATUS_FAILURE;
			}
			goto done;
		}

		prsp_info = (wlan_ioctl_get_scan_table_info *)buffer;
		pcurrent = 0;
		pnext = prsp_info->scan_table_entry_buf;

		if (scan_start == 1) {
			printf("----------------------------------------------\n");
		}

		for (idx = 0; (unsigned int)idx < prsp_info->scan_number; idx++) {

			/*
			 * Set pcurrent to pnext in case pad bytes are at the end
			 * of the last IE we processed.
			 */
			pcurrent = pnext;

			memcpy((t_u8 *)&fixed_field_length,
			       (t_u8 *)pcurrent, sizeof(fixed_field_length));
			pcurrent += sizeof(fixed_field_length);

			memcpy((t_u8 *)&bss_info_len,
			       (t_u8 *)pcurrent, sizeof(bss_info_len));
			pcurrent += sizeof(bss_info_len);

			memcpy((t_u8 *)&fixed_fields,
			       (t_u8 *)pcurrent, sizeof(fixed_fields));
			pcurrent += fixed_field_length;

			pnext = pcurrent + bss_info_len;

			if (bss_info_len >= (sizeof(tsf)
					     + sizeof(beacon_interval) +
					     sizeof(cap_info))) {
				pcurrent +=
					(sizeof(tsf) + sizeof(beacon_interval) +
					 sizeof(cap_info));
				bss_info_len -=
					(sizeof(tsf) + sizeof(beacon_interval) +
					 sizeof(cap_info));
			}
			ht_cap_present = FALSE;
			intol_bit_is_set = FALSE;
			memset(ssid, 0, MRVDRV_MAX_SSID_LENGTH + 1);
			ssid_len = 0;
			while (bss_info_len >= 2) {
				pelement_id = (IEEEtypes_ElementId_e *)pcurrent;
				pelement_len = pcurrent + 1;
				pcurrent += 2;

				switch (*pelement_id) {
				case SSID:
					if (*pelement_len &&
					    *pelement_len <=
					    MRVDRV_MAX_SSID_LENGTH) {
						memcpy(ssid, pcurrent,
						       *pelement_len);
						ssid_len = *pelement_len;
					}
					break;

				case HT_CAPABILITY:
					pht_cap =
						(IEEEtypes_HTCap_t *)
						pelement_id;
					ht_cap_present = TRUE;
					if (IS_INTOL_BIT_SET
					    (le16_to_cpu
					     (pht_cap->ht_cap.ht_cap_info))) {
						intol_bit_is_set = TRUE;
					}
					break;
				default:
					break;
				}
				pcurrent += *pelement_len;
				bss_info_len -= (2 + *pelement_len);
			}
			if (!ht_cap_present || intol_bit_is_set) {
				printf("%s AP found on channel number: %-3d ",
				       intol_bit_is_set ? "40 MHZ intolerant" :
				       "Legacy", fixed_fields.channel);
				if (ssid_len) {
					printf("SSID: ");
					/* Print out the ssid or the hex values if non-printable */
					for (ssid_idx = 0; ssid_idx < ssid_len;
					     ssid_idx++) {
						if (isprint(ssid[ssid_idx])) {
							printf("%c",
							       ssid[ssid_idx]);
						} else {
							printf("\\%02x",
							       ssid[ssid_idx]);
						}
					}
				}
				printf("\n");

				/* Verify that the channel is already listed or not */
				already_listed = FALSE;
				for (j = 0; j < i; j++) {
					if (leg_ap_chan_list[j].chan_num ==
					    fixed_fields.channel) {
						already_listed = TRUE;
						if (intol_bit_is_set)
							leg_ap_chan_list[j].
								is_intol_set =
								intol_bit_is_set;
						break;
					}
				}
				if (!already_listed) {
					/* add the channel in list */
					leg_ap_chan_list[i].chan_num =
						fixed_fields.channel;
					leg_ap_chan_list[i].is_intol_set =
						intol_bit_is_set;
					i++;
					coex_cmd_req_flag = TRUE;
					num_leg_ap_chan++;
				}
			}
		}
		scan_start += prsp_info->scan_number;

	} while (prsp_info->scan_number);

done:
	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);
	return ret;
}

/** BSS Mode any (both infra and adhoc) */
#define BSS_MODE_ANY 3

/** current scan config parameters */
#define SCAN_CFG_PARAMS 7

/** Active : 1 , Passive : 2 */
#define SCAN_TYPE_ACTIVE 1

/**
 *  @brief Issue get scan type command
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
static int
get_scan_cfg(int *scan_param)
{
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	int ret = MLAN_STATUS_SUCCESS;

	/* Start preparing the buffer */
	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return MLAN_STATUS_FAILURE;
	}

	/* buffer = CMD_NXP + <cmd_string> */
	prepare_buffer(buffer, "scancfg", 0, NULL);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: get_scan_cfg fail\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	scan_param = (int *)(buffer);
done:
	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);

	return ret;
}

/**
 *  @brief Issue OBSS scan command
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_setuserscan(void)
{
	wlan_ioctl_user_scan_cfg scan_req;
	int scan_cfg[SCAN_CFG_PARAMS];
	t_u8 *buffer = NULL, *pos = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	int status = 0;

	memset(&scan_req, 0x00, sizeof(scan_req));
	memset(scan_cfg, 0x00, SCAN_CFG_PARAMS);
	coex_cmd_req_flag = FALSE;

	if (get_scan_cfg(scan_cfg) != MLAN_STATUS_SUCCESS) {
		printf("mlan2040coex: scancfg ioctl failure");
		return -EFAULT;
	}

	if (scan_cfg[0] == SCAN_TYPE_ACTIVE)
		scan_req.chan_list[0].scan_time =
			(t_u32)le16_to_cpu(cur_obss_scan_param.
					   obss_scan_active_dwell);
	else
		scan_req.chan_list[0].scan_time =
			(t_u32)le16_to_cpu(cur_obss_scan_param.
					   obss_scan_passive_total);
	scan_req.bss_mode = (scan_cfg[1]) ? scan_cfg[1] : BSS_MODE_ANY;

	/* Start preparing the buffer */
	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return MLAN_STATUS_FAILURE;
	}

	/* buffer = CMD_NXP + <cmd_string> */
	prepare_buffer(buffer, "setuserscan", 0, NULL);
	pos = buffer + strlen(CMD_NXP) + strlen("setuserscan");

	/* buffer = buffer + 'scan_req' */
	memcpy(pos, &scan_req, sizeof(wlan_ioctl_user_scan_cfg));

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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: setuserscan fail\n");
		if (cmd)
			free(cmd);
		if (buffer)
			free(buffer);
		return MLAN_STATUS_FAILURE;
	}

    /** process scan results */
	do {
		status = process_scantable();
	} while (status == -EAGAIN);

	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Display usage
 *
 *  @return       NA
 */
static t_void
display_usage(t_void)
{
	t_u32 i;
	for (i = 0; i < NELEMENTS(usage); i++)
		fprintf(stderr, "%s\n", usage[i]);
}

/**
 *  @brief              get connection status
 *
 *  @param data         Pointer to the output buffer holding connection status
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
int
get_connstatus(int *data)
{
	struct ether_addr apaddr;
	struct ether_addr etherzero = { {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;

	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return MLAN_STATUS_FAILURE;
	}

	/* buffer = CMD_NXP + <cmd_string> */
	prepare_buffer(buffer, "getwap", 0, NULL);

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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: getwap fail\n");
		if (cmd)
			free(cmd);
		if (buffer)
			free(buffer);
		return MLAN_STATUS_FAILURE;
	}

	memset(&apaddr, 0, sizeof(struct ether_addr));
	memcpy(&apaddr, (struct ether_addr *)(buffer),
	       sizeof(struct ether_addr));

	if (!memcmp(&apaddr, &etherzero, sizeof(struct ether_addr))) {
		/* not associated */
		*data = FALSE;
	} else {
		/* associated */
		*data = TRUE;
	}

	if (buffer)
		free(buffer);
	if (cmd)
		free(cmd);

	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief Print connect and disconnect event related information
 *
 *  @param buffer   Pointer to received event buffer
 *  @param size     Length of the received event
 *
 *  @return         N/A
 */
void
print_event_drv_connected(t_u8 *buffer, t_u16 size)
{
	struct ether_addr *wap;
	struct ether_addr etherzero = { {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };
	char buf[32];

	wap = (struct ether_addr *)(buffer + strlen(CUS_EVT_AP_CONNECTED));

	if (!memcmp(wap, &etherzero, sizeof(struct ether_addr))) {
		printf("---< Disconnected from AP >---\n");
		memset(&cur_obss_scan_param, 0, sizeof(cur_obss_scan_param));
		assoc_flag = FALSE;
		is_ht_ap = FALSE;
	} else {
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
			 wap->ether_addr_octet[0],
			 wap->ether_addr_octet[1],
			 wap->ether_addr_octet[2],
			 wap->ether_addr_octet[3],
			 wap->ether_addr_octet[4], wap->ether_addr_octet[5]);
		printf("---< Connected to AP: %s >---\n", buf);
	/** set TRUE, if connected */
		assoc_flag = TRUE;
	}
}

/**
 *  @brief Parse and print received event information
 *
 *  @param event    Pointer to received event
 *  @param size     Length of the received event
 *  @param evt_conn     A pointer to a output buffer. It sets TRUE when it gets
 *  					the event CUS_EVT_OBSS_SCAN_PARAM, otherwise FALSE
 *  @param if_name  The interface name
 *
 *  @return         N/A
 */
void
print_event(event_header *event, t_u16 size, int *evt_conn, char *if_name)
{
	if (!strncmp
	    (CUS_EVT_AP_CONNECTED, (char *)event,
	     strlen(CUS_EVT_AP_CONNECTED))) {
		if (strlen(if_name))
			printf("EVENT for interface %s\n", if_name);
		print_event_drv_connected((t_u8 *)event, size);
		return;
	}
	if (!strncmp
	    (CUS_EVT_OBSS_SCAN_PARAM, (char *)event,
	     strlen(CUS_EVT_OBSS_SCAN_PARAM))) {
		if (strlen(if_name))
			printf("EVENT for interface %s\n", if_name);
		printf("---< %s >---\n", CUS_EVT_OBSS_SCAN_PARAM);
		memset(&cur_obss_scan_param, 0, sizeof(cur_obss_scan_param));
		memcpy(&cur_obss_scan_param,
		       ((t_u8 *)event + strlen(CUS_EVT_OBSS_SCAN_PARAM) + 1),
		       sizeof(cur_obss_scan_param));
	/** set TRUE, since it is an HT AP */
		is_ht_ap = TRUE;
		*evt_conn = TRUE;
		return;
	}
	if (!strncmp
	    (CUS_EVT_BW_CHANGED, (char *)event, strlen(CUS_EVT_BW_CHANGED))) {
		if (strlen(if_name))
			printf("EVENT for interface %s\n", if_name);
		printf("---< %s >---\n", CUS_EVT_BW_CHANGED);
		return;
	}
}

/**
 *  @brief              This function parses for NETLINK events
 *
 *  @param nlh          Pointer to Netlink message header
 *  @param bytes_read   Number of bytes to be read
 *  @param evt_conn     A pointer to a output buffer. It sets TRUE when it gets
 *  					the event CUS_EVT_OBSS_SCAN_PARAM, otherwise FALSE
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static int
drv_nlevt_handler(struct nlmsghdr *nlh, int bytes_read, int *evt_conn)
{
	int len, plen;
	t_u8 *buffer = NULL;
	t_u32 event_id = 0;
	event_header *event = NULL;
	char if_name[IFNAMSIZ + 1];

	/* Initialize receive buffer */
	buffer = (t_u8 *)malloc(NL_MAX_PAYLOAD);
	if (!buffer) {
		printf("ERR: Could not alloc buffer\n");
		return MLAN_STATUS_FAILURE;
	}
	memset(buffer, 0, NL_MAX_PAYLOAD);

	*evt_conn = FALSE;
	while ((unsigned int)bytes_read >= NLMSG_HDRLEN) {
		len = nlh->nlmsg_len;	/* Length of message including header */
		plen = len - NLMSG_HDRLEN;
		if (len > bytes_read || plen < 0) {
			free(buffer);
			/* malformed netlink message */
			return MLAN_STATUS_FAILURE;
		}
		if ((unsigned int)len > NLMSG_SPACE(NL_MAX_PAYLOAD)) {
			printf("ERR:Buffer overflow!\n");
			free(buffer);
			return MLAN_STATUS_FAILURE;
		}
		memset(buffer, 0, NL_MAX_PAYLOAD);
		memcpy(buffer, NLMSG_DATA(nlh), plen);

		if (NLMSG_OK(nlh, len)) {
			memcpy(&event_id, buffer, sizeof(event_id));

			if (((event_id & 0xFF000000) == 0x80000000) ||
			    ((event_id & 0xFF000000) == 0)) {
				event = (event_header *)buffer;
			} else {
				memset(if_name, 0, IFNAMSIZ + 1);
				memcpy(if_name, buffer, IFNAMSIZ);
				event = (event_header *)(buffer + IFNAMSIZ);
			}
		}

		print_event(event, bytes_read, evt_conn, if_name);
		len = NLMSG_ALIGN(len);
		bytes_read -= len;
		nlh = (struct nlmsghdr *)((char *)nlh + len);
	}
	free(buffer);
	return MLAN_STATUS_SUCCESS;
}

/** Maximum event message length */
#define MAX_MSG_LENGTH	1024

/**
 *  @brief Configure and read event data from netlink socket
 *
 *  @param nl_sk        Netlink socket handler
 *  @param msg          Pointer to message header
 *  @param ptv          Pointer to struct timeval
 *
 *  @return             Number of bytes read or MLAN_STATUS_FAILURE
 */
int
read_event(int nl_sk, struct msghdr *msg, struct timeval *ptv)
{
	int count = -1;
	int ret = MLAN_STATUS_FAILURE;
	fd_set rfds;

	/* Setup read fds and initialize event buffers */
	FD_ZERO(&rfds);
	FD_SET(nl_sk, &rfds);

	/* Wait for reply */
	ret = select(nl_sk + 1, &rfds, NULL, NULL, ptv);

	if (ret == MLAN_STATUS_FAILURE) {
		/* Error */
		terminate_flag++;
		ptv->tv_sec = DEFAULT_SCAN_INTERVAL;
		ptv->tv_usec = 0;
		goto done;
	} else if (!ret) {
		if (assoc_flag && is_ht_ap) {
	    /** Issue OBSS scan */
			process_setuserscan();
	    /** Invoke 2040coex command, if any legacy AP found or
             *  any AP has 40MHz intolarent bit set */
			if (coex_cmd_req_flag)
				invoke_coex_command();
		}
		if (assoc_flag && is_ht_ap) {
			/* Timeout. Try again after BSS channel width triger scan
			   interval when the STA is connected with a HT AP */
			ptv->tv_sec =
				(t_u32)le16_to_cpu(cur_obss_scan_param.
						   bss_chan_width_trigger_scan_int);
		} else {
			/* Timeout. Try again after default duration when the STA is
			   not connected with a HT AP */
			ptv->tv_sec = DEFAULT_SCAN_INTERVAL;
		}
		ptv->tv_usec = 0;
		goto done;
	}

	if (!FD_ISSET(nl_sk, &rfds)) {
		/* Unexpected error. Try again */
		ptv->tv_sec = DEFAULT_SCAN_INTERVAL;
		ptv->tv_usec = 0;
		goto done;
	}
	/* Success */
	count = recvmsg(nl_sk, msg, 0);

done:
	return count;
}

/** Maximum event message length */
#define MAX_MSG_LENGTH	1024
/**
 *  @brief Run the application
 *
 *  @param nl_sk    Netlink socket
 *
 *  @return         N/A
 */
void
run_app(int nl_sk)
{
	struct timeval tv;
	int bytes_read, evt_conn;
	struct msghdr msg;
	struct sockaddr_nl dest_addr;
	struct nlmsghdr *nlh;
	struct iovec iov;

    /** Get connection status */
	if (get_connstatus(&assoc_flag) == MLAN_STATUS_FAILURE)
		return;

	/* Initialize timeout value */
	tv.tv_sec = DEFAULT_SCAN_INTERVAL;
	tv.tv_usec = 0;

	/* Initialize netlink header */
	nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(NL_MAX_PAYLOAD));
	if (!nlh) {
		printf("ERR: Could not allocate space for netlink header\n");
		goto done;
	}
	memset(nlh, 0, NLMSG_SPACE(NL_MAX_PAYLOAD));
	/* Fill the netlink message header */
	nlh->nlmsg_len = NLMSG_SPACE(NL_MAX_PAYLOAD);
	nlh->nlmsg_pid = getpid();	/* self pid */
	nlh->nlmsg_flags = 0;

	/* Initialize I/O vector */
	memset(&iov, 0, sizeof(struct iovec));
	iov.iov_base = (void *)nlh;
	iov.iov_len = nlh->nlmsg_len;

	/* Set destination address */
	memset(&dest_addr, 0, sizeof(struct sockaddr_nl));
	dest_addr.nl_family = AF_NETLINK;
	dest_addr.nl_pid = 0;	/* Kernel */
	dest_addr.nl_groups = NL_MULTICAST_GROUP;

	/* Initialize message header */
	memset(&msg, 0, sizeof(struct msghdr));
	msg.msg_name = (void *)&dest_addr;
	msg.msg_namelen = sizeof(dest_addr);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	while (!terminate_flag) {
		/* event buffer is received for all the interfaces */
		bytes_read = read_event(nl_sk, &msg, &tv);
		/* handle only NETLINK events here */
		drv_nlevt_handler((struct nlmsghdr *)nlh, bytes_read,
				  &evt_conn);
		if (assoc_flag && is_ht_ap) {
	    /** If the event is connected with an HT AP then issue OBSS scan immediately */
			if (evt_conn) {
		/** Issue OBSS scan */
				process_setuserscan();
		/** Invoke 2040coex command, if any legacy AP found or
                 *  any AP has 40MHz intolarent bit set */
				if (coex_cmd_req_flag)
					invoke_coex_command();
			}
			tv.tv_sec =
				(t_u32)le16_to_cpu(cur_obss_scan_param.
						   bss_chan_width_trigger_scan_int);
		} else {
			tv.tv_sec = DEFAULT_SCAN_INTERVAL;
		}
		tv.tv_usec = 0;
	}

done:
	if (nl_sk > 0)
		close(nl_sk);
	if (nlh)
		free(nlh);
	return;
}

/**
 *  @brief Determine the netlink number
 *
 *  @return         Netlink number to use
 */
static int
get_netlink_num(int dev_index)
{
	FILE *fp = NULL;
	int netlink_num = NETLINK_NXP;
	char str[64];
	char *srch = "netlink_num";
	char filename[64];

	if (dev_index == 0) {
		strcpy(filename, "/proc/mwlan/config");
	} else if (dev_index > 0) {
		sprintf(filename, "/proc/mwlan/config%d", dev_index);
	}
	/* Try to open /proc/mwlan/config$ */
	fp = fopen(filename, "r");

	if (fp) {
		while (fgets(str, sizeof(str), fp)) {
			if (strncmp(str, srch, strlen(srch)) == 0) {
				netlink_num = atoi(str + strlen(srch) + 1);
				break;
			}
		}
		fclose(fp);
	} else {
		return -1;
	}

	printf("Netlink number = %d\n", netlink_num);
	return netlink_num;
}

/**
 *  @brief opens netlink socket to receive NETLINK events
 *  @return  socket id --success, otherwise--MLAN_STATUS_FAILURE
 */
int
open_netlink(int dev_index)
{
	int sk = -1;
	struct sockaddr_nl src_addr;
	int netlink_num = 0;

	netlink_num = get_netlink_num(dev_index);
	if (netlink_num < 0) {
		printf("ERR:Could not get netlink socket. Invalid device number.\n");
		return sk;
	}

	/* Open netlink socket */
	sk = socket(PF_NETLINK, SOCK_RAW, netlink_num);
	if (sk < 0) {
		printf("ERR:Could not open netlink socket.\n");
		return sk;
	}

	/* Set source address */
	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid();
	src_addr.nl_groups = NL_MULTICAST_GROUP;

	/* Bind socket with source address */
	if (bind(sk, (struct sockaddr *)&src_addr, sizeof(src_addr)) < 0) {
		printf("ERR:Could not bind socket!\n");
		close(sk);
		return -1;
	}
	return sk;
}

/**
 *  @brief Terminate signal handler
 *  @param signal   Signal to handle
 *  @return         NA
 */
static t_void
terminate_handler(int signal)
{
	printf("Stopping application.\n");
#if DEBUG
	printf("Process ID of process killed = %d\n", getpid());
#endif
	terminate_flag = TRUE;
}

/********************************************************
		Global Functions
********************************************************/
/**
 *  @brief Process host command
 *  @param hostcmd_idx  Host command index
 *  @param chan_list    A pointer to a channel list
 *  @param chan_num     Number of channels in the channel list
 *  @param reg_class    Regulatory class of the channels
 *  @param is_intol_ap_present Flag:is there any 40 MHz intolerant AP is present or not
 *
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_host_cmd(int hostcmd_idx, t_u8 *chan_list, t_u8 chan_num,
		 t_u8 reg_class, t_u8 is_intol_ap_present)
{
	int ret = MLAN_STATUS_SUCCESS;
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	prepare_buffer(buffer, "hostcmd", 0, NULL);
	switch (hostcmd_idx) {
	case CMD_2040COEX:
		prepare_coex_cmd_buff(buffer + strlen(CMD_NXP) +
				      strlen("hostcmd"), chan_list, chan_num,
				      reg_class, is_intol_ap_present);
		break;
	default:
		break;
	}

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: hostcmd fail\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	ret = process_host_cmd_resp("hostcmd", buffer);

done:
	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Check the STA is 40 MHz intolerant or not
 *  @param intol	Flag: TRUE when the STA is 40 MHz intolerant, otherwise FALSE
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
is_intolerant_sta(int *intol)
{
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	int htcap_info, ret = MLAN_STATUS_SUCCESS;

	*intol = FALSE;

	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* buffer = CMD_NXP + <cmd_string> */
	prepare_buffer(buffer, "htcapinfo", 0, NULL);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: htcapinfo fail\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	htcap_info = *((int *)(buffer));

	if (htcap_info & MBIT(8))
		*intol = TRUE;

done:
	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief get region code
 *  @param reg_code	Pointer to region code
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
get_region_code(int *reg_code)
{
	t_u8 *buffer = NULL;
	struct eth_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	int ret = MLAN_STATUS_SUCCESS;

	/* Initialize buffer */
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* buffer = CMD_NXP + <cmd_string> */
	prepare_buffer(buffer, "regioncode", 0, NULL);

	cmd = (struct eth_priv_cmd *)malloc(sizeof(struct eth_priv_cmd));
	if (!cmd) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = MLAN_STATUS_FAILURE;
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
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	if (ioctl(sockfd, MLAN_ETH_PRIV, &ifr)) {
		perror("mlan2040coex");
		fprintf(stderr, "mlan2040coex: regioncode fail\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	memcpy(reg_code, buffer, sizeof(int));
done:
	if (cmd)
		free(cmd);
	if (buffer)
		free(buffer);
	return ret;
}

/** No option */
#define NO_OPTION -1

/**
 *  @brief Entry function for coex
 *  @param argc		number of arguments
 *  @param argv     A pointer to arguments array
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
main(int argc, char *argv[])
{
	char ifname[IFNAMSIZ + 1] = "mlan0";
	int c, daemonize = FALSE;
	t_s32 nl_sk = 0;
		      /**< netlink socket descriptor to receive an event */
	int dev_index = 0;	     /** initialise with -1 to open multiple NETLINK Sockets */

	char temp[2];
	int arg_count = 0;
	int ifname_given = FALSE;

	for (;;) {
		c = getopt(argc, argv, "Bhi:vd:");
		/* check if all command-line options have been parsed */
		if (c == NO_OPTION)
			break;

		switch (c) {
		case 'B':
			daemonize = TRUE;
			break;
		case 'h':
			display_usage();
			return MLAN_STATUS_SUCCESS;
		case 'v':
			fprintf(stdout,
				"NXP 20/40coex application version %s\n",
				COEX_VER);
			return MLAN_STATUS_SUCCESS;
		case 'i':
			ifname_given = TRUE;
			if (strlen(optarg) < IFNAMSIZ)
				strncpy(ifname, optarg, IFNAMSIZ - 1);
			else {
				fprintf(stdout, "Interface name too long\n");
				display_usage();
				return MLAN_STATUS_SUCCESS;
			}
			arg_count += 1;
			break;
		case 'd':
			strncpy(temp, optarg, strlen(optarg));
			if (isdigit(temp[0]))
				dev_index = atoi(temp);
			arg_count += 1;
			break;
		default:
			fprintf(stdout, "Invalid argument\n");
			display_usage();
			return MLAN_STATUS_SUCCESS;
		}
	}

	if (!ifname_given) {
		sprintf(ifname, "mlan%d", dev_index);
	}

	if (optind < argc) {
		fputs("Too many arguments.\n", stderr);
		display_usage();
		goto done;
	}

	strncpy(dev_name, ifname, IFNAMSIZ - 1);

	/* create a socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		fprintf(stderr, "mlan2040coex: Cannot open socket.\n");
		goto done;
	}

	/* create netlink sockets and bind them with app side addr */
	nl_sk = open_netlink(dev_index);

	if (nl_sk < 0) {
		fprintf(stderr, "mlan2040coex: Cannot open netlink socket.\n");
		goto done;
	}

	signal(SIGHUP, terminate_handler);	/* catch hangup signal */
	signal(SIGTERM, terminate_handler);	/* catch kill signal */
	signal(SIGINT, terminate_handler);	/* catch kill signal */
	signal(SIGALRM, terminate_handler);	/* catch kill signal */

    /** Make the process background-process */
	if (daemonize) {
		if (daemon(0, 0))
			fprintf(stderr, "mlan2040coex: Cannot start daemon\n");
	}

    /** run the application */
	run_app(nl_sk);

done:
	if (sockfd > 0)
		close(sockfd);
	if (nl_sk > 0)
		close(nl_sk);

	return MLAN_STATUS_SUCCESS;
}
