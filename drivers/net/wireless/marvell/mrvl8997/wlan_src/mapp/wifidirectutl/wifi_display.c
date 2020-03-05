/** @file  wifi_display.c
 *
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
  22/09/11: Initial creation
 ****************************************************************************/

/****************************************************************************
  Header files
 ****************************************************************************/

#include <stdarg.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <stdio.h>
#include <getopt.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/wireless.h>

#include "wifi_display.h"
#include "wifidirectutl.h"

/*
 *  @brief Show usage information for the wifidisplay_config command
 *
 *  $return         N/A
*/
static void
print_wifidisplay_config_usage(void)
{
	printf("\nUsage : wifidisplay_config [CONFIG_FILE]\n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("CONFIG_FILE contains all WiFiDisplay parameters.\n");
	return;
}

/*
ss and send ie config command
 *  @param ie_index              A pointer to the IE buffer index
 *  @param data_len_wifidisplay  Length of Wifidisplay data
 *  @param buf                   Pointer to buffer to set.
 *  @return                      SUCCESS--success, FAILURE--fail
 */
static int
wifiDisplay_ie_config(t_s16 *ie_index, t_u16 data_len_wifidisplay, t_u8 *buf)
{
	struct iwreq iwr;
	t_s32 sockfd;
	int i, ret = SUCCESS;
	tlvbuf_custom_ie *tlv = NULL;
	custom_ie *ie_ptr = NULL;

	tlv = (tlvbuf_custom_ie *)buf;
	tlv->tag = MRVL_MGMT_IE_LIST_TLV_ID;
	/* Locate headers */
	ie_ptr = (custom_ie *)(tlv->ie_data);

	/* Set TLV fields : WFD IE parameters */
	if (data_len_wifidisplay) {
		/* Set IE */
#define DISPLAY_MASK 0xFFFF
		ie_ptr->mgmt_subtype_mask = DISPLAY_MASK;
		tlv->length = sizeof(custom_ie) + data_len_wifidisplay;
		ie_ptr->ie_length = data_len_wifidisplay;
		ie_ptr->ie_index = *ie_index;
	} else {
		/* Get WPS IE */
		tlv->length = 0;
	}

	/* Locate headers */
	ie_ptr = (custom_ie *)((t_u8 *)(tlv->ie_data) + sizeof(custom_ie) +
			       data_len_wifidisplay);

	memset(&iwr, 0, sizeof(iwr));
	strncpy(iwr.ifr_name, (char *)dev_name, IFNAMSIZ);

	iwr.u.data.pointer = (void *)buf;
	iwr.u.data.length =
		((2 * sizeof(custom_ie)) + sizeof(tlvbuf_custom_ie) +
		 data_len_wifidisplay);
	iwr.u.data.flags = 0;

	/*
	 *      * create a socket
	 *           */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("Cannot open socket.\n");
		ret = FAILURE;
		goto _exit_;
	}

	if (ioctl(sockfd, CUSTOM_IE, &iwr)) {
		perror("ioctl[CUSTOM_IE]");
		printf("Failed to set/get/clear the IE buffer\n");
		ret = FAILURE;
		close(sockfd);
		goto _exit_;
	}
	close(sockfd);

	if (!data_len_wifidisplay) {
		/* Get the IE buffer index number for MGMT_IE_LIST_TLV */
		tlv = (tlvbuf_custom_ie *)buf;
		*ie_index = -1;
		if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
			ie_ptr = (custom_ie *)(tlv->ie_data);
			for (i = 0; i < MAX_MGMT_IE_INDEX; i++) {
				// Index 0 and 1 are reserved for WFD in current implementation
				if ((ie_ptr->mgmt_subtype_mask == DISPLAY_MASK)
				    && (ie_ptr->ie_length) && (i != 0) &&
				    (i != 1)) {
					*ie_index = ie_ptr->ie_index;
					break;
				}
				if (i < (MAX_MGMT_IE_INDEX - 1))
					ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
							       sizeof(custom_ie)
							       +
							       ie_ptr->
							       ie_length);
			}
		}
		if (*ie_index == -1) {
			printf("\nNo free IE buffer available\n");
			ret = FAILURE;
		}
	}
_exit_:

	return ret;
}

/**
 *  @brief Creates a wifidisplay_config request and sends to the driver
 *
 *  Usage: "Usage : wfd_config [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
void
wifidisplaycmd_config(int argc, char *argv[])
{
	t_u8 *buf = NULL;
	t_u16 ie_len_wifidisplay = 0, ie_len;
	t_s16 ie_index = -1;
	int opt, ret = SUCCESS;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidisplay_config_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidisplay_config_usage();
		return;
	}
	buf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buf) {
		printf("ERR:Cannot allocate memory!\n");
		return;
	}
	memset(buf, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	if (argc >= 3) {
		/* Read parameters and send command to firmware */
		wifidisplay_file_params_config(argv[2], argv[1], buf
					       + sizeof(tlvbuf_custom_ie) +
					       sizeof(custom_ie),
					       &ie_len_wifidisplay);
		if (argc == 4) {
			ie_index = atoi(argv[3]);
			if (ie_index >= 4) {
				printf("ERR:wrong argument %s.\n", argv[3]);
				free(buf);
				return;
			}
		}
		if (ie_len_wifidisplay > MAX_SIZE_IE_BUFFER) {
			printf("ERR:IE parameter size exceeds limit in %s.\n",
			       argv[2]);
			free(buf);
			return;
		}
		ie_len = ie_len_wifidisplay + sizeof(tlvbuf_custom_ie) +
			sizeof(custom_ie);
		if (ie_len >= MRVDRV_SIZE_OF_CMD_BUFFER) {
			printf("ERR:Too much data in configuration file %s.\n",
			       argv[2]);
			free(buf);
			return;
		}
#ifdef DEBUG
		hexdump(buf, ie_len, ' ');
#endif
		ret = wifiDisplay_ie_config(&ie_index, ie_len_wifidisplay, buf);
		if (ret != SUCCESS) {
			printf("ERR:Could not set wfd parameters\n");
		}
	}
	free(buf);
}

/* *  @brief Read the wifidisplay parameters and sends to the driver
 *
 *  @param file_name File to open for configuration parameters.
 *  @param cmd_name  Command Name for which parameters are read.
 *  @param pbuf      Pointer to output buffer
 *  @param ie_len_wfd Length of wifidisplay parameters to return
 *  @return          SUCCESS or FAILURE
 */
#define DEVICE_DESCRIPTOR_LEN 24
static const t_u8 wifidisplay_oui[] = { 0x50, 0x6F, 0x9A, 0x0A };

void
wifidisplay_file_params_config(char *file_name, char *cmd_name,
			       t_u8 *pbuf, t_u16 *ie_len_wifidisplay)
{
	FILE *config_file = NULL;
	char *line = NULL;
	t_u8 *extra = NULL, *len_ptr = NULL;
	t_u8 *buffer = pbuf;
	char **args = NULL;
	t_u16 cmd_len_wifidisplay = 0, tlv_len = 0;
	tlvbuf_wifidisplay_ie_format *display_ie_buf = NULL;
	int wifiDisplay_level = 0, ret = 0, coupled_sink_bitmap = 0;
	t_u16 display_device_info = 0, session_mgmt_control_port =
		0, wfd_device_throuput = 0;
	t_u8 assoc_bssid[] = { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };
	t_u8 alternate_mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	t_u8 peer_mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	t_u8 default_mac[] = { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };
	char wfd_mac[20];
	int li = 0, arg_num = 0;
	char *pos = NULL;
	t_u8 cmd_found = 0;
	t_u16 temp;
	t_u16 wfd_session_len;
	t_u8 total_num_device_info __attribute__ ((__unused__));	/* For future use */
	t_u8 curr_dev_info = 0;
	t_u8 dev_info_dev_add[ETH_ALEN] =
		{ 0 }, dev_info_assoc_bssid[ETH_ALEN] = {
	0}, dev_info_coupled_add[ETH_ALEN];
	t_u16 descriptor_display_device_info =
		0, descriptor_wfd_device_throuput = 0;
	t_u8 descriptor_wfd_coupled_sink_status = 0;
	t_u8 wfd_dev_descriptor_arr[120], device_info_desc_len = 0, ind =
		DEVICE_DESCRIPTOR_LEN;

	memset(wfd_mac, 0, sizeof(wfd_mac));
	/* Check if file exists */
	config_file = fopen(file_name, "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return;
	}

	/* Memory allocations */
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		goto done;
	}
	memset(line, 0, MAX_CONFIG_LINE);

	extra = (t_u8 *)malloc(MAX_CONFIG_LINE);
	if (!extra) {
		printf("ERR:Cannot allocate memory for extra\n");
		goto done;
	}
	memset(extra, 0, MAX_CONFIG_LINE);

	args = (char **)malloc(sizeof(char *) * MAX_ARGS_NUM);
	if (!args) {
		printf("ERR:Cannot allocate memory for args\n");
		goto done;
	}
	memset(args, 0, (sizeof(char *) * MAX_ARGS_NUM));
	display_ie_buf = (tlvbuf_wifidisplay_ie_format *)buffer;
	display_ie_buf->ElemId = VENDOR_SPECIFIC_IE_TAG;
	len_ptr = buffer + 1;
	memcpy(&display_ie_buf->Oui[0], wifidisplay_oui,
	       sizeof(display_ie_buf->Oui));
	display_ie_buf->OuiType = wifidisplay_oui[3];
	cmd_len_wifidisplay += 2 + sizeof(wifidisplay_oui);
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
		arg_num = parse_line(line, args, MAX_ARGS_NUM);
		if (!cmd_found && (cmd_name != NULL)
		    && strncmp(args[0], cmd_name, strlen(args[0])))
			cmd_found = 1;
		if (strcmp(args[0], "display_dev_info") == 0) {
			wifiDisplay_level = DISPLAY_DEVICE_INFO;
		} else if (strcmp(args[0], "display_assoc_bssid") == 0) {
			wifiDisplay_level = DISPLAY_ASSOCIATED_BSSID;
		} else if (strcmp(args[0], "display_coupled_sink") == 0) {
			wifiDisplay_level = DISPLAY_COUPLED_SINK;
		} else if (strcmp(args[0], "display_session_info") == 0) {
			wifiDisplay_level = DISPLAY_SESSION_INFO;
		} else if (strcmp(args[0], "display_alternate_mac") == 0) {
			wifiDisplay_level = DISPLAY_ALTERNATE_MAC_ADDR;
		} else if (strcmp(args[0], "device_info") == 0) {
			if (is_wifidisplay_input_valid
			    (WFD_DEVICE_INFO, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			display_device_info = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "mgmt_control_port") == 0) {
			if (is_wifidisplay_input_valid
			    (WFD_SESSION_MGMT_CONTROL_PORT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			session_mgmt_control_port = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "device_throuput") == 0) {
			if (is_wifidisplay_input_valid
			    (WFD_DEVICE_THROUGHPUT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			wfd_device_throuput = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "assoc_bssid") == 0) {
			strncpy(wfd_mac, args[1], sizeof(wfd_mac) - 1);
			if ((ret = mac2raw(wfd_mac, assoc_bssid)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
		} else if (strcmp(args[0], "alternate_mac") == 0) {
			strncpy(wfd_mac, args[1], sizeof(wfd_mac) - 1);
			if ((ret = mac2raw(wfd_mac, alternate_mac)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
		} else if (strcmp(args[0], "coupled_sink_bitmap") == 0) {
			if (is_wifidisplay_input_valid
			    (WFD_COUPLED_SINK, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			coupled_sink_bitmap = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "session_info_len") == 0) {
			wfd_session_len = (t_u16)atoi(args[1]);
			total_num_device_info =
				(wfd_session_len / DEVICE_DESCRIPTOR_LEN);
		} else if (strncmp(args[0], "device_info_descriptor_len", 26) ==
			   0) {
			device_info_desc_len = (t_u16)atoi(args[1]);
		} else if (strncmp(args[0], "device_info_dev_id", 18) == 0) {
			strncpy(wfd_mac, args[1], sizeof(wfd_mac) - 1);
			if ((ret =
			     mac2raw(wfd_mac, dev_info_dev_add)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
		} else if (strncmp(args[0], "device_info_assoc_bssid", 18) == 0) {
			strncpy(wfd_mac, args[1], sizeof(wfd_mac) - 1);
			if ((ret =
			     mac2raw(wfd_mac,
				     dev_info_assoc_bssid)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
		} else if (strncmp(args[0], "descriptor_device_info", 22) == 0) {
			descriptor_display_device_info = (t_u16)atoi(args[1]);
			temp = htons(descriptor_display_device_info);
			memcpy(&descriptor_display_device_info, &temp, 2);
		} else if (strncmp(args[0], "descriptor_device_throuput", 24) ==
			   0) {
			descriptor_wfd_device_throuput = (t_u16)atoi(args[1]);
			temp = htons(descriptor_wfd_device_throuput);
			memcpy(&descriptor_display_device_info, &temp, 2);
		} else if (strncmp(args[0], "descriptor_cs_bitmap", 20) == 0) {
			descriptor_wfd_coupled_sink_status =
				(t_u8)atoi(args[1]);
		} else if (strncmp(args[0], "device_info_coupled_address", 27)
			   == 0) {
			strncpy(wfd_mac, args[1], sizeof(wfd_mac) - 1);
			if ((ret =
			     mac2raw(wfd_mac,
				     dev_info_coupled_add)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
			if (curr_dev_info >= 5) {
				printf("ERR in device descriptor");
				goto done;
			}
			memcpy(&wfd_dev_descriptor_arr[ind * curr_dev_info],
			       &device_info_desc_len, sizeof(t_u8));
			memcpy(&wfd_dev_descriptor_arr[ind * curr_dev_info + 1],
			       &dev_info_dev_add, ETH_ALEN);
			memcpy(&wfd_dev_descriptor_arr[ind * curr_dev_info + 7],
			       &dev_info_assoc_bssid, ETH_ALEN);
			memcpy(&wfd_dev_descriptor_arr
			       [ind * curr_dev_info + 13],
			       &descriptor_display_device_info, sizeof(t_u16));
			memcpy(&wfd_dev_descriptor_arr
			       [ind * curr_dev_info + 15],
			       &descriptor_wfd_device_throuput, sizeof(t_u16));
			memcpy(&wfd_dev_descriptor_arr
			       [ind * curr_dev_info + 17],
			       &descriptor_wfd_coupled_sink_status,
			       sizeof(t_u8));
			memcpy(&wfd_dev_descriptor_arr
			       [ind * curr_dev_info + 18],
			       &dev_info_coupled_add, ETH_ALEN);
			curr_dev_info++;
		} else if (strcmp(args[0], "}") == 0) {
			switch (wifiDisplay_level) {
			case DISPLAY_DEVICE_INFO:
				{
					tlvbuf_wfdisplay_device_info *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wfdisplay_device_info);
					tlv = (tlvbuf_wfdisplay_device_info
					       *)(buffer + cmd_len_wifidisplay);
					cmd_len_wifidisplay += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDISPLAY_DEVICE_INFO;
					tlv->length =
						htons(tlv_len -
						      (sizeof(t_u8) +
						       sizeof(t_u16)));
					*ie_len_wifidisplay =
						cmd_len_wifidisplay;
					temp = htons(display_device_info);
					memcpy(&tlv->display_device_info, &temp,
					       2);
					temp = htons(session_mgmt_control_port);
					memcpy(&tlv->session_mgmt_control_port,
					       &temp, 2);
					temp = htons(wfd_device_throuput);
					memcpy(&tlv->wfd_device_throuput, &temp,
					       2);
					wifiDisplay_level = 0;
					break;
				}
			case DISPLAY_ASSOCIATED_BSSID:
				{
					tlvbuf_wfdisplay_assoc_bssid *tlv =
						NULL;
					if (memcmp
					    (default_mac, assoc_bssid,
					     ETH_ALEN)) {
						/* Append a new TLV */
						tlv_len =
							sizeof
							(tlvbuf_wfdisplay_assoc_bssid);
						tlv = (tlvbuf_wfdisplay_assoc_bssid *)(buffer + cmd_len_wifidisplay);
						cmd_len_wifidisplay += tlv_len;
						*ie_len_wifidisplay =
							cmd_len_wifidisplay;
						/* Set TLV fields */
						tlv->tag =
							TLV_TYPE_WIFIDISPLAY_ASSOC_BSSID;
						tlv->length =
							htons(tlv_len -
							      (sizeof(t_u8) +
							       sizeof(t_u16)));
						memcpy(tlv->assoc_bssid,
						       assoc_bssid, ETH_ALEN);

						wifiDisplay_level = 0;
					}
					break;
				}
			case DISPLAY_COUPLED_SINK:
				{
					tlvbuf_wfdisplay_coupled_sink *tlv =
						NULL;
					tlv_len =
						sizeof
						(tlvbuf_wfdisplay_coupled_sink);
					tlv = (tlvbuf_wfdisplay_coupled_sink
					       *)(buffer + cmd_len_wifidisplay);
					cmd_len_wifidisplay += tlv_len;
					*ie_len_wifidisplay =
						cmd_len_wifidisplay;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDISPLAY_COUPLED_SINK;
					tlv->length =
						htons(tlv_len -
						      (sizeof(t_u8) +
						       sizeof(t_u16)));
					memcpy(tlv->peer_mac, peer_mac,
					       ETH_ALEN);
					tlv->coupled_sink = coupled_sink_bitmap;
					wifiDisplay_level = 0;
				}
				break;
			case DISPLAY_SESSION_INFO:
				{
					tlvbuf_wifi_display_session_info *tlv =
						NULL;
					if (curr_dev_info > 0) {
						tlv_len =
							DEVICE_DESCRIPTOR_LEN *
							curr_dev_info + 2;
						tlv = (tlvbuf_wifi_display_session_info *)(buffer + cmd_len_wifidisplay);
						cmd_len_wifidisplay += tlv_len;
						*ie_len_wifidisplay =
							cmd_len_wifidisplay;
						/* Set TLV fields */
						tlv->tag =
							TLV_TYPE_SESSION_INFO_SUBELEM;
						tlv->length =
							htons(tlv_len -
							      (sizeof(t_u8) +
							       sizeof(t_u16)));
						memcpy((t_u8 *)&tlv->
						       WFDDevInfoDesc,
						       (t_u8 *)
						       &wfd_dev_descriptor_arr,
						       (tlv_len - 2));
						wifiDisplay_level = 0;
					}
				}
				break;
			case DISPLAY_ALTERNATE_MAC_ADDR:
				{
					tlvbuf_wfdisplay_alternate_mac *tlv =
						NULL;
					if (memcmp
					    (default_mac, alternate_mac,
					     ETH_ALEN)) {
						/* Append a new TLV */
						tlv_len =
							sizeof
							(tlvbuf_wfdisplay_alternate_mac);
						tlv = (tlvbuf_wfdisplay_alternate_mac *)(buffer + cmd_len_wifidisplay);
						cmd_len_wifidisplay += tlv_len;
						*ie_len_wifidisplay =
							cmd_len_wifidisplay;
						/* Set TLV fields */
						tlv->tag =
							TLV_TYPE_WIFIDISPLAY_ALTERNATE_MAC;
						tlv->length =
							htons(tlv_len -
							      (sizeof(t_u8) +
							       sizeof(t_u16)));
						memcpy(tlv->alternate_mac,
						       alternate_mac, ETH_ALEN);
						wifiDisplay_level = 0;
					}
				}
				break;
			default:
				break;
			}
		}
	}
	*len_ptr = cmd_len_wifidisplay - 2;
done:
	fclose(config_file);
	if (line)
		free(line);
	if (extra)
		free(extra);
	if (args)
		free(args);
	return;
}

/**
 *  @brief Show usage information for the wifidisplay_discovery commands
 *
 *  $return         N/A
 */
static void
print_wifidisplay_discovery_usage(void)
{
	printf("\nUsage : wifidisplay_discovery_request/response [CONFIG_FILE]\n");
	printf("CONFIG_FILE contains WIFIDISPLAY service discovery payload.\n");
	return;
}

/*  @brief Creates a wifidirect_service_discovery request/response and
 *         sends to the driver
 *
 *  Usage: "Usage : wifidirect_discovery_request/response [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 **/
void
wifidisplaycmd_service_discovery(int argc, char *argv[])
{
	wifidisplay_discovery_request *req_buf = NULL;
	wifidisplay_discovery_response *resp_buf = NULL;
	char *line = NULL;
	FILE *config_file = NULL;
	int i, opt, li = 0, arg_num = 0, ret = 0, wifidirect_level = 0;
	char *args[30], *pos = NULL, wifidisplay_mac[20], wifidisplay_cmd[32];
	t_u8 dev_address[ETH_ALEN], cmd_found = 0;
	t_u8 *buffer = NULL, *buf = NULL, *tmp_buffer = NULL;
	t_u8 req_resp = 0;	/* req = 0, resp = 1 */
	t_u16 cmd_len = 0, query_len = 0, vendor_len = 0, service_len = 0;
	t_u16 ie_len_wifidisplay = 0;

	memset(args, 0, sizeof(args));
	memset(wifidisplay_mac, 0, sizeof(wifidisplay_mac));
	strncpy(wifidisplay_cmd, argv[2], sizeof(wifidisplay_cmd) - 1);
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidisplay_discovery_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 3) {
		printf("ERR:Incorrect number of arguments.\n");
		print_wifidisplay_discovery_usage();
		return;
	}

	/* Check if file exists */
	config_file = fopen(argv[2], "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return;
	}
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		goto done;
	}
	memset(line, 0, MAX_CONFIG_LINE);
	buf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buf) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}
	if (strcmp(args[0], "wifidisplay_discovery_response") == 0) {
		t_u8 wfd_oui_header = 6;
		wifidisplay_file_params_config(argv[2], NULL, buf,
					       &ie_len_wifidisplay);
		ie_len_wifidisplay += wfd_oui_header;
		buf += wfd_oui_header;
	} else {
		buf[0] = 0x03;
		buf[1] = TLV_TYPE_WIFIDISPLAY_DEVICE_INFO;
		buf[2] = TLV_TYPE_SESSION_INFO_SUBELEM;
		buf[3] = TLV_TYPE_WIFIDISPLAY_COUPLED_SINK;
		ie_len_wifidisplay = 4;
	}

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
		arg_num = parse_line(line, args, 30);
		if (!cmd_found &&
		    strncmp(args[0], wifidisplay_cmd, strlen(args[0])))
			continue;
		if (strcmp(args[0], "wifidisplay_discovery_request") == 0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			/* For wifidirect_service_discovery, basic initialization here */
			cmd_len = sizeof(wifidisplay_discovery_request);
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			req_buf = (wifidisplay_discovery_request *)buffer;
			req_buf->cmd_code =
				HostCmd_CMD_WIFIDIRECT_SERVICE_DISCOVERY;
			req_buf->size = cmd_len;
			req_buf->seq_num = 0;
			req_buf->result = 0;
			cmd_found = 1;
		} else if (strcmp(args[0], "wifidisplay_discovery_response") ==
			   0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			req_resp = 1;
			/* For wifidirect_service_discovery, basic initialization here */
			cmd_len = sizeof(wifidisplay_discovery_response);
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			resp_buf = (wifidisplay_discovery_response *)buffer;
			resp_buf->cmd_code =
				HostCmd_CMD_WIFIDIRECT_SERVICE_DISCOVERY;
			resp_buf->size = cmd_len;
			resp_buf->seq_num = 0;
			resp_buf->result = 0;
			cmd_found = 1;

		}
		if (!cmd_found)
			break;
		if (strcmp(args[0], "PeerAddr") == 0) {
			strncpy(wifidisplay_mac, args[1], 20 - 1);
			if ((ret =
			     mac2raw(wifidisplay_mac,
				     dev_address)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
			(!req_resp) ? memcpy(req_buf->peer_mac_addr,
					     dev_address,
					     ETH_ALEN) : memcpy(resp_buf->
								peer_mac_addr,
								dev_address,
								ETH_ALEN);

		} else if (strcmp(args[0], "Category") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_CATEGORY, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;

			}
			(!req_resp) ? (req_buf->category =
				       (t_u8)atoi(args[1])) : (resp_buf->
							       category =
							       (t_u8)
							       atoi(args[1]));
		} else if (strcmp(args[0], "Action") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_ACTION, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ? (req_buf->action =
				       (t_u8)A2HEXDECIMAL(args[1]))
				: (resp_buf->action =
				   (t_u8)A2HEXDECIMAL(args[1]));
		} else if (strcmp(args[0], "DialogToken") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DIALOGTOKEN, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ? (req_buf->dialog_taken =
				       (t_u8)atoi(args[1])) : (resp_buf->
							       dialog_taken =
							       (t_u8)
							       atoi(args[1]));
		} else if (strcmp(args[0], "StatusCode") == 0) {
			if (req_resp)
				resp_buf->status_code = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "GasComebackDelay") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_GAS_COMEBACK_DELAY, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp) {
				resp_buf->gas_reply =
					(t_u16)A2HEXDECIMAL(args[1]);
				resp_buf->gas_reply =
					cpu_to_le16(resp_buf->gas_reply);
			}
		} else if (strcmp(args[0], "AdvertizementProtocolIE") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_ADPROTOIE, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (!req_resp) {
				for (i = 0; i < arg_num - 1; i++)
					req_buf->advertize_protocol_ie[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0; i < arg_num - 1; i++)
					resp_buf->advertize_protocol_ie[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			}
		} else if (strcmp(args[0], "InfoId") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_INFOID, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (!req_resp) {
				for (i = 0; i < arg_num - 1; i++)
					req_buf->info_id[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0; i < arg_num - 1; i++)
					resp_buf->info_id[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			}
			query_len += arg_num - 1;
		} else if (strcmp(args[0], "OUI") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_OUI, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (!req_resp) {
				for (i = 0; i < arg_num - 1; i++)
					req_buf->oui[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0; i < arg_num - 1; i++)
					resp_buf->oui[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			}
			service_len += arg_num - 1;
			query_len += arg_num - 1;
		} else if (strcmp(args[0], "OUISubType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_OUISUBTYPE, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ? (req_buf->oui_sub_type =
				       (t_u8)atoi(args[1])) : (resp_buf->
							       oui_sub_type =
							       (t_u8)
							       atoi(args[1]));
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "QueryRequestLen") == 0 ||
			   strcmp(args[0], "QueryResponseLen") == 0) {
			wifidirect_level = WIFIDIRECT_DISCOVERY_QUERY;
		} else if (strcmp(args[0], "RequestLen") == 0 ||
			   strcmp(args[0], "ResponseLen") == 0) {
			wifidirect_level = WIFIDIRECT_DISCOVERY_SERVICE;
			query_len += 2;
		} else if (strcmp(args[0], "VendorLen") == 0) {
			wifidirect_level = WIFIDIRECT_DISCOVERY_VENDOR;
			service_len += 2;
			query_len += 2;
		} else if (strcmp(args[0], "QueryData") == 0 ||
			   strcmp(args[0], "ResponseData") == 0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_QUERY_RESPONSE_PER_PROTOCOL;
			tmp_buffer =
				realloc(buffer, cmd_len + ie_len_wifidisplay);
			if (!tmp_buffer) {
				printf("ERR:Cannot add DNS name to buffer!\n");
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			if (!req_resp) {
				req_buf =
					(wifidisplay_discovery_request *)buffer;
				for (i = 0; i < ie_len_wifidisplay; i++)
					req_buf->disc_query[i] = (t_u8)buf[i];
			} else {
				resp_buf =
					(wifidisplay_discovery_response *)
					buffer;
				for (i = 0; i < ie_len_wifidisplay; i++)
					resp_buf->disc_resp[i] = (t_u8)buf[i];
			}
			cmd_len += (ie_len_wifidisplay);
			vendor_len += (ie_len_wifidisplay);
			service_len += (ie_len_wifidisplay);
			query_len += (ie_len_wifidisplay);
		} else if (strcmp(args[0], "ServiceProtocol") == 0) {
			if (!req_resp) {
				req_buf->service_protocol = (t_u8)atoi(args[1]);
				/*
				 * For uPnP, due to union allocation, a extra byte
				 * is allocated reduce it here for uPnP
				 */
				if (req_buf->service_protocol == 2)
					cmd_len--;
			} else {
				resp_buf->service_protocol =
					(t_u8)atoi(args[1]);
				if (resp_buf->service_protocol == 2)
					cmd_len--;
			}
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "ServiceUpdateIndicator") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_SERVICEUPDATE_INDICATOR, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ? (req_buf->service_update_indicator =
				       cpu_to_le16((t_u16)atoi(args[1]))) :
				(resp_buf->service_update_indicator =
				 cpu_to_le16((t_u16)atoi(args[1])));
			service_len += 2;
			query_len += 2;
		} else if (strcmp(args[0], "ServiceTransactionId") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICETRANSACID, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ? (req_buf->service_transaction_id =
				       (t_u8)atoi(args[1])) : (resp_buf->
							       service_transaction_id
							       =
							       (t_u8)
							       atoi(args[1]));
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "}") == 0) {
			switch (wifidirect_level) {
			case WIFIDIRECT_DISCOVERY_QUERY:
				(!req_resp) ? (req_buf->query_len =
					       cpu_to_le16(query_len))
					: (resp_buf->query_len =
					   cpu_to_le16(query_len));
				break;
			case WIFIDIRECT_DISCOVERY_SERVICE:
				(!req_resp) ? (req_buf->request_len =
					       cpu_to_le16(service_len))
					: (resp_buf->response_len =
					   cpu_to_le16(service_len));
				break;
			case WIFIDIRECT_DISCOVERY_VENDOR:
				(!req_resp) ? (req_buf->vendor_len =
					       cpu_to_le16(vendor_len))
					: (resp_buf->vendor_len =
					   cpu_to_le16(vendor_len));
				break;
			default:
				break;
			}
			if (wifidirect_level) {
				if (wifidirect_level ==
				    WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE)
					break;
				wifidirect_level--;
			}
		}
	}
	/* Send collective command */
	if (buffer)
		wifidirect_ioctl((t_u8 *)buffer, &cmd_len, cmd_len);
done:
	fclose(config_file);
	if (buffer)
		free(buffer);
	if (line)
		free(line);
	if (buf)
		free(buf);
}

/**
 *  @brief Show usage information for the wfd display status command
 *
 *  $return         N/A
 */
static void
print_wifi_display_status_usage(void)
{
	printf("\nUsage : wifi_display [STATUS]");
	printf("\nOptions: STATUS :     0 - stop wfd display");
	printf("\n                      1 - start wfd display");
	printf("\n                  empty - get current wfd display status\n");
	return;
}

/**
 *  @brief Creates wfd display start or stop request and send to driver
 *
 *   Usage: "Usage : wifi_display [STATUS]"
 *
 *   Options: STATUS :     0 - start wfd status
 *                         1 - stop  wfd status
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         N/A
 */
void
wifidisplay_cmd_status(int argc, char *argv[])
{
	int opt, ret;
	t_u16 data = 0;
	t_u16 cmd_len = 0;
	t_u8 *buffer = NULL;
	wifi_display_mode_config *cmd_buf = NULL;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifi_display_status_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong arguments.\n");
		print_wifi_display_status_usage();
		return;
	}
	if (argc == 3) {
		if ((ISDIGIT(argv[2]) == 0) || (atoi(argv[2]) < 0) ||
		    (atoi(argv[2]) > 3)) {
			printf("ERR:Illegal wfd mode %s. Must be in range from '0' to '3'.\n", argv[2]);
			print_wifi_display_status_usage();
			return;
		}
		data = (t_u16)atoi(argv[2]);
	}

	/* send hostcmd now */
	cmd_len = sizeof(wifi_display_mode_config);
	buffer = (t_u8 *)malloc(cmd_len);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		return;
	}

	cmd_buf = (wifi_display_mode_config *)buffer;
	cmd_buf->cmd_code = HostCmd_CMD_WFD_DISPLAY_MODE_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = ACTION_SET;
		cmd_buf->mode = cpu_to_le16(data);
	}
	cmd_buf->action = cpu_to_le16(cmd_buf->action);
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len, cmd_len);
	if (ret != SUCCESS) {
		printf("Error executing wfd display_mode command\n");
		free(buffer);
		return;
	}

	data = le16_to_cpu(cmd_buf->mode);
	switch (data) {
	case 0:
		printf("wfd display status = stopped\n");
		break;
	case 1:
		printf("wfd display status = started\n");
		break;
	default:
		printf("wfd display status = %d\n", data);
		break;
	}
	free(buffer);
	return;
}

/**
 * @brief Checkes a particular input for validatation for wifidisplay.
 *
 * @param cmd      Type of input
 * @param argc     Number of arguments
 * @param argv     Pointer to the arguments
 * @return         SUCCESS or FAILURE
 */

int
is_wifidisplay_input_valid(display_valid_inputs cmd, int argc, char *argv[])
{
	if (argc == 0)
		return FAILURE;
	if (argc != 1) {
		printf("ERR:Incorrect number of arguments\n");
		return FAILURE;
	}
	switch (cmd) {
	case WFD_DEVICE_INFO:
		/*Bits 10-15 are reserved for device info */
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0)
		    || (atoi(argv[0]) > ((1 << 10) - 1))) {
			printf("ERR:Coupled sink paramater must be > 0 and < %d", ((1 << 10) - 1));
			return FAILURE;
		}
		/*bits 4 and 5 values 10 and 11 are reserved */
		if ((atoi(argv[0]) & 48) > 16) {
			printf("ERR:Coupled sink paramater must not have bit 4 and 5 equal to 10 or 11 ");
			return FAILURE;
		}
		break;
	case WFD_SESSION_MGMT_CONTROL_PORT:
	case WFD_DEVICE_THROUGHPUT:
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0)
		    || (atoi(argv[0]) > ((1 << 16) - 1))) {
			printf("ERR:Paramater must be > 0 and < %d",
			       ((1 << 16) - 1));
			return FAILURE;
		}
		break;
	case WFD_COUPLED_SINK:
		/*Maximum value of coupled sink is 2 */
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0)
		    || (atoi(argv[0]) > 2)) {
			printf("ERR:Coupled sink paramater must be > 0 and < 3");
			return FAILURE;
		}
		break;
	}
	return SUCCESS;
}

#define WFD_IE_HEADER_LEN 3
/**
 *  @brief Wifi display update custom ie command
 *
 *  $return         N/A
*/
void
wifidisplay_update_custom_ie(int argc, char *argv[])
{
	t_s16 display_ie_index = -1;
	t_u8 *buf;
	int opt;
	tlvbuf_custom_ie *tlv = NULL;
	custom_ie *ie_ptr = NULL;
	t_u16 len = 0, ie_len_wfd_org, ie_len_wfd, i = 0, new_wfd_dev_info = 0;
	t_u8 *wfd_ptr;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			printf("ERR:Incorrect arguments.\n");
			return;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong arguments.\n");
		return;
	}
	if (argc == 3) {
		if (ISDIGIT(argv[2]) == 0) {
			printf("ERR:Illegal wfd mode %s. Must not be '0'.\n",
			       argv[2]);
			return;
		}
		new_wfd_dev_info = (t_u16)atoi(argv[2]);
	}
	buf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buf) {
		printf("ERR:Cannot allocate memory!\n");
		return;
	}
	wifiDisplay_ie_config(&display_ie_index, 0, buf);
	/* Clear WPS IE */
	if (display_ie_index < 0) {
		free(buf);
		return;
	}
	if (display_ie_index < (MAX_MGMT_IE_INDEX - 1)) {
		tlv = (tlvbuf_custom_ie *)buf;

		if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
			ie_ptr = (custom_ie *)(tlv->ie_data);
			/* Goto appropriate Ie Index */
			for (i = 0; i < display_ie_index; i++) {
				ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
						       sizeof(custom_ie) +
						       ie_ptr->ie_length);
			}
			ie_len_wfd_org = ie_ptr->ie_length;
			wfd_ptr = ie_ptr->ie_buffer;
			ie_len_wfd = *(wfd_ptr + 1);
			wfd_ptr += sizeof(t_u8) + sizeof(t_u8);
			wfd_ptr += (WIFIDISPLAY_OUI_LEN + WIFIDISPLAY_OUI_TYPE);
			while (ie_len_wfd > WFD_IE_HEADER_LEN) {
				memcpy(&len, wfd_ptr + 1, sizeof(t_u16));
				len = ntohs(len);
				/* check capability type */
				if (*wfd_ptr ==
				    TLV_TYPE_WIFIDISPLAY_DEVICE_INFO) {
					tlvbuf_wfdisplay_device_info *wfd_tlv =
						(tlvbuf_wfdisplay_device_info *)
						wfd_ptr;
					new_wfd_dev_info =
						ntohs(new_wfd_dev_info);
					memcpy((t_u8 *)&wfd_tlv->
					       display_device_info,
					       (t_u8 *)&new_wfd_dev_info,
					       sizeof(t_u16));
					break;
				}
				wfd_ptr += len + WFD_IE_HEADER_LEN;
				ie_len_wfd -= len + WFD_IE_HEADER_LEN;
			}
			/* Update New IE now */
			wifiDisplay_ie_config((t_s16 *)&display_ie_index,
					      ie_len_wfd_org,
					      (t_u8 *)ie_ptr -
					      sizeof(tlvbuf_custom_ie));
		}
	}
	return;
}

/**
 *  @brief Wifi display update custom ie command
 *
 *  $return         N/A
*/
void
wifidisplay_update_coupledsink_bitmap(int argc, char *argv[])
{
	t_s16 display_ie_index = -1;
	t_u8 *buf;
	int opt;
	tlvbuf_custom_ie *tlv = NULL;
	custom_ie *ie_ptr = NULL;
	t_u16 len = 0, ie_len_wfd_org, ie_len_wfd, i = 0;
	t_u8 new_wfd_coupled_sink_bitmap = 0;
	t_u8 *wfd_ptr;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			printf("ERR:Incorrect arguments.\n");
			return;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong arguments.\n");
		return;
	}
	if (argc == 3) {
		if (ISDIGIT(argv[2]) == 0) {
			printf("ERR:Illegal wfd mode %s. Must not be '0'.\n",
			       argv[2]);
			return;
		}
		new_wfd_coupled_sink_bitmap = (t_u16)atoi(argv[2]);
	}
	buf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buf) {
		printf("ERR:Cannot allocate memory!\n");
		return;
	}
	wifiDisplay_ie_config(&display_ie_index, 0, buf);
	/* Clear WPS IE */
	if (display_ie_index < 0) {
		free(buf);
		return;
	}
	if (display_ie_index < (MAX_MGMT_IE_INDEX - 1)) {
		tlv = (tlvbuf_custom_ie *)buf;

		if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
			ie_ptr = (custom_ie *)(tlv->ie_data);
			/* Goto appropriate Ie Index */
			for (i = 0; i < display_ie_index; i++) {
				ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
						       sizeof(custom_ie) +
						       ie_ptr->ie_length);
			}
			ie_len_wfd_org = ie_ptr->ie_length;
			wfd_ptr = ie_ptr->ie_buffer;
			ie_len_wfd = *(wfd_ptr + 1);
			wfd_ptr += sizeof(t_u8) + sizeof(t_u8);
			wfd_ptr += (WIFIDISPLAY_OUI_LEN + WIFIDISPLAY_OUI_TYPE);
			while (ie_len_wfd > WFD_IE_HEADER_LEN) {
				memcpy(&len, wfd_ptr + 1, sizeof(t_u16));
				len = ntohs(len);
				/* check capability type */
				if (*wfd_ptr ==
				    TLV_TYPE_WIFIDISPLAY_COUPLED_SINK) {
					tlvbuf_wfdisplay_coupled_sink *wfd_tlv =
						(tlvbuf_wfdisplay_coupled_sink
						 *)wfd_ptr;
					wfd_tlv->coupled_sink =
						new_wfd_coupled_sink_bitmap;
					break;
				}
				wfd_ptr += len + WFD_IE_HEADER_LEN;
				ie_len_wfd -= len + WFD_IE_HEADER_LEN;
			}
			/* Update New IE now */
			wifiDisplay_ie_config((t_s16 *)&display_ie_index,
					      ie_len_wfd_org,
					      (t_u8 *)ie_ptr -
					      sizeof(tlvbuf_custom_ie));
		}
	}
	free(buf);
	return;
}
