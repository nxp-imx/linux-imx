/** @file  wifidirectutl.c
 *
 *  @brief Program to configure WifiDirect parameters.
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
  07/10/09: Initial creation
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
#include "wifidirectutl.h"
#include "wifi_display.h"
/****************************************************************************
  Definitions
 ****************************************************************************/

/** Convert character to integer */
#define CHAR2INT(x) (((x) >= 'A') ? ((x) - 'A' + 10) : ((x) - '0'))

/** Uncomment this to enable DEBUG */
/* #define DEBUG */

/****************************************************************************
  Global variables
 ****************************************************************************/
/** Device name */
char dev_name[IFNAMSIZ + 1];
/** Option for cmd */
struct option cmd_options[] = {
	{"help", 0, 0, 'h'},
	{0, 0, 0, 0}
};

/****************************************************************************
  Local functions
 ***************************************************************************/
/**
 *  @brief Dump hex data
 *
 *  @param p        A pointer to data buffer
 *  @param len      The len of data buffer
 *  @param delim    Deliminator character
 *  @return         Hex integer
 */
#ifdef DEBUG
static void
hexdump(void *p, t_s32 len, char delim)
{
	t_s32 i;
	t_u8 *s = p;
	printf("HexDump: len=%d\n", (int)len);
	for (i = 0; i < len; i++) {
		if (i != len - 1)
			printf("%02x%c", *s++, delim);
		else
			printf("%02x\n", *s);
		if ((i + 1) % 16 == 0)
			printf("\n");
	}
}
#endif

/**
 *  @brief		wifidirect use fixed ie indices
 *
 *  @return         	1 or 0
 */
static int
wifidir_use_fixed_ie_indices(void)
{
#define WIFIDIR_USE_FIXED_IE_INDICES "WIFIDIR_USE_FIXED_IE_INDICES"
	char *ret = getenv(WIFIDIR_USE_FIXED_IE_INDICES);

	if (ret != NULL && *ret == '1') {
		printf("Using fixed ie indices 0 and 1 for P2P and WPS IEs");
		return 1;
	} else {
		return 0;
	}
}

/**
 *  @brief Converts a string to hex value
 *
 *  @param str      A pointer to the string
 *  @param raw      A pointer to the raw data buffer
 *  @return         Number of bytes read
 */
static int
string2raw(char *str, unsigned char *raw)
{
	int len = (strlen(str) + 1) / 2;

	do {
		if (!isxdigit(*str)) {
			return -1;
		}
		*str = toupper(*str);
		*raw = CHAR2INT(*str) << 4;
		++str;
		*str = toupper(*str);
		if (*str == '\0')
			break;
		*raw |= CHAR2INT(*str);
		++raw;
	} while (*++str != '\0');
	return len;
}

/**
 *    @brief Convert string to hex integer
 *
 *    @param s            A pointer string buffer
 *    @return             Hex integer
 */
unsigned int
a2hex(char *s)
{
	unsigned int val = 0;
	if (!strncasecmp("0x", s, 2)) {
		s += 2;
	}
	while (*s && isxdigit(*s)) {
		val = (val << 4) + hexc2bin(*s++);
	}
	return val;
}

/**
 *  @brief      Hex to number
 *
 *  @param c    Hex value
 *  @return     Integer value or -1
 */
static int
hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return -1;
}

/**
 *    @brief Convert char to hex integer
 *
 *    @param chr          Char
 *    @return             Hex integer
 */
unsigned char
hexc2bin(char chr)
{
	if (chr >= '0' && chr <= '9')
		chr -= '0';
	else if (chr >= 'A' && chr <= 'F')
		chr -= ('A' - 10);
	else if (chr >= 'a' && chr <= 'f')
		chr -= ('a' - 10);

	return chr;
}

/**
 *  @brief              Check hex string
 *
 *  @param hex          A pointer to hex string
 *  @return             SUCCESS or FAILURE
 */
static int
ishexstring(void *hex)
{
	int i, a;
	char *p = hex;
	int len = strlen(p);
	if (!strncasecmp("0x", p, 2)) {
		p += 2;
		len -= 2;
	}
	for (i = 0; i < len; i++) {
		a = hex2num(*p);
		if (a < 0)
			return FAILURE;
		p++;
	}
	return SUCCESS;
}

/**
 *  @brief Prints a MAC address in colon separated form from hex data
 *
 *  @param raw      A pointer to the hex data buffer
 *  @return         N/A
 */
static void
print_mac(t_u8 *raw)
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", (unsigned int)raw[0],
	       (unsigned int)raw[1], (unsigned int)raw[2], (unsigned int)raw[3],
	       (unsigned int)raw[4], (unsigned int)raw[5]);
	return;
}

/**
 *  @brief Converts colon separated MAC address to hex value
 *
 *  @param mac      A pointer to the colon separated MAC string
 *  @param raw      A pointer to the hex data buffer
 *  @return         SUCCESS or FAILURE
 *                  WIFIDIRECT_RET_MAC_BROADCAST  - if broadcast mac
 *                  WIFIDIRECT_RET_MAC_MULTICAST - if multicast mac
 */
int
mac2raw(char *mac, t_u8 *raw)
{
	unsigned int temp_raw[ETH_ALEN];
	int num_tokens = 0;
	int i;
	if (strlen(mac) != ((2 * ETH_ALEN) + (ETH_ALEN - 1))) {
		return FAILURE;
	}
	num_tokens = sscanf(mac, "%2x:%2x:%2x:%2x:%2x:%2x",
			    temp_raw + 0, temp_raw + 1, temp_raw + 2,
			    temp_raw + 3, temp_raw + 4, temp_raw + 5);
	if (num_tokens != ETH_ALEN) {
		return FAILURE;
	}
	for (i = 0; i < num_tokens; i++)
		raw[i] = (t_u8)temp_raw[i];

	if (memcmp(raw, "\xff\xff\xff\xff\xff\xff", ETH_ALEN) == 0) {
		return WIFIDIRECT_RET_MAC_BROADCAST;
	} else if (raw[0] & 0x01) {
		return WIFIDIRECT_RET_MAC_MULTICAST;
	}
	return SUCCESS;
}

/**
 *  @brief Prints usage information of wifidirectutl
 *
 *  @return          N/A
 */
static void
print_tool_usage(void)
{

	printf("Usage:\n");
	printf("./wifidirectutl <iface> wifidirect_mode [mode]\n"
	       "./wifidirectutl <iface> wifidirect_config [*.conf]\n"
	       "./wifidirectutl <iface> wifidirect_params_config [*.conf]\n"
	       "./wifidirectutl <iface> wifidirect_action_frame <*.conf>|"
	       "	<PeerAddr> <Category> <OuiSubtype> <DialogToken>\n"
	       "./wifidirectutl <iface> wifidirect_discovery_request <*.conf>\n"
	       "./wifidirectutl <iface> wifidirect_discovery_response <*.conf>\n"
	       "./wifidirectutl <iface> wifidirect_gas_comeback_request <*.conf>\n"
	       "./wifidirectutl <iface> wifidirect_gas_comeback_response <*.conf>\n"
	       "./wifidirectutl <iface> wifidisplay_config [*.conf]\n"
	       "./wifidirectutl <iface> wifidisplay_mode [enable/disable]\n"
	       "./wifidirectutl <iface> wifidisplay_update_devinfo [value]\n"
	       "./wifidirectutl <iface> wifidisplay_update_coupledsink_bitmap [value <= 255]\n"
	       "./wifidirectutl <iface> wifidisplay_discovery_request [*.config]\n"
	       "./wifidirectutl <iface> wifidisplay_discovery_response [*.config]\n");
	printf("\nPlease see example configuration file config/wifidirect.conf\n\n");
	printf("Configuration API:\n");
	printf("./wifidirectutl <iface> wifidirect_cfg_discovery_period [<MinDiscPeriod> <MaxDiscPeriod>]\n" "./wifidirectutl <iface> wifidirect_cfg_intent [IntentValue]\n" "./wifidirectutl <iface> wifidirect_cfg_capability [<deviceCapability> <groupCapability>]\n" "./wifidirectutl <iface> wifidirect_cfg_noa <enable | disable> [<counttype> <duration> <interval>]\n" "./wifidirectutl <iface> wifidirect_cfg_opp_ps [<enable> <CTWindow>]\n" "./wifidirectutl <iface> wifidirect_cfg_invitation_list [mac_address]\n" "./wifidirectutl <iface> wifidirect_cfg_listen_channel [listenChannel]\n" "./wifidirectutl <iface> wifidirect_cfg_op_channel [operatingChannel]\n" "./wifidirecttul <iface> wifidirect_cfg_persistent_group_record [index] [role]\n" "	[<groupbss> <deviceId> <ssid> <psk>] [peermac1] [peermac2]\n" "./wifidirecttul <iface> wifidirect_cfg_persistent_group_invoke [index] | <cancel>\n" "./wifidirectutl <iface> wifidirect_cfg_presence_req_params [<type> <duration> <interval>]\n" "./wifidirectutl <iface> wifidirect_cfg_ext_listen_time [<duration> <interval>]\n");

}

/**
 *  @brief Parses a command line
 *
 *  @param line     The line to parse
 *  @param args     Pointer to the argument buffer to be filled in
 *  @param args_count Max number of elements which can be filled in buffer 'args'
 *  @return         Number of arguments in the line or EOF
 */
int
parse_line(char *line, char *args[], int args_count)
{
	int arg_num = 0;
	int is_start = 0;
	int is_quote = 0;
	int is_escape = 0;
	int length = 0;
	int i = 0;
	int j = 0;

	arg_num = 0;
	length = strlen(line);
	/* Process line */

	/* Find number of arguments */
	is_start = 0;
	is_quote = 0;
	for (i = 0; (i < length) && (arg_num < args_count); i++) {
		/* Ignore leading spaces */
		if (is_start == 0) {
			if (line[i] == ' ') {
				continue;
			} else if (line[i] == '\t') {
				continue;
			} else if (line[i] == '\n') {
				break;
			} else {
				is_start = 1;
				args[arg_num] = &line[i];
				arg_num++;
			}
		}
		if (is_start == 1) {
			if ((line[i] == '\\') && (i < (length - 1))) {
				if (line[i + 1] == '"') {
					is_escape = 1;
					for (j = i; j < length - 1; j++) {
						line[j] = line[j + 1];
					}
					line[length - 1] = '\0';
					continue;
				}
			}
			/* Ignore comments */
			if (line[i] == '#') {
				if (is_quote == 0) {
					line[i] = '\0';
					arg_num--;
				}
				break;
			}
			/* Separate by '=' */
			if (line[i] == '=') {
				if (is_quote == 0) {
					line[i] = '\0';
					is_start = 0;
					continue;
				}
			}
			/* Separate by ',' */
			if (line[i] == ',') {
				if (is_quote == 0) {
					line[i] = '\0';
					is_start = 0;
					continue;
				}
			}
			/* Change ',' to ' ', but not inside quotes */
			if ((line[i] == ',') && (is_quote == 0)) {
				line[i] = ' ';
				continue;
			}
		}
		/* Remove newlines */
		if (line[i] == '\n') {
			line[i] = '\0';
		}
		/* Check for quotes */
		if (line[i] == '"') {
			if (is_escape) {
				is_escape = 0;
				/* no change in is_quote */
			} else {
				is_quote = (is_quote == 1) ? 0 : 1;
			}
			continue;
		}
		if (((line[i] == ' ') || (line[i] == '\t')) && (is_quote == 0)) {
			line[i] = '\0';
			is_start = 0;
			continue;
		}
	}
	return arg_num;
}

/**
 *  @brief Get one line from the File
 *
 *  @param fp       File handler
 *  @param str      Storage location for data.
 *  @param size     Maximum number of characters to read.
 *  @param lineno   A pointer to return current line number
 *  @return         returns string or NULL
 */
char *
mlan_config_get_line(FILE * fp, char *str, t_s32 size, int *lineno)
{
	char *start, *end;
	int out, next_line;

	if (!fp || !str)
		return NULL;

	do {
read_line:
		if (!fgets(str, size, fp))
			break;
		start = str;
		start[size - 1] = '\0';
		end = start + strlen(str);
		(*lineno)++;

		out = 1;
		while (out && (start < end)) {
			next_line = 0;
			/* Remove empty lines and lines starting with # */
			switch (start[0]) {
			case ' ':	/* White space */
			case '\t':	/* Tab */
				start++;
				break;
			case '#':
			case '\n':
			case '\0':
				next_line = 1;
				break;
			case '\r':
				if (start[1] == '\n')
					next_line = 1;
				else
					start++;
				break;
			default:
				out = 0;
				break;
			}
			if (next_line)
				goto read_line;
		}

		/* Remove # comments unless they are within a double quoted
		 * string. Remove trailing white space. */
		end = strstr(start, "\"");
		if (end) {
			end = strstr(end + 1, "\"");
			if (!end)
				end = start;
		} else
			end = start;

		end = strstr(end + 1, "#");
		if (end)
			*end-- = '\0';
		else
			end = start + strlen(start) - 1;

		out = 1;
		while (out && (start < end)) {
			switch (*end) {
			case ' ':	/* White space */
			case '\t':	/* Tab */
			case '\n':
			case '\r':
				*end = '\0';
				end--;
				break;
			default:
				out = 0;
				break;
			}
		}

		if (*start == '\0')
			continue;

		return start;
	} while (1);

	return NULL;
}

/**
 *  @brief      Parse function for a configuration line
 *
 *  @param s        Storage buffer for data
 *  @param size     Maximum size of data
 *  @param stream   File stream pointer
 *  @param line     Pointer to current line within the file
 *  @param _pos     Output string or NULL
 *  @return     String or NULL
 */
char *
config_get_line(char *s, int size, FILE * stream, int *line, char **_pos)
{
	*_pos = mlan_config_get_line(stream, s, size, line);
	return *_pos;
}

/**
 *  @brief  Detects duplicates channel in array of strings
 *
 *  @param  argc    Number of elements
 *  @param  argv    Array of strings
 *  @return UAP_FAILURE or UAP_SUCCESS
 */
static inline int
has_dup_channel(int argc, char *argv[])
{
	int i, j;
	/* Check for duplicate */
	for (i = 0; i < (argc - 1); i++) {
		for (j = i + 1; j < argc; j++) {
			if (atoi(argv[i]) == atoi(argv[j])) {
				return FAILURE;
			}
		}
	}
	return SUCCESS;
}

/**
 *  @brief Performs the ioctl operation to send the command to
 *  the driver.
 *
 *  @param cmd           Pointer to the command buffer
 *  @param size          Pointer to the command size. This value is
 *                       overwritten by the function with the size of the
 *                       received response.
 *  @param buf_size      Size of the allocated command buffer
 *  @return              SUCCESS or FAILURE
 */
int
wifidirect_ioctl(t_u8 *cmd, t_u16 *size, t_u16 buf_size)
{
	struct ifreq ifr;
	mrvl_priv_cmd *mrvl_cmd = NULL;
	t_u8 *buf = NULL, *temp = NULL;
	wifidirectcmdbuf *header = NULL;
	t_s32 sockfd;
	t_u16 mrvl_header_len = 0;
	int ret = SUCCESS;

	if (buf_size < *size) {
		printf("buf_size should not less than cmd buffer size\n");
		return FAILURE;
	}

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return FAILURE;
	}
	*(t_u32 *)cmd = buf_size - BUF_HEADER_SIZE;

	mrvl_header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_HOSTCMD);
	buf = (unsigned char *)malloc(buf_size + sizeof(mrvl_priv_cmd) +
				      mrvl_header_len);
	if (buf == NULL) {
		close(sockfd);
		return FAILURE;
	}

	memset(buf, 0, buf_size + sizeof(mrvl_priv_cmd) + mrvl_header_len);
	/* Fill up buffer */
	mrvl_cmd = (mrvl_priv_cmd *)buf;
	mrvl_cmd->buf = buf + sizeof(mrvl_priv_cmd);
	mrvl_cmd->used_len = 0;
	mrvl_cmd->total_len = buf_size + mrvl_header_len;
	/* Copy NXP command string */
	temp = mrvl_cmd->buf;
	strncpy((char *)temp, CMD_NXP, strlen(CMD_NXP));
	temp += (strlen(CMD_NXP));
	/* Insert command string */
	strncpy((char *)temp, PRIV_CMD_HOSTCMD, strlen(PRIV_CMD_HOSTCMD));
	temp += (strlen(PRIV_CMD_HOSTCMD));

	memcpy(temp, (t_u8 *)cmd, *size);

	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)mrvl_cmd;
	header = (wifidirectcmdbuf *)(buf + sizeof(mrvl_priv_cmd) +
				      mrvl_header_len);
	header->size = *size - BUF_HEADER_SIZE;
#ifdef DEBUG
	/* Debug print */
	hexdump(mrvl_cmd, *size + sizeof(mrvl_priv_cmd) + mrvl_header_len, ' ');
#endif
	endian_convert_request_header(header);

	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, MRVLPRIVCMD, &ifr)) {
		perror("");
		printf("ERR:MRVLPRIVCMD is not supported by %s\n", dev_name);
		ret = FAILURE;
		goto done;
	}
	endian_convert_response_header(header);
	header->cmd_code &= HostCmd_CMD_ID_MASK;
	header->cmd_code |= WIFIDIRECTCMD_RESP_CHECK;
	*size = header->size;
	memcpy(cmd, buf + sizeof(mrvl_priv_cmd) + mrvl_header_len,
	       *size + BUF_HEADER_SIZE);
#ifdef DEBUG
	/* Debug print */
	hexdump(mrvl_cmd,
		*size + BUF_HEADER_SIZE + sizeof(mrvl_priv_cmd) +
		mrvl_header_len, ' ');
#endif

	/* Validate response size */
	if (*size > (buf_size - BUF_HEADER_SIZE)) {
		printf("ERR:Response size (%d) greater than buffer size (%d)! Aborting!\n", *size, buf_size);
		ret = FAILURE;
		goto done;
	}
done:
	/* Close socket */
	close(sockfd);
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Show usage information for the wifidirect_gas_comeback_discovery commands
 *
 *  $return         N/A
 */
static void
print_wifidirect_gas_comeback_usage(void)
{
	printf("\nUsage : wifidirect_gas_comeback_request/response [CONFIG_FILE]\n");
	printf("CONFIG_FILE contains WIFIDIRECT GAS comeback request/response payload.\n");
	return;
}

/**
 *  @brief Creates a wifidirect_gas_comeback_service_discovery request/response and
 *         sends to the driver
 *
 *  Usage: "Usage : wifidirect_gas_comeback_request/response [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 **/
static void
wifidirectcmd_gas_comeback_discovery(int argc, char *argv[])
{
	wifidirect_gas_comeback_request *req_buf = NULL;
	wifidirect_gas_comeback_response *resp_buf = NULL;
	char *line = NULL;
	FILE *config_file = NULL;
	int i, opt, li = 0, arg_num = 0, ret = 0, wifidirect_level = 0;
	char *args[30], *pos = NULL, wifidirect_mac[20], wifidirect_cmd[40];
	t_u8 dev_address[ETH_ALEN], cmd_found = 0;
	t_u8 *buffer = NULL, *tmp_buffer = NULL;
	t_u8 req_resp = 0;	/* req = 0, resp = 1 */
	t_u16 cmd_len = 0, query_len = 0, vendor_len = 0, service_len = 0;
	t_u16 dns_len = 0, record_len = 0, upnp_len = 0;

	memset(wifidirect_mac, 0, sizeof(wifidirect_mac));
	strncpy(wifidirect_cmd, argv[2], sizeof(wifidirect_cmd) - 1);
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_gas_comeback_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 3) {
		printf("ERR:Incorrect number of arguments.\n");
		print_wifidirect_gas_comeback_usage();
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

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
		arg_num = parse_line(line, args, 30);
		if (!cmd_found &&
		    strncmp(args[0], wifidirect_cmd, strlen(args[0])))
			continue;

		if (strcmp(args[0], "wifidirect_gas_comeback_request") == 0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			/* For wifidirect_service_gas_comeback, basic initialization here */
			/* Subtract extra two bytes added as a part of query request structure */
			cmd_len = sizeof(wifidirect_gas_comeback_request) - 2;
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			req_buf = (wifidirect_gas_comeback_request *)buffer;
			req_buf->cmd_code =
				HostCmd_CMD_WIFIDIRECT_SERVICE_DISCOVERY;
			req_buf->size = cmd_len;
			req_buf->seq_num = 0;
			req_buf->result = 0;
			cmd_found = 1;

		} else if (strcmp(args[0], "wifidirect_gas_comeback_response")
			   == 0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			req_resp = 1;
			/* For wifidirect_service_discovery, basic initialization here */
			/* Subtract extra two bytes added as a part of query response structure */
			cmd_len = sizeof(wifidirect_gas_comeback_response) - 2;
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			resp_buf = (wifidirect_gas_comeback_response *)buffer;
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
			strncpy(wifidirect_mac, args[1],
				sizeof(wifidirect_mac) - 1);
			if ((ret =
			     mac2raw(wifidirect_mac, dev_address)) != SUCCESS) {
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
			if (req_resp) {
				resp_buf->status_code =
					(t_u16)A2HEXDECIMAL(args[1]);
				resp_buf->status_code =
					cpu_to_le16(resp_buf->status_code);
			}
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
		} else if (strcmp(args[0], "GasResponseFragID") == 0) {
			if (req_resp)
				resp_buf->gas_fragment_id =
					(t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "AdvertizementProtocolIE") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_ADPROTOIE, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			for (i = 0; i < arg_num - 1; i++) {
				if (req_resp)
					resp_buf->advertize_protocol_ie[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			}
		} else if (strcmp(args[0], "InfoId") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_INFOID, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			for (i = 0; i < arg_num - 1; i++) {
				if (req_resp)
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
			for (i = 0; i < arg_num - 1; i++) {
				if (req_resp)
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
			if (req_resp)
				resp_buf->oui_sub_type = (t_u8)atoi(args[1]);
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
		} else if (strcmp(args[0], "ServiceProtocol") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICEPROTO, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->service_protocol =
					(t_u8)atoi(args[1]);
			vendor_len++;
			service_len++;
			query_len++;
			/*
			 * For uPnP, due to union allocation, a extra byte is allocated
			 * reduce it here for uPnP
			 */
			cmd_len--;
		} else if (strcmp(args[0], "ServiceUpdateIndicator") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_SERVICEUPDATE_INDICATOR, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->service_update_indicator =
					cpu_to_le16((t_u16)atoi(args[1]));
			service_len += 2;
			query_len += 2;
		} else if (strcmp(args[0], "ServiceTransactionId") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICETRANSACID, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->service_transaction_id =
					(t_u8)atoi(args[1]);
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "ServiceStatus") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICE_STATUS, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->disc_status_code =
					(t_u8)atoi(args[1]);
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "DNSName") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				dns_len = strlen(args[1]);
				tmp_buffer = realloc(buffer, cmd_len + dns_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add DNS name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (req_resp)
					strncpy((char *)resp_buf->disc_resp.u.
						bonjour.dns, args[1],
						strlen(args[1]));
			} else {
				/* HEX input */
				dns_len = arg_num - 1;
				tmp_buffer = realloc(buffer, cmd_len + dns_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add DNS name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				for (i = 0; i < arg_num - 1; i++) {
					if (req_resp)
						resp_buf->disc_resp.u.bonjour.
							dns[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				}
			}
			cmd_len += dns_len;
			vendor_len += dns_len;
			service_len += dns_len;
			query_len += dns_len;
		} else if (strcmp(args[0], "DNSType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_DNSTYPE, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp) {
				*(&resp_buf->disc_resp.u.bonjour.dns_type +
				  dns_len) = (t_u8)A2HEXDECIMAL(args[1]);
			}
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "BonjourVersion") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_BONJOUR_VERSION, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp) {
				*(&resp_buf->disc_resp.u.bonjour.version +
				  dns_len) = (t_u8)atoi(args[1]);
			}
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "uPnPVersion") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_UPNP_VERSION, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->disc_resp.u.upnp.version =
					(t_u8)A2HEXDECIMAL(args[1]);
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "uPnPQueryValue") == 0 ||
			   strcmp(args[0], "uPnPResponseValue") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				upnp_len = strlen(args[1]);
				tmp_buffer =
					realloc(buffer, cmd_len + upnp_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add uPnP value to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (req_resp)
					strncpy((char *)resp_buf->disc_resp.u.
						upnp.value, args[1], upnp_len);
			} else {
				/* HEX input */
				upnp_len = arg_num - 1;
				tmp_buffer =
					realloc(buffer, cmd_len + upnp_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add uPnP value to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				for (i = 0; i < arg_num - 1; i++) {
					if (req_resp)
						resp_buf->disc_resp.u.upnp.
							value[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				}
			}
			cmd_len += upnp_len;
			vendor_len += upnp_len;
			service_len += upnp_len;
			query_len += upnp_len;
		} else if (strcmp(args[0], "RecordData") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				record_len = strlen(args[1]);
				tmp_buffer =
					realloc(buffer, cmd_len + record_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				}
				strncpy((char *)resp_buf->disc_resp.u.bonjour.
					record, args[1], strlen(args[1]));
			} else {
				/* HEX input */
				record_len = arg_num - 1;
				tmp_buffer =
					realloc(buffer, cmd_len + record_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				}
				for (i = 0; i < arg_num - 1; i++)
					resp_buf->disc_resp.u.bonjour.
						record[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);

			}
			cmd_len += record_len;
			vendor_len += record_len;
			service_len += record_len;
			query_len += record_len;
		} else if (strcmp(args[0], "}") == 0) {
			switch (wifidirect_level) {
			case WIFIDIRECT_DISCOVERY_QUERY:
				if (req_resp)
					resp_buf->query_len =
						cpu_to_le16(query_len);
				break;
			case WIFIDIRECT_DISCOVERY_SERVICE:
				if (req_resp)
					resp_buf->response_len =
						cpu_to_le16(service_len);
				break;
			case WIFIDIRECT_DISCOVERY_VENDOR:
				if (req_resp)
					resp_buf->vendor_len =
						cpu_to_le16(vendor_len);
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
}

/**
 *  @brief Show usage information for the wifidirect_discovery commands
 *
 *  $return         N/A
 */
static void
print_wifidirect_discovery_usage(void)
{
	printf("\nUsage : wifidirect_discovery_request/response [CONFIG_FILE]\n");
	printf("CONFIG_FILE contains WIFIDIRECT service discovery payload.\n");
	return;
}

/**
 *  @brief Creates a wifidirect_service_discovery request/response and
 *         sends to the driver
 *
 *  Usage: "Usage : wifidirect_discovery_request/response [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 **/
static void
wifidirectcmd_service_discovery(int argc, char *argv[])
{
	wifidirect_discovery_request *req_buf = NULL;
	wifidirect_discovery_response *resp_buf = NULL;
	char *line = NULL;
	FILE *config_file = NULL;
	int i, opt, li = 0, arg_num = 0, ret = 0, wifidirect_level = 0;
	char *args[30], *pos = NULL, wifidirect_mac[20], wifidirect_cmd[32];
	t_u8 dev_address[ETH_ALEN], cmd_found = 0;
	t_u8 *buffer = NULL, *tmp_buffer = NULL;
	t_u8 req_resp = 0;	/* req = 0, resp = 1 */
	t_u16 cmd_len = 0, query_len = 0, vendor_len = 0, service_len = 0;
	t_u16 dns_len = 0, record_len = 0, upnp_len = 0;

	memset(wifidirect_mac, 0, sizeof(wifidirect_mac));
	strncpy(wifidirect_cmd, argv[2], sizeof(wifidirect_cmd) - 1);
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_discovery_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 3) {
		printf("ERR:Incorrect number of arguments.\n");
		print_wifidirect_discovery_usage();
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

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
		arg_num = parse_line(line, args, 30);
		if (!cmd_found &&
		    strncmp(args[0], wifidirect_cmd, strlen(args[0])))
			continue;

		if (strcmp(args[0], "wifidirect_discovery_request") == 0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			/* For wifidirect_service_discovery, basic initialization here */
			cmd_len = sizeof(wifidirect_discovery_request);
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			req_buf = (wifidirect_discovery_request *)buffer;
			req_buf->cmd_code =
				HostCmd_CMD_WIFIDIRECT_SERVICE_DISCOVERY;
			req_buf->size = cmd_len;
			req_buf->seq_num = 0;
			req_buf->result = 0;
			cmd_found = 1;

		} else if (strcmp(args[0], "wifidirect_discovery_response") ==
			   0) {
			wifidirect_level =
				WIFIDIRECT_DISCOVERY_REQUEST_RESPONSE;
			req_resp = 1;
			/* For wifidirect_service_discovery, basic initialization here */
			cmd_len = sizeof(wifidirect_discovery_response);
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				goto done;
			}
			resp_buf = (wifidirect_discovery_response *)buffer;
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
			strncpy(wifidirect_mac, args[1],
				sizeof(wifidirect_mac) - 1);
			if ((ret =
			     mac2raw(wifidirect_mac, dev_address)) != SUCCESS) {
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
				for (i = 0;
				     i < MIN(arg_num - 1, MAX_ADPROTOIE_LEN);
				     i++)
					req_buf->advertize_protocol_ie[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0;
				     i < MIN(arg_num - 1, MAX_ADPROTOIE_LEN);
				     i++)
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
				for (i = 0;
				     i < MIN(arg_num - 1, MAX_INFOID_LEN); i++)
					req_buf->info_id[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0;
				     i < MIN(arg_num - 1, MAX_INFOID_LEN); i++)
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
				for (i = 0; i < MIN(arg_num - 1, MAX_OUI_LEN);
				     i++)
					req_buf->oui[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);
			} else {
				for (i = 0; i < MIN(arg_num - 1, MAX_OUI_LEN);
				     i++)
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
		} else if (strcmp(args[0], "ServiceProtocol") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICEPROTO, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
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
		} else if (strcmp(args[0], "ServiceStatus") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_SERVICE_STATUS, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			if (req_resp)
				resp_buf->disc_status_code =
					(t_u8)atoi(args[1]);
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "DNSName") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				dns_len = strlen(args[1]);
				tmp_buffer = realloc(buffer, cmd_len + dns_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add DNS name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				(!req_resp) ?
					(strncpy
					 ((char *)req_buf->disc_query.u.bonjour.
					  dns, args[1],
					  strlen(args[1]))) : (strncpy((char *)
								       resp_buf->
								       disc_resp.
								       u.
								       bonjour.
								       dns,
								       args[1],
								       strlen
								       (args
									[1])));
			} else {
				/* HEX input */
				dns_len = arg_num - 1;
				tmp_buffer = realloc(buffer, cmd_len + dns_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add DNS name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					for (i = 0; i < arg_num - 1; i++)
						req_buf->disc_query.u.bonjour.
							dns[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				} else {
					for (i = 0; i < arg_num - 1; i++)
						resp_buf->disc_resp.u.bonjour.
							dns[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				}
			}
			cmd_len += dns_len;
			vendor_len += dns_len;
			service_len += dns_len;
			query_len += dns_len;
		} else if (strcmp(args[0], "DNSType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_DNSTYPE, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ?
				(*
				 (&req_buf->disc_query.u.bonjour.dns_type +
				  dns_len) =
				 (t_u8)A2HEXDECIMAL(args[1])) : (*(&resp_buf->
								   disc_resp.u.
								   bonjour.
								   dns_type +
								   dns_len) =
								 (t_u8)
								 A2HEXDECIMAL
								 (args[1]));
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "BonjourVersion") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_BONJOUR_VERSION, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ?
				(*
				 (&req_buf->disc_query.u.bonjour.version +
				  dns_len) =
				 (t_u8)atoi(args[1])) : (*(&resp_buf->disc_resp.
							   u.bonjour.version +
							   dns_len) =
							 (t_u8)atoi(args[1]));
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "uPnPVersion") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DISC_UPNP_VERSION, arg_num - 1,
			     args + 1)
			    != SUCCESS) {
				goto done;
			}
			(!req_resp) ?
				(req_buf->disc_query.u.upnp.version =
				 (t_u8)A2HEXDECIMAL(args[1])) : (resp_buf->
								 disc_resp.u.
								 upnp.version =
								 (t_u8)
								 A2HEXDECIMAL
								 (args[1]));
			vendor_len++;
			service_len++;
			query_len++;
		} else if (strcmp(args[0], "uPnPQueryValue") == 0 ||
			   strcmp(args[0], "uPnPResponseValue") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				upnp_len = strlen(args[1]);
				tmp_buffer =
					realloc(buffer, cmd_len + upnp_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add uPnP value to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				(!req_resp) ?
					(strncpy
					 ((char *)req_buf->disc_query.u.upnp.
					  value, args[1],
					  upnp_len)) : (strncpy((char *)
								resp_buf->
								disc_resp.u.
								upnp.value,
								args[1],
								upnp_len));
			} else {
				/* HEX input */
				upnp_len = arg_num - 1;
				tmp_buffer =
					realloc(buffer, cmd_len + upnp_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add uPnP value to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					for (i = 0; i < arg_num - 1; i++)
						req_buf->disc_query.u.upnp.
							value[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				} else {
					for (i = 0; i < arg_num - 1; i++)
						resp_buf->disc_resp.u.upnp.
							value[i] =
							(t_u8)
							A2HEXDECIMAL(args
								     [i + 1]);
				}
			}
			cmd_len += upnp_len;
			vendor_len += upnp_len;
			service_len += upnp_len;
			query_len += upnp_len;
		} else if (strcmp(args[0], "RecordData") == 0) {
			if (args[1][0] == '"') {
				args[1]++;
				if (args[1][strlen(args[1]) - 1] == '"')
					args[1][strlen(args[1]) - 1] = '\0';

				record_len = strlen(args[1]);
				tmp_buffer =
					realloc(buffer, cmd_len + record_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				}
				strncpy((char *)resp_buf->disc_resp.u.bonjour.
					record + dns_len, args[1],
					strlen(args[1]));
			} else {
				/* HEX input */
				record_len = arg_num - 1;
				tmp_buffer =
					realloc(buffer, cmd_len + record_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				if (!req_resp) {
					printf("ERR:Cannot add Record name to buffer!\n");
					goto done;
				}
				for (i = 0; i < arg_num - 1; i++)
					*(&resp_buf->disc_resp.u.bonjour.
					  record[i] + dns_len) =
		 (t_u8)A2HEXDECIMAL(args[i + 1]);
			}
			cmd_len += record_len;
			vendor_len += record_len;
			service_len += record_len;
			query_len += record_len;
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
}

/**
 *  @brief Show usage information for the wifidirect_config command
 *
 *  $return         N/A
 */
static void
print_wifidirect_config_usage(void)
{
	printf("\nUsage : wifidirect_config [CONFIG_FILE]\n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("CONFIG_FILE contains all WIFIDIRECT parameters.\n");
	return;
}

/**
 *  @brief Read the wifidirect parameters and sends to the driver
 *
 *  @param file_name File to open for configuration parameters.
 *  @param cmd_name  Command Name for which parameters are read.
 *  @param pbuf      Pointer to output buffer
 *  @param ie_len_wifidirect Length of wifidirect parameters to return
 *  @param ie_len_wps Length of WPS parameters to return
 *  @return          SUCCESS or FAILURE
 */
static const t_u8 wifidirect_oui[] = { 0x50, 0x6F, 0x9A, 0x09 };
static const t_u8 wps_oui[] = { 0x00, 0x50, 0xF2, 0x04 };

static void
wifidirect_file_params_config(char *file_name, char *cmd_name, t_u8 *pbuf,
			      t_u16 *ie_len_wifidirect, t_u16 *ie_len_wps)
{
	FILE *config_file = NULL;
	char *line = NULL;
	int i = 0, li = 0, arg_num = 0, ret = 0, wifidirect_level =
		0, no_of_chan_entries = 0, no_of_noa = 0;
	int secondary_index = -1, flag = 1, group_secondary_index = -1;
	char **args = NULL;
	char *pos = NULL;
	char wifidirect_mac[20], country[4], wifidirect_ssid[33];
	char WPS_manufacturer[33], WPS_modelname[33], WPS_devicename[33],
		wifi_group_direct_ssid[33];
	t_u8 dev_channels[MAX_CHANNELS];
	t_u8 iface_list[ETH_ALEN * MAX_INTERFACE_ADDR_COUNT];
	t_u8 dev_address[ETH_ALEN];
	t_u8 group_dev_address[ETH_ALEN];
	t_u8 *extra = NULL;
	t_u8 *buffer = pbuf;
	t_u16 cmd_len_wifidirect = 0, cmd_len_wps = 0, tlv_len = 0, extra_len =
		0, temp = 0;
	t_u16 wps_model_len = 0, wps_serial_len = 0, wps_vendor_len = 0;
	t_u16 pri_category = 0, pri_sub_category = 0, config_methods = 0;
	t_u16 sec_category = 0, sec_sub_category = 0, group_sec_sub_category =
		0, group_sec_category = 0;
	t_u16 avail_period = 0, avail_interval = 0;
	t_u8 secondary_oui[4], group_secondary_oui[4];
	t_u16 WPS_specconfigmethods = 0, WPS_associationstate = 0,
		WPS_configurationerror = 0, WPS_devicepassword = 0;
	t_u8 dev_capability = 0, group_capability = 0, cmd_found = 0,
		group_owner_intent = 0, primary_oui[4], iface_count = 0,
		regulatory_class = 0, channel_number = 0, manageability = 0,
		op_regulatory_class = 0, op_channel_number =
		0, invitation_flag = 0;
	t_u8 WPS_version = 0, WPS_setupstate = 0, WPS_requesttype =
		0, WPS_responsetype =
		0, WPS_UUID[WPS_UUID_MAX_LEN],
		WPS_primarydevicetype[WPS_DEVICE_TYPE_MAX_LEN], WPS_RFband =
		0, WPS_modelnumber[32], WPS_serialnumber[32], WPS_VendorExt[32];
	t_u8 go_config_timeout = 0, client_config_timeout = 0;
	t_u8 secondary_dev_count = 0, group_secondary_dev_count = 0;
	t_u16 temp16 = 0;
	t_u8 secondary_dev_info[WPS_DEVICE_TYPE_LEN *
				MAX_SECONDARY_DEVICE_COUNT];
	t_u8 group_secondary_dev_info[WPS_DEVICE_TYPE_LEN *
				      MAX_GROUP_SECONDARY_DEVICE_COUNT];
	t_u8 wifidirect_client_dev_count = 0, wifidirect_client_dev_index =
		0, temp8 = 0;
	wifidirect_client_dev_info
		wifidirect_client_dev_info_list[MAX_SECONDARY_DEVICE_COUNT];
	t_u8 wifidirect_total_secondary_dev_count = 0;
	t_u8 wifidirect_group_total_ssid_len = 0, tlv_offset =
		0, temp_dev_size = 0;
	t_u8 noa_index = 0, opp_ps = 0, ctwindow_opp_ps = 0, count_type = 0;
	t_u32 duration = 0, interval = 0, start_time = 0;
	t_u16 total_chan_len = 0;
	t_u8 chan_entry_regulatory_class = 0, chan_entry_num_of_channels = 0;
	t_u8 *chan_entry_list = NULL;
	t_u8 *chan_buf = NULL;
	noa_descriptor noa_descriptor_list[MAX_NOA_DESCRIPTORS];
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

	if (!wifidir_use_fixed_ie_indices()) {
		special_mask_custom_ie_buf *wifidirect_ie_buf;
		wifidirect_ie_buf = (special_mask_custom_ie_buf *)buffer;
		memcpy(&wifidirect_ie_buf->Oui[0], wifidirect_oui,
		       sizeof(wifidirect_oui));
		cmd_len_wifidirect += sizeof(wifidirect_oui);
	}
	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
		arg_num = parse_line(line, args, MAX_ARGS_NUM);
		if (!cmd_found && strncmp(args[0], cmd_name, strlen(args[0])))
			continue;
		cmd_found = 1;
		if (strcmp(args[0], "wifidirect_config") == 0) {
			wifidirect_level = WIFIDIRECT_PARAMS_CONFIG;
		} else if (strcmp(args[0], "Capability") == 0) {
			wifidirect_level = WIFIDIRECT_CAPABILITY_CONFIG;
		} else if (strcmp(args[0], "GroupOwnerIntent") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_OWNER_INTENT_CONFIG;
		} else if (strcmp(args[0], "Channel") == 0) {
			wifidirect_level = WIFIDIRECT_CHANNEL_CONFIG;
		} else if (strcmp(args[0], "OperatingChannel") == 0) {
			wifidirect_level = WIFIDIRECT_OPCHANNEL_CONFIG;
		} else if (strcmp(args[0], "InfrastructureManageabilityInfo") ==
			   0) {
			wifidirect_level = WIFIDIRECT_MANAGEABILITY_CONFIG;
		} else if (strcmp(args[0], "InvitationFlagBitmap") == 0) {
			wifidirect_level = WIFIDIRECT_INVITATION_FLAG_CONFIG;
		} else if (strcmp(args[0], "ChannelList") == 0) {
			wifidirect_level = WIFIDIRECT_CHANNEL_LIST_CONFIG;
		} else if (strcmp(args[0], "NoticeOfAbsence") == 0) {
			wifidirect_level = WIFIDIRECT_NOTICE_OF_ABSENCE;
		} else if (strcmp(args[0], "NoA_descriptor") == 0) {
			wifidirect_level = WIFIDIRECT_NOA_DESCRIPTOR;
		} else if (strcmp(args[0], "DeviceInfo") == 0) {
			wifidirect_level = WIFIDIRECT_DEVICE_INFO_CONFIG;
		} else if (strcmp(args[0], "SecondaryDeviceType") == 0) {
			wifidirect_level = WIFIDIRECT_DEVICE_SEC_INFO_CONFIG;
		} else if (strcmp(args[0], "GroupInfo") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_INFO_CONFIG;
		} else if (strcmp(args[0], "GroupSecondaryDeviceTypes") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_SEC_INFO_CONFIG;
		} else if (strcmp(args[0], "GroupWifiDirectDeviceTypes") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_CLIENT_INFO_CONFIG;
		} else if (strcmp(args[0], "GroupId") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_ID_CONFIG;
		} else if (strcmp(args[0], "GroupBSSId") == 0) {
			wifidirect_level = WIFIDIRECT_GROUP_BSS_ID_CONFIG;
		} else if (strcmp(args[0], "DeviceId") == 0) {
			wifidirect_level = WIFIDIRECT_DEVICE_ID_CONFIG;
		} else if (strcmp(args[0], "Interface") == 0) {
			wifidirect_level = WIFIDIRECT_INTERFACE_CONFIG;
		} else if (strcmp(args[0], "ConfigurationTimeout") == 0) {
			wifidirect_level = WIFIDIRECT_TIMEOUT_CONFIG;
		} else if (strcmp(args[0], "ExtendedListenTime") == 0) {
			wifidirect_level = WIFIDIRECT_EXTENDED_TIME_CONFIG;
		} else if (strcmp(args[0], "IntendedIntfAddress") == 0) {
			wifidirect_level = WIFIDIRECT_INTENDED_ADDR_CONFIG;
		} else if (strcmp(args[0], "WPSIE") == 0) {
			wifidirect_level = WIFIDIRECT_WPSIE;
		} else if (strcmp(args[0], "Extra") == 0) {
			wifidirect_level = WIFIDIRECT_EXTRA;
		} else if (strcmp(args[0], "WIFIDIRECT_MAC") == 0 ||
			   strcmp(args[0], "GroupAddr") == 0 ||
			   strcmp(args[0], "GroupBssId") == 0 ||
			   strcmp(args[0], "InterfaceAddress") == 0 ||
			   strcmp(args[0], "GroupInterfaceAddress") == 0 ||
			   strcmp(args[0], "DeviceAddress") == 0) {
			strncpy(wifidirect_mac, args[1], 20 - 1);
			if ((ret =
			     mac2raw(wifidirect_mac, dev_address)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
		} else if (strncmp(args[0], "GroupWifiDirectDeviceAddress", 21)
			   == 0) {
			strncpy(wifidirect_mac, args[1], 20 - 1);
			if ((ret =
			     mac2raw(wifidirect_mac,
				     group_dev_address)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
			wifidirect_client_dev_index++;
			if (wifidirect_client_dev_index >
			    wifidirect_client_dev_count) {
				printf("ERR: No of Client Dev count is less than no of client dev configs!!\n");
				goto done;
			}
			group_secondary_index = 0;
			tlv_offset =
				wifidirect_group_total_ssid_len +
				wifidirect_total_secondary_dev_count *
				WPS_DEVICE_TYPE_LEN;
			memcpy(wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].wifidirect_dev_address + tlv_offset,
			       group_dev_address, ETH_ALEN);
		} else if (strncmp(args[0], "GroupWifiDirectIntfAddress", 19) ==
			   0) {
			strncpy(wifidirect_mac, args[1], 20 - 1);
			if ((ret =
			     mac2raw(wifidirect_mac,
				     group_dev_address)) != SUCCESS) {
				printf("ERR: %s Address \n",
				       ret == FAILURE ? "Invalid MAC" : ret ==
				       WIFIDIRECT_RET_MAC_BROADCAST ?
				       "Broadcast" : "Multicast");
				goto done;
			}
			memcpy(wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].wifidirect_intf_address + tlv_offset,
			       group_dev_address, ETH_ALEN);
		} else if (strncmp(args[0], "GroupWifiDirectDeviceCapab", 19) ==
			   0) {
			if (is_input_valid
			    (WIFIDIRECT_DEVICECAPABILITY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			temp8 = (t_u8)atoi(args[1]);
			memcpy(&wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].wifidirect_dev_capability + tlv_offset,
			       &temp8, sizeof(temp8));
		} else if (strncmp
			   (args[0], "GroupWifiDirectWPSConfigMethods",
			    24) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSCONFMETHODS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			temp16 = (t_u16)A2HEXDECIMAL(args[1]);
			memcpy(&wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].config_methods + tlv_offset, &temp16,
			       sizeof(temp16));
			(t_u16)A2HEXDECIMAL(args[1]);
		} else if (strncmp
			   (args[0], "GroupPrimaryDeviceTypeCategory",
			    30) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPECATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			pri_category = (t_u16)atoi(args[1]);
			temp16 = htons(pri_category);
			memcpy(&wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].primary_category + tlv_offset, &temp16,
			       sizeof(temp16));
		} else if (strncmp(args[0], "GroupPrimaryDeviceTypeOUI", 25) ==
			   0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPEOUI, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < 4; i++) {
				temp8 = (t_u8)A2HEXDECIMAL(args[i + 1]);
				memcpy(&wifidirect_client_dev_info_list
				       [wifidirect_client_dev_index -
					1].primary_oui[i]
				       + tlv_offset, &temp8, sizeof(temp8));
			}
		} else if (strncmp
			   (args[0], "GroupPrimaryDeviceTypeSubCategory",
			    33) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPESUBCATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			pri_sub_category = (t_u16)atoi(args[1]);
			temp16 = htons(pri_sub_category);
			memcpy(&wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].primary_subcategory + tlv_offset, &temp16,
			       sizeof(temp16));
		} else if (strncmp(args[0], "GroupSecondaryDeviceCount", 25) ==
			   0) {
			if (is_input_valid
			    (WIFIDIRECT_GROUP_SECONDARYDEVCOUNT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			group_secondary_dev_count = (t_u8)atoi(args[1]);
			memcpy(&wifidirect_client_dev_info_list
			       [wifidirect_client_dev_index -
				1].wifidirect_secondary_dev_count + tlv_offset,
			       &group_secondary_dev_count,
			       sizeof(group_secondary_dev_count));
			wifidirect_total_secondary_dev_count +=
				group_secondary_dev_count;
			if (group_secondary_dev_count)
				memset(group_secondary_dev_info, 0,
				       sizeof(group_secondary_dev_info));
		} else if (strncmp
			   (args[0], "GroupSecondaryDeviceTypeCategory",
			    30) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPECATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			sec_category = (t_u16)atoi(args[1]);
			group_sec_category = cpu_to_le16(sec_category);
			group_secondary_index++;
		} else if (strncmp(args[0], "GroupSecondaryDeviceTypeOUI", 27)
			   == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPEOUI, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < 4; i++)
				group_secondary_oui[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
		} else if (strncmp
			   (args[0], "GroupSecondaryDeviceTypeSubCategory",
			    35) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPESUBCATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			if (group_secondary_index < 0 ||
			    group_secondary_index >=
			    MAX_SECONDARY_DEVICE_COUNT) {
				printf("Error in configuration file %s:%d",
				       file_name, li);
				goto done;
			}
			sec_sub_category = (t_u16)atoi(args[1]);
			group_sec_sub_category =
				cpu_to_le16(group_sec_sub_category);
			if (group_secondary_dev_count) {
				memcpy(&group_secondary_dev_info
				       [(group_secondary_index -
					 1) * WPS_DEVICE_TYPE_LEN],
				       &group_sec_category, sizeof(t_u16));
				memcpy(&group_secondary_dev_info
				       [((group_secondary_index -
					  1) * WPS_DEVICE_TYPE_LEN) + 2],
				       group_secondary_oui,
				       sizeof(secondary_oui));
				memcpy(&group_secondary_dev_info
				       [((group_secondary_index -
					  1) * WPS_DEVICE_TYPE_LEN) + 6],
				       &group_sec_sub_category, sizeof(t_u16));
			}

		} else if (strncmp(args[0], "GroupWifiDirectDeviceCount", 19) ==
			   0) {
			if (is_input_valid
			    (WIFIDIRECT_SECONDARYDEVCOUNT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			wifidirect_client_dev_count = (t_u8)atoi(args[1]);
		} else if (strncmp(args[0], "GroupWifiDirectDeviceName", 18) ==
			   0) {

			if (is_input_valid
			    (WIFIDIRECT_GROUP_WIFIDIRECT_DEVICE_NAME,
			     arg_num - 1, args + 1) != SUCCESS) {
				goto done;
			}

			strncpy(wifi_group_direct_ssid, args[1] + 1,
				strlen(args[1]) - 2);
			wifi_group_direct_ssid[strlen(args[1]) - 2] = '\0';
			temp = htons(SC_Device_Name);
			memcpy(((t_u8 *)
				&wifidirect_client_dev_info_list
				[wifidirect_client_dev_index -
				 1].wifidirect_device_name_type + tlv_offset),
			       &temp, sizeof(temp));
			temp = htons(strlen(wifi_group_direct_ssid));
			memcpy(((t_u8 *)
				&wifidirect_client_dev_info_list
				[wifidirect_client_dev_index -
				 1].wifidirect_device_name_len + tlv_offset),
			       &temp, sizeof(temp));
			memset(((t_u8 *)
				&wifidirect_client_dev_info_list
				[wifidirect_client_dev_index -
				 1].wifidirect_device_name + tlv_offset), 0,
			       strlen(wifi_group_direct_ssid));
			memcpy(((t_u8 *)
				&wifidirect_client_dev_info_list
				[wifidirect_client_dev_index -
				 1].wifidirect_device_name + tlv_offset),
			       &wifi_group_direct_ssid,
			       strlen(wifi_group_direct_ssid));
			wifidirect_group_total_ssid_len +=
				strlen(wifi_group_direct_ssid);

			if (wifidirect_client_dev_index - 1) {
				temp_dev_size =
					sizeof(wifidirect_client_dev_info) +
					strlen(wifi_group_direct_ssid) +
					group_secondary_dev_count *
					WPS_DEVICE_TYPE_LEN;
				memcpy(&wifidirect_client_dev_info_list
				       [wifidirect_client_dev_index -
					1].dev_length + (tlv_offset -
							 (group_secondary_dev_count
							  *
							  WPS_DEVICE_TYPE_LEN)),
				       &temp_dev_size, sizeof(temp_dev_size));
			} else {

				temp_dev_size =
					sizeof(wifidirect_client_dev_info) +
					strlen(wifi_group_direct_ssid) +
					group_secondary_dev_count *
					WPS_DEVICE_TYPE_LEN;
				wifidirect_client_dev_info_list
					[wifidirect_client_dev_index -
					 1].dev_length =
					sizeof(wifidirect_client_dev_info) +
					strlen(wifi_group_direct_ssid) +
					group_secondary_dev_count *
					WPS_DEVICE_TYPE_LEN;
			}
		} else if (strcmp(args[0], "DeviceCapability") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DEVICECAPABILITY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			dev_capability = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "GroupCapability") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_GROUPCAPABILITY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			group_capability = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "Intent") == 0) {
			/* Intent -> 0 - 15 */
			if (is_input_valid
			    (WIFIDIRECT_INTENT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			group_owner_intent = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "RegulatoryClass") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_REGULATORYCLASS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			regulatory_class = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "ChannelNumber") == 0) {
			if (is_input_valid(CHANNEL, arg_num - 1, args + 1) !=
			    SUCCESS) {
				goto done;
			}
			channel_number = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "OpRegulatoryClass") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_REGULATORYCLASS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			op_regulatory_class = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "OpChannelNumber") == 0) {
			if (is_input_valid(CHANNEL, arg_num - 1, args + 1) !=
			    SUCCESS) {
				goto done;
			}
			op_channel_number = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "Manageability") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_MANAGEABILITY, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			manageability = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "InvitationFlag") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_INVITATIONFLAG, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			invitation_flag = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "CountryString") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_COUNTRY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			strncpy(country, args[1] + 1, 3);
			country[strlen(args[1]) - 2] = '\0';
		} else if (strncmp(args[0], "Regulatory_Class_", 17) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_REGULATORYCLASS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			if (!no_of_chan_entries) {
				chan_entry_list =
					(t_u8 *)malloc(MAX_BUFFER_SIZE);
				if (!chan_entry_list) {
					printf("ERR:cannot allocate memory for chan_entry_list!\n");
					goto done;
				}
				memset(chan_entry_list, 0, MAX_BUFFER_SIZE);
				chan_buf = chan_entry_list;
			}
			no_of_chan_entries++;
			chan_entry_regulatory_class = (t_u8)atoi(args[1]);
		} else if (strncmp(args[0], "NumofChannels", 13) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_NO_OF_CHANNELS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			chan_entry_num_of_channels = (t_u8)atoi(args[1]);
		} else if (strncmp(args[0], "ChanList", 8) == 0) {
			if (chan_entry_num_of_channels != (arg_num - 1)) {
				printf("ERR:no of channels in ChanList and NumofChannels do not match!\n");
				goto done;
			}
			if (is_input_valid(SCANCHANNELS, arg_num - 1, args + 1)
			    != SUCCESS) {
				goto done;
			}
			for (i = 0; i < chan_entry_num_of_channels; i++)
				dev_channels[i] = (t_u8)atoi(args[i + 1]);
			total_chan_len += chan_entry_num_of_channels;
			memcpy(chan_buf, &chan_entry_regulatory_class,
			       sizeof(t_u8));
			memcpy(chan_buf + 1, &chan_entry_num_of_channels,
			       sizeof(t_u8));
			memcpy(chan_buf + 2, dev_channels,
			       chan_entry_num_of_channels);
			chan_buf +=
				sizeof(t_u8) + sizeof(t_u8) +
				chan_entry_num_of_channels;
		} else if (strcmp(args[0], "NoA_Index") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_NOA_INDEX, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			noa_index = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "OppPS") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_OPP_PS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			opp_ps = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "CTWindow") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_CTWINDOW, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			if ((opp_ps != 0) && (atoi(args[1]) < 10)) {
				printf("ERR: CTwindow should be greater than or equal to 10 if opp_ps is set!\n");
				goto done;
			}
			ctwindow_opp_ps =
				(t_u8)atoi(args[1]) | SET_OPP_PS(opp_ps);
		} else if (strncmp(args[0], "CountType", 9) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_COUNT_TYPE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			no_of_noa++;
			if (no_of_noa > MAX_NOA_DESCRIPTORS) {
				printf("Number of descriptors should not be greater than %d\n", MAX_NOA_DESCRIPTORS);
				goto done;
			}
			count_type = (t_u8)atoi(args[1]);
			noa_descriptor_list[no_of_noa - 1].count_type =
				count_type;
		} else if (strncmp(args[0], "Duration", 8) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_DURATION, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			duration = (t_u32)atoi(args[1]);
			duration = cpu_to_le32(duration);
			noa_descriptor_list[no_of_noa - 1].duration = duration;
		} else if (strncmp(args[0], "Interval", 8) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_INTERVAL, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			interval = (t_u32)atoi(args[1]);
			interval = cpu_to_le32(interval);
			noa_descriptor_list[no_of_noa - 1].interval = interval;
		} else if (strncmp(args[0], "StartTime", 9) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_START_TIME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			start_time = (t_u32)atoi(args[1]);
			start_time = cpu_to_le32(start_time);
			noa_descriptor_list[no_of_noa - 1].start_time =
				start_time;
		} else if (strcmp(args[0], "PrimaryDeviceTypeCategory") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPECATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			pri_category = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "PrimaryDeviceTypeOUI") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPEOUI, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < 4; i++)
				primary_oui[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
		} else if (strcmp(args[0], "PrimaryDeviceTypeSubCategory") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPESUBCATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			pri_sub_category = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "SecondaryDeviceCount") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_SECONDARYDEVCOUNT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			secondary_dev_count = (t_u8)atoi(args[1]);
			if (secondary_dev_count)
				memset(secondary_dev_info, 0,
				       sizeof(secondary_dev_info));
		} else if (strncmp(args[0], "SecondaryDeviceTypeCategory", 27)
			   == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPECATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			sec_category = (t_u16)atoi(args[1]);
			sec_category = htons(sec_category);
			secondary_index++;
		} else if (strncmp(args[0], "SecondaryDeviceTypeOUI", 22) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPEOUI, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < 4; i++)
				secondary_oui[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
		} else if (strncmp
			   (args[0], "SecondaryDeviceTypeSubCategory",
			    30) == 0) {
			if (is_input_valid
			    (WIFIDIRECT_PRIDEVTYPESUBCATEGORY, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			if (secondary_index < 0 ||
			    secondary_index >= MAX_SECONDARY_DEVICE_COUNT) {
				printf("Error in configuration file %s:%d",
				       file_name, li);
				goto done;
			}
			sec_sub_category = (t_u16)atoi(args[1]);
			sec_sub_category = htons(sec_sub_category);
			if (secondary_dev_count) {
				memcpy(&secondary_dev_info
				       [secondary_index * WPS_DEVICE_TYPE_LEN],
				       &sec_category, sizeof(t_u16));
				memcpy(&secondary_dev_info
				       [(secondary_index *
					 WPS_DEVICE_TYPE_LEN) + 2],
				       secondary_oui, sizeof(secondary_oui));
				memcpy(&secondary_dev_info
				       [(secondary_index *
					 WPS_DEVICE_TYPE_LEN) + 6],
				       &sec_sub_category, sizeof(t_u16));
			}
		} else if (strcmp(args[0], "InterfaceAddressCount") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_INTERFACECOUNT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			iface_count = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "InterfaceAddressList") == 0) {
			if (iface_count != (arg_num - 1)) {
				printf("Incorrect address list for %d entries.\n", iface_count);
				goto done;
			}
			for (i = 0;
			     i < iface_count && i < MAX_INTERFACE_ADDR_COUNT;
			     i++) {
				if ((ret =
				     mac2raw(args[i + 1],
					     &iface_list[i * ETH_ALEN])) !=
				    SUCCESS) {
					printf("ERR: %s Address \n",
					       ret ==
					       FAILURE ? "Invalid MAC" : ret ==
					       WIFIDIRECT_RET_MAC_BROADCAST ?
					       "Broadcast" : "Multicast");
					goto done;
				}
			}
		} else if (strcmp(args[0], "GroupConfigurationTimeout") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_ATTR_CONFIG_TIMEOUT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			go_config_timeout = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "ClientConfigurationTimeout") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_ATTR_CONFIG_TIMEOUT, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			client_config_timeout = (t_u8)atoi(args[1]);
		} else if (strcmp(args[0], "AvailabilityPeriod") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_ATTR_EXTENDED_TIME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			avail_period = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "AvailabilityInterval") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_ATTR_EXTENDED_TIME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			avail_interval = (t_u16)atoi(args[1]);
		} else if (strcmp(args[0], "WPSConfigMethods") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSCONFMETHODS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			config_methods = (t_u16)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "DeviceName") == 0 ||
			   strcmp(args[0], "GroupSsId") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSDEVICENAME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			memset(wifidirect_ssid, 0, sizeof(wifidirect_ssid));
			strncpy(wifidirect_ssid, args[1],
				sizeof(wifidirect_ssid) - 1);
		} else if (strcmp(args[0], "WPSVersion") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSVERSION, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_version = (t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSSetupState") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSSETUPSTATE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_setupstate = (t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSDeviceName") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSDEVICENAME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			memset(WPS_devicename, 0, sizeof(WPS_devicename));
			strncpy(WPS_devicename, args[1],
				sizeof(WPS_devicename) - 1);
		} else if (strcmp(args[0], "WPSRequestType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSREQRESPTYPE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_requesttype = (t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSResponseType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSREQRESPTYPE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_responsetype = (t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSSpecConfigMethods") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSSPECCONFMETHODS, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_specconfigmethods = (t_u16)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSUUID") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSUUID, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < WPS_UUID_MAX_LEN; i++)
				WPS_UUID[i] = (t_u8)A2HEXDECIMAL(args[i + 1]);
		} else if (strcmp(args[0], "WPSPrimaryDeviceType") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSPRIMARYDEVICETYPE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < WPS_DEVICE_TYPE_MAX_LEN; i++)
				WPS_primarydevicetype[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
		} else if (strcmp(args[0], "WPSRFBand") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSRFBAND, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_RFband = (t_u8)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSAssociationState") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSASSOCIATIONSTATE, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_associationstate = (t_u16)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSConfigurationError") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSCONFIGURATIONERROR, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_configurationerror = (t_u16)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSDevicePassword") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSDEVICEPASSWORD, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			WPS_devicepassword = (t_u16)A2HEXDECIMAL(args[1]);
		} else if (strcmp(args[0], "WPSManufacturer") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSMANUFACTURER, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			memset(WPS_manufacturer, 0, sizeof(WPS_manufacturer));
			strncpy(WPS_manufacturer, args[1],
				sizeof(WPS_manufacturer));
		} else if (strcmp(args[0], "WPSModelName") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSMODELNAME, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			memset(WPS_modelname, 0, sizeof(WPS_modelname));
			strncpy(WPS_modelname, args[1], sizeof(WPS_modelname));
		} else if (strcmp(args[0], "WPSModelNumber") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSMODELNUMBER, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < arg_num - 1; i++)
				WPS_modelnumber[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
			wps_model_len = arg_num - 1;
		} else if (strcmp(args[0], "WPSSerialNumber") == 0) {
			if (is_input_valid
			    (WIFIDIRECT_WPSSERIALNUMBER, arg_num - 1,
			     args + 1) != SUCCESS) {
				goto done;
			}
			for (i = 0; i < arg_num - 1; i++)
				WPS_serialnumber[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
			wps_serial_len = arg_num - 1;
		} else if (strcmp(args[0], "WPSVendorExtension") == 0) {
			for (i = 0; i < arg_num - 1; i++)
				WPS_VendorExt[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
			wps_vendor_len = arg_num - 1;
		} else if (strcmp(args[0], "Buffer") == 0) {
			for (i = 0; i < arg_num - 1; i++)
				extra[i] = (t_u8)A2HEXDECIMAL(args[i + 1]);
			extra_len = arg_num - 1;
		} else if (strcmp(args[0], "}") == 0) {
			/* Based on level, populate appropriate struct */
			switch (wifidirect_level) {
			case WIFIDIRECT_DEVICE_ID_CONFIG:
				{
					tlvbuf_wifidirect_device_id *tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_device_id);
					tlv = (tlvbuf_wifidirect_device_id
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_DEVICE_ID;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->dev_mac_address,
					       dev_address, ETH_ALEN);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_CAPABILITY_CONFIG:
				{
					tlvbuf_wifidirect_capability *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_capability);
					tlv = (tlvbuf_wifidirect_capability
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_CAPABILITY;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->dev_capability = dev_capability;
					tlv->group_capability =
						group_capability;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_GROUP_OWNER_INTENT_CONFIG:
				{
					tlvbuf_wifidirect_group_owner_intent
						*tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_group_owner_intent);
					tlv = (tlvbuf_wifidirect_group_owner_intent *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_GROUPOWNER_INTENT;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->dev_intent = group_owner_intent;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_MANAGEABILITY_CONFIG:
				{
					tlvbuf_wifidirect_manageability *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_manageability);
					tlv = (tlvbuf_wifidirect_manageability
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_MANAGEABILITY;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->manageability = manageability;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_INVITATION_FLAG_CONFIG:
				{
					tlvbuf_wifidirect_invitation_flag *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_invitation_flag);
					tlv = (tlvbuf_wifidirect_invitation_flag
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_INVITATION_FLAG;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->invitation_flag |= invitation_flag;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_CHANNEL_LIST_CONFIG:
				{
					tlvbuf_wifidirect_channel_list *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_channel_list)
						+
						no_of_chan_entries *
						sizeof(chan_entry) +
						total_chan_len;
					tlv = (tlvbuf_wifidirect_channel_list
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_CHANNEL_LIST;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->country_string, country, 3);
					if (tlv->country_string[2] == 0)
						tlv->country_string[2] =
							WIFIDIRECT_COUNTRY_LAST_BYTE;
					memcpy(tlv->wifidirect_chan_entry_list,
					       chan_entry_list,
					       (tlv->length - 3));
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_NOTICE_OF_ABSENCE:
				{
					tlvbuf_wifidirect_notice_of_absence *tlv
						= NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_notice_of_absence)
						+
						no_of_noa *
						sizeof(noa_descriptor);
					tlv = (tlvbuf_wifidirect_notice_of_absence *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_NOTICE_OF_ABSENCE;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->noa_index = noa_index;
					tlv->ctwindow_opp_ps = ctwindow_opp_ps;
					memcpy(tlv->
					       wifidirect_noa_descriptor_list,
					       noa_descriptor_list,
					       no_of_noa *
					       sizeof(noa_descriptor));
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					flag = 1;
					break;
				}
			case WIFIDIRECT_NOA_DESCRIPTOR:
				{
					wifidirect_level =
						WIFIDIRECT_NOTICE_OF_ABSENCE;
					flag = 0;
					break;
				}
			case WIFIDIRECT_CHANNEL_CONFIG:
				{
					tlvbuf_wifidirect_channel *tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_channel);
					tlv = (tlvbuf_wifidirect_channel
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag = TLV_TYPE_WIFIDIRECT_CHANNEL;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->country_string, country, 3);
					if (tlv->country_string[2] == 0)
						tlv->country_string[2] =
							WIFIDIRECT_COUNTRY_LAST_BYTE;
					tlv->regulatory_class =
						regulatory_class;
					tlv->channel_number = channel_number;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_OPCHANNEL_CONFIG:
				{
					tlvbuf_wifidirect_channel *tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_channel);
					tlv = (tlvbuf_wifidirect_channel
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_OPCHANNEL;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->country_string, country, 3);
					if (tlv->country_string[2] == 0)
						tlv->country_string[2] =
							WIFIDIRECT_COUNTRY_LAST_BYTE;
					tlv->regulatory_class =
						op_regulatory_class;
					tlv->channel_number = op_channel_number;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}

			case WIFIDIRECT_DEVICE_INFO_CONFIG:
				{
					tlvbuf_wifidirect_device_info *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_device_info)
						+
						secondary_dev_count *
						WPS_DEVICE_TYPE_LEN +
						strlen(wifidirect_ssid);
					tlv = (tlvbuf_wifidirect_device_info
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_DEVICE_INFO;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->dev_address, dev_address,
					       ETH_ALEN);
					tlv->config_methods =
						htons(config_methods);
					tlv->primary_category =
						htons(pri_category);
					memcpy(tlv->primary_oui, primary_oui,
					       4);
					tlv->primary_subcategory =
						htons(pri_sub_category);
					tlv->secondary_dev_count =
						secondary_dev_count;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					/* Parameters within secondary_dev_info are already htons'ed */
					memcpy(tlv->secondary_dev_info,
					       secondary_dev_info,
					       secondary_dev_count *
					       WPS_DEVICE_TYPE_LEN);
					temp = htons(SC_Device_Name);
					memcpy(((t_u8 *)(&tlv->
							 device_name_type)) +
					       secondary_dev_count *
					       WPS_DEVICE_TYPE_LEN, &temp,
					       sizeof(temp));
					temp = htons(strlen(wifidirect_ssid));
					memcpy(((t_u8 *)(&tlv->
							 device_name_len)) +
					       secondary_dev_count *
					       WPS_DEVICE_TYPE_LEN, &temp,
					       sizeof(temp));
					memcpy(((t_u8 *)(&tlv->device_name)) +
					       secondary_dev_count *
					       WPS_DEVICE_TYPE_LEN,
					       wifidirect_ssid,
					       strlen(wifidirect_ssid));
					flag = 1;
					break;
				}
			case WIFIDIRECT_GROUP_INFO_CONFIG:
				{
					tlvbuf_wifidirect_group_info *tlv =
						NULL;
					/* Append a new TLV */
					tlv_offset =
						wifidirect_group_total_ssid_len
						+
						wifidirect_total_secondary_dev_count
						* WPS_DEVICE_TYPE_LEN;
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_group_info)
						+
						wifidirect_client_dev_count *
						sizeof
						(wifidirect_client_dev_info) +
						tlv_offset;
					tlv = (tlvbuf_wifidirect_group_info
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_GROUP_INFO;
					tlv->length = tlv_len;
					memcpy(tlv->wifidirect_client_dev_list,
					       wifidirect_client_dev_info_list,
					       wifidirect_client_dev_count *
					       sizeof
					       (wifidirect_client_dev_info) +
					       tlv_offset);
					/* Parameters within secondary_dev_info are already htons'ed */
					//wps_hexdump(DEBUG_WLAN,"Group Info Hexdump:", (t_u8*)tlv, tlv_len);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					flag = 1;
					break;
				}
			case WIFIDIRECT_GROUP_SEC_INFO_CONFIG:
				{
					wifidirect_level =
						WIFIDIRECT_GROUP_CLIENT_INFO_CONFIG;

					if (wifidirect_client_dev_index &&
					    group_secondary_index) {
						memset(((t_u8 *)
							&wifidirect_client_dev_info_list
							[wifidirect_client_dev_index
							 -
							 1].
							wifidirect_secondary_dev_info
							+ tlv_offset), 0,
						       group_secondary_index *
						       WPS_DEVICE_TYPE_LEN);
						memcpy(((t_u8 *)
							&wifidirect_client_dev_info_list
							[wifidirect_client_dev_index
							 -
							 1].
							wifidirect_secondary_dev_info
							+ tlv_offset),
						       &group_secondary_dev_info,
						       group_secondary_index *
						       WPS_DEVICE_TYPE_LEN);
					}
					tlv_offset =
						wifidirect_group_total_ssid_len
						+
						wifidirect_total_secondary_dev_count
						* WPS_DEVICE_TYPE_LEN;
					flag = 0;
					break;
				}
			case WIFIDIRECT_GROUP_CLIENT_INFO_CONFIG:
				{
					wifidirect_level =
						WIFIDIRECT_GROUP_INFO_CONFIG;
					flag = 0;
					break;
				}
			case WIFIDIRECT_DEVICE_SEC_INFO_CONFIG:
				{
					wifidirect_level =
						WIFIDIRECT_DEVICE_INFO_CONFIG;
					flag = 0;
					break;
				}
			case WIFIDIRECT_GROUP_ID_CONFIG:
				{
					tlvbuf_wifidirect_group_id *tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_group_id) +
						strlen(wifidirect_ssid);
					tlv = (tlvbuf_wifidirect_group_id
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag = TLV_TYPE_WIFIDIRECT_GROUP_ID;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->group_address, dev_address,
					       ETH_ALEN);
					memcpy(tlv->group_ssid, wifidirect_ssid,
					       strlen(wifidirect_ssid));
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_GROUP_BSS_ID_CONFIG:
				{
					tlvbuf_wifidirect_group_bss_id *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_group_bss_id);
					tlv = (tlvbuf_wifidirect_group_bss_id
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_GROUP_BSS_ID;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->group_bssid, dev_address,
					       ETH_ALEN);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_INTERFACE_CONFIG:
				{
					tlvbuf_wifidirect_interface *tlv = NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_interface)
						+ iface_count * ETH_ALEN;
					tlv = (tlvbuf_wifidirect_interface
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_INTERFACE;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->interface_id, dev_address,
					       ETH_ALEN);
					tlv->interface_count = iface_count;
					memcpy(tlv->interface_idlist,
					       iface_list,
					       iface_count * ETH_ALEN);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}

			case WIFIDIRECT_TIMEOUT_CONFIG:
				{
					tlvbuf_wifidirect_config_timeout *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_config_timeout);
					tlv = (tlvbuf_wifidirect_config_timeout
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_CONFIG_TIMEOUT;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->group_config_timeout =
						go_config_timeout;
					tlv->device_config_timeout =
						client_config_timeout;
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_EXTENDED_TIME_CONFIG:
				{
					tlvbuf_wifidirect_ext_listen_time *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_ext_listen_time);
					tlv = (tlvbuf_wifidirect_ext_listen_time
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_EXTENDED_LISTEN_TIME;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					tlv->availability_period =
						le16_to_cpu(avail_period);
					tlv->availability_interval =
						le16_to_cpu(avail_interval);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}
			case WIFIDIRECT_INTENDED_ADDR_CONFIG:
				{
					tlvbuf_wifidirect_intended_addr *tlv =
						NULL;
					/* Append a new TLV */
					tlv_len =
						sizeof
						(tlvbuf_wifidirect_intended_addr);
					tlv = (tlvbuf_wifidirect_intended_addr
					       *)(buffer + cmd_len_wifidirect);
					cmd_len_wifidirect += tlv_len;
					/* Set TLV fields */
					tlv->tag =
						TLV_TYPE_WIFIDIRECT_INTENDED_ADDRESS;
					tlv->length =
						tlv_len - (sizeof(t_u8) +
							   sizeof(t_u16));
					memcpy(tlv->group_address, dev_address,
					       ETH_ALEN);
					endian_convert_tlv_wifidirect_header_out
						(tlv);
					break;
				}

			case WIFIDIRECT_WPSIE:
				{
#ifdef DEBUG
					/* Debug print */
					hexdump(buffer, cmd_len_wifidirect,
						' ');
#endif
					/* Append TLV for WPSVersion */
					tlvbuf_wps_ie *tlv = NULL;
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_version);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);

					if (!wifidir_use_fixed_ie_indices()) {
						special_mask_custom_ie_buf
							*wps_ie_buf;
						wps_ie_buf =
							(special_mask_custom_ie_buf
							 *)tlv;
						memcpy(&wps_ie_buf->Oui[0],
						       wps_oui,
						       sizeof(wps_oui));
						cmd_len_wps += sizeof(wps_oui);
					}
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Version;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					*(tlv->data) = WPS_version;
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSSetupState */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_setupstate);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Simple_Config_State;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					*(tlv->data) = WPS_setupstate;
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSRequestType */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_requesttype);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Request_Type;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					*(tlv->data) = WPS_requesttype;
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSResponseType */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_responsetype);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Response_Type;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					*(tlv->data) = WPS_responsetype;
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSSpecConfigMethods */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_specconfigmethods);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Config_Methods;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					temp = htons(WPS_specconfigmethods);
					memcpy((t_u16 *)tlv->data, &temp,
					       sizeof(temp));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSUUID */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_UUID);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_UUID_E;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_UUID,
					       WPS_UUID_MAX_LEN);
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSPrimaryDeviceType */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_primarydevicetype);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Primary_Device_Type;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_primarydevicetype,
					       WPS_DEVICE_TYPE_MAX_LEN);
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSRFBand */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_RFband);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_RF_Band;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					*(tlv->data) = WPS_RFband;
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSAssociationState */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_associationstate);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Association_State;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					temp = htons(WPS_associationstate);
					memcpy((t_u16 *)tlv->data, &temp,
					       sizeof(temp));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSConfigurationError */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_configurationerror);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Configuration_Error;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					temp = htons(WPS_configurationerror);
					memcpy((t_u16 *)tlv->data, &temp,
					       sizeof(temp));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSDevicePassword */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						sizeof(WPS_devicepassword);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Device_Password_ID;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					temp = htons(WPS_devicepassword);
					memcpy((t_u16 *)tlv->data, &temp,
					       sizeof(temp));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSDeviceName */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						strlen(WPS_devicename);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Device_Name;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_devicename,
					       strlen(WPS_devicename));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSManufacturer */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						strlen(WPS_manufacturer);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Manufacturer;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_manufacturer,
					       strlen(WPS_manufacturer));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSModelName */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						strlen(WPS_modelname);
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Model_Name;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_modelname,
					       strlen(WPS_modelname));
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSModelNumber */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						wps_model_len;
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Model_Number;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_modelnumber,
					       wps_model_len);
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSSerialNumber */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						wps_serial_len;
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Serial_Number;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_serialnumber,
					       wps_serial_len);
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;

					/* Append TLV for WPSVendorExtension */
					tlv_len =
						sizeof(tlvbuf_wps_ie) +
						wps_vendor_len;
					tlv = (tlvbuf_wps_ie *)(buffer +
								cmd_len_wifidirect
								+
								sizeof
								(custom_ie) +
								cmd_len_wps);
					tlv->tag = SC_Vendor_Extension;
					tlv->length =
						tlv_len - 2 * sizeof(t_u16);
					memcpy(tlv->data, WPS_VendorExt,
					       wps_vendor_len);
					endian_convert_tlv_wps_header_out(tlv);
					cmd_len_wps += tlv_len;
#ifdef DEBUG
					/* Debug print */
					hexdump(buffer + sizeof(custom_ie) +
						cmd_len_wifidirect, cmd_len_wps,
						' ');
#endif
					break;
				}
			case WIFIDIRECT_EXTRA:
				{
					memcpy(buffer + cmd_len_wifidirect,
					       extra, extra_len);
					cmd_len_wifidirect += extra_len;
					break;
				}
			default:
				*ie_len_wifidirect = cmd_len_wifidirect;
				if (ie_len_wps)
					*ie_len_wps = cmd_len_wps;
				break;
			}
			memset(country, 0, sizeof(country));
			if (wifidirect_level == 0)
				cmd_found = 0;
			if (flag)
				wifidirect_level = 0;
		}
	}

done:
	fclose(config_file);
	if (chan_entry_list)
		free(chan_entry_list);
	if (line)
		free(line);
	if (extra)
		free(extra);
	if (args)
		free(args);
	return;
}

/**
 *  @brief Process and send ie config command
 *  @param ie_index  A pointer to the IE buffer index
 *  @param data_len_wifidirect  Length of WIFIDIRECT data, 0 to get, else set.
 *  @param data_len_wps  Length of WPS data, 0 to get, else set.
 *  @param buf      Pointer to buffer to set.
 *  @return          SUCCESS--success, FAILURE--fail
 */
static int
wifidirect_ie_config(t_u16 *ie_index, t_u16 data_len_wifidirect,
		     t_u16 data_len_wps, t_u8 *buf)
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

	/* Set TLV fields : WIFIDIRECT IE parameters */
	if (data_len_wifidirect) {
		/* Set IE */
#define MGMT_MASK_AUTO	0xffff
		ie_ptr->mgmt_subtype_mask = MGMT_MASK_AUTO;
		tlv->length = sizeof(custom_ie) + data_len_wifidirect;
		ie_ptr->ie_length = data_len_wifidirect;
		ie_ptr->ie_index = *ie_index;
	} else {
		/* Get WPS IE */
		tlv->length = 0;
	}

	if (!wifidir_use_fixed_ie_indices()) {
		if (*ie_index != 0xFFFF) {
			(*ie_index)++;
		}
	} else {
		(*ie_index)++;
	}
	/* Locate headers */
	ie_ptr = (custom_ie *)((t_u8 *)(tlv->ie_data) + sizeof(custom_ie) +
			       data_len_wifidirect);

	/* Set WPS IE parameters */
	if (data_len_wps) {
		/* Set IE */
		/* Firmware Handled IE - mask should be set to -1 */
		ie_ptr->mgmt_subtype_mask = MGMT_MASK_AUTO;
		tlv->length += sizeof(custom_ie) + data_len_wps;
		ie_ptr->ie_length = data_len_wps;
		ie_ptr->ie_index = *ie_index;
	}

	memset(&iwr, 0, sizeof(iwr));
	strncpy(iwr.ifr_name, (char *)dev_name, IFNAMSIZ - 1);

	iwr.u.data.pointer = (void *)buf;
	iwr.u.data.length =
		((2 * sizeof(custom_ie)) + sizeof(tlvbuf_custom_ie) +
		 data_len_wifidirect + data_len_wps);
	iwr.u.data.flags = 0;

	/*
	 * create a socket
	 */
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

/** Max IE index */
#define MAX_MGMT_IE_INDEX               12

	if (!data_len_wifidirect) {
		/* Get the IE buffer index number for MGMT_IE_LIST_TLV */
		tlv = (tlvbuf_custom_ie *)buf;
		*ie_index = 0xFFFF;
		if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
			ie_ptr = (custom_ie *)(tlv->ie_data);
			for (i = 0; i < MAX_MGMT_IE_INDEX; i++) {
				/* zero mask indicates a wps IE, return previous index */
				if (ie_ptr->mgmt_subtype_mask == MGMT_MASK_AUTO
				    && ie_ptr->ie_length) {
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
		if (*ie_index == 0xFFFF) {
			ret = FAILURE;
		}
	}
_exit_:

	return ret;
}

/**
 *  @brief Creates a wifidirect_config request and sends to the driver
 *
 *  Usage: "Usage : wifidirect_config [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirectcmd_config(int argc, char *argv[])
{
	t_u8 *buf = NULL, *ptr, *dev_ptr, *array_ptr;
	t_u16 ie_len_wifidirect = 0, ie_len_wps = 0, ie_len;
	t_u16 ie_index, temp;
	int i, opt, ret = SUCCESS;
	tlvbuf_custom_ie *tlv = NULL;
	custom_ie *ie_ptr = NULL;
	t_u16 len = 0, len_wifidirect = 0;
	t_u8 type = 0;
	t_u16 wps_len = 0, wps_type = 0;

	if (!wifidir_use_fixed_ie_indices()) {
		ie_index = 0xFFFF;
	} else {
		ie_index = 0;
	}
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_config_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_config_usage();
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
		wifidirect_file_params_config(argv[2], argv[1], buf
					      + sizeof(tlvbuf_custom_ie) +
					      sizeof(custom_ie),
					      &ie_len_wifidirect, &ie_len_wps);
		if (argc == 4) {
			ie_index = atoi(argv[3]);
			if (ie_index >= 4) {
				printf("ERR:wrong argument %s.\n", argv[3]);
				return;
			}
		}
		if (ie_len_wifidirect > MAX_SIZE_IE_BUFFER ||
		    ie_len_wps > MAX_SIZE_IE_BUFFER) {
			printf("ERR:IE parameter size exceeds limit in %s.\n",
			       argv[2]);
			free(buf);
			return;
		}
		ie_len = ie_len_wifidirect + ie_len_wps +
			sizeof(tlvbuf_custom_ie) + (2 * sizeof(custom_ie));
		if (ie_len >= MRVDRV_SIZE_OF_CMD_BUFFER) {
			printf("ERR:Too much data in configuration file %s.\n",
			       argv[2]);
			free(buf);
			return;
		}
#ifdef DEBUG
		hexdump(buf, ie_len, ' ');
#endif
		ret = wifidirect_ie_config(&ie_index, ie_len_wifidirect,
					   ie_len_wps, buf);
		if (ret != SUCCESS) {
			printf("ERR:Could not set wifidirect parameters\n");
		}
	} else {
		ret = wifidirect_ie_config(&ie_index, 0, 0, buf);
		/* Print response */
		if (ret == SUCCESS && ie_index < 3) {
			tlv = (tlvbuf_custom_ie *)buf;
			if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
				ie_ptr = (custom_ie *)(tlv->ie_data);
				/* Goto appropriate Ie Index */
				for (i = 0; i < ie_index; i++) {
					ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
							       sizeof(custom_ie)
							       +
							       ie_ptr->
							       ie_length);
				}
				ie_len_wifidirect = ie_ptr->ie_length;
				ptr = ie_ptr->ie_buffer;
				/* Locate headers */
				printf("WIFIDIRECT settings:\n");
				if (!wifidir_use_fixed_ie_indices()) {
					while (memcmp
					       (ptr, wifidirect_oui,
						sizeof(wifidirect_oui))) {
						ie_ptr = (custom_ie *)((t_u8 *)
								       ie_ptr +
								       sizeof
								       (custom_ie)
								       +
								       ie_ptr->
								       ie_length);
						ie_len_wifidirect =
							ie_ptr->ie_length;
						ptr = ie_ptr->ie_buffer;
					}
					ptr += sizeof(wifidirect_oui);
					ie_len_wifidirect -=
						sizeof(wifidirect_oui);
				}
				while (ie_len_wifidirect >
				       WIFIDIRECT_IE_HEADER_LEN) {
					type = *ptr;
					memcpy(&len, ptr + 1, sizeof(t_u16));
					len = le16_to_cpu(len);
					switch (type) {
					case TLV_TYPE_WIFIDIRECT_DEVICE_ID:
						{
							tlvbuf_wifidirect_device_id
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_device_id
								 *)ptr;
							printf("\t Device ID - ");
							print_mac
								(wifidirect_tlv->
								 dev_mac_address);
							printf("\n");
						}
						break;

					case TLV_TYPE_WIFIDIRECT_CAPABILITY:
						{
							tlvbuf_wifidirect_capability
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_capability
								 *)ptr;
							printf("\t Device capability = %d\n", (int)wifidirect_tlv->dev_capability);
							printf("\t Group capability = %d\n", (int)wifidirect_tlv->group_capability);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_GROUPOWNER_INTENT:
						{
							tlvbuf_wifidirect_group_owner_intent
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_group_owner_intent
								 *)ptr;
							printf("\t Group owner intent = %d\n", (int)wifidirect_tlv->dev_intent);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_MANAGEABILITY:
						{
							tlvbuf_wifidirect_manageability
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_manageability
								 *)ptr;
							printf("\t Manageability = %d\n", (int)wifidirect_tlv->manageability);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_CHANNEL_LIST:
						{
							tlvbuf_wifidirect_channel_list
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_channel_list
								 *)ptr;
							chan_entry *temp_ptr;
							printf("\t Country String \"%c%c\"", wifidirect_tlv->country_string[0], wifidirect_tlv->country_string[1]);
							if (isalpha
							    (wifidirect_tlv->
							     country_string[2]))
								printf("\"%c\"",
								       wifidirect_tlv->
								       country_string
								       [2]);
							printf("\n");
							temp_ptr =
								(chan_entry *)
								wifidirect_tlv->
								wifidirect_chan_entry_list;
							len_wifidirect =
								le16_to_cpu
								(wifidirect_tlv->
								 length) -
								(sizeof
								 (tlvbuf_wifidirect_channel_list)
								 -
								 WIFIDIRECT_IE_HEADER_LEN);
							if (len_wifidirect)
								printf("\t Channel List :- \n");
							while (len_wifidirect) {
								printf("\t\t Regulatory_class = %d\n", (int)(temp_ptr->regulatory_class));
								printf("\t\t No of channels = %d\n", (int)temp_ptr->num_of_channels);
								printf("\t\t Channel list = ");
								for (i = 0;
								     i <
								     temp_ptr->
								     num_of_channels;
								     i++) {
									printf("%d ", *(temp_ptr->chan_list + i));
								}
								len_wifidirect
									-=
									sizeof
									(chan_entry)
									+
									temp_ptr->
									num_of_channels;
								temp_ptr =
									(chan_entry
									 *)((t_u8 *)temp_ptr + sizeof(chan_entry) + temp_ptr->num_of_channels);
								printf("\n");
							}
						}
						break;
					case TLV_TYPE_WIFIDIRECT_NOTICE_OF_ABSENCE:
						{
							tlvbuf_wifidirect_notice_of_absence
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_notice_of_absence
								 *)ptr;
							noa_descriptor
								*temp_ptr;
							printf("\t Notice of Absence index = %d\n", (int)wifidirect_tlv->noa_index);
							printf("\t CTWindow and opportunistic PS parameters = %d\n", (int)wifidirect_tlv->ctwindow_opp_ps);
							temp_ptr =
								(noa_descriptor
								 *)
								wifidirect_tlv->
								wifidirect_noa_descriptor_list;
							len_wifidirect =
								le16_to_cpu
								(wifidirect_tlv->
								 length) -
								(sizeof
								 (tlvbuf_wifidirect_notice_of_absence)
								 -
								 WIFIDIRECT_IE_HEADER_LEN);
							while (len_wifidirect) {
								printf("\t Count or Type = %d\n", (int)temp_ptr->count_type);
								printf("\t Duration = %dms\n", le32_to_cpu(temp_ptr->duration));
								printf("\t Interval = %dms\n", le32_to_cpu(temp_ptr->interval));
								printf("\t Start Time = %d\n", le32_to_cpu(temp_ptr->start_time));
								printf("\n");
								temp_ptr +=
									sizeof
									(noa_descriptor);
								len_wifidirect
									-=
									sizeof
									(noa_descriptor);
							}
						}
						break;
					case TLV_TYPE_WIFIDIRECT_DEVICE_INFO:
						{
							tlvbuf_wifidirect_device_info
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_device_info
								 *)ptr;
							printf("\t Device address - ");
							print_mac
								(wifidirect_tlv->
								 dev_address);
							printf("\n");
							/* display config_methods */
							printf("\t Config methods - 0x%02X\n", ntohs(wifidirect_tlv->config_methods));
							printf("\t Primary device type = %02d-%02X%02X%02X%02X-%02d\n", (int)ntohs(wifidirect_tlv->primary_category), (int)wifidirect_tlv->primary_oui[0], (int)wifidirect_tlv->primary_oui[1], (int)wifidirect_tlv->primary_oui[2], (int)wifidirect_tlv->primary_oui[3], (int)ntohs(wifidirect_tlv->primary_subcategory));
							printf("\t Secondary Device Count = %d\n", (int)wifidirect_tlv->secondary_dev_count);
							array_ptr =
								wifidirect_tlv->
								secondary_dev_info;
							for (i = 0;
							     i <
							     wifidirect_tlv->
							     secondary_dev_count;
							     i++) {
								memcpy(&temp,
								       array_ptr,
								       sizeof
								       (t_u16));
								printf("\t\t Secondary device type = %02d-", ntohs(temp));
								array_ptr +=
									sizeof
									(temp);
								printf("%02X%02X%02X%02X", array_ptr[0], array_ptr[1], array_ptr[2], array_ptr[3]);
								array_ptr += 4;
								memcpy(&temp,
								       array_ptr,
								       sizeof
								       (t_u16));
								printf("-%02d\n", ntohs(temp));
								array_ptr +=
									sizeof
									(temp);
							}
							array_ptr =
								wifidirect_tlv->
								device_name +
								wifidirect_tlv->
								secondary_dev_count
								*
								WPS_DEVICE_TYPE_LEN;
							dev_ptr =
								(((t_u8
								   *)
								  (&wifidirect_tlv->
								   device_name_len))
								 +
								 wifidirect_tlv->
								 secondary_dev_count
								 *
								 WPS_DEVICE_TYPE_LEN);
							if (*(t_u16 *)dev_ptr)
								printf("\t Device Name =  ");
							memcpy(&temp, dev_ptr,
							       sizeof(t_u16));
							for (i = 0;
							     i < ntohs(temp);
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case TLV_TYPE_WIFIDIRECT_GROUP_INFO:
						{
							tlvbuf_wifidirect_group_info
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_group_info
								 *)ptr;
							t_u8 no_of_wifidirect_clients = 0, wifidirect_client_dev_length = 0;
							wifidirect_client_dev_info
								*temp_ptr;
							temp_ptr =
								(wifidirect_client_dev_info
								 *)
								wifidirect_tlv->
								wifidirect_client_dev_list;
							if (temp_ptr == NULL)
								break;
							wifidirect_client_dev_length
								=
								temp_ptr->
								dev_length;
							temp = le16_to_cpu
								(wifidirect_tlv->
								 length) -
								wifidirect_client_dev_length;
							while (temp && temp_ptr) {

								printf("\t Group WifiDirect Client Device address - ");
								print_mac
									(temp_ptr->
									 wifidirect_dev_address);
								printf("\n");
								printf("\t Group WifiDirect Client Interface address - ");
								print_mac
									(temp_ptr->
									 wifidirect_intf_address);
								printf("\n");
								printf("\t Group WifiDirect Client Device capability = %d\n", (int)temp_ptr->wifidirect_dev_capability);
								printf("\t Group WifiDirect Client Config methods - 0x%02X\n", ntohs(temp_ptr->config_methods));
								printf("\t Group WifiDirect Client Primay device type = %02d-%02X%02X%02X%02X-%02d\n", (int)ntohs(temp_ptr->primary_category), (int)temp_ptr->primary_oui[0], (int)temp_ptr->primary_oui[1], (int)temp_ptr->primary_oui[2], (int)temp_ptr->primary_oui[3], (int)ntohs(temp_ptr->primary_subcategory));
								printf("\t Group WifiDirect Client Secondary Device Count = %d\n", (int)temp_ptr->wifidirect_secondary_dev_count);
								array_ptr =
									temp_ptr->
									wifidirect_secondary_dev_info;
								for (i = 0;
								     i <
								     temp_ptr->
								     wifidirect_secondary_dev_count;
								     i++) {
									memcpy(&temp, array_ptr, sizeof(t_u16));
									printf("\t Group WifiDirect Client Secondary device type = %02d-", ntohs(temp));
									array_ptr
										+=
										sizeof
										(temp);
									printf("%02X%02X%02X%02X", array_ptr[0], array_ptr[1], array_ptr[2], array_ptr[3]);
									array_ptr
										+=
										4;
									memcpy(&temp, array_ptr, sizeof(t_u16));
									printf("-%02d\n", ntohs(temp));
									array_ptr
										+=
										sizeof
										(temp);
								}
								/* display device name */
								array_ptr =
									temp_ptr->
									wifidirect_device_name
									+
									temp_ptr->
									wifidirect_secondary_dev_count
									*
									WPS_DEVICE_TYPE_LEN;
								printf("\t Group WifiDirect Device Name =  ");
								memcpy(&temp,
								       (((t_u8
									  *)
									 (&temp_ptr->
									  wifidirect_device_name_len))
									+
									temp_ptr->
									wifidirect_secondary_dev_count
									*
									WPS_DEVICE_TYPE_LEN),
								       sizeof
								       (t_u16));
								temp = ntohs
									(temp);
								for (i = 0;
								     i < temp;
								     i++)
									printf("%c", *array_ptr++);
								printf("\n");
								temp_ptr +=
									wifidirect_client_dev_length;
								temp -= wifidirect_client_dev_length;
								no_of_wifidirect_clients++;
								if (temp_ptr)
									wifidirect_client_dev_length
										=
										temp_ptr->
										dev_length;
							}
							printf("\n");
							printf("\t Group WifiDirect Client Devices count = %d\n", no_of_wifidirect_clients);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_GROUP_ID:
						{
							tlvbuf_wifidirect_group_id
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_group_id
								 *)ptr;
							printf("\t Group address - ");
							print_mac
								(wifidirect_tlv->
								 group_address);
							printf("\n");
							array_ptr =
								wifidirect_tlv->
								group_ssid;
							printf("\t Group ssid =  ");
							for (i = 0;
							     (unsigned int)i <
							     le16_to_cpu
							     (wifidirect_tlv->
							      length)
							     -
							     (sizeof
							      (tlvbuf_wifidirect_group_id)
							      -
							      WIFIDIRECT_IE_HEADER_LEN);
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case TLV_TYPE_WIFIDIRECT_GROUP_BSS_ID:
						{
							tlvbuf_wifidirect_group_bss_id
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_group_bss_id
								 *)ptr;
							printf("\t Group BSS Id - ");
							print_mac
								(wifidirect_tlv->
								 group_bssid);
							printf("\n");
						}
						break;
					case TLV_TYPE_WIFIDIRECT_INTERFACE:
						{
							tlvbuf_wifidirect_interface
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_interface
								 *)ptr;
							printf("\t Interface Id - ");
							print_mac
								(wifidirect_tlv->
								 interface_id);
							printf("\t Interface count = %d", (int)wifidirect_tlv->interface_count);
							for (i = 0;
							     i <
							     wifidirect_tlv->
							     interface_count;
							     i++) {
								printf("\n\t Interface address [%d]", i + 1);
								print_mac
									(&wifidirect_tlv->
									 interface_idlist
									 [i *
									  ETH_ALEN]);
							}
							printf("\n");
						}
						break;
					case TLV_TYPE_WIFIDIRECT_CHANNEL:
						{
							tlvbuf_wifidirect_channel
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_channel
								 *)ptr;
							printf("\t Listen Channel Country String \"%c%c\"", wifidirect_tlv->country_string[0], wifidirect_tlv->country_string[1]);
							if (isalpha
							    (wifidirect_tlv->
							     country_string[2]))
								printf("\"%c\"",
								       wifidirect_tlv->
								       country_string
								       [2]);
							printf("\n");
							printf("\t Listern Channel regulatory class = %d\n", (int)wifidirect_tlv->regulatory_class);
							printf("\t Listen Channel number = %d\n", (int)wifidirect_tlv->channel_number);
						}
						break;

					case TLV_TYPE_WIFIDIRECT_OPCHANNEL:
						{
							tlvbuf_wifidirect_channel
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_channel
								 *)ptr;
							printf("\t Operating Channel Country String %c%c", wifidirect_tlv->country_string[0], wifidirect_tlv->country_string[1]);
							if (isalpha
							    (wifidirect_tlv->
							     country_string[2]))
								printf("%c",
								       wifidirect_tlv->
								       country_string
								       [2]);
							printf("\n");
							printf("\t Operating Channel regulatory class = %d\n", (int)wifidirect_tlv->regulatory_class);
							printf("\t Operating Channel number = %d\n", (int)wifidirect_tlv->channel_number);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_INVITATION_FLAG:
						{
							tlvbuf_wifidirect_invitation_flag
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_invitation_flag
								 *)ptr;
							printf("\t Invitation flag = %d\n", (int)wifidirect_tlv->invitation_flag & INVITATION_FLAG_MASK);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_CONFIG_TIMEOUT:
						{
							tlvbuf_wifidirect_config_timeout
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_config_timeout
								 *)ptr;
							printf("\t GO configuration timeout = %d msec\n", (int)wifidirect_tlv->group_config_timeout * 10);
							printf("\t Client configuration timeout = %d msec\n", (int)wifidirect_tlv->device_config_timeout * 10);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_EXTENDED_LISTEN_TIME:
						{
							tlvbuf_wifidirect_ext_listen_time
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_ext_listen_time
								 *)ptr;
							printf("\t Availability Period = %d msec\n", (int)wifidirect_tlv->availability_period);
							printf("\t Availability Interval = %d msec\n", (int)wifidirect_tlv->availability_interval);
						}
						break;
					case TLV_TYPE_WIFIDIRECT_INTENDED_ADDRESS:
						{
							tlvbuf_wifidirect_intended_addr
								*wifidirect_tlv
								=
								(tlvbuf_wifidirect_intended_addr
								 *)ptr;
							printf("\t Intended Interface Address - ");
							print_mac
								(wifidirect_tlv->
								 group_address);
							printf("\n");
						}
						break;
					default:
						printf("unknown ie=0x%x, len=%d\n", type, len);
						break;
					}
					ptr += len + WIFIDIRECT_IE_HEADER_LEN;
					ie_len_wifidirect -=
						len + WIFIDIRECT_IE_HEADER_LEN;
				}

				/* Goto next index, Locate headers */
				printf("WPS settings:\n");
				ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
						       sizeof(custom_ie) +
						       ie_ptr->ie_length);
				ptr = ie_ptr->ie_buffer;
				ie_len_wps = ie_ptr->ie_length;
				if (!wifidir_use_fixed_ie_indices()) {
					while (memcmp
					       (ptr, wps_oui,
						sizeof(wps_oui))) {
						ie_ptr = (custom_ie *)((t_u8 *)
								       ie_ptr +
								       sizeof
								       (custom_ie)
								       +
								       ie_ptr->
								       ie_length);
						ie_len_wps = ie_ptr->ie_length;
						ptr = ie_ptr->ie_buffer;
					}
					ptr += sizeof(wps_oui);
					ie_len_wifidirect -= sizeof(wps_oui);
				}
				while (ie_len_wps > sizeof(tlvbuf_wps_ie)) {
					memcpy(&wps_type, ptr, sizeof(t_u16));
					memcpy(&wps_len, ptr + 2,
					       sizeof(t_u16));
					endian_convert_tlv_wps_header_in
						(wps_type, wps_len);
					switch (wps_type) {
					case SC_Version:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							printf("\t WPS Version = 0x%2x\n", *(wps_tlv->data));
						}
						break;
					case SC_Simple_Config_State:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							printf("\t WPS setupstate = 0x%x\n", *(wps_tlv->data));
						}
						break;
					case SC_Request_Type:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							printf("\t WPS RequestType = 0x%x\n", *(wps_tlv->data));
						}
						break;
					case SC_Response_Type:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							printf("\t WPS ResponseType = 0x%x\n", *(wps_tlv->data));
						}
						break;
					case SC_Config_Methods:
						{
							t_u16 wps_config_methods
								= 0;
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							memcpy(&wps_config_methods, wps_tlv->data, sizeof(t_u16));
							wps_config_methods =
								ntohs
								(wps_config_methods);
							printf("\t WPS SpecConfigMethods = 0x%x\n", wps_config_methods);
						}
						break;
					case SC_UUID_E:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS UUID = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%2X ",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Primary_Device_Type:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Primary Device Type = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%02X ",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_RF_Band:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							printf("\t WPS RF Band = 0x%x\n", *(wps_tlv->data));
						}
						break;
					case SC_Association_State:
						{
							t_u16 wps_association_state = 0;
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							memcpy(&wps_association_state, wps_tlv->data, sizeof(t_u16));
							printf("\t WPS Association State = 0x%x\n", wps_association_state);
						}
						break;
					case SC_Configuration_Error:
						{
							t_u16 wps_configuration_error = 0;
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							memcpy(&wps_configuration_error, wps_tlv->data, sizeof(t_u16));
							wps_configuration_error
								=
								ntohs
								(wps_configuration_error);
							printf("\t WPS Configuration Error = 0x%x\n", wps_configuration_error);
						}
						break;
					case SC_Device_Password_ID:
						{
							t_u16 wps_device_password_id = 0;
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							memcpy(&wps_device_password_id, wps_tlv->data, sizeof(t_u16));
							wps_device_password_id =
								ntohs
								(wps_device_password_id);
							printf("\t WPS Device Password ID = 0x%x\n", wps_device_password_id);
						}
						break;
					case SC_Device_Name:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Device Name = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Manufacturer:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Manufacturer = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Model_Name:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Model Name = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Model_Number:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Model Number = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Serial_Number:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Serial Number = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("%c",
								       *array_ptr++);
							printf("\n");
						}
						break;
					case SC_Vendor_Extension:
						{
							tlvbuf_wps_ie *wps_tlv =
								(tlvbuf_wps_ie
								 *)ptr;
							array_ptr =
								wps_tlv->data;
							printf("\t WPS Vendor Extension = ");
							for (i = 0; i < wps_len;
							     i++)
								printf("0x%02x ", *array_ptr++);
							printf("\n");
						}
						break;
					default:
						printf("unknown ie=0x%x, len=%d\n", wps_type, wps_len);
						break;
					}
					ptr += wps_len + sizeof(tlvbuf_wps_ie);
					ie_len_wps -=
						wps_len + sizeof(tlvbuf_wps_ie);
				}
			}
		} else {
			printf("ERR:Could not retrieve wifidirect parameters\n");
		}
	}

	free(buf);
	return;
}

/**
 *  @brief Show usage information for the wifidirect_params_config command
 *
 *  $return         N/A
 */
static void
print_wifidirect_params_config_usage(void)
{
	printf("\nUsage : wifidirect_params_config <CONFIG_FILE>\n");
	printf("\nIssue set or get request using parameters in the CONFIG_FILE.\n");
	return;
}

/**
 *  @brief Creates a wifidirect_params_config request and sends to the driver
 *
 *  Usage: "Usage : wifidirect_params_config <CONFIG_FILE>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirectcmd_params_config(int argc, char *argv[])
{
	wifidirect_params_config *param_buf = NULL;
	tlvbuf_wps_ie *new_tlv = NULL;
	char *line = NULL, wifidirect_mac[20];
	FILE *config_file = NULL;
	int i, opt, li = 0, arg_num = 0, ret = 0;
	char *args[30], *pos = NULL;
	t_u8 dev_address[ETH_ALEN], enable_scan;
	t_u8 *buffer = NULL, WPS_primarydevicetype[WPS_DEVICE_TYPE_MAX_LEN];
	t_u16 device_state = 0, tlv_len = 0, max_disc_int = 0, min_disc_int =
		0, cmd_len = 0;

	memset(wifidirect_mac, 0, sizeof(wifidirect_mac));
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_params_config_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_params_config_usage();
		return;
	}

	cmd_len = sizeof(wifidirect_params_config);
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}
	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);
	param_buf = (wifidirect_params_config *)buffer;
	param_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	param_buf->seq_num = 0;
	param_buf->result = 0;

	if (argc >= 3) {
		/* Check if file exists */
		config_file = fopen(argv[2], "r");
		if (config_file == NULL) {
			printf("\nERR:Config file can not open.\n");
			goto done;
		}
		line = (char *)malloc(MAX_CONFIG_LINE);
		if (!line) {
			printf("ERR:Cannot allocate memory for line\n");
			goto done;
		}
		memset(line, 0, MAX_CONFIG_LINE);
		param_buf->action = cpu_to_le16(ACTION_SET);

		/* Parse file and process */
		while (config_get_line
		       (line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
			arg_num = parse_line(line, args, 30);

			if (strcmp(args[0], "EnableScan") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_ENABLE_SCAN, arg_num - 1,
				     args + 1)
				    != SUCCESS) {
					goto done;

				}
				enable_scan = (t_u8)atoi(args[1]);
				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_wps_ie) + sizeof(t_u8);
				new_tlv = (tlvbuf_wps_ie *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				new_tlv->tag =
					MRVL_WIFIDIRECT_SCAN_ENABLE_TLV_ID;
				new_tlv->length = tlv_len - 2 * sizeof(t_u16);
				memcpy(new_tlv->data, &enable_scan,
				       sizeof(t_u8));
				endian_convert_tlv_header_out(new_tlv);

			} else if (strcmp(args[0], "ScanPeerDeviceId") == 0) {
				strncpy(wifidirect_mac, args[1],
					sizeof(wifidirect_mac) - 1);
				if ((ret =
				     mac2raw(wifidirect_mac,
					     dev_address)) != SUCCESS) {
					printf("ERR: %s Address \n",
					       ret ==
					       FAILURE ? "Invalid MAC" : ret ==
					       WIFIDIRECT_RET_MAC_BROADCAST ?
					       "Broadcast" : "Multicast");
					goto done;
				}
				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_wps_ie) + ETH_ALEN;
				new_tlv = (tlvbuf_wps_ie *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				new_tlv->tag =
					MRVL_WIFIDIRECT_PEER_DEVICE_TLV_ID;
				new_tlv->length = tlv_len - 2 * sizeof(t_u16);
				memcpy(new_tlv->data, dev_address, ETH_ALEN);
				endian_convert_tlv_header_out(new_tlv);

			} else if (strcmp(args[0], "MinDiscoveryInterval") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_MINDISCOVERYINT, arg_num - 1,
				     args + 1) != SUCCESS) {
					goto done;
				}
				min_disc_int = cpu_to_le16(atoi(args[1]));
			} else if (strcmp(args[0], "MaxDiscoveryInterval") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_MAXDISCOVERYINT, arg_num - 1,
				     args + 1) != SUCCESS) {
					goto done;
				}
				max_disc_int = cpu_to_le16(atoi(args[1]));

				/* Append a new TLV */
				tlv_len =
					sizeof(tlvbuf_wps_ie) +
					2 * sizeof(t_u16);
				new_tlv = (tlvbuf_wps_ie *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				new_tlv->tag =
					MRVL_WIFIDIRECT_DISC_PERIOD_TLV_ID;
				new_tlv->length = tlv_len - 2 * sizeof(t_u16);
				memcpy(&new_tlv->data, &min_disc_int,
				       sizeof(t_u16));
				memcpy((((t_u8 *)&new_tlv->data) +
					sizeof(t_u16)), &max_disc_int,
				       sizeof(t_u16));
				endian_convert_tlv_header_out(new_tlv);

			} else if (strcmp(args[0], "ScanRequestDeviceType") ==
				   0) {
				if (is_input_valid
				    (WIFIDIRECT_WPSPRIMARYDEVICETYPE,
				     arg_num - 1, args + 1)
				    != SUCCESS) {
					goto done;
				}
				for (i = 0; i < WPS_DEVICE_TYPE_MAX_LEN; i++)
					WPS_primarydevicetype[i] =
						(t_u8)A2HEXDECIMAL(args[i + 1]);

				/* Append a new TLV */
				tlv_len =
					sizeof(tlvbuf_wps_ie) +
					WPS_DEVICE_TYPE_MAX_LEN;
				new_tlv = (tlvbuf_wps_ie *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				new_tlv->tag =
					MRVL_WIFIDIRECT_SCAN_REQ_DEVICE_TLV_ID;
				new_tlv->length = tlv_len - 2 * sizeof(t_u16);
				memcpy(&new_tlv->data, WPS_primarydevicetype,
				       WPS_DEVICE_TYPE_MAX_LEN);
				endian_convert_tlv_header_out(new_tlv);

			} else if (strcmp(args[0], "DeviceState") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_DEVICE_STATE, arg_num - 1,
				     args + 1)
				    != SUCCESS) {
					goto done;
				}
				device_state =
					cpu_to_le16((t_u16)atoi(args[1]));

				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_wps_ie) + sizeof(t_u16);
				new_tlv = (tlvbuf_wps_ie *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				new_tlv->tag =
					MRVL_WIFIDIRECT_DEVICE_STATE_TLV_ID;
				new_tlv->length = tlv_len - 2 * sizeof(t_u16);
				memcpy(new_tlv->data, &device_state,
				       sizeof(t_u16));
				endian_convert_tlv_header_out(new_tlv);
			}
		}
	}

	param_buf->size = cmd_len;
	/* Send collective command */
	wifidirect_ioctl((t_u8 *)buffer, &cmd_len, MRVDRV_SIZE_OF_CMD_BUFFER);

	/* Process response */
	if (argc < 3) {
		/* Verify response */
		if (param_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (param_buf->result == CMD_SUCCESS) {
			param_buf = (wifidirect_params_config *)buffer;
			new_tlv = (tlvbuf_wps_ie *)param_buf->wifidirect_tlv;
			tlv_len =
				param_buf->size -
				(sizeof(wifidirect_params_config) -
				 BUF_HEADER_SIZE);
			if (tlv_len > sizeof(tlvbuf_wps_ie))
				printf("WifiDirect Parameter settings: \n");
			else
				printf("WifiDirect Parameter settings are not received.\n");
			while (tlv_len > sizeof(tlvbuf_wps_ie)) {
				endian_convert_tlv_header_in(new_tlv);
				switch (new_tlv->tag) {
				case MRVL_WIFIDIRECT_SCAN_ENABLE_TLV_ID:
					{
						enable_scan =
							*((t_u8 *)&new_tlv->
							  data);
						printf("\t Scan %s\n",
						       (enable_scan ==
							1) ? "enabled." :
						       "disabled.");
					}
					break;
				case MRVL_WIFIDIRECT_PEER_DEVICE_TLV_ID:
					{
						memcpy(dev_address,
						       new_tlv->data, ETH_ALEN);
						printf("\t Peer Device ID : ");
						print_mac(dev_address);
						printf("\n");
					}
					break;
				case MRVL_WIFIDIRECT_DISC_PERIOD_TLV_ID:
					{
						t_u16 temp;
						memcpy(&temp, &new_tlv->data,
						       sizeof(t_u16));
						printf("\t Minimum Discovery Interval : %d\n", le16_to_cpu(temp));
						memcpy(&temp,
						       ((t_u8 *)&new_tlv->
							data) + sizeof(t_u16),
						       sizeof(t_u16));
						printf("\t Maximum Discovery Interval : %d\n", le16_to_cpu(temp));
					}
					break;
				case MRVL_WIFIDIRECT_SCAN_REQ_DEVICE_TLV_ID:
					{
						memcpy(WPS_primarydevicetype,
						       &new_tlv->data,
						       WPS_DEVICE_TYPE_MAX_LEN);
						printf("\tType: %02x:%02x\n",
						       (unsigned int)
						       WPS_primarydevicetype[0],
						       (unsigned int)
						       WPS_primarydevicetype
						       [1]);
						printf("\tOUI: %02x:%02x:%02x:%02x\n", (unsigned int)WPS_primarydevicetype[2], (unsigned int)WPS_primarydevicetype[3], (unsigned int)WPS_primarydevicetype[4], (unsigned int)WPS_primarydevicetype[5]);
						printf("\tSubType: %02x:%02x\n",
						       (unsigned int)
						       WPS_primarydevicetype[6],
						       (unsigned int)
						       WPS_primarydevicetype
						       [7]);
					}
					break;
				case MRVL_WIFIDIRECT_DEVICE_STATE_TLV_ID:
					{
						memcpy(&device_state,
						       &new_tlv->data,
						       sizeof(t_u16));
						printf("\t Device State %d\n",
						       le16_to_cpu
						       (device_state));
					}
					break;
				default:
					printf("Unknown ie=0x%x, len=%d\n",
					       new_tlv->tag, new_tlv->length);
					break;
				}
				tlv_len -=
					new_tlv->length + sizeof(tlvbuf_wps_ie);
				new_tlv =
					(tlvbuf_wps_ie *)(((t_u8 *)new_tlv) +
							  new_tlv->length +
							  sizeof
							  (tlvbuf_wps_ie));
			}
		} else {
			printf("ERR:Could not retrieve wifidirect parameters.\n");
		}
	}

done:
	if (config_file)
		fclose(config_file);
	if (line)
		free(line);
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect_action_frame command
 *
 *  $return         N/A
 */
static void
print_wifidirect_action_frame_usage(void)
{
	printf("\nUsage : wifidirect_action_frame <CONFIG_FILE> |"
	       " [<PeerAddr> <Category> <OUISubtype> <DialogToken>]\n");
	printf("\nAction frame is sent using parameters in the CONFIG_FILE "
	       "or parameters specified on command line.\n");
	return;
}

/**
 *  @brief Creates a wifidirect_action_frame request and sends to the driver
 *
 *  Usage: "Usage : wifidirect_action_frame <CONFIG_FILE> |
 *                  [<PeerAddr> <Category> <OUISubtype> <DialogToken>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirectcmd_action_frame(int argc, char *argv[])
{
	wifidirect_action_frame *action_buf = NULL;
	char *line = NULL, wifidirect_mac[20];
	FILE *config_file = NULL;
	int i, opt, li = 0, arg_num = 0, ret = 0, cmd_found = 0;
	char *args[30], *pos = NULL;
	t_u8 dev_address[ETH_ALEN];
	t_u8 *buffer = NULL;
	t_u16 ie_len = 0, cmd_len = 0;
	memset(wifidirect_mac, 0, sizeof(wifidirect_mac));
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_action_frame_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc != 3) && (argc != 6)) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_action_frame_usage();
		return;
	}

	cmd_len = sizeof(wifidirect_action_frame);
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}
	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);
	action_buf = (wifidirect_action_frame *)buffer;
	action_buf->cmd_code = HostCmd_CMD_802_11_ACTION_FRAME;
	action_buf->seq_num = 0;
	action_buf->result = 0;

	/* Check if file exists */
	config_file = fopen(argv[2], "r");
	if (config_file && argc == 3) {
		line = (char *)malloc(MAX_CONFIG_LINE);
		if (!line) {
			printf("ERR:Cannot allocate memory for line\n");
			goto done;
		}

		/* Parse file and process */
		while (config_get_line
		       (line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
			arg_num = parse_line(line, args, 30);

			if (strcmp(args[0], argv[1]) == 0) {
				cmd_found = 1;
			}
			if (cmd_found == 0)
				continue;

			if (strcmp(args[0], "}") == 0) {
				cmd_found = 0;
			} else if (strcmp(args[0], "Category") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_CATEGORY, arg_num - 1, args + 1)
				    != SUCCESS) {
					goto done;

				}
				action_buf->category = (t_u8)atoi(args[1]);

			} else if (strcmp(args[0], "PeerAddr") == 0) {
				strncpy(wifidirect_mac, args[1],
					sizeof(wifidirect_mac) - 1);
				if ((ret =
				     mac2raw(wifidirect_mac,
					     dev_address)) != SUCCESS) {
					printf("ERR: %s Address \n",
					       ret ==
					       FAILURE ? "Invalid MAC" : ret ==
					       WIFIDIRECT_RET_MAC_BROADCAST ?
					       "Broadcast" : "Multicast");
					goto done;
				}
				memcpy(action_buf->peer_mac_addr, dev_address,
				       ETH_ALEN);
			} else if (strcmp(args[0], "Action") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_ACTION, arg_num - 1, args + 1)
				    != SUCCESS) {
					goto done;
				}
				action_buf->action =
					(t_u8)A2HEXDECIMAL(args[1]);

			} else if (strcmp(args[0], "DialogToken") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_DIALOGTOKEN, arg_num - 1,
				     args + 1)
				    != SUCCESS) {
					goto done;
				}
				action_buf->dialog_taken = (t_u8)atoi(args[1]);

			} else if (strcmp(args[0], "OUI") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_OUI, arg_num - 1, args + 1)
				    != SUCCESS) {
					goto done;
				}
				for (i = 0; i < MIN(arg_num - 1, MAX_OUI_LEN);
				     i++)
					action_buf->oui[i] =
						A2HEXDECIMAL(args[i + 1]);
			} else if (strcmp(args[0], "OUIType") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_OUITYPE, arg_num - 1, args + 1)
				    != SUCCESS) {
					goto done;
				}
				action_buf->oui_type = (t_u8)atoi(args[1]);
			} else if (strcmp(args[0], "OUISubType") == 0) {
				if (is_input_valid
				    (WIFIDIRECT_OUISUBTYPE, arg_num - 1,
				     args + 1)
				    != SUCCESS) {
					goto done;
				}
				action_buf->oui_sub_type = (t_u8)atoi(args[1]);
			}
		}
		fclose(config_file);
		config_file = NULL;

		/* Now, read IE parameters from file */
		wifidirect_file_params_config(argv[2], argv[1],
					      action_buf->ie_list, &ie_len,
					      NULL);
		if ((cmd_len + ie_len) > MRVDRV_SIZE_OF_CMD_BUFFER) {
			printf("ERR:Too much data in configuration file %s.\n",
			       argv[2]);
			goto done;
		}
		cmd_len += ie_len;
	} else {
		if (argc == 3) {
			printf("ERR:Can not open config file.\n");
			goto done;
		}
		strncpy(wifidirect_mac, argv[2], sizeof(wifidirect_mac) - 1);
		if ((ret = mac2raw(wifidirect_mac, dev_address)) != SUCCESS) {
			printf("ERR: %s Address \n",
			       ret == FAILURE ? "Invalid MAC" : ret ==
			       WIFIDIRECT_RET_MAC_BROADCAST ? "Broadcast" :
			       "Multicast");
			goto done;
		}
		memcpy(action_buf->peer_mac_addr, dev_address, ETH_ALEN);

		if (is_input_valid(WIFIDIRECT_CATEGORY, 1, &argv[3])
		    != SUCCESS) {
			goto done;
		}
		action_buf->category = (t_u8)atoi(argv[3]);

		if (is_input_valid(WIFIDIRECT_OUISUBTYPE, 1, &argv[4])
		    != SUCCESS) {
			goto done;
		}
		action_buf->oui_sub_type = (t_u8)atoi(argv[4]);

		if (is_input_valid(WIFIDIRECT_DIALOGTOKEN, 1, &argv[5])
		    != SUCCESS) {
			goto done;
		}
		action_buf->dialog_taken = (t_u8)atoi(argv[5]);

		action_buf->action = 0;
		action_buf->oui[0] = 0x50;
		action_buf->oui[1] = 0x6F;
		action_buf->oui[2] = 0x9A;
		action_buf->oui_type = OUI_TYPE_WFA_WIFIDIRECT;
	}
	action_buf->size = cmd_len;
#ifdef DEBUG
	/* Debug print */
	hexdump(buffer, cmd_len, ' ');
#endif
	/* Send collective command */
	wifidirect_ioctl((t_u8 *)buffer, &cmd_len, MRVDRV_SIZE_OF_CMD_BUFFER);

done:
	if (config_file)
		fclose(config_file);
	if (line)
		free(line);
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect status command
 *
 *  $return         N/A
 */
static void
print_wifidirect_status_usage(void)
{
	printf("\nUsage : wifidirect_mode [STATUS]");
	printf("\nOptions: STATUS :     0 - stop wifidirect status");
	printf("\n                      1 - start wifidirect status");
	printf("\n                      2 - start wifidirect group owner mode");
	printf("\n                      3 - start wifidirect client mode");
	printf("\n                      4 - start wifidirect find phase");
	printf("\n                      5 - stop  wifidirect find phase");
	printf("\n                  empty - get current wifidirect status\n");
	return;
}

/**
 *  @brief Creates wifidirect mode start or stop request and send to driver
 *
 *   Usage: "Usage : wifidirect_mode [STATUS]"
 *
 *   Options: STATUS :     0 - start wifidirect status
 *                         1 - stop  wifidirect status
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         N/A
 */
static void
wifidirectcmd_status(int argc, char *argv[])
{
	int opt, ret;
	t_u16 data = 0;
	t_u16 cmd_len = 0;
	t_u8 *buffer = NULL;
	wifidirect_mode_config *cmd_buf = NULL;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_status_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;
	/* Check arguments */
	if (argc < 2) {
		printf("ERR:wrong arguments.\n");
		print_wifidirect_status_usage();
		return;
	}
	if (argc == 3) {
		if ((ISDIGIT(argv[2]) == 0) || (atoi(argv[2]) < 0) ||
		    (atoi(argv[2]) >= 0xFF)) {
			printf("ERR:Illegal wifidirect mode %s. Must be in range from '0' to '5'.\n", argv[2]);
			print_wifidirect_status_usage();
			return;
		}
		data = (t_u16)atoi(argv[2]);
	}

	/* send hostcmd now */
	cmd_len = sizeof(wifidirect_mode_config);
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		return;
	}

	cmd_buf = (wifidirect_mode_config *)buffer;
	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_MODE_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		cmd_buf->mode = cpu_to_le16(data);
	}
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	if (ret != SUCCESS) {
		printf("Error executing wifidirect mode command\n");
		free(buffer);
		return;
	}

	data = le16_to_cpu(cmd_buf->mode);
	switch (data) {
	case 0:
		printf("Wifidirect Device Mode = Not Configured\n");
		break;
	case 1:
		printf("Wifidirect Device Mode = Device\n");
		break;
	case 2:
		printf("Wifidirect Device Mode = Wifidirect Group Owner (GO)\n");
		break;
	case 3:
		printf("Wifidirect Device Mode = Wifidirect Group Client (GC)\n");
		break;
	default:
		printf("Wifidirect Device Mode = Not specified\n");
		break;
	}
	free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect discovery period command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_discovery_period_usage(void)
{
	printf("Usage : wifidirect_cfg_discovery_period [<MinDiscPeriod> <MaxDiscPeriod>]\n");
	printf("For 'Set' both MinDiscPeriod and MaxDiscPeriod should be specified.\n");
	printf("If no parameter is given, 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect discovery period
 *
 *  Usage: "Usage : wifidirect_cfg_discovery_period [<MinDiscPeriod> <MaxDiscPeriod>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_discovery_period(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_discovery_period *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_discovery_period_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc != 2) && (argc != 4)) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_cfg_discovery_period_usage();
		return;
	} else if ((argc > 2) &&
		   ((is_input_valid(WIFIDIRECT_MINDISCOVERYINT, 1, &argv[2]) !=
		     SUCCESS) ||
		    (is_input_valid(WIFIDIRECT_MAXDISCOVERYINT, 1, &argv[3]) !=
		     SUCCESS))) {
		print_wifidirect_cfg_discovery_period_usage();
		return;
	}

	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_discovery_period);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_discovery_period *)(buffer +
						     sizeof
						     (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_DISC_PERIOD_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_discovery_period) -
		MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->min_disc_interval = cpu_to_le16((t_u16)atoi(argv[2]));
		tlv->max_disc_interval = cpu_to_le16((t_u16)atoi(argv[3]));
	}
	endian_convert_tlv_header_out(tlv);

#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif

	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);

#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif

	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Min Discovery period = %d\n",
				       le16_to_cpu(tlv->min_disc_interval));
				printf("Max discovery period = %d\n",
				       le16_to_cpu(tlv->max_disc_interval));
			} else {
				printf("Discovery period setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get min and max discovery period!\n");
			} else {
				printf("ERR:Couldn't set min and max discovery period!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect Intent command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_intent_usage(void)
{
	printf("Usage : wifidirect_cfg_intent [IntentValue]\n");
	printf("If IntentValue parameter is provided, 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect Intent
 *
 *  Usage: "Usage : wifidirect_cfg_intent [IntentValue]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_intent(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_intent *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_intent_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_intent_usage();
		return;
	} else if ((argc > 2) &&
		   (is_input_valid(WIFIDIRECT_INTENT, argc - 2, argv + 2) !=
		    SUCCESS)) {
		print_wifidirect_cfg_intent_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_intent);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_intent *)(buffer +
					   sizeof(wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_INTENT_TLV_ID;
	tlv->length = sizeof(tlvbuf_wifidirect_intent) - MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->intent = (t_u8)atoi(argv[2]);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Intent Value = %d\n", (int)tlv->intent);
			} else {
				printf("Intent value setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Intent value!\n");
			} else {
				printf("ERR:Couldn't set Intent value!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect Listen Channel command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_listen_channel_usage(void)
{
	printf("Usage : wifidirect_cfg_listen_channel [ListenChannel]\n");
	printf("If ListenChannel parameter is provided, 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect ListenChannel
 *
 *  Usage: "Usage : wifidirect_cfg_listen_channel [ListenChannel]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_listen_channel(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_listen_channel *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_listen_channel_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_listen_channel_usage();
		return;
	} else if ((argc > 2) &&
		   (is_input_valid(CHANNEL, argc - 2, argv + 2) != SUCCESS)) {
		print_wifidirect_cfg_listen_channel_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_listen_channel);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_listen_channel *)(buffer +
						   sizeof
						   (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_LISTEN_CHANNEL_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_listen_channel) - MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->listen_channel = (t_u8)atoi(argv[2]);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Listen Channel = %d\n",
				       (int)tlv->listen_channel);
			} else {
				printf("Listen Channel setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Listen Channel!\n");
			} else {
				printf("ERR:Couldn't set Listen Channel!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect Operating Channel command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_op_channel_usage(void)
{
	printf("Usage : wifidirect_cfg_op_channel [OperatingChannel]\n");
	printf("If OperatingChannel parameter is provided, 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect OperatingChannel
 *
 *  Usage: "Usage : wifidirect_cfg_op_channel [OperatingChannel]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_op_channel(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_operating_channel *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_op_channel_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_op_channel_usage();
		return;
	} else if ((argc > 2) &&
		   (is_input_valid(CHANNEL, argc - 2, argv + 2) != SUCCESS)) {
		print_wifidirect_cfg_op_channel_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_operating_channel);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_operating_channel *)(buffer +
						      sizeof
						      (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_OPERATING_CHANNEL_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_operating_channel) -
		MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->operating_channel = (t_u8)atoi(argv[2]);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Operating Channel = %d\n",
				       (int)tlv->operating_channel);
			} else {
				printf("Operating Channel setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Operating Channel!\n");
			} else {
				printf("ERR:Couldn't set Operating Channel!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect Invitation List command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_invitation_list_usage(void)
{
	printf("\nUsage : wifidirect_cfg_invitation_list [mac_addr]");
	printf("\nIf mac_addr parameter is provided, 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect InvitationList
 *
 *  Usage: "Usage : wifidirect_cfg_invitation_list [mac_addr]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_invitation_list(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_invitation_list *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0, ctr = 1;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_invitation_list_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 2) || (argc > 3)) {
		printf("ERR:Wrong number of arguments!\n");
		print_wifidirect_cfg_invitation_list_usage();
		return;
	}

	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_invitation_list);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_invitation_list *)(buffer +
						    sizeof
						    (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_INVITATION_LIST_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_invitation_list) -
		MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
		cmd_len += (WIFIDIRECT_INVITATION_LIST_MAX - 1) * ETH_ALEN;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		if ((ret = mac2raw(argv[2], tlv->inv_peer_addr)) != SUCCESS) {
			printf("ERR:Invalid MAC address %s\n", argv[2]);
			goto done;
		}
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				/* reusing variable cmd_len */
				cmd_len = tlv->length;
				if (cmd_len > 0) {
					printf("Invitation List =>");
					while (cmd_len >= ETH_ALEN) {
						if (memcmp
						    (tlv->inv_peer_addr,
						     "\x00\x00\x00\x00\x00\x00",
						     ETH_ALEN)) {
							printf("\n\t [%d] ",
							       ctr++);
							print_mac(tlv->
								  inv_peer_addr);
						}
						cmd_len -= ETH_ALEN;
						tlv = (tlvbuf_wifidirect_invitation_list *)(((t_u8 *)(tlv)) + ETH_ALEN);
					}
					if (ctr == 1)
						printf(" empty.");
				} else
					printf("Invitation List is empty.\n");
				printf("\n");
			} else {
				printf("Invitation List setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Invitation List!\n");
			} else {
				printf("ERR:Couldn't set Invitation List!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect noa config command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_noa_usage(void)
{
	printf("Usage : wifidirect_cfg_noa <enable|disable> <index> [<counttype> <duration> <interval>]\n");
	printf("If NOA parameters are provided, 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect NoA parameters
 *
 *  Usage: "Usage : wifidirect_cfg_noa <enable|disable> <index> [<counttype> <duration> <interval>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_noa(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_noa_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_noa_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc != 2) && (argc != 4) && (argc != 7)) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_noa_usage();
		return;
	}
	if (argc > 2) {
		if (argc == 4) {
			if (strcmp(argv[2], "enable") == 0) {
				printf("ERR: index, count_type, duration and interval must be" " specified if noa is enabled!\n");
				print_wifidirect_cfg_noa_usage();
				return;
			} else if (strcmp(argv[2], "disable")) {
				print_wifidirect_cfg_noa_usage();
				return;
			}
			if (atoi(argv[3]) > WIFIDIRECT_NOA_DESC_MAX) {
				print_wifidirect_cfg_noa_usage();
				return;
			}
		} else {
			if (atoi(argv[3]) > WIFIDIRECT_NOA_DESC_MAX ||
			    (is_input_valid(WIFIDIRECT_COUNT_TYPE, 1, &argv[4])
			     != SUCCESS) ||
			    (is_input_valid(WIFIDIRECT_DURATION, 1, &argv[5]) !=
			     SUCCESS) ||
			    (is_input_valid(WIFIDIRECT_INTERVAL, 1, &argv[6]) !=
			     SUCCESS)) {
				print_wifidirect_cfg_noa_usage();
				return;
			}
		}
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_noa_config);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_noa_config *)(buffer +
					       sizeof
					       (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_NOA_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_noa_config) - MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
		cmd_len +=
			(sizeof(tlvbuf_wifidirect_noa_config) -
			 MRVL_TLV_HEADER_SIZE)
			* (WIFIDIRECT_NOA_DESC_MAX - 1);
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		if (strcmp(argv[2], "disable")) {
			/* enable case */
			tlv->enable_noa = 1;
			tlv->enable_noa = cpu_to_le16(tlv->enable_noa);
		}
		tlv->noa_index = (t_u8)atoi(argv[3]);

		if (tlv->enable_noa) {
			tlv->count_type = (t_u8)atoi(argv[4]);
			tlv->duration = (t_u32)atoi(argv[5]);
			tlv->duration = cpu_to_le32(tlv->duration);
			tlv->interval = (t_u32)atoi(argv[6]);
			tlv->interval = cpu_to_le32(tlv->interval);
		}
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				if (tlv->length)
					printf("NoA settings:\n");
				else
					printf("NoA parameters are not configured.\n");
				if (tlv->length >=
				    sizeof(tlvbuf_wifidirect_noa_config) -
				    MRVL_TLV_HEADER_SIZE) {
					tlv->enable_noa =
						le16_to_cpu(tlv->enable_noa);
					if (tlv->enable_noa) {
						printf("[%d] NoA is enabled!\n",
						       (int)tlv->noa_index);
						printf("CountType = %d\n",
						       (int)tlv->count_type);
						printf("Duration = %dms\n",
						       le32_to_cpu(tlv->
								   duration));
						printf("Interval = %dms\n",
						       le32_to_cpu(tlv->
								   interval));
					} else {
						printf("[%d] NoA is disabled!\n", (int)tlv->noa_index);
					}
				}
			} else {
				printf("NoA setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get NoA parameters!\n");
			} else {
				printf("ERR:Couldn't set NoA parameters!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect opp_ps config command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_opp_ps_usage(void)
{
	printf("Usage : wifidirect_cfg_opp_ps <enable|disable> [<CTWindow>]\n");
	printf("For 'set', both enable flag and CTWindow should be provided.\n");
	printf("If no parameter is specified,'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect Opportunistic Power Save
 *
 *  Usage: "Usage : wifidirect_cfg_opp_ps <enable|disable> [<CTWindow>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_opp_ps(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_opp_ps_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u8 opp_ps = 0, ctwindow = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_opp_ps_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 2) || (argc > 4)) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_opp_ps_usage();
		return;
	}
	if (argc == 3) {
		if (strcmp(argv[2], "enable") == 0) {
			printf("ERR: If Opp PS is enabled, CTWindow must be specified!\n");
			print_wifidirect_cfg_opp_ps_usage();
			return;
		} else if (strcmp(argv[2], "disable")) {
			print_wifidirect_cfg_opp_ps_usage();
			return;
		}
	}
	if (argc == 4) {
		if (strcmp(argv[2], "disable") == 0) {
			printf("ERR: Extra parameter %s for disable command.\n",
			       argv[3]);
			return;
		}
		if (is_input_valid(WIFIDIRECT_CTWINDOW, 1, &argv[3]) != SUCCESS) {
			print_wifidirect_cfg_opp_ps_usage();
			return;
		}
	}

	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_opp_ps_config);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_opp_ps_config *)(buffer +
						  sizeof
						  (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_OPP_PS_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_opp_ps_config) - MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		if (strcmp(argv[2], "disable"))
			opp_ps = 1;
		if (argc == 4)
			tlv->ctwindow_opp_ps =
				(t_u8)atoi(argv[3]) | SET_OPP_PS(opp_ps);;
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				opp_ps = GET_OPP_PS(tlv->ctwindow_opp_ps);
				if (opp_ps) {
					ctwindow =
						(tlv->
						 ctwindow_opp_ps) &
						CT_WINDOW_MASK;
					printf("Opportunistic Power Save enabled!\n");
					printf("CTWindow  = %d\n", ctwindow);
				} else {
					printf("Opportunistic Power Save disabled!\n");
				}
			} else {
				printf("Opportunistic power save setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Opportunistic power save and CTWindow!\n");
			} else {
				printf("ERR:Couldn't set Opportunistic power save and CTWindow!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect capability config command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_capability_usage(void)
{
	printf("Usage : wifidirect_capability [<DeviceCapability> <GroupCapability>]\n");
	printf("For 'set' both DeviceCapability and GroupCapability should be provided.\n");
	printf("If no parameter is specified,'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect Capability
 *
 *  Usage: "Usage : wifidirect_cfg_capability [<DeviceCapability> <GroupCapability>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_capability(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_capability_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_capability_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 2) || (argc > 4) || (argc == 3)) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_capability_usage();
		return;
	}
	if (argc == 4) {
		if ((is_input_valid(WIFIDIRECT_DEVICECAPABILITY, 1, &argv[2]) !=
		     SUCCESS) ||
		    (is_input_valid(WIFIDIRECT_GROUPCAPABILITY, 1, &argv[3]) !=
		     SUCCESS)) {
			print_wifidirect_cfg_capability_usage();
			return;
		}
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_capability_config);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_capability_config *)(buffer +
						      sizeof
						      (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_CAPABILITY_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_capability_config) -
		MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->dev_capability = (t_u8)atoi(argv[2]);
		tlv->group_capability = (t_u8)atoi(argv[3]);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Device Capability = %d\n",
				       (int)tlv->dev_capability);
				printf("Group Capability = %d\n",
				       (int)tlv->group_capability);
			} else {
				printf("Capability setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Capability info!\n");
			} else {
				printf("ERR:Couldn't set Capability!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect_cfg_persistent_group_record command
 *
 *  $return         N/A
 */
static void
print_wifidirect_peristent_group_usage(void)
{
	printf("\nUsage : wifidirect_cfg_persistent_group_record [index] \
            <role> <bssid> <groupdevid> <ssid> <psk>> \
            [<peermac1> <peermac2> ]\n");
	printf("\nIssue set or get request using appropriate parameters.\n");
	return;
}

/**
 *  @brief Creates a wifidirect_persistent group record request and
 *         sends to the driver
 *
 *  Usage: "Usage : wifidirect_cfg_cmd_persistent_group_record [index] <
 *                  <role> <bssid> <groupdevid> <ssid> <psk>>
 *                  [<peermac1> <peermac2> ]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 **/
static void
wifidirect_cfg_cmd_persistent_group_record(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_persistent_group *tlv = NULL;
	t_u8 index = 0, role = 0, var_len = 0, psk_len = 0;
	t_u8 *buffer = NULL;
	int pi;
	t_u16 cmd_len = 0, buf_len = 0;
	int opt, ret;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_peristent_group_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 2 && argc != 3 && argc != 8 && argc != 9 && argc != 10) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_peristent_group_usage();
		return;
	}
	/* error checks */
	if (argc > 2) {
		index = atoi(argv[2]);
		if (index >= WIFIDIRECT_PERSISTENT_GROUP_MAX) {
			printf("ERR:wrong index. Value values are [0-3]\n");
			return;
		}
	}
	if (argc > 3) {
		role = atoi(argv[3]);
		if ((role != 1) && (role != 2)) {
			printf("ERR:Incorrect Role. Use 2 for client, 1 for group owner.\n");
			return;
		}
		if (role == 1 && argc == 8) {
			printf("ERR:Insufficient arguments. Please provide peer mac address(es) for group owner.\n");
			return;
		}
	}

	cmd_len = sizeof(wifidirect_params_config);
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}
	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);
	cmd_buf = (wifidirect_params_config *)buffer;
	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv = (tlvbuf_wifidirect_persistent_group *)(buffer +
						     sizeof
						     (wifidirect_params_config));

	tlv->tag = MRVL_WIFIDIRECT_PERSISTENT_GROUP_TLV_ID;

	/* parsing commands based on arguments */
	switch (argc) {
	case 2:
		cmd_buf->action = ACTION_GET;
		tlv->length = 0;
		cmd_len += MRVL_TLV_HEADER_SIZE + tlv->length;
		cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
		buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		break;

	case 3:
		cmd_buf->action = ACTION_GET;
		tlv->rec_index = index;
		tlv->length = sizeof(index);
		cmd_len += MRVL_TLV_HEADER_SIZE + tlv->length;
		cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
		buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		break;

	default:
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->rec_index = index;
		tlv->dev_role = role;
		if ((ret = mac2raw(argv[4], tlv->group_bssid)) != SUCCESS) {
			printf("ERR:Invalid MAC address %s\n", argv[4]);
			goto done;
		}
		if ((ret = mac2raw(argv[5], tlv->dev_mac_address)) != SUCCESS) {
			printf("ERR:Invalid MAC address %s\n", argv[5]);
			goto done;
		}

		/* ssid */
		tlv->group_ssid_len = strlen(argv[6]);
		var_len += tlv->group_ssid_len;
		memcpy(tlv->group_ssid, argv[6], tlv->group_ssid_len);

		/* adjust pointer from here */
		/* psk */
		if (!strncasecmp("0x", argv[7], 2)) {
			argv[7] += 2;
			if (strlen(argv[7]) / 2 != WIFIDIRECT_PSK_LEN_MAX) {
				printf("ERR:Incorrect PSK length %d. It should be %d.\n", (t_u32)strlen(argv[7]) / 2, WIFIDIRECT_PSK_LEN_MAX);
				goto done;
			}
			string2raw(argv[7], tlv->psk + var_len);
			*(&tlv->psk_len + var_len) = WIFIDIRECT_PSK_LEN_MAX;
			var_len += WIFIDIRECT_PSK_LEN_MAX;
		} else {
			/* ascii */
			psk_len = strlen(argv[7]);
			if (psk_len < WIFIDIRECT_PASSPHRASE_LEN_MIN ||
			    psk_len >= WIFIDIRECT_PSK_LEN_MAX) {
				printf("ERR:Incorrect passphrase length %d. Valid range is [8-63]\n", psk_len);
				goto done;
			}
			memcpy(tlv->psk + var_len, argv[7], psk_len);
			*(&tlv->psk_len + var_len) = psk_len;
			var_len += psk_len;
		}

		tlv->length =
			sizeof(tlvbuf_wifidirect_persistent_group) -
			MRVL_TLV_HEADER_SIZE + var_len;
		*(&tlv->num_peers + var_len) = 0;

		if (argc == 9 || argc == 10) {
			if ((ret =
			     mac2raw(argv[8], tlv->peer_mac_addrs[0] + var_len))
			    != SUCCESS) {
				printf("ERR:Invalid MAC address %s\n", argv[8]);
				goto done;
			}
			*(&tlv->num_peers + var_len) = 1;
		}
		if (argc == 10) {
			if ((ret =
			     mac2raw(argv[9], tlv->peer_mac_addrs[1] + var_len))
			    != SUCCESS) {
				printf("ERR:Invalid MAC address %s\n", argv[8]);
				goto done;
			}
			*(&tlv->num_peers + var_len) = 2;
		}

		tlv->length += *(&tlv->num_peers + var_len) * ETH_ALEN;
		cmd_len += MRVL_TLV_HEADER_SIZE + tlv->length;
		buf_len = cmd_len;
		cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
		break;
	}
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	endian_convert_tlv_header_out(tlv);

	/* Send collective command */
	wifidirect_ioctl((t_u8 *)buffer, &cmd_len, buf_len);

	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif

	if (argc == 2 || argc == 3) {
		if (tlv->length > 0)
			printf("Persistent group information =>\n");
		else
			printf("Persistent group information is empty.\n");
		buf_len = tlv->length;
		while (buf_len >
		       (sizeof(tlvbuf_wifidirect_persistent_group) -
			2 * ETH_ALEN)) {
			printf("\n\t Index = [%d]\n", tlv->rec_index);
			if (tlv->dev_role == 1)
				printf("\t Role  = Group owner\n");
			else
				printf("\t Role  = Client\n");
			printf("\t GroupBssId - ");
			print_mac(tlv->group_bssid);
			printf("\n\t DeviceId - ");
			print_mac(tlv->dev_mac_address);
			printf("\n\t SSID = ");
			for (index = 0; index < tlv->group_ssid_len; index++)
				printf("%c", tlv->group_ssid[index]);
			var_len = tlv->group_ssid_len;
			printf("\n\t PSK = ");
			for (index = 0; index < *(&tlv->psk_len + var_len);
			     index++) {
				if (index == 16)
					printf("\n\t       ");
				printf("%02x ", *(&tlv->psk[index] + var_len));
			}
			var_len += *(&tlv->psk_len + var_len);
			if (tlv->dev_role == 1) {
				for (pi = 0; pi < *(&tlv->num_peers + var_len);
				     pi++) {
					printf("\n\t Peer Mac address(%d) = ",
					       pi);
					print_mac(tlv->peer_mac_addrs[pi] +
						  var_len);
				}
				var_len +=
					(*(&tlv->num_peers + var_len) *
					 ETH_ALEN);
			}
			if (argc == 2)
				printf("\n\t -----------------------------------------");
			if (tlv->dev_role == 1) {
				buf_len -=
					sizeof
					(tlvbuf_wifidirect_persistent_group) -
					MRVL_TLV_HEADER_SIZE + var_len;
				tlv = (tlvbuf_wifidirect_persistent_group
				       *)(((t_u8 *)(tlv)) +
					  (sizeof
					   (tlvbuf_wifidirect_persistent_group)
					   - MRVL_TLV_HEADER_SIZE + var_len));
			} else {
				/* num_peer field willnt be present */
				buf_len -=
					sizeof
					(tlvbuf_wifidirect_persistent_group) -
					MRVL_TLV_HEADER_SIZE - sizeof(t_u8) +
					var_len;
				tlv = (tlvbuf_wifidirect_persistent_group
				       *)(((t_u8 *)(tlv)) +
					  (sizeof
					   (tlvbuf_wifidirect_persistent_group)
					   - MRVL_TLV_HEADER_SIZE -
					   sizeof(t_u8) + var_len));
			}
		}
		printf("\n");
	} else {
		printf("Setting persistent group information successful!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect_cfg_persistent_group_invoke command
 *
 *  $return         N/A
 */
static void
print_wifidirect_peristent_group_invoke_usage(void)
{
	printf("\nUsage : wifidirect_cfg_persistent_group_invoke [index] | <cancel>\n");
	return;
}

/**
 *  @brief Creates a wifidirect_persistent group Invoke request and
 *         sends to the driver
 *
 *  Usage: "Usage : wifidirect_cfg_cmd_persistent_group_invoke [index] | <cancel>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 **/
static void
wifidirect_cfg_cmd_persistent_group_invoke(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_persistent_group *tlv = NULL;
	t_u8 index;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_peristent_group_invoke_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 3) {
		printf("ERR:wrong number of arguments.\n");
		print_wifidirect_peristent_group_invoke_usage();
		return;
	}

	cmd_len = sizeof(wifidirect_params_config);
	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}
	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);
	cmd_buf = (wifidirect_params_config *)buffer;
	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = cpu_to_le16(ACTION_SET);
	tlv = (tlvbuf_wifidirect_persistent_group *)(buffer +
						     sizeof
						     (wifidirect_params_config));

	tlv->tag = MRVL_WIFIDIRECT_PERSISTENT_GROUP_TLV_ID;
	tlv->length = sizeof(index);

	index = atoi(argv[2]);
	if (index >= WIFIDIRECT_PERSISTENT_GROUP_MAX) {
		printf("ERR:wrong index. Value values are [0-3]\n");
		goto done;
	}
	if (!isdigit(argv[2][0])) {
		if (strcmp(argv[2], "cancel")) {
			printf("ERR:Incorrect input. Use index [0-3] or \"cancel\" \n");
			goto done;
		} else
			index = WIFIDIRECT_PERSISTENT_RECORD_CANCEL;
	}
	tlv->rec_index = index;

	cmd_len += MRVL_TLV_HEADER_SIZE + tlv->length;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	endian_convert_tlv_header_out(tlv);

	/* Send collective command */
	wifidirect_ioctl((t_u8 *)buffer, &cmd_len, MRVDRV_SIZE_OF_CMD_BUFFER);

#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif

	printf("Persistent group %s successful!\n",
	       (index ==
		WIFIDIRECT_PERSISTENT_RECORD_CANCEL) ? "cancel" : "invoke");
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect presence request parameters command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_presence_req_params_usage(void)
{
	printf("Usage : wifidirect_cfg_presence_req_params [<type> <duration> <interval>]\n");
	printf("      : Type 1: preferred values, 2: acceptable values.\n");
	printf("If presence request parameters are provided, "
	       " 'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect presence request parameters
 *
 *  Usage: "Usage : wifidirect_cfg_presence_req_params [<type> <duration> <interval>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_presence_req_params(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_presence_req_params *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_presence_req_params_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_presence_req_params_usage();
		return;
	} else if ((argc > 2) && ((argc != 5)
				  ||
				  (is_input_valid
				   (WIFIDIRECT_PRESENCE_REQ_TYPE, 1,
				    &argv[2]) != SUCCESS)
				  ||
				  (is_input_valid
				   (WIFIDIRECT_DURATION, 1,
				    &argv[3]) != SUCCESS)
				  ||
				  (is_input_valid
				   (WIFIDIRECT_INTERVAL, 1,
				    &argv[4]) != SUCCESS))) {
		print_wifidirect_cfg_presence_req_params_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_presence_req_params);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_presence_req_params *)(buffer +
							sizeof
							(wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_PRESENCE_REQ_PARAMS_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_presence_req_params) -
		MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->presence_req_type = (t_u8)atoi(argv[2]);
		tlv->duration = (t_u32)atoi(argv[3]);
		tlv->duration = cpu_to_le32(tlv->duration);
		tlv->interval = (t_u32)atoi(argv[4]);
		tlv->interval = cpu_to_le32(tlv->interval);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Presence request type = %d\n",
				       (int)tlv->presence_req_type);
				printf("Duration of NoA descriptor = %d\n",
				       le32_to_cpu(tlv->duration));
				printf("Interval of NoA descriptor = %d\n",
				       le32_to_cpu(tlv->interval));
			} else {
				printf("Presence request parameters setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get Presence request parameters!\n");
			} else {
				printf("ERR:Couldn't set Presence request parameters!\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect extended listen timing parameters command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_ext_listen_time_usage(void)
{
	printf("Usage : wifidirect_cfg_ext_listen_time [<duration> <interval>]\n");
	printf("If extended listen timing parameters are provided, "
	       "'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect extended listen time parameters
 *
 *  Usage: "Usage : wifidirect_cfg_ext_listen_time [<duration> <interval>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_ext_listen_time(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_mrvl_ext_listen_time *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_ext_listen_time_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_ext_listen_time_usage();
		return;
	} else if ((argc > 2) && ((argc != 4)
				  ||
				  (is_input_valid
				   (WIFIDIRECT_ATTR_EXTENDED_TIME, 1,
				    &argv[2]) != SUCCESS)
				  ||
				  (is_input_valid
				   (WIFIDIRECT_ATTR_EXTENDED_TIME, 1,
				    &argv[3]) != SUCCESS))) {
		print_wifidirect_cfg_ext_listen_time_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_mrvl_ext_listen_time);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_mrvl_ext_listen_time *)(buffer +
							 sizeof
							 (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_EXTENDED_LISTEN_TIME_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_mrvl_ext_listen_time) -
		MRVL_TLV_HEADER_SIZE;
	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->duration = cpu_to_le16((t_u16)atoi(argv[2]));
		tlv->interval = cpu_to_le16((t_u16)atoi(argv[3]));
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Availability Period = %d\n",
				       le16_to_cpu(tlv->duration));
				printf("Availability Interval = %d\n",
				       le16_to_cpu(tlv->interval));
			} else {
				printf("Extended listen timing parameters setting successful!\n");
			}
		} else {
			if (argc == 2)
				printf("ERR:Couldn't get Extended listen time!\n");
			else
				printf("ERR:Couldn't set Extended listen time!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect provisioing parameters command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_provisioning_params_usage(void)
{
	printf("Usage : wifidirect_cfg_provisioning_params [<action> <configMethod> <devPassword>]\n");
	printf("      : Action 1: Request parameters 2: Response parameters.\n");
	printf("If provisioning protocol parameters are provided, "
	       "'set' is performed otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect provisioning protocol related parameters
 *
 *  Usage: "Usage : wifidirect_cfg_provisioning_params [<action> <configMethod>
 *                  <devPassword>]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_provisioning_params(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_provisioning_params *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_provisioning_params_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc < 2) {
		printf("ERR: Wrong number of arguments!\n");
		print_wifidirect_cfg_provisioning_params_usage();
		return;
	} else if ((argc > 2) && (argc != 5)) {
		print_wifidirect_cfg_provisioning_params_usage();
		return;
	}
	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_provisioning_params);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_provisioning_params *)(buffer +
							sizeof
							(wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_PROVISIONING_PARAMS_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_provisioning_params) -
		MRVL_TLV_HEADER_SIZE;
	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->action = cpu_to_le16((t_u16)A2HEXDECIMAL(argv[2]));
		tlv->config_methods = cpu_to_le16((t_u16)A2HEXDECIMAL(argv[3]));
		tlv->dev_password = cpu_to_le16((t_u16)A2HEXDECIMAL(argv[4]));
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("Action = %s %s\n",
				       (le16_to_cpu(tlv->action)) ==
				       1 ? "Request" : "Response",
				       "parameters");
				printf("Config Methods = %02X\n",
				       le16_to_cpu(tlv->config_methods));
				printf("Device Password ID = %02X\n",
				       le16_to_cpu(tlv->dev_password));
			} else {
				printf("Provisioning protocol parameters setting successful!\n");
			}
		} else {
			if (argc == 2)
				printf("ERR:Couldn't get provisioing parameters!\n");
			else
				printf("ERR:Couldn't set provisioing parameters!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Show usage information for the wifidirect WPS params command
 *
 *  @return         N/A
 */
static void
print_wifidirect_cfg_wps_params_usage(void)
{
	printf("\nUsage : wifidirect_cfg_wps_params [pin/pbc/none]");
	printf("\nIf pin/pbc/none is provided, 'set' is performed"
	       " otherwise 'get' is performed.\n");
	return;
}

/**
 *  @brief Set/get wifidirect WPS parametters
 *
 *  Usage: "Usage : wifidirect_cfg_wps_params [pin/pbc/none]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         None
 */
static void
wifidirect_cfg_cmd_wps_params(int argc, char *argv[])
{
	wifidirect_params_config *cmd_buf = NULL;
	tlvbuf_wifidirect_wps_params *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	int opt, ret = 0, param = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_wifidirect_cfg_wps_params_usage();
			return;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 2) || (argc > 3)) {
		printf("ERR:Wrong number of arguments!\n");
		print_wifidirect_cfg_wps_params_usage();
		return;
	}

	cmd_len =
		sizeof(wifidirect_params_config) +
		sizeof(tlvbuf_wifidirect_wps_params);

	buffer = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);

	if (!buffer) {
		printf("ERR:Cannot allocate memory!\n");
		goto done;
	}

	memset(buffer, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd_buf = (wifidirect_params_config *)buffer;
	tlv = (tlvbuf_wifidirect_wps_params *)(buffer +
					       sizeof
					       (wifidirect_params_config));

	cmd_buf->cmd_code = HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	tlv->tag = MRVL_WIFIDIRECT_WPS_PARAMS_TLV_ID;
	tlv->length =
		sizeof(tlvbuf_wifidirect_wps_params) - MRVL_TLV_HEADER_SIZE;

	if (argc == 2) {
		cmd_buf->action = ACTION_GET;
	} else {
		if (strncmp(argv[2], "pin", 3) == 0)
			param = 1;
		else if (strncmp(argv[2], "pbc", 3) == 0)
			param = 2;
		else if (strncmp(argv[2], "none", 4)) {
			printf("Invalid input, should be pin/pbc or none.\n");
			goto done;
		}
		cmd_buf->action = cpu_to_le16(ACTION_SET);
		tlv->action = cpu_to_le16(param);
	}
	endian_convert_tlv_header_out(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len, ' ');
#endif
	ret = wifidirect_ioctl((t_u8 *)buffer, &cmd_len,
			       MRVDRV_SIZE_OF_CMD_BUFFER);
	endian_convert_tlv_header_in(tlv);
#ifdef DEBUG
	hexdump(buffer, cmd_len + BUF_HEADER_SIZE, ' ');
#endif
	/* Process Response */
	if (ret == SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_CMD_WIFIDIRECT_PARAMS_CONFIG |
		     WIFIDIRECTCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 2) {
				printf("WPS password parameter =>");
				switch (le16_to_cpu(tlv->action)) {
				case 1:
					printf("Pin\n");
					break;
				case 2:
					printf("PBC\n");
					break;
				default:
					printf("None\n");
					break;
				}
			} else {
				printf("WPS parameter setting successful!\n");
			}
		} else {
			if (argc == 2) {
				printf("ERR:Couldn't get WPS parameters !\n");
			} else {
				printf("ERR:Couldn't set WPS parameters !\n");
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	if (buffer)
		free(buffer);
	return;
}

/**
 *  @brief Checkes a particular input for validatation.
 *
 *  @param cmd      Type of input
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         SUCCESS or FAILURE
 */
int
is_input_valid(valid_inputs cmd, int argc, char *argv[])
{
	int i;
	int ret = SUCCESS;
	char country[6] = { ' ', ' ', 0, 0, 0, 0 };
	char wifidirect_dev_name[34];

	memset(wifidirect_dev_name, 0, sizeof(wifidirect_dev_name));
	if (argc == 0)
		return FAILURE;
	switch (cmd) {
	case WIFIDIRECT_MINDISCOVERYINT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for MinDiscoveryInterval\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) >= (1 << 16))) {
				printf("ERR:MinDiscoveryInterval must be 2 bytes\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_MAXDISCOVERYINT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for MaxDiscoveryInterval\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) >= (1 << 16))) {
				printf("ERR:MaxDiscoveryInterval must be 2 bytes\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DEVICECAPABILITY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DeviceCapability\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_DEV_CAPABILITY) ||
			    (atoi(argv[0]) < 0)) {
				printf("ERR:DeviceCapability must be in the range [0:%d]\n", MAX_DEV_CAPABILITY);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_GROUPCAPABILITY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for GroupCapability\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_GRP_CAPABILITY) ||
			    (atoi(argv[0]) < 0)) {
				printf("ERR:GroupCapability must be in the range [0:%d]\n", MAX_GRP_CAPABILITY);
				ret = FAILURE;
			}
		}
		break;
	case CHANNEL:
		if ((argc != 1) && (argc != 2)) {
			printf("ERR: Incorrect arguments for channel.\n");
			ret = FAILURE;
		} else {
			if (argc == 2) {
				if ((ISDIGIT(argv[1]) == 0) ||
				    (atoi(argv[1]) < 0) ||
				    (atoi(argv[1]) > 1)) {
					printf("ERR: MODE must be either 0 or 1\n");
					ret = FAILURE;
				}
				if ((atoi(argv[1]) == 1) &&
				    (atoi(argv[0]) != 0)) {
					printf("ERR: Channel must be 0 for ACS; MODE = 1.\n");
					ret = FAILURE;
				}
			}
			if ((argc == 1) || (atoi(argv[1]) == 0)) {
				if ((ISDIGIT(argv[0]) == 0) ||
				    (atoi(argv[0]) < 1) ||
				    (atoi(argv[0]) > MAX_CHANNELS)) {
					printf("ERR: Channel must be in the range of 1 to %d, configured - %d\n", MAX_CHANNELS, atoi(argv[0]));
					ret = FAILURE;
				}
			}
		}
		break;
	case SCANCHANNELS:
		if (argc > MAX_CHANNELS) {
			printf("ERR: Invalid List of Channels\n");
			ret = FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				if ((ISDIGIT(argv[i]) == 0) ||
				    (atoi(argv[i]) < 1) ||
				    (atoi(argv[i]) > MAX_CHANNELS)) {
					printf("ERR: Channel must be in the range of 1 to %d, configured - %d\n", MAX_CHANNELS, atoi(argv[i]));
					ret = FAILURE;
					break;
				}
			}
			if ((ret != FAILURE) &&
			    (has_dup_channel(argc, argv) != SUCCESS)) {
				printf("ERR: Duplicate channel values entered\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_INTENT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for intent\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_INTENT) ||
			    (atoi(argv[0]) < 0)) {
				printf("ERR:Intent must be in the range [0:%d]\n", MAX_INTENT);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_MANAGEABILITY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for manageability\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR:Manageability must be either 0 or 1\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_GROUP_WIFIDIRECT_DEVICE_NAME:
		/* 2 extra characters for quotes around device name */
		if ((strlen(argv[0]) > 34)) {
			printf("ERR:WIFIDIRECT Device name string length must not be more than 32\n");
			ret = FAILURE;
		} else {
			strncpy(wifidirect_dev_name, argv[0],
				sizeof(wifidirect_dev_name) - 1);
			if ((wifidirect_dev_name[0] != '"') ||
			    (wifidirect_dev_name
			     [strlen(wifidirect_dev_name) - 1] != '"')) {
				printf("ERR:WIFIDIRECT Device name must be within double quotes!\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_COUNTRY:
		/* 2 extra characters for quotes around country */
		if ((strlen(argv[0]) > 5) || (strlen(argv[0]) < 4)) {
			printf("ERR:Country string must have length 2 or 3\n");
			ret = FAILURE;
		} else {
			strncpy(country, argv[0], sizeof(country) - 1);
			if ((country[0] != '"') ||
			    (country[strlen(country) - 1] != '"')) {
				printf("ERR:country code must be within double quotes!\n");
				ret = FAILURE;
			} else {
				for (i = 1; i < strlen(country) - 2; i++) {
					if ((toupper(country[i]) < 'A') ||
					    (toupper(country[i]) > 'Z')) {
						printf("ERR:Invalid Country Code\n");
						ret = FAILURE;
					}
				}
			}
		}
		break;
	case WIFIDIRECT_NO_OF_CHANNELS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for num of channels\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_CHANNELS)) {
				printf("ERR:Number of channels should be less than %d\n", MAX_CHANNELS);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_NOA_INDEX:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for NoA Index\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) < MIN_NOA_INDEX) ||
			    (atoi(argv[0]) > MAX_NOA_INDEX)) {
				printf("ERR:NoA index should be in the range [%d:%d]\n", MIN_NOA_INDEX, MAX_NOA_INDEX);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_OPP_PS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Opp PS\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 0) &&
							(atoi(argv[0]) != 1))) {
				printf("ERR:Opp PS must be either 0 or 1\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_CTWINDOW:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for CTWindow\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_CTWINDOW) ||
			    (atoi(argv[0]) < MIN_CTWINDOW)) {
				printf("ERR:CT Window must be in the range [%d:%d]\n", MIN_CTWINDOW, MAX_CTWINDOW);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_COUNT_TYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Count/Type\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_COUNT_TYPE) ||
			    (atoi(argv[0]) < MIN_COUNT_TYPE)) {
				printf("ERR:Count/Type must be in the range [%d:%d] or 255\n", MIN_COUNT_TYPE, MAX_COUNT_TYPE);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_PRESENCE_REQ_TYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Presence request type\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 1)
							&& (atoi(argv[0]) !=
							    2))) {
				printf("ERR:Presence Type must be 1 or 2.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DURATION:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Duration\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_INTERVAL:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Interval\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_START_TIME:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Start Time\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_PRIDEVTYPEOUI:
		if (argc > MAX_PRIMARY_OUI_LEN) {
			printf("ERR: Incorrect number of PrimaryDeviceTypeOUI arguments.\n");
			ret = FAILURE;
			break;
		}
		for (i = 0; i < argc; i++) {
			if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
				printf("ERR:Unsupported OUI\n");
				ret = FAILURE;
				break;
			}
		}
		if (!
		    ((A2HEXDECIMAL(argv[0]) == 0x00) &&
		     (A2HEXDECIMAL(argv[1]) == 0x50)
		     && (A2HEXDECIMAL(argv[2]) == 0xF2) &&
		     (A2HEXDECIMAL(argv[3]) == 0x04))) {
			printf("ERR:Unsupported OUI\n");
			ret = FAILURE;
			break;
		}
		break;
	case WIFIDIRECT_REGULATORYCLASS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for RegulatoryClass\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_REG_CLASS) ||
			    (atoi(argv[0]) < MIN_REG_CLASS)) {
				printf("ERR:RegulatoryClass must be in the range [%d:%d] or 255\n", MIN_REG_CLASS, MAX_REG_CLASS);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_PRIDEVTYPECATEGORY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for PrimaryDeviceTypeCategory\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_PRIDEV_TYPE_CAT) ||
			    (atoi(argv[0]) < MIN_PRIDEV_TYPE_CAT)) {
				printf("ERR:PrimaryDeviceTypeCategory must be in the range [%d:%d]\n", MIN_PRIDEV_TYPE_CAT, MAX_PRIDEV_TYPE_CAT);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_SECONDARYDEVCOUNT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for SecondaryDeviceCount\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_SECONDARY_DEVICE_COUNT)) {
				printf("ERR:SecondaryDeviceCount must be less than 15.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_INTERFACECOUNT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for InterfaceCount\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_INTERFACE_ADDR_COUNT)) {
				printf("ERR:IntefaceCount must be in range.[0-41]\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_GROUP_SECONDARYDEVCOUNT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for SecondaryDeviceCount\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) >
			     MAX_GROUP_SECONDARY_DEVICE_COUNT)) {
				printf("ERR:SecondaryDeviceCount must be less than 2.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_ATTR_CONFIG_TIMEOUT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Timeout Configuration\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) > 255)) {
				printf("ERR:TimeoutConfig must be in the range [0:255]\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_ATTR_EXTENDED_TIME:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Extended time.\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) > 65535)
			    || (atoi(argv[0]) < 1)) {
				printf("ERR:Extended Time must be in the range [1:65535]\n");
				ret = FAILURE;
			}
		}
		break;

	case WIFIDIRECT_PRIDEVTYPESUBCATEGORY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for PrimaryDeviceTypeSubCategory\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_PRIDEV_TYPE_SUBCATEGORY) ||
			    (atoi(argv[0]) < MIN_PRIDEV_TYPE_SUBCATEGORY)) {
				printf("ERR:PrimaryDeviceTypeSubCategory must be in the range [%d:%d]\n", MIN_PRIDEV_TYPE_SUBCATEGORY, MAX_PRIDEV_TYPE_SUBCATEGORY);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_INVITATIONFLAG:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for InvitationFlag\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR:Invitation flag must be either 0 or 1\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSCONFMETHODS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSConfigMethods\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    (A2HEXDECIMAL(argv[0]) > MAX_WPS_CONF_METHODS) ||
			    (A2HEXDECIMAL(argv[0]) < MIN_WPS_CONF_METHODS)) {
				printf("ERR:WPSConfigMethods must be in the range [%d:%d]\n", MIN_WPS_CONF_METHODS, MAX_WPS_CONF_METHODS);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSVERSION:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSVersion\n");
			ret = FAILURE;
		} else {
			if ((A2HEXDECIMAL(argv[0]) < 0x10) &&
			    (A2HEXDECIMAL(argv[0]) > 0x20)) {
				printf("ERR:Incorrect WPS Version %s\n",
				       argv[0]);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSSETUPSTATE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSSetupState\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((A2HEXDECIMAL(argv[0]) != 0x01)
			     && (A2HEXDECIMAL(argv[0]) != 0x02))) {
				printf("ERR:Incorrect WPSSetupState %s\n",
				       argv[0]);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSREQRESPTYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSRequestType\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    (A2HEXDECIMAL(argv[0]) > WPS_MAX_REQUESTTYPE)) {
				printf("ERR:Incorrect WPSRequestType %s\n",
				       argv[0]);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSSPECCONFMETHODS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSSpecConfigMethods\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((t_u16)A2HEXDECIMAL(argv[0]) >
			     WPS_MAX_SPECCONFMETHODS) ||
			    ((t_u16)A2HEXDECIMAL(argv[0]) <
			     WPS_MIN_SPECCONFMETHODS)) {
				printf("ERR:WPSSpecConfigMethods must be in the range [%d:%d]\n", WPS_MIN_SPECCONFMETHODS, WPS_MAX_SPECCONFMETHODS);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSDEVICENAME:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments\n");
			ret = FAILURE;
		} else {
			if (argv[0][0] == '"') {
				argv[0]++;
			}
			if (argv[0][strlen(argv[0]) - 1] == '"') {
				argv[0][strlen(argv[0]) - 1] = '\0';
			}
			if (strlen(argv[0]) > WPS_DEVICE_NAME_MAX_LEN) {
				printf("ERR:Device name should contain"
				       " less than 32 charactors.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSMANUFACTURER:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments\n");
			ret = FAILURE;
		} else {
			if (strlen(argv[0]) > WPS_MANUFACT_MAX_LEN) {
				printf("ERR:Manufacturer name should contain"
				       "less than 64 charactors.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSMODELNAME:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments\n");
			ret = FAILURE;
		} else {
			if (strlen(argv[0]) > WPS_MODEL_MAX_LEN) {
				printf("ERR:Model name should contain"
				       " less than 64 charactors.\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSUUID:
		if (argc > WPS_UUID_MAX_LEN) {
			printf("ERR: Incorrect number of WPSUUID arguments.\n");
			ret = FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
					printf("ERR:Unsupported UUID\n");
					ret = FAILURE;
					break;
				}
			}
		}
		break;
	case WIFIDIRECT_WPSPRIMARYDEVICETYPE:
		if (argc > WPS_DEVICE_TYPE_MAX_LEN) {
			printf("ERR: Incorrect number of WPSPrimaryDeviceType arguments.\n");
			ret = FAILURE;
			break;
		}
		for (i = 0; i < argc; i++) {
			if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
				printf("ERR:Unsupported primary device type\n");
				ret = FAILURE;
				break;
			}
		}
		if (!
		    ((A2HEXDECIMAL(argv[2]) == 0x00) &&
		     (A2HEXDECIMAL(argv[3]) == 0x50)
		     && (A2HEXDECIMAL(argv[4]) == 0xF2) &&
		     (A2HEXDECIMAL(argv[5]) == 0x04))) {
			printf("ERR:Unsupported OUI\n");
			ret = FAILURE;
			break;
		}
		break;
	case WIFIDIRECT_WPSRFBAND:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSRFBand\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((A2HEXDECIMAL(argv[0]) != 0x01)
			     && (A2HEXDECIMAL(argv[0]) != 0x02))) {
				printf("ERR:Incorrect WPSRFBand %s\n", argv[0]);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSASSOCIATIONSTATE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSAssociationState\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((t_u16)A2HEXDECIMAL(argv[0]) >
			     WPS_MAX_ASSOCIATIONSTATE)) {
				printf("ERR:WPSAssociationState must be in the range [%d:%d]\n", WPS_MIN_ASSOCIATIONSTATE, WPS_MAX_ASSOCIATIONSTATE);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSCONFIGURATIONERROR:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSConfigurationError\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((t_u16)A2HEXDECIMAL(argv[0]) >
			     WPS_MAX_CONFIGURATIONERROR)) {
				printf("ERR:WPSConfigurationError must be in the range [%d:%d]\n", WPS_MIN_CONFIGURATIONERROR, WPS_MAX_CONFIGURATIONERROR);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSDEVICEPASSWORD:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for WPSDevicePassword\n");
			ret = FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    ((t_u16)A2HEXDECIMAL(argv[0]) >
			     WPS_MAX_DEVICEPASSWORD)) {
				printf("ERR:WPSDevicePassword must be in the range [%d:%d]\n", WPS_MIN_DEVICEPASSWORD, WPS_MAX_DEVICEPASSWORD);
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_WPSMODELNUMBER:
		if (argc > WPS_MODEL_MAX_LEN) {
			printf("ERR: Incorrect number of WPSModelNumber arguments.\n");
			ret = FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
					printf("ERR:Unsupported WPSModelNumber\n");
					ret = FAILURE;
					break;
				}
			}
		}
		break;
	case WIFIDIRECT_WPSSERIALNUMBER:
		if (argc > WPS_SERIAL_MAX_LEN) {
			printf("ERR: Incorrect number of WPSSerialNumber arguments.\n");
			ret = FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
					printf("ERR:Unsupported WPSSerialNumber\n");
					ret = FAILURE;
					break;
				}
			}
		}
		break;
	case WIFIDIRECT_CATEGORY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Category\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:Category incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_ACTION:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Action\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    ((atoi(argv[0]) > 0x10) &&
			     (atoi(argv[0]) != 0xDD))) {
				printf("ERR:Action must be less than 0x10 or 0xDD\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DIALOGTOKEN:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DialogToken\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:DialogToken incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_GAS_COMEBACK_DELAY:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for GAS Comeback Delay\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_DISC_ADPROTOIE:
		if (argc > MAX_ADPROTOIE_LEN) {
			printf("ERR: Incorrect number of AdvertisementProtocolIE arguments.\n");
			ret = FAILURE;
			break;
		}
		for (i = 0; i < argc; i++) {
			if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
				printf("ERR:Unsupported AdvertisementProtocolIE\n");
				ret = FAILURE;
				break;
			}
		}
		break;
	case WIFIDIRECT_DISC_INFOID:
		if (argc > MAX_INFOID_LEN) {
			printf("ERR: Incorrect number of DiscoveryInformationID arguments.\n");
			ret = FAILURE;
			break;
		}
		if (!
		    ((A2HEXDECIMAL(argv[0]) == 0xDD) &&
		     (A2HEXDECIMAL(argv[1]) == 0xDD)) &&
		    !((A2HEXDECIMAL(argv[0]) == 0xDE) &&
		      (A2HEXDECIMAL(argv[1]) == 0x92))) {
			printf("ERR:Unsupported DiscoveryInformationID\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_OUI:
		if (argc > MAX_OUI_LEN) {
			printf("ERR: Incorrect number of OUI arguments.\n");
			ret = FAILURE;
			break;
		}
		for (i = 0; i < argc; i++) {
			if (IS_HEX_OR_DIGIT(argv[i]) == FAILURE) {
				printf("ERR:Unsupported OUI\n");
				ret = FAILURE;
				break;
			}
		}
		break;
	case WIFIDIRECT_OUITYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for OUIType\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:OUIType incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_OUISUBTYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for OUISubtype\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:OUISubtype incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_SERVICEPROTO:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DiscoveryServiceProtocol\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) >= 4) &&
							(atoi(argv[0]) !=
							 0xFF))) {
				printf("ERR:DiscoveryServiceProtocol incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_SERVICEUPDATE_INDICATOR:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for ServiceUpdateIndicator\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:ServiceUpdateIndicator incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_SERVICETRANSACID:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DiscoveryServiceTransactionID\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:DiscoveryServiceTransactionID incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_SERVICE_STATUS:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for ServiceStatus\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) >= 4)) {
				printf("ERR:ServiceStatus incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_DNSTYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DiscoveryDNSType\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:DiscoveryDNSType incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_BONJOUR_VERSION:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Version\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:Version incorrect value\n");
				ret = FAILURE;
			}
		}
		break;
	case WIFIDIRECT_DISC_UPNP_VERSION:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for Version\n");
			ret = FAILURE;
		}
		break;
	case WIFIDIRECT_ENABLE_SCAN:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for EnableScan\n");
			ret = FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 0) &&
							(atoi(argv[0]) != 1))) {
				printf("ERR:EnableScan must be 0 or 1.\n");
				ret = FAILURE;
			}
		}
		break;

	case WIFIDIRECT_DEVICE_STATE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DeviceState\n");
			ret = FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR:Incorrect DeviceState.\n");
				ret = FAILURE;
			}
		}
		break;

	default:
		printf("Parameter validity for %d ignored\n", cmd);
		break;
	}
	return ret;
}

/** Structure of command table*/
typedef struct {
	/** Command name */
	char *cmd;
	/** Command function pointer */
	void (*func) (int argc, char *argv[]);
	/** Command usuage */
	char *help;
} command_table;

/** WiFiDisplay command table */
static command_table wifidisplay_command[] = {
	{"wifidisplay_config", wifidisplaycmd_config,
	 "\tSet/get wifidisplay configuration"},
	{"wifidisplay_mode", wifidisplay_cmd_status,
	 "\tSet/get WFD Display status"},
	{"wifidisplay_update_devinfo", wifidisplay_update_custom_ie,
	 "\tUpdate WFD device info bitmap"},
	{"wifidisplay_update_coupledsink_bitmap",
	 wifidisplay_update_coupledsink_bitmap,
	 "\tUpdate WFD coupled sink bitmap"},
	{"wifidisplay_discovery_request", wifidisplaycmd_service_discovery,
	 "Send wifidisplay service discovery request"},
	{"wifidisplay_discovery_response", wifidisplaycmd_service_discovery,
	 "Send wifidisplay service discovery response"},
	{NULL, NULL, 0},
};

/** WIFIDIRECT command table */
static command_table wifidirect_command[] = {
	{"wifidirect_config", wifidirectcmd_config,
	 "\tSet/get wifidirect configuration"},
	{"wifidirect_params_config", wifidirectcmd_params_config,
	 "\tSet/get wifidirect parameter's configuration"},
	{"wifidirect_action_frame", wifidirectcmd_action_frame,
	 "\tSend wifidirect action frame."},
	{"wifidirect_mode", wifidirectcmd_status,
	 "\tSet/get WIFIDIRECT start/stop status"},
	{"wifidirect_discovery_request", wifidirectcmd_service_discovery,
	 "Send wifidirect service discovery request"},
	{"wifidirect_discovery_response", wifidirectcmd_service_discovery,
	 "Send wifidirect service discovery response"},
	{"wifidirect_gas_comeback_request",
	 wifidirectcmd_gas_comeback_discovery,
	 "Send wifidirect GAS comeback request frame"},
	{"wifidirect_gas_comeback_response",
	 wifidirectcmd_gas_comeback_discovery,
	 "Send wifidirect GAS comeback response frame"},
	{"wifidirect_cfg_persistent_group_record",
	 wifidirect_cfg_cmd_persistent_group_record,
	 "Set/get information about a persistent group"},
	{"wifidirect_cfg_persistent_group_invoke",
	 wifidirect_cfg_cmd_persistent_group_invoke,
	 "Invoke or disable a persistent group"},
	{"wifidirect_cfg_discovery_period", wifidirect_cfg_cmd_discovery_period,
	 "\tSet/get discovery period"},
	{"wifidirect_cfg_intent", wifidirect_cfg_cmd_intent,
	 "\tSet/get intent"},
	{"wifidirect_cfg_capability", wifidirect_cfg_cmd_capability,
	 "\tSet/get capability"},
	{"wifidirect_cfg_noa", wifidirect_cfg_cmd_noa,
	 "\tSet/get notice of absence"},
	{"wifidirect_cfg_opp_ps", wifidirect_cfg_cmd_opp_ps,
	 "\tSet/get Opportunistic PS"},
	{"wifidirect_cfg_invitation_list", wifidirect_cfg_cmd_invitation_list,
	 "\tSet/get invitation list"},
	{"wifidirect_cfg_listen_channel", wifidirect_cfg_cmd_listen_channel,
	 "\tSet/get listen channel"},
	{"wifidirect_cfg_op_channel", wifidirect_cfg_cmd_op_channel,
	 "\tSet/get operating channel"},
	{"wifidirect_cfg_presence_req_params",
	 wifidirect_cfg_cmd_presence_req_params,
	 "\tSet/get presence request parameters"},
	{"wifidirect_cfg_ext_listen_time", wifidirect_cfg_cmd_ext_listen_time,
	 "Set/get extended listen timing parameters"},
	{"wifidirect_cfg_provisioning_params",
	 wifidirect_cfg_cmd_provisioning_params,
	 "Set/get provisioning protocol related parameters"},
	{"wifidirect_cfg_wps_params", wifidirect_cfg_cmd_wps_params,
	 "Set/get WPS protocol related parameters"},
	{NULL, NULL, 0}
};

/**
 *  @brief Entry function for wifidirectutl
 *  @param argc         number of arguments
 *  @param argv     A pointer to arguments array
 *  @return             SUCCESS or FAILURE
 */
int
main(int argc, char *argv[])
{
	int i;

	if (argc < 3) {
		print_tool_usage();
		exit(1);
	}

	strncpy(dev_name, argv[1], IFNAMSIZ);
	/* Process command */
	for (i = 0; wifidirect_command[i].cmd; i++) {
		if (strncmp
		    (wifidirect_command[i].cmd, argv[2],
		     strlen(wifidirect_command[i].cmd)))
			continue;
		if (strlen(wifidirect_command[i].cmd) != strlen(argv[2]))
			continue;
		wifidirect_command[i].func(argc, argv);
		break;
	}
	for (i = 0; wifidisplay_command[i].cmd; i++) {
		if (strncmp
		    (wifidisplay_command[i].cmd, argv[2],
		     strlen(wifidisplay_command[i].cmd)))
			continue;
		if (strlen(wifidisplay_command[i].cmd) != strlen(argv[2]))
			continue;
		wifidisplay_command[i].func(argc, argv);
		break;
	}
	if ((!wifidirect_command[i].cmd) && (!wifidisplay_command[i].cmd)) {
		printf("ERR: %s is not supported\n", argv[2]);
		exit(1);
	}
	return 0;

}
