/** @file  uaputl.c
  *
  * @brief Program to send AP commands to the driver/firmware of the uAP
  *        driver.
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
    03/01/08: Initial creation
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
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "uaputl.h"
#include "uapcmd.h"

/****************************************************************************
        Definitions
****************************************************************************/
/** Default debug level */
int debug_level = MSG_NONE;

/** Enable or disable debug outputs */
#define DEBUG   1

/** Convert character to integer */
#define CHAR2INT(x) (((x) >= 'A') ? ((x) - 'A' + 10) : ((x) - '0'))

/** Supported stream modes */
#define HT_STREAM_MODE_1X1   0x11
#define HT_STREAM_MODE_2X2   0x22

static int get_bss_config(t_u8 *buf);

/****************************************************************************
        Global variables
****************************************************************************/
/** Device name */
static char dev_name[IFNAMSIZ + 1];
/** Option for cmd */
struct option cmd_options[] = {
	{"help", 0, 0, 'h'},
	{0, 0, 0, 0}
};

/** Flag to check if custom IE is present in sys_config response from FW */
int custom_ie_present = 0;

/** Flag to check if max mgmt IE is printed in sys_config response from FW */
int max_mgmt_ie_print = 0;

/** Flag to bypass re-route path */
int uap_ioctl_no_reroute = 0;

/****************************************************************************
        Local functions
****************************************************************************/
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
 *  @brief Check protocol is valid or not
 *
 *  @param protocol    	     Protocol
 *
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_protocol_valid(int protocol)
{
	int ret = UAP_FAILURE;
	switch (protocol) {
	case PROTOCOL_NO_SECURITY:
	case PROTOCOL_STATIC_WEP:
	case PROTOCOL_WPA:
	case PROTOCOL_WPA2:
	case PROTOCOL_WPA2_MIXED:
		ret = UAP_SUCCESS;
		break;
	default:
		printf("ERR: Invalid Protocol: %d\n", protocol);
		break;
	}
	return ret;
}

/**
 *  @brief Function to check valid rate
 *
 *  @param  rate    Rate to verify
 *
 *  @return 	    UAP_SUCCESS or UAP_FAILURE
 **/
int
is_rate_valid(int rate)
{
	int ret = UAP_SUCCESS;
	switch (rate) {
	case 2:
	case 4:
	case 11:
	case 22:
	case 12:
	case 18:
	case 24:
	case 48:
	case 72:
	case 96:
	case 108:
	case 36:
		break;
	default:
		ret = UAP_FAILURE;
		break;
	}
	return ret;
}

/**
 *  @brief Check for mandatory rates
 *
 *
 * 2, 4, 11, 22 must be present
 *
 * 6 12 and 24 must be present for ofdm
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_FAILURE or UAP_SUCCESS
 *
 */
static int
check_mandatory_rates(int argc, char **argv)
{
	int i;
	int tmp;
	t_u32 rate_bitmap = 0;
	int cck_enable = 0;
	int ofdm_enable = 0;
#define BITMAP_RATE_1M         0x01
#define BITMAP_RATE_2M         0x02
#define BITMAP_RATE_5_5M       0x04
#define BITMAP_RATE_11M        0x8
#define B_RATE_MANDATORY       0x0f
#define BITMAP_RATE_6M         0x10
#define BITMAP_RATE_12M        0x20
#define BITMAP_RATE_24M        0x40
#define G_RATE_MANDATORY        0x70
	for (i = 0; i < argc; i++) {
		tmp = (A2HEXDECIMAL(argv[i]) & ~BASIC_RATE_SET_BIT);
		switch (tmp) {
		case 2:
			cck_enable = 1;
			rate_bitmap |= BITMAP_RATE_1M;
			break;
		case 4:
			cck_enable = 1;
			rate_bitmap |= BITMAP_RATE_2M;
			break;
		case 11:
			cck_enable = 1;
			rate_bitmap |= BITMAP_RATE_5_5M;
			break;
		case 22:
			cck_enable = 1;
			rate_bitmap |= BITMAP_RATE_11M;
			break;
		case 12:
			ofdm_enable = 1;
			rate_bitmap |= BITMAP_RATE_6M;
			break;
		case 24:
			ofdm_enable = 1;
			rate_bitmap |= BITMAP_RATE_12M;
			break;
		case 48:
			ofdm_enable = 1;
			rate_bitmap |= BITMAP_RATE_24M;
			break;
		case 18:
		case 36:
		case 72:
		case 96:
		case 108:
			ofdm_enable = 1;
			break;
		}
	}
#ifdef WIFI_DIRECT_SUPPORT
	if (strncmp(dev_name, "wfd", 3))
#endif
		if ((rate_bitmap & B_RATE_MANDATORY) != B_RATE_MANDATORY) {
			if (cck_enable) {
				printf("Basic Rates 2, 4, 11 and 22 (500K units) \n" "must be present in basic or non-basic rates\n");
				return UAP_FAILURE;
			}
		}
	if (ofdm_enable &&
	    ((rate_bitmap & G_RATE_MANDATORY) != G_RATE_MANDATORY)) {
		printf("OFDM Rates 12, 24 and 48 ( 500Kb units)\n"
		       "must be present in basic or non-basic rates\n");
		return UAP_FAILURE;
	}
	return UAP_SUCCESS;
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
 *  @brief Dump hex data
 *
 *  @param prompt	A pointer prompt buffer
 *  @param p		A pointer to data buffer
 *  @param len		The len of data buffer
 *  @param delim	Delim char
 *  @return            	None
 */
void
hexdump_data(char *prompt, void *p, int len, char delim)
{
	int i;
	unsigned char *s = p;

	if (prompt) {
		printf("%s: len=%d\n", prompt, (int)len);
	}
	for (i = 0; i < len; i++) {
		if (i != len - 1)
			printf("%02x%c", *s++, delim);
		else
			printf("%02x\n", *s);
		if ((i + 1) % 16 == 0)
			printf("\n");
	}
	printf("\n");
}

#if DEBUG
/**
 * @brief           Conditional printf
 *
 * @param level     Severity level of the message
 * @param fmt       Printf format string, followed by optional arguments
 */
void
uap_printf(int level, char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	if (level <= debug_level) {
		vprintf(fmt, ap);
	}
	va_end(ap);
}

/**
 *  @brief Dump hex data
 *
 *  @param prompt	A pointer prompt buffer
 *  @param p		A pointer to data buffer
 *  @param len		The len of data buffer
 *  @param delim	Delim char
 *  @return            	None
 */
void
hexdump(char *prompt, void *p, int len, char delim)
{
	if (debug_level < MSG_ALL)
		return;
	hexdump_data(prompt, p, len, delim);
}
#endif

/**
 *  @brief      Hex to number
 *
 *  @param c    Hex value
 *  @return     Integer value or -1
 */
int
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
 *  @brief Show usage information for the sys_info command
 *
 *  @return         N/A
 */
void
print_sys_info_usage(void)
{
	printf("\nUsage : sys_info\n");
	return;
}

/**
 *  @brief  Get Max sta num from firmware
 *
 *  @return     max number of stations
 */
int
get_max_sta_num_supported(t_u16 *max_sta_num_supported)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_max_sta_num *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_max_sta_num);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_max_sta_num *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_MAX_STA_CNT_TLV_ID;
	tlv->length = 4;
	cmd_buf->action = ACTION_GET;

	endian_convert_tlv_header_out(tlv);
	tlv->max_sta_num_configured =
		uap_cpu_to_le16(tlv->max_sta_num_configured);
	tlv->max_sta_num_supported =
		uap_cpu_to_le16(tlv->max_sta_num_supported);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->max_sta_num_configured =
		uap_le16_to_cpu(tlv->max_sta_num_configured);
	tlv->max_sta_num_supported =
		uap_le16_to_cpu(tlv->max_sta_num_supported);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_MAX_STA_CNT_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result != CMD_SUCCESS) {
			printf("ERR:Could not get max station number!\n");
			ret = UAP_FAILURE;
		} else {
			if (tlv->length == 4) {
				*max_sta_num_supported =
					tlv->max_sta_num_supported;
			} else {
				*max_sta_num_supported = MAX_STA_COUNT;
			}
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Parse domain name for domain_code
 *
 *  @param dom_name  Domain name
 *  @return domain_code / UAP_FAILURE
 */
static t_u8
parse_domain_name(char *domain_name)
{
	t_u8 domain_code = UAP_FAILURE;

	if (strstr(domain_name, "DOMAIN_CODE_")) {
		domain_name += sizeof("DOMAIN_CODE_") - 1;

		if (!(strncmp(domain_name, "FCC", sizeof("FCC") - 1)))
			domain_code = DOMAIN_CODE_FCC;
		else if (!(strcmp(domain_name, "ETSI")))
			domain_code = DOMAIN_CODE_ETSI;
		else if (!(strcmp(domain_name, "MKK")))
			domain_code = DOMAIN_CODE_MKK;
		else if (!(strcmp(domain_name, "IN")))
			domain_code = DOMAIN_CODE_IN;
		else if (!(strcmp(domain_name, "MY")))
			domain_code = DOMAIN_CODE_MY;
	}
	return domain_code;
}

/**
 *  @brief Parse domain file for country information
 *
 *  @param country  Country name
 *  @param band    Band Info. 0x01 : B band, 0x02 : G band, 0x04 : A band.
 *  @param sub_bands Band information
 *  @param pdomain_code Pointer to receive domain_code
 *  @return number of band/ UAP_FAILURE
 */
t_u8
parse_domain_file(char *country, int band, ieeetypes_subband_set_t *sub_bands,
		  t_u8 *pdomain_code)
{
	FILE *fp;
	char str[64];
	char domain_name[64];
	int cflag = 0;
	int dflag = 0;
	int found = 0;
	int skip = 0;
	int j = -1, reset_j = 0;
	t_u8 no_of_sub_band = 0;
	char *strp = NULL;
	int ret = 0;

	fp = fopen("config/80211d_domain.conf", "r");
	if (fp == NULL) {
		printf("File opening Error\n");
		return UAP_FAILURE;
	}

	memset(domain_name, 0, sizeof(domain_name));
    /**
     * Search specific domain name
     */
	memset(str, 0, 64);
	while (!feof(fp)) {
		ret = fscanf(fp, "%63s", str);
		if (ret <= 0)
			break;
		if (cflag) {
			strncpy(domain_name, str, sizeof(domain_name) - 1);
			cflag = 0;
		}
		if (!strcmp(str, "COUNTRY:")) {
	    /** Store next string to domain_name */
			cflag = 1;
		}

		if (!strcmp(str, country)) {
	    /** Country is matched ;)*/
			found = 1;
			break;
		}
	}

	if (!found) {
		printf("No match found for Country = %s in the 80211d_domain.conf \n", country);
		fclose(fp);
		found = 0;
		return UAP_FAILURE;
	}

    /**
     * Search domain specific information
     */
	while (!feof(fp)) {
		ret = fscanf(fp, "%63s", str);
		if (ret <= 0)
			break;

		if (feof(fp)
			) {
			break;
		}

		if (dflag && !strcmp(str, "DOMAIN:")) {

			if ((band & BAND_A) == 0)
				break;

			/* parse next domain */
			cflag = 0;
			dflag = 0;
			j = -1;
			reset_j = 0;
		}
		if (dflag) {
			j++;
			if (strchr(str, ','))
				reset_j = 1;

			strp = strtok(str, ", ");

			if (strp == NULL) {
				if (reset_j) {
					j = -1;
					reset_j = 0;
				}
				continue;
			} else {
				strncpy(str, strp, (sizeof(str) - 1));
			}

			if (IS_HEX_OR_DIGIT(str) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				fclose(fp);
				return UAP_FAILURE;
			}

			switch (j) {
			case 0:
				sub_bands[no_of_sub_band].first_chan =
					(t_u8)A2HEXDECIMAL(str);
				break;
			case 1:
				sub_bands[no_of_sub_band].no_of_chan =
					(t_u8)A2HEXDECIMAL(str);
				break;
			case 2:
				sub_bands[no_of_sub_band++].max_tx_pwr =
					(t_u8)A2HEXDECIMAL(str);
				break;
			default:
				printf("ERR: Incorrect 80211d_domain.conf file\n");
				fclose(fp);
				return UAP_FAILURE;
			}

			if (reset_j) {
				j = -1;
				reset_j = 0;
			}
		}

		if (cflag && !strcmp(str, domain_name)) {
			/* Followed will be the band details */
			cflag = 0;
			if (band & (BAND_B | BAND_G) || skip)
				dflag = 1;
			else
				skip = 1;
		}
		if (!dflag && !strcmp(str, "DOMAIN:")) {
			cflag = 1;
		}
	}
	fclose(fp);

	if (pdomain_code && no_of_sub_band && (band & BAND_A)) {
		*pdomain_code = parse_domain_name(domain_name);
	}
	return (no_of_sub_band);
}

/**
 *
 *  @brief Set/Get SNMP MIB
 *
 *  @param action 0-GET 1-SET
 *  @param oid    Oid
 *  @param size   Size of oid value
 *  @param oid_buf  Oid value
 *  @return UAP_FAILURE or UAP_SUCCESS
 *
 */
int
sg_snmp_mib(t_u16 action, t_u16 oid, t_u16 size, t_u8 *oid_buf)
{
	apcmdbuf_snmp_mib *cmd_buf = NULL;
	tlvbuf_header *tlv = NULL;
	int ret = UAP_FAILURE;
	t_u8 *buf = NULL;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	t_u16 cmd_len;
	int i;

	cmd_len = sizeof(apcmdbuf_snmp_mib) + sizeof(tlvbuf_header) + size;
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return ret;
	}
	memset(buf, 0, buf_len);

	/* Locate Headers */
	cmd_buf = (apcmdbuf_snmp_mib *)buf;
	tlv = (tlvbuf_header *)(buf + sizeof(apcmdbuf_snmp_mib));
	cmd_buf->size = buf_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->cmd_code = HostCmd_SNMP_MIB;

	tlv->type = uap_cpu_to_le16(oid);
	tlv->len = uap_cpu_to_le16(size);
	for (i = 0; action && (i < size); i++) {
		tlv->data[i] = oid_buf[i];
	}

	cmd_buf->action = uap_cpu_to_le16(action);
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			if (!action) {
		/** Relocate the headers */
				tlv = (tlvbuf_header *)((t_u8 *)cmd_buf +
							sizeof
							(apcmdbuf_snmp_mib));
				for (i = 0;
				     i < MIN(uap_le16_to_cpu(tlv->len), size);
				     i++) {
					oid_buf[i] = tlv->data[i];
				}
			}
			ret = UAP_SUCCESS;
		} else {
			printf("ERR:Command Response incorrect!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	free(buf);
	return ret;
}

/**
 *  @brief Creates a sys_info request and sends to the driver
 *
 *  Usage: "sys_info"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_info(int argc, char *argv[])
{
	apcmdbuf_sys_info_request *cmd_buf = NULL;
	apcmdbuf_sys_info_response *response_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_info_usage();
			return UAP_FAILURE;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_sys_info_usage();
		return UAP_FAILURE;
	}

	/* Alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);

	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_sys_info_request);
	cmd_buf = (apcmdbuf_sys_info_request *)buf;
	response_buf = (apcmdbuf_sys_info_response *)buf;

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_INFO;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (response_buf->cmd_code !=
		    (APCMD_SYS_INFO | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			free(buf);
			return UAP_FAILURE;
		}
		/* Print response */
		if (response_buf->result == CMD_SUCCESS) {
			printf("System information = %s\n",
			       response_buf->sys_info);
		} else {
			printf("ERR:Could not retrieve system information!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	free(buf);
	return ret;
}

/**
 *  @brief Show usage information for deepsleep command
 *
 *  @return         N/A
 */
void
print_deepsleep_usage(void)
{
	printf("\nUsage : deepsleep [MODE][IDLE_TIME]");
	printf("\nOptions: MODE : 0 - Disable auto deep sleep mode");
	printf("\n                1 - Enable auto deep sleep mode");
	printf("\n         IDLE_TIME: Idle time in milliseconds, default value is 100 ms");
	printf("\n         empty - get auto deep sleep mode\n");
	return;
}

/**
 *  @brief Creates deepsleep  request and send to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_deepsleep(int argc, char *argv[])
{
	int opt;
	deep_sleep_para param;
	struct ifreq ifr;
	t_s32 sockfd;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_deepsleep_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if (argc > 2) {
		printf("ERR:wrong arguments. Only support 2 arguments\n");
		print_deepsleep_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_DEEP_SLEEP;
	if (argc) {
		if (argc >= 1) {
			if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) ||
			    ((atoi(argv[0]) < 0) || (atoi(argv[0]) > 1))) {
				printf("ERR: Only Number values are allowed\n");
				print_deepsleep_usage();
				return UAP_FAILURE;
			}
		}
		param.action = 1;
		param.deep_sleep = (t_u16)A2HEXDECIMAL(argv[0]);
		param.idle_time = 0;
		if (argc == 2) {
			if (IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) {
				printf("ERR: Only Number values are allowed\n");
				print_deepsleep_usage();
				return UAP_FAILURE;
			}
			param.idle_time = (t_u16)A2HEXDECIMAL(argv[1]);
		}
	} else {
		param.action = 0;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		if (argc)
			printf("ERR: deep sleep must be disabled before changing idle time\n");
		else {
			perror("");
			printf("ERR:deep sleep failed\n");
		}
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		if (param.deep_sleep == 1) {
			printf("deep sleep mode: enabled\n");
			printf("idle time = %dms\n", param.idle_time);
		} else {
			printf("deep sleep mode: disabled\n");
		}
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for tx_data_pause command
 *
 *  $return         N/A
 */
void
print_txdatapause_usage(void)
{
	printf("\nUsage : tx_data_pause [ENABLE][TX_BUF_CNT]");
	printf("\nOptions: ENABLE : 0 - Disable Tx data pause events");
	printf("\n                  1 - Enable Tx data pause events");
	printf("\n         TX_BUF_CNT: Max number of TX buffer for PS clients");
	printf("\n         empty - get Tx data pause settings\n");
	return;
}

/**
 *  @brief Set/get txpause setting
 *
 *  @param txpause  A pointer to the Tx data pause parameters structure
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
send_txpause_ioctl(tx_data_pause_para *txpause)
{
	struct ifreq ifr;
	t_s32 sockfd;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)txpause;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR:txpause is not supported by %s\n", dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Creates tx_data_pause request and sends to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_txdatapause(int argc, char *argv[])
{
	int opt;
	tx_data_pause_para param;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_txdatapause_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if (argc > 2) {
		printf("ERR: Wrong number of arguments\n");
		print_txdatapause_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_TX_DATA_PAUSE;
	if (argc) {
		if (argc >= 1) {
			if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) ||
			    ((atoi(argv[0]) < TX_DATA_PAUSE_DISABLE) ||
			     (atoi(argv[0]) > TX_DATA_PAUSE_ENABLE))) {
				printf("ERR: First argument can either be 0 or 1\n");
				print_txdatapause_usage();
				return UAP_FAILURE;
			}
		}
		if (argc == 1) {
			param.action = ACTION_GET;
			if (UAP_FAILURE == send_txpause_ioctl(&param)) {
				return UAP_FAILURE;
			}
		}
		param.action = ACTION_SET;
		param.txpause = (t_u16)A2HEXDECIMAL(argv[0]);
		if (argc == 2) {
			if (IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE) {
				printf("ERR: Max buffer length must be numeric\n");
				print_txdatapause_usage();
				return UAP_FAILURE;
			}
			param.txbufcnt = (t_u16)A2HEXDECIMAL(argv[1]);
		}
	} else {
		param.action = ACTION_GET;
	}
	if (UAP_FAILURE == send_txpause_ioctl(&param))
		return UAP_FAILURE;
	if ((argc == 2) && ((t_u16)A2HEXDECIMAL(argv[1]) != param.txbufcnt)) {
		printf("Max number of TX buffer allowed for all PS client: %d\n", param.txbufcnt);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("Tx data pause: %s\n",
		       (param.txpause == 1) ? "enabled" : "disabled");
		printf("Max number of TX buffer allowed for all PS client: %d\n", param.txbufcnt);
	}
	return UAP_SUCCESS;
}

/**
 *  @brief Process host_cmd
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         N/A
 */
int
apcmd_hostcmd(int argc, char *argv[])
{
	apcmdbuf *hdr;
	t_u8 *buffer = NULL;
	int ret = UAP_SUCCESS;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	char cmdname[256];

	if (argc <= 2) {
		printf("Error: invalid no of arguments\n");
		printf("Syntax: ./uaputl hostcmd <hostcmd.conf> <cmdname>\n");
		return UAP_FAILURE;
	}

	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}

	memset(buffer, 0, buf_len);
	sprintf(cmdname, "%s", argv[2]);
	ret = prepare_host_cmd_buffer(argv[1], cmdname, buffer);
	if (ret == UAP_FAILURE)
		goto _exit_;

	/* Locate headers */
	hdr = (apcmdbuf *)buffer;
	cmd_len = hdr->size + BUF_HEADER_SIZE;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)buffer, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		hdr->cmd_code &= HostCmd_CMD_ID_MASK;
		if (!hdr->result) {
			printf("UAPHOSTCMD: CmdCode=%#04x, Size=%#04x, SeqNum=%#04x, Result=%#04x\n", hdr->cmd_code, hdr->size, hdr->seq_num, hdr->result);
			hexdump_data("payload",
				     (void *)(buffer + APCMDHEADERLEN),
				     hdr->size - (APCMDHEADERLEN -
						  BUF_HEADER_SIZE), ' ');
		} else
			printf("UAPHOSTCMD failed: CmdCode=%#04x, Size=%#04x, SeqNum=%#04x, Result=%#04x\n", hdr->cmd_code, hdr->size, hdr->seq_num, hdr->result);
	} else
		printf("ERR:Command sending failed!\n");

_exit_:
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Show usage information for addbapara command
 *
 *  @return         N/A
 */
void
print_addbapara_usage(void)
{
	printf("\nUsage : addbapara [timeout txwinsize rxwinsize]");
	printf("\nOptions: timeout : 0 - Disable");
	printf("\n                   1 - 65535 : Block Ack Timeout in TU");
	printf("\n         txwinsize: Buffer size for ADDBA request");
	printf("\n         rxwinsize: Buffer size for ADDBA response");
	printf("\n         txamsdu: amsdu for ADDBA request");
	printf("\n         rxamsdu: amsdu for ADDBA response");
	printf("\n         empty - get ADDBA parameters\n");
	return;
}

/**
 *  @brief Creates addbaparam request and send to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_addbapara(int argc, char *argv[])
{
	int opt;
	addba_param param;
	struct ifreq ifr;
	t_s32 sockfd;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_addbapara_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if ((argc != 0) && (argc != 5)) {
		printf("ERR:wrong arguments. Only support 0 or 5 arguments\n");
		print_addbapara_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_ADDBA_PARA;
	if (argc) {
		if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE)
		    || (IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE)
		    || (IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE)
		    || (IS_HEX_OR_DIGIT(argv[3]) == UAP_FAILURE)
		    || (IS_HEX_OR_DIGIT(argv[4]) == UAP_FAILURE)
			) {
			printf("ERR: Only Number values are allowed\n");
			print_addbapara_usage();
			return UAP_FAILURE;
		}
		param.action = 1;
		param.timeout = (t_u32)A2HEXDECIMAL(argv[0]);
		if (param.timeout > DEFAULT_BLOCK_ACK_TIMEOUT) {
			printf("ERR: Block Ack timeout should be in range [1-65535]\n");
			print_addbapara_usage();
			return UAP_FAILURE;
		}
		param.txwinsize = (t_u32)A2HEXDECIMAL(argv[1]);
		param.rxwinsize = (t_u32)A2HEXDECIMAL(argv[2]);
		if (param.txwinsize > MAX_TXRX_WINDOW_SIZE ||
		    param.rxwinsize > MAX_TXRX_WINDOW_SIZE) {
			printf("ERR: Tx/Rx window size should not be greater than 1023\n");
			print_addbapara_usage();
			return UAP_FAILURE;
		}
		param.txamsdu = (t_u8)A2HEXDECIMAL(argv[3]);
		param.rxamsdu = (t_u8)A2HEXDECIMAL(argv[4]);
		if (param.txamsdu > 1 || param.rxamsdu > 1) {
			printf("ERR: Tx/Rx amsdu should not be 0 or 1\n");
			print_addbapara_usage();
			return UAP_FAILURE;
		}
	} else {
		param.action = 0;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR:ADDBA PARA failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("ADDBA parameters:\n");
		printf("\ttimeout=%d\n", (int)param.timeout);
		printf("\ttxwinsize=%d\n", (int)param.txwinsize);
		printf("\trxwinsize=%d\n", (int)param.rxwinsize);
		printf("\ttxamsdu=%d\n", (int)param.txamsdu);
		printf("\trxamsdu=%d\n", (int)param.rxamsdu);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for aggrpriotbl command
 *
 *  @return         N/A
 */
void
print_aggrpriotbl_usage(void)
{
	printf("\nUsage : aggrpriotbl <m0> <n0> <m1> <n1> ... <m7> <n7>");
	printf("\nOptions: <mx> : 0 - 7, 0xff to disable AMPDU aggregation.");
	printf("\n         <nx> : 0 - 7, 0xff to disable AMSDU aggregation.");
	printf("\n         empty - get the priority table\n");
	return;
}

/**
 *  @brief Creates aggrpriotbl request and send to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_aggrpriotbl(int argc, char *argv[])
{
	int opt;
	aggr_prio_tbl prio_tbl;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u8 value;
	int i;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_aggrpriotbl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&prio_tbl, 0, sizeof(prio_tbl));
	/* Check arguments */
	if ((argc != 0) && (argc != 16)) {
		printf("ERR:wrong arguments. Only support 0 or 16 arguments\n");
		print_aggrpriotbl_usage();
		return UAP_FAILURE;
	}
	prio_tbl.subcmd = UAP_AGGR_PRIOTBL;
	if (argc) {
		for (i = 0; i < argc; i++) {
			if ((IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE)) {
				printf("ERR: Only Number values are allowed\n");
				print_aggrpriotbl_usage();
				return UAP_FAILURE;
			}
			value = (t_u8)A2HEXDECIMAL(argv[i]);
			if ((value > 7) && (value != 0xff)) {
				printf("ERR: Invalid priority, Valid value 0-7, 0xff to disable aggregation.\n");
				print_aggrpriotbl_usage();
				return UAP_FAILURE;
			}
		}
		prio_tbl.action = 1;
		for (i = 0; i < MAX_NUM_TID; i++) {
			prio_tbl.ampdu[i] = (t_u8)A2HEXDECIMAL(argv[i * 2]);
			prio_tbl.amsdu[i] = (t_u8)A2HEXDECIMAL(argv[i * 2 + 1]);
		}
	} else {
		prio_tbl.action = 0;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&prio_tbl;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: priority table failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("AMPDU/AMSDU priority table:");
		for (i = 0; i < MAX_NUM_TID; i++) {
			printf(" %d %d", prio_tbl.ampdu[i], prio_tbl.amsdu[i]);
		}
		printf("\n");
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for addbareject command
 *
 *  $return         N/A
 */
void
print_addba_reject_usage(void)
{
	printf("\nUsage : addbareject <m0> <m1> ... <m7>");
	printf("\nOptions: <mx> : 1 enables rejection of ADDBA request for TidX.");
	printf("\n                0 would accept any ADDBAs for TidX.");
	printf("\n         empty - get the addbareject table\n");
	return;
}

/**
 *  @brief Creates addbareject request and send to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_addbareject(int argc, char *argv[])
{
	int opt;
	addba_reject_para param;
	struct ifreq ifr;
	t_s32 sockfd;
	int i;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_addba_reject_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if ((argc != 0) && (argc != 8)) {
		printf("ERR:wrong arguments. Only support 0 or 8 arguments\n");
		print_addba_reject_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_ADDBA_REJECT;
	if (argc) {
		for (i = 0; i < argc; i++) {
			if ((ISDIGIT(argv[i]) == UAP_FAILURE) ||
			    (atoi(argv[i]) < 0) || (atoi(argv[i]) > 1)) {
				printf("ERR: Only allow 0 or 1\n");
				print_addba_reject_usage();
				return UAP_FAILURE;
			}
		}
		param.action = 1;
		for (i = 0; i < MAX_NUM_TID; i++) {
			param.addba_reject[i] = (t_u8)atoi(argv[i]);
		}
	} else {
		param.action = 0;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: addba reject table failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("addba reject table: ");
		for (i = 0; i < MAX_NUM_TID; i++) {
			printf("%d ", param.addba_reject[i]);
		}
		printf("\n");
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Creates get_fw_info request and send to driver
 *
 *
 *  @param pfw_info Pointer to FW information structure
 *  @return         0--success, otherwise fail
 */
int
get_fw_info(fw_info *pfw_info)
{
	struct ifreq ifr;
	t_s32 sockfd;

	memset(pfw_info, 0, sizeof(fw_info));
	pfw_info->subcmd = UAP_FW_INFO;
	pfw_info->action = 0;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return -1;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)pfw_info;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: get fw info failed\n");
		close(sockfd);
		return -1;
	}
	/* Close socket */
	close(sockfd);
	return 0;
}

/**
 *  @brief Show usage information for HT Tx command
 *
 *  $return         N/A
 */
void
print_ht_tx_usage(void)
{
	printf("\nUsage : httxcfg [<txcfg>]");
	printf("\nOptions: txcfg : This is a bitmap and should be used as following");
	printf("\n                      Bit 15-7: Reserved set to 0");
	printf("\n                      Bit 6: Short GI in 40 Mhz enable/disable");
	printf("\n                      Bit 5: Short GI in 20 Mhz enable/disable");
	printf("\n                      Bit 4: Green field enable/disable");
	printf("\n                      Bit 3-2: Reserved set to 0");
	printf("\n                      Bit 1: 20/40 Mhz enable disable.");
	printf("\n                      Bit 0: Reserved set to 0");
	return;
}

/**
 *  @brief Process HT Tx configuration
 *  @param argc   Number of arguments
 *  @param argv   A pointer to arguments array
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
apcmd_sys_cfg_ht_tx(int argc, char *argv[])
{
	int opt;
	ht_tx_cfg_para param;
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_ht_tx_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if (argc == 0) {
		param.action = ACTION_GET;
	} else if (argc == 1) {
		param.action = ACTION_SET;
		param.tx_cfg.httxcap = (t_u16)A2HEXDECIMAL(argv[0]);
	} else {
		print_ht_tx_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_HT_TX_CFG;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: HT Tx configuration failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}

	/* Handle response */
	if (param.action == ACTION_GET) {
		printf("HT Tx cfg: 0x%08x\n", param.tx_cfg.httxcap);
	}

	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for vhtcfg command
 *
 *  @return         N/A
 */
void
print_vht_usage(void)
{
	printf("\nUsage : vhtcfg <band> <txrx> <bwcfg> <vhtcap> <tx_mcs_map>"
	       "<rx_mcs_map>");
	printf("\nOption:  band : This is the band info for vhtcfg settings.");
	printf("\n                      0: Settings for both 2.4G and 5G bands");
	printf("\n                      1: Settings for 2.4G band");
	printf("\n                      2: Settings for 5G band");
	printf("\n         txrx : This parameter specifies the configuration of VHT" "operation for TX or/and VHT capabilities.");
	printf("\n                      1: configuration of VHT capabilities tx");
	printf("\n                      2: configuration of VHT capabilities rx");
	printf("\n         bwcfg : This parameter specifies the bandwidth (BW)"
	       "configuration applied to the vhtcfg.");
	printf("\n                  If <txrx> is 1/3 (Tx operations)");
	printf("\n                      0: Tx BW follows the BW (20/40 MHz) from" "11N CFG");
	printf("\n                      1: Tx BW follows the BW (80/160/80+80 MHz)" "from VHT Capabilities.");
	printf("\n         vhtcap : This parameter specifies the VHT capabilities info.");
	printf("\n         tx_mcs_map : This parameter specifies the TX MCS map.");
	printf("\n         rx_mcs_map : This parameter specifies the RX MCS map.\n");
	return;
}

/**
 *  @brief Handle response of the vhtcfg command
 *
 *  @param vhtcfg    Pointer to structure eth_priv_vhtcfg
 *
 *  @return         N/A
 */
void
print_vht_response(struct eth_priv_vhtcfg *vhtcfg)
{
	/* GET operation */
	/* Band */
	if (vhtcfg->band == BAND_SELECT_BG)
		printf("Band: 2.4G\n");
	else
		printf("Band: 5G\n");
	/* BW confi9 */
	if (vhtcfg->txrx & 0x3) {
		if (vhtcfg->bwcfg == 0)
			printf("    BW config: the 11N config\n");
		else
			printf("    BW config: the VHT Capabilities\n");
	}
	/* Tx/Rx */
	if (vhtcfg->txrx & 0x1)
		printf("    VHT operation for Tx: 0x%08x\n",
		       vhtcfg->vht_cap_info);
	if (vhtcfg->txrx & 0x2)
		/* VHT capabilities */
		printf("    VHT Capabilities Info: 0x%08x\n",
		       vhtcfg->vht_cap_info);
	/* MCS */
	printf("    Tx MCS set: 0x%04x\n", vhtcfg->vht_tx_mcs);
	printf("    Rx MCS set: 0x%04x\n", vhtcfg->vht_rx_mcs);
}

/**
 *  @brief Set/Get 11AC configurations
 *  @param argc   Number of arguments
 *  @param argv   A pointer to arguments array
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
apcmd_sys_cfg_vht(int argc, char *argv[])
{
	int opt, i = 0;
	vht_cfg_para param;
	struct eth_priv_vhtcfg vhtcfg;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u8 *respbuf = NULL, num = 0;
	fw_info fw;
	if (0 == get_fw_info(&fw)) {
		/*check whether support 802.11AC through BAND_AAC bit */
		if (!(fw.fw_bands & BAND_AAC)) {
			printf("ERR: No support 802 11AC.\n");
			return UAP_FAILURE;
		}
	} else {
		printf("ERR: get_fw_info fail\n");
		return UAP_FAILURE;
	}
	memset(&param, 0, sizeof(param));
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_vht_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	respbuf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!respbuf) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(respbuf, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	/* Check arguments */
	if ((argc > 6) || (argc < 2)) {
		printf("ERR: Invalid number of arguments.\n");
		print_vht_usage();
		free(respbuf);
		return UAP_FAILURE;
	}
	if ((atoi(argv[0]) < 0) || (atoi(argv[0]) > 2)) {
		printf("ERR: Invalid band selection.\n");
		print_vht_usage();
		free(respbuf);
		return UAP_FAILURE;
	} else {
		param.vht_cfg.band = (t_u32)A2HEXDECIMAL(argv[0]);
	}
	if ((atoi(argv[1]) <= 0) || (atoi(argv[1]) > 3)) {
		printf("ERR: Invalid Tx/Rx selection.\n");
		print_vht_usage();
		free(respbuf);
		return UAP_FAILURE;
	} else {
		param.vht_cfg.txrx = (t_u32)A2HEXDECIMAL(argv[1]);
	}
	if (argc == 2) {
		param.action = ACTION_GET;
	} else if (argc > 2) {
		param.vht_cfg.band = (t_u32)A2HEXDECIMAL(argv[0]);
		param.vht_cfg.txrx = (t_u32)A2HEXDECIMAL(argv[1]);
		if (argc == 3) {
			printf("ERR: Invalid number of arguments.\n");
			print_vht_usage();
			free(respbuf);
			return UAP_FAILURE;
		}
		if (argc >= 4) {
			if ((atoi(argv[2]) < 0) || (atoi(argv[2]) > 1) ||
			    ((atoi(argv[2]) == 1) &&
			     (atoi(argv[0]) & BAND_SELECT_BG))) {
				printf("ERR: Invalid BW cfg selection.\n");
				print_vht_usage();
				free(respbuf);
				return UAP_FAILURE;
			} else {
				param.vht_cfg.bwcfg =
					(t_u32)A2HEXDECIMAL(argv[2]);
			}
			param.vht_cfg.vht_cap_info =
				(t_u32)A2HEXDECIMAL(argv[3]);
			if (argc == 4) {
				param.vht_cfg.vht_tx_mcs = 0xffffffff;
				param.vht_cfg.vht_rx_mcs = 0xffffffff;
			} else {
				if (argc == 5) {
					printf("ERR: Invalid number of arguments.\n");
					print_vht_usage();
					free(respbuf);
					return UAP_FAILURE;
				}
				param.vht_cfg.vht_tx_mcs =
					(t_u32)A2HEXDECIMAL(argv[4]);
				param.vht_cfg.vht_rx_mcs =
					(t_u32)A2HEXDECIMAL(argv[5]);
			}
		}
		param.action = ACTION_SET;
	} else {
		print_vht_usage();
		free(respbuf);
		return UAP_FAILURE;
	}
	param.subcmd = UAP_VHT_CFG;
	memcpy(respbuf, &param, sizeof(vht_cfg_para));

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		free(respbuf);
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)respbuf;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: 11ac VHT configuration failed\n");
		close(sockfd);
		free(respbuf);
		return UAP_FAILURE;
	}

	/* Handle response */
	if (param.action == ACTION_GET) {
		/* Process result */
		/* the first attribute is the number of vhtcfg entries */
		num = *respbuf;
		printf("11AC VHT Configuration: \n");
		for (i = 0; i < num; i++) {
			memcpy(&vhtcfg, respbuf + 1 + i * sizeof(vhtcfg),
			       sizeof(vhtcfg));
			print_vht_response(&vhtcfg);
		}
	} else
		printf("11AC VHT Configuration success!\n");

	/* Close socket */
	close(sockfd);
	if (respbuf)
		free(respbuf);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for TX BF command
 *
 *  @return         N/A
 */
void
print_tx_bf_usage(void)
{
	printf("\nUsage : httxbfcfg <ACTION> [ACT_DATA]");
	printf("\nOptions: ACTION : 1 - Performs NDP Sounding for PEER");
	printf("\n                  2 - TX BF interval in milliseconds");
	printf("\n                  3 - Not to perform any sounding");
	printf("\n                  4 - TX BF SNR Threshold for peer");
	printf("\n         ACT_DATA : Specific data for the above actions");
	printf("\n                    For 1: PEER MAC and status");
	printf("\n                    For 2: TX BF interval");
	printf("\n                    For 3: PEER MAC");
	printf("\n                    For 4: PEER MAC and SNR");
	printf("\n                    empty - get action specific settings\n");
	return;
}

/**
 *  @brief Handle response of the TX BF command
 *
 *  @param param    Pointer to structure tx_bf_cfg_para
 *
 *  @return         N/A
 */
void
print_tx_bf_response(tx_bf_cfg_para *param)
{
	int i;
	trigger_sound_args *bf_sound = param->body.bf_sound;
	tx_bf_peer_args *tx_bf_peer = param->body.tx_bf_peer;
	snr_thr_args *bf_snr = param->body.bf_snr;
	bf_periodicity_args *bf_periodicity = param->body.bf_periodicity;
	bf_global_cfg_args *bf_global = &param->body.bf_global_cfg;

	switch (param->bf_action) {
	case BF_GLOBAL_CONFIGURATION:
		printf("Global BF Status :%s\n",
		       bf_global->bf_enbl ? "ENABLED" : "DISABLED");
		printf("Global Sounding Status :%s\n",
		       bf_global->sounding_enbl ? "ENABLED" : "DISABLED");
		printf("Default FB Type :%d\n", bf_global->fb_type);
		printf("Default SNR Threshold :%d\n", bf_global->snr_threshold);
		printf("Default Sounding Interval :%d\n",
		       bf_global->sounding_interval);
		printf("Beamforming Mode :%d\n", bf_global->bf_mode);
		break;
	case TRIGGER_SOUNDING_FOR_PEER:
		printf("PEER MAC = %02X:%02X:%02X:%02X:%02X:%02X, STATUS = %s\n", bf_sound->peer_mac[0], bf_sound->peer_mac[1], bf_sound->peer_mac[2], bf_sound->peer_mac[3], bf_sound->peer_mac[4], bf_sound->peer_mac[5], bf_sound->status ? "Failure" : "Success");
		break;
	case SET_GET_BF_PERIODICITY:
		printf("PEER MAC = %02x:%02x:%02x:%02x:%02x:%02x, Interval (ms) = %d\n", bf_periodicity->peer_mac[0], bf_periodicity->peer_mac[1], bf_periodicity->peer_mac[2], bf_periodicity->peer_mac[3], bf_periodicity->peer_mac[4], bf_periodicity->peer_mac[5], bf_periodicity->interval);
		break;
	case TX_BF_FOR_PEER_ENBL:
		for (i = 0; i < param->no_of_peers; i++) {
			printf("PEER MAC = %02x:%02x:%02x:%02x:%02x:%02x\n",
			       tx_bf_peer->peer_mac[0], tx_bf_peer->peer_mac[1],
			       tx_bf_peer->peer_mac[2], tx_bf_peer->peer_mac[3],
			       tx_bf_peer->peer_mac[4],
			       tx_bf_peer->peer_mac[5]);
			printf("BF Status : %s\n",
			       tx_bf_peer->bf_enbl ? "ENABLED" : "DISABLED");
			printf("Sounding Status : %s\n",
			       tx_bf_peer->
			       sounding_enbl ? "ENABLED" : "DISABLED");
			printf("FB Type : %d\n", tx_bf_peer->fb_type);
			tx_bf_peer++;
		}
		break;
	case SET_SNR_THR_PEER:
		for (i = 0; i < param->no_of_peers; i++) {
			printf("PEER MAC = %02x:%02x:%02x:%02x:%02x:%02x, SNR = %d\n", bf_snr->peer_mac[0], bf_snr->peer_mac[1], bf_snr->peer_mac[2], bf_snr->peer_mac[3], bf_snr->peer_mac[4], bf_snr->peer_mac[5], bf_snr->snr);
			bf_snr++;
		}
		break;
	}
}

/** Tx BF Global conf argument index */
#define BF_ENABLE_PARAM     1
#define SOUND_ENABLE_PARAM  2
#define FB_TYPE_PARAM       3
#define SNR_THRESHOLD_PARAM 4
#define SOUND_INTVL_PARAM   5
#define BF_MODE_PARAM       6
#define BF_CFG_ACT_GET      0
#define BF_CFG_ACT_SET      1

/**
 *  @brief Creates TX BF request and send to driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_tx_bf(int argc, char *argv[])
{
	int opt, ret = UAP_FAILURE;
	tx_bf_cfg_para param;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u32 bf_action = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_tx_bf_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if (argc < 1) {
		printf("ERR: wrong arguments.\n");
		print_tx_bf_usage();
		return UAP_FAILURE;
	}
	if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) ||
	    ((atoi(argv[0]) < BF_GLOBAL_CONFIGURATION) ||
	     (atoi(argv[0]) > SET_SNR_THR_PEER))) {
		printf("ERR: Only Number values are allowed\n");
		print_tx_bf_usage();
		return UAP_FAILURE;
	}
	bf_action = (t_u32)A2HEXDECIMAL(argv[0]);
	param.subcmd = UAP_TX_BF_CFG;
	param.bf_action = bf_action;
	switch (bf_action) {
	case BF_GLOBAL_CONFIGURATION:
		if (argc != 1 && argc != 7) {
			printf("Invalid argument for Global BF Configuration\n");
			return UAP_FAILURE;
		}
		if (argc == 1) {
			param.bf_cmd_action = BF_CFG_ACT_GET;
			param.action = ACTION_GET;
		} else {
			param.bf_cmd_action = BF_CFG_ACT_SET;
			param.action = ACTION_SET;
			param.body.bf_global_cfg.bf_enbl =
				atoi(argv[BF_ENABLE_PARAM]);
			param.body.bf_global_cfg.sounding_enbl =
				atoi(argv[SOUND_ENABLE_PARAM]);
			param.body.bf_global_cfg.fb_type =
				atoi(argv[FB_TYPE_PARAM]);
			param.body.bf_global_cfg.snr_threshold =
				atoi(argv[SNR_THRESHOLD_PARAM]);
			param.body.bf_global_cfg.sounding_interval =
				atoi(argv[SOUND_INTVL_PARAM]);
			param.body.bf_global_cfg.bf_mode =
				atoi(argv[BF_MODE_PARAM]);
		}
		break;
	case TRIGGER_SOUNDING_FOR_PEER:
		if (argc != 2) {
			printf("ERR: wrong arguments.\n");
			print_tx_bf_usage();
			return UAP_FAILURE;
		}
		if ((ret =
		     mac2raw(argv[1],
			     param.body.bf_sound[0].peer_mac)) != UAP_SUCCESS) {
			printf("ERR: %s Address\n",
			       ret == UAP_FAILURE ? "Invalid MAC" : ret ==
			       UAP_RET_MAC_BROADCAST ? "Broadcast" :
			       "Multicast");
			return UAP_FAILURE;
		}
		param.bf_cmd_action = BF_CFG_ACT_SET;
		param.action = ACTION_SET;
		break;
	case SET_GET_BF_PERIODICITY:
		if (argc != 2 && argc != 3) {
			printf("ERR: wrong arguments.\n");
			print_tx_bf_usage();
			return UAP_FAILURE;
		}
		if ((ret =
		     mac2raw(argv[1],
			     param.body.bf_periodicity[0].peer_mac)) !=
		    UAP_SUCCESS) {
			printf("ERR: %s Address\n",
			       ret == UAP_FAILURE ? "Invalid MAC" : ret ==
			       UAP_RET_MAC_BROADCAST ? "Broadcast" :
			       "Multicast");
			return UAP_FAILURE;
		}

		if (argc == 3) {
			if (IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE)
				return UAP_FAILURE;
			param.body.bf_periodicity[0].interval =
				(t_u32)A2HEXDECIMAL(argv[2]);
			param.bf_cmd_action = BF_CFG_ACT_SET;
			param.action = ACTION_SET;
		} else {
			param.bf_cmd_action = BF_CFG_ACT_GET;
			param.action = ACTION_GET;
		}
		break;
	case TX_BF_FOR_PEER_ENBL:
		if (argc != 1 && argc != 5) {
			printf("ERR: wrong arguments.\n");
			print_tx_bf_usage();
			return UAP_FAILURE;
		}
		if (argc == 1) {
			param.bf_cmd_action = BF_CFG_ACT_GET;
			param.action = ACTION_GET;
		} else {
			if ((ret =
			     mac2raw(argv[1],
				     param.body.tx_bf_peer[0].peer_mac)) !=
			    UAP_SUCCESS) {
				printf("ERR: %s Address\n",
				       ret ==
				       UAP_FAILURE ? "Invalid MAC" : ret ==
				       UAP_RET_MAC_BROADCAST ? "Broadcast" :
				       "Multicast");
				return UAP_FAILURE;
			}
			param.body.tx_bf_peer->bf_enbl = atoi(argv[2]);
			param.body.tx_bf_peer->sounding_enbl = atoi(argv[3]);
			param.body.tx_bf_peer->fb_type = atoi(argv[4]);
			param.bf_cmd_action = BF_CFG_ACT_SET;
			param.action = ACTION_SET;
		}
		break;
	case SET_SNR_THR_PEER:
		if (argc != 1 && argc != 3) {
			printf("ERR: wrong arguments.\n");
			print_tx_bf_usage();
			return UAP_FAILURE;
		}
		if (argc == 1) {
			param.bf_cmd_action = BF_CFG_ACT_GET;
			param.action = ACTION_GET;
		} else {
			if ((ret =
			     mac2raw(argv[1],
				     param.body.bf_snr[0].peer_mac)) !=
			    UAP_SUCCESS) {
				printf("ERR: %s Address\n",
				       ret ==
				       UAP_FAILURE ? "Invalid MAC" : ret ==
				       UAP_RET_MAC_BROADCAST ? "Broadcast" :
				       "Multicast");
				return UAP_FAILURE;
			}
			if (IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE)
				return UAP_FAILURE;
			param.body.bf_snr[0].snr = (t_u8)A2HEXDECIMAL(argv[2]);
			param.bf_cmd_action = BF_CFG_ACT_SET;
			param.action = ACTION_SET;
		}
		break;
	default:
		return UAP_FAILURE;
	}

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: TX BF configuration failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}

	printf("TX BF configuration successful\n");
	/* Handle response */
	if (param.action == ACTION_GET)
		print_tx_bf_response(&param);

	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for hscfg command
 *
 *  @return         N/A
 */
void
print_hscfg_usage(void)
{
	printf("\nUsage : hscfg [condition [[GPIO# [gap]]]]");
	printf("\nOptions: condition : bit 0 = 1   -- broadcast data");
	printf("\n                     bit 1 = 1   -- unicast data");
	printf("\n                     bit 2 = 1   -- mac event");
	printf("\n                     bit 3 = 1   -- multicast packet");
	printf("\n                     bit 6 = 1   -- mgmt frame received");
	printf("\n         GPIO: the pin number (e.g. 0-7) of GPIO used to wakeup the host");
	printf("\n               or 0xff interface (e.g. SDIO) used to wakeup the host");
	printf("\n         gap: time between wakeup signal and wakeup event (in milliseconds)");
	printf("\n              or 0xff for special setting when GPIO is used to wakeup host");
	printf("\n         empty - get current host sleep parameters\n");
	return;
}

/**
 *  @brief Creates host sleep parameter request and send to driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_hscfg(int argc, char *argv[])
{
	int opt;
	int i = 0;
	ds_hs_cfg hscfg;
	struct ifreq ifr;
	t_s32 sockfd;

	if ((argc == 2) && strstr(argv[1], "-1"))
		strcpy(argv[1], "0xffff");

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_hscfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&hscfg, 0, sizeof(hscfg));
	hscfg.subcmd = UAP_HS_CFG;
	/* Check arguments */
	if (argc > 3) {
		printf("ERR:wrong arguments.\n");
		print_hscfg_usage();
		return UAP_FAILURE;
	}
	if (argc) {
		for (i = 0; i < argc; i++) {
			if (IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE) {
				printf("ERR: Invalid argument %s\n", argv[i]);
				print_hscfg_usage();
				return UAP_FAILURE;
			}
		}
	}

	if (argc) {
		hscfg.flags |= HS_CFG_FLAG_SET | HS_CFG_FLAG_CONDITION;
		hscfg.conditions = (t_u32)A2HEXDECIMAL(argv[0]);
		if (hscfg.conditions >= 0xffff)
			hscfg.conditions = HS_CFG_CANCEL;
		if ((hscfg.conditions != HS_CFG_CANCEL) &&
		    (hscfg.conditions & ~HS_CFG_CONDITION_MASK)) {
			printf("ERR:Illegal conditions 0x%x\n",
			       hscfg.conditions);
			print_hscfg_usage();
			return UAP_FAILURE;
		}
		if (argc > 1) {
			hscfg.flags |= HS_CFG_FLAG_GPIO;
			hscfg.gpio = (t_u32)A2HEXDECIMAL(argv[1]);
			if (hscfg.gpio > 255) {
				printf("ERR:Illegal gpio 0x%x\n", hscfg.gpio);
				print_hscfg_usage();
				return UAP_FAILURE;
			}
		}
		if (argc > 2) {
			hscfg.flags |= HS_CFG_FLAG_GAP;
			hscfg.gap = (t_u32)A2HEXDECIMAL(argv[2]);
		}
	} else {
		hscfg.flags = HS_CFG_FLAG_GET;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&hscfg;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR:UAP_HS_CFG failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("Host sleep parameters:\n");
		printf("\tconditions=%d\n", (int)hscfg.conditions);
		printf("\tGPIO=%d\n", (int)hscfg.gpio);
		printf("\tgap=%d\n", (int)hscfg.gap);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for hssetpara command
 *
 *  @return         N/A
 */
void
print_hssetpara_usage(void)
{
	printf("\nUsage : hssetpara condition [[GPIO# [gap]]]");
	printf("\nOptions: condition : bit 0 = 1   -- broadcast data");
	printf("\n                     bit 1 = 1   -- unicast data");
	printf("\n                     bit 2 = 1   -- mac event");
	printf("\n                     bit 3 = 1   -- multicast packet");
	printf("\n                     bit 6 = 1   -- mgmt frame received");
	printf("\n         GPIO: the pin number (e.g. 0-7) of GPIO used to wakeup the host");
	printf("\n               or 0xff interface (e.g. SDIO) used to wakeup the host");
	printf("\n         gap: time between wakeup signal and wakeup event (in milliseconds)");
	printf("\n              or 0xff for special setting when GPIO is used to wakeup host\n");
	return;
}

/**
 *  @brief Creates host sleep parameter request and send to driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_hssetpara(int argc, char *argv[])
{
	int opt;
	int i = 0;
	ds_hs_cfg hscfg;
	struct ifreq ifr;
	t_s32 sockfd;

	if ((argc == 2) && strstr(argv[1], "-1"))
		strcpy(argv[1], "0xffff");

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_hssetpara_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&hscfg, 0, sizeof(hscfg));
	hscfg.subcmd = UAP_HS_SET_PARA;
	/* Check arguments */
	if ((argc < 1) || (argc > 3)) {
		printf("ERR:wrong arguments.\n");
		print_hssetpara_usage();
		return UAP_FAILURE;
	}
	for (i = 0; i < argc; i++) {
		if (IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE) {
			printf("ERR: Invalid argument %s\n", argv[i]);
			print_hssetpara_usage();
			return UAP_FAILURE;
		}
	}

	hscfg.flags |= HS_CFG_FLAG_SET | HS_CFG_FLAG_CONDITION;
	hscfg.conditions = (t_u32)A2HEXDECIMAL(argv[0]);
	if (hscfg.conditions >= 0xffff)
		hscfg.conditions = HS_CFG_CANCEL;
	if ((hscfg.conditions != HS_CFG_CANCEL) &&
	    (hscfg.conditions & ~HS_CFG_CONDITION_MASK)) {
		printf("ERR:Illegal conditions 0x%x\n", hscfg.conditions);
		print_hssetpara_usage();
		return UAP_FAILURE;
	}
	if (argc > 1) {
		hscfg.flags |= HS_CFG_FLAG_GPIO;
		hscfg.gpio = (t_u32)A2HEXDECIMAL(argv[1]);
		if (hscfg.gpio > 255) {
			printf("ERR:Illegal gpio 0x%x\n", hscfg.gpio);
			print_hssetpara_usage();
			return UAP_FAILURE;
		}
	}
	if (argc > 2) {
		hscfg.flags |= HS_CFG_FLAG_GAP;
		hscfg.gap = (t_u32)A2HEXDECIMAL(argv[2]);
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)&hscfg;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR:UAP_HS_SET_PARA failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	printf("Host sleep parameters setting successful!\n");
	printf("\tconditions=%d\n", (int)hscfg.conditions);
	printf("\tGPIO=%d\n", (int)hscfg.gpio);
	printf("\tgap=%d\n", (int)hscfg.gap);
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief get dfs repeater mode
 *
 *  @param mode    status of DFS repeater mode is returned here
 *  @return        UAP_SUCCESS/UAP_FAILURE
 */
int
uap_ioctl_dfs_repeater_mode(int *mode)
{
	struct ifreq ifr;
	dfs_repeater_mode param;
	t_s32 sockfd;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	param.action = ACTION_GET;
	param.subcmd = UAP_DFS_REPEATER_MODE;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		printf("ERR:UAP_DFS_REPEATER_MODE is not"
		       "supported by %s\n", dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}
	*mode = (int)param.mode;
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Get CAC timer status
 *
 *  @param mode    status of CAC timer is returned here
 *  @return        UAP_SUCCESS/UAP_FAILURE
 */
int
uap_ioctl_cac_timer_status(unsigned int *mode)
{
	struct ifreq ifr;
	cac_timer_status param;
	t_s32 sockfd;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	param.action = ACTION_GET;
	param.subcmd = UAP_CAC_TIMER_STATUS;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		printf("ERR:UAP_CAC_TIMER_STATUS is not"
		       "supported by %s\n", dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}
	*mode = (int)param.mode;
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Set/get power mode
 *
 *  @param pm      A pointer to ps_mgmt structure
 *  @param flag    flag for query
 *  @return        UAP_SUCCESS/UAP_FAILURE
 */
int
send_power_mode_ioctl(ps_mgmt * pm, int flag)
{
	struct ifreq ifr;
	t_s32 sockfd;
	t_u32 result = 0;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)pm;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_POWER_MODE, &ifr)) {
		memcpy((void *)&result, (void *)pm, sizeof(result));
		if (result == 1) {
			printf("ERR:Power mode needs to be disabled before modifying it\n");
		} else {
			perror("");
			printf("ERR:UAP_POWER_MODE is not supported by %s\n",
			       dev_name);
		}
		close(sockfd);
		return UAP_FAILURE;
	}
	if (flag) {
		/* Close socket */
		close(sockfd);
		return UAP_SUCCESS;
	}
	switch (pm->ps_mode) {
	case 0:
		printf("power mode = Disabled\n");
		break;
	case 1:
		printf("power mode = Periodic DTIM PS\n");
		break;
	case 2:
		printf("power mode = Inactivity based PS \n");
		break;
	}
	if (pm->flags & PS_FLAG_SLEEP_PARAM) {
		printf("Sleep param:\n");
		printf("\tctrl_bitmap=%d\n", (int)pm->sleep_param.ctrl_bitmap);
		printf("\tmin_sleep=%d us\n", (int)pm->sleep_param.min_sleep);
		printf("\tmax_sleep=%d us\n", (int)pm->sleep_param.max_sleep);
	}
	if (pm->flags & PS_FLAG_INACT_SLEEP_PARAM) {
		printf("Inactivity sleep param:\n");
		printf("\tinactivity_to=%d us\n",
		       (int)pm->inact_param.inactivity_to);
		printf("\tmin_awake=%d us\n", (int)pm->inact_param.min_awake);
		printf("\tmax_awake=%d us\n", (int)pm->inact_param.max_awake);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the pscfg command
 *
 *  @return         N/A
 */
void
print_pscfg_usage(void)
{
	printf("\nUsage : pscfg [MODE] [CTRL INACTTO MIN_SLEEP MAX_SLEEP MIN_AWAKE MAX_AWAKE]");
	printf("\nOptions: MODE :     0 - disable power mode");
	printf("\n                    1 - periodic DTIM power save mode");
	printf("\n                    2 - inactivity based power save mode");
	printf("\n   PS PARAMS:");
	printf("\n        CTRL:  0 - disable protection frame Tx before PS");
	printf("\n               1 - enable protection frame Tx before PS");
	printf("\n        INACTTO: Inactivity timeout in miroseconds");
	printf("\n        MIN_SLEEP: Minimum sleep duration in microseconds");
	printf("\n        MAX_SLEEP: Maximum sleep duration in miroseconds");
	printf("\n        MIN_AWAKE: Minimum awake duration in microseconds");
	printf("\n        MAX_AWAKE: Maximum awake duration in microseconds");
	printf("\n        MIN_AWAKE,MAX_AWAKE only valid for inactivity based power save mode");
	printf("\n        empty - get current power mode\n");
	return;
}

/**
 *  @brief Creates power mode request and send to driver
 *   and sends to the driver
 *
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_pscfg(int argc, char *argv[])
{
	int opt;
	ps_mgmt pm;
	int ret = UAP_SUCCESS;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_pscfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&pm, 0, sizeof(ps_mgmt));
	/* Check arguments */
	if ((argc > 7) ||
	    ((argc != 0) && (argc != 1) && (argc != 5) && (argc != 7))) {
		printf("ERR:wrong arguments.\n");
		print_pscfg_usage();
		return UAP_FAILURE;
	}

	if (argc) {
		if (send_power_mode_ioctl(&pm, 1) == UAP_FAILURE)
			return UAP_FAILURE;
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
		    (atoi(argv[0]) > 2)) {
			printf("ERR:Illegal power mode %s. Must be either '0' '1' or '2'.\n", argv[0]);
			print_pscfg_usage();
			return UAP_FAILURE;
		}
		pm.flags = PS_FLAG_PS_MODE;
		pm.ps_mode = atoi(argv[0]);
		if ((pm.ps_mode == PS_MODE_DISABLE) && (argc > 1)) {
			printf("ERR: Illegal parameter for disable power mode\n");
			print_pscfg_usage();
			return UAP_FAILURE;
		}
		if ((pm.ps_mode != PS_MODE_INACTIVITY) && (argc > 5)) {
			printf("ERR:Min awake period and Max awake period are valid only for inactivity based power save mode\n");
			print_pscfg_usage();
			return UAP_FAILURE;
		}
		if (argc >= 5) {
			if ((ISDIGIT(argv[1]) == 0) || (atoi(argv[1]) < 0) ||
			    (atoi(argv[1]) > 1)) {
				printf("ERR:Illegal ctrl bitmap = %s. Must be either '0' or '1'.\n", argv[1]);
				print_pscfg_usage();
				return UAP_FAILURE;
			}
			pm.flags |=
				PS_FLAG_SLEEP_PARAM | PS_FLAG_INACT_SLEEP_PARAM;
			pm.sleep_param.ctrl_bitmap = atoi(argv[1]);
			if ((ISDIGIT(argv[2]) == 0) || (ISDIGIT(argv[3]) == 0)
			    || (ISDIGIT(argv[4]) == 0)) {
				printf("ERR:Illegal parameter\n");
				print_pscfg_usage();
				return UAP_FAILURE;
			}
			pm.inact_param.inactivity_to = atoi(argv[2]);
			pm.sleep_param.min_sleep = atoi(argv[3]);
			pm.sleep_param.max_sleep = atoi(argv[4]);
			if (pm.sleep_param.min_sleep > pm.sleep_param.max_sleep) {
				printf("ERR: MIN_SLEEP value should be less than or equal to MAX_SLEEP\n");
				return UAP_FAILURE;
			}
			if (pm.sleep_param.min_sleep < PS_SLEEP_PARAM_MIN ||
			    ((pm.sleep_param.max_sleep > PS_SLEEP_PARAM_MAX) &&
			     pm.sleep_param.ctrl_bitmap)) {
				printf("ERR: Incorrect value of sleep period. Please check README\n");
				return UAP_FAILURE;
			}
			if (argc == 7) {
				if ((ISDIGIT(argv[5]) == 0) ||
				    (ISDIGIT(argv[6]) == 0)) {
					printf("ERR:Illegal parameter\n");
					print_pscfg_usage();
					return UAP_FAILURE;
				}
				pm.inact_param.min_awake = atoi(argv[5]);
				pm.inact_param.max_awake = atoi(argv[6]);
				if (pm.inact_param.min_awake >
				    pm.inact_param.max_awake) {
					printf("ERR: MIN_AWAKE value should be less than or equal to MAX_AWAKE\n");
					return UAP_FAILURE;
				}
				if (pm.inact_param.min_awake <
				    PS_AWAKE_PERIOD_MIN) {
					printf("ERR: Incorrect value of MIN_AWAKE period.\n");
					return UAP_FAILURE;
				}
			}
		}
	}
	ret = send_power_mode_ioctl(&pm, 0);
	return ret;
}

/**
 *  @brief Get bss status started/stopped
 *
 *  @param          current bss status
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
static int
get_bss_status(int *status)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_bss_status *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_FAILURE;

	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_bss_status);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_bss_status *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_BSS_STATUS_TLV_ID;
	tlv->length = 2;
	cmd_buf->action = ACTION_GET;

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	tlv->bss_status = uap_le16_to_cpu(tlv->bss_status);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_BSS_STATUS_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (tlv->bss_status == 0)
				*status = UAP_BSS_STOP;
			else
				*status = UAP_BSS_START;
		} else {
			printf("ERR:Could not get BSS status!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief start/stop/reset bss
 *
 *  @param mode     bss control mode
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
send_bss_ctl_ioctl(int mode)
{
	struct ifreq ifr;
	t_s32 sockfd;
	t_u32 data = (t_u32)mode;
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&data;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_BSS_CTRL, &ifr)) {
		printf("ERR:UAP_BSS_CTRL fail, result=%d\n", (int)data);
		switch (mode) {
		case UAP_BSS_START:
			if (data == BSS_FAILURE_START_INVAL)
				printf("ERR:Could not start BSS! Invalid BSS parameters.\n");
			else
				printf("ERR:Could not start BSS!\n");
			break;
		case UAP_BSS_STOP:
			printf("ERR:Could not stop BSS!\n");
			break;
		case UAP_BSS_RESET:
			printf("ERR:Could not reset system!\n");
			break;
		}
		close(sockfd);
		return UAP_FAILURE;
	}

	switch (mode) {
	case UAP_BSS_START:
		printf("BSS start successful!\n");
		break;
	case UAP_BSS_STOP:
		printf("BSS stop successful!\n");
		break;
	case UAP_BSS_RESET:
		printf("System reset successful!\n");
		break;
	}

	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the sys_reset command
 *
 *  @return         N/A
 */
void
print_sys_reset_usage(void)
{
	printf("\nUsage : sys_reset\n");
	return;
}

/**
 *  @brief Creates a sys_reset request and sends to the driver
 *
 *  Usage: "sys_reset"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_reset(int argc, char *argv[])
{
	int opt;
	int ret = UAP_SUCCESS;
	ps_mgmt pm;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_reset_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_sys_reset_usage();
		return UAP_FAILURE;
	}
	memset(&pm, 0, sizeof(ps_mgmt));
	pm.flags = PS_FLAG_PS_MODE;
	pm.ps_mode = PS_MODE_DISABLE;
	if (send_power_mode_ioctl(&pm, 0) == UAP_FAILURE)
		return UAP_FAILURE;
	ret = send_bss_ctl_ioctl(UAP_BSS_RESET);
	return ret;
}

/**
 *  @brief Show usage information for the bss_start command
 *
 *  $return         N/A
 */
void
print_bss_start_usage(void)
{
	printf("\nUsage : bss_start\n");
	return;
}

/**
 *  @brief Creates a BSS start request and sends to the driver
 *
 *   Usage: "bss_start"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_bss_start(int argc, char *argv[])
{
	int opt;
	t_u8 *buf = NULL;
	t_u16 buf_len = 0;
	int status = 0;
	int ret = UAP_SUCCESS;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_bss_start_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_bss_start_usage();
		return UAP_FAILURE;
	}

	if (get_bss_status(&status) != UAP_SUCCESS) {
		printf("ERR:Cannot get current bss status!\n");
		return UAP_FAILURE;
	}

	if (status == UAP_BSS_START) {
		printf("ERR: Could not start BSS! BSS already started!\n");
		return UAP_FAILURE;
	}

	/* Query BSS settings */

	/* Alloc buf for command */
	buf_len = sizeof(apcmdbuf_bss_configure) + sizeof(bss_config_t);
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset((char *)buf, 0, buf_len);

	/* Get all parametes first */
	if (get_bss_config(buf) == UAP_FAILURE) {
		printf("ERR:Reading current bss configuration\n");
		free(buf);
		return UAP_FAILURE;
	}

	ret = check_bss_config(buf + sizeof(apcmdbuf_bss_configure));

	if (ret == UAP_FAILURE) {
		printf("ERR: Wrong bss configuration!\n");
		goto done;
	}

	ret = send_bss_ctl_ioctl(UAP_BSS_START);

done:
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Show usage information for the bss_stop command
 *
 *  @return         N/A
 */
void
print_bss_stop_usage(void)
{
	printf("\nUsage : bss_stop\n");
	return;
}

/**
 *  @brief Creates a BSS stop request and sends to the driver
 *
 *   Usage: "bss_stop"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_bss_stop(int argc, char *argv[])
{
	int opt;
	int status = 0;
	int ret = UAP_SUCCESS;
	unsigned int cac_timer = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_bss_stop_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;		/* Check arguments */

	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_bss_stop_usage();
		return UAP_FAILURE;
	}
	if (get_bss_status(&status) != UAP_SUCCESS) {
		printf("ERR:Cannot get current bss status!\n");
		return UAP_FAILURE;
	}

	if ((status != UAP_BSS_STOP)
	    || ((uap_ioctl_cac_timer_status(&cac_timer) == UAP_SUCCESS)
		&& (cac_timer))
		)
		ret = send_bss_ctl_ioctl(UAP_BSS_STOP);
	else {
		printf("ERR: Could not stop BSS! BSS already stopped!\n");
		ret = UAP_FAILURE;
	}
	return ret;
}

/**
 *  @brief              Print skip cac usage
 *
 *  @return             N/A
 */
void
print_skip_cac_usage(void)
{
	printf("\nUsage : skip_cac [MODE]");
	printf("\nOptions: MODE : 0 - Disable skip CAC mode");
	printf("\n                1 - Enable skip CAC mode");
	printf("\n         empty - get skip CAC mode\n");
	return;
}

/**
 *  @brief Skip CAC for next immediate BSS_START
 *
 *   Usage: "skip_cac [1/0]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_skip_cac(int argc, char *argv[])
{
	int opt;
	skip_cac_para param;
	struct ifreq ifr;
	t_s32 sockfd;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_skip_cac_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&param, 0, sizeof(param));
	/* Check arguments */
	if (argc > 1) {
		printf("ERR:wrong arguments. Only support 1 argument\n");
		print_skip_cac_usage();
		return UAP_FAILURE;
	}
	param.subcmd = UAP_SKIP_CAC;
	if (argc) {
		if (argc == 1) {
			if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) ||
			    ((atoi(argv[0]) < 0) || (atoi(argv[0]) > 1))) {
				printf("ERR: Only Number values are allowed\n");
				print_skip_cac_usage();
				return UAP_FAILURE;
			}
		}
		param.action = 1;
		param.skip_cac = (t_u16)A2HEXDECIMAL(argv[0]);
	} else {
		param.action = 0;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		if (argc)
			printf("ERR:skip_cac set failed\n");
		else {
			perror("");
			printf("ERR:skip_cac get failed\n");
		}
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		if (param.skip_cac == 1) {
			printf("skip CAC mode: enabled\n");
		} else {
			printf("skip CAC mode: disabled\n");
		}
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the sta_list command
 *
 *  $return         N/A
 */
void
print_sta_list_usage(void)
{
	printf("\nUsage : sta_list\n");
	return;
}

/**
 *  @brief Creates a STA list request and sends to the driver
 *
 *   Usage: "sta_list"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sta_list(int argc, char *argv[])
{
	struct ifreq ifr;
	t_s32 sockfd;
	sta_list list;
	int i = 0;
	int opt;
	int rssi = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sta_list_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 0) {
		printf("ERR:Too many arguments.\n");
		print_sta_list_usage();
		return UAP_FAILURE;
	}
	memset(&list, 0, sizeof(sta_list));

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&list;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_GET_STA_LIST, &ifr)) {
		perror("");
		printf("ERR:UAP_GET_STA_LIST is not supported by %s\n",
		       dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}
	printf("Number of STA = %d\n\n", list.sta_count);

	for (i = 0; i < list.sta_count; i++) {
		printf("STA %d information:\n", i + 1);
		printf("=====================\n");
		printf("MAC Address: ");
		print_mac(list.info[i].mac_address);
		printf("\nPower mfg status: %s\n",
		       (list.info[i].power_mfg_status ==
			0) ? "active" : "power save");
		printf("Mode: %s\n",
		       (list.info[i].bandmode ==
			BAND_B) ? "11b," : (list.info[i].bandmode ==
					    BAND_G) ? "11g," : (list.info[i].
								bandmode ==
								BAND_A) ? "11a,"
		       : (list.info[i].bandmode ==
			  BAND_GN) ? "2.4G_11n," : (list.info[i].bandmode ==
						    BAND_AN) ? "5G_11n,"
		       : (list.info[i].bandmode ==
			  BAND_GAC) ? "2.4G_11ac," : (list.info[i].bandmode ==
						      BAND_AAC) ? "5G_11ac," :
		       "unkown");
	/** On some platform, s8 is same as unsigned char*/
		rssi = (int)list.info[i].rssi;
		if (rssi > 0x7f)
			rssi = -(256 - rssi);
		printf("Rssi : %d dBm\n\n", rssi);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the sta_deauth command
 *
 *  @return         N/A
 */
void
print_sta_deauth_usage(void)
{
	printf("\nUsage : sta_deauth <STA_MAC_ADDRESS>\n");
	return;
}

/**
 *  @brief Creates a STA deauth request and sends to the driver
 *
 *   Usage: "sta_deauth <STA_MAC_ADDRESS>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sta_deauth(int argc, char *argv[])
{
	APCMDBUF_STA_DEAUTH *cmd_buf = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_FAILURE;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sta_deauth_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 1) {
		printf("ERR:wrong arguments! Must provide STA_MAC_ADDRESS.\n");
		print_sta_deauth_usage();
		return UAP_FAILURE;
	}

	/* Initialize the command length */
	cmd_len = sizeof(APCMDBUF_STA_DEAUTH);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (APCMDBUF_STA_DEAUTH *)buffer;

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_STA_DEAUTH;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	if ((ret = mac2raw(argv[0], cmd_buf->sta_mac_address)) != UAP_SUCCESS) {
		printf("ERR: %s Address\n", ret == UAP_FAILURE ? "Invalid MAC" :
		       ret ==
		       UAP_RET_MAC_BROADCAST ? "Broadcast" : "Multicast");
		free(buffer);
		return UAP_FAILURE;
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code != (APCMD_STA_DEAUTH | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			free(buffer);
			return UAP_FAILURE;
		}

		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Deauthentication successful!\n");
		} else {
			printf("ERR:Deauthentication unsuccessful!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Show usage information for the sta_deauth_ext command
 *
 *  $return         N/A
 */
void
print_sta_deauth_ext_usage(void)
{
	printf("\nUsage : sta_deauth <STA_MAC_ADDRESS> <REASCON_CODE>\n");
	return;
}

/**
 *  @brief Creates a STA deauth request and sends to the driver
 *
 *   Usage: "sta_deauth <STA_MAC_ADDRESS><REASON_CODE>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sta_deauth_ext(int argc, char *argv[])
{
	int ret = UAP_SUCCESS;
	int opt;
	deauth_param param;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u32 result = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sta_deauth_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc != 2) {
		printf("ERR:wrong arguments! Must provide STA_MAC_ADDRESS, REASON_CODE.\n");
		print_sta_deauth_ext_usage();
		return UAP_FAILURE;
	}
	memset(&param, 0, sizeof(deauth_param));

	if ((ret = mac2raw(argv[0], param.mac_addr)) != UAP_SUCCESS) {
		printf("ERR: %s Address\n", ret == UAP_FAILURE ? "Invalid MAC" :
		       ret ==
		       UAP_RET_MAC_BROADCAST ? "Broadcast" : "Multicast");
		return UAP_FAILURE;
	}

	if ((IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE) ||
	    (atoi(argv[1]) > MAX_DEAUTH_REASON_CODE)) {
		printf("ERR: Invalid input for reason code\n");
		return UAP_FAILURE;
	}
	param.reason_code = (t_u16)A2HEXDECIMAL(argv[1]);
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_STA_DEAUTH, &ifr)) {
		memcpy((void *)&result, (void *)&param, sizeof(result));
		if (result == 1)
			printf("ERR:UAP_STA_DEAUTH fail\n");
		else
			perror("");
		close(sockfd);
		return UAP_FAILURE;
	}
	printf("Station deauth successful\n");
	/* Close socket */
	close(sockfd);
	return ret;
}

/**
 *  @brief Show usage information for the radioctrl command
 *
 *  $return         N/A
 */
void
print_radio_ctl_usage(void)
{
	printf("\nUsage : radioctrl [ 0 | 1 ]\n");
	return;
}

/**
 *  @brief Creates a Radio control request and sends to the driver
 *
 *   Usage: "radioctrl [0 | 1]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_radio_ctl(int argc, char *argv[])
{
	int opt;
	int param[2] = { 0, 0 };	/* action (Set/Get), Control (ON/OFF) */
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_radio_ctl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:wrong arguments! Only 1 or 0 arguments are supported.\n");
		print_radio_ctl_usage();
		return UAP_FAILURE;
	}
	if (argc && (is_input_valid(RADIOCONTROL, argc, argv) != UAP_SUCCESS)) {
		print_radio_ctl_usage();
		return UAP_FAILURE;
	}
	if (argc) {
		param[0] = ACTION_SET;
		param[1] = atoi(argv[0]);
	}

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_RADIO_CTL, &ifr)) {
		printf("ERR:UAP_RADIO_CTL fail\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (argc)
		printf("Radio setting successful\n");
	else
		printf("Radio is %s.\n", (param[1]) ? "on" : "off");
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the txratecfg command
 *
 *  @return         N/A
 */
void
print_txratecfg_usage(void)
{
	printf("\nUsage : txratecfg <datarate index> Index should be one of the following.\n");
	printf("\n  [l] is <format>");
	printf("\n  <format> - This parameter specifies the data rate format used in this command");
	printf("\n  0:    LG");
	printf("\n  1:    HT");
	printf("\n  2:    VHT");
	printf("\n  0xff: Auto");
	printf("\n");
	printf("\n  [m] is <index>");
	printf("\n  <index> - This parameter specifies the rate or MCS index");
	printf("\n  If <format> is 0 (LG),");
	printf("\n  0   1 Mbps");
	printf("\n  1   2 Mbps");
	printf("\n  2   5.5 Mbps");
	printf("\n  3   11 Mbps");
	printf("\n  4   6 Mbps");
	printf("\n  5   9 Mbps");
	printf("\n  6   12 Mbps");
	printf("\n  7   18 Mbps");
	printf("\n  8   24 Mbps");
	printf("\n  9   36 Mbps");
	printf("\n  10  48 Mbps");
	printf("\n  11  54 Mbps");
	printf("\n  If <format> is 1 (HT), ");
	printf("\n  0   MCS0");
	printf("\n  1   MCS1");
	printf("\n  2   MCS2");
	printf("\n  3   MCS3");
	printf("\n  4   MCS4");
	printf("\n  5   MCS5");
	printf("\n  6   MCS6");
	printf("\n  7   MCS7");
	printf("\n  8   MCS8");
	printf("\n  9   MCS9");
	printf("\n  10  MCS10");
	printf("\n  11  MCS11");
	printf("\n  12  MCS12");
	printf("\n  13  MCS13");
	printf("\n  14  MCS14");
	printf("\n  15  MCS15");
	printf("\n  32  MCS32");
	printf("\n  If <format> is 2 (VHT), ");
	printf("\n  0   MCS0");
	printf("\n  1   MCS1");
	printf("\n  2   MCS2");
	printf("\n  3   MCS3");
	printf("\n  4   MCS4");
	printf("\n  5   MCS5");
	printf("\n  6   MCS6");
	printf("\n  7   MCS7");
	printf("\n  8   MCS8");
	printf("\n  9   MCS9");
	printf("\n  [n] is <nss>");
	printf("\n  <nss> - This parameter specifies the NSS. It is valid only for VHT");
	printf("\n  If <format> is 2 (VHT), ");
	printf("\n  1   NSS1");
	printf("\n  2   NSS2");
	printf("\n");
	return;
}

/**
 *  @brief Creates a Tx Rate Config get request and sends to the driver
 *  @param rate_config Tx rate config struct
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
static int
get_tx_rate_cfg(tx_rate_cfg_t *rate_config)
{
	struct ifreq ifr;
	t_s32 sockfd;
	int ret = UAP_SUCCESS;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(rate_config, 0, sizeof(tx_rate_cfg_t));
	rate_config->subcmd = UAP_TX_RATE_CFG;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)rate_config;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		printf("ERR:UAP_IOCTL_CMD fail\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	/* Close socket */
	close(sockfd);
	return ret;
}

static char *rate_format[3] = { "LG", "HT", "VHT" };

static char *lg_rate[] = { "1 Mbps", "2 Mbps", "5.5 Mbps", "11 Mbps",
	"6 Mbps", "9 Mbps", "12 Mbps", "18 Mbps",
	"24 Mbps", "36 Mbps", "48 Mbps", "54 Mbps"
};

/**
 *  @brief Creates a Tx Rate Config request and sends to the driver
 *
 *   Usage: "txratecfg <datarate>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_tx_rate_cfg(int argc, char *argv[])
{
	int opt;
	int status = 0;
	tx_rate_cfg_t tx_rate_config;
	struct ifreq ifr;
	t_s32 sockfd;
	HTCap_t htcap;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_txratecfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 3) {
		printf("ERR:wrong arguments! Only 0~3 arguments are supported.\n");
		print_txratecfg_usage();
		return UAP_FAILURE;
	}
	if (argc && (is_input_valid(TXRATECFG, argc, argv) != UAP_SUCCESS)) {
		print_txratecfg_usage();
		return UAP_FAILURE;
	}
	memset(&tx_rate_config, 0, sizeof(tx_rate_cfg_t));
	tx_rate_config.subcmd = UAP_TX_RATE_CFG;
	if (argc) {
		tx_rate_config.action = ACTION_SET;
		tx_rate_config.rate_format = A2HEXDECIMAL(argv[0]);
		if (argc >= 2)
			tx_rate_config.rate = A2HEXDECIMAL(argv[1]);
		if (argc == 3)
			tx_rate_config.nss = A2HEXDECIMAL(argv[2]);
		tx_rate_config.user_data_cnt = argc;
		/* If bss is already started and uAP is in (short GI in 20 MHz + GF) mode, block MCS0-MCS7 rates */
		if (get_bss_status(&status) != UAP_SUCCESS) {
			printf("ERR:Cannot get current bss status!\n");
			return UAP_FAILURE;
		}
		if (UAP_SUCCESS == get_sys_cfg_11n(&htcap)) {
			if (htcap.supported_mcs_set[0] &&
			    (status == UAP_BSS_START)) {
				if (((tx_rate_config.rate >= 0) &&
				     (tx_rate_config.rate <= 7))
				    &&
				    (IS_11N_20MHZ_SHORTGI_ENABLED
				     (htcap.ht_cap_info) &&
				     IS_11N_GF_ENABLED(htcap.ht_cap_info))) {
					printf("ERR: Invalid rate for bss in (20MHz Short GI + Green Field) mode\n");
					return UAP_FAILURE;
				}
				if ((tx_rate_config.rate == 32) &&
				    (!(IS_11N_40MHZ_ENABLED
				       (htcap.ht_cap_info)))) {
					printf("ERR:uAP must be configured to operate in 40MHz if tx_rate is MCS32\n");
					return UAP_FAILURE;
				}
			}
		}
	}

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)&tx_rate_config;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		printf("ERR:UAP_IOCTL_CMD fail\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (argc) {
		printf("Transmit Rate setting successful\n");
	} else {
		/* GET operation */
		printf("Tx Rate Configuration: \n");
		/* format */
		if (tx_rate_config.rate_format == 0xFF) {
			printf("    Type:       0xFF (Auto)\n");
		} else if (tx_rate_config.rate_format <= 2) {
			printf("    Type:       %d (%s)\n",
			       tx_rate_config.rate_format,
			       rate_format[tx_rate_config.rate_format]);
			if (tx_rate_config.rate_format == 0)
				printf("    Rate Index: %d (%s)\n",
				       tx_rate_config.rate,
				       lg_rate[tx_rate_config.rate]);
			else if (tx_rate_config.rate_format >= 1)
				printf("    MCS Index:  %d\n",
				       (int)tx_rate_config.rate);
			if (tx_rate_config.rate_format == 2)
				printf("    NSS:        %d\n",
				       (int)tx_rate_config.nss);
		} else {
			printf("    Unknown rate format.\n");
		}
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the antcfg
 *   command
 *
 *  @return         N/A
 */
void
print_antcfg_usage(void)
{
	printf("\nUsage : antcfg [<TX_MODE> <RX_MODE>]\n");
	printf("\n         MODE    : 1       - Antenna A");
	printf("\n                   2       - Antenna B");
	printf("\n                   3       - Antenna A+B");
	printf("\n                   empty   - Get current antenna settings\n");
	return;
}

/**
 *  @brief Creates a RF Antenna Mode Config request and sends to the driver
 *
 *   Usage: "antcfg [MODE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_antcfg(int argc, char *argv[])
{
	int opt;
	ant_cfg_t antenna_config;
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_antcfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 2) {
		printf("ERR:wrong arguments!\n");
		print_antcfg_usage();
		return UAP_FAILURE;
	}
	if (argc) {
		if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 1) ||
		    (atoi(argv[0]) > 3)) {
			printf("ERR:Illegal ANTENNA parameter %s. Must be either '1', '2' or '3'.\n", argv[0]);
			print_antcfg_usage();
			return UAP_FAILURE;
		}
		if (argc == 2) {
			if ((ISDIGIT(argv[1]) == 0) || (atoi(argv[1]) < 1) ||
			    (atoi(argv[1]) > 3)) {
				printf("ERR:Illegal RX ANTENNA parameter %s. Must be either '1', '2' or '3'.\n", argv[1]);
				print_antcfg_usage();
				return UAP_FAILURE;
			}
		}
	}
	memset(&antenna_config, 0, sizeof(ant_cfg_t));
	antenna_config.subcmd = UAP_ANTENNA_CFG;
	if (argc) {
		antenna_config.action = ACTION_SET;
		if (argc == 1) {
			antenna_config.tx_mode = atoi(argv[0]);
			antenna_config.rx_mode = atoi(argv[0]);
		} else {
			antenna_config.tx_mode = atoi(argv[0]);
			antenna_config.rx_mode = atoi(argv[1]);
		}
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)&antenna_config;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		printf("ERR:UAP_IOCTL_CMD fail\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (argc) {
		printf("Antenna mode setting successful\n");
	} else {
		printf("TX Antenna mode is %d.\n", antenna_config.tx_mode);
		printf("RX Antenna mode is %d.\n", antenna_config.rx_mode);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the htstreamcfg
 *   command
 *
 *  $return         N/A
 */
void
print_htstreamcfg_usage(void)
{
	printf("\nUsage : htstreamcfg [<n>]\n");
	printf("\n        Where  <n>  ");
	printf("\n                 0x11: HT stream 1x1 mode");
	printf("\n                 0x22: HT stream 2x2 mode\n");
	return;
}

/**
 *  @brief Set/get HT stream configurations
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_htstreamcfg(int argc, char *argv[])
{
	int opt;
	htstream_cfg_t htstream_cfg;
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_htstreamcfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */

	memset(&htstream_cfg, 0, sizeof(htstream_cfg));
	if (argc == 0) {
		htstream_cfg.action = ACTION_GET;
	} else if (argc == 1) {
		if ((t_u32)A2HEXDECIMAL(argv[0]) != HT_STREAM_MODE_1X1
		    && (t_u32)A2HEXDECIMAL(argv[0]) != HT_STREAM_MODE_2X2) {
			printf("ERR:Invalid argument\n");
			return UAP_FAILURE;
		}
		htstream_cfg.action = ACTION_SET;
		htstream_cfg.stream_cfg = (t_u32)A2HEXDECIMAL(argv[0]);
	} else {
		print_htstreamcfg_usage();
		return UAP_FAILURE;
	}
	htstream_cfg.subcmd = UAP_HT_STREAM_CFG;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&htstream_cfg;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: HT STREAM configuration failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}

	/* Handle response */
	if (htstream_cfg.action == ACTION_GET) {
		if (htstream_cfg.stream_cfg == HT_STREAM_MODE_1X1)
			printf("HT stream is in 1x1 mode\n");
		else if (htstream_cfg.stream_cfg == HT_STREAM_MODE_2X2)
			printf("HT stream is in 2x2 mode\n");
		else
			printf("HT stream is unknown mode\n");
	}

	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief                Get 802.11 configuration
 *
 *  @param vhtcfg	   pointer to eth_priv_vhtcfg structure
 *
 *  @return           	   UAP_FAILURE or UAP_SUCCESS
 */

static int
get_802_11ac_cfg(struct eth_priv_vhtcfg *vhtcfg)
{
	t_u8 *buf = NULL;
	t_u8 *pos = NULL;
	mrvl_priv_cmd *cmd = NULL;
	struct ifreq ifr;
	t_s32 sockfd;

	buf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!buf) {
		printf("ERR: cannot allocate buffer for command payload \n");
		return UAP_FAILURE;
	}
	memset(buf, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	cmd = (mrvl_priv_cmd *)malloc(sizeof(mrvl_priv_cmd));
	if (!cmd) {
		printf("ERR: cannot allocate buffer for cmd \n");
		free(buf);
		return UAP_FAILURE;
	}

	/*prepare command: mlanutl uap0 vhtcfg 2 3 , GET operation */
	pos = buf;
	strncpy((char *)pos, CMD_NXP, strlen(CMD_NXP));
	pos += strlen(CMD_NXP);
	strncpy((char *)pos, "vhtcfg2 3", strlen("vhtcfg2 3"));

	/* fill up buffer */
	cmd->buf = buf;
	cmd->used_len = 0;
	cmd->total_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Perform IOCTL */
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)cmd;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		if (cmd)
			free(cmd);
		if (buf)
			free(buf);
		return UAP_FAILURE;
	}

	if (ioctl(sockfd, MRVLPRIVCMD, &ifr)) {
		if (cmd)
			free(cmd);
		if (buf)
			free(buf);
		close(sockfd);
		return UAP_FAILURE;
	}

	/* Process result */
	memcpy(vhtcfg, buf + 1, sizeof(struct eth_priv_vhtcfg));

	close(sockfd);
	free(cmd);
	free(buf);

	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the sys_config command
 *
 *  @return         N/A
 */
void
print_sys_config_usage(void)
{
	printf("\nUsage : sys_config [CONFIG_FILE]\n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("CONFIG_FILE is file contain all the Micro AP settings.\n");
	return;
}

/**
 *  @brief Show usage information for the rdeeprom command
 *
 *  @return         N/A
 */
void
print_apcmd_read_eeprom_usage(void)
{
	printf("\nUsage: rdeeprom <offset> <bytecount>\n");
	printf("    offset    : 0,4,8,..., multiple of 4\n");
	printf("    bytecount : 4-20, multiple of 4\n");
	return;
}

/**
 *  @brief Show protocol tlv
 *
 *  @param protocol Protocol number
 *
 *  @return         N/A
 */
void
print_protocol(t_u16 protocol)
{
	switch (protocol) {
	case 0:
	case PROTOCOL_NO_SECURITY:
		printf("PROTOCOL = No security\n");
		break;
	case PROTOCOL_STATIC_WEP:
		printf("PROTOCOL = Static WEP\n");
		break;
	case PROTOCOL_WPA:
		printf("PROTOCOL = WPA \n");
		break;
	case PROTOCOL_WPA2:
		printf("PROTOCOL = WPA2 \n");
		break;
	case PROTOCOL_WPA | PROTOCOL_WPA2:
		printf("PROTOCOL = WPA/WPA2 \n");
		break;
	default:
		printf("Unknown PROTOCOL: 0x%x \n", protocol);
		break;
	}
}

/**
 *  @brief Show wep tlv
 *
 *  @param tlv     Pointer to wep tlv
 *
 *  @return         N/A
 */
void
print_wep_key(tlvbuf_wep_key *tlv)
{
	int i;
	t_u16 tlv_len;

	tlv_len = *(t_u8 *)&tlv->length;
	tlv_len |= (*((t_u8 *)&tlv->length + 1) << 8);

	if (tlv_len <= 2) {
		printf("wrong wep_key tlv: length=%d\n", tlv_len);
		return;
	}
	printf("WEP KEY_%d = ", tlv->key_index);
	for (i = 0; i < tlv_len - 2; i++)
		printf("%02x ", tlv->key[i]);
	if (tlv->is_default)
		printf("\nDefault WEP Key = %d\n", tlv->key_index);
	else
		printf("\n");
}

/**
 *  @brief Parses a command line
 *
 *  @param line     The line to parse
 *  @param args     Pointer to the argument buffer to be filled in
 *  @return         Number of arguments in the line or EOF
 */
static int
parse_line(char *line, char *args[])
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
	for (i = 0; i < length; i++) {
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
 *  @brief      Parse function for a configuration line
 *
 *  @param s        Storage buffer for data
 *  @param size     Maximum size of data
 *  @param stream   File stream pointer
 *  @param line     Pointer to current line within the file
 *  @param _pos     Output string or NULL
 *  @return     String or NULL
 */
static char *
config_get_line(char *s, int size, FILE * stream, int *line, char **_pos)
{
	*_pos = mlan_config_get_line(stream, s, size, line);
	return *_pos;
}

/**
 *  @brief Read the profile and sends to the driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_sys_config_profile(int argc, char *argv[])
{
	FILE *config_file = NULL;
	char *line = NULL;
	int li = 0;
	char *pos = NULL;
	int arg_num = 0;
	char *args[30];
	int i;
	int is_ap_config = 0;
	int is_ap_mac_filter = 0;
	apcmdbuf_sys_configure *cmd_buf = NULL;
	HTCap_t htcap;
	int enable_11n = -1;
	t_u16 tlv_offset_11n = 0;
	t_u32 supported_mcs_set = 0;
	t_u8 *buffer = NULL;
	t_u8 *tmp_buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 tlv_len = 0;
	int keyindex = -1;
	int protocol = -1;
	int pwkcipher_wpa = -1;
	int pwkcipher_wpa2 = -1;
	int gwkcipher = -1;
	tlvbuf_sta_mac_addr_filter *filter_tlv = NULL;
	tlvbuf_channel_config *channel_band_tlv = NULL;
	int filter_mac_count = -1;
	int tx_data_rate = -1;
	int tx_beacon_rate = -1;
	int mcbc_data_rate = -1;
	t_u8 rate[MAX_RATES];
	int found = 0;
	char country_80211d[4];
	t_u8 state_80211d = 0;
	int chan_mode = 0;
	int band = 0;
	int band_flag = 0;
	int chan_number = 0;
	t_u16 max_sta_num_supported = 0;
	fw_info fw;
	struct eth_priv_vhtcfg vhtcfg = { 0 };
	int ret = UAP_SUCCESS;
	memset(rate, 0, MAX_RATES);
	/* Check if file exists */
	config_file = fopen(argv[0], "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return UAP_FAILURE;
	}
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		ret = UAP_FAILURE;
		goto done;
	}
	memset(line, 0, MAX_CONFIG_LINE);

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Received config line (%d) = %s\n",
			   li, line);
#endif
		arg_num = parse_line(line, args);
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Number of arguments = %d\n",
			   arg_num);
		for (i = 0; i < arg_num; i++) {
			uap_printf(MSG_DEBUG, "\tDBG:Argument %d. %s\n", i + 1,
				   args[i]);
		}
#endif
		/* Check for end of AP configurations */
		if (is_ap_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_ap_config = 0;
				if (tx_data_rate != -1) {
					if ((!rate[0]) && (tx_data_rate) &&
					    (is_tx_rate_valid
					     ((t_u8)tx_data_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Data Rate \n");
						ret = UAP_FAILURE;
						goto done;
					}
					if (rate[0] && tx_data_rate) {
						for (i = 0; rate[i] != 0; i++) {
							if ((rate[i] &
							     ~BASIC_RATE_SET_BIT)
							    == tx_data_rate) {
								found = 1;
								break;
							}
						}
						if (!found) {
							printf("ERR: Invalid Tx Data Rate \n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
				}
				if (tx_beacon_rate != -1) {
					if ((!rate[0]) && (tx_beacon_rate) &&
					    (is_tx_rate_valid
					     ((t_u8)tx_beacon_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Beacon Rate \n");
						ret = UAP_FAILURE;
						goto done;
					}
					if (rate[0] && tx_beacon_rate) {
						for (i = 0; rate[i] != 0; i++) {
							if ((rate[i] &
							     ~BASIC_RATE_SET_BIT)
							    == tx_beacon_rate) {
								found = 1;
								break;
							}
						}
						if (!found) {
							printf("ERR: Invalid Tx Beacon Rate \n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
					/* Append a new TLV */
					tlvbuf_tx_data_rate *tlv = NULL;
					tlv_len = sizeof(tlvbuf_tx_data_rate);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append tx beacon rate TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_tx_data_rate *)(buffer +
								      cmd_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_TX_BEACON_RATE_TLV_ID;
					tlv->length = 2;
					tlv->tx_data_rate = tx_beacon_rate;
					endian_convert_tlv_header_out(tlv);
					tlv->tx_data_rate =
						uap_cpu_to_le16(tlv->
								tx_data_rate);
				}
				if (mcbc_data_rate != -1) {
					if ((!rate[0]) && (mcbc_data_rate) &&
					    (is_mcbc_rate_valid
					     ((t_u8)mcbc_data_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Data Rate \n");
						ret = UAP_FAILURE;
						goto done;
					}
					if (rate[0] && mcbc_data_rate) {
						for (i = 0; rate[i] != 0; i++) {
							if (rate[i] &
							    BASIC_RATE_SET_BIT)
							{
								if ((rate[i] &
								     ~BASIC_RATE_SET_BIT)
								    ==
								    mcbc_data_rate)
								{
									found = 1;
									break;
								}
							}
						}
						if (!found) {
							printf("ERR: Invalid MCBC Data Rate \n");
							ret = UAP_FAILURE;
							goto done;
						}
					}

					/* Append a new TLV */
					tlvbuf_mcbc_data_rate *tlv = NULL;
					tlv_len = sizeof(tlvbuf_mcbc_data_rate);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append tx data rate TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_mcbc_data_rate *)(buffer +
									cmd_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_MCBC_DATA_RATE_TLV_ID;
					tlv->length = 2;
					tlv->mcbc_datarate = mcbc_data_rate;
					endian_convert_tlv_header_out(tlv);
					tlv->mcbc_datarate =
						uap_cpu_to_le16(tlv->
								mcbc_datarate);
				}
				if ((protocol == PROTOCOL_STATIC_WEP) &&
				    (enable_11n == 1)) {
					printf("ERR:WEP cannot be used when AP operates in 802.11n mode.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if ((protocol == PROTOCOL_WPA2_MIXED) &&
				    ((pwkcipher_wpa < 0) ||
				     (pwkcipher_wpa2 < 0))) {
					printf("ERR:Both PwkCipherWPA and PwkCipherWPA2 should be defined for Mixed mode.\n");
					ret = UAP_FAILURE;
					goto done;
				}

				if (((pwkcipher_wpa >= 0) ||
				     (pwkcipher_wpa2 >= 0)) &&
				    (gwkcipher >= 0)) {
					if ((protocol == PROTOCOL_WPA) ||
					    (protocol == PROTOCOL_WPA2_MIXED)) {
						if (enable_11n != -1) {
							if (is_cipher_valid_with_11n(pwkcipher_wpa, gwkcipher, protocol, enable_11n) != UAP_SUCCESS) {
								printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
								ret = UAP_FAILURE;
								goto done;
							}
						} else if
							(is_cipher_valid_with_proto
							 (pwkcipher_wpa,
							  gwkcipher,
							  protocol) !=
							 UAP_SUCCESS) {
							printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
					if ((protocol == PROTOCOL_WPA2) ||
					    (protocol == PROTOCOL_WPA2_MIXED)
						) {
						if (enable_11n != -1) {
							if (is_cipher_valid_with_11n(pwkcipher_wpa2, gwkcipher, protocol, enable_11n) != UAP_SUCCESS) {
								printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
								ret = UAP_FAILURE;
								goto done;
							}
						} else if
							(is_cipher_valid_with_proto
							 (pwkcipher_wpa2,
							  gwkcipher,
							  protocol) !=
							 UAP_SUCCESS) {
							printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
				}

				if (0 == get_fw_info(&fw)) {
					/*check whether support 802.11AC through BAND_AAC bit */
					if (fw.fw_bands & BAND_AAC) {
						ret = get_802_11ac_cfg(&vhtcfg);
						if (ret != UAP_SUCCESS)
							goto done;
						if (enable_11n != -1) {
							/* Note: When 11AC is disabled, FW sets vht_rx_mcs to 0xffff */
							if ((vhtcfg.
							     vht_rx_mcs !=
							     0xffff) &&
							    (!enable_11n)) {
								printf("ERR: 11n must be enabled when AP operates in 11ac mode. \n");
								ret = UAP_FAILURE;
								goto done;
							}
						}
					} else
						printf("No support 802 11AC.\n");
				} else {
					printf("ERR: get_fw_info fail\n");
					ret = UAP_FAILURE;
					goto done;
				}

				if (protocol != -1) {
					tlvbuf_protocol *tlv = NULL;
					/* Append a new TLV */
					tlv_len = sizeof(tlvbuf_protocol);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append protocol TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_protocol *)(buffer +
								  cmd_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_PROTOCOL_TLV_ID;
					tlv->length = 2;
					tlv->protocol = protocol;
					endian_convert_tlv_header_out(tlv);
					tlv->protocol =
						uap_cpu_to_le16(tlv->protocol);
					if (protocol &
					    (PROTOCOL_WPA | PROTOCOL_WPA2)) {
						tlvbuf_akmp *tlv = NULL;
						/* Append a new TLV */
						tlv_len = sizeof(tlvbuf_akmp);
						tmp_buffer =
							realloc(buffer,
								cmd_len +
								tlv_len);
						if (!tmp_buffer) {
							printf("ERR:Cannot append AKMP TLV!\n");
							ret = UAP_FAILURE;
							goto done;
						} else {
							buffer = tmp_buffer;
							tmp_buffer = NULL;
						}
						cmd_buf =
							(apcmdbuf_sys_configure
							 *)buffer;
						tlv = (tlvbuf_akmp *)(buffer +
								      cmd_len);
						cmd_len += tlv_len;
						/* Set TLV fields */
						tlv->tag = MRVL_AKMP_TLV_ID;
						tlv->length = 4;	/* sizeof(tlvbuf_akmp) - TLVHEADER */
						{
							tlv->key_mgmt =
								KEY_MGMT_PSK;
						}
						endian_convert_tlv_header_out
							(tlv);
						tlv->key_mgmt =
							uap_cpu_to_le16(tlv->
									key_mgmt);
						tlv->key_mgmt_operation = 0;
					}
				}
				if (pwkcipher_wpa >= 0) {
					tlvbuf_pwk_cipher *tlv = NULL;
					/* Append a new TLV */
					tlv_len = sizeof(tlvbuf_pwk_cipher);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append cipher TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_pwk_cipher *)(buffer +
								    cmd_len);
					memset(tlv, 0, tlv_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_CIPHER_PWK_TLV_ID;
					tlv->length =
						sizeof(tlvbuf_pwk_cipher) -
						TLVHEADER_LEN;
					tlv->pairwise_cipher = pwkcipher_wpa;
					tlv->protocol = PROTOCOL_WPA;
					endian_convert_tlv_header_out(tlv);
					tlv->protocol =
						uap_cpu_to_le16(tlv->protocol);
				}

				if (pwkcipher_wpa2 >= 0) {
					tlvbuf_pwk_cipher *tlv = NULL;
					/* Append a new TLV */
					tlv_len = sizeof(tlvbuf_pwk_cipher);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append cipher TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_pwk_cipher *)(buffer +
								    cmd_len);
					memset(tlv, 0, tlv_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_CIPHER_PWK_TLV_ID;
					tlv->length =
						sizeof(tlvbuf_pwk_cipher) -
						TLVHEADER_LEN;
					tlv->pairwise_cipher = pwkcipher_wpa2;
					tlv->protocol = PROTOCOL_WPA2;
					endian_convert_tlv_header_out(tlv);
					tlv->protocol =
						uap_cpu_to_le16(tlv->protocol);
				}

				if (gwkcipher >= 0) {
					tlvbuf_gwk_cipher *tlv = NULL;
					/* Append a new TLV */
					tlv_len = sizeof(tlvbuf_gwk_cipher);
					tmp_buffer =
						realloc(buffer,
							cmd_len + tlv_len);
					if (!tmp_buffer) {
						printf("ERR:Cannot append cipher TLV!\n");
						ret = UAP_FAILURE;
						goto done;
					} else {
						buffer = tmp_buffer;
						tmp_buffer = NULL;
					}
					cmd_buf =
						(apcmdbuf_sys_configure *)
						buffer;
					tlv = (tlvbuf_gwk_cipher *)(buffer +
								    cmd_len);
					memset(tlv, 0, tlv_len);
					cmd_len += tlv_len;
					/* Set TLV fields */
					tlv->tag = MRVL_CIPHER_GWK_TLV_ID;
					tlv->length =
						sizeof(tlvbuf_gwk_cipher) -
						TLVHEADER_LEN;
					tlv->group_cipher = gwkcipher;
					endian_convert_tlv_header_out(tlv);
				}

				cmd_buf->size = cmd_len;
				/* Send collective command */
				if (uap_ioctl
				    ((t_u8 *)cmd_buf, &cmd_len,
				     cmd_len) == UAP_SUCCESS) {
					if (cmd_buf->result != CMD_SUCCESS) {
						printf("ERR: Failed to set the configuration!\n");
						ret = UAP_FAILURE;
						goto done;
					}
				} else {
					printf("ERR: Command sending failed!\n");
					ret = UAP_FAILURE;
					goto done;
				}
				cmd_len = 0;
				if (buffer) {
					free(buffer);
					buffer = NULL;
				}
				continue;
			}
		}

		/* Check for beginning of AP configurations */
		if (strcmp(args[0], "ap_config") == 0) {
			is_ap_config = 1;
			cmd_len = sizeof(apcmdbuf_sys_configure);
			if (buffer) {
				free(buffer);
				buffer = NULL;
			}
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				ret = UAP_FAILURE;
				goto done;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
			cmd_buf->size = cmd_len;
			cmd_buf->seq_num = 0;
			cmd_buf->result = 0;
			cmd_buf->action = ACTION_SET;
			continue;
		}

		/* Check for end of AP MAC address filter configurations */
		if (is_ap_mac_filter == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_ap_mac_filter = 0;
				if (filter_tlv->count != filter_mac_count) {
					printf("ERR:Number of MAC address provided does not match 'Count'\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (filter_tlv->count) {
					filter_tlv->length =
						(filter_tlv->count * ETH_ALEN) +
						2;
					cmd_len -=
						(MAX_MAC_ONESHOT_FILTER -
						 filter_mac_count) * ETH_ALEN;
				} else {
					filter_tlv->length =
						(MAX_MAC_ONESHOT_FILTER *
						 ETH_ALEN) + 2;
					memset(filter_tlv->mac_address, 0,
					       MAX_MAC_ONESHOT_FILTER *
					       ETH_ALEN);
				}
				cmd_buf->size = cmd_len;
				endian_convert_tlv_header_out(filter_tlv);
				if (uap_ioctl
				    ((t_u8 *)cmd_buf, &cmd_len,
				     cmd_len) == UAP_SUCCESS) {
					if (cmd_buf->result != CMD_SUCCESS) {
						printf("ERR: Failed to set the configuration!\n");
						ret = UAP_FAILURE;
						goto done;
					}
				} else {
					printf("ERR: Command sending failed!\n");
					ret = UAP_FAILURE;
					goto done;
				}

				cmd_len = 0;
				if (buffer) {
					free(buffer);
					buffer = NULL;
				}
				continue;
			}
		}

		if (strcmp(args[0], "11d_enable") == 0) {
			if (IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE) {
				printf("ERR: valid input for state are 0 or 1\n");
				ret = UAP_FAILURE;
				goto done;
			}
			state_80211d = (t_u8)A2HEXDECIMAL(args[1]);

			if ((state_80211d != 0) && (state_80211d != 1)) {
				printf("ERR: valid input for state are 0 or 1 \n");
				ret = UAP_FAILURE;
				goto done;
			}
			if (sg_snmp_mib
			    (ACTION_SET, OID_80211D_ENABLE,
			     sizeof(state_80211d), &state_80211d)
			    == UAP_FAILURE) {
				ret = UAP_FAILURE;
				goto done;
			}
		}

		if (strcmp(args[0], "country") == 0) {
			apcmdbuf_cfg_80211d *cmd_buf = NULL;
			ieeetypes_subband_set_t sub_bands[MAX_SUB_BANDS];
			t_u8 no_of_sub_band = 0;
			t_u16 buf_len;
			t_u16 cmdlen;
			t_u8 *buf = NULL;

			if ((strlen(args[1]) > 3) || (strlen(args[1]) == 0)) {
				printf("In-correct country input\n");
				ret = UAP_FAILURE;
				goto done;
			}
			strncpy(country_80211d, args[1],
				sizeof(country_80211d) - 1);
			for (i = 0; (unsigned int)i < strlen(country_80211d);
			     i++) {
				if ((country_80211d[i] < 'A') ||
				    (country_80211d[i] > 'z')) {
					printf("Invalid Country Code\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (country_80211d[i] > 'Z')
					country_80211d[i] =
						country_80211d[i] - 'a' + 'A';
			}
			no_of_sub_band =
				parse_domain_file(country_80211d, band,
						  sub_bands, NULL);
			if (no_of_sub_band == UAP_FAILURE) {
				printf("Parsing Failed\n");
				ret = UAP_FAILURE;
				goto done;
			}
			buf_len = sizeof(apcmdbuf_cfg_80211d);
			buf_len +=
				no_of_sub_band *
				sizeof(ieeetypes_subband_set_t);
			buf = (t_u8 *)malloc(buf_len);
			if (!buf) {
				printf("ERR:Cannot allocate buffer from command!\n");
				ret = UAP_FAILURE;
				goto done;
			}
			memset(buf, 0, buf_len);
			cmd_buf = (apcmdbuf_cfg_80211d *)buf;
			cmdlen = buf_len;
			cmd_buf->size = cmdlen - BUF_HEADER_SIZE;
			cmd_buf->result = 0;
			cmd_buf->seq_num = 0;
			cmd_buf->action = uap_cpu_to_le16(ACTION_SET);
			cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;
			cmd_buf->domain.tag = uap_cpu_to_le16(TLV_TYPE_DOMAIN);
			cmd_buf->domain.length =
				uap_cpu_to_le16(sizeof(domain_param_t)
						- BUF_HEADER_SIZE +
						(no_of_sub_band *
						 sizeof
						 (ieeetypes_subband_set_t)));

			memset(cmd_buf->domain.country_code, ' ',
			       sizeof(cmd_buf->domain.country_code));
			memcpy(cmd_buf->domain.country_code, country_80211d,
			       strlen(country_80211d));
			memcpy(cmd_buf->domain.subband, sub_bands,
			       no_of_sub_band *
			       sizeof(ieeetypes_subband_set_t));

			/* Send the command */
			if (uap_ioctl((t_u8 *)cmd_buf, &cmdlen, cmdlen) ==
			    UAP_SUCCESS) {
				if (cmd_buf->result != CMD_SUCCESS) {
					printf("ERR: Failed to set the configuration!\n");
					ret = UAP_FAILURE;
					goto done;
				}
			} else {
				printf("ERR: Command sending failed!\n");
				ret = UAP_FAILURE;
				goto done;
			}

			if (buf)
				free(buf);
		}

		/* Check for beginning of AP MAC address filter configurations */
		if (strcmp(args[0], "ap_mac_filter") == 0) {
			is_ap_mac_filter = 1;
			cmd_len =
				sizeof(apcmdbuf_sys_configure) +
				sizeof(tlvbuf_sta_mac_addr_filter) +
				(MAX_MAC_ONESHOT_FILTER * ETH_ALEN);
			if (buffer) {
				free(buffer);
				buffer = NULL;
			}
			buffer = (t_u8 *)malloc(cmd_len);
			if (!buffer) {
				printf("ERR:Cannot allocate memory!\n");
				ret = UAP_FAILURE;
				goto done;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
			cmd_buf->size = cmd_len;
			cmd_buf->seq_num = 0;
			cmd_buf->result = 0;
			cmd_buf->action = ACTION_SET;
			filter_tlv =
				(tlvbuf_sta_mac_addr_filter *)(buffer +
							       sizeof
							       (apcmdbuf_sys_configure));
			filter_tlv->tag = MRVL_STA_MAC_ADDR_FILTER_TLV_ID;
			filter_tlv->length = 2;
			filter_tlv->count = 0;
			filter_mac_count = 0;
			continue;
		}
		if ((strcmp(args[0], "FilterMode") == 0) && is_ap_mac_filter) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0) ||
			    (atoi(args[1]) > 2)) {
				printf("ERR:Illegal FilterMode paramter %d. Must be either '0', '1', or '2'.\n", atoi(args[1]));
				ret = UAP_FAILURE;
				goto done;
			}
			filter_tlv->filter_mode = atoi(args[1]);
			continue;
		}
		if ((strcmp(args[0], "Count") == 0) && is_ap_mac_filter) {
			filter_tlv->count = atoi(args[1]);
			if ((ISDIGIT(args[1]) == 0) ||
			    (filter_tlv->count > MAX_MAC_ONESHOT_FILTER)) {
				printf("ERR: Illegal Count parameter.\n");
				ret = UAP_FAILURE;
				goto done;
			}
		}
		if ((strncmp(args[0], "mac_", 4) == 0) && is_ap_mac_filter) {
			if (filter_mac_count < MAX_MAC_ONESHOT_FILTER) {
				if (mac2raw
				    (args[1],
				     &filter_tlv->mac_address[filter_mac_count *
							      ETH_ALEN]) !=
				    UAP_SUCCESS) {
					printf("ERR: Invalid MAC address %s \n",
					       args[1]);
					ret = UAP_FAILURE;
					goto done;
				}
				filter_mac_count++;
			} else {
				printf("ERR: Filter table can not have more than %d MAC addresses\n", MAX_MAC_ONESHOT_FILTER);
				ret = UAP_FAILURE;
				goto done;
			}
		}

		if (strcmp(args[0], "SSID") == 0) {
			if (arg_num == 1) {
				printf("ERR:SSID field is blank!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				tlvbuf_ssid *tlv = NULL;
				if (args[1][0] == '"') {
					args[1]++;
				}
				if (args[1][strlen(args[1]) - 1] == '"') {
					args[1][strlen(args[1]) - 1] = '\0';
				}
				if ((strlen(args[1]) > MAX_SSID_LENGTH) ||
				    (strlen(args[1]) == 0)) {
					printf("ERR:SSID length out of range (%d to %d).\n", MIN_SSID_LENGTH, MAX_SSID_LENGTH);
					ret = UAP_FAILURE;
					goto done;
				}
				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_ssid) + strlen(args[1]);
				tmp_buffer = realloc(buffer, cmd_len + tlv_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot realloc SSID TLV!\n");
					ret = UAP_FAILURE;
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				cmd_buf = (apcmdbuf_sys_configure *)buffer;
				tlv = (tlvbuf_ssid *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				tlv->tag = MRVL_SSID_TLV_ID;
				tlv->length = strlen(args[1]);
				memcpy(tlv->ssid, args[1], tlv->length);
				endian_convert_tlv_header_out(tlv);
			}
		}
		if (strcmp(args[0], "BeaconPeriod") == 0) {
			if (is_input_valid(BEACONPERIOD, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_beacon_period *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_beacon_period);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot realloc beacon period TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_beacon_period *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_BEACON_PERIOD_TLV_ID;
			tlv->length = 2;
			tlv->beacon_period_ms = (t_u16)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->beacon_period_ms =
				uap_cpu_to_le16(tlv->beacon_period_ms);
		}
		if (strcmp(args[0], "ChanList") == 0) {
			if (is_input_valid(SCANCHANNELS, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}

			tlvbuf_channel_list *tlv = NULL;
			channel_list *pchan_list = NULL;
			/* Append a new TLV */
			tlv_len =
				sizeof(tlvbuf_channel_list) +
				((arg_num - 1) * sizeof(channel_list));
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append channel list TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_channel_list *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_CHANNELLIST_TLV_ID;
			tlv->length = sizeof(channel_list) * (arg_num - 1);
			pchan_list = (channel_list *) tlv->chan_list;
			for (i = 0; i < (arg_num - 1); i++) {
				band_flag = -1;
				sscanf(args[i + 1], "%d.%d", &chan_number,
				       &band_flag);
				pchan_list->chan_number = chan_number;
				pchan_list->bandcfg.chanBand = BAND_2GHZ;
				if (((band_flag != -1) && (band_flag)) ||
				    (chan_number > MAX_CHANNELS_BG)) {
					pchan_list->bandcfg.chanBand =
						BAND_5GHZ;
				}
				pchan_list++;
			}
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "Channel") == 0) {
			if (is_input_valid(CHANNEL, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_channel_config *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_channel_config);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append channel TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_channel_config *)(buffer + cmd_len);
			channel_band_tlv = tlv;
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_CHANNELCONFIG_TLV_ID;
			tlv->length =
				sizeof(tlvbuf_channel_config) - TLVHEADER_LEN;
			tlv->chan_number = (t_u8)atoi(args[1]);
			if (tlv->chan_number > MAX_CHANNELS_BG)
				band = BAND_A;
			else
				band = BAND_B | BAND_G;
			if ((arg_num - 1) == 2) {
				chan_mode = atoi(args[2]);
				memset(&(tlv->bandcfg), 0,
				       sizeof(tlv->bandcfg));
				if (chan_mode & BITMAP_ACS_MODE) {
					int mode;

					if (uap_ioctl_dfs_repeater_mode(&mode)
					    == UAP_SUCCESS) {
						if (mode) {
							printf("ERR: ACS in DFS Repeater mode" " is not allowed\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
					tlv->bandcfg.scanMode = SCAN_MODE_ACS;
				}
				if (chan_mode & BITMAP_CHANNEL_ABOVE)
					tlv->bandcfg.chan2Offset =
						SEC_CHAN_ABOVE;
				if (chan_mode & BITMAP_CHANNEL_BELOW)
					tlv->bandcfg.chan2Offset =
						SEC_CHAN_BELOW;
			} else
				memset(&(tlv->bandcfg), 0,
				       sizeof(tlv->bandcfg));
			if (tlv->chan_number > MAX_CHANNELS_BG) {
				tlv->bandcfg.chanBand = BAND_5GHZ;
			}
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "Band") == 0) {
			if (is_input_valid(BAND, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			if (channel_band_tlv == NULL) {
				printf("ERR: Channel parameter should be specified before Band\n");
				ret = UAP_FAILURE;
				goto done;
			}
			/* If band is provided, clear previous value of band */
			channel_band_tlv->bandcfg.chanBand = BAND_2GHZ;
			if (atoi(args[1]) == 0) {
				band = BAND_B | BAND_G;
			} else {
				channel_band_tlv->bandcfg.chanBand = BAND_5GHZ;
				band = BAND_A;
			}
		}
		if (strcmp(args[0], "AP_MAC") == 0) {
			tlvbuf_ap_mac_address *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_ap_mac_address);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append ap_mac TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_ap_mac_address *)(buffer + cmd_len);
			cmd_len += tlv_len;
			cmd_buf->action = ACTION_SET;
			tlv->tag = MRVL_AP_MAC_ADDRESS_TLV_ID;
			tlv->length = ETH_ALEN;
			if ((ret =
			     mac2raw(args[1],
				     tlv->ap_mac_addr)) != UAP_SUCCESS) {
				printf("ERR: %s Address \n",
				       ret ==
				       UAP_FAILURE ? "Invalid MAC" : ret ==
				       UAP_RET_MAC_BROADCAST ? "Broadcast" :
				       "Multicast");
				ret = UAP_FAILURE;
				goto done;
			}
			endian_convert_tlv_header_out(tlv);
		}

		if (strcmp(args[0], "Rate") == 0) {
			if (is_input_valid(RATE, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				printf("ERR: Invalid Rate input\n");
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_rates *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_rates) + arg_num - 1;
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append rates TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_rates *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_RATES_TLV_ID;
			tlv->length = arg_num - 1;
			for (i = 0; i < tlv->length; i++) {
				rate[i] = tlv->operational_rates[i] =
					(t_u8)A2HEXDECIMAL(args[i + 1]);
			}
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "TxPowerLevel") == 0) {
			if (is_input_valid(TXPOWER, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				printf("ERR:Invalid TxPowerLevel \n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				tlvbuf_tx_power *tlv = NULL;
				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_tx_power);
				tmp_buffer = realloc(buffer, cmd_len + tlv_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot append tx power level TLV!\n");
					ret = UAP_FAILURE;
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				cmd_buf = (apcmdbuf_sys_configure *)buffer;
				tlv = (tlvbuf_tx_power *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				tlv->tag = MRVL_TX_POWER_TLV_ID;
				tlv->length = 1;
				tlv->tx_power_dbm = (t_u8)atoi(args[1]);
				endian_convert_tlv_header_out(tlv);
			}
		}
		if (strcmp(args[0], "BroadcastSSID") == 0) {
			if (is_input_valid(BROADCASTSSID, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_bcast_ssid_ctl *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_bcast_ssid_ctl);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append SSID broadcast control TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_bcast_ssid_ctl *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_BCAST_SSID_CTL_TLV_ID;
			tlv->length = 1;
			tlv->bcast_ssid_ctl = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "RTSThreshold") == 0) {
			if (is_input_valid(RTSTHRESH, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_rts_threshold *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_rts_threshold);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append RTS threshold TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_rts_threshold *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_RTS_THRESHOLD_TLV_ID;
			tlv->length = 2;
			tlv->rts_threshold = (t_u16)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->rts_threshold =
				uap_cpu_to_le16(tlv->rts_threshold);
		}
		if (strcmp(args[0], "FragThreshold") == 0) {
			if (is_input_valid(FRAGTHRESH, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_frag_threshold *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_frag_threshold);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append Fragmentation threshold TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_frag_threshold *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_FRAG_THRESHOLD_TLV_ID;
			tlv->length = 2;
			tlv->frag_threshold = (t_u16)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->frag_threshold =
				uap_cpu_to_le16(tlv->frag_threshold);
		}
		if (strcmp(args[0], "DTIMPeriod") == 0) {
			if (is_input_valid(DTIMPERIOD, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_dtim_period *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_dtim_period);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append DTIM period TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_dtim_period *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_DTIM_PERIOD_TLV_ID;
			tlv->length = 1;
			tlv->dtim_period = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "RSNReplayProtection") == 0) {
			if (is_input_valid(RSNREPLAYPROT, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_rsn_replay_prot *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_rsn_replay_prot);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append RSN replay protection TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_rsn_replay_prot *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_RSN_REPLAY_PROT_TLV_ID;
			tlv->length = 1;
			tlv->rsn_replay_prot = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "TxBeaconRate") == 0) {
			if (is_input_valid(TXBEACONRATE, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tx_beacon_rate = (t_u16)A2HEXDECIMAL(args[1]);
		}
		if (strcmp(args[0], "MCBCdataRate") == 0) {
			if (is_input_valid(MCBCDATARATE, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			mcbc_data_rate = (t_u16)A2HEXDECIMAL(args[1]);
		}
		if (strcmp(args[0], "PktFwdCtl") == 0) {
			if (is_input_valid(PKTFWD, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_pkt_fwd_ctl *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_pkt_fwd_ctl);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append packet forwarding control TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_pkt_fwd_ctl *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_PKT_FWD_CTL_TLV_ID;
			tlv->length = 1;
			tlv->pkt_fwd_ctl = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "StaAgeoutTimer") == 0) {
			if (is_input_valid
			    (STAAGEOUTTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_sta_ageout_timer *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_sta_ageout_timer);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append STA ageout timer TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_sta_ageout_timer *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_STA_AGEOUT_TIMER_TLV_ID;
			tlv->length = 4;
			tlv->sta_ageout_timer_ms = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->sta_ageout_timer_ms =
				uap_cpu_to_le32(tlv->sta_ageout_timer_ms);
		}
		if (strcmp(args[0], "PSStaAgeoutTimer") == 0) {
			if (is_input_valid
			    (PSSTAAGEOUTTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_ps_sta_ageout_timer *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_ps_sta_ageout_timer);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append PS STA ageout timer TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_ps_sta_ageout_timer *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_PS_STA_AGEOUT_TIMER_TLV_ID;
			tlv->length = 4;
			tlv->ps_sta_ageout_timer_ms = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->ps_sta_ageout_timer_ms =
				uap_cpu_to_le32(tlv->ps_sta_ageout_timer_ms);
		}
		if (strcmp(args[0], "AuthMode") == 0) {
			if (is_input_valid(AUTHMODE, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_auth_mode *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_auth_mode);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append auth mode TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_auth_mode *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_AUTH_TLV_ID;
			tlv->length = 1;
			tlv->auth_mode = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "KeyIndex") == 0) {
			if (arg_num == 1) {
				printf("KeyIndex is blank!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal KeyIndex parameter. Must be either '0', '1', '2', or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				keyindex = atoi(args[1]);
				if ((keyindex < 0) || (keyindex > 3)) {
					printf("ERR:Illegal KeyIndex parameter. Must be either '0', '1', '2', or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
			}
		}
		if (strncmp(args[0], "Key_", 4) == 0) {
			if (arg_num == 1) {
				printf("ERR:%s is blank!\n", args[0]);
				ret = UAP_FAILURE;
				goto done;
			} else {
				tlvbuf_wep_key *tlv = NULL;
				int key_len = 0;
				if (args[1][0] == '"') {
					if ((strlen(args[1]) != 2) &&
					    (strlen(args[1]) != 7) &&
					    (strlen(args[1]) != 15)) {
						printf("ERR:Wrong key length!\n");
						ret = UAP_FAILURE;
						goto done;
					}
					key_len = strlen(args[1]) - 2;
				} else {
					if ((strlen(args[1]) != 0) &&
					    (strlen(args[1]) != 10) &&
					    (strlen(args[1]) != 26)) {
						printf("ERR:Wrong key length!\n");
						ret = UAP_FAILURE;
						goto done;
					}
					if (UAP_FAILURE == ishexstring(args[1])) {
						printf("ERR:Only hex digits are allowed when key length is 10 or 26\n");
						ret = UAP_FAILURE;
						goto done;
					}
					key_len = strlen(args[1]) / 2;
				}

				/* Append a new TLV */
				tlv_len = sizeof(tlvbuf_wep_key) + key_len;
				tmp_buffer = realloc(buffer, cmd_len + tlv_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot append WEP key configurations TLV!\n");
					ret = UAP_FAILURE;
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				cmd_buf = (apcmdbuf_sys_configure *)buffer;
				tlv = (tlvbuf_wep_key *)(buffer + cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				tlv->tag = MRVL_WEP_KEY_TLV_ID;
				tlv->length = key_len + 2;
				if (strcmp(args[0], "Key_0") == 0) {
					tlv->key_index = 0;
				} else if (strcmp(args[0], "Key_1") == 0) {
					tlv->key_index = 1;
				} else if (strcmp(args[0], "Key_2") == 0) {
					tlv->key_index = 2;
				} else if (strcmp(args[0], "Key_3") == 0) {
					tlv->key_index = 3;
				}
				if (keyindex == tlv->key_index) {
					tlv->is_default = 1;
				} else {
					tlv->is_default = 0;
				}
				if (args[1][0] == '"') {
					memcpy(tlv->key, &args[1][1],
					       strlen(args[1]) - 2);
				} else {
					string2raw(args[1], tlv->key);
				}
				endian_convert_tlv_header_out(tlv);
			}
		}
		if (strcmp(args[0], "PSK") == 0) {
			if (arg_num == 1) {
				printf("ERR:PSK is blank!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				tlvbuf_wpa_passphrase *tlv = NULL;
				if (args[1][0] == '"') {
					args[1]++;
				}
				if (args[1][strlen(args[1]) - 1] == '"') {
					args[1][strlen(args[1]) - 1] = '\0';
				}
				tlv_len =
					sizeof(tlvbuf_wpa_passphrase) +
					strlen(args[1]);
				if (strlen(args[1]) > MAX_WPA_PASSPHRASE_LENGTH) {
					printf("ERR:PSK too long.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (strlen(args[1]) < MIN_WPA_PASSPHRASE_LENGTH) {
					printf("ERR:PSK too short.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (strlen(args[1]) ==
				    MAX_WPA_PASSPHRASE_LENGTH) {
					if (UAP_FAILURE == ishexstring(args[1])) {
						printf("ERR:Only hex digits are allowed when passphrase's length is 64\n");
						ret = UAP_FAILURE;
						goto done;
					}
				}
				/* Append a new TLV */
				tmp_buffer = realloc(buffer, cmd_len + tlv_len);
				if (!tmp_buffer) {
					printf("ERR:Cannot append WPA passphrase TLV!\n");
					ret = UAP_FAILURE;
					goto done;
				} else {
					buffer = tmp_buffer;
					tmp_buffer = NULL;
				}
				cmd_buf = (apcmdbuf_sys_configure *)buffer;
				tlv = (tlvbuf_wpa_passphrase *)(buffer +
								cmd_len);
				cmd_len += tlv_len;
				/* Set TLV fields */
				tlv->tag = MRVL_WPA_PASSPHRASE_TLV_ID;
				tlv->length = strlen(args[1]);
				memcpy(tlv->passphrase, args[1], tlv->length);
				endian_convert_tlv_header_out(tlv);
			}
		}
		if (strcmp(args[0], "Protocol") == 0) {
			if (is_input_valid(PROTOCOL, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			protocol = (t_u16)atoi(args[1]);
		}
		if ((strcmp(args[0], "PairwiseCipher") == 0) ||
		    (strcmp(args[0], "GroupCipher") == 0)) {
			printf("ERR:PairwiseCipher and GroupCipher are not supported.\n" "    Please configure pairwise cipher using parameters PwkCipherWPA or PwkCipherWPA2\n" "    and group cipher using GwkCipher in the config file.\n");
			ret = UAP_FAILURE;
			goto done;
		}

		if ((protocol == PROTOCOL_NO_SECURITY) ||
		    (protocol == PROTOCOL_STATIC_WEP)) {
			if ((strcmp(args[0], "PwkCipherWPA") == 0) ||
			    (strcmp(args[0], "PwkCipherWPA2") == 0)
			    || (strcmp(args[0], "GwkCipher") == 0)) {
				printf("ERR:Pairwise cipher and group cipher should not be defined for Open and WEP mode.\n");
				ret = UAP_FAILURE;
				goto done;
			}
		}

		if (strcmp(args[0], "PwkCipherWPA") == 0) {
			if (arg_num == 1) {
				printf("ERR:PwkCipherWPA is blank!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal PwkCipherWPA parameter. Must be either bit '2' or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (atoi(args[1]) & ~CIPHER_BITMAP) {
					printf("ERR:Illegal PwkCipherWPA parameter. Must be either bit '2' or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				pwkcipher_wpa = atoi(args[1]);
				if (enable_11n &&
				    protocol != PROTOCOL_WPA2_MIXED) {
					memset(&htcap, 0, sizeof(htcap));
					if (UAP_SUCCESS ==
					    get_sys_cfg_11n(&htcap)) {
						if (htcap.supported_mcs_set[0]
						    && (atoi(args[1]) ==
							CIPHER_TKIP)) {
							printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
				}
			}
		}

		if (strcmp(args[0], "PwkCipherWPA2") == 0) {
			if (arg_num == 1) {
				printf("ERR:PwkCipherWPA2 is blank!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal PwkCipherWPA2 parameter. Must be either bit '2' or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				if (atoi(args[1]) & ~CIPHER_BITMAP) {
					printf("ERR:Illegal PwkCipherWPA2 parameter. Must be either bit '2' or '3'.\n");
					ret = UAP_FAILURE;
					goto done;
				}
				pwkcipher_wpa2 = atoi(args[1]);
				if (enable_11n &&
				    protocol != PROTOCOL_WPA2_MIXED) {
					memset(&htcap, 0, sizeof(htcap));
					if (UAP_SUCCESS ==
					    get_sys_cfg_11n(&htcap)) {
						if (htcap.supported_mcs_set[0]
						    && (atoi(args[1]) ==
							CIPHER_TKIP)) {
							printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
							ret = UAP_FAILURE;
							goto done;
						}
					}
				}
			}
		}
		if (strcmp(args[0], "GwkCipher") == 0) {
			if (is_input_valid(GWK_CIPHER, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			gwkcipher = atoi(args[1]);
		}
		if (strcmp(args[0], "GroupRekeyTime") == 0) {
			if (is_input_valid
			    (GROUPREKEYTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_group_rekey_timer *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_group_rekey_timer);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append group rekey timer TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_group_rekey_timer *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_GRP_REKEY_TIME_TLV_ID;
			tlv->length = 4;
			tlv->group_rekey_time_sec = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->group_rekey_time_sec =
				uap_cpu_to_le32(tlv->group_rekey_time_sec);
		}
		if (strcmp(args[0], "MaxStaNum") == 0) {
			if (is_input_valid(MAXSTANUM, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			if (get_max_sta_num_supported(&max_sta_num_supported) ==
			    UAP_FAILURE) {
				ret = UAP_FAILURE;
				goto done;
			}
			if (atoi(args[1]) > max_sta_num_supported) {
				printf("ERR: MAX_STA_NUM must be less than %d\n", max_sta_num_supported);
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_max_sta_num *tlv = NULL;

			/* Append a new TLV */
			tlv_len =
				sizeof(tlvbuf_max_sta_num) -
				sizeof(tlv->max_sta_num_supported);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot realloc max station number TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_max_sta_num *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_MAX_STA_CNT_TLV_ID;
			tlv->length = 2;
			tlv->max_sta_num_configured = (t_u16)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->max_sta_num_configured =
				uap_cpu_to_le16(tlv->max_sta_num_configured);
		}
		if (strcmp(args[0], "Retrylimit") == 0) {
			if (is_input_valid(RETRYLIMIT, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_retry_limit *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_retry_limit);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot realloc retry limit TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_retry_limit *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_RETRY_LIMIT_TLV_ID;
			tlv->length = 1;
			tlv->retry_limit = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "PairwiseUpdateTimeout") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_eapol_pwk_hsk_timeout *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_eapol_pwk_hsk_timeout);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append pairwise update timeout TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_eapol_pwk_hsk_timeout *)(buffer +
							       cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_EAPOL_PWK_HSK_TIMEOUT_TLV_ID;
			tlv->length = 4;
			tlv->pairwise_update_timeout = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->pairwise_update_timeout =
				uap_cpu_to_le32(tlv->pairwise_update_timeout);
		}
		if (strcmp(args[0], "PairwiseHandshakeRetries") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_eapol_pwk_hsk_retries *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_eapol_pwk_hsk_retries);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append pairwise handshake retries TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_eapol_pwk_hsk_retries *)(buffer +
							       cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_EAPOL_PWK_HSK_RETRIES_TLV_ID;
			tlv->length = 4;
			tlv->pwk_retries = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->pwk_retries = uap_cpu_to_le32(tlv->pwk_retries);
		}
		if (strcmp(args[0], "GroupwiseUpdateTimeout") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_eapol_gwk_hsk_timeout *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_eapol_gwk_hsk_timeout);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append groupwise update timeout TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_eapol_gwk_hsk_timeout *)(buffer +
							       cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_EAPOL_GWK_HSK_TIMEOUT_TLV_ID;
			tlv->length = 4;
			tlv->groupwise_update_timeout = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->groupwise_update_timeout =
				uap_cpu_to_le32(tlv->groupwise_update_timeout);
		}
		if (strcmp(args[0], "GroupwiseHandshakeRetries") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_eapol_gwk_hsk_retries *tlv = NULL;

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_eapol_gwk_hsk_retries);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append groupwise handshake retries TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_eapol_gwk_hsk_retries *)(buffer +
							       cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_EAPOL_GWK_HSK_RETRIES_TLV_ID;
			tlv->length = 4;
			tlv->gwk_retries = (t_u32)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
			tlv->gwk_retries = uap_cpu_to_le32(tlv->gwk_retries);
		}

		if (strcmp(args[0], "Enable11n") == 0) {
			if ((ISDIGIT(args[1]) != UAP_SUCCESS) ||
			    (atoi(args[1]) < 0) || (atoi(args[1]) > 1)) {
				printf("ERR: Invalid Enable11n value\n");
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_htcap_t *tlv = NULL;
			enable_11n = atoi(args[1]);

			memset(&htcap, 0, sizeof(htcap));
			if (UAP_SUCCESS != get_sys_cfg_11n(&htcap)) {
				printf("ERR: Reading current 11n configuration.\n");
				ret = UAP_FAILURE;
				goto done;
			}

			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_htcap_t);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append HT Cap TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_htcap_t *)(buffer + cmd_len);
			tlv_offset_11n = cmd_len;
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = HT_CAPABILITY_TLV_ID;
			tlv->length = sizeof(HTCap_t);
			memcpy(&tlv->ht_cap, &htcap, sizeof(HTCap_t));
			if (enable_11n == 1) {
				/* enable mcs rate */
				tlv->ht_cap.supported_mcs_set[0] =
					DEFAULT_MCS_SET_0;
				tlv->ht_cap.supported_mcs_set[4] =
					DEFAULT_MCS_SET_4;
				if (0 == get_fw_info(&fw)) {
					if ((fw.hw_dev_mcs_support & 0x0f) >= 2)
						tlv->ht_cap.
							supported_mcs_set[1] =
							DEFAULT_MCS_SET_1;
				}
			} else {
				/* disable mcs rate */
				tlv->ht_cap.supported_mcs_set[0] = 0;
				tlv->ht_cap.supported_mcs_set[4] = 0;
				tlv->ht_cap.supported_mcs_set[1] = 0;
			}
			endian_convert_tlv_header_out(tlv);
		}
		if (strcmp(args[0], "HTCapInfo") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before HTCapInfo.\n");
				ret = UAP_FAILURE;
				goto done;
			}
			if ((IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE) ||
			    ((((t_u16)A2HEXDECIMAL(args[1])) &
			      (~HT_CAP_CONFIG_MASK)) != HT_CAP_CHECK_MASK)) {
				printf("ERR: Invalid HTCapInfo value\n");
				ret = UAP_FAILURE;
				goto done;
			}

			/* Find HT tlv pointer in buffer and set HTCapInfo */
			tlvbuf_htcap_t *tlv = NULL;
			tlv = (tlvbuf_htcap_t *)(buffer + tlv_offset_11n);
			tlv->ht_cap.ht_cap_info =
				DEFAULT_HT_CAP_VALUE & ~HT_CAP_CONFIG_MASK;
			tlv->ht_cap.ht_cap_info |=
				(t_u16)A2HEXDECIMAL(args[1]) &
				HT_CAP_CONFIG_MASK;
			tlv->ht_cap.ht_cap_info =
				uap_cpu_to_le16(tlv->ht_cap.ht_cap_info);
		}
		if (strcmp(args[0], "AMPDU") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before AMPDU.\n");
				ret = UAP_FAILURE;
				goto done;
			}
			if ((IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE) ||
			    ((A2HEXDECIMAL(args[1])) > AMPDU_CONFIG_MASK)) {
				printf("ERR: Invalid AMPDU value\n");
				ret = UAP_FAILURE;
				goto done;
			}

			/* Find HT tlv pointer in buffer and set AMPDU */
			tlvbuf_htcap_t *tlv = NULL;
			tlv = (tlvbuf_htcap_t *)(buffer + tlv_offset_11n);
			tlv->ht_cap.ampdu_param =
				(t_u8)A2HEXDECIMAL(args[1]) & AMPDU_CONFIG_MASK;
		}
		if (strcmp(args[0], "HT_MCS_MAP") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before HT_MCS_MAP.\n");
				ret = UAP_FAILURE;
				goto done;
			}
			if (0 == get_fw_info(&fw)) {
				/* Check upper nibble of MCS support value
				 * and block MCS_SET_1 when 2X2 is not supported
				 * by the underlying hardware */
				if (((fw.hw_dev_mcs_support & 0xf0) <
				     STREAM_2X2_MASK) &&
				    (A2HEXDECIMAL(args[1]) & MCS_SET_1_MASK)) {
					printf("ERR: Invalid HT_MCS_MAP\n");
					goto done;
				}
			}

			/* Find HT tlv pointer in buffer and set supported MCS set */
			tlvbuf_htcap_t *tlv = NULL;
			tlv = (tlvbuf_htcap_t *)(buffer + tlv_offset_11n);
			supported_mcs_set = (t_u32)A2HEXDECIMAL(args[1]);
			supported_mcs_set = uap_cpu_to_le32(supported_mcs_set);
			memcpy(tlv->ht_cap.supported_mcs_set,
			       &supported_mcs_set, sizeof(t_u32));
		}
		if (strcmp(args[0], "Enable2040Coex") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0) ||
			    (atoi(args[1]) > 1)) {
				printf("ERR: Invalid Enable2040Coex value\n");
				ret = UAP_FAILURE;
				goto done;
			}
			tlvbuf_2040_coex *tlv = NULL;
			/* Append a new TLV */
			tlv_len = sizeof(tlvbuf_2040_coex);
			tmp_buffer = realloc(buffer, cmd_len + tlv_len);
			if (!tmp_buffer) {
				printf("ERR:Cannot append 2040 coex TLV!\n");
				ret = UAP_FAILURE;
				goto done;
			} else {
				buffer = tmp_buffer;
				tmp_buffer = NULL;
			}
			cmd_buf = (apcmdbuf_sys_configure *)buffer;
			tlv = (tlvbuf_2040_coex *)(buffer + cmd_len);
			cmd_len += tlv_len;
			/* Set TLV fields */
			tlv->tag = MRVL_2040_BSS_COEX_CONTROL_TLV_ID;
			tlv->length = 1;
			tlv->enable = (t_u8)atoi(args[1]);
			endian_convert_tlv_header_out(tlv);
		}
#if DEBUG
		if (cmd_len != 0) {
			hexdump("Command Buffer", (void *)cmd_buf, cmd_len,
				' ');
		}
#endif
	}
done:
	fclose(config_file);
	if (buffer)
		free(buffer);
	if (line)
		free(line);
	return ret;
}

/**
 *  @brief Get band from current channel
 *  @param         band
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
static int
get_band(int *band)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_channel_config *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len;
	int ret = UAP_SUCCESS;

	/* Initialize the command length */
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_channel_config);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(cmd_len);

	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		ret = UAP_FAILURE;
		goto done;
	}
	memset(buffer, 0, cmd_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_channel_config *)(buffer +
					sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_CHANNELCONFIG_TLV_ID;
	tlv->length = sizeof(tlvbuf_channel_config) - TLVHEADER_LEN;
	cmd_buf->action = ACTION_GET;

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);

	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_CHANNELCONFIG_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			ret = UAP_FAILURE;
			goto done;
		}
		if (cmd_buf->result == CMD_SUCCESS) {
			if (tlv->chan_number > MAX_CHANNELS_BG)
				*band = BAND_A;
			else
				*band = BAND_B | BAND_G;
		} else {
			printf("ERR:Could not get band!\n");
			ret = UAP_FAILURE;
			goto done;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		ret = UAP_FAILURE;
		goto done;
	}
done:
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Show usage information for the cfg_80211d command
 *
 *  @return         N/A
 */
void
print_apcmd_cfg_80211d_usage(void)
{
	printf("\nUsage: cfg_80211d <state 0/1> <country Country_code> \n");
	return;
}

/**
 *  @brief Show usage information for the uap_stats command
 *
 *  @return         N/A
 */
void
print_apcmd_uap_stats(void)
{
	printf("Usage: uap_stats \n");
	return;
}

/**
 *  SNMP MIB OIDs Table
 */
static oids_table snmp_oids[] = {
	{0x0b, 4, "dot11LocalTKIPMICFailures"},
	{0x0c, 4, "dot11CCMPDecryptErrors"},
	{0x0d, 4, "dot11WEPUndecryptableCount"},
	{0x0e, 4, "dot11WEPICVErrorCount"},
	{0x0f, 4, "dot11DecryptFailureCount"},
	{0x12, 4, "dot11FailedCount"},
	{0x13, 4, "dot11RetryCount"},
	{0x14, 4, "dot11MultipleRetryCount"},
	{0x15, 4, "dot11FrameDuplicateCount"},
	{0x16, 4, "dot11RTSSuccessCount"},
	{0x17, 4, "dot11RTSFailureCount"},
	{0x18, 4, "dot11ACKFailureCount"},
	{0x19, 4, "dot11ReceivedFragmentCount"},
	{0x1a, 4, "dot11MulticastReceivedFrameCount"},
	{0x1b, 4, "dot11FCSErrorCount"},
	{0x1c, 4, "dot11TransmittedFrameCount"},
	{0x1d, 4, "dot11RSNATKIPCounterMeasuresInvoked"},
	{0x1e, 4, "dot11RSNA4WayHandshakeFailures"},
	{0x1f, 4, "dot11MulticastTransmittedFrameCount"}
};

/**
 *  @brief Get uAP stats
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_uap_stats(int argc, char *argv[])
{
	t_u8 no_of_oids = sizeof(snmp_oids) / sizeof(snmp_oids[0]);
	t_u16 i, j;
	int size;
	apcmdbuf_snmp_mib *cmd_buf = NULL;
	t_u8 *buf = NULL;
	tlvbuf_header *tlv = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int opt;
	t_u16 oid_size, byte2 = 0;
	t_u32 byte4 = 0;
	int ret = UAP_SUCCESS;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_apcmd_uap_stats();
			return UAP_SUCCESS;
		}
	}

	argc -= optind;
	argv += optind;
	if (argc) {
		printf("Error: Invalid Input\n");
		print_apcmd_uap_stats();
		return UAP_FAILURE;
	}

    /**  Command Header */
	cmd_len += sizeof(apcmdbuf_snmp_mib);

	for (i = 0; i < no_of_oids; i++) {
	/**
         * Size of Oid + Oid_value + Oid_size
         */
		cmd_len += snmp_oids[i].len + sizeof(tlvbuf_header);
	}
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Locate Headers */
	cmd_buf = (apcmdbuf_snmp_mib *)buf;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->cmd_code = HostCmd_SNMP_MIB;
	cmd_buf->action = uap_cpu_to_le16(ACTION_GET);

	tlv = (tlvbuf_header *)((t_u8 *)cmd_buf + sizeof(apcmdbuf_snmp_mib));
	/* Add oid, oid_size and oid_value for each OID */
	for (i = 0; i < no_of_oids; i++) {
	/** Copy Index as Oid */
		tlv->type = uap_cpu_to_le16(snmp_oids[i].type);
	/** Copy its size */
		tlv->len = uap_cpu_to_le16(snmp_oids[i].len);
	/** Next TLV */
		tlv = (tlvbuf_header *)&(tlv->data[snmp_oids[i].len]);
	}
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			tlv = (tlvbuf_header *)((t_u8 *)cmd_buf +
						sizeof(apcmdbuf_snmp_mib));

			size = cmd_buf->size - (sizeof(apcmdbuf_snmp_mib) -
						BUF_HEADER_SIZE);

			while ((unsigned int)size >= sizeof(tlvbuf_header)) {
				tlv->type = uap_le16_to_cpu(tlv->type);
				for (i = 0; i < no_of_oids; i++) {
					if (snmp_oids[i].type == tlv->type) {
						printf("%s: ",
						       snmp_oids[i].name);
						break;
					}
				}
				oid_size = uap_le16_to_cpu(tlv->len);
				switch (oid_size) {
				case 1:
					printf("%d",
					       (unsigned int)tlv->data[0]);
					break;
				case 2:
					memcpy(&byte2, &tlv->data[0],
					       sizeof(oid_size));
					printf("%d",
					       (unsigned int)
					       uap_le16_to_cpu(byte2));
					break;
				case 4:
					memcpy(&byte4, &tlv->data[0],
					       sizeof(oid_size));
					printf("%d",
					       (unsigned int)
					       uap_le32_to_cpu(byte4));
					break;
				default:
					for (j = 0; j < oid_size; j++) {
						printf("%d ",
						       (t_u8)tlv->data[j]);
					}
					break;
				}
		/** Next TLV */
				tlv = (tlvbuf_header *)&(tlv->data[oid_size]);
				size -= (sizeof(tlvbuf_header) + oid_size);
				size = (size > 0) ? size : 0;
				printf("\n");
			}

		} else {
			printf("ERR:Command Response incorrect!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	free(buf);
	return ret;
}

/**
 *  @brief parser for sys_cfg_80211d input
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @param output   Stores indexes for "state, country"
 *                  arguments
 *
 *  @return         NA
 *
 */
static void
parse_input_80211d(int argc, char **argv, int output[2][2])
{
	int i, j, k = 0;
	char *keywords[2] = { "state", "country" };

	for (i = 0; i < 2; i++)
		output[i][0] = -1;

	for (i = 0; i < argc; i++) {
		for (j = 0; j < 2; j++) {
			if (strcmp(argv[i], keywords[j]) == 0) {
				output[j][1] = output[j][0] = i;
				k = j;
				break;
			}
		}
		output[k][1] += 1;
	}
}

/**
 *  @brief Set/Get 802.11D country information
 *
 *  Usage: cfg_80211d state country_code
 *
 *  State 0 or 1
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_cfg_80211d(int argc, char *argv[])
{
	apcmdbuf_cfg_80211d *cmd_buf = NULL;
	ieeetypes_subband_set_t *subband = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int output[2][2];
	int ret = UAP_SUCCESS;
	int opt;
	int status = 0;
	int i, j;
	t_u8 state = 0;
	char country[4] = { ' ', ' ', 0, 0 };
	t_u8 sflag = 0, cflag = 0;
	ieeetypes_subband_set_t sub_bands[MAX_SUB_BANDS];
	t_u8 no_of_sub_band = 0;
	int band;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_apcmd_cfg_80211d_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc) {
		if (strcmp(argv[0], "state") && strcmp(argv[0], "country")) {
			printf("ERR: Incorrect input. Either state or country is needed.");
			print_apcmd_cfg_80211d_usage();
			return UAP_FAILURE;
		}
	/** SET */
		parse_input_80211d(argc, argv, output);

	/** State */
		if ((output[0][0] != -1) && (output[0][1] > output[0][0])) {
			if ((output[0][1] - output[0][0]) != 2) {
				printf("ERR: Invalid state inputs\n");
				print_apcmd_cfg_80211d_usage();
				return UAP_FAILURE;
			}

			if (IS_HEX_OR_DIGIT(argv[output[0][0] + 1]) ==
			    UAP_FAILURE) {
				printf("ERR: valid input for state are 0 or 1\n");
				print_apcmd_cfg_80211d_usage();
				return UAP_FAILURE;
			}
			state = (t_u8)A2HEXDECIMAL(argv[output[0][0] + 1]);

			if ((state != 0) && (state != 1)) {
				printf("ERR: valid input for state are 0 or 1 \n");
				print_apcmd_cfg_80211d_usage();
				return UAP_FAILURE;
			}
			sflag = 1;
		}

	/** Country */
		if ((output[1][0] != -1) && (output[1][1] > output[1][0])) {
			if ((output[1][1] - output[1][0]) != 2) {
				printf("ERR: Invalid country inputs\n");
				print_apcmd_cfg_80211d_usage();
				return UAP_FAILURE;
			}
			if ((strlen(argv[output[1][0] + 1]) > 3) ||
			    (strlen(argv[output[1][0] + 1]) == 0)) {
				print_apcmd_cfg_80211d_usage();
				return UAP_FAILURE;
			}
			/* Only 2 characters of country code are copied here as indoor/outdoor
			 * conditions are not handled in domain file */
			strncpy(country, argv[output[1][0] + 1], 2);

			for (i = 0; (unsigned int)i < strlen(country); i++) {
				if ((country[i] < 'A') || (country[i] > 'z')) {
					printf("Invalid Country Code\n");
					print_apcmd_cfg_80211d_usage();
					return UAP_FAILURE;
				}
				if (country[i] > 'Z')
					country[i] = country[i] - 'a' + 'A';
			}

			cflag = 1;
			if (!get_band(&band)) {
				printf("ERR:couldn't get band from channel!\n");
				return UAP_FAILURE;
			}
	   /** Get domain information from the file */
			no_of_sub_band =
				parse_domain_file(country, band, sub_bands,
						  NULL);
			if (no_of_sub_band == UAP_FAILURE) {
				printf("Parsing Failed\n");
				return UAP_FAILURE;
			}
		}
		if (get_bss_status(&status) != UAP_SUCCESS) {
			printf("ERR:Cannot get current bss status!\n");
			return UAP_FAILURE;
		}
		if (status == UAP_BSS_START) {
			printf("ERR: 11d status can not be changed after BSS start!\n");
			return UAP_FAILURE;
		}
	}

	if (argc && !cflag && !sflag) {
		printf("ERR: Invalid input\n");
		print_apcmd_cfg_80211d_usage();
		return UAP_FAILURE;
	}

	if (sflag && !cflag) {
	/**
         * Update MIB only and return
         */
		if (sg_snmp_mib
		    (ACTION_SET, OID_80211D_ENABLE, sizeof(state),
		     &state) == UAP_SUCCESS) {
			printf("802.11d %sd \n", state ? "enable" : "disable");
			return UAP_SUCCESS;
		} else {
			return UAP_FAILURE;
		}
	}

	cmd_len = sizeof(apcmdbuf_cfg_80211d);

	if (cflag) {
		cmd_len += no_of_sub_band * sizeof(ieeetypes_subband_set_t);
	} else {
	     /** Get */
		cmd_len += MAX_SUB_BANDS * sizeof(ieeetypes_subband_set_t);
	}

	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)buf;
	cmd_len = argc ? cmd_len :
		 /** Set */
		(sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));
								/** Get */

	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->action = argc ? ACTION_SET : ACTION_GET;
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

	if (cflag) {
		/* third character of country code is copied here as indoor/outdoor
		 * condition is not supported in domain file*/
		strncpy(country, argv[output[1][0] + 1], sizeof(country) - 1);
		for (i = 0; (unsigned int)i < strlen(country); i++) {
			if ((country[i] < 'A') || (country[i] > 'z')) {
				printf("Invalid Country Code\n");
				print_apcmd_cfg_80211d_usage();
				free(buf);
				return UAP_FAILURE;
			}
			if (country[i] > 'Z')
				country[i] = country[i] - 'a' + 'A';
		}
		cmd_buf->domain.tag = uap_cpu_to_le16(TLV_TYPE_DOMAIN);
		cmd_buf->domain.length = uap_cpu_to_le16(sizeof(domain_param_t)
							 - BUF_HEADER_SIZE
							 +
							 (no_of_sub_band *
							  sizeof
							  (ieeetypes_subband_set_t)));

		memset(cmd_buf->domain.country_code, ' ',
		       sizeof(cmd_buf->domain.country_code));
		memcpy(cmd_buf->domain.country_code, country, strlen(country));
		memcpy(cmd_buf->domain.subband, sub_bands,
		       no_of_sub_band * sizeof(ieeetypes_subband_set_t));
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc) {
				printf("Set executed successfully\n");
				if (sflag) {
					if (sg_snmp_mib
					    (ACTION_SET, OID_80211D_ENABLE,
					     sizeof(state),
					     &state) == UAP_SUCCESS) {
						printf("802.11d %sd \n",
						       state ? "enable" :
						       "disable");
					}
				}
			} else {
				j = uap_le16_to_cpu(cmd_buf->domain.length);
				if (sg_snmp_mib
				    (ACTION_GET, OID_80211D_ENABLE,
				     sizeof(state), &state)
				    == UAP_SUCCESS) {
					printf("State = %sd\n",
					       state ? "enable" : "disable");
				}

				if (cmd_buf->domain.country_code[0] ||
				    cmd_buf->domain.country_code[1] ||
				    cmd_buf->domain.country_code[2]) {
					printf("Country string = %c%c%c",
					       cmd_buf->domain.country_code[0],
					       cmd_buf->domain.country_code[1],
					       cmd_buf->domain.country_code[2]);
					j -= sizeof(cmd_buf->domain.
						    country_code);
					subband =
						(ieeetypes_subband_set_t *)
						cmd_buf->domain.subband;
					printf("\nSub-band info=");
					printf("\t(1st, #chan, MAX-power) \n");
					for (i = 0; i < (j / 3); i++) {
						printf("\t\t(%d, \t%d, \t%d dbm)\n", subband->first_chan, subband->no_of_chan, subband->max_tx_pwr);
						subband++;
					}
				}
			}
		} else {
			printf("ERR:Command Response incorrect!\n");
			if (argc)
				printf("11d info is allowed to set only before bss start.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	free(buf);
	return ret;
}

/**
 *  @brief Creates a sys_config request and sends to the driver
 *
 *  Usage: "Usage : sys_config [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_config(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len;
	t_u16 buf_len;
	int ret = UAP_SUCCESS;
	int opt;
	char **argv_dummy = NULL;
	ps_mgmt pm;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_config_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_sys_config_usage();
		return UAP_FAILURE;
	}
	if (argc == 1) {
		/* Read profile and send command to firmware */
		ret = apcmd_sys_config_profile(argc, argv);
		return ret;
	}

    /** Query AP's setting */
	buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;

	/* Alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);

	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Reset custom_ie_present flag */
	custom_ie_present = 0;

	/* Reset max_mgmt_ie_print flag */
	max_mgmt_ie_print = 0;

	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_sys_configure);
	cmd_buf = (apcmdbuf_sys_configure *)buf;

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			free(buf);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("AP settings:\n");
			print_tlv(buf + sizeof(apcmdbuf_sys_configure),
				  cmd_buf->size -
				  sizeof(apcmdbuf_sys_configure) +
				  BUF_HEADER_SIZE);
			if (apcmd_addbapara(1, argv_dummy) == UAP_FAILURE) {
				printf("Couldn't get ADDBA parameters\n");
				free(buf);
				return UAP_FAILURE;
			}
			if (apcmd_aggrpriotbl(1, argv_dummy) == UAP_FAILURE) {
				printf("Couldn't get AMSDU/AMPDU priority table\n");
				free(buf);
				return UAP_FAILURE;
			}
			if (apcmd_addbareject(1, argv_dummy) == UAP_FAILURE) {
				printf("Couldn't get ADDBA reject table\n");
				free(buf);
				return UAP_FAILURE;
			}
			printf("\n802.11D setting:\n");
			if (apcmd_cfg_80211d(1, argv_dummy) == UAP_FAILURE) {
				free(buf);
				return UAP_FAILURE;
			}
			if (!custom_ie_present) {
				printf("\nCustom IE settings:\n");
				if (apcmd_sys_cfg_custom_ie(1, argv_dummy) ==
				    UAP_FAILURE) {
					free(buf);
					return UAP_FAILURE;
				}
			}
		} else {
			printf("ERR:Could not retrieve system configure\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	free(buf);
	memset(&pm, 0, sizeof(ps_mgmt));
	ret = send_power_mode_ioctl(&pm, 0);
	return ret;
}

/** Update domain info with given country and band
 *
 * @param country_80211d   country to be set
 * @param band      band of operation
 * @return          UAP_SUCCESS/UAP_FAILURE
 */
static int
update_domain_info(char *country_80211d, int band)
{
	apcmdbuf_cfg_80211d *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	ieeetypes_subband_set_t sub_bands[MAX_SUB_BANDS];
	t_u8 no_of_sub_band = 0;
	char country[4] = { ' ', ' ', 0, 0 };
	int ret = UAP_SUCCESS;
	t_u8 domain_code = 0;
	rgn_dom_code_t *prgn_dom_code = NULL;

	/* Get currently set country code from FW */
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)buf;
	cmd_len = (sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));

	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->action = ACTION_GET;
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if ((ret == UAP_SUCCESS) && (cmd_buf->result == CMD_SUCCESS) &&
	    (cmd_buf->domain.country_code[0] ||
	     cmd_buf->domain.country_code[1] ||
	     cmd_buf->domain.country_code[2])) {
		country[0] = cmd_buf->domain.country_code[0];
		country[1] = cmd_buf->domain.country_code[1];
	} else {
		/* country_80211d will only be used if country code is not already set */
		country[0] = country_80211d[0];
		country[1] = country_80211d[1];
	}

	if (band == BAND_A)
		printf("Enabling 11d for 11h operation and setting country code to %s\n", country);
	cmd_len = sizeof(apcmdbuf_cfg_80211d);
    /** Get domain information from the file */
	no_of_sub_band =
		parse_domain_file(country, band, sub_bands, &domain_code);
	if (no_of_sub_band == UAP_FAILURE) {
		printf("Parsing Failed\n");
		ret = UAP_FAILURE;
		goto done;
	}
	/* Copy third character of country code here */
	if (cmd_buf->domain.country_code[2]) {
		country[2] = cmd_buf->domain.country_code[2];
	} else {
		country[2] = country_80211d[2];
	}

	/* Set domain for this country code */
	memset(buf, 0, buf_len);
	cmd_len += no_of_sub_band * sizeof(ieeetypes_subband_set_t);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)buf;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->action = ACTION_SET;
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;

	cmd_buf->domain.tag = uap_cpu_to_le16(TLV_TYPE_DOMAIN);
	cmd_buf->domain.length = uap_cpu_to_le16(sizeof(domain_param_t)
						 - BUF_HEADER_SIZE
						 +
						 (no_of_sub_band *
						  sizeof
						  (ieeetypes_subband_set_t)));

	memset(cmd_buf->domain.country_code, ' ',
	       sizeof(cmd_buf->domain.country_code));
	memcpy(cmd_buf->domain.country_code, country, strlen(country));
	memcpy(cmd_buf->domain.subband, sub_bands,
	       no_of_sub_band * sizeof(ieeetypes_subband_set_t));

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result != CMD_SUCCESS) {
			printf("ERR:Command Response incorrect!\n");
			printf("11d info is allowed to set only before bss start.\n");
			ret = UAP_FAILURE;
			goto done;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	if ((band == BAND_A) && domain_code) {
		/* Send 11D_CMD a second time, with reg_dom_code TLV appended.
		 * Use hostcmd instead of re-route. */
		cmd_len = cmd_buf->size + BUF_HEADER_SIZE;
		prgn_dom_code = (rgn_dom_code_t *)((t_u8 *)cmd_buf + cmd_len);
		cmd_len += sizeof(rgn_dom_code_t);
		cmd_buf->size += sizeof(rgn_dom_code_t);
#if DEBUG
		uap_printf(MSG_DEBUG,
			   "Adding Region Domain Code TLV.  Domain_code=%d\n",
			   domain_code);
#endif
		prgn_dom_code->tag =
			uap_cpu_to_le16(MRVL_REGION_DOMAIN_CODE_TLV_ID);
		prgn_dom_code->length = uap_cpu_to_le16(sizeof(rgn_dom_code_t)
							- BUF_HEADER_SIZE);
		prgn_dom_code->domain_code = domain_code;

		/* Send the command */
		uap_ioctl_no_reroute = 1;
		ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
		uap_ioctl_no_reroute = 0;

		if (ret == UAP_SUCCESS) {
			if (cmd_buf->result != CMD_SUCCESS) {
				printf("ERR:Command Response incorrect!\n");
				printf("11d info is allowed to set only before bss start.\n");
				ret = UAP_FAILURE;
				goto done;
			}
		} else {
			printf("ERR:Command sending failed!\n");
		}
	}

done:
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief Convert MCS Rate Bitmap to MCS Rate index
 *
 *  @param rate_bitmap  Pointer to rate bitmap
 *  @param size         Size of the bitmap array
 *
 *  @return             Rate index
 */
int
get_rate_index_from_bitmap(t_u16 *rate_bitmap, int size)
{
	int i;

	for (i = 0; i < size * 8; i++) {
		if (rate_bitmap[i / 16] & (1 << (i % 16))) {
			return i;
		}
	}
	return 0;
}

/**
 *  @brief Checks current system configuration
 *
 *  @param buf     pointer to TLV buffer
 *  @param len     TLV buffer length
 *
 *  @return        UAP_SUCCESS/UAP_FAILURE
 */
int
check_sys_config(t_u8 *buf, t_u16 len)
{
	tlvbuf_header *pcurrent_tlv = (tlvbuf_header *)buf;
	int tlv_buf_left = len;
	t_u16 tlv_type;
	t_u16 tlv_len;
	tlvbuf_channel_config *channel_tlv = NULL;
	tlvbuf_channel_list *chnlist_tlv = NULL;
	tlvbuf_pwk_cipher *pwk_cipher_tlv = NULL;
	tlvbuf_gwk_cipher *gwk_cipher_tlv = NULL;
	tlvbuf_auth_mode *auth_tlv = NULL;
	tlvbuf_protocol *protocol_tlv = NULL;
	tlvbuf_wep_key *wep_key_tlv = NULL;
	tlvbuf_wpa_passphrase *passphrase_tlv = NULL;
	tlvbuf_rates *rates_tlv = NULL;
	tlvbuf_mcbc_data_rate *mcbc_data_rate_tlv = NULL;
	t_u16 protocol = 0;
	t_u16 mcbc_data_rate = 0;
	t_u16 op_rates_len = 0;
	t_u8 acs_mode_enabled = 0;
	t_u8 *cbuf = NULL;
	apcmdbuf_cfg_80211d *cmd_buf = NULL;
	t_u16 buf_len, cmd_len;
	char country_80211d[4] = { 'U', 'S', ' ', 0 };
	t_u8 state_80211h = 0;
	int channel_tlv_band = BAND_B | BAND_G;
	t_u8 secondary_ch_set = 0;
	int scan_channels_band = BAND_B | BAND_G;
	t_u8 state_80211d = 0;
	t_u8 rate = 0;
	t_u32 rate_bitmap = 0;
	tlvbuf_htcap_t *ht_cap_tlv = NULL;
	t_u16 enable_40Mhz = 0;
	t_u16 enable_20Mhz_sgi = 0;
	t_u16 enable_gf = 0;
	t_u8 enable_11n = 0;
	int flag = 0;
	fw_info fw;
	channel_list *pchan_list;
	tlvbuf_tx_power *txpower_tlv = NULL;
	int i = 0, ret = UAP_SUCCESS;
	int pairwise_cipher_wpa = -1;
	int pairwise_cipher_wpa2 = -1;
	int group_cipher = -1;
	int chan_list_len = 0;
	int key_set = 0;
	t_u8 channel = 0;
	tx_rate_cfg_t tx_rate_config;
	t_u32 mcs_rate_index = 0;
	struct eth_priv_vhtcfg vhtcfg = { 0 };

#define BITMAP_RATE_1M         0x01
#define BITMAP_RATE_2M         0x02
#define BITMAP_RATE_5_5M       0x04
#define BITMAP_RATE_11M        0x8
#define B_RATE_MANDATORY       0x0f

	if (pcurrent_tlv == NULL) {
		printf("ERR: No TLV buffer available!\n");
		return UAP_FAILURE;
	}

	ret = sg_snmp_mib(ACTION_GET, OID_80211D_ENABLE, sizeof(state_80211d),
			  &state_80211d);

	buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	cbuf = (t_u8 *)malloc(buf_len);
	if (!cbuf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(cbuf, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)cbuf;
	cmd_len = (sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->action = uap_cpu_to_le16(ACTION_GET);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS && cmd_buf->result == CMD_SUCCESS) {
		if (cmd_buf->domain.country_code[0] ||
		    cmd_buf->domain.country_code[1] ||
		    cmd_buf->domain.country_code[2]) {
			strncpy(country_80211d,
				(char *)cmd_buf->domain.country_code,
				COUNTRY_CODE_LEN);
		}
	}
	free(cbuf);

	/* get region code to handle special cases */
	memset(&fw, 0, sizeof(fw));
	if (get_fw_info(&fw)) {
		printf("Could not get fw info!\n");
		return UAP_FAILURE;
	}

	if (get_tx_rate_cfg(&tx_rate_config) == UAP_FAILURE) {
		printf("Could not get tx_rate_cfg!\n");
		return UAP_FAILURE;
	}
	while (tlv_buf_left >= (int)sizeof(tlvbuf_header) &&
	       (ret != UAP_FAILURE)) {
		tlv_type = *(t_u8 *)&pcurrent_tlv->type;
		tlv_type |= (*((t_u8 *)&pcurrent_tlv->type + 1) << 8);
		tlv_len = *(t_u8 *)&pcurrent_tlv->len;
		tlv_len |= (*((t_u8 *)&pcurrent_tlv->len + 1) << 8);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlv_len=%d, tlv_buf_left=%d\n",
			       tlv_len, tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case MRVL_CHANNELCONFIG_TLV_ID:
			channel_tlv = (tlvbuf_channel_config *)pcurrent_tlv;
			channel = channel_tlv->chan_number;

			if ((!state_80211d) &&
			    (channel_tlv->chan_number == MAX_CHANNELS_BG)
			    &&
			    (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1)
			     == 0)
				) {
				printf("ERR: Invalid channel 14, 802.11d disabled, country code JP!\n");
				return UAP_FAILURE;
			}
			if (channel_tlv->bandcfg.chanBand == BAND_5GHZ)
				channel_tlv_band = BAND_A;
			secondary_ch_set = channel_tlv->bandcfg.chan2Offset;
			if (channel_tlv_band != BAND_A) {
				/* When country code is not Japan, allow channels 5-11 for US and 5-13 for non-US
				 * only for secondary channel below */
				if (secondary_ch_set == SEC_CHAN_BELOW) {
					if (!strncmp
					    (country_80211d, "US",
					     COUNTRY_CODE_LEN - 1)) {
						if (channel_tlv->chan_number >
						    DEFAULT_MAX_CHANNEL_BELOW) {
							printf("ERR: Only channels 5-11 are allowed with secondary channel below for the US\n");
							return UAP_FAILURE;
						}
					} else if (strncmp
						   (country_80211d, "JP",
						    COUNTRY_CODE_LEN - 1)) {
						if (channel_tlv->chan_number >
						    DEFAULT_MAX_CHANNEL_BELOW_NON_US)
						{
							printf("ERR: Only channels 5-13 are allowed with secondary channel below for" "non-Japan countries!\n");
							return UAP_FAILURE;
						}
					}
				}

				/* When country code is not Japan, allow channels 1-7 for US and 1-9 for non-US
				 * only for secondary channel above */
				if (secondary_ch_set == SEC_CHAN_ABOVE) {
					if (!strncmp
					    (country_80211d, "US",
					     COUNTRY_CODE_LEN - 1)) {
						if (channel_tlv->chan_number >
						    DEFAULT_MAX_CHANNEL_ABOVE) {
							printf("ERR: Only channels 1-7 are allowed with secondary channel above for the US\n");
							return UAP_FAILURE;
						}
					} else if (strncmp
						   (country_80211d, "JP",
						    COUNTRY_CODE_LEN - 1)) {
						if (channel_tlv->chan_number >
						    DEFAULT_MAX_CHANNEL_ABOVE_NON_US)
						{
							printf("ERR: Only channels 1-9 are allowed with secondary channel above for" "non-Japan countries!\n");
							return UAP_FAILURE;
						}
					}
				}
			}
			if (!(channel_tlv->bandcfg.scanMode == SCAN_MODE_ACS)) {
				if (check_channel_validity_11d
				    (channel_tlv->chan_number, channel_tlv_band,
				     1) == UAP_FAILURE)
					return UAP_FAILURE;
				if (state_80211d) {
					/* Set country code to JP if channel is 8 or 12 and band is 5GHZ */
					if ((channel_tlv->chan_number <
					     MAX_CHANNELS_BG) &&
					    (channel_tlv_band == BAND_A))
						strncpy(country_80211d, "JP ",
							sizeof(country_80211d) -
							1);
				} else {
					if ((channel_tlv_band == BAND_A) &&
					    ((channel_tlv->chan_number == 8) ||
					     (channel_tlv->chan_number ==
					      12))) {
						printf("ERR:Invalid band for given channel\n");
						return UAP_FAILURE;
					}
				}
			} else {
				acs_mode_enabled = 1;
			}
#define JP_REGION_0xFE 0xFE
			if (fw.region_code == JP_REGION_0xFE &&
			    ((channel_tlv->chan_number == 12) ||
			     (channel_tlv->chan_number == 13))) {
				printf("ERR:Channel 12 or 13 is not allowed for region code 0xFE\n");
				return UAP_FAILURE;
			}
			break;
		case MRVL_CHANNELLIST_TLV_ID:
			chnlist_tlv = (tlvbuf_channel_list *)pcurrent_tlv;
			pchan_list =
				(channel_list *) & (chnlist_tlv->chan_list);
			if (tlv_len % sizeof(channel_list)) {
				printf("ERR:Invalid Scan channel list TLV!\n");
				return UAP_FAILURE;
			}
			chan_list_len = tlv_len / sizeof(channel_list);

			for (i = 0; i < chan_list_len; i++) {
				scan_channels_band = BAND_B | BAND_G;
				if ((scan_channels_band != BAND_A) &&
				    (pchan_list->bandcfg.chanBand ==
				     BAND_5GHZ)) {
					scan_channels_band = BAND_A;
				}
				if (check_channel_validity_11d
				    (pchan_list->chan_number,
				     scan_channels_band, 0) == UAP_FAILURE)
					return UAP_FAILURE;
				pchan_list++;
			}
			break;
		case MRVL_TX_POWER_TLV_ID:
			txpower_tlv = (tlvbuf_tx_power *)pcurrent_tlv;
			if (state_80211d) {
				if (check_tx_pwr_validity_11d
				    (txpower_tlv->tx_power_dbm) == UAP_FAILURE)
					return UAP_FAILURE;
			}
			break;
		case MRVL_CIPHER_PWK_TLV_ID:
			pwk_cipher_tlv = (tlvbuf_pwk_cipher *)pcurrent_tlv;
			pwk_cipher_tlv->protocol =
				uap_le16_to_cpu(pwk_cipher_tlv->protocol);
			if (pwk_cipher_tlv->protocol == PROTOCOL_WPA)
				pairwise_cipher_wpa =
					pwk_cipher_tlv->pairwise_cipher;
			else if (pwk_cipher_tlv->protocol == PROTOCOL_WPA2)
				pairwise_cipher_wpa2 =
					pwk_cipher_tlv->pairwise_cipher;
			break;
		case MRVL_CIPHER_GWK_TLV_ID:
			gwk_cipher_tlv = (tlvbuf_gwk_cipher *)pcurrent_tlv;
			group_cipher = gwk_cipher_tlv->group_cipher;
			break;
		case MRVL_AUTH_TLV_ID:
			auth_tlv = (tlvbuf_auth_mode *)pcurrent_tlv;
			break;
		case MRVL_PROTOCOL_TLV_ID:
			protocol_tlv = (tlvbuf_protocol *)pcurrent_tlv;
			protocol = uap_le16_to_cpu(protocol_tlv->protocol);
			break;
		case MRVL_WPA_PASSPHRASE_TLV_ID:
			passphrase_tlv = (tlvbuf_wpa_passphrase *)pcurrent_tlv;
			break;
		case MRVL_WEP_KEY_TLV_ID:
			wep_key_tlv = (tlvbuf_wep_key *)pcurrent_tlv;
			if (wep_key_tlv->is_default) {
				key_set = 1;
			}
			break;
		case HT_CAPABILITY_TLV_ID:
			ht_cap_tlv = (tlvbuf_htcap_t *)pcurrent_tlv;
			ht_cap_tlv->ht_cap.ht_cap_info =
				uap_le16_to_cpu(ht_cap_tlv->ht_cap.ht_cap_info);
			if (ht_cap_tlv->ht_cap.supported_mcs_set[0]) {
				enable_11n = 1;
				enable_40Mhz =
					IS_11N_40MHZ_ENABLED(ht_cap_tlv->ht_cap.
							     ht_cap_info);
				enable_20Mhz_sgi =
					IS_11N_20MHZ_SHORTGI_ENABLED
					(ht_cap_tlv->ht_cap.ht_cap_info);
				enable_gf =
					IS_11N_GF_ENABLED(ht_cap_tlv->ht_cap.
							  ht_cap_info);
			}
			break;
		case MRVL_RATES_TLV_ID:
			rates_tlv = (tlvbuf_rates *)pcurrent_tlv;
			op_rates_len = tlv_len;
			break;
		case MRVL_MCBC_DATA_RATE_TLV_ID:
			mcbc_data_rate_tlv =
				(tlvbuf_mcbc_data_rate *)pcurrent_tlv;
			mcbc_data_rate =
				uap_le16_to_cpu(mcbc_data_rate_tlv->
						mcbc_datarate);
			if (mcbc_data_rate &&
			    (is_mcbc_rate_valid((t_u8)mcbc_data_rate) !=
			     UAP_SUCCESS)) {
				printf("ERR: Invalid MCBC Data Rate \n");
				return UAP_FAILURE;
			}
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		pcurrent_tlv = (tlvbuf_header *)(pcurrent_tlv->data + tlv_len);
	}

	if ((protocol == PROTOCOL_STATIC_WEP) && !key_set) {
		printf("ERR:WEP keys not set!\n");
		return UAP_FAILURE;
	}
	if (auth_tlv == NULL) {
		printf("ERR: No authentication found\n");
		return UAP_FAILURE;
	}
	if ((auth_tlv->auth_mode == 1) && (protocol != PROTOCOL_STATIC_WEP)) {
		printf("ERR:Shared key authentication is not allowed for Open/WPA/WPA2/Mixed mode\n");
		return UAP_FAILURE;
	}

	if (passphrase_tlv == NULL) {
		printf("ERR: No passphrase found\n");
		return UAP_FAILURE;
	}
	if (((protocol == PROTOCOL_WPA) || (protocol == PROTOCOL_WPA2)
	     || (protocol == PROTOCOL_WPA2_MIXED))
	    && !(passphrase_tlv->length)) {
		printf("ERR:Passphrase must be set for WPA/WPA2/Mixed mode\n");
		return UAP_FAILURE;
	}
	if ((protocol == PROTOCOL_WPA) || (protocol == PROTOCOL_WPA2_MIXED)) {
		if (is_cipher_valid(pairwise_cipher_wpa, group_cipher) !=
		    UAP_SUCCESS) {
			printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
			return UAP_FAILURE;
		}
	}
	if ((protocol == PROTOCOL_WPA2) || (protocol == PROTOCOL_WPA2_MIXED)
		) {
		if (is_cipher_valid(pairwise_cipher_wpa2, group_cipher) !=
		    UAP_SUCCESS) {
			printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
			return UAP_FAILURE;
		}
	}

	if (chnlist_tlv == NULL) {
		printf("ERR: No channel list found\n");
		return UAP_FAILURE;
	}
	if (acs_mode_enabled) {
		pchan_list = (channel_list *) & (chnlist_tlv->chan_list);
		for (i = 0; i < chan_list_len; i++) {
			if ((!state_80211d) &&
			    (pchan_list->chan_number == MAX_CHANNELS_BG)
			    &&
			    (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1)
			     == 0)
				) {
				printf("ERR: Invalid scan channel 14, 802.11d disabled, country code JP!\n");
				return UAP_FAILURE;
			}

			if (fw.region_code == JP_REGION_0xFE &&
			    ((pchan_list->chan_number == 12) ||
			     (pchan_list->chan_number == 13))) {
				printf("ERR:Scan Channel 12 or 13 is not allowed for region code 0xFE\n");
				return UAP_FAILURE;
			}
			pchan_list++;
		}

		if (!state_80211d) {
			/* Block scan channels 8 and 12 in 5GHz band if 11d is not enabled and country code not set to JP */
			pchan_list =
				(channel_list *) & (chnlist_tlv->chan_list);
			for (i = 0; i < chan_list_len; i++) {
				if ((pchan_list->bandcfg.chanBand == BAND_5GHZ)
				    && ((pchan_list->chan_number == 8) ||
					(pchan_list->chan_number == 12))) {
					printf("ERR: Invalid band for scan channel %d\n", pchan_list->chan_number);
					return UAP_FAILURE;
				}
				pchan_list++;
			}
		}
		if (state_80211d) {
			/* Set default country code to US */
			strncpy(country_80211d, "US ",
				sizeof(country_80211d) - 1);
			pchan_list =
				(channel_list *) & (chnlist_tlv->chan_list);
			for (i = 0; i < chan_list_len; i++) {
				/* Set country code to JP if channel is 8 or 12 and band is 5GHZ */
				if ((pchan_list->chan_number < MAX_CHANNELS_BG)
				    && (scan_channels_band == BAND_A)) {
					strncpy(country_80211d, "JP ",
						sizeof(country_80211d) - 1);
				}
				if (check_channel_validity_11d
				    (pchan_list->chan_number,
				     scan_channels_band, 0) == UAP_FAILURE)
					return UAP_FAILURE;
				pchan_list++;
			}
			if (update_domain_info
			    (country_80211d,
			     scan_channels_band) == UAP_FAILURE) {
				return UAP_FAILURE;
			}
		}
	}
#ifdef WIFI_DIRECT_SUPPORT
	if (strncmp(dev_name, "wfd", 3))
#endif
		if ((!acs_mode_enabled &&
		     (channel_tlv_band == (BAND_B | BAND_G)))
		    || (acs_mode_enabled &&
			(scan_channels_band == (BAND_B | BAND_G)))) {
			if (rates_tlv == NULL) {
				printf("ERR: No rates found\n");
				return UAP_FAILURE;
			}
			for (i = 0; i < op_rates_len; i++) {
				rate = rates_tlv->
					operational_rates[i] &
					~BASIC_RATE_SET_BIT;
				switch (rate) {
				case 2:
					rate_bitmap |= BITMAP_RATE_1M;
					break;
				case 4:
					rate_bitmap |= BITMAP_RATE_2M;
					break;
				case 11:
					rate_bitmap |= BITMAP_RATE_5_5M;
					break;
				case 22:
					rate_bitmap |= BITMAP_RATE_11M;
					break;
				}
			}
			if ((rate_bitmap & B_RATE_MANDATORY) !=
			    B_RATE_MANDATORY) {
				if (acs_mode_enabled)
					printf("ERR: Rates/Scan channels do not match!\n");
				else
					printf("ERR: Rates/Channel do not match!\n");
				return UAP_FAILURE;
			}
		}
	if ((!acs_mode_enabled && (channel_tlv_band == BAND_A))
	    || (acs_mode_enabled && (scan_channels_band == BAND_A))) {
		if (rates_tlv == NULL) {
			printf("ERR: No rates found\n");
			return UAP_FAILURE;
		}
		for (i = 0; i < op_rates_len; i++) {
			rate = rates_tlv->
				operational_rates[i] & ~BASIC_RATE_SET_BIT;
			switch (rate) {
			case 2:
			case 4:
			case 11:
			case 22:
				if (acs_mode_enabled)
					printf("ERR: Rates/Scan channels do not match!\n");
				else
					printf("ERR: Rates/Channel do not match!\n");
				return UAP_FAILURE;
			}
		}
		state_80211h = 1;
		if (sg_snmp_mib
		    (ACTION_SET, OID_80211H_ENABLE, sizeof(state_80211h),
		     &state_80211h)
		    == UAP_FAILURE) {
			return UAP_FAILURE;
		}
		if (update_domain_info(country_80211d, BAND_A) == UAP_FAILURE) {
			return UAP_FAILURE;
		}
	}
#define ISSUPP_CHANWIDTH40(Dot11nDevCap) (Dot11nDevCap & MBIT(17))
#define ISSUPP_SHORTGI40(Dot11nDevCap) (Dot11nDevCap & MBIT(24))

	if (enable_40Mhz) {
		if (!ISSUPP_CHANWIDTH40(fw.hw_dot_11n_dev_cap)) {
			printf("ERR: It's not support HT40 from Hardware Cap\n");
			return UAP_FAILURE;
		}
	}

	if (enable_11n) {
		/*For protocol = Mixed, 11n enabled, only allow TKIP cipher for WPA protocol, not for WPA2 */
		if ((protocol == PROTOCOL_WPA2_MIXED) &&
		    (pairwise_cipher_wpa2 == CIPHER_TKIP)) {
			printf("ERR: WPA2 pairwise cipher cannot be TKIP when AP operates in 802.11n Mixed mode.\n");
			return UAP_FAILURE;
		} else if (protocol == PROTOCOL_STATIC_WEP) {
			printf("ERR: WEP cannot be used when AP operates in 802.11n mode.\n");
			return UAP_FAILURE;
		}
	}
	if ((tx_rate_config.rate == UAP_RATE_INDEX_MCS32) && !enable_40Mhz) {
		printf("ERR:uAP must be configured to operate in 40MHz if tx_rate is MCS32\n");
		return UAP_FAILURE;
	}
	if (enable_20Mhz_sgi && enable_gf) {
		if ((tx_rate_config.rate >= UAP_RATE_INDEX_MCS0) &&
		    (tx_rate_config.rate <= UAP_RATE_INDEX_MCS7)) {
			printf("ERR: Invalid tx_rate for uAP in (20MHz Short GI + Green Field) mode\n");
			return UAP_FAILURE;
		}
	}
	mcs_rate_index =
		get_rate_index_from_bitmap(tx_rate_config.bitmap_rates,
					   sizeof(tx_rate_config.bitmap_rates));
	if ((mcs_rate_index >= UAP_RATE_BITMAP_MCS0) &&
	    (mcs_rate_index <= UAP_RATE_BITMAP_MCS127)) {
		mcs_rate_index -= (UAP_RATE_BITMAP_MCS0 - UAP_RATE_INDEX_MCS0);
		if ((mcs_rate_index == UAP_RATE_INDEX_MCS32) & !enable_40Mhz) {
			printf("ERR:uAP must be configured to operate in 40MHz if rate_bitmap contains MCS32\n");
			return UAP_FAILURE;
		}
		if (enable_20Mhz_sgi && enable_gf) {
			if ((mcs_rate_index >= UAP_RATE_INDEX_MCS0) &&
			    (mcs_rate_index <= UAP_RATE_INDEX_MCS7)) {
				printf("ERR: Invalid tx_rate for uAP in (20MHz Short GI + Green Field) mode\n");
				return UAP_FAILURE;
			}
		}
	}

	/* if 11d enabled, Channel 14, country code "JP", only B rates are allowed */
	if ((channel == 14) && state_80211d &&
	    (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1) == 0)) {
		if (rates_tlv == NULL) {
			printf("ERR:No rates found\n");
			return UAP_FAILURE;
		}
		if (enable_11n) {
			printf("ERR:11n must be disabled and the only B rates are allowed if 11d enabled and country code is JP with channel 14.\n");
			return UAP_FAILURE;
		}
		for (i = 0; i < op_rates_len; i++) {
			rate = rates_tlv->
				operational_rates[i] & ~BASIC_RATE_SET_BIT;
			switch (rate) {
			case 2:
			case 4:
			case 11:
			case 22:
				break;
			default:
				printf("ERR:If 11d enabled, channel 14 and country code is JP, the only B rates are allowed.\n");
				return UAP_FAILURE;
			}
		}
	}

	/* Channels 14, 165 are not allowed to operate in 40MHz mode */
	if (!acs_mode_enabled && enable_40Mhz) {
		if ((channel == 14)
		    || (channel == 165)
			) {
			printf("ERR:Invalid channel %d for 40MHz operation\n",
			       channel);
			return UAP_FAILURE;
		} else if (!secondary_ch_set) {
			printf("ERR:Secondary channel should be set when 40Mhz is enabled!\n");
			return UAP_FAILURE;
		}
	}
	/* Channels 14, 140, 165 are not allowed to operate in 40MHz mode */
	if (acs_mode_enabled && enable_40Mhz) {
		if (chnlist_tlv == NULL) {
			printf("ERR: No channel list found\n");
			return UAP_FAILURE;
		}
		pchan_list = (channel_list *) & (chnlist_tlv->chan_list);
		for (i = 0; i < chan_list_len; i++) {
			if ((pchan_list->chan_number != 14)
			    && (pchan_list->chan_number != 140) &&
			    (pchan_list->chan_number != 165)
				) {
				flag = 1;
				break;
			}
			pchan_list++;
		}
		if (!flag) {
			printf("ERR:Invalid channels in scan channel list for 40MHz operation\n");
			return UAP_FAILURE;
		}
	}

	if (0 == get_fw_info(&fw)) {
		/*check whether support 802.11AC through BAND_AAC bit */
		if (fw.fw_bands & BAND_AAC) {
			ret = get_802_11ac_cfg(&vhtcfg);
			if (UAP_SUCCESS == ret) {
				/* Note: When 11AC is disabled, FW sets vht_rx_mcs to 0xffff */
				if ((vhtcfg.vht_rx_mcs != 0xffff) &&
				    (!enable_11n)) {
					printf("ERR: 11ac enable while 11n disable, it is forbidden! \n");
					return UAP_FAILURE;
				}
			} else {
				printf("Don't support 802.11AC \n");
				return UAP_SUCCESS;
			}
		} else
			printf("No support 802 11AC.\n");
	} else {
		printf("ERR: get_fw_info fail\n");
		return UAP_FAILURE;
	}

	return ret;
}

/**
 *  @brief Checks current bss configuration
 *
 *  @param buf     pointer to bss_config_t
 *  @return        UAP_SUCCESS/UAP_FAILURE
 */
int
check_bss_config(t_u8 *buf)
{
	bss_config_t *bss_config = NULL;;
	t_u8 acs_mode_enabled = 0;
	t_u8 *cbuf = NULL;
	apcmdbuf_cfg_80211d *cmd_buf = NULL;
	t_u16 buf_len, cmd_len;
	char country_80211d[4] = { 'U', 'S', ' ', 0 };
	t_u8 state_80211h = 0;
	int channel_tlv_band = BAND_B | BAND_G;
	t_u8 secondary_ch_set = 0;
	int scan_channels_band = BAND_B | BAND_G;
	t_u8 state_80211d = 0;
	t_u8 rate = 0;
	t_u32 rate_bitmap = 0;
	t_u16 enable_40Mhz = 0;
	t_u16 enable_20Mhz_sgi = 0;
	t_u16 enable_gf = 0;
	t_u8 enable_11n = 0;
	int flag = 0;
	fw_info fw;
	int i = 0, ret = UAP_SUCCESS;
	tx_rate_cfg_t tx_rate_config;
	t_u32 mcs_rate_index = 0;
	struct eth_priv_vhtcfg vhtcfg = { 0 };

#define BITMAP_RATE_1M         0x01
#define BITMAP_RATE_2M         0x02
#define BITMAP_RATE_5_5M       0x04
#define BITMAP_RATE_11M        0x8
#define B_RATE_MANDATORY       0x0f

	if (NULL == buf) {
		printf("ERR: No buffer for bss_configure!\n");
		return UAP_FAILURE;
	}

	bss_config = (bss_config_t *)buf;

	ret = sg_snmp_mib(ACTION_GET, OID_80211D_ENABLE, sizeof(state_80211d),
			  &state_80211d);

	buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	cbuf = (t_u8 *)malloc(buf_len);
	if (!cbuf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(cbuf, 0, buf_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_cfg_80211d *)cbuf;
	cmd_len = (sizeof(apcmdbuf_cfg_80211d) - sizeof(domain_param_t));
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->action = uap_cpu_to_le16(ACTION_GET);
	cmd_buf->cmd_code = HostCmd_CMD_802_11D_DOMAIN_INFO;
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	if (ret == UAP_SUCCESS && cmd_buf->result == CMD_SUCCESS) {
		if (cmd_buf->domain.country_code[0] ||
		    cmd_buf->domain.country_code[1] ||
		    cmd_buf->domain.country_code[2]) {
			strncpy(country_80211d,
				(char *)cmd_buf->domain.country_code,
				COUNTRY_CODE_LEN);
		}
	}
	free(cbuf);

	/* get region code to handle special cases */
	memset(&fw, 0, sizeof(fw));
	if (get_fw_info(&fw)) {
		printf("Could not get fw info!\n");
		return UAP_FAILURE;
	}

	if (get_tx_rate_cfg(&tx_rate_config) == UAP_FAILURE) {
		printf("Could not get tx_rate_cfg!\n");
		return UAP_FAILURE;
	}

	if ((fw.fw_bands & (BAND_B | BAND_G | BAND_GN | BAND_GAC))
	    && !(fw.fw_bands & (BAND_A | BAND_AAC))
		) {
		if (bss_config->channel > MAX_CHANNELS_BG) {
			printf("ERR: Invalid channel in 2.4GHz band!\n");
			return UAP_FAILURE;
		}
	}

	if (!(fw.fw_bands & (BAND_B | BAND_G
			     | BAND_GN
			     | BAND_GAC)) && (fw.fw_bands & (BAND_A
							     | BAND_AAC))) {
		if (bss_config->channel < 36 ||
		    bss_config->channel > MAX_CHANNELS) {
			printf("ERR: Invalid channel in 5GHz band!\n");
			return UAP_FAILURE;
		}
	}

	if ((!state_80211d) && (bss_config->channel == MAX_CHANNELS_BG)
	    && (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1) == 0)
		) {
		printf("ERR: Invalid channel 14, 802.11d disabled, country code JP!\n");
		return UAP_FAILURE;
	}

	if (bss_config->bandcfg.chanBand == BAND_5GHZ)
		channel_tlv_band = BAND_A;

	secondary_ch_set = bss_config->bandcfg.chan2Offset;
	if (channel_tlv_band != BAND_A) {
		/* When country code is not Japan, allow channels 5-11 for US and 5-13 for non-US
		 * only for secondary channel below */
		if (secondary_ch_set == SEC_CHAN_BELOW) {
			if (!strncmp
			    (country_80211d, "US", COUNTRY_CODE_LEN - 1)) {
				if (bss_config->channel >
				    DEFAULT_MAX_CHANNEL_BELOW) {
					printf("ERR: Only channels 5-11 are allowed with secondary channel below for the US\n");
					return UAP_FAILURE;
				}
			} else if (strncmp
				   (country_80211d, "JP",
				    COUNTRY_CODE_LEN - 1)) {
				if (bss_config->channel >
				    DEFAULT_MAX_CHANNEL_BELOW_NON_US) {
					printf("ERR: Only channels 5-13 are allowed with secondary channel below for" "non-Japan countries!\n");
					return UAP_FAILURE;
				}
			}
		}

		/* When country code is not Japan, allow channels 1-7 for US and 1-9 for non-US
		 * only for secondary channel above */
		if (secondary_ch_set == SEC_CHAN_ABOVE) {
			if (!strncmp
			    (country_80211d, "US", COUNTRY_CODE_LEN - 1)) {
				if (bss_config->channel >
				    DEFAULT_MAX_CHANNEL_ABOVE) {
					printf("ERR: Only channels 1-7 are allowed with secondary channel above for the US\n");
					return UAP_FAILURE;
				}
			} else if (strncmp
				   (country_80211d, "JP",
				    COUNTRY_CODE_LEN - 1)) {
				if (bss_config->channel >
				    DEFAULT_MAX_CHANNEL_ABOVE_NON_US) {
					printf("ERR: Only channels 1-9 are allowed with secondary channel above for" "non-Japan countries!\n");
					return UAP_FAILURE;
				}
			}
		}
	}

	if (!(bss_config->bandcfg.scanMode == SCAN_MODE_ACS)) {
		if (check_channel_validity_11d
		    (bss_config->channel, channel_tlv_band, 1) == UAP_FAILURE)
			return UAP_FAILURE;
		if (state_80211d) {
			/* Set country code to JP if channel is 8 or 12 and band is 5GHZ */
			if ((bss_config->channel < MAX_CHANNELS_BG) &&
			    (channel_tlv_band == BAND_A))
				strncpy(country_80211d, "JP ",
					sizeof(country_80211d) - 1);
		} else {
			if ((channel_tlv_band == BAND_A) &&
			    ((bss_config->channel == 8) ||
			     (bss_config->channel == 12))) {
				printf("ERR:Invalid band for given channel\n");
				return UAP_FAILURE;
			}
		}
	} else {
		acs_mode_enabled = 1;
	}

#define JP_REGION_0xFE 0xFE
	if (fw.region_code == JP_REGION_0xFE &&
	    ((bss_config->channel == 12) || (bss_config->channel == 13))) {
		printf("ERR:Channel 12 or 13 is not allowed for region code 0xFE\n");
		return UAP_FAILURE;
	}

	for (i = 0; i < bss_config->num_of_chan; i++) {
		scan_channels_band = BAND_B | BAND_G;
		if ((scan_channels_band != BAND_A) &&
		    (bss_config->chan_list[i].bandcfg.chanBand == BAND_5GHZ)) {
			scan_channels_band = BAND_A;
		}
		if (check_channel_validity_11d
		    (bss_config->chan_list[i].chan_number, scan_channels_band,
		     0) == UAP_FAILURE)
			return UAP_FAILURE;
	}

	if (state_80211d) {
		if (check_tx_pwr_validity_11d(bss_config->tx_power_level) ==
		    UAP_FAILURE)
			return UAP_FAILURE;
	}

	if (bss_config->supported_mcs_set[0]) {
		enable_11n = 1;
		enable_40Mhz = IS_11N_40MHZ_ENABLED(bss_config->ht_cap_info);
		enable_20Mhz_sgi =
			IS_11N_20MHZ_SHORTGI_ENABLED(bss_config->ht_cap_info);
		enable_gf = IS_11N_GF_ENABLED(bss_config->ht_cap_info);
	}

	if (bss_config->mcbc_data_rate &&
	    (is_mcbc_rate_valid(bss_config->mcbc_data_rate) != UAP_SUCCESS)) {
		printf("ERR: Invalid MCBC Data Rate \n");
		return UAP_FAILURE;
	}

	if ((bss_config->protocol == PROTOCOL_STATIC_WEP)
	    && (0 == bss_config->wep_cfg.key0.is_default)
	    && (0 == bss_config->wep_cfg.key1.is_default)
	    && (0 == bss_config->wep_cfg.key2.is_default)
	    && (0 == bss_config->wep_cfg.key3.is_default)) {
		printf("ERR:WEP keys not set!\n");
		return UAP_FAILURE;
	}

	if ((bss_config->auth_mode == 1) &&
	    (bss_config->protocol != PROTOCOL_STATIC_WEP)) {
		printf("ERR:Shared key authentication is not allowed for Open/WPA/WPA2/Mixed mode\n");
		return UAP_FAILURE;
	}

	if (((bss_config->protocol == PROTOCOL_WPA) ||
	     (bss_config->protocol == PROTOCOL_WPA2)
	     || (bss_config->protocol == PROTOCOL_WPA2_MIXED)) &&
	    !(bss_config->wpa_cfg.length)) {
		printf("ERR:Passphrase must be set for WPA/WPA2/Mixed mode\n");
		return UAP_FAILURE;
	}

	if ((bss_config->protocol == PROTOCOL_WPA) ||
	    (bss_config->protocol == PROTOCOL_WPA2_MIXED)) {
		if (is_cipher_valid
		    (bss_config->wpa_cfg.pairwise_cipher_wpa,
		     bss_config->wpa_cfg.group_cipher) != UAP_SUCCESS) {
			printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
			return UAP_FAILURE;
		}
	}
	if ((bss_config->protocol == PROTOCOL_WPA2) ||
	    (bss_config->protocol == PROTOCOL_WPA2_MIXED)
		) {
		if (is_cipher_valid
		    (bss_config->wpa_cfg.pairwise_cipher_wpa2,
		     bss_config->wpa_cfg.group_cipher) != UAP_SUCCESS) {
			printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
			return UAP_FAILURE;
		}
	}

	if (0 == bss_config->num_of_chan) {
		printf("ERR: No channel list found\n");
		return UAP_FAILURE;
	}

	if (acs_mode_enabled) {
		for (i = 0; i < bss_config->num_of_chan; i++) {
			if ((!state_80211d) &&
			    (bss_config->chan_list[i].chan_number ==
			     MAX_CHANNELS_BG)
			    &&
			    (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1)
			     == 0)
				) {
				printf("ERR: Invalid scan channel 14, 802.11d disabled, country code JP!\n");
				return UAP_FAILURE;
			}

			if (fw.region_code == JP_REGION_0xFE &&
			    ((bss_config->chan_list[i].chan_number == 12) ||
			     (bss_config->chan_list[i].chan_number == 13))) {
				printf("ERR:Scan Channel 12 or 13 is not allowed for region code 0xFE\n");
				return UAP_FAILURE;
			}
		}

		if (!state_80211d) {
			/* Block scan channels 8 and 12 in 5GHz band if 11d is not enabled and country code not set to JP */
			for (i = 0; i < bss_config->num_of_chan; i++) {
				if ((bss_config->chan_list[i].bandcfg.
				     chanBand == BAND_5GHZ)
				    &&
				    ((bss_config->chan_list[i].chan_number == 8)
				     || (bss_config->chan_list[i].chan_number ==
					 12))) {
					printf("ERR: Invalid band for scan channel %d\n", bss_config->chan_list[i].chan_number);
					return UAP_FAILURE;
				}
			}
		}

		if (state_80211d) {
			/* Set default country code to US */
			strncpy(country_80211d, "US ",
				sizeof(country_80211d) - 1);
			for (i = 0; i < bss_config->num_of_chan; i++) {
				/* Set country code to JP if channel is 8 or 12 and band is 5GHZ */
				if ((bss_config->chan_list[i].chan_number <
				     MAX_CHANNELS_BG) &&
				    (scan_channels_band == BAND_A)) {
					strncpy(country_80211d, "JP ",
						sizeof(country_80211d) - 1);
				}
				if (check_channel_validity_11d
				    (bss_config->chan_list[i].chan_number,
				     scan_channels_band, 0) == UAP_FAILURE)
					return UAP_FAILURE;
			}
			if (update_domain_info
			    (country_80211d,
			     scan_channels_band) == UAP_FAILURE) {
				return UAP_FAILURE;
			}
		}
	}
#ifdef WIFI_DIRECT_SUPPORT
	if (strncmp(dev_name, "wfd", 3))
#endif
		if ((!acs_mode_enabled &&
		     (channel_tlv_band == (BAND_B | BAND_G)))
		    || (acs_mode_enabled &&
			(scan_channels_band == (BAND_B | BAND_G)))) {
			if (0 == bss_config->rates[0]) {
				printf("ERR: No rates found\n");
				return UAP_FAILURE;
			}
			for (i = 0; i < MAX_DATA_RATES && bss_config->rates[i];
			     i++) {
				rate = bss_config->
					rates[i] & ~BASIC_RATE_SET_BIT;
				switch (rate) {
				case 2:
					rate_bitmap |= BITMAP_RATE_1M;
					break;
				case 4:
					rate_bitmap |= BITMAP_RATE_2M;
					break;
				case 11:
					rate_bitmap |= BITMAP_RATE_5_5M;
					break;
				case 22:
					rate_bitmap |= BITMAP_RATE_11M;
					break;
				}
			}
			if ((rate_bitmap & B_RATE_MANDATORY) !=
			    B_RATE_MANDATORY) {
				if (acs_mode_enabled)
					printf("ERR: Rates/Scan channels do not match!\n");
				else
					printf("ERR: Rates/Channel do not match!\n");
				return UAP_FAILURE;
			}
		}
	if ((!acs_mode_enabled && (channel_tlv_band == BAND_A))
	    || (acs_mode_enabled && (scan_channels_band == BAND_A))) {
		if (0 == bss_config->rates[0]) {
			printf("ERR: No rates found\n");
			return UAP_FAILURE;
		}
		for (i = 0; i < MAX_DATA_RATES && bss_config->rates[i]; i++) {
			rate = bss_config->rates[i] & ~BASIC_RATE_SET_BIT;
			switch (rate) {
			case 2:
			case 4:
			case 11:
			case 22:
				if (acs_mode_enabled)
					printf("ERR: Rates/Scan channels do not match!\n");
				else
					printf("ERR: Rates/Channel do not match!\n");
				return UAP_FAILURE;
			}
		}
		state_80211h = 1;
		if (sg_snmp_mib
		    (ACTION_SET, OID_80211H_ENABLE, sizeof(state_80211h),
		     &state_80211h)
		    == UAP_FAILURE) {
			return UAP_FAILURE;
		}
		if (update_domain_info(country_80211d, BAND_A) == UAP_FAILURE) {
			return UAP_FAILURE;
		}
	}
#define ISSUPP_CHANWIDTH40(Dot11nDevCap) (Dot11nDevCap & MBIT(17))
#define ISSUPP_SHORTGI40(Dot11nDevCap) (Dot11nDevCap & MBIT(24))

	if (enable_40Mhz) {
		if (!ISSUPP_CHANWIDTH40(fw.hw_dot_11n_dev_cap)) {
			printf("ERR: It's not support HT40 from Hardware Cap\n");
			return UAP_FAILURE;
		}
	}

	if (enable_11n) {
		/*For protocol = Mixed, 11n enabled, only allow TKIP cipher for WPA protocol, not for WPA2 */
		if ((bss_config->protocol == PROTOCOL_WPA2_MIXED) &&
		    (bss_config->wpa_cfg.pairwise_cipher_wpa2 == CIPHER_TKIP)) {
			printf("ERR: WPA2 pairwise cipher cannot be TKIP when AP operates in 802.11n Mixed mode.\n");
			return UAP_FAILURE;
		} else if (bss_config->protocol == PROTOCOL_STATIC_WEP) {
			printf("ERR: WEP cannot be used when AP operates in 802.11n mode.\n");
			return UAP_FAILURE;
		}
	}
	if ((tx_rate_config.rate == UAP_RATE_INDEX_MCS32) && !enable_40Mhz) {
		printf("ERR:uAP must be configured to operate in 40MHz if tx_rate is MCS32\n");
		return UAP_FAILURE;
	}
	if (enable_20Mhz_sgi && enable_gf) {
		if ((tx_rate_config.rate >= UAP_RATE_INDEX_MCS0) &&
		    (tx_rate_config.rate <= UAP_RATE_INDEX_MCS7)) {
			printf("ERR: Invalid tx_rate for uAP in (20MHz Short GI + Green Field) mode\n");
			return UAP_FAILURE;
		}
	}
	mcs_rate_index =
		get_rate_index_from_bitmap(tx_rate_config.bitmap_rates,
					   sizeof(tx_rate_config.bitmap_rates));
	if ((mcs_rate_index >= UAP_RATE_BITMAP_MCS0) &&
	    (mcs_rate_index <= UAP_RATE_BITMAP_MCS127)) {
		mcs_rate_index -= (UAP_RATE_BITMAP_MCS0 - UAP_RATE_INDEX_MCS0);
		if ((mcs_rate_index == UAP_RATE_INDEX_MCS32) & !enable_40Mhz) {
			printf("ERR:uAP must be configured to operate in 40MHz if rate_bitmap contains MCS32\n");
			return UAP_FAILURE;
		}
		if (enable_20Mhz_sgi && enable_gf) {
			if ((mcs_rate_index >= UAP_RATE_INDEX_MCS0) &&
			    (mcs_rate_index <= UAP_RATE_INDEX_MCS7)) {
				printf("ERR: Invalid tx_rate for uAP in (20MHz Short GI + Green Field) mode\n");
				return UAP_FAILURE;
			}
		}
	}

/* if 11d enabled, Channel 14, country code "JP", only B rates are allowed*/
	if ((bss_config->channel == 14) && state_80211d &&
	    (strncmp(country_80211d, "JP", COUNTRY_CODE_LEN - 1) == 0)) {
		if (0 == bss_config->rates[0]) {
			printf("ERR:No rates found\n");
			return UAP_FAILURE;
		}
		if (enable_11n) {
			printf("ERR:11n must be disabled and the only B rates are allowed if 11d enabled and country code is JP with channel 14.\n");
			return UAP_FAILURE;
		}
		for (i = 0; i < MAX_DATA_RATES && bss_config->rates[i]; i++) {
			rate = bss_config->rates[i] & ~BASIC_RATE_SET_BIT;
			switch (rate) {
			case 2:
			case 4:
			case 11:
			case 22:
				break;
			default:
				printf("ERR:If 11d enabled, channel 14 and country code is JP, the only B rates are allowed.\n");
				return UAP_FAILURE;
			}
		}
	}

	/* Channels 14, 165 are not allowed to operate in 40MHz mode */
	if (!acs_mode_enabled && enable_40Mhz) {
		if ((bss_config->channel == 14)
		    || (bss_config->channel == 165)
			) {
			printf("ERR:Invalid channel %d for 40MHz operation\n",
			       bss_config->channel);
			return UAP_FAILURE;
		} else if (!secondary_ch_set) {
			printf("ERR:Secondary channel should be set when 40Mhz is enabled!\n");
			return UAP_FAILURE;
		}
	}
/* Channels 14, 140, 165 are not allowed to operate in 40MHz mode */
	if (acs_mode_enabled && enable_40Mhz) {
		if (bss_config->num_of_chan == 0) {
			printf("ERR: No channel list found\n");
			return UAP_FAILURE;
		}
		for (i = 0; i < bss_config->num_of_chan; i++) {
			if ((bss_config->chan_list[i].chan_number != 14)
			    && (bss_config->chan_list[i].chan_number != 140) &&
			    (bss_config->chan_list[i].chan_number != 165)
				) {
				flag = 1;
				break;
			}
		}
		if (!flag) {
			printf("ERR:Invalid channels in scan channel list for 40MHz operation\n");
			return UAP_FAILURE;
		}
	}
	if (0 == get_fw_info(&fw)) {
		/*check whether support 802.11AC through BAND_AAC bit */
		if (fw.fw_bands & BAND_AAC) {
			ret = get_802_11ac_cfg(&vhtcfg);
			if (UAP_SUCCESS == ret) {
				/* Note: When 11AC is disabled, FW sets vht_rx_mcs to 0xffff */
				if ((vhtcfg.vht_rx_mcs != 0xffff) &&
				    (!enable_11n)) {
					printf("ERR: 11ac enable while 11n disable, it is forbidden! \n");
					return UAP_FAILURE;
				}
			} else {
				printf("Don't support 802.11AC \n");
				return UAP_SUCCESS;
			}
		} else
			printf("No support 802 11AC.\n");
	} else {
		printf("ERR: get_fw_info fail\n");
		return UAP_FAILURE;
	}

	return ret;
}

/**
 *  @brief Send read/write command along with register details to the driver
 *  @param reg      Reg type
 *  @param offset   Pointer to register offset string
 *  @param strvalue Pointer to value string
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_regrdwr_process(int reg, char *offset, char *strvalue)
{
	apcmdbuf_reg_rdwr *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	char *whichreg;

	/* Alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_reg_rdwr);
	cmd_buf = (apcmdbuf_reg_rdwr *)buf;

	/* Fill the command buffer */
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;

	switch (reg) {
	case CMD_MAC:
		whichreg = "MAC";
		cmd_buf->cmd_code = HostCmd_CMD_MAC_REG_ACCESS;
		break;
	case CMD_BBP:
		whichreg = "BBP";
		cmd_buf->cmd_code = HostCmd_CMD_BBP_REG_ACCESS;
		break;
	case CMD_RF:
		cmd_buf->cmd_code = HostCmd_CMD_RF_REG_ACCESS;
		whichreg = "RF";
		break;
	default:
		printf("Invalid register set specified.\n");
		free(buf);
		return UAP_FAILURE;
	}
	if (strvalue) {
		cmd_buf->action = 1;	// WRITE
	} else {
		cmd_buf->action = 0;	// READ
	}
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->offset = A2HEXDECIMAL(offset);
	cmd_buf->offset = uap_cpu_to_le16(cmd_buf->offset);
	if (strvalue) {
		cmd_buf->value = A2HEXDECIMAL(strvalue);
		cmd_buf->value = uap_cpu_to_le32(cmd_buf->value);
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Successfully executed the command\n");
			printf("%s[0x%04hx] = 0x%08x\n",
			       whichreg, uap_le16_to_cpu(cmd_buf->offset),
			       uap_le32_to_cpu(cmd_buf->value));
		} else {
			printf("ERR:Command sending failed!\n");
			free(buf);
			return UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		free(buf);
		return UAP_FAILURE;
	}

	free(buf);
	return UAP_SUCCESS;
}

/**
 *  @brief Send read command for EEPROM
 *
 *  Usage: "Usage : rdeeprom <offset> <byteCount>"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_read_eeprom(int argc, char *argv[])
{
	apcmdbuf_eeprom_access *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	t_u16 byte_count, offset;
	int ret = UAP_SUCCESS;
	int opt;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_apcmd_read_eeprom_usage();
			return UAP_FAILURE;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (!argc ||
	    (argc && is_input_valid(RDEEPROM, argc, argv) != UAP_SUCCESS)) {
		print_apcmd_read_eeprom_usage();
		return UAP_FAILURE;
	}
	offset = A2HEXDECIMAL(argv[0]);
	byte_count = A2HEXDECIMAL(argv[1]);

	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_eeprom_access *)buf;
	cmd_len = sizeof(apcmdbuf_eeprom_access);

	cmd_buf->size = sizeof(apcmdbuf_eeprom_access) - BUF_HEADER_SIZE;
	cmd_buf->result = 0;
	cmd_buf->seq_num = 0;
	cmd_buf->action = 0;

	cmd_buf->cmd_code = HostCmd_EEPROM_ACCESS;
	cmd_buf->offset = uap_cpu_to_le16(offset);
	cmd_buf->byte_count = uap_cpu_to_le16(byte_count);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Successfully executed the command\n");
			byte_count = uap_le16_to_cpu(cmd_buf->byte_count);
			offset = uap_le16_to_cpu(cmd_buf->offset);
			hexdump_data("EEPROM", (void *)cmd_buf->value,
				     byte_count, ' ');
		} else {
			printf("ERR:Command Response incorrect!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}

	free(buf);
	return ret;
}

/**
 *  @brief Show usage information for the regrdwr command
 *  command
 *
 *  $return         N/A
 */
void
print_regrdwr_usage(void)
{
	printf("\nUsage : uaputl.exe regrdwr <TYPE> <OFFSET> [value]\n");
	printf("\nTYPE Options: 1     - read/write MAC register");
	printf("\n              2     - read/write BBP register");
	printf("\n              3     - read/write RF register");
	printf("\n");
	return;

}

/**
 *  @brief Provides interface to perform read/write operations on regsiters
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_regrdwr(int argc, char *argv[])
{
	int opt;
	t_s32 reg;
	int ret = UAP_SUCCESS;
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_regrdwr_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 2) || (argc > 3)) {
		printf("ERR:wrong arguments.\n");
		print_regrdwr_usage();
		return UAP_FAILURE;
	}
	if ((atoi(argv[0]) != 1) && (atoi(argv[0]) != 2) &&
	    (atoi(argv[0]) != 3)) {
		printf("ERR:Illegal register type %s. Must be either '1','2' or '3'.\n", argv[0]);
		print_regrdwr_usage();
		return UAP_FAILURE;
	}
	reg = atoi(argv[0]);
	ret = apcmd_regrdwr_process(reg, argv[1], argc > 2 ? argv[2] : NULL);
	return ret;
}

/**
 *    @brief Show usage information for the memaccess command
 *    command
 *
 *    @return         N/A
 */
void
print_memaccess_usage(void)
{
	printf("\nUsage : uaputl.exe memaccess <ADDRESS> [value]\n");
	printf("\nRead/Write memory location");
	printf("\nADDRESS: Address of the memory location to be read/written");
	printf("\nValue  : Value to be written at that address\n");
	return;
}

/**
 *  @brief Provides interface to perform read/write memory location
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_memaccess(int argc, char *argv[])
{
	int opt;
	apcmdbuf_mem_access *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	char *address = NULL;
	char *value = NULL;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_memaccess_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc < 1) || (argc > 2)) {
		printf("ERR:wrong arguments.\n");
		print_memaccess_usage();
		return UAP_FAILURE;
	}

	address = argv[0];
	if (argc == 2)
		value = argv[1];

	/* Alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);

	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);
	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_mem_access);
	cmd_buf = (apcmdbuf_mem_access *)buf;

	/* Fill the command buffer */
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->cmd_code = HostCmd_CMD_MEM_ACCESS;

	if (value)
		cmd_buf->action = 1;	// WRITE
	else
		cmd_buf->action = 0;	// READ

	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->address = A2HEXDECIMAL(address);
	cmd_buf->address = uap_cpu_to_le32(cmd_buf->address);

	if (value) {
		cmd_buf->value = A2HEXDECIMAL(value);
		cmd_buf->value = uap_cpu_to_le32(cmd_buf->value);
	}

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Successfully executed the command\n");
			printf("[0x%04x] = 0x%08x\n",
			       uap_le32_to_cpu(cmd_buf->address),
			       uap_le32_to_cpu(cmd_buf->value));
		} else {
			printf("ERR:Command sending failed!\n");
			free(buf);
			return UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		free(buf);
		return UAP_FAILURE;
	}
	free(buf);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the bss_config command
 *
 *  $return         N/A
 */
void
print_bss_config_usage(void)
{
	printf("\nUsage : bss_config [CONFIG_FILE]\n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("CONFIG_FILE is file containing all the BSS settings.\n");
	return;
}

/**
 *  @brief Read the BSS profile and populate structure
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @param bss      Pointer to BSS configuration buffer
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
parse_bss_config(int argc, char *argv[], bss_config_t *bss)
{
	FILE *config_file = NULL;
	char *line = NULL;
	int li = 0;
	char *pos = NULL;
	int arg_num = 0;
	char *args[30];
	int i;
	int is_ap_config = 0;
	int is_ap_mac_filter = 0;
	int keyindex = -1;
	int pwkcipher_wpa = -1;
	int pwkcipher_wpa2 = -1;
	int gwkcipher = -1;
	int protocol = -1;
	int tx_data_rate = -1;
	int tx_beacon_rate = -1;
	int mcbc_data_rate = -1;
	int num_rates = 0;
	int found = 0;
	int filter_mac_count = -1;
	int retval = UAP_SUCCESS;
	int chan_mode = 0;
	int band_flag = 0;
	int chan_number = 0;
	t_u16 max_sta_num_supported = 0;
	fw_info fw;
	HTCap_t htcap;
	int enable_11n = -1;
	t_u32 supported_mcs_set = 0;
	int ac = 0;
	int is_wmm_parameters = 0;
	char oui_type[4] = { 0x00, 0x50, 0xf2, 0x02 };
	struct eth_priv_vhtcfg vhtcfg = { 0 };

	/* Check if file exists */
	config_file = fopen(argv[0], "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return UAP_FAILURE;
	}
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		retval = UAP_FAILURE;
		goto done;
	}
	memset(line, 0, MAX_CONFIG_LINE);

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Received config line (%d) = %s\n",
			   li, line);
#endif
		arg_num = parse_line(line, args);
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Number of arguments = %d\n",
			   arg_num);
		for (i = 0; i < arg_num; i++) {
			uap_printf(MSG_DEBUG, "\tDBG:Argument %d. %s\n", i + 1,
				   args[i]);
		}
#endif
		/* Check for end of AP configurations */
		if (is_ap_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_ap_config = 0;
				if (tx_data_rate != -1) {
					if ((!bss->rates[0]) && (tx_data_rate)
					    &&
					    (is_tx_rate_valid
					     ((t_u8)tx_data_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Data Rate \n");
						retval = UAP_FAILURE;
						goto done;
					}
					if (bss->rates[0] && tx_data_rate) {
						for (i = 0; bss->rates[i] != 0;
						     i++) {
							if ((bss->
							     rates[i] &
							     ~BASIC_RATE_SET_BIT)
							    == tx_data_rate) {
								found = 1;
								break;
							}
						}
						if (!found) {
							printf("ERR: Invalid Tx Data Rate \n");
							retval = UAP_FAILURE;
							goto done;
						}
					}

					/* Set Tx data rate field */
					bss->tx_data_rate = tx_data_rate;
				}
				if (tx_beacon_rate != -1) {
					if ((!bss->rates[0]) && (tx_beacon_rate)
					    &&
					    (is_tx_rate_valid
					     ((t_u8)tx_beacon_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Beacon Rate \n");
						retval = UAP_FAILURE;
						goto done;
					}
					if (bss->rates[0] && tx_beacon_rate) {
						for (i = 0; bss->rates[i] != 0;
						     i++) {
							if ((bss->
							     rates[i] &
							     ~BASIC_RATE_SET_BIT)
							    == tx_beacon_rate) {
								found = 1;
								break;
							}
						}
						if (!found) {
							printf("ERR: Invalid Tx Beacon Rate \n");
							retval = UAP_FAILURE;
							goto done;
						}
					}

					/* Set Tx beacon rate field */
					bss->tx_beacon_rate = tx_beacon_rate;
				}
				if (mcbc_data_rate != -1) {
					if ((!bss->rates[0]) && (mcbc_data_rate)
					    &&
					    (is_mcbc_rate_valid
					     ((t_u8)mcbc_data_rate) !=
					     UAP_SUCCESS)) {
						printf("ERR: Invalid Tx Data Rate \n");
						retval = UAP_FAILURE;
						goto done;
					}
					if (bss->rates[0] && mcbc_data_rate) {
						for (i = 0; bss->rates[i] != 0;
						     i++) {
							if (bss->
							    rates[i] &
							    BASIC_RATE_SET_BIT)
							{
								if ((bss->
								     rates[i] &
								     ~BASIC_RATE_SET_BIT)
								    ==
								    mcbc_data_rate)
								{
									found = 1;
									break;
								}
							}
						}
						if (!found) {
							printf("ERR: Invalid MCBC Data Rate \n");
							retval = UAP_FAILURE;
							goto done;
						}
					}

					/* Set MCBC data rate field */
					bss->mcbc_data_rate = mcbc_data_rate;
				}
				if ((protocol == PROTOCOL_STATIC_WEP) &&
				    (enable_11n == 1)) {
					printf("ERR:WEP cannot be used when AP operates in 802.11n mode.\n");
					goto done;
				}
				if (0 == get_fw_info(&fw)) {
					/*check whether support 802.11AC through BAND_AAC bit */
					if (fw.fw_bands & BAND_AAC) {
						retval = get_802_11ac_cfg
							(&vhtcfg);
						if (retval != UAP_SUCCESS)
							goto done;
						if ((enable_11n == 1) &&
						    (vhtcfg.vht_rx_mcs !=
						     0xffff)) {
							printf("ERR: 11n must be enabled when AP operates in 11ac mode.\n");
							retval = UAP_FAILURE;
							goto done;
						}
					} else
						printf("No support 802 11AC.\n");
				} else {
					printf("ERR: get_fw_info fail\n");
					retval = UAP_FAILURE;
					goto done;
				}
				if ((protocol == PROTOCOL_WPA2_MIXED) &&
				    ((pwkcipher_wpa < 0) ||
				     (pwkcipher_wpa2 < 0))) {
					printf("ERR:Both PwkCipherWPA and PwkCipherWPA2 should be defined for Mixed mode.\n");
					retval = UAP_FAILURE;
					goto done;
				}
				if (protocol != -1) {
					bss->protocol = protocol;
					if (protocol &
					    (PROTOCOL_WPA | PROTOCOL_WPA2)) {
						/* Set key management field */
						bss->key_mgmt = KEY_MGMT_PSK;
						bss->key_mgmt_operation = 0;
					}
				}
				if (((pwkcipher_wpa >= 0) ||
				     (pwkcipher_wpa2 >= 0)) &&
				    (gwkcipher >= 0)) {
					if ((protocol == PROTOCOL_WPA) ||
					    (protocol == PROTOCOL_WPA2_MIXED)) {
						if (enable_11n != -1) {
							if (is_cipher_valid_with_11n(pwkcipher_wpa, gwkcipher, protocol, enable_11n) != UAP_SUCCESS) {
								printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
								retval = UAP_FAILURE;
								goto done;
							}
						} else if
							(is_cipher_valid_with_proto
							 (pwkcipher_wpa,
							  gwkcipher,
							  protocol) !=
							 UAP_SUCCESS) {
							printf("ERR:Wrong group cipher and WPA pairwise cipher combination!\n");
							retval = UAP_FAILURE;
							goto done;
						}
					}
					if ((protocol == PROTOCOL_WPA2) ||
					    (protocol == PROTOCOL_WPA2_MIXED)
						) {
						if (enable_11n != -1) {
							if (is_cipher_valid_with_11n(pwkcipher_wpa2, gwkcipher, protocol, enable_11n) != UAP_SUCCESS) {
								printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
								retval = UAP_FAILURE;
								goto done;
							}
						} else if
							(is_cipher_valid_with_proto
							 (pwkcipher_wpa2,
							  gwkcipher,
							  protocol) !=
							 UAP_SUCCESS) {
							printf("ERR:Wrong group cipher and WPA2 pairwise cipher combination!\n");
							retval = UAP_FAILURE;
							goto done;
						}
					}
					/* Set pairwise and group cipher fields */
					bss->wpa_cfg.pairwise_cipher_wpa =
						pwkcipher_wpa;
					bss->wpa_cfg.pairwise_cipher_wpa2 =
						pwkcipher_wpa2;
					bss->wpa_cfg.group_cipher = gwkcipher;
				}
				continue;
			}
		}

		/* Check for beginning of AP configurations */
		if (strcmp(args[0], "ap_config") == 0) {
			is_ap_config = 1;
			continue;
		}

		/* Check for end of AP MAC address filter configurations */
		if (is_ap_mac_filter == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_ap_mac_filter = 0;
				if (bss->filter.mac_count != filter_mac_count) {
					printf("ERR:Number of MAC address provided does not match 'Count'\n");
					retval = UAP_FAILURE;
					goto done;
				}
				if (bss->filter.filter_mode &&
				    (bss->filter.mac_count == 0)) {
					printf("ERR:Filter list can not be empty for %s Filter mode\n", (bss->filter.filter_mode == 1) ? "'Allow'" : "'Block'");
					retval = UAP_FAILURE;
					goto done;
				}
				continue;
			}
		}

		/* Check for beginning of AP MAC address filter configurations */
		if (strcmp(args[0], "ap_mac_filter") == 0) {
			is_ap_mac_filter = 1;
			bss->filter.mac_count = 0;
			filter_mac_count = 0;
			continue;
		}
		if ((strcmp(args[0], "FilterMode") == 0) && is_ap_mac_filter) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0) ||
			    (atoi(args[1]) > 2)) {
				printf("ERR:Illegal FilterMode paramter %d. Must be either '0', '1', or '2'.\n", atoi(args[1]));
				retval = UAP_FAILURE;
				goto done;
			}
			bss->filter.filter_mode = atoi(args[1]);
			continue;
		}
		if ((strcmp(args[0], "Count") == 0) && is_ap_mac_filter) {
			bss->filter.mac_count = atoi(args[1]);
			if ((ISDIGIT(args[1]) == 0) ||
			    (bss->filter.mac_count > MAX_MAC_ONESHOT_FILTER)) {
				printf("ERR: Illegal Count parameter.\n");
				retval = UAP_FAILURE;
				goto done;
			}
		}
		if ((strncmp(args[0], "mac_", 4) == 0) && is_ap_mac_filter) {
			if (filter_mac_count < MAX_MAC_ONESHOT_FILTER) {
				if (mac2raw
				    (args[1],
				     bss->filter.mac_list[filter_mac_count]) !=
				    UAP_SUCCESS) {
					printf("ERR: Invalid MAC address %s \n",
					       args[1]);
					retval = UAP_FAILURE;
					goto done;
				}
				filter_mac_count++;
			} else {
				printf("ERR: Filter table can not have more than %d MAC addresses\n", MAX_MAC_ONESHOT_FILTER);
				retval = UAP_FAILURE;
				goto done;
			}
		}

		/* Check for end of Wmm Parameters configurations */
		if (is_wmm_parameters == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_wmm_parameters = 0;
				continue;
			}
		}
		/* Check for beginning of Sticky Tim Sta MAC address Configurations */
		if (strcmp(args[0], "Wmm_parameters") == 0) {
			is_wmm_parameters = 1;
			memset(&(bss->wmm_para), 0, sizeof(bss->wmm_para));
			memcpy(bss->wmm_para.ouitype, oui_type,
			       sizeof(oui_type));
			bss->wmm_para.ouisubtype = 1;
			bss->wmm_para.version = 1;
			continue;
		}
		if ((strcmp(args[0], "Qos_info") == 0) && is_wmm_parameters) {
			bss->wmm_para.qos_info = A2HEXDECIMAL(args[1]);
			printf("wmm_para.qos_info = %2x\n",
			       bss->wmm_para.qos_info);
			if ((bss->wmm_para.qos_info != ENABLE_WMM_PS) &&
			    (bss->wmm_para.qos_info != DISABLE_WMM_PS)) {
				printf("ERR:qos_info must be either 0x80 or 0x00.\n");
				retval = UAP_FAILURE;
				goto done;
			}
		}
		if ((strcmp(args[0], "AC_BE") == 0) && is_wmm_parameters) {
			ac = 0;
		}
		if ((strcmp(args[0], "AC_BK") == 0) && is_wmm_parameters) {
			ac = 1;
		}
		if ((strcmp(args[0], "AC_VI") == 0) && is_wmm_parameters) {
			ac = 2;
		}
		if ((strcmp(args[0], "AC_VO") == 0) && is_wmm_parameters) {
			ac = 3;
		}
		if ((strcmp(args[0], "Aifsn") == 0) && is_wmm_parameters) {
			bss->wmm_para.ac_params[ac].aci_aifsn.aifsn =
				(t_u8)A2HEXDECIMAL(args[1]);
			printf("wmm_para.ac_params[%d].aci_aifsn.aifsn = %x\n",
			       ac, bss->wmm_para.ac_params[ac].aci_aifsn.aifsn);
			bss->wmm_para.ac_params[ac].aci_aifsn.aci = (t_u8)ac;
			printf("wmm_para.ac_params[%d].aci_aifsn.aci = %x\n",
			       ac, bss->wmm_para.ac_params[ac].aci_aifsn.aci);
		}
		if ((strcmp(args[0], "Ecw_max") == 0) && is_wmm_parameters) {
			bss->wmm_para.ac_params[ac].ecw.ecw_max =
				(t_u8)A2HEXDECIMAL(args[1]);
			printf("wmm_para.ac_params[%d].ecw.ecw_max = %x\n", ac,
			       bss->wmm_para.ac_params[ac].ecw.ecw_max);
		}
		if ((strcmp(args[0], "Ecw_min") == 0) && is_wmm_parameters) {
			bss->wmm_para.ac_params[ac].ecw.ecw_min =
				(t_u8)A2HEXDECIMAL(args[1]);
			printf("wmm_para.ac_params[%d].ecw.ecw_min = %x\n", ac,
			       bss->wmm_para.ac_params[ac].ecw.ecw_min);
		}
		if ((strcmp(args[0], "Tx_op") == 0) && is_wmm_parameters) {
			bss->wmm_para.ac_params[ac].tx_op_limit =
				(t_u8)A2HEXDECIMAL(args[1]);
			printf("wmm_para.ac_params[%d].tx_op_limit = %x\n", ac,
			       bss->wmm_para.ac_params[ac].tx_op_limit);
		}

		if (strcmp(args[0], "SSID") == 0) {
			if (arg_num == 1) {
				printf("ERR:SSID field is blank!\n");
				retval = UAP_FAILURE;
				goto done;
			} else {
				if (args[1][0] == '"') {
					args[1]++;
				}
				if (args[1][strlen(args[1]) - 1] == '"') {
					args[1][strlen(args[1]) - 1] = '\0';
				}
				if ((strlen(args[1]) > MAX_SSID_LENGTH) ||
				    (strlen(args[1]) == 0)) {
					printf("ERR:SSID length out of range (%d to %d).\n", MIN_SSID_LENGTH, MAX_SSID_LENGTH);
					retval = UAP_FAILURE;
					goto done;
				}
				/* Set SSID field */
				bss->ssid.ssid_len = strlen(args[1]);
				memcpy(bss->ssid.ssid, args[1],
				       bss->ssid.ssid_len);
			}
		}
		if (strcmp(args[0], "BeaconPeriod") == 0) {
			if (is_input_valid(BEACONPERIOD, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set beacon period field */
			bss->beacon_period = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "ChanList") == 0) {
			if (is_input_valid(SCANCHANNELS, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}

			/* Set channel list field */
			if ((arg_num - 1) < MAX_CHANNELS) {
				bss->num_of_chan = arg_num - 1;
			} else {
				bss->num_of_chan = MAX_CHANNELS;
			}
			for (i = 0; (unsigned int)i < bss->num_of_chan; i++) {
				sscanf(args[i + 1], "%d.%d", &chan_number,
				       &band_flag);
				bss->chan_list[i].chan_number = chan_number;
				bss->chan_list[i].bandcfg.chanBand = BAND_2GHZ;
				if (((band_flag != -1) && (band_flag)) ||
				    (chan_number > MAX_CHANNELS_BG)) {
					bss->chan_list[i].bandcfg.chanBand =
						BAND_5GHZ;
				}
			}
		}
		if (strcmp(args[0], "Channel") == 0) {
			if (is_input_valid(CHANNEL, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set channel field */
			bss->channel = (t_u8)atoi(args[1]);
			if ((arg_num - 1) == 2) {
				chan_mode = atoi(args[2]);
				memset(&(bss->bandcfg), 0,
				       sizeof(bss->bandcfg));
				if (chan_mode & BITMAP_ACS_MODE) {
					int mode;

					if (uap_ioctl_dfs_repeater_mode(&mode)
					    == UAP_SUCCESS) {
						if (mode) {
							printf("ERR: ACS in DFS Repeater mode" " is not allowed\n");
							retval = UAP_FAILURE;
							goto done;
						}
					}

					bss->bandcfg.scanMode = SCAN_MODE_ACS;
				}
				if (chan_mode & BITMAP_CHANNEL_ABOVE)
					bss->bandcfg.chan2Offset =
						SEC_CHAN_ABOVE;
				if (chan_mode & BITMAP_CHANNEL_BELOW)
					bss->bandcfg.chan2Offset =
						SEC_CHAN_BELOW;
			} else
				memset(&(bss->bandcfg), 0,
				       sizeof(bss->bandcfg));
			if (bss->channel > MAX_CHANNELS_BG)
				bss->bandcfg.chanBand = BAND_5GHZ;
		}
		if (strcmp(args[0], "Band") == 0) {
			if (is_input_valid(BAND, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Clear previously set band */
			bss->bandcfg.chanBand = BAND_2GHZ;
			if (atoi(args[1]) == 1)
				bss->bandcfg.chanBand = BAND_5GHZ;
		}
		if (strcmp(args[0], "AP_MAC") == 0) {
			int ret;
			if ((ret =
			     mac2raw(args[1], bss->mac_addr)) != UAP_SUCCESS) {
				printf("ERR: %s Address \n",
				       ret ==
				       UAP_FAILURE ? "Invalid MAC" : ret ==
				       UAP_RET_MAC_BROADCAST ? "Broadcast" :
				       "Multicast");
				retval = UAP_FAILURE;
				goto done;
			}
		}

		if (strcmp(args[0], "Rate") == 0) {
			if (is_input_valid(RATE, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				printf("ERR: Invalid Rate input\n");
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set rates field */
			if ((arg_num - 1) < MAX_DATA_RATES) {
				num_rates = arg_num - 1;
			} else {
				num_rates = MAX_DATA_RATES;
			}
			for (i = 0; i < num_rates; i++) {
				bss->rates[i] = (t_u8)A2HEXDECIMAL(args[i + 1]);
			}
		}
		if (strcmp(args[0], "TxPowerLevel") == 0) {
			if (is_input_valid(TXPOWER, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				printf("ERR:Invalid TxPowerLevel \n");
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Tx power level field */
			bss->tx_power_level = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "BroadcastSSID") == 0) {
			if (is_input_valid(BROADCASTSSID, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set broadcast SSID control field */
			bss->bcast_ssid_ctl = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "RTSThreshold") == 0) {
			if (is_input_valid(RTSTHRESH, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set RTS threshold field */
			bss->rts_threshold = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "FragThreshold") == 0) {
			if (is_input_valid(FRAGTHRESH, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Frag threshold field */
			bss->frag_threshold = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "DTIMPeriod") == 0) {
			if (is_input_valid(DTIMPERIOD, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set DTIM period field */
			bss->dtim_period = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "RSNReplayProtection") == 0) {
			if (is_input_valid(RSNREPLAYPROT, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set RSN replay protection field */
			bss->wpa_cfg.rsn_protection = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "PairwiseUpdateTimeout") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Pairwise Update Timeout field */
			bss->pairwise_update_timeout = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "PairwiseHandshakeRetries") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Pairwise Hanshake Retries */
			bss->pwk_retries = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "GroupwiseUpdateTimeout") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Groupwise Update Timeout field */
			bss->groupwise_update_timeout = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "GroupwiseHandshakeRetries") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0)) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set Groupwise Hanshake Retries */
			bss->gwk_retries = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "TxBeaconRate") == 0) {
			if (is_input_valid(TXBEACONRATE, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			tx_beacon_rate = (t_u16)A2HEXDECIMAL(args[1]);
		}
		if (strcmp(args[0], "MCBCdataRate") == 0) {
			if (is_input_valid(MCBCDATARATE, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			mcbc_data_rate = (t_u16)A2HEXDECIMAL(args[1]);
		}
		if (strcmp(args[0], "PktFwdCtl") == 0) {
			if (is_input_valid(PKTFWD, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set packet forward control field */
			bss->pkt_forward_ctl = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "StaAgeoutTimer") == 0) {
			if (is_input_valid
			    (STAAGEOUTTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set STA ageout timer field */
			bss->sta_ageout_timer = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "PSStaAgeoutTimer") == 0) {
			if (is_input_valid
			    (PSSTAAGEOUTTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set PS STA ageout timer field */
			bss->ps_sta_ageout_timer = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "AuthMode") == 0) {
			if (is_input_valid(AUTHMODE, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set auth mode field */
			bss->auth_mode = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "KeyIndex") == 0) {
			if (arg_num == 1) {
				printf("KeyIndex is blank!\n");
				retval = UAP_FAILURE;
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal KeyIndex parameter. Must be either '0', '1', '2', or '3'.\n");
					retval = UAP_FAILURE;
					goto done;
				}
				keyindex = atoi(args[1]);
				if ((keyindex < 0) || (keyindex > 3)) {
					printf("ERR:Illegal KeyIndex parameter. Must be either '0', '1', '2', or '3'.\n");
					retval = UAP_FAILURE;
					goto done;
				}
				switch (keyindex) {
				case 0:
					bss->wep_cfg.key0.is_default = 1;
					break;
				case 1:
					bss->wep_cfg.key1.is_default = 1;
					break;
				case 2:
					bss->wep_cfg.key2.is_default = 1;
					break;
				case 3:
					bss->wep_cfg.key3.is_default = 1;
					break;
				}
			}
		}
		if (strncmp(args[0], "Key_", 4) == 0) {
			if (arg_num == 1) {
				printf("ERR:%s is blank!\n", args[0]);
				retval = UAP_FAILURE;
				goto done;
			} else {
				int key_len = 0;
				if (args[1][0] == '"') {
					if ((strlen(args[1]) != 2) &&
					    (strlen(args[1]) != 7) &&
					    (strlen(args[1]) != 15)) {
						printf("ERR:Wrong key length!\n");
						retval = UAP_FAILURE;
						goto done;
					}
					key_len = strlen(args[1]) - 2;
				} else {
					if ((strlen(args[1]) != 0) &&
					    (strlen(args[1]) != 10) &&
					    (strlen(args[1]) != 26)) {
						printf("ERR:Wrong key length!\n");
						retval = UAP_FAILURE;
						goto done;
					}
					if (UAP_FAILURE == ishexstring(args[1])) {
						printf("ERR:Only hex digits are allowed when key length is 10 or 26\n");
						retval = UAP_FAILURE;
						goto done;
					}
					key_len = strlen(args[1]) / 2;
				}
				/* Set WEP key fields */
				if (strcmp(args[0], "Key_0") == 0) {
					bss->wep_cfg.key0.key_index = 0;
					bss->wep_cfg.key0.length = key_len;
					if (args[1][0] == '"') {
						memcpy(bss->wep_cfg.key0.key,
						       &args[1][1],
						       strlen(args[1]) - 2);
					} else {
						string2raw(args[1],
							   bss->wep_cfg.key0.
							   key);
					}
				} else if (strcmp(args[0], "Key_1") == 0) {
					bss->wep_cfg.key1.key_index = 1;
					bss->wep_cfg.key1.length = key_len;
					if (args[1][0] == '"') {
						memcpy(bss->wep_cfg.key1.key,
						       &args[1][1],
						       strlen(args[1]) - 2);
					} else {
						string2raw(args[1],
							   bss->wep_cfg.key1.
							   key);
					}
				} else if (strcmp(args[0], "Key_2") == 0) {
					bss->wep_cfg.key2.key_index = 2;
					bss->wep_cfg.key2.length = key_len;
					if (args[1][0] == '"') {
						memcpy(bss->wep_cfg.key2.key,
						       &args[1][1],
						       strlen(args[1]) - 2);
					} else {
						string2raw(args[1],
							   bss->wep_cfg.key2.
							   key);
					}
				} else if (strcmp(args[0], "Key_3") == 0) {
					bss->wep_cfg.key3.key_index = 3;
					bss->wep_cfg.key3.length = key_len;
					if (args[1][0] == '"') {
						memcpy(bss->wep_cfg.key3.key,
						       &args[1][1],
						       strlen(args[1]) - 2);
					} else {
						string2raw(args[1],
							   bss->wep_cfg.key3.
							   key);
					}
				}
			}
		}
		if (strcmp(args[0], "PSK") == 0) {
			if (arg_num == 1) {
				printf("ERR:PSK is blank!\n");
				retval = UAP_FAILURE;
				goto done;
			} else {
				if (args[1][0] == '"') {
					args[1]++;
				}
				if (args[1][strlen(args[1]) - 1] == '"') {
					args[1][strlen(args[1]) - 1] = '\0';
				}
				if (strlen(args[1]) > MAX_WPA_PASSPHRASE_LENGTH) {
					printf("ERR:PSK too long.\n");
					retval = UAP_FAILURE;
					goto done;
				}
				if (strlen(args[1]) < MIN_WPA_PASSPHRASE_LENGTH) {
					printf("ERR:PSK too short.\n");
					retval = UAP_FAILURE;
					goto done;
				}
				if (strlen(args[1]) ==
				    MAX_WPA_PASSPHRASE_LENGTH) {
					if (UAP_FAILURE == ishexstring(args[1])) {
						printf("ERR:Only hex digits are allowed when passphrase's length is 64\n");
						retval = UAP_FAILURE;
						goto done;
					}
				}
				/* Set WPA passphrase field */
				bss->wpa_cfg.length = strlen(args[1]);
				memcpy(bss->wpa_cfg.passphrase, args[1],
				       bss->wpa_cfg.length);
			}
		}
		if (strcmp(args[0], "Protocol") == 0) {
			if (is_input_valid(PROTOCOL, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set protocol field */
			protocol = (t_u16)atoi(args[1]);
		}
		if ((strcmp(args[0], "PairwiseCipher") == 0) ||
		    (strcmp(args[0], "GroupCipher") == 0)) {
			printf("ERR:PairwiseCipher and GroupCipher are not supported.\n" "    Please configure pairwise cipher using parameters PwkCipherWPA or PwkCipherWPA2\n" "    and group cipher using GwkCipher in the config file.\n");
			goto done;
		}

		if ((protocol == PROTOCOL_NO_SECURITY) ||
		    (protocol == PROTOCOL_STATIC_WEP)) {
			if ((strcmp(args[0], "PwkCipherWPA") == 0) ||
			    (strcmp(args[0], "PwkCipherWPA2") == 0)
			    || (strcmp(args[0], "GwkCipher") == 0)) {
				printf("ERR:Pairwise cipher and group cipher should not be defined for Open and WEP mode.\n");
				goto done;
			}
		}

		if (strcmp(args[0], "PwkCipherWPA") == 0) {
			if (arg_num == 1) {
				printf("ERR:PwkCipherWPA is blank!\n");
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal PwkCipherWPA parameter. Must be either bit '2' or '3'.\n");
					goto done;
				}
				if (atoi(args[1]) & ~CIPHER_BITMAP) {
					printf("ERR:Illegal PwkCipherWPA parameter. Must be either bit '2' or '3'.\n");
					goto done;
				}
				pwkcipher_wpa = atoi(args[1]);
				if (enable_11n &&
				    protocol != PROTOCOL_WPA2_MIXED) {
					memset(&htcap, 0, sizeof(htcap));
					if (UAP_SUCCESS ==
					    get_sys_cfg_11n(&htcap)) {
						if (htcap.supported_mcs_set[0]
						    && (atoi(args[1]) ==
							CIPHER_TKIP)) {
							printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
							return UAP_FAILURE;
						}
					}
				}
			}
		}
		if (strcmp(args[0], "PwkCipherWPA2") == 0) {
			if (arg_num == 1) {
				printf("ERR:PwkCipherWPA2 is blank!\n");
				goto done;
			} else {
				if (ISDIGIT(args[1]) == 0) {
					printf("ERR:Illegal PwkCipherWPA2 parameter. Must be either bit '2' or '3'.\n");
					goto done;
				}
				if (atoi(args[1]) & ~CIPHER_BITMAP) {
					printf("ERR:Illegal PwkCipherWPA2 parameter. Must be either bit '2' or '3'.\n");
					goto done;
				}
				pwkcipher_wpa2 = atoi(args[1]);
				if (enable_11n &&
				    protocol != PROTOCOL_WPA2_MIXED) {
					memset(&htcap, 0, sizeof(htcap));
					if (UAP_SUCCESS ==
					    get_sys_cfg_11n(&htcap)) {
						if (htcap.supported_mcs_set[0]
						    && (atoi(args[1]) ==
							CIPHER_TKIP)) {
							printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
							return UAP_FAILURE;
						}
					}
				}
			}
		}
		if (strcmp(args[0], "GwkCipher") == 0) {
			if (is_input_valid(GWK_CIPHER, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				goto done;
			}
			gwkcipher = atoi(args[1]);
		}
		if (strcmp(args[0], "GroupRekeyTime") == 0) {
			if (is_input_valid
			    (GROUPREKEYTIMER, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}

			/* Set group rekey time field */
			bss->wpa_cfg.gk_rekey_time = (t_u32)atoi(args[1]);
		}
		if (strcmp(args[0], "MaxStaNum") == 0) {
			if (is_input_valid(MAXSTANUM, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}
			if (get_max_sta_num_supported(&max_sta_num_supported) ==
			    UAP_FAILURE) {
				retval = UAP_FAILURE;
				goto done;
			}
			if (atoi(args[1]) > max_sta_num_supported) {
				printf("ERR: MAX_STA_NUM must be less than %d\n", max_sta_num_supported);
				retval = UAP_FAILURE;
				goto done;
			}
			/* Set max STA number field */
			bss->max_sta_count = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "Retrylimit") == 0) {
			if (is_input_valid(RETRYLIMIT, arg_num - 1, args + 1) !=
			    UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}

			/* Set retry limit field */
			bss->retry_limit = (t_u16)atoi(args[1]);
		}
		if (strcmp(args[0], "PreambleType") == 0) {
			if (is_input_valid(PREAMBLETYPE, arg_num - 1, args + 1)
			    != UAP_SUCCESS) {
				retval = UAP_FAILURE;
				goto done;
			}

			/* Set preamble type field */
			bss->preamble_type = (t_u8)atoi(args[1]);
		}
		if (strcmp(args[0], "Enable11n") == 0) {
			if ((ISDIGIT(args[1]) != UAP_SUCCESS) ||
			    (atoi(args[1]) < 0) || (atoi(args[1]) > 1)) {
				printf("ERR: Invalid Enable11n value\n");
				goto done;
			}
			enable_11n = atoi(args[1]);

			memset(&htcap, 0, sizeof(htcap));
			if (UAP_SUCCESS != get_sys_cfg_11n(&htcap)) {
				printf("ERR: Reading current 11n configuration.\n");
				goto done;
			}
			bss->ht_cap_info = htcap.ht_cap_info;
			bss->ampdu_param = htcap.ampdu_param;
			memcpy(bss->supported_mcs_set, htcap.supported_mcs_set,
			       16);
			bss->ht_ext_cap = htcap.ht_ext_cap;
			bss->tx_bf_cap = htcap.tx_bf_cap;
			bss->asel = htcap.asel;
			if (enable_11n == 1) {
				/* enable mcs rate */
				bss->supported_mcs_set[0] = DEFAULT_MCS_SET_0;
				bss->supported_mcs_set[4] = DEFAULT_MCS_SET_4;
				if (0 == get_fw_info(&fw)) {
					if ((fw.hw_dev_mcs_support & 0x0f) >= 2)
						bss->supported_mcs_set[1] =
							DEFAULT_MCS_SET_1;
				}
			} else {
				/* disable mcs rate */
				bss->supported_mcs_set[0] = 0;
				bss->supported_mcs_set[4] = 0;
				bss->supported_mcs_set[1] = 0;
			}
		}
		if (strcmp(args[0], "HTCapInfo") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before HTCapInfo.\n");
				goto done;
			}
			if ((IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE) ||
			    ((((t_u16)A2HEXDECIMAL(args[1])) &
			      (~HT_CAP_CONFIG_MASK)) != HT_CAP_CHECK_MASK)) {
				printf("ERR: Invalid HTCapInfo value\n");
				goto done;
			}
			bss->ht_cap_info =
				DEFAULT_HT_CAP_VALUE & ~HT_CAP_CONFIG_MASK;
			bss->ht_cap_info |=
				(t_u16)A2HEXDECIMAL(args[1]) &
				HT_CAP_CONFIG_MASK;
			bss->ht_cap_info = uap_cpu_to_le16(bss->ht_cap_info);
		}
		if (strcmp(args[0], "AMPDU") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before AMPDU.\n");
				goto done;
			}
			if ((IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE) ||
			    ((A2HEXDECIMAL(args[1])) > AMPDU_CONFIG_MASK)) {
				printf("ERR: Invalid AMPDU value\n");
				goto done;
			}
			/* Find HT tlv pointer in buffer and set AMPDU */
			bss->ampdu_param =
				(t_u8)A2HEXDECIMAL(args[1]) & AMPDU_CONFIG_MASK;
		}
		if (strcmp(args[0], "HT_MCS_MAP") == 0) {
			if (enable_11n <= 0) {
				printf("ERR: Enable11n parameter should be set before HT_MCS_MAP.\n");
				goto done;
			}
			if ((IS_HEX_OR_DIGIT(args[1]) == UAP_FAILURE)) {
				printf("ERR: Invalid HT_MCS_MAP value\n");
				goto done;
			}
			if (0 == get_fw_info(&fw)) {
				/* Check upper nibble of MCS support value
				 * and block MCS_SET_1 when 2X2 is not supported
				 * by the underlying hardware */
				if (((fw.hw_dev_mcs_support & 0xf0) <
				     STREAM_2X2_MASK) &&
				    (A2HEXDECIMAL(args[1]) & MCS_SET_1_MASK)) {
					printf("ERR: Invalid HT_MCS_MAP\n");
					goto done;
				}
			}
			supported_mcs_set = (t_u32)A2HEXDECIMAL(args[1]);
			supported_mcs_set = uap_cpu_to_le32(supported_mcs_set);
			memcpy(bss->supported_mcs_set, &supported_mcs_set,
			       sizeof(t_u32));
		}
		if (strcmp(args[0], "Enable2040Coex") == 0) {
			if ((ISDIGIT(args[1]) == 0) || (atoi(args[1]) < 0) ||
			    (atoi(args[1]) > 1)) {
				printf("ERR:Invalid Enable2040Coex value.\n");
				goto done;
			}
			bss->enable_2040coex = (t_u8)atoi(args[1]);
		}
	}
done:
	fclose(config_file);
	if (line)
		free(line);
	return retval;
}

/**
 *  @brief Show all the BSS configuration in the buffer
 *
 *  @param buf     Pointer to BSS configuration buffer
 *
 *  @return         N/A
 */
void
print_bss_config(bss_config_t *buf)
{
	int i = 0;
	int flag = 0;

	if (!buf) {
		printf("ERR:Empty BSS config!\n");
		return;
	}

	/* Print AP MAC address */
	printf("AP MAC address = ");
	print_mac(buf->mac_addr);
	printf("\n");

	/* Print SSID */
	if (buf->ssid.ssid_len) {
		printf("SSID = %s\n", buf->ssid.ssid);
	}

	/* Print broadcast SSID control */
	printf("SSID broadcast = %s\n",
	       (buf->bcast_ssid_ctl == 1) ? "enabled" : "disabled");

	/* Print DTIM period */
	printf("DTIM period = %d\n", buf->dtim_period);

	/* Print beacon period */
	printf("Beacon period = %d\n", buf->beacon_period);

	/* Print rates */
	printf("Basic Rates =");
	for (i = 0; i < MAX_DATA_RATES; i++) {
		if (!buf->rates[i])
			break;
		if (buf->rates[i] > (BASIC_RATE_SET_BIT - 1)) {
			flag = flag ? : 1;
			printf(" 0x%x", buf->rates[i]);
		}
	}
	printf("%s\nNon-Basic Rates =", flag ? "" : " ( none ) ");
	for (flag = 0, i = 0; i < MAX_DATA_RATES; i++) {
		if (!buf->rates[i])
			break;
		if (buf->rates[i] < BASIC_RATE_SET_BIT) {
			flag = flag ? : 1;
			printf(" 0x%x", buf->rates[i]);
		}
	}
	printf("%s\n", flag ? "" : " ( none ) ");

	/* Print Tx data rate */
	printf("Tx data rate = ");
	if (buf->tx_data_rate == 0)
		printf("auto\n");
	else
		printf("0x%x\n", buf->tx_data_rate);

	/* Print MCBC data rate */
	printf("MCBC data rate = ");
	if (buf->mcbc_data_rate == 0)
		printf("auto\n");
	else
		printf("0x%x\n", buf->mcbc_data_rate);

	/* Print Tx power level */
	printf("Tx power = %d dBm\n", buf->tx_power_level);

	/* Print Tx antenna */
	printf("Tx antenna = %s\n", (buf->tx_antenna) ? "B" : "A");

	/* Print Rx antenna */
	printf("Rx antenna = %s\n", (buf->rx_antenna) ? "B" : "A");

	/* Print packet forward control */
	printf("%s handles packet forwarding -\n",
	       ((buf->pkt_forward_ctl & PKT_FWD_FW_BIT) ==
		0) ? "Host" : "Firmware");
	printf("\tIntra-BSS broadcast packets are %s\n",
	       ((buf->pkt_forward_ctl & PKT_FWD_INTRA_BCAST) ==
		0) ? "allowed" : "denied");
	printf("\tIntra-BSS unicast packets are %s\n",
	       ((buf->pkt_forward_ctl & PKT_FWD_INTRA_UCAST) ==
		0) ? "allowed" : "denied");
	printf("\tInter-BSS unicast packets are %s\n",
	       ((buf->pkt_forward_ctl & PKT_FWD_INTER_UCAST) ==
		0) ? "allowed" : "denied");

	/* Print maximum STA count */
	printf("Max Station Number configured = %d\n", buf->max_sta_count);

	/* Print mgmt frame FWD control */
	printf("MGMT frame Fwd Control = 0x%x\n", buf->mgmt_ie_passthru_mask);

	/* Print MAC filter */
	if (buf->filter.filter_mode == 0) {
		printf("Filter Mode = Filter table is disabled\n");
	} else {
		if (buf->filter.filter_mode == 1) {
			printf("Filter Mode = Allow MAC addresses specified in the allowed list\n");
		} else if (buf->filter.filter_mode == 2) {
			printf("Filter Mode = Block MAC addresses specified in the banned list\n");
		} else {
			printf("Filter Mode = Unknown\n");
		}
		for (i = 0; i < buf->filter.mac_count; i++) {
			printf("MAC_%d = ", i);
			print_mac(buf->filter.mac_list[i]);
			printf("\n");
		}
	}

	/* Print STA ageout timer */
	printf("STA ageout timer = %d\n", buf->sta_ageout_timer);

	/* Print PS STA ageout timer */
	printf("PS STA ageout timer = %d\n", buf->ps_sta_ageout_timer);

	/* Print RTS threshold */
	printf("RTS threshold = %d\n", buf->rts_threshold);

	/* Print Fragmentation threshold */
	printf("Fragmentation threshold = %d\n", buf->frag_threshold);

	/* Print retry limit */
	printf("Retry Limit = %d\n", buf->retry_limit);

	/* Print preamble type */
	printf("Preamble type = %s\n", (buf->preamble_type == 0) ?
	       "auto" : ((buf->preamble_type == 1) ? "short" : "long"));

	/* Print channel */
	printf("Channel = %d\n", buf->channel);
	printf("Band = %s\n",
	       (buf->bandcfg.chanBand == BAND_5GHZ) ? "5GHz" : "2.4GHz");
	printf("Channel Select Mode = %s\n",
	       (buf->bandcfg.scanMode == SCAN_MODE_ACS) ? "ACS" : "Manual");
	if (buf->bandcfg.chan2Offset == SEC_CHAN_NONE)
		printf("no secondary channel\n");
	else if (buf->bandcfg.chan2Offset == SEC_CHAN_ABOVE)
		printf("secondary channel is above primary channel\n");
	else if (buf->bandcfg.chan2Offset == SEC_CHAN_BELOW)
		printf("secondary channel is below primary channel\n");

	/* Print channel list */
	printf("Channels List = ");
	for (i = 0; (unsigned int)i < buf->num_of_chan; i++) {
		printf("\n%d\t%sGHz", buf->chan_list[i].chan_number,
		       (buf->chan_list[i].bandcfg.chanBand ==
			BAND_5GHZ) ? "5" : "2.4");
	}
	printf("\n");

	/* Print auth mode */
	switch (buf->auth_mode) {
	case 0:
		printf("AUTHMODE = Open authentication\n");
		break;
	case 1:
		printf("AUTHMODE = Shared key authentication\n");
		break;
	case 255:
		printf("AUTHMODE = Auto (open and shared key)\n");
		break;
	default:
		printf("ERR: Invalid authmode=%d\n", buf->auth_mode);
		break;
	}

	/* Print protocol */
	switch (buf->protocol) {
	case 0:
	case PROTOCOL_NO_SECURITY:
		printf("PROTOCOL = No security\n");
		break;
	case PROTOCOL_STATIC_WEP:
		printf("PROTOCOL = Static WEP\n");
		break;
	case PROTOCOL_WPA:
		printf("PROTOCOL = WPA \n");
		break;
	case PROTOCOL_WPA2:
		printf("PROTOCOL = WPA2 \n");
		break;
	case PROTOCOL_WPA | PROTOCOL_WPA2:
		printf("PROTOCOL = WPA/WPA2 \n");
		break;
	default:
		printf("Unknown PROTOCOL: 0x%x \n", buf->protocol);
		break;
	}

	/* Print key management */
	if (buf->key_mgmt == KEY_MGMT_PSK)
		printf("KeyMgmt = PSK\n");
	else
		printf("KeyMgmt = NONE\n");

	/* Print WEP configurations */
	if (buf->wep_cfg.key0.length) {
		printf("WEP KEY_0 = ");
		for (i = 0; i < buf->wep_cfg.key0.length; i++) {
			printf("%02x ", buf->wep_cfg.key0.key[i]);
		}
		(buf->wep_cfg.key0.
		 is_default) ? (printf("<Default>\n")) : (printf("\n"));
	} else {
		printf("WEP KEY_0 = NONE\n");
	}
	if (buf->wep_cfg.key1.length) {
		printf("WEP KEY_1 = ");
		for (i = 0; i < buf->wep_cfg.key1.length; i++) {
			printf("%02x ", buf->wep_cfg.key1.key[i]);
		}
		(buf->wep_cfg.key1.
		 is_default) ? (printf("<Default>\n")) : (printf("\n"));
	} else {
		printf("WEP KEY_1 = NONE\n");
	}
	if (buf->wep_cfg.key2.length) {
		printf("WEP KEY_2 = ");
		for (i = 0; i < buf->wep_cfg.key2.length; i++) {
			printf("%02x ", buf->wep_cfg.key2.key[i]);
		}
		(buf->wep_cfg.key2.
		 is_default) ? (printf("<Default>\n")) : (printf("\n"));
	} else {
		printf("WEP KEY_2 = NONE\n");
	}
	if (buf->wep_cfg.key3.length) {
		printf("WEP KEY_3 = ");
		for (i = 0; i < buf->wep_cfg.key3.length; i++) {
			printf("%02x ", buf->wep_cfg.key3.key[i]);
		}
		(buf->wep_cfg.key3.
		 is_default) ? (printf("<Default>\n")) : (printf("\n"));
	} else {
		printf("WEP KEY_3 = NONE\n");
	}

	/* Print WPA configurations */
	if (buf->protocol & PROTOCOL_WPA) {
		switch (buf->wpa_cfg.pairwise_cipher_wpa) {
		case CIPHER_TKIP:
			printf("PwkCipherWPA = TKIP\n");
			break;
		case CIPHER_AES_CCMP:
			printf("PwkCipherWPA = AES CCMP\n");
			break;
		case CIPHER_TKIP | CIPHER_AES_CCMP:
			printf("PwkCipherWPA = TKIP + AES CCMP\n");
			break;
		case CIPHER_NONE:
			printf("PwkCipherWPA =  None\n");
			break;
		default:
			printf("Unknown PwkCipherWPA 0x%x\n",
			       buf->wpa_cfg.pairwise_cipher_wpa);
			break;
		}
	}
	if (buf->protocol & (PROTOCOL_WPA2)) {
		switch (buf->wpa_cfg.pairwise_cipher_wpa2) {
		case CIPHER_TKIP:
			printf("PwkCipherWPA2 = TKIP\n");
			break;
		case CIPHER_AES_CCMP:
			printf("PwkCipherWPA2 = AES CCMP\n");
			break;
		case CIPHER_TKIP | CIPHER_AES_CCMP:
			printf("PwkCipherWPA2 = TKIP + AES CCMP\n");
			break;
		case CIPHER_NONE:
			printf("PwkCipherWPA2 =  None\n");
			break;
		default:
			printf("Unknown PwkCipherWPA2 0x%x\n",
			       buf->wpa_cfg.pairwise_cipher_wpa2);
			break;
		}
	}
	switch (buf->wpa_cfg.group_cipher) {
	case CIPHER_TKIP:
		printf("GroupCipher = TKIP\n");
		break;
	case CIPHER_AES_CCMP:
		printf("GroupCipher = AES CCMP\n");
		break;
	case CIPHER_NONE:
		printf("GroupCipher = None\n");
		break;
	default:
		printf("Unknown Group cipher 0x%x\n",
		       buf->wpa_cfg.group_cipher);
		break;
	}
	printf("RSN replay protection = %s\n",
	       (buf->wpa_cfg.rsn_protection) ? "enabled" : "disabled");
	printf("Pairwise Handshake timeout = %d\n",
	       buf->pairwise_update_timeout);
	printf("Pairwise Handshake Retries = %d\n", buf->pwk_retries);
	printf("Groupwise Handshake timeout = %d\n",
	       buf->groupwise_update_timeout);
	printf("Groupwise Handshake Retries = %d\n", buf->gwk_retries);
	if (buf->wpa_cfg.length > 0) {
		printf("WPA passphrase = ");
		for (i = 0; (unsigned int)i < buf->wpa_cfg.length; i++)
			printf("%c", buf->wpa_cfg.passphrase[i]);
		printf("\n");
	} else {
		printf("WPA passphrase = None\n");
	}
	if (buf->wpa_cfg.gk_rekey_time == 0)
		printf("Group re-key time = disabled\n");
	else
		printf("Group re-key time = %d second\n",
		       buf->wpa_cfg.gk_rekey_time);
	printf("20/40 coex = %s\n",
	       (buf->enable_2040coex) ? "enabled" : "disabled");
	printf("wmm parameters:\n");
	printf("\tqos_info = 0x%x\n", buf->wmm_para.qos_info);
	printf("\tBE: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
	       buf->wmm_para.ac_params[AC_BE].aci_aifsn.aifsn,
	       buf->wmm_para.ac_params[AC_BE].ecw.ecw_max,
	       buf->wmm_para.ac_params[AC_BE].ecw.ecw_min,
	       buf->wmm_para.ac_params[AC_BE].tx_op_limit);
	printf("\tBK: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
	       buf->wmm_para.ac_params[AC_BK].aci_aifsn.aifsn,
	       buf->wmm_para.ac_params[AC_BK].ecw.ecw_max,
	       buf->wmm_para.ac_params[AC_BK].ecw.ecw_min,
	       buf->wmm_para.ac_params[AC_BK].tx_op_limit);
	printf("\tVI: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
	       buf->wmm_para.ac_params[AC_VI].aci_aifsn.aifsn,
	       buf->wmm_para.ac_params[AC_VI].ecw.ecw_max,
	       buf->wmm_para.ac_params[AC_VI].ecw.ecw_min,
	       buf->wmm_para.ac_params[AC_VI].tx_op_limit);
	printf("\tVO: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
	       buf->wmm_para.ac_params[AC_VO].aci_aifsn.aifsn,
	       buf->wmm_para.ac_params[AC_VO].ecw.ecw_max,
	       buf->wmm_para.ac_params[AC_VO].ecw.ecw_min,
	       buf->wmm_para.ac_params[AC_VO].tx_op_limit);

	return;
}

/**
 *  @brief Send command to Read the BSS profile
 *
 *  @param buf      Pointer to bss command buffer for get
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
static int
get_bss_config(t_u8 *buf)
{
	apcmdbuf_bss_configure *cmd_buf = NULL;
	t_s32 sockfd;
	struct ifreq ifr;

	cmd_buf = (apcmdbuf_bss_configure *)buf;
	cmd_buf->action = ACTION_GET;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
#if DEBUG
	/* Dump request buffer */
	hexdump("Get Request buffer", (void *)buf,
		sizeof(apcmdbuf_bss_configure)
		+ sizeof(bss_config_t), ' ');
#endif
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)cmd_buf;
	if (ioctl(sockfd, UAP_BSS_CONFIG, &ifr)) {
		printf("ERR:UAP_BSS_CONFIG is not supported by %s\n", dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}
#if DEBUG
	/* Dump request buffer */
	hexdump("Get Response buffer", (void *)buf,
		sizeof(apcmdbuf_bss_configure)
		+ sizeof(bss_config_t), ' ');
#endif
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Creates a bss_config request and sends to the driver
 *
 *  Usage: "Usage : bss_config [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_bss_config(int argc, char *argv[])
{
	apcmdbuf_bss_configure *cmd_buf = NULL;
	bss_config_t *bss = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len;
	t_u16 buf_len;
	int ret = UAP_SUCCESS;
	int opt;
	t_s32 sockfd;
	struct ifreq ifr;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_bss_config_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_bss_config_usage();
		return UAP_FAILURE;
	}

	/* Query BSS settings */

	/* Alloc buf for command */
	buf_len = sizeof(apcmdbuf_bss_configure) + sizeof(bss_config_t);
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset((char *)buf, 0, buf_len);

	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_bss_configure);
	cmd_buf = (apcmdbuf_bss_configure *)buf;
	bss = (bss_config_t *)(buf + cmd_len);

	/* Get all parametes first */
	if (get_bss_config(buf) == UAP_FAILURE) {
		printf("ERR:Reading current parameters\n");
		free(buf);
		return UAP_FAILURE;
	}

	if (argc == 1) {
		/* Parse config file and populate structure */
		ret = parse_bss_config(argc, argv, bss);
		if (ret == UAP_FAILURE) {
			free(buf);
			return ret;
		}
		cmd_len += sizeof(bss_config_t);
		cmd_buf->action = ACTION_SET;

		/* Send the command */
		/* Open socket */
		if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			printf("ERR:Cannot open socket\n");
			free(buf);
			return UAP_FAILURE;
		}

		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
		ifr.ifr_ifru.ifru_data = (void *)cmd_buf;
#if DEBUG
		/* Dump request buffer */
		hexdump("Request buffer", (void *)buf, buf_len, ' ');
#endif
		if (ioctl(sockfd, UAP_BSS_CONFIG, &ifr)) {
			perror("");
			printf("ERR:UAP_BSS_CONFIG is not supported by %s\n",
			       dev_name);
			close(sockfd);
			free(buf);
			return UAP_FAILURE;
		}
#if DEBUG
		/* Dump respond buffer */
		hexdump("Respond buffer", (void *)buf, buf_len, ' ');
#endif
		close(sockfd);
	} else {
		/* Print response */
		printf("BSS settings:\n");
		print_bss_config(bss);
	}

	free(buf);
	return ret;
}

/**
 *  @brief Read the profile and sends to the driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_coex_config_profile(int argc, char *argv[])
{
	FILE *config_file = NULL;
	char *line = NULL;
	int i, index, li = 0;
	int ret = UAP_SUCCESS;
	char *pos = NULL;
	int arg_num = 0;
	char *args[30];
	int is_coex_config = 0;
	int is_coex_common_config = 0;
	int is_coex_sco_config = 0;
	int is_coex_acl_config = 0;
	t_u8 *buf = NULL;
	apcmdbuf_coex_config *cmd_buf = NULL;
	tlvbuf_coex_common_cfg *coex_common_tlv;
	tlvbuf_coex_sco_cfg *coex_sco_tlv;
	tlvbuf_coex_acl_cfg *coex_acl_tlv;
	t_u16 acl_enabled = 0;
	t_u32 conf_bitmap = 0;
	t_u32 ap_coex_enable = 0;
	t_u16 cmd_len = 0, tlv_len = 0;
	t_u16 sco_prot_qtime[4] = { 0, 0, 0, 0 }, sco_prot_rate =
		0, sco_acl_freq = 0;
	t_u16 acl_bt_time = 0, acl_wlan_time = 0, acl_prot_rate = 0;

	/* Check if file exists */
	config_file = fopen(argv[0], "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return UAP_FAILURE;
	}
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		ret = UAP_FAILURE;
		goto done;
	}
	bzero(line, MAX_CONFIG_LINE);

	/* fixed command length */
	cmd_len = sizeof(apcmdbuf_coex_config) + sizeof(tlvbuf_coex_common_cfg)
		+ sizeof(tlvbuf_coex_sco_cfg) + sizeof(tlvbuf_coex_acl_cfg);
	/* alloc buf for command */
	buf = (t_u8 *)malloc(cmd_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		ret = UAP_FAILURE;
		goto done;
	}
	bzero((char *)buf, cmd_len);

	cmd_buf = (apcmdbuf_coex_config *)buf;

	/* Fill the command buffer */
	cmd_buf->cmd_code = HostCmd_ROBUST_COEX;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = uap_cpu_to_le16(ACTION_SET);

	/* Parse file and process */
	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Received config line (%d) = %s\n",
			   li, line);
#endif
		arg_num = parse_line(line, args);
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Number of arguments = %d\n",
			   arg_num);
		for (i = 0; i < arg_num; i++) {
			uap_printf(MSG_DEBUG, "\tDBG:Argument %d. %s\n", i + 1,
				   args[i]);
		}
#endif
		/* Check for end of Coex configurations */
		if (is_coex_acl_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				coex_acl_tlv =
					(tlvbuf_coex_acl_cfg *)(cmd_buf->
								tlv_buffer +
								tlv_len);
				coex_acl_tlv->tag = MRVL_BT_COEX_ACL_CFG_TLV_ID;
				coex_acl_tlv->length =
					sizeof(tlvbuf_coex_acl_cfg) -
					sizeof(tlvbuf_header);
				endian_convert_tlv_header_out(coex_acl_tlv);
				coex_acl_tlv->enabled =
					uap_cpu_to_le16(acl_enabled);
				coex_acl_tlv->bt_time =
					uap_cpu_to_le16(acl_bt_time);
				coex_acl_tlv->wlan_time =
					uap_cpu_to_le16(acl_wlan_time);
				coex_acl_tlv->protection_rate =
					uap_cpu_to_le16(acl_prot_rate);
				tlv_len += sizeof(tlvbuf_coex_acl_cfg);
				is_coex_acl_config = 0;
			}
		} else if (is_coex_sco_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				coex_sco_tlv =
					(tlvbuf_coex_sco_cfg *)(cmd_buf->
								tlv_buffer +
								tlv_len);
				coex_sco_tlv->tag = MRVL_BT_COEX_SCO_CFG_TLV_ID;
				coex_sco_tlv->length =
					sizeof(tlvbuf_coex_sco_cfg) -
					sizeof(tlvbuf_header);
				endian_convert_tlv_header_out(coex_sco_tlv);
				for (i = 0; i < 4; i++)
					coex_sco_tlv->protection_qtime[i] =
						uap_cpu_to_le16(sco_prot_qtime
								[i]);
				coex_sco_tlv->protection_rate =
					uap_cpu_to_le16(sco_prot_rate);
				coex_sco_tlv->acl_frequency =
					uap_cpu_to_le16(sco_acl_freq);
				tlv_len += sizeof(tlvbuf_coex_sco_cfg);
				is_coex_sco_config = 0;
			}
		} else if (is_coex_common_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				coex_common_tlv =
					(tlvbuf_coex_common_cfg *)(cmd_buf->
								   tlv_buffer +
								   tlv_len);
				coex_common_tlv->tag =
					MRVL_BT_COEX_COMMON_CFG_TLV_ID;
				coex_common_tlv->length =
					sizeof(tlvbuf_coex_common_cfg) -
					sizeof(tlvbuf_header);
				endian_convert_tlv_header_out(coex_common_tlv);
				coex_common_tlv->config_bitmap =
					uap_cpu_to_le32(conf_bitmap);
				coex_common_tlv->ap_bt_coex =
					uap_cpu_to_le32(ap_coex_enable);
				tlv_len += sizeof(tlvbuf_coex_common_cfg);
				is_coex_common_config = 0;
			}
		} else if (is_coex_config == 1) {
			if (strcmp(args[0], "}") == 0)
				is_coex_config = 0;
		}
		if (strcmp(args[0], "coex_config") == 0) {
			is_coex_config = 1;
		} else if (strcmp(args[0], "common_config") == 0) {
			is_coex_common_config = 1;
		} else if (strcmp(args[0], "sco_config") == 0) {
			is_coex_sco_config = 1;
		} else if (strcmp(args[0], "acl_config") == 0) {
			is_coex_acl_config = 1;
		}
		if ((strcmp(args[0], "bitmap") == 0) && is_coex_common_config) {
			if (is_input_valid
			    (COEX_COMM_BITMAP, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			conf_bitmap = (t_u32)A2HEXDECIMAL(args[1]);
		} else if ((strcmp(args[0], "APBTCoex") == 0) &&
			   is_coex_common_config) {
			if (is_input_valid
			    (COEX_COMM_AP_COEX, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			ap_coex_enable = (t_u32)A2HEXDECIMAL(args[1]);
		} else if ((strncmp(args[0], "protectionFromQTime", 19) == 0) &&
			   is_coex_sco_config) {
			index = atoi(args[0] + strlen("protectionFromQTime"));
			if (index < 0 || index > 3) {
				printf("ERR:Incorrect index %d.\n", index);
				ret = UAP_FAILURE;
				goto done;
			}
			if (is_input_valid(COEX_PROTECTION, arg_num, args) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			sco_prot_qtime[index] = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "scoProtectionFromRate") == 0) &&
			   is_coex_sco_config) {
			if (is_input_valid(COEX_PROTECTION, arg_num, args) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			sco_prot_rate = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "aclFrequency") == 0) &&
			   is_coex_sco_config) {
			if (is_input_valid
			    (COEX_SCO_ACL_FREQ, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			sco_acl_freq = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "enabled") == 0) &&
			   is_coex_acl_config) {
			if (is_input_valid
			    (COEX_ACL_ENABLED, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			acl_enabled = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "btTime") == 0) &&
			   is_coex_acl_config) {
			if (is_input_valid
			    (COEX_ACL_BT_TIME, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			acl_bt_time = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "wlanTime") == 0) &&
			   is_coex_acl_config) {
			if (is_input_valid
			    (COEX_ACL_WLAN_TIME, arg_num - 1,
			     args + 1) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			acl_wlan_time = (t_u16)atoi(args[1]);
		} else if ((strcmp(args[0], "aclProtectionFromRate") == 0) &&
			   is_coex_acl_config) {
			if (is_input_valid(COEX_PROTECTION, arg_num, args) !=
			    UAP_SUCCESS) {
				ret = UAP_FAILURE;
				goto done;
			}
			acl_prot_rate = (t_u16)atoi(args[1]);
		}
	}
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_ROBUST_COEX | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			ret = UAP_FAILURE;
			goto done;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("BT Coex settings sucessfully set.\n");
		} else {
			printf("ERR:Could not set coex configuration.\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
done:
	fclose(config_file);
	if (buf)
		free(buf);
	if (line)
		free(line);
	return ret;
}

/**
 *  @brief Show usage information for the coex_config command
 *
 *  $return         N/A
 */
void
print_coex_config_usage(void)
{
	printf("\nUsage : coex_config [CONFIG_FILE]\n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	return;
}

/**
 *  @brief Creates a coex_config request and sends to the driver
 *
 *  Usage: "Usage : coex_config [CONFIG_FILE]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_coex_config(int argc, char *argv[])
{
	apcmdbuf_coex_config *cmd_buf = NULL;
	tlvbuf_coex_common_cfg *coex_common_tlv;
	tlvbuf_coex_sco_cfg *coex_sco_tlv;
	tlvbuf_coex_acl_cfg *coex_acl_tlv;
	tlvbuf_coex_stats *coex_stats_tlv;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_coex_config_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:Too many arguments.\n");
		print_coex_config_usage();
		return UAP_FAILURE;
	}
	if (argc == 1) {
		/* Read profile and send command to firmware */
		ret = apcmd_coex_config_profile(argc, argv);
		return ret;
	}

	/* fixed command length */
	cmd_len = sizeof(apcmdbuf_coex_config) + sizeof(tlvbuf_coex_common_cfg)
		+ sizeof(tlvbuf_coex_sco_cfg) + sizeof(tlvbuf_coex_acl_cfg)
		+ sizeof(tlvbuf_coex_stats);
	/* alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	bzero((char *)buf, buf_len);

	cmd_buf = (apcmdbuf_coex_config *)buf;

	coex_common_tlv = (tlvbuf_coex_common_cfg *)cmd_buf->tlv_buffer;
	coex_common_tlv->tag = MRVL_BT_COEX_COMMON_CFG_TLV_ID;
	coex_common_tlv->length =
		sizeof(tlvbuf_coex_common_cfg) - sizeof(tlvbuf_header);
	endian_convert_tlv_header_out(coex_common_tlv);

	coex_sco_tlv = (tlvbuf_coex_sco_cfg *)(cmd_buf->tlv_buffer +
					       sizeof(tlvbuf_coex_common_cfg));
	coex_sco_tlv->tag = MRVL_BT_COEX_SCO_CFG_TLV_ID;
	coex_sco_tlv->length =
		sizeof(tlvbuf_coex_sco_cfg) - sizeof(tlvbuf_header);
	endian_convert_tlv_header_out(coex_sco_tlv);

	coex_acl_tlv = (tlvbuf_coex_acl_cfg *)(cmd_buf->tlv_buffer +
					       sizeof(tlvbuf_coex_common_cfg) +
					       sizeof(tlvbuf_coex_sco_cfg));
	coex_acl_tlv->tag = MRVL_BT_COEX_ACL_CFG_TLV_ID;
	coex_acl_tlv->length =
		sizeof(tlvbuf_coex_acl_cfg) - sizeof(tlvbuf_header);
	endian_convert_tlv_header_out(coex_acl_tlv);

	coex_stats_tlv = (tlvbuf_coex_stats *)(cmd_buf->tlv_buffer +
					       sizeof(tlvbuf_coex_common_cfg) +
					       sizeof(tlvbuf_coex_sco_cfg)
					       + sizeof(tlvbuf_coex_acl_cfg));
	coex_stats_tlv->tag = MRVL_BT_COEX_STATS_TLV_ID;
	coex_stats_tlv->length =
		sizeof(tlvbuf_coex_stats) - sizeof(tlvbuf_header);
	endian_convert_tlv_header_out(coex_stats_tlv);

	/* Fill the command buffer */
	cmd_buf->cmd_code = HostCmd_ROBUST_COEX;
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = uap_cpu_to_le16(ACTION_GET);

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (HostCmd_ROBUST_COEX | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response!\n");
			free(buf);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("BT Coex settings:\n");
			print_tlv(buf + sizeof(apcmdbuf_coex_config),
				  cmd_buf->size - sizeof(apcmdbuf_coex_config) +
				  BUF_HEADER_SIZE);
		} else {
			printf("ERR:Could not retrieve coex configuration.\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	free(buf);
	return ret;
}

/**
 *  @brief Show usage information for the sys_cfg_custom_ie
 *   command
 *
 *  $return         N/A
 */
void
print_sys_cfg_custom_ie_usage(void)
{
	printf("\nUsage : sys_cfg_custom_ie [INDEX] [MASK] [IEBuffer]");
	printf("\n         empty - Get all IE settings\n");
	printf("\n         INDEX:  0 - Get/Set IE index 0 setting");
	printf("\n                 1 - Get/Set IE index 1 setting");
	printf("\n                 2 - Get/Set IE index 2 setting");
	printf("\n                 3 - Get/Set IE index 3 setting");
	printf("\n                 .                             ");
	printf("\n                 .                             ");
	printf("\n                 .                             ");
	printf("\n                -1 - Append/Delete IE automatically");
	printf("\n                     Delete will delete the IE from the matching IE buffer");
	printf("\n                     Append will append the IE to the buffer with the same mask");
	printf("\n         MASK :  Management subtype mask value as per bit defintions");
	printf("\n              :  Bit 0 - Association request.");
	printf("\n              :  Bit 1 - Association response.");
	printf("\n              :  Bit 2 - Reassociation request.");
	printf("\n              :  Bit 3 - Reassociation response.");
	printf("\n              :  Bit 4 - Probe request.");
	printf("\n              :  Bit 5 - Probe response.");
	printf("\n              :  Bit 8 - Beacon.");
	printf("\n         MASK :  MASK = 0 to clear the mask and the IE buffer");
	printf("\n         IEBuffer :  IE Buffer in hex (max 256 bytes)\n\n");
	return;
}

/** custom IE, auto mask value */
#define	UAP_CUSTOM_IE_AUTO_MASK	0xffff

/**
 *  @brief Get max management IE index
 *  @param         max_mgmt_ie
 *  @param         print flag
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
static int
get_max_mgmt_ie(int *max_mgmt_ie, int flag)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_max_mgmt_ie *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0, i = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;

	*max_mgmt_ie = 0;
	/* Initialize the command length */
	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_max_mgmt_ie);

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_max_mgmt_ie *)(buffer + sizeof(apcmdbuf_sys_configure));

	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_MAX_MGMT_IE_TLV_ID;
	tlv->length = 0;
	cmd_buf->action = ACTION_GET;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_MAX_MGMT_IE_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			tlv->count = uap_le16_to_cpu(tlv->count);
			for (i = 0; i < tlv->count; i++) {
				tlv->info[i].buf_size =
					uap_le16_to_cpu(tlv->info[i].buf_size);
				tlv->info[i].buf_count =
					uap_le16_to_cpu(tlv->info[i].buf_count);
				*max_mgmt_ie += tlv->info[i].buf_count;
				if (flag) {
					printf("buf%d_size = %d\n", i,
					       tlv->info[i].buf_size);
					printf("number of buffers = %d\n",
					       tlv->info[i].buf_count);
					printf("\n");
				}
			}
		} else {
			printf("ERR:Could not get max_mgmt_ie_index!\n");
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return UAP_SUCCESS;
}

/**
 *  @brief Creates a sys_cfg request for custom IE settings
 *   and sends to the driver
 *
 *   Usage: "sys_cfg_custom_ie [INDEX] [MASK] [IEBuffer]"
 *
 *   Options: INDEX :      0 - Get/Set IE index 0 setting
 *                         1 - Get/Set IE index 1 setting
 *                         2 - Get/Set IE index 2 setting
 *                         3 - Get/Set IE index 3 setting
 *                         .
 *                         .
 *                         .
 *            MASK  :      Management subtype mask value
 *            IEBuffer:      IE Buffer in hex
 *            empty - Get all IE settings
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_custom_ie(int argc, char *argv[])
{
	tlvbuf_custom_ie *tlv = NULL;
	tlvbuf_max_mgmt_ie *max_mgmt_ie_tlv = NULL;
	custom_ie *ie_ptr = NULL;
	t_u8 *buffer = NULL;
	t_u16 buf_len = 0;
	t_u16 mgmt_subtype_mask = 0;
	int ie_buf_len = 0, ie_len = 0, i = 0, max_mgmt_ie = 0, print_flag = 0;
	struct ifreq ifr;
	t_s32 sockfd;

	if (argc == 1 && max_mgmt_ie_print == 0) {
		/* Print buffer sizes only if (argc == 1) i.e. get all indices one by one
		 * && (max_mgmt_ie_print == 0) i.e. max mgmt IE is not printed via sys_config cmd */
		print_flag = 1;
	}

	/* Reset the max_mgmt_ie_print for successive cmds */
	max_mgmt_ie_print = 0;

	if (!get_max_mgmt_ie(&max_mgmt_ie, print_flag)) {
		printf("ERR:couldn't get max_mgmt_ie!\n");
		return UAP_FAILURE;
	}
	if (max_mgmt_ie == 0) {
		max_mgmt_ie = MAX_MGMT_IE_INDEX;
#if DEBUG
		uap_printf(MSG_DEBUG,
			   "WARN: max_mgmt_ie=0, defaulting to MAX_MGMT_IE_INDEX\n");
#endif
	}

	/* Check arguments */
	if (argc > 4) {
		printf("ERR:Too many arguments.\n");
		print_sys_cfg_custom_ie_usage();
		return UAP_FAILURE;
	}

	/* Error checks and initialize the command length */
	if (argc >= 2) {
		if (((IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE) &&
		     (atoi(argv[1]) != -1)) || (atoi(argv[1]) < -1)) {
			printf("ERR:Illegal index %s\n", argv[1]);
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
	}
	switch (argc) {
	case 1:
		buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		break;
	case 2:
		if ((atoi(argv[1]) < 0) || (atoi(argv[1]) >= max_mgmt_ie)) {
			printf("ERR:Illegal index %s. Must be either greater than or equal to 0 and less than %d for Get Operation \n", argv[1], max_mgmt_ie);
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		break;
	case 3:
		if (UAP_FAILURE == ishexstring(argv[2]) ||
		    A2HEXDECIMAL(argv[2]) != 0) {
			printf("ERR: Mask value should be 0 to clear IEBuffers.\n");
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		if (atoi(argv[1]) == -1) {
			printf("ERR: Buffer should be provided for automatic deletion.\n");
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		buf_len = sizeof(tlvbuf_custom_ie) + sizeof(custom_ie);
		break;
	case 4:
		/* This is to check negative numbers and special symbols */
		if (UAP_FAILURE == IS_HEX_OR_DIGIT(argv[2])) {
			printf("ERR:Mask value must be 0 or hex digits\n");
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		/* If above check is passed and mask is not hex, then it must be 0 */
		if ((ISDIGIT(argv[2]) == UAP_SUCCESS) && atoi(argv[2])) {
			printf("ERR:Mask value must be 0 or hex digits\n ");
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		if (UAP_FAILURE == ishexstring(argv[3])) {
			printf("ERR:Only hex digits are allowed\n");
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		ie_buf_len = strlen(argv[3]);
		if (!strncasecmp("0x", argv[3], 2)) {
			ie_len = (ie_buf_len - 2 + 1) / 2;
			argv[3] += 2;
		} else
			ie_len = (ie_buf_len + 1) / 2;
		if (ie_len > MAX_IE_BUFFER_LEN) {
			printf("ERR:Incorrect IE length %d\n", ie_buf_len);
			print_sys_cfg_custom_ie_usage();
			return UAP_FAILURE;
		}
		mgmt_subtype_mask = (t_u16)A2HEXDECIMAL(argv[2]);
		buf_len = sizeof(tlvbuf_custom_ie) + sizeof(custom_ie) + ie_len;
		break;
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);
	tlv = (tlvbuf_custom_ie *)buffer;
	tlv->tag = MRVL_MGMT_IE_LIST_TLV_ID;
	if (argc == 1 || argc == 2) {
		if (argc == 1)
			tlv->length = 0;
		else {
			tlv->length = sizeof(t_u16);
			ie_ptr = (custom_ie *)(tlv->ie_data);
			ie_ptr->ie_index = (t_u16)(atoi(argv[1]));
		}
	} else {
		/* Locate headers */
		ie_ptr = (custom_ie *)(tlv->ie_data);
		/* Set TLV fields */
		tlv->length = sizeof(custom_ie) + ie_len;
		ie_ptr->ie_index = atoi(argv[1]);
		ie_ptr->mgmt_subtype_mask = mgmt_subtype_mask;
		ie_ptr->ie_length = ie_len;
		if (argc == 4)
			string2raw(argv[3], ie_ptr->ie_buffer);
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		if (buffer)
			free(buffer);
		return UAP_FAILURE;
	}
	if (argc != 1) {
		/* Initialize the ifr structure */
		memset(&ifr, 0, sizeof(ifr));
		strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
		ifr.ifr_ifru.ifru_data = (void *)buffer;
		/* Perform ioctl */
		if (ioctl(sockfd, UAP_CUSTOM_IE, &ifr)) {
			if (errno < 0) {
				perror("ioctl[UAP_CUSTOM_IE]");
				printf("ERR:Command sending failed!\n");
			} else {
				printf("custom IE configuration failed!\n");
			}
			close(sockfd);
			if (buffer)
				free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (argc > 2) {
			printf("custom IE setting successful\n");
		} else {
			printf("Querying custom IE successful\n");
			tlv = (tlvbuf_custom_ie *)buffer;
			ie_len = tlv->length;
			ie_ptr = (custom_ie *)(tlv->ie_data);
			if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
				while ((unsigned int)ie_len >=
				       sizeof(custom_ie)) {
					printf("Index [%d]\n",
					       ie_ptr->ie_index);
					if (ie_ptr->ie_length)
						printf("Management Subtype Mask = 0x%02x\n", ie_ptr->mgmt_subtype_mask == 0 ? UAP_CUSTOM_IE_AUTO_MASK : ie_ptr->mgmt_subtype_mask);
					else
						printf("Management Subtype Mask = 0x%02x\n", ie_ptr->mgmt_subtype_mask);
					hexdump_data("IE Buffer",
						     (void *)ie_ptr->ie_buffer,
						     (ie_ptr->ie_length), ' ');
					ie_len -=
						sizeof(custom_ie) +
						ie_ptr->ie_length;
					ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
							       sizeof(custom_ie)
							       +
							       ie_ptr->
							       ie_length);
				}
			}
			max_mgmt_ie_tlv =
				(tlvbuf_max_mgmt_ie *)(buffer +
						       sizeof(tlvbuf_custom_ie)
						       + tlv->length);
			if (max_mgmt_ie_tlv) {
				if (max_mgmt_ie_tlv->tag ==
				    MRVL_MAX_MGMT_IE_TLV_ID) {
					for (i = 0; i < max_mgmt_ie_tlv->count;
					     i++) {
						printf("buf%d_size = %d\n", i,
						       max_mgmt_ie_tlv->info[i].
						       buf_size);
						printf("number of buffers = %d\n", max_mgmt_ie_tlv->info[i].buf_count);
						printf("\n");
					}
				}
			}
		}
	}

	/* Special handling for all indices: Get all IEs one-by-one */
	if (argc == 1) {
		for (i = 0; i < max_mgmt_ie; i++) {

			tlv->length = sizeof(t_u16);
			ie_ptr = (custom_ie *)(tlv->ie_data);
			ie_ptr->ie_index = (t_u16)(i);

			/* Initialize the ifr structure */
			memset(&ifr, 0, sizeof(ifr));
			strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
			ifr.ifr_ifru.ifru_data = (void *)buffer;
			/* Perform ioctl */
			if (ioctl(sockfd, UAP_CUSTOM_IE, &ifr)) {
				if (errno < 0) {
					perror("ioctl[UAP_CUSTOM_IE]");
					printf("ERR:Command sending failed!\n");
				} else {
					printf("custom IE configuration failed!\n");
				}
				close(sockfd);
				if (buffer)
					free(buffer);
				return UAP_FAILURE;
			}
			/* Print response */
			tlv = (tlvbuf_custom_ie *)buffer;
			ie_len = tlv->length;
			ie_ptr = (custom_ie *)(tlv->ie_data);
			if (tlv->tag == MRVL_MGMT_IE_LIST_TLV_ID) {
				while ((unsigned int)ie_len >=
				       sizeof(custom_ie)) {
					printf("Index [%d]\n",
					       ie_ptr->ie_index);
					if (ie_ptr->ie_length)
						printf("Management Subtype Mask = 0x%02x\n", ie_ptr->mgmt_subtype_mask == 0 ? UAP_CUSTOM_IE_AUTO_MASK : ie_ptr->mgmt_subtype_mask);
					else
						printf("Management Subtype Mask = 0x%02x\n", ie_ptr->mgmt_subtype_mask);
					hexdump_data("IE Buffer",
						     (void *)ie_ptr->ie_buffer,
						     (ie_ptr->ie_length), ' ');
					ie_len -=
						sizeof(custom_ie) +
						ie_ptr->ie_length;
					ie_ptr = (custom_ie *)((t_u8 *)ie_ptr +
							       sizeof(custom_ie)
							       +
							       ie_ptr->
							       ie_length);
				}
			}
		}
		printf("Querying custom IE successful\n");
	}

	if (buffer)
		free(buffer);

	close(sockfd);
	return UAP_SUCCESS;
}

#if defined(DFS_TESTING_SUPPORT)
/**
 *  @brief Show usage information for the dfstesting command
 *
 *  @return         N/A
 */
void
print_dfstesting_usage(void)
{
	printf("\nUsage : dfstesting [USER_CAC_PD  USER_NOP_PD  NO_CHAN_CHANGE  FIXED_CHAN_NUM]\n");
	printf("\n         empty - Get all dfstesting settings\n");
	printf("\n         USER_CAC_PD:  user configured Channel Availability Check period");
	printf("\n                 0       - disable, use default (60000)");
	printf("\n                 1-65535 - CAC period in msec");
	printf("\n         USER_NOP_PD:  user configured Non-Occupancy Period");
	printf("\n                 0       - disable, use default (1800)");
	printf("\n                 1-65535 - NOP period in sec");
	printf("\n         NO_CHAN_CHANGE:  user setting, don't change channel on radar");
	printf("\n                 0        - disable, default behavior");
	printf("\n                 non-zero - enable, overrides below setting");
	printf("\n         FIXED_CHAN_NUM:  user fixed channel to change to on radar");
	printf("\n                 0     - disable, use random channel [default]");
	printf("\n                 1-255 - set fixed channel (not checked for validity)\n");
	return;
}

/**
 *  @brief user configuration of dfs testing settings
 *
 *   Usage: "dfstesting [USER_CAC_PD  USER_NOP_PD  NO_CHAN_CHANGE  FIXED_CHAN_NUM]"
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_dfstesting(int argc, char *argv[])
{
	int opt;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u32 val;
	dfs_testing_para dfs_test;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_dfstesting_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	memset(&dfs_test, 0x00, sizeof(dfs_test));

	/* Check arguments */
	if (argc == 0) {
		dfs_test.action = ACTION_GET;
	} else if (argc == 4) {
		if ((IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) ||
		    (IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE) ||
		    (IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE) ||
		    (IS_HEX_OR_DIGIT(argv[3]) == UAP_FAILURE)) {
			printf("ERR: Only Number values are allowed\n");
			print_dfstesting_usage();
			return UAP_FAILURE;
		}

		val = A2HEXDECIMAL(argv[0]);
		if (val > 0xfffff) {
			printf("ERR: invalid user_cac_pd value!\n");
			return UAP_FAILURE;
		}
		dfs_test.usr_cac_period = (t_u32)val;

		val = A2HEXDECIMAL(argv[1]);
		if (val > 0xffff) {
			printf("ERR: invalid user_nop_pd value!\n");
			return UAP_FAILURE;
		}
		dfs_test.usr_nop_period = (t_u16)val;

		val = A2HEXDECIMAL(argv[2]);
		dfs_test.no_chan_change = (t_u8)(val ? 1 : 0);

		val = A2HEXDECIMAL(argv[3]);
		if (val > 0xff) {
			printf("ERR: invalid fixed_chan_num value!\n");
			return UAP_FAILURE;
		}
		dfs_test.fixed_new_chan = (t_u8)val;
		dfs_test.action = ACTION_SET;
	} else {
		printf("ERR: invalid number of arguments!  Must be 0 or 4.\n");
		print_dfstesting_usage();
		return UAP_FAILURE;
	}
	dfs_test.subcmd = UAP_DFS_TESTING;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&dfs_test;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: UAP_DFS_TESTING is not supported by %s\n",
		       dev_name);
		close(sockfd);
		return UAP_FAILURE;
	}

	if (argc)
		printf("DFS testing setting successful!\n");
	else
		printf("DFS testing settings:\n"
		       "    user_cac_period   = %d msec\n"
		       "    user_nop_period   = %d sec\n"
		       "    no_channel_change = %d\n"
		       "    fixed_channel_num = %d\n",
		       dfs_test.usr_cac_period, dfs_test.usr_nop_period,
		       dfs_test.no_chan_change, dfs_test.fixed_new_chan);
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}
#endif /* DFS_SUPPORT && DFS_TESTING_SUPPORT */

/**
 *  @brief Show usage information for the cscount_cfg
 *   command
 *
 *  $return         N/A
 */
void
print_cscount_cfg_usage(void)
{
	printf("\nUsage : cscount [<n>]");
	printf("\n        Where  <n>  ");
	printf("\n        5-20: No of beacons with Channel Switch Count IE\n");
	return;
}

/**
 *  @brief Set/get cs_count configuration
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_cscount_cfg(int argc, char *argv[])
{
	int opt;
	cscount_cfg_t cscount_cfg;
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_cscount_cfg_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */

	memset(&cscount_cfg, 0, sizeof(cscount_cfg));
	if (argc == 0) {
		cscount_cfg.action = ACTION_GET;
	} else if (argc == 1) {
		if ((t_u32)A2HEXDECIMAL(argv[0]) < 5
		    && (t_u32)A2HEXDECIMAL(argv[0]) > 20) {
			printf("ERR:Invalid Channel switch count value\n");
			return UAP_FAILURE;
		}
		cscount_cfg.action = ACTION_SET;
		cscount_cfg.cs_count = (t_u32)A2HEXDECIMAL(argv[0]);
	} else {
		print_cscount_cfg_usage();
		return UAP_FAILURE;
	}
	cscount_cfg.subcmd = UAP_CHAN_SWITCH_COUNT_CFG;

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&cscount_cfg;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: Channel switch count configuration failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}

	/* Handle response */
	if (cscount_cfg.action == ACTION_GET) {
		printf("Channel Switch count = %d\n", cscount_cfg.cs_count);
	}

	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the mgmtframectrl command
 *
 *  $return         N/A
 */
void
print_mgmtframectrl_usage(void)
{
	printf("\nUsage : mgmtframectrl [MASK]\n");
	printf("         empty - Get management frame control mask\n");
	printf("         MASK  - Set management frame control mask\n");
}

/**
 *  @brief Creates management frame control request and send to driver
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_mgmt_frame_control(int argc, char *argv[])
{
	int opt;
	mgmt_frame_ctrl param;	/* Action =0, Mask =0 */
	struct ifreq ifr;
	t_s32 sockfd;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_mgmtframectrl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if (argc > 1) {
		printf("ERR:wrong arguments.\n");
		print_mgmtframectrl_usage();
		return UAP_FAILURE;
	}

	if ((argc) && (IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE)) {
		printf("ERR: Invalid argument %s\n", argv[0]);
		print_mgmtframectrl_usage();
		return UAP_FAILURE;
	}

	memset(&param, 0, sizeof(mgmt_frame_ctrl));
	param.subcmd = UAP_MGMT_FRAME_CONTROL;
	if (argc) {
		param.action = ACTION_SET;
		param.mask = (t_u16)A2HEXDECIMAL(argv[0]);
	} else {
		param.action = ACTION_GET;
	}
	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, strlen(dev_name));
	ifr.ifr_ifru.ifru_data = (void *)&param;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR:UAP_IOCTL_CMD failed\n");
		close(sockfd);
		return UAP_FAILURE;
	}
	if (!argc) {
		printf("Management Frame control mask = 0x%02x\n",
		       (int)param.mask);
	}
	/* Close socket */
	close(sockfd);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for the sys_cfg_tdls_ext_cap
 *   command
 *
 *  @return         N/A
 */
void
print_sys_cfg_tdls_ext_cap_usage(void)
{
	printf("\nUsage : sys_cfg_tdls_ext_cap [CONFIG_FILE] \n");
	printf("\nIf CONFIG_FILE is provided, a 'set' is performed, else a 'get' is performed.\n");
	printf("\n");
	return;
}

/**
 *  @brief  Get Extended capability info from firmware
 *
 *  @param  ext_cap  A pointer to tdls_ext_cap tlv
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
static int
get_tdls_ext_cap(tlvbuf_tdls_ext_cap *ext_cap_tlv)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_tdls_ext_cap *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = 0;
	int ret = UAP_SUCCESS;

	/* Initialize the buffer length */
	buf_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tdls_ext_cap) +
		MAX_TDLS_EXT_CAP_LEN;
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	/* Initialize the command buffer */
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_tdls_ext_cap *)(buffer + sizeof(apcmdbuf_sys_configure));
	tlv->tag = MRVL_TDLS_EXT_CAP_TLV_ID;
	/* Set TLV length to zero for GET request */
	tlv->length = 0;
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tdls_ext_cap) +
		tlv->length;
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = ACTION_GET;

	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_TDLS_EXT_CAP_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		/* Copy response */
		if (cmd_buf->result == CMD_SUCCESS) {
			memcpy(ext_cap_tlv, tlv,
			       sizeof(tlvbuf_tdls_ext_cap) + tlv->length);
		} else {
			ret = UAP_FAILURE;
			printf("ERR:Could not get TDLS extended capability!\n");
		}
	} else {
		printf("ERR:Command sending failed!\n");
		ret = UAP_FAILURE;
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Read the TDLS parameters and populate structure
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @param bss      Pointer to BSS configuration buffer
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
static int
parse_tdls_config(int argc, char *argv[], tlvbuf_tdls_ext_cap *ext_cap_tlv)
{
	FILE *config_file = NULL;
	char *line = NULL;
	char *pos = NULL;
	char *args[30];
	t_u8 *buffer = NULL;
	tlvbuf_tdls_ext_cap *prev_ext_cap_tlv = NULL;
	int li = 0, arg_num = 0, i = 0, is_tdls_config = 0;
	int ret = UAP_SUCCESS;

	/* Check if file exists */
	config_file = fopen(argv[0], "r");
	if (config_file == NULL) {
		printf("\nERR:Config file can not open.\n");
		return UAP_FAILURE;
	}

	buffer = (t_u8 *)malloc(MAX_TDLS_EXT_CAP_LEN);
	if (!buffer) {
		printf("ERR:Cannot allocate memory for buffer\n");
		ret = UAP_FAILURE;
		goto done;
	}
	memset(buffer, 0, MAX_TDLS_EXT_CAP_LEN);
	prev_ext_cap_tlv = (tlvbuf_tdls_ext_cap *)buffer;
	/* Get previous setting of extended capability from FW and copy it to ext_cap_tlv */
	if (UAP_SUCCESS != get_tdls_ext_cap(prev_ext_cap_tlv)) {
		printf("ERR: Couldn't get TDLS extended capability setting!\n");
		ret = UAP_FAILURE;
		goto done;
	}
	memcpy(ext_cap_tlv, prev_ext_cap_tlv,
	       sizeof(tlvbuf_tdls_ext_cap) + prev_ext_cap_tlv->length);

	/* Parse file and process */
	line = (char *)malloc(MAX_CONFIG_LINE);
	if (!line) {
		printf("ERR:Cannot allocate memory for line\n");
		ret = UAP_FAILURE;
		goto done;
	}
	memset(line, 0, MAX_CONFIG_LINE);

	while (config_get_line(line, MAX_CONFIG_LINE, config_file, &li, &pos)) {
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Received config line (%d) = %s\n",
			   li, line);
#endif
		arg_num = parse_line(line, args);
#if DEBUG
		uap_printf(MSG_DEBUG, "DBG:Number of arguments = %d\n",
			   arg_num);
		for (i = 0; i < arg_num; i++) {
			uap_printf(MSG_DEBUG, "\tDBG:Argument %d. %s\n", i + 1,
				   args[i]);
		}
#endif
		/* Check for end of TDLS configurations */
		if (is_tdls_config == 1) {
			if (strcmp(args[0], "}") == 0) {
				is_tdls_config = 0;
			}
		}
		/* Check for beginning of TDLS configurations */
		if (strcmp(args[0], "tdls_config") == 0) {
			is_tdls_config = 1;
		}
		if (is_tdls_config) {
			if (strcmp(args[0], "tdls_prohibit") == 0) {
				if ((ISDIGIT(args[1]) != UAP_SUCCESS) ||
				    (atoi(args[1]) < 0) ||
				    (atoi(args[1]) > 1)) {
					printf("ERR: Invalid %s value!\n",
					       args[0]);
					ret = UAP_FAILURE;
					goto done;
				}
				ext_cap_tlv->ext_cap[4] &=
					(~TDLS_PROHIBIT_MASK);
				ext_cap_tlv->ext_cap[4] |=
					(atoi(args[1]) << TDLS_PROHIBIT);
			}
			if (strcmp(args[0], "tdls_channel_switch_prohibit") ==
			    0) {
				if ((ISDIGIT(args[1]) != UAP_SUCCESS) ||
				    (atoi(args[1]) < 0) ||
				    (atoi(args[1]) > 1)) {
					printf("ERR: Invalid %s value!\n",
					       args[0]);
					ret = UAP_FAILURE;
					goto done;
				}
				ext_cap_tlv->ext_cap[4] &=
					(~TDLS_CHANNEL_SWITCH_PROHIBIT_MASK);
				ext_cap_tlv->ext_cap[4] |=
					(atoi(args[1]) <<
					 TDLS_CHANNEL_SWITCH_PROHIBIT);
			}
		}
	}
done:
	fclose(config_file);
	if (buffer)
		free(buffer);
	if (line)
		free(line);
	return ret;
}

/**
 *  @brief Creates a sys_cfg request for setting extended capability for TDLS
 *  and sends to the driver
 *
 *   Usage: "sys_cfg_tdls_ext_cap [CONFIG_FILE]"
 *           If CONFIG_FILE is provided, a 'set' is performed
 *           else a 'get' is performed.
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
apcmd_sys_cfg_tdls_ext_cap(int argc, char *argv[])
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_tdls_ext_cap *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	int opt;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_tdls_ext_cap_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc > 1) {
		printf("ERR: Invalid arguments!\n");
		return UAP_FAILURE;
	}

	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(buf_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(buffer, 0, buf_len);

	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_tdls_ext_cap *)(buffer + sizeof(apcmdbuf_sys_configure));
	if (argc == 1) {
		/* Get current TDLS extended capability setting from FW and
		   parse new setting from config file */
		if (UAP_SUCCESS != parse_tdls_config(argc, argv, tlv)) {
			free(buffer);
			return UAP_FAILURE;
		}
	}
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	tlv->tag = MRVL_TDLS_EXT_CAP_TLV_ID;
	if (argc == 0) {
		cmd_buf->action = ACTION_GET;
		tlv->length = 0;
	} else {
		cmd_buf->action = ACTION_SET;
	}
	cmd_buf->size =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tdls_ext_cap) +
		tlv->length;
	cmd_len =
		sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_tdls_ext_cap) +
		tlv->length;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);
	endian_convert_tlv_header_in(tlv);
	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if ((cmd_buf->cmd_code !=
		     (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) ||
		    (tlv->tag != MRVL_TDLS_EXT_CAP_TLV_ID)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}
		/* Print response */
		if (cmd_buf->result == CMD_SUCCESS) {
			if (argc == 0) {
				printf("TDLS prohibited = %s\n",
				       (tlv->
					ext_cap[4] & TDLS_PROHIBIT_MASK) ? "YES"
				       : "NO");
				printf("TDLS Channel switch prohibited = %s\n",
				       (tlv->
					ext_cap[4] &
					TDLS_CHANNEL_SWITCH_PROHIBIT_MASK) ?
				       "YES" : "NO");
			} else {
				printf("TDLS setting successful\n");
			}
		} else {
			if (argc == 0) {
				printf("ERR:Could not get tdls setting!\n");
			} else {
				printf("ERR:Could not set tdls setting!\n");
			}
			ret = UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *    @brief Print usage information for the sys_cfg_pmf command
 *
 *    @return         N/A
 */
void
print_sys_cfg_pmf(void)
{
	printf("\nUsage : uaputl.exe sys_cfg_pmf [MFPC] [MFPR]\n");
	printf("\nSet/Get PMF capabilities");
	printf("\n          empty - Get PMF capabilites\n");
	printf("\n          MFPC: Management frames protection capable");
	printf("\n              0 - Not capable");
	printf("\n              1 - capable");
	printf("\n          MFPR: Management frames protection required");
	printf("\n          don't care if MFPC is set to 0");
	printf("\n              0 - Not required");
	printf("\n              1 - required\n");
	return;
}

/**
 *    @brief Show usage information for the sys_cfg_pmf command
 *
 *    @param argc     Number of arguments
 *    @param argv     Pointer to the arguments
 *
 *    @return         UAP_SUCCESS or UAP_FAILURE
 */
int
apcmd_sys_cfg_pmf(int argc, char *argv[])
{
	int opt;
	apcmdbuf_pmf_params *cmd_buf = NULL;
	t_u8 *buf = NULL;
	t_u16 cmd_len = 0;
	t_u16 buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	int ret = UAP_SUCCESS;
	t_u8 mfpc = 0;
	t_u8 mfpr = 0;

	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_sys_cfg_pmf();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	/* Check arguments */
	if ((argc > 2)) {
		printf("ERR:wrong arguments.\n");
		print_sys_cfg_pmf();
		return UAP_FAILURE;
	}

	if (argc > 0)
		mfpc = atoi(argv[0]);

	if (mfpc && (argc == 1)) {
		printf("ERR:wrong arguments.\n");
		print_sys_cfg_pmf();
		return UAP_FAILURE;
	}

	if (mfpc && (argc == 2))
		mfpr = atoi(argv[1]);

	/* Alloc buf for command */
	buf = (t_u8 *)malloc(buf_len);

	if (!buf) {
		printf("ERR:Cannot allocate buffer from command!\n");
		return UAP_FAILURE;
	}
	memset(buf, 0, buf_len);

	/* Locate headers */
	cmd_len = sizeof(apcmdbuf_pmf_params);
	cmd_buf = (apcmdbuf_pmf_params *) buf;

	/* Fill the command buffer */
	cmd_buf->size = cmd_len - BUF_HEADER_SIZE;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->cmd_code = HostCmd_CMD_PMF_PARAMS;
	if (argc == 0)
		cmd_buf->action = ACTION_GET;
	else
		cmd_buf->action = ACTION_SET;
	cmd_buf->action = uap_cpu_to_le16(cmd_buf->action);
	cmd_buf->params.mfpc = mfpc;
	cmd_buf->params.mfpr = mfpr;

	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, buf_len);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		if (cmd_buf->result == CMD_SUCCESS) {
			printf("Successfully executed the command\n");
			printf("mfpc: %d, mfpr: %d\n",
			       cmd_buf->params.mfpc, cmd_buf->params.mfpr);
		} else {
			printf("ERR:Command sending failed!\n");
			free(buf);
			return UAP_FAILURE;
		}
	} else {
		printf("ERR:Command sending failed!\n");
		free(buf);
		return UAP_FAILURE;
	}
	free(buf);
	return UAP_SUCCESS;
}

/**
 *  @brief Show usage information for uap operation control command
 *
 *  @return         N/A
 */
void
print_uap_oper_ctrl_usage(void)
{
	printf("\nUsage : uap_oper_ctrl <control> <chanopt> <bandcfg> <channel> ");
	printf("\n set/get uap operation when in-STA disconnected from ext-AP if multi channel is disabled");
	printf("\n       control:    0: default, do nothing");
	printf("\n                      2: uap stops and restart automatically");
	printf("\n        chanopt : specify which channel should be used when uap restarts automatically");
	printf("\n                      1: uap restarts on default 2.4G/channel 6");
	printf("\n                      2: uap restart on band/channel configured by driver previously");
	printf("\n                      3: uap restart on band/channel configured by parameter bandcfg/channel");
	printf("\n         bandcfg : This parameter specifies the bandwidth (BW)");
	printf("\n                      0: 20Mhz");
	printf("\n                      2: 40Mhz");
	printf("\n                      3: 80Mhz");
	printf("\n         channel : This parameter specifies the channel will be used when chanopt is 3.");
	return;
}

/**
 *  @brief Set/Get uap operation when in-STA disconnected from ext-AP
 *  @param argc   Number of arguments
 *  @param argv   A pointer to arguments array
 *  @return     MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
apcmd_uap_oper_ctrl(int argc, char *argv[])
{
	int opt;
	uap_operation_ctrl param;
	struct eth_priv_uap_oper_ctrl *uap_oper = NULL;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u8 *respbuf = NULL;

	memset(&param, 0, sizeof(param));
	while ((opt = getopt_long(argc, argv, "+", cmd_options, NULL)) != -1) {
		switch (opt) {
		default:
			print_uap_oper_ctrl_usage();
			return UAP_SUCCESS;
		}
	}
	argc -= optind;
	argv += optind;

	respbuf = (t_u8 *)malloc(MRVDRV_SIZE_OF_CMD_BUFFER);
	if (!respbuf) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return UAP_FAILURE;
	}
	memset(respbuf, 0, MRVDRV_SIZE_OF_CMD_BUFFER);

	/* Check arguments */
	if (argc > 4) {
		printf("ERR: Invalid number of arguments.\n");
		print_uap_oper_ctrl_usage();
		free(respbuf);
		return UAP_FAILURE;
	}

	if (argc > 0) {
		param.uap_oper.ctrl = (t_u16)A2HEXDECIMAL(argv[0]);
		if (param.uap_oper.ctrl == 2)
			param.uap_oper.chan_opt = (t_u16)A2HEXDECIMAL(argv[1]);

		if (argc == 4 && param.uap_oper.chan_opt == 3) {
			param.uap_oper.bandcfg = (t_u8)A2HEXDECIMAL(argv[2]);
			param.uap_oper.channel = (t_u8)A2HEXDECIMAL(argv[3]);
		}
		param.action = ACTION_SET;
	} else {
		param.action = ACTION_GET;
	}
	param.subcmd = UAP_OPERATION_CTRL;

	memcpy(respbuf, &param, sizeof(uap_operation_ctrl));

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		free(respbuf);
		return UAP_FAILURE;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)respbuf;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		perror("");
		printf("ERR: uap operation control set/get failed\n");
		close(sockfd);
		free(respbuf);
		return UAP_FAILURE;
	}

	/* Handle response */
	if (param.action == ACTION_GET) {
		/* Process result */
		uap_oper =
			(struct eth_priv_uap_oper_ctrl *)(respbuf +
							  2 * sizeof(t_u32));
		printf("    uap operation control %x\n", uap_oper->ctrl);
		printf("    uap channel operation %x\n", uap_oper->chan_opt);
		if (uap_oper->chan_opt == 3) {
			printf("    uap bandwidth %s\n",
			       uap_oper->
			       bandcfg ? ((uap_oper->bandcfg == 2) ? "40Mhz" :
					  "80Mhz") : "20Mhz");
			printf("    uap channel %d\n", uap_oper->channel);
		}
	} else
		printf("uap operation control set success!\n");
	/* Close socket */
	close(sockfd);
	if (respbuf)
		free(respbuf);
	return UAP_SUCCESS;
}

/** Structure of command table*/
typedef struct {
    /** Command name */
	char *cmd;
    /** Command function pointer */
	int (*func) (int argc, char *argv[]);
    /** Command usuage */
	char *help;
} command_table;

/** AP command table */
static command_table ap_command[] = {
	{"sys_config", apcmd_sys_config, "\tSet/get uAP's profile"},
	{"sys_info", apcmd_sys_info, "\tDisplay system info"},
	{"sys_reset", apcmd_sys_reset, "\tReset uAP"},
	{"bss_start", apcmd_bss_start, "\tStart the BSS"},
	{"bss_stop", apcmd_bss_stop, "\tStop the BSS"},
	{"skip_cac", apcmd_skip_cac, "\tSkip the CAC"},
	{"sta_deauth", apcmd_sta_deauth, "\tDeauth client"},
	{"sta_list", apcmd_sta_list, "\tDisplay list of clients"},
	{"sys_cfg_ap_mac_address", apcmd_sys_cfg_ap_mac_address,
	 "Set/get uAP mac address"},
	{"sys_cfg_ssid", apcmd_sys_cfg_ssid, "\tSet/get uAP ssid"},
	{"sys_cfg_beacon_period", apcmd_sys_cfg_beacon_period,
	 "Set/get uAP beacon period"},
	{"sys_cfg_dtim_period", apcmd_sys_cfg_dtim_period,
	 "Set/get uAP dtim period"},
	{"sys_cfg_bss_status", apcmd_sys_cfg_bss_status, "Get BSS status"},
	{"sys_cfg_channel", apcmd_sys_cfg_channel,
	 "\tSet/get uAP radio channel"},
	{"sys_cfg_channel_ext", apcmd_sys_cfg_channel_ext,
	 "\tSet/get uAP radio channel, band and mode"},
	{"sys_cfg_scan_channels", apcmd_sys_cfg_scan_channels,
	 "Set/get uAP radio channel list"},
	{"sys_cfg_rates", apcmd_sys_cfg_rates, "\tSet/get uAP rates"},
	{"sys_cfg_rates_ext", apcmd_sys_cfg_rates_ext, "\tSet/get uAP rates"},
	{"sys_cfg_tx_power", apcmd_sys_cfg_tx_power, "Set/get uAP tx power"},
	{"sys_cfg_bcast_ssid_ctl", apcmd_sys_cfg_bcast_ssid_ctl,
	 "Set/get uAP broadcast ssid"},
	{"sys_cfg_preamble_ctl", apcmd_sys_cfg_preamble_ctl,
	 "Get uAP preamble"},
	{"antcfg", apcmd_antcfg, "Set/get uAP tx/rx antenna"},
	{"htstreamcfg", apcmd_htstreamcfg,
	 "Set/get uAP HT stream configurations"},
	{"sys_cfg_rts_threshold", apcmd_sys_cfg_rts_threshold,
	 "Set/get uAP rts threshold"},
	{"sys_cfg_frag_threshold", apcmd_sys_cfg_frag_threshold,
	 "Set/get uAP frag threshold"},
	{"radioctrl", apcmd_radio_ctl, "Set/get uAP radio on/off"},
	{"sys_cfg_tx_beacon_rate", apcmd_sys_cfg_tx_beacon_rate,
	 "Set/get uAP tx beacon rate"},
	{"txratecfg", apcmd_tx_rate_cfg, "Set/get trasnmit data rate"},
	{"sys_cfg_mcbc_data_rate", apcmd_sys_cfg_mcbc_data_rate,
	 "Set/get uAP MCBC rate"},
	{"sys_cfg_rsn_replay_prot", apcmd_sys_cfg_rsn_replay_prot,
	 "Set/get RSN replay protection"},
	{"sys_cfg_pkt_fwd_ctl", apcmd_sys_cfg_pkt_fwd_ctl,
	 "Set/get uAP packet forwarding"},
	{"sys_cfg_sta_ageout_timer", apcmd_sys_cfg_sta_ageout_timer,
	 "Set/get station ageout timer"},
	{"sys_cfg_ps_sta_ageout_timer", apcmd_sys_cfg_ps_sta_ageout_timer,
	 "Set/get PS station ageout timer"},
	{"sys_cfg_auth", apcmd_sys_cfg_auth,
	 "\tSet/get uAP authentication mode"},
	{"sys_cfg_protocol", apcmd_sys_cfg_protocol,
	 "Set/get uAP security protocol"},
	{"sys_cfg_wep_key", apcmd_sys_cfg_wep_key, "\tSet/get uAP wep key"},
	{"sys_cfg_cipher", apcmd_sys_cfg_cipher,
	 "\tSet/get uAP WPA/WPA2 cipher"},
	{"sys_cfg_pwk_cipher", apcmd_sys_cfg_pwk_cipher,
	 "\tSet/get uAP WPA/WPA2 pairwise cipher"},
	{"sys_cfg_gwk_cipher", apcmd_sys_cfg_gwk_cipher,
	 "\tSet/get uAP WPA/WPA2 group cipher"},
	{"sys_cfg_wpa_passphrase", apcmd_sys_cfg_wpa_passphrase,
	 "Set/get uAP WPA or WPA2 passphrase"},
	{"sys_cfg_group_rekey_timer", apcmd_sys_cfg_group_rekey_timer,
	 "Set/get uAP group re-key time"},
	{"sys_cfg_max_sta_num", apcmd_sys_cfg_max_sta_num,
	 "Set/get uAP max station number"},
	{"sys_cfg_retry_limit", apcmd_sys_cfg_retry_limit,
	 "Set/get uAP retry limit number"},
	{"sys_cfg_sticky_tim_config", apcmd_sys_cfg_sticky_tim_config,
	 "Set/get uAP sticky TIM configuration"},
	{"sys_cfg_sticky_tim_sta_mac_addr",
	 apcmd_sys_cfg_sticky_tim_sta_mac_addr,
	 "Set/get uAP sticky TIM sta MAC address"},
	{"sys_cfg_eapol_pwk_hsk", apcmd_sys_cfg_eapol_pwk_hsk,
	 "Set/getuAP pairwise Handshake timeout value and retries"},
	{"sys_cfg_eapol_gwk_hsk", apcmd_sys_cfg_eapol_gwk_hsk,
	 "Set/getuAP groupwise Handshake timeout value and retries"},
	{"sys_cfg_custom_ie", apcmd_sys_cfg_custom_ie,
	 "\tSet/get custom IE configuration"},
	{"sta_filter_table", apcmd_sta_filter_table, "Set/get uAP mac filter"},
	{"regrdwr", apcmd_regrdwr, "\t\tRead/Write register command"},
	{"memaccess", apcmd_memaccess,
	 "\tRead/Write to a memory address command"},
	{"rdeeprom", apcmd_read_eeprom, "\tRead EEPROM "},
	{"cfg_data", apcmd_cfg_data,
	 "\tGet/Set configuration file from/to firmware"},
	{"sys_cfg_80211d", apcmd_cfg_80211d, "\tSet/Get 802.11D info"},
	{"uap_stats", apcmd_uap_stats, "\tGet uAP stats"},
	{"pscfg", apcmd_pscfg, "\t\tSet/get uAP power mode"},
	{"bss_config", apcmd_bss_config, "\tSet/get BSS configuration"},
	{"sta_deauth_ext", apcmd_sta_deauth_ext, "\tDeauth client"},
	{"coex_config", apcmd_coex_config,
	 "\tSet/get uAP BT coex configuration"},
	{"hscfg", apcmd_hscfg, "\t\tSet/get uAP host sleep parameters."},
	{"hssetpara", apcmd_hssetpara,
	 "\t\tSet/get uAP host sleep parameters."},
	{"addbapara", apcmd_addbapara, "\tSet/get uAP ADDBA parameters."},
	{"aggrpriotbl", apcmd_aggrpriotbl,
	 "\tSet/get uAP priority table for AMPDU/AMSDU."},
	{"addbareject", apcmd_addbareject, "\tSet/get uAP addbareject table."},
	{"sys_cfg_11n", apcmd_sys_cfg_11n, "\tSet/get uAP 802.11n parameters."},
#ifdef RX_PACKET_COALESCE
	{"rxpktcoal_cfg", apcmd_rx_pkt_coalesce,
	 "\tSet/get RX Packet coalesing paramterts."},
#endif
	{"httxbfcfg", apcmd_sys_cfg_tx_bf, "\tSet/get uAP TX BF parameters."},
	{"httxcfg", apcmd_sys_cfg_ht_tx, "\t\tSet/get uAP HT Tx parameters."},
	{"vhtcfg", apcmd_sys_cfg_vht, "\t\tSet/get uAP VHT parameters."},
	{"sys_cfg_wmm", apcmd_sys_cfg_wmm,
	 "\tSet/get uAP beacon wmm parameters."},
	{"sys_cfg_ap_wmm", apcmd_sys_cfg_ap_wmm,
	 "\tSet/get uAP hardware wmm parameters."},
	{"deepsleep", apcmd_deepsleep, "\tSet/get deepsleep mode."},
	{"hostcmd", apcmd_hostcmd, "\t\tSet/get hostcmd"},
	{"tx_data_pause", apcmd_txdatapause,
	 "\tSet/get Tx data pause settings."},
	{"sys_cfg_2040_coex", apcmd_sys_cfg_2040_coex,
	 "\tSet/get 20/40 coex settings."},
#if defined(DFS_TESTING_SUPPORT)
	{"dfstesting", apcmd_dfstesting, "\tConfigure DFS Testing settings."},
#endif
	{"cscount", apcmd_cscount_cfg, "\tConfigure DFS Channel Switch Count."},
	{"mgmtframectrl", apcmd_mgmt_frame_control,
	 "\tSpecifies mask indicating management frames to be sent from host."},
	{"sys_cfg_tdls_ext_cap", apcmd_sys_cfg_tdls_ext_cap,
	 "\tSet/get TDLS parameters in extended capabilities through config file."},
	{"sys_cfg_restrict_client_mode", apcmd_sys_cfg_restrict_client_mode,
	 "\tSet/get the mode in which client stations can connect to the uAP."},
	{"sys_cfg_pmf", apcmd_sys_cfg_pmf, "\tSet/get PMF capabilities."},
	{"uap_oper_ctrl", apcmd_uap_oper_ctrl,
	 "\tSet/get uap operation control value."},
	{NULL, NULL, 0}
};

/**
 *  @brief Prints usage information of uaputl
 *
 *  @return          N/A
 */
static void
print_tool_usage(void)
{
	int i;
	printf("uaputl.exe - uAP utility ver %s\n", UAP_VERSION);
	printf("Usage:\n"
	       "\tuaputl.exe [options] <command> [command parameters]\n");
	printf("Options:\n"
	       "\t--help\tDisplay help\n"
	       "\t-v\tDisplay version\n"
	       "\t-i <interface>\n" "\t-d <debug_level=0|1|2>\n");
	printf("Commands:\n");
	for (i = 0; ap_command[i].cmd; i++)
		printf("\t%-4s\t\t%s\n", ap_command[i].cmd, ap_command[i].help);
	printf("\n"
	       "For more information on the usage of each command use:\n"
	       "\tuaputl.exe <command> --help\n");
}

/****************************************************************************
        Global functions
****************************************************************************/
/** Option parameter*/
static struct option ap_options[] = {
	{"help", 0, NULL, 'h'},
	{"interface", 1, NULL, 'i'},
	{"debug", 1, NULL, 'd'},
	{"version", 0, NULL, 'v'},
	{NULL, 0, NULL, '\0'}
};

/**
 * @brief Checks if given channel in 'a' band is valid or not.
 *
 * @param channel   Channel number
 * @return          UAP_SUCCESS or UAP_FAILURE
 */
int
is_valid_a_band_channel(int channel)
{
	int ret = UAP_SUCCESS;
	switch (channel) {
	case 16:
	case 34:
	case 36:
	case 38:
	case 40:
	case 42:
	case 44:
	case 46:
	case 48:
	case 52:
	case 56:
	case 60:
	case 64:
	case 100:
	case 104:
	case 108:
	case 112:
	case 116:
	case 120:
	case 124:
	case 128:
	case 132:
	case 136:
	case 140:
	case 144:
	case 149:
	case 153:
	case 157:
	case 161:
	case 165:
		break;
	default:
		ret = UAP_FAILURE;
		break;
	}
	return ret;
}

/**
 * @brief Checks if secondary channel can be set above given primary channel in 'a' band or not.
 *
 * @param channel   Channel number
 * @return          UAP_SUCCESS or UAP_FAILURE
 */
int
is_valid_a_band_channel_above(int channel)
{
	int ret = UAP_SUCCESS;
	switch (channel) {
	case 36:
	case 44:
	case 52:
	case 60:
	case 100:
	case 108:
	case 116:
	case 124:
	case 132:
	case 140:
	case 149:
	case 157:
		break;
	default:
		ret = UAP_FAILURE;
		break;
	}
	return ret;
}

/**
 * @brief Checks if secondary channel can be set below given primary channel in 'a' band or not.
 *
 * @param channel   Channel number
 * @return          UAP_SUCCESS or UAP_FAILURE
 */
int
is_valid_a_band_channel_below(int channel)
{
	int ret = UAP_SUCCESS;
	switch (channel) {
	case 40:
	case 48:
	case 56:
	case 64:
	case 104:
	case 112:
	case 120:
	case 128:
	case 136:
	case 153:
	case 161:
		break;
	default:
		ret = UAP_FAILURE;
		break;
	}
	return ret;
}

/**
 *  @brief Checkes a particular input for validatation.
 *
 *  @param cmd      Type of input
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_input_valid(valid_inputs cmd, int argc, char *argv[])
{
	int i;
	int chan_number = 0;
	int band = 0;
	int ch;
	int ret = UAP_SUCCESS;
	if (argc == 0)
		return UAP_FAILURE;
	switch (cmd) {
	case RDEEPROM:
		if (argc != 2) {
			printf(" ERR: Argument count mismatch\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (ISDIGIT(argv[1]) == 0)
			    || (A2HEXDECIMAL(argv[0]) & 0x03) ||
			    ((int)(A2HEXDECIMAL(argv[0])) < 0) ||
			    (A2HEXDECIMAL(argv[1]) & 0x03) ||
			    (A2HEXDECIMAL(argv[1]) < 4) ||
			    (A2HEXDECIMAL(argv[1]) > 20)) {
				printf(" ERR: Invalid inputs for Read EEPROM\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case SCANCHANNELS:
		if (argc > MAX_CHANNELS) {
			printf("ERR: Invalid List of Channels\n");
			ret = UAP_FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				chan_number = -1;
				band = -1;
				sscanf(argv[i], "%d.%d", &chan_number, &band);
				if ((chan_number == -1) || (chan_number < 1) ||
				    (chan_number > MAX_CHANNELS)) {
					printf("ERR: Channel must be in the range of 1 to %d\n", MAX_CHANNELS);
					ret = UAP_FAILURE;
					break;
				}
				if ((chan_number > MAX_CHANNELS_BG) &&
				    !(is_valid_a_band_channel(chan_number))) {
					printf("ERR: Invalid Channel in 'a' band!\n");
					ret = UAP_FAILURE;
					break;
				}
				if ((band < -1) || (band > 1)) {
					printf("ERR:Band must be either 0 or 1\n");
					ret = UAP_FAILURE;
					break;
				} else {
					if (((chan_number < MAX_CHANNELS_BG) &&
					     (chan_number != 8) &&
					     (chan_number != 12) && (band == 1))
					    || ((chan_number > MAX_CHANNELS_BG)
						&& (band == 0))) {
						printf("ERR:Invalid band for given channel\n");
						ret = UAP_FAILURE;
						break;
					}
				}
			}
			if ((ret != UAP_FAILURE) &&
			    (has_dup_channel(argc, argv) != UAP_SUCCESS)) {
				printf("ERR: Duplicate channel values entered\n");
				ret = UAP_FAILURE;
			}
			if ((ret != UAP_FAILURE) &&
			    (has_diff_band(argc, argv) != UAP_SUCCESS)) {
				printf("ERR: Scan channel list should contain channels from only one band\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case TXPOWER:
		if ((argc > 1) || (ISDIGIT(argv[0]) == 0)) {
			printf("ERR:Invalid Transmit power\n");
			ret = UAP_FAILURE;
		} else {
			if ((atoi(argv[0]) < MIN_TX_POWER) ||
			    (atoi(argv[0]) > MAX_TX_POWER)) {
				printf("ERR: TX Powar must be in the rage of %d to %d. \n", MIN_TX_POWER, MAX_TX_POWER);
				ret = UAP_FAILURE;
			}
		}
		break;
	case PROTOCOL:
		if ((argc > 2) || (ISDIGIT(argv[0]) == 0)) {
			printf("ERR:Invalid Protocol\n");
			ret = UAP_FAILURE;
		} else
			ret = is_protocol_valid(atoi(argv[0]));
		break;
	case AKM_SUITE:
		if (argc == 2) {
			if (A2HEXDECIMAL(argv[1]) &
			    ~(KEY_MGMT_PSK | KEY_MGMT_PSK_SHA256 | KEY_MGMT_EAP
			      | KEY_MGMT_NONE)) {
				printf("ERR: Invalid AKM suite\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case CHANNEL:
		if ((argc != 1) && (argc != 2)) {
			printf("ERR: Incorrect arguments for channel.\n");
			ret = UAP_FAILURE;
		} else {
			if (argc == 2) {
				if ((ISDIGIT(argv[1]) == 0) ||
				    (atoi(argv[1]) & ~CHANNEL_MODE_MASK)) {
					printf("ERR: Invalid Mode\n");
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[1]) & BITMAP_ACS_MODE) &&
				    (atoi(argv[0]) != 0)) {
					printf("ERR: Channel must be 0 for ACS; MODE = 1.\n");
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[1]) & BITMAP_CHANNEL_ABOVE) &&
				    (atoi(argv[1]) & BITMAP_CHANNEL_BELOW)) {
					printf("ERR: secondary channel above and below both are enabled\n");
					ret = UAP_FAILURE;
				}
			}
			if ((argc == 1) || (!(atoi(argv[1]) & BITMAP_ACS_MODE))) {
				if ((ISDIGIT(argv[0]) == 0) ||
				    (atoi(argv[0]) < 1) ||
				    (atoi(argv[0]) > MAX_CHANNELS)) {
					printf("ERR: Channel must be in the range of 1 to %d\n", MAX_CHANNELS);
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[0]) > MAX_CHANNELS_BG) &&
				    !(is_valid_a_band_channel(atoi(argv[0])))) {
					printf("ERR: Invalid Channel in 'a' band!\n");
					ret = UAP_FAILURE;
				}
				ch = atoi(argv[0]);
				if (ch <= MAX_CHANNELS_BG) {
					if ((argc == 2) &&
					    (atoi(argv[1]) &
					     BITMAP_CHANNEL_ABOVE) &&
					    (atoi(argv[0]) >
					     MAX_CHANNEL_ABOVE)) {
						printf("ERR: only allow channel 1-9 for secondary channel above\n");
						ret = UAP_FAILURE;
					}
					if ((argc == 2) &&
					    (atoi(argv[1]) &
					     BITMAP_CHANNEL_BELOW) &&
					    ((atoi(argv[0]) < MIN_CHANNEL_BELOW)
					     || (atoi(argv[0]) == 14))) {
						printf("ERR: only allow channel 5-13 for secondary channel below\n");
						ret = UAP_FAILURE;
					}
				} else {
					if (argc == 2) {
						if ((atoi(argv[1]) &
						     BITMAP_CHANNEL_BELOW) &&
						    !is_valid_a_band_channel_below
						    (atoi(argv[0]))) {
							printf("ERR: For given primary channel secondary channel can not be set below\n");
							ret = UAP_FAILURE;
						}
						if ((atoi(argv[1]) &
						     BITMAP_CHANNEL_ABOVE) &&
						    !is_valid_a_band_channel_above
						    (atoi(argv[0]))) {
							printf("ERR: For given primary channel secondary channel can not be set above\n");
							ret = UAP_FAILURE;
						}
					}
				}
			}
		}
		break;
	case CHANNEL_EXT:
		if (argc > 3) {
			printf("ERR: Incorrect arguments for channel_ext.\n");
			ret = UAP_FAILURE;
		} else {
			if (argc == 3) {
				if ((ISDIGIT(argv[2]) == 0) ||
				    (atoi(argv[2]) & ~CHANNEL_MODE_MASK)) {
					printf("ERR: Invalid Mode\n");
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[2]) & BITMAP_ACS_MODE) &&
				    (atoi(argv[0]) != 0)) {
					printf("ERR: Channel must be 0 for ACS; MODE = 1.\n");
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[2]) & BITMAP_CHANNEL_ABOVE) &&
				    (atoi(argv[2]) & BITMAP_CHANNEL_BELOW)) {
					printf("ERR: secondary channel above and below both are enabled\n");
					ret = UAP_FAILURE;
				}
			}
			if ((argc == 2) &&
			    ((ISDIGIT(argv[1]) == 0) || (atoi(argv[1]) < 0) ||
			     atoi(argv[1]) > 1)) {
				printf("ERR:Invalid band\n");
				ret = UAP_FAILURE;
			}
			if ((argc == 1) ||
			    ((argc == 3) &&
			     !(atoi(argv[2]) & BITMAP_ACS_MODE))) {
				if ((ISDIGIT(argv[0]) == 0) ||
				    (atoi(argv[0]) < 1) ||
				    (atoi(argv[0]) > MAX_CHANNELS)) {
					printf("ERR: Channel must be in the range of 1 to %d\n", MAX_CHANNELS);
					ret = UAP_FAILURE;
				}
				if ((atoi(argv[0]) > MAX_CHANNELS_BG) &&
				    !(is_valid_a_band_channel(atoi(argv[0])))) {
					printf("ERR: Invalid Channel in 'a' band!\n");
					ret = UAP_FAILURE;
				}
				ch = atoi(argv[0]);
				if (ch <= MAX_CHANNELS_BG) {
					if ((argc == 3) &&
					    (atoi(argv[2]) &
					     BITMAP_CHANNEL_ABOVE) &&
					    (atoi(argv[0]) >
					     MAX_CHANNEL_ABOVE)) {
						printf("ERR: only allow channel 1-9 for secondary channel above\n");
						ret = UAP_FAILURE;
					}
					if ((argc == 3) &&
					    (atoi(argv[2]) &
					     BITMAP_CHANNEL_BELOW) &&
					    ((atoi(argv[0]) < MIN_CHANNEL_BELOW)
					     || (atoi(argv[0]) == 14))) {
						printf("ERR: only allow channel 5-13 for secondary channel below\n");
						ret = UAP_FAILURE;
					}
				} else {
					if (argc == 3) {
						if ((atoi(argv[2]) &
						     BITMAP_CHANNEL_BELOW) &&
						    !is_valid_a_band_channel_below
						    (atoi(argv[0]))) {
							printf("ERR: For given primary channel secondary channel can not be set below\n");
							ret = UAP_FAILURE;
						}
						if ((atoi(argv[2]) &
						     BITMAP_CHANNEL_ABOVE) &&
						    !is_valid_a_band_channel_above
						    (atoi(argv[0]))) {
							printf("ERR: For given primary channel secondary channel can not be set above\n");
							ret = UAP_FAILURE;
						}
					}
				}
			}
		}
		break;
	case BAND:
		if (argc > 1) {
			printf("ERR: Incorrect number of BAND arguments.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR: Invalid band.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case RATE:
		if (argc > MAX_RATES) {
			printf("ERR: Incorrect number of RATES arguments.\n");
			ret = UAP_FAILURE;
		} else {
			for (i = 0; i < argc; i++) {
				if ((IS_HEX_OR_DIGIT(argv[i]) == UAP_FAILURE) ||
				    (is_rate_valid
				     (A2HEXDECIMAL(argv[i]) &
				      ~BASIC_RATE_SET_BIT)
				     != UAP_SUCCESS)) {
					printf("ERR:Unsupported rate.\n");
					ret = UAP_FAILURE;
					break;
				}
			}
			if ((ret != UAP_FAILURE) &&
			    (has_dup_rate(argc, argv) != UAP_SUCCESS)) {
				printf("ERR: Duplicate rate values entered\n");
				ret = UAP_FAILURE;
			}
			if (check_mandatory_rates(argc, argv) != UAP_SUCCESS) {
				ret = UAP_FAILURE;
			}
		}
		break;
	case BROADCASTSSID:
		if (argc != 1) {
			printf("ERR:wrong BROADCASTSSID arguments.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    ((atoi(argv[0]) != 0) && (atoi(argv[0]) != 1) &&
			     (atoi(argv[0]) != 2))) {
				printf("ERR:Illegal parameter %s for BROADCASTSSID. Must be either '0', '1' or '2'.\n", argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case RTSTHRESH:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for RTSTHRESHOLD\n");
			ret = UAP_FAILURE;
		} else if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			   (atoi(argv[0]) > MAX_RTS_THRESHOLD)) {
			printf("ERR:Illegal RTSTHRESHOLD %s. The value must between 0 and %d\n", argv[0], MAX_RTS_THRESHOLD);
			ret = UAP_FAILURE;
		}
		break;
	case FRAGTHRESH:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for FRAGTHRESH\n");
			ret = UAP_FAILURE;
		} else if ((ISDIGIT(argv[0]) == 0) ||
			   (atoi(argv[0]) < MIN_FRAG_THRESHOLD) ||
			   (atoi(argv[0]) > MAX_FRAG_THRESHOLD)) {
			printf("ERR:Illegal FRAGTHRESH %s. The value must between %d and %d\n", argv[0], MIN_FRAG_THRESHOLD, MAX_FRAG_THRESHOLD);
			ret = UAP_FAILURE;
		}
		break;
	case DTIMPERIOD:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DTIMPERIOD\n");
			ret = UAP_FAILURE;
		} else if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 1) ||
			   (atoi(argv[0]) > MAX_DTIM_PERIOD)) {
			printf("ERR: DTIMPERIOD Value must be in range of 1 to %d\n", MAX_DTIM_PERIOD);
			ret = UAP_FAILURE;
		}
		break;
	case RSNREPLAYPROT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for RSNREPLAYPROT\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR:Illegal RSNREPLAYPROT parameter %s. Must be either '0' or '1'.\n", argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case RADIOCONTROL:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for RADIOCONTROL\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR:Illegal RADIOCONTROL parameter %s. Must be either '0' or '1'.\n", argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case TXRATECFG:
		if (argc > 3) {
			printf("ERR:Incorrect number of arguments for DATARATE\n");
			ret = UAP_FAILURE;
		} else {
			if (((argc >= 1) &&
			     (IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE)) ||
			    ((argc >= 2) &&
			     (IS_HEX_OR_DIGIT(argv[1]) == UAP_FAILURE))
			    || ((argc >= 3) &&
				(IS_HEX_OR_DIGIT(argv[2]) == UAP_FAILURE))
				) {
				printf("ERR: invalid Tx data rate\n");
				ret = UAP_FAILURE;
			} else if (argc >= 1) {
				if (A2HEXDECIMAL(argv[0]) == 0xFF) {
					if (argc != 1) {
						printf("ERR: invalid auto rate input\n");
						ret = UAP_FAILURE;
					}
				} else {
					if ((A2HEXDECIMAL(argv[0]) > 2)
						) {
						printf("ERR: invalid format\n");
						ret = UAP_FAILURE;
					}
					if (argc >= 2) {
						if (((A2HEXDECIMAL(argv[0]) ==
						      0) &&
						     (A2HEXDECIMAL(argv[1]) >
						      11)) ||
						    ((A2HEXDECIMAL(argv[0]) ==
						      1) &&
						     (A2HEXDECIMAL(argv[1]) !=
						      32) &&
						     (A2HEXDECIMAL(argv[1]) >
						      15)
						    )) {
							printf("ERR:Incorrect TxRate %s.\n", argv[1]);
							ret = UAP_FAILURE;
						}
					}
					if (argc == 3) {
						if ((A2HEXDECIMAL(argv[0]) != 2)
						    ||
						    ((A2HEXDECIMAL(argv[2]) < 1)
						     || (A2HEXDECIMAL(argv[2]) >
							 2))) {
							printf("ERR:Incorrect nss.\n");
							ret = UAP_FAILURE;
						}
					}
				}
			}
		}
		break;
	case MCBCDATARATE:
	case TXBEACONRATE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for DATARATE\n");
			ret = UAP_FAILURE;
		} else {
			if (IS_HEX_OR_DIGIT(argv[0]) == UAP_FAILURE) {
				printf("ERR: invalid data rate\n");
				ret = UAP_FAILURE;
			} else if ((A2HEXDECIMAL(argv[0]) != 0) &&
				   (is_rate_valid
				    (A2HEXDECIMAL(argv[0]) &
				     ~BASIC_RATE_SET_BIT) != UAP_SUCCESS)) {
				printf("ERR: invalid data rate\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case PKTFWD:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for PKTFWD.\n");
			ret = UAP_FAILURE;
		} else if ((ISDIGIT(argv[0]) == 0) ||
			   ((atoi(argv[0]) < 0) || (atoi(argv[0]) > 15))) {
			printf("ERR:Illegal PKTFWD parameter %s. Must be within '0' and '15'.\n", argv[0]);
			ret = UAP_FAILURE;
		}
		break;
	case STAAGEOUTTIMER:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for STAAGEOUTTIMER.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 0) &&
							((atoi(argv[0]) <
							  MIN_STAGE_OUT_TIME) ||
							 (atoi(argv[0]) >
							  MAX_STAGE_OUT_TIME))))
			{
				printf("ERR:Illegal STAAGEOUTTIMER %s. Must be between %d and %d.\n", argv[0], MIN_STAGE_OUT_TIME, MAX_STAGE_OUT_TIME);
				ret = UAP_FAILURE;
			}
		}
		break;
	case PSSTAAGEOUTTIMER:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for PSSTAAGEOUTTIMER.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 0) &&
							((atoi(argv[0]) <
							  MIN_STAGE_OUT_TIME) ||
							 (atoi(argv[0]) >
							  MAX_STAGE_OUT_TIME))))
			{
				printf("ERR:Illegal PSSTAAGEOUTTIMER %s. Must be between %d and %d.\n", argv[0], MIN_STAGE_OUT_TIME, MAX_STAGE_OUT_TIME);
				ret = UAP_FAILURE;
			}
		}
		break;
	case AUTHMODE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for AUTHMODE\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((atoi(argv[0]) != 0) &&
							(atoi(argv[0]) != 1)
							&& (atoi(argv[0]) !=
							    255))) {
				printf("ERR:Illegal AUTHMODE parameter %s. Must be either '0','1', or 255''.\n", argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case GROUPREKEYTIMER:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for GROUPREKEYTIMER.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > MAX_GRP_TIMER)) {
				printf("ERR: GROUPREKEYTIMER range is [0:%d] (0 for disable)\n", MAX_GRP_TIMER);
				ret = UAP_FAILURE;
			}
		}
		break;
	case MAXSTANUM:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for MAXSTANUM\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) <= 0)) {
				printf("ERR:Invalid STA_NUM argument %s.\n",
				       argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case BEACONPERIOD:
		if (argc != 1) {
			printf("ERR:Incorrect number of argument for BEACONPERIOD.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) < MIN_BEACON_PERIOD)
			    || (atoi(argv[0]) > MAX_BEACON_PERIOD)) {
				printf("ERR: BEACONPERIOD must be in range of %d to %d.\n", MIN_BEACON_PERIOD, MAX_BEACON_PERIOD);
				ret = UAP_FAILURE;
			}
		}
		break;
	case RETRYLIMIT:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for RETRY LIMIT\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_RETRY_LIMIT) ||
			    (atoi(argv[0]) < 0)) {
				printf("ERR:RETRY_LIMIT must be in the range of [0:%d]. The  input was %s.\n", MAX_RETRY_LIMIT, argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case STICKYTIMCONFIG:
		if ((argc != 1) && (argc != 3)) {
			printf("ERR:Incorrect number of arguments for STICKY_TIM_CONFIG\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 2)) {
				printf("ERR:Enable parameter must be 0, 1 or 2\n");
				ret = UAP_FAILURE;
				break;
			}
			if (((atoi(argv[0]) != 1) && (argc > 1))) {
				printf("ERR: Invalid arguments\n");
				ret = UAP_FAILURE;
				break;
			}
			if ((atoi(argv[0]) == 1) && (argc != 3)) {
				printf("ERR: Both duration and sticky bit mask must be provided for ENABLE = 1\n");
				ret = UAP_FAILURE;
				break;
			}
			if (argc > 1) {
				if ((ISDIGIT(argv[1]) == 0)) {
					printf("ERR: Invalid duration\n");
					ret = UAP_FAILURE;
					break;
				}
				if ((ISDIGIT(argv[2]) == 0) ||
				    (atoi(argv[2]) < 1) ||
				    (atoi(argv[2]) > 3)) {
					printf("ERR:Invalid sticky bit mask\n");
					ret = UAP_FAILURE;
					break;
				}
			}
		}
		break;
	case STICKYTIMSTAMACADDR:
		if ((argc != 1) && (argc != 2)) {
			printf("ERR:Incorrect number of STICKY_TIM_STA_MAC_ADDR arguments\n");
			ret = UAP_FAILURE;
		} else {
			if ((argc == 2) &&
			    ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			     (atoi(argv[0]) > 1))) {
				printf("ERR:Invalid control parameter\n");
				ret = UAP_FAILURE;
				break;
			}
		}
		break;
	case COEX2040CONFIG:
		if (argc != 1) {
			printf("ERR: Incorrect number of 2040 COEX CONFIG arguments\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR:Invalid enable parameter\n");
				ret = UAP_FAILURE;
				break;
			}
		}
		break;
	case EAPOL_PWK_HSK:
		if (argc != 2) {
			printf("ERR:Incorrect number of EAPOL_PWK_HSK arguments.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (ISDIGIT(argv[1]) == 0)
			    || (atoi(argv[0]) < 0) || (atoi(argv[1]) < 0)) {
				printf("ERR:Illegal parameters for EAPOL_PWK_HSK. Must be digits greater than equal to zero.\n");
			}
		}
		break;
	case EAPOL_GWK_HSK:
		if (argc != 2) {
			printf("ERR:Incorrect number of EAPOL_GWK_HSK arguments.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (ISDIGIT(argv[1]) == 0)
			    || (atoi(argv[0]) < 0) || (atoi(argv[1]) < 0)) {
				printf("ERR:Illegal parameters for EAPOL_GWK_HSK. Must be digits greater than equal to zero.\n");
			}
		}
		break;
	case PREAMBLETYPE:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for PREAMBLE TYPE\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) > MAX_PREAMBLE_TYPE) ||
			    (atoi(argv[0]) < 0)) {
				printf("ERR:PREAMBLE TYPE must be in the range of [0:%d]. The  input was %s.\n", MAX_PREAMBLE_TYPE, argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;

	case COEX_COMM_BITMAP:
		if (argc != 1) {
			printf("ERR:Incorrect number of argument for Bitmap.\n");
			ret = UAP_FAILURE;
		} else {
			/* Only bit 0 is supported now, hence check for 1 or 0 */
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) < 0) || (atoi(argv[0]) > 1)) {
				printf("ERR: Bitmap must have value of 1 or 0.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case COEX_COMM_AP_COEX:
		if (argc != 1) {
			printf("ERR:Incorrect number of argument for APBTCoex.\n");
			ret = UAP_FAILURE;
		} else {
			if ((IS_HEX_OR_DIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) < 0) || (atoi(argv[0]) > 1)) {
				printf("ERR: APBTCoex must have value of 1 or 0.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case COEX_SCO_ACL_FREQ:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for aclFrequency.\n");
			ret = UAP_FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR: Incorrect value for aclFrequency.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case COEX_ACL_ENABLED:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for (acl) enabled.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || (atoi(argv[0]) < 0) ||
			    (atoi(argv[0]) > 1)) {
				printf("ERR: (acl) enabled must have value of 1 or 0.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case COEX_ACL_BT_TIME:
	case COEX_ACL_WLAN_TIME:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for bt/wlan time.\n");
			ret = UAP_FAILURE;
		} else {
			if (ISDIGIT(argv[0]) == 0) {
				printf("ERR: Incorrect value for bt/wlan time.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case COEX_PROTECTION:
		if (argc != 2) {
			printf("ERR:Incorrect number of arguments for %s.\n",
			       argv[0]);
			ret = UAP_FAILURE;
		} else {
			if (ISDIGIT(argv[1]) == 0) {
				printf("ERR: Incorrect value for %s.\n",
				       argv[0]);
				ret = UAP_FAILURE;
			}
		}
		break;
	case PWK_CIPHER:
		if ((argc != 1) && (argc != 2)) {
			printf("ERR:Incorrect number of arguments for pwk_cipher.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) & ~PROTOCOL_BITMAP)) {
				printf("Invalid Protocol paramter.\n");
				ret = UAP_FAILURE;
			}
			if (argc == 2) {
				if ((ISDIGIT(argv[1]) == 0) ||
				    (atoi(argv[1]) & ~CIPHER_BITMAP)) {
					printf("Invalid pairwise cipher.\n");
					ret = UAP_FAILURE;
				}
			}
		}
		break;
	case GWK_CIPHER:
		if (argc != 1) {
			printf("ERR:Incorrect number of arguments for gwk_cipher.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) ||
			    (atoi(argv[0]) & ~CIPHER_BITMAP) ||
			    (atoi(argv[0]) == AES_CCMP_TKIP)) {
				printf("Invalid group cipher.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	case RESTRICT_CLIENT_MODE:
		if ((argc != 1) && (argc != 2)) {
			printf("ERR: Incorrect number of arguments.\n");
			ret = UAP_FAILURE;
		} else {
			if ((ISDIGIT(argv[0]) == 0) || ((argc == 2) &&
							((IS_HEX_OR_DIGIT
							  (argv[1]) == 0) ||
							 ((atoi(argv[0]) < 0) ||
							  (atoi(argv[0]) >
							   1))))) {
				printf("ERR: Invalid arguments\n");
				ret = UAP_FAILURE;
			}
			if ((atoi(argv[0]) == 1) && (argc == 1)) {
				printf("ERR: Mode_config parameter must be provided to enable this feature.\n");
				ret = UAP_FAILURE;
			}
			if ((argc == 2) &&
			    (((A2HEXDECIMAL(argv[1]) << 8) != B_ONLY_MASK) &&
			     ((A2HEXDECIMAL(argv[1]) << 8) != G_ONLY_MASK) &&
			     ((A2HEXDECIMAL(argv[1]) << 8) != A_ONLY_MASK) &&
			     ((A2HEXDECIMAL(argv[1]) << 8) != N_ONLY_MASK) &&
			     ((A2HEXDECIMAL(argv[1]) << 8) != AC_ONLY_MASK))) {
				printf("ERR: Exactly one mode can be enabled at a time.\n");
				ret = UAP_FAILURE;
			}
		}
		break;
	default:
		ret = UAP_FAILURE;
		break;
	}
	return ret;
}

/**
 *  @brief Converts colon separated MAC address to hex value
 *
 *  @param mac      A pointer to the colon separated MAC string
 *  @param raw      A pointer to the hex data buffer
 *  @return         UAP_SUCCESS or UAP_FAILURE
 *                  UAP_RET_MAC_BROADCAST  - if broadcast mac
 *                  UAP_RET_MAC_MULTICAST - if multicast mac
 */
int
mac2raw(char *mac, t_u8 *raw)
{
	unsigned int temp_raw[ETH_ALEN];
	int num_tokens = 0;
	int i;
	if (strlen(mac) != ((2 * ETH_ALEN) + (ETH_ALEN - 1))) {
		return UAP_FAILURE;
	}
	num_tokens = sscanf(mac, "%2x:%2x:%2x:%2x:%2x:%2x",
			    temp_raw + 0, temp_raw + 1, temp_raw + 2,
			    temp_raw + 3, temp_raw + 4, temp_raw + 5);
	if (num_tokens != ETH_ALEN) {
		return UAP_FAILURE;
	}
	for (i = 0; i < num_tokens; i++)
		raw[i] = (t_u8)temp_raw[i];

	if (memcmp(raw, "\xff\xff\xff\xff\xff\xff", ETH_ALEN) == 0) {
		return UAP_RET_MAC_BROADCAST;
	} else if (raw[0] & 0x01) {
		return UAP_RET_MAC_MULTICAST;
	}
	return UAP_SUCCESS;
}

/**
 *  @brief Converts a string to hex value
 *
 *  @param str      A pointer to the string
 *  @param raw      A pointer to the raw data buffer
 *  @return         Number of bytes read
 */
int
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
 *  @brief Prints a MAC address in colon separated form from hex data
 *
 *  @param raw      A pointer to the hex data buffer
 *  @return         N/A
 */
void
print_mac(t_u8 *raw)
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", (unsigned int)raw[0],
	       (unsigned int)raw[1], (unsigned int)raw[2], (unsigned int)raw[3],
	       (unsigned int)raw[4], (unsigned int)raw[5]);
	return;
}

/**
 *  @brief 		Check hex string
 *
 *  @param hex		A pointer to hex string
 *  @return      	UAP_SUCCESS or UAP_FAILURE
 */
int
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
			return UAP_FAILURE;
		p++;
	}
	return UAP_SUCCESS;
}

/**
 *  @brief Show auth tlv
 *
 *  @param tlv     Pointer to auth tlv
 *
 *  @return         N/A
 */
void
print_auth(tlvbuf_auth_mode *tlv)
{
	switch (tlv->auth_mode) {
	case 0:
		printf("AUTHMODE = Open authentication\n");
		break;
	case 1:
		printf("AUTHMODE = Shared key authentication\n");
		break;
	case 255:
		printf("AUTHMODE = Auto (open and shared key)\n");
		break;
	default:
		printf("ERR: Invalid authmode=%d\n", tlv->auth_mode);
		break;
	}
}

/**
 *
 *  @brief Show cipher tlv
 *
 *  @param tlv     Pointer to cipher tlv
 *
 *  @return         N/A
 */
void
print_cipher(tlvbuf_cipher *tlv)
{
	switch (tlv->pairwise_cipher) {
	case CIPHER_TKIP:
		printf("PairwiseCipher = TKIP\n");
		break;
	case CIPHER_AES_CCMP:
		printf("PairwiseCipher = AES CCMP\n");
		break;
	case CIPHER_TKIP | CIPHER_AES_CCMP:
		printf("PairwiseCipher = TKIP + AES CCMP\n");
		break;
	case CIPHER_NONE:
		printf("PairwiseCipher =  None\n");
		break;
	default:
		printf("Unknown Pairwise cipher 0x%x\n", tlv->pairwise_cipher);
		break;
	}
	switch (tlv->group_cipher) {
	case CIPHER_TKIP:
		printf("GroupCipher = TKIP\n");
		break;
	case CIPHER_AES_CCMP:
		printf("GroupCipher = AES CCMP\n");
		break;
	case CIPHER_NONE:
		printf("GroupCipher = None\n");
		break;
	default:
		printf("Unknown Group cipher 0x%x\n", tlv->group_cipher);
		break;
	}
}

/**
 *  @brief Show pairwise cipher tlv
 *
 *  @param tlv     Pointer to pairwise cipher tlv
 *
 *  @return         N/A
 */
void
print_pwk_cipher(tlvbuf_pwk_cipher *tlv)
{
	switch (tlv->protocol) {
	case PROTOCOL_WPA:
		printf("Protocol WPA  : ");
		break;
	case PROTOCOL_WPA2:
		printf("Protocol WPA2 : ");
		break;
	default:
		printf("Unknown Protocol 0x%x\n", tlv->protocol);
		break;
	}

	switch (tlv->pairwise_cipher) {
	case CIPHER_TKIP:
		printf("PairwiseCipher = TKIP\n");
		break;
	case CIPHER_AES_CCMP:
		printf("PairwiseCipher = AES CCMP\n");
		break;
	case CIPHER_TKIP | CIPHER_AES_CCMP:
		printf("PairwiseCipher = TKIP + AES CCMP\n");
		break;
	case CIPHER_NONE:
		printf("PairwiseCipher =  None\n");
		break;
	default:
		printf("Unknown Pairwise cipher 0x%x\n", tlv->pairwise_cipher);
		break;
	}
}

/**
 *  @brief Show group cipher tlv
 *
 *  @param tlv     Pointer to group cipher tlv
 *
 *  @return         N/A
 */
void
print_gwk_cipher(tlvbuf_gwk_cipher *tlv)
{
	switch (tlv->group_cipher) {
	case CIPHER_TKIP:
		printf("GroupCipher = TKIP\n");
		break;
	case CIPHER_AES_CCMP:
		printf("GroupCipher = AES CCMP\n");
		break;
	case CIPHER_NONE:
		printf("GroupCipher = None\n");
		break;
	default:
		printf("Unknown Group cipher 0x%x\n", tlv->group_cipher);
		break;
	}
}

/**
 *  @brief Show mac filter tlv
 *
 *  @param tlv     Pointer to filter tlv
 *
 *  @return         N/A
 */
void
print_mac_filter(tlvbuf_sta_mac_addr_filter *tlv)
{
	int i;
	switch (tlv->filter_mode) {
	case 0:
		printf("Filter Mode = Filter table is disabled\n");
		return;
	case 1:
		if (!tlv->count) {
			printf("No mac address is allowed to connect\n");
		} else {
			printf("Filter Mode = Allow mac address specified in the allowed list\n");
		}
		break;
	case 2:
		if (!tlv->count) {
			printf("No mac address is blocked\n");
		} else {
			printf("Filter Mode = Block MAC addresses specified in the banned list\n");
		}
		break;
	}
	for (i = 0; i < tlv->count; i++) {
		printf("MAC_%d = ", i);
		print_mac(&tlv->mac_address[i * ETH_ALEN]);
		printf("\n");
	}
}

/**
 *  @brief Show rate tlv
 *
 *  @param tlv      Pointer to rate tlv
 *
 *  @return         N/A
 */
void
print_rate(tlvbuf_rates *tlv)
{
	int flag = 0;
	int i;
	t_u16 tlv_len;

	tlv_len = *(t_u8 *)&tlv->length;
	tlv_len |= (*((t_u8 *)&tlv->length + 1) << 8);

	printf("Basic Rates =");
	for (i = 0; i < tlv_len; i++) {
		if (tlv->operational_rates[i] > (BASIC_RATE_SET_BIT - 1)) {
			flag = flag ? : 1;
			printf(" 0x%x", tlv->operational_rates[i]);
		}
	}
	printf("%s\nNon-Basic Rates =", flag ? "" : " ( none ) ");
	for (flag = 0, i = 0; i < tlv_len; i++) {
		if (tlv->operational_rates[i] < BASIC_RATE_SET_BIT) {
			flag = flag ? : 1;
			printf(" 0x%x", tlv->operational_rates[i]);
		}
	}
	printf("%s\n", flag ? "" : " ( none ) ");
}

/**
 *  @brief Show all the tlv in the buf
 *
 *  @param buf     Pointer to tlv buffer
 *  @param len     Tlv buffer len
 *
 *  @return         N/A
 */
void
print_tlv(t_u8 *buf, t_u16 len)
{
	tlvbuf_header *pcurrent_tlv = (tlvbuf_header *)buf;
	int tlv_buf_left = len;
	t_u16 tlv_type;
	t_u16 tlv_len;
	t_u16 tlv_val_16;
	t_u32 tlv_val_32;
	t_u8 ssid[33];
	int i = 0;
	tlvbuf_ap_mac_address *mac_tlv;
	tlvbuf_ssid *ssid_tlv;
	tlvbuf_beacon_period *beacon_tlv;
	tlvbuf_dtim_period *dtim_tlv;
	tlvbuf_rates *rates_tlv;
	tlvbuf_tx_power *txpower_tlv;
	tlvbuf_bcast_ssid_ctl *bcast_tlv;
	tlvbuf_preamble_ctl *preamble_tlv;
	tlvbuf_bss_status *bss_status_tlv;
	tlvbuf_rts_threshold *rts_tlv;
	tlvbuf_mcbc_data_rate *mcbcrate_tlv;
	tlvbuf_pkt_fwd_ctl *pkt_fwd_tlv;
	tlvbuf_sta_ageout_timer *ageout_tlv;
	tlvbuf_ps_sta_ageout_timer *ps_ageout_tlv;
	tlvbuf_auth_mode *auth_tlv;
	tlvbuf_protocol *proto_tlv;
	tlvbuf_akmp *akmp_tlv;
	tlvbuf_cipher *cipher_tlv;
	tlvbuf_pwk_cipher *pwk_cipher_tlv;
	tlvbuf_gwk_cipher *gwk_cipher_tlv;
	tlvbuf_group_rekey_timer *rekey_tlv;
	tlvbuf_wpa_passphrase *psk_tlv;
	tlvbuf_coex_common_cfg *coex_common_tlv;
	tlvbuf_coex_sco_cfg *coex_sco_tlv;
	tlvbuf_coex_acl_cfg *coex_acl_tlv;
	tlvbuf_coex_stats *coex_stats_tlv;
	tlvbuf_wep_key *wep_tlv;
	tlvbuf_frag_threshold *frag_tlv;
	tlvbuf_sta_mac_addr_filter *filter_tlv;
	tlvbuf_max_sta_num *max_sta_tlv;
	tlvbuf_retry_limit *retry_limit_tlv;
	tlvbuf_eapol_pwk_hsk_timeout *pwk_timeout_tlv;
	tlvbuf_eapol_pwk_hsk_retries *pwk_retries_tlv;
	tlvbuf_eapol_gwk_hsk_timeout *gwk_timeout_tlv;
	tlvbuf_eapol_gwk_hsk_retries *gwk_retries_tlv;
	tlvbuf_channel_config *channel_tlv;
	tlvbuf_channel_list *chnlist_tlv;
	channel_list *pchan_list;
	t_u16 custom_ie_len;
	tlvbuf_rsn_replay_prot *replay_prot_tlv;
	tlvbuf_custom_ie *custom_ie_tlv;
	custom_ie *custom_ie_ptr;
	tlvbuf_max_mgmt_ie *max_mgmt_ie_tlv;
	tlvbuf_wmm_para_t *wmm_para_tlv;
	int flag = 0;
	tlvbuf_htcap_t *ht_cap_tlv;
	tlvbuf_htinfo_t *ht_info_tlv;
	tlvbuf_2040_coex *coex_2040_tlv;

#ifdef RX_PACKET_COALESCE
	tlvbuf_rx_pkt_coal_t *rx_pkt_coal_tlv;
#endif

#if DEBUG
	uap_printf(MSG_DEBUG, "tlv total len=%d\n", len);
#endif
	while (tlv_buf_left >= (int)sizeof(tlvbuf_header)) {
		tlv_type = *(t_u8 *)&pcurrent_tlv->type;
		tlv_type |= (*((t_u8 *)&pcurrent_tlv->type + 1) << 8);
		tlv_len = *(t_u8 *)&pcurrent_tlv->len;
		tlv_len |= (*((t_u8 *)&pcurrent_tlv->len + 1) << 8);
		if ((sizeof(tlvbuf_header) + tlv_len) >
		    (unsigned int)tlv_buf_left) {
			printf("wrong tlv: tlv_len=%d, tlv_buf_left=%d\n",
			       tlv_len, tlv_buf_left);
			break;
		}
		switch (tlv_type) {
		case MRVL_AP_MAC_ADDRESS_TLV_ID:
			mac_tlv = (tlvbuf_ap_mac_address *)pcurrent_tlv;
			printf("AP MAC address = ");
			print_mac(mac_tlv->ap_mac_addr);
			printf("\n");
			break;
		case MRVL_SSID_TLV_ID:
			memset(ssid, 0, sizeof(ssid));
			ssid_tlv = (tlvbuf_ssid *)pcurrent_tlv;
			strncpy((char *)ssid, (char *)ssid_tlv->ssid,
				sizeof(ssid) - 1);
			printf("SSID = %s\n", ssid);
			break;
		case MRVL_BEACON_PERIOD_TLV_ID:
			beacon_tlv = (tlvbuf_beacon_period *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&beacon_tlv->beacon_period_ms;
			tlv_val_16 |=
				(*((t_u8 *)&beacon_tlv->beacon_period_ms + 1) <<
				 8);
			printf("Beacon period = %d\n", tlv_val_16);
			break;
		case MRVL_DTIM_PERIOD_TLV_ID:
			dtim_tlv = (tlvbuf_dtim_period *)pcurrent_tlv;
			printf("DTIM period = %d\n", dtim_tlv->dtim_period);
			break;
		case MRVL_CHANNELCONFIG_TLV_ID:
			channel_tlv = (tlvbuf_channel_config *)pcurrent_tlv;
			printf("Channel = %d\n", channel_tlv->chan_number);
			printf("Band = %s\n",
			       (channel_tlv->bandcfg.chanBand ==
				BAND_5GHZ) ? "5GHz" : "2.4GHz");
			printf("Channel Select Mode = %s\n",
			       (channel_tlv->bandcfg.scanMode ==
				SCAN_MODE_ACS) ? "ACS" : "Manual");
			if (channel_tlv->bandcfg.chan2Offset == SEC_CHAN_NONE)
				printf("no secondary channel\n");
			else if (channel_tlv->bandcfg.chan2Offset ==
				 SEC_CHAN_ABOVE)
				printf("secondary channel is above primary channel\n");
			else if (channel_tlv->bandcfg.chan2Offset ==
				 SEC_CHAN_BELOW)
				printf("secondary channel is below primary channel\n");
			break;
		case MRVL_CHANNELLIST_TLV_ID:
			chnlist_tlv = (tlvbuf_channel_list *)pcurrent_tlv;
			printf("Channels List = ");
			pchan_list =
				(channel_list *) & (chnlist_tlv->chan_list);
			if (tlv_len % sizeof(channel_list)) {
				break;
			}
			for (i = 0;
			     (unsigned int)i < (tlv_len / sizeof(channel_list));
			     i++) {
				printf("\n%d\t%sGHz", pchan_list->chan_number,
				       (pchan_list->bandcfg.chanBand ==
					BAND_5GHZ) ? "5" : "2.4");
				pchan_list++;
			}
			printf("\n");
			break;
		case MRVL_RSN_REPLAY_PROT_TLV_ID:
			replay_prot_tlv =
				(tlvbuf_rsn_replay_prot *)pcurrent_tlv;
			printf("RSN replay protection = %s\n",
			       replay_prot_tlv->
			       rsn_replay_prot ? "enabled" : "disabled");
			break;
		case MRVL_RATES_TLV_ID:
			rates_tlv = (tlvbuf_rates *)pcurrent_tlv;
			print_rate(rates_tlv);
			break;
		case MRVL_TX_POWER_TLV_ID:
			txpower_tlv = (tlvbuf_tx_power *)pcurrent_tlv;
			printf("Tx power = %d dBm\n",
			       txpower_tlv->tx_power_dbm);
			break;
		case MRVL_BCAST_SSID_CTL_TLV_ID:
			bcast_tlv = (tlvbuf_bcast_ssid_ctl *)pcurrent_tlv;
			printf("SSID broadcast = %s\n",
			       (bcast_tlv->bcast_ssid_ctl ==
				1) ? "enabled" : "disabled");
			break;
		case MRVL_PREAMBLE_CTL_TLV_ID:
			preamble_tlv = (tlvbuf_preamble_ctl *)pcurrent_tlv;
			printf("Preamble type = %s\n",
			       (preamble_tlv->preamble_type ==
				0) ? "auto" : ((preamble_tlv->preamble_type ==
						1) ? "short" : "long"));
			break;
		case MRVL_BSS_STATUS_TLV_ID:
			bss_status_tlv = (tlvbuf_bss_status *)pcurrent_tlv;
			printf("BSS status = %s\n",
			       (bss_status_tlv->bss_status ==
				0) ? "stopped" : "started");
			break;
		case MRVL_RTS_THRESHOLD_TLV_ID:
			rts_tlv = (tlvbuf_rts_threshold *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&rts_tlv->rts_threshold;
			tlv_val_16 |=
				(*((t_u8 *)&rts_tlv->rts_threshold + 1) << 8);
			printf("RTS threshold = %d\n", tlv_val_16);
			break;
		case MRVL_FRAG_THRESHOLD_TLV_ID:
			frag_tlv = (tlvbuf_frag_threshold *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&frag_tlv->frag_threshold;
			tlv_val_16 |=
				(*((t_u8 *)&frag_tlv->frag_threshold + 1) << 8);
			printf("Fragmentation threshold = %d\n", tlv_val_16);
			break;
		case MRVL_MCBC_DATA_RATE_TLV_ID:
			mcbcrate_tlv = (tlvbuf_mcbc_data_rate *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&mcbcrate_tlv->mcbc_datarate;
			tlv_val_16 |=
				(*((t_u8 *)&mcbcrate_tlv->mcbc_datarate + 1) <<
				 8);
			if (mcbcrate_tlv->mcbc_datarate == 0)
				printf("MCBC data rate = auto\n");
			else
				printf("MCBC data rate = 0x%x\n", tlv_val_16);
			break;
		case MRVL_PKT_FWD_CTL_TLV_ID:
			pkt_fwd_tlv = (tlvbuf_pkt_fwd_ctl *)pcurrent_tlv;
			printf("%s handles packet forwarding -\n",
			       ((pkt_fwd_tlv->pkt_fwd_ctl & PKT_FWD_FW_BIT) ==
				0) ? "Host" : "Firmware");
			printf("\tIntra-BSS broadcast packets are %s\n",
			       ((pkt_fwd_tlv->
				 pkt_fwd_ctl & PKT_FWD_INTRA_BCAST) ==
				0) ? "allowed" : "denied");
			printf("\tIntra-BSS unicast packets are %s\n",
			       ((pkt_fwd_tlv->
				 pkt_fwd_ctl & PKT_FWD_INTRA_UCAST) ==
				0) ? "allowed" : "denied");
			printf("\tInter-BSS unicast packets are %s\n",
			       ((pkt_fwd_tlv->
				 pkt_fwd_ctl & PKT_FWD_INTER_UCAST) ==
				0) ? "allowed" : "denied");
			break;
		case MRVL_STA_AGEOUT_TIMER_TLV_ID:
			ageout_tlv = (tlvbuf_sta_ageout_timer *)pcurrent_tlv;
			tlv_val_32 = *(t_u8 *)&ageout_tlv->sta_ageout_timer_ms;
			tlv_val_32 |=
				(*((t_u8 *)&ageout_tlv->sta_ageout_timer_ms + 1)
				 << 8);
			tlv_val_32 |=
				(*((t_u8 *)&ageout_tlv->sta_ageout_timer_ms + 2)
				 << 16);
			tlv_val_32 |=
				(*((t_u8 *)&ageout_tlv->sta_ageout_timer_ms + 3)
				 << 24);
			printf("STA ageout timer = %d\n", (int)tlv_val_32);
			break;
		case MRVL_PS_STA_AGEOUT_TIMER_TLV_ID:
			ps_ageout_tlv =
				(tlvbuf_ps_sta_ageout_timer *)pcurrent_tlv;
			tlv_val_32 =
				*(t_u8 *)&ps_ageout_tlv->ps_sta_ageout_timer_ms;
			tlv_val_32 |=
				(*
				 ((t_u8 *)&ps_ageout_tlv->
				  ps_sta_ageout_timer_ms + 1) << 8);
			tlv_val_32 |=
				(*
				 ((t_u8 *)&ps_ageout_tlv->
				  ps_sta_ageout_timer_ms + 2) << 16);
			tlv_val_32 |=
				(*
				 ((t_u8 *)&ps_ageout_tlv->
				  ps_sta_ageout_timer_ms + 3) << 24);
			printf("PS STA ageout timer = %d\n", (int)tlv_val_32);
			break;
		case MRVL_AUTH_TLV_ID:
			auth_tlv = (tlvbuf_auth_mode *)pcurrent_tlv;
			print_auth(auth_tlv);
			break;
		case MRVL_PROTOCOL_TLV_ID:
			proto_tlv = (tlvbuf_protocol *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&proto_tlv->protocol;
			tlv_val_16 |=
				(*((t_u8 *)&proto_tlv->protocol + 1) << 8);
			print_protocol(tlv_val_16);
			break;
		case MRVL_AKMP_TLV_ID:
			akmp_tlv = (tlvbuf_akmp *)pcurrent_tlv;
			tlv_val_16 = *(t_u8 *)&akmp_tlv->key_mgmt;
			tlv_val_16 |= (*((t_u8 *)&akmp_tlv->key_mgmt + 1) << 8);
			if (tlv_val_16 & (KEY_MGMT_PSK | KEY_MGMT_PSK_SHA256)) {
				if (tlv_val_16 & KEY_MGMT_PSK)
					printf("KeyMgmt = PSK\n");
				if (tlv_val_16 & KEY_MGMT_PSK_SHA256)
					printf("KeyMgmt = PSK_SHA256\n");
				tlv_val_16 =
					*(t_u8 *)&akmp_tlv->key_mgmt_operation;
				if (tlv_len > sizeof(t_u16)) {
					tlv_val_16 |=
						(*
						 ((t_u8 *)&akmp_tlv->
						  key_mgmt_operation + 1) << 8);
					printf("Key Exchange on : %s.\n",
					       (tlv_val_16 & 0x01) ? "Host" :
					       "Device");
					printf("1x Authentication on : %s.\n",
					       (tlv_val_16 & 0x10) ? "Host" :
					       "Device");
				}
			}
			if (tlv_val_16 & KEY_MGMT_EAP)
				printf("KeyMgmt = EAP");
			if (tlv_val_16 & KEY_MGMT_NONE)
				printf("KeyMgmt = NONE");
			break;
		case MRVL_CIPHER_TLV_ID:
			cipher_tlv = (tlvbuf_cipher *)pcurrent_tlv;
			print_cipher(cipher_tlv);
			break;
		case MRVL_CIPHER_PWK_TLV_ID:
			pwk_cipher_tlv = (tlvbuf_pwk_cipher *)pcurrent_tlv;
			pwk_cipher_tlv->protocol =
				uap_le16_to_cpu(pwk_cipher_tlv->protocol);
			print_pwk_cipher(pwk_cipher_tlv);
			break;
		case MRVL_CIPHER_GWK_TLV_ID:
			gwk_cipher_tlv = (tlvbuf_gwk_cipher *)pcurrent_tlv;
			print_gwk_cipher(gwk_cipher_tlv);
			break;
		case MRVL_GRP_REKEY_TIME_TLV_ID:
			rekey_tlv = (tlvbuf_group_rekey_timer *)pcurrent_tlv;
			tlv_val_32 = *(t_u8 *)&rekey_tlv->group_rekey_time_sec;
			tlv_val_32 |=
				(*((t_u8 *)&rekey_tlv->group_rekey_time_sec + 1)
				 << 8);
			tlv_val_32 |=
				(*((t_u8 *)&rekey_tlv->group_rekey_time_sec + 2)
				 << 16);
			tlv_val_32 |=
				(*((t_u8 *)&rekey_tlv->group_rekey_time_sec + 3)
				 << 24);
			if (tlv_val_32 == 0)
				printf("Group re-key time = disabled\n");
			else
				printf("Group re-key time = %d second\n",
				       tlv_val_32);
			break;
		case MRVL_WPA_PASSPHRASE_TLV_ID:
			psk_tlv = (tlvbuf_wpa_passphrase *)pcurrent_tlv;
			if (tlv_len > 0) {
				printf("WPA passphrase = ");
				for (i = 0; i < tlv_len; i++)
					printf("%c", psk_tlv->passphrase[i]);
				printf("\n");
			} else
				printf("WPA passphrase = None\n");
			break;
		case MRVL_WEP_KEY_TLV_ID:
			wep_tlv = (tlvbuf_wep_key *)pcurrent_tlv;
			print_wep_key(wep_tlv);
			break;
		case MRVL_STA_MAC_ADDR_FILTER_TLV_ID:
			filter_tlv = (tlvbuf_sta_mac_addr_filter *)pcurrent_tlv;
			print_mac_filter(filter_tlv);
			break;
		case MRVL_MAX_STA_CNT_TLV_ID:
			max_sta_tlv = (tlvbuf_max_sta_num *)pcurrent_tlv;
			tlv_val_16 =
				*(t_u8 *)&max_sta_tlv->max_sta_num_configured;
			tlv_val_16 |=
				(*
				 ((t_u8 *)&max_sta_tlv->max_sta_num_configured +
				  1) << 8);
			printf("Max Station Number configured = %d\n",
			       tlv_val_16);
			if (max_sta_tlv->length == 4) {
				tlv_val_16 =
					*(t_u8 *)&max_sta_tlv->
					max_sta_num_supported;
				tlv_val_16 |=
					(*
					 ((t_u8 *)&max_sta_tlv->
					  max_sta_num_supported + 1) << 8);
				printf("Max Station Number supported = %d\n",
				       tlv_val_16);
			}
			break;
		case MRVL_RETRY_LIMIT_TLV_ID:
			retry_limit_tlv = (tlvbuf_retry_limit *)pcurrent_tlv;
			printf("Retry Limit = %d\n",
			       retry_limit_tlv->retry_limit);
			break;
		case MRVL_EAPOL_PWK_HSK_TIMEOUT_TLV_ID:
			pwk_timeout_tlv =
				(tlvbuf_eapol_pwk_hsk_timeout *)pcurrent_tlv;
			pwk_timeout_tlv->pairwise_update_timeout =
				uap_le32_to_cpu(pwk_timeout_tlv->
						pairwise_update_timeout);
			printf("Pairwise handshake timeout = %d\n",
			       pwk_timeout_tlv->pairwise_update_timeout);
			break;
		case MRVL_EAPOL_PWK_HSK_RETRIES_TLV_ID:
			pwk_retries_tlv =
				(tlvbuf_eapol_pwk_hsk_retries *)pcurrent_tlv;
			pwk_retries_tlv->pwk_retries =
				uap_le32_to_cpu(pwk_retries_tlv->pwk_retries);
			printf("Pairwise handshake retries = %d\n",
			       pwk_retries_tlv->pwk_retries);
			break;
		case MRVL_EAPOL_GWK_HSK_TIMEOUT_TLV_ID:
			gwk_timeout_tlv =
				(tlvbuf_eapol_gwk_hsk_timeout *)pcurrent_tlv;
			gwk_timeout_tlv->groupwise_update_timeout =
				uap_le32_to_cpu(gwk_timeout_tlv->
						groupwise_update_timeout);
			printf("Groupwise handshake timeout = %d\n",
			       gwk_timeout_tlv->groupwise_update_timeout);
			break;
		case MRVL_EAPOL_GWK_HSK_RETRIES_TLV_ID:
			gwk_retries_tlv =
				(tlvbuf_eapol_gwk_hsk_retries *)pcurrent_tlv;
			gwk_retries_tlv->gwk_retries =
				uap_le32_to_cpu(gwk_retries_tlv->gwk_retries);
			printf("Groupwise handshake retries = %d\n",
			       gwk_retries_tlv->gwk_retries);
			break;
		case MRVL_MGMT_IE_LIST_TLV_ID:
			custom_ie_tlv = (tlvbuf_custom_ie *)pcurrent_tlv;
			custom_ie_len = tlv_len;
			custom_ie_ptr = (custom_ie *)(custom_ie_tlv->ie_data);
			while (custom_ie_len >= sizeof(custom_ie)) {
				custom_ie_ptr->ie_index =
					uap_le16_to_cpu(custom_ie_ptr->
							ie_index);
				custom_ie_ptr->ie_length =
					uap_le16_to_cpu(custom_ie_ptr->
							ie_length);
				custom_ie_ptr->mgmt_subtype_mask =
					uap_le16_to_cpu(custom_ie_ptr->
							mgmt_subtype_mask);
				printf("Index [%d]\n", custom_ie_ptr->ie_index);
				if (custom_ie_ptr->ie_length)
					printf("Management Subtype Mask = 0x%02x\n", custom_ie_ptr->mgmt_subtype_mask == 0 ? UAP_CUSTOM_IE_AUTO_MASK : custom_ie_ptr->mgmt_subtype_mask);
				else
					printf("Management Subtype Mask = 0x%02x\n", custom_ie_ptr->mgmt_subtype_mask);
				hexdump_data("IE Buffer",
					     (void *)custom_ie_ptr->ie_buffer,
					     (custom_ie_ptr->ie_length), ' ');
				custom_ie_len -=
					sizeof(custom_ie) +
					custom_ie_ptr->ie_length;
				custom_ie_ptr =
					(custom_ie *)((t_u8 *)custom_ie_ptr +
						      sizeof(custom_ie) +
						      custom_ie_ptr->ie_length);
			}
			custom_ie_present = 1;
			break;
		case MRVL_MAX_MGMT_IE_TLV_ID:
			max_mgmt_ie_tlv = (tlvbuf_max_mgmt_ie *)pcurrent_tlv;
			max_mgmt_ie_tlv->count =
				uap_le16_to_cpu(max_mgmt_ie_tlv->count);
			for (i = 0; i < max_mgmt_ie_tlv->count; i++) {
				max_mgmt_ie_tlv->info[i].buf_size =
					uap_le16_to_cpu(max_mgmt_ie_tlv->
							info[i].buf_size);
				max_mgmt_ie_tlv->info[i].buf_count =
					uap_le16_to_cpu(max_mgmt_ie_tlv->
							info[i].buf_count);
				printf("buf%d_size = %d\n", i,
				       max_mgmt_ie_tlv->info[i].buf_size);
				printf("number of buffers = %d\n",
				       max_mgmt_ie_tlv->info[i].buf_count);
				printf("\n");
			}
			max_mgmt_ie_print = 1;
			break;
		case MRVL_BT_COEX_COMMON_CFG_TLV_ID:
			printf("Coex common configuration:\n");
			coex_common_tlv =
				(tlvbuf_coex_common_cfg *)pcurrent_tlv;
			printf("\tConfig Bitmap = 0x%02x\n",
			       uap_le32_to_cpu(coex_common_tlv->config_bitmap));
			printf("\tAP Coex Enabled = %d\n",
			       uap_le32_to_cpu(coex_common_tlv->ap_bt_coex));
			break;

		case MRVL_BT_COEX_SCO_CFG_TLV_ID:
			printf("Coex sco configuration:\n");
			coex_sco_tlv = (tlvbuf_coex_sco_cfg *)pcurrent_tlv;
			for (i = 0; i < 4; i++)
				printf("\tQtime protection [%d] = %d usecs\n",
				       i,
				       uap_le16_to_cpu(coex_sco_tlv->
						       protection_qtime[i]));
			printf("\tProtection frame rate = %d\n",
			       uap_le16_to_cpu(coex_sco_tlv->protection_rate));
			printf("\tACL frequency = %d\n",
			       uap_le16_to_cpu(coex_sco_tlv->acl_frequency));
			break;

		case MRVL_BT_COEX_ACL_CFG_TLV_ID:
			printf("Coex acl configuration: ");
			coex_acl_tlv = (tlvbuf_coex_acl_cfg *)pcurrent_tlv;
			coex_acl_tlv->enabled =
				uap_le16_to_cpu(coex_acl_tlv->enabled);
			printf("%s\n",
			       (coex_acl_tlv->
				enabled) ? "enabled" : "disabled");
			if (coex_acl_tlv->enabled) {
				printf("\tBT time = %d usecs\n",
				       uap_le16_to_cpu(coex_acl_tlv->bt_time));
				printf("\tWLan time = %d usecs\n",
				       uap_le16_to_cpu(coex_acl_tlv->
						       wlan_time));
				printf("\tProtection frame rate = %d\n",
				       uap_le16_to_cpu(coex_acl_tlv->
						       protection_rate));
			}
			break;

		case MRVL_BT_COEX_STATS_TLV_ID:
			printf("Coex statistics: \n");
			coex_stats_tlv = (tlvbuf_coex_stats *)pcurrent_tlv;
			printf("\tNull not sent = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->null_not_sent));
			printf("\tNull queued = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->null_queued));
			printf("\tNull not queued = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->
					       null_not_queued));
			printf("\tCF End queued = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->cf_end_queued));
			printf("\tCF End not queued = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->
					       cf_end_not_queued));
			printf("\tNull allocation failures = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->
					       null_alloc_fail));
			printf("\tCF End allocation failures = %d\n",
			       uap_le32_to_cpu(coex_stats_tlv->
					       cf_end_alloc_fail));
			break;
		case HT_CAPABILITY_TLV_ID:
			printf("\nHT Capability Info: \n");
			ht_cap_tlv = (tlvbuf_htcap_t *)pcurrent_tlv;
			if (!ht_cap_tlv->ht_cap.supported_mcs_set[0]) {
				printf("802.11n is disabled\n");
			} else {
				printf("802.11n is enabled\n");
				printf("ht_cap_info=0x%x, ampdu_param=0x%x tx_bf_cap=%#x\n", uap_le16_to_cpu(ht_cap_tlv->ht_cap.ht_cap_info), ht_cap_tlv->ht_cap.ampdu_param, uap_le32_to_cpu(ht_cap_tlv->ht_cap.tx_bf_cap));
				printf("supported MCS set:\n");
				for (i = 0; i < MCS_SET_LEN; i++) {
					printf("0x%x ",
					       ht_cap_tlv->ht_cap.
					       supported_mcs_set[i]);
				}
				printf("\n");
			}
			break;
		case HT_INFO_TLV_ID:
			ht_info_tlv = (tlvbuf_htinfo_t *)pcurrent_tlv;
			if (ht_info_tlv->length) {
				printf("\nHT Information Element: \n");
				printf("Primary channel = %d\n",
				       ht_info_tlv->ht_info.pri_chan);
				printf("Secondary channel offset = %d\n",
				       (int)GET_SECONDARY_CHAN(ht_info_tlv->
							       ht_info.field2));
				printf("STA channel width = %dMHz\n",
				       IS_CHANNEL_WIDTH_40(ht_info_tlv->ht_info.
							   field2) ? 40 : 20);
				printf("RIFS  %s\n",
				       IS_RIFS_ALLOWED(ht_info_tlv->ht_info.
						       field2) ? "Allowed" :
				       "Prohibited");
				ht_info_tlv->ht_info.field3 =
					uap_le16_to_cpu(ht_info_tlv->ht_info.
							field3);
				ht_info_tlv->ht_info.field4 =
					uap_le16_to_cpu(ht_info_tlv->ht_info.
							field4);
				printf("HT Protection = %d\n",
				       (int)GET_HT_PROTECTION(ht_info_tlv->
							      ht_info.field3));
				printf("Non-Greenfield HT STAs present: %s\n",
				       NONGF_STA_PRESENT(ht_info_tlv->ht_info.
							 field3) ? "Yes" :
				       "No");
				printf("OBSS Non-HT STAs present: %s\n",
				       OBSS_NONHT_STA_PRESENT(ht_info_tlv->
							      ht_info.
							      field3) ? "Yes" :
				       "No");
				for (i = 0; i < MCS_SET_LEN; i++) {
					if (ht_info_tlv->ht_info.
					    basic_mcs_set[i]) {
						printf("Basic_mcs_set: \n");
						flag = 1;
						break;
					}
				}
				if (flag) {
					for (i = 0; i < MCS_SET_LEN; i++)
						printf("%x ",
						       ht_info_tlv->ht_info.
						       basic_mcs_set[i]);
					printf("\n");
				}
			}
			break;
		case MRVL_2040_BSS_COEX_CONTROL_TLV_ID:
			coex_2040_tlv = (tlvbuf_2040_coex *)pcurrent_tlv;
			printf("20/40 coex = %s\n",
			       (coex_2040_tlv->
				enable) ? "enabled" : "disabled");
			break;
		case VENDOR_SPECIFIC_IE_TLV_ID:
			wmm_para_tlv = (tlvbuf_wmm_para_t *)pcurrent_tlv;
			printf("wmm parameters:\n");
			printf("\tqos_info = 0x%x\n",
			       wmm_para_tlv->wmm_para.qos_info);
			printf("\tBE: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
			       wmm_para_tlv->wmm_para.ac_params[AC_BE].
			       aci_aifsn.aifsn,
			       wmm_para_tlv->wmm_para.ac_params[AC_BE].ecw.
			       ecw_max,
			       wmm_para_tlv->wmm_para.ac_params[AC_BE].ecw.
			       ecw_min,
			       uap_le16_to_cpu(wmm_para_tlv->wmm_para.
					       ac_params[AC_BE].tx_op_limit));
			printf("\tBK: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
			       wmm_para_tlv->wmm_para.ac_params[AC_BK].
			       aci_aifsn.aifsn,
			       wmm_para_tlv->wmm_para.ac_params[AC_BK].ecw.
			       ecw_max,
			       wmm_para_tlv->wmm_para.ac_params[AC_BK].ecw.
			       ecw_min,
			       uap_le16_to_cpu(wmm_para_tlv->wmm_para.
					       ac_params[AC_BK].tx_op_limit));
			printf("\tVI: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
			       wmm_para_tlv->wmm_para.ac_params[AC_VI].
			       aci_aifsn.aifsn,
			       wmm_para_tlv->wmm_para.ac_params[AC_VI].ecw.
			       ecw_max,
			       wmm_para_tlv->wmm_para.ac_params[AC_VI].ecw.
			       ecw_min,
			       uap_le16_to_cpu(wmm_para_tlv->wmm_para.
					       ac_params[AC_VI].tx_op_limit));
			printf("\tVO: AIFSN=%d, CW_MAX=%d CW_MIN=%d, TXOP=%d\n",
			       wmm_para_tlv->wmm_para.ac_params[AC_VO].
			       aci_aifsn.aifsn,
			       wmm_para_tlv->wmm_para.ac_params[AC_VO].ecw.
			       ecw_max,
			       wmm_para_tlv->wmm_para.ac_params[AC_VO].ecw.
			       ecw_min,
			       uap_le16_to_cpu(wmm_para_tlv->wmm_para.
					       ac_params[AC_VO].tx_op_limit));
			break;
#ifdef RX_PACKET_COALESCE
		case MRVL_RX_PKT_COAL_TLV_ID:
			rx_pkt_coal_tlv = (tlvbuf_rx_pkt_coal_t *)pcurrent_tlv;
			printf("RX packet coalesce threshold=%d\n",
			       uap_le32_to_cpu(rx_pkt_coal_tlv->rx_pkt_count));
			printf("RX packet coalesce timeout in msec=%d\n",
			       uap_le16_to_cpu(rx_pkt_coal_tlv->delay));
			break;
#endif
		default:
			break;
		}
		tlv_buf_left -= (sizeof(tlvbuf_header) + tlv_len);
		pcurrent_tlv = (tlvbuf_header *)(pcurrent_tlv->data + tlv_len);
	}
	return;
}

/**
 *  @brief  Checks if command requires driver handling, and if so,
 *          repackage and send as regular command, instead of hostcmd.
 *
 *  @param cmd           Pointer to the command buffer
 *  @param size          Pointer to the command size. This value is
 *                       overwritten by the function with the size of the
 *                       received response.
 *  @param buf_size      Size of the allocated command buffer
 *  @param reroute_ret   Pointer to return value of rerouted command
 *  @return              UAP_SUCCESS (handled here, further processing not needed)
 *                    or UAP_FAILURE (not handled here, proceed as usual)
 */
static int
uap_ioctl_reroute(t_u8 *cmd, t_u16 *size, t_u16 buf_size, int *reroute_ret)
{
	t_u8 reroute = 0;
	t_u16 cmd_code = 0;
	t_u16 cmd_action = 0;
	apcmdbuf_cfg_80211d *hcmd_domain = NULL;
	apcmdbuf_snmp_mib *hcmd_snmp = NULL;
	tlvbuf_header *tlv = NULL;
	struct ifreq ifr;
	t_s32 sockfd;
	t_u8 *buf = NULL;
	t_u16 buf_len = 0;
	snmp_mib_param *snmp_param = NULL;
	domain_info_param *domain_param = NULL;

	/* assume input is a hostcmd */
	cmd_code = ((apcmdbuf *)(cmd))->cmd_code;

	/* just check if we need to re-route right now */
	switch (cmd_code) {
	case HostCmd_SNMP_MIB:
		hcmd_snmp = (apcmdbuf_snmp_mib *)cmd;
		tlv = (tlvbuf_header *)(cmd + sizeof(apcmdbuf_snmp_mib));
		cmd_action = uap_le16_to_cpu(hcmd_snmp->action);
		/* reroute CMD_SNMP_MIB: SET */
		if (cmd_action == ACTION_SET) {
			reroute = 1;
			buf_len =
				sizeof(snmp_mib_param) +
				uap_le16_to_cpu(tlv->len);
		}
		break;
	case HostCmd_CMD_802_11D_DOMAIN_INFO:
		hcmd_domain = (apcmdbuf_cfg_80211d *)cmd;
		cmd_action = uap_le16_to_cpu(hcmd_domain->action);
		/* reroute CMD_DOMAIN_INFO: SET */
		if (cmd_action == ACTION_SET) {
			reroute = 1;
			buf_len = sizeof(domain_info_param) + (*size
							       -
							       sizeof
							       (apcmdbuf_cfg_80211d)
							       +
							       sizeof
							       (domain_param_t));
		}
		break;
	}

	/* Exit early if not re-routing */
	if (!reroute || uap_ioctl_no_reroute)
		return UAP_FAILURE;

	/* Prepare buffer */
#if DEBUG
	uap_printf(MSG_DEBUG, "DBG: rerouting CMD 0x%04x\n", cmd_code);
#endif
	buf = (t_u8 *)malloc(buf_len);
	if (!buf) {
		printf("ERR:Cannot allocate buffer for command!\n");
		goto done_no_socket;
	}
	memset(buf, 0, buf_len);

	/* Prepare param */
	switch (cmd_code) {
	case HostCmd_SNMP_MIB:
		snmp_param = (snmp_mib_param *)buf;
		snmp_param->subcmd = UAP_SNMP_MIB;
		snmp_param->action = cmd_action;
		snmp_param->oid = uap_le16_to_cpu(tlv->type);
		snmp_param->oid_val_len = uap_le16_to_cpu(tlv->len);
		memcpy(snmp_param->oid_value, tlv->data,
		       MIN(sizeof(t_u32), snmp_param->oid_val_len));
		break;
	case HostCmd_CMD_802_11D_DOMAIN_INFO:
		domain_param = (domain_info_param *)buf;
		domain_param->subcmd = UAP_DOMAIN_INFO;
		domain_param->action = cmd_action;
		memcpy(domain_param->tlv, &hcmd_domain->domain,
		       buf_len - sizeof(domain_info_param));
		tlv = (tlvbuf_header *)domain_param->tlv;
		tlv->type = uap_le16_to_cpu(tlv->type);
		tlv->len = uap_le16_to_cpu(tlv->len);

		break;
	}
#if DEBUG
	/* Dump request buffer */
	hexdump("Reroute Request buffer", buf, buf_len, ' ');
#endif

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		goto done_no_socket;
	}
	/* Initialize the ifr structure */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, dev_name, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)buf;
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, UAP_IOCTL_CMD, &ifr)) {
		*reroute_ret = UAP_FAILURE;
		perror("");
#if DEBUG
		uap_printf(MSG_DEBUG, "ERR: reroute of CMD 0x%04x failed\n",
			   cmd_code);
#endif
		goto done;
	}
	*reroute_ret = UAP_SUCCESS;

done:
	/* Close socket */
	close(sockfd);
done_no_socket:
	if (buf)
		free(buf);

	return UAP_SUCCESS;
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
 *  @return              UAP_SUCCESS or UAP_FAILURE
 */
int
uap_ioctl(t_u8 *cmd, t_u16 *size, t_u16 buf_size)
{
	struct ifreq ifr;
	apcmdbuf *header = NULL;
	t_s32 sockfd;
	int reroute_ret = 0;
	mrvl_priv_cmd *mrvl_cmd = NULL;
	t_u8 *buf = NULL, *temp = NULL;
	t_u16 mrvl_header_len = 0;
	int ret = UAP_SUCCESS;

#ifdef WIFI_DIRECT_SUPPORT
	if (strncmp(dev_name, "wfd", 3))
#endif
		if (uap_ioctl_reroute(cmd, size, buf_size, &reroute_ret) ==
		    UAP_SUCCESS) {
			return reroute_ret;
		}

	if (buf_size < *size) {
		printf("buf_size should not less than cmd buffer size\n");
		return UAP_FAILURE;
	}

	/* Open socket */
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERR:Cannot open socket\n");
		return UAP_FAILURE;
	}
	*(t_u32 *)cmd = buf_size - BUF_HEADER_SIZE;

	mrvl_header_len = strlen(CMD_NXP) + strlen(PRIV_CMD_HOSTCMD);
	buf = (unsigned char *)malloc(buf_size + sizeof(mrvl_priv_cmd) +
				      mrvl_header_len);
	if (buf == NULL) {
		close(sockfd);
		return UAP_FAILURE;
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
	header = (apcmdbuf *)(buf + sizeof(mrvl_priv_cmd) + mrvl_header_len);
	header->size = *size - BUF_HEADER_SIZE;
	if (header->cmd_code == APCMD_SYS_CONFIGURE) {
		apcmdbuf_sys_configure *sys_cfg;
		sys_cfg = (apcmdbuf_sys_configure *)header;
		sys_cfg->action = uap_cpu_to_le16(sys_cfg->action);
	}
	endian_convert_request_header(header);
#if DEBUG
	/* Dump request buffer */
	hexdump("Request buffer", mrvl_cmd,
		*size + sizeof(mrvl_priv_cmd) + mrvl_header_len, ' ');
#endif
	/* Perform ioctl */
	errno = 0;
	if (ioctl(sockfd, MRVLPRIVCMD, &ifr)) {
		perror("");
		printf("ERR:MRVLPRIVCMD is not supported by %s\n", dev_name);
		ret = UAP_FAILURE;
		goto done;
	}
	endian_convert_response_header(header);
	header->cmd_code &= HostCmd_CMD_ID_MASK;
	header->cmd_code |= APCMD_RESP_CHECK;
	*size = header->size;

	/* Validate response size */
	if (*size > (buf_size - BUF_HEADER_SIZE)) {
		printf("ERR:Response size (%d) greater than buffer size (%d)! Aborting!\n", *size, buf_size);
		ret = UAP_FAILURE;
		goto done;
	}
	memcpy(cmd, (t_u8 *)header, *size + BUF_HEADER_SIZE);
#if DEBUG
	/* Dump respond buffer */
	hexdump("Respond buffer", mrvl_cmd,
		*size + BUF_HEADER_SIZE + sizeof(mrvl_priv_cmd) +
		mrvl_header_len, ' ');
#endif

done:
	/* Close socket */
	close(sockfd);
	if (buf)
		free(buf);
	return ret;
}

/**
 *  @brief  Get protocol from the firmware
 *
 *  @param  proto   A pointer to protocol var
 *  @return         UAP_SUCCESS/UAP_FAILURE
 */
int
get_sys_cfg_protocol(t_u16 *proto)
{
	apcmdbuf_sys_configure *cmd_buf = NULL;
	tlvbuf_protocol *tlv = NULL;
	t_u8 *buffer = NULL;
	t_u16 cmd_len;
	int ret = UAP_FAILURE;

	cmd_len = sizeof(apcmdbuf_sys_configure) + sizeof(tlvbuf_protocol);
	/* Initialize the command buffer */
	buffer = (t_u8 *)malloc(cmd_len);
	if (!buffer) {
		printf("ERR:Cannot allocate buffer for command!\n");
		return ret;
	}
	memset(buffer, 0, cmd_len);
	/* Locate headers */
	cmd_buf = (apcmdbuf_sys_configure *)buffer;
	tlv = (tlvbuf_protocol *)(buffer + sizeof(apcmdbuf_sys_configure));
	/* Fill the command buffer */
	cmd_buf->cmd_code = APCMD_SYS_CONFIGURE;
	cmd_buf->size = cmd_len;
	cmd_buf->seq_num = 0;
	cmd_buf->result = 0;
	cmd_buf->action = ACTION_GET;
	tlv->tag = MRVL_PROTOCOL_TLV_ID;
	tlv->length = 2;
	endian_convert_tlv_header_out(tlv);
	/* Send the command */
	ret = uap_ioctl((t_u8 *)cmd_buf, &cmd_len, cmd_len);
	endian_convert_tlv_header_in(tlv);

	/* Process response */
	if (ret == UAP_SUCCESS) {
		/* Verify response */
		if (cmd_buf->cmd_code !=
		    (APCMD_SYS_CONFIGURE | APCMD_RESP_CHECK)) {
			printf("ERR:Corrupted response! cmd_code=%x, Tlv->tag=%x\n", cmd_buf->cmd_code, tlv->tag);
			free(buffer);
			return UAP_FAILURE;
		}

		if (cmd_buf->result == CMD_SUCCESS) {
			tlv->protocol = uap_le16_to_cpu(tlv->protocol);
			memcpy(proto, &tlv->protocol, sizeof(tlv->protocol));
		} else {
			ret = UAP_FAILURE;
		}
	}
	if (buffer)
		free(buffer);
	return ret;
}

/**
 *  @brief Check cipher is valid or not
 *
 *  @param pairwisecipher    Pairwise cipher
 *  @param groupcipher       Group cipher
 *  @param protocol          Protocol
 *  @param enable_11n        11n enabled or not.
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_cipher_valid_with_11n(int pairwisecipher, int groupcipher,
			 int protocol, int enable_11n)
{
	if ((pairwisecipher == CIPHER_NONE) && (groupcipher == CIPHER_NONE))
		return UAP_SUCCESS;
	if (pairwisecipher == CIPHER_TKIP) {
		/* Ok to have TKIP in mixed mode */
		if (enable_11n && protocol != PROTOCOL_WPA2_MIXED) {
			printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
			return UAP_FAILURE;
		}
	}
	if ((pairwisecipher == CIPHER_TKIP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) &&
	    (groupcipher == CIPHER_AES_CCMP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_BITMAP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	return UAP_FAILURE;
}

/**
 *  @brief Check cipher is valid or not based on proto
 *
 *  @param pairwisecipher    Pairwise cipher
 *  @param groupcipher       Group cipher
 *  @param protocol          Protocol
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_cipher_valid_with_proto(int pairwisecipher, int groupcipher, int protocol)
{
	HTCap_t htcap;

	if ((pairwisecipher == CIPHER_NONE) && (groupcipher == CIPHER_NONE))
		return UAP_SUCCESS;
	if (pairwisecipher == CIPHER_TKIP) {
		/* Ok to have TKIP in mixed mode */
		if (protocol != PROTOCOL_WPA2_MIXED) {
			memset(&htcap, 0, sizeof(htcap));
			if (UAP_SUCCESS == get_sys_cfg_11n(&htcap)) {
				if (htcap.supported_mcs_set[0]) {
					printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
					return UAP_FAILURE;
				}
			}
		}
	}
	if ((pairwisecipher == CIPHER_TKIP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) &&
	    (groupcipher == CIPHER_AES_CCMP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_BITMAP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	return UAP_FAILURE;
}

/**
 *  @brief Check cipher is valid or not
 *
 *  @param pairwisecipher    Pairwise cipher
 *  @param groupcipher       Group cipher
 *  @return         UAP_SUCCESS or UAP_FAILURE
 */
int
is_cipher_valid(int pairwisecipher, int groupcipher)
{
	HTCap_t htcap;
	t_u16 proto = 0;

	if ((pairwisecipher == CIPHER_NONE) && (groupcipher == CIPHER_NONE))
		return UAP_SUCCESS;
	if (pairwisecipher == CIPHER_TKIP) {
		if (UAP_SUCCESS == get_sys_cfg_protocol(&proto)) {
			/* Ok to have TKIP in mixed mode */
			if (proto != PROTOCOL_WPA2_MIXED) {
				memset(&htcap, 0, sizeof(htcap));
				if (UAP_SUCCESS == get_sys_cfg_11n(&htcap)) {
					if (htcap.supported_mcs_set[0]) {
						printf("ERR: WPA/TKIP cannot be used when AP operates in 802.11n mode.\n");
						return UAP_FAILURE;
					}
				}
			}
		}
	}
	if ((pairwisecipher == CIPHER_TKIP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) &&
	    (groupcipher == CIPHER_AES_CCMP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_AES_CCMP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	if ((pairwisecipher == CIPHER_BITMAP) && (groupcipher == CIPHER_TKIP))
		return UAP_SUCCESS;
	return UAP_FAILURE;
}

/**
 *  @brief The main function
 *
 *  @param argc     Number of arguments
 *  @param argv     Pointer to the arguments
 *  @return         0 or 1
 */
int
main(int argc, char *argv[])
{
	int opt, i;
	int ret = 0;
	memset(dev_name, 0, sizeof(dev_name));
	strcpy(dev_name, DEFAULT_DEV_NAME);

	/* Parse arguments */
	while ((opt =
		getopt_long(argc, argv, "+hi:d:v", ap_options, NULL)) != -1) {
		switch (opt) {
		case 'i':
			if (strlen(optarg) < IFNAMSIZ) {
				memset(dev_name, 0, sizeof(dev_name));
				strncpy(dev_name, optarg, strlen(optarg));
			}
			printf("dev_name:%s\n", dev_name);
			break;
		case 'v':
			printf("uaputl.exe - uAP utility ver %s\n",
			       UAP_VERSION);
			exit(0);
		case 'd':
			debug_level = strtoul(optarg, NULL, 10);
#if DEBUG
			uap_printf(MSG_DEBUG, "debug_level=%x\n", debug_level);
#endif
			break;
		case 'h':
		default:
			print_tool_usage();
			exit(0);
		}
	}

	argc -= optind;
	argv += optind;
	optind = 0;

	if (argc < 1) {
		print_tool_usage();
		exit(1);
	}

	/* Process command */
	for (i = 0; ap_command[i].cmd; i++) {
		if (strncmp
		    (ap_command[i].cmd, argv[0], strlen(ap_command[i].cmd)))
			continue;
		if (strlen(ap_command[i].cmd) != strlen(argv[0]))
			continue;
		ret = ap_command[i].func(argc, argv);
		break;
	}
	if (!ap_command[i].cmd) {
		printf("ERR: %s is not supported\n", argv[0]);
		exit(1);
	}
	if (ret == UAP_FAILURE)
		return -1;
	else
		return 0;
}
