/** @file  mlanmisc.c
  *
  * @brief Program to prepare command buffer
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
     03/10/2009: initial version
************************************************************************/

#include    "mlanconfig.h"
#include    "mlanhostcmd.h"
#include    "mlanmisc.h"

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
 *  @brief Helper function for process_getscantable_idx
 *
 *  @param pbuf     A pointer to the buffer
 *  @param buf_len  buffer length
 *
 *  @return         NA
 *
 */
static void
dump_scan_elems(const t_u8 *pbuf, uint buf_len)
{
	uint idx;
	uint marker = 2 + pbuf[1];

	for (idx = 0; idx < buf_len; idx++) {
		if (idx % 0x10 == 0) {
			printf("\n%04x: ", idx);
		}

		if (idx == marker) {
			printf("|");
			marker = idx + pbuf[idx + 1] + 2;
		} else {
			printf(" ");
		}

		printf("%02x ", pbuf[idx]);
	}

	printf("\n");
}

/**
 *  @brief Helper function for process_getscantable_idx
 *  Find next element
 *
 *  @param pp_ie_out    pointer of a IEEEtypes_Generic_t structure pointer
 *  @param p_buf_left   integer pointer, which contains the number of left p_buf
 *
 *  @return             MLAN_STATUS_SUCCESS on success, otherwise MLAN_STATUS_FAILURE
 */
static int
scantable_elem_next(IEEEtypes_Generic_t **pp_ie_out, int *p_buf_left)
{
	IEEEtypes_Generic_t *pie_gen;
	t_u8 *p_next;

	if (*p_buf_left < 2) {
		return MLAN_STATUS_FAILURE;
	}

	pie_gen = *pp_ie_out;

	p_next = (t_u8 *)pie_gen + (pie_gen->ieee_hdr.len
				    + sizeof(pie_gen->ieee_hdr));
	*p_buf_left -= (p_next - (t_u8 *)pie_gen);

	*pp_ie_out = (IEEEtypes_Generic_t *)p_next;

	if (*p_buf_left <= 0) {
		return MLAN_STATUS_FAILURE;
	}

	return MLAN_STATUS_SUCCESS;
}

 /**
  *  @brief Helper function for process_getscantable_idx
  *         scantable find element
  *
  *  @param ie_buf       pointer of the IE buffer
  *  @param ie_buf_len   IE buffer length
  *  @param ie_type      IE type
  *  @param ppie_out     pointer to the IEEEtypes_Generic_t structure pointer
  *  @return             MLAN_STATUS_SUCCESS on success, otherwise MLAN_STATUS_FAILURE
  */
static int
scantable_find_elem(t_u8 *ie_buf,
		    unsigned int ie_buf_len,
		    IEEEtypes_ElementId_e ie_type,
		    IEEEtypes_Generic_t **ppie_out)
{
	int found;
	unsigned int ie_buf_left;

	ie_buf_left = ie_buf_len;

	found = FALSE;

	*ppie_out = (IEEEtypes_Generic_t *)ie_buf;

	do {
		found = ((*ppie_out)->ieee_hdr.element_id == ie_type);

	} while (!found &&
		 (scantable_elem_next(ppie_out, (int *)&ie_buf_left) == 0));

	if (!found) {
		*ppie_out = NULL;
	}

	return (found ? MLAN_STATUS_SUCCESS : MLAN_STATUS_FAILURE);
}

 /**
  *  @brief Helper function for process_getscantable_idx
  *         It gets SSID from IE
  *
  *  @param ie_buf       IE buffer
  *  @param ie_buf_len   IE buffer length
  *  @param pssid        SSID
  *  @param ssid_buf_max size of SSID
  *  @return             MLAN_STATUS_SUCCESS on success, otherwise MLAN_STATUS_FAILURE
  */
static int
scantable_get_ssid_from_ie(t_u8 *ie_buf,
			   unsigned int ie_buf_len,
			   t_u8 *pssid, unsigned int ssid_buf_max)
{
	int retval;
	IEEEtypes_Generic_t *pie_gen;

	retval = scantable_find_elem(ie_buf, ie_buf_len, SSID, &pie_gen);
	if (retval == MLAN_STATUS_SUCCESS)
		memcpy(pssid, pie_gen->data,
		       MIN(pie_gen->ieee_hdr.len, ssid_buf_max));

	return retval;
}

/**
 *  @brief Display detailed information for a specific scan table entry
 *
 *  @param prsp_info_req    Scan table entry request structure
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_getscantable_idx(wlan_ioctl_get_scan_table_info *prsp_info_req)
{
	int ioctl_val, subioctl_val;
	struct iwreq iwr;
	t_u8 *pcurrent;
	int ret = 0;
	char ssid[33];
	t_u16 tmp_cap;
	t_u8 tsf[8];
	t_u16 beacon_interval;
	t_u8 *scan_rsp_buf = NULL;
	t_u16 cap_info;
	wlan_ioctl_get_scan_table_info *prsp_info;

	wlan_get_scan_table_fixed fixed_fields;
	t_u32 fixed_field_length;
	t_u32 bss_info_length;

	scan_rsp_buf = (t_u8 *)malloc(SCAN_RESP_BUF_SIZE);
	if (scan_rsp_buf == NULL) {
		printf("Error: allocate memory for scan_rsp_buf failed\n");
		return -ENOMEM;
	}
	memset(ssid, 0x00, sizeof(ssid));

	prsp_info = (wlan_ioctl_get_scan_table_info *)scan_rsp_buf;

	memcpy(prsp_info, prsp_info_req,
	       sizeof(wlan_ioctl_get_scan_table_info));

	if (get_priv_ioctl("getscantable",
			   &ioctl_val, &subioctl_val) == MLAN_STATUS_FAILURE) {
		ret = -EOPNOTSUPP;
		goto done;
	}

	/*
	 * Set up and execute the ioctl call
	 */
	strncpy(iwr.ifr_name, dev_name, IFNAMSIZ - 1);
	iwr.u.data.pointer = (caddr_t) prsp_info;
	iwr.u.data.length = SCAN_RESP_BUF_SIZE;
	iwr.u.data.flags = subioctl_val;

	if (ioctl(sockfd, ioctl_val, &iwr) < 0) {
		perror("mlanconfig: getscantable ioctl");
		ret = -EFAULT;
		goto done;
	}

	if (prsp_info->scan_number == 0) {
		printf("mlanconfig: getscantable ioctl - index out of range\n");
		ret = -EINVAL;
		goto done;
	}

	pcurrent = prsp_info->scan_table_entry_buf;

	memcpy((t_u8 *)&fixed_field_length,
	       (t_u8 *)pcurrent, sizeof(fixed_field_length));
	pcurrent += sizeof(fixed_field_length);

	memcpy((t_u8 *)&bss_info_length,
	       (t_u8 *)pcurrent, sizeof(bss_info_length));
	pcurrent += sizeof(bss_info_length);

	memcpy((t_u8 *)&fixed_fields, (t_u8 *)pcurrent, sizeof(fixed_fields));
	pcurrent += fixed_field_length;

	/* time stamp is 8 byte long */
	memcpy(tsf, pcurrent, sizeof(tsf));
	pcurrent += sizeof(tsf);
	bss_info_length -= sizeof(tsf);

	/* beacon interval is 2 byte long */
	memcpy(&beacon_interval, pcurrent, sizeof(beacon_interval));
	pcurrent += sizeof(beacon_interval);
	bss_info_length -= sizeof(beacon_interval);

	/* capability information is 2 byte long */
	memcpy(&cap_info, pcurrent, sizeof(cap_info));
	pcurrent += sizeof(cap_info);
	bss_info_length -= sizeof(cap_info);

	scantable_get_ssid_from_ie(pcurrent,
				   bss_info_length, (t_u8 *)ssid, sizeof(ssid));

	printf("\n*** [%s], %02x:%02x:%02x:%02x:%02x:%2x\n",
	       ssid,
	       fixed_fields.bssid[0],
	       fixed_fields.bssid[1],
	       fixed_fields.bssid[2],
	       fixed_fields.bssid[3],
	       fixed_fields.bssid[4], fixed_fields.bssid[5]);
	memcpy(&tmp_cap, &cap_info, sizeof(tmp_cap));
	printf("Channel = %d, SS = %d, CapInfo = 0x%04x, BcnIntvl = %d\n",
	       fixed_fields.channel,
	       255 - fixed_fields.rssi, tmp_cap, beacon_interval);

	printf("TSF Values: AP(0x%02x%02x%02x%02x%02x%02x%02x%02x), ",
	       tsf[7], tsf[6], tsf[5], tsf[4], tsf[3], tsf[2], tsf[1], tsf[0]);

	printf("Network(0x%016llx)\n", fixed_fields.network_tsf);
	printf("\n");
	printf("Element Data (%d bytes)\n", (int)bss_info_length);
	printf("------------");
	dump_scan_elems(pcurrent, bss_info_length);
	printf("\n");

done:

	if (scan_rsp_buf)
		free(scan_rsp_buf);

	return ret;
}

/********************************************************
		Global Functions
********************************************************/

/**
 *  @brief Retrieve and display the contents of the driver scan table.
 *
 *  The ioctl to retrieve the scan table contents will be invoked, and portions
 *   of the scan data will be displayed on stdout.  The entire beacon or
 *   probe response is also retrieved (if available in the driver).  This
 *   data would be needed in case the application was explicitly controlling
 *   the association (inserting IEs, TLVs, etc).
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_getscantable(int argc, char *argv[])
{
	int ioctl_val, subioctl_val;
	struct iwreq iwr;
	t_u8 *scan_rsp_buf = NULL;

	struct wlan_ioctl_get_scan_list *scan_list_head = NULL;
	struct wlan_ioctl_get_scan_list *scan_list_node = NULL;
	struct wlan_ioctl_get_scan_list *curr = NULL, *prev = NULL, *next =
		NULL;

	t_u32 total_scan_res = 0;

	unsigned int scan_start;
	int idx, ret = 0;

	t_u8 *pcurrent;
	t_u8 *pnext;
	IEEEtypes_ElementId_e *pelement_id;
	t_u8 *pelement_len;
	int ssid_idx;
	t_u8 *pbyte;
	t_u16 new_ss;
	t_u16 curr_ss;

	IEEEtypes_VendorSpecific_t *pwpa_ie;
	const t_u8 wpa_oui[4] = { 0x00, 0x50, 0xf2, 0x01 };

	IEEEtypes_WmmParameter_t *pwmm_ie;
	const t_u8 wmm_oui[4] = { 0x00, 0x50, 0xf2, 0x02 };
	IEEEtypes_VendorSpecific_t *pwps_ie;
	const t_u8 wps_oui[4] = { 0x00, 0x50, 0xf2, 0x04 };

	int displayed_info;

	wlan_ioctl_get_scan_table_info rspInfoReq;
	wlan_ioctl_get_scan_table_info *prsp_info;

	wlan_get_scan_table_fixed fixed_fields;
	t_u32 fixed_field_length;
	t_u32 bss_info_length;
	wlan_ioctl_get_bss_info *bss_info;

	scan_rsp_buf = (t_u8 *)malloc(SCAN_RESP_BUF_SIZE);
	if (scan_rsp_buf == NULL) {
		printf("Error: allocate memory for scan_rsp_buf failed\n");
		return -ENOMEM;
	}
	prsp_info = (wlan_ioctl_get_scan_table_info *)scan_rsp_buf;

	if (get_priv_ioctl("getscantable",
			   &ioctl_val, &subioctl_val) == MLAN_STATUS_FAILURE) {
		ret = -EOPNOTSUPP;
		goto done;
	}

	if (argc > 3 && (strcmp(argv[3], "tsf") != 0)
	    && (strcmp(argv[3], "help") != 0)) {

		idx = strtol(argv[3], NULL, 10);

		if (idx >= 0) {
			if (scan_rsp_buf) {
				free(scan_rsp_buf);
			}
			rspInfoReq.scan_number = idx;
			return process_getscantable_idx(&rspInfoReq);
		}
	}

	displayed_info = FALSE;
	scan_start = 1;

	do {
		prsp_info->scan_number = scan_start;

		/*
		 * Set up and execute the ioctl call
		 */
		strncpy(iwr.ifr_name, dev_name, IFNAMSIZ - 1);
		iwr.u.data.pointer = (caddr_t) prsp_info;
		iwr.u.data.length = SCAN_RESP_BUF_SIZE;
		iwr.u.data.flags = subioctl_val;

		if (ioctl(sockfd, ioctl_val, &iwr) < 0) {
			perror("mlanconfig: getscantable ioctl");
			ret = -EFAULT;
			goto done;
		}
		total_scan_res += prsp_info->scan_number;

		pcurrent = 0;
		pnext = prsp_info->scan_table_entry_buf;

		for (idx = 0; (unsigned int)idx < prsp_info->scan_number; idx++) {

			/* Alloc memory for new node for next BSS */
			scan_list_node = (struct wlan_ioctl_get_scan_list *)
				malloc(sizeof(struct wlan_ioctl_get_scan_list));
			if (scan_list_node == NULL) {
				printf("Error: allocate memory for scan_list_head failed\n");
				return -ENOMEM;
			}
			memset(scan_list_node, 0,
			       sizeof(struct wlan_ioctl_get_scan_list));

			/*
			 * Set pcurrent to pnext in case pad bytes are at the end
			 *   of the last IE we processed.
			 */
			pcurrent = pnext;

			/* Start extracting each BSS to prepare a linked list */
			memcpy((t_u8 *)&fixed_field_length,
			       (t_u8 *)pcurrent, sizeof(fixed_field_length));
			pcurrent += sizeof(fixed_field_length);

			memcpy((t_u8 *)&bss_info_length,
			       (t_u8 *)pcurrent, sizeof(bss_info_length));
			pcurrent += sizeof(bss_info_length);

			memcpy((t_u8 *)&fixed_fields,
			       (t_u8 *)pcurrent, sizeof(fixed_fields));
			pcurrent += fixed_field_length;

			scan_list_node->fixed_buf.fixed_field_length =
				fixed_field_length;
			scan_list_node->fixed_buf.bss_info_length =
				bss_info_length;
			scan_list_node->fixed_buf.fixed_fields = fixed_fields;

			bss_info = &scan_list_node->bss_info_buf;

			/* Set next to be the start of the next scan entry */
			pnext = pcurrent + bss_info_length;

			if (bss_info_length >=
			    (sizeof(bss_info->tsf) +
			     sizeof(bss_info->beacon_interval) +
			     sizeof(bss_info->cap_info))) {

				/* time stamp is 8 byte long */
				memcpy(bss_info->tsf, pcurrent,
				       sizeof(bss_info->tsf));
				pcurrent += sizeof(bss_info->tsf);
				bss_info_length -= sizeof(bss_info->tsf);

				/* beacon interval is 2 byte long */
				memcpy(&bss_info->beacon_interval, pcurrent,
				       sizeof(bss_info->beacon_interval));
				pcurrent += sizeof(bss_info->beacon_interval);
				bss_info_length -=
					sizeof(bss_info->beacon_interval);

				/* capability information is 2 byte long */
				memcpy(&bss_info->cap_info, pcurrent,
				       sizeof(bss_info->cap_info));
				pcurrent += sizeof(bss_info->cap_info);
				bss_info_length -= sizeof(bss_info->cap_info);
			}

			bss_info->wmm_cap = ' ';	/* M (WMM), C (WMM-Call Admission Control) */
			bss_info->wps_cap = ' ';	/* "S" */
			bss_info->dot11k_cap = ' ';	/* "K" */
			bss_info->dot11r_cap = ' ';	/* "R" */
			bss_info->ht_cap = ' ';	/* "N" */

			/* "P" for Privacy (WEP) since "W" is WPA, and "2" is RSN/WPA2 */
			bss_info->priv_cap =
				bss_info->cap_info.privacy ? 'P' : ' ';

			memset(bss_info->ssid, 0, MRVDRV_MAX_SSID_LENGTH + 1);
			bss_info->ssid_len = 0;

			while (bss_info_length >= 2) {
				pelement_id = (IEEEtypes_ElementId_e *)pcurrent;
				pelement_len = pcurrent + 1;
				pcurrent += 2;

				switch (*pelement_id) {

				case SSID:
					if (*pelement_len &&
					    *pelement_len <=
					    MRVDRV_MAX_SSID_LENGTH) {
						memcpy(bss_info->ssid, pcurrent,
						       *pelement_len);
						bss_info->ssid_len =
							*pelement_len;
					}
					break;

				case WPA_IE:
					pwpa_ie =
						(IEEEtypes_VendorSpecific_t *)
						pelement_id;
					if ((memcmp
					     (pwpa_ie->vend_hdr.oui, wpa_oui,
					      sizeof(pwpa_ie->vend_hdr.oui)) ==
					     0)
					    && (pwpa_ie->vend_hdr.oui_type ==
						wpa_oui[3])) {
						/* WPA IE found, 'W' for WPA */
						bss_info->priv_cap = 'W';
					} else {
						pwmm_ie =
							(IEEEtypes_WmmParameter_t
							 *)pelement_id;
						if ((memcmp
						     (pwmm_ie->vend_hdr.oui,
						      wmm_oui,
						      sizeof(pwmm_ie->vend_hdr.
							     oui)) == 0)
						    && (pwmm_ie->vend_hdr.
							oui_type ==
							wmm_oui[3])) {
							/* Check the subtype: 1 == parameter, 0 == info */
							if ((pwmm_ie->vend_hdr.
							     oui_subtype == 1)
							    && pwmm_ie->
							    ac_params
							    [WMM_AC_VO].
							    aci_aifsn.acm) {
								/* Call admission on VO; 'C' for CAC */
								bss_info->
									wmm_cap
									= 'C';
							} else {
								/* No CAC; 'M' for uh, WMM */
								bss_info->
									wmm_cap
									= 'M';
							}
						} else {
							pwps_ie =
								(IEEEtypes_VendorSpecific_t
								 *)pelement_id;
							if ((memcmp
							     (pwps_ie->vend_hdr.
							      oui, wps_oui,
							      sizeof(pwps_ie->
								     vend_hdr.
								     oui)) == 0)
							    && (pwps_ie->
								vend_hdr.
								oui_type ==
								wps_oui[3])) {
								bss_info->
									wps_cap
									= 'S';
							}
						}
					}
					break;

				case RSN_IE:
					/* RSN IE found; '2' for WPA2 (RSN) */
					bss_info->priv_cap = '2';
					break;
				case HT_CAPABILITY:
					bss_info->ht_cap = 'N';
					break;
				case VHT_CAPABILITY:
					bss_info->vht_cap[0] = 'A';
					bss_info->vht_cap[1] = 'C';
					break;
				default:
					break;
				}

				pcurrent += *pelement_len;
				bss_info_length -= (2 + *pelement_len);
			}

			/* Create a sorted list of BSS using Insertion Sort.
			   Sort as per Signal Strength (descending order) */
			new_ss = 255 - fixed_fields.rssi;

			if (scan_list_head == NULL) {
				// Node is the first element in the list.
				scan_list_head = scan_list_node;
				scan_list_node->next = NULL;
			} else {

				curr_ss =
					255 -
					scan_list_head->fixed_buf.fixed_fields.
					rssi;

				if (new_ss > curr_ss) {
					// Insert the node to head of the list
					scan_list_node->next = scan_list_head;
					scan_list_head = scan_list_node;
				} else {

					for (curr = scan_list_head;
					     curr != NULL; curr = curr->next) {

						curr_ss =
							255 -
							curr->fixed_buf.
							fixed_fields.rssi;
						if (prev && new_ss > curr_ss) {
							// Insert the node to current position in list
							scan_list_node->next =
								curr;
							prev->next =
								scan_list_node;
							break;
						}
						prev = curr;
					}
					if (prev && (curr == NULL)) {

						// Insert the node to tail of the list
						prev->next = scan_list_node;
						scan_list_node->next = NULL;
					}
				}
			}
		}

		scan_start += prsp_info->scan_number;
	} while (prsp_info->scan_number);

	// Display scan results
	printf("---------------------------------------");
	printf("---------------------------------------\n");
	printf("# | ch  | ss  |       bssid       |   cap    |   SSID \n");
	printf("---------------------------------------");
	printf("---------------------------------------\n");

	for (curr = scan_list_head, idx = 0;
	     (curr != NULL) && ((unsigned int)idx < total_scan_res);
	     curr = curr->next, idx++) {

		fixed_fields = curr->fixed_buf.fixed_fields;
		bss_info = &curr->bss_info_buf;

		printf("%02u| %03d | %03d | %02x:%02x:%02x:%02x:%02x:%02x |",
		       idx,
		       fixed_fields.channel,
		       255 - fixed_fields.rssi,
		       fixed_fields.bssid[0],
		       fixed_fields.bssid[1],
		       fixed_fields.bssid[2],
		       fixed_fields.bssid[3],
		       fixed_fields.bssid[4], fixed_fields.bssid[5]);

		displayed_info = TRUE;

		/* "A" for Adhoc
		 * "I" for Infrastructure,
		 * "D" for DFS (Spectrum Mgmt)
		 */
		printf(" %c%c%c%c%c%c%c%c%c%c | ", bss_info->cap_info.ibss ? 'A' : 'I', bss_info->priv_cap,	/* P (WEP), W (WPA), 2 (WPA2) */
		       bss_info->cap_info.spectrum_mgmt ? 'D' : ' ', bss_info->wmm_cap,	/* M (WMM), C (WMM-Call Admission Control) */
		       bss_info->dot11k_cap,	/* K */
		       bss_info->dot11r_cap,	/* R */
		       bss_info->wps_cap,	/* S */
		       bss_info->ht_cap,	/* N */
		       bss_info->vht_cap[0],	/* AC */
		       bss_info->vht_cap[1]);

		/* Print out the ssid or the hex values if non-printable */
		for (ssid_idx = 0; ssid_idx < bss_info->ssid_len; ssid_idx++) {
			if (isprint(bss_info->ssid[ssid_idx])) {
				printf("%c", bss_info->ssid[ssid_idx]);
			} else {
				printf("\\%02x", bss_info->ssid[ssid_idx]);
			}
		}

		printf("\n");

		if (argc > 3 && strcmp(argv[3], "tsf") == 0) {
			/* TSF is a u64, some formatted printing libs have trouble
			   printing long longs, so cast and dump as bytes */
			pbyte = (t_u8 *)&fixed_fields.network_tsf;
			printf("    TSF=%02x%02x%02x%02x%02x%02x%02x%02x\n",
			       pbyte[7], pbyte[6], pbyte[5], pbyte[4],
			       pbyte[3], pbyte[2], pbyte[1], pbyte[0]);
		}
	}

	if (displayed_info == TRUE) {
		if (argc > 3 && strcmp(argv[3], "help") == 0) {
			printf("\n\n"
			       "Capability Legend (Not all may be supported)\n"
			       "-----------------\n"
			       " I [ Infrastructure ]\n"
			       " A [ Ad-hoc ]\n"
			       " W [ WPA IE ]\n"
			       " 2 [ WPA2/RSN IE ]\n"
			       " M [ WMM IE ]\n"
			       " C [ Call Admission Control - WMM IE, VO ACM set ]\n"
			       " D [ Spectrum Management - DFS (11h) ]\n"
			       " K [ 11k ]\n"
			       " R [ 11r ]\n"
			       " S [ WPS ]\n"
			       " N [ HT (11n) ]\n"
			       " AC [VHT (11ac) ]\n" "\n\n");
		}
	} else {
		printf("< No Scan Results >\n");
	}

done:
	if (scan_rsp_buf)
		free(scan_rsp_buf);
	for (curr = scan_list_head; curr != NULL; curr = next) {
		next = curr->next;
		free(curr);
	}
	return ret;
}

/** Maximum channel scratch */
#define MAX_CHAN_SCRATCH  100

/** Maximum channel number for b/g band */
#define MAX_CHAN_BG_BAND  14

/** Maximum number of probes to send on each channel */
#define MAX_PROBES        4

/** Scan all the channels in specified band */
#define BAND_SPECIFIED    0x80
/**
 *  @brief Request a scan from the driver and display the scan table afterwards
 *
 *  Command line interface for performing a specific immediate scan based
 *    on the following keyword parsing:
 *
 *     chan=[chan#][band][mode] where band is [a,b,g,n] and mode is
 *                              blank for active or 'p' for passive
 *     bssid=xx:xx:xx:xx:xx:xx  specify a BSSID filter for the scan
 *     ssid="[SSID]"            specify a SSID filter for the scan
 *     keep=[0 or 1]            keep the previous scan results (1), discard (0)
 *     dur=[scan time]          time to scan for each channel in milliseconds
 *     probes=[#]               number of probe requests to send on each chan
 *     type=[1,2,3]             BSS type: 1 (Infra), 2(Adhoc), 3(Any)
 *
 *  Any combination of the above arguments can be supplied on the command line.
 *    If the chan token is absent, a full channel scan will be completed by
 *    the driver.  If the dur or probes tokens are absent, the drivers default
 *    setting will be used.  The bssid and ssid fields, if blank,
 *    will produce an unfiltered scan. The type field will default to 3 (Any)
 *    and the keep field will default to 0 (Discard).
 *
 *  @param argc     number of arguments
 *  @param argv     A pointer to arguments array
 *
 *  @return         MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_setuserscan(int argc, char *argv[])
{
	wlan_ioctl_user_scan_cfg scan_req;
	int ioctl_val, subioctl_val;
	struct iwreq iwr;
	char *parg_tok;
	char *pchan_tok;
	char *parg_cookie;
	char *pchan_cookie;
	int arg_idx;
	int chan_parse_idx;
	int chan_cmd_idx;
	char chan_scratch[MAX_CHAN_SCRATCH];
	char *pscratch;
	int tmp_idx;
	int scan_time;
	int num_ssid;
	int is_radio_set;
	unsigned int mac[ETH_ALEN];

	memset(&scan_req, 0x00, sizeof(scan_req));
	chan_cmd_idx = 0;
	scan_time = 0;
	num_ssid = 0;

	if (get_priv_ioctl("setuserscan",
			   &ioctl_val, &subioctl_val) == MLAN_STATUS_FAILURE) {
		return -EOPNOTSUPP;
	}

	for (arg_idx = 0; arg_idx < argc; arg_idx++) {

		if (strncmp(argv[arg_idx], "ssid=", strlen("ssid=")) == 0) {
			/*
			 *  "ssid" token string handler
			 */
			if (num_ssid < MRVDRV_MAX_SSID_LIST_LENGTH) {
				strncpy(scan_req.ssid_list[num_ssid].ssid,
					argv[arg_idx] + strlen("ssid="),
					sizeof(scan_req.ssid_list[num_ssid].
					       ssid));

				scan_req.ssid_list[num_ssid].max_len = 0;

				num_ssid++;
			}
		} else if (strncmp(argv[arg_idx], "bssid=", strlen("bssid=")) ==
			   0) {
			/*
			 *  "bssid" token string handler
			 */
			sscanf(argv[arg_idx] + strlen("bssid="),
			       "%2x:%2x:%2x:%2x:%2x:%2x", mac + 0, mac + 1,
			       mac + 2, mac + 3, mac + 4, mac + 5);

			for (tmp_idx = 0;
			     (unsigned int)tmp_idx < NELEMENTS(mac);
			     tmp_idx++) {
				scan_req.specific_bssid[tmp_idx] =
					(t_u8)mac[tmp_idx];
			}
		} else if (strncmp(argv[arg_idx], "chan=", strlen("chan=")) ==
			   0) {
			/*
			 *  "chan" token string handler
			 */
			parg_tok = argv[arg_idx] + strlen("chan=");

			if (strlen(parg_tok) > MAX_CHAN_SCRATCH) {
				printf("Error: Specified channels exceeds max limit\n");
				return MLAN_STATUS_FAILURE;
			}
			is_radio_set = FALSE;

			while ((parg_tok =
				strtok_r(parg_tok, ",",
					 &parg_cookie)) != NULL) {

				memset(chan_scratch, 0x00,
				       sizeof(chan_scratch));
				pscratch = chan_scratch;

				for (chan_parse_idx = 0;
				     (unsigned int)chan_parse_idx <
				     strlen(parg_tok); chan_parse_idx++) {
					if (isalpha
					    (*(parg_tok + chan_parse_idx))) {
						*pscratch++ = ' ';
					}

					*pscratch++ =
						*(parg_tok + chan_parse_idx);
				}
				*pscratch = 0;
				parg_tok = NULL;

				pchan_tok = chan_scratch;

				while ((pchan_tok = strtok_r(pchan_tok, " ",
							     &pchan_cookie)) !=
				       NULL) {
					if (isdigit(*pchan_tok)) {
						scan_req.
							chan_list[chan_cmd_idx].
							chan_number =
							atoi(pchan_tok);
						if (scan_req.
						    chan_list[chan_cmd_idx].
						    chan_number >
						    MAX_CHAN_BG_BAND)
							scan_req.
								chan_list
								[chan_cmd_idx].
								radio_type = 1;
					} else {
						switch (toupper(*pchan_tok)) {
						case 'A':
							scan_req.
								chan_list
								[chan_cmd_idx].
								radio_type = 1;
							is_radio_set = TRUE;
							break;
						case 'B':
						case 'G':
							scan_req.
								chan_list
								[chan_cmd_idx].
								radio_type = 0;
							is_radio_set = TRUE;
							break;
						case 'N':
							break;
						case 'P':
							scan_req.
								chan_list
								[chan_cmd_idx].
								scan_type =
								MLAN_SCAN_TYPE_PASSIVE;
							break;
						default:
							printf("Error: Band type not supported!\n");
							return -EOPNOTSUPP;
						}
						if (!chan_cmd_idx &&
						    !scan_req.
						    chan_list[chan_cmd_idx].
						    chan_number && is_radio_set)
							scan_req.
								chan_list
								[chan_cmd_idx].
								radio_type |=
								BAND_SPECIFIED;
					}
					pchan_tok = NULL;
				}
				chan_cmd_idx++;
			}
		} else if (strncmp(argv[arg_idx], "keep=", strlen("keep=")) ==
			   0) {
			/*
			 *  "keep" token string handler
			 */
			scan_req.keep_previous_scan =
				atoi(argv[arg_idx] + strlen("keep="));
		} else if (strncmp(argv[arg_idx], "dur=", strlen("dur=")) == 0) {
			/*
			 *  "dur" token string handler
			 */
			scan_time = atoi(argv[arg_idx] + strlen("dur="));
			scan_req.chan_list[0].scan_time = scan_time;

		} else if (strncmp(argv[arg_idx], "wc=", strlen("wc=")) == 0) {

			if (num_ssid < MRVDRV_MAX_SSID_LIST_LENGTH) {
				/*
				 *  "wc" token string handler
				 */
				pscratch = strrchr(argv[arg_idx], ',');

				if (pscratch) {
					*pscratch = 0;
					pscratch++;

					if (isdigit(*pscratch)) {
						scan_req.ssid_list[num_ssid].
							max_len =
							atoi(pscratch);
					} else {
						scan_req.ssid_list[num_ssid].
							max_len = *pscratch;
					}
				} else {
					/* Standard wildcard matching */
					scan_req.ssid_list[num_ssid].max_len =
						0xFF;
				}

				strncpy(scan_req.ssid_list[num_ssid].ssid,
					argv[arg_idx] + strlen("wc="),
					sizeof(scan_req.ssid_list[num_ssid].
					       ssid));

				num_ssid++;
			}
		} else if (strncmp(argv[arg_idx], "probes=", strlen("probes="))
			   == 0) {
			/*
			 *  "probes" token string handler
			 */
			scan_req.num_probes =
				atoi(argv[arg_idx] + strlen("probes="));
			if (scan_req.num_probes > MAX_PROBES) {
				fprintf(stderr, "Invalid probes (> %d)\n",
					MAX_PROBES);
				return -EOPNOTSUPP;
			}
		} else if (strncmp(argv[arg_idx], "type=", strlen("type=")) ==
			   0) {
			/*
			 *  "type" token string handler
			 */
			scan_req.bss_mode =
				atoi(argv[arg_idx] + strlen("type="));
			switch (scan_req.bss_mode) {
			case MLAN_SCAN_MODE_BSS:
			case MLAN_SCAN_MODE_IBSS:
				break;
			case MLAN_SCAN_MODE_ANY:
			default:
				/* Set any unknown types to ANY */
				scan_req.bss_mode = MLAN_SCAN_MODE_ANY;
				break;
			}
		}
	}

	/*
	 * Update all the channels to have the same scan time
	 */
	for (tmp_idx = 1; tmp_idx < chan_cmd_idx; tmp_idx++) {
		scan_req.chan_list[tmp_idx].scan_time = scan_time;
	}

	strncpy(iwr.ifr_name, dev_name, IFNAMSIZ - 1);
	iwr.u.data.pointer = (caddr_t) & scan_req;
	iwr.u.data.length = sizeof(scan_req);
	iwr.u.data.flags = subioctl_val;

	if (ioctl(sockfd, ioctl_val, &iwr) < 0) {
		perror("mlanconfig: setuserscan ioctl");
		return -EFAULT;
	}

	process_getscantable(0, 0);

	return MLAN_STATUS_SUCCESS;
}
