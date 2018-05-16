/*
 * Copyright (c) 2016-2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: wma_nan_datapath.c
 *
 * WMA NAN Data path API implementation
 */

#include "wma.h"
#include "wma_api.h"
#include "vos_api.h"
#include "wmi_unified_api.h"
#include "wmi_unified.h"
#include "wma_nan_datapath.h"

/**
 * wma_handle_ndp_initiator_req() - NDP initiator request handler
 * @wma_handle: wma handle
 * @req: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS wma_handle_ndp_initiator_req(tp_wma_handle wma_handle, void *req)
{
	VOS_STATUS status;
	int ret;
	uint16_t len;
	uint32_t vdev_id, ndp_cfg_len, ndp_app_info_len, pmk_len;
	struct ndp_initiator_rsp *rsp = NULL;
	ol_txrx_vdev_handle vdev;
	wmi_buf_t buf;
	wmi_ndp_initiator_req_fixed_param *cmd;
	vos_msg_t pe_msg = {0};
	struct ndp_initiator_req *ndp_req = req;
	wmi_channel *ch_tlv;
	uint8_t *tlv_ptr;

	if (NULL == ndp_req) {
		WMA_LOGE(FL("Invalid ndp_req."));
		goto send_ndp_initiator_fail;
	}
	vdev_id = ndp_req->vdev_id;
	vdev = wma_find_vdev_by_id(wma_handle, vdev_id);
	if (!vdev) {
		WMA_LOGE(FL("vdev not found for vdev id %d."), vdev_id);
		goto send_ndp_initiator_fail;
	}

	if (!WMA_IS_VDEV_IN_NDI_MODE(wma_handle->interfaces, vdev_id)) {
		WMA_LOGE(FL("vdev :%d, not in NDI mode"), vdev_id);
		goto send_ndp_initiator_fail;
	}

	/*
	 * WMI command expects 4 byte alligned len:
	 * round up ndp_cfg_len and ndp_app_info_len to 4 bytes
	 */
	ndp_cfg_len = vos_roundup(ndp_req->ndp_config.ndp_cfg_len, 4);
	ndp_app_info_len = vos_roundup(ndp_req->ndp_info.ndp_app_info_len, 4);
	pmk_len = vos_roundup(ndp_req->pmk.pmk_len, 4);
	/* allocated memory for fixed params as well as variable size data */
	len = sizeof(*cmd) + sizeof(*ch_tlv) + (3 * WMI_TLV_HDR_SIZE)
		+ ndp_cfg_len + ndp_app_info_len + pmk_len;

	buf = wmi_buf_alloc(wma_handle->wmi_handle, len);
	if (!buf) {
		WMA_LOGE(FL("wmi_buf_alloc failed"));
		goto send_ndp_initiator_fail;
	}
	cmd = (wmi_ndp_initiator_req_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_ndp_initiator_req_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
				wmi_ndp_initiator_req_fixed_param));
	cmd->vdev_id = ndp_req->vdev_id;
	cmd->transaction_id = ndp_req->transaction_id;
	cmd->service_instance_id = ndp_req->service_instance_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(ndp_req->peer_discovery_mac_addr.bytes,
				   &cmd->peer_discovery_mac_addr);

	cmd->ndp_cfg_len = ndp_req->ndp_config.ndp_cfg_len;
	cmd->ndp_app_info_len = ndp_req->ndp_info.ndp_app_info_len;
	cmd->ndp_channel_cfg = ndp_req->channel_cfg;
	cmd->nan_pmk_len = ndp_req->pmk.pmk_len;
	cmd->nan_csid = ndp_req->ncs_sk_type;

	ch_tlv = (wmi_channel *)&cmd[1];
	WMITLV_SET_HDR(ch_tlv, WMITLV_TAG_STRUC_wmi_channel,
			WMITLV_GET_STRUCT_TLVLEN(wmi_channel));
	ch_tlv->mhz = ndp_req->channel;
	ch_tlv->band_center_freq1 =
		vos_chan_to_freq(vos_freq_to_chan(ndp_req->channel));
	tlv_ptr = (uint8_t *)&ch_tlv[1];

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, ndp_cfg_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE],
		     ndp_req->ndp_config.ndp_cfg, cmd->ndp_cfg_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + ndp_cfg_len;

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, ndp_app_info_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE],
		     ndp_req->ndp_info.ndp_app_info, cmd->ndp_app_info_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + ndp_app_info_len;

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, pmk_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE], ndp_req->pmk.pmk,
		     cmd->nan_pmk_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + pmk_len;

	WMA_LOGE(FL("vdev_id = %d, transaction_id: %d, service_instance_id: %d, ch: %d, ch_cfg: %d, csid: %d"),
		cmd->vdev_id, cmd->transaction_id, cmd->service_instance_id,
		ch_tlv->mhz, cmd->ndp_channel_cfg, cmd->nan_csid);
	WMA_LOGE(FL("peer mac addr: mac_addr31to0: 0x%x, mac_addr47to32: 0x%x"),
		cmd->peer_discovery_mac_addr.mac_addr31to0,
		cmd->peer_discovery_mac_addr.mac_addr47to32);

	WMA_LOGE(FL("ndp_config len: %d"), cmd->ndp_cfg_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			   ndp_req->ndp_config.ndp_cfg,
			   ndp_req->ndp_config.ndp_cfg_len);

	WMA_LOGE(FL("ndp_app_info len: %d"), cmd->ndp_app_info_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			   ndp_req->ndp_info.ndp_app_info,
			   ndp_req->ndp_info.ndp_app_info_len);

	WMA_LOGE(FL("pmk len: %d"), cmd->nan_pmk_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			   ndp_req->pmk.pmk, cmd->nan_pmk_len);
	WMA_LOGE(FL("sending WMI_NDP_INITIATOR_REQ_CMDID(0x%X)"),
		WMI_NDP_INITIATOR_REQ_CMDID);

	ret = wmi_unified_cmd_send(wma_handle->wmi_handle, buf, len,
				   WMI_NDP_INITIATOR_REQ_CMDID);
	if (ret < 0) {
		WMA_LOGE(FL("WMI_NDP_INITIATOR_REQ_CMDID failed, ret: %d"),
			ret);
		wmi_buf_free(buf);
		goto send_ndp_initiator_fail;
	}

	return VOS_STATUS_SUCCESS;

send_ndp_initiator_fail:
	status = VOS_STATUS_E_FAILURE;
	if (ndp_req) {
		rsp = vos_mem_malloc(sizeof(*rsp));
		if (NULL == rsp) {
			WMA_LOGE(FL("Memory allocation failure"));
			status = VOS_STATUS_E_NOMEM;
			/* unblock SME queue, but do not send rsp to HDD */
			pe_msg.bodyval = true;
		} else {
			rsp->vdev_id = ndp_req->vdev_id;
			rsp->transaction_id = ndp_req->transaction_id;
			rsp->ndp_instance_id = ndp_req->service_instance_id;
			rsp->status = NDP_RSP_STATUS_ERROR;
			rsp->reason = NDP_DATA_INITIATOR_REQ_FAILED;
		}
	} else {
		/* unblock SME queue, but do not send rsp to HDD */
		pe_msg.bodyval = true;
	}

	pe_msg.type = SIR_HAL_NDP_INITIATOR_RSP;
	pe_msg.bodyptr = rsp;

	if (VOS_STATUS_SUCCESS !=
			vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg)) {
		WMA_LOGE("SIR_HAL_NDP_INITIATOR_RSP to PE failed");
		vos_mem_free(rsp);
	}

	return status;
}

/**
 * wma_send_ndp_responder_rsp() - NDP responder request handler
 * @rsp_ind: rsp parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
static VOS_STATUS wma_send_ndp_responder_rsp
	(struct ndp_responder_rsp_event *rsp_ind)
{
	VOS_STATUS status;
	vos_msg_t pe_msg = {0};
	uint8_t len = sizeof(*rsp_ind);

	if (NULL == rsp_ind) {
		WMA_LOGE(FL("Invalid rsp_ind"));
		/* msg to unblock SME, but not send rsp to HDD */
		pe_msg.bodyptr = NULL;
		pe_msg.bodyval = true;
		goto send_rsp;
	}
	pe_msg.bodyptr = vos_mem_malloc(len);
	if (NULL == pe_msg.bodyptr) {
		WMA_LOGE(FL("Memory allocation failure"));
		/* msg to unblock SME, but not send rsp to HDD */
		pe_msg.bodyval = true;
	} else {
		vos_mem_copy(pe_msg.bodyptr, rsp_ind, len);
		pe_msg.bodyval = 0;
	}
send_rsp:
	pe_msg.type = SIR_HAL_NDP_RESPONDER_RSP;
	status = vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg);
	if (!VOS_IS_STATUS_SUCCESS(status)) {
		WMA_LOGE(FL("fail to post msg to PE"));
		vos_mem_free(pe_msg.bodyptr);
	}
	return status;
}

/**
 * wma_handle_ndp_responder_req() - NDP responder request handler
 * @wma_handle: wma handle
 * @req_params: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS wma_handle_ndp_responder_req(tp_wma_handle wma_handle,
					struct ndp_responder_req *req_params)
{
	wmi_buf_t buf;
	ol_txrx_vdev_handle vdev;
	uint32_t vdev_id = 0, ndp_cfg_len, ndp_app_info_len, pmk_len;
	uint8_t *tlv_ptr;
	int ret;
	wmi_ndp_responder_req_fixed_param *cmd;
	uint16_t len;
	struct ndp_responder_rsp_event rsp;

	if (NULL == req_params) {
		WMA_LOGE(FL("Invalid req_params."));
		return VOS_STATUS_E_INVAL;
	}

	vdev_id = req_params->vdev_id;
	WMA_LOGD(FL("vdev_id: %d, transaction_id: %d, ndp_rsp %d, ndp_instance_id: %d, ndp_app_info_len: %d"),
			req_params->vdev_id, req_params->transaction_id,
			req_params->ndp_rsp,
			req_params->ndp_instance_id,
			req_params->ndp_info.ndp_app_info_len);
	vdev = wma_find_vdev_by_id(wma_handle, vdev_id);
	if (!vdev) {
		WMA_LOGE(FL("vdev not found for vdev id %d."), vdev_id);
		goto send_ndp_responder_fail;
	}

	if (!WMA_IS_VDEV_IN_NDI_MODE(wma_handle->interfaces, vdev_id)) {
		WMA_LOGE(FL("vdev :$%d, not in NDI mode"), vdev_id);
		goto send_ndp_responder_fail;
	}

	/*
	 * WMI command expects 4 byte alligned len:
	 * round up ndp_cfg_len and ndp_app_info_len to 4 bytes
	 */
	ndp_cfg_len = vos_roundup(req_params->ndp_config.ndp_cfg_len, 4);
	ndp_app_info_len = vos_roundup(req_params->ndp_info.ndp_app_info_len, 4);
	pmk_len = vos_roundup(req_params->pmk.pmk_len, 4);
	/* allocated memory for fixed params as well as variable size data */
	len = sizeof(*cmd) + 3*WMI_TLV_HDR_SIZE + ndp_cfg_len + ndp_app_info_len
		+ pmk_len;

	buf = wmi_buf_alloc(wma_handle->wmi_handle, len);
	if (!buf) {
		WMA_LOGE(FL("wmi_buf_alloc failed"));
		goto send_ndp_responder_fail;
	}
	cmd = (wmi_ndp_responder_req_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_ndp_responder_req_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
				wmi_ndp_responder_req_fixed_param));
	cmd->vdev_id = req_params->vdev_id;
	cmd->transaction_id = req_params->transaction_id;
	cmd->ndp_instance_id = req_params->ndp_instance_id;
	cmd->rsp_code = req_params->ndp_rsp;

	cmd->ndp_cfg_len = req_params->ndp_config.ndp_cfg_len;
	cmd->ndp_app_info_len = req_params->ndp_info.ndp_app_info_len;
	cmd->nan_pmk_len = req_params->pmk.pmk_len;
	cmd->nan_csid = req_params->ncs_sk_type;

	tlv_ptr = (uint8_t *)&cmd[1];

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, ndp_cfg_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE],
		req_params->ndp_config.ndp_cfg, cmd->ndp_cfg_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + ndp_cfg_len;

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, ndp_app_info_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE],
		     req_params->ndp_info.ndp_app_info,
		     req_params->ndp_info.ndp_app_info_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + ndp_app_info_len;

	WMITLV_SET_HDR(tlv_ptr, WMITLV_TAG_ARRAY_BYTE, pmk_len);
	vos_mem_copy(&tlv_ptr[WMI_TLV_HDR_SIZE], req_params->pmk.pmk,
		     cmd->nan_pmk_len);
	tlv_ptr = tlv_ptr + WMI_TLV_HDR_SIZE + pmk_len;

	WMA_LOGE(FL("vdev_id = %d, transaction_id: %d, csid: %d"),
		cmd->vdev_id, cmd->transaction_id, cmd->nan_csid);

	WMA_LOGD(FL("ndp_config len: %d"),
		req_params->ndp_config.ndp_cfg_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			req_params->ndp_config.ndp_cfg,
			req_params->ndp_config.ndp_cfg_len);

	WMA_LOGD(FL("ndp_app_info len: %d"),
		req_params->ndp_info.ndp_app_info_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			req_params->ndp_info.ndp_app_info,
			req_params->ndp_info.ndp_app_info_len);

	WMA_LOGE(FL("pmk len: %d"), cmd->nan_pmk_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			   req_params->pmk.pmk, cmd->nan_pmk_len);

	WMA_LOGE(FL("sending WMI_NDP_RESPONDER_REQ_CMDID(0x%X)"),
		WMI_NDP_RESPONDER_REQ_CMDID);
	ret = wmi_unified_cmd_send(wma_handle->wmi_handle, buf, len,
				   WMI_NDP_RESPONDER_REQ_CMDID);
	if (ret < 0) {
		WMA_LOGE(FL("WMI_NDP_RESPONDER_REQ_CMDID failed, ret: %d"),
			ret);
		wmi_buf_free(buf);
		goto send_ndp_responder_fail;
	}
	return VOS_STATUS_SUCCESS;
send_ndp_responder_fail:
	vos_mem_zero(&rsp, sizeof(rsp));
	rsp.vdev_id = req_params->vdev_id;
	rsp.transaction_id = req_params->transaction_id;
	rsp.status = NDP_RSP_STATUS_ERROR;
	rsp.reason = NDP_DATA_RESPONDER_REQ_FAILED;
	wma_send_ndp_responder_rsp(&rsp);
	return VOS_STATUS_E_FAILURE;
}

/**
 * wma_handle_ndp_end_req() - NDP end request handler
 * @wma_handle: wma handle
 * @ptr: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS wma_handle_ndp_end_req(tp_wma_handle wma_handle, void *ptr)
{
	int ret;
	uint16_t len;
	uint32_t ndp_end_req_len, i;
	wmi_ndp_end_req *ndp_end_req_lst;
	wmi_buf_t buf;
	vos_msg_t pe_msg = {0};
	wmi_ndp_end_req_fixed_param *cmd;
	struct ndp_end_rsp_event *end_rsp = NULL;
	struct ndp_end_req *req = ptr;

	if (NULL == req) {
		WMA_LOGE(FL("Invalid ndp_end_req"));
		goto send_ndp_end_fail;
	}

	/* len of tlv following fixed param  */
	ndp_end_req_len = sizeof(wmi_ndp_end_req) * req->num_ndp_instances;
	/* above comes out to 4 byte alligned already, no need of padding */
	len = sizeof(*cmd) + ndp_end_req_len + WMI_TLV_HDR_SIZE;
	buf = wmi_buf_alloc(wma_handle->wmi_handle, len);
	if (!buf) {
		WMA_LOGE(FL("Malloc failed"));
		return VOS_STATUS_E_NOMEM;
	}
	cmd = (wmi_ndp_end_req_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_ndp_end_req_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_ndp_end_req_fixed_param));

	cmd->transaction_id = req->transaction_id;

	/* set tlv pointer to end of fixed param */
	WMITLV_SET_HDR((uint8_t *)&cmd[1], WMITLV_TAG_ARRAY_STRUC,
			ndp_end_req_len);

	ndp_end_req_lst = (wmi_ndp_end_req *)((uint8_t *)&cmd[1] +
						WMI_TLV_HDR_SIZE);
	for (i = 0; i < req->num_ndp_instances; i++) {
		WMITLV_SET_HDR(&ndp_end_req_lst[i],
				WMITLV_TAG_ARRAY_FIXED_STRUC,
				(sizeof(*ndp_end_req_lst) - WMI_TLV_HDR_SIZE));

		ndp_end_req_lst[i].ndp_instance_id = req->ndp_ids[i];
	}

	WMA_LOGD(FL("Sending WMI_NDP_END_REQ_CMDID to FW"));
	ret = wmi_unified_cmd_send(wma_handle->wmi_handle, buf, len,
				   WMI_NDP_END_REQ_CMDID);
	if (ret < 0) {
		WMA_LOGE(FL("WMI_NDP_END_REQ_CMDID failed, ret: %d"), ret);
		wmi_buf_free(buf);
		goto send_ndp_end_fail;
	}
	return VOS_STATUS_SUCCESS;

send_ndp_end_fail:
	pe_msg.type = SIR_HAL_NDP_END_RSP;
	if (req) {
		end_rsp = vos_mem_malloc(sizeof(*end_rsp));
		if (NULL == end_rsp) {
			WMA_LOGE(FL("Malloc failed"));
			pe_msg.bodyval = true;
		} else {
			vos_mem_zero(end_rsp, sizeof(*end_rsp));
			end_rsp->status = NDP_RSP_STATUS_ERROR;
			end_rsp->reason = NDP_END_FAILED;
			end_rsp->transaction_id = req->transaction_id;
			pe_msg.bodyptr = end_rsp;
		}
	} else {
		pe_msg.bodyval = true;
	}

	if (VOS_STATUS_SUCCESS !=
		vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg)) {
		WMA_LOGE("NDP_END_RSP to PE failed");
		vos_mem_free(end_rsp);
	}
	return VOS_STATUS_E_FAILURE;
}

/**
 * wma_handle_ndp_sched_update_req() - NDP schedule update request handler
 * @wma_handle: wma handle
 * @req_params: request parameters
 *
 * Return: VOS_STATUS_SUCCESS on success; error number otherwise
 */
VOS_STATUS wma_handle_ndp_sched_update_req(tp_wma_handle wma_handle,
					struct ndp_end_req *req_params)
{
	return VOS_STATUS_SUCCESS;
}

/**
 * wma_ndp_indication_event_handler() - NDP indication event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_INDICATION_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_indication_event_handler(void *handle, uint8_t  *event_info,
					    uint32_t len)
{
	WMI_NDP_INDICATION_EVENTID_param_tlvs *event;
	wmi_ndp_indication_event_fixed_param *fixed_params;
	vos_msg_t pe_msg = {0};
	struct ndp_indication_event *ind_event;
	VOS_STATUS status;

	event = (WMI_NDP_INDICATION_EVENTID_param_tlvs *)event_info;
	fixed_params =
		(wmi_ndp_indication_event_fixed_param *)event->fixed_param;

	if (fixed_params->ndp_cfg_len > event->num_ndp_cfg) {
		WMA_LOGE("FW message ndp cfg length %d larger than TLV hdr %d",
			 fixed_params->ndp_cfg_len, event->num_ndp_cfg);
		return -EINVAL;
	}

	if (fixed_params->ndp_app_info_len > event->num_ndp_app_info) {
		WMA_LOGE("FW message ndp app info length %d more than TLV hdr %d",
			 fixed_params->ndp_app_info_len, event->num_ndp_app_info);
		return -EINVAL;
	}

	ind_event = vos_mem_malloc(sizeof(*ind_event));
	if (!ind_event) {
		WMA_LOGP(FL("Failed to allocate memory"));
		return VOS_STATUS_E_NOMEM;
	}
	vos_mem_zero(ind_event, sizeof(*ind_event));
	ind_event->vdev_id = fixed_params->vdev_id;
	ind_event->service_instance_id = fixed_params->service_instance_id;
	ind_event->ndp_instance_id = fixed_params->ndp_instance_id;
	ind_event->role = fixed_params->self_ndp_role;
	ind_event->policy = fixed_params->accept_policy;

	WMI_MAC_ADDR_TO_CHAR_ARRAY(&fixed_params->peer_ndi_mac_addr,
				ind_event->peer_mac_addr.bytes);
	WMI_MAC_ADDR_TO_CHAR_ARRAY(&fixed_params->peer_discovery_mac_addr,
				ind_event->peer_discovery_mac_addr.bytes);

	WMA_LOGD(FL("WMI_NDP_INDICATION_EVENTID(0x%X) received. vdev %d, \n"
		"service_instance %d, ndp_instance %d, role %d, policy %d, \n"
		"csid: %d, scid_len: %d, peer_mac_addr: %pM, peer_disc_mac_addr: %pM"),
		 WMI_NDP_INDICATION_EVENTID, fixed_params->vdev_id,
		 fixed_params->service_instance_id,
		 fixed_params->ndp_instance_id, fixed_params->self_ndp_role,
		 fixed_params->accept_policy,
		 fixed_params->nan_csid, fixed_params->nan_scid_len,
		 ind_event->peer_mac_addr.bytes,
		 ind_event->peer_discovery_mac_addr.bytes);

	WMA_LOGD(FL("ndp_cfg - %d bytes"), fixed_params->ndp_cfg_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			   &event->ndp_cfg, fixed_params->ndp_cfg_len);

	WMA_LOGD(FL("ndp_app_info - %d bytes"), fixed_params->ndp_app_info_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			&event->ndp_app_info, fixed_params->ndp_app_info_len);
	ind_event->ndp_config.ndp_cfg_len = fixed_params->ndp_cfg_len;
	ind_event->ndp_info.ndp_app_info_len = fixed_params->ndp_app_info_len;
	ind_event->ncs_sk_type = fixed_params->nan_csid;
	ind_event->scid.scid_len = fixed_params->nan_scid_len;

	if (ind_event->ndp_config.ndp_cfg_len) {
		ind_event->ndp_config.ndp_cfg =
			vos_mem_malloc(fixed_params->ndp_cfg_len);
		if (NULL == ind_event->ndp_config.ndp_cfg) {
			WMA_LOGE(FL("malloc failed"));
			vos_mem_free(ind_event);
			return VOS_STATUS_E_NOMEM;
		}
		vos_mem_copy(ind_event->ndp_config.ndp_cfg, event->ndp_cfg,
			     ind_event->ndp_config.ndp_cfg_len);
	}

	if (ind_event->ndp_info.ndp_app_info_len) {
		ind_event->ndp_info.ndp_app_info =
			vos_mem_malloc(ind_event->ndp_info.ndp_app_info_len);
		if (NULL == ind_event->ndp_info.ndp_app_info) {
			WMA_LOGE(FL("malloc failed"));
			vos_mem_free(ind_event->ndp_config.ndp_cfg);
			vos_mem_free(ind_event);
			return VOS_STATUS_E_NOMEM;
		}
		vos_mem_copy(ind_event->ndp_info.ndp_app_info,
			     event->ndp_app_info,
			     ind_event->ndp_info.ndp_app_info_len);
	}

	if (ind_event->scid.scid_len) {
		ind_event->scid.scid =
			vos_mem_malloc(ind_event->scid.scid_len);
		if (NULL == ind_event->scid.scid) {
			WMA_LOGE(FL("malloc failed"));
			vos_mem_free(ind_event->ndp_config.ndp_cfg);
			vos_mem_free(ind_event->ndp_info.ndp_app_info);
			vos_mem_free(ind_event);
			return VOS_STATUS_E_NOMEM;
		}
		vos_mem_copy(ind_event->scid.scid,
			     event->ndp_scid, ind_event->scid.scid_len);
		WMA_LOGD(FL("scid hex dump:"));
		VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
			ind_event->scid.scid, ind_event->scid.scid_len);
	}

	pe_msg.type = SIR_HAL_NDP_INDICATION;
	pe_msg.bodyptr = ind_event;
	pe_msg.bodyval = 0;

	WMA_LOGE(FL("Sending SIR_HAL_NDP_INDICATION msg to PE"));
	status = vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg);
	if (!VOS_IS_STATUS_SUCCESS(status)) {
		WMA_LOGE(FL("fail to post SIR_HAL_NDP_INDICATION msg to PE"));
		vos_mem_free(ind_event->ndp_config.ndp_cfg);
		vos_mem_free(ind_event->ndp_info.ndp_app_info);
		vos_mem_free(ind_event->scid.scid);
		vos_mem_free(ind_event);
	}

	return status;
}

/**
 * wma_ndp_responder_rsp_event_handler() - NDP responder response event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_RESPONDER_RSP_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_responder_rsp_event_handler(void *handle,
	uint8_t *event_info, uint32_t len)
{
	WMI_NDP_RESPONDER_RSP_EVENTID_param_tlvs *event;
	wmi_ndp_responder_rsp_event_fixed_param  *fixed_params;
	struct ndp_responder_rsp_event rsp;

	event = (WMI_NDP_RESPONDER_RSP_EVENTID_param_tlvs *)event_info;
	fixed_params = event->fixed_param;

	rsp.vdev_id = fixed_params->vdev_id;
	rsp.transaction_id = fixed_params->transaction_id;
	rsp.reason = fixed_params->reason_code;
	rsp.status = fixed_params->rsp_status;
	rsp.create_peer = fixed_params->create_peer;
	WMI_MAC_ADDR_TO_CHAR_ARRAY(&fixed_params->peer_ndi_mac_addr,
				rsp.peer_mac_addr.bytes);

	WMA_LOGE(FL("WMI_NDP_RESPONDER_RSP_EVENTID(0x%X) received. vdev_id: %d, peer_mac_addr: %pM, transaction_id: %d, status_code %d, reason_code: %d, create_peer: %d"),
			WMI_NDP_RESPONDER_RSP_EVENTID, rsp.vdev_id,
			rsp.peer_mac_addr.bytes, rsp.transaction_id,
			rsp.status, rsp.reason, rsp.create_peer);

	return wma_send_ndp_responder_rsp(&rsp);
}

/**
 * wma_ndp_confirm_event_handler() - NDP confirm event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_CONFIRM_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_confirm_event_handler(void *handle, uint8_t *event_info,
					 uint32_t len)
{
	struct ndp_confirm_event *ndp_confirm;
	vos_msg_t msg = {0};
	WMI_NDP_CONFIRM_EVENTID_param_tlvs *event;
	wmi_ndp_confirm_event_fixed_param *fixed_params;
	VOS_STATUS status;

	event = (WMI_NDP_CONFIRM_EVENTID_param_tlvs *) event_info;
	fixed_params = (wmi_ndp_confirm_event_fixed_param *)event->fixed_param;
	WMA_LOGE(FL("WMI_NDP_CONFIRM_EVENTID(0x%X) recieved. vdev %d, ndp_instance %d, rsp_code %d, reason_code: %d, num_active_ndps_on_peer: %d"),
		 WMI_NDP_CONFIRM_EVENTID, fixed_params->vdev_id,
		 fixed_params->ndp_instance_id, fixed_params->rsp_code,
		 fixed_params->reason_code,
		 fixed_params->num_active_ndps_on_peer);

	if (fixed_params->ndp_cfg_len > event->num_ndp_cfg) {
		WMA_LOGE("FW message ndp cfg length %d larger than TLV hdr %d",
			 fixed_params->ndp_cfg_len, event->num_ndp_cfg);
		return -EINVAL;
	}
	WMA_LOGE(FL("ndp_cfg - %d bytes"), fixed_params->ndp_cfg_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
		&event->ndp_cfg, fixed_params->ndp_cfg_len);

	if (fixed_params->ndp_app_info_len > event->num_ndp_app_info) {
		WMA_LOGE("FW message ndp app info length %d more than TLV hdr %d",
			 fixed_params->ndp_app_info_len, event->num_ndp_app_info);
		return -EINVAL;
	}
	WMA_LOGE(FL("ndp_app_info - %d bytes"), fixed_params->ndp_app_info_len);
	VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_DEBUG,
		&event->ndp_app_info, fixed_params->ndp_app_info_len);

	ndp_confirm = vos_mem_malloc(sizeof(*ndp_confirm));
	if (!ndp_confirm) {
		WMA_LOGP(FL("Failed to allocate memory"));
		return VOS_STATUS_E_NOMEM;
	}
	vos_mem_zero(ndp_confirm, sizeof(*ndp_confirm));

	ndp_confirm->vdev_id = fixed_params->vdev_id;
	ndp_confirm->ndp_instance_id = fixed_params->ndp_instance_id;
	ndp_confirm->rsp_code = fixed_params->rsp_code;
	ndp_confirm->reason_code = fixed_params->reason_code;
	ndp_confirm->num_active_ndps_on_peer =
				fixed_params->num_active_ndps_on_peer;

	WMI_MAC_ADDR_TO_CHAR_ARRAY(&fixed_params->peer_ndi_mac_addr,
				   ndp_confirm->peer_ndi_mac_addr.bytes);

	ndp_confirm->ndp_info.ndp_app_info_len = fixed_params->ndp_app_info_len;
	if (ndp_confirm->ndp_info.ndp_app_info_len) {
		ndp_confirm->ndp_info.ndp_app_info =
				vos_mem_malloc(fixed_params->ndp_app_info_len);
		if (NULL == ndp_confirm->ndp_info.ndp_app_info) {
			WMA_LOGE(FL("malloc failed"));
			vos_mem_free(ndp_confirm);
			return VOS_STATUS_E_NOMEM;
		}
		vos_mem_copy(&ndp_confirm->ndp_info.ndp_app_info,
			     event->ndp_app_info,
			     ndp_confirm->ndp_info.ndp_app_info_len);
	}
	msg.type = SIR_HAL_NDP_CONFIRM;
	msg.bodyptr = ndp_confirm;
	msg.bodyval = 0;
	WMA_LOGE(FL("Sending SIR_HAL_NDP_CONFIRM msg to PE"));
	status = vos_mq_post_message(VOS_MODULE_ID_PE, &msg);
	if (!VOS_IS_STATUS_SUCCESS(status)) {
		WMA_LOGE(FL("fail to post SIR_HAL_NDP_CONFIRM msg to PE"));
		vos_mem_free(ndp_confirm->ndp_info.ndp_app_info);
		vos_mem_free(ndp_confirm);
	}

	return VOS_STATUS_SUCCESS;
}

/**
 * wma_ndp_end_response_event_handler() - NDP end response event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_END_RSP_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_end_response_event_handler(void *handle,
	uint8_t  *event_info, uint32_t len)
{
	int ret = 0;
	VOS_STATUS status;
	vos_msg_t pe_msg = {0};
	struct ndp_end_rsp_event *end_rsp;
	WMI_NDP_END_RSP_EVENTID_param_tlvs *event;
	wmi_ndp_end_rsp_event_fixed_param *fixed_params = NULL;

	event = (WMI_NDP_END_RSP_EVENTID_param_tlvs *) event_info;
	fixed_params = (wmi_ndp_end_rsp_event_fixed_param *)event->fixed_param;
	WMA_LOGD(FL("WMI_NDP_END_RSP_EVENTID(0x%X) recieved. transaction_id: %d, rsp_status: %d, reason_code: %d"),
		 WMI_NDP_END_RSP_EVENTID, fixed_params->transaction_id,
		 fixed_params->rsp_status, fixed_params->reason_code);

	end_rsp = vos_mem_malloc(sizeof(*end_rsp));
	if (NULL == end_rsp) {
		WMA_LOGE("malloc failed");
		pe_msg.bodyval = true;
		ret = -ENOMEM;
		goto send_ndp_end_rsp;
	}
	pe_msg.bodyptr = end_rsp;
	vos_mem_zero(end_rsp, sizeof(*end_rsp));

	end_rsp->transaction_id = fixed_params->transaction_id;
	end_rsp->reason = fixed_params->reason_code;
	end_rsp->status = fixed_params->rsp_status;

send_ndp_end_rsp:
	pe_msg.type = SIR_HAL_NDP_END_RSP;
	WMA_LOGD(FL("Sending SIR_HAL_NDP_END_RSP msg to PE"));
	status = vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg);
	if (!VOS_IS_STATUS_SUCCESS(status)) {
		WMA_LOGE("SIR_HAL_NDP_END_RSP to PE failed");
		vos_mem_free(end_rsp);
		ret = -EINVAL;
	}

	return ret;
}

/**
 * wma_ndp_end_indication_event_handler() - NDP end indication event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_END_INDICATION_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_end_indication_event_handler(void *handle,
	uint8_t  *event_info, uint32_t len)
{
	WMI_NDP_END_INDICATION_EVENTID_param_tlvs *event;
	wmi_ndp_end_indication *ind;
	vos_msg_t pe_msg;
	struct ndp_end_indication_event *ndp_event_buf;
	VOS_STATUS vos_status;
	int i;
	v_MACADDR_t peer_addr;
	int buf_size;

	event = (WMI_NDP_END_INDICATION_EVENTID_param_tlvs *) event_info;

	if (event->num_ndp_end_indication_list == 0) {
		WMA_LOGE(
			FL("Error: Event ignored, 0 ndp instances"));
		return -EINVAL;
	}

	WMA_LOGD(FL("number of ndp instances = %d"),
		event->num_ndp_end_indication_list);
	if (event->num_ndp_end_indication_list > ((WMA_SVC_MSG_MAX_SIZE -
		sizeof(*ndp_event_buf)) / sizeof(ndp_event_buf->ndp_map[0]))) {
			WMA_LOGE("%s: excess data received from fw num_ndp_end_indication_list %d",
				__func__, event->num_ndp_end_indication_list);
			return -EINVAL;
	}
	buf_size = sizeof(*ndp_event_buf) + event->num_ndp_end_indication_list *
			sizeof(ndp_event_buf->ndp_map[0]);
	ndp_event_buf = vos_mem_malloc(buf_size);
	if (!ndp_event_buf) {
		WMA_LOGP(FL("Failed to allocate memory"));
		return -ENOMEM;
	}
	vos_mem_zero(ndp_event_buf, buf_size);
	ndp_event_buf->num_ndp_ids = event->num_ndp_end_indication_list;

	ind = event->ndp_end_indication_list;
	for (i = 0; i < ndp_event_buf->num_ndp_ids; i++) {
		WMI_MAC_ADDR_TO_CHAR_ARRAY(
			&ind[i].peer_ndi_mac_addr,
			peer_addr.bytes);
		WMA_LOGD(
			FL("ind[%d]: type %d, reason_code %d, instance_id %d num_active %d MAC: " MAC_ADDRESS_STR),
			i,
			ind[i].type,
			ind[i].reason_code,
			ind[i].ndp_instance_id,
			ind[i].num_active_ndps_on_peer,
			MAC_ADDR_ARRAY(peer_addr.bytes));

		/* Add each instance entry to the list */
		ndp_event_buf->ndp_map[i].ndp_instance_id =
			ind[i].ndp_instance_id;
		ndp_event_buf->ndp_map[i].vdev_id = ind[i].vdev_id;
		WMI_MAC_ADDR_TO_CHAR_ARRAY(&ind[i].peer_ndi_mac_addr,
			ndp_event_buf->ndp_map[i].peer_ndi_mac_addr.bytes);
		ndp_event_buf->ndp_map[i].num_active_ndp_sessions =
			ind[i].num_active_ndps_on_peer;
		ndp_event_buf->ndp_map[i].type = ind[i].type;
		ndp_event_buf->ndp_map[i].reason_code =
			ind[i].reason_code;
	}

	pe_msg.type = SIR_HAL_NDP_END_IND;
	pe_msg.bodyptr = ndp_event_buf;
	pe_msg.bodyval = 0;
	vos_status = vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg);
	if (VOS_IS_STATUS_SUCCESS(vos_status)) {
		return 0;
	}

	WMA_LOGE(FL("failed to post msg to PE"));
	vos_mem_free(ndp_event_buf);
	return -EINVAL;
}

/**
 * wma_ndp_initiator_rsp_event_handler() -NDP initiator rsp event handler
 * @handle: wma handle
 * @event_info: event handler data
 * @len: length of event_info
 *
 * Handler for WMI_NDP_INITIATOR_RSP_EVENTID
 * Return: 0 on success, negative errno on failure
 */
static int wma_ndp_initiator_rsp_event_handler(void *handle,
					uint8_t *event_info, uint32_t len)
{
	VOS_STATUS status;
	vos_msg_t pe_msg = {0};
	WMI_NDP_INITIATOR_RSP_EVENTID_param_tlvs *event;
	wmi_ndp_initiator_rsp_event_fixed_param  *fixed_params;
	struct ndp_initiator_rsp *rsp;

	rsp = vos_mem_malloc(sizeof(*rsp));
	if (NULL == rsp) {
		WMA_LOGE(FL("Invalid rsp_ind"));
		return VOS_STATUS_E_INVAL;
	}

	event = (WMI_NDP_INITIATOR_RSP_EVENTID_param_tlvs *)event_info;
	fixed_params = event->fixed_param;

	rsp->vdev_id = fixed_params->vdev_id;
	rsp->transaction_id = fixed_params->transaction_id;
	rsp->ndp_instance_id = fixed_params->ndp_instance_id;
	rsp->status = fixed_params->rsp_status;
	rsp->reason = fixed_params->reason_code;

	pe_msg.type = SIR_HAL_NDP_INITIATOR_RSP;
	pe_msg.bodyptr = rsp;
	status = vos_mq_post_message(VOS_MODULE_ID_PE, &pe_msg);
	if (!VOS_IS_STATUS_SUCCESS(status)) {
		WMA_LOGE("SIR_HAL_NDP_INITIATOR_RSP to PE failed");
		vos_mem_free(rsp);
		return status;
	}

	return status;
}

/**
 * wma_ndp_register_all_event_handlers() - Register all NDP event handlers
 * @wma_handle: WMA context
 *
 * Register the event handlers for NAN data path events from firmware.
 *
 * Return: None
 */
void wma_ndp_register_all_event_handlers(tp_wma_handle wma_handle)
{
	WMA_LOGD(FL("Register WMI_NDP_INITIATOR_RSP_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_INITIATOR_RSP_EVENTID,
		wma_ndp_initiator_rsp_event_handler);

	WMA_LOGD(FL("Register WMI_NDP_RESPONDER_RSP_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_RESPONDER_RSP_EVENTID,
		wma_ndp_responder_rsp_event_handler);

	WMA_LOGD(FL("Register WMI_NDP_END_RSP_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_END_RSP_EVENTID,
		wma_ndp_end_response_event_handler);

	WMA_LOGD(FL("Register WMI_NDP_INDICATION_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_INDICATION_EVENTID,
		wma_ndp_indication_event_handler);

	WMA_LOGD(FL("Register WMI_NDP_CONFIRM_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_CONFIRM_EVENTID,
		wma_ndp_confirm_event_handler);

	WMA_LOGD(FL("Register WMI_NDP_END_INDICATION_EVENTID"));
	wmi_unified_register_event_handler(wma_handle->wmi_handle,
		WMI_NDP_END_INDICATION_EVENTID,
		wma_ndp_end_indication_event_handler);
}

/**
 * wma_ndp_unregister_all_event_handlers() - Unregister all NDP event handlers
 * @wma_handle: WMA context
 *
 * Register the event handlers for NAN data path events from firmware.
 *
 * Return: None
 */
void wma_ndp_unregister_all_event_handlers(tp_wma_handle wma_handle)
{
	WMA_LOGD(FL("Unregister WMI_NDP_INITIATOR_RSP_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_INITIATOR_RSP_EVENTID);

	WMA_LOGD(FL("Unregister WMI_NDP_RESPONDER_RSP_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_RESPONDER_RSP_EVENTID);

	WMA_LOGD(FL("Unregister WMI_NDP_END_RSP_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_END_RSP_EVENTID);

	WMA_LOGD(FL("Unregister WMI_NDP_INDICATION_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_INDICATION_EVENTID);

	WMA_LOGD(FL("Unregister WMI_NDP_CONFIRM_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_CONFIRM_EVENTID);

	WMA_LOGD(FL("Unregister WMI_NDP_END_INDICATION_EVENTID"));
	wmi_unified_unregister_event_handler(wma_handle->wmi_handle,
		WMI_NDP_END_INDICATION_EVENTID);
}

/**
 * wma_ndp_add_wow_wakeup_event() - Add Wake on Wireless event for NDP
 * @wma_handle: WMA context
 * @enable: dis/enable flag to enable the bit for WOW_NAN_DATA_EVENT
 *
 * Enables the firmware to wake up the host on NAN data path event.
 * All NDP events such as NDP_INDICATION, NDP_CONFIRM, etc. use the
 * same event. They can be distinguished using their TLV tags.
 *
 * Return: none
 */
void wma_ndp_add_wow_wakeup_event(tp_wma_handle wma_handle,
						bool enable)
{
	wma_add_wow_wakeup_event(wma_handle, WOW_NAN_DATA_EVENT, enable);
}

/**
 * wma_ndp_get_eventid_from_tlvtag() - map tlv tag to event id
 * @tag: WMI TLV tag
 *
 * map the tag to known NDP event fixed_param tags and return the
 * corresponding NDP event id.
 *
 * Return: 0 if TLV tag is invalid
 *           else return corresponding WMI event id
 */
static int wma_ndp_get_eventid_from_tlvtag(uint32_t tag)
{
	uint32_t event_id;

	switch (tag) {
	case WMITLV_TAG_STRUC_wmi_ndp_initiator_rsp_event_fixed_param:
		event_id = WMI_NDP_INITIATOR_RSP_EVENTID;
		break;

	case WMITLV_TAG_STRUC_wmi_ndp_responder_rsp_event_fixed_param:
		event_id = WMI_NDP_RESPONDER_RSP_EVENTID;
		break;

	case WMITLV_TAG_STRUC_wmi_ndp_end_rsp_event_fixed_param:
		event_id = WMI_NDP_END_RSP_EVENTID;
		break;

	case WMITLV_TAG_STRUC_wmi_ndp_indication_event_fixed_param:
		event_id = WMI_NDP_INDICATION_EVENTID;
		break;

	case WMITLV_TAG_STRUC_wmi_ndp_confirm_event_fixed_param:
		event_id = WMI_NDP_CONFIRM_EVENTID;
		break;

	case WMITLV_TAG_STRUC_wmi_ndp_end_indication_event_fixed_param:
		event_id = WMI_NDP_END_INDICATION_EVENTID;
		break;

	default:
		event_id = 0;
		WMA_LOGE(FL("Unknown tag: %d"), tag);
		break;
	}

	WMA_LOGI(FL("For tag %d WMI event 0x%x"), tag, event_id);
	return event_id;
}

/**
 * wma_ndp_wow_event_callback() - NAN data path wow event callback
 * @handle: WMA handle
 * @event: event buffer
 * @len: length of @event buffer
 *
 * The wow event WOW_REASON_NAN_DATA is followed by the payload of the event
 * which generated the wow event.
 * Payload is 4 bytes of length followed by event buffer. First 4 bytes
 * of event buffer is common tlv header, which is a combination
 * of tag (higher 2 bytes) and length (lower 2 bytes). The tag is used to
 * identify the event which triggered wow event.
 *
 * Return: none
 */
void wma_ndp_wow_event_callback(void *handle, void *event,
						  uint32_t len)
{
	uint32_t id;
	int tlv_ok_status = 0;
	void *wmi_cmd_struct_ptr = NULL;
	uint32_t tag = WMITLV_GET_TLVTAG(WMITLV_GET_HDR(event));

	/* Reverse map fixed params tag to EVENT_ID */
	id = wma_ndp_get_eventid_from_tlvtag(tag);
	if (!id) {
		WMA_LOGE(FL("Invalid  Tag: %d"), tag);
		return;
	}

	tlv_ok_status = wmitlv_check_and_pad_event_tlvs(handle, event, len,
							id,
							&wmi_cmd_struct_ptr);
	if (tlv_ok_status != 0) {
		WMA_LOGE(FL("Invalid Tag: %d could not check and pad tlvs"),
			 tag);
		return;
	}

	switch (id) {
	case WMI_NDP_INITIATOR_RSP_EVENTID:
		wma_ndp_initiator_rsp_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	case WMI_NDP_RESPONDER_RSP_EVENTID:
		wma_ndp_responder_rsp_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	case WMI_NDP_END_RSP_EVENTID:
		wma_ndp_end_response_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	case WMI_NDP_INDICATION_EVENTID:
		wma_ndp_indication_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	case WMI_NDP_CONFIRM_EVENTID:
		wma_ndp_confirm_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	case WMI_NDP_END_INDICATION_EVENTID:
		wma_ndp_end_indication_event_handler(handle,
						wmi_cmd_struct_ptr, len);
		break;

	default:
		WMA_LOGE(FL("Unknown tag: %d"), tag);
		break;
	}
	wmitlv_free_allocated_event_tlvs(id, &wmi_cmd_struct_ptr);
}

/**
 * wma_add_bss_ndi_mode() - Process BSS creation request while adding NaN
 * Data interface
 * @wma: wma handle
 * @add_bss: Parameters for ADD_BSS command
 *
 * Sends VDEV_START command to firmware
 * Return: None
 */
void wma_add_bss_ndi_mode(tp_wma_handle wma, tpAddBssParams add_bss)
{
	ol_txrx_pdev_handle pdev;
	struct wma_vdev_start_req req;
	ol_txrx_peer_handle peer = NULL;
	struct wma_target_req *msg;
	uint8_t vdev_id, peer_id;
	VOS_STATUS status;
	uint8_t nss_2g, nss_5g;

	WMA_LOGE("%s: enter", __func__);
	if (NULL == wma_find_vdev_by_addr(wma, add_bss->bssId, &vdev_id)) {
		WMA_LOGE("%s: Failed to find vdev", __func__);
		goto send_fail_resp;
	}
	pdev = vos_get_context(VOS_MODULE_ID_TXRX, wma->vos_context);

	if (NULL == pdev) {
		WMA_LOGE("%s: Failed to get pdev", __func__);
		goto send_fail_resp;
	}

	nss_2g = wma->interfaces[vdev_id].nss_2g;
	nss_5g = wma->interfaces[vdev_id].nss_5g;
	wma_set_bss_rate_flags(&wma->interfaces[vdev_id], add_bss);

	peer = ol_txrx_find_peer_by_addr(pdev, add_bss->selfMacAddr, &peer_id);
	if (!peer) {
		WMA_LOGE("%s Failed to find peer %pM", __func__,
			add_bss->selfMacAddr);
		goto send_fail_resp;
	}

	msg = wma_fill_vdev_req(wma, vdev_id, WDA_ADD_BSS_REQ,
			WMA_TARGET_REQ_TYPE_VDEV_START, add_bss,
			WMA_VDEV_START_REQUEST_TIMEOUT);
	if (!msg) {
		WMA_LOGE("%s Failed to allocate vdev request vdev_id %d",
			 __func__, vdev_id);
		goto send_fail_resp;
	}

	add_bss->staContext.staIdx = ol_txrx_local_peer_id(peer);

	/*
	 * beacon_intval, dtim_period, hidden_ssid, is_dfs, ssid
	 * will be ignored for NDI device.
	 */
	vos_mem_zero(&req, sizeof(req));
	req.vdev_id = vdev_id;
	req.chan = add_bss->currentOperChannel;
	req.chan_offset = add_bss->currentExtChannel;
	req.vht_capable = add_bss->vhtCapable;
	req.max_txpow = add_bss->maxTxPower;
	req.oper_mode = add_bss->operMode;

	status = wma_vdev_start(wma, &req, VOS_FALSE);
	if (status != VOS_STATUS_SUCCESS) {
		wma_remove_vdev_req(wma, vdev_id,
			WMA_TARGET_REQ_TYPE_VDEV_START);
		goto send_fail_resp;
	}
	WMA_LOGI("%s: vdev start request for NDI sent to target", __func__);

	/* Initialize protection mode to no protection */
	if (wmi_unified_vdev_set_param_send(wma->wmi_handle, vdev_id,
		WMI_VDEV_PARAM_PROTECTION_MODE,
		IEEE80211_PROT_NONE)) {
		WMA_LOGE("Failed to initialize protection mode");
	}

	return;
send_fail_resp:
	add_bss->status = VOS_STATUS_E_FAILURE;
	wma_send_msg(wma, WDA_ADD_BSS_RSP, (void *)add_bss, 0);
}

/**
 * wma_delete_all_nan_remote_peers() - Delete all nan peer
 * @wma:  wma handle
 * @vdev_id: vdev id
 *
 * Return: void
 */
void wma_delete_all_nan_remote_peers(tp_wma_handle wma, uint32_t vdev_id)
{
	ol_txrx_vdev_handle vdev;
	ol_txrx_peer_handle peer, temp;

	if (!wma || vdev_id >= wma->max_bssid)
		return;

	vdev = wma->interfaces[vdev_id].handle;
	if (!vdev)
		return;

	/* remove all remote peers of ndi*/
	adf_os_spin_lock_bh(&vdev->pdev->peer_ref_mutex);

	temp = NULL;
	TAILQ_FOREACH_REVERSE(peer, &vdev->peer_list,
		peer_list_t, peer_list_elem) {
		if (temp) {
			adf_os_spin_unlock_bh(&vdev->pdev->peer_ref_mutex);
			if (adf_os_atomic_read(
				&temp->delete_in_progress) == 0)
				wma_remove_peer(wma, temp->mac_addr.raw,
					vdev_id, temp, VOS_FALSE);
			adf_os_spin_lock_bh(&vdev->pdev->peer_ref_mutex);
		}
		/* self peer is deleted last */
		if (peer == TAILQ_FIRST(&vdev->peer_list)) {
			WMA_LOGE("%s: self peer removed", __func__);
			break;
		} else
			temp = peer;
	}
	adf_os_spin_unlock_bh(&vdev->pdev->peer_ref_mutex);

	/* remove ndi self peer last */
	peer = TAILQ_FIRST(&vdev->peer_list);
	wma_remove_peer(wma, peer->mac_addr.raw, vdev_id, peer,
			false);
}
/**
 * wma_add_sta_ndi_mode() - Process ADD_STA for NaN Data path
 * @wma: wma handle
 * @add_sta: Parameters of ADD_STA command
 *
 * Sends CREATE_PEER command to firmware
 * Return: void
 */
void wma_add_sta_ndi_mode(tp_wma_handle wma, tpAddStaParams add_sta)
{
	enum ol_txrx_peer_state state = ol_txrx_peer_state_conn;
	ol_txrx_pdev_handle pdev;
	ol_txrx_vdev_handle vdev;
	ol_txrx_peer_handle peer;
	u_int8_t peer_id;
	VOS_STATUS status;
	struct wma_txrx_node *iface;

	pdev = vos_get_context(VOS_MODULE_ID_TXRX, wma->vos_context);

	if (NULL == pdev) {
		WMA_LOGE(FL("Failed to find pdev"));
		add_sta->status = VOS_STATUS_E_FAILURE;
		goto send_rsp;
	}

	vdev = wma_find_vdev_by_id(wma, add_sta->smesessionId);
	if (!vdev) {
		WMA_LOGE(FL("Failed to find vdev"));
		add_sta->status = VOS_STATUS_E_FAILURE;
		goto send_rsp;
	}

	iface = &wma->interfaces[vdev->vdev_id];
	WMA_LOGD(FL("vdev: %d, peer_mac_addr: "MAC_ADDRESS_STR),
		add_sta->smesessionId, MAC_ADDR_ARRAY(add_sta->staMac));

	peer = ol_txrx_find_peer_by_addr_and_vdev(pdev, vdev, add_sta->staMac,
						  &peer_id);
	if (peer) {
		WMA_LOGE(FL("NDI peer already exists, peer_addr %pM"),
			 add_sta->staMac);
		add_sta->status = VOS_STATUS_E_EXISTS;
		goto send_rsp;
	}

	/*
	 * The code above only checks the peer existence on its own vdev.
	 * Need to check whether the peer exists on other vDevs because firmware
	 * can't create the peer if the peer with same MAC address already
	 * exists on the pDev. As this peer belongs to other vDevs, just return
	 * here.
	 */
	peer = ol_txrx_find_peer_by_addr(pdev, add_sta->staMac, &peer_id);
	if (peer) {
		WMA_LOGE(FL("vdev:%d, peer exists on other vdev with peer_addr %pM and peer_id %d"),
			 vdev->vdev_id, add_sta->staMac, peer_id);
		add_sta->status = VOS_STATUS_E_EXISTS;
		goto send_rsp;
	}

	status = wma_create_peer(wma, pdev, vdev, add_sta->staMac,
				 WMI_PEER_TYPE_NAN_DATA, add_sta->smesessionId,
				 VOS_FALSE);
	if (status != VOS_STATUS_SUCCESS) {
		WMA_LOGE(FL("Failed to create peer for %pM"), add_sta->staMac);
		add_sta->status = status;
		goto send_rsp;
	}

	peer = ol_txrx_find_peer_by_addr_and_vdev(pdev, vdev, add_sta->staMac,
						  &peer_id);
	if (!peer) {
		WMA_LOGE(FL("Failed to find peer handle using peer mac %pM"),
			 add_sta->staMac);
		add_sta->status = VOS_STATUS_E_FAILURE;
		wma_remove_peer(wma, add_sta->staMac, add_sta->smesessionId,
				peer, VOS_FALSE);
		goto send_rsp;
	}

	WMA_LOGD(FL("Moving peer %pM to state %d"), add_sta->staMac, state);
	ol_txrx_peer_state_update(pdev, add_sta->staMac, state);

	add_sta->staIdx = ol_txrx_local_peer_id(peer);
	add_sta->nss    = iface->nss;
	add_sta->status = VOS_STATUS_SUCCESS;
send_rsp:
	WMA_LOGD(FL("Sending add sta rsp to umac (mac:%pM, status:%d)"),
		 add_sta->staMac, add_sta->status);
	wma_send_msg(wma, WDA_ADD_STA_RSP, (void *)add_sta, 0);
}

/**
 * wma_delete_sta_req_ndi_mode() - Process DEL_STA request for NDI data peer
 * @wma: WMA context
 * @del_sta: DEL_STA parameters from LIM
 *
 * Removes wma/txrx peer entry for the NDI STA
 *
 * Return: None
 */
void wma_delete_sta_req_ndi_mode(tp_wma_handle wma,
					tpDeleteStaParams del_sta)
{
	ol_txrx_pdev_handle pdev;
	struct ol_txrx_peer_t *peer;

	pdev = vos_get_context(VOS_MODULE_ID_TXRX, wma->vos_context);

	if (!pdev) {
		WMA_LOGE(FL("Failed to get pdev"));
		del_sta->status = VOS_STATUS_E_FAILURE;
		goto send_del_rsp;
	}

	peer = ol_txrx_peer_find_by_local_id(pdev, del_sta->staIdx);
	if (!peer) {
		WMA_LOGE(FL("Failed to get peer handle using peer id %d"),
			 del_sta->staIdx);
		del_sta->status = VOS_STATUS_E_FAILURE;
		goto send_del_rsp;
	}

	wma_remove_peer(wma, peer->mac_addr.raw, del_sta->smesessionId, peer,
			false);
	del_sta->status = VOS_STATUS_SUCCESS;

send_del_rsp:
	if (del_sta->respReqd) {
		WMA_LOGD(FL("Sending del rsp to umac (status: %d)"),
				del_sta->status);
		wma_send_msg(wma, WDA_DELETE_STA_RSP, del_sta, 0);
	}
}


