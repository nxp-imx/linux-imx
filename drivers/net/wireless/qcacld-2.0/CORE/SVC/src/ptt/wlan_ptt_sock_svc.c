/*
 * Copyright (c) 2012-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
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

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/******************************************************************************
 * wlan_ptt_sock_svc.c
 *
 ******************************************************************************/
#ifdef PTT_SOCK_SVC_ENABLE
#include <wlan_nlink_srv.h>
#include <halTypes.h>
#include <vos_status.h>
#include <wlan_hdd_includes.h>
#include <vos_trace.h>
#include <wlan_nlink_common.h>
#include <wlan_ptt_sock_svc.h>
#include <vos_types.h>
#include <vos_trace.h>
#include <wlan_hdd_ftm.h>
#ifdef CNSS_GENL
#include <net/cnss_nl.h>
#else

static struct hdd_context_s *hdd_ctx_handle;
#endif

#define PTT_SOCK_DEBUG
#ifdef PTT_SOCK_DEBUG
#define PTT_TRACE(level, args...) VOS_TRACE( VOS_MODULE_ID_HDD, level, ## args)
#else
#define PTT_TRACE(level, args...)
#endif
// Global variables

#ifdef PTT_SOCK_DEBUG_VERBOSE
//Utility function to perform a hex dump
static void ptt_sock_dump_buf(const unsigned char * pbuf, int cnt)
{
    int i;
    for (i = 0; i < cnt ; i++) {
        if ((i%16)==0)
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,"\n%pK:", pbuf);
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO," %02X", *pbuf);
        pbuf++;
    }
    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,"\n");
}
#endif

/**
 * nl_srv_ucast_ptt() - Wrapper function to send ucast msgs to PTT
 * @skb: sk buffer pointer
 * @dst_pid: Destination PID
 * @flag: flags
 *
 * Sends the ucast message to PTT with generic nl socket if CNSS_GENL
 * is enabled. Else, use the legacy netlink socket to send.
 *
 * Return: zero on success, error code otherwise
 */
static int nl_srv_ucast_ptt(struct sk_buff *skb, int dst_pid, int flag)
{
#ifdef CNSS_GENL
	return nl_srv_ucast(skb, dst_pid, flag, ANI_NL_MSG_PUMAC,
			CLD80211_MCGRP_DIAG_EVENTS);
#else
	return nl_srv_ucast(skb, dst_pid, flag);
#endif
}

/**
 * nl_srv_bcast_ptt() - Wrapper function to send bcast msgs to DIAG mcast group
 * @skb: sk buffer pointer
 *
 * Sends the bcast message to DIAG multicast group with generic nl socket
 * if CNSS_GENL is enabled. Else, use the legacy netlink socket to send.
 *
 * Return: zero on success, error code otherwise
 */
static int nl_srv_bcast_ptt(struct sk_buff *skb)
{
#ifdef CNSS_GENL
	return nl_srv_bcast(skb, CLD80211_MCGRP_DIAG_EVENTS, ANI_NL_MSG_PUMAC);
#else
	return nl_srv_bcast(skb);
#endif
}

//Utility function to send a netlink message to an application in user space
int ptt_sock_send_msg_to_app(tAniHdr *wmsg, int radio, int src_mod, int pid)
{
   int err = -1;
   int payload_len;
   int tot_msg_len;
   tAniNlHdr *wnl;
   struct sk_buff *skb;
   struct nlmsghdr *nlh;
   int wmsg_length = be16_to_cpu(wmsg->length);
   static int nlmsg_seq;
   if (radio < 0 || radio > ANI_MAX_RADIOS) {
      PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s: invalid radio id [%d]\n",
         __func__, radio);
      return -EINVAL;
   }
   payload_len = wmsg_length + sizeof(wnl->radio) + sizeof(tAniHdr);
   tot_msg_len = NLMSG_SPACE(payload_len);
   if ((skb = dev_alloc_skb(tot_msg_len)) == NULL) {
      PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s: dev_alloc_skb() failed for msg size[%d]\n",
         __func__, tot_msg_len);
      return -ENOMEM;
   }
   nlh = nlmsg_put(skb, pid, nlmsg_seq++, src_mod, payload_len, NLM_F_REQUEST);
   if (NULL == nlh) {
      PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s: nlmsg_put() failed for msg size[%d]\n",
         __func__, tot_msg_len);
      kfree_skb(skb);
      return -ENOMEM;
   }
   wnl = (tAniNlHdr *) nlh;
   wnl->radio = radio;
   memcpy(&wnl->wmsg, wmsg, wmsg_length);
#ifdef PTT_SOCK_DEBUG_VERBOSE
   ptt_sock_dump_buf((const unsigned char *)skb->data, skb->len);
#endif

   if (pid != -1) {
       err = nl_srv_ucast_ptt(skb, pid, MSG_DONTWAIT);
   } else {
       err = nl_srv_bcast_ptt(skb);
   }
   if (err) {
       PTT_TRACE(VOS_TRACE_LEVEL_INFO,
       "%s:Failed sending Msg Type [0x%X] to pid[%d]\n",
       __func__, be16_to_cpu(wmsg->type), pid);
   }
   return err;
}

#ifndef CNSS_GENL
/*
 * Process tregisteration request and send registration response messages
 * to the PTT Socket App in user space
 */
static void ptt_sock_proc_reg_req(tAniHdr *wmsg, int radio)
{
   tAniNlAppRegReq *reg_req;
   tAniNlAppRegRsp rspmsg;
   reg_req = (tAniNlAppRegReq *)(wmsg + 1);
   memset((char *)&rspmsg, 0, sizeof(rspmsg));
   /* send reg response message to the application */
   rspmsg.ret = ANI_NL_MSG_OK;
   rspmsg.regReq.type = reg_req->type;

   /* Save the pid */
   hdd_ctx_handle->ptt_pid = reg_req->pid;
   rspmsg.regReq.pid= reg_req->pid;
   rspmsg.wniHdr.type = cpu_to_be16(ANI_MSG_APP_REG_RSP);
   rspmsg.wniHdr.length = cpu_to_be16(sizeof(rspmsg.wniHdr));
   if (ptt_sock_send_msg_to_app((tAniHdr *)&rspmsg.wniHdr, radio,
      ANI_NL_MSG_PUMAC, reg_req->pid) < 0)
   {
      PTT_TRACE(VOS_TRACE_LEVEL_INFO, "%s: Error sending ANI_MSG_APP_REG_RSP to pid[%d]\n",
         __func__, reg_req->pid);
   }
}

/*
 * Process all the messages from the PTT Socket App in user space
 */
static void ptt_proc_pumac_msg(struct sk_buff * skb, tAniHdr *wmsg, int radio)
{
   u16 ani_msg_type = be16_to_cpu(wmsg->type);
   switch(ani_msg_type)
   {
      case ANI_MSG_APP_REG_REQ:
         PTT_TRACE(VOS_TRACE_LEVEL_INFO, "%s: Received ANI_MSG_APP_REG_REQ [0x%X]\n",
            __func__, ani_msg_type);
         ptt_sock_proc_reg_req(wmsg, radio);
         break;
      default:
         PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s: Received Unknown Msg Type[0x%X]\n",
            __func__, ani_msg_type);
         break;
   }
}
/*
 * Process all the Netlink messages from PTT Socket app in user space
 */
static int ptt_sock_rx_nlink_msg (struct sk_buff * skb)
{
   tAniNlHdr *wnl;
   int radio;
   int type;
   wnl = (tAniNlHdr *) skb->data;
   radio = wnl->radio;
   type = wnl->nlh.nlmsg_type;
   switch (type) {
      case ANI_NL_MSG_PUMAC:  // Message from the PTT socket APP
         PTT_TRACE(VOS_TRACE_LEVEL_INFO, "%s: Received ANI_NL_MSG_PUMAC Msg [0x%X]\n",
            __func__, type);
         ptt_proc_pumac_msg(skb, &wnl->wmsg, radio);
         break;
      default:
         PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s: Unknown NL Msg [0x%X]\n",__func__, type);
         break;
   }
   return 0;
}
#endif

#ifdef CNSS_GENL
/**
 * ptt_cmd_handler() - Handler function for PTT commands
 * @data: Data to be parsed
 * @data_len: Length of the data received
 * @ctx: Registered context reference
 * @pid: Process id of the user space application
 *
 * This function handles the command from PTT user space application
 *
 * Return: None
 */
static void ptt_cmd_handler(const void *data, int data_len, void *ctx, int pid)
{
	uint16_t length;
	ptt_app_reg_req *payload;
	struct nlattr *tb[CLD80211_ATTR_MAX + 1];

	/*
	 * audit note: it is ok to pass a NULL policy here since a
	 * length check on the data is added later already
	 */
	if (nla_parse(tb, CLD80211_ATTR_MAX, data, data_len, NULL)) {
		PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "Invalid ATTR");
		return;
	}

	if (!tb[CLD80211_ATTR_DATA]) {
		PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "attr ATTR_DATA failed");
		return;
	}

	if (nla_len(tb[CLD80211_ATTR_DATA]) < sizeof(struct ptt_app_reg_req)) {
		PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "%s:attr length check fails\n",
			__func__);
		return;
	}

	payload = (ptt_app_reg_req *)(nla_data(tb[CLD80211_ATTR_DATA]));
	length = be16_to_cpu(payload->wmsg.length);
	if ((USHRT_MAX - length) < (sizeof(payload->radio) + sizeof(tAniHdr))) {
		PTT_TRACE(QDF_TRACE_LEVEL_ERROR,
			"u16 overflow length %d %zu %zu",
			length,
			sizeof(payload->radio),
			sizeof(tAniHdr));
		return;
	}

	if (nla_len(tb[CLD80211_ATTR_DATA]) <  (length +
						sizeof(payload->radio) +
						sizeof(tAniHdr))) {
		PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "ATTR_DATA len check failed");
		return;
	}

	switch (payload->wmsg.type) {
	case ANI_MSG_APP_REG_REQ:
		ptt_sock_send_msg_to_app(&payload->wmsg, payload->radio,
							ANI_NL_MSG_PUMAC, pid);
		break;
	default:
		PTT_TRACE(VOS_TRACE_LEVEL_ERROR, "Unknown msg type %d",
							payload->wmsg.type);
		break;
	}
}

/**
 * ptt_sock_activate_svc() - API to register PTT/PUMAC command handler
 * @pAdapter: Pointer to adapter
 *
 * API to register the PTT/PUMAC command handlers. Argument @pAdapter
 * is sent for prototype compatibility between new genl and legacy
 * implementation
 *
 * Return: 0
 */
int ptt_sock_activate_svc(void *hdd_ctx)
{
	register_cld_cmd_cb(ANI_NL_MSG_PUMAC, ptt_cmd_handler, NULL);
	register_cld_cmd_cb(ANI_NL_MSG_PTT, ptt_cmd_handler, NULL);
	return 0;
}
#else
int ptt_sock_activate_svc(void *hdd_ctx)
{
   hdd_ctx_handle = (struct hdd_context_s *)hdd_ctx;
   hdd_ctx_handle->ptt_pid = INVALID_PID;
   nl_srv_register(ANI_NL_MSG_PUMAC, ptt_sock_rx_nlink_msg);
   nl_srv_register(ANI_NL_MSG_PTT, ptt_sock_rx_nlink_msg);
   return 0;
}
#endif
#endif // PTT_SOCK_SVC_ENABLE
