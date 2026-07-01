// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 Library
 * Copyright 2025 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/fsl/netc_global.h>
#include <linux/iopoll.h>

#include "ntmp_private.h"

#define NETC_CBDR_TIMEOUT		1000 /* us */
#define NETC_CBDR_DELAY_US		10
#define NETC_CBDR_MR_EN			BIT(31)

#define NTMP_BASE_ADDR_ALIGN		128
#define NTMP_DATA_ADDR_ALIGN		32

/* Define NTMP Table ID */
#define NTMP_MAFT_ID			1
#define NTMP_RSST_ID			3
#define NTMP_RFST_ID			4
#define NTMP_TGST_ID			5
#define NTMP_RPT_ID			10
#define NTMP_IPFT_ID			13
#define NTMP_FDBT_ID			15
#define NTMP_VFT_ID			18
#define NTMP_ISIT_ID			30
#define NTMP_IST_ID			31
#define NTMP_ISFT_ID			32
#define NTMP_ETT_ID			33
#define NTMP_ISGT_ID			34
#define NTMP_ESRT_ID			35
#define NTMP_SGIT_ID			36
#define NTMP_SGCLT_ID			37
#define NTMP_ISCT_ID			38
#define NTMP_ECT_ID			39
#define NTMP_FMT_ID			40
#define NTMP_BPT_ID			41
#define NTMP_SBPT_ID			42
#define NTMP_FMDT_ID			44

/* Generic Update Actions for most tables */
#define NTMP_GEN_UA_CFGEU		BIT(0)
#define NTMP_GEN_UA_STSEU		BIT(1)

/* Update Actions for specific tables */
#define RPT_UA_FEEU			BIT(1)
#define RPT_UA_PSEU			BIT(2)
#define RPT_UA_STSEU			BIT(3)
#define SGIT_UA_ACFGEU			BIT(0)
#define SGIT_UA_CFGEU			BIT(1)
#define SGIT_UA_SGISEU			BIT(2)
#define FDBT_UA_ACTEU			BIT(1)
#define ESRT_UA_SRSEU			BIT(2)
#define ECT_UA_STSEU			BIT(0)
#define BPT_UA_BPSEU			BIT(1)
#define SBPT_UA_BPSEU			BIT(1)
#define ISGT_UA_SGSEU			BIT(1)

/* Quary Action: 0: Full query, 1: Only query entry ID */
#define NTMP_QA_ENTRY_ID		1

#define NTMP_ENTRY_ID_SIZE		4
#define RSST_ENTRY_NUM			64
#define RSST_STSE_DATA_SIZE(n)		((n) * 8)
#define RSST_CFGE_DATA_SIZE(n)		(n)
#define SGCLT_MAX_GE_NUM		256

void ntmp_enable_cbdr(struct netc_cbdr *cbdr)
{
	cbdr->next_to_clean = netc_read(cbdr->regs.cir);
	cbdr->next_to_use = netc_read(cbdr->regs.pir);

	/* Step 1: Configure the base address of the Control BD Ring */
	netc_write(cbdr->regs.bar0, lower_32_bits(cbdr->dma_base_align));
	netc_write(cbdr->regs.bar1, upper_32_bits(cbdr->dma_base_align));

	/* Step 2: Configure the number of BDs of the Control BD Ring */
	netc_write(cbdr->regs.lenr, cbdr->bd_num);

	/* Step 3: Enable the Control BD Ring */
	netc_write(cbdr->regs.mr, NETC_CBDR_MR_EN);
}
EXPORT_SYMBOL_GPL(ntmp_enable_cbdr);

int ntmp_init_cbdr(struct netc_cbdr *cbdr, struct device *dev,
		   const struct netc_cbdr_regs *regs)
{
	int cbd_num = NETC_CBDR_BD_NUM;
	size_t size;

	size = cbd_num * sizeof(union netc_cbd) + NTMP_BASE_ADDR_ALIGN;
	cbdr->addr_base = dma_alloc_coherent(dev, size, &cbdr->dma_base,
					     GFP_KERNEL);
	if (!cbdr->addr_base)
		return -ENOMEM;

	cbdr->dma_size = size;
	cbdr->bd_num = cbd_num;
	cbdr->regs = *regs;
	cbdr->dev = dev;

	/* The base address of the Control BD Ring must be 128 bytes aligned */
	cbdr->dma_base_align =  ALIGN(cbdr->dma_base,  NTMP_BASE_ADDR_ALIGN);
	cbdr->addr_base_align = PTR_ALIGN(cbdr->addr_base,
					  NTMP_BASE_ADDR_ALIGN);

	spin_lock_init(&cbdr->ring_lock);

	ntmp_enable_cbdr(cbdr);

	return 0;
}
EXPORT_SYMBOL_GPL(ntmp_init_cbdr);

void ntmp_free_cbdr(struct netc_cbdr *cbdr)
{
	/* Disable the Control BD Ring */
	netc_write(cbdr->regs.mr, 0);
	dma_free_coherent(cbdr->dev, cbdr->dma_size, cbdr->addr_base,
			  cbdr->dma_base);
	memset(cbdr, 0, sizeof(*cbdr));
}
EXPORT_SYMBOL_GPL(ntmp_free_cbdr);

static int ntmp_get_free_cbd_num(struct netc_cbdr *cbdr)
{
	return (cbdr->next_to_clean - cbdr->next_to_use - 1 +
		cbdr->bd_num) % cbdr->bd_num;
}

static union netc_cbd *ntmp_get_cbd(struct netc_cbdr *cbdr, int index)
{
	return &((union netc_cbd *)(cbdr->addr_base_align))[index];
}

static void ntmp_clean_cbdr(struct netc_cbdr *cbdr)
{
	union netc_cbd *cbd;
	int i;

	i = cbdr->next_to_clean;
	while (netc_read(cbdr->regs.cir) != i) {
		cbd = ntmp_get_cbd(cbdr, i);
		memset(cbd, 0, sizeof(*cbd));
		i = (i + 1) % cbdr->bd_num;
	}

	cbdr->next_to_clean = i;
}

static struct netc_cbdr *netc_select_cbdr(struct ntmp_user *user)
{
	int cpu, i;

	for (i = 0; i < user->cbdr_num; i++) {
		if (spin_is_locked(&user->ring[i].ring_lock))
			continue;

		return &user->ring[i];
	}

	/* If all the command BDRs are busy now, we select
	 * one of them, but need to wait for a while to use.
	 */
	cpu = smp_processor_id();

	return &user->ring[cpu % user->cbdr_num];
}

static int netc_xmit_ntmp_cmd_common(struct ntmp_user *user, union netc_cbd *cbd,
				     bool is_v1)
{
	union netc_cbd *cur_cbd;
	struct netc_cbdr *cbdr;
	int i, err;
	u16 status;
	u32 val;

	if (user->cbdr_num == 1)
		cbdr = &user->ring[0];
	else
		cbdr = netc_select_cbdr(user);

	spin_lock_bh(&cbdr->ring_lock);

	if (unlikely(!ntmp_get_free_cbd_num(cbdr)))
		ntmp_clean_cbdr(cbdr);

	i = cbdr->next_to_use;
	cur_cbd = ntmp_get_cbd(cbdr, i);
	*cur_cbd = *cbd;
	dma_wmb();

	/* Update producer index of both software and hardware */
	i = (i + 1) % cbdr->bd_num;
	cbdr->next_to_use = i;
	netc_write(cbdr->regs.pir, i);

	err = read_poll_timeout_atomic(netc_read, val, val == i,
				       NETC_CBDR_DELAY_US, NETC_CBDR_TIMEOUT,
				       true, cbdr->regs.cir);
	if (unlikely(err))
		goto cbdr_unlock;

	dma_rmb();
	/* Get the writeback command BD, because the caller may need
	 * to check some other fields of the response header.
	 */
	*cbd = *cur_cbd;

	/* Check the writeback error status */
	if (is_v1)
		status = cbd->req_v1.status_flags & NTMP_V1_RESP_STATUS;
	else
		status = le16_to_cpu(cbd->resp_hdr.error_rr) & NTMP_RESP_ERROR;
	if (unlikely(status)) {
		err = -EIO;
		dev_err(user->dev, "Command BD error: 0x%04x\n", status);
	}

	ntmp_clean_cbdr(cbdr);
	dma_wmb();

cbdr_unlock:
	spin_unlock_bh(&cbdr->ring_lock);

	return err;
}

static int netc_xmit_ntmp_cmd(struct ntmp_user *user, union netc_cbd *cbd)
{
	return netc_xmit_ntmp_cmd_common(user, cbd, false);
}

u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 size)
{
	u32 entry_id;

	entry_id = find_first_zero_bit(bitmap, size);
	if (entry_id == size)
		return NTMP_NULL_ENTRY_ID;

	/* Set the bit once we found it */
	set_bit(entry_id, bitmap);

	return entry_id;
}
EXPORT_SYMBOL_GPL(ntmp_lookup_free_eid);

void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id)
{
	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	clear_bit(entry_id, bitmap);
}
EXPORT_SYMBOL_GPL(ntmp_clear_eid_bitmap);

u32 ntmp_lookup_free_words(unsigned long *bitmap, u32 size, u32 num_words)
{
	u32 entry_id, num;
	u32 next_eid = 0;

	do {
		entry_id = find_next_zero_bit(bitmap, size, next_eid);
		if (entry_id == size)
			return NTMP_NULL_ENTRY_ID;

		next_eid = find_next_bit(bitmap, size, entry_id + 1);
		num = next_eid - entry_id;
	} while (num < num_words && next_eid != size);

	if (num < num_words)
		return NTMP_NULL_ENTRY_ID;

	bitmap_set(bitmap, entry_id, num_words);

	return entry_id;
}

void ntmp_clear_words_bitmap(unsigned long *bitmap, u32 entry_id, u32 num_words)
{
	if (entry_id == NTMP_NULL_ENTRY_ID)
		return;

	bitmap_clear(bitmap, entry_id, num_words);
}

static int ntmp_alloc_data_mem(struct ntmp_dma_buf *data, void **buf_align)
{
	void *buf;

	buf = dma_alloc_coherent(data->dev, data->size + NTMP_DATA_ADDR_ALIGN,
				 &data->dma, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	data->buf = buf;
	*buf_align = PTR_ALIGN(buf, NTMP_DATA_ADDR_ALIGN);

	return 0;
}

static void ntmp_free_data_mem(struct ntmp_dma_buf *data)
{
	dma_free_coherent(data->dev, data->size + NTMP_DATA_ADDR_ALIGN,
			  data->buf, data->dma);
}

/* NTMP V1.0 functions */
static int netc_xmit_ntmp_v1_cmd(struct ntmp_user *user, union netc_cbd *cbdv1)
{
	return netc_xmit_ntmp_cmd_common(user, cbdv1, true);
}

static inline int ntmp_v1_cbd_alloc_data_mem(struct ntmp_dma_buf *data,
					     union netc_cbd *cbd,
					     void **data_align)
{
	dma_addr_t dma_align;
	int err;

	err = ntmp_alloc_data_mem(data, data_align);
	if (err)
		return err;

	dma_align = ALIGN(data->dma, NTMP_DATA_ADDR_ALIGN);

	cbd->req_v1.addr = cpu_to_le64(dma_align);
	cbd->req_v1.length = cpu_to_le16(data->size);

	return 0;
}

int ntmp_v1_rfst_set_entry(struct ntmp_user *user, u32 entry_id,
			   struct rfse_set_buff *rfse)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(*rfse),
	};
	union netc_cbd cbd = { .req_v1.cmd = 0 };
	struct device *dev = user->dev;
	void *tmp_align;
	int err;

	/* fill up the "set" descriptor */
	cbd.req_v1.cmd = 0;
	cbd.req_v1.cls = 4;
	cbd.req_v1.index = cpu_to_le16(entry_id);
	cbd.req_v1.opt[3] = cpu_to_le32(0); /* SI */

	err = ntmp_v1_cbd_alloc_data_mem(&data, &cbd, &tmp_align);
	if (err)
		return err;

	memcpy(tmp_align, rfse, sizeof(*rfse));

	err = netc_xmit_ntmp_v1_cmd(user, &cbd);
	if (err)
		dev_err(dev, "Set table (id: %d) entry failed: %d!",
			NTMP_RFST_ID, err);

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_v1_rfst_set_entry);

int ntmp_v1_rfst_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	struct rfse_set_buff rfse = { };

	return ntmp_v1_rfst_set_entry(user, entry_id, &rfse);
}
EXPORT_SYMBOL_GPL(ntmp_v1_rfst_delete_entry);
/* NTMP V1.0 functions end */

static void ntmp_fill_request_hdr(union netc_cbd *cbd, dma_addr_t dma,
				  int len, int table_id, int cmd,
				  int access_method)
{
	dma_addr_t dma_align;

	memset(cbd, 0, sizeof(*cbd));
	dma_align = ALIGN(dma, NTMP_DATA_ADDR_ALIGN);
	cbd->req_hdr.addr = cpu_to_le64(dma_align);
	cbd->req_hdr.len = cpu_to_le32(len);
	cbd->req_hdr.cmd = cmd;
	cbd->req_hdr.access_method = FIELD_PREP(NTMP_ACCESS_METHOD,
						access_method);
	cbd->req_hdr.table_id = table_id;
	cbd->req_hdr.ver_cci_rr = FIELD_PREP(NTMP_HDR_VERSION,
					     NTMP_HDR_VER2);
	/* For NTMP version 2.0 or later version */
	cbd->req_hdr.npf = cpu_to_le32(NTMP_NPF);
}

static void ntmp_fill_crd(struct ntmp_cmn_req_data *crd, u8 tblv,
			  u8 qa, u16 ua)
{
	crd->update_act = cpu_to_le16(ua);
	crd->tblv_qact = NTMP_TBLV_QACT(tblv, qa);
}

static void ntmp_fill_crd_eid(struct ntmp_req_by_eid *rbe, u8 tblv,
			      u8 qa, u16 ua, u32 entry_id)
{
	ntmp_fill_crd(&rbe->crd, tblv, qa, ua);
	rbe->entry_id = cpu_to_le32(entry_id);
}

static const char *ntmp_table_name(int tbl_id)
{
	switch (tbl_id) {
	case NTMP_MAFT_ID:
		return "MAC Address Filter Table";
	case NTMP_RSST_ID:
		return "RSS Table";
	case NTMP_RPT_ID:
		return "Rate Policer Table";
	case NTMP_ISIT_ID:
		return "Ingress Stream Identification Table";
	case NTMP_IST_ID:
		return "Ingress Stream Table";
	case NTMP_ISFT_ID:
		return "Ingress Stream Filter Table";
	case NTMP_SGIT_ID:
		return "Stream Gate Instance Table";
	case NTMP_SGCLT_ID:
		return "Stream Gate Control List Table";
	case NTMP_IPFT_ID:
		return "Ingress Port Filter Table";
	case NTMP_RFST_ID:
		return "RFS Table";
	case NTMP_FDBT_ID:
		return "FDB Table";
	case NTMP_ETT_ID:
		return "Egress Treatment Table";
	case NTMP_ESRT_ID:
		return "Egress Sequence Recovery Table";
	case NTMP_FMT_ID:
		return "Frame Modification Table";
	case NTMP_BPT_ID:
		return "Buffer Pool Table";
	case NTMP_SBPT_ID:
		return "Shared Buffer Pool Table";
	case NTMP_FMDT_ID:
		return "Frame Modification Data Table";
	default:
		return "Unknown Table";
	};
}

static int ntmp_delete_entry_by_id(struct ntmp_user *user, int tbl_id,
				   u8 tbl_ver, u32 entry_id, u32 req_len,
				   u32 resp_len)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = max(req_len, resp_len),
	};
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, tbl_ver, 0, 0, entry_id);
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(req_len, resp_len),
			      tbl_id, NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to delete entry 0x%x of %s, err: %pe",
			entry_id, ntmp_table_name(tbl_id), ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}

static int ntmp_query_entry_by_id(struct ntmp_user *user, int tbl_id,
				  u32 len, struct ntmp_req_by_eid *req,
				  dma_addr_t dma, bool compare_eid)
{
	struct ntmp_cmn_resp_query *resp;
	int cmd = NTMP_CMD_QUERY;
	union netc_cbd cbd;
	u32 entry_id;
	int err;

	entry_id = le32_to_cpu(req->entry_id);
	if (le16_to_cpu(req->crd.update_act))
		cmd = NTMP_CMD_QU;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, dma, len, tbl_id, cmd, NTMP_AM_ENTRY_ID);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev,
			"Failed to query entry 0x%x of %s, err: %pe\n",
			entry_id, ntmp_table_name(tbl_id), ERR_PTR(err));
		return err;
	}

	/* For a few tables, the first field of their response data is not
	 * entry_id, so directly return success.
	 */
	if (!compare_eid)
		return 0;

	resp = (struct ntmp_cmn_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(user->dev,
			"%s: query EID 0x%x doesn't match response EID 0x%x\n",
			ntmp_table_name(tbl_id), entry_id, le32_to_cpu(resp->entry_id));
		return -EIO;
	}

	return 0;
}

int ntmp_maft_add_entry(struct ntmp_user *user, u32 entry_id,
			struct maft_entry_data *maft)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct maft_req_add),
	};
	struct maft_req_add *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set mac address filter table request data buffer */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.maft_ver, 0, 0, entry_id);
	req->keye = maft->keye;
	req->cfge = maft->cfge;

	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_MAFT_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to add MAFT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_add_entry);

int ntmp_maft_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct maft_entry_data *maft)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct maft_resp_query),
	};
	struct maft_resp_query *resp;
	struct ntmp_req_by_eid *req;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.maft_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(user, NTMP_MAFT_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct maft_resp_query *)req;
	maft->keye = resp->keye;
	maft->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_query_entry);

int ntmp_maft_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_MAFT_ID, user->tbl.maft_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_maft_delete_entry);

int ntmp_rsst_update_entry(struct ntmp_user *user, const u32 *table,
			   int count)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct rsst_req_update *req;
	union netc_cbd cbd;
	int err, i;

	if (count != RSST_ENTRY_NUM)
		/* HW only takes in a full 64 entry table */
		return -EINVAL;

	data.size = struct_size(req, groups, count);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.rsst_ver, 0,
			  NTMP_GEN_UA_CFGEU | NTMP_GEN_UA_STSEU, 0);
	for (i = 0; i < count; i++)
		req->groups[i] = (u8)(table[i]);

	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_RSST_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update RSST entry, err: %pe\n",
			ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rsst_update_entry);

int ntmp_rsst_query_entry(struct ntmp_user *user, u32 *table, int count)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	int err, i;
	u8 *group;

	if (count != RSST_ENTRY_NUM)
		/* HW only takes in a full 64 entry table */
		return -EINVAL;

	data.size = NTMP_ENTRY_ID_SIZE + RSST_STSE_DATA_SIZE(count) +
		    RSST_CFGE_DATA_SIZE(count);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer */
	ntmp_fill_crd_eid(req, user->tbl.rsst_ver, 0, 0, 0);
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(sizeof(*req), data.size),
			      NTMP_RSST_ID, NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to query RSST entry, err: %pe\n",
			ERR_PTR(err));
		goto end;
	}

	group = (u8 *)req;
	group += NTMP_ENTRY_ID_SIZE + RSST_STSE_DATA_SIZE(count);
	for (i = 0; i < count; i++)
		table[i] = group[i];

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rsst_query_entry);

int ntmp_tgst_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct tgst_query_data *tgst)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct tgst_resp_query *resp;
	struct tgst_cfge_data *cfge;
	struct tgst_olse_data *olse;
	struct ntmp_req_by_eid *req;
	int i, err;

	data.size = sizeof(*resp) + struct_size(cfge, ge, TGST_MAX_ENTRY_NUM) +
		    struct_size(olse, ge, TGST_MAX_ENTRY_NUM);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.tgst_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(user, NTMP_TGST_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, false);
	if (err)
		goto end;

	resp = (struct tgst_resp_query *)req;
	cfge = (struct tgst_cfge_data *)resp->data;

	tgst->config_change_time = resp->status.cfg_ct;
	tgst->admin_bt = cfge->admin_bt;
	tgst->admin_ct = cfge->admin_ct;
	tgst->admin_ct_ext = cfge->admin_ct_ext;
	tgst->admin_cl_len = cfge->admin_cl_len;
	for (i = 0; i < le16_to_cpu(cfge->admin_cl_len); i++)
		tgst->cfge_ge[i] = cfge->ge[i];

	olse = (struct tgst_olse_data *)&cfge->ge[i];
	tgst->oper_cfg_ct = olse->oper_cfg_ct;
	tgst->oper_cfg_ce = olse->oper_cfg_ce;
	tgst->oper_bt = olse->oper_bt;
	tgst->oper_ct = olse->oper_ct;
	tgst->oper_ct_ext = olse->oper_ct_ext;
	tgst->oper_cl_len = olse->oper_cl_len;
	for (i = 0; i < le16_to_cpu(olse->oper_cl_len); i++)
		tgst->olse_ge[i] = olse->ge[i];

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_tgst_delete_admin_gate_list(struct ntmp_user *user, u32 entry_id)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct tgst_req_update),
	};
	struct tgst_req_update *req;
	struct tgst_cfge_data *cfge;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer and set the admin control list len
	 * to zero to delete the existing admin control list.
	 */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.tgst_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	cfge = &req->cfge;
	cfge->admin_cl_len = 0;

	/* Request header */
	len = NTMP_LEN(data.size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_TGST_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to delete TGST entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_tgst_update_admin_gate_list(struct ntmp_user *user, u32 entry_id,
				     struct tgst_cfge_data *cfge)
{
	u16 list_len = le16_to_cpu(cfge->admin_cl_len);
	u32 cfge_len = struct_size(cfge, ge, list_len);
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct tgst_req_update *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	/* Calculate the size of request data buffer */
	data.size = struct_size(req, cfge.ge, list_len);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Set the request data buffer */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.tgst_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	memcpy(&req->cfge, cfge, cfge_len);

	/* Request header */
	len = NTMP_LEN(data.size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_TGST_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update TGST entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_rpt_add_entry(struct ntmp_user *user, struct ntmp_rpt_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct rpt_req_ua),
	};
	struct rpt_req_ua *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.rpt_ver, 0, NTMP_GEN_UA_CFGEU |
			  RPT_UA_FEEU | RPT_UA_PSEU | RPT_UA_STSEU,
			  entry->entry_id);
	req->cfge = entry->cfge;
	req->fee = entry->fee;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_RPT_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to add RPT entry 0x%x, err: %pe\n",
			entry->entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_add_entry);

int ntmp_rpt_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct ntmp_rpt_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct rpt_resp_query),
	};
	struct ntmp_req_by_eid *req;
	struct rpt_resp_query *resp;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.rpt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_RPT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct rpt_resp_query *)req;
	entry->stse = resp->stse;
	entry->cfge = resp->cfge;
	entry->fee = resp->fee;
	entry->pse = resp->pse;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_rpt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_RPT_ID, user->tbl.rpt_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_rpt_delete_entry);

int ntmp_isit_add_entry(struct ntmp_user *user, struct ntmp_isit_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isit_resp_query),
	};
	struct isit_resp_query *resp;
	struct isit_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.isit_ver, NTMP_QA_ENTRY_ID,
		      NTMP_GEN_UA_CFGEU);
	req->ak.keye = entry->keye;
	req->is_eid = entry->is_eid;

	len = NTMP_LEN(sizeof(*req), sizeof(*resp));
	/* Add command, followed by a query. So that we can get
	 * the entry id from HW.
	 */
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_ISIT_ID,
			      NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to add ISIT entry, err: %pe\n",
			ERR_PTR(err));

		goto end;
	}

	resp = (struct isit_resp_query *)req;
	entry->entry_id = le32_to_cpu(resp->entry_id);

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isit_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct ntmp_isit_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isit_resp_query),
	};
	struct isit_resp_query *resp;
	struct isit_req_qd *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.isit_ver, 0, 0);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_ISIT_ID, len,
				     (struct ntmp_req_by_eid *)req,
				     data.dma, false);
	if (err)
		goto end;

	resp = (struct isit_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(user->dev,
			"ISIT: query EID (0x%0x) doesn't match response EID (0x%x)\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;

		goto end;
	}

	entry->keye = resp->keye;
	entry->is_eid = resp->is_eid;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isit_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	u32 req_len = sizeof(struct isit_req_qd);

	return ntmp_delete_entry_by_id(user, NTMP_ISIT_ID, user->tbl.isit_ver,
				       entry_id, req_len, NTMP_STATUS_RESP_LEN);
}

int ntmp_ist_add_entry(struct ntmp_user *user, struct ntmp_ist_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ist_req_ua),
	};
	struct ist_req_ua *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Fill up NTMP request data buffer */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.ist_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry->entry_id);
	req->cfge = entry->cfge;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_IST_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to add IST entry 0x%x, err: %pe\n",
			entry->entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_add_entry);

int ntmp_ist_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct ist_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ist_resp_query),
	};
	struct ist_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.ist_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_IST_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct ist_resp_query *)req;
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_ist_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_IST_ID, user->tbl.ist_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_ist_delete_entry);

int ntmp_isft_add_entry(struct ntmp_user *user, struct ntmp_isft_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isft_resp_query),
	};
	struct isft_resp_query *resp;
	struct isft_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.isft_ver, NTMP_QA_ENTRY_ID,
		      NTMP_GEN_UA_CFGEU);
	req->ak.keye = entry->keye;
	req->cfge = entry->cfge;

	len = NTMP_LEN(sizeof(*req), sizeof(*resp));
	/* Add command, followed by a query. So that we can get entry
	 * ID from hardware.
	 */
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_ISFT_ID,
			      NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to add ISFT entry, err: %pe\n",
			ERR_PTR(err));

		goto end;
	}

	resp = (struct isft_resp_query *)req;
	entry->entry_id = le32_to_cpu(resp->entry_id);

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isft_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct ntmp_isft_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isft_resp_query),
	};
	struct isft_resp_query *resp;
	struct isft_req_qd *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.isft_ver, 0, 0);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_ISFT_ID, len,
				     (struct ntmp_req_by_eid *)req,
				     data.dma, false);
	if (err)
		goto end;

	resp = (struct isft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(user->dev,
			"ISFT: query EID (0x%0x) doesn't match response EID (0x%x)\n",
			entry_id, le32_to_cpu(resp->entry_id));

		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_isft_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	u32 req_len = sizeof(struct isft_req_qd);

	return ntmp_delete_entry_by_id(user, NTMP_ISFT_ID, user->tbl.isft_ver,
				       entry_id, req_len, NTMP_STATUS_RESP_LEN);
}

int ntmp_sgit_add_or_update_entry(struct ntmp_user *user, bool add,
				  struct ntmp_sgit_entry *entry)
{
	int cmd = add ? NTMP_CMD_ADD : NTMP_CMD_UPDATE;
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct sgit_req_ua),
	};
	struct sgit_req_ua *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.sgit_ver, 0, SGIT_UA_ACFGEU |
			  SGIT_UA_CFGEU | SGIT_UA_SGISEU, entry->entry_id);
	req->acfge = entry->acfge;
	req->cfge = entry->cfge;
	req->icfge = entry->icfge;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_SGIT_ID, cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to %s SGIT entry 0x%x, err: %pe\n",
			add ? "add" : "update", entry->entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_add_or_update_entry);

int ntmp_sgit_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct ntmp_sgit_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct sgit_resp_query),
	};
	struct sgit_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.sgit_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_SGIT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct sgit_resp_query *)req;
	entry->sgise = resp->sgise;
	entry->cfge = resp->cfge;
	entry->icfge = resp->icfge;
	entry->acfge = resp->acfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgit_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_SGIT_ID, user->tbl.sgit_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}

int ntmp_sgclt_add_entry(struct ntmp_user *user, struct ntmp_sgclt_entry *entry)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct sgclt_req_add *req;
	u32 num_gates, cfge_len;
	union netc_cbd cbd;
	int err;

	num_gates = entry->cfge.list_length + 1;
	data.size = struct_size(req, cfge.ge, num_gates);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Fill up NTMP request data buffer */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.sgclt_ver, 0, 0,
			  entry->entry_id);
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	memcpy(&req->cfge, &entry->cfge, cfge_len);

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_SGCLT_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to add SGCLT entry 0x%x, err: %pe\n",
			entry->entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgclt_query_entry(struct ntmp_user *user, u32 entry_id,
			   struct ntmp_sgclt_entry *entry, u32 cfge_size)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct sgclt_resp_query *resp;
	u32 num_gates, cfge_len, len;
	struct ntmp_req_by_eid *req;
	int err;

	data.size = struct_size(resp, cfge.ge, SGCLT_MAX_GE_NUM);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.sgclt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_SGCLT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct sgclt_resp_query *)req;
	entry->ref_count = resp->ref_count;
	num_gates = resp->cfge.list_length + 1;
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	if (cfge_len > cfge_size) {
		dev_err(user->dev,
			"Responsed SGCLT_CFGE size (%u) is larger than %u\n",
			cfge_size, cfge_len);
		err = -ENOMEM;

		goto end;
	}

	memcpy(&entry->cfge, &resp->cfge, cfge_len);

end:
	ntmp_free_data_mem(&data);

	return err;
}

int ntmp_sgclt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_SGCLT_ID, user->tbl.sgclt_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}

int ntmp_isct_set_entry(struct ntmp_user *user, u32 entry_id, int cmd,
			struct isct_stse_data *stse)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	bool query = !!(cmd & NTMP_CMD_QUERY);
	struct isct_resp_query *resp;
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	u16 ua = 0;
	u32 len;
	int err;

	/* Check the command. */
	switch (cmd) {
	case NTMP_CMD_QUERY:
	case NTMP_CMD_QU:
		if (!stse)
			return -EINVAL;
		fallthrough;
	case NTMP_CMD_DELETE:
	case NTMP_CMD_UPDATE:
	case NTMP_CMD_ADD:
	break;
	default:
		return -EINVAL;
	}

	data.size = query ? sizeof(*resp) : sizeof(*req);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	if (cmd & NTMP_CMD_UPDATE)
		ua = NTMP_GEN_UA_CFGEU;

	ntmp_fill_crd_eid(req, user->tbl.isct_ver, 0, ua, entry_id);
	len = NTMP_LEN(sizeof(*req), query ? sizeof(*resp) : 0);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_ISCT_ID,
			      cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev,
			"Failed to set ISCT entry 0x%x, cmd:%d err: %pe\n",
			entry_id, cmd, ERR_PTR(err));

		goto end;
	}

	if (!query)
		goto end;

	resp = (struct isct_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(user->dev,
			"ISCT: query EID (0x%0x) doesn't match response EID (0x%x)\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;

		goto end;
	}

	*stse = resp->stse;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isct_set_entry);

int ntmp_ipft_add_entry(struct ntmp_user *user, struct ntmp_ipft_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ipft_resp_query),
	};
	struct ipft_resp_query *resp;
	struct ipft_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.ipft_ver, NTMP_QA_ENTRY_ID,
		      NTMP_GEN_UA_CFGEU | NTMP_GEN_UA_STSEU);
	req->ak.keye = entry->keye;
	req->cfge = entry->cfge;

	len = NTMP_LEN(sizeof(*req), data.size);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_IPFT_ID,
			      NTMP_CMD_AQ, NTMP_AM_TERNARY_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to add IPFT entry, err: %pe\n",
			ERR_PTR(err));

		goto end;
	}

	resp = (struct ipft_resp_query *)req;
	entry->entry_id = le32_to_cpu(resp->entry_id);

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_add_entry);

int ntmp_ipft_update_entry(struct ntmp_user *user, u32 entry_id,
			   struct ipft_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ipft_req_ua),
	};
	struct ipft_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd(&req->crd, user->tbl.ipft_ver, 0,
		      NTMP_GEN_UA_CFGEU | NTMP_GEN_UA_STSEU);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	req->cfge = *cfge;

	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_IPFT_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update IPFT entry, err: %pe\n",
			ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_update_entry);

int ntmp_ipft_query_entry(struct ntmp_user *user, u32 entry_id,
			  bool update, struct ntmp_ipft_entry *entry)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ipft_resp_query),
	};
	u32 req_len = sizeof(struct ipft_req_qd);
	struct ipft_resp_query *resp;
	struct ipft_req_qd *req;
	u16 ua = 0;
	u32 len;
	int err;

	/* CFGE_DATA is present when performing an update command,
	 * but we don't need to set this field because only STSEU
	 * is updated here.
	 */
	if (update) {
		req_len += sizeof(struct ipft_cfge_data);
		ua = NTMP_GEN_UA_STSEU;
	}

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.ipft_ver, 0, ua, entry_id);
	len = NTMP_LEN(req_len, data.size);
	err = ntmp_query_entry_by_id(user, NTMP_IPFT_ID, len,
				     (struct ntmp_req_by_eid *)req,
				     data.dma, false);
	if (err)
		goto end;

	resp = (struct ipft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(user->dev,
			"IPFT: query EID 0x%x doesn't match response EID 0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;

		goto end;
	}

	entry->keye = resp->keye;
	entry->match_count = resp->match_count;
	entry->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_query_entry);

int ntmp_ipft_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	u32 req_len = sizeof(struct ipft_req_qd);

	return ntmp_delete_entry_by_id(user, NTMP_IPFT_ID, user->tbl.ipft_ver,
				       entry_id, req_len, NTMP_STATUS_RESP_LEN);
}
EXPORT_SYMBOL_GPL(ntmp_ipft_delete_entry);

int ntmp_rfst_add_entry(struct ntmp_user *user, u32 entry_id,
			struct rfst_entry_data *rfst)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct rfst_req_add),
	};
	struct rfst_req_add *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.rfst_ver, 0, 0, entry_id);
	req->keye = rfst->keye;
	req->cfge = rfst->cfge;

	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_RFST_ID, NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to add RFST entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rfst_add_entry);

int ntmp_rfst_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct rfst_entry_data *rfst)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct rfst_resp_query),
	};
	struct rfst_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.rfst_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_RFST_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct rfst_resp_query *)req;
	rfst->keye = resp->keye;
	rfst->cfge = resp->cfge;
	rfst->matched_frames = resp->matched_frames;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rfst_query_entry);

int ntmp_rfst_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_RFST_ID, user->tbl.rfst_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_rfst_delete_entry);

/**
 * ntmp_fdbt_update_activity_element - update the aging time of all the dynamic
 * entries in the FDB table.
 * @user: target ntmp_user struct
 *
 * A single activity update management could be used to process all the dynamic
 * entries in the FDB table. When hardware process an activity updata management
 * command for an entry in the FDB table and the entry does not have its activity
 * flag set, the activity counter is incremented, If, however, the activity flag
 * is set, then both the activity flag and activity counter are reset.
 * Software can issue the activity update management commands at predefined times
 * and the value of the activity counter can then be used to estimate the period
 * of how long an FDB entry has been inactive.
 *
 * Returns 0 on success or < 0 on error
 */
int ntmp_fdbt_update_activity_element(struct ntmp_user *user)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_ua),
	};
	struct fdbt_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, 0, FDBT_UA_ACTEU);
	req->ak.search.resume_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	/* For activity update, the access method must be search */
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_SEARCH);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update FDBT activity, err: %pe\n",
			ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_update_activity_element);

/**
 * ntmp_fdbt_delete_aging_entries - delete all the matched dynamic entries
 * in the FDB table
 * @user: target ntmp_user struct
 * @act_cnt: the target value of the activity counter
 *
 * The matching rule is that the activity flag is not set and the activity
 * counter is greater than or equal to act_cnt
 *
 * Returns 0 on success or < 0 on error
 */
int ntmp_fdbt_delete_aging_entries(struct ntmp_user *user, u8 act_cnt)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_qd),
	};
	struct fdbt_req_qd *req;
	u32 cfg = FDBT_DYNAMIC;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	if (act_cnt > FDBT_MAX_ACT_CNT)
		act_cnt = FDBT_MAX_ACT_CNT;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, 0, 0);
	req->ak.search.resume_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	req->ak.search.cfge.cfg = cpu_to_le32(cfg);
	req->ak.search.acte.act = act_cnt & FDBT_ACT_CNT;
	/* Entry match with ACTE_DATA[ACT_FLAG] AND match >= ACTE_DATA[ACT_CNT] */
	req->ak.search.acte_mc = FDBT_ACTE_MC;
	req->ak.search.cfge_mc = FDBT_CFGE_MC_DYNAMIC;

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	/* For activity update, the access method must be search */
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_DELETE, NTMP_AM_SEARCH);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to delete aging FDBT entries, err: %pe\n",
			ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_delete_aging_entries);

/**
 * ntmp_fdbt_add_entry - add an entry into the FDB table
 * @user: target ntmp_user struct
 * @entry_id: retruned value, the ID of the FDB entry
 * @keye: key element data
 * @cfge: configuration element data
 *
 * Returns two values: entry_id and error code (0 on success or < 0 on error)
 */
int ntmp_fdbt_add_entry(struct ntmp_user *user, u32 *entry_id,
			struct fdbt_keye_data *keye,
			struct fdbt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_ua),
	};
	struct fdbt_resp_query *resp;
	struct fdbt_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, NTMP_QA_ENTRY_ID,
		      NTMP_GEN_UA_CFGEU);
	req->ak.exact.keye = *keye;
	req->cfge = *cfge;

	len = NTMP_LEN(data.size, sizeof(*resp));
	/* The entry id is allotted by hardware, so we need to a query
	 * action after the add action to get the entry id from hardware.
	 */
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to add FDBT entry, err: %pe\n",
			ERR_PTR(err));
		goto end;
	}

	if (entry_id) {
		resp = (struct fdbt_resp_query *)req;
		*entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_add_entry);

int ntmp_fdbt_update_entry(struct ntmp_user *user, u32 entry_id,
			   struct fdbt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_ua),
	};
	struct fdbt_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, 0, NTMP_GEN_UA_CFGEU);
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	req->cfge = *cfge;

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update FDBT entry, err: %pe\n",
			ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_update_entry);

int ntmp_fdbt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	u32 req_len = sizeof(struct fdbt_req_qd);

	return ntmp_delete_entry_by_id(user, NTMP_FDBT_ID, user->tbl.fdbt_ver,
				       entry_id, req_len, NTMP_STATUS_RESP_LEN);
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_delete_entry);

int ntmp_fdbt_delete_port_dynamic_entries(struct ntmp_user *user, int port)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_qd),
	};
	struct fdbt_req_qd *req;
	u32 cfg = FDBT_DYNAMIC;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, 0, 0);
	req->ak.search.resume_eid = cpu_to_le32(NTMP_NULL_ENTRY_ID);
	req->ak.search.cfge.port_bitmap = cpu_to_le32(BIT(port));
	req->ak.search.cfge.cfg = cpu_to_le32(cfg);
	/* Match CFGE_DATA[DYNAMIC & PORT_BITMAP] field */
	req->ak.search.cfge_mc = FDBT_CFGE_MC_DYNAMIC_AND_PORT_BITMAP;

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_DELETE, NTMP_AM_SEARCH);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to delete dynamic FDBT entries on port %d, err: %pe\n",
			port,  ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_delete_port_dynamic_entries);

int ntmp_fdbt_search_port_entry(struct ntmp_user *user, int port,
				u32 *resume_entry_id, u32 *entry_id,
				struct fdbt_entry_data *fdbt)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fdbt_req_qd),
	};
	struct fdbt_resp_query *resp;
	struct fdbt_req_qd *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.fdbt_ver, 0, 0);
	req->ak.search.resume_eid = cpu_to_le32(*resume_entry_id);
	req->ak.search.cfge.port_bitmap = cpu_to_le32(BIT(port));
	/* Match CFGE_DATA[PORT_BITMAP] field */
	req->ak.search.cfge_mc = FDBT_CFGE_MC_PORT_BITMAP;

	/* Request header */
	len = NTMP_LEN(data.size, sizeof(*resp));
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FDBT_ID,
			      NTMP_CMD_QUERY, NTMP_AM_SEARCH);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev,
			"Failed to search FDBT entry on port %d, err: %pe\n",
			port, ERR_PTR(err));
		goto end;
	}

	if (!cbd.resp_hdr.num_matched) {
		*entry_id = NTMP_NULL_ENTRY_ID;
		*resume_entry_id = NTMP_NULL_ENTRY_ID;
		goto end;
	}

	resp = (struct fdbt_resp_query *)req;
	*entry_id = le32_to_cpu(resp->entry_id);
	*resume_entry_id = le32_to_cpu(resp->status);
	fdbt->keye = resp->keye;
	fdbt->cfge = resp->cfge;
	fdbt->acte = resp->acte;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fdbt_search_port_entry);

/**
 * ntmp_vft_add_entry - add an entry into the VLAN filter table
 * @user: target ntmp_user struct
 * @entry_id: retruned value, the ID of the Vlan filter entry
 * @vid: VLAN ID
 * @cfge: configuration elemenet data
 *
 * Returns two values: entry_id and error code (0 on success or < 0 on error)
 */
int ntmp_vft_add_entry(struct ntmp_user *user, u32 *entry_id,
		       u16 vid, struct vft_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct vft_resp_query),
	};
	struct vft_resp_query *resp;
	struct vft_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.vft_ver, NTMP_QA_ENTRY_ID,
		      NTMP_GEN_UA_CFGEU);
	req->ak.exact.vid = cpu_to_le16(vid);
	req->cfge = *cfge;

	/* Request header */
	len = NTMP_LEN(sizeof(*req), data.size);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_VFT_ID,
			      NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev,
			"Failed to add VFT entry, vid: %u, err: %pe\n",
			vid, ERR_PTR(err));
		goto end;
	}

	if (entry_id) {
		resp = (struct vft_resp_query *)req;
		*entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vft_add_entry);

int ntmp_vft_update_entry(struct ntmp_user *user, u16 vid,
			  struct vft_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct vft_req_ua),
	};
	struct vft_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.vft_ver, 0, NTMP_GEN_UA_CFGEU);
	req->ak.exact.vid = cpu_to_le16(vid);
	req->cfge = *cfge;

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_VFT_ID,
			      NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to update VFT entry, vid: %u, err: %pe\n",
			vid, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vft_update_entry);

int ntmp_vft_delete_entry(struct ntmp_user *user, u16 vid)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct vft_req_qd),
	};
	struct vft_req_qd *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.vft_ver, 0, 0);
	req->ak.exact.vid = cpu_to_le16(vid);

	/* Request header */
	len = NTMP_LEN(data.size, NTMP_STATUS_RESP_LEN);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_VFT_ID,
			      NTMP_CMD_DELETE, NTMP_AM_EXACT_KEY);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to delete VFT entry, vid: %u, err: %pe\n",
			vid, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vft_delete_entry);

int ntmp_vft_search_entry(struct ntmp_user *user, u32 *resume_eid,
			  u32 *entry_id, u16 *vid,
			  struct vft_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct vft_resp_query),
	};
	struct vft_resp_query *resp;
	struct vft_req_qd *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.vft_ver, 0, 0);
	req->ak.resume_entry_id = cpu_to_le32(*resume_eid);

	/* Request header */
	len = NTMP_LEN(sizeof(*req), data.size);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_VFT_ID,
			      NTMP_CMD_QUERY, NTMP_AM_SEARCH);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to search VFT entry, err: %pe\n",
			ERR_PTR(err));
		goto end;
	}

	if (!cbd.resp_hdr.num_matched) {
		*entry_id = NTMP_NULL_ENTRY_ID;
		*resume_eid = NTMP_NULL_ENTRY_ID;
		goto end;
	}

	resp = (struct vft_resp_query *)req;
	/* Get the response resume_entry_id to continue search */
	*resume_eid = le32_to_cpu(resp->status);
	*entry_id = le32_to_cpu(resp->entry_id);
	*cfge = resp->cfge;
	*vid = le16_to_cpu(resp->vid);

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vft_search_entry);

int ntmp_vft_query_entry_by_vid(struct ntmp_user *user, u16 vid,
				u32 *entry_id, struct vft_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct vft_resp_query),
	};
	struct vft_resp_query *resp;
	struct vft_req_qd *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd(&req->crd, user->tbl.vft_ver, 0, 0);
	req->ak.exact.vid = cpu_to_le16(vid);

	/* Request header */
	len = NTMP_LEN(sizeof(*req), data.size);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_VFT_ID,
			      NTMP_CMD_QUERY, NTMP_AM_EXACT_KEY);
	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev,
			"Failed to search VFT entry, vid:%u, err: %pe\n",
			vid, ERR_PTR(err));
		goto end;
	}

	if (!cbd.resp_hdr.num_matched) {
		*entry_id = NTMP_NULL_ENTRY_ID;
		goto end;
	}

	resp = (struct vft_resp_query *)req;
	if (vid != le16_to_cpu(resp->vid)) {
		dev_err(user->dev,
			"IPFT: query VID %u doesn't match response VID %u\n",
			le16_to_cpu(resp->vid), vid);
		err = -EINVAL;
		goto end;
	}

	*entry_id = le32_to_cpu(resp->entry_id);
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vft_query_entry_by_vid);

int ntmp_ett_add_or_update_entry(struct ntmp_user *user, u32 entry_id,
				 bool add, struct ett_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ett_req_ua),
	};
	struct ett_req_ua *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.ett_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	req->cfge = *cfge;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_ETT_ID, add ? NTMP_CMD_ADD : NTMP_CMD_UPDATE,
			      NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to %s ETT entry 0x%x, err :%ps\n",
			add ? "Add" : "Update", entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ett_add_or_update_entry);

int ntmp_ett_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_ETT_ID, user->tbl.ett_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_ett_delete_entry);

int ntmp_ett_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct ett_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ett_resp_query),
	};
	struct ntmp_req_by_eid *req;
	struct ett_resp_query *resp;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.ett_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_ETT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct ett_resp_query *)req;
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ett_query_entry);

int ntmp_isgt_add_or_update_entry(struct ntmp_user *user, u32 entry_id,
				  bool add, struct isgt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isgt_req_ua),
	};
	struct isgt_req_ua *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.isgt_ver, 0,
			  NTMP_GEN_UA_CFGEU | ISGT_UA_SGSEU, entry_id);
	req->cfge = *cfge;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_ISGT_ID, add ? NTMP_CMD_ADD : NTMP_CMD_UPDATE,
			      NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to %s ISGT entry 0x%x, err :%pe\n",
			add ? "Add" : "Update", entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isgt_add_or_update_entry);

int ntmp_isgt_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct isgt_entry_data *isgt)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct isgt_resp_query),
	};
	struct isgt_resp_query *resp;
	struct ntmp_req_by_eid *req;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.isgt_ver, 0, 0, entry_id);
	err = ntmp_query_entry_by_id(user, NTMP_ISGT_ID,
				     NTMP_LEN(sizeof(*req), data.size),
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct isgt_resp_query *)req;
	isgt->sgse = resp->sgse;
	isgt->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isgt_query_entry);

int ntmp_isgt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_ISGT_ID, user->tbl.isgt_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_isgt_delete_entry);

int ntmp_esrt_update_entry(struct ntmp_user *user, u32 entry_id,
			   struct esrt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct esrt_req_update),
	};
	struct esrt_req_update *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.esrt_ver, 0, NTMP_GEN_UA_CFGEU |
			  NTMP_GEN_UA_STSEU | ESRT_UA_SRSEU, entry_id);
	req->cfge = *cfge;

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_ESRT_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update ESRT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_esrt_update_entry);

int ntmp_esrt_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct esrt_entry_data *erst)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct esrt_resp_query),
	};
	struct esrt_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.esrt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_ESRT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct esrt_resp_query *)req;
	erst->stse = resp->stse;
	erst->cfge = resp->cfge;
	erst->srse = resp->srse;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_esrt_query_entry);

int ntmp_ect_update_entry(struct ntmp_user *user, u32 entry_id)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ntmp_req_by_eid),
	};
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd_eid(req, user->tbl.ect_ver, 0, ECT_UA_STSEU, entry_id);

	/* Request header */
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_ECT_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev, "Failed to update ECT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ect_update_entry);

int ntmp_ect_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct ect_stse_data *stse, bool update)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct ect_resp_query),
	};
	struct ect_resp_query *resp;
	struct ntmp_req_by_eid *req;
	union netc_cbd cbd;
	u16 ua = 0;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	if (update)
		/* Query, followed by Update. */
		ua = ECT_UA_STSEU;

	ntmp_fill_crd_eid(req, user->tbl.ect_ver, 0, ua, entry_id);

	/* Request header */
	len = NTMP_LEN(sizeof(*req), data.size);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_ECT_ID,
			      update ? NTMP_CMD_QU : NTMP_CMD_QUERY,
			      NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err) {
		dev_err(user->dev, "Failed to query ECT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));
		goto end;
	}

	resp = (struct ect_resp_query *)req;
	if (unlikely(entry_id != le32_to_cpu(resp->entry_id))) {
		dev_err(user->dev,
			"ECT: query EID 0x%x doesn't match response EID 0x%x",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	*stse = resp->stse;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ect_query_entry);

int ntmp_fmt_add_or_update_entry(struct ntmp_user *user, u32 entry_id,
				 bool add, struct fmt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fmt_req_ua),
	};
	struct fmt_req_ua *req;
	union netc_cbd cbd;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	/* Request data */
	ntmp_fill_crd_eid(&req->rbe, user->tbl.fmt_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);
	req->cfge = *cfge;

	/* Request header */
	len = NTMP_LEN(data.size, 0);
	ntmp_fill_request_hdr(&cbd, data.dma, len, NTMP_FMT_ID,
			      add ? NTMP_CMD_ADD : NTMP_CMD_UPDATE,
			      NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to %s FMT entry 0x%0x, err: %pe\n",
			add ? "Add" : "Update", entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fmt_add_or_update_entry);

int ntmp_fmt_delete_entry(struct ntmp_user *user, u32 entry_id)
{
	return ntmp_delete_entry_by_id(user, NTMP_FMT_ID, user->tbl.fmt_ver,
				       entry_id, NTMP_EID_REQ_LEN, 0);
}
EXPORT_SYMBOL_GPL(ntmp_fmt_delete_entry);

int ntmp_fmt_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct fmt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct fmt_resp_query),
	};
	struct ntmp_req_by_eid *req;
	struct fmt_resp_query *resp;
	u32 len;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.fmt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_FMT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct fmt_resp_query *)req;
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fmt_query_entry);

int ntmp_bpt_update_entry(struct ntmp_user *user, u32 entry_id,
			  struct bpt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct bpt_req_update),
	};
	struct bpt_req_update *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.bpt_ver, 0,
			  NTMP_GEN_UA_CFGEU | BPT_UA_BPSEU, entry_id);
	req->cfge = *cfge;
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_BPT_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to update BPT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_bpt_update_entry);

int ntmp_bpt_query_entry(struct ntmp_user *user, u32 entry_id,
			 struct bpt_entry_data *bpt)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct bpt_resp_query),
	};
	struct ntmp_req_by_eid *req;
	struct bpt_resp_query *resp;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.bpt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_BPT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct bpt_resp_query *)req;
	bpt->bpse = resp->bpse;
	bpt->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_bpt_query_entry);

int ntmp_sbpt_update_entry(struct ntmp_user *user, u32 entry_id,
			   struct sbpt_cfge_data *cfge)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct sbpt_req_update),
	};
	struct sbpt_req_update *req;
	union netc_cbd cbd;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.sbpt_ver, 0,
			  NTMP_GEN_UA_CFGEU | SBPT_UA_BPSEU, entry_id);
	req->cfge = *cfge;
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_SBPT_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to update SBPT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sbpt_update_entry);

int ntmp_sbpt_query_entry(struct ntmp_user *user, u32 entry_id,
			  struct sbpt_entry_data *sbpt)
{
	struct ntmp_dma_buf data = {
		.dev = user->dev,
		.size = sizeof(struct sbpt_resp_query),
	};
	struct sbpt_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len;
	int err;

	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.sbpt_ver, 0, 0, entry_id);
	len =  NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_SBPT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct sbpt_resp_query *)req;
	sbpt->sbpse = resp->sbpse;
	sbpt->cfge = resp->cfge;

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sbpt_query_entry);

int ntmp_fmdt_update_entry(struct ntmp_user *user, u32 entry_id,
			   u8 *data_buff, u32 data_len)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct fmdt_req_update *req;
	u32 align = data_len;
	union netc_cbd cbd;
	int err;

	if (align % FMDT_DATA_LEN_ALIGN) {
		align = DIV_ROUND_UP(align, FMDT_DATA_LEN_ALIGN);
		align *= FMDT_DATA_LEN_ALIGN;
	}

	data.size = struct_size(req, data, align);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(&req->rbe, user->tbl.fmdt_ver, 0,
			  NTMP_GEN_UA_CFGEU, entry_id);

	/* Fill configuration element data */
	memcpy(req->data, data_buff, data_len);
	ntmp_fill_request_hdr(&cbd, data.dma, NTMP_LEN(data.size, 0),
			      NTMP_FMDT_ID, NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(user, &cbd);
	if (err)
		dev_err(user->dev,
			"Failed to update FMDT entry 0x%x, err: %pe\n",
			entry_id, ERR_PTR(err));

	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fmdt_update_entry);

int ntmp_fmdt_query_entry(struct ntmp_user *user, u32 entry_id,
			  u8 *data_buff, u32 data_len)
{
	struct ntmp_dma_buf data = {.dev = user->dev};
	struct fmdt_resp_query *resp;
	struct ntmp_req_by_eid *req;
	u32 len, align = data_len;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	if (align % FMDT_DATA_LEN_ALIGN) {
		align = DIV_ROUND_UP(align, FMDT_DATA_LEN_ALIGN);
		align *= FMDT_DATA_LEN_ALIGN;
	}

	data.size = struct_size(resp, data, align);
	err = ntmp_alloc_data_mem(&data, (void **)&req);
	if (err)
		return err;

	ntmp_fill_crd_eid(req, user->tbl.fmdt_ver, 0, 0, entry_id);
	len = NTMP_LEN(sizeof(*req), data.size);
	err = ntmp_query_entry_by_id(user, NTMP_FMDT_ID, len,
				     req, data.dma, true);
	if (err)
		goto end;

	resp = (struct fmdt_resp_query *)req;
	memcpy(data_buff, resp->data, data_len);

end:
	ntmp_free_data_mem(&data);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_fmdt_query_entry);

MODULE_DESCRIPTION("NXP NETC Library");
MODULE_LICENSE("Dual BSD/GPL");
