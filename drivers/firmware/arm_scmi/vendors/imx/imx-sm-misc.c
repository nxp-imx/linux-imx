// SPDX-License-Identifier: GPL-2.0
/*
 * System control and Management Interface (SCMI) NXP MISC Protocol
 *
 * Copyright 2024 NXP
 */

#define pr_fmt(fmt) "SCMI Notifications MISC - " fmt

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_imx_protocol.h>

#include "../../protocols.h"
#include "../../notify.h"

#define SCMI_PROTOCOL_SUPPORTED_VERSION		0x10000

#define MAX_MISC_CTRL_SOURCES			GENMASK(15, 0)

enum scmi_imx_misc_protocol_cmd {
	SCMI_IMX_MISC_CTRL_SET	= 0x3,
	SCMI_IMX_MISC_CTRL_GET	= 0x4,
	SCMI_IMX_MISC_DISCOVER_BUILDINFO = 0x6,
	SCMI_IMX_MISC_SI_INFO = 0xB,
	SCMI_IMX_MISC_CFG_INFO = 0xC,
	SCMI_IMX_MISC_SYSLOG = 0xD,
	SCMI_IMX_MISC_BOARD_INFO = 0xE,
	SCMI_IMX_MISC_CTRL_NOTIFY = 0x8,
};

struct scmi_imx_misc_info {
	u32 version;
	u32 nr_dev_ctrl;
	u32 nr_brd_ctrl;
	u32 nr_reason;
};

struct scmi_msg_imx_misc_protocol_attributes {
	__le32 attributes;
};

#define GET_BRD_CTRLS_NR(x)	le32_get_bits((x), GENMASK(31, 24))
#define GET_REASONS_NR(x)	le32_get_bits((x), GENMASK(23, 16))
#define GET_DEV_CTRLS_NR(x)	le32_get_bits((x), GENMASK(15, 0))
#define BRD_CTRL_START_ID	BIT(15)

struct scmi_imx_misc_ctrl_set_in {
	__le32 id;
	__le32 num;
	__le32 value[];
};

struct scmi_imx_misc_ctrl_notify_in {
	__le32 ctrl_id;
	__le32 flags;
};

struct scmi_imx_misc_ctrl_notify_payld {
	__le32 ctrl_id;
	__le32 flags;
};

struct scmi_imx_misc_ctrl_get_out {
	__le32 num;
	__le32 val[];
};

struct scmi_imx_misc_buildinfo_out {
	__le32 buildnum;
	__le32 buildcommit;
	u8 builddate[MISC_MAX_BUILDDATE];
	u8 buildtime[MISC_MAX_BUILDTIME];
};

struct scmi_imx_misc_board_info_out {
	__le32 attributes;
	u8 brdname[MISC_MAX_BRDNAME];
};

struct scmi_imx_misc_cfg_info_out {
	__le32 msel;
	u8 cfgname[MISC_MAX_CFGNAME];
};

struct scmi_imx_misc_si_info_out {
	__le32 deviceid;
	__le32 sirev;
	__le32 partnum;
	u8 siname[MISC_MAX_SINAME];
};

struct scmi_imx_misc_syslog_in {
	__le32 flags;
	__le32 index;
};

#define REMAINING(x)	le32_get_bits((x), GENMASK(31, 20))
#define RETURNED(x)	le32_get_bits((x), GENMASK(11, 0))

struct scmi_imx_misc_syslog_out {
	__le32 numlogflags;
	__le32 syslog[];
};

static int scmi_imx_misc_attributes_get(const struct scmi_protocol_handle *ph,
					struct scmi_imx_misc_info *mi)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_msg_imx_misc_protocol_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0,
				      sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		mi->nr_dev_ctrl = GET_DEV_CTRLS_NR(attr->attributes);
		mi->nr_brd_ctrl = GET_BRD_CTRLS_NR(attr->attributes);
		mi->nr_reason = GET_REASONS_NR(attr->attributes);
		dev_info(ph->dev, "i.MX MISC NUM DEV CTRL: %d, NUM BRD CTRL: %d,NUM Reason: %d\n",
			 mi->nr_dev_ctrl, mi->nr_brd_ctrl, mi->nr_reason);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_ctrl_validate_id(const struct scmi_protocol_handle *ph,
					  u32 ctrl_id)
{
	struct scmi_imx_misc_info *mi = ph->get_priv(ph);

	/*
	 * [0,      BRD_CTRL_START_ID) is for Dev Ctrl which is SOC related
	 * [BRD_CTRL_START_ID, 0xffff) is for Board Ctrl which is board related
	 */
	if (ctrl_id < BRD_CTRL_START_ID && ctrl_id > mi->nr_dev_ctrl)
		return -EINVAL;
	if (ctrl_id >= BRD_CTRL_START_ID + mi->nr_brd_ctrl)
		return -EINVAL;

	return 0;
}

static int scmi_imx_misc_ctrl_notify(const struct scmi_protocol_handle *ph,
				     u32 ctrl_id, u32 evt_id, u32 flags)
{
	struct scmi_imx_misc_ctrl_notify_in *in;
	struct scmi_xfer *t;
	int ret;

	ret = scmi_imx_misc_ctrl_validate_id(ph, ctrl_id);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_NOTIFY,
				      sizeof(*in), 0, &t);
	if (ret)
		return ret;

	in = t->tx.buf;
	in->ctrl_id = cpu_to_le32(ctrl_id);
	in->flags = cpu_to_le32(flags);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_imx_misc_ctrl_set_notify_enabled(const struct scmi_protocol_handle *ph,
				      u8 evt_id, u32 src_id, bool enable)
{
	int ret;

	/* misc_ctrl_req_notify is for enablement */
	if (enable)
		return 0;

	ret = scmi_imx_misc_ctrl_notify(ph, src_id, evt_id, 0);
	if (ret)
		dev_err(ph->dev, "FAIL_ENABLED - evt[%X] src[%d] - ret:%d\n",
			evt_id, src_id, ret);

	return ret;
}

static void *
scmi_imx_misc_ctrl_fill_custom_report(const struct scmi_protocol_handle *ph,
				      u8 evt_id, ktime_t timestamp,
				      const void *payld, size_t payld_sz,
				      void *report, u32 *src_id)
{
	const struct scmi_imx_misc_ctrl_notify_payld *p = payld;
	struct scmi_imx_misc_ctrl_notify_report *r = report;

	if (sizeof(*p) != payld_sz)
		return NULL;

	r->timestamp = timestamp;
	r->ctrl_id = le32_to_cpu(p->ctrl_id);
	r->flags = le32_to_cpu(p->flags);
	if (src_id)
		*src_id = r->ctrl_id;
	dev_dbg(ph->dev, "%s: ctrl_id: %d flags: %d\n", __func__,
		r->ctrl_id, r->flags);

	return r;
}

static const struct scmi_event_ops scmi_imx_misc_event_ops = {
	.set_notify_enabled = scmi_imx_misc_ctrl_set_notify_enabled,
	.fill_custom_report = scmi_imx_misc_ctrl_fill_custom_report,
};

static const struct scmi_event scmi_imx_misc_events[] = {
	{
		.id = SCMI_EVENT_IMX_MISC_CONTROL,
		.max_payld_sz = sizeof(struct scmi_imx_misc_ctrl_notify_payld),
		.max_report_sz = sizeof(struct scmi_imx_misc_ctrl_notify_report),
	},
};

static struct scmi_protocol_events scmi_imx_misc_protocol_events = {
	.queue_sz = SCMI_PROTO_QUEUE_SZ,
	.ops = &scmi_imx_misc_event_ops,
	.evts = scmi_imx_misc_events,
	.num_events = ARRAY_SIZE(scmi_imx_misc_events),
	.num_sources = MAX_MISC_CTRL_SOURCES,
};

static int scmi_imx_misc_ctrl_get(const struct scmi_protocol_handle *ph,
				  u32 ctrl_id, u32 *num, u32 *val)
{
	struct scmi_imx_misc_ctrl_get_out *out;
	struct scmi_xfer *t;
	int ret, i;
	int max_msg_size = ph->hops->get_max_msg_size(ph);
	int max_num = (max_msg_size - sizeof(*out)) / sizeof(__le32);

	ret = scmi_imx_misc_ctrl_validate_id(ph, ctrl_id);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_GET, sizeof(u32),
				      0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(ctrl_id, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		*num = le32_to_cpu(out->num);

		if (*num >= max_num ||
		    *num * sizeof(__le32) > t->rx.len - sizeof(__le32)) {
			ph->xops->xfer_put(ph, t);
			return -EINVAL;
		}

		for (i = 0; i < *num; i++)
			val[i] = le32_to_cpu(out->val[i]);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_ctrl_set(const struct scmi_protocol_handle *ph,
				  u32 ctrl_id, u32 num, u32 *val)
{
	struct scmi_imx_misc_ctrl_set_in *in;
	struct scmi_xfer *t;
	int ret, i;
	int max_msg_size = ph->hops->get_max_msg_size(ph);
	int max_num = (max_msg_size - sizeof(*in)) / sizeof(__le32);

	ret = scmi_imx_misc_ctrl_validate_id(ph, ctrl_id);
	if (ret)
		return ret;

	if (num > max_num)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_SET,
				      sizeof(*in) + num * sizeof(__le32), 0, &t);
	if (ret)
		return ret;

	in = t->tx.buf;
	in->id = cpu_to_le32(ctrl_id);
	in->num = cpu_to_le32(num);
	for (i = 0; i < num; i++)
		in->value[i] = cpu_to_le32(val[i]);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_discover_build_info(const struct scmi_protocol_handle *ph,
					struct scmi_imx_misc_system_info *info)
{
	struct scmi_imx_misc_buildinfo_out *out;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_DISCOVER_BUILDINFO, 0,
				      sizeof(*out), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		info->buildnum = le32_to_cpu(out->buildnum);
		info->buildcommit = le32_to_cpu(out->buildcommit);
		strscpy(info->date, out->builddate, MISC_MAX_BUILDDATE);
		strscpy(info->time, out->buildtime, MISC_MAX_BUILDTIME);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_board_info(const struct scmi_protocol_handle *ph,
				    struct scmi_imx_misc_system_info *info)
{
	struct scmi_imx_misc_board_info_out *out;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_BOARD_INFO, 0, sizeof(*out), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		info->brd_attributes = le32_to_cpu(out->attributes);
		strscpy(info->brdname, out->brdname, MISC_MAX_BRDNAME);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_cfg_info(const struct scmi_protocol_handle *ph,
				  struct scmi_imx_misc_system_info *info)
{
	struct scmi_imx_misc_cfg_info_out *out;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CFG_INFO, 0, sizeof(*out), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		info->msel = le32_to_cpu(out->msel);
		strscpy(info->cfgname, out->cfgname, MISC_MAX_CFGNAME);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_silicon_info(const struct scmi_protocol_handle *ph,
				      struct scmi_imx_misc_system_info *info)
{
	struct scmi_imx_misc_si_info_out *out;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_SI_INFO, 0, sizeof(*out), &t);
	if (ret)
		return ret;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		info->deviceid = le32_to_cpu(out->deviceid);
		info->sirev = le32_to_cpu(out->sirev);
		info->partnum = le32_to_cpu(out->partnum);
		strscpy(info->siname, out->siname, MISC_MAX_SINAME);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

struct scmi_imx_misc_syslog_ipriv {
	u32 *array;
};

static void iter_misc_syslog_prepare_message(void *message, u32 desc_index,
					     const void *priv)
{
	struct scmi_imx_misc_syslog_in *msg = message;

	msg->flags = cpu_to_le32(0);
	msg->index = cpu_to_le32(desc_index);
}

static int iter_misc_syslog_update_state(struct scmi_iterator_state *st,
					 const void *response, void *priv)
{
	const struct scmi_imx_misc_syslog_out *r = response;

	st->num_returned = RETURNED(r->numlogflags);
	st->num_remaining = REMAINING(r->numlogflags);

	return 0;
}

static int
iter_misc_syslog_process_response(const struct scmi_protocol_handle *ph,
				  const void *response,
				  struct scmi_iterator_state *st, void *priv)
{
	const struct scmi_imx_misc_syslog_out *r = response;
	struct scmi_imx_misc_syslog_ipriv *p = priv;

	p->array[st->desc_index + st->loop_idx] =
		le32_to_cpu(r->syslog[st->loop_idx]);

	return 0;
}

static int scmi_imx_misc_syslog(const struct scmi_protocol_handle *ph, u16 size,
				void *array)
{
	struct scmi_iterator_ops ops = {
		.prepare_message = iter_misc_syslog_prepare_message,
		.update_state = iter_misc_syslog_update_state,
		.process_response = iter_misc_syslog_process_response,
	};
	struct scmi_imx_misc_syslog_ipriv ipriv = {
		.array = array,
	};
	void *iter;

	if (!array || !size)
		return -EINVAL;

	iter = ph->hops->iter_response_init(ph, &ops, size, SCMI_IMX_MISC_SYSLOG,
					    sizeof(struct scmi_imx_misc_syslog_in),
					    &ipriv);
	if (IS_ERR(iter))
		return PTR_ERR(iter);

	return ph->hops->iter_response_run(iter);
}

static const struct scmi_imx_misc_proto_ops scmi_imx_misc_proto_ops = {
	.misc_board_info = scmi_imx_misc_board_info,
	.misc_cfg_info = scmi_imx_misc_cfg_info,
	.misc_ctrl_set = scmi_imx_misc_ctrl_set,
	.misc_ctrl_get = scmi_imx_misc_ctrl_get,
	.misc_ctrl_req_notify = scmi_imx_misc_ctrl_notify,
	.misc_discover_build_info = scmi_imx_discover_build_info,
	.misc_silicon_info = scmi_imx_misc_silicon_info,
	.misc_syslog = scmi_imx_misc_syslog,
};

static int scmi_imx_misc_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_imx_misc_info *minfo;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	dev_info(ph->dev, "NXP SM MISC Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	minfo = devm_kzalloc(ph->dev, sizeof(*minfo), GFP_KERNEL);
	if (!minfo)
		return -ENOMEM;

	ret = scmi_imx_misc_attributes_get(ph, minfo);
	if (ret)
		return ret;

	return ph->set_priv(ph, minfo, version);
}

static const struct scmi_protocol scmi_imx_misc = {
	.id = SCMI_PROTOCOL_IMX_MISC,
	.owner = THIS_MODULE,
	.instance_init = &scmi_imx_misc_protocol_init,
	.ops = &scmi_imx_misc_proto_ops,
	.events = &scmi_imx_misc_protocol_events,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = "NXP",
	.sub_vendor_id = "IMX",
};
module_scmi_protocol(scmi_imx_misc);

MODULE_DESCRIPTION("i.MX SCMI MISC driver");
MODULE_LICENSE("GPL");
