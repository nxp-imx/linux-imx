// SPDX-License-Identifier: GPL-2.0
/*
 * System control and Management Interface (SCMI) NXP cpu Protocol
 *
 * Copyright 2025 NXP
 */

#define pr_fmt(fmt) "SCMI CPU - " fmt

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

enum scmi_imx_cpu_protocol_cmd {
	SCMI_IMX_CPU_ATTRIBUTES	= 0x3,
	SCMI_IMX_CPU_START = 0x4,
	SCMI_IMX_CPU_STOP = 0x5,
	SCMI_IMX_CPU_RESET_VECTOR_SET = 0x6,
	SCMI_IMX_CPU_INFO_GET = 0xC,
};

struct scmi_imx_cpu_info {
	u32 nr_cpu;
};

#define SCMI_IMX_CPU_PROTO_ATTR_NUM_CPUS(x)  ((x) & 0xFFFF)
struct scmi_msg_imx_cpu_protocol_attributes {
	__le32 attributes;
};

struct scmi_msg_imx_cpu_attributes_out {
	__le32 attributes;
#define	CPU_MAX_NAME	16
	u8 name[CPU_MAX_NAME];
};

struct scmi_imx_cpu_reset_vector_set_in {
	__le32 cpuid;
#define	CPU_VEC_FLAGS_RESUME	BIT(31)
#define	CPU_VEC_FLAGS_START	BIT(30)
#define	CPU_VEC_FLAGS_BOOT	BIT(29)
	__le32 flags;
	__le32 resetvectorlow;
	__le32 resetvectorhigh;
};

struct scmi_imx_cpu_info_get_out {
#define	CPU_RUN_MODE_START	0
#define	CPU_RUN_MODE_HOLD	1
#define	CPU_RUN_MODE_STOP	2
#define	CPU_RUN_MODE_SLEEP	3
	__le32 runmode;
	__le32 sleepmode;
	__le32 resetvectorlow;
	__le32 resetvectorhigh;
};

static int scmi_imx_cpu_validate_cpuid(const struct scmi_protocol_handle *ph,
				       u32 cpuid)
{
	struct scmi_imx_cpu_info *info = ph->get_priv(ph);

	if (cpuid >= info->nr_cpu)
		return -EINVAL;

	return 0;
}

static int scmi_imx_cpu_start(const struct scmi_protocol_handle *ph, u32 cpuid)
{
	struct scmi_xfer *t;
	int ret;

	ret = scmi_imx_cpu_validate_cpuid(ph, cpuid);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_CPU_START, sizeof(u32),
				      0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(cpuid, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_cpu_stop(const struct scmi_protocol_handle *ph, u32 cpuid)
{
	struct scmi_xfer *t;
	int ret;

	ret = scmi_imx_cpu_validate_cpuid(ph, cpuid);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_CPU_STOP, sizeof(u32),
				      0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(cpuid, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_cpu_reset_vector_set(const struct scmi_protocol_handle *ph,
					 u32 cpuid, u64 vector, bool start,
					 bool boot, bool resume)
{
	struct scmi_imx_cpu_reset_vector_set_in *in;
	struct scmi_xfer *t;
	int ret;

	ret = scmi_imx_cpu_validate_cpuid(ph, cpuid);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_CPU_RESET_VECTOR_SET, sizeof(*in),
				      0, &t);
	if (ret)
		return ret;

	in = t->tx.buf;
	in->cpuid = cpu_to_le32(cpuid);
	in->flags = start ? CPU_VEC_FLAGS_START : 0;
	in->flags |= boot ? CPU_VEC_FLAGS_BOOT : 0;
	in->flags |= resume ? CPU_VEC_FLAGS_BOOT : 0;
	in->resetvectorlow = cpu_to_le32(lower_32_bits(vector));
	in->resetvectorhigh = cpu_to_le32(upper_32_bits(vector));
	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_cpu_started(const struct scmi_protocol_handle *ph, u32 cpuid,
				bool *started)
{
	struct scmi_imx_cpu_info_get_out *out;
	struct scmi_xfer *t;
	int ret;

	*started = false;
	ret = scmi_imx_cpu_validate_cpuid(ph, cpuid);
	if (ret)
		return ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_CPU_INFO_GET, sizeof(u32),
				      0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(cpuid, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		if ((out->runmode == CPU_RUN_MODE_START) ||
		    (out->runmode == CPU_RUN_MODE_SLEEP))
			*started = true;
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static const struct scmi_imx_cpu_proto_ops scmi_imx_cpu_proto_ops = {
	.cpu_reset_vector_set = scmi_imx_cpu_reset_vector_set,
	.cpu_start = scmi_imx_cpu_start,
	.cpu_started = scmi_imx_cpu_started,
	.cpu_stop = scmi_imx_cpu_stop,
};

static int scmi_imx_cpu_protocol_attributes_get(const struct scmi_protocol_handle *ph,
						struct scmi_imx_cpu_info *info)
{
	struct scmi_msg_imx_cpu_protocol_attributes *attr;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0,
				      sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		info->nr_cpu = SCMI_IMX_CPU_PROTO_ATTR_NUM_CPUS(attr->attributes);
		dev_info(ph->dev, "i.MX SM CPU: %d cpus\n",
			 info->nr_cpu);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_cpu_attributes_get(const struct scmi_protocol_handle *ph,
				       u32 cpuid, struct scmi_imx_cpu_info *info)
{
	struct scmi_msg_imx_cpu_attributes_out *out;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_CPU_ATTRIBUTES, sizeof(u32), 0, &t);
	if (ret)
		return ret;

	put_unaligned_le32(cpuid, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		dev_info(ph->dev, "i.MX CPU: name: %s\n", out->name);
	} else {
		dev_err(ph->dev, "i.MX cpu: Failed to get info of cpu(%u)\n", cpuid);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}


static int scmi_imx_cpu_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_imx_cpu_info *info;
	u32 version;
	int ret, i;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	dev_info(ph->dev, "NXP SM CPU Protocol Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	info = devm_kzalloc(ph->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = scmi_imx_cpu_protocol_attributes_get(ph, info);
	if (ret)
		return ret;

	for (i = 0; i < info->nr_cpu; i++) {
		ret = scmi_imx_cpu_attributes_get(ph, i, info);
		if (ret)
			return ret;
	}

	return ph->set_priv(ph, info, version);
}

static const struct scmi_protocol scmi_imx_cpu = {
	.id = SCMI_PROTOCOL_IMX_CPU,
	.owner = THIS_MODULE,
	.instance_init = &scmi_imx_cpu_protocol_init,
	.ops = &scmi_imx_cpu_proto_ops,
	.supported_version = SCMI_PROTOCOL_SUPPORTED_VERSION,
	.vendor_id = "NXP",
	.sub_vendor_id = "IMX",
};
module_scmi_protocol(scmi_imx_cpu);

MODULE_DESCRIPTION("i.MX SCMI CPU driver");
MODULE_LICENSE("GPL");
