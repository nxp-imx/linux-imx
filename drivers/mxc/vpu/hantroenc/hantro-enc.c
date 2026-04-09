// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 * Copyright (c) 2015-2020, VeriSilicon Inc.
 * Copyright (c) 2011-2014, Google Inc.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/iopoll.h>
#include <linux/freezer.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include "hantroenc-h1.h"
#include "hantroenc-vc8000e.h"

#define HANTRO_ENC_NAME				"hx280enc"

#define HANTRO_ENC_TIMEOUT_MS			300
#define HANTRO_ENC_MAX_CORE_NUM			1
#define HANTRO_ENC_CORE_INFO_AMOUNT_MASK	0x70000000
#define HANTRO_ENC_CORE_INFO_IDS_MASK		0xFF

#define hantro_wait_event_interruptible(wq_head, condition)				\
({											\
	int wait_ret = 0;								\
	do {										\
		if (wait_ret == -ERESTARTSYS && freezing(current))			\
			clear_thread_flag(TIF_SIGPENDING);				\
		wait_ret = wait_event_freezable(wq_head, condition);			\
	} while (wait_ret == -ERESTARTSYS && freezing(current));			\
	wait_ret;									\
})

#define hantro_wait_event_timeout(wq_head, condition, timeout)				\
({											\
	int wait_ret = 0;								\
	unsigned long _timeout = timeout;						\
	unsigned long stop = jiffies + _timeout;					\
	do {										\
		if (wait_ret == -ERESTARTSYS && freezing(current))			\
			clear_thread_flag(TIF_SIGPENDING);				\
		_timeout = stop - jiffies;						\
		if ((long)_timeout <= 0) {						\
			wait_ret = -ERESTARTSYS;					\
			break;								\
		}									\
		wait_ret = wait_event_freezable_timeout(wq_head, condition, _timeout);	\
	} while (wait_ret == -ERESTARTSYS && freezing(current));			\
	wait_ret;									\
})

enum {
	HANTRO_ENC_CMD_ID_INVALID = 0,
	HANTRO_ENC_CMD_ID_G_IO_ADDR,
	HANTRO_ENC_CMD_ID_G_IO_SIZE,
	HANTRO_ENC_CMD_ID_RESERVE_HW,
	HANTRO_ENC_CMD_ID_RELEASE_HW,
	HANTRO_ENC_CMD_ID_G_CORE_NUM,
	HANTRO_ENC_CMD_ID_ENABLE_CORE,
	HANTRO_ENC_CMD_ID_WAIT_CORE,
	HANTRO_ENC_CMD_ID_WRITE_REGS,
	HANTRO_ENC_CMD_ID_READ_REGS,

	HANTRO_ENC_CMD_ID_MAX
};

enum {
	HANTRO_STREAM_TYPE_UNKNOWN,
	HANTRO_STREAM_TYPE_H264,
	HANTRO_STREAM_TYPE_HEVC,
	HANTRO_STREAM_TYPE_VP8,
	HANTRO_STREAM_TYPE_MAX_NUM,
};

struct hantro_enc_done_status {
	u32 core_id;
	u32 irq_status;
};

struct hantro_cmd_param {
	u32 val;
	struct hantro_enc_regs_buffer regs;
	struct hantro_enc_done_status done_status;
};

struct hantro_enc_device;
struct hantro_enc_core;

struct hantro_enc_resource {
	const char *device_name;
	const u32 *hw_ids;
	const u32 num_regs;
	const bool resource_shared_inter_cores;

	const u32 irq_mask;
	const u32 reg_enable;
	const u32 reg_irq;
	const u32 reg_write1_clr;

	int (*get_cmd_id)(unsigned int cmd);
	int (*get_cmd_param)(int id, unsigned long arg, struct hantro_cmd_param *param);
	int (*put_cmd_result)(int id, unsigned long arg, struct hantro_cmd_param *param,
			      struct hantro_enc_device *encoder);
	void (*clear_irq)(struct hantro_enc_core *core, u32 irq_status);
	void (*clear_status_on_ready)(struct hantro_enc_core *core);
	u32 (*get_encode_fmt)(struct hantro_enc_core *core);
	void (*read_format_config)(struct hantro_enc_core *core);
};

struct hantro_enc_core {
	int id;
	unsigned long base_addr;
	size_t base_size;

	int irq;
	void __iomem *reg_base;
	u32 *mirror_regs;
	u32 num_regs;
	const struct hantro_enc_resource *resource;
	struct hantro_enc_device *dev;

	unsigned long format_config;

	spinlock_t lock;  /* lock for core */
	u32 hw_id;
	u32 irq_status;
	u32 is_valid;
	u32 is_reserved;
	u32 is_enabled;
	u32 irq_received;
	u32 format;
	unsigned long frame_num[HANTRO_STREAM_TYPE_MAX_NUM];
	struct file *filp;

	struct dentry *debugfs;
};

struct hantro_enc_device {
	struct device *dev;
	struct hantro_enc_core cores[HANTRO_ENC_MAX_CORE_NUM];
	u32 num_cores;
	u32 cores_mask;
	struct clk_bulk_data *clks;
	int num_clks;
	const struct hantro_enc_resource *resource;

	wait_queue_head_t hw_queue;
	wait_queue_head_t enc_done;
	struct semaphore shared_resource_sem;
	struct file *filp;

	dev_t devt;
	struct cdev cdev;
	struct class *enc_class;

	struct dentry *debugfs;
};

static u32 hantro_enc_readl(struct hantro_enc_core *core, u32 addr)
{
	return readl(core->reg_base + addr);
}

static void hantro_enc_writel(struct hantro_enc_core *core, u32 value, u32 addr)
{
	writel(value, core->reg_base + addr);
}

static u32 hantro_enc_readl_mirror(struct hantro_enc_core *core, u32 addr)
{
	return core->mirror_regs[addr >> 2];
}

static void hantro_enc_update_mirror_regs(struct hantro_enc_core *core)
{
	for (int i = 0; i < core->num_regs; i++)
		core->mirror_regs[i] = hantro_enc_readl(core, i * 4);

	core->mirror_regs[core->resource->reg_irq >> 2] = core->irq_status;
}

static void hantro_enc_reset_core(struct hantro_enc_core *core)
{
	int i;

	if (!core->is_valid)
		return;

	hantro_enc_writel(core, 0, core->resource->reg_enable);
	for (i = 4; i < core->base_size; i += 4)
		hantro_enc_writel(core, 0, i);

	hantro_enc_update_mirror_regs(core);
}

static struct hantro_enc_core *hantro_enc_get_core(struct hantro_enc_device *encoder, u32 idx)
{
	if (idx >= encoder->num_cores)
		return NULL;

	return &encoder->cores[idx];
}

static int hantro_enc_get_cmd_id_h1(unsigned int cmd)
{
	int id = HANTRO_ENC_CMD_ID_INVALID;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(H1ENC_IOCGHWOFFSET):
		id = HANTRO_ENC_CMD_ID_G_IO_ADDR;
		break;
	case _IOC_NR(H1ENC_IOCGHWIOSIZE):
		id = HANTRO_ENC_CMD_ID_G_IO_SIZE;
		break;
	case _IOC_NR(H1ENC_IOCH_ENC_RESERVE):
		id = HANTRO_ENC_CMD_ID_RESERVE_HW;
		break;
	case _IOC_NR(H1ENC_IOCH_ENC_RELEASE):
		id = HANTRO_ENC_CMD_ID_RELEASE_HW;
		break;
	case _IOC_NR(H1ENC_IOCG_EN_CORE):
		id = HANTRO_ENC_CMD_ID_ENABLE_CORE;
		break;
	case _IOC_NR(H1ENC_IOCG_CORE_WAIT):
		id = HANTRO_ENC_CMD_ID_WAIT_CORE;
		break;
	case _IOC_NR(H1ENC_IOC_WRITE_REGS):
		id = HANTRO_ENC_CMD_ID_WRITE_REGS;
		break;
	case _IOC_NR(H1ENC_IOC_READ_REGS):
		id = HANTRO_ENC_CMD_ID_READ_REGS;
		break;
	}

	return id;
}

static int hantro_enc_get_cmd_id_vc8000e(unsigned int cmd)
{
	int id = HANTRO_ENC_CMD_ID_INVALID;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(VC8000E_IOCGHWOFFSET):
		id = HANTRO_ENC_CMD_ID_G_IO_ADDR;
		break;
	case _IOC_NR(VC8000E_IOCGHWIOSIZE):
		id = HANTRO_ENC_CMD_ID_G_IO_SIZE;
		break;
	case _IOC_NR(VC8000E_IOCH_ENC_RESERVE):
		id = HANTRO_ENC_CMD_ID_RESERVE_HW;
		break;
	case _IOC_NR(VC8000E_IOCH_ENC_RELEASE):
		id = HANTRO_ENC_CMD_ID_RELEASE_HW;
		break;
	case _IOC_NR(VC8000E_IOCG_CORE_NUM):
		id = HANTRO_ENC_CMD_ID_G_CORE_NUM;
		break;
	case _IOC_NR(VC8000E_IOCG_EN_CORE):
		id = HANTRO_ENC_CMD_ID_ENABLE_CORE;
		break;
	case _IOC_NR(VC8000E_IOCG_CORE_WAIT):
		id = HANTRO_ENC_CMD_ID_WAIT_CORE;
		break;
	case _IOC_NR(VC8000E_IOC_WRITE_REGS):
		id = HANTRO_ENC_CMD_ID_WRITE_REGS;
		break;
	case _IOC_NR(VC8000E_IOC_READ_REGS):
		id = HANTRO_ENC_CMD_ID_READ_REGS;
		break;
	}


	return id;
}

static int hantro_enc_get_cmd_param_h1(int id, unsigned long arg, struct hantro_cmd_param *param)
{
	int ret = 0;

	switch (id) {
	case HANTRO_ENC_CMD_ID_WRITE_REGS:
	case HANTRO_ENC_CMD_ID_READ_REGS:
		ret = copy_from_user(&param->regs, (void __user *)arg, sizeof(param->regs));
		break;
		param->val = 1;
		break;
	case HANTRO_ENC_CMD_ID_G_IO_ADDR:
	case HANTRO_ENC_CMD_ID_G_IO_SIZE:
		param->val = 0;
		break;
	case HANTRO_ENC_CMD_ID_RESERVE_HW:
	case HANTRO_ENC_CMD_ID_RELEASE_HW:
	case HANTRO_ENC_CMD_ID_WAIT_CORE:
		param->val = 1;
		break;
	case HANTRO_ENC_CMD_ID_ENABLE_CORE:
		param->val = 0;
		break;
	case HANTRO_ENC_CMD_ID_G_CORE_NUM:
		break;
	default:
		break;
	}

	return ret;
}

static int hantro_enc_get_cmd_param_vc8000e(int id, unsigned long arg,
					    struct hantro_cmd_param *param)
{
	int ret = 0;

	switch (id) {
	case HANTRO_ENC_CMD_ID_WRITE_REGS:
	case HANTRO_ENC_CMD_ID_READ_REGS:
		ret = copy_from_user(&param->regs, (void __user *)arg, sizeof(param->regs));
		break;
	case HANTRO_ENC_CMD_ID_G_IO_ADDR:
	case HANTRO_ENC_CMD_ID_G_IO_SIZE:
	case HANTRO_ENC_CMD_ID_RESERVE_HW:
	case HANTRO_ENC_CMD_ID_RELEASE_HW:
	case HANTRO_ENC_CMD_ID_ENABLE_CORE:
	case HANTRO_ENC_CMD_ID_WAIT_CORE:
		get_user(param->val, (u32 __user *)arg);
		break;
	case HANTRO_ENC_CMD_ID_G_CORE_NUM:
		break;
	default:
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static int hantro_enc_get_regs_compat32(unsigned long arg, struct hantro_enc_regs_buffer *regs)
{
	void __user *up = compat_ptr(arg);
	u32 tmp1, tmp2;
	int ret;

	ret = get_user(regs->core_id, (u32 __user *)up);
	if (ret)
		return ret;
	ret = get_user(tmp1, (u32 __user *)up + 4);
	if (ret)
		return ret;
	ret = get_user(regs->offset, (u32 __user *)up + 8);
	if (ret)
		return ret;
	ret = get_user(regs->size, (u32 __user *)up + 12);
	if (ret)
		return ret;
	ret = get_user(tmp2, (u32 __user *)up + 16);
	if (ret)
		return ret;

	regs->regs = (__force u32 *)compat_ptr(tmp1);
	regs->reserved = (__force u32 *)compat_ptr(tmp2);

	return 0;
}
#endif

static int hantro_enc_put_cmd_result(int id, unsigned long arg, struct hantro_cmd_param *param,
				     struct hantro_enc_device *encoder)
{
	struct hantro_enc_core *core = NULL;
	int ret = 0;

	switch (id) {
	case HANTRO_ENC_CMD_ID_G_IO_ADDR:
		core = hantro_enc_get_core(encoder, param->val);
		if (core)
			put_user(core->base_addr, (u32 __user *)arg);
		else
			ret = -EINVAL;
		break;
	case HANTRO_ENC_CMD_ID_G_IO_SIZE:
		core = hantro_enc_get_core(encoder, param->val);
		if (core)
			put_user(core->base_size, (u32 __user *)arg);
		else
			ret = -EINVAL;
		break;
	case HANTRO_ENC_CMD_ID_RESERVE_HW:
		put_user(param->val, (u32 __user *)arg);
		break;
	case HANTRO_ENC_CMD_ID_RELEASE_HW:
	case HANTRO_ENC_CMD_ID_ENABLE_CORE:
	case HANTRO_ENC_CMD_ID_WRITE_REGS:
	case HANTRO_ENC_CMD_ID_READ_REGS:
		break;
	case HANTRO_ENC_CMD_ID_G_CORE_NUM:
		put_user(encoder->num_cores, (u32 __user *)arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int hantro_enc_put_cmd_result_h1(int id, unsigned long arg, struct hantro_cmd_param *param,
					struct hantro_enc_device *encoder)
{
	struct hantro_enc_core *core = NULL;
	int ret = 0;

	switch (id) {
	case HANTRO_ENC_CMD_ID_WAIT_CORE:
		core = hantro_enc_get_core(encoder, 0);
		if (core)
			ret = copy_to_user((void __user *)arg, core->mirror_regs, core->base_size);
		else
			ret = -EINVAL;
		break;
	default:
		ret = hantro_enc_put_cmd_result(id, arg, param, encoder);
		break;
	}

	return ret;
}

static int hantro_enc_put_cmd_result_vc8000e(int id, unsigned long arg,
					     struct hantro_cmd_param *param,
					     struct hantro_enc_device *encoder)
{
	int ret = 0;

	switch (id) {
	case HANTRO_ENC_CMD_ID_WAIT_CORE:
		put_user(param->done_status.irq_status, (u32 __user *)arg);
		break;
	default:
		ret = hantro_enc_put_cmd_result(id, arg, param, encoder);
		break;
	}

	return ret;
}

static void hantro_enc_clear_irq_h1(struct hantro_enc_core *core, u32 irq_status)
{
	u32 is_write1_clr = 0;

	if (core->resource->reg_write1_clr)
		is_write1_clr = FIELD_GET(H1_WRITE1_CLEAR_MASK,
					  hantro_enc_readl(core, core->resource->reg_write1_clr));

	if (is_write1_clr)
		hantro_enc_writel(core, irq_status & 0x111, core->resource->reg_irq);
	else
		hantro_enc_writel(core, irq_status & (~0x111), core->resource->reg_irq);
}

static void hantro_enc_clear_status_bits_h1(struct hantro_enc_core *core)
{
	u32 is_write1_clr = 0;
	u32 irq_status;

	irq_status = core->mirror_regs[core->resource->reg_irq >> 2];
	if (core->resource->reg_write1_clr) {
		u32 reg_write1_clr = core->resource->reg_write1_clr;

		is_write1_clr = FIELD_GET(H1_WRITE1_CLEAR_MASK,
					  hantro_enc_readl_mirror(core, reg_write1_clr));
	}
	if (is_write1_clr)
		hantro_enc_writel(core, irq_status, core->resource->reg_irq);
	else
		hantro_enc_writel(core, irq_status & (~0xf7d), core->resource->reg_irq);
}

static void hantro_enc_clear_irq_vc8000e(struct hantro_enc_core *core, u32 irq_status)
{
	u32 major_id;
	u32 clr;

	/*  Disable HW when buffer over-flow happen
	 *  HW behavior changed in over-flow
	 *    in-pass, HW cleanup HWIF_ENC_E auto
	 *    new version:  ask SW cleanup HWIF_ENC_E when buffer over-flow
	 */
	if (irq_status & HANTROENC_INT_STATUS_BUF_FULL)
		hantro_enc_writel(core, 0, core->resource->reg_enable);

	major_id = FIELD_GET(0xFF00, core->hw_id);
	clr = major_id >= 0x61 ? irq_status : (irq_status & (~0x1FD));
	hantro_enc_writel(core, clr, core->resource->reg_irq);
}

static void hantro_enc_read_format_config_h1(struct hantro_enc_core *core)
{
	u32 config_val = hantro_enc_readl(core, 63 * 4);

	if (FIELD_GET(BIT(27), config_val))
		set_bit(HANTRO_STREAM_TYPE_H264, &core->format_config);
	if (FIELD_GET(BIT(26), config_val))
		set_bit(HANTRO_STREAM_TYPE_VP8, &core->format_config);
}

static void hantro_enc_read_format_config_vc8000e(struct hantro_enc_core *core)
{
	u32 config_val = hantro_enc_readl(core, 80 * 4);

	if (FIELD_GET(BIT(31), config_val))
		set_bit(HANTRO_STREAM_TYPE_H264, &core->format_config);
	if (FIELD_GET(BIT(27), config_val))
		set_bit(HANTRO_STREAM_TYPE_HEVC, &core->format_config);
}

static const char *hantro_enc_get_fmt_name(u32 type)
{
	switch (type) {
	case HANTRO_STREAM_TYPE_H264:
		return "h264";
	case HANTRO_STREAM_TYPE_HEVC:
		return "hevc";
	case HANTRO_STREAM_TYPE_VP8:
		return "vp8";
	default:
		return "unknown";
	}
}

static u32 hantro_enc_get_fmt_h1(struct hantro_enc_core *core)
{
	u32 val = hantro_enc_readl(core, 0x38);
	u32 type = FIELD_GET(0x6, val);

	switch (type) {
	case 1:
		return HANTRO_STREAM_TYPE_VP8;
	case 3:
		return HANTRO_STREAM_TYPE_H264;
	default:
		return HANTRO_STREAM_TYPE_UNKNOWN;
	}
}

static u32 hantro_enc_get_fmt_vc8000e(struct hantro_enc_core *core)
{
	u32 val = hantro_enc_readl(core, 0x10);
	u32 type = FIELD_GET(0xe0000000, val);

	switch (type) {
	case 1:
		return HANTRO_STREAM_TYPE_HEVC;
	case 2:
		return HANTRO_STREAM_TYPE_H264;
	default:
		return HANTRO_STREAM_TYPE_UNKNOWN;
	}
}

static int hantro_enc_reserve_shared_resource(struct hantro_enc_device *encoder, struct file *filp)
{
	if (!encoder->resource->resource_shared_inter_cores)
		return 0;

	if (down_timeout(&encoder->shared_resource_sem, msecs_to_jiffies(HANTRO_ENC_TIMEOUT_MS)))
		return -ERESTARTSYS;

	encoder->filp = filp;

	return 0;
}

static void hantro_enc_release_shared_resource(struct hantro_enc_device *encoder, struct file *filp)
{
	if (!encoder->resource->resource_shared_inter_cores)
		return;

	if (!encoder->filp || encoder->filp != filp)
		return;
	encoder->filp = NULL;
	up(&encoder->shared_resource_sem);
}

static int hantro_enc_check_core_occupation(struct hantro_enc_core *core, struct file *filp)
{
	int ret = 0;

	scoped_guard(spinlock_irqsave, &core->lock) {
		if (!core->is_reserved) {
			core->is_reserved = 1;
			core->filp = filp;
			ret = 1;
		}
	}

	if (!ret)
		return ret;

	if (pm_runtime_resume_and_get(core->dev->dev)) {
		scoped_guard(spinlock_irqsave, &core->lock) {
			core->filp = NULL;
			core->is_reserved = 0;
			core->irq_received = 0;
			core->irq_status = 0;
			core->is_enabled = 0;
			ret = 0;
		}
	} else {
		hantro_enc_writel(core, 0, core->resource->reg_enable);
		hantro_enc_update_mirror_regs(core);
	}

	return ret;
}

static void hantro_enc_release_core(struct hantro_enc_core *core, struct file *filp)
{
	u32 val;

	if (core->is_reserved && core->filp == filp) {
		val = hantro_enc_readl(core, core->resource->reg_enable);
		val &= ~1;
		hantro_enc_writel(core, val, core->resource->reg_enable);

		pm_runtime_mark_last_busy(core->dev->dev);
		pm_runtime_put_autosuspend(core->dev->dev);

		scoped_guard(spinlock_irqsave, &core->lock) {
			core->filp = NULL;
			core->is_reserved = 0;
			core->irq_received = 0;
			core->irq_status = 0;
			core->is_enabled = 0;

		}
		wake_up_interruptible_all(&core->dev->hw_queue);
	}
}

static int hantro_enc_get_workable_core(struct hantro_enc_device *encoder,
					u32 *core_info, struct file *filp)
{
	struct hantro_enc_core *core;
	int ret = 0;
	unsigned long cores_mask;
	u32 cores_ret;
	u32 required_num;
	u32 cores_num;
	int id;

	if (!encoder || !core_info)
		return ret;

	cores_mask = *core_info;
	required_num = FIELD_GET(HANTRO_ENC_CORE_INFO_AMOUNT_MASK, cores_mask) + 1;
	cores_mask = FIELD_GET(HANTRO_ENC_CORE_INFO_IDS_MASK, cores_mask) & encoder->cores_mask;

	cores_num = 0;
	cores_ret = 0;
	for_each_set_bit(id, &cores_mask, encoder->num_cores) {
		core = hantro_enc_get_core(encoder, id);
		if (!core)
			continue;

		if (core->is_valid && hantro_enc_check_core_occupation(core, filp)) {
			cores_ret |= BIT(id);
			cores_num++;
			ret = 1;
		}

		if (cores_num >= required_num)
			break;
	}

	if (cores_ret)
		*core_info = FIELD_PREP(HANTRO_ENC_CORE_INFO_AMOUNT_MASK, cores_num) |
			     FIELD_PREP(HANTRO_ENC_CORE_INFO_IDS_MASK, cores_ret);

	return ret;
}

static void hantro_enc_put_work_core(struct hantro_enc_device *encoder,
				     u32 core_info, struct file *filp)
{
	struct hantro_enc_core *core;
	unsigned long cores_mask;
	int id;

	if (!encoder || !core_info)
		return;

	cores_mask = core_info;
	cores_mask = FIELD_GET(HANTRO_ENC_CORE_INFO_IDS_MASK, cores_mask) & encoder->cores_mask;

	for_each_set_bit(id, &cores_mask, encoder->num_cores) {
		core = hantro_enc_get_core(encoder, id);
		if (!core)
			continue;

		hantro_enc_release_core(core, filp);
	}
}

static void hantro_enc_release_encoder(struct hantro_enc_device *encoder, struct file *filp,
				       u32 core_info)
{
	hantro_enc_put_work_core(encoder, core_info, filp);
	hantro_enc_release_shared_resource(encoder, filp);
}

static int hantro_enc_reserve_encoder(struct hantro_enc_device *encoder, struct file *filp,
				      u32 *core_info)
{
	ktime_t ts;
	int ret;

	if (!core_info)
		return -EINVAL;

	ret = hantro_enc_reserve_shared_resource(encoder, filp);
	if (ret)
		return ret;

	ts = ktime_get_raw();
	if (hantro_wait_event_interruptible(encoder->hw_queue,
					    hantro_enc_get_workable_core(encoder,
									 core_info,
									 filp))) {
		dev_err(encoder->dev, "Reserve encoder interrupted, %lld\n", ktime_get_raw() - ts);
		return -ERESTARTSYS;
	}

	ts = ktime_get_raw() - ts;
	dev_dbg(encoder->dev, "Reserving encoder takes %lld us\n", ts / NSEC_PER_USEC);

	return 0;
}

static void hantro_enc_writeback_mirror_regs(struct hantro_enc_core *core)
{
	for (int i = 1; i < core->num_regs; i++)
		hantro_enc_writel(core, core->mirror_regs[i], i * 4);
}

static int hantro_enc_enable_core(struct hantro_enc_core *core)
{
	u32 value;

	if (!core)
		return -EINVAL;

	if (!core->is_reserved)
		return -EPERM;

	hantro_enc_writeback_mirror_regs(core);

	/*
	 * Write memory barrier to ensure all configuration register writes are completed
	 * before enabling the encoder. Without this, the hardware may see the ENABLE write
	 * before configuration writes complete, resulting in inconsistent encoded bitstream.
	 */
	wmb();

	value = hantro_enc_readl(core, core->resource->reg_enable);
	value |= 0x1;
	hantro_enc_writel(core, value, core->resource->reg_enable);

	scoped_guard(spinlock_irqsave, &core->lock)
		core->is_enabled = 1;
	if (core->resource->get_encode_fmt) {
		core->format = core->resource->get_encode_fmt(core);
		dev_dbg(core->dev->dev, "Encode %s frame\n", hantro_enc_get_fmt_name(core->format));
		if (core->format < HANTRO_STREAM_TYPE_MAX_NUM)
			core->frame_num[core->format]++;
	}

	return 0;
}

static int hantro_enc_disable_core(struct hantro_enc_core *core)
{
	u32 value;

	if (!core)
		return -EINVAL;

	if (!core->is_reserved)
		return -EPERM;

	value = hantro_enc_readl(core, core->resource->reg_enable);
	value &= ~1;
	hantro_enc_writel(core, value, core->resource->reg_enable);
	scoped_guard(spinlock_irqsave, &core->lock)
		core->is_enabled = 0;

	return 0;
}

static void hantro_enc_clear_status_on_ready(struct hantro_enc_core *core)
{
	if (core->resource->clear_status_on_ready)
		core->resource->clear_status_on_ready(core);
}

static int hantro_enc_check_done_status(struct hantro_enc_device *encoder, u32 core_info,
					struct hantro_enc_done_status *done_status)
{
	struct hantro_enc_core *core;
	unsigned long cores_mask;
	int done = 0;
	int id;

	if (!encoder)
		return -EINVAL;

	cores_mask = core_info;
	cores_mask = FIELD_GET(HANTRO_ENC_CORE_INFO_IDS_MASK, cores_mask) & encoder->cores_mask;

	for_each_set_bit(id, &cores_mask, encoder->num_cores) {
		core = hantro_enc_get_core(encoder, id);
		if (!core)
			continue;
		if (core->irq_received) {
			if (done_status) {
				done_status->core_id = id;
				done_status->irq_status = core->irq_status;
			}
			done = 1;
			break;
		}
	}

	return done;
}

static void hantro_enc_cancel(struct hantro_enc_device *encoder, u32 core_info)
{
	struct hantro_enc_core *core;
	unsigned long cores_mask;
	int id;

	if (!encoder)
		return;

	cores_mask = core_info;
	cores_mask = FIELD_GET(HANTRO_ENC_CORE_INFO_IDS_MASK, cores_mask) & encoder->cores_mask;
	for_each_set_bit(id, &cores_mask, encoder->num_cores) {
		core = hantro_enc_get_core(encoder, id);
		if (!core)
			continue;
		hantro_enc_disable_core(core);
	}
}

static int hantro_enc_wait_core(struct hantro_enc_device *encoder, u32 core_info,
				struct hantro_enc_done_status *done_status)
{
	struct hantro_enc_done_status status;

	if (!encoder)
		return -EINVAL;

	if (hantro_wait_event_timeout(encoder->enc_done,
				      hantro_enc_check_done_status(encoder, core_info, &status),
				      msecs_to_jiffies(HANTRO_ENC_TIMEOUT_MS)) <= 0) {
		dev_err(encoder->dev, "wait interrupt timeout, core_info = 0x%x!\n", core_info);
		hantro_enc_cancel(encoder, core_info);
		return -EINVAL;
	}

	if (done_status)
		*done_status = status;

	return status.core_id;
}

static int hantro_enc_write_regs(struct hantro_enc_device *encoder,
				 struct hantro_enc_regs_buffer *regs)
{
	struct hantro_enc_core *core;
	u32 *reg_buf;
	int count;
	int ret;

	if (!encoder || !regs)
		return -EINVAL;

	core = hantro_enc_get_core(encoder, regs->core_id);
	if (!core) {
		dev_err(encoder->dev, "Invalid core id %d for writing regs\n", regs->core_id);
		return -EINVAL;
	}

	if (!regs->size)
		return -EINVAL;
	if ((regs->offset & 0x3) || (regs->size & 0x3))
		return -EINVAL;
	if (regs->offset + regs->size > core->base_size)
		return -EINVAL;

	count = regs->size >> 2;
	reg_buf = &core->mirror_regs[regs->offset >> 2];
	ret = copy_from_user(reg_buf, (void __user *)regs->regs, regs->size);
	if (ret)
		return -EINVAL;

	return 0;
}

static int hantro_enc_read_regs(struct hantro_enc_device *encoder,
				struct hantro_enc_regs_buffer *regs)
{
	struct hantro_enc_core *core;
	u32 *reg_buf;
	int count;
	int ret;

	if (!encoder || !regs)
		return -EINVAL;

	core = hantro_enc_get_core(encoder, regs->core_id);
	if (!core) {
		dev_err(encoder->dev, "Invalid core id %d for writing regs\n", regs->core_id);
		return -EINVAL;
	}

	if (!regs->size)
		return -EINVAL;
	if ((regs->offset & 0x3) || (regs->size & 0x3))
		return -EINVAL;
	if (regs->offset + regs->size > core->base_size)
		return -EINVAL;

	count = regs->size >> 2;
	reg_buf = &core->mirror_regs[regs->offset >> 2];

	ret = copy_to_user((void __user *)regs->regs, reg_buf, regs->size);
	if (ret)
		return -EINVAL;

	return 0;
}

static int hantro_enc_open(struct inode *inode, struct file *filp)
{
	struct hantro_enc_device *encoder;

	encoder = container_of(inode->i_cdev, struct hantro_enc_device, cdev);
	filp->private_data = encoder;

	return 0;
}

static int hantro_enc_release(struct inode *inode, struct file *filp)
{
	struct hantro_enc_device *encoder = filp->private_data;

	hantro_enc_release_encoder(encoder, filp, encoder->cores_mask);

	return 0;
}

static long hantro_enc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct hantro_enc_device *encoder = filp->private_data;
	struct hantro_cmd_param param = { 0 };
	int cmd_id;
	int err = 0;

	if (_IOC_TYPE(cmd) != HX280ENC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HX280ENC_IOC_MAXNR)
		return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	cmd_id = encoder->resource->get_cmd_id(cmd);
	if (cmd_id == HANTRO_ENC_CMD_ID_INVALID)
		return -ENOTTY;

	err = encoder->resource->get_cmd_param(cmd_id, arg, &param);
	if (err)
		return err;

	dev_dbg(encoder->dev, "ioctl %d\n", cmd_id);

	switch (cmd_id) {
	case HANTRO_ENC_CMD_ID_G_IO_ADDR:
	case HANTRO_ENC_CMD_ID_G_IO_SIZE:
	case HANTRO_ENC_CMD_ID_G_CORE_NUM:
		break;
	case HANTRO_ENC_CMD_ID_RESERVE_HW:
		err = hantro_enc_reserve_encoder(encoder, filp, &param.val);
		break;
	case HANTRO_ENC_CMD_ID_RELEASE_HW:
		hantro_enc_release_encoder(encoder, filp, param.val);
		break;
	case HANTRO_ENC_CMD_ID_ENABLE_CORE:
		err = hantro_enc_enable_core(hantro_enc_get_core(encoder, param.val));
		break;
	case HANTRO_ENC_CMD_ID_WAIT_CORE:
		err = hantro_enc_wait_core(encoder, param.val, &param.done_status);
		break;
	case HANTRO_ENC_CMD_ID_WRITE_REGS:
		err = hantro_enc_write_regs(encoder, &param.regs);
		break;
	case HANTRO_ENC_CMD_ID_READ_REGS:
		err = hantro_enc_read_regs(encoder, &param.regs);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0)
		return err;

	if (encoder->resource->put_cmd_result(cmd_id, arg, &param, encoder))
		return -EINVAL;

	return err;
}

static long hantro_enc_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct hantro_enc_device *encoder = filp->private_data;
	struct hantro_cmd_param param = { 0 };
	int cmd_id;
	int err;

	cmd_id = encoder->resource->get_cmd_id(cmd);
	if (cmd_id == HANTRO_ENC_CMD_ID_INVALID)
		return -ENOTTY;

	switch (cmd_id) {
	case HANTRO_ENC_CMD_ID_WRITE_REGS:
		err = hantro_enc_get_regs_compat32(arg, &param.regs);
		if (err)
			return err;
		err = hantro_enc_write_regs(encoder, &param.regs);
		break;
	case HANTRO_ENC_CMD_ID_READ_REGS:
		err = hantro_enc_get_regs_compat32(arg, &param.regs);
		if (err)
			return err;
		err = hantro_enc_read_regs(encoder, &param.regs);
		break;
	default:
		err = compat_ptr_ioctl(filp, cmd, arg);
		break;
	}

	return err;
}

static int hantro_enc_mmap(struct file *filp, struct vm_area_struct *vm)
{
	struct hantro_enc_device *encoder = filp->private_data;

	for (int i = 0; i < encoder->num_cores; i++) {
		if (vm->vm_pgoff == (encoder->cores[i].base_addr >> PAGE_SHIFT)) {
			vm_flags_set(vm, VM_IO);
			return remap_vmalloc_range(vm, encoder->cores[i].mirror_regs, 0);
		}
	}

	return -EINVAL;
}

static const struct file_operations hantro_enc_fops = {
	.owner = THIS_MODULE,
	.open = hantro_enc_open,
	.release = hantro_enc_release,
	.unlocked_ioctl = hantro_enc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantro_enc_ioctl32,
#endif
	.fasync = NULL,
	.mmap = hantro_enc_mmap,
};

static inline int hantro_enc_read_irq_received(struct hantro_enc_core *core)
{
	guard(spinlock_irqsave)(&core->lock);

	return core->irq_received;
}

static int hantro_enc_pause_core(struct hantro_enc_core *core)
{
	int ret;

	if (!core->is_reserved)
		return 0;

	if (core->is_enabled) {
		ktime_t ts = ktime_get_raw();
		u32 data;

		ret = read_poll_timeout(hantro_enc_read_irq_received, data, data,
					10, HANTRO_ENC_TIMEOUT_MS * USEC_PER_MSEC, false, core);
		if (ret) {
			dev_err(core->dev->dev, "wait core[%d] done timeout\n", core->id);
			return -EINVAL;
		}

		ts = ktime_get_raw() - ts;
		dev_dbg(core->dev->dev, "wait core[%d] %lld us\n", core->id, ts / NSEC_PER_USEC);
	}

	dev_dbg(core->dev->dev, "suspend, irq_status = 0x%x\n", core->irq_status);

	return 0;
}

static int hantro_enc_pause_cores(struct hantro_enc_device *encoder)
{
	struct hantro_enc_core *core;
	int ret = 0;

	for (int i = 0; i < encoder->num_cores; i++) {
		core = hantro_enc_get_core(encoder, i);
		if (!core || !core->is_valid)
			continue;

		ret = hantro_enc_pause_core(core);
		if (ret)
			break;
	}

	return ret;
}

static irqreturn_t hantro_enc_isr(int irq, void *dev_id)
{
	struct hantro_enc_core *core = dev_id;
	u32 irq_status;

	scoped_guard(spinlock_irqsave, &core->lock) {
		if (!core->is_reserved) {
			dev_dbg(core->dev->dev,
				"irq received but core[%d] is not reserved\n", core->id);
			return IRQ_HANDLED;
		}
	}

	irq_status = hantro_enc_readl(core, core->resource->reg_irq);
	dev_dbg(core->dev->dev, "irq status 0x%x on core[%d]\n", irq_status, core->id);

	if (irq_status & HANTROENC_INT_STATUS_ENA) {
		core->resource->clear_irq(core, irq_status);

		irq_status &= core->resource->irq_mask;
		if (irq_status == HANTROENC_INT_STATUS_SLICE_DONE) {
			dev_dbg(core->dev->dev, "Slice ready IRQ handled\n");
			return IRQ_HANDLED;
		}

		scoped_guard(spinlock_irqsave, &core->lock) {
			core->irq_status = irq_status;
			hantro_enc_update_mirror_regs(core);
			hantro_enc_clear_status_on_ready(core);
			core->irq_received = 1;
		}
		wake_up_all(&core->dev->enc_done);
	}

	return IRQ_HANDLED;
}

static int hantro_enc_dbg_core(struct seq_file *s, void *data)
{
	struct hantro_enc_core *core = s->private;
	char str[128];
	int num;

	num = scnprintf(str, sizeof(str), "hw id : 0x%x\n", core->hw_id);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"reserved: %d, enabled: %d, irq received: %d, status 0x%x\n",
			core->is_reserved, core->is_enabled, core->irq_received, core->irq_status);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "last format: %s\n",
			hantro_enc_get_fmt_name(core->format));
	if (seq_write(s, str, num))
		return 0;

	for (int i = HANTRO_STREAM_TYPE_H264; i < HANTRO_STREAM_TYPE_MAX_NUM; i++) {
		if (core->format_config && !test_bit(i, &core->format_config))
			continue;
		num = scnprintf(str, sizeof(str), "[%.4s] = %ld\n",
				hantro_enc_get_fmt_name(i), core->frame_num[i]);
		if (seq_write(s, str, num))
			return 0;
	}

	return 0;
}

static int hantro_enc_dbg_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hantro_enc_dbg_core, inode->i_private);
}

static const struct file_operations hantro_enc_dbg_fops = {
	.owner = THIS_MODULE,
	.open = hantro_enc_dbg_open,
	.release = single_release,
	.read = seq_read,
};

static void hantro_enc_create_debugfs(struct hantro_enc_core *core)
{
	char name[64];

	if (!core || !core->dev || !core->dev->debugfs)
		return;

	if (!core->id)
		scnprintf(name, sizeof(name), "%s", core->resource->device_name);
	else
		scnprintf(name, sizeof(name), "%s.%d", core->resource->device_name, core->id);

	core->debugfs = debugfs_create_file((const char *)name,
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    core->dev->debugfs,
					    core,
					    &hantro_enc_dbg_fops);
}

static bool hantro_enc_check_hw_id(struct hantro_enc_device *encoder, u32 hw_id)
{
	bool valid = false;
	int i = 0;

	while (encoder->resource->hw_ids[i]) {
		if ((hw_id >> 16) == (encoder->resource->hw_ids[i] >> 16)) {
			valid = true;
			break;
		}
		i++;
	}

	return valid;
}

static int hantro_enc_init_cores(struct hantro_enc_device *encoder)
{
	struct hantro_enc_core *core;
	u32 hw_id;
	u32 size;
	int ret;

	ret = pm_runtime_resume_and_get(encoder->dev);
	if (ret)
		return ret;

	for (int i = 0; i < encoder->num_cores; i++) {
		core = &encoder->cores[i];
		hw_id = hantro_enc_readl(core, 0);
		if (!hantro_enc_check_hw_id(encoder, hw_id)) {
			dev_err(encoder->dev, "HW is not found at 0x%lx\n", core->base_addr);
			continue;
		}
		size = PAGE_ALIGN(core->resource->num_regs * sizeof(u32));
		core->mirror_regs = (u32 *)vmalloc_user(size);
		if (!core->mirror_regs)
			continue;

		dev_info(encoder->dev, "core[%d] hw id 0x%x\n", i, hw_id);
		core->num_regs = core->resource->num_regs;
		core->hw_id = hw_id;
		core->is_valid = 1;
		spin_lock_init(&core->lock);
		hantro_enc_reset_core(core);
		if (core->resource->read_format_config)
			core->resource->read_format_config(core);
		hantro_enc_create_debugfs(core);
	}

	pm_runtime_mark_last_busy(encoder->dev);
	pm_runtime_put_autosuspend(encoder->dev);

	return 0;
}

static int hantro_enc_init(struct hantro_enc_device *encoder)
{
	struct device *enc_dev;
	int ret;

	ret = alloc_chrdev_region(&encoder->devt, 0, 1, HANTRO_ENC_NAME);
	if (ret < 0) {
		dev_err(encoder->dev, "unable to alloc chrdev region\n");
		return ret;
	}

	encoder->enc_class = class_create(HANTRO_ENC_NAME);
	if (IS_ERR(encoder->enc_class)) {
		ret = PTR_ERR(encoder->enc_class);
		encoder->enc_class = NULL;
		goto error_clean_chrdev_region;
	}

	cdev_init(&encoder->cdev, &hantro_enc_fops);
	encoder->cdev.owner = THIS_MODULE;

	ret = cdev_add(&encoder->cdev, encoder->devt, 1);
	if (ret)
		goto error_clean_class;

	enc_dev = device_create(encoder->enc_class, NULL, encoder->devt,
				NULL, encoder->resource->device_name);
	if (IS_ERR(enc_dev)) {
		ret = PTR_ERR(enc_dev);
		goto error_del_cdev;
	}

	return 0;

error_del_cdev:
	cdev_del(&encoder->cdev);
error_clean_class:
	class_destroy(encoder->enc_class);
error_clean_chrdev_region:
	unregister_chrdev_region(encoder->devt, 1);
	return ret;
}

static void hantro_enc_cleanup(struct hantro_enc_device *encoder)
{
	device_destroy(encoder->enc_class, encoder->devt);
	cdev_del(&encoder->cdev);
	class_destroy(encoder->enc_class);
	unregister_chrdev_region(encoder->devt, 1);
}

static int hantro_enc_probe(struct platform_device *pdev)
{
	const struct hantro_enc_resource *resource;
	struct hantro_enc_device *encoder;
	int ret;

	resource = device_get_match_data(&pdev->dev);
	if (!resource) {
		dev_err(&pdev->dev, "missing match data\n");
		return -EINVAL;
	}

	encoder = devm_kzalloc(&pdev->dev, sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->dev = &pdev->dev;
	encoder->resource = resource;
	sema_init(&encoder->shared_resource_sem, 1);
	init_waitqueue_head(&encoder->hw_queue);
	init_waitqueue_head(&encoder->enc_done);

	ret = devm_clk_bulk_get_all(&pdev->dev, &encoder->clks);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}
	encoder->num_clks = ret;

	for (int i = 0; i < HANTRO_ENC_MAX_CORE_NUM; i++) {
		struct resource *res;

		encoder->cores[i].id = i;
		encoder->cores[i].reg_base = devm_platform_get_and_ioremap_resource(pdev, i, &res);
		if (IS_ERR(encoder->cores[i].reg_base)) {
			ret = PTR_ERR(encoder->cores[i].reg_base);
			break;
		}
		if (resource_size(res) < encoder->resource->num_regs * 4) {
			ret = -ENXIO;
			break;
		}
		encoder->cores[i].base_addr = res->start;
		encoder->cores[i].base_size = encoder->resource->num_regs * 4;
		encoder->cores[i].dev = encoder;
		encoder->cores[i].resource = encoder->resource;

		encoder->cores[i].irq = platform_get_irq(pdev, i);
		if (encoder->cores[i].irq < 0) {
			ret = -ENXIO;
			break;
		}

		ret = devm_request_threaded_irq(&pdev->dev, encoder->cores[i].irq,
						hantro_enc_isr, NULL,
						0, HANTRO_ENC_NAME, &encoder->cores[i]);
		if (ret) {
			dev_err(&pdev->dev, "fail to register interrupt handler: %d\n", ret);
			break;
		}
		encoder->num_cores++;
	}
	if (!encoder->num_cores) {
		dev_err(encoder->dev, "Failed to get vpu encoder core\n");
		return ret;
	}
	encoder->cores_mask = BIT(encoder->num_cores) - 1;

	encoder->debugfs = debugfs_create_dir("hantro_enc", NULL);
	dev_set_drvdata(&pdev->dev, encoder);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	hantro_enc_init_cores(encoder);

	ret = hantro_enc_init(encoder);
	if (ret) {
		pm_runtime_dont_use_autosuspend(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	return 0;
}

static void hantro_enc_remove(struct platform_device *pdev)
{
	struct hantro_enc_device *encoder = dev_get_drvdata(&pdev->dev);

	hantro_enc_cleanup(encoder);

	for (int i = 0; i < encoder->num_cores; i++) {
		vfree((const void *)encoder->cores[i].mirror_regs);
		debugfs_remove(encoder->cores[i].debugfs);
	}

	debugfs_remove(encoder->debugfs);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static int hantro_enc_runtime_suspend(struct device *dev)
{
	struct hantro_enc_device *encoder = dev_get_drvdata(dev);

	if (encoder->num_clks)
		clk_bulk_disable_unprepare(encoder->num_clks, encoder->clks);

	return 0;
}

static int hantro_enc_runtime_resume(struct device *dev)
{
	struct hantro_enc_device *encoder = dev_get_drvdata(dev);
	int ret = 0;

	if (encoder->num_clks)
		ret = clk_bulk_prepare_enable(encoder->num_clks, encoder->clks);

	return ret;
}

static int __maybe_unused hantro_enc_suspend(struct device *dev)
{
	struct hantro_enc_device *encoder = dev_get_drvdata(dev);
	int ret;

	ret = hantro_enc_pause_cores(encoder);
	if (!ret)
		pm_runtime_force_suspend(dev);
	return ret;
}

static const struct dev_pm_ops hantro_enc_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_enc_runtime_suspend, hantro_enc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_enc_suspend, pm_runtime_force_resume)
};

static u32 hantro_h1_supported_hw_ids[] = {
	0x62800000,
	0x72800000,
	0x82700000,
	0x82900000,
	0x48310000,
	0 /* sentinel */
};

static struct hantro_enc_resource hantro_h1_id = {
	.device_name = "mxc_hantro_h1",
	.hw_ids = hantro_h1_supported_hw_ids,
	.num_regs = 500,
	.irq_mask = 0x3bf,
	.reg_enable = 0x38,
	.reg_irq = 0x4,
	.reg_write1_clr = 0x4a0,
	.resource_shared_inter_cores = false,
	.get_cmd_id = hantro_enc_get_cmd_id_h1,
	.get_cmd_param = hantro_enc_get_cmd_param_h1,
	.put_cmd_result = hantro_enc_put_cmd_result_h1,
	.clear_irq = hantro_enc_clear_irq_h1,
	.clear_status_on_ready = hantro_enc_clear_status_bits_h1,
	.get_encode_fmt = hantro_enc_get_fmt_h1,
	.read_format_config = hantro_enc_read_format_config_h1,
};

static u32 hantro_vc8000e_supported_hw_ids[] = {
	0x48320100,
	0x80006000,
	0 /* sentinel */
};

static struct hantro_enc_resource hantro_vc8000e_id = {
	.device_name = "mxc_hantro_vc8000e",
	.hw_ids = hantro_vc8000e_supported_hw_ids,
	.num_regs = 479,
	.irq_mask = 0x3ff,
	.reg_enable = 0x14,
	.reg_irq = 0x4,
	.resource_shared_inter_cores = false,
	.get_cmd_id = hantro_enc_get_cmd_id_vc8000e,
	.get_cmd_param = hantro_enc_get_cmd_param_vc8000e,
	.put_cmd_result = hantro_enc_put_cmd_result_vc8000e,
	.clear_irq = hantro_enc_clear_irq_vc8000e,
	.get_encode_fmt = hantro_enc_get_fmt_vc8000e,
	.read_format_config = hantro_enc_read_format_config_vc8000e,
};

static const struct of_device_id hantro_enc_of_match[] = {
	{ .compatible = "nxp,imx8mp-hantro-vc8000e", .data = &hantro_vc8000e_id },
	{ .compatible = "nxp,imx8mm-hantro-h1", .data = &hantro_h1_id },
	{/* sentinel */}
};

static struct platform_driver hantro_enc_driver = {
	.driver = {
		.name = "mxc_hantro_encoder",
		.of_match_table = hantro_enc_of_match,
		.pm = &hantro_enc_pm_ops,
	},
	.probe = hantro_enc_probe,
	.remove = hantro_enc_remove,
};

module_platform_driver(hantro_enc_driver);

MODULE_DESCRIPTION("Hantro H1 and VC8000E encoder driver for i.MX8MP and i.MX8MM");
MODULE_LICENSE("GPL");
