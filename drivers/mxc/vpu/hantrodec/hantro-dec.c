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
#include <linux/bitfield.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/mm.h>
#include <linux/iopoll.h>
#include <linux/freezer.h>
#include <linux/compat.h>
#include <linux/regulator/consumer.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/hantrodec.h>
#include "dwl_defs.h"

#ifdef CONFIG_DEVFREQ_THERMAL
#include <linux/thermal.h>
static int hantro_clock_ratio = 2;
module_param(hantro_clock_ratio, int, 0644);
#endif

#define HANTRO_DEC_NAME					"hantrodec"
#define HANTRO_DEC_DEVICE_NAME				"mxc_hantro"
#define HANTRO_DEC_MAX_CORE_NUM				2
#define HANTRO_DEC_CLK_VOL_THR				(600000000)
#define HANTRO_DEC_TIMEOUT_MS				300

#define HANTRO_DEC_HW_ID_MASK				0xFFFF0000
#define HANTRO_DEC_CORE_FULL_IDS_BITMAP			0x00000003

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
	HANTRO_DEC_TYPE_G1,
	HANTRO_DEC_TYPE_G2,
	HANTRO_DEC_TYPE_MAX
};

enum {
	DWL_CLIENT_TYPE_H264_DEC  = 1,
	DWL_CLIENT_TYPE_MPEG4_DEC = 2,
	DWL_CLIENT_TYPE_JPEG_DEC  = 3,
	DWL_CLIENT_TYPE_PP        = 4,
	DWL_CLIENT_TYPE_VC1_DEC   = 5,
	DWL_CLIENT_TYPE_MPEG2_DEC = 6,
	DWL_CLIENT_TYPE_VP6_DEC   = 7,
	DWL_CLIENT_TYPE_AVS_DEC   = 8,
	DWL_CLIENT_TYPE_RV_DEC    = 9,
	DWL_CLIENT_TYPE_VP8_DEC   = 10,
	DWL_CLIENT_TYPE_VP9_DEC   = 11,
	DWL_CLIENT_TYPE_HEVC_DEC  = 12,
	DWL_CLIENT_TYPE_MAX,
};

struct hantro_dec_interface;

struct hantro_dec_resource {
	const u32 *hw_ids;
	const u32 type;
	const u32 num_regs;
	const u32 num_org_regs;
	const bool resource_shared_inter_cores;
};

struct hantro_dec_core {
	struct list_head list;
	struct device *dev;

	int id;
	unsigned long base_addr;
	size_t base_size;

	struct clk_bulk_data *clks;
	int num_clks;

	const struct hantro_dec_resource *resource;
	struct hantro_dec_interface *iface;

	int irq;
	void __iomem *reg_base;
	u32 *mirror_regs;
	u32 *temp_regs;
	u32 num_regs;
	u32 num_org_regs;

	struct regulator *regulator;

	unsigned long config;

	wait_queue_head_t dec_done;

	spinlock_t lock;  /* lock for core */
	u32 hw_id;
	u32 irq_status;
	u32 is_valid;
	u32 is_reserved;
	u32 is_enabled;
	u32 irq_received;
	struct file *dec_filp;
#ifdef CONFIG_DEVFREQ_THERMAL
	spinlock_t thermal_lock;  /* lock for cooling state */
	int thermal_event;
	int thermal_cur;
	unsigned long clk_rate_def;
	struct thermal_cooling_device *cooling;
#endif
	void (*cooling_update)(struct hantro_dec_core *);

	u32 format;
	unsigned long frame_num[DWL_CLIENT_TYPE_MAX];

	struct dentry *debugfs;
};

struct hantro_dec_interface {
	struct device *dev;
	dev_t devt;
	struct cdev cdev;
	struct class *dec_class;

	struct list_head cores;
	u32 num_cores;
	unsigned long idle_core_ids_bitmap;
	u32 max_clk_rate;

	atomic_t ref_cnt;
	spinlock_t lock;  /* lock for interface */

	u32 num_active_cores;
	bool resource_shared_inter_cores;

	wait_queue_head_t hw_queue;

	struct dentry *debugfs;
};

static const char *hantro_dec_get_fmt_name(u32 format)
{
	switch (format) {
	case DWL_CLIENT_TYPE_H264_DEC:
		return "h264";
	case DWL_CLIENT_TYPE_MPEG4_DEC:
		return "mpeg4";
	case DWL_CLIENT_TYPE_JPEG_DEC:
		return "jpeg";
	case DWL_CLIENT_TYPE_VC1_DEC:
		return "vc1";
	case DWL_CLIENT_TYPE_MPEG2_DEC:
		return "mpeg2";
	case DWL_CLIENT_TYPE_VP6_DEC:
		return "vp6";
	case DWL_CLIENT_TYPE_AVS_DEC:
		return "avs";
	case DWL_CLIENT_TYPE_RV_DEC:
		return "rv";
	case DWL_CLIENT_TYPE_VP8_DEC:
		return "vp8";
	case DWL_CLIENT_TYPE_VP9_DEC:
		return "vp9";
	case DWL_CLIENT_TYPE_HEVC_DEC:
		return "hevc";
	default:
		return "unknown";
	}
}

static inline bool hantro_dec_is_g1(struct hantro_dec_core *core)
{
	return core->resource->type == HANTRO_DEC_TYPE_G1 ? true : false;
}

static u32 hantro_dec_readl(struct hantro_dec_core *core, u32 addr)
{
	return readl(core->reg_base + addr);
}

static void hantro_dec_writel(struct hantro_dec_core *core, u32 value, u32 addr)
{
	writel(value, core->reg_base + addr);
}

static bool hantro_dec_core_has_format(struct hantro_dec_core *core, u32 format)
{
	return test_bit(format, &core->config) ? true : false;
}

static struct hantro_dec_core *hantro_dec_get_core_by_id(struct hantro_dec_interface *iface, u32 id)
{
	struct hantro_dec_core *core;

	guard(spinlock)(&iface->lock);

	list_for_each_entry(core, &iface->cores, list) {
		if (core->id == id)
			return core;
	}

	return NULL;
}

static struct hantro_dec_core *hantro_dec_get_core_by_format(struct hantro_dec_interface *iface,
							     u32 format)
{
	struct hantro_dec_core *core;

	guard(spinlock)(&iface->lock);

	list_for_each_entry(core, &iface->cores, list) {
		if (hantro_dec_core_has_format(core, format))
			return core;
	}

	return NULL;
}

static void hantro_dec_update_mirror_regs(struct hantro_dec_core *core)
{
	for (int i = 0; i < core->num_regs; i++)
		core->mirror_regs[i] = hantro_dec_readl(core, i * 4);
}

static void hantro_dec_reset_core(struct hantro_dec_core *core)
{
	u32 status;
	int i;

	if (!core->is_valid)
		return;

	status = hantro_dec_readl(core, HANTRODEC_IRQ_STAT_DEC_OFF);
	if (status & HANTRODEC_DEC_E) {
		status = HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		hantro_dec_writel(core, status, HANTRODEC_IRQ_STAT_DEC_OFF);
	}

	if (hantro_dec_is_g1(core))
		hantro_dec_writel(core, 0, HANTRO_IRQ_STAT_PP_OFF);	/* reset PP */

	for (i = 1; i < core->num_regs; i++)
		hantro_dec_writel(core, 0, i * 4);

	hantro_dec_update_mirror_regs(core);
}

static bool hantro_dec_get_workable_core(struct hantro_dec_interface *iface,
					struct file *filp, unsigned long format,
					struct hantro_dec_core **ret_core)
{
	struct hantro_dec_core *core = NULL, *tmp;
	bool got = false;

	guard(spinlock)(&iface->lock);

	if (iface->resource_shared_inter_cores && iface->num_active_cores)
		return false;

	list_for_each_entry(tmp, &iface->cores, list) {
		if (hantro_dec_core_has_format(tmp, format)) {
			core = tmp;
			break;
		}
	}
	if (!core)
		return false;

	scoped_guard(spinlock_irqsave, &core->lock) {
		if (core->is_valid && !core->is_reserved) {
			core->is_reserved = 1;
			core->dec_filp = filp;
			core->format = format;
			got = true;
			iface->num_active_cores++;
			if (iface->num_active_cores > 1)
				dev_dbg(iface->dev, "active cores %d\n", iface->num_active_cores);
		}
	}

	if (got && ret_core)
		*ret_core = core;

	return got;
}

static void hantro_dec_put_core(struct hantro_dec_core *core, struct file *filp, bool poweroff)
{
	struct hantro_dec_interface *iface = core->iface;
	bool flag = false;
	u32 status;

	scoped_guard(spinlock, &iface->lock) {
		scoped_guard(spinlock_irqsave, &core->lock) {
			if (core->is_reserved && core->dec_filp == filp) {
				core->is_reserved = 0;
				core->dec_filp = NULL;
				core->is_enabled = 0;
				core->irq_received = 0;
				core->irq_status = 0;

				status = hantro_dec_readl(core, HANTRODEC_IRQ_STAT_DEC_OFF);
				if (status & HANTRODEC_DEC_E) {
					dev_dbg(core->dev,
						"Dec[%d] is still enabled -> reset\n", core->id);

					status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
					hantro_dec_writel(core, status, HANTRODEC_IRQ_STAT_DEC_OFF);
				}
				iface->num_active_cores--;
				flag = true;
			}
		}
	}
	if (flag && poweroff) {
		pm_runtime_mark_last_busy(core->dev);
		pm_runtime_put_autosuspend(core->dev);
	}

	wake_up_interruptible_all(&iface->hw_queue);
}

static long hantro_dec_reserve_decoder(struct hantro_dec_interface *iface,
				       struct file *filp, unsigned long format)
{
	struct hantro_dec_core *core = NULL;
	int ret;

	if (hantro_wait_event_interruptible(iface->hw_queue,
					    hantro_dec_get_workable_core(iface,
									 filp,
									 format,
									 &core))) {
		dev_err(iface->dev, "Reserve decoder interrupted\n");
		return -ERESTARTSYS;
	}

	if (!core)
		return -EINVAL;

	ret = pm_runtime_resume_and_get(core->dev);
	if (ret) {
		dev_err(core->dev, "Failed to power on\n");
		hantro_dec_put_core(core, filp, false);
		return -EINVAL;
	}
	/* choose core in 8mq driver with blk-ctrl register */

	/* thermal check in 8mq */
	if (core->cooling_update)
		core->cooling_update(core);

	return core->id;
}

static long hantro_dec_release_decoder(struct hantro_dec_interface *iface,
				       struct file *filp, u32 id)
{
	struct hantro_dec_core *core = NULL;

	core = hantro_dec_get_core_by_id(iface, id);
	if (!core)
		return -EINVAL;

	hantro_dec_put_core(core, filp, true);
	return 0;
}

static long hantro_dec_push_regs(struct hantro_dec_interface *iface, struct core_desc *desc)
{
	struct hantro_dec_core *core;
	u32 *reg_buf;
	int count;
	int ret;

	core = hantro_dec_get_core_by_id(iface, desc->id);
	if (!core)
		return -EINVAL;
	if (!desc->size || (desc->size % 4) || desc->size > core->base_size) {
		dev_err(core->dev, "Invalid reg size %d, (%zd)\n", desc->size, core->base_size);
		return -EINVAL;
	}

	count = desc->size >> 2;
	count = min(count, core->num_org_regs);
	reg_buf = core->temp_regs;
	ret = copy_from_user(reg_buf, (void __user *)desc->regs, count << 2);
	if (ret) {
		dev_err(core->dev, "copy_from_user failed, %d\n", ret);
		return -EINVAL;
	}

	for (int i = 2; i < count; i++)
		hantro_dec_writel(core, reg_buf[i], i * 4);

	/*
	 * Write memory barrier to ensure all configuration register writes are completed
	 * before enabling the decoder.
	 */
	wmb();
	hantro_dec_writel(core, reg_buf[1], 4);
	scoped_guard(spinlock_irqsave, &core->lock) {
		core->is_enabled = 1;
		hantro_dec_update_mirror_regs(core);
	}

	if (core->format < DWL_CLIENT_TYPE_MAX)
		core->frame_num[core->format]++;

	return 0;
}

static long hantro_dec_pull_regs(struct hantro_dec_interface *iface, struct core_desc *desc)
{
	struct hantro_dec_core *core;
	u32 *reg_buf;
	int count;
	int ret;

	core = hantro_dec_get_core_by_id(iface, desc->id);
	if (!core)
		return -EINVAL;

	if (!desc->size || (desc->size % 4) || desc->size > core->base_size) {
		dev_err(core->dev, "Invalid reg size %d, (%zd)\n", desc->size, core->base_size);
		return -EINVAL;
	}

	count = desc->size >> 2;
	count = min(count, core->num_org_regs);
	reg_buf = core->mirror_regs;

	ret = copy_to_user((void __user *)desc->regs, reg_buf, count << 2);
	if (ret) {
		dev_err(core->dev, "copy_to_user failed, %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static bool hantro_dec_check_done(struct hantro_dec_core *core)
{
	bool done = false;

	scoped_guard(spinlock_irqsave, &core->lock) {
		if (core->irq_received)
			done = true;
	}

	return done;
}

static void hantro_dec_cancel(struct hantro_dec_core *core)
{
	core->mirror_regs[1] = 0x40100;
	hantro_dec_reset_core(core);
}

static long hantro_dec_wait_and_refresh_regs(struct hantro_dec_interface *iface,
					     struct core_desc *desc)
{
	struct hantro_dec_core *core;

	core = hantro_dec_get_core_by_id(iface, desc->id);
	if (!core)
		return -EINVAL;

	if (hantro_wait_event_timeout(core->dec_done,
				      hantro_dec_check_done(core),
				      msecs_to_jiffies(HANTRO_DEC_TIMEOUT_MS)) <= 0) {
		dev_err(core->dev, "wait irq timeout\n");
		hantro_dec_cancel(core);
	}

	return hantro_dec_pull_regs(iface, desc);
}

static int hantro_dec_open(struct inode *inode, struct file *filp)
{
	struct hantro_dec_interface *iface;

	iface = container_of(inode->i_cdev, struct hantro_dec_interface, cdev);
	filp->private_data = iface;

	return 0;
}

static int hantro_dec_release(struct inode *inode, struct file *filp)
{
	struct hantro_dec_interface *iface = filp->private_data;
	unsigned long cores_bitmap = HANTRO_DEC_CORE_FULL_IDS_BITMAP;
	struct hantro_dec_core *core;
	int id;

	while (cores_bitmap) {
		id = __ffs(cores_bitmap);
		clear_bit(id, &cores_bitmap);
		core = hantro_dec_get_core_by_id(iface, id);
		if (!core)
			continue;
		hantro_dec_put_core(core, filp, true);
	}

	return 0;
}

static long hantro_dec_ioctl_g_core_id(struct hantro_dec_interface *iface, unsigned long format)
{
	struct hantro_dec_core *core;

	core = hantro_dec_get_core_by_format(iface, format);
	if (!core)
		return -EINVAL;

	return core->id;
}

static long hantro_dec_ioctl_g_hw_offset(struct hantro_dec_interface *iface, unsigned long arg)
{
	struct hantro_dec_core *core;
	long err;
	u32 id;

	err = get_user(id, (u32 __user *)arg);
	if (err)
		return err;

	core = hantro_dec_get_core_by_id(iface, id);
	if (!core)
		return -EINVAL;

	return put_user(core->base_addr, (u32 __user *)arg);
}

static long hantro_dec_ioctl_g_hw_size(struct hantro_dec_interface *iface, unsigned long arg)
{
	struct hantro_dec_core *core;
	long err;
	u32 id;

	err = get_user(id, (u32 __user *)arg);
	if (err)
		return err;

	core = hantro_dec_get_core_by_id(iface, id);
	if (!core)
		return -EINVAL;

	return put_user(core->base_size, (u32 __user *)arg);
}

static long hantro_dec_ioctl_g_asic_id(struct hantro_dec_interface *iface, unsigned long arg)
{
	struct hantro_dec_core *core;
	long err;
	u32 id;

	err = get_user(id, (u32 __user *)arg);
	if (err)
		return err;

	core = hantro_dec_get_core_by_id(iface, id);
	if (!core)
		return -EINVAL;

	return put_user(core->mirror_regs[0], (u32 __user *)arg);
}

static long hantro_dec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct hantro_dec_interface *iface = filp->private_data;
	struct core_desc desc = { 0 };
	long err = 0;

	if (_IOC_TYPE(cmd) != HANTRODEC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HANTRODEC_IOC_MAXNR)
		return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
		err = put_user(iface->num_cores, (u32 __user *)arg);
		break;
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID):
		err = hantro_dec_ioctl_g_core_id(iface, arg);
		break;
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET):
		err = hantro_dec_ioctl_g_hw_offset(iface, arg);
		break;
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE):
		err = hantro_dec_ioctl_g_hw_size(iface, arg);
		break;
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID):
		err = hantro_dec_ioctl_g_asic_id(iface, arg);
		break;
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE):
		err = hantro_dec_reserve_decoder(iface, filp, arg);
		break;
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE):
		err = hantro_dec_release_decoder(iface, filp, arg);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG):
		err = copy_from_user(&desc, (void __user *)arg, sizeof(desc));
		if (err)
			break;
		err = hantro_dec_push_regs(iface, &desc);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG):
		err = copy_from_user(&desc, (void __user *)arg, sizeof(desc));
		if (err)
			break;
		err = hantro_dec_pull_regs(iface, &desc);
		break;
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT):
		err = copy_from_user(&desc, (void __user *)arg, sizeof(desc));
		if (err)
			break;
		err = hantro_dec_wait_and_refresh_regs(iface, &desc);
		break;
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG):
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG):
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE):
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT):
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT):
		dev_err(iface->dev, "unsupported ioctl 0x%x\n", _IOC_NR(cmd));
		err = -EINVAL;
		break;
	default:
		dev_err(iface->dev, "unknown ioctl 0x%x\n", _IOC_NR(cmd));
		err = -EINVAL;
		break;
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long hantro_dec_get_core_desc32(unsigned long arg, struct core_desc *desc)
{
	void __user *up = compat_ptr(arg);
	u32 tmp;
	int ret;

	ret = get_user(desc->id, (u32 __user *)up);
	if (ret)
		return ret;
	ret = get_user(tmp, (u32 __user *)up + 4);
	if (ret)
		return ret;
	ret = get_user(desc->size, (u32 __user *)up + 8);
	if (ret)
		return ret;

	desc->regs = (__force u32 *)compat_ptr(tmp);

	return 0;
}

static long hantro_dec_ioctl32(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct hantro_dec_interface *iface = filp->private_data;
	struct core_desc desc = { 0 };
	long err = 0;

	switch (cmd) {
	case HANTRODEC_IOCS_DEC_PUSH_REG:
		err = hantro_dec_get_core_desc32(arg, &desc);
		if (err)
			break;
		err = hantro_dec_push_regs(iface, &desc);
		break;
	case HANTRODEC_IOCS_DEC_PULL_REG:
		err = hantro_dec_get_core_desc32(arg, &desc);
		if (err)
			break;
		err = hantro_dec_pull_regs(iface, &desc);
		break;
	case HANTRODEC_IOCX_DEC_WAIT:
		err = hantro_dec_get_core_desc32(arg, &desc);
		if (err)
			break;
		err = hantro_dec_wait_and_refresh_regs(iface, &desc);
		break;
	default:
		err = compat_ptr_ioctl(filp, cmd, arg);
		break;
	}

	return err;
}
#endif

static int hantro_dec_mmap(struct file *filp, struct vm_area_struct *vm)
{
	struct hantro_dec_interface *iface = filp->private_data;
	struct hantro_dec_core *core = NULL;

	scoped_guard(spinlock, &iface->lock) {
		struct hantro_dec_core *tmp;

		list_for_each_entry(tmp, &iface->cores, list) {
			if (vm->vm_pgoff == (tmp->base_addr >> PAGE_SHIFT)) {
				core = tmp;
				break;
			}
		}
	}

	if (!core) {
		dev_err(iface->dev, "invalid map offset :0x%lX\n", vm->vm_pgoff);
		return -EINVAL;
	}

	return remap_vmalloc_range(vm, core->mirror_regs, 0);
}

static const struct file_operations hantro_dec_fops = {
	.owner = THIS_MODULE,
	.open = hantro_dec_open,
	.release = hantro_dec_release,
	.unlocked_ioctl = hantro_dec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hantro_dec_ioctl32,
#endif
	.fasync = NULL,
	.mmap = hantro_dec_mmap,
};

static irqreturn_t hantro_dec_isr(int irq, void *dev_id)
{
	struct hantro_dec_core *core = dev_id;
	unsigned int handled = 0;
	u32 irq_status;

	scoped_guard(spinlock_irqsave, &core->lock) {
		if (!core->is_reserved) {
			dev_dbg(core->dev, "irq received but core is not reserved\n");
			return IRQ_HANDLED;
		}
	}

	irq_status = hantro_dec_readl(core, HANTRODEC_IRQ_STAT_DEC_OFF);
	if (irq_status & HANTRODEC_DEC_IRQ) {
		irq_status &= (~HANTRODEC_DEC_IRQ);
		hantro_dec_writel(core, irq_status, HANTRODEC_IRQ_STAT_DEC_OFF);

		if (irq_status & HANTRODEC_DEC_ERROR_MASK) {
			if (irq_status & HANTRODEC_DEC_BUS_ERROR)
				dev_dbg(core->dev, "bus error\n");
			if (irq_status & HANTRODEC_DEC_STRM_BUF_EMPTY)
				dev_dbg(core->dev, "stream buffer empty\n");
			if (irq_status & HANTRODEC_DEC_ASO_DETECTED)
				dev_dbg(core->dev, "detect ASO\n");
			if (irq_status & HANTRODEC_DEC_STRM_INPUT_ERR)
				dev_dbg(core->dev, "stream input error\n");
		}
		scoped_guard(spinlock_irqsave, &core->lock) {
			core->irq_received = 1;
			core->irq_status = irq_status;

			hantro_dec_update_mirror_regs(core);
		}
		wake_up_all(&core->dec_done);
		handled++;
	}

	return IRQ_RETVAL(handled);
}

static inline int hantro_dec_read_irq_received(struct hantro_dec_core *core)
{
	guard(spinlock_irqsave)(&core->lock);

	return core->irq_received;
}

static int hantro_dec_pause(struct hantro_dec_core *core)
{
	int ret;

	if (!core || !core->is_valid)
		return 0;
	if (!core->is_reserved)
		return 0;

	if (core->is_enabled) {
		u32 data;

		ret = read_poll_timeout(hantro_dec_read_irq_received, data, data, 10,
					HANTRO_DEC_TIMEOUT_MS * USEC_PER_MSEC, false, core);
		if (ret) {
			dev_err(core->dev, "wait core[%d] done timeout\n", core->id);
			return -EINVAL;
		}
	}

	dev_dbg(core->dev, "suspend, irq_status = 0x%x\n", core->irq_status);

	return 0;
}

static int hantro_dec_pause_cores(struct hantro_dec_interface *iface)
{
	unsigned long cores_bitmap = HANTRO_DEC_CORE_FULL_IDS_BITMAP;
	struct hantro_dec_core *core;
	int ret, id;

	while (cores_bitmap) {
		id = __ffs(cores_bitmap);
		clear_bit(id, &cores_bitmap);
		core = hantro_dec_get_core_by_id(iface, id);
		if (!core)
			continue;
		ret = hantro_dec_pause(core);
		if (ret)
			return ret;
	}

	return 0;
}

static unsigned long hantro_dec_get_max_clk_rate(struct hantro_dec_interface *iface)
{
	unsigned long cores_bitmap = HANTRO_DEC_CORE_FULL_IDS_BITMAP;
	struct hantro_dec_core *core;
	unsigned long clk, clk_max = 0;
	int id;

	while (cores_bitmap) {
		id = __ffs(cores_bitmap);
		clear_bit(id, &cores_bitmap);
		core = hantro_dec_get_core_by_id(iface, id);
		if (!core)
			continue;

		clk = clk_get_rate(core->clks[0].clk);
		if (clk_max < clk)
			clk_max = clk;
	}

	return clk_max;
}

static int hantro_dec_update_voltage(struct hantro_dec_core *core)
{
	unsigned long new_vol, old_vol;
	int ret;
	unsigned long clk;

	if (!core->regulator || !core->num_clks)
		return -EINVAL;

	clk = hantro_dec_get_max_clk_rate(core->iface);
	if (!clk)
		return -EINVAL;

	old_vol = regulator_get_voltage(core->regulator);
	if (clk >= HANTRO_DEC_CLK_VOL_THR)
		new_vol = 1000000; // 1.0v
	else
		new_vol = 900000; // 0.9v

	if (old_vol != new_vol)	{
		ret = regulator_set_voltage_tol(core->regulator, new_vol, 0);
		if (ret)
			dev_info(core->dev, "failed to set hantro voltage: %ld mV\n",
				 new_vol/1000);
		else
			dev_info(core->dev, "update hantro voltage from %ld mV to %ld mV\n",
				 old_vol/1000, new_vol/1000);
	}

	return 0;
}

static void hantro_dec_destroy_interface(struct hantro_dec_interface *iface)
{
	dev_dbg(iface->dev, "destroy decoder interface\n");

	debugfs_remove(iface->debugfs);
	device_destroy(iface->dec_class, iface->devt);
	cdev_del(&iface->cdev);
	class_destroy(iface->dec_class);
	unregister_chrdev_region(iface->devt, 1);
	kfree(iface);
}

static struct hantro_dec_interface *hantro_dec_create_interface(void)
{
	struct hantro_dec_interface *iface;
	int ret;

	iface = kzalloc(sizeof(*iface), GFP_KERNEL);
	if (!iface)
		return NULL;

	atomic_set(&iface->ref_cnt, 0);
	ret = alloc_chrdev_region(&iface->devt, 0, 1, HANTRO_DEC_NAME);
	if (ret < 0) {
		pr_err("Unable to alloc hantro decoder chrdec region\n");
		goto error;
	}

	iface->dec_class = class_create(HANTRO_DEC_NAME);
	if (IS_ERR(iface->dec_class)) {
		ret = PTR_ERR(iface->dec_class);
		iface->dec_class = NULL;
		goto error_clean_chrdev_region;
	}

	cdev_init(&iface->cdev, &hantro_dec_fops);
	iface->cdev.owner = THIS_MODULE;

	ret = cdev_add(&iface->cdev, iface->devt, 1);
	if (ret)
		goto error_clean_class;

	iface->dev = device_create(iface->dec_class, NULL, iface->devt,
				   NULL, HANTRO_DEC_DEVICE_NAME);
	if (IS_ERR(iface->dev)) {
		ret = PTR_ERR(iface->dev);
		goto error_del_cdev;
	}

	iface->debugfs = debugfs_create_dir("hantro_dec", NULL);
	spin_lock_init(&iface->lock);
	INIT_LIST_HEAD(&iface->cores);
	init_waitqueue_head(&iface->hw_queue);
	iface->idle_core_ids_bitmap = HANTRO_DEC_CORE_FULL_IDS_BITMAP;

	return iface;

error_del_cdev:
	cdev_del(&iface->cdev);
error_clean_class:
	class_destroy(iface->dec_class);
error_clean_chrdev_region:
	unregister_chrdev_region(iface->devt, 1);
error:
	kfree(iface);
	return NULL;
}

static struct hantro_dec_interface *hantro_dec_get_interface(void)
{
	static struct hantro_dec_interface *iface;

	if (!iface) {
		iface = hantro_dec_create_interface();
		if (!iface)
			return NULL;
	}

	atomic_inc(&iface->ref_cnt);
	return iface;
}

static void hantro_dec_put_interface(struct hantro_dec_interface *iface)
{
	if (!iface)
		return;
	if (atomic_dec_and_test(&iface->ref_cnt))
		hantro_dec_destroy_interface(iface);
}

static int hantro_dec_add_core(struct hantro_dec_interface *iface, struct hantro_dec_core *core)
{
	scoped_guard(spinlock, &iface->lock) {
		if (!iface->idle_core_ids_bitmap)
			return -EINVAL;
		core->id = __ffs(iface->idle_core_ids_bitmap);
		clear_bit(core->id, &iface->idle_core_ids_bitmap);
		list_add_tail(&core->list, &iface->cores);
		iface->num_cores++;
	}

	return 0;
}

static void hantro_dec_del_core(struct hantro_dec_interface *iface, struct hantro_dec_core *core)
{
	scoped_guard(spinlock, &iface->lock) {
		list_del_init(&core->list);
		set_bit(core->id, &iface->idle_core_ids_bitmap);
		iface->num_cores--;
	}
}

static bool hantro_dec_check_hw_id(struct hantro_dec_core *core, u32 hw_id)
{
	bool valid = false;
	int i = 0;

	while (core->resource->hw_ids[i]) {
		if (hw_id == core->resource->hw_ids[i]) {
			valid = true;
			break;
		}
		i++;
	}

	return valid;
}

static u32 hantro_dec_format_mask[] = {
	[DWL_CLIENT_TYPE_H264_DEC]	= 3 << DWL_H264_E,
	[DWL_CLIENT_TYPE_MPEG4_DEC]	= 3 << DWL_MPEG4_E,
	[DWL_CLIENT_TYPE_JPEG_DEC]	= 1 << DWL_JPEG_E,
	[DWL_CLIENT_TYPE_PP]		= 1 << DWL_PP_E,
	[DWL_CLIENT_TYPE_VC1_DEC]	= 3 << DWL_VC1_E,
	[DWL_CLIENT_TYPE_MPEG2_DEC]	= 1 << DWL_MPEG2_E,
	[DWL_CLIENT_TYPE_VP6_DEC]	= 1 << DWL_VP6_E,
	[DWL_CLIENT_TYPE_AVS_DEC]	= 1 << DWL_AVS_E,
	[DWL_CLIENT_TYPE_RV_DEC]	= 3 << DWL_RV_E,
	/* VP7 and WEBP is part of VP8 */
	[DWL_CLIENT_TYPE_VP8_DEC]	= ((1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E)),
	[DWL_CLIENT_TYPE_VP9_DEC]	= 3 << DWL_VP9_E,
	[DWL_CLIENT_TYPE_HEVC_DEC]	= 3 << DWL_HEVC_E
};

#define HANTRO_DEC_SET_FORMAT(core, reg, type, name)			\
	do {								\
		u32 __mask = hantro_dec_format_mask[type];		\
		if (__mask && (__mask & (reg))) {			\
			dev_dbg(core->dev, "has %s\n", name);		\
			set_bit(type, &core->config);			\
		}							\
	} while (0)

static void hantro_dec_read_core_config(struct hantro_dec_core *core)
{
	u32 reg;

	if (hantro_dec_is_g1(core)) {
		reg = hantro_dec_readl(core, HANTRODEC_SYNTH_CFG * 4);

		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_H264_DEC, "H264");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_JPEG_DEC, "JPEG");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_MPEG4_DEC, "MPEG4");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_MPEG2_DEC, "MPEG2");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_VC1_DEC, "VC1");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_VP6_DEC, "VP6");

		reg = hantro_dec_readl(core, HANTRODEC_SYNTH_CFG_2 * 4);

		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_VP8_DEC, "VP8");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_AVS_DEC, "AVS");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_RV_DEC, "RV");
	} else {
		reg = hantro_dec_readl(core, HANTRODEC_SYNTH_CFG_2 * 4);

		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_HEVC_DEC, "HEVC");
		HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_VP9_DEC, "VP9");
	}
	reg = hantro_dec_readl(core, HANTRODECPP_SYNTH_CFG * 4);
	HANTRO_DEC_SET_FORMAT(core, reg, DWL_CLIENT_TYPE_PP, "PP");
}

static int hantro_dec_dbg_core(struct seq_file *s, void *data)
{
	struct hantro_dec_core *core = s->private;
	char str[128];
	int num;

	num = scnprintf(str, sizeof(str), "hw id: 0x%x\n", core->hw_id);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"reserved: %d, enabled: %d, irq received: %d, status 0x%x\n",
			core->is_reserved, core->is_enabled, core->irq_received, core->irq_status);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "last format: %s\n",
			hantro_dec_get_fmt_name(core->format));
	if (seq_write(s, str, num))
		return 0;

	for (int i = DWL_CLIENT_TYPE_H264_DEC; i < DWL_CLIENT_TYPE_MAX; i++) {
		if (!hantro_dec_core_has_format(core, i) || i == DWL_CLIENT_TYPE_PP)
			continue;
		num = scnprintf(str, sizeof(str), "[%.4s] = %ld\n",
				hantro_dec_get_fmt_name(i), core->frame_num[i]);
		if (seq_write(s, str, num))
			return 0;
	}

	return 0;
}

static int hantro_dec_dbg_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hantro_dec_dbg_core, inode->i_private);
}

static const struct file_operations hantro_dec_dbg_fops = {
	.owner = THIS_MODULE,
	.open = hantro_dec_dbg_open,
	.release = single_release,
	.read = seq_read,
};

static void hantro_dec_create_debugfs(struct hantro_dec_core *core)
{
	char name[64];

	if (!core || !core->iface || !core->iface->debugfs)
		return;
	if (hantro_dec_is_g1(core))
		scnprintf(name, sizeof(name), "g1");
	else
		scnprintf(name, sizeof(name), "g2");

	core->debugfs = debugfs_create_file((const char *)name,
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    core->iface->debugfs,
					    core,
					    &hantro_dec_dbg_fops);
}

static int hantro_dec_init_core(struct hantro_dec_core *core)
{
	int ret;
	u32 hw_id;
	u32 size;

	ret = pm_runtime_resume_and_get(core->dev);
	if (ret)
		return ret;

	hw_id = hantro_dec_readl(core, 0);
	dev_info(core->dev, "hw id 0x%x\n", hw_id);
	if (!hantro_dec_check_hw_id(core, FIELD_GET(HANTRO_DEC_HW_ID_MASK, hw_id))) {
		dev_err(core->dev, "HW is not found at 0x%lx\n", core->base_addr);
		ret = -EINVAL;
		goto exit;
	}

	core->hw_id = hw_id;
	core->num_regs = core->resource->num_regs;
	core->num_org_regs = core->resource->num_org_regs;
	size = PAGE_ALIGN(sizeof(u32) * core->num_regs);
	core->mirror_regs = (u32 *)vmalloc_user(size);
	if (!core->mirror_regs) {
		dev_err(core->dev, "Failed to allocate mirror regs\n");
		ret = -ENOMEM;
		goto exit;
	}
	core->temp_regs = (u32 *)vmalloc_user(PAGE_ALIGN(sizeof(u32) * core->num_org_regs));
	if (!core->temp_regs) {
		dev_err(core->dev, "Failed to allocate temp buffer for regs\n");
		ret = -ENOMEM;
		goto exit;
	}
	core->is_valid = 1;
	spin_lock_init(&core->lock);

	hantro_dec_read_core_config(core);
	hantro_dec_reset_core(core);
	ret = hantro_dec_add_core(core->iface, core);
	if (ret)
		goto exit;
	hantro_dec_update_voltage(core);
	if (core->resource->resource_shared_inter_cores)
		core->iface->resource_shared_inter_cores = true;
	dev_dbg(core->dev, "core[%d] format config 0x%lx\n", core->id, core->config);

	hantro_dec_create_debugfs(core);
exit:
	pm_runtime_mark_last_busy(core->dev);
	pm_runtime_put_autosuspend(core->dev);
	if (ret) {
		if (core->mirror_regs) {
			vfree(core->mirror_regs);
			core->mirror_regs = NULL;
		}
		if (core->temp_regs) {
			vfree(core->temp_regs);
			core->temp_regs = NULL;
		}
	}
	return ret;
}

#ifdef CONFIG_DEVFREQ_THERMAL
#define HANTRO_DEC_COOLING_MAX_STATE	1

static int hantro_dec_cooling_get_max_state(struct thermal_cooling_device *cdev,
					    unsigned long *state)
{
	*state = HANTRO_DEC_COOLING_MAX_STATE;
	return 0;
}

static int hantro_dec_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long *state)
{
	struct hantro_dec_core *core = cdev->devdata;

	*state = core->thermal_event;

	return 0;
}

static int hantro_dec_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long state)
{
	struct hantro_dec_core *core = cdev->devdata;

	guard(spinlock)(&core->thermal_lock);

	core->thermal_event = state;	/*event: 1: hot, 0: cool*/
	dev_dbg(core->dev, "hantro receive cooling set state: %ld\n", state);
	return 0;
}

static struct thermal_cooling_device_ops hantro_dec_cooling_ops = {
	.get_max_state = hantro_dec_cooling_get_max_state,
	.get_cur_state = hantro_dec_cooling_get_cur_state,
	.set_cur_state = hantro_dec_cooling_set_cur_state,
};

static void hantro_dec_thermal_update(struct hantro_dec_core *core)
{
	bool update_flag = false;
	unsigned long clk_rate;
	int ratio;

	if (!core->cooling)
		return;

	scoped_guard(spinlock, &core->thermal_lock) {
		if (core->thermal_cur != core->thermal_event) {
			core->thermal_cur = core->thermal_event;
			update_flag = true;
		}
	}

	if (!update_flag)
		return;

	if (core->thermal_cur) {
		ratio = hantro_clock_ratio ? hantro_clock_ratio : 2;
		clk_rate = core->clk_rate_def / ratio;
	} else {
		clk_rate = core->clk_rate_def;
	}

	clk_set_rate(core->clks[0].clk, clk_rate);
	hantro_dec_update_voltage(core);
}

static int hantro_dec_cooling_init(struct hantro_dec_core *core)
{
	spin_lock_init(&core->thermal_lock);
	core->cooling = thermal_of_cooling_device_register(core->dev->of_node,
							   dev_name(core->dev),
							   core,
							   &hantro_dec_cooling_ops);
	if (core->cooling) {
		core->clk_rate_def = clk_get_rate(core->clks[0].clk);
		core->cooling_update = hantro_dec_thermal_update;
	}

	return 0;
}

static void hantro_dec_cooling_remove(struct hantro_dec_core *core)
{
	if (core->cooling)
		thermal_cooling_device_unregister(core->cooling);
	core->cooling = NULL;
}
#endif

static int hantro_dec_probe(struct platform_device *pdev)
{
	const struct hantro_dec_resource *resource;
	struct hantro_dec_core *core;
	struct resource *res;
	int ret;

	resource = device_get_match_data(&pdev->dev);
	if (!resource) {
		dev_err(&pdev->dev, "missing match data\n");
		return -EINVAL;
	}

	core = devm_kzalloc(&pdev->dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = &pdev->dev;
	core->resource = resource;
	INIT_LIST_HEAD(&core->list);
	init_waitqueue_head(&core->dec_done);
	dev_set_drvdata(&pdev->dev, core);

	ret = devm_clk_bulk_get_all(&pdev->dev, &core->clks);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to get clocks: %d\n", ret);
		ret = 0;
	}
	core->num_clks = ret;

	core->reg_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(core->reg_base))
		return PTR_ERR(core->reg_base);

	if (resource_size(res) < core->resource->num_regs * 4)
		return -ENXIO;

	core->base_addr = res->start;
	core->base_size = core->resource->num_regs * 4;

	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0)
		return -ENXIO;

	ret = devm_request_threaded_irq(&pdev->dev, core->irq, hantro_dec_isr, NULL,
					0, HANTRO_DEC_NAME, core);
	if (ret) {
		dev_err(&pdev->dev, "fail to register interrupt handler: %d\n", ret);
		return ret;
	}

	core->regulator = devm_regulator_get_optional(&pdev->dev, "vpu");
	if (IS_ERR(core->regulator))
		core->regulator = NULL;

	core->iface = hantro_dec_get_interface();
	if (!core->iface) {
		dev_err(&pdev->dev, "failed to get decoder interface\n");
		return -EINVAL;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = hantro_dec_init_core(core);
	if (ret) {
		hantro_dec_put_interface(core->iface);
		pm_runtime_dont_use_autosuspend(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

#ifdef CONFIG_DEVFREQ_THERMAL
	hantro_dec_cooling_init(core);
#endif

	return 0;
}

static void hantro_dec_remove(struct platform_device *pdev)
{
	struct hantro_dec_core *core = dev_get_drvdata(&pdev->dev);

	debugfs_remove(core->debugfs);
#ifdef CONFIG_DEVFREQ_THERMAL
	hantro_dec_cooling_remove(core);
#endif
	hantro_dec_del_core(core->iface, core);
	hantro_dec_put_interface(core->iface);

	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	vfree(core->mirror_regs);
	vfree(core->temp_regs);
}

static int hantro_dec_runtime_suspend(struct device *dev)
{
	struct hantro_dec_core *core = dev_get_drvdata(dev);

	if (core->num_clks)
		clk_bulk_disable_unprepare(core->num_clks, core->clks);

	return 0;
}

static int hantro_dec_runtime_resume(struct device *dev)
{
	struct hantro_dec_core *core = dev_get_drvdata(dev);
	int ret = 0;

	if (core->num_clks)
		ret = clk_bulk_prepare_enable(core->num_clks, core->clks);

	return ret;
}

static int __maybe_unused hantro_dec_suspend(struct device *dev)
{
	struct hantro_dec_core *core = dev_get_drvdata(dev);
	int ret;

	ret = hantro_dec_pause_cores(core->iface);
	if (!ret)
		pm_runtime_force_suspend(dev);
	return ret;
}

static const struct dev_pm_ops hantro_dec_pm_ops = {
	SET_RUNTIME_PM_OPS(hantro_dec_runtime_suspend, hantro_dec_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hantro_dec_suspend, pm_runtime_force_resume)
};

static u32 hantro_dec_g1_supported_hw_ids[] = {
	0x6731,
	0 /* sentinel */
};

static struct hantro_dec_resource imx8mm_dec_g1_id = {
	.hw_ids = hantro_dec_g1_supported_hw_ids,
	.type = HANTRO_DEC_TYPE_G1,
	.num_regs = 155,
	.num_org_regs = 60,
};

static struct hantro_dec_resource imx8mq_dec_g1_id = {
	.hw_ids = hantro_dec_g1_supported_hw_ids,
	.type = HANTRO_DEC_TYPE_G1,
	.num_regs = 155,
	.num_org_regs = 60,
	.resource_shared_inter_cores = true,
};

static u32 hantro_dec_g2_supported_hw_ids[] = {
	0x6732,
	0 /* sentinel */
};

static struct hantro_dec_resource imx8mq_dec_g2_id = {
	.hw_ids = hantro_dec_g2_supported_hw_ids,
	.type = HANTRO_DEC_TYPE_G2,
	.num_regs = 265,
	.num_org_regs = 265,
};

static const struct of_device_id hantro_dec_of_match[] = {
	{ .compatible = "nxp,imx8mm-vpu-g1", .data = &imx8mm_dec_g1_id },
	{ .compatible = "nxp,imx8mq-vpu-g1", .data = &imx8mq_dec_g1_id },
	{ .compatible = "nxp,imx8mq-vpu-g2", .data = &imx8mq_dec_g2_id },
	{/* sentinel */}
};

static struct platform_driver hantro_dec_driver = {
	.driver = {
		.name = "mxc_hantro_decoder",
		.of_match_table = hantro_dec_of_match,
		.pm = &hantro_dec_pm_ops,
	},
	.probe = hantro_dec_probe,
	.remove = hantro_dec_remove,
};

module_platform_driver(hantro_dec_driver);

MODULE_DESCRIPTION("Hantro G1 and G2 decoder driver for i.MX8MP, i.MX8MM and i.MP8MQ");
MODULE_LICENSE("GPL");
