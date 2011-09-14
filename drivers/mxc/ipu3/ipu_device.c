/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_device.c
 *
 * @brief This file contains the IPUv3 driver device interface and fops functions.
 *
 * @ingroup IPU
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/kthread.h>
#include <asm/cacheflush.h>

#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

/* Strucutures and variables for exporting MXC IPU as device*/
typedef enum {
	RGB_CS,
	YUV_CS,
	NULL_CS
} cs_t;

typedef enum {
	STATE_OK = 0,
	STATE_NO_IPU,
	STATE_NO_IRQ,
	STATE_IRQ_FAIL,
	STATE_IRQ_TIMEOUT,
	STATE_INIT_CHAN_FAIL,
	STATE_LINK_CHAN_FAIL,
	STATE_INIT_CHAN_BUF_FAIL,
} ipu_state_t;

struct ipu_state_msg {
	int state;
	char *msg;
} state_msg[] = {
	{STATE_OK, "ok"},
	{STATE_NO_IPU, "no ipu found"},
	{STATE_NO_IRQ, "no irq found for task"},
	{STATE_IRQ_FAIL, "request irq failed"},
	{STATE_IRQ_TIMEOUT, "wait for irq timeout"},
	{STATE_INIT_CHAN_FAIL, "ipu init channel fail"},
	{STATE_LINK_CHAN_FAIL, "ipu link channel fail"},
	{STATE_INIT_CHAN_BUF_FAIL, "ipu init channel buffer fail"},
};

struct stripe_setting {
	u32 iw;
	u32 ih;
	u32 ow;
	u32 oh;
	u32 outh_resize_ratio;
	u32 outv_resize_ratio;
	u32 i_left_pos;
	u32 i_right_pos;
	u32 i_top_pos;
	u32 i_bottom_pos;
	u32 o_left_pos;
	u32 o_right_pos;
	u32 o_top_pos;
	u32 o_bottom_pos;
};

struct task_set {
#define	NULL_MODE	0x0
#define	IC_MODE		0x1
#define	ROT_MODE	0x2
#define	VDI_MODE	0x4
	u8	mode;
#define IC_VF	0x1
#define IC_PP	0x2
#define ROT_VF	0x4
#define ROT_PP	0x8
#define VDI_VF	0x10
	u8	task;

	ipu_channel_t ic_chan;
	ipu_channel_t rot_chan;
	ipu_channel_t vdi_ic_p_chan;
	ipu_channel_t vdi_ic_n_chan;

	u32 i_off;
	u32 i_uoff;
	u32 i_voff;
	u32 istride;

	u32 ov_off;
	u32 ov_uoff;
	u32 ov_voff;
	u32 ovstride;

	u32 ov_alpha_off;
	u32 ov_alpha_stride;

	u32 o_off;
	u32 o_uoff;
	u32 o_voff;
	u32 ostride;

	u32 r_fmt;
	u32 r_width;
	u32 r_height;
	u32 r_stride;
	dma_addr_t r_paddr;

#define NO_SPLIT	0x0
#define RL_SPLIT	0x1
#define UD_SPLIT	0x2
#define LEFT_STRIPE	0x1
#define RIGHT_STRIPE	0x2
#define UP_STRIPE	0x4
#define DOWN_STRIPE	0x8
	u8 split_mode;
	struct stripe_setting sp_setting;
};

struct ipu_split_task {
	struct ipu_task task;
	struct ipu_task_entry *parent_task;
	struct task_struct *thread;
	bool could_finish;
	wait_queue_head_t waitq;
	int ret;
};

struct ipu_task_entry {
	struct ipu_input input;
	struct ipu_output output;

	bool overlay_en;
	struct ipu_overlay overlay;

	u8	priority;
	u8	task_id;
#define DEF_TIMEOUT_MS	1000
	int	timeout;

	struct list_head node;
	struct device *dev;
	struct task_set set;
	struct completion comp;
	ipu_state_t state;
};

struct ipu_alloc_list {
	struct list_head list;
	dma_addr_t phy_addr;
	void *cpu_addr;
	u32 size;
};
LIST_HEAD(ipu_alloc_list);

static int major;
static struct class *ipu_class;
static struct device *ipu_dev;

int ipu_queue_sp_task(struct ipu_split_task *sp_task);

static bool deinterlace_3_field(struct ipu_task_entry *t)
{
	return ((t->set.mode & VDI_MODE) &&
		(t->input.deinterlace.motion != HIGH_MOTION));
}

static u32 fmt_to_bpp(u32 pixelformat)
{
	u32 bpp;

	switch (pixelformat) {
	case IPU_PIX_FMT_RGB565:
	/*interleaved 422*/
	case IPU_PIX_FMT_YUYV:
	case IPU_PIX_FMT_UYVY:
	/*non-interleaved 422*/
	case IPU_PIX_FMT_YUV422P:
	case IPU_PIX_FMT_YVU422P:
		bpp = 16;
		break;
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_YUV444:
		bpp = 24;
		break;
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_ABGR32:
		bpp = 32;
		break;
	/*non-interleaved 420*/
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_NV12:
		bpp = 12;
		break;
	default:
		bpp = 8;
		break;
	}
	return bpp;
}

static cs_t colorspaceofpixel(int fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_RGB565:
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_BGRA32:
	case IPU_PIX_FMT_BGR32:
	case IPU_PIX_FMT_RGBA32:
	case IPU_PIX_FMT_RGB32:
	case IPU_PIX_FMT_ABGR32:
		return RGB_CS;
		break;
	case IPU_PIX_FMT_UYVY:
	case IPU_PIX_FMT_YUYV:
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YUV420P:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_YVU422P:
	case IPU_PIX_FMT_YUV422P:
	case IPU_PIX_FMT_YUV444:
	case IPU_PIX_FMT_NV12:
		return YUV_CS;
		break;
	default:
		return NULL_CS;
	}
}

static int need_csc(int ifmt, int ofmt)
{
	cs_t ics, ocs;

	ics = colorspaceofpixel(ifmt);
	ocs = colorspaceofpixel(ofmt);

	if ((ics == NULL_CS) || (ocs == NULL_CS))
		return -1;
	else if (ics != ocs)
		return 1;

	return 0;
}

static int soc_max_in_width(void)
{
	return 4096;
}

static int soc_max_in_height(void)
{
	return 4096;
}

static int soc_max_out_width(void)
{
	/* mx51/mx53/mx6q is 1024*/
	return 1024;
}

static int soc_max_out_height(void)
{
	/* mx51/mx53/mx6q is 1024*/
	return 1024;
}

static int list_size(struct list_head *head)
{
	struct list_head *p, *n;
	int size = 0;

	list_for_each_safe(p, n, head)
		size++;

	return size;
}

static int get_task_size(struct ipu_soc *ipu, int id)
{
	struct list_head *task_list;

	if (id == IPU_TASK_ID_VF)
		task_list = &ipu->task_list[0];
	else if (id == IPU_TASK_ID_PP)
		task_list = &ipu->task_list[1];
	else {
		printk(KERN_ERR "query error task id\n");
		return -EINVAL;
	}

	return list_size(task_list);
}

static struct ipu_soc *most_free_ipu_task(struct ipu_task_entry *t)
{
	unsigned int task_num[2][2] = {
		{0xffffffff, 0xffffffff},
		{0xffffffff, 0xffffffff} };
	struct ipu_soc *ipu;
	int ipu_idx, task_id;
	int i;

	/* decide task_id */
	if (t->task_id >= IPU_TASK_ID_MAX)
		t->task_id %= IPU_TASK_ID_MAX;
	/* must use task_id VF for VDI task*/
	if ((t->set.mode & VDI_MODE) &&
		(t->task_id != IPU_TASK_ID_VF))
		t->task_id = IPU_TASK_ID_VF;

	for (i = 0; i < MXC_IPU_MAX_NUM; i++) {
		ipu = ipu_get_soc(i);
		if (!IS_ERR(ipu)) {
			task_num[i][0] = get_task_size(ipu, IPU_TASK_ID_VF);
			task_num[i][1] = get_task_size(ipu, IPU_TASK_ID_PP);
		}
	}

	task_id = t->task_id;
	if (t->task_id == IPU_TASK_ID_VF) {
		if (task_num[0][0] < task_num[1][0])
			ipu_idx = 0;
		else
			ipu_idx = 1;
	} else if (t->task_id == IPU_TASK_ID_PP) {
		if (task_num[0][1] < task_num[1][1])
			ipu_idx = 0;
		else
			ipu_idx = 1;
	} else {
		unsigned int min;
		ipu_idx = 0;
		task_id = IPU_TASK_ID_VF;
		min = task_num[0][0];
		if (task_num[0][1] < min) {
			min = task_num[0][1];
			task_id = IPU_TASK_ID_PP;
		}
		if (task_num[1][0] < min) {
			min = task_num[1][0];
			ipu_idx = 1;
			task_id = IPU_TASK_ID_VF;
		}
		if (task_num[1][1] < min) {
			ipu_idx = 1;
			task_id = IPU_TASK_ID_PP;
		}
	}

	t->task_id = task_id;
	ipu = ipu_get_soc(ipu_idx);

	return ipu;
}

static void dump_task_info(struct ipu_task_entry *t)
{
	dev_dbg(t->dev, "[0x%p]input:\n", (void *)t);
	dev_dbg(t->dev, "[0x%p]\tformat = 0x%x\n", (void *)t, t->input.format);
	dev_dbg(t->dev, "[0x%p]\twidth = %d\n", (void *)t, t->input.width);
	dev_dbg(t->dev, "[0x%p]\theight = %d\n", (void *)t, t->input.height);
	dev_dbg(t->dev, "[0x%p]\tcrop.w = %d\n", (void *)t, t->input.crop.w);
	dev_dbg(t->dev, "[0x%p]\tcrop.h = %d\n", (void *)t, t->input.crop.h);
	dev_dbg(t->dev, "[0x%p]\tcrop.pos.x = %d\n",
			(void *)t, t->input.crop.pos.x);
	dev_dbg(t->dev, "[0x%p]\tcrop.pos.y = %d\n",
			(void *)t, t->input.crop.pos.y);
	dev_dbg(t->dev, "[0x%p]input buffer:\n", (void *)t);
	dev_dbg(t->dev, "[0x%p]\tpaddr = 0x%x\n", (void *)t, t->input.paddr);
	dev_dbg(t->dev, "[0x%p]\ti_off = 0x%x\n", (void *)t, t->set.i_off);
	dev_dbg(t->dev, "[0x%p]\ti_uoff = 0x%x\n", (void *)t, t->set.i_uoff);
	dev_dbg(t->dev, "[0x%p]\ti_voff = 0x%x\n", (void *)t, t->set.i_voff);
	dev_dbg(t->dev, "[0x%p]\tistride = %d\n", (void *)t, t->set.istride);
	if (t->input.deinterlace.enable) {
		dev_dbg(t->dev, "[0x%p]deinterlace enabled with:\n", (void *)t);
		if (t->input.deinterlace.motion != HIGH_MOTION) {
			dev_dbg(t->dev, "[0x%p]\tlow/medium motion\n", (void *)t);
			dev_dbg(t->dev, "[0x%p]\tpaddr_n = 0x%x\n",
				(void *)t, t->input.paddr_n);
		} else
			dev_dbg(t->dev, "[0x%p]\thigh motion\n", (void *)t);
	}

	dev_dbg(t->dev, "[0x%p]output:\n", (void *)t);
	dev_dbg(t->dev, "[0x%p]\tformat = 0x%x\n", (void *)t, t->output.format);
	dev_dbg(t->dev, "[0x%p]\twidth = %d\n", (void *)t, t->output.width);
	dev_dbg(t->dev, "[0x%p]\theight = %d\n", (void *)t, t->output.height);
	dev_dbg(t->dev, "[0x%p]\tcrop.w = %d\n", (void *)t, t->output.crop.w);
	dev_dbg(t->dev, "[0x%p]\tcrop.h = %d\n", (void *)t, t->output.crop.h);
	dev_dbg(t->dev, "[0x%p]\tcrop.pos.x = %d\n",
			(void *)t, t->output.crop.pos.x);
	dev_dbg(t->dev, "[0x%p]\tcrop.pos.y = %d\n",
			(void *)t, t->output.crop.pos.y);
	dev_dbg(t->dev, "[0x%p]\trotate = %d\n", (void *)t, t->output.rotate);
	dev_dbg(t->dev, "[0x%p]output buffer:\n", (void *)t);
	dev_dbg(t->dev, "[0x%p]\tpaddr = 0x%x\n", (void *)t, t->output.paddr);
	dev_dbg(t->dev, "[0x%p]\to_off = 0x%x\n", (void *)t, t->set.o_off);
	dev_dbg(t->dev, "[0x%p]\to_uoff = 0x%x\n", (void *)t, t->set.o_uoff);
	dev_dbg(t->dev, "[0x%p]\to_voff = 0x%x\n", (void *)t, t->set.o_voff);
	dev_dbg(t->dev, "[0x%p]\tostride = %d\n", (void *)t, t->set.ostride);

	if (t->overlay_en) {
		dev_dbg(t->dev, "[0x%p]overlay:\n", (void *)t);
		dev_dbg(t->dev, "[0x%p]\tformat = 0x%x\n",
				(void *)t, t->overlay.format);
		dev_dbg(t->dev, "[0x%p]\twidth = %d\n",
				(void *)t, t->overlay.width);
		dev_dbg(t->dev, "[0x%p]\theight = %d\n",
				(void *)t, t->overlay.height);
		dev_dbg(t->dev, "[0x%p]\tcrop.w = %d\n",
				(void *)t, t->overlay.crop.w);
		dev_dbg(t->dev, "[0x%p]\tcrop.h = %d\n",
				(void *)t, t->overlay.crop.h);
		dev_dbg(t->dev, "[0x%p]\tcrop.pos.x = %d\n",
				(void *)t, t->overlay.crop.pos.x);
		dev_dbg(t->dev, "[0x%p]\tcrop.pos.y = %d\n",
				(void *)t, t->overlay.crop.pos.y);
		dev_dbg(t->dev, "[0x%p]overlay buffer:\n", (void *)t);
		dev_dbg(t->dev, "[0x%p]\tpaddr = 0x%x\n",
				(void *)t, t->overlay.paddr);
		dev_dbg(t->dev, "[0x%p]\tov_off = 0x%x\n",
				(void *)t, t->set.ov_off);
		dev_dbg(t->dev, "[0x%p]\tov_uoff = 0x%x\n",
				(void *)t, t->set.ov_uoff);
		dev_dbg(t->dev, "[0x%p]\tov_voff = 0x%x\n",
				(void *)t, t->set.ov_voff);
		dev_dbg(t->dev, "[0x%p]\tovstride = %d\n",
				(void *)t, t->set.ovstride);
		if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL) {
			dev_dbg(t->dev, "[0x%p]local alpha enabled with:\n",
					(void *)t);
			dev_dbg(t->dev, "[0x%p]\tpaddr = 0x%x\n",
					(void *)t, t->overlay.alpha.loc_alp_paddr);
			dev_dbg(t->dev, "[0x%p]\tov_alpha_off = 0x%x\n",
					(void *)t, t->set.ov_alpha_off);
			dev_dbg(t->dev, "[0x%p]\tov_alpha_stride = %d\n",
					(void *)t, t->set.ov_alpha_stride);
		} else
			dev_dbg(t->dev, "[0x%p]globle alpha enabled with value 0x%x\n",
					(void *)t, t->overlay.alpha.gvalue);
		if (t->overlay.colorkey.enable)
			dev_dbg(t->dev, "[0x%p]colorkey enabled with value 0x%x\n",
					(void *)t, t->overlay.colorkey.value);
	}

	dev_dbg(t->dev, "[0x%p]want task_id = %d\n", (void *)t, t->task_id);
	dev_dbg(t->dev, "[0x%p]want task mode is 0x%x\n",
				(void *)t, t->set.mode);
	dev_dbg(t->dev, "[0x%p]\tIC_MODE = 0x%x\n", (void *)t, IC_MODE);
	dev_dbg(t->dev, "[0x%p]\tROT_MODE = 0x%x\n", (void *)t, ROT_MODE);
	dev_dbg(t->dev, "[0x%p]\tVDI_MODE = 0x%x\n", (void *)t, VDI_MODE);
}

static void dump_check_err(struct device *dev, int err)
{
	switch (err) {
	case IPU_CHECK_ERR_INPUT_CROP:
		dev_err(dev, "input crop setting error\n");
		break;
	case IPU_CHECK_ERR_OUTPUT_CROP:
		dev_err(dev, "output crop setting error\n");
		break;
	case IPU_CHECK_ERR_OVERLAY_CROP:
		dev_err(dev, "overlay crop setting error\n");
		break;
	case IPU_CHECK_ERR_INPUT_OVER_LIMIT:
		dev_err(dev, "input over limitation\n");
		break;
	case IPU_CHECK_ERR_OVERLAY_WITH_VDI:
		dev_err(dev, "do not support overlay with deinterlace\n");
		break;
	case IPU_CHECK_ERR_OV_OUT_NO_FIT:
		dev_err(dev,
			"width/height of overlay and ic output should be same\n");
		break;
	case IPU_CHECK_ERR_PROC_NO_NEED:
		dev_err(dev, "no ipu processing need\n");
		break;
	case IPU_CHECK_ERR_SPLIT_INPUTW_OVER:
		dev_err(dev, "split mode input width overflow\n");
		break;
	case IPU_CHECK_ERR_SPLIT_INPUTH_OVER:
		dev_err(dev, "split mode input height overflow\n");
		break;
	case IPU_CHECK_ERR_SPLIT_OUTPUTW_OVER:
		dev_err(dev, "split mode output width overflow\n");
		break;
	case IPU_CHECK_ERR_SPLIT_OUTPUTH_OVER:
		dev_err(dev, "split mode output height overflow\n");
		break;
	default:
		break;
	}
}

static void dump_check_warn(struct device *dev, int warn)
{
	if (warn & IPU_CHECK_WARN_INPUT_OFFS_NOT8ALIGN)
		dev_warn(dev, "input u/v offset not 8 align\n");
	if (warn & IPU_CHECK_WARN_OUTPUT_OFFS_NOT8ALIGN)
		dev_warn(dev, "output u/v offset not 8 align\n");
	if (warn & IPU_CHECK_WARN_OVERLAY_OFFS_NOT8ALIGN)
		dev_warn(dev, "overlay u/v offset not 8 align\n");
}

static int set_crop(struct ipu_crop *crop, int width, int height)
{
	if (crop->w || crop->h) {
		if (((crop->w + crop->pos.x) > width)
		|| ((crop->h + crop->pos.y) > height))
			return -EINVAL;
	} else {
		crop->pos.x = 0;
		crop->pos.y = 0;
		crop->w = width;
		crop->h = height;
	}
	crop->w -= crop->w%8;
	crop->h -= crop->h%8;

	return 0;
}

static void update_offset(unsigned int fmt,
				unsigned int width, unsigned int height,
				unsigned int pos_x, unsigned int pos_y,
				int *off, int *uoff, int *voff, int *stride)
{
	/* NOTE: u v offset should based on start point of off*/
	switch (fmt) {
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YUV420P:
		*off = pos_y * width + pos_x;
		*uoff = (width * (height - pos_y) - pos_x)
			+ ((width/2 * pos_y/2) + pos_x/2);
		*voff = *uoff + (width/2 * height/2);
		break;
	case IPU_PIX_FMT_YVU420P:
		*off = pos_y * width + pos_x;
		*voff = (width * (height - pos_y) - pos_x)
			+ ((width/2 * pos_y/2) + pos_x/2);
		*uoff = *voff + (width/2 * height/2);
		break;
	case IPU_PIX_FMT_YVU422P:
		*off = pos_y * width + pos_x;
		*voff = (width * (height - pos_y) - pos_x)
			+ ((width * pos_y)/2 + pos_x/2);
		*uoff = *voff + (width * height)/2;
		break;
	case IPU_PIX_FMT_YUV422P:
		*off = pos_y * width + pos_x;
		*uoff = (width * (height - pos_y) - pos_x)
			+ (width * pos_y)/2 + pos_x/2;
		*voff = *uoff + (width * height)/2;
		break;
	case IPU_PIX_FMT_NV12:
		*off = pos_y * width + pos_x;
		*uoff = (width * (height - pos_y) - pos_x)
			+ width * pos_y/2 + pos_x;
		break;
	default:
		*off = (pos_y * width + pos_x) * fmt_to_bpp(fmt)/8;
		break;
	}
	*stride = width * bytes_per_pixel(fmt);
}

static int update_split_setting(struct ipu_task_entry *t)
{
	struct stripe_param left_stripe;
	struct stripe_param right_stripe;
	struct stripe_param up_stripe;
	struct stripe_param down_stripe;
	u32 iw, ih, ow, oh;

	iw = t->input.crop.w;
	ih = t->input.crop.h;
	if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
		ow = t->output.crop.h;
		oh = t->output.crop.w;
	} else {
		ow = t->output.crop.w;
		oh = t->output.crop.h;
	}

	if (t->set.split_mode & RL_SPLIT) {
		ipu_calc_stripes_sizes(iw,
				ow,
				soc_max_out_width(),
				(((unsigned long long)1) << 32), /* 32bit for fractional*/
				1, /* equal stripes */
				t->input.format,
				t->output.format,
				&left_stripe,
				&right_stripe);
		t->set.sp_setting.iw = left_stripe.input_width;
		t->set.sp_setting.ow = left_stripe.output_width;
		t->set.sp_setting.outh_resize_ratio = left_stripe.irr;
		t->set.sp_setting.i_left_pos = left_stripe.input_column;
		t->set.sp_setting.o_left_pos = left_stripe.output_column;
		t->set.sp_setting.i_right_pos = right_stripe.input_column;
		t->set.sp_setting.o_right_pos = right_stripe.output_column;
	} else {
		t->set.sp_setting.iw = iw;
		t->set.sp_setting.ow = ow;
		t->set.sp_setting.outh_resize_ratio = 0;
		t->set.sp_setting.i_left_pos = 0;
		t->set.sp_setting.o_left_pos = 0;
		t->set.sp_setting.i_right_pos = 0;
		t->set.sp_setting.o_right_pos = 0;
	}
	if ((t->set.sp_setting.iw + t->set.sp_setting.i_right_pos) > iw)
		return IPU_CHECK_ERR_SPLIT_INPUTW_OVER;
	if (((t->set.sp_setting.ow + t->set.sp_setting.o_right_pos) > ow)
		|| (t->set.sp_setting.ow > soc_max_out_width()))
		return IPU_CHECK_ERR_SPLIT_OUTPUTW_OVER;

	if (t->set.split_mode & UD_SPLIT) {
		ipu_calc_stripes_sizes(ih,
				oh,
				soc_max_out_height(),
				(((unsigned long long)1) << 32), /* 32bit for fractional*/
				1, /* equal stripes */
				t->input.format,
				t->output.format,
				&up_stripe,
				&down_stripe);
		t->set.sp_setting.ih = up_stripe.input_width;
		t->set.sp_setting.oh = up_stripe.output_width;
		t->set.sp_setting.outv_resize_ratio = up_stripe.irr;
		t->set.sp_setting.i_top_pos = up_stripe.input_column;
		t->set.sp_setting.o_top_pos = up_stripe.output_column;
		t->set.sp_setting.i_bottom_pos = down_stripe.input_column;
		t->set.sp_setting.o_bottom_pos = down_stripe.output_column;
	} else {
		t->set.sp_setting.ih = ih;
		t->set.sp_setting.oh = oh;
		t->set.sp_setting.outv_resize_ratio = 0;
		t->set.sp_setting.i_top_pos = 0;
		t->set.sp_setting.o_top_pos = 0;
		t->set.sp_setting.i_bottom_pos = 0;
		t->set.sp_setting.o_bottom_pos = 0;
	}
	if ((t->set.sp_setting.ih + t->set.sp_setting.i_bottom_pos) > ih)
		return IPU_CHECK_ERR_SPLIT_INPUTH_OVER;
	if (((t->set.sp_setting.oh + t->set.sp_setting.o_bottom_pos) > oh)
		|| (t->set.sp_setting.oh > soc_max_out_height()))
		return IPU_CHECK_ERR_SPLIT_OUTPUTH_OVER;

	return IPU_CHECK_OK;
}

static int check_task(struct ipu_task_entry *t)
{
	int tmp;
	int ret = IPU_CHECK_OK;

	/* check input */
	ret = set_crop(&t->input.crop, t->input.width, t->input.height);
	if (ret < 0) {
		ret = IPU_CHECK_ERR_INPUT_CROP;
		goto done;
	} else
		update_offset(t->input.format, t->input.width, t->input.height,
				t->input.crop.pos.x, t->input.crop.pos.y,
				&t->set.i_off, &t->set.i_uoff,
				&t->set.i_voff, &t->set.istride);

	/* check output */
	ret = set_crop(&t->output.crop, t->output.width, t->output.height);
	if (ret < 0) {
		ret = IPU_CHECK_ERR_OUTPUT_CROP;
		goto done;
	} else
		update_offset(t->output.format,
				t->output.width, t->output.height,
				t->output.crop.pos.x, t->output.crop.pos.y,
				&t->set.o_off, &t->set.o_uoff,
				&t->set.o_voff, &t->set.ostride);

	/* check overlay if there is */
	if (t->overlay_en) {
		if (t->input.deinterlace.enable) {
			ret = IPU_CHECK_ERR_OVERLAY_WITH_VDI;
			goto done;
		}
		ret = set_crop(&t->overlay.crop, t->overlay.width, t->overlay.height);
		if (ret < 0) {
			ret = IPU_CHECK_ERR_OVERLAY_CROP;
			goto done;
		} else {
			int ow = t->output.crop.w;
			int oh = t->output.crop.h;

			if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
				ow = t->output.crop.h;
				oh = t->output.crop.w;
			}
			if ((t->overlay.crop.w != ow) || (t->overlay.crop.h != oh)) {
				ret = IPU_CHECK_ERR_OV_OUT_NO_FIT;
				goto done;
			}

			update_offset(t->overlay.format,
					t->overlay.width, t->overlay.height,
					t->overlay.crop.pos.x, t->overlay.crop.pos.y,
					&t->set.ov_off, &t->set.ov_uoff,
					&t->set.ov_voff, &t->set.ovstride);
			if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL) {
				t->set.ov_alpha_stride = t->overlay.width;
				t->set.ov_alpha_off = t->overlay.crop.pos.y *
					t->overlay.width + t->overlay.crop.pos.x;
			}
		}
	}

	/* input overflow? */
	if ((t->input.crop.w > soc_max_in_width()) ||
		(t->input.crop.h > soc_max_in_height())) {
		ret = IPU_CHECK_ERR_INPUT_OVER_LIMIT;
		goto done;
	}

	/* check task mode */
	t->set.mode = NULL_MODE;
	t->set.split_mode = NO_SPLIT;

	if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
		/*output swap*/
		tmp = t->output.crop.w;
		t->output.crop.w = t->output.crop.h;
		t->output.crop.h = tmp;
	}

	if (t->output.rotate >= IPU_ROTATE_90_RIGHT)
		t->set.mode |= ROT_MODE;

	/*need resize or CSC?*/
	if ((t->input.crop.w != t->output.crop.w) ||
			(t->input.crop.h != t->output.crop.h) ||
			need_csc(t->input.format, t->output.format))
		t->set.mode |= IC_MODE;

	/*need flip?*/
	if ((t->set.mode == NULL_MODE) && (t->output.rotate > IPU_ROTATE_NONE))
		t->set.mode |= IC_MODE;

	/*need IDMAC do format(same color space)?*/
	if ((t->set.mode == NULL_MODE) && (t->input.format != t->output.format))
		t->set.mode |= IC_MODE;

	/*overlay support*/
	if (t->overlay_en)
		t->set.mode |= IC_MODE;

	/*deinterlace*/
	if (t->input.deinterlace.enable) {
		t->set.mode &= ~IC_MODE;
		t->set.mode |= VDI_MODE;
	}

	if (t->set.mode & (IC_MODE | VDI_MODE)) {
		if (t->output.crop.w > soc_max_out_width())
			t->set.split_mode |= RL_SPLIT;
		if (t->output.crop.h > soc_max_out_height())
			t->set.split_mode |= UD_SPLIT;
		ret = update_split_setting(t);
		if (ret > IPU_CHECK_ERR_MIN)
			goto done;
	}

	if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
		/*output swap*/
		tmp = t->output.crop.w;
		t->output.crop.w = t->output.crop.h;
		t->output.crop.h = tmp;
	}

	if (t->set.mode == NULL_MODE) {
		ret = IPU_CHECK_ERR_PROC_NO_NEED;
		goto done;
	}

	if ((t->set.i_uoff % 8) || (t->set.i_voff % 8))
		ret |= IPU_CHECK_WARN_INPUT_OFFS_NOT8ALIGN;
	if ((t->set.o_uoff % 8) || (t->set.o_voff % 8))
		ret |= IPU_CHECK_WARN_OUTPUT_OFFS_NOT8ALIGN;
	if (t->overlay_en && ((t->set.ov_uoff % 8) || (t->set.ov_voff % 8)))
		ret |= IPU_CHECK_WARN_OVERLAY_OFFS_NOT8ALIGN;

done:
	/* dump msg */
	if (ret > IPU_CHECK_ERR_MIN)
		dump_check_err(t->dev, ret);
	else if (ret != IPU_CHECK_OK)
		dump_check_warn(t->dev, ret);

	return ret;
}

static int prepare_task(struct ipu_task_entry *t)
{
	int ret = 0;

	ret = check_task(t);
	if (ret > IPU_CHECK_ERR_MIN)
		return -EINVAL;

	dump_task_info(t);

	return ret;
}

/* should call from a process context */
static int queue_task(struct ipu_task_entry *t)
{
	int ret = 0;
	struct ipu_soc *ipu;
	struct list_head *task_list = NULL;
	struct mutex *task_lock = NULL;
	wait_queue_head_t *waitq = NULL;

	ipu = most_free_ipu_task(t);
	t->dev = ipu->dev;

	dev_dbg(t->dev, "[0x%p]Queue task: id %d\n", (void *)t, t->task_id);

	init_completion(&t->comp);

	t->set.task = 0;
	switch (t->task_id) {
	case IPU_TASK_ID_VF:
		task_list = &ipu->task_list[0];
		task_lock = &ipu->task_lock[0];
		waitq = &ipu->waitq[0];
		if (t->set.mode & IC_MODE)
			t->set.task |= IC_VF;
		else if (t->set.mode & VDI_MODE)
			t->set.task |= VDI_VF;
		if (t->set.mode & ROT_MODE)
			t->set.task |= ROT_VF;
		break;
	case IPU_TASK_ID_PP:
		task_list = &ipu->task_list[1];
		task_lock = &ipu->task_lock[1];
		waitq = &ipu->waitq[1];
		if (t->set.mode & IC_MODE)
			t->set.task |= IC_PP;
		if (t->set.mode & ROT_MODE)
			t->set.task |= ROT_PP;
		break;
	default:
		dev_err(t->dev, "[0x%p]should never come here\n", (void *)t);
	}

	dev_dbg(t->dev, "[0x%p]choose task_id[%d] mode[0x%x]\n",
			(void *)t, t->task_id, t->set.task);
	dev_dbg(t->dev, "[0x%p]\tIPU_TASK_ID_VF = %d\n",
				(void *)t, IPU_TASK_ID_VF);
	dev_dbg(t->dev, "[0x%p]\tIPU_TASK_ID_PP = %d\n",
				(void *)t, IPU_TASK_ID_PP);
	dev_dbg(t->dev, "[0x%p]\tIC_VF = 0x%x\n", (void *)t, IC_VF);
	dev_dbg(t->dev, "[0x%p]\tIC_PP = 0x%x\n", (void *)t, IC_PP);
	dev_dbg(t->dev, "[0x%p]\tROT_VF = 0x%x\n", (void *)t, ROT_VF);
	dev_dbg(t->dev, "[0x%p]\tROT_PP = 0x%x\n", (void *)t, ROT_PP);
	dev_dbg(t->dev, "[0x%p]\tVDI_VF = 0x%x\n", (void *)t, VDI_VF);

	/* add and wait task */
	mutex_lock(task_lock);
	list_add_tail(&t->node, task_list);
	mutex_unlock(task_lock);

	wake_up_interruptible(waitq);

	wait_for_completion(&t->comp);

	dev_dbg(t->dev, "[0x%p]Queue task finished\n", (void *)t);

	if (t->state != STATE_OK) {
		dev_err(t->dev, "[0x%p]state %d: %s\n",
			(void *)t, t->state, state_msg[t->state].msg);
		ret = -ECANCELED;
	}

	return ret;
}

static bool need_split(struct ipu_task_entry *t)
{
	return (t->set.split_mode != NO_SPLIT);
}

static int split_task_thread(void *data)
{
	struct ipu_split_task *t = data;

	t->ret = ipu_queue_sp_task(t);

	while (!kthread_should_stop())
		wait_event_interruptible(t->waitq, t->could_finish);

	return 0;
}

static int create_split_task(
		int stripe,
		struct ipu_split_task *sp_task)
{
	struct ipu_task *task = &(sp_task->task);
	struct ipu_task_entry *t = sp_task->parent_task;

	task->input = t->input;
	task->output = t->output;
	task->overlay_en = t->overlay_en;
	if (task->overlay_en)
		task->overlay = t->overlay;
	task->priority = t->priority;
	task->task_id = t->task_id;
	task->timeout = t->timeout;

	task->input.crop.w = t->set.sp_setting.iw;
	task->input.crop.h = t->set.sp_setting.ih;
	if (task->overlay_en) {
		task->overlay.crop.w = t->set.sp_setting.ow;
		task->overlay.crop.h = t->set.sp_setting.oh;
	}
	if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
		task->output.crop.w = t->set.sp_setting.oh;
		task->output.crop.h = t->set.sp_setting.ow;
	} else {
		task->output.crop.w = t->set.sp_setting.ow;
		task->output.crop.h = t->set.sp_setting.oh;
	}

	if (stripe & LEFT_STRIPE)
		task->input.crop.pos.x += t->set.sp_setting.i_left_pos;
	else if (stripe & RIGHT_STRIPE)
		task->input.crop.pos.x += t->set.sp_setting.i_right_pos;
	if (stripe & UP_STRIPE)
		task->input.crop.pos.y += t->set.sp_setting.i_top_pos;
	else if (stripe & DOWN_STRIPE)
		task->input.crop.pos.y += t->set.sp_setting.i_bottom_pos;

	if (task->overlay_en) {
		if (stripe & LEFT_STRIPE)
			task->overlay.crop.pos.x += t->set.sp_setting.o_left_pos;
		else if (stripe & RIGHT_STRIPE)
			task->overlay.crop.pos.x += t->set.sp_setting.o_right_pos;
		if (stripe & UP_STRIPE)
			task->overlay.crop.pos.y += t->set.sp_setting.o_top_pos;
		else if (stripe & DOWN_STRIPE)
			task->overlay.crop.pos.y += t->set.sp_setting.o_bottom_pos;
	}

	switch (t->output.rotate) {
	case IPU_ROTATE_NONE:
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_left_pos;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_right_pos;
		if (stripe & UP_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_top_pos;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_bottom_pos;
		break;
	case IPU_ROTATE_VERT_FLIP:
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_left_pos;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_right_pos;
		if (stripe & UP_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_top_pos - t->set.sp_setting.oh;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_bottom_pos - t->set.sp_setting.oh;
		break;
	case IPU_ROTATE_HORIZ_FLIP:
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_left_pos - t->set.sp_setting.ow;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_right_pos - t->set.sp_setting.ow;
		if (stripe & UP_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_top_pos;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_bottom_pos;
		break;
	case IPU_ROTATE_180:
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_left_pos - t->set.sp_setting.ow;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_right_pos - t->set.sp_setting.ow;
		if (stripe & UP_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_top_pos - t->set.sp_setting.oh;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_bottom_pos - t->set.sp_setting.oh;
		break;
	case IPU_ROTATE_90_RIGHT:
		if (stripe & UP_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_top_pos - t->set.sp_setting.oh;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_bottom_pos - t->set.sp_setting.oh;
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_left_pos;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_right_pos;
		break;
	case IPU_ROTATE_90_RIGHT_HFLIP:
		if (stripe & UP_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_top_pos;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_bottom_pos;
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_left_pos;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.y += t->set.sp_setting.o_right_pos;
		break;
	case IPU_ROTATE_90_RIGHT_VFLIP:
		if (stripe & UP_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_top_pos - t->set.sp_setting.oh;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.x =
					t->output.crop.pos.x + t->output.crop.w
					- t->set.sp_setting.o_bottom_pos - t->set.sp_setting.oh;
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_left_pos - t->set.sp_setting.ow;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_right_pos - t->set.sp_setting.ow;
		break;
	case IPU_ROTATE_90_LEFT:
		if (stripe & UP_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_top_pos;
		else if (stripe & DOWN_STRIPE)
			task->output.crop.pos.x += t->set.sp_setting.o_bottom_pos;
		if (stripe & LEFT_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_left_pos - t->set.sp_setting.ow;
		else if (stripe & RIGHT_STRIPE)
			task->output.crop.pos.y =
					t->output.crop.pos.y + t->output.crop.h
					- t->set.sp_setting.o_right_pos - t->set.sp_setting.ow;
		break;
	default:
		dev_err(t->dev, "should not be here\n");
		break;
	}

	sp_task->thread = kthread_run(split_task_thread, sp_task,
					"ipu_split_task");
	if (IS_ERR(sp_task->thread)) {
		dev_err(t->dev, "split thread can not create\n");
		return PTR_ERR(sp_task->thread);
	}

	return 0;
}

static int queue_split_task(struct ipu_task_entry *t)
{
	struct ipu_split_task sp_task[4];
	int i, ret = 0, size;

	dev_dbg(t->dev, "Split task 0x%p\n", (void *)t);

	if ((t->set.split_mode == RL_SPLIT) || (t->set.split_mode == UD_SPLIT))
		size = 2;
	else
		size = 4;

	for (i = 0; i < size; i++) {
		memset(&sp_task[i], 0, sizeof(struct ipu_split_task));
		init_waitqueue_head(&(sp_task[i].waitq));
		sp_task[i].could_finish = false;
		sp_task[i].parent_task = t;
	}

	if (t->set.split_mode == RL_SPLIT) {
		create_split_task(LEFT_STRIPE, &sp_task[0]);
		create_split_task(RIGHT_STRIPE, &sp_task[1]);
	} else if (t->set.split_mode == UD_SPLIT) {
		create_split_task(UP_STRIPE, &sp_task[0]);
		create_split_task(DOWN_STRIPE, &sp_task[1]);
	} else {
		create_split_task(LEFT_STRIPE | UP_STRIPE, &sp_task[0]);
		create_split_task(RIGHT_STRIPE | UP_STRIPE, &sp_task[1]);
		create_split_task(LEFT_STRIPE | DOWN_STRIPE, &sp_task[2]);
		create_split_task(RIGHT_STRIPE | DOWN_STRIPE, &sp_task[3]);
	}

	for (i = 0; i < size; i++) {
		sp_task[i].could_finish = true;
		wake_up_interruptible(&sp_task[i].waitq);
		kthread_stop(sp_task[i].thread);
		if (sp_task[i].ret < 0) {
			ret = sp_task[i].ret;
			dev_err(t->dev,
				"split task %d fail with ret %d\n",
				i, ret);
		}
	}

	return ret;
}

static struct ipu_task_entry *create_task_entry(struct ipu_task *task)
{
	struct ipu_task_entry *tsk;

	tsk = kzalloc(sizeof(struct ipu_task_entry), GFP_KERNEL);
	if (!tsk)
		return ERR_PTR(-ENOMEM);

	tsk->dev = ipu_dev;
	tsk->input = task->input;
	tsk->output = task->output;
	tsk->overlay_en = task->overlay_en;
	if (tsk->overlay_en)
		tsk->overlay = task->overlay;
	tsk->priority = task->priority;
	tsk->task_id = task->task_id;
	if (task->timeout && (task->timeout > DEF_TIMEOUT_MS))
		tsk->timeout = task->timeout;
	else
		tsk->timeout = DEF_TIMEOUT_MS;

	return tsk;
}

int ipu_check_task(struct ipu_task *task)
{
	struct ipu_task_entry *tsk;
	int ret = 0;

	tsk = create_task_entry(task);
	if (IS_ERR(tsk))
		return PTR_ERR(tsk);

	ret = check_task(tsk);

	task->input = tsk->input;
	task->output = tsk->output;
	task->overlay = tsk->overlay;

	kfree(tsk);

	return ret;
}
EXPORT_SYMBOL_GPL(ipu_check_task);

int ipu_queue_sp_task(struct ipu_split_task *sp_task)
{
	struct ipu_task_entry *tsk;
	int ret;

	tsk = create_task_entry(&sp_task->task);
	if (IS_ERR(tsk))
		return PTR_ERR(tsk);

	ret = prepare_task(tsk);
	if (ret < 0)
		goto done;

	tsk->set.sp_setting = sp_task->parent_task->set.sp_setting;
	tsk->set.sp_setting = sp_task->parent_task->set.sp_setting;

	ret = queue_task(tsk);
done:
	kfree(tsk);
	return ret;
}

int ipu_queue_task(struct ipu_task *task)
{
	struct ipu_task_entry *tsk;
	int ret;

	tsk = create_task_entry(task);
	if (IS_ERR(tsk))
		return PTR_ERR(tsk);

	ret = prepare_task(tsk);
	if (ret < 0)
		goto done;

	if (need_split(tsk))
		ret = queue_split_task(tsk);
	else
		ret = queue_task(tsk);
done:
	kfree(tsk);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_queue_task);

static bool only_ic(u8 mode)
{
	return ((mode == IC_MODE) || (mode == VDI_MODE));
}

static bool only_rot(u8 mode)
{
	return (mode == ROT_MODE);
}

static bool ic_and_rot(u8 mode)
{
	return ((mode == (IC_MODE | ROT_MODE)) ||
		 (mode == (VDI_MODE | ROT_MODE)));
}

static int init_ic(struct ipu_soc *ipu, struct ipu_task_entry *t)
{
	int ret = 0;
	ipu_channel_params_t params;
	dma_addr_t inbuf = 0, ovbuf = 0, ov_alp_buf = 0;
	dma_addr_t inbuf_p = 0, inbuf_n = 0;
	dma_addr_t outbuf = 0;
	int out_uoff = 0, out_voff = 0, out_rot;
	int out_w = 0, out_h = 0, out_stride;
	int out_fmt;

	memset(&params, 0, sizeof(params));

	/* is it need link a rot channel */
	if (ic_and_rot(t->set.mode)) {
		outbuf = t->set.r_paddr;
		out_w = t->set.r_width;
		out_h = t->set.r_height;
		out_stride = t->set.r_stride;
		out_fmt = t->set.r_fmt;
		out_uoff = 0;
		out_voff = 0;
		out_rot = IPU_ROTATE_NONE;
	} else {
		outbuf = t->output.paddr + t->set.o_off;
		out_w = t->output.crop.w;
		out_h = t->output.crop.h;
		out_stride = t->set.ostride;
		out_fmt = t->output.format;
		out_uoff = t->set.o_uoff;
		out_voff = t->set.o_voff;
		out_rot = t->output.rotate;
	}

	/* settings */
	params.mem_prp_vf_mem.in_width = t->input.crop.w;
	params.mem_prp_vf_mem.out_width = out_w;
	params.mem_prp_vf_mem.in_height = t->input.crop.h;
	params.mem_prp_vf_mem.out_height = out_h;
	params.mem_prp_vf_mem.in_pixel_fmt = t->input.format;
	params.mem_prp_vf_mem.out_pixel_fmt = out_fmt;
	params.mem_prp_vf_mem.motion_sel = t->input.deinterlace.motion;

	params.mem_prp_vf_mem.outh_resize_ratio =
			t->set.sp_setting.outh_resize_ratio;
	params.mem_prp_vf_mem.outv_resize_ratio =
			t->set.sp_setting.outv_resize_ratio;

	if (t->overlay_en) {
		params.mem_prp_vf_mem.in_g_pixel_fmt = t->overlay.format;
		params.mem_prp_vf_mem.graphics_combine_en = 1;
		if (t->overlay.alpha.mode == IPU_ALPHA_MODE_GLOBAL)
			params.mem_prp_vf_mem.global_alpha_en = 1;
		else
			params.mem_prp_vf_mem.alpha_chan_en = 1;
		params.mem_prp_vf_mem.alpha = t->overlay.alpha.gvalue;
		if (t->overlay.colorkey.enable) {
			params.mem_prp_vf_mem.key_color_en = 1;
			params.mem_prp_vf_mem.key_color = t->overlay.colorkey.value;
		}
	}

	/* init channels */
	ret = ipu_init_channel(ipu, t->set.ic_chan, &params);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_FAIL;
		goto done;
	}

	if (deinterlace_3_field(t)) {
		ret = ipu_init_channel(ipu, t->set.vdi_ic_p_chan, &params);
		if (ret < 0) {
			t->state = STATE_INIT_CHAN_FAIL;
			goto done;
		}
		ret = ipu_init_channel(ipu, t->set.vdi_ic_n_chan, &params);
		if (ret < 0) {
			t->state = STATE_INIT_CHAN_FAIL;
			goto done;
		}
	}

	/* init channel bufs */
	if (deinterlace_3_field(t)) {
		inbuf_p = t->input.paddr + t->set.istride + t->set.i_off;
		inbuf = t->input.paddr_n + t->set.i_off;
		inbuf_n = t->input.paddr_n + t->set.istride + t->set.i_off;
	} else
		inbuf = t->input.paddr + t->set.i_off;

	if (t->overlay_en) {
		ovbuf = t->overlay.paddr + t->set.ov_off;
		if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL)
			ov_alp_buf = t->overlay.alpha.loc_alp_paddr
				+ t->set.ov_alpha_off;
	}

	ret = ipu_init_channel_buffer(ipu,
			t->set.ic_chan,
			IPU_INPUT_BUFFER,
			t->input.format,
			t->input.crop.w,
			t->input.crop.h,
			t->set.istride,
			IPU_ROTATE_NONE,
			inbuf,
			0,
			0,
			t->set.i_uoff,
			t->set.i_voff);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_BUF_FAIL;
		goto done;
	}

	if (deinterlace_3_field(t)) {
		ret = ipu_init_channel_buffer(ipu,
				t->set.vdi_ic_p_chan,
				IPU_INPUT_BUFFER,
				t->input.format,
				t->input.crop.w,
				t->input.crop.h,
				t->set.istride,
				IPU_ROTATE_NONE,
				inbuf_p,
				0,
				0,
				t->set.i_uoff,
				t->set.i_voff);
		if (ret < 0) {
			t->state = STATE_INIT_CHAN_BUF_FAIL;
			goto done;
		}

		ret = ipu_init_channel_buffer(ipu,
				t->set.vdi_ic_n_chan,
				IPU_INPUT_BUFFER,
				t->input.format,
				t->input.crop.w,
				t->input.crop.h,
				t->set.istride,
				IPU_ROTATE_NONE,
				inbuf_n,
				0,
				0,
				t->set.i_uoff,
				t->set.i_voff);
		if (ret < 0) {
			t->state = STATE_INIT_CHAN_BUF_FAIL;
			goto done;
		}
	}

	if (t->overlay_en) {
		ret = ipu_init_channel_buffer(ipu,
				t->set.ic_chan,
				IPU_GRAPH_IN_BUFFER,
				t->overlay.format,
				t->overlay.crop.w,
				t->overlay.crop.h,
				t->set.ovstride,
				IPU_ROTATE_NONE,
				ovbuf,
				0,
				0,
				t->set.ov_uoff,
				t->set.ov_voff);
		if (ret < 0) {
			t->state = STATE_INIT_CHAN_BUF_FAIL;
			goto done;
		}

		if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL) {
			ret = ipu_init_channel_buffer(ipu,
					t->set.ic_chan,
					IPU_ALPHA_IN_BUFFER,
					IPU_PIX_FMT_GENERIC,
					t->overlay.crop.w,
					t->overlay.crop.h,
					t->set.ov_alpha_stride,
					IPU_ROTATE_NONE,
					ov_alp_buf,
					0,
					0,
					0, 0);
			if (ret < 0) {
				t->state = STATE_INIT_CHAN_BUF_FAIL;
				goto done;
			}
		}
	}

	ret = ipu_init_channel_buffer(ipu,
			t->set.ic_chan,
			IPU_OUTPUT_BUFFER,
			out_fmt,
			out_w,
			out_h,
			out_stride,
			out_rot,
			outbuf,
			0,
			0,
			out_uoff,
			out_voff);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_BUF_FAIL;
		goto done;
	}

done:
	return ret;
}

static void uninit_ic(struct ipu_soc *ipu, struct ipu_task_entry *t)
{
	ipu_uninit_channel(ipu, t->set.ic_chan);
	if (deinterlace_3_field(t)) {
		ipu_uninit_channel(ipu, t->set.vdi_ic_p_chan);
		ipu_uninit_channel(ipu, t->set.vdi_ic_n_chan);
	}
}

static int init_rot(struct ipu_soc *ipu, struct ipu_task_entry *t)
{
	int ret = 0;
	dma_addr_t inbuf = 0, outbuf = 0;
	int in_uoff = 0, in_voff = 0;
	int in_fmt, in_width, in_height, in_stride;

	/* init channel */
	ret = ipu_init_channel(ipu, t->set.rot_chan, NULL);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_FAIL;
		goto done;
	}

	/* init channel buf */
	/* is it need link to a ic channel */
	if (ic_and_rot(t->set.mode)) {
		in_fmt = t->set.r_fmt;
		in_width = t->set.r_width;
		in_height = t->set.r_height;
		in_stride = t->set.r_stride;
		inbuf = t->set.r_paddr;
		in_uoff = 0;
		in_voff = 0;
	} else {
		in_fmt = t->input.format;
		in_width = t->input.crop.w;
		in_height = t->input.crop.h;
		in_stride = t->set.istride;
		inbuf = t->input.paddr + t->set.i_off;
		in_uoff = t->set.i_uoff;
		in_voff = t->set.i_voff;
	}
	outbuf = t->output.paddr + t->set.o_off;

	ret = ipu_init_channel_buffer(ipu,
			t->set.rot_chan,
			IPU_INPUT_BUFFER,
			in_fmt,
			in_width,
			in_height,
			t->set.istride,
			t->output.rotate,
			inbuf,
			0,
			0,
			in_uoff,
			in_voff);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_BUF_FAIL;
		goto done;
	}

	ret = ipu_init_channel_buffer(ipu,
			t->set.rot_chan,
			IPU_OUTPUT_BUFFER,
			t->output.format,
			t->output.crop.w,
			t->output.crop.h,
			t->set.ostride,
			IPU_ROTATE_NONE,
			outbuf,
			0,
			0,
			t->set.o_uoff,
			t->set.o_voff);
	if (ret < 0) {
		t->state = STATE_INIT_CHAN_BUF_FAIL;
		goto done;
	}

done:
	return ret;
}

static void uninit_rot(struct ipu_soc *ipu, struct ipu_task_entry *t)
{
	ipu_uninit_channel(ipu, t->set.rot_chan);
}

static int get_irq(struct ipu_task_entry *t)
{
	int irq;
	ipu_channel_t chan;

	if (only_ic(t->set.mode))
		chan = t->set.ic_chan;
	else
		chan = t->set.rot_chan;

	switch (chan) {
	case MEM_ROT_VF_MEM:
		irq = IPU_IRQ_PRP_VF_ROT_OUT_EOF;
		break;
	case MEM_ROT_PP_MEM:
		irq = IPU_IRQ_PP_ROT_OUT_EOF;
		break;
	case MEM_VDI_PRP_VF_MEM:
	case MEM_PRP_VF_MEM:
		irq = IPU_IRQ_PRP_VF_OUT_EOF;
		break;
	case MEM_PP_MEM:
		irq = IPU_IRQ_PP_OUT_EOF;
		break;
	default:
		irq = -EINVAL;
	}

	return irq;
}

static irqreturn_t task_irq_handler(int irq, void *dev_id)
{
	struct completion *comp = dev_id;
	complete(comp);
	return IRQ_HANDLED;
}

static void do_task(struct ipu_soc *ipu, struct ipu_task_entry *t)
{
	struct completion comp;
	void *r_vaddr;
	int r_size;
	int irq;
	int ret;

	if (!ipu) {
		t->state = STATE_NO_IPU;
		return;
	}

	dev_dbg(ipu->dev, "[0x%p]Do task: id %d\n", (void *)t, t->task_id);
	dump_task_info(t);

	if (t->set.task & IC_PP) {
		t->set.ic_chan = MEM_PP_MEM;
		dev_dbg(ipu->dev, "[0x%p]ic channel MEM_PP_MEM\n", (void *)t);
	} else if (t->set.task & IC_VF) {
		t->set.ic_chan = MEM_PRP_VF_MEM;
		dev_dbg(ipu->dev, "[0x%p]ic channel MEM_PRP_VF_MEM\n", (void *)t);
	} else if (t->set.task & VDI_VF) {
		t->set.ic_chan = MEM_VDI_PRP_VF_MEM;
		if (deinterlace_3_field(t)) {
			t->set.vdi_ic_p_chan = MEM_VDI_PRP_VF_MEM_P;
			t->set.vdi_ic_n_chan = MEM_VDI_PRP_VF_MEM_N;
		}
		dev_dbg(ipu->dev, "[0x%p]ic channel MEM_VDI_PRP_VF_MEM\n", (void *)t);
	}

	if (t->set.task & ROT_PP) {
		t->set.rot_chan = MEM_ROT_PP_MEM;
		dev_dbg(ipu->dev, "[0x%p]rot channel MEM_ROT_PP_MEM\n", (void *)t);
	} else if (t->set.task & ROT_VF) {
		t->set.rot_chan = MEM_ROT_VF_MEM;
		dev_dbg(ipu->dev, "[0x%p]rot channel MEM_ROT_VF_MEM\n", (void *)t);
	}

	/* channel setup */
	if (only_ic(t->set.mode)) {
		dev_dbg(t->dev, "[0x%p]only ic mode\n", (void *)t);
		ret = init_ic(ipu, t);
		if (ret < 0)
			goto chan_done;
	} else if (only_rot(t->set.mode)) {
		dev_dbg(t->dev, "[0x%p]only rot mode\n", (void *)t);
		ret = init_rot(ipu, t);
		if (ret < 0)
			goto chan_done;
	} else if (ic_and_rot(t->set.mode)) {
		dev_dbg(t->dev, "[0x%p]ic + rot mode\n", (void *)t);
		t->set.r_fmt = t->output.format;
		if (t->output.rotate >= IPU_ROTATE_90_RIGHT) {
			t->set.r_width = t->output.crop.h;
			t->set.r_height = t->output.crop.w;
		} else {
			t->set.r_width = t->output.crop.w;
			t->set.r_height = t->output.crop.h;
		}
		t->set.r_stride = t->set.r_width *
			bytes_per_pixel(t->set.r_fmt);
		r_size = t->set.r_width * t->set.r_height
			* fmt_to_bpp(t->set.r_fmt)/8;
		r_vaddr = dma_alloc_coherent(t->dev,
					  r_size,
					  &t->set.r_paddr,
					  GFP_DMA | GFP_KERNEL);
		if (r_vaddr == NULL) {
			ret = -ENOMEM;
			goto chan_done;
		}

		dev_dbg(t->dev, "[0x%p]rotation:\n", (void *)t);
		dev_dbg(t->dev, "[0x%p]\tformat = 0x%x\n", (void *)t, t->set.r_fmt);
		dev_dbg(t->dev, "[0x%p]\twidth = %d\n", (void *)t, t->set.r_width);
		dev_dbg(t->dev, "[0x%p]\theight = %d\n", (void *)t, t->set.r_height);
		dev_dbg(t->dev, "[0x%p]\tpaddr = 0x%x\n", (void *)t, t->set.r_paddr);
		dev_dbg(t->dev, "[0x%p]\trstride = %d\n", (void *)t, t->set.r_stride);

		ret = init_ic(ipu, t);
		if (ret < 0)
			goto chan_done;
		ret = init_rot(ipu, t);
		if (ret < 0)
			goto chan_done;
		ret = ipu_link_channels(ipu, t->set.ic_chan,
				t->set.rot_chan);
		if (ret < 0) {
			t->state = STATE_LINK_CHAN_FAIL;
			goto chan_done;
		}
	} else {
		dev_err(t->dev, "[0x%p]do_task: should not be here\n", (void *)t);
		return;
	}

	/* irq setup */
	irq = get_irq(t);
	if (irq < 0) {
		t->state = STATE_NO_IRQ;
		goto chan_done;
	}

	dev_dbg(t->dev, "[0x%p]task irq is %d\n", (void *)t, irq);

	init_completion(&comp);
	ret = ipu_request_irq(ipu, irq, task_irq_handler, 0, NULL, &comp);
	if (ret < 0) {
		t->state = STATE_IRQ_FAIL;
		goto chan_done;
	}

	/* enable/start channel */
	if (only_ic(t->set.mode)) {
		ipu_enable_channel(ipu, t->set.ic_chan);
		if (deinterlace_3_field(t)) {
			ipu_enable_channel(ipu, t->set.vdi_ic_p_chan);
			ipu_enable_channel(ipu, t->set.vdi_ic_n_chan);
		}

		ipu_select_buffer(ipu, t->set.ic_chan, IPU_OUTPUT_BUFFER, 0);
		if (t->overlay_en) {
			ipu_select_buffer(ipu, t->set.ic_chan, IPU_GRAPH_IN_BUFFER, 0);
			if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL)
				ipu_select_buffer(ipu, t->set.ic_chan, IPU_ALPHA_IN_BUFFER, 0);
		}
		if (deinterlace_3_field(t))
			ipu_select_multi_vdi_buffer(ipu, 0);
		else
			ipu_select_buffer(ipu, t->set.ic_chan, IPU_INPUT_BUFFER, 0);
	} else if (only_rot(t->set.mode)) {
		ipu_enable_channel(ipu, t->set.rot_chan);
		ipu_select_buffer(ipu, t->set.rot_chan, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(ipu, t->set.rot_chan, IPU_INPUT_BUFFER, 0);
	} else if (ic_and_rot(t->set.mode)) {
		ipu_enable_channel(ipu, t->set.rot_chan);
		ipu_enable_channel(ipu, t->set.ic_chan);
		if (deinterlace_3_field(t)) {
			ipu_enable_channel(ipu, t->set.vdi_ic_p_chan);
			ipu_enable_channel(ipu, t->set.vdi_ic_n_chan);
		}

		ipu_select_buffer(ipu, t->set.rot_chan, IPU_OUTPUT_BUFFER, 0);
		if (t->overlay_en) {
			ipu_select_buffer(ipu, t->set.ic_chan, IPU_GRAPH_IN_BUFFER, 0);
			if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL)
				ipu_select_buffer(ipu, t->set.ic_chan, IPU_ALPHA_IN_BUFFER, 0);
		}
		if (deinterlace_3_field(t))
			ipu_select_multi_vdi_buffer(ipu, 0);
		else
			ipu_select_buffer(ipu, t->set.ic_chan, IPU_INPUT_BUFFER, 0);
		ipu_select_buffer(ipu, t->set.ic_chan, IPU_OUTPUT_BUFFER, 0);
	}

	ret = wait_for_completion_timeout(&comp, msecs_to_jiffies(t->timeout));
	if (ret == 0)
		t->state = STATE_IRQ_TIMEOUT;

	ipu_free_irq(ipu, irq, &comp);

	if (only_ic(t->set.mode)) {
		ipu_disable_channel(ipu, t->set.ic_chan, true);
		if (deinterlace_3_field(t)) {
			ipu_disable_channel(ipu, t->set.vdi_ic_p_chan, true);
			ipu_disable_channel(ipu, t->set.vdi_ic_n_chan, true);
		}
	} else if (only_rot(t->set.mode))
		ipu_disable_channel(ipu, t->set.rot_chan, true);
	else if (ic_and_rot(t->set.mode)) {
		ipu_disable_channel(ipu, t->set.ic_chan, true);
		if (deinterlace_3_field(t)) {
			ipu_disable_channel(ipu, t->set.vdi_ic_p_chan, true);
			ipu_disable_channel(ipu, t->set.vdi_ic_n_chan, true);
		}
		ipu_disable_channel(ipu, t->set.rot_chan, true);
	}

chan_done:
	if (only_ic(t->set.mode))
		uninit_ic(ipu, t);
	else if (only_rot(t->set.mode))
		uninit_rot(ipu, t);
	else if (ic_and_rot(t->set.mode)) {
		ipu_unlink_channels(ipu, t->set.ic_chan,
			t->set.rot_chan);
		uninit_ic(ipu, t);
		uninit_rot(ipu, t);
		if (r_vaddr)
			dma_free_coherent(t->dev,
					r_size,
					r_vaddr,
					t->set.r_paddr);
	}
	return;
}

static int thread_loop(struct ipu_soc *ipu, int id)
{
	struct ipu_task_entry *tsk;
	struct list_head *task_list = &ipu->task_list[id];
	struct mutex *task_lock = &ipu->task_lock[id];
	int ret;

	while (!kthread_should_stop()) {
		int found = 0;

		ret = wait_event_interruptible(ipu->waitq[id], !list_empty(task_list));
		if (0 != ret)
			continue;

		mutex_lock(task_lock);

		list_for_each_entry(tsk, task_list, node) {
			if (tsk->priority == IPU_TASK_PRIORITY_HIGH) {
				found = 1;
				break;
			}
		}

		if (!found)
			tsk = list_first_entry(task_list, struct ipu_task_entry, node);

		mutex_unlock(task_lock);

		do_task(ipu, tsk);

		mutex_lock(task_lock);
		list_del(&tsk->node);
		mutex_unlock(task_lock);

		complete(&tsk->comp);
	}

	return 0;
}

static int task_vf_thread(void *data)
{
	struct ipu_soc *ipu = data;

	thread_loop(ipu, 0);

	return 0;
}

static int task_pp_thread(void *data)
{
	struct ipu_soc *ipu = data;

	thread_loop(ipu, 1);

	return 0;
}

static int mxc_ipu_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long mxc_ipu_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int __user *argp = (void __user *)arg;
	int ret = 0;

	switch (cmd) {
	case IPU_CHECK_TASK:
		{
			struct ipu_task task;

			if (copy_from_user
					(&task, (struct ipu_task *) arg,
					 sizeof(struct ipu_task)))
				return -EFAULT;
			ret = ipu_check_task(&task);
			if (copy_to_user((struct ipu_task *) arg,
				&task, sizeof(struct ipu_task)))
				return -EFAULT;
			break;
		}
	case IPU_QUEUE_TASK:
		{
			struct ipu_task task;

			if (copy_from_user
					(&task, (struct ipu_task *) arg,
					 sizeof(struct ipu_task)))
				return -EFAULT;
			ret = ipu_queue_task(&task);
			break;
		}
	case IPU_ALLOC:
		{
			int size;
			struct ipu_alloc_list *mem;

			mem = kzalloc(sizeof(*mem), GFP_KERNEL);
			if (mem == NULL)
				return -ENOMEM;

			if (get_user(size, argp))
				return -EFAULT;

			mem->size = PAGE_ALIGN(size);

			mem->cpu_addr = dma_alloc_coherent(ipu_dev, size,
							   &mem->phy_addr,
							   GFP_DMA);
			if (mem->cpu_addr == NULL) {
				kfree(mem);
				return -ENOMEM;
			}

			list_add(&mem->list, &ipu_alloc_list);

			dev_dbg(ipu_dev, "allocated %d bytes @ 0x%08X\n",
				mem->size, mem->phy_addr);

			if (put_user(mem->phy_addr, argp))
				return -EFAULT;

			break;
		}
	case IPU_FREE:
		{
			unsigned long offset;
			struct ipu_alloc_list *mem;

			if (get_user(offset, argp))
				return -EFAULT;

			ret = -EINVAL;
			list_for_each_entry(mem, &ipu_alloc_list, list) {
				if (mem->phy_addr == offset) {
					list_del(&mem->list);
					dma_free_coherent(ipu_dev,
							  mem->size,
							  mem->cpu_addr,
							  mem->phy_addr);
					kfree(mem);
					ret = 0;
					break;
				}
			}

			break;
		}
	default:
		break;
	}
	return ret;
}

static int mxc_ipu_mmap(struct file *file, struct vm_area_struct *vma)
{
	bool found = false;
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct ipu_alloc_list *mem;

	list_for_each_entry(mem, &ipu_alloc_list, list) {
		if (offset == mem->phy_addr) {
			found = true;
			len = mem->size;
			break;
		}
	}
	if (!found)
		return -EINVAL;

	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		printk(KERN_ERR
				"mmap failed!\n");
		return -ENOBUFS;
	}
	return 0;
}

static int mxc_ipu_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations mxc_ipu_fops = {
	.owner = THIS_MODULE,
	.open = mxc_ipu_open,
	.mmap = mxc_ipu_mmap,
	.release = mxc_ipu_release,
	.unlocked_ioctl = mxc_ipu_ioctl,
};

int register_ipu_device(struct ipu_soc *ipu, int id)
{
	int ret = 0;

	if (!major) {
		major = register_chrdev(0, "mxc_ipu", &mxc_ipu_fops);
		if (major < 0) {
			printk(KERN_ERR "Unable to register mxc_ipu as a char device\n");
			ret = major;
			goto register_cdev_fail;
		}

		ipu_class = class_create(THIS_MODULE, "mxc_ipu");
		if (IS_ERR(ipu_class)) {
			ret = PTR_ERR(ipu_class);
			goto ipu_class_fail;
		}

		ipu_dev = device_create(ipu_class, NULL, MKDEV(major, 0),
				NULL, "mxc_ipu");
		if (IS_ERR(ipu_dev)) {
			ret = PTR_ERR(ipu_dev);
			goto dev_create_fail;
		}
		ipu_dev->dma_mask = kmalloc(sizeof(*ipu_dev->dma_mask), GFP_KERNEL);
		*ipu_dev->dma_mask = DMA_BIT_MASK(32);
		ipu_dev->coherent_dma_mask = DMA_BIT_MASK(32);
	}

	INIT_LIST_HEAD(&ipu->task_list[0]);
	INIT_LIST_HEAD(&ipu->task_list[1]);
	init_waitqueue_head(&ipu->waitq[0]);
	init_waitqueue_head(&ipu->waitq[1]);
	mutex_init(&ipu->task_lock[0]);
	mutex_init(&ipu->task_lock[1]);
	ipu->thread[0] = kthread_run(task_vf_thread, ipu,
					"ipu%d_process-vf", id);
	if (IS_ERR(ipu->thread[0])) {
		ret = PTR_ERR(ipu->thread[0]);
		goto kthread0_fail;
	}

	ipu->thread[1] = kthread_run(task_pp_thread, ipu,
				"ipu%d_process-pp", id);
	if (IS_ERR(ipu->thread[1])) {
		ret = PTR_ERR(ipu->thread[1]);
		goto kthread1_fail;
	}

	return ret;

kthread1_fail:
	kthread_stop(ipu->thread[0]);
kthread0_fail:
	if (id == 0)
		device_destroy(ipu_class, MKDEV(major, 0));
dev_create_fail:
	if (id == 0) {
		class_destroy(ipu_class);
		unregister_chrdev(major, "mxc_ipu");
	}
ipu_class_fail:
	if (id == 0)
		unregister_chrdev(major, "mxc_ipu");
register_cdev_fail:
	return ret;
}

void unregister_ipu_device(struct ipu_soc *ipu, int id)
{
	kthread_stop(ipu->thread[0]);
	kthread_stop(ipu->thread[1]);
	if (major) {
		device_destroy(ipu_class, MKDEV(major, 0));
		class_destroy(ipu_class);
		unregister_chrdev(major, "mxc_ipu");
		major = 0;
	}
}
