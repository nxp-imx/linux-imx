/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <linux/mxcfb.h>
#include <linux/console.h>
#include <linux/mxc_v4l2.h>
#include <mach/ipu-v3.h>

#include <media/videobuf-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#define MAX_FB_NUM	6
#define FB_BUFS		3

struct mxc_vout_fb {
	char *name;
	int ipu_id;
	struct v4l2_rect crop_bounds;
	unsigned int disp_fmt;
	bool disp_support_csc;
	bool disp_support_windows;
};

struct mxc_vout_output {
	int open_cnt;
	struct fb_info *fbi;
	struct video_device *vfd;
	struct mutex mutex;
	struct mutex task_lock;
	enum v4l2_buf_type type;

	struct videobuf_queue vbq;
	spinlock_t vbq_lock;

	struct list_head queue_list;
	struct list_head active_list;

	struct v4l2_rect crop_bounds;
	unsigned int disp_fmt;
	struct mxcfb_pos win_pos;
	bool disp_support_windows;
	bool disp_support_csc;

	bool fmt_init;
	bool bypass_pp;
	struct ipu_task	task;

	bool timer_stop;
	struct timer_list timer;
	struct workqueue_struct *v4l_wq;
	struct work_struct disp_work;
	unsigned long frame_count;
	unsigned long start_jiffies;

	int ctrl_rotate;
	int ctrl_vflip;
	int ctrl_hflip;

	dma_addr_t disp_bufs[FB_BUFS];

	struct videobuf_buffer *pre_vb;
};

struct mxc_vout_dev {
	struct device	*dev;
	struct v4l2_device v4l2_dev;
	struct mxc_vout_output *out[MAX_FB_NUM];
	int out_num;
};

/* Driver Configuration macros */
#define VOUT_NAME		"mxc_vout"

/* Variables configurable through module params*/
static int debug;
static int video_nr = 16;

/* Module parameters */
module_param(video_nr, int, S_IRUGO);
MODULE_PARM_DESC(video_nr, "video device numbers");
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

const static struct v4l2_fmtdesc mxc_formats[] = {
	{
		.description = "RGB565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
	},
	{
		.description = "BGR24",
		.pixelformat = V4L2_PIX_FMT_BGR24,
	},
	{
		.description = "RGB24",
		.pixelformat = V4L2_PIX_FMT_RGB24,
	},
	{
		.description = "RGB32",
		.pixelformat = V4L2_PIX_FMT_RGB32,
	},
	{
		.description = "BGR32",
		.pixelformat = V4L2_PIX_FMT_BGR32,
	},
	{
		.description = "NV12",
		.pixelformat = V4L2_PIX_FMT_NV12,
	},
	{
		.description = "UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
		.description = "YUYV",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "YUV422 planar",
		.pixelformat = V4L2_PIX_FMT_YUV422P,
	},
	{
		.description = "YUV444",
		.pixelformat = V4L2_PIX_FMT_YUV444,
	},
	{
		.description = "YUV420",
		.pixelformat = V4L2_PIX_FMT_YUV420,
	},
};

#define NUM_MXC_VOUT_FORMATS (ARRAY_SIZE(mxc_formats))

#define DEF_INPUT_WIDTH		320
#define DEF_INPUT_HEIGHT	240

static int mxc_vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i);

static struct mxc_vout_fb g_fb_setting[MAX_FB_NUM];
static int config_disp_output(struct mxc_vout_output *vout);
static void release_disp_output(struct mxc_vout_output *vout);

static ipu_channel_t get_ipu_channel(struct fb_info *fbi)
{
	ipu_channel_t ipu_ch = CHAN_NONE;
	mm_segment_t old_fs;

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_GET_FB_IPU_CHAN,
				(unsigned long)&ipu_ch);
		set_fs(old_fs);
	}

	return ipu_ch;
}

static unsigned int get_ipu_fmt(struct fb_info *fbi)
{
	mm_segment_t old_fs;
	unsigned int fb_fmt;

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_GET_DIFMT,
				(unsigned long)&fb_fmt);
		set_fs(old_fs);
	}

	return fb_fmt;
}

static void update_display_setting(void)
{
	int i;
	struct fb_info *fbi;
	struct v4l2_rect bg_crop_bounds[2];

	for (i = 0; i < num_registered_fb; i++) {
		fbi = registered_fb[i];

		memset(&g_fb_setting[i], 0, sizeof(struct mxc_vout_fb));

		if (!strncmp(fbi->fix.id, "DISP3", 5))
			g_fb_setting[i].ipu_id = 0;
		else
			g_fb_setting[i].ipu_id = 1;

		g_fb_setting[i].name = fbi->fix.id;
		g_fb_setting[i].crop_bounds.left = 0;
		g_fb_setting[i].crop_bounds.top = 0;
		g_fb_setting[i].crop_bounds.width = fbi->var.xres;
		g_fb_setting[i].crop_bounds.height = fbi->var.yres;
		g_fb_setting[i].disp_fmt = get_ipu_fmt(fbi);

		if (get_ipu_channel(fbi) == MEM_BG_SYNC) {
			bg_crop_bounds[g_fb_setting[i].ipu_id] =
				g_fb_setting[i].crop_bounds;
			g_fb_setting[i].disp_support_csc = true;
		} else if (get_ipu_channel(fbi) == MEM_FG_SYNC) {
			g_fb_setting[i].disp_support_csc = true;
			g_fb_setting[i].disp_support_windows = true;
		}
	}

	for (i = 0; i < num_registered_fb; i++) {
		fbi = registered_fb[i];

		if (get_ipu_channel(fbi) == MEM_FG_SYNC)
			g_fb_setting[i].crop_bounds =
				bg_crop_bounds[g_fb_setting[i].ipu_id];
	}
}

/* called after g_fb_setting filled by update_display_setting */
static int update_setting_from_fbi(struct mxc_vout_output *vout,
			struct fb_info *fbi)
{
	int i;
	bool found = false;

	for (i = 0; i < MAX_FB_NUM; i++) {
		if (g_fb_setting[i].name) {
			if (!strcmp(fbi->fix.id, g_fb_setting[i].name)) {
				vout->crop_bounds = g_fb_setting[i].crop_bounds;
				vout->disp_fmt = g_fb_setting[i].disp_fmt;
				vout->disp_support_csc = g_fb_setting[i].disp_support_csc;
				vout->disp_support_windows =
					g_fb_setting[i].disp_support_windows;
				found = true;
				break;
			}
		}
	}

	if (!found) {
		v4l2_err(vout->vfd->v4l2_dev, "can not find output\n");
		return -EINVAL;
	}
	strlcpy(vout->vfd->name, fbi->fix.id, sizeof(vout->vfd->name));

	memset(&vout->task, 0, sizeof(struct ipu_task));

	vout->task.input.width = DEF_INPUT_WIDTH;
	vout->task.input.height = DEF_INPUT_HEIGHT;
	vout->task.input.crop.pos.x = 0;
	vout->task.input.crop.pos.y = 0;
	vout->task.input.crop.w = DEF_INPUT_WIDTH;
	vout->task.input.crop.h = DEF_INPUT_HEIGHT;

	vout->task.output.width = vout->crop_bounds.width;
	vout->task.output.height = vout->crop_bounds.height;
	vout->task.output.crop.pos.x = 0;
	vout->task.output.crop.pos.y = 0;
	vout->task.output.crop.w = vout->crop_bounds.width;
	vout->task.output.crop.h = vout->crop_bounds.height;
	if (colorspaceofpixel(vout->disp_fmt) == YUV_CS)
		vout->task.output.format = IPU_PIX_FMT_UYVY;
	else
		vout->task.output.format = IPU_PIX_FMT_RGB565;

	return 0;
}

static inline unsigned long get_jiffies(struct timeval *t)
{
	struct timeval cur;

	if (t->tv_usec >= 1000000) {
		t->tv_sec += t->tv_usec / 1000000;
		t->tv_usec = t->tv_usec % 1000000;
	}

	do_gettimeofday(&cur);
	if ((t->tv_sec < cur.tv_sec)
	    || ((t->tv_sec == cur.tv_sec) && (t->tv_usec < cur.tv_usec)))
		return jiffies;

	if (t->tv_usec < cur.tv_usec) {
		cur.tv_sec = t->tv_sec - cur.tv_sec - 1;
		cur.tv_usec = t->tv_usec + 1000000 - cur.tv_usec;
	} else {
		cur.tv_sec = t->tv_sec - cur.tv_sec;
		cur.tv_usec = t->tv_usec - cur.tv_usec;
	}

	return jiffies + timeval_to_jiffies(&cur);
}

static bool deinterlace_3_field(struct mxc_vout_output *vout)
{
	return (vout->task.input.deinterlace.enable &&
		(vout->task.input.deinterlace.motion != HIGH_MOTION));
}

static bool is_pp_bypass(struct mxc_vout_output *vout)
{
	if ((vout->task.input.width == vout->task.output.width) &&
		(vout->task.input.height == vout->task.output.height) &&
		(vout->task.input.crop.w == vout->task.output.crop.w) &&
		(vout->task.input.crop.h == vout->task.output.crop.h) &&
		(vout->task.output.rotate < IPU_ROTATE_HORIZ_FLIP) &&
		!vout->task.input.deinterlace.enable) {
		if (vout->disp_support_csc)
			return true;
		else if (!need_csc(vout->task.input.format, vout->disp_fmt))
			return true;
	/* input crop show to full output which can show based on xres_virtual/yres_virtual */
	} else if ((vout->task.input.crop.w == vout->task.output.crop.w) &&
			(vout->task.output.crop.w == vout->task.output.width) &&
			(vout->task.input.crop.h == vout->task.output.crop.h) &&
			(vout->task.output.crop.h == vout->task.output.height) &&
			(vout->task.output.rotate < IPU_ROTATE_HORIZ_FLIP) &&
			!vout->task.input.deinterlace.enable) {
		if (vout->disp_support_csc)
			return true;
		else if (!need_csc(vout->task.input.format, vout->disp_fmt))
			return true;
	}
	return false;
}

static void setup_buf_timer(struct mxc_vout_output *vout,
			struct videobuf_buffer *vb)
{
	unsigned long timeout;

	/* if timestamp is 0, then default to 30fps */
	if ((vb->ts.tv_sec == 0)
			&& (vb->ts.tv_usec == 0)
			&& vout->start_jiffies)
		timeout =
			vout->start_jiffies + vout->frame_count * HZ / 30;
	else
		timeout = get_jiffies(&vb->ts);

	if (jiffies >= timeout) {
		v4l2_dbg(1, debug, vout->vfd->v4l2_dev,
				"warning: timer timeout already expired.\n");
	}

	if (mod_timer(&vout->timer, timeout)) {
		v4l2_warn(vout->vfd->v4l2_dev,
				"warning: timer was already set\n");
	}

	v4l2_dbg(1, debug, vout->vfd->v4l2_dev,
			"timer handler next schedule: %lu\n", timeout);
}

static int show_buf(struct mxc_vout_output *vout, int idx,
	struct ipu_pos *ipos)
{
	struct fb_info *fbi = vout->fbi;
	struct fb_var_screeninfo var;
	int ret;

	memcpy(&var, &fbi->var, sizeof(var));

	if (vout->bypass_pp) {
		/*
		 * crack fb base
		 * NOTE: should not do other fb operation during v4l2
		 */
		console_lock();
		fbi->fix.smem_start = vout->task.output.paddr;
		fbi->var.yoffset = ipos->y + 1;
		var.xoffset = ipos->x;
		var.yoffset = ipos->y;
		ret = fb_pan_display(fbi, &var);
		console_unlock();
	} else {
		var.yoffset = idx * fbi->var.yres;
		console_lock();
		ret = fb_pan_display(fbi, &var);
		console_unlock();
	}

	return ret;
}

static void disp_work_func(struct work_struct *work)
{
	struct mxc_vout_output *vout =
		container_of(work, struct mxc_vout_output, disp_work);
	struct videobuf_queue *q = &vout->vbq;
	struct videobuf_buffer *vb, *vb_next = NULL;
	unsigned long flags = 0;
	struct ipu_pos ipos;
	int ret = 0;

	v4l2_dbg(1, debug, vout->vfd->v4l2_dev, "disp work begin one frame\n");

	spin_lock_irqsave(q->irqlock, flags);

	if (deinterlace_3_field(vout)) {
		if (list_is_singular(&vout->active_list)) {
			v4l2_warn(vout->vfd->v4l2_dev,
					"deinterlacing: no enough entry in active_list\n");
			spin_unlock_irqrestore(q->irqlock, flags);
			return;
		}
	} else {
		if (list_empty(&vout->active_list)) {
			v4l2_warn(vout->vfd->v4l2_dev,
					"no entry in active_list, should not be here\n");
			spin_unlock_irqrestore(q->irqlock, flags);
			return;
		}
	}
	vb = list_first_entry(&vout->active_list,
			struct videobuf_buffer, queue);

	if (deinterlace_3_field(vout))
		vb_next = list_first_entry(vout->active_list.next,
				struct videobuf_buffer, queue);

	spin_unlock_irqrestore(q->irqlock, flags);

	mutex_lock(&vout->task_lock);

	if (vb->memory == V4L2_MEMORY_USERPTR)
		vout->task.input.paddr = vb->baddr;
	else
		vout->task.input.paddr = videobuf_to_dma_contig(vb);

	if (vout->bypass_pp) {
		vout->task.output.paddr = vout->task.input.paddr;
		ipos.x = vout->task.input.crop.pos.x;
		ipos.y = vout->task.input.crop.pos.y;
	} else {
		if (deinterlace_3_field(vout)) {
			if (vb->memory == V4L2_MEMORY_USERPTR)
				vout->task.input.paddr_n = vb_next->baddr;
			else
				vout->task.input.paddr_n =
					videobuf_to_dma_contig(vb_next);
		}
		vout->task.output.paddr =
			vout->disp_bufs[vout->frame_count % FB_BUFS];
		ret = ipu_queue_task(&vout->task);
		if (ret < 0) {
			mutex_unlock(&vout->task_lock);
			goto err;
		}
	}

	mutex_unlock(&vout->task_lock);

	ret = show_buf(vout, vout->frame_count % FB_BUFS, &ipos);
	if (ret < 0)
		v4l2_dbg(1, debug, vout->vfd->v4l2_dev, "show buf with ret %d\n", ret);

	spin_lock_irqsave(q->irqlock, flags);

	list_del(&vb->queue);

	/*
	 * previous videobuf finish show, set VIDEOBUF_DONE state here
	 * to avoid tearing issue in pp bypass case, which make sure
	 * showing buffer will not be dequeue to write new data. It also
	 * bring side-effect that the last buffer can not be dequeue
	 * correctly, app need take care about it.
	 */
	if (vout->pre_vb) {
		vout->pre_vb->state = VIDEOBUF_DONE;
		wake_up_interruptible(&vout->pre_vb->done);
	}

	if (vout->bypass_pp)
		vout->pre_vb = vb;
	else {
		vout->pre_vb = NULL;
		vb->state = VIDEOBUF_DONE;
		wake_up_interruptible(&vb->done);
	}

	vout->frame_count++;

	/* pick next queue buf to setup timer */
	if (list_empty(&vout->queue_list))
		vout->timer_stop = true;
	else {
		vb = list_first_entry(&vout->queue_list,
				struct videobuf_buffer, queue);
		setup_buf_timer(vout, vb);
	}

	spin_unlock_irqrestore(q->irqlock, flags);

	v4l2_dbg(1, debug, vout->vfd->v4l2_dev, "disp work finish one frame\n");

	return;
err:
	v4l2_err(vout->vfd->v4l2_dev, "display work fail ret = %d\n", ret);
	vout->timer_stop = true;
	vb->state = VIDEOBUF_ERROR;
	return;
}

static void mxc_vout_timer_handler(unsigned long arg)
{
	struct mxc_vout_output *vout =
			(struct mxc_vout_output *) arg;
	struct videobuf_queue *q = &vout->vbq;
	struct videobuf_buffer *vb;
	unsigned long flags = 0;

	spin_lock_irqsave(q->irqlock, flags);

	/*
	 * put first queued entry into active, if previous entry did not
	 * finish, setup current entry's timer again.
	 */
	if (list_empty(&vout->queue_list)) {
		spin_unlock_irqrestore(q->irqlock, flags);
		return;
	}

	/* move videobuf from queued list to active list */
	vb = list_first_entry(&vout->queue_list,
			struct videobuf_buffer, queue);
	list_del(&vb->queue);
	list_add_tail(&vb->queue, &vout->active_list);

	if (queue_work(vout->v4l_wq, &vout->disp_work) == 0) {
		v4l2_warn(vout->vfd->v4l2_dev,
			"disp work was in queue already, queue buf again next time\n");
		list_del(&vb->queue);
		list_add(&vb->queue, &vout->queue_list);
		spin_unlock_irqrestore(q->irqlock, flags);
		return;
	}

	vb->state = VIDEOBUF_ACTIVE;

	spin_unlock_irqrestore(q->irqlock, flags);
}

/* Video buffer call backs */

/*
 * Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers
 */
static int mxc_vout_buffer_setup(struct videobuf_queue *q, unsigned int *count,
			  unsigned int *size)
{
	struct mxc_vout_output *vout = q->priv_data;

	if (!vout)
		return -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

	*size = PAGE_ALIGN(vout->task.input.width * vout->task.input.height *
			fmt_to_bpp(vout->task.input.format)/8);

	return 0;
}

/*
 * This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * converts user space virtual address into physical address if userptr memory
 * exchange mechanism is used.
 */
static int mxc_vout_buffer_prepare(struct videobuf_queue *q,
			    struct videobuf_buffer *vb,
			    enum v4l2_field field)
{
	vb->state = VIDEOBUF_PREPARED;
	return 0;
}

/*
 * Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed.
 * This function is protected by q->irqlock.
 */
static void mxc_vout_buffer_queue(struct videobuf_queue *q,
			  struct videobuf_buffer *vb)
{
	struct mxc_vout_output *vout = q->priv_data;

	list_add_tail(&vb->queue, &vout->queue_list);
	vb->state = VIDEOBUF_QUEUED;

	if (vout->timer_stop) {
		if (deinterlace_3_field(vout) &&
			list_empty(&vout->active_list)) {
			vb = list_first_entry(&vout->queue_list,
					struct videobuf_buffer, queue);
			list_del(&vb->queue);
			list_add_tail(&vb->queue, &vout->active_list);
		} else {
			setup_buf_timer(vout, vb);
			vout->timer_stop = false;
		}
	}
}

/*
 * Buffer release function is called from videobuf layer to release buffer
 * which are already allocated
 */
static void mxc_vout_buffer_release(struct videobuf_queue *q,
			    struct videobuf_buffer *vb)
{
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static int mxc_vout_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	struct mxc_vout_output *vout = file->private_data;

	if (!vout)
		return -ENODEV;

	ret = videobuf_mmap_mapper(&vout->vbq, vma);
	if (ret < 0)
		v4l2_err(vout->vfd->v4l2_dev,
				"offset invalid [offset=0x%lx]\n",
				(vma->vm_pgoff << PAGE_SHIFT));

	return ret;
}

static int mxc_vout_release(struct file *file)
{
	unsigned int ret = 0;
	struct videobuf_queue *q;
	struct mxc_vout_output *vout = file->private_data;

	if (!vout)
		return 0;

	if (--vout->open_cnt == 0) {
		q = &vout->vbq;
		if (q->streaming)
			mxc_vidioc_streamoff(file, vout, vout->type);
		else {
			release_disp_output(vout);
			videobuf_queue_cancel(q);
		}
		destroy_workqueue(vout->v4l_wq);
		ret = videobuf_mmap_free(q);
	}

	return ret;
}

static int mxc_vout_open(struct file *file)
{
	struct mxc_vout_output *vout = NULL;
	int ret = 0;

	vout = video_drvdata(file);

	if (vout == NULL)
		return -ENODEV;

	if (vout->open_cnt++ == 0) {
		vout->ctrl_rotate = 0;
		vout->ctrl_vflip = 0;
		vout->ctrl_hflip = 0;
		update_display_setting();
		ret = update_setting_from_fbi(vout, vout->fbi);
		if (ret < 0)
			goto err;

		vout->v4l_wq = create_singlethread_workqueue("v4l2q");
		if (!vout->v4l_wq) {
			v4l2_err(vout->vfd->v4l2_dev,
					"Could not create work queue\n");
			ret = -ENOMEM;
			goto err;
		}

		INIT_WORK(&vout->disp_work, disp_work_func);

		INIT_LIST_HEAD(&vout->queue_list);
		INIT_LIST_HEAD(&vout->active_list);

		vout->fmt_init = false;
		vout->frame_count = 0;

		vout->win_pos.x = 0;
		vout->win_pos.y = 0;
	}

	file->private_data = vout;

err:
	return ret;
}

/*
 * V4L2 ioctls
 */
static int mxc_vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct mxc_vout_output *vout = fh;

	strlcpy(cap->driver, VOUT_NAME, sizeof(cap->driver));
	strlcpy(cap->card, vout->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;

	return 0;
}

static int mxc_vidioc_enum_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	if (fmt->index >= NUM_MXC_VOUT_FORMATS)
		return -EINVAL;

	strlcpy(fmt->description, mxc_formats[fmt->index].description,
			sizeof(fmt->description));
	fmt->pixelformat = mxc_formats[fmt->index].pixelformat;

	return 0;
}

static int mxc_vidioc_g_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct mxc_vout_output *vout = fh;
	struct v4l2_rect *rect = NULL;

	f->fmt.pix.width = vout->task.input.width;
	f->fmt.pix.height = vout->task.input.height;
	f->fmt.pix.pixelformat = vout->task.input.format;
	f->fmt.pix.sizeimage = vout->task.input.width * vout->task.input.height *
			fmt_to_bpp(vout->task.input.format)/8;

	if (f->fmt.pix.priv) {
		rect = (struct v4l2_rect *)f->fmt.pix.priv;
		rect->left = vout->task.input.crop.pos.x;
		rect->top = vout->task.input.crop.pos.y;
		rect->width = vout->task.input.crop.w;
		rect->height = vout->task.input.crop.h;
	}

	return 0;
}

static inline int ipu_try_task(struct mxc_vout_output *vout)
{
	int ret;
	struct ipu_task *task = &vout->task;

again:
	ret = ipu_check_task(task);
	if (ret != IPU_CHECK_OK) {
		if (ret > IPU_CHECK_ERR_MIN) {
			if (ret == IPU_CHECK_ERR_SPLIT_INPUTW_OVER) {
				task->input.crop.w -= 8;
				goto again;
			}
			if (ret == IPU_CHECK_ERR_SPLIT_INPUTH_OVER) {
				task->input.crop.h -= 8;
				goto again;
			}
			if (ret == IPU_CHECK_ERR_SPLIT_OUTPUTW_OVER) {
				if (vout->disp_support_windows) {
					task->output.width -= 8;
					task->output.crop.w = task->output.width;
				} else
					task->output.crop.w -= 8;
				goto again;
			}
			if (ret == IPU_CHECK_ERR_SPLIT_OUTPUTH_OVER) {
				if (vout->disp_support_windows) {
					task->output.height -= 8;
					task->output.crop.h = task->output.height;
				} else
					task->output.crop.h -= 8;
				goto again;
			}
			ret = -EINVAL;
		}
	} else
		ret = 0;

	return ret;
}

static int mxc_vout_try_task(struct mxc_vout_output *vout)
{
	int ret = 0;

	vout->task.input.crop.w -= vout->task.input.crop.w%8;
	vout->task.input.crop.h -= vout->task.input.crop.h%8;

	/* assume task.output already set by S_CROP */
	if (is_pp_bypass(vout)) {
		v4l2_info(vout->vfd->v4l2_dev, "Bypass IC.\n");
		vout->bypass_pp = true;
		vout->task.output.format = vout->task.input.format;
	} else {
		/* if need CSC, choose IPU-DP or IPU_IC do it */
		vout->bypass_pp = false;
		if (vout->disp_support_csc) {
			if (colorspaceofpixel(vout->task.input.format) == YUV_CS)
				vout->task.output.format = IPU_PIX_FMT_UYVY;
			else
				vout->task.output.format = IPU_PIX_FMT_RGB565;
		} else {
			if (colorspaceofpixel(vout->disp_fmt) == YUV_CS)
				vout->task.output.format = IPU_PIX_FMT_UYVY;
			else
				vout->task.output.format = IPU_PIX_FMT_RGB565;
		}
		ret = ipu_try_task(vout);
	}

	return ret;
}

static int mxc_vout_try_format(struct mxc_vout_output *vout, struct v4l2_format *f)
{
	int ret = 0;
	struct v4l2_rect *rect = NULL;

	vout->task.input.width = f->fmt.pix.width;
	vout->task.input.height = f->fmt.pix.height;
	vout->task.input.format = f->fmt.pix.pixelformat;

	switch (f->fmt.pix.field) {
	/* Images are in progressive format, not interlaced */
	case V4L2_FIELD_NONE:
		break;
	/* The two fields of a frame are passed in separate buffers,
	   in temporal order, i. e. the older one first. */
	case V4L2_FIELD_ALTERNATE:
		v4l2_err(vout->vfd->v4l2_dev,
			"V4L2_FIELD_ALTERNATE field format not supported yet!\n");
		break;
	case V4L2_FIELD_INTERLACED_TB:
		v4l2_info(vout->vfd->v4l2_dev, "Enable deinterlace.\n");
		vout->task.input.deinterlace.enable = true;
		vout->task.input.deinterlace.field_fmt =
				IPU_DEINTERLACE_FIELD_TOP;
		break;
	case V4L2_FIELD_INTERLACED_BT:
		v4l2_info(vout->vfd->v4l2_dev, "Enable deinterlace.\n");
		vout->task.input.deinterlace.enable = true;
		vout->task.input.deinterlace.field_fmt =
				IPU_DEINTERLACE_FIELD_BOTTOM;
		break;
	default:
		break;
	}

	if (f->fmt.pix.priv) {
		rect = (struct v4l2_rect *)f->fmt.pix.priv;
		vout->task.input.crop.pos.x = rect->left;
		vout->task.input.crop.pos.y = rect->top;
		vout->task.input.crop.w = rect->width;
		vout->task.input.crop.h = rect->height;
	} else {
		vout->task.input.crop.pos.x = 0;
		vout->task.input.crop.pos.y = 0;
		vout->task.input.crop.w = f->fmt.pix.width;
		vout->task.input.crop.h = f->fmt.pix.height;
	}

	ret = mxc_vout_try_task(vout);
	if (!ret) {
		if (rect) {
			rect->width = vout->task.input.crop.w;
			rect->height = vout->task.input.crop.h;
		} else {
			f->fmt.pix.width = vout->task.input.crop.w;
			f->fmt.pix.height = vout->task.input.crop.h;
		}
	}

	return ret;
}

static int mxc_vidioc_s_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct mxc_vout_output *vout = fh;
	int ret = 0;

	if (vout->vbq.streaming)
		return -EBUSY;

	mutex_lock(&vout->task_lock);
	ret = mxc_vout_try_format(vout, f);
	if (ret >= 0)
		vout->fmt_init = true;
	mutex_unlock(&vout->task_lock);

	return ret;
}

static int mxc_vidioc_cropcap(struct file *file, void *fh,
		struct v4l2_cropcap *cropcap)
{
	struct mxc_vout_output *vout = fh;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	cropcap->bounds = vout->crop_bounds;
	cropcap->defrect = vout->crop_bounds;

	return 0;
}

static int mxc_vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct mxc_vout_output *vout = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	if (vout->disp_support_windows) {
		crop->c.left = vout->win_pos.x;
		crop->c.top = vout->win_pos.y;
		crop->c.width = vout->task.output.width;
		crop->c.height = vout->task.output.height;
	} else {
		if (vout->task.output.crop.w && vout->task.output.crop.h) {
			crop->c.left = vout->task.output.crop.pos.x;
			crop->c.top = vout->task.output.crop.pos.y;
			crop->c.width = vout->task.output.crop.w;
			crop->c.height = vout->task.output.crop.h;
		} else {
			crop->c.left = 0;
			crop->c.top = 0;
			crop->c.width = vout->task.output.width;
			crop->c.height = vout->task.output.height;
		}
	}

	return 0;
}

static int mxc_vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct mxc_vout_output *vout = fh;
	struct v4l2_rect *b = &vout->crop_bounds;
	int ret = 0;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	if (crop->c.width < 0 || crop->c.height < 0)
		return -EINVAL;

	if (crop->c.width == 0)
		crop->c.width = b->width - b->left;
	if (crop->c.height == 0)
		crop->c.height = b->height - b->top;

	if (crop->c.top < b->top)
		crop->c.top = b->top;
	if (crop->c.top >= b->top + b->height)
		crop->c.top = b->top + b->height - 1;
	if (crop->c.height > b->top - crop->c.top + b->height)
		crop->c.height =
			b->top - crop->c.top + b->height;

	if (crop->c.left < b->left)
		crop->c.left = b->left;
	if (crop->c.left >= b->left + b->width)
		crop->c.left = b->left + b->width - 1;
	if (crop->c.width > b->left - crop->c.left + b->width)
		crop->c.width =
			b->left - crop->c.left + b->width;

	/* stride line limitation */
	crop->c.height -= crop->c.height % 8;
	crop->c.width -= crop->c.width % 8;

	/* the same setting, return */
	if (vout->disp_support_windows) {
		if ((vout->win_pos.x == crop->c.left) &&
			(vout->win_pos.y == crop->c.top) &&
			(vout->task.output.crop.w == crop->c.width) &&
			(vout->task.output.crop.h == crop->c.height))
			return 0;
	} else {
		if ((vout->task.output.crop.pos.x == crop->c.left) &&
			(vout->task.output.crop.pos.y == crop->c.top) &&
			(vout->task.output.crop.w == crop->c.width) &&
			(vout->task.output.crop.h == crop->c.height))
			return 0;
	}

	/* wait current work finish */
	if (vout->vbq.streaming)
		cancel_work_sync(&vout->disp_work);

	mutex_lock(&vout->task_lock);

	if (vout->disp_support_windows) {
		vout->task.output.crop.pos.x = 0;
		vout->task.output.crop.pos.y = 0;
		vout->win_pos.x = crop->c.left;
		vout->win_pos.y = crop->c.top;
		vout->task.output.width = crop->c.width;
		vout->task.output.height = crop->c.height;
	} else {
		vout->task.output.crop.pos.x = crop->c.left;
		vout->task.output.crop.pos.y = crop->c.top;
	}

	vout->task.output.crop.w = crop->c.width;
	vout->task.output.crop.h = crop->c.height;

	/*
	 * must S_CROP before S_FMT, for fist time S_CROP, will not check
	 * ipu task, it will check in S_FMT, after S_FMT, S_CROP should
	 * check ipu task too.
	 */
	if (vout->fmt_init) {
		if (vout->vbq.streaming)
			release_disp_output(vout);

		ret = mxc_vout_try_task(vout);
		if (ret < 0) {
			v4l2_err(vout->vfd->v4l2_dev,
					"vout check task failed\n");
			goto done;
		}
		if (vout->vbq.streaming) {
			ret = config_disp_output(vout);
			if (ret < 0) {
				v4l2_err(vout->vfd->v4l2_dev,
						"Config display output failed\n");
				goto done;
			}
		}
	}

done:
	mutex_unlock(&vout->task_lock);

	return ret;
}

static int mxc_vidioc_queryctrl(struct file *file, void *fh,
		struct v4l2_queryctrl *ctrl)
{
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ret = v4l2_ctrl_query_fill(ctrl, 0, 270, 90, 0);
		break;
	case V4L2_CID_VFLIP:
		ret = v4l2_ctrl_query_fill(ctrl, 0, 1, 1, 0);
		break;
	case V4L2_CID_HFLIP:
		ret = v4l2_ctrl_query_fill(ctrl, 0, 1, 1, 0);
		break;
	case V4L2_CID_MXC_MOTION:
		ret = v4l2_ctrl_query_fill(ctrl, 0, 2, 1, 0);
		break;
	default:
		ctrl->name[0] = '\0';
		ret = -EINVAL;
	}
	return ret;
}

static int mxc_vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct mxc_vout_output *vout = fh;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctrl->value = vout->ctrl_rotate;
		break;
	case V4L2_CID_VFLIP:
		ctrl->value = vout->ctrl_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->value = vout->ctrl_hflip;
		break;
	case V4L2_CID_MXC_MOTION:
		if (vout->task.input.deinterlace.enable)
			ctrl->value = vout->task.input.deinterlace.motion;
		else
			ctrl->value = 0;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void setup_task_rotation(struct mxc_vout_output *vout)
{
	if (vout->ctrl_rotate == 0) {
		if (vout->ctrl_vflip && vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_180;
		else if (vout->ctrl_vflip)
			vout->task.output.rotate = IPU_ROTATE_VERT_FLIP;
		else if (vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_HORIZ_FLIP;
		else
			vout->task.output.rotate = IPU_ROTATE_NONE;
	} else if (vout->ctrl_rotate == 90) {
		if (vout->ctrl_vflip && vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_90_LEFT;
		else if (vout->ctrl_vflip)
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT_VFLIP;
		else if (vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT_HFLIP;
		else
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT;
	} else if (vout->ctrl_rotate == 180) {
		if (vout->ctrl_vflip && vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_NONE;
		else if (vout->ctrl_vflip)
			vout->task.output.rotate = IPU_ROTATE_HORIZ_FLIP;
		else if (vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_VERT_FLIP;
		else
			vout->task.output.rotate = IPU_ROTATE_180;
	} else if (vout->ctrl_rotate == 270) {
		if (vout->ctrl_vflip && vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT;
		else if (vout->ctrl_vflip)
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT_HFLIP;
		else if (vout->ctrl_hflip)
			vout->task.output.rotate = IPU_ROTATE_90_RIGHT_VFLIP;
		else
			vout->task.output.rotate = IPU_ROTATE_90_LEFT;
	}
}

static int mxc_vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct mxc_vout_output *vout = fh;

	/* wait current work finish */
	if (vout->vbq.streaming)
		cancel_work_sync(&vout->disp_work);

	mutex_lock(&vout->task_lock);
	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
	{
		vout->ctrl_rotate = (ctrl->value/90) * 90;
		if (vout->ctrl_rotate > 270)
			vout->ctrl_rotate = 270;
		setup_task_rotation(vout);
		break;
	}
	case V4L2_CID_VFLIP:
	{
		vout->ctrl_vflip = ctrl->value;
		setup_task_rotation(vout);
		break;
	}
	case V4L2_CID_HFLIP:
	{
		vout->ctrl_hflip = ctrl->value;
		setup_task_rotation(vout);
		break;
	}
	case V4L2_CID_MXC_MOTION:
	{
		vout->task.input.deinterlace.motion = ctrl->value;
		break;
	}
	default:
		ret = -EINVAL;
		goto done;
	}

	if (vout->fmt_init) {
		if (vout->vbq.streaming)
			release_disp_output(vout);

		ret = mxc_vout_try_task(vout);
		if (ret < 0) {
			v4l2_err(vout->vfd->v4l2_dev,
					"vout check task failed\n");
			goto done;
		}
		if (vout->vbq.streaming) {
			ret = config_disp_output(vout);
			if (ret < 0) {
				v4l2_err(vout->vfd->v4l2_dev,
						"Config display output failed\n");
				goto done;
			}
		}
	}

done:
	mutex_unlock(&vout->task_lock);

	return ret;
}

static int mxc_vidioc_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	int ret = 0;
	struct mxc_vout_output *vout = fh;
	struct videobuf_queue *q = &vout->vbq;

	if (req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* should not be here after streaming, videobuf_reqbufs will control */
	mutex_lock(&vout->task_lock);

	ret = videobuf_reqbufs(q, req);

	mutex_unlock(&vout->task_lock);
	return ret;
}

static int mxc_vidioc_querybuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	int ret;
	struct mxc_vout_output *vout = fh;

	ret = videobuf_querybuf(&vout->vbq, b);
	if (!ret) {
		/* return physical address */
		struct videobuf_buffer *vb = vout->vbq.bufs[b->index];
		if (b->flags & V4L2_BUF_FLAG_MAPPED)
			b->m.offset = videobuf_to_dma_contig(vb);
	}

	return ret;
}

static int mxc_vidioc_qbuf(struct file *file, void *fh,
			struct v4l2_buffer *buffer)
{
	struct mxc_vout_output *vout = fh;

	return videobuf_qbuf(&vout->vbq, buffer);
}

static int mxc_vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct mxc_vout_output *vout = fh;

	if (!vout->vbq.streaming)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
		return videobuf_dqbuf(&vout->vbq, (struct v4l2_buffer *)b, 1);
	else
		return videobuf_dqbuf(&vout->vbq, (struct v4l2_buffer *)b, 0);
}

static int set_window_position(struct mxc_vout_output *vout, struct mxcfb_pos *pos)
{
	struct fb_info *fbi = vout->fbi;
	mm_segment_t old_fs;
	int ret;

	if (vout->disp_support_windows) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = fbi->fbops->fb_ioctl(fbi, MXCFB_SET_OVERLAY_POS,
				(unsigned long)pos);
		set_fs(old_fs);
	}

	return ret;
}

static int config_disp_output(struct mxc_vout_output *vout)
{
	struct fb_info *fbi = vout->fbi;
	struct fb_var_screeninfo var;
	int i, display_buf_size, fb_num, ret;

	memcpy(&var, &fbi->var, sizeof(var));

	var.xres = vout->task.output.width;
	var.yres = vout->task.output.height;
	if (vout->bypass_pp) {
		fb_num = 1;
		/* input crop */
		if (vout->task.input.width > vout->task.output.width)
			var.xres_virtual = vout->task.input.width;
		else
			var.xres_virtual = var.xres;
		if (vout->task.input.height > vout->task.output.height)
			var.yres_virtual = vout->task.input.height;
		else
			var.yres_virtual = var.yres;
		var.rotate = vout->task.output.rotate;
	} else {
		fb_num = FB_BUFS;
		var.xres_virtual = var.xres;
		var.yres_virtual = fb_num * var.yres;
	}
	var.bits_per_pixel = fmt_to_bpp(vout->task.output.format);
	var.nonstd = vout->task.output.format;

	v4l2_dbg(1, debug, vout->vfd->v4l2_dev,
			"set display fb to %d %d\n",
			var.xres, var.yres);

	ret = set_window_position(vout, &vout->win_pos);
	if (ret < 0)
		return ret;

	/* Init display channel through fb API */
	var.yoffset = 0;
	var.activate |= FB_ACTIVATE_FORCE;
	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &var);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();
	if (ret < 0)
		return ret;

	display_buf_size = fbi->fix.line_length * fbi->var.yres;
	for (i = 0; i < fb_num; i++)
		vout->disp_bufs[i] = fbi->fix.smem_start + i * display_buf_size;

	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_blank(fbi, FB_BLANK_UNBLANK);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();

	return ret;
}

static void release_disp_output(struct mxc_vout_output *vout)
{
	struct fb_info *fbi = vout->fbi;
	struct mxcfb_pos pos;

	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	fb_blank(fbi, FB_BLANK_POWERDOWN);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();

	/* restore pos to 0,0 avoid fb pan display hang? */
	pos.x = 0;
	pos.y = 0;
	set_window_position(vout, &pos);

	/* fix if ic bypass crack smem_start */
	if (vout->bypass_pp) {
		console_lock();
		fbi->fix.smem_start = vout->disp_bufs[0];
		console_unlock();
	}

	if (get_ipu_channel(fbi) == MEM_BG_SYNC) {
		console_lock();
		fbi->flags |= FBINFO_MISC_USEREVENT;
		fb_blank(fbi, FB_BLANK_UNBLANK);
		fbi->flags &= ~FBINFO_MISC_USEREVENT;
		console_unlock();
	}
}

static int mxc_vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct mxc_vout_output *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int ret;

	if (q->streaming) {
		v4l2_err(vout->vfd->v4l2_dev,
				"video output already run\n");
		ret = -EBUSY;
		goto done;
	}

	if (deinterlace_3_field(vout) && list_is_singular(&q->stream)) {
		v4l2_err(vout->vfd->v4l2_dev,
				"deinterlacing: need queue 2 frame before streamon\n");
		ret = -EINVAL;
		goto done;
	}

	ret = config_disp_output(vout);
	if (ret < 0) {
		v4l2_err(vout->vfd->v4l2_dev,
				"Config display output failed\n");
		goto done;
	}

	init_timer(&vout->timer);
	vout->timer.function = mxc_vout_timer_handler;
	vout->timer.data = (unsigned long)vout;
	vout->timer_stop = true;

	vout->start_jiffies = jiffies;

	vout->pre_vb = NULL;

	ret = videobuf_streamon(q);
done:
	return ret;
}

static int mxc_vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct mxc_vout_output *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int ret = 0;

	if (q->streaming) {
		cancel_work_sync(&vout->disp_work);
		flush_workqueue(vout->v4l_wq);

		del_timer_sync(&vout->timer);

		release_disp_output(vout);

		ret = videobuf_streamoff(&vout->vbq);
	}

	return ret;
}

static const struct v4l2_ioctl_ops mxc_vout_ioctl_ops = {
	.vidioc_querycap      			= mxc_vidioc_querycap,
	.vidioc_enum_fmt_vid_out 		= mxc_vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out			= mxc_vidioc_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out			= mxc_vidioc_s_fmt_vid_out,
	.vidioc_cropcap				= mxc_vidioc_cropcap,
	.vidioc_g_crop				= mxc_vidioc_g_crop,
	.vidioc_s_crop				= mxc_vidioc_s_crop,
	.vidioc_queryctrl    			= mxc_vidioc_queryctrl,
	.vidioc_g_ctrl       			= mxc_vidioc_g_ctrl,
	.vidioc_s_ctrl       			= mxc_vidioc_s_ctrl,
	.vidioc_reqbufs				= mxc_vidioc_reqbufs,
	.vidioc_querybuf			= mxc_vidioc_querybuf,
	.vidioc_qbuf				= mxc_vidioc_qbuf,
	.vidioc_dqbuf				= mxc_vidioc_dqbuf,
	.vidioc_streamon			= mxc_vidioc_streamon,
	.vidioc_streamoff			= mxc_vidioc_streamoff,
};

static const struct v4l2_file_operations mxc_vout_fops = {
	.owner 		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.mmap 		= mxc_vout_mmap,
	.open 		= mxc_vout_open,
	.release 	= mxc_vout_release,
};

static struct video_device mxc_vout_template = {
	.name 		= "MXC Video Output",
	.fops           = &mxc_vout_fops,
	.ioctl_ops 	= &mxc_vout_ioctl_ops,
	.release	= video_device_release,
};

static struct videobuf_queue_ops mxc_vout_vbq_ops = {
	.buf_setup = mxc_vout_buffer_setup,
	.buf_prepare = mxc_vout_buffer_prepare,
	.buf_release = mxc_vout_buffer_release,
	.buf_queue = mxc_vout_buffer_queue,
};

static void mxc_vout_free_output(struct mxc_vout_dev *dev)
{
	int i;
	struct mxc_vout_output *vout;
	struct video_device *vfd;

	for (i = 0; i < dev->out_num; i++) {
		vout = dev->out[i];
		vfd = vout->vfd;
		if (vfd) {
			if (!video_is_registered(vfd))
				video_device_release(vfd);
			else
				video_unregister_device(vfd);
		}
		kfree(vout);
	}
}

static int __init mxc_vout_setup_output(struct mxc_vout_dev *dev)
{
	struct videobuf_queue *q;
	struct fb_info *fbi;
	struct mxc_vout_output *vout;
	int i, ret = 0;

	update_display_setting();

	/* all output/overlay based on fb */
	for (i = 0; i < num_registered_fb; i++) {
		fbi = registered_fb[i];

		vout = kzalloc(sizeof(struct mxc_vout_output), GFP_KERNEL);
		if (!vout) {
			ret = -ENOMEM;
			break;
		}

		dev->out[dev->out_num] = vout;
		dev->out_num++;

		vout->fbi = fbi;
		vout->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vout->vfd = video_device_alloc();
		if (!vout->vfd) {
			ret = -ENOMEM;
			break;
		}

		*vout->vfd = mxc_vout_template;
		vout->vfd->debug = debug;
		vout->vfd->v4l2_dev = &dev->v4l2_dev;
		vout->vfd->lock = &vout->mutex;

		mutex_init(&vout->mutex);
		mutex_init(&vout->task_lock);

		strlcpy(vout->vfd->name, fbi->fix.id, sizeof(vout->vfd->name));

		video_set_drvdata(vout->vfd, vout);

		if (video_register_device(vout->vfd,
			VFL_TYPE_GRABBER, video_nr + i) < 0) {
			ret = -ENODEV;
			break;
		}

		q = &vout->vbq;
		q->dev = dev->dev;
		spin_lock_init(&vout->vbq_lock);
		videobuf_queue_dma_contig_init(q, &mxc_vout_vbq_ops, q->dev,
				&vout->vbq_lock, vout->type, V4L2_FIELD_NONE,
				sizeof(struct videobuf_buffer), vout, NULL);

		v4l2_info(vout->vfd->v4l2_dev, "V4L2 device registered as %s\n",
				video_device_node_name(vout->vfd));

	}

	return ret;
}

static int mxc_vout_probe(struct platform_device *pdev)
{
	int ret;
	struct mxc_vout_dev *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	dev->dev->dma_mask = kmalloc(sizeof(*dev->dev->dma_mask), GFP_KERNEL);
	*dev->dev->dma_mask = DMA_BIT_MASK(32);
	dev->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	ret = v4l2_device_register(dev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(dev->dev, "v4l2_device_register failed\n");
		goto free_dev;
	}

	ret = mxc_vout_setup_output(dev);
	if (ret < 0)
		goto rel_vdev;

	return 0;

rel_vdev:
	mxc_vout_free_output(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
	kfree(dev);
	return ret;
}

static int mxc_vout_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct mxc_vout_dev *dev = container_of(v4l2_dev, struct
			mxc_vout_dev, v4l2_dev);

	mxc_vout_free_output(dev);
	v4l2_device_unregister(v4l2_dev);
	kfree(dev);
	return 0;
}

static struct platform_driver mxc_vout_driver = {
	.driver = {
		.name = "mxc_v4l2_output",
	},
	.probe = mxc_vout_probe,
	.remove = mxc_vout_remove,
};

static int __init mxc_vout_init(void)
{
	if (platform_driver_register(&mxc_vout_driver) != 0) {
		printk(KERN_ERR VOUT_NAME ":Could not register Video driver\n");
		return -EINVAL;
	}
	return 0;
}

static void mxc_vout_cleanup(void)
{
	platform_driver_unregister(&mxc_vout_driver);
}

module_init(mxc_vout_init);
module_exit(mxc_vout_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("V4L2-driver for MXC video output");
MODULE_LICENSE("GPL");
