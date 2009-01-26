/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_v4l2_output.c
 *
 * @brief MX27 V4L2 Video Output Driver
 *
 * Video4Linux2 Output Device using MX27 eMMA Post-processing functionality.
 *
 * @ingroup MXC_V4L2_OUTPUT
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include "mxc_v4l2_output.h"
#include "mx27_pp.h"
#include "../drivers/video/mxc/mx2fb.h"

#define SDC_FG_FB_FORMAT	V4L2_PIX_FMT_RGB565

struct v4l2_output mxc_outputs[1] = {
	{
	 .index = 0,
	 .name = "DISP0 Video Out",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,	/* not really correct,
						   but no other choice */
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN},
};

static int video_nr = 16;
static spinlock_t g_lock = SPIN_LOCK_UNLOCKED;
vout_data *g_vout;

/* debug counters */
uint32_t g_irq_cnt;
uint32_t g_buf_output_cnt;
uint32_t g_buf_q_cnt;
uint32_t g_buf_dq_cnt;

#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
static uint32_t g_output_fb = -1;
static uint32_t g_fb_enabled = 0;
static uint32_t g_pp_ready = 0;

static int fb_event_notify(struct notifier_block *self,
			   unsigned long action, void *data)
{
	struct fb_event *event = data;
	struct fb_info *info = event->info;
	unsigned long lock_flags;
	int blank, i;

	for (i = 0; i < num_registered_fb; i++)
		if (registered_fb[i] == info)
			break;

	/*
	 * Check if the event is sent by the framebuffer in which
	 * the video is displayed.
	 */
	if (i != g_output_fb)
		return 0;

	switch (action) {
	case FB_EVENT_BLANK:
		blank = *(int *)event->data;
		spin_lock_irqsave(&g_lock, lock_flags);
		g_fb_enabled = !blank;
		if (blank && g_pp_ready) {
			if (pp_enable(1))
				pr_debug("unable to enable PP\n");
			g_pp_ready = 0;
		}
		spin_unlock_irqrestore(&g_lock, lock_flags);
		break;
	case FB_EVENT_MXC_EOF:
		spin_lock_irqsave(&g_lock, lock_flags);
		g_fb_enabled = 1;
		if (g_pp_ready) {
			if (pp_enable(1))
				pr_debug("unable to enable PP\n");
			g_pp_ready = 0;
		}
		spin_unlock_irqrestore(&g_lock, lock_flags);
		break;
	}

	return 0;
}

static struct notifier_block fb_event_notifier = {
	.notifier_call = fb_event_notify,
};

static struct notifier_block mx2fb_event_notifier = {
	.notifier_call = fb_event_notify,
};
#endif

#define QUEUE_SIZE (MAX_FRAME_NUM + 1)
static __inline int queue_size(v4l_queue * q)
{
	if (q->tail >= q->head)
		return (q->tail - q->head);
	else
		return ((q->tail + QUEUE_SIZE) - q->head);
}

static __inline int queue_buf(v4l_queue * q, int idx)
{
	if (((q->tail + 1) % QUEUE_SIZE) == q->head)
		return -1;	/* queue full */
	q->list[q->tail] = idx;
	q->tail = (q->tail + 1) % QUEUE_SIZE;
	return 0;
}

static __inline int dequeue_buf(v4l_queue * q)
{
	int ret;
	if (q->tail == q->head)
		return -1;	/* queue empty */
	ret = q->list[q->head];
	q->head = (q->head + 1) % QUEUE_SIZE;
	return ret;
}

static __inline int peek_next_buf(v4l_queue * q)
{
	if (q->tail == q->head)
		return -1;	/* queue empty */
	return q->list[q->head];
}

static __inline unsigned long get_jiffies(struct timeval *t)
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

/*!
 * Private function to free buffers
 *
 * @param bufs_paddr	Array of physical address of buffers to be freed
 *
 * @param bufs_vaddr	Array of virtual address of buffers to be freed
 *
 * @param num_buf	Number of buffers to be freed
 *
 * @param size		Size for each buffer to be free
 *
 * @return status  0 success.
 */
static int mxc_free_buffers(dma_addr_t bufs_paddr[], void *bufs_vaddr[],
			    int num_buf, int size)
{
	int i;

	for (i = 0; i < num_buf; i++) {
		if (bufs_vaddr[i] != 0) {
			dma_free_coherent(0, size, bufs_vaddr[i],
					  bufs_paddr[i]);
			pr_debug("freed @ paddr=0x%08X\n", (u32) bufs_paddr[i]);
			bufs_paddr[i] = 0;
			bufs_vaddr[i] = NULL;
		}
	}
	return 0;
}

/*!
 * Private function to allocate buffers
 *
 * @param bufs_paddr	Output array of physical address of buffers allocated
 *
 * @param bufs_vaddr	Output array of virtual address of buffers allocated
 *
 * @param num_buf	Input number of buffers to allocate
 *
 * @param size		Input size for each buffer to allocate
 *
 * @return status	-0 Successfully allocated a buffer, -ENOBUFS failed.
 */
static int mxc_allocate_buffers(dma_addr_t bufs_paddr[], void *bufs_vaddr[],
				int num_buf, int size)
{
	int i;

	for (i = 0; i < num_buf; i++) {
		bufs_vaddr[i] = dma_alloc_coherent(0, size,
						   &bufs_paddr[i],
						   GFP_DMA | GFP_KERNEL);

		if (bufs_vaddr[i] == 0) {
			mxc_free_buffers(bufs_paddr, bufs_vaddr, i, size);
			pr_debug("dma_alloc_coherent failed.\n");
			return -ENOBUFS;
		}
		pr_debug("allocated @ paddr=0x%08X, size=%d.\n",
			 (u32) bufs_paddr[i], size);
	}

	return 0;
}

static void mxc_v4l2out_timer_handler(unsigned long arg)
{
	int index;
	unsigned long timeout;
	unsigned long lock_flags;
	vout_data *vout = (vout_data *) arg;

	pr_debug("timer handler: %lu\n", jiffies);

	spin_lock_irqsave(&g_lock, lock_flags);

	if ((vout->state == STATE_STREAM_OFF)
	    || (vout->state == STATE_STREAM_STOPPING)) {
		pr_debug("stream has stopped\n");
		goto exit0;
	}

	/*
	 * If timer occurs before PP h/w is ready, then set the state to
	 * paused and the timer will be set again when next buffer is queued
	 * or PP completes.
	 */
	if (vout->ipu_buf[0] != -1) {
		pr_debug("buffer is busy\n");
		vout->state = STATE_STREAM_PAUSED;
		goto exit0;
	}

	/* Dequeue buffer and pass to PP */
	index = dequeue_buf(&vout->ready_q);
	if (index == -1) {	/* no buffers ready, should never occur */
		pr_debug("mxc_v4l2out: timer - no queued buffers ready\n");
		goto exit0;
	}

	g_buf_dq_cnt++;
	vout->frame_count++;
	vout->ipu_buf[0] = index;

	if (pp_ptr((unsigned int)vout->queue_buf_paddr[index])) {
		pr_debug("unable to update buffer\n");
		goto exit0;
	}
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	if (g_fb_enabled && (vout->v4l2_fb.flags != V4L2_FBUF_FLAG_OVERLAY))
		g_pp_ready = 1;
	else if (pp_enable(1)) {
		pr_debug("unable to enable PP\n");
		goto exit0;
	}
#else
	if (pp_enable(1)) {
		pr_debug("unable to enable PP\n");
		goto exit0;
	}
#endif
	pr_debug("enabled index %d\n", index);

	/* Setup timer for next buffer */
	index = peek_next_buf(&vout->ready_q);
	pr_debug("next index %d\n", index);
	if (index != -1) {
		/* if timestamp is 0, then default to 30fps */
		if ((vout->v4l2_bufs[index].timestamp.tv_sec == 0)
		    && (vout->v4l2_bufs[index].timestamp.tv_usec == 0))
			timeout =
			    vout->start_jiffies + vout->frame_count * HZ / 30;
		else
			timeout =
			    get_jiffies(&vout->v4l2_bufs[index].timestamp);

		if (jiffies >= timeout) {
			pr_debug("warning: timer timeout already expired.\n");
		}

		if (mod_timer(&vout->output_timer, timeout))
			pr_debug("warning: timer was already set\n");

		pr_debug("timer handler next schedule: %lu\n", timeout);
	} else {
		vout->state = STATE_STREAM_PAUSED;
		pr_debug("timer handler paused\n");
	}

      exit0:
	spin_unlock_irqrestore(&g_lock, lock_flags);
}

irqreturn_t mxc_v4l2out_pp_in_irq_handler(int irq, void *dev_id)
{
	int last_buf;
	int index;
	unsigned long timeout;
	unsigned long lock_flags;
	vout_data *vout = dev_id;

	spin_lock_irqsave(&g_lock, lock_flags);

	g_irq_cnt++;

	if ((vout->state == STATE_STREAM_OFF)
	    || (vout->state == STATE_STREAM_STOPPING)) {
		spin_unlock_irqrestore(&g_lock, lock_flags);
		return IRQ_HANDLED;
	}

	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		gwinfo.enabled = 1;
		gwinfo.alpha_value = 255;
		gwinfo.ck_enabled = 0;
		gwinfo.xpos = vout->crop_current.left;
		gwinfo.ypos = vout->crop_current.top;
		gwinfo.base = (unsigned long)vout->display_bufs[pp_num_last()];
		gwinfo.xres = vout->crop_current.width;
		gwinfo.yres = vout->crop_current.height;
		gwinfo.xres_virtual = vout->crop_current.width;
		gwinfo.vs_reversed = 0;

		mx2_gw_set(&gwinfo);
	}

	/* Process previous buffer */
	last_buf = vout->ipu_buf[0];
	pr_debug("last_buf %d g_irq_cnt %d\n", last_buf, g_irq_cnt);
	if (last_buf != -1) {
		g_buf_output_cnt++;
		vout->v4l2_bufs[last_buf].flags = V4L2_BUF_FLAG_DONE;
		queue_buf(&vout->done_q, last_buf);
		vout->ipu_buf[0] = -1;
		wake_up_interruptible(&vout->v4l_bufq);
	}

	/* Setup timer for next buffer, when stream has been paused */
	if ((vout->state == STATE_STREAM_PAUSED)
	    && ((index = peek_next_buf(&vout->ready_q)) != -1)) {
		pr_debug("next index %d\n", index);
		/* if timestamp is 0, then default to 30fps */
		if ((vout->v4l2_bufs[index].timestamp.tv_sec == 0)
		    && (vout->v4l2_bufs[index].timestamp.tv_usec == 0))
			timeout =
			    vout->start_jiffies + vout->frame_count * HZ / 30;
		else
			timeout =
			    get_jiffies(&vout->v4l2_bufs[index].timestamp);

		if (jiffies >= timeout) {
			pr_debug("warning: timer timeout already expired.\n");
		}

		vout->state = STATE_STREAM_ON;

		if (mod_timer(&vout->output_timer, timeout))
			pr_debug("warning: timer was already set\n");

		pr_debug("timer handler next schedule: %lu\n", timeout);
	}

	spin_unlock_irqrestore(&g_lock, lock_flags);

	return IRQ_HANDLED;
}

/*!
 * Start the output stream
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int mxc_v4l2out_streamon(vout_data * vout)
{
	unsigned long timeout;
	int index;

	if (!vout)
		return -EINVAL;

	if (vout->state != STATE_STREAM_OFF)
		return -EBUSY;

	if (queue_size(&vout->ready_q) < 1) {
		pr_debug("no buffers queued yet!\n");
		return -EINVAL;
	}

	vout->ipu_buf[0] = -1;

	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		/* Free previously allocated buffer */
		mxc_free_buffers(vout->display_bufs, vout->display_bufs_vaddr,
				 2, vout->display_buf_size);
		/* Allocate buffers for foreground */
		if (mxc_allocate_buffers(vout->display_bufs,
					 vout->display_bufs_vaddr, 2,
					 vout->display_buf_size) < 0) {
			pr_debug("unable to allocate SDC FG buffers\n");
			return -ENOMEM;
		}
	}

	/* Configure PP */
	if (pp_cfg(vout)) {
		/* Free previously allocated buffer */
		mxc_free_buffers(vout->display_bufs, vout->display_bufs_vaddr,
				 2, vout->display_buf_size);
		pr_debug("failed to config PP.\n");
		return -EINVAL;
	}
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	g_output_fb = vout->output_fb_num[vout->cur_disp_output];
	g_fb_enabled = 0;
	g_pp_ready = 0;
	fb_register_client(&fb_event_notifier);
	mx2fb_register_client(&mx2fb_event_notifier);
#endif
	vout->frame_count = 0;
	vout->state = STATE_STREAM_ON;
	index = peek_next_buf(&vout->ready_q);

	/* if timestamp is 0, then default to 30fps */
	if ((vout->v4l2_bufs[index].timestamp.tv_sec == 0)
	    && (vout->v4l2_bufs[index].timestamp.tv_usec == 0))
		timeout = jiffies;
	else
		timeout = get_jiffies(&vout->v4l2_bufs[index].timestamp);

	if (jiffies >= timeout) {
		pr_debug("warning: timer timeout already expired.\n");
	}

	vout->start_jiffies = vout->output_timer.expires = timeout;
	pr_debug("STREAMON:Add timer %d timeout @ %lu jiffies, current = %lu\n",
		 index, timeout, jiffies);
	add_timer(&vout->output_timer);

	return 0;
}

/*!
 * Shut down the voutera
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int mxc_v4l2out_streamoff(vout_data * vout)
{
	int i, retval = 0;
	unsigned long lock_flag = 0;

	if (!vout)
		return -EINVAL;

	if (vout->state == STATE_STREAM_OFF) {
		return 0;
	}

	spin_lock_irqsave(&g_lock, lock_flag);

	del_timer(&vout->output_timer);
	pp_enable(0);		/* Disable PP */

	if (vout->state == STATE_STREAM_ON) {
		vout->state = STATE_STREAM_STOPPING;
	}

	spin_unlock_irqrestore(&g_lock, lock_flag);

	vout->ready_q.head = vout->ready_q.tail = 0;
	vout->done_q.head = vout->done_q.tail = 0;
	for (i = 0; i < vout->buffer_cnt; i++) {
		vout->v4l2_bufs[i].flags = 0;
		vout->v4l2_bufs[i].timestamp.tv_sec = 0;
		vout->v4l2_bufs[i].timestamp.tv_usec = 0;
	}

	vout->state = STATE_STREAM_OFF;

	if (vout->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		/* Disable graphic window */
		gwinfo.enabled = 0;
		mx2_gw_set(&gwinfo);
	}
#ifdef CONFIG_VIDEO_MXC_OUTPUT_FBSYNC
	g_output_fb = -1;
	g_fb_enabled = 0;
	g_pp_ready = 0;
	fb_unregister_client(&fb_event_notifier);
	mx2fb_unregister_client(&mx2fb_event_notifier);
#endif

	mxc_free_buffers(vout->display_bufs, vout->display_bufs_vaddr,
			 2, vout->display_buf_size);

	return retval;
}

/*
 * Valid whether the palette is supported
 *
 * @param palette  V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_BGR24 or V4L2_PIX_FMT_BGR32
 *
 * @return 1 if supported, 0 if failed
 */
static inline int valid_mode(u32 palette)
{
	return (palette == V4L2_PIX_FMT_YUV420);
}

/*
 * Returns bits per pixel for given pixel format
 *
 * @param pixelformat  V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_BGR24 or V4L2_PIX_FMT_BGR32
 *
 * @return bits per pixel of pixelformat
 */
static u32 fmt_to_bpp(u32 pixelformat)
{
	u32 bpp;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		bpp = 16;
		break;
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB24:
		bpp = 24;
		break;
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		bpp = 32;
		break;
	default:
		bpp = 8;
		break;
	}
	return bpp;
}

/*
 * V4L2 - Handles VIDIOC_G_FMT Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param v4l2_format structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2out_g_fmt(vout_data * vout, struct v4l2_format *f)
{
	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		return -EINVAL;
	}
	*f = vout->v2f;
	return 0;
}

/*
 * V4L2 - Handles VIDIOC_S_FMT Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param v4l2_format structure v4l2_format *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_v4l2out_s_fmt(vout_data * vout, struct v4l2_format *f)
{
	int retval = 0;
	u32 size = 0;
	u32 bytesperline;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		retval = -EINVAL;
		goto err0;
	}
	if (!valid_mode(f->fmt.pix.pixelformat)) {
		pr_debug("pixel format not supported\n");
		retval = -EINVAL;
		goto err0;
	}

	bytesperline = (f->fmt.pix.width * fmt_to_bpp(f->fmt.pix.pixelformat)) /
	    8;
	if (f->fmt.pix.bytesperline < bytesperline) {
		f->fmt.pix.bytesperline = bytesperline;
	} else {
		bytesperline = f->fmt.pix.bytesperline;
	}

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		/* byteperline for YUV planar formats is for
		   Y plane only */
		size = bytesperline * f->fmt.pix.height * 2;
		break;
	case V4L2_PIX_FMT_YUV420:
		size = (bytesperline * f->fmt.pix.height * 3) / 2;
		break;
	default:
		size = bytesperline * f->fmt.pix.height;
		break;
	}

	/* Return the actual size of the image to the app */
	f->fmt.pix.sizeimage = size;

	vout->v2f.fmt.pix.sizeimage = size;
	vout->v2f.fmt.pix.width = f->fmt.pix.width;
	vout->v2f.fmt.pix.height = f->fmt.pix.height;
	vout->v2f.fmt.pix.pixelformat = f->fmt.pix.pixelformat;
	vout->v2f.fmt.pix.bytesperline = f->fmt.pix.bytesperline;

	retval = 0;
      err0:
	return retval;
}

/*
 * V4L2 - Handles VIDIOC_G_CTRL Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param c           structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_get_v42lout_control(vout_data * vout, struct v4l2_control *c)
{
	switch (c->id) {
	case V4L2_CID_HFLIP:
		return (vout->rotate & IPU_ROTATE_HORIZ_FLIP) ? 1 : 0;
	case V4L2_CID_VFLIP:
		return (vout->rotate & IPU_ROTATE_VERT_FLIP) ? 1 : 0;
	case (V4L2_CID_PRIVATE_BASE + 1):
		return vout->rotate;
	default:
		return -EINVAL;
	}
}

/*
 * V4L2 - Handles VIDIOC_S_CTRL Ioctl
 *
 * @param vout         structure vout_data *
 *
 * @param c           structure v4l2_control *
 *
 * @return  status    0 success, EINVAL failed
 */
static int mxc_set_v42lout_control(vout_data * vout, struct v4l2_control *c)
{
	switch (c->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
	case V4L2_CID_MXC_ROT:
		return 0;
	default:
		return -EINVAL;
	}
	return 0;
}

/*!
 * V4L2 interface - open function
 *
 * @param inode        structure inode *
 *
 * @param file         structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *                    ENODEV timeout, ERESTARTSYS interrupted by user
 */
static int mxc_v4l2out_open(struct inode *inode, struct file *file)
{
	struct video_device *dev = video_devdata(file);
	vout_data *vout = video_get_drvdata(dev);
	int err;

	if (!vout) {
		pr_info("Internal error, vout_data not found!\n");
		return -ENODEV;
	}

	down(&vout->busy_lock);

	err = -EINTR;
	if (signal_pending(current))
		goto oops;

	if (vout->open_count++ == 0) {
		pp_init(vout);

		init_waitqueue_head(&vout->v4l_bufq);

		init_timer(&vout->output_timer);
		vout->output_timer.function = mxc_v4l2out_timer_handler;
		vout->output_timer.data = (unsigned long)vout;

		vout->state = STATE_STREAM_OFF;
		g_irq_cnt = g_buf_output_cnt = g_buf_q_cnt = g_buf_dq_cnt = 0;

	}

	file->private_data = dev;
	up(&vout->busy_lock);
	return 0;

      oops:
	up(&vout->busy_lock);
	return err;
}

/*!
 * V4L2 interface - close function
 *
 * @param inode    struct inode *
 *
 * @param file     struct file *
 *
 * @return         0 success
 */
static int mxc_v4l2out_close(struct inode *inode, struct file *file)
{
	struct video_device *dev = file->private_data;
	vout_data *vout = video_get_drvdata(dev);

	if (--vout->open_count == 0) {
		pr_debug("release resource\n");

		pp_exit(vout);
		if (vout->state != STATE_STREAM_OFF)
			mxc_v4l2out_streamoff(vout);

		file->private_data = NULL;

		mxc_free_buffers(vout->queue_buf_paddr,
				 vout->queue_buf_vaddr,
				 vout->buffer_cnt, vout->queue_buf_size);
		vout->buffer_cnt = 0;
		mxc_free_buffers(vout->display_bufs,
				 vout->display_bufs_vaddr,
				 2, vout->display_buf_size);

		/* capture off */
		wake_up_interruptible(&vout->v4l_bufq);
	}

	return 0;
}

/*!
 * V4L2 interface - ioctl function
 *
 * @param inode      struct inode *
 *
 * @param file       struct file *
 *
 * @param ioctlnr    unsigned int
 *
 * @param arg        void *
 *
 * @return           0 success, ENODEV for invalid device instance,
 *                   -1 for other errors.
 */
static int
mxc_v4l2out_do_ioctl(struct inode *inode, struct file *file,
		     unsigned int ioctlnr, void *arg)
{
	struct video_device *dev = file->private_data;
	vout_data *vout = video_get_drvdata(dev);
	int retval = 0;
	int i = 0;

	if (!vout)
		return -EBADF;

	/* make this _really_ smp-safe */
	if (down_interruptible(&vout->busy_lock))
		return -EBUSY;

	switch (ioctlnr) {
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap = arg;
			strcpy(cap->driver, "mxc_v4l2_output");
			cap->version = 0;
			cap->capabilities =
			    V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
			cap->card[0] = '\0';
			cap->bus_info[0] = '\0';
			retval = 0;
			break;
		}
	case VIDIOC_G_FMT:
		{
			struct v4l2_format *gf = arg;
			retval = mxc_v4l2out_g_fmt(vout, gf);
			break;
		}
	case VIDIOC_S_FMT:
		{
			struct v4l2_format *sf = arg;
			if (vout->state != STATE_STREAM_OFF) {
				retval = -EBUSY;
				break;
			}
			retval = mxc_v4l2out_s_fmt(vout, sf);
			break;
		}
	case VIDIOC_REQBUFS:
		{
			struct v4l2_requestbuffers *req = arg;
			if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
			    (req->memory != V4L2_MEMORY_MMAP)) {
				pr_debug
				    ("VIDIOC_REQBUFS: incorrect buffer type\n");
				retval = -EINVAL;
				break;
			}

			if (req->count == 0)
				mxc_v4l2out_streamoff(vout);

			if (vout->state == STATE_STREAM_OFF) {
				if (vout->queue_buf_paddr[0] != 0) {
					mxc_free_buffers(vout->queue_buf_paddr,
							 vout->queue_buf_vaddr,
							 vout->buffer_cnt,
							 vout->queue_buf_size);
					pr_debug
					    ("VIDIOC_REQBUFS: freed buffers\n");
				}
				vout->buffer_cnt = 0;
			} else {
				pr_debug("VIDIOC_REQBUFS: Buffer is in use\n");
				retval = -EBUSY;
				break;
			}

			if (req->count == 0)
				break;

			if (req->count < MIN_FRAME_NUM) {
				req->count = MIN_FRAME_NUM;
			} else if (req->count > MAX_FRAME_NUM) {
				req->count = MAX_FRAME_NUM;
			}
			vout->buffer_cnt = req->count;
			vout->queue_buf_size =
			    PAGE_ALIGN(vout->v2f.fmt.pix.sizeimage);

			retval = mxc_allocate_buffers(vout->queue_buf_paddr,
						      vout->queue_buf_vaddr,
						      vout->buffer_cnt,
						      vout->queue_buf_size);
			if (retval < 0)
				break;

			/* Init buffer queues */
			vout->done_q.head = 0;
			vout->done_q.tail = 0;
			vout->ready_q.head = 0;
			vout->ready_q.tail = 0;

			for (i = 0; i < vout->buffer_cnt; i++) {
				memset(&(vout->v4l2_bufs[i]), 0,
				       sizeof(vout->v4l2_bufs[i]));
				vout->v4l2_bufs[i].flags = 0;
				vout->v4l2_bufs[i].memory = V4L2_MEMORY_MMAP;
				vout->v4l2_bufs[i].index = i;
				vout->v4l2_bufs[i].type =
				    V4L2_BUF_TYPE_VIDEO_OUTPUT;
				vout->v4l2_bufs[i].length =
				    PAGE_ALIGN(vout->v2f.fmt.pix.sizeimage);
				vout->v4l2_bufs[i].m.offset =
				    (unsigned long)vout->queue_buf_paddr[i];
				vout->v4l2_bufs[i].timestamp.tv_sec = 0;
				vout->v4l2_bufs[i].timestamp.tv_usec = 0;
			}
			break;
		}
	case VIDIOC_QUERYBUF:
		{
			struct v4l2_buffer *buf = arg;
			u32 type = buf->type;
			int index = buf->index;

			if ((type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
			    (index >= vout->buffer_cnt)) {
				pr_debug
				    ("VIDIOC_QUERYBUFS: incorrect buffer type\n");
				retval = -EINVAL;
				break;
			}
			down(&vout->param_lock);
			memcpy(buf, &(vout->v4l2_bufs[index]), sizeof(*buf));
			up(&vout->param_lock);
			break;
		}
	case VIDIOC_QBUF:
		{
			struct v4l2_buffer *buf = arg;
			int index = buf->index;
			unsigned long lock_flags;
			unsigned long timeout;

			if ((buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
			    (index >= vout->buffer_cnt) || (buf->flags != 0)) {
				retval = -EINVAL;
				break;
			}

			pr_debug("VIDIOC_QBUF: %d\n", buf->index);

			spin_lock_irqsave(&g_lock, lock_flags);

			memcpy(&(vout->v4l2_bufs[index]), buf, sizeof(*buf));
			vout->v4l2_bufs[index].flags |= V4L2_BUF_FLAG_QUEUED;

			g_buf_q_cnt++;
			queue_buf(&vout->ready_q, index);

			if (vout->state == STATE_STREAM_PAUSED) {
				index = peek_next_buf(&vout->ready_q);

				/* if timestamp is 0, then default to 30fps */
				if ((vout->v4l2_bufs[index].timestamp.tv_sec ==
				     0)
				    && (vout->v4l2_bufs[index].timestamp.
					tv_usec == 0))
					timeout =
					    vout->start_jiffies +
					    vout->frame_count * HZ / 30;
				else
					timeout =
					    get_jiffies(&vout->v4l2_bufs[index].
							timestamp);

				if (jiffies >= timeout) {
					pr_debug
					    ("warning: timer timeout already expired.\n");
				}

				vout->output_timer.expires = timeout;
				pr_debug
				    ("QBUF:Add timer %d timeout @ %lu jiffies, "
				     "current = %lu\n", index, timeout,
				     jiffies);
				add_timer(&vout->output_timer);
				vout->state = STATE_STREAM_ON;
			}

			spin_unlock_irqrestore(&g_lock, lock_flags);
			break;
		}
	case VIDIOC_DQBUF:
		{
			struct v4l2_buffer *buf = arg;
			int idx;

			pr_debug("VIDIOC_DQBUF: q size = %d\n",
				 queue_size(&vout->done_q));

			if ((queue_size(&vout->done_q) == 0) &&
			    (file->f_flags & O_NONBLOCK)) {
				retval = -EAGAIN;
				break;
			}

			if (!wait_event_interruptible_timeout(vout->v4l_bufq,
							      queue_size(&vout->
									 done_q)
							      != 0, 10 * HZ)) {
				pr_debug("VIDIOC_DQBUF: timeout\n");
				retval = -ETIME;
				break;
			} else if (signal_pending(current)) {
				pr_debug("VIDIOC_DQBUF: interrupt received\n");
				retval = -ERESTARTSYS;
				break;
			}
			idx = dequeue_buf(&vout->done_q);
			if (idx == -1) {	/* No frame free */
				pr_debug
				    ("VIDIOC_DQBUF: no free buffers, returning\n");
				retval = -EAGAIN;
				break;
			}
			if ((vout->v4l2_bufs[idx].flags & V4L2_BUF_FLAG_DONE) ==
			    0)
				pr_debug
				    ("VIDIOC_DQBUF: buffer in done q, but not "
				     "flagged as done\n");

			vout->v4l2_bufs[idx].flags = 0;
			memcpy(buf, &(vout->v4l2_bufs[idx]), sizeof(*buf));
			pr_debug("VIDIOC_DQBUF: %d\n", buf->index);
			break;
		}
	case VIDIOC_STREAMON:
		{
			retval = mxc_v4l2out_streamon(vout);
			break;
		}
	case VIDIOC_STREAMOFF:
		{
			retval = mxc_v4l2out_streamoff(vout);
			break;
		}
	case VIDIOC_G_CTRL:
		{
			retval = mxc_get_v42lout_control(vout, arg);
			break;
		}
	case VIDIOC_S_CTRL:
		{
			retval = mxc_set_v42lout_control(vout, arg);
			break;
		}
	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *cap = arg;

			if (cap->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			cap->bounds = vout->crop_bounds[vout->cur_disp_output];
			cap->defrect = vout->crop_bounds[vout->cur_disp_output];
			retval = 0;
			break;
		}
	case VIDIOC_G_CROP:
		{
			struct v4l2_crop *crop = arg;

			if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			crop->c = vout->crop_current;
			break;
		}
	case VIDIOC_S_CROP:
		{
			struct v4l2_crop *crop = arg;
			struct v4l2_rect *b =
			    &(vout->crop_bounds[vout->cur_disp_output]);

			if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				retval = -EINVAL;
				break;
			}
			if (crop->c.height < 0) {
				retval = -EINVAL;
				break;
			}
			if (crop->c.width < 0) {
				retval = -EINVAL;
				break;
			}

			if (crop->c.top < b->top)
				crop->c.top = b->top;
			if (crop->c.top > b->top + b->height)
				crop->c.top = b->top + b->height;
			if (crop->c.height > b->top - crop->c.top + b->height)
				crop->c.height =
				    b->top - crop->c.top + b->height;

			if (crop->c.left < b->left)
				crop->c.top = b->left;
			if (crop->c.left > b->left + b->width)
				crop->c.top = b->left + b->width;
			if (crop->c.width > b->left - crop->c.left + b->width)
				crop->c.width =
				    b->left - crop->c.left + b->width;

			/* stride line limitation */
			crop->c.height -= crop->c.height % 8;
			crop->c.width -= crop->c.width % 8;

			vout->crop_current = crop->c;

			vout->display_buf_size = vout->crop_current.width *
			    vout->crop_current.height;
			vout->display_buf_size *=
			    fmt_to_bpp(SDC_FG_FB_FORMAT) / 8;
			break;
		}
	case VIDIOC_ENUMOUTPUT:
		{
			struct v4l2_output *output = arg;

			if ((output->index >= 2) ||
			    (vout->output_enabled[output->index] == false)) {
				retval = -EINVAL;
				break;
			}

			*output = mxc_outputs[0];
			output->name[4] = '0' + output->index;
			break;
		}
	case VIDIOC_G_OUTPUT:
		{
			int *p_output_num = arg;

			*p_output_num = vout->cur_disp_output;
			break;
		}
	case VIDIOC_S_OUTPUT:
		{
			int *p_output_num = arg;

			if ((*p_output_num >= 2) ||
			    (vout->output_enabled[*p_output_num] == false)) {
				retval = -EINVAL;
				break;
			}

			if (vout->state != STATE_STREAM_OFF) {
				retval = -EBUSY;
				break;
			}

			vout->cur_disp_output = *p_output_num;
			break;
		}
	case VIDIOC_G_FBUF:
		{
			struct v4l2_framebuffer *fb = arg;
			*fb = vout->v4l2_fb;
			break;
		}
	case VIDIOC_S_FBUF:
		{
			struct v4l2_framebuffer *fb = arg;
			vout->v4l2_fb = *fb;
			vout->v4l2_fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
			break;
		}
	case VIDIOC_ENUM_FMT:
	case VIDIOC_TRY_FMT:
	case VIDIOC_QUERYCTRL:
	case VIDIOC_G_PARM:
	case VIDIOC_ENUMSTD:
	case VIDIOC_G_STD:
	case VIDIOC_S_STD:
	case VIDIOC_G_TUNER:
	case VIDIOC_S_TUNER:
	case VIDIOC_G_FREQUENCY:
	case VIDIOC_S_FREQUENCY:
	default:
		retval = -EINVAL;
		break;
	}

	up(&vout->busy_lock);
	return retval;
}

/*
 * V4L2 interface - ioctl function
 *
 * @return  None
 */
static int
mxc_v4l2out_ioctl(struct inode *inode, struct file *file,
		  unsigned int cmd, unsigned long arg)
{
	return video_usercopy(inode, file, cmd, arg, mxc_v4l2out_do_ioctl);
}

/*!
 * V4L2 interface - mmap function
 *
 * @param file          structure file *
 *
 * @param vma           structure vm_area_struct *
 *
 * @return status       0 Success, EINTR busy lock error,
 *                      ENOBUFS remap_page error
 */
static int mxc_v4l2out_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *dev = file->private_data;
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	int res = 0;
	vout_data *vout = video_get_drvdata(dev);

	/* make this _really_ smp-safe */
	if (down_interruptible(&vout->busy_lock))
		return -EINTR;

	/* make buffers write-thru cacheable */
	vma->vm_page_prot = __pgprot(pgprot_val(vma->vm_page_prot) &
				     ~L_PTE_BUFFERABLE);

	if (remap_pfn_range(vma, start, vma->vm_pgoff, size, vma->vm_page_prot)) {
		pr_debug("mxc_mmap(V4L)i - remap_pfn_range failed\n");
		res = -ENOBUFS;
		goto mxc_mmap_exit;
	}

      mxc_mmap_exit:
	up(&vout->busy_lock);
	return res;
}

/*!
 * V4L2 interface - poll function
 *
 * @param file       structure file *
 *
 * @param wait       structure poll_table *
 *
 * @return  status   POLLIN | POLLRDNORM
 */
static unsigned int mxc_v4l2out_poll(struct file *file, poll_table * wait)
{
	struct video_device *dev = file->private_data;
	vout_data *vout = video_get_drvdata(dev);

	wait_queue_head_t *queue = NULL;
	int res = POLLIN | POLLRDNORM;

	if (down_interruptible(&vout->busy_lock))
		return -EINTR;

	queue = &vout->v4l_bufq;
	poll_wait(file, queue, wait);

	up(&vout->busy_lock);
	return res;
}

static struct file_operations mxc_v4l2out_fops = {
	.owner = THIS_MODULE,
	.open = mxc_v4l2out_open,
	.release = mxc_v4l2out_close,
	.ioctl = mxc_v4l2out_ioctl,
	.mmap = mxc_v4l2out_mmap,
	.poll = mxc_v4l2out_poll,
};

static struct video_device mxc_v4l2out_template = {
	.name = "MXC Video Output",
	.vfl_type = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING,
	.fops = &mxc_v4l2out_fops,
	.release = video_device_release,
};

/*!
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process.      The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, IPU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int mxc_v4l2out_probe(struct platform_device *pdev)
{
	int i;
	vout_data *vout;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	g_vout = vout = kmalloc(sizeof(vout_data), GFP_KERNEL);

	if (!vout)
		return 0;

	memset(vout, 0, sizeof(vout_data));

	vout->video_dev = video_device_alloc();
	if (vout->video_dev == NULL)
		return -1;
	vout->video_dev->minor = -1;

	*(vout->video_dev) = mxc_v4l2out_template;

	/* register v4l device */
	if (video_register_device(vout->video_dev,
				  VFL_TYPE_GRABBER, video_nr) == -1) {
		pr_debug("video_register_device failed\n");
		return 0;
	}
	pr_debug("mxc_v4l2out: registered device video%d\n",
		 vout->video_dev->minor & 0x1f);

	video_set_drvdata(vout->video_dev, vout);

	init_MUTEX(&vout->param_lock);
	init_MUTEX(&vout->busy_lock);

	/* setup outputs and cropping */
	vout->cur_disp_output = -1;
	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;
		if (strncmp(idstr, "DISP", 4) == 0) {
			int disp_num = i;
			vout->crop_bounds[disp_num].left = 0;
			vout->crop_bounds[disp_num].top = 0;
			vout->crop_bounds[disp_num].width =
			    registered_fb[i]->var.xres;
			vout->crop_bounds[disp_num].height =
			    registered_fb[i]->var.yres;
			vout->output_enabled[disp_num] = true;
			vout->output_fb_num[disp_num] = i;
			if (vout->cur_disp_output == -1)
				vout->cur_disp_output = disp_num;
		}

	}
	vout->crop_current = vout->crop_bounds[vout->cur_disp_output];

	/* Setup framebuffer parameters */
	vout->v4l2_fb.capability = V4L2_FBUF_CAP_EXTERNOVERLAY;
	vout->v4l2_fb.flags = V4L2_FBUF_FLAG_PRIMARY;

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_v4l2out_driver = {
	.driver = {
		   .name = "MXC Video Output",
		   .owner = THIS_MODULE,
		   .bus = &platform_bus_type,
		   },
	.probe = mxc_v4l2out_probe,
	.remove = NULL,
};

static void camera_platform_release(struct device *device)
{
}

static struct platform_device mxc_v4l2out_device = {
	.name = "MXC Video Output",
	.dev = {
		.release = camera_platform_release,
		},
	.id = 0,
};

/*!
 * mxc v4l2 init function
 *
 */
static int mxc_v4l2out_init(void)
{
	u8 err = 0;

	err = platform_driver_register(&mxc_v4l2out_driver);
	if (err == 0) {
		platform_device_register(&mxc_v4l2out_device);
	}
	return err;
}

/*!
 * mxc v4l2 cleanup function
 *
 */
static void mxc_v4l2out_clean(void)
{
	pr_debug("unregistering video\n");

	video_unregister_device(g_vout->video_dev);

	platform_driver_unregister(&mxc_v4l2out_driver);
	platform_device_unregister(&mxc_v4l2out_device);
	kfree(g_vout);
	g_vout = NULL;
}

module_init(mxc_v4l2out_init);
module_exit(mxc_v4l2out_clean);

module_param(video_nr, int, 0444);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("V4L2-driver for MXC video output");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
