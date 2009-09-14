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
 * @file drivers/media/video/mxc/output/mxc_v4l2_output.c
 *
 * @brief MXC V4L2 Video Output Driver
 *
 * Video4Linux2 Output Device using MXC IPU Post-processing functionality.
 *
 * @ingroup MXC_V4L2_OUTPUT
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/console.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mxcfb.h>
#include <media/v4l2-ioctl.h>
#include <asm/cacheflush.h>

#include "mxc_v4l2_output.h"

vout_data *g_vout;
#define INTERLACED_CONTENT(vout) ((cpu_is_mx51_rev(CHIP_REV_2_0) >= 1) && \
				  (((vout)->field_fmt == V4L2_FIELD_INTERLACED_TB) || \
				   ((vout)->field_fmt == V4L2_FIELD_INTERLACED_BT)))
#define LOAD_3FIELDS(vout) ((INTERLACED_CONTENT(vout)) && \
			    ((vout)->motion_sel != HIGH_MOTION))

#define SDC_FG_FB_FORMAT        IPU_PIX_FMT_RGB565

struct v4l2_output mxc_outputs[2] = {
	{
	 .index = MXC_V4L2_OUT_2_SDC,
	 .name = "DISP3 Video Out",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,	/* not really correct,
						   but no other choice */
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN},
	{
	 .index = MXC_V4L2_OUT_2_ADC,
	 .name = "DISPx Video Out",
	 .type = V4L2_OUTPUT_TYPE_ANALOG,	/* not really correct,
						   but no other choice */
	 .audioset = 0,
	 .modulator = 0,
	 .std = V4L2_STD_UNKNOWN}
};

static int video_nr = 16;
static int pending_buffer;
static int pp_eof;
static spinlock_t g_lock = SPIN_LOCK_UNLOCKED;
static int last_index_n;
static int last_index_c;

/* debug counters */
uint32_t g_irq_cnt;
uint32_t g_buf_output_cnt;
uint32_t g_buf_q_cnt;
uint32_t g_buf_dq_cnt;

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
			printk(KERN_ERR "dma_alloc_coherent failed.\n");
			return -ENOBUFS;
		}
		pr_debug("allocated @ paddr=0x%08X, size=%d.\n",
			 (u32) bufs_paddr[i], size);
	}

	return 0;
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

	bpp = 8*bytes_per_pixel(pixelformat);
	return bpp;
}

static bool format_is_yuv(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_NV12:
		return true;
		break;
	}
	return false;
}

static u32 bpp_to_fmt(struct fb_info *fbi)
{
	if (fbi->var.nonstd)
		return fbi->var.nonstd;

	if (fbi->var.bits_per_pixel == 24)
		return V4L2_PIX_FMT_BGR24;
	else if (fbi->var.bits_per_pixel == 32)
		return V4L2_PIX_FMT_BGR32;
	else if (fbi->var.bits_per_pixel == 16)
		return V4L2_PIX_FMT_RGB565;

	return 0;
}

static irqreturn_t mxc_v4l2out_disp_refresh_irq_handler(int irq, void *dev_id)
{
	vout_data *vout = dev_id;
	int index, last_buf, ret;
	unsigned long timeout;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&g_lock, lock_flags);

	g_irq_cnt++;

	if (vout->ic_bypass && (pending_buffer || vout->frame_count < 3)) {
		last_buf = vout->ipu_buf[vout->next_done_ipu_buf];
		if (last_buf != -1) {
			g_buf_output_cnt++;
			vout->v4l2_bufs[last_buf].flags = V4L2_BUF_FLAG_DONE;
			queue_buf(&vout->done_q, last_buf);
			vout->ipu_buf[vout->next_done_ipu_buf] = -1;
			wake_up_interruptible(&vout->v4l_bufq);
			vout->next_done_ipu_buf = !vout->next_done_ipu_buf;
		}
	}

	if ((pending_buffer) && (pp_eof || vout->ic_bypass)) {
		pp_eof = 0;
		if (vout->ic_bypass) {
			ret = ipu_select_buffer(vout->display_ch, IPU_INPUT_BUFFER,
					  vout->next_rdy_ipu_buf);
		} else {
			if (LOAD_3FIELDS(vout)) {
				ret = ipu_select_multi_vdi_buffer(vout->next_rdy_ipu_buf);
			} else {
				ret = ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER,
							vout->next_rdy_ipu_buf);
			}
		}
		if (ret < 0) {
			dev_err(&vout->video_dev->dev,
				"unable to set IPU buffer ready\n");
		}
		vout->next_rdy_ipu_buf = !vout->next_rdy_ipu_buf;

		pending_buffer = 0;

		/* Setup timer for next buffer */
		index = peek_next_buf(&vout->ready_q);
		if (index != -1) {
			/* if timestamp is 0, then default to 30fps */
			if ((vout->v4l2_bufs[index].timestamp.tv_sec == 0)
				&& (vout->v4l2_bufs[index].timestamp.tv_usec == 0)
				&& vout->start_jiffies)
				timeout =
					vout->start_jiffies + vout->frame_count * HZ / 30;
			else
				timeout =
					get_jiffies(&vout->v4l2_bufs[index].timestamp);

			if (jiffies >= timeout) {
				dev_dbg(&vout->video_dev->dev,
					"warning: timer timeout already expired.\n");
			}
			if (mod_timer(&vout->output_timer, timeout))
				dev_dbg(&vout->video_dev->dev,
					"warning: timer was already set\n");

			dev_dbg(&vout->video_dev->dev,
				"timer handler next schedule: %lu\n", timeout);
		} else {
			vout->state = STATE_STREAM_PAUSED;
		}
	}

	if (vout->state == STATE_STREAM_STOPPING) {
		if ((vout->ipu_buf[0] == -1) && (vout->ipu_buf[1] == -1)) {
			vout->state = STATE_STREAM_OFF;
		}
	}

	spin_unlock_irqrestore(&g_lock, lock_flags);

	return IRQ_HANDLED;
}

static int get_display_irq(vout_data *vout)
{

	int disp_irq = 0;

	switch (vout->display_ch) {
	case MEM_FG_SYNC:
	case MEM_BG_SYNC:
		disp_irq = IPU_IRQ_BG_SF_END;
		break;
	case MEM_DC_SYNC:
		disp_irq = IPU_IRQ_DC_FC_1;
		break;
	default:
		dev_err(&vout->video_dev->dev,
			"not support display channel\n");
	}

	return disp_irq;
}

static void mxc_v4l2out_timer_handler(unsigned long arg)
{
	int index, ret;
	unsigned long lock_flags = 0;
	vout_data *vout = (vout_data *) arg;

	spin_lock_irqsave(&g_lock, lock_flags);

	if ((vout->state == STATE_STREAM_STOPPING)
	    || (vout->state == STATE_STREAM_OFF))
		goto exit0;
	/*
	 * If timer occurs before IPU h/w is ready, then set the state to
	 * paused and the timer will be set again when next buffer is queued
	 * or PP comletes
	 */
	if (vout->ipu_buf[vout->next_rdy_ipu_buf] != -1) {
		dev_dbg(&vout->video_dev->dev, "IPU buffer busy\n");
		vout->state = STATE_STREAM_PAUSED;
		goto exit0;
	}

	/* Dequeue buffer and pass to IPU */
	unsigned int aid_field_offset, current_field_offset;
	if (INTERLACED_CONTENT(vout)) {
		if (((LOAD_3FIELDS(vout)) && (vout->next_rdy_ipu_buf)) ||
		    ((!LOAD_3FIELDS(vout)) && !(vout->next_rdy_ipu_buf))) {
			aid_field_offset = vout->bytesperline;
			current_field_offset = 0;
			index = last_index_n;
		} else {
			aid_field_offset = 0;
			current_field_offset = vout->bytesperline;
			index = dequeue_buf(&vout->ready_q);
			if (index == -1) {	/* no buffers ready, should never occur */
				dev_err(&vout->video_dev->dev,
					"mxc_v4l2out: timer - no queued buffers ready\n");
				goto exit0;
			}
			g_buf_dq_cnt++;
			vout->frame_count++;
			last_index_n = index;
		}
	} else {
		current_field_offset = 0;
		index = dequeue_buf(&vout->ready_q);
		if (index == -1) {	/* no buffers ready, should never occur */
			dev_err(&vout->video_dev->dev,
				"mxc_v4l2out: timer - no queued buffers ready\n");
			goto exit0;
		}
		g_buf_dq_cnt++;
		vout->frame_count++;
	}

	if (vout->ic_bypass) {
		vout->ipu_buf[vout->next_rdy_ipu_buf] = index;
		ret = ipu_update_channel_buffer(vout->display_ch, IPU_INPUT_BUFFER,
				      vout->next_rdy_ipu_buf,
				      vout->v4l2_bufs[index].m.offset);
	} else {
		if (LOAD_3FIELDS(vout)) {
			int index_n = index;
			index = last_index_n;
			int index_p = last_index_c;
			vout->ipu_buf_p[vout->next_rdy_ipu_buf] = index_p;
			vout->ipu_buf[vout->next_rdy_ipu_buf] = last_index_c = index;
			vout->ipu_buf_n[vout->next_rdy_ipu_buf] = last_index_n = index_n;
			last_index_n = vout->ipu_buf_n[vout->next_rdy_ipu_buf];
			last_index_c = vout->ipu_buf[vout->next_rdy_ipu_buf];
			ret = ipu_update_channel_buffer(vout->post_proc_ch,
							IPU_INPUT_BUFFER,
							vout->next_rdy_ipu_buf,
							vout->v4l2_bufs[index].m.offset+current_field_offset);
			ret += ipu_update_channel_buffer(MEM_VDI_PRP_VF_MEM_P,
							 IPU_INPUT_BUFFER,
							 vout->next_rdy_ipu_buf,
							 vout->v4l2_bufs[index_p].m.offset+aid_field_offset);
			ret += ipu_update_channel_buffer(MEM_VDI_PRP_VF_MEM_N,
							 IPU_INPUT_BUFFER,
							 vout->next_rdy_ipu_buf,
							 vout->v4l2_bufs[index_n].m.offset+aid_field_offset);
		} else {
			vout->ipu_buf[vout->next_rdy_ipu_buf] = index;
			ret = ipu_update_channel_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER,
							vout->next_rdy_ipu_buf,
							vout->v4l2_bufs[index].m.offset+current_field_offset);
		}
	}
	if (ret < 0) {
		dev_err(&vout->video_dev->dev,
			"unable to update buffer %d address rc=%d\n",
			vout->next_rdy_ipu_buf, ret);
		goto exit0;
	}

	pending_buffer = 1;

	spin_unlock_irqrestore(&g_lock, lock_flags);

	return;

      exit0:
	spin_unlock_irqrestore(&g_lock, lock_flags);
}

static irqreturn_t mxc_v4l2out_pp_in_irq_handler(int irq, void *dev_id)
{
	int last_buf;
	int index;
	unsigned long timeout;
	unsigned long lock_flags = 0;
	vout_data *vout = dev_id;

	spin_lock_irqsave(&g_lock, lock_flags);

	g_irq_cnt++;

	/* Process previous buffer */
	if (LOAD_3FIELDS(vout))
		last_buf = vout->ipu_buf_p[vout->next_done_ipu_buf];
	else
		last_buf = vout->ipu_buf[vout->next_done_ipu_buf];

	if (last_buf != -1) {
		if ((!INTERLACED_CONTENT(vout)) || (vout->next_done_ipu_buf)) {
			g_buf_output_cnt++;
			vout->v4l2_bufs[last_buf].flags = V4L2_BUF_FLAG_DONE;
			queue_buf(&vout->done_q, last_buf);
			wake_up_interruptible(&vout->v4l_bufq);
		}
		vout->ipu_buf[vout->next_done_ipu_buf] = -1;
		if (LOAD_3FIELDS(vout)) {
			vout->ipu_buf_p[vout->next_done_ipu_buf] = -1;
			vout->ipu_buf_n[vout->next_done_ipu_buf] = -1;
		}
		vout->next_done_ipu_buf = !vout->next_done_ipu_buf;
	}
	pp_eof = 1;

	if (vout->state == STATE_STREAM_STOPPING) {
		if ((vout->ipu_buf[0] == -1) && (vout->ipu_buf[1] == -1)) {
			vout->state = STATE_STREAM_OFF;
		}
	} else if ((vout->state == STATE_STREAM_PAUSED)
		   && ((index = peek_next_buf(&vout->ready_q)) != -1)) {
		/* Setup timer for next buffer, when stream has been paused */
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
 *  Initialize VDI channels
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int init_VDI_channel(vout_data *vout, ipu_channel_params_t params)
{
	struct device *dev = &vout->video_dev->dev;

	if (ipu_init_channel(MEM_VDI_PRP_VF_MEM, &params) != 0) {
		dev_dbg(dev, "Error initializing VDI current channel\n");
		return -EINVAL;
	}
	if (LOAD_3FIELDS(vout)) {
		if (ipu_init_channel(MEM_VDI_PRP_VF_MEM_P, &params) != 0) {
			dev_err(dev, "Error initializing VDI previous channel\n");
			return -EINVAL;
		}
		if (ipu_init_channel(MEM_VDI_PRP_VF_MEM_N, &params) != 0) {
			dev_err(dev, "Error initializing VDI next channel\n");
			return -EINVAL;
		}
	}
	return 0;
}

/*!
 * Initialize VDI channel buffers
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int init_VDI_in_channel_buffer(vout_data *vout, uint32_t in_pixel_fmt,
				   uint16_t in_width, uint16_t in_height,
				   uint32_t stride,
				   dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
				   uint32_t u_offset, uint32_t v_offset)
{
	struct device *dev = &vout->video_dev->dev;

	if (ipu_init_channel_buffer(MEM_VDI_PRP_VF_MEM, IPU_INPUT_BUFFER,
				    in_pixel_fmt, in_width, in_height, stride,
				    IPU_ROTATE_NONE,
				    vout->v4l2_bufs[vout->ipu_buf[0]].m.offset+vout->bytesperline,
				    vout->v4l2_bufs[vout->ipu_buf[0]].m.offset,
				    u_offset, v_offset) != 0) {
		dev_err(dev, "Error initializing VDI current input buffer\n");
		return -EINVAL;
	}
	if (LOAD_3FIELDS(vout)) {
		if (ipu_init_channel_buffer(MEM_VDI_PRP_VF_MEM_P,
					    IPU_INPUT_BUFFER,
					    in_pixel_fmt, in_width, in_height,
					    stride, IPU_ROTATE_NONE,
					    vout->v4l2_bufs[vout->ipu_buf_p[0]].m.offset,
					    vout->v4l2_bufs[vout->ipu_buf_p[0]].m.offset+vout->bytesperline,
					    u_offset, v_offset) != 0) {
			dev_err(dev, "Error initializing VDI previous input buffer\n");
			return -EINVAL;
		}
		if (ipu_init_channel_buffer(MEM_VDI_PRP_VF_MEM_N,
					    IPU_INPUT_BUFFER,
					    in_pixel_fmt, in_width, in_height,
					    stride, IPU_ROTATE_NONE,
					    vout->v4l2_bufs[vout->ipu_buf_n[0]].m.offset,
					    vout->v4l2_bufs[vout->ipu_buf_n[0]].m.offset+vout->bytesperline,
					    u_offset, v_offset) != 0) {
			dev_err(dev, "Error initializing VDI next input buffer\n");
			return -EINVAL;
		}
	}
	return 0;
}

/*!
 * Initialize VDI path
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int init_VDI(ipu_channel_params_t params, vout_data *vout,
			struct device *dev, struct fb_info *fbi,
			ipu_channel_t  *display_input_ch, u16 out_width,
			u16 out_height)
{
	params.mem_prp_vf_mem.in_width = vout->v2f.fmt.pix.width;
	params.mem_prp_vf_mem.in_height = vout->v2f.fmt.pix.height;
	params.mem_prp_vf_mem.motion_sel = vout->motion_sel;
	params.mem_prp_vf_mem.field_fmt = vout->field_fmt;
	params.mem_prp_vf_mem.in_pixel_fmt = vout->v2f.fmt.pix.pixelformat;
	params.mem_prp_vf_mem.out_width = out_width;
	params.mem_prp_vf_mem.out_height = out_height;
	if (vout->display_ch == ADC_SYS2)
		params.mem_prp_vf_mem.out_pixel_fmt = SDC_FG_FB_FORMAT;
	else
		params.mem_prp_vf_mem.out_pixel_fmt = bpp_to_fmt(fbi);

	if (init_VDI_channel(vout, params) != 0) {
		dev_err(dev, "Error init_VDI_channel channel\n");
		return -EINVAL;
	}


	if (init_VDI_in_channel_buffer(vout,
				       params.mem_prp_vf_mem.in_pixel_fmt,
				       params.mem_prp_vf_mem.in_width,
				       params.mem_prp_vf_mem.in_height,
				       bytes_per_pixel(params.mem_prp_vf_mem.
						       in_pixel_fmt),
				       vout->v4l2_bufs[vout->ipu_buf[0]].m.offset,
				       vout->v4l2_bufs[vout->ipu_buf[1]].m.offset,
				       vout->offset.u_offset,
				       vout->offset.v_offset) != 0) {
		return -EINVAL;
	}

	if (!ipu_can_rotate_in_place(vout->rotate)) {
		if (vout->rot_pp_bufs[0]) {
			mxc_free_buffers(vout->rot_pp_bufs,
					 vout->rot_pp_bufs_vaddr, 2,
					 vout->display_buf_size);
		}
		if (mxc_allocate_buffers
		    (vout->rot_pp_bufs, vout->rot_pp_bufs_vaddr, 2,
		     vout->display_buf_size) < 0) {
			return -ENOBUFS;
		}

		if (ipu_init_channel_buffer(vout->post_proc_ch,
					    IPU_OUTPUT_BUFFER,
					    params.mem_prp_vf_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    IPU_ROTATE_NONE,
					    vout->rot_pp_bufs[0],
					    vout->rot_pp_bufs[1], 0, 0) != 0) {
			dev_err(dev, "Error initializing PRP output buffer\n");
			return -EINVAL;
		}

		if (ipu_init_channel(MEM_ROT_VF_MEM, NULL) != 0) {
			dev_err(dev, "Error initializing PP ROT channel\n");
			return -EINVAL;
		}
		if (ipu_init_channel_buffer(MEM_ROT_VF_MEM,
					    IPU_INPUT_BUFFER,
					    params.mem_prp_vf_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    vout->rotate,
					    vout->rot_pp_bufs[0],
					    vout->rot_pp_bufs[1], 0, 0) != 0) {
			dev_err(dev,
				"Error initializing PP ROT input buffer\n");
			return -EINVAL;
		}

		/* swap width and height */
		if (vout->rotate >= IPU_ROTATE_90_RIGHT) {
			out_width = vout->crop_current.width;
			out_height = vout->crop_current.height;
		}

		if (ipu_init_channel_buffer(MEM_ROT_VF_MEM,
					    IPU_OUTPUT_BUFFER,
					    params.mem_prp_vf_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    IPU_ROTATE_NONE,
					    vout->display_bufs[0],
					    vout->display_bufs[1], 0, 0) != 0) {
			dev_err(dev,
				"Error initializing PP-VDI output buffer\n");
			return -EINVAL;
		}

		if (ipu_link_channels(vout->post_proc_ch, MEM_ROT_VF_MEM) < 0)
			return -EINVAL;

		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(MEM_ROT_VF_MEM, IPU_OUTPUT_BUFFER, 1);

		ipu_enable_channel(MEM_ROT_VF_MEM);
		*display_input_ch = MEM_ROT_VF_MEM;

	} else {
		if (ipu_init_channel_buffer(vout->post_proc_ch,
					    IPU_OUTPUT_BUFFER,
					    params.mem_prp_vf_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    vout->rotate,
					    vout->display_bufs[0],
					    vout->display_bufs[1], 0, 0) != 0) {
			dev_err(dev,
				"Error initializing PP-VDI output buffer\n");
			return -EINVAL;
		}
	}
	return 0;
}

/*!
 * Initialize PP path
 *
 * @param params    structure ipu_channel_params_t
 *
 * @param vout      structure vout_data *
 *
 * @return status  0 Success
 */
static int init_PP(ipu_channel_params_t params, vout_data *vout,
		   struct device *dev, struct fb_info *fbi,
		   ipu_channel_t *display_input_ch, u16 out_width,
		   u16 out_height)
{
	params.mem_pp_mem.in_width = vout->v2f.fmt.pix.width;
	params.mem_pp_mem.in_height = vout->v2f.fmt.pix.height;
	params.mem_pp_mem.in_pixel_fmt = vout->v2f.fmt.pix.pixelformat;
	params.mem_pp_mem.out_width = out_width;
	params.mem_pp_mem.out_height = out_height;
	if (vout->display_ch == ADC_SYS2)
		params.mem_pp_mem.out_pixel_fmt = SDC_FG_FB_FORMAT;
	else
		params.mem_pp_mem.out_pixel_fmt = bpp_to_fmt(fbi);
	if (ipu_init_channel(vout->post_proc_ch, &params) != 0) {
		dev_err(dev, "Error initializing PP channel\n");
		return -EINVAL;
	}

	if (ipu_init_channel_buffer(vout->post_proc_ch,
				    IPU_INPUT_BUFFER,
				    params.mem_pp_mem.in_pixel_fmt,
				    params.mem_pp_mem.in_width,
				    params.mem_pp_mem.in_height,
				    vout->v2f.fmt.pix.bytesperline /
				    bytes_per_pixel(params.mem_pp_mem.
						    in_pixel_fmt),
				    IPU_ROTATE_NONE,
				    vout->v4l2_bufs[vout->ipu_buf[0]].m.offset,
				    vout->v4l2_bufs[vout->ipu_buf[1]].m.offset,
				    vout->offset.u_offset,
				    vout->offset.v_offset) != 0) {
		dev_err(dev, "Error initializing PP input buffer\n");
		return -EINVAL;
	}

	if (!ipu_can_rotate_in_place(vout->rotate)) {
		if (vout->rot_pp_bufs[0]) {
			mxc_free_buffers(vout->rot_pp_bufs,
					 vout->rot_pp_bufs_vaddr, 2,
					 vout->display_buf_size);
		}
		if (mxc_allocate_buffers
		    (vout->rot_pp_bufs, vout->rot_pp_bufs_vaddr, 2,
		     vout->display_buf_size) < 0) {
			return -ENOBUFS;
		}

		if (ipu_init_channel_buffer(vout->post_proc_ch,
					    IPU_OUTPUT_BUFFER,
					    params.mem_pp_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    IPU_ROTATE_NONE,
					    vout->rot_pp_bufs[0],
					    vout->rot_pp_bufs[1], 0, 0) != 0) {
			dev_err(dev, "Error initializing PP output buffer\n");
			return -EINVAL;
		}

		if (ipu_init_channel(MEM_ROT_PP_MEM, NULL) != 0) {
			dev_err(dev, "Error initializing PP ROT channel\n");
			return -EINVAL;
		}
		if (ipu_init_channel_buffer(MEM_ROT_PP_MEM,
					    IPU_INPUT_BUFFER,
					    params.mem_pp_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    vout->rotate,
					    vout->rot_pp_bufs[0],
					    vout->rot_pp_bufs[1], 0, 0) != 0) {
			dev_err(dev,
				"Error initializing PP ROT input buffer\n");
			return -EINVAL;
		}

		/* swap width and height */
		if (vout->rotate >= IPU_ROTATE_90_RIGHT) {
			out_width = vout->crop_current.width;
			out_height = vout->crop_current.height;
		}

		if (ipu_init_channel_buffer(MEM_ROT_PP_MEM,
					    IPU_OUTPUT_BUFFER,
					    params.mem_pp_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    IPU_ROTATE_NONE,
					    vout->display_bufs[0],
					    vout->display_bufs[1], 0, 0) != 0) {
			dev_err(dev, "Error initializing PP output buffer\n");
			return -EINVAL;
		}

		if (ipu_link_channels(vout->post_proc_ch, MEM_ROT_PP_MEM) < 0)
			return -EINVAL;

		ipu_select_buffer(MEM_ROT_PP_MEM, IPU_OUTPUT_BUFFER, 0);
		ipu_select_buffer(MEM_ROT_PP_MEM, IPU_OUTPUT_BUFFER, 1);

		ipu_enable_channel(MEM_ROT_PP_MEM);
		*display_input_ch = MEM_ROT_PP_MEM;

	} else {
		if (ipu_init_channel_buffer(vout->post_proc_ch,
					    IPU_OUTPUT_BUFFER,
					    params.mem_pp_mem.
					    out_pixel_fmt, out_width,
					    out_height, out_width,
					    vout->rotate,
					    vout->display_bufs[0],
					    vout->display_bufs[1], 0, 0) != 0) {
			dev_err(dev, "Error initializing PP output buffer\n");
			return -EINVAL;
		}
	}
	return 0;
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
	struct device *dev = &vout->video_dev->dev;
	ipu_channel_params_t params;
	struct mxcfb_pos fb_pos;
	struct fb_var_screeninfo fbvar;
	struct fb_info *fbi =
	    registered_fb[vout->output_fb_num[vout->cur_disp_output]];
	u16 out_width;
	u16 out_height;
	int disp_irq = 0;
	ipu_channel_t display_input_ch;
	bool use_direct_adc = false;
	mm_segment_t old_fs;

	dev_dbg(dev, "mxc_v4l2out_streamon: field format=%d\n",
		vout->field_fmt);
	if (INTERLACED_CONTENT(vout)) {
		ipu_request_irq(IPU_IRQ_PRP_VF_OUT_EOF,
				mxc_v4l2out_pp_in_irq_handler,
				0, &vout->video_dev->name, vout);
		display_input_ch = MEM_VDI_PRP_VF_MEM;
	} else {
		ipu_request_irq(IPU_IRQ_PP_IN_EOF,
				mxc_v4l2out_pp_in_irq_handler,
				0, &vout->video_dev->name, vout);
		display_input_ch = MEM_PP_MEM;
	}

	if (!vout)
		return -EINVAL;

	if (vout->state != STATE_STREAM_OFF)
		return -EBUSY;

	if (queue_size(&vout->ready_q) < 2) {
		dev_err(dev, "2 buffers not been queued yet!\n");
		return -EINVAL;
	}

	if ((vout->field_fmt == V4L2_FIELD_BOTTOM) || (vout->field_fmt == V4L2_FIELD_TOP)) {
		dev_err(dev, "4 queued buffers need, not supported yet!\n");
		return -EINVAL;
	}

	pending_buffer = 0;

	out_width = vout->crop_current.width;
	out_height = vout->crop_current.height;
	vout->next_done_ipu_buf = 0;
	vout->next_rdy_ipu_buf = 1;

	if (!INTERLACED_CONTENT(vout)) {
		vout->next_done_ipu_buf = vout->next_rdy_ipu_buf = 0;
		vout->ipu_buf[0] = dequeue_buf(&vout->ready_q);
		vout->ipu_buf[1] = dequeue_buf(&vout->ready_q);
		vout->frame_count = 2;
	} else if (!LOAD_3FIELDS(vout)) {
		vout->ipu_buf[0] = dequeue_buf(&vout->ready_q);
		vout->ipu_buf[1] = -1;
		vout->frame_count = 1;
		last_index_n = vout->ipu_buf[0];
	} else {
		vout->ipu_buf_p[0] = dequeue_buf(&vout->ready_q);
		vout->ipu_buf[0] = vout->ipu_buf_p[0];
		vout->ipu_buf_n[0] = dequeue_buf(&vout->ready_q);
		vout->ipu_buf_p[1] = -1;
		vout->ipu_buf[1] = -1;
		vout->ipu_buf_n[1] = -1;
		last_index_c = vout->ipu_buf[0];
		last_index_n = vout->ipu_buf_n[0];
		vout->frame_count = 2;
	}

	/* Init Display Channel */
#ifdef CONFIG_FB_MXC_ASYNC_PANEL
	if (INTERLACED_CONTENT(vout))
		ipu_enable_irq(IPU_IRQ_PRP_VF_OUT_EOF);
	else
		ipu_enable_irq(IPU_IRQ_PP_IN_EOF);

	if (vout->cur_disp_output < DISP3) {
		mxcfb_set_refresh_mode(fbi, MXCFB_REFRESH_OFF, 0);
		fbi = NULL;
		if (ipu_can_rotate_in_place(vout->rotate)) {
			dev_dbg(dev, "Using PP direct to ADC channel\n");
			use_direct_adc = true;
			vout->display_ch = MEM_PP_ADC;
			vout->post_proc_ch = MEM_PP_ADC;

			memset(&params, 0, sizeof(params));
			params.mem_pp_adc.in_width = vout->v2f.fmt.pix.width;
			params.mem_pp_adc.in_height = vout->v2f.fmt.pix.height;
			params.mem_pp_adc.in_pixel_fmt =
			    vout->v2f.fmt.pix.pixelformat;
			params.mem_pp_adc.out_width = out_width;
			params.mem_pp_adc.out_height = out_height;
			params.mem_pp_adc.out_pixel_fmt = SDC_FG_FB_FORMAT;
#ifdef CONFIG_FB_MXC_EPSON_PANEL
			params.mem_pp_adc.out_left =
			    2 + vout->crop_current.left;
#else
			params.mem_pp_adc.out_left =
			    12 + vout->crop_current.left;
#endif
			params.mem_pp_adc.out_top = vout->crop_current.top;
			if (ipu_init_channel(vout->post_proc_ch, &params) != 0) {
				dev_err(dev, "Error initializing PP chan\n");
				return -EINVAL;
			}
			if (ipu_init_channel_buffer(vout->post_proc_ch,
						    IPU_INPUT_BUFFER,
						    params.mem_pp_adc.
						    in_pixel_fmt,
						    params.mem_pp_adc.in_width,
						    params.mem_pp_adc.in_height,
						    vout->v2f.fmt.pix.
						    bytesperline /
						    bytes_per_pixel(params.
								    mem_pp_adc.
								    in_pixel_fmt),
						    vout->rotate,
						    vout->v4l2_bufs[vout->ipu_buf[0]].m.offset,
						    vout->v4l2_bufs[vout->ipu_buf[1]].m.offset,
						    vout->offset.u_offset,
						    vout->offset.v_offset) !=
			    0) {
				dev_err(dev, "Error initializing PP in buf\n");
				return -EINVAL;
			}

			if (ipu_init_channel_buffer(vout->post_proc_ch,
						    IPU_OUTPUT_BUFFER,
						    params.mem_pp_adc.
						    out_pixel_fmt, out_width,
						    out_height, out_width,
						    vout->rotate, 0, 0, 0,
						    0) != 0) {
				dev_err(dev,
					"Error initializing PP output buffer\n");
				return -EINVAL;
			}

		} else {
			dev_dbg(dev, "Using ADC SYS2 channel\n");
			vout->display_ch = ADC_SYS2;
			vout->post_proc_ch = MEM_PP_MEM;

			if (vout->display_bufs[0]) {
				mxc_free_buffers(vout->display_bufs,
						 vout->display_bufs_vaddr,
						 2, vout->display_buf_size);
			}

			vout->display_buf_size = vout->crop_current.width *
			    vout->crop_current.height *
			    fmt_to_bpp(SDC_FG_FB_FORMAT) / 8;
			mxc_allocate_buffers(vout->display_bufs,
					     vout->display_bufs_vaddr,
					     2, vout->display_buf_size);

			memset(&params, 0, sizeof(params));
			params.adc_sys2.disp = vout->cur_disp_output;
			params.adc_sys2.ch_mode = WriteTemplateNonSeq;
#ifdef CONFIG_FB_MXC_EPSON_PANEL
			params.adc_sys2.out_left = 2 + vout->crop_current.left;
#else
			params.adc_sys2.out_left = 12 + vout->crop_current.left;
#endif
			params.adc_sys2.out_top = vout->crop_current.top;
			if (ipu_init_channel(ADC_SYS2, &params) < 0)
				return -EINVAL;

			if (ipu_init_channel_buffer(vout->display_ch,
						    IPU_INPUT_BUFFER,
						    SDC_FG_FB_FORMAT,
						    out_width, out_height,
						    out_width, IPU_ROTATE_NONE,
						    vout->display_bufs[0],
						    vout->display_bufs[1], 0,
						    0) != 0) {
				dev_err(dev,
					"Error initializing SDC FG buffer\n");
				return -EINVAL;
			}
		}
	} else
#endif
	{			/* Use SDC */
		unsigned int ipu_ch = CHAN_NONE;

		dev_dbg(dev, "Using SDC channel\n");

		/* Bypass IC if resizing and rotation not needed
		   Always do CSC in DP
		   Meanwhile, apply IC bypass to SDC only
		 */
		if (out_width == vout->v2f.fmt.pix.width &&
			out_height == vout->v2f.fmt.pix.height &&
			ipu_can_rotate_in_place(vout->rotate)) {
			pr_debug("Bypassing IC\n");
			vout->ic_bypass = 1;
			ipu_disable_irq(IPU_IRQ_PP_IN_EOF);
		} else {
			vout->ic_bypass = 0;
		}

#ifdef CONFIG_MXC_IPU_V1
		/* IPUv1 needs IC to do CSC */
		if (format_is_yuv(vout->v2f.fmt.pix.pixelformat) !=
		    format_is_yuv(bpp_to_fmt(fbi)))
			vout->ic_bypass = 0;
#endif

		fbvar = fbi->var;

		if (fbi->fbops->fb_ioctl) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			fbi->fbops->fb_ioctl(fbi, MXCFB_GET_FB_IPU_CHAN,
					(unsigned long)&ipu_ch);
			set_fs(old_fs);
		}

		if (ipu_ch == CHAN_NONE) {
			dev_err(dev,
					"Can not get display ipu channel\n");
			return -EINVAL;
		}

		vout->display_ch = ipu_ch;

		if (vout->cur_disp_output == 3 || vout->cur_disp_output == 5) {
			fbvar.bits_per_pixel = 16;
			if (format_is_yuv(vout->v2f.fmt.pix.pixelformat))
				fbvar.nonstd = IPU_PIX_FMT_UYVY;
			else
				fbvar.nonstd = 0;

			fbvar.xres = fbvar.xres_virtual = out_width;
			fbvar.yres = out_height;
			fbvar.yres_virtual = out_height * 2;
		}

		if (vout->ic_bypass) {
			fbvar.bits_per_pixel = 8*
				bytes_per_pixel(vout->v2f.fmt.pix.pixelformat);
			fbvar.nonstd = vout->v2f.fmt.pix.pixelformat;
		}

		fbvar.activate |= FB_ACTIVATE_FORCE;
		fb_set_var(fbi, &fbvar);

		fb_pos.x = vout->crop_current.left;
		fb_pos.y = vout->crop_current.top;
		if (fbi->fbops->fb_ioctl) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			fbi->fbops->fb_ioctl(fbi, MXCFB_SET_OVERLAY_POS,
					     (unsigned long)&fb_pos);
			set_fs(old_fs);
		}

		vout->display_bufs[1] = fbi->fix.smem_start;
		vout->display_bufs[0] = fbi->fix.smem_start +
		    (fbi->fix.line_length * fbi->var.yres);
		vout->display_buf_size = vout->crop_current.width *
		    vout->crop_current.height * fbi->var.bits_per_pixel / 8;
		if (INTERLACED_CONTENT(vout))
			vout->post_proc_ch = MEM_VDI_PRP_VF_MEM;
		else
			vout->post_proc_ch = MEM_PP_MEM;
	}

	/* Init PP */
	if (use_direct_adc == false && !vout->ic_bypass) {
		if (INTERLACED_CONTENT(vout)) {
			vout->post_proc_ch = MEM_VDI_PRP_VF_MEM;
			ipu_enable_irq(IPU_IRQ_PRP_VF_OUT_EOF);
		} else {
			vout->post_proc_ch = MEM_PP_MEM;
			ipu_enable_irq(IPU_IRQ_PP_IN_EOF);
		}

		if (vout->rotate >= IPU_ROTATE_90_RIGHT) {
			out_width = vout->crop_current.height;
			out_height = vout->crop_current.width;
		}
		memset(&params, 0, sizeof(params));
		int rc;
		if (INTERLACED_CONTENT(vout)) {
			rc = init_VDI(params, vout, dev, fbi, &display_input_ch,
				      out_width, out_height);
		} else {
			rc = init_PP(params, vout, dev, fbi, &display_input_ch,
				     out_width, out_height);
		}
		if (rc < 0)
			return rc;
		if (ipu_link_channels(display_input_ch, vout->display_ch) < 0) {
			dev_err(dev, "Error linking ipu channels\n");
			return -EINVAL;
		}
	}

	vout->state = STATE_STREAM_PAUSED;

	if (use_direct_adc == false) {
		if (!vout->ic_bypass) {
#ifndef CONFIG_MXC_IPU_V1
			ipu_enable_channel(vout->post_proc_ch);
#endif
			if (LOAD_3FIELDS(vout)) {
				ipu_enable_channel(MEM_VDI_PRP_VF_MEM_P);
				ipu_enable_channel(MEM_VDI_PRP_VF_MEM_N);
				ipu_select_multi_vdi_buffer(0);
			} else if (INTERLACED_CONTENT(vout)) {
				ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER, 0);
			} else {
				ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER, 0);
				ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER, 1);
			}
			ipu_select_buffer(vout->post_proc_ch, IPU_OUTPUT_BUFFER, 0);
			ipu_select_buffer(vout->post_proc_ch, IPU_OUTPUT_BUFFER, 1);
#ifdef CONFIG_MXC_IPU_V1
			ipu_enable_channel(vout->post_proc_ch);
#endif
		} else {
			ipu_update_channel_buffer(vout->display_ch,
				IPU_INPUT_BUFFER,
				0, vout->v4l2_bufs[vout->ipu_buf[0]].m.offset);
			ipu_update_channel_buffer(vout->display_ch,
				IPU_INPUT_BUFFER,
				1, vout->v4l2_bufs[vout->ipu_buf[1]].m.offset);
			ipu_select_buffer(vout->display_ch, IPU_INPUT_BUFFER, 0);
			ipu_select_buffer(vout->display_ch, IPU_INPUT_BUFFER, 1);
		}
		disp_irq = get_display_irq(vout);
		ipu_request_irq(disp_irq, mxc_v4l2out_disp_refresh_irq_handler,
				0, NULL, vout);

		if (fbi) {
			acquire_console_sem();
			fb_blank(fbi, FB_BLANK_UNBLANK);
			release_console_sem();
		} else {
			ipu_enable_channel(vout->display_ch);
		}
	} else {
		ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER, 0);
		ipu_select_buffer(vout->post_proc_ch, IPU_INPUT_BUFFER, 1);
		ipu_enable_channel(vout->post_proc_ch);
	}
	vout->start_jiffies = jiffies;

	msleep(1);

	dev_dbg(dev,
		"streamon: start time = %lu jiffies\n", vout->start_jiffies);

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
	struct fb_info *fbi =
	    registered_fb[vout->output_fb_num[vout->cur_disp_output]];
	int i, retval = 0, disp_irq = 0;
	unsigned long lockflag = 0;

	if (!vout)
		return -EINVAL;

	if (vout->state == STATE_STREAM_OFF) {
		return 0;
	}

	if (INTERLACED_CONTENT(vout))
		ipu_free_irq(IPU_IRQ_PRP_VF_OUT_EOF, vout);
	else
		ipu_free_irq(IPU_IRQ_PP_IN_EOF, vout);

	spin_lock_irqsave(&g_lock, lockflag);

	del_timer(&vout->output_timer);

	if (vout->state == STATE_STREAM_ON) {
		vout->state = STATE_STREAM_STOPPING;
	}

	if (!vout->ic_bypass) {
		if (INTERLACED_CONTENT(vout))
			ipu_disable_irq(IPU_IRQ_PRP_VF_OUT_EOF);
		else
			ipu_disable_irq(IPU_IRQ_PP_IN_EOF);
	}

	spin_unlock_irqrestore(&g_lock, lockflag);

	pending_buffer = 0;
	disp_irq = get_display_irq(vout);
	ipu_free_irq(disp_irq, vout);

	if (vout->display_ch == MEM_FG_SYNC) {
		struct mxcfb_pos fb_pos;
		mm_segment_t old_fs;

		fb_pos.x = 0;
		fb_pos.y = 0;
		if (fbi->fbops->fb_ioctl) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			fbi->fbops->fb_ioctl(fbi, MXCFB_SET_OVERLAY_POS,
					(unsigned long)&fb_pos);
			set_fs(old_fs);
		}
	}

	if (vout->post_proc_ch == MEM_PP_MEM ||
	    vout->post_proc_ch == MEM_PRP_VF_MEM) {
		/* SDC or ADC with Rotation */
		if (!ipu_can_rotate_in_place(vout->rotate)) {
			ipu_unlink_channels(MEM_PP_MEM, MEM_ROT_PP_MEM);
			ipu_unlink_channels(MEM_ROT_PP_MEM,
					    vout->display_ch);
			ipu_disable_channel(MEM_ROT_PP_MEM, true);

			if (vout->rot_pp_bufs[0]) {
				mxc_free_buffers(vout->rot_pp_bufs,
						 vout->rot_pp_bufs_vaddr, 2,
						 vout->display_buf_size);
			}
		} else {
			ipu_unlink_channels(MEM_PP_MEM, vout->display_ch);
		}
		ipu_disable_channel(MEM_PP_MEM, true);

		if (vout->display_ch == ADC_SYS2 ||
			vout->display_ch == MEM_FG_SYNC) {
			ipu_disable_channel(vout->display_ch, true);
			ipu_uninit_channel(vout->display_ch);
		} else {
			fbi->var.activate |= FB_ACTIVATE_FORCE;
			fb_set_var(fbi, &fbi->var);

			if (vout->display_ch == MEM_FG_SYNC) {
				acquire_console_sem();
				fb_blank(fbi, FB_BLANK_POWERDOWN);
				release_console_sem();
			}

			vout->display_bufs[0] = 0;
			vout->display_bufs[1] = 0;
		}

		ipu_uninit_channel(MEM_PP_MEM);
		if (!ipu_can_rotate_in_place(vout->rotate))
			ipu_uninit_channel(MEM_ROT_PP_MEM);
	} else if (INTERLACED_CONTENT(vout) && (vout->post_proc_ch == MEM_VDI_PRP_VF_MEM)) {
		if (!ipu_can_rotate_in_place(vout->rotate)) {
			ipu_unlink_channels(MEM_VDI_PRP_VF_MEM,
					    MEM_ROT_VF_MEM);
			ipu_unlink_channels(MEM_ROT_VF_MEM,
					    vout->display_ch);
			ipu_disable_channel(MEM_ROT_VF_MEM, true);

			if (vout->rot_pp_bufs[0]) {
				mxc_free_buffers(vout->rot_pp_bufs,
						 vout->rot_pp_bufs_vaddr, 2,
						 vout->display_buf_size);
			}
		} else {
			ipu_unlink_channels(MEM_VDI_PRP_VF_MEM,
					    vout->display_ch);
		}

		ipu_disable_channel(MEM_VDI_PRP_VF_MEM, true);

		if (vout->display_ch == ADC_SYS2 ||
			vout->display_ch == MEM_FG_SYNC) {
			ipu_disable_channel(vout->display_ch, true);
			ipu_uninit_channel(vout->display_ch);
		} else {
			fbi->var.activate |= FB_ACTIVATE_FORCE;
			fb_set_var(fbi, &fbi->var);

			if (vout->display_ch == MEM_FG_SYNC) {
				acquire_console_sem();
				fb_blank(fbi, FB_BLANK_POWERDOWN);
				release_console_sem();
			}

			vout->display_bufs[0] = 0;
			vout->display_bufs[1] = 0;
		}

		ipu_uninit_channel(MEM_VDI_PRP_VF_MEM);
		if (!ipu_can_rotate_in_place(vout->rotate))
			ipu_uninit_channel(MEM_ROT_VF_MEM);
	} else {		/* ADC Direct */
		ipu_disable_channel(MEM_PP_ADC, true);
		ipu_uninit_channel(MEM_PP_ADC);
	}
	vout->ready_q.head = vout->ready_q.tail = 0;
	vout->done_q.head = vout->done_q.tail = 0;
	for (i = 0; i < vout->buffer_cnt; i++) {
		vout->v4l2_bufs[i].flags = 0;
		vout->v4l2_bufs[i].timestamp.tv_sec = 0;
		vout->v4l2_bufs[i].timestamp.tv_usec = 0;
	}

	vout->state = STATE_STREAM_OFF;

#ifdef CONFIG_FB_MXC_ASYNC_PANEL
	if (vout->cur_disp_output < DISP3) {
		if (vout->display_bufs[0] != 0) {
			mxc_free_buffers(vout->display_bufs,
					 vout->display_bufs_vaddr, 2,
					 vout->display_buf_size);
		}

		mxcfb_set_refresh_mode(registered_fb
				       [vout->
					output_fb_num[vout->cur_disp_output]],
				       MXCFB_REFRESH_PARTIAL, 0);
	}
#endif

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
	return ((palette == V4L2_PIX_FMT_RGB565) ||
		(palette == V4L2_PIX_FMT_BGR24) ||
		(palette == V4L2_PIX_FMT_RGB24) ||
		(palette == V4L2_PIX_FMT_BGR32) ||
		(palette == V4L2_PIX_FMT_RGB32) ||
		(palette == V4L2_PIX_FMT_NV12) ||
		(palette == V4L2_PIX_FMT_YUV422P) ||
		(palette == V4L2_PIX_FMT_YUV420));
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
		dev_err(&vout->video_dev->dev, "pixel format not supported\n");
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
	vout->bytesperline = bytesperline;

	/* Based on http://v4l2spec.bytesex.org/spec/x6386.htm#V4L2-FIELD */
	vout->field_fmt = f->fmt.pix.field;
	switch (vout->field_fmt) {
		/* Images are in progressive format, not interlaced */
	case V4L2_FIELD_NONE:
		break;
		/* The two fields of a frame are passed in separate buffers,
		   in temporal order, i. e. the older one first. */
	case V4L2_FIELD_ALTERNATE:
		dev_err(&vout->video_dev->dev,
			"V4L2_FIELD_ALTERNATE field format not supported yet!\n");
		break;
	case V4L2_FIELD_INTERLACED_TB:
		if (cpu_is_mx51())
			break;
	case V4L2_FIELD_INTERLACED_BT:
		dev_err(&vout->video_dev->dev,
			"V4L2_FIELD_INTERLACED_BT field format not supported yet!\n");
	default:
		vout->field_fmt = V4L2_FIELD_NONE;
		break;
	}

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		/* byteperline for YUV planar formats is for
		   Y plane only */
		size = bytesperline * f->fmt.pix.height * 2;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_NV12:
		size = (bytesperline * f->fmt.pix.height * 3) / 2;
		break;
	default:
		size = bytesperline * f->fmt.pix.height;
		break;
	}

	/* Return the actual size of the image to the app */
	if (f->fmt.pix.sizeimage < size) {
		f->fmt.pix.sizeimage = size;
	} else {
		size = f->fmt.pix.sizeimage;
	}

	vout->v2f.fmt.pix = f->fmt.pix;
	if (vout->v2f.fmt.pix.priv != 0) {
		if (copy_from_user(&vout->offset,
				   (void *)vout->v2f.fmt.pix.priv,
				   sizeof(vout->offset))) {
			retval = -EFAULT;
			goto err0;
		}
	} else {
		vout->offset.u_offset = 0;
		vout->offset.v_offset = 0;
	}

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
		vout->rotate |= c->value ? IPU_ROTATE_HORIZ_FLIP :
		    IPU_ROTATE_NONE;
		break;
	case V4L2_CID_VFLIP:
		vout->rotate |= c->value ? IPU_ROTATE_VERT_FLIP :
		    IPU_ROTATE_NONE;
		break;
	case V4L2_CID_MXC_ROT:
		vout->rotate = c->value;
		break;
	case V4L2_CID_MXC_MOTION:
		vout->motion_sel = c->value;
		break;
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
		return -ENODEV;
	}

	down(&vout->busy_lock);

	err = -EINTR;
	if (signal_pending(current))
		goto oops;


	if (vout->open_count++ == 0) {
		init_waitqueue_head(&vout->v4l_bufq);

		init_timer(&vout->output_timer);
		vout->output_timer.function = mxc_v4l2out_timer_handler;
		vout->output_timer.data = (unsigned long)vout;

		vout->state = STATE_STREAM_OFF;
		vout->rotate = IPU_ROTATE_NONE;
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
	struct video_device *dev = video_devdata(file);
	vout_data *vout = video_get_drvdata(dev);

	if (--vout->open_count == 0) {
		if (vout->state != STATE_STREAM_OFF)
			mxc_v4l2out_streamoff(vout);

		file->private_data = NULL;

		mxc_free_buffers(vout->queue_buf_paddr, vout->queue_buf_vaddr,
				 vout->buffer_cnt, vout->queue_buf_size);
		vout->buffer_cnt = 0;
		mxc_free_buffers(vout->rot_pp_bufs, vout->rot_pp_bufs_vaddr, 2,
				 vout->display_buf_size);

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
	struct video_device *vdev = file->private_data;
	vout_data *vout = video_get_drvdata(vdev);
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
				dev_dbg(&vdev->dev,
					"VIDIOC_REQBUFS: incorrect buffer type\n");
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
					dev_dbg(&vdev->dev,
						"VIDIOC_REQBUFS: freed buffers\n");
				}
				vout->buffer_cnt = 0;
			} else {
				dev_dbg(&vdev->dev,
					"VIDIOC_REQBUFS: Buffer is in use\n");
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
				dev_dbg(&vdev->dev,
					"VIDIOC_QUERYBUFS: incorrect buffer type\n");
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
			int param[5][3];

			if ((buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) ||
			    (index >= vout->buffer_cnt)) {
				retval = -EINVAL;
				break;
			}

			dev_dbg(&vdev->dev, "VIDIOC_QBUF: %d field = %d\n", buf->index, buf->field);

			/* mmapped buffers are L1 WB cached,
			 * so we need to clean them */
			if (buf->memory & V4L2_MEMORY_MMAP) {
				flush_cache_all();
			}

			spin_lock_irqsave(&g_lock, lock_flags);

			memcpy(&(vout->v4l2_bufs[index]), buf, sizeof(*buf));
			vout->v4l2_bufs[index].flags |= V4L2_BUF_FLAG_QUEUED;

			g_buf_q_cnt++;
			if (vout->v4l2_bufs[index].reserved)
				if (!copy_from_user(&param[0][0],
						    (void *)vout->
						    v4l2_bufs[index]
						    .reserved, sizeof(param)))
					ipu_set_csc_coefficients(vout->
								 display_ch,
								 param);
			queue_buf(&vout->ready_q, index);
			if (vout->state == STATE_STREAM_PAUSED) {
				unsigned long timeout;

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
					dev_dbg(&vout->video_dev->dev,
						"warning: timer timeout already expired.\n");
				}
				vout->output_timer.expires = timeout;
				dev_dbg(&vdev->dev,
					"QBUF: frame #%u timeout @ %lu jiffies, current = %lu\n",
					vout->frame_count, timeout, jiffies);
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

			if ((queue_size(&vout->done_q) == 0) &&
			    (file->f_flags & O_NONBLOCK)) {
				retval = -EAGAIN;
				break;
			}

			if (!wait_event_interruptible_timeout(vout->v4l_bufq,
							      queue_size(&vout->
									 done_q)
							      != 0, 10 * HZ)) {
				dev_dbg(&vdev->dev, "VIDIOC_DQBUF: timeout\n");
				retval = -ETIME;
				break;
			} else if (signal_pending(current)) {
				dev_dbg(&vdev->dev,
					"VIDIOC_DQBUF: interrupt received\n");
				retval = -ERESTARTSYS;
				break;
			}
			idx = dequeue_buf(&vout->done_q);
			if (idx == -1) {	/* No frame free */
				dev_dbg(&vdev->dev,
					"VIDIOC_DQBUF: no free buffers, returning\n");
				retval = -EAGAIN;
				break;
			}
			if ((vout->v4l2_bufs[idx].flags & V4L2_BUF_FLAG_DONE) ==
			    0)
				dev_dbg(&vdev->dev,
					"VIDIOC_DQBUF: buffer in done q, but not "
					"flagged as done\n");

			vout->v4l2_bufs[idx].flags = 0;
			memcpy(buf, &(vout->v4l2_bufs[idx]), sizeof(*buf));
			dev_dbg(&vdev->dev, "VIDIOC_DQBUF: %d\n", buf->index);
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

			/* only full screen supported for SDC BG and SDC DC */
			if (vout->cur_disp_output == 4 ||
			    vout->cur_disp_output == 5) {
				crop->c = vout->crop_current;
				break;
			}

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

			vout->crop_current = crop->c;
			break;
		}
	case VIDIOC_ENUMOUTPUT:
		{
			struct v4l2_output *output = arg;

			if ((output->index >= 5) ||
			    (vout->output_enabled[output->index] == false)) {
				retval = -EINVAL;
				break;
			}

			if (output->index < 3) {
				*output = mxc_outputs[MXC_V4L2_OUT_2_ADC];
				output->name[4] = '0' + output->index;
			} else {
				*output = mxc_outputs[MXC_V4L2_OUT_2_SDC];
			}
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
			int fbnum;
			struct v4l2_rect *b;

			if ((*p_output_num >= MXC_V4L2_OUT_NUM_OUTPUTS) ||
			    (vout->output_enabled[*p_output_num] == false)) {
				retval = -EINVAL;
				break;
			}

			if (vout->state != STATE_STREAM_OFF) {
				retval = -EBUSY;
				break;
			}

			vout->cur_disp_output = *p_output_num;

			/* Update bounds in case they have changed */
			b = &vout->crop_bounds[vout->cur_disp_output];

			fbnum = vout->output_fb_num[vout->cur_disp_output];

			/*
			 * For FG overlay, it uses BG window parameter as
			 * limitation reference; and BG must be enabled to
			 * support FG.
			 */
			if (vout->cur_disp_output == 3) {
				unsigned int i, ipu_ch = CHAN_NONE;
				struct fb_info *fbi;
				mm_segment_t old_fs;

				for (i = 0; i < num_registered_fb; i++) {
					fbi = registered_fb[i];
					if (fbi->fbops->fb_ioctl) {
						old_fs = get_fs();
						set_fs(KERNEL_DS);
						fbi->fbops->fb_ioctl(fbi,
							MXCFB_GET_FB_IPU_CHAN,
							(unsigned long)&ipu_ch);
						set_fs(old_fs);
					}
					if (ipu_ch == CHAN_NONE) {
						dev_err(&vdev->dev,
						"Can't get disp ipu channel\n");
						retval = -EINVAL;
						break;
					}

					if (ipu_ch == MEM_BG_SYNC) {
						fbnum = i;
						break;
					}
				}
			}

			b->width = registered_fb[fbnum]->var.xres;
			b->height = registered_fb[fbnum]->var.yres;

			vout->crop_current = *b;
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
	struct video_device *vdev = video_devdata(file);
	unsigned long size = vma->vm_end - vma->vm_start;
	int res = 0;
	int i;
	vout_data *vout = video_get_drvdata(vdev);

	dev_dbg(&vdev->dev, "pgoff=0x%lx, start=0x%lx, end=0x%lx\n",
		vma->vm_pgoff, vma->vm_start, vma->vm_end);

	/* make this _really_ smp-safe */
	if (down_interruptible(&vout->busy_lock))
		return -EINTR;

	for (i = 0; i < vout->buffer_cnt; i++) {
		if ((vout->v4l2_bufs[i].m.offset ==
		     (vma->vm_pgoff << PAGE_SHIFT)) &&
		    (vout->v4l2_bufs[i].length >= size)) {
			vout->v4l2_bufs[i].flags |= V4L2_BUF_FLAG_MAPPED;
			break;
		}
	}
	if (i == vout->buffer_cnt) {
		res = -ENOBUFS;
		goto mxc_mmap_exit;
	}

	/* make buffers inner write-back, outer write-thru cacheable */
	/* vma->vm_page_prot = pgprot_outer_wrthru(vma->vm_page_prot);*/

	if (remap_pfn_range(vma, vma->vm_start,
			    vma->vm_pgoff, size, vma->vm_page_prot)) {
		dev_dbg(&vdev->dev, "mmap remap_pfn_range failed\n");
		res = -ENOBUFS;
		goto mxc_mmap_exit;
	}

	vma->vm_flags &= ~VM_IO;	/* using shared anonymous pages */

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
	struct video_device *dev = video_devdata(file);
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

static struct
file_operations mxc_v4l2out_fops = {
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
		dev_dbg(&pdev->dev, "video_register_device failed\n");
		return 0;
	}
	dev_info(&pdev->dev, "Registered device video%d\n",
		 vout->video_dev->minor & 0x1f);
	/*vout->video_dev->dev = &pdev->dev;*/

	video_set_drvdata(vout->video_dev, vout);

	init_MUTEX(&vout->param_lock);
	init_MUTEX(&vout->busy_lock);

	/* setup outputs and cropping */
	vout->cur_disp_output = -1;
	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;
		if (strncmp(idstr, "DISP", 4) == 0) {
			int disp_num = idstr[4] - '0';
			if (disp_num == 3) {
				if (strcmp(idstr, "DISP3 BG - DI1") == 0)
					disp_num = 5;
				else if (strncmp(idstr, "DISP3 BG", 8) == 0)
					disp_num = 4;
			}
			vout->crop_bounds[disp_num].left = 0;
			vout->crop_bounds[disp_num].top = 0;
			vout->crop_bounds[disp_num].width =
			    registered_fb[i]->var.xres;
			vout->crop_bounds[disp_num].height =
			    registered_fb[i]->var.yres;
			vout->output_enabled[disp_num] = true;
			vout->output_fb_num[disp_num] = i;
			if (vout->cur_disp_output == -1) {
				vout->cur_disp_output = disp_num;
			}
		}

	}
	vout->crop_current = vout->crop_bounds[vout->cur_disp_output];

	platform_set_drvdata(pdev, vout);

	return 0;
}

static int mxc_v4l2out_remove(struct platform_device *pdev)
{
	vout_data *vout = platform_get_drvdata(pdev);

	if (vout->video_dev) {
		if (-1 != vout->video_dev->minor)
			video_unregister_device(vout->video_dev);
		else
			video_device_release(vout->video_dev);
		vout->video_dev = NULL;
	}

	platform_set_drvdata(pdev, NULL);

	kfree(vout);

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_v4l2out_driver = {
	.driver = {
		   .name = "MXC Video Output",
		   },
	.probe = mxc_v4l2out_probe,
	.remove = mxc_v4l2out_remove,
};

static struct platform_device mxc_v4l2out_device = {
	.name = "MXC Video Output",
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
