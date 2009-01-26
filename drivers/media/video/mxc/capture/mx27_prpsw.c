/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_prpsw.c
 *
 * @brief MX27 Video For Linux 2 capture driver
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/fb.h>
#include <linux/pci.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "mxc_v4l2_capture.h"
#include "mx27_prp.h"
#include "mx27_csi.h"
#include "../drivers/video/mxc/mx2fb.h"
#include "../opl/opl.h"

#define MEAN_COEF	(SZ_COEF >> 1)

static char prp_dev[] = "emma_prp";
static int g_still_on = 0;
static emma_prp_cfg g_prp_cfg;
static int g_vfbuf, g_rotbuf;
static struct tasklet_struct prp_vf_tasklet;

/*
 * The following variables represents the virtual address for the cacheable
 * buffers accessed by SW rotation/mirroring. The rotation/mirroring in
 * cacheable buffers has significant performance improvement than it in
 * non-cacheable buffers.
 */
static char *g_vaddr_vfbuf[2] = { 0, 0 };
static char *g_vaddr_rotbuf[2] = { 0, 0 };
static char *g_vaddr_fb = 0;

static int set_ch1_addr(emma_prp_cfg * cfg, cam_data * cam);
static int prp_v4l2_cfg(emma_prp_cfg * cfg, cam_data * cam);
static int prp_vf_mem_alloc(cam_data * cam);
static void prp_vf_mem_free(cam_data * cam);
static int prp_rot_mem_alloc(cam_data * cam);
static void prp_rot_mem_free(cam_data * cam);
static int prp_enc_update_eba(u32 eba, int *buffer_num);
static int prp_enc_enable(void *private);
static int prp_enc_disable(void *private);
static int prp_vf_start(void *private);
static int prp_vf_stop(void *private);
static int prp_still_start(void *private);
static int prp_still_stop(void *private);
static irqreturn_t prp_isr(int irq, void *dev_id);
static void rotation(unsigned long private);
static int prp_resize_check_ch1(emma_prp_cfg * cfg);
static int prp_resize_check_ch2(emma_prp_cfg * cfg);

#define PRP_DUMP(val)	pr_debug("%s\t = 0x%08X\t%d\n", #val, val, val)

/*!
 * @brief Dump PrP configuration parameters.
 * @param cfg	The pointer to PrP configuration parameter
 */
static void prp_cfg_dump(emma_prp_cfg * cfg)
{
	PRP_DUMP(cfg->in_pix);
	PRP_DUMP(cfg->in_width);
	PRP_DUMP(cfg->in_height);
	PRP_DUMP(cfg->in_csi);
	PRP_DUMP(cfg->in_line_stride);
	PRP_DUMP(cfg->in_line_skip);
	PRP_DUMP(cfg->in_ptr);

	PRP_DUMP(cfg->ch1_pix);
	PRP_DUMP(cfg->ch1_width);
	PRP_DUMP(cfg->ch1_height);
	PRP_DUMP(cfg->ch1_scale.algo);
	PRP_DUMP(cfg->ch1_scale.width.num);
	PRP_DUMP(cfg->ch1_scale.width.den);
	PRP_DUMP(cfg->ch1_scale.height.num);
	PRP_DUMP(cfg->ch1_scale.height.den);
	PRP_DUMP(cfg->ch1_stride);
	PRP_DUMP(cfg->ch1_ptr);
	PRP_DUMP(cfg->ch1_ptr2);
	PRP_DUMP(cfg->ch1_csi);

	PRP_DUMP(cfg->ch2_pix);
	PRP_DUMP(cfg->ch2_width);
	PRP_DUMP(cfg->ch2_height);
	PRP_DUMP(cfg->ch2_scale.algo);
	PRP_DUMP(cfg->ch2_scale.width.num);
	PRP_DUMP(cfg->ch2_scale.width.den);
	PRP_DUMP(cfg->ch2_scale.height.num);
	PRP_DUMP(cfg->ch2_scale.height.den);
	PRP_DUMP(cfg->ch2_ptr);
	PRP_DUMP(cfg->ch2_ptr2);
	PRP_DUMP(cfg->ch2_csi);
}

/*!
 * @brief Set PrP channel 1 output address.
 * @param cfg	Pointer to emma_prp_cfg structure
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static int set_ch1_addr(emma_prp_cfg * cfg, cam_data * cam)
{
	if (cam->rotation != V4L2_MXC_ROTATE_NONE) {
		cfg->ch1_ptr = (unsigned int)cam->rot_vf_bufs[0];
		cfg->ch1_ptr2 = (unsigned int)cam->rot_vf_bufs[1];
		if ((cam->rotation == V4L2_MXC_ROTATE_90_RIGHT)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_VFLIP)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_HFLIP)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_LEFT))
			cfg->ch1_stride = cam->win.w.height;
		else
			cfg->ch1_stride = cam->win.w.width;

		if (cam->v4l2_fb.flags != V4L2_FBUF_FLAG_OVERLAY) {
			struct fb_info *fb = cam->overlay_fb;
			if (!fb)
				return -1;
			if (g_vaddr_fb)
				iounmap(g_vaddr_fb);
			g_vaddr_fb = ioremap_cached(fb->fix.smem_start,
						    fb->fix.smem_len);
			if (!g_vaddr_fb)
				return -1;
		}
	} else if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		cfg->ch1_ptr = (unsigned int)cam->vf_bufs[0];
		cfg->ch1_ptr2 = (unsigned int)cam->vf_bufs[1];
		cfg->ch1_stride = cam->win.w.width;
	} else {
		struct fb_info *fb = cam->overlay_fb;

		if (!fb)
			return -1;

		cfg->ch1_ptr = fb->fix.smem_start;
		cfg->ch1_ptr += cam->win.w.top * fb->var.xres_virtual
		    * (fb->var.bits_per_pixel >> 3)
		    + cam->win.w.left * (fb->var.bits_per_pixel >> 3);
		cfg->ch1_ptr2 = cfg->ch1_ptr;
		cfg->ch1_stride = fb->var.xres_virtual;
	}

	return 0;
}

/*!
 * @brief Setup PrP configuration parameters.
 * @param cfg	Pointer to emma_prp_cfg structure
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static int prp_v4l2_cfg(emma_prp_cfg * cfg, cam_data * cam)
{
	cfg->in_pix = PRP_PIXIN_YUYV;
	cfg->in_width = cam->crop_current.width;
	cfg->in_height = cam->crop_current.height;
	cfg->in_line_stride = cam->crop_current.left;
	cfg->in_line_skip = cam->crop_current.top;
	cfg->in_ptr = 0;
	cfg->in_csi = PRP_CSI_LOOP;
	memset(cfg->in_csc, 0, sizeof(cfg->in_csc));

	if (cam->overlay_on) {
		/* Convert V4L2 pixel format to PrP pixel format */
		switch (cam->v4l2_fb.fmt.pixelformat) {
		case V4L2_PIX_FMT_RGB332:
			cfg->ch1_pix = PRP_PIX1_RGB332;
			break;
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_BGR32:
			cfg->ch1_pix = PRP_PIX1_RGB888;
			break;
		case V4L2_PIX_FMT_YUYV:
			cfg->ch1_pix = PRP_PIX1_YUYV;
			break;
		case V4L2_PIX_FMT_UYVY:
			cfg->ch1_pix = PRP_PIX1_UYVY;
			break;
		case V4L2_PIX_FMT_RGB565:
		default:
			cfg->ch1_pix = PRP_PIX1_RGB565;
			break;
		}
		if ((cam->rotation == V4L2_MXC_ROTATE_90_RIGHT)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_VFLIP)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_HFLIP)
		    || (cam->rotation == V4L2_MXC_ROTATE_90_LEFT)) {
			cfg->ch1_width = cam->win.w.height;
			cfg->ch1_height = cam->win.w.width;
		} else {
			cfg->ch1_width = cam->win.w.width;
			cfg->ch1_height = cam->win.w.height;
		}

		if (set_ch1_addr(cfg, cam))
			return -1;
	} else {
		cfg->ch1_pix = PRP_PIX1_UNUSED;
		cfg->ch1_width = cfg->in_width;
		cfg->ch1_height = cfg->in_height;
	}
	cfg->ch1_scale.algo = 0;
	cfg->ch1_scale.width.num = cfg->in_width;
	cfg->ch1_scale.width.den = cfg->ch1_width;
	cfg->ch1_scale.height.num = cfg->in_height;
	cfg->ch1_scale.height.den = cfg->ch1_height;
	cfg->ch1_csi = PRP_CSI_EN;

	if (cam->capture_on || g_still_on) {
		switch (cam->v2f.fmt.pix.pixelformat) {
		case V4L2_PIX_FMT_YUYV:
			cfg->ch2_pix = PRP_PIX2_YUV422;
			break;
		case V4L2_PIX_FMT_YUV420:
			cfg->ch2_pix = PRP_PIX2_YUV420;
			break;
			/*
			 * YUV444 is not defined by V4L2.
			 * We support it in default case.
			 */
		default:
			cfg->ch2_pix = PRP_PIX2_YUV444;
			break;
		}
		cfg->ch2_width = cam->v2f.fmt.pix.width;
		cfg->ch2_height = cam->v2f.fmt.pix.height;
	} else {
		cfg->ch2_pix = PRP_PIX2_UNUSED;
		cfg->ch2_width = cfg->in_width;
		cfg->ch2_height = cfg->in_height;
	}
	cfg->ch2_scale.algo = 0;
	cfg->ch2_scale.width.num = cfg->in_width;
	cfg->ch2_scale.width.den = cfg->ch2_width;
	cfg->ch2_scale.height.num = cfg->in_height;
	cfg->ch2_scale.height.den = cfg->ch2_height;
	cfg->ch2_csi = PRP_CSI_EN;

	memset(cfg->scale, 0, sizeof(cfg->scale));
	cfg->scale[0].algo = cfg->ch1_scale.algo & 3;
	cfg->scale[1].algo = (cfg->ch1_scale.algo >> 2) & 3;
	cfg->scale[2].algo = cfg->ch2_scale.algo & 3;
	cfg->scale[3].algo = (cfg->ch2_scale.algo >> 2) & 3;

	prp_cfg_dump(cfg);

	if (prp_resize_check_ch2(cfg))
		return -1;

	if (prp_resize_check_ch1(cfg))
		return -1;

	return 0;
}

/*!
 * @brief PrP interrupt handler
 */
static irqreturn_t prp_isr(int irq, void *dev_id)
{
	int status;
	cam_data *cam = (cam_data *) dev_id;

	status = prphw_isr();

	if (g_still_on && (status & PRP_INTRSTAT_CH2BUF1)) {
		prp_still_stop(cam);
		cam->still_counter++;
		wake_up_interruptible(&cam->still_queue);
		/*
		 * Still & video capture use the same PrP channel 2.
		 * They are execlusive.
		 */
	} else if (cam->capture_on) {
		if (status & (PRP_INTRSTAT_CH2BUF1 | PRP_INTRSTAT_CH2BUF2)) {
			cam->enc_callback(0, cam);
		}
	}
	if (cam->overlay_on
	    && (status & (PRP_INTRSTAT_CH1BUF1 | PRP_INTRSTAT_CH1BUF2))) {
		if (cam->rotation != V4L2_MXC_ROTATE_NONE) {
			g_rotbuf = (status & PRP_INTRSTAT_CH1BUF1) ? 0 : 1;
			tasklet_schedule(&prp_vf_tasklet);
		} else if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
			struct fb_gwinfo gwinfo;

			gwinfo.enabled = 1;
			gwinfo.alpha_value = 255;
			gwinfo.ck_enabled = 0;
			gwinfo.xpos = cam->win.w.left;
			gwinfo.ypos = cam->win.w.top;
			gwinfo.xres = cam->win.w.width;
			gwinfo.yres = cam->win.w.height;
			gwinfo.xres_virtual = cam->win.w.width;
			gwinfo.vs_reversed = 0;
			if (status & PRP_INTRSTAT_CH1BUF1)
				gwinfo.base = (unsigned long)cam->vf_bufs[0];
			else
				gwinfo.base = (unsigned long)cam->vf_bufs[1];

			mx2_gw_set(&gwinfo);
		}
	}

	return IRQ_HANDLED;
}

/*!
 * @brief PrP initialization.
 * @param dev_id	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_init(void *dev_id)
{
	enable_irq(MXC_INT_EMMAPRP);
	if (request_irq(MXC_INT_EMMAPRP, prp_isr, 0, prp_dev, dev_id))
		return -1;
	prphw_init();

	return 0;
}

/*!
 * @brief PrP initialization.
 * @param dev_id	Pointer to cam_data structure
 */
void prp_exit(void *dev_id)
{
	prphw_exit();
	disable_irq(MXC_INT_EMMAPRP);
	free_irq(MXC_INT_EMMAPRP, dev_id);
}

/*!
 * @brief Update PrP channel 2 output buffer address.
 * @param eba		Physical address for PrP output buffer
 * @param buffer_num	The PrP channel 2 buffer number to be updated
 * @return		Zero on success, others on failure
 */
static int prp_enc_update_eba(u32 eba, int *buffer_num)
{
	if (*buffer_num) {
		g_prp_cfg.ch2_ptr2 = eba;
		prphw_ch2ptr2(&g_prp_cfg);
		*buffer_num = 0;
	} else {
		g_prp_cfg.ch2_ptr = eba;
		prphw_ch2ptr(&g_prp_cfg);
		*buffer_num = 1;
	}

	return 0;
}

/*!
 * @brief Enable PrP for encoding.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_enc_enable(void *private)
{
	cam_data *cam = (cam_data *) private;

	if (prp_v4l2_cfg(&g_prp_cfg, cam))
		return -1;

	csi_enable_mclk(CSI_MCLK_ENC, true, true);
	prphw_reset();

	if (prphw_cfg(&g_prp_cfg))
		return -1;

	prphw_enable(cam->overlay_on ? (PRP_CHANNEL_1 | PRP_CHANNEL_2)
		     : PRP_CHANNEL_2);

	return 0;
}

/*!
 * @brief Disable PrP for encoding.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_enc_disable(void *private)
{
	prphw_disable(PRP_CHANNEL_2);
	csi_enable_mclk(CSI_MCLK_ENC, false, false);

	return 0;
}

/*!
 * @brief Setup encoding functions.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_enc_select(void *private)
{
	int ret = 0;
	cam_data *cam = (cam_data *) private;

	if (cam) {
		cam->enc_update_eba = prp_enc_update_eba;
		cam->enc_enable = prp_enc_enable;
		cam->enc_disable = prp_enc_disable;
	} else
		ret = -EIO;

	return ret;
}

/*!
 * @brief Uninstall encoding functions.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_enc_deselect(void *private)
{
	int ret = 0;
	cam_data *cam = (cam_data *) private;

	ret = prp_enc_disable(private);

	if (cam) {
		cam->enc_update_eba = NULL;
		cam->enc_enable = NULL;
		cam->enc_disable = NULL;
	}

	return ret;
}

/*!
 * @brief Allocate memory for overlay.
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static int prp_vf_mem_alloc(cam_data * cam)
{
	int i;

	for (i = 0; i < 2; i++) {
		cam->vf_bufs_size[i] = cam->win.w.width * cam->win.w.height * 2;
		cam->vf_bufs_vaddr[i] = dma_alloc_coherent(0,
							   cam->vf_bufs_size[i],
							   &cam->vf_bufs[i],
							   GFP_DMA |
							   GFP_KERNEL);
		if (!cam->vf_bufs_vaddr[i]) {
			pr_debug("Failed to alloc memory for vf.\n");
			prp_vf_mem_free(cam);
			return -1;
		}

		g_vaddr_vfbuf[i] =
		    ioremap_cached(cam->vf_bufs[i], cam->vf_bufs_size[i]);
		if (!g_vaddr_vfbuf[i]) {
			pr_debug("Failed to ioremap_cached() for vf.\n");
			prp_vf_mem_free(cam);
			return -1;
		}
	}

	return 0;
}

/*!
 * @brief Free memory for overlay.
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static void prp_vf_mem_free(cam_data * cam)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (cam->vf_bufs_vaddr[i]) {
			dma_free_coherent(0,
					  cam->vf_bufs_size[i],
					  cam->vf_bufs_vaddr[i],
					  cam->vf_bufs[i]);
		}
		cam->vf_bufs[i] = 0;
		cam->vf_bufs_vaddr[i] = 0;
		cam->vf_bufs_size[i] = 0;
		if (g_vaddr_vfbuf[i]) {
			iounmap(g_vaddr_vfbuf[i]);
			g_vaddr_vfbuf[i] = 0;
		}
	}
}

/*!
 * @brief Allocate intermediate memory for overlay rotation/mirroring.
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static int prp_rot_mem_alloc(cam_data * cam)
{
	int i;

	for (i = 0; i < 2; i++) {
		cam->rot_vf_buf_size[i] =
		    cam->win.w.width * cam->win.w.height * 2;
		cam->rot_vf_bufs_vaddr[i] =
		    dma_alloc_coherent(0, cam->rot_vf_buf_size[i],
				       &cam->rot_vf_bufs[i],
				       GFP_DMA | GFP_KERNEL);
		if (!cam->rot_vf_bufs_vaddr[i]) {
			pr_debug("Failed to alloc memory for vf rotation.\n");
			prp_rot_mem_free(cam);
			return -1;
		}

		g_vaddr_rotbuf[i] =
		    ioremap_cached(cam->rot_vf_bufs[i],
				   cam->rot_vf_buf_size[i]);
		if (!g_vaddr_rotbuf[i]) {
			pr_debug
			    ("Failed to ioremap_cached() for rotation buffer.\n");
			prp_rot_mem_free(cam);
			return -1;
		}
	}

	return 0;
}

/*!
 * @brief Free intermedaite memory for overlay rotation/mirroring.
 * @param cam	Pointer to cam_data structure
 * @return	Zero on success, others on failure
 */
static void prp_rot_mem_free(cam_data * cam)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (cam->rot_vf_bufs_vaddr[i]) {
			dma_free_coherent(0,
					  cam->rot_vf_buf_size[i],
					  cam->rot_vf_bufs_vaddr[i],
					  cam->rot_vf_bufs[i]);
		}
		cam->rot_vf_bufs[i] = 0;
		cam->rot_vf_bufs_vaddr[i] = 0;
		cam->rot_vf_buf_size[i] = 0;
		if (g_vaddr_rotbuf[i]) {
			iounmap(g_vaddr_rotbuf[i]);
			g_vaddr_rotbuf[i] = 0;
		}
	}
}

/*!
 * @brief Start overlay (view finder).
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_vf_start(void *private)
{
	cam_data *cam = (cam_data *) private;

	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		prp_vf_mem_free(cam);
		if (prp_vf_mem_alloc(cam)) {
			pr_info("Error to allocate vf buffer\n");
			return -ENOMEM;
		}
	}

	if (cam->rotation != V4L2_MXC_ROTATE_NONE) {
		prp_rot_mem_free(cam);
		if (prp_rot_mem_alloc(cam)) {
			pr_info("Error to allocate rotation buffer\n");
			prp_vf_mem_free(cam);
			return -ENOMEM;
		}
	}

	if (prp_v4l2_cfg(&g_prp_cfg, cam)) {
		prp_vf_mem_free(cam);
		prp_rot_mem_free(cam);
		return -1;
	}

	csi_enable_mclk(CSI_MCLK_VF, true, true);
	prphw_reset();

	if (prphw_cfg(&g_prp_cfg)) {
		prp_vf_mem_free(cam);
		prp_rot_mem_free(cam);
		return -1;
	}
	g_vfbuf = g_rotbuf = 0;
	tasklet_init(&prp_vf_tasklet, rotation, (unsigned long)private);

	prphw_enable(cam->capture_on ? (PRP_CHANNEL_1 | PRP_CHANNEL_2)
		     : PRP_CHANNEL_1);

	return 0;
}

/*!
 * @brief Stop overlay (view finder).
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_vf_stop(void *private)
{
	cam_data *cam = (cam_data *) private;

	prphw_disable(PRP_CHANNEL_1);

	csi_enable_mclk(CSI_MCLK_VF, false, false);
	tasklet_kill(&prp_vf_tasklet);

	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		/* Disable graphic window */
		gwinfo.enabled = 0;
		mx2_gw_set(&gwinfo);

		prp_vf_mem_free(cam);
	}
	prp_rot_mem_free(cam);
	if (g_vaddr_fb) {
		iounmap(g_vaddr_fb);
		g_vaddr_fb = 0;
	}

	return 0;
}

/*!
 * @brief Setup overlay functions.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_vf_select(void *private)
{
	int ret = 0;
	cam_data *cam = (cam_data *) private;

	if (cam) {
		cam->vf_start_sdc = prp_vf_start;
		cam->vf_stop_sdc = prp_vf_stop;
		cam->overlay_active = false;
	} else
		ret = -EIO;

	return ret;
}

/*!
 * @brief Uninstall overlay functions.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_vf_deselect(void *private)
{
	int ret = 0;
	cam_data *cam = (cam_data *) private;

	ret = prp_vf_stop(private);

	if (cam) {
		cam->vf_start_sdc = NULL;
		cam->vf_stop_sdc = NULL;
	}

	return ret;
}

/*!
 * @brief Start still picture capture.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_still_start(void *private)
{
	cam_data *cam = (cam_data *) private;

	g_still_on = 1;
	g_prp_cfg.ch2_ptr = (unsigned int)cam->still_buf;
	g_prp_cfg.ch2_ptr2 = 0;

	if (prp_v4l2_cfg(&g_prp_cfg, cam))
		return -1;

	csi_enable_mclk(CSI_MCLK_RAW, true, true);
	prphw_reset();

	if (prphw_cfg(&g_prp_cfg)) {
		g_still_on = 0;
		return -1;
	}

	prphw_enable(cam->overlay_on ? (PRP_CHANNEL_1 | PRP_CHANNEL_2)
		     : PRP_CHANNEL_2);

	return 0;
}

/*!
 * @brief Stop still picture capture.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
static int prp_still_stop(void *private)
{
	prphw_disable(PRP_CHANNEL_2);

	csi_enable_mclk(CSI_MCLK_RAW, false, false);

	g_still_on = 0;

	return 0;
}

/*!
 * @brief Setup functions for still picture capture.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_still_select(void *private)
{
	cam_data *cam = (cam_data *) private;

	if (cam) {
		cam->csi_start = prp_still_start;
		cam->csi_stop = prp_still_stop;
	}

	return 0;
}

/*!
 * @brief Uninstall functions for still picture capture.
 * @param private	Pointer to cam_data structure
 * @return		Zero on success, others on failure
 */
int prp_still_deselect(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	err = prp_still_stop(cam);

	if (cam) {
		cam->csi_start = NULL;
		cam->csi_stop = NULL;
	}

	return err;
}

/*!
 * @brief Perform software rotation or mirroring
 * @param private	Argument passed to the tasklet
 */
static void rotation(unsigned long private)
{
	char *src, *dst;
	int width, height, s_stride, d_stride;
	int size;
	cam_data *cam = (cam_data *) private;

	src = g_vaddr_rotbuf[g_rotbuf];
	size = cam->rot_vf_buf_size[g_rotbuf];

	if ((cam->rotation == V4L2_MXC_ROTATE_90_RIGHT)
	    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_VFLIP)
	    || (cam->rotation == V4L2_MXC_ROTATE_90_RIGHT_HFLIP)
	    || (cam->rotation == V4L2_MXC_ROTATE_90_LEFT)) {
		width = cam->win.w.height;
		height = cam->win.w.width;
		s_stride = cam->win.w.height << 1;
	} else {
		width = cam->win.w.width;
		height = cam->win.w.height;
		s_stride = cam->win.w.width << 1;
	}

	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		dst = g_vaddr_vfbuf[g_vfbuf];
		d_stride = cam->win.w.width << 1;
	} else {		/* The destination is the framebuffer */
		struct fb_info *fb = cam->overlay_fb;
		if (!fb)
			return;
		dst = g_vaddr_fb;
		dst += cam->win.w.top * fb->var.xres_virtual
		    * (fb->var.bits_per_pixel >> 3)
		    + cam->win.w.left * (fb->var.bits_per_pixel >> 3);
		d_stride = fb->var.xres_virtual << 1;
	}

	/*
	 * Invalidate the data in cache before performing the SW rotaion
	 * or mirroring in case the image size is less than QVGA. For image
	 * larger than QVGA it is not invalidated becase the invalidation
	 * will consume much time while we don't see any artifacts on the
	 * output if we don't perform invalidation for them.
	 * Similarly we don't flush the data after SW rotation/mirroring.
	 */
	if (size < 320 * 240 * 2)
		dmac_inv_range(src, src + size);
	switch (cam->rotation) {
	case V4L2_MXC_ROTATE_VERT_FLIP:
		opl_vmirror_u16(src, s_stride, width, height, dst, d_stride);
		break;
	case V4L2_MXC_ROTATE_HORIZ_FLIP:
		opl_hmirror_u16(src, s_stride, width, height, dst, d_stride);
		break;
	case V4L2_MXC_ROTATE_180:
		opl_rotate180_u16(src, s_stride, width, height, dst, d_stride);
		break;
	case V4L2_MXC_ROTATE_90_RIGHT:
		opl_rotate90_u16(src, s_stride, width, height, dst, d_stride);
		break;
	case V4L2_MXC_ROTATE_90_RIGHT_VFLIP:
		opl_rotate90_vmirror_u16(src, s_stride, width, height, dst,
					 d_stride);
		break;
	case V4L2_MXC_ROTATE_90_RIGHT_HFLIP:
		/* ROTATE_90_RIGHT_HFLIP = ROTATE_270_RIGHT_VFLIP */
		opl_rotate270_vmirror_u16(src, s_stride, width, height, dst,
					  d_stride);
		break;
	case V4L2_MXC_ROTATE_90_LEFT:
		opl_rotate270_u16(src, s_stride, width, height, dst, d_stride);
		break;
	default:
		return;
	}

	/* Config and display the graphic window */
	if (cam->v4l2_fb.flags == V4L2_FBUF_FLAG_OVERLAY) {
		struct fb_gwinfo gwinfo;

		gwinfo.enabled = 1;
		gwinfo.alpha_value = 255;
		gwinfo.ck_enabled = 0;
		gwinfo.xpos = cam->win.w.left;
		gwinfo.ypos = cam->win.w.top;
		gwinfo.xres = cam->win.w.width;
		gwinfo.yres = cam->win.w.height;
		gwinfo.xres_virtual = cam->win.w.width;
		gwinfo.vs_reversed = 0;
		gwinfo.base = (unsigned long)cam->vf_bufs[g_vfbuf];
		mx2_gw_set(&gwinfo);

		g_vfbuf = g_vfbuf ? 0 : 1;
	}
}

/*
 * @brief Check if the resize ratio is supported based on the input and output
 *        dimension
 * @param	input	input dimension
 * @param	output	output dimension
 * @return	output dimension (should equal the parameter *output*)
 * 		-1 on failure
 */
static int check_simple(scale_t * scale, int input, int output)
{
	unsigned short int_out;	/* PrP internel width or height */
	unsigned short orig_out = output;

	if (prp_scale(scale, input, output, input, &orig_out, &int_out, 0))
		return -1;	/* resize failed */
	else
		return int_out;
}

/*
 * @brief Check if the resize ratio is supported based on the input and output
 *        dimension
 * @param	input	input dimension
 * @param	output	output dimension
 * @return	output dimension, may be rounded.
 * 		-1 on failure
 */
static int check_simple_retry(scale_t * scale, int input, int output)
{
	unsigned short int_out;	/* PrP internel width or height */
	unsigned short orig_out = output;

	if (prp_scale(scale, input, output, input, &orig_out, &int_out,
		      SCALE_RETRY))
		return -1;	/* resize failed */
	else
		return int_out;
}

/*!
 * @brief Check if the resize ratio is supported by PrP channel 1
 * @param cfg	Pointer to emma_prp_cfg structure
 * @return	Zero on success, others on failure
 */
static int prp_resize_check_ch1(emma_prp_cfg * cfg)
{
	int in_w, in_h, ch1_w, ch1_h, ch2_w, ch2_h, w, h;
	scale_t *pscale = &cfg->scale[0];	/* Ch1 width resize coeff */

	if (cfg->ch1_pix == PRP_PIX1_UNUSED)
		return 0;

	in_w = cfg->in_width;
	in_h = cfg->in_height;
	ch1_w = cfg->ch1_width;
	ch1_h = cfg->ch1_height;
	ch2_w = cfg->ch2_width;
	ch2_h = cfg->ch2_height;

	/*
	 * For channel 1, try parallel resize first. If the resize
	 * ratio is not exactly supported, try cascade resize. If it
	 * still fails, use parallel resize but with rounded value.
	 */
	w = check_simple(pscale, in_w, ch1_w);
	h = check_simple(pscale + 1, in_h, ch1_h);
	if ((w == ch1_w) && (h == ch1_h))
		goto exit_parallel;

	if (cfg->ch2_pix != PRP_PIX2_UNUSED) {
		/*
		 * Channel 2 is already used. The pscale is still pointing
		 * to ch1 resize coeff for temporary use.
		 */
		w = check_simple(pscale, in_w, ch2_w);
		h = check_simple(pscale + 1, in_h, ch2_h);
		if ((w == ch2_w) && (h == ch2_h)) {
			/* Try cascade resize now */
			w = check_simple(pscale, ch2_w, ch1_w);
			h = check_simple(pscale + 1, ch2_h, ch1_h);
			if ((w == ch1_w) && (h == ch1_h))
				goto exit_cascade;
		}
	} else {
		/*
		 * Try cascade resize for width, width is multiple of 2.
		 * Channel 2 is not used. So we have more values to pick
		 * for channel 2 resize.
		 */
		for (w = in_w - 2; w > ch1_w; w -= 2) {
			/* Ch2 width resize */
			if (check_simple(pscale + 2, in_w, w) != w)
				continue;
			/* Ch1 width resize */
			if (check_simple(pscale, w, ch1_w) != ch1_w)
				continue;
			break;
		}
		if ((ch2_w = w) > ch1_w) {
			/* try cascade resize for height */
			for (h = in_h - 1; h > ch1_h; h--) {
				/* Ch2 height resize */
				if (check_simple(pscale + 3, in_h, h) != h)
					continue;
				/* Ch1 height resize */
				if (check_simple(pscale + 1, h, ch1_h) != ch1_h)
					continue;
				break;
			}
			if ((ch2_h = h) > ch1_h)
				goto exit_cascade;
		}
	}

	/* Have to try parallel resize again and round the dimensions */
	w = check_simple_retry(pscale, in_w, ch1_w);
	h = check_simple_retry(pscale + 1, in_h, ch1_h);
	if ((w != -1) && (h != -1))
		goto exit_parallel;

	pr_debug("Ch1 resize error.\n");
	return -1;

      exit_parallel:
	cfg->ch1_scale.algo |= PRP_ALGO_BYPASS;
	pr_debug("ch1 parallel resize.\n");
	pr_debug("original width = %d internel width = %d\n", ch1_w, w);
	pr_debug("original height = %d internel height = %d\n", ch1_h, h);
	return 0;

      exit_cascade:
	cfg->ch1_scale.algo &= ~PRP_ALGO_BYPASS;
	pr_debug("ch1 cascade resize.\n");
	pr_debug("[width] in : ch2 : ch1=%d : %d : %d\n", in_w, ch2_w, ch1_w);
	pr_debug("[height] in : ch2 : ch1=%d : %d : %d\n", in_h, ch2_h, ch1_h);
	return 0;
}

/*!
 * @brief Check if the resize ratio is supported by PrP channel 2
 * @param cfg	Pointer to emma_prp_cfg structure
 * @return	Zero on success, others on failure
 */
static int prp_resize_check_ch2(emma_prp_cfg * cfg)
{
	int w, h;
	scale_t *pscale = &cfg->scale[2];	/* Ch2 width resize coeff */

	if (cfg->ch2_pix == PRP_PIX2_UNUSED)
		return 0;

	w = check_simple_retry(pscale, cfg->in_width, cfg->ch2_width);
	h = check_simple_retry(pscale + 1, cfg->in_height, cfg->ch2_height);
	if ((w != -1) && (h != -1)) {
		pr_debug("Ch2 resize.\n");
		pr_debug("Original width = %d internel width = %d\n",
			 cfg->ch2_width, w);
		pr_debug("Original height = %d internel height = %d\n",
			 cfg->ch2_height, h);
		return 0;
	} else {
		pr_debug("Ch2 resize error.\n");
		return -1;
	}
}
