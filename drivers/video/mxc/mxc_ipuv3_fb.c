/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb.c
 *
 * @brief MXC Frame buffer driver for SDC
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/uaccess.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>

/*
 * Driver name
 */
#define MXCFB_NAME      "mxc_sdc_fb"

/* Display port number */
#define MXCFB_PORT_NUM	2
/*!
 * Structure containing the MXC specific framebuffer information.
 */
struct mxcfb_info {
	char *fb_mode_str;
	int default_bpp;
	int cur_blank;
	int next_blank;
	ipu_channel_t ipu_ch;
	int ipu_di;
	u32 ipu_di_pix_fmt;
	bool ipu_int_clk;
	bool overlay;
	bool alpha_chan_en;
	dma_addr_t alpha_phy_addr0;
	dma_addr_t alpha_phy_addr1;
	void *alpha_virt_addr0;
	void *alpha_virt_addr1;
	uint32_t alpha_mem_len;
	uint32_t ipu_ch_irq;
	uint32_t ipu_alp_ch_irq;
	uint32_t cur_ipu_buf;
	uint32_t cur_ipu_alpha_buf;

	u32 pseudo_palette[16];

	bool wait4vsync;
	struct semaphore flip_sem;
	struct semaphore alpha_flip_sem;
	struct completion vsync_complete;
};

struct mxcfb_mode {
	int dev_mode;
	int num_modes;
	struct fb_videomode *mode;
};

struct mxcfb_alloc_list {
	struct list_head list;
	dma_addr_t phy_addr;
	void *cpu_addr;
	u32 size;
};

enum {
	BOTH_ON,
	SRC_ON,
	TGT_ON,
	BOTH_OFF
};

static bool g_dp_in_use;
LIST_HEAD(fb_alloc_list);
static struct fb_info *mxcfb_info[3];
static struct mxcfb_mode mxc_disp_mode[MXCFB_PORT_NUM];
static int (*mxcfb_pre_setup[MXCFB_PORT_NUM])(struct fb_info *info);

/*
 * register pre-setup callback for some display
 * driver which need prepare clk etc.
 */
void mxcfb_register_presetup(int disp_port,
	int (*pre_setup)(struct fb_info *info))
{
	if (pre_setup)
		mxcfb_pre_setup[disp_port] = pre_setup;
}
EXPORT_SYMBOL(mxcfb_register_presetup);

/*
 * mode register from each display driver before
 * primary fb setting.
 */
void mxcfb_register_mode(int disp_port,
	const struct fb_videomode *modedb,
	int num_modes, int dev_mode)
{
	struct fb_videomode *mode;
	int mode_sum;

	if (disp_port > MXCFB_PORT_NUM)
		return;

	/*
	 * if there is new DDC device, overwrite old modes.
	 * if there is old DDC device while new device is not DDC,
	 * just keep old DDC modes.
	 */
	if (dev_mode & MXC_DISP_DDC_DEV) {
		if (mxc_disp_mode[disp_port].num_modes) {
			kfree(mxc_disp_mode[disp_port].mode);
			mxc_disp_mode[disp_port].num_modes = 0;
		}
	} else if (mxc_disp_mode[disp_port].dev_mode & MXC_DISP_DDC_DEV)
		return;

	mode_sum = mxc_disp_mode[disp_port].num_modes + num_modes;
	mode = kzalloc(mode_sum * sizeof(struct fb_videomode), GFP_KERNEL);

	if (mxc_disp_mode[disp_port].num_modes)
		memcpy(mode, mxc_disp_mode[disp_port].mode,
			mxc_disp_mode[disp_port].num_modes
			* sizeof(struct fb_videomode));
	if (modedb)
		memcpy(mode + mxc_disp_mode[disp_port].num_modes,
			modedb, num_modes * sizeof(struct fb_videomode));

	if (mxc_disp_mode[disp_port].num_modes)
		kfree(mxc_disp_mode[disp_port].mode);

	mxc_disp_mode[disp_port].mode = mode;
	mxc_disp_mode[disp_port].num_modes += num_modes;
	mxc_disp_mode[disp_port].dev_mode = dev_mode;

	return;
}
EXPORT_SYMBOL(mxcfb_register_mode);

static uint32_t bpp_to_pixfmt(struct fb_info *fbi)
{
	uint32_t pixfmt = 0;

	if (fbi->var.nonstd)
		return fbi->var.nonstd;

	switch (fbi->var.bits_per_pixel) {
	case 24:
		pixfmt = IPU_PIX_FMT_BGR24;
		break;
	case 32:
		pixfmt = IPU_PIX_FMT_BGR32;
		break;
	case 16:
		pixfmt = IPU_PIX_FMT_RGB565;
		break;
	}
	return pixfmt;
}

static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id);
static int mxcfb_blank(int blank, struct fb_info *info);
static int mxcfb_map_video_memory(struct fb_info *fbi);
static int mxcfb_unmap_video_memory(struct fb_info *fbi);
static int mxcfb_option_setup(struct fb_info *info, char *options);

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 0;
	fix->ywrapstep = 1;
	fix->ypanstep = 1;

	return 0;
}

static int _setup_disp_channel1(struct fb_info *fbi)
{
	ipu_channel_params_t params;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	memset(&params, 0, sizeof(params));
	params.mem_dp_bg_sync.di = mxc_fbi->ipu_di;

	/*
	 * Assuming interlaced means yuv output, below setting also
	 * valid for mem_dc_sync. FG should have the same vmode as BG.
	 */
	if (mxc_fbi->ipu_ch == MEM_FG_SYNC) {
		struct mxcfb_info *mxc_fbi_tmp;
		int i;

		for (i = 0; i < num_registered_fb; i++) {
			mxc_fbi_tmp = (struct mxcfb_info *)
				(registered_fb[i]->par);
			if (mxc_fbi_tmp->ipu_ch == MEM_BG_SYNC) {
				fbi->var.vmode =
				registered_fb[i]->var.vmode;
				mxc_fbi->ipu_di_pix_fmt =
				mxc_fbi_tmp->ipu_di_pix_fmt;
				break;
			}
		}
	}
	if (mxc_fbi->ipu_ch == MEM_DC_SYNC) {
		if (fbi->var.vmode & FB_VMODE_INTERLACED) {
			params.mem_dc_sync.interlaced = true;
			params.mem_dc_sync.out_pixel_fmt =
				IPU_PIX_FMT_YUV444;
		} else {
			if (mxc_fbi->ipu_di_pix_fmt)
				params.mem_dc_sync.out_pixel_fmt = mxc_fbi->ipu_di_pix_fmt;
			else
				params.mem_dc_sync.out_pixel_fmt = IPU_PIX_FMT_RGB666;
		}
		params.mem_dc_sync.in_pixel_fmt = bpp_to_pixfmt(fbi);
	} else {
		if (fbi->var.vmode & FB_VMODE_INTERLACED) {
			params.mem_dp_bg_sync.interlaced = true;
			params.mem_dp_bg_sync.out_pixel_fmt =
				IPU_PIX_FMT_YUV444;
		} else {
			if (mxc_fbi->ipu_di_pix_fmt)
				params.mem_dp_bg_sync.out_pixel_fmt = mxc_fbi->ipu_di_pix_fmt;
			else
				params.mem_dp_bg_sync.out_pixel_fmt = IPU_PIX_FMT_RGB666;
		}
		params.mem_dp_bg_sync.in_pixel_fmt = bpp_to_pixfmt(fbi);
		if (mxc_fbi->alpha_chan_en)
			params.mem_dp_bg_sync.alpha_chan_en = true;
	}
	ipu_init_channel(mxc_fbi->ipu_ch, &params);

	return 0;
}

static int _setup_disp_channel2(struct fb_info *fbi)
{
	int retval = 0;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	int fb_stride;
	unsigned long base;

	switch (bpp_to_pixfmt(fbi)) {
	case IPU_PIX_FMT_YUV420P2:
	case IPU_PIX_FMT_YVU420P:
	case IPU_PIX_FMT_NV12:
	case IPU_PIX_FMT_YUV422P:
	case IPU_PIX_FMT_YVU422P:
	case IPU_PIX_FMT_YUV420P:
		fb_stride = fbi->var.xres_virtual;
		break;
	default:
		fb_stride = fbi->fix.line_length;
	}

	mxc_fbi->cur_ipu_buf = 2;
	sema_init(&mxc_fbi->flip_sem, 1);
	if (mxc_fbi->alpha_chan_en) {
		mxc_fbi->cur_ipu_alpha_buf = 1;
		sema_init(&mxc_fbi->alpha_flip_sem, 1);
	}
	fbi->var.xoffset = 0;

	base = (fbi->var.yoffset * fbi->var.xres_virtual + fbi->var.xoffset);
	base = (fbi->var.bits_per_pixel) * base / 8;
	base += fbi->fix.smem_start;

	retval = ipu_init_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
					 bpp_to_pixfmt(fbi),
					 fbi->var.xres, fbi->var.yres,
					 fb_stride,
					 IPU_ROTATE_NONE,
					 base,
					 base,
					 (fbi->var.accel_flags ==
					  FB_ACCEL_TRIPLE_FLAG) ? base : 0,
					 0, 0);
	if (retval) {
		dev_err(fbi->device,
			"ipu_init_channel_buffer error %d\n", retval);
	}

	if (mxc_fbi->alpha_chan_en) {
		retval = ipu_init_channel_buffer(mxc_fbi->ipu_ch,
						 IPU_ALPHA_IN_BUFFER,
						 IPU_PIX_FMT_GENERIC,
						 fbi->var.xres, fbi->var.yres,
						 fbi->var.xres,
						 IPU_ROTATE_NONE,
						 mxc_fbi->alpha_phy_addr1,
						 mxc_fbi->alpha_phy_addr0,
						 0,
						 0, 0);
		if (retval) {
			dev_err(fbi->device,
				"ipu_init_channel_buffer error %d\n", retval);
			return retval;
		}
	}

	return retval;
}

/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_set_par(struct fb_info *fbi)
{
	int retval = 0;
	u32 mem_len, alpha_mem_len;
	ipu_di_signal_cfg_t sig_cfg;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	dev_dbg(fbi->device, "Reconfiguring framebuffer\n");

	ipu_disable_irq(mxc_fbi->ipu_ch_irq);
	ipu_disable_channel(mxc_fbi->ipu_ch, true);
	ipu_uninit_channel(mxc_fbi->ipu_ch);
	ipu_clear_irq(mxc_fbi->ipu_ch_irq);
	mxcfb_set_fix(fbi);

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			mxcfb_unmap_video_memory(fbi);

		if (mxcfb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}
	if (mxc_fbi->alpha_chan_en) {
		alpha_mem_len = fbi->var.xres * fbi->var.yres;
		if ((!mxc_fbi->alpha_phy_addr0 && !mxc_fbi->alpha_phy_addr1) ||
		    (alpha_mem_len > mxc_fbi->alpha_mem_len)) {
			if (mxc_fbi->alpha_phy_addr0)
				dma_free_coherent(fbi->device,
						  mxc_fbi->alpha_mem_len,
						  mxc_fbi->alpha_virt_addr0,
						  mxc_fbi->alpha_phy_addr0);
			if (mxc_fbi->alpha_phy_addr1)
				dma_free_coherent(fbi->device,
						  mxc_fbi->alpha_mem_len,
						  mxc_fbi->alpha_virt_addr1,
						  mxc_fbi->alpha_phy_addr1);

			mxc_fbi->alpha_virt_addr0 =
					dma_alloc_coherent(fbi->device,
						  alpha_mem_len,
						  &mxc_fbi->alpha_phy_addr0,
						  GFP_DMA | GFP_KERNEL);

			mxc_fbi->alpha_virt_addr1 =
					dma_alloc_coherent(fbi->device,
						  alpha_mem_len,
						  &mxc_fbi->alpha_phy_addr1,
						  GFP_DMA | GFP_KERNEL);
			if (mxc_fbi->alpha_virt_addr0 == NULL ||
			    mxc_fbi->alpha_virt_addr1 == NULL) {
				dev_err(fbi->device, "mxcfb: dma alloc for"
					" alpha buffer failed.\n");
				if (mxc_fbi->alpha_virt_addr0)
					dma_free_coherent(fbi->device,
						  mxc_fbi->alpha_mem_len,
						  mxc_fbi->alpha_virt_addr0,
						  mxc_fbi->alpha_phy_addr0);
				if (mxc_fbi->alpha_virt_addr1)
					dma_free_coherent(fbi->device,
						  mxc_fbi->alpha_mem_len,
						  mxc_fbi->alpha_virt_addr1,
						  mxc_fbi->alpha_phy_addr1);
				return -ENOMEM;
			}
			mxc_fbi->alpha_mem_len = alpha_mem_len;
		}
	}

	if (mxc_fbi->next_blank != FB_BLANK_UNBLANK)
		return retval;

	_setup_disp_channel1(fbi);

	if (!mxc_fbi->overlay) {
		uint32_t out_pixel_fmt;

		memset(&sig_cfg, 0, sizeof(sig_cfg));
		if (fbi->var.vmode & FB_VMODE_INTERLACED) {
			sig_cfg.interlaced = true;
			out_pixel_fmt = IPU_PIX_FMT_YUV444;
		} else {
			if (mxc_fbi->ipu_di_pix_fmt)
				out_pixel_fmt = mxc_fbi->ipu_di_pix_fmt;
			else
				out_pixel_fmt = IPU_PIX_FMT_RGB666;
		}
		if (fbi->var.vmode & FB_VMODE_ODD_FLD_FIRST) /* PAL */
			sig_cfg.odd_field_first = true;
		if (mxc_fbi->ipu_int_clk)
			sig_cfg.int_clk = true;
		if (fbi->var.sync & FB_SYNC_HOR_HIGH_ACT)
			sig_cfg.Hsync_pol = true;
		if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT)
			sig_cfg.Vsync_pol = true;
		if (!(fbi->var.sync & FB_SYNC_CLK_LAT_FALL))
			sig_cfg.clk_pol = true;
		if (fbi->var.sync & FB_SYNC_DATA_INVERT)
			sig_cfg.data_pol = true;
		if (!(fbi->var.sync & FB_SYNC_OE_LOW_ACT))
			sig_cfg.enable_pol = true;
		if (fbi->var.sync & FB_SYNC_CLK_IDLE_EN)
			sig_cfg.clkidle_en = true;

		dev_dbg(fbi->device, "pixclock = %ul Hz\n",
			(u32) (PICOS2KHZ(fbi->var.pixclock) * 1000UL));

		if (ipu_init_sync_panel(mxc_fbi->ipu_di,
					(PICOS2KHZ(fbi->var.pixclock)) * 1000UL,
					fbi->var.xres, fbi->var.yres,
					out_pixel_fmt,
					fbi->var.left_margin,
					fbi->var.hsync_len,
					fbi->var.right_margin,
					fbi->var.upper_margin,
					fbi->var.vsync_len,
					fbi->var.lower_margin,
					0, sig_cfg) != 0) {
			dev_err(fbi->device,
				"mxcfb: Error initializing panel.\n");
			return -EINVAL;
		}

		fbi->mode =
		    (struct fb_videomode *)fb_match_mode(&fbi->var,
							 &fbi->modelist);

		ipu_disp_set_window_pos(mxc_fbi->ipu_ch, 0, 0);
	}

	retval = _setup_disp_channel2(fbi);
	if (retval)
		return retval;

	ipu_enable_channel(mxc_fbi->ipu_ch);

	return retval;
}

static int _swap_channels(struct fb_info *fbi,
			  struct fb_info *fbi_to, bool both_on)
{
	int retval, tmp;
	ipu_channel_t old_ch;
	struct mxcfb_info *mxc_fbi_from = (struct mxcfb_info *)fbi->par;
	struct mxcfb_info *mxc_fbi_to = (struct mxcfb_info *)fbi_to->par;

	if (both_on) {
		ipu_disable_channel(mxc_fbi_to->ipu_ch, true);
		ipu_uninit_channel(mxc_fbi_to->ipu_ch);
	}

	/* switch the mxc fbi parameters */
	old_ch = mxc_fbi_from->ipu_ch;
	mxc_fbi_from->ipu_ch = mxc_fbi_to->ipu_ch;
	mxc_fbi_to->ipu_ch = old_ch;
	tmp = mxc_fbi_from->ipu_ch_irq;
	mxc_fbi_from->ipu_ch_irq = mxc_fbi_to->ipu_ch_irq;
	mxc_fbi_to->ipu_ch_irq = tmp;

	_setup_disp_channel1(fbi);
	retval = _setup_disp_channel2(fbi);
	if (retval)
		return retval;

	/* switch between dp and dc, disable old idmac, enable new idmac */
	retval = ipu_swap_channel(old_ch, mxc_fbi_from->ipu_ch);
	ipu_uninit_channel(old_ch);

	if (both_on) {
		_setup_disp_channel1(fbi_to);
		retval = _setup_disp_channel2(fbi_to);
		if (retval)
			return retval;
		ipu_enable_channel(mxc_fbi_to->ipu_ch);
	}

	return retval;
}

static int swap_channels(struct fb_info *fbi)
{
	int i;
	int swap_mode;
	ipu_channel_t ch_to;
	struct mxcfb_info *mxc_fbi_from = (struct mxcfb_info *)fbi->par;
	struct fb_info *fbi_to = NULL;
	struct mxcfb_info *mxc_fbi_to;

	/* what's the target channel? */
	if (mxc_fbi_from->ipu_ch == MEM_BG_SYNC)
		ch_to = MEM_DC_SYNC;
	else
		ch_to = MEM_BG_SYNC;

	for (i = 0; i < num_registered_fb; i++) {
		mxc_fbi_to =
			(struct mxcfb_info *)mxcfb_info[i]->par;
		if (mxc_fbi_to->ipu_ch == ch_to) {
			fbi_to = mxcfb_info[i];
			break;
		}
	}
	if (fbi_to == NULL)
		return -1;

	ipu_clear_irq(mxc_fbi_from->ipu_ch_irq);
	ipu_clear_irq(mxc_fbi_to->ipu_ch_irq);
	ipu_free_irq(mxc_fbi_from->ipu_ch_irq, fbi);
	ipu_free_irq(mxc_fbi_to->ipu_ch_irq, fbi_to);

	if (mxc_fbi_from->cur_blank == FB_BLANK_UNBLANK) {
		if (mxc_fbi_to->cur_blank == FB_BLANK_UNBLANK)
			swap_mode = BOTH_ON;
		else
			swap_mode = SRC_ON;
	} else {
		if (mxc_fbi_to->cur_blank == FB_BLANK_UNBLANK)
			swap_mode = TGT_ON;
		else
			swap_mode = BOTH_OFF;
	}

	/* tvout di-1: for DC use UYVY, for DP use RGB */
	if (mxc_fbi_from->ipu_di == 1 && ch_to == MEM_DC_SYNC) {
		fbi->var.bits_per_pixel = 16;
		fbi->var.nonstd = IPU_PIX_FMT_UYVY;
	} else if (mxc_fbi_from->ipu_di == 1 && ch_to == MEM_BG_SYNC) {
		fbi->var.nonstd = 0;
	} else if (mxc_fbi_from->ipu_di == 0 && ch_to == MEM_DC_SYNC) {
		fbi_to->var.nonstd = 0;
	} else if (mxc_fbi_from->ipu_di == 0 && ch_to == MEM_BG_SYNC) {
		fbi->var.bits_per_pixel = 16;
		fbi->var.nonstd = IPU_PIX_FMT_UYVY;
	}

	switch (swap_mode) {
	case BOTH_ON:
		/* disable target->switch src->enable target */
		_swap_channels(fbi, fbi_to, true);
		break;
	case SRC_ON:
		/* just switch src */
		_swap_channels(fbi, fbi_to, false);
		break;
	case TGT_ON:
		/* just switch target */
		_swap_channels(fbi_to, fbi, false);
		break;
	case BOTH_OFF:
		/* switch directly, no more need to do */
		mxc_fbi_to->ipu_ch = mxc_fbi_from->ipu_ch;
		mxc_fbi_from->ipu_ch = ch_to;
		i = mxc_fbi_from->ipu_ch_irq;
		mxc_fbi_from->ipu_ch_irq = mxc_fbi_to->ipu_ch_irq;
		mxc_fbi_to->ipu_ch_irq = i;
		break;
	default:
		break;
	}

	if (ipu_request_irq(mxc_fbi_from->ipu_ch_irq, mxcfb_irq_handler, 0,
		MXCFB_NAME, fbi) != 0) {
		dev_err(fbi->device, "Error registering irq %d\n",
			mxc_fbi_from->ipu_ch_irq);
		return -EBUSY;
	}
	ipu_disable_irq(mxc_fbi_from->ipu_ch_irq);
	if (ipu_request_irq(mxc_fbi_to->ipu_ch_irq, mxcfb_irq_handler, 0,
		MXCFB_NAME, fbi_to) != 0) {
		dev_err(fbi_to->device, "Error registering irq %d\n",
			mxc_fbi_to->ipu_ch_irq);
		return -EBUSY;
	}
	ipu_disable_irq(mxc_fbi_to->ipu_ch_irq);

	return 0;
}

/*
 * Check framebuffer variable parameters and adjust to valid values.
 *
 * @param       var      framebuffer variable parameters
 *
 * @param       info     framebuffer information pointer
 */
static int mxcfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 vtotal;
	u32 htotal;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;

	/* fg should not bigger than bg */
	if (mxc_fbi->ipu_ch == MEM_FG_SYNC) {
		struct fb_info *fbi_tmp;
		struct mxcfb_info *mxc_fbi_tmp;
		int i, bg_xres, bg_yres;
		int16_t pos_x, pos_y;

		bg_xres = var->xres;
		bg_yres = var->yres;

		for (i = 0; i < num_registered_fb; i++) {
			fbi_tmp = registered_fb[i];
			mxc_fbi_tmp = (struct mxcfb_info *)
				(fbi_tmp->par);
			if (mxc_fbi_tmp->ipu_ch == MEM_BG_SYNC) {
				bg_xres = fbi_tmp->var.xres;
				bg_yres = fbi_tmp->var.yres;
				break;
			}
		}

		ipu_disp_get_window_pos(mxc_fbi->ipu_ch, &pos_x, &pos_y);

		if ((var->xres + pos_x) > bg_xres)
			var->xres = bg_xres - pos_x;
		if ((var->yres + pos_y) > bg_yres)
			var->yres = bg_yres - pos_y;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

	/* Default Y virtual size is 3*yres */
	if (var->yres_virtual < var->yres * 3)
		var->yres_virtual = var->yres * 3;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 12) &&
	    (var->bits_per_pixel != 8))
		var->bits_per_pixel = 16;

	switch (var->bits_per_pixel) {
	case 8:
		var->red.length = 3;
		var->red.offset = 5;
		var->red.msb_right = 0;

		var->green.length = 3;
		var->green.offset = 2;
		var->green.msb_right = 0;

		var->blue.length = 2;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	if (var->pixclock < 1000) {
		htotal = var->xres + var->right_margin + var->hsync_len +
		    var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
		    var->upper_margin;
		var->pixclock = (vtotal * htotal * 6UL) / 100UL;
		var->pixclock = KHZ2PICOS(var->pixclock);
		dev_dbg(info->device,
			"pixclock set for 60Hz refresh = %u ps\n",
			var->pixclock);
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxcfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int trans, struct fb_info *fbi)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (fbi->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->pseudo_palette;

			val = _chan_to_field(red, &fbi->var.red);
			val |= _chan_to_field(green, &fbi->var.green);
			val |= _chan_to_field(blue, &fbi->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

/*
 * Function to handle custom ioctls for MXC framebuffer.
 *
 * @param       inode   inode struct
 *
 * @param       file    file struct
 *
 * @param       cmd     Ioctl command to handle
 *
 * @param       arg     User pointer to command arguments
 *
 * @param       fbi     framebuffer information pointer
 */
static int mxcfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int __user *argp = (void __user *)arg;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	switch (cmd) {
	case MXCFB_SET_GBL_ALPHA:
		{
			struct mxcfb_gbl_alpha ga;

			if (copy_from_user(&ga, (void *)arg, sizeof(ga))) {
				retval = -EFAULT;
				break;
			}

			if (ipu_disp_set_global_alpha(mxc_fbi->ipu_ch,
						      (bool)ga.enable,
						      ga.alpha)) {
				retval = -EINVAL;
				break;
			}

			if (ga.enable)
				mxc_fbi->alpha_chan_en = false;

			if (ga.enable)
				dev_dbg(fbi->device,
					"Set global alpha of %s to %d\n",
					fbi->fix.id, ga.alpha);
			break;
		}
	case MXCFB_SET_LOC_ALPHA:
		{
			struct mxcfb_loc_alpha la;
			int i;
			char *video_plane_idstr = "";

			if (copy_from_user(&la, (void *)arg, sizeof(la))) {
				retval = -EFAULT;
				break;
			}

			if (ipu_disp_set_global_alpha(mxc_fbi->ipu_ch,
						      !(bool)la.enable, 0)) {
				retval = -EINVAL;
				break;
			}

			if (la.enable && !la.alpha_in_pixel) {
				mxc_fbi->alpha_chan_en = true;

				if (mxc_fbi->ipu_ch == MEM_FG_SYNC)
					video_plane_idstr = "DISP3 BG";
				else if (mxc_fbi->ipu_ch == MEM_BG_SYNC)
					video_plane_idstr = "DISP3 FG";

				for (i = 0; i < num_registered_fb; i++) {
					char *idstr = registered_fb[i]->fix.id;
					if (strcmp(idstr, video_plane_idstr) == 0) {
						((struct mxcfb_info *)(registered_fb[i]->par))->alpha_chan_en = false;
						break;
					}
				}
			} else
				mxc_fbi->alpha_chan_en = false;

			mxcfb_set_par(fbi);

			la.alpha_phy_addr0 = mxc_fbi->alpha_phy_addr0;
			la.alpha_phy_addr1 = mxc_fbi->alpha_phy_addr1;
			if (copy_to_user((void *)arg, &la, sizeof(la))) {
				retval = -EFAULT;
				break;
			}

			if (la.enable)
				dev_dbg(fbi->device,
					"Enable DP local alpha for %s\n",
					fbi->fix.id);
			break;
		}
	case MXCFB_SET_LOC_ALP_BUF:
		{
			unsigned long base;
			uint32_t ipu_alp_ch_irq;

			if (!(((mxc_fbi->ipu_ch == MEM_FG_SYNC) ||
			     (mxc_fbi->ipu_ch == MEM_BG_SYNC)) &&
			     (mxc_fbi->alpha_chan_en))) {
				dev_err(fbi->device,
					"Should use background or overlay "
					"framebuffer to set the alpha buffer "
					"number\n");
				return -EINVAL;
			}

			if (get_user(base, argp))
				return -EFAULT;

			if (base != mxc_fbi->alpha_phy_addr0 &&
			    base != mxc_fbi->alpha_phy_addr1) {
				dev_err(fbi->device,
					"Wrong alpha buffer physical address "
					"%lu\n", base);
				return -EINVAL;
			}

			if (mxc_fbi->ipu_ch == MEM_FG_SYNC)
				ipu_alp_ch_irq = IPU_IRQ_FG_ALPHA_SYNC_EOF;
			else
				ipu_alp_ch_irq = IPU_IRQ_BG_ALPHA_SYNC_EOF;

			down(&mxc_fbi->alpha_flip_sem);

			mxc_fbi->cur_ipu_alpha_buf =
						!mxc_fbi->cur_ipu_alpha_buf;
			if (ipu_update_channel_buffer(mxc_fbi->ipu_ch,
						      IPU_ALPHA_IN_BUFFER,
						      mxc_fbi->
							cur_ipu_alpha_buf,
						      base) == 0) {
				ipu_select_buffer(mxc_fbi->ipu_ch,
						  IPU_ALPHA_IN_BUFFER,
						  mxc_fbi->cur_ipu_alpha_buf);
				ipu_clear_irq(ipu_alp_ch_irq);
				ipu_enable_irq(ipu_alp_ch_irq);
			} else {
				dev_err(fbi->device,
					"Error updating %s SDC alpha buf %d "
					"to address=0x%08lX\n",
					fbi->fix.id,
					mxc_fbi->cur_ipu_alpha_buf, base);
			}
			break;
		}
	case MXCFB_SET_CLR_KEY:
		{
			struct mxcfb_color_key key;
			if (copy_from_user(&key, (void *)arg, sizeof(key))) {
				retval = -EFAULT;
				break;
			}
			retval = ipu_disp_set_color_key(mxc_fbi->ipu_ch,
							key.enable,
							key.color_key);
			dev_dbg(fbi->device, "Set color key to 0x%08X\n",
				key.color_key);
			break;
		}
	case MXCFB_SET_GAMMA:
		{
			struct mxcfb_gamma gamma;
			if (copy_from_user(&gamma, (void *)arg, sizeof(gamma))) {
				retval = -EFAULT;
				break;
			}
			retval = ipu_disp_set_gamma_correction(mxc_fbi->ipu_ch,
							gamma.enable,
							gamma.constk,
							gamma.slopek);
			break;
		}
	case MXCFB_WAIT_FOR_VSYNC:
		{
			if (mxc_fbi->ipu_ch == MEM_FG_SYNC) {
				struct mxcfb_info *bg_mxcfbi = NULL;
				int i;
				for (i = 0; i < num_registered_fb; i++) {
					bg_mxcfbi =
					((struct mxcfb_info *)(registered_fb[i]->par));

					if (bg_mxcfbi->ipu_ch == MEM_BG_SYNC)
						break;
				}
				if (bg_mxcfbi->cur_blank != FB_BLANK_UNBLANK) {
					retval = -EINVAL;
					break;
				}
			}
			if (mxc_fbi->cur_blank != FB_BLANK_UNBLANK) {
				retval = -EINVAL;
				break;
			}

			init_completion(&mxc_fbi->vsync_complete);

			ipu_clear_irq(mxc_fbi->ipu_ch_irq);
			mxc_fbi->wait4vsync = 1;
			ipu_enable_irq(mxc_fbi->ipu_ch_irq);
			retval = wait_for_completion_interruptible_timeout(
				&mxc_fbi->vsync_complete, 1 * HZ);
			if (retval == 0) {
				dev_err(fbi->device,
					"MXCFB_WAIT_FOR_VSYNC: timeout %d\n",
					retval);
				mxc_fbi->wait4vsync = 0;
				retval = -ETIME;
			} else if (retval > 0) {
				retval = 0;
			}
			break;
		}
	case FBIO_ALLOC:
		{
			int size;
			struct mxcfb_alloc_list *mem;

			mem = kzalloc(sizeof(*mem), GFP_KERNEL);
			if (mem == NULL)
				return -ENOMEM;

			if (get_user(size, argp))
				return -EFAULT;

			mem->size = PAGE_ALIGN(size);

			mem->cpu_addr = dma_alloc_coherent(fbi->device, size,
							   &mem->phy_addr,
							   GFP_DMA);
			if (mem->cpu_addr == NULL) {
				kfree(mem);
				return -ENOMEM;
			}

			list_add(&mem->list, &fb_alloc_list);

			dev_dbg(fbi->device, "allocated %d bytes @ 0x%08X\n",
				mem->size, mem->phy_addr);

			if (put_user(mem->phy_addr, argp))
				return -EFAULT;

			break;
		}
	case FBIO_FREE:
		{
			unsigned long offset;
			struct mxcfb_alloc_list *mem;

			if (get_user(offset, argp))
				return -EFAULT;

			retval = -EINVAL;
			list_for_each_entry(mem, &fb_alloc_list, list) {
				if (mem->phy_addr == offset) {
					list_del(&mem->list);
					dma_free_coherent(fbi->device,
							  mem->size,
							  mem->cpu_addr,
							  mem->phy_addr);
					kfree(mem);
					retval = 0;
					break;
				}
			}

			break;
		}
	case MXCFB_SET_OVERLAY_POS:
		{
			struct mxcfb_pos pos;
			struct fb_info *bg_fbi = NULL;
			struct mxcfb_info *bg_mxcfbi = NULL;
			int i;

			if (mxc_fbi->ipu_ch != MEM_FG_SYNC) {
				dev_err(fbi->device, "Should use the overlay "
					"framebuffer to set the position of "
					"the overlay window\n");
				retval = -EINVAL;
				break;
			}

			if (copy_from_user(&pos, (void *)arg, sizeof(pos))) {
				retval = -EFAULT;
				break;
			}

			for (i = 0; i < num_registered_fb; i++) {
				bg_mxcfbi =
				((struct mxcfb_info *)(registered_fb[i]->par));

				if (bg_mxcfbi->ipu_ch == MEM_BG_SYNC) {
					bg_fbi = registered_fb[i];
					break;
				}
			}

			if (bg_fbi == NULL) {
				dev_err(fbi->device, "Cannot find the "
					"background framebuffer\n");
				retval = -ENOENT;
				break;
			}

			if (fbi->var.xres + pos.x > bg_fbi->var.xres) {
				if (bg_fbi->var.xres < fbi->var.xres)
					pos.x = 0;
				else
					pos.x = bg_fbi->var.xres - fbi->var.xres;
			}
			if (fbi->var.yres + pos.y > bg_fbi->var.yres) {
				if (bg_fbi->var.yres < fbi->var.yres)
					pos.y = 0;
				else
					pos.y = bg_fbi->var.yres - fbi->var.yres;
			}

			retval = ipu_disp_set_window_pos(mxc_fbi->ipu_ch,
							 pos.x, pos.y);

			if (copy_to_user((void *)arg, &pos, sizeof(pos))) {
				retval = -EFAULT;
				break;
			}
			break;
		}
	case MXCFB_GET_FB_IPU_CHAN:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_ch, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_DIFMT:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_di_pix_fmt, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_FB_IPU_DI:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->ipu_di, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_GET_FB_BLANK:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (put_user(mxc_fbi->cur_blank, argp))
				return -EFAULT;
			break;
		}
	case MXCFB_SET_DIFMT:
		{
			struct mxcfb_info *mxc_fbi =
				(struct mxcfb_info *)fbi->par;

			if (get_user(mxc_fbi->ipu_di_pix_fmt, argp))
				return -EFAULT;

			break;
		}
	default:
		retval = -EINVAL;
	}
	return retval;
}

/*
 * mxcfb_blank():
 *      Blank the display.
 */
static int mxcfb_blank(int blank, struct fb_info *info)
{
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par;

	dev_dbg(info->device, "blank = %d\n", blank);

	if (mxc_fbi->cur_blank == blank)
		return 0;

	mxc_fbi->next_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		ipu_disable_channel(mxc_fbi->ipu_ch, true);
		ipu_uninit_sync_panel(mxc_fbi->ipu_di);
		ipu_uninit_channel(mxc_fbi->ipu_ch);
		break;
	case FB_BLANK_UNBLANK:
		mxcfb_set_par(info);
		break;
	}
	mxc_fbi->cur_blank = blank;
	return 0;
}

/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 *
 * @param               var     Variable screen buffer information
 * @param               info    Framebuffer information pointer
 */
static int
mxcfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)info->par,
			  *mxc_graphic_fbi = NULL;
	u_int y_bottom;
	unsigned long base, active_alpha_phy_addr = 0;
	bool loc_alpha_en = false;
	int i = 0;

	if (info->var.yoffset == var->yoffset)
		return 0;	/* No change, do nothing */

	/* no pan display during fb blank */
	if (mxc_fbi->ipu_ch == MEM_FG_SYNC) {
		struct mxcfb_info *bg_mxcfbi = NULL;
		int j;
		for (j = 0; j < num_registered_fb; j++) {
			bg_mxcfbi =
				((struct mxcfb_info *)(registered_fb[j]->par));

			if (bg_mxcfbi->ipu_ch == MEM_BG_SYNC)
				break;
		}
		if (bg_mxcfbi->cur_blank != FB_BLANK_UNBLANK)
			return -EINVAL;
	}
	if (mxc_fbi->cur_blank != FB_BLANK_UNBLANK)
		return -EINVAL;

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual)
		return -EINVAL;

	base = (var->yoffset * var->xres_virtual + var->xoffset);
	base = (var->bits_per_pixel) * base / 8;
	base += info->fix.smem_start;

	/* Check if DP local alpha is enabled and find the graphic fb */
	if (mxc_fbi->ipu_ch == MEM_BG_SYNC || mxc_fbi->ipu_ch == MEM_FG_SYNC) {
		for (i = 0; i < num_registered_fb; i++) {
			char *idstr = registered_fb[i]->fix.id;
			if ((strcmp(idstr, "DISP3 BG") == 0 ||
			     strcmp(idstr, "DISP3 FG") == 0) &&
			    ((struct mxcfb_info *)
			      (registered_fb[i]->par))->alpha_chan_en) {
				loc_alpha_en = true;
				mxc_graphic_fbi = (struct mxcfb_info *)
						(registered_fb[i]->par);
				active_alpha_phy_addr =
					mxc_fbi->cur_ipu_alpha_buf ?
					mxc_graphic_fbi->alpha_phy_addr1 :
					mxc_graphic_fbi->alpha_phy_addr0;
				dev_dbg(info->device, "Updating SDC alpha "
					"buf %d address=0x%08lX\n",
					!mxc_fbi->cur_ipu_alpha_buf,
					active_alpha_phy_addr);
				break;
			}
		}
	}

	down(&mxc_fbi->flip_sem);

	mxc_fbi->cur_ipu_buf = (++mxc_fbi->cur_ipu_buf) % 3;
	mxc_fbi->cur_ipu_alpha_buf = !mxc_fbi->cur_ipu_alpha_buf;

	dev_dbg(info->device, "Updating SDC %s buf %d address=0x%08lX\n",
		info->fix.id, mxc_fbi->cur_ipu_buf, base);

	if (ipu_update_channel_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
				      mxc_fbi->cur_ipu_buf, base) == 0) {
		/* Update the DP local alpha buffer only for graphic plane */
		if (loc_alpha_en && mxc_graphic_fbi == mxc_fbi &&
		    ipu_update_channel_buffer(mxc_graphic_fbi->ipu_ch,
					      IPU_ALPHA_IN_BUFFER,
					      mxc_fbi->cur_ipu_alpha_buf,
					      active_alpha_phy_addr) == 0) {
			ipu_select_buffer(mxc_graphic_fbi->ipu_ch,
					  IPU_ALPHA_IN_BUFFER,
					  mxc_fbi->cur_ipu_alpha_buf);
		}

		ipu_select_buffer(mxc_fbi->ipu_ch, IPU_INPUT_BUFFER,
				  mxc_fbi->cur_ipu_buf);
		ipu_clear_irq(mxc_fbi->ipu_ch_irq);
		ipu_enable_irq(mxc_fbi->ipu_ch_irq);
	} else {
		dev_err(info->device,
			"Error updating SDC buf %d to address=0x%08lX, "
			"current buf %d, buf0 ready %d, buf1 ready %d, "
			"buf2 ready %d\n", mxc_fbi->cur_ipu_buf, base,
			ipu_get_cur_buffer_idx(mxc_fbi->ipu_ch,
					       IPU_INPUT_BUFFER),
			ipu_check_buffer_ready(mxc_fbi->ipu_ch,
					       IPU_INPUT_BUFFER, 0),
			ipu_check_buffer_ready(mxc_fbi->ipu_ch,
					       IPU_INPUT_BUFFER, 1),
			ipu_check_buffer_ready(mxc_fbi->ipu_ch,
					       IPU_INPUT_BUFFER, 2));
		mxc_fbi->cur_ipu_buf = (++mxc_fbi->cur_ipu_buf) % 3;
		mxc_fbi->cur_ipu_buf = (++mxc_fbi->cur_ipu_buf) % 3;
		mxc_fbi->cur_ipu_alpha_buf = !mxc_fbi->cur_ipu_alpha_buf;
		ipu_clear_irq(mxc_fbi->ipu_ch_irq);
		ipu_enable_irq(mxc_fbi->ipu_ch_irq);
		return -EBUSY;
	}

	dev_dbg(info->device, "Update complete\n");

	info->var.yoffset = var->yoffset;

	return 0;
}

/*
 * Function to handle custom mmap for MXC framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       vma     Pointer to vm_area_struct
 */
static int mxcfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	bool found = false;
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct mxcfb_alloc_list *mem;
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory */
		len = fbi->fix.smem_len - offset;
		vma->vm_pgoff = (fbi->fix.smem_start + offset) >> PAGE_SHIFT;
	} else if ((vma->vm_pgoff ==
			(mxc_fbi->alpha_phy_addr0 >> PAGE_SHIFT)) ||
		   (vma->vm_pgoff ==
			(mxc_fbi->alpha_phy_addr1 >> PAGE_SHIFT))) {
		len = mxc_fbi->alpha_mem_len;
	} else {
		list_for_each_entry(mem, &fb_alloc_list, list) {
			if (offset == mem->phy_addr) {
				found = true;
				len = mem->size;
				break;
			}
		}
		if (!found)
			return -EINVAL;
	}

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(fbi->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core framebuffer driver to perform operations like
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops mxcfb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = mxcfb_set_par,
	.fb_check_var = mxcfb_check_var,
	.fb_setcolreg = mxcfb_setcolreg,
	.fb_pan_display = mxcfb_pan_display,
	.fb_ioctl = mxcfb_ioctl,
	.fb_mmap = mxcfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_blank = mxcfb_blank,
};

static irqreturn_t mxcfb_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mxcfb_info *mxc_fbi = fbi->par;

	if (mxc_fbi->wait4vsync) {
		complete(&mxc_fbi->vsync_complete);
		ipu_disable_irq(irq);
		mxc_fbi->wait4vsync = 0;
	} else {
		up(&mxc_fbi->flip_sem);
		ipu_disable_irq(irq);
	}
	return IRQ_HANDLED;
}

static irqreturn_t mxcfb_alpha_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mxcfb_info *mxc_fbi = fbi->par;

	up(&mxc_fbi->alpha_flip_sem);
	ipu_disable_irq(irq);
	return IRQ_HANDLED;
}

/*
 * Suspends the framebuffer and blanks the screen. Power management support
 */
static int mxcfb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;
	int saved_blank;
#ifdef CONFIG_FB_MXC_LOW_PWR_DISPLAY
	void *fbmem;
#endif

	acquire_console_sem();
	fb_set_suspend(fbi, 1);
	saved_blank = mxc_fbi->cur_blank;
	mxcfb_blank(FB_BLANK_POWERDOWN, fbi);
	mxc_fbi->next_blank = saved_blank;
	release_console_sem();

	return 0;
}

/*
 * Resumes the framebuffer and unblanks the screen. Power management support
 */
static int mxcfb_resume(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = (struct mxcfb_info *)fbi->par;

	acquire_console_sem();
	mxcfb_blank(mxc_fbi->next_blank, fbi);
	fb_set_suspend(fbi, 0);
	release_console_sem();

	return 0;
}

/*
 * Main framebuffer functions
 */

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxcfb_map_video_memory(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxcfb_unmap_video_memory(struct fb_info *fbi)
{
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info *mxcfb_init_fbinfo(struct device *dev, struct fb_ops *ops)
{
	struct fb_info *fbi;
	struct mxcfb_info *mxcfbi;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	fbi = framebuffer_alloc(sizeof(struct mxcfb_info), dev);
	if (!fbi)
		return NULL;

	mxcfbi = (struct mxcfb_info *)fbi->par;

	fbi->var.activate = FB_ACTIVATE_NOW;

	fbi->fbops = ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = mxcfbi->pseudo_palette;

	/*
	 * Allocate colormap
	 */
	fb_alloc_cmap(&fbi->cmap, 16, 0);

	return fbi;
}

static ssize_t show_disp_chan(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)info->par;

	if (mxcfbi->ipu_ch == MEM_BG_SYNC)
		return sprintf(buf, "2-layer-fb-bg\n");
	else if (mxcfbi->ipu_ch == MEM_FG_SYNC)
		return sprintf(buf, "2-layer-fb-fg\n");
	else if (mxcfbi->ipu_ch == MEM_DC_SYNC)
		return sprintf(buf, "1-layer-fb\n");
	else
		return sprintf(buf, "err: no display chan\n");
}

static ssize_t swap_disp_chan(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fb_info *info = dev_get_drvdata(dev);
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)info->par;
	struct mxcfb_info *fg_mxcfbi = NULL;

	acquire_console_sem();
	/* swap only happen between DP-BG and DC, while DP-FG disable */
	if (((mxcfbi->ipu_ch == MEM_BG_SYNC) &&
	     (strstr(buf, "1-layer-fb") != NULL)) ||
	    ((mxcfbi->ipu_ch == MEM_DC_SYNC) &&
	     (strstr(buf, "2-layer-fb-bg") != NULL))) {
		int i;

		for (i = 0; i < num_registered_fb; i++) {
			fg_mxcfbi =
				(struct mxcfb_info *)mxcfb_info[i]->par;
			if (fg_mxcfbi->ipu_ch == MEM_FG_SYNC)
				break;
			else
				fg_mxcfbi = NULL;
		}
		if (!fg_mxcfbi ||
			fg_mxcfbi->cur_blank == FB_BLANK_UNBLANK) {
			dev_err(dev,
				"Can not switch while fb2(fb-fg) is on.\n");
			release_console_sem();
			return count;
		}

		if (swap_channels(info) < 0)
			dev_err(dev, "Swap display channel failed.\n");
	}

	release_console_sem();
	return count;
}
DEVICE_ATTR(fsl_disp_property, 644, show_disp_chan, swap_disp_chan);

static int mxcfb_setup(struct fb_info *fbi, struct platform_device *pdev)
{
	struct mxcfb_info *mxcfbi = (struct mxcfb_info *)fbi->par;
	struct mxc_fb_platform_data *plat_data = pdev->dev.platform_data;
	int ret = 0;

	/* Need dummy values until real panel is configured */
	fbi->var.xres = 240;
	fbi->var.yres = 320;

	if (!mxcfbi->default_bpp)
		mxcfbi->default_bpp = 16;

	if (plat_data && !mxcfbi->ipu_di_pix_fmt)
		mxcfbi->ipu_di_pix_fmt = plat_data->interface_pix_fmt;

	if (!mxcfbi->fb_mode_str && plat_data && plat_data->mode_str)
		mxcfbi->fb_mode_str = plat_data->mode_str;

	if (mxcfbi->fb_mode_str) {
		if (mxcfbi->ipu_di >= 0) {
			const struct fb_videomode *mode;
			struct fb_videomode m;
			int num, found = 0;

			dev_dbg(fbi->device, "Config display port %d\n",
					mxcfbi->ipu_di);

			INIT_LIST_HEAD(&fbi->modelist);

			if (mxc_disp_mode[mxcfbi->ipu_di].num_modes) {
				int i;
				mode = mxc_disp_mode[mxcfbi->ipu_di].mode;
				num = mxc_disp_mode[mxcfbi->ipu_di].num_modes;

				for (i = 0; i < num; i++) {
					/*
					 * FIXME now we do not support interlaced
					 * mode for ddc mode
					 */
					if ((mxc_disp_mode[mxcfbi->ipu_di].dev_mode
						& MXC_DISP_DDC_DEV) &&
						(mode[i].vmode & FB_VMODE_INTERLACED))
						continue;
					else
						fb_add_videomode(&mode[i], &fbi->modelist);
				}
			}

			if ((mxc_disp_mode[mxcfbi->ipu_di].dev_mode
				& MXC_DISP_DDC_DEV) &&
				!list_empty(&fbi->modelist)) {
				dev_dbg(fbi->device,
					"Look for video mode %s in ddc modelist\n",
					mxcfbi->fb_mode_str);
				/*
				 * For DDC mode, try to get compatible mode first.
				 * If get one, try to find nearest mode, otherwise,
				 * use first mode provide by DDC.
				 */
				ret = fb_find_mode(&fbi->var, fbi,
						mxcfbi->fb_mode_str, NULL, 0,
						NULL, mxcfbi->default_bpp);
				if (ret) {
					fb_var_to_videomode(&m, &fbi->var);
					mode = fb_find_nearest_mode(&m,
						&fbi->modelist);
					fb_videomode_to_var(&fbi->var, mode);
				} else {
					struct list_head *pos, *head;
					struct fb_modelist *modelist;

					head = &fbi->modelist;
					list_for_each(pos, head) {
						modelist = list_entry(pos,
							struct fb_modelist, list);
						m = modelist->mode;
						if (m.flag & FB_MODE_IS_FIRST)
							break;
					}
					/* if no first mode, use last one */
					mode = &m;
					fb_videomode_to_var(&fbi->var, mode);
				}
				found = 1;
			} else if (!list_empty(&fbi->modelist)) {
				dev_dbg(fbi->device,
					"Look for video mode %s in spec modelist\n",
					mxcfbi->fb_mode_str);
				/*
				 * For specific mode, try to get specified mode
				 * from fbi modelist.
				 */
				ret = fb_find_mode(&fbi->var, fbi,
						mxcfbi->fb_mode_str, mode, num,
						NULL, mxcfbi->default_bpp);
				if (ret == 1)
					found = 1;

			}
			/*
			 * if no DDC mode and spec mode found,
			 * try plat_data mode.
			 */
			if (!found) {
				dev_dbg(fbi->device,
					"Look for video mode %s in plat modelist\n",
					mxcfbi->fb_mode_str);
				if (plat_data && plat_data->mode
					&& plat_data->num_modes)
					ret = fb_find_mode(&fbi->var, fbi,
						mxcfbi->fb_mode_str,
						plat_data->mode,
						plat_data->num_modes,
						NULL,
						mxcfbi->default_bpp);
				else
					ret = fb_find_mode(&fbi->var, fbi,
						mxcfbi->fb_mode_str, NULL, 0,
						NULL, mxcfbi->default_bpp);
				if (ret)
					found = 1;
			}

			if (!found) {
				dev_err(fbi->device,
					"Not found any valid video mode");
				ret = -EINVAL;
				goto done;
			}

			/*added found mode to fbi modelist*/
			fb_var_to_videomode(&m, &fbi->var);
			fb_add_videomode(&m, &fbi->modelist);
		}
	}

	mxcfb_check_var(&fbi->var, fbi);

	mxcfb_set_fix(fbi);

	/* setup display */
	if (mxcfbi->ipu_di >= 0)
		if (mxcfb_pre_setup[mxcfbi->ipu_di])
			(mxcfb_pre_setup[mxcfbi->ipu_di])(fbi);

	fbi->var.activate |= FB_ACTIVATE_FORCE;
	acquire_console_sem();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &fbi->var);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
done:
	return ret;
}

/*!
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process.      The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, IPU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int mxcfb_probe(struct platform_device *pdev)
{
	struct fb_info *fbi;
	struct mxcfb_info *mxcfbi;
	struct resource *res;
	char *options;
	char name[] = "mxcdi0fb";
	int ret = 0;

	/*
	 * Initialize FB structures
	 */
	fbi = mxcfb_init_fbinfo(&pdev->dev, &mxcfb_ops);
	if (!fbi) {
		ret = -ENOMEM;
		goto err0;
	}
	mxcfbi = (struct mxcfb_info *)fbi->par;

	name[5] += pdev->id;
	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto err1;
	}

	if (options)
		mxcfb_option_setup(fbi, options);

	if (!g_dp_in_use) {
		mxcfbi->ipu_ch_irq = IPU_IRQ_BG_SYNC_EOF;
		mxcfbi->ipu_ch = MEM_BG_SYNC;
		mxcfbi->cur_blank = mxcfbi->next_blank = FB_BLANK_UNBLANK;
	} else {
		mxcfbi->ipu_ch_irq = IPU_IRQ_DC_SYNC_EOF;
		mxcfbi->ipu_ch = MEM_DC_SYNC;
		mxcfbi->cur_blank = mxcfbi->next_blank = FB_BLANK_POWERDOWN;
	}

	mxcfbi->ipu_di = pdev->id;
	mxcfbi->ipu_alp_ch_irq = -1;

	if (pdev->id == 0) {
		ipu_disp_set_global_alpha(mxcfbi->ipu_ch, true, 0x80);
		ipu_disp_set_color_key(mxcfbi->ipu_ch, false, 0);
		strcpy(fbi->fix.id, "DISP3 BG");

		if (!g_dp_in_use)
			mxcfbi->ipu_alp_ch_irq = IPU_IRQ_BG_ALPHA_SYNC_EOF;
		g_dp_in_use = true;
	} else if (pdev->id == 1) {
		strcpy(fbi->fix.id, "DISP3 BG - DI1");

		if (!g_dp_in_use)
			mxcfbi->ipu_alp_ch_irq = IPU_IRQ_BG_ALPHA_SYNC_EOF;
		g_dp_in_use = true;
	} else if (pdev->id == 2) {	/* Overlay */
		mxcfbi->ipu_ch_irq = IPU_IRQ_FG_SYNC_EOF;
		mxcfbi->ipu_alp_ch_irq = IPU_IRQ_FG_ALPHA_SYNC_EOF;
		mxcfbi->ipu_ch = MEM_FG_SYNC;
		mxcfbi->ipu_di = -1;
		mxcfbi->overlay = true;
		mxcfbi->cur_blank = mxcfbi->next_blank = FB_BLANK_POWERDOWN;

		strcpy(fbi->fix.id, "DISP3 FG");
	}

	mxcfb_info[pdev->id] = fbi;

	if (ipu_request_irq(mxcfbi->ipu_ch_irq, mxcfb_irq_handler, 0,
			    MXCFB_NAME, fbi) != 0) {
		dev_err(&pdev->dev, "Error registering BG irq handler.\n");
		ret = -EBUSY;
		goto err1;
	}
	ipu_disable_irq(mxcfbi->ipu_ch_irq);

	if (mxcfbi->ipu_alp_ch_irq != -1)
		if (ipu_request_irq(mxcfbi->ipu_alp_ch_irq,
					mxcfb_alpha_irq_handler, 0,
					MXCFB_NAME, fbi) != 0) {
			dev_err(&pdev->dev, "Error registering alpha irq "
					"handler.\n");
			ret = -EBUSY;
			goto err2;
		}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && res->end) {
		fbi->fix.smem_len = res->end - res->start + 1;
		fbi->fix.smem_start = res->start;
		fbi->screen_base = ioremap(fbi->fix.smem_start, fbi->fix.smem_len);
	}

	ret =  mxcfb_setup(fbi, pdev);
	if (ret < 0)
		goto err3;

	ret = register_framebuffer(fbi);
	if (ret < 0)
		goto err3;

	platform_set_drvdata(pdev, fbi);

	ret = device_create_file(fbi->dev, &dev_attr_fsl_disp_property);
	if (ret)
		dev_err(&pdev->dev, "Error %d on creating file\n", ret);

#ifdef CONFIG_LOGO
	fb_prepare_logo(fbi, 0);
	fb_show_logo(fbi, 0);
#endif

	return 0;
err3:
	if (mxcfbi->ipu_alp_ch_irq != -1)
		ipu_free_irq(mxcfbi->ipu_alp_ch_irq, fbi);
err2:
	ipu_free_irq(mxcfbi->ipu_ch_irq, fbi);
err1:
	fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
err0:
	return ret;
}

static int mxcfb_remove(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mxcfb_info *mxc_fbi = fbi->par;

	if (!fbi)
		return 0;

	mxcfb_blank(FB_BLANK_POWERDOWN, fbi);
	ipu_free_irq(mxc_fbi->ipu_ch_irq, fbi);
	mxcfb_unmap_video_memory(fbi);

	if (&fbi->cmap)
		fb_dealloc_cmap(&fbi->cmap);

	unregister_framebuffer(fbi);
	framebuffer_release(fbi);
	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxcfb_driver = {
	.driver = {
		   .name = MXCFB_NAME,
		   },
	.probe = mxcfb_probe,
	.remove = mxcfb_remove,
	.suspend = mxcfb_suspend,
	.resume = mxcfb_resume,
};

/*
 * Parse user specified options (`video=trident:')
 * example:
 * 	video=mxcdi0fb:RGB24, 1024x768M-16@60,bpp=16,noaccel
 */
static int mxcfb_option_setup(struct fb_info *info, char *options)
{
	struct mxcfb_info *mxcfbi = info->par;
	char *opt;

	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "RGB24", 5)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_RGB24;
			continue;
		}
		if (!strncmp(opt, "BGR24", 5)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_BGR24;
			continue;
		}
		if (!strncmp(opt, "GBR24", 5)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_GBR24;
			continue;
		}
		if (!strncmp(opt, "RGB565", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_RGB565;
			continue;
		}
		if (!strncmp(opt, "RGB666", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_RGB666;
			continue;
		}
		if (!strncmp(opt, "YUV444", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_YUV444;
			continue;
		}
		if (!strncmp(opt, "LVDS666", 7)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_LVDS666;
			continue;
		}
		if (!strncmp(opt, "YUYV16", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_YUYV;
			continue;
		}
		if (!strncmp(opt, "UYVY16", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_UYVY;
			continue;
		}
		if (!strncmp(opt, "YVYU16", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_YVYU;
			continue;
		}
		if (!strncmp(opt, "VYUY16", 6)) {
			mxcfbi->ipu_di_pix_fmt = IPU_PIX_FMT_VYUY;
			continue;
		}
		if (!strncmp(opt, "int_clk", 7)) {
			mxcfbi->ipu_int_clk = true;
			continue;
		}
		if (!strncmp(opt, "bpp=", 4))
			mxcfbi->default_bpp =
				simple_strtoul(opt + 4, NULL, 0);
		else
			mxcfbi->fb_mode_str = opt;
	}

	return 0;
}

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MXCFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
int __init mxcfb_init(void)
{
	return platform_driver_register(&mxcfb_driver);
}

void mxcfb_exit(void)
{
	platform_driver_unregister(&mxcfb_driver);
}

module_init(mxcfb_init);
module_exit(mxcfb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
