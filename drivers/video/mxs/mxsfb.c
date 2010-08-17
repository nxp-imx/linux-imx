/*
 * Freescale MXS framebuffer driver
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>

#include <mach/hardware.h>
#include <mach/regs-lcdif.h>
#include <mach/clock.h>
#include <mach/lcdif.h>

#define NUM_SCREENS	1

enum {
	F_DISABLE = 0,
	F_ENABLE,
	F_REENABLE,
};

struct mxs_fb_data {
	struct fb_info info;
	struct mxs_platform_fb_data *pdata;
	struct work_struct work;
	struct mutex blank_mutex;
	u32 state;
	u32 task_state;
	ssize_t mem_size;
	ssize_t map_size;
	dma_addr_t phys_start;
	dma_addr_t cur_phys;
	int irq;
	unsigned long regbase;
	void *virt_start;
	struct device *dev;
	wait_queue_head_t vsync_wait_q;
	u32 vsync_count;
	void *par;
};

/* forward declaration */
static int mxsfb_blank(int blank, struct fb_info *info);
static unsigned char *default_panel_name;
static struct mxs_fb_data *cdata;
static void init_timings(struct mxs_fb_data *data);

static void mxsfb_enable_controller(struct mxs_fb_data *data)
{
	struct mxs_platform_fb_entry *pentry = data->pdata->cur;

	if (!data || !data->pdata || !data->pdata->cur)
		return;

	mxs_init_lcdif();
	init_timings(data);
	pentry->init_panel(data->dev, data->phys_start,
			   data->info.fix.smem_len, data->pdata->cur);
	pentry->run_panel();

	if (pentry->blank_panel)
		pentry->blank_panel(FB_BLANK_UNBLANK);
}

static void mxsfb_disable_controller(struct mxs_fb_data *data)
{
	struct mxs_platform_fb_entry *pentry = data->pdata->cur;

	if (!data || !data->pdata || !data->pdata->cur)
		return;

	if (pentry->blank_panel)
		pentry->blank_panel(FB_BLANK_POWERDOWN);

	if (pentry->stop_panel)
		pentry->stop_panel();
	pentry->release_panel(data->dev, pentry);
}

static void set_controller_state(struct mxs_fb_data *data, u32 state)
{
	struct mxs_platform_fb_entry *pentry = data->pdata->cur;
	struct fb_info *info = &data->info;
	u32 old_state;

	mutex_lock(&data->blank_mutex);
	old_state = data->state;
	pr_debug("%s, old_state %d, state %d\n", __func__, old_state, state);

	switch (state) {
	case F_DISABLE:
		/*
		 * Disable controller
		 */
		if (old_state != F_DISABLE) {
			data->state = F_DISABLE;
			mxsfb_disable_controller(data);
		}
		break;

	case F_REENABLE:
		/*
		 * Re-enable the controller when panel changed.
		 */
		if (old_state == F_ENABLE) {
			mxsfb_disable_controller(data);

			pentry = data->pdata->cur = data->pdata->next;
			info->fix.smem_len = pentry->y_res * pentry->x_res *
			    pentry->bpp / 8;
			info->screen_size = info->fix.smem_len;
			memset((void *)info->screen_base, 0, info->screen_size);

			mxsfb_enable_controller(data);

			data->state = F_ENABLE;
		} else if (old_state == F_DISABLE) {
			pentry = data->pdata->cur = data->pdata->next;
			info->fix.smem_len = pentry->y_res * pentry->x_res *
			    pentry->bpp / 8;
			info->screen_size = info->fix.smem_len;
			memset((void *)info->screen_base, 0, info->screen_size);

			data->state = F_DISABLE;
		}
		break;

	case F_ENABLE:
		if (old_state != F_ENABLE) {
			data->state = F_ENABLE;
			mxsfb_enable_controller(data);
		}
		break;
	}
	mutex_unlock(&data->blank_mutex);

}

static void mxsfb_task(struct work_struct *work)
{
	struct mxs_fb_data *data = container_of(work, struct mxs_fb_data, work);

	u32 state = xchg(&data->task_state, -1);
	pr_debug("%s: state = %d, data->task_state = %d\n",
		 __func__, state, data->task_state);

	set_controller_state(data, state);
}

static void mxs_schedule_work(struct mxs_fb_data *data, u32 state)
{
	unsigned long flags;

	local_irq_save(flags);

	data->task_state = state;
	schedule_work(&data->work);

	local_irq_restore(flags);
}

static irqreturn_t lcd_irq_handler(int irq, void *dev_id)
{
	struct mxs_fb_data *data = dev_id;
	u32 status_lcd = __raw_readl(data->regbase + HW_LCDIF_CTRL1);
	pr_debug("%s: irq %d\n", __func__, irq);

	if (status_lcd & BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ) {
		pr_debug("%s: VSYNC irq\n", __func__);
		data->vsync_count++;
		__raw_writel(BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ,
			     data->regbase + HW_LCDIF_CTRL1_CLR);
		wake_up_interruptible(&data->vsync_wait_q);
	}
	if (status_lcd & BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ) {
		pr_debug("%s: frame done irq\n", __func__);
		__raw_writel(BM_LCDIF_CTRL1_CUR_FRAME_DONE_IRQ,
			     data->regbase + HW_LCDIF_CTRL1_CLR);
		data->vsync_count++;
	}
	if (status_lcd & BM_LCDIF_CTRL1_UNDERFLOW_IRQ) {
		pr_debug("%s: underflow irq\n", __func__);
		__raw_writel(BM_LCDIF_CTRL1_UNDERFLOW_IRQ,
			     data->regbase + HW_LCDIF_CTRL1_CLR);
	}
	if (status_lcd & BM_LCDIF_CTRL1_OVERFLOW_IRQ) {
		pr_debug("%s: overflow irq\n", __func__);
		__raw_writel(BM_LCDIF_CTRL1_OVERFLOW_IRQ,
			     data->regbase + HW_LCDIF_CTRL1_CLR);
	}
	return IRQ_HANDLED;
}

static struct fb_var_screeninfo mxsfb_default __devinitdata = {
	.activate = FB_ACTIVATE_TEST,
	.height = -1,
	.width = -1,
	.pixclock = 20000,
	.left_margin = 64,
	.right_margin = 64,
	.upper_margin = 32,
	.lower_margin = 32,
	.hsync_len = 64,
	.vsync_len = 2,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo mxsfb_fix __devinitdata = {
	.id = "mxsfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

int mxsfb_get_info(struct fb_var_screeninfo *var, struct fb_fix_screeninfo *fix)
{
	if (!cdata)
		return -ENODEV;

	*var = cdata->info.var;
	*fix = cdata->info.fix;
	return 0;
}

void mxsfb_cfg_pxp(int enable, dma_addr_t pxp_phys)
{
	if (enable)
		cdata->pdata->cur->pan_display(pxp_phys);
	else
		cdata->pdata->cur->pan_display(cdata->cur_phys);
}

static int mxsfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;

	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off < info->fix.smem_len)
		return dma_mmap_writecombine(data->dev, vma,
					     data->virt_start,
					     data->phys_start,
					     info->fix.smem_len);
	else
		return -EINVAL;
}

static int mxsfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to
	 *      (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    uses offset = 0 && length = RAMDAC register width.
	 *    var->{color}.offset is 0
	 *    var->{color}.length contains widht of DAC
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to
	 *      (red << red.offset) | (green << green.offset) |
	 *      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {

		if (regno >= 16)
			return 1;

		((u32 *) (info->pseudo_palette))[regno] =
		    (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
	}
	return 0;
}

static inline u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return length;
}

static int get_matching_pentry(struct mxs_platform_fb_entry *pentry,
			       void *data, int ret_prev)
{
	struct fb_var_screeninfo *info = data;
	pr_debug("%s: %d:%d:%d vs %d:%d:%d\n", __func__,
		 pentry->x_res, pentry->y_res, pentry->bpp,
		 info->yres, info->xres, info->bits_per_pixel);
	if (pentry->x_res == info->yres && pentry->y_res == info->xres &&
	    pentry->bpp == info->bits_per_pixel)
		ret_prev = (int)pentry;
	return ret_prev;
}

static int get_matching_pentry_by_name(struct mxs_platform_fb_entry *pentry,
				       void *data, int ret_prev)
{
	unsigned char *name = data;
	if (!strcmp(pentry->name, name))
		ret_prev = (int)pentry;
	return ret_prev;
}

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 * XXX: REVISIT
 */
static int mxsfb_set_par(struct fb_info *info)
{
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;
	struct mxs_platform_fb_data *pdata = data->pdata;
	struct mxs_platform_fb_entry *pentry;
	pentry = (void *)mxs_lcd_iterate_pdata(pdata,
					       get_matching_pentry, &info->var);

	dev_dbg(data->dev, "%s: xres %d, yres %d, bpp %d\n",
		__func__,
		info->var.xres, info->var.yres, info->var.bits_per_pixel);
	if (!pentry)
		return -EINVAL;

	info->fix.line_length = get_line_length(info->var.xres_virtual,
						info->var.bits_per_pixel);

	if (pentry == pdata->cur || !pdata->cur)
		return 0;

	/* init next panel */
	pdata->next = pentry;

	set_controller_state(data, F_REENABLE);

	return 0;
}

static int mxsfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 line_length;
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;
	struct mxs_platform_fb_data *pdata = data->pdata;

	/*
	 *  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally
	 */

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	pr_debug("%s: xres %d, yres %d, bpp %d\n", __func__,
		 var->xres, var->yres, var->bits_per_pixel);
	/*
	 *  Some very basic checks
	 */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	line_length = get_line_length(var->xres_virtual, var->bits_per_pixel);
	dev_dbg(data->dev,
		"line_length %d, var->yres_virtual %d, data->mem_size %d\n",
		line_length, var->yres_virtual, data->mem_size);
	if (line_length * var->yres_virtual > data->map_size)
		return -ENOMEM;

	if (!mxs_lcd_iterate_pdata(pdata, get_matching_pentry, var))
		return -EINVAL;

	if (var->bits_per_pixel == 16) {
		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
	} else {
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
	}

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	return 0;
}

static int mxsfb_wait_for_vsync(u32 channel, struct fb_info *info)
{
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;
	u32 count = data->vsync_count;
	int ret = 0;

	__raw_writel(BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN,
		     data->regbase + HW_LCDIF_CTRL1_SET);
	ret = wait_event_interruptible_timeout(data->vsync_wait_q,
					       count != data->vsync_count,
					       HZ / 10);
	__raw_writel(BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN,
		     data->regbase + HW_LCDIF_CTRL1_CLR);
	if (!ret) {
		dev_err(data->dev, "wait for vsync timed out\n");
		ret = -ETIMEDOUT;
	}
	return ret;
}

static int mxsfb_ioctl(struct fb_info *info, unsigned int cmd,
		       unsigned long arg)
{
	u32 channel = 0;
	int ret = -EINVAL;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (!get_user(channel, (__u32 __user *) arg))
			ret = mxsfb_wait_for_vsync(channel, info);
		break;
	default:
		break;
	}
	return ret;
}

static int mxsfb_blank(int blank, struct fb_info *info)
{
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;
	int ret = 0;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		pr_debug("%s: FB_BLANK_POWERDOWN\n", __func__);
		mxs_schedule_work(data, F_DISABLE);
		break;

	case FB_BLANK_UNBLANK:
		pr_debug("%s: FB_BLANK_UNBLANK\n", __func__);
		mxs_schedule_work(data, F_ENABLE);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static int mxsfb_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct mxs_fb_data *data = (struct mxs_fb_data *)info;
	int ret = 0;

	pr_debug("%s: var->xoffset %d, info->var.xoffset %d\n",
		 __func__, var->xoffset, info->var.xoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) {
		ret = -EINVAL;
		goto out;
	}

	if (!data->pdata->cur->pan_display) {
		ret = -EINVAL;
		goto out;
	}

	/* update framebuffer visual */
	data->cur_phys = data->phys_start +
	    info->fix.line_length * var->yoffset;
	data->pdata->cur->pan_display(data->cur_phys);
out:
	return ret;
}

static struct fb_ops mxsfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxsfb_check_var,
	.fb_set_par = mxsfb_set_par,
	.fb_mmap = mxsfb_mmap,
	.fb_setcolreg = mxsfb_setcolreg,
	.fb_ioctl = mxsfb_ioctl,
	.fb_blank = mxsfb_blank,
	.fb_pan_display = mxsfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static void init_timings(struct mxs_fb_data *data)
{
	unsigned phase_time;
	unsigned timings;

	/* Just use a phase_time of 1. As optimal as it gets, now. */
	phase_time = 1;

	/* Program all 4 timings the same */
	timings = phase_time;
	timings |= timings << 8;
	timings |= timings << 16;
	__raw_writel(timings, data->regbase + HW_LCDIF_TIMING);
}

#ifdef CONFIG_CPU_FREQ

struct mxsfb_notifier_block {
	struct mxs_fb_data *fb_data;
	struct notifier_block nb;
};

static int mxsfb_notifier(struct notifier_block *self,
			  unsigned long phase, void *p)
{
	struct mxsfb_notifier_block *block =
	    container_of(self, struct mxsfb_notifier_block, nb);
	struct mxs_fb_data *data = block->fb_data;
	struct mxs_platform_fb_entry *pentry = data->pdata->cur;
	u32 old_state = data->state;

	if (!data || !data->pdata || !data->pdata->cur)
		return NOTIFY_BAD;

	/* REVISIT */
	switch (phase) {
	case CPUFREQ_PRECHANGE:
		if (old_state == F_ENABLE)
			if (pentry->blank_panel)
				pentry->blank_panel(FB_BLANK_POWERDOWN);
		break;

	case CPUFREQ_POSTCHANGE:
		if (old_state == F_ENABLE)
			if (pentry->blank_panel)
				pentry->blank_panel(FB_BLANK_UNBLANK);
		break;

	default:
		dev_dbg(data->dev, "didn't handle notify %ld\n", phase);
	}

	return NOTIFY_DONE;
}

static struct mxsfb_notifier_block mxsfb_nb = {
	.nb = {
	       .notifier_call = mxsfb_notifier,
	       },
};
#endif /* CONFIG_CPU_FREQ */

static int get_max_memsize(struct mxs_platform_fb_entry *pentry,
			   void *data, int ret_prev)
{
	struct mxs_fb_data *fbdata = data;
	int sz = pentry->x_res * pentry->y_res * pentry->bpp / 8;
	fbdata->mem_size = sz < ret_prev ? ret_prev : sz;
	pr_debug("%s: mem_size now %d\n", __func__, fbdata->mem_size);
	return fbdata->mem_size;
}

static int __devinit mxsfb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxs_fb_data *data;
	struct resource *res;
	struct fb_info *info;
	struct mxs_platform_fb_data *pdata = pdev->dev.platform_data;
	struct mxs_platform_fb_entry *pentry = NULL;

	if (pdata == NULL) {
		ret = -ENODEV;
		goto out;
	}

	if (default_panel_name) {
		pentry = (void *)mxs_lcd_iterate_pdata(pdata,
						       get_matching_pentry_by_name,
						       default_panel_name);
		if (pentry) {
			mxs_lcd_move_pentry_up(pentry, pdata);
			pdata->cur = pentry;
		}
	}
	if (!default_panel_name || !pentry)
		pentry = pdata->cur;
	if (!pentry || !pentry->init_panel || !pentry->run_panel ||
	    !pentry->release_panel) {
		ret = -EINVAL;
		goto out;
	}

	data =
	    (struct mxs_fb_data *)framebuffer_alloc(sizeof(struct mxs_fb_data) +
						    sizeof(u32) * 256 -
						    sizeof(struct fb_info),
						    &pdev->dev);
	if (data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	cdata = data;
	data->dev = &pdev->dev;
	data->pdata = pdata;
	platform_set_drvdata(pdev, data);
	info = &data->info;

	dev_dbg(&pdev->dev, "resolution %dx%d, bpp %d\n", pentry->x_res,
		pentry->y_res, pentry->bpp);

	mxs_lcd_iterate_pdata(pdata, get_max_memsize, data);

	data->map_size = PAGE_ALIGN(data->mem_size) * NUM_SCREENS;
	dev_dbg(&pdev->dev, "memory to allocate: %d\n", data->map_size);

	data->virt_start = dma_alloc_writecombine(&pdev->dev,
						  data->map_size,
						  &data->phys_start,
						  GFP_KERNEL);

	if (data->virt_start == NULL) {
		ret = -ENOMEM;
		goto out_dma;
	}
	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", data->virt_start,
		data->phys_start);
	mutex_init(&data->blank_mutex);
	INIT_WORK(&data->work, mxsfb_task);
	data->state = F_ENABLE;

	mxsfb_default.bits_per_pixel = pentry->bpp;
	/* NB: rotated */
	mxsfb_default.xres = pentry->y_res;
	mxsfb_default.yres = pentry->x_res;
	mxsfb_default.xres_virtual = pentry->y_res;
	mxsfb_default.yres_virtual = data->map_size /
	    (pentry->y_res * pentry->bpp / 8);
	if (mxsfb_default.yres_virtual >= mxsfb_default.yres * 2)
		mxsfb_default.yres_virtual = mxsfb_default.yres * 2;
	else
		mxsfb_default.yres_virtual = mxsfb_default.yres;

	mxsfb_fix.smem_start = data->phys_start;
	mxsfb_fix.smem_len = pentry->y_res * pentry->x_res * pentry->bpp / 8;
	mxsfb_fix.ypanstep = 1;

	switch (pentry->bpp) {
	case 32:
	case 24:
		mxsfb_default.red.offset = 16;
		mxsfb_default.red.length = 8;
		mxsfb_default.green.offset = 8;
		mxsfb_default.green.length = 8;
		mxsfb_default.blue.offset = 0;
		mxsfb_default.blue.length = 8;
		break;

	case 16:
		mxsfb_default.red.offset = 11;
		mxsfb_default.red.length = 5;
		mxsfb_default.green.offset = 5;
		mxsfb_default.green.length = 6;
		mxsfb_default.blue.offset = 0;
		mxsfb_default.blue.length = 5;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n", pentry->bpp);
		ret = -EINVAL;
		goto out_dma;
	}

	info->screen_base = data->virt_start;
	info->fbops = &mxsfb_ops;
	info->var = mxsfb_default;
	info->fix = mxsfb_fix;
	info->pseudo_palette = &data->par;
	data->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	init_waitqueue_head(&data->vsync_wait_q);
	data->vsync_count = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma;
	}
	data->regbase = (unsigned long)IO_ADDRESS(res->start);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma;
	}
	data->irq = res->start;

	mxsfb_check_var(&info->var, info);

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_cmap;

	mxsfb_set_par(info);

	mxs_init_lcdif();
	ret = pentry->init_panel(data->dev, data->phys_start,
				 mxsfb_fix.smem_len, pentry);
	if (ret) {
		dev_err(&pdev->dev, "cannot initialize LCD panel\n");
		goto out_panel;
	}
	dev_dbg(&pdev->dev, "LCD panel initialized\n");
	init_timings(data);

	ret = request_irq(data->irq, lcd_irq_handler, 0, "fb_irq", data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			data->irq, ret);
		goto out_panel;
	}
	ret = register_framebuffer(info);
	if (ret)
		goto out_irq;

	pentry->run_panel();
	/* REVISIT: temporary workaround for MX23EVK */
	mxsfb_disable_controller(data);
	mxsfb_enable_controller(data);
	data->cur_phys = data->phys_start;
	dev_dbg(&pdev->dev, "LCD running now\n");

#ifdef CONFIG_CPU_FREQ
	mxsfb_nb.fb_data = data;
	cpufreq_register_notifier(&mxsfb_nb.nb, CPUFREQ_TRANSITION_NOTIFIER);
#endif /* CONFIG_CPU_FREQ */

	goto out;

out_irq:
	free_irq(data->irq, data);
out_panel:
	fb_dealloc_cmap(&info->cmap);
out_cmap:
	dma_free_writecombine(&pdev->dev, data->map_size, data->virt_start,
			      data->phys_start);
out_dma:
	kfree(data);
out:
	return ret;
}

static int mxsfb_remove(struct platform_device *pdev)
{
	struct mxs_fb_data *data = platform_get_drvdata(pdev);

	set_controller_state(data, F_DISABLE);

	unregister_framebuffer(&data->info);
	framebuffer_release(&data->info);
	fb_dealloc_cmap(&data->info.cmap);
	free_irq(data->irq, data);
	dma_free_writecombine(&pdev->dev, data->map_size, data->virt_start,
			      data->phys_start);
	kfree(data);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int mxsfb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mxs_fb_data *data = platform_get_drvdata(pdev);

	set_controller_state(data, F_DISABLE);

	return 0;
}

static int mxsfb_resume(struct platform_device *pdev)
{
	struct mxs_fb_data *data = platform_get_drvdata(pdev);

	set_controller_state(data, F_ENABLE);
	return 0;
}
#else
#define mxsfb_suspend	NULL
#define	mxsfb_resume	NULL
#endif

static struct platform_driver mxsfb_driver = {
	.probe = mxsfb_probe,
	.remove = mxsfb_remove,
	.suspend = mxsfb_suspend,
	.resume = mxsfb_resume,
	.driver = {
		   .name = "mxs-fb",
		   .owner = THIS_MODULE,
		   },
};

static int __init mxsfb_init(void)
{
	return platform_driver_register(&mxsfb_driver);
}

static void __exit mxsfb_exit(void)
{
	platform_driver_unregister(&mxsfb_driver);
}

module_init(mxsfb_init);
module_exit(mxsfb_exit);

/*
 * LCD panel select
 */
static int __init default_panel_select(char *str)
{
	default_panel_name = str;
	return 0;
}

__setup("lcd_panel=", default_panel_select);

MODULE_AUTHOR("Vitaly Wool <vital@embeddedalley.com>");
MODULE_DESCRIPTION("MXS Framebuffer Driver");
MODULE_LICENSE("GPL");
