/*
 * Freescale STMP378X PxP driver
 *
 * Author: Matt Porter <mporter@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008-2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

struct pxps {
	struct platform_device *pdev;
	struct resource *res;
	int irq;
	void __iomem *regs;

	spinlock_t lock;
	struct mutex mutex;
	int users;

	struct video_device *vdev;

	struct videobuf_queue s0_vbq;
	struct videobuf_buffer *active;
	struct list_head outq;

	int output;
	u32 *outb;
	dma_addr_t outb_phys;

	/* Current S0 configuration */
	struct pxp_data_format *s0_fmt;
	u32 s0_width;
	u32 s0_height;
	u32 s0_bgcolor;
	u32 s0_chromakey;

	struct v4l2_framebuffer fb;
	struct v4l2_rect drect;
	struct v4l2_rect srect;

	/* Transformation support */
	int scaling;
	int hflip;
	int vflip;
	int rotate;
	int yuv;

	/* Output overlay support */
	int overlay_state;
	int global_alpha_state;
	u8  global_alpha;
	int local_alpha_state;
	int s1_chromakey_state;
	u32 s1_chromakey;
};

struct pxp_data_format {
	char *name;
	unsigned int bpp;
	u32 fourcc;
	enum v4l2_colorspace colorspace;
	u32 ctrl_s0_fmt;
};

extern int stmp3xxxfb_get_info(struct fb_var_screeninfo *var,
				struct fb_fix_screeninfo *fix);
extern void stmp3xxxfb_cfg_pxp(int enable, dma_addr_t pxp_phys);
