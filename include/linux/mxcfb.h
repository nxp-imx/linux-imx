/*
 * Copyright 2004-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * @file linux/mxcfb.h
 *
 * @brief Global header file for the MXC Frame buffer
 *
 * @ingroup Framebuffer
 */
#ifndef __LINUX_MXCFB_H__
#define __LINUX_MXCFB_H__

#include <uapi/linux/mxcfb.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/dma-fence.h>

extern struct fb_videomode mxcfb_modedb[];
extern int mxcfb_modedb_sz;

enum {
	MXC_DISP_SPEC_DEV = 0,
	MXC_DISP_DDC_DEV = 1,
};

enum {
	MXCFB_REFRESH_OFF,
	MXCFB_REFRESH_AUTO,
	MXCFB_REFRESH_PARTIAL,
};

#if IS_ENABLED(CONFIG_FB_FENCE)
/* fb fence event */
struct fb_fence_event {
	struct dma_fence fence;
	struct list_head link;
};

struct fb_fence_context
{
	struct fb_info *fb_info;
	unsigned int fence_context;
	spinlock_t fence_lock;
	struct list_head present_queue;
	struct list_head release_queue;
	struct fb_fence_event *on_screen;
	unsigned long fence_seqno;
	char driver_name[32];
	char timeline_name[32];
	struct mutex mutex;

	int (*update_screen)(int64_t dma_address,
		struct fb_var_screeninfo *screeninfo,
		struct fb_info *fb_info);
};

struct fb_fence_commit
{
	struct work_struct work;
	struct fb_fence_context *context;

	// physical address.
	unsigned long smem_start;
	struct fb_var_screeninfo screeninfo;

	struct fb_fence_event *in_fence;
	// used for display.
	struct fb_fence_event *present_fence;
	// used for overlay.
	struct fb_fence_event *release_fence;
};

bool fb_handle_fence(struct fb_fence_context *ctx);
int fb_update_overlay(struct fb_fence_context *context, struct mxcfb_datainfo *buffer);
int fb_present_screen(struct fb_fence_context *context, struct mxcfb_datainfo *buffer);
int fb_init_fence_context(struct fb_fence_context *context, const char *driver, struct fb_info *fbi,
                int(*init)(int64_t, struct fb_var_screeninfo *, struct fb_info *));

#endif

int mxcfb_set_refresh_mode(struct fb_info *fbi, int mode,
			   struct mxcfb_rect *update_region);
int mxc_elcdif_frame_addr_setup(dma_addr_t phys);
void mxcfb_elcdif_register_mode(const struct fb_videomode *modedb,
		int num_modes, int dev_mode);

#endif
