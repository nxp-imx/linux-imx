/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP main header file
 *
 * Copyright 2023-2026 NXP
 */

#ifndef __NXP_NEOISP_H
#define __NXP_NEOISP_H

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/media/nxp/nxp_neoisp.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "neoisp_hw.h"

#define NEOISP_NAME			"neoisp"

#define NEOISP_NODE_GROUPS_COUNT	8
#define NEOISP_MIN_W			64U
#define NEOISP_MIN_H			64U
#define NEOISP_MAX_W			4096U
#define NEOISP_MAX_H			4096U
#define NEOISP_MAX_BPP			4U
#define NEOISP_ALIGN_W			3
#define NEOISP_ALIGN_H			3
#define NEOISP_DEF_W			640U
#define NEOISP_DEF_H			480U

#define NEOISP_SUSPEND_TIMEOUT_MS	500

/*
 * 16 controls have been reserved for this driver for future extension, but
 * let's limit the related driver allocation to the effective number of controls
 * in use.
 */
enum neoisp_ctrls_e {
	NEOISP_CTRLS_QUERYCAP,
	NEOISP_CTRLS_SUPPORTED_PARAMS_BLOCKS,
	NEOISP_CTRLS_COUNT,
};

static inline bool format_is_monochrome(u32 format)
{
	return format == V4L2_PIX_FMT_GREY || format == V4L2_PIX_FMT_Y10 ||
		format == V4L2_PIX_FMT_Y12 || format == V4L2_PIX_FMT_Y14 ||
		format == V4L2_PIX_FMT_Y16 || format == V4L2_PIX_FMT_Y16_BE;
}

#define NEOISP_COLORSPACE_MASK(colorspace) BIT(colorspace)

#define NEOISP_COLORSPACE_MASK_JPEG \
	NEOISP_COLORSPACE_MASK(V4L2_COLORSPACE_JPEG)
#define NEOISP_COLORSPACE_MASK_SMPTE170M \
	NEOISP_COLORSPACE_MASK(V4L2_COLORSPACE_SMPTE170M)
#define NEOISP_COLORSPACE_MASK_REC709 \
	NEOISP_COLORSPACE_MASK(V4L2_COLORSPACE_REC709)
#define NEOISP_COLORSPACE_MASK_SRGB \
	NEOISP_COLORSPACE_MASK(V4L2_COLORSPACE_SRGB)
#define NEOISP_COLORSPACE_MASK_RAW \
	NEOISP_COLORSPACE_MASK(V4L2_COLORSPACE_RAW)

/*
 * JPEG, SMPTE170M and REC709 colorspaces are fundamentally sRGB underneath
 * with different YCbCr encodings. All these colorspaces are defined for
 * every YUV/RGB video capture formats.
 */
#define NEOISP_COLORSPACE_MASK_ALL_SRGB (NEOISP_COLORSPACE_MASK_JPEG	  | \
					 NEOISP_COLORSPACE_MASK_SRGB	  | \
					 NEOISP_COLORSPACE_MASK_SMPTE170M | \
					 NEOISP_COLORSPACE_MASK_REC709)

enum neoisp_fmt_type_e {
	NEOISP_FMT_VIDEO_CAPTURE = BIT(0),
	NEOISP_FMT_VIDEO_OUTPUT = BIT(1),
	NEOISP_FMT_META_CAPTURE = BIT(2),
	NEOISP_FMT_META_OUTPUT = BIT(3),
};

enum neoisp_node_e {
	NEOISP_INPUT0_NODE,
	NEOISP_INPUT1_NODE,
	NEOISP_PARAMS_NODE,
	NEOISP_FRAME_NODE,
	NEOISP_IR_NODE,
	NEOISP_STATS_NODE,
	NEOISP_NODES_COUNT
};

struct neoisp_fmt_s {
	u32 fourcc;
	u32 align;
	u32 bit_depth;
	u32 num_planes;
	u8 pl_divisors[VB2_MAX_PLANES];
	u8 bpp_enc;
	u8 is_rgb;
	u32 colorspace_mask;
	enum v4l2_colorspace colorspace_default;
	enum neoisp_fmt_type_e type;
};

struct neoisp_dev_s;

struct neoisp_context_s {
	struct neoisp_hw_s hw;
	struct neoisp_vignetting_table_mem_params_s vig;
	struct neoisp_drc_global_tonemap_mem_params_s gtm;
	struct neoisp_drc_local_tonemap_mem_params_s ltm;
};

/*
 * struct neoisp_context_ops_s - Context related operations across HW revisions
 *
 * @get_irq_status: Read irq status register
 * @set_irq_enable: Set irq enable register
 * @clear_irq: Clear irq status register
 * @adjust_gain: Callback to adjust gain for data alignment
 */
struct neoisp_context_ops_s {
	u32 (*get_irq_status)(struct neoisp_dev_s *neoispd);
	void (*set_irq_enable)(struct neoisp_dev_s *neoispd, u32 val);
	void (*clear_irq)(struct neoisp_dev_s *neoispd, u32 val);
	void (*adjust_gain)(struct neoisp_context_s *ctx, u32 ibpp);
};

struct isp_block_map_s {
	u32 vignetting_table;
	u32 drc_global_tonemap;
	u32 drc_global_hist_roi0;
	u32 drc_global_hist_roi1;
	u32 drc_local_tonemap;
	u32 drc_local_sum;
};

/*
 * struct neoisp_info_s - ISP Hardware various information
 *
 * @capabilities: ISP hardware and driver capabilities
 * @blocks_list: The list of ISP units supported by an ISP version
 *
 * This structure contains information about the ISP specific model,
 * like parameters block list or SoC integration specifities.
 */
struct neoisp_info_s {
	u32 capabilities;
	const unsigned int *blocks_list;
};

struct neoisp_node_desc_s {
	const char *ent_name;
	enum v4l2_buf_type buf_type;
	u32 caps;
	u32 link_flags;
};

/*
 * Structure to describe a single node /dev/video<N> which represents a single
 * input or output queue to the neoisp device.
 */
struct neoisp_node_s {
	u32 id;
	s32 vfl_dir;
	enum v4l2_buf_type buf_type;
	struct video_device vfd;
	struct media_pad pad;
	struct media_intf_devnode *intf_devnode;
	struct media_link *intf_link;
	struct neoisp_node_group_s *node_group;
	/* Video device lock */
	struct mutex node_lock;
	/* vb2_queue lock */
	struct mutex queue_lock;
	struct list_head ready_queue;
	struct vb2_queue queue;
	struct v4l2_format format;
	const struct neoisp_fmt_s *neoisp_format;
	struct v4l2_rect crop;
};

struct neoisp_node_group_s {
	u32 id;
	u32 frame_sequence;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *ctrls[NEOISP_CTRLS_COUNT];
	struct media_device mdev;
	struct neoisp_dev_s *neoisp_dev;
	struct neoisp_node_s node[NEOISP_NODES_COUNT];
	u32 streaming_map; /* Bitmap of which nodes are streaming */
	struct media_pad pad[NEOISP_NODES_COUNT]; /* Output pads first */
	dma_addr_t params_dma_addr;
	u32 *dummy_buf;
	dma_addr_t dummy_dma;
	u32 dummy_size;
	struct neoisp_context_s *context;
};

struct neoisp_buffer_s {
	struct vb2_v4l2_buffer vb;
	struct list_head ready_list;
};

static inline struct neoisp_buffer_s *to_neoisp_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct neoisp_buffer_s, vb);
}

/* Catch currently running or queued jobs on the neoisp hw */
struct neoisp_job_s {
	struct neoisp_node_group_s *node_group;
	struct neoisp_buffer_s *buf[NEOISP_NODES_COUNT];
};

/* Records a job configuration */
struct neoisp_job_desc_s {
	struct list_head queue;
	struct neoisp_node_group_s *node_group;
	struct neoisp_buffer_s *buffers[NEOISP_NODES_COUNT];
};

struct neoisp_dev_s {
	struct device *dev;
	struct neoisp_info_s *info;
	void __iomem *mmio;
	void __iomem *mmio_tcm;
	struct clk_bulk_data *clks;
	s32 num_clks;
	struct neoisp_node_group_s node_group[NEOISP_NODE_GROUPS_COUNT];
	struct neoisp_job_s queued_job;
	bool hw_busy; /* Non-zero if a job is queued or is being started */
	u8 media_registered;
	struct list_head job_queue;
	/* Protects "hw_busy" flag, streaming_map and job_queue */
	spinlock_t hw_lock;
	struct dentry *debugfs_entry;
	struct debugfs_regset32 *regset;
};

#if IS_ENABLED(CONFIG_DEBUG_FS)
void neoisp_debugfs_init(struct neoisp_dev_s *neoispd);
void neoisp_debugfs_exit(struct neoisp_dev_s *neoispd);
#else
static inline void neoisp_debugfs_init(struct neoisp_dev_s *neoispd)
{
}

static inline void neoisp_debugfs_exit(struct neoisp_dev_s *neoispd)
{
}
#endif

static inline int neoisp_node_link_is_enabled(struct neoisp_node_s *node)
{
	return (node->intf_link->flags & MEDIA_LNK_FL_ENABLED);
}

static inline u32 neoisp_rd(struct neoisp_dev_s *neoispd, u32 offset)
{
	return readl(neoispd->mmio + offset);
}

static inline void neoisp_wr(struct neoisp_dev_s *neoispd, u32 offset, u32 val)
{
	writel(val, neoispd->mmio + offset);
}

const struct neoisp_fmt_s *neoisp_find_video_capture_format(u32 pixel_format);

#endif /* __NXP_NEOISP_H */
