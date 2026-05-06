/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP context definition
 *
 * Copyright 2023-2026 NXP
 */

#ifndef __NXP_NEOISP_CTX_H
#define __NXP_NEOISP_CTX_H

#include <linux/media/nxp/nxp_neoisp.h>

#include "neoisp.h"
#include "neoisp_regs.h"

#define NEOISP_HDR_SHIFT_RADIX	5  /* Hdr decompress block ratio field format is u7.5 */

/* Block offset */
#define ISP_OFF_POS		0UL
#define ISP_OFF_MASK		(0xFFFFUL << ISP_OFF_POS)
#define ISP_GET_OFF(x)		(((x) & ISP_OFF_MASK) >> ISP_OFF_POS)
#define ISP_OFF(x)		(((x) << ISP_OFF_POS) & ISP_OFF_MASK)

/* Block size */
#define ISP_SZ_POS		16UL
#define ISP_SZ_MASK		(0xFFFFUL << ISP_SZ_POS)
#define ISP_GET_SZ(x)		(((x) & ISP_SZ_MASK) >> ISP_SZ_POS)
#define ISP_SZ(x)		(((x) << ISP_SZ_POS) & ISP_SZ_MASK)

#define ISP_MAP_TUPLE(x, y, z)	(ISP_OFF((x)) | ISP_SZ(((y) * sizeof(z))))

enum isp_block_map_e {
	NEO_CTEMP_R_SUM_MAP = ISP_MAP_TUPLE(0x0, NEO_CTEMP_R_SUM_CNT, u32),
	NEO_CTEMP_G_SUM_MAP = ISP_MAP_TUPLE(0x100, NEO_CTEMP_G_SUM_CNT, u32),
	NEO_CTEMP_B_SUM_MAP = ISP_MAP_TUPLE(0x200, NEO_CTEMP_B_SUM_CNT, u32),
	NEO_CTEMP_PIX_CNT_MAP = ISP_MAP_TUPLE(0x300, NEO_CTEMP_PIX_CNT_CNT, u16),
	NEO_RGBIR_HIST_MAP = ISP_MAP_TUPLE(0x400, NEO_RGBIR_HIST_CNT, u32),
	NEO_HIST_STAT_MAP = ISP_MAP_TUPLE(0x800, NEO_HIST_STAT_CNT, u32),

	NEO_DRC_GLOBAL_HIST_ROI0_MAP = ISP_MAP_TUPLE(0x1000, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),
	NEO_DRC_GLOBAL_HIST_ROI1_MAP = ISP_MAP_TUPLE(0x1700, NEO_DRC_GLOBAL_HIST_ROI_CNT, u32),
	NEO_DRC_LOCAL_SUM_MAP = ISP_MAP_TUPLE(0x1E00, NEO_DRC_LOCAL_SUM_CNT, u32),
	NEO_VIGNETTING_TABLE_MAP = ISP_MAP_TUPLE(0x2E00, NEO_VIGNETTING_TABLE_SIZE, u16),
	NEO_DRC_GLOBAL_TONEMAP_MAP = ISP_MAP_TUPLE(0x4600, NEO_DRC_GLOBAL_TONEMAP_SIZE, u16),
	NEO_DRC_LOCAL_TONEMAP_MAP = ISP_MAP_TUPLE(0x4A00, NEO_DRC_LOCAL_TONEMAP_SIZE, u8),
};

/*
 * Neoisp context API functions, used to configure and update a context from
 * the params buffer, to upload a context into HW blocks once image processing
 * can start, and to capture the generated stats once processing is done.
 */
void neoisp_ctx_set_default_context(struct neoisp_dev_s *neoispd,
				    struct neoisp_context_s *context);

void neoisp_ctx_update_buf_addr(struct neoisp_dev_s *neoispd);
void neoisp_ctx_update_gcm(struct neoisp_dev_s *neoispd,
			   struct neoisp_context_s *context,
			   struct v4l2_pix_format_mplane *pix_mp,
			   enum v4l2_ycbcr_encoding enc);
void neoisp_ctx_update_hdr_mode(struct neoisp_dev_s *neoispd,
				struct neoisp_context_s *context);
void neoisp_ctx_update_head_color(struct neoisp_dev_s *neoispd,
				  struct neoisp_context_s *context,
				  u32 pixfmt);
void neoisp_ctx_update_monochrome_fmt(struct neoisp_dev_s *neoispd,
				      struct neoisp_context_s *context,
				      u32 pixfmt);
void neoisp_ctx_update_packetizer(struct neoisp_node_group_s *node_group);
void neoisp_ctx_update_pipe_conf(struct neoisp_node_group_s *node_group);
void neoisp_ctx_update_w_user_params(struct neoisp_dev_s *neoispd);

void neoisp_ctx_upload_context(struct neoisp_dev_s *neoispd);

void neoisp_ctx_get_stats(struct neoisp_dev_s *neoispd,
			  struct neoisp_buffer_s *buf);

#endif /* __NXP_NEOISP_CTX_H */
