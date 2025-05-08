/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP context definition
 *
 * Copyright 2023-2025 NXP
 */

#ifndef NEOISP_CTX_H
#define NEOISP_CTX_H

#include <uapi/linux/nxp_neoisp.h>
#include "neoisp.h"
#include "neoisp_regs.h"

#define NEOISP_PIPELINE0_BPP    (20) /* internal bit depth for input 0 path */
#define NEOISP_PIPELINE1_BPP    (16) /* internal bit depth for input 1 path */
#define NEOISP_HDR_SHIFT_MAX    (7)  /* hdr decompress block ratio field format is u7.5 */
#define NEOISP_HDR_SHIFT_RADIX  (5)  /* hdr decompress block ratio field format is u7.5 */
#define NEOISP_OBWB_SHIFT_RADIX (8)  /* obwb block gain field format is u8.8 */
#define NEOISP_HDR_KNPOINT_MAX  GENMASK(15, 0) /* knee point field is 16 bits */

/* block offset */
#define ISP_OFF_POS (0UL)
#define ISP_OFF_MASK (0xFFFFUL << ISP_OFF_POS)
#define ISP_GET_OFF(x) (((x) & ISP_OFF_MASK) >> ISP_OFF_POS)
#define ISP_OFF(x) (((x) << ISP_OFF_POS) & ISP_OFF_MASK)

/* block size */
#define ISP_WSZ_POS (16UL)
#define ISP_WSZ_MASK (0xFFFFUL << ISP_WSZ_POS)
#define ISP_GET_WSZ(x) (((x) & ISP_WSZ_MASK) >> ISP_WSZ_POS)
#define ISP_WSZ(x) (((x) << ISP_WSZ_POS) & ISP_WSZ_MASK)

enum isp_block_map_e {
	NEO_VIGNETTING_TABLE_MAP_V1 = ISP_OFF(0x1000 / sizeof(__u32)) |
				      ISP_WSZ(3072 / sizeof(__u16)),
	NEO_DRC_GLOBAL_TONEMAP_MAP_V1 = ISP_OFF(0x4000 / sizeof(__u32)) |
					ISP_WSZ(416 / sizeof(__u16)),
	NEO_DRC_LOCAL_TONEMAP_MAP_V1 = ISP_OFF(0x4400 / sizeof(__u32)) |
				       ISP_WSZ(1024 / sizeof(__u32)),
	NEO_VIGNETTING_TABLE_MAP_V2 = ISP_OFF(0x2E00 / sizeof(__u32)) |
				      ISP_WSZ(3072 / sizeof(__u16)),
	NEO_DRC_GLOBAL_TONEMAP_MAP_V2 = ISP_OFF(0x4600 / sizeof(__u32)) |
					ISP_WSZ(416 / sizeof(__u16)),
	NEO_DRC_LOCAL_TONEMAP_MAP_V2 = ISP_OFF(0x4A00 / sizeof(__u32)) |
				       ISP_WSZ(1024 / sizeof(__u32)),

	NEO_CTEMP_REG_STATS_MAP =  ISP_OFF(0x6000 / sizeof(__u32)) | ISP_WSZ(59),
	NEO_DRC_REG_STATS_MAP = ISP_OFF(0x60ec / sizeof(__u32)) | ISP_WSZ(2),
	NEO_AF_REG_STATS_MAP = ISP_OFF(0x60f4 / sizeof(__u32)) | ISP_WSZ(18),
	NEO_BNR_REG_STATS_MAP = ISP_OFF(0x613c / sizeof(__u32)) | ISP_WSZ(2),
	NEO_NR_REG_STATS_MAP = ISP_OFF(0x6144 / sizeof(__u32)) | ISP_WSZ(1),
	NEO_EE_REG_STATS_MAP = ISP_OFF(0x6148 / sizeof(__u32)) | ISP_WSZ(1),
	NEO_DF_REG_STATS_MAP = ISP_OFF(0x614c / sizeof(__u32)) | ISP_WSZ(1),

	NEO_CTEMP_R_SUM_MAP = ISP_OFF(0x0 / sizeof(__u32)) | ISP_WSZ(64),
	NEO_CTEMP_G_SUM_MAP = ISP_OFF(0x100 / sizeof(__u32)) | ISP_WSZ(64),
	NEO_CTEMP_B_SUM_MAP = ISP_OFF(0x200 / sizeof(__u32)) | ISP_WSZ(64),
	NEO_CTEMP_PIX_CNT_MAP = ISP_OFF(0x300 / sizeof(__u32)) | ISP_WSZ(64),
	NEO_RGBIR_HIST_MAP = ISP_OFF(0x400 / sizeof(__u32)) | ISP_WSZ(256),
	NEO_HIST_STAT_MAP = ISP_OFF(0x800 / sizeof(__u32)) | ISP_WSZ(512),

	NEO_DRC_LOCAL_SUM_MAP_V1 = ISP_OFF(0x4800 / sizeof(__u32)) | ISP_WSZ(1024),
	NEO_DRC_GLOBAL_HIST_ROI0_MAP_V1 = ISP_OFF(0x5800 / sizeof(__u32)) | ISP_WSZ(416),
	NEO_DRC_GLOBAL_HIST_ROI1_MAP_V1 = ISP_OFF(0x5F00 / sizeof(__u32)) | ISP_WSZ(416),
	NEO_DRC_LOCAL_SUM_MAP_V2 = ISP_OFF(0x1E00 / sizeof(__u32)) | ISP_WSZ(1024),
	NEO_DRC_GLOBAL_HIST_ROI0_MAP_V2 = ISP_OFF(0x1000 / sizeof(__u32)) | ISP_WSZ(416),
	NEO_DRC_GLOBAL_HIST_ROI1_MAP_V2 = ISP_OFF(0x1700 / sizeof(__u32)) | ISP_WSZ(416),
};

struct isp_block_map_s {
	__u32 vignetting_table;
	__u32 drc_global_tonemap;
	__u32 drc_global_hist_roi0;
	__u32 drc_global_hist_roi1;
	__u32 drc_local_tonemap;
	__u32 drc_local_sum;
};

static const struct isp_block_map_s active_block_map[] = {
	[NEO_ISP_V1] = {
		.vignetting_table = NEO_VIGNETTING_TABLE_MAP_V1,
		.drc_global_tonemap = NEO_DRC_GLOBAL_TONEMAP_MAP_V1,
		.drc_global_hist_roi0 = NEO_DRC_GLOBAL_HIST_ROI0_MAP_V1,
		.drc_global_hist_roi1 = NEO_DRC_GLOBAL_HIST_ROI1_MAP_V1,
		.drc_local_tonemap = NEO_DRC_LOCAL_TONEMAP_MAP_V1,
		.drc_local_sum = NEO_DRC_LOCAL_SUM_MAP_V1,
	}, [NEO_ISP_V2] = {
		.vignetting_table = NEO_VIGNETTING_TABLE_MAP_V2,
		.drc_global_tonemap = NEO_DRC_GLOBAL_TONEMAP_MAP_V2,
		.drc_global_hist_roi0 = NEO_DRC_GLOBAL_HIST_ROI0_MAP_V2,
		.drc_global_hist_roi1 = NEO_DRC_GLOBAL_HIST_ROI1_MAP_V2,
		.drc_local_tonemap = NEO_DRC_LOCAL_TONEMAP_MAP_V2,
		.drc_local_sum = NEO_DRC_LOCAL_SUM_MAP_V2,
	},
};

/*
 * functions
 */
int neoisp_set_params(struct neoisp_dev_s *neoispd, struct neoisp_meta_params_s *p, bool force);
void neoisp_set_gcm(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd);
int neoisp_program_ctx(struct neoisp_dev_s *neoispd, __u32 ctx_id);
int neoisp_update_ctx(struct neoisp_dev_s *neoispd, __u32 ctx_id);

#endif /* NEOISP_CTX_H */
