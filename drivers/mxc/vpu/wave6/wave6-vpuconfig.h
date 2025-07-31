/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - product config definitions
 *
 * Copyright (C) 2025 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VPUCONFIG_H__
#define __WAVE6_VPUCONFIG_H__

#define WAVE617_CODE                    0x6170
#define WAVE627_CODE                    0x6270
#define WAVE633_CODE                    0x6330
#define WAVE637_CODE                    0x6370
#define WAVE663_CODE                    0x6630
#define WAVE677_CODE                    0x6670

#define PRODUCT_CODE_W_SERIES(x) ({					\
		int c = x;						\
		((c) == WAVE617_CODE ||	(c) == WAVE627_CODE ||		\
		 (c) == WAVE633_CODE || (c) == WAVE637_CODE ||		\
		 (c) == WAVE663_CODE || (c) == WAVE677_CODE);		\
})

#define WAVE627ENC_WORKBUF_SIZE         (512 * 1024)
#define WAVE637DEC_WORKBUF_SIZE         (2 * 512 * 1024)
#define WAVE637DEC_WORKBUF_SIZE_FOR_CQ  (3 * 512 * 1024)

#define W6_MAX_PIC_STRIDE               (4096U * 4)
#define W6_PIC_STRIDE_ALIGNMENT         32
#define W6_FBC_BUF_ALIGNMENT            32
#define W6_DEC_BUF_ALIGNMENT            32

#define W6_DEF_DEC_PIC_WIDTH            720U
#define W6_DEF_DEC_PIC_HEIGHT           480U
#define W6_MIN_DEC_PIC_WIDTH            64U
#define W6_MIN_DEC_PIC_HEIGHT           64U
#define W6_MAX_DEC_PIC_WIDTH            4096U
#define W6_MAX_DEC_PIC_HEIGHT           4096U
#define W6_DEC_PIC_SIZE_STEP            1

#define W6_DEF_ENC_PIC_WIDTH            416U
#define W6_DEF_ENC_PIC_HEIGHT           240U
#define W6_MIN_ENC_PIC_WIDTH            256U
#define W6_MIN_ENC_PIC_HEIGHT           128U
#define W6_MAX_ENC_PIC_WIDTH            4096U
#define W6_MAX_ENC_PIC_HEIGHT           4096U
#define W6_ENC_PIC_SIZE_STEP            8
#define W6_ENC_CROP_X_POS_STEP          32
#define W6_ENC_CROP_Y_POS_STEP          2
#define W6_ENC_CROP_STEP                2
#define W6_MAX_CUSTOM_MAP_UNITS         \
		(DIV_ROUND_UP(W6_MAX_ENC_PIC_WIDTH, 16) * DIV_ROUND_UP(W6_MAX_ENC_PIC_HEIGHT, 16))
#define W6_ENC_CTU_WIDTH_ALIGNMENT      512

#define W6_VPU_POLL_DELAY_US            10
#define W6_VPU_POLL_TIMEOUT             300000
#define W6_BOOT_WAIT_TIMEOUT            10000
#define W6_VPU_TIMEOUT                  6000
#define W6_VPU_TIMEOUT_CYCLE_COUNT      (8000000 * 4 * 4)

#define HOST_ENDIAN                     VDI_128BIT_LITTLE_ENDIAN
#define VPU_FRAME_ENDIAN                HOST_ENDIAN
#define VPU_STREAM_ENDIAN               HOST_ENDIAN
#define VPU_USER_DATA_ENDIAN            HOST_ENDIAN
#define VPU_SOURCE_ENDIAN               HOST_ENDIAN

#define USE_SRC_PRP_AXI         0
#define USE_SRC_PRI_AXI         1
#define DEFAULT_SRC_AXI         USE_SRC_PRP_AXI

#define COMMAND_QUEUE_DEPTH             (1)

#define W6_REMAP_INDEX0                 0
#define W6_REMAP_INDEX1                 1
#define W6_REMAP_MAX_SIZE               (1024 * 1024)

#define WAVE6_MAX_INST_NUMBER           32

#define WAVE6_ARBUF_SIZE                (1024)
#define WAVE6_MAX_CODE_BUF_SIZE         (4 * 1024 * 1024)
#define WAVE6_CODE_BUF_SIZE             (1 * 1024 * 1024)
#define WAVE6_EXTRA_CODE_BUF_SIZE       (256 * 1024)
#define WAVE6_TEMPBUF_SIZE              (3 * 1024 * 1024)
#define WAVE6_WORKBUF_SIZE              (1536 * 1024)

#define WAVE6_UPPER_PROC_AXI_ID     0x0

#endif /* __WAVE6_VPUCONFIG_H__ */
