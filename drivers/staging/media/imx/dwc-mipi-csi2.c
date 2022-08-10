// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/*
 * DWC MIPI CSI2 Host registers
 */
#define DWC_MIPI_CSI2_VERSION			0x0

#define DWC_MIPI_CSI2_N_LANES			0x4
#define DWC_MIPI_CSI2_N_LANES_N_LANES(x)	((x) & 0x7)

#define DWC_MIPI_CSI2_HOST_RESETN		0x8
#define DWC_MIPI_CSI2_HOST_RESETN_ENABLE	(0x1)

#define DWC_MIPI_CSI2_INT_ST_MAIN		0xC

#define DWC_MIPI_CSI2_DATA_IDS_1		0x10
#define DWC_MIPI_CSI2_DATA_IDS_1_DI0_DT(x)	(((x) & 0x3f))
#define DWC_MIPI_CSI2_DATA_IDS_1_DI1_DT(x)	(((x) & 0x3f) << 8)
#define DWC_MIPI_CSI2_DATA_IDS_1_DI2_DT(x)	(((x) & 0x3f) << 16)
#define DWC_MIPI_CSI2_DATA_IDS_1_DI3_DT(x)	(((x) & 0x3f) << 24)

#define DWC_MIPI_CSI2_DATA_IDS_2		0x14
#define DWC_MIPI_CSI2_DATA_IDS_2_DI0_DT(x)	(((x) & 0x3f))
#define DWC_MIPI_CSI2_DATA_IDS_2_DI1_DT(x)	(((x) & 0x3f) << 8)
#define DWC_MIPI_CSI2_DATA_IDS_2_DI2_DT(x)	(((x) & 0x3f) << 16)
#define DWC_MIPI_CSI2_DATA_IDS_2_DI3_DT(x)	(((x) & 0x3f) << 24)

#define DWC_MIPI_CSI2_DPHY_CFG			0x18
#define DWC_MIPI_CSI2_DPHY_CFG_PPI8		(0x0)
#define DWC_MIPI_CSI2_DPHY_CFG_PPI16		(0x1)

#define DWC_MIPI_CSI2_DPHY_MODE			0x1C
#define DWC_MIPI_CSI2_DPHY_MODE_DPHY		(0x0)
#define DWC_MIPI_CSI2_DPHY_MODE_CPHY		(0x1)

#define DWC_MIPI_CSI2_INT_ST_AP_MAIN		0x2C

#define DWC_MIPI_CSI2_DATA_IDS_VC_1			0x30
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI0_VC(x)		((x & 0x3))
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI0_VC_0_1(x)	((x & 0x3) << 2)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI0_VC_2		(0x1 << 4)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI1_VC(x)		((x & 0x3) << 8)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI1_VC_0_1(x)	((x & 0x3) << 10)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI1_VC_2		(0x1 << 12)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI2_VC(x)		((x & 0x3) << 16)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI2_VC_0_1(x)	((x & 0x3) << 18)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI2_VC_2		(0x1 << 20)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI3_VC(x)		((x & 0x3) << 24)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI3_VC_0_1(x)	((x & 0x3) << 26)
#define DWC_MIPI_CSI2_DATA_IDS_VC_1_DI3_VC_2		(0x1 << 28)

#define DWC_MIPI_CSI2_DATA_IDS_VC_2			0x34
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI4_VC(x)		((x & 0x3))
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI4_VC_0_1(x)	((x & 0x3) << 2)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI4_VC_2		(0x1 << 4)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI5_VC(x)		((x & 0x3) << 8)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI5_VC_0_1(x)	((x & 0x3) << 10)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI5_VC_2		(0x1 << 12)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI6_VC(x)		((x & 0x3) << 16)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI6_VC_0_1(x)	((x & 0x3) << 18)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI6_VC_2		(0x1 << 20)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI7_VC(x)		((x & 0x3) << 24)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI7_VC_0_1(x)	((x & 0x3) << 26)
#define DWC_MIPI_CSI2_DATA_IDS_VC_2_DI7_VC_2		(0x1 << 28)

#define DWC_MIPI_CSI2_DPHY_SHUTDOWNZ		0x40
#define DWC_MIPI_CSI2_DPHY_SHUTDOWNZ_ENABLE	(0x1)

#define DWC_MIPI_CSI2_DPHY_RSTZ			0x44
#define DWC_MIPI_CSI2_DPHY_RSTZ_ENABLE		(0x1)

#define DWC_MIPI_CSI2_DPHY_RX_STATUS			0x48
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_CLK_LANE_HS	(0x1 << 17)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_CLK_LANE_ULP	(0x1 << 16)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE0_ULP	(0x1)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE1_ULP	(0x1 << 1)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE2_ULP	(0x1 << 2)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE3_ULP	(0x1 << 3)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE4_ULP	(0x1 << 4)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE5_ULP	(0x1 << 5)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE6_ULP	(0x1 << 6)
#define DWC_MIPI_CSI2_DPHY_RX_STATUS_DATA_LANE7_ULP	(0x1 << 7)

#define DWC_MIPI_CSI2_DPHY_STOPSTATE			0x4C
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_CLK_LANE		(0x1 << 16)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE0		(0x1)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE1		(0x1 << 1)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE2		(0x1 << 2)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE3		(0x1 << 3)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE4		(0x1 << 4)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE5		(0x1 << 5)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE6		(0x1 << 6)
#define DWC_MIPI_CSI2_DPHY_STOPSTATE_DATA_LANE7		(0x1 << 7)

#define DWC_MIPI_CSI2_DPHY_TEST_CTRL0			0x50
#define DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR		(0x1)
#define DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLKEN	(0x1 << 1)

#define DWC_MIPI_CSI2_DPHY_TEST_CTRL1			0x54
#define DWC_MIPI_CSI2_DPHY_TEST_CTRL1_TEST_DIN(x)	((x) & 0xFF)
#define DWC_MIPI_CSI2_DPHY_TEST_CTRL1_TEST_DOUT(x)	(((x) & 0xFF) << 8)
#define DWC_MIPI_CSI2_DPHY_TEST_CTRL1_TEST_EN		(0x1 << 16)

#define DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES		0x60
#define DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES_VRES(x)	((x) & 0xFFFF)

#define DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES		0x64
#define DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES_HRES(x)	((x) & 0xFFFF)

#define DWC_MIPI_CSI2_PPI_PG_CONFIG			0x68
#define DWC_MIPI_CSI2_PPI_PG_CONFIG_DATA_TYPE(x)	(((x) & 0x3F) << 8)
#define DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN(x)		(((x) & 0x3) << 14)
#define DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN_EX(x)	(((x) & 0x3) << 16)
#define DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN_EX_2_EN	(0x1 << 18)
#define DWC_MIPI_CSI2_PPI_PG_CONFIG_PG_MODE(x)		(x)

#define DWC_MIPI_CSI2_PPI_PG_ENABLE			0x6C
#define DWC_MIPI_CSI2_PPI_PG_ENABLE_EN			0x1

#define DWC_MIPI_CSI2_PPI_PG_STATUS			0x70

#define DWC_MIPI_CSI2_IPI_MODE				0x80
#define DWC_MIPI_CSI2_IPI_MODE_CAMERA			0x0
#define DWC_MIPI_CSI2_IPI_MODE_CONTROLLER		0x1
#define DWC_MIPI_CSI2_IPI_MODE_COLOR_MODE16		(0x1 << 8)
#define DWC_MIPI_CSI2_IPI_MODE_COLOR_MODE48		(0x0 << 8)
#define DWC_MIPI_CSI2_IPI_MODE_CUT_THROUGH		(0x1 << 16)
#define DWC_MIPI_CSI2_IPI_MODE_ENABLE			(0x1 << 24)

#define DWC_MIPI_CSI2_IPI_VCID				0x84
#define DWC_MIPI_CSI2_IPI_VCID_VC(x)			((x)  & 0x3)
#define DWC_MIPI_CSI2_IPI_VCID_VC_0_1(x)		(((x) & 0x3) << 2)
#define DWC_MIPI_CSI2_IPI_VCID_VC_2			(0x1 << 4)

#define DWC_MIPI_CSI2_IPI_DATA_TYPE			0x88
#define DWC_MIPI_CSI2_IPI_DATA_TYPE_DT(x)		((x) & 0x3F)
#define DWC_MIPI_CSI2_IPI_DATA_TYPE_EMB_DATA_EN		(0x1 << 8)

#define DWC_MIPI_CSI2_IPI_MEM_FLUSH			0x8C
#define DWC_MIPI_CSI2_IPI_MEM_FLUSH_AUTO		(0x1 << 8)

#define DWC_MIPI_CSI2_IPI_HSA_TIME			0x90
#define DWC_MIPI_CSI2_IPI_HSA_TIME_VAL(x)		((x) & 0xFFF)

#define DWC_MIPI_CSI2_IPI_HBP_TIME			0x94
#define DWC_MIPI_CSI2_IPI_HBP_TIME_VAL(x)		((x) & 0xFFF)

#define DWC_MIPI_CSI2_IPI_HSD_TIME			0x98
#define DWC_MIPI_CSI2_IPI_HSD_TIME_VAL(x)		((x) & 0xFFF)

#define DWC_MIPI_CSI2_IPI_HLINE_TIME			0x9C
#define DWC_MIPI_CSI2_IPI_HLINE_TIME_VAL(x)		((x) & 0x3FFF)

#define DWC_MIPI_CSI2_IPI_SOFTRSTN			0xA0
#define DWC_MIPI_CSI2_IPI_ADV_FEATURES			0xAC

#define DWC_MIPI_CSI2_IPI_VSA_LINES			0xB0
#define DWC_MIPI_CSI2_IPI_VSA_LINES_VAL(x)		((x) & 0x3FF)

#define DWC_MIPI_CSI2_IPI_VBP_LINES			0xB4
#define DWC_MIPI_CSI2_IPI_VBP_LINES_VAL(x)		((x) & 0x3FF)

#define DWC_MIPI_CSI2_IPI_VFP_LINES			0xB8
#define DWC_MIPI_CSI2_IPI_VFP_LINES_VAL(x)		((x) & 0x3FF)

#define DWC_MIPI_CSI2_IPI_VACTIVE_LINES			0xBC
#define DWC_MIPI_CSI2_IPI_VACTIVE_LINES_VAL(x)		((x) & 0x3FFF)

#define DWC_MIPI_CSI2_VC_EXTENSION			0xC8

#define DWC_MIPI_CSI2_DPHY_CAL				0xCC
#define DWC_MIPI_CSI2_INT_ST_DPHY_FATAL			0xE0
#define DWC_MIPI_CSI2_INT_MSK_DPHY_FATAL		0xE4
#define DWC_MIPI_CSI2_INT_FORCE_DPHY_FATAL		0xE8
#define DWC_MIPI_CSI2_INT_ST_PKT_FATAL			0xF0
#define DWC_MIPI_CSI2_INT_MSK_PKT_FATAL			0xF4
#define DWC_MIPI_CSI2_INT_FORCE_PKT_FATAL		0xF8

#define DWC_MIPI_CSI2_INT_ST_DPHY			0x110
#define DWC_MIPI_CSI2_INT_MSK_DPHY			0x114
#define DWC_MIPI_CSI2_INT_FORCE_DPHY			0x118
#define DWC_MIPI_CSI2_INT_ST_LINE			0x130
#define DWC_MIPI_CSI2_INT_MSK_LINE			0x134
#define DWC_MIPI_CSI2_INT_FORCE_LINE			0x138
#define DWC_MIPI_CSI2_INT_ST_IPI_FATAL			0x140
#define DWC_MIPI_CSI2_INT_MSK_IPI_FATAL			0x144
#define DWC_MIPI_CSI2_INT_FORCE_IPI_FATAL		0x148

#define DWC_MIPI_CSI2_INT_ST_AP_GENERIC			0x180
#define DWC_MIPI_CSI2_INT_MSK_AP_GENERIC		0x184
#define DWC_MIPI_CSI2_INT_FORCE_AP_GENERIC		0x188
#define DWC_MIPI_CSI2_INT_ST_AP_IPI_FATAL		0x190
#define DWC_MIPI_CSI2_INT_MSK_AP_IPI_FATAL		0x194
#define DWC_MIPI_CSI2_INT_FORCE_AP_IPI_FATAL		0x198

#define DWC_MIPI_CSI2_INT_ST_BNDRY_FRAME_FATAL		0x280
#define DWC_MIPI_CSI2_INT_MSK_BNDRY_FRAME_FATAL		0x284
#define DWC_MIPI_CSI2_INT_FORCE_BNDRY_FRAME_FATAL	0x288
#define DWC_MIPI_CSI2_INT_ST_SEQ_FRAME_FATAL		0x290
#define DWC_MIPI_CSI2_INT_MSK_SEQ_FRAME_FATAL		0x294
#define DWC_MIPI_CSI2_INT_FORCE_SEQ_FRAME_FATAL		0x298
#define DWC_MIPI_CSI2_INT_ST_CRC_FRAME_FATAL		0x2A0
#define DWC_MIPI_CSI2_INT_MSK_CRC_FRAME_FATAL		0x2A4
#define DWC_MIPI_CSI2_INT_FORCE_CRC_FRAME_FATAL		0x2A8
#define DWC_MIPI_CSI2_INT_ST_PLD_CRC_FATAL		0x2B0
#define DWC_MIPI_CSI2_INT_MSK_PLD_CRC_FATAL		0x2B4
#define DWC_MIPI_CSI2_INT_FORCE_PLD_CRC_FATAL		0x2B8
#define DWC_MIPI_CSI2_INT_ST_DATA_ID			0x2C0
#define DWC_MIPI_CSI2_INT_MSK_DATA_ID			0x2C4
#define DWC_MIPI_CSI2_INT_FORCE_DATA_ID			0x2C8
#define DWC_MIPI_CSI2_INT_ST_ECC_CORRECTED		0x2D0
#define DWC_MIPI_CSI2_INT_MSK_ECC_CORRECTED		0x2D4
#define DWC_MIPI_CSI2_INT_FORCE_ECC_CORRECTED		0x2D8
#define DWC_MIPI_CSI2_IPI_RAM_ERR_LOG_AP		0x2E0
#define DWC_MIPI_CSI2_IPI_RAM_ERR_ADDR_AP		0x2E4

#define DWC_MIPI_CSI2_SCRAMBLING			0x300
#define DWC_MIPI_CSI2_SCRAMBLING_SEED1			0x304
#define DWC_MIPI_CSI2_SCRAMBLING_SEED2			0x308

/* mediamix_GPR register */
#define DISP_MIX_CAMERA_MUX				0x30
#define DISP_MIX_CAMERA_MUX_DATA_TYPE(x)		(((x) & 0x3f) << 3)
#define DISP_MIX_CAMERA_MUX_GASKET_ENABLE		(1 << 16)

#define DISP_MIX_CSI_REG				0x48
#define DISP_MIX_CSI_REG_CFGFREQRANGE(x)		((x)  & 0x3f)
#define DISP_MIX_CSI_REG_HSFREQRANGE(x)			(((x) & 0x7f) << 8)

#define dwc_mipi_csi2h_write(__csi2h, __r, __v) writel(__v, __csi2h->base_regs + __r)
#define dwc_mipi_csi2h_read(__csi2h, __r) readl(__csi2h->base_regs + __r)

#define DWC_MIPI_CSI2_HOST_DRIVER_NAME		"dwc-mipi-csi2-host"
#define DWC_MIPI_CSI2_SUBDEV_NAME		"mxc-mipi-csi2"
#define DWC_MIPI_CSI2_IPI_NUM			8

#define DWC_MIPI_CSI2_VC0_PAD_SINK	0
#define DWC_MIPI_CSI2_VC1_PAD_SINK	1
#define DWC_MIPI_CSI2_VC2_PAD_SINK	2
#define DWC_MIPI_CSI2_VC3_PAD_SINK	3
#define DWC_MIPI_CSI2_VC0_PAD_SOURCE	4
#define DWC_MIPI_CSI2_VC1_PAD_SOURCE	5
#define DWC_MIPI_CSI2_VC2_PAD_SOURCE	6
#define DWC_MIPI_CSI2_VC3_PAD_SOURCE	7
#define DWC_MIPI_CSI2_VCX_PADS_NUM	8

#define DEF_WIDTH	1920
#define DEF_HEIGHT	1080

enum data_type {
	DT_YUV420_8	= 0x18,
	DT_YUV420_10	= 0x19,
	DT_YUV422_8	= 0x1E,
	DT_YUV422_10	= 0x1F,
	DT_RGB565	= 0x22,
	DT_RGB888	= 0x24,
	DT_RAW8		= 0x2A,
	DT_RAW10	= 0x2B,
	DT_RAW12	= 0x2C,
};

enum pg_mode {
	DWC_VERTICAL_PATTERN,
	DWC_HORIZONTIAL_APTTERN,
};

enum ipi_mode {
	CAMERA_MODE,
	CONTROLLER_MODE,
};

struct dwc_pg_config {
	enum pg_mode mode;
	u32 data_type;
	u32 virtual_ch;
	u16 virtual_ch_ex;
	u16 virtual_ch_ex_2;
};

struct dphy_config {
	u16 addr;
	u16 data;
	u16 out;
};

struct ipi_config {
	u8 data_type;
	u8 vir_chan;
	u8 vir_ext;
	u8 vir_ext_extra;

	// IPI horizontal frame information
	u16 hsa_time;
	u16 hbp_time;
	u16 hsd_time;
	u16 hline_time;

	// IPI vertical frame information
	u16 vsa_lines;
	u16 vbp_lines;
	u16 vfp_lines;
	u16 vactive_lines;

	// IPI mode control
	u8 controller_mode;
	u8 color_mode_16;
	u8 embeded_data;
};

struct csi2h_pix_format {
	u32 code;
	u32 fmt_reg;
};

struct dwc_mipi_csi2_host {
	struct v4l2_subdev    sd;
	struct v4l2_device    v4l2_dev;
	struct v4l2_subdev    *sensor_sd;
	struct media_pad      pads[8];

	struct platform_device *pdev;
	struct dwc_pg_config   pg_config;
	struct dphy_config     dphy_cfg;
	struct ipi_config      ipi_cfg[DWC_MIPI_CSI2_IPI_NUM];

	const struct csi2h_pix_format *csi2h_fmt;

	struct clk *csi_apb;
	struct clk *csi_pixel;
	struct clk *phy_cfg;

	struct v4l2_mbus_framefmt format;

	struct mutex lock;

	struct regmap *gasket;

	u32 num_lanes;
	u32 lane_state;
	u32 interlace_mode;
	u32 yuv420_line_sel;
	u32 cfgclkfreqrange;
	u32 hsclkfreqrange;

	bool ppi_pg_enable;
	bool ppi_bus_width_ppi8;

	void __iomem *base_regs;
};

static const struct csi2h_pix_format dwc_csi2h_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.fmt_reg = 0x18,
	}, {
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.fmt_reg = 0x24,
	}, {
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.fmt_reg = 0x22,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.fmt_reg = 0x2A,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.fmt_reg = 0x2B,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.fmt_reg = 0x2C,
	}, {
		/* sentinel */
	}
};

static void dwc_mipi_csi2_dump(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;

	dev_dbg(dev, "DWC CSI2 Version: %#x\n", dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_VERSION));
	dev_dbg(dev, "DWC CSI2 lanes: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_N_LANES));
	dev_dbg(dev, "DWC CSI2 HOST RESETN: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_HOST_RESETN));
	dev_dbg(dev, "DWC CSI2 INT STATUS MAIN: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_INT_ST_MAIN));
	dev_dbg(dev, "DWC CSI2 DATA IDS1: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DATA_IDS_1));
	dev_dbg(dev, "DWC CSI2 DATA IDS2: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DATA_IDS_2));
	dev_dbg(dev, "DWC CSI2 DPHY CFG: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_CFG));
	dev_dbg(dev, "DWC CSI2 DPHY MODE: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_MODE));
	dev_dbg(dev, "DWC CSI2 INT STATUS AP MAIN: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_INT_ST_AP_MAIN));
	dev_dbg(dev, "DWC CSI2 DATA IDS VC1: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DATA_IDS_VC_1));
	dev_dbg(dev, "DWC CSI2 DATA IDS VC2: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DATA_IDS_VC_2));
	dev_dbg(dev, "DWC CSI2 DPHY SHUTDOWN: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_SHUTDOWNZ));
	dev_dbg(dev, "DWC CSI2 DPHY RESET: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_RSTZ));
	dev_dbg(dev, "DWC CSI2 DPHY RX STATUS: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_RX_STATUS));
	dev_dbg(dev, "DWC CSI2 DPHY STOP STATUS: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_STOPSTATE));
	dev_dbg(dev, "DWC CSI2 DPHY TEST CTRL0: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0));
	dev_dbg(dev, "DWC CSI2 DPHY TEST CTRL1: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL1));
	dev_dbg(dev, "DWC CSI2 PPI PG PATTERN HIGH: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES));
	dev_dbg(dev, "DWC CSI2 PPI PG PATTERN WIDTH: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES));
	dev_dbg(dev, "DWC CSI2 PPI PG CONFIG: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_CONFIG));
	dev_dbg(dev, "DWC CSI2 PPI PG ENABLE: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_ENABLE));
	dev_dbg(dev, "DWC CSI2 PPI PG STATUS: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_STATUS));
	dev_dbg(dev, "DWC CSI2 IPI MODE: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_MODE));
	dev_dbg(dev, "DWC CSI2 IPI VCID: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_VCID));
	dev_dbg(dev, "DWC CSI2 IPI DATA TYPE: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_DATA_TYPE));
	dev_dbg(dev, "DWC CSI2 IPI SOFT RESET: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_SOFTRSTN));
	dev_dbg(dev, "DWC CSI2 IPI ADV FEATURE: %#x\n",   dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_ADV_FEATURES));
	dev_dbg(dev, "DWC CSI2 INT_MSK_PHY_FATAL: %#x\n", dwc_mipi_csi2h_read(csi2h, 0xE4));
	dev_dbg(dev, "DWC CSI2 INT_MSK_PKT_FATAL: %#x\n", dwc_mipi_csi2h_read(csi2h, 0xF4));
	dev_dbg(dev, "DWC CSI2 INT_MSK_PHY: %#x\n", dwc_mipi_csi2h_read(csi2h, 0x114));
	dev_dbg(dev, "DWC CSI2 INT_MSK_IPI_FATAL: %#x\n", dwc_mipi_csi2h_read(csi2h, 0x144));
	dev_dbg(dev, "DWC CSI2 INT_MSK_BNDRY_FRAME_FATAL: %#x\n", dwc_mipi_csi2h_read(csi2h, 0x284));
	dev_dbg(dev, "DWC CSI2 INT_MSK_SEQ_FRAME_FATAL: %#x\n", dwc_mipi_csi2h_read(csi2h, 0x294));
	dev_dbg(dev, "DWC CSI2 INT_MSK_DATA_ID: %#x\n", dwc_mipi_csi2h_read(csi2h, 0x2c4));
}

static void gasket_dump(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct regmap *gasket = csi2h->gasket;
	u32 val;

	regmap_read(gasket, DISP_MIX_CAMERA_MUX, &val);
	dev_dbg(dev, "gasket: CAMERA MUX: %#x\n", val);

	regmap_read(gasket, DISP_MIX_CSI_REG, &val);
	dev_dbg(dev, "gasket: CSI REG: %#x\n", val);

	regmap_read(gasket, 0x3C, &val);
	dev_dbg(dev, "gasket: MIPI -> ISI pixel ctrl: %#x\n", val);

	regmap_read(gasket, 0x40, &val);
	dev_dbg(dev, "gasket: MIPI -> ISI pixel cnt: %#x\n", val);

	regmap_read(gasket, 0x44, &val);
	dev_dbg(dev, "gasket: MIPI -> ISI line cnt: %#x\n", val);
}

static inline struct dwc_mipi_csi2_host *sd_to_dwc_mipi_csi2h(
						struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct dwc_mipi_csi2_host, sd);
}

static inline u32 get_csi2_version(struct dwc_mipi_csi2_host *csi2h)
{
	return dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_VERSION);
}

static const struct csi2h_pix_format *find_csi2h_format(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc_csi2h_formats); i++)
		if (code == dwc_csi2h_formats[i].code)
			return &dwc_csi2h_formats[i];
	return NULL;
}


static ssize_t dwc_ppi_pg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dwc_mipi_csi2_host *csi2h = dev_get_drvdata(dev);
	char temp[32];

	if (csi2h->ppi_pg_enable)
		strcpy(temp, "enabled");
	else
		strcpy(temp, "disabled");
	return sprintf(buf, "%s\n", temp);
}

static ssize_t dwc_ppi_pg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dwc_mipi_csi2_host *csi2h = dev_get_drvdata(dev);
	char temp[32];

	if (sscanf(buf, "%s", temp) > 0) {
		if (!strcmp(temp, "enabled"))
			csi2h->ppi_pg_enable = true;
		else
			csi2h->ppi_pg_enable = false;
		return count;
	}
	return -EINVAL;
}
static DEVICE_ATTR(ppi_pg_enable, 0644, dwc_ppi_pg_show, dwc_ppi_pg_store);

static bool is_ppi_pg_active(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_PPI_PG_STATUS);

	return (val) ? true : false;
}

static void dwc_pattern_generate(struct dwc_mipi_csi2_host *csi2h)
{
	struct dwc_pg_config *pg_config = &csi2h->pg_config; 
	struct v4l2_mbus_framefmt *mf = &csi2h->format;
	struct device *dev = &csi2h->pdev->dev;
	u32 val;

	/*configure resolution for pattern generator*/
	val = DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES_HRES(mf->width);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_PPI_PG_PATTERN_HRES, val);

	val = DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES_VRES(mf->height);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_PPI_PG_PATTERN_VRES, val);

	/*pattern generator configuration*/
	val  = DWC_MIPI_CSI2_PPI_PG_CONFIG_DATA_TYPE(pg_config->data_type);
	val |= DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN(pg_config->virtual_ch);
	val |= DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN_EX(pg_config->virtual_ch_ex);
	val |= DWC_MIPI_CSI2_PPI_PG_CONFIG_PG_MODE(pg_config->mode);

	if (pg_config->virtual_ch_ex_2)
		val |= DWC_MIPI_CSI2_PPI_PG_CONFIG_VIR_CHAN_EX_2_EN;

	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_PPI_PG_CONFIG, val);

	/*Enable ppi pattern generator*/
	val = (csi2h->ppi_pg_enable) ? DWC_MIPI_CSI2_PPI_PG_ENABLE_EN : 0;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_PPI_PG_ENABLE, val);

	dev_dbg(dev, "enable DWC MIPI CSI2 pattern generator\n");
}

static void dwc_mipi_csi2_dphy_reset(struct dwc_mipi_csi2_host *csi2h)
{
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_RSTZ, 0x0);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_SHUTDOWNZ, 0x0);
	udelay(50);

	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_SHUTDOWNZ, 0x1);
	udelay(50);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_RSTZ, 0x1);
}

static void dwc_mipi_csi2_test_code_reset(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	/*set PHY_TST_CTRL0, bit[0]*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0);
	val |= DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0, val);

	/*clear PHY_TST_CTRL0, bit[0]*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0);
	val &= ~DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0, val);
}

static void dwc_mipi_csi2_host_reset(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	/*reset mipi csi host, active low*/
	val = 0;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_HOST_RESETN, val);

	val = DWC_MIPI_CSI2_HOST_RESETN_ENABLE;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_HOST_RESETN, val);
}

static void dwc_mipi_csi2_ipi_config_htiming(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct ipi_config *ipi_cfg = &csi2h->ipi_cfg[0];
	u32 val;

	val = DWC_MIPI_CSI2_IPI_HSA_TIME_VAL(ipi_cfg->hsa_time);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_HSA_TIME, val);

	val = DWC_MIPI_CSI2_IPI_HBP_TIME_VAL(ipi_cfg->hbp_time);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_HBP_TIME, val);

	val = DWC_MIPI_CSI2_IPI_HSD_TIME_VAL(ipi_cfg->hsd_time);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_HSD_TIME, val);

	val = DWC_MIPI_CSI2_IPI_HLINE_TIME_VAL(ipi_cfg->hline_time);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_HLINE_TIME, val);

	dev_dbg(dev, "hsa_time=%d, hbp_time=%d, hsd_time=%d, hline_time=%d\n",
		ipi_cfg->hsa_time, ipi_cfg->hbp_time,
		ipi_cfg->hsd_time, ipi_cfg->hline_time);
}

static void dwc_mipi_csi2_ipi_config_vtiming(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct ipi_config *ipi_cfg = &csi2h->ipi_cfg[0];
	u32 val;

	val = DWC_MIPI_CSI2_IPI_VSA_LINES_VAL(ipi_cfg->vsa_lines);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_VSA_LINES, val);

	val = DWC_MIPI_CSI2_IPI_VBP_LINES_VAL(ipi_cfg->vbp_lines);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_VBP_LINES, val);

	val = DWC_MIPI_CSI2_IPI_VFP_LINES_VAL(ipi_cfg->vfp_lines);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_VFP_LINES, val);

	val = DWC_MIPI_CSI2_IPI_VACTIVE_LINES_VAL(ipi_cfg->vactive_lines);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_VACTIVE_LINES, val);

	dev_dbg(dev, "vsa_lines=%d, vbp_lines=%d, vsd_lines=%d, active_lines=%d\n",
		ipi_cfg->vsa_lines, ipi_cfg->vbp_lines,
		ipi_cfg->vfp_lines, ipi_cfg->vactive_lines);
}

static void dwc_mipi_csi2_ipi_enable(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	/*enable ipi*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_MODE);
	val |= DWC_MIPI_CSI2_IPI_MODE_ENABLE;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_MODE, val);
}

static void dwc_mipi_csi2_ipi_disable(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	/*disable ipi*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_MODE);
	val &= ~DWC_MIPI_CSI2_IPI_MODE_ENABLE;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_MODE, val);
}

/*
 * function: Start up DWC_mipi_csi2_host
 */
static void dwc_mipi_csi2_host_startup(struct dwc_mipi_csi2_host *csi2h)
{
	/* Release DWC_mipi_csi2_host from reset */
	dwc_mipi_csi2_host_reset(csi2h);

	// Apply PHY Reset
	dwc_mipi_csi2_dphy_reset(csi2h);

	// Release PHY test codes from reset
	dwc_mipi_csi2_test_code_reset(csi2h);
}

static void disp_mix_gasket_config(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct regmap *gasket = csi2h->gasket;
	struct v4l2_mbus_framefmt *mf = &csi2h->format;
	s32 fmt_val = -EINVAL;
	u32 val;


	switch (mf->code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		fmt_val = DT_RGB888;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		fmt_val = DT_YUV422_8;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		fmt_val = DT_RAW8;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		fmt_val = DT_RAW10;
		break;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		fmt_val = DT_RAW12;
		break;
	default:
		pr_err("gasket not support format %d\n", mf->code);
		return;
	}

	if (csi2h->ppi_pg_enable)
		fmt_val = DT_RGB888;

	regmap_write(gasket, DISP_MIX_CAMERA_MUX, 0x0);

	regmap_read(gasket, DISP_MIX_CAMERA_MUX, &val);
	val |= DISP_MIX_CAMERA_MUX_DATA_TYPE(fmt_val);
	val |= csi2h->ipi_cfg[0].vir_chan << 11;
	val |= csi2h->interlace_mode << 9;
	val |= csi2h->yuv420_line_sel << 13;
	regmap_write(gasket, DISP_MIX_CAMERA_MUX, val);

	dev_dbg(dev, "format: %#x, w/h=(%d, %d)\n", mf->code, mf->width, mf->height);
	if (WARN_ON(!mf->width || !mf->height)) {
		pr_err("Invaid width/height\n");
		return;
	}

	/*Configure the PHY frequency range*/
	val = DISP_MIX_CSI_REG_CFGFREQRANGE(csi2h->cfgclkfreqrange);
	val |= DISP_MIX_CSI_REG_HSFREQRANGE(csi2h->hsclkfreqrange);
	regmap_write(gasket, DISP_MIX_CSI_REG, val);

	/*enable gasket*/
	regmap_read(gasket, DISP_MIX_CAMERA_MUX, &val);
	val |= DISP_MIX_CAMERA_MUX_GASKET_ENABLE;
	regmap_write(gasket, DISP_MIX_CAMERA_MUX, val);
}

/*
 * function: Initialize DWC_mipi_csi2_host
 */
static int dwc_mipi_csi2_host_init(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct regmap *gasket = csi2h->gasket;
	struct ipi_config *ipi_cfg = &csi2h->ipi_cfg[0];
	u32 val, ret;

	/* Release Synopsys DPHY test codes from reset */
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_RSTZ, 0x0);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_SHUTDOWNZ, 0x0);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_HOST_RESETN, 0);

	/*Set testclr=1'b1*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0);
	val |= DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0, val);

	/* Wait for at least 15ns */
	udelay(1);

	/*Set testclr=1'b0*/
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0);
	val &= ~DWC_MIPI_CSI2_DPHY_TEST_CTRL0_TEST_CLR;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_TEST_CTRL0, val);

	/*config PHY frequency ranage*/
	val = DISP_MIX_CSI_REG_CFGFREQRANGE(csi2h->cfgclkfreqrange);
	val |= DISP_MIX_CSI_REG_HSFREQRANGE(csi2h->hsclkfreqrange);
	regmap_write(gasket, DISP_MIX_CSI_REG, val);

	/*config the number of active lanes*/
	val = DWC_MIPI_CSI2_N_LANES_N_LANES(csi2h->num_lanes - 1);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_N_LANES, val);

	/*config PHY_CFG[1:0], PPI8 or PPI16(default)*/
	val = csi2h->ppi_bus_width_ppi8 ? DWC_MIPI_CSI2_DPHY_CFG_PPI8 :
	                                  DWC_MIPI_CSI2_DPHY_CFG_PPI16;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_CFG, val);

	/*release PHY from reset*/
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_SHUTDOWNZ, 0x1);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DPHY_RSTZ, 0x1);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_HOST_RESETN, 0x1);

	/*Define errors to be masked*/
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_DPHY_FATAL, 0xffff);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_PKT_FATAL,  0xffff);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_IPI_FATAL,  0xffff);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_AP_GENERIC, 0xffff);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_DPHY,  0xffff);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_INT_MSK_LINE,  0xffff);

	/*add IDI DT config*/
	val  = DWC_MIPI_CSI2_DATA_IDS_1_DI0_DT(ipi_cfg->data_type);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DATA_IDS_1, val);

	/*add IDI VC config*/
	val  = DWC_MIPI_CSI2_DATA_IDS_VC_1_DI0_VC(ipi_cfg->vir_chan);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_DATA_IDS_VC_1, val);

	/* Check that data lanes are in stop state, active 0 or 1? */
	ret = readl_poll_timeout(csi2h->base_regs + DWC_MIPI_CSI2_DPHY_STOPSTATE,
			   val, val != 0x10003, 10, 10000);
	if (ret) {
		dev_err(dev, "dwc data lane not in stop state, state=%#x\n", val);
		return ret;
	}

	return 0;
}

/*
 * function: Configure IPI
 */
static int dwc_mipi_csi2_host_ipi_config(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct ipi_config *ipi_cfg = &csi2h->ipi_cfg[0];
	u32 val;

	/*do IPI soft reset*/
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_SOFTRSTN, 0x0);
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_SOFTRSTN, 0xfffff);

	/*Select virtual channel and data type to be processed by IPI*/
	val = DWC_MIPI_CSI2_IPI_DATA_TYPE_DT(ipi_cfg->data_type);
	if (ipi_cfg->embeded_data)
		val |= DWC_MIPI_CSI2_IPI_DATA_TYPE_EMB_DATA_EN;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_DATA_TYPE, val);

	val  = DWC_MIPI_CSI2_IPI_VCID_VC(ipi_cfg->vir_chan);
	val |= DWC_MIPI_CSI2_IPI_VCID_VC_0_1(ipi_cfg->vir_ext);
	if (ipi_cfg->vir_ext_extra)
		val |= DWC_MIPI_CSI2_IPI_VCID_VC_2;
	else
		val &= ~DWC_MIPI_CSI2_IPI_VCID_VC_2;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_VCID, val);

	dev_dbg(dev, "data_type:0x%x, virtual chan: %d\n",
		ipi_cfg->data_type, ipi_cfg->vir_chan);

	/* 1. Select the IPI mode, camera timing by default
	 * 2. PPI color mode, mode16/18: 16 bits interface by default
	 */
	val  = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_MODE);
	val |= ipi_cfg->controller_mode ? DWC_MIPI_CSI2_IPI_MODE_CONTROLLER :
					  DWC_MIPI_CSI2_IPI_MODE_CAMERA;
	val |= ipi_cfg->color_mode_16 ? DWC_MIPI_CSI2_IPI_MODE_COLOR_MODE16 :
					DWC_MIPI_CSI2_IPI_MODE_COLOR_MODE48;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_MODE, val);

	/* Enable ipi_cut_through */
	val  = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_IPI_MODE);
	val |= DWC_MIPI_CSI2_IPI_MODE_CUT_THROUGH;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_MODE, val);

	/* Configure the IPI horizontal frame information */
	dwc_mipi_csi2_ipi_config_htiming(csi2h);

	if (csi2h->ppi_pg_enable)
		dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_ADV_FEATURES,
				     0x01050000);

	/* Configure the IPI vertical frame information for controller mode */
	if (ipi_cfg->controller_mode || csi2h->ppi_pg_enable)
		dwc_mipi_csi2_ipi_config_vtiming(csi2h);

	dev_dbg(dev, "ipi mode: %s, color_mode: %s\n",
		ipi_cfg->controller_mode ? "controller" : "camera",
		ipi_cfg->color_mode_16   ? "color mode 16" : "color mode 48");

	return 0;
}

/*
 * function: Start the High Speed Reception Mode
 */
static int dwc_mipi_csi2_host_hs_rx_start(struct dwc_mipi_csi2_host *csi2h)
{
	u32 val;

	/* Memory is automatically flushed at each Frame Start */
	val = DWC_MIPI_CSI2_IPI_MEM_FLUSH_AUTO;
	dwc_mipi_csi2h_write(csi2h, DWC_MIPI_CSI2_IPI_MEM_FLUSH, val);

	/*Check if user enable PPI pattern generator*/
	if (csi2h->ppi_pg_enable && !is_ppi_pg_active(csi2h))
		dwc_pattern_generate(csi2h);

	/* Enable IPI */
	dwc_mipi_csi2_ipi_enable(csi2h);

	return 0;
}

/*
 * function: Stop the High Speed Reception Mode
 */
static int dwc_mipi_csi2_host_hs_rx_stop(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	u32 val;

	dwc_mipi_csi2_ipi_disable(csi2h);
	dwc_mipi_csi2_dphy_reset(csi2h);

	// Check clock lane is not in High Speed Mode
	val = dwc_mipi_csi2h_read(csi2h, DWC_MIPI_CSI2_DPHY_RX_STATUS);
	if (val & DWC_MIPI_CSI2_DPHY_RX_STATUS_CLK_LANE_HS) {
		dev_err(dev, "DWC MIPI CSI clock lanes still in HS mode\n");
		return -EINVAL;
	}

	return 0;
}

static struct media_pad *dwc_csi2_get_remote_sensor_pad(struct dwc_mipi_csi2_host *csi2h)
{
	struct v4l2_subdev *subdev = &csi2h->sd;
	struct media_pad *sink_pad, *source_pad;
	int i;

	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];

			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_entity_remote_pad(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(&csi2h->sd, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static struct v4l2_subdev *dwc_get_remote_subdev(struct dwc_mipi_csi2_host *csi2h,
						 const char * const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = dwc_csi2_get_remote_sensor_pad(csi2h);
	if (!source_pad) {
		v4l2_err(&csi2h->sd, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&csi2h->sd, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	return sen_sd;
}

static int dwc_mipi_csi2_param_init(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	struct device_node *node = dev->of_node;
	struct dwc_pg_config *pg_config = &csi2h->pg_config;
	struct ipi_config *ipi_cfg;
	int i;

	/*Disable pattern generator by default*/
	csi2h->ppi_pg_enable = false;

	memset(pg_config, 0x0, sizeof(*pg_config));

	pg_config->virtual_ch = 0;
	pg_config->data_type  = DT_RGB888;
	pg_config->mode       = DWC_VERTICAL_PATTERN;

	for (i = 0; i < DWC_MIPI_CSI2_IPI_NUM; i++) {
		ipi_cfg = &csi2h->ipi_cfg[i];
		memset(ipi_cfg, 0x0, sizeof(*ipi_cfg));

		if (csi2h->ppi_pg_enable) {
			ipi_cfg->data_type  = DT_RGB888;
			ipi_cfg->vir_chan   = 0;
			ipi_cfg->hsa_time   = 3;
			ipi_cfg->hbp_time   = 2;
			ipi_cfg->hsd_time   = 0x10;
			ipi_cfg->hline_time = 2500;
			ipi_cfg->vsa_lines  = 1;
			ipi_cfg->vbp_lines  = 1;
			ipi_cfg->vfp_lines  = 0xf;
			ipi_cfg->vactive_lines   = 1080;
			ipi_cfg->controller_mode = 0;
			ipi_cfg->color_mode_16   = 0;
			ipi_cfg->embeded_data    = 0;
		} else {
			ipi_cfg->data_type  = DT_YUV422_8;
			ipi_cfg->vir_chan   = 0;
			ipi_cfg->hsa_time   = 0;
			ipi_cfg->hbp_time   = 0;
			ipi_cfg->hsd_time   = 0;
			ipi_cfg->hline_time = 0x500;
			ipi_cfg->vsa_lines  = 0;
			ipi_cfg->vbp_lines  = 0;
			ipi_cfg->vfp_lines  = 0;
			ipi_cfg->vactive_lines   = 0x320;
			ipi_cfg->controller_mode = 0;
			ipi_cfg->color_mode_16   = 0;
			ipi_cfg->embeded_data    = 0;
		}
	}

	csi2h->format.width  = DEF_WIDTH;
	csi2h->format.height = DEF_HEIGHT;
	csi2h->format.code   = dwc_csi2h_formats[0].code;
	csi2h->csi2h_fmt     = &dwc_csi2h_formats[0];

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(dev, "No port node\n");
		return -EINVAL;
	}
	of_property_read_u32(node, "data-lanes", &csi2h->num_lanes);
	of_property_read_u32(node, "cfg-clk-range", &csi2h->cfgclkfreqrange);
	of_property_read_u32(node, "hs-clk-range", &csi2h->hsclkfreqrange);

	dev_dbg(dev, "cfgclkfreqrange=%d, hsfreqrange=%d\n",
		 csi2h->cfgclkfreqrange, csi2h->hsclkfreqrange);
	return 0;
}

static int dwc_mipi_csi2_enum_framesizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_size, NULL, fse);
}

static int dwc_mipi_csi2_enum_frame_interval(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *cfg,
					 struct v4l2_subdev_frame_interval_enum *fie)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_interval, NULL, fie);
}

static int dwc_mipi_csi2_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *format)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_mbus_framefmt *mf = &csi2h->format;
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;
	int ret;

	/* Get remote source pad */
	source_pad = dwc_csi2_get_remote_sensor_pad(csi2h);
	if (!source_pad) {
		v4l2_err(&csi2h->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd) {
		v4l2_err(&csi2h->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	format->pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, format);
	if (ret < 0) {
		v4l2_err(&csi2h->sd, "%s, call get_fmt of subdev failed!\n", __func__);
		return ret;
	}

	memcpy(mf, &format->format, sizeof(struct v4l2_mbus_framefmt));
	return 0;
}

static int dwc_mipi_csi2_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *cfg,
			     struct v4l2_subdev_format *format)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct csi2h_pix_format const *csi2h_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;
	int ret;

	/* Get remote source pad */
	source_pad = dwc_csi2_get_remote_sensor_pad(csi2h);
	if (!source_pad) {
		v4l2_err(&csi2h->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd) {
		v4l2_err(&csi2h->sd, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	format->pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, set_fmt, NULL, format);
	if (ret < 0) {
		v4l2_err(&csi2h->sd, "%s, set sensor format fail\n", __func__);
		return -EINVAL;
	}

	csi2h_fmt = find_csi2h_format(mf->code);
	if (!csi2h_fmt) {
		csi2h_fmt = &dwc_csi2h_formats[0];
		mf->code = csi2h_fmt->code;
	}

	v4l2_info(&csi2h->sd, "format: %#x\n", mf->code);

	return 0;
}

static int dwc_mipi_csi2_s_power(struct v4l2_subdev *sd, int on)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, core, s_power, on);
}

static int dwc_mipi_csi2_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, g_frame_interval, interval);
}

static int dwc_mipi_csi2_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = dwc_get_remote_subdev(csi2h, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, s_frame_interval, interval);
}

static int dwc_mipi_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dwc_mipi_csi2_host *csi2h = sd_to_dwc_mipi_csi2h(sd);
	struct device *dev = &csi2h->pdev->dev;
	int ret = 0;

	dev_info(&csi2h->pdev->dev, "enter enable=%d\n", enable);

	if (enable) {
		pm_runtime_get_sync(dev);
		dwc_mipi_csi2_host_startup(csi2h);
		dwc_mipi_csi2_host_init(csi2h);
		dwc_mipi_csi2_host_ipi_config(csi2h);
		dwc_mipi_csi2_host_hs_rx_start(csi2h);
		disp_mix_gasket_config(csi2h);
		dwc_mipi_csi2_dump(csi2h);
		gasket_dump(csi2h);
	} else {
		dwc_mipi_csi2_host_hs_rx_stop(csi2h);
		pm_runtime_put(dev);
	}

	return ret;
}
static struct v4l2_subdev_pad_ops dwc_mipi_csi2_pad_ops = {
	.enum_frame_size     = dwc_mipi_csi2_enum_framesizes,
	.enum_frame_interval = dwc_mipi_csi2_enum_frame_interval,
	.get_fmt             = dwc_mipi_csi2_get_fmt,
	.set_fmt             = dwc_mipi_csi2_set_fmt,
};

static struct v4l2_subdev_core_ops dwc_mipi_csi2_core_ops = {
	.s_power = dwc_mipi_csi2_s_power,
};

static struct v4l2_subdev_video_ops dwc_mipi_csi2_video_ops = {
	.g_frame_interval = dwc_mipi_csi2_g_frame_interval,
	.s_frame_interval = dwc_mipi_csi2_s_frame_interval,
	.s_stream	  = dwc_mipi_csi2_s_stream,
};

static struct v4l2_subdev_ops dwc_mipi_csi2_subdev_ops = {
	.core  = &dwc_mipi_csi2_core_ops,
	.video = &dwc_mipi_csi2_video_ops,
	.pad   = &dwc_mipi_csi2_pad_ops,
};

static int dwc_mipi_csi2_link_setup(struct media_entity *entity,
				    const struct media_pad *local,
				    const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations dwc_mipi_csi2_sd_media_ops = {
	.link_setup = dwc_mipi_csi2_link_setup,
};

static int dwc_mipi_csi2_clk_init(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;

	csi2h->csi_apb = devm_clk_get(dev, "clk_core");
	if (IS_ERR(csi2h->csi_apb)) {
		if (PTR_ERR(csi2h->csi_apb) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get media csi apb clock\n");
		return PTR_ERR(csi2h->csi_apb);
	}

	csi2h->csi_pixel = devm_clk_get(dev, "clk_pixel");
	if (IS_ERR(csi2h->csi_pixel)) {
		if (PTR_ERR(csi2h->csi_pixel) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get media csi pixel clock\n");
		return PTR_ERR(csi2h->csi_pixel);
	}

	csi2h->phy_cfg = devm_clk_get(dev, "phy_cfg");
	if (IS_ERR(csi2h->phy_cfg)) {
		if (PTR_ERR(csi2h->phy_cfg) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get csi phy cfg clock\n");
		return PTR_ERR(csi2h->phy_cfg);
	}

	return 0;
}

static int dwc_mipi_csi2_clk_enable(struct dwc_mipi_csi2_host *csi2h)
{
	struct device *dev = &csi2h->pdev->dev;
	int ret;

	ret = clk_prepare_enable(csi2h->csi_apb);
	if (ret) {
		dev_err(dev, "enable csi apb clock failed!\n");
		return ret;
	}

	ret = clk_prepare_enable(csi2h->csi_pixel);
	if (ret) {
		dev_err(dev, "enable csi pixel clock failed!\n");
		return ret;
	}

	ret = clk_prepare_enable(csi2h->phy_cfg);
	if (ret) {
		dev_err(dev, "enable csi phy cfg clock failed!\n");
		return ret;
	}

	return 0;
}

static void dwc_mipi_csi2_clk_disable(struct dwc_mipi_csi2_host *csi2h)
{
	clk_disable_unprepare(csi2h->phy_cfg);
	clk_disable_unprepare(csi2h->csi_apb);
	clk_disable_unprepare(csi2h->csi_pixel);
}

static int dwc_mipi_csi2_parse_dt(struct dwc_mipi_csi2_host *csi2h)
{
	return 0;
}

static int dwc_mipi_csis_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);;
}

static int dwc_mipi_csis_system_resume(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0) {
		dev_err(dev, "force resume %s failed!\n", dev_name(dev));
		return ret;
	}

	return 0;
}

static int dwc_mipi_csis_runtime_suspend(struct device *dev)
{
	struct dwc_mipi_csi2_host *csi2h = dev_get_drvdata(dev);

	dwc_mipi_csi2_clk_disable(csi2h);

	return 0;
}

static int dwc_mipi_csis_runtime_resume(struct device *dev)
{
	struct dwc_mipi_csi2_host *csi2h = dev_get_drvdata(dev);
	int ret;

	ret = dwc_mipi_csi2_clk_enable(csi2h);
	if (ret < 0)
		return ret;

	return 0;
}
static const struct dev_pm_ops dwc_mipi_csi2_host_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc_mipi_csis_system_suspend,
				dwc_mipi_csis_system_resume)
	SET_RUNTIME_PM_OPS(dwc_mipi_csis_runtime_suspend,
			   dwc_mipi_csis_runtime_resume,
			   NULL)
};

static int dwc_mipi_csi2_host_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc_mipi_csi2_host *csi2h;
	struct resource *mem_res;
	struct regmap *gasket;
	int ret;

	csi2h = devm_kzalloc(dev, sizeof(*csi2h), GFP_KERNEL);
	if (!csi2h)
		return -ENOMEM;

	csi2h->pdev = pdev;
	mutex_init(&csi2h->lock);

	ret = dwc_mipi_csi2_parse_dt(csi2h);
	if (ret < 0) {
		dev_err(dev, "fail to parse DWC property\n");
		return ret;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi2h->base_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2h->base_regs)) {
		dev_err(dev, "Failed to get mipi csi2 register\n");
		return PTR_ERR(csi2h->base_regs);
	}

	gasket = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "gasket");
	if (IS_ERR(gasket)) {
		dev_err(dev, "fail to get csi gasket\n");
		return PTR_ERR(gasket);
	}
	csi2h->gasket = gasket;

	ret = dwc_mipi_csi2_clk_init(csi2h);
	if (ret < 0) {
		dev_err(dev, "Failed to init DWC mipi csi clocks\n");
		return ret;
	}

	ret = dwc_mipi_csi2_param_init(csi2h);
	if (ret < 0)
		return ret;
	v4l2_subdev_init(&csi2h->sd, &dwc_mipi_csi2_subdev_ops);

	csi2h->sd.owner = THIS_MODULE;
	snprintf(csi2h->sd.name, sizeof(csi2h->sd.name), "%s.0",
		 DWC_MIPI_CSI2_SUBDEV_NAME);

	csi2h->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2h->sd.entity.function = MEDIA_ENT_F_IO_V4L;
	csi2h->sd.dev = dev;

	csi2h->pads[DWC_MIPI_CSI2_VC0_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2h->pads[DWC_MIPI_CSI2_VC1_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2h->pads[DWC_MIPI_CSI2_VC2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2h->pads[DWC_MIPI_CSI2_VC3_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	csi2h->pads[DWC_MIPI_CSI2_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2h->pads[DWC_MIPI_CSI2_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2h->pads[DWC_MIPI_CSI2_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2h->pads[DWC_MIPI_CSI2_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&csi2h->sd.entity,
				     DWC_MIPI_CSI2_VCX_PADS_NUM, csi2h->pads);
	if (ret < 0) {
		dev_err(dev, "DWC MIPI CSI entity pads init fail\n");
		return ret;
	}

	csi2h->sd.entity.ops = &dwc_mipi_csi2_sd_media_ops;

	v4l2_set_subdevdata(&csi2h->sd, pdev);
	platform_set_drvdata(pdev, csi2h);

	ret = device_create_file(&pdev->dev, &dev_attr_ppi_pg_enable);
	if (ret) {
		dev_err(&pdev->dev, "Fail to create ppi_pg_enable property\n");
		return ret;
	}
	pm_runtime_enable(dev);

	dev_info(&pdev->dev, "lanes: %d, name: %s\n", csi2h->num_lanes, csi2h->sd.name);
	return 0;
}

static int dwc_mipi_csi2_host_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dwc_mipi_csi2_host_of_match[] = {
	{ .compatible = "fsl,dwc-mipi-csi2-host", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dwc_mipi_csi2_host_of_match);

static struct platform_driver dwc_mipi_csi2_host_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = DWC_MIPI_CSI2_HOST_DRIVER_NAME,
		.of_match_table = dwc_mipi_csi2_host_of_match,
		.pm             = &dwc_mipi_csi2_host_pm_ops,
	},
	.probe  = dwc_mipi_csi2_host_probe,
	.remove = dwc_mipi_csi2_host_remove,
};

module_platform_driver(dwc_mipi_csi2_host_driver);

MODULE_DESCRIPTION("DWC MIPI CSI2 Host driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DWC_MIPI_CSI2_HOST_DRIVER_NAME);
