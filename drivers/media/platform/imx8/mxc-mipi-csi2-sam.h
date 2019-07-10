/*
 * Copyright 2019 NXP
 */

#ifndef _MN_MIPI_CSI2_H_
#define _MN_MIPI_CSI2_H_

#include <media/v4l2-device.h>

#define CSIS_DRIVER_NAME		"mxc-mipi-csi2-sam"
#define CSIS_SUBDEV_NAME		CSIS_DRIVER_NAME
#define CSIS_MAX_ENTITIES		2
#define CSIS0_MAX_LANES			4
#define CSIS1_MAX_LANES			2

#define MIPI_CSIS_OF_NODE_NAME		"csi"

#define MIPI_CSIS_VC0_PAD_SINK		0
#define MIPI_CSIS_VC1_PAD_SINK		1
#define MIPI_CSIS_VC2_PAD_SINK		2
#define MIPI_CSIS_VC3_PAD_SINK		3

#define MIPI_CSIS_VC0_PAD_SOURCE	4
#define MIPI_CSIS_VC1_PAD_SOURCE	5
#define MIPI_CSIS_VC2_PAD_SOURCE	6
#define MIPI_CSIS_VC3_PAD_SOURCE	7
#define MIPI_CSIS_VCX_PADS_NUM		8


#define MIPI_CSIS_DEF_PIX_WIDTH		640
#define MIPI_CSIS_DEF_PIX_HEIGHT	480

/* Register map definition */

/* CSIS version */
#define MIPI_CSIS_VERSION			0x00

/* CSIS common control */
#define MIPI_CSIS_CMN_CTRL			0x04
#define MIPI_CSIS_CMN_CTRL_UPDATE_SHADOW	(1 << 16)
#define MIPI_CSIS_CMN_CTRL_INTER_MODE		(1 << 10)
#define MIPI_CSIS_CMN_CTRL_LANE_NR_OFFSET	8
#define MIPI_CSIS_CMN_CTRL_LANE_NR_MASK		(3 << 8)
#define MIPI_CSIS_CMN_CTRL_UPDATE_SHADOW_CTRL	(1 << 2)
#define MIPI_CSIS_CMN_CTRL_RESET		(1 << 1)
#define MIPI_CSIS_CMN_CTRL_ENABLE		(1 << 0)

/* CSIS clock control */
#define MIPI_CSIS_CLK_CTRL			0x08
#define MIPI_CSIS_CLK_CTRL_CLKGATE_TRAIL_CH3(x)	(x << 28)
#define MIPI_CSIS_CLK_CTRL_CLKGATE_TRAIL_CH2(x)	(x << 24)
#define MIPI_CSIS_CLK_CTRL_CLKGATE_TRAIL_CH1(x)	(x << 20)
#define MIPI_CSIS_CLK_CTRL_CLKGATE_TRAIL_CH0(x)	(x << 16)
#define MIPI_CSIS_CLK_CTRL_CLKGATE_EN_MSK	(0xf << 4)
#define MIPI_CSIS_CLK_CTRL_WCLK_SRC		(1 << 0)

/* CSIS Interrupt mask */
#define MIPI_CSIS_INTMSK			0x10
#define MIPI_CSIS_INTMSK_EVEN_BEFORE		(1 << 31)
#define MIPI_CSIS_INTMSK_EVEN_AFTER		(1 << 30)
#define MIPI_CSIS_INTMSK_ODD_BEFORE		(1 << 29)
#define MIPI_CSIS_INTMSK_ODD_AFTER		(1 << 28)
#define MIPI_CSIS_INTMSK_FRAME_START		(1 << 24)
#define MIPI_CSIS_INTMSK_FRAME_END		(1 << 20)
#define MIPI_CSIS_INTMSK_ERR_SOT_HS		(1 << 16)
#define MIPI_CSIS_INTMSK_ERR_LOST_FS		(1 << 12)
#define MIPI_CSIS_INTMSK_ERR_LOST_FE		(1 << 8)
#define MIPI_CSIS_INTMSK_ERR_OVER		(1 << 4)
#define MIPI_CSIS_INTMSK_ERR_WRONG_CFG		(1 << 3)
#define MIPI_CSIS_INTMSK_ERR_ECC		(1 << 2)
#define MIPI_CSIS_INTMSK_ERR_CRC		(1 << 1)
#define MIPI_CSIS_INTMSK_ERR_UNKNOWN		(1 << 0)

/* CSIS Interrupt source */
#define MIPI_CSIS_INTSRC			0x14
#define MIPI_CSIS_INTSRC_EVEN_BEFORE		(1 << 31)
#define MIPI_CSIS_INTSRC_EVEN_AFTER		(1 << 30)
#define MIPI_CSIS_INTSRC_EVEN			(0x3 << 30)
#define MIPI_CSIS_INTSRC_ODD_BEFORE		(1 << 29)
#define MIPI_CSIS_INTSRC_ODD_AFTER		(1 << 28)
#define MIPI_CSIS_INTSRC_ODD			(0x3 << 28)
#define MIPI_CSIS_INTSRC_NON_IMAGE_DATA		(0xf << 28)
#define MIPI_CSIS_INTSRC_FRAME_START		(1 << 24)
#define MIPI_CSIS_INTSRC_FRAME_END		(1 << 20)
#define MIPI_CSIS_INTSRC_ERR_SOT_HS		(1 << 16)
#define MIPI_CSIS_INTSRC_ERR_LOST_FS		(1 << 12)
#define MIPI_CSIS_INTSRC_ERR_LOST_FE		(1 << 8)
#define MIPI_CSIS_INTSRC_ERR_OVER		(1 << 4)
#define MIPI_CSIS_INTSRC_ERR_WRONG_CFG		(1 << 3)
#define MIPI_CSIS_INTSRC_ERR_ECC		(1 << 2)
#define MIPI_CSIS_INTSRC_ERR_CRC		(1 << 1)
#define MIPI_CSIS_INTSRC_ERR_UNKNOWN		(1 << 0)
#define MIPI_CSIS_INTSRC_ERRORS			0xfffff

/* D-PHY status control */
#define MIPI_CSIS_DPHYSTATUS			0x20
#define MIPI_CSIS_DPHYSTATUS_ULPS_DAT		(1 << 8)
#define MIPI_CSIS_DPHYSTATUS_STOPSTATE_DAT	(1 << 4)
#define MIPI_CSIS_DPHYSTATUS_ULPS_CLK		(1 << 1)
#define MIPI_CSIS_DPHYSTATUS_STOPSTATE_CLK	(1 << 0)

/* D-PHY common control */
#define MIPI_CSIS_DPHYCTRL			0x24
#define MIPI_CSIS_DPHYCTRL_HSS_MASK		(0xff << 24)
#define MIPI_CSIS_DPHYCTRL_HSS_OFFSET		24
#define MIPI_CSIS_DPHYCTRL_SCLKS_MASK		(0x3 << 22)
#define MIPI_CSIS_DPHYCTRL_SCLKS_OFFSET		22
#define MIPI_CSIS_DPHYCTRL_DPDN_SWAP_CLK	(1 << 6)
#define MIPI_CSIS_DPHYCTRL_DPDN_SWAP_DAT	(1 << 5)
#define MIPI_CSIS_DPHYCTRL_ENABLE_DAT		(1 << 1)
#define MIPI_CSIS_DPHYCTRL_ENABLE_CLK		(1 << 0)
#define MIPI_CSIS_DPHYCTRL_ENABLE		(0x1f << 0)

/* D-PHY Master and Slave Control register Low */
#define MIPI_CSIS_DPHYBCTRL_L		0x30
/* D-PHY Master and Slave Control register High */
#define MIPI_CSIS_DPHYBCTRL_H		0x34
/* D-PHY Slave Control register Low */
#define MIPI_CSIS_DPHYSCTRL_L		0x38
/* D-PHY Slave Control register High */
#define MIPI_CSIS_DPHYSCTRL_H		0x3c


/* ISP Configuration register */
#define MIPI_CSIS_ISPCONFIG_CH0				0x40
#define MIPI_CSIS_ISPCONFIG_CH0_PIXEL_MODE_MASK		(0x3 << 12)
#define MIPI_CSIS_ISPCONFIG_CH0_PIXEL_MODE_SHIFT	12

#define MIPI_CSIS_ISPCONFIG_CH1				0x50
#define MIPI_CSIS_ISPCONFIG_CH1_PIXEL_MODE_MASK		(0x3 << 12)
#define MIPI_CSIS_ISPCONFIG_CH1_PIXEL_MODE_SHIFT	12

#define MIPI_CSIS_ISPCONFIG_CH2				0x60
#define MIPI_CSIS_ISPCONFIG_CH2_PIXEL_MODE_MASK		(0x3 << 12)
#define MIPI_CSIS_ISPCONFIG_CH2_PIXEL_MODE_SHIFT	12

#define MIPI_CSIS_ISPCONFIG_CH3				0x70
#define MIPI_CSIS_ISPCONFIG_CH3_PIXEL_MODE_MASK		(0x3 << 12)
#define MIPI_CSIS_ISPCONFIG_CH3_PIXEL_MODE_SHIFT	12

#define PIXEL_MODE_SINGLE_PIXEL_MODE			0x0
#define PIXEL_MODE_DUAL_PIXEL_MODE			0x1
#define PIXEL_MODE_QUAD_PIXEL_MODE			0x2
#define PIXEL_MODE_INVALID_PIXEL_MODE			0x3


#define MIPI_CSIS_ISPCFG_MEM_FULL_GAP_MSK	(0xff << 24)
#define MIPI_CSIS_ISPCFG_MEM_FULL_GAP(x)	(x << 24)
#define MIPI_CSIS_ISPCFG_DOUBLE_CMPNT		(1 << 12)
#define MIPI_CSIS_ISPCFG_ALIGN_32BIT		(1 << 11)
#define MIPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT	(0x1e << 2)
#define MIPI_CSIS_ISPCFG_FMT_RAW8		(0x2a << 2)
#define MIPI_CSIS_ISPCFG_FMT_RAW10		(0x2b << 2)
#define MIPI_CSIS_ISPCFG_FMT_RAW12		(0x2c << 2)
#define MIPI_CSIS_ISPCFG_FMT_RGB888		(0x24 << 2)
#define MIPI_CSIS_ISPCFG_FMT_RGB565		(0x22 << 2)
/* User defined formats, x = 1...4 */
#define MIPI_CSIS_ISPCFG_FMT_USER(x)		((0x30 + x - 1) << 2)
#define MIPI_CSIS_ISPCFG_FMT_MASK		(0x3f << 2)

/* ISP Image Resolution register */
#define MIPI_CSIS_ISPRESOL_CH0			0x44
#define MIPI_CSIS_ISPRESOL_CH1			0x54
#define MIPI_CSIS_ISPRESOL_CH2			0x64
#define MIPI_CSIS_ISPRESOL_CH3			0x74
#define CSIS_MAX_PIX_WIDTH			0xffff
#define CSIS_MAX_PIX_HEIGHT			0xffff

/* ISP SYNC register */
#define MIPI_CSIS_ISPSYNC_CH0			0x48
#define MIPI_CSIS_ISPSYNC_CH1			0x58
#define MIPI_CSIS_ISPSYNC_CH2			0x68
#define MIPI_CSIS_ISPSYNC_CH3			0x78

#define MIPI_CSIS_ISPSYNC_HSYNC_LINTV_OFFSET	18
#define MIPI_CSIS_ISPSYNC_VSYNC_SINTV_OFFSET 	12
#define MIPI_CSIS_ISPSYNC_VSYNC_EINTV_OFFSET	0

#define MIPI_CSIS_FRAME_COUNTER_CH0	0x0100
#define MIPI_CSIS_FRAME_COUNTER_CH1	0x0104
#define MIPI_CSIS_FRAME_COUNTER_CH2	0x0108
#define MIPI_CSIS_FRAME_COUNTER_CH3	0x010C

/* Non-image packet data buffers */
#define MIPI_CSIS_PKTDATA_ODD		0x2000
#define MIPI_CSIS_PKTDATA_EVEN		0x3000
#define MIPI_CSIS_PKTDATA_SIZE		SZ_4K

#define DEFAULT_SCLK_CSIS_FREQ		166000000UL

/* display_mix_clk_en_csr */
#define DISP_MIX_GASKET_0_CTRL			0x00
#define GASKET_0_CTRL_DATA_TYPE(x)		(((x) & (0x3F)) << 8)
#define GASKET_0_CTRL_DATA_TYPE_MASK		((0x3FUL) << (8))

#define GASKET_0_CTRL_DATA_TYPE_YUV420_8	0x18
#define GASKET_0_CTRL_DATA_TYPE_YUV420_10	0x19
#define GASKET_0_CTRL_DATA_TYPE_LE_YUV420_8	0x1a
#define GASKET_0_CTRL_DATA_TYPE_CS_YUV420_8	0x1c
#define GASKET_0_CTRL_DATA_TYPE_CS_YUV420_10	0x1d
#define GASKET_0_CTRL_DATA_TYPE_YUV422_8	0x1e
#define GASKET_0_CTRL_DATA_TYPE_YUV422_10	0x1f
#define GASKET_0_CTRL_DATA_TYPE_RGB565		0x22
#define GASKET_0_CTRL_DATA_TYPE_RGB666		0x23
#define GASKET_0_CTRL_DATA_TYPE_RGB888		0x24
#define GASKET_0_CTRL_DATA_TYPE_RAW6		0x28
#define GASKET_0_CTRL_DATA_TYPE_RAW7		0x29
#define GASKET_0_CTRL_DATA_TYPE_RAW8		0x2a
#define GASKET_0_CTRL_DATA_TYPE_RAW10		0x2b
#define GASKET_0_CTRL_DATA_TYPE_RAW12		0x2c
#define GASKET_0_CTRL_DATA_TYPE_RAW14		0x2d

#define GASKET_0_CTRL_DUAL_COMP_ENABLE		BIT(1)
#define GASKET_0_CTRL_ENABLE			BIT(0)

#define DISP_MIX_GASKET_0_HSIZE			0x04
#define DISP_MIX_GASKET_0_VSIZE			0x08

struct mipi_csis_event {
	u32 mask;
	const char * const name;
	unsigned int counter;
};

static const struct mipi_csis_event mipi_csis_events[] = {
	/* Errors */
	{ MIPI_CSIS_INTSRC_ERR_SOT_HS,	"SOT Error" },
	{ MIPI_CSIS_INTSRC_ERR_LOST_FS,	"Lost Frame Start Error" },
	{ MIPI_CSIS_INTSRC_ERR_LOST_FE,	"Lost Frame End Error" },
	{ MIPI_CSIS_INTSRC_ERR_OVER,	"FIFO Overflow Error" },
	{ MIPI_CSIS_INTSRC_ERR_ECC,	"ECC Error" },
	{ MIPI_CSIS_INTSRC_ERR_CRC,	"CRC Error" },
	{ MIPI_CSIS_INTSRC_ERR_UNKNOWN,	"Unknown Error" },
	/* Non-image data receive events */
	{ MIPI_CSIS_INTSRC_EVEN_BEFORE,	"Non-image data before even frame" },
	{ MIPI_CSIS_INTSRC_EVEN_AFTER,	"Non-image data after even frame" },
	{ MIPI_CSIS_INTSRC_ODD_BEFORE,	"Non-image data before odd frame" },
	{ MIPI_CSIS_INTSRC_ODD_AFTER,	"Non-image data after odd frame" },
	/* Frame start/end */
	{ MIPI_CSIS_INTSRC_FRAME_START,	"Frame Start" },
	{ MIPI_CSIS_INTSRC_FRAME_END,	"Frame End" },
};
#define MIPI_CSIS_NUM_EVENTS ARRAY_SIZE(mipi_csis_events)

struct csis_pktbuf {
	u32 *data;
	unsigned int len;
};

struct csis_hw_reset1 {
	struct regmap *src;
	u8 req_src;
	u8 rst_bit;
};

/**
 * struct csi_state - the driver's internal state data structure
 * @lock: mutex serializing the subdev and power management operations,
 *        protecting @format and @flags members
 * @sd: v4l2_subdev associated with CSIS device instance
 * @index: the hardware instance index
 * @pdev: CSIS platform device
 * @phy: pointer to the CSIS generic PHY
 * @regs: mmaped I/O registers memory
 * @supplies: CSIS regulator supplies
 * @clock: CSIS clocks
 * @irq: requested s5p-mipi-csis irq number
 * @flags: the state variable for power and streaming control
 * @clock_frequency: device bus clock frequency
 * @hs_settle: HS-RX settle time
 * @clk_settle: Clk settle time
 * @num_lanes: number of MIPI-CSI data lanes used
 * @max_num_lanes: maximum number of MIPI-CSI data lanes supported
 * @wclk_ext: CSI wrapper clock: 0 - bus clock, 1 - external SCLK_CAM
 * @csis_fmt: current CSIS pixel format
 * @format: common media bus format for the source and sink pad
 * @slock: spinlock protecting structure members below
 * @pkt_buf: the frame embedded (non-image) data buffer
 * @events: MIPI-CSIS event (error) counters
 */
struct csi_state {
	struct mutex lock;
	struct device		*dev;
	struct v4l2_subdev	sd;
	struct v4l2_device	v4l2_dev;

	struct media_pad pads[MIPI_CSIS_VCX_PADS_NUM];

	u8 index;
	struct platform_device *pdev;
	struct phy *phy;
	void __iomem *regs;
	struct clk *mipi_clk;
	struct clk *phy_clk;
	struct clk *disp_axi;
	struct clk *disp_apb;
	int irq;
	u32 flags;

	u32 clk_frequency;
	u32 hs_settle;
	u32 clk_settle;
	u32 num_lanes;
	u32 max_num_lanes;
	int	 id;
	u8 wclk_ext;

	u8 vchannel;
	const struct csis_pix_format *csis_fmt;
	struct v4l2_mbus_framefmt format;

	spinlock_t slock;
	struct csis_pktbuf pkt_buf;
	struct mipi_csis_event events[MIPI_CSIS_NUM_EVENTS];

	struct v4l2_async_subdev    asd;
	struct v4l2_async_notifier  subdev_notifier;
	struct v4l2_async_subdev    *async_subdevs[2];

	struct csis_hw_reset1 hw_reset;
	struct regulator     *mipi_phy_regulator;

	struct regmap *gasket;
	struct reset_control *soft_resetn;
	struct reset_control *clk_enable;
	struct reset_control *mipi_reset;
};

/**
 * struct csis_pix_format - CSIS pixel format description
 * @pix_width_alignment: horizontal pixel alignment, width will be
 *                       multiple of 2^pix_width_alignment
 * @code: corresponding media bus code
 * @fmt_reg: MIPI_CSIS_CONFIG register value
 * @data_alignment: MIPI-CSI data alignment in bits
 */
struct csis_pix_format {
	unsigned int pix_width_alignment;
	u32 code;
	u32 fmt_reg;
	u8 data_alignment;
};
#endif // _MN_MIPI_CSI2_H_
