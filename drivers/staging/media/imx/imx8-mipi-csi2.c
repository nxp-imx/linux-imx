// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Capture CSI Subdev for Freescale i.MX8QM/QXP SOC
 *
 * Copyright 2019-2021 NXP
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
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>

#include "imx8-common.h"

#define MXC_MIPI_CSI2_DRIVER_NAME	"mxc-mipi-csi2"
#define MXC_MIPI_CSI2_SUBDEV_NAME	MXC_MIPI_CSI2_DRIVER_NAME
#define MXC_MIPI_CSI2_MAX_LANES		4

/* Subsystem CSR */
#define CSI2SS_BASE_OFFSET		0x0

#define CSI2SS_PLM_CTRL				(CSI2SS_BASE_OFFSET + 0x0)
#define CSI2SS_PLM_CTRL_PL_CLK_RUN		0x80000000
#define CSI2SS_PLM_CTRL_VSYNC_OVERRIDE		0x200
#define CSI2SS_PLM_CTRL_HSYNC_OVERRIDE		0x400
#define CSI2SS_PLM_CTRL_VALID_OVERRIDE		0x800
#define CSI2SS_PLM_CTRL_POLARITY_MASK		0x1000
#define CSI2SS_PLM_CTRL_POLARITY_HIGH		0x1000
#define CSI2SS_PLM_CTRL_POLARITY_LOW		0x0
#define CSI2SS_PLM_CTRL_ENABLE_PL		1
#define CSI2SS_PLM_CTRL_ENABLE_PL_OFFSET	0
#define CSI2SS_PLM_CTRL_ENABLE_PL_MASK		1

#define CSI2SS_PHY_CTRL				(CSI2SS_BASE_OFFSET + 0x4)
#define CSI2SS_PHY_CTRL_PD			1
#define CSI2SS_PHY_CTRL_PD_OFFSET		22
#define CSI2SS_PHY_CTRL_PD_MASK			0x400000
#define CSI2SS_PHY_CTRL_RTERM_SEL		1
#define CSI2SS_PHY_CTRL_RTERM_SEL_OFFSET	21
#define CSI2SS_PHY_CTRL_RTERM_SEL_MASK		0x200000
#define CSI2SS_PHY_CTRL_RX_HS_SETTLE_OFFSET	4
#define CSI2SS_PHY_CTRL_RX_HS_SETTLE_MASK	0x3F0
#define CSI2SS_PHY_CTRL_CONT_CLK_MODE		1
#define CSI2SS_PHY_CTRL_CONT_CLK_MODE_OFFSET	3
#define CSI2SS_PHY_CTRL_CONT_CLK_MODE_MASK	0x8
#define CSI2SS_PHY_CTRL_DDRCLK_EN		1
#define CSI2SS_PHY_CTRL_DDRCLK_EN_OFFSET	2
#define CSI2SS_PHY_CTRL_DDRCLK_EN_MASK		0x4
#define CSI2SS_PHY_CTRL_AUTO_PD_EN		1
#define CSI2SS_PHY_CTRL_AUTO_PD_EN_OFFSET	1
#define CSI2SS_PHY_CTRL_AUTO_PD_EN_MASK		0x2
#define CSI2SS_PHY_CTRL_RX_ENABLE		1
#define CSI2SS_PHY_CTRL_RX_ENABLE_OFFSET	0
#define CSI2SS_PHY_CTRL_RX_ENABLE_MASK		0x1

#define CSI2SS_PHY_STATUS			(CSI2SS_BASE_OFFSET + 0x8)
#define CSI2SS_PHY_TEST_STATUS			(CSI2SS_BASE_OFFSET + 0x10)
#define CSI2SS_PHY_TEST_STATUS_D0		(CSI2SS_BASE_OFFSET + 0x14)
#define CSI2SS_PHY_TEST_STATUS_D1		(CSI2SS_BASE_OFFSET + 0x18)
#define CSI2SS_PHY_TEST_STATUS_D2		(CSI2SS_BASE_OFFSET + 0x1C)
#define CSI2SS_PHY_TEST_STATUS_D3		(CSI2SS_BASE_OFFSET + 0x20)

#define CSI2SS_VC_INTERLACED			(CSI2SS_BASE_OFFSET + 0x30)
#define CSI2SS_VC_INTERLACED_VC0		1
#define CSI2SS_VC_INTERLACED_VC1		2
#define CSI2SS_VC_INTERLACED_VC2		4
#define CSI2SS_VC_INTERLACED_VC3		8
#define CSI2SS_VC_INTERLACED_OFFSET		0
#define CSI2SS_VC_INTERLACED_MASK		0xF

#define CSI2SS_DATA_TYPE			(CSI2SS_BASE_OFFSET + 0x38)
#define CSI2SS_DATA_TYPE_LEGACY_YUV420_8BIT	BIT(2)
#define CSI2SS_DATA_TYPE_YUV422_8BIT		BIT(6)
#define CSI2SS_DATA_TYPE_YUV422_10BIT		BIT(7)
#define CSI2SS_DATA_TYPE_RGB444			BIT(8)
#define CSI2SS_DATA_TYPE_RGB555			BIT(9)
#define CSI2SS_DATA_TYPE_RGB565			BIT(10)
#define CSI2SS_DATA_TYPE_RGB666			BIT(11)
#define CSI2SS_DATA_TYPE_RGB888			BIT(12)
#define CSI2SS_DATA_TYPE_RAW6			BIT(16)
#define CSI2SS_DATA_TYPE_RAW8			BIT(18)
#define CSI2SS_DATA_TYPE_RAW10			BIT(19)
#define CSI2SS_DATA_TYPE_RAW12			BIT(20)
#define CSI2SS_DATA_TYPE_RAW14			BIT(21)

#define CSI2SS_YUV420_1ST_LINE_DATA_TYPE	(CSI2SS_BASE_OFFSET + 0x40)
#define CSI2SS_YUV420_1ST_LINE_DATA_TYPE_ODD	0
#define CSI2SS_YUV420_1ST_LINE_DATA_TYPE_EVEN	1
#define CSI2SS_YUV420_1ST_LINE_DATA_TYPE_OFFSET	0
#define CSI2SS_YUV420_1ST_LINE_DATA_TYPE_MASK	1

#define CSI2SS_CTRL_CLK_RESET			(CSI2SS_BASE_OFFSET + 0x44)
#define CSI2SS_CTRL_CLK_RESET_EN		1
#define CSI2SS_CTRL_CLK_RESET_OFFSET		0
#define CSI2SS_CTRL_CLK_RESET_MASK		1
#define CSI2SS_CTRL_CLK_RESET_CLK_OFF		1
#define CSI2SS_CTRL_CLK_RESET_CLK_OFFSET	1
#define CSI2SS_CTRL_CLK_RESET_CLK_MASK		0x1

#define CSI2SS_STREAM_FENCE_CTRL		(CSI2SS_BASE_OFFSET + 0x48)
#define CSI2SS_STREAM_FENCE_VC0			1
#define CSI2SS_STREAM_FENCE_VC1			2
#define CSI2SS_STREAM_FENCE_VC2			4
#define CSI2SS_STREAM_FENCE_VC3			8
#define CSI2SS_STREAM_FENCE_CTRL_OFFSET		0
#define CSI2SS_STREAM_FENCE_CTRL_MASK		0xF

#define CSI2SS_STREAM_FENCE_STATUS		(CSI2SS_BASE_OFFSET + 0x4C)

/* CSI-2 controller CSR */
#define CSI2RX_BASE_OFFSET			(0x100)

#define CSI2RX_CFG_NUM_LANES			(CSI2RX_BASE_OFFSET + 0x0)
#define CSI2RX_CFG_NUM_LANES_OFFSET		0
#define CSI2RX_CFG_NUM_LANES_MASK		0x3

#define CSI2RX_CFG_DISABLE_DATA_LANES		(CSI2RX_BASE_OFFSET + 0x4)
#define CSI2RX_CFG_DISABLE_DATA_LANES_3		8
#define CSI2RX_CFG_DISABLE_DATA_LANES_2		4
#define CSI2RX_CFG_DISABLE_DATA_LANES_1		2
#define CSI2RX_CFG_DISABLE_DATA_LANES_0		1
#define CSI2RX_CFG_DISABLE_DATA_LANES_OFFSET	0
#define CSI2RX_CFG_DISABLE_DATA_LANES_MASK	0xF

#define CSI2RX_BIT_ERR				(CSI2RX_BASE_OFFSET + 0x8)

#define CSI2RX_IRQ_STATUS			(CSI2RX_BASE_OFFSET + 0xC)
#define CSI2RX_IRQ_STATUS_CRC_ERROR		0x1
#define CSI2RX_IRQ_STATUS_1BIT_CRC_ERROR	0x2
#define CSI2RX_IRQ_STATUS_2BIT_CRC_ERROR	0x4
#define CSI2RX_IRQ_STATUS_ULPS_CHANGE		0x8
#define CSI2RX_IRQ_STATUS_DPHY_ERRSOTHS		0x10
#define CSI2RX_IRQ_STATUS_DPHY_ERRSOTSYNC_HS	0x20
#define CSI2RX_IRQ_STATUS_DPHY_ERRESC		0x40
#define CSI2RX_IRQ_STATUS_DPHY_ERRSYNCESC	0x80
#define CSI2RX_IRQ_STATUS_DPHY_ERRCTRL		0x100

#define CSI2RX_IRQ_MASK				(CSI2RX_BASE_OFFSET + 0x10)
#define CSI2RX_IRQ_MASK_CRC_ERROR		0x1
#define CSI2RX_IRQ_MASK_1BIT_CRC_ERROR		0x2
#define CSI2RX_IRQ_MASK_2BIT_CRC_ERROR		0x4
#define CSI2RX_IRQ_MASK_ULPS_CHANGE		0x8
#define CSI2RX_IRQ_MASK_DPHY_ERRSOTHS		0x10
#define CSI2RX_IRQ_MASK_DPHY_ERRSOTSYNC_HS	0x20
#define CSI2RX_IRQ_MASK_DPHY_ERRESC		0x40
#define CSI2RX_IRQ_MASK_DPHY_ERRSYNCESC		0x80
#define CSI2RX_IRQ_MASK_DPHY_ERRCTRL		0x100

#define CSI2RX_ULPS_STATUS			(CSI2RX_BASE_OFFSET + 0x14)
#define CSI2RX_ULPS_STATUS_CLK_LANE_ULPS	0x1
#define CSI2RX_ULPS_STATUS_DAT_LANE0_ULPS	0x2
#define CSI2RX_ULPS_STATUS_DAT_LANE1_ULPS	0x4
#define CSI2RX_ULPS_STATUS_DAT_LANE2_ULPS	0x8
#define CSI2RX_ULPS_STATUS_DAT_LANE3_ULPS	0x10
#define CSI2RX_ULPS_STATUS_CLK_LANE_MARK	0x20
#define CSI2RX_ULPS_STATUS_DAT_LANE0_MARK	0x40
#define CSI2RX_ULPS_STATUS_DAT_LANE1_MARK	0x80
#define CSI2RX_ULPS_STATUS_DAT_LANE2_MARK	0x100
#define CSI2RX_ULPS_STATUS_DAT_LANE3_MARK	0x200

#define CSI2RX_PPI_ERRSOT_HS			(CSI2RX_BASE_OFFSET + 0x18)
#define CSI2RX_PPI_ERRSOT_HS_DAT_LANE0		0x1
#define CSI2RX_PPI_ERRSOT_HS_DAT_LANE1		0x2
#define CSI2RX_PPI_ERRSOT_HS_DAT_LANE2		0x4
#define CSI2RX_PPI_ERRSOT_HS_DAT_LANE3		0x8

#define CSI2RX_PPI_ERRSOTSYNC_HS		(CSI2RX_BASE_OFFSET + 0x1C)
#define CSI2RX_PPI_ERRSOTSYNC_HS_DAT_LANE0	0x1
#define CSI2RX_PPI_ERRSOTSYNC_HS_DAT_LANE1	0x2
#define CSI2RX_PPI_ERRSOTSYNC_HS_DAT_LANE2	0x4
#define CSI2RX_PPI_ERRSOTSYNC_HS_DAT_LANE3	0x8

#define CSI2RX_PPI_ERRESC			(CSI2RX_BASE_OFFSET + 0x20)
#define CSI2RX_PPI_ERRESC_DAT_LANE0		0x1
#define CSI2RX_PPI_ERRESC_DAT_LANE1		0x2
#define CSI2RX_PPI_ERRESC_DAT_LANE2		0x4
#define CSI2RX_PPI_ERRESC_DAT_LANE3		0x8

#define CSI2RX_PPI_ERRSYNCESC			(CSI2RX_BASE_OFFSET + 0x24)
#define CSI2RX_PPI_ERRSYNCESC_DAT_LANE0		0x1
#define CSI2RX_PPI_ERRSYNCESC_DAT_LANE1		0x2
#define CSI2RX_PPI_ERRSYNCESC_DAT_LANE2		0x4
#define CSI2RX_PPI_ERRSYNCESC_DAT_LANE3		0x8

#define CSI2RX_PPI_ERRCONTROL			(CSI2RX_BASE_OFFSET + 0x28)
#define CSI2RX_PPI_ERRCONTROL_DAT_LANE0		0x1
#define CSI2RX_PPI_ERRCONTROL_DAT_LANE1		0x2
#define CSI2RX_PPI_ERRCONTROL_DAT_LANE2		0x4
#define CSI2RX_PPI_ERRCONTROL_DAT_LANE3		0x8

#define CSI2RX_CFG_DISABLE_PAYLOAD_0		(CSI2RX_BASE_OFFSET + 0x2C)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_LEGACY_YUV420_8BIT	BIT(10)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_YUV422_8BIT		BIT(14)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_YUV422_10BIT		BIT(15)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RGB444			BIT(16)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RGB555			BIT(17)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RGB565			BIT(18)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RGB666			BIT(19)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RGB888			BIT(20)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW6			BIT(24)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW7			BIT(25)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW8			BIT(26)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW10			BIT(27)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW12			BIT(28)
#define CSI2RX_CFG_DISABLE_PAYLOAD_TYPE_RAW14			BIT(29)

#define CSI2RX_CFG_DISABLE_PAYLOAD_1		(CSI2RX_BASE_OFFSET + 0x30)

struct mxc_mipi_csi2_dev;
static const struct of_device_id mipi_csi2_of_match[];

struct csis_hw_reset {
	struct regmap *src;
	u8 req_src;
	u8 rst_val;
};

struct csis_phy_gpr {
	struct regmap *gpr;
	u8 req_src;
};

struct mxc_mipi_csi2_clk_ops {
	int (*clk_get)(struct mxc_mipi_csi2_dev *csi2dev);
	int (*clk_enable)(struct mxc_mipi_csi2_dev *csi2dev);
	void (*clk_disable)(struct mxc_mipi_csi2_dev *csi2dev);
};

struct mxc_mipi_csi2_pd_ops {
	int (*attach)(struct mxc_mipi_csi2_dev *csi2dev);
	void (*detach)(struct mxc_mipi_csi2_dev *csi2dev);
};

struct mxc_mipi_csi2_rst_ops {
	int (*reset)(struct mxc_mipi_csi2_dev *csi2dev, int enable);
};

struct mxc_mipi_csi2_plat_data {
	struct mxc_mipi_csi2_clk_ops *clk_ops;
	struct mxc_mipi_csi2_pd_ops *pd_ops;
	struct mxc_mipi_csi2_rst_ops *rst_ops;
};

struct mxc_mipi_csi2_dev {
	struct v4l2_subdev	sd;
	struct v4l2_device	v4l2_dev;
	struct v4l2_subdev	*sensor_sd;

	struct media_pad pads[MXC_MIPI_CSI2_VCX_PADS_NUM];
	struct v4l2_mbus_framefmt format;

	void __iomem *csr_regs;
	void __iomem *base_regs;
	struct platform_device *pdev;
	u32 flags;
	int irq;

	struct clk *clk_core;
	struct clk *clk_esc;
	struct clk *clk_pxl;
	struct clk *clk_regs;

	const struct mxc_mipi_csi2_plat_data *pdata;

	struct csis_hw_reset hw_reset;
	struct csis_phy_gpr  phy_gpr;

	struct v4l2_async_subdev    asd;
	struct v4l2_async_notifier  subdev_notifier;
	struct v4l2_async_subdev    *async_subdevs[2];

	struct device *pd_csi;
	struct device *pd_isi;
	struct device_link *pd_csi_link;
	struct device_link *pd_isi_link;

	struct mutex lock;

	int id;
	u32 hs_settle;
	u32 send_level;
	u32 num_lanes;
	u8  data_lanes[4];
	u8  vchannel;
	u8  running;
};

struct mxc_hs_info {
	u32 width;
	u32 height;
	u32 frame_rate;
	u32 val;
};

enum mxc_mipi_csi2_pm_state {
	MXC_MIPI_CSI2_PM_POWERED	= 0x1,
	MXC_MIPI_CSI2_PM_SUSPENDED	= 0x2,
	MXC_MIPI_CSI2_RUNTIME_SUSPENDED	= 0x4,
};

/* 0~ 80Mbps: 0xB
 * 80~250Mbps: 0x8
 * 250~1.5Gbps: 0x6
 */
static u8 rxhs_settle[3] = {0xD, 0xA, 0x7};

static struct mxc_hs_info hs_setting[] = {
	{2592, 1944, 30, 0x0B},
	{2592, 1944, 15, 0x10},

	{1920, 1080, 30, 0x0B},
	{1920, 1080, 15, 0x10},

	{1280, 720,  30, 0x11},
	{1280, 720,  15, 0x16},

	{1024, 768,  30, 0x11},
	{1024, 768,  15, 0x23},

	{720,  576,  30, 0x1E},
	{720,  576,  15, 0x23},

	{720,  480,  30, 0x1E},
	{720,  480,  15, 0x23},

	{640,  480,  30, 0x1E},
	{640,  480,  15, 0x23},

	{320,  240,  30, 0x1E},
	{320,  240,  15, 0x23},

	{176,  144,  30, 0x1E},
	{176,  144,  15, 0x23},
};

static struct imx_sc_ipc *pm_ipc_handle;

static inline struct mxc_mipi_csi2_dev *sd_to_mxc_mipi_csi2_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct mxc_mipi_csi2_dev, sd);
}

/****************************************
 * rxhs-settle calculate
 * UI = 1000 / mipi csi phy clock
 * THS-SETTLE_mim = 85ns + 6 * UI
 * THS-SETTLE_max = 145ns +10 * UI
 * THS-SETTLE = (THS-SETTLE_mim + THS-SETTLE_max) / 2
 * PRG_RXHS_SETTLE =  THS-SETTLE / (Tperiod of RxClk_ESC) + 1
 ****************************************/
static int calc_hs_settle(struct mxc_mipi_csi2_dev *csi2dev, u32 dphy_clk)
{
	u32 esc_rate;
	u32 hs_settle;
	u32 rxhs_settle;
	u32 hs_settle_min;
	u32 hs_settle_max;

	esc_rate = clk_get_rate(csi2dev->clk_esc) / 1000000;
	if (!esc_rate)
		return 0;

	hs_settle_min = 85 + 6 * 1000 / dphy_clk;
	hs_settle_max = 145 + 10 * 1000 / dphy_clk;
	hs_settle = (hs_settle_min + hs_settle_max) >> 1;
	rxhs_settle = hs_settle / (1000 / esc_rate) - 1;
	return rxhs_settle;
}

static void mxc_mipi_csi2_reg_dump(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct {
		u32 offset;
		const char name[32];
	} registers[] = {
		{ 0x100, "MIPI CSI2 HC num of lanes" },
		{ 0x104, "MIPI CSI2 HC dis lanes" },
		{ 0x108, "MIPI CSI2 HC BIT ERR" },
		{ 0x10C, "MIPI CSI2 HC IRQ STATUS" },
		{ 0x110, "MIPI CSI2 HC IRQ MASK" },
		{ 0x114, "MIPI CSI2 HC ULPS STATUS" },
		{ 0x118, "MIPI CSI2 HC DPHY ErrSotHS" },
		{ 0x11c, "MIPI CSI2 HC DPHY ErrSotSync" },
		{ 0x120, "MIPI CSI2 HC DPHY ErrEsc" },
		{ 0x124, "MIPI CSI2 HC DPHY ErrSyncEsc" },
		{ 0x128, "MIPI CSI2 HC DPHY ErrControl" },
		{ 0x12C, "MIPI CSI2 HC DISABLE_PAYLOAD" },
		{ 0x130, "MIPI CSI2 HC DISABLE_PAYLOAD" },
		{ 0x180, "MIPI CSI2 HC IGNORE_VC" },
		{ 0x184, "MIPI CSI2 HC VID_VC" },
		{ 0x188, "MIPI CSI2 HC FIFO_SEND_LEVEL" },
		{ 0x18C, "MIPI CSI2 HC VID_VSYNC" },
		{ 0x190, "MIPI CSI2 HC VID_SYNC_FP" },
		{ 0x194, "MIPI CSI2 HC VID_HSYNC" },
		{ 0x198, "MIPI CSI2 HC VID_HSYNC_BP" },
		{ 0x000, "MIPI CSI2 CSR PLM_CTRL" },
		{ 0x004, "MIPI CSI2 CSR PHY_CTRL" },
		{ 0x008, "MIPI CSI2 CSR PHY_Status" },
		{ 0x010, "MIPI CSI2 CSR PHY_Test_Status" },
		{ 0x014, "MIPI CSI2 CSR PHY_Test_Status" },
		{ 0x018, "MIPI CSI2 CSR PHY_Test_Status" },
		{ 0x01C, "MIPI CSI2 CSR PHY_Test_Status" },
		{ 0x020, "MIPI CSI2 CSR PHY_Test_Status" },
		{ 0x030, "MIPI CSI2 CSR VC Interlaced" },
		{ 0x038, "MIPI CSI2 CSR Data Type Dis" },
		{ 0x040, "MIPI CSI2 CSR 420 1st type" },
		{ 0x044, "MIPI CSI2 CSR Ctr_Ck_Rst_Ctr" },
		{ 0x048, "MIPI CSI2 CSR Stream Fencing" },
		{ 0x04C, "MIPI CSI2 CSR Stream Fencing" },
	};
	u32 i;

	dev_dbg(dev, "MIPI CSI2 CSR and HC register dump, mipi csi%d\n", csi2dev->id);
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 reg = readl(csi2dev->base_regs + registers[i].offset);
		dev_dbg(dev, "%20s[0x%.3x]: 0x%.3x\n",
			registers[i].name, registers[i].offset, reg);
	}
}

static int mipi_sc_fw_init(struct mxc_mipi_csi2_dev *csi2dev, int enable)
{
	const struct mxc_mipi_csi2_rst_ops *ops = csi2dev->pdata->rst_ops;

	if (ops && ops->reset)
		return ops->reset(csi2dev, enable);

	return 0;
}

static uint16_t find_hs_configure(struct v4l2_subdev_format *sd_fmt)
{
	struct v4l2_mbus_framefmt *fmt = &sd_fmt->format;
	u32 frame_rate;
	int i;

	if (!fmt)
		return -EINVAL;

	frame_rate = fmt->reserved[1];

	for (i = 0; i < ARRAY_SIZE(hs_setting); i++) {
		if (hs_setting[i].width  == fmt->width &&
		    hs_setting[i].height == fmt->height &&
		    hs_setting[i].frame_rate == frame_rate)
			return hs_setting[i].val;
	}

	if (i == ARRAY_SIZE(hs_setting))
		pr_err("can not find HS setting for w/h@fps=(%d, %d)@%d\n",
		       fmt->width, fmt->height, frame_rate);

	return -EINVAL;
}

static void mxc_mipi_csi2_reset(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val;

	/* Reset MIPI CSI */
	val = CSI2SS_CTRL_CLK_RESET_EN | CSI2SS_CTRL_CLK_RESET_CLK_OFF;
	writel(val, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	u32 val = 0;

	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	while (val & CSI2SS_PLM_CTRL_PL_CLK_RUN) {
		msleep(10);
		val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
		dev_dbg(dev, "Waiting pl clk running, val=0x%x\n", val);
	}

	/* Enable Pixel link Master*/
	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	val |= CSI2SS_PLM_CTRL_ENABLE_PL;
	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	val |= CSI2SS_PLM_CTRL_VALID_OVERRIDE;
	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	/* PHY Enable */
	val = readl(csi2dev->csr_regs + CSI2SS_PHY_CTRL);
	val &= ~(CSI2SS_PHY_CTRL_PD_MASK | CSI2SS_PLM_CTRL_POLARITY_MASK);
	writel(val, csi2dev->csr_regs + CSI2SS_PHY_CTRL);

	/* Deassert reset */
	writel(1, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	/* Disable Data lanes */
	writel(0xf, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);

	/* Disable Pixel Link */
	writel(0, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	/* Disable  PHY */
	writel(0, csi2dev->csr_regs + CSI2SS_PHY_CTRL);

	/* Reset */
	writel(2, csi2dev->csr_regs + CSI2SS_CTRL_CLK_RESET);
}

static void mxc_mipi_csi2_csr_config(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val;

	/* format */
	val = 0;
	writel(val, csi2dev->csr_regs + CSI2SS_DATA_TYPE);

	/* polarity */
	val = readl(csi2dev->csr_regs + CSI2SS_PLM_CTRL);
	val &= ~(CSI2SS_PLM_CTRL_VSYNC_OVERRIDE |
		 CSI2SS_PLM_CTRL_HSYNC_OVERRIDE |
		 CSI2SS_PLM_CTRL_VALID_OVERRIDE |
		 CSI2SS_PLM_CTRL_POLARITY_MASK);

	writel(val, csi2dev->csr_regs + CSI2SS_PLM_CTRL);

	val = CSI2SS_PHY_CTRL_RX_ENABLE |
	      CSI2SS_PHY_CTRL_DDRCLK_EN << CSI2SS_PHY_CTRL_DDRCLK_EN_OFFSET |
	      CSI2SS_PHY_CTRL_CONT_CLK_MODE << CSI2SS_PHY_CTRL_CONT_CLK_MODE_OFFSET |
	      csi2dev->hs_settle << CSI2SS_PHY_CTRL_RX_HS_SETTLE_OFFSET |
	      CSI2SS_PHY_CTRL_PD << CSI2SS_PHY_CTRL_PD_OFFSET |
	      CSI2SS_PHY_CTRL_RTERM_SEL << CSI2SS_PHY_CTRL_RTERM_SEL_OFFSET |
	      CSI2SS_PHY_CTRL_AUTO_PD_EN << CSI2SS_PHY_CTRL_AUTO_PD_EN_OFFSET;

	writel(val, csi2dev->csr_regs + CSI2SS_PHY_CTRL);
}

static void mxc_mipi_csi2_hc_config(struct mxc_mipi_csi2_dev *csi2dev)
{
	u32 val0, val1;
	u32 i;

	val0 = 0;

	/* Lanes */
	writel(csi2dev->num_lanes - 1, csi2dev->base_regs + CSI2RX_CFG_NUM_LANES);

	for (i = 0; i < csi2dev->num_lanes; i++)
		val0 |= (1 << (csi2dev->data_lanes[i] - 1));

	val1 = 0xF & ~val0;
	writel(val1, csi2dev->base_regs + CSI2RX_CFG_DISABLE_DATA_LANES);

	/* Mask interrupt */
	writel(0x1FF, csi2dev->base_regs + CSI2RX_IRQ_MASK);

	/* vid_vc */
	writel(3, csi2dev->base_regs + 0x184);
}

static struct media_pad *mxc_csi2_get_remote_sensor_pad(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct v4l2_subdev *subdev = &csi2dev->sd;
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
		v4l2_err(&csi2dev->sd, "%s, No remote pad found!\n", __func__);

	return NULL;
}

static struct v4l2_subdev *mxc_get_remote_subdev(struct mxc_mipi_csi2_dev *csi2dev,
						 const char * const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_csi2_get_remote_sensor_pad(csi2dev);
	if (!source_pad) {
		v4l2_err(&csi2dev->sd, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&csi2dev->sd, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	return sen_sd;
}

static int mxc_csi2_get_sensor_fmt(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct v4l2_mbus_framefmt *mf = &csi2dev->format;
	struct v4l2_subdev *sen_sd;
	struct v4l2_subdev_format src_fmt;
	struct media_pad *source_pad;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_csi2_get_remote_sensor_pad(csi2dev);
	if (!source_pad) {
		v4l2_err(&csi2dev->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	memset(&src_fmt, 0, sizeof(src_fmt));
	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return -EINVAL;

	/* Update input frame size and formate  */
	memcpy(mf, &src_fmt.format, sizeof(struct v4l2_mbus_framefmt));

	dev_dbg(&csi2dev->pdev->dev, "width=%d, height=%d, fmt.code=0x%x\n",
		mf->width, mf->height, mf->code);

	/* Get rxhs settle */
	if (src_fmt.format.reserved[0] != 0) {
		csi2dev->hs_settle =
			calc_hs_settle(csi2dev, src_fmt.format.reserved[0]);
	} else if (src_fmt.format.reserved[1] != 0) {
		csi2dev->hs_settle = find_hs_configure(&src_fmt);
	} else {
		if (src_fmt.format.height * src_fmt.format.width > 1024 * 768)
			csi2dev->hs_settle = rxhs_settle[2];
		else if (src_fmt.format.height * src_fmt.format.width < 480 * 320)
			csi2dev->hs_settle = rxhs_settle[0];
		else
			csi2dev->hs_settle = rxhs_settle[1];
	}

	return 0;
}

static int mxc_imx8_csi2_clk_get(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;

	csi2dev->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(csi2dev->clk_core)) {
		dev_err(dev, "failed to get csi core clk\n");
		return PTR_ERR(csi2dev->clk_core);
	}

	csi2dev->clk_esc = devm_clk_get(dev, "clk_esc");
	if (IS_ERR(csi2dev->clk_esc)) {
		dev_err(dev, "failed to get csi esc clk\n");
		return PTR_ERR(csi2dev->clk_esc);
	}

	csi2dev->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(csi2dev->clk_pxl)) {
		dev_err(dev, "failed to get csi pixel link clk\n");
		return PTR_ERR(csi2dev->clk_pxl);
	}

	return 0;
}

static int mxc_imx8_csi2_clk_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	int ret;

	ret = clk_prepare_enable(csi2dev->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_core error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_esc);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_esc error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(csi2dev->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_pxl error\n", __func__);
		return ret;
	}

	return 0;
}

static void mxc_imx8_csi2_clk_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	clk_disable_unprepare(csi2dev->clk_core);
	clk_disable_unprepare(csi2dev->clk_esc);
	clk_disable_unprepare(csi2dev->clk_pxl);
}

static struct mxc_mipi_csi2_clk_ops mxc_csi2_clk_ops = {
	.clk_get     = mxc_imx8_csi2_clk_get,
	.clk_enable  = mxc_imx8_csi2_clk_enable,
	.clk_disable = mxc_imx8_csi2_clk_disable,
};

static int mxc_imx8_csi2_pd_attach(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_link *link;

	csi2dev->pd_csi = dev_pm_domain_attach_by_name(dev, "pd_csi");
	if (IS_ERR(csi2dev->pd_csi)) {
		if (PTR_ERR(csi2dev->pd_csi) != -EPROBE_DEFER) {
			dev_err(dev, "attach pd_csi domain for csi fail\n");
			return PTR_ERR(csi2dev->pd_csi);
		} else {
			return PTR_ERR(csi2dev->pd_csi);
		}
	}
	link = device_link_add(dev, csi2dev->pd_csi,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME);
	if (IS_ERR(link))
		return PTR_ERR(link);
	csi2dev->pd_csi_link = link;

	csi2dev->pd_isi = dev_pm_domain_attach_by_name(dev, "pd_isi_ch0");
	if (IS_ERR(csi2dev->pd_isi)) {
		if (PTR_ERR(csi2dev->pd_isi) != -EPROBE_DEFER) {
			dev_err(dev, "attach pd_isi_ch0 domain for csi fail\n");
			return PTR_ERR(csi2dev->pd_isi);
		} else {
			return PTR_ERR(csi2dev->pd_isi);
		}
	}
	link = device_link_add(dev, csi2dev->pd_isi,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME);
	if (IS_ERR(link))
		return PTR_ERR(link);
	csi2dev->pd_isi_link = link;

	return 0;
}

static void mxc_imx8_csi2_pd_detach(struct mxc_mipi_csi2_dev *csi2dev)
{
	device_link_del(csi2dev->pd_csi_link);
	device_link_del(csi2dev->pd_isi_link);
	dev_pm_domain_detach(csi2dev->pd_csi, true);
	dev_pm_domain_detach(csi2dev->pd_isi, true);
}

static struct mxc_mipi_csi2_pd_ops mxc_csi2_pd_ops = {
	.attach = mxc_imx8_csi2_pd_attach,
	.detach = mxc_imx8_csi2_pd_detach,
};

static int mxc_imx8_csi2_reset(struct mxc_mipi_csi2_dev *csi2dev, int enable)
{
	struct device *dev = &csi2dev->pdev->dev;
	u32 rsrc_id;
	int ret;

	ret = imx_scu_get_handle(&pm_ipc_handle);
	if (ret) {
		dev_err(dev, "sc_misc_MIPI get ipc handle failed! ret = (%d)\n", ret);
		return ret;
	}

	if (csi2dev->id == 1)
		rsrc_id = IMX_SC_R_CSI_1;
	else
		rsrc_id = IMX_SC_R_CSI_0;

	ret = imx_sc_misc_set_control(pm_ipc_handle,
				      rsrc_id, IMX_SC_C_MIPI_RESET, enable);
	if (ret < 0) {
		dev_err(dev, "sc_misc_MIPI reset failed! ret = (%d)\n", ret);
		return ret;
	}

	msleep(10);
	return 0;
}

static struct mxc_mipi_csi2_rst_ops mxc_csi2_rst_ops = {
	.reset = mxc_imx8_csi2_reset,
};

static struct mxc_mipi_csi2_plat_data mxc_imx8_csi2_pdata = {
	.clk_ops = &mxc_csi2_clk_ops,
	.pd_ops  = &mxc_csi2_pd_ops,
	.rst_ops = &mxc_csi2_rst_ops,
};

static int mxc_imx8ulp_csi2_clk_get(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;

	csi2dev->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(csi2dev->clk_core)) {
		dev_err(dev, "failed to get csi core clk\n");
		return PTR_ERR(csi2dev->clk_core);
	}

	csi2dev->clk_pxl = devm_clk_get(dev, "clk_ui");
	if (IS_ERR(csi2dev->clk_pxl)) {
		dev_err(dev, "failed to get csi ui clk\n");
		return PTR_ERR(csi2dev->clk_pxl);
	}

	csi2dev->clk_esc = devm_clk_get(dev, "clk_esc");
	if (IS_ERR(csi2dev->clk_esc)) {
		dev_err(dev, "failed to get csi esc clk\n");
		return PTR_ERR(csi2dev->clk_esc);
	}

	csi2dev->clk_regs = devm_clk_get(dev, "clk_regs");
	if (IS_ERR(csi2dev->clk_regs)) {
		dev_err(dev, "failed to get csi regs clk\n");
		return PTR_ERR(csi2dev->clk_regs);
	}

	return 0;
}

static int mxc_imx8ulp_csi2_clk_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	int ret;

	ret = clk_prepare_enable(csi2dev->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk_core error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(csi2dev->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk ui error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(csi2dev->clk_esc);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk esc error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(csi2dev->clk_regs);
	if (ret < 0) {
		dev_err(dev, "%s, prepare clk regs error\n", __func__);
		return ret;
	}

	return 0;
}

static void mxc_imx8ulp_csi2_clk_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	clk_disable_unprepare(csi2dev->clk_core);
	clk_disable_unprepare(csi2dev->clk_pxl);
	clk_disable_unprepare(csi2dev->clk_esc);
	clk_disable_unprepare(csi2dev->clk_regs);
}

static struct mxc_mipi_csi2_clk_ops mxc_imx8ulp_csi2_clk_ops = {
	.clk_get     = mxc_imx8ulp_csi2_clk_get,
	.clk_enable  = mxc_imx8ulp_csi2_clk_enable,
	.clk_disable = mxc_imx8ulp_csi2_clk_disable,
};

static int mxc_imx8ulp_csi2_pd_attach(struct mxc_mipi_csi2_dev *csi2dev)
{
	return 0;
}

static void mxc_imx8ulp_csi2_pd_detach(struct mxc_mipi_csi2_dev *csi2dev)
{
}

static struct mxc_mipi_csi2_pd_ops mxc_imx8ulp_csi2_pd_ops = {
	.attach = mxc_imx8ulp_csi2_pd_attach,
	.detach = mxc_imx8ulp_csi2_pd_detach,
};

static int mxc_imx8ulp_csi2_reset(struct mxc_mipi_csi2_dev *csi2dev, int enable)
{
	return 0;
}

static struct mxc_mipi_csi2_rst_ops mxc_imx8ulp_csi2_rst_ops = {
	.reset = mxc_imx8ulp_csi2_reset,
};

static struct mxc_mipi_csi2_plat_data mxc_imx8ulp_csi2_pdata = {
	.clk_ops = &mxc_imx8ulp_csi2_clk_ops,
	.pd_ops  = &mxc_imx8ulp_csi2_pd_ops,
	.rst_ops = &mxc_imx8ulp_csi2_rst_ops,
};

static int mipi_csi2_clk_init(struct mxc_mipi_csi2_dev *csi2dev)
{

	const struct mxc_mipi_csi2_clk_ops *ops = csi2dev->pdata->clk_ops;

	if (!ops || !ops->clk_get)
		return -EINVAL;

	return ops->clk_get(csi2dev);
}

static int mipi_csi2_attach_pd(struct mxc_mipi_csi2_dev *csi2dev)
{
	const struct mxc_mipi_csi2_pd_ops *ops = csi2dev->pdata->pd_ops;

	if (ops && ops->attach)
		return ops->attach(csi2dev);

	return 0;
}

static void mipi_csi2_detach_pd(struct mxc_mipi_csi2_dev *csi2dev)
{
	const struct mxc_mipi_csi2_pd_ops *ops = csi2dev->pdata->pd_ops;

	if (ops && ops->detach)
		ops->detach(csi2dev);
}

static int mipi_csi2_clk_enable(struct mxc_mipi_csi2_dev *csi2dev)
{
	const struct mxc_mipi_csi2_clk_ops *ops = csi2dev->pdata->clk_ops;

	if (!ops || !ops->clk_enable)
		return -EINVAL;

	return ops->clk_enable(csi2dev);
}

static void mipi_csi2_clk_disable(struct mxc_mipi_csi2_dev *csi2dev)
{
	const struct mxc_mipi_csi2_clk_ops *ops = csi2dev->pdata->clk_ops;

	if (!ops || !ops->clk_disable)
		return;

	ops->clk_disable(csi2dev);
}

static int mipi_csi2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

/* mipi csi2 subdev media entity operations */
static int mipi_csi2_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	/* TODO */
	/* Add MIPI source and sink pad link configuration */
	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_MIPI_CSI2_VC0_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC1_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC2_PAD_SOURCE:
		case MXC_MIPI_CSI2_VC3_PAD_SOURCE:
			break;
		default:
			return 0;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case MXC_MIPI_CSI2_VC0_PAD_SINK:
		case MXC_MIPI_CSI2_VC1_PAD_SINK:
		case MXC_MIPI_CSI2_VC2_PAD_SINK:
		case MXC_MIPI_CSI2_VC3_PAD_SINK:
			break;
		default:
			return 0;
		}
	}
	return 0;
}

static const struct media_entity_operations mipi_csi2_sd_media_ops = {
	.link_setup = mipi_csi2_link_setup,
};

/*
 * V4L2 subdev operations
 */
static int mipi_csi2_s_power(struct v4l2_subdev *sd, int on)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, core, s_power, on);
}

static int mipi_csi2_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, g_frame_interval, interval);
}

static int mipi_csi2_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, s_frame_interval, interval);
}

static int mipi_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct device *dev = &csi2dev->pdev->dev;
	int ret = 0;

	dev_dbg(&csi2dev->pdev->dev, "%s: %d, csi2dev: 0x%x\n",
		__func__, enable, csi2dev->flags);

	if (enable) {
		pm_runtime_get_sync(dev);
		if (!csi2dev->running++) {
			mxc_csi2_get_sensor_fmt(csi2dev);
			mxc_mipi_csi2_hc_config(csi2dev);
			mxc_mipi_csi2_reset(csi2dev);
			mxc_mipi_csi2_csr_config(csi2dev);
			mxc_mipi_csi2_enable(csi2dev);
			mxc_mipi_csi2_reg_dump(csi2dev);
		}
	} else {
		if (!--csi2dev->running)
			mxc_mipi_csi2_disable(csi2dev);

		pm_runtime_put(dev);
	}

	return ret;
}

static int mipi_csi2_enum_framesizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_size, NULL, fse);
}

static int mipi_csi2_enum_frame_interval(struct v4l2_subdev *sd,
					 struct v4l2_subdev_pad_config *cfg,
					 struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_interval, NULL, fie);
}

static int mipi_csi2_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mxc_csi2_get_sensor_fmt(csi2dev);

	memcpy(mf, &csi2dev->format, sizeof(struct v4l2_mbus_framefmt));
	/* Source/Sink pads crop rectangle size */

	return 0;
}

static int mipi_csi2_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);
	struct v4l2_subdev *sen_sd;
	struct media_pad *source_pad;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_csi2_get_remote_sensor_pad(csi2dev);
	if (!source_pad) {
		v4l2_err(&csi2dev->sd, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	sen_sd = mxc_get_remote_subdev(csi2dev, __func__);
	if (!sd)
		return -EINVAL;

	fmt->pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, set_fmt, NULL, fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	return 0;
}

static const struct v4l2_subdev_internal_ops mipi_csi2_sd_internal_ops = {
	.open = mipi_csi2_open,
};

static struct v4l2_subdev_pad_ops mipi_csi2_pad_ops = {
	.enum_frame_size = mipi_csi2_enum_framesizes,
	.enum_frame_interval = mipi_csi2_enum_frame_interval,
	.get_fmt = mipi_csi2_get_fmt,
	.set_fmt = mipi_csi2_set_fmt,
};

static struct v4l2_subdev_core_ops mipi_csi2_core_ops = {
	.s_power = mipi_csi2_s_power,
};

static struct v4l2_subdev_video_ops mipi_csi2_video_ops = {
	.g_frame_interval = mipi_csi2_g_frame_interval,
	.s_frame_interval = mipi_csi2_s_frame_interval,
	.s_stream	  = mipi_csi2_s_stream,
};

static struct v4l2_subdev_ops mipi_csi2_subdev_ops = {
	.core = &mipi_csi2_core_ops,
	.video = &mipi_csi2_video_ops,
	.pad = &mipi_csi2_pad_ops,
};

static int mipi_csi2_parse_dt(struct mxc_mipi_csi2_dev *csi2dev)
{
	struct device *dev = &csi2dev->pdev->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_fwnode_endpoint endpoint;
	u32 i;

	csi2dev->id = of_alias_get_id(node, "csi");

	csi2dev->vchannel = of_property_read_bool(node, "virtual-channel");

	node = of_graph_get_next_endpoint(node, NULL);
	if (!node) {
		dev_err(dev, "No port node\n");
		return -EINVAL;
	}

	/* Get port node */
	memset(&endpoint, 0x0, sizeof(endpoint));
	v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &endpoint);

	csi2dev->num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	for (i = 0; i < 4; i++)
		csi2dev->data_lanes[i] = endpoint.bus.mipi_csi2.data_lanes[i];

	of_node_put(node);
	return 0;
}

static int mipi_csi2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct mxc_mipi_csi2_dev *csi2dev;
	const struct of_device_id *of_id;
	int ret = -ENOMEM;

	csi2dev = devm_kzalloc(dev, sizeof(*csi2dev), GFP_KERNEL);
	if (!csi2dev)
		return -ENOMEM;

	csi2dev->pdev = pdev;
	mutex_init(&csi2dev->lock);

	ret = mipi_csi2_parse_dt(csi2dev);
	if (ret < 0)
		return ret;

	of_id = of_match_node(mipi_csi2_of_match, dev->of_node);
	if (!of_id)
		return -EINVAL;

	csi2dev->pdata = of_id->data;
	if (!csi2dev->pdata) {
		dev_err(dev, "Can't get csi platform device data\n");
		return -EINVAL;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi2dev->base_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2dev->base_regs)) {
		dev_err(dev, "Failed to get mipi csi2 HC register\n");
		return PTR_ERR(csi2dev->base_regs);
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	csi2dev->csr_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(csi2dev->csr_regs)) {
		dev_err(dev, "Failed to get mipi CSR register\n");
		return PTR_ERR(csi2dev->csr_regs);
	}

	ret = mipi_csi2_clk_init(csi2dev);
	if (ret < 0)
		return ret;

	ret = mipi_csi2_attach_pd(csi2dev);
	if (ret < 0)
		return ret;

	v4l2_subdev_init(&csi2dev->sd, &mipi_csi2_subdev_ops);

	csi2dev->sd.owner = THIS_MODULE;
	snprintf(csi2dev->sd.name, sizeof(csi2dev->sd.name), "%s.%d",
		 MXC_MIPI_CSI2_SUBDEV_NAME, csi2dev->id);

	csi2dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csi2dev->sd.entity.function = MEDIA_ENT_F_IO_V4L;
	csi2dev->sd.dev = dev;

	csi2dev->pads[MXC_MIPI_CSI2_VC0_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC1_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC2_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC3_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi2dev->pads[MXC_MIPI_CSI2_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	csi2dev->pads[MXC_MIPI_CSI2_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&csi2dev->sd.entity,
				     MXC_MIPI_CSI2_VCX_PADS_NUM, csi2dev->pads);
	if (ret < 0)
		goto e_clkdis;

	csi2dev->sd.entity.ops = &mipi_csi2_sd_media_ops;

	v4l2_set_subdevdata(&csi2dev->sd, pdev);
	platform_set_drvdata(pdev, csi2dev);

	mipi_sc_fw_init(csi2dev, 1);

	csi2dev->running = 0;
	pm_runtime_enable(dev);

	dev_info(&pdev->dev, "lanes: %d, name: %s\n",
		 csi2dev->num_lanes, csi2dev->sd.name);

	return 0;

e_clkdis:
	media_entity_cleanup(&csi2dev->sd.entity);
	return ret;
}

static int mipi_csi2_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);

	mipi_sc_fw_init(csi2dev, 0);
	mipi_csi2_detach_pd(csi2dev);
	media_entity_cleanup(&csi2dev->sd.entity);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int  mipi_csi2_pm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mxc_mipi_csi2_dev *csi2dev = sd_to_mxc_mipi_csi2_dev(sd);

	if (csi2dev->running > 0) {
		dev_warn(dev, "running, prevent entering suspend.\n");
		return -EAGAIN;
	}

	return pm_runtime_force_suspend(dev);
}

static int  mipi_csi2_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static int  mipi_csi2_runtime_suspend(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);

	mipi_csi2_clk_disable(csi2dev);
	return 0;
}

static int  mipi_csi2_runtime_resume(struct device *dev)
{
	struct mxc_mipi_csi2_dev *csi2dev = dev_get_drvdata(dev);
	int ret;

	ret = mipi_csi2_clk_enable(csi2dev);
	if (ret)
		return ret;

	return 0;
}

static const struct dev_pm_ops mipi_csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mipi_csi2_pm_suspend, mipi_csi2_pm_resume)
	SET_RUNTIME_PM_OPS(mipi_csi2_runtime_suspend, mipi_csi2_runtime_resume, NULL)
};

static const struct of_device_id mipi_csi2_of_match[] = {
	{ .compatible = "fsl,mxc-mipi-csi2", .data = &mxc_imx8_csi2_pdata },
	{ .compatible = "fsl,imx8ulp-mipi-csi2", .data = &mxc_imx8ulp_csi2_pdata },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mipi_csi2_of_match);

static struct platform_driver mipi_csi2_driver = {
	.driver = {
		.name = MXC_MIPI_CSI2_DRIVER_NAME,
		.of_match_table = mipi_csi2_of_match,
		.pm = &mipi_csi_pm_ops,
	},
	.probe = mipi_csi2_probe,
	.remove = mipi_csi2_remove,
};

module_platform_driver(mipi_csi2_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC MIPI CSI2 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_MIPI_CSI2_DRIVER_NAME);
