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
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <linux/firmware/imx/sci.h>
#include <dt-bindings/pinctrl/pads-imx8qxp.h>
#include <linux/init.h>
#include <linux/pm_domain.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "imx8-common.h"

#define MXC_PARALLEL_CSI_DRIVER_NAME	"mxc-parallel-csi"
#define MXC_PARALLEL_CSI_SUBDEV_NAME	MXC_PARALLEL_CSI_DRIVER_NAME

#define BIT_U(nr)		(1U << (nr))
#define CI_PI_BASE_OFFSET	0x0U

/* CI_PI INTERFACE Control */
#define IF_CTRL_REG_PL_ENABLE           BIT_U(0)
#define IF_CTRL_REG_PL_VALID            BIT_U(1)
#define IF_CTRL_REG_PL_ADDR(x)          (((x) & 0x7U) << 2)
#define IF_CTRL_REG_IF_FORCE(x)         (((x) & 0x7U) << 5)
#define IF_CTRL_REG_DATA_TYPE_SEL       BIT_U(8)
#define IF_CTRL_REG_DATA_TYPE(x)        (((x) & 0x1FU) << 9)

#define DATA_TYPE_OUT_NULL           (0x00)
#define DATA_TYPE_OUT_RGB            (0x04)
#define DATA_TYPE_OUT_YUV444         (0x08)
#define DATA_TYPE_OUT_YYU420_ODD     (0x10)
#define DATA_TYPE_OUT_YYU420_EVEN    (0x12)
#define DATA_TYPE_OUT_YYY_ODD        (0x18)
#define DATA_TYPE_OUT_UYVY_EVEN      (0x1A)
#define DATA_TYPE_OUT_RAW            (0x1C)

#define IF_CTRL_REG_IF_FORCE_HSYNV_OVERRIDE         0x4
#define IF_CTRL_REG_IF_FORCE_VSYNV_OVERRIDE         0x2
#define IF_CTRL_REG_IF_FORCE_DATA_ENABLE_OVERRIDE   0x1

/* CSI INTERFACE CONTROL REG */
#define CSI_CTRL_REG_CSI_EN                     BIT_U(0)
#define CSI_CTRL_REG_PIXEL_CLK_POL              BIT_U(1)
#define CSI_CTRL_REG_PIXEL_CLK_POL_OFFSET       (1)
#define CSI_CTRL_REG_HSYNC_POL                  BIT_U(2)
#define CSI_CTRL_REG_HSYNC_POL_OFFSET		(2)
#define CSI_CTRL_REG_VSYNC_POL                  BIT_U(3)
#define CSI_CTRL_REG_VSYNC_POL_OFFSET           (3)
#define CSI_CTRL_REG_DE_POL                     BIT_U(4)
#define CSI_CTRL_REG_PIXEL_DATA_POL             BIT_U(5)
#define CSI_CTRL_REG_CCIR_EXT_VSYNC_EN          BIT_U(6)
#define CSI_CTRL_REG_CCIR_EN                    BIT_U(7)
#define CSI_CTRL_REG_CCIR_VIDEO_MODE            BIT_U(8)
#define CSI_CTRL_REG_CCIR_NTSC_EN               BIT_U(9)
#define CSI_CTRL_REG_CCIR_VSYNC_RESET_EN        BIT_U(10)
#define CSI_CTRL_REG_CCIR_ECC_ERR_CORRECT_EN    BIT_U(11)
#define CSI_CTRL_REG_HSYNC_FORCE_EN             BIT_U(12)
#define CSI_CTRL_REG_VSYNC_FORCE_EN             BIT_U(13)
#define CSI_CTRL_REG_GCLK_MODE_EN               BIT_U(14)
#define CSI_CTRL_REG_VALID_SEL                  BIT_U(15)
#define CSI_CTRL_REG_RAW_OUT_SEL                BIT_U(16)
#define CSI_CTRL_REG_HSYNC_OUT_SEL              BIT_U(17)
#define CSI_CTRL_REG_HSYNC_PULSE(x)             (((x) & 0x7U) << 19)
#define CSI_CTRL_REG_UV_SWAP_EN                 BIT_U(22)
#define CSI_CTRL_REG_DATA_TYPE_IN(x)            (((x) & 0xFU) << 23)
#define CSI_CTRL_REG_MASK_VSYNC_COUNTER(x)      (((x) & 0x3U) << 27)
#define CSI_CTRL_REG_SOFTRST                    BIT_U(31)

#define DATA_TYPE_IN_UYVY_BT656_8BITS     0x0
#define DATA_TYPE_IN_UYVY_BT656_10BITS    0x1
#define DATA_TYPE_IN_RGB_8BITS            0x2
#define DATA_TYPE_IN_BGR_8BITS            0x3
#define DATA_TYPE_IN_RGB_24BITS           0x4
#define DATA_TYPE_IN_YVYU_8BITS           0x5
#define DATA_TYPE_IN_YUV_8BITS            0x6
#define DATA_TYPE_IN_YVYU_16BITS          0x7
#define DATA_TYPE_IN_YUV_24BITS           0x8
#define DATA_TYPE_IN_BAYER_8BITS          0x9
#define DATA_TYPE_IN_BAYER_10BITS         0xA
#define DATA_TYPE_IN_BAYER_12BITS         0xB
#define DATA_TYPE_IN_BAYER_16BITS         0xC

/* CSI interface Status */
#define CSI_STATUS_FIELD_TOGGLE         BIT_U(0)
#define CSI_STATUS_ECC_ERROR            BIT_U(1)

/* CSI INTERFACE CONTROL REG1 */
#define CSI_CTRL_REG1_PIXEL_WIDTH(v)    (((v) & 0xFFFFU) << 0)
#define CSI_CTRL_REG1_VSYNC_PULSE(v)    (((v) & 0xFFFFU) << 16)

#define DISP_MIX_CAMERA_MUX			0x30
#define DISP_MIX_CAMERA_MUX_DATA_TYPE(x)	(((x) & 0x3f) << 3)
#define DISP_MIX_CAMERA_MUX_GASKET_ENABLE	(1 << 16)

enum csi_in_data_type {
	CSI_IN_DT_UYVY_BT656_8	= 0x0,
	CSI_IN_DT_UYVY_BT656_10,
	CSI_IN_DT_RGB_8,
	CSI_IN_DT_BGR_8,
	CSI_IN_DT_YVYU_8	= 0x5,
	CSI_IN_DT_YUV_8,
	CSI_IN_DT_RAW_8		= 0x9,
	CSI_IN_DT_RAW_10,
};

enum source_type {
	SRC_TYPE_MIPI_CSI = 0,
	SRC_TYPE_PARALLEL,
};

struct mxc_parallel_csi_dev;

struct mxc_pcsi_power_domain_ops {
	int (*attach_pd)(struct mxc_parallel_csi_dev *pcsidev);
	void (*deattach_pd)(struct mxc_parallel_csi_dev *pcsidev);
};

struct mxc_pcsi_plat_data {
	u32 version;
	u32 if_ctrl_reg;
	u32 interface_status;
	u32 interface_ctrl_reg;
	u32 interface_ctrl_reg1;
	bool has_gasket;
	u8 def_hsync_pol;
	u8 def_vsync_pol;
	u8 def_pixel_clk_pol;
	u8 def_csi_in_data_type;
	const struct mxc_pcsi_power_domain_ops *pd_ops;
};

enum {
	PI_MODE_INIT,
	PI_GATE_CLOCK_MODE,
	PI_CCIR_MODE,
};

enum {
	PI_V1 = 0x0,
	PI_V2,
};

struct mxc_parallel_csi_dev {
	struct v4l2_subdev	sd;
	struct v4l2_device	v4l2_dev;
	struct v4l2_subdev	*sensor_sd;

	struct media_pad	pads[MXC_PARALLEL_CSI_PADS_NUM];

	void __iomem *csr_regs;
	void __iomem *lpcg_regs;
	struct device *dev;
	u32 flags;
	int irq;

	struct clk *clk_ipg;
	struct clk *clk_pixel;
	bool clk_enable;

	struct v4l2_async_subdev	asd;
	struct v4l2_async_notifier	subdev_notifier;
	struct v4l2_async_subdev	*async_subdevs[2];
	struct v4l2_mbus_framefmt	format;

	struct device *pd_pi;
	struct device *pd_isi;
	struct device_link *pd_pi_link;
	struct device_link *pd_isi_link;

	const struct mxc_pcsi_plat_data *pdata;
	struct regmap *gasket;

	struct mutex lock;

	u8 running;
	u8 mode;
	u8 uv_swap;
	u8 tvdec;
};

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

static int format;
module_param(format, int, 0644);
MODULE_PARM_DESC(format, "Format level (0-2)");

static const struct of_device_id parallel_csi_of_match[];

#ifdef DEBUG
static void mxc_pcsi_regs_dump(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	struct {
		u32 offset;
		const char *const name;
	} registers[] = {
		{ pdata->if_ctrl_reg, "HW_IF_CTRL_REG" },
		{ pdata->interface_ctrl_reg, "HW_CSI_CTRL_REG" },
		{ pdata->interface_status, "HW_CSI_STATUS" },
		{ pdata->interface_ctrl_reg1, "HW_CSI_CTRL_REG1" },

	};
	u32 i;

	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 reg = readl(pcsidev->csr_regs + registers[i].offset);
		dev_dbg(dev, "%20s[0x%.2x]: 0x%.8x\n",
			registers[i].name, registers[i].offset, reg);
	}
}
#else
static void mxc_pcsi_regs_dump(struct mxc_parallel_csi_dev *pcsidev) { }
#endif

static void gasket_dump(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;
	struct regmap *gasket = pcsidev->gasket;
	u32 val;

	if (!pcsidev->pdata->has_gasket)
		return;

	regmap_read(gasket, DISP_MIX_CAMERA_MUX, &val);
	dev_dbg(dev, "gasket: CAMERA MUX: %#x\n", val);
}

static struct mxc_parallel_csi_dev *sd_to_mxc_pcsi_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct mxc_parallel_csi_dev, sd);
}

static void disp_mix_gasket_config(struct mxc_parallel_csi_dev *pcsidev)
{
	struct regmap *gasket = pcsidev->gasket;
	u32 val;

	if (!pcsidev->pdata->has_gasket)
		return;

	regmap_read(gasket, DISP_MIX_CAMERA_MUX, &val);
	val |= (SRC_TYPE_PARALLEL << 17);
	regmap_write(gasket, DISP_MIX_CAMERA_MUX, val);
}

static int mxc_pcsi_clk_get(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;

	pcsidev->clk_pixel = devm_clk_get(dev, "pixel");
	if (IS_ERR(pcsidev->clk_pixel)) {
		dev_err(dev, "failed to get parallel csi pixel clk\n");
		return PTR_ERR(pcsidev->clk_pixel);
	}

	pcsidev->clk_ipg = devm_clk_get(dev, "ipg");
	if (IS_ERR(pcsidev->clk_ipg)) {
		dev_err(dev, "failed to get parallel ipg pixel clk\n");
		return PTR_ERR(pcsidev->clk_ipg);
	}

	return 0;
}

static int mxc_pcsi_attach_pd(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;

	if (!pdata || !pdata->pd_ops || !pdata->pd_ops->attach_pd)
		return 0;

	return pdata->pd_ops->attach_pd(pcsidev);
}

static void mxc_pcsi_detach_pd(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;

	if (!pdata || !pdata->pd_ops || !pdata->pd_ops->deattach_pd)
		return;

	return pdata->pd_ops->deattach_pd(pcsidev);
}

static int mxc_pcsi_clk_enable(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;
	int ret;

	if (pcsidev->clk_enable)
		return 0;

	ret = clk_prepare_enable(pcsidev->clk_pixel);
	if (ret < 0) {
		dev_err(dev, "enable pixel clk error (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(pcsidev->clk_ipg);
	if (ret < 0) {
		dev_err(dev, "enable ipg clk error (%d)\n", ret);
		return ret;
	}
	pcsidev->clk_enable = true;

	return 0;
}

static void mxc_pcsi_clk_disable(struct mxc_parallel_csi_dev *pcsidev)
{
	if (!pcsidev->clk_enable)
		return;

	clk_disable_unprepare(pcsidev->clk_pixel);
	clk_disable_unprepare(pcsidev->clk_ipg);

	pcsidev->clk_enable = false;
}

static void mxc_pcsi_sw_reset(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	u32 val;

	/* Softwaret Reset */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val |= CSI_CTRL_REG_SOFTRST;
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);

	msleep(1);
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val &= ~CSI_CTRL_REG_SOFTRST;
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);
}

static void mxc_pcsi_csr_config(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	u32 val;

	/* Software Reset */
	mxc_pcsi_sw_reset(pcsidev);

	/* Config PL Data Type */
	val = readl(pcsidev->csr_regs + pdata->if_ctrl_reg);
	val |= IF_CTRL_REG_DATA_TYPE(DATA_TYPE_OUT_YUV444);
	writel(val, pcsidev->csr_regs + pdata->if_ctrl_reg);

	/* Enable sync Force */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val |= (CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);

	/* Enable Pixel Link */
	val = readl(pcsidev->csr_regs + pdata->if_ctrl_reg);
	val |= IF_CTRL_REG_PL_ENABLE;
	writel(val, pcsidev->csr_regs + pdata->if_ctrl_reg);

	/* Enable Pixel Link */
	val = readl(pcsidev->csr_regs + pdata->if_ctrl_reg);
	val |= IF_CTRL_REG_PL_VALID;
	writel(val, pcsidev->csr_regs + pdata->if_ctrl_reg);

	/* Config CTRL REG */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);

	val |= (CSI_CTRL_REG_DATA_TYPE_IN(pdata->def_csi_in_data_type) |
		pdata->def_hsync_pol << CSI_CTRL_REG_HSYNC_POL_OFFSET |
		pdata->def_vsync_pol << CSI_CTRL_REG_VSYNC_POL_OFFSET |
		pdata->def_pixel_clk_pol << CSI_CTRL_REG_PIXEL_CLK_POL_OFFSET |
		CSI_CTRL_REG_MASK_VSYNC_COUNTER(3) |
		CSI_CTRL_REG_HSYNC_PULSE(2));

	if (pcsidev->uv_swap)
		val |= CSI_CTRL_REG_UV_SWAP_EN;

	if (pcsidev->mode & PI_GATE_CLOCK_MODE) {
		val |= CSI_CTRL_REG_GCLK_MODE_EN;
	} else if (pcsidev->mode & PI_CCIR_MODE) {
		val |= (CSI_CTRL_REG_CCIR_EN |
			CSI_CTRL_REG_CCIR_VSYNC_RESET_EN |
			CSI_CTRL_REG_CCIR_EXT_VSYNC_EN |
			CSI_CTRL_REG_CCIR_ECC_ERR_CORRECT_EN);
	}

	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);
}

static int get_interface_ctrl_reg1_param(struct mxc_parallel_csi_dev *pcsidev,
					 u32 *pixel_width, u32 *vsync_pulse)
{
	u32 version = pcsidev->pdata->version;

	switch (version) {
	case PI_V1:
		*pixel_width = pcsidev->format.width - 1;
		*vsync_pulse = pcsidev->format.width << 1;
		break;
	case PI_V2:
		*pixel_width = 10;
		*vsync_pulse = pcsidev->format.width - 1;
		break;
	default:
		dev_err(pcsidev->dev, "Not support PI version %d\n", version);
		return -EINVAL;
	}

	return 0;
}

static void mxc_pcsi_config_ctrl_reg1(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	u32 pixel_width;
	u32 vsync_pulse;
	u32 val;
	int ret;

	if (pcsidev->format.width <= 0 || pcsidev->format.height <= 0) {
		dev_dbg(dev, "%s width/height invalid\n", __func__);
		return;
	}

	ret = get_interface_ctrl_reg1_param(pcsidev, &pixel_width, &vsync_pulse);
	if (ret < 0)
		return;

	val = (CSI_CTRL_REG1_PIXEL_WIDTH(pixel_width) |
	       CSI_CTRL_REG1_VSYNC_PULSE(vsync_pulse));
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg1);
}

static void mxc_pcsi_enable_csi(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	u32 val;

	/* Enable CSI */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val |= CSI_CTRL_REG_CSI_EN;
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);

	/* Disable SYNC Force */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val &= ~(CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);
}

static void mxc_pcsi_disable_csi(struct mxc_parallel_csi_dev *pcsidev)
{
	const struct mxc_pcsi_plat_data *pdata = pcsidev->pdata;
	u32 val;

	/* Enable Sync Force */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val |= (CSI_CTRL_REG_HSYNC_FORCE_EN | CSI_CTRL_REG_VSYNC_FORCE_EN);
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);

	/* Disable CSI */
	val = readl(pcsidev->csr_regs + pdata->interface_ctrl_reg);
	val &= ~CSI_CTRL_REG_CSI_EN;
	writel(val, pcsidev->csr_regs + pdata->interface_ctrl_reg);

	/* Disable Pixel Link */
	val = readl(pcsidev->csr_regs + pdata->if_ctrl_reg);
	val &= ~(IF_CTRL_REG_PL_VALID | IF_CTRL_REG_PL_ENABLE);
	writel(val, pcsidev->csr_regs + pdata->if_ctrl_reg);
}

static struct media_pad *
mxc_pcsi_get_remote_sensor_pad(struct mxc_parallel_csi_dev *pcsidev)
{
	struct v4l2_subdev *subdev = &pcsidev->sd;
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
		v4l2_err(&pcsidev->v4l2_dev,
			 "%s, No remote pad found!\n", __func__);

	return NULL;
}

static struct v4l2_subdev *mxc_get_remote_subdev(struct mxc_parallel_csi_dev *pcsidev,
						 const char * const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (!source_pad) {
		v4l2_err(&pcsidev->sd, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&pcsidev->sd, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	return sen_sd;
}

static int mxc_pcsi_get_sensor_fmt(struct mxc_parallel_csi_dev *pcsidev)
{
	struct v4l2_mbus_framefmt *mf = &pcsidev->format;
	struct v4l2_subdev *sen_sd;
	struct media_pad *source_pad;
	struct v4l2_subdev_format src_fmt;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (!source_pad) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(sen_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return -EINVAL;

	/* Update input frame size and formate  */
	memcpy(mf, &src_fmt.format, sizeof(struct v4l2_mbus_framefmt));

	if (mf->code == MEDIA_BUS_FMT_YUYV8_2X8 ||
	    mf->code == MEDIA_BUS_FMT_UYVY8_2X8)
		pcsidev->uv_swap = 1;

	dev_dbg(pcsidev->dev, "width=%d, height=%d, fmt.code=0x%x\n",
				mf->width, mf->height, mf->code);

	return 0;
}

static int mxc_pcsi_enum_framesizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(pcsidev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_size, NULL, fse);
}

static int mxc_pcsi_enum_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *sd_state,
					struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(pcsidev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, pad, enum_frame_interval, NULL, fie);
}

static int mxc_pcsi_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mxc_pcsi_get_sensor_fmt(pcsidev);

	memcpy(mf, &pcsidev->format, sizeof(struct v4l2_mbus_framefmt));
	/* Source/Sink pads crop rectangle size */

	return 0;
}

static int mxc_pcsi_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;
	struct media_pad *source_pad;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_pcsi_get_remote_sensor_pad(pcsidev);
	if (!source_pad) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(&pcsidev->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	fmt->pad = source_pad->index;
	ret = v4l2_subdev_call(sen_sd, pad, set_fmt, NULL, fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		return ret;

	return 0;
}

static int mxc_pcsi_s_power(struct v4l2_subdev *sd, int on)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(pcsidev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, core, s_power, on);
}

static int mxc_pcsi_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(pcsidev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, g_frame_interval, interval);
}

static int mxc_pcsi_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct v4l2_subdev *sen_sd;

	sen_sd = mxc_get_remote_subdev(pcsidev, __func__);
	if (!sen_sd)
		return -EINVAL;

	return v4l2_subdev_call(sen_sd, video, s_frame_interval, interval);
}

static int mxc_pcsi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_parallel_csi_dev *pcsidev = sd_to_mxc_pcsi_dev(sd);
	struct device *dev = pcsidev->dev;

	dev_dbg(dev, "%s: enable = %d\n", __func__, enable);

	if (enable) {
		pm_runtime_get_sync(dev);
		if (!pcsidev->running) {
			mxc_pcsi_get_sensor_fmt(pcsidev);
			mxc_pcsi_csr_config(pcsidev);
			mxc_pcsi_config_ctrl_reg1(pcsidev);
			mxc_pcsi_enable_csi(pcsidev);
			disp_mix_gasket_config(pcsidev);
			mxc_pcsi_regs_dump(pcsidev);
			gasket_dump(pcsidev);
		}
		pcsidev->running++;
	} else {
		if (pcsidev->running) {
			mxc_pcsi_regs_dump(pcsidev);
			mxc_pcsi_disable_csi(pcsidev);
		}
		pcsidev->running--;
		pm_runtime_put(dev);
	}

	return 0;
}

static int mxc_pcsi_link_setup(struct media_entity *entity,
			       const struct media_pad *local,
			       const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct platform_device *pdev = v4l2_get_subdevdata(sd);

	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_PARALLEL_CSI_PAD_SOURCE:
			break;
		default:
			dev_err(&pdev->dev, "%s invalid source pad\n", __func__);
			return -EINVAL;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case MXC_PARALLEL_CSI_PAD_SINK:
			break;
		default:
			dev_err(&pdev->dev, "%s invalid sink pad\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}

static struct v4l2_subdev_pad_ops pcsi_pad_ops = {
	.enum_frame_size = mxc_pcsi_enum_framesizes,
	.enum_frame_interval = mxc_pcsi_enum_frame_interval,
	.get_fmt = mxc_pcsi_get_fmt,
	.set_fmt = mxc_pcsi_set_fmt,
};

static struct v4l2_subdev_core_ops pcsi_core_ops = {
	.s_power = mxc_pcsi_s_power,
};

static struct v4l2_subdev_video_ops pcsi_video_ops = {
	.g_frame_interval = mxc_pcsi_g_frame_interval,
	.s_frame_interval = mxc_pcsi_s_frame_interval,
	.s_stream	  = mxc_pcsi_s_stream,
};

static struct v4l2_subdev_ops pcsi_subdev_ops = {
	.core = &pcsi_core_ops,
	.video = &pcsi_video_ops,
	.pad = &pcsi_pad_ops,
};

static const struct media_entity_operations mxc_pcsi_sd_media_ops = {
	.link_setup = mxc_pcsi_link_setup,
};

static int mxc_parallel_csi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *mem_res;
	struct mxc_parallel_csi_dev *pcsidev;
	int ret;

	pcsidev = devm_kzalloc(dev, sizeof(*pcsidev), GFP_KERNEL);
	if (!pcsidev)
		return -ENOMEM;

	pcsidev->dev = dev;
	platform_set_drvdata(pdev, pcsidev);

	pcsidev->pdata = of_device_get_match_data(dev);
	if (!pcsidev->pdata) {
		dev_err(dev, "Can't get platform device data\n");
		return -EINVAL;
	}

	if (pcsidev->pdata->has_gasket) {
		pcsidev->gasket = syscon_regmap_lookup_by_phandle(dev->of_node,
								  "pi_gpr");
		if (IS_ERR(pcsidev->gasket)) {
			ret = PTR_ERR(pcsidev->gasket);
			dev_err(dev, "failed to get gasket: %d\n", ret);
			return ret;
		}
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pcsidev->csr_regs = devm_ioremap_resource(dev, mem_res);
	if (IS_ERR(pcsidev->csr_regs)) {
		dev_dbg(dev, "Failed to get parallel CSI CSR register\n");
		return PTR_ERR(pcsidev->csr_regs);
	}

	ret = mxc_pcsi_clk_get(pcsidev);
	if (ret < 0)
		return ret;

	ret = mxc_pcsi_attach_pd(pcsidev);
	if (ret < 0)
		return ret;

	v4l2_subdev_init(&pcsidev->sd, &pcsi_subdev_ops);

	pcsidev->mode = PI_GATE_CLOCK_MODE;

	pcsidev->sd.owner = THIS_MODULE;
	sprintf(pcsidev->sd.name, "%s", MXC_PARALLEL_CSI_SUBDEV_NAME);

	pcsidev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pcsidev->sd.entity.function = MEDIA_ENT_F_IO_V4L;

	pcsidev->sd.dev = dev;

	pcsidev->pads[MXC_PARALLEL_CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pcsidev->pads[MXC_PARALLEL_CSI_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&pcsidev->sd.entity,
				     MXC_PARALLEL_CSI_PADS_NUM,
				     pcsidev->pads);
	if (ret < 0)
		goto e_clkdis;

	pcsidev->sd.entity.ops = &mxc_pcsi_sd_media_ops;

	v4l2_set_subdevdata(&pcsidev->sd, pdev);

	pcsidev->running = 0;
	pm_runtime_enable(dev);

	dev_info(dev, "%s probe successfully\n", __func__);
	return 0;

e_clkdis:
	media_entity_cleanup(&pcsidev->sd.entity);
	return ret;
}

static int mxc_parallel_csi_remove(struct platform_device *pdev)
{
	struct mxc_parallel_csi_dev *pcsidev =
			(struct mxc_parallel_csi_dev *)platform_get_drvdata(pdev);

	mxc_pcsi_detach_pd(pcsidev);
	media_entity_cleanup(&pcsidev->sd.entity);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int parallel_csi_pm_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int parallel_csi_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static int parallel_csi_runtime_suspend(struct device *dev)
{
	struct mxc_parallel_csi_dev *pcsidev = dev_get_drvdata(dev);

	mxc_pcsi_clk_disable(pcsidev);

	return 0;
}

static int parallel_csi_runtime_resume(struct device *dev)
{
	struct mxc_parallel_csi_dev *pcsidev = dev_get_drvdata(dev);
	int ret;

	ret = mxc_pcsi_clk_enable(pcsidev);
	if (ret < 0)
		return ret;

	return 0;
}

static int imx8_pcsi_attach_pd(struct mxc_parallel_csi_dev *pcsidev)
{
	struct device *dev = pcsidev->dev;
	struct device_link *link;

	pcsidev->pd_pi = dev_pm_domain_attach_by_name(dev, "pd_pi");
	if (IS_ERR(pcsidev->pd_pi)) {
		if (PTR_ERR(pcsidev->pd_pi) != -EPROBE_DEFER) {
			dev_err(dev, "attach pd_pi domain for pi fail\n");
			return PTR_ERR(pcsidev->pd_pi);
		} else {
			return PTR_ERR(pcsidev->pd_pi);
		}
	}
	link = device_link_add(dev, pcsidev->pd_pi,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME);
	if (IS_ERR(link))
		return PTR_ERR(link);
	pcsidev->pd_pi_link = link;

	pcsidev->pd_isi = dev_pm_domain_attach_by_name(dev, "pd_isi_ch0");
	if (IS_ERR(pcsidev->pd_isi)) {
		if (PTR_ERR(pcsidev->pd_isi) != -EPROBE_DEFER) {
			dev_err(dev, "attach pd_isi_ch0 domain for pi fail\n");
			return PTR_ERR(pcsidev->pd_isi);
		} else {
			return PTR_ERR(pcsidev->pd_isi);
		}
	}
	link = device_link_add(dev, pcsidev->pd_isi,
			       DL_FLAG_STATELESS |
			       DL_FLAG_PM_RUNTIME);
	if (IS_ERR(link))
		return PTR_ERR(link);
	pcsidev->pd_isi_link = link;

	return 0;
}

static void imx8_pcsi_deattch_pd(struct mxc_parallel_csi_dev *pcsidev)
{
	device_link_del(pcsidev->pd_pi_link);
	device_link_del(pcsidev->pd_isi_link);
	dev_pm_domain_detach(pcsidev->pd_pi, true);
	dev_pm_domain_detach(pcsidev->pd_isi, true);
}

static const struct mxc_pcsi_power_domain_ops imx8_pd_ops = {
	.attach_pd	= imx8_pcsi_attach_pd,
	.deattach_pd	= imx8_pcsi_deattch_pd,
};

static const struct mxc_pcsi_plat_data imx8_pdata = {
	.version		= PI_V1,
	.if_ctrl_reg		= 0x0,
	.interface_ctrl_reg	= 0x10,
	.interface_status	= 0x20,
	.interface_ctrl_reg1	= 0x30,
	.has_gasket		= false,
	.def_hsync_pol		= 1,
	.def_vsync_pol		= 0,
	.def_pixel_clk_pol	= 0,
	.def_csi_in_data_type	= CSI_IN_DT_UYVY_BT656_8,
	.pd_ops			= &imx8_pd_ops,
};

static const struct mxc_pcsi_plat_data imx93_pdata = {
	.version		= PI_V2,
	.if_ctrl_reg		= 0x0,
	.interface_status	= 0x4,
	.interface_ctrl_reg	= 0x8,
	.interface_ctrl_reg1	= 0xc,
	.has_gasket		= true,
	.def_hsync_pol		= 0,
	.def_vsync_pol		= 0,
	.def_pixel_clk_pol	= 0,
	.def_csi_in_data_type	= CSI_IN_DT_YVYU_8,
	.pd_ops			= NULL,
};

static const struct dev_pm_ops parallel_csi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(parallel_csi_pm_suspend, parallel_csi_pm_resume)
	SET_RUNTIME_PM_OPS(parallel_csi_runtime_suspend,
			   parallel_csi_runtime_resume,
			   NULL)
};

static const struct of_device_id parallel_csi_of_match[] = {
	{	.compatible = "fsl,mxc-parallel-csi",   .data = &imx8_pdata},
	{	.compatible = "fsl,imx93-parallel-csi", .data = &imx93_pdata},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, parallel_csi_of_match);

static struct platform_driver parallel_csi_driver = {
	.driver = {
		.name = MXC_PARALLEL_CSI_DRIVER_NAME,
		.of_match_table = parallel_csi_of_match,
		.pm = &parallel_csi_pm_ops,
	},
	.probe = mxc_parallel_csi_probe,
	.remove = mxc_parallel_csi_remove,
};

module_platform_driver(parallel_csi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC PARALLEL CSI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" MXC_PARALLEL_CSI_DRIVER_NAME);
