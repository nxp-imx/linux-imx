// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <dt-bindings/clock/imx8mp-clock.h>
#include <dt-bindings/reset/imx8mp-reset.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <soc/imx/soc.h>

#include "clk.h"
#include "clk-blk-ctrl.h"

#define	IMX_AUDIO_BLK_CTRL_CLKEN0		0x0
#define	IMX_AUDIO_BLK_CTRL_CLKEN1		0x4
#define	IMX_AUDIO_BLK_CTRL_EARC			0x200
#define	IMX_AUDIO_BLK_CTRL_SAI1_MCLK_SEL	0x300
#define	IMX_AUDIO_BLK_CTRL_SAI2_MCLK_SEL	0x304
#define	IMX_AUDIO_BLK_CTRL_SAI3_MCLK_SEL	0x308
#define	IMX_AUDIO_BLK_CTRL_SAI5_MCLK_SEL	0x30C
#define	IMX_AUDIO_BLK_CTRL_SAI6_MCLK_SEL	0x310
#define	IMX_AUDIO_BLK_CTRL_SAI7_MCLK_SEL	0x314
#define	IMX_AUDIO_BLK_CTRL_PDM_CLK		0x318
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_GNRL_CTL	0x400
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL0	0x404
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL1	0x408
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_SSCG_CTL	0x40C
#define	IMX_AUDIO_BLK_CTRL_SAI_PLL_MNIT_CTL	0x410
#define	IMX_AUDIO_BLK_CTRL_IPG_LP_CTRL		0x504

#define IMX_MEDIA_BLK_CTRL_SFT_RSTN		0x0
#define IMX_MEDIA_BLK_CTRL_CLK_EN		0x4

static u32 share_count_nand;
static u32 share_count_media;
static u32 share_count_audio;

static int shared_count_pdm;

/* descending order */
static const struct imx_pll14xx_rate_table imx_blk_ctrl_sai_pll_tbl[] = {
	PLL_1443X_RATE(245760000U, 328, 4, 3, 0xae15),
	PLL_1443X_RATE(225792000U, 226, 3, 3, 0xcac1),
	PLL_1443X_RATE(122880000U, 328, 4, 4, 0xae15),
	PLL_1443X_RATE(112896000U, 226,	3, 4, 0xcac1),
	PLL_1443X_RATE(61440000U, 328, 4, 5, 0xae15),
	PLL_1443X_RATE(56448000U, 226, 3, 5, 0xcac1),
	PLL_1443X_RATE(49152000U, 393, 3, 6, 0x374c),
	PLL_1443X_RATE(45158400U, 241, 2, 6, 0xd845),
	PLL_1443X_RATE(40960000U, 109, 1, 6, 0x3a07),
};

static const struct imx_pll14xx_clk imx_blk_ctrl_sai_pll = {
	.type = PLL_1443X,
	.rate_table = imx_blk_ctrl_sai_pll_tbl,
	.rate_count = ARRAY_SIZE(imx_blk_ctrl_sai_pll_tbl),
};

static const char *imx_sai_mclk2_sels[] = {"sai1_root", "sai2_root", "sai3_root", "dummy",
					   "sai5_root", "sai6_root", "sai7_root", "sai1_mclk",
					   "sai2_mclk", "sai3_mclk", "dummy",
					   "sai5_mclk", "sai6_mclk", "sai7_mclk", "spdif1_ext_clk"};
static const char *imx_sai1_mclk1_sels[] = {"sai1_root", "sai1_mclk", };
static const char *imx_sai2_mclk1_sels[] = {"sai2_root", "sai2_mclk", };
static const char *imx_sai3_mclk1_sels[] = {"sai3_root", "sai3_mclk", };
static const char *imx_sai5_mclk1_sels[] = {"sai5_root", "sai5_mclk", };
static const char *imx_sai6_mclk1_sels[] = {"sai6_root", "sai6_mclk", };
static const char *imx_sai7_mclk1_sels[] = {"sai7_root", "sai7_mclk", };
static const char *imx_pdm_sels[] = {"pdm_root", "sai_pll_div2", "dummy", "dummy" };
static const char *imx_sai_pll_ref_sels[] = {"osc_24m", "dummy", "dummy", "dummy", };
static const char *imx_sai_pll_bypass_sels[] = {"sai_pll", "sai_pll_ref_sel", };

static const char *imx_hdmi_phy_clks_sels[] = { "hdmi_glb_24m", "dummy",};
static const char *imx_lcdif_clks_sels[] = { "dummy", "hdmi_glb_pix", };
static const char *imx_hdmi_pipe_clks_sels[] = {"dummy","hdmi_glb_pix", };

static struct imx_blk_ctrl_hw imx8mp_hdmi_blk_ctrl_hws[] = {
	/* clocks */
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_apb", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_APB_CLK, 0x40, 0, "hdmi_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_b", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_B_CLK, 0x40, 1, "hdmi_axi"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_ref_266m", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_REF266M_CLK, 0x40, 2, "hdmi_ref_266m"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_24m", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_XTAL24M_CLK, 0x40, 4, "hdmi_24m"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_32k", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_XTAL32K_CLK, 0x40, 5, "osc_32k"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_glb_pix", IMX8MP_CLK_HDMI_BLK_CTRL_GLOBAL_TX_PIX_CLK, 0x40, 7, "hdmi_phy"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_irq_steer", IMX8MP_CLK_HDMI_BLK_CTRL_IRQS_STEER_CLK, 0x40, 9, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_noc", IMX8MP_CLK_HDMI_BLK_CTRL_NOC_HDMI_CLK, 0x40, 10, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdcp_noc", IMX8MP_CLK_HDMI_BLK_CTRL_NOC_HDCP_CLK, 0x40, 11,  "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("lcdif3_apb", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_APB_CLK, 0x40, 16, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("lcdif3_b", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_B_CLK, 0x40, 17, "hdmi_glb_b"),
	IMX_BLK_CTRL_CLK_GATE("lcdif3_pdi", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_PDI_CLK, 0x40, 18, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("lcdif3_pxl", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_PIX_CLK, 0x40, 19, "hdmi_glb_pix"),
	IMX_BLK_CTRL_CLK_GATE("lcdif3_spu", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_SPU_CLK, 0x40, 20, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_fdcc_ref", IMX8MP_CLK_HDMI_BLK_CTRL_FDCC_REF_CLK, 0x50, 2, "hdmi_fdcc_tst"),
	IMX_BLK_CTRL_CLK_GATE("hrv_mwr_apb", IMX8MP_CLK_HDMI_BLK_CTRL_HRV_MWR_APB_CLK, 0x50, 3, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hrv_mwr_b", IMX8MP_CLK_HDMI_BLK_CTRL_HRV_MWR_B_CLK, 0x50, 4, "hdmi_glb_axi"),
	IMX_BLK_CTRL_CLK_GATE("hrv_mwr_cea", IMX8MP_CLK_HDMI_BLK_CTRL_HRV_MWR_CEA_CLK, 0x50, 5, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("vsfd_cea", IMX8MP_CLK_HDMI_BLK_CTRL_VSFD_CEA_CLK, 0x50, 6, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_hpi", IMX8MP_CLK_HDMI_BLK_CTRL_TX_HPI_CLK, 0x50, 13, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_apb", IMX8MP_CLK_HDMI_BLK_CTRL_TX_APB_CLK, 0x50, 14, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_cec", IMX8MP_CLK_HDMI_BLK_CTRL_TX_CEC_CLK, 0x50, 15, "hdmi_glb_32k"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_esm", IMX8MP_CLK_HDMI_BLK_CTRL_TX_ESM_CLK, 0x50, 16, "hdmi_glb_ref_266m"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_gpa", IMX8MP_CLK_HDMI_BLK_CTRL_TX_GPA_CLK, 0x50, 17, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_pix", IMX8MP_CLK_HDMI_BLK_CTRL_TX_PIXEL_CLK, 0x50, 18, "hdmi_glb_pix"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_sfr", IMX8MP_CLK_HDMI_BLK_CTRL_TX_SFR_CLK, 0x50, 19, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_skp", IMX8MP_CLK_HDMI_BLK_CTRL_TX_SKP_CLK, 0x50, 20, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_tx_prep", IMX8MP_CLK_HDMI_BLK_CTRL_TX_PREP_CLK, 0x50, 21, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_phy_apb", IMX8MP_CLK_HDMI_BLK_CTRL_TX_PHY_APB_CLK, 0x50, 22, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_phy_int", IMX8MP_CLK_HDMI_BLK_CTRL_TX_PHY_INT_CLK, 0x50, 24, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_sec_mem", IMX8MP_CLK_HDMI_BLK_CTRL_TX_SEC_MEM_CLK, 0x50, 25, "hdmi_glb_ref_266m"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_trng_skp", IMX8MP_CLK_HDMI_BLK_CTRL_TX_TRNG_SKP_CLK, 0x50, 27, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_vid_pix",  IMX8MP_CLK_HDMI_BLK_CTRL_TX_VID_LINK_PIX_CLK, 0x50, 28, "hdmi_glb_pix"),
	IMX_BLK_CTRL_CLK_GATE("hdmi_trng_apb", IMX8MP_CLK_HDMI_BLK_CTRL_TX_TRNG_APB_CLK, 0x50, 30, "hdmi_glb_apb"),
	IMX_BLK_CTRL_CLK_MUX("hdmi_phy_sel", IMX8MP_CLK_HDMI_BLK_CTRL_HTXPHY_CLK_SEL, 0x50, 10, 1, imx_hdmi_phy_clks_sels),
	IMX_BLK_CTRL_CLK_MUX("lcdif_clk_sel", IMX8MP_CLK_HDMI_BLK_CTRL_LCDIF_CLK_SEL, 0x50, 11, 1, imx_lcdif_clks_sels),
	IMX_BLK_CTRL_CLK_MUX("hdmi_pipe_sel", IMX8MP_CLK_HDMI_BLK_CTRL_TX_PIPE_CLK_SEL, 0x50, 12, 1, imx_hdmi_pipe_clks_sels),

	/* resets */
	IMX_BLK_CTRL_RESET_MASK(IMX8MP_HDMI_BLK_CTRL_HDMI_TX_RESET, 0x20, 6, 0x33),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_PHY_RESET, 0x20, 12),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_PAI_RESET, 0x20, 18),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_PVI_RESET, 0x20, 22),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_TRNG_RESET, 0x20, 20),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_IRQ_STEER_RESET, 0x20, 16),
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_HDCP_RESET, 0x20, 13),
	IMX_BLK_CTRL_RESET_MASK(IMX8MP_HDMI_BLK_CTRL_LCDIF_RESET, 0x20, 4, 0x3),
};

static struct imx_blk_ctrl_hw imx8mp_media_blk_ctrl_hws[] = {
	/* clocks */
	IMX_BLK_CTRL_CLK_GATE("mipi_dsi_pclk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_DSI_PCLK, 0x4, 0, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("mipi_dsi_clkref", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_DSI_CLKREF, 0x4, 1, "media_mipi_phy1_ref"),
	IMX_BLK_CTRL_CLK_GATE("mipi_csi_pclk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_CSI_PCLK, 0x4, 2, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("mipi_csi_aclk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_CSI_ACLK, 0x4, 3, "media_cam1_pix_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("lcdif_pixel_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF_PIXEL, 0x4, 4, "media_disp1_pix_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("lcdif_apb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF_APB, 0x4, 5, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isi_proc_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISI_PROC, 0x4, 6, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isi_apb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISI_APB, 0x4, 7, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("mipi_csi2_pclk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_CSI2_PCLK, 0x4, 9, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("mipi_csi2_aclk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_CSI2_ACLK, 0x4, 10, "media_cam2_pix_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("lcdif2_pixel_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF2_PIXEL, 0x4, 11, "media_disp2_pix_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("lcdif2_apb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF2_APB, 0x4, 12, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp_cor_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP_COR, 0x4, 16, "media_isp_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP_AXI, 0x4, 17, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp_ahb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP_AHB, 0x4, 18, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("dwe_cor_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_DWE_COR, 0x4, 19, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("dwe_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_DWE_AXI, 0x4, 20, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("dwe_ahb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_DWE_AHB, 0x4, 21, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("mipi_dsi2_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_MIPI_DSI2, 0x4, 22, "media_mipi_phy1_ref"),
	IMX_BLK_CTRL_CLK_GATE("lcdif_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF_AXI, 0x4, 23, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("lcdif2_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_LCDIF2_AXI, 0x4, 24, "media_axi_root_clk"),

	/* resets */
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_DSI_PCLK, 0, 0),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_DSI_CLKREF, 0, 1),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_CSI_PCLK, 0, 2),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_CSI_ACLK, 0, 3),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF_PIXEL, 0, 4),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF_APB, 0, 5),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISI_PROC, 0, 6),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISI_APB, 0, 7),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_BUS_BLK, 0, 8),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_CSI2_PCLK, 0, 9),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_CSI2_ACLK, 0, 10),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF2_PIXEL, 0, 11),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF2_APB, 0, 12),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP1_COR, 0, 13),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP1_AXI, 0, 14),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP1_AHB, 0, 15),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP0_COR, 0, 16),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP0_AXI, 0, 17),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_ISP0_AHB, 0, 18),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_DWE_COR, 0, 19),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_DWE_AXI, 0, 20),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_DWE_AHB, 0, 21),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_MIPI_DSI2, 0, 22),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF_AXI, 0, 23),
	IMX_BLK_CTRL_RESET(IMX8MP_MEDIA_BLK_CTRL_RESET_LCDIF2_AXI, 0, 24)
};

static struct imx_blk_ctrl_hw imx8mp_audio_blk_ctrl_hws[] = {
	/* clocks */
	IMX_BLK_CTRL_CLK_MUX("sai_pll_ref_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI_PLL_REF_SEL, 0x400, 0, 2, imx_sai_pll_ref_sels),
	IMX_BLK_CTRL_CLK_PLL14XX("sai_pll", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI_PLL, 0x400, "sai_pll_ref_sel", &imx_blk_ctrl_sai_pll),
	IMX_BLK_CTRL_CLK_MUX_FLAGS("sai_pll_bypass", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI_PLL_BYPASS, 0x400, 4, 1, imx_sai_pll_bypass_sels, CLK_SET_RATE_PARENT),
	IMX_BLK_CTRL_CLK_GATE("sai_pll_out", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI_PLL_OUT, 0x400, 13, "sai_pll_bypass"),
	IMX_BLK_CTRL_CLK_MUX_FLAGS("sai1_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_MCLK1_SEL, 0x300, 0, 1, imx_sai1_mclk1_sels, CLK_SET_RATE_PARENT),
	IMX_BLK_CTRL_CLK_MUX("sai1_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_MCLK2_SEL, 0x300, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX_FLAGS("sai2_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_MCLK1_SEL, 0x304, 0, 1, imx_sai2_mclk1_sels, CLK_SET_RATE_PARENT),
	IMX_BLK_CTRL_CLK_MUX("sai2_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_MCLK2_SEL, 0x304, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX_FLAGS("sai3_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_MCLK1_SEL, 0x308, 0, 1, imx_sai3_mclk1_sels, CLK_SET_RATE_PARENT),
	IMX_BLK_CTRL_CLK_MUX("sai3_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_MCLK2_SEL, 0x308, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX("sai5_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_MCLK1_SEL, 0x30C, 0, 1, imx_sai5_mclk1_sels),
	IMX_BLK_CTRL_CLK_MUX("sai5_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_MCLK2_SEL, 0x30C, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX("sai6_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_MCLK1_SEL, 0x310, 0, 1, imx_sai6_mclk1_sels),
	IMX_BLK_CTRL_CLK_MUX("sai6_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_MCLK2_SEL, 0x310, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX("sai7_mclk1_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_MCLK1_SEL, 0x314, 0, 1, imx_sai7_mclk1_sels),
	IMX_BLK_CTRL_CLK_MUX("sai7_mclk2_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_MCLK2_SEL, 0x314, 1, 4, imx_sai_mclk2_sels),
	IMX_BLK_CTRL_CLK_MUX_FLAGS("pdm_sel", IMX8MP_CLK_AUDIO_BLK_CTRL_PDM_SEL, 0x318, 0, 2, imx_pdm_sels, CLK_SET_RATE_PARENT),
	IMX_BLK_CTRL_CLK_GATE("sai1_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_IPG, 0, 0, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai1_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_MCLK1, 0, 1, "sai1_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai1_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_MCLK2, 0, 2, "sai1_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai1_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI1_MCLK3, 0, 3, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("sai2_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_IPG, 0, 4, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai2_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_MCLK1, 0, 5, "sai2_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai2_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_MCLK2, 0, 6, "sai2_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai2_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI2_MCLK3, 0, 7, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("sai3_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_IPG, 0, 8, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai3_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_MCLK1, 0, 9, "sai3_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai3_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_MCLK2, 0, 10, "sai3_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai3_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI3_MCLK3, 0, 11, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("sai5_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_IPG, 0, 12, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai5_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_MCLK1, 0, 13, "sai5_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai5_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_MCLK2, 0, 14, "sai5_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai5_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI5_MCLK3, 0, 15, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("sai6_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_IPG, 0, 16, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai6_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_MCLK1, 0, 17, "sai6_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai6_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_MCLK2, 0, 18, "sai6_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai6_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI6_MCLK3, 0, 19, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("sai7_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_IPG, 0, 20, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("sai7_mclk1_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_MCLK1, 0, 21, "sai7_mclk1_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai7_mclk2_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_MCLK2, 0, 22, "sai7_mclk2_sel"),
	IMX_BLK_CTRL_CLK_GATE("sai7_mclk3_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SAI7_MCLK3, 0, 23, "sai_pll_out"),
	IMX_BLK_CTRL_CLK_GATE("asrc_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_ASRC_IPG, 0, 24, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_SHARED_GATE("pdm_ipg_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_PDM_IPG, 0, 25, "audio_ahb_root", &shared_count_pdm),
	IMX_BLK_CTRL_CLK_SHARED_GATE("pdm_root_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_PDM_ROOT, 0, 25, "pdm_sel", &shared_count_pdm),
	IMX_BLK_CTRL_CLK_GATE("sdma3_root_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SDMA3_ROOT, 0, 27, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("spba2_root_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_SPBA2_ROOT, 0, 28, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("dsp_root_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_DSP_ROOT, 0, 29, "audio_axi_root"),
	IMX_BLK_CTRL_CLK_GATE("dsp_dbg_clk",    IMX8MP_CLK_AUDIO_BLK_CTRL_DSPDBG_ROOT, 0, 30, "audio_axi_root"),
	IMX_BLK_CTRL_CLK_GATE("earc_ipg_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_EARC_IPG, 0, 31, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("ocram_a_ipg_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_OCRAMA_IPG, 4, 0, "audio_axi_root"),
	IMX_BLK_CTRL_CLK_GATE("aud2htx_ipg_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_AUD2HTX_IPG, 4, 1, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("edma_root_clk",   IMX8MP_CLK_AUDIO_BLK_CTRL_EDMA_ROOT, 4, 2, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("aud_pll_clk",  IMX8MP_CLK_AUDIO_BLK_CTRL_AUDPLL_ROOT, 4, 3, "osc_24m"),
	IMX_BLK_CTRL_CLK_GATE("mu2_root_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_MU2_ROOT, 4, 4, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("mu3_root_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_MU3_ROOT, 4, 5, "audio_ahb_root"),
	IMX_BLK_CTRL_CLK_GATE("earc_phy_clk", IMX8MP_CLK_AUDIO_BLK_CTRL_EARC_PHY, 4, 6, "sai_pll_out"),

	/* resets */
	IMX_BLK_CTRL_RESET(IMX8MP_AUDIO_BLK_CTRL_EARC_RESET, 0x200, 0),
	IMX_BLK_CTRL_RESET(IMX8MP_AUDIO_BLK_CTRL_EARC_PHY_RESET, 0x200, 1),
};

const struct imx_blk_ctrl_dev_data imx8mp_hdmi_blk_ctrl_dev_data = {
	.hws = imx8mp_hdmi_blk_ctrl_hws,
	.hws_num = ARRAY_SIZE(imx8mp_hdmi_blk_ctrl_hws),
	.clocks_max = IMX8MP_CLK_HDMI_BLK_CTRL_END,
	.resets_max = IMX8MP_HDMI_BLK_CTRL_RESET_NUM,
	.pm_runtime_saved_regs_num = 0
};
EXPORT_SYMBOL_GPL(imx8mp_hdmi_blk_ctrl_dev_data);

const struct imx_blk_ctrl_dev_data imx8mp_media_blk_ctrl_dev_data = {
	.hws = imx8mp_media_blk_ctrl_hws,
	.hws_num = ARRAY_SIZE(imx8mp_media_blk_ctrl_hws),
	.clocks_max = IMX8MP_CLK_MEDIA_BLK_CTRL_END,
	.resets_max = IMX8MP_MEDIA_BLK_CTRL_RESET_NUM,
	.pm_runtime_saved_regs_num = 2,
	.pm_runtime_saved_regs = {
		IMX_MEDIA_BLK_CTRL_SFT_RSTN,
		IMX_MEDIA_BLK_CTRL_CLK_EN,
	},
};
EXPORT_SYMBOL_GPL(imx8mp_media_blk_ctrl_dev_data);

const struct imx_blk_ctrl_dev_data imx8mp_audio_blk_ctrl_dev_data = {
	.hws = imx8mp_audio_blk_ctrl_hws,
	.hws_num = ARRAY_SIZE(imx8mp_audio_blk_ctrl_hws),
	.clocks_max = IMX8MP_CLK_AUDIO_BLK_CTRL_END,
	.resets_max = IMX8MP_AUDIO_BLK_CTRL_RESET_NUM,
	.pm_runtime_saved_regs_num = 16,
	.pm_runtime_saved_regs = {
		IMX_AUDIO_BLK_CTRL_CLKEN0,
		IMX_AUDIO_BLK_CTRL_CLKEN1,
		IMX_AUDIO_BLK_CTRL_EARC,
		IMX_AUDIO_BLK_CTRL_SAI1_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI2_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI3_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI5_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI6_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_SAI7_MCLK_SEL,
		IMX_AUDIO_BLK_CTRL_PDM_CLK,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_GNRL_CTL,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL0,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_FDIVL_CTL1,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_SSCG_CTL,
		IMX_AUDIO_BLK_CTRL_SAI_PLL_MNIT_CTL,
		IMX_AUDIO_BLK_CTRL_IPG_LP_CTRL
	},
};
EXPORT_SYMBOL_GPL(imx8mp_audio_blk_ctrl_dev_data);

static const char * const pll_ref_sels[] = { "osc_24m", "dummy", "dummy", "dummy", };
static const char * const audio_pll1_bypass_sels[] = {"audio_pll1", "audio_pll1_ref_sel", };
static const char * const audio_pll2_bypass_sels[] = {"audio_pll2", "audio_pll2_ref_sel", };
static const char * const video_pll1_bypass_sels[] = {"video_pll1", "video_pll1_ref_sel", };
static const char * const dram_pll_bypass_sels[] = {"dram_pll", "dram_pll_ref_sel", };
static const char * const gpu_pll_bypass_sels[] = {"gpu_pll", "gpu_pll_ref_sel", };
static const char * const vpu_pll_bypass_sels[] = {"vpu_pll", "vpu_pll_ref_sel", };
static const char * const arm_pll_bypass_sels[] = {"arm_pll", "arm_pll_ref_sel", };
static const char * const sys_pll1_bypass_sels[] = {"sys_pll1", "sys_pll1_ref_sel", };
static const char * const sys_pll2_bypass_sels[] = {"sys_pll2", "sys_pll2_ref_sel", };
static const char * const sys_pll3_bypass_sels[] = {"sys_pll3", "sys_pll3_ref_sel", };

static const char * const imx8mp_a53_sels[] = {"osc_24m", "arm_pll_out", "sys_pll2_500m",
					       "sys_pll2_1000m", "sys_pll1_800m", "sys_pll1_400m",
					       "audio_pll1_out", "sys_pll3_out", };

static const char * const imx8mp_a53_core_sels[] = {"arm_a53_div", "arm_pll_out", };

static const char * const imx8mp_m7_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_250m",
					      "vpu_pll_out", "sys_pll1_800m", "audio_pll1_out",
					      "video_pll1_out", "sys_pll3_out", };

static const char * const imx8mp_ml_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m",
					      "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
					      "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_gpu3d_core_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m",
						      "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						      "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_gpu3d_shader_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m",
							"sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
							"video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_gpu2d_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m",
						 "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						 "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_audio_axi_sels[] = {"osc_24m", "gpu_pll_out", "sys_pll1_800m",
						     "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						     "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_hsio_axi_sels[] = {"osc_24m", "sys_pll2_500m", "sys_pll1_800m",
						    "sys_pll2_100m", "sys_pll2_200m", "clk_ext2",
						    "clk_ext4", "audio_pll2_out", };

static const char * const imx8mp_media_isp_sels[] = {"osc_24m", "sys_pll2_1000m", "sys_pll1_800m",
						     "sys_pll3_out", "sys_pll1_400m", "audio_pll2_out",
						     "clk_ext1", "sys_pll2_500m", };

static const char * const imx8mp_main_axi_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll1_800m",
						    "sys_pll2_250m", "sys_pll2_1000m", "audio_pll1_out",
						    "video_pll1_out", "sys_pll1_100m",};

static const char * const imx8mp_enet_axi_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll1_800m",
						    "sys_pll2_250m", "sys_pll2_200m", "audio_pll1_out",
						    "video_pll1_out", "sys_pll3_out", };

static const char * const imx8mp_nand_usdhc_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll1_800m",
						      "sys_pll2_200m", "sys_pll1_133m", "sys_pll3_out",
						      "sys_pll2_250m", "audio_pll1_out", };

static const char * const imx8mp_vpu_bus_sels[] = {"osc_24m", "sys_pll1_800m", "vpu_pll_out",
						   "audio_pll2_out", "sys_pll3_out", "sys_pll2_1000m",
						   "sys_pll2_200m", "sys_pll1_100m", };

static const char * const imx8mp_media_axi_sels[] = {"osc_24m", "sys_pll2_1000m", "sys_pll1_800m",
						     "sys_pll3_out", "sys_pll1_40m", "audio_pll2_out",
						     "clk_ext1", "sys_pll2_500m", };

static const char * const imx8mp_media_apb_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll1_800m",
						     "sys_pll3_out", "sys_pll1_40m", "audio_pll2_out",
						     "clk_ext1", "sys_pll1_133m", };

static const char * const imx8mp_gpu_axi_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out",
						   "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						   "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_gpu_ahb_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out",
						   "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						   "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_noc_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll3_out",
					       "sys_pll2_1000m", "sys_pll2_500m", "audio_pll1_out",
					       "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_noc_io_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll3_out",
						  "sys_pll2_1000m", "sys_pll2_500m", "audio_pll1_out",
						  "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_ml_axi_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out",
						  "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						  "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_ml_ahb_sels[] = {"osc_24m", "sys_pll1_800m", "gpu_pll_out",
						  "sys_pll3_out", "sys_pll2_1000m", "audio_pll1_out",
						  "video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_ahb_sels[] = {"osc_24m", "sys_pll1_133m", "sys_pll1_800m",
					       "sys_pll1_400m", "sys_pll2_125m", "sys_pll3_out",
					       "audio_pll1_out", "video_pll1_out", };

static const char * const imx8mp_audio_ahb_sels[] = {"osc_24m", "sys_pll2_500m", "sys_pll1_800m",
						     "sys_pll2_1000m", "sys_pll2_166m", "sys_pll3_out",
						     "audio_pll1_out", "video_pll1_out", };

static const char * const imx8mp_mipi_dsi_esc_rx_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_80m",
							   "sys_pll1_800m", "sys_pll2_1000m",
							   "sys_pll3_out", "clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_media_disp2_pix_sels[] = {"osc_24m", "video_pll1_out", "audio_pll2_out",
							   "audio_pll1_out", "sys_pll1_800m", "sys_pll2_1000m",
							   "sys_pll3_out", "clk_ext4", };

static const char * const imx8mp_dram_alt_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll1_100m",
						    "sys_pll2_500m", "sys_pll2_1000m", "sys_pll3_out",
						    "audio_pll1_out", "sys_pll1_266m", };

static const char * const imx8mp_dram_apb_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						    "sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						    "sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_vpu_g1_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m",
						  "sys_pll2_1000m", "sys_pll1_100m", "sys_pll2_125m",
						  "sys_pll3_out", "audio_pll1_out", };

static const char * const imx8mp_vpu_g2_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m",
						  "sys_pll2_1000m", "sys_pll1_100m", "sys_pll2_125m",
						  "sys_pll3_out", "audio_pll1_out", };

static const char * const imx8mp_can1_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						"sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						"sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_can2_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						"sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						"sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_pcie_aux_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_50m",
						    "sys_pll3_out", "sys_pll2_100m", "sys_pll1_80m",
						    "sys_pll1_160m", "sys_pll1_200m", };

static const char * const imx8mp_i2c5_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_i2c6_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_sai1_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext1", "clk_ext2", };

static const char * const imx8mp_sai2_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext2", "clk_ext3", };

static const char * const imx8mp_sai3_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext3", "clk_ext4", };

static const char * const imx8mp_sai4_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext1", "clk_ext2", };

static const char * const imx8mp_sai5_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext2", "clk_ext3", };

static const char * const imx8mp_sai6_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext3", "clk_ext4", };

static const char * const imx8mp_enet_qos_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll2_50m",
						    "sys_pll2_100m", "sys_pll1_160m", "audio_pll1_out",
						    "video_pll1_out", "clk_ext4", };

static const char * const imx8mp_enet_qos_timer_sels[] = {"osc_24m", "sys_pll2_100m", "audio_pll1_out",
							  "clk_ext1", "clk_ext2", "clk_ext3",
							  "clk_ext4", "video_pll1_out", };

static const char * const imx8mp_enet_ref_sels[] = {"osc_24m", "sys_pll2_125m", "sys_pll2_50m",
						    "sys_pll2_100m", "sys_pll1_160m", "audio_pll1_out",
						    "video_pll1_out", "clk_ext4", };

static const char * const imx8mp_enet_timer_sels[] = {"osc_24m", "sys_pll2_100m", "audio_pll1_out",
						      "clk_ext1", "clk_ext2", "clk_ext3",
						      "clk_ext4", "video_pll1_out", };

static const char * const imx8mp_enet_phy_ref_sels[] = {"osc_24m", "sys_pll2_50m", "sys_pll2_125m",
							"sys_pll2_200m", "sys_pll2_500m", "audio_pll1_out",
							"video_pll1_out", "audio_pll2_out", };

static const char * const imx8mp_nand_sels[] = {"osc_24m", "sys_pll2_500m", "audio_pll1_out",
						"sys_pll1_400m", "audio_pll2_out", "sys_pll3_out",
						"sys_pll2_250m", "video_pll1_out", };

static const char * const imx8mp_qspi_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll2_333m",
						"sys_pll2_500m", "audio_pll2_out", "sys_pll1_266m",
						"sys_pll3_out", "sys_pll1_100m", };

static const char * const imx8mp_usdhc1_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m",
						  "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
						  "audio_pll2_out", "sys_pll1_100m", };

static const char * const imx8mp_usdhc2_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m",
						  "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
						  "audio_pll2_out", "sys_pll1_100m", };

static const char * const imx8mp_i2c1_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_i2c2_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_i2c3_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_i2c4_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						"sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						"audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_uart1_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m",
						 "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
						 "clk_ext4", "audio_pll2_out", };

static const char * const imx8mp_uart2_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m",
						 "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
						 "clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_uart3_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m",
						 "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
						 "clk_ext4", "audio_pll2_out", };

static const char * const imx8mp_uart4_sels[] = {"osc_24m", "sys_pll1_80m", "sys_pll2_200m",
						 "sys_pll2_100m", "sys_pll3_out", "clk_ext2",
						 "clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_usb_core_ref_sels[] = {"osc_24m", "sys_pll1_100m", "sys_pll1_40m",
							"sys_pll2_100m", "sys_pll2_200m", "clk_ext2",
							"clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_usb_phy_ref_sels[] = {"osc_24m", "sys_pll1_100m", "sys_pll1_40m",
						       "sys_pll2_100m", "sys_pll2_200m", "clk_ext2",
						       "clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_gic_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
					       "sys_pll2_100m", "sys_pll1_800m",
					       "sys_pll2_500m", "clk_ext4", "audio_pll2_out" };

static const char * const imx8mp_ecspi1_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						  "sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						  "sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_ecspi2_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						  "sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						  "sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_pwm1_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m",
						"sys_pll1_40m", "sys_pll3_out", "clk_ext1",
						"sys_pll1_80m", "video_pll1_out", };

static const char * const imx8mp_pwm2_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m",
						"sys_pll1_40m", "sys_pll3_out", "clk_ext1",
						"sys_pll1_80m", "video_pll1_out", };

static const char * const imx8mp_pwm3_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m",
						"sys_pll1_40m", "sys_pll3_out", "clk_ext2",
						"sys_pll1_80m", "video_pll1_out", };

static const char * const imx8mp_pwm4_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_160m",
						"sys_pll1_40m", "sys_pll3_out", "clk_ext2",
						"sys_pll1_80m", "video_pll1_out", };

static const char * const imx8mp_gpt1_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext1" };

static const char * const imx8mp_gpt2_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext2" };

static const char * const imx8mp_gpt3_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext3" };

static const char * const imx8mp_gpt4_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext1" };

static const char * const imx8mp_gpt5_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext2" };

static const char * const imx8mp_gpt6_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_400m",
						"sys_pll1_40m", "video_pll1_out", "sys_pll1_80m",
						"audio_pll1_out", "clk_ext3" };

static const char * const imx8mp_wdog_sels[] = {"osc_24m", "sys_pll1_133m", "sys_pll1_160m",
						"vpu_pll_out", "sys_pll2_125m", "sys_pll3_out",
						"sys_pll1_80m", "sys_pll2_166m" };

static const char * const imx8mp_wrclk_sels[] = {"osc_24m", "sys_pll1_40m", "vpu_pll_out",
						 "sys_pll3_out", "sys_pll2_200m", "sys_pll1_266m",
						 "sys_pll2_500m", "sys_pll1_100m" };

static const char * const imx8mp_ipp_do_clko1_sels[] = {"osc_24m", "sys_pll1_800m", "sys_pll1_133m",
							"sys_pll1_200m", "audio_pll2_out", "sys_pll2_500m",
							"vpu_pll_out", "sys_pll1_80m" };

static const char * const imx8mp_ipp_do_clko2_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_400m",
							"sys_pll1_166m", "sys_pll3_out", "audio_pll1_out",
							"video_pll1_out", "osc_32k" };

static const char * const imx8mp_hdmi_fdcc_tst_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m",
							 "sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out",
							 "audio_pll2_out", "video_pll1_out", };

static const char * const imx8mp_hdmi_24m_sels[] = {"osc_24m", "sys_pll1_160m", "sys_pll2_50m",
						    "sys_pll3_out", "audio_pll1_out", "video_pll1_out",
						    "audio_pll2_out", "sys_pll1_133m", };

static const char * const imx8mp_hdmi_ref_266m_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll3_out",
							 "sys_pll2_333m", "sys_pll1_266m", "sys_pll2_200m",
							 "audio_pll1_out", "video_pll1_out", };

static const char * const imx8mp_usdhc3_sels[] = {"osc_24m", "sys_pll1_400m", "sys_pll1_800m",
						  "sys_pll2_500m", "sys_pll3_out", "sys_pll1_266m",
						  "audio_pll2_out", "sys_pll1_100m", };

static const char * const imx8mp_media_cam1_pix_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m",
							  "sys_pll1_800m", "sys_pll2_1000m",
							  "sys_pll3_out", "audio_pll2_out",
							  "video_pll1_out", };

static const char * const imx8mp_media_mipi_phy1_ref_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll2_100m",
							       "sys_pll1_800m", "sys_pll2_1000m",
							       "clk_ext2", "audio_pll2_out",
							       "video_pll1_out", };

static const char * const imx8mp_media_disp1_pix_sels[] = {"osc_24m", "video_pll1_out", "audio_pll2_out",
							   "audio_pll1_out", "sys_pll1_800m",
							   "sys_pll2_1000m", "sys_pll3_out", "clk_ext4", };

static const char * const imx8mp_media_cam2_pix_sels[] = {"osc_24m", "sys_pll1_266m", "sys_pll2_250m",
							  "sys_pll1_800m", "sys_pll2_1000m",
							  "sys_pll3_out", "audio_pll2_out",
							  "video_pll1_out", };

static const char * const imx8mp_media_ldb_sels[] = {"osc_24m", "sys_pll2_333m", "sys_pll2_100m",
						     "sys_pll1_800m", "sys_pll2_1000m",
						     "clk_ext2", "audio_pll2_out",
						     "video_pll1_out", };

static const char * const imx8mp_memrepair_sels[] = {"osc_24m", "sys_pll2_100m", "sys_pll1_80m",
							"sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out",
							"clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_media_mipi_test_byte_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll2_50m",
								"sys_pll3_out", "sys_pll2_100m",
								"sys_pll1_80m", "sys_pll1_160m",
								"sys_pll1_200m", };

static const char * const imx8mp_ecspi3_sels[] = {"osc_24m", "sys_pll2_200m", "sys_pll1_40m",
						  "sys_pll1_160m", "sys_pll1_800m", "sys_pll3_out",
						  "sys_pll2_250m", "audio_pll2_out", };

static const char * const imx8mp_pdm_sels[] = {"osc_24m", "sys_pll2_100m", "audio_pll1_out",
					       "sys_pll1_800m", "sys_pll2_1000m", "sys_pll3_out",
					       "clk_ext3", "audio_pll2_out", };

static const char * const imx8mp_vpu_vc8000e_sels[] = {"osc_24m", "vpu_pll_out", "sys_pll1_800m",
						       "sys_pll2_1000m", "audio_pll2_out", "sys_pll2_125m",
						       "sys_pll3_out", "audio_pll1_out", };

static const char * const imx8mp_sai7_sels[] = {"osc_24m", "audio_pll1_out", "audio_pll2_out",
						"video_pll1_out", "sys_pll1_133m", "osc_hdmi",
						"clk_ext3", "clk_ext4", };

static const char * const imx8mp_dram_core_sels[] = {"dram_pll_out", "dram_alt_root", };

static struct clk_hw **hws;
static struct clk_hw_onecell_data *clk_hw_data;

static int imx_clk_init_on(struct device_node *np,
				  struct clk_hw * const clks[])
{
	u32 *array;
	int i, ret, elems;

	elems = of_property_count_u32_elems(np, "init-on-array");
	if (elems < 0)
		return elems;
	array = kcalloc(elems, sizeof(elems), GFP_KERNEL);
	if (!array)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "init-on-array", array, elems);
	if (ret)
		return ret;

	for (i = 0; i < elems; i++) {
		ret = clk_prepare_enable(clks[array[i]]->clk);
		if (ret)
			pr_err("clk_prepare_enable failed %d\n", array[i]);
	}

	kfree(array);

	return 0;
}

static int imx8mp_clocks_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	void __iomem *anatop_base, *ccm_base;

	check_m4_enabled();

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mp-anatop");
	anatop_base = of_iomap(np, 0);
	of_node_put(np);
	if (WARN_ON(!anatop_base))
		return -ENOMEM;

	np = dev->of_node;
	ccm_base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(ccm_base))) {
		iounmap(anatop_base);
		return PTR_ERR(ccm_base);
	}

	clk_hw_data = kzalloc(struct_size(clk_hw_data, hws, IMX8MP_CLK_END), GFP_KERNEL);
	if (WARN_ON(!clk_hw_data)) {
		iounmap(anatop_base);
		return -ENOMEM;
	}

	clk_hw_data->num = IMX8MP_CLK_END;
	hws = clk_hw_data->hws;

	hws[IMX8MP_CLK_DUMMY] = imx_clk_hw_fixed("dummy", 0);
	hws[IMX8MP_CLK_24M] = imx_obtain_fixed_clk_hw(np, "osc_24m");
	hws[IMX8MP_CLK_32K] = imx_obtain_fixed_clk_hw(np, "osc_32k");
	hws[IMX8MP_CLK_EXT1] = imx_obtain_fixed_clk_hw(np, "clk_ext1");
	hws[IMX8MP_CLK_EXT2] = imx_obtain_fixed_clk_hw(np, "clk_ext2");
	hws[IMX8MP_CLK_EXT3] = imx_obtain_fixed_clk_hw(np, "clk_ext3");
	hws[IMX8MP_CLK_EXT4] = imx_obtain_fixed_clk_hw(np, "clk_ext4");

	hws[IMX8MP_AUDIO_PLL1_REF_SEL] = imx_clk_hw_mux("audio_pll1_ref_sel", anatop_base + 0x0, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_AUDIO_PLL2_REF_SEL] = imx_clk_hw_mux("audio_pll2_ref_sel", anatop_base + 0x14, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_VIDEO_PLL1_REF_SEL] = imx_clk_hw_mux("video_pll1_ref_sel", anatop_base + 0x28, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_DRAM_PLL_REF_SEL] = imx_clk_hw_mux("dram_pll_ref_sel", anatop_base + 0x50, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_GPU_PLL_REF_SEL] = imx_clk_hw_mux("gpu_pll_ref_sel", anatop_base + 0x64, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_VPU_PLL_REF_SEL] = imx_clk_hw_mux("vpu_pll_ref_sel", anatop_base + 0x74, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_ARM_PLL_REF_SEL] = imx_clk_hw_mux("arm_pll_ref_sel", anatop_base + 0x84, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_SYS_PLL1_REF_SEL] = imx_clk_hw_mux("sys_pll1_ref_sel", anatop_base + 0x94, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_SYS_PLL2_REF_SEL] = imx_clk_hw_mux("sys_pll2_ref_sel", anatop_base + 0x104, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));
	hws[IMX8MP_SYS_PLL3_REF_SEL] = imx_clk_hw_mux("sys_pll3_ref_sel", anatop_base + 0x114, 0, 2, pll_ref_sels, ARRAY_SIZE(pll_ref_sels));

	hws[IMX8MP_AUDIO_PLL1] = imx_clk_hw_pll14xx("audio_pll1", "audio_pll1_ref_sel", anatop_base, &imx_1443x_pll);
	hws[IMX8MP_AUDIO_PLL2] = imx_clk_hw_pll14xx("audio_pll2", "audio_pll2_ref_sel", anatop_base + 0x14, &imx_1443x_pll);
	hws[IMX8MP_VIDEO_PLL1] = imx_clk_hw_pll14xx("video_pll1", "video_pll1_ref_sel", anatop_base + 0x28, &imx_1443x_pll);
	hws[IMX8MP_DRAM_PLL] = imx_clk_hw_pll14xx("dram_pll", "dram_pll_ref_sel", anatop_base + 0x50, &imx_1443x_dram_pll);
	hws[IMX8MP_GPU_PLL] = imx_clk_hw_pll14xx("gpu_pll", "gpu_pll_ref_sel", anatop_base + 0x64, &imx_1416x_pll);
	hws[IMX8MP_VPU_PLL] = imx_clk_hw_pll14xx("vpu_pll", "vpu_pll_ref_sel", anatop_base + 0x74, &imx_1416x_pll);
	hws[IMX8MP_ARM_PLL] = imx_clk_hw_pll14xx("arm_pll", "arm_pll_ref_sel", anatop_base + 0x84, &imx_1416x_pll);
	hws[IMX8MP_SYS_PLL1] = imx_clk_hw_pll14xx("sys_pll1", "sys_pll1_ref_sel", anatop_base + 0x94, &imx_1416x_pll);
	hws[IMX8MP_SYS_PLL2] = imx_clk_hw_pll14xx("sys_pll2", "sys_pll2_ref_sel", anatop_base + 0x104, &imx_1416x_pll);
	hws[IMX8MP_SYS_PLL3] = imx_clk_hw_pll14xx("sys_pll3", "sys_pll3_ref_sel", anatop_base + 0x114, &imx_1416x_pll);

	hws[IMX8MP_AUDIO_PLL1_BYPASS] = imx_clk_hw_mux_flags("audio_pll1_bypass", anatop_base, 16, 1, audio_pll1_bypass_sels, ARRAY_SIZE(audio_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_AUDIO_PLL2_BYPASS] = imx_clk_hw_mux_flags("audio_pll2_bypass", anatop_base + 0x14, 16, 1, audio_pll2_bypass_sels, ARRAY_SIZE(audio_pll2_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_VIDEO_PLL1_BYPASS] = imx_clk_hw_mux_flags("video_pll1_bypass", anatop_base + 0x28, 16, 1, video_pll1_bypass_sels, ARRAY_SIZE(video_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_DRAM_PLL_BYPASS] = imx_clk_hw_mux_flags("dram_pll_bypass", anatop_base + 0x50, 16, 1, dram_pll_bypass_sels, ARRAY_SIZE(dram_pll_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_GPU_PLL_BYPASS] = imx_clk_hw_mux_flags("gpu_pll_bypass", anatop_base + 0x64, 28, 1, gpu_pll_bypass_sels, ARRAY_SIZE(gpu_pll_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_VPU_PLL_BYPASS] = imx_clk_hw_mux_flags("vpu_pll_bypass", anatop_base + 0x74, 28, 1, vpu_pll_bypass_sels, ARRAY_SIZE(vpu_pll_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_ARM_PLL_BYPASS] = imx_clk_hw_mux_flags("arm_pll_bypass", anatop_base + 0x84, 28, 1, arm_pll_bypass_sels, ARRAY_SIZE(arm_pll_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_SYS_PLL1_BYPASS] = imx_clk_hw_mux_flags("sys_pll1_bypass", anatop_base + 0x94, 28, 1, sys_pll1_bypass_sels, ARRAY_SIZE(sys_pll1_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_SYS_PLL2_BYPASS] = imx_clk_hw_mux_flags("sys_pll2_bypass", anatop_base + 0x104, 28, 1, sys_pll2_bypass_sels, ARRAY_SIZE(sys_pll2_bypass_sels), CLK_SET_RATE_PARENT);
	hws[IMX8MP_SYS_PLL3_BYPASS] = imx_clk_hw_mux_flags("sys_pll3_bypass", anatop_base + 0x114, 28, 1, sys_pll3_bypass_sels, ARRAY_SIZE(sys_pll3_bypass_sels), CLK_SET_RATE_PARENT);

	hws[IMX8MP_AUDIO_PLL1_OUT] = imx_clk_hw_gate("audio_pll1_out", "audio_pll1_bypass", anatop_base, 13);
	hws[IMX8MP_AUDIO_PLL2_OUT] = imx_clk_hw_gate("audio_pll2_out", "audio_pll2_bypass", anatop_base + 0x14, 13);
	hws[IMX8MP_VIDEO_PLL1_OUT] = imx_clk_hw_gate("video_pll1_out", "video_pll1_bypass", anatop_base + 0x28, 13);
	hws[IMX8MP_DRAM_PLL_OUT] = imx_clk_hw_gate("dram_pll_out", "dram_pll_bypass", anatop_base + 0x50, 13);
	hws[IMX8MP_GPU_PLL_OUT] = imx_clk_hw_gate("gpu_pll_out", "gpu_pll_bypass", anatop_base + 0x64, 11);
	hws[IMX8MP_VPU_PLL_OUT] = imx_clk_hw_gate("vpu_pll_out", "vpu_pll_bypass", anatop_base + 0x74, 11);
	hws[IMX8MP_ARM_PLL_OUT] = imx_clk_hw_gate("arm_pll_out", "arm_pll_bypass", anatop_base + 0x84, 11);
	hws[IMX8MP_SYS_PLL3_OUT] = imx_clk_hw_gate("sys_pll3_out", "sys_pll3_bypass", anatop_base + 0x114, 11);

	hws[IMX8MP_SYS_PLL1_OUT] = imx_clk_hw_gate("sys_pll1_out", "sys_pll1_bypass", anatop_base + 0x94, 11);

	hws[IMX8MP_SYS_PLL1_40M] = imx_clk_hw_fixed_factor("sys_pll1_40m", "sys_pll1_out", 1, 20);
	hws[IMX8MP_SYS_PLL1_80M] = imx_clk_hw_fixed_factor("sys_pll1_80m", "sys_pll1_out", 1, 10);
	hws[IMX8MP_SYS_PLL1_100M] = imx_clk_hw_fixed_factor("sys_pll1_100m", "sys_pll1_out", 1, 8);
	hws[IMX8MP_SYS_PLL1_133M] = imx_clk_hw_fixed_factor("sys_pll1_133m", "sys_pll1_out", 1, 6);
	hws[IMX8MP_SYS_PLL1_160M] = imx_clk_hw_fixed_factor("sys_pll1_160m", "sys_pll1_out", 1, 5);
	hws[IMX8MP_SYS_PLL1_200M] = imx_clk_hw_fixed_factor("sys_pll1_200m", "sys_pll1_out", 1, 4);
	hws[IMX8MP_SYS_PLL1_266M] = imx_clk_hw_fixed_factor("sys_pll1_266m", "sys_pll1_out", 1, 3);
	hws[IMX8MP_SYS_PLL1_400M] = imx_clk_hw_fixed_factor("sys_pll1_400m", "sys_pll1_out", 1, 2);
	hws[IMX8MP_SYS_PLL1_800M] = imx_clk_hw_fixed_factor("sys_pll1_800m", "sys_pll1_out", 1, 1);

	hws[IMX8MP_SYS_PLL2_OUT] = imx_clk_hw_gate("sys_pll2_out", "sys_pll2_bypass", anatop_base + 0x104, 11);

	hws[IMX8MP_SYS_PLL2_50M] = imx_clk_hw_fixed_factor("sys_pll2_50m", "sys_pll2_out", 1, 20);
	hws[IMX8MP_SYS_PLL2_100M] = imx_clk_hw_fixed_factor("sys_pll2_100m", "sys_pll2_out", 1, 10);
	hws[IMX8MP_SYS_PLL2_125M] = imx_clk_hw_fixed_factor("sys_pll2_125m", "sys_pll2_out", 1, 8);
	hws[IMX8MP_SYS_PLL2_166M] = imx_clk_hw_fixed_factor("sys_pll2_166m", "sys_pll2_out", 1, 6);
	hws[IMX8MP_SYS_PLL2_200M] = imx_clk_hw_fixed_factor("sys_pll2_200m", "sys_pll2_out", 1, 5);
	hws[IMX8MP_SYS_PLL2_250M] = imx_clk_hw_fixed_factor("sys_pll2_250m", "sys_pll2_out", 1, 4);
	hws[IMX8MP_SYS_PLL2_333M] = imx_clk_hw_fixed_factor("sys_pll2_333m", "sys_pll2_out", 1, 3);
	hws[IMX8MP_SYS_PLL2_500M] = imx_clk_hw_fixed_factor("sys_pll2_500m", "sys_pll2_out", 1, 2);
	hws[IMX8MP_SYS_PLL2_1000M] = imx_clk_hw_fixed_factor("sys_pll2_1000m", "sys_pll2_out", 1, 1);

	hws[IMX8MP_CLK_A53_DIV] = imx8m_clk_hw_composite_core("arm_a53_div", imx8mp_a53_sels, ccm_base + 0x8000);
	hws[IMX8MP_CLK_A53_SRC] = hws[IMX8MP_CLK_A53_DIV];
	hws[IMX8MP_CLK_A53_CG] = hws[IMX8MP_CLK_A53_DIV];
	hws[IMX8MP_CLK_M7_CORE] = imx8m_clk_hw_composite_core("m7_core", imx8mp_m7_sels, ccm_base + 0x8080);
	hws[IMX8MP_CLK_ML_CORE] = imx8m_clk_hw_composite_core("ml_core", imx8mp_ml_sels, ccm_base + 0x8100);
	hws[IMX8MP_CLK_GPU3D_CORE] = imx8m_clk_hw_composite_core("gpu3d_core", imx8mp_gpu3d_core_sels, ccm_base + 0x8180);
	hws[IMX8MP_CLK_GPU3D_SHADER_CORE] = imx8m_clk_hw_composite("gpu3d_shader_core", imx8mp_gpu3d_shader_sels, ccm_base + 0x8200);
	hws[IMX8MP_CLK_GPU2D_CORE] = imx8m_clk_hw_composite("gpu2d_core", imx8mp_gpu2d_sels, ccm_base + 0x8280);
	hws[IMX8MP_CLK_AUDIO_AXI] = imx8m_clk_hw_composite("audio_axi", imx8mp_audio_axi_sels, ccm_base + 0x8300);
	hws[IMX8MP_CLK_AUDIO_AXI_SRC] = hws[IMX8MP_CLK_AUDIO_AXI];
	hws[IMX8MP_CLK_HSIO_AXI] = imx8m_clk_hw_composite("hsio_axi", imx8mp_hsio_axi_sels, ccm_base + 0x8380);
	hws[IMX8MP_CLK_MEDIA_ISP] = imx8m_clk_hw_composite("media_isp", imx8mp_media_isp_sels, ccm_base + 0x8400);

	/* CORE SEL */
	hws[IMX8MP_CLK_A53_CORE] = imx_clk_hw_mux2("arm_a53_core", ccm_base + 0x9880, 24, 1, imx8mp_a53_core_sels, ARRAY_SIZE(imx8mp_a53_core_sels));

	hws[IMX8MP_CLK_MAIN_AXI] = imx8m_clk_hw_composite_bus_critical("main_axi", imx8mp_main_axi_sels, ccm_base + 0x8800);
	hws[IMX8MP_CLK_ENET_AXI] = imx8m_clk_hw_composite_bus("enet_axi", imx8mp_enet_axi_sels, ccm_base + 0x8880);
	hws[IMX8MP_CLK_NAND_USDHC_BUS] = imx8m_clk_hw_composite("nand_usdhc_bus", imx8mp_nand_usdhc_sels, ccm_base + 0x8900);
	hws[IMX8MP_CLK_VPU_BUS] = imx8m_clk_hw_composite_bus("vpu_bus", imx8mp_vpu_bus_sels, ccm_base + 0x8980);
	hws[IMX8MP_CLK_MEDIA_AXI] = imx8m_clk_hw_composite_bus("media_axi", imx8mp_media_axi_sels, ccm_base + 0x8a00);
	hws[IMX8MP_CLK_MEDIA_APB] = imx8m_clk_hw_composite_bus("media_apb", imx8mp_media_apb_sels, ccm_base + 0x8a80);
	hws[IMX8MP_CLK_HDMI_APB] = imx8m_clk_hw_composite_bus("hdmi_apb", imx8mp_media_apb_sels, ccm_base + 0x8b00);
	hws[IMX8MP_CLK_HDMI_AXI] = imx8m_clk_hw_composite_bus("hdmi_axi", imx8mp_media_axi_sels, ccm_base + 0x8b80);
	hws[IMX8MP_CLK_GPU_AXI] = imx8m_clk_hw_composite_bus("gpu_axi", imx8mp_gpu_axi_sels, ccm_base + 0x8c00);
	hws[IMX8MP_CLK_GPU_AHB] = imx8m_clk_hw_composite_bus("gpu_ahb", imx8mp_gpu_ahb_sels, ccm_base + 0x8c80);
	hws[IMX8MP_CLK_NOC] = imx8m_clk_hw_composite_bus_critical("noc", imx8mp_noc_sels, ccm_base + 0x8d00);
	hws[IMX8MP_CLK_NOC_IO] = imx8m_clk_hw_composite_bus_critical("noc_io", imx8mp_noc_io_sels, ccm_base + 0x8d80);
	hws[IMX8MP_CLK_ML_AXI] = imx8m_clk_hw_composite_bus("ml_axi", imx8mp_ml_axi_sels, ccm_base + 0x8e00);
	hws[IMX8MP_CLK_ML_AHB] = imx8m_clk_hw_composite_bus("ml_ahb", imx8mp_ml_ahb_sels, ccm_base + 0x8e80);

	hws[IMX8MP_CLK_AHB] = imx8m_clk_hw_composite_bus_critical("ahb_root", imx8mp_ahb_sels, ccm_base + 0x9000);
	hws[IMX8MP_CLK_AUDIO_AHB] = imx8m_clk_hw_composite_bus("audio_ahb", imx8mp_audio_ahb_sels, ccm_base + 0x9100);
	hws[IMX8MP_CLK_MIPI_DSI_ESC_RX] = imx8m_clk_hw_composite_bus("mipi_dsi_esc_rx", imx8mp_mipi_dsi_esc_rx_sels, ccm_base + 0x9200);
	hws[IMX8MP_CLK_MEDIA_DISP2_PIX] = imx8m_clk_hw_composite_bus("media_disp2_pix", imx8mp_media_disp2_pix_sels, ccm_base + 0x9300);

	hws[IMX8MP_CLK_IPG_ROOT] = imx_clk_hw_divider2("ipg_root", "ahb_root", ccm_base + 0x9080, 0, 1);

	hws[IMX8MP_CLK_DRAM_ALT] = imx8m_clk_hw_fw_managed_composite("dram_alt", imx8mp_dram_alt_sels, ccm_base + 0xa000);
	hws[IMX8MP_CLK_DRAM_APB] = imx8m_clk_hw_fw_managed_composite_critical("dram_apb", imx8mp_dram_apb_sels, ccm_base + 0xa080);
	hws[IMX8MP_CLK_VPU_G1] = imx8m_clk_hw_composite("vpu_g1", imx8mp_vpu_g1_sels, ccm_base + 0xa100);
	hws[IMX8MP_CLK_VPU_G2] = imx8m_clk_hw_composite("vpu_g2", imx8mp_vpu_g2_sels, ccm_base + 0xa180);
	hws[IMX8MP_CLK_CAN1] = imx8m_clk_hw_composite("can1", imx8mp_can1_sels, ccm_base + 0xa200);
	hws[IMX8MP_CLK_CAN2] = imx8m_clk_hw_composite("can2", imx8mp_can2_sels, ccm_base + 0xa280);
	hws[IMX8MP_CLK_PCIE_AUX] = imx8m_clk_hw_composite("pcie_aux", imx8mp_pcie_aux_sels, ccm_base + 0xa400);
	hws[IMX8MP_CLK_I2C5] = imx8m_clk_hw_composite("i2c5", imx8mp_i2c5_sels, ccm_base + 0xa480);
	hws[IMX8MP_CLK_I2C6] = imx8m_clk_hw_composite("i2c6", imx8mp_i2c6_sels, ccm_base + 0xa500);
	hws[IMX8MP_CLK_SAI1] = imx8m_clk_hw_composite("sai1", imx8mp_sai1_sels, ccm_base + 0xa580);
	hws[IMX8MP_CLK_SAI2] = imx8m_clk_hw_composite("sai2", imx8mp_sai2_sels, ccm_base + 0xa600);
	hws[IMX8MP_CLK_SAI3] = imx8m_clk_hw_composite("sai3", imx8mp_sai3_sels, ccm_base + 0xa680);
	hws[IMX8MP_CLK_SAI4] = imx8m_clk_hw_composite("sai4", imx8mp_sai4_sels, ccm_base + 0xa700);
	hws[IMX8MP_CLK_SAI5] = imx8m_clk_hw_composite("sai5", imx8mp_sai5_sels, ccm_base + 0xa780);
	hws[IMX8MP_CLK_SAI6] = imx8m_clk_hw_composite("sai6", imx8mp_sai6_sels, ccm_base + 0xa800);
	hws[IMX8MP_CLK_ENET_QOS] = imx8m_clk_hw_composite("enet_qos", imx8mp_enet_qos_sels, ccm_base + 0xa880);
	hws[IMX8MP_CLK_ENET_QOS_TIMER] = imx8m_clk_hw_composite("enet_qos_timer", imx8mp_enet_qos_timer_sels, ccm_base + 0xa900);
	hws[IMX8MP_CLK_ENET_REF] = imx8m_clk_hw_composite("enet_ref", imx8mp_enet_ref_sels, ccm_base + 0xa980);
	hws[IMX8MP_CLK_ENET_TIMER] = imx8m_clk_hw_composite("enet_timer", imx8mp_enet_timer_sels, ccm_base + 0xaa00);
	hws[IMX8MP_CLK_ENET_PHY_REF] = imx8m_clk_hw_composite("enet_phy_ref", imx8mp_enet_phy_ref_sels, ccm_base + 0xaa80);
	hws[IMX8MP_CLK_NAND] = imx8m_clk_hw_composite("nand", imx8mp_nand_sels, ccm_base + 0xab00);
	hws[IMX8MP_CLK_QSPI] = imx8m_clk_hw_composite("qspi", imx8mp_qspi_sels, ccm_base + 0xab80);
	hws[IMX8MP_CLK_USDHC1] = imx8m_clk_hw_composite("usdhc1", imx8mp_usdhc1_sels, ccm_base + 0xac00);
	hws[IMX8MP_CLK_USDHC2] = imx8m_clk_hw_composite("usdhc2", imx8mp_usdhc2_sels, ccm_base + 0xac80);
	hws[IMX8MP_CLK_I2C1] = imx8m_clk_hw_composite("i2c1", imx8mp_i2c1_sels, ccm_base + 0xad00);
	hws[IMX8MP_CLK_I2C2] = imx8m_clk_hw_composite("i2c2", imx8mp_i2c2_sels, ccm_base + 0xad80);
	hws[IMX8MP_CLK_I2C3] = imx8m_clk_hw_composite("i2c3", imx8mp_i2c3_sels, ccm_base + 0xae00);
	hws[IMX8MP_CLK_I2C4] = imx8m_clk_hw_composite("i2c4", imx8mp_i2c4_sels, ccm_base + 0xae80);

	hws[IMX8MP_CLK_UART1] = imx8m_clk_hw_composite("uart1", imx8mp_uart1_sels, ccm_base + 0xaf00);
	hws[IMX8MP_CLK_UART2] = imx8m_clk_hw_composite_critical("uart2", imx8mp_uart2_sels, ccm_base + 0xaf80);
	hws[IMX8MP_CLK_UART3] = imx8m_clk_hw_composite("uart3", imx8mp_uart3_sels, ccm_base + 0xb000);
	hws[IMX8MP_CLK_UART4] = imx8m_clk_hw_composite("uart4", imx8mp_uart4_sels, ccm_base + 0xb080);
	hws[IMX8MP_CLK_USB_CORE_REF] = imx8m_clk_hw_composite("usb_core_ref", imx8mp_usb_core_ref_sels, ccm_base + 0xb100);
	hws[IMX8MP_CLK_USB_PHY_REF] = imx8m_clk_hw_composite("usb_phy_ref", imx8mp_usb_phy_ref_sels, ccm_base + 0xb180);
	hws[IMX8MP_CLK_GIC] = imx8m_clk_hw_composite_critical("gic", imx8mp_gic_sels, ccm_base + 0xb200);
	hws[IMX8MP_CLK_ECSPI1] = imx8m_clk_hw_composite("ecspi1", imx8mp_ecspi1_sels, ccm_base + 0xb280);
	hws[IMX8MP_CLK_ECSPI2] = imx8m_clk_hw_composite("ecspi2", imx8mp_ecspi2_sels, ccm_base + 0xb300);
	hws[IMX8MP_CLK_PWM1] = imx8m_clk_hw_composite("pwm1", imx8mp_pwm1_sels, ccm_base + 0xb380);
	hws[IMX8MP_CLK_PWM2] = imx8m_clk_hw_composite("pwm2", imx8mp_pwm2_sels, ccm_base + 0xb400);
	hws[IMX8MP_CLK_PWM3] = imx8m_clk_hw_composite("pwm3", imx8mp_pwm3_sels, ccm_base + 0xb480);
	hws[IMX8MP_CLK_PWM4] = imx8m_clk_hw_composite("pwm4", imx8mp_pwm4_sels, ccm_base + 0xb500);

	hws[IMX8MP_CLK_GPT1] = imx8m_clk_hw_composite("gpt1", imx8mp_gpt1_sels, ccm_base + 0xb580);
	hws[IMX8MP_CLK_GPT2] = imx8m_clk_hw_composite("gpt2", imx8mp_gpt2_sels, ccm_base + 0xb600);
	hws[IMX8MP_CLK_GPT3] = imx8m_clk_hw_composite("gpt3", imx8mp_gpt3_sels, ccm_base + 0xb680);
	hws[IMX8MP_CLK_GPT4] = imx8m_clk_hw_composite("gpt4", imx8mp_gpt4_sels, ccm_base + 0xb700);
	hws[IMX8MP_CLK_GPT5] = imx8m_clk_hw_composite("gpt5", imx8mp_gpt5_sels, ccm_base + 0xb780);
	hws[IMX8MP_CLK_GPT6] = imx8m_clk_hw_composite("gpt6", imx8mp_gpt6_sels, ccm_base + 0xb800);
	hws[IMX8MP_CLK_WDOG] = imx8m_clk_hw_composite("wdog", imx8mp_wdog_sels, ccm_base + 0xb900);
	hws[IMX8MP_CLK_WRCLK] = imx8m_clk_hw_composite("wrclk", imx8mp_wrclk_sels, ccm_base + 0xb980);
	hws[IMX8MP_CLK_IPP_DO_CLKO1] = imx8m_clk_hw_composite("ipp_do_clko1", imx8mp_ipp_do_clko1_sels, ccm_base + 0xba00);
	hws[IMX8MP_CLK_IPP_DO_CLKO2] = imx8m_clk_hw_composite("ipp_do_clko2", imx8mp_ipp_do_clko2_sels, ccm_base + 0xba80);
	hws[IMX8MP_CLK_HDMI_FDCC_TST] = imx8m_clk_hw_composite("hdmi_fdcc_tst", imx8mp_hdmi_fdcc_tst_sels, ccm_base + 0xbb00);
	hws[IMX8MP_CLK_HDMI_24M] = imx8m_clk_hw_composite("hdmi_24m", imx8mp_hdmi_24m_sels, ccm_base + 0xbb80);
	hws[IMX8MP_CLK_HDMI_REF_266M] = imx8m_clk_hw_composite("hdmi_ref_266m", imx8mp_hdmi_ref_266m_sels, ccm_base + 0xbc00);
	hws[IMX8MP_CLK_USDHC3] = imx8m_clk_hw_composite("usdhc3", imx8mp_usdhc3_sels, ccm_base + 0xbc80);
	hws[IMX8MP_CLK_MEDIA_CAM1_PIX] = imx8m_clk_hw_composite("media_cam1_pix", imx8mp_media_cam1_pix_sels, ccm_base + 0xbd00);
	hws[IMX8MP_CLK_MEDIA_MIPI_PHY1_REF] = imx8m_clk_hw_composite("media_mipi_phy1_ref", imx8mp_media_mipi_phy1_ref_sels, ccm_base + 0xbd80);
	hws[IMX8MP_CLK_MEDIA_DISP1_PIX] = imx8m_clk_hw_composite("media_disp1_pix", imx8mp_media_disp1_pix_sels, ccm_base + 0xbe00);
	hws[IMX8MP_CLK_MEDIA_CAM2_PIX] = imx8m_clk_hw_composite("media_cam2_pix", imx8mp_media_cam2_pix_sels, ccm_base + 0xbe80);
	hws[IMX8MP_CLK_MEDIA_LDB] = imx8m_clk_hw_composite("media_ldb", imx8mp_media_ldb_sels, ccm_base + 0xbf00);
	hws[IMX8MP_CLK_MEMREPAIR] = imx8m_clk_hw_composite_critical("mem_repair", imx8mp_memrepair_sels, ccm_base + 0xbf80);
	hws[IMX8MP_CLK_MEDIA_MIPI_TEST_BYTE] = imx8m_clk_hw_composite("media_mipi_test_byte", imx8mp_media_mipi_test_byte_sels, ccm_base + 0xc100);
	hws[IMX8MP_CLK_ECSPI3] = imx8m_clk_hw_composite("ecspi3", imx8mp_ecspi3_sels, ccm_base + 0xc180);
	hws[IMX8MP_CLK_PDM] = imx8m_clk_hw_composite("pdm", imx8mp_pdm_sels, ccm_base + 0xc200);
	hws[IMX8MP_CLK_VPU_VC8000E] = imx8m_clk_hw_composite("vpu_vc8000e", imx8mp_vpu_vc8000e_sels, ccm_base + 0xc280);
	hws[IMX8MP_CLK_SAI7] = imx8m_clk_hw_composite("sai7", imx8mp_sai7_sels, ccm_base + 0xc300);

	hws[IMX8MP_CLK_DRAM_ALT_ROOT] = imx_clk_hw_fixed_factor("dram_alt_root", "dram_alt", 1, 4);
	hws[IMX8MP_CLK_DRAM_CORE] = imx_clk_hw_mux2_flags("dram_core_clk", ccm_base + 0x9800, 24, 1, imx8mp_dram_core_sels, ARRAY_SIZE(imx8mp_dram_core_sels), CLK_IS_CRITICAL);

	hws[IMX8MP_CLK_DRAM1_ROOT] = imx_clk_hw_gate4_flags("dram1_root_clk", "dram_core_clk", ccm_base + 0x4050, 0, CLK_IS_CRITICAL);
	hws[IMX8MP_CLK_ECSPI1_ROOT] = imx_clk_hw_gate4("ecspi1_root_clk", "ecspi1", ccm_base + 0x4070, 0);
	hws[IMX8MP_CLK_ECSPI2_ROOT] = imx_clk_hw_gate4("ecspi2_root_clk", "ecspi2", ccm_base + 0x4080, 0);
	hws[IMX8MP_CLK_ECSPI3_ROOT] = imx_clk_hw_gate4("ecspi3_root_clk", "ecspi3", ccm_base + 0x4090, 0);
	hws[IMX8MP_CLK_ENET1_ROOT] = imx_clk_hw_gate4("enet1_root_clk", "enet_axi", ccm_base + 0x40a0, 0);
	hws[IMX8MP_CLK_GPIO1_ROOT] = imx_clk_hw_gate4("gpio1_root_clk", "ipg_root", ccm_base + 0x40b0, 0);
	hws[IMX8MP_CLK_GPIO2_ROOT] = imx_clk_hw_gate4("gpio2_root_clk", "ipg_root", ccm_base + 0x40c0, 0);
	hws[IMX8MP_CLK_GPIO3_ROOT] = imx_clk_hw_gate4("gpio3_root_clk", "ipg_root", ccm_base + 0x40d0, 0);
	hws[IMX8MP_CLK_GPIO4_ROOT] = imx_clk_hw_gate4("gpio4_root_clk", "ipg_root", ccm_base + 0x40e0, 0);
	hws[IMX8MP_CLK_GPIO5_ROOT] = imx_clk_hw_gate4("gpio5_root_clk", "ipg_root", ccm_base + 0x40f0, 0);
	hws[IMX8MP_CLK_GPT1_ROOT] = imx_clk_hw_gate4("gpt1_root_clk", "gpt1", ccm_base + 0x4100, 0);
	hws[IMX8MP_CLK_GPT2_ROOT] = imx_clk_hw_gate4("gpt2_root_clk", "gpt2", ccm_base + 0x4110, 0);
	hws[IMX8MP_CLK_GPT3_ROOT] = imx_clk_hw_gate4("gpt3_root_clk", "gpt3", ccm_base + 0x4120, 0);
	hws[IMX8MP_CLK_GPT4_ROOT] = imx_clk_hw_gate4("gpt4_root_clk", "gpt4", ccm_base + 0x4130, 0);
	hws[IMX8MP_CLK_GPT5_ROOT] = imx_clk_hw_gate4("gpt5_root_clk", "gpt5", ccm_base + 0x4140, 0);
	hws[IMX8MP_CLK_GPT6_ROOT] = imx_clk_hw_gate4("gpt6_root_clk", "gpt6", ccm_base + 0x4150, 0);
	hws[IMX8MP_CLK_I2C1_ROOT] = imx_clk_hw_gate4("i2c1_root_clk", "i2c1", ccm_base + 0x4170, 0);
	hws[IMX8MP_CLK_I2C2_ROOT] = imx_clk_hw_gate4("i2c2_root_clk", "i2c2", ccm_base + 0x4180, 0);
	hws[IMX8MP_CLK_I2C3_ROOT] = imx_clk_hw_gate4("i2c3_root_clk", "i2c3", ccm_base + 0x4190, 0);
	hws[IMX8MP_CLK_I2C4_ROOT] = imx_clk_hw_gate4("i2c4_root_clk", "i2c4", ccm_base + 0x41a0, 0);
	hws[IMX8MP_CLK_MU_ROOT] = imx_clk_hw_gate4("mu_root_clk", "ipg_root", ccm_base + 0x4210, 0);
	hws[IMX8MP_CLK_OCOTP_ROOT] = imx_clk_hw_gate4("ocotp_root_clk", "ipg_root", ccm_base + 0x4220, 0);
	hws[IMX8MP_CLK_PCIE_ROOT] = imx_clk_hw_gate4("pcie_root_clk", "pcie_aux", ccm_base + 0x4250, 0);
	hws[IMX8MP_CLK_PWM1_ROOT] = imx_clk_hw_gate4("pwm1_root_clk", "pwm1", ccm_base + 0x4280, 0);
	hws[IMX8MP_CLK_PWM2_ROOT] = imx_clk_hw_gate4("pwm2_root_clk", "pwm2", ccm_base + 0x4290, 0);
	hws[IMX8MP_CLK_PWM3_ROOT] = imx_clk_hw_gate4("pwm3_root_clk", "pwm3", ccm_base + 0x42a0, 0);
	hws[IMX8MP_CLK_PWM4_ROOT] = imx_clk_hw_gate4("pwm4_root_clk", "pwm4", ccm_base + 0x42b0, 0);
	hws[IMX8MP_CLK_QOS_ROOT] = imx_clk_hw_gate4("qos_root_clk", "ipg_root", ccm_base + 0x42c0, 0);
	hws[IMX8MP_CLK_QOS_ENET_ROOT] = imx_clk_hw_gate4("qos_enet_root_clk", "ipg_root", ccm_base + 0x42e0, 0);
	hws[IMX8MP_CLK_QSPI_ROOT] = imx_clk_hw_gate4("qspi_root_clk", "qspi", ccm_base + 0x42f0, 0);
	hws[IMX8MP_CLK_NAND_ROOT] = imx_clk_hw_gate2_shared2("nand_root_clk", "nand", ccm_base + 0x4300, 0, &share_count_nand);
	hws[IMX8MP_CLK_NAND_USDHC_BUS_RAWNAND_CLK] = imx_clk_hw_gate2_shared2("nand_usdhc_rawnand_clk", "nand_usdhc_bus", ccm_base + 0x4300, 0, &share_count_nand);
	hws[IMX8MP_CLK_I2C5_ROOT] = imx_clk_hw_gate2("i2c5_root_clk", "i2c5", ccm_base + 0x4330, 0);
	hws[IMX8MP_CLK_I2C6_ROOT] = imx_clk_hw_gate2("i2c6_root_clk", "i2c6", ccm_base + 0x4340, 0);
	hws[IMX8MP_CLK_CAN1_ROOT] = imx_clk_hw_gate2("can1_root_clk", "can1", ccm_base + 0x4350, 0);
	hws[IMX8MP_CLK_CAN2_ROOT] = imx_clk_hw_gate2("can2_root_clk", "can2", ccm_base + 0x4360, 0);
	hws[IMX8MP_CLK_SDMA1_ROOT] = imx_clk_hw_gate4("sdma1_root_clk", "ipg_root", ccm_base + 0x43a0, 0);
	hws[IMX8MP_CLK_SIM_ENET_ROOT] = imx_clk_hw_gate4("sim_enet_root_clk", "enet_axi", ccm_base + 0x4400, 0);
	hws[IMX8MP_CLK_ENET_QOS_ROOT] = imx_clk_hw_gate4("enet_qos_root_clk", "sim_enet_root_clk", ccm_base + 0x43b0, 0);
	hws[IMX8MP_CLK_GPU2D_ROOT] = imx_clk_hw_gate4("gpu2d_root_clk", "gpu2d_core", ccm_base + 0x4450, 0);
	hws[IMX8MP_CLK_GPU3D_ROOT] = imx_clk_hw_gate4("gpu3d_root_clk", "gpu3d_core", ccm_base + 0x4460, 0);
	hws[IMX8MP_CLK_SNVS_ROOT] = imx_clk_hw_gate2_flags("snvs_root_clk", "ipg_root", ccm_base + 0x4470, 0, CLK_IS_CRITICAL);
	hws[IMX8MP_CLK_UART1_ROOT] = imx_clk_hw_gate4("uart1_root_clk", "uart1", ccm_base + 0x4490, 0);
	hws[IMX8MP_CLK_UART2_ROOT] = imx_clk_hw_gate4("uart2_root_clk", "uart2", ccm_base + 0x44a0, 0);
	hws[IMX8MP_CLK_UART3_ROOT] = imx_clk_hw_gate4("uart3_root_clk", "uart3", ccm_base + 0x44b0, 0);
	hws[IMX8MP_CLK_UART4_ROOT] = imx_clk_hw_gate4("uart4_root_clk", "uart4", ccm_base + 0x44c0, 0);
	hws[IMX8MP_CLK_USB_ROOT] = imx_clk_hw_gate4("usb_root_clk", "hsio_axi", ccm_base + 0x44d0, 0);
	hws[IMX8MP_CLK_USB_PHY_ROOT] = imx_clk_hw_gate4("usb_phy_root_clk", "usb_phy_ref", ccm_base + 0x44f0, 0);
	hws[IMX8MP_CLK_USDHC1_ROOT] = imx_clk_hw_gate4("usdhc1_root_clk", "usdhc1", ccm_base + 0x4510, 0);
	hws[IMX8MP_CLK_USDHC2_ROOT] = imx_clk_hw_gate4("usdhc2_root_clk", "usdhc2", ccm_base + 0x4520, 0);
	hws[IMX8MP_CLK_WDOG1_ROOT] = imx_clk_hw_gate4("wdog1_root_clk", "wdog", ccm_base + 0x4530, 0);
	hws[IMX8MP_CLK_WDOG2_ROOT] = imx_clk_hw_gate4("wdog2_root_clk", "wdog", ccm_base + 0x4540, 0);
	hws[IMX8MP_CLK_WDOG3_ROOT] = imx_clk_hw_gate4("wdog3_root_clk", "wdog", ccm_base + 0x4550, 0);
	hws[IMX8MP_CLK_VPU_G1_ROOT] = imx_clk_hw_gate4("vpu_g1_root_clk", "vpu_g1", ccm_base + 0x4560, 0);
	hws[IMX8MP_CLK_GPU_ROOT] = imx_clk_hw_gate4("gpu_root_clk", "gpu_axi", ccm_base + 0x4570, 0);
	hws[IMX8MP_CLK_VPU_VC8KE_ROOT] = imx_clk_hw_gate4("vpu_vc8ke_root_clk", "vpu_vc8000e", ccm_base + 0x4590, 0);
	hws[IMX8MP_CLK_VPU_G2_ROOT] = imx_clk_hw_gate4("vpu_g2_root_clk", "vpu_g2", ccm_base + 0x45a0, 0);
	hws[IMX8MP_CLK_NPU_ROOT] = imx_clk_hw_gate4("npu_root_clk", "ml_core", ccm_base + 0x45b0, 0);
	hws[IMX8MP_CLK_HSIO_ROOT] = imx_clk_hw_gate4("hsio_root_clk", "ipg_root", ccm_base + 0x45c0, 0);
	hws[IMX8MP_CLK_MEDIA_APB_ROOT] = imx_clk_hw_gate2_shared2("media_apb_root_clk", "media_apb", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_AXI_ROOT] = imx_clk_hw_gate2_shared2("media_axi_root_clk", "media_axi", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_CAM1_PIX_ROOT] = imx_clk_hw_gate2_shared2("media_cam1_pix_root_clk", "media_cam1_pix", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_CAM2_PIX_ROOT] = imx_clk_hw_gate2_shared2("media_cam2_pix_root_clk", "media_cam2_pix", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_DISP1_PIX_ROOT] = imx_clk_hw_gate2_shared2("media_disp1_pix_root_clk", "media_disp1_pix", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_DISP2_PIX_ROOT] = imx_clk_hw_gate2_shared2("media_disp2_pix_root_clk", "media_disp2_pix", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_LDB_ROOT] = imx_clk_hw_gate2_shared2("media_ldb_root_clk", "media_ldb", ccm_base + 0x45d0, 0, &share_count_media);
	hws[IMX8MP_CLK_MEDIA_ISP_ROOT] = imx_clk_hw_gate2_shared2("media_isp_root_clk", "media_isp", ccm_base + 0x45d0, 0, &share_count_media);

	hws[IMX8MP_CLK_USDHC3_ROOT] = imx_clk_hw_gate4("usdhc3_root_clk", "usdhc3", ccm_base + 0x45e0, 0);
	hws[IMX8MP_CLK_HDMI_ROOT] = imx_clk_hw_gate4("hdmi_root_clk", "hdmi_axi", ccm_base + 0x45f0, 0);
	hws[IMX8MP_CLK_TSENSOR_ROOT] = imx_clk_hw_gate4("tsensor_root_clk", "ipg_root", ccm_base + 0x4620, 0);
	hws[IMX8MP_CLK_VPU_ROOT] = imx_clk_hw_gate4("vpu_root_clk", "vpu_bus", ccm_base + 0x4630, 0);

	hws[IMX8MP_CLK_AUDIO_AHB_ROOT] = imx_clk_hw_gate2_shared2("audio_ahb_root", "audio_ahb", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_AUDIO_AXI_ROOT] = imx_clk_hw_gate2_shared2("audio_axi_root", "audio_axi", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI1_ROOT] = imx_clk_hw_gate2_shared2("sai1_root", "sai1", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI2_ROOT] = imx_clk_hw_gate2_shared2("sai2_root", "sai2", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI3_ROOT] = imx_clk_hw_gate2_shared2("sai3_root", "sai3", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI5_ROOT] = imx_clk_hw_gate2_shared2("sai5_root", "sai5", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI6_ROOT] = imx_clk_hw_gate2_shared2("sai6_root", "sai6", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_SAI7_ROOT] = imx_clk_hw_gate2_shared2("sai7_root", "sai7", ccm_base + 0x4650, 0, &share_count_audio);
	hws[IMX8MP_CLK_PDM_ROOT] = imx_clk_hw_gate2_shared2("pdm_root", "pdm", ccm_base + 0x4650, 0, &share_count_audio);

	hws[IMX8MP_CLK_ARM] = imx_clk_hw_cpu("arm", "arm_a53_core",
					     hws[IMX8MP_CLK_A53_CORE]->clk,
					     hws[IMX8MP_CLK_A53_CORE]->clk,
					     hws[IMX8MP_ARM_PLL_OUT]->clk,
					     hws[IMX8MP_CLK_A53_DIV]->clk);

	imx_check_clk_hws(hws, IMX8MP_CLK_END);

	of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_hw_data);

	imx_clk_init_on(np, hws);
	
	imx_register_uart_clocks(4);

	return 0;
}

static const struct of_device_id imx8mp_clk_of_match[] = {
	{ .compatible = "fsl,imx8mp-ccm" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx8mp_clk_of_match);

static struct platform_driver imx8mp_clk_driver = {
	.probe = imx8mp_clocks_probe,
	.driver = {
		.name = "imx8mp-ccm",
		/*
		 * Disable bind attributes: clocks are not removed and
		 * reloading the driver will crash or break devices.
		 */
		.suppress_bind_attrs = true,
		.of_match_table = imx8mp_clk_of_match,
	},
};
module_platform_driver(imx8mp_clk_driver);
module_param(mcore_booted, bool, S_IRUGO);
MODULE_PARM_DESC(mcore_booted, "See Cortex-M core is booted or not");

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX8MP clock driver");
MODULE_LICENSE("GPL v2");

#ifndef MODULE
/*
 * Debugfs interface for audio PLL K divider change dynamically.
 * Monitor control for the Audio PLL K-Divider
 */
#ifdef CONFIG_DEBUG_FS

#define KDIV_MASK	GENMASK(15, 0)
#define MDIV_SHIFT	12
#define MDIV_MASK	GENMASK(21, 12)
#define PDIV_SHIFT	4
#define PDIV_MASK	GENMASK(9, 4)
#define SDIV_SHIFT	0
#define SDIV_MASK	GENMASK(2, 0)

static int pll_delta_k_set(void *data, u64 val)
{
	struct clk_hw *hw;
	short int delta_k;

	hw = data;
	delta_k = (short int) (val & KDIV_MASK);

	clk_set_delta_k(hw, val);

	pr_debug("the delta k is %d\n", delta_k);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(delta_k_fops, NULL, pll_delta_k_set, "%lld\n");

static int pll_setting_show(struct seq_file *s, void *data)
{
	struct clk_hw *hw;
	u32 pll_div_ctrl0, pll_div_ctrl1;
	u32 mdiv, pdiv, sdiv, kdiv;

	hw = s->private;

	clk_get_pll_setting(hw, &pll_div_ctrl0, &pll_div_ctrl1);
	mdiv = (pll_div_ctrl0 & MDIV_MASK) >> MDIV_SHIFT;
	pdiv = (pll_div_ctrl0 & PDIV_MASK) >> PDIV_SHIFT;
	sdiv = (pll_div_ctrl0 & SDIV_MASK) >> SDIV_SHIFT;
	kdiv = (pll_div_ctrl1 & KDIV_MASK);

	seq_printf(s, "Mdiv: 0x%x; Pdiv: 0x%x; Sdiv: 0x%x; Kdiv: 0x%x\n",
		mdiv, pdiv, sdiv, kdiv);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pll_setting);

static int __init pll_debug_init(void)
{
	struct dentry *root, *audio_pll1, *audio_pll2;

	if (of_machine_is_compatible("fsl,imx8mp") && hws) {
		/* create a root dir for audio pll monitor */
		root = debugfs_create_dir("audio_pll_monitor", NULL);
		audio_pll1 = debugfs_create_dir("audio_pll1", root);
		audio_pll2 = debugfs_create_dir("audio_pll2", root);

		debugfs_create_file_unsafe("delta_k", 0444, audio_pll1,
			hws[IMX8MP_AUDIO_PLL1], &delta_k_fops);
		debugfs_create_file("pll_parameter", 0x444, audio_pll1,
			hws[IMX8MP_AUDIO_PLL1], &pll_setting_fops);
		debugfs_create_file_unsafe("delta_k", 0444, audio_pll2,
			hws[IMX8MP_AUDIO_PLL2], &delta_k_fops);
		debugfs_create_file("pll_parameter", 0x444, audio_pll2,
			hws[IMX8MP_AUDIO_PLL2], &pll_setting_fops);
	}

	return 0;
}
late_initcall(pll_debug_init);
#endif /* CONFIG_DEBUG_FS */
#endif /* MODULE */
