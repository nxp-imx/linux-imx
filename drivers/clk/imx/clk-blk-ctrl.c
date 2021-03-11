// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 NXP.
 */

#include <dt-bindings/clock/imx8mp-clock.h>
#include <dt-bindings/reset/imx8mp-reset.h>
#include <linux/clk.h>
#include <linux/reset-controller.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

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
	IMX_BLK_CTRL_RESET(IMX8MP_HDMI_BLK_CTRL_HDMI_PAI_RESET, 0x20, 22),
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
	IMX_BLK_CTRL_CLK_GATE("isp1_cor_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP1_COR, 0x4, 13, "media_isp_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp1_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP1_AXI, 0x4, 14, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp1_ahb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP1_AHB, 0x4, 15, "media_apb_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp0_cor_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP0_COR, 0x4, 16, "media_isp_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp0_axi_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP0_AXI, 0x4, 17, "media_axi_root_clk"),
	IMX_BLK_CTRL_CLK_GATE("isp0_ahb_clk", IMX8MP_CLK_MEDIA_BLK_CTRL_ISP0_AHB, 0x4, 18, "media_apb_root_clk"),
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

const struct imx_blk_ctrl_dev_data imx8mp_hdmi_blk_ctrl_dev_data __initconst = {
	.hws = imx8mp_hdmi_blk_ctrl_hws,
	.hws_num = ARRAY_SIZE(imx8mp_hdmi_blk_ctrl_hws),
	.clocks_max = IMX8MP_CLK_HDMI_BLK_CTRL_END,
	.resets_max = IMX8MP_HDMI_BLK_CTRL_RESET_NUM,
	.pm_runtime_saved_regs_num = 0
};

const struct imx_blk_ctrl_dev_data imx8mp_media_blk_ctrl_dev_data __initconst = {
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

const struct imx_blk_ctrl_dev_data imx8mp_audio_blk_ctrl_dev_data __initconst = {
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

struct reset_hw {
	u32 offset;
	u32 shift;
	u32 mask;
	unsigned long asserted;
};

struct pm_safekeep_info {
	uint32_t *regs_values;
	uint32_t *regs_offsets;
	uint32_t regs_num;
};

struct imx_blk_ctrl_drvdata {
	void __iomem *base;
	struct reset_controller_dev rcdev;
	struct reset_hw *rst_hws;
	struct pm_safekeep_info pm_info;

	spinlock_t *lock;
};

static int imx_blk_ctrl_reset_set(struct reset_controller_dev *rcdev,
				  unsigned long id, bool assert)
{
	struct imx_blk_ctrl_drvdata *drvdata = container_of(rcdev,
			struct imx_blk_ctrl_drvdata, rcdev);
	unsigned int offset = drvdata->rst_hws[id].offset;
	unsigned int shift = drvdata->rst_hws[id].shift;
	unsigned int mask = drvdata->rst_hws[id].mask;
	void __iomem *reg_addr = drvdata->base + offset;
	unsigned long flags;
	u32 reg;

	if (!assert && !test_bit(1, &drvdata->rst_hws[id].asserted))
		return -ENODEV;

	if (assert && !test_and_set_bit(1, &drvdata->rst_hws[id].asserted))
		pm_runtime_get_sync(rcdev->dev);

	spin_lock_irqsave(drvdata->lock, flags);

	reg = readl(reg_addr);
	if (assert)
		writel(reg & ~(mask << shift), reg_addr);
	else
		writel(reg | (mask << shift), reg_addr);

	spin_unlock_irqrestore(drvdata->lock, flags);

	if (!assert && test_and_clear_bit(1, &drvdata->rst_hws[id].asserted))
		pm_runtime_put(rcdev->dev);

	return 0;
}

static int imx_blk_ctrl_reset_reset(struct reset_controller_dev *rcdev,
					   unsigned long id)
{
	imx_blk_ctrl_reset_set(rcdev, id, true);
	return imx_blk_ctrl_reset_set(rcdev, id, false);
}

static int imx_blk_ctrl_reset_assert(struct reset_controller_dev *rcdev,
					   unsigned long id)
{
	return imx_blk_ctrl_reset_set(rcdev, id, true);
}

static int imx_blk_ctrl_reset_deassert(struct reset_controller_dev *rcdev,
					     unsigned long id)
{
	return imx_blk_ctrl_reset_set(rcdev, id, false);
}

static const struct reset_control_ops imx_blk_ctrl_reset_ops = {
	.reset		= imx_blk_ctrl_reset_reset,
	.assert		= imx_blk_ctrl_reset_assert,
	.deassert	= imx_blk_ctrl_reset_deassert,
};

static int imx_blk_ctrl_register_reset_controller(struct device *dev)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct reset_hw *hws;
	int max = dev_data->resets_max;
	int i;

	drvdata->lock = &imx_ccm_lock;

	drvdata->rcdev.owner     = THIS_MODULE;
	drvdata->rcdev.nr_resets = max;
	drvdata->rcdev.ops       = &imx_blk_ctrl_reset_ops;
	drvdata->rcdev.of_node   = dev->of_node;
	drvdata->rcdev.dev	 = dev;

	drvdata->rst_hws = devm_kzalloc(dev, sizeof(struct reset_hw) * max,
					GFP_KERNEL);
	hws = drvdata->rst_hws;

	for (i = 0; i < dev_data->hws_num; i++) {
		struct imx_blk_ctrl_hw *hw = &dev_data->hws[i];

		if (hw->type != BLK_CTRL_RESET)
			continue;

		hws[hw->id].offset = hw->offset;
		hws[hw->id].shift = hw->shift;
		hws[hw->id].mask = hw->mask;
	}

	return devm_reset_controller_register(dev, &drvdata->rcdev);
}
static struct clk_hw *imx_blk_ctrl_register_one_clock(struct device *dev,
						struct imx_blk_ctrl_hw *hw)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	void __iomem *base = drvdata->base;
	struct clk_hw *clk_hw;

	switch (hw->type) {
	case BLK_CTRL_CLK_MUX:
		clk_hw = imx_dev_clk_hw_mux_flags(dev, hw->name,
						  base + hw->offset,
						  hw->shift, hw->width,
						  hw->parents,
						  hw->parents_count,
						  hw->flags);
		break;
	case BLK_CTRL_CLK_GATE:
		clk_hw = imx_dev_clk_hw_gate(dev, hw->name, hw->parents,
					     base + hw->offset, hw->shift);
		break;
	case BLK_CTRL_CLK_SHARED_GATE:
		clk_hw = imx_dev_clk_hw_gate_shared(dev, hw->name,
						    hw->parents,
						    base + hw->offset,
						    hw->shift,
						    hw->shared_count);
		break;
	case BLK_CTRL_CLK_PLL14XX:
		clk_hw = imx_dev_clk_hw_pll14xx(dev, hw->name, hw->parents,
						base + hw->offset, hw->pll_tbl);
		break;
	default:
		clk_hw = NULL;
	};

	return clk_hw;
}

static int imx_blk_ctrl_register_clock_controller(struct device *dev)
{
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct clk_hw_onecell_data *clk_hw_data;
	struct clk_hw **hws;
	int i;

	clk_hw_data = devm_kzalloc(dev, struct_size(clk_hw_data, hws,
				dev_data->hws_num), GFP_KERNEL);
	if (WARN_ON(!clk_hw_data))
		return -ENOMEM;

	clk_hw_data->num = dev_data->clocks_max;
	hws = clk_hw_data->hws;

	for (i = 0; i < dev_data->hws_num; i++) {
		struct imx_blk_ctrl_hw *hw = &dev_data->hws[i];
		struct clk_hw *tmp = imx_blk_ctrl_register_one_clock(dev, hw);

		if (!tmp)
			continue;
		hws[hw->id] = tmp;
	}

	imx_check_clk_hws(hws, dev_data->clocks_max);

	return of_clk_add_hw_provider(dev->of_node, of_clk_hw_onecell_get,
					clk_hw_data);
}

static int imx_blk_ctrl_init_runtime_pm_safekeeping(struct device *dev)
{
	const struct imx_blk_ctrl_dev_data *dev_data = of_device_get_match_data(dev);
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct pm_safekeep_info *pm_info = &drvdata->pm_info;
	u32 regs_num = dev_data->pm_runtime_saved_regs_num;
	const u32 *regs_offsets = dev_data->pm_runtime_saved_regs;

	if (!dev_data->pm_runtime_saved_regs_num)
		return 0;

	pm_info->regs_values = devm_kzalloc(dev,
					    sizeof(u32) * regs_num,
					    GFP_KERNEL);
	if (WARN_ON(IS_ERR(pm_info->regs_values)))
		return PTR_ERR(pm_info->regs_values);

	pm_info->regs_offsets = kmemdup(regs_offsets,
					regs_num * sizeof(u32), GFP_KERNEL);
	if (WARN_ON(IS_ERR(pm_info->regs_offsets)))
		return PTR_ERR(pm_info->regs_offsets);

	pm_info->regs_num = regs_num;

	return 0;
}

static int imx_blk_ctrl_probe(struct platform_device *pdev)
{
	struct imx_blk_ctrl_drvdata *drvdata;
	struct device *dev = &pdev->dev;
	int ret;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (WARN_ON(!drvdata))
		return -ENOMEM;

	drvdata->base = devm_platform_ioremap_resource(pdev, 0);
	if (WARN_ON(IS_ERR(drvdata->base)))
		return PTR_ERR(drvdata->base);

	dev_set_drvdata(dev, drvdata);

	ret = imx_blk_ctrl_init_runtime_pm_safekeeping(dev);
	if (ret)
		return ret;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	pm_runtime_forbid(dev);

	ret = imx_blk_ctrl_register_clock_controller(dev);
	if (ret) {
		pm_runtime_put(dev);
		return ret;
	}

	ret = imx_blk_ctrl_register_reset_controller(dev);

	pm_runtime_put(dev);

	return ret;
}

static void imx_blk_ctrl_read_write(struct device *dev, bool write)
{
	struct imx_blk_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	struct pm_safekeep_info *pm_info = &drvdata->pm_info;
	void __iomem *base = drvdata->base;
	unsigned long flags;
	int i;

	if (!pm_info->regs_num)
		return;

	spin_lock_irqsave(drvdata->lock, flags);

	for (i = 0; i < pm_info->regs_num; i++) {
		u32 offset = pm_info->regs_offsets[i];

		if (write)
			writel(pm_info->regs_values[i], base + offset);
		else
			pm_info->regs_values[i] = readl(base + offset);
	}

	spin_unlock_irqrestore(drvdata->lock, flags);

}

static int imx_blk_ctrl_runtime_suspend(struct device *dev)
{
	imx_blk_ctrl_read_write(dev, false);

	return 0;
}

static int imx_blk_ctrl_runtime_resume(struct device *dev)
{
	imx_blk_ctrl_read_write(dev, true);

	return 0;
}

static const struct dev_pm_ops imx_blk_ctrl_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_blk_ctrl_runtime_suspend,
			   imx_blk_ctrl_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
			   pm_runtime_force_resume)
};

static const struct of_device_id imx_blk_ctrl_of_match[] = {
	{
		.compatible = "fsl,imx8mp-audio-blk-ctrl",
		.data = &imx8mp_audio_blk_ctrl_dev_data
	},
	{
		.compatible = "fsl,imx8mp-media-blk-ctrl",
		.data = &imx8mp_media_blk_ctrl_dev_data
	},
	{
		.compatible = "fsl,imx8mp-hdmi-blk-ctrl",
		.data = &imx8mp_hdmi_blk_ctrl_dev_data
	},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_blk_ctrl_of_match);

static struct platform_driver imx_blk_ctrl_driver = {
	.probe = imx_blk_ctrl_probe,
	.driver = {
		.name = "imx-blk-ctrl",
		.of_match_table = of_match_ptr(imx_blk_ctrl_of_match),
		.pm = &imx_blk_ctrl_pm_ops,
	},
};
module_platform_driver(imx_blk_ctrl_driver);
MODULE_LICENSE("GPL v2");
