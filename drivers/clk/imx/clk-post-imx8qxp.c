/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <dt-bindings/clock/imx8qxp-clock.h>
#include <dt-bindings/soc/imx8_pd.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

#include <soc/imx8/imx8qxp/lpcg.h>
#include <soc/imx8/sc/sci.h>
#include "clk-imx8.h"

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)

static const char *mipi0_sels[] = {
	"dummy",
	"mipi0_pll_clk",
	"mipi0_pll_div2_clk",
	"dummy",
	"mipi0_lvds_bypass_clk",
};

static const char *mipi1_sels[] = {
	"dummy",
	"mipi1_pll_clk",
	"mipi1_pll_div2_clk",
	"dummy",
	"mipi1_lvds_bypass_clk",
};

static struct clk *clks[IMX8QXP_CLK_END];
static struct clk_onecell_data clk_data;

static const char *enet_sels[] = { "enet_25MHz", "enet_125MHz", };
static const char *enet0_rmii_tx_sels[] = { "enet0_ref_div", "dummy", };
static const char *enet1_rmii_tx_sels[] = { "enet1_ref_div", "dummy", };

static int imx8qxp_post_clk_probe(struct platform_device *pdev)
{
	struct device_node *ccm_node = pdev->dev.of_node;
	struct device_node *np_acm;
	void __iomem *base_acm;
	int i, ret;

	pr_info("***** imx8qxp_post_clocks_init *****\n");

	clks[IMX8QXP_I2C1_IPG_CLK]   = imx_clk_gate2_scu("i2c1_ipg_clk", "ipg_dma_clk_root", (void __iomem *)(LPI2C_1_LPCG), 16, FUNCTION_NAME(PD_DMA_I2C_1));
	clks[IMX8QXP_I2C1_DIV] = imx_clk_divider_scu("i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QXP_I2C1_CLK] = imx_clk_gate_scu("i2c1_clk", "i2c1_div", SC_R_I2C_1, SC_PM_CLK_PER, (void __iomem *)(LPI2C_1_LPCG), 0, 0);
	/* Display controller - DC0 SS */
	clks[IMX8QXP_DC0_PLL0_DIV] = imx_clk_divider_scu("dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QXP_DC0_PLL1_DIV] = imx_clk_divider_scu("dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QXP_DC0_PLL0_CLK] = imx_clk_gate_scu("dc0_pll0_clk", "dc0_pll0_div", SC_R_DC_0_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QXP_DC0_PLL1_CLK] = imx_clk_gate_scu("dc0_pll1_clk", "dc0_pll1_div", SC_R_DC_0_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QXP_DC0_DISP0_DIV] = imx_clk_divider_scu("dc0_disp0_div", SC_R_DC_0, SC_PM_CLK_MISC0);
	clks[IMX8QXP_DC0_DISP1_DIV] = imx_clk_divider_scu("dc0_disp1_div", SC_R_DC_0, SC_PM_CLK_MISC1);
	clks[IMX8QXP_DC0_DISP0_CLK] = imx_clk_gate_scu("dc0_disp0_clk", "dc0_disp0_div", SC_R_DC_0, SC_PM_CLK_MISC0, (void __iomem *)(DC_0_LPCG), 0, 0);
	clks[IMX8QXP_DC0_DISP1_CLK] = imx_clk_gate_scu("dc0_disp1_clk", "dc0_disp1_div", SC_R_DC_0, SC_PM_CLK_MISC1, (void __iomem *)(DC_0_LPCG), 4, 0);
	clks[IMX8QXP_DC0_BYPASS_0_DIV] = imx_clk_divider_scu("dc0_bypass0_div", SC_R_DC_0_VIDEO0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_DC0_BYPASS_1_DIV] = imx_clk_divider_scu("dc0_bypass1_div", SC_R_DC_0_VIDEO1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_DC0_PRG0_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg0_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x20), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG0_APB_CLK] = imx_clk_gate2_scu("dc0_prg0_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x20), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG1_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg1_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x24), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG1_APB_CLK] = imx_clk_gate2_scu("dc0_prg1_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x24), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG2_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg2_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x28), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG2_APB_CLK] = imx_clk_gate2_scu("dc0_prg2_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x28), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG3_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg3_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x34), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG3_APB_CLK] = imx_clk_gate2_scu("dc0_prg3_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x34), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG4_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg4_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x38), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG4_APB_CLK] = imx_clk_gate2_scu("dc0_prg4_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x38), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG5_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg5_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x3c), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG5_APB_CLK] = imx_clk_gate2_scu("dc0_prg5_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x3c), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG6_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg6_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x40), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG6_APB_CLK] = imx_clk_gate2_scu("dc0_prg6_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x40), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG7_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg7_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x44), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG7_APB_CLK] = imx_clk_gate2_scu("dc0_prg7_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x44), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG8_RTRAM_CLK] = imx_clk_gate2_scu("dc0_prg8_rtram_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x48), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_PRG8_APB_CLK] = imx_clk_gate2_scu("dc0_prg8_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x48), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR0_APB_CLK] = imx_clk_gate2_scu("dc0_dpr0_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x18), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR0_B_CLK] = imx_clk_gate2_scu("dc0_dpr0_b_clk", "axi_ext_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x18), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_RTRAM0_CLK] = imx_clk_gate2_scu("dc0_rtrm0_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x1C), 0, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR1_APB_CLK] = imx_clk_gate2_scu("dc0_dpr1_apb_clk", "cfg_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x2C), 16, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_DPR1_B_CLK] = imx_clk_gate2_scu("dc0_dpr1_b_clk", "axi_ext_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x2C), 20, FUNCTION_NAME(PD_DC_0));
	clks[IMX8QXP_DC0_RTRAM1_CLK] = imx_clk_gate2_scu("dc0_rtrm1_clk", "axi_int_dc_clk_root", (void __iomem *)(DC_0_LPCG + 0x30), 0, FUNCTION_NAME(PD_DC_0));

	/* Display interface - MIPI-LVDS SS */
	clks[IMX8QXP_MIPI0_BYPASS_CLK] = imx_clk_divider_scu("mipi0_bypass_clk", SC_R_MIPI_0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI0_PIXEL_DIV] = imx_clk_divider_scu("mipi0_pixel_div", SC_R_MIPI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI0_PIXEL_CLK] = imx_clk_gate_scu("mipi0_pixel_clk", "mipi0_pixel_div", SC_R_MIPI_0, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_LVDS_PIXEL_DIV] = imx_clk_divider_scu("mipi0_lvds_pixel_div", SC_R_LVDS_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_LVDS_PIXEL_CLK] = imx_clk_gate_scu("mipi0_lvds_pixel_clk", "mipi0_lvds_pixel_div", SC_R_LVDS_0, SC_PM_CLK_MISC2, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_LVDS_BYPASS_CLK] = imx_clk_divider_scu("mipi0_lvds_bypass_clk", SC_R_LVDS_0, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI0_LVDS_PHY_DIV] = imx_clk_divider_scu("mipi0_lvds_phy_div", SC_R_LVDS_0, SC_PM_CLK_MISC3);
	clks[IMX8QXP_MIPI0_LVDS_PHY_CLK] = imx_clk_gate_scu("mipi0_lvds_phy_clk", "mipi0_lvds_phy_div", SC_R_LVDS_0, SC_PM_CLK_MISC3, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_SEL] = imx_clk_mux2_scu("mipi0_dsi_tx_esc_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_DIV] = imx_clk_divider2_scu("mipi0_dsi_tx_esc_div", "mipi0_dsi_tx_esc_sel", SC_R_MIPI_0, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI0_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_tx_esc_clk", "mipi0_dsi_tx_esc_div", SC_R_MIPI_0, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_SEL] = imx_clk_mux2_scu("mipi0_dsi_rx_esc_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_DIV] = imx_clk_divider2_scu("mipi0_dsi_rx_esc_div", "mipi0_dsi_rx_esc_sel", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI0_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi0_dsi_rx_esc_clk", "mipi0_dsi_rx_esc_div", SC_R_MIPI_0, SC_PM_CLK_SLV_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_DSI_PHY_SEL] = imx_clk_mux2_scu("mipi0_dsi_phy_sel", mipi0_sels, ARRAY_SIZE(mipi0_sels), SC_R_MIPI_0, SC_PM_CLK_PHY);
	clks[IMX8QXP_MIPI0_DSI_PHY_DIV] = imx_clk_divider2_scu("mipi0_dsi_phy_div", "mipi0_dsi_phy_sel", SC_R_MIPI_0, SC_PM_CLK_PHY);
	clks[IMX8QXP_MIPI0_DSI_PHY_CLK] = imx_clk_gate_scu("mipi0_dsi_phy_clk", "mipi0_dsi_phy_div", SC_R_MIPI_0, SC_PM_CLK_PHY, NULL, 0, 0);
	clks[IMX8QXP_MIPI0_I2C0_DIV] = imx_clk_divider_scu("mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_I2C1_DIV] = imx_clk_divider_scu("mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI0_I2C0_CLK] = imx_clk_gate_scu("mipi0_i2c0_clk", "mipi0_i2c0_div", SC_R_MIPI_0_I2C_0, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI0_LPCG + 0x10), 0, 0);
	clks[IMX8QXP_MIPI0_I2C1_CLK] = imx_clk_gate_scu("mipi0_i2c1_clk", "mipi0_i2c1_div", SC_R_MIPI_0_I2C_1, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI0_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c0_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_0_DSI_I2C0));
	clks[IMX8QXP_MIPI0_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi0_i2c1_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_0_DSI_I2C1));
	clks[IMX8QXP_MIPI0_PWM_IPG_CLK] = imx_clk_gate2_scu("mipi0_pwm_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0xC), 16, FUNCTION_NAME(PD_MIPI_0_DSI_PWM0));
	clks[IMX8QXP_MIPI0_PWM_32K_CLK] = imx_clk_gate2_scu("mipi0_pwm_32K_clk", "xtal_32KHz", (void __iomem *)(DI_MIPI0_LPCG + 0xC), 4, FUNCTION_NAME(PD_MIPI_0_DSI_PWM0));
	clks[IMX8QXP_MIPI0_PWM_DIV] = imx_clk_divider_scu("mipi0_pwm_div", SC_R_MIPI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI0_PWM_CLK] = imx_clk_gate_scu("mipi0_pwm_clk", "mipi0_pwm_div", SC_R_MIPI_0_PWM_0, SC_PM_CLK_PER, (void __iomem *)(DI_MIPI0_LPCG + 0xC), 0, 0);
	clks[IMX8QXP_MIPI0_GPIO_IPG_CLK] = imx_clk_gate2_scu("mipi0_gpio_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x8), 16, FUNCTION_NAME(PD_MIPI_0_GPIO_0));
	clks[IMX8QXP_MIPI0_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi0_lis_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI0_LPCG + 0x0), 16, FUNCTION_NAME(PD_MIPI_0_DSI));
	clks[IMX8QXP_MIPI1_BYPASS_CLK] = imx_clk_divider_scu("mipi1_bypass_clk", SC_R_MIPI_1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI1_PIXEL_DIV] = imx_clk_divider_scu("mipi1_pixel_div", SC_R_MIPI_1, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI1_PIXEL_CLK] = imx_clk_gate_scu("mipi1_pixel_clk", "mipi1_pixel_div", SC_R_MIPI_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_LVDS_PIXEL_DIV] = imx_clk_divider_scu("mipi1_lvds_pixel_div", SC_R_LVDS_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_LVDS_PIXEL_CLK] = imx_clk_gate_scu("mipi1_lvds_pixel_clk", "mipi1_lvds_pixel_div", SC_R_LVDS_1, SC_PM_CLK_MISC2, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_LVDS_BYPASS_CLK] = imx_clk_divider_scu("mipi1_lvds_bypass_clk", SC_R_LVDS_1, SC_PM_CLK_BYPASS);
	clks[IMX8QXP_MIPI1_LVDS_PHY_DIV] = imx_clk_divider_scu("mipi1_lvds_phy_div", SC_R_LVDS_1, SC_PM_CLK_MISC3);
	clks[IMX8QXP_MIPI1_LVDS_PHY_CLK] = imx_clk_gate_scu("mipi1_lvds_phy_clk", "mipi1_lvds_phy_div", SC_R_LVDS_1, SC_PM_CLK_MISC3, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_SEL] = imx_clk_mux2_scu("mipi1_dsi_tx_esc_sel", mipi1_sels, ARRAY_SIZE(mipi1_sels), SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_DIV] = imx_clk_divider2_scu("mipi1_dsi_tx_esc_div", "mipi1_dsi_tx_esc_sel", SC_R_MIPI_1, SC_PM_CLK_MST_BUS);
	clks[IMX8QXP_MIPI1_DSI_TX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_tx_esc_clk", "mipi1_dsi_tx_esc_div", SC_R_MIPI_1, SC_PM_CLK_MST_BUS, NULL, 0, 0);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_SEL] = imx_clk_mux2_scu("mipi1_dsi_rx_esc_sel", mipi1_sels, ARRAY_SIZE(mipi1_sels), SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_DIV] = imx_clk_divider2_scu("mipi1_dsi_rx_esc_div", "mipi1_dsi_rx_esc_sel", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS);
	clks[IMX8QXP_MIPI1_DSI_RX_ESC_CLK] = imx_clk_gate_scu("mipi1_dsi_rx_esc_clk", "mipi1_dsi_rx_esc_div", SC_R_MIPI_1, SC_PM_CLK_SLV_BUS, NULL, 0, 0);

	clks[IMX8QXP_MIPI1_I2C0_DIV] = imx_clk_divider_scu("mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_I2C1_DIV] = imx_clk_divider_scu("mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2);
	clks[IMX8QXP_MIPI1_I2C0_CLK] = imx_clk_gate_scu("mipi1_i2c0_clk", "mipi1_i2c0_div", SC_R_MIPI_1_I2C_0, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI1_LPCG + 0x10), 0, 0);
	clks[IMX8QXP_MIPI1_I2C1_CLK] = imx_clk_gate_scu("mipi1_i2c1_clk", "mipi1_i2c1_div", SC_R_MIPI_1_I2C_1, SC_PM_CLK_MISC2, (void __iomem *)(DI_MIPI1_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_MIPI1_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c0_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_1_DSI_I2C0));
	clks[IMX8QXP_MIPI1_I2C1_IPG_CLK] = imx_clk_gate2_scu("mipi1_i2c1_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_1_DSI_I2C1));
	clks[IMX8QXP_MIPI1_PWM_IPG_CLK] = imx_clk_gate2_scu("mipi1_pwm_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0xC), 16, FUNCTION_NAME(PD_MIPI_1_DSI_PWM0));
	clks[IMX8QXP_MIPI1_PWM_32K_CLK] = imx_clk_gate2_scu("mipi1_pwm_32K_clk", "xtal_32KHz", (void __iomem *)(DI_MIPI1_LPCG + 0xC), 4, FUNCTION_NAME(PD_MIPI_1_DSI_PWM0));
	clks[IMX8QXP_MIPI1_PWM_DIV] = imx_clk_divider_scu("mipi1_pwm_div", SC_R_MIPI_1_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_MIPI1_PWM_CLK] = imx_clk_gate_scu("mipi1_pwm_clk", "mipi1_pwm_div", SC_R_MIPI_1_PWM_0, SC_PM_CLK_PER, (void __iomem *)(DI_MIPI1_LPCG + 0xC), 0, 0);
	clks[IMX8QXP_MIPI1_GPIO_IPG_CLK] = imx_clk_gate2_scu("mipi1_gpio_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x8), 16, FUNCTION_NAME(PD_MIPI_1_GPIO_0));
	clks[IMX8QXP_MIPI1_LIS_IPG_CLK] = imx_clk_gate2_scu("mipi1_lis_ipg_clk", "ipg_mipi_clk_root", (void __iomem *)(DI_MIPI1_LPCG + 0x0), 16, FUNCTION_NAME(PD_MIPI_1_DSI));

	/* MIPI CSI SS */
	clks[IMX8QXP_CSI0_I2C0_DIV] = imx_clk_divider_scu("mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_PWM0_DIV] = imx_clk_divider_scu("mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_CORE_DIV] = imx_clk_divider_scu("mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER);
	clks[IMX8QXP_CSI0_ESC_DIV] = imx_clk_divider_scu("mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC);
	clks[IMX8QXP_CSI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_i2c0_ipg_s", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_CSI0_I2C0));
	clks[IMX8QXP_CSI0_I2C0_CLK] = imx_clk_gate_scu("mipi_csi0_i2c0_clk", "mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x14), 0, 0);
	clks[IMX8QXP_CSI0_PWM0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_pwm0_ipg_s", "ipg_mipi_csi_clk_root", (void __iomem *)(MIPI_CSI_0_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_CSI0_PWM));
	clks[IMX8QXP_CSI0_PWM0_CLK] = imx_clk_gate_scu("mipi_csi0_pwm0_clk", "mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x10), 0, 0);
	clks[IMX8QXP_CSI0_CORE_CLK] = imx_clk_gate_scu("mipi_csi0_core_clk", "mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER, (void __iomem *)(MIPI_CSI_0_LPCG + 0x18), 16, 0);
	clks[IMX8QXP_CSI0_ESC_CLK] = imx_clk_gate_scu("mipi_csi0_esc_clk", "mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC, (void __iomem *)(MIPI_CSI_0_LPCG + 0x1C), 16, 0);

	/* CM40 */
	clks[IMX8QXP_CM40_I2C_DIV]     = imx_clk_divider_scu("cm40_i2c_div", SC_R_M4_0_I2C, SC_PM_CLK_PER);
	clks[IMX8QXP_CM40_I2C_CLK]     = imx_clk_gate_scu("cm40_i2c_clk", "cm40_i2c_div", SC_R_M4_0_I2C, SC_PM_CLK_PER, (void __iomem *)(CM40_I2C_LPCG), 0, 0);
	clks[IMX8QXP_CM40_I2C_IPG_CLK] = imx_clk_gate2_scu("cm40_i2c_ipg_clk", "ipg_cm40_clk_root", (void __iomem *)(CM40_I2C_LPCG), 16, FUNCTION_NAME(PD_CM40_I2C));


	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(ccm_node, of_clk_src_onecell_get, &clk_data);

	return 0;
}

static const struct of_device_id imx8qxp_match[] = {
	{ .compatible = "fsl,imx8qxp-post-clk", },
	{ /* sentinel value */ }
};

static struct platform_driver imx8qxp_post_clk_driver = {
	.driver = {
		.name = "imx8qxp-post-clk",
		.of_match_table = imx8qxp_match,
	},
	.probe = imx8qxp_post_clk_probe,
};

static int __init imx8qxp_post_clk_init(void)
{
	return platform_driver_register(&imx8qxp_post_clk_driver);
}
core_initcall(imx8qxp_post_clk_init);
