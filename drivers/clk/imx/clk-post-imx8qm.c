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

#include <dt-bindings/clock/imx8qm-clock.h>
#include <dt-bindings/soc/imx8_pd.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-conf.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/types.h>

#include <soc/imx8/imx8qm/lpcg.h>
#include <soc/imx8/sc/sci.h>

#include "clk-imx8.h"

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)

static const char *dc1_sels[] = {
	"dummy",
	"dummy",
	"dc1_pll0_clk",
	"dc1_pll1_clk",
	"dc1_bypass0_div",
};

static struct clk *clks[IMX8QM_CLK_END];
static struct clk_onecell_data clk_data;

static const char *enet_sels[] = {"enet_25MHz", "enet_125MHz",};
static const char *enet0_rmii_tx_sels[] = {"enet0_ref_div", "dummy",};
static const char *enet1_rmii_tx_sels[] = {"enet1_ref_div", "dummy",};

#define LPCG_ADDR(arg) ((void __iomem *)(base_lpcg + arg))

static int imx8qm_post_clk_probe(struct platform_device *pdev)
{
	struct device_node *ccm_node = pdev->dev.of_node;
	struct device_node *np_acm;
	void __iomem *base_acm;
	u64 base_lpcg = 0;
	int i, ret;

	pr_info("***** imx8qm_post_clocks_init *****\n");

	/* Parse lpcg_base_offset for virtualization cases */
	ret = of_property_read_u64(ccm_node, "fsl,lpcg_base_offset", &base_lpcg);
	if (ret && ret != -EINVAL) {
		dev_err(&pdev->dev, "failed to parse fsl,lpcg_base_offset: %d\n", ret);
		return ret;
	}

	clks[IMX8QM_DC1_PLL0_DIV] = imx_clk_divider_scu("dc1_pll0_div", SC_R_DC_1_PLL_0, SC_PM_CLK_PLL);
	clks[IMX8QM_DC1_PLL1_DIV] = imx_clk_divider_scu("dc1_pll1_div", SC_R_DC_1_PLL_1, SC_PM_CLK_PLL);
	clks[IMX8QM_DC1_PLL0_CLK] = imx_clk_gate_scu("dc1_pll0_clk", "dc1_pll0_div", SC_R_DC_1_PLL_0, SC_PM_CLK_PLL, NULL, 0, 0);
	clks[IMX8QM_DC1_PLL1_CLK] = imx_clk_gate_scu("dc1_pll1_clk", "dc1_pll1_div", SC_R_DC_1_PLL_1, SC_PM_CLK_PLL, NULL, 0, 0);

	clks[IMX8QM_LVDS1_BYPASS_CLK] = imx_clk_divider_scu("lvds1_bypass_clk", SC_R_LVDS_1, SC_PM_CLK_BYPASS);
	clks[IMX8QM_LVDS1_PIXEL_DIV] = imx_clk_divider_scu("lvds1_pixel_div", SC_R_LVDS_1, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_I2C0_DIV] = imx_clk_divider_scu("lvds1_i2c0_div", SC_R_LVDS_1_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_I2C1_DIV] = imx_clk_divider_scu("lvds1_i2c1_div", SC_R_LVDS_1_I2C_1, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_PWM0_DIV] = imx_clk_divider_scu("lvds1_pwm0_div", SC_R_LVDS_1_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_LVDS1_PHY_DIV] = imx_clk_divider_scu("lvds1_phy_div", SC_R_LVDS_1, SC_PM_CLK_PHY);

	clks[IMX8QM_CSI0_I2C0_DIV] = imx_clk_divider_scu("mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_PWM0_DIV] = imx_clk_divider_scu("mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_CORE_DIV] = imx_clk_divider_scu("mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER);
	clks[IMX8QM_CSI0_ESC_DIV] = imx_clk_divider_scu("mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC);

	clks[IMX8QM_DC1_DISP0_SEL] = imx_clk_mux2_scu("dc1_disp0_sel", dc1_sels, ARRAY_SIZE(dc1_sels), SC_R_DC_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC1_DISP0_DIV] = imx_clk_divider2_scu("dc1_disp0_div", "dc1_disp0_sel", SC_R_DC_1, SC_PM_CLK_MISC0);
	clks[IMX8QM_DC1_DISP1_SEL] = imx_clk_mux2_scu("dc1_disp1_sel", dc1_sels, ARRAY_SIZE(dc1_sels), SC_R_DC_1, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC1_DISP1_DIV] = imx_clk_divider2_scu("dc1_disp1_div", "dc1_disp1_sel", SC_R_DC_1, SC_PM_CLK_MISC1);
	clks[IMX8QM_DC1_BYPASS_0_DIV] = imx_clk_divider_scu("dc1_bypass0_div", SC_R_DC_1_VIDEO0, SC_PM_CLK_BYPASS);
	clks[IMX8QM_DC1_BYPASS_1_DIV] = imx_clk_divider_scu("dc1_bypass1_div", SC_R_DC_1_VIDEO1, SC_PM_CLK_BYPASS);

	/* MIPI CSI */
	clks[IMX8QM_CSI0_I2C0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_i2c0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x14), 16, FUNCTION_NAME(PD_MIPI_CSI0_I2C0));
	clks[IMX8QM_CSI0_I2C0_CLK] = imx_clk_gate_scu("mipi_csi0_i2c0_clk", "mipi_csi0_i2c0_div", SC_R_CSI_0_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x14), 0, 0);
	clks[IMX8QM_CSI0_PWM0_IPG_CLK] = imx_clk_gate2_scu("mipi_csi0_pwm0_ipg_s", "ipg_mipi_csi_clk_root", LPCG_ADDR(MIPI_CSI_0_LPCG + 0x10), 16, FUNCTION_NAME(PD_MIPI_CSI0_PWM));
	clks[IMX8QM_CSI0_PWM0_CLK] = imx_clk_gate_scu("mipi_csi0_pwm0_clk", "mipi_csi0_pwm0_div", SC_R_CSI_0_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x10), 0, 0);
	clks[IMX8QM_CSI0_CORE_CLK] = imx_clk_gate_scu("mipi_csi0_core_clk", "mipi_csi0_core_div", SC_R_CSI_0, SC_PM_CLK_PER, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x18), 16, 0);
	clks[IMX8QM_CSI0_ESC_CLK] = imx_clk_gate_scu("mipi_csi0_esc_clk", "mipi_csi0_esc_div", SC_R_CSI_0, SC_PM_CLK_MISC, LPCG_ADDR(MIPI_CSI_0_LPCG + 0x1C), 16, 0);

	/* DC1 */
	clks[IMX8QM_DC1_DISP0_CLK] = imx_clk_gate_scu("dc1_disp0_clk", "dc1_disp0_div", SC_R_DC_1, SC_PM_CLK_MISC0, LPCG_ADDR(DC_1_LPCG), 0, 0);
	clks[IMX8QM_DC1_DISP1_CLK] = imx_clk_gate_scu("dc1_disp1_clk", "dc1_disp1_div", SC_R_DC_1, SC_PM_CLK_MISC1, LPCG_ADDR(DC_1_LPCG), 4, 0);
	clks[IMX8QM_DC1_PRG0_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg0_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x20), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG0_APB_CLK] = imx_clk_gate2_scu("dc1_prg0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x20), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG1_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg1_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x24), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG1_APB_CLK] = imx_clk_gate2_scu("dc1_prg1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x24), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG2_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg2_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x28), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG2_APB_CLK] = imx_clk_gate2_scu("dc1_prg2_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x28), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG3_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg3_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x34), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG3_APB_CLK] = imx_clk_gate2_scu("dc1_prg3_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x34), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG4_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg4_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x38), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG4_APB_CLK] = imx_clk_gate2_scu("dc1_prg4_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x38), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG5_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg5_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x3c), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG5_APB_CLK] = imx_clk_gate2_scu("dc1_prg5_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x3c), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG6_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg6_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x40), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG6_APB_CLK] = imx_clk_gate2_scu("dc1_prg6_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x40), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG7_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg7_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x44), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG7_APB_CLK] = imx_clk_gate2_scu("dc1_prg7_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x44), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG8_RTRAM_CLK] = imx_clk_gate2_scu("dc1_prg8_rtram_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x48), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_PRG8_APB_CLK] = imx_clk_gate2_scu("dc1_prg8_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x48), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR0_APB_CLK] = imx_clk_gate2_scu("dc1_dpr0_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x18), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR0_B_CLK] = imx_clk_gate2_scu("dc1_dpr0_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x18), 20, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR1_APB_CLK] = imx_clk_gate2_scu("dc1_dpr1_apb_clk", "cfg_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x2c), 16, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_DPR1_B_CLK] = imx_clk_gate2_scu("dc1_dpr1_b_clk", "axi_ext_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x2c), 20, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_RTRAM0_CLK] = imx_clk_gate2_scu("dc1_rtrm0_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x1C), 0, FUNCTION_NAME(PD_DC_1));
	clks[IMX8QM_DC1_RTRAM1_CLK] = imx_clk_gate2_scu("dc1_rtrm1_clk", "axi_int_dc_clk_root", LPCG_ADDR(DC_1_LPCG + 0x30), 0, FUNCTION_NAME(PD_DC_1));

	clks[IMX8QM_LVDS1_PIXEL_CLK] = imx_clk_gate_scu("lvds1_pixel_clk", "lvds1_pixel_div", SC_R_LVDS_1, SC_PM_CLK_PER, NULL, 0, 0);
	clks[IMX8QM_LVDS1_I2C0_CLK] = imx_clk_gate_scu("lvds1_i2c0_clk", "lvds1_i2c0_div", SC_R_LVDS_1_I2C_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x10), 0, 0);
	clks[IMX8QM_LVDS1_I2C1_CLK] = imx_clk_gate_scu("lvds1_i2c1_clk", "lvds1_i2c1_div", SC_R_LVDS_1_I2C_1, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x14), 0, 0);
	clks[IMX8QM_LVDS1_PWM0_CLK] = imx_clk_gate_scu("lvds1_pwm0_clk", "lvds1_pwm0_div", SC_R_LVDS_1_PWM_0, SC_PM_CLK_PER, LPCG_ADDR(DI_LVDS_1_LPCG + 0x0C), 0, 0);
	clks[IMX8QM_LVDS1_PHY_CLK] = imx_clk_gate_scu("lvds1_phy_clk", "lvds1_phy_div", SC_R_LVDS_1, SC_PM_CLK_PHY, NULL, 0, 0);
	clks[IMX8QM_LVDS1_I2C0_IPG_CLK] = imx_clk_gate2_scu("lvds1_i2c0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x10), 16, FUNCTION_NAME(PD_LVDS1_I2C0));
	clks[IMX8QM_LVDS1_I2C1_IPG_CLK] = imx_clk_gate2_scu("lvds1_i2c1_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x14), 16, FUNCTION_NAME(PD_LVDS1_I2C1));
	clks[IMX8QM_LVDS1_PWM0_IPG_CLK] = imx_clk_gate2_scu("lvds1_pwm0_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x0C), 16, FUNCTION_NAME(PD_LVDS1_PWM));
	clks[IMX8QM_LVDS1_GPIO_IPG_CLK] = imx_clk_gate2_scu("lvds1_gpio_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x08), 16, FUNCTION_NAME(PD_LVDS1_GPIO));
	clks[IMX8QM_LVDS1_LIS_IPG_CLK] = imx_clk_gate2_scu("lvds1_lis_ipg_clk", "ipg_lvds_clk_root", LPCG_ADDR(DI_LVDS_1_LPCG + 0x0), 16, FUNCTION_NAME(PD_LVDS1));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	of_clk_add_provider(ccm_node, of_clk_src_onecell_get, &clk_data);
}

static const struct of_device_id imx8qm_match[] = {
	{ .compatible = "fsl,imx8qm-post-clk", },
	{ /* sentinel value */ }
};

static struct platform_driver imx8qm_post_clk_driver = {
	.driver = {
		.name = "imx8qm-post-clk",
		.of_match_table = imx8qm_match,
	},
	.probe = imx8qm_post_clk_probe,
};

static int __init imx8qm_post_clk_init(void)
{
	return platform_driver_register(&imx8qm_post_clk_driver);
}
core_initcall(imx8qm_post_clk_init);
