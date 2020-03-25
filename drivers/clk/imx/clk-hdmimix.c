// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 NXP.
 */

#include <dt-bindings/clock/imx8mp-clock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>

#include "clk.h"

static struct clk *clks[IMX8MP_CLK_HDMIMIX_END];
static struct clk_onecell_data clk_data;
static uint32_t hdmimix_clk_saved_regs[2];

static const char *imx_hdmi_phy_clks_sels[] = { "hdmi_glb_24m", "dummy",};
static const char *imx_lcdif_clks_sels[] = { "dummy", "hdmi_glb_pix", };
static const char *imx_hdmi_pipe_clks_sels[] = {"dummy","hdmi_glb_pix", };

static int imx_hdmimix_clk_suspend(struct device *dev)
{
	void __iomem *base;

	base = dev_get_drvdata(dev->parent);

	hdmimix_clk_saved_regs[0] = readl(base + 0x40);
	hdmimix_clk_saved_regs[1] = readl(base + 0x50);

	return 0;
}

static int imx_hdmimix_clk_resume(struct device *dev)
{
	void __iomem *base;

	base = dev_get_drvdata(dev->parent);

	writel(hdmimix_clk_saved_regs[0], base + 0x40);
	writel(hdmimix_clk_saved_regs[1], base + 0x50);

	return 0;
}

static int imx_hdmimix_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *base;
	struct clk *clk_dummy;
	int i;
	int ret;

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	/* Defer until CCM driver is probed */
	clk_dummy = of_clk_get_by_name(np, "dummy");
	if (IS_ERR(clk_dummy))
		return PTR_ERR(clk_dummy);

	base = dev_get_drvdata(dev->parent);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_APB_CLK]	   = imx_dev_clk_gate(dev, "hdmi_glb_apb",     "hdmi_apb",      base + 0x40, 0);
	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_B_CLK]      = imx_dev_clk_gate(dev, "hdmi_glb_b",       "hdmi_axi",      base + 0x40, 1);
	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_REF266M_CLK]= imx_dev_clk_gate(dev, "hdmi_glb_ref_266m","hdmi_ref_266m", base + 0x40, 2);
	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_XTAL24M_CLK]= imx_dev_clk_gate(dev, "hdmi_glb_24m",     "hdmi_24m",      base + 0x40, 4);
	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_XTAL32K_CLK]= imx_dev_clk_gate(dev, "hdmi_glb_32k",     "osc_32k",       base + 0x40, 5);
	clks[IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK] = imx_dev_clk_gate(dev, "hdmi_glb_pix",     "hdmi_phy",      base + 0x40, 7);
	clks[IMX8MP_CLK_HDMIMIX_IRQS_STEER_CLK]	   = imx_dev_clk_gate(dev, "hdmi_irq_steer",   "hdmi_glb_apb",  base + 0x40, 9);
	clks[IMX8MP_CLK_HDMIMIX_NOC_HDMI_CLK]      = imx_dev_clk_gate(dev, "hdmi_noc",         "hdmi_glb_apb",  base + 0x40, 10);
	clks[IMX8MP_CLK_HDMIMIX_NOC_HDCP_CLK]      = imx_dev_clk_gate(dev, "hdcp_noc",         "hdmi_glb_apb",  base + 0x40, 11);
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_APB_CLK]     = imx_dev_clk_gate(dev, "lcdif3_apb",       "hdmi_glb_apb",  base + 0x40, 16);
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_B_CLK]       = imx_dev_clk_gate(dev, "lcdif3_b",         "hdmi_glb_b",    base + 0x40, 17);
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_PDI_CLK]     = imx_dev_clk_gate(dev, "lcdif3_pdi",       "hdmi_glb_apb",  base + 0x40, 18);
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_PIX_CLK]     = imx_dev_clk_gate(dev, "lcdif3_pxl",       "hdmi_glb_pix",  base + 0x40, 19);
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_SPU_CLK]     = imx_dev_clk_gate(dev, "lcdif3_spu",       "hdmi_glb_apb",  base + 0x40, 20);

	clks[IMX8MP_CLK_HDMIMIX_FDCC_REF_CLK]      = imx_dev_clk_gate(dev, "hdmi_fdcc_ref",    "hdmi_fdcc_tst", base + 0x50, 2);
	clks[IMX8MP_CLK_HDMIMIX_HRV_MWR_APB_CLK]   = imx_dev_clk_gate(dev, "hrv_mwr_apb",       "hdmi_glb_apb", base + 0x50, 3);
	clks[IMX8MP_CLK_HDMIMIX_HRV_MWR_B_CLK]     = imx_dev_clk_gate(dev, "hrv_mwr_b",         "hdmi_glb_axi", base + 0x50, 4);
	clks[IMX8MP_CLK_HDMIMIX_HRV_MWR_CEA_CLK]   = imx_dev_clk_gate(dev, "hrv_mwr_cea",       "hdmi_glb_apb", base + 0x50, 5);
	clks[IMX8MP_CLK_HDMIMIX_VSFD_CEA_CLK]      = imx_dev_clk_gate(dev, "vsfd_cea",          "hdmi_glb_apb", base + 0x50, 6);
	clks[IMX8MP_CLK_HDMIMIX_TX_HPI_CLK]        = imx_dev_clk_gate(dev, "hdmi_tx_hpi",       "hdmi_glb_apb", base + 0x50, 13);
	clks[IMX8MP_CLK_HDMIMIX_TX_APB_CLK]        = imx_dev_clk_gate(dev, "hdmi_tx_apb",       "hdmi_glb_apb", base + 0x50, 14);
	clks[IMX8MP_CLK_HDMIMIX_TX_CEC_CLK]        = imx_dev_clk_gate(dev, "hdmi_cec",          "hdmi_glb_apb", base + 0x50, 15);
	clks[IMX8MP_CLK_HDMIMIX_TX_ESM_CLK]        = imx_dev_clk_gate(dev, "hdmi_esm",     "hdmi_glb_ref_266m", base + 0x50, 16);
	clks[IMX8MP_CLK_HDMIMIX_TX_GPA_CLK]        = imx_dev_clk_gate(dev, "hdmi_tx_gpa",       "hdmi_glb_apb", base + 0x50, 17);
	clks[IMX8MP_CLK_HDMIMIX_TX_PIXEL_CLK]      = imx_dev_clk_gate(dev, "hdmi_tx_pix",       "hdmi_glb_pix", base + 0x50, 18);
	clks[IMX8MP_CLK_HDMIMIX_TX_SFR_CLK]        = imx_dev_clk_gate(dev, "hdmi_tx_sfr",       "hdmi_glb_apb", base + 0x50, 19);
	clks[IMX8MP_CLK_HDMIMIX_TX_SKP_CLK]        = imx_dev_clk_gate(dev, "hdmi_tx_skp",       "hdmi_glb_apb", base + 0x50, 20);
	clks[IMX8MP_CLK_HDMIMIX_TX_PREP_CLK]       = imx_dev_clk_gate(dev, "hdmi_tx_prep",      "hdmi_glb_apb", base + 0x50, 21);
	clks[IMX8MP_CLK_HDMIMIX_TX_PHY_APB_CLK]	   = imx_dev_clk_gate(dev, "hdmi_phy_apb",      "hdmi_glb_apb", base + 0x50, 22);
	clks[IMX8MP_CLK_HDMIMIX_TX_PHY_INT_CLK]	   = imx_dev_clk_gate(dev, "hdmi_phy_int",      "hdmi_glb_apb", base + 0x50, 24);
	clks[IMX8MP_CLK_HDMIMIX_TX_SEC_MEM_CLK]	   = imx_dev_clk_gate(dev, "hdmi_sec_mem", "hdmi_glb_ref_266m", base + 0x50, 25);
	clks[IMX8MP_CLK_HDMIMIX_TX_TRNG_SKP_CLK]   = imx_dev_clk_gate(dev, "hdmi_trng_skp",     "hdmi_glb_apb", base + 0x50, 27);
	clks[IMX8MP_CLK_HDMIMIX_TX_VID_LINK_PIX_CLK]= imx_dev_clk_gate(dev, "hdmi_vid_pix",     "hdmi_glb_pix", base + 0x50, 28);
	clks[IMX8MP_CLK_HDMIMIX_TX_TRNG_APB_CLK]   = imx_dev_clk_gate(dev, "hdmi_trng_apb",     "hdmi_glb_apb", base + 0x50, 30);

	clks[IMX8MP_CLK_HDMIMIX_HTXPHY_CLK_SEL]    = imx_dev_clk_mux(dev, "hdmi_phy_sel", base + 0x50, 10, 1, imx_hdmi_phy_clks_sels, ARRAY_SIZE(imx_hdmi_phy_clks_sels));
	clks[IMX8MP_CLK_HDMIMIX_LCDIF_CLK_SEL]     = imx_dev_clk_mux(dev, "lcdif_clk_sel", base + 0x50, 11, 1, imx_lcdif_clks_sels, ARRAY_SIZE(imx_hdmi_phy_clks_sels));
	clks[IMX8MP_CLK_HDMIMIX_TX_PIPE_CLK_SEL]   = imx_dev_clk_mux(dev, "hdmi_pipe_sel", base + 0x50, 12, 1, imx_hdmi_pipe_clks_sels, ARRAY_SIZE(imx_hdmi_pipe_clks_sels));

	/* hdmi/lcdif pixel clock parent to hdmi phy */
	clk_set_parent(clks[IMX8MP_CLK_HDMIMIX_TX_PIPE_CLK_SEL], clks[IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK]);
	clk_set_parent(clks[IMX8MP_CLK_HDMIMIX_LCDIF_CLK_SEL], clks[IMX8MP_CLK_HDMIMIX_GLOBAL_TX_PIX_CLK]);
	/* hdmi ref clock from 24MHz */
	clk_set_parent(clks[IMX8MP_CLK_HDMIMIX_HTXPHY_CLK_SEL], clks[IMX8MP_CLK_HDMIMIX_GLOBAL_XTAL24M_CLK]);

	imx_check_clocks(clks, ARRAY_SIZE(clks));

	clk_data.clks = clks;
	clk_data.clk_num = ARRAY_SIZE(clks);
	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (ret < 0) {
		pr_err("failed to register clks for i.MX8MP\n");
		return -EINVAL;
	}
	for (i = 0; i < IMX8MP_CLK_HDMIMIX_END; i++) {
		clk_prepare_enable(clks[i]);
	}

	pm_runtime_put(dev);

	return 0;
}

UNIVERSAL_DEV_PM_OPS(imx_hdmimix_clk_pm_ops, imx_hdmimix_clk_suspend,
			imx_hdmimix_clk_resume, NULL);

static const struct of_device_id imx_hdmimix_clk_of_match[] = {
	{ .compatible = "fsl,imx8mp-hdmimix-clk" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx_hdmimix_clk_of_match);


static struct platform_driver imx_hdmimix_clk_driver = {
	.probe = imx_hdmimix_clk_probe,
	.driver = {
		.name = "imx-hdmimix-clk",
		.of_match_table = of_match_ptr(imx_hdmimix_clk_of_match),
		.pm = &imx_hdmimix_clk_pm_ops,
	},
};
module_platform_driver(imx_hdmimix_clk_driver);
