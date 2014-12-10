/*
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "clk.h"
#include "common.h"
#include "hardware.h"

#define CCM_CCGR_OFFSET(index)	(index * 2)

static const char *step_sels[]	= { "osc", "pll2_pfd2_396m", };
static const char *pll1_sw_sels[]	= { "pll1_sys", "step", };
static const char *periph_pre_sels[]	= { "pll2_bus", "pll2_pfd2_396m", "pll2_pfd0_352m", "pll2_198m", };
static const char *periph_clk2_sels[]	= { "pll3_usb_otg", "osc", "osc", "dummy", };
static const char *periph2_clk2_sels[]	= { "pll3_usb_otg", "pll2_bus", };
static const char *periph_sels[]	= { "periph_pre", "periph_clk2", };
static const char *periph2_sels[]	= { "periph2_pre", "periph2_clk2", };
static const char *axi_alt_sels[]	= { "pll2_pfd2_396m", "pll3_pfd1_540m", };
static const char *axi_sels[]		= { "periph", "axi_alt_sel", };
static const char *audio_sels[]	= { "pll4_audio_div", "pll3_pfd2_508m", "pll3_pfd3_454m", "pll3_usb_otg", };
static const char *gpu_axi_sels[]	= { "axi", "ahb", };
static const char *gpu2d_core_sels[]	= { "axi", "pll3_usb_otg", "pll2_pfd0_352m", "pll2_pfd2_396m", };
static const char *gpu3d_core_sels[]	= { "mmdc_ch0_axi_podf", "pll3_usb_otg", "pll2_pfd1_594m", "pll2_pfd2_396m", };
static const char *gpu3d_shader_sels[] = { "mmdc_ch0_axi_podf", "pll3_usb_otg", "pll2_pfd1_594m", "pll3_pfd0_720m", };
static const char *ipu_sels[]		= { "mmdc_ch0_axi_podf", "pll2_pfd2_396m", "pll3_120m", "pll3_pfd1_540m", };
static const char *ldb_di_sels[]	= { "pll5_video_div", "pll2_pfd0_352m", "pll2_pfd2_396m", "mmdc_ch1_axi", "pll3_usb_otg", };
static const char *ldb_di0_div_sels[]	= { "ldb_di0_div_3_5", "ldb_di0_div_7", };
static const char *ldb_di1_div_sels[]	= { "ldb_di1_div_3_5", "ldb_di1_div_7", };
static const char *ipu_di_pre_sels[]	= { "mmdc_ch0_axi_podf", "pll3_usb_otg", "pll5_video_div", "pll2_pfd0_352m", "pll2_pfd2_396m", "pll3_pfd1_540m", };
static const char *ipu1_di0_sels[]	= { "ipu1_di0_pre", "dummy", "dummy", "ldb_di0", "ldb_di1", };
static const char *ipu1_di1_sels[]	= { "ipu1_di1_pre", "dummy", "dummy", "ldb_di0", "ldb_di1", };
static const char *ipu2_di0_sels[]	= { "ipu2_di0_pre", "dummy", "dummy", "ldb_di0", "ldb_di1", };
static const char *ipu2_di1_sels[]	= { "ipu2_di1_pre", "dummy", "dummy", "ldb_di0", "ldb_di1", };
static const char *hsi_tx_sels[]	= { "pll3_120m", "pll2_pfd2_396m", };
static const char *pcie_axi_sels[]	= { "axi", "ahb", };
static const char *ssi_sels[]		= { "pll3_pfd2_508m", "pll3_pfd3_454m", "pll4_audio_div", };
static const char *usdhc_sels[]	= { "pll2_pfd2_396m", "pll2_pfd0_352m", };
static const char *enfc_sels[]	= { "pll2_pfd0_352m", "pll2_bus", "pll3_usb_otg", "pll2_pfd2_396m", };
static const char *emi_sels[]		= { "pll2_pfd2_396m", "pll3_usb_otg", "axi", "pll2_pfd0_352m", };
static const char *emi_slow_sels[]      = { "axi", "pll3_usb_otg", "pll2_pfd2_396m", "pll2_pfd0_352m", };
static const char *vdo_axi_sels[]	= { "axi", "ahb", };
static const char *vpu_axi_sels[]	= { "axi", "pll2_pfd2_396m", "pll2_pfd0_352m", };
static const char *cko1_sels[]	= { "pll3_usb_otg", "pll2_bus", "pll1_sys", "pll5_video_div",
				    "dummy", "axi", "enfc", "ipu1_di0", "ipu1_di1", "ipu2_di0",
				    "ipu2_di1", "ahb", "ipg", "ipg_per", "ckil", "pll4_audio_div", };
static const char *cko2_sels[] = {
	"mmdc_ch0_axi_podf", "mmdc_ch1_axi", "usdhc4", "usdhc1",
	"gpu2d_axi", "dummy", "ecspi_root", "gpu3d_axi",
	"usdhc3", "dummy", "arm", "ipu1",
	"ipu2", "vdo_axi", "osc", "gpu2d_core",
	"gpu3d_core", "usdhc2", "ssi1", "ssi2",
	"ssi3", "gpu3d_shader", "vpu_axi", "can_root",
	"ldb_di0", "ldb_di1", "esai_extal", "eim_slow",
	"uart_serial", "spdif", "spdif1", "hsi_tx",
};
static const char *cko_sels[] = { "cko1", "cko2", };
static const char *lvds_sels[]	= { "arm", "pll1_sys", "dummy", "dummy", "dummy", "dummy", "dummy", "pll5_video_div",
				    "dummy", "dummy", "pcie_ref", "sata_ref", "usbphy1", "usbphy2", };
static const char *pll_av_sels[] = { "osc", "lvds1_in", "lvds2_in", "dummy", };
static void __iomem *anatop_base;
static void __iomem *ccm_base;

static u32 share_count_esai;
static u32 share_count_ssi1;
static u32 share_count_ssi2;
static u32 share_count_ssi3;

enum mx6q_clks {
	dummy, ckil, ckih, osc, pll2_pfd0_352m, pll2_pfd1_594m, pll2_pfd2_396m,
	pll3_pfd0_720m, pll3_pfd1_540m, pll3_pfd2_508m, pll3_pfd3_454m,
	pll2_198m, pll3_120m, pll3_80m, pll3_60m, twd, step, pll1_sw,
	periph_pre, periph2_pre, periph_clk2_sel, periph2_clk2_sel, axi_sel,
	esai_sel, spdif1_sel, spdif_sel, gpu2d_axi, gpu3d_axi, gpu2d_core_sel,
	gpu3d_core_sel, gpu3d_shader_sel, ipu1_sel, ipu2_sel, ldb_di0_sel,
	ldb_di1_sel, ipu1_di0_pre_sel, ipu1_di1_pre_sel, ipu2_di0_pre_sel,
	ipu2_di1_pre_sel, ipu1_di0_sel, ipu1_di1_sel, ipu2_di0_sel,
	ipu2_di1_sel, hsi_tx_sel, pcie_axi_sel, ssi1_sel, ssi2_sel, ssi3_sel,
	usdhc1_sel, usdhc2_sel, usdhc3_sel, usdhc4_sel, enfc_sel, emi_sel,
	emi_slow_sel, vdo_axi_sel, vpu_axi_sel, cko1_sel, periph, periph2,
	periph_clk2, periph2_clk2, ipg, ipg_per, esai_pred, esai_podf,
	spdif1_pred, spdif1_podf, spdif_pred, spdif_podf, can_root, ecspi_root,
	gpu2d_core_podf, gpu3d_core_podf, gpu3d_shader, ipu1_podf, ipu2_podf,
	ldb_di0_podf_unused, ldb_di1_podf_unused, ipu1_di0_pre, ipu1_di1_pre,
	ipu2_di0_pre, ipu2_di1_pre, hsi_tx_podf, ssi1_pred, ssi1_podf,
	ssi2_pred, ssi2_podf, ssi3_pred, ssi3_podf, uart_serial_podf,
	usdhc1_podf, usdhc2_podf, usdhc3_podf, usdhc4_podf, enfc_pred, enfc_podf,
	emi_podf, emi_slow_podf, vpu_axi_podf, cko1_podf, axi, mmdc_ch0_axi_podf,
	mmdc_ch1_axi_podf, arm, ahb, apbh_dma, asrc_gate, can1_ipg, can1_serial,
	can2_ipg, can2_serial, ecspi1, ecspi2, ecspi3, ecspi4, ecspi5, enet,
	esai_extal, gpt_ipg, gpt_ipg_per, gpu2d_core, gpu3d_core, hdmi_iahb,
	hdmi_isfr, i2c1, i2c2, i2c3, iim, enfc, ipu1, ipu1_di0, ipu1_di1, ipu2,
	ipu2_di0, ldb_di0, ldb_di1, ipu2_di1, hsi_tx, mlb, mmdc_ch0_axi,
	mmdc_ch1_axi, ocram, openvg_axi, pcie_axi, pwm1, pwm2, pwm3, pwm4, per1_bch,
	gpmi_bch_apb, gpmi_bch, gpmi_io, gpmi_apb, sata, sdma, spba, ssi1,
	ssi2, ssi3, uart_ipg, uart_serial, usboh3, usdhc1, usdhc2, usdhc3,
	usdhc4, vdo_axi, vpu_axi, cko1, pll1_sys, pll2_bus, pll3_usb_otg,
	pll4_audio, pll5_video, pll8_mlb, pll7_usb_host, pll6_enet, ssi1_ipg,
	ssi2_ipg, ssi3_ipg, rom, usbphy1, usbphy2, ldb_di0_div_3_5, ldb_di1_div_3_5,
	sata_ref, sata_ref_100m, pcie_ref, pcie_ref_125m, enet_ref, usbphy1_gate,
	usbphy2_gate, pll4_post_div, pll5_post_div, pll5_video_div, eim_slow,
	spdif, cko2_sel, cko2_podf, cko2, cko, vdoa, gpt_3m, video_27m,
	ldb_di0_div_7, ldb_di1_div_7, ldb_di0_div_sel, ldb_di1_div_sel,
	pll4_audio_div, lvds1_sel, lvds1_in, lvds1_out, caam_mem, caam_aclk,
	caam_ipg, epit1, epit2, tzasc2, pll4_sel, lvds2_sel, lvds2_in, lvds2_out,
	anaclk1, anaclk2, spdif1, asrc_ipg, asrc_mem, esai_ipg, esai_mem,
	axi_alt_sel, dcic1, dcic2, clk_max
};

static struct clk *clk[clk_max];
static struct clk_onecell_data clk_data;

static enum mx6q_clks const clks_init_on[] __initconst = {
	mmdc_ch0_axi, rom, arm, ocram,
};

static struct clk_div_table clk_enet_ref_table[] = {
	{ .val = 0, .div = 20, },
	{ .val = 1, .div = 10, },
	{ .val = 2, .div = 5, },
	{ .val = 3, .div = 4, },
};

static struct clk_div_table post_div_table[] = {
	{ .val = 2, .div = 1, },
	{ .val = 1, .div = 2, },
	{ .val = 0, .div = 4, },
	{ }
};

static struct clk_div_table video_div_table[] = {
	{ .val = 0, .div = 1, },
	{ .val = 1, .div = 2, },
	{ .val = 2, .div = 1, },
	{ .val = 3, .div = 4, },
	{ }
};

/*
 * Kernel parameter 'ldb_di_clk_sel' is used to select parent of ldb_di_clk,
 * among the following clocks.
 *   'pll5_video_div'
 *   'pll2_pfd0_352m'
 *   'pll2_pfd2_396m'
 *   'mmdc_ch1_axi'
 *   'pll3_usb_otg'
 * Example format: ldb_di_clk_sel=pll5_video_div
 * If the kernel parameter is absent or invalid, pll2_pfd0_352m will be
 * selected by default.
 */
static int ldb_di_sel = 1;

static int __init get_ldb_di_parent(char *p)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ldb_di_sels); i++) {
		if (strcmp(p, ldb_di_sels[i]) == 0) {
			ldb_di_sel = i;
			break;
		}
	}

	return 0;
}
early_param("ldb_di_clk_sel", get_ldb_di_parent);

static void init_ldb_clks(void)
{
	u32 reg;

	/*
	 * Need to follow a strict procedure when changing the LDB
	 * clock, else we can introduce a glitch. Things to keep in
	 * mind:
	 * 1. The current and new parent clocks must be disabled.
	 * 2. The default clock for ldb_dio_clk is mmdc_ch1 which has
	 * no CG bit.
	 * 3. In the RTL implementation of the LDB_DI_CLK_SEL mux
	 * the top four options are in one mux and the PLL3 option along
	 * with another option is in the second mux. There is third mux
	 * used to decide between the first and second mux.
	 * The code below switches the parent to the bottom mux first
	 * and then manipulates the top mux. This ensures that no glitch
	 * will enter the divider.
	 *
	 * Need to disable MMDC_CH1 clock manually as there is no CG bit
	 * for this clock. The only way to disable this clock is to move
	 * it topll3_sw_clk and then to disable pll3_sw_clk
	 * Make sure periph2_clk2_sel is set to pll3_sw_clk
	 */
	reg = readl_relaxed(ccm_base + 0x18);
	reg &= ~(1 << 20);
	writel_relaxed(reg, ccm_base + 0x18);

	/*
	 * Set MMDC_CH1 mask bit.
	 */
	reg = readl_relaxed(ccm_base + 0x4);
	reg |= 1 << 16;
	writel_relaxed(reg, ccm_base + 0x4);

	/*
	 * Set the periph2_clk_sel to the top mux so that
	 * mmdc_ch1 is from pll3_sw_clk.
	 */
	reg = readl_relaxed(ccm_base + 0x14);
	reg |= 1 << 26;
	writel_relaxed(reg, ccm_base + 0x14);

	/*
	 * Wait for the clock switch.
	 */
	while (readl_relaxed(ccm_base + 0x48))
		;

	/*
	 * Disable pll3_sw_clk by selecting the bypass clock source.
	 */
	reg = readl_relaxed(ccm_base + 0xc);
	reg |= 1 << 0;
	writel_relaxed(reg, ccm_base + 0xc);

	/*
	 * Set the ldb_di0_clk and ldb_di1_clk to 111b.
	 */
	reg = readl_relaxed(ccm_base + 0x2c);
	reg |= ((7 << 9) | (7 << 12));
	writel_relaxed(reg, ccm_base + 0x2c);

	/*
	 * Set the ldb_di0_clk and ldb_di1_clk to 100b.
	 */
	reg = readl_relaxed(ccm_base + 0x2c);
	reg &= ~((7 << 9) | (7 << 12));
	reg |= ((4 << 9) | (4 << 12));
	writel_relaxed(reg, ccm_base + 0x2c);

	/*
	 * Perform the LDB parent clock switch.
	 */
	reg = readl_relaxed(ccm_base + 0x2c);
	reg &= ~((7 << 9) | (7 << 12));
	reg |= ((ldb_di_sel << 9) | (ldb_di_sel << 12));
	writel_relaxed(reg, ccm_base + 0x2c);

	/*
	 * Unbypass pll3_sw_clk.
	 */
	reg = readl_relaxed(ccm_base + 0xc);
	reg &= ~(1 << 0);
	writel_relaxed(reg, ccm_base + 0xc);

	/*
	 * Set the periph2_clk_sel back to the bottom mux so that
	 * mmdc_ch1 is from its original parent.
	 */
	reg = readl_relaxed(ccm_base + 0x14);
	reg &= ~(1 << 26);
	writel_relaxed(reg, ccm_base + 0x14);

	/*
	 * Wait for the clock switch.
	 */
	while (readl_relaxed(ccm_base + 0x48))
		;

	/*
	 * Clear MMDC_CH1 mask bit.
	 */
	reg = readl_relaxed(ccm_base + 0x4);
	reg &= ~(1 << 16);
	writel_relaxed(reg, ccm_base + 0x4);

}

static void __init imx6q_clocks_init(struct device_node *ccm_node)
{
	struct device_node *np;
	void __iomem *base;
	int i, irq;
	u32 reg;

	clk[dummy] = imx_clk_fixed("dummy", 0);
	clk[ckil] = imx_obtain_fixed_clock("ckil", 0);
	clk[ckih] = imx_obtain_fixed_clock("ckih1", 0);
	clk[osc] = imx_obtain_fixed_clock("osc", 0);
	/* Clock source from external clock via ANACLK1/2 PADs */
	clk[anaclk1] = imx_obtain_fixed_clock("anaclk1", 0);
	clk[anaclk2] = imx_obtain_fixed_clock("anaclk2", 0);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-anatop");
	anatop_base = base = of_iomap(np, 0);
	WARN_ON(!base);

	/* Audio/video PLL post dividers do not work on i.MX6q revision 1.0 */
	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_1_0) {
		post_div_table[1].div = 1;
		post_div_table[2].div = 1;
		video_div_table[1].div = 1;
		video_div_table[2].div = 1;
	};

	/*                   type                               name         parent_name  base     div_mask */
	clk[pll1_sys]      = imx_clk_pllv3(IMX_PLLV3_SYS,	"pll1_sys",	"osc", base,        0x7f, false);
	clk[pll2_bus]      = imx_clk_pllv3(IMX_PLLV3_GENERIC,	"pll2_bus",	"osc", base + 0x30, 0x1, false);
	clk[pll3_usb_otg]  = imx_clk_pllv3(IMX_PLLV3_USB,	"pll3_usb_otg",	"osc", base + 0x10, 0x3, false);
	clk[pll4_audio]    = imx_clk_pllv3(IMX_PLLV3_AV,	"pll4_audio",	"pll4_sel", base + 0x70, 0x7f, false);
	clk[pll5_video]    = imx_clk_pllv3(IMX_PLLV3_AV,	"pll5_video",	"osc", base + 0xa0, 0x7f, false);
	clk[pll6_enet]     = imx_clk_pllv3(IMX_PLLV3_ENET,	"pll6_enet",	"osc", base + 0xe0, 0x3, false);
	clk[pll7_usb_host] = imx_clk_pllv3(IMX_PLLV3_USB,	"pll7_usb_host",	"osc", base + 0x20, 0x3, false);

	/*                              name            reg       shift width parent_names     num_parents */
	clk[lvds1_sel]    = imx_clk_mux("lvds1_sel",    base + 0x160, 0,  5,  lvds_sels,       ARRAY_SIZE(lvds_sels));
	clk[lvds2_sel]    = imx_clk_mux("lvds2_sel",    base + 0x160, 5,  5,  lvds_sels,       ARRAY_SIZE(lvds_sels));
	clk[pll4_sel]     = imx_clk_mux("pll4_sel",     base + 0x70, 14,  2,  pll_av_sels,     ARRAY_SIZE(pll_av_sels));

	/*
	 * Bit 20 is the reserved and read-only bit, we do this only for:
	 * - Do nothing for usbphy clk_enable/disable
	 * - Keep refcount when do usbphy clk_enable/disable, in that case,
	 * the clk framework may need to enable/disable usbphy's parent
	 */
	clk[usbphy1] = imx_clk_gate("usbphy1", "pll3_usb_otg", base + 0x10, 20);
	clk[usbphy2] = imx_clk_gate("usbphy2", "pll7_usb_host", base + 0x20, 20);

	/*
	 * usbphy*_gate needs to be on after system boots up, and software
	 * never needs to control it anymore.
	 */
	clk[usbphy1_gate] = imx_clk_gate("usbphy1_gate", "dummy", base + 0x10, 6);
	clk[usbphy2_gate] = imx_clk_gate("usbphy2_gate", "dummy", base + 0x20, 6);

	clk[sata_ref] = imx_clk_fixed_factor("sata_ref", "pll6_enet", 1, 5);
	clk[pcie_ref] = imx_clk_fixed_factor("pcie_ref", "pll6_enet", 1, 4);
	/* NOTICE: The gate of the lvds1/2 in/out is used to select the clk direction */
	clk[lvds1_in] = imx_clk_gate("lvds1_in", "anaclk1", base + 0x160, 12);
	clk[lvds2_in] = imx_clk_gate("lvds2_in", "anaclk2", base + 0x160, 13);
	clk[lvds1_out] = imx_clk_gate("lvds1_out", "lvds1_sel", base + 0x160, 10);
	clk[lvds2_out] = imx_clk_gate("lvds2_out", "lvds2_sel", base + 0x160, 11);

	clk[sata_ref_100m] = imx_clk_gate("sata_ref_100m", "sata_ref", base + 0xe0, 20);
	clk[pcie_ref_125m] = imx_clk_gate("pcie_ref_125m", "pcie_ref", base + 0xe0, 19);

	clk[enet_ref] = clk_register_divider_table(NULL, "enet_ref", "pll6_enet", 0,
			base + 0xe0, 0, 2, 0, clk_enet_ref_table,
			&imx_ccm_lock);

	/*                                name              parent_name        reg       idx */
	clk[pll2_pfd0_352m] = imx_clk_pfd("pll2_pfd0_352m", "pll2_bus",     base + 0x100, 0);
	clk[pll2_pfd1_594m] = imx_clk_pfd("pll2_pfd1_594m", "pll2_bus",     base + 0x100, 1);
	clk[pll2_pfd2_396m] = imx_clk_pfd("pll2_pfd2_396m", "pll2_bus",     base + 0x100, 2);
	clk[pll3_pfd0_720m] = imx_clk_pfd("pll3_pfd0_720m", "pll3_usb_otg", base + 0xf0,  0);
	clk[pll3_pfd1_540m] = imx_clk_pfd("pll3_pfd1_540m", "pll3_usb_otg", base + 0xf0,  1);
	clk[pll3_pfd2_508m] = imx_clk_pfd("pll3_pfd2_508m", "pll3_usb_otg", base + 0xf0,  2);
	clk[pll3_pfd3_454m] = imx_clk_pfd("pll3_pfd3_454m", "pll3_usb_otg", base + 0xf0,  3);

	/*                                    name         parent_name     mult div */
	clk[pll2_198m] = imx_clk_fixed_factor("pll2_198m", "pll2_pfd2_396m", 1, 2);
	clk[pll3_120m] = imx_clk_fixed_factor("pll3_120m", "pll3_usb_otg",   1, 4);
	clk[pll3_80m]  = imx_clk_fixed_factor("pll3_80m",  "pll3_usb_otg",   1, 6);
	clk[pll3_60m]  = imx_clk_fixed_factor("pll3_60m",  "pll3_usb_otg",   1, 8);
	clk[twd]       = imx_clk_fixed_factor("twd",       "arm",            1, 2);
	clk[gpt_3m]    = imx_clk_fixed_factor("gpt_3m",    "osc",            1, 8);
	clk[video_27m] = imx_clk_fixed_factor("video_27m", "pll3_pfd1_540m", 1, 20);
	if (cpu_is_imx6dl()) {
		clk[gpu2d_axi] = imx_clk_fixed_factor("gpu2d_axi", "mmdc_ch0_axi_podf", 1, 1);
		clk[gpu3d_axi] = imx_clk_fixed_factor("gpu3d_axi", "mmdc_ch0_axi_podf", 1, 1);
	}

	clk[pll4_post_div] = clk_register_divider_table(NULL, "pll4_post_div", "pll4_audio", CLK_SET_RATE_PARENT, base + 0x70, 19, 2, 0, post_div_table, &imx_ccm_lock);
	clk[pll4_audio_div] = clk_register_divider(NULL, "pll4_audio_div", "pll4_post_div", CLK_SET_RATE_PARENT, base + 0x170, 15, 1, 0, &imx_ccm_lock);
	clk[pll5_post_div] = clk_register_divider_table(NULL, "pll5_post_div", "pll5_video", CLK_SET_RATE_PARENT, base + 0xa0, 19, 2, 0, post_div_table, &imx_ccm_lock);
	clk[pll5_video_div] = clk_register_divider_table(NULL, "pll5_video_div", "pll5_post_div", CLK_SET_RATE_PARENT, base + 0x170, 30, 2, 0, video_div_table, &imx_ccm_lock);

	np = ccm_node;
	ccm_base = base = of_iomap(np, 0);
	WARN_ON(!base);
	imx6_pm_set_ccm_base(base);

	/*                                  name                reg       shift width parent_names     num_parents */
	clk[step]             = imx_clk_mux("step",	        base + 0xc,  8,  1, step_sels,	       ARRAY_SIZE(step_sels));
	clk[pll1_sw]          = imx_clk_mux_glitchless("pll1_sw", base + 0xc, 2, 1, pll1_sw_sels,      ARRAY_SIZE(pll1_sw_sels));
	clk[periph_pre]       = imx_clk_mux_bus("periph_pre",       base + 0x18, 18, 2, periph_pre_sels,   ARRAY_SIZE(periph_pre_sels));
	clk[periph2_pre]      = imx_clk_mux("periph2_pre",      base + 0x18, 21, 2, periph_pre_sels,   ARRAY_SIZE(periph_pre_sels));
	clk[periph_clk2_sel]  = imx_clk_mux_bus("periph_clk2_sel",  base + 0x18, 12, 2, periph_clk2_sels,  ARRAY_SIZE(periph_clk2_sels));
	clk[periph2_clk2_sel] = imx_clk_mux("periph2_clk2_sel", base + 0x18, 20, 1, periph2_clk2_sels, ARRAY_SIZE(periph2_clk2_sels));
	clk[axi_alt_sel]      = imx_clk_mux("axi_alt_sel",      base + 0x14, 7,  1, axi_alt_sels,      ARRAY_SIZE(axi_alt_sels));
	clk[axi_sel]          = imx_clk_mux_glitchless("axi_sel", base + 0x14, 6, 1, axi_sels,         ARRAY_SIZE(axi_sels));
	clk[esai_sel]         = imx_clk_mux("esai_sel",         base + 0x20, 19, 2, audio_sels,        ARRAY_SIZE(audio_sels));
	clk[spdif1_sel]       = imx_clk_mux("spdif1_sel",       base + 0x30, 7,  2, audio_sels,        ARRAY_SIZE(audio_sels));
	clk[spdif_sel]        = imx_clk_mux("spdif_sel",        base + 0x30, 20, 2, audio_sels,        ARRAY_SIZE(audio_sels));
	if (cpu_is_imx6q()) {
		clk[gpu2d_axi]        = imx_clk_mux("gpu2d_axi",        base + 0x18, 0,  1, gpu_axi_sels,      ARRAY_SIZE(gpu_axi_sels));
		clk[gpu3d_axi]        = imx_clk_mux("gpu3d_axi",        base + 0x18, 1,  1, gpu_axi_sels,      ARRAY_SIZE(gpu_axi_sels));
	}
	clk[gpu2d_core_sel]   = imx_clk_mux("gpu2d_core_sel",   base + 0x18, 16, 2, gpu2d_core_sels,   ARRAY_SIZE(gpu2d_core_sels));
	clk[gpu3d_core_sel]   = imx_clk_mux("gpu3d_core_sel",   base + 0x18, 4,  2, gpu3d_core_sels,   ARRAY_SIZE(gpu3d_core_sels));
	clk[gpu3d_shader_sel] = imx_clk_mux("gpu3d_shader_sel", base + 0x18, 8,  2, gpu3d_shader_sels, ARRAY_SIZE(gpu3d_shader_sels));
	clk[ipu1_sel]         = imx_clk_mux("ipu1_sel",         base + 0x3c, 9,  2, ipu_sels,          ARRAY_SIZE(ipu_sels));
	clk[ipu2_sel]         = imx_clk_mux("ipu2_sel",         base + 0x3c, 14, 2, ipu_sels,          ARRAY_SIZE(ipu_sels));
	clk[ldb_di0_div_sel]  = imx_clk_mux_flags("ldb_di0_div_sel", base + 0x20, 10, 1, ldb_di0_div_sels, ARRAY_SIZE(ldb_di0_div_sels), CLK_SET_RATE_PARENT);
	clk[ldb_di1_div_sel]  = imx_clk_mux_flags("ldb_di1_div_sel", base + 0x20, 11, 1, ldb_di1_div_sels, ARRAY_SIZE(ldb_di1_div_sels), CLK_SET_RATE_PARENT);
	clk[ipu1_di0_pre_sel] = imx_clk_mux_flags("ipu1_di0_pre_sel", base + 0x34, 6,  3, ipu_di_pre_sels,   ARRAY_SIZE(ipu_di_pre_sels), CLK_SET_RATE_PARENT);
	clk[ipu1_di1_pre_sel] = imx_clk_mux_flags("ipu1_di1_pre_sel", base + 0x34, 15, 3, ipu_di_pre_sels,   ARRAY_SIZE(ipu_di_pre_sels), CLK_SET_RATE_PARENT);
	clk[ipu2_di0_pre_sel] = imx_clk_mux_flags("ipu2_di0_pre_sel", base + 0x38, 6,  3, ipu_di_pre_sels,   ARRAY_SIZE(ipu_di_pre_sels), CLK_SET_RATE_PARENT);
	clk[ipu2_di1_pre_sel] = imx_clk_mux_flags("ipu2_di1_pre_sel", base + 0x38, 15, 3, ipu_di_pre_sels,   ARRAY_SIZE(ipu_di_pre_sels), CLK_SET_RATE_PARENT);
	clk[ipu1_di0_sel]     = imx_clk_mux_flags("ipu1_di0_sel",     base + 0x34, 0,  3, ipu1_di0_sels,     ARRAY_SIZE(ipu1_di0_sels), CLK_SET_RATE_PARENT);
	clk[ipu1_di1_sel]     = imx_clk_mux_flags("ipu1_di1_sel",     base + 0x34, 9,  3, ipu1_di1_sels,     ARRAY_SIZE(ipu1_di1_sels), CLK_SET_RATE_PARENT);
	clk[ipu2_di0_sel]     = imx_clk_mux_flags("ipu2_di0_sel",     base + 0x38, 0,  3, ipu2_di0_sels,     ARRAY_SIZE(ipu2_di0_sels), CLK_SET_RATE_PARENT);
	clk[ipu2_di1_sel]     = imx_clk_mux_flags("ipu2_di1_sel",     base + 0x38, 9,  3, ipu2_di1_sels,     ARRAY_SIZE(ipu2_di1_sels), CLK_SET_RATE_PARENT);
	clk[hsi_tx_sel]       = imx_clk_mux("hsi_tx_sel",       base + 0x30, 28, 1, hsi_tx_sels,       ARRAY_SIZE(hsi_tx_sels));
	clk[pcie_axi_sel]     = imx_clk_mux("pcie_axi_sel",     base + 0x18, 10, 1, pcie_axi_sels,     ARRAY_SIZE(pcie_axi_sels));
	clk[ssi1_sel]         = imx_clk_fixup_mux("ssi1_sel",   base + 0x1c, 10, 2, ssi_sels,          ARRAY_SIZE(ssi_sels),          imx_cscmr1_fixup);
	clk[ssi2_sel]         = imx_clk_fixup_mux("ssi2_sel",   base + 0x1c, 12, 2, ssi_sels,          ARRAY_SIZE(ssi_sels),          imx_cscmr1_fixup);
	clk[ssi3_sel]         = imx_clk_fixup_mux("ssi3_sel",   base + 0x1c, 14, 2, ssi_sels,          ARRAY_SIZE(ssi_sels),          imx_cscmr1_fixup);
	clk[usdhc1_sel]       = imx_clk_fixup_mux("usdhc1_sel", base + 0x1c, 16, 1, usdhc_sels,        ARRAY_SIZE(usdhc_sels),        imx_cscmr1_fixup);
	clk[usdhc2_sel]       = imx_clk_fixup_mux("usdhc2_sel", base + 0x1c, 17, 1, usdhc_sels,        ARRAY_SIZE(usdhc_sels),        imx_cscmr1_fixup);
	clk[usdhc3_sel]       = imx_clk_fixup_mux("usdhc3_sel", base + 0x1c, 18, 1, usdhc_sels,        ARRAY_SIZE(usdhc_sels),        imx_cscmr1_fixup);
	clk[usdhc4_sel]       = imx_clk_fixup_mux("usdhc4_sel", base + 0x1c, 19, 1, usdhc_sels,        ARRAY_SIZE(usdhc_sels),        imx_cscmr1_fixup);
	clk[enfc_sel]         = imx_clk_mux("enfc_sel",         base + 0x2c, 16, 2, enfc_sels,         ARRAY_SIZE(enfc_sels));
	clk[emi_sel]          = imx_clk_fixup_mux("emi_sel",      base + 0x1c, 27, 2, emi_sels,        ARRAY_SIZE(emi_sels),          imx_cscmr1_fixup);
	clk[emi_slow_sel]     = imx_clk_fixup_mux("emi_slow_sel", base + 0x1c, 29, 2, emi_slow_sels,   ARRAY_SIZE(emi_slow_sels),     imx_cscmr1_fixup);
	clk[vdo_axi_sel]      = imx_clk_mux("vdo_axi_sel",      base + 0x18, 11, 1, vdo_axi_sels,      ARRAY_SIZE(vdo_axi_sels));
	clk[vpu_axi_sel]      = imx_clk_mux("vpu_axi_sel",      base + 0x18, 14, 2, vpu_axi_sels,      ARRAY_SIZE(vpu_axi_sels));
	clk[cko1_sel]         = imx_clk_mux("cko1_sel",         base + 0x60, 0,  4, cko1_sels,         ARRAY_SIZE(cko1_sels));
	clk[cko2_sel]         = imx_clk_mux("cko2_sel",         base + 0x60, 16, 5, cko2_sels,         ARRAY_SIZE(cko2_sels));
	clk[cko]              = imx_clk_mux("cko",              base + 0x60, 8, 1,  cko_sels,          ARRAY_SIZE(cko_sels));

	/*                              name         reg      shift width busy: reg, shift parent_names  num_parents */
	clk[periph]  = imx_clk_busy_mux("periph",  base + 0x14, 25,  1,   base + 0x48, 5,  periph_sels,  ARRAY_SIZE(periph_sels));
	clk[periph2] = imx_clk_busy_mux("periph2", base + 0x14, 26,  1,   base + 0x48, 3,  periph2_sels, ARRAY_SIZE(periph2_sels));

	/*                                      name                parent_name          reg       shift width */
	clk[periph_clk2]      = imx_clk_divider("periph_clk2",      "periph_clk2_sel",   base + 0x14, 27, 3);
	clk[periph2_clk2]     = imx_clk_divider("periph2_clk2",     "periph2_clk2_sel",  base + 0x14, 0,  3);
	clk[ipg]              = imx_clk_divider("ipg",              "ahb",               base + 0x14, 8,  2);
	clk[ipg_per]          = imx_clk_fixup_divider("ipg_per",    "ipg",               base + 0x1c, 0,  6, imx_cscmr1_fixup);
	clk[esai_pred]        = imx_clk_divider("esai_pred",        "esai_sel",          base + 0x28, 9,  3);
	clk[esai_podf]        = imx_clk_divider("esai_podf",        "esai_pred",         base + 0x28, 25, 3);
	clk[spdif1_pred]      = imx_clk_divider("spdif1_pred",      "spdif1_sel",        base + 0x30, 12, 3);
	clk[spdif1_podf]      = imx_clk_divider("spdif1_podf",      "spdif1_pred",       base + 0x30, 9,  3);
	clk[spdif_pred]       = imx_clk_divider("spdif_pred",       "spdif_sel",         base + 0x30, 25, 3);
	clk[spdif_podf]       = imx_clk_divider("spdif_podf",       "spdif_pred",        base + 0x30, 22, 3);
	clk[can_root]         = imx_clk_divider("can_root",         "pll3_60m",          base + 0x20, 2,  6);
	clk[ecspi_root]       = imx_clk_divider("ecspi_root",       "pll3_60m",          base + 0x38, 19, 6);
	clk[gpu2d_core_podf]  = imx_clk_divider("gpu2d_core_podf",  "gpu2d_core_sel",    base + 0x18, 23, 3);
	clk[gpu3d_core_podf]  = imx_clk_divider("gpu3d_core_podf",  "gpu3d_core_sel",    base + 0x18, 26, 3);
	clk[gpu3d_shader]     = imx_clk_divider("gpu3d_shader",     "gpu3d_shader_sel",  base + 0x18, 29, 3);
	clk[ipu1_podf]        = imx_clk_divider("ipu1_podf",        "ipu1_sel",          base + 0x3c, 11, 3);
	clk[ipu2_podf]        = imx_clk_divider("ipu2_podf",        "ipu2_sel",          base + 0x3c, 16, 3);
	clk[ldb_di0_div_3_5]  = imx_clk_fixed_factor("ldb_di0_div_3_5", ldb_di_sels[ldb_di_sel], 2, 7);
	clk[ldb_di0_div_7]    = imx_clk_fixed_factor("ldb_di0_div_7",   ldb_di_sels[ldb_di_sel], 1, 7);
	clk[ldb_di1_div_3_5]  = imx_clk_fixed_factor("ldb_di1_div_3_5", ldb_di_sels[ldb_di_sel], 2, 7);
	clk[ldb_di1_div_7]    = imx_clk_fixed_factor("ldb_di1_div_7",   ldb_di_sels[ldb_di_sel], 1, 7);
	clk[ipu1_di0_pre]     = imx_clk_divider("ipu1_di0_pre",     "ipu1_di0_pre_sel",  base + 0x34, 3,  3);
	clk[ipu1_di1_pre]     = imx_clk_divider("ipu1_di1_pre",     "ipu1_di1_pre_sel",  base + 0x34, 12, 3);
	clk[ipu2_di0_pre]     = imx_clk_divider("ipu2_di0_pre",     "ipu2_di0_pre_sel",  base + 0x38, 3,  3);
	clk[ipu2_di1_pre]     = imx_clk_divider("ipu2_di1_pre",     "ipu2_di1_pre_sel",  base + 0x38, 12, 3);
	clk[hsi_tx_podf]      = imx_clk_divider("hsi_tx_podf",      "hsi_tx_sel",        base + 0x30, 29, 3);
	clk[ssi1_pred]        = imx_clk_divider("ssi1_pred",        "ssi1_sel",          base + 0x28, 6,  3);
	clk[ssi1_podf]        = imx_clk_divider("ssi1_podf",        "ssi1_pred",         base + 0x28, 0,  6);
	clk[ssi2_pred]        = imx_clk_divider("ssi2_pred",        "ssi2_sel",          base + 0x2c, 6,  3);
	clk[ssi2_podf]        = imx_clk_divider("ssi2_podf",        "ssi2_pred",         base + 0x2c, 0,  6);
	clk[ssi3_pred]        = imx_clk_divider("ssi3_pred",        "ssi3_sel",          base + 0x28, 22, 3);
	clk[ssi3_podf]        = imx_clk_divider("ssi3_podf",        "ssi3_pred",         base + 0x28, 16, 6);
	clk[uart_serial_podf] = imx_clk_divider("uart_serial_podf", "pll3_80m",          base + 0x24, 0,  6);
	clk[usdhc1_podf]      = imx_clk_divider("usdhc1_podf",      "usdhc1_sel",        base + 0x24, 11, 3);
	clk[usdhc2_podf]      = imx_clk_divider("usdhc2_podf",      "usdhc2_sel",        base + 0x24, 16, 3);
	clk[usdhc3_podf]      = imx_clk_divider("usdhc3_podf",      "usdhc3_sel",        base + 0x24, 19, 3);
	clk[usdhc4_podf]      = imx_clk_divider("usdhc4_podf",      "usdhc4_sel",        base + 0x24, 22, 3);
	clk[enfc_pred]        = imx_clk_divider("enfc_pred",        "enfc_sel",          base + 0x2c, 18, 3);
	clk[enfc_podf]        = imx_clk_divider("enfc_podf",        "enfc_pred",         base + 0x2c, 21, 6);
	clk[emi_podf]         = imx_clk_fixup_divider("emi_podf",   "emi_sel",           base + 0x1c, 20, 3, imx_cscmr1_fixup);
	clk[emi_slow_podf]    = imx_clk_fixup_divider("emi_slow_podf", "emi_slow_sel",   base + 0x1c, 23, 3, imx_cscmr1_fixup);
	clk[vpu_axi_podf]     = imx_clk_divider("vpu_axi_podf",     "vpu_axi_sel",       base + 0x24, 25, 3);
	clk[cko1_podf]        = imx_clk_divider("cko1_podf",        "cko1_sel",          base + 0x60, 4,  3);
	clk[cko2_podf]        = imx_clk_divider("cko2_podf",        "cko2_sel",          base + 0x60, 21, 3);

	/*                                            name                 parent_name    reg        shift width busy: reg, shift */
	clk[axi]               = imx_clk_busy_divider("axi",               "axi_sel",     base + 0x14, 16,  3,   base + 0x48, 0);
	clk[mmdc_ch0_axi_podf] = imx_clk_busy_divider("mmdc_ch0_axi_podf", "periph",      base + 0x14, 19,  3,   base + 0x48, 4);
	clk[mmdc_ch1_axi_podf] = imx_clk_busy_divider("mmdc_ch1_axi_podf", "periph2",     base + 0x14, 3,   3,   base + 0x48, 2);
	clk[arm]               = imx_clk_busy_divider("arm",               "pll1_sw",     base + 0x10, 0,   3,   base + 0x48, 16);
	clk[ahb]               = imx_clk_busy_divider("ahb",               "periph",      base + 0x14, 10,  3,   base + 0x48, 1);

	/*                                name             parent_name          reg         shift */
	clk[apbh_dma]     = imx_clk_gate2("apbh_dma",      "usdhc3",            base + 0x68, 4);
	clk[asrc_gate]    = imx_clk_gate2("asrc_gate",     "ahb",               base + 0x68, 6);
	clk[asrc_ipg]     = imx_clk_fixed_factor("asrc_ipg", "asrc_gate", 1, 1);
	clk[asrc_mem]     = imx_clk_fixed_factor("asrc_mem", "asrc_gate", 1, 1);
	clk[caam_mem]     = imx_clk_gate2("caam_mem",      "ahb",               base + 0x68, 8);
	clk[caam_aclk]    = imx_clk_gate2("caam_aclk",     "ahb",               base + 0x68, 10);
	clk[caam_ipg]     = imx_clk_gate2("caam_ipg",      "ipg",               base + 0x68, 12);
	clk[can1_ipg]     = imx_clk_gate2("can1_ipg",      "ipg",               base + 0x68, 14);
	clk[can1_serial]  = imx_clk_gate2("can1_serial",   "can_root",          base + 0x68, 16);
	clk[can2_ipg]     = imx_clk_gate2("can2_ipg",      "ipg",               base + 0x68, 18);
	clk[can2_serial]  = imx_clk_gate2("can2_serial",   "can_root",          base + 0x68, 20);
	clk[dcic1]        = imx_clk_gate2("dcic1",		   "ipu1_podf",         base + 0x68, 24);
	clk[dcic2]        = imx_clk_gate2("dcic2",		   "ipu2_podf",         base + 0x68, 26);
	clk[ecspi1]       = imx_clk_gate2("ecspi1",        "ecspi_root",        base + 0x6c, 0);
	clk[ecspi2]       = imx_clk_gate2("ecspi2",        "ecspi_root",        base + 0x6c, 2);
	clk[ecspi3]       = imx_clk_gate2("ecspi3",        "ecspi_root",        base + 0x6c, 4);
	clk[ecspi4]       = imx_clk_gate2("ecspi4",        "ecspi_root",        base + 0x6c, 6);
	if (cpu_is_imx6dl())
		/* ecspi5 is replaced with i2c4 on imx6dl & imx6s */
		clk[ecspi5] = imx_clk_gate2("i2c4",        "ipg_per",           base + 0x6c, 8);
	else
		clk[ecspi5] = imx_clk_gate2("ecspi5",      "ecspi_root",        base + 0x6c, 8);
	clk[enet]         = imx_clk_gate2("enet",          "ipg",               base + 0x6c, 10);
	clk[epit1]        = imx_clk_gate2("epit1",         "ipg",               base + 0x6c, 12);
	clk[epit2]        = imx_clk_gate2("epit2",         "ipg",               base + 0x6c, 14);
	clk[esai_extal]   = imx_clk_gate2_shared("esai_extal", "esai_podf",     base + 0x6c, 16, &share_count_esai);
	clk[esai_ipg]     = imx_clk_gate2_shared("esai_ipg",   "ahb",           base + 0x6c, 16, &share_count_esai);
	clk[esai_mem]     = imx_clk_gate2_shared("esai_mem",   "ahb",           base + 0x6c, 16, &share_count_esai);
	clk[gpt_ipg]      = imx_clk_gate2("gpt_ipg",       "ipg",               base + 0x6c, 20);
	clk[gpt_ipg_per]  = imx_clk_gate2("gpt_ipg_per",   "ipg_per",           base + 0x6c, 22);
	if (cpu_is_imx6dl())
		/*
		 * The multiplexer and divider of imx6q clock gpu3d_shader get
		 * redefined/reused as gpu2d_core_sel and gpu2d_core_podf on imx6dl.
		 */
		clk[gpu2d_core] = imx_clk_gate2("gpu2d_core", "gpu3d_shader", base + 0x6c, 24);
	else
		clk[gpu2d_core] = imx_clk_gate2("gpu2d_core", "gpu2d_core_podf", base + 0x6c, 24);
	clk[gpu3d_core]   = imx_clk_gate2("gpu3d_core",    "gpu3d_core_podf",   base + 0x6c, 26);
	clk[hdmi_iahb]    = imx_clk_gate2("hdmi_iahb",     "ahb",               base + 0x70, 0);
	clk[hdmi_isfr]    = imx_clk_gate2("hdmi_isfr",     "pll3_pfd1_540m",    base + 0x70, 4);
	clk[i2c1]         = imx_clk_gate2("i2c1",          "ipg_per",           base + 0x70, 6);
	clk[i2c2]         = imx_clk_gate2("i2c2",          "ipg_per",           base + 0x70, 8);
	clk[i2c3]         = imx_clk_gate2("i2c3",          "ipg_per",           base + 0x70, 10);
	clk[iim]          = imx_clk_gate2("iim",           "ipg",               base + 0x70, 12);
	clk[enfc]         = imx_clk_gate2("enfc",          "enfc_podf",         base + 0x70, 14);
	clk[tzasc2]       = imx_clk_gate2("tzasc2",        "mmdc_ch0_axi_podf", base + 0x70, 24);
	clk[vdoa]         = imx_clk_gate2("vdoa",          "vdo_axi",           base + 0x70, 26);
	clk[ipu1]         = imx_clk_gate2("ipu1",          "ipu1_podf",         base + 0x74, 0);
	clk[ipu1_di0]     = imx_clk_gate2("ipu1_di0",      "ipu1_di0_sel",      base + 0x74, 2);
	clk[ipu1_di1]     = imx_clk_gate2("ipu1_di1",      "ipu1_di1_sel",      base + 0x74, 4);
	clk[ipu2]         = imx_clk_gate2("ipu2",          "ipu2_podf",         base + 0x74, 6);
	clk[ipu2_di0]     = imx_clk_gate2("ipu2_di0",      "ipu2_di0_sel",      base + 0x74, 8);
	clk[ipu2_di1]     = imx_clk_gate2("ipu2_di1",      "ipu2_di1_sel",      base + 0x74, 10);
	clk[ldb_di0]      = imx_clk_gate2("ldb_di0",       "ldb_di0_div_sel",   base + 0x74, 12);
	clk[ldb_di1]      = imx_clk_gate2("ldb_di1",       "ldb_di1_div_sel",   base + 0x74, 14);
	clk[hsi_tx]       = imx_clk_gate2("hsi_tx",        "hsi_tx_podf",       base + 0x74, 16);
	if (cpu_is_imx6dl())
		/*
		 * The multiplexer and divider of the imx6q clock gpu2d get
		 * redefined/reused as mlb_sys_sel and mlb_sys_clk_podf on imx6dl.
		 */
		clk[mlb] = imx_clk_gate2("mlb",            "gpu2d_core_podf",   base + 0x74, 18);
	else
		clk[mlb] = imx_clk_gate2("mlb",            "axi",               base + 0x74, 18);
	clk[ocram]        = imx_clk_busy_gate("ocram",         "ahb",               base + 0x74, 28);
	clk[openvg_axi]   = imx_clk_gate2("openvg_axi",    "axi",               base + 0x74, 30);
	clk[pcie_axi]     = imx_clk_gate2("pcie_axi",      "pcie_axi_sel",      base + 0x78, 0);
	clk[per1_bch]     = imx_clk_gate2("per1_bch",      "usdhc3",            base + 0x78, 12);
	clk[pwm1]         = imx_clk_gate2("pwm1",          "ipg_per",           base + 0x78, 16);
	clk[pwm2]         = imx_clk_gate2("pwm2",          "ipg_per",           base + 0x78, 18);
	clk[pwm3]         = imx_clk_gate2("pwm3",          "ipg_per",           base + 0x78, 20);
	clk[pwm4]         = imx_clk_gate2("pwm4",          "ipg_per",           base + 0x78, 22);
	clk[gpmi_bch_apb] = imx_clk_gate2("gpmi_bch_apb",  "usdhc3",            base + 0x78, 24);
	clk[gpmi_bch]     = imx_clk_gate2("gpmi_bch",      "usdhc4",            base + 0x78, 26);
	clk[gpmi_io]      = imx_clk_gate2("gpmi_io",       "enfc",              base + 0x78, 28);
	clk[gpmi_apb]     = imx_clk_gate2("gpmi_apb",      "usdhc3",            base + 0x78, 30);
	clk[rom]          = imx_clk_gate2("rom",           "ahb",               base + 0x7c, 0);
	clk[sata]         = imx_clk_gate2("sata",          "ipg",               base + 0x7c, 4);
	clk[sdma]         = imx_clk_gate2("sdma",          "ahb",               base + 0x7c, 6);
	clk[spba]         = imx_clk_gate2("spba",          "ipg",               base + 0x7c, 12);
	clk[spdif]        = imx_clk_gate2("spdif",         "spdif_podf",    	base + 0x7c, 14);
	clk[ssi1_ipg]     = imx_clk_gate2_shared("ssi1_ipg",   "ipg",           base + 0x7c, 18, &share_count_ssi1);
	clk[ssi2_ipg]     = imx_clk_gate2_shared("ssi2_ipg",   "ipg",           base + 0x7c, 20, &share_count_ssi2);
	clk[ssi3_ipg]     = imx_clk_gate2_shared("ssi3_ipg",   "ipg",           base + 0x7c, 22, &share_count_ssi3);
	clk[ssi1]         = imx_clk_gate2_shared("ssi1",       "ssi1_podf",     base + 0x7c, 18, &share_count_ssi1);
	clk[ssi2]         = imx_clk_gate2_shared("ssi2",       "ssi2_podf",     base + 0x7c, 20, &share_count_ssi2);
	clk[ssi3]         = imx_clk_gate2_shared("ssi3",       "ssi3_podf",     base + 0x7c, 22, &share_count_ssi3);
	clk[uart_ipg]     = imx_clk_gate2("uart_ipg",      "ipg",               base + 0x7c, 24);
	clk[uart_serial]  = imx_clk_gate2("uart_serial",   "uart_serial_podf",  base + 0x7c, 26);
	clk[usboh3]       = imx_clk_gate2("usboh3",        "ipg",               base + 0x80, 0);
	clk[usdhc1]       = imx_clk_gate2("usdhc1",        "usdhc1_podf",       base + 0x80, 2);
	clk[usdhc2]       = imx_clk_gate2("usdhc2",        "usdhc2_podf",       base + 0x80, 4);
	clk[usdhc3]       = imx_clk_gate2("usdhc3",        "usdhc3_podf",       base + 0x80, 6);
	clk[usdhc4]       = imx_clk_gate2("usdhc4",        "usdhc4_podf",       base + 0x80, 8);
	clk[eim_slow]     = imx_clk_gate2("eim_slow",      "emi_slow_podf",     base + 0x80, 10);
	clk[vdo_axi]      = imx_clk_gate2("vdo_axi",       "vdo_axi_sel",       base + 0x80, 12);
	clk[vpu_axi]      = imx_clk_gate2("vpu_axi",       "vpu_axi_podf",      base + 0x80, 14);
	clk[cko1]         = imx_clk_gate("cko1",           "cko1_podf",         base + 0x60, 7);
	clk[cko2]         = imx_clk_gate("cko2",           "cko2_podf",         base + 0x60, 24);

	/*
	 * These two clocks (mmdc_ch0_axi and mmdc_ch1_axi) were incorrectly
	 * implemented as gate at the beginning.  To fix them with the minimized
	 * impact, let's point them to their dividers.
	 */
	clk[mmdc_ch0_axi] = clk[mmdc_ch0_axi_podf];
	clk[mmdc_ch1_axi] = clk[mmdc_ch1_axi_podf];

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("i.MX6q clk %d: register failed with %ld\n",
				i, PTR_ERR(clk[i]));

	/* Initialize clock gate status */
	writel_relaxed(1 << CCM_CCGR_OFFSET(11) |
		3 << CCM_CCGR_OFFSET(1) |
		3 << CCM_CCGR_OFFSET(0), base + 0x68);
	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_1_0)
		writel_relaxed(3 << CCM_CCGR_OFFSET(11) |
			3 << CCM_CCGR_OFFSET(10), base + 0x6c);
	else
		writel_relaxed(3 << CCM_CCGR_OFFSET(10), base + 0x6c);
	writel_relaxed(1 << CCM_CCGR_OFFSET(12) |
		3 << CCM_CCGR_OFFSET(11) |
		3 << CCM_CCGR_OFFSET(10) |
		3 << CCM_CCGR_OFFSET(9) |
		3 << CCM_CCGR_OFFSET(8), base + 0x70);
	writel_relaxed(1 << CCM_CCGR_OFFSET(13) |
		3 << CCM_CCGR_OFFSET(12) |
		1 << CCM_CCGR_OFFSET(11) |
		3 << CCM_CCGR_OFFSET(10), base + 0x74);
	writel_relaxed(3 << CCM_CCGR_OFFSET(7) |
		3 << CCM_CCGR_OFFSET(6) |
		3 << CCM_CCGR_OFFSET(4), base + 0x78);
	writel_relaxed(1 << CCM_CCGR_OFFSET(0), base + 0x7c);
	writel_relaxed(0, base + 0x80);

	/* Make sure PFDs are disabled at boot. */
	reg = readl_relaxed(anatop_base + 0x100);
	/* Cannot disable pll2_pfd2_396M, as it is the MMDC clock in iMX6DL */
	if (cpu_is_imx6dl())
		reg |= 0x80008080;
	else
		reg |= 0x80808080;
	writel_relaxed(reg, anatop_base + 0x100);

	/* Disable PLL3 PFDs. */
	reg = readl_relaxed(anatop_base + 0xF0);
	reg |= 0x80808080;
	writel_relaxed(reg, anatop_base + 0xF0);

	/* Make sure PLLs is disabled */
	reg = readl_relaxed(anatop_base + 0xA0);
	reg &= ~(1 << 13);
	writel_relaxed(reg, anatop_base + 0xA0);

	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);

	clk_register_clkdev(clk[gpt_ipg], "ipg", "imx-gpt.0");
	clk_register_clkdev(clk[gpt_ipg_per], "per", "imx-gpt.0");
	clk_register_clkdev(clk[gpt_3m], "gpt_3m", "imx-gpt.0");
	clk_register_clkdev(clk[cko1_sel], "cko1_sel", NULL);
	clk_register_clkdev(clk[ahb], "ahb", NULL);
	clk_register_clkdev(clk[cko1], "cko1", NULL);
	clk_register_clkdev(clk[arm], NULL, "cpu0");
	clk_register_clkdev(clk[pll4_audio_div], "pll4_audio_div", NULL);
	clk_register_clkdev(clk[pll4_sel], "pll4_sel", NULL);
	clk_register_clkdev(clk[lvds2_in], "lvds2_in", NULL);
	clk_register_clkdev(clk[esai_extal], "esai_extal", NULL);

	/*
	 * The gpmi needs 100MHz frequency in the EDO/Sync mode,
	 * We can not get the 100MHz from the pll2_pfd0_352m.
	 * So choose pll2_pfd2_396m as enfc_sel's parent.
	 */
	imx_clk_set_parent(clk[enfc_sel], clk[pll2_pfd2_396m]);

	/* Set the parent clks of PCIe lvds1 and pcie_axi to be sata ref, axi */
	imx_clk_set_parent(clk[lvds1_sel], clk[sata_ref]);
	imx_clk_set_parent(clk[pcie_axi_sel], clk[axi]);

	/* gpu clock initilazation */
	/*
	 * On mx6dl, 2d core clock sources(sel, podf) is from 3d
	 * shader core clock, but 3d shader clock multiplexer of
	 * mx6dl is different. For instance the equivalent of
	 * pll2_pfd_594M on mx6q is pll2_pfd_528M on mx6dl.
	 * Make a note here.
	 */
	imx_clk_set_parent(clk[gpu3d_shader_sel], clk[pll2_pfd1_594m]);
	if (cpu_is_imx6dl()) {
		imx_clk_set_rate(clk[gpu2d_core], 528000000);
		/* for mx6dl, change gpu3d_core parent to 594_PFD*/
		imx_clk_set_parent(clk[gpu3d_core_sel], clk[pll2_pfd1_594m]);
		imx_clk_set_rate(clk[gpu3d_core], 528000000);
	} else if (cpu_is_imx6q()) {
		imx_clk_set_rate(clk[gpu3d_shader], 594000000);
		imx_clk_set_parent(clk[gpu3d_core_sel], clk[mmdc_ch0_axi]);
		imx_clk_set_rate(clk[gpu3d_core], 528000000);
		imx_clk_set_parent(clk[gpu2d_core_sel], clk[pll3_usb_otg]);
	}



	/* ipu clock initialization */
	init_ldb_clks();
	imx_clk_set_parent(clk[ipu1_di0_pre_sel], clk[pll5_video_div]);
	imx_clk_set_parent(clk[ipu1_di1_pre_sel], clk[pll5_video_div]);
	imx_clk_set_parent(clk[ipu2_di0_pre_sel], clk[pll5_video_div]);
	imx_clk_set_parent(clk[ipu2_di1_pre_sel], clk[pll5_video_div]);
	imx_clk_set_parent(clk[ipu1_di0_sel], clk[ipu1_di0_pre]);
	imx_clk_set_parent(clk[ipu1_di1_sel], clk[ipu1_di1_pre]);
	imx_clk_set_parent(clk[ipu2_di0_sel], clk[ipu2_di0_pre]);
	imx_clk_set_parent(clk[ipu2_di1_sel], clk[ipu2_di1_pre]);
	if (cpu_is_imx6dl()) {
		imx_clk_set_rate(clk[pll3_pfd1_540m], 540000000);
		imx_clk_set_parent(clk[ipu1_sel], clk[pll3_pfd1_540m]);
		imx_clk_set_parent(clk[axi_alt_sel], clk[pll3_pfd1_540m]);
		imx_clk_set_parent(clk[axi_sel], clk[axi_alt_sel]);
		/* set epdc/pxp axi clock to 200Mhz */
		imx_clk_set_parent(clk[ipu2_sel], clk[pll2_pfd2_396m]);
		imx_clk_set_rate(clk[ipu2], 200000000);
	} else if (cpu_is_imx6q()) {
		imx_clk_set_parent(clk[ipu1_sel], clk[mmdc_ch0_axi]);
		imx_clk_set_parent(clk[ipu2_sel], clk[mmdc_ch0_axi]);
	}

	/*
	 * Let's initially set up CLKO with OSC24M, since this configuration
	 * is widely used by imx6q board designs to clock audio codec.
	 */
	imx_clk_set_parent(clk[cko2_sel], clk[osc]);
	imx_clk_set_parent(clk[cko], clk[cko2]);

	/* Audio clocks */
	imx_clk_set_parent(clk[ssi1_sel], clk[pll4_audio_div]);
	imx_clk_set_parent(clk[ssi2_sel], clk[pll4_audio_div]);
	imx_clk_set_parent(clk[ssi3_sel], clk[pll4_audio_div]);
	imx_clk_set_parent(clk[esai_sel], clk[pll4_audio_div]);
	imx_clk_set_parent(clk[spdif_sel], clk[pll3_pfd3_454m]);
	imx_clk_set_rate(clk[spdif_podf], 227368421);
	imx_clk_set_parent(clk[spdif1_sel], clk[pll3_usb_otg]);
	imx_clk_set_rate(clk[spdif1_sel], 7500000);

	/* Set pll4_audio to a value that can derive 5K-88.2KHz and 8K-96KHz */
	imx_clk_set_rate(clk[pll4_audio_div], 541900800);

	/*Set enet_ref clock to 125M to supply for RGMII tx_clk */
	clk_set_rate(clk[enet_ref], 125000000);

#ifdef CONFIG_MX6_VPU_352M
	/*
	 * If VPU 352M is enabled, then PLL2_PDF2 need to be
	 * set to 352M, cpufreq will be disabled as VDDSOC/PU
	 * need to be at highest voltage, scaling cpu freq is
	 * not saving any power, and busfreq will be also disabled
	 * as the PLL2_PFD2 is not at default freq, in a word,
	 * all modules that sourceing clk from PLL2_PFD2 will
	 * be impacted.
	 */
	imx_clk_set_rate(clk[pll2_pfd2_396m], 352000000);
	imx_clk_set_parent(clk[vpu_axi_sel], clk[pll2_pfd2_396m]);
	pr_info("VPU 352M is enabled!\n");
#endif

	/*
	 * Enable clocks only after both parent and rate are all initialized
	 * as needed
	 */
	for (i = 0; i < ARRAY_SIZE(clks_init_on); i++)
		imx_clk_prepare_enable(clk[clks_init_on[i]]);

	if (IS_ENABLED(CONFIG_USB_MXS_PHY)) {
		imx_clk_prepare_enable(clk[usbphy1_gate]);
		imx_clk_prepare_enable(clk[usbphy2_gate]);
	}

	/* Set initial power mode */
	imx6_set_lpm(WAIT_CLOCKED);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpt");
	base = of_iomap(np, 0);
	WARN_ON(!base);
	irq = irq_of_parse_and_map(np, 0);
	mxc_timer_init(base, irq);
}
CLK_OF_DECLARE(imx6q, "fsl,imx6q-ccm", imx6q_clocks_init);
