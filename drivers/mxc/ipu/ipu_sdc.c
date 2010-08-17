/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_sdc.c
 *
 * @brief IPU SDC submodule API functions
 *
 * @ingroup IPU
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

static uint32_t g_h_start_width;
static uint32_t g_v_start_width;

static const uint32_t di_mappings[] = {
	0x1600AAAA, 0x00E05555, 0x00070000, 3,	/* RGB888 */
	0x0005000F, 0x000B000F, 0x0011000F, 1,	/* RGB666 */
	0x0011000F, 0x000B000F, 0x0005000F, 1,	/* BGR666 */
	0x0004003F, 0x000A000F, 0x000F003F, 1	/* RGB565 */
};

/*!
 * This function is called to initialize a synchronous LCD panel.
 *
 * @param       panel           The type of panel.
 *
 * @param       pixel_clk       Desired pixel clock frequency in Hz.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer. Pixel
 *                              format is a FOURCC ASCII code.
 *
 * @param       width           The width of panel in pixels.
 *
 * @param       height          The height of panel in pixels.
 *
 * @param       hStartWidth     The number of pixel clocks between the HSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       hSyncWidth      The width of the HSYNC signal in units of pixel
 *                              clocks.
 *
 * @param       hEndWidth       The number of pixel clocks between the end of
 *                              valid data and the HSYNC signal for next line.
 *
 * @param       vStartWidth     The number of lines between the VSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       vSyncWidth      The width of the VSYNC signal in units of lines
 *
 * @param       vEndWidth       The number of lines between the end of valid
 *                              data and the VSYNC signal for next frame.
 *
 * @param       sig             Bitfield of signal polarities for LCD interface.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_sdc_init_panel(ipu_panel_t panel,
			   uint32_t pixel_clk,
			   uint16_t width, uint16_t height,
			   uint32_t pixel_fmt,
			   uint16_t h_start_width, uint16_t h_sync_width,
			   uint16_t h_end_width, uint16_t v_start_width,
			   uint16_t v_sync_width, uint16_t v_end_width,
			   ipu_di_signal_cfg_t sig)
{
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t old_conf;
	uint32_t div;

	dev_dbg(g_ipu_dev, "panel size = %d x %d\n", width, height);

	if ((v_sync_width == 0) || (h_sync_width == 0))
		return EINVAL;

	/* Init panel size and blanking periods */
	reg =
	    ((uint32_t) (h_sync_width - 1) << 26) |
	    ((uint32_t) (width + h_sync_width + h_start_width + h_end_width - 1)
	     << 16);
	__raw_writel(reg, SDC_HOR_CONF);

	reg = ((uint32_t) (v_sync_width - 1) << 26) | SDC_V_SYNC_WIDTH_L |
	    ((uint32_t)
	     (height + v_sync_width + v_start_width + v_end_width - 1) << 16);
	__raw_writel(reg, SDC_VER_CONF);

	g_h_start_width = h_start_width + h_sync_width;
	g_v_start_width = v_start_width + v_sync_width;

	switch (panel) {
	case IPU_PANEL_SHARP_TFT:
		__raw_writel(0x00FD0102L, SDC_SHARP_CONF_1);
		__raw_writel(0x00F500F4L, SDC_SHARP_CONF_2);
		__raw_writel(SDC_COM_SHARP | SDC_COM_TFT_COLOR, SDC_COM_CONF);
		break;
	case IPU_PANEL_TFT:
		__raw_writel(SDC_COM_TFT_COLOR, SDC_COM_CONF);
		break;
	default:
		return EINVAL;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	/* Init clocking */

	/* Calculate divider */
	/* fractional part is 4 bits so simply multiple by 2^4 to get fractional part */
	dev_dbg(g_ipu_dev, "pixel clk = %d\n", pixel_clk);
	div = (clk_get_rate(g_ipu_clk) * 16) / pixel_clk;
	if (div < 0x40) {	/* Divider less than 4 */
		dev_dbg(g_ipu_dev,
			"InitPanel() - Pixel clock divider less than 1\n");
		div = 0x40;
	}
	/* DISP3_IF_CLK_DOWN_WR is half the divider value and 2 less fraction bits */
	/* Subtract 1 extra from DISP3_IF_CLK_DOWN_WR based on timing debug     */
	/* DISP3_IF_CLK_UP_WR is 0 */
	__raw_writel((((div / 8) - 1) << 22) | div, DI_DISP3_TIME_CONF);

	/* DI settings */
	old_conf = __raw_readl(DI_DISP_IF_CONF) & 0x78FFFFFF;
	old_conf |= sig.datamask_en << DI_D3_DATAMSK_SHIFT |
	    sig.clksel_en << DI_D3_CLK_SEL_SHIFT |
	    sig.clkidle_en << DI_D3_CLK_IDLE_SHIFT;
	__raw_writel(old_conf, DI_DISP_IF_CONF);

	old_conf = __raw_readl(DI_DISP_SIG_POL) & 0xE0FFFFFF;
	old_conf |= sig.data_pol << DI_D3_DATA_POL_SHIFT |
	    sig.clk_pol << DI_D3_CLK_POL_SHIFT |
	    sig.enable_pol << DI_D3_DRDY_SHARP_POL_SHIFT |
	    sig.Hsync_pol << DI_D3_HSYNC_POL_SHIFT |
	    sig.Vsync_pol << DI_D3_VSYNC_POL_SHIFT;
	__raw_writel(old_conf, DI_DISP_SIG_POL);

	switch (pixel_fmt) {
	case IPU_PIX_FMT_RGB24:
		__raw_writel(di_mappings[0], DI_DISP3_B0_MAP);
		__raw_writel(di_mappings[1], DI_DISP3_B1_MAP);
		__raw_writel(di_mappings[2], DI_DISP3_B2_MAP);
		__raw_writel(__raw_readl(DI_DISP_ACC_CC) |
			     ((di_mappings[3] - 1) << 12), DI_DISP_ACC_CC);
		break;
	case IPU_PIX_FMT_RGB666:
		__raw_writel(di_mappings[4], DI_DISP3_B0_MAP);
		__raw_writel(di_mappings[5], DI_DISP3_B1_MAP);
		__raw_writel(di_mappings[6], DI_DISP3_B2_MAP);
		__raw_writel(__raw_readl(DI_DISP_ACC_CC) |
			     ((di_mappings[7] - 1) << 12), DI_DISP_ACC_CC);
		break;
	case IPU_PIX_FMT_BGR666:
		__raw_writel(di_mappings[8], DI_DISP3_B0_MAP);
		__raw_writel(di_mappings[9], DI_DISP3_B1_MAP);
		__raw_writel(di_mappings[10], DI_DISP3_B2_MAP);
		__raw_writel(__raw_readl(DI_DISP_ACC_CC) |
			     ((di_mappings[11] - 1) << 12), DI_DISP_ACC_CC);
		break;
	default:
		__raw_writel(di_mappings[12], DI_DISP3_B0_MAP);
		__raw_writel(di_mappings[13], DI_DISP3_B1_MAP);
		__raw_writel(di_mappings[14], DI_DISP3_B2_MAP);
		__raw_writel(__raw_readl(DI_DISP_ACC_CC) |
			     ((di_mappings[15] - 1) << 12), DI_DISP_ACC_CC);
		break;
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	dev_dbg(g_ipu_dev, "DI_DISP_IF_CONF = 0x%08X\n",
		__raw_readl(DI_DISP_IF_CONF));
	dev_dbg(g_ipu_dev, "DI_DISP_SIG_POL = 0x%08X\n",
		__raw_readl(DI_DISP_SIG_POL));
	dev_dbg(g_ipu_dev, "DI_DISP3_TIME_CONF = 0x%08X\n",
		__raw_readl(DI_DISP3_TIME_CONF));

	return 0;
}

/*!
 * This function sets the foreground and background plane global alpha blending
 * modes.
 *
 * @param       enable          Boolean to enable or disable global alpha
 *                              blending. If disabled, per pixel blending is used.
 *
 * @param       alpha           Global alpha value.
 *
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_sdc_set_global_alpha(bool enable, uint8_t alpha)
{
	uint32_t reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (enable) {
		reg = __raw_readl(SDC_GW_CTRL) & 0x00FFFFFFL;
		__raw_writel(reg | ((uint32_t) alpha << 24), SDC_GW_CTRL);

		reg = __raw_readl(SDC_COM_CONF);
		__raw_writel(reg | SDC_COM_GLB_A, SDC_COM_CONF);
	} else {
		reg = __raw_readl(SDC_COM_CONF);
		__raw_writel(reg & ~SDC_COM_GLB_A, SDC_COM_CONF);
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}

/*!
 * This function sets the transparent color key for SDC graphic plane.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       enable          Boolean to enable or disable color key
 *
 * @param       colorKey        24-bit RGB color to use as transparent color key.
 *
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_sdc_set_color_key(ipu_channel_t channel, bool enable,
			      uint32_t color_key)
{
	uint32_t reg, sdc_conf;
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	sdc_conf = __raw_readl(SDC_COM_CONF);
	if (channel == MEM_SDC_BG) {
		sdc_conf &= ~SDC_COM_GWSEL;
	} else {
		sdc_conf |= SDC_COM_GWSEL;
	}

	if (enable) {
		reg = __raw_readl(SDC_GW_CTRL) & 0xFF000000L;
		__raw_writel(reg | (color_key & 0x00FFFFFFL), SDC_GW_CTRL);

		sdc_conf |= SDC_COM_KEY_COLOR_G;
	} else {
		sdc_conf &= ~SDC_COM_KEY_COLOR_G;
	}
	__raw_writel(sdc_conf, SDC_COM_CONF);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}

int32_t ipu_sdc_set_brightness(uint8_t value)
{
	__raw_writel(0x03000000UL | value << 16, SDC_PWM_CTRL);
	return 0;
}

/*!
 * This function sets the window position of the foreground or background plane.
 * modes.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       x_pos           The X coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @param       y_pos           The Y coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @return      This function returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_window_pos(ipu_channel_t channel, int16_t x_pos,
			       int16_t y_pos)
{
	x_pos += g_h_start_width;
	y_pos += g_v_start_width;

	if (channel == MEM_SDC_BG) {
		__raw_writel((x_pos << 16) | y_pos, SDC_BG_POS);
	} else if (channel == MEM_SDC_FG) {
		__raw_writel((x_pos << 16) | y_pos, SDC_FG_POS);
	} else {
		return EINVAL;
	}
	return 0;
}

void _ipu_sdc_fg_init(ipu_channel_params_t *params)
{
	uint32_t reg;
	(void)params;

	/* Enable FG channel */
	reg = __raw_readl(SDC_COM_CONF);
	__raw_writel(reg | SDC_COM_FG_EN | SDC_COM_BG_EN, SDC_COM_CONF);
}

uint32_t _ipu_sdc_fg_uninit(void)
{
	uint32_t reg;

	/* Disable FG channel */
	reg = __raw_readl(SDC_COM_CONF);
	__raw_writel(reg & ~SDC_COM_FG_EN, SDC_COM_CONF);

	return reg & SDC_COM_FG_EN;
}

void _ipu_sdc_bg_init(ipu_channel_params_t *params)
{
	uint32_t reg;
	(void)params;

	/* Enable FG channel */
	reg = __raw_readl(SDC_COM_CONF);
	__raw_writel(reg | SDC_COM_BG_EN, SDC_COM_CONF);
}

uint32_t _ipu_sdc_bg_uninit(void)
{
	uint32_t reg;

	/* Disable BG channel */
	reg = __raw_readl(SDC_COM_CONF);
	__raw_writel(reg & ~SDC_COM_BG_EN, SDC_COM_CONF);

	return reg & SDC_COM_BG_EN;
}

/* Exported symbols for modules. */
EXPORT_SYMBOL(ipu_sdc_init_panel);
EXPORT_SYMBOL(ipu_sdc_set_global_alpha);
EXPORT_SYMBOL(ipu_sdc_set_color_key);
EXPORT_SYMBOL(ipu_sdc_set_brightness);
EXPORT_SYMBOL(ipu_disp_set_window_pos);
