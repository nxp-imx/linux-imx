/*
 * Copyright 2005-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_csi.c
 *
 * @brief IPU CMOS Sensor interface functions
 *
 * @ingroup IPU
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/ipu.h>

#include "ipu_prv.h"
#include "ipu_regs.h"

static bool gipu_csi_get_mclk_flag;
static int csi_mclk_flag;

extern void gpio_sensor_suspend(bool flag);

/*!
 * ipu_csi_init_interface
 *    Sets initial values for the CSI registers.
 *    The width and height of the sensor and the actual frame size will be
 *    set to the same values.
 * @param       width        Sensor width
 * @param       height       Sensor height
 * @param       pixel_fmt    pixel format
 * @param       sig          ipu_csi_signal_cfg_t structure
 *
 * @return      0 for success, -EINVAL for error
 */
int32_t
ipu_csi_init_interface(uint16_t width, uint16_t height, uint32_t pixel_fmt,
		       ipu_csi_signal_cfg_t sig)
{
	uint32_t data = 0;

	/* Set SENS_DATA_FORMAT bits (8 and 9)
	   RGB or YUV444 is 0 which is current value in data so not set explicitly
	   This is also the default value if attempts are made to set it to
	   something invalid. */
	switch (pixel_fmt) {
	case IPU_PIX_FMT_UYVY:
		data = CSI_SENS_CONF_DATA_FMT_YUV422;
		break;
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_BGR24:
		data = CSI_SENS_CONF_DATA_FMT_RGB_YUV444;
		break;
	case IPU_PIX_FMT_GENERIC:
		data = CSI_SENS_CONF_DATA_FMT_BAYER;
		break;
	default:
		return -EINVAL;
	}

	/* Set the CSI_SENS_CONF register remaining fields */
	data |= sig.data_width << CSI_SENS_CONF_DATA_WIDTH_SHIFT |
	    sig.data_pol << CSI_SENS_CONF_DATA_POL_SHIFT |
	    sig.Vsync_pol << CSI_SENS_CONF_VSYNC_POL_SHIFT |
	    sig.Hsync_pol << CSI_SENS_CONF_HSYNC_POL_SHIFT |
	    sig.pixclk_pol << CSI_SENS_CONF_PIX_CLK_POL_SHIFT |
	    sig.ext_vsync << CSI_SENS_CONF_EXT_VSYNC_SHIFT |
	    sig.clk_mode << CSI_SENS_CONF_SENS_PRTCL_SHIFT |
	    sig.sens_clksrc << CSI_SENS_CONF_SENS_CLKSRC_SHIFT;

	__raw_writel(data, CSI_SENS_CONF);

	/* Setup frame size     */
	__raw_writel(width | height << 16, CSI_SENS_FRM_SIZE);

	__raw_writel(width << 16, CSI_FLASH_STROBE_1);
	__raw_writel(height << 16 | 0x22, CSI_FLASH_STROBE_2);

	/* Set CCIR registers */
	if (sig.clk_mode == IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE) {
		__raw_writel(0x40030, CSI_CCIR_CODE_1);
		__raw_writel(0xFF0000, CSI_CCIR_CODE_3);
	} else if (sig.clk_mode == IPU_CSI_CLK_MODE_CCIR656_INTERLACED) {
		__raw_writel(0xD07DF, CSI_CCIR_CODE_1);
		__raw_writel(0x40596, CSI_CCIR_CODE_2);
		__raw_writel(0xFF0000, CSI_CCIR_CODE_3);
	}

	dev_dbg(g_ipu_dev, "CSI_SENS_CONF = 0x%08X\n",
		__raw_readl(CSI_SENS_CONF));
	dev_dbg(g_ipu_dev, "CSI_ACT_FRM_SIZE = 0x%08X\n",
		__raw_readl(CSI_ACT_FRM_SIZE));

	return 0;
}

/*!
 * ipu_csi_flash_strobe
 *
 * @param       flag         true to turn on flash strobe
 *
 * @return      0 for success
 */
void ipu_csi_flash_strobe(bool flag)
{
	if (flag == true) {
		__raw_writel(__raw_readl(CSI_FLASH_STROBE_2) | 0x1,
			     CSI_FLASH_STROBE_2);
	}
}

/*!
 * ipu_csi_enable_mclk
 *
 * @param       src         enum define which source to control the clk
 *                          CSI_MCLK_VF CSI_MCLK_ENC CSI_MCLK_RAW CSI_MCLK_I2C
 * @param       flag        true to enable mclk, false to disable mclk
 * @param       wait        true to wait 100ms make clock stable, false not wait
 *
 * @return      0 for success
 */
int32_t ipu_csi_enable_mclk(int src, bool flag, bool wait)
{
	if (flag == true) {
		csi_mclk_flag |= src;
	} else {
		csi_mclk_flag &= ~src;
	}

	if (gipu_csi_get_mclk_flag == flag)
		return 0;

	if (flag == true) {
		clk_enable(g_ipu_csi_clk);
		if (wait == true)
			msleep(10);
		/*printk("enable csi clock from source %d\n", src);     */
		gipu_csi_get_mclk_flag = true;
	} else if (csi_mclk_flag == 0) {
		clk_disable(g_ipu_csi_clk);
		/*printk("disable csi clock from source %d\n", src); */
		gipu_csi_get_mclk_flag = flag;
	}

	return 0;
}

/*!
 * ipu_csi_read_mclk_flag
 *
 * @return  csi_mclk_flag
 */
int ipu_csi_read_mclk_flag(void)
{
	return csi_mclk_flag;
}

/*!
 * ipu_csi_get_window_size
 *
 * @param       width        pointer to window width
 * @param       height       pointer to window height
 * @param       dummy        dummy for IPUv1 to keep the same interface with IPUv3
 *
 */
void ipu_csi_get_window_size(uint32_t *width, uint32_t *height,
		uint32_t dummy)
{
	uint32_t reg;

	reg = __raw_readl(CSI_ACT_FRM_SIZE);
	*width = (reg & 0xFFFF) + 1;
	*height = (reg >> 16 & 0xFFFF) + 1;
}

/*!
 * ipu_csi_set_window_size
 *
 * @param       width        window width
 * @param       height       window height
 * @param       dummy        dummy for IPUv1 to keep the same interface with IPUv3
 *
 */
void ipu_csi_set_window_size(uint32_t width, uint32_t height, uint32_t dummy)
{
	__raw_writel((width - 1) | (height - 1) << 16, CSI_ACT_FRM_SIZE);
}

/*!
 * ipu_csi_set_window_pos
 *
 * @param       left        uint32 window x start
 * @param       top         uint32 window y start
 * @param       dummy       dummy for IPUv1 to keep the same interface with IPUv3
 *
 */
void ipu_csi_set_window_pos(uint32_t left, uint32_t top, uint32_t dummy)
{
	uint32_t temp = __raw_readl(CSI_OUT_FRM_CTRL);
	temp &= 0xffff0000;
	temp = top | (left << 8) | temp;
	__raw_writel(temp, CSI_OUT_FRM_CTRL);
}

/*!
 * ipu_csi_get_sensor_protocol
 *
 * @param	csi         csi 0 or csi 1
 *
 * @return	Returns sensor protocol
 */
int32_t ipu_csi_get_sensor_protocol(uint32_t csi)
{
	/* TODO */
}
EXPORT_SYMBOL(ipu_csi_get_sensor_protocol);

/* Exported symbols for modules. */
EXPORT_SYMBOL(ipu_csi_set_window_pos);
EXPORT_SYMBOL(ipu_csi_set_window_size);
EXPORT_SYMBOL(ipu_csi_get_window_size);
EXPORT_SYMBOL(ipu_csi_read_mclk_flag);
EXPORT_SYMBOL(ipu_csi_enable_mclk);
EXPORT_SYMBOL(ipu_csi_flash_strobe);
EXPORT_SYMBOL(ipu_csi_init_interface);
