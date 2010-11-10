/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_capture.c
 *
 * @brief IPU capture dase functions
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
#include <linux/clk.h>
#include <mach/mxc_dvfs.h>

#include "ipu_prv.h"
#include "ipu_regs.h"

/*!
 * ipu_csi_init_interface
 *	Sets initial values for the CSI registers.
 *	The width and height of the sensor and the actual frame size will be
 *	set to the same values.
 * @param	width		Sensor width
 * @param       height		Sensor height
 * @param       pixel_fmt	pixel format
 * @param       cfg_param	ipu_csi_signal_cfg_t structure
 * @param       csi             csi 0 or csi 1
 *
 * @return      0 for success, -EINVAL for error
 */
int32_t
ipu_csi_init_interface(uint16_t width, uint16_t height, uint32_t pixel_fmt,
	ipu_csi_signal_cfg_t cfg_param)
{
	uint32_t data = 0;
	uint32_t csi = cfg_param.csi;
	unsigned long lock_flags;

	/* Set SENS_DATA_FORMAT bits (8, 9 and 10)
	   RGB or YUV444 is 0 which is current value in data so not set
	   explicitly
	   This is also the default value if attempts are made to set it to
	   something invalid. */
	switch (pixel_fmt) {
	case IPU_PIX_FMT_YUYV:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_YUYV;
		break;
	case IPU_PIX_FMT_UYVY:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_UYVY;
		break;
	case IPU_PIX_FMT_RGB24:
	case IPU_PIX_FMT_BGR24:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_RGB_YUV444;
		break;
	case IPU_PIX_FMT_GENERIC:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_BAYER;
		break;
	case IPU_PIX_FMT_RGB565:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_RGB565;
		break;
	case IPU_PIX_FMT_RGB555:
		cfg_param.data_fmt = CSI_SENS_CONF_DATA_FMT_RGB555;
		break;
	default:
		return -EINVAL;
	}

	/* Set the CSI_SENS_CONF register remaining fields */
	data |= cfg_param.data_width << CSI_SENS_CONF_DATA_WIDTH_SHIFT |
		cfg_param.data_fmt << CSI_SENS_CONF_DATA_FMT_SHIFT |
		cfg_param.data_pol << CSI_SENS_CONF_DATA_POL_SHIFT |
		cfg_param.Vsync_pol << CSI_SENS_CONF_VSYNC_POL_SHIFT |
		cfg_param.Hsync_pol << CSI_SENS_CONF_HSYNC_POL_SHIFT |
		cfg_param.pixclk_pol << CSI_SENS_CONF_PIX_CLK_POL_SHIFT |
		cfg_param.ext_vsync << CSI_SENS_CONF_EXT_VSYNC_SHIFT |
		cfg_param.clk_mode << CSI_SENS_CONF_SENS_PRTCL_SHIFT |
		cfg_param.pack_tight << CSI_SENS_CONF_PACK_TIGHT_SHIFT |
		cfg_param.force_eof << CSI_SENS_CONF_FORCE_EOF_SHIFT |
		cfg_param.data_en_pol << CSI_SENS_CONF_DATA_EN_POL_SHIFT;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	__raw_writel(data, CSI_SENS_CONF(csi));

	/* Setup sensor frame size */
	__raw_writel((width - 1) | (height - 1) << 16, CSI_SENS_FRM_SIZE(csi));

	/* Set CCIR registers */
	if (cfg_param.clk_mode == IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE) {
		__raw_writel(0x40030, CSI_CCIR_CODE_1(csi));
		__raw_writel(0xFF0000, CSI_CCIR_CODE_3(csi));
	} else if (cfg_param.clk_mode == IPU_CSI_CLK_MODE_CCIR656_INTERLACED) {
		if (width == 720 && height == 625) {
			/* PAL case */
			/*
			 * Field0BlankEnd = 0x6, Field0BlankStart = 0x2,
			 * Field0ActiveEnd = 0x4, Field0ActiveStart = 0
			 */
			__raw_writel(0x40596, CSI_CCIR_CODE_1(csi));
			/*
			 * Field1BlankEnd = 0x7, Field1BlankStart = 0x3,
			 * Field1ActiveEnd = 0x5, Field1ActiveStart = 0x1
			 */
			__raw_writel(0xD07DF, CSI_CCIR_CODE_2(csi));
			__raw_writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		} else if (width == 720 && height == 525) {
			/* NTSC case */
			/*
			 * Field0BlankEnd = 0x7, Field0BlankStart = 0x3,
			 * Field0ActiveEnd = 0x5, Field0ActiveStart = 0x1
			 */
			__raw_writel(0xD07DF, CSI_CCIR_CODE_1(csi));
			/*
			 * Field1BlankEnd = 0x6, Field1BlankStart = 0x2,
			 * Field1ActiveEnd = 0x4, Field1ActiveStart = 0
			 */
			__raw_writel(0x40596, CSI_CCIR_CODE_2(csi));
			__raw_writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		} else {
			spin_unlock_irqrestore(&ipu_lock, lock_flags);
			dev_err(g_ipu_dev, "Unsupported CCIR656 interlaced "
					"video mode\n");
			return -EINVAL;
		}
		_ipu_csi_ccir_err_detection_enable(csi);
	} else if ((cfg_param.clk_mode ==
			IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR) ||
		(cfg_param.clk_mode ==
			IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR) ||
		(cfg_param.clk_mode ==
			IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR) ||
		(cfg_param.clk_mode ==
			IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR)) {
		__raw_writel(0x40030, CSI_CCIR_CODE_1(csi));
		__raw_writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		_ipu_csi_ccir_err_detection_enable(csi);
	} else if ((cfg_param.clk_mode == IPU_CSI_CLK_MODE_GATED_CLK) ||
		   (cfg_param.clk_mode == IPU_CSI_CLK_MODE_NONGATED_CLK)) {
		_ipu_csi_ccir_err_detection_disable(csi);
	}

	dev_dbg(g_ipu_dev, "CSI_SENS_CONF = 0x%08X\n",
		__raw_readl(CSI_SENS_CONF(csi)));
	dev_dbg(g_ipu_dev, "CSI_ACT_FRM_SIZE = 0x%08X\n",
		__raw_readl(CSI_ACT_FRM_SIZE(csi)));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_csi_init_interface);

/*!
 * ipu_csi_get_sensor_protocol
 *
 * @param	csi         csi 0 or csi 1
 *
 * @return	Returns sensor protocol
 */
int32_t ipu_csi_get_sensor_protocol(uint32_t csi)
{
	return (__raw_readl(CSI_SENS_CONF(csi)) &
		CSI_SENS_CONF_SENS_PRTCL_MASK) >>
		CSI_SENS_CONF_SENS_PRTCL_SHIFT;
}
EXPORT_SYMBOL(ipu_csi_get_sensor_protocol);

/*!
 * _ipu_csi_mclk_set
 *
 * @param	pixel_clk   desired pixel clock frequency in Hz
 * @param	csi         csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_mclk_set(uint32_t pixel_clk, uint32_t csi)
{
	uint32_t temp;
	uint32_t div_ratio;

	div_ratio = (clk_get_rate(g_ipu_clk) / pixel_clk) - 1;

	if (div_ratio > 0xFF || div_ratio < 0) {
		dev_dbg(g_ipu_dev, "The value of pixel_clk extends normal range\n");
		return -EINVAL;
	}

	temp = __raw_readl(CSI_SENS_CONF(csi));
	temp &= ~CSI_SENS_CONF_DIVRATIO_MASK;
	__raw_writel(temp | (div_ratio << CSI_SENS_CONF_DIVRATIO_SHIFT),
			CSI_SENS_CONF(csi));

	return 0;
}

/*!
 * ipu_csi_enable_mclk
 *
 * @param	csi         csi 0 or csi 1
 * @param       flag        true to enable mclk, false to disable mclk
 * @param       wait        true to wait 100ms make clock stable, false not wait
 *
 * @return      Returns 0 on success
 */
int ipu_csi_enable_mclk(int csi, bool flag, bool wait)
{
	if (flag) {
		clk_enable(g_csi_clk[csi]);
		if (wait == true)
			msleep(10);
	} else {
		clk_disable(g_csi_clk[csi]);
	}

	return 0;
}
EXPORT_SYMBOL(ipu_csi_enable_mclk);

/*!
 * ipu_csi_get_window_size
 *
 * @param	width	pointer to window width
 * @param	height	pointer to window height
 * @param	csi	csi 0 or csi 1
 */
void ipu_csi_get_window_size(uint32_t *width, uint32_t *height, uint32_t csi)
{
	uint32_t reg;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = __raw_readl(CSI_ACT_FRM_SIZE(csi));
	*width = (reg & 0xFFFF) + 1;
	*height = (reg >> 16 & 0xFFFF) + 1;

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_get_window_size);

/*!
 * ipu_csi_set_window_size
 *
 * @param	width	window width
 * @param       height	window height
 * @param       csi	csi 0 or csi 1
 */
void ipu_csi_set_window_size(uint32_t width, uint32_t height, uint32_t csi)
{
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	__raw_writel((width - 1) | (height - 1) << 16, CSI_ACT_FRM_SIZE(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_set_window_size);

/*!
 * ipu_csi_set_window_pos
 *
 * @param       left	uint32 window x start
 * @param       top	uint32 window y start
 * @param       csi	csi 0 or csi 1
 */
void ipu_csi_set_window_pos(uint32_t left, uint32_t top, uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_OUT_FRM_CTRL(csi));
	temp &= ~(CSI_HSC_MASK | CSI_VSC_MASK);
	temp |= ((top << CSI_VSC_SHIFT) | (left << CSI_HSC_SHIFT));
	__raw_writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_set_window_pos);

/*!
 * _ipu_csi_horizontal_downsize_enable
 *	Enable horizontal downsizing(decimation) by 2.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_horizontal_downsize_enable(uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_OUT_FRM_CTRL(csi));
	temp |= CSI_HORI_DOWNSIZE_EN;
	__raw_writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * _ipu_csi_horizontal_downsize_disable
 *	Disable horizontal downsizing(decimation) by 2.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_horizontal_downsize_disable(uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_OUT_FRM_CTRL(csi));
	temp &= ~CSI_HORI_DOWNSIZE_EN;
	__raw_writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * _ipu_csi_vertical_downsize_enable
 *	Enable vertical downsizing(decimation) by 2.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_vertical_downsize_enable(uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_OUT_FRM_CTRL(csi));
	temp |= CSI_VERT_DOWNSIZE_EN;
	__raw_writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * _ipu_csi_vertical_downsize_disable
 *	Disable vertical downsizing(decimation) by 2.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_vertical_downsize_disable(uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_OUT_FRM_CTRL(csi));
	temp &= ~CSI_VERT_DOWNSIZE_EN;
	__raw_writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * ipu_csi_set_test_generator
 *
 * @param	active       1 for active and 0 for inactive
 * @param       r_value	     red value for the generated pattern of even pixel
 * @param       g_value      green value for the generated pattern of even
 *			     pixel
 * @param       b_value      blue value for the generated pattern of even pixel
 * @param	pixel_clk   desired pixel clock frequency in Hz
 * @param       csi          csi 0 or csi 1
 */
void ipu_csi_set_test_generator(bool active, uint32_t r_value,
	uint32_t g_value, uint32_t b_value, uint32_t pix_clk, uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_TST_CTRL(csi));

	if (active == false) {
		temp &= ~CSI_TEST_GEN_MODE_EN;
		__raw_writel(temp, CSI_TST_CTRL(csi));
	} else {
		/* Set sensb_mclk div_ratio*/
		_ipu_csi_mclk_set(pix_clk, csi);

		temp &= ~(CSI_TEST_GEN_R_MASK | CSI_TEST_GEN_G_MASK |
			CSI_TEST_GEN_B_MASK);
		temp |= CSI_TEST_GEN_MODE_EN;
		temp |= (r_value << CSI_TEST_GEN_R_SHIFT) |
			(g_value << CSI_TEST_GEN_G_SHIFT) |
			(b_value << CSI_TEST_GEN_B_SHIFT);
		__raw_writel(temp, CSI_TST_CTRL(csi));
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_set_test_generator);

/*!
 * _ipu_csi_ccir_err_detection_en
 *	Enable error detection and correction for
 *	CCIR interlaced mode with protection bit.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_ccir_err_detection_enable(uint32_t csi)
{
	uint32_t temp;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	temp = __raw_readl(CSI_CCIR_CODE_1(csi));
	temp |= CSI_CCIR_ERR_DET_EN;
	__raw_writel(temp, CSI_CCIR_CODE_1(csi));
}

/*!
 * _ipu_csi_ccir_err_detection_disable
 *	Disable error detection and correction for
 *	CCIR interlaced mode with protection bit.
 *
 * @param	csi	csi 0 or csi 1
 */
void _ipu_csi_ccir_err_detection_disable(uint32_t csi)
{
	uint32_t temp;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	temp = __raw_readl(CSI_CCIR_CODE_1(csi));
	temp &= ~CSI_CCIR_ERR_DET_EN;
	__raw_writel(temp, CSI_CCIR_CODE_1(csi));
}

/*!
 * _ipu_csi_set_mipi_di
 *
 * @param	num	MIPI data identifier 0-3 handled by CSI
 * @param	di_val	data identifier value
 * @param	csi	csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_set_mipi_di(uint32_t num, uint32_t di_val, uint32_t csi)
{
	uint32_t temp;
	int retval = 0;
	unsigned long lock_flags;

	if (di_val > 0xFFL) {
		retval = -EINVAL;
		goto err;
	}

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_MIPI_DI(csi));

	switch (num) {
	case IPU_CSI_MIPI_DI0:
		temp &= ~CSI_MIPI_DI0_MASK;
		temp |= (di_val << CSI_MIPI_DI0_SHIFT);
		__raw_writel(temp, CSI_MIPI_DI(csi));
		break;
	case IPU_CSI_MIPI_DI1:
		temp &= ~CSI_MIPI_DI1_MASK;
		temp |= (di_val << CSI_MIPI_DI1_SHIFT);
		__raw_writel(temp, CSI_MIPI_DI(csi));
		break;
	case IPU_CSI_MIPI_DI2:
		temp &= ~CSI_MIPI_DI2_MASK;
		temp |= (di_val << CSI_MIPI_DI2_SHIFT);
		__raw_writel(temp, CSI_MIPI_DI(csi));
		break;
	case IPU_CSI_MIPI_DI3:
		temp &= ~CSI_MIPI_DI3_MASK;
		temp |= (di_val << CSI_MIPI_DI3_SHIFT);
		__raw_writel(temp, CSI_MIPI_DI(csi));
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
err:
	return retval;
}

/*!
 * _ipu_csi_set_skip_isp
 *
 * @param	skip		select frames to be skipped and set the
 *				correspond bits to 1
 * @param	max_ratio	number of frames in a skipping set and the
 * 				maximum value of max_ratio is 5
 * @param	csi		csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_set_skip_isp(uint32_t skip, uint32_t max_ratio, uint32_t csi)
{
	uint32_t temp;
	int retval = 0;
	unsigned long lock_flags;

	if (max_ratio > 5) {
		retval = -EINVAL;
		goto err;
	}

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_SKIP(csi));
	temp &= ~(CSI_MAX_RATIO_SKIP_ISP_MASK | CSI_SKIP_ISP_MASK);
	temp |= (max_ratio << CSI_MAX_RATIO_SKIP_ISP_SHIFT) |
		(skip << CSI_SKIP_ISP_SHIFT);
	__raw_writel(temp, CSI_SKIP(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
err:
	return retval;
}

/*!
 * _ipu_csi_set_skip_smfc
 *
 * @param	skip		select frames to be skipped and set the
 *				correspond bits to 1
 * @param	max_ratio	number of frames in a skipping set and the
 *				maximum value of max_ratio is 5
 * @param	id		csi to smfc skipping id
 * @param	csi		csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_set_skip_smfc(uint32_t skip, uint32_t max_ratio,
	uint32_t id, uint32_t csi)
{
	uint32_t temp;
	int retval = 0;
	unsigned long lock_flags;

	if (max_ratio > 5 || id > 3) {
		retval = -EINVAL;
		goto err;
	}

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(CSI_SKIP(csi));
	temp &= ~(CSI_MAX_RATIO_SKIP_SMFC_MASK | CSI_ID_2_SKIP_MASK |
			CSI_SKIP_SMFC_MASK);
	temp |= (max_ratio << CSI_MAX_RATIO_SKIP_SMFC_SHIFT) |
			(id << CSI_ID_2_SKIP_SHIFT) |
			(skip << CSI_SKIP_SMFC_SHIFT);
	__raw_writel(temp, CSI_SKIP(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
err:
	return retval;
}

/*!
 * _ipu_smfc_init
 *	Map CSI frames to IDMAC channels.
 *
 * @param	channel		IDMAC channel 0-3
 * @param	mipi_id		mipi id number 0-3
 * @param	csi		csi0 or csi1
 */
void _ipu_smfc_init(ipu_channel_t channel, uint32_t mipi_id, uint32_t csi)
{
	uint32_t temp;

	temp = __raw_readl(SMFC_MAP);

	switch (channel) {
	case CSI_MEM0:
		temp &= ~SMFC_MAP_CH0_MASK;
		temp |= ((csi << 2) | mipi_id) << SMFC_MAP_CH0_SHIFT;
		break;
	case CSI_MEM1:
		temp &= ~SMFC_MAP_CH1_MASK;
		temp |= ((csi << 2) | mipi_id) << SMFC_MAP_CH1_SHIFT;
		break;
	case CSI_MEM2:
		temp &= ~SMFC_MAP_CH2_MASK;
		temp |= ((csi << 2) | mipi_id) << SMFC_MAP_CH2_SHIFT;
		break;
	case CSI_MEM3:
		temp &= ~SMFC_MAP_CH3_MASK;
		temp |= ((csi << 2) | mipi_id) << SMFC_MAP_CH3_SHIFT;
		break;
	default:
		return;
	}

	__raw_writel(temp, SMFC_MAP);
}

/*!
 * _ipu_smfc_set_wmc
 *	Caution: The number of required channels,  the enabled channels
 *	and the FIFO size per channel are configured restrictedly.
 *
 * @param	channel		IDMAC channel 0-3
 * @param	set		set 1 or clear 0
 * @param	level		water mark level when FIFO is on the
 *				relative size
 */
void _ipu_smfc_set_wmc(ipu_channel_t channel, bool set, uint32_t level)
{
	uint32_t temp;
	unsigned long lock_flags;

	if (g_ipu_clk_enabled == false) {
		stop_dvfs_per();
		g_ipu_clk_enabled = true;
		clk_enable(g_ipu_clk);
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(SMFC_WMC);

	switch (channel) {
	case CSI_MEM0:
		if (set == true) {
			temp &= ~SMFC_WM0_SET_MASK;
			temp |= level << SMFC_WM0_SET_SHIFT;
		} else {
			temp &= ~SMFC_WM0_CLR_MASK;
			temp |= level << SMFC_WM0_CLR_SHIFT;
		}
		break;
	case CSI_MEM1:
		if (set == true) {
			temp &= ~SMFC_WM1_SET_MASK;
			temp |= level << SMFC_WM1_SET_SHIFT;
		} else {
			temp &= ~SMFC_WM1_CLR_MASK;
			temp |= level << SMFC_WM1_CLR_SHIFT;
		}
		break;
	case CSI_MEM2:
		if (set == true) {
			temp &= ~SMFC_WM2_SET_MASK;
			temp |= level << SMFC_WM2_SET_SHIFT;
		} else {
			temp &= ~SMFC_WM2_CLR_MASK;
			temp |= level << SMFC_WM2_CLR_SHIFT;
		}
		break;
	case CSI_MEM3:
		if (set == true) {
			temp &= ~SMFC_WM3_SET_MASK;
			temp |= level << SMFC_WM3_SET_SHIFT;
		} else {
			temp &= ~SMFC_WM3_CLR_MASK;
			temp |= level << SMFC_WM3_CLR_SHIFT;
		}
		break;
	default:
		return;
	}

	__raw_writel(temp, SMFC_WMC);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * _ipu_smfc_set_burst_size
 *
 * @param	channel		IDMAC channel 0-3
 * @param	bs		burst size of IDMAC channel,
 *				the value programmed here shoud be BURST_SIZE-1
 */
void _ipu_smfc_set_burst_size(ipu_channel_t channel, uint32_t bs)
{
	uint32_t temp;
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = __raw_readl(SMFC_BS);

	switch (channel) {
	case CSI_MEM0:
		temp &= ~SMFC_BS0_MASK;
		temp |= bs << SMFC_BS0_SHIFT;
		break;
	case CSI_MEM1:
		temp &= ~SMFC_BS1_MASK;
		temp |= bs << SMFC_BS1_SHIFT;
		break;
	case CSI_MEM2:
		temp &= ~SMFC_BS2_MASK;
		temp |= bs << SMFC_BS2_SHIFT;
		break;
	case CSI_MEM3:
		temp &= ~SMFC_BS3_MASK;
		temp |= bs << SMFC_BS3_SHIFT;
		break;
	default:
		return;
	}

	__raw_writel(temp, SMFC_BS);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}

/*!
 * _ipu_csi_init
 *
 * @param	channel      IDMAC channel
 * @param	csi	     csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_init(ipu_channel_t channel, uint32_t csi)
{
	uint32_t csi_sens_conf, csi_dest;
	int retval = 0;

	switch (channel) {
	case CSI_MEM0:
	case CSI_MEM1:
	case CSI_MEM2:
	case CSI_MEM3:
		csi_dest = CSI_DATA_DEST_IDMAC;
		break;
	case CSI_PRP_ENC_MEM:
	case CSI_PRP_VF_MEM:
		csi_dest = CSI_DATA_DEST_IC;
		break;
	default:
		retval = -EINVAL;
		goto err;
	}

	csi_sens_conf = __raw_readl(CSI_SENS_CONF(csi));
	csi_sens_conf &= ~CSI_SENS_CONF_DATA_DEST_MASK;
	__raw_writel(csi_sens_conf | (csi_dest <<
		CSI_SENS_CONF_DATA_DEST_SHIFT), CSI_SENS_CONF(csi));
err:
	return retval;
}
