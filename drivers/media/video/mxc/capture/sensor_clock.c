/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file sensor_clock.c
 *
 * @brief camera clock function
 *
 * @ingroup Camera
 */
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <mach/hardware.h>

#if defined(CONFIG_MXC_IPU_V1) || defined(CONFIG_VIDEO_MXC_EMMA_CAMERA) \
			       || defined(CONFIG_VIDEO_MXC_CSI_CAMERA_MODULE) \
			       || defined(CONFIG_VIDEO_MXC_CSI_CAMERA)
/*
 * set_mclk_rate
 *
 * @param       p_mclk_freq  mclk frequence
 *
 */
void set_mclk_rate(uint32_t *p_mclk_freq)
{
	struct clk *clk;
	uint32_t freq = 0;

	clk = clk_get(NULL, "csi_clk");

	freq = clk_round_rate(clk, *p_mclk_freq);
	clk_set_rate(clk, freq);

	*p_mclk_freq = freq;

	clk_put(clk);
	pr_debug("mclk frequency = %d\n", *p_mclk_freq);
}
#else
/*
 * set_mclk_rate
 *
 * @param       p_mclk_freq  mclk frequence
 * @param	csi         csi 0 or csi 1
 *
 */
void set_mclk_rate(uint32_t *p_mclk_freq, uint32_t csi)
{
	struct clk *clk;
	uint32_t freq = 0;
	char *mclk;

	if (cpu_is_mx53()) {
		if (csi == 0)
			mclk = "ssi_ext1_clk";
		else {
			pr_err("invalid csi num %d\n", csi);
			return;
		}
	} else {
		if (csi == 0) {
			mclk = "csi_mclk1";
		} else if (csi == 1) {
			mclk = "csi_mclk2";
		} else {
			pr_err("invalid csi num %d\n", csi);
			return;
		}
	}

	clk = clk_get(NULL, mclk);

	freq = clk_round_rate(clk, *p_mclk_freq);
	clk_set_rate(clk, freq);

	*p_mclk_freq = freq;

	clk_put(clk);
	pr_debug("%s frequency = %d\n", mclk, *p_mclk_freq);
}
#endif

/* Exported symbols for modules. */
EXPORT_SYMBOL(set_mclk_rate);
