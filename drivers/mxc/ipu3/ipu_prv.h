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
#ifndef __INCLUDE_IPU_PRV_H__
#define __INCLUDE_IPU_PRV_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>

/* Globals */
extern struct device *g_ipu_dev;
extern spinlock_t ipu_lock;
extern bool g_ipu_clk_enabled;
extern struct clk *g_ipu_clk;
extern struct clk *g_di_clk[2];
extern struct clk *g_pixel_clk[2];
extern struct clk *g_csi_clk[2];
extern unsigned char g_dc_di_assignment[];
extern int g_ipu_hw_rev;
extern int dmfc_type_setup;

#define IDMA_CHAN_INVALID	0xFF
#define HIGH_RESOLUTION_WIDTH	1024

struct ipu_channel {
	u8 video_in_dma;
	u8 alpha_in_dma;
	u8 graph_in_dma;
	u8 out_dma;
};

enum ipu_dmfc_type {
	DMFC_NORMAL = 0,
	DMFC_HIGH_RESOLUTION_DC,
	DMFC_HIGH_RESOLUTION_DP,
	DMFC_HIGH_RESOLUTION_ONLY_DP,
};

int register_ipu_device(void);
ipu_color_space_t format_to_colorspace(uint32_t fmt);
bool ipu_pixel_format_has_alpha(uint32_t fmt);

void ipu_dump_registers(void);

uint32_t _ipu_channel_status(ipu_channel_t channel);

void _ipu_init_dc_mappings(void);
int _ipu_dp_init(ipu_channel_t channel, uint32_t in_pixel_fmt,
		 uint32_t out_pixel_fmt);
void _ipu_dp_uninit(ipu_channel_t channel);
void _ipu_dc_init(int dc_chan, int di, bool interlaced, uint32_t pixel_fmt);
void _ipu_dc_uninit(int dc_chan);
void _ipu_dp_dc_enable(ipu_channel_t channel);
void _ipu_dp_dc_disable(ipu_channel_t channel, bool swap);
void _ipu_dmfc_init(int dmfc_type, int first);
void _ipu_dmfc_set_wait4eot(int dma_chan, int width);
void _ipu_dmfc_set_burst_size(int dma_chan, int burst_size);
int _ipu_disp_chan_is_interlaced(ipu_channel_t channel);

void _ipu_ic_enable_task(ipu_channel_t channel);
void _ipu_ic_disable_task(ipu_channel_t channel);
void _ipu_ic_init_prpvf(ipu_channel_params_t *params, bool src_is_csi);
void _ipu_vdi_init(ipu_channel_t channel, ipu_channel_params_t *params);
void _ipu_vdi_uninit(void);
void _ipu_ic_uninit_prpvf(void);
void _ipu_ic_init_rotate_vf(ipu_channel_params_t *params);
void _ipu_ic_uninit_rotate_vf(void);
void _ipu_ic_init_csi(ipu_channel_params_t *params);
void _ipu_ic_uninit_csi(void);
void _ipu_ic_init_prpenc(ipu_channel_params_t *params, bool src_is_csi);
void _ipu_ic_uninit_prpenc(void);
void _ipu_ic_init_rotate_enc(ipu_channel_params_t *params);
void _ipu_ic_uninit_rotate_enc(void);
void _ipu_ic_init_pp(ipu_channel_params_t *params);
void _ipu_ic_uninit_pp(void);
void _ipu_ic_init_rotate_pp(ipu_channel_params_t *params);
void _ipu_ic_uninit_rotate_pp(void);
int _ipu_ic_idma_init(int dma_chan, uint16_t width, uint16_t height,
		      int burst_size, ipu_rotate_mode_t rot);
void _ipu_vdi_toggle_top_field_man(void);
int _ipu_csi_init(ipu_channel_t channel, uint32_t csi);
void ipu_csi_set_test_generator(bool active, uint32_t r_value,
		uint32_t g_value, uint32_t b_value,
		uint32_t pix_clk, uint32_t csi);
void _ipu_csi_ccir_err_detection_enable(uint32_t csi);
void _ipu_csi_ccir_err_detection_disable(uint32_t csi);
void _ipu_smfc_init(ipu_channel_t channel, uint32_t mipi_id, uint32_t csi);
void _ipu_smfc_set_burst_size(ipu_channel_t channel, uint32_t bs);
void _ipu_dp_set_csc_coefficients(ipu_channel_t channel, int32_t param[][3]);
void _ipu_clear_buffer_ready(ipu_channel_t channel, ipu_buffer_t type,
			     uint32_t bufNum);

#endif				/* __INCLUDE_IPU_PRV_H__ */
