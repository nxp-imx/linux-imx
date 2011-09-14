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
#include <mach/hardware.h>

/* Globals */
extern struct device *g_ipu_dev;
extern spinlock_t ipu_lock;
extern struct clk *g_ipu_clk;
extern struct clk *g_ipu_csi_clk;

int register_ipu_device(void);
ipu_color_space_t format_to_colorspace(uint32_t fmt);

uint32_t _ipu_channel_status(ipu_channel_t channel);

void _ipu_sdc_fg_init(ipu_channel_params_t *params);
uint32_t _ipu_sdc_fg_uninit(void);
void _ipu_sdc_bg_init(ipu_channel_params_t *params);
uint32_t _ipu_sdc_bg_uninit(void);

void _ipu_ic_enable_task(ipu_channel_t channel);
void _ipu_ic_disable_task(ipu_channel_t channel);
void _ipu_ic_init_prpvf(ipu_channel_params_t *params, bool src_is_csi);
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

int32_t _ipu_adc_init_channel(ipu_channel_t chan, display_port_t disp,
			      mcu_mode_t cmd, int16_t x_pos, int16_t y_pos);
int32_t _ipu_adc_uninit_channel(ipu_channel_t chan);

#endif				/* __INCLUDE_IPU_PRV_H__ */
