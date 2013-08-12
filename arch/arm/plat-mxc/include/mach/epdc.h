/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __MACH_EPDC_H_
#define __MACH_EPDC_H_

struct imx_epdc_fb_mode {
    struct fb_videomode *vmode;
    int vscan_holdoff;
    int sdoed_width;
    int sdoed_delay;
    int sdoez_width;
    int sdoez_delay;
    int gdclk_hp_offs;
    int gdsp_offs;
    int gdoe_offs;
    int gdclk_offs;
    int num_ce;
};

struct imx_epdc_fb_platform_data {
    struct imx_epdc_fb_mode *epdc_mode;
    int num_modes;
    int (*get_pins) (void);
    void (*put_pins) (void);
    void (*enable_pins) (void);
    void (*disable_pins) (void);
};

struct imx_spdc_panel_init_set {
    bool yoe_pol;
    bool dual_gate;
    u8 resolution;
    bool ud;
    bool rl;
    bool data_filter_n;
    bool power_ready;
    bool rgbw_mode_enable;
    bool hburst_len_en;
};

struct imx_spdc_fb_mode {
    struct fb_videomode *vmode;
    struct imx_spdc_panel_init_set *init_set;
    const char *wave_timing;
};

struct imx_spdc_fb_platform_data {
    struct imx_spdc_fb_mode *spdc_mode;
    int num_modes;
    int (*get_pins) (void);
    void (*put_pins) (void);
    void (*enable_pins) (void);
    void (*disable_pins) (void);
};

#endif /* __MACH_EPDC_H_ */
