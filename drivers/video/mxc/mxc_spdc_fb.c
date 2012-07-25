/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 * Based on MXC EPDC Driver, Freescale Semiconductor, Inc. All Rights Reserved.
 */

#include "mxc_spdc_fb.h"

#define MERGE_OK	0
#define MERGE_FAIL      1
#define MERGE_BLOCK     2

#define SPDC_DEFAULT_TEMP	30
#define TEMP_NO_SET		0xFF
#define POWER_STATE_OFF		0
#define POWER_STATE_ON		1
#define POWER_READY_OFF		false
#define POWER_READY_ON		true

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define SPDC_MAX_NUM_UPDATES	32
#define SPDC_MAX_NUM_BUFFERS	2
#define SPDC_MAX_NUM_PREPROCESS	15
#define NUM_SCREENS_MIN		2
#define SPDC_DEFAULT_BPP	16

mxc_spdc_t *g_spdc_fb_data;

static int mxc_spdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info);
static int mxc_spdc_fb_wait_update_complete(struct mxcfb_update_marker_data
		*marker_data, struct fb_info *info);

static const struct mxc_spdc_resolution_map_para spdc_gray_res_map[] = {
/*define Gray Mode resolution mapping*/
	{0x18, 600, 800, PORTRAIT},
	{0x19, 768, 1024, PORTRAIT},
	{0x1a, 0,   0,    RESERVED},
	{0x1b, 600, 1024, PORTRAIT},
	{0x1c, 825, 1200, PORTRAIT},
	{0x1d, 1024, 1280, PORTRAIT},
	{0x1e, 1200, 1600, PORTRAIT},
	{0x10, 800, 1024, PORTRAIT},
	{0x11, 825, 1280, PORTRAIT},
	{0x12, 800, 1280, PORTRAIT},
	{0x13, 768, 1280, PORTRAIT},
	{0x14, 960, 1280, PORTRAIT},
	{0x0, 800, 600, LANDSCAPE},
	{0x1, 1024, 768, LANDSCAPE},
	{0x2, 0, 0, RESERVED},
	{0x3, 1024, 600, LANDSCAPE},
	{0x4, 1200, 825, LANDSCAPE},
	{0x5, 1280, 1024, LANDSCAPE},
	{0x6, 1600, 1200, LANDSCAPE},
	{0x7, 1024, 800, LANDSCAPE},
	{0x8, 1280, 825, LANDSCAPE},
	{0x9, 1280, 800, LANDSCAPE},
	{0xa, 1280, 768, LANDSCAPE},
	{0xb, 1280, 960, LANDSCAPE},
	{0xFFFF, 800, 600, LANDSCAPE},
};

static const struct mxc_spdc_resolution_map_para spdc_rgbw_res_map[] = {
/*define RGBW Mode resolution mapping*/
	{0x18, 300, 400, PORTRAIT},
	{0x19, 384, 512, PORTRAIT},
	{0x1a, 0,   0,    RESERVED},
	{0x1b, 300, 512, PORTRAIT},
	{0x1c, 0, 0, RESERVED},
	{0x1d, 512, 640, PORTRAIT},
	{0x1e, 600, 800, PORTRAIT},
	{0x10, 400, 512, PORTRAIT},
	{0x11, 0, 0, RESERVED},
	{0x12, 400, 640, PORTRAIT},
	{0x13, 384, 640, PORTRAIT},
	{0x14, 480, 640, PORTRAIT},
	{0x0, 400, 300, LANDSCAPE},
	{0x1, 512, 384, LANDSCAPE},
	{0x2, 0, 0, RESERVED},
	{0x3, 512, 300, LANDSCAPE},
	{0x4, 0, 0, RESERVED},
	{0x5, 640, 512, LANDSCAPE},
	{0x6, 800, 600, LANDSCAPE},
	{0x7, 512, 400, LANDSCAPE},
	{0x8, 0, 0, RESERVED},
	{0x9, 640, 400, LANDSCAPE},
	{0xa, 640, 384, LANDSCAPE},
	{0xb, 640, 480, LANDSCAPE},
	{0xFFFF, 400, 300, LANDSCAPE},
};

static void get_panel_init_set(struct imx_spdc_panel_init_set*
					panel_set, u32 *val)
{
	*val = panel_set->yoe_pol |
		(panel_set->dual_gate << 1) |
		(panel_set->ud << 7) |
		(panel_set->rl << 8) |
		(panel_set->data_filter_n  << 9) |
		(panel_set->power_ready  << 10) |
		(panel_set->rgbw_mode_enable  << 11) |
		(panel_set->hburst_len_en << 13) |
		((panel_set->resolution & 0x1F) << 2);
}

static inline void spdc_intr_enable(mxc_spdc_t *fb_data, u32 int_type)
{
	u32 status;

	status = __raw_readl(fb_data->hwp + SPDC_INT_ENABLE);
	status |= (int_type & SPDC_IRQ_ALL_MASK);
	__raw_writel(status, fb_data->hwp + SPDC_INT_ENABLE);
}

static bool spdc_is_update_finish(mxc_spdc_t *fb_data)
{
	u32 val =  __raw_readl(fb_data->hwp + SPDC_INT_STA_CLR);
	bool is_finish = (val & SPDC_IRQ_STA_FRAME_UPDATE) ? true : false;

	return is_finish;
}

static int spdc_get_intr_stat(mxc_spdc_t *fb_data)
{
	u32 status = __raw_readl(fb_data->hwp + SPDC_INT_STA_CLR);
	return status & 0xF;
}

static inline void
spdc_intr_stat_clear(mxc_spdc_t *fb_data, u32 int_type)
{
	/* write 1 to clear status */
	u32 status = (int_type & SPDC_IRQ_STA_ALL_MASK);
	__raw_writel(status, fb_data->hwp + SPDC_INT_STA_CLR);
}

static inline void spdc_set_nextbuf_addr(mxc_spdc_t *fb_data)
{
	u32 addr = fb_data->fresh_param.buf_addr.next_buf_phys_addr;
	__raw_writel(addr, fb_data->hwp + SPDC_NEXT_BUF);
	dev_dbg(fb_data->dev, "add: 0x%x\n", addr);
}

static inline void spdc_set_curbuf_addr(mxc_spdc_t *fb_data)
{
	u32 addr = fb_data->fresh_param.buf_addr.cur_buf_phys_addr;
	__raw_writel(addr, fb_data->hwp + SPDC_CURRENT_BUF);
}

static inline void spdc_set_prebuf_addr(mxc_spdc_t *fb_data)
{
	u32 addr = fb_data->fresh_param.buf_addr.pre_buf_phys_addr;
	__raw_writel(addr, fb_data->hwp + SPDC_PRE_BUF);
}

static inline void spdc_set_cntbuf_addr(mxc_spdc_t *fb_data)
{
	u32 addr = fb_data->fresh_param.buf_addr.frm_cnt_buf_phys_addr;
	__raw_writel(addr, fb_data->hwp + SPDC_CNT_BUF);
}

static inline void spdc_set_lutbuf_addr(mxc_spdc_t *fb_data)
{
	u32 addr = fb_data->fresh_param.buf_addr.lut_buf_phys_addr;
	__raw_writel(addr, fb_data->hwp + SPDC_LUT_BUF);
}

static inline void spdc_set_update_coord(mxc_spdc_t *fb_data)
{
	u32 x = fb_data->fresh_param.update_region.left;
	u32 y = fb_data->fresh_param.update_region.top;

	if (!x)
		x++;
	if (!y)
		y++;

	dev_dbg(fb_data->dev, "x:%d, y:%d\n", x, y);
	x = (u32)(((x & SPDC_UPDATE_X_Y_MAX_SIZE) << 16) |
			(y & SPDC_UPDATE_X_Y_MAX_SIZE));
	__raw_writel(x, fb_data->hwp + SPDC_UPDATA_X_Y);
}

static inline void spdc_set_update_dimensions(mxc_spdc_t *fb_data)
{
	u32 w = fb_data->fresh_param.update_region.width;
	u32 h = fb_data->fresh_param.update_region.height;

	if (!w)
		w++;
	if (!h)
		h++;

	dev_dbg(fb_data->dev, "w:%d, h:%d\n", w, h);
	w = (u32)(((w & SPDC_UPDATE_W_H_MAX_SIZE) << 16) |
			(h & SPDC_UPDATE_W_H_MAX_SIZE));
	__raw_writel(w, fb_data->hwp + SPDC_UPDATE_W_H);
}

static inline void spdc_set_update_temper(mxc_spdc_t *fb_data)
{
	s8 temper = (s8)(fb_data->fresh_param.temper & 0xFF) << 1;

	if (temper > -110 && temper < 200)
		__raw_writel(temper, fb_data->hwp + SPDC_TEMP_INFO);
	else
		__raw_writel(SPDC_DEFAULT_TEMP, fb_data->hwp + SPDC_TEMP_INFO);
}

static inline void spdc_trigger_update(mxc_spdc_t *fb_data)
{
	u32 val;
	struct partial_refresh_param *fresh_param = &fb_data->fresh_param;

	if ((fresh_param->wave_mode & SPDC_WAV_MODE_MASK) && fresh_param->flash)
		val = SPDC_DISP_TRIGGER_FLASH;
	else
		val = 0;

	val |= fresh_param->wave_mode << 1;
	val |= SPDC_DISP_TRIGGER_ENABLE;
	dev_dbg(fb_data->dev, "wave:%d\n", fresh_param->wave_mode);
	__raw_writel(val, fb_data->hwp + SPDC_DISP_TRIGGER);
}

static bool is_lut_checksum_ok(mxc_spdc_t *fb_data)
{
	u32 status;

	status = __raw_readl(fb_data->hwp + SPDC_STATUS);
	status &= SPDC_IRQ_STA_ERR;

	return status ? true : false;
}

static void spdc_clk_gate(mxc_spdc_t *fb_data, bool enable)
{
	if (enable)
		__raw_writel(SPDC_SW_GATE_CLK_ENABLE,
		fb_data->hwp + SPDC_SW_GATE_CLK);
	else
		__raw_writel(~SPDC_SW_GATE_CLK_ENABLE,
		fb_data->hwp + SPDC_SW_GATE_CLK);
}

static int update_panel_init_set(mxc_spdc_t *fb_data)
{
	int ret = 0;
	u32 init_val;

	get_panel_init_set(&fb_data->panel_set, &init_val);
	dev_dbg(fb_data->dev, "panel init setting:%x\n", init_val);

	__raw_writel(init_val, fb_data->hwp + SPDC_PANEL_INIT_SET);

	/*wait init setting update finish*/
	ret = wait_for_completion_timeout(&fb_data->init_finish,
				msecs_to_jiffies(4000));
	if (!ret)
		dev_err(fb_data->dev, "Timed out for init setting!\n");

	return ret;
}

static void spdc_panel_pwr_on(mxc_spdc_t *fb_data)
{
	fb_data->panel_set.power_ready = POWER_READY_ON;
}

static void spdc_panel_pwr_down(mxc_spdc_t *fb_data)
{
	fb_data->panel_set.power_ready = POWER_READY_OFF;
}

static void spdc_powerdown(mxc_spdc_t *fb_data)
{
	mutex_lock(&fb_data->power_mutex);

	/* If powering_down has been cleared, a powerup
	* request is pre-empting this powerdown request.
	*/
	if (!fb_data->powering_down
		|| (fb_data->power_state == POWER_STATE_OFF)) {
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	dev_dbg(fb_data->dev, "spdc Powerdown\n");

	/* Disable power to the AUO panel */
	regulator_disable(fb_data->vcom_regulator);
	regulator_disable(fb_data->display_regulator);

	/*enable spdc clock gating*/
	spdc_clk_gate(fb_data, true);
	clk_disable(fb_data->spdc_clk_pix);
	clk_disable(fb_data->spdc_clk_axi);

	/* Disable pins used by SPDC (to prevent leakage current) */
	if (fb_data->pdata->disable_pins)
		fb_data->pdata->disable_pins();

	/* turn off the V3p3 */
	regulator_disable(fb_data->v3p3_regulator);

	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	spdc_panel_pwr_down(fb_data);

	if (fb_data->wait_for_powerdown) {
		fb_data->wait_for_powerdown = false;
		complete(&fb_data->powerdown_compl);
	}

	mutex_unlock(&fb_data->power_mutex);
}

static void spdc_powerup(mxc_spdc_t *fb_data)
{
	int ret = 0;
	mutex_lock(&fb_data->power_mutex);

	/*
	 * If power down request is pending, clear
	 * powering_down to cancel the request.
	 */
	if (fb_data->powering_down)
		fb_data->powering_down = false;

	if (fb_data->power_state == POWER_STATE_ON) {
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	dev_dbg(fb_data->dev, "spdc Powerup\n");

	/* Enable the v3p3 regulator */
	ret = regulator_enable(fb_data->v3p3_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3 regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	msleep(1);

	/* Enable pins used by SPDC */
	if (fb_data->pdata->enable_pins)
		fb_data->pdata->enable_pins();

	/* Enable clocks to SPDC */
	clk_enable(fb_data->spdc_clk_axi);
	clk_enable(fb_data->spdc_clk_pix);

	/*disable spdc gate*/
	spdc_clk_gate(fb_data, false);

	/* Enable power to the EPD panel */
	ret = regulator_enable(fb_data->display_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable DISPLAY regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}
	ret = regulator_enable(fb_data->vcom_regulator);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable VCOM regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	fb_data->power_state = POWER_STATE_ON;
	spdc_panel_pwr_on(fb_data);

	mutex_unlock(&fb_data->power_mutex);
}

#ifdef DEBUG
static void
check_waveform(u32 *wv_buf_orig, u32 *wv_buf_cur, u32 wv_buf_size)
{
	int i;
	bool is_mismatch = false;
	for (i = 0; i < wv_buf_size; i++) {
		if (wv_buf_orig[i] != wv_buf_cur[i]) {
			is_mismatch = true;
			printk(KERN_ERR "Waveform mismatch!\n");
		}
	}

	if (!is_mismatch)
		printk(KERN_DEBUG "No mismatches!\n");
}
#else
static void
check_waveform(u32 *wv_buf_orig, u32 *wv_buf_cur, u32 wv_buf_size) {}
#endif

static void get_spdc_version(mxc_spdc_t *fb_data)
{
	struct mxc_spdc_version *spdc_ver = &fb_data->spdc_ver;
	u32 disp_id, tcon_id;

	disp_id = __raw_readl(fb_data->hwp + SPDC_DISP_VER);
	tcon_id = __raw_readl(fb_data->hwp + SPDC_TCON_VER);

	spdc_ver->disp_ver.product_id = disp_id & 0xFFFF;
	spdc_ver->disp_ver.lut_ver = (disp_id >> 16) & 0xFF;
	spdc_ver->disp_ver.epd_type = (disp_id >> 24) & 0xFF;
	spdc_ver->tcon_ver = tcon_id & 0xFF;

	dev_info(fb_data->dev, "EPD type ID:%x, Tcon ID:%x\n",
		spdc_ver->disp_ver.product_id, spdc_ver->tcon_ver);
}

static void spdc_set_update_concurrency(mxc_spdc_t *fb_data)
{
	u32 concur_mode;

	concur_mode = fb_data->fresh_param.concur & 0xFF;
	concur_mode |= (SPDC_LUT_MODE_OFFSET << 8);

	__raw_writel(concur_mode, fb_data->hwp + SPDC_LUT_PARA_UPDATE);
}

static bool is_preprocess_list_full(mxc_spdc_t *fb_data)
{
	/* Check to see if preprocess are full in this list */
	if (fb_data->upd_preprocess_num >= SPDC_MAX_NUM_PREPROCESS)
		return true;
	else
		return false;
}

static void spdc_submit_update(mxc_spdc_t *fb_data)
{
	fb_data->updates_active = true;

	spdc_set_nextbuf_addr(fb_data);
	spdc_set_update_coord(fb_data);
	spdc_set_update_dimensions(fb_data);
	spdc_set_update_temper(fb_data);
	spdc_trigger_update(fb_data);
}

static int spdc_init_sequence(mxc_spdc_t *fb_data)
{
	struct fb_var_screeninfo *screeninfo = &fb_data->spdc_fb_var;
	struct spdc_buffer_addr *buf_addr = &fb_data->fresh_param.buf_addr;
	struct imx_spdc_panel_init_set *init_set =
		fb_data->pdata->spdc_mode->init_set;
	u32 xres, yres;
	int ret = -EFAULT;

	/*init spdc power*/
	spdc_powerup(fb_data);

	/* enable all interrupt */
	spdc_intr_stat_clear(fb_data, SPDC_INT_STA_CLR);
	spdc_intr_enable(fb_data, SPDC_IRQ_ALL_MASK);
	/* set ACC concurrency update mode */
	if (fb_data->fresh_param.concur)
		spdc_set_update_concurrency(fb_data);

	/* program SPDC register and trigger to process buffer*/
	buf_addr->next_buf_phys_addr = fb_data->phy_next_buf;
	buf_addr->cur_buf_phys_addr = fb_data->phy_current_buf;
	buf_addr->pre_buf_phys_addr = fb_data->phy_pre_buf;
	buf_addr->frm_cnt_buf_phys_addr = fb_data->phy_cnt_buf;
	buf_addr->lut_buf_phys_addr = fb_data->phy_lut_buf;

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}
	fb_data->fresh_param.update_region.left = 0;
	fb_data->fresh_param.update_region.top = 0;
	fb_data->fresh_param.update_region.width = xres;
	fb_data->fresh_param.update_region.height = yres;

	/* set panel temperature as environment temperature */
	fb_data->fresh_param.temper = SPDC_DEFAULT_TEMP;
	/* set waveform mode */
	fb_data->fresh_param.wave_mode = SPDC_WAV_MODE_DEFAULT;

	spdc_set_update_coord(fb_data);
	spdc_set_update_dimensions(fb_data);

	spdc_set_update_temper(fb_data);

	spdc_set_nextbuf_addr(fb_data);
	spdc_set_curbuf_addr(fb_data);
	spdc_set_prebuf_addr(fb_data);
	spdc_set_cntbuf_addr(fb_data);

	/* load waveform*/
	spdc_set_lutbuf_addr(fb_data);
	ret = wait_for_completion_timeout(&fb_data->lut_down,
					msecs_to_jiffies(4000));
	if (!ret) {
		dev_err(fb_data->dev,
			"Timed out for lut!\n");
		return ret;
	}

	/* modify lut to fix DC com fading issue,
	 * and fix mode4 close over push
	 */
	__raw_writel(0x00003906, fb_data->hwp + SPDC_LUT_PARA_UPDATE);
	__raw_writel(0x00003300, fb_data->hwp + SPDC_LUT_PARA_UPDATE);

	/* init SPDC setting, the setting get from platform data */
	fb_data->panel_set.yoe_pol = init_set->yoe_pol;
	fb_data->panel_set.dual_gate = init_set->dual_gate;
	fb_data->panel_set.ud = init_set->ud;
	fb_data->panel_set.rl = init_set->rl;
	fb_data->panel_set.data_filter_n = init_set->data_filter_n;
	fb_data->panel_set.rgbw_mode_enable = init_set->rgbw_mode_enable;
	fb_data->panel_set.hburst_len_en = init_set->hburst_len_en;
	ret = update_panel_init_set(fb_data);

	return ret;
}

static u32 mxc_spdc_partial_refresh_low(mxc_spdc_t *fb_data, void *buffer)
{
	u8 *fresh_addr;
	void *pattern = buffer;
	struct partial_refresh_param *fresh_param = &fb_data->fresh_param;
	u32 fresh_size;
	int ret = 0;

	fb_data->updates_active = true;

	fresh_addr = (u8 *)(fb_data->virt_start) +
		(fresh_param->update_region.top * fresh_param->stride) +
		((fresh_param->update_region.left * fb_data->default_bpp) >> 3);
	fresh_size = (u32)((fresh_param->update_region.width *
		fresh_param->update_region.height * fb_data->default_bpp) >> 3);

	if (buffer != NULL) {
		while (fresh_size > 0) {
			memcpy((void *)fresh_addr, pattern,
				fresh_param->update_region.width);
			fresh_size -= fresh_param->update_region.width;
			fresh_addr += fresh_param->update_region.top *
					fresh_param->stride;
			pattern +=  fresh_param->update_region.width;
		}
	}

	/* program SPDC register and trigger to process buffer*/
	fb_data->fresh_param.buf_addr.next_buf_phys_addr =
		fb_data->info.fix.smem_start + fb_data->fb_offset;
	fb_data->fresh_param.wave_mode = fb_data->wv_modes.mode_init;
	spdc_submit_update(fb_data);

	ret = wait_for_completion_timeout(&fb_data->update_finish,
					msecs_to_jiffies(3000));
	if (!ret) {
		dev_err(fb_data->dev,
			"display update timeout!\n");
		return -ETIMEDOUT;
	}

	return ret;
}

static u32 spdc_fb_dev_init(mxc_spdc_t *fb_data)
{
	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->fresh_param.wave_mode = SPDC_WAV_MODE_0;
	fb_data->operation_mode = SPDC_NO_OPERATION;
	fb_data->is_deep_fresh = false;

	/* Init the concurrency update */
	fb_data->fresh_param.concur = 0;
	fb_data->upd_preprocess_num = 0;
	fb_data->submit_upd_sta = 0;

	fb_data->fresh_param.temper = SPDC_DEFAULT_TEMP;

	return 0;
}

/**
 * mxc_spdc_device_is_busy - check spdc device busy status.
 * Returns 0 if spdc device is idle.
 */
static int mxc_spdc_device_is_busy(mxc_spdc_t *fb_data)
{
	u32 status;
	u32 orig_jiffies = jiffies;

	while (1) {
		status = __raw_readl(fb_data->hwp + SPDC_STATUS);
		if ((status & SPDC_PANEL_STAUTS_BUSY) &&
			((status & 0xF0) == SPDC_TCON_STATUS_IDLE))
			break;

		if (signal_pending(current)) {
			dev_dbg(fb_data->dev, "SPDC Interrupted\n");
			return -EINTR;
		}

		if (time_after(jiffies, orig_jiffies +
				msecs_to_jiffies(3000))) {
			dev_dbg(fb_data->dev, "SPDC is busy\n");
			 return -ETIMEDOUT;
		}

		schedule();
	}

	return 0;
}

static bool is_free_list_full(mxc_spdc_t *fb_data)
{
	int count = 0;
	struct update_data_list *plist;

	/* Count buffers in free buffer list */
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		count++;

	/* Check to see if all buffers are in this list */
	if (count == fb_data->max_num_updates)
		return true;
	else
		return false;
}

static void spdc_draw_mode0(mxc_spdc_t *fb_data)
{
	struct mxcfb_update_data update;
	struct mxcfb_update_marker_data upd_marker_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->spdc_fb_var;
	u32 xres, yres;
	int ret;

	fb_data->fresh_param.buf_addr.next_buf_phys_addr =
		fb_data->phys_start;

	fb_data->hw_ready = true;
	fb_data->hw_initializing = false;

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}

	update.update_region.left = 0;
	update.update_region.width = xres;
	update.update_region.top = 0;
	update.update_region.height = yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.waveform_mode = fb_data->wv_modes.mode_init;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = SPDC_DEFAULT_TEMP;
	update.flags = 0;

	upd_marker_data.update_marker = update.update_marker;

	mxc_spdc_fb_send_update(&update, &fb_data->info);

	/* Block on initial update */
	ret = mxc_spdc_fb_wait_update_complete(&upd_marker_data,
		&fb_data->info);
	if (ret < 0)
		dev_err(fb_data->dev,
		"Wait for update complete failed, Err:%d", ret);
}

static void
spdc_fb_fw_handler(const struct firmware *fw, void *context)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)context;
	struct clk *spdc_parent;
	unsigned long rounded_parent_rate, spdc_pix_rate,
			rounded_pix_clk, target_pix_clk;
	u8 *wv_file;
	int ret;

	if (fw == NULL) {
		/* If default FW file load failed, we give up */
		if (fb_data->fw_default_load)
			return;

		/* Try to load default waveform */
		fb_data->fw_default_load = true;

		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				fb_data->fw_str, fb_data->dev, GFP_KERNEL,
				fb_data, spdc_fb_fw_handler);
		if (ret) {
			dev_err(fb_data->dev,
			"Failed to load waveform image with err %d\n", ret);
			return;
		}
	}

	wv_file = (u8 *)fw->data;
	memcpy(fb_data->virt_lut_buf, wv_file, fw->size);

	check_waveform((u32 *)wv_file, (u32 *)fb_data->virt_lut_buf,
					fw->size / 4);
	release_firmware(fw);

	/* Enable clocks to access SPDC regs */
	clk_enable(fb_data->spdc_clk_axi);

	target_pix_clk = fb_data->cur_mode->vmode->pixclock;
	/* Enable pix clk for SPDC */
	clk_enable(fb_data->spdc_clk_pix);
	rounded_pix_clk = clk_round_rate(fb_data->spdc_clk_pix, target_pix_clk);

	if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
		(rounded_pix_clk <= target_pix_clk - target_pix_clk/100))) {
		/* Can't get close enough without changing parent clk */
		spdc_parent = clk_get_parent(fb_data->spdc_clk_pix);
		rounded_parent_rate =
			clk_round_rate(spdc_parent, target_pix_clk);

		spdc_pix_rate = target_pix_clk;
		while (spdc_pix_rate < rounded_parent_rate)
			spdc_pix_rate *= 2;
		clk_set_rate(spdc_parent, spdc_pix_rate);

		rounded_pix_clk =
			clk_round_rate(fb_data->spdc_clk_pix, target_pix_clk);
		if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
		(rounded_pix_clk <= target_pix_clk - target_pix_clk/100)))
			/* Still can't get a good clock, provide warning */
			dev_err(fb_data->dev,
				"Unable to get an accurate SPDC pix clk"
				"desired = %lu, actual = %lu\n", target_pix_clk,
				rounded_pix_clk);
	}

	clk_set_rate(fb_data->spdc_clk_pix, rounded_pix_clk);
	/* Disable clocks */
	clk_disable(fb_data->spdc_clk_axi);
	clk_disable(fb_data->spdc_clk_pix);

	if (!spdc_init_sequence(fb_data))
		return;

	/* display log on picture */
	spdc_draw_mode0(fb_data);
}


static int spdc_fb_init_hw(struct fb_info *info)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int ret;

	fb_data->fw_default_load = false;
	/*
	 * Create fw search string based on ID string in selected videomode.
	 * Format is "imx/spdc_[wave_timing].fw: spdc_pvi.fw, spdc_auo.fw"
	 */
	if (fb_data->cur_mode) {
		memset(fb_data->fw_str, 0, sizeof(fb_data->fw_str));
		strcat(fb_data->fw_str, "imx/spdc_");
		strcat(fb_data->fw_str, fb_data->cur_mode->wave_timing);
		strcat(fb_data->fw_str, ".fw");
	} else
		strcat(fb_data->fw_str, "imx/spdc_pvi.fw");

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				fb_data->fw_str, fb_data->dev, GFP_KERNEL,
				fb_data, spdc_fb_fw_handler);
	if (ret) {
		dev_err(fb_data->dev,
			"Failed to load waveform image with err %d\n", ret);
		return ret;
	}

	return ret;
}

static int mxc_spdc_partial_refresh(mxc_spdc_t *fb_data, void *buffer)
{
	int ret = 0;
	struct partial_refresh_param *fresh_param = &fb_data->fresh_param;

	if (!fb_data->panel_set.power_ready)
		spdc_powerup(fb_data);

	if (!fresh_param->blocking) {
		if (mxc_spdc_device_is_busy(fb_data)) {
			dev_err(fb_data->dev, "spdc busy!\n");
			return (u32) -1;
		}
	} else {
		while (mxc_spdc_device_is_busy(fb_data)) {
			dev_err(fb_data->dev, "Waiting for spdc idle..\n");
			msleep(500);
		}
	}

	ret = mxc_spdc_partial_refresh_low(fb_data, buffer);

	return ret;
}

static int mxc_operaton_update(mxc_spdc_t *fb_data)
{
	int ret = 0;
	struct partial_refresh_param *fresh_param = &fb_data->fresh_param;
	u32 operation_mode = fb_data->operation_mode;

	if (!fb_data->panel_set.power_ready)
		spdc_powerup(fb_data);

	if (operation_mode != SPDC_SW_TCON_RESET) {
		if (!fresh_param->blocking) {
			if (mxc_spdc_device_is_busy(fb_data)) {
				dev_err(fb_data->dev,  "spdc busy\n");
				return (u32) -1;
			}
		} else {
			while (mxc_spdc_device_is_busy(fb_data)) {
				dev_err(fb_data->dev, "Waiting spdc idle...\n");
				msleep(500);
			}
		}
	} else
		operation_mode = SPDC_SW_TCON_RESET_SET;

	/* don't add to queue list */
	mutex_lock(&fb_data->queue_mutex);
	__raw_writel(operation_mode, fb_data->hwp + SPDC_OPERATE);
	mutex_unlock(&fb_data->queue_mutex);

	if (operation_mode == SPDC_SW_TCON_RESET_SET) {
		dev_dbg(fb_data->dev, "reinit hw\n");
		mdelay(500);

		fb_data->hw_ready = false;
		fb_data->operation_mode = SPDC_NO_OPERATION;
		ret = spdc_fb_init_hw(&fb_data->info);
		if (ret && !fb_data->hw_ready)
			dev_err(fb_data->dev, "Failed to init HW!\n");
	}

	return ret;
}

static int mxc_spdc_refresh_display(mxc_spdc_t *fb_data)
{
	struct partial_refresh_param *fresh_param = &fb_data->fresh_param;
	u32 operation_mode = fb_data->operation_mode;
	int ret = 0;

	fresh_param->update_region.left = 0;
	fresh_param->update_region.top = 0;
	fresh_param->update_region.width = fb_data->spdc_fb_var.xres;
	fresh_param->update_region.height = fb_data->spdc_fb_var.yres;
	fresh_param->stride = (fb_data->spdc_fb_var.xres *
				fb_data->spdc_fb_var.bits_per_pixel) >> 3;

	if (operation_mode && operation_mode < SPDC_FULL_REFRESH)
		ret = mxc_operaton_update(fb_data);
	else
		ret = mxc_spdc_partial_refresh(fb_data, NULL);

	return ret;
}

static void mxc_spdc_find_match_mode(mxc_spdc_t *fb_data)
{
	struct imx_spdc_fb_mode *spdc_mode =
		&fb_data->pdata->spdc_mode[0];
	const struct mxc_spdc_resolution_map_para *spdc_res_map;
	u32 i = 0;
	u32 j = 0;
	u32 default_mode = 0xFF;

	if (fb_data->panel_set.rgbw_mode_enable)
		spdc_res_map = &spdc_rgbw_res_map[0];
	else
		spdc_res_map = &spdc_gray_res_map[0];

	while (spdc_mode != NULL) {
		while (spdc_res_map[j].resolution != 0xFFFF) {
			if (spdc_mode->vmode->xres == spdc_res_map[j].res_x
			&& spdc_mode->vmode->yres == spdc_res_map[j].res_y) {
				fb_data->panel_set.resolution =
					spdc_res_map[j].resolution;
				default_mode = i;
				break;
			}
			j++;
		}

		if (default_mode != 0xFF)
			break;
		j = 0;
		i++;
		spdc_mode = &fb_data->pdata->spdc_mode[i];
	}

	fb_data->cur_mode = spdc_mode;
}

static int mxc_spdc_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED | VM_IO;

	while (size > 0) {
		page = pos;
		if (io_remap_pfn_range(vma, start, page >> PAGE_SHIFT,
					PAGE_SIZE, vma->vm_page_prot))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxc_spdc_fb_setcolreg(u_int regno, u_int red, u_int green,
				u_int blue, u_int transp, struct fb_info *info)
{
	if (regno >= 256) /* no. of hw registers */
		return 1;

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8); /* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		if (regno >= 16)
			return 1;

		((u32 *) (info->pseudo_palette))[regno] =
		(red << info->var.red.offset) |
		(green << info->var.green.offset) |
		(blue << info->var.blue.offset) |
		(transp << info->var.transp.offset);
	}

	return 0;
}

void mxc_spdc_fb_flush_updates(mxc_spdc_t *fb_data)
{
	int ret;

	/* Grab queue lock to prevent any new updates from being submitted */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * 3 places to check for updates that are active or pending:
	 *   1) Updates in the pending list
	 *   2) Update buffers in use (e.g., PxP processing)
	 *   3) Active updates to panel - We can key off of SPDC
	 *      power state to know if we have active updates.
	 */
	if (!list_empty(&fb_data->upd_pending_list) ||
		!is_free_list_full(fb_data) ||
		(fb_data->updates_active == true)) {
		/* Initialize event signalling updates are done */
		init_completion(&fb_data->updates_done);
		fb_data->waiting_for_idle = true;

		mutex_unlock(&fb_data->queue_mutex);
		/* Wait for any currently active updates to complete */
		ret = wait_for_completion_timeout(&fb_data->updates_done,
						msecs_to_jiffies(8000));
		if (!ret)
			dev_err(fb_data->dev,
				"Flush updates timeout! ret = 0x%x\n", ret);

		mutex_lock(&fb_data->queue_mutex);
		fb_data->waiting_for_idle = false;
	}

	mutex_unlock(&fb_data->queue_mutex);
}

static int mxc_spdc_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int i;

	dev_dbg(fb_data->dev, "setcmap\n");

	if (info->fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR) {
		/* Only support an 8-bit, 256 entry lookup */
		if (cmap->len != 256)
			return 1;

		mxc_spdc_fb_flush_updates(fb_data);

		mutex_lock(&fb_data->pxp_mutex);
		/*
		 * Store colormap in pxp_conf structure for later transmit
		 * to PxP during update process to convert gray pixels.
		 *
		 * Since red=blue=green for pseudocolor visuals, we can
		 * just use red values.
		 */
		for (i = 0; i < 256; i++)
			fb_data->pxp_conf.proc_data.lut_map[i] =
					cmap->red[i] & 0xFF;

		fb_data->pxp_conf.proc_data.lut_map_updated = true;

		mutex_unlock(&fb_data->pxp_mutex);
	} else {
		red = cmap->red;
		green   = cmap->green;
		blue    = cmap->blue;
		transp  = cmap->transp;
		index   = cmap->start;

		for (count = 0; count < cmap->len; count++) {
			if (transp)
				trans = *transp++;
			r = mxc_spdc_fb_setcolreg(index++, *red++,
				*green++, *blue++, trans, info);
			if (r != 0)
				return r;
		}
	}

	return 0;
}

static void adjust_coordinates(u32 xres, u32 yres, u32 rotation,
	struct mxcfb_rect *update_region, struct mxcfb_rect *adj_update_region)
{
	u32 temp;

	/* If adj_update_region == NULL, pass result back in update_region */
	/* If adj_update_region == valid, use it to pass back result */
	if (adj_update_region)
		switch (rotation) {
		case FB_ROTATE_UR:
			adj_update_region->top = update_region->top;
			adj_update_region->left = update_region->left;
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			break;
		case FB_ROTATE_CW:
			adj_update_region->top = update_region->left;
			adj_update_region->left = yres -
				(update_region->top + update_region->height);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		case FB_ROTATE_UD:
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			adj_update_region->top = yres -
				(update_region->top + update_region->height);
			adj_update_region->left = xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			adj_update_region->left = update_region->top;
			adj_update_region->top = xres -
				(update_region->left + update_region->width);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		}
	else
		switch (rotation) {
		case FB_ROTATE_UR:
			/* No adjustment needed */
			break;
		case FB_ROTATE_CW:
			temp = update_region->top;
			update_region->top = update_region->left;
			update_region->left = yres -
				(temp + update_region->height);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		case FB_ROTATE_UD:
			update_region->top = yres -
				(update_region->top + update_region->height);
			update_region->left = xres -
				(update_region->left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			temp = update_region->left;
			update_region->left = update_region->top;
			update_region->top = xres -
				(temp + update_region->width);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		}
}

static int mxc_spdc_fb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
		fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

static int mxc_spdc_fb_set_par(struct fb_info *info)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct imx_spdc_fb_mode *spdc_modes = fb_data->pdata->spdc_mode;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	int i, ret;
	__u32 xoffset_old, yoffset_old;

	/*
	 * Can't change the FB parameters until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_spdc_fb_flush_updates(fb_data);

	mutex_lock(&fb_data->queue_mutex);
	/*
	 * Set all screeninfo except for xoffset/yoffset
	 * Subsequent call to pan_display will handle those.
	 */
	xoffset_old = fb_data->spdc_fb_var.xoffset;
	yoffset_old = fb_data->spdc_fb_var.yoffset;
	fb_data->spdc_fb_var = *screeninfo;
	fb_data->spdc_fb_var.xoffset = xoffset_old;
	fb_data->spdc_fb_var.yoffset = yoffset_old;
	mutex_unlock(&fb_data->queue_mutex);

	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */
	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GY04;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = screeninfo->xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GY04;

	mutex_unlock(&fb_data->pxp_mutex);

	/* active new config, If HW not yet initialized,
	 * check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;
		bool found_match = false;
		u32 xres_temp;

		fb_var_to_videomode(&mode, screeninfo);

		/* When comparing requested fb mode,
		 * we need to use unrotated dimensions
		 */
		if ((screeninfo->rotate == FB_ROTATE_CW) ||
			(screeninfo->rotate == FB_ROTATE_CCW)) {
			xres_temp = mode.xres;
			mode.xres = mode.yres;
			mode.yres = xres_temp;
		}

		/* Match videomode against spdc modes */
		for (i = 0; i < fb_data->pdata->num_modes; i++) {
			if (!fb_mode_is_equal(spdc_modes[i].vmode, &mode))
				continue;
			fb_data->cur_mode = &spdc_modes[i];
			found_match = true;
			break;
		}

		if (!found_match) {
			dev_err(fb_data->dev,
				"Failed to match requested video mode\n");
			return EINVAL;
		}

		/* Initialize SPDC settings and init panel */
		ret =
		spdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(fb_data->dev,
				"Failed to load panel waveform data\n");
			return ret;
		}
	}

	mxc_spdc_fb_set_fix(info);

	return 0;
}

static int
mxc_spdc_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;

	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
		(var->bits_per_pixel != 16) && (var->bits_per_pixel != 8) &&
		(var->bits_per_pixel != 4))
		var->bits_per_pixel = SPDC_DEFAULT_BPP;

	switch (var->bits_per_pixel) {
	case 4:
		var->red.offset = 0;
		var->red.length = var->bits_per_pixel;
		var->green = var->red;
		var->blue = var->red;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 8:
		if (var->grayscale != 0) {
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green = var->red;
			var->blue = var->red;
			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
			var->red.length = 3;
			var->red.offset = 5;
			var->red.msb_right = 0;

			var->green.length = 3;
			var->green.offset = 2;
			var->green.msb_right = 0;

			var->blue.length = 2;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		}
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width;
		var->yres = fb_data->native_height;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height;
		var->yres = fb_data->native_width;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_dbg(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	return 0;
}

void mxc_spdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
	struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;

	mutex_lock(&fb_data->queue_mutex);

	memcpy(&fb_data->wv_modes, modes, sizeof(struct mxcfb_waveform_modes));

	mutex_unlock(&fb_data->queue_mutex);
}
EXPORT_SYMBOL(mxc_spdc_fb_set_waveform_modes);

/* To stick with non-fractional degrees for the sake
 *  of API consistency with EPDC.
 */
int mxc_spdc_fb_set_temperature(int temperature, struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;
	s8 temper = (s8)(temperature & 0xFF) << 1;

	mutex_lock(&fb_data->queue_mutex);

	if (temper > -110 && temper < 200)
		__raw_writel(temper, fb_data->hwp + SPDC_TEMP_INFO);
	else
		__raw_writel(SPDC_DEFAULT_TEMP, fb_data->hwp + SPDC_TEMP_INFO);

	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}
EXPORT_SYMBOL(mxc_spdc_fb_set_temperature);


int mxc_spdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;

	dev_dbg(fb_data->dev, "Setting auto update mode to %d\n", auto_mode);

	if ((auto_mode == AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		|| (auto_mode == AUTO_UPDATE_MODE_REGION_MODE))
		fb_data->auto_mode = auto_mode;
	else {
		dev_err(fb_data->dev, "Invalid auto update mode parameter.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_spdc_fb_set_auto_update);


int mxc_spdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;

	dev_dbg(fb_data->dev, "Setting optimization level to %d\n", upd_scheme);

	/*
	 * Can't change the scheme until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_spdc_fb_flush_updates(fb_data);

	if ((upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE_AND_MERGE))
		fb_data->upd_scheme = upd_scheme;
	else {
		dev_err(fb_data->dev, "Invalid update scheme specified.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_spdc_fb_set_upd_scheme);

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	mxc_spdc_t *fb_data = pxp_chan->client;

	/* This call will signal wait_for_completion_timeout()
	 * in send_buffer_to_pxp
	 */
	complete(&fb_data->pxp_tx_cmpl);
}

static bool chan_filter(struct dma_chan *chan, void *arg)
{
	if (imx_dma_is_pxp(chan))
		return true;
	else
		return false;
}

/* Function to request PXP DMA channel */
static int pxp_chan_init(mxc_spdc_t *fb_data)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;

	/*
	 * Request a free channel
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	chan = dma_request_channel(mask, chan_filter, NULL);
	if (!chan) {
		dev_err(fb_data->dev, "Unsuccessfully received channel!!!!\n");
		return -EBUSY;
	}

	fb_data->pxp_chan = to_pxp_channel(chan);
	fb_data->pxp_chan->client = fb_data;

	init_completion(&fb_data->pxp_tx_cmpl);

	return 0;
}

/*
 * Function to call PxP DMA driver and send our latest FB update region
 * through the PxP and out to an intermediate buffer.
 * Note: This is a blocking call, so upon return the PxP tx should be complete.
 */
static int pxp_process_update(mxc_spdc_t *fb_data,
			  u32 src_width, u32 src_height,
			  struct mxcfb_rect *update_region)
{
	dma_cookie_t cookie;
	struct scatterlist *sg = fb_data->sg;
	struct dma_chan *dma_chan;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &fb_data->pxp_conf.proc_data;
	int i, ret;
	int length;

	/* First, check to see that we have acquired a PxP Channel object */
	if (fb_data->pxp_chan == NULL) {
		/*
		 * PxP Channel has not yet been created and initialized,
		 * so let's go ahead and try
		 */
		ret = pxp_chan_init(fb_data);
		if (ret) {
			/*
			 * PxP channel init failed, and we can't use the
			 * PxP until the PxP DMA driver has loaded, so we abort
			 */
			dev_err(fb_data->dev, "PxP chan init failed\n");
			return -ENODEV;
		}
	}

	/*
	 * Init completion, so that we
	 * can be properly informed of the completion
	 * of the PxP task when it is done.
	 */
	init_completion(&fb_data->pxp_tx_cmpl);

	dma_chan = &fb_data->pxp_chan->dma_chan;

	txd = dma_chan->device->device_prep_slave_sg(dma_chan, sg, 2,
						 DMA_TO_DEVICE,
						 DMA_PREP_INTERRUPT);
	if (!txd) {
		dev_err(fb_data->info.device,
			"Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	/*
	 * Configure PxP for processing of new update region
	 * The rest of our config params were set up in
	 * probe() and should not need to be changed.
	 */
	pxp_conf->s0_param.width = src_width;
	pxp_conf->s0_param.height = src_height;
	proc_data->srect.top = update_region->top;
	proc_data->srect.left = update_region->left;
	proc_data->srect.width = update_region->width;
	proc_data->srect.height = update_region->height;

	/*
	 * Because only YUV/YCbCr image can be scaled, configure
	 * drect equivalent to srect, as such do not perform scaling.
	 */
	proc_data->drect.top = 0;
	proc_data->drect.left = 0;
	proc_data->drect.width = proc_data->srect.width;
	proc_data->drect.height = proc_data->srect.height;

	/* PXP expects rotation in terms of degrees */
	proc_data->rotate = fb_data->spdc_fb_var.rotate * 90;
	if (proc_data->rotate > 270)
		proc_data->rotate = 0;

	pxp_conf->out_param.width = update_region->width;
	pxp_conf->out_param.height = update_region->height;

	if ((proc_data->rotate == 90) || (proc_data->rotate == 270))
		pxp_conf->out_param.stride = update_region->height;
	else
		pxp_conf->out_param.stride = update_region->width;

	desc = to_tx_desc(txd);
	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {/* S0 */
			memcpy(&desc->proc_data, proc_data,
				sizeof(struct pxp_proc_data));
			pxp_conf->s0_param.paddr = sg_dma_address(&sg[0]);
			memcpy(&desc->layer_param.s0_param, &pxp_conf->s0_param,
				sizeof(struct pxp_layer_param));
		} else if (i == 1) {
			pxp_conf->out_param.paddr = sg_dma_address(&sg[1]);
			memcpy(&desc->layer_param.out_param,
					&pxp_conf->out_param,
					sizeof(struct pxp_layer_param));
		}
		/* TODO: OverLay */

		desc = desc->next;
	}

	/* Submitting our TX starts the PxP processing task */
	cookie = txd->tx_submit(txd);
	if (cookie < 0) {
		dev_err(fb_data->info.device, "Error sending FB through PxP\n");
		return -EIO;
	}

	fb_data->txd = txd;

	/* trigger ePxP */
	dma_async_issue_pending(dma_chan);

	return 0;
}

static int pxp_complete_update(mxc_spdc_t *fb_data, u32 *hist_stat)
{
	int ret;
	/*
	 * Wait for completion event, which will be set
	 * through our TX callback function.
	 */
	ret = wait_for_completion_timeout(&fb_data->pxp_tx_cmpl, HZ / 10);
	if (ret <= 0) {
		dev_info(fb_data->info.device,
			 "PxP operation failed due to %s\n",
			 ret < 0 ? "user interrupt" : "timeout");
		dma_release_channel(&fb_data->pxp_chan->dma_chan);
		fb_data->pxp_chan = NULL;
		return ret ? : -ETIMEDOUT;
	}

	if ((fb_data->pxp_conf.proc_data.lut_transform & EPDC_FLAG_USE_CMAP) &&
		fb_data->pxp_conf.proc_data.lut_map_updated)
		fb_data->pxp_conf.proc_data.lut_map_updated = false;

	*hist_stat = to_tx_desc(fb_data->txd)->hist_status;
	dma_release_channel(&fb_data->pxp_chan->dma_chan);
	fb_data->pxp_chan = NULL;

	dev_dbg(fb_data->dev, "TX completed\n");

	return 0;
}

static void copy_to_next_buffer(mxc_spdc_t *fb_data,
	struct update_data_list *upd_data_list)
{
	struct mxcfb_update_data *upd_data =
		&upd_data_list->update_desc->upd_data;
	unsigned char *temp_buf_ptr = fb_data->virt_addr_copybuf;
	unsigned char *dst_ptr = upd_data_list->virt_addr;
	struct mxcfb_rect adj_update_region;
	int dst_stride, left_offs, line_width;
	int i;

	switch (fb_data->spdc_fb_var.rotate) {
	case FB_ROTATE_UR:
		adj_update_region.top = upd_data->update_region.top;
		adj_update_region.left = upd_data->update_region.left;
		adj_update_region.width = upd_data->update_region.width;
		adj_update_region.height = upd_data->update_region.height;
		dst_stride = fb_data->spdc_fb_var.xres_virtual / 2;
		break;
	case FB_ROTATE_CW:
		adj_update_region.top = upd_data->update_region.left;
		adj_update_region.left = fb_data->spdc_fb_var.yres -
				(upd_data->update_region.top +
				upd_data->update_region.height);
		adj_update_region.width = upd_data->update_region.height;
		adj_update_region.height = upd_data->update_region.width;
		dst_stride = fb_data->spdc_fb_var.yres / 2;
		break;
	case FB_ROTATE_UD:
		adj_update_region.width = upd_data->update_region.width;
		adj_update_region.height = upd_data->update_region.height;
		adj_update_region.top = fb_data->spdc_fb_var.yres -
		(upd_data->update_region.top + upd_data->update_region.height);
		adj_update_region.left = fb_data->spdc_fb_var.xres -
				(upd_data->update_region.left +
				upd_data->update_region.width);
		dst_stride = fb_data->spdc_fb_var.xres_virtual / 2;
		break;
	case FB_ROTATE_CCW:
		adj_update_region.left = upd_data->update_region.top;
		adj_update_region.top = fb_data->spdc_fb_var.xres -
				(upd_data->update_region.left +
				upd_data->update_region.width);
		adj_update_region.width = upd_data->update_region.height;
		adj_update_region.height = upd_data->update_region.width;
		dst_stride = fb_data->spdc_fb_var.yres / 2;
		break;
	}

	/* pxp output Y4 data.
	 * Copy the raw data to related region in next buffer.
	 */
	left_offs = adj_update_region.left / 2;
	line_width = adj_update_region.width / 2;

	dst_ptr += (adj_update_region.top * dst_stride + left_offs);
	for (i = 0; i < adj_update_region.height; i++) {
		/* Copy the full line */
		memcpy(dst_ptr,	temp_buf_ptr, line_width);

		dst_ptr += dst_stride;
		temp_buf_ptr += line_width;
	}
}

static int spdc_process_update(struct update_data_list *upd_data_list,
				   mxc_spdc_t *fb_data)
{
	/* Region of src buffer for update */
	struct mxcfb_rect *src_upd_region;
	struct mxcfb_rect pxp_upd_region;
	struct update_desc_list *upd_desc_list = upd_data_list->update_desc;
	u32 src_width, src_height;
	u32 offset_from_4, bytes_per_pixel;
	u32 post_rotation_xcoord, post_rotation_ycoord, width_pxp_blocks;
	u32 pxp_input_offs, pxp_output_offs, pxp_output_shift;
	bool input_unaligned = false;
	u32 hist_stat = 0;
	bool use_temp_buf = false;
	int ret;

	/*
	 * Are we using FB or an alternate (overlay)
	 * buffer for source of update?
	 */
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_width = upd_desc_list->upd_data.alt_buffer_data.width;
		src_height = upd_desc_list->upd_data.alt_buffer_data.height;
		src_upd_region =
		&upd_desc_list->upd_data.alt_buffer_data.alt_update_region;
	} else {
		src_width = fb_data->spdc_fb_var.xres_virtual;
		src_height = fb_data->spdc_fb_var.yres;
		src_upd_region = &upd_desc_list->upd_data.update_region;
	}

	if (!(src_upd_region->width == fb_data->spdc_fb_var.xres_virtual &&
		fb_data->spdc_fb_var.rotate == FB_ROTATE_UR))
		use_temp_buf = true;

	bytes_per_pixel = fb_data->spdc_fb_var.bits_per_pixel / 8;

	/* Grab pxp_mutex here so that we protect access
	 * to copybuf in addition to the PxP structures */
	mutex_lock(&fb_data->pxp_mutex);

	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;

	if (input_unaligned) {
		/* Leave a gap between PxP input addr
		 * and update region pixels
		 */
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel & 0xFFFFFFFC;
		/* Update region left changes to reflect
		 * relative position to input ptr
		 */
		pxp_upd_region.left = (offset_from_4 * bytes_per_pixel % 4)
					/ bytes_per_pixel;
	} else {
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel;
		pxp_upd_region.left = 0;
	}
	pxp_upd_region.top = 0;

	/* Update region dimensions to meet 8x8 pixel requirement */
	if (fb_data->spdc_fb_var.rotate == 0) {
		pxp_upd_region.width = ALIGN(src_upd_region->width, 8);
		pxp_upd_region.height = ALIGN(src_upd_region->height, 8);
	} else {
		pxp_upd_region.width =
			ALIGN(src_upd_region->width + pxp_upd_region.left, 8);
		pxp_upd_region.height = ALIGN(src_upd_region->height, 8);
	}

	switch (fb_data->spdc_fb_var.rotate) {
	case FB_ROTATE_UR:
	default:
		post_rotation_xcoord = pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.top;
		width_pxp_blocks = pxp_upd_region.width;
		break;
	case FB_ROTATE_CW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = width_pxp_blocks -
					src_upd_region->height;
		post_rotation_ycoord = pxp_upd_region.left;
		break;
	case FB_ROTATE_UD:
		width_pxp_blocks = pxp_upd_region.width;
		post_rotation_xcoord = width_pxp_blocks -
			src_upd_region->width -	pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.height -
			src_upd_region->height - pxp_upd_region.top;
		break;
	case FB_ROTATE_CCW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = pxp_upd_region.top;
		post_rotation_ycoord = pxp_upd_region.width -
			src_upd_region->width -	pxp_upd_region.left;
		break;
	}

	/* Update region start coord to force PxP to
	 * process full 8x8 regions
	 */
	pxp_upd_region.top &= ~0x7;
	pxp_upd_region.left &= ~0x7;

	pxp_output_shift = ALIGN(post_rotation_xcoord, 8)
		- post_rotation_xcoord;
	pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
		+ pxp_output_shift;
	upd_desc_list->spdc_offs = ALIGN(pxp_output_offs, 8);

	/* Source address either comes from alternate buffer
	   provided in update data, or from the framebuffer. */
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		sg_dma_address(&fb_data->sg[0]) =
			upd_desc_list->upd_data.alt_buffer_data.phys_addr
				+ pxp_input_offs;
	else {
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->info.fix.smem_start + fb_data->fb_offset
			+ pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(fb_data->info.screen_base),
			fb_data->info.fix.smem_len,
			offset_in_page(fb_data->info.screen_base));
	}

	/* Update sg[1] to point to output of PxP proc task */
	if (!use_temp_buf) {
		sg_dma_address(&fb_data->sg[1]) = upd_data_list->phys_addr;
		sg_set_page(&fb_data->sg[1],
				virt_to_page(upd_data_list->virt_addr),
		fb_data->max_pix_size,
		offset_in_page(upd_data_list->virt_addr));
	} else {
		sg_dma_address(&fb_data->sg[1]) = fb_data->phys_addr_copybuf;
		sg_set_page(&fb_data->sg[1],
			virt_to_page(fb_data->virt_addr_copybuf),
		fb_data->max_pix_size,
		offset_in_page(fb_data->virt_addr_copybuf));
	}

	/*
	 * Set PxP LUT transform type based on update flags.
	 */
	fb_data->pxp_conf.proc_data.lut_transform = 0;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_ENABLE_INVERSION)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_INVERT;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_FORCE_MONOCHROME)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_BLACK_WHITE;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_CMAP)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_USE_CMAP;

	/*
	 * Toggle inversion processing if 8-bit
	 * inverted is the current pixel format.
	 */
	if (fb_data->spdc_fb_var.grayscale == GRAYSCALE_4BIT_INVERTED)
		fb_data->pxp_conf.proc_data.lut_transform ^= PXP_LUT_INVERT;

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_process_update(fb_data, src_width, src_height,
		&pxp_upd_region);
	if (ret) {
		dev_err(fb_data->dev, "Unable to submit PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* If needed, enable SPDC HW while ePxP is processing */
	if ((fb_data->power_state == POWER_STATE_OFF)
		|| fb_data->powering_down) {
		spdc_powerup(fb_data);
	}

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_complete_update(fb_data, &hist_stat);
	if (ret) {
		dev_err(fb_data->dev, "Unable to complete PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	if (use_temp_buf)
		copy_to_next_buffer(fb_data, upd_data_list);

	mutex_unlock(&fb_data->pxp_mutex);

	/* Update waveform mode from PxP histogram results */
	if (upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO) {
		if (hist_stat & 0x1)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_du;
		else if (hist_stat & 0x2)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc4;
		else if (hist_stat & 0x4)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc8;
		else if (hist_stat & 0x8)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc16;
		else
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc32;

		dev_dbg(fb_data->dev, "hist_stat = 0x%x, new waveform = 0x%x\n",
			hist_stat, upd_desc_list->upd_data.waveform_mode);
	}

	return 0;
}

static bool spdc_submit_concur(mxc_spdc_t *fb_data,
			struct update_desc_list *update_to_concur)
{
	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct update_data_list *next_upd;
	int i = 0;

	a = &update_to_concur->upd_data;
	arect = &update_to_concur->upd_data.update_region;

	list_for_each_entry(next_upd,
		&fb_data->upd_buf_preprocess_list, list) {
		b = &next_upd->update_desc->upd_data;
		brect = &next_upd->update_desc->upd_data.update_region;

		/* Updates with different waveform
		 * must be executed sequentially.
		 */
		if (a->waveform_mode != b->waveform_mode)
			break;

		/*
		  * Concurrency update must has no overlay
		  */
		if (!(arect->left > (brect->left + brect->width) ||
			brect->left > (arect->left + arect->width) ||
			arect->top > (brect->top + brect->height) ||
			brect->top > (arect->top + arect->height)))
			break;

		i++;
	}

	if (i != fb_data->upd_preprocess_num)
		return false;

	return true;
}

static int spdc_submit_merge(struct update_desc_list *upd_desc_list,
				struct update_desc_list *update_to_merge)
{
	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct mxcfb_rect combine;
	bool use_flags = false;

	a = &upd_desc_list->upd_data;
	b = &update_to_merge->upd_data;
	arect = &upd_desc_list->upd_data.update_region;
	brect = &update_to_merge->upd_data.update_region;

	/*
	 * Updates with different flags must be executed sequentially.
	 * Halt the merge process to ensure this.
	 */
	if (a->flags != b->flags) {
		/*
		 * Special exception: if update regions are identical,
		 * we may be able to merge them.
		 */
		if ((arect->left != brect->left) ||
			(arect->top != brect->top) ||
			(arect->width != brect->width) ||
			(arect->height != brect->height))
			return MERGE_BLOCK;

		use_flags = true;
	}

	if (a->waveform_mode != b->waveform_mode)
		a->waveform_mode = WAVEFORM_MODE_AUTO;

	if (arect->left > (brect->left + brect->width) ||
		brect->left > (arect->left + arect->width) ||
		arect->top > (brect->top + brect->height) ||
		brect->top > (arect->top + arect->height))
		return MERGE_FAIL;

	combine.left = arect->left < brect->left ? arect->left : brect->left;
	combine.top = arect->top < brect->top ? arect->top : brect->top;
	combine.width = (arect->left + arect->width) >
			(brect->left + brect->width) ?
			(arect->left + arect->width - combine.left) :
			(brect->left + brect->width - combine.left);
	combine.height = (arect->top + arect->height) >
			(brect->top + brect->height) ?
			(arect->top + arect->height - combine.top) :
			(brect->top + brect->height - combine.top);

	*arect = combine;

	/* Use flags of the later update */
	if (use_flags)
		a->flags = b->flags;

	/* Merge markers */
	list_splice_tail(&update_to_merge->upd_marker_list,
		&upd_desc_list->upd_marker_list);

	return MERGE_OK;

}

static void spdc_submit_work_func(struct work_struct *work)
{
	struct update_desc_list *next_desc, *temp_desc;
	mxc_spdc_t *fb_data =
		container_of(work, mxc_spdc_t, spdc_submit_work);
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect adj_update_region, *upd_region;
	struct update_marker_data *current_marker;
	bool end_merge = false;
	bool is_transform;
	u32 update_addr;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/* get a buffer from free list */
	if (list_empty(&fb_data->upd_buf_free_list)) {
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	if (fb_data->fresh_param.concur == SPDC_LUT_ACC_MODE) {
		list_for_each_entry_safe(next_desc, temp_desc,
			&fb_data->upd_pending_list, list) {

			current_marker =
			list_entry((&next_desc->upd_marker_list)->next,
				struct update_marker_data, upd_list);

			if (current_marker->update_marker) {
				fb_data->submit_upd_sta = 0;
				break;
			}

			/* require free buffer list */
			if (list_empty(&fb_data->upd_buf_free_list)) {
				dev_dbg(fb_data->dev,
					"buf free list is empty\n");
				break;
			}

			upd_data_list =
			list_entry(fb_data->upd_buf_free_list.next,
				struct update_data_list, list);
			upd_data_list->update_desc = next_desc;

			if (!is_preprocess_list_full(fb_data)) {
				if (fb_data->cur_update == NULL &&
					!fb_data->upd_preprocess_num)
					list_del_init(&next_desc->list);
				else if (spdc_submit_concur(fb_data,
						next_desc)) {
					list_del_init(&next_desc->list);
					list_add_tail(&upd_data_list->list,
					&fb_data->upd_buf_preprocess_list);
					fb_data->upd_preprocess_num++;
					fb_data->submit_upd_sta =
							SPDC_CONCUR_UPD;
				} else
					break;
			} else
				break;

			/* submit to pxp process */
			list_del_init(&upd_data_list->list);
			goto pxp_process;
		}

		upd_data_list = NULL;
		if (fb_data->submit_upd_sta == SPDC_CONCUR_UPD)
			fb_data->submit_upd_sta |= SPDC_QUEUE_UPD;
		else
			fb_data->submit_upd_sta = SPDC_QUEUE_UPD;
	}

	list_for_each_entry_safe(next_desc, temp_desc,
			&fb_data->upd_pending_list, list) {

		if (!upd_data_list) {

			if (list_empty(&fb_data->upd_buf_free_list)) {
				dev_dbg(fb_data->dev,
					"buf_free_list is empty\n");
				break;
			}
			upd_data_list =
				list_entry(fb_data->upd_buf_free_list.next,
					struct update_data_list, list);
			list_del_init(&upd_data_list->list);
			upd_data_list->update_desc = next_desc;
			list_del_init(&next_desc->list);

			if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
				break;
		} else {
			switch (spdc_submit_merge(upd_data_list->update_desc,
					next_desc)) {
			case MERGE_OK:
				dev_dbg(fb_data->dev,
					"Update merged [queue]\n");
				list_del_init(&next_desc->list);
				kfree(next_desc);
				break;
			case MERGE_FAIL:
				dev_dbg(fb_data->dev,
					"Update not merged [queue]\n");
				break;
			case MERGE_BLOCK:
				dev_dbg(fb_data->dev,
					"Merge blocked [collision]\n");
				end_merge = true;
				break;
			}

			if (end_merge)
				break;
		}
	}

	/* Is update list empty? */
	if (!upd_data_list) {
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

pxp_process:
	/*
	 * If no processing required, skip update processing
	 * No processing means:
	 *   - FB unrotated
	 *   - FB pixel format = 4-bit grayscale
	 *   - No look-up transformations (inversion, posterization, etc.)
	 */
	is_transform = upd_data_list->update_desc->upd_data.flags &
		(EPDC_FLAG_ENABLE_INVERSION |
		EPDC_FLAG_FORCE_MONOCHROME | EPDC_FLAG_USE_CMAP) ?
		true : false;
	if ((fb_data->spdc_fb_var.rotate == FB_ROTATE_UR) &&
		(fb_data->spdc_fb_var.grayscale == GRAYSCALE_4BIT) &&
		!is_transform) {

		/* If needed, enable SPDC HW while ePxP is processing */
		if ((fb_data->power_state == POWER_STATE_OFF)
			|| fb_data->powering_down)
			spdc_powerup(fb_data);

		/*
		 * Set update buffer pointer to the start of
		 * the update region in the frame buffer.
		 */
		upd_region =
			&upd_data_list->update_desc->upd_data.update_region;
		update_addr = fb_data->info.fix.smem_start +
			((upd_region->top * fb_data->info.var.xres_virtual) +
			upd_region->left) * fb_data->info.var.bits_per_pixel/8;
	} else {

		/* Select from PxP output buffers */
		upd_data_list->phys_addr =
			fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
		upd_data_list->virt_addr =
			fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
		fb_data->upd_buffer_num++;
		if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
			fb_data->upd_buffer_num = 0;
		dev_dbg(fb_data->dev,
			"pxp out addr:0x%x\n", upd_data_list->phys_addr);

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);

		/* Perform PXP processing - SPDC power will also be enabled */
		if (spdc_process_update(upd_data_list, fb_data)) {

			dev_dbg(fb_data->dev, "PXP processing error.\n");
			/* Protect access to buffer queues and to update HW */
			mutex_lock(&fb_data->queue_mutex);
			list_del_init(&upd_data_list->update_desc->list);
			kfree(upd_data_list->update_desc);
			upd_data_list->update_desc = NULL;

			/* Add to free buffer list */
			list_add_tail(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
			/* Release buffer queues */
			mutex_unlock(&fb_data->queue_mutex);
			return;
		}

		/* Protect access to buffer queues and to update HW */
		mutex_lock(&fb_data->queue_mutex);

		/* output Y4 format */
		update_addr = upd_data_list->phys_addr +
			+ (upd_data_list->update_desc->spdc_offs / 2);
	}

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->spdc_fb_var.xres,
		fb_data->spdc_fb_var.yres, fb_data->spdc_fb_var.rotate,
		&upd_data_list->update_desc->upd_data.update_region,
		&adj_update_region);

	/*
	 * Is the working buffer idle?
	 * If the working buffer is busy, we must wait for the resource
	 * to become free.
	 */
	if (fb_data->cur_update != NULL &&
	fb_data->submit_upd_sta != SPDC_CONCUR_UPD) {
		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_wb = true;

		/* Leave spinlock while waiting for WB to complete */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
	}

	if (fb_data->submit_upd_sta != SPDC_CONCUR_UPD)
		fb_data->cur_update = upd_data_list;

	/* program SPDC register and trigger to process buffer*/
	fb_data->fresh_param.buf_addr.next_buf_phys_addr = update_addr;

	fb_data->fresh_param.update_region.left = adj_update_region.left;
	fb_data->fresh_param.update_region.top = adj_update_region.top;
	fb_data->fresh_param.update_region.width = adj_update_region.width;
	fb_data->fresh_param.update_region.height = adj_update_region.height;
	fb_data->fresh_param.temper = upd_data_list->update_desc->upd_data.temp;
	fb_data->fresh_param.wave_mode =
		upd_data_list->update_desc->upd_data.waveform_mode;

	spdc_submit_update(fb_data);

	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);
}

int mxc_spdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	struct update_desc_list *upd_desc;
	struct update_marker_data *marker_data;
	int ret;

	/* Has SPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		/* Throw message if we are not mid-initialization */
		if (!fb_data->hw_initializing)
			dev_err(fb_data->dev, "Display HW not properly"
				"initialized. Aborting update.\n");
		return -EPERM;
	}

	if ((upd_data->waveform_mode > SPDC_WAV_MODE_5) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
	if ((upd_data->update_region.left + upd_data->update_region.width >
		fb_data->spdc_fb_var.xres + 1) ||
		(upd_data->update_region.top + upd_data->update_region.height >
		fb_data->spdc_fb_var.yres + 1)) {
		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer."
			"Aborting update.\n");
		return -EINVAL;
	}
	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		if ((upd_data->update_region.width !=
			upd_data->alt_buffer_data.alt_update_region.width) ||
			(upd_data->update_region.height !=
			upd_data->alt_buffer_data.alt_update_region.height)) {
			dev_err(fb_data->dev,
				"Alternate update region dimensions must "
				"match screen update region dimensions.\n");
			return -EINVAL;
		}
		/* Validate physical address parameter */
		if ((upd_data->alt_buffer_data.phys_addr <
			fb_data->info.fix.smem_start) ||
			(upd_data->alt_buffer_data.phys_addr >
			fb_data->info.fix.smem_start + fb_data->map_size)) {
			dev_err(fb_data->dev,
				"Invalid physical address for alternate "
				"buffer.  Aborting update...\n");
			return -EINVAL;
		}
	}

	mutex_lock(&fb_data->queue_mutex);

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	if (fb_data->waiting_for_idle) {
		dev_dbg(fb_data->dev, "SPDC not active."
			"Update request abort.\n");
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}

	if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
		int count = 0;
		struct update_data_list *plist;

		/* Count buffers in free buffer list */
		list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
			count++;

		/* Use count to determine if we have enough
		 * free buffers to handle this update request */
		if (count + fb_data->max_num_buffers
			<= fb_data->max_num_updates) {
			dev_err(fb_data->dev,
				"No free intermediate buffers available.\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}

		/* Grab first available buffer and delete from the free list */
		upd_data_list =
		list_entry(fb_data->upd_buf_free_list.next,
			   struct update_data_list, list);

		list_del_init(&upd_data_list->list);
	}

	/*
	 * Create new update data structure, fill it with new update
	 * data and add it to the list of pending updates
	 */
	upd_desc = kzalloc(sizeof(struct update_desc_list), GFP_KERNEL);
	if (!upd_desc) {
		dev_err(fb_data->dev,
			"Insufficient system memory for update! Aborting.\n");
		if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
			list_add(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
		}
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}
	/* Initialize per-update marker list */
	INIT_LIST_HEAD(&upd_desc->upd_marker_list);
	upd_desc->upd_data = *upd_data;
	list_add_tail(&upd_desc->list, &fb_data->upd_pending_list);

	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {

		/* Allocate new update marker and set it up */
		marker_data = kzalloc(sizeof(struct update_marker_data),
				GFP_KERNEL);
		if (!marker_data) {
			dev_err(fb_data->dev, "No memory for marker!\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}
		list_add_tail(&marker_data->upd_list,
			&upd_desc->upd_marker_list);
		marker_data->update_marker = upd_data->update_marker;
		init_completion(&marker_data->update_completion);

		/* Add marker to master marker list */
		list_add_tail(&marker_data->full_list,
			&fb_data->full_marker_list);
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		mutex_unlock(&fb_data->queue_mutex);

		/* Signal workqueue to handle new update */
		queue_work(fb_data->spdc_submit_workqueue,
			&fb_data->spdc_submit_work);

		return 0;
	}

	/* Set descriptor for current update, delete from pending list */
	upd_data_list->update_desc = upd_desc;
	list_del_init(&upd_desc->list);

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * Hold on to original screen update region, which we
	 * will ultimately use when telling SPDC where to update on panel
	 */
	screen_upd_region = &upd_desc->upd_data.update_region;

	/* Select from PxP output buffers */
	upd_data_list->phys_addr =
		fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
	upd_data_list->virt_addr =
		fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
	fb_data->upd_buffer_num++;
	if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
		fb_data->upd_buffer_num = 0;

	ret = spdc_process_update(upd_data_list, fb_data);
	if (ret) {
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* Pass selected waveform mode back to user */
	upd_data->waveform_mode = upd_desc->upd_data.waveform_mode;

	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->spdc_fb_var.xres,
		fb_data->spdc_fb_var.yres, fb_data->spdc_fb_var.rotate,
		&upd_desc->upd_data.update_region, NULL);

	/* Grab lock for queue manipulation and update submission */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Is the working buffer idle?
	 * If either the working buffer is busy, or there are no LUTs available,
	 * then we return and let the ISR handle the update later
	 */
	if (fb_data->cur_update != NULL) {

		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		return 0;
	}

	/* Save current update */
	fb_data->cur_update = upd_data_list;

	/* program SPDC register and trigger to process buffer*/
	fb_data->fresh_param.buf_addr.next_buf_phys_addr =
	upd_data_list->phys_addr + (upd_data_list->update_desc->spdc_offs / 2);

	fb_data->fresh_param.update_region.left = screen_upd_region->left;
	fb_data->fresh_param.update_region.top = screen_upd_region->top;
	fb_data->fresh_param.update_region.width = screen_upd_region->width;
	fb_data->fresh_param.update_region.height = screen_upd_region->height;
	fb_data->fresh_param.temper = upd_desc->upd_data.temp;
	fb_data->fresh_param.wave_mode = upd_desc->upd_data.waveform_mode;
	spdc_submit_update(fb_data);

	mutex_unlock(&fb_data->queue_mutex);
	return 0;
}
EXPORT_SYMBOL(mxc_spdc_fb_send_update);

/*
 * return 0 : spdc update is update
 */
int
mxc_spdc_fb_wait_update_complete(struct mxcfb_update_marker_data *marker_data,
				struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	bool marker_found = false;
	int ret = 0;

	/* 0 is an invalid update_marker value */
	if (marker_data->update_marker == 0)
		return -EINVAL;

	/*
	 * Find completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared, it won't be found, and function will just return.
	 */

	/* Grab queue lock to protect access to marker list */
	mutex_lock(&fb_data->queue_mutex);

	list_for_each_entry_safe(next_marker, temp,
		&fb_data->full_marker_list, full_list) {
		if (next_marker->update_marker == marker_data->update_marker) {
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
				marker_data->update_marker);
			next_marker->waiting = true;
			marker_found = true;
			break;
		}
	}

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * If marker not found, it has either been signalled already
	 * or the update request failed.  In either case, just return.
	 */
	if (!marker_found)
		return ret;

	ret = wait_for_completion_timeout(&next_marker->update_completion,
						msecs_to_jiffies(8000));
	if (!ret) {
		dev_err(fb_data->dev,
			"Timed out waiting for update completion\n");
		return -ETIMEDOUT;
	}

	/** since SPDC don't support auto collision detect,
	 *  there alway returns no collision
	 */
	marker_data->collision_test = false;

	/* Free update marker object */
	kfree(next_marker);

	return ret;
}
EXPORT_SYMBOL(mxc_spdc_fb_wait_update_complete);

int mxc_spdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
				struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;

	fb_data->pwrdown_delay = pwrdown_delay;

	return 0;
}
EXPORT_SYMBOL(mxc_spdc_fb_set_pwrdown_delay);

int mxc_spdc_get_pwrdown_delay(struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;

	return fb_data->pwrdown_delay;
}
EXPORT_SYMBOL(mxc_spdc_get_pwrdown_delay);

static int
mxc_spdc_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	void __user *argp = (void __user *)arg;
	int ret = -EINVAL;

	dev_dbg(fb_data->dev, "cmd = %08X, arg = %08X\n", cmd, (u32)arg);

	switch (cmd) {
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_spdc_fb_set_waveform_modes(&modes, info);
				ret = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			if (!get_user(temperature, (int32_t __user *) arg))
				ret = mxc_spdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			if (!get_user(auto_mode, (__u32 __user *) arg))
				ret = mxc_spdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				ret = mxc_spdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;
			if (!copy_from_user(&upd_data, argp,
				sizeof(upd_data))) {
				ret = mxc_spdc_fb_send_update(&upd_data, info);
				if (ret == 0 && copy_to_user(argp, &upd_data,
					sizeof(upd_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}
	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			struct mxcfb_update_marker_data upd_marker_data;
			if (!copy_from_user(&upd_marker_data, argp,
				sizeof(upd_marker_data))) {
				ret = mxc_spdc_fb_wait_update_complete(
					&upd_marker_data, info);
				if (copy_to_user(argp, &upd_marker_data,
					sizeof(upd_marker_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}
	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			if (!get_user(delay, (__u32 __user *) arg))
				ret =
				mxc_spdc_fb_set_pwrdown_delay(delay, info);
			break;
		}
	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_spdc_get_pwrdown_delay(info);
			if (put_user(pwrdown_delay, (int __user *)arg))
				ret = -EFAULT;
			break;
		}
	default:
		dev_err(fb_data->dev, "IOCTL_CMD: not such command\n");
		return -ENOTTY;
	}

	return ret;
}

static void mxc_spdc_fb_update_pages(mxc_spdc_t *fb_data, u16 y1, u16 y2)
{
	struct mxcfb_update_data update;

	/* Do partial screen update, Update full horizontal lines */
	update.update_region.left = 0;
	update.update_region.width = fb_data->spdc_fb_var.xres;
	update.update_region.top = y1;
	update.update_region.height = y2 - y1;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_mode = UPDATE_MODE_FULL;
	update.update_marker = 0;
	update.temp = SPDC_DEFAULT_TEMP;
	update.flags = 0;

	mxc_spdc_fb_send_update(&update, &fb_data->info);
}

/* this is called back from the deferred io workqueue */
static void mxc_spdc_fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	struct page *page;
	unsigned long beg, end;
	int y1, y2, miny, maxy;

	if (fb_data->auto_mode != AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		return;

	miny = INT_MAX;
	maxy = 0;
	list_for_each_entry(page, pagelist, lru) {
		beg = page->index << PAGE_SHIFT;
		end = beg + PAGE_SIZE - 1;
		y1 = beg / info->fix.line_length;
		y2 = end / info->fix.line_length;
		if (y2 >= fb_data->spdc_fb_var.yres)
			y2 = fb_data->spdc_fb_var.yres - 1;
		if (miny > y1)
			miny = y1;
		if (maxy < y2)
			maxy = y2;
	}

	mxc_spdc_fb_update_pages(fb_data, miny, maxy);
}

static int mxc_spdc_fb_blank(int blank, struct fb_info *info)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int ret = 0;

	dev_dbg(fb_data->dev, "blank = %d\n", blank);

	if (fb_data->blank == blank)
		return 0;

	fb_data->blank = blank;
	switch (blank) {
	case FB_BLANK_POWERDOWN:
		mxc_spdc_fb_flush_updates(fb_data);
		/* Wait for powerdown */
		mutex_lock(&fb_data->power_mutex);
		if ((fb_data->power_state == POWER_STATE_ON) &&
			(fb_data->pwrdown_delay == FB_POWERDOWN_DISABLE)) {

			/* Powerdown disabled, so we disable SPDC manually */
			int count = 0;
			int sleep_ms = 10;

			mutex_unlock(&fb_data->power_mutex);

			/* If any active updates, wait for them to complete */
			while (fb_data->updates_active) {
				/* Timeout after 1 sec */
				if ((count * sleep_ms) > 1000)
					break;
				msleep(sleep_ms);
				count++;
			}

			fb_data->powering_down = true;
			spdc_powerdown(fb_data);
		} else if (fb_data->power_state != POWER_STATE_OFF) {
			fb_data->wait_for_powerdown = true;
			init_completion(&fb_data->powerdown_compl);
			mutex_unlock(&fb_data->power_mutex);
			ret =
			wait_for_completion_timeout(&fb_data->powerdown_compl,
				msecs_to_jiffies(5000));
			if (!ret) {
				dev_err(fb_data->dev,
					"No powerdown received!\n");
				return -ETIMEDOUT;
			}
		} else
			mutex_unlock(&fb_data->power_mutex);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		mxc_spdc_fb_flush_updates(fb_data);
		break;
	}

	return ret;
}

static int mxc_spdc_fb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{
	mxc_spdc_t *fb_data = info ?
		(mxc_spdc_t *)info:g_spdc_fb_data;
	u_int y_bottom;

	dev_dbg(info->device, "%s: var->yoffset %d, info->var.yoffset %d\n",
		 __func__, var->yoffset, info->var.yoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	(var->yoffset + var->yres > var->yres_virtual)) {
		dev_dbg(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((fb_data->spdc_fb_var.xoffset == var->xoffset) &&
		(fb_data->spdc_fb_var.yoffset == var->yoffset))
		return 0;	/* No change, do nothing */

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual)
		return -EINVAL;

	mutex_lock(&fb_data->queue_mutex);

	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;

	fb_data->spdc_fb_var.xoffset = var->xoffset;
	fb_data->spdc_fb_var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}

static struct fb_ops mxc_spdc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_spdc_fb_check_var,
	.fb_set_par = mxc_spdc_fb_set_par,
	.fb_setcmap = mxc_spdc_fb_setcmap,
	.fb_setcolreg = mxc_spdc_fb_setcolreg,
	.fb_pan_display = mxc_spdc_fb_pan_display,
	.fb_ioctl = mxc_spdc_fb_ioctl,
	.fb_mmap = mxc_spdc_fb_mmap,
	.fb_blank = mxc_spdc_fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static struct fb_deferred_io mxc_spdc_fb_defio = {
	.delay = HZ,
	.deferred_io = mxc_spdc_fb_deferred_io,
};

static void spdc_done_work_func(struct work_struct *work)
{
	mxc_spdc_t *fb_data =
		container_of(work, mxc_spdc_t, spdc_done_work.work);
	spdc_powerdown(fb_data);
}

static irqreturn_t mxc_spdc_irq_handler(int irq, void *dev_id)
{
	mxc_spdc_t *fb_data = (mxc_spdc_t *)dev_id;
	ulong flags;
	u32 int_events;
	irqreturn_t ret = IRQ_NONE;

	spin_lock_irqsave(&fb_data->lock, flags);

	int_events = spdc_get_intr_stat(fb_data);
	dev_dbg(fb_data->dev, "spdc int:%x\n", int_events);

	if (int_events & SPDC_IRQ_STA_ERR) {
		ret = IRQ_HANDLED;
		spdc_intr_stat_clear(fb_data, SPDC_IRQ_STA_ERR);
		dev_err(fb_data->dev, "Error IRQ\n");
		return ret;
	}

	/*
	 * If we just completed one-time panel init, bypass
	 * queue handling, clear interrupt and return
	 */
	if (fb_data->operation_mode &&
		(int_events & SPDC_IRQ_STA_FRAME_UPDATE)) {
		mutex_lock(&fb_data->queue_mutex);
		fb_data->updates_active = false;
		complete(&fb_data->update_finish);
		spdc_intr_stat_clear(fb_data, SPDC_IRQ_STA_FRAME_UPDATE);

		if ((fb_data->operation_mode == SPDC_DEEP_REFRESH) ||
			fb_data->is_deep_fresh)
			fb_data->is_deep_fresh = false;
		else
			fb_data->operation_mode = SPDC_NO_OPERATION;

		mutex_unlock(&fb_data->queue_mutex);

		return IRQ_HANDLED;
	}

	/* waveform loading to SPDC  from memory */
	if (int_events & SPDC_IRQ_STA_LUT_DOWNLOAD) {
		if (is_lut_checksum_ok(fb_data)) {
			complete(&fb_data->lut_down);
			spdc_intr_stat_clear(fb_data,
				SPDC_IRQ_STA_LUT_DOWNLOAD);
		} else
			dev_dbg(fb_data->dev, "Lut checksum is err!\n");

		return IRQ_HANDLED;
	}

	/* SPDC init setting IRQ */
	if (int_events & SPDC_IRQ_STA_TCON_INIT) {
		complete(&fb_data->init_finish);
		spdc_intr_stat_clear(fb_data, SPDC_IRQ_STA_TCON_INIT);

		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&fb_data->lock, flags);

	if (spdc_is_update_finish(fb_data)) {
		/* clear interrupt status */
		spdc_intr_stat_clear(fb_data, SPDC_IRQ_STA_FRAME_UPDATE);
		queue_work(fb_data->spdc_intr_workqueue,
			&fb_data->spdc_intr_work);
	}

	 return ret;
}

static void spdc_intr_work_func(struct work_struct *work)
{
	mxc_spdc_t *fb_data =
		container_of(work, mxc_spdc_t, spdc_intr_work);
	struct mxcfb_rect *next_upd_region;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	struct update_data_list *next_upd, *temp_upd;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/* Check to see if all updates have completed */
	if (list_empty(&fb_data->upd_pending_list) &&
		is_free_list_full(fb_data) &&
		(fb_data->cur_update == NULL)) {

		fb_data->updates_active = false;

		if (fb_data->pwrdown_delay != FB_POWERDOWN_DISABLE) {
			/*
			 * Set variable to prevent overlapping
			 * enable/disable requests
			 */
			fb_data->powering_down = true;

			/* Schedule task to disable SPDC HW until next update */
			schedule_delayed_work(&fb_data->spdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

			/* Reset counter to reduce chance of overflow */
			fb_data->order_cnt = 0;
		}

		if (fb_data->waiting_for_idle)
			complete(&fb_data->updates_done);
	}

	if (mxc_spdc_device_is_busy(fb_data)) {
		/* Can't submit another update until SPDC is idle */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/*
	 * Were we waiting on working buffer?
	 * If so, update queues and check for collisions
	 */
	if (fb_data->cur_update != NULL) {
		list_for_each_entry_safe(next_marker, temp,
			&fb_data->cur_update->update_desc->upd_marker_list,
			upd_list) {

			/* Del from per-update & full list */
			list_del_init(&next_marker->upd_list);
			list_del_init(&next_marker->full_list);

			/* Signal completion of update */
			dev_dbg(fb_data->dev,
				"Signaling marker %d\n",
				next_marker->update_marker);
			if (next_marker->waiting)
				complete(&next_marker->update_completion);
			else
				kfree(next_marker);
		}

		/* Free marker list and update descriptor */
		kfree(fb_data->cur_update->update_desc);

		/* Add to free buffer list */
		list_add_tail(&fb_data->cur_update->list,
			 &fb_data->upd_buf_free_list);

		if (fb_data->submit_upd_sta & SPDC_CONCUR_UPD) {
			fb_data->upd_preprocess_num = 0;
			fb_data->submit_upd_sta &= (~SPDC_CONCUR_UPD);
		}

		/* free ACC update list */
		list_for_each_entry_safe(next_upd, temp_upd,
			&fb_data->upd_buf_preprocess_list, list) {
			next_marker = list_entry(
			(&next_upd->update_desc->upd_marker_list)->next,
				struct update_marker_data, upd_list);

			list_del_init(&next_marker->upd_list);
			list_del_init(&next_marker->full_list);

			/* Signal completion of update */
			dev_dbg(fb_data->dev,
				"Signaling marker %d\n",
				next_marker->update_marker);

			if (next_marker->waiting)
				complete(&next_marker->update_completion);
			else
				kfree(next_marker);


			list_del_init(&next_upd->list);
			/* Add to free buffer list */
			list_add_tail(&next_upd->list,
				&fb_data->upd_buf_free_list);
			kfree(next_upd->update_desc);
		}

		/* Check to see if all updates have completed */
		if (list_empty(&fb_data->upd_pending_list) &&
			is_free_list_full(fb_data)) {

			fb_data->updates_active = false;

			if (fb_data->pwrdown_delay !=
					FB_POWERDOWN_DISABLE) {
				/*
				 * Set variable to prevent overlapping
				 * enable/disable requests
				 */
				fb_data->powering_down = true;

				/* Schedule SPDC disable */
				schedule_delayed_work(&fb_data->spdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

				/* Reset counter to reduce chance of overflow */
				fb_data->order_cnt = 0;
			}

			if (fb_data->waiting_for_idle)
				complete(&fb_data->updates_done);
		}

		/* Signal completion if submit workqueue was waiting on WB */
		if (fb_data->waiting_for_wb) {
			dev_dbg(fb_data->dev, "free update_res_free\n");
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_wb = false;
		}

		/* Clear current update */
		fb_data->cur_update = NULL;
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */
		/* Schedule task to submit collision and pending update */
		if (!fb_data->powering_down)
			queue_work(fb_data->spdc_submit_workqueue,
				&fb_data->spdc_submit_work);

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/* Snapshot update scheme processing */
	if (list_empty(&fb_data->upd_buf_queue)) {
		dev_dbg(fb_data->dev, "No pending updates.\n");

		/* No updates pending, so we are done */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	} else {
		dev_dbg(fb_data->dev, "Found a pending update!\n");

		/* Process next item in update list */
		fb_data->cur_update =
			list_entry(fb_data->upd_buf_queue.next,
				struct update_data_list, list);
		list_del_init(&fb_data->cur_update->list);
	}

	/* program SPDC register and trigger to process buffer*/
	next_upd_region =
		&fb_data->cur_update->update_desc->upd_data.update_region;
	fb_data->fresh_param.buf_addr.next_buf_phys_addr =
		fb_data->cur_update->phys_addr +
		(fb_data->cur_update->update_desc->spdc_offs / 2);

	fb_data->fresh_param.update_region.left = next_upd_region->left;
	fb_data->fresh_param.update_region.top = next_upd_region->top;
	fb_data->fresh_param.update_region.width = next_upd_region->width;
	fb_data->fresh_param.update_region.height = next_upd_region->height;
	fb_data->fresh_param.temper =
		fb_data->cur_update->update_desc->upd_data.temp;
	fb_data->fresh_param.wave_mode =
		fb_data->cur_update->update_desc->upd_data.waveform_mode;
	spdc_submit_update(fb_data);

	mutex_unlock(&fb_data->queue_mutex);

	return;
}

/*
 * Sysfs functions
 */
static ssize_t show_update(struct device *device,
				struct device_attribute *attr, char *buf) {
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;

	return sprintf(buf, "mode%d\n", fb_data->fresh_param.wave_mode);
}

static ssize_t store_update(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	if (strncmp(buf, "direct", 6) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;
	else if (strncmp(buf, "init", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_init;
	else if (strncmp(buf, "gu", 2) == 0)
		update.waveform_mode = SPDC_WAV_MODE_1;
	else if (strncmp(buf, "auto", 4) == 0)
		update.waveform_mode = WAVEFORM_MODE_AUTO;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = fb_data->spdc_fb_var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->spdc_fb_var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.temp = SPDC_DEFAULT_TEMP;
	update.update_marker = 0;
	update.flags = 0;
	dev_dbg(fb_data->dev, "rotation:%d, gray:%d\n",
	fb_data->spdc_fb_var.rotate, fb_data->spdc_fb_var.grayscale);

	mxc_spdc_fb_send_update(&update, info);

	return count;
}

static ssize_t fresh_show(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;

	return sprintf(buf, "%d\n", fb_data->operation_mode);
}

static ssize_t fresh_store(struct device *device, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int ret, operation;

	ret = kstrtoint(buf, 10, &operation);
	if (ret)
		return ret;

	if (operation == SPDC_DEEP_REFRESH)
		fb_data->is_deep_fresh = true;
	fb_data->operation_mode = operation;
	if (operation > SPDC_NO_OPERATION &&
		operation < SPDC_FULL_REFRESH)
		mxc_operaton_update(fb_data);
	else
		mxc_spdc_refresh_display(fb_data);

	return count;
}

static ssize_t temp_show(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int temp;

	temp = fb_data->fresh_param.temper >> 1;
	return sprintf(buf, "%d\n", temp);
}

static ssize_t initset_show(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	u32 init_val;

	get_panel_init_set(&fb_data->panel_set, &init_val);
	return sprintf(buf, "%x\n", init_val);
}

static ssize_t concurrency_show(struct device *device,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	bool temp;

	temp = (fb_data->fresh_param.concur ? 1 : 0);
	return sprintf(buf, "%d\n", temp);
}

static ssize_t concurrency_update(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *info = dev_get_drvdata(device);
	mxc_spdc_t *fb_data = (mxc_spdc_t *)info;
	int ret, concur;

	ret = kstrtoint(buf, 10, &concur);
	if (ret)
		return ret;

	if (fb_data->fresh_param.concur != concur) {
		fb_data->fresh_param.concur = concur;
		spdc_set_update_concurrency(fb_data);
	}

	return count;
}

static DEVICE_ATTR(store_update, 0644, show_update, store_update);
static DEVICE_ATTR(fresh, 0644, fresh_show, fresh_store);
static DEVICE_ATTR(temp, 0644, temp_show, NULL);
static DEVICE_ATTR(initset, 0644, initset_show, NULL);
static DEVICE_ATTR(concurrency, 0644, concurrency_show, concurrency_update);

static struct attribute *spdc_attributes[] = {
	&dev_attr_store_update.attr,
	&dev_attr_fresh.attr,
	&dev_attr_temp.attr,
	&dev_attr_initset.attr,
	&dev_attr_concurrency.attr,
	NULL
};

static const struct attribute_group spdc_attr_group = {
	.attrs = spdc_attributes,
};

static int __devinit mxc_spdc_fb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	mxc_spdc_t *fb_data;
	struct resource *res, *mem;
	struct fb_videomode *vmode;
	int xres_virt, yres_virt, buf_size;
	int xres_virt_rot, yres_virt_rot, pix_size_rot;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	char *options, *opt;
	char *panel_str = NULL;
	char name[] = "mxcspdcfb";
	unsigned long x_mem_size = 0;
	u32 i, cmap_size;
	int ret = 0;

	fb_data = (mxc_spdc_t *)framebuffer_alloc(sizeof(mxc_spdc_t),
							&pdev->dev);
	if (!fb_data) {
		ret = -ENOMEM;
		goto out;
	}

	info = &fb_data->info;
	fb_data->dev = &pdev->dev;
	platform_set_drvdata(pdev, fb_data);

	fb_data->pdata = pdev->dev.platform_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->spdc_mode == NULL)
		|| (fb_data->pdata->spdc_mode->vmode == NULL)) {
		ret = -EINVAL;
		goto dealloc_fb;
	}

	/*get panel para from command*/
	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto dealloc_fb;
	}
	if (options)
		while ((opt = strsep(&options, ",")) != NULL) {
			if (!*opt)
				continue;
			if (!strncmp(opt, "bpp=", 4)) {
				if (kstrtoint(opt + 4, 0,
				&fb_data->default_bpp) < 0)
					fb_data->default_bpp = 0;
			} else if (!strncmp(opt, "x_mem=", 6))
				x_mem_size = memparse(opt + 6, NULL);
			else
				panel_str = opt;
		}

	if (!fb_data->default_bpp)
		fb_data->default_bpp = SPDC_DEFAULT_BPP;

	/* Set default (first defined mode) for a match */
	mxc_spdc_find_match_mode(fb_data);

	if (panel_str)
		for (i = 0; i < fb_data->pdata->num_modes; i++)
			if (!strcmp(fb_data->pdata->spdc_mode[i].vmode->name,
							panel_str)) {
				fb_data->cur_mode =
					 &fb_data->pdata->spdc_mode[i];
				break;
			}
	vmode = fb_data->cur_mode->vmode;

	/* Allocate color map for the FB */
	cmap_size = 256;
	ret = fb_alloc_cmap(&info->cmap, cmap_size, 0);
	if (ret)
		goto dealloc_fb;

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	fb_data->max_pix_size = PAGE_ALIGN(xres_virt * yres_virt);

	/*
	 * Have to check to see if aligned buffer size when rotated
	 * is bigger than when not rotated, and use the max
	 */
	xres_virt_rot = ALIGN(vmode->yres, 32);
	yres_virt_rot = ALIGN(vmode->xres, 128);
	pix_size_rot = PAGE_ALIGN(xres_virt_rot * yres_virt_rot);
	fb_data->max_pix_size = (fb_data->max_pix_size > pix_size_rot) ?
				fb_data->max_pix_size : pix_size_rot;
	buf_size = fb_data->max_pix_size * fb_data->default_bpp/8;

	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	dev_dbg(&pdev->dev, "memory allocate: %d\n", fb_data->map_size);

	/* get IO memory*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory register\n");
		ret = -ENXIO;
		goto release_cmap;
	}

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto release_cmap;
	}

	fb_data->hwp = ioremap(res->start, SZ_4K);
	if (fb_data->hwp == NULL) {
		dev_err(&pdev->dev, "ioremap registers failed\n");
		ret = -ENOENT;
		goto release_mem;
	}

	/* Allocate FB memory */
	fb_data->virt_start = dma_alloc_writecombine(&pdev->dev,
		fb_data->map_size, &fb_data->phys_start, GFP_DMA);
	if (fb_data->virt_start == NULL) {
		dev_err(&pdev->dev, "probe err - dma_alloc for framebuffer\n");
		ret = -ENOMEM;
		goto release_hwp;
	}

	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres;
	var_info->yres = vmode->yres;
	var_info->xres_virtual = xres_virt;
	var_info->yres_virtual = yres_virt * fb_data->num_screens;
	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;

	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;
	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;
	case 8:
		var_info->grayscale = GRAYSCALE_8BIT;

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;
	case 4:
		var_info->grayscale = GRAYSCALE_4BIT;

		var_info->red.length = 4;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 4;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 4;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;
	default:
		dev_err(&pdev->dev, "unsupported bit-width:%d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto release_dma_fb;
	}

	fix_info = &info->fix;
	strcpy(fix_info->id, SPDC_DRIVER_NAME);
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	/* Set up FB info */
	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;
	info->screen_base = fb_data->virt_start;
	info->screen_size = info->fix.smem_len;
	info->fbops = &mxc_spdc_fb_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_spdc_fb_set_fix(info);

	/* use the same AXI and  PIX clock source */
	fb_data->spdc_clk_axi = clk_get(fb_data->dev, "epdc_axi");
	if (IS_ERR(fb_data->spdc_clk_axi)) {
		dev_err(&pdev->dev, "Unable to get AXI clk."
			"err = 0x%x\n", (int)fb_data->spdc_clk_axi);
		ret = -ENODEV;
		goto release_dma_fb;
	}
	fb_data->spdc_clk_pix = clk_get(fb_data->dev, "epdc_pix");
	if (IS_ERR(fb_data->spdc_clk_pix)) {
		dev_err(&pdev->dev, "Unable to get pix clk."
			"err = 0x%x\n", (int)fb_data->spdc_clk_pix);
		ret = -ENODEV;
		goto release_dma_fb;
	}

	/*
	 * Initialize update list and allocate buffer.
	 */
	INIT_LIST_HEAD(&fb_data->upd_pending_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_queue);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_preprocess_list);
	INIT_LIST_HEAD(&fb_data->full_marker_list);

	fb_data->max_num_updates = SPDC_MAX_NUM_UPDATES;
	fb_data->max_num_buffers = SPDC_MAX_NUM_BUFFERS;

	/* Allocate free buffers */
	for (i = 0; i < fb_data->max_num_updates; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto release_dma_fb;
		}

		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list);
	}

	/* Allocate PXP output buffer */
	fb_data->virt_addr_updbuf =
		kzalloc(sizeof(void *) * fb_data->max_num_buffers, GFP_KERNEL);
	fb_data->phys_addr_updbuf =
		kzalloc(sizeof(dma_addr_t) * fb_data->max_num_buffers,
			GFP_KERNEL);
	for (i = 0; i < fb_data->max_num_buffers; i++) {
		/*
		 * Allocate memory for PxP output buffer.
		 * Output raw data is Y4 format.
		 * Each update buffer is 1/2 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */
		fb_data->virt_addr_updbuf[i] =
			dma_alloc_coherent(fb_data->info.device,
				fb_data->max_pix_size / 2,
				   &fb_data->phys_addr_updbuf[i], GFP_DMA);
		if (fb_data->virt_addr_updbuf[i] == NULL) {
			ret = -ENOMEM;
			goto release_freebuf_lists;
		}
	}
	/* Counter indicating which update buffer should be used next. */
	fb_data->upd_buffer_num = 0;

	/* Allocate memory for partical process region buffer.
	 *  Output raw data is Y4 format.
	 */
	fb_data->virt_addr_copybuf =
	dma_alloc_coherent(fb_data->info.device,
			fb_data->max_pix_size / 2,
			&fb_data->phys_addr_copybuf, GFP_DMA);
	if (fb_data->virt_addr_copybuf == NULL) {
		ret = -ENOMEM;
		goto release_output_buf;
	}

	/* Allocate next & current & privious & count & lut buffers.
	 *  next buffer size is Y4 raw data
	 */
	fb_data->next_buf_size =
		(fb_data->map_size / fb_data->num_screens) >> 1;
	fb_data->virt_next_buf =
		dma_alloc_coherent(&pdev->dev, fb_data->next_buf_size,
				&fb_data->phy_next_buf, GFP_DMA);
	if (fb_data->virt_next_buf == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for next buf!\n");
		ret = -ENOMEM;
		goto release_copy_buf;
	}

	fb_data->current_buf_size =
		(fb_data->map_size / fb_data->num_screens) >> 1;
	fb_data->virt_current_buf =
		dma_alloc_coherent(&pdev->dev, fb_data->current_buf_size,
				&fb_data->phy_current_buf, GFP_DMA);
	if (fb_data->virt_current_buf == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for current buf!\n");
		ret = -ENOMEM;
		goto release_next_buf;
	}

	fb_data->pre_buf_size =
		(fb_data->map_size / fb_data->num_screens) >> 1;
	fb_data->virt_pre_buf =
		dma_alloc_coherent(&pdev->dev, fb_data->pre_buf_size,
				&fb_data->phy_pre_buf, GFP_DMA);
	if (fb_data->virt_pre_buf == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for current buf!\n");
		ret = -ENOMEM;
		goto release_current_buf;
	}

	fb_data->cnt_buf_size = info->var.xres * info->var.yres;
	fb_data->virt_cnt_buf =
		dma_alloc_coherent(&pdev->dev, fb_data->cnt_buf_size,
				&fb_data->phy_cnt_buf, GFP_DMA);
	if (fb_data->virt_cnt_buf == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for current buf!\n");
		ret = -ENOMEM;
		goto release_pre_buf;
	}

	fb_data->lut_buf_size = SZ_1M;
	fb_data->virt_lut_buf =
		dma_alloc_coherent(&pdev->dev, fb_data->lut_buf_size,
				&fb_data->phy_lut_buf, GFP_DMA);
	if (fb_data->virt_lut_buf == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for current buf!\n");
		ret = -ENOMEM;
		goto release_cnt_buf;
	}

	/* Initialize SPDC pins */
	if (fb_data->pdata->get_pins)
		fb_data->pdata->get_pins();

	fb_data->hw_ready = false;
	fb_data->hw_initializing = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = SPDC_WAV_MODE_DEFAULT;
	fb_data->wv_modes.mode_du = SPDC_WAV_MODE_2;
	fb_data->wv_modes.mode_gc4 = SPDC_WAV_MODE_2;
	fb_data->wv_modes.mode_gc8 = SPDC_WAV_MODE_1;
	fb_data->wv_modes.mode_gc16 = SPDC_WAV_MODE_1;
	fb_data->wv_modes.mode_gc32 = SPDC_WAV_MODE_1;

	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->upd_scheme = UPDATE_SCHEME_QUEUE_AND_MERGE;
	fb_data->spdc_fb_var = *var_info;
	fb_data->fb_offset = 0;

	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->powering_down = false;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->pwrdown_delay = 0;
	fb_data->cur_update = NULL;
	fb_data->waiting_for_idle = false;
	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->wait_for_powerdown = false;
	fb_data->updates_active = false;

	spin_lock_init(&fb_data->lock);
	mutex_init(&fb_data->queue_mutex);
	mutex_init(&fb_data->pxp_mutex);
	mutex_init(&fb_data->power_mutex);
	init_completion(&fb_data->lut_down);
	init_completion(&fb_data->init_finish);
	init_completion(&fb_data->update_finish);
	INIT_DELAYED_WORK(&fb_data->spdc_done_work, spdc_done_work_func);

	fb_data->spdc_submit_workqueue = alloc_workqueue("SPDC Submit",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->spdc_submit_work, spdc_submit_work_func);
	fb_data->spdc_intr_workqueue = alloc_workqueue("SPDC Interrupt",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->spdc_intr_work, spdc_intr_work_func);

	/* Retrieve spdc IRQ num */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto release_lut_buf;
	}
	fb_data->spdc_irq = res->start;
	ret = request_irq(fb_data->spdc_irq, mxc_spdc_irq_handler, 0,
			"fb_dma", fb_data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			fb_data->spdc_irq, ret);
		ret = -ENODEV;
		goto release_lut_buf;
	}

	/* define deferred io */
	info->fbdefio = &mxc_spdc_fb_defio;
#ifdef CONFIG_FB_MXC_SIPIX_AUTO_UPDATE_MODE
	fb_deferred_io_init(info);
#endif

	/* get pmic regulators */
	fb_data->display_regulator = regulator_get(NULL, "DISPLAY");
	if (IS_ERR(fb_data->display_regulator)) {
		dev_err(&pdev->dev, "Unable to get display PMIC regulator."
			"err = 0x%x\n", (int)fb_data->display_regulator);
		ret = -ENODEV;
		goto release_irq;
	}
	fb_data->vcom_regulator = regulator_get(NULL, "VCOM");
	if (IS_ERR(fb_data->vcom_regulator)) {
		regulator_put(fb_data->display_regulator);
		dev_err(&pdev->dev, "Unable to get VCOM regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto release_regulator1;
	}
	fb_data->v3p3_regulator = regulator_get(NULL, "V3P3");
	if (IS_ERR(fb_data->v3p3_regulator)) {
		regulator_put(fb_data->vcom_regulator);
		regulator_put(fb_data->display_regulator);
		dev_err(&pdev->dev, "Unable to get V3P3 regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator);
		ret = -ENODEV;
		goto release_regulator2;
	}

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = fb_data->info.var.xres;
	proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = fb_data->info.var.yres;
	proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;
	proc_data->lut_map = NULL;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	for (i = 0; i < 8; i++) {
		pxp_conf->ol_param[i].combine_enable = false;
		pxp_conf->ol_param[i].width = 0;
		pxp_conf->ol_param[i].height = 0;
		pxp_conf->ol_param[i].pixel_fmt = PXP_PIX_FMT_RGB565;
		pxp_conf->ol_param[i].color_key_enable = false;
		pxp_conf->ol_param[i].color_key = -1;
		pxp_conf->ol_param[i].global_alpha_enable = false;
		pxp_conf->ol_param[i].global_alpha = 0;
		pxp_conf->ol_param[i].local_alpha_enable = false;
	}

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->info.var.xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GY04;
	pxp_conf->out_param.stride = pxp_conf->out_param.width;

	/* Initialize color map for conversion of 8-bit gray pixels */
	fb_data->pxp_conf.proc_data.lut_map = kmalloc(256, GFP_KERNEL);
	if (fb_data->pxp_conf.proc_data.lut_map == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for lut map!\n");
		ret = -ENOMEM;
		goto release_regulator3;
	}
	for (i = 0; i < 256; i++)
		fb_data->pxp_conf.proc_data.lut_map[i] = i;

	fb_data->pxp_conf.proc_data.lut_map_updated = true;

	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;

	/* Initialize Scatter-gather list containing 2 buffer addresses. */
	sg = fb_data->sg;
	sg_init_table(sg, 2);

	/*
	 * For use in PxP transfers:
	 * sg[0] holds the FB buffer pointer
	 * sg[1] holds the Output buffer pointer (configured before TX request)
	 */
	sg_dma_address(&sg[0]) = info->fix.smem_start;
	sg_set_page(&sg[0], virt_to_page(info->screen_base),
		info->fix.smem_len, offset_in_page(info->screen_base));

	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register framebuffer failed\n");
		goto release_lutmap;
	}

	ret = sysfs_create_group(&info->device->kobj, &spdc_attr_group);
	if (ret)
		dev_err(&pdev->dev, "Unable to create file from fb_attrs\n");

	/* use for spdc test */
	g_spdc_fb_data = fb_data;

	/* hw init */
	spdc_fb_dev_init(fb_data);

	/*detect spdc epd disp & tcon version*/
	get_spdc_version(fb_data);

	goto out;

release_lutmap:
	kfree(fb_data->pxp_conf.proc_data.lut_map);
release_regulator3:
	regulator_put(fb_data->v3p3_regulator);
release_regulator2:
	regulator_put(fb_data->vcom_regulator);
release_regulator1:
	regulator_put(fb_data->display_regulator);
release_irq:
	free_irq(fb_data->spdc_irq, fb_data);
release_lut_buf:
	dma_free_coherent(&pdev->dev, fb_data->lut_buf_size,
		fb_data->virt_lut_buf, fb_data->phy_lut_buf);
release_cnt_buf:
	dma_free_coherent(&pdev->dev, fb_data->cnt_buf_size,
		fb_data->virt_cnt_buf, fb_data->phy_cnt_buf);
release_pre_buf:
	dma_free_coherent(&pdev->dev, fb_data->pre_buf_size,
		fb_data->virt_pre_buf, fb_data->phy_pre_buf);
release_current_buf:
	dma_free_coherent(&pdev->dev, fb_data->current_buf_size,
		fb_data->virt_current_buf, fb_data->phy_current_buf);
release_next_buf:
	dma_free_coherent(&pdev->dev, fb_data->next_buf_size,
		fb_data->virt_next_buf, fb_data->phy_next_buf);
release_copy_buf:
	dma_free_writecombine(&pdev->dev, fb_data->max_pix_size / 2,
		fb_data->virt_addr_copybuf, fb_data->phys_addr_copybuf);
release_output_buf:
	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_writecombine(&pdev->dev,
			fb_data->max_pix_size / 2, fb_data->virt_addr_updbuf[i],
			fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);
release_freebuf_lists:
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
release_dma_fb:
	dma_free_writecombine(&pdev->dev,
	fb_data->map_size, fb_data->virt_start, fb_data->phys_start);
release_hwp:
	iounmap(fb_data->hwp);
release_mem:
	release_resource(mem);
release_cmap:
	fb_dealloc_cmap(&info->cmap);
dealloc_fb:
	framebuffer_release(info);
out:
	return ret;
}

static int mxc_spdc_fb_remove(struct platform_device *pdev)
{
	struct update_data_list *plist, *temp_list;
	mxc_spdc_t *fb_data = platform_get_drvdata(pdev);
	struct fb_info *info = &fb_data->info;
	int i;

	mxc_spdc_fb_blank(FB_BLANK_POWERDOWN, &fb_data->info);

	flush_workqueue(fb_data->spdc_submit_workqueue);
	destroy_workqueue(fb_data->spdc_submit_workqueue);

	regulator_put(fb_data->display_regulator);
	regulator_put(fb_data->vcom_regulator);
	regulator_put(fb_data->v3p3_regulator);

	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_writecombine(&pdev->dev,
				fb_data->max_pix_size,
				fb_data->virt_addr_updbuf[i],
				fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);

	/* free output temporary buffer */
	dma_free_writecombine(&pdev->dev, fb_data->max_pix_size / 2,
		fb_data->virt_addr_copybuf, fb_data->phys_addr_copybuf);

	list_for_each_entry_safe(plist, temp_list,
			&fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}

#if defined(CONFIG_FB_MXC_SIPIX_AUTO_UPDATE_MODE)
	fb_deferred_io_cleanup(&fb_data->info);
#endif

	/* free frame buffer */
	dma_free_writecombine(&pdev->dev, fb_data->map_size,
			info->screen_base, fb_data->phys_start);

	/* free SPDC hw allocate buffer */
	dma_free_coherent(&pdev->dev, fb_data->next_buf_size,
		fb_data->virt_next_buf, fb_data->phy_next_buf);
	dma_free_coherent(&pdev->dev, fb_data->current_buf_size,
		fb_data->virt_current_buf, fb_data->phy_current_buf);
	dma_free_coherent(&pdev->dev, fb_data->pre_buf_size,
		fb_data->virt_pre_buf, fb_data->phy_pre_buf);
	dma_free_coherent(&pdev->dev, fb_data->cnt_buf_size,
		fb_data->virt_cnt_buf, fb_data->phy_cnt_buf);
	dma_free_coherent(&pdev->dev, fb_data->lut_buf_size,
		fb_data->virt_lut_buf, fb_data->phy_lut_buf);

	sysfs_remove_group(&info->device->kobj, &spdc_attr_group);
	unregister_framebuffer(info);

	if (fb_data->pdata->put_pins)
		fb_data->pdata->put_pins();

	free_irq(fb_data->spdc_irq, fb_data);
	iounmap(fb_data->hwp);

	fb_dealloc_cmap(&info->cmap);

	framebuffer_release(info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int mxc_spdc_fb_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	mxc_spdc_t *data = platform_get_drvdata(pdev);
	int ret;

	data->pwrdown_delay = FB_POWERDOWN_DISABLE;
	ret = mxc_spdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);

	return ret;
}

static int mxc_spdc_fb_resume(struct platform_device *pdev)
{
	mxc_spdc_t *data = platform_get_drvdata(pdev);

	mxc_spdc_fb_blank(FB_BLANK_UNBLANK, &data->info);
	spdc_init_sequence(data);

	return 0;
}
#else
#define mxc_spdc_fb_suspend	NULL
#define mxc_spdc_fb_resume	NULL
#endif

static struct platform_driver mxc_spdc_fb_driver = {
	.probe = mxc_spdc_fb_probe,
	.remove = mxc_spdc_fb_remove,
	.suspend = mxc_spdc_fb_suspend,
	.resume = mxc_spdc_fb_resume,
	.driver = {
		.name = SPDC_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init mxc_spdc_fb_init(void)
{
	return platform_driver_register(&mxc_spdc_fb_driver);
}
late_initcall(mxc_spdc_fb_init);

static void __exit mxc_spdc_fb_exit(void)
{
	platform_driver_unregister(&mxc_spdc_fb_driver);
}
module_exit(mxc_spdc_fb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC SPDC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
