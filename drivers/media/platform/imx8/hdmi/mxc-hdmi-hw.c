/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "API_AFE_ss28fdsoi_hdmirx.h"
#include "mxc-hdmi-rx.h"

#include <asm/unaligned.h>
#include <linux/ktime.h>

u8 block0[128] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x3B, 0x10, 0xFD, 0x5A, 0x9B, 0x5F, 0x02, 0x00,
	0x19, 0x1C, 0x01, 0x04, 0xB3, 0x3C, 0x22, 0x78,
	0x9F, 0x30, 0x35, 0xA7, 0x55, 0x4E, 0xA3, 0x26,
	0x0F, 0x50, 0x54, 0x20, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0xFC, 0x00, 0x69, 0x4D, 0x58, 0x38, 0x51,
	0x4D, 0x20, 0x48, 0x44, 0x4D, 0x49, 0x52, 0x58,
	0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x18,
	0x3D, 0x19, 0x87, 0x1E, 0x01, 0x0A, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE2
};

u8 block1[128] = {

	0x02, 0x03, 0x1B, 0x71, 0x46, 0x90, 0x04, 0x03,
	0x5D, 0x1F, 0x22, 0x23, 0x09, 0x07, 0x07, 0x67,
	0x03, 0x0C, 0x00, 0x10, 0x00, 0x00, 0x3C, 0x83,
	0x01, 0x00, 0x00, 0x04, 0x74, 0x00, 0x30, 0xF2,
	0x70, 0x5A, 0x80, 0xB0, 0x58, 0x8A, 0x00, 0x20,
	0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD,
};

S_HDMI_SCDC_SET_MSG scdcExampleData = {
	.sink_ver = 1,
	.manufacturer_oui_1 = 0x1,
	.manufacturer_oui_2 = 0x2,
	.manufacturer_oui_3 = 0x3,
	.devId = {1, 2, 3, 4, 5, 6, 7, 8},
	.hardware_major_rev = 12,
	.hardware_minor_rev = 13,
	.software_major_rev = 0xAB,
	.software_minor_rev = 0XBC,
	.manufacturerSpecific = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	     20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34}
};

#define ktime_timeout_ms(ms_) ktime_add(ktime_get(), ms_to_ktime(ms_))
int infoframe_poll(state_struct *state, u8 type, u8 *buf)
{
	const time_t timeout = ktime_timeout_ms(1000);

	u32 reg;
	int ret = -1;
	u8 i, cs;

	/* Enable masking of invalid packets */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_TRANS_CTRL << 2), 1);

	/*
	 * Unmask interrupts for TYPE1 packet reception
	 * and packet transfer PKT transfer complete
	 */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INT_MASK << 2), ~0x10001);

	/* Invalidate packet */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_HEADER << 2), 0);
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_CTRL << 2),
		      F_PACKET_RDN_WR(0x1) | F_PACKET_NUM(0x0));
	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2),
			     &reg);
		if (ktime_after(ktime_get(), timeout))
			goto exit;
	} while (!(reg & (1 << 16)));

	/* Set packet type to wait for */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2),
		      F_INFO_TYPE1(type));

	/* Wait for Infoframe of TYPE1 type */
	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2),
			     &reg);
		if (ktime_after(ktime_get(), timeout))
			goto exit;
	} while (!(reg & (1 << 0)));

	/* Load Infoframe contents to registers */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_CTRL << 2),
		      F_PACKET_RDN_WR(0x0) | F_PACKET_NUM(0x0));
	do {
		cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2),
			     &reg);
		if (ktime_after(ktime_get(), timeout))
			goto exit;
	} while (!(reg & (1 << 16)));

	/* Read data */
	for (i = 0; i < 8; ++i) {
		cdn_apb_read(state,
			     ADDR_SINK_PIF + ((PKT_INFO_HEADER + i) << 2),
			     &reg);
		put_unaligned_le32(reg, buf + i * 4);
	}

	/* Check if InfoFrame size is within maximum range */
	if (buf[2] > 31)
		goto exit;

	/* Veryfy checksum */
	for (cs = 0, i = 0; i < 5 + buf[2]; ++i)
		cs += buf[i];
	if (cs)
		goto exit;

	ret = 0;

exit:
	/* Disable and clear interrupts */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INT_MASK << 2), ~0x00000);
	cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &reg);
	/* Disable detection of Infoframes */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2),
		      F_INFO_TYPE1(0));

	return ret;
}

/*
 * Exemplary code to configure the controller for the AVI_InfoFrame reception.
 * Returns -1 on error, 0 if AVI did not change, 1 if AVI changed.
 */
int get_avi_infoframe(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	int ret;
	u8 buf[32];
	/* AVI Data Byte 1 is at offset 5 */
	u8 *avi = &buf[5];

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);

	/* Try to receive AVI */
	ret = infoframe_poll(state, 0x82, buf);
	if (ret < 0)
		return -1;

	/* Check if header is valid (version 2 only) */
	if (buf[0] != 0x82 || buf[1] != 0x02 || buf[2] != 0x0D || buf[3] != 0)
		return -1;

	/*
	 * Check if received AVI differs from previous one.
	 * Compare only data bytes 1-5.
	 */
	if (!memcmp(hdmi_rx->avi, avi, 5))
		return 0;

	pr_info("--- avi infoframe new: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n",
		buf[0], buf[1], buf[2], buf[3], buf[4],
		buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
		buf[12], buf[13], buf[14], buf[15], buf[16], buf[17]);
	pr_info("--- avi infoframe old: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n",
		hdmi_rx->avi[0], hdmi_rx->avi[1], hdmi_rx->avi[2],
		hdmi_rx->avi[3], hdmi_rx->avi[4], hdmi_rx->avi[5],
		hdmi_rx->avi[6], hdmi_rx->avi[7], hdmi_rx->avi[8],
		hdmi_rx->avi[9], hdmi_rx->avi[10], hdmi_rx->avi[11],
		hdmi_rx->avi[12]);

	memcpy(hdmi_rx->avi, avi, sizeof(hdmi_rx->avi));

	hdmi_rx->vic_code = avi[3] & 0x7F; /* VIC value */
	hdmi_rx->pixel_encoding = avi[0] >> 5 & 3; /* Y value */

	return 1;
}

static int get_vendor_infoframe(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	u32 ieee_oui;
	int ret;
	u8 buf[32];

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);

	/* Try to receive AVI */
	ret = infoframe_poll(state, 0x81, buf);
	if (ret < 0)
		return -1;

	/* Check if header is valid (version 1 only) */
	if (buf[0] != 0x81 || buf[1] != 0x01)
		return -1;

	ieee_oui = buf[5] | buf[6] << 8 | buf[7] << 16;

	/* Get IEEE OUI */
	if (ieee_oui == 0x000C03)
		dev_info(&hdmi_rx->pdev->dev, "HDMI 1.4b Vendor Specific Infoframe\n");
	else if (ieee_oui == 0xC45DD8)
		dev_info(&hdmi_rx->pdev->dev, "HDMI 2.0 Vendor Specific Infoframe\n");
	else
		dev_err(&hdmi_rx->pdev->dev,
			"Error Vendro Infoframe IEEE OUI=0x%6X\n", ieee_oui);

	/* Extened resoluction format */
	if ((buf[9] >> 5 & 0x07) == 1) {
		hdmi_rx->hdmi_vic = buf[10];
		dev_info(&hdmi_rx->pdev->dev, "hdmi_vic=%d\n", hdmi_rx->hdmi_vic);
	}

	return 0;
}

static void get_color_depth(struct mxc_hdmi_rx_dev *hdmi_rx,
			    clk_ratio_t clk_ratio)
{
	u8 pixel_encoding = hdmi_rx->pixel_encoding;

	hdmi_rx->color_depth = 8;

	switch (pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_4:
		case CLK_RATIO_3_2:
		case CLK_RATIO_2_1:
		case CLK_RATIO_1_2:
		case CLK_RATIO_5_8:
		case CLK_RATIO_3_4:
		default:
			pr_err("YUV422 Not supported clk ration\n");
		}
		break;
	case PIXEL_ENCODING_YUV420:
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_8:
			hdmi_rx->color_depth = 10;
			break;
		case CLK_RATIO_3_4:
			hdmi_rx->color_depth = 12;
			break;
		case CLK_RATIO_5_4:
		case CLK_RATIO_3_2:
		case CLK_RATIO_2_1:
		default:
			pr_err("YUV420 Not supported clk ration\n");
		}
		break;
	default:		/* RGB/YUV444 */
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_4:
			hdmi_rx->color_depth = 10;
			break;
		case CLK_RATIO_3_2:
			hdmi_rx->color_depth = 12;
			break;
		case CLK_RATIO_2_1:
			hdmi_rx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
		case CLK_RATIO_5_8:
		case CLK_RATIO_3_4:
		default:
			pr_err("RGB/YUV444 Not supported clk ration\n");
		}
	}
}

/* Set edid data sample */
void hdmirx_edid_set(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);

	/* Set EDID - block 0 */
	CDN_API_HDMIRX_SET_EDID_blocking(state, 0, 0, &block0[0]);
	/* Set EDID - block 1 */
	CDN_API_HDMIRX_SET_EDID_blocking(state, 0, 1, &block1[0]);
	dev_dbg(&hdmi_rx->pdev->dev, "EDID block 0/1 set complete.\n");
}

/* Set SCDC data sample */
void hdmirx_scdc_set(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);

	CDN_API_HDMIRX_SET_SCDC_SLAVE_blocking(state, &scdcExampleData);
	dev_dbg(&hdmi_rx->pdev->dev, "SCDC set complete.\n");
}

/* Set AVI data */
static inline void hdmirx_avi_set(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);

	memset(hdmi_rx->avi, 0, sizeof(hdmi_rx->avi));
}

int hdprx_fw_check(state_struct *state)
{
	u16 ver, verlib;
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	int ret;

	ret = CDN_API_General_Test_Echo_Ext_blocking(state,
						     echo_msg,
						     echo_resp,
						     sizeof(echo_msg),
						     CDN_BUS_TYPE_APB);

	if (strncmp(echo_msg, echo_resp, sizeof(echo_msg)) != 0) {
		pr_err("CDN_API_General_Test_Echo_Ext_blocking - echo test failed, check HDMI RX firmware!");
		return -ENXIO;
	}
	pr_info("CDN_API_General_Test_Echo_Ext_blocking - HDMI RX APB(ret = %d echo_resp = %s)\n",
		 ret, echo_resp);

	ret = CDN_API_General_getCurVersion(state, &ver, &verlib);
	if (ret != 0) {
		pr_err("CDN_API_General_getCurVersion - check HDMI RX firmware!\n");
		return -ENXIO;
	}
	pr_info("CDN_API_General_getCurVersion - HDMI RX ver %d verlib %d\n",
		ver, verlib);
	/* we can add a check here to reject older firmware
	 * versions if needed
	 */

	return 0;
}

int hdmirx_init(state_struct *state)
{
	u8 sts;
	u32 ret = 0;

	/* Set uCPU Clock frequency for FW's use [MHz]; */
	CDN_API_SetClock(state, 200);

	/* Relase uCPU */
	cdn_apb_write(state, ADDR_APB_CFG, 0);

	/* Checkpr_info( if the firmware is running */
	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		pr_err("NO HDMI RX FW running\n");
		return -ENXIO;
	}

	/* Check firmware echo and vesion */
	hdprx_fw_check(state);

	/* Set driver and firmware active */
	CDN_API_MainControl_blocking(state, 1, &sts);

	/* Set sample edid and scdc */
	hdmirx_edid_set(state);
	hdmirx_scdc_set(state);
	hdmirx_avi_set(state);

	return ret;
}

int hdmirx_get_hpd_state(state_struct *state, u8 *hpd)
{
	pr_info("%s\n", __func__);
	return CDN_API_General_GetHpdState_blocking(state, hpd);
}

void hdmirx_hotplug_trigger(state_struct *state)
{
	/* Clear HPD */
	CDN_API_HDMIRX_SetHpd_blocking(state, 0);
	/* provide minimum low pulse length (100ms) */
	msleep(110);
	CDN_API_HDMIRX_SetHpd_blocking(state, 1);
}

int hdmirx_config(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	S_HDMI_SCDC_GET_MSG *scdcData = &hdmi_rx->scdcData;
	clk_ratio_t clk_ratio, clk_ratio_detected;
	u8 data_rate_change = 0;
	u8 scrambling_en;
	tmds_bit_clock_ratio_t tmds_bit_clock_ratio;

	int ret;

	/* Get TMDS_Bit_Clock_Ratio and Scrambling setting */
	CDN_API_HDMIRX_GET_SCDC_SLAVE_blocking(state, scdcData);
	tmds_bit_clock_ratio = ((scdcData->TMDS_Config & (1 << 1)) >> 1) ?
			       TMDS_BIT_CLOCK_RATIO_1_40 :
			       TMDS_BIT_CLOCK_RATIO_1_10;
	scrambling_en = scdcData->TMDS_Config & (1 << 0);

	dev_dbg(&hdmi_rx->pdev->dev,
		"TMDS ratio: 1/%0d, Scrambling %0d).\n",
		tmds_bit_clock_ratio, scrambling_en);

	/* Configure the PHY */
	pma_config(state);

	dev_info(&hdmi_rx->pdev->dev, "Releasing PHY reset\n");
	/* Reset HDMI RX PHY */
	imx8qm_hdmi_phy_reset(state, 1);

	dev_info(&hdmi_rx->pdev->dev, "Waiting for pma_cmn_ready\n");
	ret = pma_cmn_ready(state);
	if (ret < 0) {
		dev_err(&hdmi_rx->pdev->dev, "pma_cmn_ready failed\n");
		return -1;
	}

	/* init ARC */
	arc_config(state);

	/* Detect rx clk signal */
	ret = pma_rx_clk_signal_detect(state);
	if (ret < 0) {
		dev_err(&hdmi_rx->pdev->dev, "Common rx_clk signal detect failed\n");
		return -1;
	}
	dev_info(&hdmi_rx->pdev->dev, "pma_rx_clk_signal detected\n");

	/* Get TMDS clock frequency */
	hdmi_rx->tmds_clk = hdmirx_get_stable_tmds(state);
	if (hdmi_rx->tmds_clk <= 0) {
		dev_err(&hdmi_rx->pdev->dev, "detect tmds clock failed\n");
		return -1;
	}
	dev_info(&hdmi_rx->pdev->dev, "detect TMDS clock freq: %d kHz\n",
		 hdmi_rx->tmds_clk);

	/* Start from TMDS/pixel clock ratio of 1:1.
	 * It affects only pixel clock frequency as the character/data clocks
	 * are generated based on a measured TMDS clock.
	 * This guarantees that the TMDS characters are correctly decoded in
	 * the controller regardless of the pixel clock ratio being
	 * programmed.
	 */
	clk_ratio = CLK_RATIO_1_1;

	ret = pma_pll_config(state, hdmi_rx->tmds_clk, clk_ratio,
			     tmds_bit_clock_ratio, data_rate_change);
	if (ret < 0) {
		dev_err(&hdmi_rx->pdev->dev, "pma_pll_config failed\n");
		return -1;
	}

	/* Setup the scrambling mode */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_CTRL << 2),
						F_SCRAMBLER_MODE(scrambling_en));
	dev_info(&hdmi_rx->pdev->dev,
				"Scrambling %s.\n", (scrambling_en) ? "enabled" : "disabled");
	/*Just to initiate the counters: */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_CNT_INT_CTRL << 2),
						F_SCRAMBLER_SSCP_LINE_DET_THR(0) |
						F_SCRAMBLER_CTRL_LINE_DET_THR(0));
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_VALID_CTRL << 2),
						F_SCRAMBLER_SSCP_LINE_VALID_THR(1) |
						F_SCRAMBLER_CTRL_LINE_VALID_THR(0));

	/* The PHY got programmed with the assumed TMDS/pixel clock ratio of 1:1.
	 * Implement the link training procedure to find out the real clock ratio:
	 * 1. Wait for AVI InfoFrame packet
	 * 2. Get the VIC code and pixel encoding from the packet
	 * 3. Evaluate the TMDS/pixel clock ratio based on the vic_table.c
	 * 4. Compare the programmed clock ratio with evaluated one
	 * 5. If mismatch found - reprogram the PHY
	 * 6. Enable the video data path in the controller */
	ret = get_avi_infoframe(state);
	if (ret < 0)
		dev_err(&hdmi_rx->pdev->dev, "Get AVI info frame failed\n");

	ret = get_vendor_infoframe(state);
	if (ret < 0)
		dev_warn(&hdmi_rx->pdev->dev, "No Vendor info frame\n");

	dev_info(&hdmi_rx->pdev->dev, "VIC: %0d, pixel_encoding: %0d.\n",
			hdmi_rx->vic_code, hdmi_rx->pixel_encoding);
	ret = mxc_hdmi_frame_timing(hdmi_rx);
	if (ret < 0) {
		dev_err(&hdmi_rx->pdev->dev, "Get frame timing failed\n\n");
		return -1;
	}

	clk_ratio_detected = clk_ratio_detect(state, hdmi_rx->tmds_clk,
					      hdmi_rx->timings->timings.bt.
					      pixelclock / 1000,
					      hdmi_rx->vic_code,
					      hdmi_rx->pixel_encoding,
					      tmds_bit_clock_ratio);

	data_rate_change = (clk_ratio != clk_ratio_detected);
	if (data_rate_change) {
		dev_dbg(&hdmi_rx->pdev->dev,
				"TMDS/pixel clock ratio mismatch detected (programmed: %0d, detected: %0d)\n",
				clk_ratio, clk_ratio_detected);

		/* Reconfigure the PHY */
		pre_data_rate_change(state);
		pma_rx_clk_signal_detect(state);
		hdmi_rx->tmds_clk = hdmirx_get_stable_tmds(state);
		pma_pll_config(state, hdmi_rx->tmds_clk, clk_ratio_detected,
			       tmds_bit_clock_ratio, data_rate_change);
	} else
		dev_info(&hdmi_rx->pdev->dev, "TMDS/pixel clock ratio correct\n");

	get_color_depth(hdmi_rx, clk_ratio_detected);
	switch (hdmi_rx->pixel_encoding) {
		case PIXEL_ENCODING_YUV422:
			dev_info(&hdmi_rx->pdev->dev, "Detect mode VIC %d %dbit YUV422\n",
					hdmi_rx->vic_code,  hdmi_rx->color_depth);
			break;
		case PIXEL_ENCODING_YUV420:
			dev_info(&hdmi_rx->pdev->dev, "Detect mode VIC %d %dbit YUV420\n",
					hdmi_rx->vic_code,  hdmi_rx->color_depth);
			break;
		case PIXEL_ENCODING_YUV444:
			dev_info(&hdmi_rx->pdev->dev, "Detect mode VIC %d %dbit YUV444\n",
					hdmi_rx->vic_code,  hdmi_rx->color_depth);
			break;
		case PIXEL_ENCODING_RGB:
			dev_info(&hdmi_rx->pdev->dev, "Detect mode VIC %d %dbit RGB\n",
					hdmi_rx->vic_code, hdmi_rx->color_depth);
			break;
		default:
			dev_err(&hdmi_rx->pdev->dev, "Unknow color format\n");
	}

	{
	u8 sts;
	/* Do post PHY programming settings */
	CDN_API_MainControl_blocking(state, 0x80, &sts);
	dev_dbg(&hdmi_rx->pdev->dev,
		"CDN_API_MainControl_blocking() Stage 2 complete.\n");
	}

	return 0;
}

/* Bring-up sequence for the HDMI-RX */
int hdmirx_startup(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	int ret = 0;

	ret = hdmirx_config(state);
	if (ret) {
		dev_err(&hdmi_rx->pdev->dev, "PHY configuration failed\n");
		hdmi_rx->tmds_clk = -1;
		return ret;
	}

	/* Initialize HDMI RX */
	CDN_API_HDMIRX_Init_blocking(state);
	dev_dbg(&hdmi_rx->pdev->dev,
				"CDN_API_HDMIRX_Init_blocking() complete.\n");

	/* XXX dkos: needed? done in fw in MainControl */
	/* Initialize HDMI RX CEC */
	CDN_API_General_Write_Register_blocking(state,
				ADDR_SINK_CAR + (SINK_CEC_CAR << 2),
				F_SINK_CEC_SYS_CLK_EN(1) |
				F_SINK_CEC_SYS_CLK_RSTN_EN(1));

	return 0;
}

/* Stop for the HDMI-RX */
void hdmirx_stop(state_struct *state)
{
	CDN_API_HDMIRX_Stop_blocking(state);
}

void hdmirx_phy_pix_engine_reset(state_struct *state)
{
	GENERAL_Read_Register_response regresp;

	CDN_API_General_Read_Register_blocking(state, ADDR_SINK_CAR +
					       (SINK_MHL_HD_CAR << 2),
					       &regresp);
	CDN_API_General_Write_Register_blocking(state, ADDR_SINK_CAR +
						(SINK_MHL_HD_CAR << 2),
						regresp.val & 0x3D);
	CDN_API_General_Write_Register_blocking(state, ADDR_SINK_CAR +
						(SINK_MHL_HD_CAR << 2),
						regresp.val);
}
