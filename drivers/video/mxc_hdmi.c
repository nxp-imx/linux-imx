/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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
 * SH-Mobile High-Definition Multimedia Interface (HDMI) driver
 * for SLISHDMI13T and SLIPHDMIT IP cores
 *
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>
#include <linux/ipu.h>

#include <linux/console.h>
#include <linux/types.h>

#include <mach/mxc_edid.h>
#include "mxc/mxc_dispdrv.h"

#include <mach/mxc_hdmi.h>

#define DISPDRV_HDMI	"hdmi"
#define HDMI_EDID_LEN		512

#define TRUE 1
#define FALSE 0

#define NUM_CEA_VIDEO_MODES	64
#define DEFAULT_VIDEO_MODE	16 /* 1080P */

/* VIC = Video ID Code */
static struct fb_videomode hdmi_cea_video_modes[NUM_CEA_VIDEO_MODES] = {
	{ /* 1 */
		.xres = 640,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 60,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 60,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 60,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 60,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 240,
		.refresh = 60,
	},
	{
		.xres = 720,
		.yres = 240,
		.refresh = 60,
	},
	{ /* 10 */
		.xres = 2880,
		.yres = 480,
		.refresh = 60,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 2880,
		.yres = 480,
		.refresh = 60,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 2880,
		.yres = 240,
		.refresh = 60,
	},
	{
		.xres = 2880,
		.yres = 240,
		.refresh = 60,
	},
	{
		.xres = 1440,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 1440,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 60,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 50,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 50,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 50,
	},
	{ /* 20 */
		.xres = 1920,
		.yres = 1080,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 288,
		.refresh = 50,
	},
	{
		.xres = 720,
		.yres = 288,
		.refresh = 50,
	},
	{
		.xres = 2880,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 2880,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 2880,
		.yres = 288,
		.refresh = 50,
	},
	{
		.xres = 2880,
		.yres = 288,
		.refresh = 50,
	},
	{
		.xres = 1440,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{ /* 30 */
		.xres = 1440,
		.yres = 576,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 50,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 24,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 25,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 30,
	},
	{
		.xres = 2880,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 2880,
		.yres = 480,
		.refresh = 60,
	},
	{
		.xres = 2880,
		.yres = 576,
		.refresh = 50,
	},
	{
		.xres = 2880,
		.yres = 576,
		.refresh = 50,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 50,
		.flag = FB_VMODE_INTERLACED,
	},
	{ /* 40 */
		.xres = 1920,
		.yres = 1080,
		.refresh = 100,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 100,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 100,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 100,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 100,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 100,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 120,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 120,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 120,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 120,
	},
	{ /* 50 */
		.xres = 720,
		.yres = 480,
		.refresh = 120,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 120,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 200,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 200,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 200,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 576,
		.refresh = 200,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 240,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 240,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 240,
		.flag = FB_VMODE_INTERLACED,
	},
	{
		.xres = 720,
		.yres = 480,
		.refresh = 240,
		.flag = FB_VMODE_INTERLACED,
	},
	{ /* 60 */
		.xres = 1280,
		.yres = 720,
		.refresh = 24,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 25,
	},
	{
		.xres = 1280,
		.yres = 720,
		.refresh = 30,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 120,
	},
	{
		.xres = 1920,
		.yres = 1080,
		.refresh = 100,
	},
};

enum hdmi_datamap {
	RGB444_8B = 0x01,
	RGB444_10B = 0x03,
	RGB444_12B = 0x05,
	RGB444_16B = 0x07,
	YCbCr444_8B = 0x09,
	YCbCr444_10B = 0x0B,
	YCbCr444_12B = 0x0D,
	YCbCr444_16B = 0x0F,
	YCbCr422_8B = 0x16,
	YCbCr422_10B = 0x14,
	YCbCr422_12B = 0x12,
};

enum hdmi_csc_enc_format {
	eRGB = 0x0,
	eYCC444 = 0x01,
	eYCC422 = 0x2,
	eExtended = 0x3,
};

enum hdmi_colorimetry {
	eITU601,
	eITU709,
};

struct hdmi_vmode {
	unsigned int mCode;
	unsigned int mHdmiDviSel;
	unsigned int mRVBlankInOSC;
	unsigned int mRefreshRate;
	unsigned int mHImageSize;
	unsigned int mVImageSize;
	unsigned int mHActive;
	unsigned int mVActive;
	unsigned int mHBlanking;
	unsigned int mVBlanking;
	unsigned int mHSyncOffset;
	unsigned int mVSyncOffset;
	unsigned int mHSyncPulseWidth;
	unsigned int mVSyncPulseWidth;
	unsigned int mHSyncPolarity;
	unsigned int mVSyncPolarity;
	unsigned int mDataEnablePolarity;
	unsigned int mInterlaced;
	unsigned int mPixelClock;
	unsigned int mHBorder;
	unsigned int mVBorder;
	unsigned int mPixelRepetitionInput;
};

struct hdmi_data_info {
	unsigned int enc_in_format;
	unsigned int enc_out_format;
	unsigned int enc_color_depth;
	unsigned int colorimetry;
	unsigned int pix_repet_factor;
	unsigned int hdcp_enable;
	struct hdmi_vmode video_mode;
};

enum hotplug_state {
	HDMI_HOTPLUG_DISCONNECTED,
	HDMI_HOTPLUG_CONNECTED,
	HDMI_HOTPLUG_EDID_DONE,
};

struct mxc_hdmi {
	void __iomem *base;
	enum hotplug_state hp_state;	/* hot-plug status */

	struct regulator *io_reg;
	struct regulator *analog_reg;
	struct mxc_dispdrv_entry *disp_mxc_hdmi;
	struct hdmi_data_info hdmi_data;
	struct clk *hdmi_clk;

	struct mxc_edid_cfg edid_cfg;
	u8 edid[HDMI_EDID_LEN];

	struct platform_device *pdev;
	struct fb_info *fbi;
	struct mutex mutex;		/* Protect the info pointer */
	struct delayed_work edid_work;
	struct fb_var_screeninfo var;
	struct fb_monspecs monspec;
	struct notifier_block nb;
};

struct i2c_client *hdmi_i2c;

static irqreturn_t mxc_hdmi_hotplug(int irq, void *dev_id)
{
}

/*!
 * this submodule is responsible for the video/audio data composition.
 */
void hdmi_set_video_mode(struct hdmi_vmode *vmode)
{
	vmode->mHBorder = 0;
	vmode->mVBorder = 0;
	vmode->mPixelRepetitionInput = 0;
	vmode->mHImageSize = 16;
	vmode->mVImageSize = 9;

	switch (vmode->mCode) {
	case 1:				/* 640x480p @ 59.94/60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
		vmode->mHActive = 640;
		vmode->mVActive = 480;
		vmode->mHBlanking = 160;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 16;
		vmode->mVSyncOffset = 10;
		vmode->mHSyncPulseWidth = 96;
		vmode->mVSyncPulseWidth = 2;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE; /* not(progressive_nI) */
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 2517 : 2520;
		break;
	case 2:				/* 720x480p @ 59.94/60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 3:				/* 720x480p @ 59.94/60Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 480;
		vmode->mHBlanking = 138;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 16;
		vmode->mVSyncOffset = 9;
		vmode->mHSyncPulseWidth = 62;
		vmode->mVSyncPulseWidth = 6;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 2700 : 2702;
		break;
	case 4:				/* 1280x720p @ 59.94/60Hz 16:9 */
		vmode->mHActive = 1280;
		vmode->mVActive = 720;
		vmode->mHBlanking = 370;
		vmode->mVBlanking = 30;
		vmode->mHSyncOffset = 110;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 40;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 7417 : 7425;
		break;
	case 5:				/* 1920x1080i @ 59.94/60Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 540;
		vmode->mHBlanking = 280;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 88;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 7417 : 7425;
		break;
	case 6:				/* 720(1440)x480i @ 59.94/60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 7:				/* 720(1440)x480i @ 59.94/60Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 240;
		vmode->mHBlanking = 276;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 38;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 124;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 2700 : 2702;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 8:	/* 720(1440)x240p @ 59.826/60.054/59.886/60.115Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 9:	/* 720(1440)x240p @ 59.826/60.054/59.886/60.115Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 240;
		vmode->mHBlanking = 276;
		vmode->mVBlanking = (vmode->mRefreshRate > 60000) ? 22 : 23;
		vmode->mHSyncOffset = 38;
		vmode->mVSyncOffset = (vmode->mRefreshRate > 60000) ? 4 : 5;
		vmode->mHSyncPulseWidth = 124;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			((vmode->mRefreshRate == 60054) ||
			vmode->mRefreshRate == 59826)
			? 2700 : 2702; /*  else 60.115/59.886 Hz */
		vmode->mPixelRepetitionInput = 1;
		break;
	case 10:			/* 2880x480i @ 59.94/60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 11:			/* 2880x480i @ 59.94/60Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 240;
		vmode->mHBlanking = 552;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 76;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 248;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 5400 : 5405;
		break;
	case 12:	/* 2880x240p @ 59.826/60.054/59.886/60.115Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 13:	/* 2880x240p @ 59.826/60.054/59.886/60.115Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 240;
		vmode->mHBlanking = 552;
		vmode->mVBlanking = (vmode->mRefreshRate > 60000) ? 22 : 23;
		vmode->mHSyncOffset = 76;
		vmode->mVSyncOffset = (vmode->mRefreshRate > 60000) ? 4 : 5;
		vmode->mHSyncPulseWidth = 248;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			((vmode->mRefreshRate == 60054) ||
			vmode->mRefreshRate == 59826)
			? 5400 : 5405; /*  else 60.115/59.886 Hz */
		break;
	case 14:			/* 1440x480p @ 59.94/60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 15:			/* 1440x480p @ 59.94/60Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 480;
		vmode->mHBlanking = 276;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 32;
		vmode->mVSyncOffset = 9;
		vmode->mHSyncPulseWidth = 124;
		vmode->mVSyncPulseWidth = 6;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 5400 : 5405;
		break;
	case 16:			/* 1920x1080p @ 59.94/60Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 1080;
		vmode->mHBlanking = 280;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 88;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 14835 : 14850;
		break;
	case 17:			/* 720x576p @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 18:			/* 720x576p @ 50Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 576;
		vmode->mHBlanking = 144;
		vmode->mVBlanking = 49;
		vmode->mHSyncOffset = 12;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 64;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 2700;
		break;
	case 19:			/* 1280x720p @ 50Hz 16:9 */
		vmode->mHActive = 1280;
		vmode->mVActive = 720;
		vmode->mHBlanking = 700;
		vmode->mVBlanking = 30;
		vmode->mHSyncOffset = 440;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 40;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 7425;
		break;
	case 20:			/* 1920x1080i @ 50Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 540;
		vmode->mHBlanking = 720;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 528;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 7425;
		break;
	case 21:			/* 720(1440)x576i @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 22:			/* 720(1440)x576i @ 50Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 288;
		vmode->mHBlanking = 288;
		vmode->mVBlanking = 24;
		vmode->mHSyncOffset = 24;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 126;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 2700;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 23:			/* 720(1440)x288p @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 24:			/* 720(1440)x288p @ 50Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 288;
		vmode->mHBlanking = 288;
		vmode->mVBlanking =
			(vmode->mRefreshRate == 50080) ?
			24 : ((vmode->mRefreshRate == 49920) ? 25 : 26);
		vmode->mHSyncOffset = 24;
		vmode->mVSyncOffset =
			(vmode->mRefreshRate == 50080) ?
			2 : ((vmode->mRefreshRate == 49920) ? 3 : 4);
		vmode->mHSyncPulseWidth = 126;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 2700;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 25:			/* 2880x576i @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 26:			/* 2880x576i @ 50Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 288;
		vmode->mHBlanking = 576;
		vmode->mVBlanking = 24;
		vmode->mHSyncOffset = 48;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 252;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 5400;
		break;
	case 27:			/* 2880x288p @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 28:			/* 2880x288p @ 50Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 288;
		vmode->mHBlanking = 576;
		vmode->mVBlanking =
			(vmode->mRefreshRate == 50080) ?
			24 : ((vmode->mRefreshRate == 49920) ? 25 : 26);
		vmode->mHSyncOffset = 48;
		vmode->mVSyncOffset =
			(vmode->mRefreshRate == 50080) ?
			2 : ((vmode->mRefreshRate == 49920) ? 3 : 4);
		vmode->mHSyncPulseWidth = 252;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 5400;
		break;
	case 29:			/* 1440x576p @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 30:			/* 1440x576p @ 50Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 576;
		vmode->mHBlanking = 288;
		vmode->mVBlanking = 49;
		vmode->mHSyncOffset = 24;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 128;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 5400;
		break;
	case 31:			/* 1920x1080p @ 50Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 1080;
		vmode->mHBlanking = 720;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 528;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 14850;
		break;
	case 32:			/* 1920x1080p @ 23.976/24Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 1080;
		vmode->mHBlanking = 830;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 638;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 23976) ? 7417 : 7425;
		break;
	case 33:			/* 1920x1080p @ 25Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 1080;
		vmode->mHBlanking = 720;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 528;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 7425;
		break;
	case 34:			/* 1920x1080p @ 29.97/30Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 1080;
		vmode->mHBlanking = 280;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 88;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 29970) ? 7417 : 7425;
		break;
	case 35:			/* 2880x480p @ 60Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 36:			/* 2880x480p @ 60Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 480;
		vmode->mHBlanking = 552;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 64;
		vmode->mVSyncOffset = 9;
		vmode->mHSyncPulseWidth = 248;
		vmode->mVSyncPulseWidth = 6;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 59940) ? 10800 : 10810;
		break;
	case 37:			/* 2880x576p @ 50Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 38:			/* 2880x576p @ 50Hz 16:9 */
		vmode->mHActive = 2880;
		vmode->mVActive = 576;
		vmode->mHBlanking = 576;
		vmode->mVBlanking = 49;
		vmode->mHSyncOffset = 48;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 256;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 10800;
		break;
	case 39:		/* 1920x1080i (1250 total) @ 50Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 540;
		vmode->mHBlanking = 384;
		vmode->mVBlanking = 85;
		vmode->mHSyncOffset = 32;
		vmode->mVSyncOffset = 23;
		vmode->mHSyncPulseWidth = 168;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 7200;
		break;
	case 40:			/* 1920x1080i @ 100Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 540;
		vmode->mHBlanking = 720;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 528;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 14850;
		break;
	case 41:			/* 1280x720p @ 100Hz 16:9 */
		vmode->mHActive = 1280;
		vmode->mVActive = 720;
		vmode->mHBlanking = 700;
		vmode->mVBlanking = 30;
		vmode->mHSyncOffset = 440;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 40;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 14850;
		break;
	case 42:			/* 720x576p @ 100Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 43:			/* 720x576p @ 100Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 576;
		vmode->mHBlanking = 144;
		vmode->mVBlanking = 49;
		vmode->mHSyncOffset = 12;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 64;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 5400;
		break;
	case 44:			/* 720(1440)x576i @ 100Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 45:			/* 720(1440)x576i @ 100Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 288;
		vmode->mHBlanking = 288;
		vmode->mVBlanking = 24;
		vmode->mHSyncOffset = 24;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 126;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 5400;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 46:			/* 1920x1080i @ 119.88/120Hz 16:9 */
		vmode->mHActive = 1920;
		vmode->mVActive = 540;
		vmode->mHBlanking = 288;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 88;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 44;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 119880) ? 14835 : 14850;
		break;
	case 47:			/* 1280x720p @ 119.88/120Hz 16:9 */
		vmode->mHActive = 1280;
		vmode->mVActive = 720;
		vmode->mHBlanking = 370;
		vmode->mVBlanking = 30;
		vmode->mHSyncOffset = 110;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 40;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = TRUE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 119880) ? 14835 : 14850;
		break;
	case 48:			/* 720x480p @ 119.88/120Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 49:			/* 720x480p @ 119.88/120Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 480;
		vmode->mHBlanking = 138;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 16;
		vmode->mVSyncOffset = 9;
		vmode->mHSyncPulseWidth = 62;
		vmode->mVSyncPulseWidth = 6;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 119880) ? 5400 : 5405;
		break;
	case 50:		/* 720(1440)x480i @ 119.88/120Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 51:		/* 720(1440)x480i @ 119.88/120Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 240;
		vmode->mHBlanking = 276;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 38;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 124;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 119880) ? 5400 : 5405;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 52:			/* 720X576p @ 200Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 53:			/* 720X576p @ 200Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 576;
		vmode->mHBlanking = 144;
		vmode->mVBlanking = 49;
		vmode->mHSyncOffset = 12;
		vmode->mVSyncOffset = 5;
		vmode->mHSyncPulseWidth = 64;
		vmode->mVSyncPulseWidth = 5;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock = 10800;
		break;
	case 54:			/* 720(1440)x576i @ 200Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 55:			/* 720(1440)x576i @ 200Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 288;
		vmode->mHBlanking = 288;
		vmode->mVBlanking = 24;
		vmode->mHSyncOffset = 24;
		vmode->mVSyncOffset = 2;
		vmode->mHSyncPulseWidth = 126;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock = 10800;
		vmode->mPixelRepetitionInput = 1;
		break;
	case 56:			/* 720x480p @ 239.76/240Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 57:			/* 720x480p @ 239.76/240Hz 16:9 */
		vmode->mHActive = 720;
		vmode->mVActive = 480;
		vmode->mHBlanking = 138;
		vmode->mVBlanking = 45;
		vmode->mHSyncOffset = 16;
		vmode->mVSyncOffset = 9;
		vmode->mHSyncPulseWidth = 62;
		vmode->mVSyncPulseWidth = 6;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = FALSE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 239760) ? 10800 : 10810;
		break;
	case 58:		/* 720(1440)x480i @ 239.76/240Hz 4:3 */
		vmode->mHImageSize = 4;
		vmode->mVImageSize = 3;
	case 59:		/* 720(1440)x480i @ 239.76/240Hz 16:9 */
		vmode->mHActive = 1440;
		vmode->mVActive = 240;
		vmode->mHBlanking = 276;
		vmode->mVBlanking = 22;
		vmode->mHSyncOffset = 38;
		vmode->mVSyncOffset = 4;
		vmode->mHSyncPulseWidth = 124;
		vmode->mVSyncPulseWidth = 3;
		vmode->mHSyncPolarity = vmode->mVSyncPolarity = FALSE;
		vmode->mInterlaced = TRUE;
		vmode->mPixelClock =
			(vmode->mRefreshRate == 239760) ? 10800 : 10810;
		vmode->mPixelRepetitionInput = 1;
		break;
	default:
		vmode->mCode = -1;
		return;
	}
	return;
}

/*!
 * this submodule is responsible for the video data synchronization.
 * for example, for RGB 4:4:4 input, the data map is defined as
 *			pin{47~40} <==> R[7:0]
 *			pin{31~24} <==> G[7:0]
 *			pin{15~8}  <==> B[7:0]
 */
void hdmi_video_sample(struct mxc_hdmi *hdmi)
{
	int color_format = 0;
	u8 val;

	if (hdmi->hdmi_data.enc_in_format == eRGB) {
		if (hdmi->hdmi_data.enc_color_depth == 8)
			color_format = 0x01;
		else if (hdmi->hdmi_data.enc_color_depth == 10)
			color_format = 0x03;
		else if (hdmi->hdmi_data.enc_color_depth == 12)
			color_format = 0x05;
		else if (hdmi->hdmi_data.enc_color_depth == 16)
			color_format = 0x07;
		else
			return;
	} else if (hdmi->hdmi_data.enc_in_format == eYCC444) {
		if (hdmi->hdmi_data.enc_color_depth == 8)
			color_format = 0x09;
		else if (hdmi->hdmi_data.enc_color_depth == 10)
			color_format = 0x0B;
		else if (hdmi->hdmi_data.enc_color_depth == 12)
			color_format = 0x0D;
		else if (hdmi->hdmi_data.enc_color_depth == 16)
			color_format = 0x0F;
		else
			return;
	} else if (hdmi->hdmi_data.enc_in_format == eYCC422) {
		if (hdmi->hdmi_data.enc_color_depth == 8)
			color_format = 0x16;
		else if (hdmi->hdmi_data.enc_color_depth == 10)
			color_format = 0x14;
		else if (hdmi->hdmi_data.enc_color_depth == 12)
			color_format = 0x12;
		else
			return;
	}

	val = HDMI_TX_INVID0_INTERNAL_DE_GENERATOR_DISABLE |
		((color_format << HDMI_TX_INVID0_VIDEO_MAPPING_OFFSET) &
		HDMI_TX_INVID0_VIDEO_MAPPING_MASK);
	writeb(val, hdmi->base + HDMI_TX_INVID0);

	/* Enable TX stuffing: When DE is inactive, fix the output data to 0 */
	val = HDMI_TX_INSTUFFING_BDBDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_RCRDATA_STUFFING_ENABLE |
		HDMI_TX_INSTUFFING_GYDATA_STUFFING_ENABLE;
	writeb(val, hdmi->base + HDMI_TX_INSTUFFING);
	writeb(0x0, hdmi->base + HDMI_TX_GYDATA0);
	writeb(0x0, hdmi->base + HDMI_TX_GYDATA1);
	writeb(0x0, hdmi->base + HDMI_TX_RCRDATA0);
	writeb(0x0, hdmi->base + HDMI_TX_RCRDATA1);
	writeb(0x0, hdmi->base + HDMI_TX_BCBDATA0);
	writeb(0x0, hdmi->base + HDMI_TX_BCBDATA1);
}

static int isColorSpaceConversion(struct mxc_hdmi *hdmi)
{
	return (hdmi->hdmi_data.enc_in_format !=
			hdmi->hdmi_data.enc_out_format) ? TRUE : FALSE;
}

static int isColorSpaceDecimation(struct mxc_hdmi *hdmi)
{
	return (hdmi->hdmi_data.enc_in_format !=
			hdmi->hdmi_data.enc_out_format) ? TRUE : FALSE;
}

static int isColorSpaceInterpolation(struct mxc_hdmi *hdmi)
{
	return ((hdmi->hdmi_data.enc_in_format == eYCC422) &&
			(hdmi->hdmi_data.enc_out_format == eRGB
			 || hdmi->hdmi_data.enc_out_format == eYCC444)) ?
			 TRUE : FALSE;
}

/*!
 * update the color space conversion coefficients.
 */
void update_csc_coeffs(struct mxc_hdmi *hdmi)
{
	unsigned short csc_coeff[3][4];
	unsigned int csc_scale = 1;
	u8 val;

	if (isColorSpaceConversion(hdmi) == TRUE) { /* csc needed */
		if (hdmi->hdmi_data.enc_out_format == eRGB) {
			if (hdmi->hdmi_data.colorimetry == eITU601) {
				csc_coeff[0][0] = 0x2000;
				csc_coeff[0][1] = 0x6926;
				csc_coeff[0][2] = 0x74fd;
				csc_coeff[0][3] = 0x010e;

				csc_coeff[1][0] = 0x2000;
				csc_coeff[1][1] = 0x2cdd;
				csc_coeff[1][2] = 0x0000;
				csc_coeff[1][3] = 0x7e9a;

				csc_coeff[2][0] = 0x2000;
				csc_coeff[2][1] = 0x0000;
				csc_coeff[2][2] = 0x38b4;
				csc_coeff[2][3] = 0x7e3b;

				csc_scale = 1;
			} else if (hdmi->hdmi_data.colorimetry == eITU709) {
				csc_coeff[0][0] = 0x2000;
				csc_coeff[0][1] = 0x7106;
				csc_coeff[0][2] = 0x7a02;
				csc_coeff[0][3] = 0x00a7;

				csc_coeff[1][0] = 0x2000;
				csc_coeff[1][1] = 0x3264;
				csc_coeff[1][2] = 0x0000;
				csc_coeff[1][3] = 0x7e6d;

				csc_coeff[2][0] = 0x2000;
				csc_coeff[2][1] = 0x0000;
				csc_coeff[2][2] = 0x3b61;
				csc_coeff[2][3] = 0x7e25;

				csc_scale = 1;
			}
		} else if (hdmi->hdmi_data.enc_in_format == eRGB) {
			if (hdmi->hdmi_data.colorimetry == eITU601) {
				csc_coeff[0][0] = 0x2591;
				csc_coeff[0][1] = 0x1322;
				csc_coeff[0][2] = 0x074b;
				csc_coeff[0][3] = 0x0000;

				csc_coeff[1][0] = 0x6535;
				csc_coeff[1][1] = 0x2000;
				csc_coeff[1][2] = 0x7acc;
				csc_coeff[1][3] = 0x0200;

				csc_coeff[1][0] = 0x6acd;
				csc_coeff[1][1] = 0x7534;
				csc_coeff[1][2] = 0x2000;
				csc_coeff[1][3] = 0x0200;

				csc_scale = 1;
			} else if (hdmi->hdmi_data.colorimetry == eITU709) {
				csc_coeff[0][0] = 0x2dc5;
				csc_coeff[0][1] = 0x0d9b;
				csc_coeff[0][2] = 0x049e;
				csc_coeff[0][3] = 0x0000;

				csc_coeff[1][0] = 0x63f0;
				csc_coeff[1][1] = 0x2000;
				csc_coeff[1][2] = 0x7d11;
				csc_coeff[1][3] = 0x0200;

				csc_coeff[2][0] = 0x6756;
				csc_coeff[2][1] = 0x78ab;
				csc_coeff[2][2] = 0x2000;
				csc_coeff[2][3] = 0x0200;

				csc_scale = 1;
			}
		}
	} else {
		csc_coeff[0][0] = 0x2000;
		csc_coeff[0][1] = 0x0000;
		csc_coeff[0][2] = 0x0000;
		csc_coeff[0][3] = 0x0000;

		csc_coeff[1][0] = 0x0000;
		csc_coeff[1][1] = 0x2000;
		csc_coeff[1][2] = 0x0000;
		csc_coeff[1][3] = 0x0000;

		csc_coeff[2][0] = 0x0000;
		csc_coeff[2][1] = 0x0000;
		csc_coeff[2][2] = 0x2000;
		csc_coeff[2][3] = 0x0000;

		csc_scale = 1;
	}

	/* Update CSC parameters in HDMI CSC registers */
	writeb((unsigned char)(csc_coeff[0][0] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_A1_LSB);
	writeb((unsigned char)(csc_coeff[0][0] >> 8),
		hdmi->base + HDMI_CSC_COEF_A1_MSB);
	writeb((unsigned char)(csc_coeff[0][1] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_A2_LSB);
	writeb((unsigned char)(csc_coeff[0][1] >> 8),
		hdmi->base + HDMI_CSC_COEF_A2_MSB);
	writeb((unsigned char)(csc_coeff[0][2] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_A3_LSB);
	writeb((unsigned char)(csc_coeff[0][2] >> 8),
		hdmi->base + HDMI_CSC_COEF_A3_MSB);
	writeb((unsigned char)(csc_coeff[0][3] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_A4_LSB);
	writeb((unsigned char)(csc_coeff[0][3] >> 8),
		hdmi->base + HDMI_CSC_COEF_A4_MSB);

	writeb((unsigned char)(csc_coeff[1][0] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_B1_LSB);
	writeb((unsigned char)(csc_coeff[1][0] >> 8),
		hdmi->base + HDMI_CSC_COEF_B1_MSB);
	writeb((unsigned char)(csc_coeff[1][1] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_B2_LSB);
	writeb((unsigned char)(csc_coeff[1][1] >> 8),
		hdmi->base + HDMI_CSC_COEF_B2_MSB);
	writeb((unsigned char)(csc_coeff[1][2] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_B3_LSB);
	writeb((unsigned char)(csc_coeff[1][2] >> 8),
		hdmi->base + HDMI_CSC_COEF_B3_MSB);
	writeb((unsigned char)(csc_coeff[1][3] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_B4_LSB);
	writeb((unsigned char)(csc_coeff[1][3] >> 8),
		hdmi->base + HDMI_CSC_COEF_B4_MSB);

	writeb((unsigned char)(csc_coeff[2][0] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_C1_LSB);
	writeb((unsigned char)(csc_coeff[2][0] >> 8),
		hdmi->base + HDMI_CSC_COEF_C1_MSB);
	writeb((unsigned char)(csc_coeff[2][1] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_C2_LSB);
	writeb((unsigned char)(csc_coeff[2][1] >> 8),
		hdmi->base + HDMI_CSC_COEF_C2_MSB);
	writeb((unsigned char)(csc_coeff[2][2] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_C3_LSB);
	writeb((unsigned char)(csc_coeff[2][2] >> 8),
		hdmi->base + HDMI_CSC_COEF_C3_MSB);
	writeb((unsigned char)(csc_coeff[2][3] & 0xFF),
		hdmi->base + HDMI_CSC_COEF_C4_LSB);
	writeb((unsigned char)(csc_coeff[2][3] >> 8),
		hdmi->base + HDMI_CSC_COEF_C4_MSB);

	readb(hdmi->base + HDMI_CSC_SCALE);
	val &= ~HDMI_CSC_SCALE_CSCSCALE_MASK;
	val |= csc_scale & HDMI_CSC_SCALE_CSCSCALE_MASK;
	writeb(val, hdmi->base + HDMI_CSC_SCALE);
}

void hdmi_video_csc(struct mxc_hdmi *hdmi)
{
	int color_depth = 0;
	int interpolation = HDMI_CSC_CFG_INTMODE_DISABLE;
	int decimation = 0;
	u8 val;

	/* YCC422 interpolation to 444 mode */
	if (isColorSpaceInterpolation(hdmi))
		interpolation = HDMI_CSC_CFG_INTMODE_CHROMA_INT_FORMULA1;
	else if (isColorSpaceDecimation(hdmi))
		decimation = HDMI_CSC_CFG_DECMODE_CHROMA_INT_FORMULA1;

	if (hdmi->hdmi_data.enc_color_depth == 8)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_24BPP;
	else if (hdmi->hdmi_data.enc_color_depth == 10)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_30BPP;
	else if (hdmi->hdmi_data.enc_color_depth == 12)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_36BPP;
	else if (hdmi->hdmi_data.enc_color_depth == 16)
		color_depth = HDMI_CSC_SCALE_CSC_COLORDE_PTH_48BPP;
	else
		return;

	/*configure the CSC registers */
	writeb(interpolation | decimation, hdmi->base + HDMI_CSC_CFG);
	val = readb(hdmi->base + HDMI_CSC_SCALE);
	val &= ~HDMI_CSC_SCALE_CSC_COLORDE_PTH_MASK;
	val |= color_depth;
	writeb(val, hdmi->base + HDMI_CSC_SCALE);

	update_csc_coeffs(hdmi);
}

/*!
 * HDMI video packetizer is used to packetize the data.
 * for example, if input is YCC422 mode or repeater is used,
 * data should be repacked this module can be bypassed.
 */
void hdmi_video_packetize(struct mxc_hdmi *hdmi)
{
	unsigned int color_depth = 0;
	unsigned int remap_size = HDMI_VP_REMAP_YCC422_16bit;
	unsigned int output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_PP;
	struct hdmi_data_info *hdmi_data = &hdmi->hdmi_data;
	u8 val;

	if (hdmi_data->enc_out_format == eRGB
		|| hdmi_data->enc_out_format == eYCC444) {
		if (hdmi_data->enc_color_depth == 0)
			output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS;
		else if (hdmi_data->enc_color_depth == 8) {
			color_depth = 4;
			output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS;
		} else if (hdmi_data->enc_color_depth == 10)
			color_depth = 5;
		else if (hdmi_data->enc_color_depth == 12)
			color_depth = 6;
		else if (hdmi_data->enc_color_depth == 16)
			color_depth = 7;
		else
			return;
	} else if (hdmi_data->enc_out_format == eYCC422) {
		if (hdmi_data->enc_color_depth == 0 ||
			hdmi_data->enc_color_depth == 8)
			remap_size = HDMI_VP_REMAP_YCC422_16bit;
		else if (hdmi_data->enc_color_depth == 10)
			remap_size = HDMI_VP_REMAP_YCC422_20bit;
		else if (hdmi_data->enc_color_depth == 12)
			remap_size = HDMI_VP_REMAP_YCC422_24bit;
		else
			return;
		output_select = HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422;
	} else
		return;

	/* set the packetizer registers */
	val = ((color_depth << HDMI_VP_PR_CD_COLOR_DEPTH_OFFSET) &
		HDMI_VP_PR_CD_COLOR_DEPTH_MASK) |
		((hdmi_data->pix_repet_factor <<
		HDMI_VP_PR_CD_DESIRED_PR_FACTOR_OFFSET) &
		HDMI_VP_PR_CD_DESIRED_PR_FACTOR_MASK);
	writeb(val, hdmi->base + HDMI_VP_PR_CD);

	val = readb(hdmi->base + HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_PR_STUFFING_MASK;
	val |= HDMI_VP_STUFF_PR_STUFFING_STUFFING_MODE;
	writeb(val, hdmi->base + HDMI_VP_STUFF);

	/* Data from pixel repeater block */
	if (hdmi_data->pix_repet_factor > 1) {
		val = readb(hdmi->base + HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_PR_EN_MASK |
			HDMI_VP_CONF_BYPASS_SELECT_MASK);
		val |= HDMI_VP_CONF_PR_EN_ENABLE |
			HDMI_VP_CONF_BYPASS_SELECT_PIX_REPEATER;
		writeb(val, hdmi->base + HDMI_VP_CONF);
	} else { /* data from packetizer block */
		val = readb(hdmi->base + HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_PR_EN_MASK |
			HDMI_VP_CONF_BYPASS_SELECT_MASK);
		val |= HDMI_VP_CONF_PR_EN_DISABLE |
			HDMI_VP_CONF_BYPASS_SELECT_VID_PACKETIZER;
		writeb(val, hdmi->base + HDMI_VP_CONF);
	}

	val = readb(hdmi->base + HDMI_VP_STUFF);
	val &= ~HDMI_VP_STUFF_IDEFAULT_PHASE_MASK;
	val |= 1 << HDMI_VP_STUFF_IDEFAULT_PHASE_OFFSET;
	writeb(val, hdmi->base + HDMI_VP_STUFF);

	writeb(remap_size, hdmi->base + HDMI_VP_REMAP);

	if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_PP) {
		val = readb(hdmi->base + HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_ENABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		writeb(val, hdmi->base + HDMI_VP_CONF);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_YCC422) {
		val = readb(hdmi->base + HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_DISABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_ENABLE;
		writeb(val, hdmi->base + HDMI_VP_CONF);
	} else if (output_select == HDMI_VP_CONF_OUTPUT_SELECTOR_BYPASS) {
		val = readb(hdmi->base + HDMI_VP_CONF);
		val &= ~(HDMI_VP_CONF_BYPASS_EN_MASK |
			HDMI_VP_CONF_PP_EN_ENMASK |
			HDMI_VP_CONF_YCC422_EN_MASK);
		val |= HDMI_VP_CONF_BYPASS_EN_ENABLE |
			HDMI_VP_CONF_PP_EN_DISABLE |
			HDMI_VP_CONF_YCC422_EN_DISABLE;
		writeb(val, hdmi->base + HDMI_VP_CONF);
	} else {
		return;
	}

	val = readb(hdmi->base + HDMI_VP_STUFF);
	val &= ~(HDMI_VP_STUFF_PP_STUFFING_MASK |
		HDMI_VP_STUFF_YCC422_STUFFING_MASK);
	val |= HDMI_VP_STUFF_PP_STUFFING_STUFFING_MODE |
		HDMI_VP_STUFF_YCC422_STUFFING_STUFFING_MODE;
	writeb(val, hdmi->base + HDMI_VP_STUFF);

	val = readb(hdmi->base + HDMI_VP_CONF);
	val &= ~HDMI_VP_CONF_OUTPUT_SELECTOR_MASK;
	val |= output_select;
	writeb(val, hdmi->base + HDMI_VP_CONF);
}

void hdmi_video_force_output(struct mxc_hdmi *hdmi, unsigned char force)
{
	u8 val;

	if (force == TRUE) {
		writeb(0x00, hdmi->base + HDMI_FC_DBGTMDS2);   /* R */
		writeb(0x00, hdmi->base + HDMI_FC_DBGTMDS1);   /* G */
		writeb(0xFF, hdmi->base + HDMI_FC_DBGTMDS0);   /* B */
		val = readb(hdmi->base + HDMI_FC_DBGFORCE);
		val |= HDMI_FC_DBGFORCE_FORCEVIDEO;
		writeb(val, hdmi->base + HDMI_FC_DBGFORCE);
	} else {
		val = readb(hdmi->base + HDMI_FC_DBGFORCE);
		val &= ~HDMI_FC_DBGFORCE_FORCEVIDEO;
		writeb(val, hdmi->base + HDMI_FC_DBGFORCE);
		writeb(0x00, hdmi->base + HDMI_FC_DBGTMDS2);   /* R */
		writeb(0x00, hdmi->base + HDMI_FC_DBGTMDS1);   /* G */
		writeb(0x00, hdmi->base + HDMI_FC_DBGTMDS0);   /* B */
	}
}

static inline void hdmi_phy_test_clear(struct mxc_hdmi *hdmi,
						unsigned char bit)
{
	u8 val = readb(hdmi->base + HDMI_PHY_TST0);
	val &= ~HDMI_PHY_TST0_TSTCLR_MASK;
	val |= (bit << HDMI_PHY_TST0_TSTCLR_OFFSET) &
		HDMI_PHY_TST0_TSTCLR_MASK;
	writeb(val, hdmi->base + HDMI_PHY_TST0);
}

static inline void hdmi_phy_test_enable(struct mxc_hdmi *hdmi,
						unsigned char bit)
{
	u8 val = readb(hdmi->base + HDMI_PHY_TST0);
	val &= ~HDMI_PHY_TST0_TSTEN_MASK;
	val |= (bit << HDMI_PHY_TST0_TSTEN_OFFSET) &
		HDMI_PHY_TST0_TSTEN_MASK;
	writeb(val, hdmi->base + HDMI_PHY_TST0);
}

static inline void hdmi_phy_test_clock(struct mxc_hdmi *hdmi,
						unsigned char bit)
{
	u8 val = readb(hdmi->base + HDMI_PHY_TST0);
	val &= ~HDMI_PHY_TST0_TSTCLK_MASK;
	val |= (bit << HDMI_PHY_TST0_TSTCLK_OFFSET) &
		HDMI_PHY_TST0_TSTCLK_MASK;
	writeb(val, hdmi->base + HDMI_PHY_TST0);
}

static inline void hdmi_phy_test_din(struct mxc_hdmi *hdmi,
						unsigned char bit)
{
	writeb(bit, hdmi->base + HDMI_PHY_TST1);
}

static inline void hdmi_phy_test_dout(struct mxc_hdmi *hdmi,
						unsigned char bit)
{
	writeb(bit, hdmi->base + HDMI_PHY_TST2);
}

static int hdmi_phy_test_control(struct mxc_hdmi *hdmi, unsigned char value)
{
	hdmi_phy_test_din(hdmi, value);
	hdmi_phy_test_enable(hdmi, 1);
	hdmi_phy_test_clock(hdmi, 1);
	hdmi_phy_test_clock(hdmi, 0);
	hdmi_phy_test_enable(hdmi, 0);

	return TRUE;
}

static int hdmi_phy_test_data(struct mxc_hdmi *hdmi, unsigned char value)
{
	hdmi_phy_test_din(hdmi, value);
	hdmi_phy_test_enable(hdmi, 0);
	hdmi_phy_test_clock(hdmi, 1);
	hdmi_phy_test_clock(hdmi, 0);

	return TRUE;
}

int hdmi_phy_wait_i2c_done(struct mxc_hdmi *hdmi, int msec)
{
	unsigned char val = 0;
	val = readb(hdmi->base + HDMI_IH_I2CMPHY_STAT0) & 0x3;
	while (val == 0) {
		udelay(1000);
		if (msec-- == 0)
			return FALSE;
		val = readb(hdmi->base + HDMI_IH_I2CMPHY_STAT0) & 0x3;
	}
	return TRUE;
}

int hdmi_phy_i2c_write(struct mxc_hdmi *hdmi, unsigned short data,
				unsigned char addr)
{
	writeb(0xFF, hdmi->base + HDMI_IH_I2CMPHY_STAT0);
	writeb(addr, hdmi->base + HDMI_PHY_I2CM_ADDRESS_ADDR);
	writeb((unsigned char)(data >> 8),
		hdmi->base + HDMI_PHY_I2CM_DATAO_1_ADDR);
	writeb((unsigned char)(data >> 0),
		hdmi->base + HDMI_PHY_I2CM_DATAO_0_ADDR);
	writeb(HDMI_PHY_I2CM_OPERATION_ADDR_WRITE,
		hdmi->base + HDMI_PHY_I2CM_OPERATION_ADDR);
	hdmi_phy_wait_i2c_done(hdmi, 1000);
	return TRUE;
}

unsigned short hdmi_phy_i2c_read(struct mxc_hdmi *hdmi, unsigned char addr)
{
	unsigned short data;
	unsigned char msb = 0, lsb = 0;
	writeb(0xFF, hdmi->base + HDMI_IH_I2CMPHY_STAT0);
	writeb(addr, hdmi->base + HDMI_PHY_I2CM_ADDRESS_ADDR);
	writeb(HDMI_PHY_I2CM_OPERATION_ADDR_READ,
		hdmi->base + HDMI_PHY_I2CM_OPERATION_ADDR);
	hdmi_phy_wait_i2c_done(hdmi, 1000);
	msb = readb(hdmi->base + HDMI_PHY_I2CM_DATAI_1_ADDR);
	lsb = readb(hdmi->base + HDMI_PHY_I2CM_DATAI_0_ADDR);
	data = (msb << 8) | lsb;
	return data;
}

int hdmi_phy_i2c_write_verify(struct mxc_hdmi *hdmi, unsigned short data,
				unsigned char addr)
{
	unsigned short val = 0;
	hdmi_phy_i2c_write(hdmi, data, addr);
	val = hdmi_phy_i2c_read(hdmi, addr);
	if (val != data)
		return FALSE;
	return TRUE;
}

int hdmi_phy_configure(struct mxc_hdmi *hdmi, unsigned char pRep,
				unsigned char cRes, int cscOn, int audioOn,
				int cecOn, int hdcpOn)
{
	unsigned short clk = 0, rep = 0;
	u8 val;

	/* color resolution 0 is 8 bit colour depth */
	if (cRes == 0)
		cRes = 8;

	if (pRep != 0)
		return FALSE;
	else if (cRes != 8 && cRes != 12)
		return FALSE;

	switch (hdmi->hdmi_data.video_mode.mPixelClock) {
	case 2520:
		clk = 0x93C1;
		rep = (cRes == 8) ? 0x6A4A : 0x6653;
		break;
	case 2700:
		clk = 0x96C1;
		rep = (cRes == 8) ? 0x6A4A : 0x6653;
		break;
	case 5400:
		clk = 0x8CC3;
		rep = (cRes == 8) ? 0x6A4A : 0x6653;
		break;
	case 7200:
		clk = 0x90C4;
		rep = (cRes == 8) ? 0x6A4A : 0x6654;
		break;
	case 7425:
		clk = 0x95C8;
		rep = (cRes == 8) ? 0x6A4A : 0x6654;
		break;
	case 10800:
		clk = 0x98C6;
		rep = (cRes == 8) ? 0x6A4A : 0x6653;
		break;
	case 14850:
		clk = 0x89C9;
		rep = (cRes == 8) ? 0x6A4A : 0x6654;
		break;
	default:
		return FALSE;
	}

	if (cscOn)
		val = HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_IN_PATH;
	else
		val = HDMI_MC_FLOWCTRL_FEED_THROUGH_OFF_CSC_BYPASS;
	writeb(val, hdmi->base + HDMI_MC_FLOWCTRL);

#if 0
	/* clock gate == 0 => turn on modules */
	val = hdcpOn ? HDMI_MC_CLKDIS_PIXELCLK_DISABLE_ENABLE :
		HDMI_MC_CLKDIS_PIXELCLK_DISABLE_DISABLE;
	val |= HDMI_MC_CLKDIS_PIXELCLK_DISABLE_ENABLE;
	val |= HDMI_MC_CLKDIS_TMDSCLK_DISABLE_ENABLE;
	val |= (pRep > 0) ? HDMI_MC_CLKDIS_PREPCLK_DISABLE_ENABLE :
		HDMI_MC_CLKDIS_PREPCLK_DISABLE_DISABLE;
	val |= cecOn ? HDMI_MC_CLKDIS_CECCLK_DISABLE_ENABLE :
		HDMI_MC_CLKDIS_CECCLK_DISABLE_DISABLE;
	val |= cscOn ? HDMI_MC_CLKDIS_CSCCLK_DISABLE_ENABLE :
		HDMI_MC_CLKDIS_CSCCLK_DISABLE_DISABLE;
	val |= audioOn ? HDMI_MC_CLKDIS_AUDCLK_DISABLE_ENABLE :
		HDMI_MC_CLKDIS_AUDCLK_DISABLE_DISABLE;
#else
	/* clock gate == 0 => turn on modules */
	val = hdcpOn ? HDMI_MC_CLKDIS_PIXELCLK_DISABLE_DISABLE :
		HDMI_MC_CLKDIS_PIXELCLK_DISABLE_ENABLE;
	val |= HDMI_MC_CLKDIS_PIXELCLK_DISABLE_ENABLE;
	val |= HDMI_MC_CLKDIS_TMDSCLK_DISABLE_ENABLE;
	val |= (pRep > 0) ? HDMI_MC_CLKDIS_PREPCLK_DISABLE_DISABLE :
		HDMI_MC_CLKDIS_PREPCLK_DISABLE_ENABLE;
	val |= cecOn ? HDMI_MC_CLKDIS_CECCLK_DISABLE_DISABLE :
		HDMI_MC_CLKDIS_CECCLK_DISABLE_ENABLE;
	val |= cscOn ? HDMI_MC_CLKDIS_CSCCLK_DISABLE_DISABLE :
		HDMI_MC_CLKDIS_CSCCLK_DISABLE_ENABLE;
	val |= audioOn ? HDMI_MC_CLKDIS_AUDCLK_DISABLE_DISABLE :
		HDMI_MC_CLKDIS_AUDCLK_DISABLE_ENABLE;
#endif
	writeb(val, hdmi->base + HDMI_MC_CLKDIS);

	/* gen2 tx power off */
	val = readb(hdmi->base + HDMI_PHY_CONF0);
	val &= ~HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	val |= HDMI_PHY_CONF0_GEN2_TXPWRON_POWER_OFF;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	/* gen2 pddq */
	val = readb(hdmi->base + HDMI_PHY_CONF0);
	val &= ~HDMI_PHY_CONF0_GEN2_PDDQ_MASK;
	val |= HDMI_PHY_CONF0_GEN2_PDDQ_ENABLE;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	/* PHY reset */
#if 0
	writeb(HDMI_MC_PHYRSTZ_ASSERT, hdmi->base + HDMI_MC_PHYRSTZ);
	writeb(HDMI_MC_PHYRSTZ_DEASSERT, hdmi->base + HDMI_MC_PHYRSTZ);
#else
	writeb(HDMI_MC_PHYRSTZ_DEASSERT, hdmi->base + HDMI_MC_PHYRSTZ);
	writeb(HDMI_MC_PHYRSTZ_ASSERT, hdmi->base + HDMI_MC_PHYRSTZ);
#endif

	writeb(HDMI_MC_HEACPHY_RST_ASSERT,
		hdmi->base + HDMI_MC_HEACPHY_RST);

	hdmi_phy_test_clear(hdmi, 1);
	writeb(HDMI_PHY_I2CM_SLAVE_ADDR_PHY_GEN2,
			hdmi->base + HDMI_PHY_I2CM_SLAVE_ADDR);
	hdmi_phy_test_clear(hdmi, 0);

	switch (hdmi->hdmi_data.video_mode.mPixelClock) {
	case 2520:
		switch (cRes) {
		case 8:
			/* PLL/MPLL Cfg */
			hdmi_phy_i2c_write(hdmi, 0x01e0, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);  /* CURRCTRL */
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);  /* GMPCTRL */
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);  /* PLLPHBYCTRL */
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			/* RESISTANCE TERM 133Ohm Cfg */
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);  /* TXTERM */
			/* PREEMP Cgf 0.00 */
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);  /* CKSYMTXCTRL */
			/* TX/CK LVL 10 */
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);  /* VLEVCTRL */
			/* REMOVE CLK TERM */
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);  /* CKCALCTRL */
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x21e1, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x41e2, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 2700:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x01e0, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x21e1, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x41e2, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 5400:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x0140, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x2141, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x4142, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 7200:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x0140, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x2141, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x40a2, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 7425:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x0140, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x20a1, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x0b5c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x40a2, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 10800:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x00a0, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x20a1, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x40a2, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	case 14850:
		switch (cRes) {
		case 8:
			hdmi_phy_i2c_write(hdmi, 0x00a0, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x06dc, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000a, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 10:
			hdmi_phy_i2c_write(hdmi, 0x2001, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x0b5c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000f, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x8009, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0210, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		case 12:
			hdmi_phy_i2c_write(hdmi, 0x4002, 0x06);
			hdmi_phy_i2c_write(hdmi, 0x091c, 0x10);
			hdmi_phy_i2c_write(hdmi, 0x000f, 0x15);
			hdmi_phy_i2c_write(hdmi, 0x0000, 0x13);
			hdmi_phy_i2c_write(hdmi, 0x0006, 0x17);
			hdmi_phy_i2c_write(hdmi, 0x0005, 0x19);
			hdmi_phy_i2c_write(hdmi, 0x800b, 0x09);
			hdmi_phy_i2c_write(hdmi, 0x0129, 0x0E);
			hdmi_phy_i2c_write(hdmi, 0x8000, 0x05);
			break;
		default:
			return FALSE;
		}
		break;
	default:
		return FALSE;
	}

	/* gen2 tx power on */
	val = readb(hdmi->base + HDMI_PHY_CONF0);
	val &= ~HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	val |= HDMI_PHY_CONF0_GEN2_TXPWRON_POWER_ON;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	val = readb(hdmi->base + HDMI_PHY_CONF0);
	val &= ~HDMI_PHY_CONF0_GEN2_PDDQ_MASK;
	val |= HDMI_PHY_CONF0_GEN2_PDDQ_DISABLE;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	udelay(1000);

	if ((readb(hdmi->base + HDMI_PHY_STAT0) & 0x01) == 0)
		return FALSE;

	return TRUE;
}

void hdmi_phy_init(struct mxc_hdmi *hdmi, unsigned char de)
{
	u8 val;

	/* configure the interrupt mask of source PHY. */
	writeb(0xFF, hdmi->base + HDMI_PHY_MASK0);

	val = readb(hdmi->base + HDMI_PHY_CONF0);
	/* set the DE polarity */
	val |= (de << HDMI_PHY_CONF0_SELDATAENPOL_OFFSET) &
		HDMI_PHY_CONF0_SELDATAENPOL_MASK;
	/* set the interface control to 0 */
	val |= (0 << HDMI_PHY_CONF0_SELDIPIF_OFFSET) &
		HDMI_PHY_CONF0_SELDIPIF_MASK;
	/* enable TMDS output */
	val |= (1 << HDMI_PHY_CONF0_ENTMDS_OFFSET) &
		HDMI_PHY_CONF0_ENTMDS_MASK;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	/* PHY power down disable (i.e., enable PHY) */
	val = readb(hdmi->base + HDMI_PHY_CONF0);
	val |= (1 << HDMI_PHY_CONF0_PDZ_OFFSET) &
		HDMI_PHY_CONF0_PDZ_MASK;
	writeb(val, hdmi->base + HDMI_PHY_CONF0);

	hdmi_phy_configure(hdmi, 0, 8, FALSE, FALSE, FALSE, FALSE);
}

void hdmi_tx_tmds_clock(unsigned short tmdsclk, int i2cfsmode)
{
}

void hdmi_tx_hdcp_config(struct mxc_hdmi *hdmi)
{
	u8 de, val;

	if (hdmi->hdmi_data.video_mode.mDataEnablePolarity)
		de = HDMI_A_VIDPOLCFG_DATAENPOL_ACTIVE_HIGH;
	else
		de = HDMI_A_VIDPOLCFG_DATAENPOL_ACTIVE_LOW;

	/* disable rx detect */
	val = readb(hdmi->base + HDMI_A_HDCPCFG0);
	val &= HDMI_A_HDCPCFG0_RXDETECT_MASK;
	val |= HDMI_A_HDCPCFG0_RXDETECT_DISABLE;
	writeb(val, hdmi->base + HDMI_A_HDCPCFG0);

	val = readb(hdmi->base + HDMI_A_VIDPOLCFG);
	val &= HDMI_A_VIDPOLCFG_DATAENPOL_MASK;
	val |= de;
	writeb(val, hdmi->base + HDMI_A_VIDPOLCFG);

	val = readb(hdmi->base + HDMI_A_HDCPCFG1);
	val &= HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_MASK;
	val |= HDMI_A_HDCPCFG1_ENCRYPTIONDISABLE_DISABLE;
	writeb(val, hdmi->base + HDMI_A_HDCPCFG1);

	hdmi_tx_tmds_clock(0, 0);
}

void hdmi_audio_mute(struct mxc_hdmi *hdmi, int en)
{
	u8 val;
	val = readb(hdmi->base + HDMI_FC_AUDSCONF);
	val &= HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_MASK;
	val |= ((en == TRUE) ? 0xF : 0) <<
		HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_OFFSET;
	writeb(val, hdmi->base + HDMI_FC_AUDSCONF);
}

void preamble_filter_set(struct mxc_hdmi *hdmi, unsigned char value,
				unsigned char channel)
{
	if (channel == 0)
		writeb(value, hdmi->base + HDMI_FC_CH0PREAM);
	else if (channel == 1)
		writeb(value, hdmi->base + HDMI_FC_CH1PREAM);
	else if (channel == 2)
		writeb(value, hdmi->base + HDMI_FC_CH2PREAM);
	else

	return;
}

/*!
 * this submodule is responsible for the video/audio data composition.
 */
void hdmi_av_composer(struct mxc_hdmi *hdmi)
{
	unsigned char i = 0;
	u8 val;

	hdmi_set_video_mode(&hdmi->hdmi_data.video_mode);

	/* Set up HDMI_FC_INVIDCONF */
	val = ((hdmi->hdmi_data.hdcp_enable == TRUE) ?
		HDMI_FC_INVIDCONF_HDCP_KEEPOUT_ACTIVE :
		HDMI_FC_INVIDCONF_HDCP_KEEPOUT_INACTIVE);
	val |= ((hdmi->hdmi_data.video_mode.mVSyncPolarity == TRUE) ?
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_HIGH :
		HDMI_FC_INVIDCONF_VSYNC_IN_POLARITY_ACTIVE_LOW);
	val |= ((hdmi->hdmi_data.video_mode.mHSyncPolarity == TRUE) ?
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_HIGH :
		HDMI_FC_INVIDCONF_HSYNC_IN_POLARITY_ACTIVE_LOW);
	val |= ((hdmi->hdmi_data.video_mode.mDataEnablePolarity == TRUE) ?
		HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_HIGH :
		HDMI_FC_INVIDCONF_DE_IN_POLARITY_ACTIVE_LOW);
	val |= ((hdmi->hdmi_data.video_mode.mHdmiDviSel == TRUE) ?
		HDMI_FC_INVIDCONF_DVI_MODEZ_HDMI_MODE :
		HDMI_FC_INVIDCONF_DVI_MODEZ_DVI_MODE);
	if (hdmi->hdmi_data.video_mode.mCode == 39)
		val |= HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH;
	else
		val |= ((hdmi->hdmi_data.video_mode.mInterlaced == TRUE) ?
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_HIGH :
			HDMI_FC_INVIDCONF_R_V_BLANK_IN_OSC_ACTIVE_LOW);
	val |= ((hdmi->hdmi_data.video_mode.mInterlaced == TRUE) ?
		HDMI_FC_INVIDCONF_IN_I_P_INTERLACED :
		HDMI_FC_INVIDCONF_IN_I_P_PROGRESSIVE);
	writeb(val, hdmi->base + HDMI_FC_INVIDCONF);

	/* Set up horizontal active pixel region width */
	writeb(hdmi->hdmi_data.video_mode.mHActive,
			hdmi->base + HDMI_FC_INHACTV0);
	writeb(hdmi->hdmi_data.video_mode.mHActive >> 8,
			hdmi->base + HDMI_FC_INHACTV1);

	/* Set up horizontal blanking pixel region width */
	writeb(hdmi->hdmi_data.video_mode.mHBlanking,
			hdmi->base + HDMI_FC_INHBLANK0);
	writeb(hdmi->hdmi_data.video_mode.mHBlanking >> 8,
			hdmi->base + HDMI_FC_INHBLANK1);

	/* Set up vertical blanking pixel region width */
	writeb(hdmi->hdmi_data.video_mode.mVActive,
			hdmi->base + HDMI_FC_INVACTV0);
	writeb(hdmi->hdmi_data.video_mode.mVActive >> 8,
			hdmi->base + HDMI_FC_INVACTV1);

	/* Set up vertical blanking pixel region width */
	writeb(hdmi->hdmi_data.video_mode.mVBlanking,
			hdmi->base + HDMI_FC_INVBLANK);

	/* Set up HSYNC active edge delay width (in pixel clks) */
	writeb(hdmi->hdmi_data.video_mode.mHSyncOffset,
			hdmi->base + HDMI_FC_HSYNCINDELAY0);
	writeb(hdmi->hdmi_data.video_mode.mHSyncOffset >> 8,
			hdmi->base + HDMI_FC_HSYNCINDELAY1);

	/* Set up HSYNC active pulse width (in pixel clks) */
	writeb(hdmi->hdmi_data.video_mode.mHSyncPulseWidth,
			hdmi->base + HDMI_FC_HSYNCINWIDTH0);
	writeb(hdmi->hdmi_data.video_mode.mHSyncPulseWidth >> 8,
			hdmi->base + HDMI_FC_HSYNCINWIDTH1);

	/* Set up vertical blanking pixel region width */
	writeb(hdmi->hdmi_data.video_mode.mVBlanking,
			hdmi->base + HDMI_FC_INVBLANK);

	/* Set up VSYNC active edge delay (in pixel clks) */
	writeb(hdmi->hdmi_data.video_mode.mVSyncOffset,
			hdmi->base + HDMI_FC_VSYNCINDELAY);

	/* Set up VSYNC active edge delay (in pixel clks) */
	writeb(hdmi->hdmi_data.video_mode.mVSyncPulseWidth,
			hdmi->base + HDMI_FC_VSYNCINWIDTH);

	/* control period minimum duration */
	writeb(12, hdmi->base + HDMI_FC_CTRLDUR);
	writeb(32, hdmi->base + HDMI_FC_EXCTRLDUR);
	writeb(1, hdmi->base + HDMI_FC_EXCTRLSPAC);

	for (i = 0; i < 3; i++)
		preamble_filter_set(hdmi, (i + 1) * 11, i);

	/* Set up input and output pixel repetition */
	val = (((hdmi->hdmi_data.video_mode.mPixelRepetitionInput + 1) <<
		HDMI_FC_PRCONF_INCOMING_PR_FACTOR_OFFSET) &
		HDMI_FC_PRCONF_INCOMING_PR_FACTOR_MASK) |
		((0 << HDMI_FC_PRCONF_INCOMING_PR_FACTOR_OFFSET) &
		HDMI_FC_PRCONF_INCOMING_PR_FACTOR_MASK);
	writeb(val, hdmi->base + HDMI_FC_PRCONF);

	/* AVI - underscan , IT601 */
	writeb(0x20, hdmi->base + HDMI_FC_AVICONF0);
	writeb(0x40, hdmi->base + HDMI_FC_AVICONF1);
}

static void mxc_hdmi_poweron(struct mxc_hdmi *hdmi)
{
	struct fsl_mxc_lcd_platform_data *plat = hdmi->pdev->dev.platform_data;

	dev_dbg(&hdmi->pdev->dev, "power on\n");

	/* Enable pins to HDMI */
	if (plat->enable_pins)
		plat->enable_pins();
}
static int mxc_hdmi_read_edid(struct mxc_hdmi *hdmi,
			struct fb_info *fbi)
{
	int ret;
	u8 edid_old[HDMI_EDID_LEN];

	/* save old edid */
	memcpy(edid_old, hdmi->edid, HDMI_EDID_LEN);

	/* edid reading */
	ret = mxc_edid_read(hdmi_i2c->adapter, hdmi_i2c->addr,
				hdmi->edid, &hdmi->edid_cfg, fbi);

	if (!memcmp(edid_old, hdmi->edid, HDMI_EDID_LEN))
		ret = -2;
	return ret;
}

static void mxc_hdmi_poweroff(struct mxc_hdmi *hdmi)
{
	struct fsl_mxc_lcd_platform_data *plat = hdmi->pdev->dev.platform_data;

	dev_dbg(&hdmi->pdev->dev, "power off\n");

	/* Disable pins to HDMI */
	if (plat->disable_pins)
		plat->disable_pins();
}

static int mxc_hdmi_fb_event(struct notifier_block *nb,
					unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	struct mxc_hdmi *hdmi = container_of(nb, struct mxc_hdmi, nb);

	if (strcmp(event->info->fix.id, hdmi->fbi->fix.id))
		return 0;

	switch (val) {
	case FB_EVENT_MODE_CHANGE:
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			mxc_hdmi_poweron(hdmi);
		else
			mxc_hdmi_poweroff(hdmi);
		break;
	}
	return 0;
}

static bool hdmi_is_mode_equal(struct fb_videomode *mode1,
	struct fb_videomode *mode2)
{
	if ((mode1->xres == mode2->xres) &&
		(mode1->yres == mode2->yres) &&
		((mode1->refresh >= mode2->refresh - 1) &&
		(mode1->refresh <= mode2->refresh + 1)) &&
		((mode1->flag & mode2->flag) == mode2->flag))
		return true;
	else
		return false;
}

static int mxc_hdmi_get_vmode(struct fb_info *fbi)
{
	int i;
	int vci_code = 0;
	struct fb_videomode m;

	fb_var_to_videomode(&m, &fbi->var);

	for (i = 1; i < NUM_CEA_VIDEO_MODES; i++)
		if (hdmi_is_mode_equal(&m, &hdmi_cea_video_modes[i-1])) {
			vci_code = i;
			break;
		}

	return vci_code;
}


static int mxc_hdmi_disp_init(struct mxc_dispdrv_entry *disp)
{
	int ret = 0;
	struct mxc_hdmi *hdmi = mxc_dispdrv_getdata(disp);
	struct mxc_dispdrv_setting *setting = mxc_dispdrv_getsetting(disp);
	struct fsl_mxc_lcd_platform_data *plat = hdmi->pdev->dev.platform_data;
	struct resource *res =
		platform_get_resource(hdmi->pdev, IORESOURCE_MEM, 0);
	int irq = platform_get_irq(hdmi->pdev, 0);
	bool found = false;
	int i;
	int mCode;

	if (!res || !plat || irq < 0)
		return -ENODEV;

	setting->dev_id = plat->ipu_id;
	setting->disp_id = plat->disp_id;
	setting->if_fmt = IPU_PIX_FMT_RGB24;

	hdmi->fbi = setting->fbi;

	if (plat->io_reg) {
		hdmi->io_reg = regulator_get(&hdmi->pdev->dev, plat->io_reg);
		if (!IS_ERR(hdmi->io_reg)) {
			regulator_set_voltage(hdmi->io_reg, 3300000, 3300000);
			regulator_enable(hdmi->io_reg);
		}
	}
	if (plat->analog_reg) {
		hdmi->analog_reg =
			regulator_get(&hdmi->pdev->dev, plat->analog_reg);
		if (!IS_ERR(hdmi->analog_reg)) {
			regulator_set_voltage(hdmi->analog_reg,
						1300000, 1300000);
			regulator_enable(hdmi->analog_reg);
		}
	}

	/* Claim HDMI pins */
	if (plat->get_pins)
		if (!plat->get_pins()) {
			ret = -EACCES;
			goto egetpins;
		}

	hdmi->hdmi_clk = clk_get(&hdmi->pdev->dev, "hdmi_isfr_clk");
	if (IS_ERR(hdmi->hdmi_clk)) {
		ret = PTR_ERR(hdmi->hdmi_clk);
		dev_err(&hdmi->pdev->dev,
			"Unable to get HDMI clk: %d\n", ret);
		goto egetclk;
	}

	ret = clk_enable(hdmi->hdmi_clk);
	if (ret < 0) {
		dev_err(&hdmi->pdev->dev,
			"Cannot enable HDMI clock: %d\n", ret);
		goto erate;
	}
	dev_dbg(&hdmi->pdev->dev, "Enabled HDMI clocks\n");

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&hdmi->pdev->dev))) {
		dev_err(&hdmi->pdev->dev,
			"HDMI register region already claimed\n");
		ret = -EBUSY;
		goto ereqreg;
	}

	hdmi->base = ioremap(res->start, resource_size(res));
	if (!hdmi->base) {
		dev_err(&hdmi->pdev->dev,
			"HDMI register iomap failed\n");
		ret = -ENOMEM;
		goto emap;
	}

	/* Product and revision IDs */
	dev_info(&hdmi->pdev->dev,
		"Detected HDMI controller 0x%x:0x%x:0x%x:0x%x\n",
		readb(hdmi->base + HDMI_DESIGN_ID),
		readb(hdmi->base + HDMI_REVISION_ID),
		readb(hdmi->base + HDMI_PRODUCT_ID0),
		readb(hdmi->base + HDMI_PRODUCT_ID1));

	ret = request_irq(irq, mxc_hdmi_hotplug, 0,
			  dev_name(&hdmi->pdev->dev), hdmi);
	if (ret < 0) {
		dev_err(&hdmi->pdev->dev, "Unable to request irq: %d\n", ret);
		goto ereqirq;
	}

	/* try to read edid */
	ret = mxc_hdmi_read_edid(hdmi, hdmi->fbi);
	if (ret < 0)
		dev_warn(&hdmi->pdev->dev, "Can not read edid\n");
	else {
		INIT_LIST_HEAD(&hdmi->fbi->modelist);
		if (hdmi->fbi->monspecs.modedb_len > 0) {
			int i;
			const struct fb_videomode *mode;
			struct fb_videomode m;

			for (i = 0; i < hdmi->fbi->monspecs.modedb_len; i++) {
				/*FIXME now we do not support interlaced mode */
				if (!(hdmi->fbi->monspecs.modedb[i].vmode
						& FB_VMODE_INTERLACED)) {
					fb_add_videomode(
						&hdmi->fbi->monspecs.modedb[i],
						&hdmi->fbi->modelist);
				}
			}

			fb_find_mode(&hdmi->fbi->var, hdmi->fbi,
				setting->dft_mode_str, NULL, 0, NULL,
				setting->default_bpp);

			fb_var_to_videomode(&m, &hdmi->fbi->var);
			mode = fb_find_nearest_mode(&m,
					&hdmi->fbi->modelist);
			fb_videomode_to_var(&hdmi->fbi->var, mode);
			found = 1;
		}
	}

	if (!found) {
		ret = fb_find_mode(&hdmi->fbi->var, hdmi->fbi,
					setting->dft_mode_str, NULL, 0, NULL,
					setting->default_bpp);
		if (!ret) {
			ret = -EINVAL;
			goto efindmode;
		}
	}

	hdmi->nb.notifier_call = mxc_hdmi_fb_event;
	ret = fb_register_client(&hdmi->nb);
	if (ret < 0)
		goto efbclient;

	mCode = mxc_hdmi_get_vmode(hdmi->fbi);
	if (mCode == 0)
		mCode = DEFAULT_VIDEO_MODE;

	memset(&hdmi->hdmi_data, 0, sizeof(struct hdmi_data_info));

	hdmi->hdmi_data.enc_in_format = eRGB;
	hdmi->hdmi_data.enc_out_format = eRGB;
	hdmi->hdmi_data.enc_color_depth = 8;
	hdmi->hdmi_data.colorimetry = eITU601;
	hdmi->hdmi_data.pix_repet_factor = 0;
	hdmi->hdmi_data.hdcp_enable = 0;
	hdmi->hdmi_data.video_mode.mCode = mCode;
	hdmi->hdmi_data.video_mode.mHdmiDviSel = TRUE;
	hdmi->hdmi_data.video_mode.mRVBlankInOSC = FALSE;
	hdmi->hdmi_data.video_mode.mRefreshRate = 60000;
	hdmi->hdmi_data.video_mode.mDataEnablePolarity = TRUE;

	hdmi_video_force_output(hdmi, TRUE);
	hdmi_av_composer(hdmi);
	hdmi_video_packetize(hdmi);
	hdmi_video_csc(hdmi);
	hdmi_video_sample(hdmi);
	hdmi_audio_mute(hdmi, TRUE);
	hdmi_tx_hdcp_config(hdmi);
	hdmi_phy_init(hdmi, TRUE);
	hdmi_video_force_output(hdmi, FALSE);

	return ret;

efbclient:
	free_irq(irq, hdmi);
efindmode:
ereqirq:
	iounmap(hdmi->base);
emap:
	release_mem_region(res->start, resource_size(res));
ereqreg:
	clk_disable(hdmi->hdmi_clk);
erate:
	clk_put(hdmi->hdmi_clk);
egetclk:
egetpins:
	return ret;
}

static void mxc_hdmi_disp_deinit(struct mxc_dispdrv_entry *disp)
{
	struct mxc_hdmi *hdmi = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_lcd_platform_data *plat = hdmi->pdev->dev.platform_data;

	fb_unregister_client(&hdmi->nb);

	mxc_hdmi_poweroff(hdmi);

	/* Release HDMI pins */
	if (plat->put_pins)
		plat->put_pins();

	platform_device_unregister(hdmi->pdev);

	kfree(hdmi);
}

static struct mxc_dispdrv_driver mxc_hdmi_drv = {
	.name	= DISPDRV_HDMI,
	.init	= mxc_hdmi_disp_init,
	.deinit	= mxc_hdmi_disp_deinit,
};

static int __init mxc_hdmi_probe(struct platform_device *pdev)
{
	struct mxc_hdmi *hdmi;
	int ret = 0;

	/* Check that I2C driver is loaded and available */
	if (!hdmi_i2c)
		return -ENODEV;

	hdmi = kzalloc(sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi) {
		dev_err(&pdev->dev, "Cannot allocate device data\n");
		ret = -ENOMEM;
		goto ealloc;
	}

	hdmi->pdev = pdev;

	hdmi->disp_mxc_hdmi = mxc_dispdrv_register(&mxc_hdmi_drv);
	if (IS_ERR(hdmi->disp_mxc_hdmi)) {
		dev_err(&pdev->dev, "Failed to register dispdrv - 0x%x\n",
			(int)hdmi->disp_mxc_hdmi);
		ret = (int)hdmi->disp_mxc_hdmi;
		goto edispdrv;
	}
	mxc_dispdrv_setdata(hdmi->disp_mxc_hdmi, hdmi);

	platform_set_drvdata(pdev, hdmi);

	return 0;
edispdrv:
	kfree(hdmi);
ealloc:
	return ret;
}

static int  mxc_hdmi_remove(struct platform_device *pdev)
{
	struct fsl_mxc_lcd_platform_data *pdata = pdev->dev.platform_data;
	struct mxc_hdmi *hdmi = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irq = platform_get_irq(pdev, 0);

	fb_unregister_client(&hdmi->nb);

	/* No new work will be scheduled, wait for running ISR */
	free_irq(irq, hdmi);
	clk_disable(hdmi->hdmi_clk);
	clk_put(hdmi->hdmi_clk);
	iounmap(hdmi->base);
	release_mem_region(res->start, resource_size(res));
	kfree(hdmi);

	return 0;
}

static struct platform_driver mxc_hdmi_driver = {
	.probe = mxc_hdmi_probe,
	.remove = mxc_hdmi_remove,
	.driver = {
		   .name = "mxc_hdmi",
		   .owner = THIS_MODULE,
	},
};

static int __init mxc_hdmi_init(void)
{
	return platform_driver_register(&mxc_hdmi_driver);
}
module_init(mxc_hdmi_init);

static void __exit mxc_hdmi_exit(void)
{
	platform_driver_unregister(&mxc_hdmi_driver);
}
module_exit(mxc_hdmi_exit);


static int __devinit mxc_hdmi_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	hdmi_i2c = client;

	return 0;
}

static int __devexit mxc_hdmi_i2c_remove(struct i2c_client *client)
{
	hdmi_i2c = NULL;
	return 0;
}

static const struct i2c_device_id mxc_hdmi_i2c_id[] = {
	{ "mxc_hdmi_i2c", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mxc_hdmi_i2c_id);

static struct i2c_driver mxc_hdmi_i2c_driver = {
	.driver = {
		   .name = "mxc_hdmi_i2c",
		   },
	.probe = mxc_hdmi_i2c_probe,
	.remove = mxc_hdmi_i2c_remove,
	.id_table = mxc_hdmi_i2c_id,
};

static int __init mxc_hdmi_i2c_init(void)
{
	return i2c_add_driver(&mxc_hdmi_i2c_driver);
}

static void __exit mxc_hdmi_i2c_exit(void)
{
	i2c_del_driver(&mxc_hdmi_i2c_driver);
}

module_init(mxc_hdmi_i2c_init);
module_exit(mxc_hdmi_i2c_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
