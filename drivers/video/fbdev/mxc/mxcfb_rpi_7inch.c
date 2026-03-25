/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2024-2026 Emcraft Systems
 * Author(s): Vladimir Skvortsov <vskvortsov@emcraft.com>
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mipi_dsi.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define RPI_DSI_DRIVER_NAME "rpi-ts-dsi"

/* PPI layer registers */
#define PPI_STARTPPI		0x0104 /* START control bit */
#define PPI_LPTXTIMECNT		0x0114 /* LPTX timing signal */
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D0S_CLRSIPOCOUNT	0x0164 /* Assertion timer for Lane 0 */
#define PPI_D1S_CLRSIPOCOUNT	0x0168 /* Assertion timer for Lane 1 */
#define PPI_START_FUNCTION	1

/* DSI layer registers */
#define DSI_STARTDSI		0x0204 /* START control bit of DSI-TX */
#define DSI_LANEENABLE		0x0210 /* Enables each lane */
#define DSI_RX_START		1

/* LCDC/DPI Host Registers, based on guesswork that this matches TC358764 */
#define LCDCTRL			0x0420 /* Video Path Control */
#define LCDCTRL_MSF		BIT(0) /* Magic square in RGB666 */
#define LCDCTRL_VTGEN		BIT(4)/* Use chip clock for timing */
#define LCDCTRL_UNK6		BIT(6) /* Unknown */
#define LCDCTRL_EVTMODE		BIT(5) /* Event mode */
#define LCDCTRL_RGB888		BIT(8) /* RGB888 mode */
#define LCDCTRL_HSPOL		BIT(17) /* Polarity of HSYNC signal */
#define LCDCTRL_DEPOL		BIT(18) /* Polarity of DE signal */
#define LCDCTRL_VSPOL		BIT(19) /* Polarity of VSYNC signal */
#define LCDCTRL_VSDELAY(v)	(((v) & 0xfff) << 20) /* VSYNC delay */

/* First parameter is in the 16bits, second is in the top 16bits */
#define LCD_HS_HBP		0x0424
#define LCD_HDISP_HFP		0x0428
#define LCD_VS_VBP		0x042c
#define LCD_VDISP_VFP		0x0430

/* SPI Master Registers */
#define SPICMR			0x0450
#define SPITCR			0x0454

/* System Controller Registers */
#define SYSCTRL			0x0464

/* System registers */
#define LPX_PERIOD		3

/* Lane enable PPI and DSI register bits */
#define LANEENABLE_CLEN		BIT(0)
#define LANEENABLE_L0EN		BIT(1)
#define LANEENABLE_L1EN		BIT(2)

struct rpi_touchscreen {
	struct device *dev;
	struct mipi_dsi_info *dsi;
};

#if 1
/* modern RPi from official RPi kernel
 * drivers/gpu/drm/panel/panel-simple.c, raspberrypi_7inch_mode
 * fine-tuned form MaaxRT */
static struct fb_videomode lcd_mode[] = {
	/* 800 x 480 */
	{
		"rpi", 60, 800, 480, KHZ2PICOS(30000000 / 1000),
		45, 136,
		23, 8,
		6, 6,
		/* Both HSYNC and VSYNC active low */
		/*FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT*/ 0,
		FB_VMODE_NONINTERLACED,
		0,
	}
};
#endif

#if 0
/* drivers/gpu/drm/panel/panel-raspberrypi-touchscreen.c */
static struct fb_videomode lcd_mode[] = {
	/* 800 x 480 */
	{
		"rpi", 60, 800, 480, KHZ2PICOS(25979400 / 1000),
		47, 1,
		21, 7,
		2, 2,
		/* Both HSYNC and VSYNC active low */
		/*FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT*/ 0,
		FB_VMODE_NONINTERLACED,
		0,
	}
};
#endif

#if 0
/* imx8 ports,
 * drivers/gpu/drm/panel/panel-simple.c, powertip_ph800480t013_idf02 */
static struct fb_videomode lcd_mode[] = {
	/* 800 x 480 */
	{
		"rpi", 60, 800, 480, KHZ2PICOS(24750000 / 1000),
		44, 54,
		22, 49,
		2, 2,
		/* Both HSYNC and VSYNC active low */
		/*FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT*/ 0,
		FB_VMODE_NONINTERLACED,
		0,
	}
};
#endif


static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = 1,
	.max_phy_clk    = 800,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_rpi_get_lcd_videomode(struct fb_videomode **mode, int *size,
				     struct mipi_lcd_config **data)
{
	*mode = &lcd_mode[0];
	*size = ARRAY_SIZE(lcd_mode);
	*data = &lcd_config;
}

static int rpi_touchscreen_write(struct rpi_touchscreen *ts, u16 reg, u32 val)
{
	u8 data[6];

	data[0] = reg;
	data[1] = reg >> 8;
	data[2] = val;
	data[3] = val >> 8;
	data[4] = val >> 16;
	data[5] = val >> 24;

	if (ts->dsi->mipi_dsi_pkt_write(ts->dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					(u32 *)data, sizeof(data))) {
		dev_err(&ts->dsi->pdev->dev, "DSI write failure!\n");
	}

	return 0;
}

#define rpi_touchscreen_dcs_write(ts, ret, cmd, param...) \
	do {\
		static const u8 data[] = {cmd, param};\
		ret = local_dcs_write(ts, data, ARRAY_SIZE(data));\
	} while (0)

static int local_dcs_write(struct rpi_touchscreen *ts, const u8 *data,
	unsigned int len)
{
	int rc;
	u8 dt;

	switch (len) {
	case 0:
		dt = MIPI_DSI_DCS_SHORT_WRITE;
		break;
	case 1:
		dt = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;
	default:
		dt = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	rc = ts->dsi->mipi_dsi_pkt_write(ts->dsi, dt, (u32 *)data, len);
	if (rc)
		dev_err(&ts->dsi->pdev->dev, "DSI write failure!\n");

	return rc;
}

static int rpi_touchscreen_enable(struct rpi_touchscreen *ts)
{
	u32 lcdctrl;
	int ret;

	rpi_touchscreen_write(ts, DSI_LANEENABLE,
		       LANEENABLE_L0EN | LANEENABLE_CLEN);
	rpi_touchscreen_write(ts, PPI_D0S_CLRSIPOCOUNT, 5);
	rpi_touchscreen_write(ts, PPI_D1S_CLRSIPOCOUNT, 5);
	rpi_touchscreen_write(ts, PPI_D0S_ATMR, 0);
	rpi_touchscreen_write(ts, PPI_D1S_ATMR, 0);
	rpi_touchscreen_write(ts, PPI_LPTXTIMECNT, LPX_PERIOD);

	rpi_touchscreen_write(ts, SPICMR, 0x00);

	lcdctrl = LCDCTRL_VSDELAY(1) | LCDCTRL_RGB888 |
		  LCDCTRL_UNK6 | LCDCTRL_VTGEN;

	if (!(lcd_mode->sync & FB_SYNC_HOR_HIGH_ACT))
		lcdctrl |= LCDCTRL_HSPOL;

	if (!(lcd_mode->sync & FB_SYNC_VERT_HIGH_ACT))
		lcdctrl |= LCDCTRL_VSPOL;

	rpi_touchscreen_write(ts, LCDCTRL, lcdctrl);

	rpi_touchscreen_write(ts, SYSCTRL, 0x040f);

	rpi_touchscreen_write(ts, LCD_HS_HBP, (lcd_mode->hsync_len) |
		       ((lcd_mode->left_margin) << 16));
	rpi_touchscreen_write(ts, LCD_HDISP_HFP, (lcd_mode->xres) |
		       ((lcd_mode->right_margin) << 16));
	rpi_touchscreen_write(ts, LCD_VS_VBP, (lcd_mode->vsync_len) |
		       ((lcd_mode->upper_margin) << 16));
	rpi_touchscreen_write(ts, LCD_VDISP_VFP, (lcd_mode->yres) |
		       ((lcd_mode->lower_margin) << 16));
	msleep(100);

	rpi_touchscreen_write(ts, PPI_STARTPPI, PPI_START_FUNCTION);
	rpi_touchscreen_write(ts, DSI_STARTDSI, DSI_RX_START);

	msleep(100);
	rpi_touchscreen_dcs_write(ts, ret, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(100);

	rpi_touchscreen_dcs_write(ts, ret, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);

	return 0;
}

int mipid_rpi_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	int err = 0;
	struct rpi_touchscreen ts_dev = {0};


	dev_info(&mipi_dsi->pdev->dev, "MIPI DSI LCD RPI setup.\n");

	ts_dev.dev = &mipi_dsi->pdev->dev;
	ts_dev.dsi = mipi_dsi;

	err = rpi_touchscreen_enable(&ts_dev);

	return err;
}
