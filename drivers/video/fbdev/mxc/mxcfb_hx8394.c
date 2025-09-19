/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2024 Emcraft Systems
 * Author(s): Vladimir Skvortsov <vskvortsov@emcraft.com>
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mipi_dsi.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

/* User Define command set */
#define UD_SETADDRESSMODE	0x36 /* Set address mode */
#define UD_SETSEQUENCE		0xB0 /* Set sequence */
#define UD_SETPOWER		0xB1 /* Set power */
#define UD_SETDISP		0xB2 /* Set display related register */
#define UD_SETCYC		0xB4 /* Set display waveform cycles */
#define UD_SETVCOM		0xB6 /* Set VCOM voltage */
#define UD_SETTE		0xB7 /* Set internal TE function */
#define UD_SETSENSOR		0xB8 /* Set temperature sensor */
#define UD_SETEXTC		0xB9 /* Set extension command */
#define UD_SETMIPI		0xBA /* Set MIPI control */
#define UD_SETOTP		0xBB /* Set OTP */
#define UD_SETREGBANK		0xBD /* Set register bank */
#define UD_SETDGCLUT		0xC1 /* Set DGC LUT */
#define UD_SETID		0xC3 /* Set ID */
#define UD_SETDDB		0xC4 /* Set DDB */
#define UD_SETCABC		0xC9 /* Set CABC control */
#define UD_SETCABCGAIN		0xCA
#define UD_SETPANEL		0xCC
#define UD_SETOFFSET		0xD2
#define UD_SETGIP0		0xD3 /* Set GIP Option0 */
#define UD_SETGIP1		0xD5 /* Set GIP Option1 */
#define UD_SETGIP2		0xD6 /* Set GIP Option2 */
#define UD_SETGPO		0xD9
#define UD_SETSCALING		0xDD
#define UD_SETIDLE		0xDF
#define UD_SETGAMMA		0xE0 /* Set gamma curve related setting */
#define UD_SETCHEMODE_DYN	0xE4
#define UD_SETCHE		0xE5
#define UD_SETCESEL		0xE6 /* Enable color enhance */
#define UD_SET_SP_CMD		0xE9
#define UD_SETREADINDEX		0xFE /* Set SPI Read Index */
#define UD_GETSPIREAD		0xFF /* SPI Read Command Data */

struct hx8394_cmd
{
    const u8 *cmd;
    u8 cmdLen;
};

static const struct hx8394_cmd hx8394_cmds[] = {
	{(const u8[]){UD_SETEXTC, 0xFF, 0x83, 0x94}, 4},

	{(const u8[]){UD_SETMIPI, 0x61, 0x03, 0x68, 0x6B, 0xB2, 0xC0}, 7},

	{(const u8[]){UD_SETADDRESSMODE, 0x02}, 2},

	{(const u8[]){UD_SETPOWER, 0x48, 0x12, 0x72, 0x09, 0x32, 0x54, 0x71, 0x71, 0x57, 0x47}, 11},

	{(const u8[]){UD_SETDISP, 0x00, 0x80, 0x64, 0x15, 0x0E, 0x11}, 7},

	{(const u8[]){UD_SETCYC, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01, 0x0C, 0x86,
		      0x75, 0x00, 0x3F, 0x73, 0x74, 0x73, 0x74, 0x73, 0x74, 0x01,
		      0x0C, 0x86}, 22},

	{(const u8[]){UD_SETGIP0, 0x00, 0x00, 0x07, 0x07, 0x40, 0x07, 0x0C, 0x00, 0x08,
		      0x10, 0x08, 0x00, 0x08, 0x54, 0x15, 0x0A, 0x05, 0x0A, 0x02,
		      0x15, 0x06, 0x05, 0x06, 0x47, 0x44, 0x0A, 0x0A, 0x4B, 0x10,
		      0x07, 0x07, 0x0C, 0x40}, 34},

	{(const u8[]){UD_SETGIP1, 0x1C, 0x1C, 0x1D, 0x1D, 0x00, 0x01, 0x02, 0x03, 0x04,
		      0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x24, 0x25, 0x18,
		      0x18, 0x26, 0x27, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x20,
		      0x21, 0x18, 0x18, 0x18, 0x18}, 45},

	{(const u8[]){UD_SETGIP2, 0x1C, 0x1C, 0x1D, 0x1D, 0x07, 0x06, 0x05, 0x04, 0x03,
		      0x02, 0x01, 0x00, 0x0B, 0x0A, 0x09, 0x08, 0x21, 0x20, 0x18,
		      0x18, 0x27, 0x26, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
		      0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x25,
		      0x24, 0x18, 0x18, 0x18, 0x18}, 45},

	{(const u8[]){UD_SETVCOM, 0x92, 0x92}, 3},

	{(const u8[]){UD_SETGAMMA, 0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47,
		      0x56, 0x65, 0x66, 0x6E, 0x82, 0x88, 0x8B, 0x9A, 0x9D, 0x98,
		      0xA8, 0xB9, 0x5D, 0x5C, 0x61, 0x66, 0x6A, 0x6F, 0x7F, 0x7F,
		      0x00, 0x0A, 0x15, 0x1B, 0x1E, 0x21, 0x24, 0x22, 0x47, 0x56,
		      0x65, 0x65, 0x6E, 0x81, 0x87, 0x8B, 0x98, 0x9D, 0x99, 0xA8,
		      0xBA, 0x5D, 0x5D, 0x62, 0x67, 0x6B, 0x72, 0x7F, 0x7F}, 59},

	{(const u8[]){0xC0, 0x1F, 0x31}, 3},
	{(const u8[]){UD_SETPANEL, 0x03}, 2},
	{(const u8[]){0xD4, 0x02}, 2},
	{(const u8[]){UD_SETREGBANK, 0x02}, 2},

	{(const u8[]){0xD8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		      0xFF, 0xFF, 0xFF}, 13},

	{(const u8[]){UD_SETREGBANK, 0x00}, 2},
	{(const u8[]){UD_SETREGBANK, 0x01}, 2},
	{(const u8[]){UD_SETPOWER, 0x00}, 2},
	{(const u8[]){UD_SETREGBANK, 0x00}, 2},

	{(const u8[]){0xBF, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01}, 8},
	{(const u8[]){0xC6, 0xED}, 2},
	{(const u8[]){0x35, 0x00}, 2},
};

static struct fb_videomode rk_lcd_modedb[] = {
	/* 720 x 1280 */
	{
		"hx8394", 60, 720, 1280, KHZ2PICOS(66000),
		10, 52,
		7, 16,
		52, 16,
		0x0,
		FB_VMODE_NONINTERLACED,
		0,
	}
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = 2,
	.max_phy_clk    = 800,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_hx8394_get_lcd_videomode(struct fb_videomode **mode, int *size,
				     struct mipi_lcd_config **data)
{
	*mode = &rk_lcd_modedb[0];
	*size = ARRAY_SIZE(rk_lcd_modedb);
	*data = &lcd_config;
}

static int hx8394_generic_write(struct mipi_dsi_info *mipi_dsi, const u8 *buf, u32 n)
{
	int err;

	if (n > 2)
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
						   MIPI_DSI_GENERIC_LONG_WRITE, (u32*)buf, n);
	else if (n == 2)
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
						   MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, (u32*)buf, 0);
	else if (n == 1)
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
						   MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM, (u32*)buf, 0);
	else if (n == 0)
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
						   MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM, (u32*)buf, 0);
	return err;
}

int mipid_hx8394_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];
	int err;
	int i;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD HX8394 setup.\n");

        for (i = 0; i < ARRAY_SIZE(hx8394_cmds); i++)
        {
		err = hx8394_generic_write(mipi_dsi, hx8394_cmds[i].cmd, (u32)hx8394_cmds[i].cmdLen);
		if (err)
			goto err_out;
        }

	/* exit sleep mode and set display on */
	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
	buf[1] = 0;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE,
					   (u32*)buf, 0);
	if (err) {
		goto err_out;
	}
	/* To allow time for the supply voltages
	 * and clock circuits to stabilize.
	 */
	msleep(5);
	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE,
					   (u32*)buf, 0);
	if (err) {
		goto err_out;
	}

 err_out:
	return err;
}
