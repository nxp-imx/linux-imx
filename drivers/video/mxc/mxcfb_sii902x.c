/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_sii902x.c
 *
 * @brief MXC Frame buffer driver for SII902x
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mxc_edid.h>
#include "mxc_dispdrv.h"

#define DISPDRV_SII	"sii902x_hdmi"

#define TPI_PIX_CLK_LSB                 (0x00)
#define TPI_PIX_CLK_MSB                 (0x01)
#define TPI_VERT_FREQ_LSB               (0x02)
#define TPI_VERT_FREQ_MSB               (0x03)
#define TPI_TOTAL_PIX_LSB               (0x04)
#define TPI_TOTAL_PIX_MSB         	(0x05)
#define TPI_TOTAL_LINES_LSB		(0x06)
#define TPI_TOTAL_LINES_MSB		(0x07)
#define TPI_PIX_REPETITION              (0x08)
#define TPI_INPUT_FORMAT_REG            (0x09)
#define TPI_OUTPUT_FORMAT_REG           (0x0A)

#define TPI_AVI_BYTE_0                  (0x0C)
#define TPI_AVI_BYTE_1                  (0x0D)
#define TPI_AVI_BYTE_2                  (0x0E)
#define TPI_AVI_BYTE_3                  (0x0F)
#define TPI_AVI_BYTE_4                  (0x10)
#define TPI_AVI_BYTE_5                  (0x11)

#define TPI_END_TOP_BAR_LSB             (0x12)
#define TPI_END_TOP_BAR_MSB             (0x13)

#define TPI_START_BTM_BAR_LSB           (0x14)
#define TPI_START_BTM_BAR_MSB           (0x15)

#define TPI_END_LEFT_BAR_LSB            (0x16)
#define TPI_END_LEFT_BAR_MSB            (0x17)

#define TPI_END_RIGHT_BAR_LSB           (0x18)
#define TPI_END_RIGHT_BAR_MSB           (0x19)

#define TPI_SYSTEM_CONTROL_DATA_REG	(0x1A)
#define TPI_DEVICE_ID			(0x1B)
#define TPI_DEVICE_REV_ID		(0x1C)
#define TPI_RESERVED2			(0x1D)
#define TPI_DEVICE_POWER_STATE_CTRL_REG	(0x1E)

#define TPI_I2S_EN                      (0x1F)
#define TPI_I2S_IN_CFG                  (0x20)
#define TPI_I2S_CHST_0                  (0x21)
#define TPI_I2S_CHST_1                  (0x22)
#define TPI_I2S_CHST_2                  (0x23)
#define TPI_I2S_CHST_3                  (0x24)
#define TPI_I2S_CHST_4                  (0x25)

#define TPI_AUDIO_HANDLING              (0x25)
#define TPI_AUDIO_INTERFACE_REG         (0x26)
#define TPI_AUDIO_SAMPLE_CTRL           (0x27)

#define TPI_INTERRUPT_ENABLE_REG	(0x3C)
#define TPI_INTERRUPT_STATUS_REG	(0x3D)

#define TPI_INTERNAL_PAGE_REG		0xBC
#define TPI_INDEXED_OFFSET_REG		0xBD
#define TPI_INDEXED_VALUE_REG		0xBE

#define MISC_INFO_FRAMES_CTRL           (0xBF)
#define MISC_INFO_FRAMES_TYPE           (0xC0)
#define EN_AND_RPT_AUDIO                0xC2
#define DISABLE_AUDIO                   0x02

#define TPI_ENABLE			(0xC7)

#define INDEXED_PAGE_0		0x01
#define INDEXED_PAGE_1		0x02
#define INDEXED_PAGE_2		0x03

#define HOT_PLUG_EVENT          0x01
#define RX_SENSE_EVENT          0x02
#define HOT_PLUG_STATE          0x04
#define RX_SENSE_STATE          0x08

#define OUTPUT_MODE_MASK	(0x01)
#define OUTPUT_MODE_DVI		(0x00)
#define OUTPUT_MODE_HDMI	(0x01)

#define LINK_INTEGRITY_MODE_MASK        0x40
#define LINK_INTEGRITY_STATIC           (0x00)
#define LINK_INTEGRITY_DYNAMIC          (0x40)

#define TMDS_OUTPUT_CONTROL_MASK	0x10
#define TMDS_OUTPUT_CONTROL_ACTIVE      (0x00)
#define TMDS_OUTPUT_CONTROL_POWER_DOWN  (0x10)

#define AV_MUTE_MASK                    0x08
#define AV_MUTE_NORMAL                  (0x00)
#define AV_MUTE_MUTED                   (0x08)

#define TX_POWER_STATE_MASK             0x3
#define TX_POWER_STATE_D0               (0x00)
#define TX_POWER_STATE_D1               (0x01)
#define TX_POWER_STATE_D2               (0x02)
#define TX_POWER_STATE_D3               (0x03)

#define AUDIO_MUTE_MASK                  0x10
#define AUDIO_MUTE_NORMAL                (0x00)
#define AUDIO_MUTE_MUTED                 (0x10)

#define AUDIO_SEL_MASK                   0xC0
#define AUD_IF_SPDIF                     0x40
#define AUD_IF_I2S                       0x80
#define AUD_IF_DSD                       0xC0
#define AUD_IF_HBR                       0x04

#define REFER_TO_STREAM_HDR		0x00

#define AUD_PASS_BASIC      0x00
#define AUD_PASS_ALL        0x01
#define AUD_DOWN_SAMPLE     0x02
#define AUD_DO_NOT_CHECK    0x03

#define BITS_IN_RGB         0x00
#define BITS_IN_YCBCR444    0x01
#define BITS_IN_YCBCR422    0x02

#define BITS_IN_AUTO_RANGE  0x00
#define BITS_IN_FULL_RANGE  0x04
#define BITS_IN_LTD_RANGE   0x08

#define BIT_EN_DITHER_10_8  0x40
#define BIT_EXTENDED_MODE   0x80

#define SII_EDID_LEN		512
#define SIZE_AVI_INFOFRAME	0x0E
#define SIZE_AUDIO_INFOFRAME    0x0F

#define _4_To_3                 0x10
#define _16_To_9                0x20
#define SAME_AS_AR              0x08

struct sii902x_data {
	struct platform_device *pdev;
	struct i2c_client *client;
	struct mxc_dispdrv_handle *disp_hdmi;
	struct regulator *io_reg;
	struct regulator *analog_reg;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	bool cable_plugin;
	bool rx_powerup;
	bool need_mode_change;
	u8 edid[SII_EDID_LEN];
	struct notifier_block nb;

	u8 power_state;
	u8 tpivmode[3];
	u8 pixrep;

	/* SII902x video setting:
	 * 1. hdmi video fmt:
	 * 	0 = CEA-861 VIC; 1 = HDMI_VIC; 2 = 3D
	 * 2. vic: video mode index
	 * 3. aspect ratio:
	 * 	4x3 or 16x9
	 * 4. color space:
	 *	0 = RGB; 1 = YCbCr4:4:4; 2 = YCbCr4:2:2_16bits;
	 *	3 = YCbCr4:2:2_8bits;4 = xvYCC4:4:4
	 * 5. color depth:
	 *	0 = 8bits; 1 = 10bits; 2 = 12bits; 3 = 16bits
	 * 6. colorimetry:
	 *	0 = 601; 1 = 709
	 * 7. syncmode:
	 * 	0 = external HS/VS/DE; 1 = external HS/VS and internal DE;
	 *	2 = embedded sync
	 */
#define VMD_HDMIFORMAT_CEA_VIC                  0x00
#define VMD_HDMIFORMAT_HDMI_VIC                 0x01
#define VMD_HDMIFORMAT_3D                       0x02
#define VMD_HDMIFORMAT_PC                       0x03
	u8 hdmi_vid_fmt;
	u8 vic;
#define VMD_ASPECT_RATIO_4x3                    0x01
#define VMD_ASPECT_RATIO_16x9                   0x02
	u8 aspect_ratio;
#define RGB			0
#define YCBCR444		1
#define YCBCR422_16BITS		2
#define YCBCR422_8BITS		3
#define XVYCC444		4
	u8 icolor_space;
	u8 ocolor_space;
#define VMD_COLOR_DEPTH_8BIT                    0x00
#define VMD_COLOR_DEPTH_10BIT                   0x01
#define VMD_COLOR_DEPTH_12BIT                   0x02
#define VMD_COLOR_DEPTH_16BIT                   0x03
	u8 color_depth;
#define COLORIMETRY_601		0
#define COLORIMETRY_709		1
	u8 colorimetry;
#define EXTERNAL_HSVSDE		0
#define INTERNAL_DE             1
#define EMBEDDED_SYNC           2
	u8 syncmode;
	u8 threeDstruct;
	u8 threeDextdata;

#define AMODE_I2S               0
#define AMODE_SPDIF             1
#define AMODE_HBR               2
#define AMODE_DSD               3
	u8 audio_mode;
#define ACHANNEL_2CH            1
#define ACHANNEL_3CH            2
#define ACHANNEL_4CH            3
#define ACHANNEL_5CH            4
#define ACHANNEL_6CH            5
#define ACHANNEL_7CH            6
#define ACHANNEL_8CH            7
	u8 audio_channels;
	u8 audiofs;
	u8 audio_word_len;
	u8 audio_i2s_fmt;
};

static __attribute__ ((unused)) void dump_regs(struct sii902x_data *sii902x,
					u8 reg, int len)
{
	u8 buf[50];
	int i;

	i2c_smbus_read_i2c_block_data(sii902x->client, reg, len, buf);
	for (i = 0; i < len; i++)
		dev_dbg(&sii902x->client->dev, "reg[0x%02X]: 0x%02X\n",
				i+reg, buf[i]);
}

static ssize_t sii902x_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);

	strcpy(buf, sii902x->fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, sii902x_show_name, NULL);

static ssize_t sii902x_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);

	if (sii902x->cable_plugin == false)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, sii902x_show_state, NULL);

static ssize_t sii902x_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);
	int i, j, len = 0;

	for (j = 0; j < SII_EDID_LEN/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					sii902x->edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, sii902x_show_edid, NULL);

/*------------------------------------------------------------------------------
 * Function Description: Write "0" to all bits in TPI offset "Offset" that are set
 *                 to "1" in "Pattern"; Leave all other bits in "Offset"
 *                 unchanged.
 *-----------------------------------------------------------------------------
 */
void read_clr_write_tpi(struct i2c_client *client, u8 offset, u8 mask)
{
	u8 tmp;

	tmp = i2c_smbus_read_byte_data(client, offset);
	tmp &= ~mask;
	i2c_smbus_write_byte_data(client, offset, tmp);
}

/*------------------------------------------------------------------------------
 * Function Description: Write "1" to all bits in TPI offset "Offset" that are set
 *                 to "1" in "Pattern"; Leave all other bits in "Offset"
 *                 unchanged.
 *-----------------------------------------------------------------------------
 */
void read_set_write_tpi(struct i2c_client *client, u8 offset, u8 mask)
{
	u8 tmp;

	tmp = i2c_smbus_read_byte_data(client, offset);
	tmp |= mask;
	i2c_smbus_write_byte_data(client, offset, tmp);
}

/*------------------------------------------------------------------------------
 * Function Description: Write "Value" to all bits in TPI offset "Offset" that are set
 *                 to "1" in "Mask"; Leave all other bits in "Offset"
 *                 unchanged.
 *----------------------------------------------------------------------------
 */
void read_modify_tpi(struct i2c_client *client, u8 offset, u8 mask, u8 value)
{
	u8 tmp;

	tmp = i2c_smbus_read_byte_data(client, offset);
	tmp &= ~mask;
	tmp |= (value & mask);
	i2c_smbus_write_byte_data(client, offset, tmp);
}

/*------------------------------------------------------------------------------
 * Function Description: Read an indexed register value
 *                 Write:
 *                     1. 0xBC => Internal page num
 *                     2. 0xBD => Indexed register offset
 *                 Read:
 *                     3. 0xBE => Returns the indexed register value
 *----------------------------------------------------------------------------
 */
int read_idx_reg(struct i2c_client *client, u8 page, u8 regoffset)
{
	i2c_smbus_write_byte_data(client, TPI_INTERNAL_PAGE_REG, page);
	i2c_smbus_write_byte_data(client, TPI_INDEXED_OFFSET_REG, regoffset);
	return i2c_smbus_read_byte_data(client, TPI_INDEXED_VALUE_REG);
}

/*------------------------------------------------------------------------------
 * Function Description: Write a value to an indexed register
 *
 *                  Write:
 *                      1. 0xBC => Internal page num
 *                      2. 0xBD => Indexed register offset
 *                      3. 0xBE => Set the indexed register value
 *------------------------------------------------------------------------------
 */
void write_idx_reg(struct i2c_client *client, u8 page, u8 regoffset, u8 regval)
{
	i2c_smbus_write_byte_data(client, TPI_INTERNAL_PAGE_REG, page);
	i2c_smbus_write_byte_data(client, TPI_INDEXED_OFFSET_REG, regoffset);
	i2c_smbus_write_byte_data(client, TPI_INDEXED_VALUE_REG, regval);
}

/*------------------------------------------------------------------------------
 * Function Description: Write "Value" to all bits in TPI offset "Offset" that are set
 *                 to "1" in "Mask"; Leave all other bits in "Offset"
 *                 unchanged.
 *----------------------------------------------------------------------------
 */
void read_modify_idx_reg(struct i2c_client *client, u8 page, u8 regoffset, u8 mask, u8 value)
{
	u8 tmp;

	i2c_smbus_write_byte_data(client, TPI_INTERNAL_PAGE_REG, page);
	i2c_smbus_write_byte_data(client, TPI_INDEXED_OFFSET_REG, regoffset);
	tmp = i2c_smbus_read_byte_data(client, TPI_INDEXED_VALUE_REG);
	tmp &= ~mask;
	tmp |= (value & mask);
	i2c_smbus_write_byte_data(client, TPI_INDEXED_VALUE_REG, tmp);
}

static void sii902x_set_powerstate(struct sii902x_data *sii902x, u8 state)
{
	if (sii902x->power_state != state) {
		read_modify_tpi(sii902x->client, TPI_DEVICE_POWER_STATE_CTRL_REG,
				TX_POWER_STATE_MASK, state);
		sii902x->power_state = state;
	}
}

static void sii902x_setAVI(struct sii902x_data *sii902x)
{
	u8 avi_data[SIZE_AVI_INFOFRAME];
	u8 tmp;
	int i;

	dev_dbg(&sii902x->client->dev, "set AVI frame\n");

	memset(avi_data, 0, SIZE_AVI_INFOFRAME);

	if (sii902x->edid_cfg.cea_ycbcr444)
		tmp = 2;
	else if (sii902x->edid_cfg.cea_ycbcr422)
		tmp = 1;
	else
		tmp = 0;

	/* AVI byte1: Y1Y0 (output format) */
	avi_data[1] = (tmp << 5) & 0x60;
	/* A0 = 1; Active format identification data is present in the AVI InfoFrame.
	 * S1:S0 = 00;
	 */
	avi_data[1] |= 0x10;

	if (sii902x->ocolor_space == XVYCC444) {
		avi_data[2] = 0xC0;
		if (sii902x->colorimetry == COLORIMETRY_601)
			avi_data[3] &= ~0x70;
		else if (sii902x->colorimetry == COLORIMETRY_709)
			avi_data[3] = (avi_data[3] & ~0x70) | 0x10;
	} else if (sii902x->ocolor_space != RGB) {
		if (sii902x->colorimetry == COLORIMETRY_709)
			avi_data[2] = 0x80;/* AVI byte2: C1C0*/
		else if (sii902x->colorimetry == COLORIMETRY_601)
			avi_data[2] = 0x40;/* AVI byte2: C1C0 */
	} else {/* Carries no data */
		/* AVI Byte2: C1C0 */
		avi_data[2] &= ~0xc0; /* colorimetry = 0 */
		avi_data[3] &= ~0x70; /* Extended colorimetry = 0 */
	}

	avi_data[4] = sii902x->vic;

	/*  Set the Aspect Ration info into the Infoframe Byte 2 */
	if (sii902x->aspect_ratio == VMD_ASPECT_RATIO_16x9)
		avi_data[2] |= _16_To_9; /* AVI Byte2: M1M0 */
	else
		avi_data[2] |= _4_To_3;

       avi_data[2] |= SAME_AS_AR;  /* AVI Byte2: R3..R1 - Set to "Same as Picture Aspect Ratio" */
       avi_data[5] = sii902x->pixrep; /* AVI Byte5: Pixel Replication - PR3..PR0 */

       /* Calculate AVI InfoFrame ChecKsum */
       avi_data[0] = 0x82 + 0x02 + 0x0D;
       for (i = 1; i < SIZE_AVI_INFOFRAME; i++)
	       avi_data[0] += avi_data[i];
       avi_data[0] = 0x100 - avi_data[0];

       /* Write the Inforframe data to the TPI Infoframe registers */
	for (i = 0; i < SIZE_AVI_INFOFRAME; i++)
		i2c_smbus_write_byte_data(sii902x->client,
				TPI_AVI_BYTE_0 + i, avi_data[i]);

	dump_regs(sii902x, TPI_AVI_BYTE_0, SIZE_AVI_INFOFRAME);
}

#define TYPE_AUDIO_INFOFRAMES       0x84
#define AUDIO_INFOFRAMES_VERSION    0x01
#define AUDIO_INFOFRAMES_LENGTH     0x0A
/*------------------------------------------------------------------------------
* Function Description: Load Audio InfoFrame data into registers and send to sink
*
* Accepts: (1) Channel count
*              (2) speaker configuration per CEA-861D Tables 19, 20
*              (3) Coding type: 0x09 for DSD Audio. 0 (refer to stream header) for all the rest
*              (4) Sample Frequency. Non zero for HBR only
*              (5) Audio Sample Length. Non zero for HBR only.
*------------------------------------------------------------------------------
*/
static void  sii902x_setAIF(struct sii902x_data *sii902x,
		u8 codingtype, u8 sample_size, u8 sample_freq,
		u8 speaker_cfg)
{
	u8 aif_data[SIZE_AUDIO_INFOFRAME];
	u8 channel_count = sii902x->audio_channels & 0x07;
	int i;

	dev_dbg(&sii902x->client->dev, "set AIF frame\n");

	memset(aif_data, 0, SIZE_AUDIO_INFOFRAME);

	/*  Disbale MPEG/Vendor Specific InfoFrames */
	i2c_smbus_write_byte_data(sii902x->client, MISC_INFO_FRAMES_CTRL, DISABLE_AUDIO);

	aif_data[0] = TYPE_AUDIO_INFOFRAMES;
	aif_data[1] = AUDIO_INFOFRAMES_VERSION;
	aif_data[2] = AUDIO_INFOFRAMES_LENGTH;
	/* Calculate checksum - 0x84 + 0x01 + 0x0A */
	aif_data[3] = TYPE_AUDIO_INFOFRAMES +
		AUDIO_INFOFRAMES_VERSION + AUDIO_INFOFRAMES_LENGTH;

	aif_data[4] = channel_count; /* 0 for "Refer to Stream Header" or for 2 Channels. 0x07 for 8 Channels*/
	aif_data[4] |= (codingtype << 4); /* 0xC7[7:4] == 0b1001 for DSD Audio */
	aif_data[5] = ((sample_freq & 0x07) << 2) | (sample_size & 0x03);
	aif_data[7] = speaker_cfg;

	for (i = 4; i < SIZE_AUDIO_INFOFRAME; i++)
		aif_data[3] += aif_data[i];

	aif_data[3] = 0x100 - aif_data[3];

	/* Re-enable Audio InfoFrame transmission and repeat */
	i2c_smbus_write_byte_data(sii902x->client, MISC_INFO_FRAMES_CTRL, EN_AND_RPT_AUDIO);

	for (i = 0; i < SIZE_AUDIO_INFOFRAME; i++)
		i2c_smbus_write_byte_data(sii902x->client,
				MISC_INFO_FRAMES_TYPE + i, aif_data[i]);

	dump_regs(sii902x, MISC_INFO_FRAMES_TYPE, SIZE_AUDIO_INFOFRAME);
}

static void sii902x_setaudio(struct sii902x_data *sii902x)
{
	dev_dbg(&sii902x->client->dev, "set audio\n");

	/* mute audio */
	read_modify_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG,
			AUDIO_MUTE_MASK, AUDIO_MUTE_MUTED);
	if (sii902x->audio_mode == AMODE_I2S) {
		read_modify_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG,
				AUDIO_SEL_MASK, AUD_IF_I2S);
		i2c_smbus_write_byte_data(sii902x->client, TPI_AUDIO_HANDLING,
				0x08 | AUD_DO_NOT_CHECK);
	} else {
		read_modify_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG,
				AUDIO_SEL_MASK, AUD_IF_SPDIF);
		i2c_smbus_write_byte_data(sii902x->client, TPI_AUDIO_HANDLING,
				AUD_PASS_BASIC);
	}

	if (sii902x->audio_channels == ACHANNEL_2CH)
		read_clr_write_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG, 0x20);
	else
		read_set_write_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG, 0x20);

	if (sii902x->audio_mode == AMODE_I2S) {
		/* I2S - Map channels */
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_EN, 0x80);

		if (sii902x->audio_channels > ACHANNEL_2CH)
			i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_EN, 0x91);

		if (sii902x->audio_channels > ACHANNEL_4CH)
			i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_EN, 0xA2);

		if (sii902x->audio_channels > ACHANNEL_6CH)
			i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_EN, 0xB3);

		/* I2S - Stream Header Settings */
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_CHST_0, 0x00);
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_CHST_1, 0x00);
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_CHST_2, 0x00);
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_CHST_3, sii902x->audiofs);
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_CHST_4,
					(sii902x->audiofs << 4) | sii902x->audio_word_len);

		/* added for 16bit auido noise issue */
		write_idx_reg(sii902x->client, INDEXED_PAGE_1, 0x24, sii902x->audio_word_len);

		/* I2S - Input Configuration */
		i2c_smbus_write_byte_data(sii902x->client, TPI_I2S_IN_CFG, sii902x->audio_i2s_fmt);
	}

	i2c_smbus_write_byte_data(sii902x->client, TPI_AUDIO_SAMPLE_CTRL, REFER_TO_STREAM_HDR);

	sii902x_setAIF(sii902x, REFER_TO_STREAM_HDR, REFER_TO_STREAM_HDR, REFER_TO_STREAM_HDR, 0x00);

	/* unmute audio */
	read_modify_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG, AUDIO_MUTE_MASK, AUDIO_MUTE_NORMAL);
}

static void sii902x_setup(struct sii902x_data *sii902x, struct fb_info *fbi)
{
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	mm_segment_t old_fs;
	unsigned int fmt;
	int i;

	dev_dbg(&sii902x->client->dev, "setup..\n");

	sii902x->vic = mxc_edid_var_to_vic(&fbi->var);

	/* set TPI video mode */
	data[0] = PICOS2KHZ(fbi->var.pixclock) / 10;
	data[2] = fbi->var.hsync_len + fbi->var.left_margin +
		  fbi->var.xres + fbi->var.right_margin;
	data[3] = fbi->var.vsync_len + fbi->var.upper_margin +
		  fbi->var.yres + fbi->var.lower_margin;
	refresh = data[2] * data[3];
	refresh = (PICOS2KHZ(fbi->var.pixclock) * 1000) / refresh;
	data[1] = refresh * 100;
	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		i2c_smbus_write_byte_data(sii902x->client, i, tmp[i]);

	dump_regs(sii902x, 0, 8);

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_GET_DIFMT, (unsigned long)&fmt);
		set_fs(old_fs);
		if (fmt == IPU_PIX_FMT_VYU444) {
			sii902x->icolor_space = YCBCR444;
			dev_dbg(&sii902x->client->dev, "input color space YUV\n");
		} else {
			sii902x->icolor_space = RGB;
			dev_dbg(&sii902x->client->dev, "input color space RGB\n");
		}
	}

	/* reg 0x08: input bus/pixel: full pixel wide (24bit), rising edge */
	sii902x->tpivmode[0] = 0x70;
	/* reg 0x09: Set input format */
	if (sii902x->icolor_space == RGB)
		sii902x->tpivmode[1] =
			(((BITS_IN_RGB | BITS_IN_AUTO_RANGE) & ~BIT_EN_DITHER_10_8) & ~BIT_EXTENDED_MODE);
	else if (sii902x->icolor_space == YCBCR444)
		sii902x->tpivmode[1] =
			(((BITS_IN_YCBCR444 | BITS_IN_AUTO_RANGE) & ~BIT_EN_DITHER_10_8) & ~BIT_EXTENDED_MODE);
	else if ((sii902x->icolor_space == YCBCR422_16BITS) || (sii902x->icolor_space == YCBCR422_8BITS))
		sii902x->tpivmode[1] =
			(((BITS_IN_YCBCR422 | BITS_IN_AUTO_RANGE) & ~BIT_EN_DITHER_10_8) & ~BIT_EXTENDED_MODE);
	/* reg 0x0a: set output format to RGB */
	sii902x->tpivmode[2] = 0x00;

	if (fbi->var.vmode & FB_VMODE_ASPECT_16_9)
		sii902x->aspect_ratio = VMD_ASPECT_RATIO_16x9;
	else if (fbi->var.vmode & FB_VMODE_ASPECT_4_3)
		sii902x->aspect_ratio = VMD_ASPECT_RATIO_4x3;
	else if (fbi->var.xres/16 == fbi->var.yres/9)
		sii902x->aspect_ratio = VMD_ASPECT_RATIO_16x9;
	else
		sii902x->aspect_ratio = VMD_ASPECT_RATIO_4x3;

	if ((sii902x->vic == 6) || (sii902x->vic == 7) ||
		(sii902x->vic == 21) || (sii902x->vic == 22) ||
		(sii902x->vic == 2) || (sii902x->vic == 3) ||
		(sii902x->vic == 17) || (sii902x->vic == 18)) {
		sii902x->tpivmode[2] &= ~0x10; /*BT.601*/
		sii902x->colorimetry = COLORIMETRY_601;
		sii902x->aspect_ratio = VMD_ASPECT_RATIO_4x3;
	} else {
		sii902x->tpivmode[2] |= 0x10; /*BT.709*/
		sii902x->colorimetry = COLORIMETRY_709;
	}

	if ((sii902x->vic == 10) || (sii902x->vic == 11) ||
		(sii902x->vic == 12) || (sii902x->vic == 13) ||
		(sii902x->vic == 14) || (sii902x->vic == 15) ||
		(sii902x->vic == 25) || (sii902x->vic == 26) ||
		(sii902x->vic == 27) || (sii902x->vic == 28) ||
		(sii902x->vic == 29) || (sii902x->vic == 30) ||
		(sii902x->vic == 35) || (sii902x->vic == 36) ||
		(sii902x->vic == 37) || (sii902x->vic == 38))
		sii902x->pixrep = 1;
	else
		sii902x->pixrep = 0;

	dev_dbg(&sii902x->client->dev, "vic %d\n", sii902x->vic);
	dev_dbg(&sii902x->client->dev, "pixrep %d\n", sii902x->pixrep);
	if (sii902x->aspect_ratio == VMD_ASPECT_RATIO_4x3) {
		dev_dbg(&sii902x->client->dev, "aspect 4:3\n");
	} else {
		dev_dbg(&sii902x->client->dev, "aspect 16:9\n");
	}
	if (sii902x->colorimetry == COLORIMETRY_601) {
		dev_dbg(&sii902x->client->dev, "COLORIMETRY_601\n");
	} else {
		dev_dbg(&sii902x->client->dev, "COLORIMETRY_709\n");
	}
	dev_dbg(&sii902x->client->dev, "hdmi capbility %d\n", sii902x->edid_cfg.hdmi_cap);

	sii902x->ocolor_space = RGB;
	if (sii902x->edid_cfg.hdmi_cap) {
		if (sii902x->edid_cfg.cea_ycbcr444) {
			sii902x->ocolor_space = YCBCR444;
			sii902x->tpivmode[2] |= 0x1; /*Ycbcr444*/
		} else if (sii902x->edid_cfg.cea_ycbcr422) {
			sii902x->ocolor_space = YCBCR422_8BITS;
			sii902x->tpivmode[2] |= 0x2; /*Ycbcr422*/
		}
	}

	dev_dbg(&sii902x->client->dev, "write reg 0x08 0X%2X\n", sii902x->tpivmode[0]);
	dev_dbg(&sii902x->client->dev, "write reg 0x09 0X%2X\n", sii902x->tpivmode[1]);
	dev_dbg(&sii902x->client->dev, "write reg 0x0a 0X%2X\n", sii902x->tpivmode[2]);

	i2c_smbus_write_byte_data(sii902x->client, TPI_PIX_REPETITION, sii902x->tpivmode[0]);
	i2c_smbus_write_byte_data(sii902x->client, TPI_INPUT_FORMAT_REG, sii902x->tpivmode[1]);
	i2c_smbus_write_byte_data(sii902x->client, TPI_OUTPUT_FORMAT_REG, sii902x->tpivmode[2]);

	/* goto state D0*/
	sii902x_set_powerstate(sii902x, TX_POWER_STATE_D0);

	if (sii902x->edid_cfg.hdmi_cap) {
		sii902x_setAVI(sii902x);
		sii902x_setaudio(sii902x);
	} else {
		/* set last byte of TPI AVI InfoFrame for TPI AVI I/O format to take effect ?? */
		i2c_smbus_write_byte_data(sii902x->client, TPI_END_RIGHT_BAR_MSB, 0x00);

		/* mute audio */
		read_modify_tpi(sii902x->client, TPI_AUDIO_INTERFACE_REG,
				AUDIO_MUTE_MASK, AUDIO_MUTE_MUTED);
	}
}

#ifdef CONFIG_FB_MODE_HELPERS
static int sii902x_read_edid(struct sii902x_data *sii902x,
			struct fb_info *fbi)
{
	int old, dat, ret, cnt = 100;
	unsigned short addr = 0x50;
	u8 edid_old[SII_EDID_LEN];

	old = i2c_smbus_read_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG);

	i2c_smbus_write_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	i2c_smbus_write_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG, old | 0x06);

	/* save old edid */
	memcpy(edid_old, sii902x->edid, SII_EDID_LEN);

	/* edid reading */
	ret = mxc_edid_read(sii902x->client->adapter, addr,
				sii902x->edid, &sii902x->edid_cfg, fbi);

	cnt = 100;
	do {
		cnt--;
		i2c_smbus_write_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG, old & ~0x6);
		msleep(10);
		dat = i2c_smbus_read_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

done:
	i2c_smbus_write_byte_data(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG, old);

	if (!memcmp(edid_old, sii902x->edid, SII_EDID_LEN))
		ret = -2;
	return ret;
}
#else
static int sii902x_read_edid(struct sii902x_data *sii902x,
			struct fb_info *fbi)
{
	return -1;
}
#endif

static void sii902x_enable_tmds(struct sii902x_data *sii902x)
{
	/* goto state D0*/
	sii902x_set_powerstate(sii902x, TX_POWER_STATE_D0);

	/* Turn on DVI or HDMI */
	if (sii902x->edid_cfg.hdmi_cap)
		read_modify_tpi(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG,
				OUTPUT_MODE_MASK, OUTPUT_MODE_HDMI);
	else
		read_modify_tpi(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG,
				OUTPUT_MODE_MASK, OUTPUT_MODE_DVI);

	read_modify_tpi(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG,
		LINK_INTEGRITY_MODE_MASK | TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK,
		LINK_INTEGRITY_DYNAMIC | TMDS_OUTPUT_CONTROL_ACTIVE | AV_MUTE_NORMAL);

	i2c_smbus_write_byte_data(sii902x->client, TPI_PIX_REPETITION,
				sii902x->tpivmode[0]);
}

static void sii902x_disable_tmds(struct sii902x_data *sii902x)
{
	read_modify_tpi(sii902x->client, TPI_SYSTEM_CONTROL_DATA_REG,
		TMDS_OUTPUT_CONTROL_MASK | AV_MUTE_MASK | OUTPUT_MODE_MASK,
		TMDS_OUTPUT_CONTROL_POWER_DOWN | AV_MUTE_MUTED | OUTPUT_MODE_DVI);

	/* goto state D2*/
	sii902x_set_powerstate(sii902x, TX_POWER_STATE_D2);
}

static void sii902x_poweron(struct sii902x_data *sii902x)
{
	struct fsl_mxc_lcd_platform_data *plat = sii902x->client->dev.platform_data;

	dev_dbg(&sii902x->client->dev, "power on\n");

	/* Enable pins to HDMI */
	if (plat->enable_pins)
		plat->enable_pins();

	if (sii902x->rx_powerup)
		sii902x_enable_tmds(sii902x);
}

static void sii902x_poweroff(struct sii902x_data *sii902x)
{
	struct fsl_mxc_lcd_platform_data *plat = sii902x->client->dev.platform_data;

	dev_dbg(&sii902x->client->dev, "power off\n");

	/* Disable pins to HDMI */
	if (plat->disable_pins)
		plat->disable_pins();

	if (sii902x->rx_powerup)
		sii902x_disable_tmds(sii902x);
}

static void sii902x_rx_powerup(struct sii902x_data *sii902x)
{

	dev_dbg(&sii902x->client->dev, "rx power up\n");

	if (sii902x->need_mode_change) {
		sii902x->fbi->var.activate |= FB_ACTIVATE_FORCE;
		console_lock();
		sii902x->fbi->flags |= FBINFO_MISC_USEREVENT;
		fb_set_var(sii902x->fbi, &sii902x->fbi->var);
		sii902x->fbi->flags &= ~FBINFO_MISC_USEREVENT;
		console_unlock();
		sii902x->need_mode_change = false;
	}

	sii902x_enable_tmds(sii902x);

	sii902x->rx_powerup = true;
}

static void sii902x_rx_powerdown(struct sii902x_data *sii902x)
{
	dev_dbg(&sii902x->client->dev, "rx power down\n");

	sii902x_disable_tmds(sii902x);

	sii902x->rx_powerup = false;
}

static int sii902x_cable_connected(struct sii902x_data *sii902x)
{
	int ret;

	dev_dbg(&sii902x->client->dev, "cable connected\n");

	sii902x->cable_plugin = true;

	/* edid read */
	ret = sii902x_read_edid(sii902x, sii902x->fbi);
	if (ret == -1)
		dev_err(&sii902x->client->dev,
				"read edid fail\n");
	else if (ret == -2)
		dev_info(&sii902x->client->dev,
				"same edid\n");
	else {
		if (sii902x->fbi->monspecs.modedb_len > 0) {
			int i;
			const struct fb_videomode *mode;
			struct fb_videomode m;

			fb_destroy_modelist(&sii902x->fbi->modelist);

			for (i = 0; i < sii902x->fbi->monspecs.modedb_len; i++) {
				/*FIXME now we do not support interlaced mode */
				if (!(sii902x->fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
					fb_add_videomode(&sii902x->fbi->monspecs.modedb[i],
							&sii902x->fbi->modelist);
			}

			fb_var_to_videomode(&m, &sii902x->fbi->var);
			mode = fb_find_nearest_mode(&m,
					&sii902x->fbi->modelist);

			fb_videomode_to_var(&sii902x->fbi->var, mode);
			sii902x->need_mode_change = true;
		}
	}

	/* ?? remain it for control back door register */
	read_modify_idx_reg(sii902x->client, INDEXED_PAGE_0, 0x0a, 0x08, 0x08);

	return 0;
}

static void sii902x_cable_disconnected(struct sii902x_data *sii902x)
{
	dev_dbg(&sii902x->client->dev, "cable disconnected\n");
	sii902x_rx_powerdown(sii902x);
	sii902x->cable_plugin = false;
}

static void det_worker(struct work_struct *work)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct sii902x_data *sii902x =
		container_of(delay_work, struct sii902x_data, det_work);
	int status;
	char event_string[16];
	char *envp[] = { event_string, NULL };

	status = i2c_smbus_read_byte_data(sii902x->client, TPI_INTERRUPT_STATUS_REG);

	/* check cable status */
	if (status & HOT_PLUG_EVENT) {
		/* cable connection changes */
		if ((status & HOT_PLUG_STATE) != sii902x->cable_plugin) {
			if (status & HOT_PLUG_STATE) {
				sprintf(event_string, "EVENT=plugin");
				sii902x_cable_connected(sii902x);
			} else {
				sprintf(event_string, "EVENT=plugout");
				sii902x_cable_disconnected(sii902x);
			}
			kobject_uevent_env(&sii902x->pdev->dev.kobj, KOBJ_CHANGE, envp);
		}
	}

	/* check rx power */
	if (((status & RX_SENSE_STATE) >> 3) != sii902x->rx_powerup) {
		if (sii902x->cable_plugin) {
			if (status & RX_SENSE_STATE)
				sii902x_rx_powerup(sii902x);
			else
				sii902x_rx_powerdown(sii902x);
		}
	}

	/* clear interrupt pending status */
	i2c_smbus_write_byte_data(sii902x->client, TPI_INTERRUPT_STATUS_REG, status);
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{
	struct sii902x_data *sii902x = data;

	schedule_delayed_work(&(sii902x->det_work), msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static int sii902x_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	struct sii902x_data *sii902x = container_of(nb, struct sii902x_data, nb);

	if (strcmp(event->info->fix.id, sii902x->fbi->fix.id))
		return 0;

	switch (val) {
	case FB_EVENT_MODE_CHANGE:
		sii902x_setup(sii902x, fbi);
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			sii902x_poweron(sii902x);
		else
			sii902x_poweroff(sii902x);
		break;
	}
	return 0;
}

static int sii902x_TPI_init(struct i2c_client *client)
{
	struct fsl_mxc_lcd_platform_data *plat = client->dev.platform_data;
	u8 devid = 0;
	u16 wid = 0;

	if (plat->reset)
		plat->reset();

	/* sii902x back door register - Set terminations to default */
	i2c_smbus_write_byte_data(client, 0x82, 0x25);
	/* sii902x back door register - HW debounce to 64ms (0x14) */
	i2c_smbus_write_byte_data(client, 0x7c, 0x14);

	/* Set 902x in hardware TPI mode on and jump out of D3 state */
	if (i2c_smbus_write_byte_data(client, TPI_ENABLE, 0x00) < 0) {
		dev_err(&client->dev,
			"cound not find device\n");
		return -ENODEV;
	}

	msleep(100);

	/* read device ID */
	devid = read_idx_reg(client, INDEXED_PAGE_0, 0x03);
	wid = devid;
	wid <<= 8;
	devid = read_idx_reg(client, INDEXED_PAGE_0, 0x02);
	wid |= devid;
	devid = i2c_smbus_read_byte_data(client, TPI_DEVICE_ID);

	if (devid == 0xB0)
		dev_info(&client->dev, "found device %04X", wid);
	else {
		dev_err(&client->dev, "cound not find device\n");
		return -ENODEV;
	}

	return 0;
}

static int sii902x_disp_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret = 0;
	struct sii902x_data *sii902x = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_lcd_platform_data *plat = sii902x->client->dev.platform_data;
	bool found = false;
	static bool inited;

	if (inited)
		return -EBUSY;

	inited = true;

	setting->dev_id = plat->ipu_id;
	setting->disp_id = plat->disp_id;
	setting->if_fmt = IPU_PIX_FMT_RGB24;

	sii902x->fbi = setting->fbi;
	sii902x->power_state = TX_POWER_STATE_D2;
	sii902x->icolor_space = RGB;
	sii902x->audio_mode = AMODE_SPDIF;
	sii902x->audio_channels = ACHANNEL_2CH;

	sii902x->pdev = platform_device_register_simple("sii902x", 0, NULL, 0);
	if (IS_ERR(sii902x->pdev)) {
		dev_err(&sii902x->client->dev,
				"Unable to register Sii902x as a platform device\n");
		ret = PTR_ERR(sii902x->pdev);
		goto register_pltdev_failed;
	}

	if (plat->io_reg) {
		sii902x->io_reg = regulator_get(&sii902x->client->dev, plat->io_reg);
		if (!IS_ERR(sii902x->io_reg)) {
			regulator_set_voltage(sii902x->io_reg, 3300000, 3300000);
			regulator_enable(sii902x->io_reg);
		}
	}
	if (plat->analog_reg) {
		sii902x->analog_reg = regulator_get(&sii902x->client->dev, plat->analog_reg);
		if (!IS_ERR(sii902x->analog_reg)) {
			regulator_set_voltage(sii902x->analog_reg, 1300000, 1300000);
			regulator_enable(sii902x->analog_reg);
		}
	}

	/* Claim HDMI pins */
	if (plat->get_pins)
		if (!plat->get_pins()) {
			ret = -EACCES;
			goto get_pins_failed;
		}

	ret = sii902x_TPI_init(sii902x->client);
	if (ret < 0)
		goto init_failed;

	/* try to read edid */
	ret = sii902x_read_edid(sii902x, sii902x->fbi);
	if (ret < 0)
		dev_warn(&sii902x->client->dev, "Can not read edid\n");
	else {
		INIT_LIST_HEAD(&sii902x->fbi->modelist);
		if (sii902x->fbi->monspecs.modedb_len > 0) {
			int i;
			const struct fb_videomode *mode;
			struct fb_videomode m;

			for (i = 0; i < sii902x->fbi->monspecs.modedb_len; i++) {
				/*FIXME now we do not support interlaced mode */
				if (!(sii902x->fbi->monspecs.modedb[i].vmode
							& FB_VMODE_INTERLACED))
					fb_add_videomode(
							&sii902x->fbi->monspecs.modedb[i],
							&sii902x->fbi->modelist);
			}

			fb_find_mode(&sii902x->fbi->var, sii902x->fbi, setting->dft_mode_str,
					NULL, 0, NULL, setting->default_bpp);

			fb_var_to_videomode(&m, &sii902x->fbi->var);
			mode = fb_find_nearest_mode(&m,
					&sii902x->fbi->modelist);
			fb_videomode_to_var(&sii902x->fbi->var, mode);
			found = true;
		}

	}

	if (!found) {
		ret = fb_find_mode(&sii902x->fbi->var, sii902x->fbi, setting->dft_mode_str,
				NULL, 0, NULL, setting->default_bpp);
		if (!ret) {
			ret = -EINVAL;
			goto find_mode_failed;
		}
	}

	if (sii902x->client->irq) {
		ret = request_irq(sii902x->client->irq, sii902x_detect_handler,
				IRQF_TRIGGER_FALLING,
				"SII902x_det", sii902x);
		if (ret < 0)
			dev_warn(&sii902x->client->dev,
				"cound not request det irq %d\n",
				sii902x->client->irq);
		else {
			/*enable cable hot plug irq*/
			i2c_smbus_write_byte_data(sii902x->client,
					TPI_INTERRUPT_ENABLE_REG,
					HOT_PLUG_EVENT | RX_SENSE_EVENT);
			INIT_DELAYED_WORK(&(sii902x->det_work), det_worker);
			/*clear hot plug event status*/
			i2c_smbus_write_byte_data(sii902x->client,
					TPI_INTERRUPT_STATUS_REG,
					HOT_PLUG_EVENT | RX_SENSE_EVENT);
		}

		ret = device_create_file(&sii902x->pdev->dev, &dev_attr_fb_name);
		if (ret < 0)
			dev_warn(&sii902x->client->dev,
				"cound not create sys node for fb name\n");
		ret = device_create_file(&sii902x->pdev->dev, &dev_attr_cable_state);
		if (ret < 0)
			dev_warn(&sii902x->client->dev,
				"cound not create sys node for cable state\n");
		ret = device_create_file(&sii902x->pdev->dev, &dev_attr_edid);
		if (ret < 0)
			dev_warn(&sii902x->client->dev,
				"cound not create sys node for edid\n");

		dev_set_drvdata(&sii902x->pdev->dev, sii902x);
	}

	sii902x->nb.notifier_call = sii902x_fb_event;
	ret = fb_register_client(&sii902x->nb);
	if (ret < 0)
		goto reg_fbclient_failed;

	return ret;

reg_fbclient_failed:
find_mode_failed:
init_failed:
get_pins_failed:
	platform_device_unregister(sii902x->pdev);
register_pltdev_failed:
	return ret;
}

static void sii902x_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct sii902x_data *sii902x = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_lcd_platform_data *plat = sii902x->client->dev.platform_data;

	if (sii902x->client->irq)
		free_irq(sii902x->client->irq, sii902x);

	fb_unregister_client(&sii902x->nb);

	sii902x_poweroff(sii902x);

	/* Release HDMI pins */
	if (plat->put_pins)
		plat->put_pins();

	platform_device_unregister(sii902x->pdev);

	kfree(sii902x);
}

static struct mxc_dispdrv_driver sii902x_drv = {
	.name 	= DISPDRV_SII,
	.init 	= sii902x_disp_init,
	.deinit	= sii902x_disp_deinit,
};

static int __devinit sii902x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sii902x_data *sii902x;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	sii902x = kzalloc(sizeof(struct sii902x_data), GFP_KERNEL);
	if (!sii902x) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	sii902x->client = client;

	sii902x->disp_hdmi = mxc_dispdrv_register(&sii902x_drv);
	mxc_dispdrv_setdata(sii902x->disp_hdmi, sii902x);

	i2c_set_clientdata(client, sii902x);

alloc_failed:
	return ret;
}

static int __devexit sii902x_remove(struct i2c_client *client)
{
	struct sii902x_data *sii902x = i2c_get_clientdata(client);

	mxc_dispdrv_puthandle(sii902x->disp_hdmi);
	mxc_dispdrv_unregister(sii902x->disp_hdmi);
	kfree(sii902x);
	return 0;
}

static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static struct i2c_driver sii902x_i2c_driver = {
	.driver = {
		   .name = "sii902x",
		   },
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.id_table = sii902x_id,
};

static int __init sii902x_init(void)
{
	return i2c_add_driver(&sii902x_i2c_driver);
}

static void __exit sii902x_exit(void)
{
	i2c_del_driver(&sii902x_i2c_driver);
}

module_init(sii902x_init);
module_exit(sii902x_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");
