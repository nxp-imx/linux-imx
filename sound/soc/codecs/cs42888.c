/*
 * cs42888.c  -- CS42888 ALSA SoC Audio Driver
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>
#include "cs42888.h"

#include "../imx/imx-pcm.h"
#define CS42888_NUM_SUPPLIES 4
static const char *cs42888_supply_names[CS42888_NUM_SUPPLIES] = {
	"VA",
	"VD",
	"VLS",
	"VLC",
};

#define CS42888_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* CS42888 registers addresses */
#define CS42888_CHIPID		0x01	/* Chip ID */
#define CS42888_PWRCTL		0x02	/* Power Control */
#define CS42888_MODE		0x03	/* Functional Mode */
#define CS42888_FORMAT		0x04	/* Interface Formats */
#define CS42888_ADCCTL		0x05	/* ADC Control */
#define CS42888_TRANS		0x06	/* Transition Control */
#define CS42888_MUTE		0x07	/* Mute Control */
#define CS42888_VOLAOUT1	0x08	/* Volume Control AOUT1*/
#define CS42888_VOLAOUT2	0x09	/* Volume Control AOUT2*/
#define CS42888_VOLAOUT3	0x0A	/* Volume Control AOUT3*/
#define CS42888_VOLAOUT4	0x0B	/* Volume Control AOUT4*/
#define CS42888_VOLAOUT5	0x0C	/* Volume Control AOUT5*/
#define CS42888_VOLAOUT6	0x0D	/* Volume Control AOUT6*/
#define CS42888_VOLAOUT7	0x0E	/* Volume Control AOUT7*/
#define CS42888_VOLAOUT8	0x0F	/* Volume Control AOUT8*/
#define CS42888_DACINV		0x10	/* DAC Channel Invert */
#define CS42888_VOLAIN1		0x11	/* Volume Control AIN1 */
#define CS42888_VOLAIN2		0x12	/* Volume Control AIN2 */
#define CS42888_VOLAIN3		0x13	/* Volume Control AIN3 */
#define CS42888_VOLAIN4		0x14	/* Volume Control AIN4 */
#define CS42888_ADCINV		0x17	/* ADC Channel Invert */
#define CS42888_STATUSCTL	0x18	/* Status Control */
#define CS42888_STATUS		0x19	/* Status */
#define CS42888_STATUSMASK	0x1A	/* Status Mask */

#define CS42888_FIRSTREG	0x01
#define CS42888_LASTREG		0x1A
#define CS42888_NUMREGS	(CS42888_LASTREG - CS42888_FIRSTREG + 1)
#define CS42888_I2C_INCR	0x80

/* Bit masks for the CS42888 registers */
#define CS42888_CHIPID_ID_MASK	0xF0
#define CS42888_CHIPID_REV	0x0F
#define CS42888_PWRCTL_PDN_ADC2_OFFSET		6
#define CS42888_PWRCTL_PDN_ADC1_OFFSET		5
#define CS42888_PWRCTL_PDN_DAC4_OFFSET		4
#define CS42888_PWRCTL_PDN_DAC3_OFFSET		3
#define CS42888_PWRCTL_PDN_DAC2_OFFSET		2
#define CS42888_PWRCTL_PDN_DAC1_OFFSET		1
#define CS42888_PWRCTL_PDN_OFFSET		0
#define CS42888_PWRCTL_PDN_ADC2_MASK	(1 << CS42888_PWRCTL_PDN_ADC2_OFFSET)
#define CS42888_PWRCTL_PDN_ADC1_MASK	(1 << CS42888_PWRCTL_PDN_ADC1_OFFSET)
#define CS42888_PWRCTL_PDN_DAC4_MASK	(1 << CS42888_PWRCTL_PDN_DAC4_OFFSET)
#define CS42888_PWRCTL_PDN_DAC3_MASK	(1 << CS42888_PWRCTL_PDN_DAC3_OFFSET)
#define CS42888_PWRCTL_PDN_DAC2_MASK	(1 << CS42888_PWRCTL_PDN_DAC2_OFFSET)
#define CS42888_PWRCTL_PDN_DAC1_MASK	(1 << CS42888_PWRCTL_PDN_DAC1_OFFSET)
#define CS42888_PWRCTL_PDN_MASK		(1 << CS42888_PWRCTL_PDN_OFFSET)

#define CS42888_MODE_SPEED_MASK	0xF0
#define CS42888_MODE_1X		0x00
#define CS42888_MODE_2X		0x50
#define CS42888_MODE_4X		0xA0
#define CS42888_MODE_SLAVE	0xF0
#define CS42888_MODE_DIV_MASK	0x0E
#define CS42888_MODE_DIV1	0x00
#define CS42888_MODE_DIV2	0x02
#define CS42888_MODE_DIV3	0x04
#define CS42888_MODE_DIV4	0x06
#define CS42888_MODE_DIV5	0x08

#define CS42888_FORMAT_FREEZE_OFFSET	7
#define CS42888_FORMAT_AUX_DIF_OFFSET	6
#define CS42888_FORMAT_DAC_DIF_OFFSET	3
#define CS42888_FORMAT_ADC_DIF_OFFSET	0
#define CS42888_FORMAT_FREEZE_MASK	(1 << CS42888_FORMAT_FREEZE_OFFSET)
#define CS42888_FORMAT_AUX_DIF_MASK	(1 << CS42888_FORMAT_AUX_DIF_OFFSET)
#define CS42888_FORMAT_DAC_DIF_MASK	(7 << CS42888_FORMAT_DAC_DIF_OFFSET)
#define CS42888_FORMAT_ADC_DIF_MASK	(7 << CS42888_FORMAT_ADC_DIF_OFFSET)

#define CS42888_TRANS_DAC_SNGVOL_OFFSET	    7
#define CS42888_TRANS_DAC_SZC_OFFSET	    5
#define CS42888_TRANS_AMUTE_OFFSET	    4
#define CS42888_TRANS_MUTE_ADC_SP_OFFSET    3
#define CS42888_TRANS_ADC_SNGVOL_OFFSET	    2
#define CS42888_TRANS_ADC_SZC_OFFSET	    0
#define CS42888_TRANS_DAC_SNGVOL_MASK	(1 << CS42888_TRANS_DAC_SNGVOL_OFFSET)
#define CS42888_TRANS_DAC_SZC_MASK	(3 << CS42888_TRANS_DAC_SZC_OFFSET)
#define CS42888_TRANS_AMUTE_MASK	(1 << CS42888_TRANS_AMUTE_OFFSET)
#define CS42888_TRANS_MUTE_ADC_SP_MASK	(1 << CS42888_TRANS_MUTE_ADC_SP_OFFSET)
#define CS42888_TRANS_ADC_SNGVOL_MASK	(1 << CS42888_TRANS_ADC_SNGVOL_OFFSET)
#define CS42888_TRANS_ADC_SZC_MASK	(3 << CS42888_TRANS_ADC_SZC_OFFSET)

#define CS42888_TRANS_DAC_SZC_IC     (0 << CS42888_TRANS_DAC_SZC_OFFSET)
#define CS42888_TRANS_DAC_SZC_ZC     (1 << CS42888_TRANS_DAC_SZC_OFFSET)
#define CS42888_TRANS_DAC_SZC_SR     (2 << CS42888_TRANS_DAC_SZC_OFFSET)
#define CS42888_TRANS_DAC_SZC_SRZC   (3 << CS42888_TRANS_DAC_SZC_OFFSET)

#define CS42888_MUTE_AOUT8	(0x1 << 7)
#define CS42888_MUTE_AOUT7	(0x1 << 6)
#define CS42888_MUTE_AOUT6	(0x1 << 5)
#define CS42888_MUTE_AOUT5	(0x1 << 4)
#define CS42888_MUTE_AOUT4	(0x1 << 3)
#define CS42888_MUTE_AOUT3	(0x1 << 2)
#define CS42888_MUTE_AOUT2	(0x1 << 1)
#define CS42888_MUTE_AOUT1	(0x1 << 0)
#define CS42888_MUTE_ALL	(CS42888_MUTE_AOUT1 | CS42888_MUTE_AOUT2 | \
				CS42888_MUTE_AOUT3 | CS42888_MUTE_AOUT4 | \
				CS42888_MUTE_AOUT5 | CS42888_MUTE_AOUT6 | \
				CS42888_MUTE_AOUT7 | CS42888_MUTE_AOUT8)

#define DIF_LEFT_J		0
#define DIF_I2S			1
#define DIF_RIGHT_J		2
#define DIF_TDM			6

/* Private data for the CS42888 */
struct cs42888_private {
	struct snd_soc_codec *codec;
	u8 reg_cache[CS42888_NUMREGS + 1];
	unsigned int mclk; /* Input frequency of the MCLK pin */
	unsigned int mode; /* The mode (I2S or left-justified) */
	unsigned int slave_mode;
	unsigned int manual_mute;
	struct regulator_bulk_data supplies[CS42888_NUM_SUPPLIES];
	struct mxc_audio_codec_platform_data pdata;
};

/**
 * cs42888_fill_cache - pre-fill the CS42888 register cache.
 * @codec: the codec for this CS42888
 *
 * This function fills in the CS42888 register cache by reading the register
 * values from the hardware.
 *
 * This CS42888 registers are cached to avoid excessive I2C I/O operations.
 * After the initial read to pre-fill the cache, the CS42888 never updates
 * the register values, so we won't have a cache coherency problem.
 *
 * We use the auto-increment feature of the CS42888 to read all registers in
 * one shot.
 */
static int cs42888_fill_cache(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	struct i2c_client *i2c_client = codec->control_data;
	s32 length;

	length = i2c_smbus_read_i2c_block_data(i2c_client,
		CS42888_FIRSTREG | CS42888_I2C_INCR, CS42888_NUMREGS, \
		cache + 1);

	if (length != CS42888_NUMREGS) {
		dev_err(codec->dev, "i2c read failure, addr=0x%x\n",
		       i2c_client->addr);
		return -EIO;
	}

	return 0;
}


#ifdef CS42888_DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	int ret;
	u8 *cache = codec->reg_cache + 1;

	printk(KERN_DEBUG "dump begin\n");
	printk(KERN_DEBUG "reg value in cache\n");
	for (i = 0; i < CS42888_NUMREGS; i++)
		printk(KERN_DEBUG "reg[%d] = 0x%x\n", i, cache[i]);

	printk(KERN_DEBUG "real reg value\n");
	ret = cs42888_fill_cache(codec);
	if (ret < 0) {
		pr_err("failed to fill register cache\n");
		return ret;
	}
	for (i = 0; i < CS42888_NUMREGS; i++)
		printk(KERN_DEBUG "reg[%d] = 0x%x\n", i, cache[i]);

	printk(KERN_DEBUG "dump end\n");
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif

/* -127.5dB to 0dB with step of 0.5dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 50, 1);
/* -64dB to 24dB with step of 0.5dB */
static const DECLARE_TLV_DB_SCALE(adc_tlv, -6400, 50, 1);

static int cs42888_out_vu(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	int ret;
	u16 val;

	ret = snd_soc_put_volsw_2r(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	/* Now write again with the volume update bit set */
	val = snd_soc_read(codec, reg);
	ret = snd_soc_write(codec, reg, val);

	val = snd_soc_read(codec, reg2);
	ret = snd_soc_write(codec, reg2, val);
	return 0;
}

int cs42888_info_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	int min = mc->min;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max-min;
	return 0;
}

int cs42888_get_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	int min = mc->min;
	int val = snd_soc_read(codec, reg);

	ucontrol->value.integer.value[0] =
		((signed char)(val))-min;

	val = snd_soc_read(codec, reg2);
	ucontrol->value.integer.value[1] =
		((signed char)(val))-min;
	return 0;
}

int cs42888_put_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	int min = mc->min;
	unsigned short val;
	int ret;

	val = (ucontrol->value.integer.value[0]+min);
	ret = snd_soc_write(codec, reg, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	val = ((ucontrol->value.integer.value[1]+min));
	ret = snd_soc_write(codec, reg2, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	return 0;
}

#define SOC_CS42888_DOUBLE_R_TLV(xname, reg_left, reg_right, xshift, xmax, \
				    xinvert, tlv_array)			\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_2r, \
	.put = cs42888_out_vu, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, \
		    .rreg = reg_right, \
		    .shift = xshift, \
		    .max = xmax, \
		    .invert = xinvert} \
}

#define SOC_CS42888_DOUBLE_R_S8_TLV(xname, reg_left, reg_right, xmin, xmax, \
				    tlv_array) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p  = (tlv_array), \
	.info   = cs42888_info_volsw_s8, .get = cs42888_get_volsw_s8, \
	.put    = cs42888_put_volsw_s8, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, \
		    .rreg = reg_right, \
		    .min = xmin, \
		    .max = xmax} \
}

static const char *cs42888_adcfilter[] = { "None", "High Pass" };
static const char *cs42888_dacinvert[] = { "Disabled", "Enabled" };
static const char *cs42888_adcinvert[] = { "Disabled", "Enabled" };
static const char *cs42888_dacamute[] = { "Disabled", "AutoMute" };
static const char *cs42888_dac_sngvol[] = { "Disabled", "Enabled" };
static const char *cs42888_dac_szc[] = { "Immediate Change", "Zero Cross",
				"Soft Ramp", "Soft Ramp on Zero Cross" };
static const char *cs42888_mute_adc[] = { "UnMute", "Mute" };
static const char *cs42888_adc_sngvol[] = { "Disabled", "Enabled" };
static const char *cs42888_adc_szc[] = { "Immediate Change", "Zero Cross",
				"Soft Ramp", "Soft Ramp on Zero Cross" };
static const char *cs42888_dac_dem[] = { "No-De-Emphasis", "De-Emphasis" };
static const char *cs42888_adc_single[] = { "Differential", "Single-Ended" };

static const struct soc_enum cs42888_enum[] = {
	SOC_ENUM_SINGLE(CS42888_ADCCTL, 7, 2, cs42888_adcfilter),
	SOC_ENUM_DOUBLE(CS42888_DACINV, 0, 1, 2, cs42888_dacinvert),
	SOC_ENUM_DOUBLE(CS42888_DACINV, 2, 3, 2, cs42888_dacinvert),
	SOC_ENUM_DOUBLE(CS42888_DACINV, 4, 5, 2, cs42888_dacinvert),
	SOC_ENUM_DOUBLE(CS42888_DACINV, 6, 7, 2, cs42888_dacinvert),
	SOC_ENUM_DOUBLE(CS42888_ADCINV, 0, 1, 2, cs42888_adcinvert),
	SOC_ENUM_DOUBLE(CS42888_ADCINV, 2, 3, 2, cs42888_adcinvert),
	SOC_ENUM_SINGLE(CS42888_TRANS, 4, 2, cs42888_dacamute),
	SOC_ENUM_SINGLE(CS42888_TRANS, 7, 2, cs42888_dac_sngvol),
	SOC_ENUM_SINGLE(CS42888_TRANS, 5, 4, cs42888_dac_szc),
	SOC_ENUM_SINGLE(CS42888_TRANS, 3, 2, cs42888_mute_adc),
	SOC_ENUM_SINGLE(CS42888_TRANS, 2, 2, cs42888_adc_sngvol),
	SOC_ENUM_SINGLE(CS42888_TRANS, 0, 4, cs42888_adc_szc),
	SOC_ENUM_SINGLE(CS42888_ADCCTL, 5, 2, cs42888_dac_dem),
	SOC_ENUM_SINGLE(CS42888_ADCCTL, 4, 2, cs42888_adc_single),
	SOC_ENUM_SINGLE(CS42888_ADCCTL, 3, 2, cs42888_adc_single),
};

static const struct snd_kcontrol_new cs42888_snd_controls[] = {
SOC_CS42888_DOUBLE_R_TLV("DAC1 Playback Volume",
			    CS42888_VOLAOUT1,
			    CS42888_VOLAOUT2,
			    0, 0xff, 1, dac_tlv),
SOC_CS42888_DOUBLE_R_TLV("DAC2 Playback Volume",
			    CS42888_VOLAOUT3,
			    CS42888_VOLAOUT4,
			    0, 0xff, 1, dac_tlv),
SOC_CS42888_DOUBLE_R_TLV("DAC3 Playback Volume",
			    CS42888_VOLAOUT5,
			    CS42888_VOLAOUT6,
			    0, 0xff, 1, dac_tlv),
SOC_CS42888_DOUBLE_R_TLV("DAC4 Playback Volume",
			    CS42888_VOLAOUT7,
			    CS42888_VOLAOUT8,
			    0, 0xff, 1, dac_tlv),
SOC_CS42888_DOUBLE_R_S8_TLV("ADC1 Capture Volume",
			    CS42888_VOLAIN1,
			    CS42888_VOLAIN2,
			    -128, 48, adc_tlv),
SOC_CS42888_DOUBLE_R_S8_TLV("ADC2 Capture Volume",
			    CS42888_VOLAIN3,
			    CS42888_VOLAIN4,
			    -128, 48, adc_tlv),
SOC_ENUM("ADC High-Pass Filter Switch", cs42888_enum[0]),
SOC_ENUM("DAC1 Invert Switch", cs42888_enum[1]),
SOC_ENUM("DAC2 Invert Switch", cs42888_enum[2]),
SOC_ENUM("DAC3 Invert Switch", cs42888_enum[3]),
SOC_ENUM("DAC4 Invert Switch", cs42888_enum[4]),
SOC_ENUM("ADC1 Invert Switch", cs42888_enum[5]),
SOC_ENUM("ADC2 Invert Switch", cs42888_enum[6]),
SOC_ENUM("DAC Auto Mute Switch", cs42888_enum[7]),
SOC_ENUM("DAC Single Volume Control Switch", cs42888_enum[8]),
SOC_ENUM("DAC Soft Ramp and Zero Cross Control Switch", cs42888_enum[9]),
SOC_ENUM("Mute ADC Serial Port Switch", cs42888_enum[10]),
SOC_ENUM("ADC Single Volume Control Switch", cs42888_enum[11]),
SOC_ENUM("ADC Soft Ramp and Zero Cross Control Switch", cs42888_enum[12]),
SOC_ENUM("DAC Deemphasis Switch", cs42888_enum[13]),
SOC_ENUM("ADC1 Single Ended Mode Switch", cs42888_enum[14]),
SOC_ENUM("ADC2 Single Ended Mode Switch", cs42888_enum[15]),
};


static const struct snd_soc_dapm_widget cs42888_dapm_widgets[] = {
SND_SOC_DAPM_DAC("DAC1", "Playback", CS42888_PWRCTL, 1, 1),
SND_SOC_DAPM_DAC("DAC2", "Playback", CS42888_PWRCTL, 2, 1),
SND_SOC_DAPM_DAC("DAC3", "Playback", CS42888_PWRCTL, 3, 1),
SND_SOC_DAPM_DAC("DAC4", "Playback", CS42888_PWRCTL, 4, 1),

SND_SOC_DAPM_OUTPUT("AOUT1L"),
SND_SOC_DAPM_OUTPUT("AOUT1R"),
SND_SOC_DAPM_OUTPUT("AOUT2L"),
SND_SOC_DAPM_OUTPUT("AOUT2R"),
SND_SOC_DAPM_OUTPUT("AOUT3L"),
SND_SOC_DAPM_OUTPUT("AOUT3R"),
SND_SOC_DAPM_OUTPUT("AOUT4L"),
SND_SOC_DAPM_OUTPUT("AOUT4R"),

SND_SOC_DAPM_ADC("ADC1", "Capture", CS42888_PWRCTL, 5, 1),
SND_SOC_DAPM_ADC("ADC2", "Capture", CS42888_PWRCTL, 6, 1),

SND_SOC_DAPM_INPUT("AIN1L"),
SND_SOC_DAPM_INPUT("AIN1R"),
SND_SOC_DAPM_INPUT("AIN2L"),
SND_SOC_DAPM_INPUT("AIN2R"),

SND_SOC_DAPM_PGA_E("PWR", CS42888_PWRCTL, 0, 1, NULL, 0,
			NULL, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Playback */
	{ "PWR", NULL, "DAC1" },
	{ "PWR", NULL, "DAC1" },

	{ "PWR", NULL, "DAC2" },
	{ "PWR", NULL, "DAC2" },

	{ "PWR", NULL, "DAC3" },
	{ "PWR", NULL, "DAC3" },

	{ "PWR", NULL, "DAC4" },
	{ "PWR", NULL, "DAC4" },

	{ "AOUT1L", NULL, "PWR" },
	{ "AOUT1R", NULL, "PWR" },

	{ "AOUT2L", NULL, "PWR" },
	{ "AOUT2R", NULL, "PWR" },

	{ "AOUT3L", NULL, "PWR" },
	{ "AOUT3R", NULL, "PWR" },

	{ "AOUT4L", NULL, "PWR" },
	{ "AOUT4R", NULL, "PWR" },

	/* Capture */
	{ "PWR", NULL, "AIN1L" },
	{ "PWR", NULL, "AIN1R" },

	{ "PWR", NULL, "AIN2L" },
	{ "PWR", NULL, "AIN2R" },

	{ "ADC1", NULL, "PWR" },
	{ "ADC1", NULL, "PWR" },

	{ "ADC2", NULL, "PWR" },
	{ "ADC2", NULL, "PWR" },
};


static int cs42888_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(&codec->dapm, cs42888_dapm_widgets,
				  ARRAY_SIZE(cs42888_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(&codec->dapm);
	return 0;
}

/**
 * struct cs42888_mode_ratios - clock ratio tables
 * @ratio: the ratio of MCLK to the sample rate
 * @speed_mode: the Speed Mode bits to set in the Mode Control register for
 *              this ratio
 * @mclk: the Ratio Select bits to set in the Mode Control register for this
 *        ratio
 *
 * The data for this chart is taken from Table 10 of the CS42888 reference
 * manual.
 *
 * This table is used to determine how to program the Functional Mode register.
 * It is also used by cs42888_set_dai_sysclk() to tell ALSA which sampling
 * rates the CS42888 currently supports.
 *
 * @speed_mode is the corresponding bit pattern to be written to the
 * MODE bits of the Mode Control Register
 *
 * @mclk is the corresponding bit pattern to be wirten to the MCLK bits of
 * the Mode Control Register.
 *
 */
struct cs42888_mode_ratios {
	unsigned int ratio;
	u8 speed_mode;
	u8 mclk;
};

static struct cs42888_mode_ratios cs42888_mode_ratios[] = {
	{64, CS42888_MODE_4X, CS42888_MODE_DIV1},
	{96, CS42888_MODE_4X, CS42888_MODE_DIV2},
	{128, CS42888_MODE_2X, CS42888_MODE_DIV1},
	{192, CS42888_MODE_2X, CS42888_MODE_DIV2},
	{256, CS42888_MODE_1X, CS42888_MODE_DIV1},
	{384, CS42888_MODE_2X, CS42888_MODE_DIV4},
	{512, CS42888_MODE_1X, CS42888_MODE_DIV3},
	{768, CS42888_MODE_1X, CS42888_MODE_DIV4},
	{1024, CS42888_MODE_1X, CS42888_MODE_DIV5}
};

/* The number of MCLK/LRCK ratios supported by the CS42888 */
#define NUM_MCLK_RATIOS		ARRAY_SIZE(cs42888_mode_ratios)

/**
 * cs42888_set_dai_sysclk - determine the CS42888 samples rates.
 * @codec_dai: the codec DAI
 * @clk_id: the clock ID (ignored)
 * @freq: the MCLK input frequency
 * @dir: the clock direction (ignored)
 *
 * This function is used to tell the codec driver what the input MCLK
 * frequency is.
 *
 */
static int cs42888_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs42888_private *cs42888 = snd_soc_codec_get_drvdata(codec);

	cs42888->mclk = freq;
	return 0;
}

/**
 * cs42888_set_dai_fmt - configure the codec for the selected audio format
 * @codec_dai: the codec DAI
 * @format: a SND_SOC_DAIFMT_x value indicating the data format
 *
 * This function takes a bitmask of SND_SOC_DAIFMT_x bits and programs the
 * codec accordingly.
 *
 * Currently, this function only supports SND_SOC_DAIFMT_I2S and
 * SND_SOC_DAIFMT_LEFT_J.  The CS42888 codec also supports right-justified
 * data for playback only, but ASoC currently does not support different
 * formats for playback vs. record.
 */
static int cs42888_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs42888_private *cs42888 =  snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	u8 val;

	val = snd_soc_read(codec, CS42888_FORMAT);
	val &= ~CS42888_FORMAT_DAC_DIF_MASK;
	val &= ~CS42888_FORMAT_ADC_DIF_MASK;
	/* set DAI format */
	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J:
		val |= DIF_LEFT_J << CS42888_FORMAT_DAC_DIF_OFFSET;
		val |= DIF_LEFT_J << CS42888_FORMAT_ADC_DIF_OFFSET;
		break;
	case SND_SOC_DAIFMT_I2S:
		val |= DIF_I2S << CS42888_FORMAT_DAC_DIF_OFFSET;
		val |= DIF_I2S << CS42888_FORMAT_ADC_DIF_OFFSET;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val |= DIF_RIGHT_J << CS42888_FORMAT_DAC_DIF_OFFSET;
		val |= DIF_RIGHT_J << CS42888_FORMAT_ADC_DIF_OFFSET;
		break;
	default:
		dev_err(codec->dev, "invalid dai format\n");
		ret = -EINVAL;
		return ret;
	}

	ret = snd_soc_write(codec, CS42888_FORMAT, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	val = snd_soc_read(codec, CS42888_MODE);
	/* set master/slave audio interface */
	switch (format & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		cs42888->slave_mode = 1;
		val &= ~CS42888_MODE_SPEED_MASK;
		val |= CS42888_MODE_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		cs42888->slave_mode = 0;
		break;
	default:
		/* all other modes are unsupported by the hardware */
		ret = -EINVAL;
		return ret;
	}

	ret = snd_soc_write(codec, CS42888_MODE, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	dump_reg(codec);
	return ret;
}

/**
 * cs42888_hw_params - program the CS42888 with the given hardware parameters.
 * @substream: the audio stream
 * @params: the hardware parameters to set

 * @dai: the SOC DAI (ignored)
 *
 * This function programs the hardware with the values provided.
 * Specifically, the sample rate and the data format.
 *
 * The .ops functions are used to provide board-specific data, like input
 * frequencies, to this driver.  This function takes that information,
 * combines it with the hardware parameters provided, and programs the
 * hardware accordingly.
 */
static int cs42888_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct imx_pcm_runtime_data *iprtd = substream->runtime->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct cs42888_private *cs42888 =  snd_soc_codec_get_drvdata(codec);
	int ret;
	unsigned int i;
	unsigned int rate;
	unsigned int ratio;
	u32 val;

	if (iprtd->asrc_enable)
		rate = iprtd->p2p->p2p_rate;
	else
		rate = params_rate(params);	/* Sampling rate, in Hz */
	ratio = cs42888->mclk / rate;	/* MCLK/LRCK ratio */
	for (i = 0; i < NUM_MCLK_RATIOS; i++) {
		if (cs42888_mode_ratios[i].ratio == ratio)
			break;
	}

	if (i == NUM_MCLK_RATIOS) {
		/* We did not find a matching ratio */
		dev_err(codec->dev, "could not find matching ratio\n");
		return -EINVAL;
	}

	if (!cs42888->slave_mode) {
		val = snd_soc_read(codec, CS42888_MODE);
		val &= ~CS42888_MODE_SPEED_MASK;
		val |= cs42888_mode_ratios[i].speed_mode;
		val &= ~CS42888_MODE_DIV_MASK;
		val |= cs42888_mode_ratios[i].mclk;
	} else {
		val = snd_soc_read(codec, CS42888_MODE);
		val &= ~CS42888_MODE_SPEED_MASK;
		val |= CS42888_MODE_SLAVE;
		val &= ~CS42888_MODE_DIV_MASK;
		val |= cs42888_mode_ratios[i].mclk;
	}
	ret = snd_soc_write(codec, CS42888_MODE, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	/* Unmute all the channels */
	val = snd_soc_read(codec, CS42888_MUTE);
	val &= ~CS42888_MUTE_ALL;
	ret = snd_soc_write(codec, CS42888_MUTE, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}

	ret = cs42888_fill_cache(codec);
	if (ret < 0) {
		pr_err("failed to fill register cache\n");
		return ret;
	}

	dump_reg(codec);
	return ret;
}

/**
 * cs42888_shutdown - cs42888 enters into low power mode again.
 * @substream: the audio stream
 * @dai: the SOC DAI (ignored)
 *
 * The .ops functions are used to provide board-specific data, like input
 * frequencies, to this driver.  This function takes that information,
 * combines it with the hardware parameters provided, and programs the
 * hardware accordingly.
 */
static void cs42888_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int ret;
	u8 val;

	/* Mute all the channels */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val = snd_soc_read(codec, CS42888_MUTE);
		val |= CS42888_MUTE_ALL;
		ret = snd_soc_write(codec, CS42888_MUTE, val);
		if (ret < 0)
			pr_err("i2c write failed\n");
	}

}

static int cs42888_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *tmp_codec_dai;
	struct snd_soc_pcm_runtime *tmp_rtd;
	u32 i;

	for (i = 0; i < card->num_rtd; i++) {
		tmp_codec_dai = card->rtd[i].codec_dai;
		tmp_rtd = (struct snd_soc_pcm_runtime *)(card->rtd + i);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
			tmp_codec_dai->pop_wait) {
			tmp_codec_dai->pop_wait = 0;
			cancel_delayed_work(&tmp_rtd->delayed_work);
		}
	}
	return 0;
}

static struct snd_soc_dai_ops cs42888_dai_ops = {
	.set_fmt	= cs42888_set_dai_fmt,
	.set_sysclk	= cs42888_set_dai_sysclk,
	.hw_params	= cs42888_hw_params,
	.shutdown	= cs42888_shutdown,
	.prepare	= cs42888_prepare,
};


struct snd_soc_dai_driver cs42888_dai[] = {
	{
	.name = "CS42888",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = (SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000),
		.formats = CS42888_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = (SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000),
		.formats = CS42888_FORMATS,
	},
	.ops = &cs42888_dai_ops,
	},
	{
		.name = "CS42888_ASRC",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = CS42888_FORMATS,
		},
		.ops = &cs42888_dai_ops,
	},
};

/**
 * cs42888_probe - ASoC probe function
 * @pdev: platform device
 *
 * This function is called when ASoC has all the pieces it needs to
 * instantiate a sound driver.
 */
static int cs42888_probe(struct snd_soc_codec *codec)
{
	int ret, i;
	int val;

	struct cs42888_private *cs42888 = snd_soc_codec_get_drvdata(codec);
	cs42888->codec = codec;

	/* setup i2c data ops */
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	if (cpu_is_mx6q() || cpu_is_mx6dl()) {
		for (i = 0; i < ARRAY_SIZE(cs42888->supplies); i++)
			cs42888->supplies[i].supply = cs42888_supply_names[i];

		ret = regulator_bulk_get(codec->dev,
			ARRAY_SIZE(cs42888->supplies), cs42888->supplies);
		if (ret != 0) {
			dev_err(codec->dev, "Failed to request supplies: %d\n",
				ret);
			return ret;
		}

		ret = regulator_bulk_enable(ARRAY_SIZE(cs42888->supplies),
				cs42888->supplies);
		if (ret != 0) {
			dev_err(codec->dev, "Failed to enable supplies: %d\n",
				ret);
			goto err;
		}
	}
	msleep(1);

	/* The I2C interface is set up, so pre-fill our register cache */
	ret = cs42888_fill_cache(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to fill register cache\n");
		goto err;
	}

	/* Enter low power state */
	val = snd_soc_read(codec, CS42888_PWRCTL);
	val |= CS42888_PWRCTL_PDN_MASK;
	ret = snd_soc_write(codec, CS42888_PWRCTL, val);
	if (ret < 0) {
		dev_err(codec->dev, "i2c write failed\n");
		return ret;
	}

	/* Disable auto-mute */
	val = snd_soc_read(codec, CS42888_TRANS);
	val &= ~CS42888_TRANS_AMUTE_MASK;
	val &= ~CS42888_TRANS_DAC_SZC_MASK;
	val |=  CS42888_TRANS_DAC_SZC_SR;
	ret = snd_soc_write(codec, CS42888_TRANS, val);
	if (ret < 0) {
		pr_err("i2c write failed\n");
		return ret;
	}
	/* Add the non-DAPM controls */
	snd_soc_add_controls(codec, cs42888_snd_controls,
				ARRAY_SIZE(cs42888_snd_controls));

	/* Add DAPM controls */
	cs42888_add_widgets(codec);

	return 0;
err:
	regulator_bulk_disable(ARRAY_SIZE(cs42888->supplies),
						cs42888->supplies);
	regulator_bulk_free(ARRAY_SIZE(cs42888->supplies),
				cs42888->supplies);
	return ret;
}

/**
 * cs42888_remove - ASoC remove function
 * @pdev: platform device
 *
 * This function is the counterpart to cs42888_probe().
 */
static int cs42888_remove(struct snd_soc_codec *codec)
{
	struct cs42888_private *cs42888 = snd_soc_codec_get_drvdata(codec);

	regulator_bulk_disable(ARRAY_SIZE(cs42888->supplies),
						cs42888->supplies);
	regulator_bulk_free(ARRAY_SIZE(cs42888->supplies),
				cs42888->supplies);

	return 0;
};

/*
 * ASoC codec device structure
 *
 * Assign this variable to the codec_dev field of the machine driver's
 * snd_soc_device structure.
 */
struct snd_soc_codec_driver cs42888_driver = {
	.probe =	cs42888_probe,
	.remove =	cs42888_remove,
	.reg_cache_size = CS42888_NUMREGS + 1,
	.reg_word_size = sizeof(u8),
	.reg_cache_step = 1,
};

/**
 * cs42888_i2c_probe - initialize the I2C interface of the CS42888
 * @i2c_client: the I2C client object
 * @id: the I2C device ID (ignored)
 *
 * This function is called whenever the I2C subsystem finds a device that
 * matches the device ID given via a prior call to i2c_add_driver().
 */
static int cs42888_i2c_probe(struct i2c_client *i2c_client,
	const struct i2c_device_id *id)
{
	struct cs42888_private *cs42888;
	int ret;
	int val;

	/* Verify that we have a CS42888 */
	val = i2c_smbus_read_byte_data(i2c_client, CS42888_CHIPID);
	if (val < 0) {
		pr_err("Device with ID register %x is not a CS42888", val);
		return -ENODEV;
	}
	/* The top four bits of the chip ID should be 0000. */
	if ((val & CS42888_CHIPID_ID_MASK) != 0x00) {
		dev_err(&i2c_client->dev, "device is not a CS42888\n");
		return -ENODEV;
	}

	dev_info(&i2c_client->dev, "found device at i2c address %X\n",
		i2c_client->addr);
	dev_info(&i2c_client->dev, "hardware revision %X\n", val & 0xF);

	/* Allocate enough space for the snd_soc_codec structure
	   and our private data together. */
	cs42888 = kzalloc(sizeof(struct cs42888_private), GFP_KERNEL);
	if (!cs42888) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	if (i2c_client->dev.platform_data) {
		memcpy(&cs42888->pdata, i2c_client->dev.platform_data,
				sizeof(cs42888->pdata));
		cs42888_dai[0].playback.rates = cs42888->pdata.rates;
		cs42888_dai[0].capture.rates = cs42888->pdata.rates;
	}

	i2c_set_clientdata(i2c_client, cs42888);

	ret = snd_soc_register_codec(&i2c_client->dev,
		&cs42888_driver, cs42888_dai, 2);
	if (ret) {
		dev_err(&i2c_client->dev, "Failed to register codec:%d\n", ret);
		kfree(cs42888);
	return ret;
	}
	return 0;
}

/**
 * cs42888_i2c_remove - remove an I2C device
 * @i2c_client: the I2C client object
 *
 * This function is the counterpart to cs42888_i2c_probe().
 */
static int cs42888_i2c_remove(struct i2c_client *i2c_client)
{
	struct cs42888_private *cs42888 = i2c_get_clientdata(i2c_client);

	snd_soc_unregister_codec(&i2c_client->dev);
	kfree(cs42888);

	return 0;
}

/*
 * cs42888_i2c_id - I2C device IDs supported by this driver
 */
static struct i2c_device_id cs42888_i2c_id[] = {
	{"cs42888", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs42888_i2c_id);

#ifdef CONFIG_PM
/* This suspend/resume implementation can handle both - a simple standby
 * where the codec remains powered, and a full suspend, where the voltage
 * domain the codec is connected to is teared down and/or any other hardware
 * reset condition is asserted.
 *
 * The codec's own power saving features are enabled in the suspend callback,
 * and all registers are written back to the hardware when resuming.
 */

static int cs42888_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cs42888_private *cs42888 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = cs42888->codec;
	int reg = snd_soc_read(codec, CS42888_PWRCTL) | CS42888_PWRCTL_PDN_MASK;

	return snd_soc_write(codec, CS42888_PWRCTL, reg);
}

static int cs42888_i2c_resume(struct i2c_client *client)
{
	struct cs42888_private *cs42888 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = cs42888->codec;
	int reg;

	/* In case the device was put to hard reset during sleep, we need to
	 * wait 500ns here before any I2C communication. */
	ndelay(500);

	/* first restore the entire register cache ... */
	for (reg = CS42888_FIRSTREG; reg <= CS42888_LASTREG; reg++) {
		u8 val = snd_soc_read(codec, reg);

		if (i2c_smbus_write_byte_data(client, reg, val)) {
			dev_err(codec->dev, "i2c write failed\n");
			return -EIO;
		}
	}

	/* ... then disable the power-down bits */
	reg = snd_soc_read(codec, CS42888_PWRCTL);
	reg &= ~CS42888_PWRCTL_PDN_MASK;

	return snd_soc_write(codec, CS42888_PWRCTL, reg);
}
#else
#define cs42888_i2c_suspend	NULL
#define cs42888_i2c_resume	NULL
#endif /* CONFIG_PM */

/*
 * cs42888_i2c_driver - I2C device identification
 *
 * This structure tells the I2C subsystem how to identify and support a
 * given I2C device type.
 */
static struct i2c_driver cs42888_i2c_driver = {
	.driver = {
		.name = "cs42888",
		.owner = THIS_MODULE,
	},
	.probe = cs42888_i2c_probe,
	.remove = cs42888_i2c_remove,
	.suspend = cs42888_i2c_suspend,
	.resume = cs42888_i2c_resume,
	.id_table = cs42888_i2c_id,
};

static int __init cs42888_init(void)
{
	pr_info("Cirrus Logic CS42888 ALSA SoC Codec Driver\n");
	return i2c_add_driver(&cs42888_i2c_driver);
}
module_init(cs42888_init);

static void __exit cs42888_exit(void)
{
	i2c_del_driver(&cs42888_i2c_driver);
}
module_exit(cs42888_exit);

MODULE_AUTHOR("Xu Lionel <R63889@freescale.com>");
MODULE_DESCRIPTION("Cirrus Logic CS42888 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
