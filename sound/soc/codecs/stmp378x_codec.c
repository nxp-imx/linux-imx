/*
 * ALSA codec for Freescale STMP378X ADC/DAC
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <asm/dma.h>

#include <mach/regs-apbx.h>
#include <mach/regs-audioin.h>
#include <mach/regs-audioout.h>
#include <mach/regs-rtc.h>

#include "stmp378x_codec.h"

#define BV_AUDIOIN_ADCVOL_SELECT__MIC 0x00	/* missing define */

#define STMP378X_VERSION	"0.1"
struct stmp378x_codec_priv {
	struct device *dev;
	struct clk *clk;
};

/*
 * ALSA API
 */
static u32 adc_regmap[] = {
	HW_AUDIOOUT_CTRL_ADDR,
	HW_AUDIOOUT_STAT_ADDR,
	HW_AUDIOOUT_DACSRR_ADDR,
	HW_AUDIOOUT_DACVOLUME_ADDR,
	HW_AUDIOOUT_DACDEBUG_ADDR,
	HW_AUDIOOUT_HPVOL_ADDR,
	HW_AUDIOOUT_PWRDN_ADDR,
	HW_AUDIOOUT_REFCTRL_ADDR,
	HW_AUDIOOUT_ANACTRL_ADDR,
	HW_AUDIOOUT_TEST_ADDR,
	HW_AUDIOOUT_BISTCTRL_ADDR,
	HW_AUDIOOUT_BISTSTAT0_ADDR,
	HW_AUDIOOUT_BISTSTAT1_ADDR,
	HW_AUDIOOUT_ANACLKCTRL_ADDR,
	HW_AUDIOOUT_DATA_ADDR,
	HW_AUDIOOUT_SPEAKERCTRL_ADDR,
	HW_AUDIOOUT_VERSION_ADDR,
	HW_AUDIOIN_CTRL_ADDR,
	HW_AUDIOIN_STAT_ADDR,
	HW_AUDIOIN_ADCSRR_ADDR,
	HW_AUDIOIN_ADCVOLUME_ADDR,
	HW_AUDIOIN_ADCDEBUG_ADDR,
	HW_AUDIOIN_ADCVOL_ADDR,
	HW_AUDIOIN_MICLINE_ADDR,
	HW_AUDIOIN_ANACLKCTRL_ADDR,
	HW_AUDIOIN_DATA_ADDR,
};

static u16 stmp378x_audio_regs[ADC_REGNUM];

static u8 dac_volumn_control_word[] = {
		0x37,  0x5e,  0x7e,  0x8e,
		0x9e,  0xae,  0xb6,  0xbe,
		0xc6,  0xce,  0xd6,  0xde,
		0xe6,  0xee,  0xf6,  0xfe,
};

/*
 * ALSA core supports only 16 bit registers. It means we have to simulate it
 * by virtually splitting a 32bit ADC/DAC registers into two halves
 * high (bits 31:16) and low (bits 15:0). The routins abow detects which part
 * of 32bit register is accessed.
 */
static void stmp378x_codec_write_cache(struct snd_soc_codec *codec,
					unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg < ADC_REGNUM)
		cache[reg] = value;
}

static int stmp378x_codec_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	unsigned int reg_val;
	unsigned int mask = 0xffff;

	if (reg >= ADC_REGNUM)
		return -EIO;

	stmp378x_codec_write_cache(codec, reg, value);

	if (reg & 0x1) {
		mask <<= 16;
		value <<= 16;
	}

	reg_val = __raw_readl(adc_regmap[reg >> 1]);
	reg_val = (reg_val & ~mask) | value;
	__raw_writel(reg_val, adc_regmap[reg >> 1]);

	return 0;
}

static unsigned int stmp378x_codec_read(struct snd_soc_codec *codec,
					unsigned int reg)
{
	unsigned int reg_val;

	if (reg >= ADC_REGNUM)
		return -1;

	reg_val = __raw_readl(adc_regmap[reg >> 1]);
	if (reg & 1)
		reg_val >>= 16;

	return reg_val & 0xffff;
}

static unsigned int stmp378x_codec_read_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg >= ADC_REGNUM)
		return -EINVAL;
	return cache[reg];
}

static void stmp378x_codec_sync_reg_cache(struct snd_soc_codec *codec)
{
	int reg;
	for (reg = 0; reg < ADC_REGNUM; reg += 1)
		stmp378x_codec_write_cache(codec, reg,
					   stmp378x_codec_read(codec, reg));
}


static int stmp378x_codec_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int cached_val, hw_val;

	cached_val = stmp378x_codec_read_cache(codec, reg);
	hw_val = stmp378x_codec_read(codec, reg);

	if (hw_val != cached_val)
		return stmp378x_codec_write(codec, reg, cached_val);

	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xf;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;
	int i;

	reg = HW_AUDIOOUT_DACVOLUME_RD();

	l = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT) >>
			BP_AUDIOOUT_DACVOLUME_VOLUME_LEFT;
	r = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT) >>
			BP_AUDIOOUT_DACVOLUME_VOLUME_RIGHT;
	/*Left channel*/
	i = 0;
	while (i < 16) {
		if (l == dac_volumn_control_word[i]) {
			ucontrol->value.integer.value[0] = i;
			break;
		}
		i++;
	}
	if (i == 16)
		ucontrol->value.integer.value[0] = i;
	/*Right channel*/
	i = 0;
	while (i < 16) {
		if (r == dac_volumn_control_word[i]) {
			ucontrol->value.integer.value[1] = i;
			break;
		}
		i++;
	}
	if (i == 16)
		ucontrol->value.integer.value[1] = i;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;
	int i;

	i = ucontrol->value.integer.value[0];
	l = dac_volumn_control_word[i];
	/*Get dac volume for left channel*/
	reg = BF_AUDIOOUT_DACVOLUME_VOLUME_LEFT(l);

	i = ucontrol->value.integer.value[1];
	r = dac_volumn_control_word[i];
	/*Get dac volume for right channel*/
	reg = reg | BF_AUDIOOUT_DACVOLUME_VOLUME_RIGHT(r);

	/*Clear left/right dac volume*/
	HW_AUDIOOUT_DACVOLUME_CLR(BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT |
					BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT);
	HW_AUDIOOUT_DACVOLUME_SET(reg);
}

static const char *stmp378x_codec_adc_input_sel[] =
				{"Mic", "Line In 1", "Head Phone", "Line In 2"};

static const char *stmp378x_codec_hp_output_sel[] =
				{"DAC Out", "Line In 1"};

static const char *stmp378x_codec_adc_3d_sel[] =
				{"Off", "Low", "Medium", "High"};

static const struct soc_enum stmp378x_codec_enum[] = {
	SOC_ENUM_SINGLE(ADC_ADCVOL_L, 12, 4, stmp378x_codec_adc_input_sel),
	SOC_ENUM_SINGLE(ADC_ADCVOL_L, 4, 4, stmp378x_codec_adc_input_sel),
	SOC_ENUM_SINGLE(DAC_HPVOL_H, 0, 2, stmp378x_codec_hp_output_sel),
	SOC_ENUM_SINGLE(DAC_CTRL_L, 8, 4, stmp378x_codec_adc_3d_sel),
};

/* Codec controls */
static const struct snd_kcontrol_new stmp378x_snd_controls[] = {
	/* Playback Volume */
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "DAC Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	},

	SOC_DOUBLE_R("DAC Playback Switch",
			DAC_VOLUME_H, DAC_VOLUME_L, 8, 0x01, 1),
	SOC_DOUBLE("HP Playback Volume", DAC_HPVOL_L, 8, 0, 0x7F, 1),
	SOC_SINGLE("HP Playback Switch", DAC_HPVOL_H, 8, 0x1, 1),
	SOC_SINGLE("Speaker Playback Switch", DAC_SPEAKERCTRL_H, 8, 0x1, 1),

	/* Capture Volume */
	SOC_DOUBLE_R("ADC Capture Volume",
			ADC_VOLUME_H, ADC_VOLUME_L, 0, 0xFF, 0),
	SOC_DOUBLE("ADC PGA Capture Volume", ADC_ADCVOL_L, 8, 0, 0x0F, 0),
	SOC_SINGLE("ADC PGA Capture Switch", ADC_ADCVOL_H, 8, 0x1, 1),
	SOC_SINGLE("Mic PGA Capture Volume", ADC_MICLINE_L, 0, 0x03, 0),

	/* Virtual 3D effect */
	SOC_ENUM("3D effect", stmp378x_codec_enum[3]),
};

/* add non dapm controls */
static int stmp378x_codec_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(stmp378x_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&stmp378x_snd_controls[i],
				  codec, NULL));
	       if (err < 0)
		       return err;
	}

	return 0;
}

/* Left ADC Mux */
static const struct snd_kcontrol_new stmp378x_left_adc_controls =
SOC_DAPM_ENUM("Route", stmp378x_codec_enum[0]);

/* Right ADC Mux */
static const struct snd_kcontrol_new stmp378x_right_adc_controls =
SOC_DAPM_ENUM("Route", stmp378x_codec_enum[1]);

/* Head Phone Mux */
static const struct snd_kcontrol_new stmp378x_hp_controls =
SOC_DAPM_ENUM("Route", stmp378x_codec_enum[2]);

static const struct snd_soc_dapm_widget stmp378x_codec_widgets[] = {

	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", DAC_PWRDN_L, 8, 1),
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", DAC_PWRDN_H, 0, 1),

	SND_SOC_DAPM_DAC("DAC", "Playback", DAC_PWRDN_L, 12, 1),

	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
			 &stmp378x_left_adc_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
			 &stmp378x_right_adc_controls),
	SND_SOC_DAPM_MUX("HP Mux", SND_SOC_NOPM, 0, 0,
			 &stmp378x_hp_controls),

	SND_SOC_DAPM_PGA("HP_AMP", DAC_PWRDN_L, 0, 1, NULL, 0),

	SND_SOC_DAPM_PGA("HP_CAPLESS", DAC_PWRDN_L, 4, 1, NULL, 0),

	SND_SOC_DAPM_PGA("SPK_AMP", DAC_PWRDN_H, 8, 1, NULL, 0),

	SND_SOC_DAPM_INPUT("LINE1L"),
	SND_SOC_DAPM_INPUT("LINE1R"),
	SND_SOC_DAPM_INPUT("LINE2L"),
	SND_SOC_DAPM_INPUT("LINE2R"),
	SND_SOC_DAPM_INPUT("MIC"),

	SND_SOC_DAPM_OUTPUT("SPEAKER"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
};
static const struct snd_soc_dapm_route intercon[] = {

	/* Left ADC Mux */
	{"Left ADC Mux", "Mic", "MIC"},
	{"Left ADC Mux", "Line In 1", "LINE1L"},
	{"Left ADC Mux", "Line In 2", "LINE2L"},
	{"Left ADC Mux", "Head Phone", "HPL"},

	/* Right ADC Mux */
	{"Right ADC Mux", "Mic", "MIC"},
	{"Right ADC Mux", "Line In 1", "LINE1R"},
	{"Right ADC Mux", "Line In 2", "LINE2R"},
	{"Right ADC Mux", "Head Phone", "HPR"},

	/* ADC */
	{"Left ADC", NULL, "Left ADC Mux"},
	{"Right ADC", NULL, "Right ADC Mux"},

	/* HP Mux */
	{"HP Mux", "DAC Out", "DAC"},
	{"HP Mux", "Line In 1", "LINE1L"},
	{"HP Mux", "Line In 1", "LINE1R"},

	/* HP output */
	{"HP_CAPLESS", NULL, "HP Mux"},
	{"HP_AMP", NULL, "HP_CAPLESS"},
	{"HPR", NULL, "HP_AMP"},
	{"HPL", NULL, "HP_AMP"},

	/* Speaker amp */
	{"SPK_AMP", NULL, "DAC"},
	{"SPEAKER", NULL, "SPK_AMP"},
};

static int stmp378x_codec_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, stmp378x_codec_widgets, 
		ARRAY_SIZE(stmp378x_codec_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct dac_srr {
	u32 rate;
	u32 basemult;
	u32 src_hold;
	u32 src_int;
	u32 src_frac;
};

static struct dac_srr srr_values[] = {
	{192000, 0x4, 0x0, 0x0F, 0x13FF},
	{176400, 0x4, 0x0, 0x11, 0x0037},
	{128000, 0x4, 0x0, 0x17, 0x0E00},
	{96000, 0x2, 0x0, 0x0F, 0x13FF},
	{88200, 0x2, 0x0, 0x11, 0x0037},
	{64000, 0x2, 0x0, 0x17, 0x0E00},
	{48000, 0x1, 0x0, 0x0F, 0x13FF},
	{44100, 0x1, 0x0, 0x11, 0x0037},
	{32000, 0x1, 0x0, 0x17, 0x0E00},
	{24000, 0x1, 0x1, 0x0F, 0x13FF},
	{22050, 0x1, 0x1, 0x11, 0x0037},
	{16000, 0x1, 0x1, 0x17, 0x0E00},
	{12000, 0x1, 0x3, 0x0F, 0x13FF},
	{11025, 0x1, 0x3, 0x11, 0x0037},
	{8000, 0x1, 0x3, 0x17, 0x0E00}
};

static inline int get_srr_values(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(srr_values); i++)
		if (srr_values[i].rate == rate)
			return i;

	return -1;
}

static int stmp378x_codec_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int i;
	u32 srr_value = 0;
	u32 src_hold = 0;

	i = get_srr_values(params_rate(params));
	if (i < 0)
		printk(KERN_WARNING "%s doesn't support rate %d\n",
		       codec->name, params_rate(params));
	else {
		src_hold = srr_values[i].src_hold;

		srr_value =
			BF_AUDIOOUT_DACSRR_BASEMULT(srr_values[i].basemult) |
			BF_AUDIOOUT_DACSRR_SRC_INT(srr_values[i].src_int) |
			BF_AUDIOOUT_DACSRR_SRC_FRAC(srr_values[i].src_frac) |
			BF_AUDIOOUT_DACSRR_SRC_HOLD(src_hold);

		if (playback)
			HW_AUDIOOUT_DACSRR_WR(srr_value);
		else
			HW_AUDIOIN_ADCSRR_WR(srr_value);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_WORD_LENGTH);
		else
			HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_WORD_LENGTH);

		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_WORD_LENGTH);
		else
			HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_WORD_LENGTH);

		break;

	default:
		printk(KERN_WARNING "%s doesn't support format %d\n",
		       codec->name, params_format(params));

	}

	return 0;
}

static int stmp378x_codec_dig_mute(struct snd_soc_dai *dai, int mute)
{
	u32 dac_mask = BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
		BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT;

	if (mute) {
		HW_AUDIOOUT_DACVOLUME_SET(dac_mask);
		HW_AUDIOOUT_HPVOL_SET(BM_AUDIOOUT_HPVOL_MUTE);
		HW_AUDIOOUT_SPEAKERCTRL_SET(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
	} else {
		HW_AUDIOOUT_DACVOLUME_CLR(dac_mask);
		HW_AUDIOOUT_HPVOL_CLR(BM_AUDIOOUT_HPVOL_MUTE);
		HW_AUDIOOUT_SPEAKERCTRL_CLR(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
	}
	return 0;
}

/*
 * Codec initialization
 */
#define VAG_BASE_VALUE  ((1400/2 - 625)/25)
static void stmp378x_codec_dac_set_vag(void)
{
	u32 refctrl_val = HW_AUDIOOUT_REFCTRL_RD();

	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VAG_VAL);
	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VBG_ADJ);
	refctrl_val |= BF_AUDIOOUT_REFCTRL_VAG_VAL(VAG_BASE_VALUE) |
		       BM_AUDIOOUT_REFCTRL_ADJ_VAG |
		       BF_AUDIOOUT_REFCTRL_ADC_REFVAL(0xF) |
		       BM_AUDIOOUT_REFCTRL_ADJ_ADC |
		       BF_AUDIOOUT_REFCTRL_VBG_ADJ(0x3) |
		       BM_AUDIOOUT_REFCTRL_RAISE_REF;

	HW_AUDIOOUT_REFCTRL_WR(refctrl_val);
}

static void
stmp378x_codec_dac_power_on(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Ungate DAC clocks */
	HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_CLKGATE);
	HW_AUDIOOUT_ANACLKCTRL_CLR(BM_AUDIOOUT_ANACLKCTRL_CLKGATE);

	/* 16 bit word length */
	HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_WORD_LENGTH);

	/* Update DAC volume over zero crossings */
	HW_AUDIOOUT_DACVOLUME_SET(BM_AUDIOOUT_DACVOLUME_EN_ZCD);
	/* Mute DAC */
	HW_AUDIOOUT_DACVOLUME_SET(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
				  BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT);

	/* Update HP volume over zero crossings */
	HW_AUDIOOUT_HPVOL_SET(BM_AUDIOOUT_HPVOL_EN_MSTR_ZCD);

	/* Prepare powering up HP output */
	HW_AUDIOOUT_ANACTRL_SET(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND);
	HW_RTC_PERSISTENT0_SET(BF_RTC_PERSISTENT0_SPARE_ANALOG(0x2));
	HW_AUDIOOUT_ANACTRL_SET(BM_AUDIOOUT_ANACTRL_HP_CLASSAB);
	HW_AUDIOOUT_ANACTRL_CLR(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND);
	/* Mute HP output */
	HW_AUDIOOUT_HPVOL_SET(BM_AUDIOOUT_HPVOL_MUTE);

	/* Mute speaker amp */
	HW_AUDIOOUT_SPEAKERCTRL_SET(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
}

static void
stmp378x_codec_dac_power_down(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Disable class AB */
	HW_AUDIOOUT_ANACTRL_CLR(BM_AUDIOOUT_ANACTRL_HP_CLASSAB);

	/* Set hold to ground */
	HW_AUDIOOUT_ANACTRL_SET(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND);

	/* Mute HP output */
	HW_AUDIOOUT_HPVOL_SET(BM_AUDIOOUT_HPVOL_MUTE);
	/* Power down HP output */
	HW_AUDIOOUT_PWRDN_SET(BM_AUDIOOUT_PWRDN_HEADPHONE);

	/* Mute speaker amp */
	HW_AUDIOOUT_SPEAKERCTRL_SET(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
	/* Power down speaker amp */
	HW_AUDIOOUT_PWRDN_SET(BM_AUDIOOUT_PWRDN_SPEAKER);

	/* Mute DAC */
	HW_AUDIOOUT_DACVOLUME_SET(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
				  BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT);
	/* Power down DAC */
	HW_AUDIOOUT_PWRDN_SET(BM_AUDIOOUT_PWRDN_DAC);

	/* Gate DAC clocks */
	HW_AUDIOOUT_ANACLKCTRL_SET(BM_AUDIOOUT_ANACLKCTRL_CLKGATE);
	HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_CLKGATE);
}

static void
stmp378x_codec_adc_power_on(struct stmp378x_codec_priv *stmp378x_adc)
{
	u32 reg;

	/* Ungate ADC clocks */
	HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_CLKGATE);
	HW_AUDIOIN_ANACLKCTRL_CLR(BM_AUDIOIN_ANACLKCTRL_CLKGATE);

	/* 16 bit word length */
	HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_WORD_LENGTH);

	/* Unmute ADC channels */
	HW_AUDIOIN_ADCVOL_CLR(BM_AUDIOIN_ADCVOL_MUTE);

	/*
	 * The MUTE_LEFT and MUTE_RIGHT fields need to be cleared.
	 * They aren't presented in the datasheet, so this is hardcode.
	 */
	HW_AUDIOIN_ADCVOLUME_CLR(0x01000100);

	/* Set the Input channel gain 3dB */
	HW_AUDIOIN_ADCVOL_CLR(BM_AUDIOIN_ADCVOL_GAIN_LEFT);
	HW_AUDIOIN_ADCVOL_CLR(BM_AUDIOIN_ADCVOL_GAIN_RIGHT);
	HW_AUDIOIN_ADCVOL_SET(BF_AUDIOIN_ADCVOL_GAIN_LEFT(2));
	HW_AUDIOIN_ADCVOL_SET(BF_AUDIOIN_ADCVOL_GAIN_RIGHT(2));

	/* Select default input - Microphone */
	HW_AUDIOIN_ADCVOL_CLR(BM_AUDIOIN_ADCVOL_SELECT_LEFT);
	HW_AUDIOIN_ADCVOL_CLR(BM_AUDIOIN_ADCVOL_SELECT_RIGHT);
	HW_AUDIOIN_ADCVOL_SET(
	  BF_AUDIOIN_ADCVOL_SELECT_LEFT(BV_AUDIOIN_ADCVOL_SELECT__MIC));
	HW_AUDIOIN_ADCVOL_SET(
	  BF_AUDIOIN_ADCVOL_SELECT_RIGHT(BV_AUDIOIN_ADCVOL_SELECT__MIC));

	/* Supply bias voltage to microphone */
	HW_AUDIOIN_MICLINE_SET(BF_AUDIOIN_MICLINE_MIC_RESISTOR(2));
	HW_AUDIOIN_MICLINE_SET(BM_AUDIOIN_MICLINE_MIC_SELECT);

	/* Set max ADC volume */
	reg = HW_AUDIOIN_ADCVOLUME_RD();
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;
	reg |= BF_AUDIOIN_ADCVOLUME_VOLUME_LEFT(ADC_VOLUME_MAX);
	reg |= BF_AUDIOIN_ADCVOLUME_VOLUME_RIGHT(ADC_VOLUME_MAX);
	HW_AUDIOIN_ADCVOLUME_WR(reg);
}

static void
stmp378x_codec_adc_power_down(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Mute ADC channels */
	HW_AUDIOIN_ADCVOL_SET(BM_AUDIOIN_ADCVOL_MUTE);

	/* Power Down ADC */
	HW_AUDIOOUT_PWRDN_SET(
		BM_AUDIOOUT_PWRDN_ADC | BM_AUDIOOUT_PWRDN_RIGHT_ADC);

	/* Gate ADC clocks */
	HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_CLKGATE);
	HW_AUDIOIN_ANACLKCTRL_SET(BM_AUDIOIN_ANACLKCTRL_CLKGATE);

	/* Disable bias voltage to microphone*/
	HW_AUDIOIN_MICLINE_SET(BF_AUDIOIN_MICLINE_MIC_RESISTOR(0));
}

static void
stmp378x_codec_dac_enable(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Move DAC codec out of reset */
	HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_SFTRST);

	/* Reduce analog power */
	HW_AUDIOOUT_TEST_CLR(BM_AUDIOOUT_TEST_HP_I1_ADJ);
	HW_AUDIOOUT_TEST_SET(BF_AUDIOOUT_TEST_HP_I1_ADJ(0x1));
	HW_AUDIOOUT_REFCTRL_SET(BM_AUDIOOUT_REFCTRL_LOW_PWR);
	HW_AUDIOOUT_REFCTRL_SET(BM_AUDIOOUT_REFCTRL_XTAL_BGR_BIAS);
	HW_AUDIOOUT_REFCTRL_CLR(BM_AUDIOOUT_REFCTRL_BIAS_CTRL);
	HW_AUDIOOUT_REFCTRL_CLR(BF_AUDIOOUT_REFCTRL_BIAS_CTRL(0x1));

	/* Set Vag value */
	stmp378x_codec_dac_set_vag();

	/* Power on DAC codec */
	stmp378x_codec_dac_power_on(stmp378x_adc);
}

static void stmp378x_codec_dac_disable(struct stmp378x_codec_priv *stmp378x_adc)
{
	stmp378x_codec_dac_power_down(stmp378x_adc);
}

static void
stmp378x_codec_adc_enable(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Move ADC codec out of reset */
	HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_SFTRST);

	/* Power on ADC codec */
	stmp378x_codec_adc_power_on(stmp378x_adc);
}

static void stmp378x_codec_adc_disable(struct stmp378x_codec_priv *stmp378x_adc)
{
	stmp378x_codec_adc_power_down(stmp378x_adc);
}

static void
stmp378x_codec_init(struct snd_soc_codec *codec)
{
	struct stmp378x_codec_priv *stmp378x_adc = codec->private_data;

	/* Soft reset DAC block */
	HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_SFTRST);
	while (!(HW_AUDIOOUT_CTRL_RD() & BM_AUDIOOUT_CTRL_CLKGATE));

	/* Soft reset ADC block */
	HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_SFTRST);
	while (!(HW_AUDIOIN_CTRL_RD() & BM_AUDIOIN_CTRL_CLKGATE));

	stmp378x_codec_dac_enable(stmp378x_adc);
	stmp378x_codec_adc_enable(stmp378x_adc);

	/*Sync regs and cache*/
	stmp378x_codec_sync_reg_cache(codec);

	stmp378x_codec_add_controls(codec);
	stmp378x_codec_add_widgets(codec);
}

static void
stmp378x_codec_exit(struct snd_soc_codec *codec)
{
	struct stmp378x_codec_priv *stmp378x_adc = codec->private_data;
	stmp378x_codec_dac_disable(stmp378x_adc);
	stmp378x_codec_adc_disable(stmp378x_adc);
}

#define STMP378X_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define STMP378X_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)
struct snd_soc_dai stmp378x_codec_dai = {
	.name = "stmp378x adc/dac",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP378X_ADC_RATES,
		.formats = STMP378X_ADC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP378X_ADC_RATES,
		.formats = STMP378X_ADC_FORMATS,
	},
	.ops = {
		.hw_params = stmp378x_codec_hw_params,
	},
	.dai_ops = {
		.digital_mute = stmp378x_codec_dig_mute,
	}
};
EXPORT_SYMBOL_GPL(stmp378x_codec_dai);

static int stmp378x_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct stmp378x_codec_priv *stmp378x_adc;
	int ret = 0;

	printk(KERN_INFO "STMP378X ADC/DAC Audio Codec %s\n", STMP378X_VERSION);

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	stmp378x_adc = kzalloc(sizeof(struct stmp378x_codec_priv), GFP_KERNEL);
	if (stmp378x_adc == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->name = "stmp378x adc/dac";
	codec->owner = THIS_MODULE;
	codec->private_data = stmp378x_adc;
	codec->read = stmp378x_codec_read;
	codec->write = stmp378x_codec_write;
	codec->dai = &stmp378x_codec_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(stmp378x_audio_regs) >> 1;
	codec->reg_cache_step = 1;
	codec->reg_cache = (void *)&stmp378x_audio_regs;
	socdev->codec = codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		goto pcm_err;
	}

	/* Turn on audio clock */
	stmp378x_adc->dev = &pdev->dev;
	stmp378x_adc->clk = clk_get(stmp378x_adc->dev, "audio");
	if (IS_ERR(stmp378x_adc->clk)) {
		ret = PTR_ERR(stmp378x_adc->clk);
		printk(KERN_ERR "%s: Clocks initialization failed\n", __func__);
		goto clk_err;
	}
	clk_enable(stmp378x_adc->clk);

	stmp378x_codec_init(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register card\n", __func__);
		goto card_err;
	}

	return ret;

card_err:
	clk_disable(stmp378x_adc->clk);
	clk_put(stmp378x_adc->clk);
clk_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(socdev->codec);
	return ret;
}

static int stmp378x_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp378x_codec_priv *stmp378x_adc;

	if (codec == NULL)
		return 0;

	stmp378x_adc = codec->private_data;

	clk_disable(stmp378x_adc->clk);
	clk_put(stmp378x_adc->clk);

	stmp378x_codec_exit(codec);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(socdev->codec);

	return 0;
}

#ifdef CONFIG_PM
static int stmp378x_codec_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp378x_codec_priv *stmp378x_adc;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	stmp378x_adc = codec->private_data;

	stmp378x_codec_dac_disable(stmp378x_adc);
	stmp378x_codec_adc_disable(stmp378x_adc);
	clk_disable(stmp378x_adc->clk);
	ret = 0;

out:
	return ret;
}

static int stmp378x_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct stmp378x_codec_priv *stmp378x_adc;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	stmp378x_adc = codec->private_data;
	clk_enable(stmp378x_adc->clk);

	/* Soft reset DAC block */
	HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_SFTRST);
	while (!(HW_AUDIOOUT_CTRL_RD() & BM_AUDIOOUT_CTRL_CLKGATE));

	/* Soft reset ADC block */
	HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_SFTRST);
	while (!(HW_AUDIOIN_CTRL_RD() & BM_AUDIOIN_CTRL_CLKGATE));

	stmp378x_codec_dac_enable(stmp378x_adc);
	stmp378x_codec_adc_enable(stmp378x_adc);

	/*restore registers relevant to amixer controls*/
	stmp378x_codec_restore_reg(codec, DAC_CTRL_L);
	stmp378x_codec_restore_reg(codec, DAC_VOLUME_L);
	stmp378x_codec_restore_reg(codec, DAC_VOLUME_H);
	stmp378x_codec_restore_reg(codec, DAC_HPVOL_L);
	stmp378x_codec_restore_reg(codec, DAC_HPVOL_H);
	stmp378x_codec_restore_reg(codec, DAC_SPEAKERCTRL_H);
	stmp378x_codec_restore_reg(codec, ADC_VOLUME_L);
	stmp378x_codec_restore_reg(codec, ADC_VOLUME_H);
	stmp378x_codec_restore_reg(codec, ADC_ADCVOL_L);
	stmp378x_codec_restore_reg(codec, ADC_ADCVOL_H);
	stmp378x_codec_restore_reg(codec, ADC_MICLINE_L);

	ret = 0;

out:
	return ret;
}
#else
#define stmp378x_codec_suspend	NULL
#define stmp378x_codec_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_codec_device soc_codec_dev_stmp378x = {
	.probe		= stmp378x_codec_probe,
	.remove		= stmp378x_codec_remove,
	.suspend	= stmp378x_codec_suspend,
	.resume		= stmp378x_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_stmp378x);

MODULE_DESCRIPTION("STMP378X ADC/DAC codec");
MODULE_AUTHOR("Vladislav Buzov");
MODULE_LICENSE("GPL");
