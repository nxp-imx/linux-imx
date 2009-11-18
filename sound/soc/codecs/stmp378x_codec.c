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
#include <mach/platform.h>

#include "stmp378x_codec.h"

#define BV_AUDIOIN_ADCVOL_SELECT__MIC 0x00	/* missing define */

#define STMP378X_VERSION	"0.1"
struct stmp378x_codec_priv {
	struct clk *clk;
	struct snd_soc_codec codec;
};

/*
 * ALSA API
 */
static void __iomem *adc_regmap[] = {
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_STAT,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACSRR,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACDEBUG,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_TEST,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_BISTCTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_BISTSTAT0,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_BISTSTAT1,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACLKCTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DATA,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOOUT_VERSION,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_CTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_STAT,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_ADCSRR,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_ADCVOLUME,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_ADCDEBUG,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_ADCVOL,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_MICLINE,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_ANACLKCTRL,
	REGS_AUDIOOUT_BASE + HW_AUDIOIN_DATA,
};

static u16 stmp378x_audio_regs[ADC_REGNUM];

static u8 dac_volumn_control_word[] = {
	0x37, 0x5e, 0x7e, 0x8e,
	0x9e, 0xae, 0xb6, 0xbe,
	0xc6, 0xce, 0xd6, 0xde,
	0xe6, 0xee, 0xf6, 0xfe,
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

static int stmp378x_codec_restore_reg(struct snd_soc_codec *codec,
				      unsigned int reg)
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
	int reg, l, r;
	int i;

	reg = __raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME);

	l = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT) >>
	    BP_AUDIOOUT_DACVOLUME_VOLUME_LEFT;
	r = (reg & BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT) >>
	    BP_AUDIOOUT_DACVOLUME_VOLUME_RIGHT;
	/*Left channel */
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
	/*Right channel */
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
	int reg, l, r;
	int i;

	i = ucontrol->value.integer.value[0];
	l = dac_volumn_control_word[i];
	/*Get dac volume for left channel */
	reg = BF(l, AUDIOOUT_DACVOLUME_VOLUME_LEFT);

	i = ucontrol->value.integer.value[1];
	r = dac_volumn_control_word[i];
	/*Get dac volume for right channel */
	reg = reg | BF(r, AUDIOOUT_DACVOLUME_VOLUME_RIGHT);

	/*Clear left/right dac volume */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_VOLUME_LEFT |
			BM_AUDIOOUT_DACVOLUME_VOLUME_RIGHT,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_CLR);
	__raw_writel(reg, REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_SET);

	return 0;
}

static const char *stmp378x_codec_adc_input_sel[] =
    { "Mic", "Line In 1", "Head Phone", "Line In 2" };

static const char *stmp378x_codec_hp_output_sel[] = { "DAC Out", "Line In 1" };

static const char *stmp378x_codec_adc_3d_sel[] =
    { "Off", "Low", "Medium", "High" };

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
	int ret = 0;

	snd_soc_dapm_new_controls(codec, stmp378x_codec_widgets,
				  ARRAY_SIZE(stmp378x_codec_widgets));

	if (ret) {
		dev_err(codec->dev, "dapm control register failed\n");
		return ret;
	}
	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	if (ret) {
		dev_err(codec->dev, "DAPM route register failed\n");
		return ret;
	}

	return snd_soc_dapm_new_widgets(codec);
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
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int i;
	u32 srr_value = 0;
	u32 src_hold = 0;

	i = get_srr_values(params_rate(params));
	if (i < 0)
		dev_warn(socdev->dev, "%s doesn't support rate %d\n",
		       codec->name, params_rate(params));
	else {
		src_hold = srr_values[i].src_hold;

		srr_value =
		    BF(srr_values[i].basemult, AUDIOOUT_DACSRR_BASEMULT) |
		    BF(srr_values[i].src_int, AUDIOOUT_DACSRR_SRC_INT) |
		    BF(srr_values[i].src_frac, AUDIOOUT_DACSRR_SRC_FRAC) |
		    BF(src_hold, AUDIOOUT_DACSRR_SRC_HOLD);

		if (playback)
			__raw_writel(srr_value,
				     REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACSRR);
		else
			__raw_writel(srr_value,
				     REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCSRR);
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback)
			__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);
		else
			__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
				REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);

		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		if (playback)
			__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
				REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
		else
			__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
				REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);

		break;

	default:
		dev_warn(socdev->dev, "%s doesn't support format %d\n",
		       codec->name, params_format(params));

	}

	return 0;
}

static int stmp378x_codec_dig_mute(struct snd_soc_dai *dai, int mute)
{
	u32 dac_mask = BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
	    BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT;

	if (mute) {
		__raw_writel(dac_mask,
			      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_SET);
		__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
			      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);
		__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
			      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_SET);
	} else {
		__raw_writel(dac_mask,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_CLR);
		__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_CLR);
		__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_CLR);
	}
	return 0;
}

/*
 * Codec initialization
 */
#define VAG_BASE_VALUE  ((1400/2 - 625)/25)
static void stmp378x_codec_dac_set_vag(void)
{
	u32 refctrl_val = __raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL);

	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VAG_VAL);
	refctrl_val &= ~(BM_AUDIOOUT_REFCTRL_VBG_ADJ);
	refctrl_val |= BF(VAG_BASE_VALUE, AUDIOOUT_REFCTRL_VAG_VAL) |
	    BM_AUDIOOUT_REFCTRL_ADJ_VAG |
	    BF(0xF, AUDIOOUT_REFCTRL_ADC_REFVAL) |
	    BM_AUDIOOUT_REFCTRL_ADJ_ADC |
	    BF(0x3, AUDIOOUT_REFCTRL_VBG_ADJ) | BM_AUDIOOUT_REFCTRL_RAISE_REF;

	__raw_writel(refctrl_val, REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL);
}

static void
stmp378x_codec_dac_power_on(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Ungate DAC clocks */
	__raw_writel(BM_AUDIOOUT_CTRL_CLKGATE,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);
	__raw_writel(BM_AUDIOOUT_ANACLKCTRL_CLKGATE,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACLKCTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_AUDIOOUT_CTRL_WORD_LENGTH,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);

	/* Update DAC volume over zero crossings */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_EN_ZCD,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_SET);
	/* Mute DAC */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
		      BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_SET);

	/* Update HP volume over zero crossings */
	__raw_writel(BM_AUDIOOUT_HPVOL_EN_MSTR_ZCD,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);

	/* Prepare powering up HP output */
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);
	__raw_writel(BF(0x2, RTC_PERSISTENT0_SPARE_ANALOG),
		      REGS_RTC_BASE + HW_RTC_PERSISTENT0_SET);
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);
	/* Mute HP output */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);

	/* Mute speaker amp */
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_SET);
}

static void
stmp378x_codec_dac_power_down(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Disable class AB */
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_CLASSAB,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_CLR);

	/* Set hold to ground */
	__raw_writel(BM_AUDIOOUT_ANACTRL_HP_HOLD_GND,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACTRL_SET);

	/* Mute HP output */
	__raw_writel(BM_AUDIOOUT_HPVOL_MUTE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_HPVOL_SET);
	/* Power down HP output */
	__raw_writel(BM_AUDIOOUT_PWRDN_HEADPHONE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);

	/* Mute speaker amp */
	__raw_writel(BM_AUDIOOUT_SPEAKERCTRL_MUTE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_SPEAKERCTRL_SET);
	/* Power down speaker amp */
	__raw_writel(BM_AUDIOOUT_PWRDN_SPEAKER,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);

	/* Mute DAC */
	__raw_writel(BM_AUDIOOUT_DACVOLUME_MUTE_LEFT |
		      BM_AUDIOOUT_DACVOLUME_MUTE_RIGHT,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_DACVOLUME_SET);
	/* Power down DAC */
	__raw_writel(BM_AUDIOOUT_PWRDN_DAC,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);

	/* Gate DAC clocks */
	__raw_writel(BM_AUDIOOUT_ANACLKCTRL_CLKGATE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_ANACLKCTRL_SET);
	__raw_writel(BM_AUDIOOUT_CTRL_CLKGATE,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);
}

static void
stmp378x_codec_adc_power_on(struct stmp378x_codec_priv *stmp378x_adc)
{
	u32 reg;

	/* Ungate ADC clocks */
	__raw_writel(BM_AUDIOIN_CTRL_CLKGATE,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);
	__raw_writel(BM_AUDIOIN_ANACLKCTRL_CLKGATE,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ANACLKCTRL_CLR);

	/* 16 bit word length */
	__raw_writel(BM_AUDIOIN_CTRL_WORD_LENGTH,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);

	/* Unmute ADC channels */
	__raw_writel(BM_AUDIOIN_ADCVOL_MUTE,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_CLR);

	/*
	 * The MUTE_LEFT and MUTE_RIGHT fields need to be cleared.
	 * They aren't presented in the datasheet, so this is hardcode.
	 */
	__raw_writel(0x01000100, REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOLUME_CLR);

	/* Set the Input channel gain 3dB */
	__raw_writel(BM_AUDIOIN_ADCVOL_GAIN_LEFT,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BM_AUDIOIN_ADCVOL_GAIN_RIGHT,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BF(2, AUDIOIN_ADCVOL_GAIN_LEFT),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_SET);
	__raw_writel(BF(2, AUDIOIN_ADCVOL_GAIN_RIGHT),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_SET);

	/* Select default input - Microphone */
	__raw_writel(BM_AUDIOIN_ADCVOL_SELECT_LEFT,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BM_AUDIOIN_ADCVOL_SELECT_RIGHT,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_CLR);
	__raw_writel(BF
		      (BV_AUDIOIN_ADCVOL_SELECT__MIC,
		       AUDIOIN_ADCVOL_SELECT_LEFT),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_SET);
	__raw_writel(BF
		      (BV_AUDIOIN_ADCVOL_SELECT__MIC,
		       AUDIOIN_ADCVOL_SELECT_RIGHT),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_SET);

	/* Supply bias voltage to microphone */
	__raw_writel(BF(2, AUDIOIN_MICLINE_MIC_RESISTOR),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_MICLINE_SET);
	__raw_writel(BM_AUDIOIN_MICLINE_MIC_SELECT,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_MICLINE_SET);

	/* Set max ADC volume */
	reg = __raw_readl(REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOLUME);
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_LEFT;
	reg &= ~BM_AUDIOIN_ADCVOLUME_VOLUME_RIGHT;
	reg |= BF(ADC_VOLUME_MAX, AUDIOIN_ADCVOLUME_VOLUME_LEFT);
	reg |= BF(ADC_VOLUME_MAX, AUDIOIN_ADCVOLUME_VOLUME_RIGHT);
	__raw_writel(reg, REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOLUME);
}

static void
stmp378x_codec_adc_power_down(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Mute ADC channels */
	__raw_writel(BM_AUDIOIN_ADCVOL_MUTE,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ADCVOL_SET);

	/* Power Down ADC */
	__raw_writel(BM_AUDIOOUT_PWRDN_ADC | BM_AUDIOOUT_PWRDN_RIGHT_ADC,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_PWRDN_SET);

	/* Gate ADC clocks */
	__raw_writel(BM_AUDIOIN_CTRL_CLKGATE,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);
	__raw_writel(BM_AUDIOIN_ANACLKCTRL_CLKGATE,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_ANACLKCTRL_SET);

	/* Disable bias voltage to microphone */
	__raw_writel(BF(0, AUDIOIN_MICLINE_MIC_RESISTOR),
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_MICLINE_SET);
}

static void stmp378x_codec_dac_enable(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Move DAC codec out of reset */
	__raw_writel(BM_AUDIOOUT_CTRL_SFTRST,
		REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_CLR);

	/* Reduce analog power */
	__raw_writel(BM_AUDIOOUT_TEST_HP_I1_ADJ,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_TEST_CLR);
	__raw_writel(BF(0x1, AUDIOOUT_TEST_HP_I1_ADJ),
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_TEST_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_LOW_PWR,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_XTAL_BGR_BIAS,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL_SET);
	__raw_writel(BM_AUDIOOUT_REFCTRL_BIAS_CTRL,
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL_CLR);
	__raw_writel(BF(0x1, AUDIOOUT_REFCTRL_BIAS_CTRL),
			REGS_AUDIOOUT_BASE + HW_AUDIOOUT_REFCTRL_CLR);

	/* Set Vag value */
	stmp378x_codec_dac_set_vag();

	/* Power on DAC codec */
	stmp378x_codec_dac_power_on(stmp378x_adc);
}

static void stmp378x_codec_dac_disable(struct stmp378x_codec_priv *stmp378x_adc)
{
	stmp378x_codec_dac_power_down(stmp378x_adc);
}

static void stmp378x_codec_adc_enable(struct stmp378x_codec_priv *stmp378x_adc)
{
	/* Move ADC codec out of reset */
	__raw_writel(BM_AUDIOIN_CTRL_SFTRST,
			REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_CLR);

	/* Power on ADC codec */
	stmp378x_codec_adc_power_on(stmp378x_adc);
}

static void stmp378x_codec_adc_disable(struct stmp378x_codec_priv *stmp378x_adc)
{
	stmp378x_codec_adc_power_down(stmp378x_adc);
}

static void stmp378x_codec_startup(struct snd_soc_codec *codec)
{
	struct stmp378x_codec_priv *stmp378x_adc = codec->private_data;

	/* Soft reset DAC block */
	__raw_writel(BM_AUDIOOUT_CTRL_SFTRST,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);
	while (!(__raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL) &
		BM_AUDIOOUT_CTRL_CLKGATE));

	/* Soft reset ADC block */
	__raw_writel(BM_AUDIOIN_CTRL_SFTRST,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);
	while (!(__raw_readl(REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL) &
		BM_AUDIOIN_CTRL_CLKGATE)) ;

	stmp378x_codec_dac_enable(stmp378x_adc);
	stmp378x_codec_adc_enable(stmp378x_adc);

	/*Sync regs and cache */
	stmp378x_codec_sync_reg_cache(codec);

	snd_soc_add_controls(codec, stmp378x_snd_controls,
				ARRAY_SIZE(stmp378x_snd_controls));

	stmp378x_codec_add_widgets(codec);
}

static void stmp378x_codec_stop(struct snd_soc_codec *codec)
{
	struct stmp378x_codec_priv *stmp378x_adc = codec->private_data;
	stmp378x_codec_dac_disable(stmp378x_adc);
	stmp378x_codec_adc_disable(stmp378x_adc);
}

#define STMP378X_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define STMP378X_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops stmp378x_dai_ops = {
	.hw_params = stmp378x_codec_hw_params,
	.digital_mute = stmp378x_codec_dig_mute,
};

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
	.ops = &stmp378x_dai_ops,
};

EXPORT_SYMBOL_GPL(stmp378x_codec_dai);

static struct snd_soc_codec *stmp378x_codec;

static int stmp378x_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	socdev->card->codec = stmp378x_codec;
	codec = stmp378x_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

	stmp378x_codec_startup(codec);

	/* Register the socdev */
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(codec->dev, "failed to register card\n");
		snd_soc_dapm_free(socdev);
		snd_soc_free_pcms(socdev);
		return ret;
	}

	return 0;
}

static int stmp378x_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	stmp378x_codec_stop(codec);

	snd_soc_dapm_free(socdev);
	snd_soc_free_pcms(socdev);

	return 0;
}

#ifdef CONFIG_PM
static int stmp378x_codec_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
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
	struct snd_soc_codec *codec = socdev->card->codec;
	struct stmp378x_codec_priv *stmp378x_adc;
	int ret = -EINVAL;

	if (codec == NULL)
		goto out;

	stmp378x_adc = codec->private_data;
	clk_enable(stmp378x_adc->clk);

	/* Soft reset DAC block */
	__raw_writel(BM_AUDIOOUT_CTRL_SFTRST,
		      REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL_SET);
	while (!
	       (__raw_readl(REGS_AUDIOOUT_BASE + HW_AUDIOOUT_CTRL) &
		BM_AUDIOOUT_CTRL_CLKGATE)) ;

	/* Soft reset ADC block */
	__raw_writel(BM_AUDIOIN_CTRL_SFTRST,
		      REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL_SET);
	while (!
	       (__raw_readl(REGS_AUDIOIN_BASE + HW_AUDIOIN_CTRL) &
		BM_AUDIOIN_CTRL_CLKGATE)) ;

	stmp378x_codec_dac_enable(stmp378x_adc);
	stmp378x_codec_adc_enable(stmp378x_adc);

	/*restore registers relevant to amixer controls */
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
	.probe = stmp378x_codec_probe,
	.remove = stmp378x_codec_remove,
	.suspend = stmp378x_codec_suspend,
	.resume = stmp378x_codec_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_stmp378x);

/* codec register, unregister function */
static int __init stmp378x_audio_probe(struct platform_device *pdev)
{
	struct stmp378x_codec_priv *stmp378x_adc;
	struct snd_soc_codec *codec;
	int ret = 0;

	dev_info(&pdev->dev,
		"STMP378X ADC/DAC Audio Codec %s\n", STMP378X_VERSION);

	stmp378x_adc = kzalloc(sizeof(struct stmp378x_codec_priv), GFP_KERNEL);
	if (stmp378x_adc == NULL)
		return -ENOMEM;

	codec = &stmp378x_adc->codec;
	codec->dev = &pdev->dev;
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

	platform_set_drvdata(pdev, stmp378x_adc);

	stmp378x_codec = codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* Turn on audio clock */
	stmp378x_adc->clk = clk_get(&pdev->dev, "audio");
	if (IS_ERR(stmp378x_adc->clk)) {
		ret = PTR_ERR(stmp378x_adc->clk);
		dev_err(&pdev->dev, "%s: Clocks initialization failed\n", __func__);
		goto clk_err;
	}
	clk_enable(stmp378x_adc->clk);

	ret = snd_soc_register_codec(codec);
	if (ret) {
		dev_err(&pdev->dev, "failed to register card\n");
		goto card_err;
	}

	ret = snd_soc_register_dai(&stmp378x_codec_dai);
	if (ret) {
		dev_err(&pdev->dev, "failed to register codec dai\n");
		goto dai_err;
	}

	return 0;

dai_err:
	snd_soc_unregister_codec(codec);
card_err:
	clk_disable(stmp378x_adc->clk);
	clk_put(stmp378x_adc->clk);
clk_err:
	kfree(stmp378x_adc);
	return ret;
}

static int __devexit stmp378x_audio_remove(struct platform_device *pdev)
{
	struct stmp378x_codec_priv *stmp378x_adc = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = &stmp378x_adc->codec;

	snd_soc_unregister_codec(codec);

	clk_disable(stmp378x_adc->clk);
	clk_put(stmp378x_adc->clk);

	kfree(stmp378x_adc);

	return 0;
}

struct platform_driver stmp378x_audio_driver = {
	.driver		= {
		.name = "stmp378x-audio",
	},
	.probe		= stmp378x_audio_probe,
	.remove		= __devexit_p(stmp378x_audio_remove),
};

static int __init stmp378x_codec_init(void)
{
	return platform_driver_register(&stmp378x_audio_driver);
}

static void __exit stmp378x_codec_exit(void)
{
	return platform_driver_unregister(&stmp378x_audio_driver);
}

module_init(stmp378x_codec_init);
module_exit(stmp378x_codec_exit);

MODULE_DESCRIPTION("STMP378X ADC/DAC codec");
MODULE_AUTHOR("Vladislav Buzov");
MODULE_LICENSE("GPL");
