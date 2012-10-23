/*
 * MXC HDMI ALSA Soc Codec Driver
 *
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * Some code from patch_hdmi.c
 *  Copyright (c) 2008-2010 Intel Corporation. All rights reserved.
 *  Copyright (c) 2006 ATI Technologies Inc.
 *  Copyright (c) 2008 NVIDIA Corp.  All rights reserved.
 *  Copyright (c) 2008 Wei Ni <wni@nvidia.com>
 */

/*
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/asoundef.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include <linux/mfd/mxc-hdmi-core.h>
#include <mach/mxc_edid.h>
#include <mach/mxc_hdmi.h>
#include "../imx/imx-hdmi.h"

struct mxc_hdmi_priv {
	struct snd_soc_codec codec;
	struct snd_card *card;	/* ALSA HDMI sound card handle */
	struct snd_pcm *pcm;	/* ALSA hdmi driver type handle */
	struct clk *isfr_clk;
	struct clk *iahb_clk;
};

static struct mxc_edid_cfg edid_cfg;

static unsigned int playback_rates[HDMI_MAX_RATES];
static unsigned int playback_sample_size[HDMI_MAX_SAMPLE_SIZE];
static unsigned int playback_channels[HDMI_MAX_CHANNEL_CONSTRAINTS];

static struct snd_pcm_hw_constraint_list playback_constraint_rates;
static struct snd_pcm_hw_constraint_list playback_constraint_bits;
static struct snd_pcm_hw_constraint_list playback_constraint_channels;

#ifdef DEBUG
static void dumpregs(void)
{
	int n, cts;

	cts = (hdmi_readb(HDMI_AUD_CTS3) << 16) |
	      (hdmi_readb(HDMI_AUD_CTS2) << 8) |
	      hdmi_readb(HDMI_AUD_CTS1);

	n = (hdmi_readb(HDMI_AUD_N3) << 16) |
	    (hdmi_readb(HDMI_AUD_N2) << 8) |
	    hdmi_readb(HDMI_AUD_N1);

	pr_debug("\n");
	pr_debug("HDMI_PHY_CONF0      0x%02x\n", hdmi_readb(HDMI_PHY_CONF0));
	pr_debug("HDMI_MC_CLKDIS      0x%02x\n", hdmi_readb(HDMI_MC_CLKDIS));
	pr_debug("HDMI_AUD_N[1-3]     0x%06x (decimal %d)\n", n, n);
	pr_debug("HDMI_AUD_CTS[1-3]   0x%06x (decimal %d)\n", cts, cts);
	pr_debug("HDMI_FC_AUDSCONF    0x%02x\n", hdmi_readb(HDMI_FC_AUDSCONF));
}
#else
static void dumpregs(void) {}
#endif

enum cea_speaker_placement {
	FL  = (1 <<  0),	/* Front Left           */
	FC  = (1 <<  1),	/* Front Center         */
	FR  = (1 <<  2),	/* Front Right          */
	FLC = (1 <<  3),	/* Front Left Center    */
	FRC = (1 <<  4),	/* Front Right Center   */
	RL  = (1 <<  5),	/* Rear Left            */
	RC  = (1 <<  6),	/* Rear Center          */
	RR  = (1 <<  7),	/* Rear Right           */
	RLC = (1 <<  8),	/* Rear Left Center     */
	RRC = (1 <<  9),	/* Rear Right Center    */
	LFE = (1 << 10),	/* Low Frequency Effect */
	FLW = (1 << 11),	/* Front Left Wide      */
	FRW = (1 << 12),	/* Front Right Wide     */
	FLH = (1 << 13),	/* Front Left High      */
	FCH = (1 << 14),	/* Front Center High    */
	FRH = (1 << 15),	/* Front Right High     */
	TC  = (1 << 16),	/* Top Center           */
};

/*
 * EDID SA bits in the CEA Speaker Allocation data block
 */
static int edid_speaker_allocation_bits[] = {
	[0] = FL | FR,
	[1] = LFE,
	[2] = FC,
	[3] = RL | RR,
	[4] = RC,
	[5] = FLC | FRC,
	[6] = RLC | RRC,
	[7] = FLW | FRW,
	[8] = FLH | FRH,
	[9] = TC,
	[10] = FCH,
};

struct cea_channel_speaker_allocation {
	int ca_index;
	int speakers[8];

	/* derived values, just for convenience */
	int channels;
	int spk_mask;
};

/*
 * This is an ordered list!
 *
 * The preceding ones have better chances to be selected by
 * hdmi_channel_allocation().
 */
static struct cea_channel_speaker_allocation channel_allocations[] = {
/*			  channel:   7     6    5    4    3     2    1    0  */
{ .ca_index = 0x00,  .speakers = {   0,    0,   0,   0,   0,    0,  FR,  FL } },
				 /* 2.1 */
{ .ca_index = 0x01,  .speakers = {   0,    0,   0,   0,   0,  LFE,  FR,  FL } },
				 /* Dolby Surround */
{ .ca_index = 0x02,  .speakers = {   0,    0,   0,   0,  FC,    0,  FR,  FL } },
{ .ca_index = 0x03,  .speakers = {   0,    0,   0,   0,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x04,  .speakers = {   0,    0,   0,  RC,   0,    0,  FR,  FL } },
{ .ca_index = 0x05,  .speakers = {   0,    0,   0,  RC,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x06,  .speakers = {   0,    0,   0,  RC,  FC,    0,  FR,  FL } },
{ .ca_index = 0x07,  .speakers = {   0,    0,   0,  RC,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x08,  .speakers = {   0,    0,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x09,  .speakers = {   0,    0,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x0a,  .speakers = {   0,    0,  RR,  RL,  FC,    0,  FR,  FL } },
				 /* surround51 */
{ .ca_index = 0x0b,  .speakers = {   0,    0,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x0c,  .speakers = {   0,   RC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x0d,  .speakers = {   0,   RC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x0e,  .speakers = {   0,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
				 /* 6.1 */
{ .ca_index = 0x0f,  .speakers = {   0,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x10,  .speakers = { RRC,  RLC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x11,  .speakers = { RRC,  RLC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x12,  .speakers = { RRC,  RLC,  RR,  RL,  FC,    0,  FR,  FL } },
				 /* surround71 */
{ .ca_index = 0x13,  .speakers = { RRC,  RLC,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x14,  .speakers = { FRC,  FLC,   0,   0,   0,    0,  FR,  FL } },
{ .ca_index = 0x15,  .speakers = { FRC,  FLC,   0,   0,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x16,  .speakers = { FRC,  FLC,   0,   0,  FC,    0,  FR,  FL } },
{ .ca_index = 0x17,  .speakers = { FRC,  FLC,   0,   0,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x18,  .speakers = { FRC,  FLC,   0,  RC,   0,    0,  FR,  FL } },
{ .ca_index = 0x19,  .speakers = { FRC,  FLC,   0,  RC,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x1a,  .speakers = { FRC,  FLC,   0,  RC,  FC,    0,  FR,  FL } },
{ .ca_index = 0x1b,  .speakers = { FRC,  FLC,   0,  RC,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x1c,  .speakers = { FRC,  FLC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x1d,  .speakers = { FRC,  FLC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x1e,  .speakers = { FRC,  FLC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x1f,  .speakers = { FRC,  FLC,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x20,  .speakers = {   0,  FCH,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x21,  .speakers = {   0,  FCH,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x22,  .speakers = {  TC,    0,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x23,  .speakers = {  TC,    0,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x24,  .speakers = { FRH,  FLH,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x25,  .speakers = { FRH,  FLH,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x26,  .speakers = { FRW,  FLW,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x27,  .speakers = { FRW,  FLW,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x28,  .speakers = {  TC,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x29,  .speakers = {  TC,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x2a,  .speakers = { FCH,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x2b,  .speakers = { FCH,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x2c,  .speakers = {  TC,  FCH,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x2d,  .speakers = {  TC,  FCH,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x2e,  .speakers = { FRH,  FLH,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x2f,  .speakers = { FRH,  FLH,  RR,  RL,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x30,  .speakers = { FRW,  FLW,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x31,  .speakers = { FRW,  FLW,  RR,  RL,  FC,  LFE,  FR,  FL } },
};

/*
 * Compute derived values in channel_allocations[].
 */
static void init_channel_allocations(void)
{
	int i, j;
	struct cea_channel_speaker_allocation *p;

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		p = channel_allocations + i;
		p->channels = 0;
		p->spk_mask = 0;
		for (j = 0; j < ARRAY_SIZE(p->speakers); j++)
			if (p->speakers[j]) {
				p->channels++;
				p->spk_mask |= p->speakers[j];
			}
	}
}

/*
 * The transformation takes two steps:
 *
 * speaker_alloc => (edid_speaker_allocation_bits[]) => spk_mask
 * spk_mask      => (channel_allocations[])         => CA
 *
 * TODO: it could select the wrong CA from multiple candidates.
*/
static int hdmi_channel_allocation(int channels)
{
	int i;
	int ca = 0;
	int spk_mask = 0;

	/*
	 * CA defaults to 0 for basic stereo audio
	 */
	if (channels <= 2)
		return 0;

	/*
	 * expand EDID's speaker allocation mask
	 *
	 * EDID tells the speaker mask in a compact(paired) form,
	 * expand EDID's notions to match the ones used by Audio InfoFrame.
	 */
	for (i = 0; i < ARRAY_SIZE(edid_speaker_allocation_bits); i++) {
		if (edid_cfg.speaker_alloc & (1 << i))
			spk_mask |= edid_speaker_allocation_bits[i];
	}

	/* search for the first working match in the CA table */
	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (channels == channel_allocations[i].channels &&
		    (spk_mask & channel_allocations[i].spk_mask) ==
				channel_allocations[i].spk_mask) {
			ca = channel_allocations[i].ca_index;
			break;
		}
	}

	return ca;
}

static void hdmi_set_audio_flat(u8 value)
{
	/* Indicates the subpacket represents a flatline sample */
	hdmi_mask_writeb(value, HDMI_FC_AUDSCONF,
			 HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_OFFSET,
			 HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_MASK);
}

static void hdmi_set_layout(unsigned int channels)
{
	hdmi_mask_writeb((channels > 2) ? 1 : 0, HDMI_FC_AUDSCONF,
			HDMI_FC_AUDSCONF_AUD_PACKET_LAYOUT_OFFSET,
			HDMI_FC_AUDSCONF_AUD_PACKET_LAYOUT_MASK);
}

static void hdmi_set_audio_infoframe(unsigned int channels)
{
	unsigned char audiconf0, audiconf2;

	/* From CEA-861-D spec:
	 * NOTE: HDMI requires the CT, SS and SF fields to be set to 0 ("Refer
	 * to Stream Header") as these items are carried in the audio stream.
	 *
	 * So we only set the CC and CA fields.
	 */
	audiconf0 = ((channels - 1) << HDMI_FC_AUDICONF0_CC_OFFSET) &
		    HDMI_FC_AUDICONF0_CC_MASK;

	audiconf2 = hdmi_channel_allocation(channels);

	hdmi_writeb(audiconf0, HDMI_FC_AUDICONF0);
	hdmi_writeb(0, HDMI_FC_AUDICONF1);
	hdmi_writeb(audiconf2, HDMI_FC_AUDICONF2);
	hdmi_writeb(0, HDMI_FC_AUDICONF3);
}

static int cea_audio_rates[HDMI_MAX_RATES] = {
	32000,
	44100,
	48000,
	88200,
	96000,
	176400,
	192000,
};

static void mxc_hdmi_get_playback_rates(void)
{
	int i, count = 0;
	u8 rates;

	/* always assume basic audio support */
	rates = edid_cfg.sample_rates | 0x7;

	for (i = 0 ; i < HDMI_MAX_RATES ; i++)
		if ((rates & (1 << i)) != 0)
			playback_rates[count++] = cea_audio_rates[i];

	playback_constraint_rates.list = playback_rates;
	playback_constraint_rates.count = count;

#ifdef DEBUG
	for (i = 0 ; i < playback_constraint_rates.count ; i++)
		pr_debug("%s: constraint = %d Hz\n", __func__,
			 playback_rates[i]);
#endif
}

static void mxc_hdmi_get_playback_sample_size(void)
{
	int i = 0;

	/* always assume basic audio support */
	playback_sample_size[i++] = 16;

	if (edid_cfg.sample_sizes & 0x4)
		playback_sample_size[i++] = 24;

	playback_constraint_bits.list = playback_sample_size;
	playback_constraint_bits.count = i;

#ifdef DEBUG
	for (i = 0 ; i < playback_constraint_bits.count ; i++)
		pr_debug("%s: constraint = %d bits\n", __func__,
			 playback_sample_size[i]);
#endif
}

static void mxc_hdmi_get_playback_channels(void)
{
	int channels = 2, i = 0;

	/* always assume basic audio support */
	playback_channels[i++] = channels;
	channels += 2;

	while ((i < HDMI_MAX_CHANNEL_CONSTRAINTS) &&
		(channels <= edid_cfg.max_channels)) {
		playback_channels[i++] = channels;
		channels += 2;
	}

	playback_constraint_channels.list = playback_channels;
	playback_constraint_channels.count = i;

#ifdef DEBUG
	for (i = 0 ; i < playback_constraint_channels.count ; i++)
		pr_debug("%s: constraint = %d channels\n", __func__,
			 playback_channels[i]);
#endif
}

static int mxc_hdmi_update_constraints(struct mxc_hdmi_priv *priv,
				       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	hdmi_get_edid_cfg(&edid_cfg);

	mxc_hdmi_get_playback_rates();
	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&playback_constraint_rates);
	if (ret < 0)
		return ret;

	mxc_hdmi_get_playback_sample_size();
	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
					&playback_constraint_bits);
	if (ret < 0)
		return ret;

	mxc_hdmi_get_playback_channels();
	ret = snd_pcm_hw_constraint_list(runtime, 0,
					SNDRV_PCM_HW_PARAM_CHANNELS,
					&playback_constraint_channels);
	if (ret < 0)
		return ret;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	return 0;
}

static int mxc_hdmi_codec_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct mxc_hdmi_priv *hdmi_priv = snd_soc_dai_get_drvdata(dai);
	int ret;

	clk_enable(hdmi_priv->isfr_clk);
	clk_enable(hdmi_priv->iahb_clk);

	pr_debug("%s hdmi clks: isfr:%d iahb:%d\n", __func__,
		(int)clk_get_rate(hdmi_priv->isfr_clk),
		(int)clk_get_rate(hdmi_priv->iahb_clk));

	ret = mxc_hdmi_update_constraints(hdmi_priv, substream);
	if (ret < 0)
		return ret;

	hdmi_set_audio_flat(0);

	return 0;
}

static int mxc_hdmi_codec_prepare(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	hdmi_set_audio_infoframe(runtime->channels);
	hdmi_set_layout(runtime->channels);
	hdmi_set_sample_rate(runtime->rate);
	dumpregs();

	return 0;
}

static void mxc_hdmi_codec_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct mxc_hdmi_priv *hdmi_priv = snd_soc_dai_get_drvdata(dai);

	clk_disable(hdmi_priv->iahb_clk);
	clk_disable(hdmi_priv->isfr_clk);
}

/*
 * IEC60958 status functions
 */
static int mxc_hdmi_iec_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int mxc_hdmi_iec_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	int i;

	for (i = 0 ; i < 4 ; i++)
		uvalue->value.iec958.status[i] = iec_header.status[i];

	return 0;
}

static int mxc_hdmi_iec_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	int i;

	/* Do not allow professional mode */
	if (uvalue->value.iec958.status[0] & IEC958_AES0_PROFESSIONAL)
		return -EPERM;

	for (i = 0 ; i < 4 ; i++) {
		iec_header.status[i] = uvalue->value.iec958.status[i];
		pr_debug("%s status[%d]=0x%02x\n", __func__, i,
			 iec_header.status[i]);
	}

	return 0;
}

static int mxc_hdmi_channels_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_channels();

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = playback_constraint_channels.count;

	return 0;
}

static int mxc_hdmi_channels_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	int i;
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_channels();

	for (i = 0 ; i < playback_constraint_channels.count ; i++)
		uvalue->value.integer.value[i] = playback_channels[i];

	return 0;
}

static int mxc_hdmi_rates_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_rates();

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = playback_constraint_rates.count;

	return 0;
}

static int mxc_hdmi_rates_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	int i;
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_rates();

	for (i = 0 ; i < playback_constraint_rates.count ; i++)
		uvalue->value.integer.value[i] = playback_rates[i];

	return 0;
}

static int mxc_hdmi_formats_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_sample_size();

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = playback_constraint_bits.count;

	return 0;
}

static int mxc_hdmi_formats_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	int i;
	hdmi_get_edid_cfg(&edid_cfg);
	mxc_hdmi_get_playback_sample_size();

	for (i = 0 ; i < playback_constraint_bits.count ; i++)
		uvalue->value.integer.value[i] = playback_sample_size[i];

	return 0;
}

static struct snd_kcontrol_new mxc_hdmi_ctrls[] = {
	/* status cchanel controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_hdmi_iec_info,
	 .get = mxc_hdmi_iec_get,
	 .put = mxc_hdmi_iec_put,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "HDMI Support Channels",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_hdmi_channels_info,
	 .get = mxc_hdmi_channels_get,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "HDMI Support Rates",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_hdmi_rates_info,
	 .get = mxc_hdmi_rates_get,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "HDMI Support Formats",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ |
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_hdmi_formats_info,
	 .get = mxc_hdmi_formats_get,
	 },
};

static struct snd_soc_dai_ops mxc_hdmi_codec_dai_ops = {
	.startup = mxc_hdmi_codec_startup,
	.prepare = mxc_hdmi_codec_prepare,
	.shutdown = mxc_hdmi_codec_shutdown,
};

static struct snd_soc_dai_driver mxc_hdmi_codec_dai = {
	.name = "mxc-hdmi-soc",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = MXC_HDMI_RATES_PLAYBACK,
		.formats = MXC_HDMI_FORMATS_PLAYBACK,
	},
	.ops = &mxc_hdmi_codec_dai_ops,
};

static int mxc_hdmi_codec_soc_probe(struct snd_soc_codec *codec)
{
	struct mxc_hdmi_priv *hdmi_priv;
	int ret = 0;

	if (!hdmi_get_registered())
		return -ENOMEM;

	hdmi_priv = kzalloc(sizeof(struct mxc_hdmi_priv), GFP_KERNEL);
	if (hdmi_priv == NULL)
		return -ENOMEM;

	hdmi_priv->isfr_clk = clk_get(NULL, "hdmi_isfr_clk");
	if (IS_ERR(hdmi_priv->isfr_clk)) {
		ret = PTR_ERR(hdmi_priv->isfr_clk);
		pr_err("%s Unable to get HDMI isfr clk: %d\n", __func__, ret);
		goto e_clk_get1;
	}

	hdmi_priv->iahb_clk = clk_get(NULL, "hdmi_iahb_clk");
	if (IS_ERR(hdmi_priv->iahb_clk)) {
		ret = PTR_ERR(hdmi_priv->iahb_clk);
		pr_err("%s Unable to get HDMI iahb clk: %d\n", __func__, ret);
		goto e_clk_get2;
	}

	ret = snd_soc_add_controls(codec, mxc_hdmi_ctrls,
			     ARRAY_SIZE(mxc_hdmi_ctrls));
	if (ret)
		goto e_add_ctrls;

	init_channel_allocations();

	snd_soc_codec_set_drvdata(codec, hdmi_priv);

	return 0;

e_add_ctrls:
	clk_put(hdmi_priv->iahb_clk);
e_clk_get2:
	clk_put(hdmi_priv->isfr_clk);
e_clk_get1:
	kfree(hdmi_priv);
	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_hdmi = {
	.probe = mxc_hdmi_codec_soc_probe,
};

static int __devinit mxc_hdmi_codec_probe(struct platform_device *pdev)
{
	int ret = 0;

	dev_info(&pdev->dev, "MXC HDMI Audio\n");

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_hdmi,
				     &mxc_hdmi_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register codec\n");
		return ret;
	}

	return 0;
}

static int __devexit mxc_hdmi_codec_remove(struct platform_device *pdev)
{
	struct mxc_hdmi_priv *hdmi_priv = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	kfree(hdmi_priv);

	return 0;
}

struct platform_driver mxc_hdmi_driver = {
	.driver = {
		   .name = "mxc_hdmi_soc",
		   .owner = THIS_MODULE,
		   },
	.probe = mxc_hdmi_codec_probe,
	.remove = __devexit_p(mxc_hdmi_codec_remove),
};

static int __init mxc_hdmi_codec_init(void)
{
	return platform_driver_register(&mxc_hdmi_driver);
}

static void __exit mxc_hdmi_codec_exit(void)
{
	return platform_driver_unregister(&mxc_hdmi_driver);
}

module_init(mxc_hdmi_codec_init);
module_exit(mxc_hdmi_codec_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC HDMI Audio");
MODULE_LICENSE("GPL");
