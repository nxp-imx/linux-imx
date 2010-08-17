/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

 /*!
  * @file       mxc-alsa-mixer.c
  * @brief      this file implements the mxc sound driver mixer interface for ALSA.
  *             The mxc sound driver supports mono/stereo recording (there are
  *             some limitations due to hardware), mono/stereo playback and
  *             audio mixing. This file implements output switching, volume/balance controls
  *             mono adder config, I/P dev switching and gain on the PCM streams.
  *             Recording supports 8000 khz and 16000 khz sample rate.
  *             Playback supports 8000, 11025, 16000, 22050, 24000, 32000,
  *             44100 and 48000 khz for mono and stereo.
  *
  * @ingroup SOUND_DRV
  */

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/soundcard.h>
#include <mach/pmic_audio.h>
#include "mxc-alsa-common.h"
/*!
  * These are the functions implemented in the ALSA PCM driver that
  * are used for mixer operations
  *
  */

/*!
  * These are the callback functions for mixer controls
  *
  */
/* Output device control*/
static int pmic_mixer_output_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 15;
	uinfo->value.integer.step = 1;
	return 0;
}
static int pmic_mixer_output_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *uvalue)
{
	int dev, i;
	dev = uvalue->value.integer.value[0];
	for (i = OP_EARPIECE; i < OP_MAXDEV; i++) {
		if (dev & (1 << i)) {
			set_mixer_output_device(NULL, MIXER_OUT, i, 1);
		} else {
			set_mixer_output_device(NULL, MIXER_OUT, i, 0);
		}
	}
	return 0;
}
static int pmic_mixer_output_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *uvalue)
{
	int val, ret = 0, i = 0;
	for (i = OP_EARPIECE; i < OP_MAXDEV; i++) {
		val = get_mixer_output_device();
		if (val & SOUND_MASK_PHONEOUT)
			ret = ret | 1;
		if (val & SOUND_MASK_SPEAKER)
			ret = ret | 2;
		if (val & SOUND_MASK_VOLUME)
			ret = ret | 4;
		if (val & SOUND_MASK_PCM)
			ret = ret | 8;
		uvalue->value.integer.value[0] = ret;
	}
	return 0;

}

/* Input gain control*/
static int pmic_cap_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	uinfo->value.integer.step = 1;
	return 0;
}
static int pmic_cap_volume_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *uvalue)
{
	int val;
	val = get_mixer_input_gain();
	val = val & 0xFF;
	uvalue->value.integer.value[0] = val;
	return 0;
}

static int pmic_cap_volume_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *uvalue)
{

	int vol;
	vol = uvalue->value.integer.value[0];
	vol = vol | (vol << 8);
	set_mixer_input_gain(NULL, vol);
	return 0;
}

/* Mono adder control*/
static int pmic_pb_monoconfig_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 3;
	uinfo->value.integer.step = 1;
	return 0;
}
static int pmic_pb_monoconfig_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	int mono;
	mono = uvalue->value.integer.value[0];
	set_mixer_output_mono_adder(mono);
	return 0;
}
static int pmic_pb_monoconfig_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = get_mixer_output_mono_adder();
	return 0;
}

/*!
  * These are the ALSA control structures with init values
  *
  */

/* Input device control*/
static int pmic_cap_input_info(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 7;
	uinfo->value.integer.step = 1;
	return 0;
}
static int pmic_cap_input_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	int dev, i;
	dev = uvalue->value.integer.value[0];
	for (i = IP_HANDSET; i < IP_MAXDEV; i++) {
		dev &= (1 << i);
		if (dev) {
			set_mixer_input_device(NULL, i, 1);
		} else {
			set_mixer_input_device(NULL, i, 0);
		}
	}
	return 0;
}
static int pmic_cap_input_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	int val, ret = 0, i = 0;
	for (i = IP_HANDSET; i < IP_MAXDEV; i++) {
		val = get_mixer_input_device();
		if (val & SOUND_MASK_PHONEIN)
			ret = ret | 1;
		if (val & SOUND_MASK_MIC)
			ret = ret | 2;
		if (val & SOUND_MASK_LINE)
			ret = ret | 4;
		uvalue->value.integer.value[0] = ret;
	}
	return 0;
}

/* Volume control*/
static int pmic_pb_volume_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	int volume;
	volume = uvalue->value.integer.value[0];
	volume = volume | (volume << 8);
	set_mixer_output_volume(NULL, volume, OP_NODEV);
	return 0;
}
static int pmic_pb_volume_info(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 100;
	uinfo->value.integer.step = 1;
	return 0;
}

static int pmic_pb_volume_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *uvalue)
{
	int val;
	val = get_mixer_output_volume();
	val = val & 0xFF;
	uvalue->value.integer.value[0] = val;
	return 0;
}

/* Balance control start */
static int pmic_pb_balance_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	uinfo->value.integer.step = 1;
	return 0;
}

static int pmic_pb_balance_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = get_mixer_output_balance();
	return 0;

}
static int pmic_pb_balance_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *uvalue)
{
	int bal;
	bal = uvalue->value.integer.value[0];
	set_mixer_output_balance(bal);
	return 0;
}

/* Balance control end */

/* loopback control start */
static int pmic_loopback_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int pmic_loopback_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = kcontrol->private_value;
	return 0;

}
static int pmic_loopback_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *uvalue)
{
	int changed;
	long flag = uvalue->value.integer.value[0];
	changed =
	    (uvalue->value.integer.value[0] == kcontrol->private_value) ? 0 : 1;
	kcontrol->private_value = uvalue->value.integer.value[0];
	if (flag)
		pmic_audio_fm_output_enable(true);
	else
		pmic_audio_fm_output_enable(false);

	return changed;
}

/* Loopback control end */

/* Kcontrol structure definitions */
struct snd_kcontrol_new pmic_control_pb_vol __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Playback Volume",
	.index = 0x00,
	.info = pmic_pb_volume_info,
	.get = pmic_pb_volume_get,
	.put = pmic_pb_volume_put,
	.private_value = 0xffab1,
};

struct snd_kcontrol_new pmic_control_pb_bal __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Balance Playback Volume",
	.index = 0x00,
	.info = pmic_pb_balance_info,
	.get = pmic_pb_balance_get,
	.put = pmic_pb_balance_put,
	.private_value = 0xffab2,
};
struct snd_kcontrol_new pmic_control_pb_monoconfig __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Monoconfig Playback Volume",
	.index = 0x00,
	.info = pmic_pb_monoconfig_info,
	.get = pmic_pb_monoconfig_get,
	.put = pmic_pb_monoconfig_put,
	.private_value = 0xffab2,
};
struct snd_kcontrol_new pmic_control_op_sw __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Output Playback Volume",
	.index = 0x00,
	.info = pmic_mixer_output_info,
	.get = pmic_mixer_output_get,
	.put = pmic_mixer_output_put,
	.private_value = 0xffab4,
};

struct snd_kcontrol_new pmic_control_cap_vol __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Capture Volume",
	.index = 0x00,
	.info = pmic_cap_volume_info,
	.get = pmic_cap_volume_get,
	.put = pmic_cap_volume_put,
	.private_value = 0xffab5,
};
struct snd_kcontrol_new pmic_control_ip_sw __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Master Input Capture Volume",
	.index = 0x00,
	.info = pmic_cap_input_info,
	.get = pmic_cap_input_get,
	.put = pmic_cap_input_put,
	.private_value = 0xffab5,
};

struct snd_kcontrol_new pmic_control_loop_out __devinitdata = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Loopback Line-in",
	.index = 0x00,
	.info = pmic_loopback_info,
	.get = pmic_loopback_get,
	.put = pmic_loopback_put,
	.private_value = 0,
};

/*!
  * This function registers the control components of ALSA Mixer
  * It is called by ALSA PCM init.
  *
  * @param	card pointer to the ALSA sound card structure.
  *
  * @return              0 on success, -ve otherwise.
  */
int __devinit mxc_alsa_create_ctl(struct snd_card *card, void *p_value)
{
	int err = 0;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_op_sw, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_pb_vol, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_pb_monoconfig, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_pb_bal, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_cap_vol, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_ip_sw, p_value));
	if (err < 0)
		return err;

	err = snd_ctl_add(card, snd_ctl_new1(&pmic_control_loop_out, p_value));
	if (err < 0)
		return err;

	return 0;
}

EXPORT_SYMBOL(mxc_alsa_create_ctl);
