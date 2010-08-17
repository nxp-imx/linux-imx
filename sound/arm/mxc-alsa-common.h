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
  * @file mxc-alsa-common.h
  * @brief
  * @ingroup SOUND_DRV
  */

#ifndef __MXC_ALSA_COMMON_H__
#define __MXC_ALSA_COMMON_H__

/* Enums typically used by the Mixer support APIs
  * Emunerates IP, OP and mixer sources.
  */

typedef enum {
	CODEC_DIR_OUT,
	MIXER_OUT
} OUTPUT_SOURCE;

typedef enum {
	OP_NODEV = -1,
	OP_EARPIECE,
	OP_HANDSFREE,
	OP_HEADSET,
	OP_LINEOUT,
	OP_MAXDEV,
	OP_MONO
} OUTPUT_DEVICES;

typedef enum {
	IP_NODEV = -1,
	IP_HANDSET,
	IP_HEADSET,
	IP_LINEIN,
	IP_MAXDEV
} INPUT_DEVICES;

extern int mxc_alsa_create_ctl(struct snd_card *card, void *p_value);

extern int set_mixer_output_device(PMIC_AUDIO_HANDLE handle, OUTPUT_SOURCE src,
				   OUTPUT_DEVICES dev, bool enable);
extern int set_mixer_output_volume(PMIC_AUDIO_HANDLE handle, int volume,
				   OUTPUT_DEVICES dev);
extern int set_mixer_input_device(PMIC_AUDIO_HANDLE handle, INPUT_DEVICES dev,
				  bool enable);
extern int set_mixer_output_mono_adder(PMIC_AUDIO_MONO_ADDER_MODE mode);
extern int set_mixer_input_gain(PMIC_AUDIO_HANDLE handle, int val);
extern int set_mixer_output_balance(int bal);

extern int get_mixer_output_device(void);
extern int get_mixer_output_volume(void);
extern int get_mixer_output_mono_adder(void);
extern int get_mixer_output_balance(void);
extern int get_mixer_input_gain(void);
extern int get_mixer_input_device(void);
#endif				/* __MXC_ALSA_COMMON_H__ */
