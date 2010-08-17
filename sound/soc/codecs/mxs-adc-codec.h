/*
 * ALSA codec for Freescale MXS ADC/DAC Audio
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
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
#ifndef __MXS_ADC_CODEC_H
#define __MXS_ADC_CODEC_H

#define DAC_CTRL_L		0
#define DAC_CTRL_H		1
#define DAC_STAT_L		2
#define DAC_STAT_H		3
#define DAC_SRR_L		4
#define DAC_VOLUME_L		6
#define DAC_VOLUME_H		7
#define DAC_DEBUG_L		8
#define DAC_DEBUG_H		9
#define DAC_HPVOL_L		10
#define DAC_HPVOL_H		11
#define DAC_PWRDN_L		12
#define DAC_PWRDN_H		13
#define DAC_REFCTRL_L		14
#define DAC_REFCTRL_H		15
#define DAC_ANACTRL_L		16
#define DAC_ANACTRL_H		17
#define DAC_TEST_L		18
#define DAC_TEST_H		19
#define DAC_BISTCTRL_L		20
#define DAC_BISTCTRL_H		21
#define DAC_BISTSTAT0_L		22
#define DAC_BISTSTAT0_H		23
#define DAC_BISTSTAT1_L		24
#define DAC_BISTSTAT1_H		25
#define DAC_ANACLKCTRL_L	26
#define DAC_ANACLKCTRL_H	27
#define DAC_DATA_L		28
#define DAC_DATA_H		29
#define DAC_SPEAKERCTRL_L	30
#define DAC_SPEAKERCTRL_H	31
#define DAC_VERSION_L		32
#define DAC_VERSION_H		33
#define ADC_CTRL_L		34
#define ADC_CTRL_H		35
#define ADC_STAT_L		36
#define ADC_STAT_H		37
#define ADC_SRR_L		38
#define ADC_SRR_H		39
#define ADC_VOLUME_L		40
#define ADC_VOLUME_H		41
#define ADC_DEBUG_L		42
#define ADC_DEBUG_H		43
#define ADC_ADCVOL_L		44
#define ADC_ADCVOL_H		45
#define ADC_MICLINE_L		46
#define ADC_MICLINE_H		47
#define ADC_ANACLKCTRL_L	48
#define ADC_ANACLKCTRL_H	49
#define ADC_DATA_L		50
#define ADC_DATA_H		51

#define ADC_REGNUM	52

#define DAC_VOLUME_MIN	0x37
#define DAC_VOLUME_MAX	0xFE
#define ADC_VOLUME_MIN	0x37
#define ADC_VOLUME_MAX	0xFE
#define HP_VOLUME_MAX	0x0
#define HP_VOLUME_MIN	0x7F
#define LO_VOLUME_MAX	0x0
#define LO_VOLUME_MIN	0x1F

extern struct snd_soc_dai mxs_codec_dai;
extern struct snd_soc_codec_device soc_codec_dev_mxs;

#endif /* __MXS_ADC_CODEC_H */
