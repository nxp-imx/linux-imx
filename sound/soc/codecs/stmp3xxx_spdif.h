/*
 * ALSA SoC STMP378x SPDIF codec driver
 *
 * Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Copyright 2008 SigmaTel, Inc
 * Copyright 2008 Embedded Alley Solutions, Inc
 * Copyright 2008-2009 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program  is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef __STMP3XXX_SPDIF_CODEC_H
#define __STMP3XXX_SPDIF_CODEC_H

#define SPDIF_CTRL_L		0
#define SPDIF_CTRL_H		1
#define SPDIF_STAT_L		2
#define SPDIF_STAT_H		3
#define SPDIF_FRAMECTRL_L	4
#define SPDIF_FRAMECTRL_H	5
#define SPDIF_SRR_L		6
#define SPDIF_SRR_H		7
#define SPDIF_DEBUG_L		8
#define SPDIF_DEBUG_H		9
#define SPDIF_DATA_L		10
#define SPDIF_DATA_H		11
#define SPDIF_VERSION_L		12
#define SPDIF_VERSION_H		13

#define SPDIF_REGNUM		14

extern struct snd_soc_dai stmp3xxx_spdif_codec_dai;
extern struct snd_soc_codec_device soc_spdif_codec_dev_stmp3xxx;

#endif /* __STMP3XXX_SPDIF_CODEC_H */
