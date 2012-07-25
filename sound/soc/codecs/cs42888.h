/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _CS42888_H
#define _CS42888_H

/*
 * The ASoC codec DAI structure for the CS42888.  Assign this structure to
 * the .codec_dai field of your machine driver's snd_soc_dai_link structure.
 */
extern struct snd_soc_dai_driver cs42888_dai[];

/*
 * The ASoC codec device structure for the CS42888.  Assign this structure
 * to the .codec_dev field of your machine driver's snd_soc_device
 * structure.
 */
extern struct snd_soc_codec_device soc_codec_device_cs42888;

extern void gpio_cs42888_pdwn(int pdwn);
#endif
