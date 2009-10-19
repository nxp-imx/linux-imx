/*
 * ASoC PCM interface for Freescale STMP37XX/STMP378X ADC/DAC
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
#ifndef _STMP3XXX_PCM_H
#define _STMP3XXX_PCM_H

struct stmp3xxx_pcm_dma_params {
	char *name;
	int dma_bus;	/* DMA bus */
	int dma_ch;	/* DMA channel number */
	int irq;	/* DMA interrupt number */
};

extern struct snd_soc_platform stmp3xxx_soc_platform;

#endif
