/*
 * imx-esai.h  --  ESAI driver header file for Freescale IMX
 *
 * Copyright 2008-2009 Freescale  Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _MXC_ESAI_H
#define _MXC_ESAI_H

#define IMX_DAI_ESAI_TX 0x04
#define IMX_DAI_ESAI_RX 0x08
#define IMX_DAI_ESAI_TXRX (IMX_DAI_ESAI_TX | IMX_DAI_ESAI_RX)

extern struct snd_soc_dai imx_esai_dai;

#endif
