/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef _STMP3XXX_DCP_H_
#define _STMP3XXX_DCP_H_

#include <mach/platform.h>
#include <mach/stmp3xxx.h>
#include <mach/regs-dcp.h>

#define CIPHER_CHAN	1
#define CIPHER_MASK	(1 << CIPHER_CHAN)

#define HASH_CHAN	0
#define HASH_MASK	(1 << HASH_CHAN)

#define ALL_MASK	(CIPHER_MASK | HASH_MASK)

/* Defines the initialization value for the dcp control register */
#define STMP3XXX_DCP_CTRL_INIT \
   (BM_DCP_CTRL_GATHER_RESIDUAL_WRITES | \
    BM_DCP_CTRL_ENABLE_CONTEXT_CACHING | \
    BV_DCP_CTRL_CHANNEL_INTERRUPT_ENABLE__CH0 | \
    BV_DCP_CTRL_CHANNEL_INTERRUPT_ENABLE__CH1 | \
    BV_DCP_CTRL_CHANNEL_INTERRUPT_ENABLE__CH2 | \
    BV_DCP_CTRL_CHANNEL_INTERRUPT_ENABLE__CH3)

/* Defines the initialization value for the dcp channel control register */
#define STMP3XXX_DCP_CHANNELCTRL_INIT \
    BF(ALL_MASK, DCP_CHANNELCTRL_ENABLE_CHANNEL)

/* DCP work packet 1 value for encryption */
#define STMP3XXX_DCP_PKT1_ENCRYPT \
   (BM_DCP_PACKET1_DECR_SEMAPHORE | \
    BM_DCP_PACKET1_ENABLE_CIPHER | \
    BM_DCP_PACKET1_CIPHER_ENCRYPT | \
    BM_DCP_PACKET1_CIPHER_INIT)

/* DCP work packet 1 value for decryption */
#define DCP_PKT1_DECRYPT \
   (BM_DCP_PACKET1_DECR_SEMAPHORE | \
    BM_DCP_PACKET1_ENABLE_CIPHER | \
    BM_DCP_PACKET1_CIPHER_INIT)

/* DCP (decryption) work packet definition */
struct stmp3xxx_dcp_hw_packet {
	uint32_t pNext;     /* next dcp work packet address */
	uint32_t pkt1;      /* dcp work packet 1 (control 0) */
	uint32_t pkt2;      /* dcp work packet 2 (control 1) */
	uint32_t pSrc;      /* source buffer address */
	uint32_t pDst;      /* destination buffer address */
	uint32_t size;      /* buffer size in bytes */
	uint32_t pPayload;  /* payload buffer address */
	uint32_t stat;      /* dcp status (written by dcp) */
};

#define STMP3XXX_DCP_NUM_CHANNELS 4

#endif
