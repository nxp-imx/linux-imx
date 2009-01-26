/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 *@file mach-mx27/dma.c
 *@brief This file contains the dma parameter which is depend on the platform  .
 * @ingroup DMA_MX27
 */

#include <linux/module.h>
#include <linux/init.h>
#include <asm/dma.h>

#define MXC_SOUND_PLAYBACK_CHAIN_DMA 1
#define MXC_SOUND_CAPTURE_CHAIN_DMA 1

/*!
 * @brief  the structure stored device_id and dma_info pointer
 */
typedef struct dma_info_entry_s {
	mxc_dma_device_t device;
	/* if there are two dma_info , first is for reading, another is for writing */
	mx2_dma_info_t *info;
} dma_info_entry_t;

/*!
 * @brief dma_info from memory to memory for dma testing
 */
static mx2_dma_info_t ram2ram_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_LINEAR,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 0
};

/*!
 * @brief dma_info from 2D memory to 2D memory for dma testing
 */
static mx2_dma_info_t ram2d2ram2d_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.rto_en = 0,
	.dma_chaining = 0,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_2D,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_2D,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 1,.msel = 0,.W = 0x80,.X = 0x40,.Y = 0x10
};

/*!
 * @brief dma_info from memory to 2D memory for dma testing
 */
static mx2_dma_info_t ram2ram2d_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_LINEAR,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_2D,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 1,.msel = 0,.W = 0x100,.X = 0x80,.Y = 0x10
};

/*!
 * @brief dma_info from 2D memory to memory for dma testing
 */
static mx2_dma_info_t ram2d2ram_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_2D,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 1,.msel = 0,.W = 0x100,.X = 0x100,.Y = 0x10
};

/*!
 * @brief dma_info with dma chaining feature for dma testing
 */
static mx2_dma_info_t hw_chaining_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 1,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_LINEAR,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info without dma chaining feature for dma testing
 */
static mx2_dma_info_t sw_chaining_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 0,
	.burstLength = 4,.request = 0,.busuntils = 0,
	.sourceType = DMA_TYPE_LINEAR,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for ATA recieveing
 */
static mx2_dma_info_t ata_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 32,.request = 29,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.per_address = (ATA_BASE_ADDR + 0x18),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for ATA transmitting
 */
static mx2_dma_info_t ata_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 32,.request = 28,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_32BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_32BIT,
	.per_address = (ATA_BASE_ADDR + 0x18),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART1 recieveing
 */
static mx2_dma_info_t uart1_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 1,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 26,.busuntils = 8,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART1_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART1 transmitting
 */
static mx2_dma_info_t uart1_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 27,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART1_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART2 recieveing
 */
static mx2_dma_info_t uart2_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 24,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART2_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART2 transmitting
 */
static mx2_dma_info_t uart2_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 25,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART2_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART3 recieveing
 */
static mx2_dma_info_t uart3_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 22,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART3_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART3 transmitting
 */
static mx2_dma_info_t uart3_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 23,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART3_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART4 recieveing
 */
static mx2_dma_info_t uart4_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 20,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART4_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART4transmitting
 */
static mx2_dma_info_t uart4_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 21,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART4_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART5 recieveing
 */
static mx2_dma_info_t uart5_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 32,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART5_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART5 transmitting
 */
static mx2_dma_info_t uart5_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 33,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART5_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

/*!
 * @brief dma_info for UART6 recieveing
 */
static mx2_dma_info_t uart6_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 34,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART6_BASE_ADDR),
	.M2D_Valid = 0,
};

/*!
 * @brief: dma_info for UART6 transmitting
 */
static mx2_dma_info_t uart6_tx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 1,.request = 35,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = TRANSFER_8BIT,
	.destType = DMA_TYPE_LINEAR,.destPort = TRANSFER_8BIT,
	.per_address = (UART6_BASE_ADDR + 0x40),
	.M2D_Valid = 0,
};

static mx2_dma_info_t ssi1_16bit_rx0_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = MXC_SOUND_CAPTURE_CHAIN_DMA,.ren = 1,
	.burstLength = 8,.request = DMA_REQ_SSI1_RX0,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_16,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SSI1_BASE_ADDR + 0x08),
	.M2D_Valid = 0,
};

static mx2_dma_info_t ssi1_16bit_tx0_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = MXC_SOUND_PLAYBACK_CHAIN_DMA,.ren = 1,
	.burstLength = 8,.request = DMA_REQ_SSI1_TX0,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_16,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SSI1_BASE_ADDR),
	.M2D_Valid = 0,
};

static mx2_dma_info_t ssi2_16bit_rx0_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = MXC_SOUND_CAPTURE_CHAIN_DMA,.ren = 1,
	.burstLength = 8,.request = DMA_REQ_SSI2_RX0,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_16,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SSI2_BASE_ADDR + 0x08),
	.M2D_Valid = 0,
};

static mx2_dma_info_t ssi2_16bit_tx0_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 1,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = MXC_SOUND_PLAYBACK_CHAIN_DMA,.ren = 1,
	.burstLength = 8,.request = DMA_REQ_SSI2_TX0,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_16,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SSI2_BASE_ADDR),
	.M2D_Valid = 0,
};

static mx2_dma_info_t mmc1_width1_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 16,.request = DMA_REQ_SDHC1,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_32,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SDHC1_BASE_ADDR + 0x38),
	.M2D_Valid = 0,
};

static mx2_dma_info_t mmc1_width4_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 0,.request = DMA_REQ_SDHC1,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_32,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SDHC1_BASE_ADDR + 0x38),
	.M2D_Valid = 0,
};

static mx2_dma_info_t mmc2_width1_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 16,.request = DMA_REQ_SDHC2,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_32,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SDHC2_BASE_ADDR + 0x38),
	.M2D_Valid = 0,
};

static mx2_dma_info_t mmc2_width4_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 0,.ren = 1,
	.burstLength = 0,.request = DMA_REQ_SDHC2,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_32,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (SDHC2_BASE_ADDR + 0x38),
	.M2D_Valid = 0,
};

static mx2_dma_info_t csi_rx_dma_info = {
	.dma_chan = MXC_DMA_DYNAMIC_CHANNEL,
	.mode = 0,
	.rto_en = 0,
	.dir = 0,
	.dma_chaining = 1,.ren = 1,
	.burstLength = 64,.request = DMA_REQ_CSI_RX,.busuntils = 0,
	.sourceType = DMA_TYPE_FIFO,.sourcePort = DMA_MEM_SIZE_32,
	.destType = DMA_TYPE_LINEAR,.destPort = DMA_MEM_SIZE_32,
	.per_address = (CSI_BASE_ADDR + 0x10),
	.M2D_Valid = 0,
};

/*!
 * @brief dma info array which is actived
 *    DEVICE_ID  RX/(RX&TX)	TX
 */
static dma_info_entry_t active_dma_info[] = {
	{MXC_DMA_TEST_RAM2RAM, &ram2ram_dma_info},
	{MXC_DMA_TEST_RAM2D2RAM2D, &ram2d2ram2d_dma_info},
	{MXC_DMA_TEST_RAM2RAM2D, &ram2ram2d_dma_info},
	{MXC_DMA_TEST_RAM2D2RAM, &ram2d2ram_dma_info},
	{MXC_DMA_TEST_HW_CHAINING, &hw_chaining_dma_info},
	{MXC_DMA_TEST_SW_CHAINING, &sw_chaining_dma_info},
	{MXC_DMA_ATA_RX, &ata_rx_dma_info},
	{MXC_DMA_ATA_TX, &ata_tx_dma_info},
	{MXC_DMA_UART1_RX, &uart1_rx_dma_info},
	{MXC_DMA_UART1_TX, &uart1_tx_dma_info},
	{MXC_DMA_UART2_RX, &uart2_rx_dma_info},
	{MXC_DMA_UART2_TX, &uart2_tx_dma_info},
	{MXC_DMA_UART3_RX, &uart3_rx_dma_info},
	{MXC_DMA_UART3_TX, &uart3_tx_dma_info},
	{MXC_DMA_UART4_RX, &uart4_rx_dma_info},
	{MXC_DMA_UART4_TX, &uart4_tx_dma_info},
	{MXC_DMA_UART5_RX, &uart5_rx_dma_info},
	{MXC_DMA_UART5_TX, &uart5_tx_dma_info},
	{MXC_DMA_UART6_RX, &uart6_rx_dma_info},
	{MXC_DMA_UART6_TX, &uart6_tx_dma_info},
	{MXC_DMA_SSI1_16BIT_RX0, &ssi1_16bit_rx0_dma_info},
	{MXC_DMA_SSI1_16BIT_TX0, &ssi1_16bit_tx0_dma_info},
	{MXC_DMA_SSI2_16BIT_RX0, &ssi2_16bit_rx0_dma_info},
	{MXC_DMA_SSI2_16BIT_TX0, &ssi2_16bit_tx0_dma_info},
	{MXC_DMA_MMC1_WIDTH_1, &mmc1_width1_dma_info},
	{MXC_DMA_MMC1_WIDTH_4, &mmc1_width4_dma_info},
	{MXC_DMA_MMC2_WIDTH_1, &mmc2_width1_dma_info},
	{MXC_DMA_MMC2_WIDTH_4, &mmc2_width4_dma_info},
	{MXC_DMA_CSI_RX, &csi_rx_dma_info},
};

/*!
 * @brief the number of actived dma info
 */
static int dma_info_entrys =
    sizeof(active_dma_info) / sizeof(active_dma_info[0]);

/*!
 * @brief get the dma info by channel_id
 */
mx2_dma_info_t *mxc_dma_get_info(mxc_dma_device_t channel_id)
{
	dma_info_entry_t *p = active_dma_info;
	int i;
	for (i = 0; i < dma_info_entrys; i++, p++) {
		if (p->device == channel_id)
			return p->info;
	}
	return NULL;
}

/*!
 * @brief: scan dma parameter list . And collect information about which channels are dynamic .
 */
void mxc_dma_load_info(mxc_dma_channel_t * dma)
{
	int i, idx;
	dma_info_entry_t *p = active_dma_info;

	BUG_ON(dma == NULL);
	BUG_ON(p == NULL);

	for (i = 0; i < MXC_DMA_CHANNELS; i++) {
		dma[i].dynamic = 1;
	}

	for (i = 0; i < dma_info_entrys; i++, p++) {
		BUG_ON((p->info == NULL));

		idx = p->info->dma_chan;

		BUG_ON(((idx >= MAX_DMA_CHANNELS)
			&& (idx != MXC_DMA_DYNAMIC_CHANNEL)));
		if ((idx < 0) || (idx == MXC_DMA_DYNAMIC_CHANNEL))
			continue;
		dma[idx].dynamic = 0;
	}
}

EXPORT_SYMBOL(mxc_dma_get_info);
EXPORT_SYMBOL(mxc_dma_load_info);
