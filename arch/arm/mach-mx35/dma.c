/*
 *  Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/device.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include "serial.h"

#ifdef CONFIG_SND_MXC_SOC_IRAM
#define soc_trans_type  int_2_per
#else
#define soc_trans_type emi_2_per
#endif

#define MXC_SPDIF_TXFIFO_WML      8
#define MXC_SPDIF_RXFIFO_WML      8
#define MXC_SPDIF_TX_REG          0x2C
#define MXC_SPDIF_RX_REG          0x14

#define MXC_SSI_TX0_REG           0x0
#define MXC_SSI_TX1_REG           0x4
#define MXC_SSI_RX0_REG           0x8
#define MXC_SSI_RX1_REG           0xC
#define MXC_SSI_TXFIFO_WML        0x4
#define MXC_SSI_RXFIFO_WML        0x6

#define MXC_ASRC_FIFO_WML	0x40
#define MXC_ASRCA_RX_REG	0x60
#define MXC_ASRCA_TX_REG	0x64
#define MXC_ASRCB_RX_REG	0x68
#define MXC_ASRCB_TX_REG	0x6C
#define MXC_ASRCC_RX_REG        0x70
#define MXC_ASRCC_TX_REG        0x74

#define MXC_ESAI_TX_REG	0x00
#define MXC_ESAI_RX_REG	0x04
#define MXC_ESAI_FIFO_WML 0x40

struct mxc_sdma_info_entry_s {
	mxc_dma_device_t device;
	void *chnl_info;
};

static mxc_sdma_channel_params_t mxc_sdma_uart1_rx_params = {
	.chnl_params = {
			.watermark_level = UART1_UFCR_RXTL,
			.per_address = UART1_BASE_ADDR,
			.peripheral_type = UART,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART1_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART1_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart1_tx_params = {
	.chnl_params = {
			.watermark_level = UART1_UFCR_TXTL,
			.per_address = UART1_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART1_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART1_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart2_rx_params = {
	.chnl_params = {
			.watermark_level = UART2_UFCR_RXTL,
			.per_address = UART2_BASE_ADDR,
			.peripheral_type = UART,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART2_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART2_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart2_tx_params = {
	.chnl_params = {
			.watermark_level = UART2_UFCR_TXTL,
			.per_address = UART2_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART2_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART2_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart3_rx_params = {
	.chnl_params = {
			.watermark_level = UART3_UFCR_RXTL,
			.per_address = UART3_BASE_ADDR,
			.peripheral_type = UART_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_UART3_RX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART3_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_uart3_tx_params = {
	.chnl_params = {
			.watermark_level = UART3_UFCR_TXTL,
			.per_address = UART3_BASE_ADDR + MXC_UARTUTXD,
			.peripheral_type = UART_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_UART3_TX,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_UART3_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_spdif_16bit_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_SPDIF_TXFIFO_WML,
			.per_address = SPDIF_BASE_ADDR + MXC_SPDIF_TX_REG,
			.peripheral_type = SPDIF,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SPDIF_TX,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SPDIF_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_spdif_32bit_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_SPDIF_TXFIFO_WML,
			.per_address = SPDIF_BASE_ADDR + MXC_SPDIF_TX_REG,
			.peripheral_type = SPDIF,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SPDIF_TX,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SPDIF_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_spdif_32bit_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_SPDIF_RXFIFO_WML,
			.per_address = SPDIF_BASE_ADDR + MXC_SPDIF_RX_REG,
			.peripheral_type = SPDIF,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SPDIF_RX,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SPDIF_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_memory_params = {
	.chnl_params = {
			.peripheral_type = MEMORY,
			.transfer_type = emi_2_emi,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_MEMORY,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_8bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_8bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_16bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_16bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_24bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_24bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_8bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_8bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_16bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_16bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_24bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI1_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi1_24bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI1_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_SSI1_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI1_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_8bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_8bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_16bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_16bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_24bit_rx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX1,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_24bit_tx0_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX0_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX1,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_8bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_8bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_8BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_16bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_16bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_24bit_rx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_RXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_RX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_SSI2_RX2,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_RX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_ssi2_24bit_tx1_params = {
	.chnl_params = {
			.watermark_level = MXC_SSI_TXFIFO_WML,
			.per_address = SSI2_BASE_ADDR + MXC_SSI_TX1_REG,
			.peripheral_type = SSI_SP,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_SSI2_TX2,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_SSI2_TX,
	.chnl_priority = 2,
};

static mxc_sdma_channel_params_t mxc_sdma_asrca_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCA_RX_REG,
			.peripheral_type = ASRC,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_ASRC_DMA1,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCA_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_asrca_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			.peripheral_type = ASRC,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_ASRC_DMA4,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCA_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_asrcb_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCB_RX_REG,
			.peripheral_type = ASRC,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_ASRC_DMA2,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCB_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_asrcb_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			.peripheral_type = ASRC,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_ASRC_DMA5,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCB_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_asrcc_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML * 3,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCC_RX_REG,
			.peripheral_type = ASRC,
			.transfer_type = emi_2_per,
			.event_id = DMA_REQ_ASRC_DMA3,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCC_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_asrcc_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_ASRC_FIFO_WML * 3,
			.per_address = ASRC_BASE_ADDR + MXC_ASRCC_TX_REG,
			.peripheral_type = ASRC,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_ASRC_DMA6,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ASRCC_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrca_ssi1_tx0_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI1_BASE_ADDR + MXC_SSI_TX0_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI1_TX1,
				       .event_id2 = DMA_REQ_ASRC_DMA4,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCA_SSI1_TX0,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrca_ssi1_tx1_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI1_BASE_ADDR + MXC_SSI_TX1_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI1_TX2,
				       .event_id2 = DMA_REQ_ASRC_DMA4,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCA_SSI1_TX1,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrca_ssi2_tx0_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI2_BASE_ADDR + MXC_SSI_TX0_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI2_TX1,
				       .event_id2 = DMA_REQ_ASRC_DMA4,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCA_SSI2_TX0,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrca_ssi2_tx1_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI2_BASE_ADDR + MXC_SSI_TX1_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI2_TX2,
				       .event_id2 = DMA_REQ_ASRC_DMA4,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCA_SSI2_TX1,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcb_ssi1_tx0_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI1_BASE_ADDR + MXC_SSI_TX0_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI1_TX1,
				       .event_id2 = DMA_REQ_ASRC_DMA5,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCB_SSI1_TX0,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcb_ssi1_tx1_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI1_BASE_ADDR + MXC_SSI_TX1_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI1_TX2,
				       .event_id2 = DMA_REQ_ASRC_DMA5,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCB_SSI1_TX1,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcb_ssi2_tx0_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI2_BASE_ADDR + MXC_SSI_TX0_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI2_TX1,
				       .event_id2 = DMA_REQ_ASRC_DMA5,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCB_SSI2_TX0,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcb_ssi2_tx1_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       SSI2_BASE_ADDR + MXC_SSI_TX1_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_SSI2_TX2,
				       .event_id2 = DMA_REQ_ASRC_DMA5,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_SSI_TXFIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCB_SSI2_TX1,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrca_esai_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       ESAI_BASE_ADDR + MXC_ESAI_TX_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_ESAI_TX,
				       .event_id2 = DMA_REQ_ASRC_DMA4,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_ESAI_FIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCA_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCA_ESAI,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcb_esai_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       ESAI_BASE_ADDR + MXC_ESAI_TX_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_ESAI_TX,
				       .event_id2 = DMA_REQ_ASRC_DMA5,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_ESAI_FIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCB_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCB_ESAI,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_ext_params_t mxc_sdma_asrcc_esai_params = {
	.chnl_ext_params = {
			    .common = {
				       .watermark_level =
				       MXC_ASRC_FIFO_WML >> 1,
				       .per_address =
				       ESAI_BASE_ADDR + MXC_ESAI_TX_REG,
				       .peripheral_type = ASRC,
				       .transfer_type = per_2_per,
				       .event_id = DMA_REQ_ESAI_TX,
				       .event_id2 = DMA_REQ_ASRC_DMA6,
				       .bd_number = 32,
				       .word_size = TRANSFER_32BIT,
				       .ext = 1,
				       },
			    .p2p_dir = 0,
			    .info_bits =
			    SDMA_ASRC_P2P_INFO_CONT | SDMA_ASRC_P2P_INFO_SP |
			    SDMA_ASRC_P2P_INFO_DP,
			    .watermark_level2 = MXC_ASRC_FIFO_WML,
			    .per_address2 = ASRC_BASE_ADDR + MXC_ASRCC_TX_REG,
			    },
	.channel_num = MXC_DMA_CHANNEL_ASRCC_ESAI,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_esai_16bit_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_ESAI_FIFO_WML,
			.per_address = ESAI_BASE_ADDR + MXC_ESAI_RX_REG,
			.peripheral_type = ESAI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_ESAI_RX,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ESAI_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_esai_16bit_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_ESAI_FIFO_WML,
			.per_address = ESAI_BASE_ADDR + MXC_ESAI_TX_REG,
			.peripheral_type = ESAI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_ESAI_TX,
			.bd_number = 32,
			.word_size = TRANSFER_16BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ESAI_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_esai_24bit_rx_params = {
	.chnl_params = {
			.watermark_level = MXC_ESAI_FIFO_WML,
			.per_address = ESAI_BASE_ADDR + MXC_ESAI_RX_REG,
			.peripheral_type = ESAI,
			.transfer_type = per_2_emi,
			.event_id = DMA_REQ_ESAI_RX,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ESAI_RX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static mxc_sdma_channel_params_t mxc_sdma_esai_24bit_tx_params = {
	.chnl_params = {
			.watermark_level = MXC_ESAI_FIFO_WML,
			.per_address = ESAI_BASE_ADDR + MXC_ESAI_TX_REG,
			.peripheral_type = ESAI,
			.transfer_type = soc_trans_type,
			.event_id = DMA_REQ_ESAI_TX,
			.bd_number = 32,
			.word_size = TRANSFER_32BIT,
			},
	.channel_num = MXC_DMA_CHANNEL_ESAI_TX,
	.chnl_priority = MXC_SDMA_DEFAULT_PRIORITY,
};

static struct mxc_sdma_info_entry_s mxc_sdma_active_dma_info[] = {
	{MXC_DMA_UART1_RX, &mxc_sdma_uart1_rx_params},
	{MXC_DMA_UART1_TX, &mxc_sdma_uart1_tx_params},
	{MXC_DMA_UART2_RX, &mxc_sdma_uart2_rx_params},
	{MXC_DMA_UART2_TX, &mxc_sdma_uart2_tx_params},
	{MXC_DMA_UART3_RX, &mxc_sdma_uart3_rx_params},
	{MXC_DMA_UART3_TX, &mxc_sdma_uart3_tx_params},
	{MXC_DMA_SPDIF_16BIT_TX, &mxc_sdma_spdif_16bit_tx_params},
	{MXC_DMA_SPDIF_32BIT_TX, &mxc_sdma_spdif_32bit_tx_params},
	{MXC_DMA_SPDIF_32BIT_RX, &mxc_sdma_spdif_32bit_rx_params},
	{MXC_DMA_SSI1_8BIT_RX0, &mxc_sdma_ssi1_8bit_rx0_params},
	{MXC_DMA_SSI1_8BIT_TX0, &mxc_sdma_ssi1_8bit_tx0_params},
	{MXC_DMA_SSI1_16BIT_RX0, &mxc_sdma_ssi1_16bit_rx0_params},
	{MXC_DMA_SSI1_16BIT_TX0, &mxc_sdma_ssi1_16bit_tx0_params},
	{MXC_DMA_SSI1_24BIT_RX0, &mxc_sdma_ssi1_24bit_rx0_params},
	{MXC_DMA_SSI1_24BIT_TX0, &mxc_sdma_ssi1_24bit_tx0_params},
	{MXC_DMA_SSI1_8BIT_RX1, &mxc_sdma_ssi1_8bit_rx1_params},
	{MXC_DMA_SSI1_8BIT_TX1, &mxc_sdma_ssi1_8bit_tx1_params},
	{MXC_DMA_SSI1_16BIT_RX1, &mxc_sdma_ssi1_16bit_rx1_params},
	{MXC_DMA_SSI1_16BIT_TX1, &mxc_sdma_ssi1_16bit_tx1_params},
	{MXC_DMA_SSI1_24BIT_RX1, &mxc_sdma_ssi1_24bit_rx1_params},
	{MXC_DMA_SSI1_24BIT_TX1, &mxc_sdma_ssi1_24bit_tx1_params},
	{MXC_DMA_SSI2_8BIT_RX0, &mxc_sdma_ssi2_8bit_rx0_params},
	{MXC_DMA_SSI2_8BIT_TX0, &mxc_sdma_ssi2_8bit_tx0_params},
	{MXC_DMA_SSI2_16BIT_RX0, &mxc_sdma_ssi2_16bit_rx0_params},
	{MXC_DMA_SSI2_16BIT_TX0, &mxc_sdma_ssi2_16bit_tx0_params},
	{MXC_DMA_SSI2_24BIT_RX0, &mxc_sdma_ssi2_24bit_rx0_params},
	{MXC_DMA_SSI2_24BIT_TX0, &mxc_sdma_ssi2_24bit_tx0_params},
	{MXC_DMA_SSI2_8BIT_RX1, &mxc_sdma_ssi2_8bit_rx1_params},
	{MXC_DMA_SSI2_8BIT_TX1, &mxc_sdma_ssi2_8bit_tx1_params},
	{MXC_DMA_SSI2_16BIT_RX1, &mxc_sdma_ssi2_16bit_rx1_params},
	{MXC_DMA_SSI2_16BIT_TX1, &mxc_sdma_ssi2_16bit_tx1_params},
	{MXC_DMA_SSI2_24BIT_RX1, &mxc_sdma_ssi2_24bit_rx1_params},
	{MXC_DMA_SSI2_24BIT_TX1, &mxc_sdma_ssi2_24bit_tx1_params},
	{MXC_DMA_ASRC_A_RX, &mxc_sdma_asrca_rx_params},
	{MXC_DMA_ASRC_A_TX, &mxc_sdma_asrca_tx_params},
	{MXC_DMA_ASRC_B_RX, &mxc_sdma_asrcb_rx_params},
	{MXC_DMA_ASRC_B_TX, &mxc_sdma_asrcb_tx_params},
	{MXC_DMA_ASRC_C_RX, &mxc_sdma_asrcc_rx_params},
	{MXC_DMA_ASRC_C_TX, &mxc_sdma_asrcc_tx_params},
	{MXC_DMA_ASRCA_SSI1_TX0, &mxc_sdma_asrca_ssi1_tx0_params},
	{MXC_DMA_ASRCA_SSI1_TX1, &mxc_sdma_asrca_ssi1_tx1_params},
	{MXC_DMA_ASRCA_SSI2_TX0, &mxc_sdma_asrca_ssi2_tx0_params},
	{MXC_DMA_ASRCA_SSI2_TX1, &mxc_sdma_asrca_ssi2_tx1_params},
	{MXC_DMA_ASRCB_SSI1_TX0, &mxc_sdma_asrcb_ssi1_tx0_params},
	{MXC_DMA_ASRCB_SSI1_TX1, &mxc_sdma_asrcb_ssi1_tx1_params},
	{MXC_DMA_ASRCB_SSI2_TX0, &mxc_sdma_asrcb_ssi2_tx0_params},
	{MXC_DMA_ASRCB_SSI2_TX1, &mxc_sdma_asrcb_ssi2_tx1_params},
	{MXC_DMA_ASRCA_ESAI, &mxc_sdma_asrca_esai_params},
	{MXC_DMA_ASRCB_ESAI, &mxc_sdma_asrcb_esai_params},
	{MXC_DMA_ASRCC_ESAI, &mxc_sdma_asrcc_esai_params},
	{MXC_DMA_ESAI_16BIT_RX, &mxc_sdma_esai_16bit_rx_params},
	{MXC_DMA_ESAI_16BIT_TX, &mxc_sdma_esai_16bit_tx_params},
	{MXC_DMA_ESAI_24BIT_RX, &mxc_sdma_esai_24bit_rx_params},
	{MXC_DMA_ESAI_24BIT_TX, &mxc_sdma_esai_24bit_tx_params},
	{MXC_DMA_MEMORY, &mxc_sdma_memory_params},
};

static int mxc_sdma_info_entrys =
    sizeof(mxc_sdma_active_dma_info) / sizeof(mxc_sdma_active_dma_info[0]);
/*!
 * This functions Returns the SDMA paramaters associated for a module
 *
 * @param channel_id the ID of the module requesting DMA
 * @return returns the sdma parameters structure for the device
 */
mxc_sdma_channel_params_t *mxc_sdma_get_channel_params(mxc_dma_device_t
						       channel_id)
{
	struct mxc_sdma_info_entry_s *p = mxc_sdma_active_dma_info;
	int i;

	for (i = 0; i < mxc_sdma_info_entrys; i++, p++) {
		if (p->device == channel_id)
			return p->chnl_info;
	}
	return NULL;
}

EXPORT_SYMBOL(mxc_sdma_get_channel_params);

/*!
 * This functions marks the SDMA channels that are statically allocated
 *
 * @param chnl the channel array used to store channel information
 */
void mxc_get_static_channels(mxc_dma_channel_t *chnl)
{
	/* No channels statically allocated for MX35 */
#ifdef CONFIG_SDMA_IRAM
	int i;
	for (i = MXC_DMA_CHANNEL_IRAM; i < MAX_DMA_CHANNELS; i++)
		chnl[i].dynamic = 0;
#endif
}

EXPORT_SYMBOL(mxc_get_static_channels);
