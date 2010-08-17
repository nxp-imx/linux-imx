/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx35_asrc.h
 *
 * @brief MX35 Asynchronous Sample Rate Converter
 *
 * @ingroup ??
 */

#ifndef __MXC_ASRC_H__
#define __MXC_ASRC_H__

#define ASRC_IOC_MAGIC	'C'

#define ASRC_REQ_PAIR	_IOWR(ASRC_IOC_MAGIC, 0, struct asrc_req)
#define ASRC_CONFIG_PAIR	_IOWR(ASRC_IOC_MAGIC, 1, struct asrc_config)
#define ASRC_RELEASE_PAIR	_IOW(ASRC_IOC_MAGIC, 2, enum asrc_pair_index)
#define ASRC_QUERYBUF	_IOWR(ASRC_IOC_MAGIC, 3, struct asrc_buffer)
#define ASRC_Q_INBUF	_IOW(ASRC_IOC_MAGIC, 4, struct asrc_buffer)
#define ASRC_DQ_INBUF	_IOW(ASRC_IOC_MAGIC, 5, struct asrc_buffer)
#define ASRC_Q_OUTBUF	_IOW(ASRC_IOC_MAGIC, 6, struct asrc_buffer)
#define ASRC_DQ_OUTBUF	_IOW(ASRC_IOC_MAGIC, 7, struct asrc_buffer)
#define ASRC_START_CONV	_IOW(ASRC_IOC_MAGIC, 8, enum asrc_pair_index)
#define ASRC_STOP_CONV	_IOW(ASRC_IOC_MAGIC, 9, enum asrc_pair_index)
#define ASRC_STATUS	_IOW(ASRC_IOC_MAGIC, 10, struct asrc_status_flags)
#define ASRC_FLUSH	_IOW(ASRC_IOC_MAGIC, 11, enum asrc_pair_index)

enum asrc_pair_index {
	ASRC_PAIR_A,
	ASRC_PAIR_B,
	ASRC_PAIR_C
};

enum asrc_inclk {
	INCLK_NONE = 0x03,
	INCLK_ESAI_RX = 0x00,
	INCLK_SSI1_RX = 0x01,
	INCLK_SSI2_RX = 0x02,
	INCLK_SPDIF_RX = 0x04,
	INCLK_MLB_CLK = 0x05,
	INCLK_ESAI_TX = 0x08,
	INCLK_SSI1_TX = 0x09,
	INCLK_SSI2_TX = 0x0a,
	INCLK_SPDIF_TX = 0x0c,
	INCLK_ASRCK1_CLK = 0x0f,
};

enum asrc_outclk {
	OUTCLK_NONE = 0x03,
	OUTCLK_ESAI_TX = 0x00,
	OUTCLK_SSI1_TX = 0x01,
	OUTCLK_SSI2_TX = 0x02,
	OUTCLK_SPDIF_TX = 0x04,
	OUTCLK_MLB_CLK = 0x05,
	OUTCLK_ESAI_RX = 0x08,
	OUTCLK_SSI1_RX = 0x09,
	OUTCLK_SSI2_RX = 0x0a,
	OUTCLK_SPDIF_RX = 0x0c,
	OUTCLK_ASRCK1_CLK = 0x0f,
};

struct asrc_config {
	enum asrc_pair_index pair;
	unsigned int channel_num;
	unsigned int buffer_num;
	unsigned int dma_buffer_size;
	unsigned int input_sample_rate;
	unsigned int output_sample_rate;
	unsigned int word_width;
	enum asrc_inclk inclk;
	enum asrc_outclk outclk;
};

struct asrc_pair {
	unsigned int start_channel;
	unsigned int chn_num;
	unsigned int chn_max;
	unsigned int active;
	unsigned int overload_error;
};

struct asrc_req {
	unsigned int chn_num;
	enum asrc_pair_index index;
};

struct asrc_querybuf {
	unsigned int buffer_index;
	unsigned int input_length;
	unsigned int output_length;
	unsigned long input_offset;
	unsigned long output_offset;
};

struct asrc_buffer {
	unsigned int index;
	unsigned int length;
	int buf_valid;
};

struct asrc_status_flags {
	enum asrc_pair_index index;
	unsigned int overload_error;
};

#define ASRC_BUF_NA	    -35	/* ASRC DQ's buffer is NOT available */
#define ASRC_BUF_AV	    35	/* ASRC DQ's buffer is available */
enum asrc_error_status {
	ASRC_TASK_Q_OVERLOAD = 0x01,
	ASRC_OUTPUT_TASK_OVERLOAD = 0x02,
	ASRC_INPUT_TASK_OVERLOAD = 0x04,
	ASRC_OUTPUT_BUFFER_OVERFLOW = 0x08,
	ASRC_INPUT_BUFFER_UNDERRUN = 0x10,
};

#ifdef __KERNEL__

#define ASRC_DMA_BUFFER_NUM 8

#define ASRC_ASRCTR_REG 	0x00
#define ASRC_ASRIER_REG 	0x04
#define ASRC_ASRCNCR_REG 	0x0C
#define ASRC_ASRCFG_REG 	0x10
#define ASRC_ASRCSR_REG 	0x14
#define ASRC_ASRCDR1_REG 	0x18
#define ASRC_ASRCDR2_REG 	0x1C
#define ASRC_ASRSTR_REG 	0x20
#define ASRC_ASRRA_REG 		0x24
#define ASRC_ASRRB_REG 		0x28
#define ASRC_ASRRC_REG 		0x2C
#define ASRC_ASRPM1_REG 	0x40
#define ASRC_ASRPM2_REG 	0x44
#define ASRC_ASRPM3_REG 	0x48
#define ASRC_ASRPM4_REG 	0x4C
#define ASRC_ASRPM5_REG 	0x50
#define ASRC_ASRTFR1		0x54
#define ASRC_ASRCCR_REG 	0x5C
#define ASRC_ASRDIA_REG 	0x60
#define ASRC_ASRDOA_REG 	0x64
#define ASRC_ASRDIB_REG 	0x68
#define ASRC_ASRDOB_REG 	0x6C
#define ASRC_ASRDIC_REG 	0x70
#define ASRC_ASRDOC_REG 	0x74
#define ASRC_ASRIDRHA_REG 	0x80
#define ASRC_ASRIDRLA_REG 	0x84
#define ASRC_ASRIDRHB_REG 	0x88
#define ASRC_ASRIDRLB_REG 	0x8C
#define ASRC_ASRIDRHC_REG 	0x90
#define ASRC_ASRIDRLC_REG 	0x94
#define ASRC_ASR76K_REG 	0x98
#define ASRC_ASR56K_REG 	0x9C

struct dma_block {
	unsigned int index;
	unsigned int length;
	unsigned char *dma_vaddr;
	dma_addr_t dma_paddr;
	struct list_head queue;
};

struct asrc_pair_params {
	enum asrc_pair_index index;
	struct list_head input_queue;
	struct list_head input_done_queue;
	struct list_head output_queue;
	struct list_head output_done_queue;
	wait_queue_head_t input_wait_queue;
	wait_queue_head_t output_wait_queue;
	unsigned int input_counter;
	unsigned int output_counter;
	unsigned int input_queue_empty;
	unsigned int output_queue_empty;
	unsigned int input_dma_channel;
	unsigned int output_dma_channel;
	unsigned int input_buffer_size;
	unsigned int output_buffer_size;
	unsigned int buffer_num;
	unsigned int pair_hold;
	unsigned int asrc_active;
	struct dma_block input_dma[ASRC_DMA_BUFFER_NUM];
	struct dma_block output_dma[ASRC_DMA_BUFFER_NUM];
	struct semaphore busy_lock;
};

struct asrc_data {
	struct asrc_pair asrc_pair[3];
};

extern int asrc_req_pair(int chn_num, enum asrc_pair_index *index);
extern void asrc_release_pair(enum asrc_pair_index index);
extern int asrc_config_pair(struct asrc_config *config);
extern void asrc_get_status(struct asrc_status_flags *flags);
extern void asrc_start_conv(enum asrc_pair_index index);
extern void asrc_stop_conv(enum asrc_pair_index index);

#endif				/* __kERNEL__ */

#endif				/* __MXC_ASRC_H__ */
