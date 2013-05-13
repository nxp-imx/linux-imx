/*
 * Copyright 2008-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#define ASRC_CONVERT	_IOW(ASRC_IOC_MAGIC, 3, struct asrc_convert_buffer)
#define ASRC_START_CONV	_IOW(ASRC_IOC_MAGIC, 4, enum asrc_pair_index)
#define ASRC_STOP_CONV	_IOW(ASRC_IOC_MAGIC, 5, enum asrc_pair_index)
#define ASRC_STATUS	_IOW(ASRC_IOC_MAGIC, 6, struct asrc_status_flags)
#define ASRC_FLUSH	_IOW(ASRC_IOC_MAGIC, 7, enum asrc_pair_index)

enum asrc_pair_index {
	ASRC_UNVALID_PAIR = -1,
	ASRC_PAIR_A,
	ASRC_PAIR_B,
	ASRC_PAIR_C
};

#define ASRC_PAIR_MAX_NUM	(ASRC_PAIR_C + 1)

enum asrc_inclk {
	INCLK_NONE = 0x03,
	INCLK_ESAI_RX = 0x00,
	INCLK_SSI1_RX = 0x01,
	INCLK_SSI2_RX = 0x02,
	INCLK_SSI3_RX = 0x07,
	INCLK_SPDIF_RX = 0x04,
	INCLK_MLB_CLK = 0x05,
	INCLK_PAD = 0x06,
	INCLK_ESAI_TX = 0x08,
	INCLK_SSI1_TX = 0x09,
	INCLK_SSI2_TX = 0x0a,
	INCLK_SSI3_TX = 0x0b,
	INCLK_SPDIF_TX = 0x0c,
	INCLK_ASRCK1_CLK = 0x0f,
};

enum asrc_outclk {
	OUTCLK_NONE = 0x03,
	OUTCLK_ESAI_TX = 0x00,
	OUTCLK_SSI1_TX = 0x01,
	OUTCLK_SSI2_TX = 0x02,
	OUTCLK_SSI3_TX = 0x07,
	OUTCLK_SPDIF_TX = 0x04,
	OUTCLK_MLB_CLK = 0x05,
	OUTCLK_PAD = 0x06,
	OUTCLK_ESAI_RX = 0x08,
	OUTCLK_SSI1_RX = 0x09,
	OUTCLK_SSI2_RX = 0x0a,
	OUTCLK_SSI3_RX = 0x0b,
	OUTCLK_SPDIF_RX = 0x0c,
	OUTCLK_ASRCK1_CLK = 0x0f,
};

enum asrc_word_width {
	ASRC_WIDTH_24_BIT = 0,
	ASRC_WIDTH_16_BIT = 1,
	ASRC_WIDTH_8_BIT = 2,
};

struct asrc_config {
	enum asrc_pair_index pair;
	unsigned int channel_num;
	unsigned int buffer_num;
	unsigned int dma_buffer_size;
	unsigned int input_sample_rate;
	unsigned int output_sample_rate;
	enum asrc_word_width input_word_width;
	enum asrc_word_width output_word_width;
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

struct asrc_convert_buffer {
	void *input_buffer_vaddr;
	void *output_buffer_vaddr;
	unsigned int input_buffer_length;
	unsigned int output_buffer_length;
};

struct asrc_buffer {
	unsigned int index;
	unsigned int length;
	unsigned int output_last_length;
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
#include <linux/scatterlist.h>

#define ASRC_DMA_BUFFER_NUM		2
#define ASRC_INPUTFIFO_THRESHOLD	32
#define ASRC_OUTPUTFIFO_THRESHOLD	32
#define ASRC_DMA_BUFFER_SIZE	(1024 * 48 * 4)
#define ASRC_MAX_BUFFER_SIZE	(1024 * 48)
#define ASRC_OUTPUT_LAST_SAMPLE	8


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
#define ASRC_ASRMCRA_REG    0xA0
#define ASRC_ASRFSTA_REG    0xA4
#define ASRC_ASRMCRB_REG    0xA8
#define ASRC_ASRFSTB_REG    0xAC
#define ASRC_ASRMCRC_REG    0xB0
#define ASRC_ASRFSTC_REG    0xB4
#define ASRC_ASRMCR1A_REG   0xC0
#define ASRC_ASRMCR1B_REG   0xC4
#define ASRC_ASRMCR1C_REG   0xC8


#define ASRC_ASRFSTX_INPUT_FIFO_WIDTH	7
#define ASRC_ASRFSTX_INPUT_FIFO_OFFSET	0
#define ASRC_ASRFSTX_INPUT_FIFO_MASK	0x7F

#define ASRC_ASRFSTX_OUTPUT_FIFO_WIDTH	7
#define ASRC_ASRFSTX_OUTPUT_FIFO_OFFSET	12
#define ASRC_ASRFSTX_OUTPUT_FIFO_MASK (0x7F << ASRC_ASRFSTX_OUTPUT_FIFO_OFFSET)


struct dma_block {
	unsigned int index;
	unsigned int length;
	void *dma_vaddr;
	dma_addr_t dma_paddr;
	struct list_head queue;
};

struct asrc_p2p_params {
	u32 p2p_rate;/* ASRC output rate for p2p */
	enum asrc_word_width p2p_width;/* ASRC output wordwidth for p2p */
};

struct asrc_pair_params {
	enum asrc_pair_index index;
	wait_queue_head_t input_wait_queue;
	wait_queue_head_t output_wait_queue;
	unsigned int input_counter;
	unsigned int output_counter;
	struct dma_chan *input_dma_channel;
	struct dma_chan *output_dma_channel;
	unsigned int input_buffer_size;
	unsigned int output_buffer_size;
	unsigned int buffer_num;
	unsigned int pair_hold;
	unsigned int asrc_active;
	unsigned int channel_nums;
	struct dma_block input_dma_total;
	struct dma_block input_dma[ASRC_DMA_BUFFER_NUM];
	struct dma_block output_dma_total;
	struct dma_block output_dma[ASRC_DMA_BUFFER_NUM];
	struct dma_block output_last_period;
	struct dma_async_tx_descriptor *desc_in;
	struct dma_async_tx_descriptor *desc_out;
	struct work_struct task_output_work;
	unsigned int		input_sg_nodes;
	unsigned int		output_sg_nodes;
	struct scatterlist	input_sg[4], output_sg[4];
	enum asrc_word_width input_word_width;
	enum asrc_word_width output_word_width;
	u32 input_sample_rate;
	u32 output_sample_rate;
	u32 input_wm;
	u32 output_wm;
};

struct asrc_data {
	struct asrc_pair asrc_pair[3];
	struct proc_dir_entry *proc_asrc;
	unsigned long vaddr;
	unsigned long paddr;
	int dmarx[3];
	int dmatx[3];
	struct class *asrc_class;
	int asrc_major;
	struct imx_asrc_platform_data *mxc_asrc_data;
	struct device *dev;
};

struct asrc_p2p_ops {
	void (*asrc_p2p_start_conv)(enum asrc_pair_index);
	void (*asrc_p2p_stop_conv)(enum asrc_pair_index);
	int (*asrc_p2p_get_dma_request)(enum asrc_pair_index, bool);
	u32 (*asrc_p2p_per_addr)(enum asrc_pair_index, bool);
	int (*asrc_p2p_req_pair)(int, enum asrc_pair_index *index);
	int (*asrc_p2p_config_pair)(struct asrc_config *config);
	void (*asrc_p2p_release_pair)(enum asrc_pair_index);
	void (*asrc_p2p_finish_conv)(enum asrc_pair_index);
};

extern void asrc_p2p_hook(struct asrc_p2p_ops *asrc_p2p_ct);

extern int asrc_req_pair(int chn_num, enum asrc_pair_index *index);
extern void asrc_release_pair(enum asrc_pair_index index);
extern int asrc_config_pair(struct asrc_config *config);
extern void asrc_get_status(struct asrc_status_flags *flags);
extern void asrc_start_conv(enum asrc_pair_index index);
extern void asrc_stop_conv(enum asrc_pair_index index);
extern u32 asrc_get_per_addr(enum asrc_pair_index index, bool i);
extern int asrc_get_dma_request(enum asrc_pair_index index, bool i);
extern void asrc_finish_conv(enum asrc_pair_index index);
extern int asrc_set_watermark(enum asrc_pair_index index,
					u32 in_wm, u32 out_wm);

#endif				/* __kERNEL__ */

#endif				/* __MXC_ASRC_H__ */
