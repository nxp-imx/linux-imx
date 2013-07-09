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
 * @file mxc_asrc.c
 *
 * @brief MXC Asynchronous Sample Rate Converter
 *
 * @ingroup SOUND
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/mxc_asrc.h>
#include <linux/fsl_devices.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/memory.h>
#include <mach/dma.h>
#include <mach/mxc_asrc.h>
#include <linux/delay.h>


#define ASRC_PROC_PATH        "driver/asrc"

#define ASRC_RATIO_DECIMAL_DEPTH 26

DEFINE_SPINLOCK(data_lock);
DEFINE_SPINLOCK(pair_lock);
DEFINE_SPINLOCK(input_int_lock);
DEFINE_SPINLOCK(output_int_lock);

#define AICPA		0	/* Input Clock Prescaler A Offset */
#define AICDA		3	/* Input Clock Divider A Offset */
#define AICPB           6	/* Input Clock Prescaler B Offset */
#define AICDB           9	/* Input Clock Divider B Offset */
#define AOCPA           12	/* Output Clock Prescaler A Offset */
#define AOCDA           15	/* Output Clock Divider A Offset */
#define AOCPB           18	/* Output Clock Prescaler B Offset */
#define AOCDB           21	/* Output Clock Divider B Offset */
#define AICPC           0	/* Input Clock Prescaler C Offset */
#define AICDC           3	/* Input Clock Divider C Offset */
#define AOCPC           6	/* Output Clock Prescaler C Offset */
#define AOCDC           9	/* Output Clock Divider C Offset */

char *asrc_pair_id[] = {
	[0] = "ASRC RX PAIR A",
	[1] = "ASRC TX PAIR A",
	[2] = "ASRC RX PAIR B",
	[3] = "ASRC TX PAIR B",
	[4] = "ASRC RX PAIR C",
	[5] = "ASRC TX PAIR C",
};

enum asrc_status {
	ASRC_ASRSTR_AIDEA = 0x01,
	ASRC_ASRSTR_AIDEB = 0x02,
	ASRC_ASRSTR_AIDEC = 0x04,
	ASRC_ASRSTR_AODFA = 0x08,
	ASRC_ASRSTR_AODFB = 0x10,
	ASRC_ASRSTR_AODFC = 0x20,
	ASRC_ASRSTR_AOLE = 0x40,
	ASRC_ASRSTR_FPWT = 0x80,
	ASRC_ASRSTR_AIDUA = 0x100,
	ASRC_ASRSTR_AIDUB = 0x200,
	ASRC_ASRSTR_AIDUC = 0x400,
	ASRC_ASRSTR_AODOA = 0x800,
	ASRC_ASRSTR_AODOB = 0x1000,
	ASRC_ASRSTR_AODOC = 0x2000,
	ASRC_ASRSTR_AIOLA = 0x4000,
	ASRC_ASRSTR_AIOLB = 0x8000,
	ASRC_ASRSTR_AIOLC = 0x10000,
	ASRC_ASRSTR_AOOLA = 0x20000,
	ASRC_ASRSTR_AOOLB = 0x40000,
	ASRC_ASRSTR_AOOLC = 0x80000,
	ASRC_ASRSTR_ATQOL = 0x100000,
	ASRC_ASRSTR_DSLCNT = 0x200000,
};

/* Sample rates are aligned with that defined in pcm.h file */
static const unsigned char asrc_process_table[][8][2] = {
	/* 32kHz 44.1kHz 48kHz   64kHz   88.2kHz 96kHz  176kHz   192kHz */
/*5512Hz*/
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*8kHz*/
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*11025Hz*/
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*16kHz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*22050Hz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},
/*32kHz*/
	{{0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0},},
/*44.1kHz*/
	{{0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0},},
/*48kHz*/
	{{0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0},},
/*64kHz*/
	{{1, 2}, {0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0},},
/*88.2kHz*/
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},
/*96kHz*/
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},
/*176kHz*/
	{{2, 2}, {2, 2}, {2, 2}, {2, 1}, {2, 1}, {2, 1}, {2, 1}, {2, 1},},
/*192kHz*/
	{{2, 2}, {2, 2}, {2, 2}, {2, 1}, {2, 1}, {2, 1}, {2, 1}, {2, 1},},
};

static struct asrc_data *g_asrc;

/* The following tables map the relationship between asrc_inclk/asrc_outclk in
 * mxc_asrc.h and the registers of ASRCSR
 */
static unsigned char input_clk_map_v1[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
};

static unsigned char output_clk_map_v1[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
};

/* V2 uses the same map for input and output */
static unsigned char input_clk_map_v2[] = {
/*	0x0  0x1  0x2  0x3  0x4  0x5  0x6  0x7  0x8  0x9  0xa  0xb  0xc  0xd  0xe  0xf*/
	0x0, 0x1, 0x2, 0x7, 0x4, 0x5, 0x6, 0x3, 0x8, 0x9, 0xa, 0xb, 0xc, 0xf, 0xe, 0xd,
};

static unsigned char output_clk_map_v2[] = {
/*	0x0  0x1  0x2  0x3  0x4  0x5  0x6  0x7  0x8  0x9  0xa  0xb  0xc  0xd  0xe  0xf*/
	0x8, 0x9, 0xa, 0x7, 0xc, 0x5, 0x6, 0xb, 0x0, 0x1, 0x2, 0x3, 0x4, 0xf, 0xe, 0xd,
};

static unsigned char *input_clk_map, *output_clk_map;

struct asrc_p2p_ops asrc_pcm_p2p_ops_asrc;

static struct dma_chan *imx_asrc_dma_alloc(u32 dma_req);
static int imx_asrc_dma_config(
					struct asrc_pair_params *params,
					struct dma_chan *chan,
					u32 dma_addr, void *buf_addr,
					u32 buf_len, bool in,
					enum asrc_word_width word_width);

static int asrc_set_clock_ratio(enum asrc_pair_index index,
				int input_sample_rate, int output_sample_rate)
{
	int i;
	int integ = 0;
	unsigned long reg_val = 0;

	if (output_sample_rate == 0)
		return -1;
	while (input_sample_rate >= output_sample_rate) {
		input_sample_rate -= output_sample_rate;
		integ++;
	}
	reg_val |= (integ << 26);

	for (i = 1; i <= ASRC_RATIO_DECIMAL_DEPTH; i++) {
		if ((input_sample_rate * 2) >= output_sample_rate) {
			reg_val |= (1 << (ASRC_RATIO_DECIMAL_DEPTH - i));
			input_sample_rate =
			    input_sample_rate * 2 - output_sample_rate;
		} else
			input_sample_rate = input_sample_rate << 1;

		if (input_sample_rate == 0)
			break;
	}

	__raw_writel(reg_val,
		(g_asrc->vaddr + ASRC_ASRIDRLA_REG + (index << 3)));
	__raw_writel((reg_val >> 24),
		(g_asrc->vaddr + ASRC_ASRIDRHA_REG + (index << 3)));
	return 0;
}

static int asrc_set_process_configuration(enum asrc_pair_index index,
					  int input_sample_rate,
					  int output_sample_rate)
{
	int i = 0, j = 0;
	unsigned long reg;
	switch (input_sample_rate) {
	case 5512:
		i = 0;
	case 8000:
		i = 1;
		break;
	case 11025:
		i = 2;
		break;
	case 16000:
		i = 3;
		break;
	case 22050:
		i = 4;
		break;
	case 32000:
		i = 5;
		break;
	case 44100:
		i = 6;
		break;
	case 48000:
		i = 7;
		break;
	case 64000:
		i = 8;
		break;
	case 88200:
		i = 9;
		break;
	case 96000:
		i = 10;
		break;
	case 176400:
		i = 11;
		break;
	case 192000:
		i = 12;
		break;
	default:
		return -1;
	}

	switch (output_sample_rate) {
	case 32000:
		j = 0;
		break;
	case 44100:
		j = 1;
		break;
	case 48000:
		j = 2;
		break;
	case 64000:
		j = 3;
		break;
	case 88200:
		j = 4;
		break;
	case 96000:
		j = 5;
		break;
	case 176400:
		j = 6;
		break;
	case 192000:
		j = 7;
		break;
	default:
		return -1;
	}

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCFG_REG);
	reg &= ~(0x0f << (6 + (index << 2)));
	reg |=
	    ((asrc_process_table[i][j][0] << (6 + (index << 2))) |
	     (asrc_process_table[i][j][1] << (8 + (index << 2))));
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCFG_REG);

	return 0;
}

static int asrc_get_asrck_clock_divider(int samplerate)
{
	unsigned int prescaler, divider;
	unsigned int i;
	unsigned int ratio, ra;
	unsigned long bitclk;

	bitclk = clk_get_rate(g_asrc->mxc_asrc_data->asrc_audio_clk);

	ra = bitclk/samplerate;
	ratio = ra;
	/*calculate the prescaler*/
	i = 0;
	while (ratio > 8) {
		i++;
		ratio = ratio >> 1;
	}
	prescaler = i;
	/*calculate the divider*/
	if (i >= 1)
		divider = ((ra + (1 << (i - 1)) - 1) >> i) - 1;
	else
		divider = ra - 1;
	/*the totally divider is (2^prescaler)*divider*/
	return (divider << 3) + prescaler;
}

int asrc_req_pair(int chn_num, enum asrc_pair_index *index)
{
	int err = 0;
	unsigned long lock_flags;
	struct asrc_pair *pair;
	int imax = 0, busy = 0, i;

	spin_lock_irqsave(&data_lock, lock_flags);

	for (i = ASRC_PAIR_A; i < ASRC_PAIR_MAX_NUM; i++) {
		pair = &g_asrc->asrc_pair[i];
		if (chn_num > pair->chn_max) {
			imax++;
			continue;
		} else if (pair->active) {
			busy++;
			continue;
		}
		/* Save the current qualified pair */
		*index = i;

		/* Check if this pair is a perfect one */
		if (chn_num == pair->chn_max)
			break;
	}

	if (imax >= ASRC_PAIR_MAX_NUM) {
		pr_err("No pair could afford requested channel number.\n");
		err = -EINVAL;
	} else if (busy >= ASRC_PAIR_MAX_NUM) {
		pr_err("All pairs are busy now.\n");
		err = -EBUSY;
	} else if (busy + imax >= ASRC_PAIR_MAX_NUM) {
		pr_err("All affordable pairs are busy now.\n");
		err = -EBUSY;
	} else {
		pair = &g_asrc->asrc_pair[*index];
		pair->chn_num = chn_num;
		pair->active = 1;
	}

	spin_unlock_irqrestore(&data_lock, lock_flags);

	if (!err) {
		clk_enable(g_asrc->mxc_asrc_data->asrc_core_clk);
		clk_enable(g_asrc->mxc_asrc_data->asrc_audio_clk);
	}
	return err;
}

EXPORT_SYMBOL(asrc_req_pair);

void asrc_release_pair(enum asrc_pair_index index)
{
	unsigned long reg;
	unsigned long lock_flags;
	struct asrc_pair *pair;
	pair = &g_asrc->asrc_pair[index];

	spin_lock_irqsave(&data_lock, lock_flags);
	pair->active = 0;
	pair->overload_error = 0;
	/********Disable PAIR*************/
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
	reg &= ~(1 << (index + 1));
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);
	spin_unlock_irqrestore(&data_lock, lock_flags);
}

EXPORT_SYMBOL(asrc_release_pair);

int asrc_config_pair(struct asrc_config *config)
{
	int err = 0;
	int reg, tmp, channel_num;
	unsigned long lock_flags;
	unsigned long aicp_shift, aocp_shift;
	unsigned long asrc_asrcdr_reg, dp_clear_mask;

	/* Set the channel number */
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCNCR_REG);
	spin_lock_irqsave(&data_lock, lock_flags);
	g_asrc->asrc_pair[config->pair].chn_num = config->channel_num;
	spin_unlock_irqrestore(&data_lock, lock_flags);
	reg &=
	    ~((0xFFFFFFFF >> (32 - g_asrc->mxc_asrc_data->channel_bits)) <<
	      (g_asrc->mxc_asrc_data->channel_bits * config->pair));
	if (g_asrc->mxc_asrc_data->channel_bits > 3)
		channel_num = config->channel_num;
	else
		channel_num = (config->channel_num + 1) / 2;
	tmp = channel_num <<
		(g_asrc->mxc_asrc_data->channel_bits * config->pair);
	reg |= tmp;
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCNCR_REG);

	/* Set the clock source */
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCSR_REG);
	tmp = ~(0x0f << (config->pair << 2));
	reg &= tmp;
	tmp = ~(0x0f << (12 + (config->pair << 2)));
	reg &= tmp;
	reg |= ((input_clk_map[config->inclk] << (config->pair << 2)) |
		(output_clk_map[config->outclk] << (12 + (config->pair << 2))));

	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCSR_REG);

	/* default setting */
	/* automatic selection for processing mode */
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
	reg |= (1 << (20 + config->pair));
	reg &= ~(1 << (14 + (config->pair << 1)));

	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRRA_REG);
	reg &= 0xffbfffff;
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRRA_REG);

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
	reg = reg & (~(1 << 23));
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);

	/* Default Clock Divider Setting */
	switch (config->pair) {
	case ASRC_PAIR_A:
		asrc_asrcdr_reg = ASRC_ASRCDR1_REG;
		dp_clear_mask = 0xfc0fc0;
		aicp_shift = AICPA;
		aocp_shift = AOCPA;
		break;
	case ASRC_PAIR_B:
		asrc_asrcdr_reg = ASRC_ASRCDR1_REG;
		dp_clear_mask = 0x03f03f;
		aicp_shift = AICPB;
		aocp_shift = AOCPB;
		break;
	case ASRC_PAIR_C:
		asrc_asrcdr_reg = ASRC_ASRCDR2_REG;
		dp_clear_mask = 0x00;
		aicp_shift = AICPC;
		aocp_shift = AOCPC;
		break;
	default:
		pr_err("Invalid Pair number %d\n", config->pair);
		return -EFAULT;
	}

	reg = __raw_readl(g_asrc->vaddr + asrc_asrcdr_reg);
	reg &= dp_clear_mask;
	/* Input Part */
	if ((config->inclk & 0x0f) == INCLK_SPDIF_RX)
		reg |= 7 << aicp_shift;
	else if ((config->inclk & 0x0f) == INCLK_SPDIF_TX)
		reg |= 6 << aicp_shift;
	else if ((config->inclk & 0x0f) == INCLK_ASRCK1_CLK) {
		tmp = asrc_get_asrck_clock_divider(config->input_sample_rate);
		reg |= tmp << aicp_shift;
	} else {
		if (config->input_word_width == ASRC_WIDTH_16_BIT)
			reg |= 5 << aicp_shift;
		else if (config->input_word_width == ASRC_WIDTH_24_BIT)
			reg |= 6 << aicp_shift;
		else
			err = -EFAULT;
	}
	/* Output Part */
	if ((config->outclk & 0x0f) == OUTCLK_SPDIF_RX)
		reg |= 7 << aocp_shift;
	else if ((config->outclk & 0x0f) == OUTCLK_SPDIF_TX)
		reg |= 6 << aocp_shift;
	else if (((config->outclk & 0x0f) == OUTCLK_ASRCK1_CLK)
			&& ((config->inclk & 0x0f) == INCLK_NONE))
		reg |= 5 << aocp_shift;
	else if ((config->outclk & 0x0f) == OUTCLK_ASRCK1_CLK) {
		tmp = asrc_get_asrck_clock_divider(config->output_sample_rate);
		reg |= tmp << aocp_shift;
	} else {
		if (config->output_word_width == ASRC_WIDTH_16_BIT)
			reg |= 5 << aocp_shift;
		else if (config->output_word_width == ASRC_WIDTH_24_BIT)
			reg |= 6 << aocp_shift;
		else
			err = -EFAULT;
	}
	__raw_writel(reg, g_asrc->vaddr + asrc_asrcdr_reg);

	/* check whether ideal ratio is a must */
	if ((config->inclk & 0x0f) == INCLK_NONE) {
		reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
		reg &= ~(1 << (20 + config->pair));
		reg |= (0x03 << (13 + (config->pair << 1)));
		__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);
		err = asrc_set_clock_ratio(config->pair,
					   config->input_sample_rate,
					   config->output_sample_rate);
		if (err < 0)
			return err;

		err = asrc_set_process_configuration(config->pair,
						     config->input_sample_rate,
						     config->
						     output_sample_rate);
		if (err < 0)
			return err;
	} else if ((config->inclk & 0x0f) == INCLK_ASRCK1_CLK) {
		if (config->input_sample_rate == 44100
		    || config->input_sample_rate == 88200) {
			pr_err("ASRC core clock cann't support sample rate %d\n",
			     config->input_sample_rate);
			err = -EFAULT;
		}
	} else if ((config->outclk & 0x0f) == OUTCLK_ASRCK1_CLK) {
		if (config->output_sample_rate == 44100
		    || config->output_sample_rate == 88200) {
			pr_err("ASRC core clock cann't support sample rate %d\n",
			     config->input_sample_rate);
			err = -EFAULT;
		}
	}

	/* Config input and output wordwidth */
	reg = __raw_readl(
		g_asrc->vaddr + ASRC_ASRMCR1A_REG + (config->pair << 2));
	/* BIT 11-9 stands for input word width,
	 * BIT 0 stands for output word width */
	reg &= ~0xE01;
	switch (config->input_word_width) {
	case ASRC_WIDTH_16_BIT:
		reg |= 1 << 9;
		break;
	case ASRC_WIDTH_24_BIT:
		reg |= 0 << 9;
		break;
	default:
		return -EINVAL;
	}

	switch (config->output_word_width) {
	case ASRC_WIDTH_16_BIT:
		reg |= 1;
		break;
	case ASRC_WIDTH_24_BIT:
		reg |= 0;
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(reg,
		 g_asrc->vaddr + ASRC_ASRMCR1A_REG + (config->pair << 2));


	/* Enable BUFFER STALL*/
	reg = __raw_readl(
		g_asrc->vaddr + ASRC_ASRMCRA_REG + (config->pair << 3));
	reg |= 1 << 21;
	reg |= 1 << 22;
	reg &= ~0x3f;
	reg += ASRC_INPUTFIFO_THRESHOLD;
	reg &= ~(0x3f << 12);
	reg += ASRC_OUTPUTFIFO_THRESHOLD << 12;
	__raw_writel(reg,
		g_asrc->vaddr + ASRC_ASRMCRA_REG + (config->pair << 3));

	return err;
}

EXPORT_SYMBOL(asrc_config_pair);

int asrc_set_watermark(enum asrc_pair_index index, u32 in_wm, u32 out_wm)
{
	u32 reg;

	if ((in_wm > 63) || (out_wm > 63)) {
		pr_err("error watermark!\n");
		return -EINVAL;
	}

	reg = __raw_readl(
		g_asrc->vaddr + ASRC_ASRMCRA_REG + (index << 3));
	reg |= 1 << 22;
	reg &= ~0x3f;
	reg += in_wm;
	reg &= ~(0x3f << 12);
	reg += out_wm << 12;
	__raw_writel(reg,
		g_asrc->vaddr + ASRC_ASRMCRA_REG + (index << 3));
	return 0;
}
EXPORT_SYMBOL(asrc_set_watermark);

void asrc_start_conv(enum asrc_pair_index index)
{
	int reg, reg_1;
	unsigned long lock_flags;
	int i;

	spin_lock_irqsave(&data_lock, lock_flags);
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
	reg |= (1 << (1 + index));
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCFG_REG);
	while (!(reg & (1 << (index + 21))))
		reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCFG_REG);
	reg_1 = __raw_readl(g_asrc->vaddr + ASRC_ASRSTR_REG);

	reg = 0;
	for (i = 0; i < 20; i++) {
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
		__raw_writel(reg,
			     g_asrc->vaddr + ASRC_ASRDIA_REG +
			     (index << 3));
	}

	__raw_writel(0x40, g_asrc->vaddr + ASRC_ASRIER_REG);
	spin_unlock_irqrestore(&data_lock, lock_flags);
	return;
}

EXPORT_SYMBOL(asrc_start_conv);

void asrc_stop_conv(enum asrc_pair_index index)
{
	int reg;
	unsigned long lock_flags;
	spin_lock_irqsave(&data_lock, lock_flags);
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCTR_REG);
	reg &= ~(1 << (1 + index));
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCTR_REG);
	spin_unlock_irqrestore(&data_lock, lock_flags);

	return;
}

EXPORT_SYMBOL(asrc_stop_conv);

void asrc_finish_conv(enum asrc_pair_index index)
{
	clk_disable(g_asrc->mxc_asrc_data->asrc_audio_clk);
	clk_disable(g_asrc->mxc_asrc_data->asrc_core_clk);

	return;
}
EXPORT_SYMBOL(asrc_finish_conv);

int asrc_get_dma_request(enum asrc_pair_index index, bool in)
{
	if (in)
		return g_asrc->dmarx[index];
	else
		return g_asrc->dmatx[index];
}

EXPORT_SYMBOL(asrc_get_dma_request);

/*!
 * @brief asrc interrupt handler
 */
static irqreturn_t asrc_isr(int irq, void *dev_id)
{
	unsigned long status;
	int reg = 0x40;

	status = __raw_readl(g_asrc->vaddr + ASRC_ASRSTR_REG);
	if (g_asrc->asrc_pair[ASRC_PAIR_A].active == 1) {
		if (status & ASRC_ASRSTR_ATQOL)
			g_asrc->asrc_pair[ASRC_PAIR_A].overload_error |=
			    ASRC_TASK_Q_OVERLOAD;
		if (status & ASRC_ASRSTR_AOOLA)
			g_asrc->asrc_pair[ASRC_PAIR_A].overload_error |=
			    ASRC_OUTPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AIOLA)
			g_asrc->asrc_pair[ASRC_PAIR_A].overload_error |=
			    ASRC_INPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AODOA)
			g_asrc->asrc_pair[ASRC_PAIR_A].overload_error |=
			    ASRC_OUTPUT_BUFFER_OVERFLOW;
		if (status & ASRC_ASRSTR_AIDUA)
			g_asrc->asrc_pair[ASRC_PAIR_A].overload_error |=
			    ASRC_INPUT_BUFFER_UNDERRUN;
	} else if (g_asrc->asrc_pair[ASRC_PAIR_B].active == 1) {
		if (status & ASRC_ASRSTR_ATQOL)
			g_asrc->asrc_pair[ASRC_PAIR_B].overload_error |=
			    ASRC_TASK_Q_OVERLOAD;
		if (status & ASRC_ASRSTR_AOOLB)
			g_asrc->asrc_pair[ASRC_PAIR_B].overload_error |=
			    ASRC_OUTPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AIOLB)
			g_asrc->asrc_pair[ASRC_PAIR_B].overload_error |=
			    ASRC_INPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AODOB)
			g_asrc->asrc_pair[ASRC_PAIR_B].overload_error |=
			    ASRC_OUTPUT_BUFFER_OVERFLOW;
		if (status & ASRC_ASRSTR_AIDUB)
			g_asrc->asrc_pair[ASRC_PAIR_B].overload_error |=
			    ASRC_INPUT_BUFFER_UNDERRUN;
	} else if (g_asrc->asrc_pair[ASRC_PAIR_C].active == 1) {
		if (status & ASRC_ASRSTR_ATQOL)
			g_asrc->asrc_pair[ASRC_PAIR_C].overload_error |=
			    ASRC_TASK_Q_OVERLOAD;
		if (status & ASRC_ASRSTR_AOOLC)
			g_asrc->asrc_pair[ASRC_PAIR_C].overload_error |=
			    ASRC_OUTPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AIOLC)
			g_asrc->asrc_pair[ASRC_PAIR_C].overload_error |=
			    ASRC_INPUT_TASK_OVERLOAD;
		if (status & ASRC_ASRSTR_AODOC)
			g_asrc->asrc_pair[ASRC_PAIR_C].overload_error |=
			    ASRC_OUTPUT_BUFFER_OVERFLOW;
		if (status & ASRC_ASRSTR_AIDUC)
			g_asrc->asrc_pair[ASRC_PAIR_C].overload_error |=
			    ASRC_INPUT_BUFFER_UNDERRUN;
	}
	/* try to clean the overload error  */
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRSTR_REG);

	return IRQ_HANDLED;
}

void asrc_get_status(struct asrc_status_flags *flags)
{
	unsigned long lock_flags;
	enum asrc_pair_index index;

	spin_lock_irqsave(&data_lock, lock_flags);
	index = flags->index;
	flags->overload_error = g_asrc->asrc_pair[index].overload_error;

	spin_unlock_irqrestore(&data_lock, lock_flags);
	return;
}

EXPORT_SYMBOL(asrc_get_status);

u32 asrc_get_per_addr(enum asrc_pair_index index, bool in)
{
	if (in)
		return g_asrc->paddr + ASRC_ASRDIA_REG + (index << 3);
	else
		return g_asrc->paddr + ASRC_ASRDOA_REG + (index << 3);
}

EXPORT_SYMBOL(asrc_get_per_addr);

static int mxc_init_asrc(void)
{
	/* Halt ASRC internal FP when input FIFO needs data for pair A, B, C */
	__raw_writel(0x0001, g_asrc->vaddr + ASRC_ASRCTR_REG);

	/* Enable overflow interrupt */
	__raw_writel(0x00, g_asrc->vaddr + ASRC_ASRIER_REG);

	/* Default 2: 6: 2 channel assignment */
	__raw_writel((0x02 << g_asrc->mxc_asrc_data->channel_bits *
		      2) | (0x06 << g_asrc->mxc_asrc_data->channel_bits) | 0x02,
		     g_asrc->vaddr + ASRC_ASRCNCR_REG);

	/* Parameter Registers recommended settings */
	__raw_writel(0x7fffff, g_asrc->vaddr + ASRC_ASRPM1_REG);
	__raw_writel(0x255555, g_asrc->vaddr + ASRC_ASRPM2_REG);
	__raw_writel(0xff7280, g_asrc->vaddr + ASRC_ASRPM3_REG);
	__raw_writel(0xff7280, g_asrc->vaddr + ASRC_ASRPM4_REG);
	__raw_writel(0xff7280, g_asrc->vaddr + ASRC_ASRPM5_REG);

	__raw_writel(0x001f00, g_asrc->vaddr + ASRC_ASRTFR1);

	/* Set the processing clock for 76KHz, 133M  */
	__raw_writel(0x06D6, g_asrc->vaddr + ASRC_ASR76K_REG);

	/* Set the processing clock for 56KHz, 133M */
	__raw_writel(0x0947, g_asrc->vaddr + ASRC_ASR56K_REG);

	return 0;
}

static void asrc_input_dma_callback(void *data)
{
	struct asrc_pair_params *params;
	unsigned long lock_flags;

	params = data;
	dma_unmap_sg(NULL, params->input_sg,
		params->input_sg_nodes, DMA_MEM_TO_DEV);
	spin_lock_irqsave(&input_int_lock, lock_flags);
	params->input_counter++;
	wake_up_interruptible(&params->input_wait_queue);
	spin_unlock_irqrestore(&input_int_lock, lock_flags);
	schedule_work(&params->task_output_work);
	return;
}

static void asrc_output_dma_callback(void *data)
{
	struct asrc_pair_params *params;

	params = data;
	dma_unmap_sg(NULL, params->output_sg,
		params->output_sg_nodes, DMA_MEM_TO_DEV);
	return;
}

static unsigned int asrc_get_output_FIFO_size(enum asrc_pair_index index)
{
	u32 reg;

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRFSTA_REG + (index << 3));
	return (reg & ASRC_ASRFSTX_OUTPUT_FIFO_MASK)
			>> ASRC_ASRFSTX_OUTPUT_FIFO_OFFSET;
}

static unsigned int asrc_get_input_FIFO_size(enum asrc_pair_index index)
{
	u32 reg;

	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRFSTA_REG + (index << 3));
	return (reg & ASRC_ASRFSTX_INPUT_FIFO_MASK) >>
				ASRC_ASRFSTX_INPUT_FIFO_OFFSET;
}


static u32 asrc_read_one_from_output_FIFO(enum asrc_pair_index index)
{
	return __raw_readl(g_asrc->vaddr + ASRC_ASRDOA_REG + (index << 3));
}

static void asrc_write_one_to_output_FIFO(enum asrc_pair_index index, u32 value)
{
	__raw_writel(value, g_asrc->vaddr + ASRC_ASRDIA_REG + (index << 3));
}

static void asrc_read_output_FIFO_S16(struct asrc_pair_params *params)
{
	u32 size, i, j, reg, t_size;
	u16 *index = params->output_last_period.dma_vaddr;

	t_size = 0;
	udelay(100);
	size = asrc_get_output_FIFO_size(params->index);
	while (size) {
		for (i = 0; i < size; i++) {
			for (j = 0; j < params->channel_nums; j++) {
				reg = asrc_read_one_from_output_FIFO(
							params->index);
				*(index) = (u16)reg;
				index++;
			}
		}
		t_size += size;
		mdelay(1);
		size = asrc_get_output_FIFO_size(params->index);
	}

	if (t_size > ASRC_OUTPUT_LAST_SAMPLE)
		t_size = ASRC_OUTPUT_LAST_SAMPLE;
	params->output_last_period.length = t_size * params->channel_nums * 2;
}

static void asrc_read_output_FIFO_S24(struct asrc_pair_params *params)
{
	u32 size, i, j, reg, t_size;
	u32 *index = params->output_last_period.dma_vaddr;

	t_size = 0;
	udelay(100);
	size = asrc_get_output_FIFO_size(params->index);
	while (size) {
		for (i = 0; i < size; i++) {
			for (j = 0; j < params->channel_nums; j++) {
				reg = asrc_read_one_from_output_FIFO(
							params->index);
				*(index) = reg;
				index++;
			}
		}
		t_size += size;
		mdelay(1);
		size = asrc_get_output_FIFO_size(params->index);
	}

	if (t_size > ASRC_OUTPUT_LAST_SAMPLE)
		t_size = ASRC_OUTPUT_LAST_SAMPLE;
	params->output_last_period.length = t_size * params->channel_nums * 4;
}


static void asrc_output_task_worker(struct work_struct *w)
{
	struct asrc_pair_params *params =
		container_of(w, struct asrc_pair_params, task_output_work);
	unsigned long lock_flags;

	/* asrc output work struct */
	spin_lock_irqsave(&pair_lock, lock_flags);
	if (!params->pair_hold) {
		spin_unlock_irqrestore(&pair_lock, lock_flags);
		return;
	}
	switch (params->output_word_width) {
	case ASRC_WIDTH_24_BIT:
		asrc_read_output_FIFO_S24(params);
		break;
	case ASRC_WIDTH_16_BIT:
	case ASRC_WIDTH_8_BIT:
		asrc_read_output_FIFO_S16(params);
		break;
	default:
		pr_err("%s: error word width\n", __func__);
	}
	spin_unlock_irqrestore(&pair_lock, lock_flags);

	/* finish receiving all output data */
	spin_lock_irqsave(&output_int_lock, lock_flags);
	params->output_counter++;
	wake_up_interruptible(&params->output_wait_queue);
	spin_unlock_irqrestore(&output_int_lock, lock_flags);
}

static void mxc_free_dma_buf(struct asrc_pair_params *params)
{
	if (params->input_dma_total.dma_vaddr != NULL) {
		kfree(params->input_dma_total.dma_vaddr);
		params->input_dma_total.dma_vaddr = NULL;
	}

	if (params->output_dma_total.dma_vaddr != NULL) {
		kfree(params->output_dma_total.dma_vaddr);
		params->output_dma_total.dma_vaddr = NULL;
	}

	if (params->output_last_period.dma_vaddr) {
		dma_free_coherent(
			g_asrc->dev, 4096,
			params->output_last_period.dma_vaddr,
			params->output_last_period.dma_paddr);
		params->output_last_period.dma_vaddr = NULL;
	}

	return;
}

static int mxc_allocate_dma_buf(struct asrc_pair_params *params)
{
	struct dma_block *input_a, *output_a, *last_period;
	input_a = &params->input_dma_total;
	output_a = &params->output_dma_total;
	last_period = &params->output_last_period;

	input_a->dma_vaddr = kzalloc(input_a->length, GFP_KERNEL);
	if (!input_a->dma_vaddr) {
		pr_err("fail to allocate input dma buffer!\n");
		goto exit;
	}
	input_a->dma_paddr = virt_to_dma(NULL, input_a->dma_vaddr);

	output_a->dma_vaddr = kzalloc(output_a->length, GFP_KERNEL);
	if (!output_a->dma_vaddr) {
		pr_err("fail to allocate output dma buffer!\n");
		goto exit;
	}
	output_a->dma_paddr = virt_to_dma(NULL, output_a->dma_vaddr);

	last_period->dma_vaddr =
		dma_alloc_coherent(NULL,
			4096,
			&last_period->dma_paddr,
			GFP_KERNEL);

	return 0;

exit:
	mxc_free_dma_buf(params);
	pr_err("can't allocate buffer\n");
	return -ENOBUFS;
}

static bool filter(struct dma_chan *chan, void *param)
{

	if (!imx_dma_is_general_purpose(chan))
		return false;

	chan->private = param;
	return true;
}

static struct dma_chan *imx_asrc_dma_alloc(u32 dma_req)
{
	dma_cap_mask_t mask;
	struct imx_dma_data dma_data = {0};

	dma_data.peripheral_type = IMX_DMATYPE_ASRC;
	dma_data.priority = DMA_PRIO_MEDIUM;
	dma_data.dma_request = dma_req;

	/* Try to grab a DMA channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	return dma_request_channel(mask, filter, &dma_data);
}

static int imx_asrc_dma_config(
				struct asrc_pair_params *params,
				struct dma_chan *chan,
				u32 dma_addr, void *buf_addr,
				u32 buf_len, bool in,
				enum asrc_word_width word_width)
{
	struct dma_slave_config slave_config;
	enum dma_slave_buswidth buswidth;
	struct scatterlist *sg;
	unsigned int sg_nent, sg_index;
	struct dma_async_tx_descriptor *desc;
	int ret;

	if (in) {
		sg = params->input_sg;
		sg_nent = params->input_sg_nodes;
		desc = params->desc_in;
	} else {
		sg = params->output_sg;
		sg_nent = params->output_sg_nodes;
		desc = params->desc_out;
	}

	switch (word_width) {
	case ASRC_WIDTH_16_BIT:
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case ASRC_WIDTH_24_BIT:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		pr_err("Error word_width\n");
		return -EINVAL;
	}

	if (in) {
		slave_config.direction = DMA_MEM_TO_DEV;
		slave_config.dst_addr = dma_addr;
		slave_config.dst_addr_width = buswidth;
		slave_config.dst_maxburst =
			params->input_wm * params->channel_nums;
	} else {
		slave_config.direction = DMA_DEV_TO_MEM;
		slave_config.src_addr = dma_addr;
		slave_config.src_addr_width = buswidth;
		slave_config.src_maxburst =
			params->output_wm * params->channel_nums;
	}
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		pr_err("imx_asrc_dma_config(%d) failed\r\n", in);
		return -EINVAL;
	}

	sg_init_table(sg, sg_nent);
	switch (sg_nent) {
	case 1:
		sg_init_one(sg, buf_addr, buf_len);
		break;
	case 2:
	case 3:
	case 4:
		for (sg_index = 0; sg_index < (sg_nent - 1); sg_index++) {
			sg_set_buf(&sg[sg_index],
				buf_addr + sg_index * ASRC_MAX_BUFFER_SIZE,
				ASRC_MAX_BUFFER_SIZE);
		}
		sg_set_buf(&sg[sg_index],
			buf_addr + sg_index * ASRC_MAX_BUFFER_SIZE,
			buf_len - ASRC_MAX_BUFFER_SIZE * sg_index);
		break;
	default:
		pr_err("Error Input DMA nodes number[%d]!", sg_nent);
		return -EINVAL;
	}

	ret = dma_map_sg(NULL, sg, sg_nent, slave_config.direction);
	if (ret != sg_nent) {
		pr_err("DMA mapping error!!\n");
		return -EINVAL;
	}

	desc = chan->device->device_prep_slave_sg(chan,
					sg, sg_nent, slave_config.direction, 1);

	if (in) {
		params->desc_in = desc;
		params->desc_in->callback = asrc_input_dma_callback;
	} else {
		params->desc_out = desc;
		params->desc_out->callback = asrc_output_dma_callback;
	}
	if (desc) {
		desc->callback = in ?
			asrc_input_dma_callback : asrc_output_dma_callback;
		desc->callback_param = params;
	} else {
		return -EINVAL;
	}

	return 0;

}

static int mxc_asrc_prepare_input_buffer(struct asrc_pair_params *params,
					struct asrc_convert_buffer *pbuf)
{
	u32 word_size;

	switch (params->input_word_width) {
	case ASRC_WIDTH_24_BIT:
		word_size = 4;
		break;
	case ASRC_WIDTH_16_BIT:
	case ASRC_WIDTH_8_BIT:
		word_size = 2;
		break;
	default:
		pr_err("error input word size!\n");
		return -EINVAL;
	}

	if (pbuf->input_buffer_length <
		word_size * params->channel_nums * params->input_wm) {
		pr_err("input buffer size[%d] is too small!\n",
					pbuf->input_buffer_length);
		return -EINVAL;
	}

	/* copy origin data into input buffer */
	if (copy_from_user(
		params->input_dma_total.dma_vaddr,
		(void __user *)pbuf->input_buffer_vaddr,
		pbuf->input_buffer_length)) {
		return -EFAULT;
	}

	params->input_dma_total.length = pbuf->input_buffer_length;
	params->input_sg_nodes =
		params->input_dma_total.length / ASRC_MAX_BUFFER_SIZE + 1;

	return imx_asrc_dma_config(
			params,
			params->input_dma_channel,
			asrc_get_per_addr(params->index, 1),
			params->input_dma_total.dma_vaddr,
			params->input_dma_total.length, 1,
			params->input_word_width);

}

static int mxc_asrc_prepare_output_buffer(struct asrc_pair_params *params,
					struct asrc_convert_buffer *pbuf)
{
	u32 word_size;

	switch (params->output_word_width) {
	case ASRC_WIDTH_24_BIT:
		word_size = 4;
		break;
	case ASRC_WIDTH_16_BIT:
	case ASRC_WIDTH_8_BIT:
		word_size = 2;
		break;
	default:
		pr_err("error word size!\n");
		return -EINVAL;
	}

	if (pbuf->output_buffer_length <
		ASRC_OUTPUT_LAST_SAMPLE * word_size * params->channel_nums) {
		pr_err("output buffer size[%d] is wrong.\n",
				pbuf->output_buffer_length);
		return -EINVAL;
	}

	params->output_dma_total.length =
		pbuf->output_buffer_length -
		ASRC_OUTPUT_LAST_SAMPLE * word_size * params->channel_nums ;

	params->output_sg_nodes =
		params->output_dma_total.length / ASRC_MAX_BUFFER_SIZE + 1;

	return imx_asrc_dma_config(
			params,
			params->output_dma_channel,
			asrc_get_per_addr(params->index, 0),
			params->output_dma_total.dma_vaddr,
			params->output_dma_total.length, 0,
			params->output_word_width);
}

int mxc_asrc_process_output_buffer(struct asrc_pair_params *params,
					struct asrc_convert_buffer *pbuf)
{
	unsigned long lock_flags;

	if (!wait_event_interruptible_timeout
	    (params->output_wait_queue,
	     params->output_counter != 0, 10 * HZ)) {
		pr_info
		    ("ASRC_DQ_OUTBUF timeout counter %x\n",
		     params->output_counter);
		return -ETIME;
	} else if (signal_pending(current)) {
		pr_info("ASRC_DQ_INBUF interrupt received\n");
		return -ERESTARTSYS;
	}
	spin_lock_irqsave(&output_int_lock, lock_flags);
	params->output_counter--;
	spin_unlock_irqrestore(&output_int_lock, lock_flags);

	pbuf->output_buffer_length = params->output_dma_total.length;

	if (copy_to_user((void __user *)pbuf->output_buffer_vaddr,
				params->output_dma_total.dma_vaddr,
				params->output_dma_total.length))
		return -EFAULT;

	pbuf->output_buffer_length += params->output_last_period.length;

	if (copy_to_user((void __user *)(pbuf->output_buffer_vaddr +
				params->output_dma_total.length),
				params->output_last_period.dma_vaddr,
				params->output_last_period.length))
		return -EFAULT;

	return 0;

}

int mxc_asrc_process_input_buffer(struct asrc_pair_params *params,
					struct asrc_convert_buffer *pbuf)
{
	unsigned long lock_flags;

	if (!wait_event_interruptible_timeout
	    (params->input_wait_queue,
	     params->input_counter != 0, 10 * HZ)) {
		pr_info
		    ("ASRC_DQ_INBUF timeout counter %x\n",
		     params->input_counter);
		return -ETIME;
	} else if (signal_pending(current)) {
		pr_info("ASRC_DQ_INBUF interrupt received\n");
		return -ERESTARTSYS;
	}
	spin_lock_irqsave(&input_int_lock, lock_flags);
	params->input_counter--;
	spin_unlock_irqrestore(&input_int_lock, lock_flags);

	pbuf->input_buffer_length = params->input_dma_total.length;

	return 0;
}

static void mxc_asrc_submit_dma(struct asrc_pair_params *params)
{
	enum asrc_pair_index index;
	u32 size, i, j;
	index = params->index;

	/*  read all data in OUTPUT FIFO*/
	size = asrc_get_output_FIFO_size(params->index);
	while (size) {
		for (j = 0; j < size; j++) {
			for (i = 0; i < params->channel_nums; i++)
				asrc_read_one_from_output_FIFO(params->index);
		}
		mdelay(1);
		size = asrc_get_output_FIFO_size(params->index);
	}

	/* Fill the input FIFO until reach the stall level */
	size = asrc_get_input_FIFO_size(params->index);
	while (size < 3) {
		for (i = 0; i < params->channel_nums; i++)
			asrc_write_one_to_output_FIFO(params->index, 0);
		size = asrc_get_input_FIFO_size(params->index);
	}

	/* submit dma request */
	dmaengine_submit(params->desc_in);
	dmaengine_submit(params->desc_out);
	sdma_set_event_pending(params->input_dma_channel);

}

/*!
 * asrc interface -  function
 *
 * @param inode      struct inode *
 *
 * @param file       struct file *
 *
 * @param cmd    unsigned int
 *
 * @param arg        unsigned long
 *
 * @return           0 success, ENODEV for invalid device instance,
 *                   -1 for other errors.
 */
static long asrc_ioctl(struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct asrc_pair_params *params;
	params = file->private_data;

	switch (cmd) {
	case ASRC_REQ_PAIR:
		{
			struct asrc_req req;
			if (copy_from_user(&req, (void __user *)arg,
					   sizeof(struct asrc_req))) {
				err = -EFAULT;
				break;
			}
			err = asrc_req_pair(req.chn_num, &req.index);
			if (err < 0)
				break;
			params->pair_hold = 1;
			params->index = req.index;
			params->channel_nums = req.chn_num;
			if (copy_to_user
			    ((void __user *)arg, &req, sizeof(struct asrc_req)))
				err = -EFAULT;

			break;
		}
	case ASRC_CONFIG_PAIR:
		{
			struct asrc_config config;
			u32 rx_id, tx_id;
			char *rx_name, *tx_name;
			if (copy_from_user
			    (&config, (void __user *)arg,
			     sizeof(struct asrc_config))) {
				err = -EFAULT;
				break;
			}
			err = asrc_config_pair(&config);
			if (err < 0)
				break;

			params->input_wm = 4;
			params->output_wm = 2;
			err = asrc_set_watermark(config.pair,
					params->input_wm, params->output_wm);
			if (err < 0)
				break;
			params->output_buffer_size = config.dma_buffer_size;
			params->input_buffer_size = config.dma_buffer_size;
			if (config.buffer_num > ASRC_DMA_BUFFER_NUM)
				params->buffer_num = ASRC_DMA_BUFFER_NUM;
			else
				params->buffer_num = config.buffer_num;

			params->input_dma_total.length = ASRC_DMA_BUFFER_SIZE;
			params->output_dma_total.length = ASRC_DMA_BUFFER_SIZE;

			params->input_word_width = config.input_word_width;
			params->output_word_width = config.output_word_width;

			params->input_sample_rate = config.input_sample_rate;
			params->output_sample_rate = config.output_sample_rate;

			err = mxc_allocate_dma_buf(params);
			if (err < 0)
				break;

			/* TBD - need to update when new SDMA interface ready */
			if (config.pair == ASRC_PAIR_A) {
				rx_id = asrc_get_dma_request(ASRC_PAIR_A, 1);
				tx_id = asrc_get_dma_request(ASRC_PAIR_A, 0);
				rx_name = asrc_pair_id[0];
				tx_name = asrc_pair_id[1];
			} else if (config.pair == ASRC_PAIR_B) {
				rx_id = asrc_get_dma_request(ASRC_PAIR_B, 1);
				tx_id = asrc_get_dma_request(ASRC_PAIR_B, 0);
				rx_name = asrc_pair_id[2];
				tx_name = asrc_pair_id[3];
			} else {
				rx_id = asrc_get_dma_request(ASRC_PAIR_C, 1);
				tx_id = asrc_get_dma_request(ASRC_PAIR_C, 0);
				rx_name = asrc_pair_id[4];
				tx_name = asrc_pair_id[5];
			}

			params->input_dma_channel = imx_asrc_dma_alloc(rx_id);
			if (params->input_dma_channel == NULL) {
				pr_err("unable to get rx channel %d\n", rx_id);
				err = -EBUSY;
			}

			params->output_dma_channel = imx_asrc_dma_alloc(tx_id);
			if (params->output_dma_channel == NULL) {
				pr_err("unable to get tx channel %d\n", tx_id);
				err = -EBUSY;
			}

			init_waitqueue_head(&params->input_wait_queue);
			init_waitqueue_head(&params->output_wait_queue);
			/* Add work struct to cover the task of
			 * receive last period of output data.*/
			INIT_WORK(&params->task_output_work,
						asrc_output_task_worker);

			if (copy_to_user
			    ((void __user *)arg, &config,
			     sizeof(struct asrc_config)))
				err = -EFAULT;
			break;
		}
	case ASRC_RELEASE_PAIR:
		{
			enum asrc_pair_index index;
			unsigned long lock_flags;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}

			if (index < 0) {
				pr_err("unvalid index: %d!\n", index);
				err = -EFAULT;
				break;
			}

			params->asrc_active = 0;

			spin_lock_irqsave(&pair_lock, lock_flags);
			params->pair_hold = 0;
			spin_unlock_irqrestore(&pair_lock, lock_flags);
			if (params->input_dma_channel)
				dma_release_channel(params->input_dma_channel);
			if (params->output_dma_channel)
				dma_release_channel(params->output_dma_channel);

			mxc_free_dma_buf(params);
			asrc_release_pair(index);
			asrc_finish_conv(index);
			break;
		}
	case ASRC_CONVERT:
		{
			struct asrc_convert_buffer buf;
			if (copy_from_user
			    (&buf, (void __user *)arg,
			     sizeof(struct asrc_convert_buffer))) {
				err = -EFAULT;
				break;
			}

			err = mxc_asrc_prepare_input_buffer(params, &buf);
			if (err)
				break;

			err = mxc_asrc_prepare_output_buffer(params, &buf);
			if (err)
				break;

			mxc_asrc_submit_dma(params);

			err = mxc_asrc_process_output_buffer(params, &buf);
			if (err)
				break;

			err = mxc_asrc_process_input_buffer(params, &buf);
			if (err)
				break;

			if (copy_to_user
			    ((void __user *)arg, &buf,
			     sizeof(struct asrc_convert_buffer)))
				err = -EFAULT;

			break;
		}
	case ASRC_START_CONV:{
			enum asrc_pair_index index;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}

			params->asrc_active = 1;
			asrc_start_conv(index);

			break;
		}
	case ASRC_STOP_CONV:{
			enum asrc_pair_index index;
			if (copy_from_user
			    (&index, (void __user *)arg,
			     sizeof(enum asrc_pair_index))) {
				err = -EFAULT;
				break;
			}
			dmaengine_terminate_all(params->input_dma_channel);
			dmaengine_terminate_all(params->output_dma_channel);
			asrc_stop_conv(index);
			params->asrc_active = 0;
			break;
		}
	case ASRC_STATUS:{
			struct asrc_status_flags flags;
			if (copy_from_user
			    (&flags, (void __user *)arg,
			     sizeof(struct asrc_status_flags))) {
				err = -EFAULT;
				break;
			}
			asrc_get_status(&flags);
			if (copy_to_user
			    ((void __user *)arg, &flags,
			     sizeof(struct asrc_status_flags)))
				err = -EFAULT;
			break;
		}
	case ASRC_FLUSH:{
			/* flush input dma buffer */
			unsigned long lock_flags;
			u32 rx_id, tx_id;
			char *rx_name, *tx_name;
			spin_lock_irqsave(&input_int_lock, lock_flags);

			params->input_counter = 0;
			spin_unlock_irqrestore(&input_int_lock, lock_flags);

			/* flush output dma buffer */
			spin_lock_irqsave(&output_int_lock, lock_flags);

			params->output_counter = 0;
			spin_unlock_irqrestore(&output_int_lock, lock_flags);

			/* release DMA and request again */
			dma_release_channel(params->input_dma_channel);
			dma_release_channel(params->output_dma_channel);
			if (params->index == ASRC_PAIR_A) {
				rx_id = g_asrc->dmarx[ASRC_PAIR_A];
				tx_id = g_asrc->dmatx[ASRC_PAIR_A];
				rx_name = asrc_pair_id[0];
				tx_name = asrc_pair_id[1];
			} else if (params->index == ASRC_PAIR_B) {
				rx_id = g_asrc->dmarx[ASRC_PAIR_B];
				tx_id = g_asrc->dmatx[ASRC_PAIR_B];
				rx_name = asrc_pair_id[2];
				tx_name = asrc_pair_id[3];
			} else {
				rx_id = g_asrc->dmarx[ASRC_PAIR_C];
				tx_id = g_asrc->dmatx[ASRC_PAIR_C];
				rx_name = asrc_pair_id[4];
				tx_name = asrc_pair_id[5];
			}

			params->input_dma_channel = imx_asrc_dma_alloc(rx_id);
			if (params->input_dma_channel == NULL) {
				pr_err("unable to get rx channel %d\n", rx_id);
				err = -EBUSY;
			}

			params->output_dma_channel = imx_asrc_dma_alloc(tx_id);
			if (params->output_dma_channel == NULL) {
				pr_err("unable to get tx channel %d\n", tx_id);
				err = -EBUSY;
			}

			break;
		}
	default:
		break;
	}

	return err;
}

/*!
 * asrc interface - open function
 *
 * @param inode        structure inode *
 *
 * @param file         structure file *
 *
 * @return  status    0 success, ENODEV invalid device instance,
 *      ENOBUFS failed to allocate buffer, ERESTARTSYS interrupted by user
 */
static int mxc_asrc_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct asrc_pair_params *pair_params;

	if (signal_pending(current))
		return -EINTR;
	pair_params = kzalloc(sizeof(struct asrc_pair_params), GFP_KERNEL);
	if (pair_params == NULL) {
		pr_debug("Failed to allocate pair_params\n");
		err = -ENOBUFS;
	}
	file->private_data = pair_params;
	return err;
}

/*!
 * asrc interface - close function
 *
 * @param inode    struct inode *
 * @param file        structure file *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int mxc_asrc_close(struct inode *inode, struct file *file)
{
	struct asrc_pair_params *pair_params;
	unsigned long lock_flags;
	pair_params = file->private_data;
	if (pair_params) {
		if (pair_params->asrc_active) {
			pair_params->asrc_active = 0;
			dmaengine_terminate_all(
					pair_params->input_dma_channel);
			dmaengine_terminate_all(
					pair_params->output_dma_channel);
			asrc_stop_conv(pair_params->index);
			wake_up_interruptible(&pair_params->input_wait_queue);
			wake_up_interruptible(&pair_params->output_wait_queue);
		}
		if (pair_params->pair_hold) {
			spin_lock_irqsave(&pair_lock, lock_flags);
			pair_params->pair_hold = 0;
			spin_unlock_irqrestore(&pair_lock, lock_flags);
			if (pair_params->input_dma_channel)
				dma_release_channel(
					pair_params->input_dma_channel);
			if (pair_params->output_dma_channel)
				dma_release_channel(
					pair_params->output_dma_channel);
			mxc_free_dma_buf(pair_params);
			asrc_release_pair(pair_params->index);
			asrc_finish_conv(pair_params->index);
		}
		kfree(pair_params);
		file->private_data = NULL;
	}
	return 0;
}

/*!
 * asrc interface - mmap function
 *
 * @param file        structure file *
 *
 * @param vma         structure vm_area_struct *
 *
 * @return status     0 Success, EINTR busy lock error, ENOBUFS remap_page error
 */
static int mxc_asrc_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size;
	int res = 0;
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start,
			    vma->vm_pgoff, size, vma->vm_page_prot))
		return -ENOBUFS;

	vma->vm_flags &= ~VM_IO;
	return res;
}

static struct file_operations asrc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl	= asrc_ioctl,
	.mmap = mxc_asrc_mmap,
	.open = mxc_asrc_open,
	.release = mxc_asrc_close,
};

static int asrc_read_proc_attr(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
	unsigned long reg;
	int len = 0;
	clk_enable(g_asrc->mxc_asrc_data->asrc_core_clk);
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCNCR_REG);
	clk_disable(g_asrc->mxc_asrc_data->asrc_core_clk);

	len += sprintf(page, "ANCA: %d\n",
		       (int)(reg &
			     (0xFFFFFFFF >>
			      (32 - g_asrc->mxc_asrc_data->channel_bits))));
	len +=
	    sprintf(page + len, "ANCB: %d\n",
		    (int)((reg >> g_asrc->mxc_asrc_data->
			   channel_bits) & (0xFFFFFFFF >>
				(32 - g_asrc->mxc_asrc_data->channel_bits))));
	len +=
	    sprintf(page + len, "ANCC: %d\n",
		(int)((reg >> (g_asrc->mxc_asrc_data->channel_bits * 2)) &
		(0xFFFFFFFF >> (32 - g_asrc->mxc_asrc_data->channel_bits))));

	if (off > len)
		return 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return min(count, len - (int)off);
}

static int asrc_write_proc_attr(struct file *file, const char *buffer,
				unsigned long count, void *data)
{
	char buf[50];
	unsigned long reg;
	int na, nb, nc;
	int total;
	if (count > 48)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count)) {
		pr_debug("Attr proc write, Failed to copy buffer from user\n");
		return -EFAULT;
	}

	clk_enable(g_asrc->mxc_asrc_data->asrc_core_clk);
	reg = __raw_readl(g_asrc->vaddr + ASRC_ASRCNCR_REG);
	clk_disable(g_asrc->mxc_asrc_data->asrc_core_clk);
	sscanf(buf, "ANCA: %d\nANCB: %d\nANCC: %d", &na, &nb, &nc);
	if (g_asrc->mxc_asrc_data->channel_bits > 3)
		total = 10;
	else
		total = 5;

	if ((na + nb + nc) > total) {
		pr_err("Don't surpass %d for total.\n", total);
		return -EINVAL;
	} else if (na % 2 != 0 || nb % 2 != 0 || nc % 2 != 0) {
		pr_err("Please set an even number for each pair.\n");
		return -EINVAL;
	} else if (na < 0 || nb < 0 || nc < 0) {
		pr_err("Please set an positive number for each pair.\n");
		return -EINVAL;
	}

	reg = na | (nb << g_asrc->mxc_asrc_data->channel_bits) |
		(nc << (g_asrc->mxc_asrc_data->channel_bits * 2));

	/* Update chn_max */
	g_asrc->asrc_pair[ASRC_PAIR_A].chn_max = na;
	g_asrc->asrc_pair[ASRC_PAIR_B].chn_max = nb;
	g_asrc->asrc_pair[ASRC_PAIR_C].chn_max = nc;

	clk_enable(g_asrc->mxc_asrc_data->asrc_core_clk);
	__raw_writel(reg, g_asrc->vaddr + ASRC_ASRCNCR_REG);
	clk_disable(g_asrc->mxc_asrc_data->asrc_core_clk);

	return count;
}

static void asrc_proc_create(void)
{
	struct proc_dir_entry *proc_attr;
	g_asrc->proc_asrc = proc_mkdir(ASRC_PROC_PATH, NULL);
	if (g_asrc->proc_asrc) {
		proc_attr = create_proc_entry("ChSettings",
					      S_IFREG | S_IRUGO |
					      S_IWUSR, g_asrc->proc_asrc);
		if (proc_attr) {
			proc_attr->read_proc = asrc_read_proc_attr;
			proc_attr->write_proc = asrc_write_proc_attr;
			proc_attr->size = 48;
			proc_attr->uid = proc_attr->gid = 0;
		} else {
			remove_proc_entry(ASRC_PROC_PATH, NULL);
			pr_info("Failed to create proc attribute entry \n");
		}
	} else {
		pr_info("ASRC: Failed to create proc entry %s\n",
			ASRC_PROC_PATH);
	}
}

/*!
 * Entry point for the asrc device
 *
 * @param	pdev Pionter to the registered platform device
 * @return  Error code indicating success or failure
 */
static int mxc_asrc_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *res;
	struct device *temp_class;
	int irq;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	g_asrc = kzalloc(sizeof(struct asrc_data), GFP_KERNEL);

	if (g_asrc == NULL) {
		pr_info("Failed to allocate g_asrc\n");
		return -ENOMEM;
	}

	g_asrc->dev = &pdev->dev;
	g_asrc->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	g_asrc->asrc_pair[0].chn_max = 2;
	g_asrc->asrc_pair[1].chn_max = 6;
	g_asrc->asrc_pair[2].chn_max = 2;
	g_asrc->asrc_pair[0].overload_error = 0;
	g_asrc->asrc_pair[1].overload_error = 0;
	g_asrc->asrc_pair[2].overload_error = 0;

	g_asrc->asrc_major =
		register_chrdev(g_asrc->asrc_major, "mxc_asrc", &asrc_fops);
	if (g_asrc->asrc_major < 0) {
		pr_info("Unable to register asrc device\n");
		err = -EBUSY;
		goto error;
	}

	g_asrc->asrc_class = class_create(THIS_MODULE, "mxc_asrc");
	if (IS_ERR(g_asrc->asrc_class)) {
		err = PTR_ERR(g_asrc->asrc_class);
		goto err_out_chrdev;
	}

	temp_class =
		device_create(g_asrc->asrc_class, NULL,
			MKDEV(g_asrc->asrc_major, 0), NULL, "mxc_asrc");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	g_asrc->paddr = res->start;
	g_asrc->vaddr =
	    (unsigned long)ioremap(res->start, res->end - res->start + 1);
	g_asrc->mxc_asrc_data =
	    (struct imx_asrc_platform_data *)pdev->dev.platform_data;

	clk_enable(g_asrc->mxc_asrc_data->asrc_core_clk);

	switch (g_asrc->mxc_asrc_data->clk_map_ver) {
	case 1:
		input_clk_map = &input_clk_map_v1[0];
		output_clk_map = &output_clk_map_v1[0];
		break;
	case 2:
	default:
		input_clk_map = &input_clk_map_v2[0];
		output_clk_map = &output_clk_map_v2[0];
		break;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx1");
	if (res)
		g_asrc->dmatx[0] = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx1");
	if (res)
		g_asrc->dmarx[0] = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx2");
	if (res)
		g_asrc->dmatx[1] = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx2");
	if (res)
		g_asrc->dmarx[1] = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "tx3");
	if (res)
		g_asrc->dmatx[2] = res->start;

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx3");
	if (res)
		g_asrc->dmarx[2] = res->start;

	irq = platform_get_irq(pdev, 0);
	if (request_irq(irq, asrc_isr, 0, "asrc", NULL))
		return -1;

	asrc_proc_create();
	err = mxc_init_asrc();
	if (err < 0)
		goto err_out_class;

	goto out;

      err_out_class:
	device_destroy(g_asrc->asrc_class, MKDEV(g_asrc->asrc_major, 0));
	class_destroy(g_asrc->asrc_class);
      err_out_chrdev:
	unregister_chrdev(g_asrc->asrc_major, "mxc_asrc");
      error:
	kfree(g_asrc);
      out:
	clk_disable(g_asrc->mxc_asrc_data->asrc_core_clk);
	pr_info("mxc_asrc registered\n");
	return err;
}

/*!
 * Exit asrc device
 *
 * @param	pdev Pionter to the registered platform device
 * @return  Error code indicating success or failure
 */
static int mxc_asrc_remove(struct platform_device *pdev)
{
	int irq = platform_get_irq(pdev, 0);
	free_irq(irq, NULL);
	kfree(g_asrc);
	g_asrc->mxc_asrc_data = NULL;
	iounmap((unsigned long __iomem *)g_asrc->vaddr);
	remove_proc_entry("ChSettings", g_asrc->proc_asrc);
	remove_proc_entry(ASRC_PROC_PATH, NULL);
	device_destroy(g_asrc->asrc_class, MKDEV(g_asrc->asrc_major, 0));
	class_destroy(g_asrc->asrc_class);
	unregister_chrdev(g_asrc->asrc_major, "mxc_asrc");
	return 0;
}

/*! mxc asrc driver definition
 *
 */
static struct platform_driver mxc_asrc_driver = {
	.driver = {
		   .name = "mxc_asrc",
		   },
	.probe = mxc_asrc_probe,
	.remove = mxc_asrc_remove,
};

/*!
 * Register asrc driver
 *
 */
static __init int asrc_init(void)
{
	int ret;

	asrc_pcm_p2p_ops_asrc.asrc_p2p_start_conv = asrc_start_conv;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_stop_conv = asrc_stop_conv;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_get_dma_request = asrc_get_dma_request;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_per_addr = asrc_get_per_addr;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_req_pair = asrc_req_pair;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_config_pair = asrc_config_pair;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_release_pair = asrc_release_pair;
	asrc_pcm_p2p_ops_asrc.asrc_p2p_finish_conv = asrc_finish_conv;

	asrc_p2p_hook(&asrc_pcm_p2p_ops_asrc);

	ret = platform_driver_register(&mxc_asrc_driver);
	return ret;
}

/*!
 * Exit and free the asrc data
 *
 */ static void __exit asrc_exit(void)
{
	asrc_p2p_hook(NULL);

	platform_driver_unregister(&mxc_asrc_driver);
	return;
}

module_init(asrc_init);
module_exit(asrc_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Asynchronous Sample Rate Converter");
MODULE_LICENSE("GPL");
