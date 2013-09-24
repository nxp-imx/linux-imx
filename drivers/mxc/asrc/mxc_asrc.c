/*
 * Copyright 2008-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @file mxc_asrc.c
 *
 * @brief MXC Asynchronous Sample Rate Converter
 *
 * @ingroup Audio
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/regmap.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_devices.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/mxc_asrc.h>

#define ASRC_PROC_PATH "driver/asrc"

#define ASRC_RATIO_DECIMAL_DEPTH 26

DEFINE_SPINLOCK(data_lock);
DEFINE_SPINLOCK(pair_lock);
DEFINE_SPINLOCK(input_int_lock);
DEFINE_SPINLOCK(output_int_lock);

/* Sample rates are aligned with that defined in pcm.h file */
static const unsigned char asrc_process_table[][8][2] = {
	/* 32kHz 44.1kHz 48kHz   64kHz   88.2kHz 96kHz   176kHz  192kHz */
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},	/* 5512Hz */
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},	/* 8kHz */
	{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},	/* 11025Hz */
	{{0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},	/* 16kHz */
	{{0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},},	/* 22050Hz */
	{{0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0}, {0, 0},},	/* 32kHz */
	{{0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0},},	/* 44.1kHz */
	{{0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0}, {0, 0},},	/* 48kHz */
	{{1, 2}, {0, 2}, {0, 2}, {0, 1}, {0, 1}, {0, 1}, {0, 1}, {0, 0},},	/* 64kHz */
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},	/* 88.2kHz */
	{{1, 2}, {1, 2}, {1, 2}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1},},	/* 96kHz */
	{{2, 2}, {2, 2}, {2, 2}, {2, 1}, {2, 1}, {2, 1}, {2, 1}, {2, 1},},	/* 176kHz */
	{{2, 2}, {2, 2}, {2, 2}, {2, 1}, {2, 1}, {2, 1}, {2, 1}, {2, 1},},	/* 192kHz */
};

static struct asrc_data *asrc;

/*
 * The following tables map the relationship between asrc_inclk/asrc_outclk in
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

/* ALL registers of ASRC are 24-bit efficient */
static u32 asrc_regmap_read(struct regmap *map, unsigned int reg,
			      unsigned int *val)
{
#ifndef ASRC_USE_REGMAP
	*val = readl((void __iomem *)asrc->vaddr + reg) & 0xffffff;
	return *val;
#else
	return regmap_read(map, reg, val);
#endif
}

static void asrc_regmap_write(struct regmap *map, unsigned int reg,
			       unsigned int val)
{
#ifndef ASRC_USE_REGMAP
	writel(val & 0xffffff, (void __iomem *)asrc->vaddr + reg);
#else
	return regmap_write(map, reg, val);
#endif
}

static void asrc_regmap_update_bits(struct regmap *map, unsigned int reg,
				     unsigned int mask, unsigned int val)
{
#ifndef ASRC_USE_REGMAP
	u32 regval;

	regval = readl((void __iomem *)asrc->vaddr + reg) & 0xffffff;
	regval = (regval & ~mask) | (val & mask);
	writel(regval & 0xffffff, (void __iomem *)asrc->vaddr + reg);
#else
	regmap_update_bits(map, reg, mask, val);
#endif
}

/* Set ASRC_REG_ASRCNCR reg, only supporting one-pair setting at once */
static int asrc_set_channel_number(enum asrc_pair_index index, u32 val)
{
	u32 num;

	asrc_regmap_read(asrc->regmap, REG_ASRCNCR, &num);

	switch (index) {
	case ASRC_PAIR_A:
		num &= ~ASRCNCR_ANCA_MASK(asrc->channel_bits);
		num |= val;
		break;
	case ASRC_PAIR_B:
		num &= ~ASRCNCR_ANCB_MASK(asrc->channel_bits);
		num |= val << asrc->channel_bits;
		break;
	case ASRC_PAIR_C:
		num &= ~ASRCNCR_ANCC_MASK(asrc->channel_bits);
		num |= val << asrc->channel_bits * 2;
		break;
	default:
		dev_err(asrc->dev, "ASRC pair number not exists.\n");
		return -EINVAL;
	}

	asrc_regmap_write(asrc->regmap, REG_ASRCNCR, num);

	return 0;
}

#ifdef DEBUG
u32 asrc_reg[] = {
	REG_ASRCTR,
	REG_ASRIER,
	REG_ASRCNCR,
	REG_ASRCFG,
	REG_ASRCSR,
	REG_ASRCDR1,
	REG_ASRCDR2,
	REG_ASRSTR,
	REG_ASRRA,
	REG_ASRRB,
	REG_ASRRC,
	REG_ASRPM1,
	REG_ASRPM2,
	REG_ASRPM3,
	REG_ASRPM4,
	REG_ASRPM5,
	REG_ASRTFR1,
	REG_ASRCCR,
	REG_ASRIDRHA,
	REG_ASRIDRLA,
	REG_ASRIDRHB,
	REG_ASRIDRLB,
	REG_ASRIDRHC,
	REG_ASRIDRLC,
	REG_ASR76K,
	REG_ASR56K,
	REG_ASRMCRA,
	REG_ASRFSTA,
	REG_ASRMCRB,
	REG_ASRFSTB,
	REG_ASRMCRC,
	REG_ASRFSTC,
	REG_ASRMCR1A,
	REG_ASRMCR1B,
	REG_ASRMCR1C,
};

static void dump_regs(void)
{
	u32 reg, val;
	int i;

	for (i = 0; i < ARRAY_SIZE(asrc_reg); i++) {
		reg = asrc_reg[i];
		asrc_regmap_read(asrc->regmap, reg, &val);
		pr_debug("REG addr=0x%x val=0x%x\n", reg, val);
	}
}
#else
static void dump_regs(void) {}
#endif

/* Only used for Ideal Ratio mode */
static int asrc_set_clock_ratio(enum asrc_pair_index index,
		int inrate, int outrate)
{
	unsigned long val = 0;
	int integ;
	int i;

	if (outrate == 0) {
		dev_err(asrc->dev, "Wrong output sample rate: %d\n", outrate);
		return -EINVAL;
	}

	/* Formula: r = (1 << ASRC_RATIO_DECIMAL_DEPTH) / outrate * inrate; */
	for (integ = 0; inrate >= outrate; integ++)
		inrate -= outrate;

	val |= (integ << ASRC_RATIO_DECIMAL_DEPTH);

	for (i = 1; i <= ASRC_RATIO_DECIMAL_DEPTH; i++) {
		if ((inrate * 2) >= outrate) {
			val |= (1 << (ASRC_RATIO_DECIMAL_DEPTH - i));
			inrate = inrate * 2 - outrate;
		} else
			inrate = inrate << 1;

		if (inrate == 0)
			break;
	}

	asrc_regmap_write(asrc->regmap, REG_ASRIDRL(index), val);
	asrc_regmap_write(asrc->regmap, REG_ASRIDRH(index), (val >> 24));

	return 0;
}

/* Corresponding to asrc_process_table */
static int supported_input_rate[] = {
	5512, 8000, 11025, 16000, 22050, 32000, 44100, 48000, 64000, 88200,
	96000, 176400, 192000,
};

static int supported_output_rate[] = {
	32000, 44100, 48000, 64000, 88200, 96000, 176400, 192000,
};

static int asrc_set_process_configuration(enum asrc_pair_index index,
		int inrate, int outrate)
{
	int in, out;

	for (in = 0; in < ARRAY_SIZE(supported_input_rate); in++) {
		if (inrate == supported_input_rate[in])
			break;
	}

	if (in == ARRAY_SIZE(supported_input_rate)) {
		dev_err(asrc->dev, "Unsupported input sample rate: %d\n", in);
		return -EINVAL;
	}

	for (out = 0; out < ARRAY_SIZE(supported_output_rate); out++) {
		if (outrate == supported_output_rate[out])
			break;
	}

	if (out == ARRAY_SIZE(supported_output_rate)) {
		dev_err(asrc->dev, "Unsupported output sample rate: %d\n", out);
		return -EINVAL;
	}

	asrc_regmap_update_bits(asrc->regmap, REG_ASRCFG,
			ASRCFG_PREMODx_MASK(index) | ASRCFG_POSTMODx_MASK(index),
			ASRCFG_PREMOD(index, asrc_process_table[in][out][0]) |
			ASRCFG_POSTMOD(index, asrc_process_table[in][out][1]));

	return 0;
}

static int asrc_get_asrck_clock_divider(int samplerate)
{
	unsigned int prescaler, divider;
	unsigned int ratio, ra;
	unsigned long bitclk;
	unsigned int i;

	if (samplerate == 0) {
		dev_err(asrc->dev, "Wrong sample rate: %d\n", samplerate);
		return -EINVAL;
	}

	bitclk = clk_get_rate(asrc->asrc_clk);

	ra = bitclk/samplerate;
	ratio = ra;

	/* Calculate the prescaler */
	for (i = 0; ratio > 8; i++)
		ratio >>= 1;

	prescaler = i;

	/* Calculate the divider */
	if (i)
		divider = ((ra + (1 << (i - 1)) - 1) >> i) - 1;
	else
		divider = ra - 1;

	/* The totally divider is (2 ^ prescaler) * divider */
	return (divider << ASRCDRx_AxCPx_WIDTH) + prescaler;
}

int asrc_req_pair(int chn_num, enum asrc_pair_index *index)
{
	struct asrc_pair *pair;
	unsigned long lock_flags;
	int imax = 0, busy = 0, i, ret = 0;

	spin_lock_irqsave(&data_lock, lock_flags);

	for (i = ASRC_PAIR_A; i < ASRC_PAIR_MAX_NUM; i++) {
		pair = &asrc->asrc_pair[i];
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

	if (imax == ASRC_PAIR_MAX_NUM) {
		dev_err(asrc->dev, "No pair could afford requested channel number.\n");
		ret = -EINVAL;
	} else if (busy == ASRC_PAIR_MAX_NUM) {
		dev_err(asrc->dev, "All pairs are busy now.\n");
		ret = -EBUSY;
	} else if (busy + imax >= ASRC_PAIR_MAX_NUM) {
		pr_err("All affordable pairs are busy now.\n");
		ret = -EBUSY;
	} else {
		pair = &asrc->asrc_pair[*index];
		pair->chn_num = chn_num;
		pair->active = 1;
	}

	spin_unlock_irqrestore(&data_lock, lock_flags);

	if (!ret) {
		clk_enable(asrc->asrc_clk);
		clk_enable(asrc->dma_clk);
	}

	return ret;
}
EXPORT_SYMBOL(asrc_req_pair);

void asrc_release_pair(enum asrc_pair_index index)
{
	struct asrc_pair *pair = &asrc->asrc_pair[index];
	unsigned long lock_flags;

	spin_lock_irqsave(&data_lock, lock_flags);

	pair->active = 0;
	pair->overload_error = 0;

	/* Disable PAIR */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
			ASRCTR_ASRCEx_MASK(index), 0);

	spin_unlock_irqrestore(&data_lock, lock_flags);
}
EXPORT_SYMBOL(asrc_release_pair);

int asrc_config_pair(struct asrc_config *config)
{
	u32 inrate = config->input_sample_rate, indiv;
	u32 outrate = config->output_sample_rate, outdiv;
	unsigned long lock_flags;
	int index = config->pair;
	int channel_num;
	int ret;

	/* Set the channel number */
	spin_lock_irqsave(&data_lock, lock_flags);
	asrc->asrc_pair[index].chn_num = config->channel_num;
	spin_unlock_irqrestore(&data_lock, lock_flags);

	if (asrc->channel_bits > 3)
		channel_num = config->channel_num;
	else
		channel_num = (config->channel_num + 1) / 2;

	asrc_set_channel_number(index, channel_num);

	/* Set the clock source */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRCSR,
			ASRCSR_AICSx_MASK(index) | ASRCSR_AOCSx_MASK(index),
			ASRCSR_AICS(index, input_clk_map[config->inclk]) |
			ASRCSR_AOCS(index, output_clk_map[config->outclk]));

	/* Default setting: Automatic selection for processing mode */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
			ASRCTR_ATSx_MASK(index), ASRCTR_ATS(index));
	asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
			ASRCTR_USRx_MASK(index), 0);

	/* Default Input Clock Divider Setting */
	switch (config->inclk & ASRCSR_AxCSx_MASK) {
	case INCLK_SPDIF_RX:
		indiv = ASRC_PRESCALER_SPDIF_RX;
		break;
	case INCLK_SPDIF_TX:
		indiv = ASRC_PRESCALER_SPDIF_TX;
		break;
	case INCLK_ASRCK1_CLK:
		indiv = asrc_get_asrck_clock_divider(inrate);
		break;
	default:
		switch (config->input_word_width) {
		case ASRC_WIDTH_16_BIT:
			indiv = ASRC_PRESCALER_I2S_16BIT;
			break;
		case ASRC_WIDTH_24_BIT:
			indiv = ASRC_PRESCALER_I2S_24BIT;
			break;
		default:
			dev_err(asrc->dev, "Unsupported input word width %d\n",
					config->input_word_width);
			return -EINVAL;
		}
		break;
	}

	/* Default Output Clock Divider Setting */
	switch (config->outclk & ASRCSR_AxCSx_MASK) {
	case OUTCLK_SPDIF_RX:
		outdiv = ASRC_PRESCALER_SPDIF_RX;
		break;
	case OUTCLK_SPDIF_TX:
		outdiv = ASRC_PRESCALER_SPDIF_TX;
		break;
	case OUTCLK_ASRCK1_CLK:
		if ((config->inclk & ASRCSR_AxCSx_MASK) == INCLK_NONE)
			outdiv = ASRC_PRESCALER_IDEAL_RATIO;
		else
			outdiv = asrc_get_asrck_clock_divider(outrate);
		break;
	default:
		switch (config->output_word_width) {
		case ASRC_WIDTH_16_BIT:
			outdiv = ASRC_PRESCALER_I2S_16BIT;
			break;
		case ASRC_WIDTH_24_BIT:
			outdiv = ASRC_PRESCALER_I2S_24BIT;
			break;
		default:
			dev_err(asrc->dev, "Unsupported output word width %d\n",
					config->input_word_width);
			return -EINVAL;
		}
		break;
	}

	/* indiv and outdiv'd include prescaler's value, so add its MASK too */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRCDR(index),
			ASRCDRx_AOCPx_MASK(index) | ASRCDRx_AICPx_MASK(index) |
			ASRCDRx_AOCDx_MASK(index) | ASRCDRx_AICDx_MASK(index),
			ASRCDRx_AOCP(index, outdiv) | ASRCDRx_AICP(index, indiv));

	/* Check whether ideal ratio is a must */
	switch (config->inclk & ASRCSR_AxCSx_MASK) {
	case INCLK_NONE:
		/* Clear ASTSx bit to use ideal ratio */
		asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
				ASRCTR_ATSx_MASK(index), 0);

		asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
				ASRCTR_IDRx_MASK(index) | ASRCTR_USRx_MASK(index),
				ASRCTR_IDR(index) | ASRCTR_USR(index));

		ret = asrc_set_clock_ratio(index, inrate, outrate);
		if (ret)
			return ret;

		ret = asrc_set_process_configuration(index, inrate, outrate);
		if (ret)
			return ret;

		break;
	case INCLK_ASRCK1_CLK:
		/* This case and default are both remained for v1 */
		if (inrate == 44100 || inrate == 88200) {
			dev_err(asrc->dev, "Unsupported sample rate %d by ASRC clock.\n",
					inrate);
			return -EINVAL;
		}
		break;
	default:
		if ((config->outclk & ASRCSR_AxCSx_MASK) != OUTCLK_ASRCK1_CLK)
			break;

		if (outrate == 44100 || outrate == 88200) {
			dev_err(asrc->dev, "Unsupported sample rate %d by ASRC clock.\n",
					outrate);
			return -EINVAL;
		}
		break;
	}

	/* Config input and output wordwidth */
	if (config->output_word_width == ASRC_WIDTH_8_BIT) {
		dev_err(asrc->dev, "Unsupported wordwidth for output: 8bit.\n");
		dev_err(asrc->dev, "Output only support: 16bit or 24bit.\n");
		return -EINVAL;
	}

	asrc_regmap_update_bits(asrc->regmap, REG_ASRMCR1(index),
			ASRMCR1x_OW16_MASK | ASRMCR1x_IWD_MASK,
			ASRMCR1x_OW16(config->output_word_width) |
			ASRMCR1x_IWD(config->input_word_width));

	/* Enable BUFFER STALL */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRMCR(index),
			ASRMCRx_BUFSTALLx_MASK, ASRMCRx_BUFSTALLx);

	/* Set Threshold for input and output FIFO */
	ret = asrc_set_watermark(index, ASRC_INPUTFIFO_THRESHOLD,
			ASRC_INPUTFIFO_THRESHOLD);

	return ret;
}
EXPORT_SYMBOL(asrc_config_pair);

#define ASRC_MAX_FIFO_THRESHOLD		63

int asrc_set_watermark(enum asrc_pair_index index, u32 in_wm, u32 out_wm)
{
	if (in_wm > ASRC_MAX_FIFO_THRESHOLD ||
			out_wm > ASRC_MAX_FIFO_THRESHOLD) {
		dev_err(asrc->dev, "Error watermark!\n");
		return -EINVAL;
	}

	asrc_regmap_update_bits(asrc->regmap, REG_ASRMCR(index),
			ASRMCRx_EXTTHRSHx_MASK | ASRMCRx_INFIFO_THRESHOLD_MASK |
			ASRMCRx_OUTFIFO_THRESHOLD_MASK,
			ASRMCRx_EXTTHRSHx | ASRMCRx_INFIFO_THRESHOLD(in_wm) |
			ASRMCRx_OUTFIFO_THRESHOLD(out_wm));

	return 0;
}
EXPORT_SYMBOL(asrc_set_watermark);

void asrc_start_conv(enum asrc_pair_index index)
{
	unsigned long lock_flags;
	int reg, retry;

	spin_lock_irqsave(&data_lock, lock_flags);

	asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
			ASRCTR_ASRCEx_MASK(index), ASRCTR_ASRCE(index));

	/* Wait for status of initialization */
	for (retry = 10, reg = 0; !reg && retry; --retry) {
		udelay(5);
		asrc_regmap_read(asrc->regmap, REG_ASRCFG, &reg);
		reg &= ASRCFG_INIRQx_MASK(index);
	}

	/* Overload Interrupt Enable */
	asrc_regmap_write(asrc->regmap, REG_ASRIER, ASRIER_AOLIE);

	spin_unlock_irqrestore(&data_lock, lock_flags);

	return;
}
EXPORT_SYMBOL(asrc_start_conv);

void asrc_stop_conv(enum asrc_pair_index index)
{
	unsigned long lock_flags;

	spin_lock_irqsave(&data_lock, lock_flags);

	asrc_regmap_update_bits(asrc->regmap, REG_ASRCTR,
			ASRCTR_ASRCEx_MASK(index), 0);

	spin_unlock_irqrestore(&data_lock, lock_flags);

	return;
}
EXPORT_SYMBOL(asrc_stop_conv);

void asrc_finish_conv(enum asrc_pair_index index)
{
	clk_disable(asrc->dma_clk);
	clk_disable(asrc->asrc_clk);
	return;
}
EXPORT_SYMBOL(asrc_finish_conv);

#define SET_OVERLOAD_ERR(index, err) \
	do {asrc->asrc_pair[index].overload_error |= err; } while (0)

static irqreturn_t asrc_isr(int irq, void *dev_id)
{
	enum asrc_pair_index index;
	u32 status;

	asrc_regmap_read(asrc->regmap, REG_ASRSTR, &status);

	for (index = ASRC_PAIR_A; index < ASRC_PAIR_MAX_NUM; index++) {
		if (asrc->asrc_pair[index].active == 0)
			continue;
		if (status & ASRSTR_ATQOL)
			SET_OVERLOAD_ERR(index, ASRC_TASK_Q_OVERLOAD);
		if (status & ASRSTR_AOOL(index))
			SET_OVERLOAD_ERR(index, ASRC_OUTPUT_TASK_OVERLOAD);
		if (status & ASRSTR_AIOL(index))
			SET_OVERLOAD_ERR(index, ASRC_INPUT_TASK_OVERLOAD);
		if (status & ASRSTR_AODO(index))
			SET_OVERLOAD_ERR(index, ASRC_OUTPUT_BUFFER_OVERFLOW);
		if (status & ASRSTR_AIDU(index))
			SET_OVERLOAD_ERR(index, ASRC_INPUT_BUFFER_UNDERRUN);
	}

	/* Clean overload error  */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRSTR,
			ASRSTR_AOLE_MASK, ASRSTR_AOLE);

	return IRQ_HANDLED;
}

void asrc_get_status(struct asrc_status_flags *flags)
{
	enum asrc_pair_index index = flags->index;
	unsigned long lock_flags;

	spin_lock_irqsave(&data_lock, lock_flags);

	flags->overload_error = asrc->asrc_pair[index].overload_error;

	spin_unlock_irqrestore(&data_lock, lock_flags);

	return;
}
EXPORT_SYMBOL(asrc_get_status);

u32 asrc_get_per_addr(enum asrc_pair_index index, bool in)
{
	u32 addr = in ? REG_ASRDI(index) : REG_ASRDO(index);

	return asrc->paddr + addr;
}
EXPORT_SYMBOL(asrc_get_per_addr);

static int mxc_init_asrc(void)
{
	clk_enable(asrc->asrc_clk);

	/* Halt ASRC internal FP when input FIFO needs data for pair A, B, C */
	asrc_regmap_write(asrc->regmap, REG_ASRCTR, ASRCTR_ASRCEN);

	/* Disable interrupt by default */
	asrc_regmap_write(asrc->regmap, REG_ASRIER, 0x0);

	/* Default 2: 6: 2 channel assignment */
	asrc_set_channel_number(ASRC_PAIR_A, 2);
	asrc_set_channel_number(ASRC_PAIR_B, 6);
	asrc_set_channel_number(ASRC_PAIR_C, 2);

	/* Parameter Registers recommended settings */
	asrc_regmap_write(asrc->regmap, REG_ASRPM1, 0x7fffff);
	asrc_regmap_write(asrc->regmap, REG_ASRPM2, 0x255555);
	asrc_regmap_write(asrc->regmap, REG_ASRPM3, 0xff7280);
	asrc_regmap_write(asrc->regmap, REG_ASRPM4, 0xff7280);
	asrc_regmap_write(asrc->regmap, REG_ASRPM5, 0xff7280);

	/* Base address for task queue FIFO. Set to 0x7C */
	asrc_regmap_update_bits(asrc->regmap, REG_ASRTFR1,
			ASRTFR1_TF_BASE_MASK, ASRTFR1_TF_BASE(0xfc));

	/* Set the processing clock for 76KHz, 133M */
	asrc_regmap_write(asrc->regmap, REG_ASR76K, 0x06D6);

	/* Set the processing clock for 56KHz, 133M */
	asrc_regmap_write(asrc->regmap, REG_ASR56K, 0x0947);

	clk_disable(asrc->asrc_clk);

	return 0;
}

#define ASRC_xPUT_DMA_CALLBACK(in) \
	((in) ? asrc_input_dma_callback : asrc_output_dma_callback)

static void asrc_input_dma_callback(void *data)
{
	struct asrc_pair_params *params = (struct asrc_pair_params *)data;
	unsigned long lock_flags;

	dma_unmap_sg(NULL, params->input_sg, params->input_sg_nodes,
			DMA_MEM_TO_DEV);

	spin_lock_irqsave(&input_int_lock, lock_flags);

	params->input_counter++;
	wake_up_interruptible(&params->input_wait_queue);

	spin_unlock_irqrestore(&input_int_lock, lock_flags);

	schedule_work(&params->task_output_work);

	return;
}

static void asrc_output_dma_callback(void *data)
{
	struct asrc_pair_params *params = (struct asrc_pair_params *)data;

	dma_unmap_sg(NULL, params->output_sg, params->output_sg_nodes,
			DMA_MEM_TO_DEV);
}

static unsigned int asrc_get_output_FIFO_size(enum asrc_pair_index index)
{
	u32 val;

	asrc_regmap_read(asrc->regmap, REG_ASRFST(index), &val);

	val &= ASRFSTx_OUTPUT_FIFO_MASK;

	return val >> ASRFSTx_OUTPUT_FIFO_SHIFT;
}

static unsigned int asrc_get_input_FIFO_size(enum asrc_pair_index index)
{
	u32 val;

	asrc_regmap_read(asrc->regmap, REG_ASRFST(index), &val);

	val &= ASRFSTx_INPUT_FIFO_MASK;

	return val >> ASRFSTx_INPUT_FIFO_SHIFT;
}

static u32 asrc_read_one_from_output_FIFO(enum asrc_pair_index index)
{
	u32 val;

	asrc_regmap_read(asrc->regmap, REG_ASRDO(index), &val);

	return val;
}

static void asrc_write_one_to_output_FIFO(enum asrc_pair_index index, u32 val)
{
	asrc_regmap_write(asrc->regmap, REG_ASRDI(index), val);
}

static void asrc_read_output_FIFO(struct asrc_pair_params *params)
{
	u32 *reg24 = params->output_last_period.dma_vaddr;
	u16 *reg16 = params->output_last_period.dma_vaddr;
	enum asrc_pair_index index = params->index;
	u32 d, i, j, reg, size, t_size;
	bool bit24 = false;

	if (params->output_word_width == ASRC_WIDTH_24_BIT)
		bit24 = true;

	/* Delay for last period data output */
	d = 1000000 / params->output_sample_rate * params->last_period_sample;

	t_size = 0;
	do {
		mdelay(1);
		size = asrc_get_output_FIFO_size(index);
		for (i = 0; i < size; i++) {
			for (j = 0; j < params->channel_nums; j++) {
				reg = asrc_read_one_from_output_FIFO(index);
				if (bit24) {
					*(reg24) = reg;
					reg24++;
				} else {
					*(reg16) = (u16)reg;
					reg16++;
				}
			}
		}
		t_size += size;
	} while (size);

	if (t_size > params->last_period_sample)
		t_size = params->last_period_sample;

	params->output_last_period.length = t_size * params->channel_nums * 2;
	if (bit24)
		params->output_last_period.length *= 2;
}

static void asrc_output_task_worker(struct work_struct *w)
{
	struct asrc_pair_params *params =
		container_of(w, struct asrc_pair_params, task_output_work);
	unsigned long lock_flags;

	if (!params->pair_hold)
		return;

	spin_lock_irqsave(&pair_lock, lock_flags);
	asrc_read_output_FIFO(params);
	spin_unlock_irqrestore(&pair_lock, lock_flags);

	/* Finish receiving all output data */
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
		dma_free_coherent(asrc->dev, 1024 * params->last_period_sample,
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
		dev_err(asrc->dev, "failed to allocate input dma buffer!\n");
		goto exit;
	}
	input_a->dma_paddr = virt_to_dma(NULL, input_a->dma_vaddr);

	output_a->dma_vaddr = kzalloc(output_a->length, GFP_KERNEL);
	if (!output_a->dma_vaddr) {
		dev_err(asrc->dev, "failed to allocate output dma buffer!\n");
		goto exit;
	}
	output_a->dma_paddr = virt_to_dma(NULL, output_a->dma_vaddr);

	last_period->dma_vaddr = dma_alloc_coherent(NULL,
			1024 * params->last_period_sample,
			&last_period->dma_paddr, GFP_KERNEL);

	return 0;

exit:
	mxc_free_dma_buf(params);
	dev_err(asrc->dev, "failed to allocate buffer.\n");

	return -ENOBUFS;
}

static struct dma_chan *imx_asrc_dma_request_channel(
		struct asrc_pair_params *params, bool in)
{
	char name[4];

	sprintf(name, "%cx%c", in ? 'r' : 't', params->index + 'a');

	return dma_request_slave_channel(asrc->dev, name);
}

static int imx_asrc_dma_config(struct asrc_pair_params *params,
		struct dma_chan *chan, u32 dma_addr, void *buf_addr,
		u32 buf_len, bool in, enum asrc_word_width word_width)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config slave_config;
	enum dma_slave_buswidth buswidth;
	struct scatterlist *sg;
	unsigned int sg_nent, i;
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
		dev_err(asrc->dev, "Error word_width.\n");
		return -EINVAL;
	}

	slave_config.dma_request0 = 0;
	slave_config.dma_request1 = 0;

	if (in) {
		slave_config.direction = DMA_MEM_TO_DEV;
		slave_config.dst_addr = dma_addr;
		slave_config.dst_addr_width = buswidth;
		slave_config.dst_maxburst =
			params->input_wm * params->channel_nums / buswidth;
	} else {
		slave_config.direction = DMA_DEV_TO_MEM;
		slave_config.src_addr = dma_addr;
		slave_config.src_addr_width = buswidth;
		slave_config.src_maxburst =
			params->output_wm * params->channel_nums / buswidth;
	}
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret) {
		dev_err(asrc->dev, "imx_asrc_dma_config(%d) failed: %d\r\n", in, ret);
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
		for (i = 0; i < (sg_nent - 1); i++)
			sg_set_buf(&sg[i], buf_addr + i * ASRC_MAX_BUFFER_SIZE,
					ASRC_MAX_BUFFER_SIZE);

		sg_set_buf(&sg[i], buf_addr + i * ASRC_MAX_BUFFER_SIZE,
				buf_len - ASRC_MAX_BUFFER_SIZE * i);
		break;
	default:
		dev_err(asrc->dev, "Error Input DMA nodes number[%d]!\n", sg_nent);
		return -EINVAL;
	}

	ret = dma_map_sg(NULL, sg, sg_nent, slave_config.direction);
	if (ret != sg_nent) {
		dev_err(asrc->dev, "DMA mapping error!\n");
		return -EINVAL;
	}

	desc = chan->device->device_prep_slave_sg(chan, sg, sg_nent,
			slave_config.direction, 1, NULL);

	if (in) {
		params->desc_in = desc;
		params->desc_in->callback = asrc_input_dma_callback;
	} else {
		params->desc_out = desc;
		params->desc_out->callback = asrc_output_dma_callback;
	}

	if (desc) {
		desc->callback = ASRC_xPUT_DMA_CALLBACK(in);
		desc->callback_param = params;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int mxc_asrc_prepare_io_buffer(struct asrc_pair_params *params,
		struct asrc_convert_buffer *pbuf, bool in)
{
	struct dma_chan *dma_channel;
	enum asrc_word_width width;
	unsigned int *dma_len, *sg_nodes, buf_len, wm;
	void __user *buf_vaddr;
	void *dma_vaddr;
	u32 word_size, fifo_addr;

	if (in) {
		dma_channel = params->input_dma_channel;
		dma_vaddr = params->input_dma_total.dma_vaddr;
		dma_len = &params->input_dma_total.length;
		width = params->input_word_width;
		sg_nodes = &params->input_sg_nodes;
		wm = params->input_wm;
		buf_vaddr = (void __user *)pbuf->input_buffer_vaddr;
		buf_len = pbuf->input_buffer_length;
	} else {
		dma_channel = params->output_dma_channel;
		dma_vaddr = params->output_dma_total.dma_vaddr;
		dma_len = &params->output_dma_total.length;
		width = params->output_word_width;
		sg_nodes = &params->output_sg_nodes;
		wm = params->last_period_sample;
		buf_vaddr = (void __user *)pbuf->output_buffer_vaddr;
		buf_len = pbuf->output_buffer_length;
	}

	switch (width) {
	case ASRC_WIDTH_24_BIT:
		word_size = 4;
		break;
	case ASRC_WIDTH_16_BIT:
	case ASRC_WIDTH_8_BIT:
		word_size = 2;
		break;
	default:
		dev_err(asrc->dev, "error %s word size!\n", in ? "input" : "output");
		return -EINVAL;
	}

	if (buf_len < word_size * params->channel_nums * wm) {
		dev_err(asrc->dev, "%s buffer size[%d] is too small!\n",
				in ? "input" : "output", buf_len);
		return -EINVAL;
	}

	/* Copy origin data into input buffer */
	if (in && copy_from_user(dma_vaddr, buf_vaddr, buf_len))
		return -EFAULT;

	*dma_len = buf_len;
	if (!in)
		*dma_len -= wm * word_size * params->channel_nums;

	*sg_nodes = *dma_len / ASRC_MAX_BUFFER_SIZE + 1;

	fifo_addr = asrc_get_per_addr(params->index, in);

	return imx_asrc_dma_config(params, dma_channel, fifo_addr, dma_vaddr,
			*dma_len, in, width);
}

static int mxc_asrc_prepare_buffer(struct asrc_pair_params *params,
		struct asrc_convert_buffer *pbuf)
{
	int ret;

	ret = mxc_asrc_prepare_io_buffer(params, pbuf, true);
	if (ret) {
		dev_err(asrc->dev, "failed to prepare input buffer: %d\n", ret);
		return ret;
	}

	ret = mxc_asrc_prepare_io_buffer(params, pbuf, false);
	if (ret) {
		dev_err(asrc->dev, "failed to prepare output buffer: %d\n", ret);
		return ret;
	}

	return 0;
}

int mxc_asrc_process_io_buffer(struct asrc_pair_params *params,
		struct asrc_convert_buffer *pbuf, bool in)
{
	void *last_vaddr = params->output_last_period.dma_vaddr;
	unsigned int *last_len = &params->output_last_period.length;
	unsigned int *counter, dma_len, *buf_len;
	unsigned long lock_flags;
	void __user *buf_vaddr;
	void *dma_vaddr;
	wait_queue_head_t *q;
	spinlock_t *lock;

	if (in) {
		dma_vaddr = params->input_dma_total.dma_vaddr;
		dma_len = params->input_dma_total.length;
		q = &params->input_wait_queue;
		counter = &params->input_counter;
		buf_len = &pbuf->input_buffer_length;
		lock = &input_int_lock;
		buf_vaddr = (void __user *)pbuf->input_buffer_vaddr;
	} else {
		dma_vaddr = params->output_dma_total.dma_vaddr;
		dma_len = params->output_dma_total.length;
		q = &params->output_wait_queue;
		counter = &params->output_counter;
		buf_len = &pbuf->output_buffer_length;
		lock = &output_int_lock;
		buf_vaddr = (void __user *)pbuf->output_buffer_vaddr;
	}

	if (!wait_event_interruptible_timeout(*q, *counter != 0, 10 * HZ)) {
		dev_err(asrc->dev, "ASRC_DQ_OUTBUF timeout counter %x\n", *counter);
		return -ETIME;
	} else if (signal_pending(current)) {
		dev_err(asrc->dev, "ASRC_DQ_INBUF interrupt received.\n");
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(lock, lock_flags);
	(*counter)--;
	spin_unlock_irqrestore(lock, lock_flags);

	*buf_len = dma_len;

	/* Only output need return data to user space */
	if (!in) {
		if (copy_to_user(buf_vaddr, dma_vaddr, dma_len))
			return -EFAULT;

		*buf_len += *last_len;

		if (copy_to_user(buf_vaddr + dma_len, last_vaddr, *last_len))
			return -EFAULT;
	}

	return 0;
}

int mxc_asrc_process_buffer(struct asrc_pair_params *params,
		struct asrc_convert_buffer *pbuf)
{
	int ret;

	ret = mxc_asrc_process_io_buffer(params, pbuf, true);
	if (ret) {
		dev_err(asrc->dev, "failed to process input buffer: %d\n", ret);
		return ret;
	}

	ret = mxc_asrc_process_io_buffer(params, pbuf, false);
	if (ret) {
		dev_err(asrc->dev, "failed to process output buffer: %d\n", ret);
		return ret;
	}

	return 0;
}

static void mxc_asrc_submit_dma(struct asrc_pair_params *params)
{
	enum asrc_pair_index index = params->index;
	u32 size = asrc_get_output_FIFO_size(params->index);
	int i, j;

	/* Read all data in OUTPUT FIFO */
	while (size) {
		for (j = 0; j < size; j++)
			for (i = 0; i < params->channel_nums; i++)
				asrc_read_one_from_output_FIFO(index);
		/*
		 * delay before size-getting should be:
		 * 1s / output_sample_rate * last_period_sample
		 * but we don't know last time's output_sample_rate and
		 * last_period_sample, and in order to cover max case
		 * so use min(sample_rate)=32kHz and max(last_period_sample)=32
		 * Thus 1s / 32k * 32 = 1ms
		 */
		mdelay(1);

		size = asrc_get_output_FIFO_size(index);
	}

	/* Fill the input FIFO until reach the stall level */
	size = asrc_get_input_FIFO_size(index);
	while (size < 3) {
		for (i = 0; i < params->channel_nums; i++)
			asrc_write_one_to_output_FIFO(index, 0);
		size = asrc_get_input_FIFO_size(index);
	}

	/* Submit dma request */
	dmaengine_submit(params->desc_in);
	dma_async_issue_pending(params->desc_in->chan);

	dmaengine_submit(params->desc_out);
	dma_async_issue_pending(params->desc_out->chan);

	sdma_set_event_pending(params->input_dma_channel);
}

static long asrc_ioctl_req_pair(struct asrc_pair_params *params,
		void __user *user)
{
	struct asrc_req req;
	long ret;

	ret = copy_from_user(&req, user, sizeof(req));
	if (ret) {
		dev_err(asrc->dev, "failed to get req from user space: %ld\n", ret);
		return ret;
	}

	ret = asrc_req_pair(req.chn_num, &req.index);
	if (ret) {
		dev_err(asrc->dev, "failed to request pair: %ld\n", ret);
		return ret;
	}

	params->pair_hold = 1;
	params->index = req.index;
	params->channel_nums = req.chn_num;

	ret = copy_to_user(user, &req, sizeof(req));
	if (ret) {
		dev_err(asrc->dev, "failed to send req to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long asrc_ioctl_config_pair(struct asrc_pair_params *params,
		void __user *user)
{
	struct asrc_config config;
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&config, user, sizeof(config));
	if (ret) {
		dev_err(asrc->dev, "failed to get config from user space: %ld\n", ret);
		return ret;
	}

	ret = asrc_config_pair(&config);
	if (ret) {
		dev_err(asrc->dev, "failed to config pair: %ld\n", ret);
		return ret;
	}

	index = config.pair;

	params->input_wm = 4;
	params->output_wm = 2;

	ret = asrc_set_watermark(index, params->input_wm, params->output_wm);
	if (ret)
		return ret;

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

	params->last_period_sample = ASRC_OUTPUT_LAST_SAMPLE_DEFAULT;

	/* Expand last period buffer if output_sample_rate is much bigger */
	if (params->output_sample_rate / params->input_sample_rate > 2)
		params->last_period_sample *= 5;
	else if (params->output_sample_rate / params->input_sample_rate > 1)
		params->last_period_sample *= 3;

	ret = mxc_allocate_dma_buf(params);
	if (ret) {
		dev_err(asrc->dev, "failed to allocate dma buffer: %ld\n", ret);
		return ret;
	}

	/* Request DMA channel for both input and output */
	params->input_dma_channel =
			imx_asrc_dma_request_channel(params, true);
	if (params->input_dma_channel == NULL) {
		dev_err(asrc->dev, "failed to request rx channel for Pair %c\n",
				'A' + index);
		return  -EBUSY;
	}

	params->output_dma_channel =
			imx_asrc_dma_request_channel(params, false);
	if (params->output_dma_channel == NULL) {
		dev_err(asrc->dev, "failed to request tx channel for Pair %c\n",
				'A' + index);
		return  -EBUSY;
	}

	init_waitqueue_head(&params->input_wait_queue);
	init_waitqueue_head(&params->output_wait_queue);

	/* Add work struct to receive last period of output data */
	INIT_WORK(&params->task_output_work, asrc_output_task_worker);

	ret = copy_to_user(user, &config, sizeof(config));
	if (ret) {
		dev_err(asrc->dev, "failed to send config to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long asrc_ioctl_release_pair(struct asrc_pair_params *params,
		void __user *user)
{
	enum asrc_pair_index index;
	unsigned long lock_flags;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(asrc->dev, "failed to get index from user space: %ld\n", ret);
		return ret;
	}

	/* index might be not valid due to some application failure. */
	if (index < 0)
		return -EINVAL;

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

	return 0;
}

static long asrc_ioctl_convert(struct asrc_pair_params *params,
		void __user *user)
{
	struct asrc_convert_buffer buf;
	long ret;

	ret = copy_from_user(&buf, user, sizeof(buf));
	if (ret) {
		dev_err(asrc->dev, "failed to get buf from user space: %ld\n", ret);
		return ret;
	}

	ret = mxc_asrc_prepare_buffer(params, &buf);
	if (ret) {
		dev_err(asrc->dev, "failed to prepare buffer: %ld\n", ret);
		return ret;
	}

	mxc_asrc_submit_dma(params);

	ret = mxc_asrc_process_buffer(params, &buf);
	if (ret) {
		dev_err(asrc->dev, "failed to process buffer: %ld\n", ret);
		return ret;
	}

	ret = copy_to_user(user, &buf, sizeof(buf));
	if (ret) {
		dev_err(asrc->dev, "failed to send buf to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long asrc_ioctl_start_conv(struct asrc_pair_params *params,
		void __user *user)
{
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(asrc->dev, "failed to get index from user space: %ld\n", ret);
		return ret;
	}

	params->asrc_active = 1;
	asrc_start_conv(index);

	return 0;
}

static long asrc_ioctl_stop_conv(struct asrc_pair_params *params,
		void __user *user)
{
	enum asrc_pair_index index;
	long ret;

	ret = copy_from_user(&index, user, sizeof(index));
	if (ret) {
		dev_err(asrc->dev, "failed to get index from user space: %ld\n", ret);
		return ret;
	}

	dmaengine_terminate_all(params->input_dma_channel);
	dmaengine_terminate_all(params->output_dma_channel);

	asrc_stop_conv(index);
	params->asrc_active = 0;

	return 0;
}

static long asrc_ioctl_status(struct asrc_pair_params *params,
		void __user *user)
{
	struct asrc_status_flags flags;
	long ret;

	ret = copy_from_user(&flags, user, sizeof(flags));
	if (ret) {
		dev_err(asrc->dev, "failed to get flags from user space: %ld\n", ret);
		return ret;
	}

	asrc_get_status(&flags);

	ret = copy_to_user(user, &flags, sizeof(flags));
	if (ret) {
		dev_err(asrc->dev, "failed to send flags to user space: %ld\n", ret);
		return ret;
	}

	return 0;
}

static long asrc_ioctl_flush(struct asrc_pair_params *params,
		void __user *user)
{
	enum asrc_pair_index index = params->index;
	unsigned long lock_flags;

	/* Flush input dma buffer */
	spin_lock_irqsave(&input_int_lock, lock_flags);
	params->input_counter = 0;
	spin_unlock_irqrestore(&input_int_lock, lock_flags);

	/* Flush output dma buffer */
	spin_lock_irqsave(&output_int_lock, lock_flags);
	params->output_counter = 0;
	spin_unlock_irqrestore(&output_int_lock, lock_flags);

	/* Release DMA and request again */
	dma_release_channel(params->input_dma_channel);
	dma_release_channel(params->output_dma_channel);

	params->input_dma_channel = imx_asrc_dma_request_channel(params, true);
	if (params->input_dma_channel == NULL) {
		dev_err(asrc->dev, "failed to request rx channel for Pair %c\n",
				'A' + index);
		return -EBUSY;
	}

	params->output_dma_channel =
			imx_asrc_dma_request_channel(params, false);
	if (params->output_dma_channel == NULL) {
		dev_err(asrc->dev, "failed to request tx channel for Pair %c\n",
				'A' + index);
		return -EBUSY;
	}

	return 0;
}

static long asrc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct asrc_pair_params *params = file->private_data;
	void __user *user = (void __user *)arg;
	long ret = 0;

	switch (cmd) {
	case ASRC_REQ_PAIR:
		ret = asrc_ioctl_req_pair(params, user);
		break;
	case ASRC_CONFIG_PAIR:
		ret = asrc_ioctl_config_pair(params, user);
		break;
	case ASRC_RELEASE_PAIR:
		ret = asrc_ioctl_release_pair(params, user);
		break;
	case ASRC_CONVERT:
		ret = asrc_ioctl_convert(params, user);
		break;
	case ASRC_START_CONV:
		ret = asrc_ioctl_start_conv(params, user);
		dump_regs();
		break;
	case ASRC_STOP_CONV:
		ret = asrc_ioctl_stop_conv(params, user);
		break;
	case ASRC_STATUS:
		ret = asrc_ioctl_status(params, user);
		break;
	case ASRC_FLUSH:
		ret = asrc_ioctl_flush(params, user);
		break;
	default:
		dev_err(asrc->dev, "Unsupported ioctl cmd!\n");
		break;
	}

	return ret;
}

static int mxc_asrc_open(struct inode *inode, struct file *file)
{
	struct asrc_pair_params *pair_params;
	int ret = 0;

	ret = signal_pending(current);
	if (ret) {
		dev_err(asrc->dev, "Current process has a signal pending.\n");
		return ret;
	}

	pair_params = kzalloc(sizeof(struct asrc_pair_params), GFP_KERNEL);
	if (pair_params == NULL) {
		dev_err(asrc->dev, "failed to allocate pair_params.\n");
		return -ENOBUFS;
	}

	file->private_data = pair_params;

	return ret;
}

static int mxc_asrc_close(struct inode *inode, struct file *file)
{
	struct asrc_pair_params *pair_params;
	unsigned long lock_flags;

	pair_params = file->private_data;

	if (!pair_params)
		return 0;

	if (pair_params->asrc_active) {
		pair_params->asrc_active = 0;

		dmaengine_terminate_all(pair_params->input_dma_channel);
		dmaengine_terminate_all(pair_params->output_dma_channel);

		asrc_stop_conv(pair_params->index);

		wake_up_interruptible(&pair_params->input_wait_queue);
		wake_up_interruptible(&pair_params->output_wait_queue);
	}
	if (pair_params->pair_hold) {
		spin_lock_irqsave(&pair_lock, lock_flags);
		pair_params->pair_hold = 0;
		spin_unlock_irqrestore(&pair_lock, lock_flags);

		if (pair_params->input_dma_channel)
			dma_release_channel(pair_params->input_dma_channel);
		if (pair_params->output_dma_channel)
			dma_release_channel(pair_params->output_dma_channel);

		mxc_free_dma_buf(pair_params);

		asrc_release_pair(pair_params->index);
		asrc_finish_conv(pair_params->index);
	}

	kfree(pair_params);
	file->private_data = NULL;

	return 0;
}

static int mxc_asrc_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			size, vma->vm_page_prot);
	if (ret) {
		dev_err(asrc->dev, "failed to memory map!\n");
		return ret;
	}

	vma->vm_flags &= ~VM_IO;

	return ret;
}

static const struct file_operations asrc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= asrc_ioctl,
	.mmap		= mxc_asrc_mmap,
	.open		= mxc_asrc_open,
	.release	= mxc_asrc_close,
};

static struct miscdevice asrc_miscdev = {
	.name	= "mxc_asrc",
	.fops	= &asrc_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};

static int asrc_read_proc_attr(struct file *file, char __user *buf,
				size_t count, loff_t *off)
{
	char tmpbuf[80];
	int len = 0;
	u32 reg;

	if (*off)
		return 0;

	asrc_regmap_read(asrc->regmap, REG_ASRCNCR, &reg);

	len += sprintf(tmpbuf, "ANCA: %d\nANCB: %d\nANCC: %d\n",
			(int)ASRCNCR_ANCA_get(reg, asrc->channel_bits),
			(int)ASRCNCR_ANCB_get(reg, asrc->channel_bits),
			(int)ASRCNCR_ANCC_get(reg, asrc->channel_bits));

	if (len > count)
		return 0;

	if (copy_to_user(buf, &tmpbuf, len))
		return -EFAULT;

	*off += len;

	return len;
}

#define ASRC_MAX_PROC_BUFFER_SIZE 63

static int asrc_write_proc_attr(struct file *file, const char __user *buffer,
				size_t count, loff_t *data)
{
	char buf[ASRC_MAX_PROC_BUFFER_SIZE];
	int na, nb, nc;
	int total;

	if (count > ASRC_MAX_PROC_BUFFER_SIZE) {
		dev_err(asrc->dev, "Attr proc write: The input string was too long.\n");
		return -EINVAL;
	}

	if (copy_from_user(buf, buffer, count)) {
		dev_err(asrc->dev, "Attr proc write: Failed to copy buffer from user.\n");
		return -EFAULT;
	}

	sscanf(buf, "ANCA: %d\nANCB: %d\nANCC: %d", &na, &nb, &nc);

	total = asrc->channel_bits > 3 ? 10 : 5;

	if (na + nb + nc > total) {
		dev_err(asrc->dev, "Don't surpass %d for total.\n", total);
		return -EINVAL;
	} else if (na % 2 != 0 || nb % 2 != 0 || nc % 2 != 0) {
		dev_err(asrc->dev, "Please set an even number for each pair.\n");
		return -EINVAL;
	} else if (na < 0 || nb < 0 || nc < 0) {
		dev_err(asrc->dev, "Please set an positive number for each pair.\n");
		return -EINVAL;
	}


	asrc->asrc_pair[ASRC_PAIR_A].chn_max = na;
	asrc->asrc_pair[ASRC_PAIR_B].chn_max = nb;
	asrc->asrc_pair[ASRC_PAIR_C].chn_max = nc;

	asrc_set_channel_number(ASRC_PAIR_A, na);
	asrc_set_channel_number(ASRC_PAIR_B, nb);
	asrc_set_channel_number(ASRC_PAIR_C, nc);

	return count;
}

static const struct file_operations asrc_proc_fops = {
	.read		= asrc_read_proc_attr,
	.write		= asrc_write_proc_attr,
};

static void asrc_proc_create(void)
{
	struct proc_dir_entry *proc_attr;

	asrc->proc_asrc = proc_mkdir(ASRC_PROC_PATH, NULL);
	if (!asrc->proc_asrc) {
		dev_err(asrc->dev, "failed to create proc entry %s\n", ASRC_PROC_PATH);
		return;
	}

	proc_attr = proc_create("ChSettings", S_IFREG | S_IRUGO | S_IWUSR,
			asrc->proc_asrc, &asrc_proc_fops);
	if (!proc_attr) {
		remove_proc_entry(ASRC_PROC_PATH, NULL);
		dev_err(asrc->dev, "failed to create proc attribute entry.\n");
	}
}

static void asrc_proc_remove(void)
{
	remove_proc_entry("ChSettings", asrc->proc_asrc);
	remove_proc_entry(ASRC_PROC_PATH, NULL);
}


#ifdef ASRC_USE_REGMAP
/* ============= ASRC REGMAP ============= */

static bool asrc_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_ASRCTR:
	case REG_ASRIER:
	case REG_ASRCNCR:
	case REG_ASRCFG:
	case REG_ASRCSR:
	case REG_ASRCDR1:
	case REG_ASRCDR2:
	case REG_ASRSTR:
	case REG_ASRPM1:
	case REG_ASRPM2:
	case REG_ASRPM3:
	case REG_ASRPM4:
	case REG_ASRPM5:
	case REG_ASRTFR1:
	case REG_ASRCCR:
	case REG_ASRDOA:
	case REG_ASRDOB:
	case REG_ASRDOC:
	case REG_ASRIDRHA:
	case REG_ASRIDRLA:
	case REG_ASRIDRHB:
	case REG_ASRIDRLB:
	case REG_ASRIDRHC:
	case REG_ASRIDRLC:
	case REG_ASR76K:
	case REG_ASR56K:
	case REG_ASRMCRA:
	case REG_ASRFSTA:
	case REG_ASRMCRB:
	case REG_ASRFSTB:
	case REG_ASRMCRC:
	case REG_ASRFSTC:
	case REG_ASRMCR1A:
	case REG_ASRMCR1B:
	case REG_ASRMCR1C:
		return true;
	default:
		return false;
	}
}

static bool asrc_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_ASRCTR:
	case REG_ASRIER:
	case REG_ASRCNCR:
	case REG_ASRCFG:
	case REG_ASRCSR:
	case REG_ASRCDR1:
	case REG_ASRCDR2:
	case REG_ASRPM1:
	case REG_ASRPM2:
	case REG_ASRPM3:
	case REG_ASRPM4:
	case REG_ASRPM5:
	case REG_ASRTFR1:
	case REG_ASRCCR:
	case REG_ASRDIA:
	case REG_ASRDIB:
	case REG_ASRDIC:
	case REG_ASRIDRHA:
	case REG_ASRIDRLA:
	case REG_ASRIDRHB:
	case REG_ASRIDRLB:
	case REG_ASRIDRHC:
	case REG_ASRIDRLC:
	case REG_ASR76K:
	case REG_ASR56K:
	case REG_ASRMCRA:
	case REG_ASRMCRB:
	case REG_ASRMCRC:
	case REG_ASRMCR1A:
	case REG_ASRMCR1B:
	case REG_ASRMCR1C:
		return true;
	default:
		return false;
	}
}

static bool asrc_volatile_reg(struct device *dev, unsigned int reg)
{
	/* Sync all registers after reset */
	return true;
}

static const struct regmap_config asrc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = REG_ASRMCR1C,
	.readable_reg = asrc_readable_reg,
	.writeable_reg = asrc_writeable_reg,
	.volatile_reg = asrc_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};
#endif

static int mxc_asrc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource res;
	void __iomem *regs;
	int ret;

	/* Check if the device is existed */
	if (!of_device_is_available(np)) {
		dev_err(&pdev->dev, "improper devicetree status.\n");
		return -ENODEV;
	}

	asrc = devm_kzalloc(&pdev->dev, sizeof(struct asrc_data), GFP_KERNEL);
	if (asrc == NULL) {
		dev_err(&pdev->dev, "failed to allocate asrc.\n");
		return -ENOMEM;
	}

	asrc->dev = &pdev->dev;
	asrc->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	asrc->asrc_pair[ASRC_PAIR_A].chn_max = 2;
	asrc->asrc_pair[ASRC_PAIR_B].chn_max = 6;
	asrc->asrc_pair[ASRC_PAIR_C].chn_max = 2;
	asrc->asrc_pair[ASRC_PAIR_A].overload_error = 0;
	asrc->asrc_pair[ASRC_PAIR_B].overload_error = 0;
	asrc->asrc_pair[ASRC_PAIR_C].overload_error = 0;

	/* Map the address */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "failed to map address: %d\n", ret);
		return ret;
	}
	asrc->paddr = res.start;

	regs = of_iomap(np, 0);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "failed to map io resources.\n");
		return IS_ERR(regs);
	}
	asrc->vaddr = (unsigned long)regs;

#ifdef ASRC_USE_REGMAP
	asrc->regmap = devm_regmap_init_mmio_clk(&pdev->dev,
			"core", regs, &asrc_regmap_config);
	if (IS_ERR(asrc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(asrc->regmap);
	}

	regcache_cache_only(asrc->regmap, false);
#endif

	asrc->irq = irq_of_parse_and_map(np, 0);
	if (asrc->irq == NO_IRQ) {
		dev_err(&pdev->dev, "no irq for node %s\n", np->full_name);
		goto err_iomap;
	}

	ret = devm_request_irq(&pdev->dev, asrc->irq,
			asrc_isr, 0, "asrc", NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ: %d\n", ret);
		goto err_iomap;
	}

	asrc->asrc_clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(asrc->asrc_clk)) {
		ret = PTR_ERR(asrc->asrc_clk);
		goto err_iomap;
	}

	asrc->dma_clk = devm_clk_get(&pdev->dev, "dma");
	if (IS_ERR(asrc->dma_clk)) {
		ret = PTR_ERR(asrc->dma_clk);
		goto err_iomap;
	}
#ifndef ASRC_USE_REGMAP
	clk_prepare(asrc->asrc_clk);
	clk_prepare(asrc->dma_clk);
#endif

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"fsl,clk-channel-bits", &asrc->channel_bits, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to get clk-channel-bits.\n");
		goto err_iomap;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"fsl,clk-map-version", &asrc->clk_map_ver, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to get clk-map-version.\n");
		goto err_iomap;
	}

	switch (asrc->clk_map_ver) {
	case 1:
		input_clk_map = input_clk_map_v1;
		output_clk_map = output_clk_map_v1;
		break;
	case 2:
	default:
		input_clk_map = input_clk_map_v2;
		output_clk_map = output_clk_map_v2;
		break;
	}

	ret = misc_register(&asrc_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register char device %d\n", ret);
		goto err_iomap;
	}

	asrc_proc_create();

	ret = mxc_init_asrc();
	if (ret)
		goto err_misc;

	dev_info(&pdev->dev, "mxc_asrc registered.\n");

	return ret;

err_misc:
	misc_deregister(&asrc_miscdev);
err_iomap:
	iounmap(regs);

	dev_err(&pdev->dev, "mxc_asrc register failed: err %d\n", ret);

	return ret;
}

static int mxc_asrc_remove(struct platform_device *pdev)
{
#ifndef ASRC_USE_REGMAP
	clk_unprepare(asrc->dma_clk);
	clk_unprepare(asrc->asrc_clk);
#endif
	asrc_proc_remove();
	misc_deregister(&asrc_miscdev);

	return 0;
}

static const struct of_device_id fsl_asrc_ids[] = {
	{ .compatible = "fsl,imx6q-asrc", },
	{}
};

static struct platform_driver mxc_asrc_driver = {
	.driver = {
		.name = "mxc_asrc",
		.of_match_table = fsl_asrc_ids,
	},
	.probe = mxc_asrc_probe,
	.remove = mxc_asrc_remove,
};

module_platform_driver(mxc_asrc_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Asynchronous Sample Rate Converter");
MODULE_LICENSE("GPL");
