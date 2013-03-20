/*
 * MXC SPDIF ALSA Soc Codec Driver
 *
 * Copyright (C) 2007-2013 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/asoundef.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include "mxc_spdif.h"

#define MXC_SPDIF_DEBUG 0

static unsigned int gainsel_multi[GAINSEL_MULTI_MAX] = {
	24 * 1024, 16 * 1024, 12 * 1024,
	8 * 1024, 6 * 1024, 4 * 1024,
	3 * 1024,
};

/*
 * SPDIF control structure
 * Defines channel status, subcode and Q sub
 */
struct spdif_mixer_control {
	/* spinlock to access control data */
	spinlock_t ctl_lock;
	/* IEC958 channel tx status bit */
	unsigned char ch_status[4];
	/* User bits */
	unsigned char subcode[2 * SPDIF_UBITS_SIZE];
	/* Q subcode part of user bits */
	unsigned char qsub[2 * SPDIF_QSUB_SIZE];
	/* buffer ptrs for writer */
	unsigned int upos;
	unsigned int qpos;
	/* ready buffer index of the two buffers */
	unsigned int ready_buf;
};

struct mxc_spdif_priv {
	struct snd_soc_codec codec;
	struct mxc_spdif_platform_data *plat_data;
	unsigned long __iomem *reg_base;
	unsigned long reg_phys_base;
	struct snd_card *card;	/* ALSA SPDIF sound card handle */
	struct snd_pcm *pcm;	/* ALSA spdif driver type handle */
	atomic_t dpll_locked;	/* DPLL locked status */
	bool tx_active;
	bool rx_active;
};

struct spdif_mixer_control mxc_spdif_control;

static unsigned long spdif_base_addr;

#if MXC_SPDIF_DEBUG
static void dumpregs(struct mxc_spdif_priv *priv)
{
	unsigned int value, i;

	if (!priv->tx_active && !priv->rx_active)
		clk_enable(priv->plat_data->spdif_core_clk);

	for (i = 0 ; i <= 0x38 ; i += 4) {
		value = readl(spdif_base_addr + i) & 0xffffff;
		pr_debug("reg 0x%02x = 0x%06x\n", i, value);
	}

	i = 0x44;
	value = readl(spdif_base_addr + i) & 0xffffff;
	pr_debug("reg 0x%02x = 0x%06x\n", i, value);

	i = 0x50;
	value = readl(spdif_base_addr + i) & 0xffffff;
	pr_debug("reg 0x%02x = 0x%06x\n", i, value);

	if (!priv->tx_active && !priv->rx_active)
		clk_disable(priv->plat_data->spdif_core_clk);
}
#else
static void dumpregs(struct mxc_spdif_priv *priv) {}
#endif

/* define each spdif interrupt handlers */
typedef void (*spdif_irq_func_t) (unsigned int bit, void *desc);

/* spdif irq functions declare */
static void spdif_irq_fifo(unsigned int bit, void *devid);
static void spdif_irq_dpll_lock(unsigned int bit, void *devid);
static void spdif_irq_uq(unsigned int bit, void *devid);
static void spdif_irq_bit_error(unsigned int bit, void *devid);
static void spdif_irq_sym_error(unsigned int bit, void *devid);
static void spdif_irq_valnogood(unsigned int bit, void *devid);
static void spdif_irq_cnew(unsigned int bit, void *devid);

/* irq function list */
static spdif_irq_func_t spdif_irq_handlers[] = {
	spdif_irq_fifo,
	spdif_irq_fifo,
	spdif_irq_dpll_lock,
	NULL,
	spdif_irq_fifo,
	spdif_irq_uq,
	spdif_irq_uq,
	spdif_irq_uq,
	spdif_irq_uq,
	spdif_irq_uq,
	spdif_irq_uq,
	NULL,
	NULL,
	NULL,
	spdif_irq_bit_error,
	spdif_irq_sym_error,
	spdif_irq_valnogood,
	spdif_irq_cnew,
	NULL,
	spdif_irq_fifo,
	spdif_irq_dpll_lock,
};

static void spdif_intr_enable(unsigned long intr, int enable)
{
	unsigned long value;

	value = __raw_readl(spdif_base_addr + SPDIF_REG_SIE) & 0xffffff;

	if (enable)
		value |= intr;
	else
		value &= ~intr;

	__raw_writel(value, spdif_base_addr + SPDIF_REG_SIE);
}

/*
 * Get spdif interrupt status and clear the interrupt
 */
static int spdif_intr_status(void)
{
	unsigned long value;

	value = __raw_readl(SPDIF_REG_SIS + spdif_base_addr) & 0xffffff;
	value &= __raw_readl(spdif_base_addr + SPDIF_REG_SIE);
	__raw_writel(value, SPDIF_REG_SIC + spdif_base_addr);

	return value;
}

static irqreturn_t spdif_isr(int irq, void *dev_id)
{
	unsigned long int_stat;
	int line;

	int_stat = spdif_intr_status();

	while ((line = ffs(int_stat)) != 0) {
		int_stat &= ~(1UL << (line - 1));
		if (spdif_irq_handlers[line - 1] != NULL)
			spdif_irq_handlers[line - 1] (line - 1, dev_id);
	}

	return IRQ_HANDLED;
}

/*
 * Rx FIFO Full, Underrun/Overrun interrupts
 * Tx FIFO Empty, Underrun/Overrun interrupts
 */
static void spdif_irq_fifo(unsigned int bit, void *devid)
{
	pr_debug("irq %s\n", __func__);
}

/*
 * DPLL locked and lock loss interrupt handler
 */
static void spdif_irq_dpll_lock(unsigned int bit, void *devid)
{
	struct mxc_spdif_priv *spdif_priv = (struct mxc_spdif_priv *)devid;
	unsigned int locked = __raw_readl(spdif_base_addr + SPDIF_REG_SRPC);

	if (locked & SRPC_DPLL_LOCKED) {
		pr_debug("SPDIF Rx dpll locked\n");
		atomic_set(&spdif_priv->dpll_locked, 1);
	} else {
		/* INT_LOSS_LOCK */
		pr_debug("SPDIF Rx dpll loss lock\n");
		atomic_set(&spdif_priv->dpll_locked, 0);
	}
}

/*
 * U/Q channel related interrupts handler
 *
 * U/QChannel full, overrun interrupts
 * U/QChannel sync error and frame error interrupts
 */
static void spdif_irq_uq(unsigned int bit, void *devid)
{
	unsigned long val;
	int index;
	struct spdif_mixer_control *ctrl = &mxc_spdif_control;

	bit = 1 << bit;
	/* get U/Q channel datas */
	switch (bit) {

	case INT_URX_OV:	/* read U data */
		pr_debug("User bit receive overrun\n");
	case INT_URX_FUL:
		pr_debug("U bit receive full\n");

		if (ctrl->upos >= SPDIF_UBITS_SIZE * 2) {
			ctrl->upos = 0;
		} else if (unlikely((ctrl->upos % SPDIF_UBITS_SIZE) + 3
				    > SPDIF_UBITS_SIZE)) {
			pr_err("User bit receivce buffer overflow\n");
			break;
		}
		val = __raw_readl(spdif_base_addr + SPDIF_REG_SQU);
		ctrl->subcode[ctrl->upos++] = val >> 16;
		ctrl->subcode[ctrl->upos++] = val >> 8;
		ctrl->subcode[ctrl->upos++] = val;

		break;

	case INT_QRX_OV:	/* read Q data */
		pr_debug("Q bit receive overrun\n");
	case INT_QRX_FUL:
		pr_debug("Q bit receive full\n");

		if (ctrl->qpos >= SPDIF_QSUB_SIZE * 2) {
			ctrl->qpos = 0;
		} else if (unlikely((ctrl->qpos % SPDIF_QSUB_SIZE) + 3
				    > SPDIF_QSUB_SIZE)) {
			pr_debug("Q bit receivce buffer overflow\n");
			break;
		}
		val = __raw_readl(spdif_base_addr + SPDIF_REG_SRQ);
		ctrl->qsub[ctrl->qpos++] = val >> 16;
		ctrl->qsub[ctrl->qpos++] = val >> 8;
		ctrl->qsub[ctrl->qpos++] = val;

		break;

	case INT_UQ_ERR:	/* read U/Q data and do buffer reset */
		pr_debug("U/Q bit receive error\n");
		val = __raw_readl(spdif_base_addr + SPDIF_REG_SQU);
		val = __raw_readl(spdif_base_addr + SPDIF_REG_SRQ);
		/* drop this U/Q buffer */
		ctrl->ready_buf = 0;
		ctrl->upos = 0;
		ctrl->qpos = 0;
		break;

	case INT_UQ_SYNC:	/* U/Q buffer reset */
		pr_debug("U/Q sync receive\n");

		if (ctrl->qpos == 0)
			break;
		index = (ctrl->qpos - 1) / SPDIF_QSUB_SIZE;
		/* set ready to this buffer */
		ctrl->ready_buf = index + 1;
		break;
	}
}

/*
 * SPDIF receiver found parity bit error interrupt handler
 */
static void spdif_irq_bit_error(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt parity bit error\n");
}

/*
 * SPDIF receiver found illegal symbol interrupt handler
 */
static void spdif_irq_sym_error(unsigned int bit, void *devid)
{
	struct mxc_spdif_priv *spdif_priv = (struct mxc_spdif_priv *)devid;

	pr_debug("SPDIF interrupt symbol error\n");
	if (!atomic_read(&spdif_priv->dpll_locked)) {
		/* dpll unlocked seems no audio stream */
		spdif_intr_enable(INT_SYM_ERR, 0);
	}
}

/*
 * SPDIF validity flag no good interrupt handler
 */
static void spdif_irq_valnogood(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt validate is not good\n");
}

/*
 * SPDIF receive change in value of control channel
 */
static void spdif_irq_cnew(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt cstatus new\n");
}

static void spdif_softreset(void)
{
	unsigned long value = 1;
	int cycle = 0;

	__raw_writel(SCR_SOFT_RESET, spdif_base_addr + SPDIF_REG_SCR);

	while (value && (cycle++ < 10))
		value = __raw_readl(spdif_base_addr + SPDIF_REG_SCR) & 0x1000;
}

static void spdif_set_clk_accuracy(u8 level)
{
	mxc_spdif_control.ch_status[3] &= ~IEC958_AES3_CON_CLOCK;
	mxc_spdif_control.ch_status[3] |= level & IEC958_AES3_CON_CLOCK;
}

static void spdif_set_cstatus_fs(u8 cstatus_fs)
{
	/* set fs field in consumer channel status */
	mxc_spdif_control.ch_status[3] &= ~IEC958_AES3_CON_FS;
	mxc_spdif_control.ch_status[3] |= cstatus_fs & IEC958_AES3_CON_FS;
}

static u8 reverse_bits(u8 input)
{
	u8 i, output = 0;
	for (i = 8 ; i > 0 ; i--) {
		output <<= 1;
		output |= input & 0x01;
		input >>= 1;
	}
	return output;
}

static void spdif_write_channel_status(void)
{
	unsigned int ch_status;

	ch_status =
		(reverse_bits(mxc_spdif_control.ch_status[0]) << 16) |
		(reverse_bits(mxc_spdif_control.ch_status[1]) << 8) |
		reverse_bits(mxc_spdif_control.ch_status[2]);

	__raw_writel(ch_status, SPDIF_REG_STCSCH + spdif_base_addr);

	ch_status = reverse_bits(mxc_spdif_control.ch_status[3]) << 16;
	__raw_writel(ch_status, SPDIF_REG_STCSCL + spdif_base_addr);

	pr_debug("STCSCH: 0x%06x\n", __raw_readl(spdif_base_addr + SPDIF_REG_STCSCH));
	pr_debug("STCSCL: 0x%06x\n", __raw_readl(spdif_base_addr + SPDIF_REG_STCSCL));
}

/*
 * set SPDIF PhaseConfig register for rx clock
 */
static int spdif_set_rx_clksrc(enum spdif_clk_src clksrc,
			       enum spdif_gainsel gainsel, int dpll_locked)
{
	unsigned long value;

	if (clksrc > SPDIF_CLK_SRC5 || gainsel > GAINSEL_MULTI_3)
		return 1;

	if (dpll_locked)
		value = clksrc;
	else
		value = clksrc + SRPC_CLKSRC_SEL_LOCKED;

	value = (value << SRPC_CLKSRC_SEL_OFFSET) |
		(gainsel << SRPC_GAINSEL_OFFSET);

	__raw_writel(value, spdif_base_addr + SPDIF_REG_SRPC);

	return 0;
}

/*
 * Get RX data clock rate given the SPDIF bus_clk
 */
static int spdif_get_rxclk_rate(struct clk *bus_clk, enum spdif_gainsel gainsel)
{
	unsigned long freqmeas, phaseconf, busclk_freq = 0;
	u64 tmpval64;
	enum spdif_clk_src clksrc;

	freqmeas = __raw_readl(spdif_base_addr + SPDIF_REG_SRFM);
	phaseconf = __raw_readl(spdif_base_addr + SPDIF_REG_SRPC);

	clksrc = (phaseconf >> SRPC_CLKSRC_SEL_OFFSET) & 0x0F;
	if (clksrc < 5 && (phaseconf & SRPC_DPLL_LOCKED)) {
		/* get bus clock from system */
		busclk_freq = clk_get_rate(bus_clk);
	}

	/* FreqMeas_CLK = (BUS_CLK*FreqMeas[23:0])/2^10/GAINSEL/128 */
	tmpval64 = (u64) busclk_freq * freqmeas;
	do_div(tmpval64, gainsel_multi[gainsel]);
	do_div(tmpval64, 128 * 1024);

	pr_debug("FreqMeas:    %d\n", (int)freqmeas);
	pr_debug("busclk_freq: %d\n", (int)busclk_freq);
	pr_debug("rx rate:     %d\n", (int)tmpval64);

	return (int)tmpval64;
}

static int spdif_set_sample_rate(struct snd_soc_codec *codec, int sample_rate)
{
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned long stc, clk = -1, div = 1, cstatus_fs = 0;
	int clk_fs;

	switch (sample_rate) {
	case 44100:
		clk_fs = 44100;
		clk = plat_data->spdif_clk_44100;
		div = plat_data->spdif_div_44100;
		cstatus_fs = IEC958_AES3_CON_FS_44100;
		break;

	case 48000:
		clk_fs = 48000;
		clk = plat_data->spdif_clk_48000;
		div = plat_data->spdif_div_48000;
		cstatus_fs = IEC958_AES3_CON_FS_48000;
		break;

	case 32000:
		/* use 48K clk for 32K */
		clk_fs = 48000;
		clk = plat_data->spdif_clk_48000;
		div = plat_data->spdif_div_32000;
		cstatus_fs = IEC958_AES3_CON_FS_32000;
		break;

	default:
		pr_err("%s: unsupported sample rate %d\n", __func__, sample_rate);
		return -EINVAL;
	}

	if (clk < 0) {
		pr_info("%s: no defined %d clk src\n", __func__, clk_fs);
		return -EINVAL;
	}

	/*
	 * The S/PDIF block needs a clock of 64 * fs * div.  The S/PDIF block
	 * will divide by (div).  So request 64 * fs * (div+1) which will
	 * get rounded.
	 */
	if (plat_data->spdif_clk_set_rate)
		plat_data->spdif_clk_set_rate(plat_data->spdif_clk,
					      64 * sample_rate * (div + 1));

#if MXC_SPDIF_DEBUG
	pr_debug("%s wanted spdif clock rate = %d\n", __func__,
		(int)(64 * sample_rate * div));
	pr_debug("%s got spdif clock rate    = %d\n", __func__,
		(int)clk_get_rate(plat_data->spdif_clk));
#endif

	spdif_set_cstatus_fs(cstatus_fs);

	/* select clock source and divisor */
	stc = __raw_readl(SPDIF_REG_STC + spdif_base_addr) & ~0x7FF;
	stc |= STC_TXCLK_SRC_EN | (clk << STC_TXCLK_SRC_OFFSET) | (div - 1);
	__raw_writel(stc, SPDIF_REG_STC + spdif_base_addr);
	pr_debug("set sample rate to %d\n", sample_rate);

	return 0;
}

static unsigned int spdif_playback_rates[] = { 32000, 44100, 48000 };

static unsigned int spdif_capture_rates[] = {
	16000, 32000, 44100, 48000, 64000, 96000
};

static struct snd_pcm_hw_constraint_list hw_playback_rates_stereo = {
	.count = ARRAY_SIZE(spdif_playback_rates),
	.list = spdif_playback_rates,
	.mask = 0,
};

static struct snd_pcm_hw_constraint_list hw_capture_rates_stereo = {
	.count = ARRAY_SIZE(spdif_capture_rates),
	.list = spdif_capture_rates,
	.mask = 0,
};

static int mxc_spdif_playback_startup(struct snd_pcm_substream *substream,
					       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	int err = 0;

	if (!plat_data->spdif_tx)
		return -EINVAL;

	spdif_priv->tx_active = true;
	err = clk_enable(plat_data->spdif_clk);
	if (err < 0)
		goto failed_clk;

	err = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &hw_playback_rates_stereo);
	if (err < 0)
		goto failed;
	err = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		goto failed;

	return 0;

failed:
	clk_disable(plat_data->spdif_clk);
failed_clk:
	spdif_priv->tx_active = false;
	return err;
}

static int mxc_spdif_playback_start(struct snd_pcm_substream *substream,
					  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long regval;
	int err;

	if (!plat_data->spdif_tx)
		return -EINVAL;

	spdif_priv->tx_active = true;
	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);
	regval &= 0xfc33e3;
	regval &= ~SCR_LOW_POWER;
	regval |= SCR_TXFIFO_AUTOSYNC | SCR_TXFIFO_NORMAL |
	    SCR_TXSEL_NORMAL | SCR_USRC_SEL_CHIP | (2 << SCR_TXFIFO_ESEL_BIT);
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	/* Default clock source from EXTAL, divider by 8, generate 44.1kHz
	   sample rate */
	__raw_writel(0x07, SPDIF_REG_STC + spdif_base_addr);

	spdif_intr_enable(INT_TXFIFO_RESYNC, 1);

	err = spdif_set_sample_rate(codec, runtime->rate);
	if (err < 0) {
		pr_info("%s - err < 0\n", __func__);
		return err;
	}
	spdif_set_clk_accuracy(IEC958_AES3_CON_CLOCK_1000PPM);
	spdif_write_channel_status();

	pr_debug("SCR: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SCR));
	pr_debug("SIE: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIE));
	pr_debug("STC: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_STC));

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr);
	regval |= SCR_DMA_TX_EN;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	dumpregs(spdif_priv);
	return 0;
}

static int mxc_spdif_playback_stop(struct snd_pcm_substream *substream,
						struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned long regval;

	if (!plat_data->spdif_tx)
		return -EINVAL;

	dumpregs(spdif_priv);
	pr_debug("SIS: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIS));

	spdif_intr_status();
	spdif_intr_enable(INT_TXFIFO_RESYNC, 0);

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr) & 0xffffe3;
	regval |= SCR_TXSEL_OFF;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);
	regval = __raw_readl(SPDIF_REG_STC + spdif_base_addr) & ~0x7FF;
	regval |= (0x7 << STC_TXCLK_SRC_OFFSET);
	__raw_writel(regval, SPDIF_REG_STC + spdif_base_addr);

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr);
	regval &= ~SCR_DMA_TX_EN;
	regval |= SCR_LOW_POWER;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	spdif_priv->tx_active = false;

	return 0;
}

static int mxc_spdif_playback_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	int ret = 0;

	if (!plat_data->spdif_tx)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = mxc_spdif_playback_start(substream, dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = mxc_spdif_playback_stop(substream, dai);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int mxc_spdif_capture_startup(struct snd_pcm_substream *substream,
					      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	int err = 0;

	if (!plat_data->spdif_rx)
		return -EINVAL;

	spdif_priv->rx_active = true;

	/* enable rx bus clock */
	err = clk_enable(plat_data->spdif_clk);
	if (err < 0)
		goto failed_clk;

	/* set hw param constraints */
	err = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &hw_capture_rates_stereo);
	if (err < 0)
		goto failed;

	err = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		goto failed;

	return 0;

failed:
	clk_disable(plat_data->spdif_clk);
failed_clk:
	spdif_priv->rx_active = false;
	return err;
}

static int mxc_spdif_capture_start(struct snd_pcm_substream *substream,
					 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned long regval;

	if (!plat_data->spdif_rx)
		return -EINVAL;

	spdif_priv->rx_active = true;

	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);
	/*
	 * initial and reset SPDIF configuration:
	 * RxFIFO off
	 * RxFIFO sel to 8 sample
	 * Autosync
	 * Valid bit set
	 */
	regval &= ~(SCR_RXFIFO_OFF | SCR_RXFIFO_CTL_ZERO | SCR_LOW_POWER);
	regval |= (2 << SCR_RXFIFO_FSEL_BIT) | SCR_RXFIFO_AUTOSYNC;
	__raw_writel(regval, spdif_base_addr + SPDIF_REG_SCR);

	/* enable interrupts, include DPLL lock */
	spdif_intr_enable(INT_SYM_ERR | INT_BIT_ERR | INT_URX_FUL |
			  INT_URX_OV | INT_QRX_FUL | INT_QRX_OV |
			  INT_UQ_SYNC | INT_UQ_ERR | INT_RX_RESYNC |
			  INT_LOSS_LOCK | INT_DPLL_LOCKED, 1);

	/* setup rx clock source */
	spdif_set_rx_clksrc(plat_data->spdif_rx_clk, SPDIF_DEFAULT_GAINSEL, 1);

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr);
	regval |= SCR_DMA_RX_EN;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	/* Debug: dump registers */
	pr_debug("SCR: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SCR));
	pr_debug("SIE: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIE));
	pr_debug("SRPC: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SRPC));
	pr_debug("FreqMeas: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SRFM));

	return 0;
}

static int mxc_spdif_capture_stop(struct snd_pcm_substream *substream,
					       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned long regval;

	if (!plat_data->spdif_rx || !spdif_priv->rx_active)
		return -EINVAL;

	pr_debug("SIS: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIS));
	pr_debug("SRPC: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SRPC));
	pr_debug("FreqMeas: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SRFM));

	spdif_intr_enable(INT_DPLL_LOCKED | INT_SYM_ERR | INT_BIT_ERR |
			  INT_URX_FUL | INT_URX_OV | INT_QRX_FUL | INT_QRX_OV |
			  INT_UQ_SYNC | INT_UQ_ERR | INT_RX_RESYNC |
			  INT_LOSS_LOCK, 0);

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr);
	regval &= SCR_DMA_RX_EN;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	/* turn off RX fifo, disable dma and autosync */
	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);
	regval |= SCR_RXFIFO_OFF | SCR_RXFIFO_CTL_ZERO | SCR_LOW_POWER;
	regval &= ~(SCR_DMA_RX_EN | SCR_RXFIFO_AUTOSYNC);
	__raw_writel(regval, spdif_base_addr + SPDIF_REG_SCR);

	spdif_priv->rx_active = false;

	return 0;
}

static int mxc_spdif_capture_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	int ret = 0;

	if (!plat_data->spdif_rx)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = mxc_spdif_capture_start(substream, dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = mxc_spdif_capture_stop(substream, dai);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * MXC SPDIF IEC958 controller(mixer) functions
 *
 *	Channel status get/put control
 *	User bit value get/put control
 *	Valid bit value get control
 *	DPLL lock status get control
 *	User bit sync mode selection control
 */
static int mxc_pb_spdif_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int mxc_pb_spdif_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.iec958.status[0] = mxc_spdif_control.ch_status[0];
	uvalue->value.iec958.status[1] = mxc_spdif_control.ch_status[1];
	uvalue->value.iec958.status[2] = mxc_spdif_control.ch_status[2];
	uvalue->value.iec958.status[3] = mxc_spdif_control.ch_status[3];
	return 0;
}

static int mxc_pb_spdif_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;

	mxc_spdif_control.ch_status[0] = uvalue->value.iec958.status[0];
	mxc_spdif_control.ch_status[1] = uvalue->value.iec958.status[1];
	mxc_spdif_control.ch_status[2] = uvalue->value.iec958.status[2];
	mxc_spdif_control.ch_status[3] = uvalue->value.iec958.status[3];

	clk_enable(plat_data->spdif_clk);

	spdif_write_channel_status();

	clk_disable(plat_data->spdif_clk);

	return 0;
}

static int mxc_spdif_info(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

/*
 * Get channel status from SPDIF_RX_CCHAN register
 */
static int mxc_spdif_capture_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned int cstatus;

	clk_enable(plat_data->spdif_clk);

	if (!(__raw_readl(spdif_base_addr + SPDIF_REG_SIS) & INT_CNEW)) {
		clk_disable(plat_data->spdif_clk);
		return -EAGAIN;
	}

	cstatus = __raw_readl(spdif_base_addr + SPDIF_REG_SRCSLH);
	ucontrol->value.iec958.status[0] = (cstatus >> 16) & 0xFF;
	ucontrol->value.iec958.status[1] = (cstatus >> 8) & 0xFF;
	ucontrol->value.iec958.status[2] = cstatus & 0xFF;
	cstatus = __raw_readl(spdif_base_addr + SPDIF_REG_SRCSLL);
	ucontrol->value.iec958.status[3] = (cstatus >> 16) & 0xFF;
	ucontrol->value.iec958.status[4] = (cstatus >> 8) & 0xFF;
	ucontrol->value.iec958.status[5] = cstatus & 0xFF;

	/* clear intr */
	__raw_writel(INT_CNEW, spdif_base_addr + SPDIF_REG_SIC);

	clk_disable(plat_data->spdif_clk);

	return 0;
}

/*
 * Get User bits (subcode) from chip value which readed out
 * in UChannel register.
 */
static int mxc_spdif_subcode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mxc_spdif_control.ctl_lock, flags);
	if (mxc_spdif_control.ready_buf) {
		memcpy(&ucontrol->value.iec958.subcode[0],
		       &mxc_spdif_control.subcode[(mxc_spdif_control.ready_buf -
						   1) * SPDIF_UBITS_SIZE],
		       SPDIF_UBITS_SIZE);
	} else {
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&mxc_spdif_control.ctl_lock, flags);

	return ret;
}

/*
 * Q-subcode infomation.
 * the byte size is SPDIF_UBITS_SIZE/8
 */
static int mxc_spdif_qinfo(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = SPDIF_QSUB_SIZE;
	return 0;
}

/*
 * Get Q subcode from chip value which readed out
 * in QChannel register.
 */
static int mxc_spdif_qget(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mxc_spdif_control.ctl_lock, flags);
	if (mxc_spdif_control.ready_buf) {
		memcpy(&ucontrol->value.bytes.data[0],
		       &mxc_spdif_control.qsub[(mxc_spdif_control.ready_buf -
						1) * SPDIF_QSUB_SIZE],
		       SPDIF_QSUB_SIZE);
	} else {
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&mxc_spdif_control.ctl_lock, flags);

	return ret;
}

/*
 * Valid bit infomation.
 */
static int mxc_spdif_vbit_info(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 * Get valid good bit from interrupt status register.
 */
static int mxc_spdif_vbit_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned int int_val;

	clk_enable(plat_data->spdif_clk);

	int_val = __raw_readl(spdif_base_addr + SPDIF_REG_SIS);
	ucontrol->value.integer.value[0] = (int_val & INT_VAL_NOGOOD) != 0;
	__raw_writel(INT_VAL_NOGOOD, spdif_base_addr + SPDIF_REG_SIC);

	clk_disable(plat_data->spdif_clk);

	return 0;
}

/*
 * DPLL lock infomation.
 */
static int mxc_spdif_rxrate_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 16000;
	uinfo->value.integer.max = 96000;
	return 0;
}

/*
 * Get DPLL lock or not info from stable interrupt status register.
 * User application must use this control to get locked,
 * then can do next PCM operation
 */
static int mxc_spdif_rxrate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);

	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;

	if (atomic_read(&spdif_priv->dpll_locked)) {
		clk_enable(plat_data->spdif_clk);
		ucontrol->value.integer.value[0] =
		    spdif_get_rxclk_rate(plat_data->spdif_clk,
					 SPDIF_DEFAULT_GAINSEL);
		clk_disable(plat_data->spdif_clk);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}
	return 0;
}

/*
 * User bit sync mode info
 */
static int mxc_spdif_usync_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 * User bit sync mode:
 * 1 CD User channel subcode
 * 0 Non-CD data
 */
static int mxc_spdif_usync_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned int int_val;

	clk_enable(plat_data->spdif_clk);

	int_val = __raw_readl(spdif_base_addr + SPDIF_REG_SRCD);
	ucontrol->value.integer.value[0] = (int_val & SRCD_CD_USER) != 0;

	clk_disable(plat_data->spdif_clk);

	return 0;
}

/*
 * User bit sync mode:
 * 1 CD User channel subcode
 * 0 Non-CD data
 */
static int mxc_spdif_usync_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	unsigned int int_val;

	clk_enable(plat_data->spdif_clk);

	int_val = ucontrol->value.integer.value[0] << SRCD_CD_USER_OFFSET;
	__raw_writel(int_val, spdif_base_addr + SPDIF_REG_SRCD);

	clk_disable(plat_data->spdif_clk);

	return 0;
}

/*
 * MXC SPDIF IEC958 controller defines
 */
static struct snd_kcontrol_new mxc_spdif_ctrls[] = {
	/* status cchanel controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_pb_spdif_info,
	 .get = mxc_pb_spdif_get,
	 .put = mxc_pb_spdif_put,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = SNDRV_CTL_NAME_IEC958("", CAPTURE, DEFAULT),
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_info,
	 .get = mxc_spdif_capture_get,
	 },
	/* user bits controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 Subcode Capture Default",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_info,
	 .get = mxc_spdif_subcode_get,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 Q-subcode Capture Default",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_qinfo,
	 .get = mxc_spdif_qget,
	 },
	/* valid bit error controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 V-Bit Errors",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_vbit_info,
	 .get = mxc_spdif_vbit_get,
	 },
	/* DPLL lock info get controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "RX Sample Rate",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_rxrate_info,
	 .get = mxc_spdif_rxrate_get,
	 },
	/* User bit sync mode set/get controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 USyncMode CDText",
	 .access =
	 SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = mxc_spdif_usync_info,
	 .get = mxc_spdif_usync_get,
	 .put = mxc_spdif_usync_put,
	 },
};

static int mxc_spdif_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;
	int ret = 0;

	/* enable spdif_xtal_clk */
	ret = clk_enable(plat_data->spdif_core_clk);
	if (ret < 0)
		goto failed_clk;

	spdif_softreset();
	/* disable all the interrupts */
	spdif_intr_enable(0xffffff, 0);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = mxc_spdif_playback_startup(substream, dai);
	else
		ret = mxc_spdif_capture_startup(substream, dai);

failed_clk:
	return ret;
}

static void mxc_spdif_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mxc_spdif_priv *spdif_priv = snd_soc_codec_get_drvdata(codec);
	struct mxc_spdif_platform_data *plat_data = spdif_priv->plat_data;

	/* disable spdif_core clock  */
	clk_disable(plat_data->spdif_core_clk);

	clk_disable(plat_data->spdif_clk);
}

static int mxc_spdif_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = mxc_spdif_playback_trigger(substream, cmd, dai);
	else
		ret = mxc_spdif_capture_trigger(substream, cmd, dai);

	return ret;
}

struct snd_soc_dai_ops mxc_spdif_codec_dai_ops = {
	.startup = mxc_spdif_startup,
	.trigger = mxc_spdif_trigger,
	.shutdown = mxc_spdif_shutdown,
};

struct snd_soc_dai_driver mxc_spdif_codec_dai = {
	.name = "mxc-spdif",
	.ops = &mxc_spdif_codec_dai_ops,
};

static int mxc_spdif_soc_probe(struct snd_soc_codec *codec)
{
	snd_soc_add_controls(codec, mxc_spdif_ctrls,
			     ARRAY_SIZE(mxc_spdif_ctrls));
	return 0;
}

static int mxc_spdif_soc_remove(struct snd_soc_codec *codec)
{
	/* all done in mxc_spdif_remove */
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_spdif = {
	.probe = mxc_spdif_soc_probe,
	.remove = mxc_spdif_soc_remove,
};

static int __devinit mxc_spdif_probe(struct platform_device *pdev)
{
	struct mxc_spdif_platform_data *plat_data =
	    (struct mxc_spdif_platform_data *)pdev->dev.platform_data;
	struct resource *res;
	struct mxc_spdif_priv *spdif_priv;
	int err, irq;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	dev_info(&pdev->dev, "MXC SPDIF Audio\n");

	spdif_priv = kzalloc(sizeof(struct mxc_spdif_priv), GFP_KERNEL);
	if (spdif_priv == NULL)
		return -ENOMEM;

	if (plat_data->spdif_tx) {
		mxc_spdif_codec_dai.playback.stream_name = "Playback";
		mxc_spdif_codec_dai.playback.channels_min = 2;
		mxc_spdif_codec_dai.playback.channels_max = 2;

		if (plat_data->spdif_clk_44100 >= 0)
			mxc_spdif_codec_dai.playback.rates |= SNDRV_PCM_RATE_44100;
		if (plat_data->spdif_clk_48000 >= 0)
			mxc_spdif_codec_dai.playback.rates |= SNDRV_PCM_RATE_32000 |
							      SNDRV_PCM_RATE_48000;

		mxc_spdif_codec_dai.playback.formats = MXC_SPDIF_FORMATS_PLAYBACK;
	}

	if (plat_data->spdif_rx) {
		mxc_spdif_codec_dai.capture.stream_name = "Capture";
		mxc_spdif_codec_dai.capture.channels_min = 2;
		mxc_spdif_codec_dai.capture.channels_max = 2;

		/* rx clock is recovered from audio stream, so it is not
		   dependent on tx clocks available */
		mxc_spdif_codec_dai.capture.rates = MXC_SPDIF_RATES_CAPTURE;

		mxc_spdif_codec_dai.capture.formats = MXC_SPDIF_FORMATS_CAPTURE;
	}

	spdif_priv->tx_active = false;
	spdif_priv->rx_active = false;
	platform_set_drvdata(pdev, spdif_priv);

	spdif_priv->reg_phys_base = res->start;
	spdif_priv->reg_base = ioremap(res->start, res->end - res->start + 1);
	spdif_priv->plat_data = plat_data;

	spdif_base_addr = (unsigned long)spdif_priv->reg_base;

	/* initial spinlock for control data */
	spin_lock_init(&mxc_spdif_control.ctl_lock);

	if (plat_data->spdif_tx) {
		/* init tx channel status default value */
		mxc_spdif_control.ch_status[0] =
		    IEC958_AES0_CON_NOT_COPYRIGHT |
		    IEC958_AES0_CON_EMPHASIS_5015;
		mxc_spdif_control.ch_status[1] = IEC958_AES1_CON_DIGDIGCONV_ID;
		mxc_spdif_control.ch_status[2] = 0x00;
		mxc_spdif_control.ch_status[3] =
		    IEC958_AES3_CON_FS_44100 | IEC958_AES3_CON_CLOCK_1000PPM;
	}

	plat_data->spdif_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(plat_data->spdif_clk)) {
		ret = PTR_ERR(plat_data->spdif_clk);
		dev_err(&pdev->dev, "can't get clock: %d\n", ret);
		goto failed_clk;
	}

	atomic_set(&spdif_priv->dpll_locked, 0);

	/* spdif interrupt register and disable */
	irq = platform_get_irq(pdev, 0);
	if ((irq <= 0) || request_irq(irq, spdif_isr, 0, "spdif", spdif_priv)) {
		pr_err("MXC spdif: failed to request irq\n");
		err = -EBUSY;
		goto card_err;
	}

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_spdif,
				     &mxc_spdif_codec_dai, 1);

	if (ret) {
		dev_err(&pdev->dev, "failed to register codec\n");
		goto card_err;
	}

	dumpregs(spdif_priv);

	return 0;

card_err:
	clk_put(plat_data->spdif_clk);
failed_clk:
	platform_set_drvdata(pdev, NULL);
	kfree(spdif_priv);

	return ret;
}

static int __devexit mxc_spdif_remove(struct platform_device *pdev)
{
	struct mxc_spdif_priv *spdif_priv = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);

	platform_set_drvdata(pdev, NULL);
	kfree(spdif_priv);

	return 0;
}

struct platform_driver mxc_spdif_driver = {
	.driver = {
		   .name = "mxc_spdif",
		   .owner = THIS_MODULE,
		   },
	.probe = mxc_spdif_probe,
	.remove = __devexit_p(mxc_spdif_remove),
};

static int __init mxc_spdif_init(void)
{
	return platform_driver_register(&mxc_spdif_driver);
}

static void __exit mxc_spdif_exit(void)
{
	return platform_driver_unregister(&mxc_spdif_driver);
}

module_init(mxc_spdif_init);
module_exit(mxc_spdif_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC SPDIF transmitter");
MODULE_LICENSE("GPL");
