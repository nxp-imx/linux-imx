/*
 * Copyright (C) 2007-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup SOUND_DRV MXC Sound Driver for ALSA
 */

/*!
 * @file       mxc-alsa-spdif.c
 * @brief      this fle       mxc-alsa-spdif.c
 * @brief      this file implements mxc alsa driver for spdif.
 *             The spdif tx supports consumer channel for linear PCM and
 *	       compressed audio data. The supported sample rates are
 *	       48KHz, 44.1KHz and 32KHz.
 *
 * @ingroup SOUND_DRV
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/soundcard.h>
#include <linux/clk.h>
#include <linux/fsl_devices.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/asoundef.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <mach/dma.h>

#define MXC_SPDIF_NAME "MXC_SPDIF"
static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE;
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for spdif sound card.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for spdif sound card.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable spdif sound card.");

#define SPDIF_MAX_BUF_SIZE      (32*1024)
#define SPDIF_DMA_BUF_SIZE	(8*1024)
#define SPDIF_MIN_PERIOD_SIZE	64
#define SPDIF_MIN_PERIOD	2
#define SPDIF_MAX_PERIOD	255

#define SPDIF_REG_SCR		0x00
#define SPDIF_REG_SRCD		0x04
#define SPDIF_REG_SRPC		0x08
#define SPDIF_REG_SIE		0x0C
#define SPDIF_REG_SIS		0x10
#define SPDIF_REG_SIC		0x10
#define SPDIF_REG_SRL		0x14
#define SPDIF_REG_SRR		0x18
#define SPDIF_REG_SRCSLH	0x1C
#define SPDIF_REG_SRCSLL	0x20
#define SPDIF_REG_SQU		0x24
#define SPDIF_REG_SRQ		0x28
#define SPDIF_REG_STL		0x2C
#define SPDIF_REG_STR		0x30
#define SPDIF_REG_STCSCH	0x34
#define SPDIF_REG_STCSCL	0x38
#define SPDIF_REG_SRFM		0x44
#define SPDIF_REG_STC		0x50

/* SPDIF Configuration register */
#define SCR_RXFIFO_CTL_ZERO	(1 << 23)
#define SCR_RXFIFO_OFF		(1 << 22)
#define SCR_RXFIFO_RST		(1 << 21)
#define SCR_RXFIFO_FSEL_BIT	(19)
#define SCR_RXFIFO_FSEL_MASK	(0x3 << SCR_RXFIFO_FSEL_BIT)
#define SCR_RXFIFO_AUTOSYNC	(1 << 18)
#define SCR_TXFIFO_AUTOSYNC	(1 << 17)
#define SCR_TXFIFO_ESEL_BIT	(15)
#define SCR_TXFIFO_ESEL_MASK	(0x3 << SCR_TXFIFO_FSEL_BIT)
#define SCR_LOW_POWER		(1 << 13)
#define SCR_SOFT_RESET		(1 << 12)
#define SCR_TXFIFO_ZERO		(0 << 10)
#define SCR_TXFIFO_NORMAL	(1 << 10)
#define SCR_TXFIFO_ONESAMPLE	(1 << 11)
#define SCR_DMA_RX_EN		(1 << 9)
#define SCR_DMA_TX_EN		(1 << 8)
#define SCR_VAL_CLEAR		(1 << 5)
#define SCR_TXSEL_OFF		(0 << 2)
#define SCR_TXSEL_RX		(1 << 2)
#define SCR_TXSEL_NORMAL	(0x5 << 2)
#define SCR_USRC_SEL_NONE	(0x0)
#define SCR_USRC_SEL_RECV	(0x1)
#define SCR_USRC_SEL_CHIP	(0x3)

/* SPDIF CDText control */
#define SRCD_CD_USER_OFFSET	1
#define SRCD_CD_USER		(1 << SRCD_CD_USER_OFFSET)

/* SPDIF Phase Configuration register */
#define SRPC_DPLL_LOCKED	(1 << 6)
#define SRPC_CLKSRC_SEL_OFFSET	7
#define SRPC_CLKSRC_SEL_LOCKED	5
/* gain sel */
#define SRPC_GAINSEL_OFFSET	3

enum spdif_gainsel {
	GAINSEL_MULTI_24 = 0,
	GAINSEL_MULTI_16,
	GAINSEL_MULTI_12,
	GAINSEL_MULTI_8,
	GAINSEL_MULTI_6,
	GAINSEL_MULTI_4,
	GAINSEL_MULTI_3,
	GAINSEL_MULTI_MAX,
};

#define SPDIF_DEFAULT_GAINSEL	GAINSEL_MULTI_8

/* SPDIF interrupt mask define */
#define INT_DPLL_LOCKED		(1 << 20)
#define INT_TXFIFO_UNOV		(1 << 19)
#define INT_TXFIFO_RESYNC	(1 << 18)
#define INT_CNEW		(1 << 17)
#define INT_VAL_NOGOOD		(1 << 16)
#define INT_SYM_ERR		(1 << 15)
#define INT_BIT_ERR		(1 << 14)
#define INT_URX_FUL		(1 << 10)
#define INT_URX_OV		(1 << 9)
#define INT_QRX_FUL		(1 << 8)
#define INT_QRX_OV		(1 << 7)
#define INT_UQ_SYNC		(1 << 6)
#define INT_UQ_ERR		(1 << 5)
#define INT_RX_UNOV		(1 << 4)
#define INT_RX_RESYNC		(1 << 3)
#define INT_LOSS_LOCK		(1 << 2)
#define INT_TX_EMPTY		(1 << 1)
#define INT_RXFIFO_FUL		(1 << 0)

/* SPDIF Clock register */
#define STC_SYSCLK_DIV_OFFSET	11
#define STC_TXCLK_SRC_OFFSET	8
#define STC_TXCLK_DIV_OFFSET	0

#define SPDIF_CSTATUS_BYTE	6
#define SPDIF_UBITS_SIZE	96
#define SPDIF_QSUB_SIZE		(SPDIF_UBITS_SIZE/8)

enum spdif_clk_accuracy {
	SPDIF_CLK_ACCURACY_LEV2 = 0,
	SPDIF_CLK_ACCURACY_LEV1 = 2,
	SPDIF_CLK_ACCURACY_LEV3 = 1,
	SPDIF_CLK_ACCURACY_RESV = 3
};

/* SPDIF clock source */
enum spdif_clk_src {
	SPDIF_CLK_SRC1 = 0,
	SPDIF_CLK_SRC2,
	SPDIF_CLK_SRC3,
	SPDIF_CLK_SRC4,
	SPDIF_CLK_SRC5,
};

enum spdif_max_wdl {
	SPDIF_MAX_WDL_20,
	SPDIF_MAX_WDL_24
};

enum spdif_wdl {
	SPDIF_WDL_DEFAULT = 0,
	SPDIF_WDL_FIFTH = 4,
	SPDIF_WDL_FOURTH = 3,
	SPDIF_WDL_THIRD = 2,
	SPDIF_WDL_SECOND = 1,
	SPDIF_WDL_MAX = 5
};

static unsigned int gainsel_multi[GAINSEL_MULTI_MAX] = {
	24 * 1024, 16 * 1024, 12 * 1024,
	8 * 1024, 6 * 1024, 4 * 1024,
	3 * 1024,
};

/*!
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

/*!
 * This structure represents an audio stream in term of
 * channel DMA, HW configuration on spdif controller.
 */
struct mxc_spdif_stream {

	/*!
	 * identification string
	 */
	char *id;
	/*!
	 * device identifier for DMA
	 */
	int dma_wchannel;
	/*!
	 * we are using this stream for transfer now
	 */
	int active:1;
	/*!
	 * current transfer period
	 */
	int period;
	/*!
	 * current count of transfered periods
	 */
	int periods;
	/*!
	 * for locking in DMA operations
	 */
	spinlock_t dma_lock;
	/*!
	 * Alsa substream pointer
	 */
	struct snd_pcm_substream *stream;
};

struct mxc_spdif_device {
	/*!
	 * SPDIF module register base address
	 */
	unsigned long __iomem *reg_base;
	unsigned long reg_phys_base;

	/*!
	 * spdif tx available or not
	 */
	int mxc_spdif_tx;

	/*!
	 * spdif rx available or not
	 */
	int mxc_spdif_rx;

	/*!
	 * spdif 44100 clock src
	 */
	int spdif_txclk_44100;

	/*!
	 * spdif 48000 clock src
	 */
	int spdif_txclk_48000;

	/*!
	 * ALSA SPDIF sound card handle
	 */
	struct snd_card *card;

	/*!
	 * ALSA spdif driver type handle
	 */
	struct snd_pcm *pcm;

	/*!
	 * DPLL locked status
	 */
	atomic_t dpll_locked;

	/*!
	 * Playback/Capture substream
	 */
	struct mxc_spdif_stream s[2];
};

static struct spdif_mixer_control mxc_spdif_control;

static unsigned long spdif_base_addr;

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

/*!
 * @brief Enable/Disable spdif DMA request
 *
 * This function is called to enable or disable the dma transfer
 */
static void spdif_dma_enable(int txrx, int enable)
{
	unsigned long value;

	value = __raw_readl(SPDIF_REG_SCR + spdif_base_addr) & 0xfffeff;
	if (enable)
		value |= txrx;

	__raw_writel(value, SPDIF_REG_SCR + spdif_base_addr);

}

/*!
 * @brief Enable spdif interrupt
 *
 * This function is called to enable relevant interrupts.
 */
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

/*!
 * @brief Set the clock accuracy level in the channel bit
 *
 * This function is called to set the clock accuracy level
 */
static int spdif_set_clk_accuracy(enum spdif_clk_accuracy level)
{
	unsigned long value;

	value = __raw_readl(SPDIF_REG_STCSCL + spdif_base_addr) & 0xffffcf;
	value |= (level << 4);
	__raw_writel(value, SPDIF_REG_STCSCL + spdif_base_addr);

	return 0;
}

/*!
 * set SPDIF PhaseConfig register for rx clock
 */
static int spdif_set_rx_clksrc(enum spdif_clk_src clksrc,
			       enum spdif_gainsel gainsel, int dpll_locked)
{
	unsigned long value;
	if (clksrc > SPDIF_CLK_SRC5 || gainsel > GAINSEL_MULTI_3)
		return 1;

	value = (dpll_locked ? (clksrc) :
		 (clksrc + SRPC_CLKSRC_SEL_LOCKED)) << SRPC_CLKSRC_SEL_OFFSET |
	    (gainsel << SRPC_GAINSEL_OFFSET);
	__raw_writel(value, spdif_base_addr + SPDIF_REG_SRPC);

	return 0;
}

/*!
 * Get RX data clock rate
 * given the SPDIF bus_clk
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
	return (int)tmpval64;
}

/*!
 * @brief Set the audio sample rate in the channel status bit
 *
 * This function is called to set the audio sample rate to be transfered.
 */
static int spdif_set_sample_rate(int src_44100, int src_48000, int sample_rate)
{
	unsigned long cstatus, stc;
	int ret = 0;

	cstatus = __raw_readl(SPDIF_REG_STCSCL + spdif_base_addr) & 0xfffff0;
	stc = __raw_readl(SPDIF_REG_STC + spdif_base_addr) & ~0x7FF;

	switch (sample_rate) {
	case 44100:
		if (src_44100 < 0) {
			pr_info("spdif_set_sample_rate: no defined 44100 clk src\n");
			ret = -1;
		} else {
			__raw_writel(cstatus, SPDIF_REG_STCSCL + spdif_base_addr);
			stc |= (src_44100 << 8) | 0x07;
			__raw_writel(stc, SPDIF_REG_STC + spdif_base_addr);
			pr_debug("set sample rate to 44100\n");
		}
		break;
	case 48000:
		if (src_48000 < 0) {
			pr_info("spdif_set_sample_rate: no defined 48000 clk src\n");
			ret = -1;
		} else {
			cstatus |= 0x04;
			__raw_writel(cstatus, SPDIF_REG_STCSCL + spdif_base_addr);
			stc |= (src_48000 << 8) | 0x07;
			__raw_writel(stc, SPDIF_REG_STC + spdif_base_addr);
			pr_debug("set sample rate to 48000\n");
		}
		break;
	case 32000:
		if (src_48000 < 0) {
			pr_info("spdif_set_sample_rate: no defined 48000 clk src\n");
			ret = -1;
		} else {
			cstatus |= 0x0c;
			__raw_writel(cstatus, SPDIF_REG_STCSCL + spdif_base_addr);
			stc |= (src_48000 << 8) | 0x0b;
			__raw_writel(stc, SPDIF_REG_STC + spdif_base_addr);
			pr_debug("set sample rate to 32000\n");
		}
		break;
	}

	return ret;
}

/*!
 * @brief Set the lchannel status bit
 *
 * This function is called to set the channel status
 */
static int spdif_set_channel_status(int value, unsigned long reg)
{
	__raw_writel(value & 0xffffff, reg + spdif_base_addr);

	return 0;
}

/*!
 * @brief Get spdif interrupt status and clear the interrupt
 *
 * This function is called to check relevant interrupt status
 */
static int spdif_intr_status(void)
{
	unsigned long value;

	value = __raw_readl(SPDIF_REG_SIS + spdif_base_addr) & 0xffffff;
	value &= __raw_readl(spdif_base_addr + SPDIF_REG_SIE);
	__raw_writel(value, SPDIF_REG_SIC + spdif_base_addr);

	return value;
}

/*!
 * @brief spdif interrupt handler
 */
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

/*!
 * Interrupt handlers
 *
 */
/*!
 * FIFO related interrupts handler
 *
 * Rx FIFO Full, Underrun/Overrun interrupts
 * Tx FIFO Empty, Underrun/Overrun interrupts
 */
static void spdif_irq_fifo(unsigned int bit, void *devid)
{

}

/*!
 * DPLL lock related interrupts handler
 *
 * DPLL locked and lock loss interrupts
 */
static void spdif_irq_dpll_lock(unsigned int bit, void *devid)
{
	struct mxc_spdif_device *chip = (struct mxc_spdif_device *)devid;
	unsigned int locked = __raw_readl(spdif_base_addr + SPDIF_REG_SRPC);

	if (locked & SRPC_DPLL_LOCKED) {
		pr_debug("SPDIF Rx dpll locked\n");
		atomic_set(&chip->dpll_locked, 1);
	} else {
		/* INT_LOSS_LOCK */
		pr_debug("SPDIF Rx dpll loss lock\n");
		atomic_set(&chip->dpll_locked, 0);
	}
}

/*!
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
			pr_err("Q bit receivce buffer overflow\n");
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
		ctrl->ready_buf = ctrl->upos = ctrl->qpos = 0;
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

/*!
 * SPDIF receiver found parity bit error interrupt handler
 */
static void spdif_irq_bit_error(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt parity bit error\n");
}

/*!
 * SPDIF receiver found illegal symbol interrupt handler
 */
static void spdif_irq_sym_error(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt symbol error\n");
}

/*!
 * SPDIF validity flag no good interrupt handler
 */
static void spdif_irq_valnogood(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt validate is not good\n");
}

/*!
 * SPDIF receive change in value of control channel
 */
static void spdif_irq_cnew(unsigned int bit, void *devid)
{
	pr_debug("SPDIF interrupt cstatus new\n");
}

/*!
 * Do software reset to SPDIF
 */
static void spdif_softreset(void)
{
	unsigned long value = 1;
	int cycle = 0;
	__raw_writel(SCR_SOFT_RESET, spdif_base_addr + SPDIF_REG_SCR);
	while (value && (cycle++ < 10)) {
		value = __raw_readl(spdif_base_addr + SPDIF_REG_SCR) & 0x1000;
	}

}

/*!
 * SPDIF RX initial function
 */
static void spdif_rx_init(void)
{
	unsigned long regval;

	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);
	/**
	 * initial and reset SPDIF configuration:
	 * RxFIFO off
	 * RxFIFO sel to 8 sample
	 * Autosync
	 * Valid bit set
	 */
	regval &= ~(SCR_RXFIFO_OFF | SCR_RXFIFO_CTL_ZERO | SCR_LOW_POWER);
	regval |= (2 << SCR_RXFIFO_FSEL_BIT) | SCR_RXFIFO_AUTOSYNC;
	__raw_writel(regval, spdif_base_addr + SPDIF_REG_SCR);
}

/*!
 * SPDIF RX un-initial function
 */
static void spdif_rx_uninit(void)
{
	unsigned long regval;

	/* turn off RX fifo, disable dma and autosync */
	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);
	regval |= SCR_RXFIFO_OFF | SCR_RXFIFO_CTL_ZERO;
	regval &= ~(SCR_DMA_RX_EN | SCR_RXFIFO_AUTOSYNC);
	__raw_writel(regval, spdif_base_addr + SPDIF_REG_SCR);
}

/*!
 * @brief Initialize spdif module
 *
 * This function is called to set the spdif to initial state.
 */
static void spdif_tx_init(void)
{
	unsigned long regval;

	regval = __raw_readl(spdif_base_addr + SPDIF_REG_SCR);

	regval &= 0xfc32e3;
	regval |= SCR_TXFIFO_AUTOSYNC | SCR_TXFIFO_NORMAL |
	    SCR_TXSEL_NORMAL | SCR_USRC_SEL_CHIP | (2 << SCR_TXFIFO_ESEL_BIT);
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);

	/* Default clock source from EXTAL, divider by 8, generate 44.1kHz
	   sample rate */
	regval = 0x07;
	__raw_writel(regval, SPDIF_REG_STC + spdif_base_addr);

}

/*!
 * @brief deinitialize spdif module
 *
 * This function is called to stop the spdif
 */
static void spdif_tx_uninit(void)
{
	unsigned long regval;

	regval = __raw_readl(SPDIF_REG_SCR + spdif_base_addr) & 0xffffe3;
	regval |= SCR_TXSEL_OFF;
	__raw_writel(regval, SPDIF_REG_SCR + spdif_base_addr);
	regval = __raw_readl(SPDIF_REG_STC + spdif_base_addr) & ~0x7FF;
	regval |= (0x7 << STC_TXCLK_SRC_OFFSET);
	__raw_writel(regval, SPDIF_REG_STC + spdif_base_addr);

}

static unsigned int spdif_playback_rates[] = { 32000, 44100, 48000 };
static unsigned int spdif_capture_rates[] = {
	16000, 32000, 44100, 48000, 64000, 96000
};

/*!
  * this structure represents the sample rates supported
  * by SPDIF
  */
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

/*!
 * This structure reprensents the capabilities of the driver
 * in playback mode.
 */
static struct snd_pcm_hardware snd_spdif_playback_hw = {
	.info =
	    (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
	     SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_HALF_DUPLEX |
	     SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_PAUSE |
	     SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
	    SNDRV_PCM_FMTBIT_S24_LE,
	.rates =
	    SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
	.rate_min = 32000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = SPDIF_MAX_BUF_SIZE,
	.period_bytes_min = SPDIF_MIN_PERIOD_SIZE,
	.period_bytes_max = SPDIF_DMA_BUF_SIZE,
	.periods_min = SPDIF_MIN_PERIOD,
	.periods_max = SPDIF_MAX_PERIOD,
	.fifo_size = 0,
};

/*!
 * This structure reprensents the capabilities of the driver
 * in capture mode.
 */
static struct snd_pcm_hardware snd_spdif_capture_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rates =
	    (SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100
	     | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_64000 |
	     SNDRV_PCM_RATE_96000),
	.rate_min = 16000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = SPDIF_MAX_BUF_SIZE,
	.period_bytes_min = SPDIF_MIN_PERIOD_SIZE,
	.period_bytes_max = SPDIF_DMA_BUF_SIZE,
	.periods_min = SPDIF_MIN_PERIOD,
	.periods_max = SPDIF_MAX_PERIOD,
	.fifo_size = 0,

};

/*!
  * This function configures the DMA channel used to transfer
  * audio from MCU to SPDIF or from SPDIF to MCU
  *
  * @param	s	pointer to the structure of the current stream.
  * @param      callback        pointer to function that will be
  *                              called when a SDMA TX transfer finishes.
  *
  * @return              0 on success, -1 otherwise.
  */
static int
spdif_configure_dma_channel(struct mxc_spdif_stream *s,
			    mxc_dma_callback_t callback)
{
	int ret = -1;
	int channel = -1;

	if (s->dma_wchannel != 0)
		mxc_dma_free(s->dma_wchannel);

	if (s->stream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		if (s->stream->runtime->sample_bits > 16) {
			channel =
			    mxc_dma_request(MXC_DMA_SPDIF_32BIT_TX,
					    "SPDIF TX DMA");
		} else {
			channel =
			    mxc_dma_request(MXC_DMA_SPDIF_16BIT_TX,
					    "SPDIF TX DMA");
		}

	} else if (s->stream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		channel = mxc_dma_request(MXC_DMA_SPDIF_32BIT_RX,
					  "SPDIF RX DMA");

	}

	pr_debug("spdif_configure_dma_channel: %d\n", channel);

	ret = mxc_dma_callback_set(channel,
				   (mxc_dma_callback_t) callback, (void *)s);
	if (ret != 0) {
		pr_info("spdif_configure_dma_channel - err\n");
		mxc_dma_free(channel);
		return -1;
	}
	s->dma_wchannel = channel;
	return 0;
}

/*!
  * This function gets the dma pointer position during playback/capture.
  * Our DMA implementation does not allow to retrieve this position
  * when a transfert is active, so, it answers the middle of
  * the current period beeing transfered
  *
  * @param	s	pointer to the structure of the current stream.
  *
  */
static u_int spdif_get_dma_pos(struct mxc_spdif_stream *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int offset = 0;
	substream = s->stream;
	runtime = substream->runtime;

	offset = (runtime->period_size * (s->periods));
	if (offset >= runtime->buffer_size)
		offset = 0;
	pr_debug
	    ("MXC: spdif_get_dma_pos BIS offset  %d, buffer_size %d\n",
	     offset, (int)runtime->buffer_size);
	return offset;
}

/*!
  * This function stops the current dma transfert for playback
  * and clears the dma pointers.
  *
  * @param	s	pointer to the structure of the current stream.
  *
  */
static void spdif_stop_tx(struct mxc_spdif_stream *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;

	substream = s->stream;
	runtime = substream->runtime;
	dma_size = frames_to_bytes(runtime, runtime->period_size);
	offset = dma_size * s->periods;

	s->active = 0;
	s->period = 0;
	s->periods = 0;

	/* this stops the dma channel and clears the buffer ptrs */
	mxc_dma_disable(s->dma_wchannel);
	spdif_dma_enable(SCR_DMA_TX_EN, 0);
	dma_unmap_single(NULL, runtime->dma_addr + offset, dma_size,
			 DMA_TO_DEVICE);
}

/*!
  * This function is called whenever a new audio block needs to be
  * transferred to SPDIF. The function receives the address and the size
  * of the new block and start a new DMA transfer.
  *
  * @param	s	pointer to the structure of the current stream.
  *
  */
static void spdif_start_tx(struct mxc_spdif_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_spdif_device *chip = snd_pcm_substream_chip(substream);
	unsigned int dma_size = 0;
	unsigned int offset;
	int ret = 0;
	mxc_dma_requestbuf_t dma_request;
	substream = s->stream;
	runtime = substream->runtime;
	memset(&dma_request, 0, sizeof(mxc_dma_requestbuf_t));
	if (s->active) {
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		offset = dma_size * s->period;
		dma_request.src_addr =
		    (dma_addr_t) (dma_map_single
				  (NULL, runtime->dma_area + offset, dma_size,
				   DMA_TO_DEVICE));

		dma_request.dst_addr = (dma_addr_t) (chip->reg_phys_base + 0x2c);

		dma_request.num_of_bytes = dma_size;
		mxc_dma_config(s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable(s->dma_wchannel);
		spdif_dma_enable(SCR_DMA_TX_EN, 1);
		if (ret) {
			pr_info("audio_process_dma: cannot queue DMA \
				buffer\n");
			return;
		}
		s->period++;
		s->period %= runtime->periods;

		if ((s->period > s->periods)
		    && ((s->period - s->periods) > 1)) {
			pr_debug("audio playback chain dma: already double \
				buffered\n");
			return;
		}

		if ((s->period < s->periods)
		    && ((s->period + runtime->periods - s->periods) > 1)) {
			pr_debug("audio playback chain dma: already double  \
				buffered\n");
			return;
		}

		if (s->period == s->periods) {
			pr_debug("audio playback chain dma: s->period == \
				s->periods\n");
			return;
		}

		if (snd_pcm_playback_hw_avail(runtime) <
		    2 * runtime->period_size) {
			pr_debug("audio playback chain dma: available data \
				is not enough\n");
			return;
		}

		pr_debug
		    ("audio playback chain dma:to set up the 2nd dma buffer\n");
		pr_debug("SCR: 0x%08x\n",
			 __raw_readl(spdif_base_addr + SPDIF_REG_SCR));

		offset = dma_size * s->period;
		dma_request.src_addr =
		    (dma_addr_t) (dma_map_single
				  (NULL, runtime->dma_area + offset, dma_size,
				   DMA_TO_DEVICE));
		mxc_dma_config(s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_WRITE);
		ret = mxc_dma_enable(s->dma_wchannel);
		s->period++;
		s->period %= runtime->periods;

	}
	return;
}

/*!
  * This is a callback which will be called
  * when a TX transfer finishes. The call occurs
  * in interrupt context.
  *
  * @param	data	pointer to the structure of the current stream
  * @param	error	DMA error flag
  * @param	count	number of bytes transfered by the DMA
  */
static void spdif_tx_callback(void *data, int error, unsigned int count)
{
	struct mxc_spdif_stream *s;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int previous_period;
	unsigned int offset;
	s = data;
	substream = s->stream;
	runtime = substream->runtime;
	previous_period = s->periods;
	dma_size = frames_to_bytes(runtime, runtime->period_size);
	offset = dma_size * previous_period;

	spin_lock(&s->dma_lock);
	s->periods++;
	s->periods %= runtime->periods;
	dma_unmap_single(NULL, runtime->dma_addr + offset, dma_size,
			 DMA_TO_DEVICE);
	spin_unlock(&s->dma_lock);

	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	spdif_start_tx(s);
	spin_unlock(&s->dma_lock);
}

/*!
  * This function is a dispatcher of command to be executed
  * by the driver for playback.
  *
  * @param	substream	pointer to the structure of the current stream.
  * @param	cmd		command to be executed
  *
  * @return              0 on success, -1 otherwise.
  */
static int
snd_mxc_spdif_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mxc_spdif_device *chip;
	struct mxc_spdif_stream *s;
	int err = 0;
	unsigned long flags;
	chip = snd_pcm_substream_chip(substream);
	s = &chip->s[SNDRV_PCM_STREAM_PLAYBACK];

	spin_lock_irqsave(&s->dma_lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		s->active = 1;
		spdif_start_tx(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spdif_stop_tx(s);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		s->active = 0;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		s->active = 1;
		spdif_start_tx(s);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		s->active = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		s->active = 1;
		spdif_start_tx(s);
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&s->dma_lock, flags);
	return err;
}

/*!
  * This function configures the hardware to allow audio
  * playback operations. It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_spdif_playback_prepare(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	struct snd_pcm_runtime *runtime;
	int err;
	unsigned int ch_status;

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;

	spdif_tx_init();

	ch_status =
	    ((mxc_spdif_control.ch_status[2] << 16) | (mxc_spdif_control.
						       ch_status[1] << 8) |
	     mxc_spdif_control.ch_status[0]);
	spdif_set_channel_status(ch_status, SPDIF_REG_STCSCH);
	ch_status = mxc_spdif_control.ch_status[3];
	spdif_set_channel_status(ch_status, SPDIF_REG_STCSCL);
	spdif_intr_enable(INT_TXFIFO_RESYNC, 1);
	err = spdif_set_sample_rate(chip->spdif_txclk_44100, chip->spdif_txclk_48000,
			      runtime->rate);
	if (err < 0) {
		pr_info("snd_mxc_spdif_playback_prepare - err < 0\n");
		return err;
	}
	spdif_set_clk_accuracy(SPDIF_CLK_ACCURACY_LEV2);
	/* setup DMA controller for spdif tx */
	err = spdif_configure_dma_channel(&chip->
					  s[SNDRV_PCM_STREAM_PLAYBACK],
					  spdif_tx_callback);
	if (err < 0) {
		pr_info("snd_mxc_spdif_playback_prepare - err < 0\n");
		return err;
	}

	/**
	 * FIXME: dump registers
	 */
	pr_debug("SCR: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SCR));
	pr_debug("SIE: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIE));
	pr_debug("STC: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_STC));
	return 0;
}

/*!
  * This function gets the current playback pointer position.
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  */
static snd_pcm_uframes_t
snd_mxc_spdif_playback_pointer(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	chip = snd_pcm_substream_chip(substream);
	return spdif_get_dma_pos(&chip->s[SNDRV_PCM_STREAM_PLAYBACK]);
}

static int snd_card_mxc_spdif_playback_open(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	struct snd_pcm_runtime *runtime;
	int err;
	struct mxc_spdif_platform_data *spdif_data;

	chip = snd_pcm_substream_chip(substream);

	spdif_data = chip->card->dev->platform_data;
	/* enable tx clock */
	clk_enable(spdif_data->spdif_clk);
	clk_enable(spdif_data->spdif_audio_clk);

	runtime = substream->runtime;
	chip->s[SNDRV_PCM_STREAM_PLAYBACK].stream = substream;
	runtime->hw = snd_spdif_playback_hw;
	err = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &hw_playback_rates_stereo);
	if (err < 0)
		goto failed;
	err =
	    snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		goto failed;

	return 0;
      failed:
	clk_disable(spdif_data->spdif_clk);
	return err;
}

/*!
  * This function closes an spdif device for playback.
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_card_mxc_spdif_playback_close(struct snd_pcm_substream
					     *substream)
{
	struct mxc_spdif_device *chip;
	struct mxc_spdif_platform_data *spdif_data;

	chip = snd_pcm_substream_chip(substream);
	spdif_data = chip->card->dev->platform_data;

	pr_debug("SIS: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIS));

	spdif_intr_status();
	spdif_intr_enable(INT_TXFIFO_RESYNC, 0);
	spdif_tx_uninit();
	clk_disable(spdif_data->spdif_audio_clk);
	clk_disable(spdif_data->spdif_clk);
	mxc_dma_free(chip->s[SNDRV_PCM_STREAM_PLAYBACK].dma_wchannel);
	chip->s[SNDRV_PCM_STREAM_PLAYBACK].dma_wchannel = 0;

	return 0;
}

/*! TODO: update the dma start/stop callback routine
  * This function stops the current dma transfert for capture
  * and clears the dma pointers.
  *
  * @param	s	pointer to the structure of the current stream.
  *
  */
static void spdif_stop_rx(struct mxc_spdif_stream *s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;

	substream = s->stream;
	runtime = substream->runtime;
	dma_size = frames_to_bytes(runtime, runtime->period_size);
	offset = dma_size * s->periods;

	s->active = 0;
	s->period = 0;
	s->periods = 0;

	/* this stops the dma channel and clears the buffer ptrs */
	mxc_dma_disable(s->dma_wchannel);
	spdif_dma_enable(SCR_DMA_RX_EN, 0);
	dma_unmap_single(NULL, runtime->dma_addr + offset, dma_size,
			 DMA_FROM_DEVICE);
}

/*!
  * This function is called whenever a new audio block needs to be
  * received from SPDIF. The function receives the address and the size
  * of the new block and start a new DMA transfer.
  *
  * @param	s	pointer to the structure of the current stream.
  *
  */
static void spdif_start_rx(struct mxc_spdif_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mxc_spdif_device *chip = snd_pcm_substream_chip(substream);
	unsigned int dma_size = 0;
	unsigned int offset;
	int ret = 0;
	mxc_dma_requestbuf_t dma_request;

	memset(&dma_request, 0, sizeof(mxc_dma_requestbuf_t));

	if (s->active) {
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		pr_debug("s->period (%x) runtime->periods (%d)\n",
			 s->period, runtime->periods);
		pr_debug("runtime->period_size (%d) dma_size (%d)\n",
			 (unsigned int)runtime->period_size,
			 runtime->dma_bytes);

		offset = dma_size * s->period;
		dma_request.dst_addr =
		    (dma_addr_t) (dma_map_single
				  (NULL, runtime->dma_area + offset, dma_size,
				   DMA_FROM_DEVICE));

		dma_request.src_addr =
		    (dma_addr_t) (chip->reg_phys_base + SPDIF_REG_SRL);
		dma_request.num_of_bytes = dma_size;
		/* config and enable sdma for RX */
		mxc_dma_config(s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_READ);
		ret = mxc_dma_enable(s->dma_wchannel);
		/* enable SPDIF dma */
		spdif_dma_enable(SCR_DMA_RX_EN, 1);

		if (ret) {
			pr_info("audio_process_dma: cannot queue DMA \
				buffer\n");
			return;
		}
		s->period++;
		s->period %= runtime->periods;

		if ((s->period > s->periods)
		    && ((s->period - s->periods) > 1)) {
			pr_debug("audio capture chain dma: already double \
				buffered\n");
			return;
		}

		if ((s->period < s->periods)
		    && ((s->period + runtime->periods - s->periods) > 1)) {
			pr_debug("audio capture chain dma: already double  \
				buffered\n");
			return;
		}

		if (s->period == s->periods) {
			pr_debug("audio capture chain dma: s->period == \
				s->periods\n");
			return;
		}

		if (snd_pcm_capture_hw_avail(runtime) <
		    2 * runtime->period_size) {
			pr_debug("audio capture chain dma: available data \
				is not enough\n");
			return;
		}

		pr_debug
		    ("audio playback chain dma:to set up the 2nd dma buffer\n");

		offset = dma_size * s->period;
		dma_request.dst_addr =
		    (dma_addr_t) (dma_map_single
				  (NULL, runtime->dma_area + offset, dma_size,
				   DMA_FROM_DEVICE));
		mxc_dma_config(s->dma_wchannel, &dma_request, 1,
			       MXC_DMA_MODE_READ);
		ret = mxc_dma_enable(s->dma_wchannel);
		s->period++;
		s->period %= runtime->periods;

	}
	return;
}

/*!
  * This is a callback which will be called
  * when a RX transfer finishes. The call occurs
  * in interrupt context.
  *
  * @param	data	pointer to the structure of the current stream
  * @param	error	DMA error flag
  * @param	count	number of bytes transfered by the DMA
  */
static void spdif_rx_callback(void *data, int error, unsigned int count)
{
	struct mxc_spdif_stream *s;
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int previous_period;
	unsigned int offset;

	s = data;
	substream = s->stream;
	runtime = substream->runtime;
	previous_period = s->periods;
	dma_size = frames_to_bytes(runtime, runtime->period_size);
	offset = dma_size * previous_period;

	spin_lock(&s->dma_lock);
	s->periods++;
	s->periods %= runtime->periods;

	dma_unmap_single(NULL, runtime->dma_addr + offset, dma_size,
			 DMA_FROM_DEVICE);
	spin_unlock(&s->dma_lock);

	if (s->active)
		snd_pcm_period_elapsed(s->stream);
	spin_lock(&s->dma_lock);
	spdif_start_rx(s);
	spin_unlock(&s->dma_lock);
}

/*!
  * This function is a dispatcher of command to be executed
  * by the driver for capture.
  *
  * @param	substream	pointer to the structure of the current stream.
  * @param	cmd		command to be executed
  *
  * @return              0 on success, -1 otherwise.
  */
static int
snd_mxc_spdif_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mxc_spdif_device *chip;
	struct mxc_spdif_stream *s;
	int err = 0;
	unsigned long flags;
	chip = snd_pcm_substream_chip(substream);
	s = &chip->s[SNDRV_PCM_STREAM_CAPTURE];

	spin_lock_irqsave(&s->dma_lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		s->active = 1;
		spdif_start_rx(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spdif_stop_rx(s);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		s->active = 0;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		s->active = 1;
		spdif_start_rx(s);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		s->active = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		s->active = 1;
		spdif_start_rx(s);
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&s->dma_lock, flags);
	return err;
}

/*!
  * This function configures the hardware to allow audio
  * capture operations. It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_spdif_capture_prepare(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	struct mxc_spdif_platform_data *spdif_data;
	struct snd_pcm_runtime *runtime;
	int err;

	chip = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	spdif_data = chip->card->dev->platform_data;

	spdif_rx_init();
	/* enable interrupts, include DPLL lock */
	spdif_intr_enable(INT_SYM_ERR | INT_BIT_ERR | INT_URX_FUL |
			  INT_URX_OV | INT_QRX_FUL | INT_QRX_OV |
			  INT_UQ_SYNC | INT_UQ_ERR | INT_RX_RESYNC |
			  INT_LOSS_LOCK, 1);

	/* setup rx clock source */
	spdif_set_rx_clksrc(spdif_data->spdif_clkid, SPDIF_DEFAULT_GAINSEL, 1);

	/* setup DMA controller for spdif rx */
	err = spdif_configure_dma_channel(&chip->
					  s[SNDRV_PCM_STREAM_CAPTURE],
					  spdif_rx_callback);
	if (err < 0) {
		pr_info("snd_mxc_spdif_playback_prepare - err < 0\n");
		return err;
	}

	/* Debug: dump registers */
	pr_debug("SCR: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SCR));
	pr_debug("SIE: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIE));
	pr_debug("SRPC: 0x%08x\n",
		 __raw_readl(spdif_base_addr + SPDIF_REG_SRPC));
	pr_debug("FreqMeas: 0x%08x\n",
		 __raw_readl(spdif_base_addr + SPDIF_REG_SRFM));

	return 0;
}

/*!
  * This function gets the current capture pointer position.
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  */
static snd_pcm_uframes_t
snd_mxc_spdif_capture_pointer(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	chip = snd_pcm_substream_chip(substream);
	return spdif_get_dma_pos(&chip->s[SNDRV_PCM_STREAM_CAPTURE]);
}

/*!
  * This function opens a spdif device in capture mode
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_card_mxc_spdif_capture_open(struct snd_pcm_substream *substream)
{
	struct mxc_spdif_device *chip;
	struct snd_pcm_runtime *runtime;
	int err = 0;
	struct mxc_spdif_platform_data *spdif_data;

	chip = snd_pcm_substream_chip(substream);

	spdif_data = chip->card->dev->platform_data;
	/* enable rx bus clock */
	clk_enable(spdif_data->spdif_clk);

	runtime = substream->runtime;
	chip->s[SNDRV_PCM_STREAM_CAPTURE].stream = substream;
	runtime->hw = snd_spdif_capture_hw;

	/* set hw param constraints */
	err = snd_pcm_hw_constraint_list(runtime, 0,
					 SNDRV_PCM_HW_PARAM_RATE,
					 &hw_capture_rates_stereo);
	if (err < 0)
		goto failed;
	err =
	    snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		goto failed;

	/* enable spdif dpll lock interrupt */
	spdif_intr_enable(INT_DPLL_LOCKED, 1);

	return 0;

      failed:
	clk_disable(spdif_data->spdif_clk);
	return err;
}

/*!
  * This function closes an spdif device for capture.
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_card_mxc_spdif_capture_close(struct snd_pcm_substream
					    *substream)
{
	struct mxc_spdif_device *chip;
	struct mxc_spdif_platform_data *spdif_data;

	chip = snd_pcm_substream_chip(substream);
	spdif_data = chip->card->dev->platform_data;

	pr_debug("SIS: 0x%08x\n", __raw_readl(spdif_base_addr + SPDIF_REG_SIS));
	pr_debug("SRPC: 0x%08x\n",
		 __raw_readl(spdif_base_addr + SPDIF_REG_SRPC));
	pr_debug("FreqMeas: 0x%08x\n",
		 __raw_readl(spdif_base_addr + SPDIF_REG_SRFM));

	spdif_intr_enable(INT_DPLL_LOCKED | INT_SYM_ERR | INT_BIT_ERR |
			  INT_URX_FUL | INT_URX_OV | INT_QRX_FUL | INT_QRX_OV |
			  INT_UQ_SYNC | INT_UQ_ERR | INT_RX_RESYNC |
			  INT_LOSS_LOCK, 0);
	spdif_rx_uninit();
	clk_disable(spdif_data->spdif_clk);
	mxc_dma_free(chip->s[SNDRV_PCM_STREAM_CAPTURE].dma_wchannel);
	chip->s[SNDRV_PCM_STREAM_CAPTURE].dma_wchannel = 0;
	return 0;
}

/*!
  * This function configure the Audio HW in terms of memory allocation.
  * It is called by ALSA framework.
  *
  * @param	substream	pointer to the structure of the current stream.
  * @param	hw_params	Pointer to hardware paramters structure
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_spdif_hw_params(struct snd_pcm_substream
				   *substream, struct snd_pcm_hw_params
				   *hw_params)
{
	struct snd_pcm_runtime *runtime;
	int ret = 0;
	runtime = substream->runtime;
	ret =
	    snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
	if (ret < 0) {
		pr_info("snd_mxc_spdif_hw_params - ret: %d\n", ret);
		return ret;
	}
	runtime->dma_addr = virt_to_phys(runtime->dma_area);
	return ret;
}

/*!
  * This function frees the spdif hardware at the end of playback.
  *
  * @param	substream	pointer to the structure of the current stream.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_mxc_spdif_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/*!
  * This structure is the list of operation that the driver
  * must provide for the playback interface
  */
static struct snd_pcm_ops snd_card_mxc_spdif_playback_ops = {
	.open = snd_card_mxc_spdif_playback_open,
	.close = snd_card_mxc_spdif_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_mxc_spdif_hw_params,
	.hw_free = snd_mxc_spdif_hw_free,
	.prepare = snd_mxc_spdif_playback_prepare,
	.trigger = snd_mxc_spdif_playback_trigger,
	.pointer = snd_mxc_spdif_playback_pointer,
};

static struct snd_pcm_ops snd_card_mxc_spdif_capture_ops = {
	.open = snd_card_mxc_spdif_capture_open,
	.close = snd_card_mxc_spdif_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_mxc_spdif_hw_params,
	.hw_free = snd_mxc_spdif_hw_free,
	.prepare = snd_mxc_spdif_capture_prepare,
	.trigger = snd_mxc_spdif_capture_trigger,
	.pointer = snd_mxc_spdif_capture_pointer,
};

/*!
  * This functions initializes the playback audio device supported by
  * spdif
  *
  * @param	mxc_spdif	pointer to the sound card structure.
  *
  */
void mxc_init_spdif_device(struct mxc_spdif_device *mxc_spdif)
{

	/* initial spinlock for control data */
	spin_lock_init(&mxc_spdif_control.ctl_lock);

	if (mxc_spdif->mxc_spdif_tx) {

		mxc_spdif->s[SNDRV_PCM_STREAM_PLAYBACK].id = "spdif tx";
		/* init tx channel status default value */
		mxc_spdif_control.ch_status[0] =
		    IEC958_AES0_CON_NOT_COPYRIGHT |
		    IEC958_AES0_CON_EMPHASIS_5015;
		mxc_spdif_control.ch_status[1] = IEC958_AES1_CON_DIGDIGCONV_ID;
		mxc_spdif_control.ch_status[2] = 0x00;
		mxc_spdif_control.ch_status[3] =
		    IEC958_AES3_CON_FS_44100 | IEC958_AES3_CON_CLOCK_1000PPM;
	}
	if (mxc_spdif->mxc_spdif_rx) {

		/* TODO: Add code here if capture is available */
		mxc_spdif->s[SNDRV_PCM_STREAM_CAPTURE].id = "spdif rx";
	}

}

/*!
 * MXC SPDIF IEC958 controller(mixer) functions
 *
 * 	Channel status get/put control
 * 	User bit value get/put control
 * 	Valid bit value get control
 * 	DPLL lock status get control
 * 	User bit sync mode selection control
 *
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
	unsigned int ch_status;
	mxc_spdif_control.ch_status[0] = uvalue->value.iec958.status[0];
	mxc_spdif_control.ch_status[1] = uvalue->value.iec958.status[1];
	mxc_spdif_control.ch_status[2] = uvalue->value.iec958.status[2];
	mxc_spdif_control.ch_status[3] = uvalue->value.iec958.status[3];
	ch_status =
	    ((mxc_spdif_control.ch_status[2] << 16) | (mxc_spdif_control.
						       ch_status[1] << 8) |
	     mxc_spdif_control.ch_status[0]);
	spdif_set_channel_status(ch_status, SPDIF_REG_STCSCH);
	ch_status = mxc_spdif_control.ch_status[3];
	spdif_set_channel_status(ch_status, SPDIF_REG_STCSCL);
	return 0;
}

static int snd_mxc_spdif_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

/*!
 * Get channel status from SPDIF_RX_CCHAN register
 */
static int snd_mxc_spdif_capture_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	unsigned int cstatus;

	if (!(__raw_readl(spdif_base_addr + SPDIF_REG_SIS) & INT_CNEW))
		return -EAGAIN;

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

	return 0;
}

/*!
 * Get User bits (subcode) from chip value which readed out
 * in UChannel register.
 */
static int snd_mxc_spdif_subcode_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mxc_spdif_control.ctl_lock, flags);
	if (mxc_spdif_control.ready_buf) {
		memcpy(&ucontrol->value.iec958.subcode[0],
		       &mxc_spdif_control.
		       subcode[(mxc_spdif_control.ready_buf -
				1) * SPDIF_UBITS_SIZE], SPDIF_UBITS_SIZE);
	} else {
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&mxc_spdif_control.ctl_lock, flags);

	return ret;
}

/*!
 * Q-subcode infomation.
 * the byte size is SPDIF_UBITS_SIZE/8
 */
static int snd_mxc_spdif_qinfo(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = SPDIF_QSUB_SIZE;
	return 0;
}

/*!
 * Get Q subcode from chip value which readed out
 * in QChannel register.
 */
static int snd_mxc_spdif_qget(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mxc_spdif_control.ctl_lock, flags);
	if (mxc_spdif_control.ready_buf) {
		memcpy(&ucontrol->value.bytes.data[0],
		       &mxc_spdif_control.
		       qsub[(mxc_spdif_control.ready_buf -
			     1) * SPDIF_QSUB_SIZE], SPDIF_QSUB_SIZE);
	} else {
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&mxc_spdif_control.ctl_lock, flags);

	return ret;
}

/*!
 * Valid bit infomation.
 */
static int snd_mxc_spdif_vbit_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*!
 * Get valid good bit from interrupt status register.
 */
static int snd_mxc_spdif_vbit_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned int int_val;

	int_val = __raw_readl(spdif_base_addr + SPDIF_REG_SIS);
	ucontrol->value.integer.value[0] = (int_val & INT_VAL_NOGOOD) != 0;
	__raw_writel(INT_VAL_NOGOOD, spdif_base_addr + SPDIF_REG_SIC);

	return 0;
}

/*!
 * DPLL lock infomation.
 */
static int snd_mxc_spdif_rxrate_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 16000;
	uinfo->value.integer.max = 96000;
	return 0;
}

/*!
 * Get DPLL lock or not info from stable interrupt status register.
 * User application must use this control to get locked,
 * then can do next PCM operation
 */
static int snd_mxc_spdif_rxrate_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct mxc_spdif_device *chip = snd_kcontrol_chip(kcontrol);
	struct mxc_spdif_platform_data *spdif_data;

	spdif_data = chip->card->dev->platform_data;

	if (atomic_read(&chip->dpll_locked)) {
		ucontrol->value.integer.value[0] =
		    spdif_get_rxclk_rate(spdif_data->spdif_clk,
					 SPDIF_DEFAULT_GAINSEL);
	} else {
		ucontrol->value.integer.value[0] = 0;
	}
	return 0;
}

/*!
 * User bit sync mode info
 */
static int snd_mxc_spdif_usync_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*!
 * User bit sync mode:
 * 1 CD User channel subcode
 * 0 Non-CD data
 */
static int snd_mxc_spdif_usync_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	unsigned int int_val;

	int_val = __raw_readl(spdif_base_addr + SPDIF_REG_SRCD);
	ucontrol->value.integer.value[0] = (int_val & SRCD_CD_USER) != 0;
	return 0;
}

/*!
 * User bit sync mode:
 * 1 CD User channel subcode
 * 0 Non-CD data
 */
static int snd_mxc_spdif_usync_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	unsigned int int_val;

	int_val = ucontrol->value.integer.value[0] << SRCD_CD_USER_OFFSET;
	__raw_writel(int_val, spdif_base_addr + SPDIF_REG_SRCD);
	return 0;
}

/*!
 * MXC SPDIF IEC958 controller defines
 */
static struct snd_kcontrol_new snd_mxc_spdif_ctrls[] = {
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
	 .info = snd_mxc_spdif_info,
	 .get = snd_mxc_spdif_capture_get,
	 },
	/* user bits controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 Subcode Capture Default",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = snd_mxc_spdif_info,
	 .get = snd_mxc_spdif_subcode_get,
	 },
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 Q-subcode Capture Default",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = snd_mxc_spdif_qinfo,
	 .get = snd_mxc_spdif_qget,
	 },
	/* valid bit error controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 V-Bit Errors",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = snd_mxc_spdif_vbit_info,
	 .get = snd_mxc_spdif_vbit_get,
	 },
	/* DPLL lock info get controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "RX Sample Rate",
	 .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = snd_mxc_spdif_rxrate_info,
	 .get = snd_mxc_spdif_rxrate_get,
	 },
	/* User bit sync mode set/get controller */
	{
	 .iface = SNDRV_CTL_ELEM_IFACE_PCM,
	 .name = "IEC958 USyncMode CDText",
	 .access =
	 SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = snd_mxc_spdif_usync_info,
	 .get = snd_mxc_spdif_usync_get,
	 .put = snd_mxc_spdif_usync_put,
	 },
};

/*!
  * This function the soundcard structure.
  *
  * @param	mxc_spdif	pointer to the sound card structure.
  *
  * @return              0 on success, -1 otherwise.
  */
static int snd_card_mxc_spdif_pcm(struct mxc_spdif_device *mxc_spdif)
{
	struct snd_pcm *pcm;
	int err;
	err = snd_pcm_new(mxc_spdif->card, MXC_SPDIF_NAME, 0,
			  mxc_spdif->mxc_spdif_tx,
			  mxc_spdif->mxc_spdif_rx, &pcm);
	if (err < 0)
		return err;

	snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL),
					      SPDIF_MAX_BUF_SIZE * 2,
					      SPDIF_MAX_BUF_SIZE * 2);
	if (mxc_spdif->mxc_spdif_tx)
		snd_pcm_set_ops(pcm,
				SNDRV_PCM_STREAM_PLAYBACK,
				&snd_card_mxc_spdif_playback_ops);
	if (mxc_spdif->mxc_spdif_rx)
		snd_pcm_set_ops(pcm,
				SNDRV_PCM_STREAM_CAPTURE,
				&snd_card_mxc_spdif_capture_ops);
	pcm->private_data = mxc_spdif;
	pcm->info_flags = 0;
	strncpy(pcm->name, MXC_SPDIF_NAME, sizeof(pcm->name));
	mxc_spdif->pcm = pcm;
	mxc_init_spdif_device(mxc_spdif);
	return 0;
}

extern void gpio_spdif_active(void);

/*!
  * This function initializes the driver in terms of memory of the soundcard
  * and some basic HW clock settings.
  *
  * @param	pdev	Pointer to the platform device
  * @return              0 on success, -1 otherwise.
  */
static int mxc_alsa_spdif_probe(struct platform_device
				*pdev)
{
	int err, idx, irq;
	static int dev;
	struct snd_card *card;
	struct mxc_spdif_device *chip;
	struct resource *res;
	struct snd_kcontrol *kctl;
	struct mxc_spdif_platform_data *plat_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	/* register the soundcard */
	err = snd_card_create(index[dev], id[dev], THIS_MODULE,
			    sizeof(struct mxc_spdif_device), &card);
	if (err < 0)
		return err;
	chip = card->private_data;
	chip->card = card;
	card->dev = &pdev->dev;
	chip->reg_phys_base = res->start;
	chip->reg_base = ioremap(res->start, res->end - res->start + 1);
	spdif_base_addr = (unsigned long)chip->reg_base;
	plat_data = (struct mxc_spdif_platform_data *)pdev->dev.platform_data;
	chip->mxc_spdif_tx = plat_data->spdif_tx;
	chip->mxc_spdif_rx = plat_data->spdif_rx;
	chip->spdif_txclk_44100 = plat_data->spdif_clk_44100;
	chip->spdif_txclk_48000 = plat_data->spdif_clk_48000;
	if ((chip->spdif_txclk_44100 == 1) ||
		(chip->spdif_txclk_48000 == 1)) {
		/*spdif0_clk used as clk src*/
		struct clk *spdif_clk;
		spdif_clk = clk_get(&pdev->dev, NULL);
		clk_enable(spdif_clk);
	}
	atomic_set(&chip->dpll_locked, 0);

	err = snd_card_mxc_spdif_pcm(chip);
	if (err < 0)
		goto nodev;

	/*!
	 * Add controls to the card
	 */
	for (idx = 0; idx < ARRAY_SIZE(snd_mxc_spdif_ctrls); idx++) {

		kctl = snd_ctl_new1(&snd_mxc_spdif_ctrls[idx], chip);
		if (kctl == NULL) {
			err = -ENOMEM;
			goto nodev;
		}
		/* check to add control to corresponding substream */
		if (strstr(kctl->id.name, "Playback"))
			kctl->id.device = 0;
		else
			kctl->id.device = 1;

		err = snd_ctl_add(card, kctl);
		if (err < 0)
			goto nodev;
	}

	clk_enable(plat_data->spdif_core_clk);
	/*!
	 * SPDIF interrupt initialization
	 * software reset to SPDIF
	 */
	spdif_softreset();
	/* disable all the interrupts */
	spdif_intr_enable(0xffffff, 0);
	/* spdif interrupt register and disable */
	irq = platform_get_irq(pdev, 0);
	if ((irq <= 0) || request_irq(irq, spdif_isr, 0, "spdif", chip)) {
		pr_err("MXC spdif: failed to request irq\n");
		err = -EBUSY;
		goto nodev;
	}

	if (chip->mxc_spdif_tx)
		spin_lock_init(&chip->s[SNDRV_PCM_STREAM_PLAYBACK].dma_lock);
	if (chip->mxc_spdif_rx)
		spin_lock_init(&chip->s[SNDRV_PCM_STREAM_CAPTURE].dma_lock);
	strcpy(card->driver, MXC_SPDIF_NAME);
	strcpy(card->shortname, "MXC SPDIF TX/RX");
	sprintf(card->longname, "MXC Freescale with SPDIF");

	err = snd_card_register(card);
	if (err == 0) {
		pr_info("MXC spdif support initialized\n");
		platform_set_drvdata(pdev, card);
		gpio_spdif_active();
		return 0;
	}

      nodev:
	snd_card_free(card);
	return err;
}

extern void gpio_spdif_inactive(void);

/*!
  * This function releases the sound card and unmap the io address
  *
  * @param      pdev    Pointer to the platform device
  * @return              0 on success, -1 otherwise.
  */

static int mxc_alsa_spdif_remove(struct platform_device *pdev)
{
	struct mxc_spdif_device *chip;
	struct snd_card *card;
	struct mxc_spdif_platform_data *plat_data;

	card = platform_get_drvdata(pdev);
	plat_data = pdev->dev.platform_data;
	chip = card->private_data;
	free_irq(platform_get_irq(pdev, 0), chip);
	iounmap(chip->reg_base);

	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);

	clk_disable(plat_data->spdif_core_clk);
	gpio_spdif_inactive();

	return 0;
}

#ifdef CONFIG_PM
/*!
  * This function suspends all active streams.
  *
  * TBD
  *
  * @param	card	pointer to the sound card structure.
  * @param	state	requested state
  *
  * @return              0 on success, -1 otherwise.
  */
static int mxc_alsa_spdif_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	return 0;
}

/*!
  * This function resumes all suspended streams.
  *
  * TBD
  *
  * @param	card	pointer to the sound card structure.
  * @param	state	requested state
  *
  * @return              0 on success, -1 otherwise.
  */
static int mxc_alsa_spdif_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver mxc_alsa_spdif_driver = {
	.probe = mxc_alsa_spdif_probe,
	.remove = mxc_alsa_spdif_remove,
#ifdef CONFIG_PM
	.suspend = mxc_alsa_spdif_suspend,
	.resume = mxc_alsa_spdif_resume,
#endif
	.driver = {
		   .name = "mxc_alsa_spdif",
		   },
};

/*!
  * This function registers the sound driver structure.
  *
  */
static int __init mxc_alsa_spdif_init(void)
{
	return platform_driver_register(&mxc_alsa_spdif_driver);
}

/*!
  * This function frees the sound driver structure.
  *
  */
static void __exit mxc_alsa_spdif_exit(void)
{
	platform_driver_unregister(&mxc_alsa_spdif_driver);
}

module_init(mxc_alsa_spdif_init);
module_exit(mxc_alsa_spdif_exit);
MODULE_AUTHOR("FREESCALE SEMICONDUCTOR");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC ALSA driver for SPDIF");
