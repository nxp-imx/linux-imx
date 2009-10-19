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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/math64.h>

#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/version.h>	/* for KERNEL_VERSION MACRO */
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <mach/regs-dri.h>
#include <mach/regs-apbx.h>
#include <mach/regs-clkctrl.h>

#include "stfm1000.h"

static DEFINE_MUTEX(devlist_lock);
static unsigned int stfm1000_devcount;

LIST_HEAD(stfm1000_devlist);
EXPORT_SYMBOL(stfm1000_devlist);

/* alsa interface */
struct stfm1000_alsa_ops *stfm1000_alsa_ops;
EXPORT_SYMBOL(stfm1000_alsa_ops);

/* region, 0=US, 1=europe */
static int georegion = 1;	/* default is europe */
static int rds_enable = 1;	/* default is enabled */

static int sw_tune(struct stfm1000 *stfm1000, u32 freq);

static const const char *stfm1000_get_rev_txt(u32 id)
{
	switch (id) {
	case 0x01: return "TA1";
	case 0x02: return "TA2";
	case 0x11: return "TB1";
	case 0x12: return "TB2";
	}
	return NULL;
}

static const struct stfm1000_reg stfm1000_tb2_powerup[] = {
	STFM1000_REG(REF, 0x00200000),
	STFM1000_DELAY(20),
	STFM1000_REG(DATAPATH, 0x00010210),
	STFM1000_REG(TUNE1, 0x0004CF01),
	STFM1000_REG(SDNOMINAL, 0x1C5EBCF0),
	STFM1000_REG(PILOTTRACKING, 0x000001B6),
	STFM1000_REG(INITIALIZATION1, 0x9fb80008),
	STFM1000_REG(INITIALIZATION2, 0x8516e444 | STFM1000_DEEMPH_50_75B),
	STFM1000_REG(INITIALIZATION3, 0x1402190b),
	STFM1000_REG(INITIALIZATION4, 0x525bf052),
	STFM1000_REG(INITIALIZATION5, 0x1000d106),
	STFM1000_REG(INITIALIZATION6, 0x000062cb),
	STFM1000_REG(AGC_CONTROL1, 0x1BCB2202),
	STFM1000_REG(AGC_CONTROL2, 0x000020F0),
	STFM1000_REG(CLK1, 0x10000000),
	STFM1000_REG(CLK1, 0x20000000),
	STFM1000_REG(CLK1, 0x00000000),
	STFM1000_REG(CLK2, 0x7f000000),
	STFM1000_REG(REF, 0x00B8222D),
	STFM1000_REG(CLK1, 0x30000000),
	STFM1000_REG(CLK1, 0x30002000),
	STFM1000_REG(CLK1, 0x10002000),
	STFM1000_REG(LNA, 0x0D080009),
	STFM1000_DELAY(10),
	STFM1000_REG(MIXFILT, 0x00008000),
	STFM1000_REG(MIXFILT, 0x00000000),
	STFM1000_REG(MIXFILT, 0x00007205),
	STFM1000_REG(ADC, 0x001B3282),
	STFM1000_REG(ATTENTION, 0x0000003F),
	STFM1000_END,
};

static const struct stfm1000_reg stfm1000_ta2_powerup[] = {
	STFM1000_REG(REF, 0x00200000),
	STFM1000_DELAY(20),
	STFM1000_REG(DATAPATH, 0x00010210),
	STFM1000_REG(TUNE1, 0x00044F01),
	STFM1000_REG(SDNOMINAL, 0x1C5EBCF0),
	STFM1000_REG(PILOTTRACKING, 0x000001B6),
	STFM1000_REG(INITIALIZATION1, 0x9fb80008),
	STFM1000_REG(INITIALIZATION2, 0x8506e444),
	STFM1000_REG(INITIALIZATION3, 0x1402190b),
	STFM1000_REG(INITIALIZATION4, 0x525bf052),
	STFM1000_REG(INITIALIZATION5, 0x7000d106),
	STFM1000_REG(INITIALIZATION6, 0x0000c2cb),
	STFM1000_REG(AGC_CONTROL1, 0x002c8402),
	STFM1000_REG(AGC_CONTROL2, 0x00140050),
	STFM1000_REG(CLK1, 0x10000000),
	STFM1000_REG(CLK1, 0x20000000),
	STFM1000_REG(CLK1, 0x00000000),
	STFM1000_REG(CLK2, 0x7f000000),
	STFM1000_REG(REF, 0x0030222D),
	STFM1000_REG(CLK1, 0x30000000),
	STFM1000_REG(CLK1, 0x30002000),
	STFM1000_REG(CLK1, 0x10002000),
	STFM1000_REG(LNA, 0x05080009),
	STFM1000_REG(MIXFILT, 0x00008000),
	STFM1000_REG(MIXFILT, 0x00000000),
	STFM1000_REG(MIXFILT, 0x00007200),
	STFM1000_REG(ADC, 0x00033000),
	STFM1000_REG(ATTENTION, 0x0000003F),
	STFM1000_END,
};

static const struct stfm1000_reg stfm1000_powerdown[] = {
	STFM1000_REG(DATAPATH, 0x00010210),
	STFM1000_REG(REF, 0),
	STFM1000_REG(LNA, 0),
	STFM1000_REG(MIXFILT, 0),
	STFM1000_REG(CLK1, 0x20000000),
	STFM1000_REG(CLK1, 0),
	STFM1000_REG(CLK2, 0),
	STFM1000_REG(ADC, 0),
	STFM1000_REG(TUNE1, 0),
	STFM1000_REG(SDNOMINAL, 0),
	STFM1000_REG(PILOTTRACKING, 0),
	STFM1000_REG(INITIALIZATION1, 0),
	STFM1000_REG(INITIALIZATION2, 0),
	STFM1000_REG(INITIALIZATION3, 0),
	STFM1000_REG(INITIALIZATION4, 0),
	STFM1000_REG(INITIALIZATION5, 0),
	STFM1000_REG(INITIALIZATION6, 0x00007E00),
	STFM1000_REG(AGC_CONTROL1, 0),
	STFM1000_REG(AGC_CONTROL2, 0),
	STFM1000_REG(DATAPATH, 0x00000200),
};

struct stfm1000_tuner_pmi {
	u32 min;
	u32 max;
	u32 freq;
	u32 pll_xtal;	/* 1 = pll, 0 = xtal */
};

#define PLL 1
#define XTAL 0

static const struct stfm1000_tuner_pmi stfm1000_pmi_lookup[] = {
	{ .min =  76100, .max =  76500, .freq = 19200, .pll_xtal = PLL },
	{ .min =  79700, .max =  79900, .freq = 19200, .pll_xtal = PLL },
	{ .min =  80800, .max =  81200, .freq = 19200, .pll_xtal = PLL },
	{ .min =  82100, .max =  82600, .freq = 19200, .pll_xtal = PLL },
	{ .min =  86800, .max =  87200, .freq = 19200, .pll_xtal = PLL },
	{ .min =  88100, .max =  88600, .freq = 19200, .pll_xtal = PLL },
	{ .min =  89800, .max =  90500, .freq = 19200, .pll_xtal = PLL },
	{ .min =  91400, .max =  91900, .freq = 19200, .pll_xtal = PLL },
	{ .min =  92800, .max =  93300, .freq = 19200, .pll_xtal = PLL },
	{ .min =  97400, .max =  97900, .freq = 19200, .pll_xtal = PLL },
	{ .min =  98800, .max =  99200, .freq = 19200, .pll_xtal = PLL },
	{ .min = 100200, .max = 100400, .freq = 19200, .pll_xtal = PLL },
	{ .min = 103500, .max = 103900, .freq = 19200, .pll_xtal = PLL },
	{ .min = 104800, .max = 105200, .freq = 19200, .pll_xtal = PLL },
	{ .min = 106100, .max = 106500, .freq = 19200, .pll_xtal = PLL },

	{ .min =  76600, .max =  77000, .freq = 20000, .pll_xtal = PLL },
	{ .min =  77800, .max =  78300, .freq = 20000, .pll_xtal = PLL },
	{ .min =  79200, .max =  79600, .freq = 20000, .pll_xtal = PLL },
	{ .min =  80600, .max =  80700, .freq = 20000, .pll_xtal = PLL },
	{ .min =  83900, .max =  84400, .freq = 20000, .pll_xtal = PLL },
	{ .min =  85300, .max =  85800, .freq = 20000, .pll_xtal = PLL },
	{ .min =  94200, .max =  94700, .freq = 20000, .pll_xtal = PLL },
	{ .min =  95600, .max =  96100, .freq = 20000, .pll_xtal = PLL },
	{ .min = 100500, .max = 100800, .freq = 20000, .pll_xtal = PLL },
	{ .min = 101800, .max = 102200, .freq = 20000, .pll_xtal = PLL },
	{ .min = 103100, .max = 103400, .freq = 20000, .pll_xtal = PLL },
	{ .min = 106600, .max = 106900, .freq = 20000, .pll_xtal = PLL },
	{ .min = 107800, .max = 108000, .freq = 20000, .pll_xtal = PLL },

	{ .min =      0, .max =      0, .freq = 24000, .pll_xtal = XTAL }
};

int stfm1000_power_up(struct stfm1000 *stfm1000)
{
	struct stfm1000_reg *reg, *pwrup_reg;
	const struct stfm1000_reg *orig_reg, *treg;
	int ret, size;

	mutex_lock(&stfm1000->state_lock);

	/* Enable DRI clock for 24Mhz. */
	HW_CLKCTRL_XTAL_CLR(BM_CLKCTRL_XTAL_DRI_CLK24M_GATE);

	orig_reg = stfm1000->revid == STFM1000_CHIP_REV_TA2 ?
			stfm1000_ta2_powerup : stfm1000_tb2_powerup;

	/* find size of the set */
	for (treg = orig_reg; treg->regno != STFM1000_REG_END; treg++)
		;
	size = (treg + 1 - orig_reg) * sizeof(*treg);

	/* allocate copy */
	pwrup_reg = kmalloc(size, GFP_KERNEL);
	if (pwrup_reg == NULL) {
		printk(KERN_ERR "%s: out of memory\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	/* copy it */
	memcpy(pwrup_reg, orig_reg, size);

	/* fixup region of INITILIZATION2 */
	for (reg = pwrup_reg; reg->regno != STFM1000_REG_END; reg++) {

		/* we only care for INITIALIZATION2 register */
		if (reg->regno != STFM1000_INITIALIZATION2)
			continue;

		/* geographic region select */
		if (stfm1000->georegion == 0)	/* USA */
			reg->value &= ~STFM1000_DEEMPH_50_75B;
		else			/* Europe */
			reg->value |=  STFM1000_DEEMPH_50_75B;

		/* RDS enabled */
		if (stfm1000->revid == STFM1000_CHIP_REV_TB2) {
			if (stfm1000->rds_enable)
				reg->value |=  STFM1000_RDS_ENABLE;
			else
				reg->value &= ~STFM1000_RDS_ENABLE;
		}
	}

	ret = stfm1000_write_regs(stfm1000, pwrup_reg);

	kfree(pwrup_reg);
out:
	mutex_unlock(&stfm1000->state_lock);

	return ret;
}

int stfm1000_power_down(struct stfm1000 *stfm1000)
{
	int ret;

	mutex_lock(&stfm1000->state_lock);

	/* Disable DRI clock for 24Mhz. */
	HW_CLKCTRL_XTAL_CLR(BM_CLKCTRL_XTAL_DRI_CLK24M_GATE);

	ret = stfm1000_write_regs(stfm1000, stfm1000_powerdown);

	/* Disable DRI clock for 24Mhz. */
	/* XXX bug warning, disabling the DRI clock is bad news */
	/* doing so causes noise to be received from the DRI */
	/* interface. Leave it on for now */
	/* HW_CLKCTRL_XTAL_CLR(BM_CLKCTRL_XTAL_DRI_CLK24M_GATE); */

	mutex_unlock(&stfm1000->state_lock);

	return ret;
}

int stfm1000_dcdc_update(struct stfm1000 *stfm1000, u32 freq)
{
	const struct stfm1000_tuner_pmi *pmi;
	int i;

	/* search for DCDC frequency */
	pmi = stfm1000_pmi_lookup;
	for (i = 0;  i < ARRAY_SIZE(stfm1000_pmi_lookup); i++, pmi++) {
		if (freq >= pmi->min && freq <= pmi->max)
			break;
	}
	if (i >= ARRAY_SIZE(stfm1000_pmi_lookup))
		return -1;

	/* adjust DCDC frequency so that it is out of Tuner PLL range */
	/* XXX there is no adjustment API (os_pmi_SetDcdcFreq)*/
	return 0;
}

static void Mute_Audio(struct stfm1000 *stfm1000)
{
	stfm1000->mute = 1;
}

static void Unmute_Audio(struct stfm1000 *stfm1000)
{
	stfm1000->mute = 0;
}

static const struct stfm1000_reg sd_dp_on_regs[] = {
	STFM1000_REG_SETBITS(DATAPATH, STFM1000_DP_EN),
	STFM1000_DELAY(3),
	STFM1000_REG_SETBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_REG_CLRBITS(AGC_CONTROL1, STFM1000_B2_BYPASS_AGC_CTL),
	STFM1000_REG_CLRBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_END,
};

static int SD_DP_On(struct stfm1000 *stfm1000)
{
	int ret;

	ret = stfm1000_write_regs(stfm1000, sd_dp_on_regs);
	if (ret != 0)
		return ret;

	return 0;
}

static const struct stfm1000_reg sd_dp_off_regs[] = {
	STFM1000_REG_SETBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_REG_CLRBITS(DATAPATH, STFM1000_DP_EN),
	STFM1000_REG_SETBITS(AGC_CONTROL1, STFM1000_B2_BYPASS_AGC_CTL),
	STFM1000_REG_CLRBITS(PILOTTRACKING, STFM1000_B2_PILOTTRACKING_EN),
	STFM1000_REG_CLRBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_END,
};

static int SD_DP_Off(struct stfm1000 *stfm1000)
{
	int ret;

	ret = stfm1000_write_regs(stfm1000, sd_dp_off_regs);
	if (ret != 0)
		return ret;

	return 0;
}

static int DRI_Start_Stream(struct stfm1000 *stfm1000)
{
	dma_addr_t dma_buffer_phys;
	int i, next;
	u32 cmd;

	/* we must not be gated */
	BUG_ON(HW_CLKCTRL_XTAL_RD() & BM_CLKCTRL_XTAL_DRI_CLK24M_GATE);

	/* hw_dri_SetReset */
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_SFTRST | BM_DRI_CTRL_CLKGATE);
	HW_DRI_CTRL_SET(BM_DRI_CTRL_SFTRST);
	while ((HW_DRI_CTRL_RD() & BM_DRI_CTRL_CLKGATE) == 0)
		cpu_relax();
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_SFTRST | BM_DRI_CTRL_CLKGATE);

	/* DRI enable/config */
	HW_DRI_TIMING_WR(BF_DRI_TIMING_GAP_DETECTION_INTERVAL(0x10) |
			 BF_DRI_TIMING_PILOT_REP_RATE(0x08));

	/* XXX SDK bug */
	/* While the SDK enables the gate here, everytime the stream */
	/* is started, doing so, causes the DRI to input audio noise */
	/* at any subsequent starts */
	/* Enable DRI clock for 24Mhz. */
	/* HW_CLKCTRL_XTAL_CLR(BM_CLKCTRL_XTAL_DRI_CLK24M_GATE); */

	stmp3xxx_arch_dma_reset_channel(stfm1000->dma_ch);

	dma_buffer_phys = stfm1000->dri_phys;

	for (i = 0; i < stfm1000->blocks; i++) {
		next = (i + 1) % stfm1000->blocks;

		/* link */
		stfm1000->dma[i].command->next = stfm1000->dma[next].handle;
		stfm1000->dma[i].next_descr = &stfm1000->dma[next];

		/* receive DRI is 8 bytes per 4 samples */
		cmd = BF_APBX_CHn_CMD_XFER_COUNT(stfm1000->blksize * 2) |
		      BM_APBX_CHn_CMD_IRQONCMPLT |
		      BM_APBX_CHn_CMD_CHAIN |
		      BF_APBX_CHn_CMD_COMMAND(
			BV_APBX_CHn_CMD_COMMAND__DMA_WRITE);

		stfm1000->dma[i].command->cmd = cmd;
		stfm1000->dma[i].command->buf_ptr = dma_buffer_phys;
		stfm1000->dma[i].command->pio_words[0] =
			BM_DRI_CTRL_OVERFLOW_IRQ_EN |
			BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ_EN |
			BM_DRI_CTRL_ATTENTION_IRQ_EN |
			/* BM_DRI_CTRL_STOP_ON_OFLOW_ERROR | */
			/* BM_DRI_CTRL_STOP_ON_PILOT_ERROR | */
			BM_DRI_CTRL_ENABLE_INPUTS;

		dma_buffer_phys += stfm1000->blksize * 2;

	}

	/* Enable completion interrupt */
	stmp3xxx_dma_clear_interrupt(stfm1000->dma_ch);
	stmp3xxx_dma_enable_interrupt(stfm1000->dma_ch);

	/* clear DRI interrupts pending */
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_OVERFLOW_IRQ |
			BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ |
			BM_DRI_CTRL_ATTENTION_IRQ);

	/* Stop DRI on error */
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_STOP_ON_OFLOW_ERROR |
			BM_DRI_CTRL_STOP_ON_PILOT_ERROR);

	/* Reacquire data stream */
	HW_DRI_CTRL_SET(BM_DRI_CTRL_REACQUIRE_PHASE |
		BM_DRI_CTRL_OVERFLOW_IRQ_EN |
		BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ_EN |
		BM_DRI_CTRL_ATTENTION_IRQ_EN |
		BM_DRI_CTRL_ENABLE_INPUTS);

	stmp3xxx_dma_go(stfm1000->dma_ch, stfm1000->dma, 1);

	/* Turn on DRI hardware (don't forget to leave RUN bit ON) */
	HW_DRI_CTRL_SET(BM_DRI_CTRL_RUN);

	return 0;
}

static int DRI_Stop_Stream(struct stfm1000 *stfm1000)
{
	int desc;

	/* disable interrupts */
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_OVERFLOW_IRQ_EN |
			BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ_EN |
			BM_DRI_CTRL_ATTENTION_IRQ_EN);

	/* Freeze DMA channel for a moment */
	stmp3xxx_dma_freeze(stfm1000->dma_ch);

	/* all descriptors, set sema bit */
	for (desc = 0; desc < stfm1000->blocks; desc++)
		stfm1000->dma[desc].command->cmd |= BM_APBX_CHn_CMD_SEMAPHORE;

	/* Let the current DMA transaction finish */
	stmp3xxx_dma_unfreeze(stfm1000->dma_ch);
	msleep(5);

	/* dma shutdown */
	stmp3xxx_arch_dma_reset_channel(stfm1000->dma_ch);

	/* Turn OFF data lines and stop controller */
	HW_DRI_CTRL_CLR(BM_DRI_CTRL_ENABLE_INPUTS | BM_DRI_CTRL_RUN);

	/* hw_dri_SetReset */
	HW_DRI_CTRL_SET(BM_DRI_CTRL_SFTRST | BM_DRI_CTRL_CLKGATE);

	/* XXX SDK bug */
	/* While the SDK enables the gate here, everytime the stream */
	/* is started, doing so, causes the DRI to input audio noise */
	/* at any subsequent starts */
	/* Enable DRI clock for 24Mhz. */
	/* Disable DRI clock for 24Mhz. */
	/* HW_CLKCTRL_XTAL_SET(BM_CLKCTRL_XTAL_DRI_CLK24M_GATE); */

	return 0;
}

static int DRI_On(struct stfm1000 *stfm1000)
{
	int ret;

	if (stfm1000->active)
		DRI_Start_Stream(stfm1000);

	ret = stfm1000_set_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_SAI_EN);
	return ret;
}

static int DRI_Off(struct stfm1000 *stfm1000)
{
	int ret;

	if (stfm1000->active)
		DRI_Stop_Stream(stfm1000);

	ret = stfm1000_clear_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_SAI_EN);

	return 0;
}

static int SD_Set_Channel_Filter(struct stfm1000 *stfm1000)
{
	int bypass_setting;
	int sig_qual;
	u32 tmp;
	int ret;

	/*
	 * set channel filter
	 *
	 * B2_NEAR_CHAN_MIX_REG_MASK values from T-Spec
	 * 000 : 0 kHz mix.
	 * 001 : +100 kHz mix.
	 * 010 : +200 kHz mix.
	 * 011 : +300 kHz mix.
	 * 100 : -400 kHz mix.
	 * 101 : -300 kHz mix.
	 * 110 : -200 kHz mix.
	 * 111 : -100 kHz mix.
	 */

	/* get near channel amplitude */
	ret = stfm1000_write_masked(stfm1000, STFM1000_INITIALIZATION3,
		STFM1000_B2_NEAR_CHAN_MIX(0x01),
		STFM1000_B2_NEAR_CHAN_MIX_MASK);
	if (ret != 0)
		return ret;

	msleep(10);	/* wait for the signal quality to settle */

	ret = stfm1000_read(stfm1000, STFM1000_SIGNALQUALITY, &tmp);
	if (ret != 0)
		return ret;

	sig_qual = (tmp & STFM1000_NEAR_CHAN_AMPLITUDE_MASK) >>
		STFM1000_NEAR_CHAN_AMPLITUDE_SHIFT;

	bypass_setting = 0;

	/* check near channel amplitude vs threshold */
	if (sig_qual < stfm1000->adj_chan_th) {
		/* get near channel amplitude again */
		ret = stfm1000_write_masked(stfm1000, STFM1000_INITIALIZATION3,
			STFM1000_B2_NEAR_CHAN_MIX(0x05),
			STFM1000_B2_NEAR_CHAN_MIX_MASK);
		if (ret != 0)
			return ret;

		msleep(10);	/* wait for the signal quality to settle */

		ret = stfm1000_read(stfm1000, STFM1000_SIGNALQUALITY, &tmp);
		if (ret != 0)
			return ret;

		sig_qual = (tmp & STFM1000_NEAR_CHAN_AMPLITUDE_MASK) >>
			STFM1000_NEAR_CHAN_AMPLITUDE_SHIFT;

		if (sig_qual < stfm1000->adj_chan_th)
			bypass_setting = 2;
	}

	/* set filter settings */
	ret = stfm1000_write_masked(stfm1000, STFM1000_INITIALIZATION1,
		STFM1000_B2_BYPASS_FILT(bypass_setting),
		STFM1000_B2_BYPASS_FILT_MASK);
	if (ret != 0)
		return ret;

	return 0;
}

static int SD_Look_For_Pilot_TA2(struct stfm1000 *stfm1000)
{
	int i;
	u32 pilot;
	int ret;

	/* assume pilot */
	stfm1000->pilot_present = 1;

	for (i = 0; i < 3; i++) {

		ret = stfm1000_read(stfm1000, STFM1000_PILOTCORRECTION,
			&pilot);
		if (ret != 0)
			return ret;

		pilot &= STFM1000_PILOTEST_TA2_MASK;
		pilot >>= STFM1000_PILOTEST_TA2_SHIFT;

		/* out of range? */
		if (pilot < 0xe2 || pilot >= 0xb5) {
			stfm1000->pilot_present = 0;
			break;
		}
	}

	return 0;
}


static int SD_Look_For_Pilot_TB2(struct stfm1000 *stfm1000)
{
	int i;
	u32 pilot;
	int ret;

	/* assume pilot */
	stfm1000->pilot_present = 1;

	for (i = 0; i < 3; i++) {

		ret = stfm1000_read(stfm1000, STFM1000_PILOTCORRECTION,
			&pilot);
		if (ret != 0)
			return ret;

		pilot &= STFM1000_PILOTEST_TB2_MASK;
		pilot >>= STFM1000_PILOTEST_TB2_SHIFT;

		/* out of range? */
		if (pilot < 0x1e || pilot >= 0x7f) {
			stfm1000->pilot_present = 0;
			break;
		}
	}

	return 0;
}

static int SD_Look_For_Pilot(struct stfm1000 *stfm1000)
{
	int ret;

	if (stfm1000->revid == STFM1000_CHIP_REV_TA2)
		ret = SD_Look_For_Pilot_TA2(stfm1000);
	else
		ret = SD_Look_For_Pilot_TB2(stfm1000);

	if (ret != 0)
		return ret;

	if (!stfm1000->pilot_present) {
		ret = stfm1000_clear_bits(stfm1000, STFM1000_PILOTTRACKING,
			STFM1000_B2_PILOTTRACKING_EN);
		if (ret != 0)
			return ret;

		/* set force mono parameters for the filter */
		stfm1000->filter_parms.pCoefForcedMono = 1;

		/* yeah, I know, it's stupid */
		stfm1000->rds_state.demod.pCoefForcedMono =
			stfm1000->filter_parms.pCoefForcedMono;
	}

	return 0;
}

static int SD_Gear_Shift_Pilot_Tracking(struct stfm1000 *stfm1000)
{
	static const struct {
		int delay;
		u32 value;
	} track_table[] = {
		{ .delay = 10, .value = 0x81b6 },
		{ .delay =  6, .value = 0x82a5 },
		{ .delay =  6, .value = 0x8395 },
		{ .delay =  8, .value = 0x8474 },
		{ .delay = 20, .value = 0x8535 },
		{ .delay = 50, .value = 0x8632 },
		{ .delay =  0, .value = 0x8810 },
	};
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(track_table); i++) {
		ret = stfm1000_write(stfm1000, STFM1000_PILOTTRACKING,
			track_table[i].value);
		if (ret != 0)
			return ret;

		if (i < ARRAY_SIZE(track_table) - 1)	/* last one no delay */
			msleep(track_table[i].delay);
	}

	return 0;
}

static int SD_Optimize_Channel(struct stfm1000 *stfm1000)
{
	int ret;

	ret = stfm1000_set_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_DB_ACCEPT);
	if (ret != 0)
		return ret;

	ret = stfm1000_write(stfm1000, STFM1000_PILOTTRACKING,
		STFM1000_B2_PILOTTRACKING_EN |
		STFM1000_B2_PILOTLPF_TIMECONSTANT(0x01) |
		STFM1000_B2_PFDSCALE(0x0B) |
		STFM1000_B2_PFDFILTER_SPEEDUP(0x06));	/* 0x000081B6 */
	if (ret != 0)
		return ret;

	ret = SD_Set_Channel_Filter(stfm1000);
	if (ret != 0)
		return ret;

	ret = SD_Look_For_Pilot(stfm1000);
	if (ret != 0)
		return ret;

	if (stfm1000->pilot_present) {
		ret = SD_Gear_Shift_Pilot_Tracking(stfm1000);
		if (ret != 0)
			return ret;
	}

	ret = stfm1000_clear_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_DB_ACCEPT);
	if (ret != 0)
		return ret;

	return 0;
}

static int Monitor_STFM_Quality(struct stfm1000 *stfm1000)
{
	u32 tmp, rssi_dc_est, tone_data;
	u32 lna_rms, bias, agc_out, lna_th, lna, ref;
	u16 rssi_mantissa, rssi_exponent, rssi_decoded;
	u16 prssi;
	s16 mpx_dc;
	int rssi_log;
	int bypass_filter;
	int ret;

	ret = stfm1000_set_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_DB_ACCEPT);
	if (ret != 0)
		return ret;

	/* Get Rssi register readings from STFM1000 */
	stfm1000_read(stfm1000, STFM1000_RSSI_TONE, &tmp);
	rssi_dc_est = tmp & 0xffff;
	tone_data = (tmp >> 16) & 0x0fff;

	rssi_mantissa = (rssi_dc_est & 0xffe0) >> 5;	/* 11Msb */
	rssi_exponent = rssi_dc_est & 0x001f;	/* 5 lsb */
	rssi_decoded = (u32)rssi_mantissa << rssi_exponent;

	/* Convert Rsst to 10log(Rssi) */
	for (prssi = 20; prssi > 0; prssi--)
		if (rssi_decoded >= (1 << prssi))
			break;

	rssi_log = (3 * rssi_decoded >> prssi) + (3 * prssi - 3);
	/* clamp to positive */
	if (rssi_log < 0)
		rssi_log = 0;
	/* Compensate for errors in truncation/approximation by adding 1 */
	rssi_log++;

	stfm1000->rssi_dc_est_log = rssi_log;
	stfm1000->signal_strength = stfm1000->rssi_dc_est_log;

	/* determine absolute value */
	if (tmp & 0x0800)
		mpx_dc = ((tmp >> 16) & 0x0fff) | 0xf000;
	else
		mpx_dc = (tmp >> 16) & 0x0fff;
	stfm1000->mpx_dc = mpx_dc;
	mpx_dc = mpx_dc < 0 ? -mpx_dc : mpx_dc;

	if (stfm1000->tuning_grid_50KHz)
		stfm1000->is_station = rssi_log > stfm1000->tune_rssi_th;
	else
		stfm1000->is_station = rssi_log > stfm1000->tune_rssi_th &&
			mpx_dc > stfm1000->tune_mpx_dc_th;

	/* weak signal? */
	if (stfm1000->rssi_dc_est_log <
		(stfm1000->filter_parms.pCoefLmrGaTh - 20)) {

		if (stfm1000->pilot_present)
			bypass_filter = 1;	/* Filter settings #2 */
		else
			bypass_filter = 0;

		/* configure filter for narrow band */
		ret = stfm1000_write_masked(stfm1000, STFM1000_AGC_CONTROL1,
				STFM1000_B2_BYPASS_FILT(bypass_filter),
				STFM1000_B2_BYPASS_FILT_MASK);
		if (ret != 0)
			return ret;

		/* Turn off pilot tracking */
		ret = stfm1000_clear_bits(stfm1000, STFM1000_PILOTTRACKING,
				STFM1000_B2_PILOTTRACKING_EN);
		if (ret != 0)
			return ret;

		/* enable "forced mono" in black box */
		stfm1000->filter_parms.pCoefForcedMono = 1;

		/* yeah, I know, it's stupid */
		stfm1000->rds_state.demod.pCoefForcedMono =
			stfm1000->filter_parms.pCoefForcedMono;

		/* Set weak signal flag */
		stfm1000->weak_signal = 1;

		if (stfm1000->revid == STFM1000_CHIP_REV_TA2) {

			/* read AGC_STAT register */
			ret = stfm1000_read(stfm1000, STFM1000_AGC_STAT, &tmp);
			if (ret != 0)
				return ret;

			lna_rms = (tmp & STFM1000_LNA_RMS_MASK) >>
				STFM1000_LNA_RMS_SHIFT;

			/* Check  the energy level from LNA Power Meter A/D */
			if (lna_rms == 0)
				bias = STFM1000_IBIAS2_DN | STFM1000_IBIAS1_UP;
			else
				bias = STFM1000_IBIAS2_UP | STFM1000_IBIAS1_DN;

			if (lna_rms == 0 || lna_rms > 2) {
				ret = stfm1000_write_masked(stfm1000,
					STFM1000_LNA, bias,
					STFM1000_IBIAS2_UP |
					STFM1000_IBIAS2_DN |
					STFM1000_IBIAS1_UP |
					STFM1000_IBIAS1_DN);
				if (ret != 0)
					return ret;
			}

		} else {

			/* Set LNA bias */

			/* read AGC_STAT register */
			ret = stfm1000_read(stfm1000, STFM1000_AGC_STAT, &tmp);
			if (ret != 0)
				return ret;

			agc_out = (tmp & STFM1000_AGCOUT_STAT_MASK) >>
					STFM1000_AGCOUT_STAT_SHIFT;

			/* read LNA register (this is a cached register) */
			ret = stfm1000_read(stfm1000, STFM1000_LNA, &lna);
			if (ret != 0)
				return ret;

			/* read REF register (this is a cached register) */
			ret = stfm1000_read(stfm1000, STFM1000_REF, &ref);
			if (ret != 0)
				return ret;

/* work around the 80 line width problem */
#undef LNADEF
#define LNADEF STFM1000_LNA_AMP1_IMPROVE_DISTORTION
			if (agc_out == 31) {
				if (rssi_log <= 16) {
					if (lna & STFM1000_IBIAS1_DN)
						lna &= ~STFM1000_IBIAS1_DN;
					else {
						lna |=  STFM1000_IBIAS1_UP;
						ref &= ~LNADEF;
					}
				}
				if (rssi_log >= 26) {
					if (lna & STFM1000_IBIAS1_UP) {
						lna &= ~STFM1000_IBIAS1_UP;
						ref |=  LNADEF;
					} else
						lna |=  STFM1000_IBIAS1_DN;
				}
			} else {
				lna &= ~STFM1000_IBIAS1_UP;
				lna |=  STFM1000_IBIAS1_DN;
				ref |=  LNADEF;
			}
#undef LNADEF

			ret = stfm1000_write_masked(stfm1000, STFM1000_LNA,
				lna, STFM1000_IBIAS1_UP | STFM1000_IBIAS1_DN);
			if (ret != 0)
				return ret;

			ret = stfm1000_write_masked(stfm1000, STFM1000_REF,
				ref, STFM1000_LNA_AMP1_IMPROVE_DISTORTION);
			if (ret != 0)
				return ret;
		}

	} else if (stfm1000->rssi_dc_est_log >
		(stfm1000->filter_parms.pCoefLmrGaTh - 17)) {

			bias = STFM1000_IBIAS2_UP | STFM1000_IBIAS1_DN;

			ret = stfm1000_write_masked(stfm1000, STFM1000_LNA,
				bias, STFM1000_IBIAS2_UP | STFM1000_IBIAS2_DN |
				STFM1000_IBIAS1_UP | STFM1000_IBIAS1_DN);
			if (ret != 0)
				return ret;

			ret = SD_Set_Channel_Filter(stfm1000);
			if (ret != 0)
				return ret;

			ret = SD_Look_For_Pilot(stfm1000);
			if (ret != 0)
				return ret;

			if (stfm1000->pilot_present) {
				if (stfm1000->prev_pilot_present ||
					stfm1000->weak_signal) {

					/* gear shift pilot tracking */
					ret = SD_Gear_Shift_Pilot_Tracking(
						stfm1000);
					if (ret != 0)
						return ret;

					/* set force mono parameters for the
					 * filter */
					stfm1000->filter_parms.
						pCoefForcedMono = stfm1000->
							force_mono;

					/* yeah, I know, it's stupid */
					stfm1000->rds_state.demod.
						pCoefForcedMono = stfm1000->
							filter_parms.
							pCoefForcedMono;
				}
			} else {
				ret = stfm1000_clear_bits(stfm1000,
					STFM1000_PILOTTRACKING,
					STFM1000_B2_PILOTTRACKING_EN);
				if (ret != 0)
					return ret;

				/* set force mono parameters for the filter */
				stfm1000->filter_parms.pCoefForcedMono = 1;

				/* yeah, I know, it's stupid */
				stfm1000->rds_state.demod.pCoefForcedMono =
					stfm1000->filter_parms.pCoefForcedMono;
			}

			/* Reset weak signal flag */
			stfm1000->weak_signal = 0;
			stfm1000->prev_pilot_present = stfm1000->pilot_present;

	} else {

		ret = SD_Look_For_Pilot(stfm1000);
		if (ret != 0)
			return ret;

		if (!stfm1000->pilot_present) {
			ret = stfm1000_clear_bits(stfm1000,
				STFM1000_PILOTTRACKING,
				STFM1000_B2_PILOTTRACKING_EN);
			if (ret != 0)
				return ret;

			/* set force mono parameters for the filter */
			stfm1000->filter_parms.pCoefForcedMono = 1;

			/* yeah, I know, it's stupid */
			stfm1000->rds_state.demod.pCoefForcedMono =
				stfm1000->filter_parms.pCoefForcedMono;

			/* Reset weak signal flag */
			stfm1000->weak_signal = 0;
			stfm1000->prev_pilot_present = stfm1000->pilot_present;
		}

	}

	if (stfm1000->revid == STFM1000_CHIP_REV_TA2) {

		/* read AGC_STAT register */
		ret = stfm1000_read(stfm1000, STFM1000_AGC_STAT, &tmp);
		if (ret != 0)
			return ret;

		agc_out = (tmp & STFM1000_AGCOUT_STAT_MASK) >>
			STFM1000_AGCOUT_STAT_SHIFT;
		lna_rms = (tmp & STFM1000_LNA_RMS_MASK) >>
			STFM1000_LNA_RMS_SHIFT;

		ret = stfm1000_read(stfm1000, STFM1000_AGC_CONTROL1, &tmp);
		if (ret != 0)
			return ret;

		/* extract LNATH */
		lna_th = (tmp & STFM1000_B2_LNATH_MASK) >>
			STFM1000_B2_LNATH_SHIFT;

		if (lna_rms > lna_th && agc_out <= 1) {

			ret = stfm1000_write_masked(stfm1000, STFM1000_LNA,
				STFM1000_USEATTEN(1), STFM1000_USEATTEN_MASK);
			if (ret != 0)
				return ret;

		} else if (agc_out > 15) {

			ret = stfm1000_write_masked(stfm1000, STFM1000_LNA,
				STFM1000_USEATTEN(0), STFM1000_USEATTEN_MASK);
			if (ret != 0)
				return ret;
		}
	}

	/* disable buffered writes */
	ret = stfm1000_clear_bits(stfm1000, STFM1000_DATAPATH,
		STFM1000_DB_ACCEPT);
	if (ret != 0)
		return ret;

	return ret;
}

static int Is_Station(struct stfm1000 *stfm1000)
{
	u32 tmp, rssi_dc_est, tone_data;
	u16 rssi_mantissa, rssi_exponent, rssi_decoded;
	u16 prssi;
	s16 mpx_dc;
	int rssi_log;

	/* Get Rssi register readings from STFM1000 */
	stfm1000_read(stfm1000, STFM1000_RSSI_TONE, &tmp);
	rssi_dc_est = tmp & 0xffff;
	tone_data = (tmp >> 16) & 0x0fff;

	rssi_mantissa = (rssi_dc_est & 0xffe0) >> 5;	/* 11Msb */
	rssi_exponent = rssi_dc_est & 0x001f;	/* 5 lsb */
	rssi_decoded = (u32)rssi_mantissa << rssi_exponent;

	/* Convert Rsst to 10log(Rssi) */
	for (prssi = 20; prssi > 0; prssi--)
		if (rssi_decoded >= (1 << prssi))
			break;

	rssi_log = (3 * rssi_decoded >> prssi) + (3 * prssi - 3);
	/* clamp to positive */
	if (rssi_log < 0)
		rssi_log = 0;
	/* Compensate for errors in truncation/approximation by adding 1 */
	rssi_log++;

	stfm1000->rssi_dc_est_log = rssi_log;
	stfm1000->signal_strength = stfm1000->rssi_dc_est_log;

	/* determine absolute value */
	if (tmp & 0x0800)
		mpx_dc = ((tmp >> 16) & 0x0fff) | 0xf000;
	else
		mpx_dc = (tmp >> 16) & 0x0fff;
	stfm1000->mpx_dc = mpx_dc;
	mpx_dc = mpx_dc < 0 ? -mpx_dc : mpx_dc;

	if (stfm1000->tuning_grid_50KHz)
		stfm1000->is_station = rssi_log > stfm1000->tune_rssi_th;
	else
		stfm1000->is_station = rssi_log > stfm1000->tune_rssi_th &&
			mpx_dc > stfm1000->tune_mpx_dc_th;

	return 0;
}

int Monitor_STFM_AGC(struct stfm1000 *stfm1000)
{
	/* we don't do any AGC for now */
	return 0;
}

static int Take_Down(struct stfm1000 *stfm1000)
{
	Mute_Audio(stfm1000);

	DRI_Off(stfm1000);

	SD_DP_Off(stfm1000);

	return 0;
}

static int Bring_Up(struct stfm1000 *stfm1000)
{
	SD_DP_On(stfm1000);

	SD_Optimize_Channel(stfm1000);

	DRI_On(stfm1000);

	Unmute_Audio(stfm1000);

	if (stfm1000->rds_enable)
		stfm1000_rds_reset(&stfm1000->rds_state);

	stfm1000->rds_sync = stfm1000->rds_enable; /* force sync (if RDS) */
	stfm1000->rds_demod_running = 0;
	stfm1000->rssi_dc_est_log = 0;
	stfm1000->signal_strength = 0;

	stfm1000->next_quality_monitor = jiffies + msecs_to_jiffies(
		stfm1000->quality_monitor_period);
	stfm1000->next_agc_monitor = jiffies + msecs_to_jiffies(
		stfm1000->agc_monitor_period);
	stfm1000->rds_pkt_bad = 0;
	stfm1000->rds_pkt_good = 0;
	stfm1000->rds_pkt_recovered = 0;
	stfm1000->rds_pkt_lost_sync = 0;
	stfm1000->rds_bit_overruns = 0;

	return 0;
}

/* These are not used yet */

static int Lock_Station(struct stfm1000 *stfm1000)
{
	int ret;

	ret = SD_Optimize_Channel(stfm1000);
	if (ret != 0)
		return ret;

	/* AGC monitor start? */

	return ret;
}

static const struct stfm1000_reg sd_unlock_regs[] = {
	STFM1000_REG_SETBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_REG_CLRBITS(PILOTTRACKING, STFM1000_B2_PILOTTRACKING_EN),
	STFM1000_REG_CLRBITS(DATAPATH, STFM1000_DB_ACCEPT),
	STFM1000_END,
};

static int Unlock_Station(struct stfm1000 *stfm1000)
{
	int ret;

	ret = stfm1000_write_regs(stfm1000, sd_unlock_regs);
	return ret;
}

irqreturn_t stfm1000_dri_dma_irq(int irq, void *dev_id)
{
	struct stfm1000 *stfm1000 = dev_id;
	u32 err_mask, irq_mask;
	u32 ctrl;
	int handled = 0;

#ifdef CONFIG_ARCH_STMP37XX
	err_mask = 1 << (16 + stfm1000->dma_ch);
#endif
#ifdef CONFIG_ARCH_STMP378X
	err_mask = 1 << stfm1000->dma_ch;
#endif
	irq_mask = 1 << stfm1000->dma_ch;

#ifdef CONFIG_ARCH_STMP37XX
	ctrl = HW_APBX_CTRL1_RD();
#endif
#ifdef CONFIG_ARCH_STMP378X
	ctrl = HW_APBX_CTRL2_RD();
#endif

	if (ctrl & err_mask) {
		handled = 1;
		printk(KERN_WARNING "%s: DMA audio channel %d error\n",
			__func__, stfm1000->dma_ch);
#ifdef CONFIG_ARCH_STMP37XX
		HW_APBX_CTRL1_CLR(err_mask);
#endif
#ifdef CONFIG_ARCH_STMP378X
		HW_APBX_CTRL2_CLR(err_mask);
#endif
	}

	if (HW_APBX_CTRL1_RD() & irq_mask) {
		handled = 1;
		stmp3xxx_dma_clear_interrupt(stfm1000->dma_ch);

		if (stfm1000->alsa_initialized) {
			BUG_ON(stfm1000_alsa_ops->dma_irq == NULL);
			(*stfm1000_alsa_ops->dma_irq)(stfm1000);
		}
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}
EXPORT_SYMBOL(stfm1000_dri_dma_irq);

irqreturn_t stfm1000_dri_attn_irq(int irq, void *dev_id)
{
	struct stfm1000 *stfm1000 = dev_id;
	int handled = 1;
	u32 mask;

	(void)stfm1000;
	mask = HW_DRI_CTRL_RD();
	mask &= BM_DRI_CTRL_OVERFLOW_IRQ | BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ |
		BM_DRI_CTRL_ATTENTION_IRQ;

	HW_DRI_CTRL_CLR(mask);

	printk(KERN_INFO "DRI_ATTN:%s%s%s\n",
			(mask & BM_DRI_CTRL_OVERFLOW_IRQ) ? " OV" : "",
			(mask & BM_DRI_CTRL_PILOT_SYNC_LOSS_IRQ) ? " SL" : "",
			(mask & BM_DRI_CTRL_ATTENTION_IRQ) ? " AT" : "");

	if (stfm1000->alsa_initialized) {
		BUG_ON(stfm1000_alsa_ops->attn_irq == NULL);
		(*stfm1000_alsa_ops->attn_irq)(stfm1000);
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}
EXPORT_SYMBOL(stfm1000_dri_attn_irq);

void stfm1000_decode_block(struct stfm1000 *stfm1000, const s16 *src, s16 *dst,
	int count)
{
	int i;

	if (stfm1000->mute) {
		memset(dst, 0, count * sizeof(s16) * 2);
		return;

	}

	for (i = 0; i < count; i++, dst += 2, src += 4) {

		stfm1000_filter_decode(&stfm1000->filter_parms,
			src[0],	src[1], src[2]);

		dst[0] = stfm1000_filter_value_left(&stfm1000->filter_parms);
		dst[1] = stfm1000_filter_value_right(&stfm1000->filter_parms);
	}

	stfm1000->rssi = stfm1000->filter_parms.RssiDecoded;
	stfm1000->stereo = stfm1000->pilot_present &&
		!stfm1000->filter_parms.pCoefForcedMono;

	/* RDS processing */
	if (stfm1000->rds_demod_running) {
		/* rewind */
		src -= count * 4;
		stfm1000_rds_demod(&stfm1000->rds_state, src, count);
	}

}
EXPORT_SYMBOL(stfm1000_decode_block);

void stfm1000_take_down(struct stfm1000 *stfm1000)
{
	mutex_lock(&stfm1000->state_lock);
	stfm1000->active = 0;
	Take_Down(stfm1000);
	mutex_unlock(&stfm1000->state_lock);
}
EXPORT_SYMBOL(stfm1000_take_down);

void stfm1000_bring_up(struct stfm1000 *stfm1000)
{
	mutex_lock(&stfm1000->state_lock);

	stfm1000->active = 1;

	stfm1000_filter_reset(&stfm1000->filter_parms);

	Bring_Up(stfm1000);

	mutex_unlock(&stfm1000->state_lock);
}
EXPORT_SYMBOL(stfm1000_bring_up);

void stfm1000_tune_current(struct stfm1000 *stfm1000)
{
	mutex_lock(&stfm1000->state_lock);
	sw_tune(stfm1000, stfm1000->freq);
	mutex_unlock(&stfm1000->state_lock);
}
EXPORT_SYMBOL(stfm1000_tune_current);

/* Alternate ZIF Tunings to avoid EMI */
const struct stfm1000_tune1
stfm1000_board_emi_tuneups[STFM1000_FREQUENCY_100KHZ_RANGE] = {
#undef TUNE_ENTRY
#define TUNE_ENTRY(f, t1, sd) \
	[(f) - STFM1000_FREQUENCY_100KHZ_MIN] = \
		{ .tune1 = (t1), .sdnom = (sd) }
	TUNE_ENTRY(765, 0x84030, 0x1BF5E50D),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(780, 0x84240, 0x1BA5162F),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(795, 0x84250, 0x1C2D2F39),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(810, 0x84460, 0x1BDD207E),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(825, 0x84470, 0x1C6138CD),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(839, 0xC4680, 0x1C11F704),	/* 061215 Jon, IF +100kHz */
	TUNE_ENTRY(840, 0x84680, 0x1c11f704),
	TUNE_ENTRY(855, 0x84890, 0x1BC71C71),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(870, 0x848A0, 0x1C43DE10),	/* 061215 Jon, IF +0kHz */
	TUNE_ENTRY(885, 0x84AB0, 0x1BF9B021),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(899, 0xC4CC0, 0x1BB369A9),	/* 061025 Arthur, IF +100kHz */
	TUNE_ENTRY(900, 0x84CC0, 0x1BB369A9),	/* 061025 Arthur, IF 0kHz */
	TUNE_ENTRY(915, 0x84CD0, 0x1C299A5B),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(930, 0x84ee0, 0x1be3e6aa),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(945, 0x84ef0, 0x1c570f8b),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(959, 0xC5100, 0x1c11f704),
	TUNE_ENTRY(960, 0x85100, 0x1c11f704),
	TUNE_ENTRY(975, 0x85310, 0x1bd03d57),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(990, 0x85320, 0x1c3dc822),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(1005, 0x85530, 0x1bfc93ff),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(1019, 0xC5740, 0x1BBE683C),	/* 061025 Arthur, IF +100kHz */
	TUNE_ENTRY(1020, 0x85740, 0x1bbe683c),	/* 061025 Arthur, IF +0kHz */
	TUNE_ENTRY(1035, 0x85750, 0x1c26dab6),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(1050, 0x85960, 0x1be922b4),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(1065, 0x85970, 0x1c4f357c),	/* 061101 Arthur, IF +0kHz */
	TUNE_ENTRY(1079, 0xC5B80, 0x1c11f704),
	TUNE_ENTRY(1080, 0x85B80, 0x1c11f704),
#undef TUNE_ENTRY
};

static const struct stfm1000_tune1 *stfm1000_board_emi_tune(int freq100)
{
	const struct stfm1000_tune1 *tune1;

	if ((unsigned int)(freq100 - STFM1000_FREQUENCY_100KHZ_MIN) >=
			STFM1000_FREQUENCY_100KHZ_RANGE)
		return NULL;

	tune1 = &stfm1000_board_emi_tuneups[freq100 -
		STFM1000_FREQUENCY_100KHZ_MIN];
	if (tune1->tune1 == 0 && tune1->sdnom == 0)
		return NULL;
	return tune1;
}

/* freq in kHz */
static int sw_tune(struct stfm1000 *stfm1000, u32 freq)
{
	u32 freq100 = freq / 100;
	int tune_cap;
	int i2s_clock;
	int mix_reg;
	int if_freq, fe_freq;
	u32 tune1, sdnom, agc1;
	const struct stfm1000_tune1 *tp;
	int ret;

	if_freq = 0;
	mix_reg = 1;
	switch (mix_reg) {
	case 0: if_freq = -2; break;
	case 1: if_freq = -1; break;
	case 2: if_freq =  0; break;
	case 3: if_freq =  1; break;
	case 4: if_freq =  2; break;
	}

	/* handle board specific EMI tuning */
	tp = stfm1000_board_emi_tune(freq100);
	if (tp != NULL) {
		tune1 = tp->tune1;
		sdnom = tp->sdnom;
	} else {
		fe_freq = freq100 + if_freq;

		/* clamp into range */
		if (fe_freq < STFM1000_FREQUENCY_100KHZ_MIN)
			fe_freq = STFM1000_FREQUENCY_100KHZ_MIN;
		else if (fe_freq > STFM1000_FREQUENCY_100KHZ_MAX)
			fe_freq = STFM1000_FREQUENCY_100KHZ_MAX;

		tp = &stfm1000_tune1_table[fe_freq -
			STFM1000_FREQUENCY_100KHZ_MIN];

		/* bits [14:0], [20:18] */
		tune1 = (tp->tune1 & 0x7fff) | (mix_reg << 18);
		sdnom = tp->sdnom;
	}

	agc1 = stfm1000->revid == STFM1000_CHIP_REV_TA2 ? 0x0400 : 0x2200;

	ret = stfm1000_write_masked(stfm1000, STFM1000_AGC_CONTROL1,
			agc1, 0x3f00);
	if (ret != 0)
		goto err;

	ret = stfm1000_write_masked(stfm1000, STFM1000_TUNE1, tune1,
		0xFFFF7FFF);	/* do not set bit-15 */
	if (ret != 0)
		goto err;

	/* keep this around */
	stfm1000->sdnominal_pivot = sdnom;

	ret = stfm1000_write(stfm1000, STFM1000_SDNOMINAL, sdnom);
	if (ret != 0)
		goto err;

	/* fix for seek-not-stopping on alternate tunings */
	ret = stfm1000_set_bits(stfm1000, STFM1000_DATAPATH,
			STFM1000_DB_ACCEPT);
	if (ret != 0)
		goto err;

	ret = stfm1000_clear_bits(stfm1000, STFM1000_DATAPATH,
			STFM1000_DB_ACCEPT);
	if (ret != 0)
		goto err;

	ret = stfm1000_set_bits(stfm1000, STFM1000_INITIALIZATION2,
			STFM1000_DRI_CLK_EN);
	if (ret != 0)
		goto err;

	/* 6MHz spur fix */
	if ((freq100 >=  778 && freq100 <=  782) ||
	    (freq100 >=  838 && freq100 <=  842) ||
	    (freq100 >=  898 && freq100 <=  902) ||
	    (freq100 >=  958 && freq100 <=  962) ||
	    (freq100 >= 1018 && freq100 <= 1022) ||
	    (freq100 >= 1078 && freq100 <= 1080))
		i2s_clock = 5;  /* 4.8MHz */
	else
		i2s_clock = 4;

	ret = stfm1000_write_masked(stfm1000, STFM1000_DATAPATH,
		STFM1000_SAI_CLK_DIV(i2s_clock), STFM1000_SAI_CLK_DIV_MASK);
	if (ret != 0)
		goto err;

	ret = stfm1000_set_bits(stfm1000, STFM1000_INITIALIZATION2,
		STFM1000_DRI_CLK_EN);
	if (ret != 0)
		goto err;

	if (tune1 & 0xf)
		ret = stfm1000_set_bits(stfm1000, STFM1000_CLK1,
			STFM1000_ENABLE_TAPDELAYFIX);
	else
		ret = stfm1000_clear_bits(stfm1000, STFM1000_CLK1,
			STFM1000_ENABLE_TAPDELAYFIX);

	if (ret != 0)
		goto err;

	tune_cap = (int)(stfm1000->tune_cap_a_f -
		stfm1000->tune_cap_b_f * freq100);
	if (tune_cap < 4)
		tune_cap = 4;
	ret = stfm1000_write_masked(stfm1000, STFM1000_LNA,
		STFM1000_ANTENNA_TUNECAP(tune_cap),
		STFM1000_ANTENNA_TUNECAP_MASK);
	if (ret != 0)
		goto err;

	/* set signal strenth to 0 */
	/* stfm1000_dcdc_update(); */

	/* cmp_rds_setRdsStatus(0) */
	/* cmp_rds_ResetGroupCallbacks(); */
	stfm1000->freq = freq;

	return 0;
err:
	return -1;
}

static const struct v4l2_queryctrl radio_qctrl[] = {
	{
		.id		= V4L2_CID_AUDIO_MUTE,
		.name		= "Mute",
		.minimum	= 0,
		.maximum	= 1,
		.default_value	= 1,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	},
};

static int vidioc_querycap(struct file *file, void *priv,
				struct v4l2_capability *v)
{
	strlcpy(v->driver, "radio-stfm1000", sizeof(v->driver));
	strlcpy(v->card, "STFM1000 Radio", sizeof(v->card));
	sprintf(v->bus_info, "i2c");
	v->version = KERNEL_VERSION(0, 0, 1);
	v->capabilities = V4L2_CAP_TUNER;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);
	u32 tmp, rssi_dc_est, tone_data;
	u16 rssi_mantissa, rssi_exponent, rssi_decoded;
	u16 prssi;
	s16 mpx_dc;
	int rssi_log;
	int ret;

	if (v->index > 0)
		return -EINVAL;

	mutex_lock(&stfm1000->state_lock);

	strcpy(v->name, "FM");
	v->type = V4L2_TUNER_RADIO;
	v->rangelow = (u32)(87.5 * 16000);
	v->rangehigh = (u32)(108 * 16000);
	v->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	v->capability = V4L2_TUNER_CAP_LOW;
	v->audmode = V4L2_TUNER_MODE_STEREO;
	v->signal = 0; /* tr_getsigstr(); */

	msleep(50);

	ret = stfm1000_read(stfm1000, STFM1000_RSSI_TONE, &tmp);
	if (ret != 0)
		goto out;

	rssi_dc_est = tmp & 0xffff;
	tone_data = (tmp >> 16) & 0x0fff;

	rssi_mantissa = (rssi_dc_est & 0xffe0) >> 5;	/* 11Msb */
	rssi_exponent = rssi_dc_est & 0x001f;	/* 5 lsb */
	rssi_decoded = (u32)rssi_mantissa << rssi_exponent;

	/* Convert Rsst to 10log(Rssi) */
	for (prssi = 20; prssi > 0; prssi--)
		if (rssi_decoded >= (1 << prssi))
			break;

	rssi_log = (3 * rssi_decoded >> prssi) + (3 * prssi - 3);
	/* clamp to positive */
	if (rssi_log < 0)
		rssi_log = 0;
	/* Compensate for errors in truncation/approximation by adding 1 */
	rssi_log++;

	stfm1000->rssi_dc_est_log = rssi_log;
	stfm1000->signal_strength = stfm1000->rssi_dc_est_log;

	/* determine absolute value */
	if (tmp & 0x0800)
		mpx_dc = ((tmp >> 16) & 0x0fff) | 0xf000;
	else
		mpx_dc = (tmp >> 16) & 0x0fff;
	stfm1000->mpx_dc = mpx_dc;
	mpx_dc = mpx_dc < 0 ? -mpx_dc : mpx_dc;

	v->signal = rssi_decoded & 0xffff;

out:
	mutex_unlock(&stfm1000->state_lock);

	return ret;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	(void)stfm1000;

	if (v->index > 0)
		return -EINVAL;

	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	mutex_lock(&stfm1000->state_lock);

	/* convert from the crazy linux value to our decimal based values */
	stfm1000->freq = (u32)div_u64((u64)(125 * (u64)f->frequency), 2000);

	if (stfm1000->active)
		Take_Down(stfm1000);

	sw_tune(stfm1000, stfm1000->freq);

	if (stfm1000->active)
		Bring_Up(stfm1000);

	mutex_unlock(&stfm1000->state_lock);

	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	f->type = V4L2_TUNER_RADIO;
	f->frequency = stfm1000->freq * 16;

	return 0;
}

static int vidioc_queryctrl(struct file *file, void *priv,
				struct v4l2_queryctrl *qc)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);
	int i;

	(void)stfm1000;

	for (i = 0; i < ARRAY_SIZE(radio_qctrl); i++) {
		if (qc->id && qc->id == radio_qctrl[i].id) {
			memcpy(qc, &radio_qctrl[i], sizeof(*qc));
			return 0;
		}
	}
	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	switch (ctrl->id) {

	case V4L2_CID_AUDIO_MUTE:
		ctrl->value = stfm1000->mute;
		return 0;

	}
	return -EINVAL;
}

static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);
	int ret;

	mutex_lock(&stfm1000->state_lock);

	ret = -EINVAL;

	switch (ctrl->id) {

	case V4L2_CID_AUDIO_MUTE:
		stfm1000->mute = ctrl->value;
		ret = 0;
		break;
	}

	mutex_unlock(&stfm1000->state_lock);

	return ret;
}

static int vidioc_g_audio(struct file *file, void *priv,
				struct v4l2_audio *a)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	(void)stfm1000;

	if (a->index > 1)
		return -EINVAL;

	strcpy(a->name, "Radio");
	a->capability = V4L2_AUDCAP_STEREO;
	return 0;
}

static int vidioc_s_audio(struct file *file, void *priv,
					struct v4l2_audio *a)
{
	if (a->index > 1)
		return -EINVAL;
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	(void)stfm1000;

	*i = 0;

	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	(void)stfm1000;

	if (i != 0)
		return -EINVAL;

	return 0;
}

const struct v4l2_ioctl_ops stfm_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_g_tuner		= vidioc_g_tuner,
	.vidioc_s_tuner		= vidioc_s_tuner,
	.vidioc_g_frequency	= vidioc_g_frequency,
	.vidioc_s_frequency	= vidioc_s_frequency,
	.vidioc_queryctrl	= vidioc_queryctrl,
	.vidioc_g_ctrl		= vidioc_g_ctrl,
	.vidioc_s_ctrl		= vidioc_s_ctrl,
	.vidioc_g_audio		= vidioc_g_audio,
	.vidioc_s_audio		= vidioc_s_audio,
	.vidioc_g_input		= vidioc_g_input,
	.vidioc_s_input		= vidioc_s_input,
};

static int stfm1000_open(struct inode *inode, struct file *file)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	mutex_lock(&stfm1000->state_lock);
	stfm1000->users = 1;
	mutex_unlock(&stfm1000->state_lock);

	return 0;
}
static int stfm1000_close(struct inode *inode, struct file *file)
{
	struct stfm1000 *stfm1000 = stfm1000_from_file(file);

	if (!stfm1000)
		return -ENODEV;

	stfm1000->users = 0;
	if (stfm1000->removed)
		kfree(stfm1000);
	return 0;
}

static const struct file_operations stfm1000_fops = {
	.owner		= THIS_MODULE,
	.open		= stfm1000_open,
	.release	= stfm1000_close,
	.ioctl		= video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= v4l_compat_ioctl32,
#endif
	.llseek		= no_llseek,
};

/* sysfs */

#define STFM1000_RO_ATTR(var) \
static ssize_t stfm1000_show_ ## var(struct device *d, \
	struct device_attribute *attr, char *buf) \
{ \
	struct i2c_client *client = to_i2c_client(d); \
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client); \
	return sprintf(buf, "%d\n", stfm1000->var); \
} \
static DEVICE_ATTR(var, 0444, stfm1000_show_ ##var, NULL)

#define STFM1000_RW_ATTR(var) \
static ssize_t stfm1000_show_ ## var(struct device *d, \
	struct device_attribute *attr, char *buf) \
{ \
	struct i2c_client *client = to_i2c_client(d); \
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client); \
	return sprintf(buf, "%u\n", stfm1000->var); \
} \
static ssize_t stfm1000_store_ ## var(struct device *d, \
	struct device_attribute *attr, const char *buf, size_t size) \
{ \
	struct i2c_client *client = to_i2c_client(d); \
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client); \
	unsigned long v; \
	\
	strict_strtoul(buf, 0, &v); \
	stfm1000_commit_ ## var(stfm1000, v); \
	return size; \
} \
static DEVICE_ATTR(var, 0644, stfm1000_show_ ##var, stfm1000_store_ ##var)

#define STFM1000_RW_ATTR_SIMPLE(var) \
static void stfm1000_commit_ ## var(struct stfm1000 *stfm1000, \
	unsigned long value) \
{ \
	stfm1000->var = value; \
} \
STFM1000_RW_ATTR(var)

STFM1000_RO_ATTR(weak_signal);
STFM1000_RO_ATTR(pilot_present);
STFM1000_RO_ATTR(stereo);
STFM1000_RO_ATTR(rssi);
STFM1000_RO_ATTR(mpx_dc);
STFM1000_RO_ATTR(signal_strength);
STFM1000_RW_ATTR_SIMPLE(rds_signal_th);
STFM1000_RO_ATTR(rds_present);
STFM1000_RO_ATTR(is_station);

static void stfm1000_commit_georegion(struct stfm1000 *stfm1000,
	unsigned long value)
{
	/* don't do anything for illegal region */
	if (value != 0 && value != 1)
		return;

	mutex_lock(&stfm1000->state_lock);

	stfm1000->georegion = value;
	if (stfm1000->georegion == 0)
		stfm1000_clear_bits(stfm1000, STFM1000_INITIALIZATION2,
			STFM1000_DEEMPH_50_75B);
	else
		stfm1000_set_bits(stfm1000, STFM1000_INITIALIZATION2,
			STFM1000_DEEMPH_50_75B);

	mutex_unlock(&stfm1000->state_lock);
}
STFM1000_RW_ATTR(georegion);

static void stfm1000_commit_freq(struct stfm1000 *stfm1000,
	unsigned long value)
{
	mutex_lock(&stfm1000->state_lock);

	/* clamp */
	if (value < STFM1000_FREQUENCY_100KHZ_MIN * 100)
		value = STFM1000_FREQUENCY_100KHZ_MIN * 100;
	else if (value > STFM1000_FREQUENCY_100KHZ_MAX * 100)
		value = STFM1000_FREQUENCY_100KHZ_MAX * 100;

	stfm1000->freq = value;

	if (stfm1000->active)
		Take_Down(stfm1000);

	sw_tune(stfm1000, stfm1000->freq);

	if (stfm1000->active)
		Bring_Up(stfm1000);

	mutex_unlock(&stfm1000->state_lock);
}
STFM1000_RW_ATTR(freq);

static void stfm1000_commit_mute(struct stfm1000 *stfm1000,
	unsigned long value)
{
	stfm1000->mute = !!value;
}
STFM1000_RW_ATTR(mute);

static void stfm1000_commit_force_mono(struct stfm1000 *stfm1000,
	unsigned long value)
{
	stfm1000->force_mono = !!value;
	/* set force mono parameters for the filter */
	stfm1000->filter_parms.pCoefForcedMono = stfm1000->force_mono;

	/* yeah, I know, it's stupid */
	stfm1000->rds_state.demod.pCoefForcedMono =
		stfm1000->filter_parms.pCoefForcedMono;
}
STFM1000_RW_ATTR(force_mono);

STFM1000_RW_ATTR_SIMPLE(monitor_period);
STFM1000_RW_ATTR_SIMPLE(quality_monitor);
STFM1000_RW_ATTR_SIMPLE(quality_monitor_period);
STFM1000_RW_ATTR_SIMPLE(agc_monitor_period);
STFM1000_RW_ATTR_SIMPLE(tune_rssi_th);
STFM1000_RW_ATTR_SIMPLE(tune_mpx_dc_th);

static void stfm1000_commit_rds_enable(struct stfm1000 *stfm1000,
	unsigned long value)
{
	/* don't do anything for illegal values (or for not TB2) */
	if ((value != 0 && value != 1) ||
		stfm1000->revid == STFM1000_CHIP_REV_TA2)
		return;

	mutex_lock(&stfm1000->state_lock);

	stfm1000->rds_enable = value;
	if (stfm1000->rds_enable == 0)
		stfm1000_clear_bits(stfm1000, STFM1000_INITIALIZATION2,
			STFM1000_RDS_ENABLE);
	else
		stfm1000_set_bits(stfm1000, STFM1000_INITIALIZATION2,
			STFM1000_RDS_ENABLE);

	mutex_unlock(&stfm1000->state_lock);
}
STFM1000_RW_ATTR(rds_enable);

static void stfm1000_commit_rds_sync(struct stfm1000 *stfm1000,
	unsigned long value)
{
	stfm1000->rds_sync = stfm1000->rds_enable && !!value;
}
STFM1000_RW_ATTR(rds_sync);

STFM1000_RW_ATTR_SIMPLE(rds_pkt_good);
STFM1000_RW_ATTR_SIMPLE(rds_pkt_bad);
STFM1000_RW_ATTR_SIMPLE(rds_pkt_recovered);
STFM1000_RW_ATTR_SIMPLE(rds_pkt_lost_sync);
STFM1000_RW_ATTR_SIMPLE(rds_bit_overruns);
STFM1000_RW_ATTR_SIMPLE(rds_info);

static void stfm1000_commit_rds_sdnominal_adapt(struct stfm1000 *stfm1000,
	unsigned long value)
{
	stfm1000->rds_sdnominal_adapt = !!value;
	stfm1000->rds_state.demod.sdnom_adapt = stfm1000->rds_sdnominal_adapt;
}
STFM1000_RW_ATTR(rds_sdnominal_adapt);

static void stfm1000_commit_rds_phase_pop(struct stfm1000 *stfm1000,
	unsigned long value)
{
	stfm1000->rds_phase_pop = !!value;
	stfm1000->rds_state.demod.PhasePoppingEnabled =
		stfm1000->rds_phase_pop;
}
STFM1000_RW_ATTR(rds_phase_pop);

STFM1000_RW_ATTR_SIMPLE(tuning_grid_50KHz);

static ssize_t stfm1000_show_rds_ps(struct device *d,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(d);
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client);
	char ps[9];

	if (stfm1000_rds_get_ps(&stfm1000->rds_state, ps, sizeof(ps)) <= 0)
		ps[0] = '\0';

	return sprintf(buf, "%s\n", ps);
}
static DEVICE_ATTR(rds_ps, 0444, stfm1000_show_rds_ps, NULL);

static ssize_t stfm1000_show_rds_text(struct device *d,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(d);
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client);
	char text[65];

	if (stfm1000_rds_get_text(&stfm1000->rds_state, text,
		sizeof(text)) <= 0)
		text[0] = '\0';

	return sprintf(buf, "%s\n", text);
}
static DEVICE_ATTR(rds_text, 0444, stfm1000_show_rds_text, NULL);

static struct device_attribute *stfm1000_attrs[] = {
	&dev_attr_agc_monitor_period,
	&dev_attr_force_mono,
	&dev_attr_freq,
	&dev_attr_georegion,
	&dev_attr_is_station,
	&dev_attr_monitor_period,
	&dev_attr_mpx_dc,
	&dev_attr_mute,
	&dev_attr_pilot_present,
	&dev_attr_quality_monitor,
	&dev_attr_quality_monitor_period,
	&dev_attr_rds_bit_overruns,
	&dev_attr_rds_enable,
	&dev_attr_rds_info,
	&dev_attr_rds_phase_pop,
	&dev_attr_rds_pkt_bad,
	&dev_attr_rds_pkt_good,
	&dev_attr_rds_pkt_lost_sync,
	&dev_attr_rds_pkt_recovered,
	&dev_attr_rds_present,
	&dev_attr_rds_ps,
	&dev_attr_rds_sdnominal_adapt,
	&dev_attr_rds_signal_th,
	&dev_attr_rds_sync,
	&dev_attr_rds_text,
	&dev_attr_rssi,
	&dev_attr_signal_strength,
	&dev_attr_stereo,
	&dev_attr_tune_mpx_dc_th,
	&dev_attr_tune_rssi_th,
	&dev_attr_tuning_grid_50KHz,
	&dev_attr_weak_signal,
	NULL,
};

/* monitor thread */

static void rds_process(struct stfm1000 *stfm1000)
{
	int count, bit;
	int mix_reg, sdnominal_reg;
	u32 sdnom, sdnom_new, limit;
	u8 buf[8];

	if (!stfm1000->rds_enable)
		return;

	if (stfm1000->rds_sync &&
		stfm1000->rssi_dc_est_log > stfm1000->rds_signal_th) {
		if (stfm1000->rds_info)
			printk(KERN_INFO "RDS: sync\n");
		stfm1000_rds_reset(&stfm1000->rds_state);
		stfm1000->rds_demod_running = 1;
		stfm1000->rds_sync = 0;
	}

	if (!stfm1000->rds_demod_running)
		return;

	/* process mix reg requests */
	spin_lock_irq(&stfm1000->rds_lock);
	mix_reg = stfm1000_rds_mix_msg_get(&stfm1000->rds_state);
	spin_unlock_irq(&stfm1000->rds_lock);

	if (mix_reg != -1) {

		if (stfm1000->rds_info)
			printk(KERN_INFO "RDS: new RDS_MIXOFFSET %d\n",
				mix_reg & 1);

		/* update register */
		if (mix_reg & 1)
			stfm1000_set_bits(stfm1000, STFM1000_INITIALIZATION2,
				STFM1000_RDS_MIXOFFSET);
		else
			stfm1000_clear_bits(stfm1000, STFM1000_INITIALIZATION2,
				STFM1000_RDS_MIXOFFSET);

		/* signal it's processed */
		spin_lock_irq(&stfm1000->rds_lock);
		stfm1000_rds_mix_msg_processed(&stfm1000->rds_state, mix_reg);
		spin_unlock_irq(&stfm1000->rds_lock);
	}

	/* process sdnominal reg requests */
	spin_lock_irq(&stfm1000->rds_lock);
	sdnominal_reg = stfm1000_rds_sdnominal_msg_get(&stfm1000->rds_state);
	spin_unlock_irq(&stfm1000->rds_lock);

	/* any change? */
	if (sdnominal_reg != 0) {

		stfm1000_read(stfm1000, STFM1000_SDNOMINAL, &sdnom);

		sdnom_new = sdnom + sdnominal_reg;

		/* Limit SDNOMINAL to within 244 ppm of its ideal value */
		limit = stfm1000->sdnominal_pivot +
			(stfm1000->sdnominal_pivot >> 12);
		if (sdnom_new > limit)
			sdnom_new = limit;

		limit = stfm1000->sdnominal_pivot -
			(stfm1000->sdnominal_pivot >> 12);
		if (sdnom_new < limit)
			sdnom_new = limit;

		/* write the register */
		stfm1000_write(stfm1000, STFM1000_SDNOMINAL, sdnom_new);

		/* signal it's processed */
		spin_lock_irq(&stfm1000->rds_lock);
		stfm1000_rds_sdnominal_msg_processed(&stfm1000->rds_state,
			sdnominal_reg);
		spin_unlock_irq(&stfm1000->rds_lock);
	}

	/* pump bits out & pass them to the process function */
	spin_lock_irq(&stfm1000->rds_lock);
	while (stfm1000_rds_bits_available(&stfm1000->rds_state) > 128) {
		count = 0;
		while (count++ < 128 &&
			(bit = stmf1000_rds_get_bit(
				&stfm1000->rds_state)) >= 0) {
			spin_unlock_irq(&stfm1000->rds_lock);

			/* push bit for packet processing */
			stfm1000_rds_packet_bit(&stfm1000->rds_state, bit);

			spin_lock_irq(&stfm1000->rds_lock);
		}
	}
	spin_unlock_irq(&stfm1000->rds_lock);

	/* now we're free to process non-interrupt related work */
	while (stfm1000_rds_packet_dequeue(&stfm1000->rds_state, buf) == 0) {

		if (stfm1000->rds_info)
			printk(KERN_INFO "RDS-PKT: %02x %02x %02x %02x "
					"%02x %02x %02x %02x\n",
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7]);

		stfm1000_rds_process_packet(&stfm1000->rds_state, buf);
	}

	/* update our own counters */
	stfm1000->rds_pkt_good += stfm1000->rds_state.pkt.good_packets;
	stfm1000->rds_pkt_bad += stfm1000->rds_state.pkt.bad_packets;
	stfm1000->rds_pkt_recovered +=
		stfm1000->rds_state.pkt.recovered_packets;
	stfm1000->rds_pkt_lost_sync +=
		stfm1000->rds_state.pkt.sync_lost_packets;
	stfm1000->rds_bit_overruns +=
		stfm1000->rds_state.demod.RdsDemodSkippedBitCnt;

	/* zero them now */
	stfm1000->rds_state.pkt.good_packets = 0;
	stfm1000->rds_state.pkt.bad_packets = 0;
	stfm1000->rds_state.pkt.recovered_packets = 0;
	stfm1000->rds_state.pkt.sync_lost_packets = 0;
	stfm1000->rds_state.demod.RdsDemodSkippedBitCnt = 0;

	/* reset requested from RDS handler? */
	if (stfm1000_rds_get_reset_req(&stfm1000->rds_state)) {
		if (stfm1000->rds_info)
			printk(KERN_INFO "RDS: reset requested\n");
		stfm1000_rds_reset(&stfm1000->rds_state);

		stfm1000->rds_sync = stfm1000->rds_enable; /* force sync (if RDS) */
		stfm1000->rds_demod_running = 0;
		stfm1000->rssi_dc_est_log = 0;
		stfm1000->signal_strength = 0;
	}
}

void stfm1000_monitor_signal(struct stfm1000 *stfm1000, int bit)
{
	set_bit(bit, &stfm1000->thread_events);
	return wake_up_interruptible(&stfm1000->thread_wait);
}

static int stfm1000_monitor_thread(void *data)
{
	struct stfm1000 *stfm1000 = data;
	int ret;

	printk(KERN_INFO "stfm1000: monitor thread started\n");

	set_freezable();

	/* Hmm, linux becomes *very* unhappy without this ... */
	while (!kthread_should_stop()) {

		ret = wait_event_interruptible_timeout(stfm1000->thread_wait,
				stfm1000->thread_events == 0,
				msecs_to_jiffies(stfm1000->monitor_period));

		stfm1000->thread_events = 0;

		if (kthread_should_stop())
			break;

		try_to_freeze();

		mutex_lock(&stfm1000->state_lock);

		/* we must be active */
		if (!stfm1000->active)
			goto next;

		if (stfm1000->rds_enable)
			rds_process(stfm1000);

		/* perform quality monitor */
		if (time_after_eq(jiffies, stfm1000->next_quality_monitor)) {

			/* full quality monitor? */
			if (stfm1000->quality_monitor)
				Monitor_STFM_Quality(stfm1000);
			else	/* simple */
				Is_Station(stfm1000);

			while (time_after_eq(jiffies,
					stfm1000->next_quality_monitor))
				stfm1000->next_quality_monitor +=
					msecs_to_jiffies(
					stfm1000->quality_monitor_period);
		}

		/* perform AGC monitor (if enabled) */
		if (stfm1000->agc_monitor && time_after_eq(jiffies,
				stfm1000->next_agc_monitor)) {
			Monitor_STFM_AGC(stfm1000);
			while (time_after_eq(jiffies,
					stfm1000->next_agc_monitor))
				stfm1000->next_agc_monitor +=
					msecs_to_jiffies(
						stfm1000->agc_monitor_period);
		}
next:
		mutex_unlock(&stfm1000->state_lock);
	}

	printk(KERN_INFO "stfm1000: monitor thread stopped\n");

	return 0;
}

static u64 stfm1000_dma_mask = DMA_32BIT_MASK;

static int stfm1000_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct device *dev;
	struct stfm1000 *stfm1000;
	struct video_device *vd;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;
	u32 id;
	const char *idtxt;
	int i;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			"I2C doesn't support I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -EIO;
	}

	/* make sure the dma masks are set correctly */
	dev = &client->dev;
	if (!dev->dma_mask)
		dev->dma_mask = &stfm1000_dma_mask;
	if (!dev->coherent_dma_mask)
		dev->coherent_dma_mask = DMA_32BIT_MASK;

	stfm1000 = kzalloc(sizeof(*stfm1000), GFP_KERNEL);
	if (!stfm1000)
		return -ENOMEM;

	stfm1000->client = client;
	i2c_set_clientdata(client, stfm1000);

	mutex_init(&stfm1000->xfer_lock);
	mutex_init(&stfm1000->state_lock);

	vd = &stfm1000->radio;

	strcpy(vd->name, "stfm1000");
	vd->vfl_type		= VID_TYPE_TUNER;
	vd->fops		= &stfm1000_fops;
	vd->ioctl_ops		= &stfm_ioctl_ops;

	/* vd->debug = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG; */

	vd->parent = &client->dev;

	ret = video_register_device(vd, VFL_TYPE_RADIO, -1);
	if (ret != 0) {
		dev_warn(&adapter->dev,
			 "Cannot register radio device\n");
		goto out;
	}

	spin_lock_init(&stfm1000->rds_lock);

	stfm1000_setup_reg_set(stfm1000);

	/* stfm1000->dbgflg |= STFM1000_DBGFLG_I2C; */

	ret = stfm1000_read(stfm1000, STFM1000_CHIPID, &id);
	if (ret < 0) {
		dev_warn(&adapter->dev,
			 "Cannot read ID register\n");
		goto out;
	}
	stfm1000->revid = id & 0xff;

	/* NOTE: the tables are precalculated */
	stfm1000->tune_rssi_th = 28;
	stfm1000->tune_mpx_dc_th = 300;
	stfm1000->adj_chan_th = 100;
	stfm1000->pilot_est_th = 25;
	stfm1000->agc_monitor = 0;	/* AGC monitor disabled */
	stfm1000->quality_monitor = 1;
	stfm1000->weak_signal = 0;
	stfm1000->prev_pilot_present = 0;
	stfm1000->tune_cap_a_f = (u32)(72.4 * 65536);
	stfm1000->tune_cap_b_f = (u32)(0.07 * 65536);

	/* only TB2 supports RDS */
	stfm1000->rds_enable = stfm1000->revid == STFM1000_CHIP_REV_TB2 &&
		rds_enable;
	stfm1000->rds_present = 0;
	stfm1000->rds_signal_th = 33;

	stfm1000->freq = 92600;

	stfm1000->georegion = georegion;
	stfm1000->rssi = 0;
	stfm1000->stereo = 0;
	stfm1000->force_mono = 0;
	stfm1000->monitor_period = 100;
	stfm1000->quality_monitor_period = 1000;
	stfm1000->agc_monitor_period = 200;

	stfm1000->rds_sdnominal_adapt = 0;
	stfm1000->rds_phase_pop = 1;

	/* enable info about RDS */
	stfm1000->rds_info = 0;

	ret = stfm1000_power_up(stfm1000);
	if (ret != 0) {
		printk(KERN_ERR "%s: stfm1000_power_up failed\n",
				__func__);
		goto out;
	}

	if (stfm1000_alsa_ops && stfm1000_alsa_ops->init) {
		ret = (*stfm1000_alsa_ops->init)(stfm1000);
		if (ret != 0)
			goto out;
		stfm1000->alsa_initialized = 1;
	}

	ret = 0;
	for (i = 0; stfm1000_attrs[i]; i++) {
		ret = device_create_file(dev, stfm1000_attrs[i]);
		if (ret)
			break;
	}
	if (ret) {
		while (--i >= 0)
			device_remove_file(dev, stfm1000_attrs[i]);
		goto out;
	}

	/* add it to the list */
	mutex_lock(&devlist_lock);
	stfm1000->idx = stfm1000_devcount++;
	list_add_tail(&stfm1000->devlist, &stfm1000_devlist);
	mutex_unlock(&devlist_lock);

	init_waitqueue_head(&stfm1000->thread_wait);
	stfm1000->thread = kthread_run(stfm1000_monitor_thread, stfm1000,
			"stfm1000-%d", stfm1000->idx);
	if (stfm1000->thread == NULL) {
		printk(KERN_ERR "stfm1000: kthread_run failed\n");
		goto out;
	}

	idtxt = stfm1000_get_rev_txt(stfm1000->revid);
	if (idtxt == NULL)
		printk(KERN_INFO "STFM1000: Loaded for unknown revision id "
			"0x%02x\n", stfm1000->revid);
	else
		printk(KERN_INFO "STFM1000: Loaded for revision %s\n", idtxt);

	return 0;

out:
	kfree(stfm1000);
	return ret;
}

static int stfm1000_remove(struct i2c_client *client)
{
	struct stfm1000 *stfm1000 = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int i;

	kthread_stop(stfm1000->thread);

	for (i = 0; stfm1000_attrs[i]; i++)
		device_remove_file(dev, stfm1000_attrs[i]);

	if (stfm1000->alsa_initialized) {
		BUG_ON(stfm1000_alsa_ops->release == NULL);
		(*stfm1000_alsa_ops->release)(stfm1000);
		stfm1000->alsa_initialized = 0;
	}

	stfm1000_power_down(stfm1000);

	video_unregister_device(&stfm1000->radio);

	mutex_lock(&devlist_lock);
	list_del(&stfm1000->devlist);
	mutex_unlock(&devlist_lock);

	kfree(stfm1000);
	return 0;
}

static const struct i2c_device_id stfm1000_id[] = {
	{ "stfm1000", 0xC0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stfm1000_id);

static struct i2c_driver stfm1000_i2c_driver = {
	.driver = {
		.name = "stfm1000",
	},
	.probe		= stfm1000_probe,
	.remove		= stfm1000_remove,
	.id_table	= stfm1000_id,
};

static int __init
stfm1000_init(void)
{
	/* pull those in */
	(void)Lock_Station;
	(void)Unlock_Station;
	return i2c_add_driver(&stfm1000_i2c_driver);
}

static void __exit
stfm1000_exit(void)
{
	i2c_del_driver(&stfm1000_i2c_driver);

	stfm1000_alsa_ops = NULL;
}

module_init(stfm1000_init);
module_exit(stfm1000_exit);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("A driver for the STFM1000 chip.");
MODULE_LICENSE("GPL");

module_param(georegion, int, 0400);
module_param(rds_enable, int, 0400);
