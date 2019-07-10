// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sched/signal.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/gcd.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/core.h>

#include "fsl_easrc.h"
#include "imx-pcm.h"

extern struct snd_soc_platform_driver fsl_easrc_platform;

#define FSL_EASRC_FORMATS       (SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_U16_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S24_3LE | \
				 SNDRV_PCM_FMTBIT_U24_LE | \
				 SNDRV_PCM_FMTBIT_U24_3LE | \
				 SNDRV_PCM_FMTBIT_S32_LE | \
				 SNDRV_PCM_FMTBIT_U32_LE | \
				 SNDRV_PCM_FMTBIT_S20_3LE | \
				 SNDRV_PCM_FMTBIT_U20_3LE | \
				 SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static int fsl_easrc_iec958_put_bits(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_easrc *easrc = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	easrc->bps_iec958 = val;

	return 0;
}

static int fsl_easrc_iec958_get_bits(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_easrc *easrc = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = easrc->bps_iec958;

	return 0;
}

static const char * const fsl_easrc_iec958_bits[] = {
	"16bit", "20bit", "24bit"
};

static const struct soc_enum fsl_easrc_iec958_bits_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fsl_easrc_iec958_bits),
			    fsl_easrc_iec958_bits);

static const struct snd_kcontrol_new fsl_easrc_snd_controls[] = {
	SOC_SINGLE("Context 0 Dither Switch", REG_EASRC_COC(0), 0, 1, 0),
	SOC_SINGLE("Context 1 Dither Switch", REG_EASRC_COC(1), 0, 1, 0),
	SOC_SINGLE("Context 2 Dither Switch", REG_EASRC_COC(2), 0, 1, 0),
	SOC_SINGLE("Context 3 Dither Switch", REG_EASRC_COC(3), 0, 1, 0),

	SOC_SINGLE("Context 0 IEC958 Validity", REG_EASRC_COC(0), 2, 1, 0),
	SOC_SINGLE("Context 1 IEC958 Validity", REG_EASRC_COC(1), 2, 1, 0),
	SOC_SINGLE("Context 2 IEC958 Validity", REG_EASRC_COC(2), 2, 1, 0),
	SOC_SINGLE("Context 3 IEC958 Validity", REG_EASRC_COC(3), 2, 1, 0),
	SOC_ENUM_EXT("IEC958 Bits Per Sample",
		     fsl_easrc_iec958_bits_enum,
		     fsl_easrc_iec958_get_bits,
		     fsl_easrc_iec958_put_bits),
};

/* set_rs_ratio
 *
 * According to the resample taps, calculate the resample ratio
 */
static int set_rs_ratio(struct fsl_easrc_context *ctx)
{
	struct fsl_easrc *easrc = ctx->easrc;
	unsigned int in_rate = ctx->in_params.norm_rate;
	unsigned int out_rate = ctx->out_params.norm_rate;
	unsigned int int_bits;
	unsigned int frac_bits;
	u64 val;
	u32 *r;
	int ret;

	switch (easrc->rs_num_taps) {
	case EASRC_RS_32_TAPS:
		int_bits = 5;
		frac_bits = 39;
		break;
	case EASRC_RS_64_TAPS:
		int_bits = 6;
		frac_bits = 38;
		break;
	case EASRC_RS_128_TAPS:
		int_bits = 7;
		frac_bits = 37;
		break;
	default:
		return -EINVAL;
	}

	val = ((uint64_t)in_rate << frac_bits) / out_rate;
	r = (uint32_t *)&val;
	ret = regmap_write(easrc->regmap,
			   REG_EASRC_RRL(ctx->index),
			   EASRC_RRL_RS_RL(r[0]));
	ret |= regmap_write(easrc->regmap,
			    REG_EASRC_RRH(ctx->index),
			    EASRC_RRH_RS_RH(r[1]));

	return ret;
}

/* normalize input and output sample rates */
static void fsl_easrc_normalize_rates(struct fsl_easrc_context *ctx)
{
	int a, b;

	if (!ctx)
		return;

	a = ctx->in_params.sample_rate;
	b = ctx->out_params.sample_rate;

	a = gcd(a, b);

	/* divide by gcd to normalize the rate */
	ctx->in_params.norm_rate = ctx->in_params.sample_rate / a;
	ctx->out_params.norm_rate = ctx->out_params.sample_rate / a;
}

/* resets the pointer of the coeff memory pointers */
static int fsl_coeff_mem_ptr_reset(struct fsl_easrc *easrc,
				   unsigned int ctx_id, int mem_type)
{
	struct device *dev;
	int ret = 0;
	u32 reg, mask, val;

	if (!easrc)
		return -ENODEV;

	dev = &easrc->pdev->dev;

	switch (mem_type) {
	case EASRC_PF_COEFF_MEM:
		/* This resets the prefilter memory pointer addr */
		if (ctx_id >= EASRC_CTX_MAX_NUM) {
			dev_err(dev, "Invalid context id[%d]\n", ctx_id);
			return -EINVAL;
		}

		reg = REG_EASRC_CCE1(ctx_id);
		mask = EASRC_CCE1_COEF_MEM_RST_MASK;
		val = EASRC_CCE1_COEF_MEM_RST;
		break;
	case EASRC_RS_COEFF_MEM:
		/* This resets the resampling memory pointer addr */
		reg = REG_EASRC_CRCC;
		mask = EASRC_CRCC_RS_CPR_MASK;
		val = EASRC_CRCC_RS_CPR;
		break;
	default:
		dev_err(dev, "Unknown memory type\n");
		return -EINVAL;
	}

	/*  To reset the write pointer back to zero, the register field
	 *  ASRC_CTX_CTRL_EXT1x[PF_COEFF_MEM_RST] can be toggled from
	 *  0x0 to 0x1 to 0x0.
	 */
	ret |= regmap_update_bits(easrc->regmap, reg, mask, 0);
	ret |= regmap_update_bits(easrc->regmap, reg, mask, val);
	ret |= regmap_update_bits(easrc->regmap, reg, mask, 0);

	return ret;
}

static inline uint32_t bits_taps_to_val(unsigned int t)
{
	switch (t) {
	case EASRC_RS_32_TAPS:
		return 32;
	case EASRC_RS_64_TAPS:
		return 64;
	case EASRC_RS_128_TAPS:
		return 128;
	}

	return 0;
}

static int fsl_easrc_resampler_config(struct fsl_easrc *easrc)
{
	struct device *dev = &easrc->pdev->dev;
	struct asrc_firmware_hdr *hdr =  easrc->firmware_hdr;
	struct interp_params *interp = easrc->interp;
	struct interp_params *selected_interp = NULL;
	unsigned int num_coeff;
	unsigned int i;
	u64 *arr;
	u32 r0, r1;
	u32 *r;
	int ret;

	if (!hdr) {
		dev_err(dev, "firmware not loaded!\n");
		return -ENODEV;
	}

	for (i = 0; i < hdr->interp_scen; i++) {
		if ((interp[i].num_taps - 1) ==
				bits_taps_to_val(easrc->rs_num_taps)) {
			arr = interp[i].coeff;
			selected_interp = &interp[i];
			dev_dbg(dev, "Selected interp_filter: %u taps - %u phases\n",
				selected_interp->num_taps,
				selected_interp->num_phases);
			break;
		}
	}

	if (!selected_interp) {
		dev_err(dev, "failed to get interpreter configuration\n");
		return -EINVAL;
	}

	/*
	 *  RS_LOW - first half of center tap of the sinc function
	 *  RS_HIGH - second half of center tap of the sinc function
	 *  This is due to the fact the resampling function must be
	 *  symetrical - i.e. odd number of taps
	 */
	r = (uint32_t *)&selected_interp->center_tap;
	r0 = r[0] & EASRC_32b_MASK;
	r1 = r[1] & EASRC_32b_MASK;

	ret = regmap_write(easrc->regmap,
			   REG_EASRC_RCTCL,
			   EASRC_RCTCL_RS_CL(r0));
	if (ret)
		return ret;

	ret = regmap_write(easrc->regmap,
			   REG_EASRC_RCTCH,
			   EASRC_RCTCH_RS_CH(r1));
	if (ret)
		return ret;

	/* Write Number of Resampling Coefficient Taps
	 * 00b - 32-Tap Resampling Filter
	 * 01b - 64-Tap Resampling Filter
	 * 10b - 128-Tap Resampling Filter
	 * 11b - N/A
	 */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CRCC,
				 EASRC_CRCC_RS_TAPS_MASK,
				 EASRC_CRCC_RS_TAPS(easrc->rs_num_taps));
	if (ret)
		return ret;

	/* Reset prefilter coefficient pointer back to 0 */
	ret = fsl_coeff_mem_ptr_reset(easrc, 0, EASRC_RS_COEFF_MEM);
	if (ret)
		return ret;

	/* When the filter is programmed to run in:
	 * 32-tap mode, 16-taps, 128-phases 4-coefficients per phase
	 * 64-tap mode, 32-taps, 64-phases 4-coefficients per phase
	 * 128-tap mode, 64-taps, 32-phases 4-coefficients per phase
	 * This means the number of writes is constant no matter
	 * the mode we are using
	 */
	num_coeff = 16 * 128 * 4;

	for (i = 0; i < num_coeff; i++) {
		r = (uint32_t *)&arr[i];
		r0 = r[0] & EASRC_32b_MASK;
		r1 = r[1] & EASRC_32b_MASK;

		ret |= regmap_write(easrc->regmap,
				    REG_EASRC_CRCM,
				    EASRC_CRCM_RS_CWD(r0));

		ret |= regmap_write(easrc->regmap,
				    REG_EASRC_CRCM,
				    EASRC_CRCM_RS_CWD(r1));

		if (ret)
			return ret;
	}

	return 0;
}

static int write_pf_coeff_mem(struct fsl_easrc *easrc, int ctx_id,
			      u64 *arr, int n_taps)
{
	struct device *dev = &easrc->pdev->dev;
	int ret = 0;
	int i;
	u32 *r;
	u32 r0, r1;

	/* If STx_NUM_TAPS is set to 0x0 then return */
	if (!n_taps)
		return 0;

	if (!arr) {
		dev_err(dev, "NULL buffer\n");
		return -EINVAL;
	}

	/* When switching between stages, the address pointer
	 * should be reset back to 0x0 before performing a write
	 */
	ret = fsl_coeff_mem_ptr_reset(easrc, ctx_id, EASRC_PF_COEFF_MEM);
	if (ret)
		return ret;

	for (i = 0; i < (n_taps + 1) / 2; i++) {
		r = (uint32_t *)&arr[i];
		r0 = r[0] & EASRC_32b_MASK;
		r1 = r[1] & EASRC_32b_MASK;

		ret |= regmap_write(easrc->regmap,
				    REG_EASRC_PCF(ctx_id),
				    EASRC_PCF_CD(r0));

		ret |= regmap_write(easrc->regmap,
				    REG_EASRC_PCF(ctx_id),
				    EASRC_PCF_CD(r1));

		if (ret)
			return ret;
	}

	return 0;
}

static int fsl_easrc_prefilter_config(struct fsl_easrc *easrc,
				      unsigned int ctx_id)
{
	struct fsl_easrc_context *ctx;
	struct asrc_firmware_hdr *hdr;
	struct prefil_params *prefil, *selected_prefil = NULL;
	struct device *dev;
	u32 inrate, outrate, offset = 0;
	int ret, i;

	/* to modify prefilter coeficients, the user must perform
	 * a write in ASRC_PRE_COEFF_FIFO[COEFF_DATA] while the
	 * RUN_EN for that context is set to 0
	 */
	if (!easrc)
		return -ENODEV;

	dev = &easrc->pdev->dev;

	if (ctx_id >= EASRC_CTX_MAX_NUM) {
		dev_err(dev, "Invalid context id[%d]\n", ctx_id);
		return -EINVAL;
	}

	ctx = easrc->ctx[ctx_id];

	ctx->in_filled_sample = bits_taps_to_val(easrc->rs_num_taps) / 2;
	ctx->out_missed_sample = ctx->in_filled_sample *
				 ctx->out_params.sample_rate /
				 ctx->in_params.sample_rate;

	ctx->st1_num_taps = 0;
	ctx->st2_num_taps = 0;

	ret = regmap_write(easrc->regmap, REG_EASRC_CCE1(ctx_id), 0);
	if (ret)
		return ret;

	ret = regmap_write(easrc->regmap, REG_EASRC_CCE2(ctx_id), 0);
	if (ret)
		return ret;

	/* prefilter is enabled only when doing downsampling.
	 * When out_rate >= in_rate, pf will be in bypass mode
	 */
	if (ctx->out_params.sample_rate >= ctx->in_params.sample_rate) {
		/* set pf in bypass mode */
		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE1(ctx_id),
					 EASRC_CCE1_PF_BYPASS_MASK,
					 EASRC_CCE1_PF_BYPASS);
		if (ret)
			return ret;

		/* PF_EXPANSION_FACTOR must be set to 0x0 when
		 * operating in bypass mode
		 */
		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE1(ctx_id),
					 EASRC_CCE1_PF_EXP_MASK,
					 EASRC_CCE1_PF_EXP(0));
		if (ret)
			return ret;

		if (ctx->out_params.sample_rate == ctx->in_params.sample_rate) {
			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_CCE1(ctx_id),
						 EASRC_CCE1_RS_BYPASS_MASK,
						 EASRC_CCE1_RS_BYPASS);
			if (ret)
				return ret;
		}

		return 0;
	}

	inrate = ctx->in_params.norm_rate;
	outrate = ctx->out_params.norm_rate;

	hdr = easrc->firmware_hdr;
	prefil = easrc->prefil;

	for (i = 0; i < hdr->prefil_scen; i++) {
		if (inrate == prefil[i].insr && outrate == prefil[i].outsr) {
			selected_prefil = &prefil[i];
			dev_dbg(dev, "Selected prefilter: %u insr, %u outsr, %u st1_taps, %u st2_taps\n",
				selected_prefil->insr,
				selected_prefil->outsr,
				selected_prefil->st1_taps,
				selected_prefil->st2_taps);
			break;
		}
	}

	if (!selected_prefil) {
		dev_err(dev, "Conversion from in ratio %u(%u) to out ratio %u(%u) is not supported\n",
			ctx->in_params.sample_rate,
			inrate,
			ctx->out_params.sample_rate, outrate);
		return -EINVAL;
	}

	/* in prefilter coeff array, first st1_num_taps represent the
	 * stage1 prefilter coefficients followed by next st2_num_taps
	 * representing stage 2 coefficients
	 */
	ctx->st1_num_taps = selected_prefil->st1_taps;
	ctx->st1_coeff    = selected_prefil->coeff;
	ctx->st1_num_exp  = selected_prefil->st1_exp;

	offset = ((selected_prefil->st1_taps + 1) / 2) *
			sizeof(selected_prefil->coeff[0]);
	ctx->st2_num_taps = selected_prefil->st2_taps;
	ctx->st2_coeff    = (uint64_t *)((uint64_t)selected_prefil->coeff +
								offset);

	ctx->in_filled_sample += (ctx->st1_num_taps / 2) * ctx->st1_num_exp +
				  ctx->st2_num_taps / 2;
	ctx->out_missed_sample = ctx->in_filled_sample *
				 ctx->out_params.sample_rate /
				 ctx->in_params.sample_rate;

	if (ctx->in_filled_sample * ctx->out_params.sample_rate %
				ctx->in_params.sample_rate != 0)
		ctx->out_missed_sample += 1;
	/* To modify the value of a prefilter coefficient, the user must
	 * perform a write to the register ASRC_PRE_COEFF_FIFOn[COEFF_DATA]
	 * while the respective context RUN_EN bit is set to 0b0
	 */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx_id),
				 EASRC_CC_EN_MASK, 0);
	if (ret)
		goto ctx_error;

	if (ctx->st1_num_taps > EASRC_MAX_PF_TAPS) {
		dev_err(dev, "ST1 taps [%d] mus be lower than %d\n",
			ctx->st1_num_taps, EASRC_MAX_PF_TAPS);
		ret = -EINVAL;
		goto ctx_error;
	}

	/* Update ctx ST1_NUM_TAPS in Context Control Extended 2 register */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CCE2(ctx_id),
				 EASRC_CCE2_ST1_TAPS_MASK,
				 EASRC_CCE2_ST1_TAPS(ctx->st1_num_taps - 1));
	if (ret)
		goto ctx_error;

	/* Prefilter Coefficient Write Select to write in ST1 coeff */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CCE1(ctx_id),
				 EASRC_CCE1_COEF_WS_MASK,
				 (EASRC_PF_ST1_COEFF_WR << EASRC_CCE1_COEF_WS_SHIFT));
	if (ret)
		goto ctx_error;

	ret = write_pf_coeff_mem(easrc, ctx_id,
				 ctx->st1_coeff, ctx->st1_num_taps);
	if (ret)
		goto ctx_error;

	if (ctx->st2_num_taps > 0) {
		if (ctx->st2_num_taps + ctx->st1_num_taps > EASRC_MAX_PF_TAPS) {
			dev_err(dev, "ST2 taps [%d] mus be lower than %d\n",
				ctx->st2_num_taps, EASRC_MAX_PF_TAPS);
			ret = -EINVAL;
			goto ctx_error;
		}

		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE1(ctx_id),
					 EASRC_CCE1_PF_TSEN_MASK,
					 EASRC_CCE1_PF_TSEN);
		if (ret)
			goto ctx_error;

		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE1(ctx_id),
					 EASRC_CCE1_PF_EXP_MASK,
					 EASRC_CCE1_PF_EXP(ctx->st1_num_exp - 1));
		if (ret)
			goto ctx_error;

		/* Update ctx ST2_NUM_TAPS in Context Control Extended 2 reg */
		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE2(ctx_id),
					 EASRC_CCE2_ST2_TAPS_MASK,
					 EASRC_CCE2_ST2_TAPS(ctx->st2_num_taps - 1));
		if (ret)
			goto ctx_error;

		/* Prefilter Coefficient Write Select to write in ST2 coeff */
		ret = regmap_update_bits(easrc->regmap,
					 REG_EASRC_CCE1(ctx_id),
					 EASRC_CCE1_COEF_WS_MASK,
					 EASRC_PF_ST2_COEFF_WR << EASRC_CCE1_COEF_WS_SHIFT);
		if (ret)
			goto ctx_error;

		ret = write_pf_coeff_mem(easrc, ctx_id,
					 ctx->st2_coeff, ctx->st2_num_taps);
		if (ret)
			goto ctx_error;
	}

	return 0;

ctx_error:
	return ret;
}

/* fsl_easrc_config_slot
 *
 * A single context can be split amongst any of the 4 context processing pipes
 * in the design.
 * The total number of channels consumed within the context processor must be
 * less than or equal to 8. if a single context is configured to contain more
 * than 8 channels then it must be distributed across multiple context
 * processing pipe slots.
 *
 */
static int fsl_easrc_config_slot(struct fsl_easrc *easrc, unsigned int ctx_id)
{
	struct fsl_easrc_context *ctx = easrc->ctx[ctx_id];
	int req_channels = ctx->channels;
	int start_channel = 0, avail_channel;
	int st1_chanxexp, st1_mem_alloc = 0, st2_mem_alloc = 0;
	bool continue_loop = false;
	struct fsl_easrc_slot *slot0, *slot1;
	int i, ret;

	if (req_channels <= 0)
		return -EINVAL;

	for (i = 0; i < EASRC_CTX_MAX_NUM; i++) {
		slot0 = &easrc->slot[i][0];
		slot1 = &easrc->slot[i][1];

		if (slot0->busy && slot1->busy)
			continue;

		if (!slot0->busy && !slot1->busy) {
			if (req_channels <= 8) {
				slot0->num_channel = req_channels;
				slot0->min_channel = start_channel;
				slot0->max_channel =
					start_channel + req_channels - 1;
				slot0->ctx_index = ctx->index;
				slot0->busy = true;
				start_channel += req_channels;
				req_channels = 0;
				continue_loop = false;
			} else {
				slot0->num_channel = 8;
				slot0->min_channel = start_channel;
				slot0->max_channel = start_channel + 7;
				slot0->ctx_index = ctx->index;
				slot0->busy = true;
				start_channel += 8;
				req_channels -= 8;
				continue_loop = true;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_MAXCH_MASK,
						 EASRC_DPCS0R0_MAXCH(slot0->max_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_MINCH_MASK,
						 EASRC_DPCS0R0_MINCH(slot0->min_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_NUMCH_MASK,
						 EASRC_DPCS0R0_NUMCH(slot0->num_channel - 1));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_CTXNUM_MASK,
						 EASRC_DPCS0R0_CTXNUM(slot0->ctx_index));
			if (ret)
				return ret;

			if (ctx->st1_num_taps > 0) {
				if (ctx->st2_num_taps > 0)
					st1_mem_alloc =
						(ctx->st1_num_taps - 1) *
						slot0->num_channel *
						ctx->st1_num_exp +
						slot0->num_channel;
				else
					st1_mem_alloc = ctx->st1_num_taps *
						slot0->num_channel;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R2(i),
							 EASRC_DPCS0R2_ST1_MA_MASK,
							 EASRC_DPCS0R2_ST1_MA(st1_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R2(i),
							 EASRC_DPCS0R2_ST1_SA_MASK,
							 EASRC_DPCS0R2_ST1_SA(0));
				if (ret)
					return ret;
			}

			if (ctx->st2_num_taps > 0) {
				st1_chanxexp = slot0->num_channel *
						(ctx->st1_num_exp - 1);

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R1(i),
							 EASRC_DPCS0R1_ST1_EXP_MASK,
							 EASRC_DPCS0R1_ST1_EXP(st1_chanxexp));
				if (ret)
					return ret;

				st2_mem_alloc = slot0->num_channel *
							ctx->st2_num_taps;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R3(i),
							 EASRC_DPCS0R3_ST2_MA_MASK,
							 EASRC_DPCS0R3_ST2_MA(st2_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R3(i),
							 EASRC_DPCS0R3_ST2_SA_MASK,
							 EASRC_DPCS0R3_ST2_SA(st1_mem_alloc));
				if (ret)
					return ret;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_EN_MASK,
						 EASRC_DPCS0R0_EN);
			if (ret)
				return ret;

			if (continue_loop)
				continue;
			else
				break;
		}

		if (slot0->busy && !slot1->busy) {
			if (slot0->ctx_index == ctx->index)
				continue;

			avail_channel = 8 - slot0->num_channel;

			if (avail_channel <= 0)
				continue;

			if (req_channels <= avail_channel) {
				slot1->num_channel = req_channels;
				slot1->min_channel = start_channel;
				slot1->max_channel =
					start_channel + req_channels - 1;
				slot1->ctx_index = ctx->index;
				slot1->busy = true;
				start_channel += req_channels;
				req_channels = 0;
				continue_loop = false;
			} else {
				slot1->num_channel = avail_channel;
				slot1->min_channel = start_channel;
				slot1->max_channel =
					start_channel + avail_channel - 1;
				slot1->ctx_index = ctx->index;
				slot1->busy = true;
				start_channel += avail_channel;
				req_channels -= avail_channel;
				continue_loop = true;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS1R0(i),
						 EASRC_DPCS0R0_MAXCH_MASK,
						 EASRC_DPCS0R0_MAXCH(slot1->max_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS1R0(i),
						 EASRC_DPCS0R0_MINCH_MASK,
						 EASRC_DPCS0R0_MINCH(slot1->min_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS1R0(i),
						 EASRC_DPCS0R0_NUMCH_MASK,
						 EASRC_DPCS0R0_NUMCH(slot1->num_channel - 1));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS1R0(i),
						 EASRC_DPCS0R0_CTXNUM_MASK,
						 EASRC_DPCS0R0_CTXNUM(slot1->ctx_index));
			if (ret)
				return ret;

			if (ctx->st1_num_taps > 0) {
				if (ctx->st2_num_taps > 0)
					st1_mem_alloc =
						(ctx->st1_num_taps - 1) *
						slot1->num_channel *
						ctx->st1_num_exp +
						slot1->num_channel;
				else
					st1_mem_alloc = ctx->st1_num_taps  *
						slot1->num_channel;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS1R2(i),
							 EASRC_DPCS0R2_ST1_MA_MASK,
							 EASRC_DPCS0R2_ST1_MA(st1_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS1R2(i),
							 EASRC_DPCS0R2_ST1_SA_MASK,
							 EASRC_DPCS0R2_ST1_SA(0x1800 -
							 st1_mem_alloc));
				if (ret)
					return ret;
			}

			if (ctx->st2_num_taps > 0) {
				st1_chanxexp = slot1->num_channel *
						(ctx->st1_num_exp - 1);

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS1R1(i),
							 EASRC_DPCS0R1_ST1_EXP_MASK,
							 EASRC_DPCS0R1_ST1_EXP(st1_chanxexp));
				if (ret)
					return ret;

				st2_mem_alloc = slot1->num_channel *
						ctx->st2_num_taps;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS1R3(i),
							 EASRC_DPCS0R3_ST2_MA_MASK,
							 EASRC_DPCS0R3_ST2_MA(st2_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS1R3(i),
							 EASRC_DPCS0R3_ST2_SA_MASK,
							 EASRC_DPCS0R3_ST2_SA(0x1800 -
								st1_mem_alloc -
								st2_mem_alloc));
				if (ret)
					return ret;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS1R0(i),
						 EASRC_DPCS0R0_EN_MASK,
						 EASRC_DPCS0R0_EN);
			if (ret)
				return ret;

			if (continue_loop)
				continue;
			else
				break;
		}

		if (!slot0->busy && slot1->busy) {
			if (slot1->ctx_index == ctx->index)
				continue;

			avail_channel = 8 - slot1->num_channel;

			if (avail_channel <= 0)
				continue;

			if (req_channels <= avail_channel) {
				slot0->num_channel = req_channels;
				slot0->min_channel = start_channel;
				slot0->max_channel =
					start_channel + req_channels - 1;
				slot0->ctx_index = ctx->index;
				slot0->busy = true;
				start_channel += req_channels;
				req_channels = 0;
				continue_loop = false;
			} else {
				slot0->num_channel = avail_channel;
				slot0->min_channel = start_channel;
				slot0->max_channel =
					start_channel + avail_channel - 1;
				slot0->ctx_index = ctx->index;
				slot0->busy = true;
				start_channel += avail_channel;
				req_channels -= avail_channel;
				continue_loop = true;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_MAXCH_MASK,
						 EASRC_DPCS0R0_MAXCH(slot0->max_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_MINCH_MASK,
						 EASRC_DPCS0R0_MINCH(slot0->min_channel));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_NUMCH_MASK,
						 EASRC_DPCS0R0_NUMCH(slot0->num_channel - 1));
			if (ret)
				return ret;

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_CTXNUM_MASK,
						 EASRC_DPCS0R0_CTXNUM(slot0->ctx_index));
			if (ret)
				return ret;

			if (ctx->st1_num_taps > 0) {
				if (ctx->st2_num_taps > 0)
					st1_mem_alloc =
						(ctx->st1_num_taps - 1) *
						slot0->num_channel *
						ctx->st1_num_exp +
						slot0->num_channel;
				else
					st1_mem_alloc =  ctx->st1_num_taps *
						slot0->num_channel;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R2(i),
							 EASRC_DPCS0R2_ST1_MA_MASK,
							 EASRC_DPCS0R2_ST1_MA(st1_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R2(i),
							 EASRC_DPCS0R2_ST1_SA_MASK,
							 EASRC_DPCS0R2_ST1_SA(0));
				if (ret)
					return ret;
			}

			if (ctx->st2_num_taps > 0) {
				st1_chanxexp = slot0->num_channel *
						(ctx->st1_num_exp - 1);

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R1(i),
							 EASRC_DPCS0R1_ST1_EXP_MASK,
							 EASRC_DPCS0R1_ST1_EXP(st1_chanxexp));
				if (ret)
					return ret;

				st2_mem_alloc = slot0->num_channel *
						ctx->st2_num_taps;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R3(i),
							 EASRC_DPCS0R3_ST2_MA_MASK,
							 EASRC_DPCS0R3_ST2_MA(st2_mem_alloc));
				if (ret)
					return ret;

				ret = regmap_update_bits(easrc->regmap,
							 REG_EASRC_DPCS0R3(i),
							 EASRC_DPCS0R3_ST2_SA_MASK,
							 EASRC_DPCS0R3_ST2_SA(st1_mem_alloc));
				if (ret)
					return ret;
			}

			ret = regmap_update_bits(easrc->regmap,
						 REG_EASRC_DPCS0R0(i),
						 EASRC_DPCS0R0_EN_MASK,
						 EASRC_DPCS0R0_EN);
			if (ret)
				return ret;

			if (continue_loop)
				continue;
			else
				break;
		}
	}

	if (req_channels > 0) {
		dev_err(&easrc->pdev->dev, "no avail slot, should not happen\n");
		return -EINVAL;
	}

	return 0;
}

/* fsl_easrc_release_slot
 *
 * clear the slot configuration
 */
static int fsl_easrc_release_slot(struct fsl_easrc *easrc, unsigned int ctx_id)
{
	struct fsl_easrc_context *ctx = easrc->ctx[ctx_id];
	int i, ret;

	for (i = 0; i < EASRC_CTX_MAX_NUM; i++) {
		if (easrc->slot[i][0].busy &&
		    easrc->slot[i][0].ctx_index == ctx->index) {
			easrc->slot[i][0].busy = false;
			/* set registers */
			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS0R0(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS0R1(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS0R2(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS0R3(i), 0);
			if (ret)
				return ret;
		}

		if (easrc->slot[i][1].busy &&
		    easrc->slot[i][1].ctx_index == ctx->index) {
			easrc->slot[i][1].busy = false;
			/* set registers */
			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS1R0(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS1R1(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS1R2(i), 0);
			if (ret)
				return ret;

			ret = regmap_write(easrc->regmap,
					   REG_EASRC_DPCS1R3(i), 0);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* fsl_easrc_config_context
 *
 * configure the register relate with context.
 */
int fsl_easrc_config_context(struct fsl_easrc *easrc, unsigned int ctx_id)
{
	struct fsl_easrc_context *ctx;
	struct device *dev;
	unsigned long lock_flags;
	int ret;

	/* to modify prefilter coeficients, the user must perform
	 * a write in ASRC_PRE_COEFF_FIFO[COEFF_DATA] while the
	 * RUN_EN for that context is set to 0
	 */
	if (!easrc)
		return -ENODEV;

	dev = &easrc->pdev->dev;

	if (ctx_id >= EASRC_CTX_MAX_NUM) {
		dev_err(dev, "Invalid context id[%d]\n", ctx_id);
		return -EINVAL;
	}

	ctx = easrc->ctx[ctx_id];

	fsl_easrc_normalize_rates(ctx);

	ret = set_rs_ratio(ctx);
	if (ret)
		return ret;

	/* initialize the context coeficients */
	ret = fsl_easrc_prefilter_config(easrc, ctx->index);
	if (ret)
		return ret;

	spin_lock_irqsave(&easrc->lock, lock_flags);
	ret = fsl_easrc_config_slot(easrc, ctx->index);
	spin_unlock_irqrestore(&easrc->lock, lock_flags);
	if (ret)
		return ret;

	/* Both prefilter and resampling filters can use following
	 * initialization modes:
	 * 2 - zero-fil mode
	 * 1 - replication mode
	 * 0 - software control
	 */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CCE1(ctx_id),
				 EASRC_CCE1_RS_INIT_MASK,
				 EASRC_CCE1_RS_INIT(ctx->rs_init_mode));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CCE1(ctx_id),
				 EASRC_CCE1_PF_INIT_MASK,
				 EASRC_CCE1_PF_INIT(ctx->pf_init_mode));
	if (ret)
		return ret;

	/* Context Input FIFO Watermark */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx_id),
				 EASRC_CC_FIFO_WTMK_MASK,
				 EASRC_CC_FIFO_WTMK(ctx->in_params.fifo_wtmk));
	if (ret)
		return ret;

	/* Context Output FIFO Watermark */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx_id),
				 EASRC_COC_FIFO_WTMK_MASK,
				 EASRC_COC_FIFO_WTMK(ctx->out_params.fifo_wtmk - 1));
	if (ret)
		return ret;

	/* number of channels */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx_id),
				 EASRC_CC_CHEN_MASK,
				 EASRC_CC_CHEN(ctx->channels - 1));
	return ret;
}

static int fsl_easrc_process_format(struct fsl_easrc *easrc,
			      struct fsl_easrc_data_fmt *fmt,
			      snd_pcm_format_t raw_fmt)
{
	int ret;

	if (!fmt)
		return -EINVAL;

	/* Context Input Floating Point Format
	 * 0 - Integer Format
	 * 1 - Single Precision FP Format
	 */
	fmt->floating_point = !snd_pcm_format_linear(raw_fmt);
	fmt->sample_pos = 0;
	fmt->iec958 = 0;

	/* get the data width */
	switch (snd_pcm_format_width(raw_fmt)) {
	case 16:
		fmt->width = EASRC_WIDTH_16_BIT;
		break;
	case 20:
		fmt->width = EASRC_WIDTH_20_BIT;
		break;
	case 24:
		fmt->width = EASRC_WIDTH_24_BIT;
		break;
	case 32:
		fmt->width = EASRC_WIDTH_32_BIT;
		break;
	default:
		return -EINVAL;
	}

	switch (raw_fmt) {
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
		fmt->width = easrc->bps_iec958;
		fmt->iec958 = 1;
		fmt->floating_point = 0;
		if (fmt->width == EASRC_WIDTH_16_BIT)
			fmt->sample_pos = 12;
		else if (fmt->width == EASRC_WIDTH_20_BIT)
			fmt->sample_pos = 8;
		else if (fmt->width == EASRC_WIDTH_24_BIT)
			fmt->sample_pos = 4;
		break;
	default:
		break;
	}

	/* Data Endianness
	 * 0 - Little-Endian
	 * 1 - Big-Endian
	 */
	ret = snd_pcm_format_big_endian(raw_fmt);
	if (ret < 0)
		return ret;

	fmt->endianness = ret;
	/* Input Data sign
	 * 0b - Signed Format
	 * 1b - Unsigned Format
	 */
	ret = snd_pcm_format_unsigned(raw_fmt) > 0 ? 1 : 0;
	if (ret < 0)
		return ret;

	fmt->unsign = ret;

	return 0;
}

int fsl_easrc_set_ctx_format(struct fsl_easrc_context *ctx,
			     snd_pcm_format_t *in_raw_format,
			     snd_pcm_format_t *out_raw_format)
{
	struct fsl_easrc *easrc = ctx->easrc;
	struct fsl_easrc_data_fmt *in_fmt = &ctx->in_params.fmt;
	struct fsl_easrc_data_fmt *out_fmt = &ctx->out_params.fmt;
	int ret;

	/* get the bitfield values for input data format */
	if (in_raw_format && out_raw_format) {
		ret = fsl_easrc_process_format(easrc, in_fmt, *in_raw_format);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_BPS_MASK,
				 EASRC_CC_BPS(in_fmt->width));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_ENDIANNESS_MASK,
				 in_fmt->endianness << EASRC_CC_ENDIANNESS_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_FMT_MASK,
				 in_fmt->floating_point << EASRC_CC_FMT_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_INSIGN_MASK,
				 in_fmt->unsign << EASRC_CC_INSIGN_SHIFT);

	/* In Sample Position */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_SAMPLE_POS_MASK,
				 EASRC_CC_SAMPLE_POS(in_fmt->sample_pos));
	if (ret)
		return ret;

	/* get the bitfield values for input data format */
	if (in_raw_format && out_raw_format) {
		ret = fsl_easrc_process_format(easrc, out_fmt, *out_raw_format);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_BPS_MASK,
				 EASRC_COC_BPS(out_fmt->width));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_ENDIANNESS_MASK,
				 out_fmt->endianness << EASRC_COC_ENDIANNESS_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_FMT_MASK,
				 out_fmt->floating_point << EASRC_COC_FMT_SHIFT);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_OUTSIGN_MASK,
				 out_fmt->unsign << EASRC_COC_OUTSIGN_SHIFT);
	if (ret)
		return ret;

	/* Out Sample Position */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_SAMPLE_POS_MASK,
				 EASRC_COC_SAMPLE_POS(out_fmt->sample_pos));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_IEC_EN_MASK,
				 out_fmt->iec958 << EASRC_COC_IEC_EN_SHIFT);

	return ret;
}

/* The ASRC provides interleaving support in hardware to ensure that a
 * variety of sample sources can be internally combined
 * to conform with this format. Interleaving parameters are accessed
 * through the ASRC_CTRL_IN_ACCESSa and ASRC_CTRL_OUT_ACCESSa registers
 */
int fsl_easrc_set_ctx_organziation(struct fsl_easrc_context *ctx)
{
	struct device *dev;
	struct fsl_easrc *easrc;
	int ret;

	if (!ctx)
		return -ENODEV;

	easrc = ctx->easrc;
	dev = &easrc->pdev->dev;

	/* input interleaving parameters */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CIA(ctx->index),
				 EASRC_CIA_ITER_MASK,
				 EASRC_CIA_ITER(ctx->in_params.iterations));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CIA(ctx->index),
				 EASRC_CIA_GRLEN_MASK,
				 EASRC_CIA_GRLEN(ctx->in_params.group_len));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CIA(ctx->index),
				 EASRC_CIA_ACCLEN_MASK,
				 EASRC_CIA_ACCLEN(ctx->in_params.access_len));
	if (ret)
		return ret;

	/* output interleaving parameters */
	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COA(ctx->index),
				 EASRC_COA_ITER_MASK,
				 EASRC_COA_ITER(ctx->out_params.iterations));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COA(ctx->index),
				 EASRC_COA_GRLEN_MASK,
				 EASRC_COA_GRLEN(ctx->out_params.group_len));
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COA(ctx->index),
				 EASRC_COA_ACCLEN_MASK,
				 EASRC_COA_ACCLEN(ctx->out_params.access_len));
	if (ret)
		return ret;

	return 0;
}

/* Request one of the available contexts
 *
 * Returns a negative number on error and >=0 as context id
 * on success
 */
int fsl_easrc_request_context(struct fsl_easrc_context *ctx,
			      unsigned int channels)
{
	enum asrc_pair_index index = ASRC_INVALID_PAIR;
	struct fsl_easrc *easrc = ctx->easrc;
	struct device *dev;
	unsigned long lock_flags;
	int ret = 0;
	int i;

	dev = &easrc->pdev->dev;

	spin_lock_irqsave(&easrc->lock, lock_flags);

	for (i = ASRC_PAIR_A; i < EASRC_CTX_MAX_NUM; i++) {
		if (easrc->ctx[i])
			continue;

		index = i;
		break;
	}

	if (index == ASRC_INVALID_PAIR) {
		dev_err(dev, "all contexts are busy\n");
		ret = -EBUSY;
	} else if (channels > easrc->chn_avail) {
		dev_err(dev, "can't give the required channels: %d\n",
			channels);
		ret = -EINVAL;
	} else {
		ctx->index = index;
		ctx->channels = channels;
		easrc->ctx[index] = ctx;
		easrc->chn_avail -= channels;
	}

	spin_unlock_irqrestore(&easrc->lock, lock_flags);

	return ret;
}

/* Release the context
 *
 * This funciton is mainly doing the revert thing in request context
 */
int fsl_easrc_release_context(struct fsl_easrc_context *ctx)
{
	unsigned long lock_flags;
	struct fsl_easrc *easrc;
	struct device *dev;
	int ret;

	if (!ctx)
		return 0;

	easrc = ctx->easrc;
	dev = &easrc->pdev->dev;

	spin_lock_irqsave(&easrc->lock, lock_flags);

	ret = fsl_easrc_release_slot(easrc, ctx->index);

	easrc->chn_avail += ctx->channels;
	easrc->ctx[ctx->index] = NULL;

	spin_unlock_irqrestore(&easrc->lock, lock_flags);

	return ret;
}

/* Start the context
 *
 * Enable the DMA request and context
 */
int fsl_easrc_start_context(struct fsl_easrc_context *ctx)
{
	struct fsl_easrc *easrc = ctx->easrc;
	int ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_FWMDE_MASK,
				 EASRC_CC_FWMDE);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_FWMDE_MASK,
				 EASRC_COC_FWMDE);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_EN_MASK,
				 EASRC_CC_EN);
	if (ret)
		return ret;

	return 0;
}

/* Stop the context
 *
 * Disable the DMA request and context
 */
int fsl_easrc_stop_context(struct fsl_easrc_context *ctx)
{
	struct fsl_easrc *easrc = ctx->easrc;
	int ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_EN_MASK, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_CC(ctx->index),
				 EASRC_CC_FWMDE_MASK, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(easrc->regmap,
				 REG_EASRC_COC(ctx->index),
				 EASRC_COC_FWMDE_MASK, 0);
	if (ret)
		return ret;

	return 0;
}

struct dma_chan *fsl_easrc_get_dma_channel(struct fsl_easrc_context *ctx,
					   bool dir)
{
	struct fsl_easrc *easrc = ctx->easrc;
	enum asrc_pair_index index = ctx->index;
	char name[8];

	/* example of dma name: ctx0_rx */
	sprintf(name, "ctx%c_%cx", index + '0', dir == IN ? 'r' : 't');

	return dma_request_slave_channel(&easrc->pdev->dev, name);
};
EXPORT_SYMBOL_GPL(fsl_easrc_get_dma_channel);

static const unsigned int easrc_rates[] = {
	8000, 11025, 12000, 16000,
	22050, 24000, 32000, 44100,
	48000, 64000, 88200, 96000,
	128000, 176400, 192000, 256000,
	352800, 384000, 705600, 768000,
};

static const struct snd_pcm_hw_constraint_list easrc_rate_constraints = {
	.count = ARRAY_SIZE(easrc_rates),
	.list = easrc_rates,
};

static int fsl_easrc_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  &easrc_rate_constraints);
}

static int fsl_easrc_trigger(struct snd_pcm_substream *substream,
			     int cmd, struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_easrc_context *ctx = runtime->private_data;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_easrc_start_context(ctx);
		if (ret)
			return ret;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = fsl_easrc_stop_context(ctx);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fsl_easrc_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct fsl_easrc *easrc = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = &easrc->pdev->dev;
	struct fsl_easrc_context *ctx = runtime->private_data;
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	snd_pcm_format_t format = params_format(params);
	int ret;

	ret = fsl_easrc_request_context(ctx, channels);
	if (ret) {
		dev_err(dev, "failed to request context\n");
		return ret;
	}

	ctx->ctx_streams |= BIT(substream->stream);

	/* set the input and output ratio so we can compute
	 * the resampling ratio in RS_LOW/HIGH
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ctx->in_params.sample_rate = rate;
		ctx->in_params.sample_format = format;
		ctx->out_params.sample_rate = easrc->easrc_rate;
		ctx->out_params.sample_format = easrc->easrc_format;
	} else {
		ctx->out_params.sample_rate = rate;
		ctx->out_params.sample_format = format;
		ctx->in_params.sample_rate = easrc->easrc_rate;
		ctx->in_params.sample_format = easrc->easrc_format;
	}

	ctx->channels = channels;
	ctx->in_params.fifo_wtmk  = 0x20;
	ctx->out_params.fifo_wtmk = 0x20;

	ret = fsl_easrc_config_context(easrc, ctx->index);
	if (ret) {
		dev_err(dev, "failed to config context\n");
		return ret;
	}

	/* do only rate conversion and keep the same format for input
	 * and output data
	 */
	ret = fsl_easrc_set_ctx_format(ctx,
				       &ctx->in_params.sample_format,
				       &ctx->out_params.sample_format);
	if (ret) {
		dev_err(dev, "failed to set format %d", ret);
		return ret;
	}

	ctx->in_params.iterations = 1;
	ctx->in_params.group_len = ctx->channels;
	ctx->in_params.access_len = ctx->channels;
	ctx->out_params.iterations = 1;
	ctx->out_params.group_len = ctx->channels;
	ctx->out_params.access_len = ctx->channels;

	ret = fsl_easrc_set_ctx_organziation(ctx);
	if (ret) {
		dev_err(dev, "failed to set fifo organization\n");
		return ret;
	}

	return 0;
}

static int fsl_easrc_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_easrc_context *ctx = runtime->private_data;
	int ret;

	if (ctx && (ctx->ctx_streams & BIT(substream->stream))) {
		ctx->ctx_streams &= ~BIT(substream->stream);
		ret = fsl_easrc_release_context(ctx);
		if (ret)
			return ret;
	}

	return 0;
}

static struct snd_soc_dai_ops fsl_easrc_dai_ops = {
	.startup = fsl_easrc_startup,
	.trigger = fsl_easrc_trigger,
	.hw_params = fsl_easrc_hw_params,
	.hw_free = fsl_easrc_hw_free,
};

static int fsl_easrc_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_easrc *easrc = dev_get_drvdata(cpu_dai->dev);

	snd_soc_dai_init_dma_data(cpu_dai,
				  &easrc->dma_params_tx,
				  &easrc->dma_params_rx);
	return 0;
}

static struct snd_soc_dai_driver fsl_easrc_dai = {
	.probe = fsl_easrc_dai_probe,
	.playback = {
		.stream_name = "ASRC-Playback",
		.channels_min = 1,
		.channels_max = 32,
		.rate_min = 8000,
		.rate_max = 768000,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_EASRC_FORMATS,
	},
	.capture = {
		.stream_name = "ASRC-Capture",
		.channels_min = 1,
		.channels_max = 32,
		.rate_min = 8000,
		.rate_max = 768000,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_EASRC_FORMATS,
	},
	.ops = &fsl_easrc_dai_ops,
};

static const struct snd_soc_component_driver fsl_easrc_component = {
	.name		= "fsl-easrc-dai",
	.controls       = fsl_easrc_snd_controls,
	.num_controls   = ARRAY_SIZE(fsl_easrc_snd_controls),
};

static const struct reg_default fsl_easrc_reg_defaults[] = {
	{REG_EASRC_WRFIFO(0),	0x00000000},
	{REG_EASRC_WRFIFO(1),	0x00000000},
	{REG_EASRC_WRFIFO(2),	0x00000000},
	{REG_EASRC_WRFIFO(3),	0x00000000},
	{REG_EASRC_RDFIFO(0),	0x00000000},
	{REG_EASRC_RDFIFO(1),	0x00000000},
	{REG_EASRC_RDFIFO(2),	0x00000000},
	{REG_EASRC_RDFIFO(3),	0x00000000},
	{REG_EASRC_CC(0),	0x00000000},
	{REG_EASRC_CC(1),	0x00000000},
	{REG_EASRC_CC(2),	0x00000000},
	{REG_EASRC_CC(3),	0x00000000},
	{REG_EASRC_CCE1(0),	0x00000000},
	{REG_EASRC_CCE1(1),	0x00000000},
	{REG_EASRC_CCE1(2),	0x00000000},
	{REG_EASRC_CCE1(3),	0x00000000},
	{REG_EASRC_CCE2(0),	0x00000000},
	{REG_EASRC_CCE2(1),	0x00000000},
	{REG_EASRC_CCE2(2),	0x00000000},
	{REG_EASRC_CCE2(3),	0x00000000},
	{REG_EASRC_CIA(0),	0x00000000},
	{REG_EASRC_CIA(1),	0x00000000},
	{REG_EASRC_CIA(2),	0x00000000},
	{REG_EASRC_CIA(3),	0x00000000},
	{REG_EASRC_DPCS0R0(0),	0x00000000},
	{REG_EASRC_DPCS0R0(1),	0x00000000},
	{REG_EASRC_DPCS0R0(2),	0x00000000},
	{REG_EASRC_DPCS0R0(3),	0x00000000},
	{REG_EASRC_DPCS0R1(0),	0x00000000},
	{REG_EASRC_DPCS0R1(1),	0x00000000},
	{REG_EASRC_DPCS0R1(2),	0x00000000},
	{REG_EASRC_DPCS0R1(3),	0x00000000},
	{REG_EASRC_DPCS0R2(0),	0x00000000},
	{REG_EASRC_DPCS0R2(1),	0x00000000},
	{REG_EASRC_DPCS0R2(2),	0x00000000},
	{REG_EASRC_DPCS0R2(3),	0x00000000},
	{REG_EASRC_DPCS0R3(0),	0x00000000},
	{REG_EASRC_DPCS0R3(1),	0x00000000},
	{REG_EASRC_DPCS0R3(2),	0x00000000},
	{REG_EASRC_DPCS0R3(3),	0x00000000},
	{REG_EASRC_DPCS1R0(0),	0x00000000},
	{REG_EASRC_DPCS1R0(1),	0x00000000},
	{REG_EASRC_DPCS1R0(2),	0x00000000},
	{REG_EASRC_DPCS1R0(3),	0x00000000},
	{REG_EASRC_DPCS1R1(0),	0x00000000},
	{REG_EASRC_DPCS1R1(1),	0x00000000},
	{REG_EASRC_DPCS1R1(2),	0x00000000},
	{REG_EASRC_DPCS1R1(3),	0x00000000},
	{REG_EASRC_DPCS1R2(0),	0x00000000},
	{REG_EASRC_DPCS1R2(1),	0x00000000},
	{REG_EASRC_DPCS1R2(2),	0x00000000},
	{REG_EASRC_DPCS1R2(3),	0x00000000},
	{REG_EASRC_DPCS1R3(0),	0x00000000},
	{REG_EASRC_DPCS1R3(1),	0x00000000},
	{REG_EASRC_DPCS1R3(2),	0x00000000},
	{REG_EASRC_DPCS1R3(3),	0x00000000},
	{REG_EASRC_COC(0),	0x00000000},
	{REG_EASRC_COC(1),	0x00000000},
	{REG_EASRC_COC(2),	0x00000000},
	{REG_EASRC_COC(3),	0x00000000},
	{REG_EASRC_COA(0),	0x00000000},
	{REG_EASRC_COA(1),	0x00000000},
	{REG_EASRC_COA(2),	0x00000000},
	{REG_EASRC_COA(3),	0x00000000},
	{REG_EASRC_SFS(0),	0x00000000},
	{REG_EASRC_SFS(1),	0x00000000},
	{REG_EASRC_SFS(2),	0x00000000},
	{REG_EASRC_SFS(3),	0x00000000},
	{REG_EASRC_RRL(0),	0x00000000},
	{REG_EASRC_RRL(1),	0x00000000},
	{REG_EASRC_RRL(2),	0x00000000},
	{REG_EASRC_RRL(3),	0x00000000},
	{REG_EASRC_RRH(0),	0x00000000},
	{REG_EASRC_RRH(1),	0x00000000},
	{REG_EASRC_RRH(2),	0x00000000},
	{REG_EASRC_RRH(3),	0x00000000},
	{REG_EASRC_RUC(0),	0x00000000},
	{REG_EASRC_RUC(1),	0x00000000},
	{REG_EASRC_RUC(2),	0x00000000},
	{REG_EASRC_RUC(3),	0x00000000},
	{REG_EASRC_RUR(0),	0x7FFFFFFF},
	{REG_EASRC_RUR(1),	0x7FFFFFFF},
	{REG_EASRC_RUR(2),	0x7FFFFFFF},
	{REG_EASRC_RUR(3),	0x7FFFFFFF},
	{REG_EASRC_RCTCL,	0x00000000},
	{REG_EASRC_RCTCH,	0x00000000},
	{REG_EASRC_PCF(0),	0x00000000},
	{REG_EASRC_PCF(1),	0x00000000},
	{REG_EASRC_PCF(2),	0x00000000},
	{REG_EASRC_PCF(3),	0x00000000},
	{REG_EASRC_CRCM,	0x00000000},
	{REG_EASRC_CRCC,	0x00000000},
	{REG_EASRC_IRQC,	0x00000FFF},
	{REG_EASRC_IRQF,	0x00000000},
	{REG_EASRC_CS0(0),	0x00000000},
	{REG_EASRC_CS0(1),	0x00000000},
	{REG_EASRC_CS0(2),	0x00000000},
	{REG_EASRC_CS0(3),	0x00000000},
	{REG_EASRC_CS1(0),	0x00000000},
	{REG_EASRC_CS1(1),	0x00000000},
	{REG_EASRC_CS1(2),	0x00000000},
	{REG_EASRC_CS1(3),	0x00000000},
	{REG_EASRC_CS2(0),	0x00000000},
	{REG_EASRC_CS2(1),	0x00000000},
	{REG_EASRC_CS2(2),	0x00000000},
	{REG_EASRC_CS2(3),	0x00000000},
	{REG_EASRC_CS3(0),	0x00000000},
	{REG_EASRC_CS3(1),	0x00000000},
	{REG_EASRC_CS3(2),	0x00000000},
	{REG_EASRC_CS3(3),	0x00000000},
	{REG_EASRC_CS4(0),	0x00000000},
	{REG_EASRC_CS4(1),	0x00000000},
	{REG_EASRC_CS4(2),	0x00000000},
	{REG_EASRC_CS4(3),	0x00000000},
	{REG_EASRC_CS5(0),	0x00000000},
	{REG_EASRC_CS5(1),	0x00000000},
	{REG_EASRC_CS5(2),	0x00000000},
	{REG_EASRC_CS5(3),	0x00000000},
	{REG_EASRC_DBGC,	0x00000000},
	{REG_EASRC_DBGS,	0x00000000},
};

static bool fsl_easrc_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_EASRC_RDFIFO(0):
	case REG_EASRC_RDFIFO(1):
	case REG_EASRC_RDFIFO(2):
	case REG_EASRC_RDFIFO(3):
	case REG_EASRC_CC(0):
	case REG_EASRC_CC(1):
	case REG_EASRC_CC(2):
	case REG_EASRC_CC(3):
	case REG_EASRC_CCE1(0):
	case REG_EASRC_CCE1(1):
	case REG_EASRC_CCE1(2):
	case REG_EASRC_CCE1(3):
	case REG_EASRC_CCE2(0):
	case REG_EASRC_CCE2(1):
	case REG_EASRC_CCE2(2):
	case REG_EASRC_CCE2(3):
	case REG_EASRC_CIA(0):
	case REG_EASRC_CIA(1):
	case REG_EASRC_CIA(2):
	case REG_EASRC_CIA(3):
	case REG_EASRC_DPCS0R0(0):
	case REG_EASRC_DPCS0R0(1):
	case REG_EASRC_DPCS0R0(2):
	case REG_EASRC_DPCS0R0(3):
	case REG_EASRC_DPCS0R1(0):
	case REG_EASRC_DPCS0R1(1):
	case REG_EASRC_DPCS0R1(2):
	case REG_EASRC_DPCS0R1(3):
	case REG_EASRC_DPCS0R2(0):
	case REG_EASRC_DPCS0R2(1):
	case REG_EASRC_DPCS0R2(2):
	case REG_EASRC_DPCS0R2(3):
	case REG_EASRC_DPCS0R3(0):
	case REG_EASRC_DPCS0R3(1):
	case REG_EASRC_DPCS0R3(2):
	case REG_EASRC_DPCS0R3(3):
	case REG_EASRC_DPCS1R0(0):
	case REG_EASRC_DPCS1R0(1):
	case REG_EASRC_DPCS1R0(2):
	case REG_EASRC_DPCS1R0(3):
	case REG_EASRC_DPCS1R1(0):
	case REG_EASRC_DPCS1R1(1):
	case REG_EASRC_DPCS1R1(2):
	case REG_EASRC_DPCS1R1(3):
	case REG_EASRC_DPCS1R2(0):
	case REG_EASRC_DPCS1R2(1):
	case REG_EASRC_DPCS1R2(2):
	case REG_EASRC_DPCS1R2(3):
	case REG_EASRC_DPCS1R3(0):
	case REG_EASRC_DPCS1R3(1):
	case REG_EASRC_DPCS1R3(2):
	case REG_EASRC_DPCS1R3(3):
	case REG_EASRC_COC(0):
	case REG_EASRC_COC(1):
	case REG_EASRC_COC(2):
	case REG_EASRC_COC(3):
	case REG_EASRC_COA(0):
	case REG_EASRC_COA(1):
	case REG_EASRC_COA(2):
	case REG_EASRC_COA(3):
	case REG_EASRC_SFS(0):
	case REG_EASRC_SFS(1):
	case REG_EASRC_SFS(2):
	case REG_EASRC_SFS(3):
	case REG_EASRC_RRL(0):
	case REG_EASRC_RRL(1):
	case REG_EASRC_RRL(2):
	case REG_EASRC_RRL(3):
	case REG_EASRC_RRH(0):
	case REG_EASRC_RRH(1):
	case REG_EASRC_RRH(2):
	case REG_EASRC_RRH(3):
	case REG_EASRC_RUC(0):
	case REG_EASRC_RUC(1):
	case REG_EASRC_RUC(2):
	case REG_EASRC_RUC(3):
	case REG_EASRC_RUR(0):
	case REG_EASRC_RUR(1):
	case REG_EASRC_RUR(2):
	case REG_EASRC_RUR(3): /* fallthrough */
	case REG_EASRC_RCTCL:
	case REG_EASRC_RCTCH:
	case REG_EASRC_PCF(0):
	case REG_EASRC_PCF(1):
	case REG_EASRC_PCF(2):
	case REG_EASRC_PCF(3): /* fallthrough */
	case REG_EASRC_CRCC:
	case REG_EASRC_IRQC:
	case REG_EASRC_IRQF:
	case REG_EASRC_CS0(0):
	case REG_EASRC_CS0(1):
	case REG_EASRC_CS0(2):
	case REG_EASRC_CS0(3):
	case REG_EASRC_CS1(0):
	case REG_EASRC_CS1(1):
	case REG_EASRC_CS1(2):
	case REG_EASRC_CS1(3):
	case REG_EASRC_CS2(0):
	case REG_EASRC_CS2(1):
	case REG_EASRC_CS2(2):
	case REG_EASRC_CS2(3):
	case REG_EASRC_CS3(0):
	case REG_EASRC_CS3(1):
	case REG_EASRC_CS3(2):
	case REG_EASRC_CS3(3):
	case REG_EASRC_CS4(0):
	case REG_EASRC_CS4(1):
	case REG_EASRC_CS4(2):
	case REG_EASRC_CS4(3):
	case REG_EASRC_CS5(0):
	case REG_EASRC_CS5(1):
	case REG_EASRC_CS5(2):
	case REG_EASRC_CS5(3): /* fallthrough */
	case REG_EASRC_DBGC:
	case REG_EASRC_DBGS:
		return true;
	default:
		return false;
	}
}

static bool fsl_easrc_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_EASRC_WRFIFO(0):
	case REG_EASRC_WRFIFO(1):
	case REG_EASRC_WRFIFO(2):
	case REG_EASRC_WRFIFO(3):
	case REG_EASRC_CC(0):
	case REG_EASRC_CC(1):
	case REG_EASRC_CC(2):
	case REG_EASRC_CC(3):
	case REG_EASRC_CCE1(0):
	case REG_EASRC_CCE1(1):
	case REG_EASRC_CCE1(2):
	case REG_EASRC_CCE1(3):
	case REG_EASRC_CCE2(0):
	case REG_EASRC_CCE2(1):
	case REG_EASRC_CCE2(2):
	case REG_EASRC_CCE2(3):
	case REG_EASRC_CIA(0):
	case REG_EASRC_CIA(1):
	case REG_EASRC_CIA(2):
	case REG_EASRC_CIA(3):
	case REG_EASRC_DPCS0R0(0):
	case REG_EASRC_DPCS0R0(1):
	case REG_EASRC_DPCS0R0(2):
	case REG_EASRC_DPCS0R0(3):
	case REG_EASRC_DPCS0R1(0):
	case REG_EASRC_DPCS0R1(1):
	case REG_EASRC_DPCS0R1(2):
	case REG_EASRC_DPCS0R1(3):
	case REG_EASRC_DPCS0R2(0):
	case REG_EASRC_DPCS0R2(1):
	case REG_EASRC_DPCS0R2(2):
	case REG_EASRC_DPCS0R2(3):
	case REG_EASRC_DPCS0R3(0):
	case REG_EASRC_DPCS0R3(1):
	case REG_EASRC_DPCS0R3(2):
	case REG_EASRC_DPCS0R3(3):
	case REG_EASRC_DPCS1R0(0):
	case REG_EASRC_DPCS1R0(1):
	case REG_EASRC_DPCS1R0(2):
	case REG_EASRC_DPCS1R0(3):
	case REG_EASRC_DPCS1R1(0):
	case REG_EASRC_DPCS1R1(1):
	case REG_EASRC_DPCS1R1(2):
	case REG_EASRC_DPCS1R1(3):
	case REG_EASRC_DPCS1R2(0):
	case REG_EASRC_DPCS1R2(1):
	case REG_EASRC_DPCS1R2(2):
	case REG_EASRC_DPCS1R2(3):
	case REG_EASRC_DPCS1R3(0):
	case REG_EASRC_DPCS1R3(1):
	case REG_EASRC_DPCS1R3(2):
	case REG_EASRC_DPCS1R3(3):
	case REG_EASRC_COC(0):
	case REG_EASRC_COC(1):
	case REG_EASRC_COC(2):
	case REG_EASRC_COC(3):
	case REG_EASRC_COA(0):
	case REG_EASRC_COA(1):
	case REG_EASRC_COA(2):
	case REG_EASRC_COA(3):
	case REG_EASRC_RRL(0):
	case REG_EASRC_RRL(1):
	case REG_EASRC_RRL(2):
	case REG_EASRC_RRL(3):
	case REG_EASRC_RRH(0):
	case REG_EASRC_RRH(1):
	case REG_EASRC_RRH(2):
	case REG_EASRC_RRH(3):
	case REG_EASRC_RUC(0):
	case REG_EASRC_RUC(1):
	case REG_EASRC_RUC(2):
	case REG_EASRC_RUC(3):
	case REG_EASRC_RUR(0):
	case REG_EASRC_RUR(1):
	case REG_EASRC_RUR(2):
	case REG_EASRC_RUR(3):  /* fallthrough */
	case REG_EASRC_RCTCL:
	case REG_EASRC_RCTCH:
	case REG_EASRC_PCF(0):
	case REG_EASRC_PCF(1):
	case REG_EASRC_PCF(2):
	case REG_EASRC_PCF(3):  /* fallthrough */
	case REG_EASRC_CRCM:
	case REG_EASRC_CRCC:
	case REG_EASRC_IRQC:
	case REG_EASRC_IRQF:
	case REG_EASRC_CS0(0):
	case REG_EASRC_CS0(1):
	case REG_EASRC_CS0(2):
	case REG_EASRC_CS0(3):
	case REG_EASRC_CS1(0):
	case REG_EASRC_CS1(1):
	case REG_EASRC_CS1(2):
	case REG_EASRC_CS1(3):
	case REG_EASRC_CS2(0):
	case REG_EASRC_CS2(1):
	case REG_EASRC_CS2(2):
	case REG_EASRC_CS2(3):
	case REG_EASRC_CS3(0):
	case REG_EASRC_CS3(1):
	case REG_EASRC_CS3(2):
	case REG_EASRC_CS3(3):
	case REG_EASRC_CS4(0):
	case REG_EASRC_CS4(1):
	case REG_EASRC_CS4(2):
	case REG_EASRC_CS4(3):
	case REG_EASRC_CS5(0):
	case REG_EASRC_CS5(1):
	case REG_EASRC_CS5(2):
	case REG_EASRC_CS5(3): /* fallthrough */
	case REG_EASRC_DBGC:
		return true;
	default:
		return false;
	}
}

static bool fsl_easrc_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_EASRC_RDFIFO(0):
	case REG_EASRC_RDFIFO(1):
	case REG_EASRC_RDFIFO(2):
	case REG_EASRC_RDFIFO(3):
	case REG_EASRC_SFS(0):
	case REG_EASRC_SFS(1):
	case REG_EASRC_SFS(2):
	case REG_EASRC_SFS(3):
	case REG_EASRC_CS0(0):
	case REG_EASRC_CS0(1):
	case REG_EASRC_CS0(2):
	case REG_EASRC_CS0(3):
	case REG_EASRC_CS1(0):
	case REG_EASRC_CS1(1):
	case REG_EASRC_CS1(2):
	case REG_EASRC_CS1(3):
	case REG_EASRC_CS2(0):
	case REG_EASRC_CS2(1):
	case REG_EASRC_CS2(2):
	case REG_EASRC_CS2(3):
	case REG_EASRC_CS3(0):
	case REG_EASRC_CS3(1):
	case REG_EASRC_CS3(2):
	case REG_EASRC_CS3(3):
	case REG_EASRC_CS4(0):
	case REG_EASRC_CS4(1):
	case REG_EASRC_CS4(2):
	case REG_EASRC_CS4(3):
	case REG_EASRC_CS5(0):
	case REG_EASRC_CS5(1):
	case REG_EASRC_CS5(2):
	case REG_EASRC_CS5(3): /* fallthrough */
	case REG_EASRC_DBGS:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_easrc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = REG_EASRC_DBGS,
	.reg_defaults = fsl_easrc_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_easrc_reg_defaults),
	.readable_reg = fsl_easrc_readable_reg,
	.volatile_reg = fsl_easrc_volatile_reg,
	.writeable_reg = fsl_easrc_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

#include "fsl_easrc_m2m.c"

void easrc_dump_firmware(struct fsl_easrc *easrc)
{
	struct device *dev = &easrc->pdev->dev;
	struct asrc_firmware_hdr *firm = easrc->firmware_hdr;
	struct interp_params *interp = easrc->interp;
	struct prefil_params *prefil = easrc->prefil;
	int i;

	if (firm->magic != FIRMWARE_MAGIC) {
		dev_err(dev, "Wrong magic. Something went wrong!");
		return;
	}

	dev_dbg(dev, "Firmware v%u dump:\n", firm->firmware_version);
	pr_debug("Num prefitler scenarios: %u\n", firm->prefil_scen);
	pr_debug("Num interpolation scenarios: %u\n", firm->interp_scen);
	pr_debug("\nInterpolation scenarios:\n");

	for (i = 0; i < firm->interp_scen; i++) {
		if (interp[i].magic != FIRMWARE_MAGIC) {
			pr_debug("%d. wrong interp magic: %x\n",
				 i, interp[i].magic);
			continue;
		}
		pr_debug("%d. taps: %u, phases: %u, center: %llu\n", i,
			 interp[i].num_taps, interp[i].num_phases,
			 interp[i].center_tap);
	}

	for (i = 0; i < firm->prefil_scen; i++) {
		if (prefil[i].magic != FIRMWARE_MAGIC) {
			pr_debug("%d. wrong prefil magic: %x\n",
				 i, prefil[i].magic);
			continue;
		}
		pr_debug("%d. insr: %u, outsr: %u, st1: %u, st2: %u\n", i,
			 prefil[i].insr, prefil[i].outsr,
			 prefil[i].st1_taps, prefil[i].st2_taps);
	}

	dev_dbg(dev, "end of firmware dump\n");
}

int easrc_get_firmware(struct fsl_easrc *easrc)
{
	u32 pnum, inum, offset;
	int ret;

	if (!easrc)
		return -EINVAL;

	ret = request_firmware(&easrc->fw, easrc->fw_name,
			       &easrc->pdev->dev);
	if (ret)
		return ret;

	easrc->firmware_hdr = (struct asrc_firmware_hdr *)easrc->fw->data;
	pnum = easrc->firmware_hdr->prefil_scen;
	inum = easrc->firmware_hdr->interp_scen;

	if (inum) {
		offset = sizeof(struct asrc_firmware_hdr);
		easrc->interp =
			(struct interp_params *)(easrc->fw->data + offset);
	}

	if (pnum) {
		offset = sizeof(struct asrc_firmware_hdr) +
				inum * sizeof(struct interp_params);
		easrc->prefil =
			(struct prefil_params *)(easrc->fw->data + offset);
	}

	return 0;
}

static irqreturn_t fsl_easrc_isr(int irq, void *dev_id)
{
	struct fsl_easrc *easrc = (struct fsl_easrc *)dev_id;
	struct device *dev = &easrc->pdev->dev;

	/* TODO treat the interrupt */
	dev_err(dev, "interrupt\n");

	return IRQ_HANDLED;
}

static const struct of_device_id fsl_easrc_dt_ids[] = {
	{ .compatible = "fsl,imx8mn-easrc",},
	{}
};

MODULE_DEVICE_TABLE(of, fsl_easrc_dt_ids);

static int fsl_easrc_probe(struct platform_device *pdev)
{
	struct fsl_easrc *easrc;
	struct resource *res;
	struct device_node *np;
	void __iomem *regs;
	int ret, irq;
	u32 width;

	easrc = devm_kzalloc(&pdev->dev, sizeof(*easrc), GFP_KERNEL);
	if (!easrc)
		return -ENOMEM;

	strncpy(easrc->name, "mxc_asrc", sizeof(easrc->name) - 1);

	easrc->pdev = pdev;
	np = pdev->dev.of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "failed ioremap\n");
		return PTR_ERR(regs);
	}

	easrc->paddr = res->start;

	easrc->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "mem", regs,
						  &fsl_easrc_regmap_config);
	if (IS_ERR(easrc->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap");
		return PTR_ERR(easrc->regmap);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for node %s\n",
			dev_name(&pdev->dev));
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, fsl_easrc_isr, 0,
			       dev_name(&pdev->dev), easrc);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %u: %d\n", irq, ret);
		return ret;
	}

	easrc->mem_clk = devm_clk_get(&pdev->dev, "mem");
	if (IS_ERR(easrc->mem_clk)) {
		dev_err(&pdev->dev, "failed to get mem clock\n");
		return PTR_ERR(easrc->mem_clk);
	}

	/*Set default value*/
	easrc->chn_avail = 32;
	easrc->rs_num_taps = EASRC_RS_128_TAPS;
	easrc->bps_iec958 = EASRC_WIDTH_16_BIT;

	ret = of_property_read_u32(np, "fsl,asrc-rate",
				   &easrc->easrc_rate);
	if (ret) {
		dev_err(&pdev->dev, "failed to asrc rate\n");
		return ret;
	}

	ret = of_property_read_u32(np, "fsl,asrc-width",
				   &width);
	if (ret) {
		dev_err(&pdev->dev, "failed to asrc width\n");
		return ret;
	}

	if (width != 16 && width != 24) {
		dev_warn(&pdev->dev, "unsupported width, switching to 24bit\n");
		width = 24;
	}

	if (width == 24)
		easrc->easrc_format = SNDRV_PCM_FORMAT_S24_LE;
	else
		easrc->easrc_format = SNDRV_PCM_FORMAT_S16_LE;

	platform_set_drvdata(pdev, easrc);
	pm_runtime_enable(&pdev->dev);

	spin_lock_init(&easrc->lock);

	regcache_cache_only(easrc->regmap, true);

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_easrc_component,
					      &fsl_easrc_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC DAI\n");
		return ret;
	}

	ret = devm_snd_soc_register_platform(&pdev->dev, &fsl_easrc_platform);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC platform\n");
		return ret;
	}

	ret = fsl_easrc_m2m_init(easrc);
	if (ret) {
		dev_err(&pdev->dev, "failed to init m2m device %d\n", ret);
		return ret;
	}

	ret = of_property_read_string(np,
				      "fsl,easrc-ram-script-name",
				      &easrc->fw_name);
	if (ret) {
		dev_err(&pdev->dev, "failed to get firmware name\n");
		return ret;
	}

	return 0;
}

static int fsl_easrc_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_easrc_runtime_suspend(struct device *dev)
{
	struct fsl_easrc *easrc = dev_get_drvdata(dev);
	unsigned long lock_flags;

	regcache_cache_only(easrc->regmap, true);

	clk_disable_unprepare(easrc->mem_clk);

	spin_lock_irqsave(&easrc->lock, lock_flags);
	easrc->firmware_loaded = 0;
	spin_unlock_irqrestore(&easrc->lock, lock_flags);

	return 0;
}

static int fsl_easrc_runtime_resume(struct device *dev)
{
	struct fsl_easrc *easrc = dev_get_drvdata(dev);
	struct fsl_easrc_context *ctx;
	unsigned long lock_flags;
	int ret;
	int i;

	ret = clk_prepare_enable(easrc->mem_clk);
	if (ret)
		return ret;

	regcache_cache_only(easrc->regmap, false);
	regcache_mark_dirty(easrc->regmap);
	regcache_sync(easrc->regmap);

	spin_lock_irqsave(&easrc->lock, lock_flags);
	if (easrc->firmware_loaded) {
		spin_unlock_irqrestore(&easrc->lock, lock_flags);
		goto skip_load;
	}
	easrc->firmware_loaded = 1;
	spin_unlock_irqrestore(&easrc->lock, lock_flags);

	ret = easrc_get_firmware(easrc);
	if (ret) {
		dev_err(dev, "failed to get firmware\n");
		goto disable_mem_clk;
	}

	/* Write Resampling Coefficients
	 * The coefficient RAM must be configured prior to beginning of
	 * any context processing within the ASRC
	 */
	ret = fsl_easrc_resampler_config(easrc);
	if (ret) {
		dev_err(dev, "resampler config failed\n");
		goto disable_mem_clk;
	}

	for (i = ASRC_PAIR_A; i < EASRC_CTX_MAX_NUM; i++) {
		ctx = easrc->ctx[i];
		if (ctx) {
			set_rs_ratio(ctx);
			ctx->out_missed_sample = ctx->in_filled_sample *
						ctx->out_params.sample_rate /
						ctx->in_params.sample_rate;
			if (ctx->in_filled_sample * ctx->out_params.sample_rate
					% ctx->in_params.sample_rate != 0)
				ctx->out_missed_sample += 1;

			ret = write_pf_coeff_mem(easrc, i, ctx->st1_coeff,
						 ctx->st1_num_taps);
			if (ret)
				goto disable_mem_clk;

			ret = write_pf_coeff_mem(easrc, i, ctx->st2_coeff,
						 ctx->st2_num_taps);
			if (ret)
				goto disable_mem_clk;
		}
	}

skip_load:
	return 0;

disable_mem_clk:
	clk_disable_unprepare(easrc->mem_clk);
	return ret;
}
#endif /*CONFIG_PM*/

#ifdef CONFIG_PM_SLEEP
static int fsl_easrc_suspend(struct device *dev)
{
	struct fsl_easrc *easrc = dev_get_drvdata(dev);
	int ret;

	fsl_easrc_m2m_suspend(easrc);

	ret = pm_runtime_force_suspend(dev);

	return ret;
}

static int fsl_easrc_resume(struct device *dev)
{
	struct fsl_easrc *easrc = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);

	fsl_easrc_m2m_resume(easrc);

	return ret;
}
#endif /*CONFIG_PM_SLEEP*/

static const struct dev_pm_ops fsl_easrc_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_easrc_runtime_suspend,
			   fsl_easrc_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_easrc_suspend,
				fsl_easrc_resume)
};

static struct platform_driver fsl_easrc_driver = {
	.probe = fsl_easrc_probe,
	.remove = fsl_easrc_remove,
	.driver = {
		.name = "fsl-easrc",
		.pm = &fsl_easrc_pm_ops,
		.of_match_table = fsl_easrc_dt_ids,
	},
};
module_platform_driver(fsl_easrc_driver);

MODULE_DESCRIPTION("NXP Enhanced Asynchronous Sample Rate (eASRC) driver");
MODULE_LICENSE("GPL v2");
