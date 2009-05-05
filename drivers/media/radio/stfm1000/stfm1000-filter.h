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
#ifndef STFM1000_FILTER_H
#define STFM1000_FILTER_H

/* STFM1000 Black Box Filter parameters */
struct stfm1000_filter_parms {
	s16 LprXzz;		/* LPR x(n-2) stereo filter */
	s16 LmrXzz;		/* LMR x(n-2) stereo filter */
	s16 LprYzz;		/* LPR y(n-2) stereo filter */
	s16 LmrYzz;		/* LMR y(n-2) stereo filter */

	s16 LprXz;		/* LPR x(n-1) stereo filter */
	s16 LmrXz;		/* LMR x(n-1) stereo filter */
	s16 FilteredLpr;	/* LPR filter output */
	s16 FilteredLmr;	/* LMR filter output */

	s16 LprB0;		/* LPR stereo filter coef */
	s16 LprB1over2;		/* LPR stereo filter coef */
	s16 LprA1over2;		/* LPR stereo filter coef */
	s16 LprA2;		/* LPR stereo filter coef */

	s16 LmrB0;		/* LMR stereo filter coef */
	s16 LmrB1over2;		/* LMR stereo filter coef */
	s16 LmrA1over2;		/* LMR stereo filter coef */
	s16 LmrA2;		/* LMR stereo filter coef */

	s16 LprYz;		/* LPR y(n-1) stereo filter */
	s16 LmrYz;		/* LMR y(n-1) stereo filter */

	s16 Left;		/* left channel audio out */
	s16 Right;		/* right channel audio out */
	s32 LeftLb;		/* left channel dc estimate */
	s32 RightLb;		/* right channel dc estimate */

	u32 RssiDecoded;	/* integer decoded RSSI */

	u16 RssiMant;		/* mantissa of float-coded RSSI */
	s16 RssiLog;		/* 10log10(decoded RSSI) */

	u16 RssiExp;		/* exponent of float-coded RSSI */
	u16 RssiLb;		/* leaky bucket dc of rssi */

	u16 Prssi;		/* power of 2 for RSSI */
	u16 TrueRssi;		/* DC estimate of log RSSI */

	u16 ScaledRssiDecoded;	/* scaled log RSSI */
	s16 Echo;		/* Echo info from HiPass(RSSI) */
	u16 ScaledRssiDecodedZ;	/* history buffer for above */
	u16 ScaledRssiDecodedZz;/* ditto */

	u16 ScaledTrueRssi;	/* scaled version for precision */
	u16 FilteredRssi;	/* Attack/Decay filtered RSSI */
	u16 PrevFilteredRssi;	/* previous version of above */

	u16 EchoLb;		/* DC estimate of Echo energy */
	u16 TrueEcho;		/* scaled version of above */
	u16 FilteredEchoLpr;	/* Attack/Decay filt. Echo */
	u16 PrevFilteredEchoLpr;/* previous version of above */
	u16 FilteredEchoLmr;	/* Attack/Decay filt. Echo */
	u16 PrevFilteredEchoLmr;/* previous version of above */
	s16 GatedEcho;		/* Echo gated by threshold */

	s16 ControlLpr;		/* master control for LPR */
	s16 ControlLmr;		/* master control for LMR */
	u16 LprBw;		/* LPR Bandwidth desired */
	u16 LmrBw;		/* LMR Bandwidth desired */
	u16 LprGa;		/* LPR Gain (SoftMute) desired */
	u16 LmrGa;		/* LMR Gain (Blend) desired */
	u16 ScaledControlLmr;	/* Scaled down version Ctl LMR */
	u16 ScaledControlLpr;	/* Scaled down version Ctl LPR */

	s16 B0M;		/* BW ctl B0 coef slope */
	s16 B0B;		/* BW ctl B0 coef y-intercept */

	u16 B0S;		/* BW ctl B0 coef scale */
	s16 B1over2M;		/* BW ctl B1/2 coef slope */

	s16 B1over2B;		/* BW ctl B1/2 coef y-intercept */
	s16 A1over2B;		/* BW ctl A1/2 coef y-intercept */

	u16 B1over2S;		/* BW ctl B1/2 coef scale */
	u16 A1over2S;		/* BW ctl A1/2 coef scale */

	s16 A1over2M;		/* BW ctl A1/2 coef slope */
	u16 A2S;		/* BW ctl A2 coef scale */

	s16 A2M;		/* BW ctl A2 coef slope */
	s16 A2B;		/* BW ctl A2 coef y-intercept */

	u16 AdjBw;		/* Desired Filter BW scaled into range */

	u16 DecRssi;		/*! Decimation modulo counter */

	s16 ScaleAudio;		/*! Scale factor for Audio Mute */
	u8 MuteAudio;		/*! Control for muting audio */
	u8 PrevMuteAudio;	/*! History of control for muting audio */
	u8 MuteActionFlag;	/*! Indicator of when mute ramping occurs */

	u32 Acc;		/* mimics H/W accumulator */
	s32 Acc_signed;
	s16 temp1_reg;		/* mimics 16 bit register */
	s16 temp2_reg;		/* mimics 16 bit register */
	s16 temp3_reg;		/* mimics 16 bit register */
	s16 temp4_reg;		/* mimics 16 bit register */
	s16 temp5_reg;		/* mimics 16 bit register */

	/* *** Programmable Coefficients */
	u16 pCoefRssiAttack;	/* prog coef RSSI attack */
	u16 pCoefRssiDecay;	/* prog coef RSSI decay */
	u16 pCoefEchoLprAttack;	/* prog coef Echo LPR attack */
	u16 pCoefEchoLprDecay;	/* prog coef Echo LPR decay */
	u16 pCoefEchoLmrAttack;	/* prog coef Echo LMR attack */
	u16 pCoefEchoLmrDecay;	/* prog coef Echo LMR decay */

	u16 pCoefEchoTh;	/* prog coef Echo threshold */

	u16 pCoefEchoScLpr;	/* prog coef scale Echo LPR infl. */
	u16 pCoefEchoScLmr;	/* prog coef scale Echo LMR infl. */
	u16 pCoefEchoShLpr;	/* prog coef shift Echo LPR infl. */
	u16 pCoefEchoShLmr;	/* prog coef shift Echo LMR infl. */

	u16 pCoefLprBwThLo;	/* prog coef Low Th LPR BW */
	u16 pCoefLprBwThHi;	/* prog coef High Th LPR BW */
	u16 pCoefLmrBwThLo;	/* prog coef Low Th LMR BW */
	u16 pCoefLmrBwThHi;	/* prog coef High Th LMR BW */

	u16 pCoefLprGaTh;	/* prog coef Th LPR Gain (SoftMute) */
	u16 pCoefLmrGaTh;	/* prog coef Th LMR Gain (Blend) */

	u16 pCoefLprBwSlSc;	/* prog coef Slope scale LPR BW */
	u16 pCoefLprBwSlSh;	/* prog coef Slope shift LPR BW */
	u16 pCoefLmrBwSlSc;	/* prog coef Slope scale LMR BW */
	u16 pCoefLmrBwSlSh;	/* prog coef Slope shift LMR BW */
	u16 pCoefLprGaSlSc;	/* prog coef Slope scale LPR Gain */
	u16 pCoefLprGaSlSh;	/* prog coef Slope shift LPR Gain */

	u8 pCoefForcedMono;	/* Forced Mono control bit */
	u8 pCoefBypassBlend;	/* Forced bypass of stereo blend */
	u8 pCoefBypassSoftmute;	/* Forced bypass of softmute */
	u8 pCoefBypassDcCut;	/* Forced bypass of audio DC Cut filter */

	u8 pCoefBypassBwCtl;	/* Forced bypass of bandwidth control */
	u8 pCoefForceLockLmrBw;	/* prog flag to force LMR BW=LPR BW */

	/* XXX added here, they were global */
	s16 temp2_reg_sm;
	s16 temp3_reg_sm;

};

/* STFM1000 Black Box Filter Function prototypes */
void stfm1000_filter_reset(struct stfm1000_filter_parms *sdf);
void stfm1000_filter_decode(struct stfm1000_filter_parms *sdf, s16 Lpr,
	s16 Lmr, u16 Rssi);

static inline s16
stfm1000_filter_value_left(struct stfm1000_filter_parms *sdf)
{
	return sdf->Left;
}

static inline s16
stfm1000_filter_value_right(struct stfm1000_filter_parms *sdf)
{
	return sdf->Right;
}

static inline u32
stfm1000_filter_value_rssi(struct stfm1000_filter_parms *sdf)
{
	return sdf->RssiDecoded;
}

#endif
