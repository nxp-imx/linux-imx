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
#include <linux/init.h>

#include "stfm1000.h"

void stfm1000_filter_reset(struct stfm1000_filter_parms *sdf)
{
	sdf->Left = 0;
	sdf->Right = 0;
	sdf->RssiDecoded = 0;
	sdf->RssiMant = 0;
	sdf->RssiExp = 0;
	sdf->RssiLb = 0;
	sdf->TrueRssi = 0;
	sdf->Prssi = 0;
	sdf->RssiLog = 0;
	sdf->ScaledTrueRssi = 0;
	sdf->FilteredRssi = 0;
	sdf->PrevFilteredRssi = 0;
	sdf->DecRssi = 0;
	sdf->ScaledRssiDecoded = 0;
	sdf->ScaledRssiDecodedZ = 0;
	sdf->ScaledRssiDecodedZz = 0;
	sdf->Echo = 0;
	sdf->EchoLb = 0;
	sdf->TrueEcho = 0;
	sdf->FilteredEchoLpr = 0;
	sdf->PrevFilteredEchoLpr = 0;
	sdf->FilteredEchoLmr = 0;
	sdf->PrevFilteredEchoLmr = 0;
	sdf->GatedEcho = 0;
	sdf->ControlLpr = 0;
	sdf->ControlLmr = 0;
	sdf->LprBw = 0;
	sdf->LmrBw = 0;

	sdf->LprXz = 0;
	sdf->LprXzz = 0;
	sdf->LprYz = 0;
	sdf->LprYzz = 0;
	sdf->LmrXz = 0;
	sdf->LmrXzz = 0;
	sdf->LmrYz = 0;
	sdf->LmrYzz = 0;
	sdf->FilteredLpr = 0;
	sdf->FilteredLmr = 0;

	sdf->B0B = 0;
	sdf->B0S = 0;
	sdf->B0M = 0;
	sdf->B1over2B = 0;
	sdf->B1over2S = 0;
	sdf->B1over2M = 0;
	sdf->A1over2B = 0;
	sdf->A1over2S = 0;
	sdf->A1over2M = 0;
	sdf->A2B = 0;
	sdf->A2S = 0;
	sdf->A2M = 0;

	sdf->AdjBw = 0;

	sdf->pCoefLprBwThLo = 20 << 8;
	sdf->pCoefLprBwThHi = 30 << 8;
	sdf->pCoefLmrBwThLo = 40 << 8;
	sdf->pCoefLmrBwThHi = 50 << 8;
	sdf->pCoefLprBwSlSc = 4800;	/* SDK-2287 */
	sdf->pCoefLprBwSlSh = 10;	/* SDK-2287 */
	sdf->pCoefLmrBwSlSc = 4800;	/* SDK-2287 */
	sdf->pCoefLmrBwSlSh = 10;	/* SDK-2287 */
	sdf->pCoefLprGaSlSc = 0;
	sdf->pCoefLprGaSlSh = 0;

	sdf->ScaledControlLmr = 0;

	sdf->LprGa = 32767;
	sdf->LmrGa = 32767;

	sdf->pCoefLprGaTh = 20;		/* 25 */
	sdf->pCoefLmrGaTh = 55;		/* 60 50 */

	sdf->MuteAudio = 0;
	sdf->PrevMuteAudio = 0;
	sdf->MuteActionFlag = 0;
	sdf->ScaleAudio = 0;

	/* *** Programmable initial setup for stereo path filters */
	sdf->LprB0 = 18806;		/* -3dB cutoff = 17 kHz */
	sdf->LprB1over2 = 18812;	/* -3dB cutoff = 17 kHz */
	sdf->LprA1over2 = -16079;	/* -3dB cutoff = 17 kHz */
	sdf->LprA2 = -11125;		/* -3dB cutoff = 17 kHz */
	sdf->LmrB0 = 18806;		/* -3dB cutoff = 17 kHz */
	sdf->LmrB1over2 = 18812;	/* -3dB cutoff = 17 kHz */
	sdf->LmrA1over2 = -16079;	/* -3dB cutoff = 17 kHz */
	sdf->LmrA2 = -11125;		/* -3dB cutoff = 17 kHz */

	sdf->pCoefForceLockLmrBw = 0;	/* Force Lock LMR BW = LPR BW
					 * XXX BUG WARNING -
					 * This control doesn't work! */

	sdf->pCoefForcedMono = 0;	/* Do not set this =
					 * Quality Monitor will overwrite it */
	sdf->pCoefBypassBlend = 0;	/* BUG WARNING -
					 * This control doesn't work! */
	sdf->pCoefBypassSoftmute = 0;	/* BUG WARNING -
					 * This control doesn't work! */
	sdf->pCoefBypassBwCtl = 0;	/* BUG WARNING -
					 * This control doesn't work! */

	/* There's a bug or something in the attack/decay section b/c
	 * setting these coef's to anything */
	/* higher than 100ms or so causes the RSSI to be artificially low -
	 * Needs investigation! 15DEC06 */
	sdf->pCoefRssiAttack = 65386;		/* changed to 100ms to avoid
						 * stereo crackling
						 * 60764 corresponds to 3 */
	sdf->pCoefRssiDecay = 65386;		/* changed to 100ms to avoid
						 * stereo crackling
						 * 65530 corresponds to 10 */
	sdf->pCoefEchoLprAttack = 52239;	/* corresponds to 1 */
	sdf->pCoefEchoLprDecay = 64796;		/* corresponds to 20 */
	sdf->pCoefEchoLmrAttack = 52239;	/* corresponds to 1 */
	sdf->pCoefEchoLmrDecay = 65520;		/* corresponds to 20 */
	sdf->pCoefEchoTh = 100;
	sdf->pCoefEchoScLpr = (u16) (0.9999 * 32767.0);
	sdf->pCoefEchoScLmr = (u16) (0.9999 * 32767.0);
}

void stfm1000_filter_decode(struct stfm1000_filter_parms *sdf, s16 Lpr,
	s16 Lmr, u16 Rssi)
{
	s16 temp1_reg;		/* mimics 16 bit register */
	s16 temp2_reg;		/* mimics 16 bit register */
	s16 temp3_reg;		/* mimics 16 bit register */
	s16 temp4_reg;		/* mimics 16 bit register */
#ifndef _TUNER_STFM_MUTE
	s16 temp5_reg;		/* mimics 16 bit register */
#endif
	s32 temp2_reg_32;	/*eI 108 27th Feb 06 temp variables. */

	/* **************************************************************** */
	/* *** Stereo Processing ****************************************** */
	/* **************************************************************** */
	/* *** This block operates at Fs = 44.1kHz */
	/* ******** */
	/* *** LPR path filter (2nd order IIR) */

	sdf->Acc_signed = sdf->LprB0 * Lpr + 2 * (sdf->LprB1over2 * sdf->LprXz)
	    + sdf->LprB0 * sdf->LprXzz + 2 * (sdf->LprA1over2 * sdf->LprYz)
	    + sdf->LprA2 * sdf->LprYzz;

	sdf->FilteredLpr = sdf->Acc_signed >> 15;

	sdf->LprXzz = sdf->LprXz;	/* update taps */
	sdf->LprXz = Lpr;
	sdf->LprYzz = sdf->LprYz;
	sdf->LprYz = sdf->FilteredLpr;

	/* *** LMR path filter (2nd order IIR) */
	sdf->Acc_signed = sdf->LmrB0 * Lmr + 2 * (sdf->LmrB1over2 * sdf->LmrXz)
	    + sdf->LmrB0 * sdf->LmrXzz + 2 * (sdf->LmrA1over2 * sdf->LmrYz)
	    + sdf->LmrA2 * sdf->LmrYzz;

	sdf->FilteredLmr = sdf->Acc_signed >> 15;

	sdf->LmrXzz = sdf->LmrXz;	/* update taps */
	sdf->LmrXz = Lmr;
	sdf->LmrYzz = sdf->LmrYz;
	sdf->LmrYz = sdf->FilteredLmr;

	/* *** Stereo Matrix */
	if (0 == sdf->pCoefBypassBlend)
		temp1_reg = sdf->LmrGa * sdf->FilteredLmr >> 15; /* Blend */
	else
		temp1_reg = sdf->FilteredLmr;

	if (sdf->pCoefForcedMono)	/* Forced Mono */
		temp1_reg = 0;

	if (0 == sdf->pCoefBypassSoftmute) {
		temp2_reg = sdf->LprGa * sdf->FilteredLpr >> 15; /* LPR */
		temp3_reg = sdf->LprGa * temp1_reg >> 15;	 /* LMR */
	} else {
		temp2_reg = sdf->FilteredLpr;
		temp3_reg = temp1_reg;
	}

	temp4_reg = (temp2_reg + temp3_reg) / 2;	/* Matrix */

#ifndef _TUNER_STFM_MUTE
	temp5_reg = (temp2_reg - temp3_reg) / 2;
#endif

#if 0
	/* *** DC Cut Filter (leaky bucket estimate) */
	if (0 == sdf->pCoefBypassDcCut) {
		sdf->LeftLb_i32 =
		    sdf->LeftLb_i32 + temp4_reg - (sdf->LeftLb_i32 >> 8);
		temp2_reg = temp4_reg - (sdf->LeftLb_i32 >> 8);	/* signal -
								dc_estimate */

		sdf->RightLb_i32 =
		    sdf->RightLb_i32 + temp5_reg - (sdf->RightLb_i32 >> 8);
		temp3_reg = temp5_reg - (sdf->RightLb_i32 >> 8); /* signal -
								dc_estimate */
	} else {
		temp2_reg = temp4_reg;
		temp3_reg = temp5_reg;
	}
#endif
#ifdef _TUNER_STFM_MUTE
	/* *** Mute Audio */
	if (sdf->MuteAudio != sdf->PrevMuteAudio)	/* Mute transition */
		sdf->MuteActionFlag = 1;	/* set flag */
	sdf->PrevMuteAudio = sdf->MuteAudio;	/* update history */

	if (sdf->MuteActionFlag) {
		if (0 == sdf->MuteAudio) {	/* Mute to zero */
			/* gradual mute down */
			sdf->ScaleAudio = sdf->ScaleAudio - sdf->pCoefMuteStep;

			/* eI-117:Oct28:as per C++ code */
			/* if (0 < sdf->ScaleAudio) */
			if (0 > sdf->ScaleAudio) {
				sdf->ScaleAudio = 0;	 /* Minimum scale
							  * factor */
				sdf->MuteActionFlag = 0; /* End Mute Action */
			}
		} else {	/* Un-Mute to one */
				/* gradual mute up */
			sdf->ScaleAudio = sdf->ScaleAudio + sdf->pCoefMuteStep;
			if (0 > sdf->ScaleAudio) {	/* look for rollover
							 * beyong 32767 */
				sdf->ScaleAudio = 32767; /* Maximum scale
							  * factor */
				sdf->MuteActionFlag = 0; /* End Mute Action */
			}
		}		/* end else */
	}			/* end if (sdf->MuteActionFlag) */

/*! Output Processed Sample */

	sdf->Left = (temp2_reg * sdf->ScaleAudio) >> 15;	/* Scale */
	sdf->Right = (temp3_reg * sdf->ScaleAudio) >> 15;	/* Scale */

#else /* !_TUNER_STFM_MUTE */

	sdf->Left = temp4_reg;
	sdf->Right = temp5_reg;

#endif /* !_TUNER_STFM_MUTE */

	/* *** End Stereo Processing ************************************** */
	/* **************************************************************** */

	/* **************************************************************** */
	/* *** Signal Quality Indicators ********************************** */
	/* **************************************************************** */
	/* *** This block operates at Fs = 44.1kHz */
	/* ******** */
	/* *** RSSI */
	/* ******** */
	/* Decode Floating Point RSSI data */
	/*! Input RSSI sample */
	sdf->RssiMant = (Rssi & 0xFFE0) >> 5;	/* 11 msb's */
	sdf->RssiExp = Rssi & 0x001F;	/* 5 lsb's */
	sdf->RssiDecoded = sdf->RssiMant << sdf->RssiExp;

	/* *** Convert RSSI to 10*Log10(RSSI) */
	/* This is easily accomplished in DSP code using the CLZ instruction */
	/* rather than using all these comparisons. */
	/* The basic idea is this: */
	/* if x >= 2^P */
	/*   f(x) = 3*x>>P + (3*P-3) */
	/* Approx. is valid over the range of sdf->RssiDecoded in [0, 2^21] */
	/* *** */
	if (sdf->RssiDecoded >= 1048576)
		sdf->Prssi = 20;
	else if (sdf->RssiDecoded >= 524288)
		sdf->Prssi = 19;
	else if (sdf->RssiDecoded >= 262144)
		sdf->Prssi = 18;
	else if (sdf->RssiDecoded >= 131072)
		sdf->Prssi = 17;
	else if (sdf->RssiDecoded >= 65536)
		sdf->Prssi = 16;
	else if (sdf->RssiDecoded >= 32768)
		sdf->Prssi = 15;
	else if (sdf->RssiDecoded >= 16384)
		sdf->Prssi = 14;
	else if (sdf->RssiDecoded >= 8192)
		sdf->Prssi = 13;
	else if (sdf->RssiDecoded >= 4096)
		sdf->Prssi = 12;
	else if (sdf->RssiDecoded >= 2048)
		sdf->Prssi = 11;
	else if (sdf->RssiDecoded >= 1024)
		sdf->Prssi = 10;
	else if (sdf->RssiDecoded >= 512)
		sdf->Prssi = 9;
	else if (sdf->RssiDecoded >= 256)
		sdf->Prssi = 8;
	else if (sdf->RssiDecoded >= 128)
		sdf->Prssi = 7;
	else if (sdf->RssiDecoded >= 64)
		sdf->Prssi = 6;
	else if (sdf->RssiDecoded >= 32)
		sdf->Prssi = 5;
	else if (sdf->RssiDecoded >= 16)
		sdf->Prssi = 4;
	else if (sdf->RssiDecoded >= 8)
		sdf->Prssi = 3;
	else if (sdf->RssiDecoded >= 4)
		sdf->Prssi = 2;
	else if (sdf->RssiDecoded >= 2)
		sdf->Prssi = 1;
	else
		sdf->Prssi = 0;
	sdf->RssiLog =
	    (3 * sdf->RssiDecoded >> sdf->Prssi) + (3 * sdf->Prssi - 3);

	if (0 > sdf->RssiLog)	/* Clamp to positive */
		sdf->RssiLog = 0;

	/* Compensate for errors in truncation/approximation by adding 1 */
	sdf->RssiLog = sdf->RssiLog + 1;

	/* Leaky Bucket Filter DC estimate of RSSI */
	sdf->RssiLb = sdf->RssiLb + sdf->RssiLog - (sdf->RssiLb >> 3);
	sdf->TrueRssi = sdf->RssiLb >> 3;

	/* Scale up so we have some room for precision */
	sdf->ScaledTrueRssi = sdf->TrueRssi << 8;
	/* ************ */
	/* *** end RSSI */
	/* ************ */

	/* ******** */
	/* *** Echo */
	/* ******** */
	/* *** Isolate Echo information as higher frequency info */
	/* using [1 -2 1] highpass FIR */
	sdf->ScaledRssiDecoded = sdf->RssiDecoded >> 4;
	sdf->Echo =
	    (s16) ((sdf->ScaledRssiDecoded -
		    2 * sdf->ScaledRssiDecodedZ + sdf->ScaledRssiDecodedZz));
	sdf->ScaledRssiDecodedZz = sdf->ScaledRssiDecodedZ;
	sdf->ScaledRssiDecodedZ = sdf->ScaledRssiDecoded;
	/* ************ */
	/* *** end Echo */
	/* ************ */
	/* *** End Signal Quality Indicators ******************************* */
	/* ***************************************************************** */

	/* ***************************************************************** */
	/* *** Weak Signal Processing ************************************** */
	/* ***************************************************************** */
	/* *** This block operates at Fs = 44.1/16 = 2.75 Khz
	 * *eI 108 28th Feb 06 WSP and SM executes at 2.75Khz */
	/* decimate by 16                   STFM_FILTER_BLOCK_MULTIPLE is 16 */
	if (0 == sdf->DecRssi) {
		/* *** Filter RSSI via attack/decay structure */
		if (sdf->ScaledTrueRssi > sdf->PrevFilteredRssi)
			sdf->Acc =
			    sdf->pCoefRssiAttack *
			    sdf->PrevFilteredRssi + (65535 -
						     sdf->pCoefRssiAttack)
			    * sdf->ScaledTrueRssi;
		else
			sdf->Acc =
			    sdf->pCoefRssiDecay *
			    sdf->PrevFilteredRssi + (65535 -
						     sdf->pCoefRssiDecay)
			    * sdf->ScaledTrueRssi;

		sdf->FilteredRssi = sdf->Acc >> 16;
		sdf->PrevFilteredRssi = sdf->FilteredRssi;

		/* *** Form Echo "energy" representation */
		if (0 > sdf->Echo)
			sdf->Echo = -sdf->Echo;	/* ABS() */

		/* Threshold compare */
		sdf->GatedEcho = (s16) (sdf->Echo - sdf->pCoefEchoTh);
		if (0 > sdf->GatedEcho)	/* Clamp to (+)ve */
			sdf->GatedEcho = 0;

		/* *** Leaky bucket DC estimate of Echo energy */
		sdf->EchoLb = sdf->EchoLb + sdf->GatedEcho -
				(sdf->EchoLb >> 3);
		sdf->TrueEcho = sdf->EchoLb >> 3;

		/* *** Filter Echo via attack/decay structure for LPR */
		if (sdf->TrueEcho > sdf->PrevFilteredEchoLpr)
			sdf->Acc =
			    sdf->pCoefEchoLprAttack *
			    sdf->PrevFilteredEchoLpr +
				(65535 - sdf->pCoefEchoLprAttack) *
					sdf->TrueEcho;
		else
			sdf->Acc =
			    sdf->pCoefEchoLprDecay *
			    sdf->PrevFilteredEchoLpr +
				(65535 - sdf->pCoefEchoLprDecay) *
					sdf->TrueEcho;

		sdf->FilteredEchoLpr = sdf->Acc >> 16;
		sdf->PrevFilteredEchoLpr = sdf->FilteredEchoLpr;

		/* *** Filter Echo via attack/decay structure for LMR */
		if (sdf->TrueEcho > sdf->PrevFilteredEchoLmr)
			sdf->Acc = sdf->pCoefEchoLmrAttack *
				sdf->PrevFilteredEchoLmr +
					(65535 - sdf->pCoefEchoLmrAttack)
						* sdf->TrueEcho;
		else
			sdf->Acc =
			    sdf->pCoefEchoLmrDecay *
			    sdf->PrevFilteredEchoLmr + (65535 -
							sdf->pCoefEchoLmrDecay)
			    * sdf->TrueEcho;

		sdf->FilteredEchoLmr = sdf->Acc >> 16;
		sdf->PrevFilteredEchoLmr = sdf->FilteredEchoLmr;

		/* *** Form control variables */
		/* Generically speaking, ctl = f(RSSI, Echo) =
		 * RSSI - (a*Echo)<<b, where a,b are programmable */
		sdf->ControlLpr = sdf->FilteredRssi -
		    ((sdf->pCoefEchoScLpr *
		      sdf->FilteredEchoLpr << sdf->pCoefEchoShLpr) >> 15);
		if (0 > sdf->ControlLpr)
			sdf->ControlLpr = 0;	/* Clamp to positive */

		sdf->ControlLmr = sdf->FilteredRssi -
		    ((sdf->pCoefEchoScLmr *
		      sdf->FilteredEchoLmr << sdf->pCoefEchoShLmr) >> 15);
		if (0 > sdf->ControlLmr)
			sdf->ControlLmr = 0;	/* Clamp to positive */

		/* *** Define LPR_BW = f(control LPR) */
		/* Assume that 5 kHz and 17 kHz are limits of LPR_BW control */
		if (sdf->ControlLpr <= sdf->pCoefLprBwThLo)
			sdf->LprBw = 5000;	/* lower limit is 5 kHz */
		else if (sdf->ControlLpr >= sdf->pCoefLprBwThHi)
			sdf->LprBw = 17000;	/* upper limit is 17 kHz */
		else
			sdf->LprBw = 17000 -
			    ((sdf->pCoefLprBwSlSc *
			      (sdf->pCoefLprBwThHi -
			       sdf->ControlLpr)) >> sdf->pCoefLprBwSlSh);

		/* *** Define LMR_BW = f(control LMR) */
		/* Assume that 5 kHz and 17 kHz are limits of LPR_BW control */
		if (0 == sdf->pCoefForceLockLmrBw) {	/* only do these calc's
							 * if LMR BW not
							 * ForceLocked */
			if (sdf->ControlLmr <= sdf->pCoefLmrBwThLo)
				sdf->LmrBw = 5000;	/* lower limit is
							 * 5 kHz */
			else if (sdf->ControlLmr >= sdf->pCoefLmrBwThHi)
				sdf->LmrBw = 17000;	/* upper limit is
							 * 17 kHz */
			else
				sdf->LmrBw = 17000 -
				    ((sdf->pCoefLmrBwSlSc *
				      (sdf->pCoefLmrBwThHi -
				       sdf->ControlLmr)) >>
				     sdf->pCoefLmrBwSlSh);
		}
		/* *** Define LMR_Gain = f(control LMR)
		 * Assume that Blending occurs across 20 dB range of
		 * control LMR. For sake of listenability, approximate
		 * antilog blending curve
		 * To simplify antilog approx, scale control LMR back into
		 * "RSSI in dB range" [0,60] */
		sdf->ScaledControlLmr = sdf->ControlLmr >> 8;

		/* how far below blend threshold are we? */
		temp1_reg = sdf->pCoefLmrGaTh - sdf->ScaledControlLmr;
		if (0 > temp1_reg)	/* We're not below threshold,
					 * so no blending needed */
			temp1_reg = 0;
		temp2_reg = 20 - temp1_reg;	/* Blend range = 20 dB */
		if (0 > temp2_reg)
			temp2_reg = 0;	/* if beyond that range,
					 * then clamp to 0 */

		/* We want stereo separation (n dB) to rolloff linearly over
		 * the 20 dB wide blend region.
		 * this necessitates a particular rolloff for the blend
		 * parameter, which is not obvious.
		 * See sw_audio/log_approx.m for calculation of this rolloff,
		 * implemented below...
		 * Note that stereo_separation (in dB) = 20*log10((1+a)/(1-a)),
		 * where a = blend scaler
		 * appropriately scaled for 2^15.  This relationship sits at
		 * the heart of why this curve is needed. */
		if (15 <= temp2_reg)
			temp3_reg = 264 * temp2_reg + 27487;
		else if (10 <= temp2_reg)
			temp3_reg = 650 * temp2_reg + 21692;
		else if (5 <= temp2_reg)
			temp3_reg = 1903 * temp2_reg + 9166;
		else
			temp3_reg = 3736 * temp2_reg;

		sdf->LmrGa = temp3_reg;

		if (32767 < sdf->LmrGa)
			sdf->LmrGa = 32767;	/* Clamp to '1' */

		/* *** Define LPR_Gain = f(control LPR)
		 * Assume that SoftMuting occurs across 20 dB range of
		 * control LPR
		 * For sake of listenability, approximate antilog softmute
		 * curve To simplify antilog approx, scale control LPR back
		 * into "RSSI in dB range" [0,60] */
		sdf->ScaledControlLpr = sdf->ControlLpr >> 8;
		/* how far below softmute threshold are we? */
		temp1_reg = sdf->pCoefLprGaTh - sdf->ScaledControlLpr;
		if (0 > temp1_reg)	/* We're not below threshold,
					 * so no softmute needed */
			temp1_reg = 0;
		temp2_reg = 20 - temp1_reg;	/* SoftmMute range = 20 dB */
		if (0 > temp2_reg)
			temp2_reg = 0;	/* if beyond that range,
					 * then clamp to 0 */
		/* Form 100*10^((temp2_reg-20)/20) approximation (antilog)
		 * over range [0,20] dB
		 * approximation in range [0,100], but we only want to
		 * softmute down to -20 dB, no further */
		if (16 < temp2_reg)
			temp3_reg = 9 * temp2_reg - 80;
		else if (12 < temp2_reg)
			temp3_reg = 6 * temp2_reg - 33;
		else if (8 < temp2_reg)
			temp3_reg = 4 * temp2_reg - 8;
		else
			temp3_reg = 2 * temp2_reg + 9;

		sdf->LprGa = 328 * temp3_reg;	/* close to 32767*(1/100) */

		if (32767 < sdf->LprGa)
			sdf->LprGa = 32767;	/* Clamp to '1' */

		if (3277 > sdf->LprGa)
			sdf->LprGa = 3277;	/* Clamp to 0.1*32767 =
						 * -20 dB min gain */

		/* *************** Bandwidth Sweep Algorithm ************ */
		/* *** Calculate 2nd order filter coefficients as function
		 * of desired BW. We do this by constructing piece-wise
		 * linear filter coef's as f(BW), which is why we break the
		 * calc's into different BW regions below.
		 * coef(BW) = S*(M*BW + B)
		 * For more info, see sw_audio/ws_filter.m checked into CVS */
		if (0 == sdf->pCoefBypassBwCtl)	{ /* if ==1, then we just go
						   * with default coef set */
			/* determine if we run thru loop once or twice... */
			if (1 == sdf->pCoefForceLockLmrBw)
				temp4_reg = 1;	/* run thru once only to calc.
						 * LPR coef's */
			else
				temp4_reg = 2;	/* run thru twice to calc.
						 * LPR and LMR coef's */

			/* Here's the big coef. calc. loop */
			for (temp1_reg = 0; temp1_reg < temp4_reg;
				temp1_reg++) {

				if (0 == temp1_reg)
					temp2_reg = (s16) sdf->LprBw;
				else
					temp2_reg = (s16) sdf->LmrBw;


				if (6000 > temp2_reg) {
					/* interval = [4.4kHz, 6.0kHz) */
					sdf->B0M = 22102;
					sdf->B0B = -2209;
					sdf->B0S = 1;

					sdf->B1over2M = 22089;
					sdf->B1over2B = -2205;
					sdf->B1over2S = 1;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = -24664;
					sdf->A2B = 11698;
					sdf->A2S = 2;
				} else if (8000 > temp2_reg) {
					/* interval = [6.0kHz, 8.0kHz) */
					sdf->B0M = 22102;
					sdf->B0B = -2209;
					sdf->B0S = 1;

					sdf->B1over2M = 22089;
					sdf->B1over2B = -2205;
					sdf->B1over2S = 1;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = -31231;
					sdf->A2B = 18468;
					sdf->A2S = 1;
				} else if (10000 > temp2_reg) {
					/* interval = [8.0kHz, 10.0kHz) */
					sdf->B0M = 28433;
					sdf->B0B = -4506;
					sdf->B0S = 1;

					sdf->B1over2M = 28462;
					sdf->B1over2B = -4584;
					sdf->B1over2S = 1;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = -14811;
					sdf->A2B = 12511;
					sdf->A2S = 1;
				} else if (12000 > temp2_reg) {
					/* interval = [10.0kHz, 12.0kHz) */
					sdf->B0M = 28433;
					sdf->B0B = -4506;
					sdf->B0S = 1;

					sdf->B1over2M = 28462;
					sdf->B1over2B = -4584;
					sdf->B1over2S = 1;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = -181;
					sdf->A2B = 5875;
					sdf->A2S = 1;
				} else if (14000 > temp2_reg) {
					/* interval = [12.0kHz, 14.0kHz) */
					sdf->B0M = 18291;
					sdf->B0B = -4470;
					sdf->B0S = 2;

					sdf->B1over2M = 18461;
					sdf->B1over2B = -4597;
					sdf->B1over2S = 2;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = 14379;
					sdf->A2B = -2068;
					sdf->A2S = 1;
				} else if (16000 > temp2_reg) {
					/* interval = [14.0kHz, 16.0kHz) */
					sdf->B0M = 18291;
					sdf->B0B = -4470;
					sdf->B0S = 2;

					sdf->B1over2M = 18461;
					sdf->B1over2B = -4597;
					sdf->B1over2S = 2;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = 30815;
					sdf->A2B = -12481;
					sdf->A2S = 1;
				} else if (18000 > temp2_reg) {
					/* interval = [16.0kHz, 18.0kHz) */
					sdf->B0M = 24740;
					sdf->B0B = -9152;
					sdf->B0S = 2;

					sdf->B1over2M = 24730;
					sdf->B1over2B = -9142;
					sdf->B1over2S = 2;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = 25631;
					sdf->A2B = -13661;
					sdf->A2S = 2;
				} else {
					/* interval = [18.0kHz, 19.845kHz) */
					sdf->B0M = 24740;
					sdf->B0B = -9152;
					sdf->B0S = 2;

					sdf->B1over2M = 24730;
					sdf->B1over2B = -9142;
					sdf->B1over2S = 2;

					sdf->A1over2M = 31646;
					sdf->A1over2B = -15695;
					sdf->A1over2S = 2;

					sdf->A2M = 19382;
					sdf->A2B = -12183;
					sdf->A2S = 4;
				}

				if (0 == temp1_reg) {
					/* The piece-wise linear eq's are
					 * based on a scaled version
					 * (32768/22050) of BW */

					/* Note 32768/22050 <-> 2*(16384/22050)
					 * <-> 2*((16384/22050)*32768)>>15 */
					sdf->AdjBw = ((temp2_reg << 1) *
						24348) >> 15;

					/* temp = mx */
					temp3_reg = (sdf->B0M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LprB0 = sdf->B0S *
						(temp3_reg + sdf->B0B);

					/* temp = mx */
					temp3_reg = (sdf->B1over2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LprB1over2 = sdf->B1over2S *
						(temp3_reg + sdf->B1over2B);

					/* temp = mx */
					temp3_reg = (sdf->A1over2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LprA1over2 = -sdf->A1over2S *
						(temp3_reg + sdf->A1over2B);

					/* temp = mx */
					temp3_reg = (sdf->A2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LprA2 = -sdf->A2S *
						(temp3_reg + sdf->A2B);
					/* *** end LPR channel --
					 * LPR coefficients now ready for
					 * Stereo Path next time */
				} else {
					/* The piece-wise linear eq's are
					 * based on a scaled version
					 * (32768/22050) of BW */

					/* Note 32768/22050 <-> 2*(16384/22050)
					 * <-> 2*((16384/22050)*32768)>>15 */
					sdf->AdjBw = ((temp2_reg << 1) *
						24348) >> 15;

					/* temp = mx */
					temp3_reg = (sdf->B0M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LmrB0 = sdf->B0S *
						(temp3_reg + sdf->B0B);

					/* temp = mx */
					temp3_reg = (sdf->B1over2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LmrB1over2 = sdf->B1over2S *
						(temp3_reg + sdf->B1over2B);

					/* temp = mx */
					temp3_reg = (sdf->A1over2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LmrA1over2 = -sdf->A1over2S *
						(temp3_reg + sdf->A1over2B);

					/* temp = mx */
					temp3_reg = (sdf->A2M *
						sdf->AdjBw) >> 15;

					/* y = S*(mx + b) */
					sdf->LmrA2 = -sdf->A2S *
						(temp3_reg + sdf->A2B);
					/* *** end LMR channel -- LMR
					 * coefficients now ready for Stereo
					 * Path next time */
				}
			}	/* end for (temp1_reg=0... */
			if (1 == sdf->pCoefForceLockLmrBw) {
				/* if Force Lock LMR BW = LPR BW */
				/* then set LMR coef's = LPR coef's */
				sdf->LmrB0 = sdf->LprB0;
				sdf->LmrB1over2 = sdf->LprB1over2;
				sdf->LmrA1over2 = sdf->LprA1over2;
				sdf->LmrA2 = sdf->LprA2;
			}

		}		/* end if (0 == sdf->pCoef_BypassBwCtl) */
		/* eI 108 24th Feb 06 Streo Matrix part moved after
		 * weak signal processing. */
		if (0 == sdf->pCoefBypassBlend)
			temp1_reg = sdf->LmrGa;	/* Blend */
		else
			temp1_reg = 1;

		if (sdf->pCoefForcedMono)	/* Forced Mono */
			temp1_reg = 0;

		if (0 == sdf->pCoefBypassSoftmute) {

			/* SoftMute applied to LPR */
			sdf->temp2_reg_sm = sdf->LprGa;

			temp2_reg_32 = sdf->LprGa * temp1_reg;

			/* SoftMute applied to LMR */
			sdf->temp3_reg_sm = (temp2_reg_32) >> 15;
		} else {
			sdf->temp2_reg_sm = 1;	/* eI 108 24th Feb 06 update
						 * global variable for IIR
						 * filter. */
			sdf->temp3_reg_sm = temp1_reg;
		}

	}			/* end if (0 == sdf->DecRssi) */

	sdf->DecRssi = ((sdf->DecRssi + 1) % 16);	/* end decimation
							 * by 16 */

	/* *** End Weak Signal Processing ********************************** */
	/* ***************************************************************** */
}
