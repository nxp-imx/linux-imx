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

#include "stfm1000-rds.h"

#define bitstream_to_rds_state(b)	\
	container_of(b, struct stfm1000_rds_state, bitstream)
#define demod_to_rds_state(d)		\
	container_of(d, struct stfm1000_rds_state, demod)
#define pkt_to_rds_state(p)		\
	container_of(p, struct stfm1000_rds_state, pkt)
#define text_to_rds_state(t)		\
	container_of(t, struct stfm1000_rds_state, text)
#define rds_state_to_stfm1000(r)	\
	container_of(r, struct stfm1000, rds_state)

#define TADJSH 8 /*Shifts used in bitslice loop filter */

/* Reverse of Matlab's Fquant (see MatchedFilterDecomposition.m), so that */
/* mixandsum code is easy;   Used by rds_bitstream_stfmdemod.arm */
const s16 u16_rds_basis[2*RDS_BASISLENGTH+8] = {
	14, 24, 34, 43, 50, 56, 60, 62, 62,
	60, 55, 49, 41, 32, 22, 11, 14, 24,
	34, 43, 50, 56, 60, 62, 62, 60, 55,
	49, 41, 32, 22, 11, 14, 24, 34, 43,
	50, 56, 60, 62
};

static int bits_free(struct stfm1000_rds_bitstream *rdsb)
{
	/* Do not show the last one word free. */
	int FreeSpace = rdsb->TailBitCount - rdsb->HeadBitCount - 32;

	if (FreeSpace < 0)
		FreeSpace = (RDS_BITBUFSIZE * 32) + FreeSpace;
	return FreeSpace;
}

static void put1bit(struct stfm1000_rds_bitstream *rdsb, int bit)
{
	int index = (rdsb->HeadBitCount >> 5);
	u32 CurBit = (rdsb->HeadBitCount & 0x1f);
	u32 CurWord = rdsb->buf[index];

	if (CurBit == 0)
		CurWord = 0;

	CurWord = CurWord | (((u32)bit & 1) << CurBit);
	rdsb->buf[index] = CurWord;
	rdsb->HeadBitCount++;
	if (rdsb->HeadBitCount >= RDS_BITBUFSIZE * 32)
		rdsb->HeadBitCount = 0;
}

static int get1bit(struct stfm1000_rds_bitstream *rdsb)
{
	int Bit = 0;
	int index = (rdsb->TailBitCount >> 5);
	int CurBit = (rdsb->TailBitCount & 0x1f);
	u32 CurWord = rdsb->buf[index];

	Bit = (CurWord >> CurBit) & 1;
	rdsb->TailBitCount++;
	if (rdsb->TailBitCount == RDS_BITBUFSIZE*32)
		rdsb->TailBitCount = 0;

	return Bit;
}

static int bits_filled(struct stfm1000_rds_bitstream *rdsb)
{
	int FilledSpace = rdsb->HeadBitCount - rdsb->TailBitCount;

	if (FilledSpace < 0)
		FilledSpace = (RDS_BITBUFSIZE * 32) + FilledSpace;
	return FilledSpace;
}

static void rds_mix_msg(struct stfm1000_rds_demod *rdsd, u8 MixSetting)
{
	if (rdsd->mix_msg_pending)
		rdsd->mix_msg_overrun++;
	rdsd->mix_msg = MixSetting;
	rdsd->mix_msg_pending = 1;

	/* signal monitor thread */
	stfm1000_monitor_signal(
		rds_state_to_stfm1000(demod_to_rds_state(rdsd)),
		EVENT_RDS_MIXFILT);
}

/* call with interrupts disabled please */
int stfm1000_rds_mix_msg_get(struct stfm1000_rds_state *rds)
{
	struct stfm1000_rds_demod *rdsd = &rds->demod;

	if (!rdsd->mix_msg_pending)
		return -1;

	return rdsd->mix_msg;
}

/* call with interrupts disabled please */
int stfm1000_rds_mix_msg_processed(struct stfm1000_rds_state *rds, int mix_msg)
{
	struct stfm1000_rds_demod *rdsd = &rds->demod;

	if (!rdsd->mix_msg_pending)
		return -1;

	rdsd->mix_msg_pending = 0;

	/* update the completion indication bit */
	if ((mix_msg & 0x8) == 0)
		rdsd->MixPopDone = 1;

	/* this is reflected off the hardware register */
	rdsd->rds_mix_offset = mix_msg & 1;

	if (rdsd->mix_msg != mix_msg) {
		rdsd->mix_msg_processed_changed++;
		return -1;
	}
	return 0;
}

static void rds_sdnominal_msg(struct stfm1000_rds_demod *rdsd, int sdnominal)
{
	if (rdsd->sdnominal_msg_pending)
		rdsd->sdnominal_msg_overrun++;
	rdsd->sdnominal_msg = sdnominal;
	rdsd->sdnominal_msg_pending = 1;

	/* signal monitor thread */
	stfm1000_monitor_signal(
		rds_state_to_stfm1000(demod_to_rds_state(rdsd)),
		EVENT_RDS_SDNOMINAL);
}

/* call with interrupts disabled please */
int stfm1000_rds_sdnominal_msg_get(struct stfm1000_rds_state *rds)
{
	struct stfm1000_rds_demod *rdsd = &rds->demod;

	if (!rdsd->sdnominal_msg_pending)
		return 0;

	return rdsd->sdnominal_msg;
}

/* call with interrupts disabled please */
int stfm1000_rds_sdnominal_msg_processed(struct stfm1000_rds_state *rds,
	int sdnominal_msg)
{
	struct stfm1000_rds_demod *rdsd = &rds->demod;

	if (!rdsd->sdnominal_msg_pending)
		return -1;

	rdsd->sdnominal_msg_pending = 0;
	return 0;
}

void demod_loop(struct stfm1000_rds_bitstream *rdsb,
	struct stfm1000_rds_demod *rdsd)
{
	s32 filter_out;
	u32 freeSpace;
	s32 decomp_hist_pp;
	u8 phase;

	/* Check if we're at a half-basis point */
	if ((rdsd->i & (RDS_BASISLENGTH/2 - 1)) != 0)
		return;	/* Nope, return */

	/* Yes, time to do our work */
	/* Rotate the length 3 history buffer */
	decomp_hist_pp = rdsd->decomp_hist_p;
	rdsd->decomp_hist_p = rdsd->decomp_hist;
	if ((rdsd->i & (RDS_BASISLENGTH-1)) == 0) {
		rdsd->decomp_hist = rdsd->mixandsum1>>9; /* Grab output of
							  * mixandsum1/512 */
		rdsd->mixandsum1 = 0;	 /* Reset mixandsum #1 */
	} else {
		rdsd->decomp_hist = rdsd->mixandsum2>>9; /*Grab output of
							  * mixandsum2/512 */
		rdsd->mixandsum2 = 0;	/* Reset mixandsum #2 */
	}

	/* Form correlator/decimator output by convolving with the
	 * decomposition coefficients, DecompQuant from Matlab work. */
	filter_out = (-58*rdsd->decomp_hist + 59*decomp_hist_pp)>>7;

	/*Figure out which half-basis we are in (out of a bit-length cycle) */
	phase = rdsd->i*2/RDS_BASISLENGTH;
	/*Now what we do depends on the phase variable */
	/*Phase 0:      Bitslice and do timing alignment */
	/*others (1-3): Keep value for timing alignment */

	if (phase == 0) {   /*Main processing (bitslice) */
		u32 Ph;
		u8 OldBit = rdsd->sliced_data;	/* Save the previous value */

		rdsd->return_num = 1;
		if (filter_out >= 0) { /*This bit is "1" */
			/*return value is XOR of previous bit (still in
			 * sliced_data) w/ this */
			/*  bit (1), which equals (NOT of the previous bit) */
			rdsd->return_rdsdemod = !OldBit;
			rdsd->sliced_data = 1;	/*Newest bit value is 1 */
		} else { /*This bit is "0" */
			/*return value is XOR of previous bit (still in
			 * sliced_data) w/ this */
			/*  bit (0), which equals the previous bit */
			rdsd->return_rdsdemod = OldBit;
			rdsd->sliced_data = 0;	/*Newest bit value is 0 */
		}

		freeSpace = bits_free(rdsb);

		if (freeSpace > 0)
			put1bit(rdsb, rdsd->return_rdsdemod);
		else
			rdsd->RdsDemodSkippedBitCnt++;

		/*Increment bits received counter */
		rdsd->BitAlignmentCounter++;
		/*If mixer phase determination hasn't been done, start it */
		if ((rdsd->MixPhaseState == 0) && (!rdsd->MixPhaseDetInProg)) {
			rdsd->MixPhaseDetInProg = 1;
			/*Go to first mixer setting (0) */
			rds_mix_msg(rdsd, 0);
		}

		/* Do bit-slicing time adaption after the mixer phase
		 * determination */
		if (!(rdsd->MixPhaseDetInProg) && !(rdsd->Synchronous)) {

			/* Bitslice Timing Adjust Code (runs after
			 * MixPhaseDetInProg and if RDS is not synchronous to
			 * the FM pilot. */

			u8 BigPh2;	/* Expecting a large value in
					 * PhaseValue[2] */
			u32 MaxRMS = 0;	/*Largest phase RMS */
			s8 MaxPh = 0;	/*Index of largest phase RMS */
			s32 zerocross;

			/* Locate the largest phase RMS
			 * (should be at phase zero) */
			for (Ph = 0; Ph < 4; Ph++)
				if (rdsd->Ph_RMS[Ph] > MaxRMS) {
					MaxRMS = rdsd->Ph_RMS[Ph];
					MaxPh = Ph;
				}

			/* During each bit time we expect the four phases to
			 * take one of the following patterns, where 1
			 * corresponds to maximum modulation:
			 *   1,    0,  -1,    0			Case I
			 *  -1,    0,   1,    0			Case II
			 *   1,  1/2,   0, -1/2			Case III
			 *  -1, -1/2,   0,  1/2			Case IV
			 * We need to distinguish between cases in order to do
			 * the timing adjustment. Below we compare the
			 * correlation of the samples with Case I and Case III
			 * to see which has a bigger abs(correlation).  Thus
			 * BigPh2, if set, means that we decided on Case I or
			 * Case II; if BigPh2 clear, we decided Case III or IV.
			 */
			BigPh2 = abs(rdsd->PhaseValue[0]-rdsd->PhaseValue[2]) >
				 abs(rdsd->PhaseValue[0] +
				 ((rdsd->PhaseValue[1]-
					rdsd->PhaseValue[3])>>1));
			/* If BigPh2, use the difference between phase 1 value
			 * (downgoing for Case I, upgoing for Case II) and
			 * phase 3 value (upgoing for Case I, downgoing for
			 * Case II, thus the subtraction) to indicate timing
			 * error.  If not BigPh2, use the sum of the phase 1
			 * value (downgoing for Case III, upgoing for Case IV)
			 * and phase 3 value (downgoing for Case III, upgoing
			 * for Case IV, thus the addition) to indicate timing
			 * error.  If BigPh2, the slopes at phase 1 & phase 3
			 * are approximately double that if not BigPh2.
			 * Since we are trying to measure timing, scale
			 * by 1/2 in the BigPh2 case. */
			if (BigPh2)
				zerocross = (rdsd->PhaseValue[1]-
					rdsd->PhaseValue[3])>>1;
			else
				zerocross = rdsd->PhaseValue[1]+
					rdsd->PhaseValue[3];
			/* Now if the prev bit was a "1", then the first zero
			 * crossing (phase 1 if BigPh2, phase 2 if !BigPh2)
			 * was a falling one, and if we were late then
			 * zerocross should be negative. If the prev bit was a
			 * "0", then the first zero crossing was a rising one,
			 * and if we were late then zerocross would be
			 * positive. If we are "late" it means that we need to
			 * do a shorter cycle of, say, 15 samples instead of
			 * 16, to "catch up" so that in the future we will be
			 * sampling earlier.  We shorten the cycle by adding
			 * to i, so "late" is going to mean "increment i".
			 * Therefore "late" should be positive, which is done
			 * here by inverting zerocross if the previous bit was
			 * 1.  You could say that this step reflects cases I
			 * and III into II and IV, respectively. */
			if (OldBit)
				zerocross = -zerocross;
			if (!rdsd->DisablePushing) {
				/*The algorithm so far has a stable operating
				 * point 17 phases away from the correct one.
				 * The following code is experimental and may
				 * be deleterious in low SNR conditions, but is
				 * an attempt to move off of the incorrect
				 * operating point. */

				if (MaxPh != 0) {
					/* If it isn't the same MaxPh as the
					 * last non-zero one, clear the counter
					 */
					if (MaxPh != rdsd->PushLastMaxPh) {
						/*Reset the counter */
						rdsd->PushCounter = 0;
						/*Record which phase we're now
						 * counting */
						rdsd->PushLastMaxPh = MaxPh;
					}
					/* If the Max RMS is on the same
					 * non-zero phase, count up */
					rdsd->PushCounter++;
				}
				/* Once every 128 bits, check and then reset
				 * PushCounter */
				if (!(rdsd->BitAlignmentCounter & 0x0FF)) {
					/*If 90% of the time the max phase has
					 * been from the same non-zero phase,
					 * decide that we are latched onto a 0
					 * lock point. Do a large push of the
					 * timing. */
					if (rdsd->PushCounter > 230) {
						s32 pshiph;
						/*Convert from phase number to
						 * the number of filter
						 *  output samples that we need
						 *  to shift */
						if (rdsd->PushLastMaxPh >= 2)
							pshiph =
								4 - (s8)rdsd->
								PushLastMaxPh;
						else
							pshiph =
								-(s8)rdsd->
								PushLastMaxPh;
						/* Scale by the number of i-
						 * phases per output sample */
						pshiph <<=
							RDS_BASISSHIFTS-1;
						/* Perform big pop to get near
						 * correct timing */
						rdsd->i += (RDS_BASISLENGTH<<1)
							+ pshiph;
						/* Set status indicating big
						 * pop was needed.  Reset all
						 * leaky-bucket and summation
						 * variables because the big
						 * timing shift has invalidated
						 * them. Ph_RMS values don't
						 * need to be reset because
						 * they will shift over to
						 * reasonable values again
						 * before their erroneous
						 * values could have effect. */
						rdsd->rds_big_timeshift = 1;
						/*rdsd->Ph_RMS[0] = 0; */
						/*rdsd->Ph_RMS[1] = 0; */
						/*rdsd->Ph_RMS[2] = 0; */
						/*rdsd->Ph_RMS[3] = 0; */
						rdsd->mixandsum1 = 0;
						rdsd->mixandsum2 = 0;
						rdsd->SkipsAccum +=
							pshiph;

						/* Make adjustments in other
						 * values because of the push
						 * (they wouldn't otherwise be
						 * able to use the information
						 * that a push was needed in
						 * their future control
						 * decisions). */
						if (rdsd->PushLastMaxPh != 2) {
							/* If we weren't
							 * pushing from phase
							 * two, accumulate (for
							 * use in adapting
							 * SDNOMINAL) the
							 * phases moved by
							 * pushing. Phase two
							 * pushes are not used;
							 * the push direction
							 * is arbitrary since
							 * Phase 2 is 180
							 * degrees out.  Also,
							 * phase 2 pushes don't
							 * result from
							 * reasonable slippage.
							 * */

							if (rdsd->sdnom_adapt)
								rdsd->SdnomSk
								+= pshiph;

							/* Modify timing_adj to
							 * account for half of
							 * the DC response that
							 * would have occurred
							 * in timing_adj if
							 * that control loop
							 * had seen the push
							 * happen. (Why half?
							 * Because the loop has
							 * already seen a
							 * history of zerocross
							 * values that heads it
							 * in the same
							 * direction as this
							 * adjustment, but may
							 * have seen as few as
							 * half of what it
							 * should have.) */
							rdsd->timing_adj +=
								pshiph <<
								(TADJSH+1);
						}
						/*Set countdown timer that will
						 * prevent any mixer popping
						 * until the Ph_RMS variables
						 * have had enough time to
						 * stabilize */

						/* 2.5 time constants */
						rdsd->PushSafetyZone = 5;
					}
					/*Reset the push counter */
					rdsd->PushCounter = 0;
				}  /*end once every 128 bits */
			}  /*end if !DisablePushing */

			/* Further possible additions:
			 *
			 * 1. Pushes modify timing_adj to decrease convergence
			 *    time.
			 * 2. Separate timing_adj into pilottracking and non-pt
			 *    cases (avoids convergence time after stereo/mono
			 *    transitions)
			 *
			 * Old loop filter was a leaky bucket integrator, and
			 * it always lagged behind if the FM station had RDS
			 * asynchronous to the pilot, because the control loop
			 * needs another integrator to converge on a frequency
			 * error.
			 * New loop filter = 1/(1-1/z) * (a-1/z) * k,
			 * where a = 1+1/256 and k = 1/1024.
			 * You can narrow the loop bandwidth by making "a"
			 * twice as close to 1 and halving k, e.g. a = 1+1/512
			 * and k = 1/2048.
			 * (The way implemented, that narrowing loop BW by
			 * a factor of 2 can be done by incrementing TADJSH.)
			 *
			 * TGR 8/31/2007 */

			/*Integrator, 1/(1-1/z) */
			rdsd->timing_adj += zerocross;
			/*Limit to 1 phase every 8 samples */
			if (rdsd->SkipSafetyZone) {
				rdsd->SkipSafetyZone--;
				rdsd->sampskip = 0;
			} else {
				/*sampskip of non-zero is allowed,
				 * calculate what it really is */

				/*Saturate timing_adj to 2's comp
				 * (2*TADJSH+4)-bit range. */
				if (rdsd->timing_adj > (1<<(2*TADJSH+3))-1)
					rdsd->timing_adj = (1<<(2*TADJSH+3))-1;
				if (rdsd->timing_adj < -(1<<(2*TADJSH+3)))
					rdsd->timing_adj = -(1<<(2*TADJSH+3));

				/* Zero, implemented after the integrator
				 * output.
				 * (a-1/z) = (1+1/256) - 1/z = (1-1/z) + 1/256.
				 * But (1 - 1/z) is timing_adj-
				 * prev_timing_adj = zerocross. */
				rdsd->sampskip = zerocross	/* 1 - 1/z */
					/* 1/256 (with rounding) */
					+ ((rdsd->timing_adj
						+ (1<<(TADJSH-1)))>>TADJSH);
				/*Round and apply k */
				rdsd->sampskip += (1<<(TADJSH+1));
				rdsd->sampskip >>= (TADJSH+2);
				/*Limit to [-1,+1] inclusive */
				if (rdsd->sampskip > 1)
					rdsd->sampskip = 1;
				if (rdsd->sampskip < -1)
					rdsd->sampskip = -1;
				/* If non-zero, start the skip safety zone,
				 * which excludes more sample skipping for a
				 * while.  Note that the safety zone only
				 * applies to the skips -- pushes can still
				 * happen inside a SkipSafetyZone. */
				if (rdsd->sampskip)
					rdsd->SkipSafetyZone = 8-1;
			}
			/**********************************************
			* End Timing Adjust Code
			**********************************************/

			/**********************************************
			* Begin Phase Popper Code
			**********************************************/
			/* If Phase Popping is enabled and 1/2 of a
			 * time constant has gone by... */
			if (rdsd->PhasePoppingEnabled &&
				!(rdsd->BitAlignmentCounter &
					((1<<(RMSALPHASHIFTS-1))-1))) {

				u8 ForcePop = 0; /* Used to force a pop */

				/*Record the maximum of the envelope */
				if (MaxRMS > rdsd->PhasePopMaxRMS)
					rdsd->PhasePopMaxRMS = MaxRMS;
				/* Also track MaxRMS into MixPhase0/1Mag, so
				 * that we can see what the largest RMS on each
				 * of those phases is.  On synchronous stations
				 * (meaning the RDS carrier and bit rate are
				 * synchronized with the pilot), the right mix
				 * phase will always be big and the wrong phase
				 * small. On asynchronous stations (and
				 * stations without RDS), both phases will at
				 * some time or other have about the
				 * same amplitude on each of the phases. */
				if (rdsd->rds_mix_offset) {
					if (MaxRMS > rdsd->MixPhase1Mag)
						rdsd->MixPhase1Mag = MaxRMS;
				} else {
					if (MaxRMS > rdsd->MixPhase0Mag)
						rdsd->MixPhase0Mag = MaxRMS;
				}
				/* Update PopSafetyZone and PushSafetyZone
				 * counters.  With RMSALPHASHIFTS = 5, each
				 * tick is 16/1187.5 =~ 13.5 ms. */
				if (rdsd->PopSafetyZone) {
					rdsd->PopSafetyZone--;
					/* If safety zone just ended and this
					 * mix phase is giving smaller RMS than
					 * before the pop, then the pop was a
					 * mistake.  Go back to previous mixer
					 * phase */
					if (!(rdsd->PopSafetyZone)
						&& (rdsd->PhasePopMaxRMS <
							rdsd->PrePopRMS))
						ForcePop = 1;
				}
				/* If there is no recent push, and Phase 0 has
				 * the maximum RMS, and at least 1/7th of a
				 * second has passed since the last phase pop,
				 * and ((the RMS is less than 1/2 of
				 * PhasePopMaxRMS) or (the RMS is less than
				 * 100)), then try a phase pop. */
				if (/* (rdsd->Ph_RMS[0] == MaxRMS) &&
						* Phase 0 has maximum RMS  */
					!(rdsd->PopSafetyZone))	{
					 /* and Long enough since last
					  * phase pop */

					/* Eligible for a pop, see if one of
					 * the pop conditions is met */
					if ((MaxRMS<<1) <
							rdsd->PhasePopMaxRMS) {
						/*RMS decline from its peak */
						ForcePop = 1;
					} else if ((MaxRMS>>RMSALPHASHIFTS)
						< 50) {
						/*RMS too small to receive,
						 * either there's no RDS or
						 * this is the wrong phase */
						ForcePop = 1;
					}
				}
				if (ForcePop) {

					/*Pop to opposite setting */
					rds_mix_msg(rdsd, 0x8 |
						!rdsd->rds_mix_offset);

					/*Save the pre-pop RMS so that later we
					 * can see if the pop was actually
					 * effective */
					rdsd->PrePopRMS = MaxRMS;
					/*Reset the PhasePopMaxRMS.  We rely on
					 * the PopSafetyZone to give time to
					 * get a new valid max RMS before we're
					 * eligible for the next phase pop.  If
					 * there were no reset we'd be forever
					 * incrementing PhasePopMaxRMS due
					 * to just happenstance large-noise
					 * samples and it might eventually get
					 * some freakish large value causing
					 * frequent erroneous pops. */
					rdsd->PhasePopMaxRMS = 0;
					/* Pop Safety zone length is decided by
					 * how much of an asynchronous
					 * frequency can be supported. Allowing
					* 50 ppm of transmitter error (error
					* between their own pilot, that we
					* should be locked to, and their RDS
					* carrier (which by RDS spec should be
					* locked to their pilot, but we've
					* recently found frequently isn't).
					* 50ppm * 57kHz = 2.85Hz.
					* (2.85 cycles/sec)(4 pops/cycle)
					* = 11.4 pops/second.
					* Safety zone = (1/11.4) seconds =~ 104
					* bits, round down to 96 bits which
					* yields 6 ticks if RMSALPHASHIFTS = 5.
					* */
					rdsd->PopSafetyZone = 96>>
						(RMSALPHASHIFTS-1);
				}
			}
			/******************************************************
			* End Phase Popper Code
			******************************************************/

			/* SDNOMINAL adaption */
			if (rdsd->sdnom_adapt) {
				rdsd->SdnomSk += rdsd->sampskip;
				if (rdsd->pCoefForcedMono &&
					(rdsd->BitAlignmentCounter & 0xFFF) ==
						0x800) {

					rds_sdnominal_msg(rdsd,
						-(rdsd->SdnomSk<<9));

					/*Reset skips counter */
					rdsd->SdnomSk = 0;
				}
			}

			rdsd->SkipsAccum += rdsd->sampskip;
			/* Once per 3.45 seconds, print out signal strength,
			 * skips and pops. Then reset the variables totalling
			 * those occurrences */
			if (!(rdsd->BitAlignmentCounter & 0xFFF)) {
				/* During very noisy input (or if no RDS, or no
				 * station present), timing_adj can go crazy,
				 * since it is the integral of noise.  Although
				 * it is a saturated value (earlier, in the
				 * timing adjust code), the level at which we
				 * can saturate still leaves room for
				 * timing_adj to get too big.  A large value of
				 * timing_adj is a persistent pathology because
				 * the phase is shifting so quickly that the
				 * push detector (which relies on stable
				 * phase-RMS values) never triggers, thus there
				 * is no implemented rescue besides this
				 * clearing that restores proper function. */
				if (abs(rdsd->SkipsAccum) > 300)
					rdsd->timing_adj = 0;
				/*Reset the accumulations. */
				rdsd->SkipsAccum = 0;
			}
		} /*End of bit timing adaption */

		/* If mixer phase determination in progress,
		 * perform actions at certain times */
		if (rdsd->MixPhaseDetInProg) {
			/*~10ms settling time after mixer phase change */
			#define MIXPHASE_STARTMEAS 12
			/*~20ms measurement window */
			#define MIXPHASE_ENDMEAS (MIXPHASE_STARTMEAS+24)
			if (rdsd->BitAlignmentCounter == MIXPHASE_STARTMEAS) {
				/*Reset the RMS variables */
				rdsd->Ph_RMS[0] = 0;
				rdsd->Ph_RMS[1] = 0;
				rdsd->Ph_RMS[2] = 0;
				rdsd->Ph_RMS[3] = 0;
				/* Don't reset mixandsum values because at
				 * least they have filtered continuously.  All
				 * we really need for the mixer phase decision
				 * is a constant measurement window. */
			} else if (rdsd->BitAlignmentCounter ==
					MIXPHASE_ENDMEAS) {
				/*Measurement = mean of RMS values */
				u32 Ndx, MeasVal = 0;
				for (Ndx = 0; Ndx < 4;
					MeasVal += rdsd->Ph_RMS[Ndx++]>>2);
				/*Store measurement in correct place */
				if (rdsd->MixPhaseState == 1) {
					rdsd->MixPhase0Mag = MeasVal;
					/*Go to next mixer setting */
					rds_mix_msg(rdsd, 1);
				} else if (rdsd->MixPhaseState == 2) {
					u8 NextMixSetting;
					rdsd->MixPhase1Mag = MeasVal;
					/* Both measurements done now, see what
					 * mixer setting we need to use.
					 * 0 if MixPhase0Mag > MixPhase1Mag,
					 * 1 otherwise. */
					NextMixSetting = (rdsd->MixPhase0Mag
						<= rdsd->MixPhase1Mag);
					/* If the mixer setting needed is 1,
					 * that is already the current setting.
					 * Terminate mixer phase determination.
					 * Otherwise send message to switch the
					 * mixer phase setting. */
					if (NextMixSetting) {
						rdsd->MixPhaseState = 3;
						rdsd->MixPhaseDetInProg = 0;
					} else
						rds_mix_msg(rdsd, 0);
				}
			}
			/* Reset BitAlignmentCounter if the Mixer just popped
			 * Change state, if required.  States are:
			 * 0: Initial state, send msg causing RDS_MIXOFFSET=>0
			 * 1: Measure with RDS_MIXOFFSET = 0.
			 * 	Lasts just over 30 ms.
			 * 2: Measure with RDS_MIXOFFSET = 1.
			 * 	Lasts just over 30 ms.
			 * 3: At final RDS_MIXOFFSET value.
			 * 	Lasts as long as RDS continues. */
			if (rdsd->MixPopDone) {
				rdsd->MixPopDone = 0;
				rdsd->BitAlignmentCounter = 0;
				rdsd->MixPhaseState++;	/*Go to next state */
				/* If we got to state 3, turn off mixer phase
				 * determination code */
				if (rdsd->MixPhaseState == 3)
					rdsd->MixPhaseDetInProg = 0;
			}
		}

		/* Update status variables */
		rdsd->RDS_BIT_AMP_STAT_REG9 = rdsd->Ph_RMS[0]>>RMSALPHASHIFTS;
		/*Saturate */
		if (rdsd->RDS_BIT_AMP_STAT_REG9 > 511)
			rdsd->RDS_BIT_AMP_STAT_REG9 = 511;
	}  /*End phase 0 code */

	/***************************************************
	* Actions common to all phases
	***************************************************/

	/* Save the output of each phase for possible
	 * calculations during phase 0 */
	rdsd->PhaseValue[phase] = filter_out;

	/*So that we can measure signal amplitude and/or determine what (if */
	/*  any) big jump is needed, maintain the RMS of each phase.  Phase */
	/*  0 RMS is already in Ph_RMS[0] (see bitslicing code, earlier). */
	rdsd->Ph_RMS[phase] += abs(filter_out) -
		(rdsd->Ph_RMS[phase]>>RMSALPHASHIFTS);
}

#if defined(CONFIG_ARM)

/* assembly version for ARM */
#define RDS_MAC(_acc, _x, _y) \
	__asm__ __volatile__ (			 \
	"smlabb	%0, %1, %2, %0\n"		 \
	: "=&r" (_acc)				 \
	: "r" (_x), "r" (_y)			 \
	: "cc")

#else

/* all others, use standard C */
#define RDS_MAC(_acc, _x, _y) 			 \
	do {					 \
		(_acc) += (s16)(_x) * (s16)(_y); \
	} while (0)

#endif

static void rds_demod(const u16 *data, struct stfm1000_rds_demod *rdsd,
	struct stfm1000_rds_bitstream *rbit, int total)
{
	register const s16 *basis0;
	register const s16 *basis1;
	register s16 val;
	register int i;
	register int sampskip;
	register s32 acc1;
	register s32 acc2;

	/* point to the table */
	basis0 = u16_rds_basis;
	basis1 = basis0 + 8;

	rdsd->return_num = 0;

	/* restore state */
	i = rdsd->i;
	acc1 = rdsd->mixandsum1;
	acc2 = rdsd->mixandsum2;	/* 64 bit */
	sampskip = rdsd->sampskip;

	while (total-- > 0) {

		val = data[3];	/* load RDS data */
		data += 4;
		if (val == 0x7fff)	/* illegal RDS sample */
			continue;

		RDS_MAC(acc1, val, basis0[i]);
		RDS_MAC(acc2, val, basis1[i]);

		if (i == 4) {
			i += sampskip;
			sampskip = 0;
		}

		if ((i & (RDS_BASISLENGTH / 2 - 1)) == 0) {

			/* save state */
			rdsd->mixandsum1 = acc1;
			rdsd->mixandsum2 = acc2;
			rdsd->i = i;
			rdsd->sampskip = sampskip;

			demod_loop(rbit, rdsd);

			/* restore state */
			acc1 = rdsd->mixandsum1;
			acc2 = rdsd->mixandsum2;
			i = rdsd->i;
			sampskip = rdsd->sampskip;
		}
		i = (i + 1) & 31;
	}

	/* save state */
	rdsd->mixandsum1 = acc1;
	rdsd->mixandsum2 = acc2;
	rdsd->i = i;
	rdsd->sampskip = sampskip;
}

void stfm1000_rds_demod(struct stfm1000_rds_state *rds, const u16 *dri_data,
	int total)
{
	rds_demod(dri_data, &rds->demod, &rds->bitstream, total);

	/* signal only when we have enough */
	if (bits_filled(&rds->bitstream) > 128)
		stfm1000_monitor_signal(rds_state_to_stfm1000(rds),
				EVENT_RDS_BITS);
}

static void bitstream_reset(struct stfm1000_rds_bitstream *rdsb)
{
	memset(rdsb, 0, sizeof(*rdsb));
}

static void demod_reset(struct stfm1000_rds_demod *rdsd)
{
	memset(rdsd, 0, sizeof(*rdsd));
	rdsd->sdnom_adapt = 0;	/* XXX this doesn't really work right */
					/* it causes underruns at ALSA */
	rdsd->PhasePoppingEnabled = 1;	/* does this? */
}

static void packet_reset(struct stfm1000_rds_pkt *rdsp)
{
	memset(rdsp, 0, sizeof(*rdsp));
	rdsp->state = SYNC_OFFSET_A;
}

static void text_reset(struct stfm1000_rds_text *rdst)
{
	memset(rdst, 0, sizeof(*rdst));
}

void stfm1000_rds_reset(struct stfm1000_rds_state *rds)
{
	bitstream_reset(&rds->bitstream);
	demod_reset(&rds->demod);
	packet_reset(&rds->pkt);
	text_reset(&rds->text);
	rds->reset_req = 0;
}

int stfm1000_rds_bits_available(struct stfm1000_rds_state *rds)
{
	return bits_filled(&rds->bitstream);
}

int stmf1000_rds_get_bit(struct stfm1000_rds_state *rds)
{
	if (bits_filled(&rds->bitstream) == 0)
		return -1;
	return get1bit(&rds->bitstream);
}

int stmf1000_rds_avail_bits(struct stfm1000_rds_state *rds)
{
	return bits_filled(&rds->bitstream);
}

static const u32 rds_ParityCheck[] = {
	0x31B, 0x38F, 0x2A7, 0x0F7, 0x1EE,
	0x3DC, 0x201, 0x1BB, 0x376, 0x355,
	0x313, 0x39F, 0x287, 0x0B7, 0x16E,
	0x2DC, 0x001, 0x002, 0x004, 0x008,
	0x010, 0x020, 0x040, 0x080, 0x100,
	0x200
};

static int calc_syndrome(u32 rdscrc)
{
	int i;
	u32 syndrome = 0;
	int word = 0x1;

	for (i = 0; i < 26; i++) {
		if (rdscrc & word)
			syndrome ^= rds_ParityCheck[i];
		word <<= 1;
	}
	return syndrome;
}

static u32 ecc_table[1024];
static int ecc_table_generated;

static void generate_ecc_table(void)
{
	int i, j, size;
	u32 syndrome, word;

	for (i = 0; i < ECC_TBL_SIZE; i++)
		ecc_table[i] = 0xFFFFFFFF;
	ecc_table[0] = 0x0;

	for (j = 0; j < 5; j++) {
		word = (1 << (j + 1)) - 1;	/* 0x01 0x03 0x07 0x0f 0x1f */
		size = 26 - j; 			/* 26, 25, 24, 23, 22 */
		syndrome = 0;
		for (i = 0; i < size; i++) {
			syndrome = calc_syndrome(word);
			ecc_table[syndrome] = word;
			word <<= 1;
		}
	}
}

static u32 ecc_correct(u32 rdsBits, int *recovered)
{
	u32 syndrome;
	u32 errorBits;

	if (recovered)
		*recovered = 0;

	/* Calculate Syndrome on Received Packet */
	syndrome = calc_syndrome(rdsBits);

	if (syndrome == 0)
		return rdsBits; /* block is clean */

	/* generate table first time we get here */
	if (!ecc_table_generated) {
		generate_ecc_table();
		ecc_table_generated = 1;
	}

	/* Attempt to recover block */
	errorBits = ecc_table[syndrome];
	if (errorBits == UNRECOVERABLE_RDS_BLOCK)
		return UNRECOVERABLE_RDS_BLOCK;	/* Block can not be recovered.
						 * it is bad packet */

	rdsBits = rdsBits ^ errorBits;
	if (recovered)
		(*recovered)++;
	return rdsBits;                 /* ECC correct */
}

/* The following table lists the RDS and RBDS Program Type codes
 * and their meanings:
 * PTY code RDS Program type RBDS Program type */
static const struct stfm1000_rds_pty stc_tss_pty_tab[] = {
	{  0, "No program type",    "No program type"},
	{  1, "News",               "News"},
	{  2, "Current affairs",    "Information"},
	{  3, "Information",        "Sports"},
	{  4, "Sports",             "Talk"},
	{  5, "Education",          "Rock"},
	{  6, "Drama",              "Classic Rock"},
	{  7, "Culture",            "Adult Hits"},
	{  8, "Science",            "Soft Rock"},
	{  9, "Varied",             "Top 40"},
	{ 10, "Pop",                "Music Country"},
	{ 11, "Rock",               "Music Oldies"},
	{ 12, "M.O.R.",             "Music Soft"},
	{ 13, "Light classical",    "Nostalgia"},
	{ 14, "Serious",            "Classical Jazz"},
	{ 15, "Other Music",        "Classical"},
	{ 16, "Weather",            "Rhythm and Blues"},
	{ 17, "Finance",            "Soft Rhythm and Blues"},
	{ 18, "Children's programs", "Language"},
	{ 19, "Social Affairs",     "Religious Music"},
	{ 20, "Religion",           "Religious Talk"},
	{ 21, "Phone In",           "Personality"},
	{ 22, "Travel",             "Public"},
	{ 23, "Leisure",            "College"},
	{ 24, "Jazz Music",         "Unassigned"},
	{ 25, "Country Music",      "Unassigned"},
	{ 26, "National Music",     "Unassigned"},
	{ 27, "Oldies Music",       "Unassigned"},
	{ 28, "Folk Music",         "Unassigned"},
	{ 29, "Documentary",        "Weather"},
	{ 30, "Alarm Test",         "Emergency Test"},
	{ 31, "Alarm",              "Emergency"},
};

#if 0
static const char *rds_group_txt[] = {
	[RDS_GROUP_TYPE_0A] = "Basic tuning and switching information (0A)",
	[RDS_GROUP_TYPE_0B] = "Basic tuning and switching information (0B)",
	[RDS_GROUP_TYPE_1A] = "Program item number and slow labeling codes",
	[RDS_GROUP_TYPE_1B] = "Program item number",
	[RDS_GROUP_TYPE_2A] = "Radio Text (2A)",
	[RDS_GROUP_TYPE_2B] = "Radio Text (2B)",
	[RDS_GROUP_TYPE_3A] = "Application identification for ODA only",
	[RDS_GROUP_TYPE_3B] = "Open data applications",
	[RDS_GROUP_TYPE_4A] = "Clock-time and date",
	[RDS_GROUP_TYPE_4B] = "Open data applications",
	[RDS_GROUP_TYPE_5A] = "Transparent Data Channels (32 ch.) or ODA (5A)",
	[RDS_GROUP_TYPE_5B] = "Transparent Data Channels (32 ch.) or ODA (5B)",
	[RDS_GROUP_TYPE_6A] = "In House Applications or ODA (6A)",
	[RDS_GROUP_TYPE_6B] = "In House Applications or ODA (6B)",
	[RDS_GROUP_TYPE_7A] = "Radio Paging or ODA",
	[RDS_GROUP_TYPE_7B] = "Open Data Applications",
	[RDS_GROUP_TYPE_8A] = "Traffic Message Channel or ODA",
	[RDS_GROUP_TYPE_8B] = "Open Data Applications",
	[RDS_GROUP_TYPE_9A] = "Emergency warning system or ODA",
	[RDS_GROUP_TYPE_9B] = "Open Data Applications",
	[RDS_GROUP_TYPE_10A] = "Program Type Name",
	[RDS_GROUP_TYPE_10B] = "Open Data Applications (10B)",
	[RDS_GROUP_TYPE_11A] = "Open Data Applications (11A)",
	[RDS_GROUP_TYPE_11B] = "Open Data Applications (11B)",
	[RDS_GROUP_TYPE_12A] = "Open Data Applications (12A)",
	[RDS_GROUP_TYPE_12B] = "Open Data Applications (12B)",
	[RDS_GROUP_TYPE_13A] = "Enhanced Radio Paging or ODA",
	[RDS_GROUP_TYPE_13B] = "Open Data Applications",
	[RDS_GROUP_TYPE_14A] = "Enhanced Other Networks information (14A)",
	[RDS_GROUP_TYPE_14B] = "Enhanced Other Networks information (14B)",
	[RDS_GROUP_TYPE_15A] = "Defined in RBDS",
	[RDS_GROUP_TYPE_15B] = "Fast switching information",
};
#endif

static void dump_rds_packet(u8 *buf)
{
	u16 pi, offb;

	pi   = (u16)(buf[0] << 8) | buf[1];
	offb = (u16)(buf[1] << 8) | buf[2];

	printk(KERN_INFO "GRP: "
		"PI=0x%04x "
		"GT=%2d VER=%d TP=%d PTY=%2d "
		"PS_SEG=%2d RT_AB=%2d RT_SEG=%2d\n", pi,
		RDS_GROUP_TYPE(offb), RDS_VERSION(offb), RDS_TP(offb),
		RDS_PTY(offb),
		RDS_PS_SEG(offb), RDS_RT_AB(offb), RDS_RT_SEG(offb));
}

void stfm1000_rds_process_packet(struct stfm1000_rds_state *rds, u8 *buffer)
{
	struct stfm1000_rds_text *rdst = &rds->text;
	/* char tempCallLetters[5] = {0}; */
	struct rds_group_data grp;
	int grpno;
	u32 offset;
	char tps[9];
	int i, seg, idx;

	grp.piCode  = ((u16)buffer[0] << 8) | buffer[1];
	grp.offsetB = ((u16)buffer[2] << 8) | buffer[3];
	grp.offsetC = ((u16)buffer[4] << 8) | buffer[5];
	grp.offsetD = ((u16)buffer[6] << 8) | buffer[7];

	grpno = (grp.offsetB >> (8 + 3)) & 0x1f;

	if (rds_state_to_stfm1000(rds)->rds_info)
		dump_rds_packet(buffer);

	/* Is this the first time through? */
	if (!rdst->bRds_detected) {
		rdst->pi            = grp.piCode;
		rdst->tp            = RDS_TP(grp.offsetB);
		rdst->version       = RDS_VERSION(grp.offsetB);
		rdst->pty.id        = RDS_PTY(grp.offsetB);
		rdst->pty.pRds      = stc_tss_pty_tab[rdst->pty.id].pRds;
		rdst->pty.pRdbs     = stc_tss_pty_tab[rdst->pty.id].pRdbs;
		rdst->bRds_detected = 1;
	}

	/* Have we process too many PI errors? */
	if (grp.piCode != rdst->pi) {
		if (rdst->mismatch++ > 10) {

			/* requested reset of RDS */
			rds->reset_req = 1;

			/* signal monitor thread */
			stfm1000_monitor_signal(rds_state_to_stfm1000(rds),
				EVENT_RDS_RESET);

			if (rds_state_to_stfm1000(rds)->rds_info)
				printk(KERN_INFO "RDS: RESET!!!\n");

			text_reset(rdst);
		}
		rdst->consecutiveGood = 0;
		return;
	}

	if (rdst->consecutiveGood++ > 10)
		rdst->mismatch = 0;            /* reset bad count */

	if (rdst->consecutiveGood > rdst->consecutiveGoodMax)
		rdst->consecutiveGoodMax = rdst->consecutiveGood;

	switch (grpno) {
	case RDS_GROUP_TYPE_0A:
	case RDS_GROUP_TYPE_0B:
		/* Extract Service Name information */
		offset = RDS_PS_SEG(grp.offsetB) * 2;
		rdst->wk_ps[offset]     = buffer[6];	/* better */
		rdst->wk_ps[offset + 1] = buffer[7];
		rdst->wk_ps_mask |= 1 << RDS_PS_SEG(grp.offsetB);

		if (rds_state_to_stfm1000(rds)->rds_info) {
			for (i = 0; i < 8; i++) {
				if (rdst->wk_ps_mask & (1 << i)) {
					tps[i * 2] =
						rdst->wk_ps[i * 2];
					tps[i * 2 + 1] =
						rdst->wk_ps[i * 2 + 1];
				} else {
					tps[i * 2] = '_';
					tps[i * 2 + 1] = '_';
				}
			}
			tps[ARRAY_SIZE(tps) - 1] = '\0';
			if (rds_state_to_stfm1000(rds)->rds_info)
				printk(KERN_INFO "RDS-PS (curr): %s\n", tps);
		}

		if (rdst->wk_ps_mask != ALL_SEGMENT_BITS)
			break;

		if (rdst->ps_valid) {
			if (memcmp(rdst->ps, rdst->wk_ps, 8) != 0) {
				memset(rdst->cp_ps, 0, 8);
				memset(rdst->wk_ps, 0, 8);
				rdst->wk_ps_mask = 0;
			}

			memset(rdst->ps, 0, 8);
			rdst->ps_valid = 0;
			break;
		}

		/* does working buffer == compare buffer */
		if (memcmp(rdst->cp_ps, rdst->wk_ps, 8) != 0) {
			/* just copy from working to compare buffer */
			memcpy(rdst->cp_ps, rdst->wk_ps, 8);
			rdst->wk_ps_mask = 0;
			break;
		}

		/* working buffer matches compare buffer, send to UI */
		memcpy(rdst->ps, rdst->cp_ps, 8);
		rdst->ps_valid = 1;

		if (rds_state_to_stfm1000(rds)->rds_info)
			printk(KERN_INFO "RDS: PS '%s'\n", rdst->ps);

		/* clear working mask-only */
		rdst->wk_ps_mask = 0;
		break;

	case RDS_GROUP_TYPE_2A:

		/* Clear buffer */
		if (rdst->textAB_flag != RDS_RT_AB(grp.offsetB)) {
			memset(rdst->wk_text, 0, 64);
			rdst->wk_text_mask = 0;
			rdst->textAB_flag  = RDS_RT_AB(grp.offsetB);
		}

		/* Extract Text */
		seg = RDS_RT_SEG(grp.offsetB);
		idx = seg * 4;

		#define CNVT_EOT(x) ((x) != RDS_EOT ? (x) : 0)
		rdst->wk_text[idx++] = CNVT_EOT(buffer[4]);
		rdst->wk_text[idx++] = CNVT_EOT(buffer[5]);
		rdst->wk_text[idx++] = CNVT_EOT(buffer[6]);
		rdst->wk_text[idx++] = CNVT_EOT(buffer[7]);

		rdst->wk_text_mask |= 1 << seg;
		/* scan msg data for EOT.  If found set all higher
		 * mask bits */
		for (idx = 0; idx < 4; idx++) {
			if (rdst->text[idx] == RDS_EOT)
				break;
		}
		if (idx < 4) {
			/* set current and all higher bits  */
			for (idx = RDS_RT_SEG(grp.offsetB); idx < 16;
				idx++)
				rdst->wk_text_mask |= 1 << idx;
		}

		/* Process buffer when filled */
		if (rdst->wk_text_mask != ALL_TEXT_BITS)
			break;

		if (!rdst->text_valid)
			rdst->text_valid = 1;
		else if (memcmp(rdst->text, rdst->wk_text, 64) == 0)
			break;

		memcpy(rdst->text, rdst->wk_text, 64);

		if (rds_state_to_stfm1000(rds)->rds_info)
			printk(KERN_INFO "RDS: TEXT '%s'\n", rdst->text);

		memset(rdst->wk_text, 0, 64);
		rdst->wk_text_mask = 0;
		break;

	default:
		break;
	}
}

int stfm1000_rds_packet_dequeue(struct stfm1000_rds_state *rds, u8 *buf)
{
	struct stfm1000_rds_pkt *pkt = &rds->pkt;

	if (pkt->buf_cnt == 0)
		return -1;

	memcpy(buf, &pkt->buf_queue[pkt->buf_tail][0], 8);
	if (++pkt->buf_tail >= RDS_PKT_QUEUE)
		pkt->buf_tail = 0;
	pkt->buf_cnt--;

	return 0;
}

void stfm1000_rds_packet_bit(struct stfm1000_rds_state *rds, int bit)
{
	struct stfm1000_rds_pkt *pkt = &rds->pkt;
	u32 rdsdata, rdscrc, rdscrc_c, rdscrc_cp;
	int correct, correct2, recovered, recovered2;
	int RetVal;

	/* Stick into shift register */
	pkt->rdsstream = ((pkt->rdsstream << 1) | bit) & 0x03ffffff;
	pkt->bitsinfifo++;
	pkt->bitcount++;

	/* wait for 26 bits of block */
	if (pkt->bitsinfifo < 26)
		return;

	rdsdata = pkt->rdsstream & 0x03fffc00;	/* 16 bits of Info. word */
	rdscrc = pkt->rdsstream & 0x3ff;	/* 10 bits of Checkword */

	switch (pkt->state) {
	case SYNC_OFFSET_A:

		RetVal = calc_syndrome(pkt->rdsstream);

		switch (RetVal) {
		case RDS_SYNDROME_OFFSETA:
			pkt->state = OFFSET_B;
			break;
		case RDS_SYNDROME_OFFSETB:
			pkt->state = OFFSET_C_CP;
			break;
		case RDS_SYNDROME_OFFSETC:
			pkt->state = OFFSET_D;
			break;
		case RDS_SYNDROME_OFFSETCP:
			pkt->state = OFFSET_D;
			break;
		case RDS_SYNDROME_OFFSETD:
			pkt->state = OFFSET_A;
			break;
		default:
			pkt->state = SYNC_OFFSET_A;
			break;
		}
		if (pkt->state == SYNC_OFFSET_A) {
			pkt->sync_lost_packets++;
			/* XXX send info? */
			break;
		}

		pkt->good_packets++;

		rdsdata = pkt->rdsstream & 0x03fffc00;

		/* Save type A packet in buffer */
		rdsdata >>= 10;
		pkt->buffer[0] = (rdsdata >> 8);
		pkt->buffer[1] = (rdsdata & 0xff);
		pkt->bitsinfifo = 0;

		/* We found a block with zero errors, but it is not at the
		 * start of the group. */
		if (pkt->state == OFFSET_B)
			pkt->discardpacket = 0;
		else
			pkt->discardpacket = 1;
		break;

	case OFFSET_A:		/* Type A: we are in sync now */
		rdscrc ^= RDS_OFFSETA;
		correct = ecc_correct(rdsdata | rdscrc, &recovered);
		if (correct == UNRECOVERABLE_RDS_BLOCK) {
			pkt->bad_packets++;
			pkt->discardpacket++;
			pkt->state++;
			pkt->bitsinfifo = 0;
			break;
		}

		if (recovered)
			pkt->recovered_packets++;
		pkt->good_packets++;

		/* Attempt to see, if we can get the entire group.
		 * Don't discard. */
		pkt->discardpacket = 0;
		rdsdata = correct & 0x03fffc00;

		/* Save type A packet in buffer */
		rdsdata >>= 10;
		pkt->buffer[0] = (rdsdata >> 8);
		pkt->buffer[1] = (rdsdata & 0xff);
		pkt->bitsinfifo = 0;
		pkt->state++;
		break;

	case OFFSET_B:		/* Waiting for type B */
		rdscrc ^= RDS_OFFSETB;
		correct = ecc_correct(rdsdata | rdscrc, &recovered);
		if (correct == UNRECOVERABLE_RDS_BLOCK) {
			pkt->bad_packets++;
			pkt->discardpacket++;
			pkt->state++;
			pkt->bitsinfifo = 0;
			break;
		}
		if (recovered)
			pkt->recovered_packets++;
		pkt->good_packets++;

		rdsdata = correct & 0x03fffc00;

		/* Save type B packet in buffer */
		rdsdata >>= 10;
		pkt->buffer[2] = (rdsdata >> 8);
		pkt->buffer[3] = (rdsdata & 0xff);
		pkt->bitsinfifo = 0;
		pkt->state++;
		break;

	case OFFSET_C_CP:	/* Waiting for type C or C' */
		rdscrc_c = rdscrc ^ RDS_OFFSETC;
		rdscrc_cp = rdscrc ^ RDS_OFFSETCP;
		correct = ecc_correct(rdsdata | rdscrc_c, &recovered);
		correct2 = ecc_correct(rdsdata | rdscrc_cp, &recovered2);
		if (correct == UNRECOVERABLE_RDS_BLOCK
		    && correct2 == UNRECOVERABLE_RDS_BLOCK) {
			pkt->bad_packets++;
			pkt->discardpacket++;
			pkt->state++;
			pkt->bitsinfifo = 0;
			break;
		}

		if (recovered || recovered2)
			pkt->recovered_packets++;
		pkt->good_packets++;

		if (correct == UNRECOVERABLE_RDS_BLOCK)
			correct = correct2;

		rdsdata = correct & 0x03fffc00;

		/* Save type C packet in buffer */
		rdsdata >>= 10;
		pkt->buffer[4] = (rdsdata >> 8);
		pkt->buffer[5] = (rdsdata & 0xff);
		pkt->bitsinfifo = 0;
		pkt->state++;
		break;

	case OFFSET_D:		/* Waiting for type D */
		rdscrc ^= RDS_OFFSETD;
		correct = ecc_correct(rdsdata | rdscrc, &recovered);
		if (correct == UNRECOVERABLE_RDS_BLOCK) {
			pkt->bad_packets++;
			pkt->discardpacket++;
			pkt->state = OFFSET_A;
			pkt->bitsinfifo = 0;
			break;
		}

		if (recovered)
			pkt->recovered_packets++;
		pkt->good_packets++;

		rdsdata = correct & 0x03fffc00;

		/* Save type D packet in buffer */
		rdsdata >>= 10;
		pkt->buffer[6] = (rdsdata >> 8);
		pkt->buffer[7] = (rdsdata & 0xff);

		/* buffer it if all segments were ok */
		if (pkt->discardpacket) {
			/* We're still in sync, so back to state 1 */
			pkt->state = OFFSET_A;
			pkt->bitsinfifo = 0;
			pkt->discardpacket = 0;
			break;
		}

		pkt->state++;
		/* fall-through */

	case PACKET_OUT:
		pkt->GroupDropOnce = 1;

		/* queue packet */
		if (pkt->buf_cnt < RDS_PKT_QUEUE) {
			memcpy(&pkt->buf_queue[pkt->buf_head][0],
				pkt->buffer, 8);
			if (++pkt->buf_head >= RDS_PKT_QUEUE)
				pkt->buf_head = 0;
			pkt->buf_cnt++;
		} else
			pkt->buf_overruns++;

		/* We're still in sync, so back to state 1 */
		pkt->state = OFFSET_A;
		pkt->bitsinfifo = 0;
		pkt->discardpacket = 0;
		break;

	}

	/* Lots of errors? If so, go back to resync mode */
	if (pkt->discardpacket >= 10) {
		pkt->state = SYNC_OFFSET_A;	/* reset sync state */
		pkt->bitsinfifo = 26;	/* resync a bit faster */
	}
}

/* GROUP_TYPE 0A-0B (buffer must have enough space for 9 bytes) */
int stfm1000_rds_get_ps(struct stfm1000_rds_state *rds, u8 *buffer,
	int bufsize)
{
	struct stfm1000_rds_text *rdst = &rds->text;

	if (bufsize < 9)
		return -1;

	if (!rdst->ps_valid)
		return -1;

	memcpy(buffer, rdst->ps, 8);
	buffer[8] = '\0';

	return 8;
}

/* GROUP_TYPE 2A (buffer must have enough space for 65 bytes) */
int stfm1000_rds_get_text(struct stfm1000_rds_state *rds, u8 *buffer,
	int bufsize)
{
	struct stfm1000_rds_text *rdst = &rds->text;

	if (bufsize < 9)
		return -1;

	if (!rdst->text_valid)
		return -1;

	memcpy(buffer, rdst->text, 64);
	buffer[64] = '\0';

	return 64;
}
