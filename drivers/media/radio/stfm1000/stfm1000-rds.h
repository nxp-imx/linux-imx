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
#ifndef STFM1000_RDS_H
#define STFM1000_RDS_H

#include <linux/types.h>

/* log2(number of samples in a filter basis) */
#define RDS_BASISSHIFTS		4

/* number of samples in a filter basis */
#define RDS_BASISLENGTH		(1 << RDS_BASISSHIFTS)

#define TIME_ADAPT_OVER		100

/* 2^(-this) is the RMS leaky bucket time constant */
#define RMSALPHASHIFTS		5

#define PROCESS_RDS_BITS	128

#define RDS_BITBUFSIZE		1024	/* was 128 */
struct stfm1000_rds_bitstream {
	u32 buf[RDS_BITBUFSIZE];	/* bit buffer */
	int HeadBitCount;		/* bit buffer head counter */
	int TailBitCount;		/* bit buffer tail counter */
};

struct stfm1000_rds_demod {
	u32 mixandsum1;			/* Accumulator for first
					 * basis filter */
	u32 mixandsum2;			/* Accumulator for 2nd
					 * basis filter */
	u32 i;				/* Phase Index, 32 phases per
					 * RDS bit */
	u32 return_num;			/* Set if there is a new RDS bit */
	u32 BitAlignmentCounter;	/* Counts bits for timing purposes */
	int sampskip;			/* Requested timing shift (on i) */

	int DisablePushing;		/* Disables phase push algorithm
					 * (phase push happens when Ph_RMS[x],
					 * x != 0, is consistently the maximum
					 * Ph_RMS) */
	int MixPopDone;			/* Last mixer phase set request is
					 * done */
	u8 rds_big_timeshift;		/* If set, indicates a push or large
					 * timing shift occurred */
	int return_rdsdemod;		/* Output, (most recent bit) XOR
					 * (prev bit) */
	u32 RDS_BIT_AMP_STAT_REG9;	/* Size of bit (RMS of RDS signal at
					 * bitslicing instant, typically 220
					 * to 270) */
	s32 decomp_hist;		/* Most recent basis filter output */
	s32 decomp_hist_p;		/* Previous basis filter output */
	s32 PhaseValue[4];		/* Half-basis phase samples over the
					 * most recent bit */
	u32 Ph_RMS[4];			/* RMS of the four half-basis phases */
	s32 timing_adj;			/* Timing loop leaky-bucket
					 * accumulator */
	u32 MixPhase0Mag;		/* Magnitude of RDS signal with RDS
					 * mixer phase 0 (from mixer phase
					 * determination) */
	u32 MixPhase1Mag;		/* Magnitude of RDS signal with RDS
					 * mixer phase 1 (from mixer phase
					 * determination) */
	u32 PhasePopMaxRMS;		/* Maximum RMS observed since last
					 * phase pop */
	u32 PrePopRMS;			/* Max of Ph_RMS array right before the
					 * most recent phase pop */
	u8 MixPhaseState;		/* State of RDS mixer phase
					 * determination state machine */
	int MixPhaseDetInProg;		/* Set if RDS mix phase determination
					 * is in progress */
	int sliced_data;		/* The most recent bit decision */
	u8 PopSafetyZone;		/* Countdown timer, holds off next
					 * phase pop after a recent one */
	u8 PushSafetyZone;		/* Countdown timer, holds off next
					 * phase pop after a timing push (b/c
					 * timing push resets Ph_RMS vars) */
	u8 SkipSafetyZone;		/* Countdown timer, holds off next
					 * phase skip (small timing adj) */
	int Synchronous;		/* RDS has been determined to be
					 * synchronous to pilot */
	u8 PushLastMaxPh;		/* The index at which Ph_RMS is
					 * maximum ("x" in the above two
					 * comments) */
	s32 PushCounter;		/* Counts instances of Ph_RMS[x], x!=0,
					 * being the maximum Ph_RMS */
	s32 SkipsAccum;			/* Accumulation of all timing skips
					 * since RDS demod started */
	s32 SdnomSk;			/* Skips counter used for SDNOMINAL
					 * adaption */

	/* update this everytime it's changed & put it here */
	unsigned int rds_mix_offset : 1;

	unsigned int sdnom_adapt : 1;
	unsigned int pCoefForcedMono : 1;	/* copy of filter parameter */
	unsigned int PhasePoppingEnabled : 1;

	unsigned int mix_msg_pending : 1;
	u8 mix_msg;
	unsigned int mix_msg_overrun;
	unsigned int mix_msg_processed_changed;

	unsigned int sdnominal_msg_pending : 1;
	int sdnominal_msg;
	unsigned int sdnominal_msg_overrun;

	u32 RdsDemodSkippedBitCnt;	/* bit skipped by RDS demodulator due
					 * to unavailable space in buf[]
					 * (bit buffer) */
};

#define RDS_OFFSETA	0x0fc
#define RDS_OFFSETB	0x198
#define RDS_OFFSETC	0x168
#define RDS_OFFSETCP	0x350
#define RDS_OFFSETD	0x1b4

#define RDS_SYNDROME_OFFSETA    0x3d8
#define RDS_SYNDROME_OFFSETB    0x3d4
#define RDS_SYNDROME_OFFSETC    0x25c
#define RDS_SYNDROME_OFFSETCP   0x3cc
#define RDS_SYNDROME_OFFSETD    0x258

#define SYNC_OFFSET_A 0 /* default state */
#define OFFSET_A      1
#define OFFSET_B      2
#define OFFSET_C_CP   3
#define OFFSET_D      4
#define PACKET_OUT    5

#define ECC_TBL_SIZE   		    1024
#define UNRECOVERABLE_RDS_BLOCK 0xffffffff

#define RDS_PKT_QUEUE	16

struct stfm1000_rds_pkt {
	int state;		/* Current state */
	u32 rdsstream;		/* Current RDS data */
	u8 buffer[8];		/* temporary storage of RDS data */
	int discardpacket;	/* discard packet count */
	int sync_lost_packets;	/* sync lost */
	int good_packets;	/* good packet */
	int bad_packets;	/* bad packet */
	int recovered_packets;	/* recovered packet */
	int bitsinfifo;		/* bits count */
	int GroupDropOnce;	/* Send Group Drop Message once */
	int bitcount;		/* Counter for Number of Bits read */

	/* queue the packets here */
	int buf_overruns;
	int buf_head;
	int buf_tail;
	int buf_cnt;
	int buf_queue[RDS_PKT_QUEUE][8];
};

#define AUDIT            0
#define ALL_SEGMENT_BITS 0xF
#define ALL_TEXT_BITS    0xFFFF

struct stfm1000_rds_pty {
	u8  id;         /* Program Type ID */
	u8 *pRds;       /* RDS description */
	u8 *pRdbs;      /* RDBS description */
};

struct stfm1000_rds_text {
	u8 bRds_detected;		/* Has the first packet come in yet? */
	u16 pi;				/* Program Identification Code (PI) */
	struct stfm1000_rds_pty pty;	/* Program Type (PTY)) */
	u8 tp;				/* Traffic Program (TP) identification
					 * code */
	u8 ps[9];			/* Program Service Name Sent to UI */
	u8 altFreq[2];			/* Alternate frequency (AF) */
	u8 callLetters[5];		/* For US, stations call letters */

	u8 text[65];			/* Radio Text A */

	unsigned int version : 1;	/* Is station broadcasting version
					 * A or B (B0) */
	unsigned int ps_valid : 1;	/* station name is valid */
	unsigned int text_valid : 1;	/* Text is valid */
	unsigned int textAB_flag : 1;	/* Current flag setting, reset if flag
					 * changes */

	/*------------------Working area--------------------------- */
	u8 cp_ps[8];			/* Compare buffer for PS */
	u8 wk_ps[8];			/* Program Service buffer */
	u8 wk_ps_mask;			/* lower 4 bits must be set
					 * before copy */
	u8 wk_text[64];			/* Radio Text buffer */
	u16 wk_text_mask;		/* all bits must be set before copy */

	/*-------------------Counters------------------------------ */
	u32 messages;			/* total number of messages recieved */
	u32 unsupported;		/* call to unsupported group type */
	u32 mismatch;			/* Mismatched values */
	u32 consecutiveGood;		/* Consecutive good will clear bad  */
	u32 consecutiveGoodMax;		/* Max counter for paramaters */
};

/* Maximum number of RDS groups described in the U.S. RBDS Standard. */
#define MAX_RDS_GROUPS_SUPPORTED 32

/* Common Constants */
#define RDS_LINE_FEED   0xA
#define RDS_EOT         0xD

/* Offsets into OFFSETB  */
#define RDS_GROUP_TYPE(x)         (((x) >> 12) & 0xF)
#define RDS_VERSION(x)            (((x) >> 11) & 0x1)
#define RDS_TP(x)                 (((x) >> 10) & 0x1)
#define RDS_PTY(x)                (((x) >>  5) & 0x1F)
#define RDS_PS_SEG(x)              ((x) & 0x3)
#define RDS_RT_AB(x)              (((x) >>  4) & 0x1)
#define RDS_RT_SEG(x)              ((x) & 0xF)

/* This values corresond to the Group Types defined */
/* In the U.S. RBDS standard. */
#define RDS_GROUP_TYPE_0A	 0 /* Basic tuning and switching information */
#define RDS_GROUP_TYPE_0B	 1 /* Basic tuning and switching information */
#define RDS_GROUP_TYPE_1A	 2 /* Program item number and slow labeling
				    * codes */
#define RDS_GROUP_TYPE_1B	 3 /* Program item number */
#define RDS_GROUP_TYPE_2A	 4 /* Radio Text */
#define RDS_GROUP_TYPE_2B	 5 /* Radio Text */
#define RDS_GROUP_TYPE_3A	 6 /* Application identification for ODA
				    * only */
#define RDS_GROUP_TYPE_3B	 7 /* Open data applications */
#define RDS_GROUP_TYPE_4A	 8 /* Clock-time and date */
#define RDS_GROUP_TYPE_4B	 9 /* Open data applications */
#define RDS_GROUP_TYPE_5A	10 /* Transparent Data Channels (32 channels)
				    * or ODA */
#define RDS_GROUP_TYPE_5B	11 /* Transparent Data Channels (32 channels)
				    * or ODA */
#define RDS_GROUP_TYPE_6A	12 /* In House Applications or ODA */
#define RDS_GROUP_TYPE_6B	13 /* In House Applications or ODA */
#define RDS_GROUP_TYPE_7A	14 /* Radio Paging or ODA */
#define RDS_GROUP_TYPE_7B	15 /* Open Data Applications */
#define RDS_GROUP_TYPE_8A	16 /* Traffic Message Channel or ODA */
#define RDS_GROUP_TYPE_8B	17 /* Open Data Applications */
#define RDS_GROUP_TYPE_9A	18 /* Emergency warning system or ODA */
#define RDS_GROUP_TYPE_9B	19 /* Open Data Applications */
#define RDS_GROUP_TYPE_10A	20 /* Program Type Name */
#define RDS_GROUP_TYPE_10B	21 /* Open Data Applications */
#define RDS_GROUP_TYPE_11A	22 /* Open Data Applications */
#define RDS_GROUP_TYPE_11B	23 /* Open Data Applications */
#define RDS_GROUP_TYPE_12A	24 /* Open Data Applications */
#define RDS_GROUP_TYPE_12B	25 /* Open Data Applications */
#define RDS_GROUP_TYPE_13A	26 /* Enhanced Radio Paging or ODA */
#define RDS_GROUP_TYPE_13B	27 /* Open Data Applications */
#define RDS_GROUP_TYPE_14A	28 /* Enhanced Other Networks information */
#define RDS_GROUP_TYPE_14B	29 /* Enhanced Other Networks information */
#define RDS_GROUP_TYPE_15A	30 /* Defined in RBDS */
#define RDS_GROUP_TYPE_15B	31 /* Fast switching information */
#define NUM_DEFINED_RDS_GROUPS	32 /* Number of groups defined in RBDS
				    * standard */

/* Structure representing Generic packet of 64 bits. */
struct rds_group_data {
	u16 piCode;	/* * Program ID */
	u16 offsetB;	/* subject to group type */
	u16 offsetC;	/* subject to group type */
	u16 offsetD;	/* subject to group type */
};

/* Structure representing Group 0A (Service Name) */
struct rds_group0A {
	u16 piCode;	/* * Program ID */
	u16 offsetB;	/* subject to group type */
	u8 freq[2];	/* alt frequency 0=1 */
	u8 text[2];	/* Name segment */
};

/* Structure representing Group 0B (Service Name) */
struct rds_group0B {
	u16 piCode;	/* * Program ID */
	u16 offsetB;	/* subject to group type */
	u16 piCode_dup;	/* Duplicate PI Code */
	u8 text[2];	/* station text */
};

/* Structure representing Group 2A (Radio Text) (64 char) */
struct rds_group2A {
	u16 piCode;	/* * Program ID */
	u16 offsetB;	/* subject to group type */
	u8 text[4];
};

/* Structure representing Group 2B (Radio Text) (32 char) */
struct rds_group2B {
	u16 piCode;	/* * Program ID */
	u16 offsetB;	/* subject to group type */
	u16 piCode_dup;	/* Duplicate PI Code */
	u8 text[2];
};

/* Structure representing all groups */
union rds_msg {
	struct rds_group2B gt2B;
	struct rds_group2A gt2A;
	struct rds_group0B gt0B;
	struct rds_group0A gt0A;
	struct rds_group_data gt00;
};

struct stfm1000_rds_state {
	struct stfm1000_rds_bitstream bitstream;
	struct stfm1000_rds_demod demod;
	struct stfm1000_rds_pkt pkt;
	struct stfm1000_rds_text text;
	unsigned int reset_req : 1;
};

/* callback from rds etc. */
void stfm1000_rds_reset(struct stfm1000_rds_state *rds);
void stfm1000_rds_start(struct stfm1000_rds_state *rds);
void stfm1000_rds_stop(struct stfm1000_rds_state *rds);

/* call these from the monitor thread, but with interrupts disabled */
int stfm1000_rds_mix_msg_get(struct stfm1000_rds_state *rds);
int stfm1000_rds_mix_msg_processed(struct stfm1000_rds_state *rds,
	int mix_msg);
int stfm1000_rds_sdnominal_msg_get(struct stfm1000_rds_state *rds);
int stfm1000_rds_sdnominal_msg_processed(struct stfm1000_rds_state *rds,
	int sdnominal_msg);
int stfm1000_rds_bits_available(struct stfm1000_rds_state *rds);
int stmf1000_rds_get_bit(struct stfm1000_rds_state *rds);

/* called from audio handler (interrupt) */
void stfm1000_rds_demod(struct stfm1000_rds_state *rds, const u16 *dri_data,
	int total);

/* call these from monitor thread, interrupts enabled */
void stfm1000_rds_packet_bit(struct stfm1000_rds_state *rds, int bit);
int stfm1000_rds_packet_dequeue(struct stfm1000_rds_state *rds, u8 *buf);
void stfm1000_rds_process_packet(struct stfm1000_rds_state *rds, u8 *buffer);

static inline int stfm1000_rds_get_reset_req(struct stfm1000_rds_state *rds)
{
	return rds->reset_req;
}

/* GROUP_TYPE 0A-0B */
int stfm1000_rds_get_ps(struct stfm1000_rds_state *rds, u8 *buffer,
	int bufsize);

/* GROUP_TYPE 2A */
int stfm1000_rds_get_text(struct stfm1000_rds_state *rds, u8 *buffer,
	int bufsize);

#endif
