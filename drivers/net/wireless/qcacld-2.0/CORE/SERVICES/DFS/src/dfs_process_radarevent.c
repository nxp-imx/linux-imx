/*
 * Copyright (c) 2002-2014, 2016-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/*===========================================================================

                     dfs_radarevent.c

  OVERVIEW:

  Source code borrowed from QCA_MAIN DFS module

  DEPENDENCIES:

  Are listed for each API below.

===========================================================================*/

/*===========================================================================

                      EDIT HISTORY FOR FILE


  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.



  when        who     what, where, why
----------    ---    --------------------------------------------------------

===========================================================================*/


#include "dfs.h"
/* TO DO DFS
#include <ieee80211_var.h>
*/
#ifdef ATH_SUPPORT_DFS

#define FREQ_5500_MHZ  5500
#define FREQ_5500_MHZ       5500

#define DFS_MAX_FREQ_SPREAD            1375 * 1
#define DFS_INVALID_PRI_LIMIT 100  /* should we use 135? */
#define DFS_BIG_SIDX          10000

#define FRAC_PRI_SCORE_ARRAY_SIZE 40

static char debug_dup[33];
static int debug_dup_cnt;

/*
 * Convert the hardware provided duration to TSF ticks (usecs)
 * taking the clock (fast or normal) into account.
 *
 * Legacy (pre-11n, Owl, Sowl, Howl) operate 5GHz using a 40MHz
 * clock.  Later 11n chips (Merlin, Osprey, etc) operate 5GHz using
 * a 44MHz clock, so the reported pulse durations are different.
 *
 * Peregrine reports the pulse duration in microseconds regardless
 * of the operating mode. (XXX TODO: verify this, obviously.)
 */
static inline u_int8_t
dfs_process_pulse_dur(struct ath_dfs *dfs, u_int8_t re_dur)
{
   /*
    * Short pulses are sometimes returned as having a duration of 0,
    * so round those up to 1.
    *
    * XXX This holds true for BB TLV chips too, right?
    */
   if (re_dur == 0)
      return 1;

   /*
    * For BB TLV chips, the hardware always returns microsecond
    * pulse durations.
    */
   if (dfs->dfs_caps.ath_chip_is_bb_tlv)
      return re_dur;

   /*
    * This is for 11n and legacy chips, which may or may not
    * use the 5GHz fast clock mode.
    */
   /* Convert 0.8us durations to TSF ticks (usecs) */
   return (u_int8_t)dfs_round((int32_t)((dfs->dur_multiplier)*re_dur));
}


/**
 * dfs_confirm_radar() - more rigid check for radar detection
 * check for jitter in frequency (sidx) to be within certain limit
 * introduce a fractional PRI check
 * add a check to look for chirp in ETSI type 4 radar
 * @dfs: pointer to dfs structure
 *
 * Return: max dur difference in sidx1_sidx2 pulse line
 */
static int dfs_confirm_radar(struct ath_dfs *dfs, struct dfs_filter *rf,
			int ext_chan_flag)
{
	int i = 0;
	int index;
	struct dfs_delayline *dl = &rf->rf_dl;
	struct dfs_delayelem *de;
	u_int64_t target_ts = 0;
	struct dfs_pulseline *pl;
	int start_index = 0, current_index, next_index;
	unsigned char scores[FRAC_PRI_SCORE_ARRAY_SIZE];
	u_int32_t pri_margin;
	u_int64_t this_diff_ts;
	u_int32_t search_bin;
	unsigned char max_score = 0;
	int max_score_index = 0;

	pl = dfs->pulses;
	OS_MEMZERO(scores, sizeof(scores));
	scores[0] = rf->rf_threshold;
	pri_margin = dfs_get_pri_margin(dfs, ext_chan_flag,
					(rf->rf_patterntype == 1));

	/**
	 * look for the entry that matches dl_seq_num_second
	 * we need the time stamp and diff_ts from there
	 */
	for (i = 0; i < dl->dl_numelems; i++) {
		index = (dl->dl_firstelem + i) & DFS_MAX_DL_MASK;
		de = &dl->dl_elems[index];
		if (dl->dl_seq_num_second == de->de_seq_num)
			target_ts = de->de_ts - de->de_time;
	}

	if (dfs->dfs_debug_mask & ATH_DEBUG_DFS2) {
		dfs_print_delayline(dfs, &rf->rf_dl);

		/* print pulse line */
		DFS_DPRINTK(dfs, ATH_DEBUG_DFS2, "%s: Pulse Line\n", __func__);
		for (i = 0; i < pl->pl_numelems; i++) {
			index =  (pl->pl_firstelem + i) &
				DFS_MAX_PULSE_BUFFER_MASK;
			DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,
				"Elem %u: ts=%llu dur=%u, seq_num=%d, delta_peak=%d\n",
				i, pl->pl_elems[index].p_time,
				pl->pl_elems[index].p_dur,
				pl->pl_elems[index].p_seq_num,
				pl->pl_elems[index].p_delta_peak);
		}
	}

	/**
	 * walk through the pulse line and find pulse with target_ts
	 * then continue until we find entry with seq_number dl_seq_num_stop
	 */

	for (i = 0; i < pl->pl_numelems; i++) {
		index =  (pl->pl_firstelem + i) & DFS_MAX_PULSE_BUFFER_MASK;
		if (pl->pl_elems[index].p_time == target_ts) {
			dl->dl_seq_num_start = pl->pl_elems[index].p_seq_num;
			/* save for future use */
			start_index = index;
		}
	}

	DFS_DPRINTK(dfs, ATH_DEBUG_DFS2, "%s: target_ts=%llu, dl_seq_num_start=%d, dl_seq_num_second=%d, dl_seq_num_stop=%d\n",
					__func__, target_ts,
					dl->dl_seq_num_start,
					dl->dl_seq_num_second,
					dl->dl_seq_num_stop);

	current_index = start_index;
	while (pl->pl_elems[current_index].p_seq_num < dl->dl_seq_num_stop) {
		next_index = (current_index + 1) & DFS_MAX_PULSE_BUFFER_MASK;
		this_diff_ts = pl->pl_elems[next_index].p_time -
				pl->pl_elems[current_index].p_time;
		/* now update the score for this diff_ts */
		for (i = 1; i < FRAC_PRI_SCORE_ARRAY_SIZE; i++) {
			search_bin = dl->dl_search_pri / (i + 1);

			/**
			 * we do not give score to PRI that is lower then the
			 * limit
			 */
			if (search_bin < DFS_INVALID_PRI_LIMIT)
				break;
			/**
			 * increment the score if this_diff_ts belongs to this
			 * search_bin +/- margin
			 */
			if ((this_diff_ts >= (search_bin - pri_margin)) &&
				(this_diff_ts <= (search_bin + pri_margin)))
				/*increment score */
				scores[i]++;
		}
		current_index = next_index;
	}
	for (i = 0; i < FRAC_PRI_SCORE_ARRAY_SIZE; i++) {
		if (scores[i] > max_score) {
			max_score = scores[i];
			max_score_index = i;
		}
	}
	if (max_score_index != 0) {
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			"%s, Rejecting Radar since Fractional PRI detected: searchpri=%d, threshold=%d, fractional PRI=%d, Fractional PRI score=%d\n",
			__func__, dl->dl_search_pri, scores[0],
			dl->dl_search_pri/(max_score_index + 1), max_score);
		DFS_PRINTK("%s: Rejecting Radar since Fractional PRI detected: searchpri=%d, threshold=%d, fractional PRI=%d, Fractional PRI score=%d\n",
			 __func__, dl->dl_search_pri, scores[0],
			dl->dl_search_pri/(max_score_index + 1), max_score);
		return 0;
	}

	/* check for frequency spread */
	if (dl->dl_min_sidx > pl->pl_elems[start_index].p_sidx)
		dl->dl_min_sidx = pl->pl_elems[start_index].p_sidx;
	if (dl->dl_max_sidx < pl->pl_elems[start_index].p_sidx)
		dl->dl_max_sidx = pl->pl_elems[start_index].p_sidx;
	if ((dl->dl_max_sidx - dl->dl_min_sidx) > rf->rf_sidx_spread) {
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			"%s: Rejecting Radar since frequency spread is too large : min_sidx=%d, max_sidx=%d, rf_sidx_spread=%d\n",
			__func__, dl->dl_min_sidx, dl->dl_max_sidx,
			rf->rf_sidx_spread);
		DFS_PRINTK("%s: Rejecting Radar since frequency spread is too large : min_sidx=%d, max_sidx=%d, rf_sidx_spread=%d\n",
			__func__, dl->dl_min_sidx, dl->dl_max_sidx,
			rf->rf_sidx_spread);
		return 0;
	}

	if ((rf->rf_check_delta_peak) &&
		((dl->dl_delta_peak_match_count) < rf->rf_threshold)) {
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			"%s: Rejecting Radar since delta peak values are invalid : dl_delta_peak_match_count=%d, rf_threshold=%d\n",
			__func__, dl->dl_delta_peak_match_count,
			rf->rf_threshold);

		DFS_PRINTK("%s: Rejecting Radar since delta peak values are invalid : dl_delta_peak_match_count=%d, rf_threshold=%d\n",
			__func__, dl->dl_delta_peak_match_count,
			rf->rf_threshold);
		return 0;
	}
	return 1;
}

/*
 * dfs_process_dc_pulse: process dc pulses
 * @dfs: pointer to dfs structure
 * @event: dfs event
 * @retval: current found status
 * @this_ts: event's ts
 *
 * Return: None
 */
static void dfs_process_dc_pulse(struct ath_dfs *dfs, struct dfs_event *event,
                          int *retval, uint64_t this_ts, int *false_radar_found)
{
    struct dfs_event re;
    struct dfs_state *rs=NULL;
    struct dfs_filtertype *ft;
    struct dfs_filter *rf = NULL;
    int found, p, empty;
    int min_pri, miss_pulse_number = 0, deviation = 0;
    u_int32_t tabledepth = 0;
    u_int64_t deltaT;
    int ext_chan_event_flag = 0;
    int i;

    OS_MEMCPY(&re, event, sizeof(*event));
    if (re.re_chanindex < DFS_NUM_RADAR_STATES)
       rs = &dfs->dfs_radar[re.re_chanindex];

    while ((tabledepth < DFS_MAX_RADAR_OVERLAP) &&
           ((dfs->dfs_dc_radartable[re.re_dur])[tabledepth] != -1) &&
           (!*retval) && (!*false_radar_found)) {
        ft = dfs->dfs_dc_radarf[((dfs->dfs_dc_radartable
                                             [re.re_dur])[tabledepth])];

        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                  FL("** RD (%d): ts %x dur %u rssi %u"),
                  rs->rs_chan.ic_freq,
                  re.re_ts, re.re_dur, re.re_rssi);

        deltaT = this_ts - ft->ft_last_ts;
        if (re.re_rssi < ft->ft_rssithresh && re.re_dur > 4) {
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                   FL("Rejecting on rssi rssi=%u thresh=%u depth=%d"),
                   re.re_rssi, ft->ft_rssithresh,
                   tabledepth);
            tabledepth++;
            dfs_reset_filter_delaylines(ft);
            ATH_DFSQ_LOCK(dfs);
            empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
            ATH_DFSQ_UNLOCK(dfs);
            continue;
        }
        if ((deltaT < ft->ft_minpri) && (deltaT !=0)) {
            /* This check is for the whole filter type. Individual filters
             * will check this again. This is first line of filtering. */
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                FL("Rejecting on pri pri=%lld minpri=%u depth=%d"),
                (unsigned long long)deltaT,
                ft->ft_minpri, tabledepth);
            dfs_reset_filter_delaylines(ft);
            tabledepth++;
            continue;
        }
        for (p = 0, found = 0; (p < ft->ft_numfilters) && (!found) &&
                               (!*false_radar_found); p++) {
            rf = ft->ft_filters[p];
            if ((re.re_dur >= rf->rf_mindur) &&
                (re.re_dur <= rf->rf_maxdur)) {
                deltaT = (this_ts < rf->rf_dl.dl_last_ts) ?
                    (int64_t) ((DFS_TSF_WRAP - rf->rf_dl.dl_last_ts) +
                                this_ts + 1) :
                    this_ts - rf->rf_dl.dl_last_ts;

                if ((deltaT < rf->rf_minpri) && (deltaT != 0)) {
                    /* Second line of PRI filtering. */
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                        FL("filterID=%d :: Rejected and cleared individual filter min PRI deltaT=%lld rf->rf_minpri=%u"),
                        rf->rf_pulseid,
                        (unsigned long long)deltaT, rf->rf_minpri);
                    dfs_reset_delayline(&rf->rf_dl);
                    rf->rf_dl.dl_last_ts = this_ts;
                    continue;
                }

                if (rf->rf_ignore_pri_window > 0) {
                    if (deltaT < rf->rf_minpri) {
                        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                            FL("filterID=%d :: Rejected and cleared on individual filter max PRI deltaT=%lld rf->rf_minpri=%u"),
                            rf->rf_pulseid,
                            (unsigned long long)deltaT, rf->rf_minpri);
                        dfs_reset_delayline(&rf->rf_dl);
                        rf->rf_dl.dl_last_ts = this_ts;
                        continue;
                    }
                } else {
                    if (deltaT < rf->rf_minpri) {
                        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                           FL("filterID=%d :: Rejected and cleared on individual filter max PRI deltaT=%lld rf->rf_minpri=%u"),
                           rf->rf_pulseid,
                           (unsigned long long)deltaT, rf->rf_minpri);
                        dfs_reset_delayline(&rf->rf_dl);
                        rf->rf_dl.dl_last_ts = this_ts;
                        continue;
                    }
                }
                dfs_add_pulse(dfs, rf, &re, deltaT, this_ts);

                /* extra WAR */
                if ((dfs->dfsdomain == DFS_FCC_DOMAIN &&
                    dfs->dfsdomain == DFS_MKK4_DOMAIN) &&
                    ((rf->rf_pulseid != 31) && (rf->rf_pulseid != 32))) {

                    min_pri = 0xffff;
                    for (i = 0; i < rf->rf_dl.dl_numelems; i++) {
                        if (rf->rf_dl.dl_elems[i].de_time  < min_pri)
                            min_pri = rf->rf_dl.dl_elems[i].de_time;
                    }

                    for (i=0; i < rf->rf_dl.dl_numelems; i++) {
                        miss_pulse_number = vos_round_div(
                               (rf->rf_dl.dl_elems[i].de_time), min_pri);
                        deviation = __adf_os_abs(min_pri *
                                        miss_pulse_number -
                                        rf->rf_dl.dl_elems[i].de_time);
                        if (deviation > miss_pulse_number*3) {
                            dfs_reset_delayline(&rf->rf_dl);
                            VOS_TRACE(VOS_MODULE_ID_SAP,
                                VOS_TRACE_LEVEL_INFO,
                                FL("filterID=%d :: cleared individual deleyline min_pri =%d miss_pulse_number =%d deviation =%d"),
                                rf->rf_pulseid,
                                min_pri, miss_pulse_number, deviation);
                        }
                    }
                }

                /* If this is an extension channel event,
                 * flag it for false alarm reduction */
                if (re.re_chanindex == dfs->dfs_extchan_radindex)
                    ext_chan_event_flag = 1;

                if (rf->rf_patterntype == 2)
                    found = dfs_staggered_check(dfs, rf, (uint32_t)deltaT,
                                                re.re_dur);
                else {
                    if (dfs_bin_check(dfs, rf, (uint32_t) deltaT,
                                          re.re_dur, ext_chan_event_flag)) {
                        found = 1;
                        /**
                         * do additioal check to conirm radar except for the
                         * following staggered, chirp FCC Bin 5, frequency
                         * hopping indicated by rf_patterntype == 1
                         */
                        if (rf->rf_patterntype != 1) {
                            found = dfs_confirm_radar(dfs, rf,
                                                      ext_chan_event_flag);
                            *false_radar_found = (found == 1)? 0 : 1;
                        }
                    }
                }

                if (dfs->dfs_debug_mask & ATH_DEBUG_DFS2)
                    dfs_print_delayline(dfs, &rf->rf_dl);

                rf->rf_dl.dl_last_ts = this_ts;
            } else {
                /* if we are rejecting this, clear the queue */
                dfs_reset_delayline(&rf->rf_dl);
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                    FL("filterID= %d :: cleared individual deleyline"),
                    rf->rf_pulseid);
            }
        }
        ft->ft_last_ts = this_ts;
        *retval |= found;
        if (found) {
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                FL(":## Radar Found minDur=%d, filterId=%d ##"),
                ft->ft_mindur,
                rf != NULL ? rf->rf_pulseid : -1);
        }
        tabledepth++;
    }
}

/**
 * dfs_cal_sidx1_sidx2_dur_diff() - cal dur difference in sidx1_sidx2
 * pluse line
 * @dfs: pointer to dfs structure
 *
 * Return: max dur difference in sidx1_sidx2 pulse line
 */
static int dfs_cal_sidx1_sidx2_dur_diff(struct ath_dfs *dfs)
{
	u_int32_t index, loop;
	u_int32_t lowdur, highdur;
	struct dfs_sidx1_sidx2_pulse_line *sidx1_sidx2_p;
	struct dfs_pulseline *pl;

	if (dfs == NULL) {
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
			"%s[%d]: dfs is NULL", __func__, __LINE__);
		return 0;
	}
	pl = dfs->pulses;
	sidx1_sidx2_p = &dfs->sidx1_sidx2_elems;
	lowdur = highdur =
		sidx1_sidx2_p->pl_elems[sidx1_sidx2_p->pl_lastelem].p_dur;
	for (loop = 0; loop < sidx1_sidx2_p->pl_numelems; loop++) {
		index = sidx1_sidx2_p->pl_firstelem + loop;
		index &= DFS_SIDX1_SIDX2_MASK;
		if (sidx1_sidx2_p->pl_elems[index].p_time >
				pl->pl_elems[pl->pl_lastelem].p_time -
			DFS_SIDX1_SIDX2_TIME_WINDOW) {
			if (sidx1_sidx2_p->pl_elems[index].p_dur < lowdur)
				lowdur = sidx1_sidx2_p->pl_elems[index].p_dur;
			if (sidx1_sidx2_p->pl_elems[index].p_dur > highdur)
				highdur = sidx1_sidx2_p->pl_elems[index].p_dur;
		}
	}
	return highdur - lowdur;
}

/*
 * Process a radar event.
 *
 * If a radar event is found, return 1.  Otherwise, return 0.
 *
 * There is currently no way to specify that a radar event has occured on
 * a specific channel, so the current methodology is to mark both the pri
 * and ext channels as being unavailable.  This should be fixed for 802.11ac
 * or we'll quickly run out of valid channels to use.
 */
int
dfs_process_radarevent(struct ath_dfs *dfs, struct ieee80211_channel *chan)
{
    struct dfs_event re,*event;
    struct dfs_state *rs=NULL;
    struct dfs_filtertype *ft;
    struct dfs_filter *rf;
    int found, retval = 0, p, empty;
    int events_processed = 0;
    u_int32_t tabledepth, index, sidx1_sidx2_index;
    u_int64_t deltafull_ts = 0, this_ts, deltaT;
    struct ieee80211_channel *thischan;
    struct dfs_pulseline *pl;
    struct dfs_sidx1_sidx2_pulse_line *sidx1_sidx2_p;
    static u_int32_t  test_ts  = 0;
    static u_int32_t  diff_ts  = 0;
    int ext_chan_event_flag = 0;
#if 0
    int pri_multiplier = 2;
#endif
    int i;
    int false_radar_found = 0;

   if (dfs == NULL) {
      VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                      "%s[%d]: dfs is NULL", __func__, __LINE__);
      return 0;
   }
    pl = dfs->pulses;
    sidx1_sidx2_p = &dfs->sidx1_sidx2_elems;
   adf_os_spin_lock_bh(&dfs->ic->chan_lock);
   if ( !(IEEE80211_IS_CHAN_DFS(dfs->ic->ic_curchan))) {
           adf_os_spin_unlock_bh(&dfs->ic->chan_lock);
           DFS_DPRINTK(dfs, ATH_DEBUG_DFS2, "%s: radar event on non-DFS chan",
                        __func__);
                dfs_reset_radarq(dfs);
                dfs_reset_alldelaylines(dfs);
         return 0;
        }
   adf_os_spin_unlock_bh(&dfs->ic->chan_lock);
#ifndef ATH_DFS_RADAR_DETECTION_ONLY
   /* TEST : Simulate radar bang, make sure we add the channel to NOL (bug 29968) */
        if (dfs->dfs_bangradar) {
                    /* bangradar will always simulate radar found on the primary channel */
           rs = &dfs->dfs_radar[dfs->dfs_curchan_radindex];
           dfs->dfs_bangradar = 0; /* reset */
                DFS_DPRINTK(dfs, ATH_DEBUG_DFS, "%s: bangradar", __func__);
           retval = 1;
                     goto dfsfound;
    }
#endif

        /*
          The HW may miss some pulses especially with high channel loading.
          This is true for Japan W53 where channel loaoding is 50%. Also
          for ETSI where channel loading is 30% this can be an issue too.
          To take care of missing pulses, we introduce pri_margin multiplie.
          This is normally 2 but can be higher for W53.
        */

        if ((dfs->dfsdomain  == DFS_MKK4_DOMAIN) &&
            (dfs->dfs_caps.ath_chip_is_bb_tlv) &&
            (chan->ic_freq < FREQ_5500_MHZ)) {

            dfs->dfs_pri_multiplier = dfs->dfs_pri_multiplier_ini;

            /* do not process W53 pulses,
               unless we have a minimum number of them
             */
            if (dfs->dfs_phyerr_w53_counter >= 5) {
               DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,
                       "%s: w53_counter=%d, freq_max=%d, "
                       "freq_min=%d, pri_multiplier=%d",
                       __func__,
                       dfs->dfs_phyerr_w53_counter,
                       dfs->dfs_phyerr_freq_max,
                       dfs->dfs_phyerr_freq_min,
                       dfs->dfs_pri_multiplier);
                dfs->dfs_phyerr_freq_min     = 0x7fffffff;
                dfs->dfs_phyerr_freq_max     = 0;
            } else {
                return 0;
            }
        }
        DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,
                    "%s: pri_multiplier=%d",
                    __func__,
                    dfs->dfs_pri_multiplier);

   ATH_DFSQ_LOCK(dfs);
   empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
   ATH_DFSQ_UNLOCK(dfs);

   while ((!empty) && (!retval) && (events_processed < MAX_EVENTS) &&
          (!false_radar_found)) {
      ATH_DFSQ_LOCK(dfs);
      event = STAILQ_FIRST(&(dfs->dfs_radarq));
      if (event != NULL)
         STAILQ_REMOVE_HEAD(&(dfs->dfs_radarq), re_list);
      ATH_DFSQ_UNLOCK(dfs);

      if (event == NULL) {
         empty = 1;
         VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "%s[%d]: event is NULL ",__func__,__LINE__);
                        break;
      }
                events_processed++;
                re = *event;

      OS_MEMZERO(event, sizeof(struct dfs_event));
      ATH_DFSEVENTQ_LOCK(dfs);
      STAILQ_INSERT_TAIL(&(dfs->dfs_eventq), event, re_list);
      ATH_DFSEVENTQ_UNLOCK(dfs);

      found = 0;

      adf_os_spin_lock_bh(&dfs->ic->chan_lock);
      if (dfs->ic->disable_phy_err_processing) {
         ATH_DFSQ_LOCK(dfs);
         empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
         ATH_DFSQ_UNLOCK(dfs);
         adf_os_spin_unlock_bh(&dfs->ic->chan_lock);
         continue;
      }

      adf_os_spin_unlock_bh(&dfs->ic->chan_lock);

      if (re.re_chanindex < DFS_NUM_RADAR_STATES)
         rs = &dfs->dfs_radar[re.re_chanindex];
      else {
         ATH_DFSQ_LOCK(dfs);
         empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
         ATH_DFSQ_UNLOCK(dfs);
         continue;
      }
      if (rs->rs_chan.ic_flagext & CHANNEL_INTERFERENCE) {
         ATH_DFSQ_LOCK(dfs);
         empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
         ATH_DFSQ_UNLOCK(dfs);
         continue;
      }

      if (dfs->dfs_rinfo.rn_lastfull_ts == 0) {
         /*
          * Either not started, or 64-bit rollover exactly to zero
          * Just prepend zeros to the 15-bit ts
          */
         dfs->dfs_rinfo.rn_ts_prefix = 0;
      } else {
                         /* WAR 23031- patch duplicate ts on very short pulses */
                        /* This pacth has two problems in linux environment.
                         * 1)The time stamp created and hence PRI depends entirely on the latency.
                         *   If the latency is high, it possibly can split two consecutive
                         *   pulses in the same burst so far away (the same amount of latency)
                         *   that make them look like they are from differenct bursts. It is
                         *   observed to happen too often. It sure makes the detection fail.
                         * 2)Even if the latency is not that bad, it simply shifts the duplicate
                         *   timestamps to a new duplicate timestamp based on how they are processed.
                         *   This is not worse but not good either.
                         *
                         *   Take this pulse as a good one and create a probable PRI later
                         */
                        if (re.re_dur == 0 && re.re_ts == dfs->dfs_rinfo.rn_last_unique_ts) {
                                debug_dup[debug_dup_cnt++] = '1';
                                DFS_DPRINTK(dfs, ATH_DEBUG_DFS1, " %s deltaT is 0 ", __func__);
                        } else {
                                dfs->dfs_rinfo.rn_last_unique_ts = re.re_ts;
                                debug_dup[debug_dup_cnt++] = '0';
                        }
                        if (debug_dup_cnt >= 32){
                                 debug_dup_cnt = 0;
                        }


         if (re.re_ts <= dfs->dfs_rinfo.rn_last_ts) {
            dfs->dfs_rinfo.rn_ts_prefix +=
               (((u_int64_t) 1) << DFS_TSSHIFT);
            /* Now, see if it's been more than 1 wrap */
            deltafull_ts = re.re_full_ts - dfs->dfs_rinfo.rn_lastfull_ts;
            if (deltafull_ts > ((u_int64_t)(DFS_TSMASK -
                                      dfs->dfs_rinfo.rn_last_ts) +
                                      1 + re.re_ts))
               deltafull_ts -= (DFS_TSMASK - dfs->dfs_rinfo.rn_last_ts) + 1 + re.re_ts;
            deltafull_ts = deltafull_ts >> DFS_TSSHIFT;
            if (deltafull_ts > 1) {
               dfs->dfs_rinfo.rn_ts_prefix +=
                  ((deltafull_ts - 1) << DFS_TSSHIFT);
            }
         } else {
            deltafull_ts = re.re_full_ts - dfs->dfs_rinfo.rn_lastfull_ts;
            if (deltafull_ts > (u_int64_t) DFS_TSMASK) {
               deltafull_ts = deltafull_ts >> DFS_TSSHIFT;
               dfs->dfs_rinfo.rn_ts_prefix +=
                  ((deltafull_ts - 1) << DFS_TSSHIFT);
            }
         }
      }
      /*
       * At this stage rn_ts_prefix has either been blanked or
       * calculated, so it's safe to use.
       */
      this_ts = dfs->dfs_rinfo.rn_ts_prefix | ((u_int64_t) re.re_ts);
      dfs->dfs_rinfo.rn_lastfull_ts = re.re_full_ts;
      dfs->dfs_rinfo.rn_last_ts = re.re_ts;

      /*
       * The hardware returns the duration in a variety of formats,
       * so it's converted from the hardware format to TSF (usec)
       * values here.
       *
       * XXX TODO: this should really be done when the PHY error
       * is processed, rather than way out here..
       */
      re.re_dur = dfs_process_pulse_dur(dfs, re.re_dur);

      /*
       * Calculate the start of the radar pulse.
       *
       * The TSF is stamped by the MAC upon reception of the
       * event, which is (typically?) at the end of the event.
       * But the pattern matching code expects the event timestamps
       * to be at the start of the event.  So to fake it, we
       * subtract the pulse duration from the given TSF.
       *
       * This is done after the 64-bit timestamp has been calculated
       * so long pulses correctly under-wrap the counter.  Ie, if
       * this was done on the 32 (or 15!) bit TSF when the TSF
       * value is closed to 0, it will underflow to 0xfffffXX, which
       * would mess up the logical "OR" operation done above.
       *
       * This isn't valid for Peregrine as the hardware gives us
       * the actual TSF offset of the radar event, not just the MAC
       * TSF of the completed receive.
       *
       * XXX TODO: ensure that the TLV PHY error processing
       * code will correctly calculate the TSF to be the start
       * of the radar pulse.
       *
       * XXX TODO TODO: modify the TLV parsing code to subtract
       * the duration from the TSF, based on the current fast clock
       * value.
       */
      if ((! dfs->dfs_caps.ath_chip_is_bb_tlv) && re.re_dur != 1) {
         this_ts -= re.re_dur;
      }

              /* Save the pulse parameters in the pulse buffer(pulse line) */
                index = (pl->pl_lastelem + 1) & DFS_MAX_PULSE_BUFFER_MASK;
                if (pl->pl_numelems == DFS_MAX_PULSE_BUFFER_SIZE)
                        pl->pl_firstelem = (pl->pl_firstelem+1) & DFS_MAX_PULSE_BUFFER_MASK;
                else
                        pl->pl_numelems++;
                pl->pl_lastelem = index;
                pl->pl_elems[index].p_time = this_ts;
                pl->pl_elems[index].p_dur = re.re_dur;
                pl->pl_elems[index].p_rssi = re.re_rssi;
                pl->pl_elems[index].p_sidx = re.sidx;
                pl->pl_elems[index].p_delta_peak = re.re_delta_peak;
                if (dfs->dfs_enable_radar_war &&
                            (re.sidx == 1 || re.sidx == 2)) {
                        sidx1_sidx2_index = (sidx1_sidx2_p->pl_lastelem + 1) &
                                DFS_SIDX1_SIDX2_MASK;
                        if (sidx1_sidx2_p->pl_numelems == DFS_SIDX1_SIDX2_SIZE)
                                sidx1_sidx2_p->pl_firstelem =
                                        (sidx1_sidx2_p->pl_firstelem + 1) &
                                        DFS_SIDX1_SIDX2_MASK;
                        else
                                sidx1_sidx2_p->pl_numelems++;
                        sidx1_sidx2_p->pl_lastelem = sidx1_sidx2_index;
                        sidx1_sidx2_p->pl_elems[sidx1_sidx2_index].p_time =
                                this_ts;
                        sidx1_sidx2_p->pl_elems[sidx1_sidx2_index].p_dur =
                                re.re_dur;
                }
                diff_ts = (u_int32_t)this_ts - test_ts;
                test_ts = (u_int32_t)this_ts;
                DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,"ts%u %u %u diff %u pl->pl_lastelem.p_time=%llu",(u_int32_t)this_ts, re.re_dur, re.re_rssi, diff_ts, (unsigned long long)pl->pl_elems[index].p_time);
                if (dfs->dfs_event_log_on) {
                        i = dfs->dfs_event_log_count % DFS_EVENT_LOG_SIZE;
                        dfs->radar_log[i].ts      = this_ts;
                        dfs->radar_log[i].diff_ts = diff_ts;
                        dfs->radar_log[i].rssi    = re.re_rssi;
                        dfs->radar_log[i].dur     = re.re_dur;
                        dfs->dfs_event_log_count++;
                }
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                    "%s[%d]:xxxxx ts =%u re.re_dur=%u re.re_rssi =%u diff =%u sidx = %u flags = %x pl->pl_lastelem.p_time=%llu xxxxx",
                    __func__, __LINE__, (u_int32_t)this_ts,
                    re.re_dur, re.re_rssi, diff_ts, re.sidx, re.re_flags,
                    (unsigned long long)pl->pl_elems[index].p_time);
                dfs->dfs_seq_num++;
                pl->pl_elems[index].p_seq_num = dfs->dfs_seq_num;

                /* If diff_ts is very small, we might be getting false pulse detects
                 * due to heavy interference. We might be getting spectral splatter
                 * from adjacent channel. In order to prevent false alarms we
                 * clear the delay-lines. This might impact positive detections under
                 * harsh environments, but helps with false detects. */

         if (diff_ts < DFS_INVALID_PRI_LIMIT) {
            dfs->dfs_seq_num = 0;
            dfs_reset_alldelaylines(dfs);
            dfs_reset_radarq(dfs);
            index = (pl->pl_lastelem + 1) & DFS_MAX_PULSE_BUFFER_MASK;
            if (pl->pl_numelems == DFS_MAX_PULSE_BUFFER_SIZE)
                pl->pl_firstelem = (pl->pl_firstelem+1) &
                                    DFS_MAX_PULSE_BUFFER_MASK;
            else
                pl->pl_numelems++;
            pl->pl_lastelem = index;
            pl->pl_elems[index].p_time = this_ts;
            pl->pl_elems[index].p_dur = re.re_dur;
            pl->pl_elems[index].p_rssi = re.re_rssi;
            pl->pl_elems[index].p_sidx = re.sidx;
            pl->pl_elems[index].p_delta_peak = re.re_delta_peak;
            dfs->dfs_seq_num++;
            pl->pl_elems[index].p_seq_num = dfs->dfs_seq_num;
         }

         found = 0;

         /*
          * Use this fix only when device is not in test mode, as
          * it drops some valid phyerrors.
          * In FCC or JAPAN domain,if the follwing signature matches
          * its likely that this is a false radar pulse pattern
          * so process the next pulse in the queue.
          */
         if ((dfs->disable_dfs_ch_switch == VOS_FALSE) &&
             (DFS_FCC_DOMAIN == dfs->dfsdomain ||
              DFS_MKK4_DOMAIN == dfs->dfsdomain) &&
             (re.re_dur >= 11 && re.re_dur <= 20) &&
             (diff_ts > 500 || diff_ts <= 305) &&
             (re.sidx == -4)) {
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
            "\n%s: Rejecting on Peak Index = %d,re.re_dur = %d,diff_ts = %d\n",
            __func__,re.sidx, re.re_dur, diff_ts);

            ATH_DFSQ_LOCK(dfs);
            empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
            ATH_DFSQ_UNLOCK(dfs);
            continue;
         }

         /*
          * Modifying the pulse duration for FCC Type 4
          * or JAPAN W56 Type 6 radar pulses when the
          * following condition is reported in radar
          * summary report.
          */
         if ((DFS_FCC_DOMAIN == dfs->dfsdomain ||
              DFS_MKK4_DOMAIN == dfs->dfsdomain) &&
             ((chan->ic_flags & IEEE80211_CHAN_VHT80) ==
              IEEE80211_CHAN_VHT80) &&
             (chan->ic_pri_freq_center_freq_mhz_separation ==
                                DFS_WAR_PLUS_30_MHZ_SEPARATION ||
              chan->ic_pri_freq_center_freq_mhz_separation ==
                                DFS_WAR_MINUS_30_MHZ_SEPARATION) &&
             (re.sidx == DFS_WAR_PEAK_INDEX_ZERO) &&
             (re.re_dur > DFS_TYPE4_WAR_PULSE_DURATION_LOWER_LIMIT &&
              re.re_dur < DFS_TYPE4_WAR_PULSE_DURATION_UPPER_LIMIT) &&
             (diff_ts > DFS_TYPE4_WAR_PRI_LOWER_LIMIT &&
              diff_ts < DFS_TYPE4_WAR_PRI_UPPER_LIMIT)) {
             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                       "\n%s:chan->ic_flags=0x%x, Pri Chan MHz Separation=%d\n",
                       __func__, chan->ic_flags,
                       chan->ic_pri_freq_center_freq_mhz_separation);

             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                "\n%s: Reported Peak Index = %d,re.re_dur = %d,diff_ts = %d\n",
                 __func__, re.sidx, re.re_dur, diff_ts);

             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                "\n%s: Modifying the pulse duration to fit the valid range \n",
                 __func__);

             re.re_dur = DFS_TYPE4_WAR_VALID_PULSE_DURATION;
         }

         /*
          * Modifying the pulse duration for ETSI Type 2
          * and ETSI type 3 radar pulses when the following
          * condition is reported in radar summary report.
          */
         if ((DFS_ETSI_DOMAIN == dfs->dfsdomain) &&
             ((chan->ic_flags & IEEE80211_CHAN_VHT80) ==
              IEEE80211_CHAN_VHT80) &&
             (chan->ic_pri_freq_center_freq_mhz_separation ==
                                DFS_WAR_PLUS_30_MHZ_SEPARATION ||
              chan->ic_pri_freq_center_freq_mhz_separation ==
                                DFS_WAR_MINUS_30_MHZ_SEPARATION) &&
             (re.sidx == DFS_WAR_PEAK_INDEX_ZERO) &&
             (re.re_dur > DFS_ETSI_TYPE2_TYPE3_WAR_PULSE_DUR_LOWER_LIMIT &&
              re.re_dur < DFS_ETSI_TYPE2_TYPE3_WAR_PULSE_DUR_UPPER_LIMIT) &&
             ((diff_ts > DFS_ETSI_TYPE2_WAR_PRI_LOWER_LIMIT &&
               diff_ts < DFS_ETSI_TYPE2_WAR_PRI_UPPER_LIMIT) ||
              (diff_ts > DFS_ETSI_TYPE3_WAR_PRI_LOWER_LIMIT &&
               diff_ts < DFS_ETSI_TYPE3_WAR_PRI_UPPER_LIMIT ))) {
             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                       "\n%s:chan->ic_flags=0x%x, Pri Chan MHz Separation=%d\n",
                       __func__, chan->ic_flags,
                       chan->ic_pri_freq_center_freq_mhz_separation);

             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                "\n%s: Reported Peak Index = %d,re.re_dur = %d,diff_ts = %d\n",
                 __func__, re.sidx, re.re_dur, diff_ts);

             VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                "\n%s: Modifying the ETSI pulse dur to fit the valid range \n",
                 __func__);

             re.re_dur  = DFS_ETSI_WAR_VALID_PULSE_DURATION;
         }

      /* BIN5 pulses are FCC and Japan specific */

      if ((dfs->dfsdomain == DFS_FCC_DOMAIN) || (dfs->dfsdomain == DFS_MKK4_DOMAIN)) {
         for (p=0; (p < dfs->dfs_rinfo.rn_numbin5radars) && (!found); p++) {
            struct dfs_bin5radars *br;

            br = &(dfs->dfs_b5radars[p]);
            if (dfs_bin5_check_pulse(dfs, &re, br)) {
               // This is a valid Bin5 pulse, check if it belongs to a burst
               re.re_dur = dfs_retain_bin5_burst_pattern(dfs, diff_ts, re.re_dur);
               // Remember our computed duration for the next pulse in the burst (if needed)
               dfs->dfs_rinfo.dfs_bin5_chirp_ts = this_ts;
               dfs->dfs_rinfo.dfs_last_bin5_dur = re.re_dur;

               if( dfs_bin5_addpulse(dfs, br, &re, this_ts) ) {
                  found |= dfs_bin5_check(dfs);
               }
            } else{
               DFS_DPRINTK(dfs, ATH_DEBUG_DFS_BIN5_PULSE,
                   "%s not a BIN5 pulse (dur=%d)",
                   __func__, re.re_dur);
                                 }
         }
      }

      if (found) {
         DFS_DPRINTK(dfs, ATH_DEBUG_DFS, "%s: Found bin5 radar", __func__);
         retval |= found;
         goto dfsfound;
      }

      tabledepth = 0;
      rf = NULL;
      DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,"  *** chan freq (%d): ts %llu dur %u rssi %u",
         rs->rs_chan.ic_freq, (unsigned long long)this_ts, re.re_dur, re.re_rssi);


      /*
       * DC pulses processing will be done in this seperate block since some
       * device may generate pulse which may cause false detection.
       * This processing is similar kind as of normal pulse processing with
       * some exception such as:
       * 1. Clear the queue if pulse doesn't belong to it
       * 2. Remove chan load optimization(can cause some valid pulses to drop)
       * 3. Drop pulses on basis of mean deviation for some filters
       */
      if (((re.sidx == 0) && DFS_EVENT_NOTCHIRP(&re)) &&
              ((dfs->dfsdomain == DFS_FCC_DOMAIN) ||
               (dfs->dfsdomain == DFS_MKK4_DOMAIN) ||
               (dfs->dfsdomain == DFS_ETSI_DOMAIN && re.re_dur < 18))) {
          dfs_process_dc_pulse(dfs, &re, &retval, this_ts, &false_radar_found);
      }

      /* Pulse not at DC position */
      else {
        while ((tabledepth < DFS_MAX_RADAR_OVERLAP) &&
             ((dfs->dfs_radartable[re.re_dur])[tabledepth] != -1) &&
             (!retval) && (!false_radar_found)) {
         ft = dfs->dfs_radarf[((dfs->dfs_radartable[re.re_dur])[tabledepth])];
         DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,"  ** RD (%d): ts %x dur %u rssi %u",
                   rs->rs_chan.ic_freq,
                   re.re_ts, re.re_dur, re.re_rssi);

         if (re.re_rssi < ft->ft_rssithresh && re.re_dur > 4) {
               DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,"%s : Rejecting on rssi rssi=%u thresh=%u", __func__, re.re_rssi, ft->ft_rssithresh);
                           VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "%s[%d]: Rejecting on rssi rssi=%u thresh=%u",__func__,__LINE__,re.re_rssi, ft->ft_rssithresh);
                     tabledepth++;
            ATH_DFSQ_LOCK(dfs);
            empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
            ATH_DFSQ_UNLOCK(dfs);
            continue;
         }
         deltaT = this_ts - ft->ft_last_ts;
         DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,"deltaT = %lld (ts: 0x%llx) (last ts: 0x%llx)",(unsigned long long)deltaT, (unsigned long long)this_ts, (unsigned long long)ft->ft_last_ts);
         if ((deltaT < ft->ft_minpri) && (deltaT !=0)){
                                /* This check is for the whole filter type. Individual filters
                                 will check this again. This is first line of filtering.*/
            DFS_DPRINTK(dfs, ATH_DEBUG_DFS2, "%s: Rejecting on pri pri=%lld minpri=%u", __func__, (unsigned long long)deltaT, ft->ft_minpri);
                                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "%s[%d]:Rejecting on pri pri=%lld minpri=%u",__func__,__LINE__,(unsigned long long)deltaT,ft->ft_minpri);
                                tabledepth++;
            continue;
         }
         for (p=0, found = 0; (p<ft->ft_numfilters) && (!found) &&
                              (!false_radar_found); p++) {
                                    rf = ft->ft_filters[p];
                                    if ((re.re_dur >= rf->rf_mindur) && (re.re_dur <= rf->rf_maxdur)) {
                                        /* The above check is probably not necessary */
                                        deltaT = (this_ts < rf->rf_dl.dl_last_ts) ?
                                            (int64_t) ((DFS_TSF_WRAP - rf->rf_dl.dl_last_ts) + this_ts + 1) :
                                            this_ts - rf->rf_dl.dl_last_ts;

                                        if ((deltaT < rf->rf_minpri) && (deltaT != 0)) {
                                                /* Second line of PRI filtering. */
                                                DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,
                                                "filterID %d : Rejecting on individual filter min PRI deltaT=%lld rf->rf_minpri=%u",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "%s[%d]:filterID= %d::Rejecting on individual filter min PRI deltaT=%lld rf->rf_minpri=%u",__func__,__LINE__,rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                continue;
                                        }

                                        if (rf->rf_ignore_pri_window > 0) {
                                           if (deltaT < rf->rf_minpri) {
                                                DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,
                                                "filterID %d : Rejecting on individual filter max PRI deltaT=%lld rf->rf_minpri=%u",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "%s[%d]:filterID= %d :: Rejecting on individual filter max PRI deltaT=%lld rf->rf_minpri=%u",__func__,__LINE__,rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                /* But update the last time stamp */
                                                rf->rf_dl.dl_last_ts = this_ts;
                                                continue;
                                           }
                                        } else {

                                        /*
                                            The HW may miss some pulses especially with high channel loading.
                                            This is true for Japan W53 where channel loaoding is 50%. Also
                                            for ETSI where channel loading is 30% this can be an issue too.
                                            To take care of missing pulses, we introduce pri_margin multiplie.
                                            This is normally 2 but can be higher for W53.
                                        */

                                        if ( (deltaT > ((u_int64_t)dfs->dfs_pri_multiplier * rf->rf_maxpri) ) || (deltaT < rf->rf_minpri) ) {
                                                DFS_DPRINTK(dfs, ATH_DEBUG_DFS2,
                                                "filterID %d : Rejecting on individual filter max PRI deltaT=%lld rf->rf_minpri=%u",
                                                rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "%s[%d]:filterID= %d :: Rejecting on individual filter max PRI deltaT=%lld rf->rf_minpri=%u",__func__,__LINE__,rf->rf_pulseid, (unsigned long long)deltaT, rf->rf_minpri);
                                                /* But update the last time stamp */
                                                rf->rf_dl.dl_last_ts = this_ts;
                                                continue;
                                        }
                                        }
                                        dfs_add_pulse(dfs, rf, &re, deltaT, this_ts);


                                       /* If this is an extension channel event, flag it for false alarm reduction */
                                        if (re.re_chanindex == dfs->dfs_extchan_radindex) {
                                            ext_chan_event_flag = 1;
                                        }
                                        if (rf->rf_patterntype == 2) {
                                            found = dfs_staggered_check(dfs, rf, (u_int32_t) deltaT, re.re_dur);
                                        } else {
                                            if (dfs_bin_check(dfs, rf,
                                                         (u_int32_t) deltaT,
                                                         re.re_dur,
                                                         ext_chan_event_flag)) {
                                                found = 1;
                                                /**
                                                 * do additioal check to conirm
                                                 * radar except for the
                                                 * following staggered, chirp
                                                 * FCC Bin 5, frequency hopping
                                                 * indicated by
                                                 * rf_patterntype == 1
                                                 */
                                                if (rf->rf_patterntype != 1) {
                                                    found = dfs_confirm_radar(
                                                           dfs, rf,
                                                           ext_chan_event_flag);
                                                    false_radar_found =
                                                            (found == 1)? 0 : 1;
                                                }
                                            }
                                        }
                                        if (dfs->dfs_debug_mask & ATH_DEBUG_DFS2) {
                                            if (rf->rf_patterntype != 1)
                                                dfs_print_delayline(dfs, &rf->rf_dl);
                                        }
                                        rf->rf_dl.dl_last_ts = this_ts;
                                }
                            }
         ft->ft_last_ts = this_ts;
         retval |= found;
         if (found) {
            DFS_DPRINTK(dfs, ATH_DEBUG_DFS3,
               "Found on channel minDur = %d, filterId = %d",ft->ft_mindur,
               rf != NULL ? rf->rf_pulseid : -1);
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                 "%s[%d]:### Found on channel minDur = %d, filterId = %d ###",
                 __func__,__LINE__,ft->ft_mindur,
                 rf != NULL ? rf->rf_pulseid : -1);
         }
         tabledepth++;
        }
      }
      ATH_DFSQ_LOCK(dfs);
      empty = STAILQ_EMPTY(&(dfs->dfs_radarq));
      ATH_DFSQ_UNLOCK(dfs);
   }
dfsfound:
   if (retval) {
      if (dfs->dfs_enable_radar_war &&
            (DFS_SIDX1_SIDX2_DR_LIM < dfs_cal_sidx1_sidx2_dur_diff(dfs))) {
         VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
            "%s [%d] false detection",__func__,__LINE__);
         return 0;
      }

      /* Collect stats */
      dfs->ath_dfs_stats.num_radar_detects++;
      thischan = &rs->rs_chan;
   VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "%s[%d]: ### RADAR FOUND ON CHANNEL %d (%d MHz) ###",__func__,__LINE__,thischan->ic_ieee,
thischan->ic_freq);
      DFS_PRINTK("Radar found on channel %d (%d MHz)",
          thischan->ic_ieee,
          thischan->ic_freq);

#if 0 //UMACDFS : TODO
      /* Disable radar for now */
      rfilt = ath_hal_getrxfilter(ah);
      rfilt &= ~HAL_RX_FILTER_PHYRADAR;
      ath_hal_setrxfilter(ah, rfilt);
#endif
      dfs_reset_radarq(dfs);
      dfs_reset_alldelaylines(dfs);
      /* XXX Should we really enable again? Maybe not... */
/* No reason to re-enable so far - Ajay*/
#if 0
      pe.pe_firpwr = rs->rs_firpwr;
      pe.pe_rrssi = rs->rs_radarrssi;
      pe.pe_height = rs->rs_height;
      pe.pe_prssi = rs->rs_pulserssi;
      pe.pe_inband = rs->rs_inband;
      /* 5413 specific */
      pe.pe_relpwr = rs->rs_relpwr;
      pe.pe_relstep = rs->rs_relstep;
      pe.pe_maxlen = rs->rs_maxlen;

      ath_hal_enabledfs(ah, &pe);
      rfilt |= HAL_RX_FILTER_PHYRADAR;
      ath_hal_setrxfilter(ah, rfilt);
#endif
         DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,
             "Primary channel freq = %u flags=0x%x",
             chan->ic_freq, chan->ic_flagext);
         adf_os_spin_lock_bh(&dfs->ic->chan_lock);
         if ((dfs->ic->ic_curchan->ic_freq!= thischan->ic_freq)) {
         DFS_DPRINTK(dfs, ATH_DEBUG_DFS1,
             "Ext channel freq = %u flags=0x%x",
             thischan->ic_freq, thischan->ic_flagext);
      }

      adf_os_spin_unlock_bh(&dfs->ic->chan_lock);
                dfs->dfs_phyerr_freq_min     = 0x7fffffff;
                dfs->dfs_phyerr_freq_max     = 0;
                dfs->dfs_phyerr_w53_counter  = 0;
   }
        //VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO, "IN FUNC %s[%d]: retval = %d ",__func__,__LINE__,retval);
   if (false_radar_found) {
       dfs->dfs_seq_num = 0;
       dfs_reset_radarq(dfs);
       dfs_reset_alldelaylines(dfs);
       dfs->dfs_phyerr_freq_min     = 0x7fffffff;
       dfs->dfs_phyerr_freq_max     = 0;
       dfs->dfs_phyerr_w53_counter  = 0;
   }
   return retval;
//        return 1;
}

#endif /* ATH_SUPPORT_DFS */
