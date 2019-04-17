/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Module Name:    TimeStamp.c
 *
 * Description:    include TimeStamp stratege for VPU / SW video decoder plugin
 *
 * Portability:    This code is written for Linux OS and Gstreamer
 */

#include <linux/slab.h>
#include <linux/string.h>

#include "vpu_ts.h"

int debug_level;

static void tsm_free_received_entry(TSMRecivedCtl *rctl,
				TSMReceivedEntry *entry)
{
	entry->next = rctl->free;
	rctl->free = entry;
}


static TSMReceivedEntry *tsm_new_received_entry(TSMRecivedCtl *rctl)
{
	TSMReceivedEntry *ret = NULL;

	if (rctl->free) {
		ret = rctl->free;
		rctl->free = ret->next;
	} else {
		TSMReceivedEntryMemory *p = kzalloc(sizeof(TSMReceivedEntryMemory), GFP_KERNEL);

		if (p) {
			int i;

			for (i = 1; i < TSM_RECEIVED_NUNBER; i++) {
				TSMReceivedEntry *e = &p->entrys[i];

				tsm_free_received_entry(rctl, e);
			};
			p->next = rctl->memory;
			rctl->memory = p;
			ret = p->entrys;
		}
	}
	return ret;
}


void TSManagerReceive2(void *handle, TSM_TIMESTAMP timestamp, int size)
{
#define CLEAR_TSM_RENTRY(entry)\
	do { \
		(entry)->used = 0; \
		(entry)->subentry = 0; \
		(entry)->next = NULL; \
	} while (0)
	TSManager *tsm = (TSManager *) handle;

	TSM_VERBOSE("receive2 %u:%02u:%02u.%09u size %d\n",
			TSM_TIME_H(timestamp),
			TSM_TIME_M(timestamp),
			TSM_TIME_S(timestamp),
			TSM_TIME_NS(timestamp),
			size);

	if (tsm) {
		if (size > 0) {
			TSMRecivedCtl *rctl = &tsm->rctl;
			TSMReceivedEntry *e = tsm_new_received_entry(rctl);

			if (e) {
				CLEAR_TSM_RENTRY(e);
				if ((rctl->tail) && (rctl->tail->ts == timestamp))
					e->subentry = 1;
				e->ts = timestamp;
				e->size = size;
				if (rctl->tail) {
					rctl->tail->next = e;
					rctl->tail = e;
				} else
					rctl->head = rctl->tail = e;
			}
			rctl->cnt++;
		} else
			TSManagerReceive(handle, timestamp);
	}
}


static TSM_TIMESTAMP TSManagerGetLastTimeStamp(TSMRecivedCtl *rctl,
						int size, int use)
{
	TSM_TIMESTAMP ts = TSM_TIMESTAMP_NONE;
	TSMReceivedEntry *e;

	while ((size > 0) && (e = rctl->head)) {
		ts = ((e->used) ? (TSM_TIMESTAMP_NONE) : (e->ts));

		TSM_VERBOSE("ts get: %u:%02u:%02u.%09u\n",
				TSM_TIME_H(ts),
				TSM_TIME_M(ts),
				TSM_TIME_S(ts),
				TSM_TIME_NS(ts));

		if (use)
			e->used = 1;
		if (size >= e->size) {
			rctl->head = e->next;
			if (rctl->head == NULL)
				rctl->tail = NULL;
			else {
				if (rctl->head->subentry)
					rctl->head->used = e->used;
			}
			size -= e->size;
			rctl->cnt--;
			tsm_free_received_entry(rctl, e);
		} else {
			e->size -= size;
			size = 0;
		}
	}
	return ts;
}


void TSManagerFlush2(void *handle, int size)
{
	TSManager *tsm = (TSManager *) handle;

	if (tsm)
		TSManagerGetLastTimeStamp(&tsm->rctl, size, 0);
}


/*======================================================================================
FUNCTION:           mfw_gst_receive_ts

DESCRIPTION:        Check timestamp and do frame dropping if enabled

ARGUMENTS PASSED:   pTimeStamp_Object  - TimeStamp Manager to handle related timestamp
					timestamp - time stamp of the input buffer which has video data.
RETURN VALUE:       None
PRE-CONDITIONS:     None
POST-CONDITIONS:    None
IMPORTANT NOTES:    None
=======================================================================================*/
static void _TSManagerReceive(void *handle, TSM_TIMESTAMP timestamp, void *key)
{
	TSManager *tsm = (TSManager *) handle;

	if (tsm) {
		if (TSM_TS_IS_VALID(timestamp) && (tsm->rx_cnt))
			tsm->valid_ts_received = 1;
		tsm->rx_cnt++;
		if (tsm->cnt < tsm->ts_buf_size - 1) {
			tsm->cnt++;
			if (tsm->mode == MODE_AI) {
				if (TSM_TS_IS_VALID(timestamp)) {
					if (tsm->first_rx) {
						tsm->last_ts_received = timestamp;
						tsm->first_rx = 0;
					} else {
						if (tsm->suspicious_ts) {
							if (timestamp >= tsm->suspicious_ts)
								tsm->last_ts_received = timestamp;
							tsm->suspicious_ts = 0;
						}
						if ((timestamp > tsm->last_ts_received)
								&& (timestamp - tsm->last_ts_received > tsm->discont_threshold)) {
							tsm->suspicious_ts = timestamp;
							timestamp = TSM_TIMESTAMP_NONE;
						}
					}
				}

				if (TSM_TS_IS_VALID(timestamp)) {        // && (TSM_ABS(timestamp, tsm->last_ts_sent)<TSM_SECOND*10))
					tsm->ts_buf[tsm->rx].ts = timestamp;
					tsm->ts_buf[tsm->rx].age = tsm->age + TSM_PLUS_AGE(tsm);
					tsm->ts_buf[tsm->rx].key = key;
					tsm->last_ts_received = timestamp;
#ifdef DEBUG
					//printf("age should %lld %lld\n", tsm->age, tsm->ts_buf[tsm->rx].age);
					//printf("++++++ distance = %d  tx=%d, rx=%d, invalid count=%d\n", TSM_DISTANCE(tsm), tsm->tx, tsm->rx,tsm->invalid_ts_count);
#endif
					tsm->rx = ((tsm->rx + 1) % tsm->ts_buf_size);
				} else
					tsm->invalid_ts_count++;
			} else if (tsm->mode == MODE_FIFO) {
				tsm->ts_buf[tsm->rx].ts = timestamp;
				tsm->rx = ((tsm->rx + 1) % tsm->ts_buf_size);
			}
			TSM_LOG("++Receive %d:%u:%02u:%02u.%09u, invalid:%d, size:%d key %p\n",
					tsm->rx_cnt,
					TSM_TIME_H(timestamp),
					TSM_TIME_M(timestamp),
					TSM_TIME_S(timestamp),
					TSM_TIME_NS(timestamp),
					tsm->invalid_ts_count,
					tsm->cnt,
					key);
		} else
			TSM_ERROR("Too many timestamps recieved!! (cnt=%d)\n", tsm->cnt);
	}
}

void TSManagerValid2(void *handle, int size, void *key)
{
	TSManager *tsm = (TSManager *) handle;

	TSM_VERBOSE("valid2 size %d\n", size);

	if (tsm) {
		TSM_TIMESTAMP ts;

		ts = TSManagerGetLastTimeStamp(&tsm->rctl, size, 1);
		TSM_VERBOSE("TSManagerGetLastTimeStamp: %u:%02u:%02u.%09u\n",
				TSM_TIME_H(ts),
				TSM_TIME_M(ts),
				TSM_TIME_S(ts),
				TSM_TIME_NS(ts));
		_TSManagerReceive(tsm, ts, key);
	}
}

void TSManagerReceive(void *handle, TSM_TIMESTAMP timestamp)
{
	_TSManagerReceive(handle, timestamp, TSM_KEY_NONE);
}


/*======================================================================================
FUNCTION:           TSManagerSend

DESCRIPTION:        Check timestamp and do frame dropping if enabled

ARGUMENTS PASSED:   pTimeStamp_Object  - TimeStamp Manager to handle related timestamp
					ptimestamp - returned timestamp to use at render

RETURN VALUE:       None
PRE-CONDITIONS:     None
POST-CONDITIONS:    None
IMPORTANT NOTES:    None
=======================================================================================*/
static TSM_TIMESTAMP _TSManagerSend2(void *handle, void *key, int send)
{
	TSManager *tsm = (TSManager *) handle;
	int i;
	int index = -1;
	int isValidTs = 0;
	TSM_TIMESTAMP ts0 = 0, tstmp = TSM_TIMESTAMP_NONE;
	unsigned long long age = 0;
	TSM_TIMESTAMP half_interval;

	if (tsm == NULL)
		return tstmp;

	i = tsm->tx;
	half_interval = TSM_ADAPTIVE_INTERVAL (tsm) >> 1;

	if (tsm) {
		if (send)
			tsm->tx_cnt++;
		else {
			tsm->cnt++;
			tsm->invalid_ts_count++;
		}
		if (tsm->cnt > 0) {
			if (send)
				tsm->cnt--;

			if (tsm->mode == MODE_AI) {
				if (tsm->first_tx == 0)
					tstmp = tsm->last_ts_sent + TSM_ADAPTIVE_INTERVAL(tsm);
				else
					tstmp = tsm->last_ts_sent;

				while (i != tsm->rx) {
					if (index >= 0) {
						if (tsm->ts_buf[i].ts < ts0) {
							ts0 = tsm->ts_buf[i].ts;
							age = tsm->ts_buf[i].age;
							index = i;
						}
					} else {
						ts0 = tsm->ts_buf[i].ts;
						age = tsm->ts_buf[i].age;
						index = i;
					}
					if ((TSM_KEY_IS_VALID(key)) && (key == tsm->ts_buf[i].key))
						break;
					i = ((i + 1) % tsm->ts_buf_size);
				}
				if (index >= 0) {
					if ((tsm->invalid_ts_count) && (ts0 >= ((tstmp) + half_interval))
							&& (age > tsm->age)) {
						/* use calculated ts0 */
						if (send)
							tsm->invalid_ts_count--;
					} else {
						if (send) {
							if (index != tsm->tx)
								tsm->ts_buf[index] = tsm->ts_buf[tsm->tx];

							tsm->tx = ((tsm->tx + 1) % tsm->ts_buf_size);
						}
#if 0
						if (ts0 >= ((tstmp) + half_interval))
							tstmp = tstmp;
						else
							tstmp = ts0;
#else
						tstmp = ts0;
#endif
						isValidTs = 1;
					}
				} else {
					if (send)
						tsm->invalid_ts_count--;
				}

				if (tsm->first_tx == 0) {
					if (tstmp > tsm->last_ts_sent)
						ts0 = (tstmp - tsm->last_ts_sent);
					else {
						ts0 = 0;
						//reset the timestamp to last frame only when new frames's timestamp is earlier than one frame.
						if (tstmp + TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2 < tsm->last_ts_sent)
							tstmp = tsm->last_ts_sent;
					}

					if (ts0 > TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2) {
						TSM_WARNING("Jitter1:%u:%02u:%02u.%09u %u:%02u:%02u.%09u\n",
								TSM_TIME_H(ts0),
								TSM_TIME_M(ts0),
								TSM_TIME_S(ts0),
								TSM_TIME_NS(ts0),
								TSM_TIME_H(TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2),
								TSM_TIME_M(TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2),
								TSM_TIME_S(TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2),
								TSM_TIME_NS(TSM_ADAPTIVE_INTERVAL(tsm) * 3 / 2));
					} else if (ts0 == 0)
						TSM_WARNING("Jitter:%u:%02u:%02u.%09u\n",
								TSM_TIME_H(ts0),
								TSM_TIME_M(ts0),
								TSM_TIME_S(ts0),
								TSM_TIME_NS(ts0));

					if (send) {
						if (isValidTs && ts0 > TSM_ADAPTIVE_INTERVAL(tsm) * 2)
							ts0 = TSM_ADAPTIVE_INTERVAL(tsm) * 2;
						if ((ts0 <= TSM_ADAPTIVE_INTERVAL(tsm) * 2) || (tsm->big_cnt > 3)) {
							tsm->big_cnt = 0;
							tsm->dur_history_total -=
								tsm->dur_history_buf[tsm->dur_history_tx];
							tsm->dur_history_buf[tsm->dur_history_tx] = ts0;
							tsm->dur_history_tx =
								((tsm->dur_history_tx + 1) % TSM_HISTORY_SIZE);
							tsm->dur_history_total += ts0;
						} else
							tsm->big_cnt++;
					}
				}
				if (send) {
					tsm->last_ts_sent = tstmp;
					tsm->age++;
					tsm->first_tx = 0;
				}
			} else if (tsm->mode == MODE_FIFO) {
				tstmp = tsm->ts_buf[tsm->tx].ts;
				if (send)
					tsm->tx = ((tsm->tx + 1) % tsm->ts_buf_size);

				ts0 = tstmp - tsm->last_ts_sent;
				if (send)
					tsm->last_ts_sent = tstmp;
			}

			if (send) {
				TSM_LOG("--Send %d:%u:%02u:%02u.%09u, int:%u:%02u:%02u.%09u, avg:%u:%02u:%02u.%09u inkey %p\n",
						tsm->tx_cnt,
						TSM_TIME_H(tstmp),
						TSM_TIME_M(tstmp),
						TSM_TIME_S(tstmp),
						TSM_TIME_NS(tstmp),
						TSM_TIME_H(ts0),
						TSM_TIME_M(ts0),
						TSM_TIME_S(ts0),
						TSM_TIME_NS(ts0),
						TSM_TIME_H(TSM_ADAPTIVE_INTERVAL(tsm)),
						TSM_TIME_M(TSM_ADAPTIVE_INTERVAL(tsm)),
						TSM_TIME_S(TSM_ADAPTIVE_INTERVAL(tsm)),
						TSM_TIME_NS(TSM_ADAPTIVE_INTERVAL(tsm)),
						key);
			}
		} else {
			if (tsm->valid_ts_received == 0) {
				if (tsm->first_tx)
					tstmp = tsm->last_ts_sent;
				else
					tstmp = tsm->last_ts_sent + TSM_ADAPTIVE_INTERVAL(tsm);

				if (send) {
					tsm->first_tx = 0;
					tsm->last_ts_sent = tstmp;
				}
			}
			TSM_ERROR("Too many timestamps send!!\n");
		}
		if (send == 0) {
			tsm->cnt--;
			tsm->invalid_ts_count--;
		}
	}

	return tstmp;
}


TSM_TIMESTAMP TSManagerSend2(void *handle, void *key)
{
	return _TSManagerSend2(handle, key, 1);
}


TSM_TIMESTAMP TSManagerQuery2(void *handle, void *key)
{
	return _TSManagerSend2(handle, key, 0);
}


TSM_TIMESTAMP TSManagerSend(void *handle)
{
	return TSManagerSend2(handle, TSM_KEY_NONE);
}


TSM_TIMESTAMP TSManagerQuery(void *handle)
{
	return TSManagerQuery2(handle, TSM_KEY_NONE);
}


void resyncTSManager(void *handle, TSM_TIMESTAMP synctime, TSMGR_MODE mode)
{
	TSManager *tsm = (TSManager *) handle;

	if (tsm) {
		TSMRecivedCtl *rctl = &tsm->rctl;
		TSMReceivedEntry *e = rctl->head;

		while ((e = rctl->head)) {
			rctl->head = e->next;
			tsm_free_received_entry(rctl, e);
		};
		rctl->cnt = 0;

		rctl->tail = NULL;

		tsm->first_tx = 1;
		tsm->first_rx = 1;
		tsm->suspicious_ts = 0;

		if (TSM_TS_IS_VALID(synctime))
			tsm->last_ts_sent = synctime;

		tsm->tx = tsm->rx = 0;
		tsm->invalid_ts_count = 0;
		tsm->mode = mode;
		tsm->age = 0;
		tsm->rx_cnt = tsm->tx_cnt = tsm->cnt = 0;
		tsm->valid_ts_received = 0;
		tsm->big_cnt = 0;
	}
}


/*======================================================================================
FUNCTION:           mfw_gst_init_ts

DESCRIPTION:        alloc and initialize timestamp strcture

ARGUMENTS PASSED:   ppTimeStamp_Object  - pointer of TimeStamp Manager to handle related timestamp

RETURN VALUE:       TimeStamp structure pointer
PRE-CONDITIONS:     None
POST-CONDITIONS:    None
IMPORTANT NOTES:    None
=======================================================================================*/
void *createTSManager(int ts_buf_size)
{
	TSManager *tsm = (TSManager *) kzalloc(sizeof(TSManager), GFP_KERNEL);

	if (tsm) {
		memset(tsm, 0, sizeof(TSManager));
		if (ts_buf_size <= 0)
			ts_buf_size = TSM_DEFAULT_TS_BUFFER_SIZE;

		tsm->ts_buf_size = ts_buf_size;
		tsm->ts_buf = kzalloc(sizeof(TSMControl) * ts_buf_size, GFP_KERNEL);

		if (tsm->ts_buf == NULL)
			goto fail;

		resyncTSManager(tsm, (TSM_TIMESTAMP) 0, MODE_AI);

		tsm->dur_history_tx = 0;
		TSM_BUFFER_SET(tsm->dur_history_buf, TSM_DEFAULT_INTERVAL,
				TSM_HISTORY_SIZE);
		tsm->dur_history_total = TSM_DEFAULT_INTERVAL << TSM_HISTORY_POWER;

		tsm->discont_threshold = 10000000000LL;     // 10s
	}
	return tsm;
fail:
	if (tsm) {
		if (tsm->ts_buf)
			kfree(tsm->ts_buf);
		kfree(tsm);
		tsm = NULL;
	}
	return tsm;
}


void destroyTSManager(void *handle)
{
	TSManager *tsm = (TSManager *) handle;

	if (tsm) {
		TSMRecivedCtl *rctl = &tsm->rctl;
		TSMReceivedEntryMemory *rmem;

		if (tsm->ts_buf)
			kfree(tsm->ts_buf);

		while ((rmem = rctl->memory)) {
			rctl->memory = rmem->next;
			kfree(rmem);
		}
		kfree(tsm);
		tsm = NULL;
	}
}


void setTSManagerFrameRate(void *handle, int fps_n, int fps_d)
//void setTSManagerFrameRate(void * handle, float framerate)
{
	TSManager *tsm = (TSManager *) handle;
	TSM_TIMESTAMP ts;

	if ((fps_n > 0) && (fps_d > 0) && (fps_n / fps_d <= 80))
		ts = TSM_SECOND * fps_d / fps_n;
	else
		ts = TSM_DEFAULT_INTERVAL;
	// TSM_TIMESTAMP ts = TSM_SECOND / framerate;

	if (tsm) {
		TSM_BUFFER_SET(tsm->dur_history_buf, ts, TSM_HISTORY_SIZE);
		tsm->dur_history_total = (ts << TSM_HISTORY_POWER);
		TSM_LOG("Set frame intrval:%u:%02u:%02u.%09u\n",
				TSM_TIME_H(ts),
				TSM_TIME_M(ts),
				TSM_TIME_S(ts),
				TSM_TIME_NS(ts));
	}
}

TSM_TIMESTAMP getTSManagerFrameInterval(void *handle)
{
	TSManager *tsm = (TSManager *) handle;
	TSM_TIMESTAMP ts = 0;

	if (tsm)
		ts = TSM_ADAPTIVE_INTERVAL(tsm);

	return ts;
}


TSM_TIMESTAMP getTSManagerPosition(void *handle)
{
	TSManager *tsm = (TSManager *) handle;
	TSM_TIMESTAMP ts = 0;

	if (tsm)
		ts = tsm->last_ts_sent;

	return ts;
}


int getTSManagerPreBufferCnt(void *handle)
{
	int i = 0;
	TSManager *tsm = (TSManager *) handle;

	if (tsm)
		i = tsm->rctl.cnt;

	return i;
}
