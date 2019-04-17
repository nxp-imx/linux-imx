/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * Module Name:    TimeStamp.h
 *
 * Description:    include TimeStamp stratege for VPU / SW video decoder plugin
 *
 * Portability:    This code is written for Linux OS and Gstreamer
 */

#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_


/**
 * GST_CLOCK_TIME_NONE:
 *
 * Constant to define an undefined clock time.
 */

typedef long long TSM_TIMESTAMP;

typedef enum {
	MODE_AI,
	MODE_FIFO,
} TSMGR_MODE;

#define TSM_TIMESTAMP_NONE ((long long)(-1))
#define TSM_KEY_NONE ((void *)0)

enum {
	DEBUG_LEVEL_ERROR = 1,
	DEBUG_LEVEL_WARNING,
	DEBUG_LEVEL_LOG,
	DEBUG_LEVEL_VERBOSE,
};

#define TSM_MESSAGE(level, fmt, ...) \
	do { \
		if (debug_level >= (level)) { \
			pr_info("TSM:"fmt, ##__VA_ARGS__); \
		} \
	} while (0)

#define TSM_ERROR(...) TSM_MESSAGE(DEBUG_LEVEL_ERROR, ##__VA_ARGS__)
#define TSM_WARNING(...) TSM_MESSAGE(DEBUG_LEVEL_WARNING, ##__VA_ARGS__)
#define TSM_LOG(...) TSM_MESSAGE(DEBUG_LEVEL_LOG, ##__VA_ARGS__)
#define TSM_VERBOSE(...) TSM_MESSAGE(DEBUG_LEVEL_VERBOSE, ##__VA_ARGS__)

#define TSM_HISTORY_POWER 5
#define TSM_HISTORY_SIZE (1 << TSM_HISTORY_POWER)
#define TSM_ADAPTIVE_INTERVAL(tsm) \
	(tsm->dur_history_total >> TSM_HISTORY_POWER)

#define TSM_SECOND ((TSM_TIMESTAMP)1000000000)
#define TSM_DEFAULT_INTERVAL (TSM_SECOND/30)
#define TSM_DEFAULT_TS_BUFFER_SIZE (128)

#define TSM_TS_IS_VALID(ts)	\
	((ts) != TSM_TIMESTAMP_NONE)

#define TSM_KEY_IS_VALID(key) \
	((key) != TSM_KEY_NONE)

#define TSM_DISTANCE(tsm)\
	(((tsm->rx) >= (tsm->tx))?((tsm->rx)-(tsm->tx)):(tsm->ts_buf_size-(tsm->tx)+(tsm->rx)))

#define TSM_PLUS_AGE(tsm)\
	(TSM_DISTANCE(tsm)+tsm->invalid_ts_count+2)

#define TSM_ABS(ts0, ts1)\
	(((ts0) > (ts1))?((ts0)-(ts1)):((ts1)-(ts0)))

#define TSM_TIME_H(t)	\
	(TSM_TS_IS_VALID(t) ? \
	 (unsigned int) (((TSM_TIMESTAMP)(t)) / (TSM_SECOND * 60 * 60)) : 99)
#define TSM_TIME_M(t)	\
	(TSM_TS_IS_VALID(t) ? \
	 (unsigned int) ((((TSM_TIMESTAMP)(t)) / (TSM_SECOND * 60)) % 60) : 99)
#define TSM_TIME_S(t)	\
	(TSM_TS_IS_VALID(t) ? \
	 (unsigned int) ((((TSM_TIMESTAMP)(t)) / TSM_SECOND) % 60) : 99)
#define TSM_TIME_NS(t) \
	(TSM_TS_IS_VALID(t) ? \
	 (unsigned int) (((TSM_TIMESTAMP)(t)) % TSM_SECOND) : 999999999)

#define TSM_BUFFER_SET(buf, value, size) \
	do { \
		int i; \
		for (i = 0; i < (size); i++) { \
			(buf)[i] = (value); \
		} \
	} while (0)

#define TSM_RECEIVED_NUNBER 512

typedef struct {
	TSM_TIMESTAMP ts;
	unsigned long long age;
	void *key;
} TSMControl;

typedef struct _TSMReceivedEntry {
	TSM_TIMESTAMP ts;
	struct _TSMReceivedEntry *next;
	unsigned int used:1;
	unsigned int subentry:1;
	int size;
} TSMReceivedEntry;

typedef struct _TSMReceivedEntryMemory {
	struct _TSMReceivedEntryMemory *next;
	TSMReceivedEntry entrys[TSM_RECEIVED_NUNBER];
} TSMReceivedEntryMemory;

typedef struct {
	TSMReceivedEntry *head;
	TSMReceivedEntry *tail;
	TSMReceivedEntry *free;
	TSMReceivedEntryMemory *memory;
	int cnt;
} TSMRecivedCtl;

typedef struct _TSManager {
	int first_tx;
	int first_rx;
	int rx;                       //timestamps received
	int tx;                       //timestamps transferred
	TSM_TIMESTAMP last_ts_sent;   //last time stamp sent
	TSM_TIMESTAMP last_ts_received;
	TSM_TIMESTAMP suspicious_ts;

	TSM_TIMESTAMP discont_threshold;

	unsigned int invalid_ts_count;
	TSMGR_MODE mode;
	int ts_buf_size;
	int dur_history_tx;
	TSM_TIMESTAMP dur_history_total;
	TSM_TIMESTAMP dur_history_buf[TSM_HISTORY_SIZE];
	TSMControl *ts_buf;
	unsigned long long age;
	int tx_cnt;
	int rx_cnt;
	int cnt;
	int valid_ts_received:1;
	int big_cnt;

	TSMRecivedCtl rctl;
} TSManager;

/**
 * GST_CLOCK_TIME_IS_VALID:
 * @time: clock time to validate
 *
 * Tests if a given #GstClockTime represents a valid defined time.
 */

/*!
 * This function receive timestamp into timestamp manager.
 *
 * @param	handle		handle of timestamp manager.
 *
 * @param	timestamp	timestamp received
 *
 * @return
 */
void TSManagerReceive(void *handle, TSM_TIMESTAMP timestamp);
void TSManagerReceive2(void *handle, TSM_TIMESTAMP timestamp, int size);
void TSManagerFlush2(void *handle, int size);
void TSManagerValid2(void *handle, int size, void *key);
/*!
 * This function send the timestamp for next output frame.
 *
 * @param	handle		handle of timestamp manager.
 *
 * @return	timestamp for next output frame.
 */
TSM_TIMESTAMP TSManagerSend(void *handle);
TSM_TIMESTAMP TSManagerSend2(void *handle, void *key);
TSM_TIMESTAMP TSManagerQuery2(void *handle, void *key);
TSM_TIMESTAMP TSManagerQuery(void *handle);
/*!
 * This function resync timestamp handler when reset and seek
 *
 * @param	handle		handle of timestamp manager.
 *
 * @param	synctime    the postion time needed to set, if value invalid, position keeps original
 *
 * @param	mode		playing mode (AI or FIFO)
 *
 * @return
 */
void resyncTSManager(void *handle, TSM_TIMESTAMP synctime, TSMGR_MODE mode);
/*!
 * This function create and reset timestamp handler
 *
 * @param	ts_buf_size	 time stamp queue buffer size
 *
 * @return
 */
void *createTSManager(int ts_buf_size);
/*!
 * This function destroy timestamp handler
 *
 * @param	handle		handle of timestamp manager.
 *
 * @return
 */
void destroyTSManager(void *handle);
/*!
 * This function set  history buffer frame interval by fps_n and fps_d
 *
 * @param	handle		handle of timestamp manager.
 *
 * @param	framerate       the framerate to be set
 *
 * @return
 */
void setTSManagerFrameRate(void *handle, int fps_n, int fps_d);
// void setTSManagerFrameRate(void * handle, float framerate);
/*!
 * This function set the current calculated Frame Interval
 *
 * @param	handle		handle of timestamp manager.
 *
 * @return
 */
TSM_TIMESTAMP getTSManagerFrameInterval(void *handle);
/*!
 * This function get  the current time stamp postion
 *
 * @param	handle		handle of timestamp manager.
 *
 * @return
 */
TSM_TIMESTAMP getTSManagerPosition(void *handle);
int getTSManagerPreBufferCnt(void *handle);


#endif /*_TIMESTAMP_H_ */
