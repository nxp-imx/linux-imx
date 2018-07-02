/*
 * Copyright (c) 2015-2018 The Linux Foundation. All rights reserved.
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

/**
 * wlan_hdd_tsf.c - WLAN Host Device Driver tsf related implementation
 */

#include "wlan_hdd_main.h"
#include "wlan_hdd_tsf.h"
#include "wma_api.h"
#include <linux/errqueue.h>

/**
 * enum hdd_tsf_op_result - result of tsf operation
 *
 * HDD_TSF_OP_SUCC:  succeed
 * HDD_TSF_OP_FAIL:  fail
 */
enum hdd_tsf_op_result {
	HDD_TSF_OP_SUCC,
	HDD_TSF_OP_FAIL
};

#define WLAN_HDD_CAPTURE_TSF_REQ_TIMEOUT_MS 500
static void hdd_capture_req_timer_expired_handler(void *arg);

#ifdef WLAN_FEATURE_TSF_PLUS
static inline void hdd_set_th_sync_status(hdd_adapter_t *adapter,
		bool initialized)
{
	adf_os_atomic_set(&adapter->tsf_sync_ready_flag,
			(initialized ? 1 : 0));
}

static inline bool hdd_get_th_sync_status(hdd_adapter_t *adapter)
{
	return (!adf_os_atomic_read(&adapter->tsf_sync_ready_flag) == 0);
}

#else
static inline bool hdd_get_th_sync_status(hdd_adapter_t *adapter)
{
	return true;
}
#endif

static
enum hdd_tsf_get_state hdd_tsf_check_conn_state(hdd_adapter_t *adapter)
{
	enum hdd_tsf_get_state ret = TSF_RETURN;
	hdd_station_ctx_t *hdd_sta_ctx;

	if (adapter->device_mode == WLAN_HDD_INFRA_STATION ||
			adapter->device_mode == WLAN_HDD_P2P_CLIENT) {
		hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);
		if (hdd_sta_ctx->conn_info.connState !=
				eConnectionState_Associated) {
			hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("failed to cap tsf, not connect with ap"));
			ret = TSF_STA_NOT_CONNECTED_NO_TSF;
		}
	} else if ((adapter->device_mode == WLAN_HDD_SOFTAP ||
				adapter->device_mode == WLAN_HDD_P2P_GO) &&
			!(test_bit(SOFTAP_BSS_STARTED,
					&adapter->event_flags))) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Soft AP / P2p GO not beaconing"));
		ret = TSF_SAP_NOT_STARTED_NO_TSF;
	}
	return ret;
}

static bool hdd_tsf_is_initialized(hdd_adapter_t *adapter)
{
	hdd_context_t *hddctx;

	if (!adapter) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("invalid adapter"));
		return false;
	}

	hddctx = WLAN_HDD_GET_CTX(adapter);
	if (!hddctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("invalid hdd context"));
		return false;
	}

	if (!adf_os_atomic_read(&hddctx->tsf_ready_flag) ||
		!hdd_get_th_sync_status(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("TSF is not initialized"));
		return false;
	}

	return true;
}

static inline int hdd_reset_tsf_gpio(hdd_adapter_t *adapter)
{
	return process_wma_set_command((int)adapter->sessionId,
			(int)GEN_PARAM_RESET_TSF_GPIO,
			adapter->sessionId,
			GEN_CMD);
}

static enum hdd_tsf_op_result hdd_capture_tsf_internal(
	hdd_adapter_t *adapter, uint32_t *buf, int len)
{
	int ret;
	hdd_context_t *hddctx;

	if (adapter == NULL || buf == NULL) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid pointer"));
		return HDD_TSF_OP_FAIL;
	}

	if (len != 1)
		return HDD_TSF_OP_FAIL;

	hddctx = WLAN_HDD_GET_CTX(adapter);
	if (!hddctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid hdd context"));
		return HDD_TSF_OP_FAIL;
	}

	if (!hdd_tsf_is_initialized(adapter)) {
		buf[0] = TSF_NOT_READY;
		return HDD_TSF_OP_SUCC;
	}

	buf[0] = hdd_tsf_check_conn_state(adapter);
	if (buf[0] != TSF_RETURN)
		return HDD_TSF_OP_SUCC;

	if (adf_os_atomic_inc_return(&hddctx->cap_tsf_flag) > 1) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("current in capture state"));
		buf[0] = TSF_CURRENT_IN_CAP_STATE;
		return HDD_TSF_OP_SUCC;
	}

	/* record adapter for cap_tsf_irq_handler  */
	hddctx->cap_tsf_context = adapter;

	hddLog(VOS_TRACE_LEVEL_INFO, FL("+ioctl issue cap tsf cmd"));

	/* Reset TSF value for new capture */
	adapter->cur_target_time = 0;

	buf[0] = TSF_RETURN;

	vos_timer_init(&adapter->host_capture_req_timer, VOS_TIMER_TYPE_SW,
		       hdd_capture_req_timer_expired_handler,
		       (void *)adapter);
	vos_timer_start(&adapter->host_capture_req_timer,
			WLAN_HDD_CAPTURE_TSF_REQ_TIMEOUT_MS);

	ret = process_wma_set_command((int)adapter->sessionId,
			(int)GEN_PARAM_CAPTURE_TSF,
			adapter->sessionId,
			GEN_CMD);
	if (0 != ret) {
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("cap tsf fail"));
		buf[0] = TSF_CAPTURE_FAIL;
		hddctx->cap_tsf_context = NULL;
		adf_os_atomic_set(&hddctx->cap_tsf_flag, 0);
		vos_timer_stop(&adapter->host_capture_req_timer);
		vos_timer_destroy(&adapter->host_capture_req_timer);
		return HDD_TSF_OP_SUCC;
	}
	hddLog(VOS_TRACE_LEVEL_INFO,
			FL("-ioctl return cap tsf cmd"));

	return HDD_TSF_OP_SUCC;
}

static enum hdd_tsf_op_result hdd_indicate_tsf_internal(
	hdd_adapter_t *adapter, uint32_t *buf, int len)
{
	int ret;
	hdd_context_t *hddctx;

	if (adapter == NULL || buf == NULL) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid pointer"));
		return HDD_TSF_OP_FAIL;
	}

	if (len != 3)
		return HDD_TSF_OP_FAIL;

	hddctx = WLAN_HDD_GET_CTX(adapter);
	if (!hddctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid hdd context"));
		return HDD_TSF_OP_FAIL;
	}

	buf[1] = 0;
	buf[2] = 0;

	if (!hdd_tsf_is_initialized(adapter)) {
		buf[0] = TSF_NOT_READY;
		return HDD_TSF_OP_SUCC;
	}

	buf[0] = hdd_tsf_check_conn_state(adapter);
	if (buf[0] != TSF_RETURN)
		return HDD_TSF_OP_SUCC;

	if (adapter->cur_target_time == 0) {
		hddLog(VOS_TRACE_LEVEL_INFO,
				FL("not getting tsf value"));
		buf[0] = TSF_NOT_RETURNED_BY_FW;
		return HDD_TSF_OP_SUCC;
	} else {
		buf[0] = TSF_RETURN;
		buf[1] = (uint32_t)(adapter->cur_target_time & 0xffffffff);
		buf[2] = (uint32_t)((adapter->cur_target_time >> 32) &
				0xffffffff);

		if (!adf_os_atomic_read(&hddctx->cap_tsf_flag)) {
			hddLog(VOS_TRACE_LEVEL_INFO,
				FL("old: status=%u, tsf_low=%u, tsf_high=%u"),
				buf[0], buf[1], buf[2]);
			return HDD_TSF_OP_SUCC;
		}

		ret = hdd_reset_tsf_gpio(adapter);
		if (0 != ret) {
			hddLog(VOS_TRACE_LEVEL_ERROR,
					FL("reset tsf gpio fail"));
			buf[0] = TSF_RESET_GPIO_FAIL;
			return HDD_TSF_OP_SUCC;
		}
		hddctx->cap_tsf_context = NULL;
		adf_os_atomic_set(&hddctx->cap_tsf_flag, 0);
		hddLog(VOS_TRACE_LEVEL_INFO,
			FL("get tsf cmd,status=%u, tsf_low=%u, tsf_high=%u"),
			buf[0], buf[1], buf[2]);
		return HDD_TSF_OP_SUCC;
	}
}

#ifdef WLAN_FEATURE_TSF_PLUS
/* unit for target time: us;  host time: ns */
#define HOST_TO_TARGET_TIME_RATIO NSEC_PER_USEC
#define MAX_ALLOWED_DEVIATION_NS (100 * NSEC_PER_USEC)
#define MAX_CONTINUOUS_ERROR_CNT 3

/**
 * to distinguish 32-bit overflow case, this inverval should:
 * equal or less than (1/2 * OVERFLOW_INDICATOR32 us)
 */
#define WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC 10
#define WLAN_HDD_CAPTURE_TSF_INIT_INTERVAL_MS 100
#define OVERFLOW_INDICATOR32 (((int64_t)0x1) << 32)
#define MAX_UINT64 ((uint64_t)0xffffffffffffffff)
#define MASK_UINT32 0xffffffff
#define CAP_TSF_TIMER_FIX_SEC 1
#define WLAN_HDD_CAPTURE_TSF_RESYNC_INTERVAL 9

/**
 * TS_STATUS - timestamp status
 *
 * HDD_TS_STATUS_WAITING:  one of the stamp-pair
 *    is not updated
 * HDD_TS_STATUS_READY:  valid tstamp-pair
 * HDD_TS_STATUS_INVALID: invalid tstamp-pair
 */
enum hdd_ts_status {
	HDD_TS_STATUS_WAITING,
	HDD_TS_STATUS_READY,
	HDD_TS_STATUS_INVALID
};

static
enum hdd_tsf_op_result __hdd_start_tsf_sync(hdd_adapter_t *adapter)
{
	VOS_STATUS ret;

	if (!hdd_get_th_sync_status(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Host Target sync has not initialized"));
		return HDD_TSF_OP_FAIL;
	}

	ret = vos_timer_start(&adapter->host_target_sync_timer,
			WLAN_HDD_CAPTURE_TSF_INIT_INTERVAL_MS);
	if (ret != VOS_STATUS_SUCCESS && ret != VOS_STATUS_E_ALREADY) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("Failed to start timer, ret: %d"), ret);
		return HDD_TSF_OP_FAIL;
	}
	return HDD_TSF_OP_SUCC;
}

static
enum hdd_tsf_op_result __hdd_stop_tsf_sync(hdd_adapter_t *adapter)
{
	VOS_STATUS ret;

	if (!hdd_get_th_sync_status(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Host Target sync has not initialized"));
		return HDD_TSF_OP_SUCC;
	}

	ret = vos_timer_stop(&adapter->host_target_sync_timer);
	if (ret != VOS_STATUS_SUCCESS) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("Failed to stop timer, ret: %d"), ret);
		return HDD_TSF_OP_FAIL;
	}
	return HDD_TSF_OP_SUCC;
}

static inline void hdd_reset_timestamps(hdd_adapter_t *adapter)
{
	spin_lock_bh(&adapter->host_target_sync_lock);
	adapter->cur_host_time = 0;
	adapter->cur_target_time = 0;
	adapter->last_host_time = 0;
	adapter->last_target_time = 0;
	spin_unlock_bh(&adapter->host_target_sync_lock);
}

/**
 * hdd_check_timestamp_status() - return the tstamp status
 *
 * @last_target_time: the last saved target time
 * @last_host_time: the last saved host time
 * @cur_target_time : new target time
 * @cur_host_time : new host time
 *
 * This function check the new timstamp-pair(cur_host_time/cur_target_time)
 *
 * Return:
 * HDD_TS_STATUS_WAITING: cur_host_time or cur_host_time is 0
 * HDD_TS_STATUS_READY: cur_target_time/cur_host_time is a valid pair,
 *    and can be saved
 * HDD_TS_STATUS_INVALID: cur_target_time/cur_host_time is a invalid pair,
 *    should be discard
 */
static
enum hdd_ts_status hdd_check_timestamp_status(
		uint64_t last_target_time,
		uint64_t last_host_time,
		uint64_t cur_target_time,
		uint64_t cur_host_time)
{
	uint64_t delta_ns, delta_target_time, delta_host_time;

	/* one or more are not updated, need to wait */
	if (cur_target_time == 0 || cur_host_time == 0)
		return HDD_TS_STATUS_WAITING;

	/* init value, it's the first time to update the pair */
	if (last_target_time == 0 && last_host_time == 0)
		return HDD_TS_STATUS_READY;

	/* the new values should be greater than the saved values */
	if ((cur_target_time <= last_target_time) ||
			(cur_host_time <= last_host_time)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Invalid timestamps!last_target_time: %llu;"
				"last_host_time: %llu; cur_target_time: %llu;"
				"cur_host_time: %llu"),
			last_target_time, last_host_time,
			cur_target_time, cur_host_time);
		return HDD_TS_STATUS_INVALID;
	}

	delta_target_time = (cur_target_time - last_target_time) *
		HOST_TO_TARGET_TIME_RATIO;
	delta_host_time = cur_host_time - last_host_time;

	/*
	 * DO NOT use abs64() , a big uint64 value might be turned to
	 * a small int64 value
	 */
	delta_ns = ((delta_target_time > delta_host_time) ?
			(delta_target_time - delta_host_time) :
			(delta_host_time - delta_target_time));

	/* the deviation should be smaller than a threshold */
	if (delta_ns > MAX_ALLOWED_DEVIATION_NS) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Invalid timestamps - delta: %llu ns"), delta_ns);
		return HDD_TS_STATUS_INVALID;
	}
	return HDD_TS_STATUS_READY;
}

static void hdd_update_timestamp(hdd_adapter_t *adapter,
		uint64_t target_time, uint64_t host_time)
{
	int interval = 0;
	enum hdd_ts_status sync_status;

	if (!adapter)
		return;

	/* host time is updated in IRQ context, it's always before target time,
	 * and so no need to update last_host_time at present;
	 * assume the interval of capturing TSF
	 * (WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC) is long enough, host and target
	 * time are updated in pairs, we can return here to avoid requiring
	 * spin lock, and to speed up the IRQ processing.
	 */
	if (host_time > 0) {
		adapter->cur_host_time = host_time;
		return;
	}

	spin_lock_bh(&adapter->host_target_sync_lock);
	if (target_time > 0)
		adapter->cur_target_time = target_time;

	sync_status = hdd_check_timestamp_status(adapter->last_target_time,
			adapter->last_host_time,
			adapter->cur_target_time,
			adapter->cur_host_time);
	switch (sync_status) {
	case HDD_TS_STATUS_INVALID:
		if (++adapter->continuous_error_count <
			MAX_CONTINUOUS_ERROR_CNT) {
			interval =
				WLAN_HDD_CAPTURE_TSF_INIT_INTERVAL_MS;
			adapter->cur_target_time = 0;
			adapter->cur_host_time = 0;
			break;
		}
		hddLog(VOS_TRACE_LEVEL_INFO,
			FL("Reach the max continuous error count"));
		/*
		 * fall through:
		 * If reach MAX_CONTINUOUS_ERROR_CNT, treat it as a
		 * valid pair
		 */
	case HDD_TS_STATUS_READY:
		adapter->last_target_time = adapter->cur_target_time;
		adapter->last_host_time = adapter->cur_host_time;
		adapter->cur_target_time = 0;
		adapter->cur_host_time = 0;
		hddLog(VOS_TRACE_LEVEL_INFO,
			FL("ts-pair updated: target: %llu; host: %llu"),
			adapter->last_target_time,
			adapter->last_host_time);

		/* TSF-HOST need to be updated in at most
		 * WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC, it couldn't be achieved
		 * if the timer interval is also
		 * WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC, due to processing or
		 * schedule delay. So deduct several seconds from
		 * WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC.
		 * Without this change, hdd_get_hosttime_from_targettime() will
		 * get wrong host time when it's longer than
		 * WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC from last
		 * TSF-HOST update. */
		interval = (WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC -
			    CAP_TSF_TIMER_FIX_SEC) * MSEC_PER_SEC;
		adapter->continuous_error_count = 0;
		break;
	case HDD_TS_STATUS_WAITING:
		interval = 0;
		break;
	}
	spin_unlock_bh(&adapter->host_target_sync_lock);

	if (interval > 0)
		vos_timer_start(&adapter->host_target_sync_timer, interval);
}

static inline bool hdd_tsf_is_in_cap(hdd_adapter_t *adapter)
{
	hdd_context_t *hddctx;

	hddctx = WLAN_HDD_GET_CTX(adapter);
	if (!hddctx)
		return false;

	return (adf_os_atomic_read(&hddctx->cap_tsf_flag) > 0);
}

/* define 64bit plus/minus to deal with overflow */
static inline int hdd_64bit_plus(uint64_t x, int64_t y, uint64_t *ret)
{
	if ((y < 0 && (-y) > x) ||
	    (y > 0 && (y > MAX_UINT64 - x))) {
		*ret = 0;
		return -EINVAL;
	}

	*ret = x + y;
	return 0;
}

static inline int hdd_uint64_plus(uint64_t x, uint64_t y, uint64_t *ret)
{
	if (!ret)
		return -EINVAL;

	if (x > (MAX_UINT64 - y)) {
		*ret = 0;
		return -EINVAL;
	}

	*ret = x + y;
	return 0;
}

static inline int hdd_uint64_minus(uint64_t x, uint64_t y, uint64_t *ret)
{
	if (!ret)
		return -EINVAL;

	if (x < y) {
		*ret = 0;
		return -EINVAL;
	}

	*ret = x - y;
	return 0;
}

static inline int32_t hdd_get_hosttime_from_targettime(
	hdd_adapter_t *adapter, uint64_t target_time,
	uint64_t *host_time)
{
	int32_t ret = -EINVAL;
	int64_t delta32_target;
	bool in_cap_state;
	int64_t normal_interval_target_value;

	in_cap_state = hdd_tsf_is_in_cap(adapter);

	/*
	 * To avoid check the lock when it's not capturing tsf
	 * (the tstamp-pair won't be changed)
	 */
	if (in_cap_state)
		spin_lock_bh(&adapter->host_target_sync_lock);

	/* at present, target_time is only 32bit in fact */
	delta32_target = (int64_t)((target_time & MASK_UINT32) -
			(adapter->last_target_time & MASK_UINT32));

	normal_interval_target_value =
		 (int64_t)WLAN_HDD_CAPTURE_TSF_INTERVAL_SEC  * NSEC_PER_SEC;
	do_div(normal_interval_target_value, HOST_TO_TARGET_TIME_RATIO);

	if (delta32_target <
			(normal_interval_target_value - OVERFLOW_INDICATOR32))
		delta32_target += OVERFLOW_INDICATOR32;
	else if (delta32_target >
			(OVERFLOW_INDICATOR32 - normal_interval_target_value))
		delta32_target -= OVERFLOW_INDICATOR32;

	ret = hdd_64bit_plus(adapter->last_host_time,
			     HOST_TO_TARGET_TIME_RATIO * delta32_target,
			     host_time);

	if (in_cap_state)
		spin_unlock_bh(&adapter->host_target_sync_lock);

	return ret;
}

static inline int32_t hdd_get_targettime_from_hosttime(
	hdd_adapter_t *adapter, uint64_t host_time,
	uint64_t *target_time)
{
	int32_t ret = -EINVAL;
	bool in_cap_state;

	if (!adapter || host_time == 0)
		return ret;

	in_cap_state = hdd_tsf_is_in_cap(adapter);
	if (in_cap_state)
		spin_lock_bh(&adapter->host_target_sync_lock);

	if (host_time < adapter->last_host_time)
		ret = hdd_uint64_minus(adapter->last_target_time,
				       vos_do_div(adapter->last_host_time -
						  host_time,
						  HOST_TO_TARGET_TIME_RATIO),
				       target_time);
	else
		ret = hdd_uint64_plus(adapter->last_target_time,
				      vos_do_div(host_time -
						 adapter->last_host_time,
						 HOST_TO_TARGET_TIME_RATIO),
				      target_time);

	if (in_cap_state)
		spin_unlock_bh(&adapter->host_target_sync_lock);

	return ret;
}

static inline uint64_t hdd_get_monotonic_host_time(hdd_context_t *hdd_ctx)
{
	return (HDD_TSF_IS_RAW_SET(hdd_ctx) ?
		ktime_get_ns() : ktime_get_real_ns());
}

static ssize_t __hdd_wlan_tsf_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	hdd_station_ctx_t *hdd_sta_ctx;
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	ssize_t size;
	uint64_t host_time, target_time;

	struct net_device *net_dev = container_of(dev, struct net_device, dev);

	adapter = (hdd_adapter_t *)(netdev_priv(net_dev));
	if (adapter->magic != WLAN_HDD_ADAPTER_MAGIC)
		return scnprintf(buf, PAGE_SIZE, "Invalid device\n");

	if (!hdd_get_th_sync_status(adapter))
		return scnprintf(buf, PAGE_SIZE,
				 "TSF sync is not initialized\n");

	hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);
	if (eConnectionState_Associated != hdd_sta_ctx->conn_info.connState)
		return scnprintf(buf, PAGE_SIZE, "NOT connected\n");

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx)
		return scnprintf(buf, PAGE_SIZE, "Invalid HDD context\n");

	host_time = hdd_get_monotonic_host_time(hdd_ctx);
	if (hdd_get_targettime_from_hosttime(adapter, host_time,
					     &target_time))
		size = scnprintf(buf, PAGE_SIZE, "Invalid timestamp\n");
	else
		size = scnprintf(buf, PAGE_SIZE, "%s%llu %llu %pM\n",
				 buf, target_time, host_time,
				 hdd_sta_ctx->conn_info.bssId);
	return size;
}

static ssize_t hdd_wlan_tsf_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	vos_ssr_protect(__func__);
	ret = __hdd_wlan_tsf_show(dev, attr, buf);
	vos_ssr_unprotect(__func__);

	return ret;
}

static DEVICE_ATTR(tsf, 0400, hdd_wlan_tsf_show, NULL);

static void hdd_capture_tsf_timer_expired_handler(void *arg)
{
	uint32_t tsf_op_resp;
	hdd_adapter_t *adapter;

	if (!arg)
		return;

	adapter = (hdd_adapter_t *)arg;
	hdd_capture_tsf_internal(adapter, &tsf_op_resp, 1);
}

static irqreturn_t hdd_tsf_captured_irq_handler(int irq, void *arg)
{
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	uint64_t host_time;
	char *name = NULL;

	if (!arg)
		return IRQ_NONE;

	hdd_ctx = (hdd_context_t *)arg;
	host_time = hdd_get_monotonic_host_time(hdd_ctx);

	adapter = hdd_ctx->cap_tsf_context;
	if (!adapter)
		return IRQ_HANDLED;

	if (!hdd_tsf_is_initialized(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("tsf is not init, ignore irq"));
		return IRQ_HANDLED;
	}

	hdd_update_timestamp(adapter, 0, host_time);
	if (adapter->dev)
		name = adapter->dev->name;

	hddLog(VOS_TRACE_LEVEL_INFO,
	       FL("irq: %d - iface: %s - host_time: %llu"),
	       irq, (!name ? "none" : name), host_time);

	return IRQ_HANDLED;
}

static void hdd_capture_req_timer_expired_handler(void *arg)
{
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	VOS_TIMER_STATE capture_req_timer_status;
	int interval;
	int ret;

	if (!arg)
		return;
	adapter = (hdd_adapter_t *)arg;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid hdd context"));
		return;
	}

	if (!hdd_tsf_is_initialized(adapter)) {
		vos_timer_destroy(&adapter->host_capture_req_timer);
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("tsf not init"));
		return;
	}

	spin_lock_bh(&adapter->host_target_sync_lock);
	adapter->cur_host_time = 0;
	adapter->cur_target_time = 0;
	spin_unlock_bh(&adapter->host_target_sync_lock);

	ret = hdd_reset_tsf_gpio(adapter);
	if (0 != ret)
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("reset tsf gpio fail"));
	hdd_ctx->cap_tsf_context = NULL;
	adf_os_atomic_set(&hdd_ctx->cap_tsf_flag, 0);
	vos_timer_destroy(&adapter->host_capture_req_timer);

	capture_req_timer_status =
		vos_timer_getCurrentState(&adapter->host_target_sync_timer);
	if (capture_req_timer_status == VOS_TIMER_STATE_UNUSED)
	{
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("invalid timer status"));
		return;
	}
	interval = WLAN_HDD_CAPTURE_TSF_RESYNC_INTERVAL * MSEC_PER_SEC;
	vos_timer_start(&adapter->host_target_sync_timer, interval);
}

static enum hdd_tsf_op_result hdd_tsf_sync_init(hdd_adapter_t *adapter)
{
	VOS_STATUS ret;
	hdd_context_t *hddctx;
	struct net_device *net_dev;

	if (!adapter)
		return HDD_TSF_OP_FAIL;

	hddctx = WLAN_HDD_GET_CTX(adapter);
	if (!hddctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("invalid hdd context"));
		return HDD_TSF_OP_FAIL;
	}

	if (!adf_os_atomic_read(&hddctx->tsf_ready_flag)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("TSF feature has NOT been initialized"));
		return HDD_TSF_OP_FAIL;
	}

	if (hdd_get_th_sync_status(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("Host Target sync has been initialized!!"));
		return HDD_TSF_OP_SUCC;
	}

	spin_lock_init(&adapter->host_target_sync_lock);

	hdd_reset_timestamps(adapter);

	ret = vos_timer_init(&adapter->host_target_sync_timer,
			     VOS_TIMER_TYPE_SW,
			     hdd_capture_tsf_timer_expired_handler,
			     (void *)adapter);
	if (ret != VOS_STATUS_SUCCESS) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
		       FL("Failed to init timer, ret: %d"), ret);
		goto fail;
	}

	net_dev = adapter->dev;
	if (net_dev && HDD_TSF_IS_DBG_FS_SET(hddctx))
		device_create_file(&net_dev->dev, &dev_attr_tsf);
	hdd_set_th_sync_status(adapter, true);

	return HDD_TSF_OP_SUCC;
fail:
	hdd_set_th_sync_status(adapter, false);
	return HDD_TSF_OP_FAIL;
}

static enum hdd_tsf_op_result hdd_tsf_sync_deinit(hdd_adapter_t *adapter)
{
	VOS_STATUS ret;
	hdd_context_t *hddctx;
	struct net_device *net_dev;

	if (!adapter)
		return HDD_TSF_OP_FAIL;

	if (!hdd_get_th_sync_status(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Host Target sync has not been initialized!!"));
		return HDD_TSF_OP_SUCC;
	}

	hdd_set_th_sync_status(adapter, false);

	ret = vos_timer_destroy(&adapter->host_target_sync_timer);
	if (ret != VOS_STATUS_SUCCESS)
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Failed to destroy timer, ret: %d"), ret);

	hddctx = WLAN_HDD_GET_CTX(adapter);

	/* reset the cap_tsf flag and gpio if needed */
	if (hddctx && adf_os_atomic_read(&hddctx->cap_tsf_flag) &&
			hddctx->cap_tsf_context == adapter) {
		int reset_ret = hdd_reset_tsf_gpio(adapter);

		if (reset_ret)
			hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("Failed to reset tsf gpio, ret:%d"),
				reset_ret);
		hddctx->cap_tsf_context = NULL;
		adf_os_atomic_set(&hddctx->cap_tsf_flag, 0);
	}

	hdd_reset_timestamps(adapter);

	net_dev = adapter->dev;
	if (net_dev && HDD_TSF_IS_DBG_FS_SET(hddctx)) {
		struct device *dev = &net_dev->dev;

		device_remove_file(dev, &dev_attr_tsf);
	}
	return HDD_TSF_OP_SUCC;
}

static inline void hdd_update_tsf(hdd_adapter_t *adapter, uint64_t tsf)
{
	uint32_t tsf_op_resp[3];

	hdd_indicate_tsf_internal(adapter, tsf_op_resp, 3);
	hdd_update_timestamp(adapter, tsf, 0);
}

static inline
enum hdd_tsf_op_result hdd_netbuf_timestamp(adf_nbuf_t netbuf,
					    uint64_t target_time)
{
	hdd_adapter_t *adapter;
	struct net_device *net_dev = netbuf->dev;

	if (!net_dev)
		return HDD_TSF_OP_FAIL;

	adapter = (hdd_adapter_t *)(netdev_priv(net_dev));
	if (adapter && adapter->magic == WLAN_HDD_ADAPTER_MAGIC &&
	    hdd_get_th_sync_status(adapter)) {
		uint64_t host_time;
		int32_t ret = hdd_get_hosttime_from_targettime(adapter,
				target_time, &host_time);
		if (!ret) {
			netbuf->tstamp = ns_to_ktime(host_time);
			return HDD_TSF_OP_SUCC;
		}
	}

	return HDD_TSF_OP_FAIL;
}

int hdd_start_tsf_sync(hdd_adapter_t *adapter)
{
	enum hdd_tsf_op_result ret;

	if (!adapter)
		return -EINVAL;

	ret = hdd_tsf_sync_init(adapter);
	if (ret != HDD_TSF_OP_SUCC) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Failed to init tsf sync, ret: %d"), ret);
		return -EINVAL;
	}

	return (__hdd_start_tsf_sync(adapter) ==
		HDD_TSF_OP_SUCC ? 0 : -EINVAL);
}

int hdd_stop_tsf_sync(hdd_adapter_t *adapter)
{
	enum hdd_tsf_op_result ret;

	if (!adapter)
		return -EINVAL;

	ret = __hdd_stop_tsf_sync(adapter);
	if (ret != HDD_TSF_OP_SUCC)
		return -EINVAL;

	ret = hdd_tsf_sync_deinit(adapter);
	if (ret != HDD_TSF_OP_SUCC) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Failed to deinit tsf sync, ret: %d"), ret);
		return -EINVAL;
	}
	return 0;
}

int hdd_tx_timestamp(adf_nbuf_t netbuf, uint64_t target_time)
{
	struct sock *sk = NULL;

	if (netbuf->sk != NULL)
		sk = netbuf->sk;
	else
		memcpy((void *)(&sk), (void *)(&netbuf->tstamp.tv64),
		       sizeof(sk));

	if (!sk)
		return -EINVAL;

	if ((skb_shinfo(netbuf)->tx_flags & SKBTX_SW_TSTAMP) &&
	    !(skb_shinfo(netbuf)->tx_flags & SKBTX_IN_PROGRESS)) {
		struct sock_exterr_skb *serr;
		adf_nbuf_t new_netbuf;
		int err;

		if (hdd_netbuf_timestamp(netbuf, target_time) !=
		    HDD_TSF_OP_SUCC)
			return -EINVAL;

		new_netbuf = adf_nbuf_clone(netbuf);
		if (!new_netbuf)
			return -ENOMEM;

		serr = SKB_EXT_ERR(new_netbuf);
		memset(serr, 0, sizeof(*serr));
		serr->ee.ee_errno = ENOMSG;
		serr->ee.ee_origin = SO_EE_ORIGIN_TIMESTAMPING;

		err = sock_queue_err_skb(sk, new_netbuf);
		if (err) {
			adf_nbuf_free(new_netbuf);
			return err;
		}

		return 0;
	}
	return -EINVAL;
}

int hdd_rx_timestamp(adf_nbuf_t netbuf, uint64_t target_time)
{
	if (hdd_netbuf_timestamp(netbuf, target_time) ==
		HDD_TSF_OP_SUCC)
		return 0;

	/* reset tstamp when failed */
	netbuf->tstamp = ns_to_ktime(0);
	return -EINVAL;
}

static inline int __hdd_capture_tsf(hdd_adapter_t *adapter,
		uint32_t *buf, int len)
{
	if (!adapter || !buf) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid pointer"));
		return -EINVAL;
	}

	if (len != 1)
		return -EINVAL;

	buf[0] = TSF_DISABLED_BY_TSFPLUS;

	return 0;
}

static inline int __hdd_indicate_tsf(hdd_adapter_t *adapter,
		uint32_t *buf, int len)
{
	if (!adapter || !buf) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
				FL("invalid pointer"));
		return -EINVAL;
	}

	if (len != 3)
		return -EINVAL;

	buf[0] = TSF_DISABLED_BY_TSFPLUS;
	buf[1] = 0;
	buf[2] = 0;

	return 0;
}

static inline
enum hdd_tsf_op_result wlan_hdd_tsf_plus_init(hdd_context_t *hdd_ctx)
{
	int ret;

	if (!HDD_TSF_IS_PTP_ENABLED(hdd_ctx)) {
		hddLog(VOS_TRACE_LEVEL_INFO,
		       FL("To enable TSF_PLUS, set gtsf_ptp_options in ini"));
		return HDD_TSF_OP_FAIL;
	}

	ret = cnss_common_register_tsf_captured_handler(
			hdd_ctx->parent_dev,
			hdd_tsf_captured_irq_handler,
			(void *)hdd_ctx);
	if (ret != 0) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Failed to register irq handler: %d"), ret);
		return HDD_TSF_OP_FAIL;
	}

	if (HDD_TSF_IS_TX_SET(hdd_ctx))
		ol_register_timestamp_callback(hdd_tx_timestamp);

	return HDD_TSF_OP_SUCC;
}

static inline
enum hdd_tsf_op_result wlan_hdd_tsf_plus_deinit(hdd_context_t *hdd_ctx)
{
	int ret;

	if (!HDD_TSF_IS_PTP_ENABLED(hdd_ctx))
		return HDD_TSF_OP_SUCC;

	if (HDD_TSF_IS_TX_SET(hdd_ctx))
		ol_deregister_timestamp_callback();

	ret = cnss_common_unregister_tsf_captured_handler(
				hdd_ctx->parent_dev,
				(void *)hdd_ctx);
	if (ret != 0) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("Failed to unregister irq handler, ret:%d"),
			ret);
		ret = HDD_TSF_OP_FAIL;
	}

	return HDD_TSF_OP_SUCC;
}

void hdd_tsf_notify_wlan_state_change(hdd_adapter_t *adapter,
	eConnectionState old_state,
	eConnectionState new_state)
{
	if (!adapter)
		return;

	if (old_state != eConnectionState_Associated &&
		new_state == eConnectionState_Associated)
		hdd_start_tsf_sync(adapter);
	else if (old_state == eConnectionState_Associated &&
		new_state != eConnectionState_Associated)
		hdd_stop_tsf_sync(adapter);
}

void
hdd_tsf_record_sk_for_skb(hdd_context_t *hdd_ctx, adf_nbuf_t nbuf)
{
	/* if TSF_TX is enabled, skb->sk is required when timestamping
	 * the tx skb; at the same time, skb->tstamp is useless at this
	 * point in such case, so record sk in tstamp, in case it will
	 * be set to NULL in skb_orphan().
	 */
	if (HDD_TSF_IS_TX_SET(hdd_ctx))
		memcpy((void *)(&nbuf->tstamp.tv64), (void *)(&nbuf->sk),
		       sizeof(nbuf->sk));
}
#else
static inline void hdd_update_tsf(hdd_adapter_t *adapter, uint64_t tsf)
{
}

static inline int __hdd_indicate_tsf(hdd_adapter_t *adapter,
		uint32_t *buf, int len)
{
	return (hdd_indicate_tsf_internal(adapter, buf, len) ==
		HDD_TSF_OP_SUCC ? 0 : -EINVAL);
}

static inline int __hdd_capture_tsf(hdd_adapter_t *adapter,
		uint32_t *buf, int len)
{
	return (hdd_capture_tsf_internal(adapter, buf, len) ==
		HDD_TSF_OP_SUCC ? 0 : -EINVAL);
}

static void hdd_capture_req_timer_expired_handler(void *arg)
{
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	int ret;

	if (!arg)
		return;
	adapter = (hdd_adapter_t *)arg;

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("invalid hdd context"));
		return;
	}
	hddLog(VOS_TRACE_LEVEL_WARN, FL("tsf ind timeout"));
	adapter->cur_target_time = 0;
	ret = hdd_reset_tsf_gpio(adapter);
	if (0 != ret)
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("reset tsf gpio fail"));
	hdd_ctx->cap_tsf_context = NULL;
	adf_os_atomic_set(&hdd_ctx->cap_tsf_flag, 0);
	vos_timer_destroy(&adapter->host_capture_req_timer);
}

static inline
enum hdd_tsf_op_result wlan_hdd_tsf_plus_init(hdd_context_t *hdd_ctx)
{
	return HDD_TSF_OP_SUCC;
}

static inline
enum hdd_tsf_op_result wlan_hdd_tsf_plus_deinit(hdd_context_t *hdd_ctx)
{
	return HDD_TSF_OP_SUCC;
}
#endif /* WLAN_FEATURE_TSF_PLUS */

int hdd_capture_tsf(hdd_adapter_t *adapter, uint32_t *buf, int len)
{
	return __hdd_capture_tsf(adapter, buf, len);
}

int hdd_indicate_tsf(hdd_adapter_t *adapter, uint32_t *buf, int len)
{
	return __hdd_indicate_tsf(adapter, buf, len);
}

/**
 * hdd_get_tsf_cb() - handle tsf callback
 *
 * @pcb_cxt: pointer to the hdd_contex
 * @ptsf: pointer to struct stsf
 *
 * This function handle the event that reported by firmware at first.
 * The event contains the vdev_id, current tsf value of this vdev,
 * tsf value is 64bits, discripted in two varaible tsf_low and tsf_high.
 * These two values each is uint32.
 *
 * Return: Describe the execute result of this routine
 */
static int hdd_get_tsf_cb(void *pcb_cxt, struct stsf *ptsf)
{
	hdd_context_t *hddctx;
	hdd_adapter_t *adapter;
	int status;
	VOS_TIMER_STATE capture_req_timer_status;

	if (pcb_cxt == NULL || ptsf == NULL) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("HDD context is not valid"));
			return -EINVAL;
	}

	hddctx = (hdd_context_t *)pcb_cxt;
	status = wlan_hdd_validate_context(hddctx);
	if (0 != status)
		return -EINVAL;

	adapter = hdd_get_adapter_by_vdev(hddctx, ptsf->vdev_id);

	if (NULL == adapter) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("failed to find adapter"));
		return -EINVAL;
	}

	if (!hdd_tsf_is_initialized(adapter)) {
		hddLog(VOS_TRACE_LEVEL_ERROR,
			FL("tsf is not init, ignore tsf event"));
		return -EINVAL;
	}

	capture_req_timer_status =
		vos_timer_getCurrentState(&adapter->host_capture_req_timer);
	if (capture_req_timer_status == VOS_TIMER_STATE_UNUSED)
	{
		hddLog(VOS_TRACE_LEVEL_ERROR, FL("invalid timer status"));
		return -EINVAL;
	}
	vos_timer_stop(&adapter->host_capture_req_timer);
	status = vos_timer_destroy(&adapter->host_capture_req_timer);
	if (status != VOS_STATUS_SUCCESS)
		hddLog(VOS_TRACE_LEVEL_WARN,
		       FL("destroy cap req timer fail, ret: %d"),
		       status);

	hddLog(VOS_TRACE_LEVEL_INFO,
		FL("tsf cb handle event, device_mode is %d"),
		adapter->device_mode);

	adapter->cur_target_time = ((uint64_t)ptsf->tsf_high << 32 |
			 ptsf->tsf_low);

	hdd_update_tsf(adapter, adapter->cur_target_time);

	hddLog(VOS_TRACE_LEVEL_INFO,
		FL("hdd_get_tsf_cb sta=%u, tsf_low=%u, tsf_high=%u"),
		ptsf->vdev_id, ptsf->tsf_low, ptsf->tsf_high);
	return 0;
}

void wlan_hdd_tsf_init(hdd_context_t *hdd_ctx)
{
	eHalStatus hal_status;

	if (!hdd_ctx)
		return;

	if (adf_os_atomic_inc_return(&hdd_ctx->tsf_ready_flag) > 1)
		return;

	adf_os_atomic_init(&hdd_ctx->cap_tsf_flag);

	if (hdd_ctx->cfg_ini->tsf_gpio_pin == TSF_GPIO_PIN_INVALID)
		goto fail;

	hal_status = sme_set_tsf_gpio(hdd_ctx->hHal,
			hdd_ctx->cfg_ini->tsf_gpio_pin);
	if (eHAL_STATUS_SUCCESS != hal_status) {
		hddLog(LOGE, FL("set tsf GPIO failed, status: %d"),
				hal_status);
		goto fail;
	}

	hal_status = sme_set_tsfcb(hdd_ctx->hHal, hdd_get_tsf_cb, hdd_ctx);
	if (eHAL_STATUS_SUCCESS != hal_status) {
		hddLog(LOGE, FL("set tsf cb failed, status: %d"),
				hal_status);
		goto fail;
	}

	if (wlan_hdd_tsf_plus_init(hdd_ctx) != HDD_TSF_OP_SUCC)
		goto fail;

	return;

fail:
	adf_os_atomic_set(&hdd_ctx->tsf_ready_flag, 0);
	return;
}

void wlan_hdd_tsf_deinit(hdd_context_t *hdd_ctx)
{
	eHalStatus hal_status;

	if (!hdd_ctx)
		return;

	if (!adf_os_atomic_read(&hdd_ctx->tsf_ready_flag))
		return;

	hal_status = sme_set_tsfcb(hdd_ctx->hHal, NULL, NULL);
	if (eHAL_STATUS_SUCCESS != hal_status) {
		hddLog(LOGE, FL("reset tsf cb failed, status: %d"),
				hal_status);
	}

	wlan_hdd_tsf_plus_deinit(hdd_ctx);
	adf_os_atomic_set(&hdd_ctx->tsf_ready_flag, 0);
	adf_os_atomic_set(&hdd_ctx->cap_tsf_flag, 0);
}
