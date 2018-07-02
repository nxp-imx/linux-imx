/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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

#ifdef WLAN_OPEN_SOURCE
#include <wlan_hdd_includes.h>
#include <vos_sched.h>
#include "wlan_hdd_debugfs.h"
#include "wlan_hdd_debugfs_ocb.h"
#include "ol_tx.h"
#include "wma_api.h"
#include "wlan_hdd_ocb.h"
#include "adf_os_time.h"

#define WLAN_DSRC_COMMAND_MAX_SIZE 8
#define WLAN_DSRC_CHAN_STATS_ENABLE 1
#define WLAN_DSRC_CHAN_STATS_REQUEST 2

/**
 * __wlan_hdd_write_dsrc_chan_stats_debugfs() - config DSRC channel stats by
 * DSRC app
 * @file: file pointer
 * @buf: buffer
 * @count: count
 * @pos: position pointer
 *
 * Only after DSRC channel stats enabled, FW collects channel stats.
 *
 * <cmd_id> <cmd_param>
 * cmd_id=1: config channel stats parameters
 * cmd_param=0: disable DSRC channel stats
 * cmd_param=1: enable DSRC channel stats
 *
 * Before reading the channel stats, MUST specify which channel to request.
 *
 * <cmd_id> <cmd_param> [chan_freq]. i.e.: "2 1 5860", "2 2".
 * cmd_id=2 : request channel stats parameters
 * cmd_param=1 : request one channel stats, need channel frequency provided.
 * chan_freq=5860: channel frequency
 * cmd_param=2 : request all configured channel stats.
 *
 * Return: @count on success, error number otherwise
 */
static int __wlan_hdd_write_dsrc_chan_stats_debugfs(struct file *file,
						    const char __user *buf,
						    size_t count, loff_t *pos)
{
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	char cmd[WLAN_DSRC_COMMAND_MAX_SIZE + 1];
	char *sptr, *token;
	uint8_t cmd_idx = 0;
	int ret;

	ENTER();

	adapter = (hdd_adapter_t *)file->private_data;
	if ((NULL == adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic)) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Invalid adapter or adapter has invalid magic.",
			  __func__);
		return -EINVAL;
	}

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	ret = wlan_hdd_validate_context(hdd_ctx);
	if (ret)
		return ret;

	if (count > WLAN_DSRC_COMMAND_MAX_SIZE) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Command length is larger that %d bytes,",
			  __func__, WLAN_DSRC_COMMAND_MAX_SIZE);
		return -EINVAL;
	}

	if (copy_from_user(cmd, buf, count))
		return -EINVAL;
	cmd[count] = '\0';
	sptr = cmd;

	token = strsep(&sptr, " ");
	if (!token)
		return -EINVAL;
	if (kstrtou8(token, 0, &cmd_idx))
		return -EINVAL;
	switch (cmd_idx) {
	case WLAN_DSRC_CHAN_STATS_ENABLE:
	{
		bool enable;
		uint32_t value;

		token = strsep(&sptr, " ");
		if (!token)
			return -EINVAL;
		if (kstrtou32(token, 0, &value))
			return -EINVAL;
		enable = !!value;
		if (wlan_hdd_dsrc_config_radio_chan_stats(adapter, enable)) {
			VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
				  "%s: Config DSRC channel stats %d failure.",
				  __func__, enable);
			return -EINVAL;
		}
		break;
	}
	case WLAN_DSRC_CHAN_STATS_REQUEST:
	{
		uint32_t i, req_type = 0, chan_freq = 0;
		struct dsrc_radio_chan_stats_ctxt *ctx;
		struct radio_chan_stats_req *req = NULL;

		ctx = &adapter->dsrc_chan_stats;
		if (!ctx->enable_chan_stats) {
			VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
				"DSRC radio channel stats not enabled.");
			return -EINVAL;
		}

		token = strsep(&sptr, " ");
		if (!token)
			return -EINVAL;
		if (kstrtou32(token, 0, &req_type))
			return -EINVAL;
		if (req_type == WLAN_DSRC_REQUEST_ONE_RADIO_CHAN_STATS) {
			token = strsep(&sptr, " ");
			if (!token)
				return -EINVAL;
			if (kstrtou32(token, 0, &chan_freq))
				return -EINVAL;
			for (i = 0; i < ctx->config_chans_num; i++) {
				if (chan_freq == ctx->config_chans_freq[i])
					break;
			}
			if (i == ctx->config_chans_num) {
				VOS_TRACE(VOS_MODULE_ID_HDD,
					VOS_TRACE_LEVEL_ERROR,
					"%s: No channel freq %d found.",
					__func__, chan_freq);
				return -EINVAL;
			}
		} else if (req_type != WLAN_DSRC_REQUEST_ALL_RADIO_CHAN_STATS) {
			VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
				  "%s: Invalid dsrc chan stats req type %d.",
				  __func__, req_type);
			return -EINVAL;
		}
		req = vos_mem_malloc(sizeof(*req));
		if (!req) {
			VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
				"No memory for dsrc chan stats request.");
			return -ENOMEM;
		}
		vos_mem_zero(req, sizeof(*req));
		req->req_type = req_type;
		req->reset_after_req = true;
		if (req_type == WLAN_DSRC_REQUEST_ONE_RADIO_CHAN_STATS)
			req->chan_freq = chan_freq;
		if (ctx->cur_req)
			vos_mem_free(ctx->cur_req);
		ctx->cur_req = req;
		break;
	}
	default:
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Invalid dsrc chan stats cmd %d\n",
			  __func__, cmd_idx);
		return -EINVAL;
	}

	EXIT();
	return count;
}

/**
 * __wlan_hdd_read_dsrc_chan_stats_debugfs() - read DSRC channel stats by
 * DSRC app
 * @file: file pointer
 * @buf: buffer
 * @count: count
 * @pos: position pointer
 *
 * Return: bytes read on success, error number otherwise
 */
static int __wlan_hdd_read_dsrc_chan_stats_debugfs(struct file *file,
						   char __user *buf,
						   size_t count, loff_t *pos)
{
	int i, len;
	ssize_t ret_cnt = 0;
	uint32_t chan_cnt = 0;
	hdd_adapter_t *adapter;
	hdd_context_t *hdd_ctx;
	char *chan_stats_buf, *ptr;
	struct radio_chan_stats_req *req;
	struct dsrc_radio_chan_stats_ctxt *ctx;
	struct radio_chan_stats_info *chan_stats;

	ENTER();

	adapter = (hdd_adapter_t *)file->private_data;
	if ((NULL == adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic)) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Invalid adapter or adapter has invalid magic.",
			  __func__);
		return -EINVAL;
	}

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (wlan_hdd_validate_context(hdd_ctx)) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Invalid hdd context.", __func__);
		return -EINVAL;
	}

	ctx = &adapter->dsrc_chan_stats;
	req = ctx->cur_req;
	if (!req) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Request DSRC radio chan stats is null.",
			  __func__);
		return -EINVAL;
	}

	if (wlan_hdd_dsrc_request_radio_chan_stats(adapter, req)) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Request DSRC Radio chan stats failed.",
			  __func__);
		return -EINVAL;
	}

	len = sizeof(uint32_t) + DSRC_MAX_CHAN_STATS_CNT * sizeof(*chan_stats);
	chan_stats_buf = vos_mem_malloc(len);
	if (!chan_stats_buf) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "dsrc radio chan stats buffer alloc fail.");
		return -EINVAL;
	}
	ptr = chan_stats_buf + sizeof(uint32_t);

	chan_stats = ctx->chan_stats;
	spin_lock(&ctx->chan_stats_lock);
	/* Now only two channel stats supported */
	for (i = 0; i < DSRC_MAX_CHAN_STATS_CNT; i++, chan_stats++) {
		if (chan_stats->chan_freq == 0)
			continue;
		if ((req->req_type == WLAN_DSRC_REQUEST_ONE_RADIO_CHAN_STATS)
		    && (chan_stats->chan_freq != req->chan_freq))
			continue;

		vos_mem_copy(ptr, chan_stats, sizeof(*chan_stats));
		ptr += sizeof(*chan_stats);
		chan_cnt++;
	}
	spin_unlock(&ctx->chan_stats_lock);
	vos_mem_copy(chan_stats_buf, &chan_cnt, sizeof(uint32_t));

	ret_cnt = sizeof(uint32_t) + chan_cnt * sizeof(*chan_stats);
	ret_cnt = simple_read_from_buffer(buf, count, pos,
				chan_stats_buf, ret_cnt);
	vos_mem_free(chan_stats_buf);
	vos_mem_free(ctx->cur_req);
	ctx->cur_req = NULL;

	return ret_cnt;
}

static ssize_t wlan_hdd_write_dsrc_chan_stats_debugfs(struct file *file,
						      const char __user *buf,
						      size_t count,
						      loff_t *ppos)
{
	ssize_t ret;

	vos_ssr_protect(__func__);
	ret = __wlan_hdd_write_dsrc_chan_stats_debugfs(file, buf, count, ppos);
	vos_ssr_unprotect(__func__);

	return ret;
}

static ssize_t wlan_hdd_read_dsrc_chan_stats_debugfs(struct file *file,
						     char __user *buf,
						     size_t count, loff_t *pos)
{
	int ret;

	vos_ssr_protect(__func__);
	ret = __wlan_hdd_read_dsrc_chan_stats_debugfs(file, buf, count, pos);
	vos_ssr_unprotect(__func__);

	return ret;
}

static const struct file_operations fops_dsrc_chan_stats = {
	.write = wlan_hdd_write_dsrc_chan_stats_debugfs,
	.read = wlan_hdd_read_dsrc_chan_stats_debugfs,
	.open = wlan_hdd_debugfs_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

/**
 * wlan_hdd_create_dsrc_chan_stats_file() - API to create dsrc radio channel
 * statistics file
 * @adapter: HDD adapter
 * @hdd_ctx: HDD context
 *
 * Return: 0 on success, -%ENODEV otherwise.
 */
int wlan_hdd_create_dsrc_chan_stats_file(hdd_adapter_t *adapter,
					 hdd_context_t *hdd_ctx)
{
	if (NULL == debugfs_create_file("dsrc_chan_stats",
					S_IRUSR | S_IWUSR | S_IROTH,
					hdd_ctx->debugfs_phy, adapter,
					&fops_dsrc_chan_stats))
		return -ENODEV;

	return 0;
}

static inline bool __tx_stats_enabled(void)
{
	return ol_per_pkt_tx_stats_enabled();
}

static inline void __enable_tx_stats(bool enable)
{
	ol_per_pkt_tx_stats_enable(enable);
}

/**
 * __per_pkt_tx_stats_read() - read DSRC per-packet tx stats by DSRC app
 * @file: file pointer
 * @buf: buffer
 * @count: count
 * @pos: position pointer
 *
 * Return: bytes read on success, error number otherwise
 */
static ssize_t __per_pkt_tx_stats_read(struct file *file,
				       char __user *buf,
				       size_t count, loff_t *pos)
{
	hdd_adapter_t *adapter;
	ssize_t ret = 0;
	struct ol_tx_per_pkt_stats tx_stats;
	long rc;

	ENTER();
	adapter = (hdd_adapter_t *)file->private_data;
	if ((NULL == adapter) || (WLAN_HDD_ADAPTER_MAGIC != adapter->magic)) {
		hddLog(LOGE,FL("Invalid adapter or adapter has invalid magic"));
		return -EINVAL;
	}

	ret = wlan_hdd_validate_context(WLAN_HDD_GET_CTX(adapter));
	if (ret)
		return ret;

	if (!__tx_stats_enabled())
		return ret;

	ret = ol_tx_stats_ring_deque(&tx_stats);
	if (ret) {
		ret = sizeof(tx_stats);
		rc = copy_to_user(buf, &tx_stats, sizeof(tx_stats));
		if (rc)
			ret = -EFAULT;
	}

	EXIT();
	return ret;
}

static ssize_t per_pkt_tx_stats_read(struct file *file,
				     char __user *buf,
				     size_t count, loff_t *pos)
{
	int ret;

	vos_ssr_protect(__func__);
	ret = __per_pkt_tx_stats_read(file, buf, count, pos);
	vos_ssr_unprotect(__func__);

	return ret;
}

/**
 * __enable_per_pkt_tx_stats() - en/disable DSRC tx stats
 * @enable: 1 for enable, 0 for disable
 *
 * Return: 0 on success, error number otherwise
 */
static int __enable_per_pkt_tx_stats(bool enable)
{
	int ret = 0;

	if (enable == __tx_stats_enabled())
		return ret;

	ret = process_wma_set_command(0, WMI_PDEV_PARAM_RADIO_DIAGNOSIS_ENABLE,
				      enable ? 1 : 0, PDEV_CMD);
	if (!ret) {
		adf_os_msleep(100);
		__enable_tx_stats(enable);
	} else {
		hddLog(LOGE,FL("WMI_PDEV_PARAM_RADIO_DIAGNOSIS_ENABLE failed"));
	}

	return ret;
}

/**
 * __per_pkt_tx_stats_write() - en/disable DSRC tx stats by DSRC app
 * @file: file pointer
 * @buf: buffer
 * @count: count
 * @ppos: position pointer
 *
 * Return: @count on success, error number otherwise
 */
static ssize_t __per_pkt_tx_stats_write(struct file *file,
					const char __user *buf,
					size_t count, loff_t *pos)
{
	hdd_adapter_t *pAdapter;
	int ret = -EINVAL;
	char *cmd = 0;
	v_U8_t enable = 0;

	ENTER();

	pAdapter = (hdd_adapter_t *)file->private_data;
	if ((NULL == pAdapter) || (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic)) {
		hddLog(LOGE,FL("Invalid adapter or adapter has invalid magic"));
		return -EINVAL;
	}

	ret = wlan_hdd_validate_context(WLAN_HDD_GET_CTX(pAdapter));
	if (ret)
		return ret;

	/* Get command from user */
	if (count <= MAX_USER_COMMAND_SIZE_FRAME) {
		cmd = vos_mem_malloc(count + 1);
	} else {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Command length is larger than %d bytes.",
			  __func__, MAX_USER_COMMAND_SIZE_FRAME);
		return -EINVAL;
	}

	if (!cmd) {
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			  "%s: Memory allocation for cmd failed!", __func__);
		return -EFAULT;
	}

	if (copy_from_user(cmd, buf, count)) {
		ret = -EFAULT;
		goto failure;
	}
	cmd[count] = '\0';

	if (kstrtou8(cmd, 0, &enable)) {
		ret = -EINVAL;
		goto failure;
	}

	__enable_per_pkt_tx_stats(!!enable);

	vos_mem_free(cmd);
	EXIT();
	return count;

failure:
	vos_mem_free(cmd);
	return ret;
}

static ssize_t per_pkt_tx_stats_write(struct file *file,
				      const char __user *buf,
				      size_t count, loff_t *pos)
{
	ssize_t ret;

	vos_ssr_protect(__func__);
	ret = __per_pkt_tx_stats_write(file, buf, count, pos);
	vos_ssr_unprotect(__func__);

	return ret;
}

static const struct file_operations fops_dsrc_tx_stats = {
	.read = per_pkt_tx_stats_read,
	.write = per_pkt_tx_stats_write,
	.open = wlan_hdd_debugfs_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

/**
 * wlan_hdd_create_dsrc_tx_stats_file() - API to create dsrc tx stats file
 * @adapter: HDD adapter
 * @hdd_ctx: HDD context
 *
 * Return: 0 on success, -%ENODEV otherwise.
 */
int wlan_hdd_create_dsrc_tx_stats_file(hdd_adapter_t *adapter,
				       hdd_context_t *hdd_ctx)
{
	if (NULL == debugfs_create_file("dsrc_tx_stats",
					S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
					hdd_ctx->debugfs_phy, adapter,
					&fops_dsrc_tx_stats))
		return -ENODEV;

	return 0;
}
#endif /* WLAN_OPEN_SOURCE */
