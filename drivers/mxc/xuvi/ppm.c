/*
 * Copyright 2019-2020 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ppm.c
 *
 * copyright here may be changed later
 *
 *
 */

#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include "ppm.h"
#include "xuvi_pm.h"
#include "xuvi_mu.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#define CSR_CM0Px_ADDR_OFFSET  0x00000000
#define CSR_CM0Px_CPUWAIT      0x00000004

#define LVL_DEBUG 3
#define LVL_INFO 2
#define LVL_WARN  1
#define LVL_ERR  0

static int ppm_debug_level = 1;
static int debug_firmware;

module_param(ppm_debug_level, int, 0644);
MODULE_PARM_DESC(ppm_debug_level, "ppm debug level (0-3)");
module_param(debug_firmware, int, 0644);
MODULE_PARM_DESC(debug_firmware, "open FW debug log");

#define dprintk(level, fmt, arg...) do { \
	if (ppm_debug_level >= (level)) \
	printk(KERN_ERR pr_fmt("level: %d %s line: %d: " fmt), \
	level, __func__, __LINE__, ##arg); \
} while (0)

static void dvb_dmx_filter(struct dvb_demux *demux, const u8 *buf,
			   uint32_t len, u16 pid)
{
	struct dvb_demux_feed *feed;
	struct ppm_ctx *ctx = demux->priv;
	struct ppm_dev *ppm = ctx->dev;

	list_for_each_entry(feed, &demux->feed_list, list_head) {

		if (ctx->stop_start || !feed)
			break;

		if ((feed->pid != pid) && (feed->pid != 0x2000))
			continue;

		while (feed->cb.ts(buf, len, NULL, 0, &feed->feed.ts,
				   &feed->buffer_flags) < 0) {
			dprintk(LVL_DEBUG, "ringbuffer overflow, retry");
			if (ctx->stop_start || ppm->suspend) {
				dprintk(LVL_WARN, "suspend or stop drop");
				break;
			}
			msleep(1000);
		}
	}
}

static void send_msg(struct ppm_dev *dev, uint32_t value0, uint32_t value1,
		     uint32_t value2, uint32_t value3)
{
	xuvi_mu_send_msg(dev, value0, value1, value2, value3);
}

static void ppm_send_cmd(struct ppm_dev *dev, uint32_t idx, uint32_t cmdid,
			 uint32_t value1, uint32_t value2, uint32_t value3)
{
	uint32_t cmdword;

	cmdword = 0;
	cmdword |= ((idx & 0x000000ff) << 16);
	cmdword |= ((cmdid & 0x00003fff) << 0);

	dprintk(LVL_DEBUG, "send idx: %d msg: %x 0x%x %d %d\n", idx,
		cmdword, value1, value2, value3);
	send_msg(dev, cmdword, value1, value2, value3);
}

static struct ppm_buf_list *find_buf(struct ppm_dev *ppm, uint32_t phy,
				     bool is_input)
{
	int i;
	struct ppm_buf_list *buf;

	for (i = 0; i < BUF_NUM; i++) {
		if (is_input)
			buf = &ppm->bufs[i];
		else
			buf = &ppm->out_bufs[i];

		if (buf->phy == phy)
			return buf;
	}
	dprintk(LVL_ERR, "can't find buffer: 0x%x is_input %d\n", phy,
		is_input);

	return NULL;
}

static void ppm_event_handler(struct ppm_ctx *ctx, uint32_t idx,
			      uint32_t evt, uint32_t *event_data)
{
	struct ppm_dev *dev;
	struct ppm_buf_list *buf;
	uint32_t phy = event_data[1];
	uint32_t len = event_data[2];
	uint32_t pid = event_data[3];

	if (ctx == NULL) {
		dprintk(LVL_ERR,
			"receive event: 0x%X after instance released, ignore it\n",
			evt);
		return;
	}

	dev = ctx->dev;

	switch (evt) {
	case STOP:
		dprintk(LVL_ERR, "receive PPM_EVENT_STOPPED\n");
		ctx->fw_stopped = true;
		complete(&ctx->stop_comp);
		break;
	case BUF_FROM:
		if (len) {
			buf = find_buf(dev, phy, false);
			dvb_dmx_filter(&ctx->demux, buf->vir, len, pid);
			mutex_lock(&dev->buf_list_lock);
			ppm_send_cmd(dev, 0, BUF_TO, buf->phy, 0,
				     buf->size);
			mutex_unlock(&dev->buf_list_lock);
		} else {
			buf = find_buf(dev, phy, true);
			mutex_lock(&dev->buf_list_lock);
			list_add_tail(&buf->list, &dev->buf_list);
			mutex_unlock(&dev->buf_list_lock);
		}
		break;
	default:
		dprintk(LVL_ERR, "warning: evt %d is not handled\n", evt);
		break;
	}
	dprintk(LVL_DEBUG, "leave, evt %d\n", evt);
}

static void send_msg_queue(struct ppm_ctx *ctx, struct event_msg *msg)
{
	uint32_t ret;

	ret =
	    kfifo_in(&ctx->msg_fifo, msg,
		     sizeof(uint32_t) * (MSG_WORD_LENGTH + msg->msgnum));
	if (ret != sizeof(uint32_t) * (MSG_WORD_LENGTH + msg->msgnum))
		dprintk(LVL_ERR,
			"There is no memory for msg fifo, ret=%d\n", ret);
}

static bool receive_msg_queue(struct ppm_ctx *ctx, struct event_msg *msg)
{
	uint32_t ret;
	struct ppm_dev *ppm = ctx->dev;

	if (ppm->suspend)
		return false;

	if (kfifo_len(&ctx->msg_fifo) >=
	    sizeof(uint32_t) * MSG_WORD_LENGTH) {
		ret =
		    kfifo_out(&ctx->msg_fifo, msg,
			      sizeof(uint32_t) * MSG_WORD_LENGTH);
		if (ret != sizeof(uint32_t) * MSG_WORD_LENGTH) {
			dprintk(LVL_ERR,
				"kfifo_out msg word has error, ret=%d\n",
				ret);
			return false;
		} else {
			if (msg->msgnum > 0) {
				if (kfifo_len(&ctx->msg_fifo) >=
				    sizeof(uint32_t) * msg->msgnum) {
					ret =
					    kfifo_out(&ctx->msg_fifo,
						      msg->msgdata,
						      sizeof(uint32_t) *
						      msg->msgnum);
					if (ret !=
					    sizeof(uint32_t) *
					    msg->msgnum) {
						dprintk(LVL_ERR,
							"kfifo_out msg data has error, ret=%d\n",
							ret);
						return false;
					} else
						return true;
				} else
					return false;
			} else
				return true;
		}
	} else
		return false;
}

static void read_msg(struct ppm_dev *dev, struct event_msg *msg)
{
	uint32_t msg_mu[4];
	uint32_t msgword;
	uint32_t ret;

	ret = kfifo_out(&dev->msg_fifo, msg_mu, sizeof(uint32_t) * 4);
	if (ret != sizeof(uint32_t) * 4) {
		dprintk(LVL_ERR, "kfifo_out msg word has error, ret=%d\n",
			ret);
	}

	msgword = msg_mu[0];
	msg->idx = ((msgword & 0x00ff0000) >> 16);
	msg->msgid = ((msgword & 0x00003fff) >> 0);

	msg->msgnum = 4;
	msg->msgdata[0] = msg_mu[0];
	msg->msgdata[1] = msg_mu[1];
	msg->msgdata[2] = msg_mu[2];
	msg->msgdata[3] = msg_mu[3];
}

static void ppm_msg_instance_work(struct work_struct *work)
{
	struct ppm_ctx *ctx =
	    container_of(work, struct ppm_ctx, instance_work);
	struct event_msg msg;

	if (!ctx)
		return;

	memset(&msg, 0, sizeof(struct event_msg));

	while (receive_msg_queue(ctx, &msg))
		ppm_event_handler(ctx, msg.idx, msg.msgid, msg.msgdata);
}

static int ensure_buf(struct ppm_dev *ppm, bool is_input)
{
	uint32_t i;
	struct ppm_buf_list *buf;

	if (is_input)
		buf = &ppm->bufs[0];
	else
		buf = &ppm->out_bufs[0];

	if (buf->vir)
		return 0;

	for (i = 0; i < BUF_NUM; i++) {
		if (is_input)
			buf = &ppm->bufs[i];
		else
			buf = &ppm->out_bufs[i];

		buf->vir = dma_alloc_coherent(&ppm->plat_dev->dev,
					      BUF_SIZE,
					      (dma_addr_t *) &buf->phy,
					      GFP_KERNEL | GFP_DMA32);

		if (!buf->vir) {
			dprintk(LVL_ERR, "dma buf alloc size(%d) fail!\n",
				BUF_SIZE);
			return -ENOMEM;
		}
		buf->size = BUF_SIZE;
		if (is_input)
			list_add_tail(&buf->list, &ppm->buf_list);
		else
			ppm_send_cmd(ppm, 0, BUF_TO, buf->phy, 0,
				     buf->size);

		dprintk(LVL_DEBUG, "Allocate buffer: 0x%x 0x%p\n",
			buf->phy, buf->vir);
	}

	return 0;
}

static int ppm_dvbdmx_write(struct dmx_demux *demux,
			    const char __user *buf_in, size_t count)
{
	struct dvb_demux *dvbdemux = (struct dvb_demux *) demux;
	struct ppm_ctx *ctx = dvbdemux->priv;
	struct ppm_dev *dev = ctx->dev;
	struct ppm_buf_list *buf;
	int ret;
	void *p;

	if ((!demux->frontend)
	    || (demux->frontend->source != DMX_MEMORY_FE))
		return -EINVAL;

	if (!count)
		return 0;

	p = memdup_user(buf_in, count);
	if (IS_ERR(p))
		return PTR_ERR(p);

	ret = ensure_buf(dev, true);
	if (ret) {
		dprintk(LVL_ERR, "alloc ppm buffer fail\n");
		return ret;
	}

	mutex_lock(&dev->dev_ins_mutex);
	while (list_empty(&dev->buf_list)) {
		if (signal_pending(current)) {
			dprintk(LVL_WARN, "fatal_signal_pending");
			mutex_unlock(&dev->dev_ins_mutex);
			return 0;
		}
		msleep(20);
	}

	mutex_lock(&dev->buf_list_lock);
	buf = list_first_entry(&dev->buf_list, struct ppm_buf_list, list);
	list_del(&buf->list);
	mutex_unlock(&dev->buf_list_lock);
	mutex_unlock(&dev->dev_ins_mutex);
	memcpy(buf->vir, p, count);
	mutex_lock(&dev->buf_list_lock);

	ppm_send_cmd(dev, ctx->idx, BUF_TO, buf->phy, count, buf->size);
	mutex_unlock(&dev->buf_list_lock);

	if (mutex_lock_interruptible(&dvbdemux->mutex)) {
		kfree(p);
		return -ERESTARTSYS;
	}

	kfree(p);
	mutex_unlock(&dvbdemux->mutex);

	return count;
}

static void ppm_run_fw(struct ppm_dev *dev)
{
	writel(dev->fw_space_phy, dev->csr_base);
	writel(0x0, dev->csr_base + CSR_CM0Px_CPUWAIT);
}

static int ppm_fw_download(struct ppm_dev *dev)
{
	unsigned char *image;
	unsigned int fw_size = 0;
	int ret = 0;
	char *p = dev->fw_space_vir;

	ret = request_firmware((const struct firmware **) &dev->pfw,
			       M0FW_FILENAME, dev->generic_dev);
	if (ret) {
		dprintk(LVL_ERR, "request fw %s failed(%d)\n",
			M0FW_FILENAME, ret);
		return ret;
	} else {
		dprintk(LVL_INFO, "request fw %s got size(%d)\n",
			M0FW_FILENAME, (int) dev->pfw->size);
		image = (uint8_t *) dev->pfw->data;
		fw_size = dev->pfw->size;
	}
	memcpy(dev->fw_space_vir, image, fw_size);

	release_firmware(dev->pfw);
	dev->pfw = NULL;

	p[18] = 1;
	ppm_run_fw(dev);

	return ret;
}

static int ppm_dev_event_handler(struct ppm_dev *dev,
				 struct event_msg *msg)
{
	int cmd;

	cmd = msg->msgdata[0];
	cmd &= 0x00003fff;
	if (cmd == 0xaa) {
		ppm_send_cmd(dev, 0, BOOT_ADDRESS, dev->fw_space_phy, 0,
			     0);
		ppm_send_cmd(dev, 0, INIT_DONE, LOG_OFFSET,
			     dev->boot_size - LOG_OFFSET, 0);
		dev->print_buf = dev->fw_space_vir + LOG_OFFSET;
	} else if (cmd == 0x55) {
		dev->fw_started = true;
		complete(&dev->fw_start_comp);
	} else if (cmd == 0xA5) {
		complete(&dev->snap_done_comp);
	} else if (cmd == HARD_FAULT) {
		dprintk(LVL_ERR, "FW exception. lr: %x addr: %x\n",
			msg->msgdata[1], msg->msgdata[2]);
	} else {
		return 1;
	}

	return 0;
}

static void ppm_msg_run_work(struct work_struct *work)
{
	struct ppm_dev *dev = container_of(work, struct ppm_dev, msg_work);
	struct ppm_ctx *ctx;
	struct event_msg msg;

	memset(&msg, 0, sizeof(struct event_msg));

	while (kfifo_len(&dev->msg_fifo) >= sizeof(uint32_t) * 4) {
		read_msg(dev, &msg);
		if (ppm_dev_event_handler(dev, &msg) == 0)
			continue;
		mutex_lock(&dev->dev_mutex);
		ctx = dev->ctx[msg.idx];
		if (ctx != NULL) {
			mutex_lock(&ctx->instance_mutex);
			if (!ctx->ctx_released) {
				send_msg_queue(ctx, &msg);
				queue_work(ctx->instance_wq,
					   &ctx->instance_work);
			}
			mutex_unlock(&ctx->instance_mutex);
		}
		mutex_unlock(&dev->dev_mutex);
	}
}

static void print_firmware_debug(char *ptr, u32 size)
{
	u32 total = 0;
	u32 len;

	while (total < size) {
		len = min_t(u32, size - total, 256);
		dprintk(LVL_ERR, "%.*s", len, ptr + total);
		total += len;
	}
}

static void firmware_debug(struct ppm_dev *dev)
{
	char *ptr;
	u32 rptr;
	u32 wptr;

	if (!dev || !dev->print_buf)
		return;

	if (!debug_firmware)
		return;

	rptr = dev->print_buf->read;
	wptr = dev->print_buf->write;
	if (rptr == wptr)
		return;

	ptr = dev->print_buf->buffer;
	if (rptr > wptr) {
		print_firmware_debug(ptr + rptr,
				     dev->print_buf->bytes - rptr);
		rptr = 0;
	}
	if (rptr < wptr) {
		print_firmware_debug(ptr + rptr, wptr - rptr);
		rptr = wptr;
	}
	if (rptr >= dev->print_buf->bytes)
		rptr = 0;
	dev->print_buf->read = rptr;
}

static void ppm_watchdog_handler(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct ppm_dev *dev;

	if (!work)
		return;

	dwork = to_delayed_work(work);
	dev = container_of(dwork, struct ppm_dev, watchdog);

	firmware_debug(dev);

	schedule_delayed_work(&dev->watchdog,
			      msecs_to_jiffies(PPM_WATCHDOG_INTERVAL_MS));
}

static void init_ppm_watchdog(struct ppm_dev *dev)
{
	if (!dev)
		return;

	INIT_DELAYED_WORK(&dev->watchdog, ppm_watchdog_handler);
	schedule_delayed_work(&dev->watchdog,
			      msecs_to_jiffies(PPM_WATCHDOG_INTERVAL_MS));
}

static int ppm_start(struct ppm_ctx *ctx)
{
	int ret = 0;
	struct ppm_dev *dev = ctx->dev;

	pm_runtime_get_sync(&dev->plat_dev->dev);

	ctx->ctx_released = false;
	ctx->fw_stopped = false;
	ctx->stop_start = false;
	if (!ctx->instance_activated) {
		init_completion(&ctx->stop_comp);
		init_completion(&ctx->eos_comp);
		INIT_WORK(&ctx->instance_work, ppm_msg_instance_work);
		mutex_init(&ctx->instance_mutex);

		if (kfifo_alloc(&ctx->msg_fifo,
				sizeof(struct event_msg) *
				PPM_MESSAGE_LIMIT, GFP_KERNEL)) {
			dprintk(LVL_ERR, "fail to alloc fifo when open\n");
			ret = -ENOMEM;
			goto err_alloc_fifo;
		}

		ctx->instance_wq =
		    alloc_workqueue("ppm_instance",
				    WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
		if (!ctx->instance_wq) {
			dprintk(LVL_ERR,
				"unable to alloc workqueue for ctx\n");
			ret = -ENOMEM;
			goto err_alloc_wq;
		}
		ctx->instance_activated = true;
	}

	mutex_lock(&dev->dev_mutex);
	if (!dev->fw_is_ready) {
		dev->fw_space_vir =
		    ioremap_wc(dev->fw_space_phy, dev->boot_size);
		if (!dev->fw_space_vir) {
			dprintk(LVL_ERR, "can't map firmware space\n");
			ret = -1;
			mutex_unlock(&dev->dev_mutex);
			goto err_fw_load;
		}
		memset_io(dev->fw_space_vir, 0, dev->boot_size);

		ret = ppm_fw_download(dev);
		if (ret) {
			dprintk(LVL_ERR, "ppm_fw_download fail\n");
			mutex_unlock(&dev->dev_mutex);
			goto err_fw_load;
		}
		dprintk(LVL_INFO, "done: ppm_fw_download\n");
		if (!ctx->dev->fw_started)
			if (!wait_for_completion_timeout
			    (&ctx->dev->fw_start_comp,
			     msecs_to_jiffies(10000))) {
				dprintk(LVL_ERR,
					"don't get start interrupt\n");
				ret = -1;
				mutex_unlock(&dev->dev_mutex);
				goto err_fw_load;
			}
		dev->fw_is_ready = true;
	}

	ret = ensure_buf(dev, false);
	if (ret) {
		dprintk(LVL_ERR, "alloc ppm buffer fail\n");
		mutex_unlock(&dev->dev_mutex);
		goto err_fw_load;
	}
	mutex_unlock(&dev->dev_mutex);

	return 0;

err_fw_load:
	destroy_workqueue(ctx->instance_wq);
err_alloc_wq:
	if (&ctx->msg_fifo)
		kfifo_free(&ctx->msg_fifo);
err_alloc_fifo:
	pm_runtime_put_sync(dev->generic_dev);

	return ret;
}

static int ppm_stop(struct ppm_ctx *ctx)
{
	struct ppm_dev *dev = ctx->dev;

	dprintk(LVL_ERR, "ppm stop");
	if (!ctx->fw_stopped && ctx->start_flag == false) {
		dprintk(LVL_INFO, "send STOP\n");
		ppm_send_cmd(dev, ctx->idx, STOP, 0, 0, 0);
		if (!wait_for_completion_timeout
		    (&ctx->stop_comp, msecs_to_jiffies(1000))) {
			dprintk(LVL_ERR,
				"the path id:%d fw hang after send PPM_CMD_STOP\n",
				ctx->idx);
		}
	} else
		dprintk(LVL_INFO, "stopped(%d): skip PPM_CMD_STOP\n",
			ctx->fw_stopped);

	pm_runtime_put_sync(dev->generic_dev);

	return 0;
}

static int ppm_dvb_start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ppm_ctx *ppm = dvbdmx->priv;
	struct dvb_demux_feed *feed;

	dprintk(LVL_DEBUG, "PPM DVB Start feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(LVL_DEBUG, "no frontend ?");
		return -EINVAL;
	}

	ppm->feeds++;
	dprintk(LVL_DEBUG, "ppm start feed, feeds=%d", ppm->feeds);

	if (ppm->feeds == 1) {
		dprintk(LVL_DEBUG, "ppm start feed & dma");
		ppm_start(ppm);
	}

	list_for_each_entry(feed, &dvbdmx->feed_list, list_head) {
		dprintk(LVL_DEBUG, "feed pid: 0x%X\n", feed->pid);
		ppm_send_cmd(ppm->dev, 0, ADD_PID, feed->pid, 0, 0);
	}

	return ppm->feeds;
}

static int ppm_dvb_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ppm_ctx *ctx = dvbdmx->priv;
	struct dvb_demux_feed *feed;

	dprintk(LVL_DEBUG, "PPM DVB Stop feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(LVL_DEBUG, "no frontend ?");
		return -EINVAL;
	}

	ctx->feeds--;
	ctx->stop_start = true;
	if (ctx->feeds == 0) {
		dprintk(LVL_DEBUG, "ppm stop feed and dma");
		ppm_stop(ctx);
	}

	list_for_each_entry(feed, &dvbdmx->feed_list, list_head) {
		dprintk(LVL_DEBUG, "feed pid: 0x%X\n", feed->pid);
		ppm_send_cmd(ctx->dev, 0, RM_PID, feed->pid, 0, 0);
	}

	return 0;
}

int ppm_dvb_init(struct ppm_ctx *ppm)
{
	int result = -1;

	dprintk(LVL_DEBUG, "dvb_register_adapter");

	result = dvb_register_adapter(&ppm->dvb_adapter,
				      "PPM DVB adapter",
				      THIS_MODULE,
				      &ppm->dev->plat_dev->dev,
				      adapter_nr);

	if (result < 0) {
		dprintk(LVL_ERR, "Error registering adapter");
		return -ENODEV;
	}

	ppm->dvb_adapter.priv = ppm;
	ppm->demux.dmx.capabilities = DMX_TS_FILTERING |
	    DMX_SECTION_FILTERING | DMX_MEMORY_BASED_FILTERING;

	ppm->demux.priv = ppm;
	ppm->demux.filternum = 256;
	ppm->demux.feednum = 256;
	ppm->demux.start_feed = ppm_dvb_start_feed;
	ppm->demux.stop_feed = ppm_dvb_stop_feed;
	ppm->demux.write_to_decoder = NULL;

	dprintk(LVL_DEBUG, "dvb_dmx_init");
	result = dvb_dmx_init(&ppm->demux);
	if (result < 0) {
		dprintk(LVL_ERR, "dvb_dmx_init failed, ERROR=%d", result);
		goto err0;
	}

	ppm->dmxdev.filternum = 256;
	ppm->dmxdev.demux = &ppm->demux.dmx;
	ppm->dmxdev.demux->write = ppm_dvbdmx_write;
	ppm->dmxdev.capabilities = 0;
	dprintk(LVL_DEBUG, "dvb_dmxdev_init");

	result = dvb_dmxdev_init(&ppm->dmxdev, &ppm->dvb_adapter);
	if (result < 0) {
		dprintk(LVL_ERR, "dvb_dmxdev_init failed, ERROR=%d",
			result);
		goto err1;
	}

	ppm->fe_hw.source = DMX_FRONTEND_0;
	result = ppm->demux.dmx.add_frontend(&ppm->demux.dmx, &ppm->fe_hw);
	if (result < 0) {
		dprintk(LVL_ERR, "dvb_dmx_init failed, ERROR=%d", result);
		goto err2;
	}

	ppm->fe_mem.source = DMX_MEMORY_FE;
	result =
	    ppm->demux.dmx.add_frontend(&ppm->demux.dmx, &ppm->fe_mem);
	if (result < 0) {
		dprintk(LVL_ERR, "dvb_dmx_init failed, ERROR=%d", result);
		goto err3;
	}

	result =
	    ppm->demux.dmx.connect_frontend(&ppm->demux.dmx, &ppm->fe_hw);
	if (result < 0) {
		dprintk(LVL_ERR, "dvb_dmx_init failed, ERROR=%d", result);
		goto err4;
	}

	dvb_net_init(&ppm->dvb_adapter, &ppm->dvbnet, &ppm->demux.dmx);

	return 0;

	/* Error conditions ..        */
err4:
	ppm->demux.dmx.remove_frontend(&ppm->demux.dmx, &ppm->fe_mem);

err3:
	ppm->demux.dmx.remove_frontend(&ppm->demux.dmx, &ppm->fe_hw);

err2:
	dvb_dmxdev_release(&ppm->dmxdev);

err1:
	dvb_dmx_release(&ppm->demux);

err0:
	dvb_unregister_adapter(&ppm->dvb_adapter);

	return result;
}

static int ppm_dvb_exit(struct ppm_ctx *ppm)
{
	dvb_net_release(&ppm->dvbnet);

	ppm->demux.dmx.remove_frontend(&ppm->demux.dmx, &ppm->fe_mem);
	ppm->demux.dmx.remove_frontend(&ppm->demux.dmx, &ppm->fe_hw);

	dvb_dmxdev_release(&ppm->dmxdev);
	dvb_dmx_release(&ppm->demux);

	dprintk(LVL_DEBUG, "dvb_unregister_adapter");
	dvb_unregister_adapter(&ppm->dvb_adapter);

	return 0;
}

static int parse_dt_info(struct ppm_dev *dev, struct device_node *np)
{
	struct resource reserved_res;
	struct device_node *reserved_node;
	uint32_t csr_base;
	int ret;

	if (!dev || !np)
		return -EINVAL;

	reserved_node = of_parse_phandle(np, "boot-region", 0);
	if (!reserved_node) {
		dprintk(LVL_ERR, "boot-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		dprintk(LVL_ERR,
			"boot-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->fw_space_phy = reserved_res.start;
	dev->boot_size = resource_size(&reserved_res);
	dprintk(LVL_DEBUG, "boot-region: %x size: %d\n", dev->fw_space_phy,
		dev->boot_size);

	ret = of_property_read_u32(np, "reg-csr", &csr_base);
	if (ret) {
		dprintk(LVL_ERR, "Cannot get csr offset %d\n", ret);
		return -EINVAL;
	}
	dev->csr_base = ioremap(csr_base, 8);

	return 0;
}

static int ppm_probe(struct platform_device *pdev)
{
	struct ppm_dev *dev;
	struct ppm_ctx *ctx;
	struct device_node *np = pdev->dev.of_node;
	int ret, i;

	dprintk(LVL_DEBUG, "enter probe\n");

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->plat_dev = pdev;

	ret = xuvi_attach_pm_domains(dev);
	if (ret)
		goto err_pm;

	ret = parse_dt_info(dev, np);
	if (ret) {
		dprintk(LVL_ERR, "parse device tree fail\n");
		goto err_put_dev;
	}

	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
		if (!ctx) {
			ret = -ENOMEM;
			goto err_put_dev;
		}

		ctx->dev = dev;
		ctx->idx = i;
		dev->ctx[i] = ctx;

		ret = ppm_dvb_init(ctx);
		if (ret) {
			dprintk(LVL_ERR, "ppm dvb init fail\n");
			goto err_put_dev;
		}
	}

	platform_set_drvdata(pdev, dev);

	init_ppm_watchdog(dev);
	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->dev_ins_mutex);
	INIT_WORK(&dev->msg_work, ppm_msg_run_work);
	INIT_LIST_HEAD(&dev->buf_list);
	mutex_init(&dev->buf_list_lock);
	init_waitqueue_head(&dev->buffer_wq);
	init_completion(&dev->fw_start_comp);
	init_completion(&dev->snap_done_comp);
	if (kfifo_alloc(&dev->msg_fifo,
			4 * 4 * PPM_MESSAGE_LIMIT, GFP_KERNEL)) {
		dprintk(LVL_ERR, "fail to alloc fifo when open\n");
		ret = -ENOMEM;
		goto err_put_dev;
	}

	dev->workqueue =
	    alloc_workqueue("ppm", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		dprintk(LVL_ERR, "unable to alloc workqueue for dev\n");
		ret = -ENOMEM;
		goto err_put_dev;
	}

	dev->fw_started = false;
	dev->fw_is_ready = false;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	dev->generic_dev = get_device(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);

	return 0;

err_put_dev:
	xuvi_detach_pm_domains(dev);
err_pm:
	if (&dev->msg_fifo)
		kfifo_free(&dev->msg_fifo);

	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	devm_kfree(&pdev->dev, dev);
	dprintk(LVL_ERR, "probe failed\n");

	return ret;
}

static void free_dma_buf(struct ppm_dev *dev)
{
	struct ppm_buf_list *buf;
	uint32_t i;

	for (i = 0; i < BUF_NUM; i++) {
		buf = &dev->out_bufs[i];
		if (!buf->vir)
			break;
		dma_free_coherent(&dev->plat_dev->dev, buf->size, buf->vir,
				  buf->phy);
		buf->vir = NULL;
	}

	for (i = 0; i < BUF_NUM; i++) {
		buf = &dev->bufs[i];
		if (!buf->vir)
			break;
		dma_free_coherent(&dev->plat_dev->dev, buf->size, buf->vir,
				  buf->phy);
		buf->vir = NULL;
	}
}

static int ppm_remove(struct platform_device *pdev)
{
	struct ppm_dev *dev = platform_get_drvdata(pdev);
	struct ppm_ctx *ctx;
	int i;

	dev->fw_started = false;
	dev->fw_is_ready = false;

	cancel_delayed_work_sync(&dev->watchdog);

	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		ctx = dev->ctx[i];
		if (!ctx->instance_activated)
			continue;
		mutex_lock(&ctx->instance_mutex);
		ctx->ctx_released = true;
		kfifo_free(&ctx->msg_fifo);
		destroy_workqueue(ctx->instance_wq);
		mutex_unlock(&ctx->instance_mutex);
	}

	free_dma_buf(dev);

	if (dev->fw_space_vir)
		iounmap(dev->fw_space_vir);
	dev->fw_space_vir = NULL;
	dev->fw_space_phy = 0;

	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		ppm_dvb_exit(dev->ctx[i]);
		kfree(dev->ctx[i]);
		dev->ctx[i] = NULL;
	}

	if (&dev->msg_fifo)
		kfifo_free(&dev->msg_fifo);

	destroy_workqueue(dev->workqueue);

	pm_runtime_disable(&pdev->dev);

	xuvi_detach_pm_domains(dev);
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}
	devm_kfree(&pdev->dev, dev);

	return 0;
}

static int ppm_runtime_suspend(struct device *dev)
{
	struct ppm_dev *ppmdev = (struct ppm_dev *)dev_get_drvdata(dev);

	dprintk(LVL_DEBUG, "ppm_runtime_suspend\n");
	if (ppmdev->generic_dev)
		xuvi_mu_free(ppmdev);

	return 0;
}

static int ppm_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct ppm_dev *ppmdev = (struct ppm_dev *)dev_get_drvdata(dev);

	dprintk(LVL_DEBUG, "ppm_runtime_resume\n");
	if (ppmdev->generic_dev)
		ret = xuvi_mu_request(ppmdev);

	return ret;
}

static int find_first_available_instance(struct ppm_dev *dev)
{
	int strIdx = 0;

	if (!dev)
		return -EINVAL;

	return strIdx;
}

static void ppm_send_snapshot(struct ppm_dev *dev)
{
	int strIdx;

	strIdx = find_first_available_instance(dev);
	if (strIdx >= 0 && strIdx < PPM_MAX_INSTANCE) {
		dprintk(LVL_INFO, "send SNAP_SHOT\n");
		ppm_send_cmd(dev, strIdx, SNAP_SHOT, 0, 0, 0);
	} else
		dprintk(LVL_WARN,
			"warning: all path hang, need to reset\n");
}

static void ppm_dec_cancel_work(struct ppm_dev *ppmdev)
{
	int i;

	mutex_lock(&ppmdev->dev_mutex);
	cancel_work_sync(&ppmdev->msg_work);
	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		struct ppm_ctx *ctx = ppmdev->ctx[i];

		if (!ctx->instance_activated)
			continue;
		cancel_work_sync(&ppmdev->ctx[i]->instance_work);
	}
	cancel_delayed_work_sync(&ppmdev->watchdog);
	mutex_unlock(&ppmdev->dev_mutex);
}

static bool ppm_has_instance(struct ppm_dev *ppmdev)
{
	int i;

	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		struct ppm_ctx *ctx = ppmdev->ctx[i];

		if (ctx)
			return true;
	}
	return false;
}

static void ppm_dec_resume_work(struct ppm_dev *ppmdev)
{
	int i;

	mutex_lock(&ppmdev->dev_mutex);
	queue_work(ppmdev->workqueue, &ppmdev->msg_work);
	for (i = 0; i < PPM_MAX_INSTANCE; i++) {
		struct ppm_ctx *ctx = ppmdev->ctx[i];

		if (!ctx->instance_activated)
			continue;
		if (!ctx->ctx_released)
			queue_work(ctx->instance_wq, &ctx->instance_work);
	}
	schedule_delayed_work(&ppmdev->watchdog,
			      msecs_to_jiffies(PPM_WATCHDOG_INTERVAL_MS));
	mutex_unlock(&ppmdev->dev_mutex);
}

static int __maybe_unused ppm_suspend(struct device *dev)
{
	struct ppm_dev *ppmdev = (struct ppm_dev *) dev_get_drvdata(dev);
	int ret = 0;

	dprintk(LVL_INFO, "suspend\n");
	pm_runtime_get_sync(ppmdev->generic_dev);
	if (ppmdev->fw_is_ready) {
		ppm_send_snapshot(ppmdev);
		reinit_completion(&ppmdev->snap_done_comp);
		if (!wait_for_completion_timeout
		    (&ppmdev->snap_done_comp, msecs_to_jiffies(1000))) {
			dprintk(LVL_ERR,
				"error: wait for ppm decoder snapdone event timeout!\n");
			ret = -1;
		}
	}

	ppmdev->suspend = true;
	ppm_dec_cancel_work(ppmdev);
	ppmdev->suspend = false;
	dprintk(LVL_INFO, "suspend done\n");
	pm_runtime_put_sync(ppmdev->generic_dev);

	return ret;
}

static bool is_ppm_poweroff(struct ppm_dev *ppmdev)
{
	if (!ppmdev)
		return false;

	if (readl_relaxed(ppmdev->csr_base + CSR_CM0Px_CPUWAIT) == 1)
		return true;
	else
		return false;

}

static int resume_mu(struct ppm_dev *ppmdev)
{
	if (!ppmdev)
		return -EINVAL;

	return 0;
}

static int resume_from_snapshot(struct ppm_dev *ppmdev)
{
	int ret = 0;

	ppm_run_fw(ppmdev);
	/*wait for firmware resotre done */
	reinit_completion(&ppmdev->fw_start_comp);
	if (!wait_for_completion_timeout
	    (&ppmdev->fw_start_comp, msecs_to_jiffies(1000))) {
		dprintk(LVL_ERR,
			"error: wait for ppm decoder resume done timeout!\n");
		ret = -1;
	}

	return ret;
}

static int resume_from_ppm_poweroff(struct ppm_dev *ppmdev)
{
	int ret = 0;

	dprintk(LVL_INFO, "resume from poweroff\n");
	ret = resume_from_snapshot(ppmdev);

	return ret;
}

static int __maybe_unused ppm_resume(struct device *dev)
{
	struct ppm_dev *ppmdev = (struct ppm_dev *) dev_get_drvdata(dev);
	int ret = 0;

	dprintk(LVL_INFO, "resume\n");
	pm_runtime_get_sync(ppmdev->generic_dev);

	if (ppmdev->fw_is_ready == false)
		goto exit;

	resume_mu(ppmdev);

	if (is_ppm_poweroff(ppmdev))
		ret = resume_from_ppm_poweroff(ppmdev);
	else if (ppm_has_instance(ppmdev))
		dprintk(LVL_INFO, "resume from instance\n");

	ppm_dec_resume_work(ppmdev);

exit:
	pm_runtime_put_sync(ppmdev->generic_dev);
	dprintk(LVL_INFO, "resume done\n");

	return ret;
}

static const struct dev_pm_ops ppm_pm_ops = {
	SET_RUNTIME_PM_OPS(ppm_runtime_suspend, ppm_runtime_resume, NULL)
	    SET_SYSTEM_SLEEP_PM_OPS(ppm_suspend, ppm_resume)
};

static const struct of_device_id demux_device_match[] = {
	{.compatible = "nxp,imx8qm-b0-vpu-ts",},
	{},
};

MODULE_DEVICE_TABLE(of, demux_device_match);

static struct platform_driver demux_platform_driver = {
	.probe = ppm_probe,
	.remove = ppm_remove,
	.driver = {
		   .pm = &ppm_pm_ops,
		   .name = "ppm",
		   .of_match_table = demux_device_match,
		   .owner = THIS_MODULE,
		   },
};

static int ppm_device_init(void)
{
	return platform_driver_register(&demux_platform_driver);
}

static void ppm_device_exit(void)
{
	platform_driver_unregister(&demux_platform_driver);
}

module_init(ppm_device_init)
module_exit(ppm_device_exit)

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX8QM PPM driver");
MODULE_LICENSE("GPL");
