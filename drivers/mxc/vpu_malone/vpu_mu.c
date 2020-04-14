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

/*
 *
 * @file vpu_mu.c
 *
 */

#include "vpu_mu.h"

static void vpu_mu_inq_msg(struct vpu_dev *dev, void *msg)
{
	if (&dev->mu_msg_fifo == NULL) {
		vpu_err("&dev->mu_msg_fifo == NULL\n");
		return;
	}

	if (kfifo_in(&dev->mu_msg_fifo, msg, sizeof(u_int32)) != sizeof(u_int32)) {
		vpu_err("No memory for mu msg fifo\n");
		return;
	}

	queue_work(dev->workqueue, &dev->msg_work);
	queue_delayed_work(dev->workqueue,
			&dev->delayed_msg_work, msecs_to_jiffies(10));
}

static void vpu_mbox_free(struct vpu_dev *dev)
{
	if (dev->sc_chan_tx0.ch && !IS_ERR(dev->sc_chan_tx0.ch))
		mbox_free_channel(dev->sc_chan_tx0.ch);
	if (dev->sc_chan_tx1.ch && !IS_ERR(dev->sc_chan_tx1.ch))
		mbox_free_channel(dev->sc_chan_tx1.ch);
	if (dev->sc_chan_rx.ch && !IS_ERR(dev->sc_chan_rx.ch))
		mbox_free_channel(dev->sc_chan_rx.ch);

	dev->sc_chan_tx0.ch = NULL;
	dev->sc_chan_tx1.ch = NULL;
	dev->sc_chan_rx.ch = NULL;
}

static void vpu_rx_callback(struct mbox_client *c, void *msg)
{
	struct vpu_sc_chan *sc_chan = container_of(c, struct vpu_sc_chan, cl);
	struct vpu_dev *dev =
		container_of(sc_chan, struct vpu_dev, sc_chan_rx);

	if (&dev->mu_msg_fifo == NULL) {
		vpu_err("&dev->mu_msg_fifo == NULL\n");
		return;
	}

	vpu_mu_inq_msg(dev, msg);
}

static int vpu_rtx_channel_request(struct vpu_sc_chan *sc_chan, bool block, struct vpu_dev *vdev)
{
	struct device *dev = &vdev->plat_dev->dev;
	struct mbox_client *cl;
	int ret = 0;

	if (sc_chan->ch && !IS_ERR(sc_chan->ch)) {
		vpu_dbg(LVL_WARN, "vpu_sc_chan(%s) has been requested\n",
			sc_chan->name);
		return 0;
	}

	/* Normally, use block mode to ensure msg be writed correctly.
	 * Firmware do not separate read tx1 channel until receive tx0.
	 * Hence, when use tx1 must combine with tx0, and flow is tx1 -> tx0 to
	 * ensure firmware read tx1 correctly. Due to tx1 write first, tx1 shall
	 * use non-block mode, otherwise always wait timeout.
	 * In addition, tx0 use block mode then also can ensure tx1 msg be
	 * writed correctly as tx0 and tx1 are considered as a whole in this
	 * case.
	 */

	/* core device id count from 0, mu count from 1 */

	cl = &sc_chan->cl;
	cl->dev = dev;
	if (block) {
		cl->tx_block = true;
		cl->tx_tout = 1000;
	} else {
		cl->tx_block = false;
	}
	cl->knows_txdone = false;
	cl->rx_callback = vpu_rx_callback;

	sc_chan->ch = mbox_request_channel_byname(cl, sc_chan->name);
	if (IS_ERR(sc_chan->ch)) {
		ret = PTR_ERR(sc_chan->ch);
		if (ret != -EPROBE_DEFER)
			vpu_err("Failed to request mbox chan %s ret %d\n",
				sc_chan->name, ret);
	}

	return ret;
}

static int vpu_rtx_channel_init(struct vpu_dev *vdev)
{
	int ret = 0;

	vdev->sc_chan_tx0.dev = vdev;
	scnprintf(vdev->sc_chan_tx0.name, sizeof(vdev->sc_chan_tx0.name) - 1,
		  "tx0");
	ret = vpu_rtx_channel_request(&vdev->sc_chan_tx0, true, vdev);
	if (ret)
		goto err;

	vdev->sc_chan_tx1.dev = vdev;
	scnprintf(vdev->sc_chan_tx1.name, sizeof(vdev->sc_chan_tx1.name) - 1,
		  "tx1");
	ret = vpu_rtx_channel_request(&vdev->sc_chan_tx1, false, vdev);
	if (ret)
		goto err;

	vdev->sc_chan_rx.dev = vdev;
	scnprintf(vdev->sc_chan_rx.name, sizeof(vdev->sc_chan_rx.name) - 1,
		  "rx");
	ret = vpu_rtx_channel_request(&vdev->sc_chan_rx, true, vdev);
	if (ret)
		goto err;

	return ret;

err:
	vpu_mbox_free(vdev);

	return ret;
}

int vpu_mu_request(struct vpu_dev *dev)
{
	int ret = 0;

	if (!dev->vpu_mu_init) {
		ret = vpu_rtx_channel_init(dev);
		if (!ret)
			dev->vpu_mu_init = TRUE;
		else
			vpu_dbg(LVL_WARN, "warning: init rtx channel failed, ret: %d\n",
				ret);
	}

	return ret;
}

void vpu_mu_free(struct vpu_dev *dev)
{
	if (dev->vpu_mu_init) {
		vpu_mbox_free(dev);
		dev->vpu_mu_init = FALSE;
	}
}

void vpu_mu_send_msg(struct vpu_dev *dev, MSG_Type type, u32 value)
{
	/* Ensure firmware read tx1 correctly when receive tx0,
	 * need to write tx1 first.
	 */

	if (value != 0xffff)
		mbox_send_message(dev->sc_chan_tx1.ch, &value);
	if (type != 0xffff)
		mbox_send_message(dev->sc_chan_tx0.ch, &type);
}

u_int32 vpu_mu_receive_msg(struct vpu_dev *dev, void *msg)
{
	int ret = 0;

	if (kfifo_len(&dev->mu_msg_fifo) >= sizeof(u_int32)) {
		ret = kfifo_out(&dev->mu_msg_fifo, msg, sizeof(u_int32));
		if (ret != sizeof(u_int32))
			vpu_err("error: kfifo_out mu msg failed\n, ret=%d\n",
				ret);
	} else {
		ret = kfifo_len(&dev->mu_msg_fifo);
		if (ret)
			vpu_err("error: broken msg\n");
		ret = 0;
	}

	return ret;
}

static int vpu_sc_get_fuse(struct imx_sc_ipc *ipc, u_int32 word, u_int32 *fuse)
{
	struct vpu_imx_sc_msg_misc msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_MISC;
	hdr->func = IMX_SC_MISC_FUNC_OTP_FUSE_READ;
	hdr->size = 2;

	msg.word = word;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	*fuse = msg.word;

	return ret;
}

int vpu_sc_check_fuse(struct vpu_dev *dev, struct vpu_v4l2_fmt *pformat_table,
		      u_int32 table_size)
{
	u_int32 fuse = 0xff;
	u_int32 val;
	u_int32 i;
	int ret = 0;
	struct imx_sc_ipc *ipcHandle;

	ret = imx_scu_get_handle(&ipcHandle);
	if (ret) {
		vpu_err("error: scu get handle fail: %d\n", ret);
		return ret;
	}

	vpu_sc_get_fuse(ipcHandle, VPU_DISABLE_BITS, &fuse);
	if (ret) {
		vpu_dbg(LVL_WARN, "warning: read value fail: %d\n", ret);
		return ret;
	}

	val = (fuse >> VPU_IMX_DECODER_FUSE_OFFSET) & 0x3UL;
	if (val == 0x1UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == VPU_PIX_FMT_HEVC)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H265 is disabled\n");
	} else if (val == 0x2UL) {
		for (i = 0; i < table_size; i++)
			if (pformat_table[i].fourcc == V4L2_PIX_FMT_H264)
				pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "H264 is disabled\n");
	} else if (val == 0x3UL) {
		for (i = 0; i < table_size; i++)
			pformat_table[i].disable = 1;
		vpu_dbg(LVL_WARN, "All decoder disabled\n");
	}

	return ret;
}

void vpu_mu_enable_rx(struct vpu_dev *dev)
{

}
