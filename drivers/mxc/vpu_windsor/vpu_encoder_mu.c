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
 * @file vpu_encoder_mu.h
 *
 */

#include "vpu_encoder_mu.h"

static void vpu_enc_mbox_free(struct core_device *core_dev)
{
	if (core_dev->sc_chan_tx0.ch && !IS_ERR(core_dev->sc_chan_tx0.ch))
		mbox_free_channel(core_dev->sc_chan_tx0.ch);
	if (core_dev->sc_chan_tx1.ch && !IS_ERR(core_dev->sc_chan_tx1.ch))
		mbox_free_channel(core_dev->sc_chan_tx1.ch);
	if (core_dev->sc_chan_rx.ch && !IS_ERR(core_dev->sc_chan_rx.ch))
		mbox_free_channel(core_dev->sc_chan_rx.ch);

	core_dev->sc_chan_tx0.ch = NULL;
	core_dev->sc_chan_tx1.ch = NULL;
	core_dev->sc_chan_rx.ch = NULL;
}

static void vpu_enc_mu_inq_msg(struct core_device *core_dev, void *msg)
{
	if (&core_dev->mu_msg_fifo == NULL) {
		vpu_err("mu_msg_fifo is NULL\n");
		return;
	}

	if (kfifo_in(&core_dev->mu_msg_fifo, msg, sizeof(u_int32)) != sizeof(u_int32)) {
		vpu_err("No memory for mu msg fifo\n");
		return;
	}

	queue_work(core_dev->workqueue, &core_dev->msg_work);
}

static void vpu_enc_rx_callback(struct mbox_client *c, void *msg)
{
	struct vpu_sc_chan *sc_chan = container_of(c, struct vpu_sc_chan, cl);
	struct core_device *dev =
		container_of(sc_chan, struct core_device, sc_chan_rx);

	vpu_enc_mu_inq_msg(dev, msg);
}

static int vpu_enc_rtx_channel_request(struct vpu_sc_chan *sc_chan, bool block)
{
	struct device *dev = sc_chan->dev->generic_dev;
	struct mbox_client *cl;
	int ret = 0;

	if (sc_chan->ch && !IS_ERR(sc_chan->ch)) {
		vpu_dbg(LVL_WARN, "vpu_sc_chan(%s) has been requested\n",
			sc_chan->name);
		return 0;
	}

	cl = &sc_chan->cl;
	cl->dev = dev;
	if (block) {
		cl->tx_block = true;
		cl->tx_tout = 1000;
	} else {
		cl->tx_block = false;
	}
	cl->knows_txdone = false;
	cl->rx_callback = vpu_enc_rx_callback;

	sc_chan->ch = mbox_request_channel_byname(cl, sc_chan->name);
	if (IS_ERR(sc_chan->ch)) {
		ret = PTR_ERR(sc_chan->ch);
		if (ret != -EPROBE_DEFER)
			vpu_err("Failed to request mbox chan %s, ret: %d\n",
				sc_chan->name, ret);
	}

	return ret;
}

static int vpu_enc_rtx_channel_init(struct core_device *core_dev)
{
	int ret = 0;

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

	core_dev->sc_chan_tx0.dev = core_dev;
	scnprintf(core_dev->sc_chan_tx0.name, sizeof(core_dev->sc_chan_tx0.name) - 1,
		  "enc%d_tx0", core_dev->id + 1);
	ret = vpu_enc_rtx_channel_request(&core_dev->sc_chan_tx0, true);
	if (ret)
		goto err;

	core_dev->sc_chan_tx1.dev = core_dev;
	scnprintf(core_dev->sc_chan_tx1.name, sizeof(core_dev->sc_chan_tx1.name) - 1,
		  "enc%d_tx1", core_dev->id + 1);
	ret = vpu_enc_rtx_channel_request(&core_dev->sc_chan_tx1, false);
	if (ret)
		goto err;

	core_dev->sc_chan_rx.dev = core_dev;
	scnprintf(core_dev->sc_chan_rx.name, sizeof(core_dev->sc_chan_rx.name) - 1,
		  "enc%d_rx", core_dev->id + 1);
	ret = vpu_enc_rtx_channel_request(&core_dev->sc_chan_rx, true);
	if (ret)
		goto err;

	return ret;

err:
	vpu_enc_mbox_free(core_dev);

	return ret;
}

int vpu_enc_mu_request(struct core_device *core_dev)
{
	int ret = 0;

	if (!core_dev->vpu_mu_init) {
		ret = vpu_enc_rtx_channel_init(core_dev);
		if (!ret)
			core_dev->vpu_mu_init = TRUE;
		else
			vpu_dbg(LVL_WARN, "warning: %s init rtx channel failed, ret: %d\n",
				core_dev->name, ret);
	}

	return ret;
}

void vpu_enc_mu_free(struct core_device *core_dev)
{
	if (core_dev->vpu_mu_init) {
		vpu_enc_mbox_free(core_dev);
		core_dev->vpu_mu_init = FALSE;
	}
}

void vpu_enc_mu_send_msg(struct core_device *core_dev, MSG_Type type, u_int32 value)
{
	/* Ensure firmware read tx1 correctly when receive tx0,
	 * need to write tx1 first.
	 */

	if (value != 0xffff)
		mbox_send_message(core_dev->sc_chan_tx1.ch, &value);
	if (type != 0xffff)
		mbox_send_message(core_dev->sc_chan_tx0.ch, &type);
}

u_int32 vpu_enc_mu_receive_msg(struct core_device *core_dev, void *msg)
{
	int ret = 0;

	if (kfifo_len(&core_dev->mu_msg_fifo) >= sizeof(u_int32)) {
		ret = kfifo_out(&core_dev->mu_msg_fifo, msg, sizeof(u_int32));
		if (ret != sizeof(u_int32))
			vpu_err("error: kfifo_out mu msg failed\n, ret=%d\n",
				ret);
	} else {
		ret = kfifo_len(&core_dev->mu_msg_fifo);
	}

	return ret;
}

static int vpu_enc_sc_get_fuse(struct imx_sc_ipc *ipc, u32 word, u32 *fuse)
{
	struct vpu_sc_msg_misc msg;
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

int vpu_enc_sc_check_fuse(void)
{
	u_int32 fuse = 0xffff;
	int ret = 0;
	struct imx_sc_ipc *ipcHandle;

	ret = imx_scu_get_handle(&ipcHandle);
	if (ret) {
		vpu_err("error: scu get handle fail: %d\n", ret);
		return -EINVAL;
	}

	ret = vpu_enc_sc_get_fuse(ipcHandle, VPU_DISABLE_BITS, &fuse);
	if (ret) {
		vpu_dbg(LVL_WARN, "warning: read value fail: %d\n", ret);
		return ret;
	}

	if (fuse & VPU_ENCODER_MASK) {
		vpu_err("warning: VPU Encoder is disabled\n");
		return -EINVAL;
	}

	return ret;
}

void vpu_enc_mu_enable_rx(struct core_device *core_dev)
{

}
