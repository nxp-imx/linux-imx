// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 *  Author: Robert Chiras <robert.chiras@nxp.com>
 *
 * Implementation of the DISPLAYMIX CM0+ comms using MUs (client side).
 *
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/imx/disp-mu.h>

static DEFINE_MUTEX(cl_mutex);

static struct disp_mu_ipc *disp_ipc_handle;

static void imx_dispmu_rx_callback(struct mbox_client *c, void *msg)
{
	struct disp_mu_chan *chan = container_of(c, struct disp_mu_chan, cl);
	struct disp_mu_dev *mu = chan->mu_dev;

	/* This is an unsolicited message, just redirect it*/
	if (!mu->need_reply) {
		if (!mu->cl || !mu->cl->rx_callback) {
			dev_err(c->dev, "RX callback received, but no registered client!\n");
			return;
		}

		mu->cl->rx_callback(mu->cl, msg);
		return;
	}

	/* This is a reply to a message request, set the reply */
	memcpy((void *)(&mu->reply[0]), msg, ARRAY_SIZE(mu->reply) * sizeof(u32));
	complete(&mu->done);
}

int imx_dispmu_client_register(struct disp_mu_client *cl)
{
	struct disp_mu_ipc *ipc = disp_ipc_handle;
	struct disp_mu_dev *mu;

	if (WARN_ON(!ipc))
		return -EPROBE_DEFER;

	if (!cl->rx_callback)
		return -EINVAL;

	guard(mutex)(&cl_mutex);

	mu = &ipc->mu_dev;

	if (mu->cl)
		return -EBUSY;

	mu->cl = cl;
	cl->mu = mu;

	return 0;
}
EXPORT_SYMBOL(imx_dispmu_client_register);

int imx_dispmu_client_unregister(struct disp_mu_client *cl)
{
	struct disp_mu_ipc *ipc = disp_ipc_handle;
	struct disp_mu_dev *mu;
	int ret = 0;

	if (WARN_ON(!ipc))
		return -EPROBE_DEFER;

	guard(mutex)(&cl_mutex);

	mu = &ipc->mu_dev;

	if (!mu->cl)
		return ret;

	if (mu->cl != cl)
		return -EBUSY;

	mu->cl = NULL;
	cl->mu = NULL;

	return ret;
}
EXPORT_SYMBOL(imx_dispmu_client_unregister);

int imx_dispmu_send_msg(struct disp_mu_client *cl, void *msg, void **reply)
{
	struct disp_mu_dev *mu = cl->mu;
	struct disp_mu_chan *chan;
	struct disp_mu_ipc *ipc;
	int ret = 0;

	if (!mu)
		return -ENODEV;

	if (!msg)
		return -EINVAL;

	ipc = container_of(mu, typeof(*ipc), mu_dev);

	guard(mutex)(&mu->lock);
	reinit_completion(&mu->done);

	chan = &mu->chans[MU_CHAN_TX];

	mu->need_reply = false;
	if (reply) {
		mu->need_reply = true;
		*reply = &mu->reply[0];
	}

	ret = mbox_send_message(chan->ch, msg);

	/* Successful send means a return code greater than 0 */
	if (ret <= 0)
		return ret;

	/* Wait for other end to send us a reply */
	if (mu->need_reply) {
		if (!wait_for_completion_timeout(&mu->done,
						 MAX_RX_TIMEOUT)) {
			dev_err(ipc->dev, "MSG sent, no response\n");
			return -ETIMEDOUT;
		}
	}

	return ret;
}
EXPORT_SYMBOL(imx_dispmu_send_msg);

static int imx_dispmu_init_mu_dev(struct disp_mu_ipc *ipc)
{
	struct disp_mu_dev *mu_dev = &ipc->mu_dev;
	struct device *dev = ipc->dev;
	struct disp_mu_chan *mu_chan;
	struct mbox_client *cl;
	const char *chan_name;
	int ret;
	int i;

	for (i = 0; i < DISP_MU_CHAN_NUM; i++) {
		chan_name = i ? MBOX_RX_NAME : MBOX_TX_NAME;

		mu_chan = &mu_dev->chans[i];
		cl = &mu_chan->cl;
		cl->dev = dev;
		cl->tx_block = false;
		cl->knows_txdone = false;
		cl->rx_callback = imx_dispmu_rx_callback;

		mu_chan->mu_dev = mu_dev;
		mu_chan->idx = i;
		mu_chan->ch = mbox_request_channel_byname(cl, chan_name);
		if (IS_ERR(mu_chan->ch)) {
			ret = PTR_ERR(mu_chan->ch);
			return dev_err_probe(dev, ret,
					     "Failed to request mbox chan %s\n",
					     chan_name);
		}

		dev_dbg(ipc->dev, "request mbox chan %s\n", chan_name);
	}

	mutex_init(&mu_dev->lock);
	init_completion(&mu_dev->done);

	return 0;
}

static int imx_dispmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct disp_mu_ipc *disp_ipc;
	int ret;

	disp_ipc = devm_kzalloc(dev, sizeof(*disp_ipc), GFP_KERNEL);
	if (!disp_ipc)
		return -ENOMEM;

	disp_ipc->dev = dev;
	dev_set_drvdata(dev, disp_ipc);

	ret = imx_dispmu_init_mu_dev(disp_ipc);
	if (ret)
		return ret;

	disp_ipc_handle = disp_ipc;

	pm_runtime_enable(dev);
	pm_runtime_resume_and_get(dev);

	dev_info(dev, "i.MX DISP-MU initialized\n");

	return devm_of_platform_populate(dev);
}

static void imx_dispmu_remove(struct platform_device *pdev)
{
	struct disp_mu_ipc *ipc = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < DISP_MU_CHAN_NUM; i++)
		mbox_free_channel(ipc->mu_dev.chans[i].ch);

	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id imx_dispmu_match[] = {
	{ .compatible = "nxp,imx-disp-mu", },
	{ /* Sentinel */ }
};

static struct platform_driver imx_dispmu_driver = {
	.driver = {
		.name = "imx-disp-mu",
		.of_match_table = imx_dispmu_match,
	},
	.probe = imx_dispmu_probe,
	.remove = imx_dispmu_remove,
};

static int __init imx_dispmu_driver_init(void)
{
	return platform_driver_register(&imx_dispmu_driver);
}
subsys_initcall_sync(imx_dispmu_driver_init);

MODULE_AUTHOR("Robert Chiras <robert.chiras@nxp.com>");
MODULE_DESCRIPTION("IMX DISPMIX MU protocol driver");
MODULE_LICENSE("GPL");
