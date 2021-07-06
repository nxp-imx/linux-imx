// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 * Author: Alice Guo <alice.guo@nxp.com>
 */

#include <linux/completion.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/firmware/imx/s400-api.h>
#include <linux/init.h>
#include <linux/mailbox_client.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define MSG_TAG(x)		(((x) & 0xff000000) >> 24)
#define MSG_COMMAND(x)		(((x) & 0x00ff0000) >> 16)
#define MSG_SIZE(x)		(((x) & 0x0000ff00) >> 8)
#define MSG_VER(x)		((x) & 0x000000ff)
#define RES_STATUS(x)		((x) & 0x000000ff)

struct imx_s400_api *s400_api_export;

int get_imx_s400_api(struct imx_s400_api **export)
{
	if (!s400_api_export)
		return -EPROBE_DEFER;

	*export = s400_api_export;
	return 0;
}
EXPORT_SYMBOL_GPL(get_imx_s400_api);

static int s400_api_send_command(struct imx_s400_api *s400_api)
{
	int err;

	err = mbox_send_message(s400_api->tx_chan, &s400_api->tx_msg);
	if (err < 0)
		return err;

	return 0;
}

static int read_otp_uniq_id(struct imx_s400_api *s400_api, u32 *value)
{
	unsigned int tag, command, size, ver, status;

	tag = MSG_TAG(s400_api->rx_msg.header);
	command = MSG_COMMAND(s400_api->rx_msg.header);
	size = MSG_SIZE(s400_api->rx_msg.header);
	ver = MSG_VER(s400_api->rx_msg.header);
	status = RES_STATUS(s400_api->rx_msg.data[0]);

	if (tag == 0xe1 && command == S400_READ_FUSE_REQ &&
	    size == 0x07 && ver == 0x06 && status == S400_SUCCESS_IND) {
		value[0] = s400_api->rx_msg.data[1];
		value[1] = s400_api->rx_msg.data[2];
		value[2] = s400_api->rx_msg.data[3];
		value[3] = s400_api->rx_msg.data[4];
		return 0;
	}

	return -EINVAL;
}

static int read_fuse_word(struct imx_s400_api *s400_api, u32 *value)
{
	unsigned int tag, command, size, ver, status;

	tag = MSG_TAG(s400_api->rx_msg.header);
	command = MSG_COMMAND(s400_api->rx_msg.header);
	size = MSG_SIZE(s400_api->rx_msg.header);
	ver = MSG_VER(s400_api->rx_msg.header);
	status = RES_STATUS(s400_api->rx_msg.data[0]);

	if (tag == 0xe1 && command == S400_READ_FUSE_REQ &&
	    size == 0x03 && ver == 0x06 && status == S400_SUCCESS_IND) {
		value[0] = s400_api->rx_msg.data[1];
		return 0;
	}

	return -EINVAL;
}

static int read_common_fuse(struct imx_s400_api *s400_api, u32 *value)
{
	unsigned int size = MSG_SIZE(s400_api->tx_msg.header);
	unsigned int wait, fuse_id;
	int err;

	err = -EINVAL;
	if (size == 2) {
		err = s400_api_send_command(s400_api);
		if (err < 0)
			return err;

		wait = msecs_to_jiffies(1000);
		if (!wait_for_completion_timeout(&s400_api->done, wait))
			return -ETIMEDOUT;

		fuse_id = s400_api->tx_msg.data[0] & 0xff;
		switch (fuse_id) {
		case OTP_UNIQ_ID:
			err = read_otp_uniq_id(s400_api, value);
			break;
		default:
			err = read_fuse_word(s400_api, value);
			break;
		}
	}

	return err;
}

int imx_s400_api_call(struct imx_s400_api *s400_api, void *value)
{
	unsigned int tag, command, ver;
	int err;

	err = -EINVAL;
	tag = MSG_TAG(s400_api->tx_msg.header);
	command = MSG_COMMAND(s400_api->tx_msg.header);
	ver = MSG_VER(s400_api->tx_msg.header);

	if (tag == 0x17 && ver == 0x06) {
		reinit_completion(&s400_api->done);

		switch (command) {
		case S400_READ_FUSE_REQ:
			err = read_common_fuse(s400_api, value);
			break;
		default:
			return -EINVAL;
		}
	}

	return err;
}
EXPORT_SYMBOL_GPL(imx_s400_api_call);

static void s400_api_receive_message(struct mbox_client *rx_cl, void *mssg)
{
	struct imx_s400_api *s400_api;

	s400_api = container_of(rx_cl, struct imx_s400_api, rx_cl);

	spin_lock(&s400_api->lock);
	if (mssg) {
		s400_api->rx_msg = *(struct s400_api_msg *)mssg;
		complete(&s400_api->done);
	}
	spin_unlock(&s400_api->lock);
}

struct device *imx_soc_device_register(void)
{
	struct soc_device_attribute *attr;
	struct soc_device *dev;
	u32 v[4];
	int err;

	s400_api_export->tx_msg.header = 0x17970206;
	s400_api_export->tx_msg.data[0] = 0x1;
	err = imx_s400_api_call(s400_api_export, v);
	if (err)
		return NULL;

	attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return NULL;

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err) {
		kfree(attr);
		return NULL;
	}
	attr->family = kasprintf(GFP_KERNEL, "Freescale i.MX");
	attr->revision = kasprintf(GFP_KERNEL, "1.0");
	attr->serial_number = kasprintf(GFP_KERNEL, "%016llX", (u64)v[3] << 32 | v[0]);
	attr->soc_id = kasprintf(GFP_KERNEL, "i.MX8ULP");

	dev = soc_device_register(attr);
	if (IS_ERR(dev)) {
		kfree(attr->soc_id);
		kfree(attr->serial_number);
		kfree(attr->revision);
		kfree(attr->family);
		kfree(attr->machine);
		kfree(attr);
		return ERR_CAST(dev);
	}

	return soc_device_to_device(dev);
}

static int s400_api_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct of_phandle_args mboxargs;
	struct imx_s400_api *s400_api;
	struct mbox_client cl;
	struct device *soc;
	int err;

	if (of_parse_phandle_with_args(dev->of_node, "mboxes", "#mbox-cells",
				       0, &mboxargs))
		return -ENODEV;

	if (!of_device_is_compatible(mboxargs.np, "fsl,imx8ulp-mu-s4"))
		return -ENODEV;

	s400_api = devm_kzalloc(dev, sizeof(*s400_api), GFP_KERNEL);
	if (!s400_api)
		return -ENOMEM;

	cl.dev		= dev;
	cl.tx_block	= false;
	cl.knows_txdone	= true;
	cl.rx_callback	= s400_api_receive_message;

	s400_api->tx_cl = cl;
	s400_api->rx_cl = cl;

	s400_api->tx_chan = mbox_request_channel_byname(&s400_api->tx_cl, "tx");
	if (IS_ERR(s400_api->tx_chan)) {
		err = PTR_ERR(s400_api->tx_chan);
		dev_err(dev, "failed to request tx chan: %d\n", err);
		return err;
	}

	s400_api->rx_chan = mbox_request_channel_byname(&s400_api->rx_cl, "rx");
	if (IS_ERR(s400_api->rx_chan)) {
		err = PTR_ERR(s400_api->rx_chan);
		dev_err(dev, "failed to request rx chan: %d\n", err);
		goto free_tx;
	}

	init_completion(&s400_api->done);
	spin_lock_init(&s400_api->lock);

	s400_api_export = s400_api;

	soc = imx_soc_device_register();
	if (IS_ERR(soc)) {
		pr_err("failed to register SoC device: %ld\n", PTR_ERR(soc));
		return PTR_ERR(soc);
	}

	dev_set_drvdata(dev, s400_api);
	return devm_of_platform_populate(dev);

free_tx:
	mbox_free_channel(s400_api->tx_chan);

	return err;
}

static int s400_api_remove(struct platform_device *pdev)
{
	struct imx_s400_api *s400_api;

	s400_api = dev_get_drvdata(&pdev->dev);
	mbox_free_channel(s400_api->tx_chan);
	mbox_free_channel(s400_api->rx_chan);

	return 0;
}

static const struct of_device_id s400_api_match[] = {
	{ .compatible = "fsl,imx8ulp-s400", },
	{},
};

static struct platform_driver s400_api_driver = {
	.driver = {
		.name = "fsl-s400-api",
		.of_match_table = s400_api_match,
	},
	.probe = s400_api_probe,
	.remove = s400_api_remove,
};
module_platform_driver(s400_api_driver);

MODULE_AUTHOR("Alice Guo <alice.guo@nxp.com>");
MODULE_DESCRIPTION("S400 Baseline API");
MODULE_LICENSE("GPL v2");
