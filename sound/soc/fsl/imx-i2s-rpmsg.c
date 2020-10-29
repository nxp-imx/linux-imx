// SPDX-License-Identifier: GPL-2.0+
// Copyright 2020 NXP

#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/imx_rpmsg.h>

#include "fsl_rpmsg_i2s.h"

extern struct i2s_info *i2s_info_g;

static int i2s_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct i2s_rpmsg_r *msg = (struct i2s_rpmsg_r *)data;
	struct i2s_rpmsg *rpmsg;
	unsigned long flags;

	dev_dbg(&rpdev->dev, "get from%d: cmd:%d. %d\n",
				src, msg->header.cmd, msg->param.resp);

	if (msg->header.type == I2S_TYPE_C) {
		if (msg->header.cmd == I2S_TX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[0], flags);
			rpmsg = &i2s_info_g->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];

			if (msg->header.major == 1 && msg->header.minor == 2)
				rpmsg->recv_msg.param.buffer_tail =
							msg->param.buffer_tail;
			else
				rpmsg->recv_msg.param.buffer_tail++;

			rpmsg->recv_msg.param.buffer_tail %=
						i2s_info_g->num_period[0];

			spin_unlock_irqrestore(&i2s_info_g->lock[0], flags);
			i2s_info_g->callback[0](i2s_info_g->callback_param[0]);

		} else if (msg->header.cmd == I2S_RX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[1], flags);
			rpmsg = &i2s_info_g->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

			if (msg->header.major == 1 && msg->header.minor == 2)
				rpmsg->recv_msg.param.buffer_tail =
							msg->param.buffer_tail;
			else
				rpmsg->recv_msg.param.buffer_tail++;

			rpmsg->recv_msg.param.buffer_tail %=
						i2s_info_g->num_period[1];
			spin_unlock_irqrestore(&i2s_info_g->lock[1], flags);
			i2s_info_g->callback[1](i2s_info_g->callback_param[1]);
		}
	}

	if (msg->header.type == I2S_TYPE_B) {
		memcpy(&i2s_info_g->recv_msg, msg, sizeof(struct i2s_rpmsg_r));
		complete(&i2s_info_g->cmd_complete);
	}

	return 0;
}

static int i2s_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct platform_device *codec_pdev;
	struct fsl_rpmsg_i2s *rpmsg_i2s = NULL;
	struct fsl_rpmsg_codec  rpmsg_codec[3];
	int ret;

	if (!i2s_info_g)
		return 0;

	i2s_info_g->rpdev = rpdev;

	init_completion(&i2s_info_g->cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	rpmsg_i2s = container_of(i2s_info_g, struct fsl_rpmsg_i2s, i2s_info);

	if (rpmsg_i2s->codec_in_dt)
		return 0;

	if (rpmsg_i2s->codec_wm8960) {
		rpmsg_codec[0].audioindex = rpmsg_i2s->codec_wm8960 >> 16;
		rpmsg_codec[0].shared_lrclk = true;
		rpmsg_codec[0].capless = false;
		codec_pdev = platform_device_register_data(
					&rpmsg_i2s->pdev->dev,
					RPMSG_CODEC_DRV_NAME_WM8960,
					PLATFORM_DEVID_NONE,
					&rpmsg_codec[0], sizeof(struct fsl_rpmsg_codec));
		if (IS_ERR(codec_pdev)) {
			dev_err(&rpdev->dev,
				"failed to register rpmsg audio codec\n");
			ret = PTR_ERR(codec_pdev);
			return ret;
		}
	}

	if (rpmsg_i2s->codec_cs42888) {
		rpmsg_codec[1].audioindex = rpmsg_i2s->codec_cs42888 >> 16;
		strcpy(rpmsg_codec[1].name, "cs42888");
		rpmsg_codec[1].num_adcs = 2;

		codec_pdev = platform_device_register_data(
					&rpmsg_i2s->pdev->dev,
					RPMSG_CODEC_DRV_NAME_CS42888,
					PLATFORM_DEVID_NONE,
					&rpmsg_codec[1], sizeof(struct fsl_rpmsg_codec));
		if (IS_ERR(codec_pdev)) {
			dev_err(&rpdev->dev,
				"failed to register rpmsg audio codec\n");
			ret = PTR_ERR(codec_pdev);
			return ret;
		}
	}

	if (rpmsg_i2s->codec_ak4497) {
		rpmsg_codec[2].audioindex = rpmsg_i2s->codec_ak4497 >> 16;
		codec_pdev = platform_device_register_data(
					&rpmsg_i2s->pdev->dev,
					RPMSG_CODEC_DRV_NAME_AK4497,
					PLATFORM_DEVID_NONE,
					&rpmsg_codec[2], sizeof(struct fsl_rpmsg_codec));
		if (IS_ERR(codec_pdev)) {
			dev_err(&rpdev->dev,
				"failed to register rpmsg audio codec\n");
			ret = PTR_ERR(codec_pdev);
			return ret;
		}
	}

	return 0;
}

static void i2s_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "i2s rpmsg driver is removed\n");
}

static struct rpmsg_device_id i2s_rpmsg_id_table[] = {
	{ .name	= "rpmsg-audio-channel" },
	{ },
};

static struct rpmsg_driver i2s_rpmsg_driver = {
	.drv.name	= "i2s_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= i2s_rpmsg_id_table,
	.probe		= i2s_rpmsg_probe,
	.callback	= i2s_rpmsg_cb,
	.remove		= i2s_rpmsg_remove,
};

static int __init i2s_rpmsg_init(void)
{
	return register_rpmsg_driver(&i2s_rpmsg_driver);
}

static void __exit i2s_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&i2s_rpmsg_driver);
}
module_init(i2s_rpmsg_init);
module_exit(i2s_rpmsg_exit);

MODULE_LICENSE("GPL v2");
