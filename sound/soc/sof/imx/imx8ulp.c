// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright 2021 NXP
//
// Author: Peng Zhang <peng.zhang_8@nxp.com>
//
// Hardware interface for audio DSP on i.MX8ULP

#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_domain.h>
#include <linux/mfd/syscon.h>
#include <linux/of_reserved_mem.h>

#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/dsp.h>

#include <linux/firmware/imx/svc/misc.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include "../ops.h"
#include "imx-common.h"

/* SIM Domain register */
#define FSL_SIP_HIFI_XRDC                 0xc200000e
struct arm_smccc_res smc_resource;

#define SYSCTRL0		0x8
#define EXECUTE_BIT		BIT(13)
#define RESET_BIT		BIT(16)
#define HIFI4_CLK_BIT	BIT(17)
#define PB_CLK_BIT		BIT(18)
#define PLAT_CLK_BIT	BIT(19)
#define DEBUG_LOGIC_BIT	BIT(25)

#define MBOX_OFFSET	0x800000
#define MBOX_SIZE	0x1000

/* TODO define clk */
#define IMX8ULP_DSP_CLK_NUM	4
static const char *imx8ulp_dsp_clks_names[IMX8ULP_DSP_CLK_NUM] = {
	/* DSP clocks */
	"core", "pbclk", "nic", "mu_b",
};

#define IMX8ULP_DAI_CLK_NUM	12
static const char *imx8ulp_dai_clks_names[IMX8ULP_DAI_CLK_NUM] = {
	"bus", "mclk0", "mclk1",
	"edma-mp-clk",
	"edma2-chan0-clk", "edma2-chan1-clk",
	"edma2-chan2-clk", "edma2-chan3-clk",
	"edma2-chan4-clk", "edma2-chan5-clk",
	"edma2-chan6-clk", "edma2-chan7-clk",
};

struct imx8ulp_priv {
	struct device *dev;
	struct snd_sof_dev *sdev;

	/* DSP IPC handler */
	struct imx_dsp_ipc *dsp_ipc;
	struct platform_device *ipc_dev;

	/* Power domain handling */
	int num_domains;
	struct device **pd_dev;
	struct device_link **link;

	struct regmap *regmap_sim;
	struct clk *dsp_clks[IMX8ULP_DSP_CLK_NUM];
	struct clk *dai_clks[IMX8ULP_DAI_CLK_NUM];
};

/* System integration module(SIM) control dsp configurtion */
int imx8ulp_configure_sim_lpav(struct imx8ulp_priv *priv)
{
	priv->regmap_sim = syscon_regmap_lookup_by_compatible("nxp,imx8ulp-avd-sim");
	if (IS_ERR(priv->regmap_sim))
		return -EPROBE_DEFER;
	return 0;
}

static void imx8ulp_sim_lpav_start(struct imx8ulp_priv *priv)
{
	/* Controls the HiFi4 DSP Reset: 1 in reset, 0 out of reset */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, RESET_BIT, 0);
	/* Reset HiFi4 DSP Debug logic: 1 reset, 0 not set */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, DEBUG_LOGIC_BIT, 0);
	/* Stall HIFI4 DSP Execution: 1 stall, 0 not stall */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, EXECUTE_BIT, 0);
}

static int imx8ulp_init_clocks(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8ulp_priv *priv = (struct imx8ulp_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8ULP_DSP_CLK_NUM; i++) {
		priv->dsp_clks[i] = devm_clk_get(priv->dev, imx8ulp_dsp_clks_names[i]);
		if (IS_ERR(priv->dsp_clks[i]))
			return PTR_ERR(priv->dsp_clks[i]);
	}

	for (i = 0; i < IMX8ULP_DAI_CLK_NUM; i++)
		priv->dai_clks[i] = devm_clk_get_optional(priv->dev, imx8ulp_dai_clks_names[i]);

	return 0;
}

static int imx8ulp_prepare_clocks(struct snd_sof_dev *sdev)
{
	int i, j, ret;
	struct imx8ulp_priv *priv = (struct imx8ulp_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8ULP_DSP_CLK_NUM; i++) {
		ret = clk_prepare_enable(priv->dsp_clks[i]);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable clk %s\n",
				imx8ulp_dsp_clks_names[i]);
			goto err_dsp_clks;
		}
	}

	for (j = 0; j < IMX8ULP_DAI_CLK_NUM; j++) {
		ret = clk_prepare_enable(priv->dai_clks[j]);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable clk %s\n",
				imx8ulp_dai_clks_names[j]);
			goto err_dai_clks;
		}
	}

	return 0;

err_dai_clks:
	while (--j >= 0)
		clk_disable_unprepare(priv->dai_clks[j]);

err_dsp_clks:
	while (--i >= 0)
		clk_disable_unprepare(priv->dsp_clks[i]);

	return ret;
}

static void imx8ulp_disable_clocks(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8ulp_priv *priv = (struct imx8ulp_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8ULP_DSP_CLK_NUM; i++)
		clk_disable_unprepare(priv->dsp_clks[i]);

	for (i = 0; i < IMX8ULP_DAI_CLK_NUM; i++)
		clk_disable_unprepare(priv->dai_clks[i]);
}

static void imx8ulp_get_reply(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc_msg *msg = sdev->msg;
	struct sof_ipc_reply reply;
	int ret = 0;

	if (!msg) {
		dev_warn(sdev->dev, "unexpected ipc interrupt\n");
		return;
	}

	/* get reply */
	sof_mailbox_read(sdev, sdev->host_box.offset, &reply, sizeof(reply));

	if (reply.error < 0) {
		memcpy(msg->reply_data, &reply, sizeof(reply));
		ret = reply.error;
	} else {
		/* reply has correct size? */
		if (reply.hdr.size != msg->reply_size) {
			dev_err(sdev->dev, "error: reply expected %zu got %u bytes\n",
				msg->reply_size, reply.hdr.size);
			ret = -EINVAL;
		}

		/* read the message */
		if (msg->reply_size > 0)
			sof_mailbox_read(sdev, sdev->host_box.offset,
					 msg->reply_data, msg->reply_size);
	}

	msg->reply_error = ret;
}

static int imx8ulp_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MBOX_OFFSET;
}

static int imx8ulp_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	return MBOX_OFFSET;
}

static void imx8ulp_dsp_handle_reply(struct imx_dsp_ipc *ipc)
{
	struct imx8ulp_priv *priv = imx_dsp_get_data(ipc);
	unsigned long flags;

	spin_lock_irqsave(&priv->sdev->ipc_lock, flags);

	imx8ulp_get_reply(priv->sdev);
	snd_sof_ipc_reply(priv->sdev, 0);
	spin_unlock_irqrestore(&priv->sdev->ipc_lock, flags);
}

static void imx8ulp_dsp_handle_request(struct imx_dsp_ipc *ipc)
{
	struct imx8ulp_priv *priv = imx_dsp_get_data(ipc);
	u32 p; /* panic code */

	/* Read the message from the debug box. */
	sof_mailbox_read(priv->sdev, priv->sdev->debug_box.offset + 4, &p, sizeof(p));

	/* Check to see if the message is a panic code (0x0dead***) */
	if ((p & SOF_IPC_PANIC_MAGIC_MASK) == SOF_IPC_PANIC_MAGIC)
		snd_sof_dsp_panic(priv->sdev, p);
	else
		snd_sof_ipc_msgs_rx(priv->sdev);
}

static struct imx_dsp_ops dsp_ops = {
	.handle_reply		= imx8ulp_dsp_handle_reply,
	.handle_request		= imx8ulp_dsp_handle_request,
};

static int imx8ulp_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct imx8ulp_priv *priv = sdev->pdata->hw_pdata;

	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);
	imx_dsp_ring_doorbell(priv->dsp_ipc, 0);

	return 0;
}

/* config sc firmware */
static int imx8ulp_run(struct snd_sof_dev *sdev)
{
	struct imx8ulp_priv *priv = sdev->pdata->hw_pdata;

	imx8ulp_sim_lpav_start(priv);

	return 0;
}

static int imx8ulp_reset(struct snd_sof_dev *sdev)
{
	struct imx8ulp_priv *priv = sdev->pdata->hw_pdata;

	/* HiFi4 Platform Clock Enable: 1 enabled, 0 disabled */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, PLAT_CLK_BIT, PLAT_CLK_BIT);
	/* HiFi4 PBCLK clock enable: 1 enabled, 0 disabled */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, PB_CLK_BIT, PB_CLK_BIT);
	/* HiFi4 Clock Enable: 1 enabled, 0 disabled */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, HIFI4_CLK_BIT, HIFI4_CLK_BIT);

	regmap_update_bits(priv->regmap_sim, SYSCTRL0, RESET_BIT, RESET_BIT);
	usleep_range(1, 2);
	/* Stall HIFI4 DSP Execution: 1 stall, 0 not stall */
	regmap_update_bits(priv->regmap_sim, SYSCTRL0, EXECUTE_BIT, EXECUTE_BIT);
	usleep_range(1, 2);

	arm_smccc_smc(FSL_SIP_HIFI_XRDC, 0, 0, 0, 0, 0, 0, 0, &smc_resource);

	return 0;
}

static int imx8ulp_probe(struct snd_sof_dev *sdev)
{
	struct platform_device *pdev =
		container_of(sdev->dev, struct platform_device, dev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *res_node;
	struct resource *mmio;
	struct imx8ulp_priv *priv;
	struct resource res;
	u32 base, size;
	int ret = 0;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sdev->pdata->hw_pdata = priv;
	priv->dev = sdev->dev;
	priv->sdev = sdev;

	ret = imx8ulp_configure_sim_lpav(priv);
	if (ret)
		return -EPROBE_DEFER;

	priv->ipc_dev = platform_device_register_data(sdev->dev, "imx-dsp",
						      PLATFORM_DEVID_NONE,
						      pdev, sizeof(*pdev));
	if (IS_ERR(priv->ipc_dev)) {
		ret = PTR_ERR(priv->ipc_dev);
		goto exit_unroll_pm;
	}

	priv->dsp_ipc = dev_get_drvdata(&priv->ipc_dev->dev);
	if (!priv->dsp_ipc) {
		/* DSP IPC driver not probed yet, try later */
		ret = -EPROBE_DEFER;
		dev_err(sdev->dev, "Failed to get drvdata\n");
		goto exit_pdev_unregister;
	}

	imx_dsp_set_data(priv->dsp_ipc, priv);
	priv->dsp_ipc->ops = &dsp_ops;

	/* DSP base */
	mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mmio) {
		base = mmio->start;
		size = resource_size(mmio);
	} else {
		dev_err(sdev->dev, "error: failed to get DSP base at idx 0\n");
		ret = -EINVAL;
		goto exit_pdev_unregister;
	}

	sdev->bar[SOF_FW_BLK_TYPE_IRAM] = devm_ioremap(sdev->dev, base, size);
	if (!sdev->bar[SOF_FW_BLK_TYPE_IRAM]) {
		dev_err(sdev->dev, "failed to ioremap base 0x%x size 0x%x\n",
			base, size);
		ret = -ENODEV;
		goto exit_pdev_unregister;
	}
	sdev->mmio_bar = SOF_FW_BLK_TYPE_IRAM;

	res_node = of_parse_phandle(np, "memory-reserved", 0);
	if (!res_node) {
		dev_err(&pdev->dev, "failed to get memory region node\n");
		ret = -ENODEV;
		goto exit_pdev_unregister;
	}

	ret = of_address_to_resource(res_node, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "failed to get reserved region address\n");
		goto exit_pdev_unregister;
	}

	sdev->bar[SOF_FW_BLK_TYPE_SRAM] = devm_ioremap_wc(sdev->dev, res.start,
							  resource_size(&res));
	if (!sdev->bar[SOF_FW_BLK_TYPE_SRAM]) {
		dev_err(sdev->dev, "failed to ioremap mem 0x%x size 0x%x\n",
			base, size);
		ret = -ENOMEM;
		goto exit_pdev_unregister;
	}
	sdev->mailbox_bar = SOF_FW_BLK_TYPE_SRAM;

	/* set default mailbox offset for FW ready message */
	sdev->dsp_box.offset = MBOX_OFFSET;

	ret = of_reserved_mem_device_init(sdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init reserved memory region %d\n", ret);
		goto exit_pdev_unregister;
	}

	imx8ulp_init_clocks(sdev);
	imx8ulp_prepare_clocks(sdev);

	return 0;

exit_pdev_unregister:
	platform_device_unregister(priv->ipc_dev);
exit_unroll_pm:
	while (--i >= 0) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return ret;
}

static int imx8ulp_remove(struct snd_sof_dev *sdev)
{
	struct imx8ulp_priv *priv = sdev->pdata->hw_pdata;
	int i;

	platform_device_unregister(priv->ipc_dev);

	for (i = 0; i < priv->num_domains; i++) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return 0;
}

/* on i.MX8 there is 1 to 1 match between type and BAR idx */
static int imx8ulp_get_bar_index(struct snd_sof_dev *sdev, u32 type)
{
	return type;
}

static void imx8ulp_ipc_msg_data(struct snd_sof_dev *sdev,
			      struct snd_sof_pcm_stream *sps,
			      void *p, size_t sz)
{
	sof_mailbox_read(sdev, sdev->dsp_box.offset, p, sz);
}

static int imx8ulp_ipc_pcm_params(struct snd_sof_dev *sdev,
			       struct snd_pcm_substream *substream,
			       const struct sof_ipc_pcm_params_reply *reply)
{
	return 0;
}

static int imx8ulp_pcm_open(struct snd_sof_dev *sdev,
			  struct snd_pcm_substream *substream)
{
	return 0;
}

int imx8ulp_suspend(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8ulp_priv *priv = (struct imx8ulp_priv *)sdev->pdata->hw_pdata;

	regmap_update_bits(priv->regmap_sim, SYSCTRL0, EXECUTE_BIT, EXECUTE_BIT);

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_free_channel(priv->dsp_ipc, i);

	imx8ulp_disable_clocks(priv->sdev);

	return 0;
}

int imx8ulp_resume(struct snd_sof_dev *sdev)
{
	struct imx8ulp_priv *priv = (struct imx8ulp_priv *)sdev->pdata->hw_pdata;
	int i;

	imx8ulp_prepare_clocks(sdev);

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_request_channel(priv->dsp_ipc, i);

	return 0;
}

int imx8ulp_dsp_runtime_resume(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D0,
		.substate = 0,
	};

	imx8ulp_resume(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

int imx8ulp_dsp_runtime_suspend(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D3,
		.substate = 0,
	};

	imx8ulp_suspend(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}


int imx8ulp_dsp_suspend(struct snd_sof_dev *sdev, unsigned int target_state)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = target_state,
		.substate = 0,
	};

	if (!pm_runtime_suspended(sdev->dev))
		imx8ulp_suspend(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

int imx8ulp_dsp_resume(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D0,
		.substate = 0,
	};

	imx8ulp_resume(sdev);

	if (pm_runtime_suspended(sdev->dev)) {
		pm_runtime_disable(sdev->dev);
		pm_runtime_set_active(sdev->dev);
		pm_runtime_mark_last_busy(sdev->dev);
		pm_runtime_enable(sdev->dev);
		pm_runtime_idle(sdev->dev);
	}

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

static struct snd_soc_dai_driver imx8ulp_dai[] = {
{
	.name = "sai5",
	.playback = {
		.channels_min = 1,
		.channels_max = 32,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 32,
	},
},
};

int imx8ulp_dsp_set_power_state(struct snd_sof_dev *sdev,
			     const struct sof_dsp_power_state *target_state)
{
	sdev->dsp_power_state = *target_state;

	return 0;
}

/* i.MX8 ops */
struct snd_sof_dsp_ops sof_imx8ulp_ops = {
	/* probe and remove */
	.probe		= imx8ulp_probe,
	.remove		= imx8ulp_remove,
	/* DSP core boot */
	.run		= imx8ulp_run,
	.reset      = imx8ulp_reset,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* Module IO */
	.read64	= sof_io_read64,

	/* ipc */
	.send_msg	= imx8ulp_send_msg,
	.fw_ready	= sof_fw_ready,
	.get_mailbox_offset	= imx8ulp_get_mailbox_offset,
	.get_window_offset	= imx8ulp_get_window_offset,

	.ipc_msg_data	= imx8ulp_ipc_msg_data,
	.ipc_pcm_params	= imx8ulp_ipc_pcm_params,

	/* pcm ops*/
	.pcm_open = imx8ulp_pcm_open,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,
	.get_bar_index	= imx8ulp_get_bar_index,
	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* Debug information */
	.dbg_dump = imx8_dump,

	/* Firmware ops */
	.arch_ops = &sof_xtensa_arch_ops,

	/* DAI drivers */
	.drv = imx8ulp_dai,
	.num_drv = ARRAY_SIZE(imx8ulp_dai),

	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,

	/* PM */
	.runtime_suspend	= imx8ulp_dsp_runtime_suspend,
	.runtime_resume		= imx8ulp_dsp_runtime_resume,

	.suspend	= imx8ulp_dsp_suspend,
	.resume		= imx8ulp_dsp_resume,

	.set_power_state	= imx8ulp_dsp_set_power_state,
	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP
};
EXPORT_SYMBOL(sof_imx8ulp_ops);

MODULE_IMPORT_NS(SND_SOC_SOF_XTENSA);
MODULE_LICENSE("Dual BSD/GPL");
