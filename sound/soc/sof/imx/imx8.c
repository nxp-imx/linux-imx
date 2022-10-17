// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright 2019 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>
//
// Hardware interface for audio DSP on i.MX8

#include <linux/clk.h>
#include <linux/extcon-provider.h>
#include <linux/firmware.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_domain.h>

#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/dsp.h>

#include <linux/firmware/imx/svc/misc.h>
#include <dt-bindings/firmware/imx/rsrc.h>
#include "../ops.h"
#include "../sof-of-dev.h"
#include "imx-common.h"

/* DSP memories */
#define IRAM_OFFSET		0x10000
#define IRAM_SIZE		(2 * 1024)
#define DRAM0_OFFSET		0x0
#define DRAM0_SIZE		(32 * 1024)
#define DRAM1_OFFSET		0x8000
#define DRAM1_SIZE		(32 * 1024)
#define SYSRAM_OFFSET		0x18000
#define SYSRAM_SIZE		(256 * 1024)
#define SYSROM_OFFSET		0x58000
#define SYSROM_SIZE		(192 * 1024)

#define RESET_VECTOR_VADDR	0x596f8000

#define MBOX_OFFSET	0x800000
#define MBOX_SIZE	0x1000

#define IMX8_DSP_CLK_NUM	3
static const char *imx8_dsp_clks_names[IMX8_DSP_CLK_NUM] =
{
	/* DSP clocks */
	"ipg", "ocram", "core",
};

#define IMX8_DAI_CLK_NUM	9
static const char *imx8_dai_clks_names[IMX8_DAI_CLK_NUM] =
{
	/* ESAI0 clocks */
	"esai0_core", "esai0_extal", "esai0_fsys", "esai0_spba",
	/* SAI1 clocks */
	"sai1_bus", "sai1_mclk0", "sai1_mclk1", "sai1_mclk2", "sai1_mclk3",
};

#ifdef CONFIG_EXTCON
struct extcon_dev *sof_imx8_edev;
static const unsigned int sof_imx8_extcon_cables[] = {
	EXTCON_JACK_LINE_OUT,
	EXTCON_NONE,
};
#endif

struct imx8_priv {
	struct device *dev;
	struct snd_sof_dev *sdev;

	/* DSP IPC handler */
	struct imx_dsp_ipc *dsp_ipc;
	struct platform_device *ipc_dev;

	/* System Controller IPC handler */
	struct imx_sc_ipc *sc_ipc;

	/* Power domain handling */
	int num_domains;
	struct device **pd_dev;
	struct device_link **link;

	struct clk *dsp_clks[IMX8_DSP_CLK_NUM];
	struct clk *dai_clks[IMX8_DAI_CLK_NUM];
};

static int imx8_init_clocks(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8_priv *priv = (struct imx8_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8_DSP_CLK_NUM; i++) {
		priv->dsp_clks[i] = devm_clk_get(priv->dev, imx8_dsp_clks_names[i]);
		if (IS_ERR(priv->dsp_clks[i]))
		    return PTR_ERR(priv->dsp_clks[i]);
	}

	for (i = 0; i < IMX8_DAI_CLK_NUM; i++)
		priv->dai_clks[i] = devm_clk_get_optional(priv->dev, imx8_dai_clks_names[i]);

	return 0;
}

static int imx8_prepare_clocks(struct snd_sof_dev *sdev)
{
	int i, j, ret;
	struct imx8_priv *priv = (struct imx8_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8_DSP_CLK_NUM; i++) {
		ret = clk_prepare_enable(priv->dsp_clks[i]);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable clk %s\n",
				imx8_dsp_clks_names[i]);
			goto err_dsp_clks;
		}
	}

	for (j = 0; j < IMX8_DAI_CLK_NUM; j++) {
		ret = clk_prepare_enable(priv->dai_clks[j]);
		if (ret < 0) {
			dev_err(priv->dev, "Failed to enable clk %s\n",
				imx8_dai_clks_names[j]);
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

static void imx8_disable_clocks(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8_priv *priv = (struct imx8_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < IMX8_DSP_CLK_NUM; i++)
		clk_disable_unprepare(priv->dsp_clks[i]);

	for (i = 0; i < IMX8_DAI_CLK_NUM; i++)
		clk_disable_unprepare(priv->dai_clks[i]);
}

static void imx8_get_reply(struct snd_sof_dev *sdev)
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

static int imx8_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MBOX_OFFSET;
}

static int imx8_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	return MBOX_OFFSET;
}

static void imx8_dsp_handle_reply(struct imx_dsp_ipc *ipc)
{
	struct imx8_priv *priv = imx_dsp_get_data(ipc);
	unsigned long flags;

	spin_lock_irqsave(&priv->sdev->ipc_lock, flags);
	imx8_get_reply(priv->sdev);
	snd_sof_ipc_reply(priv->sdev, 0);
	spin_unlock_irqrestore(&priv->sdev->ipc_lock, flags);
}

static void imx8_dsp_handle_request(struct imx_dsp_ipc *ipc)
{
	struct imx8_priv *priv = imx_dsp_get_data(ipc);
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
	.handle_reply		= imx8_dsp_handle_reply,
	.handle_request		= imx8_dsp_handle_request,
};

static int imx8_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct imx8_priv *priv = sdev->pdata->hw_pdata;

	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);
	imx_dsp_ring_doorbell(priv->dsp_ipc, 0);

	return 0;
}

/*
 * DSP control.
 */
static int imx8x_run(struct snd_sof_dev *sdev)
{
	struct imx8_priv *dsp_priv = sdev->pdata->hw_pdata;
	int ret;

	ret = imx_sc_misc_set_control(dsp_priv->sc_ipc, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_SEL, 1);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset source select\n");
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_priv->sc_ipc, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_AUDIO, 0x80);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of AUDIO\n");
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_priv->sc_ipc, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_PERIPH, 0x5A);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of PERIPH %d\n",
			ret);
		return ret;
	}

	ret = imx_sc_misc_set_control(dsp_priv->sc_ipc, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_IRQ, 0x51);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset of IRQ\n");
		return ret;
	}

	imx_sc_pm_cpu_start(dsp_priv->sc_ipc, IMX_SC_R_DSP, true,
			    RESET_VECTOR_VADDR);

	return 0;
}

static int imx8_run(struct snd_sof_dev *sdev)
{
	struct imx8_priv *dsp_priv = sdev->pdata->hw_pdata;
	int ret;

	ret = imx_sc_misc_set_control(dsp_priv->sc_ipc, IMX_SC_R_DSP,
				      IMX_SC_C_OFS_SEL, 0);
	if (ret < 0) {
		dev_err(sdev->dev, "Error system address offset source select\n");
		return ret;
	}

	imx_sc_pm_cpu_start(dsp_priv->sc_ipc, IMX_SC_R_DSP, true,
			    RESET_VECTOR_VADDR);

	return 0;
}

static int imx8_probe(struct snd_sof_dev *sdev)
{
	struct platform_device *pdev =
		container_of(sdev->dev, struct platform_device, dev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *res_node;
	struct resource *mmio;
	struct imx8_priv *priv;
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

	/* power up device associated power domains */
	priv->num_domains = of_count_phandle_with_args(np, "power-domains",
						       "#power-domain-cells");
	if (priv->num_domains < 0) {
		dev_err(sdev->dev, "no power-domains property in %pOF\n", np);
		return priv->num_domains;
	}

	priv->pd_dev = devm_kmalloc_array(&pdev->dev, priv->num_domains,
					  sizeof(*priv->pd_dev), GFP_KERNEL);
	if (!priv->pd_dev)
		return -ENOMEM;

	priv->link = devm_kmalloc_array(&pdev->dev, priv->num_domains,
					sizeof(*priv->link), GFP_KERNEL);
	if (!priv->link)
		return -ENOMEM;

	for (i = 0; i < priv->num_domains; i++) {
		priv->pd_dev[i] = dev_pm_domain_attach_by_id(&pdev->dev, i);
		if (IS_ERR(priv->pd_dev[i])) {
			ret = PTR_ERR(priv->pd_dev[i]);
			goto exit_unroll_pm;
		}
		priv->link[i] = device_link_add(&pdev->dev, priv->pd_dev[i],
						DL_FLAG_STATELESS |
						DL_FLAG_PM_RUNTIME |
						DL_FLAG_RPM_ACTIVE);
		if (!priv->link[i]) {
			ret = -ENOMEM;
			dev_pm_domain_detach(priv->pd_dev[i], false);
			goto exit_unroll_pm;
		}
	}

	ret = imx_scu_get_handle(&priv->sc_ipc);
	if (ret) {
		dev_err(sdev->dev, "Cannot obtain SCU handle (err = %d)\n",
			ret);
		goto exit_unroll_pm;
	}

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

	res_node = of_parse_phandle(np, "memory-region", 0);
	if (!res_node) {
		dev_err(&pdev->dev, "failed to get memory region node\n");
		ret = -ENODEV;
		goto exit_pdev_unregister;
	}

	ret = of_address_to_resource(res_node, 0, &res);
	of_node_put(res_node);
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

	imx8_init_clocks(sdev);
	imx8_prepare_clocks(sdev);

#ifdef CONFIG_EXTCON
	sof_imx8_edev  = devm_extcon_dev_allocate(&pdev->dev, sof_imx8_extcon_cables);
	if (IS_ERR(sof_imx8_edev)) {
		dev_err(&pdev->dev, "failed to allocate extcon device\n");
		return 0;
	}
	ret = devm_extcon_dev_register(&pdev->dev, sof_imx8_edev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return 0;
	}
	extcon_set_state_sync(sof_imx8_edev, EXTCON_JACK_LINE_OUT, 1);
#endif

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

static int imx8_remove(struct snd_sof_dev *sdev)
{
	struct imx8_priv *priv = sdev->pdata->hw_pdata;
	int i;

	platform_device_unregister(priv->ipc_dev);

	for (i = 0; i < priv->num_domains; i++) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	return 0;
}

/* on i.MX8 there is 1 to 1 match between type and BAR idx */
static int imx8_get_bar_index(struct snd_sof_dev *sdev, u32 type)
{
	/* Only IRAM and SRAM bars are valid */
	switch (type) {
	case SOF_FW_BLK_TYPE_IRAM:
	case SOF_FW_BLK_TYPE_SRAM:
		return type;
	default:
		return -EINVAL;
	}
}

int imx8_suspend(struct snd_sof_dev *sdev)
{
	int i;
	struct imx8_priv *priv = (struct imx8_priv *)sdev->pdata->hw_pdata;

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_free_channel(priv->dsp_ipc, i);

	imx8_disable_clocks(priv->sdev);

	return 0;
}

int imx8_resume(struct snd_sof_dev *sdev)
{
	struct imx8_priv *priv = (struct imx8_priv *)sdev->pdata->hw_pdata;
	int i;

	imx8_prepare_clocks(sdev);

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_request_channel(priv->dsp_ipc, i);

	return 0;
}

int imx8_dsp_runtime_resume(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D0,
		.substate = 0,
	};

	imx8_resume(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

int imx8_dsp_runtime_suspend(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D3,
		.substate = 0,
	};

	imx8_suspend(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}


int imx8_dsp_suspend(struct snd_sof_dev *sdev, unsigned int target_state)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = target_state,
		.substate = 0,
	};

	if (!pm_runtime_suspended(sdev->dev))
		imx8_suspend(sdev);

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

int imx8_dsp_resume(struct snd_sof_dev *sdev)
{
	const struct sof_dsp_power_state target_dsp_state = {
		.state = SOF_DSP_PM_D0,
		.substate = 0,
	};

	imx8_resume(sdev);

	if (pm_runtime_suspended(sdev->dev)) {
		pm_runtime_disable(sdev->dev);
		pm_runtime_set_active(sdev->dev);
		pm_runtime_mark_last_busy(sdev->dev);
		pm_runtime_enable(sdev->dev);
		pm_runtime_idle(sdev->dev);
	}

	return snd_sof_dsp_set_power_state(sdev, &target_dsp_state);
}

static struct snd_soc_dai_driver imx8_dai[] = {
{
	.name = "esai0",
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 8,
	},
},
{
	.name = "sai1",
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

int imx8_dsp_set_power_state(struct snd_sof_dev *sdev,
			     const struct sof_dsp_power_state *target_state)
{
	sdev->dsp_power_state = *target_state;

	return 0;
}

/* i.MX8 ops */
static const struct snd_sof_dsp_ops sof_imx8_ops = {
	/* probe and remove */
	.probe		= imx8_probe,
	.remove		= imx8_remove,
	/* DSP core boot */
	.run		= imx8_run,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* Module IO */
	.read64	= sof_io_read64,

	/* ipc */
	.send_msg	= imx8_send_msg,
	.fw_ready	= sof_fw_ready,
	.get_mailbox_offset	= imx8_get_mailbox_offset,
	.get_window_offset	= imx8_get_window_offset,

	.ipc_msg_data	= sof_ipc_msg_data,
	.ipc_pcm_params	= sof_ipc_pcm_params,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,
	.get_bar_index	= imx8_get_bar_index,
	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* Debug information */
	.dbg_dump = imx8_dump,

	/* stream callbacks */
	.pcm_open = sof_stream_pcm_open,
	.pcm_close = sof_stream_pcm_close,

	/* Firmware ops */
	.arch_ops = &sof_xtensa_arch_ops,

	/* DAI drivers */
	.drv = imx8_dai,
	.num_drv = ARRAY_SIZE(imx8_dai),

	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,

	/* PM */
	.runtime_suspend	= imx8_dsp_runtime_suspend,
	.runtime_resume		= imx8_dsp_runtime_resume,

	.suspend	= imx8_dsp_suspend,
	.resume		= imx8_dsp_resume,

	.set_power_state	= imx8_dsp_set_power_state,
};

/* i.MX8X ops */
static const struct snd_sof_dsp_ops sof_imx8x_ops = {
	/* probe and remove */
	.probe		= imx8_probe,
	.remove		= imx8_remove,
	/* DSP core boot */
	.run		= imx8x_run,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* Module IO */
	.read64	= sof_io_read64,

	/* ipc */
	.send_msg	= imx8_send_msg,
	.fw_ready	= sof_fw_ready,
	.get_mailbox_offset	= imx8_get_mailbox_offset,
	.get_window_offset	= imx8_get_window_offset,

	.ipc_msg_data	= sof_ipc_msg_data,
	.ipc_pcm_params	= sof_ipc_pcm_params,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,
	.get_bar_index	= imx8_get_bar_index,
	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* Debug information */
	.dbg_dump = imx8_dump,

	/* stream callbacks */
	.pcm_open = sof_stream_pcm_open,
	.pcm_close = sof_stream_pcm_close,

	/* Firmware ops */
	.arch_ops = &sof_xtensa_arch_ops,

	/* DAI drivers */
	.drv = imx8_dai,
	.num_drv = ARRAY_SIZE(imx8_dai),

	/* PM */
	.runtime_suspend	= imx8_dsp_runtime_suspend,
	.runtime_resume		= imx8_dsp_runtime_resume,

	.suspend	= imx8_dsp_suspend,
	.resume		= imx8_dsp_resume,

	.set_power_state	= imx8_dsp_set_power_state,

	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP
};

static struct sof_dev_desc sof_of_imx8qxp_desc = {
	.default_fw_path = "imx/sof",
	.default_tplg_path = "imx/sof-tplg",
	.default_fw_filename = "sof-imx8x.ri",
	.nocodec_tplg_filename = "sof-imx8-nocodec.tplg",
	.ops = &sof_imx8x_ops,
};

static struct sof_dev_desc sof_of_imx8qm_desc = {
	.default_fw_path = "imx/sof",
	.default_tplg_path = "imx/sof-tplg",
	.default_fw_filename = "sof-imx8.ri",
	.nocodec_tplg_filename = "sof-imx8-nocodec.tplg",
	.ops = &sof_imx8_ops,
};

static const struct of_device_id sof_of_imx8_ids[] = {
	{ .compatible = "fsl,imx8qxp-dsp", .data = &sof_of_imx8qxp_desc},
	{ .compatible = "fsl,imx8qm-dsp", .data = &sof_of_imx8qm_desc},
	{ }
};
MODULE_DEVICE_TABLE(of, sof_of_imx8_ids);

/* DT driver definition */
static struct platform_driver snd_sof_of_imx8_driver = {
	.probe = sof_of_probe,
	.remove = sof_of_remove,
	.driver = {
		.name = "sof-audio-of-imx8",
		.pm = &sof_of_pm,
		.of_match_table = sof_of_imx8_ids,
	},
};
module_platform_driver(snd_sof_of_imx8_driver);

MODULE_IMPORT_NS(SND_SOC_SOF_XTENSA);
MODULE_LICENSE("Dual BSD/GPL");
