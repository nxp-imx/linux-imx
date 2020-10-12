// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// Copyright 2019 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>
//
// Hardware interface for audio DSP on i.MX8

#include <linux/firmware.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_domain.h>

#include <linux/module.h>
#include <sound/sof.h>
#include <sound/sof/xtensa.h>
#include <linux/firmware/imx/dsp.h>
#include <linux/clk.h>
#include <linux/bits.h>

#include "../ops.h"
#include "../../fsl/fsl_dsp_audiomix.h"

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

#define IMX8M_DAP_DEBUG		0x28800000
#define IMX8M_DAP_DEBUG_SIZE	(64 * 1024)
#define IMX8M_DAP_PWRCTL	(0x4000 + 0x3020)
#define IMX8M_PWRCTL_CORERESET		BIT(16)

#define IMX8M_DSP_CLK_NUM	9
static const char *imx8m_dsp_clks[IMX8M_DSP_CLK_NUM] = {
	"ocram",
	"core",
	"debug",
	"sdma3",
	"sai3_bus",
	"sai3_mclk0",
	"sai3_mclk1",
	"sai3_mclk2",
	"sai3_mclk3",
};

struct imx8m_priv {
	struct device *dev;
	struct snd_sof_dev *sdev;
	bool suspended;

	struct imx_audiomix_dsp_data *audiomix;

	/* DSP IPC handler */
	struct imx_dsp_ipc *dsp_ipc;
	struct platform_device *ipc_dev;

	/* Power domain handling */
	int num_domains;
	struct device **pd_dev;
	struct device_link **link;

	struct clk *clks[IMX8M_DSP_CLK_NUM];
	void __iomem *dap;
};

int imx8m_dsp_configure_audmix(struct imx8m_priv *dsp_priv)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_node_by_name(NULL, "audiomix_dsp");
	if (!np)
		return -EPROBE_DEFER;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return -EPROBE_DEFER;

	dsp_priv->audiomix = dev_get_drvdata(&pdev->dev);
	if (!dsp_priv->audiomix)
		return -EPROBE_DEFER;

	return 0;
}

static void imx8m_get_reply(struct snd_sof_dev *sdev)
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

static int imx8m_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MBOX_OFFSET;
}

static int imx8m_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	return MBOX_OFFSET;
}

static void imx8m_dsp_handle_reply(struct imx_dsp_ipc *ipc)
{
	struct imx8m_priv *priv = imx_dsp_get_data(ipc);
	unsigned long flags;

	spin_lock_irqsave(&priv->sdev->ipc_lock, flags);
	imx8m_get_reply(priv->sdev);
	snd_sof_ipc_reply(priv->sdev, 0);
	spin_unlock_irqrestore(&priv->sdev->ipc_lock, flags);
}

static void imx8m_dsp_handle_request(struct imx_dsp_ipc *ipc)
{
	struct imx8m_priv *priv = imx_dsp_get_data(ipc);

	snd_sof_ipc_msgs_rx(priv->sdev);
}

static struct imx_dsp_ops dsp_ops = {
	.handle_reply		= imx8m_dsp_handle_reply,
	.handle_request		= imx8m_dsp_handle_request,
};

static int imx8m_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;

	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);
	imx_dsp_ring_doorbell(priv->dsp_ipc, 0);

	return 0;
}

/*
 * DSP control.
 */
static int imx8m_run(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *dsp_priv = (struct imx8m_priv *)sdev->private;

	imx_audiomix_dsp_start(dsp_priv->audiomix);

	return 0;
}

static int imx8m_reset(struct snd_sof_dev *sdev) {

	struct imx8m_priv *dsp_priv = (struct imx8m_priv *)sdev->private;
	u32 pwrctl;

	/* put DSP into reset and stall */
	pwrctl = readl(dsp_priv->dap + IMX8M_DAP_PWRCTL);
	pwrctl |= IMX8M_PWRCTL_CORERESET;
	writel(pwrctl, dsp_priv->dap + IMX8M_DAP_PWRCTL);

	/* keep reset asserted for 10 cycles */
	usleep_range(1, 2);

	imx_audiomix_dsp_stall(dsp_priv->audiomix);

	/* take the DSP out of reset and keep stalled for FW loading */
	pwrctl = readl(dsp_priv->dap + IMX8M_DAP_PWRCTL);
	pwrctl &= ~IMX8M_PWRCTL_CORERESET;
	writel(pwrctl, dsp_priv->dap + IMX8M_DAP_PWRCTL);

	return 0;
}

static int imx8m_probe(struct snd_sof_dev *sdev)
{
	struct platform_device *pdev =
		container_of(sdev->dev, struct platform_device, dev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *res_node;
	struct resource *mmio;
	struct imx8m_priv *priv;
	struct resource res;
	u32 base, size;
	int ret = 0;
	int i = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sdev->private = priv;
	priv->dev = sdev->dev;
	priv->sdev = sdev;

	ret = imx8m_dsp_configure_audmix(priv);
	if (ret < 0)
		return ret;

	/* power up device associated power domains */
	priv->num_domains = of_count_phandle_with_args(np, "power-domains",
						       "#power-domain-cells");
	if (priv->num_domains < 0) {
		dev_err(sdev->dev, "no power-domains property in %pOF\n", np);
		return priv->num_domains;
	}

	/* power domain already enabled by PM core */
	if (priv->num_domains == 1)
		goto done_pm;

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

done_pm:
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

	priv->dap = devm_ioremap(sdev->dev, IMX8M_DAP_DEBUG, IMX8M_DAP_DEBUG_SIZE);
	if (!priv->dap ) {
		dev_err(sdev->dev, "error: failed to map DAP debug memory area");
		return -ENODEV;
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
	if (ret) {
		dev_err(&pdev->dev, "failed to get reserved region address\n");
		goto exit_pdev_unregister;
	}

	sdev->bar[SOF_FW_BLK_TYPE_SRAM] = devm_ioremap_wc(sdev->dev, res.start,
							  res.end - res.start +
							  1);
	if (!sdev->bar[SOF_FW_BLK_TYPE_SRAM]) {
		dev_err(sdev->dev, "failed to ioremap mem 0x%x size 0x%x\n",
			base, size);
		ret = -ENOMEM;
		goto exit_pdev_unregister;
	}
	sdev->mailbox_bar = SOF_FW_BLK_TYPE_SRAM;

	/* set default mailbox offset for FW ready message */
	sdev->dsp_box.offset = MBOX_OFFSET;

	for (i = 0; i < IMX8M_DSP_CLK_NUM; i++) {
		priv->clks[i] = devm_clk_get(&pdev->dev, imx8m_dsp_clks[i]);
		if (IS_ERR(priv->clks[i]))
			priv->clks[i] = NULL;
	}
	/* TODO: handle clocks at PM suspend/resume time */
	for (i = 0; i < IMX8M_DSP_CLK_NUM; i++)
		clk_prepare_enable(priv->clks[i]);

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

static int imx8m_remove(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;
	int i;

	platform_device_unregister(priv->ipc_dev);

	for (i = 0; i < priv->num_domains; i++) {
		device_link_del(priv->link[i]);
		dev_pm_domain_detach(priv->pd_dev[i], false);
	}

	for (i = 0; i < IMX8M_DSP_CLK_NUM; i++)
		clk_disable_unprepare(priv->clks[i]);

	return 0;
}

/* on i.MX8 there is 1 to 1 match between type and BAR idx */
static int imx8m_get_bar_index(struct snd_sof_dev *sdev, u32 type)
{
	return type;
}

static void imx8m_ipc_msg_data(struct snd_sof_dev *sdev,
			       struct snd_pcm_substream *substream,
			       void *p, size_t sz)
{
	sof_mailbox_read(sdev, sdev->dsp_box.offset, p, sz);
}

static int imx8m_ipc_pcm_params(struct snd_sof_dev *sdev,
				struct snd_pcm_substream *substream,
				const struct sof_ipc_pcm_params_reply *reply)
{
	return 0;
}

static struct snd_soc_dai_driver imx8m_dai[] = {
{
	.name = "sai3",
},
};

int imx8m_resume(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;
	int i;

	for (i = 0; i < IMX8M_DSP_CLK_NUM; i++)
		clk_prepare_enable(priv->clks[i]);

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_request_channel(priv->dsp_ipc, i);

	return 0;
}

int imx8m_suspend(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;
	int i;

	for (i = 0; i < DSP_MU_CHAN_NUM; i++)
		imx_dsp_free_channel(priv->dsp_ipc, i);

	for (i = 0; i < IMX8M_DSP_CLK_NUM; i++)
		clk_disable_unprepare(priv->clks[i]);

	return 0;
}

static int imx8m_dsp_runtime_resume(struct snd_sof_dev *sdev)
{
	return imx8m_resume(sdev);
}

static int imx8m_dsp_runtime_suspend(struct snd_sof_dev *sdev)
{
	return imx8m_suspend(sdev);
}

static int imx8m_dsp_resume(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;

	if (priv->suspended) {
		imx8m_resume(sdev);
		priv->suspended = false;
	}

	return 0;
}

static int imx8m_dsp_suspend(struct snd_sof_dev *sdev)
{
	struct imx8m_priv *priv = (struct imx8m_priv *)sdev->private;

	if (!priv->suspended) {
		imx8m_suspend(sdev);
		priv->suspended = true;
	}

	return 0;
}

/* i.MX8 ops */
struct snd_sof_dsp_ops sof_imx8m_ops = {
	/* probe and remove */
	.probe		= imx8m_probe,
	.remove		= imx8m_remove,
	/* DSP core boot */
	.run		= imx8m_run,
	.reset		= imx8m_reset,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* ipc */
	.send_msg	= imx8m_send_msg,
	.fw_ready	= sof_fw_ready,
	.get_mailbox_offset	= imx8m_get_mailbox_offset,
	.get_window_offset	= imx8m_get_window_offset,

	.ipc_msg_data	= imx8m_ipc_msg_data,
	.ipc_pcm_params	= imx8m_ipc_pcm_params,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,
	.get_bar_index	= imx8m_get_bar_index,
	/* firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,

	/* DAI drivers */
	.drv = imx8m_dai,
	.num_drv = 1, /* we have only 1 ESAI interface on i.MX8 */

	.suspend	= imx8m_dsp_suspend,
	.resume		= imx8m_dsp_resume,

	.runtime_suspend = imx8m_dsp_runtime_suspend,
	.runtime_resume = imx8m_dsp_runtime_resume,

	.hw_info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
};
EXPORT_SYMBOL(sof_imx8m_ops);

MODULE_LICENSE("Dual BSD/GPL");
