// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Coda series multi-standard codec IP - platform driver
 *
 * Copyright (C) 2024-2026 CHIPS&MEDIA INC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/genalloc.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include "coda-vpu.h"
#include "coda-regdefine.h"
#include "coda-helper.h"

#define VPU_PLATFORM_DEVICE_NAME "coda-vpu"
#define VPU_CLK_NAME "vcodec"

#define CODA_IS_ENC BIT(0)
#define CODA_IS_DEC BIT(1)

struct coda_match_data {
	int flags;
	const char *fw_name;
	u32 sram_size;
};

static const struct coda_match_data nxp_coda980_data = {
	.flags = CODA_IS_ENC,
	.fw_name = "cnm/coda980_enc_fw.bin",
	/* For AVC, 4096x2304, 8bit */
	.sram_size = 0xD000,
};

static u8 coda_vpu_char_to_hex(u8 char_in)
{
	if (char_in >= '0' && char_in <= '9')
		char_in = char_in - '0';
	else if (char_in >= 'a' && char_in <= 'f')
		char_in = char_in - 'a' + 10;
	else if (char_in >= 'A' && char_in <= 'F')
		char_in = char_in - 'A' + 10;

	return (char_in & 0xF);
}

static size_t coda_vpu_convert_firmware_to_binary(struct vpu_device *vpu,
						  const struct firmware *fw,
						  u8 *out)
{
	u8 *in = (u8 *)fw->data;
	size_t in_size = fw->size;
	size_t pos = 0;
	int i;

	if (!in || !out)
		return pos;

	for (i = 0; i < in_size; i += 5) {
		out[pos++] = (coda_vpu_char_to_hex(in[i]) << 4) |
			     coda_vpu_char_to_hex(in[i + 1]);
		out[pos++] = (coda_vpu_char_to_hex(in[i + 2]) << 4) |
			     coda_vpu_char_to_hex(in[i + 3]);
	}

	return pos;
}

static void coda_vpu_reorder_binary_for_hw(struct vpu_device *vpu, u8 *code,
					   size_t code_size)
{
	u8 *temp;
	int i;

	temp = devm_kzalloc(vpu->dev, code_size, GFP_KERNEL);

	for (i = 0; i < code_size; i++)
		temp[i] = code[(i & ~0x7) + (6 - (i & 0x6)) + ((i & 1) ^ 1)];

	memcpy(code, temp, code_size);

	devm_kfree(vpu->dev, temp);
}

static int coda_vpu_load_firmware(struct vpu_device *vpu)
{
	const struct firmware *fw;
	int ret;
	u32 version, revision;
	u8 *code;
	size_t code_size;

	ret = request_firmware(&fw, vpu->fw_name, vpu->dev);
	if (ret) {
		dev_err(vpu->dev, "request_firmware fail\n");
		return ret;
	}

	code = devm_kzalloc(vpu->dev, fw->size, GFP_KERNEL);

	code_size = coda_vpu_convert_firmware_to_binary(vpu, fw, code);
	coda_vpu_reorder_binary_for_hw(vpu, code, code_size);

	ret = coda_vpuapi_init_with_bitcode(vpu->dev, code, code_size);
	if (ret) {
		dev_err(vpu->dev, "vpu_init_with_bitcode fail\n");
		devm_kfree(vpu->dev, code);
		release_firmware(fw);
		return ret;
	}

	devm_kfree(vpu->dev, code);
	release_firmware(fw);

	ret = coda_vpuapi_get_version_info(vpu->dev, &version, &revision);
	if (ret) {
		dev_err(vpu->dev, "vpu_get_version_info fail: %d\n", ret);
		return ret;
	}

	dev_info(vpu->dev, "Product Code:      0x%x\n", vpu->product_code);
	dev_info(vpu->dev, "Product ID:        0x%x\n", vpu->product_id);
	dev_info(vpu->dev, "Firmware Version:  %u\n", version);
	dev_info(vpu->dev, "Firmware Revision: %u\n", revision);

	return 0;
}

static irqreturn_t coda_vpu_irq_thread(int irq, void *dev_id)
{
	struct vpu_device *vpu = dev_id;
	struct vpu_instance *inst;
	u32 irq_status;

	if (coda_vdi_readl(vpu->dev, BIT_INT_STS)) {
		irq_status = coda_vdi_readl(vpu->dev, BIT_INT_REASON);

		coda_vdi_writel(vpu->dev, BIT_INT_REASON, 0);
		coda_vdi_writel(vpu->dev, BIT_INT_CLEAR, 0x1);

		if (irq_status & BIT(INT_BIT_SEQ_INIT)) {
			complete(&vpu->irq_done);
		} else if (irq_status & BIT(INT_BIT_PIC_RUN)) {
			inst = v4l2_m2m_get_curr_priv(vpu->m2m_dev);
			if (inst)
				inst->ops->finish_process(inst);
		}
	}

	return IRQ_HANDLED;
}

static int coda_vpu_probe(struct platform_device *pdev)
{
	int ret;
	struct vpu_device *vpu;
	const struct coda_match_data *match_data;
	struct device_node *np;
	struct resource mem;

	match_data = device_get_match_data(&pdev->dev);
	if (!match_data) {
		dev_err(&pdev->dev, "missing match_data\n");
		return -EINVAL;
	}

	/* physical addresses limited to 32 bits */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_err(&pdev->dev, "dma_set_mask_and_coherent failed: %d\n", ret);
		return ret;
	}

	vpu = devm_kzalloc(&pdev->dev, sizeof(struct vpu_device), GFP_KERNEL);
	if (!vpu)
		return -ENOMEM;

	ida_init(&vpu->inst_ida);
	mutex_init(&vpu->dev_lock);
	mutex_init(&vpu->hw_lock);
	init_completion(&vpu->irq_done);
	dev_set_drvdata(&pdev->dev, vpu);
	vpu->dev = &pdev->dev;
	vpu->fw_name = match_data->fw_name;

	vpu->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpu->reg_base))
		return PTR_ERR(vpu->reg_base);

	np = of_parse_phandle(pdev->dev.of_node, "boot", 0);
	if (!np) {
		dev_err(&pdev->dev, "boot node is not found.\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &mem);
	of_node_put(np);
	if (ret) {
		dev_err(&pdev->dev, "boot resource not available.\n");
		return -EINVAL;
	}

	vpu->common_mem.paddr = mem.start;
	vpu->common_mem.size = resource_size(&mem);
	if (vpu->common_mem.size < CODA_SIZE_COMMON) {
		dev_err(&pdev->dev, "boot memory size is small.\n");
		return -EINVAL;
	}

	vpu->common_mem.vaddr = devm_memremap(&pdev->dev,
					      vpu->common_mem.paddr,
					      vpu->common_mem.size,
					      MEMREMAP_WC);
	if (!vpu->common_mem.vaddr) {
		dev_err(&pdev->dev, "boot memory mapping fail.\n");
		return PTR_ERR(vpu->common_mem.vaddr);
	}

	vpu->common_mem.daddr = dma_map_resource(&pdev->dev,
						 vpu->common_mem.paddr,
						 vpu->common_mem.size,
						 DMA_BIDIRECTIONAL,
						 0);

	dev_info(&pdev->dev, "boot daddr: %pad, size: 0x%lx\n",
		 &vpu->common_mem.daddr, vpu->common_mem.size);

	vpu->sram_pool = of_gen_pool_get(pdev->dev.of_node, "sram", 0);
	if (vpu->sram_pool) {
		vpu->sram_buf.size = match_data->sram_size;
		vpu->sram_buf.vaddr = gen_pool_dma_alloc(vpu->sram_pool,
							 vpu->sram_buf.size,
							 &vpu->sram_buf.paddr);
		if (!vpu->sram_buf.vaddr)
			vpu->sram_buf.size = 0;
		else
			vpu->sram_buf.daddr = dma_map_resource(&pdev->dev,
							       vpu->sram_buf.paddr,
							       vpu->sram_buf.size,
							       DMA_BIDIRECTIONAL,
							       0);

		dev_info(&pdev->dev, "sram daddr: %pad, size: 0x%lx\n",
			 &vpu->sram_buf.daddr, vpu->sram_buf.size);
	}

	ret = devm_clk_bulk_get_all(&pdev->dev, &vpu->clks);
	if (ret < 0) {
		dev_warn(&pdev->dev, "Getting clocks, fail: %d\n", ret);
		ret = 0;
	}
	vpu->num_clks = ret;

	ret = v4l2_device_register(&pdev->dev, &vpu->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register fail: %d\n", ret);
		goto err_dma_unmap;
	}

	ret = coda_vpu_init_m2m_dev(vpu);
	if (ret)
		goto err_v4l2_unregister;

	if (match_data->flags & CODA_IS_ENC) {
		ret = coda_vpu_enc_register_device(vpu);
		if (ret) {
			dev_err(&pdev->dev, "coda_vpu_enc_register_device fail: %d\n", ret);
			goto err_m2m_dev_release;
		}
	}

	vpu->irq = platform_get_irq(pdev, 0);
	if (vpu->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource\n");
		ret = -ENXIO;
		goto err_enc_unreg;
	}

	ret = devm_request_threaded_irq(&pdev->dev, vpu->irq, NULL,
					coda_vpu_irq_thread, IRQF_ONESHOT, "vpu_irq", vpu);
	if (ret) {
		dev_err(&pdev->dev, "failed to register interrupt handler: %d\n", ret);
		goto err_enc_unreg;
	}

	pm_runtime_enable(&pdev->dev);

	return 0;

err_enc_unreg:
	if (match_data->flags & CODA_IS_ENC)
		coda_vpu_enc_unregister_device(vpu);
err_m2m_dev_release:
	coda_vpu_release_m2m_dev(vpu);
err_v4l2_unregister:
	v4l2_device_unregister(&vpu->v4l2_dev);
err_dma_unmap:
	if (vpu->sram_pool && vpu->sram_buf.vaddr) {
		dma_unmap_resource(&pdev->dev,
				   vpu->sram_buf.daddr,
				   vpu->sram_buf.size,
				   DMA_BIDIRECTIONAL,
				   0);
		gen_pool_free(vpu->sram_pool,
			      (unsigned long)vpu->sram_buf.vaddr,
			      vpu->sram_buf.size);
	}
	if (vpu->common_mem.daddr)
		dma_unmap_resource(&pdev->dev,
				   vpu->common_mem.daddr,
				   vpu->common_mem.size,
				   DMA_BIDIRECTIONAL,
				   0);

	return ret;
}

static void coda_vpu_remove(struct platform_device *pdev)
{
	struct vpu_device *vpu = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(vpu->dev);
	coda_vpu_enc_unregister_device(vpu);
	coda_vpu_release_m2m_dev(vpu);
	v4l2_device_unregister(&vpu->v4l2_dev);
	if (vpu->sram_pool && vpu->sram_buf.vaddr) {
		dma_unmap_resource(vpu->dev,
				   vpu->sram_buf.daddr,
				   vpu->sram_buf.size,
				   DMA_BIDIRECTIONAL,
				   0);
		gen_pool_free(vpu->sram_pool,
			      (unsigned long)vpu->sram_buf.vaddr,
			      vpu->sram_buf.size);
	}
	if (vpu->common_mem.daddr)
		dma_unmap_resource(vpu->dev,
				   vpu->common_mem.daddr,
				   vpu->common_mem.size,
				   DMA_BIDIRECTIONAL,
				   0);
	mutex_destroy(&vpu->hw_lock);
	mutex_destroy(&vpu->dev_lock);
	ida_destroy(&vpu->inst_ida);
}

static int __maybe_unused coda_vpu_runtime_suspend(struct device *dev)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = coda_vpuapi_sleep_wake(dev, true);

	clk_bulk_disable_unprepare(vpu->num_clks, vpu->clks);

	return ret;
}

static int __maybe_unused coda_vpu_runtime_resume(struct device *dev)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(vpu->num_clks, vpu->clks);
	if (ret)
		return ret;

	if (!vpu->product_code) {
		ret = coda_vpu_load_firmware(vpu);
		if (ret) {
			dev_err(vpu->dev, "load_firmware fail: %d\n", ret);
			return ret;
		}
	} else {
		ret = coda_vpuapi_sleep_wake(dev, false);
	}

	return ret;
}

static int __maybe_unused coda_vpu_suspend(struct device *dev)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	v4l2_m2m_suspend(vpu->m2m_dev);
	ret = pm_runtime_force_suspend(dev);
	if (ret < 0)
		v4l2_m2m_resume(vpu->m2m_dev);
	return ret;
}

static int __maybe_unused coda_vpu_resume(struct device *dev)
{
	struct vpu_device *vpu = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		return ret;

	v4l2_m2m_resume(vpu->m2m_dev);
	return 0;
}

static const struct dev_pm_ops coda_vpu_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(coda_vpu_suspend,
				coda_vpu_resume)
	SET_RUNTIME_PM_OPS(coda_vpu_runtime_suspend,
			   coda_vpu_runtime_resume, NULL)
};

static const struct of_device_id coda980_dt_ids[] = {
	{ .compatible = "nxp,coda980-vpu", .data = &nxp_coda980_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, coda980_dt_ids);

static struct platform_driver coda_vpu_driver = {
	.driver = {
		.name = VPU_PLATFORM_DEVICE_NAME,
		.of_match_table = coda980_dt_ids,
		.pm = &coda_vpu_pm,
	},
	.probe = coda_vpu_probe,
	.remove = coda_vpu_remove,
};

module_platform_driver(coda_vpu_driver);
MODULE_DESCRIPTION("chips&media VPU V4L2 driver");
MODULE_LICENSE("Dual BSD/GPL");
