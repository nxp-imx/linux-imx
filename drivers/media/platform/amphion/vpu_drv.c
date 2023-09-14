// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 NXP
 */

#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-map-ops.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/of_reserved_mem.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <linux/debugfs.h>
#include "vpu.h"
#include "vpu_imx8q.h"
#include "vpu_core.h"
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <uapi/linux/dma-heap.h>

bool debug;
module_param(debug, bool, 0644);

struct vpu_fastcall_message* fastcall_msg = NULL;
u8* hdr = NULL;
u32 hdr_size = 8192;
struct vpu_ctx vctx;

u32 trusty_vpu_reg(struct device *dev, u32 target, u32 val, u32 w_r) {
        if (w_r == 0x2)
                return trusty_fast_call32(dev, SMC_WV_CONTROL_VPU, target, OPT_WRITE, val);
        if (w_r == 0x1)
                return trusty_fast_call32(dev, SMC_WV_CONTROL_VPU, target, OPT_READ, 0);
        return 0;
}

void copy_wrapper(struct vpu_inst *inst, void *src_virt, void *dst_virt, u32 size) {
	trusty_fast_call32(inst->vpu->trusty_dev, SMC_WV_COPY, vctx.message_buffer_id, 0, 0);
}

void memset_wrapper(struct vpu_inst *inst, void *dst_virt, u32 offset, u32 value, u32 size) {
	trusty_fast_call32(inst->vpu->trusty_dev, SMC_WV_MEMSET, offset, value, size);
}


void vpu_writel(struct vpu_dev *vpu, u32 reg, u32 val)
{
	writel_vpu(val, vpu->base + reg);
}

u32 vpu_readl(struct vpu_dev *vpu, u32 reg)
{
	if (vpu->trusty_dev) {
		return trusty_vpu_reg(vpu->trusty_dev, reg, 0, OPT_READ);
	} else {
		return readl(vpu->base + reg);
	}
}

static void vpu_dev_get(struct vpu_dev *vpu)
{
	if (atomic_inc_return(&vpu->ref_vpu) == 1 && vpu->res->setup)
		vpu->res->setup(vpu);
}

static void vpu_dev_put(struct vpu_dev *vpu)
{
	atomic_dec(&vpu->ref_vpu);
}

static void vpu_enc_get(struct vpu_dev *vpu)
{
	if (atomic_inc_return(&vpu->ref_enc) == 1 && vpu->res->setup_encoder)
		vpu->res->setup_encoder(vpu);
}

static void vpu_enc_put(struct vpu_dev *vpu)
{
	atomic_dec(&vpu->ref_enc);
}

static void vpu_dec_get(struct vpu_dev *vpu)
{
	if (atomic_inc_return(&vpu->ref_dec) == 1 && vpu->res->setup_decoder)
		vpu->res->setup_decoder(vpu);
}

static void vpu_dec_put(struct vpu_dev *vpu)
{
	atomic_dec(&vpu->ref_dec);
}

static int vpu_init_media_device(struct vpu_dev *vpu)
{
	vpu->mdev.dev = vpu->dev;
	strscpy(vpu->mdev.model, "amphion-vpu", sizeof(vpu->mdev.model));
	strscpy(vpu->mdev.bus_info, "platform: amphion-vpu", sizeof(vpu->mdev.bus_info));
	media_device_init(&vpu->mdev);
	vpu->v4l2_dev.mdev = &vpu->mdev;

	return 0;
}

static int vpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpu_dev *vpu;
	int ret;
	struct device_node *sp = NULL;

	dev_dbg(dev, "probe\n");
	vpu = devm_kzalloc(dev, sizeof(*vpu), GFP_KERNEL);
	if (!vpu)
		return -ENOMEM;

	vpu->pdev = pdev;
	vpu->dev = dev;
	mutex_init(&vpu->lock);
	mutex_init(&vpu->copy_lock);
	mutex_init(&vpu->memset_lock);
	mutex_init(&vpu->hdr_lock);
	INIT_LIST_HEAD(&vpu->cores);
	platform_set_drvdata(pdev, vpu);
	atomic_set(&vpu->ref_vpu, 0);
	atomic_set(&vpu->ref_enc, 0);
	atomic_set(&vpu->ref_dec, 0);
	vpu->get_vpu = vpu_dev_get;
	vpu->put_vpu = vpu_dev_put;
	vpu->get_enc = vpu_enc_get;
	vpu->put_enc = vpu_enc_put;
	vpu->get_dec = vpu_dec_get;
	vpu->put_dec = vpu_dec_put;

	vpu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpu->base))
		return PTR_ERR(vpu->base);
	vpu_malone_base = vpu->base;

	vpu->res = of_device_get_match_data(dev);
	if (!vpu->res)
		return -ENODEV;

	pm_runtime_enable(dev);

	ret = v4l2_device_register(dev, &vpu->v4l2_dev);
	if (ret)
		goto err_vpu_deinit;

	vpu_init_media_device(vpu);
	vpu->encoder.type = VPU_CORE_TYPE_ENC;
	vpu->encoder.function = MEDIA_ENT_F_PROC_VIDEO_ENCODER;
	vpu->decoder.type = VPU_CORE_TYPE_DEC;
	vpu->decoder.function = MEDIA_ENT_F_PROC_VIDEO_DECODER;

	/*check trusty node*/
	if (vpu->res->plat_type == IMX8QM) {
		sp = of_find_node_by_name(NULL, "trusty");
		if (sp != NULL) {
			struct platform_device *pd;
			pd = of_find_device_by_node(sp);
			if (pd != NULL) {
				vpu->trusty_dev = &(pd->dev);
				dev_info(dev,"vpu amphion will use trusty mode\n");
				ret = trusty_fast_call32(vpu->trusty_dev, SMC_WV_PROBE, 0, 0, 0);
				if (ret < 0) {
					dev_err(dev, "trusty dev failed to probe!nr=0x%x error=%d\n", SMC_WV_PROBE, ret);
					vpu->trusty_dev = NULL;
				} else {
					dev_info(dev,"vpu amphion will use trusty mode\n");
					trusty_dev = vpu->trusty_dev;
				}
			} else {
				vpu->trusty_dev = NULL;
			}
		} else {
			vpu->trusty_dev = NULL;
		}
	} else {
		vpu->trusty_dev = NULL;
	}
	/* end */

	ret = vpu_add_func(vpu, &vpu->decoder);
	if (ret)
		goto err_add_decoder;
	ret = vpu_add_func(vpu, &vpu->encoder);
	if (ret)
		goto err_add_encoder;
	ret = media_device_register(&vpu->mdev);
	if (ret)
		goto err_vpu_media;
	vpu->debugfs = debugfs_create_dir("amphion_vpu", NULL);

	of_platform_populate(dev->of_node, NULL, NULL, dev);

	if (vpu->trusty_dev && vpu->res->plat_type == IMX8QM) {
		vctx.message_buffer_sz = PAGE_ALIGN(sizeof(trusty_shared_mem_id_t) + 16);
		vctx.message_buffer = alloc_pages_exact(vctx.message_buffer_sz, GFP_KERNEL);
		fastcall_msg = (struct vpu_fastcall_message*)vctx.message_buffer;
		sg_init_one(&vctx.message_sg, vctx.message_buffer, vctx.message_buffer_sz);

		ret = trusty_share_memory_compat(vpu->trusty_dev,
				&vctx.message_buffer_id, &vctx.message_sg, 1, PAGE_KERNEL);
		if (ret) {
			dev_err(dev, "vpu share messgae buffer failed ret=0x%d\n",ret);
		}

		ret = trusty_fast_call32(vpu->trusty_dev, SMC_WV_MESSAGE_BUFFER,
				(vctx.message_buffer_id & 0xffffffff), (vctx.message_buffer_id >> 32),vctx.message_buffer_sz);

		vctx.hdr_buffer_sz = hdr_size;
		vctx.hdr_buffer = alloc_pages_exact(vctx.hdr_buffer_sz, GFP_KERNEL);
		sg_init_one(&vctx.hdr_sg, vctx.hdr_buffer, vctx.hdr_buffer_sz);
		hdr = vctx.hdr_buffer;
		ret = trusty_share_memory_compat(vpu->trusty_dev,
				&vctx.hdr_buffer_id, &vctx.hdr_sg, 1, PAGE_KERNEL);
		if (ret)
			dev_err(dev, "vpu share hdr buffer failed ret=0x%d\n",ret);
		else
		ret = trusty_fast_call32(vpu->trusty_dev, SMC_WV_HDR, (vctx.hdr_buffer_id & 0xffffffff),
				(vctx.hdr_buffer_id >> 32), vctx.hdr_buffer_sz);

		//mmap secure heap, message, hdr shared memory
		ret = trusty_fast_call32(vpu->trusty_dev, SMC_WV_MMAP_SAHRE_MEMORY, 1, 0, 0);
		if (ret)
			dev_err(dev, "decoder mmap shared buffer failed\n");
	}

	return 0;

err_vpu_media:
	vpu_remove_func(&vpu->encoder);
err_add_encoder:
	vpu_remove_func(&vpu->decoder);
err_add_decoder:
	media_device_cleanup(&vpu->mdev);
	v4l2_device_unregister(&vpu->v4l2_dev);
err_vpu_deinit:
	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev *vpu = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	if (vpu->trusty_dev && vpu->res->plat_type == IMX8QM) {
		/* unmmap trusty shared memory */
		ret = trusty_fast_call32(vpu->trusty_dev, SMC_WV_MMAP_SAHRE_MEMORY, 0, 0, 0);
		if (ret)
			dev_err(dev, "decoder unmmap shared buffer failed\n");

		/* free hdr shared memory */
		ret = trusty_reclaim_memory(vpu->trusty_dev, vctx.hdr_buffer_id,
				&vctx.hdr_sg, 1);
		if (WARN_ON(ret)) {
			dev_err(dev, "vpu_revoke_hdr_shared_memory failed: %d 0x%llx\n",
						ret, vctx.hdr_buffer_id);
		} else {
			free_pages_exact(vctx.hdr_buffer, vctx.hdr_buffer_sz);
			fastcall_msg = NULL;
		}
		/* free fastcall message memory */
		ret = trusty_reclaim_memory(vpu->trusty_dev, vctx.message_buffer_id,
				&vctx.message_sg, 1);
		if (WARN_ON(ret)) {
			dev_err(dev, "vpu_revoke_message_shared_memory failed: %d 0x%llx\n",
				ret, vctx.message_buffer_id);
		} else {
			free_pages_exact(vctx.message_buffer, vctx.message_buffer_sz);
			hdr = NULL;
		}
		/* free shared memory end */
	}

	debugfs_remove_recursive(vpu->debugfs);
	vpu->debugfs = NULL;

	pm_runtime_disable(dev);

	media_device_unregister(&vpu->mdev);
	vpu_remove_func(&vpu->decoder);
	vpu_remove_func(&vpu->encoder);
	media_device_cleanup(&vpu->mdev);
	v4l2_device_unregister(&vpu->v4l2_dev);
	mutex_destroy(&vpu->lock);
	mutex_destroy(&vpu->copy_lock);
	mutex_destroy(&vpu->memset_lock);
	mutex_destroy(&vpu->hdr_lock);

	return 0;
}

static int __maybe_unused vpu_runtime_resume(struct device *dev)
{
	return 0;
}

static int __maybe_unused vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused vpu_resume(struct device *dev)
{
	return 0;
}

static int __maybe_unused vpu_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};

static struct vpu_resources imx8qxp_res = {
	.plat_type = IMX8QXP,
	.mreg_base = 0x40000000,
	.setup = vpu_imx8q_setup,
	.setup_encoder = vpu_imx8q_setup_enc,
	.setup_decoder = vpu_imx8q_setup_dec,
	.reset = vpu_imx8q_reset
};

static struct vpu_resources imx8qm_res = {
	.plat_type = IMX8QM,
	.mreg_base = 0x40000000,
	.setup = vpu_imx8q_setup,
	.setup_encoder = vpu_imx8q_setup_enc,
	.setup_decoder = vpu_imx8q_setup_dec,
	.reset = vpu_imx8q_reset
};

static const struct of_device_id vpu_dt_match[] = {
	{ .compatible = "nxp,imx8qxp-vpu", .data = &imx8qxp_res },
	{ .compatible = "nxp,imx8qm-vpu", .data = &imx8qm_res },
	{}
};
MODULE_DEVICE_TABLE(of, vpu_dt_match);

static struct platform_driver amphion_vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.driver = {
		.name = "amphion-vpu",
		.of_match_table = vpu_dt_match,
		.pm = &vpu_pm_ops,
	},
};

static int __init vpu_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&amphion_vpu_driver);
	if (ret)
		return ret;

	ret = vpu_core_driver_init();
	if (ret)
		platform_driver_unregister(&amphion_vpu_driver);

	return ret;
}

static void __exit vpu_driver_exit(void)
{
	vpu_core_driver_exit();
	platform_driver_unregister(&amphion_vpu_driver);
}
module_init(vpu_driver_init);
module_exit(vpu_driver_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX8Q");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(DMA_BUF);
