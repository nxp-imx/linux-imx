// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - platform driver
 *
 * Copyright (C) 2026 CHIPS&MEDIA INC
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/iopoll.h>
#include <linux/of_platform.h>
#include "wave5-vpu.h"
#include "wave5-regdefine.h"
#include "wave5-vpuconfig.h"
#include "wave5-hw.h"
#include "wave5-vpu-dbg.h"

#define VPU_PLATFORM_DEVICE_NAME "wave5-vpu"
#define VPU_CLK_NAME "vcodec"

#define WAVE5_IS_DEC BIT(1)

struct wave5_match_data {
	int flags;
	const char *fw_name;
	u32 compatible_fw_api_version;
};

static const struct wave5_match_data nxp_wave511_data = {
	.flags = WAVE5_IS_DEC,
	.fw_name = "cnm/wave511_dec_fw.bin",
	.compatible_fw_api_version = 0x1000000,
};

static int vpu_poll_interval = 5;
module_param(vpu_poll_interval, int, 0644);

int wave5_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_timeout(&inst->irq_done,
					  msecs_to_jiffies(timeout));
	if (!ret)
		return -ETIMEDOUT;

	reinit_completion(&inst->irq_done);

	return 0;
}

void wave5_vpu_enable_instance(struct vpu_instance *inst)
{
	if (!inst)
		return;

	scoped_guard(spinlock_irqsave, &inst->dev->inst_lock) {
		atomic_set(&inst->refcount, 0);
		inst->enable = true;
	}
}

void wave5_vpu_disable_instance(struct vpu_instance *inst)
{
	int count;
	int ret;

	if (!inst)
		return;

	scoped_guard(spinlock_irqsave, &inst->dev->inst_lock)
		inst->enable = false;

	ret = read_poll_timeout(atomic_read, count, !count,
				VPU_POLL_CHECK_INTERVAL, VPU_BUSY_CHECK_TIMEOUT,
				false, &inst->refcount);
	if (ret)
		dev_err(inst->dev->dev, "disable instance[%d] timeout\n", inst->id);
}

static struct vpu_instance *wave5_vpu_get_instance(struct vpu_device *dev, u32 mask)
{
	struct vpu_instance *inst;

	if (!mask)
		return NULL;

	scoped_guard(spinlock_irqsave, &dev->inst_lock) {
		list_for_each_entry(inst, &dev->instances, list) {
			if (mask & BIT(inst->id)) {
				if (!inst->enable)
					return NULL;

				atomic_inc(&inst->refcount);
				return inst;
			}
		}
	}

	return NULL;
}

static void wave5_vpu_put_instance(struct vpu_instance *inst)
{
	if (!inst)
		return;

	scoped_guard(spinlock_irqsave, &inst->dev->inst_lock)
		atomic_dec_if_positive(&inst->refcount);
}

static irqreturn_t wave5_vpu_irq(int irq, void *dev_id)
{
	struct vpu_device *dev = dev_id;
	struct vpu_irq_status status;
	struct vpu_instance *inst;
	int i;

	if (!wave5_vdi_read_register(dev, W5_VPU_VPU_INT_STS))
		return IRQ_NONE;

	status.reason = wave5_vdi_read_register(dev, W5_VPU_VINT_REASON);
	status.cmd_done = wave5_vdi_read_register(dev, W5_RET_QUEUE_CMD_DONE_INST);
	status.seq_done = wave5_vdi_read_register(dev, W5_RET_SEQ_DONE_INSTANCE_INFO);

	wave5_vdi_write_register(dev, W5_VPU_VINT_REASON_CLR, status.reason);
	wave5_vdi_write_register(dev, W5_VPU_VINT_CLEAR, 0x1);
	wave5_vpu_clear_interrupt(dev, status.reason);

	dev_dbg(dev->dev, "%s: reason 0x%x, cmd_done 0x%x, seq_done 0x%x\n",
		__func__, status.reason, status.cmd_done, status.seq_done);

	if (status.reason & BIT(INT_WAVE5_REQ_WORK_BUF)) {
		if (dev->ctrl)
			wave5_vpu_ctrl_require_buffer(dev->ctrl, &dev->entity);

		status.reason &= ~BIT(INT_WAVE5_REQ_WORK_BUF);
	}

	if (status.reason & BIT(INT_WAVE5_INIT_SEQ)) {
		unsigned long seq_mask = status.seq_done;

		if (dev->product_code == WAVE515_CODE)
			seq_mask = status.cmd_done;

		for_each_set_bit(i, &seq_mask, MAX_NUM_INSTANCE) {
			inst = wave5_vpu_get_instance(dev, BIT(i));
			if (inst) {
				complete(&inst->irq_done);
				wave5_vpu_put_instance(inst);

				if (dev->product_code == WAVE515_CODE)
					status.cmd_done &= ~BIT(i);
			}
		}

		status.reason &= ~BIT(INT_WAVE5_INIT_SEQ);
	}

	if (status.reason) {
		if (!kfifo_in_spinlocked_noirqsave(&dev->irq_fifo,
						   &status,
						   sizeof(status),
						   &dev->irq_lock))
			dev_warn(dev->dev, "IRQ fifo full, interrupt may be lost\n");

		return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

static irqreturn_t wave5_vpu_irq_thread(int irq, void *dev_id)
{
	struct vpu_device *dev = dev_id;
	struct vpu_irq_status status;
	struct vpu_instance *inst;
	int i;

	while (kfifo_out_spinlocked(&dev->irq_fifo, &status, sizeof(status),
				    &dev->irq_lock)) {
		if (status.reason & BIT(INT_WAVE5_DEC_PIC)) {
			unsigned long cmd_mask = status.cmd_done;

			for_each_set_bit(i, &cmd_mask, MAX_NUM_INSTANCE) {
				inst = wave5_vpu_get_instance(dev, BIT(i));
				if (inst) {
					inst->ops->finish_process(inst);
					wave5_vpu_put_instance(inst);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

static void wave5_vpu_handle_irq(void *dev_id)
{
	u32 seq_done;
	u32 cmd_done;
	u32 irq_reason;
	struct vpu_instance *inst;
	struct vpu_device *dev = dev_id;

	irq_reason = wave5_vdi_read_register(dev, W5_VPU_VINT_REASON);
	seq_done = wave5_vdi_read_register(dev, W5_RET_SEQ_DONE_INSTANCE_INFO);
	cmd_done = wave5_vdi_read_register(dev, W5_RET_QUEUE_CMD_DONE_INST);

	dev_dbg(dev->dev, "%s: irq_reason 0x%x, seq_done 0x%x, cmd_done 0x%x\n",
		__func__, irq_reason, seq_done, cmd_done);

	if (irq_reason & BIT(INT_WAVE5_REQ_WORK_BUF)) {
		if (dev->ctrl)
			wave5_vpu_ctrl_require_buffer(dev->ctrl, &dev->entity);
	}

	for (int i = 0; i < MAX_NUM_INSTANCE; i++) {
		u32 mask = BIT(i);

		if (irq_reason & BIT(INT_WAVE5_INIT_SEQ)) {
			if (dev->product_code == WAVE515_CODE) {
				inst = wave5_vpu_get_instance(dev, mask & cmd_done);
				if (inst) {
					cmd_done &= ~mask;
					wave5_vdi_write_register(dev, W5_RET_QUEUE_CMD_DONE_INST,
								 cmd_done);
					complete(&inst->irq_done);
					wave5_vpu_put_instance(inst);
				}
			} else {
				inst = wave5_vpu_get_instance(dev, mask & seq_done);
				if (inst) {
					seq_done &= ~mask;
					wave5_vdi_write_register(dev, W5_RET_SEQ_DONE_INSTANCE_INFO,
								 seq_done);
					complete(&inst->irq_done);
					wave5_vpu_put_instance(inst);
				}
			}
		}
		if (irq_reason & BIT(INT_WAVE5_DEC_PIC)) {
			inst = wave5_vpu_get_instance(dev, mask & cmd_done);
			if (inst) {
				cmd_done &= ~mask;
				wave5_vdi_write_register(dev, W5_RET_QUEUE_CMD_DONE_INST,
							 cmd_done);
				inst->ops->finish_process(inst);
				wave5_vpu_put_instance(inst);
			}
		}
	}

	wave5_vdi_write_register(dev, W5_VPU_VINT_REASON_CLR, irq_reason);
	wave5_vdi_write_register(dev, W5_VPU_VINT_CLEAR, 0x1);
	wave5_vpu_clear_interrupt(dev, irq_reason);
}

static void wave5_vpu_irq_work_fn(struct kthread_work *work)
{
	struct vpu_device *dev = container_of(work, struct vpu_device, work);

	if (wave5_vdi_read_register(dev, W5_VPU_VPU_INT_STS))
		wave5_vpu_handle_irq(dev);
}

static enum hrtimer_restart wave5_vpu_timer_callback(struct hrtimer *timer)
{
	struct vpu_device *dev =
			container_of(timer, struct vpu_device, hrtimer);

	kthread_queue_work(dev->worker, &dev->work);
	hrtimer_forward_now(timer, ns_to_ktime(vpu_poll_interval * NSEC_PER_MSEC));

	return HRTIMER_RESTART;
}

static u32 wave5_vpu_read_reg(struct device *dev, u32 addr)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	return wave5_vdi_read_register(vpu_dev, addr);
}

static void wave5_vpu_write_reg(struct device *dev, u32 addr, u32 data)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	wave5_vdi_write_register(vpu_dev, addr, data);
}

static void wave5_vpu_on_boot(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	struct vpu_attr *p_attr = &vpu_dev->attr;
	u32 revision;
	unsigned int product_id;
	int ret;

	vpu_dev->product_code = wave5_vdi_read_register(vpu_dev, W5_VPU_RET_PRODUCT_VERSION);
	vpu_dev->product = wave5_vpu_get_product_id(vpu_dev);

	ret = wave5_vpu_get_version_info(dev, &revision, &product_id);
	if (ret) {
		dev_err(dev, "vpu_get_version_info fail: %d\n", ret);
		return;
	}

	dev_dbg(dev, "%s: enum product_id: %08x, fw version: %d.%d.%d(r%u)\n",
		__func__, product_id,
		(p_attr->fw_api_version >> 24) & 0xFF,
		(p_attr->fw_api_version >> 16) & 0xFF,
		(p_attr->fw_api_version >> 0) & 0xFFFF,
		revision);

	if (vpu_dev->res->compatible_fw_api_version > p_attr->fw_api_version)
		dev_err(dev, "compatible fw version is v%d.%d.%d or higher, but only v%d.%d.%d\n",
			(vpu_dev->res->compatible_fw_api_version >> 24) & 0xFF,
			(vpu_dev->res->compatible_fw_api_version >> 16) & 0xFF,
			(vpu_dev->res->compatible_fw_api_version >> 0) & 0xFFFF,
			(p_attr->fw_api_version >> 24) & 0xFF,
			(p_attr->fw_api_version >> 16) & 0xFF,
			(p_attr->fw_api_version >> 0) & 0xFFFF);
}

static void wave5_vpu_scan_instances(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	struct dec_open_param open_param = { 0 };
	struct vpu_instance *instances;
	struct vpu_instance *inst;
	int count = 0;
	int ret;

	instances = kcalloc(MAX_NUM_INSTANCE, sizeof(struct vpu_instance), GFP_KERNEL);
	if (!instances)
		return;

	open_param.reorder_enable = true;
	for (int i = 0; i < MAX_NUM_INSTANCE; i++) {
		inst = &instances[i];
		inst->dev = vpu_dev;
		inst->type = VPU_INST_TYPE_DEC;
		inst->std = W_HEVC_DEC;
		inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
		if (!inst->codec_info)
			break;
		ret = wave5_vpu_dec_open(inst, &open_param);
		if (ret)
			break;
		count++;
	}

	for (int i = 0; i < count; i++) {
		u32 fail_res = 0;

		inst = &instances[i];
		wave5_vpu_dec_close(inst, &fail_res);
		kfree(inst->codec_info);
	}

	kfree(instances);
	dev_dbg(dev, "scan %d instances\n", count);
}

u32 wave5_vpu_cq_depth(struct vpu_device *vpu_dev)
{
	if (vpu_dev->product_code == WAVE515_CODE)
		return WAVE515_COMMAND_QUEUE_DEPTH;
	else
		return WAVE521_COMMAND_QUEUE_DEPTH;
}

static bool wave5_vpu_cq_is_empty(struct vpu_device *vpu_dev)
{
	struct vpu_instance *inst;

	scoped_guard(spinlock_irqsave, &vpu_dev->inst_lock) {
		list_for_each_entry(inst, &vpu_dev->instances, list) {
			if (atomic_read(&inst->queued_dec_cmd) && !inst->dynamic_source_change)
				return false;
		}
	}

	return true;
}

static int wave5_vpu_cq_wait_empty(struct vpu_device *dev)
{
	const unsigned long timeout = wave5_vpu_cq_depth(dev) * VPU_DEC_TIMEOUT_US;
	bool empty;
	int ret;

	ret = read_poll_timeout(wave5_vpu_cq_is_empty, empty, empty,
				VPU_POLL_CHECK_INTERVAL, timeout, false, dev);
	if (ret) {
		dev_err(dev->dev, "wait CQ empty timeout\n");
		return ret;
	}

	return 0;
}

void wave5_vpu_activate(struct vpu_device *dev)
{
	dev->active = true;
}

void wave5_vpu_wait_activated(struct vpu_device *dev)
{
	wave5_vpu_check_state(dev);
}

static int wave5_vpu_probe(struct platform_device *pdev)
{
	int ret;
	struct vpu_device *dev;
	struct device_node *np;
	const struct wave5_match_data *match_data;

	match_data = device_get_match_data(&pdev->dev);
	if (!match_data) {
		dev_err(&pdev->dev, "missing device match data\n");
		return -EINVAL;
	}

	// TODO support 40bit address.
	/* physical addresses limited to 32 bits */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set DMA mask: %d\n", ret);
		return ret;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->vdb_register = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dev->vdb_register))
		return PTR_ERR(dev->vdb_register);

	np = of_parse_phandle(pdev->dev.of_node, "cnm,ctrl", 0);
	if (np) {
		struct platform_device *pctrl = of_find_device_by_node(np);

		of_node_put(np);
		if (pctrl) {
			dev->ctrl = &pctrl->dev;
			if (wave5_vpu_ctrl_get_state(dev->ctrl) < 0) {
				dev_info(&pdev->dev, "vpu ctrl is not ready, defer probe\n");
				return -EPROBE_DEFER;
			}
		} else {
			dev_info(&pdev->dev, "vpu ctrl is not found\n");
			return -EINVAL;
		}
	} else {
		dev_info(&pdev->dev, "it's a follower vpu device\n");
	}

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->hw_lock);
	spin_lock_init(&dev->inst_lock);
	dev_set_drvdata(&pdev->dev, dev);
	dev->dev = &pdev->dev;
	dev->res = match_data;

	dev->entity.dev = dev->dev;
	dev->entity.read_reg = wave5_vpu_read_reg;
	dev->entity.write_reg = wave5_vpu_write_reg;
	dev->entity.on_boot = wave5_vpu_on_boot;
	dev->entity.scan_instances = wave5_vpu_scan_instances;

	dev->resets = devm_reset_control_array_get_optional_exclusive(&pdev->dev);
	if (IS_ERR(dev->resets)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(dev->resets),
				     "Failed to get reset control\n");
	}

	ret = reset_control_deassert(dev->resets);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to deassert resets\n");

	ret = devm_clk_bulk_get_all(&pdev->dev, &dev->clks);

	/* continue without clock, assume externally managed */
	if (ret < 0) {
		dev_warn(&pdev->dev, "Getting clocks, fail: %d\n", ret);
		ret = 0;
	}
	dev->num_clks = ret;

	spin_lock_init(&dev->irq_lock);
	ret = kfifo_alloc(&dev->irq_fifo,
			  MAX_NUM_INSTANCE * sizeof(struct vpu_irq_status),
			  GFP_KERNEL);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate IRQ fifo\n");
		goto err_reset_assert;
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource, falling back to polling\n");
		hrtimer_setup(&dev->hrtimer, wave5_vpu_timer_callback, CLOCK_MONOTONIC,
			      HRTIMER_MODE_REL_PINNED);
		dev->worker = kthread_create_worker(0, "vpu_irq_thread");
		if (IS_ERR(dev->worker)) {
			dev_err(&pdev->dev, "failed to create vpu irq worker\n");
			ret = PTR_ERR(dev->worker);
			goto err_irq_fifo_free;
		}
		dev->vpu_poll_interval = vpu_poll_interval;
		kthread_init_work(&dev->work, wave5_vpu_irq_work_fn);
	} else {
		ret = devm_request_threaded_irq(&pdev->dev, dev->irq,
						wave5_vpu_irq,
						wave5_vpu_irq_thread,
						0, "vpu_irq", dev);
		if (ret) {
			dev_err(&pdev->dev, "Register interrupt handler, fail: %d\n", ret);
			goto err_irq_fifo_free;
		}
	}

	dev->temp_vbuf.size = ALIGN(WAVE5_TEMPBUF_SIZE, 4096);
	ret = wave5_vdi_allocate_dma_memory(&pdev->dev, &dev->temp_vbuf);
	if (ret) {
		dev_err(&pdev->dev, "alloc temp of size %zu failed\n", dev->temp_vbuf.size);
		goto err_irq_fifo_free;
	}

	INIT_LIST_HEAD(&dev->instances);
	pm_runtime_enable(&pdev->dev);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register, fail: %d\n", ret);
		goto err_temp_vbuf_free;
	}

	if (dev->res->flags & WAVE5_IS_DEC) {
		ret = wave5_vpu_dec_register_device(dev);
		if (ret) {
			dev_err(&pdev->dev, "wave5_vpu_dec_register_device, fail: %d\n", ret);
			goto err_v4l2_unregister;
		}
	}

	if (dev->ctrl && wave5_vpu_ctrl_support_follower(dev->ctrl)) {
		wave5_vpu_activate(dev);
		ret = pm_runtime_resume_and_get(&pdev->dev);
		if (ret)
			goto err_dec_unreg;
	}

	dev->debugfs = debugfs_lookup(WAVE5_VPU_DEBUGFS_DIR, NULL);
	if (IS_ERR_OR_NULL(dev->debugfs))
		dev->debugfs = debugfs_create_dir(WAVE5_VPU_DEBUGFS_DIR, NULL);

	dev_info(&pdev->dev, "Added wave5 driver with caps: %s\n",
		 (dev->res->flags & WAVE5_IS_DEC) ? "'DECODE'" : "");
	return 0;

err_dec_unreg:
	if (dev->res->flags & WAVE5_IS_DEC)
		wave5_vpu_dec_unregister_device(dev);
err_v4l2_unregister:
	v4l2_device_unregister(&dev->v4l2_dev);
err_temp_vbuf_free:
	wave5_vdi_free_dma_memory(&dev->temp_vbuf);
err_irq_fifo_free:
	kfifo_free(&dev->irq_fifo);
err_reset_assert:
	reset_control_assert(dev->resets);

	return ret;
}

static void wave5_vpu_remove(struct platform_device *pdev)
{
	struct vpu_device *dev = dev_get_drvdata(&pdev->dev);

	if (dev->ctrl && wave5_vpu_ctrl_support_follower(dev->ctrl)) {
		if (!pm_runtime_suspended(&pdev->dev))
			pm_runtime_put_sync(&pdev->dev);
	}
	pm_runtime_disable(&pdev->dev);

	if (dev->irq < 0) {
		kthread_destroy_worker(dev->worker);
		hrtimer_cancel(&dev->hrtimer);
	}

	mutex_destroy(&dev->dev_lock);
	mutex_destroy(&dev->hw_lock);
	kfifo_free(&dev->irq_fifo);
	reset_control_assert(dev->resets);
	wave5_vpu_dec_unregister_device(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	wave5_vdi_free_dma_memory(&dev->temp_vbuf);
}

#ifdef CONFIG_PM
static int wave5_vpu_runtime_suspend(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);

	if (!vpu_dev)
		return -ENODEV;

	if (vpu_dev->ctrl && vpu_dev->active)
		wave5_vpu_ctrl_put_sync(vpu_dev->ctrl, &vpu_dev->entity);
	if (vpu_dev->num_clks)
		clk_bulk_disable_unprepare(vpu_dev->num_clks, vpu_dev->clks);

	return 0;
}

static int wave5_vpu_runtime_resume(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret = 0;

	if (!vpu_dev)
		return -ENODEV;

	if (vpu_dev->num_clks) {
		ret = clk_bulk_prepare_enable(vpu_dev->num_clks, vpu_dev->clks);
		if (ret) {
			dev_err(dev, "failed to enable clocks: %d\n", ret);
			return ret;
		}
	}

	if (vpu_dev->ctrl && vpu_dev->active) {
		ret = wave5_vpu_ctrl_resume_and_get(vpu_dev->ctrl, &vpu_dev->entity);
		if (ret && vpu_dev->num_clks)
			clk_bulk_disable_unprepare(vpu_dev->num_clks, vpu_dev->clks);
	} else {
		wave5_vpu_check_state(vpu_dev);
	}

	return ret;
}
#endif
#ifdef CONFIG_PM_SLEEP
static int wave5_vpu_suspend(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	v4l2_m2m_suspend(vpu_dev->v4l2_m2m_dec_dev);
	wave5_vpu_cq_wait_empty(vpu_dev);

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		v4l2_m2m_resume(vpu_dev->v4l2_m2m_dec_dev);

	return ret;
}

static int wave5_vpu_resume(struct device *dev)
{
	struct vpu_device *vpu_dev = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	v4l2_m2m_resume(vpu_dev->v4l2_m2m_dec_dev);

	return 0;
}
#endif
static const struct dev_pm_ops wave5_vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(wave5_vpu_runtime_suspend, wave5_vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(wave5_vpu_suspend, wave5_vpu_resume)
};

static const struct of_device_id wave5_dt_ids[] = {
	{ .compatible = "nxp,wave511-vpu", .data = &nxp_wave511_data},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wave5_dt_ids);

static struct platform_driver wave5_vpu_driver = {
	.driver = {
		.name = VPU_PLATFORM_DEVICE_NAME,
		.of_match_table = of_match_ptr(wave5_dt_ids),
		.pm = &wave5_vpu_pm_ops,
		},
	.probe = wave5_vpu_probe,
	.remove = wave5_vpu_remove,
};

module_platform_driver(wave5_vpu_driver);
MODULE_DESCRIPTION("chips&media VPU V4L2 driver");
MODULE_LICENSE("Dual BSD/GPL");
