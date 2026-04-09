// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2026 NXP
 */

#include <linux/kthread.h>
#include <linux/of_address.h>
#include <linux/types.h>

#include "dpu95.h"
#include "dpu95-drv.h"
#include "dpu95-ld.h"
#include "dpu95-ld-regs.h"
#include "dpu95-ld-regs-mbi6535.h"

#define DPU952_IRQ_LOCALDIMMING0_IRQ1 33

/* TODO: we need to define these into our FW api */
struct dpu95_ldfw_cmd {
	u32 cmd_data2;
	u32 cmd_data1;
	u32 cmd_data0;
	u32 cmd_id;
} __packed __aligned(4);
#define DPU95_LD_FW_READY	0x0A
#define DPU95_LD_FW_CFG		0x03
#define DPU95_LD_FW_START	0x04
#define DPU95_LD_FW_STOP	0x05

static struct dpu95_ld_cfg mbi6353_cfg = {
	.led_size = 192,
	.h_led = 48,
	.v_led = 4
};

static inline u32 dpu95_ld_read(struct dpu95_localdimming *ld, u32 offset)
{
	return readl(ld->base + offset);
}

static inline void dpu95_ld_write(struct dpu95_localdimming *ld, u32 offset, u32 value)
{
	writel(value, ld->base + offset);
}

static inline u32 dpu95_disp_irq0_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->disp_irq0_reg + offset);
}

static inline void dpu95_disp_irq0_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->disp_irq0_reg + offset);
}

static void dpu95_irq_set(struct dpu95_soc *dpu, int irq, bool enable)
{
	u32 regval;
	u32 offset = INTERRUPTENABLE(irq / 32);

	regval = dpu95_disp_irq0_read(dpu, offset);

	if (enable)
		regval |= (1 << (irq % 32));
	else
		regval &= ~(1 << (irq % 32));

	dpu95_disp_irq0_write(dpu, offset, regval);
}

static void dpu95_irq_clear(struct dpu95_soc *dpu, int irq)
{
	u32 offset = INTERRUPTCLEAR(irq / 32);

	dpu95_disp_irq0_write(dpu, offset, (1 << (irq % 32)));
}

static void dpu95_ld_set_intr(struct dpu95_localdimming *ld,
			      struct dpu95_ld_intr *intr)
{
	u32 data_hifreg00a;
	u32 data_hifreg00b;

	data_hifreg00a = dpu95_ld_read(ld, LD_REG_LDHIFREG00A);
	data_hifreg00b = dpu95_ld_read(ld, LD_REG_LDHIFREG00B);

	data_hifreg00a |= LDHIFREG00A_LDVINTDLY_SET(intr->ldvintdly);
	data_hifreg00b |= LDHIFREG00B_SMSLVON_SET(intr->smslvon) |
			  LDHIFREG00B_SMVLVON_SET(intr->smvlvon) |
			  LDHIFREG00B_SMBLVON_SET(intr->smblvon) |
			  LDHIFREG00B_VINTON_SET(intr->vinton) |
			  LDHIFREG00B_LEDINTON_SET(intr->ledinton) |
			  LDHIFREG00B_BLINTON_SET(intr->blinton);

	dpu95_ld_write(ld, LD_REG_LDHIFREG00A, data_hifreg00a);
	dpu95_ld_write(ld, LD_REG_LDHIFREG00B, data_hifreg00b);
}

static void dpu95_ldfw_mu_callback(struct disp_mu_client *cl, void *msg)
{
	struct dpu95_ld_fw *fw = container_of(cl, struct dpu95_ld_fw, mu);
	struct dpu95_localdimming *ld = container_of(fw, struct dpu95_localdimming, fw);
	struct dpu95_ldfw_cmd *cmd = msg;

	dev_err(ld->dpu->dev, "ID:0x%02X, D0:0x%08X, D1:0x%08X, D2:0x%08X\n",
		cmd->cmd_id,
		cmd->cmd_data0, cmd->cmd_data1, cmd->cmd_data2);

	switch (cmd->cmd_id) {
	case DPU95_LD_FW_READY:
		ld->fw.ready = true;
		/*
		 * Core needs to be enable first. If ld core is already enabled,
		 * we can go ahead with fw enablement.
		 */
		if (ld->enabled)
			dpu95_ld_enable(ld);
		break;

	case DPU95_LD_FW_CFG:
		break;

	case DPU95_LD_FW_START:
		ld->fw.enabled = true;
		break;

	case DPU95_LD_FW_STOP:
		ld->fw.enabled = false;
		break;
	}
}

static int dpu95_ldfw_rproc_boot_thread(void *arg)
{
	struct dpu95_localdimming *ld = arg;
	struct device *dev = ld->dpu->dev;
	struct rproc *rproc = ld->fw.rproc;
	int ret;

	/* Request a boot from the remote processor */
	ret = rproc_boot(rproc);
	if (ret)
		dev_err(dev, "LD-FW rproc_boot failed: %d\n", ret);

	return ret;
}

static void dpu95_ld_ldim_init_default_regs(struct dpu95_localdimming *ld,
					    struct dpu95_ld_reg_def *regs,
					    unsigned int size)
{
	int i;

	for (i = 0; i < size; i++)
		dpu95_ld_write(ld, regs[i].offset, regs[i].value);
}

int dpu95_ld_mode_set(struct dpu95_localdimming *ld,
		      struct drm_display_mode *mode)
{
	int ret = 0;

	if (!ld)
		return 0;

	/*
	 * Copy display mode if the mode_set event occurs before the LD module is
	 * powered on.
	 * The LD registers are accessible only after VSYNC generation is on, so
	 * copy the mode here and apply it again during LD power-on.
	 */
	ld->mode = *mode;

	dpu95_ld_write(ld, LD_REG_CLKCTRL0, CLKCTRL0_GLOBCLKEN);

	dpu95_ld_write(ld, LD_REG_CLKCTRL1, CLKCTRL1_CLKLDEN | CLKCTRL1_CLKLDSYSEN);

	/* HSYNC Timing */
	dpu95_ld_write(ld, LD_REG_HSYNCTIMING, HSYNC_SRT_SET(1));

	/* VSYNC Timing */
	dpu95_ld_write(ld, LD_REG_VSYNCTIMING,
		       VSYNC_SRT_SET(mode->vtotal - 3) |
		       VSYNC_END_SET(mode->vtotal - 2));

	/* Static Control */
	dpu95_ld_write(ld, LD_REG_STATICCONTROL,
		       STATICCONTROL_DLYCMP_SET(0x17) |
		       STATICCONTROL_MODE_SET(0x3));

	dpu95_ld_ldim_init_default_regs(ld, mbi6353_regs, ARRAY_SIZE(mbi6353_regs));

	return ret;
}

static int dpu95_ld_core_enable(struct dpu95_localdimming *ld)
{
	struct dpu95_ld_intr intr;

	if (ld->enabled)
		return 0;

	/* Global Clock enable */
	dpu95_ld_write(ld, LD_REG_CLKCTRL0, CLKCTRL0_GLOBCLKEN);

	/* LD Clock enable */
	dpu95_ld_write(ld, LD_REG_CLKCTRL1, CLKCTRL1_CLKLDEN | CLKCTRL1_CLKLDSYSEN);

	/* Enable backlight specific interrupts */
	intr.ldvintdly = 1;
	intr.smslvon   = 0;
	intr.smvlvon   = 0;
	intr.smblvon   = 0;
	intr.ledinton  = 0;
	intr.blinton   = 1;
	intr.vinton    = 1;
	dpu95_ld_set_intr(ld, &intr);

	/* Clear interrupt status */
	dpu95_irq_clear(ld->dpu, DPU952_IRQ_LOCALDIMMING0_IRQ1);
	/* Enable interrupt */
	dpu95_irq_set(ld->dpu, DPU952_IRQ_LOCALDIMMING0_IRQ1, true);

	/* Enable Local Dimming */
	dpu95_ld_write(ld, LD_REG_LDUPREG000, 0x01);

	ld->enabled = true;

	return 0;
}

static int dpu95_ld_core_disable(struct dpu95_localdimming *ld)
{
	struct dpu95_ld_intr intr;

	if (!ld->enabled)
		return 0;

	/* Disable Local Dimming */
	dpu95_ld_write(ld, LD_REG_LDUPREG000, 0x0);

	/* Clear interrupt status */
	dpu95_irq_clear(ld->dpu, DPU952_IRQ_LOCALDIMMING0_IRQ1);
	/* Disable interrupt */
	dpu95_irq_set(ld->dpu, DPU952_IRQ_LOCALDIMMING0_IRQ1, false);

	/* Disable all interrupts */
	intr.ldvintdly = 0;
	intr.smslvon   = 0;
	intr.smvlvon   = 0;
	intr.smblvon   = 0;
	intr.ledinton  = 0;
	intr.blinton   = 0;
	intr.vinton    = 0;
	dpu95_ld_set_intr(ld, &intr);

	/* LD Clock disable */
	dpu95_ld_write(ld, LD_REG_CLKCTRL1, 0);

	/* Global Clock disable */
	dpu95_ld_write(ld, LD_REG_CLKCTRL0, 0);

	ld->enabled = false;

	return 0;
}

static void dpu95_ldfw_start_worker(struct work_struct *work)
{
	struct dpu95_ld_fw *fw = container_of(work, struct dpu95_ld_fw,
						start_work);
	struct dpu95_localdimming *ld = container_of(fw, struct dpu95_localdimming,
						     fw);
	struct dpu95_ldfw_cmd fw_cmd = {0, 0, 0, 0};

	/* First, configure the LD-FW */
	fw_cmd.cmd_id = DPU95_LD_FW_CFG;
	fw_cmd.cmd_data0 = ld->config.led_size;
	fw_cmd.cmd_data1 = ld->config.h_led;
	fw_cmd.cmd_data2 = ld->config.v_led;
	imx_dispmu_send_msg(&fw->mu, &fw_cmd, NULL);

	/* TODO: we need to sleep here so that the BL driver is fully powered on.
	 * This need to be reworked and sync-up with the BL driver power-on event.
	 */
	msleep(80);

	memset(&fw_cmd, 0, sizeof(fw_cmd));
	fw_cmd.cmd_id = DPU95_LD_FW_START;
	imx_dispmu_send_msg(&fw->mu, &fw_cmd, NULL);
}

static int dpu95_ld_fw_enable(struct dpu95_localdimming *ld)
{
	struct dpu95_ld_fw *fw = &ld->fw;

	/*
	 * Check if the remote processor is running and has already responded
	 * with a READY_AFTER_BOOT message.
	 */
	if (!fw->rproc || !fw->ready)
		return -ENODEV;

	/* If the FW is already enabled, nothing to do */
	if (fw->enabled)
		return 0;

	queue_work(ld->fw.workq, &ld->fw.start_work);

	return 0;
}

static int dpu95_ld_fw_disable(struct dpu95_localdimming *ld)
{
	struct dpu95_ld_fw *fw = &ld->fw;
	struct dpu95_ldfw_cmd fw_cmd = {0, 0, 0, 0};

	/*
	 * Check if the remote processor is running and has already responded
	 * with a READY_AFTER_BOOT message.
	 */
	if (!fw->rproc || !fw->ready)
		return -ENODEV;

	/* If the FW is already disabled, nothing to do */
	if (!fw->enabled)
		return 0;

	/* Disable the LD-FW from processing of LD interrupts */
	fw_cmd.cmd_id = DPU95_LD_FW_STOP;
	imx_dispmu_send_msg(&fw->mu, &fw_cmd, NULL);

	/* TODO: probably we should use a semaphore and wait until the confirmation
	 * message is received from the remote processor.
	 */

	return 0;
}

int dpu95_ld_enable(struct dpu95_localdimming *ld)
{
	int ret = 0;

	/*
	 * This could happen only if LD feature is not enabled, so just don't do
	 * anything and return 0.
	 */
	if (!ld)
		return 0;

	dpu95_ld_mode_set(ld, &ld->mode);

	ret |= dpu95_ld_core_enable(ld);
	ret |= dpu95_ld_fw_enable(ld);

	return ret;
}

int dpu95_ld_disable(struct dpu95_localdimming *ld)
{
	int ret = 0;

	/*
	 * This could happen only if LD feature is not enabled, so just don't do
	 * anything and return 0.
	 */
	if (!ld)
		return 0;

	ret |= dpu95_ld_fw_disable(ld);
	ret |= dpu95_ld_core_disable(ld);

	return ret;
}

int dpu95_ld_load(struct dpu95_drm_device *dpu_drm)
{
	struct dpu95_localdimming *ld = dpu_drm->dpu_soc.ld;
	struct device *dev = dpu_drm->dpu_soc.dev;
	struct task_struct *task;
	int ret;

	if (!ld)
		return 0;

	ld->fw.ready = false;
	ld->fw.mu.rx_callback = dpu95_ldfw_mu_callback;

	ret = imx_dispmu_client_register(&ld->fw.mu);
	if (ret) {
		dev_err(dev, "Failed registering to DISPMU: %d\n", ret);
		return ret;
	}

	if (ld->fw.rproc->state != RPROC_RUNNING) {
		task = kthread_run(dpu95_ldfw_rproc_boot_thread, ld,
				   "ldfw_rproc_loader");
		if (IS_ERR(task)) {
			dev_err(dev, "can't create LD-FW rproc_boot thread\n");
			ret = PTR_ERR(task);
			goto err_out;
		}
	}

	ld->fw.workq = create_singlethread_workqueue("ld-fw");
	if (!ld->fw.workq) {
		dev_err(dev, "failed to create workqueue\n");
		ret = -EINVAL;
		goto err_out;
	}
	INIT_WORK(&ld->fw.start_work, dpu95_ldfw_start_worker);

	/* TODO: we need maybe to load this from a dts property? */
	ld->config = mbi6353_cfg;

	return 0;

err_out:
	imx_dispmu_client_unregister(&ld->fw.mu);
	rproc_put(ld->fw.rproc);

	return ret;
}

void dpu95_ld_unload(struct dpu95_drm_device *dpu_drm)
{
	struct dpu95_localdimming *ld = dpu_drm->dpu_soc.ld;
	int ret;

	if (!ld)
		return;

	ret = imx_dispmu_client_unregister(&ld->fw.mu);
	if (ret)
		dev_err(ld->dpu->dev, "Failed unloading DISPMU: %d\n", ret);

	destroy_workqueue(ld->fw.workq);
}

struct dpu95_localdimming *dpu95_ld_get(struct dpu95_soc *dpu, unsigned int id)
{
	struct dpu95_localdimming *ld = dpu->ld;

	if (ld && ld->id != id)
		ld = NULL;

	return ld;
}

void dpu95_ld_hw_init(struct dpu95_soc *dpu, unsigned int index)
{
	/* Noting to do here */
}

int dpu95_ld_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long unused, unsigned long base)
{
	struct dpu95_localdimming *ld;
	struct device *dev = dpu->dev;
	phandle rproc_phandle;

	if (of_property_read_u32(dev->of_node, "nxp,ldfw-rproc",
				 &rproc_phandle)) {
		dev_dbg(dev, "Local Dimming feature is not enabled!\n");
		return 0;
	}

	ld = devm_kzalloc(dpu->dev, sizeof(*ld), GFP_KERNEL);
	if (!ld)
		return -ENOMEM;

	ld->base = devm_ioremap(dpu->dev, base, SZ_64K);
	if (!ld->base)
		return -ENOMEM;

	ld->fw.rproc = rproc_get_by_phandle(rproc_phandle);
	if (!ld->fw.rproc)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "could not get LD-FW rproc handle\n");

	dpu->ld = ld;
	ld->dpu = dpu;
	ld->id = id;
	ld->index = index;

	return 0;
}
