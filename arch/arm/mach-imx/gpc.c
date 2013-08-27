/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include "common.h"
#include "hardware.h"

#define GPC_IMR1		0x008
#define GPC_PGC_CPU_PDN		0x2a0
#define GPC_PGC_GPU_PDN		0x260
#define GPC_PGC_GPU_PUPSCR	0x264
#define GPC_PGC_GPU_PDNSCR	0x268
#define GPC_PGC_GPU_SW_SHIFT		0
#define GPC_PGC_GPU_SW_MASK		0x3f
#define GPC_PGC_GPU_SW2ISO_SHIFT	8
#define GPC_PGC_GPU_SW2ISO_MASK		0x3f
#define GPC_PGC_CPU_PUPSCR	0x2a4
#define GPC_PGC_CPU_PDNSCR	0x2a8
#define GPC_PGC_CPU_SW_SHIFT		0
#define GPC_PGC_CPU_SW_MASK		0x3f
#define GPC_PGC_CPU_SW2ISO_SHIFT	8
#define GPC_PGC_CPU_SW2ISO_MASK		0x3f
#define GPC_CNTR		0x0
#define GPC_CNTR_PU_UP_REQ_SHIFT	0x1
#define GPC_CNTR_PU_DOWN_REQ_SHIFT	0x0

#define IMR_NUM			4

static void __iomem *gpc_base;
static u32 gpc_wake_irqs[IMR_NUM];
static u32 gpc_saved_imrs[IMR_NUM];
static struct clk *gpu3d_clk, *gpu3d_shader_clk, *gpu2d_clk, *gpu2d_axi_clk;
static struct clk *openvg_axi_clk, *vpu_clk, *ipg_clk;
static struct device *gpc_dev;
struct regulator *pu_reg;
struct notifier_block nb;
static struct regulator_dev *pu_dummy_regulator_rdev;
static struct regulator_init_data pu_dummy_initdata = {
	.constraints = {
		.max_uV = 1450000,	/* allign with real max of anatop */
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
				REGULATOR_CHANGE_VOLTAGE,
	},
};
static int pu_dummy_enable;

void imx_gpc_pre_suspend(bool arm_power_off)
{
	void __iomem *reg_imr1 = gpc_base + GPC_IMR1;
	int i;

	if (arm_power_off)
		/* Tell GPC to power off ARM core when suspend */
		writel_relaxed(0x1, gpc_base + GPC_PGC_CPU_PDN);

	for (i = 0; i < IMR_NUM; i++) {
		gpc_saved_imrs[i] = readl_relaxed(reg_imr1 + i * 4);
		writel_relaxed(~gpc_wake_irqs[i], reg_imr1 + i * 4);
	}
}

void imx_gpc_post_resume(void)
{
	void __iomem *reg_imr1 = gpc_base + GPC_IMR1;
	int i;

	/* Keep ARM core powered on for other low-power modes */
	writel_relaxed(0x0, gpc_base + GPC_PGC_CPU_PDN);

	for (i = 0; i < IMR_NUM; i++)
		writel_relaxed(gpc_saved_imrs[i], reg_imr1 + i * 4);
}

static int imx_gpc_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned int idx = d->irq / 32 - 1;
	u32 mask;

	/* Sanity check for SPI irq */
	if (d->irq < 32)
		return -EINVAL;

	mask = 1 << d->irq % 32;
	gpc_wake_irqs[idx] = on ? gpc_wake_irqs[idx] | mask :
				  gpc_wake_irqs[idx] & ~mask;

	return 0;
}

void imx_gpc_mask_all(void)
{
	void __iomem *reg_imr1 = gpc_base + GPC_IMR1;
	int i;

	for (i = 0; i < IMR_NUM; i++) {
		gpc_saved_imrs[i] = readl_relaxed(reg_imr1 + i * 4);
		writel_relaxed(~0, reg_imr1 + i * 4);
	}

}

void imx_gpc_restore_all(void)
{
	void __iomem *reg_imr1 = gpc_base + GPC_IMR1;
	int i;

	for (i = 0; i < IMR_NUM; i++)
		writel_relaxed(gpc_saved_imrs[i], reg_imr1 + i * 4);
}

void imx_gpc_irq_unmask(struct irq_data *d)
{
	void __iomem *reg;
	u32 val;

	/* Sanity check for SPI irq */
	if (d->irq < 32)
		return;

	reg = gpc_base + GPC_IMR1 + (d->irq / 32 - 1) * 4;
	val = readl_relaxed(reg);
	val &= ~(1 << d->irq % 32);
	writel_relaxed(val, reg);
}

void imx_gpc_irq_mask(struct irq_data *d)
{
	void __iomem *reg;
	u32 val;

	/* Sanity check for SPI irq */
	if (d->irq < 32)
		return;

	reg = gpc_base + GPC_IMR1 + (d->irq / 32 - 1) * 4;
	val = readl_relaxed(reg);
	val |= 1 << (d->irq % 32);
	writel_relaxed(val, reg);
}

static void imx_pu_clk(bool enable)
{
	if (enable) {
		if (cpu_is_imx6sl()) {
			clk_prepare_enable(gpu2d_clk);
			clk_prepare_enable(openvg_axi_clk);
		} else {
			clk_prepare_enable(gpu3d_clk);
			clk_prepare_enable(gpu3d_shader_clk);
			clk_prepare_enable(vpu_clk);
			clk_prepare_enable(gpu2d_clk);
			clk_prepare_enable(gpu2d_axi_clk);
			clk_prepare_enable(openvg_axi_clk);
		}
	} else {
		if (cpu_is_imx6sl()) {
			clk_disable_unprepare(gpu2d_clk);
			clk_disable_unprepare(openvg_axi_clk);
		} else {
			clk_disable_unprepare(gpu3d_clk);
			clk_disable_unprepare(gpu3d_shader_clk);
			clk_disable_unprepare(vpu_clk);
			clk_disable_unprepare(gpu2d_clk);
			clk_disable_unprepare(gpu2d_axi_clk);
			clk_disable_unprepare(openvg_axi_clk);
		}
	}
}

static void imx_gpc_pu_enable(bool enable)
{
	u32 rate, delay_us;
	u32 gpu_pupscr_sw2iso, gpu_pdnscr_iso2sw;
	u32 gpu_pupscr_sw, gpu_pdnscr_iso;

	/* get ipg clk rate for PGC delay */
	rate = clk_get_rate(ipg_clk);

	if (enable) {
		imx_anatop_pu_enable(true);
		/*
		 * need to add necessary delay between powering up PU LDO and
		 * disabling PU isolation in PGC, the counter of PU isolation
		 * is based on ipg clk.
		 */
		gpu_pupscr_sw2iso = (readl_relaxed(gpc_base +
			GPC_PGC_GPU_PUPSCR) >> GPC_PGC_GPU_SW2ISO_SHIFT)
			& GPC_PGC_GPU_SW2ISO_MASK;
		gpu_pupscr_sw = (readl_relaxed(gpc_base +
			GPC_PGC_GPU_PUPSCR) >> GPC_PGC_GPU_SW_SHIFT)
			& GPC_PGC_GPU_SW_MASK;
		delay_us = (gpu_pupscr_sw2iso + gpu_pupscr_sw) * 1000000
			/ rate + 1;
		udelay(delay_us);

		imx_pu_clk(true);
		writel_relaxed(1, gpc_base + GPC_PGC_GPU_PDN);
		writel_relaxed(1 << GPC_CNTR_PU_UP_REQ_SHIFT,
			gpc_base + GPC_CNTR);
		while (readl_relaxed(gpc_base + GPC_CNTR) &
			(1 << GPC_CNTR_PU_UP_REQ_SHIFT))
			;
		imx_pu_clk(false);
	} else {
		writel_relaxed(1, gpc_base + GPC_PGC_GPU_PDN);
		writel_relaxed(1 << GPC_CNTR_PU_DOWN_REQ_SHIFT,
			gpc_base + GPC_CNTR);
		while (readl_relaxed(gpc_base + GPC_CNTR) &
			(1 << GPC_CNTR_PU_DOWN_REQ_SHIFT))
			;
		/*
		 * need to add necessary delay between enabling PU isolation
		 * in PGC and powering down PU LDO , the counter of PU isolation
		 * is based on ipg clk.
		 */
		gpu_pdnscr_iso2sw = (readl_relaxed(gpc_base +
			GPC_PGC_GPU_PDNSCR) >> GPC_PGC_GPU_SW2ISO_SHIFT)
			& GPC_PGC_GPU_SW2ISO_MASK;
		gpu_pdnscr_iso = (readl_relaxed(gpc_base +
			GPC_PGC_GPU_PDNSCR) >> GPC_PGC_GPU_SW_SHIFT)
			& GPC_PGC_GPU_SW_MASK;
		delay_us = (gpu_pdnscr_iso2sw + gpu_pdnscr_iso) * 1000000
			/ rate + 1;
		udelay(delay_us);
		imx_anatop_pu_enable(false);
	}
}

static int imx_gpc_regulator_notify(struct notifier_block *nb,
					unsigned long event,
					void *ignored)
{
	switch (event) {
	case REGULATOR_EVENT_PRE_DISABLE:
		imx_gpc_pu_enable(false);
		break;
	case REGULATOR_EVENT_ENABLE:
		imx_gpc_pu_enable(true);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

void __init imx_gpc_init(void)
{
	struct device_node *np;
	int i;
	u32 val;
	u32 cpu_pupscr_sw2iso, cpu_pupscr_sw;
	u32 cpu_pdnscr_iso2sw, cpu_pdnscr_iso;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpc");
	gpc_base = of_iomap(np, 0);
	WARN_ON(!gpc_base);

	/* Initially mask all interrupts */
	for (i = 0; i < IMR_NUM; i++)
		writel_relaxed(~0, gpc_base + GPC_IMR1 + i * 4);

	/* Register GPC as the secondary interrupt controller behind GIC */
	gic_arch_extn.irq_mask = imx_gpc_irq_mask;
	gic_arch_extn.irq_unmask = imx_gpc_irq_unmask;
	gic_arch_extn.irq_set_wake = imx_gpc_irq_set_wake;

	/*
	 * If there are CPU isolation timing settings in dts,
	 * update them according to dts, otherwise, keep them
	 * with default value in registers.
	 */
	cpu_pupscr_sw2iso = cpu_pupscr_sw =
		cpu_pdnscr_iso2sw = cpu_pdnscr_iso = 0;

	/* Read CPU isolation setting for GPC */
	of_property_read_u32(np, "fsl,cpu_pupscr_sw2iso", &cpu_pupscr_sw2iso);
	of_property_read_u32(np, "fsl,cpu_pupscr_sw", &cpu_pupscr_sw);
	of_property_read_u32(np, "fsl,cpu_pdnscr_iso2sw", &cpu_pdnscr_iso2sw);
	of_property_read_u32(np, "fsl,cpu_pdnscr_iso", &cpu_pdnscr_iso);

	/* Update CPU PUPSCR timing if it is defined in dts */
	val = readl_relaxed(gpc_base + GPC_PGC_CPU_PUPSCR);
	if (cpu_pupscr_sw2iso)
		val &= ~(GPC_PGC_CPU_SW2ISO_MASK << GPC_PGC_CPU_SW2ISO_SHIFT);
	if (cpu_pupscr_sw)
		val &= ~(GPC_PGC_CPU_SW_MASK << GPC_PGC_CPU_SW_SHIFT);
	val |= cpu_pupscr_sw2iso << GPC_PGC_CPU_SW2ISO_SHIFT;
	val |= cpu_pupscr_sw << GPC_PGC_CPU_SW_SHIFT;
	writel_relaxed(val, gpc_base + GPC_PGC_CPU_PUPSCR);

	/* Update CPU PDNSCR timing if it is defined in dts */
	val = readl_relaxed(gpc_base + GPC_PGC_CPU_PDNSCR);
	if (cpu_pdnscr_iso2sw)
		val &= ~(GPC_PGC_CPU_SW2ISO_MASK << GPC_PGC_CPU_SW2ISO_SHIFT);
	if (cpu_pdnscr_iso)
		val &= ~(GPC_PGC_CPU_SW_MASK << GPC_PGC_CPU_SW_SHIFT);
	val |= cpu_pdnscr_iso2sw << GPC_PGC_CPU_SW2ISO_SHIFT;
	val |= cpu_pdnscr_iso << GPC_PGC_CPU_SW_SHIFT;
	writel_relaxed(val, gpc_base + GPC_PGC_CPU_PDNSCR);
}

static int imx_pureg_set_voltage(struct regulator_dev *reg, int min_uV,
					int max_uV, unsigned *selector)
{
	return 0;
}

static int imx_pureg_enable(struct regulator_dev *rdev)
{
	pu_dummy_enable = 1;

	return 0;
}

static int imx_pureg_disable(struct regulator_dev *rdev)
{
	pu_dummy_enable = 0;

	return 0;
}

static int imx_pureg_is_enable(struct regulator_dev *rdev)
{
	return pu_dummy_enable;
}

static int imx_pureg_list_voltage(struct regulator_dev *rdev,
				unsigned int selector)
{
	return 0;
}

static struct regulator_ops pu_dummy_ops = {
	.set_voltage = imx_pureg_set_voltage,
	.enable	= imx_pureg_enable,
	.disable = imx_pureg_disable,
	.is_enabled = imx_pureg_is_enable,
	.list_voltage = imx_pureg_list_voltage,
};

static struct regulator_desc pu_dummy_desc = {
	.name = "pureg-dummy",
	.id = -1,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &pu_dummy_ops,
};

static int pu_dummy_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	int ret;

	config.dev = &pdev->dev;
	config.init_data = &pu_dummy_initdata;
	config.of_node = pdev->dev.of_node;

	pu_dummy_regulator_rdev = regulator_register(&pu_dummy_desc, &config);
	if (IS_ERR(pu_dummy_regulator_rdev)) {
		ret = PTR_ERR(pu_dummy_regulator_rdev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id imx_pudummy_ids[] = {
	{ .compatible = "fsl,imx6-dummy-pureg" },
};
MODULE_DEVICE_TABLE(of, imx_pudummy_ids);

static struct platform_driver pu_dummy_driver = {
	.probe	= pu_dummy_probe,
	.driver	= {
		.name	= "pu-dummy",
		.owner	= THIS_MODULE,
		.of_match_table = imx_pudummy_ids,
	},
};

static int imx_gpc_probe(struct platform_device *pdev)
{
	int ret;

	gpc_dev = &pdev->dev;

	pu_reg = devm_regulator_get(gpc_dev, "pu");
	if (IS_ERR(pu_reg)) {
		ret = PTR_ERR(pu_reg);
		dev_info(gpc_dev, "pu regulator not ready.\n");
		return ret;
	}
	nb.notifier_call = &imx_gpc_regulator_notify;

	/* Get gpu&vpu clk for power up PU by GPC */
	if (cpu_is_imx6sl()) {
		gpu2d_clk = devm_clk_get(gpc_dev, "gpu2d_podf");
		openvg_axi_clk = devm_clk_get(gpc_dev, "gpu2d_ovg");
		ipg_clk = devm_clk_get(gpc_dev, "ipg");
		if (IS_ERR(gpu2d_clk) || IS_ERR(openvg_axi_clk)
			|| IS_ERR(ipg_clk)) {
			dev_err(gpc_dev, "failed to get clk!\n");
			return -ENOENT;
		}
	} else {
		gpu3d_clk = devm_clk_get(gpc_dev, "gpu3d_core");
		gpu3d_shader_clk = devm_clk_get(gpc_dev, "gpu3d_shader");
		gpu2d_clk = devm_clk_get(gpc_dev, "gpu2d_core");
		gpu2d_axi_clk = devm_clk_get(gpc_dev, "gpu2d_axi");
		openvg_axi_clk = devm_clk_get(gpc_dev, "openvg_axi");
		vpu_clk = devm_clk_get(gpc_dev, "vpu_axi");
		ipg_clk = devm_clk_get(gpc_dev, "ipg");
		if (IS_ERR(gpu3d_clk) || IS_ERR(gpu3d_shader_clk)
			|| IS_ERR(gpu2d_clk) || IS_ERR(gpu2d_axi_clk)
			|| IS_ERR(openvg_axi_clk) || IS_ERR(vpu_clk)
			|| IS_ERR(ipg_clk)) {
			dev_err(gpc_dev, "failed to get clk!\n");
			return -ENOENT;
		}
	}

	ret = regulator_register_notifier(pu_reg, &nb);
	if (ret) {
		dev_err(gpc_dev,
			"regulator notifier request failed\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id imx_gpc_ids[] = {
	{ .compatible = "fsl,imx6q-gpc" },
};
MODULE_DEVICE_TABLE(of, imx_gpc_ids);

static struct platform_driver imx_gpc_platdrv = {
	.driver = {
		.name	= "imx-gpc",
		.owner	= THIS_MODULE,
		.of_match_table = imx_gpc_ids,
	},
	.probe		= imx_gpc_probe,
};
module_platform_driver(imx_gpc_platdrv);

module_platform_driver(pu_dummy_driver);

MODULE_AUTHOR("Anson Huang <b20788@freescale.com>");
MODULE_DESCRIPTION("Freescale i.MX GPC driver");
MODULE_LICENSE("GPL");
