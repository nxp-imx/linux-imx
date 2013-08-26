/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk-provider.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/opp.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static struct platform_device imx6sl_cpufreq_pdev = {
	.name = "imx6-cpufreq",
};

static void __init imx6sl_fec_init(void)
{
	struct regmap *gpr;

	/* set FEC clock from internal PLL clock source */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sl-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX2_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6SL_GPR1_FEC_CLOCK_MUX1_SEL_MASK, 0);
	} else
		pr_err("failed to find fsl,imx6sl-iomux-gpr regmap\n");

}

static void __init imx6sl_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	imx6sl_fec_init();
	imx_anatop_init();
	imx6_pm_init();
}

static void __init imx6sl_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

put_node:
	of_node_put(np);
}

static void __init imx6sl_init_late(void)
{
	struct regmap *gpr;

	/*
	 * Need to force IOMUXC irq pending to meet CCM low power mode
	 * restriction, this is recommended by hardware team.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sl-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
			IMX6Q_GPR1_GINT_MASK,
			IMX6Q_GPR1_GINT_ASSERT);
	else
		pr_err("failed to find fsl,imx6sl-iomux-gpr regmap\n");

	/* Init CPUIDLE */
	imx6sl_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6_CPUFREQ)) {
		imx6sl_opp_init(&imx6sl_cpufreq_pdev.dev);
		platform_device_register(&imx6sl_cpufreq_pdev);
	}

}

static void __init imx6sl_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
}

static void __init imx6sl_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static void __init imx6sl_timer_init(void)
{
	of_clk_init(NULL);
}

static const char *imx6sl_dt_compat[] __initdata = {
	"fsl,imx6sl",
	NULL,
};

DT_MACHINE_START(IMX6SL, "Freescale i.MX6 SoloLite (Device Tree)")
	.map_io		= imx6sl_map_io,
	.init_irq	= imx6sl_init_irq,
	.init_time	= imx6sl_timer_init,
	.init_machine	= imx6sl_init_machine,
	.init_late      = imx6sl_init_late,
	.dt_compat	= imx6sl_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
