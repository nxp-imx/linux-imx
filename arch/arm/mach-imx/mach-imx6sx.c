/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/can/platform/flexcan.h>
#include <linux/gpio.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/pm_opp.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"

static struct fec_platform_data fec_pdata[2];
static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_en_active_high;
static int flexcan_stby_gpio;
static int flexcan_stby_active_high;
static int flexcan0_en;
static int flexcan1_en;

static void imx6sx_fec1_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sx-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR4,
					   IMX6SX_GPR4_FEC_ENET1_STOP_REQ,
					   IMX6SX_GPR4_FEC_ENET1_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR4,
					   IMX6SX_GPR4_FEC_ENET1_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6sx-iomux-gpr regmap\n");
}

static void imx6sx_fec2_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sx-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR4,
					   IMX6SX_GPR4_FEC_ENET2_STOP_REQ,
					   IMX6SX_GPR4_FEC_ENET2_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR4,
					   IMX6SX_GPR4_FEC_ENET2_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6sx-iomux-gpr regmap\n");
}

static void __init imx6sx_enet_plt_init(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata[0].sleep_mode_enable = imx6sx_fec1_sleep_enable;
	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@021b4000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata[1].sleep_mode_enable = imx6sx_fec2_sleep_enable;
}

static void mx6sx_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		gpio_set_value_cansleep(flexcan_en_gpio,
					!flexcan_en_active_high);
		gpio_set_value_cansleep(flexcan_stby_gpio,
					!flexcan_stby_active_high);
		gpio_set_value_cansleep(flexcan_en_gpio,
					flexcan_en_active_high);
		gpio_set_value_cansleep(flexcan_stby_gpio,
					flexcan_stby_active_high);
	} else {
		/*
		* avoid to disable CAN xcvr if any of the CAN interfaces
		* are down. XCRV will be disabled only if both CAN2
		* interfaces are DOWN.
		*/
		gpio_set_value_cansleep(flexcan_en_gpio,
					!flexcan_en_active_high);
		gpio_set_value_cansleep(flexcan_stby_gpio,
					!flexcan_stby_active_high);
	}
}

static void imx6sx_arm2_flexcan0_switch(int enable)
{
	flexcan0_en = enable;
	mx6sx_flexcan_switch();
}

static void imx6sx_arm2_flexcan1_switch(int enable)
{
	flexcan1_en = enable;
	mx6sx_flexcan_switch();
}

static int __init imx6sx_arm2_flexcan_fixup(void)
{
	struct device_node *np;
	enum of_gpio_flags en_flags, stby_flags;
	bool canfd_en = false;
	int wakeup_gpio;

	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
		return -ENODEV;


	/* Wakeup transceiver first in case it's in sleep mode by default */
	wakeup_gpio = of_get_named_gpio(np, "trx-wakeup-gpio", 0);
	if (gpio_is_valid(wakeup_gpio) &&
		!gpio_request_one(wakeup_gpio, GPIOF_OUT_INIT_HIGH, "flexcan-trx-wakeup")) {
		gpio_set_value_cansleep(wakeup_gpio, 0);
		gpio_set_value_cansleep(wakeup_gpio, 1);
	}

	flexcan_en_gpio = of_get_named_gpio_flags(np, "trx-en-gpio", 0, &en_flags);
	flexcan_stby_gpio = of_get_named_gpio_flags(np, "trx-stby-gpio", 0, &stby_flags);

	if (gpio_is_valid(flexcan_en_gpio) && gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en") &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby")) {
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		flexcan_pdata[0].transceiver_switch = imx6sx_arm2_flexcan0_switch;
		flexcan_pdata[1].transceiver_switch = imx6sx_arm2_flexcan1_switch;
		if (!(en_flags & OF_GPIO_ACTIVE_LOW))
			flexcan_en_active_high = 1;

		if (!(stby_flags & OF_GPIO_ACTIVE_LOW))
			flexcan_stby_active_high = 1;
	}

	/*
	 * Switch on the transceiver by default for board with canfd enabled
	 * since canfd driver does not handle it.
	 * Two CAN instances share the same switch.
	 */
	for_each_node_by_name(np, "canfd") {
		if (of_device_is_available(np)) {
			canfd_en = true;
			break;
		}
	}

	if (of_machine_is_compatible("fsl,imx6sx-sdb") && canfd_en)
		imx6sx_arm2_flexcan0_switch(1);

	return 0;
}

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);
	return 0;
}

#define PHY_ID_AR8031   0x004dd074
static void __init imx6sx_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
					   ar8031_phy_fixup);
}

static void __init imx6sx_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6sx-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_FEC_CLOCK_MUX_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1,
				   IMX6SX_GPR1_FEC_CLOCK_PAD_DIR_MASK, 0);
	} else {
		pr_err("failed to find fsl,imx6sx-iomux-gpr regmap\n");
	}
}

static inline void imx6sx_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6sx-fec");
	imx6sx_enet_phy_init();
	imx6sx_enet_clk_sel();
	imx6sx_enet_plt_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6sx_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	OF_DEV_AUXDATA("fsl,imx6sx-fec", 0x02188000, NULL, &fec_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6sx-fec", 0x021b4000, NULL, &fec_pdata[1]),
	{ /* sentinel */ }
};

static inline void imx6sx_qos_init(void)
{
	struct device_node *np;
	void   __iomem *src_base;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-gpu");
	if (!np || !of_device_is_available(np))
		return;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-qosc");
	if (!np)
		return;
	src_base = of_iomap(np, 0);
	writel_relaxed(0, src_base); /* Disable clkgate & soft_rst */
	writel_relaxed(0, src_base+0x60); /* Enable all masters */
	writel_relaxed(0, src_base+0x1400); /* Disable clkgate & soft_rst for gpu */
	writel_relaxed(0x0f000222, src_base+0x1400+0xd0); /* Set Write QoS 2 for gpu */
	writel_relaxed(0x0f000822, src_base+0x1400+0xe0); /* Set Read QoS 8 for gpu */
	return;
}

static void __init imx6sx_init_machine(void)
{
	struct device *parent;

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6sx_auxdata_lookup, parent);

	imx6sx_enet_init();
	imx_anatop_init();
	imx6sx_pm_init();
	imx6sx_qos_init();
}

static void __init imx6sx_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static const char *imx6sx_dt_compat[] __initdata = {
	"fsl,imx6sx",
	NULL,
};

static void __init imx6sx_opp_init(struct device *cpu_dev)
{
	struct device_node *np;

	np = of_find_node_by_path("/cpus/cpu@0");
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	cpu_dev->of_node = np;
	if (of_init_opp_table(cpu_dev))
		pr_warn("failed to init OPP table\n");

	of_node_put(np);
}

static struct platform_device imx6sx_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};

static void __init imx6sx_init_late(void)
{
	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6sx_opp_init(&imx6sx_cpufreq_pdev.dev);
		platform_device_register(&imx6sx_cpufreq_pdev);
	}

	if (of_machine_is_compatible("fsl,imx6sx-sdb") ||
		of_machine_is_compatible("fsl,imx6sx-sabreauto"))
		imx6sx_arm2_flexcan_fixup();

	imx6sx_cpuidle_init();
}

static void __init imx6sx_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
	imx6_busfreq_map_io();
}

DT_MACHINE_START(IMX6SX, "Freescale i.MX6 SoloX (Device Tree)")
	.map_io		= imx6sx_map_io,
	.init_irq	= imx6sx_init_irq,
	.init_machine	= imx6sx_init_machine,
	.init_late	= imx6sx_init_late,
	.dt_compat	= imx6sx_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
