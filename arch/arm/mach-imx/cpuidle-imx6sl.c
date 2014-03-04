/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/genalloc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <asm/cpuidle.h>
#include <asm/fncpy.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/tlb.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

extern u32 audio_bus_freq_mode;
extern u32 ultra_low_bus_freq_mode;
extern unsigned long reg_addrs[];
extern void imx6sl_low_power_wfi(void);

extern unsigned long save_ttbr1(void);
extern void restore_ttbr1(unsigned long ttbr1);

static void __iomem *iomux_base;
static void *wfi_iram_base;
static struct regulator *vbus_ldo;

static struct regulator_dev *ldo2p5_dummy_regulator_rdev;
static struct regulator_init_data ldo2p5_dummy_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};
static int ldo2p5_dummy_enable;

void (*imx6sl_wfi_in_iram_fn)(void *wfi_iram_base,
	bool vbus_ldo, u32 audio_mode) = NULL;


static int imx6sl_enter_wait(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv, int index)
{
	imx6_set_lpm(WAIT_UNCLOCKED);
	if (ultra_low_bus_freq_mode || audio_bus_freq_mode) {
		unsigned long ttbr1;
		/*
		 * Run WFI code from IRAM.
		 * Drop the DDR freq to 1MHz and AHB to 3MHz
		 * Also float DDR IO pads.
		 */
		ttbr1 = save_ttbr1();
		imx6sl_wfi_in_iram_fn(wfi_iram_base, regulator_is_enabled(vbus_ldo),
					audio_bus_freq_mode);
		restore_ttbr1(ttbr1);
	} else {
		imx6sl_set_wait_clk(true);
		cpu_do_idle();
		imx6sl_set_wait_clk(false);
	}
	imx6_set_lpm(WAIT_CLOCKED);

	return index;
}

static struct cpuidle_driver imx6sl_cpuidle_driver = {
	.name = "imx6sl_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		/* WFI */
		ARM_CPUIDLE_WFI_STATE,
		/* WAIT */
		{
			.exit_latency = 50,
			.target_residency = 75,
			.flags = CPUIDLE_FLAG_TIME_VALID |
					CPUIDLE_FLAG_TIMER_STOP,
			.enter = imx6sl_enter_wait,
			.name = "WAIT",
			.desc = "Clock off",
		},
	},
	.state_count = 2,
	.safe_state_index = 0,
};

int __init imx6sl_cpuidle_init(void)
{
	unsigned int iram_paddr;
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-iomuxc");
	if (!node) {
		pr_err("failed to find imx6sl-iomuxc device tree data!\n");
		return -EINVAL;
	}
	iomux_base = of_iomap(node, 0);
	WARN(!iomux_base, "unable to map iomux registers\n");

	vbus_ldo = regulator_get(NULL, "ldo2p5-dummy");
	if (IS_ERR(vbus_ldo))
		vbus_ldo = NULL;

	iram_paddr = MX6SL_WFI_IRAM_ADDR;
	/* Calculate the virtual address of the code */
	wfi_iram_base = (void *)IMX_IO_P2V(MX6Q_IRAM_TLB_BASE_ADDR) +
				(iram_paddr - MX6Q_IRAM_TLB_BASE_ADDR);

	imx6sl_wfi_in_iram_fn = (void *)fncpy(wfi_iram_base,
		&imx6sl_low_power_wfi, MX6SL_WFI_IRAM_CODE_SIZE);

	return cpuidle_register(&imx6sl_cpuidle_driver, NULL);
}

static int imx_ldo2p5_dummy_enable(struct regulator_dev *rdev)
{
	ldo2p5_dummy_enable = 1;

	return 0;
}

static int imx_ldo2p5_dummy_disable(struct regulator_dev *rdev)
{
	ldo2p5_dummy_enable = 0;

	return 0;
}

static int imx_ldo2p5_dummy_is_enable(struct regulator_dev *rdev)
{
	return ldo2p5_dummy_enable;
}


static struct regulator_ops ldo2p5_dummy_ops = {
	.enable	= imx_ldo2p5_dummy_enable,
	.disable = imx_ldo2p5_dummy_disable,
	.is_enabled = imx_ldo2p5_dummy_is_enable,
};

static struct regulator_desc ldo2p5_dummy_desc = {
	.name = "ldo2p5-dummy",
	.id = -1,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &ldo2p5_dummy_ops,
};

static int ldo2p5_dummy_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	int ret;

	config.dev = &pdev->dev;
	config.init_data = &ldo2p5_dummy_initdata;
	config.of_node = pdev->dev.of_node;

	ldo2p5_dummy_regulator_rdev = regulator_register(&ldo2p5_dummy_desc, &config);
	if (IS_ERR(ldo2p5_dummy_regulator_rdev)) {
		ret = PTR_ERR(ldo2p5_dummy_regulator_rdev);
		dev_err(&pdev->dev, "Failed to register dummy ldo2p5 regulator: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id imx_ldo2p5_dummy_ids[] = {
	{ .compatible = "fsl,imx6-dummy-ldo2p5" },
};
MODULE_DEVICE_TABLE(of, imx_ldo2p5_dummy_ids);

static struct platform_driver ldo2p5_dummy_driver = {
	.probe	= ldo2p5_dummy_probe,
	.driver	= {
		.name	= "ldo2p5-dummy",
		.owner	= THIS_MODULE,
		.of_match_table = imx_ldo2p5_dummy_ids,
	},
};

module_platform_driver(ldo2p5_dummy_driver);

