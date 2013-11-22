/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc.
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
#include <asm/cpuidle.h>
#include <asm/fncpy.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/tlb.h>

#include "common.h"
#include "cpuidle.h"

extern u32 audio_bus_freq_mode;
extern u32 ultra_low_bus_freq_mode;
extern unsigned long reg_addrs[];
extern void imx6sl_low_power_wfi(void);

static void __iomem *iomux_base;
static void *wfi_iram_base;

void (*imx6sl_wfi_in_iram_fn)(void *wfi_iram_base,
	void *iomux_addr, void *regs_addr, u32 audio_mode) = NULL;

#define WFI_IN_IRAM_SIZE	0x1000

static int imx6sl_enter_wait(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv, int index)
{
	imx6_set_lpm(WAIT_UNCLOCKED);
	if (ultra_low_bus_freq_mode || audio_bus_freq_mode) {
		/*
		 * Flush the TLB, to ensure no TLB maintenance occurs
		 * when DDR is in self-refresh.
		 */
		local_flush_tlb_all();
		/*
		 * Run WFI code from IRAM.
		 * Drop the DDR freq to 1MHz and AHB to 3MHz
		 * Also float DDR IO pads.
		 */
		imx6sl_wfi_in_iram_fn(wfi_iram_base, iomux_base, reg_addrs, audio_bus_freq_mode);
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
	struct platform_device *ocram_dev;
	unsigned int iram_paddr;
	struct device_node *node;
	struct gen_pool *iram_pool;

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-iomuxc");
	if (!node) {
		pr_err("failed to find imx6sl-iomuxc device tree data!\n");
		return -EINVAL;
	}
	iomux_base = of_iomap(node, 0);
	WARN(!iomux_base, "unable to map iomux registers\n");

	node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		pr_err("%s: failed to find ocram node\n",
			__func__);
		return -EINVAL;
	}

	ocram_dev = of_find_device_by_node(node);
	if (!ocram_dev) {
		pr_err("failed to find ocram device!\n");
		return -EINVAL;
	}

	iram_pool = dev_get_gen_pool(&ocram_dev->dev);
	if (!iram_pool) {
		pr_err("iram pool unavailable!\n");
		return -EINVAL;
	}
	/*
	 * Allocate IRAM memory when ARM executes WFI in
	 * ultra_low_power_mode.
	 */
	wfi_iram_base = (void *)gen_pool_alloc(iram_pool,
						WFI_IN_IRAM_SIZE);
	if (!wfi_iram_base) {
		pr_err("Cannot alloc iram for wfi code!\n");
		return -ENOMEM;
	}

	iram_paddr = gen_pool_virt_to_phys(iram_pool,
				(unsigned long)wfi_iram_base);
	/*
	 * Need to remap the area here since we want
	 * the memory region to be executable.
	 */
	wfi_iram_base = __arm_ioremap(iram_paddr,
					WFI_IN_IRAM_SIZE,
					MT_MEMORY_NONCACHED);
	if (!wfi_iram_base)
		pr_err("wfi_ram_base NOT remapped\n");

	imx6sl_wfi_in_iram_fn = (void *)fncpy(wfi_iram_base,
		&imx6sl_low_power_wfi, WFI_IN_IRAM_SIZE);

	return cpuidle_register(&imx6sl_cpuidle_driver, NULL);
}
