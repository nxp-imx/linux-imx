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
extern unsigned long iram_tlb_phys_addr;

static void __iomem *iomux_base;
static void *wfi_iram_base;

void (*imx6sl_wfi_in_iram_fn)(void *wfi_iram_base,
	void *iomux_addr, u32 audio_mode) = NULL;


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
		imx6sl_wfi_in_iram_fn(wfi_iram_base, iomux_base,
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
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-iomuxc");
	if (!node) {
		pr_err("failed to find imx6sl-iomuxc device tree data!\n");
		return -EINVAL;
	}
	iomux_base = of_iomap(node, 0);
	WARN(!iomux_base, "unable to map iomux registers\n");

	/* Calculate the virtual address of the code */
	wfi_iram_base = (void *)IMX_IO_P2V(iram_tlb_phys_addr) +
				MX6SL_WFI_IRAM_ADDR_OFFSET;

	imx6sl_wfi_in_iram_fn = (void *)fncpy(wfi_iram_base,
		&imx6sl_low_power_wfi, MX6SL_WFI_IRAM_CODE_SIZE);

	return cpuidle_register(&imx6sl_cpuidle_driver, NULL);
}
