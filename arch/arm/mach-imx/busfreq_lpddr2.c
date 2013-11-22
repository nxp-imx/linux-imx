/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file busfreq_lpddr2.c
 *
 * @brief iMX6 LPDDR2 frequency change specific file.
 *
 * @ingroup PM
 */
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/tlb.h>
#include <linux/clk.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/smp.h>

#include "hardware.h"

/* DDR settings */
static void __iomem *mmdc_base;
static void __iomem *anatop_base;
static void __iomem *ccm_base;
static void __iomem *l2_base;
static struct device *busfreq_dev;
static void *ddr_freq_change_iram_base;
static int curr_ddr_rate;

unsigned long reg_addrs[4];

void (*mx6_change_lpddr2_freq)(u32 ddr_freq, int bus_freq_mode,
	void *iram_addr) = NULL;

extern unsigned int ddr_normal_rate;
extern int low_bus_freq_mode;
extern int ultra_low_bus_freq_mode;
extern void mx6_lpddr2_freq_change(u32 freq, int bus_freq_mode,
	void *iram_addr);


#define LPDDR2_FREQ_CHANGE_SIZE	0x1000


/* change the DDR frequency. */
int update_lpddr2_freq(int ddr_rate)
{
	if (ddr_rate == curr_ddr_rate)
		return 0;

	dev_dbg(busfreq_dev, "\nBus freq set to %d start...\n", ddr_rate);

	/*
	 * Flush the TLB, to ensure no TLB maintenance occurs
	 * when DDR is in self-refresh.
	 */
	local_flush_tlb_all();
	/* Now change DDR frequency. */
	mx6_change_lpddr2_freq(ddr_rate,
		(low_bus_freq_mode | ultra_low_bus_freq_mode),
		reg_addrs);

	curr_ddr_rate = ddr_rate;

	dev_dbg(busfreq_dev, "\nBus freq set to %d done...\n", ddr_rate);

	return 0;
}

int init_mmdc_lpddr2_settings(struct platform_device *busfreq_pdev)
{
	struct platform_device *ocram_dev;
	unsigned int iram_paddr;
	struct device_node *node;
	struct gen_pool *iram_pool;

	busfreq_dev = &busfreq_pdev->dev;
	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-mmdc");
	if (!node) {
		printk(KERN_ERR "failed to find imx6sl-mmdc device tree data!\n");
		return -EINVAL;
	}
	mmdc_base = of_iomap(node, 0);
	WARN(!mmdc_base, "unable to map mmdc registers\n");

	node = NULL;
	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-ccm");
	if (!node) {
		printk(KERN_ERR "failed to find imx6sl-ccm device tree data!\n");
		return -EINVAL;
	}
	ccm_base = of_iomap(node, 0);
	WARN(!ccm_base, "unable to map ccm registers\n");

	node = of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
	if (!node) {
		printk(KERN_ERR "failed to find imx6sl-pl310-cache device tree data!\n");
		return -EINVAL;
	}
	l2_base = of_iomap(node, 0);
	WARN(!l2_base, "unable to map PL310 registers\n");

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6sl-anatop");
	if (!node) {
		printk(KERN_ERR "failed to find imx6sl-pl310-cache device tree data!\n");
		return -EINVAL;
	}
	anatop_base = of_iomap(node, 0);
	WARN(!anatop_base, "unable to map anatop registers\n");

	node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mmio-sram");
	if (!node) {
		dev_err(busfreq_dev, "%s: failed to find ocram node\n",
			__func__);
		return -EINVAL;
	}

	ocram_dev = of_find_device_by_node(node);
	if (!ocram_dev) {
		dev_err(busfreq_dev, "failed to find ocram device!\n");
		return -EINVAL;
	}

	iram_pool = dev_get_gen_pool(&ocram_dev->dev);
	if (!iram_pool) {
		dev_err(busfreq_dev, "iram pool unavailable!\n");
		return -EINVAL;
	}

	reg_addrs[0] = (unsigned long)anatop_base;
	reg_addrs[1] = (unsigned long)ccm_base;
	reg_addrs[2] = (unsigned long)mmdc_base;
	reg_addrs[3] = (unsigned long)l2_base;

	ddr_freq_change_iram_base = (void *)gen_pool_alloc(iram_pool,
						LPDDR2_FREQ_CHANGE_SIZE);
	if (!ddr_freq_change_iram_base) {
		dev_err(busfreq_dev,
			"Cannot alloc iram for ddr freq change code!\n");
		return -ENOMEM;
	}

	iram_paddr = gen_pool_virt_to_phys(iram_pool,
				(unsigned long)ddr_freq_change_iram_base);
	/*
	 * Need to remap the area here since we want
	 * the memory region to be executable.
	 */
	ddr_freq_change_iram_base = __arm_ioremap(iram_paddr,
						LPDDR2_FREQ_CHANGE_SIZE,
						MT_MEMORY_NONCACHED);
	mx6_change_lpddr2_freq = (void *)fncpy(ddr_freq_change_iram_base,
		&mx6_lpddr2_freq_change, LPDDR2_FREQ_CHANGE_SIZE);

	curr_ddr_rate = ddr_normal_rate;

	return 0;
}
