/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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


static struct device *busfreq_dev;
static void *ddr_freq_change_iram_base;
static int curr_ddr_rate;

void (*mx6_change_lpddr2_freq)(u32 ddr_freq, int bus_freq_mode) = NULL;

extern unsigned int ddr_normal_rate;
extern int low_bus_freq_mode;
extern int ultra_low_bus_freq_mode;
extern void mx6_lpddr2_freq_change(u32 freq, int bus_freq_mode);

extern unsigned long save_ttbr1(void);
extern void restore_ttbr1(unsigned long ttbr1);
extern unsigned long iram_tlb_phys_addr;

/* change the DDR frequency. */
int update_lpddr2_freq(int ddr_rate)
{
	unsigned long ttbr1;
	if (ddr_rate == curr_ddr_rate)
		return 0;

	dev_dbg(busfreq_dev, "\nBus freq set to %d start...\n", ddr_rate);

	/*
	 * Flush the TLB, to ensure no TLB maintenance occurs
	 * when DDR is in self-refresh.
	 */
	ttbr1 = save_ttbr1();

	/* Now change DDR frequency. */
	mx6_change_lpddr2_freq(ddr_rate,
		(low_bus_freq_mode | ultra_low_bus_freq_mode));
	restore_ttbr1(ttbr1);

	curr_ddr_rate = ddr_rate;

	dev_dbg(busfreq_dev, "\nBus freq set to %d done...\n", ddr_rate);

	return 0;
}

int init_mmdc_lpddr2_settings(struct platform_device *busfreq_pdev)
{
	busfreq_dev = &busfreq_pdev->dev;

	/* Calculate the virtual address of the code */
	ddr_freq_change_iram_base =
			(void *)IMX_IO_P2V(iram_tlb_phys_addr) +
			MX6SL_LPDDR2_FREQ_ADDR_OFFSET;

	mx6_change_lpddr2_freq = (void *)fncpy(ddr_freq_change_iram_base,
		&mx6_lpddr2_freq_change, LPDDR2_FREQ_CODE_SIZE);

	curr_ddr_rate = ddr_normal_rate;

	return 0;
}
