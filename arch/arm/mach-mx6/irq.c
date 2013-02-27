/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <asm/hardware/gic.h>
#include <mach/hardware.h>
#ifdef CONFIG_CPU_FREQ_GOV_INTERACTIVE
#include <linux/cpufreq.h>
#endif
#ifdef CONFIG_PCI_MSI
#include "msi.h"
#endif

int mx6q_register_gpios(void);
unsigned int gpc_wake_irq[4];
extern bool enable_wait_mode;
#ifdef CONFIG_CPU_FREQ_GOV_INTERACTIVE
extern int cpufreq_gov_irq_tuner_register(struct irq_tuner dbs_irq_tuner);
#endif

static int mx6_gic_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	if ((d->irq < MXC_INT_START) || (d->irq > MXC_INT_END)) {
		printk(KERN_ERR "Invalid irq number!\n");
		return -EINVAL;
	}

	if (enable) {
		gpc_wake_irq[d->irq / 32 - 1] |= 1 << (d->irq % 32);
		printk(KERN_INFO "add wake up source irq %d\n", d->irq);
	} else {
		printk(KERN_INFO "remove wake up source irq %d\n", d->irq);
		gpc_wake_irq[d->irq / 32 - 1] &= ~(1 << (d->irq % 32));
	}
	return 0;
}
#ifdef CONFIG_CPU_FREQ_GOV_INTERACTIVE
static struct irq_tuner mxc_irq_tuner[] = {
	{
	 .irq_number = 41, /* GPU 3D */
	 .up_threshold = 0,
	 .enable = 0,},
	{
	 .irq_number = 42, /* GPU 2D */
	 .up_threshold = 40,
	 .enable = 0,},
	{
	 .irq_number = 43, /* GPU VG */
	 .up_threshold = 0,
	 .enable = 0,},
	{
	 .irq_number = 54, /* uSDHC1 */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 55, /* uSDHC2 */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 56, /* uSDHC3 */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 57, /* uSDHC4 */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 71, /* SATA */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 75, /* OTG */
	 .up_threshold = 10,
	 .enable = 1,},
	{
	 .irq_number = 150, /* ENET */
	 .up_threshold = 4,
	 .enable = 1,},
	{
	 .irq_number = 0, /* END */
	 .up_threshold = 0,
	 .enable = 0,},
};
#endif
void mx6_init_irq(void)
{
	void __iomem *gpc_base = IO_ADDRESS(GPC_BASE_ADDR);
	struct irq_desc *desc;
	unsigned int i;

	/* start offset if private timer irq id, which is 29.
	 * ID table:
	 * Global timer, PPI -> ID27
	 * A legacy nFIQ, PPI -> ID28
	 * Private timer, PPI -> ID29
	 * Watchdog timers, PPI -> ID30
	 * A legacy nIRQ, PPI -> ID31
	 */
	gic_init(0, 29, IO_ADDRESS(IC_DISTRIBUTOR_BASE_ADDR),
		IO_ADDRESS(IC_INTERFACES_BASE_ADDR));

	if (enable_wait_mode) {
		/* Mask the always pending interrupts - HW bug. */
		__raw_writel(0x00400000, gpc_base + 0x0c);
		__raw_writel(0x20000000, gpc_base + 0x10);
	}


	for (i = MXC_INT_START; i <= MXC_INT_END; i++) {
		desc = irq_to_desc(i);
		desc->irq_data.chip->irq_set_wake = mx6_gic_irq_set_wake;
	}
	mx6q_register_gpios();
#ifdef CONFIG_CPU_FREQ_GOV_INTERACTIVE
	for (i = 0; i < ARRAY_SIZE(mxc_irq_tuner); i++)
		cpufreq_gov_irq_tuner_register(mxc_irq_tuner[i]);
#endif
#ifdef CONFIG_PCI_MSI
	imx_msi_init();
#endif
}
