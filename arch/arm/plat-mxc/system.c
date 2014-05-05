/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2006-2014 Freescale Semiconductor, Inc.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, Emcraft Systems Ltd, yanok@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/system.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#ifdef CONFIG_SMP
#include <linux/smp.h>
#endif
#include <asm/mach-types.h>

static void __iomem *wdog_base;
extern u32 enable_ldo_mode;

extern int dvfs_core_is_active;
extern void stop_dvfs(void);

static void arch_reset_special_mode(char mode, const char *cmd)
{
	if (strcmp(cmd, "download") == 0)
		do_switch_mfgmode();
	else if (strcmp(cmd, "recovery") == 0)
		do_switch_recovery();
	else if (strcmp(cmd, "fastboot") == 0)
		do_switch_fastboot();
}

/*
 * Reset the system. It is called by machine_restart().
 */
void arch_reset(char mode, const char *cmd)
{
	unsigned int wcr_enable;

	arch_reset_special_mode(mode, cmd);

#ifdef CONFIG_ARCH_MX6
	/* wait for reset to assert... */
	if (enable_ldo_mode == LDO_MODE_BYPASSED && !(machine_is_mx6sl_evk()
		|| machine_is_mx6sl_arm2())) {
		/*On Sabresd board use WDOG2 to reset external PMIC, so here do
		* more WDOG2 reset.*/
		wcr_enable = 0x14;
		__raw_writew(wcr_enable, IO_ADDRESS(MX6Q_WDOG2_BASE_ADDR));
		__raw_writew(wcr_enable, IO_ADDRESS(MX6Q_WDOG2_BASE_ADDR));
	} else {
		wcr_enable = (1 << 2);
		__raw_writew(wcr_enable, wdog_base);
		/* errata TKT039676, SRS bit may be missed when
		SRC sample it, need to write the wdog controller
		twice to avoid it */
		__raw_writew(wcr_enable, wdog_base);
	}

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	return;
#endif

#ifdef CONFIG_MACH_MX51_EFIKAMX
	if (machine_is_mx51_efikamx()) {
		mx51_efikamx_reset();
		return;
	}
#endif
#ifdef CONFIG_ARCH_MX51
	/* Workaround to reset NFC_CONFIG3 register
	 * due to the chip warm reset does not reset it
	 */
	 if (cpu_is_mx53())
		__raw_writel(0x20600, MX53_IO_ADDRESS(MX53_NFC_BASE_ADDR)+0x28);
	 if (cpu_is_mx51())
		__raw_writel(0x20600, MX51_IO_ADDRESS(MX51_NFC_BASE_ADDR)+0x28);
#endif

#ifdef CONFIG_ARCH_MX5
	/* Stop DVFS-CORE before reboot. */
	if (dvfs_core_is_active)
		stop_dvfs();
#endif

	if (cpu_is_mx1()) {
		wcr_enable = (1 << 0);
	} else {
		struct clk *clk;

		clk = clk_get_sys("imx2-wdt.0", NULL);
		if (!IS_ERR(clk))
			clk_enable(clk);
		wcr_enable = (1 << 2);
	}

	/* Assert SRS signal */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

void mxc_arch_reset_init(void __iomem *base)
{
	wdog_base = base;
}
