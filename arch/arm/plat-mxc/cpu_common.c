/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/module.h>
#include <asm/setup.h>
extern int mxc_early_serial_console_init(char *);

/*!
 * @file plat-mxc/cpu_common.c
 *
 * @brief This file contains the common CPU initialization code.
 *
 * @ingroup MSL_MX31 MSL_MXC91321
 */

static void __init system_rev_setup(char **p)
{
	system_rev = simple_strtoul(*p, NULL, 16);
}

__early_param("system_rev=", system_rev_setup);

int mxc_jtag_enabled;		/* OFF: 0 (default), ON: 1 */

/*
 * Here are the JTAG options from the command line. By default JTAG
 * is OFF which means JTAG is not connected and WFI is enabled
 *
 *       "on" --  JTAG is connected, so WFI is disabled
 *       "off" -- JTAG is disconnected, so WFI is enabled
 */

static void __init jtag_wfi_setup(char **p)
{
	if (memcmp(*p, "on", 2) == 0) {
		mxc_jtag_enabled = 1;
		*p += 2;
	} else if (memcmp(*p, "off", 3) == 0) {
		mxc_jtag_enabled = 0;
		*p += 3;
	}
}

__early_param("jtag=", jtag_wfi_setup);

void __init mxc_cpu_common_init(void)
{
	pr_info("CPU is %s%x Revision %u.%u\n",
		(mxc_cpu() < 0x100) ? "i.MX" : "MXC",
		mxc_cpu(), mxc_cpu_rev_major(), mxc_cpu_rev_minor());
}

/**
 * early_console_setup - setup debugging console
 *
 * Consoles started here require little enough setup that we can start using
 * them very early in the boot process, either right after the machine
 * vector initialization, or even before if the drivers can detect their hw.
 *
 * Returns non-zero if a console couldn't be setup.
 * This function is developed based on
 * early_console_setup function as defined in arch/ia64/kernel/setup.c
 */
int __init early_console_setup(char *cmdline)
{
	int earlycons = 0;

#ifdef CONFIG_SERIAL_MXC_CONSOLE
	if (!mxc_early_serial_console_init(cmdline))
		earlycons++;
#endif

	return (earlycons) ? 0 : -1;
}
