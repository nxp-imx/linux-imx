/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mach-mx25/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX25
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/hardware.h>

/*!
 * CPU initialization. It is called by fixup_mxc_board()
 */
void __init mxc_cpu_init(void)
{
	if (!system_rev)
		mxc_set_system_rev(0x25, CHIP_REV_1_0);
}
