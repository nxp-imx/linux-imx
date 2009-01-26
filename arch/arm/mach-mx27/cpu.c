/*
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *  Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*!
 * @file mach-mx27/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX27
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
	if (!system_rev) {
		mxc_set_system_rev(0x27, CHIP_REV_2_0);
	}
}
