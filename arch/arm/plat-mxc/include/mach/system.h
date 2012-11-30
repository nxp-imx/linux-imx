/*
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARCH_MXC_SYSTEM_H__
#define __ASM_ARCH_MXC_SYSTEM_H__

#include <mach/hardware.h>
#include <mach/common.h>

#define LDO_MODE_DEFAULT	0
#define LDO_MODE_BYPASSED	1
#define LDO_MODE_ENABLED	2
extern void mx5_cpu_lp_set(enum mxc_cpu_pwr_mode mode);

void arch_idle(void);

void arch_reset(char mode, const char *cmd);

#ifdef CONFIG_MXC_REBOOT_MFGMODE
void do_switch_mfgmode(void);
void mxc_clear_mfgmode(void);
#else
#define do_switch_mfgmode() do {} while (0)
#define mxc_clear_mfgmode() do {} while (0)
#endif

#ifdef CONFIG_MXC_REBOOT_ANDROID_CMD
void do_switch_recovery(void);
void do_switch_fastboot(void);
#else
#define do_switch_recovery() do {} while (0)
#define do_switch_fastboot() do {} while (0)
#endif

#endif /* __ASM_ARCH_MXC_SYSTEM_H__ */
