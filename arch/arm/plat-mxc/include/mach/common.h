/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_COMMON_H__
#define __ASM_ARCH_MXC_COMMON_H__

struct platform_device;

extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_timer_init(const char *clk_timer);
extern int __init mxc_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2);
extern int mxc_init_devices(void);
extern void mxc_cpu_init(void) __init;
extern void mxc_cpu_common_init(void);
extern void __init early_console_setup(char *);
extern int mxc_register_gpios(void);
extern int mxc_register_device(struct platform_device *pdev, void *data);

#endif
