/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_COMMON_H__
#define __ASM_ARCH_MXC_COMMON_H__

struct platform_device;
struct clk;

extern void mx1_map_io(void);
extern void mx21_map_io(void);
extern void mx25_map_io(void);
extern void mx27_map_io(void);
extern void mx31_map_io(void);
extern void mx35_map_io(void);
extern void mx37_map_io(void);
extern void mx5_map_io(void);
extern void mxc91231_map_io(void);
extern void mxc_init_irq(void __iomem *);
extern void tzic_init_irq(void __iomem *);
extern void mx1_init_irq(void);
extern void mx21_init_irq(void);
extern void mx25_init_irq(void);
extern void mx27_init_irq(void);
extern void mx31_init_irq(void);
extern void mx35_init_irq(void);
extern void mx5_init_irq(void);
extern void mxc91231_init_irq(void);
extern void mxc_tzic_init_irq(unsigned long);
extern void mxc_timer_init(struct clk *timer_clk, void __iomem *, int);
extern int mx1_clocks_init(unsigned long fref);
extern int mx21_clocks_init(unsigned long lref, unsigned long fref);
extern int mx25_clocks_init(void);
extern int mx27_clocks_init(unsigned long fref);
extern int mx31_clocks_init(unsigned long fref);
extern int mx35_clocks_init(void);
extern int mx37_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2);
extern int mx50_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih);
extern int mx51_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern int mx53_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern int mxc91231_clocks_init(unsigned long fref);
extern void mxc_cpu_init(void);
extern void mxc_cpu_common_init(void);
extern void early_console_setup(unsigned long base, struct clk *clk);
extern int mxc_register_gpios(void);
extern int mxc_register_device(struct platform_device *pdev, void *data);
extern void mxc_set_cpu_type(unsigned int type);
extern void mxc_arch_reset_init(void __iomem *);
extern void mxc91231_power_off(void);
extern void mxc91231_arch_reset(int, const char *);
extern void mxc91231_prepare_idle(void);

#ifdef CONFIG_ARCH_MX5
extern int mx50_revision(void);
extern int mx51_revision(void);
extern int mx53_revision(void);
#else
static inline int mx50_revision(void) {return 0; }
static inline int mx51_revision(void) {return 0; }
static inline int mx53_revision(void) {return 0; }
#endif
#endif
