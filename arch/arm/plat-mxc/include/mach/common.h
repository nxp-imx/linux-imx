/*
 * Copyright (C) 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __ASM_ARCH_MXC_COMMON_H__
#define __ASM_ARCH_MXC_COMMON_H__

struct fec_platform_data;
struct platform_device;
struct clk;

extern char *gp_reg_id;
extern void mx1_map_io(void);
extern void mx21_map_io(void);
extern void mx25_map_io(void);
extern void mx27_map_io(void);
extern void mx31_map_io(void);
extern void mx35_map_io(void);
extern void mx50_map_io(void);
extern void mx51_map_io(void);
extern void mx53_map_io(void);
extern void mx6_map_io(void);
extern void mxc91231_map_io(void);
extern void imx1_init_early(void);
extern void imx21_init_early(void);
extern void imx25_init_early(void);
extern void imx27_init_early(void);
extern void imx31_init_early(void);
extern void imx35_init_early(void);
extern void imx50_init_early(void);
extern void imx51_init_early(void);
extern void imx53_init_early(void);
extern void mxc91231_init_early(void);
extern void mxc_init_irq(void __iomem *);
extern void tzic_init_irq(void __iomem *);
extern void mx1_init_irq(void);
extern void mx21_init_irq(void);
extern void mx25_init_irq(void);
extern void mx27_init_irq(void);
extern void mx31_init_irq(void);
extern void mx35_init_irq(void);
extern void mx50_init_irq(void);
extern void mx51_init_irq(void);
extern void mx53_init_irq(void);
extern void mx6_init_irq(void);
extern void mxc91231_init_irq(void);
extern void epit_timer_init(struct clk *timer_clk, void __iomem *base, int irq);
extern void mxc_timer_init(struct clk *timer_clk, void __iomem *, int);
extern int mx1_clocks_init(unsigned long fref);
extern int mx21_clocks_init(unsigned long lref, unsigned long fref);
extern int mx25_clocks_init(void);
extern int mx27_clocks_init(unsigned long fref);
extern int mx31_clocks_init(unsigned long fref);
extern int mx35_clocks_init(void);
extern int mx51_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern int mx53_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern int mx50_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1);
extern int mx6_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern int mx6sl_clocks_init(unsigned long ckil, unsigned long osc,
			unsigned long ckih1, unsigned long ckih2);
extern void imx6_init_fec(struct fec_platform_data fec_data);
extern int mxc91231_clocks_init(unsigned long fref);
extern int mxc_register_gpios(void);
extern int mxc_register_device(struct platform_device *pdev, void *data);
extern void mxc_set_cpu_type(unsigned int type);
extern void mxc_arch_reset_init(void __iomem *);
extern void mxc91231_power_off(void);
extern void mxc91231_arch_reset(int, const char *);
extern void mxc91231_prepare_idle(void);
extern void mx51_efikamx_reset(void);
extern int mx53_revision(void);
extern int mx50_revision(void);
extern int mx53_display_revision(void);
extern int mxs_reset_block(void __iomem *);
extern void early_console_setup(unsigned long base, struct clk *clk);
extern void mx6_cpu_regulator_init(void);
extern int mx6q_sabreauto_init_pfuze100(u32 int_gpio);
extern int mx6q_sabresd_init_pfuze100(u32 int_gpio);
extern void imx_print_silicon_rev(const char *cpu, int srev);
#endif
