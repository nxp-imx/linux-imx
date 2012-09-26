/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/iram_alloc.h>
#include <linux/delay.h>
#include <linux/amba/bus.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/hardware/coresight.h>

static struct __init amba_device mx6_etb_device = {
	.dev = {
		.init_name = "etb",
		},
	.res = {
		.start = MX6Q_ETB_BASE_ADDR,
		.end = MX6Q_ETB_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
		},
	.periphid = 0x3bb907,
};

static struct __init amba_device mx6_etm_device[] = {
	{
	 .dev = {
		 .init_name = "etm.0",
		 },
	 .res = {
		 .start = MX6Q_PTM0_BASE_ADDR,
		 .end = MX6Q_PTM0_BASE_ADDR + SZ_4K - 1,
		 },
	 .periphid = 0x1bb950,
	},
	{
	 .dev = {
		 .init_name = "etm.1",
		 },
	 .res = {
		 .start = MX6Q_PTM1_BASE_ADDR,
		 .end = MX6Q_PTM1_BASE_ADDR + SZ_4K - 1,
		 },
	 .periphid = 0x1bb950,
	},
	{
	 .dev = {
		 .init_name = "etm.2",
		 },
	 .res = {
		 .start = MX6Q_PTM2_BASE_ADDR,
		 .end = MX6Q_PTM2_BASE_ADDR + SZ_4K - 1,
		 },
	 .periphid = 0x1bb950,
	},
	{
	 .dev = {
		 .init_name = "etm.3",
		 },
	 .res = {
		 .start = MX6Q_PTM3_BASE_ADDR,
		 .end = MX6Q_PTM3_BASE_ADDR + SZ_4K - 1,
		 },
	 .periphid = 0x1bb950,
	},
};

#define FUNNEL_CTL 0
static int __init etm_init(void)
{
	int i;
	__iomem void *base;
	base = ioremap(0x02144000, SZ_4K);
	/*Unlock Funnel*/
	__raw_writel(UNLOCK_MAGIC, base + CSMR_LOCKACCESS);
	/*Enable all funnel port*/
	__raw_writel(__raw_readl(base + FUNNEL_CTL) | 0xFF,
		base + FUNNEL_CTL);
	/*Lock Funnel*/
	__raw_writel(0, base + CSMR_LOCKACCESS);
		iounmap(base);

	amba_device_register(&mx6_etb_device, &iomem_resource);
	for (i = 0; i < num_possible_cpus(); i++)
		amba_device_register(mx6_etm_device + i, &iomem_resource);

	return 0;
}

subsys_initcall(etm_init);
