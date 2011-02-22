/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Jason Chen <jason.chen@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx5_vpu_data_entry_single(soc, vpu_reset, vpu_pg)	\
	{							\
		.iobase = soc ## _VPU_BASE_ADDR,		\
		.irq = soc ## _INT_VPU,				\
		.reset = vpu_reset,				\
		.pg = vpu_pg,					\
	}

#ifdef CONFIG_SOC_IMX51
void mx51_vpu_reset(void)
{
	u32 reg;
	void __iomem *src_base;

	src_base = ioremap(MX51_SRC_BASE_ADDR, PAGE_SIZE);

	/* mask interrupt due to vpu passed reset */
	reg = __raw_readl(src_base + 0x18);
	reg |= 0x02;
	__raw_writel(reg, src_base + 0x18);

	reg = __raw_readl(src_base);
	reg |= 0x5;    /* warm reset vpu */
	__raw_writel(reg, src_base);
	while (__raw_readl(src_base) & 0x04)
		;

	iounmap(src_base);
}

void mx51_vpu_pg(int enable)
{
	if (enable) {
		__raw_writel(MXC_PGCR_PCR, MX51_PGC_VPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MX51_PGC_VPU_PGSR);
	} else {
		__raw_writel(0x0, MX51_PGC_VPU_PGCR);
		if (__raw_readl(MX51_PGC_VPU_PGSR) & MXC_PGSR_PSR)
			printk(KERN_DEBUG "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MX51_PGC_VPU_PGSR);
	}
}
const struct imx_vpu_data imx51_vpu_data __initconst =
			imx5_vpu_data_entry_single(MX51,
			mx51_vpu_reset, mx51_vpu_pg);
#endif

#ifdef CONFIG_SOC_IMX53
void mx53_vpu_reset(void)
{
	u32 reg;
	void __iomem *src_base;

	src_base = ioremap(MX53_SRC_BASE_ADDR, PAGE_SIZE);

	/* mask interrupt due to vpu passed reset */
	reg = __raw_readl(src_base + 0x18);
	reg |= 0x02;
	__raw_writel(reg, src_base + 0x18);

	reg = __raw_readl(src_base);
	reg |= 0x5;    /* warm reset vpu */
	__raw_writel(reg, src_base);
	while (__raw_readl(src_base) & 0x04)
		;

	iounmap(src_base);
}

void mx53_vpu_pg(int enable)
{
	if (enable) {
		__raw_writel(MXC_PGCR_PCR, MX53_PGC_VPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MX53_PGC_VPU_PGSR);
	} else {
		__raw_writel(0x0, MX53_PGC_VPU_PGCR);
		if (__raw_readl(MX53_PGC_VPU_PGSR) & MXC_PGSR_PSR)
			printk(KERN_DEBUG "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MX53_PGC_VPU_PGSR);
	}
}

const struct imx_vpu_data imx53_vpu_data __initconst =
			imx5_vpu_data_entry_single(MX53,
			mx53_vpu_reset, mx53_vpu_pg);
#endif

struct platform_device *__init imx_add_vpu(
		const struct imx_vpu_data *data)
{
	struct mxc_vpu_platform_data pdata;
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	pdata.reset = data->reset;
	pdata.pg = data->pg;

	return imx_add_platform_device("mxc_vpu", -1,
			res, ARRAY_SIZE(res), &pdata, sizeof(pdata));
}

