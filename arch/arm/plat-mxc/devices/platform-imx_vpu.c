/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Jason Chen <jason.chen@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx5_vpu_data_entry_single(soc, flag, size, vpu_reset, vpu_pg)	\
	{							\
		.iobase = soc ## _VPU_BASE_ADDR,		\
		.irq_ipi = soc ## _INT_VPU,			\
		.iram_enable = flag,				\
		.iram_size = size,				\
		.reset = vpu_reset,				\
		.pg = vpu_pg,					\
	}

#define imx6_vpu_data_entry_single(soc, flag, size, vpu_reset, vpu_pg)  \
	{                                                       \
		.iobase = soc ## _VPU_BASE_ADDR,                \
		.irq_ipi = soc ## _INT_VPU_IPI,                 \
		.irq_jpg = soc ## _INT_VPU_JPG,			\
		.iram_enable = flag,                            \
		.iram_size = size,                              \
		.reset = vpu_reset,                             \
		.pg = vpu_pg,                                   \
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
			false, 0x14000, mx51_vpu_reset, mx51_vpu_pg);
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
			true, 0x14000, mx53_vpu_reset, mx53_vpu_pg);
#endif

#ifdef CONFIG_SOC_IMX6Q
void mx6q_vpu_reset(void)
{
	u32 reg;
	void __iomem *src_base;

	src_base = ioremap(SRC_BASE_ADDR, PAGE_SIZE);

	/* mask interrupt due to vpu passed reset */
	reg = __raw_readl(src_base + 0x18);
	reg |= 0x02;
	 __raw_writel(reg, src_base + 0x18);

	reg = __raw_readl(src_base);
	reg |= 0x4;    /* warm reset vpu */
	__raw_writel(reg, src_base);
	while (__raw_readl(src_base) & 0x04)
		;

	iounmap(src_base);
}

const struct imx_vpu_data imx6q_vpu_data __initconst =
			imx6_vpu_data_entry_single(MX6Q,
			true, 0x21000, mx6q_vpu_reset, NULL);
#endif

struct platform_device *__init imx_add_vpu(
		const struct imx_vpu_data *data)
{
	struct mxc_vpu_platform_data pdata;
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_16K - 1,
			.name = "vpu_regs",
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq_ipi,
			.end = data->irq_ipi,
			.name = "vpu_ipi_irq",
			.flags = IORESOURCE_IRQ,
		},
#ifdef CONFIG_SOC_IMX6Q
		{
			.start = data->irq_jpg,
			.end = data->irq_jpg,
			.name = "vpu_jpu_irq",
			.flags = IORESOURCE_IRQ,
		},
#endif
	};

	pdata.reset = data->reset;
	pdata.pg = data->pg;
	pdata.iram_enable = data->iram_enable;
	pdata.iram_size = data->iram_size;

	if (!fuse_dev_is_available(MXC_DEV_VPU))
		return ERR_PTR(-ENODEV);

	if (cpu_is_mx6dl())
		pdata.iram_enable = false;

	return imx_add_platform_device("mxc_vpu", -1,
			res, ARRAY_SIZE(res), &pdata, sizeof(pdata));
}

