/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/clk.h>

#define imx5_ipuv3_data_entry_single(soc, size, ipu_init, ipu_pg)	\
	{								\
		.iobase = soc ## _IPU_CTRL_BASE_ADDR,			\
		.irq_err = soc ## _INT_IPU_ERR,				\
		.irq = soc ## _INT_IPU_SYN,				\
		.iosize = size,						\
		.init = ipu_init,					\
		.pg = ipu_pg,						\
	}

#ifdef CONFIG_SOC_IMX51
/*
 * The MIPI HSC unit has been removed from the i.MX51 Reference Manual by
 * the Freescale marketing division. However this did not remove the
 * hardware from the chip which still needs to be configured...
 */
static int __init ipu_mipi_setup(void)
{
	struct clk *hsc_clk;
	void __iomem *hsc_addr;
	int ret = 0;

	hsc_addr = ioremap(MX51_MIPI_HSC_BASE_ADDR, PAGE_SIZE);
	if (!hsc_addr)
		return -ENOMEM;

	hsc_clk = clk_get_sys(NULL, "mipi_hsp");
	if (IS_ERR(hsc_clk)) {
		ret = PTR_ERR(hsc_clk);
		goto unmap;
	}
	clk_enable(hsc_clk);

	/* setup MIPI module to legacy mode */
	__raw_writel(0xF00, hsc_addr);

	/* CSI mode: reserved; DI control mode: legacy (from Freescale BSP) */
	__raw_writel(__raw_readl(hsc_addr + 0x800) | 0x30ff,
			hsc_addr + 0x800);

	clk_disable(hsc_clk);
	clk_put(hsc_clk);
unmap:
	iounmap(hsc_addr);

	return ret;
}

int __init mx51_ipuv3_init(void)
{
	int ret = 0;
	u32 val;

	ret = ipu_mipi_setup();

	/* hard reset the IPU */
	val = readl(MX51_IO_ADDRESS(MX51_SRC_BASE_ADDR));
	val |= 1 << 3;
	writel(val, MX51_IO_ADDRESS(MX51_SRC_BASE_ADDR));

	return ret;
}

void mx51_ipuv3_pg(int enable)
{
	if (enable) {
		__raw_writel(MXC_PGCR_PCR, MX51_PGC_IPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MX51_PGC_IPU_PGSR);
	} else {
		__raw_writel(0x0, MX51_PGC_IPU_PGCR);
		if (__raw_readl(MX51_PGC_IPU_PGSR) & MXC_PGSR_PSR)
			printk(KERN_DEBUG "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MX51_PGC_IPU_PGSR);
	}
}

const struct imx_ipuv3_data imx51_ipuv3_data __initconst =
			imx5_ipuv3_data_entry_single(MX51, SZ_512M,
					mx51_ipuv3_init, mx51_ipuv3_pg);
#endif

#ifdef CONFIG_SOC_IMX53
int __init mx53_ipuv3_init(void)
{
	int ret = 0;
	u32 val;

	/* hard reset the IPU */
	val = readl(MX53_IO_ADDRESS(MX53_SRC_BASE_ADDR));
	val |= 1 << 3;
	writel(val, MX53_IO_ADDRESS(MX53_SRC_BASE_ADDR));

	return ret;
}

void mx53_ipuv3_pg(int enable)
{
	if (enable) {
		__raw_writel(MXC_PGCR_PCR, MX53_PGC_IPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MX53_PGC_IPU_PGSR);
	} else {
		__raw_writel(0x0, MX53_PGC_IPU_PGCR);
		if (__raw_readl(MX53_PGC_IPU_PGSR) & MXC_PGSR_PSR)
			printk(KERN_DEBUG "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MX53_PGC_IPU_PGSR);
	}
}

const struct imx_ipuv3_data imx53_ipuv3_data __initconst =
			imx5_ipuv3_data_entry_single(MX53, SZ_128M,
					mx53_ipuv3_init, mx53_ipuv3_pg);
#endif

struct platform_device *__init imx_add_ipuv3(
		const struct imx_ipuv3_data *data,
		struct imx_ipuv3_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq_err,
			.end = data->irq_err,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	pdata->init = data->init;
	pdata->pg = data->pg;

	return imx_add_platform_device("imx-ipuv3", -1,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata));
}

