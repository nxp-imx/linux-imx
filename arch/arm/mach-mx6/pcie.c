/*
 * arch/arm/mach-mx6/pcie.c
 *
 * PCIe host controller driver for IMX6 SOCs
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Bits taken from arch/arm/mach-dove/pcie.c
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/sizes.h>

#include "crm_regs.h"

/* Register Definitions */
#define PRT_LOG_R_BaseAddress 0x700

/* Register DB_R0 */
/* Debug Register 0 */
#define DB_R0 (PRT_LOG_R_BaseAddress + 0x28)
#define DB_R0_RegisterSize 32
#define DB_R0_RegisterResetValue 0x0
#define DB_R0_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R0 */

/* Register DB_R1 */
/* Debug Register 1 */
#define DB_R1 (PRT_LOG_R_BaseAddress + 0x2c)
#define DB_R1_RegisterSize 32
#define DB_R1_RegisterResetValue 0x0
#define DB_R1_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R1 */

#define ATU_R_BaseAddress 0x900
#define ATU_VIEWPORT_R (ATU_R_BaseAddress + 0x0)
#define ATU_REGION_CTRL1_R (ATU_R_BaseAddress + 0x4)
#define ATU_REGION_CTRL2_R (ATU_R_BaseAddress + 0x8)
#define ATU_REGION_LOWBASE_R (ATU_R_BaseAddress + 0xC)
#define ATU_REGION_UPBASE_R (ATU_R_BaseAddress + 0x10)
#define ATU_REGION_LIMIT_ADDR_R (ATU_R_BaseAddress + 0x14)
#define ATU_REGION_LOW_TRGT_ADDR_R (ATU_R_BaseAddress + 0x18)
#define ATU_REGION_UP_TRGT_ADDR_R (ATU_R_BaseAddress + 0x1C)

/* GPR1: iomuxc_gpr1_pcie_ref_clk_en(iomuxc_gpr1[16]) */
#define iomuxc_gpr1_pcie_ref_clk_en		(1 << 16)
/* GPR1: iomuxc_gpr1_test_powerdown(iomuxc_gpr1_18) */
#define iomuxc_gpr1_test_powerdown		(1 << 18)

/* GPR12: iomuxc_gpr12_los_level(iomuxc_gpr12[8:4]) */
#define iomuxc_gpr12_los_level			(0x1F << 4)
/* GPR12: iomuxc_gpr12_app_ltssm_enable(iomuxc_gpr12[10]) */
#define iomuxc_gpr12_app_ltssm_enable		(1 << 10)
/* GPR12: iomuxc_gpr12_device_type(iomuxc_gpr12[15:12]) */
#define iomuxc_gpr12_device_type		(0xF << 12)

/* GPR8: iomuxc_gpr8_tx_deemph_gen1(iomuxc_gpr8[5:0]) */
#define iomuxc_gpr8_tx_deemph_gen1		(0x3F << 0)
/* GPR8: iomuxc_gpr8_tx_deemph_gen2_3p5db(iomuxc_gpr8[11:6]) */
#define iomuxc_gpr8_tx_deemph_gen2_3p5db	(0x3F << 6)
/* GPR8: iomuxc_gpr8_tx_deemph_gen2_6db(iomuxc_gpr8[17:12]) */
#define iomuxc_gpr8_tx_deemph_gen2_6db		(0x3F << 12)
/* GPR8: iomuxc_gpr8_tx_swing_full(iomuxc_gpr8[24:18]) */
#define iomuxc_gpr8_tx_swing_full		(0x7F << 18)
/* GPR8: iomuxc_gpr8_tx_swing_low(iomuxc_gpr8[31:25]) */
#define iomuxc_gpr8_tx_swing_low		(0x7F << 25)

/* End of Register Definitions */

#define PCIE_DBI_BASE_ADDR	(PCIE_ARB_END_ADDR - SZ_16K + 1)

#define  PCIE_CONF_BUS(b)		(((b) & 0xFF) << 16)
#define  PCIE_CONF_DEV(d)		(((d) & 0x1F) << 11)
#define  PCIE_CONF_FUNC(f)		(((f) & 0x7) << 8)

enum {
	MemRdWr = 0,
	MemRdLk = 1,
	IORdWr = 2,
	CfgRdWr0 = 4,
	CfgRdWr1 = 5
};

struct imx_pcie_port {
	u8			index;
	u8			root_bus_nr;
	void __iomem		*base;
	void __iomem		*dbi_base;
	spinlock_t		conf_lock;

	char			io_space_name[16];
	char			mem_space_name[16];

	struct resource		res[2];
};

static struct imx_pcie_port imx_pcie_port[1];
static int num_pcie_ports;

/* IMX PCIE GPR configure routines */
static inline void imx_pcie_clrset(u32 mask, u32 val, void __iomem *addr)
{
	writel(((readl(addr) & ~mask) | (val & mask)), addr);
}

static struct imx_pcie_port *bus_to_port(int bus)
{
	int i;

	for (i = num_pcie_ports - 1; i >= 0; i--) {
		int rbus = imx_pcie_port[i].root_bus_nr;
		if (rbus != -1 && rbus <= bus)
			break;
	}

	return i >= 0 ? imx_pcie_port + i : NULL;
}

static int __init imx_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct imx_pcie_port *pp;

	if (nr >= num_pcie_ports)
		return 0;

	pp = &imx_pcie_port[nr];
	pp->root_bus_nr = sys->busnr;

	/*
	 * IORESOURCE_IO
	 */
	snprintf(pp->io_space_name, sizeof(pp->io_space_name),
		 "PCIe %d I/O", pp->index);
	pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
	pp->res[0].name = pp->io_space_name;
	if (pp->index == 0) {
		pp->res[0].start = PCIE_ARB_BASE_ADDR;
		pp->res[0].end = pp->res[0].start + SZ_64K - 1;
	}
	pp->res[0].flags = IORESOURCE_IO;
	if (request_resource(&ioport_resource, &pp->res[0]))
		panic("Request PCIe IO resource failed\n");
	sys->resource[0] = &pp->res[0];

	/*
	 * IORESOURCE_MEM
	 */
	snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->index);
	pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
	pp->res[1].name = pp->mem_space_name;
	if (pp->index == 0) {
		pp->res[1].start = PCIE_ARB_BASE_ADDR + SZ_64K;
		pp->res[1].end = pp->res[1].start + SZ_16M - SZ_128K - 1;
	}
	pp->res[1].flags = IORESOURCE_MEM;
	if (request_resource(&iomem_resource, &pp->res[1]))
		panic("Request PCIe Memory resource failed\n");
	sys->resource[1] = &pp->res[1];

	sys->resource[2] = NULL;

	return 1;
}

static void __init imx_pcie_preinit(void)
{
	pcibios_setup("debug");
}

static int imx_pcie_link_up(void __iomem *dbi_base)
{
	/* Check the pcie link up or link down */
	u32 rc, iterations = 0x100000;

	do {
		/* link is debug bit 36 debug 1 start in bit 32 */
		rc = readl(dbi_base + DB_R1) & (0x1 << (36-32)) ;
		iterations--;
		if ((iterations % 0x100000) == 0)
			pr_info("link up failed!\n");
	} while (!rc && iterations);

	if (!rc)
		return 0;
	return 1;
}

static int pcie_valid_config(struct imx_pcie_port *pp, int bus_num, int dev)
{
	/*If there is no link, then there is no device*/
	if (bus_num != pp->root_bus_nr) {
		if (!imx_pcie_link_up(pp->dbi_base))
			return 0;
	}

	/*
	 * Don't go out when trying to access nonexisting devices
	 * on the local bus.
	 * We have only one slot on the root port.
	 */
	if (bus_num == pp->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static void imx_pcie_regions_setup(void __iomem *dbi_base)
{
	/*
	 * i.MX6 defines 16MB in the AXI address map for PCIe.
	 *
	 * That address space excepted the pcie registers is
	 * split and defined into different regions by iATU,
	 * with sizes and offsets as follows:
	 *
	 * 0x0100_0000 --- 0x0100_FFFF 64KB IORESOURCE_IO
	 * 0x0101_0000 --- 0x01FE_FFFF 16MB - 128KB IORESOURCE_MEM
	 * 0x01FF_0000 --- 0x01FF_FFFF 64KB Cfg + Registers
	 */

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	writel(readl(dbi_base + PCI_COMMAND)
			| PCI_COMMAND_IO
			| PCI_COMMAND_MEMORY
			| PCI_COMMAND_MASTER,
			dbi_base + PCI_COMMAND);
	/*
	 * region0 outbound used to access target cfg
	 */
	writel(0, dbi_base + ATU_VIEWPORT_R);
	writel(PCIE_ARB_END_ADDR - SZ_64K + 1, dbi_base + ATU_REGION_LOWBASE_R);
	writel(0, dbi_base + ATU_REGION_UPBASE_R);
	writel(PCIE_ARB_END_ADDR, dbi_base + ATU_REGION_LIMIT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_UP_TRGT_ADDR_R);
	writel(CfgRdWr0, dbi_base + ATU_REGION_CTRL1_R);
	writel((1<<31), dbi_base + ATU_REGION_CTRL2_R);

	/*
	 * region1 outbound used to as IORESOURCE_IO
	 */
	writel(1, dbi_base + ATU_VIEWPORT_R);
	writel(0, dbi_base + ATU_REGION_LOWBASE_R);
	writel(0, dbi_base + ATU_REGION_UPBASE_R);
	writel(SZ_64K - 1, dbi_base + ATU_REGION_LIMIT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_UP_TRGT_ADDR_R);
	writel(IORdWr, dbi_base + ATU_REGION_CTRL1_R);
	writel((1<<31), dbi_base + ATU_REGION_CTRL2_R);

	/*
	 * region2 outbound used to as IORESOURCE_MEM
	 */
	writel(2, dbi_base + ATU_VIEWPORT_R);
	writel(0, dbi_base + ATU_REGION_LOWBASE_R);
	writel(0, dbi_base + ATU_REGION_UPBASE_R);
	writel(SZ_16M - SZ_128K - 1, dbi_base + ATU_REGION_LIMIT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_LOW_TRGT_ADDR_R);
	writel(0, dbi_base + ATU_REGION_UP_TRGT_ADDR_R);
	writel(MemRdWr, dbi_base + ATU_REGION_CTRL1_R);
	writel((1<<31), dbi_base + ATU_REGION_CTRL2_R);
}

static int imx_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	u32 va_address;

	if (pcie_valid_config(pp, bus->number, PCI_SLOT(devfn)) == 0) {
		*val = 0xFFFFFFFF;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}


	spin_lock_irqsave(&pp->conf_lock, flags);

	va_address = (u32)pp->base +
		PCIE_CONF_BUS(bus->number) +
		PCIE_CONF_DEV(PCI_SLOT(devfn)) +
		PCIE_CONF_FUNC(PCI_FUNC(devfn)) +
		(where & ~0x3);

	*val = readl(va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xFF;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xFFFF;

	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return PCIBIOS_SUCCESSFUL;
}

static int imx_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	unsigned long flags;
	u32 va_address = 0, mask = 0, tmp = 0;
	int ret = PCIBIOS_SUCCESSFUL;

	if (pcie_valid_config(pp, bus->number, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	spin_lock_irqsave(&pp->conf_lock, flags);

	va_address = (u32)pp->base +
		PCIE_CONF_BUS(bus->number) +
		PCIE_CONF_DEV(PCI_SLOT(devfn)) +
		PCIE_CONF_FUNC(PCI_FUNC(devfn)) +
		(where & ~0x3);

	if (size == 4) {
		writel(val, va_address);
		goto exit;
	}

	if (size == 2)
		mask = ~(0xFFFF << ((where & 0x3) * 8));
	else if (size == 1)
		mask = ~(0xFF << ((where & 0x3) * 8));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = readl(va_address) & mask;
	tmp |= val << ((where & 0x3) * 8);
	writel(tmp, va_address);
exit:
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	return ret;
}

static struct pci_ops imx_pcie_ops = {
	.read = imx_pcie_rd_conf,
	.write = imx_pcie_wr_conf,
};

static struct pci_bus __init *
imx_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct pci_bus *bus;

	if (nr < num_pcie_ports) {
		bus = pci_scan_bus(sys->busnr, &imx_pcie_ops, sys);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int __init imx_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	return MXC_INT_PCIE_0;
}

static struct hw_pci imx_pci __initdata = {
	.nr_controllers	= 1,
	.swizzle	= pci_std_swizzle,
	.setup		= imx_pcie_setup,
	.preinit	= imx_pcie_preinit,
	.scan		= imx_pcie_scan_bus,
	.map_irq	= imx_pcie_map_irq,
};

static void imx_pcie_enable_controller(void)
{
	struct clk *pcie_clk;

	imx_pcie_clrset(iomuxc_gpr1_test_powerdown, 0 << 18, IOMUXC_GPR1);

	/* enable the clks */
	pcie_clk = clk_get(NULL, "pcie_clk");
	if (IS_ERR(pcie_clk))
		pr_err("no pcie clock.\n");

	if (clk_enable(pcie_clk)) {
		pr_err("can't enable pcie clock.\n");
		clk_put(pcie_clk);
	}
}

static void card_reset(void)
{
	/* Add one reset to the pcie external device */
}

static void __init add_pcie_port(void __iomem *base, void __iomem *dbi_base)
{
	if (imx_pcie_link_up(dbi_base)) {
		struct imx_pcie_port *pp = &imx_pcie_port[num_pcie_ports++];

		pr_info("IMX PCIe port: link up.\n");

		pp->index = 0;
		pp->root_bus_nr = -1;
		pp->base = base;
		pp->dbi_base = dbi_base;
		spin_lock_init(&pp->conf_lock);
		memset(pp->res, 0, sizeof(pp->res));
	} else
		pr_info("IMX PCIe port: link down!\n");
}

static int __init imx_pcie_init(void)
{
	void __iomem *base, *dbi_base;

	base = ioremap_nocache(PCIE_ARB_END_ADDR - SZ_64K + 1, SZ_32K);
	if (!base) {
		pr_err("error with ioremap in function %s\n", __func__);
		return -EIO;
	}

	dbi_base = ioremap_nocache(PCIE_DBI_BASE_ADDR, SZ_16K);
	if (!dbi_base) {
		pr_err("error with ioremap in function %s\n", __func__);
		iounmap(base);
		return -EIO;
	}

	/* FIXME the field name should be aligned to RM */
	imx_pcie_clrset(iomuxc_gpr12_app_ltssm_enable, 0 << 10, IOMUXC_GPR12);

	/* configure constant input signal to the pcie ctrl and phy */
	imx_pcie_clrset(iomuxc_gpr12_device_type, PCI_EXP_TYPE_ROOT_PORT << 12,
			IOMUXC_GPR12);
	imx_pcie_clrset(iomuxc_gpr12_los_level, 9 << 4, IOMUXC_GPR12);
	imx_pcie_clrset(iomuxc_gpr8_tx_deemph_gen1, 21 << 0, IOMUXC_GPR8);
	imx_pcie_clrset(iomuxc_gpr8_tx_deemph_gen2_3p5db, 21 << 6, IOMUXC_GPR8);
	imx_pcie_clrset(iomuxc_gpr8_tx_deemph_gen2_6db, 32 << 12, IOMUXC_GPR8);
	imx_pcie_clrset(iomuxc_gpr8_tx_swing_full, 115 << 18, IOMUXC_GPR8);
	imx_pcie_clrset(iomuxc_gpr8_tx_swing_low, 115 << 25, IOMUXC_GPR8);

	/* Enable the pwr, clks and so on */
	imx_pcie_enable_controller();

	imx_pcie_clrset(iomuxc_gpr1_pcie_ref_clk_en, 1 << 16, IOMUXC_GPR1);

	/* togle the external card's reset */
	card_reset() ;

	usleep_range(3000, 4000);
	imx_pcie_regions_setup(dbi_base);
	usleep_range(3000, 4000);

	/* start link up */
	imx_pcie_clrset(iomuxc_gpr12_app_ltssm_enable, 1 << 10, IOMUXC_GPR12);

	/* add the pcie port */
	add_pcie_port(base, dbi_base);

	pci_common_init(&imx_pci);
	pr_info("pcie init successful\n");
	return 0;
}
subsys_initcall(imx_pcie_init);
