// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Mobiveil PCIe Host controller
 *
 * Copyright (c) 2018 Mobiveil Inc.
 * Copyright 2019 NXP
 *
 * Author: Subrahmanya Lingappa <l.subrahmanya@mobiveil.co.in>
 * Refactor: Zhiqiang Hou <Zhiqiang.Hou@nxp.com>
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "pcie-mobiveil.h"

/*
 * mobiveil_pcie_sel_page - routine to access paged register
 *
 * Registers whose address greater than PAGED_ADDR_BNDRY (0xc00) are paged,
 * for this scheme to work extracted higher 6 bits of the offset will be
 * written to pg_sel field of PAB_CTRL register and rest of the lower 10
 * bits enabled with PAGED_ADDR_BNDRY are used as offset of the register.
 */
static void mobiveil_pcie_sel_page(struct mobiveil_pcie *pcie, u8 pg_idx)
{
	u32 val;

	val = readl(pcie->csr_axi_slave_base + PAB_CTRL);
	val &= ~(PAGE_SEL_MASK << PAGE_SEL_SHIFT);
	val |= (pg_idx & PAGE_SEL_MASK) << PAGE_SEL_SHIFT;

	writel(val, pcie->csr_axi_slave_base + PAB_CTRL);
}

static void *mobiveil_pcie_comp_addr(struct mobiveil_pcie *pcie, u32 off)
{
	if (off < PAGED_ADDR_BNDRY) {
		/* For directly accessed registers, clear the pg_sel field */
		mobiveil_pcie_sel_page(pcie, 0);
		return pcie->csr_axi_slave_base + off;
	}

	mobiveil_pcie_sel_page(pcie, OFFSET_TO_PAGE_IDX(off));
	return pcie->csr_axi_slave_base + OFFSET_TO_PAGE_ADDR(off);
}

static int mobiveil_pcie_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	switch (size) {
	case 4:
		*val = readl(addr);
		break;
	case 2:
		*val = readw(addr);
		break;
	case 1:
		*val = readb(addr);
		break;
	default:
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int mobiveil_pcie_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	switch (size) {
	case 4:
		writel(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	case 1:
		writeb(val, addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

u32 csr_read(struct mobiveil_pcie *pcie, u32 off, size_t size)
{
	void *addr;
	u32 val;
	int ret;

	addr = mobiveil_pcie_comp_addr(pcie, off);

	ret = mobiveil_pcie_read(addr, size, &val);
	if (ret)
		dev_err(&pcie->pdev->dev, "read CSR address failed\n");

	return val;
}

void csr_write(struct mobiveil_pcie *pcie, u32 val, u32 off, size_t size)
{
	void *addr;
	int ret;

	addr = mobiveil_pcie_comp_addr(pcie, off);

	ret = mobiveil_pcie_write(addr, size, val);
	if (ret)
		dev_err(&pcie->pdev->dev, "write CSR address failed\n");
}

bool mobiveil_pcie_link_up(struct mobiveil_pcie *pcie)
{
	if (pcie->ops->link_up)
		return pcie->ops->link_up(pcie);

	return (csr_readl(pcie, LTSSM_STATUS) &
		LTSSM_STATUS_L0_MASK) == LTSSM_STATUS_L0;
}

void program_ib_windows(struct mobiveil_pcie *pcie, int win_num, u64 cpu_addr,
			u64 pci_addr, u32 type, u64 size)
{
	u32 value;
	u64 size64 = ~(size - 1);

	if (win_num >= pcie->ppio_wins) {
		dev_err(&pcie->pdev->dev,
			"ERROR: max inbound windows reached !\n");
		return;
	}

	value = csr_readl(pcie, PAB_PEX_AMAP_CTRL(win_num));
	value &= ~(AMAP_CTRL_TYPE_MASK << AMAP_CTRL_TYPE_SHIFT | WIN_SIZE_MASK);
	value |= type << AMAP_CTRL_TYPE_SHIFT | 1 << AMAP_CTRL_EN_SHIFT |
		 (lower_32_bits(size64) & WIN_SIZE_MASK);
	csr_writel(pcie, value, PAB_PEX_AMAP_CTRL(win_num));

	csr_writel(pcie, upper_32_bits(size64),
		   PAB_EXT_PEX_AMAP_SIZEN(win_num));

	csr_writel(pcie, lower_32_bits(cpu_addr),
		   PAB_PEX_AMAP_AXI_WIN(win_num));
	csr_writel(pcie, upper_32_bits(cpu_addr),
		   PAB_EXT_PEX_AMAP_AXI_WIN(win_num));

	csr_writel(pcie, lower_32_bits(pci_addr),
		   PAB_PEX_AMAP_PEX_WIN_L(win_num));
	csr_writel(pcie, upper_32_bits(pci_addr),
		   PAB_PEX_AMAP_PEX_WIN_H(win_num));

	pcie->ib_wins_configured++;
}

/*
 * routine to program the outbound windows
 */
void __program_ob_windows(struct mobiveil_pcie *pcie, u8 func_no, int win_num,
			  u64 cpu_addr, u64 pci_addr, u32 type, u64 size)
{
	u32 value;
	u64 size64 = ~(size - 1);

	/*
	 * program Enable Bit to 1, Type Bit to (00) base 2, AXI Window Size Bit
	 * to 4 KB in PAB_AXI_AMAP_CTRL register
	 */
	value = csr_readl(pcie, PAB_AXI_AMAP_CTRL(win_num));
	value &= ~(WIN_TYPE_MASK << WIN_TYPE_SHIFT | WIN_SIZE_MASK);
	value |= 1 << WIN_ENABLE_SHIFT | type << WIN_TYPE_SHIFT |
		 (lower_32_bits(size64) & WIN_SIZE_MASK);
	csr_writel(pcie, value, PAB_AXI_AMAP_CTRL(win_num));

	csr_writel(pcie, upper_32_bits(size64), PAB_EXT_AXI_AMAP_SIZE(win_num));

	csr_writel(pcie, func_no, PAB_AXI_AMAP_PCI_HDR_PARAM(win_num));
	/*
	 * program AXI window base with appropriate value in
	 * PAB_AXI_AMAP_AXI_WIN0 register
	 */
	csr_writel(pcie, lower_32_bits(cpu_addr) & (~AXI_WINDOW_ALIGN_MASK),
		   PAB_AXI_AMAP_AXI_WIN(win_num));
	csr_writel(pcie, upper_32_bits(cpu_addr),
		   PAB_EXT_AXI_AMAP_AXI_WIN(win_num));

	csr_writel(pcie, lower_32_bits(pci_addr),
		   PAB_AXI_AMAP_PEX_WIN_L(win_num));
	csr_writel(pcie, upper_32_bits(pci_addr),
		   PAB_AXI_AMAP_PEX_WIN_H(win_num));
}

void program_ob_windows(struct mobiveil_pcie *pcie, int win_num, u64 cpu_addr,
			u64 pci_addr, u32 type, u64 size)
{
	if (win_num >= pcie->apio_wins) {
		dev_err(&pcie->pdev->dev,
			"ERROR: max outbound windows reached !\n");
		return;
	}

	__program_ob_windows(pcie, 0, win_num, cpu_addr,
			     pci_addr, type, size);

	pcie->ob_wins_configured++;
}

void program_ob_windows_ep(struct mobiveil_pcie *pcie, u8 func_no, int win_num,
			   u64 cpu_addr, u64 pci_addr, u32 type, u64 size)
{
	if (size & (size - 1))
		size = 1 << (1 + ilog2(size));

	__program_ob_windows(pcie, func_no, win_num, cpu_addr,
			     pci_addr, type, size);
}

void program_ib_windows_ep(struct mobiveil_pcie *pcie, u8 func_no,
			   int bar, u64 phys)
{
	csr_writel(pcie, upper_32_bits(phys),
		   PAB_EXT_PEX_BAR_AMAP(func_no, bar));
	csr_writel(pcie, lower_32_bits(phys) | PEX_BAR_AMAP_EN,
		   PAB_PEX_BAR_AMAP(func_no, bar));
}

void mobiveil_pcie_disable_ib_win_ep(struct mobiveil_pcie *pcie,
				     u8 func_no, u8 bar)
{
	u32 val;

	val = csr_readl(pcie, PAB_PEX_BAR_AMAP(func_no, bar));
	val &= ~(1 << 0);
	csr_writel(pcie, val, PAB_PEX_BAR_AMAP(func_no, bar));
}

void mobiveil_pcie_disable_ob_win(struct mobiveil_pcie *pcie, int win_num)
{
	u32 val;

	val = csr_readl(pcie, PAB_AXI_AMAP_CTRL(win_num));
	val &= ~(1 << WIN_ENABLE_SHIFT);
	csr_writel(pcie, val, PAB_AXI_AMAP_CTRL(win_num));
}

void mobiveil_pcie_enable_bridge_pio(struct mobiveil_pcie *pcie)
{
	u32 val;

	val = csr_readl(pcie, PAB_CTRL);
	val |= 1 << AMBA_PIO_ENABLE_SHIFT;
	val |= 1 << PEX_PIO_ENABLE_SHIFT;
	csr_writel(pcie, val, PAB_CTRL);
}

void mobiveil_pcie_enable_engine_apio(struct mobiveil_pcie *pcie)
{
	u32 val;

	val = csr_readl(pcie, PAB_AXI_PIO_CTRL);
	val |= APIO_EN_MASK;
	csr_writel(pcie, val, PAB_AXI_PIO_CTRL);
}

void mobiveil_pcie_enable_engine_ppio(struct mobiveil_pcie *pcie)
{
	u32 val;

	val = csr_readl(pcie, PAB_PEX_PIO_CTRL);
	val |= 1 << PIO_ENABLE_SHIFT;
	csr_writel(pcie, val, PAB_PEX_PIO_CTRL);
}

void mobiveil_pcie_enable_msi_ep(struct mobiveil_pcie *pcie)
{
	u32 val;

	val =  csr_readl(pcie, PAB_INTP_AMBA_MISC_ENB);
	val |= PAB_INTP_PAMR;
	csr_writel(pcie, val, PAB_INTP_AMBA_MISC_ENB);
}

int mobiveil_bringup_link(struct mobiveil_pcie *pcie)
{
	int retries;

	/* check if the link is up or not */
	for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		if (mobiveil_pcie_link_up(pcie))
			return 0;

		usleep_range(LINK_WAIT_MIN, LINK_WAIT_MAX);
	}

	dev_info(&pcie->pdev->dev, "link never came up\n");

	return -ETIMEDOUT;
}
