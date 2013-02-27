/*
 * arch/arm/mach-mx6/msi.c
 *
 * PCI MSI support for the imx processor
 *
 * Copyright (c) 2013 Boundary Devices.
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#include <linux/pci.h>
#include <linux/msi.h>
#include <asm/bitops.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include "msi.h"


#define IMX_NUM_MSI_IRQS 128
static DECLARE_BITMAP(msi_irq_in_use, IMX_NUM_MSI_IRQS);

static void imx_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	int i, j;
	unsigned int status;
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned int base_irq = IRQ_IMX_MSI_0;

	chained_irq_enter(chip, desc);
	for (i = 0; i < 8; i++) {
		status = imx_pcie_msi_pending(i);
		while (status) {
			j = __fls(status);
			generic_handle_irq(base_irq + j);
			status &= ~(1 << j);
		}
		base_irq += 32;
	}
	chained_irq_exit(chip, desc);
}

/*
* Dynamic irq allocate and deallocation
*/
int create_irq(void)
{
	int irq, pos;

	do {
		pos = find_first_zero_bit(msi_irq_in_use, IMX_NUM_MSI_IRQS);
		if ((unsigned int)pos >= IMX_NUM_MSI_IRQS)
			return -ENOSPC;
		/* test_and_set_bit operates on 32-bits at a time */
	} while (test_and_set_bit(pos, msi_irq_in_use));

	irq = IRQ_IMX_MSI_0 + pos;
	dynamic_irq_init(irq);
	return irq;
}

void destroy_irq(unsigned int irq)
{
	int pos = irq - IRQ_IMX_MSI_0;

	dynamic_irq_cleanup(irq);
	clear_bit(pos, msi_irq_in_use);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	destroy_irq(irq);
}

static void imx_msi_irq_ack(struct irq_data *d)
{
	return;
}

static void imx_msi_irq_enable(struct irq_data *d)
{
	imx_pcie_enable_irq(d->irq - IRQ_IMX_MSI_0, 1);
	return unmask_msi_irq(d);
}

static void imx_msi_irq_disable(struct irq_data *d)
{
	imx_pcie_enable_irq(d->irq - IRQ_IMX_MSI_0, 0);
	return mask_msi_irq(d);
}

static void imx_msi_irq_mask(struct irq_data *d)
{
	imx_pcie_mask_irq(d->irq - IRQ_IMX_MSI_0, 1);
	return mask_msi_irq(d);
}

static void imx_msi_irq_unmask(struct irq_data *d)
{
	imx_pcie_mask_irq(d->irq - IRQ_IMX_MSI_0, 0);
	return unmask_msi_irq(d);
}

static struct irq_chip imx_msi_chip = {
	.name = "PCIe-MSI",
	.irq_ack = imx_msi_irq_ack,
	.irq_enable = imx_msi_irq_enable,
	.irq_disable = imx_msi_irq_disable,
	.irq_mask = imx_msi_irq_mask,
	.irq_unmask = imx_msi_irq_unmask,
};

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq = create_irq();
	struct msi_msg msg;

	if (irq < 0)
		return irq;

	irq_set_msi_desc(irq, desc);

	msg.address_hi = 0x0;
	msg.address_lo = MSI_MATCH_ADDR;
	/* 16bits msg.data: set cpu type to the upper 8bits*/
	msg.data = (mxc_cpu_type << 8) | ((irq - IRQ_IMX_MSI_0) & 0xFF);

	write_msi_msg(irq, &msg);
	irq_set_chip_and_handler(irq, &imx_msi_chip, handle_simple_irq);
	set_irq_flags(irq, IRQF_VALID);
	pr_info("IMX-PCIe: MSI 0x%04x @%#x:%#x, irq = %d\n",
			msg.data, msg.address_hi,
			msg.address_lo, irq);
	return 0;
}

void imx_msi_init(void)
{
	irq_set_chained_handler(MXC_INT_PCIE_0, imx_msi_handler);
}
