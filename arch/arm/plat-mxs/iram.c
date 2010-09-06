/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>

static unsigned long iram_phys_base;
static __iomem void *iram_virt_base;
static struct gen_pool *iram_pool;

#define iram_phys_to_virt(p) (iram_virt_base + ((p) - iram_phys_base))

void *iram_alloc(unsigned int size, unsigned long *dma_addr)
{
	if (!iram_pool)
		return NULL;

	*dma_addr = gen_pool_alloc(iram_pool, size);
	pr_debug("iram alloc - %dB@0x%p\n", size, (void *)*dma_addr);

	WARN_ON(!*dma_addr);
	if (!*dma_addr)
		return NULL;

	return iram_phys_to_virt(*dma_addr);
}
EXPORT_SYMBOL(iram_alloc);

void iram_free(unsigned long addr, unsigned int size)
{
	if (!iram_pool)
		return;

	gen_pool_free(iram_pool, addr, size);
}
EXPORT_SYMBOL(iram_free);

int __init iram_init(unsigned long base, unsigned long size)
{
	iram_phys_base = base;

	iram_pool = gen_pool_create(12, -1);
	gen_pool_add(iram_pool, base, size, -1);
	iram_virt_base = ioremap(iram_phys_base, size);

	pr_info("i.MX IRAM pool: %ld KB@0x%p\n", size / 1024, iram_virt_base);
	return 0;
}
