/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file plat-mxc/sdma/sdma_malloc.c
 * @brief This file contains functions for SDMA non-cacheable buffers allocation
 *
 * SDMA (Smart DMA) is used for transferring data between MCU and peripherals
 *
 * @ingroup SDMA
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/iram_alloc.h>
#include <asm/dma.h>
#include <mach/hardware.h>


#define DEBUG 0

#if DEBUG
#define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifdef CONFIG_SDMA_IRAM
#define IRAM_SDMA_SIZE	SZ_4K
#endif

/*!
 * Defines SDMA non-cacheable buffers pool
 */
static struct dma_pool *pool;
static struct gen_pool *sdma_iram_pool;

/*!
 * SDMA memory conversion hashing structure
 */
typedef struct {
	struct list_head node;
	/*! Virtual address */
	void *virt;
	/*! Physical address */
	unsigned long phys;
	int size;
	bool in_iram;
} virt_phys_struct;

static struct list_head alloc_list;

/*!
 * Defines the size of each buffer in SDMA pool.
 * The size must be at least 512 bytes, because
 * sdma channel control blocks array size is 512 bytes
 */
#define SDMA_POOL_SIZE 1024

#ifdef CONFIG_SDMA_IRAM
static unsigned long iram_paddr;
static void *iram_vaddr;
#define iram_phys_to_virt(p) (iram_vaddr + ((p) - iram_paddr))
#define iram_virt_to_phys(v) (iram_paddr + ((v) - iram_vaddr))
#endif

/*!
 * Virtual to physical address conversion functio
 *
 * @param   buf  pointer to virtual address
 *
 * @return       physical address
 */
unsigned long sdma_virt_to_phys(void *buf)
{
	u32 offset = (u32) buf & (~PAGE_MASK);
	virt_phys_struct *p;

	DPRINTK("searching for vaddr 0x%p\n", buf);

	list_for_each_entry(p, &alloc_list, node) {
		if (((u32)p->virt & PAGE_MASK) == ((u32) buf & PAGE_MASK)) {
			return (p->phys & PAGE_MASK) | offset;
		}
	}

	if (virt_addr_valid(buf)) {
		return virt_to_phys(buf);
	}

	printk(KERN_WARNING
	       "SDMA malloc: could not translate virt address 0x%p\n", buf);
	return 0;
}

/*!
 * Physical to virtual address conversion functio
 *
 * @param   buf  pointer to physical address
 *
 * @return       virtual address
 */
void *sdma_phys_to_virt(unsigned long buf)
{
	u32 offset = buf & (~PAGE_MASK);
	virt_phys_struct *p;

	DPRINTK("searching for paddr 0x%p\n", buf);

	list_for_each_entry(p, &alloc_list, node) {
		if ((p->phys & PAGE_MASK) == (buf & PAGE_MASK)) {
			return (void *)(((u32)p->virt & PAGE_MASK) | offset);
		}
	}

	printk(KERN_WARNING
	       "SDMA malloc: could not translate phys address 0x%lx\n", buf);
	return 0;
}

/*!
 * Allocates uncacheable buffer
 *
 * @param   size    size of allocated buffer
 * @return  pointer to buffer
 */
void *sdma_malloc(size_t size)
{
	void *buf;
	dma_addr_t dma_addr;
	virt_phys_struct *p;

	if (size > SDMA_POOL_SIZE) {
		printk(KERN_WARNING
		       "size in sdma_malloc is more than %d bytes\n",
		       SDMA_POOL_SIZE);
		return 0;
	}

	buf = dma_pool_alloc(pool, GFP_KERNEL, &dma_addr);
	if (buf == 0)
		return 0;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	p->virt = buf;
	p->phys = dma_addr;
	list_add_tail(&p->node, &alloc_list);

	DPRINTK("allocated vaddr 0x%p\n", buf);
	return buf;
}

/*!
 * Frees uncacheable buffer
 *
 * @param  buf    buffer pointer for deletion
 */
void sdma_free(void *buf)
{
	virt_phys_struct *p;

	list_for_each_entry(p, &alloc_list, node) {
		if (p->virt == buf) {
			if (p->in_iram)
				gen_pool_free(sdma_iram_pool, p->phys, p->size);
			else
				dma_pool_free(pool, p->virt, p->phys);
			list_del(&p->node);
			kfree(p);
			return;
		}
	}
}

#ifdef CONFIG_SDMA_IRAM
/*!
 * Allocates uncacheable buffer from IRAM
 */
void *sdma_iram_malloc(size_t size)
{
	virt_phys_struct *p = kzalloc(sizeof(*p), GFP_KERNEL);
	unsigned long buf;

	buf = gen_pool_alloc(sdma_iram_pool, size);
	if (!buf) {
		kfree(p);
		return NULL;
	}

	p->virt = iram_vaddr + (buf - iram_paddr);
	p->phys = buf;
	p->size = size;
	p->in_iram = true;
	list_add_tail(&p->node, &alloc_list);
	return p->virt;
}
#endif				/*CONFIG_SDMA_IRAM */

/*!
 * SDMA buffers pool initialization function
 */
void __init init_sdma_pool(void)
{
	pool = dma_pool_create("SDMA", NULL, SDMA_POOL_SIZE, 0, 0);

#ifdef CONFIG_SDMA_IRAM
	iram_vaddr = iram_alloc(SZ_4K, &iram_paddr);
	sdma_iram_pool = gen_pool_create(6, -1);
	gen_pool_add(sdma_iram_pool, iram_paddr, SZ_4K, -1);
#endif

	INIT_LIST_HEAD(&alloc_list);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Linux SDMA API");
MODULE_LICENSE("GPL");
