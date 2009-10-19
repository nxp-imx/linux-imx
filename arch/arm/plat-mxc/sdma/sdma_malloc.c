/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <asm/dma.h>
#include <mach/hardware.h>

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#define DEBUG 0

#if DEBUG
#define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#ifdef CONFIG_SDMA_IRAM
#define IRAM_VIRT_BASE  IRAM_BASE_ADDR_VIRT
#define IRAM_PHYS_BASE  IRAM_BASE_ADDR
#if (CONFIG_SDMA_IRAM_SIZE&0x3FF)
#error  "IRAM size of SDMA should be multiple of 1Kbytes"
#else
#define IRAM_SDMA_SIZE  CONFIG_SDMA_IRAM_SIZE	/* 4K */
#endif
#define IRAM_UNIT_SIZE  512
#define IRAM_POOL_SIZE  (IRAM_SDMA_SIZE/IRAM_UNIT_SIZE)

#define IS_IRAM_VIRT(x) (((x)<IRAM_VIRT_BASE)?0:\
                                (((x) - IRAM_VIRT_BASE)>IRAM_SDMA_SIZE)?0:1)

#define IS_IRAM_PHYS(x) (((x)<IRAM_PHYS_BASE)?0:\
                                (((x) - IRAM_PHYS_BASE)>IRAM_SDMA_SIZE)?0:1)
#endif				/*CONFIG_SDMA_IRAM */

/*!
 * Defines SDMA non-cacheable buffers pool
 */
static struct dma_pool *pool;

#ifdef CONFIG_SDMA_IRAM
typedef struct iram_head_s {
	struct list_head list;
} iram_head_t;

static spinlock_t iram_pool_lock = SPIN_LOCK_UNLOCKED;
static struct list_head iram_free_list;
static unsigned char iram_pool_flag[IRAM_POOL_SIZE];

static void sdma_iram_free(void *buf);
#endif				/*CONFIG_SDMA_IRAM */

/*!
 * SDMA memory conversion hashing structure
 */
typedef struct {
	struct list_head node;
	int use_count;
	/*! Virtual address */
	void *virt;
	/*! Physical address */
	unsigned long phys;
} virt_phys_struct;

static struct list_head buf_map;

/*!
 * Defines the size of each buffer in SDMA pool.
 * The size must be at least 512 bytes, because
 * sdma channel control blocks array size is 512 bytes
 */
#define SDMA_POOL_SIZE 1024

/*!
 * Adds new buffer structure into conversion hash tables
 *
 * @param   vf   SDMA memory conversion hashing structure
 *
 * @return       1 on success, 0 on fail
 */
static int add_entry(virt_phys_struct * vf)
{
	virt_phys_struct *p;

	vf->phys &= PAGE_MASK;
	vf->virt = (void *)((u32) vf->virt & PAGE_MASK);

	list_for_each_entry(p, &buf_map, node) {
		if (p->virt == vf->virt) {
			p->use_count++;
			return 0;
		}
	}

	p = kmalloc(sizeof(virt_phys_struct), GFP_KERNEL);
	if (p == 0) {
		return -ENOMEM;
	}

	*p = *vf;
	p->use_count = 1;
	list_add_tail(&p->node, &buf_map);

	DPRINTK("added vaddr 0x%p, paddr 0x%08X to list\n", p->virt, p->phys);

	return 0;
}

/*!
 * Deletes buffer stracture from conversion hash tables
 *
 * @param   buf   SDMA memory buffer virtual addr
 *
 * @return       0 on success, -1 on fail
 */
static int delete_entry(void *buf)
{
	virt_phys_struct *p;

	buf = (void *)((u32) buf & PAGE_MASK);

	list_for_each_entry(p, &buf_map, node) {
		if (p->virt == buf) {
			p->use_count--;
			break;
		}
	}

	if (p->use_count == 0) {
		list_del(&p->node);
		kfree(p);
	}

	return 0;
}

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

#ifdef CONFIG_SDMA_IRAM
	if (IS_IRAM_VIRT((unsigned long)buf)) {
		if ((unsigned long)buf & (IRAM_UNIT_SIZE - 1)) {
			printk(KERN_WARNING "%s buffer offset = %ld\n",
			       __FUNCTION__, (unsigned long)buf);
		}
		return (unsigned long)buf + IRAM_PHYS_BASE - IRAM_VIRT_BASE;
	}
#endif				/*CONFIG_SDMA_IRAM */

	list_for_each_entry(p, &buf_map, node) {
		if ((u32) p->virt == ((u32) buf & PAGE_MASK)) {
			return p->phys | offset;
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

#ifdef CONFIG_SDMA_IRAM
	if (IS_IRAM_PHYS((unsigned long)buf)) {
		if (buf & (IRAM_UNIT_SIZE - 1)) {
			printk(KERN_WARNING "%s buffer offset = %ld\n",
			       __FUNCTION__, (unsigned long)buf);
		}
		return (void *)buf + IRAM_VIRT_BASE - IRAM_PHYS_BASE;
	}
#endif				/*CONFIG_SDMA_IRAM */

	list_for_each_entry(p, &buf_map, node) {
		if (p->phys == (buf & PAGE_MASK)) {
			return (void *)((u32) p->virt | offset);
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
	virt_phys_struct vf;

	if (size > SDMA_POOL_SIZE) {
		printk(KERN_WARNING
		       "size in sdma_malloc is more than %d bytes\n",
		       SDMA_POOL_SIZE);
		buf = 0;
	} else {
		buf = dma_pool_alloc(pool, GFP_KERNEL, &dma_addr);
		if (buf > 0) {
			vf.virt = buf;
			vf.phys = dma_addr;

			if (add_entry(&vf) < 0) {
				dma_pool_free(pool, buf, dma_addr);
				buf = 0;
			}
		}
	}

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
#ifdef CONFIG_SDMA_IRAM
	if (IS_IRAM_VIRT((unsigned long)buf)) {
		sdma_iram_free(buf);
		return;
	}
#endif				/*CONFIG_SDMA_IRAM */

	dma_pool_free(pool, buf, sdma_virt_to_phys(buf));
	delete_entry(buf);
}

#ifdef CONFIG_SDMA_IRAM
/*!
 * Allocates uncacheable buffer from IRAM
 */
void *sdma_iram_malloc(size_t size)
{
	void *buf = NULL;
	int index = -1;
	unsigned long flags;
	if (size > IRAM_UNIT_SIZE) {
		printk(KERN_WARNING
		       "size in sdma_iram_malloc is more than %d bytes\n",
		       IRAM_UNIT_SIZE);
	} else {
		spin_lock_irqsave(&iram_pool_lock, flags);
		if (!list_empty(&iram_free_list)) {
			buf =
			    list_entry(iram_free_list.next, iram_head_t, list);
			list_del(iram_free_list.next);
			index =
			    ((unsigned long)buf -
			     IRAM_VIRT_BASE) / IRAM_UNIT_SIZE;
			if (index < 0 || index >= IRAM_POOL_SIZE) {
				spin_unlock_irqrestore(&iram_pool_lock, flags);
				printk(KERN_ERR "The iram pool has crashed\n");
				return NULL;
			}
			if (iram_pool_flag[index]) {
				spin_unlock_irqrestore(&iram_pool_lock, flags);
				printk(KERN_WARNING
				       "iram block %d  already has been allocated \n",
				       index);
			}
			iram_pool_flag[index] = 1;
		}
		spin_unlock_irqrestore(&iram_pool_lock, flags);
		if ((unsigned long)buf & (IRAM_UNIT_SIZE - 1)) {
			printk(KERN_WARNING
			       "the start address is not align of %d, buffer offset %ld\n",
			       IRAM_UNIT_SIZE, (unsigned long)buf);

			buf = PTR_ALIGN(buf, IRAM_UNIT_SIZE);
		}
	}
	return buf;
}

/*!
 * Free uncacheable buffer into IRAM.
 */
static void sdma_iram_free(void *buf)
{
	iram_head_t *p;
	int index;
	unsigned long flags;
	/* The check of parameter will be done in sdma_free */
	index = ((unsigned long)buf - IRAM_VIRT_BASE) / IRAM_UNIT_SIZE;
	spin_lock_irqsave(&iram_pool_lock, flags);
	p = (iram_head_t *) ((unsigned long)buf & (~(IRAM_UNIT_SIZE - 1)));
	list_add_tail(&(p->list), &iram_free_list);
	if (iram_pool_flag[index]) {
		iram_pool_flag[index] = 0;
		spin_unlock_irqrestore(&iram_pool_lock, flags);
	} else {
		printk(KERN_WARNING
		       "Free %p which IRAM block %d is already freed\n", buf,
		       index);
		spin_unlock_irqrestore(&iram_pool_lock, flags);
	}
}

/*!
 * Initialized the free list of IRAM.
 */
static void iram_pool_init(void)
{
	int i;
	iram_head_t *p;
	memset(iram_pool_flag, 0, IRAM_POOL_SIZE);
	INIT_LIST_HEAD(&iram_free_list);
	for (i = 0; i < IRAM_POOL_SIZE; i++) {
		p = (iram_head_t *) (IRAM_VIRT_BASE + i * IRAM_UNIT_SIZE);
		list_add_tail(&(p->list), &iram_free_list);
	}
}
#endif				/*CONFIG_SDMA_IRAM */

/*!
 * SDMA buffers pool initialization function
 */
void __init init_sdma_pool(void)
{
#ifdef CONFIG_SDMA_IRAM
	iram_pool_init();
#endif				/*CONFIG_SDMA_IRAM */

	pool = dma_pool_create("SDMA", NULL, SDMA_POOL_SIZE, 0, 0);

	INIT_LIST_HEAD(&buf_map);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Linux SDMA API");
MODULE_LICENSE("GPL");
