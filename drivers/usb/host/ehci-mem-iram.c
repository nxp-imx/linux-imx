/*
 * Copyright (c) 2001 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* this file is part of ehci-hcd.c */

/*-------------------------------------------------------------------------*/

/*
 * There's basically three types of memory:
 *	- data used only by the HCD ... kmalloc is fine
 *	- async and periodic schedules, shared by HC and HCD ... these
 *	  need to use dma_pool or dma_alloc_coherent
 *	- driver buffers, read/written by HC ... single shot DMA mapped
 *
 * There's also "register" data (e.g. PCI or SOC), which is memory mapped.
 * No memory seen by this driver is pageable.
 */

/*-------------------------------------------------------------------------*/

/* Allocate the key transfer structures from the previously allocated pool */
#include <linux/smp_lock.h>
#include <linux/iram_alloc.h>

bool use_iram_qtd;

struct memDesc {
	u32 start;
	u32 end;
	struct memDesc *next;
} ;

static u32 g_usb_pool_start;
static s32 g_usb_pool_count;
static u32 g_total_pages;
static u32 g_alignment = 32;
struct memDesc *g_allocated_desc;
static spinlock_t g_usb_sema;
static u32 g_debug_qtd_allocated;
static u32 g_debug_qH_allocated;
static int g_alloc_map;
static unsigned long g_iram_base;
static __iomem void *g_iram_addr;

/*!
 * usb_pool_initialize
 *
 * @param       memPool         start address of the pool
 * @param       poolSize        memory pool size
 * @param       alignment       alignment for example page alignmnet will be 4K
 *
 * @return      0 for success  -1 for errors
 */
static int usb_pool_initialize(u32 memPool, u32 poolSize, u32 alignment)
{
	if (g_usb_pool_count) {
		printk(KERN_INFO "usb_pool_initialize : already initialzed.\n");
		return 0;
	}

	g_alignment = alignment;
	if (g_alignment == 0) {
		printk(KERN_INFO
		       "usb_pool_initialize : g_alignment can not be zero.\n");
		g_alignment = 32;
	}

	g_total_pages = poolSize / g_alignment;
	g_usb_pool_start = (u32) memPool;

	g_allocated_desc = kmalloc(sizeof(struct memDesc), GFP_KERNEL);
	if (!g_allocated_desc) {
		printk(KERN_ALERT "usb_pool_initialize : kmalloc failed \n");
		return (-1);
	}

	g_allocated_desc->start = 0;
	g_allocated_desc->end = 0;
	g_allocated_desc->next = NULL;

	spin_lock_init(&g_usb_sema);
	g_usb_pool_count++;
	return (0);
}

static void usb_pool_deinit()
{
	if (--g_usb_pool_count < 0)
		g_usb_pool_count = 0;
}

/*!
 * usb_malloc
 *
 * @param       size        memory pool size
 *
 * @return      physical address, 0 for error
 */
static u32 usb_malloc(u32 size, gfp_t mem_flags)
{
	unsigned long flags;
	struct memDesc *prevDesc = NULL;
	struct memDesc *nextDesc = NULL;
	struct memDesc *currentDesc = NULL;
	u32 pages = (size + g_alignment - 1) / g_alignment;

	if ((size == 0) || (pages > g_total_pages))
		return 0;

	currentDesc = kmalloc(sizeof(struct memDesc), mem_flags);
	if (!currentDesc) {
		printk(KERN_ALERT "usb_malloc: kmalloc failed \n");
		return 0;
	}

	spin_lock_irqsave(&g_usb_sema, flags);

	/* Create the first Allocated descriptor */
	if (!g_allocated_desc->next) {
		g_allocated_desc->next = currentDesc;
		currentDesc->start = 0;
		currentDesc->end = pages;
		currentDesc->next = NULL;
		spin_unlock_irqrestore(&g_usb_sema, flags);
		return (g_usb_pool_start + currentDesc->start * g_alignment);
	}

	/* Find the free spot */
	prevDesc = g_allocated_desc;
	while (prevDesc->next) {
		nextDesc = prevDesc->next;
		if (pages <= nextDesc->start - prevDesc->end) {
			currentDesc->start = prevDesc->end;
			currentDesc->end = currentDesc->start + pages;
			currentDesc->next = nextDesc;
			prevDesc->next = currentDesc;
			break;
		}
		prevDesc = nextDesc;
	}

	/* Do not find the free spot inside the chain, append to the end */
	if (!prevDesc->next) {
		if (pages > (g_total_pages - prevDesc->end)) {
			spin_unlock_irqrestore(&g_usb_sema, flags);
			kfree(currentDesc);
			return 0;
		} else {
			currentDesc->start = prevDesc->end;
			currentDesc->end = currentDesc->start + pages;
			currentDesc->next = NULL;
			prevDesc->next = currentDesc;
		}
	}

	spin_unlock_irqrestore(&g_usb_sema, flags);
	return (g_usb_pool_start + currentDesc->start * g_alignment);
}

/*!
 * usb_free
 *
 * @param       physical        physical address try to free
 *
 */
static void usb_free(u32 physical)
{
	unsigned long flags;
	struct memDesc *prevDesc = NULL;
	struct memDesc *nextDesc = NULL;
	u32 pages = (physical - g_usb_pool_start) / g_alignment;

	/* Protect the memory pool data structures. */
	spin_lock_irqsave(&g_usb_sema, flags);

	prevDesc = g_allocated_desc;
	while (prevDesc->next) {
		nextDesc = prevDesc->next;
		if (nextDesc->start == pages) {
			prevDesc->next = nextDesc->next;
			kfree(nextDesc);
			break;
		}
		prevDesc = prevDesc->next;
	}
	/* All done with memory pool data structures. */
	spin_unlock_irqrestore(&g_usb_sema, flags);
}

static int address_to_buffer(struct ehci_hcd *ehci, int address)
{
	int i;

	for (i = 0; i < IRAM_NTD; i++) {
		if (ehci->usb_address[i] == address)
			return i;
	}
	return IRAM_NTD;
}

static void use_buffer(struct ehci_hcd *ehci, int address)
{
	int i;

	for (i = 0; i < IRAM_NTD; i++) {
		if (ehci->usb_address[i] == address)
			return;
	}

	if (ehci->usb_address[0] == 0) {
		ehci->usb_address[0] = address;
		printk(KERN_INFO "usb_address[0] %x\n", address);
		return;
	} else if (ehci->usb_address[1] == 0) {
		ehci->usb_address[1] = address;
		printk(KERN_INFO "usb_address[1] %x\n", address);
		return;
	} else
		printk(KERN_ALERT "qh_make run out of iRAM, already be used");
}

static u32 alloc_iram_buf(void)
{
	int i;

	for (i = 0; i < IRAM_NTD; i++) {
		if (!(g_alloc_map & (1 << i))) {
			g_alloc_map |= (1 << i);
			return g_iram_base + i * (IRAM_TD_SIZE * 2);
		}
	}
	panic("Out of IRAM buffers\n");
}

void free_iram_buf(u32 buf)
{
	int i = (buf - g_iram_base) / (IRAM_TD_SIZE * 2);

	g_alloc_map &= ~(1 << i);
}

static inline void ehci_qtd_init(struct ehci_hcd *ehci, struct ehci_qtd *qtd,
				 dma_addr_t dma)
{
	memset(qtd, 0, sizeof *qtd);
	qtd->qtd_dma = dma;
	qtd->hw_token = cpu_to_le32(QTD_STS_HALT);
	qtd->hw_next = EHCI_LIST_END(ehci);
	qtd->hw_alt_next = EHCI_LIST_END(ehci);
	INIT_LIST_HEAD(&qtd->qtd_list);
}

static struct ehci_qtd *ehci_qtd_alloc(struct ehci_hcd *ehci, gfp_t flags)
{
	struct ehci_qtd *qtd;
	dma_addr_t dma;

	if (use_iram_qtd) {
		dma = usb_malloc(sizeof(struct ehci_qtd), flags);
		if (dma != 0)
			qtd = (struct ehci_qtd *)(g_iram_addr + (dma - g_iram_base));
		else
			qtd = dma_pool_alloc(ehci->qtd_pool, flags, &dma);
	}
	else
		qtd = dma_pool_alloc(ehci->qtd_pool, flags, &dma);

	if (qtd != NULL) {
		ehci_qtd_init(ehci, qtd, dma);
		++g_debug_qtd_allocated;
	} else {
		panic
		    ("out of i-ram for qtd allocation g_debug_qtd_allocated %d \
			size%d \n", g_debug_qtd_allocated,
			sizeof(struct ehci_qtd));
	}
	return qtd;
}

static inline void ehci_qtd_free(struct ehci_hcd *ehci, struct ehci_qtd *qtd)
{
	if ((qtd->qtd_dma & (g_iram_base & 0xFFF00000)) ==
	    (g_iram_base & 0xFFF00000))
		usb_free(qtd->qtd_dma);
	else
		dma_pool_free(ehci->qtd_pool, qtd, qtd->qtd_dma);
	--g_debug_qtd_allocated;
}

static void qh_destroy(struct ehci_qh *qh)
{
	struct ehci_hcd *ehci = qh->ehci;

	/* clean qtds first, and know this is not linked */
	if (!list_empty(&qh->qtd_list) || qh->qh_next.ptr) {
		ehci_dbg(ehci, "unused qh not empty!\n");
		BUG();
	}
	if (qh->dummy)
		ehci_qtd_free(ehci, qh->dummy);
	int i;
	for (i = 0; i < IRAM_NTD; i++) {
		if (ehci->usb_address[i] == (qh->hw_info1 & 0x7F))
			ehci->usb_address[i] = 0;
	}

	if ((qh->qh_dma & (g_iram_base & 0xFFF00000)) ==
	    (g_iram_base & 0xFFF00000))
		usb_free(qh->qh_dma);
	else
		dma_pool_free(ehci->qh_pool, qh, qh->qh_dma);
	--g_debug_qH_allocated;
}

static struct ehci_qh *ehci_qh_alloc(struct ehci_hcd *ehci, gfp_t flags)
{
	struct ehci_qh *qh;
	dma_addr_t dma;

	dma = usb_malloc(sizeof(struct ehci_qh), flags);
	if (dma != 0)
		qh = (struct ehci_qh *)(g_iram_addr + (dma - g_iram_base));
	else
		qh = (struct ehci_qh *)
		    dma_pool_alloc(ehci->qh_pool, flags, &dma);
	++g_debug_qH_allocated;
	if (qh == NULL) {
		panic("run out of i-ram for qH allocation\n");
		return qh;
	}

	memset(qh, 0, sizeof *qh);
	qh->refcount = 1;
	qh->ehci = ehci;
	qh->qh_dma = dma;
	INIT_LIST_HEAD(&qh->qtd_list);

	/* dummy td enables safe urb queuing */
	qh->dummy = ehci_qtd_alloc(ehci, flags);
	if (qh->dummy == NULL) {
		ehci_dbg(ehci, "no dummy td\n");
		dma_pool_free(ehci->qh_pool, qh, qh->qh_dma);
		qh = NULL;
	}
	return qh;
}

/* to share a qh (cpu threads, or hc) */
static inline struct ehci_qh *qh_get(struct ehci_qh *qh)
{
	WARN_ON(!qh->refcount);
	qh->refcount++;
	return qh;
}

static inline void qh_put(struct ehci_qh *qh)
{
	if (!--qh->refcount)
		qh_destroy(qh);
}

/*-------------------------------------------------------------------------*/

/* The queue heads and transfer descriptors are managed from pools tied
 * to each of the "per device" structures.
 * This is the initialisation and cleanup code.
 */

static void ehci_mem_cleanup(struct ehci_hcd *ehci)
{
	if (ehci->async)
		qh_put(ehci->async);
	ehci->async = NULL;

	/* DMA consistent memory and pools */
	if (ehci->qtd_pool)
		dma_pool_destroy(ehci->qtd_pool);
	ehci->qtd_pool = NULL;

	if (ehci->qh_pool) {
		dma_pool_destroy(ehci->qh_pool);
		ehci->qh_pool = NULL;
	}

	if (ehci->itd_pool)
		dma_pool_destroy(ehci->itd_pool);
	ehci->itd_pool = NULL;

	if (ehci->sitd_pool)
		dma_pool_destroy(ehci->sitd_pool);
	ehci->sitd_pool = NULL;

	if (ehci->periodic)
		dma_free_coherent(ehci_to_hcd(ehci)->self.controller,
				  ehci->periodic_size * sizeof(u32),
				  ehci->periodic, ehci->periodic_dma);
	ehci->periodic = NULL;

	if (ehci->iram_buffer[0])
		free_iram_buf(ehci->iram_buffer[0]);
	if (ehci->iram_buffer[1])
		free_iram_buf(ehci->iram_buffer[1]);

	iounmap(g_iram_addr);
	iram_free(g_iram_base, USB_IRAM_SIZE);

	/* shadow periodic table */
	kfree(ehci->pshadow);
	ehci->pshadow = NULL;
	usb_pool_deinit();
}

/* remember to add cleanup code (above) if you add anything here */
static int ehci_mem_init(struct ehci_hcd *ehci, gfp_t flags)
{
	int i;
	g_usb_pool_count = 0;
	g_debug_qtd_allocated = 0;
	g_debug_qH_allocated = 0;
	g_alloc_map = 0;

	if (cpu_is_mx37())
		use_iram_qtd = 0;
	else
		use_iram_qtd = 1;

	g_iram_addr = iram_alloc(USB_IRAM_SIZE, &g_iram_base);

	usb_pool_initialize(g_iram_base + IRAM_TD_SIZE * IRAM_NTD * 2,
			    USB_IRAM_SIZE - IRAM_TD_SIZE * IRAM_NTD * 2, 32);

	if (!ehci->iram_buffer[0]) {
		ehci->iram_buffer[0] = alloc_iram_buf();
		ehci->iram_buffer_v[0] = g_iram_addr + (ehci->iram_buffer[0] - g_iram_base);
		ehci->iram_buffer[1] = alloc_iram_buf();
		ehci->iram_buffer_v[1] = g_iram_addr + (ehci->iram_buffer[1] - g_iram_base);
	}

	/* QTDs for control/bulk/intr transfers */
	ehci->qtd_pool = dma_pool_create("ehci_qtd",
					ehci_to_hcd(ehci)->self.controller,
					sizeof(struct ehci_qtd),
					32/* byte alignment (for hw parts) */
					 , 4096 /* can't cross 4K */);
	if (!ehci->qtd_pool)
		goto fail;

	/* QHs for control/bulk/intr transfers */
	ehci->qh_pool = dma_pool_create("ehci_qh",
					ehci_to_hcd(ehci)->self.controller,
					sizeof(struct ehci_qh),
					32 /* byte alignment (for hw parts) */ ,
					4096 /* can't cross 4K */);
	if (!ehci->qh_pool)
		goto fail;

	ehci->async = ehci_qh_alloc(ehci, flags);
	if (!ehci->async)
		goto fail;

	/* ITD for high speed ISO transfers */
	ehci->itd_pool = dma_pool_create("ehci_itd",
					ehci_to_hcd(ehci)->self.controller,
					sizeof(struct ehci_itd),
					32/* byte alignment (for hw parts) */
					 , 4096 /* can't cross 4K */);
	if (!ehci->itd_pool)
		goto fail;

	/* SITD for full/low speed split ISO transfers */
	ehci->sitd_pool = dma_pool_create("ehci_sitd",
					ehci_to_hcd(ehci)->self.controller,
					sizeof(struct ehci_sitd),
					32/* byte alignment (for hw parts) */
					  , 4096 /* can't cross 4K */);
	if (!ehci->sitd_pool)
		goto fail;

	ehci->periodic = (__le32 *)
	    dma_alloc_coherent(ehci_to_hcd(ehci)->self.controller,
			       ehci->periodic_size * sizeof(__le32),
			       &ehci->periodic_dma, 0);

	if (ehci->periodic == NULL)
		goto fail;

	for (i = 0; i < ehci->periodic_size; i++)
		ehci->periodic[i] = EHCI_LIST_END(ehci);

	/* software shadow of hardware table */
	ehci->pshadow = kcalloc(ehci->periodic_size, sizeof(void *), flags);
	if (ehci->pshadow != NULL)
		return 0;

fail:
	ehci_dbg(ehci, "couldn't init memory\n");
	ehci_mem_cleanup(ehci);
	return -ENOMEM;
}
