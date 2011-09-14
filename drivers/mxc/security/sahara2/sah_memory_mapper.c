/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_memory_mapper.c
*
* @brief Re-creates SAHARA data structures in Kernel memory such that they are
*       suitable for DMA.  Provides support for kernel API.
*
* This file needs to be ported.
*
* The memory mapper gets a call at #sah_Init_Mem_Map() during driver
* initialization.
*
* The routine #sah_Copy_Descriptors() is used to bring descriptor chains from
* user memory down to kernel memory, relink using physical addresses, and make
* sure that all user data will be accessible by the Sahara DMA.
* #sah_Destroy_Descriptors() does the inverse.
*
* The #sah_Alloc_Block(), #sah_Free_Block(), and #sah_Block_Add_Page() routines
* implement a cache of free blocks used when allocating descriptors and links
* within the kernel.
*
* The memory mapper gets a call at #sah_Stop_Mem_Map() during driver shutdown.
*
*/

#include <sah_driver_common.h>
#include <sah_kernel.h>
#include <sah_queue_manager.h>
#include <sah_memory_mapper.h>
#ifdef FSL_HAVE_SCC2
#include <linux/mxc_scc2_driver.h>
#else
#include <linux/mxc_scc_driver.h>
#endif

#if defined(DIAG_DRV_IF) || defined(DIAG_MEM) || defined(DO_DBG)
#include <diagnostic.h>
#include <sah_hardware_interface.h>
#endif

#include <linux/mm.h>		/* get_user_pages() */
#include <linux/pagemap.h>
#include <linux/dmapool.h>

#include <linux/slab.h>
#include <linux/highmem.h>

#if defined(DIAG_MEM) || defined(DIAG_DRV_IF)
#define DIAG_MSG_SIZE   1024
static char Diag_msg[DIAG_MSG_SIZE];
#endif

#ifdef LINUX_VERSION_CODE
#define FLUSH_SPECIFIC_DATA_ONLY
#else
#define SELF_MANAGED_POOL
#endif

#if defined(LINUX_VERSION_CODE)
EXPORT_SYMBOL(sah_Alloc_Link);
EXPORT_SYMBOL(sah_Free_Link);
EXPORT_SYMBOL(sah_Alloc_Descriptor);
EXPORT_SYMBOL(sah_Free_Descriptor);
EXPORT_SYMBOL(sah_Alloc_Head_Descriptor);
EXPORT_SYMBOL(sah_Free_Head_Descriptor);
EXPORT_SYMBOL(sah_Physicalise_Descriptors);
EXPORT_SYMBOL(sah_DePhysicalise_Descriptors);
#endif

/* Determine if L2 cache support should be built in. */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21))
#ifdef CONFIG_OUTER_CACHE
#define HAS_L2_CACHE
#endif
#else
#ifdef CONFIG_CPU_CACHE_L210
#define HAS_L2_CACHE
#endif
#endif

/* Number of bytes the hardware uses out of sah_Link and sah_*Desc structs */
#define SAH_HW_LINK_LEN  1
#define SAH_HW_DESC_LEN  24

/* Macros for Descriptors */
#define SAH_LLO_BIT 0x01000000
#define sah_Desc_Get_LLO(desc)          (desc->header & SAH_LLO_BIT)
#define sah_Desc_Set_Header(desc, h)    (desc->header = (h))

#define sah_Desc_Get_Next(desc)     (desc->next)
#define sah_Desc_Set_Next(desc, n)  (desc->next = (n))

#define sah_Desc_Get_Ptr1(desc)     (desc->ptr1)
#define sah_Desc_Get_Ptr2(desc)     (desc->ptr2)
#define sah_Desc_Set_Ptr1(desc,p1)  (desc->ptr1 = (p1))
#define sah_Desc_Set_Ptr2(desc,p2)  (desc->ptr2 = (p2))

#define sah_Desc_Get_Len1(desc)     (desc->len1)
#define sah_Desc_Get_Len2(desc)     (desc->len2)
#define sah_Desc_Set_Len1(desc,l1)  (desc->len1 = (l1))
#define sah_Desc_Set_Len2(desc,l2)  (desc->len2 = (l2))

/* Macros for Links */
#define sah_Link_Get_Next(link)     (link->next)
#define sah_Link_Set_Next(link, n)  (link->next = (n))

#define sah_Link_Get_Data(link)     (link->data)
#define sah_Link_Set_Data(link,d)   (link->data = (d))

#define sah_Link_Get_Len(link)      (link->len)
#define sah_Link_Set_Len(link, l)   (link->len = (l))

#define sah_Link_Get_Flags(link)    (link->flags)

/* Memory block defines */
/* Warning!  This assumes that kernel version of sah_Link
 * is larger than kernel version of sah_Desc.
 */
#define MEM_BLOCK_SIZE sizeof(sah_Link)

/*! Structure for  link/descriptor memory blocks in internal pool */
typedef struct mem_block {
	uint8_t data[MEM_BLOCK_SIZE];	/*!< the actual buffer area  */
	struct mem_block *next;	/*!< next block when in free chain */
	dma_addr_t dma_addr;	/*!< physical address of @a data  */
} Mem_Block;

#define MEM_BLOCK_ENTRIES  (PAGE_SIZE / sizeof(Mem_Block))

#define MEM_BIG_BLOCK_SIZE     sizeof(sah_Head_Desc)

/*! Structure for head descriptor memory blocks in internal pool */
typedef struct mem_big_block {
	uint8_t data[MEM_BIG_BLOCK_SIZE];	/*!< the actual buffer area  */
	struct mem_big_block *next;	/*!< next block when in free chain */
	uint32_t dma_addr;	/*!< physical address of @a data  */
} Mem_Big_Block;

#define MEM_BIG_BLOCK_ENTRIES  (PAGE_SIZE / sizeof(Mem_Big_Block))

/* Shared variables */

/*!
 * Lock to protect the memory chain composed of #block_free_head and
 * #block_free_tail.
 */
static os_lock_t mem_lock;

#ifndef SELF_MANAGED_POOL
static struct dma_pool *big_dma_pool = NULL;
static struct dma_pool *small_dma_pool = NULL;
#endif

#ifdef SELF_MANAGED_POOL
/*!
 * Memory block free pool - pointer to first block.  Chain is protected by
 * #mem_lock.
 */
static Mem_Block *block_free_head = NULL;
/*!
 * Memory block free pool - pointer to last block.  Chain is protected by
 * #mem_lock.
 */
static Mem_Block *block_free_tail = NULL;
/*!
 * Memory block free pool - pointer to first block.  Chain is protected by
 * #mem_lock.
 */
static Mem_Big_Block *big_block_free_head = NULL;
/*!
 * Memory block free pool - pointer to last block.  Chain is protected by
 * #mem_lock.
a */
static Mem_Big_Block *big_block_free_tail = NULL;
#endif				/* SELF_MANAGED_POOL */

static Mem_Block *sah_Alloc_Block(void);
static void sah_Free_Block(Mem_Block * block);
static Mem_Big_Block *sah_Alloc_Big_Block(void);
static void sah_Free_Big_Block(Mem_Big_Block * block);
#ifdef SELF_MANAGED_POOL
static void sah_Append_Block(Mem_Block * block);
static void sah_Append_Big_Block(Mem_Big_Block * block);
#endif				/* SELF_MANAGED_POOL */

/* Page context structure.  Used by wire_user_memory and unwire_user_memory */
typedef struct page_ctx_t {
	uint32_t count;
	struct page **local_pages;
} page_ctx_t;

/*!
*******************************************************************************
* Map and wire down a region of user memory.
*
*
* @param    address     Userspace address of the memory to wire
* @param    length      Length of the memory region to wire
* @param    page_ctx    Page context, to be passed to unwire_user_memory
*
* @return   (if successful) Kernel virtual address of the wired pages
*/
void *wire_user_memory(void *address, uint32_t length, void **page_ctx)
{
	void *kernel_black_addr = NULL;
	int result = -1;
	int page_index = 0;
	page_ctx_t *page_context;
	int nr_pages = 0;
	unsigned long start_page;
	fsl_shw_return_t status;

	/* Determine the number of pages being used for this link */
	nr_pages = (((unsigned long)(address) & ~PAGE_MASK)
		    + length + ~PAGE_MASK) >> PAGE_SHIFT;

	start_page = (unsigned long)(address) & PAGE_MASK;

	/* Allocate some memory to keep track of the wired user pages, so that
	 * they can be deallocated later.  The block of memory will contain both
	 * the structure and the array of pages.
	 */
	page_context = kmalloc(sizeof(page_ctx_t)
			       + nr_pages * sizeof(struct page *), GFP_KERNEL);

	if (page_context == NULL) {
		status = FSL_RETURN_NO_RESOURCE_S;	/* no memory! */
#ifdef DIAG_DRV_IF
		LOG_KDIAG("kmalloc() failed.");
#endif
		return NULL;
	}

	/* Set the page pointer to point to the allocated region of memory */
	page_context->local_pages = (void *)page_context + sizeof(page_ctx_t);

#ifdef DIAG_DRV_IF
	LOG_KDIAG_ARGS("page_context at: %p, local_pages at: %p",
		       (void *)page_context,
		       (void *)(page_context->local_pages));
#endif

	/* Wire down the pages from user space */
	down_read(&current->mm->mmap_sem);
	result = get_user_pages(current, current->mm,
				start_page, nr_pages, WRITE, 0 /* noforce */ ,
				(page_context->local_pages), NULL);
	up_read(&current->mm->mmap_sem);

	if (result < nr_pages) {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("get_user_pages() failed.");
#endif
		if (result > 0) {
			for (page_index = 0; page_index < result; page_index++) {
				page_cache_release((page_context->
						    local_pages[page_index]));
			}

			kfree(page_context);
		}
		return NULL;
	}

	kernel_black_addr = page_address(page_context->local_pages[0]) +
	    ((unsigned long)address & ~PAGE_MASK);

	page_context->count = nr_pages;
	*page_ctx = page_context;

	return kernel_black_addr;
}

/*!
*******************************************************************************
* Release and unmap a region of user memory.
*
* @param    page_ctx    Page context from wire_user_memory
*/
void unwire_user_memory(void **page_ctx)
{
	int page_index = 0;
	struct page_ctx_t *page_context = *page_ctx;

#ifdef DIAG_DRV_IF
	LOG_KDIAG_ARGS("page_context at: %p, first page at:%p, count: %i",
		       (void *)page_context,
		       (void *)(page_context->local_pages),
		       page_context->count);
#endif

	if ((page_context != NULL) && (page_context->local_pages != NULL)) {
		for (page_index = 0; page_index < page_context->count;
		     page_index++) {
			page_cache_release(page_context->
					   local_pages[page_index]);
		}

		kfree(page_context);
		*page_ctx = NULL;
	}
}

/*!
*******************************************************************************
* Map some physical memory into a users memory space
*
* @param    vma             Memory structure to map to
* @param    physical_addr   Physical address of the memory to be mapped in
* @param    size            Size of the memory to map (bytes)
*
* @return
*/
os_error_code
map_user_memory(struct vm_area_struct *vma, uint32_t physical_addr,
		uint32_t size)
{
	os_error_code retval;

	/* Map the acquired partition into the user's memory space */
	vma->vm_end = vma->vm_start + size;

	/* set cache policy to uncached so that each write of the UMID and
	 * permissions get directly to the SCC2 in order to engage it
	 * properly.  Once the permissions have been written, it may be
	 * useful to provide a service for the user to request a different
	 * cache policy
	 */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Make sure that the user cannot fork() a child which will inherit
	 * this mapping, as it creates a security hole.  Likewise, do not
	 * allow the user to 'expand' his mapping beyond this partition.
	 */
	vma->vm_flags |= VM_IO | VM_RESERVED | VM_DONTCOPY | VM_DONTEXPAND;

	retval = remap_pfn_range(vma,
				 vma->vm_start,
				 __phys_to_pfn(physical_addr),
				 size, vma->vm_page_prot);

	return retval;
}

/*!
*******************************************************************************
* Remove some memory from a user's memory space
*
* @param    user_addr       Userspace address of the memory to be unmapped
* @param    size            Size of the memory to map (bytes)
*
* @return
*/
os_error_code unmap_user_memory(uint32_t user_addr, uint32_t size)
{
	os_error_code retval;
	struct mm_struct *mm = current->mm;

	/* Unmap the memory region (see sys_munmap in mmap.c) */
	down_write(&mm->mmap_sem);
	retval = do_munmap(mm, (unsigned long)user_addr, size);
	up_write(&mm->mmap_sem);

	return retval;
}

/*!
*******************************************************************************
* Free descriptor back to free pool
*
* @brief Free descriptor
*
* @param    desc        A descriptor allocated with sah_Alloc_Descriptor().
*
* @return none
*
*/
void sah_Free_Descriptor(sah_Desc * desc)
{
	memset(desc, 0x45, sizeof(*desc));
	sah_Free_Block((Mem_Block *) desc);
}

/*!
*******************************************************************************
* Free Head descriptor back to free pool
*
* @brief Free Head descriptor
*
* @param    desc  A Head descriptor allocated with sah_Alloc_Head_Descriptor().
*
* @return none
*
*/
void sah_Free_Head_Descriptor(sah_Head_Desc * desc)
{
	memset(desc, 0x43, sizeof(*desc));
	sah_Free_Big_Block((Mem_Big_Block *) desc);
}

/*!
*******************************************************************************
* Free link back to free pool
*
* @brief Free link
*
* @param    link        A link allocated with sah_Alloc_Link().
*
* @return none
*
*/
void sah_Free_Link(sah_Link * link)
{
	memset(link, 0x41, sizeof(*link));
	sah_Free_Block((Mem_Block *) link);
}

/*!
*******************************************************************************
* This function runs through a descriptor chain pointed to by a user-space
* address. It duplicates each descriptor in Kernel space memory and calls
* sah_Copy_Links() to handle any links attached to the descriptors. This
* function cleans-up everything that it created in the case of a failure.
*
* @brief Kernel Descriptor Chain Copier
*
* @param    fsl_shw_uco_t  The user context to act under
* @param    user_head_desc A Head Descriptor pointer from user-space.
*
* @return   sah_Head_Desc *  - A virtual address of the first descriptor in the
*                              chain.
* @return   NULL        - If there was some error.
*
*/
sah_Head_Desc *sah_Copy_Descriptors(fsl_shw_uco_t * user_ctx,
				sah_Head_Desc * user_head_desc)
{
	sah_Desc *curr_desc = NULL;
	sah_Desc *prev_desc = NULL;
	sah_Desc *next_desc = NULL;
	sah_Head_Desc *head_desc = NULL;
	sah_Desc *user_desc = NULL;
	unsigned long result;

	/* Internal status variable to be used in this function */
	fsl_shw_return_t status = FSL_RETURN_OK_S;
	sah_Head_Desc *ret_val = NULL;

	/* This will be set to True when we have finished processing our
	 * descriptor chain.
	 */
	int drv_if_done = FALSE;
	int is_this_the_head = TRUE;

	do {
		/* Allocate memory for this descriptor */
		if (is_this_the_head) {
			head_desc =
			    (sah_Head_Desc *) sah_Alloc_Head_Descriptor();

#ifdef DIAG_MEM
			sprintf(Diag_msg,
				"Alloc_Head_Descriptor returned %p\n",
				head_desc);
			LOG_KDIAG(Diag_msg);
#endif
			if (head_desc == NULL) {
#ifdef DIAG_DRV_IF
				LOG_KDIAG
				    ("sah_Alloc_Head_Descriptor() failed.");
#endif
				drv_if_done = TRUE;
				status = FSL_RETURN_NO_RESOURCE_S;
			} else {
				void *virt_addr = head_desc->desc.virt_addr;
				dma_addr_t dma_addr = head_desc->desc.dma_addr;

				/* Copy the head descriptor from user-space */
				/* Instead of copying the whole structure,
				 * unneeded bits at the end are left off.
				 * The user space version is missing virt/dma addrs, which
				 * means that the copy will be off for flags... */
				result = copy_from_user(head_desc,
							user_head_desc,
							(sizeof(*head_desc) -
							 sizeof(head_desc->desc.
								dma_addr) -
							 sizeof(head_desc->desc.
								virt_addr) -
							 sizeof(head_desc->desc.
								original_ptr1) -
/*                                sizeof(head_desc->desc.original_ptr2) -
                                sizeof(head_desc->status) -
                                sizeof(head_desc->error_status) -
                                sizeof(head_desc->fault_address) -
                                sizeof(head_desc->current_dar) -
                                sizeof(head_desc->result) -
                                sizeof(head_desc->next) -
                                sizeof(head_desc->prev) -
                                sizeof(head_desc->user_desc) -
*/ sizeof(head_desc->out1_ptr) -
							 sizeof(head_desc->
								out2_ptr) -
							 sizeof(head_desc->
								out_len)));
				/* there really isn't a 'next' descriptor at this point, so
				 * set that pointer to NULL, but remember it for if/when there
				 * is a next */
				next_desc = head_desc->desc.next;
				head_desc->desc.next = NULL;

				if (result != 0) {
#ifdef DIAG_DRV_IF
					LOG_KDIAG("copy_from_user() failed.");
#endif
					drv_if_done = TRUE;
					status = FSL_RETURN_INTERNAL_ERROR_S;
					/* when destroying the descriptor, skip these links.
					 * They've not been copied down, so don't exist */
					head_desc->desc.ptr1 = NULL;
					head_desc->desc.ptr2 = NULL;

				} else {
					/* The kernel DESC has five more words than user DESC, so
					 * the missing values are in the middle of the HEAD DESC,
					 * causing values after the missing ones to be at different
					 * offsets in kernel and user space.
					 *
					 * Patch up the problem by moving field two spots.
					 * This assumes sizeof(pointer) == sizeof(uint32_t).
					 * Note that 'user_info' is not needed, so not copied.
					 */
					head_desc->user_ref =
					    (uint32_t) head_desc->desc.dma_addr;
					head_desc->uco_flags =
					    (uint32_t) head_desc->desc.
					    original_ptr1;
#ifdef DIAG_DRV_IF
					LOG_KDIAG_ARGS(
						"User flags: %x; User Reference: %x",
						head_desc->uco_flags,
						head_desc->user_ref);
#endif
					/* These values were destroyed by the copy. */
					head_desc->desc.virt_addr = virt_addr;
					head_desc->desc.dma_addr = dma_addr;

					/* ensure that the save descriptor chain bit is not set.
					 * the copy of the user space descriptor chain should
					 * always be deleted */
					head_desc->uco_flags &=
					    ~FSL_UCO_SAVE_DESC_CHAIN;

					curr_desc = (sah_Desc *) head_desc;
					is_this_the_head = FALSE;
				}
			}
		} else {	/* not head */
			curr_desc = sah_Alloc_Descriptor();
#ifdef DIAG_MEM
			LOG_KDIAG_ARGS("Alloc_Descriptor returned %p\n",
				curr_desc);
#endif
			if (curr_desc == NULL) {
#ifdef DIAG_DRV_IF
				LOG_KDIAG("sah_Alloc_Descriptor() failed.");
#endif
				drv_if_done = TRUE;
				status = FSL_RETURN_NO_RESOURCE_S;
			} else {
				/* need to update the previous descriptors' next field to
				 * pointer to the current descriptor. */
				prev_desc->original_next = curr_desc;
				prev_desc->next =
				    (sah_Desc *) curr_desc->dma_addr;

				/* Copy the current descriptor from user-space */
				/* The virtual address and DMA address part of the sah_Desc
				 * struct are not copied to user space */
				result = copy_from_user(curr_desc, user_desc, (sizeof(sah_Desc) - sizeof(dma_addr_t) -	/* dma_addr */
									       sizeof(uint32_t) -	/* virt_addr */
									       sizeof(void *) -	/* original_ptr1 */
									       sizeof(void *) -	/* original_ptr2 */
									       sizeof(sah_Desc **)));	/* original_next */
				/* there really isn't a 'next' descriptor at this point, so
				 * set that pointer to NULL, but remember it for if/when there
				 * is a next */
				next_desc = curr_desc->next;
				curr_desc->next = NULL;
				curr_desc->original_next = NULL;

				if (result != 0) {
#ifdef DIAG_DRV_IF
					LOG_KDIAG("copy_from_user() failed.");
#endif
					drv_if_done = TRUE;
					status = FSL_RETURN_INTERNAL_ERROR_S;
					/* when destroying the descriptor chain, skip these links.
					 * They've not been copied down, so don't exist */
					curr_desc->ptr1 = NULL;
					curr_desc->ptr2 = NULL;
				}
			}
		}		/* end if (is_this_the_head) */

		if (status == FSL_RETURN_OK_S) {
			if (!(curr_desc->header & SAH_LLO_BIT)) {
				/* One or both pointer fields being NULL is a valid
				 * configuration. */
				if (curr_desc->ptr1 == NULL) {
					curr_desc->original_ptr1 = NULL;
				} else {
					/* pointer fields point to sah_Link structures */
					curr_desc->original_ptr1 =
					    sah_Copy_Links(user_ctx, curr_desc->ptr1);
					if (curr_desc->original_ptr1 == NULL) {
						/* This descriptor and any links created successfully
						 * are cleaned-up at the bottom of this function. */
						drv_if_done = TRUE;
						status =
						    FSL_RETURN_INTERNAL_ERROR_S;
						/* mark that link 2 doesn't exist */
						curr_desc->ptr2 = NULL;
#ifdef DIAG_DRV_IF
						LOG_KDIAG
						    ("sah_Copy_Links() failed.");
#endif
					} else {
						curr_desc->ptr1 = (void *)
						    ((sah_Link *) curr_desc->
						     original_ptr1)->dma_addr;
					}
				}

				if (status == FSL_RETURN_OK_S) {
					if (curr_desc->ptr2 == NULL) {
						curr_desc->original_ptr2 = NULL;
					} else {
						/* pointer fields point to sah_Link structures */
						curr_desc->original_ptr2 =
						    sah_Copy_Links(user_ctx, curr_desc->ptr2);
						if (curr_desc->original_ptr2 ==
						    NULL) {
							/* This descriptor and any links created
							 * successfully are cleaned-up at the bottom of
							 * this function. */
							drv_if_done = TRUE;
							status =
							    FSL_RETURN_INTERNAL_ERROR_S;
#ifdef DIAG_DRV_IF
							LOG_KDIAG
							    ("sah_Copy_Links() failed.");
#endif
						} else {
							curr_desc->ptr2 =
							    (void
							     *)(((sah_Link *)
								 curr_desc->
								 original_ptr2)
								->dma_addr);
						}
					}
				}
			} else {
				/* Pointer fields point directly to user buffers. We don't
				 * support this mode.
				 */
#ifdef DIAG_DRV_IF
				LOG_KDIAG
				    ("The LLO bit in the Descriptor Header field was "
				     "set. This an invalid configuration.");
#endif
				drv_if_done = TRUE;
				status = FSL_RETURN_INTERNAL_ERROR_S;
			}
		}

		if (status == FSL_RETURN_OK_S) {
			user_desc = next_desc;
			prev_desc = curr_desc;
			if (user_desc == NULL) {
				/* We have reached the end our our descriptor chain */
				drv_if_done = TRUE;
			}
		}

	} while (drv_if_done == FALSE);

	if (status != FSL_RETURN_OK_S) {
		/* Clean-up if failed */
		if (head_desc != NULL) {
#ifdef DIAG_DRV_IF
			LOG_KDIAG("Error!  Calling destroy descriptors!\n");
#endif
			sah_Destroy_Descriptors(head_desc);
		}
		ret_val = NULL;
	} else {
		/* Flush the caches */
#ifndef FLUSH_SPECIFIC_DATA_ONLY
		os_flush_cache_all();
#endif

		/* Success. Return the DMA'able head descriptor. */
		ret_val = head_desc;

	}

	return ret_val;
}				/* sah_Copy_Descriptors() */

/*!
*******************************************************************************
* This function runs through a sah_Link chain pointed to by a kernel-space
* address.  It computes the physical address for each pointer, and converts
* the chain to use these physical addresses.
*
******
*   This function needs to return some indication that the chain could not be
*   converted.  It also needs to back out any conversion already taken place on
*   this chain of links.
*
*   Then, of course, sah_Physicalise_Descriptors() will need to recognize that
*   an error occured, and then be able to back out any physicalization of the
*   chain which had taken place up to that point!
******
*
* @brief Convert kernel Link chain
*
* @param    first_link   A sah_Link pointer from kernel space; must not be
*                        NULL, so error case can be distinguished.
*
* @return   sah_Link *   A dma'able address of the first descriptor in the
*                         chain.
* @return   NULL         If Link chain could not be physicalised, i.e. ERROR
*
*/
sah_Link *sah_Physicalise_Links(sah_Link * first_link)
{
	sah_Link *link = first_link;
	struct page *pg;

	while (link != NULL) {
#ifdef DO_DBG
		sah_Dump_Words("Link", (unsigned *)link, link->dma_addr, 3);
#endif
		link->vm_info = NULL;

		/* need to retrieve stored key? */
		if (link->flags & SAH_STORED_KEY_INFO) {
			uint32_t max_len = 0;	/* max slot length */
			fsl_shw_return_t ret_status;

			/* get length and physical address of stored key */
			ret_status = system_keystore_get_slot_info(link->ownerid, link->slot, (uint32_t *) & link->data,	/* RED key address */
						       &max_len);
			if ((ret_status != FSL_RETURN_OK_S) || (link->len > max_len)) {
				/* trying to illegally/incorrectly access a key. Cause the
				 * error status register to show a Link Length Error by
				 * putting a zero in the links length. */
				link->len = 0;	/* Cause error.  Somebody is up to no good. */
			}
		} else if (link->flags & SAH_IN_USER_KEYSTORE) {

#ifdef FSL_HAVE_SCC2
			/* The data field points to the virtual address of the key.  Convert
			 * this to a physical address by modifying the address based
			 * on where the secure memory was mapped to the kernel.  Note: In
			 * kernel mode, no attempt is made to track or control who owns what
			 * memory partition.
			 */
			link->data = (uint8_t *) scc_virt_to_phys(link->data);

			/* Do bounds checking to ensure that the user is not overstepping
			 * the bounds of their partition.  This is a simple implementation
			 * that assumes the user only owns one partition.  It only checks
			 * to see if the address of the last byte of data steps over a
			 * page boundary.
			 */

#ifdef DO_DBG
			LOG_KDIAG_ARGS("start page: %08x, end page: %08x"
				       "first addr: %p, last addr: %p, len; %i",
				       ((uint32_t) (link->data) >> PAGE_SHIFT),
				       (((uint32_t) link->data +
					 link->len) >> PAGE_SHIFT), link->data,
				       link->data + link->len, link->len);
#endif

			if ((((uint32_t) link->data +
			      link->len) >> PAGE_SHIFT) !=
			    ((uint32_t) link->data >> PAGE_SHIFT)) {
				link->len = 0;	/* Cause error.  Somebody is up to no good. */
			}
#else				/* FSL_HAVE_SCC2 */

			/* User keystores are not valid on non-SCC2 platforms */
			link->len = 0;	/* Cause error.  Somebody is up to no good. */

#endif				/* FSL_HAVE_SCC2 */

		} else {
			if (!(link->flags & SAH_PREPHYS_DATA)) {
				link->original_data = link->data;

				/* if the data buffer is not in kernel direct-
				  * mapped region, find the physical addr
				  * via other means.
				  */
				if (!virt_addr_valid(link->data)) {
					pg = vmalloc_to_page(link->data);
					link->data = (uint8_t *) (
						page_to_phys(pg) +
						(((unsigned int) link->data) &
							~PAGE_MASK));
				} else
					link->data = (void *)os_pa(link->data);
#ifdef DO_DBG
				os_printk("%sput: %p (%d)\n",
					  (link->
					   flags & SAH_OUTPUT_LINK) ? "out" :
					  "in", link->data, link->len);
#endif

				if (link->flags & SAH_OUTPUT_LINK) {
					/* clean and invalidate */
					os_cache_flush_range(link->
							     original_data,
							     link->len);
				} else {
					os_cache_clean_range(link->original_data,
							     link->len);
				}
			}	/* not prephys */
		}		/* else not key reference */

#if defined(NO_OUTPUT_1K_CROSSING) || defined(NO_1K_CROSSING)
		if (
#ifdef NO_OUTPUT_1K_CROSSING
			   /* Insert extra link if 1k boundary on output pointer
			    * crossed not at an 8-word boundary */
			   (link->flags & SAH_OUTPUT_LINK) &&
			   (((uint32_t) link->data % 32) != 0) &&
#endif
			   ((((uint32_t) link->data & 1023) + link->len) >
			    1024)) {
			uint32_t full_length = link->len;
			sah_Link *new_link = sah_Alloc_Link();
			link->len = 1024 - ((uint32_t) link->data % 1024);
			new_link->len = full_length - link->len;
			new_link->data = link->data + link->len;
			new_link->original_data =
			    link->original_data + link->len;
			new_link->flags = link->flags & ~(SAH_OWNS_LINK_DATA);
			new_link->flags |= SAH_LINK_INSERTED_LINK;
			new_link->next = link->next;

			link->next = (sah_Link *) new_link->dma_addr;
			link->original_next = new_link;
			link = new_link;
		}
#endif				/* ALLOW_OUTPUT_1K_CROSSING */

		link->original_next = link->next;
		if (link->next != NULL) {
			link->next = (sah_Link *) link->next->dma_addr;
		}
#ifdef DO_DBG
		sah_Dump_Words("Link", link, link->dma_addr, 3);
#endif

		link = link->original_next;
	}

	return (sah_Link *) first_link->dma_addr;
}				/* sah_Physicalise_Links */

/*!
 * Run through descriptors and links created by KM-API and set the
 * dma addresses and 'do not free' flags.
 *
 * @param first_desc  KERNEL VIRTUAL address of first descriptor in chain.
 *
 * Warning!  This ONLY works without LLO flags in headers!!!
 *
 * @return Virtual address of @a first_desc.
 * @return NULL if Descriptor Chain could not be physicalised
 */
sah_Head_Desc *sah_Physicalise_Descriptors(sah_Head_Desc * first_desc)
{
	sah_Desc *desc = &first_desc->desc;

	if (!(first_desc->uco_flags & FSL_UCO_CHAIN_PREPHYSICALIZED)) {
		while (desc != NULL) {
			sah_Desc *next_desc;

#ifdef DO_DBG

			sah_Dump_Words("Desc", (unsigned *)desc, desc->dma_addr, 6);
#endif

			desc->original_ptr1 = desc->ptr1;
			if (desc->ptr1 != NULL) {
				if ((desc->ptr1 =
				     sah_Physicalise_Links(desc->ptr1)) ==
				    NULL) {
					/* Clean up ... */
					sah_DePhysicalise_Descriptors
					    (first_desc);
					first_desc = NULL;
					break;
				}
			}
			desc->original_ptr2 = desc->ptr2;
			if (desc->ptr2 != NULL) {
				if ((desc->ptr2 =
				     sah_Physicalise_Links(desc->ptr2)) ==
				    NULL) {
					/* Clean up ... */
					sah_DePhysicalise_Descriptors
					    (first_desc);
					first_desc = NULL;
					break;
				}
			}

			desc->original_next = desc->next;
			next_desc = desc->next;	/* save for bottom of while loop */
			if (desc->next != NULL) {
				desc->next = (sah_Desc *) desc->next->dma_addr;
			}

			desc = next_desc;
		}
	}
	/* not prephysicalized */
#ifdef DO_DBG
	os_printk("Physicalise finished\n");
#endif

	return first_desc;
}				/* sah_Physicalise_Descriptors() */

/*!
*******************************************************************************
* This function runs through a sah_Link chain pointed to by a physical address.
* It computes the virtual address for each pointer
*
* @brief Convert physical Link chain
*
* @param    first_link   A kernel address of a sah_Link
*
* @return   sah_Link *  A kernal address for the link chain of @c first_link
* @return   NULL        If there was some error.
*
* @post  All links will be chained together by original virtual addresses,
*        data pointers will point to virtual addresses.  Appropriate cache
*        lines will be flushed, memory unwired, etc.
*/
sah_Link *sah_DePhysicalise_Links(sah_Link * first_link)
{
	sah_Link *link = first_link;
	sah_Link *prev_link = NULL;

	/* Loop on virtual link pointer */
	while (link != NULL) {

#ifdef DO_DBG
		sah_Dump_Words("Link", (unsigned *)link, link->dma_addr, 3);
#endif

		/* if this references stored keys, don't want to dephysicalize them */
		if (!(link->flags & SAH_STORED_KEY_INFO)
		    && !(link->flags & SAH_PREPHYS_DATA)
		    && !(link->flags & SAH_IN_USER_KEYSTORE)) {

			/* */
			if (link->flags & SAH_OUTPUT_LINK) {
				os_cache_inv_range(link->original_data,
						   link->len);
			}

			/* determine if there is a page in user space associated with this
			 * link */
			if (link->vm_info != NULL) {
				/* check that this isn't reserved and contains output */
				if (!PageReserved(link->vm_info) &&
				    (link->flags & SAH_OUTPUT_LINK)) {

					/* Mark to force page eventually to backing store */
					SetPageDirty(link->vm_info);
				}

				/* Untie this page from physical memory */
				page_cache_release(link->vm_info);
			} else {
				/* kernel-mode data */
#ifdef DO_DBG
				os_printk("%sput: %p (%d)\n",
					  (link->
					   flags & SAH_OUTPUT_LINK) ? "out" :
					  "in", link->original_data, link->len);
#endif
			}
			link->data = link->original_data;
		}
#ifndef ALLOW_OUTPUT_1K_CROSSING
		if (link->flags & SAH_LINK_INSERTED_LINK) {
			/* Reconsolidate data by merging this link with previous */
			prev_link->len += link->len;
			prev_link->next = link->next;
			prev_link->original_next = link->original_next;
			sah_Free_Link(link);
			link = prev_link;

		}
#endif

		if (link->next != NULL) {
			link->next = link->original_next;
		}
		prev_link = link;
		link = link->next;
	}

	return first_link;
}				/* sah_DePhysicalise_Links() */

/*!
 * Run through descriptors and links that have been Physicalised
 * (sah_Physicalise_Descriptors function) and set the dma addresses back
 * to KM virtual addresses
 *
 * @param first_desc Kernel virtual address of first descriptor in chain.
 *
 * Warning!  This ONLY works without LLO flags in headers!!!
 */
sah_Head_Desc *sah_DePhysicalise_Descriptors(sah_Head_Desc * first_desc)
{
	sah_Desc *desc = &first_desc->desc;

	if (!(first_desc->uco_flags & FSL_UCO_CHAIN_PREPHYSICALIZED)) {
		while (desc != NULL) {
#ifdef DO_DBG
			sah_Dump_Words("Desc", (unsigned *)desc, desc->dma_addr, 6);
#endif

			if (desc->ptr1 != NULL) {
				desc->ptr1 =
				    sah_DePhysicalise_Links(desc->
							    original_ptr1);
			}
			if (desc->ptr2 != NULL) {
				desc->ptr2 =
				    sah_DePhysicalise_Links(desc->
							    original_ptr2);
			}
			if (desc->next != NULL) {
				desc->next = desc->original_next;
			}
			desc = desc->next;
		}
	}
	/* not prephysicalized */
	return first_desc;
}				/* sah_DePhysicalise_Descriptors() */

/*!
*******************************************************************************
* This walks through a SAHARA descriptor chain and free()'s everything
* that is not NULL. Finally it also unmaps all of the physical memory and
* frees the kiobuf_list Queue.
*
* @brief Kernel Descriptor Chain Destructor
*
* @param    head_desc   A Descriptor pointer from kernel-space.
*
* @return   void
*
*/
void sah_Free_Chained_Descriptors(sah_Head_Desc * head_desc)
{
	sah_Desc *desc = NULL;
	sah_Desc *next_desc = NULL;
	int this_is_head = 1;

	desc = &head_desc->desc;

	while (desc != NULL) {

		sah_Free_Chained_Links(desc->ptr1);
		sah_Free_Chained_Links(desc->ptr2);

		/* Get a bus pointer to the next Descriptor */
		next_desc = desc->next;

		/* Zero the header and Length fields for security reasons. */
		desc->header = 0;
		desc->len1 = 0;
		desc->len2 = 0;

		if (this_is_head) {
			sah_Free_Head_Descriptor(head_desc);
			this_is_head = 0;
#ifdef DIAG_MEM
			sprintf(Diag_msg, "Free_Head_Descriptor: %p\n",
				head_desc);
			LOG_KDIAG(Diag_msg);
#endif
		} else {
			/* free this descriptor */
			sah_Free_Descriptor(desc);
#ifdef DIAG_MEM
			sprintf(Diag_msg, "Free_Descriptor: %p\n", desc);
			LOG_KDIAG(Diag_msg);
#endif
		}

		/* Look at the next Descriptor */
		desc = next_desc;
	}
}				/* sah_Free_Chained_Descriptors() */

/*!
*******************************************************************************
* This walks through a SAHARA link chain and frees everything that is
* not NULL, excluding user-space buffers.
*
* @brief Kernel Link Chain Destructor
*
* @param    link    A Link pointer from kernel-space. This is in bus address
*                   space.
*
* @return   void
*
*/
void sah_Free_Chained_Links(sah_Link * link)
{
	sah_Link *next_link = NULL;

	while (link != NULL) {
		/* Get a bus pointer to the next Link */
		next_link = link->next;

		/* Zero some fields for security reasons. */
		link->data = NULL;
		link->len = 0;
		link->ownerid = 0;

		/* Free this Link */
#ifdef DIAG_MEM
		sprintf(Diag_msg, "Free_Link: %p(->%p)\n", link, link->next);
		LOG_KDIAG(Diag_msg);
#endif
		sah_Free_Link(link);

		/* Move on to the next Link */
		link = next_link;
	}
}

/*!
*******************************************************************************
* This function runs through a link chain pointed to by a user-space
* address. It makes a temporary kernel-space copy of each link in the
* chain and calls sah_Make_Links() to create a set of kernel-side links
* to replace it.
*
* @brief Kernel Link Chain Copier
*
* @param    ptr         A link pointer from user-space.
*
* @return   sah_Link *  - The virtual address of the first link in the
*                         chain.
* @return   NULL        - If there was some error.
*/
sah_Link *sah_Copy_Links(fsl_shw_uco_t * user_ctx, sah_Link * ptr)
{
	sah_Link *head_link = NULL;
	sah_Link *new_head_link = NULL;
	sah_Link *new_tail_link = NULL;
	sah_Link *prev_tail_link = NULL;
	sah_Link *user_link = ptr;
	sah_Link link_copy;
	int link_data_length = 0;

	/* Internal status variable to be used in this function */
	fsl_shw_return_t status = FSL_RETURN_OK_S;
	sah_Link *ret_val = NULL;

	/* This will be set to True when we have finished processing our
	 * link chain. */
	int drv_if_done = FALSE;
	int is_this_the_head = TRUE;
	int result;

	/* transfer all links, on this link chain, from user space */
	while (drv_if_done == FALSE) {
		/* Copy the current link from user-space. The virtual address, DMA
		 * address, and vm_info fields of the sah_Link struct are not part
		 * of the user-space structure. They must be the last elements and
		 * should not be copied. */
		result = copy_from_user(&link_copy,
					user_link, (sizeof(sah_Link) -
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
						    sizeof(struct page *) -	/* vm_info */
#endif
						    sizeof(dma_addr_t) -	/* dma_addr */
						    sizeof(uint32_t) -	/* virt_addr */
						    sizeof(uint8_t *) -	/* original_data */
						    sizeof(sah_Link *)));	/* original_next */

		if (result != 0) {
#ifdef DIAG_DRV_IF
			LOG_KDIAG("copy_from_user() failed.");
#endif
			drv_if_done = TRUE;
			status = FSL_RETURN_INTERNAL_ERROR_S;
		}

		if (status == FSL_RETURN_OK_S) {
			/* This will create new links which can be used to replace tmp_link
			 * in the chain. This will return a new head and tail link. */
			link_data_length = link_data_length + link_copy.len;
			new_head_link =
			    sah_Make_Links(user_ctx, &link_copy, &new_tail_link);

			if (new_head_link == NULL) {
				/* If we ran out of memory or a user pointer was invalid */
				drv_if_done = TRUE;
				status = FSL_RETURN_INTERNAL_ERROR_S;
#ifdef DIAG_DRV_IF
				LOG_KDIAG("sah_Make_Links() failed.");
#endif
			} else {
				if (is_this_the_head == TRUE) {
					/* Keep a reference to the head link */
					head_link = new_head_link;
					is_this_the_head = FALSE;
				} else {
					/* Need to update the previous links' next field to point
					 * to the current link. */
					prev_tail_link->next =
					    (void *)new_head_link->dma_addr;
					prev_tail_link->original_next =
					    new_head_link;
				}
			}
		}

		if (status == FSL_RETURN_OK_S) {
			/* Get to the next link in the chain. */
			user_link = link_copy.next;
			prev_tail_link = new_tail_link;

			/* Check if the end of the link chain was reached (TRUE) or if
			 * there is another linked to this one (FALSE) */
			drv_if_done = (user_link == NULL) ? TRUE : FALSE;
		}
	}			/* end while */

	if (status != FSL_RETURN_OK_S) {
		ret_val = NULL;
		/* There could be clean-up to do here because we may have made some
		 * successful iterations through the while loop and as a result, the
		 * links created by sah_Make_Links() need to be destroyed.
		 */
		if (head_link != NULL) {
			/* Failed somewhere in the while loop and need to clean-up. */
			sah_Destroy_Links(head_link);
		}
	} else {
		/* Success. Return the head link. */
		ret_val = head_link;
	}

	return ret_val;
}				/* sah_Copy_Links() */

/*!
*******************************************************************************
* This function takes an input link pointed to by a user-space address
* and returns a chain of links that span the physical pages pointed
* to by the input link.
*
* @brief Kernel Link Chain Constructor
*
* @param    ptr         A link pointer from user-space.
* @param    tail        The address of a link pointer. This is used to return
*                       the tail link created by this function.
*
* @return   sah_Link *  - A virtual address of the first link in the
*                         chain.
* @return   NULL        - If there was some error.
*
*/
sah_Link *sah_Make_Links(fsl_shw_uco_t * user_ctx,
							sah_Link * ptr, sah_Link ** tail)
{
	int result = -1;
	int page_index = 0;
	fsl_shw_return_t status = FSL_RETURN_OK_S;
	int is_this_the_head = TRUE;
	void *buffer_start = NULL;
	sah_Link *link = NULL;
	sah_Link *prev_link = NULL;
	sah_Link *head_link = NULL;
	sah_Link *ret_val = NULL;
	int buffer_length = 0;
	struct page **local_pages = NULL;
	int nr_pages = 0;
	int write = (sah_Link_Get_Flags(ptr) & SAH_OUTPUT_LINK) ? WRITE : READ;
	dma_addr_t phys_addr;

	/* need to retrieve stored key? */
	if (ptr->flags & SAH_STORED_KEY_INFO) {
		fsl_shw_return_t ret_status;

		/* allocate space for this link */
		link = sah_Alloc_Link();
#ifdef DIAG_MEM
		sprintf(Diag_msg, "Alloc_Link returned %p/%p\n", link,
			(void *)link->dma_addr);
		LOG_KDIAG(Diag_msg);
#endif

		if (link == NULL) {
			status = FSL_RETURN_NO_RESOURCE_S;
#ifdef DIAG_DRV_IF
			LOG_KDIAG("sah_Alloc_Link() failed!");
#endif
			return link;
		} else {
			uint32_t max_len = 0;	/* max slot length */

			/* get length and physical address of stored key */
			ret_status = system_keystore_get_slot_info(ptr->ownerid, ptr->slot, (uint32_t *) & link->data,	/* RED key address */
						      &max_len);
#ifdef DIAG_DRV_IF
		LOG_KDIAG_ARGS
			("ret_status==SCC_RET_OK? %s.  slot: %i. data: %p"
			 ". len: %i, key length: %i",
			(ret_status == FSL_RETURN_OK_S ? "yes" : "no"),
			 ptr->slot, link->data, max_len, ptr->len);
#endif

			if ((ret_status == FSL_RETURN_OK_S) && (ptr->len <= max_len)) {
				/* finish populating the link */
				link->len = ptr->len;
				link->flags = ptr->flags & ~SAH_PREPHYS_DATA;
				*tail = link;
			} else {
#ifdef DIAG_DRV_IF
				if (ret_status == FSL_RETURN_OK_S) {
					LOG_KDIAG
					    ("SCC sah_Link key slot reference is too long");
				} else {
					LOG_KDIAG
					    ("SCC sah_Link slot slot reference is invalid");
				}
#endif
				sah_Free_Link(link);
				status = FSL_RETURN_INTERNAL_ERROR_S;
				return NULL;
			}
			return link;
		}
	}	else if (ptr->flags & SAH_IN_USER_KEYSTORE) {

#ifdef FSL_HAVE_SCC2

		void *kernel_base;

		/* allocate space for this link */
		link = sah_Alloc_Link();
#ifdef DIAG_MEM
		sprintf(Diag_msg, "Alloc_Link returned %p/%p\n", link,
			(void *)link->dma_addr);
		LOG_KDIAG(Diag_msg);
#endif				/* DIAG_MEM */

		if (link == NULL) {
			status = FSL_RETURN_NO_RESOURCE_S;
#ifdef DIAG_DRV_IF
			LOG_KDIAG("sah_Alloc_Link() failed!");
#endif
			return link;
		} else {
			/* link->data points to the virtual address of the key data, however
			 * this memory does not need to be locked down.
			 */
			kernel_base = lookup_user_partition(user_ctx,
							    (uint32_t) ptr->
							    data & PAGE_MASK);

			link->data = (uint8_t *) scc_virt_to_phys(kernel_base +
								  ((unsigned
								    long)ptr->
								   data &
								   ~PAGE_MASK));

			/* Do bounds checking to ensure that the user is not overstepping
			 * the bounds of their partition.  This is a simple implementation
			 * that assumes the user only owns one partition.  It only checks
			 * to see if the address of the last byte of data steps over a
			 * page boundary.
			 */
			if ((kernel_base != NULL) &&
			    ((((uint32_t) link->data +
			       link->len) >> PAGE_SHIFT) ==
			     ((uint32_t) link->data >> PAGE_SHIFT))) {
				/* finish populating the link */
				link->len = ptr->len;
				link->flags = ptr->flags & ~SAH_PREPHYS_DATA;
				*tail = link;
			} else {
#ifdef DIAG_DRV_IF
				if (kernel_base != NULL) {
					LOG_KDIAG
					    ("SCC sah_Link key slot reference is too long");
				} else {
					LOG_KDIAG
					    ("SCC sah_Link slot slot reference is invalid");
				}
#endif
				sah_Free_Link(link);
				status = FSL_RETURN_INTERNAL_ERROR_S;
				return NULL;
			}
			return link;
		}

#else				/* FSL_HAVE_SCC2 */

		return NULL;

#endif				/* FSL_HAVE_SCC2 */
	}

	if (ptr->data == NULL) {
		/* The user buffer must not be NULL because map_user_kiobuf() cannot
		 * handle NULL pointer input.
		 */
		status = FSL_RETURN_BAD_DATA_LENGTH_S;
#ifdef DIAG_DRV_IF
		LOG_KDIAG("sah_Link data pointer is NULL.");
#endif
	}

	if (status == FSL_RETURN_OK_S) {
		unsigned long start_page = (unsigned long)ptr->data & PAGE_MASK;

		/* determine number of pages being used for this link */
		nr_pages = (((unsigned long)(ptr->data) & ~PAGE_MASK)
			    + ptr->len + ~PAGE_MASK) >> PAGE_SHIFT;

		/* ptr contains all the 'user space' information, add the pages
		 * to it also just so everything is in one place */
		local_pages =
		    kmalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);

		if (local_pages == NULL) {
			status = FSL_RETURN_NO_RESOURCE_S;	/* no memory! */
#ifdef DIAG_DRV_IF
			LOG_KDIAG("kmalloc() failed.");
#endif
		} else {
			/* get the actual pages being used in 'user space' */

			down_read(&current->mm->mmap_sem);
			result = get_user_pages(current, current->mm,
						start_page, nr_pages,
						write, 0 /* noforce */ ,
						local_pages, NULL);
			up_read(&current->mm->mmap_sem);

			if (result < nr_pages) {
#ifdef DIAG_DRV_IF
				LOG_KDIAG("get_user_pages() failed.");
#endif
				if (result > 0) {
					for (page_index = 0;
					     page_index < result;
					     page_index++) {
						page_cache_release(local_pages
								   [page_index]);
					}
				}
				status = FSL_RETURN_INTERNAL_ERROR_S;
			}
		}
	}

	/* Now we can walk through the list of pages in the buffer */
	if (status == FSL_RETURN_OK_S) {

#if defined(FLUSH_SPECIFIC_DATA_ONLY) && !defined(HAS_L2_CACHE)
		/*
		 * Now that pages are wired, clear user data from cache lines.  When
		 * there is just an L1 cache, clean based on user virtual for ARM.
		 */
		if (write == WRITE) {
			os_cache_flush_range(ptr->data, ptr->len);
		} else {
			os_cache_clean_range(ptr->data, ptr->len);
		}
#endif

		for (page_index = 0; page_index < nr_pages; page_index++) {
			/* Allocate a new link structure */
			link = sah_Alloc_Link();
#ifdef DIAG_MEM
			sprintf(Diag_msg, "Alloc_Link returned %p/%p\n", link,
				(void *)link->dma_addr);
			LOG_KDIAG(Diag_msg);
#endif
			if (link == NULL) {
#ifdef DIAG_DRV_IF
				LOG_KDIAG("sah_Alloc_Link() failed.");
#endif
				status = FSL_RETURN_NO_RESOURCE_S;

				/* need to free the rest of the pages. Destroy_Links will take
				 * care of the ones already assigned to a link */
				for (; page_index < nr_pages; page_index++) {
					page_cache_release(local_pages
							   [page_index]);
				}
				break;	/* exit 'for page_index' loop */
			}

			if (status == FSL_RETURN_OK_S) {
				if (is_this_the_head == TRUE) {
					/* keep a reference to the head link */
					head_link = link;
					/* remember that we have seen the head link */
					is_this_the_head = FALSE;
				} else {
					/* If this is not the head link then set the previous
					 * link's next pointer to point to this link */
					prev_link->original_next = link;
					prev_link->next =
					    (sah_Link *) link->dma_addr;
				}

				buffer_start =
				    page_address(local_pages[page_index]);

				phys_addr =
					page_to_phys(local_pages[page_index]);

				if (page_index == 0) {
					/* If this is the first page, there might be an
					 * offset. We need to increment the address by this offset
					 * so we don't just get the start of the page.
					 */
					buffer_start +=
					    (unsigned long)
					    sah_Link_Get_Data(ptr)
					    & ~PAGE_MASK;
					phys_addr +=
					    (unsigned long)
					    sah_Link_Get_Data(ptr)
					    & ~PAGE_MASK;
					buffer_length = PAGE_SIZE
					    -
					    ((unsigned long)
					     sah_Link_Get_Data(ptr)
					     & ~PAGE_MASK);
				} else {
					buffer_length = PAGE_SIZE;
				}

				if (page_index == nr_pages - 1) {
					/* if this is the last page, we need to adjust
					 * the buffer_length to account for the last page being
					 * partially used.
					 */
					buffer_length -=
					    nr_pages * PAGE_SIZE -
					    sah_Link_Get_Len(ptr) -
					    ((unsigned long)
					     sah_Link_Get_Data(ptr) &
					     ~PAGE_MASK);
				}
#if defined(FLUSH_SPECIFIC_DATA_ONLY) && defined(HAS_L2_CACHE)
				/*
				 * When there is an L2 cache, clean based on kernel
				 * virtual..
				 */
				if (write == WRITE) {
					os_cache_flush_range(buffer_start,
							     buffer_length);
				} else {
					os_cache_clean_range(buffer_start,
							     buffer_length);
				}
#endif

				/* Fill in link information */
				link->len = buffer_length;
#if !defined(HAS_L2_CACHE)
				/* use original virtual */
				link->original_data = ptr->data;
#else
				/* use kernel virtual */
				link->original_data = buffer_start;
#endif
				link->data = (void *)phys_addr;
				link->flags = ptr->flags & ~SAH_PREPHYS_DATA;
				link->vm_info = local_pages[page_index];
				prev_link = link;

#if defined(NO_OUTPUT_1K_CROSSING) || defined(NO_1K_CROSSING)
				if (
#ifdef NO_OUTPUT_1K_CROSSING
					   /* Insert extra link if 1k boundary on output pointer
					    * crossed not at an 8-word boundary */
					   (link->flags & SAH_OUTPUT_LINK) &&
					   (((uint32_t) buffer_start % 32) != 0)
					   &&
#endif
					   ((((uint32_t) buffer_start & 1023) +
					     buffer_length) > 1024)) {

					/* Shorten current link to 1k boundary */
					link->len =
					    1024 -
					    ((uint32_t) buffer_start % 1024);

					/* Get new link to follow it */
					link = sah_Alloc_Link();
					prev_link->len =
					    1024 -
					    ((uint32_t) buffer_start % 1024);
					prev_link->original_next = link;
					prev_link->next =
					    (sah_Link *) link->dma_addr;
					buffer_length -= prev_link->len;
					buffer_start += prev_link->len;

#if !defined(HAS_L2_CACHE)
					/* use original virtual */
					link->original_data = ptr->data;
#else
					/* use kernel virtual */
					link->original_data = buffer_start;
#endif
					link->data = (void *)phys_addr;
					link->vm_info = prev_link->vm_info;
					prev_link->vm_info = NULL;	/* delay release */
					link->flags = ptr->flags;
					link->len = buffer_length;
					prev_link = link;
				}	/* while link would cross 1K boundary */
#endif				/* 1K_CROSSING */
			}
		}		/* for each page */
	}

	if (local_pages != NULL) {
		kfree(local_pages);
	}

	if (status != FSL_RETURN_OK_S) {
		/* De-allocated any links created, this routine first looks if
		 * head_link is NULL */
		sah_Destroy_Links(head_link);

		/* Clean-up of the KIOBUF will occur in the * sah_Copy_Descriptors()
		 * function.
		 * Clean-up of the Queue entry must occur in the function called
		 * sah_Copy_Descriptors().
		 */
	} else {

		/* Success. Return the head link. */
		ret_val = head_link;
		link->original_next = NULL;
		/* return the tail link as well */
		*tail = link;
	}

	return ret_val;
}				/* sah_Make_Links() */

/*!
*******************************************************************************
* This walks through a SAHARA descriptor chain and frees everything
* that is not NULL. Finally it also unmaps all of the physical memory and
* frees the kiobuf_list Queue.
*
* @brief Kernel Descriptor Chain Destructor
*
* @param    desc        A Descriptor pointer from kernel-space. This should be
*                       in bus address space.
*
* @return   void
*
*/
void sah_Destroy_Descriptors(sah_Head_Desc * head_desc)
{
	sah_Desc *this_desc = (sah_Desc *) head_desc;
	sah_Desc *next_desc = NULL;
	int this_is_head = 1;

	/*
	 * Flush the D-cache. This flush is here because the hardware has finished
	 * processing this descriptor and probably has changed the contents of
	 * some linked user buffers as a result.  This flush will enable
	 * user-space applications to see the correct data rather than the
	 * out-of-date cached version.
	 */
#ifndef FLUSH_SPECIFIC_DATA_ONLY
	os_flush_cache_all();
#endif

	head_desc = (sah_Head_Desc *) this_desc->virt_addr;

	while (this_desc != NULL) {
		if (this_desc->ptr1 != NULL) {
			sah_Destroy_Links(this_desc->original_ptr1
					  ? this_desc->
					  original_ptr1 : this_desc->ptr1);
		}
		if (this_desc->ptr2 != NULL) {
			sah_Destroy_Links(this_desc->original_ptr2
					  ? this_desc->
					  original_ptr2 : this_desc->ptr2);
		}

		/* Get a bus pointer to the next Descriptor */
		next_desc = (this_desc->original_next
			     ? this_desc->original_next : this_desc->next);

		/* Zero the header and Length fields for security reasons. */
		this_desc->header = 0;
		this_desc->len1 = 0;
		this_desc->len2 = 0;

		if (this_is_head) {
			sah_Free_Head_Descriptor(head_desc);
#ifdef DIAG_MEM
			sprintf(Diag_msg, "Free_Head_Descriptor: %p\n",
				head_desc);
			LOG_KDIAG(Diag_msg);
#endif
			this_is_head = 0;
		} else {
			/* free this descriptor */
			sah_Free_Descriptor(this_desc);
#ifdef DIAG_MEM
			sprintf(Diag_msg, "Free_Descriptor: %p\n", this_desc);
			LOG_KDIAG(Diag_msg);
#endif
		}

		/* Set up for next round. */
		this_desc = (sah_Desc *) next_desc;
	}
}

/*!
*******************************************************************************
* This walks through a SAHARA link chain and frees everything that is
* not NULL excluding user-space buffers.
*
* @brief Kernel Link Chain Destructor
*
* @param    link    A Link pointer from kernel-space.
*
* @return   void
*
*/
void sah_Destroy_Links(sah_Link * link)
{
	sah_Link *this_link = link;
	sah_Link *next_link = NULL;

	while (this_link != NULL) {

		/* if this link indicates an associated page, process it */
		if (this_link->vm_info != NULL) {
			/* since this function is only called from the routine that
			 * creates a kernel copy of the user space descriptor chain,
			 * there are no pages to dirty. All that is needed is to release
			 * the page from cache */
			page_cache_release(this_link->vm_info);
		}

		/* Get a bus pointer to the next Link */
		next_link = (this_link->original_next
			     ? this_link->original_next : this_link->next);

		/* Zero the Pointer and Length fields for security reasons. */
		this_link->data = NULL;
		this_link->len = 0;

		/* Free this Link */
		sah_Free_Link(this_link);
#ifdef DIAG_MEM
		sprintf(Diag_msg, "Free_Link: %p\n", this_link);
		LOG_KDIAG(Diag_msg);
#endif

		/* Look at the next Link */
		this_link = next_link;
	}
}

/*!
*******************************************************************************
* @brief Initialize memory manager/mapper.
*
* In 2.4, this function also allocates a kiovec to be used when mapping user
* data to kernel space
*
* @return 0 for success, OS error code on failure
*
*/
int sah_Init_Mem_Map(void)
{
	int ret = OS_ERROR_FAIL_S;

	mem_lock = os_lock_alloc_init();

	/*
	 * If one of these fails, change the calculation in the #define earlier in
	 * the file to be the other one.
	 */
	if (sizeof(sah_Link) > MEM_BLOCK_SIZE) {
		os_printk("Sahara Driver: sah_Link structure is too large\n");
	} else if (sizeof(sah_Desc) > MEM_BLOCK_SIZE) {
		os_printk("Sahara Driver: sah_Desc structure is too large\n");
	} else {
		ret = OS_ERROR_OK_S;
	}

#ifndef SELF_MANAGED_POOL

	big_dma_pool = dma_pool_create("sah_big_blocks", NULL,
				       sizeof(Mem_Big_Block), sizeof(uint32_t),
				       PAGE_SIZE);
	small_dma_pool = dma_pool_create("sah_small_blocks", NULL,
					 sizeof(Mem_Block), sizeof(uint32_t),
					 PAGE_SIZE);
#else

#endif
	return ret;
}

/*!
*******************************************************************************
* @brief Clean up memory manager/mapper.
*
* In 2.4, this function also frees the kiovec used when mapping user data to
* kernel space.
*
* @return none
*
*/
void sah_Stop_Mem_Map(void)
{
	os_lock_deallocate(mem_lock);

#ifndef SELF_MANAGED_POOL
	if (big_dma_pool != NULL) {
		dma_pool_destroy(big_dma_pool);
	}
	if (small_dma_pool != NULL) {
		dma_pool_destroy(small_dma_pool);
	}
#endif
}

/*!
*******************************************************************************
* Allocate Head descriptor from free pool.
*
* @brief Allocate Head descriptor
*
* @return sah_Head_Desc Free descriptor, NULL if no free descriptors available.
*
*/
sah_Head_Desc *sah_Alloc_Head_Descriptor(void)
{
	Mem_Big_Block *block;
	sah_Head_Desc *desc;

	block = sah_Alloc_Big_Block();
	if (block != NULL) {
		/* initialize everything */
		desc = (sah_Head_Desc *) block->data;

		desc->desc.virt_addr = (sah_Desc *) desc;
		desc->desc.dma_addr = block->dma_addr;
		desc->desc.original_ptr1 = NULL;
		desc->desc.original_ptr2 = NULL;
		desc->desc.original_next = NULL;

		desc->desc.ptr1 = NULL;
		desc->desc.ptr2 = NULL;
		desc->desc.next = NULL;
	} else {
		desc = NULL;
	}

	return desc;
}

/*!
*******************************************************************************
* Allocate descriptor from free pool.
*
* @brief Allocate descriptor
*
* @return sah_Desc Free descriptor, NULL if no free descriptors available.
*
*/
sah_Desc *sah_Alloc_Descriptor(void)
{
	Mem_Block *block;
	sah_Desc *desc;

	block = sah_Alloc_Block();
	if (block != NULL) {
		/* initialize everything */
		desc = (sah_Desc *) block->data;

		desc->virt_addr = desc;
		desc->dma_addr = block->dma_addr;
		desc->original_ptr1 = NULL;
		desc->original_ptr2 = NULL;
		desc->original_next = NULL;

		desc->ptr1 = NULL;
		desc->ptr2 = NULL;
		desc->next = NULL;
	} else {
		desc = NULL;
	}

	return (desc);
}

/*!
*******************************************************************************
* Allocate link from free pool.
*
* @brief Allocate link
*
* @return sah_Link Free link, NULL if no free links available.
*
*/
sah_Link *sah_Alloc_Link(void)
{
	Mem_Block *block;
	sah_Link *link;

	block = sah_Alloc_Block();
	if (block != NULL) {
		/* initialize everything */
		link = (sah_Link *) block->data;

		link->virt_addr = link;
		link->original_next = NULL;
		link->original_data = NULL;
		/* information found in allocated block */
		link->dma_addr = block->dma_addr;

		/* Sahara link fields */
		link->len = 0;
		link->data = NULL;
		link->next = NULL;

		/* driver required fields */
		link->flags = 0;
		link->vm_info = NULL;
	} else {
		link = NULL;
	}

	return link;
}

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Add a new page to end of block free pool. This will allocate one page and
* fill the pool with entries, appending to the end.
*
* @brief Add page of blocks to block free pool.
*
* @pre This function must be called with the #mem_lock held.
*
* @param   big    0        - make blocks big enough for sah_Desc
*                 non-zero - make blocks big enough for sah_Head_Desc
*
* @return int TRUE if blocks added succeesfully, FALSE otherwise
*
*/
int sah_Block_Add_Page(int big)
{
	void *page;
	int success;
	dma_addr_t dma_addr;
	unsigned block_index;
	uint32_t dma_offset;
	unsigned block_entries =
	    big ? MEM_BIG_BLOCK_ENTRIES : MEM_BLOCK_ENTRIES;
	unsigned block_size = big ? sizeof(Mem_Big_Block) : sizeof(Mem_Block);
	void *block;

	/* Allocate page of memory */
#ifndef USE_COHERENT_MEMORY
	page = os_alloc_memory(PAGE_SIZE, GFP_ATOMIC | __GFP_DMA);
	dma_addr = os_pa(page);
#else
	page = os_alloc_coherent(PAGE_SIZE, &dma_addr, GFP_ATOMIC);
#endif
	if (page != NULL) {
		/*
		 * Find the difference between the virtual address and the DMA
		 * address of the page. This is used later to determine the DMA
		 * address of each individual block.
		 */
		dma_offset = page - (void *)dma_addr;

		/* Split page into blocks and add to free pool */
		block = page;
		for (block_index = 0; block_index < block_entries;
		     block_index++) {
			if (big) {
				register Mem_Big_Block *blockp = block;
				blockp->dma_addr =
				    (uint32_t) (block - dma_offset);
				sah_Append_Big_Block(blockp);
			} else {
				register Mem_Block *blockp = block;
				blockp->dma_addr =
				    (uint32_t) (block - dma_offset);
				/* sah_Append_Block must be protected with spin locks. This is
				 * done in sah_Alloc_Block(), which calls
				 * sah_Block_Add_Page() */
				sah_Append_Block(blockp);
			}
			block += block_size;
		}
		success = TRUE;
#ifdef DIAG_MEM
		LOG_KDIAG("Succeeded in allocating new page");
#endif
	} else {
		success = FALSE;
#ifdef DIAG_MEM
		LOG_KDIAG("Failed in allocating new page");
#endif
	}

	return success;
}
#endif				/* SELF_MANAGED_POOL */

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Allocate block from free pool. A block is large enough to fit either a link
* or descriptor.
*
* @brief Allocate memory block
*
* @return Mem_Block Free block, NULL if no free blocks available.
*
*/
static Mem_Big_Block *sah_Alloc_Big_Block(void)
{
	Mem_Big_Block *block;
	os_lock_context_t lock_flags;

	os_lock_save_context(mem_lock, lock_flags);

	/* If the pool is empty, try to allocate more entries */
	if (big_block_free_head == NULL) {
		(void)sah_Block_Add_Page(1);
	}

	/* Check that the pool now has some free entries */
	if (big_block_free_head != NULL) {
		/* Return the head of the free pool */
		block = big_block_free_head;

		big_block_free_head = big_block_free_head->next;
		if (big_block_free_head == NULL) {
			/* Allocated last entry in pool */
			big_block_free_tail = NULL;
		}
	} else {
		block = NULL;
	}
	os_unlock_restore_context(mem_lock, lock_flags);

	return block;
}
#else
/*!
*******************************************************************************
* Allocate block from free pool. A block is large enough to fit either a link
* or descriptor.
*
* @brief Allocate memory block
*
* @return Mem_Block Free block, NULL if no free blocks available.
*
*/
static Mem_Big_Block *sah_Alloc_Big_Block(void)
{
	dma_addr_t dma_addr;
	Mem_Big_Block *block =
	    dma_pool_alloc(big_dma_pool, GFP_ATOMIC, &dma_addr);

	if (block == NULL) {
	} else {
		block->dma_addr = dma_addr;
	}

	return block;
}
#endif

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Allocate block from free pool. A block is large enough to fit either a link
* or descriptor.
*
* @brief Allocate memory block
*
* @return Mem_Block Free block, NULL if no free blocks available.
*
*/
/******************************************************************************
*
* MODIFICATION HISTORY:
*
* Date         Person     Change
* 31/10/2003   RWK        PR52734 - Implement functions to allocate
*                                   descriptors and links. Replace
*                                   consistent_alloc() calls. Initial creation.
*
******************************************************************************/
static Mem_Block *sah_Alloc_Block(void)
{
	Mem_Block *block;
	os_lock_context_t lock_flags;

	os_lock_save_context(mem_lock, lock_flags);

	/* If the pool is empty, try to allocate more entries */
	if (block_free_head == NULL) {
		(void)sah_Block_Add_Page(0);
	}

	/* Check that the pool now has some free entries */
	if (block_free_head != NULL) {
		/* Return the head of the free pool */
		block = block_free_head;

		block_free_head = block_free_head->next;
		if (block_free_head == NULL) {
			/* Allocated last entry in pool */
			block_free_tail = NULL;
		}
	} else {
		block = NULL;
	}
	os_unlock_restore_context(mem_lock, lock_flags);

	return block;
}
#else
/*!
*******************************************************************************
* Allocate block from free pool. A block is large enough to fit either a link
* or descriptor.
*
* @brief Allocate memory block
*
* @return Mem_Block Free block, NULL if no free blocks available.
*
*/
/******************************************************************************
*
* MODIFICATION HISTORY:
*
* Date         Person     Change
* 31/10/2003   RWK        PR52734 - Implement functions to allocate
*                                   descriptors and links. Replace
*                                   consistent_alloc() calls. Initial creation.
*
******************************************************************************/
static Mem_Block *sah_Alloc_Block(void)
{

	dma_addr_t dma_addr;
	Mem_Block *block =
	    dma_pool_alloc(small_dma_pool, GFP_ATOMIC, &dma_addr);

	if (block == NULL) {
	} else {
		block->dma_addr = dma_addr;
	}

	return block;
}
#endif

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Free memory block back to free pool
*
* @brief Free memory block
*
* @param    block       A block allocated with sah_Alloc_Block().
*
* @return none
*
*/
static void sah_Free_Block(Mem_Block * block)
{
	os_lock_context_t lock_flags;

	os_lock_save_context(mem_lock, lock_flags);
	sah_Append_Block(block);
	os_unlock_restore_context(mem_lock, lock_flags);
}
#else
/*!
*******************************************************************************
* Free memory block back to free pool
*
* @brief Free memory block
*
* @param    block       A block allocated with sah_Alloc_Block().
*
* @return none
*
*/
static void sah_Free_Block(Mem_Block * block)
{
	dma_pool_free(small_dma_pool, block, block->dma_addr);
}
#endif

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Free memory block back to free pool
*
* @brief Free memory block
*
* @param    block       A block allocated with sah_Alloc_Block().
*
* @return none
*
*/
static void sah_Free_Big_Block(Mem_Big_Block * block)
{
	os_lock_context_t lock_flags;

	os_lock_save_context(mem_lock, lock_flags);
	sah_Append_Big_Block(block);
	os_unlock_restore_context(mem_lock, lock_flags);
}
#else
/*!
*******************************************************************************
* Free memory block back to free pool
*
* @brief Free memory block
*
* @param    block       A block allocated with sah_Alloc_Block().
*
* @return none
*
*/
static void sah_Free_Big_Block(Mem_Big_Block * block)
{
	dma_pool_free(big_dma_pool, block, block->dma_addr);
}
#endif

#ifdef SELF_MANAGED_POOL
/*!
*******************************************************************************
* Append memory block to end of free pool.
*
* @param    block       A block entry
*
* @return none
*
* @pre This function must be called with the #mem_lock held.
*
* @brief Append memory block to free pool
*/
static void sah_Append_Big_Block(Mem_Big_Block * block)
{

	/* Initialise block */
	block->next = NULL;

	/* Remember that block may be the first in the pool */
	if (big_block_free_tail != NULL) {
		big_block_free_tail->next = block;
	} else {
		/* Pool is empty */
		big_block_free_head = block;
	}

	big_block_free_tail = block;
}

/*!
*******************************************************************************
* Append memory block to end of free pool.
*
* @brief Append memory block to free pool
*
* @param    block       A block entry
*
* @return none
*
* @pre #mem_lock must be held
*
*/
static void sah_Append_Block(Mem_Block * block)
{

	/* Initialise block */
	block->next = NULL;

	/* Remember that block may be the first in the pool */
	if (block_free_tail != NULL) {
		block_free_tail->next = block;
	} else {
		/* Pool is empty */
		block_free_head = block;
	}

	block_free_tail = block;
}
#endif				/* SELF_MANAGED_POOL */

/* End of sah_memory_mapper.c */
