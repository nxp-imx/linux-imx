/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */



/**
 * Memory management functions, from Sahara Crypto API
 *
 * This is a subset of the memory management functions from the Sahara Crypto
 * API, and is intended to support user secure partitions.
 */

#include "portable_os.h"
#include "fsl_shw.h"

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pagemap.h>

#ifdef SHW_DEBUG
#include <diagnostic.h>
#endif

/* Page context structure.  Used by wire_user_memory and unwire_user_memory */
typedef struct page_ctx_t {
    uint32_t        count;
    struct page   **local_pages;
} page_ctx_t;

/**
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
void* wire_user_memory(void* address, uint32_t length, void **page_ctx)
{
    void* kernel_black_addr = NULL;
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
        status = FSL_RETURN_NO_RESOURCE_S; /* no memory! */
#ifdef DIAG_DRV_IF
        LOG_KDIAG("kmalloc() failed.");
#endif
        return NULL;
    }

    /* Set the page pointer to point to the allocated region of memory */
    page_context->local_pages = (void*)page_context + sizeof(page_ctx_t);

#ifdef DIAG_DRV_IF
    LOG_KDIAG_ARGS("page_context at: %p, local_pages at: %p",
                   (void *)page_context,
                   (void *)(page_context->local_pages));
#endif

    /* Wire down the pages from user space */
    down_read(&current->mm->mmap_sem);
    result = get_user_pages(current, current->mm,
                            start_page, nr_pages,
                            WRITE, 0 /* noforce */,
                            (page_context->local_pages), NULL);
    up_read(&current->mm->mmap_sem);

    if (result < nr_pages) {
#ifdef DIAG_DRV_IF
        LOG_KDIAG("get_user_pages() failed.");
#endif
        if (result > 0) {
            for (page_index = 0; page_index < result; page_index++)
                page_cache_release((page_context->local_pages[page_index]));

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


/**
*******************************************************************************
* Release and unmap a region of user memory.
*
* @param    page_ctx    Page context from wire_user_memory
*/
void unwire_user_memory(void** page_ctx)
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
        for (page_index = 0; page_index < page_context->count; page_index++)
            page_cache_release(page_context->local_pages[page_index]);

        kfree(page_context);
        *page_ctx = NULL;
    }
}


/**
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
map_user_memory(struct vm_area_struct *vma, uint32_t physical_addr, uint32_t size)
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
                             size,
                             vma->vm_page_prot);

    return retval;
}


/**
*******************************************************************************
* Remove some memory from a user's memory space
*
* @param    user_addr       Userspace address of the memory to be unmapped
* @param    size            Size of the memory to map (bytes)
*
* @return
*/
os_error_code
unmap_user_memory(uint32_t user_addr, uint32_t size)
{
    os_error_code retval;
    struct mm_struct *mm = current->mm;

    /* Unmap the memory region (see sys_munmap in mmap.c) */
    down_write(&mm->mmap_sem);
    retval = do_munmap(mm, (unsigned long)user_addr, size);
    up_write(&mm->mmap_sem);

    return retval;
}
