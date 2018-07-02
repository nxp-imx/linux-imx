/*
 * Copyright (c) 2010,2015-2016 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */


#include <adf_os_mem.h>
#ifdef MEMORY_DEBUG
#include <vos_memory.h>
#endif

#ifdef MEMORY_DEBUG
void *
adf_os_mem_alloc_debug(adf_os_device_t osdev,
				adf_os_size_t size,
				const char *fileName,
				a_uint32_t lineNum)
{
	void *p = vos_mem_malloc_debug(size, fileName, lineNum);
	if (p) {
		memset(p, 0, size);
	}
	return p;
}

void
adf_os_mem_free_debug(void *buf)
{
	vos_mem_free(buf);
}

#endif

void *
adf_os_mem_alloc_outline(adf_os_device_t osdev, size_t size)
{
    return __adf_os_mem_alloc(osdev, size);
}

void
adf_os_mem_free_outline(void *buf)
{
    __adf_os_mem_free(buf);
}

void
adf_os_mem_zero_outline(void *buf, adf_os_size_t size)
{
    __adf_os_mem_zero(buf, size);
}

/**
 * adf_os_mem_multi_pages_alloc() - allocate large size of kernel memory
 * @osdev:          OS device handle pointer
 * @pages:          Multi page information storage
 * @element_size:   Each element size
 * @element_num:    Total number of elements should be allocated
 * @memctxt:        Memory context
 * @cacheable:      Coherent memory or cacheable memory
 *
 * This function will allocate large size of memory over multiple pages.
 * Large size of contiguous memory allocation will fail frequentely, so
 * instead of allocate large memory by one shot, allocate through multiple, non
 * contiguous memory and combine pages when actual usage
 *
 * Return: None
 */
void adf_os_mem_multi_pages_alloc(adf_os_device_t osdev,
			struct adf_os_mem_multi_page_t *pages,
			size_t element_size,
			uint16_t element_num,
			adf_os_dma_context_t memctxt,
			bool cacheable)
{
	uint16_t page_idx;
	struct adf_os_mem_dma_page_t *dma_pages;
	void **cacheable_pages = NULL;
	uint16_t i;

	pages->num_element_per_page = PAGE_SIZE / element_size;
	if (!pages->num_element_per_page) {
		adf_os_print("Invalid page %d or element size %d",
			(int)PAGE_SIZE, (int)element_size);
		goto out_fail;
	}

	pages->num_pages = element_num / pages->num_element_per_page;
	if (element_num % pages->num_element_per_page)
		pages->num_pages++;

	if (cacheable) {
		/* Pages information storage */
		pages->cacheable_pages = adf_os_mem_alloc(osdev,
			pages->num_pages * sizeof(pages->cacheable_pages));
		if (!pages->cacheable_pages) {
			adf_os_print("Cacheable page storage alloc fail");
			goto out_fail;
		}

		cacheable_pages = pages->cacheable_pages;
		for (page_idx = 0; page_idx < pages->num_pages; page_idx++) {
			cacheable_pages[page_idx] = adf_os_mem_alloc(
				osdev, PAGE_SIZE);
			if (!cacheable_pages[page_idx]) {
				adf_os_print("cacheable page alloc fail, pi %d",
					page_idx);
				goto page_alloc_fail;
			}
		}
		pages->dma_pages = NULL;
	} else {
		pages->dma_pages = adf_os_mem_alloc(osdev,
		       pages->num_pages * sizeof(struct adf_os_mem_dma_page_t));
		if (!pages->dma_pages) {
			adf_os_print("dmaable page storage alloc fail");
			goto out_fail;
		}

		dma_pages = pages->dma_pages;
		for (page_idx = 0; page_idx < pages->num_pages; page_idx++) {
			dma_pages->page_v_addr_start =
				adf_os_mem_alloc_consistent(osdev, PAGE_SIZE,
				&dma_pages->page_p_addr, memctxt);
			if (!dma_pages->page_v_addr_start) {
				adf_os_print("dmaable page alloc fail pi %d",
					page_idx);
				goto page_alloc_fail;
			}
			dma_pages->page_v_addr_end =
				dma_pages->page_v_addr_start + PAGE_SIZE;
			dma_pages++;
		}
		pages->cacheable_pages = NULL;
	}
	return;

page_alloc_fail:
	if (cacheable) {
		for (i = 0; i < page_idx; i++)
			adf_os_mem_free(pages->cacheable_pages[i]);
		adf_os_mem_free(pages->cacheable_pages);
	} else {
		dma_pages = pages->dma_pages;
		for (i = 0; i < page_idx; i++) {
			adf_os_mem_free_consistent(osdev, PAGE_SIZE,
				dma_pages->page_v_addr_start,
				dma_pages->page_p_addr, memctxt);
			dma_pages++;
		}
		adf_os_mem_free(pages->dma_pages);
	}

out_fail:
	pages->cacheable_pages = NULL;
	pages->dma_pages = NULL;
	pages->num_pages = 0;
	return;
}

/**
 * adf_os_mem_multi_pages_free() - free large size of kernel memory
 * @osdev:      OS device handle pointer
 * @pages:      Multi page information storage
 * @memctxt:    Memory context
 * @cacheable:  Coherent memory or cacheable memory
 *
 * This function will free large size of memory over multiple pages.
 *
 * Return: None
 */
void adf_os_mem_multi_pages_free(adf_os_device_t osdev,
			struct adf_os_mem_multi_page_t *pages,
			adf_os_dma_context_t memctxt,
			bool cacheable)
{
	unsigned int page_idx;
	struct adf_os_mem_dma_page_t *dma_pages;

	if (cacheable) {
		for (page_idx = 0; page_idx < pages->num_pages; page_idx++)
			adf_os_mem_free(pages->cacheable_pages[page_idx]);
		adf_os_mem_free(pages->cacheable_pages);
	} else {
		dma_pages = pages->dma_pages;
		for (page_idx = 0; page_idx < pages->num_pages; page_idx++) {
			adf_os_mem_free_consistent(osdev, PAGE_SIZE,
				dma_pages->page_v_addr_start,
				dma_pages->page_p_addr, memctxt);
			dma_pages++;
		}
		adf_os_mem_free(pages->dma_pages);
	}

	pages->cacheable_pages = NULL;
	pages->dma_pages = NULL;
	pages->num_pages = 0;
	return;
}

