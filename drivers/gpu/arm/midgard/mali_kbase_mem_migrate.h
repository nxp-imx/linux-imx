/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022-2026 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */
#ifndef _KBASE_MEM_MIGRATE_H
#define _KBASE_MEM_MIGRATE_H

#include <linux/version_compat_defs.h>

#include <linux/types.h>
#include <linux/migrate.h>

struct kbase_device;
struct file;
struct page;

/**
 * DOC: Base kernel page migration implementation.
 */

#define PAGE_STATUS_MASK ((u8)0x3F)
#define PAGE_STATUS_GET(status) (status & PAGE_STATUS_MASK)
#define PAGE_STATUS_SET(status, value) ((status & ~PAGE_STATUS_MASK) | (value & PAGE_STATUS_MASK))

#define PAGE_ISOLATE_SHIFT (7)
#define PAGE_ISOLATE_MASK ((u8)1 << PAGE_ISOLATE_SHIFT)
#define PAGE_ISOLATE_SET(status, value) \
	((status & ~PAGE_ISOLATE_MASK) | (value << PAGE_ISOLATE_SHIFT))
#define IS_PAGE_ISOLATED(status) ((bool)(status & PAGE_ISOLATE_MASK))

#define PAGE_MOVABLE_SHIFT (6)
#define PAGE_MOVABLE_MASK ((u8)1 << PAGE_MOVABLE_SHIFT)
#define PAGE_MOVABLE_CLEAR(status) ((status) & ~PAGE_MOVABLE_MASK)
#define PAGE_MOVABLE_SET(status) (status | PAGE_MOVABLE_MASK)

#define IS_PAGE_MOVABLE(status) ((bool)(status & PAGE_MOVABLE_MASK))

#if (KERNEL_VERSION(6, 0, 0) <= LINUX_VERSION_CODE)
extern const struct movable_operations movable_ops;
#endif

/**
 * kbase_clear_page_movable - Clear the "movable" property from the page
 * @p: Page to clear the "movable" property from.
 *
 * This function is supposed to be called just before releasing
 * the page and will take care of removing the "movable" property
 * from the page, if necessary.
 */
void kbase_clear_page_movable(struct page *p);

#if (KERNEL_VERSION(6, 0, 0) <= LINUX_VERSION_CODE)
/**
 * kbase_set_page_movable - Set the "movable" property for the page
 * @p:   Page to set the "movable" property for.
 * @ops: Movable operations to be set for the page.
 *
 * This function will set the "movable" property for the page and,
 * if necessary, it will also set and initialize the movable operations.
 */
void kbase_set_page_movable(struct page *p, const struct movable_operations *ops);
#endif

/**
 * kbase_alloc_page_metadata - Allocate and initialize page metadata
 * @kbdev:    Pointer to kbase device.
 * @p:        Page to assign metadata to.
 * @dma_addr: DMA address mapped to paged.
 * @group_id: Memory group ID associated with the entity that is
 *            allocating the page metadata.
 *
 * This will allocate memory for the page's metadata, initialize it and
 * assign a reference to the page's private field. Importantly, once
 * the metadata is set and ready this function will mark the page as
 * movable.
 *
 * Return: true if successful or false otherwise.
 */
bool kbase_alloc_page_metadata(struct kbase_device *kbdev, struct page *p, dma_addr_t dma_addr,
			       u8 group_id);

bool kbase_is_page_migration_enabled(void);

/**
 * kbase_free_page_later - Defer freeing of given page.
 * @kbdev:  Pointer to kbase device
 * @p:      Page to free
 *
 * This will add given page to a list of pages which will be freed at
 * a later time.
 */
void kbase_free_page_later(struct kbase_device *kbdev, struct page *p);

#if (KERNEL_VERSION(6, 0, 0) > LINUX_VERSION_CODE)
/*
 * kbase_mem_migrate_set_address_space_ops - Set address space operations
 *
 * @kbdev: Pointer to object representing an instance of GPU platform device.
 * @filp:  Pointer to the struct file corresponding to device file
 *         /dev/malixx instance, passed to the file's open method.
 *
 * Assign address space operations to the given file struct @filp and
 * add a reference to @kbdev.
 */
void kbase_mem_migrate_set_address_space_ops(struct kbase_device *kbdev, struct file *const filp);
#endif

/*
 * kbase_mem_migrate_init - Initialise kbase page migration
 *
 * @kbdev: Pointer to kbase device
 *
 * Enables page migration by default based on GPU and setup work queue to
 * defer freeing pages during page migration callbacks.
 * This function must be called only when a kbase device is initialized.
 */
void kbase_mem_migrate_init(struct kbase_device *kbdev);

/*
 * kbase_mem_migrate_term - Terminate kbase page migration
 *
 * @kbdev: Pointer to kbase device
 *
 * This will flush any work left to free pages from page migration
 * and destroy workqueue associated.
 */
void kbase_mem_migrate_term(struct kbase_device *kbdev);

/**
 * enum kbase_page_migration_test_hook_point - Page migration test hook points.
 * @KBASE_PM_TEST_HOOK_PAGE_MIGRATE_AFTER_STATUS: kbase_page_migrate()
 *	has read the isolated page status and dropped the metadata lock.
 * @KBASE_PM_TEST_HOOK_ALLOC_MAPPED_AFTER_MD:
 *	kbasep_migrate_page_allocated_mapped() has cached metadata and dropped
 *	the metadata lock.
 * @KBASE_PM_TEST_HOOK_PT_MAPPED_AFTER_MD:
 *	kbasep_migrate_page_pt_mapped() has cached metadata and dropped the
 *	metadata lock.
 * @KBASE_PM_TEST_HOOK_DATA_MMU_AFTER_MD:
 *	kbase_mmu_migrate_data_page() has cached metadata and dropped the
 *	metadata lock before taking mmu_lock.
 * @KBASE_PM_TEST_HOOK_PGD_MMU_AFTER_MD:
 *	kbase_mmu_migrate_pgd_page() has cached metadata and dropped the metadata
 *	lock before taking mmu_lock.
 * @KBASE_PM_TEST_HOOK_PGD_MMU_AFTER_KCTX_DEREF:
 *	kbase_mmu_migrate_pgd_page() has dereferenced the cached mmut kctx pointer.
 * @KBASE_PM_TEST_HOOK_PAGE_FREE_IN_PROGRESS:
 *	Context termination has marked the page as free-in-progress while isolated.
 */
enum kbase_page_migration_test_hook_point {
	KBASE_PM_TEST_HOOK_PAGE_MIGRATE_AFTER_STATUS,
	KBASE_PM_TEST_HOOK_ALLOC_MAPPED_AFTER_MD,
	KBASE_PM_TEST_HOOK_PT_MAPPED_AFTER_MD,
	KBASE_PM_TEST_HOOK_DATA_MMU_AFTER_MD,
	KBASE_PM_TEST_HOOK_PGD_MMU_AFTER_MD,
	KBASE_PM_TEST_HOOK_PGD_MMU_AFTER_KCTX_DEREF,
	KBASE_PM_TEST_HOOK_PAGE_FREE_IN_PROGRESS,
};

#if MALI_UNIT_TEST
void kbase_page_migration_set_test_hook(void (*hook)
	(enum kbase_page_migration_test_hook_point hook_point, struct page *old_page));
void kbase_page_migration_test_hook(enum kbase_page_migration_test_hook_point hook_point,
				    struct page *old_page);

/*
 * kbase_migrate_page_allocated_mapped - Expose private function to migrate
 *                                       allocated mapped page for testing purposes.
 *
 * @old_page: Existing allocated mapped page to migrate.
 * @new_page: New page the existing page has to migrate to.
 *
 * Return: 0 if successful, otherwise error code.
 */
int kbase_migrate_page_allocated_mapped(struct page *old_page, struct page *new_page);
#else
#define kbase_page_migration_test_hook(hook_point, old_page) \
	do { \
		(void)(hook_point); \
		(void)(old_page); \
	} while (0)
#endif

#endif /* _KBASE_MEM_MIGRATE_H */
