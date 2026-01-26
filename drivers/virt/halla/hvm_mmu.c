// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/gfp.h>

#include "exynos-hvc.h"
#include "hvm_drv.h"

static int hvm_gfn_to_hva_memslot(struct hvm_memslot *memslot,
				  u64 gfn,
				  u64 *hva_memslot)
{
	u64 offset;

	if (gfn < memslot->base_gfn)
		return -EINVAL;

	offset = gfn - memslot->base_gfn;
	*hva_memslot = memslot->userspace_addr + offset * PAGE_SIZE;
	return 0;
}

static int cmp_ppages(struct rb_node *node, const struct rb_node *parent)
{
	struct hvm_pinned_page *a = container_of(node,
						  struct hvm_pinned_page,
						  node);
	struct hvm_pinned_page *b = container_of(parent,
						  struct hvm_pinned_page,
						  node);

	if (a->ipa < b->ipa)
		return -1;
	if (a->ipa > b->ipa)
		return 1;
	return 0;
}

/* Invoker of this function is responsible for locking */
static int hvm_insert_ppage(struct hvm *vm, struct hvm_pinned_page *ppage)
{
	if (rb_find_add(&ppage->node, &vm->pinned_pages, cmp_ppages))
		return -EEXIST;
	return 0;
}

static int pin_one_page(struct hvm *vm, unsigned long hva, u64 gpa,
			struct page **out_page)
{
	u32 flags = FOLL_HWPOISON | FOLL_LONGTERM | FOLL_WRITE;
	struct hvm_pinned_page *ppage = NULL;
	struct mm_struct *mm = current->mm;
	struct page *page = NULL;
	int ret;

	ppage = kmalloc(sizeof(*ppage), GFP_KERNEL_ACCOUNT);
	if (!ppage)
		return -ENOMEM;

	mmap_read_lock(mm);
	ret = pin_user_pages(hva, 1, flags, &page);
	mmap_read_unlock(mm);

	if (ret != 1 || !page) {
		kfree(ppage);
		return -EFAULT;
	}

	ppage->page = page;
	ppage->ipa = gpa;

	mutex_lock(&vm->mem_lock);
	ret = hvm_insert_ppage(vm, ppage);

	/**
	 * The return of -EEXIST from hvm_insert_ppage is considered an
	 * expected behavior in this context.
	 * This situation arises when two or more VCPUs are concurrently
	 * engaged in demand paging handling. The initial VCPU has already
	 * allocated and pinned a page, while the subsequent VCPU attempts
	 * to pin the same page again. As a result, we prompt the unpinning
	 * and release of the allocated structure, followed by a return -EAGAIN.
	 */
	if (ret == -EEXIST) {
		kfree(ppage);
		unpin_user_pages(&page, 1);
		mutex_unlock(&vm->mem_lock);
		return -EAGAIN;
	}
	mutex_unlock(&vm->mem_lock);
	*out_page = page;

	return ret;
}

void *hvm_alloc_pages(size_t size)
{
	return alloc_pages_exact(size, GFP_KERNEL | __GFP_ZERO);
}

void hvm_free_pages(void *virt, size_t size)
{
	free_pages_exact(virt, size);
}

static int hvm_register_vm_pgtable(struct hvm *hvm)
{
	unsigned long *buf;
	unsigned long buf_pa;
	size_t size = PAGE_SIZE * 2;
	unsigned long ret;

	buf = hvm_alloc_pages(size);
	if (buf == NULL)
		return -ENOMEM;

	buf_pa = virt_to_phys(buf);

	ret = exynos_hvc(HVC_FID_HVM_REGISTER_VM_PGTABLE,
			 hvm->vm_id, buf_pa, size, 0);
	if (ret) {
		hvm_free_pages(buf, size);
		return -EINVAL;
	}

	hvm->vm_pgtable[hvm->pgtable_num++] = buf;

	return 0;
}

int hvm_init_vm_stage2_mmu(struct hvm *hvm)
{
	unsigned long *buf;
	unsigned long buf_pa;
	unsigned long ret;
	u16 vm_id = hvm->vm_id;

	buf = hvm_alloc_pages(PAGE_SIZE);
	if (buf == NULL)
		return -ENOMEM;

	buf_pa = virt_to_phys(buf);

	ret = exynos_hvc(HVC_FID_HVM_INIT_STAGE2_MMU,
			 vm_id, buf_pa, PAGE_SIZE, 0);
	if (ret) {
		hvm_free_pages(buf, PAGE_SIZE);
		return -EINVAL;
	}

	hvm->s2table_buf = buf;
	hvm->pgtable_num = 0;

	return hvm_register_vm_pgtable(hvm);
}

static int hvm_vm_allocate_guest_page(struct hvm *vm, struct hvm_memslot *slot,
				u64 gfn, u64 *pfn)
{
	struct page *page = NULL;
	unsigned long hva;
	int ret;

	if (hvm_gfn_to_hva_memslot(slot, gfn, (u64 *)&hva) != 0)
		return -EINVAL;

	ret = pin_one_page(vm, hva, PFN_PHYS(gfn), &page);
	if (ret != 0)
		return ret;

	if (page == NULL)
		return -EFAULT;
	/**
	 * As `pin_user_pages` already gets the page struct, we don't need to
	 * call other APIs to reduce function call overhead.
	 */
	*pfn = page_to_pfn(page);

	return 0;
}

static int hvm_alloc_vm_pgtable(struct hvm *hvm)
{
	if (!exynos_hvc(HVC_FID_HVM_NEED_VM_PGTABLE,
			hvm->vm_id, 0, 0, 0))
		return 0;

	return hvm_register_vm_pgtable(hvm);
}

static int hvm_map_guest(struct hvm *hvm, int memslot_id,
		  u64 pfn, u64 gfn, u64 nr_pages)
{
	unsigned long ret;
	u64 ids;

	ret = hvm_alloc_vm_pgtable(hvm);
	if (ret)
		return (int)ret;

	ids = (u32)memslot_id;
	ids = (ids << 32) | hvm->vm_id;

	ret = exynos_hvc(HVC_FID_HVM_MAP_GUEST, ids, pfn, gfn, nr_pages);

	return (int)ret;
}

static int handle_demand_page(struct hvm *vm, int memslot_id, u64 gfn)
{
	int ret;
	u64 pfn;

	ret = hvm_vm_allocate_guest_page(vm, &vm->memslot[memslot_id], gfn, &pfn);
	if (unlikely(ret)) {
		if (ret == -EAGAIN)
			return 0;
		else
			return -EFAULT;
	}

	ret = hvm_map_guest(vm, memslot_id, pfn, gfn, 1);
	if (unlikely(ret))
		return -EFAULT;

	return ret;
}

/**
 * hvm_find_memslot() - Find memslot containing this @gpa
 * @vm: Pointer to struct hvm
 * @gfn: Guest frame number
 *
 * Return:
 * * >=0		- Index of memslot
 * * -EFAULT		- Not found
 */
static int hvm_find_memslot(struct hvm *vm, u64 gfn)
{
	int i;

	for (i = 0; i < HVM_MAX_MEM_REGION; i++) {
		if (vm->memslot[i].npages == 0)
			continue;

		if (gfn >= vm->memslot[i].base_gfn &&
		    gfn < vm->memslot[i].base_gfn + vm->memslot[i].npages)
			return i;
	}

	return -EFAULT;
}

/**
 * hvm_handle_page_fault() - Handle guest page fault, find corresponding page
 *                            for the faulting gpa
 * @vcpu: Pointer to struct hvm_vcpu_run of the faulting vcpu
 *
 * Return:
 * * 0		- Success to handle guest page fault
 * * -EFAULT	- Failed to map phys addr to guest's GPA
 */
int hvm_handle_page_fault(struct hvm_vcpu *vcpu)
{
	struct hvm *vm = vcpu->hvm;
	int memslot_id;
	u64 gfn;

	gfn = PHYS_PFN(vcpu->run->exception.fault_gpa);
	memslot_id = hvm_find_memslot(vm, gfn);
	if (unlikely(memslot_id < 0))
		return -EFAULT;

	return handle_demand_page(vm, memslot_id, gfn);
}
