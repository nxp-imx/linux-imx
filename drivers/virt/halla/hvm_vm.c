// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 */

#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/kdev_t.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/debugfs.h>

#include "exynos-hvc.h"
#include "hvm_drv.h"

static DEFINE_MUTEX(hvm_list_lock);
static LIST_HEAD(hvm_list);

/**
 * register_memslot_addr_range() - Register memory region to Halla
 * @hvm: Pointer to struct hvm
 * @memslot: Pointer to struct hvm_memslot
 *
 * Return: 0 for success, negative number for error
 */
static int
register_memslot_addr_range(struct hvm *hvm, struct hvm_memslot *memslot)
{
	struct hvm_memory_region_ranges *region;
	u32 buf_size = PAGE_SIZE * 2;
	u64 gfn;
	unsigned long ret;

	region = alloc_pages_exact(buf_size, GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	region->slot = memslot->slot_id;
	region->total_pages = memslot->npages;
	gfn = memslot->base_gfn;
	region->gpa = PFN_PHYS(gfn);

	ret = exynos_hvc(HVC_FID_HVM_SET_MEMREGION,
			 hvm->vm_id, buf_size,
			 virt_to_phys(region), 0);
	if (ret) {
		free_pages_exact(region, buf_size);
		return -EFAULT;
	}

	free_pages_exact(region, buf_size);

	return 0;
}

/**
 * memregion_validity_check() - Preliminary check for memory region
 * @hvm: Pointer to struct hvm.
 * @mem: Input memory region from user.
 *
 * Return: true for check passed, false for invalid input.
 */
static bool
memregion_validity_check(struct hvm *hvm,
			struct hvm_userspace_memory_region *mem)
{
	if (mem->slot >= HVM_MAX_MEM_REGION)
		return false;

	if (!PAGE_ALIGNED(mem->guest_phys_addr) ||
	    !PAGE_ALIGNED(mem->memory_size))
		return false;

	if (mem->guest_phys_addr + mem->memory_size < mem->guest_phys_addr)
		return false;

	if ((mem->memory_size >> PAGE_SHIFT) > HVM_MEM_MAX_NR_PAGES)
		return false;

	return true;
}

/**
 * hvm_vm_ioctl_set_memory_region() - Set memory region of guest
 * @hvm: Pointer to struct hvm.
 * @mem: Input memory region from user.
 *
 * Return: 0 for success, negative number for error
 *
 * -EXIO		- The memslot is out-of-range
 * -EFAULT		- Cannot find corresponding vma
 * -EINVAL		- Region size and VMA size mismatch
 */
static int
hvm_vm_ioctl_set_memory_region(struct hvm *hvm,
				struct hvm_userspace_memory_region *mem)
{
	int ret;
	struct vm_area_struct *vma;
	struct hvm_memslot *memslot;
	unsigned long size;

	if (memregion_validity_check(hvm, mem) != true)
		return -EINVAL;

	memslot = &hvm->memslot[mem->slot];

	vma = vma_lookup(hvm->mm, mem->userspace_addr);
	if (!vma)
		return -EFAULT;

	size = vma->vm_end - vma->vm_start;
	if (size != mem->memory_size)
		return -EINVAL;

	memslot->base_gfn = __phys_to_pfn(mem->guest_phys_addr);
	memslot->npages = size >> PAGE_SHIFT;
	memslot->userspace_addr = mem->userspace_addr;
	memslot->vma = vma;
	memslot->flags = mem->flags;
	memslot->slot_id = mem->slot;

	ret = (int)exynos_hvc(HVC_FID_HVM_MEMREGION_PURPOSE,
			hvm->vm_id, mem->guest_phys_addr,
			mem->memory_size, mem->flags);
	if (ret)
		return -EFAULT;

	return register_memslot_addr_range(hvm, memslot);
}

/**
 * hvm_inject_irq() - Inject virtual interrupt to a VM
 * @hvm: Pointer to struct hvm
 * @vcpu_idx: vcpu index, only valid if PPI
 * @irq: *SPI* irq number (excluding offset value `32`)
 * @level: 1 if true else 0
 *
 * Return:
 * * 0			- Success.
 * * Negative		- Failure.
 */
int hvm_inject_irq(struct hvm *hvm, unsigned int vcpu_idx,
		   u32 irq, bool level)
{
	unsigned long ret;
	u64 ids;
	struct hvm_vcpu *vcpu = hvm->vcpus[vcpu_idx];

	ids = (u32)hvm->vm_id;
	ids = (ids << 16) | vcpu_idx;

	ret = exynos_hvc(HVC_FID_HVM_IRQ_LINE,
			 ids, irq, level, 0);

	hvm_vcpu_wakeup(vcpu);

	return (int)ret;
}

static int hvm_vm_ioctl_irq_line(struct hvm *hvm,
				  struct hvm_irq_level *irq_level)
{
	u32 irq = irq_level->irq;
	u32 vcpu_idx, vcpu2_idx, irq_num;
	bool level = irq_level->level;

	vcpu_idx = FIELD_GET(HVM_IRQ_LINE_VCPU, irq);
	vcpu2_idx = FIELD_GET(HVM_IRQ_LINE_VCPU2, irq) * (HVM_IRQ_VCPU_MASK + 1);
	irq_num = FIELD_GET(HVM_IRQ_LINE_NUM, irq);

	return hvm_inject_irq(hvm, vcpu_idx + vcpu2_idx, irq_num,
				       level);
}

static int hvm_vm_ioctl_create_device(struct hvm *hvm, void __user *argp)
{
	struct hvm_create_device *hvm_dev;
	void *dev_data = NULL;
	int ret;

	hvm_dev = alloc_pages_exact(PAGE_SIZE, GFP_KERNEL);
	if (!hvm_dev)
		return -ENOMEM;

	if (copy_from_user(hvm_dev, argp, sizeof(struct hvm_create_device))) {
		ret = -EFAULT;
		goto err_free_dev;
	}

	if (hvm_dev->attr_addr != 0 && hvm_dev->attr_size != 0) {
		size_t attr_size = hvm_dev->attr_size;
		void __user *attr_addr = u64_to_user_ptr(hvm_dev->attr_addr);

		/* Size of device specific data should not be over a page. */
		if (attr_size > PAGE_SIZE) {
			ret = -EINVAL;
			goto err_free_dev;
		}

		dev_data = alloc_pages_exact(attr_size, GFP_KERNEL);
		if (!dev_data) {
			ret = -ENOMEM;
			goto err_free_dev;
		}

		if (copy_from_user(dev_data, attr_addr, attr_size)) {
			ret = -EFAULT;
			goto err_free_dev_data;
		}
		hvm_dev->attr_addr = virt_to_phys(dev_data);
	}

	ret = (int)exynos_hvc(HVC_FID_HVM_CREATE_DEVICE,
			hvm->vm_id, virt_to_phys(hvm_dev), 0, 0);
err_free_dev_data:
	if (dev_data)
		free_pages_exact(dev_data, 0);
err_free_dev:
	free_pages_exact(hvm_dev, 0);
	return ret;
}

static int hvm_vm_enable_cap(struct hvm *hvm,
			     struct hvm_enable_cap *cap,
			     struct arm_smccc_res *res)
{
	arm_smccc_hvc(HVC_FID_HVM_ENABLE_CAP,
			 hvm->vm_id, cap->cap, cap->args[0], cap->args[1], 0, 0, 0, res);

	return res->a0;
}

static int hvm_vm_ioctl_enable_cap(struct hvm *hvm,
				   struct hvm_enable_cap *cap,
				   void __user *argp)
{
	struct arm_smccc_res res = {0};
	int ret;

	/* Currently, only pvm-related capabilities exist */
	if (cap->cap != HVM_CAP_PROTECTED_VM)
		return -EINVAL;

	switch (cap->args[0]) {
	case HVM_CAP_PVM_SET_PVMFW_GPA:
		fallthrough;
	case HVM_CAP_PVM_SET_PROTECTED_VM:
		ret = hvm_vm_enable_cap(hvm, cap, &res);
		return ret;
	case HVM_CAP_PVM_GET_PVMFW_SIZE: {
		ret = hvm_vm_enable_cap(hvm, cap, &res);
		if (ret)
			return -EINVAL;
		cap->args[1] = res.a1;
		if (copy_to_user(argp, cap, sizeof(*cap)))
			return -EFAULT;

		return 0;
	}
	default:
		break;
	}

	return -EINVAL;
}

static int set_dtb_config(struct hvm *hvm, struct hvm_dtb_config *cfg)
{
	unsigned long ret;

	ret = exynos_hvc(HVC_FID_HVM_SET_DTB_CONFIG,
			 hvm->vm_id, cfg->dtb_addr, cfg->dtb_size, 0);

	return (int)ret;
}

static unsigned long hvm_pre_destory_vm(struct hvm *hvm)
{
	return exynos_hvc(HVC_FID_HVM_PRE_DESTROY_VM,
			  hvm->vm_id, 0, 0, 0);
}

static unsigned long
__hvm_reclaim_guest_memory(u16 vmid, u64 gfn, unsigned long nr_pages)
{
	phys_addr_t ipa = PFN_PHYS(gfn);
	unsigned long left_pages = nr_pages;
	unsigned long min_pages = HVM_RECLAIM_MIN_SIZE >> PAGE_SHIFT;
	unsigned long order = min_pages;
	unsigned long ret = 0;

	while (left_pages > 0) {
		if (order > left_pages)
			order = left_pages;

		ret = exynos_hvc(HVC_FID_HVM_RECLAIM_GUEST_MEMORY,
				 vmid, ipa, order, 0);
		if (ret)
			break;

		left_pages -= order;
		ipa += (order << PAGE_SHIFT);
	}

	return ret;
}

static unsigned long hvm_memslot_reclaim_guest_memory(struct hvm *hvm)
{
	struct hvm_memslot *slot;
	unsigned long ret = 0;

	for (unsigned int i = 0; i < HVM_MAX_MEM_REGION; i++) {
		slot = &hvm->memslot[i];

		if (slot->npages == 0)
			continue;

		ret = __hvm_reclaim_guest_memory(hvm->vm_id,
						 slot->base_gfn,
						 slot->npages);
		if (ret)
			break;
	}
	return ret;
}

static unsigned long hvm_reclaim_guest_memory(struct hvm *hvm)
{
	return hvm_memslot_reclaim_guest_memory(hvm);
}

static void hvm_reclaim_guest_stage2_pgtable(struct hvm *hvm)
{
	unsigned long ret;

	ret = exynos_hvc(HVC_FID_HVM_RECLAIM_GUEST_S2_PGTABLE,
			 hvm->vm_id, 0, 0, 0);
	if (ret)
		return;

	hvm_free_pages(hvm->s2table_buf, PAGE_SIZE);

	for (unsigned int i = 0; i < hvm->pgtable_num; i++)
		hvm_free_pages(hvm->vm_pgtable[i], PAGE_SIZE * 2);
}

/* Invoker of this function is responsible for locking */
static void hvm_destroy_all_ppage(struct hvm *hvm)
{
	struct hvm_pinned_page *ppage;
	struct rb_node *node;

	node = rb_first(&hvm->pinned_pages);
	while (node) {
		ppage = rb_entry(node, struct hvm_pinned_page, node);
		unpin_user_pages_dirty_lock(&ppage->page, 1, true);
		node = rb_next(node);
		rb_erase(&ppage->node, &hvm->pinned_pages);
		kfree(ppage);
	}
}

static void hvm_destroy_vm(struct hvm *hvm)
{
	unsigned long ret = 0;

	mutex_lock(&hvm->lock);

	hvm_pre_destory_vm(hvm);

	hvm_vm_irqfd_release(hvm);

	ret = hvm_reclaim_guest_memory(hvm);
	hvm_reclaim_guest_stage2_pgtable(hvm);

	hvm_destroy_vcpus(hvm);

	exynos_hvc(HVC_FID_HVM_DESTROY_VM,
		   hvm->vm_id, 0, 0, 0);

	hvm_destroy_vm_debugfs(hvm);

	mutex_lock(&hvm_list_lock);
	list_del(&hvm->vm_list);
	mutex_unlock(&hvm_list_lock);

	mutex_unlock(&hvm->lock);

	/**
	 * Have to leave the page pinned as we don't know whether its
	 * save to access
	 */
	if (!ret) {
		/**
		 * No need to lock here becauese it's single-threaded
		 * execution
		 */
		hvm_destroy_all_ppage(hvm);
	}

	kfree(hvm);
}

void hvm_destroy_all_vms(void)
{
	struct hvm *hvm, *tmp;

	mutex_lock(&hvm_list_lock);
	if (list_empty(&hvm_list))
		goto out;

	list_for_each_entry_safe(hvm, tmp, &hvm_list, vm_list)
		hvm_destroy_vm(hvm);

out:
	mutex_unlock(&hvm_list_lock);
}

int hvm_check_extension(struct hvm *hvm, u64 cap, void __user *argp)
{
	switch (cap) {
	case HVM_CAP_PROTECTED_VM: {
		u64 success = 1;

		if (copy_to_user(argp, &success, sizeof(__u64)))
			return -EFAULT;

		return 0;
	}
	case HVM_CAP_VM_GPA_SIZE: {
		u64 value = CONFIG_ARM64_PA_BITS;

		if (copy_to_user(argp, &value, sizeof(__u64)))
			return -EFAULT;

		return 0;
	}
	default:
		break;
	}

	return -EOPNOTSUPP;
}

/* hvm_vm_ioctl() - Ioctl handler of VM FD */
static long hvm_vm_ioctl(struct file *filp, unsigned int ioctl,
			  unsigned long arg)
{
	long ret;
	void __user *argp = (void __user *)arg;
	struct hvm *hvm = filp->private_data;

	switch (ioctl) {
	case HVM_CHECK_EXTENSION: {
		u64 cap;

		if (copy_from_user(&cap, argp, sizeof(__u64)))
			return -EFAULT;

		ret = hvm_check_extension(hvm, cap, argp);
		break;
	}
	case HVM_CREATE_VCPU: {
		ret = hvm_vm_ioctl_create_vcpu(hvm, arg);
		break;
	}
	case HVM_SET_USER_MEMORY_REGION: {
		struct hvm_userspace_memory_region userspace_mem;

		if (copy_from_user(&userspace_mem, argp, sizeof(userspace_mem)))
			return -EFAULT;

		ret = hvm_vm_ioctl_set_memory_region(hvm, &userspace_mem);
		break;
	}
	case HVM_IRQ_LINE: {
		struct hvm_irq_level irq_level;

		if (copy_from_user(&irq_level, argp, sizeof(irq_level))) {
			ret = -EFAULT;
			goto out;
		}
		ret = hvm_vm_ioctl_irq_line(hvm, &irq_level);
		break;
	}
	case HVM_CREATE_DEVICE: {
		ret = hvm_vm_ioctl_create_device(hvm, argp);
		break;
	}
	case HVM_IRQFD: {
		struct hvm_irqfd data;

		if (copy_from_user(&data, argp, sizeof(data))) {
			ret = -EFAULT;
			goto out;
		}
		ret = hvm_irqfd(hvm, &data);
		break;
	}
	case HVM_IOEVENTFD: {
		struct hvm_ioeventfd data;

		if (copy_from_user(&data, argp, sizeof(data))) {
			ret = -EFAULT;
			goto out;
		}
		ret = hvm_ioeventfd(hvm, &data);
		break;
	}
	case HVM_ENABLE_CAP: {
		struct hvm_enable_cap cap;

		if (copy_from_user(&cap, argp, sizeof(cap))) {
			ret = -EFAULT;
			goto out;
		}
		ret = hvm_vm_ioctl_enable_cap(hvm, &cap, argp);
		break;
	}
	case HVM_SET_DTB_CONFIG: {
		struct hvm_dtb_config cfg;

		if (copy_from_user(&cfg, argp, sizeof(cfg))) {
			ret = -EFAULT;
			goto out;
		}
		ret = set_dtb_config(hvm, &cfg);
		break;
	}
	default:
		ret = -ENOTTY;
	}
out:
	return ret;
}

static int hvm_vm_release(struct inode *inode, struct file *filp)
{
	struct hvm *hvm = filp->private_data;

	hvm_destroy_vm(hvm);

	return 0;
}

static const struct file_operations hvm_vm_fops = {
	.release        = hvm_vm_release,
	.unlocked_ioctl = hvm_vm_ioctl,
};

static int hvm_arch_drv_init(struct hvm *hvm)
{
	clocks_calc_mult_shift(&hvm->clock_scale_factor.mult,
			&hvm->clock_scale_factor.shift,
			arch_timer_get_cntfrq(),
			NSEC_PER_SEC,
			0);

	return 0;
}

/**
 * hvm_dev_ioctl_create_vm - Create vm fd
 * @vm_type: VM type. Only supports Linux VM now.
 *
 * Return: fd of vm, negative if error
 */
int hvm_dev_ioctl_create_vm(unsigned long vm_type)
{
	struct hvm *hvm;
	int ret;

	hvm = kzalloc(sizeof(struct hvm), GFP_KERNEL);
	if (!hvm)
		return -ENOMEM;

	ret = exynos_hvc(HVC_FID_HVM_CREATE_VM, vm_type, 0, 0, 0);
	if (ret == 0) {
		kfree(hvm);
		return -ENOMEM;
	}

	hvm->vm_id = ret;
	hvm->mm = current->mm;
	mutex_init(&hvm->lock);
	mutex_init(&hvm->mem_lock);
	hvm->pinned_pages = RB_ROOT;

	ret = hvm_vm_irqfd_init(hvm);
	if (ret) {
		kfree(hvm);
		return ret;
	}

	ret = hvm_init_ioeventfd(hvm);
	if (ret) {
		kfree(hvm);
		return ret;
	}

	ret = hvm_init_vm_stage2_mmu(hvm);
	if (ret) {
		kfree(hvm);
		return ret;
	}

	ret = hvm_create_vm_debugfs(hvm);
	if (ret) {
		kfree(hvm);
		return ret;
	}

	ret = hvm_arch_drv_init(hvm);
	if (ret) {
		kfree(hvm);
		return ret;
	}

	mutex_lock(&hvm_list_lock);
	list_add(&hvm->vm_list, &hvm_list);
	mutex_unlock(&hvm_list_lock);

	return anon_inode_getfd("hvm-vm", &hvm_vm_fops, hvm,
			       O_RDWR | O_CLOEXEC);
}
