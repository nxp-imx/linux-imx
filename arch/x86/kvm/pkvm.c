// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/kvm_host.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <asm/e820/api.h>
#include <asm/kvm_pkvm.h>

/* Kernel command-line parameter */
bool __read_mostly enable_pkvm;

/* Flag denoting pKVM successfully initialized */
DEFINE_STATIC_KEY_FALSE(pkvm_enabled_key);

static struct memblock_region *pkvm_memory = pkvm_sym(pkvm_memory);
static unsigned int pkvm_memblock_nr;

phys_addr_t pkvm_mem_base;
phys_addr_t pkvm_mem_size;

bool pvmfw_present;
phys_addr_t pvmfw_base;
phys_addr_t pvmfw_size;

static int cmp_pkvm_memblock(const void *p1, const void *p2)
{
	const struct memblock_region *r1 = p1;
	const struct memblock_region *r2 = p2;

	return r1->base < r2->base ? -1 : (r1->base > r2->base);
}

static void __init sort_memblock_regions(void)
{
	sort(pkvm_memory,
	     pkvm_memblock_nr,
	     sizeof(struct memblock_region),
	     cmp_pkvm_memblock,
	     NULL);
}

static int __init register_memblock_regions(void)
{
	struct memblock_region *reg;

	for_each_mem_region(reg) {
		if (pkvm_memblock_nr >= PKVM_MEMBLOCK_REGIONS)
			return -ENOMEM;

		pkvm_memory[pkvm_memblock_nr] = *reg;
		pkvm_memblock_nr++;
	}
	sort_memblock_regions();
	pkvm_sym(pkvm_memblock_nr) = pkvm_memblock_nr;

	return 0;
}

void __init pkvm_reserve(void)
{
	int ret;

	if (!enable_pkvm)
		return;

	ret = register_memblock_regions();
	if (ret) {
		pkvm_memblock_nr = 0;
		kvm_err("Failed to register pkvm memblocks: %d\n", ret);
		return;
	}

	/*
	 * Try to allocate a PMD-aligned region to reduce TLB pressure once
	 * this is unmapped from the host stage-2, and fallback to PAGE_SIZE.
	 */
	pkvm_mem_size = pkvm_total_reserve_pages() << PAGE_SHIFT;
	pkvm_mem_base = memblock_phys_alloc(ALIGN(pkvm_mem_size, PMD_SIZE),
					   PMD_SIZE);
	if (!pkvm_mem_base)
		pkvm_mem_base = memblock_phys_alloc(pkvm_mem_size, PAGE_SIZE);
	else
		pkvm_mem_size = ALIGN(pkvm_mem_size, PMD_SIZE);

	if (!pkvm_mem_base) {
		kvm_err("Failed to reserve pkvm memory\n");
		return;
	}

	kvm_info("Reserved %lld MiB at 0x%llx for pkvm\n", pkvm_mem_size >> 20,
		 pkvm_mem_base);
}

static phys_addr_t kvm_host_pa(void *addr)
{
	return __pa(addr);
}

static void *kvm_host_va(phys_addr_t phys)
{
	return __va(phys);
}

static void *kvm_alloc_pkvm_page(void *flags)
{
	void *addr = (void *)__get_free_page(GFP_KERNEL_ACCOUNT);

	if (addr && (unsigned long)flags & PKVM_MC_ACCOUNT_PGTABLE_PAGES)
		kvm_account_pgtable_pages(addr, 1);

	return addr;
}

static void kvm_free_pkvm_page_range(struct pkvm_page_range range, void *flags)
{
	void *vaddr = __va(range.addr);
	u64 nr_pages = range.nr_pages;

	if (WARN_ON_ONCE(!nr_pages))
		return;

	if ((unsigned long)flags & PKVM_MC_ACCOUNT_PGTABLE_PAGES)
		kvm_account_pgtable_pages(vaddr, -nr_pages);

	if (nr_pages > 1)
		free_pages_exact(vaddr, nr_pages << PAGE_SHIFT);
	else
		free_page((unsigned long)vaddr);
}

int kvm_topup_pkvm_memcache(struct pkvm_memcache *mc, unsigned long min_pages)
{
	return topup_pkvm_memcache(mc, min_pages, kvm_alloc_pkvm_page,
				   kvm_host_pa, (void *)mc->flags);
}

void kvm_free_pkvm_memcache(struct pkvm_memcache *mc)
{
	free_pkvm_memcache(mc, kvm_free_pkvm_page_range, kvm_host_va,
			   (void *)mc->flags);
}

static int pkvm_vm_ioctl_set_fw_gpa(struct kvm *kvm, u64 gpa)
{
	struct kvm_pkvm_vm *pkvm = &kvm->arch.pkvm;
	int ret = 0;

	if (!pvmfw_present)
		return -EINVAL;

	mutex_lock(&pkvm->finalized_lock);
	if (pkvm->finalized) {
		ret = -EBUSY;
		goto out;
	}
	pkvm->pvmfw_load_addr = gpa;
out:
	mutex_unlock(&pkvm->finalized_lock);
	return ret;
}

static int pkvm_vm_ioctl_info(struct kvm *kvm,
			      struct kvm_protected_vm_info __user *info)
{
	struct kvm_protected_vm_info kinfo = {
		.firmware_size = pvmfw_present ? pvmfw_size : 0,
	};

	return copy_to_user(info, &kinfo, sizeof(kinfo)) ? -EFAULT : 0;
}

int pkvm_vm_ioctl_enable_cap(struct kvm *kvm, struct kvm_enable_cap *cap)
{
	if (!pkvm_is_protected_vm(kvm))
		return -EINVAL;

	if (cap->args[1] || cap->args[2] || cap->args[3])
		return -EINVAL;

	switch (cap->flags) {
	case KVM_CAP_X86_PROTECTED_VM_FLAGS_SET_FW_GPA:
		return pkvm_vm_ioctl_set_fw_gpa(kvm, cap->args[0]);
	case KVM_CAP_X86_PROTECTED_VM_FLAGS_INFO:
		return pkvm_vm_ioctl_info(kvm, (void __force __user *)cap->args[0]);
	default:
		return -EINVAL;
	}
}

struct pkvm_ramoops_console_info {
	phys_addr_t start;
	size_t size;
};

/*
 * Console offset needs to be calculated manually since ACPI only provides the
 * overall region boundaries. Offset of the Console is effectively the size of
 * Dmesg area.
 *
 * Below definitions based on drivers/platform/chrome/chromeos_pstore.c:
 */
#define GOOG9999_RAMOOPS_PMSG_SIZE    0x20000
#define GOOG9999_RAMOOPS_FTRACE_SIZE  0x20000
#define GOOG9999_RAMOOPS_CONSOLE_SIZE 0x20000

#define GOOG9999_RAMOOPS_NON_DMESG_SIZE \
	(GOOG9999_RAMOOPS_PMSG_SIZE + \
	 GOOG9999_RAMOOPS_FTRACE_SIZE + \
	 GOOG9999_RAMOOPS_CONSOLE_SIZE)

/* The combined size of the fixed partitions at the end of the region */
#define GOOG9999_RAMOOPS_DMESG_SIZE(total_size) \
	((total_size) - GOOG9999_RAMOOPS_NON_DMESG_SIZE)

static void update_cros_ramoops_console_info(struct pkvm_ramoops_console_info *info,
					     struct resource_entry *rentry)
{
	size_t console_offset;
	size_t ramoops_size = resource_size(rentry->res);

	/*
	 * Note: The ChromeOS ramoops layout (for GOOG9999) partitions the
	 * memory region as:
	 * [Dmesg Area] [Console (128KB)] [Pmsg (128KB)] [Ftrace (128KB)]
	 *
	 * Since ramoops must recover logs across hardware resets and different
	 * kernel versions, this layout should be stable and has remained
	 * invariant for over a decade (chromeos_pstore.c's
	 * chromeos_ramoops_data hasn't changed for a decade).
	 *
	 * The console buffer is located 3 slots (384KB) before the end of the
	 * total region.
	 */
	if (ramoops_size >= GOOG9999_RAMOOPS_NON_DMESG_SIZE) {
		console_offset = GOOG9999_RAMOOPS_DMESG_SIZE(ramoops_size);
		info->start = rentry->res->start + console_offset;
		info->size = GOOG9999_RAMOOPS_CONSOLE_SIZE;
	}
}

static int find_cros_ramoops(struct pkvm_ramoops_console_info *info)
{
	struct acpi_device *adev;
	LIST_HEAD(resource_list);
	struct resource_entry *rentry;
	int ret = -ENODEV;

	adev = acpi_dev_get_first_match_dev("GOOG9999", NULL, -1);
	if (!adev)
		return ret;

	if (acpi_dev_get_resources(adev, &resource_list, NULL, NULL) < 0)
		goto out;

	list_for_each_entry(rentry, &resource_list, node) {
		if (resource_type(rentry->res) == IORESOURCE_MEM) {
			update_cros_ramoops_console_info(info, rentry);
			ret = 0;
			break;
		}
	}
	acpi_dev_free_resource_list(&resource_list);

out:
	acpi_dev_put(adev);

	return ret;
}

static void find_ramoops(struct pkvm_ramoops_console_info *info)
{
	/*
	 * Try chromeos pstore ramoops first.
	 *
	 * TODO: currently supporting only ACPI GOOG9999 chromeos_pstore but it can
	 * be extended to other pstores like ramoops cmdline options.
	 */
	if (!find_cros_ramoops(info))
		return;
}

void __init pkvm_ramoops_init(void)
{
	struct pkvm_ramoops_console_info r_info = {};

	find_ramoops(&r_info);
	if (r_info.start && r_info.size) {
		phys_addr_t end = r_info.start + r_info.size - 1;
		phys_addr_t pvmfw_end = pvmfw_base + pvmfw_size - 1;

		if (pvmfw_present && (r_info.start <= pvmfw_end) && (pvmfw_base <= end)) {
			pr_err("ramoops [0x%llx-0x%llx] overlaps with pvmfw region\n",
			       r_info.start, end);
			return;
		}

		if (!e820__mapped_all(r_info.start, end, E820_TYPE_RESERVED)) {
			pr_err("ramoops [0x%llx-0x%llx] is not reserved in e820\n",
			       r_info.start, end);
			return;
		}

		if (!PAGE_ALIGNED(r_info.start) || !PAGE_ALIGNED(r_info.size)) {
			pr_err("ramoops [0x%llx-0x%llx] is not page-aligned\n",
			       r_info.start, end);
			return;
		}

		pkvm_sym(pkvm_ramoops_console_pa) = r_info.start;
		pkvm_sym(pkvm_ramoops_console_size) = r_info.size;
	}
}
