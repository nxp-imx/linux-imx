// SPDX-License-Identifier: GPL-2.0

#include <linux/kvm_para.h>
#include <linux/virtio_balloon.h>
#include <asm/coco.h>
#include <asm/pkvm_guest.h>
#include <asm/pgtable.h>
#include <asm/apic.h>

DEFINE_STATIC_KEY_FALSE(pkvm_guest_detected);
EXPORT_SYMBOL(pkvm_guest_detected);

int pkvm_set_mem_host_visibility(unsigned long addr, int numpages, bool enc)
{
	unsigned long size = numpages * PAGE_SIZE;
	int ret;

	if (!enc) {
		int i;

		/*
		 * If the guest has never touched these pages before, they have
		 * not been donated to the guest yet, i.e. are still owned by
		 * the host. In such case the sharing will fail.
		 *
		 * So touch these pages first, to make sure they are owned by
		 * the guest before sharing.
		 */
		for (i = 0; i < numpages; i++)
			READ_ONCE(*(u8 *)(addr + i * PAGE_SIZE));

		/*
		 * pKVM may not have enough memory to perform the sharing. In such
		 * case it requests the host to provide more memory, and then may
		 * ask the guest to retry the hypercall.
		 */
		do {
			ret = kvm_hypercall2(PKVM_GHC_SHARE_MEM, __pa(addr), size);
		} while (ret == -EAGAIN);
	} else {
		ret = kvm_hypercall2(PKVM_GHC_UNSHARE_MEM, __pa(addr), size);
	}

	return ret;
}

static int pkvm_virt_mmio(int size, bool write, unsigned long vaddr, unsigned long *val)
{
	unsigned long paddr;
	pte_t *pte;
	int level;

	pte = lookup_address(vaddr, &level);
	if (WARN_ON_ONCE(!pte))
		return -EIO;

	paddr = (pte_pfn(*pte) << PAGE_SHIFT) | (vaddr & ~page_level_mask(level));

	if (write)
		kvm_hypercall3(PKVM_GHC_IOWRITE, paddr, size, *val);
	else
		*val = kvm_hypercall2(PKVM_GHC_IOREAD, paddr, size);

	return 0;
}

static unsigned char pkvm_mmio_readb(const volatile void __iomem *addr)
{
	unsigned long val;

	if (pkvm_virt_mmio(1, false, (unsigned long)addr, &val))
		return 0xff;
	return val;
}

static unsigned short pkvm_mmio_readw(const volatile void __iomem *addr)
{
	unsigned long val;

	if (pkvm_virt_mmio(2, false, (unsigned long)addr, &val))
		return 0xffff;
	return val;
}

static unsigned int pkvm_mmio_readl(const volatile void __iomem *addr)
{
	unsigned long val;

	if (pkvm_virt_mmio(4, false, (unsigned long)addr, &val))
		return 0xffffffff;
	return val;
}

static u64 pkvm_mmio_readq(const volatile void __iomem *addr)
{
	unsigned long val;

	if (pkvm_virt_mmio(8, false, (unsigned long)addr, &val))
		return 0xffffffffffffffff;
	return val;
}

static void pkvm_mmio_writeb(unsigned char v, volatile void __iomem *addr)
{
	unsigned long val = v;

	pkvm_virt_mmio(1, true, (unsigned long)addr, &val);
}

static void pkvm_mmio_writew(unsigned short v, volatile void __iomem *addr)
{
	unsigned long val = v;

	pkvm_virt_mmio(2, true, (unsigned long)addr, &val);
}

static void pkvm_mmio_writel(unsigned int v, volatile void __iomem *addr)
{
	unsigned long val = v;

	pkvm_virt_mmio(4, true, (unsigned long)addr, &val);
}

static void pkvm_mmio_writeq(u64 v, volatile void __iomem *addr)
{
	unsigned long val = v;

	pkvm_virt_mmio(8, true, (unsigned long)addr, &val);
}

static int pkvm_wakeup_secondary_cpu(u32 apic_id, unsigned long start_ip, unsigned int cpu)
{
	return kvm_hypercall2(PKVM_GHC_START_CPU, apic_id, start_ip);
}

#ifdef CONFIG_VIRTIO_BALLOON_HYP_OPS
static bool pkvm_page_relinquish_disallowed(void)
{
	/*
	 * pKVM-x86 does not support balloon for protected guests yet,
	 * so relinquish is always disallowed.
	 */
	return true;
}

static void pkvm_page_relinquish(struct page *page, unsigned int nr)
{
	WARN_ON_ONCE(1);
}

static struct virtio_balloon_hyp_ops pkvm_virtio_balloon_hyp_ops = {
	.page_relinquish_disallowed = pkvm_page_relinquish_disallowed,
	.page_relinquish = pkvm_page_relinquish,
};
#endif

__init void pkvm_guest_init_coco(void)
{
	cc_vendor = CC_VENDOR_PKVM;

	static_branch_enable(&pkvm_guest_detected);

	pv_ops.mmio.raw_readb = pkvm_mmio_readb;
	pv_ops.mmio.raw_readw = pkvm_mmio_readw;
	pv_ops.mmio.raw_readl = pkvm_mmio_readl;
	pv_ops.mmio.raw_readb_relaxed = pkvm_mmio_readb;
	pv_ops.mmio.raw_readw_relaxed = pkvm_mmio_readw;
	pv_ops.mmio.raw_readl_relaxed = pkvm_mmio_readl;
	pv_ops.mmio.raw_writeb = pkvm_mmio_writeb;
	pv_ops.mmio.raw_writew = pkvm_mmio_writew;
	pv_ops.mmio.raw_writel = pkvm_mmio_writel;
	pv_ops.mmio.raw_writeb_relaxed = pkvm_mmio_writeb;
	pv_ops.mmio.raw_writew_relaxed = pkvm_mmio_writew;
	pv_ops.mmio.raw_writel_relaxed = pkvm_mmio_writel;
#ifdef CONFIG_X86_64
	pv_ops.mmio.raw_readq = pkvm_mmio_readq;
	pv_ops.mmio.raw_readq_relaxed = pkvm_mmio_readq;
	pv_ops.mmio.raw_writeq = pkvm_mmio_writeq;
	pv_ops.mmio.raw_writeq_relaxed = pkvm_mmio_writeq;
#endif
	pv_ops.mmio.pci_mmcfg_readb = pkvm_mmio_readb;
	pv_ops.mmio.pci_mmcfg_readw = pkvm_mmio_readw;
	pv_ops.mmio.pci_mmcfg_readl = pkvm_mmio_readl;
	pv_ops.mmio.pci_mmcfg_writeb = pkvm_mmio_writeb;
	pv_ops.mmio.pci_mmcfg_writew = pkvm_mmio_writew;
	pv_ops.mmio.pci_mmcfg_writel = pkvm_mmio_writel;

	apic_update_callback(wakeup_secondary_cpu, pkvm_wakeup_secondary_cpu);

#ifdef CONFIG_VIRTIO_BALLOON_HYP_OPS
	virtio_balloon_hyp_ops = &pkvm_virtio_balloon_hyp_ops;
#endif
}
