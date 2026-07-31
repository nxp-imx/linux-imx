/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PKVM_X86_MEM_PROTECT_H
#define __PKVM_X86_MEM_PROTECT_H

#include <linux/log2.h>

/**
 * enum pkvm_page_state - Track how a page is owned and mapped among page tables
 * @PKVM_PAGE_NONE:		The page is not mapped in this page table and
 *				not owned by this page table owner.
 * @PKVM_PAGE_OWNED:		The page is mapped in this page table and
 *				exclusively owned by this page table owner.
 * @PKVM_PAGE_SHARED_OWNED:	The page is mapped in this page table and owned
 *				by this page table owner. But it is also mapped
 *				in some other page tables.
 * @PKVM_PAGE_SHARED_BORROWED:	The page is mapped in this page table but not
 *				owned by this page table owner. It is owned by
 *				another page table owner, and mapped in the
 *				owner's page table as well.
 *
 * The page state would be stored either in the pkvm_page structure or in the
 * page table leaf entry (both present leaf and non-present leaf), depending on
 * the kind of a page table. If it is stored in the pte, then ignored bits
 * should be picked to avoid conflicting with the hardware bits, thus changing
 * page state doesn't require flushing TLB. As different vendors may have
 * different page table formats, which ignored bits are used to store page state
 * is vendor specific.
 *
 * There will be 3 kinds of page tables: the host mmu, the guest mmu and the
 * hypervisor mmu. The host mmu and the guest mmu are the 2nd level page tables
 * which have a vendor specific page table format. So the vendor code should
 * decide which ignored bits can be used to store the page state. To have a
 * better performance, the page state of the host memory will be stored in the
 * pkvm_page structure, to reduce the host page-table walk when checking the
 * host page state, as well as to avoid splitting a huge host mapping for only
 * changing page state. But as there is no pkvm_page structure for MMIO, the
 * host MMIO page state will be stored in the host mmu pte. The hypervisor mmu
 * by default maps all the memory pages, and doesn't have to store page states
 * in its ignore bits. A guest page state will always be stored in the guest mmu
 * pte.
 *
 * After pKVM init finalized, all the memory pages are mapped in the host mmu
 * with PKVM_PAGE_OWNED page state, which represents owned by the host
 * exclusively, except for the pKVM hypervisor TEXT/DATA and pKVM reserved
 * memory pages, which are PKVM_PAGE_NONE to the host to represent the host is
 * not able to access.
 *
 * At the running time, a page state can be transferred from one to another
 * by the pKVM hypervisor via the memory protection API.
 */
enum pkvm_page_state {
	PKVM_PAGE_NONE = 0ULL,
	PKVM_PAGE_OWNED,
	PKVM_PAGE_SHARED_OWNED,
	PKVM_PAGE_SHARED_BORROWED,
	NR_PKVM_PAGE_STATES,
};

#define PKVM_PAGE_STATE_BITS		bits_per(NR_PKVM_PAGE_STATES - 1)

/**
 * enum pkvm_owner_id - Id to indicate the page owner.
 * @PKVM_ID_HOST:	Host VM.
 * @PKVM_ID_HYP:	pKVM hypervisor.
 * @PKVM_ID_GUEST:	(Any) guest VM.
 *
 * The owner ID is used to indicate the page owner. It is saved in the host
 * MMU non-present leaf entry.
 */
enum pkvm_owner_id {
	PKVM_ID_HOST,
	PKVM_ID_HYP,
	PKVM_ID_GUEST,
	PKVM_MAX_ID,
};

#define PKVM_OWNER_ID_BITS		bits_per(PKVM_MAX_ID - 1)

int pkvm_host_donate_hyp(unsigned long phys, unsigned long size, bool clear);
int pkvm_host_donate_hyp_share_ro(unsigned long phys, unsigned long size,
				  bool clear);
void pkvm_hyp_donate_host(unsigned long phys, unsigned long size, bool clear);
int pkvm_host_donate_hyp_mmio(unsigned long phys, unsigned long size);
int pkvm_hyp_donate_host_mmio_locked(unsigned long phys, unsigned long size);
int pkvm_host_share_hyp(unsigned long phys, unsigned long size);
void pkvm_host_unshare_hyp(unsigned long phys, unsigned long size);
int pkvm_host_donate_guest(struct kvm_vcpu *vcpu, unsigned long gpa,
			   unsigned long hpa, unsigned long size);
int pkvm_host_share_guest(struct kvm_vcpu *vcpu, unsigned long gpa,
			  unsigned long hpa, unsigned long size,
			  bool writable);
int pkvm_host_unshare_guest(struct kvm *kvm, unsigned long gpa,
			    unsigned long size);
int pkvm_host_test_clear_young_guest(struct kvm *kvm, unsigned long gpa,
				     unsigned long size, bool mkold);
int pkvm_guest_share_host(struct kvm_vcpu *vcpu, unsigned long gpa,
			  unsigned long size, unsigned long *need_mc_pages);
int pkvm_guest_unshare_host(struct kvm_vcpu *vcpu, unsigned long gpa,
			    unsigned long size, unsigned long *need_mc_pages);
int pkvm_host_use_dma(unsigned long phys, unsigned long size);
void pkvm_host_unuse_dma(unsigned long phys, unsigned long size);

#endif /* __PKVM_X86_MEM_PROTECT_H */
