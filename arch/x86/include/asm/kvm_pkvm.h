/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_KVM_PKVM_H
#define _ASM_X86_KVM_PKVM_H

#ifdef CONFIG_PKVM_X86
#include <linux/bug.h>
#include <linux/kvm_host.h>
#include <linux/memblock.h>
#include <linux/mm.h>
#include <asm/desc.h>
#include <asm/kvm_para.h>
#include <asm/pkvm_image.h>

#define PKVM_MEMBLOCK_REGIONS		128
#define PKVM_STACK_SIZE			SZ_16K
/* Size of reserved space for private parameter in pKVM stack */
#define PKVM_STACK_TOP_RESV		16
#define PKVM_PGTABLE_MAX_LEVELS		5

struct idt_page {
	gate_desc idt[IDT_ENTRIES];
} __aligned(PAGE_SIZE);

struct pkvm_pcpu {
	u8 stack[PKVM_STACK_SIZE] __aligned(16);
	struct gdt_page gdt_page;
	struct idt_page idt_page;
	struct tss_struct tss;
};

struct pkvm_hyp {
	int num_cpus;
	struct pkvm_pcpu *pcpus[CONFIG_NR_CPUS];
	struct kvm *host_kvm;
	struct kvm_vcpu *host_vcpus[CONFIG_NR_CPUS];
};

#define PKVM_HYP_PAGES		(PAGE_ALIGN(sizeof(struct pkvm_hyp)) >> PAGE_SHIFT)
#define PKVM_PCPU_PAGES		(PAGE_ALIGN(sizeof(struct pkvm_pcpu)) >> PAGE_SHIFT)

enum pkvm_mem_type {
	PKVM_RESERVED_USED_MEMORY,
	PKVM_RESERVED_UNUSED_MEMORY,
	PKVM_TEXT_DATA,
};

struct pkvm_mem_info {
	enum pkvm_mem_type type;
	unsigned long va;
	unsigned long pa;
	unsigned long size;
	u64 prot;
};

#define TO_PKVM_HC(f)		CONCATENATE(__pkvm__, f)

enum pkvm_hc {
	#define PKVM_HC(f)	TO_PKVM_HC(f),
	#include <asm/pkvm_hypercalls.h>

	MAX_PKVM_HYPERCALLS,
};

#define PKVM_HC_DATA_MAX_NUM		4

union pkvm_hc_data {
	struct {
		struct pkvm_memcache memcache;
	} vm_destroy;
	struct {
		struct pkvm_memcache memcache;
	} vcpu_free;
	struct {
		u64 data;
	} get_msr;
	union {
		unsigned long rsp;
		unsigned long rip;
		unsigned long cr0;
		unsigned long cr3;
		unsigned long cr4;
		u64 pdptrs[4];
	} cache_reg;
	struct {
		unsigned long data;
	} get_rflags;
	struct {
		struct kvm_segment seg_val;
		int seg;
	} set_segment;
	struct {
		struct kvm_segment seg_val;
	} get_segment;
	struct {
		u64 data;
	} get_segment_base;
	struct {
		struct desc_ptr desc;
	} set_idt;
	struct {
		struct desc_ptr desc;
	} get_idt;
	struct {
		struct desc_ptr desc;
	} set_gdt;
	struct {
		struct desc_ptr desc;
	} get_gdt;
	struct {
		u32 data;
	} get_interrupt_shadow;
	struct {
		bool data;
	} get_nmi_mask;
	struct {
		struct pkvm_memcache memcache;
	} vcpu_after_set_cpuid;
	struct {
		struct pkvm_memcache memcache;
	} vcpu_add_fpstate;
	struct {
		u64 data[PKVM_HC_DATA_MAX_NUM];
	} raw;
};

/*
 * The union pkvm_hc_data is passed in hypercalls via the registers used for
 * hypercall arguments, and PKVM_HC_DATA_MAX_NUM represents the number of those
 * registers. So the size of the union cannot be larger than that.
 */
static_assert(sizeof(union pkvm_hc_data) == PKVM_HC_DATA_MAX_NUM * sizeof(u64));

#define PKVM_HC_DATA_NUM(f)		\
	(ALIGN(sizeof(((union pkvm_hc_data *)0)->f), sizeof(u64)) / sizeof(u64))

#define PKVM_HC_OUTPUT_NUM(f)	f##_output_num
#define PKVM_HC_INPUT_NUM(f)	f##_input_num

enum {
	#define PKVM_HC(f)	PKVM_HC_OUTPUT_NUM(f) = 0,
	#define PKVM_HC_OUT(f)	PKVM_HC_OUTPUT_NUM(f) = PKVM_HC_DATA_NUM(f),
	#include <asm/pkvm_hypercalls.h>

	#define PKVM_HC(f)	PKVM_HC_INPUT_NUM(f) = 0,
	#define PKVM_HC_IN(f)	PKVM_HC_INPUT_NUM(f) = PKVM_HC_DATA_NUM(f),
	#include <asm/pkvm_hypercalls.h>
};

static inline int pkvm_hc_output_num(enum pkvm_hc hc)
{
	switch (hc) {
	#define PKVM_HC(f) case TO_PKVM_HC(f): return PKVM_HC_OUTPUT_NUM(f);
	#include <asm/pkvm_hypercalls.h>
	default:
		return 0;
	}
}

static inline int pkvm_hc_input_num(enum pkvm_hc hc)
{
	switch (hc) {
	#define PKVM_HC(f) case TO_PKVM_HC(f): return PKVM_HC_INPUT_NUM(f);
	#include <asm/pkvm_hypercalls.h>
	default:
		return 0;
	}
}

#define PKVM_HC_IN_0()
#define PKVM_HC_IN_1(a1)		, "b"((unsigned long)a1)
#define PKVM_HC_IN_2(a1, a2)		PKVM_HC_IN_1(a1), "c"((unsigned long)a2)
#define PKVM_HC_IN_3(a1, a2, a3)	PKVM_HC_IN_2(a1, a2), "d"((unsigned long)a3)
#define PKVM_HC_IN_4(a1, a2, a3, a4)	PKVM_HC_IN_3(a1, a2, a3), "S"((unsigned long)a4)

#define PKVM_HC_OUT_0(o)
#define PKVM_HC_OUT_1(o)		, "=b"((o)->raw.data[0])
#define PKVM_HC_OUT_2(o)		PKVM_HC_OUT_1(o), "=c"((o)->raw.data[1])
#define PKVM_HC_OUT_3(o)		PKVM_HC_OUT_2(o), "=d"((o)->raw.data[2])
#define PKVM_HC_OUT_4(o)		PKVM_HC_OUT_3(o), "=S"((o)->raw.data[3])

#define __pkvm_hypercall(f, o, n, ...)							\
({											\
	int ret;									\
	asm volatile(KVM_HYPERCALL							\
		     : "=a"(ret) CONCATENATE(PKVM_HC_OUT_, n)(o)			\
		     : "a"(TO_PKVM_HC(f))						\
		       CONCATENATE(PKVM_HC_IN_, COUNT_ARGS(__VA_ARGS__))(__VA_ARGS__)	\
		     : "memory");							\
	ret;										\
})

#define PKVM_HC_UNREACHABLE(f)								\
({											\
	BUILD_BUG_ON_MSG(1, #f " requires unsupported number of data");			\
	-EINVAL;									\
})

#define pkvm_hypercall(f, ...)								\
({											\
	BUILD_BUG_ON(PKVM_HC_OUTPUT_NUM(f));						\
	__pkvm_hypercall(f, NULL, 0, ##__VA_ARGS__);					\
})

#define pkvm_hypercall_out(f, o, ...)							\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 0,				\
			      __pkvm_hypercall(f, NULL, 0, ##__VA_ARGS__),		\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 1,				\
			      __pkvm_hypercall(f, o, 1, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 2,				\
			      __pkvm_hypercall(f, o, 2, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 3,				\
			      __pkvm_hypercall(f, o, 3, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 4,				\
			      __pkvm_hypercall(f, o, 4, ##__VA_ARGS__),			\
	PKVM_HC_UNREACHABLE(f))))))

#define pkvm_hypercall_in(f, i)								\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 1,				\
			      __pkvm_hypercall(f, NULL, 0, (i)->raw.data[0]),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 2,				\
			      __pkvm_hypercall(f, NULL, 0, (i)->raw.data[0],		\
					       (i)->raw.data[1]),			\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 3,				\
			      __pkvm_hypercall(f, NULL, 0, (i)->raw.data[0],		\
					       (i)->raw.data[1],			\
					       (i)->raw.data[2]),			\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 4,				\
			      __pkvm_hypercall(f, NULL, 0, (i)->raw.data[0],		\
					       (i)->raw.data[1],			\
					       (i)->raw.data[2],			\
					       (i)->raw.data[3]),			\
	PKVM_HC_UNREACHABLE(f)))))

static inline unsigned long pkvm_hc(struct kvm_vcpu *vcpu)
{
	return vcpu->arch.regs[VCPU_REGS_RAX];
}

#define DEFINE_PKVM_HC_INPUT(n, reg)							\
static inline unsigned long pkvm_hc_input##n(struct kvm_vcpu *vcpu)			\
{											\
	return vcpu->arch.regs[VCPU_REGS_##reg];					\
}											\
static inline void pkvm_hc_get_input##n(struct kvm_vcpu *vcpu, union pkvm_hc_data *p)	\
{											\
	BUILD_BUG_ON(n == 0 || n > PKVM_HC_DATA_MAX_NUM);				\
	p->raw.data[n - 1] = vcpu->arch.regs[VCPU_REGS_##reg];				\
}

DEFINE_PKVM_HC_INPUT(1, RBX)
DEFINE_PKVM_HC_INPUT(2, RCX)
DEFINE_PKVM_HC_INPUT(3, RDX)
DEFINE_PKVM_HC_INPUT(4, RSI)

static inline void pkvm_hc_get_input(struct kvm_vcpu *vcpu, enum pkvm_hc hc,
				     union pkvm_hc_data *in)
{
	switch (pkvm_hc_input_num(hc)) {
	case 4:
		pkvm_hc_get_input4(vcpu, in);
		fallthrough;
	case 3:
		pkvm_hc_get_input3(vcpu, in);
		fallthrough;
	case 2:
		pkvm_hc_get_input2(vcpu, in);
		fallthrough;
	case 1:
		pkvm_hc_get_input1(vcpu, in);
		fallthrough;
	case 0:
		break;
	default:
		BUG();
	}
}

static inline void pkvm_hc_set_ret(struct kvm_vcpu *vcpu, int ret)
{
	vcpu->arch.regs[VCPU_REGS_RAX] = ret;
}

#define DEFINE_PKVM_HC_OUTPUT(n, reg)							\
static inline void pkvm_hc_set_output##n(struct kvm_vcpu *vcpu, union pkvm_hc_data *p)	\
{											\
	BUILD_BUG_ON(n == 0 || n > PKVM_HC_DATA_MAX_NUM);				\
	vcpu->arch.regs[VCPU_REGS_##reg] = p->raw.data[n - 1];				\
}

DEFINE_PKVM_HC_OUTPUT(1, RBX)
DEFINE_PKVM_HC_OUTPUT(2, RCX)
DEFINE_PKVM_HC_OUTPUT(3, RDX)
DEFINE_PKVM_HC_OUTPUT(4, RSI)

static inline void pkvm_hc_set_output(struct kvm_vcpu *vcpu, enum pkvm_hc hc,
				      union pkvm_hc_data *out)
{
	switch (pkvm_hc_output_num(hc)) {
	case 4:
		pkvm_hc_set_output4(vcpu, out);
		fallthrough;
	case 3:
		pkvm_hc_set_output3(vcpu, out);
		fallthrough;
	case 2:
		pkvm_hc_set_output2(vcpu, out);
		fallthrough;
	case 1:
		pkvm_hc_set_output1(vcpu, out);
		fallthrough;
	case 0:
		break;
	default:
		BUG();
	}
}

extern unsigned long pkvm_sym(page_offset_base);
extern unsigned long pkvm_sym(phys_base);
extern struct pkvm_hyp *pkvm_sym(pkvm_hyp);
extern struct memblock_region pkvm_sym(pkvm_memory)[];
extern unsigned int pkvm_sym(pkvm_memblock_nr);
extern struct cpuinfo_x86 pkvm_sym(boot_cpu_data);
#ifdef CONFIG_DYNAMIC_PHYSICAL_MASK
extern phys_addr_t pkvm_sym(physical_mask);
#endif
extern pteval_t pkvm_sym(__default_kernel_pte_mask);
#ifdef CONFIG_AMD_MEM_ENCRYPT
extern u64 pkvm_sym(sme_me_mask);
#endif
extern struct pkvm_init_ops *pkvm_sym(init_ops);
extern struct cpumask pkvm_sym(__cpu_possible_mask);
extern unsigned int pkvm_sym(nr_cpu_ids);
DECLARE_STATIC_KEY_FALSE(pkvm_sym(switch_vcpu_ibpb));
extern u64 pkvm_sym(x86_pred_cmd);
extern struct fpu_state_config pkvm_sym(fpu_kernel_cfg);
extern struct fpu_state_config pkvm_sym(fpu_user_cfg);
#ifdef CONFIG_X86_64
DECLARE_STATIC_KEY_FALSE(pkvm_sym(__fpu_state_size_dynamic));
#endif

u64 pkvm_total_reserve_pages(void);
PKVM_DECLARE(void *, pkvm_early_alloc_page, (void));
PKVM_DECLARE(void *, pkvm_early_alloc_contig, (unsigned int nr_pages));
PKVM_DECLARE(void, pkvm_early_alloc_init, (void *virt, unsigned long size));
PKVM_DECLARE(int, pkvm_setup_per_cpu, (int cpu, unsigned long base));
PKVM_DECLARE(unsigned int, pkvm_per_cpu_nr_pages, (void));
PKVM_DECLARE(unsigned long, pkvm_per_cpu_offset, (int cpu));
#define GEN(x, ...) PKVM_DECLARE(void, handle_exception_##x, (void));
#include <asm/GEN-for-each-exc.h>
#undef GEN
PKVM_DECLARE(void, set_x86_spec_ctrl, (u64 spec_ctrl));

static inline unsigned long pkvm_data_pages(unsigned long extra_global,
					    unsigned long extra_percpu)
{
	unsigned long global_pages = PKVM_HYP_PAGES + extra_global;
	unsigned long percpu_pages = PKVM_PCPU_PAGES + extra_percpu +
				     pkvm_sym(pkvm_per_cpu_nr_pages)();

	return global_pages + percpu_pages * num_possible_cpus();
}

static inline unsigned long get_host_stack_top(struct pkvm_pcpu *pcpu)
{
	return (unsigned long) &pcpu->stack[sizeof(pcpu->stack)];
}

static inline unsigned long __pkvm_pgtable_max_pages(unsigned long nr_pages)
{
	unsigned long total = 0, i;

	/* Provision the worst case */
	for (i = 0; i < PKVM_PGTABLE_MAX_LEVELS; i++) {
		nr_pages = DIV_ROUND_UP(nr_pages, PTRS_PER_PTE);
		total += nr_pages;
	}

	/*
	 * For each level except the last one, may need an extra page table
	 * if the VA range is not aligned to the next level's page size.
	 * For example, range [0x1ff000, 0x201000) consists of just two
	 * 4K pages, however, not one but two page tables at the first
	 * level are required for mapping this range, since it crosses the
	 * 2M boundary.
	 */
	total += PKVM_PGTABLE_MAX_LEVELS - 1;

	return total;
}

static inline unsigned long __pkvm_pgtable_total_pages(void)
{
	unsigned long total = 0, i;

	for (i = 0; i < pkvm_sym(pkvm_memblock_nr); i++) {
		struct memblock_region *reg = &pkvm_sym(pkvm_memory)[i];

		total += __pkvm_pgtable_max_pages(reg->size >> PAGE_SHIFT);
	}

	return total;
}

static inline unsigned long pkvm_hyp_pgtable_pages(void)
{
	return __pkvm_pgtable_total_pages();
}

static inline unsigned long __vmemmap_memblock_size(struct memblock_region *reg,
						    size_t vmemmap_entry_size)
{
	unsigned long nr_pages = reg->size >> PAGE_SHIFT;
	unsigned long start, end;

	/* Translate the pfn to the vmemmap entry */
	start = (reg->base >> PAGE_SHIFT) * vmemmap_entry_size;
	end = start + nr_pages * vmemmap_entry_size;
	start = ALIGN_DOWN(start, PAGE_SIZE);
	end = ALIGN(end, PAGE_SIZE);

	return end - start;
}

static inline unsigned long pkvm_vmemmap_pages(size_t vmemmap_entry_size)
{
	unsigned long total_size = 0, i;

	for (i = 0; i < pkvm_sym(pkvm_memblock_nr); i++) {
		total_size += __vmemmap_memblock_size(&pkvm_sym(pkvm_memory)[i],
						      vmemmap_entry_size);
	}

	return total_size >> PAGE_SHIFT;
}

static inline unsigned long pkvm_host_pgtable_pages(void)
{
	/*
	 * Usually the 2G ~ 4G are used as MMIO hole. As the host mmu should
	 * contain the mapping for the MMIO, reserve additional memory pages
	 * for 2GB MMIO size.
	 *
	 * MMIO regions also exists in the high-end physical address space which
	 * is not able to know the size precisely during the early boot.
	 * Therefore, instead of specifically reserving memory pages for these
	 * MMIO regions, share the reservation for the 2G ~ 4G MMIO region.
	 * While this approach cannot guarantee that memory page allocation for
	 * the required MMIO mappings will always success, it is feasible
	 * because the reservation for 2G ~ 4G MMIO region is based on the worst
	 * case (4K page size) while in practice the MMIO region will be mapped
	 * using the possible largest page size(e.g. 1G/2M), which significantly
	 * reduces the memory consumption and makes sharing possible.
	 */
	return __pkvm_pgtable_total_pages() +
	       __pkvm_pgtable_max_pages(SZ_2G >> PAGE_SHIFT);
}

static inline bool pkvm_is_protected_vm(struct kvm *kvm)
{
	return kvm->arch.vm_type == KVM_X86_PKVM_PROTECTED_VM;
}

static inline bool pkvm_is_protected_vcpu(struct kvm_vcpu *vcpu)
{
	return pkvm_is_protected_vm(vcpu->kvm);
}

static inline size_t pkvm_guest_initial_fpstate_size(struct kvm *kvm)
{
	/*
	 * The pkvm hypervisor requires to have at least the size of struct
	 * fpstate for both pVM (to switch FPU and emulate XFD MSR) and npVM
	 * (to emulate XFD MSR only).
	 */
	size_t size = ALIGN(offsetof(struct fpstate, regs), 64);

	/*
	 * The pkvm hypervisor switches the FPU registers for pVM thus the size
	 * should be extended with fpu_user_cfg.default_size to satisfy the
	 * default features (w/o dynamic features).
	 */
	if (pkvm_is_protected_vm(kvm))
		size += fpu_user_cfg.default_size;

	return PAGE_ALIGN(size);
}

#ifdef __PKVM_HYP__

#undef kvm_err
#undef kvm_info
#undef kvm_debug
#undef kvm_debug_ratelimited
#undef kvm_pr_unimpl

#ifdef CONFIG_PKVM_X86_DEBUG

#define kvm_err(fmt, ...) \
	pr_err("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_info(fmt, ...) \
	pr_info("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_debug(fmt, ...) \
	pr_debug("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_debug_ratelimited(fmt, ...) \
	pr_debug_ratelimited("pkvm: " fmt, ## __VA_ARGS__)
#define kvm_pr_unimpl(fmt, ...) \
	pr_err_ratelimited("pkvm: " fmt, ## __VA_ARGS__)

#else /* CONFIG_PKVM_X86_DEBUG */

#define kvm_err(fmt, ...) do {} while(0)
#define kvm_info(fmt, ...) do {} while(0)
#define kvm_debug(fmt, ...) do {} while(0)
#define kvm_debug_ratelimited(fmt, ...) do {} while(0)
#define kvm_pr_unimpl(fmt, ...) do {} while(0)

#undef WARN_ON
#undef WARN
#undef WARN_ON_ONCE
#undef WARN_ONCE
#undef _BUG_FLAGS

#define WARN_ON(condition) ({						\
	int __ret_warn_on = !!(condition);				\
	unlikely(__ret_warn_on);					\
})

#define WARN(condition, format...) ({					\
	int __ret_warn_on = !!(condition);				\
	no_printk(format);						\
	unlikely(__ret_warn_on);					\
})

#define WARN_ON_ONCE(condition) WARN_ON(condition)
#define WARN_ONCE(condition, format...) WARN(condition, format)

#define _BUG_FLAGS(ins, flags, extra)  asm volatile(ins)

#endif /* CONFIG_PKVM_X86_DEBUG */

#undef KVM_BUG_ON
#define KVM_BUG_ON(cond, kvm)						\
({									\
	bool __ret = !!(cond);						\
									\
	BUG_ON(__ret);							\
	unlikely(__ret);						\
})

#endif /* __PKVM_HYP__ */

#else /* !CONFIG_PKVM_X86 */

static inline bool pkvm_is_protected_vm(struct kvm *kvm) { return false; }
static inline bool pkvm_is_protected_vcpu(struct kvm_vcpu *vcpu) { return false; }

#endif /* CONFIG_PKVM_X86 */

#endif /* _ASM_X86_KVM_PKVM_H */
