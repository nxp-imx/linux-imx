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
#include <asm/pkvm_redef.h>

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
	u32 apic_id;
	u32 msi_dest_id;
	int cpu;
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

#define PKVM_HOST_VM_HANDLE	INT_MAX

#ifdef CONFIG_PKVM_INTEL
struct clear_ce_data {
	u64 phys;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
};

struct set_lm_ce_data {
	u64 phys;
	u64 pgd_gpa;
	u64 donation_page_gpa;
	u16 did;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
	u8 ats_supported: 1;
};

struct set_sm_ce_data {
	u64 phys;
	u64 pasid_table_gpa;
	u64 donation_page_gpa;
	u32 max_pasid;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
	u8 ats_supported: 1;
	u8 pasid_supported: 3;
	u8 pasid_enabled: 1;
};

struct pasid_setup_fl_data {
	u64 phys;
	u64 fsptptr_gpa;
	u64 donation_page_gpa;
	u32 pasid;
	u16 did;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
	u8 force_snoop: 1;
};

struct pasid_setup_sl_data {
	u64 phys;
	u64 ssptptr_gpa;
	u64 donation_page_gpa;
	u32 pasid;
	u16 did;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
};

struct pasid_teardown_data {
	u64 phys;
	u32 pasid;
	u8 bus;
	u8 devfn;
	u8 ats_qdep;
};

struct alloc_domain_data {
	u64 phys;
	u64 max_addr;
	u64 pgd_gpa;
	u16 bdf;
	u16 gaw;
	u8 agaw;
	u8 iommu_superpage;
	u8 iommu_coherency;
	u8 use_first_level;
};

struct domain_map_data {
	u64 pgd_gpa;
	u64 iov_pfn;
	u64 phys_pfn;
	u64 nr_pages;
	u64 prot;
	struct pkvm_memcache mc;
};

struct modify_irte_data {
	u64 phys;
	u32 index;   /* IRTE index */
	u32 pad;
	u64 irte_lo;
	u64 irte_hi;
};
#endif

#define TO_PKVM_HC(f)		CONCATENATE(__pkvm__, f)

enum pkvm_hc {
	#define PKVM_HC(f)	TO_PKVM_HC(f),
	#include <asm/pkvm_hypercalls.h>

	MAX_PKVM_HYPERCALLS,
};

#define PKVM_HC_DATA_MAX_NUM		10

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
		unsigned long reqs_to_host;
#define HOST_HANDLE_EXIT			0
#define HOST_HANDLE_GUESTDBG_SINGLESTEP		1
#define HOST_INIT_MMU				2
#define HOST_RESET_MMU				3
#define HOST_APF_READY				4
	} vcpu_run;
#ifdef CONFIG_PKVM_INTEL
	struct {
		u64 val;
	} iommu_mmio_read;
	struct {
		struct clear_ce_data data;
	} iommu_clear_ce;
	union {
		struct set_lm_ce_data in;
		struct set_lm_ce_data out;
	} iommu_set_lm_ce;
	union {
		struct set_sm_ce_data in;
		struct set_sm_ce_data out;
	} iommu_set_sm_ce;
	union {
		struct pasid_setup_fl_data in;
		struct pasid_setup_fl_data out;
	} iommu_pasid_setup_fl;
	union {
		struct pasid_setup_sl_data in;
		struct pasid_setup_sl_data out;
	} iommu_pasid_setup_sl;
	struct {
		struct pasid_teardown_data data;
	} iommu_pasid_teardown;
	struct {
		struct alloc_domain_data data;
	} iommu_alloc_domain;
	struct {
		struct pkvm_memcache memcache;
	} iommu_free_domain;
	union {
		struct domain_map_data in;
		struct domain_map_data out;
	} iommu_domain_map;
	struct {
		struct modify_irte_data data;
	} iommu_modify_irte;
#endif
	struct {
		bool has_intr;
	} protected_apic_has_interrupt;
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
#define PKVM_HC_DATA_OUT_NUM(f)		\
	(ALIGN(sizeof(((union pkvm_hc_data *)0)->f.out), sizeof(u64)) / sizeof(u64))
#define PKVM_HC_DATA_IN_NUM(f)		\
	(ALIGN(sizeof(((union pkvm_hc_data *)0)->f.in), sizeof(u64)) / sizeof(u64))

#define PKVM_HC_OUTPUT_NUM(f)	f##_output_num
#define PKVM_HC_INPUT_NUM(f)	f##_input_num

enum {
	#define PKVM_HC(f)		PKVM_HC_OUTPUT_NUM(f) = 0,
	#define PKVM_HC_OUT(f)		PKVM_HC_OUTPUT_NUM(f) = PKVM_HC_DATA_NUM(f),
	#define PKVM_HC_INOUT(f)	PKVM_HC_OUTPUT_NUM(f) = PKVM_HC_DATA_OUT_NUM(f),
	#include <asm/pkvm_hypercalls.h>

	#define PKVM_HC(f)		PKVM_HC_INPUT_NUM(f) = 0,
	#define PKVM_HC_IN(f)		PKVM_HC_INPUT_NUM(f) = PKVM_HC_DATA_NUM(f),
	#define PKVM_HC_INOUT(f)	PKVM_HC_INPUT_NUM(f) = PKVM_HC_DATA_IN_NUM(f),
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

#define IN_ARG_0()
#define IN_ARG_1(a1, ...)						(unsigned long)a1
#define IN_ARG_2(a1, a2, ...)						(unsigned long)a2
#define IN_ARG_3(a1, a2, a3, ...)					(unsigned long)a3
#define IN_ARG_4(a1, a2, a3, a4, ...)					(unsigned long)a4
#define IN_ARG_5(a1, a2, a3, a4, a5, ...)				(unsigned long)a5
#define IN_ARG_6(a1, a2, a3, a4, a5, a6, ...)				(unsigned long)a6
#define IN_ARG_7(a1, a2, a3, a4, a5, a6, a7, ...)			(unsigned long)a7
#define IN_ARG_8(a1, a2, a3, a4, a5, a6, a7, a8, ...)			(unsigned long)a8
#define IN_ARG_9(a1, a2, a3, a4, a5, a6, a7, a8, a9, ...)		(unsigned long)a9
#define IN_ARG_10(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, ...)		(unsigned long)a10

#define PKVM_HC_DECLARE_IN_0(...)
#define PKVM_HC_DECLARE_IN_1(...)
#define PKVM_HC_DECLARE_IN_2(...)
#define PKVM_HC_DECLARE_IN_3(...)
#define PKVM_HC_DECLARE_IN_4(...)
#define PKVM_HC_DECLARE_IN_5(...)
#define PKVM_HC_DECLARE_IN_6(...)	register unsigned long r8_in asm("r8") = 	\
								IN_ARG_6(__VA_ARGS__)
#define PKVM_HC_DECLARE_IN_7(...)	PKVM_HC_DECLARE_IN_6(__VA_ARGS__);		\
					register unsigned long r9_in asm("r9") = 	\
								IN_ARG_7(__VA_ARGS__)
#define PKVM_HC_DECLARE_IN_8(...)	PKVM_HC_DECLARE_IN_7(__VA_ARGS__);		\
					register unsigned long r10_in asm("r10") = 	\
								IN_ARG_8(__VA_ARGS__)
#define PKVM_HC_DECLARE_IN_9(...)	PKVM_HC_DECLARE_IN_8(__VA_ARGS__);		\
					register unsigned long r11_in asm("r11") = 	\
								IN_ARG_9(__VA_ARGS__)
#define PKVM_HC_DECLARE_IN_10(...)	PKVM_HC_DECLARE_IN_9(__VA_ARGS__);		\
					register unsigned long r12_in asm("r12") = 	\
								IN_ARG_10(__VA_ARGS__)

#define PKVM_HC_IN_0(...)
#define PKVM_HC_IN_1(...)		, "b"(IN_ARG_1(__VA_ARGS__))
#define PKVM_HC_IN_2(...)		PKVM_HC_IN_1(__VA_ARGS__), "c"(IN_ARG_2(__VA_ARGS__))
#define PKVM_HC_IN_3(...)		PKVM_HC_IN_2(__VA_ARGS__), "d"(IN_ARG_3(__VA_ARGS__))
#define PKVM_HC_IN_4(...)		PKVM_HC_IN_3(__VA_ARGS__), "S"(IN_ARG_4(__VA_ARGS__))
#define PKVM_HC_IN_5(...)		PKVM_HC_IN_4(__VA_ARGS__), "D"(IN_ARG_5(__VA_ARGS__))
#define PKVM_HC_IN_6(...)		PKVM_HC_IN_5(__VA_ARGS__), "r" (r8_in)
#define PKVM_HC_IN_7(...)		PKVM_HC_IN_6(__VA_ARGS__), "r" (r9_in)
#define PKVM_HC_IN_8(...)		PKVM_HC_IN_7(__VA_ARGS__), "r" (r10_in)
#define PKVM_HC_IN_9(...)		PKVM_HC_IN_8(__VA_ARGS__), "r" (r11_in)
#define PKVM_HC_IN_10(...)		PKVM_HC_IN_9(__VA_ARGS__), "r" (r12_in)

#define PKVM_HC_DECLARE_OUT_0
#define PKVM_HC_DECLARE_OUT_1
#define PKVM_HC_DECLARE_OUT_2
#define PKVM_HC_DECLARE_OUT_3
#define PKVM_HC_DECLARE_OUT_4
#define PKVM_HC_DECLARE_OUT_5
#define PKVM_HC_DECLARE_OUT_6		register unsigned long r8_out asm("r8")
#define PKVM_HC_DECLARE_OUT_7		PKVM_HC_DECLARE_OUT_6;				\
					register unsigned long r9_out asm("r9")
#define PKVM_HC_DECLARE_OUT_8		PKVM_HC_DECLARE_OUT_7;				\
					register unsigned long r10_out asm("r10")
#define PKVM_HC_DECLARE_OUT_9		PKVM_HC_DECLARE_OUT_8;				\
					register unsigned long r11_out asm("r11")
#define PKVM_HC_DECLARE_OUT_10		PKVM_HC_DECLARE_OUT_9;				\
					register unsigned long r12_out asm("r12")

#define PKVM_HC_OUT_INSN_0
#define PKVM_HC_OUT_INSN_1
#define PKVM_HC_OUT_INSN_2
#define PKVM_HC_OUT_INSN_3
#define PKVM_HC_OUT_INSN_4
#define PKVM_HC_OUT_INSN_5
#define PKVM_HC_OUT_INSN_6		"movq %%r8, %[out5]\n\t"
#define PKVM_HC_OUT_INSN_7		PKVM_HC_OUT_INSN_6 "movq %%r9, %[out6]\n\t"
#define PKVM_HC_OUT_INSN_8		PKVM_HC_OUT_INSN_7 "movq %%r10, %[out7]\n\t"
#define PKVM_HC_OUT_INSN_9		PKVM_HC_OUT_INSN_8 "movq %%r11, %[out8]\n\t"
#define PKVM_HC_OUT_INSN_10		PKVM_HC_OUT_INSN_9 "movq %%r12, %[out9]\n\t"

#define PKVM_HC_OUT_0(o)
#define PKVM_HC_OUT_1(o)		, "=b"((o)->raw.data[0])
#define PKVM_HC_OUT_2(o)		PKVM_HC_OUT_1(o), "=c"((o)->raw.data[1])
#define PKVM_HC_OUT_3(o)		PKVM_HC_OUT_2(o), "=d"((o)->raw.data[2])
#define PKVM_HC_OUT_4(o)		PKVM_HC_OUT_3(o), "=S"((o)->raw.data[3])
#define PKVM_HC_OUT_5(o)		PKVM_HC_OUT_4(o), "=D" ((o)->raw.data[4])
#define PKVM_HC_OUT_6(o)		PKVM_HC_OUT_5(o), "=r"(r8_out),			\
					[out5] "=m" ((o)->raw.data[5])
#define PKVM_HC_OUT_7(o)		PKVM_HC_OUT_6(o), "=r"(r9_out),			\
					[out6] "=m" ((o)->raw.data[6])
#define PKVM_HC_OUT_8(o)		PKVM_HC_OUT_7(o), "=r"(r10_out),		\
					[out7] "=m" ((o)->raw.data[7])
#define PKVM_HC_OUT_9(o)		PKVM_HC_OUT_8(o), "=r"(r11_out),		\
					[out8] "=m" ((o)->raw.data[8])
#define PKVM_HC_OUT_10(o)		PKVM_HC_OUT_9(o), "=r"(r12_out),		\
					[out9] "=m" ((o)->raw.data[9])

#define __pkvm_hypercall(f, o, n, ...)							\
({											\
	CONCATENATE(PKVM_HC_DECLARE_IN_, COUNT_ARGS(__VA_ARGS__))(__VA_ARGS__);		\
	CONCATENATE(PKVM_HC_DECLARE_OUT_, n);						\
	int ret;									\
	asm volatile(KVM_HYPERCALL							\
		     PKVM_HC_OUT_INSN_##n						\
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

#define __pkvm_hypercall_inout(f, o, ...)						\
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
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 5,				\
			      __pkvm_hypercall(f, o, 5, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 6,				\
			      __pkvm_hypercall(f, o, 6, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 7,				\
			      __pkvm_hypercall(f, o, 7, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 8,				\
			      __pkvm_hypercall(f, o, 8, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 9,				\
			      __pkvm_hypercall(f, o, 9, ##__VA_ARGS__),			\
	__builtin_choose_expr(PKVM_HC_OUTPUT_NUM(f) == 10,				\
			      __pkvm_hypercall(f, o, 10, ##__VA_ARGS__),		\
	PKVM_HC_UNREACHABLE(f))))))))))))

#define INPUT_DATA_1(i)		(i)->raw.data[0]
#define INPUT_DATA_2(i)		INPUT_DATA_1(i), (i)->raw.data[1]
#define INPUT_DATA_3(i)		INPUT_DATA_2(i), (i)->raw.data[2]
#define INPUT_DATA_4(i)		INPUT_DATA_3(i), (i)->raw.data[3]
#define INPUT_DATA_5(i)		INPUT_DATA_4(i), (i)->raw.data[4]
#define INPUT_DATA_6(i)		INPUT_DATA_5(i), (i)->raw.data[5]
#define INPUT_DATA_7(i)		INPUT_DATA_6(i), (i)->raw.data[6]
#define INPUT_DATA_8(i)		INPUT_DATA_7(i), (i)->raw.data[7]
#define INPUT_DATA_9(i)		INPUT_DATA_8(i), (i)->raw.data[8]
#define INPUT_DATA_10(i)	INPUT_DATA_9(i), (i)->raw.data[9]

#define pkvm_hypercall_inout(f, i, o)							\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 1,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_1(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 2,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_2(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 3,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_3(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 4,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_4(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 5,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_5(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 6,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_6(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 7,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_7(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 8,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_8(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 9,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_9(i)),		\
	__builtin_choose_expr(PKVM_HC_INPUT_NUM(f) == 10,				\
			      __pkvm_hypercall_inout(f, o, INPUT_DATA_10(i)),		\
	PKVM_HC_UNREACHABLE(f)))))))))))

#define pkvm_hypercall_out(f, o, ...)							\
	__pkvm_hypercall_inout(f, o, ##__VA_ARGS__)

#define pkvm_hypercall_in(f, i)								\
	pkvm_hypercall_inout(f, i, (union pkvm_hc_data *)NULL)

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
DEFINE_PKVM_HC_INPUT(5, RDI)
DEFINE_PKVM_HC_INPUT(6, R8)
DEFINE_PKVM_HC_INPUT(7, R9)
DEFINE_PKVM_HC_INPUT(8, R10)
DEFINE_PKVM_HC_INPUT(9, R11)
DEFINE_PKVM_HC_INPUT(10, R12)

static inline void pkvm_hc_get_input(struct kvm_vcpu *vcpu, enum pkvm_hc hc,
				     union pkvm_hc_data *in)
{
	switch (pkvm_hc_input_num(hc)) {
	case 10:
		pkvm_hc_get_input10(vcpu, in);
		fallthrough;
	case 9:
		pkvm_hc_get_input9(vcpu, in);
		fallthrough;
	case 8:
		pkvm_hc_get_input8(vcpu, in);
		fallthrough;
	case 7:
		pkvm_hc_get_input7(vcpu, in);
		fallthrough;
	case 6:
		pkvm_hc_get_input6(vcpu, in);
		fallthrough;
	case 5:
		pkvm_hc_get_input5(vcpu, in);
		fallthrough;
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
DEFINE_PKVM_HC_OUTPUT(5, RDI)
DEFINE_PKVM_HC_OUTPUT(6, R8)
DEFINE_PKVM_HC_OUTPUT(7, R9)
DEFINE_PKVM_HC_OUTPUT(8, R10)
DEFINE_PKVM_HC_OUTPUT(9, R11)
DEFINE_PKVM_HC_OUTPUT(10, R12)

static inline void pkvm_hc_set_output(struct kvm_vcpu *vcpu, enum pkvm_hc hc,
				      union pkvm_hc_data *out)
{
	switch (pkvm_hc_output_num(hc)) {
	case 10:
		pkvm_hc_set_output10(vcpu, out);
		fallthrough;
	case 9:
		pkvm_hc_set_output9(vcpu, out);
		fallthrough;
	case 8:
		pkvm_hc_set_output8(vcpu, out);
		fallthrough;
	case 7:
		pkvm_hc_set_output7(vcpu, out);
		fallthrough;
	case 6:
		pkvm_hc_set_output6(vcpu, out);
		fallthrough;
	case 5:
		pkvm_hc_set_output5(vcpu, out);
		fallthrough;
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
extern uint16_t pkvm_sym(__cachemode2pte_tbl)[];
extern struct pkvm_init_ops *pkvm_sym(init_ops);
extern struct cpumask pkvm_sym(__cpu_possible_mask);
extern unsigned int pkvm_sym(nr_cpu_ids);
extern bool pkvm_sym(msi_dest_mode_logical);
DECLARE_STATIC_KEY_FALSE(pkvm_sym(switch_vcpu_ibpb));
extern u64 pkvm_sym(x86_pred_cmd);
extern struct fpu_state_config pkvm_sym(fpu_kernel_cfg);
extern struct fpu_state_config pkvm_sym(fpu_user_cfg);
#ifdef CONFIG_X86_64
DECLARE_STATIC_KEY_FALSE(pkvm_sym(__fpu_state_size_dynamic));
#endif
extern unsigned int pkvm_sym(tsc_khz);
extern bool pkvm_sym(pvmfw_present);
extern phys_addr_t pkvm_sym(pvmfw_base);
extern phys_addr_t pkvm_sym(pvmfw_size);

extern phys_addr_t pkvm_sym(pkvm_ramoops_console_pa);
extern size_t pkvm_sym(pkvm_ramoops_console_size);

extern unsigned long pkvm_sym(kaslr_offset_val);

extern bool __read_mostly pkvm_sym(enable_apicv);
extern bool __read_mostly pkvm_sym(enable_ipiv);
extern bool __read_mostly pkvm_sym(enable_vpid);

u64 pkvm_total_reserve_pages(void);
PKVM_DECLARE(void *, pkvm_early_alloc_page, (struct pkvm_memcache *mc));
PKVM_DECLARE(void *, pkvm_early_alloc_contig, (unsigned int nr_pages));
PKVM_DECLARE(void, pkvm_early_alloc_init, (void *virt, unsigned long size));
PKVM_DECLARE(int, pkvm_setup_per_cpu, (int cpu, unsigned long base,
				       unsigned long pcpu_pa, unsigned long vcpu_pa));
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

static inline void push_pkvm_memcache(struct pkvm_memcache *mc,
				      void *addr, size_t size,
				      phys_addr_t (*to_pa)(void *virt))
{
	struct pkvm_page_range *head = addr;

	if (WARN_ON_ONCE(!PAGE_ALIGNED(addr) || !PAGE_ALIGNED(size)))
		return;

	*head = mc->head;
	mc->head.addr = to_pa(addr);
	mc->head.nr_pages = size >> PAGE_SHIFT;

	mc->count++;
}

static inline struct pkvm_page_range
pop_pkvm_memcache(struct pkvm_memcache *mc, void *(*to_va)(phys_addr_t phys))
{
	struct pkvm_page_range head = { 0 };

	if (!mc->count)
		return head;

	/*
	 * The popped memory should be at least one page which contains the
	 * next head.
	 */
	if (WARN_ON_ONCE(!mc->head.nr_pages))
		return head;

	head = mc->head;
	mc->head = *(struct pkvm_page_range *)to_va(head.addr);

	mc->count--;

	return head;
}

static inline void push_pkvm_memcache_page(struct pkvm_memcache *mc,
					   void *addr,
					   phys_addr_t (*to_pa)(void *virt))
{
	push_pkvm_memcache(mc, addr, PAGE_SIZE, to_pa);
}

/* For memcaches containing single-page ranges only. */
static inline void *pop_pkvm_memcache_page(struct pkvm_memcache *mc,
					   void *(*to_va)(phys_addr_t phys))
{
	if (!mc->count)
		return NULL;

	if (WARN_ON_ONCE(mc->head.nr_pages != 1))
		return NULL;

	return to_va(pop_pkvm_memcache(mc, to_va).addr);
}

/* For memcaches containing single-page ranges only. */
static inline int topup_pkvm_memcache(struct pkvm_memcache *mc,
				      unsigned long min_pages,
				      void *(*alloc_pg)(void *arg),
				      phys_addr_t (*to_pa)(void *virt),
				      void *arg)
{
	while (mc->count < min_pages) {
		void *p = alloc_pg(arg);

		if (!p)
			return -ENOMEM;

		push_pkvm_memcache_page(mc, p, to_pa);
	}

	return 0;
}

static inline void free_pkvm_memcache(struct pkvm_memcache *mc,
				      void (*free)(struct pkvm_page_range range,
						   void *arg),
				      void *(*to_va)(phys_addr_t phys),
				      void *arg)
{
	while (mc->count)
		free(pop_pkvm_memcache(mc, to_va), arg);
}

static inline void init_pkvm_mmu_memcache(struct pkvm_memcache *mc)
{
	memset(mc, 0, sizeof(*mc));
	mc->flags = PKVM_MC_ACCOUNT_PGTABLE_PAGES;
}

struct pkvm_mapping {
	struct rb_node node;
	gfn_t gfn;
	kvm_pfn_t pfn;
	gfn_t nr_pages;
	gfn_t __subtree_last;	/* Internal member for interval tree */

	struct page *pinned_page;
};

void pkvm_mapping_insert(struct pkvm_mapping *node,
			 struct rb_root_cached *root);
void pkvm_mapping_remove(struct pkvm_mapping *node,
			 struct rb_root_cached *root);
struct pkvm_mapping *pkvm_mapping_iter_first(struct rb_root_cached *root,
					     gfn_t start, gfn_t last);
struct pkvm_mapping *pkvm_mapping_iter_next(struct pkvm_mapping *node,
					    gfn_t start, gfn_t last);

/*
 * Iterates the interval tree safely, allowing removing __map node from it
 * while iterating.
 * Caution: __start and __end are evaluated multiple times.
 */
#define for_each_pkvm_mapping(__kvm, __start, __end, __map)					\
	for (struct pkvm_mapping *__tmp = pkvm_mapping_iter_first(&(__kvm)->arch.pkvm.mappings,	\
								  (__start), (__end) - 1);	\
	     __tmp && ({									\
				__map = __tmp;							\
				__tmp = pkvm_mapping_iter_next(__map, (__start), (__end) - 1);	\
				true;								\
		       });									\
	    )

#else /* !CONFIG_PKVM_X86 */

static inline bool pkvm_is_protected_vm(struct kvm *kvm) { return false; }
static inline bool pkvm_is_protected_vcpu(struct kvm_vcpu *vcpu) { return false; }

#endif /* CONFIG_PKVM_X86 */

#endif /* _ASM_X86_KVM_PKVM_H */
