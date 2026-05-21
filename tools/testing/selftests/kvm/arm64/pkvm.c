// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Google LLC
 * Author: Fuad Tabba <tabba@google.com>
 * Author: Will Deacon <will@kernel.org>
 */

#define _GNU_SOURCE
#include <linux/arm-smccc.h>
#include <linux/kernel.h>	/* Must be included before bitfield.h */
#include <linux/bitfield.h>
#include <asm/hwcap.h>
#include <fcntl.h>
#include <kvm_util.h>
#include <processor.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <sys/auxv.h>
#include <sys/resource.h>
#include <test_util.h>

#include "pkvm.h"

enum guest_commands {
	CMD_HEARTBEAT = 1,	/* Lets host know that guest is alive. */
	CMD_INC,		/* Ask host to increment shared/relinquished page. */
	CMD_INC_PRIVATE,	/* Ask host to increment private page. */
};

#define  PC(v)  ((uint64_t)&(v))

#define GUEST_ASSERT_REG_RAZ(reg)	GUEST_ASSERT_EQ(read_sysreg_s(reg), 0)
#define GUEST_ASSERT_REG_MASKED(reg, mask) \
	GUEST_ASSERT_EQ(read_sysreg_s(reg) & ~(uint64_t)(mask), 0)

/* KVM UID value: 28b46fb6-2ec5-11e9-a9ca-4b564d003a74 */
#define ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_0	0xb66fb428U
#define ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_1	0xe911c52eU
#define ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_2	0x564bcaa9U
#define ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_3	0x743a004dU

#define ARM_SMCC_KVM_FUNC_MEM_MASK				\
	(							\
		BIT(ARM_SMCCC_KVM_FUNC_HYP_MEMINFO) |		\
		BIT(ARM_SMCCC_KVM_FUNC_MEM_SHARE) |		\
		BIT(ARM_SMCCC_KVM_FUNC_MEM_UNSHARE) |		\
		BIT(ARM_SMCCC_KVM_FUNC_MEM_RELINQUISH)		\
	)

#define ARM_SMCC_KVM_FUNC_MMIO_GUARD_MASK			\
	(							\
		BIT(ARM_SMCCC_KVM_FUNC_MMIO_GUARD_INFO) |	\
		BIT(ARM_SMCCC_KVM_FUNC_MMIO_GUARD_ENROLL) |	\
		BIT(ARM_SMCCC_KVM_FUNC_MMIO_GUARD_MAP) |	\
		BIT(ARM_SMCCC_KVM_FUNC_MMIO_GUARD_UNMAP)	\
	)

#define PAGE_SIZE		MIN_PAGE_SIZE
#define PAGE_MASK		(~(PAGE_SIZE - 1))

#define GPA_BASE (BIT_ULL(30))
#define GVA_BASE GPA_BASE
#define GPA_PAGES (10)
#define GPA_SIZE (GPA_PAGES * PAGE_SIZE)
#define MMIO_UNMAPPED_ADDR (GVA_BASE + PAGE_SIZE * (GPA_PAGES + 1) + 13)

extern unsigned char brk_pc, mmio_pc;
static volatile uint64_t guest_ex_pc;
static volatile uint64_t guest_ex_addr;

/*
 * Returns the amount of memory locked by the current process.
 */
static int get_proc_locked_vm_size(void)
{
	unsigned long lock_size = 0;
	char *line = NULL;
	size_t size = 0;
	int ret = -1;
	FILE *f;

	f = fopen("/proc/self/status", "r");
	if (!f) {
		perror("fopen");
		return -1;
	}

	while (getline(&line, &size, f) > 0) {
		if (sscanf(line, "VmLck:\t%8lu kB", &lock_size) > 0) {
			ret = (int)(lock_size << 10);
			goto out;
		}

		free(line);
		line = NULL;
		size = 0;
	}

	perror("Unable to parse VmLck in /proc/self/status");
out:
	free(line);
	fclose(f);
	return ret;
}

static void smccc(uint32_t func, uint64_t arg, struct arm_smccc_res *res)
{
	smccc_hvc(func, arg, 0, 0, 0, 0, 0, 0, res);
}

static void smccc2(uint32_t func, uint64_t arg1, uint64_t arg2, struct arm_smccc_res *res)
{
	smccc_hvc(func, arg1, arg2, 0, 0, 0, 0, 0, res);
}

/*
 * Issues an smccc call from the guest to hyp and returns the status.
 */
static unsigned long smccc_status(uint32_t func, uint64_t arg)
{
	struct arm_smccc_res res;

	smccc(func, arg, &res);
	return res.a0;
}

/*
 * Helper function to share/unshare the range specified by the physical address phys.
 */
static int smccc_xshare(u32 func_id, phys_addr_t phys, size_t size)
{
	phys_addr_t end = phys + size;

	if (phys & ~PAGE_MASK)
		return -1;

	if (end <= phys)
		return -1;

	while (phys < end) {
		if (smccc_status(func_id, phys))
			return -1;

		phys += PAGE_SIZE;
	}

	return 0;
}

/*
 * Issues hypercalls to share the range specified by the physical address phys.
 */
static int smccc_share(phys_addr_t phys, size_t size)
{
	return smccc_xshare(ARM_SMCCC_VENDOR_HYP_KVM_MEM_SHARE_FUNC_ID, phys, size);
}

/*
 * Issues hypercalls to unshare the range specified by the physical address phys.
 */
static int smccc_unshare(phys_addr_t phys, size_t size)
{
	return smccc_xshare(ARM_SMCCC_VENDOR_HYP_KVM_MEM_UNSHARE_FUNC_ID, phys, size);
}

/*
 * Issues hypercalls to unshare the range specified by the physical address phys.
 */
static int smccc_relinquish(phys_addr_t phys)
{
	return smccc_status(ARM_SMCCC_VENDOR_HYP_KVM_MEM_RELINQUISH_FUNC_ID, phys);
}

/*
 * Checks that the hyp calls and their parameters are as expected.
 */
static void check_hyp_call(void)
{
	struct arm_smccc_res res;

	smccc(ARM_SMCCC_VENDOR_HYP_CALL_UID_FUNC_ID, 0, &res);

	GUEST_ASSERT(res.a0 == ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_0);
	GUEST_ASSERT(res.a1 == ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_1);
	GUEST_ASSERT(res.a2 == ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_2);
	GUEST_ASSERT(res.a3 == ARM_SMCCC_VENDOR_HYP_UID_KVM_REG_3);
}

/*
 * Checks that the hypervisor services in service_mask are supported.
 */
static void check_hyp_services(u64 service_mask)
{
	struct arm_smccc_res res;

	smccc(ARM_SMCCC_VENDOR_HYP_KVM_FEATURES_FUNC_ID, 0, &res);
	GUEST_ASSERT((res.a0 & service_mask) == service_mask);
}

/*
 * Checks that the memory sharing services are there and correctly setup.
 */
static void check_mem_services(void)
{
	check_hyp_services(ARM_SMCC_KVM_FUNC_MEM_MASK);
	GUEST_ASSERT(smccc_status(ARM_SMCCC_VENDOR_HYP_KVM_HYP_MEMINFO_FUNC_ID, 0) == PAGE_SIZE);
}

/*
 * Enrolls the guest in MMIO Guard.
 */
static void enroll_mmio_guard(void)
{
	check_hyp_services(ARM_SMCC_KVM_FUNC_MMIO_GUARD_MASK);
	GUEST_ASSERT(smccc_status(ARM_SMCCC_VENDOR_HYP_KVM_MMIO_GUARD_INFO_FUNC_ID, 0)
				  == PAGE_SIZE);
	GUEST_ASSERT(smccc_status(ARM_SMCCC_VENDOR_HYP_KVM_MMIO_GUARD_ENROLL_FUNC_ID, 0) == 0);
}

/*
 * Maps the addr (one page) into mmio guard.
 */
static void map_ucall_mmio(vm_paddr_t addr)
{
	struct arm_smccc_res res;

	smccc2(ARM_SMCCC_VENDOR_HYP_KVM_MMIO_GUARD_MAP_FUNC_ID, addr,
	       PROT_READ | PROT_WRITE, &res);
	GUEST_ASSERT(res.a0 == 0);
}

/*
 * Test sharing and unsharing of memory from the guest to the host.
 * The guest issues commands to the host to test the view from the host-side.
 */
static void test_xshare(void)
{
	unsigned long status;
	u64 *val = (u64 *)GVA_BASE;

	*val = 42;

	GUEST_SYNC(CMD_INC_PRIVATE);
	GUEST_ASSERT(*val == 42);

	/* Test sharing of memory outside of the guest range. */
	status = smccc_share(MMIO_UNMAPPED_ADDR, 1);
	GUEST_ASSERT(status != 0);

	/* Test sharing and unsharing guest memory. */
	status = smccc_share(GPA_BASE, GPA_PAGES);
	GUEST_ASSERT(status == 0);

	status = smccc_share(GPA_BASE, GPA_PAGES);
	GUEST_ASSERT(status != 0);

	GUEST_SYNC(CMD_INC);
	GUEST_ASSERT(*val == 43);

	status = smccc_unshare(GPA_BASE, GPA_PAGES);
	GUEST_ASSERT(status == 0);

	status = smccc_unshare(GPA_BASE, GPA_PAGES);
	GUEST_ASSERT(status != 0);

	GUEST_SYNC(CMD_INC_PRIVATE);
	GUEST_ASSERT(*val == 43);
}

/*
 * Test memory relinquish.
 * Relinquishes a page, asks the host to increment it, and then accesses it
 * again to check that the host saw a poisoned (zero) page and that it was able
 * to access it (by incrementing the value stored in it).
 */
static void test_relinquish(void)
{
	unsigned long status;
	u64 *val = (u64 *)GVA_BASE;

	*val = 42;

	status = smccc_relinquish(GPA_BASE);
	GUEST_ASSERT(status == 0);

	GUEST_SYNC(CMD_INC);
	GUEST_ASSERT(*val == 1);
}

#define ID_AA64DFR0_EL1_NONZERO_NI					\
(									\
	SYS_FIELD_PREP_ENUM(ID_AA64DFR0_EL1, DoubleLock, NI)	|	\
	SYS_FIELD_PREP_ENUM(ID_AA64DFR0_EL1, MTPMU, NI)			\
)

/*
 * Tests that feature registers have the values expected for protected guests.
 * This isn't extensive, and could be made more so by knowing that the host
 * view of the feature resisters is.
 */
static void test_feature_id_regs(void)
{
	GUEST_ASSERT_EQ(read_sysreg_s(SYS_ID_AA64DFR0_EL1), ID_AA64DFR0_EL1_NONZERO_NI);
	GUEST_ASSERT_REG_RAZ(SYS_ID_AA64DFR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_AA64AFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_AA64AFR1_EL1);

	GUEST_ASSERT_REG_RAZ(SYS_ID_PFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_PFR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_DFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_AFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR2_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR3_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR2_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR3_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR4_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR5_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR4_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_ISAR6_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_MVFR0_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_MVFR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_MVFR2_EL1);
	GUEST_ASSERT_REG_RAZ(sys_reg(3, 0, 0, 3, 3));
	GUEST_ASSERT_REG_RAZ(SYS_ID_PFR2_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_DFR1_EL1);
	GUEST_ASSERT_REG_RAZ(SYS_ID_MMFR5_EL1);
	GUEST_ASSERT_REG_RAZ(sys_reg(3, 0, 0, 3, 7));
	GUEST_ASSERT_REG_MASKED(SYS_ID_AA64PFR2_EL1, ID_AA64PFR2_EL1_FPMR);
}

static void reset_handler_globals(void)
{
	guest_ex_pc = ~0ULL;
	guest_ex_addr = ~0ULL;
}

static void assert_handler_globals(u64 pc, u64 addr)
{
	GUEST_ASSERT_EQ(guest_ex_pc, pc);
	GUEST_ASSERT_EQ(guest_ex_addr, addr);
}

/*
 * Handler for data abort.
 * Sets guest_ex_pc to the PC that triggered the abort,
 * and guest_ex_addr to the memory access of the attempted access.
 */
static void dabt_handler(struct ex_regs *regs)
{
	guest_ex_pc = regs->pc;
	guest_ex_addr = read_sysreg(far_el1);
	regs->pc += 4;
}

/*
 * Handler for brk abort.
 * Sets guest_ex_pc to the PC that triggered the abort,
 * and clears guest_ex_addr to the memory access of the attempted access.
 */
static void brk_handler(struct ex_regs *regs)
{
	guest_ex_pc = regs->pc;
	guest_ex_addr = 0;
	regs->pc += 4;
}

/*
 * Tests that mmio guard is working by trying to increment an address not mapped
 * to mmio guard and ensuring that the guest receives a data abort.
 * All in all, tests that the access does not generate an MMIO exit to the host,
 * and that it injects a data abort into the guest.
 */
static void test_mmio_guard(void)
{
	volatile int *addr = (int *)MMIO_UNMAPPED_ADDR;

	reset_handler_globals();

	/*
	 * Attempt to store anything to the unmapped mmio address. The value
	 * doesn't matter as it shouldn't succeed, but aborts and handled
	 * by the handler installed earlier.
	 */
	asm volatile("mmio_pc: str x0, [%0]\n" : : "r"(addr));

	assert_handler_globals(PC(mmio_pc), (u64)addr);
}

static void test_mpidr(void)
{
	u64 mpidr_el1 = read_sysreg(mpidr_el1);

	GUEST_PRINTF("The guest sees mpidr_el1 as 0x%lx.\n", mpidr_el1);
}

/*
 * Dirties the whole first memslot donated to the guest. Used to ensure later
 * that guest memory is poisoned (cleared) after teardown.
 */
static void guest_dirty_memslot(void)
{
	memset((void *)GVA_BASE, 0x0badf00d, GPA_PAGES * PAGE_SIZE);
}

static void test_restricted_debug(void)
{
	u64 dfr0 = read_sysreg(id_aa64dfr0_el1);
	u64 brps = FIELD_GET(ID_AA64DFR0_EL1_BRPs_MASK, dfr0);
	u64 wrps = FIELD_GET(ID_AA64DFR0_EL1_WRPs_MASK, dfr0);

	GUEST_ASSERT(brps == 0);
	GUEST_ASSERT(wrps == 0);

	reset_handler_globals();
	asm volatile("brk_pc: brk #0");
	assert_handler_globals(PC(brk_pc), 0);

	write_sysreg(~0ULL, dbgbcr0_el1);
	write_sysreg(~0ULL, dbgbvr0_el1);
	write_sysreg(~0ULL, dbgwcr0_el1);
	write_sysreg(~0ULL, dbgwvr0_el1);
	write_sysreg(~0ULL, mdscr_el1);
	write_sysreg(~0ULL, oslar_el1);
	write_sysreg(~0ULL, osdlr_el1);

	isb();

	GUEST_ASSERT(read_sysreg(dbgbcr0_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(dbgbvr0_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(dbgwcr0_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(dbgwcr0_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(mdscr_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(oslsr_el1) == 0x0);
	GUEST_ASSERT(read_sysreg(osdlr_el1) == 0x0);

	GUEST_SYNC(CMD_HEARTBEAT);
}

static void test_s1pie_s1poe(void)
{
	uint64_t mmfr3 = read_sysreg_s(SYS_ID_AA64MMFR3_EL1);
	uint64_t s1pie = (mmfr3 >> 8) & 0xf;
	uint64_t s1poe = (mmfr3 >> 16) & 0xf;
	uint64_t pir, pire0, por_el1, por_el0;

	/* If features are supported, ensure registers don't trap. */
	if (s1pie) {
		GUEST_PRINTF("s1pie enabled\n");

		pir = read_sysreg_s(SYS_PIR_EL1);
		pire0 = read_sysreg_s(SYS_PIRE0_EL1);

		write_sysreg_s(0xdeadbeef, SYS_PIR_EL1);
		write_sysreg_s(0xdeadbeef, SYS_PIRE0_EL1);

		GUEST_ASSERT(read_sysreg_s(SYS_PIR_EL1) == 0xdeadbeef);
		GUEST_ASSERT(read_sysreg_s(SYS_PIRE0_EL1) == 0xdeadbeef);

		write_sysreg_s(pir, SYS_PIR_EL1);
		write_sysreg_s(pire0, SYS_PIRE0_EL1);
	}

	if (s1poe) {
		GUEST_PRINTF("s1poe enabled\n");

		por_el1 = read_sysreg_s(SYS_POR_EL1);
		por_el0 = read_sysreg_s(SYS_POR_EL0);

		write_sysreg_s(0xcafebabe, SYS_POR_EL1);
		write_sysreg_s(0xcafebabe, SYS_POR_EL0);

		GUEST_ASSERT(read_sysreg_s(SYS_POR_EL1) == 0xcafebabe);
		GUEST_ASSERT(read_sysreg_s(SYS_POR_EL0) == 0xcafebabe);

		write_sysreg_s(por_el1, SYS_POR_EL1);
		write_sysreg_s(por_el0, SYS_POR_EL0);
	}
}

static void test_fpmr(void)
{
	uint64_t pfr2 = read_sysreg_s(SYS_ID_AA64PFR2_EL1);
	uint64_t fpmr = (pfr2 >> 32) & 0xf;
	uint64_t saved, val;

	if (!fpmr)
		return;

	GUEST_PRINTF("fpmr enabled\n");

	saved = read_sysreg_s(SYS_FPMR);

	write_sysreg_s((uint64_t)0x42 << 24, SYS_FPMR);
	val = read_sysreg_s(SYS_FPMR);
	GUEST_ASSERT_EQ(val, (uint64_t)0x42 << 24);

	write_sysreg_s(saved, SYS_FPMR);
}

/*
 * Main code to run by the guest vm.
 */
static void guest_code(vm_paddr_t ucall_pool_phys, size_t ucall_pool_size,
		       vm_paddr_t ucall_mmio_phys)
{
	/*
	 * For protected VMs, if the following fails it will segfault since the
	 * VM hasn't shared, and won't be able to share the ucall_pool.
	 */

	check_hyp_call();

	check_mem_services();

	/* Share the ucall pool to be able to use the ucall interface. */
	GUEST_ASSERT(smccc_share(ucall_pool_phys, ucall_pool_size) == 0);

	/* Enroll in MMIO guard */
	enroll_mmio_guard();

	/* Map mmio range for the ucall. */
	map_ucall_mmio(ucall_mmio_phys);

	GUEST_SYNC(CMD_HEARTBEAT);

	test_mmio_guard();

	test_mpidr();

	test_xshare();

	test_relinquish();

	test_feature_id_regs();

	test_restricted_debug();

	test_s1pie_s1poe();

	test_fpmr();

	/* Populate the donated memslot to facilitate testing poisoning after destruction. */
	guest_dirty_memslot();

	GUEST_SYNC(CMD_HEARTBEAT);
	GUEST_DONE();
}

/*
 * Creates a protected VM with one vcpu and returns a pointer to it.
 */
static struct kvm_vm *test_vm_create_protected(struct kvm_vcpu **vcpu)
{
	extern char pvm_entry_start[], pvm_entry_end[];
	struct vm_shape shape = VM_SHAPE_DEFAULT;
	vm_paddr_t idmap_gpa;
	struct kvm_vm *vm;

	shape.type = VM_TYPE_PROTECTED;
	vm = vm_create_shape_with_one_vcpu(shape, vcpu, NULL);

	idmap_gpa = vm_compute_max_gfn(vcpu[0]->vm) * vm->page_size;
	vm_userspace_mem_region_add(vm, VM_MEM_SRC_ANONYMOUS, idmap_gpa, 2, 1, 0);
	memcpy(addr_gpa2hva(vm, idmap_gpa), pvm_entry_start,
	       pvm_entry_end - pvm_entry_start);
	virt_map(vm, idmap_gpa, idmap_gpa, 1);

	vcpu_set_reg(vcpu[0], ARM64_CORE_REG(regs.pc), idmap_gpa);
	return vm;
}

static void vcpu_set_pvm_boot_args(struct kvm_vcpu *vcpu)
{
	struct pvm_boot_args *pvm_boot_args;
	vm_paddr_t arg_gpa;
	u64 reg;
	int i;

	reg = vcpu_get_reg(vcpu, ARM64_CORE_REG(regs.pc));
	arg_gpa = (reg + vcpu->vm->page_size) - sizeof(*pvm_boot_args);
	pvm_boot_args = addr_gpa2hva(vcpu->vm, arg_gpa);

	for (i = 0; i < ARRAY_SIZE(pvm_boot_args->regs); ++i) {
		reg = vcpu_get_reg(vcpu, ARM64_CORE_REG(regs.regs[i]));
		pvm_boot_args->regs[i] = reg;
	}

	pvm_boot_args->jump_tgt = (uint64_t)guest_code;
	vcpu_set_reg(vcpu, ARM64_CORE_REG(regs.regs[0]), (uint64_t)arg_gpa);

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_CPACR_EL1));
	pvm_boot_args->cpacr_el1 = reg;

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_SCTLR_EL1));
	pvm_boot_args->sctlr_el1 = reg;

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_TCR_EL1));
	pvm_boot_args->tcr_el1 = reg;

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_TTBR0_EL1));
	pvm_boot_args->ttbr0_el1 = reg;

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_VBAR_EL1));
	pvm_boot_args->vbar_el1 = reg;

	reg = vcpu_get_reg(vcpu, KVM_ARM64_SYS_REG(SYS_TPIDR_EL1));
	pvm_boot_args->tpidr_el1 = reg;

	reg = vcpu_get_reg(vcpu, ARM64_CORE_REG(sp_el1));
	pvm_boot_args->sp_el1 = reg;
}

sigjmp_buf jmpbuf;
u64 *expected_addr;
/*
 * Handler for catching expected segfaults triggered when accessing guest memory
 * not shared with the host.
 */
void segfault_sigaction(int signum, siginfo_t *si, void *ctx)
{
	TEST_ASSERT(si->si_addr == expected_addr, "Caught fault at unexpected address.");
	pr_info("Caught expected segfault at address %p\n", si->si_addr);
	siglongjmp(jmpbuf, 1);
}

/*
 * Increments a value in guest memory shared with the host.
 */
static void cmd_inc(struct kvm_vm *vm)
{
	struct userspace_mem_region *slot1 = memslot2region(vm, 1);
	u64 *addr = (u64 *)slot1->host_mem;

	expected_addr = addr;
	(*addr)++;
}

/*
 * Increments a value in guest memory not shared with the host, and asserts that
 * the access triggers a segfault.
 */
static void cmd_inc_private(struct kvm_vm *vm)
{
	struct sigaction sa = {
		.sa_sigaction = segfault_sigaction,
		.sa_flags = SA_SIGINFO,
	};

	if (sigsetjmp(jmpbuf, 1) == 0) {
		sigaction(SIGSEGV, &sa, NULL);
		cmd_inc(vm);
	}

	signal(SIGSEGV, SIG_DFL);
}

/*
 * Returns true if the memory region is zero.
 */
static bool is_zero(const u64 *mem, size_t size)
{
	TEST_ASSERT(mem, "Invalid memory to test.");
	TEST_ASSERT(size, "Invalid size to test.");

	while (size) {
		if (*mem++)
			return false;

		size -= sizeof(*mem);
	}

	return true;
}

/*
 * Tests that the guest memory is poisoned after it has been torn down.
 */
static void test_poison_guest_mem(struct kvm_vm *vm, uint64_t npages)
{
	struct userspace_mem_region *region = memslot2region(vm, 1);

	pr_info("test poison: mmap_start is 0x%p, size is %lu\n",
		region->host_mem, PAGE_SIZE * npages);

	TEST_ASSERT(is_zero(region->host_mem, PAGE_SIZE * npages),
		    "Guest memory not poisoned!");
}

static void set_guest_mem(struct kvm_vm *vm, uint64_t npages)
{
	struct userspace_mem_region *region = memslot2region(vm, 1);

	memset(region->host_mem, 0xaa, PAGE_SIZE * npages);
}

/*
 * Processes commands issued by the guest to the host via the ucall interface.
 */
static void handle_cmd(struct kvm_vm *vm, int cmd)
{
	switch (cmd) {
	case CMD_HEARTBEAT:
		pr_info("Guest heartbeat.\n");
		break;
	case CMD_INC:
		cmd_inc(vm);
		break;
	case CMD_INC_PRIVATE:
		cmd_inc_private(vm);
		break;
	default:
		TEST_FAIL("Unexpected guest command: %d\n", cmd);
		break;
	}
}

static void test_run(enum vm_mem_backing_src_type src_type)
{
	struct kvm_vcpu *vcpu, *t;
	struct kvm_vm *vm;
	struct ucall uc;
	bool guest_done = false;
	struct rusage usage;

	getrusage(RUSAGE_SELF, &usage);
	pr_info("Memory usage at start: %ld bytes\n", usage.ru_maxrss);
	pr_info("Host VmLck at start: %d bytes\n", get_proc_locked_vm_size());

	vm = test_vm_create_protected(&vcpu);

	TEST_ASSERT(vm->page_size == PAGE_SIZE,
		    "Page size expected to be PAGE_SIZE.");

	vm_userspace_mem_region_add(vm, src_type, GPA_BASE, 1, GPA_PAGES, 0);

	test_poison_guest_mem(vm, GPA_PAGES);
	set_guest_mem(vm, GPA_PAGES);

	virt_map(vm, GPA_BASE, GVA_BASE, GPA_PAGES);
	vm_init_descriptor_tables(vm);

	kvm_for_each_vcpu(vm, t) {
		vcpu_init_descriptor_tables(t);
		vcpu_args_set(t, 3, get_ucall_pool_gpa(vm), get_ucall_pool_size(vm),
			      get_ucall_mmio_gpa(vm));
	}

	vm_install_sync_handler(vm, VECTOR_SYNC_CURRENT,
				ESR_ELx_EC_DABT_CUR, dabt_handler);
	vm_install_sync_handler(vm, VECTOR_SYNC_CURRENT,
				ESR_ELx_EC_BRK64, brk_handler);
	vcpu_set_pvm_boot_args(vcpu);

	while (!guest_done) {
		uint64_t uc_num;

		vcpu_run(vcpu);

		switch (uc_num = get_ucall(vcpu, &uc)) {
		case UCALL_SYNC:
			handle_cmd(vm, uc.args[1]);
			break;
		case UCALL_DONE:
			pr_info("Guest done\n");
			guest_done = true;
			break;
		case UCALL_ABORT:
			REPORT_GUEST_ASSERT(uc);
			break;
		case UCALL_PRINTF:
			REPORT_GUEST_PRINTF(uc);
			break;
		default:
			pr_info("Guest ucall %ld %ld\n", uc_num, uc.args[1]);
			TEST_FAIL("Unexpected guest exit\n");
		}
	}

	pr_info("Host VmLck before teardown: %d bytes\n", get_proc_locked_vm_size());
	TEST_ASSERT(get_proc_locked_vm_size() > GPA_SIZE, "Assert: No memory locked.");

	getrusage(RUSAGE_SELF, &usage);
	pr_info("Memory usage before teardown: %ld bytes\n", usage.ru_maxrss);

	kvm_vm_release(vm);

	test_poison_guest_mem(vm, GPA_PAGES);

	kvm_vm_free(vm);

	pr_info("Host VmLck after teardown: %d\n", get_proc_locked_vm_size());
	TEST_ASSERT(get_proc_locked_vm_size() == 0, "Assert: Memory locked.");

	getrusage(RUSAGE_SELF, &usage);
	pr_info("Memory usage after teardown: %ld bytes\n", usage.ru_maxrss);
}

int main(int argc, char *argv[])
{
	pr_info("\n*** Testing regular pages.\n");
	test_run(VM_MEM_SRC_ANONYMOUS);

	if (thp_configured()) {
		pr_info("\n*** Testing THP. This is best-effort.\n");
		test_run(VM_MEM_SRC_ANONYMOUS_THP);
	} else {
		pr_info("\n*** THP is not configured. Skipping THP tests.\n");
	}

	pr_info("All ok!\n");
	return 0;
}
