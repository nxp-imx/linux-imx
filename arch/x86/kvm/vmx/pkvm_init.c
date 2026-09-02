// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) "pkvm: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/extable.h>
#include <asm/e820/api.h>
#include <asm/pkvm_image.h>
#include <asm/setup.h>
#include <asm/set_memory.h>
#include "pkvm_constants.h"
#include "vmx.h"
#include "pkvm_iommu.h"

extern u64 x86_pred_cmd;

static int __init early_pkvm_parse_cmdline(char *buf)
{
	return kstrtobool(buf, &enable_pkvm);
}
early_param("kvm-intel.pkvm", early_pkvm_parse_cmdline);

static bool relax_cpu_bugs = true;
static int __init early_pkvm_relax_cpu_bugs_parse_cmdline(char *buf)
{
	return kstrtobool(buf, &relax_cpu_bugs);
}
early_param("kvm-intel.pkvm_relax_cpu_bugs", early_pkvm_relax_cpu_bugs_parse_cmdline);

static int __init early_pvmfw_parse_cmdline(char *buf)
{
	u64 start, size;
	char *p;

	size = memparse(buf, &p);
	if (p == buf || *p != '@')
		return -EINVAL;

	buf = p + 1;
	start = memparse(buf, &p);
	if (p == buf)
		return -EINVAL;

	/* clflush_cache_range() takes size as int */
	if (size > UINT_MAX)
		return -EINVAL;

	pvmfw_present = true;
	pvmfw_base = start;
	pvmfw_size = size;
	return 0;
}
early_param("pvmfw", early_pvmfw_parse_cmdline);

static DEFINE_PER_CPU(struct vmcs *, pkvm_vmxarea);
static DEFINE_PER_CPU(struct pkvm_pcpu*, pkvm_pcpu);
static DEFINE_PER_CPU(struct kvm_vcpu*, host_vcpu);
static unsigned long data_pages;

/* Only need GDT entries for KERNEL_CS & KERNEL_DS as pKVM only use these two */
static struct gdt_page pkvm_gdt_page = {
	.gdt = {
		[GDT_ENTRY_KERNEL_CS]		= GDT_ENTRY_INIT(0xa09b, 0, 0xfffff),
		[GDT_ENTRY_KERNEL_DS]		= GDT_ENTRY_INIT(0xc093, 0, 0xfffff),
	},
};

static unsigned int intercept_w_msrs[] = {
	MSR_CORE_PERF_GLOBAL_CTRL,
	MSR_IA32_APICBASE,
	X2APIC_MSR(APIC_ID),
	MSR_IA32_XSS,
	/* User-return MSRs (except MSR_EFER which is isolated via VMCS): */
	MSR_SYSCALL_MASK,
	MSR_LSTAR,
	MSR_CSTAR,
	MSR_TSC_AUX,
	MSR_STAR,
	MSR_IA32_TSX_CTRL,
};

u64 pkvm_total_reserve_pages(void)
{
	u64 total = pkvm_vmx_data_pages();

	total += pkvm_hyp_pgtable_pages();
	total += pkvm_host_pgtable_pages();
	total += pkvm_vmemmap_pages(PKVM_VMEMMAP_ENTRY_SIZE);

	return total;
}

static __init void pkvm_setup_syms(void)
{
	int i;

	/*
	 * The pKVM hypervisor has defined the same symbol page_offset_base
	 * and phys_base with the linux kernel. Initialize with the same value
	 * used by the linux kernel before deprivilege. With this, the pkvm
	 * hypervisor code can use __va and __pa to translate between VA and PA.
	 */
	pkvm_sym(page_offset_base) = page_offset_base;
	pkvm_sym(phys_base) = phys_base;
	pkvm_sym(kaslr_offset_val) = kaslr_offset();

	/*
	 * For the pKVM hypervisor to leverage the boot_cpu_has macro to check
	 * if a specific feature is supported or not.
	 */
	memcpy(&pkvm_sym(boot_cpu_data), &boot_cpu_data, sizeof(struct cpuinfo_x86));

#ifdef CONFIG_DYNAMIC_PHYSICAL_MASK
	/* For pKVM hypervisor to decode the valid physical address bits */
	pkvm_sym(physical_mask) = physical_mask;
#endif
	/* For the pKVM hypervisor to leverage pgprot_val macro */
	pkvm_sym(__default_kernel_pte_mask) = __default_kernel_pte_mask;
	for (i = 0; i < _PAGE_CACHE_MODE_NUM; i++)
		pkvm_sym(__cachemode2pte_tbl)[i] = cachemode2protval(i);

#ifdef CONFIG_AMD_MEM_ENCRYPT
	pkvm_sym(sme_me_mask) = sme_me_mask;
#endif
	cpumask_copy(&pkvm_sym(__cpu_possible_mask), cpu_possible_mask);
	pkvm_sym(nr_cpu_ids) = nr_cpu_ids;
	pkvm_sym(msi_dest_mode_logical) = apic->dest_mode_logical;
	pkvm_sym(fpu_kernel_cfg) = fpu_kernel_cfg;
	pkvm_sym(fpu_user_cfg) = fpu_user_cfg;
#ifdef CONFIG_X86_64
	if (static_branch_unlikely(&__fpu_state_size_dynamic))
		static_branch_enable(&pkvm_sym(__fpu_state_size_dynamic));
#endif
	pkvm_sym(x86_pred_cmd) = x86_pred_cmd;
	pkvm_sym(tsc_khz) = tsc_khz;

	/* Respect below host KVM's module parameters */
	pkvm_sym(enable_apicv) = enable_apicv;
	pkvm_sym(enable_ipiv) = enable_ipiv;
	pkvm_sym(enable_vpid) = enable_vpid;
}

static __init int pkvm_setup_host_vmcs_config(void)
{
	struct vmcs_config *vmcs_config = &pkvm_sym(host_vmcs_config);
	struct vmx_capability *vmx_cap = &pkvm_sym(vmx_capability);
	struct vmcs_config_setting setting = {
		.cpu_based_vm_exec_ctrl_req =
			CPU_BASED_INTR_WINDOW_EXITING |
			CPU_BASED_USE_MSR_BITMAPS |
			CPU_BASED_ACTIVATE_SECONDARY_CONTROLS,
		.cpu_based_vm_exec_ctrl_opt = 0,
		.secondary_vm_exec_ctrl_req =
			SECONDARY_EXEC_ENABLE_EPT,
		.secondary_vm_exec_ctrl_opt =
			SECONDARY_EXEC_ENABLE_VPID |
			SECONDARY_EXEC_ENABLE_INVPCID |
			SECONDARY_EXEC_ENABLE_XSAVES |
			SECONDARY_EXEC_ENABLE_RDTSCP |
			SECONDARY_EXEC_ENABLE_USR_WAIT_PAUSE,
		.tertiary_vm_exec_ctrl_opt = 0,
		.pin_based_vm_exec_ctrl_req =
			PIN_BASED_VMX_PREEMPTION_TIMER,
		.pin_based_vm_exec_ctrl_opt = 0,
		.vmexit_ctrl_req =
			VM_EXIT_HOST_ADDR_SPACE_SIZE |
			VM_EXIT_LOAD_IA32_PAT |
			VM_EXIT_LOAD_IA32_EFER |
			VM_EXIT_SAVE_IA32_PAT |
			VM_EXIT_SAVE_IA32_EFER |
			VM_EXIT_SAVE_DEBUG_CONTROLS |
			VM_EXIT_LOAD_IA32_PERF_GLOBAL_CTRL,
		.vmexit_ctrl_opt = 0,
		.vmentry_ctrl_req =
			VM_ENTRY_LOAD_DEBUG_CONTROLS |
			VM_ENTRY_IA32E_MODE |
			VM_ENTRY_LOAD_IA32_EFER |
			VM_ENTRY_LOAD_IA32_PAT |
			VM_ENTRY_LOAD_IA32_PERF_GLOBAL_CTRL,
		.vmentry_ctrl_opt = 0,
	};

	if (boot_cpu_has(X86_FEATURE_INTEL_PT)) {
		/*
		 * Enable the RTIT VM-Exit/VM-Entry controls to guarantee the
		 * deprivileged host cannot profile the pKVM via Intel PT.
		 *
		 * Hide the vmx non-root indications and the VMCS packets from
		 * the Intel PT, and make the output written to the GPA which
		 * can be translated by the EPT.
		 */
		setting.vmexit_ctrl_req |= VM_EXIT_PT_CONCEAL_PIP |
					   VM_EXIT_CLEAR_IA32_RTIT_CTL;
		setting.vmentry_ctrl_req |= VM_ENTRY_PT_CONCEAL_PIP |
					    VM_ENTRY_LOAD_IA32_RTIT_CTL;
		setting.secondary_vm_exec_ctrl_req |= SECONDARY_EXEC_PT_CONCEAL_VMX |
						      SECONDARY_EXEC_PT_USE_GPA;
	}

	if (boot_cpu_has(X86_FEATURE_ARCH_LBR)) {
		/*
		 * Enable the arch-LBR VM-Exit/VM-Entry controls to guarantee the
		 * deprivileged host cannot profile the pKVM via arch-LBR.
		 */
		setting.vmexit_ctrl_req |= VM_EXIT_CLEAR_IA32_LBR_CTL;
		setting.vmentry_ctrl_req |= VM_ENTRY_LOAD_IA32_LBR_CTL;
	}

	if (boot_cpu_has(X86_FEATURE_IBT) || boot_cpu_has(X86_FEATURE_SHSTK)) {
		unsigned long s_cet;

		rdmsrq(MSR_IA32_S_CET, s_cet);
		/*
		 * Supervisor shadow stack is not enabled in the linux kernel
		 * yet. The VMCS shadow stack fields of both guest (deprivileged
		 * host) and host (pKVM) will be initialized as 0 to simplify.
		 * So check the shadow stack enable bit here to guarantee this
		 * assumption.
		 */
		if (s_cet & CET_SHSTK_EN) {
			pr_warn("Supervisor Shadow Stack is enabled but not supported by pKVM\n");
			return -EOPNOTSUPP;
		}

		/*
		 * Enable the CET VM-Exit/VM-Entry controls to guarantee the
		 * deprivileged host cannot manipulate the control flow of the
		 * pKVM.
		 */
		setting.vmexit_ctrl_req |= VM_EXIT_LOAD_CET_STATE;
		setting.vmentry_ctrl_req |= VM_ENTRY_LOAD_CET_STATE;
	}

	if (boot_cpu_has(X86_FEATURE_MPX)) {
		/*
		 * The MPX is deprecated in the newer Intel CPU, e.g., PTL. In
		 * case the pKVM hypervisor runs on some older Intel CPU which
		 * has the MPX, enable the MPX vmexit control to guarantee the
		 * MSR_IA32_BNDCFGS will be cleared for the pKVM hypervisor.
		 *
		 * From the security/function point of view, there is no need to
		 * enable VM-Entry control to load IA32_BNDCFGS for the
		 * deprivileged host as the linux kernel will not use the MPX
		 * even if the CPU supports it. But as the code setup the VMCS
		 * config via reusing KVM's setup_vmcs_config_common() which
		 * checks if MPX VM-Entry and VM-Exit configs are in pair of
		 * not, it still needs to set VM_ENTRY_LOAD_BNDCFGS to pass
		 * this check.
		 */
		setting.vmexit_ctrl_req |= VM_EXIT_CLEAR_BNDCFGS;
		setting.vmentry_ctrl_req |= VM_ENTRY_LOAD_BNDCFGS;
	}

	if (setup_vmcs_config_common(vmcs_config, vmx_cap, &setting))
		return -EINVAL;

	pr_info("pin_based_exec_ctrl 0x%x\n", vmcs_config->pin_based_exec_ctrl);
	pr_info("cpu_based_exec_ctrl 0x%x\n", vmcs_config->cpu_based_exec_ctrl);
	pr_info("cpu_based_2nd_exec_ctrl 0x%x\n", vmcs_config->cpu_based_2nd_exec_ctrl);
	pr_info("vmexit_ctrl 0x%x\n", vmcs_config->vmexit_ctrl);
	pr_info("vmentry_ctrl 0x%x\n", vmcs_config->vmentry_ctrl);

	return 0;
}

static __init int pkvm_setup_host_vm(struct pkvm_hyp *pkvm)
{
	struct kvm_vmx *kvmx = pkvm_sym(pkvm_early_alloc_contig)(PKVM_HOST_KVM_VMX_PAGES);

	if (!kvmx) {
		pr_err("no kvm_vmx memory\n");
		return -ENOMEM;
	}

	kvmx->kvm.arch.pkvm.handle = PKVM_HOST_VM_HANDLE;
	/*
	 * Only a few fields in the kvm structure will be used, e.g.,
	 * hlt_in_guest for exception injection code to clear hlt state.
	 * As HLT instruction will be passthrough to the host VM, set
	 * hlt_in_guest as true. As the mwait/pause/cstate will also be
	 * passthrough, initialized them as well to reflect the fact.
	 */
	kvm_disable_exits(&kvmx->kvm, KVM_X86_DISABLE_EXITS_MWAIT |
				      KVM_X86_DISABLE_EXITS_HLT   |
				      KVM_X86_DISABLE_EXITS_PAUSE |
				      KVM_X86_DISABLE_EXITS_CSTATE);
	pkvm->host_kvm = &kvmx->kvm;

	return 0;
}

static struct vmcs *pkvm_alloc_vmcs(void)
{
	struct vmcs *vmcs = pkvm_sym(pkvm_early_alloc_page)(NULL);

	if (!vmcs)
		return NULL;

	if (!PAGE_ALIGNED(__pa(vmcs)))
		return NULL;

	vmcs->hdr.revision_id = vmx_basic_vmcs_revision_id(pkvm_sym(host_vmcs_config).basic);

	return vmcs;
}

static __init int pkvm_alloc_vmxarea(int cpu)
{
	struct vmcs *vmcs = pkvm_alloc_vmcs();

	if (!vmcs)
		return -ENOMEM;

	per_cpu(pkvm_vmxarea, cpu) = vmcs;
	return 0;
}

static __init void init_gdt(struct pkvm_pcpu *pcpu)
{
	pcpu->gdt_page = pkvm_gdt_page;
}

static __init void init_idt(struct pkvm_pcpu *pcpu)
{
	void (*pkvm_exception_handlers[X86_TRAP_IRET])(void) = {
#define GEN(x, ...)	\
		[x] = pkvm_sym(handle_exception_##x),
#include <asm/GEN-for-each-exc.h>
#undef GEN
	};
	gate_desc *idt = pcpu->idt_page.idt;
	struct idt_data d = {
		.segment = __KERNEL_CS,
		.bits.ist = 0,
		.bits.zero = 0,
		.bits.type = GATE_INTERRUPT,
		.bits.dpl = 0,
		.bits.p = 1,
	};
	gate_desc desc;
	int i;

	for (i = 0; i < X86_TRAP_IRET; i++) {
		d.vector = i;
		d.bits.ist = 0;
		d.addr = (const void *)pkvm_exception_handlers[i];
		idt_init_desc(&desc, &d);
		write_idt_entry(idt, i, &desc);
	}
}

static __init void init_tss(struct pkvm_pcpu *pcpu)
{
	struct desc_struct *d = pcpu->gdt_page.gdt;
	tss_desc tss;

	set_tssldt_descriptor(&tss, (unsigned long)&pcpu->tss, DESC_TSS,
			      __KERNEL_TSS_LIMIT);

	write_gdt_entry(d, GDT_ENTRY_TSS, &tss, DESC_TSS);
}

static __init int pkvm_setup_pcpu(int cpu)
{
	struct pkvm_pcpu *pcpu;
	int ret;

	if (cpu >= CONFIG_NR_CPUS) {
		pr_err("setup_pcpu: invalid CPU number %d\n", cpu);
		return -EINVAL;
	}

	pcpu = pkvm_sym(pkvm_early_alloc_contig)(PKVM_PCPU_PAGES);
	if (!pcpu) {
		pr_err("no pcpu memory for CPU%d\n", cpu);
		return -ENOMEM;
	}

	init_gdt(pcpu);
	init_idt(pcpu);
	init_tss(pcpu);

	ret = pkvm_alloc_vmxarea(cpu);
	if (ret) {
		pr_err("alloc vmxarea for CPU%d failed with ret %d\n", cpu, ret);
		return ret;
	}

	pcpu->apic_id = per_cpu(x86_cpu_to_apicid, cpu);
	pcpu->msi_dest_id = apic->calc_dest_apicid(cpu);
	pcpu->cpu = cpu;
	per_cpu(pkvm_pcpu, cpu) = pcpu;
	return 0;
}

static __init int pkvm_setup_host_vcpu(struct kvm *kvm, int cpu)
{
	struct vcpu_vmx *vmx;

	if (cpu >= CONFIG_NR_CPUS) {
		pr_err("setup_host_vcpu: invalid CPU number %d\n", cpu);
		return -EINVAL;
	}

	vmx = pkvm_sym(pkvm_early_alloc_contig)(PKVM_HOST_VCPU_VMX_PAGES);
	if (!vmx) {
		pr_err("no host vcpu memory for CPU%d\n", cpu);
		return -ENOMEM;
	}

	vmx->vmcs01.vmcs = pkvm_alloc_vmcs();
	if (!vmx->vmcs01.vmcs) {
		pr_err("no vmcs page for CPU%d\n", cpu);
		return -ENOMEM;
	}

	vmx->vmcs01.msr_bitmap = pkvm_sym(pkvm_early_alloc_page)(NULL);
	if (!vmx->vmcs01.msr_bitmap) {
		pr_err("no msr_bitmap page for CPU%d\n", cpu);
		return -ENOMEM;
	}

	/* Set msr bitmap to intercept some MSR writing */
	for (int i = 0; i < ARRAY_SIZE(intercept_w_msrs); i++)
		vmx_set_msr_bitmap_write(vmx->vmcs01.msr_bitmap, intercept_w_msrs[i]);

	vmx->vcpu.cpu = cpu;
	vmx->vcpu.vcpu_id = kvm->created_vcpus;
	vmx->vcpu.kvm = kvm;
	kvm->created_vcpus++;

	per_cpu(host_vcpu, cpu) = &vmx->vcpu;
	return 0;
}

static __init int pkvm_setup_per_cpu(int cpu)
{
	struct pkvm_pcpu *pcpu = per_cpu(pkvm_pcpu, cpu);
	struct kvm_vcpu *vcpu = per_cpu(host_vcpu, cpu);
#ifndef CONFIG_PKVM_X86_DEBUG
	unsigned int nr_pages;
	void *per_cpu_base;
#endif

	if (cpu >= CONFIG_NR_CPUS) {
		pr_err("setup_percpu: invalid CPU number %d\n", cpu);
		return -EINVAL;
	}

#ifndef CONFIG_PKVM_X86_DEBUG
	nr_pages = pkvm_sym(pkvm_per_cpu_nr_pages)();
	if (!nr_pages)
		return 0;

	per_cpu_base = pkvm_sym(pkvm_early_alloc_contig)(nr_pages);
	if (!per_cpu_base || pkvm_sym(pkvm_setup_per_cpu)(cpu, __pa(per_cpu_base),
				      __pa(pcpu), __pa(vcpu))) {
		pr_err("no percpu page for CPU%d\n", cpu);
		return -ENOMEM;
	}
#else
	/*
	 * Overwrite the pkvm's percpu setup symbols with the host percpu value
	 * as the same percpu base will be used by the pKVM and the host in the
	 * debug build.
	 */
	if (pkvm_sym(pkvm_setup_per_cpu)(cpu, __per_cpu_offset[cpu],
					 __pa(pcpu), __pa(vcpu))) {
		pr_err("no percpu page for CPU%d\n", cpu);
		return -ENOMEM;
	}
#endif

	return 0;
}

static bool mitigate_spectre_v2(struct cpuinfo_x86 *c)
{
	u64 spec_ctrl = SPEC_CTRL_IBRS;

	/* Require to set IBRS in spec ctrl MSR */
	if (!boot_cpu_has(X86_FEATURE_MSR_SPEC_CTRL))
		return false;

	/* Require eIBRS */
	if (!boot_cpu_has(X86_FEATURE_IBRS_ENHANCED))
		return false;

	if (boot_cpu_has_bug(X86_BUG_EIBRS_PBRSB)) {
		/* Fill RSB after vmexit */
		set_cpu_cap(&pkvm_sym(boot_cpu_data), X86_FEATURE_RSB_VMEXIT_LITE);

		clear_bit(X86_BUG_EIBRS_PBRSB, (unsigned long *)c->x86_capability);
		pr_info("mitigated eibrs_pbrsb when mitigating spectre_v2\n");
	}

	if (boot_cpu_has_bug(X86_BUG_BHI)) {
		/* Require to set BHI_DIS_S to mitigate BHI bug */
		if (!boot_cpu_has(X86_FEATURE_BHI_CTRL))
			return false;
		spec_ctrl |= SPEC_CTRL_BHI_DIS_S;
	}

	pkvm_sym(set_x86_spec_ctrl)(spec_ctrl);

	if (spec_ctrl & SPEC_CTRL_BHI_DIS_S) {
		clear_bit(X86_BUG_BHI, (unsigned long *)c->x86_capability);
		pr_info("mitigated bhi when mitigating spectre_v2\n");
	}

	return true;
}

static bool __mitigate_spectre_v2_user(void)
{
	/* Requires eIBRS so that no need for STIBP in case SMT is activated. */
	if (!boot_cpu_has(X86_FEATURE_IBRS_ENHANCED))
		return false;

	/* Require IBPB */
	if (!boot_cpu_has(X86_FEATURE_IBPB))
		return false;

	static_branch_enable(&pkvm_sym(switch_vcpu_ibpb));

	return true;
}

static bool mitigate_spectre_v2_user(struct cpuinfo_x86 *c)
{
	if (!__mitigate_spectre_v2_user())
		return false;

	if (boot_cpu_has_bug(X86_BUG_VMSCAPE)) {
		clear_bit(X86_BUG_VMSCAPE, (unsigned long *)c->x86_capability);
		pr_info("mitigated vmscape when mitigating spectre_v2_user\n");
	}

	return true;
}

static bool mitigate_spec_store_bypass(void)
{
	u64 spec_ctrl = SPEC_CTRL_SSBD;

	/* Requires spec ctrl MSR to set SSBD to disable SSB. */
	if (!boot_cpu_has(X86_FEATURE_MSR_SPEC_CTRL) ||
	    !boot_cpu_has(X86_FEATURE_SPEC_CTRL_SSBD))
		return false;

	pkvm_sym(set_x86_spec_ctrl)(spec_ctrl);

	return true;
}

static bool mitigate_bhi(void)
{
	u64 spec_ctrl = SPEC_CTRL_BHI_DIS_S;

	/* Requires spec ctrl MSR to set BHI_DIS_S to mitigate BHI */
	if (!boot_cpu_has(X86_FEATURE_MSR_SPEC_CTRL) ||
	    !boot_cpu_has(X86_FEATURE_BHI_CTRL))
		return false;

	pkvm_sym(set_x86_spec_ctrl)(spec_ctrl);

	return true;
}

static bool mitigate_eibrs_pbrsb(void)
{
	/* Fill RSB after vmexit */
	set_cpu_cap(&pkvm_sym(boot_cpu_data), X86_FEATURE_RSB_VMEXIT_LITE);

	return true;
}

static bool mitigate_rfds(void)
{
	if (!(x86_read_arch_cap_msr() & ARCH_CAP_RFDS_CLEAR))
		return false;

	set_cpu_cap(&pkvm_sym(boot_cpu_data), X86_FEATURE_CLEAR_CPU_BUF);
	return true;
}

static bool mitigate_vmscape(struct cpuinfo_x86 *c)
{
	if (!__mitigate_spectre_v2_user())
		return false;

	if (boot_cpu_has_bug(X86_BUG_SPECTRE_V2_USER)) {
		clear_bit(X86_BUG_SPECTRE_V2_USER, (unsigned long *)c->x86_capability);
		pr_info("mitigated spectre_v2_user when mitigating vmscape\n");
	}

	return true;
}

/*
 * Make sure the CPU only with the bugs that can be mitigated by the pKVM
 * hypervisor can pass the check. And these mitigated CPU bugs are listed in
 * possible_cpu_bugs[].
 *
 * The assumption is that the linux kernel is trusted before deprivileging and
 * can report the CPU bugs/features precisely.
 *
 * Below are how these CPU bugs be mitigated by the pKVM hypervisor:
 *
 * 1) X86_BUG_SPECTRE_V1.
 * 1.1) usercopy/swapgs are not used. Thus can mitigate X86_BUG_SWAPGS.
 * 1.2) The array index passed from the host VM or the guest VM are sanitized
 * by array_index_nospec to prevent bypassing the bounds check due to CPU
 * speculation.
 *
 * 2) X86_BUG_SPECTRE_V2.
 * 2.1) Leverage hardware mitigation eIBRS feature; Set SPEC_CTRL_IBRS in
 * spec ctrl MSR.
 * 2.2) No context switch in pKVM hypervisor. No need to fill RSB.
 * 2.3) If eIBRS is affected by PBRSB, fill RSB for vmexits.
 * 2.4) Set SPEC_CTRL_BHI_DIS_S in spec ctrl MSR to mitigate BHI bug.
 *
 * 3) X86_BUG_SPECTRE_V2_USER.
 * Requires eIBRS (thus no need for STIBP) in case SMT is enabled at runtime.
 * Require IBPB feature to perform indirect branch prediction barrier when
 * switching vcpu.
 *
 * 4) X86_BUG_SPEC_STORE_BYPASS.
 * Set SPEC_CTRL_SSBD in spec ctrl MSR to mitigate.
 *
 * 5) X86_BUG_SWAPGS:
 * See comments for X86_BUG_SPECTRE_V1 1.1.
 *
 * 6) X86_BUG_BHI:
 * Set SPEC_CTRL_BHI_DIS_S in spec ctrl MSR to mitigate BHI bug.
 *
 * 7) X86_BUG_EIBRS_PBRSB.
 * Fill RSB after vmexit.
 *
 * 8) X86_BUG_RFDS.
 * Requires ARCH_CAP_RFDS_CLEAR to clears CPU register file via VERW.
 *
 * 9) X86_BUG_VMSCAPE
 * Requires eIBRS (thus no need for STIBP) in case SMT is enabled at runtime.
 * No need to perform IBPB before exit to the user space as the pKVM hypervisor
 * doesn't have. But the pKVM hypervisor can switch to the host VMM which runs
 * at the user space privilege level, and the host is untrusted to guarantee
 * this bug will be mitigated. Thus perform IBPB before switching from a guest
 * to the host to mitigate this bug. So the mitigation is the same with
 * X86_BUG_SPECTRE_V2_USER.
 *
 * Note: Beyond the above mitigations, the pKVM hypervisor also supports boot
 * time retpoline/rethunk patching to mitigate certain older CPU bugs (not
 * listed in the possible_cpu_bugs[]). The reason is that, the pKVM hypervisor
 * is part of linux kernel, and linux kernel could enable retpoline/rethunk
 * patching via kernel command line parameters even for a CPU which doesn't have
 * such bugs. With this, the kernel image (including the pkvm hypervisor) will
 * be patched with linux kernel's retpoline/rethunk symbols at the boot time. To
 * support this usage, the pKVM hypervisor should support retpoline/rethunk
 * patching with its own retpoline/rethunk symbols, otherwise it will not be
 * able to run due to isolation.
 */
static void pkvm_mitigate_cpu_bug(struct cpuinfo_x86 *c, unsigned long bug)
{
	bool mitigated = false;

	if (!boot_cpu_has(bug)) {
		pr_info("CPU doesn't have bug %s\n", x86_bug_flags[bug - NCAPINTS * 32]);
		return;
	}

	/*
	 * CPU has this bug but it is already mitigated when mitigating some
	 * other bug.
	 */
	if (!cpu_has_bug(c, bug))
		return;

	switch (bug) {
	case X86_BUG_SPECTRE_V1:
	case X86_BUG_SWAPGS:
		/* Guaranteed by the pKVM hypervisor code */
		mitigated = true;
		break;
	case X86_BUG_SPECTRE_V2:
		mitigated = mitigate_spectre_v2(c);
		break;
	case X86_BUG_SPECTRE_V2_USER:
		mitigated = mitigate_spectre_v2_user(c);
		break;
	case X86_BUG_SPEC_STORE_BYPASS:
		mitigated = mitigate_spec_store_bypass();
		break;
	case X86_BUG_BHI:
		mitigated = mitigate_bhi();
		break;
	case X86_BUG_EIBRS_PBRSB:
		mitigated = mitigate_eibrs_pbrsb();
		break;
	case X86_BUG_RFDS:
		mitigated = mitigate_rfds();
		break;
	case X86_BUG_VMSCAPE:
		mitigated = mitigate_vmscape(c);
		break;
	default:
		break;
	}

	if (mitigated) {
		clear_bit(bug, (unsigned long *)c->x86_capability);
		pr_info("mitigated CPU bug %s\n", x86_bug_flags[bug - NCAPINTS * 32]);
	} else {
		pr_err("cannot mitigate CPU bug %s\n", x86_bug_flags[bug - NCAPINTS * 32]);
	}
}

/*
 * The CPU bugs list based on Intel ADL/PTL CPU. Could be extended beyond those
 * CPUs in the future.
 */
static unsigned long possible_cpu_bugs[] = {
	X86_BUG_SPECTRE_V1,
	X86_BUG_SPECTRE_V2,
	X86_BUG_SPECTRE_V2_USER,
	X86_BUG_SPEC_STORE_BYPASS,
	X86_BUG_SWAPGS,
	X86_BUG_BHI,
	X86_BUG_EIBRS_PBRSB,
	X86_BUG_RFDS,
	X86_BUG_VMSCAPE,
};

static int pkvm_mitigate_cpu_bugs(void)
{
	struct cpuinfo_x86 c = boot_cpu_data;
	int i, unmitigated_cpu_bugs = 0;

	for (i = 0; i < ARRAY_SIZE(possible_cpu_bugs); i++)
		pkvm_mitigate_cpu_bug(&c, possible_cpu_bugs[i]);

	for_each_set_bit(i, (unsigned long *)&c.x86_capability[NCAPINTS], NBUGINTS * 32) {
		pr_err("unmitigated cpu bug %s\n", x86_bug_flags[i]);
		unmitigated_cpu_bugs++;
	}

	if (unmitigated_cpu_bugs) {
		pr_err("in total has %d unmitigated cpu bugs\n", unmitigated_cpu_bugs);
		return -EOPNOTSUPP;
	}

	return 0;
}

static inline u32 get_ar(u16 sel)
{
	u32 access_rights;

	if (sel == 0) {
		access_rights = 0x10000;
	} else {
		asm ("lar %%ax, %%rax\n"
				: "=a"(access_rights) : "a"(sel));
		access_rights = access_rights >> 8;
		access_rights = access_rights & 0xf0ff;
	}

	return access_rights;
}

#define init_guestsegment(seg, SEG, base, limit)		\
	do  {							\
		u16 sel;					\
		u32 ar;						\
								\
		savesegment(seg, sel);				\
		ar = get_ar(sel);				\
		vmcs_write16(GUEST_##SEG##_SELECTOR, sel);	\
		vmcs_write32(GUEST_##SEG##_AR_BYTES, ar);	\
		vmcs_writel(GUEST_##SEG##_BASE, base);		\
		vmcs_write32(GUEST_##SEG##_LIMIT, limit);	\
	} while (0)

static __init void init_guest_state_area_from_native(struct vcpu_vmx *vmx)
{
	int cpu = smp_processor_id();
	struct desc_ptr dt;
	u64 msrq;
	u16 ldtr;

	/* Initialize CR registers */
	vmcs_writel(GUEST_CR0, read_cr0() & ~X86_CR0_TS);
	vmcs_writel(GUEST_CR3, __read_cr3());
	vmcs_writel(GUEST_CR4, __read_cr4());

	/* Initialize cs/ss/ds/es */
	init_guestsegment(cs, CS, 0x0, 0xffffffff);
	init_guestsegment(ss, SS, 0x0, 0xffffffff);
	init_guestsegment(ds, DS, 0x0, 0xffffffff);
	init_guestsegment(es, ES, 0x0, 0xffffffff);

	/* Initialize fs/gs */
	rdmsrq(MSR_FS_BASE, msrq);
	init_guestsegment(fs, FS, msrq, 0xffffffff);
	rdmsrq(MSR_GS_BASE, msrq);
	init_guestsegment(gs, GS, msrq, 0xffffffff);

	/* Initialize GDTR */
	native_store_gdt(&dt);
	vmcs_writel(GUEST_GDTR_BASE, dt.address);
	vmcs_write32(GUEST_GDTR_LIMIT, dt.size);

	/* Initialize TR */
	vmcs_write16(GUEST_TR_SELECTOR, GDT_ENTRY_TSS*8);
	vmcs_write32(GUEST_TR_AR_BYTES, get_ar(GDT_ENTRY_TSS*8));
	vmcs_writel(GUEST_TR_BASE, (unsigned long)&get_cpu_entry_area(cpu)->tss.x86_tss);
	vmcs_write32(GUEST_TR_LIMIT, __KERNEL_TSS_LIMIT);

	/* Initialize LDTR */
	store_ldt(ldtr);
	vmcs_write16(GUEST_LDTR_SELECTOR, ldtr);
	vmcs_write32(GUEST_LDTR_AR_BYTES, 0x10000);
	vmcs_writel(GUEST_LDTR_BASE, 0x0);
	vmcs_write32(GUEST_LDTR_LIMIT, 0xffffffff);

	/* Initialize IDTR */
	store_idt(&dt);
	vmcs_writel(GUEST_IDTR_BASE, dt.address);
	vmcs_write32(GUEST_IDTR_LIMIT, dt.size);

	/* Set MSRs */
	vmcs_write64(GUEST_IA32_DEBUGCTL, get_debugctlmsr());

	rdmsrq(MSR_IA32_SYSENTER_CS, msrq);
	vmcs_write32(GUEST_SYSENTER_CS, (u32)msrq);

	rdmsrq(MSR_IA32_SYSENTER_ESP, msrq);
	vmcs_writel(GUEST_SYSENTER_ESP, msrq);

	rdmsrq(MSR_IA32_SYSENTER_EIP, msrq);
	vmcs_writel(GUEST_SYSENTER_EIP, msrq);

	rdmsrq(MSR_EFER, msrq);
	vmcs_write64(GUEST_IA32_EFER, msrq);

	rdmsrq(MSR_IA32_CR_PAT, msrq);
	vmcs_write64(GUEST_IA32_PAT, msrq);

	if (!rdmsrq_safe(MSR_CORE_PERF_GLOBAL_CTRL, &msrq)) {
		struct kvm_pmu *pmu = vcpu_to_pmu(&vmx->vcpu);
		union cpuid10_eax eax = {
			.full = native_cpuid_eax(10),
		};

		pmu->version = eax.split.version_id;
		pmu->global_ctrl = msrq;
		vmcs_write64(GUEST_IA32_PERF_GLOBAL_CTRL, msrq);
	}

	if (boot_cpu_has(X86_FEATURE_INTEL_PT)) {
		rdmsrq(MSR_IA32_RTIT_CTL, msrq);
		vmcs_write64(GUEST_IA32_RTIT_CTL, msrq);
	}

	if (boot_cpu_has(X86_FEATURE_ARCH_LBR)) {
		rdmsrq(MSR_ARCH_LBR_CTL, msrq);
		vmcs_write64(GUEST_IA32_LBR_CTL, msrq);
	}

	if (boot_cpu_has(X86_FEATURE_IBT) || boot_cpu_has(X86_FEATURE_SHSTK)) {
		rdmsrq(MSR_IA32_S_CET, msrq);
		vmcs_writel(GUEST_S_CET, msrq);
		/*
		 * Supervisor shadow stack is guaranteed not to be enabled. See
		 * comments in pkvm_setup_host_vmcs_config.
		 */
		if (boot_cpu_has(X86_FEATURE_SHSTK)) {
			vmcs_writel(GUEST_SSP, 0);
			vmcs_writel(GUEST_INTR_SSP_TABLE, 0);
		}
	}
}

static __init void init_guest_state_area(struct vcpu_vmx *vmx)
{
	init_guest_state_area_from_native(vmx);

	/*Guest non register state*/
	vmcs_write32(GUEST_ACTIVITY_STATE, GUEST_ACTIVITY_ACTIVE);
	vmcs_write32(GUEST_INTERRUPTIBILITY_INFO, 0);
	vmcs_writel(GUEST_PENDING_DBG_EXCEPTIONS, 0);
	vmcs_write64(VMCS_LINK_POINTER, -1ull);
}

static __init void init_host_state_area(struct vcpu_vmx *vmx)
{
	struct pkvm_pcpu *pcpu = this_cpu_read(pkvm_pcpu);
	int cpu = smp_processor_id();
	unsigned long host_rsp;
#ifdef CONFIG_PKVM_X86_DEBUG
	struct desc_ptr dt;
	u16 selector;
#endif
	u64 msrq;

	vmcs_writel(HOST_CR0, read_cr0() & ~X86_CR0_TS);
	/* Use host cr3 until the pKVM hypervisor created its own MMU */
	vmcs_writel(HOST_CR3, __read_cr3());
	/*
	 * Disable FRED for the pKVM hypervisor if it is enabled by the host.
	 * There is no too much benifit for the pKVM hypervisor to use the FRED
	 * event delivery as the NMI is the only event expected to be received
	 * by the pKVM hypervisor. The exceptions are not expected to be
	 * happened in the pKVM hypervisor and all hardware interrupts will
	 * directly go to the host. Meanwhile, enabling the FRED in the pkvm
	 * hypervisor will result in additional FRED MSRs switching overhead. So
	 * keep the FRED being disabled in the pKVM hypervisor.
	 */
	vmcs_writel(HOST_CR4, __read_cr4() & ~X86_CR4_FRED);

#ifdef CONFIG_PKVM_X86_DEBUG
	savesegment(cs, selector);
	vmcs_write16(HOST_CS_SELECTOR, selector);
	savesegment(ss, selector);
	vmcs_write16(HOST_SS_SELECTOR, selector);
	savesegment(ds, selector);
	vmcs_write16(HOST_DS_SELECTOR, selector);
	savesegment(es, selector);
	vmcs_write16(HOST_ES_SELECTOR, selector);
	savesegment(fs, selector);
	vmcs_write16(HOST_FS_SELECTOR, selector);
	rdmsrq(MSR_FS_BASE, msrq);
	vmcs_writel(HOST_FS_BASE, msrq);
	savesegment(gs, selector);
	vmcs_write16(HOST_GS_SELECTOR, selector);
	rdmsrq(MSR_GS_BASE, msrq);
	vmcs_writel(HOST_GS_BASE, msrq);

	vmcs_write16(HOST_TR_SELECTOR, GDT_ENTRY_TSS*8);
	vmcs_writel(HOST_TR_BASE, (unsigned long)&get_cpu_entry_area(cpu)->tss.x86_tss);

	native_store_gdt(&dt);
	vmcs_writel(HOST_GDTR_BASE, dt.address);

	/*
	 * Use pKVM's exception handlers, to minimize differences from
	 * non-debug mode.
	 */
	vmcs_writel(HOST_IDTR_BASE, (unsigned long)(&pcpu->idt_page));

	rdmsrq(MSR_IA32_SYSENTER_CS, msrq);
	vmcs_write32(HOST_IA32_SYSENTER_CS, (u32)msrq);

	rdmsrq(MSR_IA32_SYSENTER_ESP, msrq);
	vmcs_writel(HOST_IA32_SYSENTER_ESP, msrq);

	rdmsrq(MSR_IA32_SYSENTER_EIP, msrq);
	vmcs_writel(HOST_IA32_SYSENTER_EIP, msrq);
#else
	vmcs_write16(HOST_CS_SELECTOR, __KERNEL_CS);
	vmcs_write16(HOST_SS_SELECTOR, __KERNEL_DS);
	vmcs_write16(HOST_DS_SELECTOR, __KERNEL_DS);
	vmcs_write16(HOST_ES_SELECTOR, 0);
	vmcs_write16(HOST_TR_SELECTOR, GDT_ENTRY_TSS*8);
	vmcs_write16(HOST_FS_SELECTOR, 0);
	vmcs_write16(HOST_GS_SELECTOR, 0);
	vmcs_writel(HOST_FS_BASE, 0);
	vmcs_writel(HOST_GS_BASE, pkvm_sym(pkvm_per_cpu_offset)(cpu));

	vmcs_writel(HOST_TR_BASE, (unsigned long)&pcpu->tss);
	vmcs_writel(HOST_GDTR_BASE, (unsigned long)(&pcpu->gdt_page));
	vmcs_writel(HOST_IDTR_BASE, (unsigned long)(&pcpu->idt_page));
#endif

	rdmsrq(MSR_EFER, msrq);
	vmcs_write64(HOST_IA32_EFER, msrq);

	rdmsrq(MSR_IA32_CR_PAT, msrq);
	vmcs_write64(HOST_IA32_PAT, msrq);

	vmcs_write64(HOST_IA32_PERF_GLOBAL_CTRL, 0);

	if (boot_cpu_has(X86_FEATURE_IBT) || boot_cpu_has(X86_FEATURE_SHSTK)) {
		rdmsrq(MSR_IA32_S_CET, msrq);
		vmcs_writel(HOST_S_CET, msrq);
		/*
		 * Supervisor shadow stack is guaranteed not to be enabled. See
		 * comments in pkvm_setup_host_vmcs_config.
		 */
		if (boot_cpu_has(X86_FEATURE_SHSTK)) {
			vmcs_writel(HOST_SSP, 0);
			vmcs_writel(HOST_INTR_SSP_TABLE, 0);
		}
	}

	/*
	 * [pcpu->stack, pcpu->stack + PKVM_STACK_SIZE) is per cpu stack.
	 * It is used as stack when the pcpu enters pKVM, i.e. HOST stack from
	 * VMX point of view.
	 *
	 * Within the top of stack, a small region starting from stack_resv
	 * is reserved  to store private paremeters,
	 *
	 * ------------ Stack layout ----------
	 * stack_top:
	 * stack_resv + 8:	struct vcpu_vmx *vmx
	 * stack_resv + 0:	pointer to vcpu->arch.regs
	 * stack_resv:		(stack_top - PKVM_STACK_TOP_RESV) = VMCS.HOST_RSP for PCPU
	 *			.........
	 *			.........
	 * stack_bottom:
	 */
	host_rsp = get_host_stack_top(pcpu) - PKVM_STACK_TOP_RESV;

	vmcs_writel(HOST_RSP, host_rsp);
	*((struct vcpu_vmx **) (host_rsp + 8)) = vmx;
	*((unsigned long **) host_rsp) = vmx->vcpu.arch.regs;

	vmcs_writel(HOST_RIP, (unsigned long)pkvm_sym(pkvm_host_vmexit_entry));
}

static __init void init_execution_control(struct vcpu_vmx *vmx)
{
	/* Preemption timer is toggled dynamically */
	pin_controls_set(vmx, pkvm_sym(host_vmcs_config).pin_based_exec_ctrl &
			      ~PIN_BASED_VMX_PREEMPTION_TIMER);

	/*
	 * CR3 LOAD/STORE EXITING are always read as 1 from the
	 * MSR_IA32_VMX_PROCBASED_CTLS. Clear these two bits as the CR3 will be
	 * passthrough to the host VM.
	 * INTR WINDOW EXITING is toggled dynamically.
	 */
	exec_controls_set(vmx, pkvm_sym(host_vmcs_config).cpu_based_exec_ctrl &
			       ~(CPU_BASED_CR3_LOAD_EXITING |
				 CPU_BASED_CR3_STORE_EXITING |
				 CPU_BASED_INTR_WINDOW_EXITING));

	/* Disable EPT/VPID first, enable after EPT pgtable created */
	secondary_exec_controls_set(vmx, pkvm_sym(host_vmcs_config).cpu_based_2nd_exec_ctrl &
					 ~(SECONDARY_EXEC_ENABLE_EPT |
					   SECONDARY_EXEC_ENABLE_VPID));

	/*
	 * The SECONDARY_EXEC_PT_USE_GPA bit is depending on the
	 * SECONDARY_EXEC_ENABLE_EPT bit. Remove it at this point
	 * and re-enable it after the EPT is enabled.
	 */
	if (boot_cpu_has(X86_FEATURE_INTEL_PT))
		secondary_exec_controls_clearbit(vmx, SECONDARY_EXEC_PT_USE_GPA);

	/* Host VM owns cr3 */
	vmcs_write32(CR3_TARGET_COUNT, 0);

	/* Host VM handles exceptions directly */
	vmcs_write32(EXCEPTION_BITMAP, 0);

	vmcs_write64(MSR_BITMAP, __pa(vmx->vmcs01.msr_bitmap));

	/*
	 * Host VM owns cr0 and cr4 except VMXE bit.
	 * Does not care about IA32_VMX_CRx_FIXED0/1 setting, so if host VM
	 * modifies cr0/cr4 conflicting with FIXED0/1, just let #GP happen.
	 * For example, as pKVM does not enable unrestricted guest feature,
	 * cr0.PE/PG must keep as 1 in host VM.
	 */
	vmcs_writel(CR0_GUEST_HOST_MASK, 0);
	vmcs_writel(CR4_GUEST_HOST_MASK, X86_CR4_VMXE);

	/*
	 * Set the VMXE bit in CR4_READ_SHADOW so that the host VM will see the
	 * consistent values between "native" cr4 and its cached cpu_tlbstate.cr4
	 * (which is set when turns on VMX via kvm_cpu_vmxon).
	 */
	vmcs_writel(CR4_READ_SHADOW, X86_CR4_VMXE);
}

static __init void init_vmexit_control(struct vcpu_vmx *vmx)
{
	u32 vmexit_ctrl = pkvm_sym(host_vmcs_config).vmexit_ctrl;
	struct kvm_pmu *pmu = vcpu_to_pmu(&vmx->vcpu);

	/* No need to switch if PMU is not enabled */
	if (!pmu->global_ctrl)
		vmexit_ctrl &= ~VM_EXIT_LOAD_IA32_PERF_GLOBAL_CTRL;

	vm_exit_controls_set(vmx, vmexit_ctrl);
	vmcs_write32(VM_EXIT_MSR_STORE_COUNT, 0);
}

static __init void init_vmentry_control(struct vcpu_vmx *vmx)
{
	u32 vmentry_ctrl = pkvm_sym(host_vmcs_config).vmentry_ctrl;
	struct kvm_pmu *pmu = vcpu_to_pmu(&vmx->vcpu);

	/* No need to switch if PMU is not enabled */
	if (!pmu->global_ctrl)
		vmentry_ctrl &= ~VM_ENTRY_LOAD_IA32_PERF_GLOBAL_CTRL;

	if (boot_cpu_has(X86_FEATURE_INTEL_PT)) {
		u64 rtit;

		rdmsrq(MSR_IA32_RTIT_CTL, rtit);
		if (rtit & RTIT_CTL_TRACEEN) {
			/*
			 * According to SDM Vol.3 VM-Execution Control Fields:
			 * If the logical processor is operating with Intel PT
			 * enabled (if IA32_RTIT_CTL.TraceEn = 1) at the time of
			 * VM entry, the “load IA32_RTIT_CTL” VM-entry control
			 * must be 0.
			 *
			 * So need to clear the VM_ENTRY_LOAD_IA32_RTIT_CTL bit
			 * and set it back after VM-Exit.
			 */
			vmentry_ctrl &= ~VM_ENTRY_LOAD_IA32_RTIT_CTL;
		}
	}

	vm_entry_controls_set(vmx, vmentry_ctrl);
	vmcs_write32(VM_ENTRY_INTR_INFO_FIELD, 0);
	vmcs_write32(VM_ENTRY_MSR_LOAD_COUNT, 0);
}

static __init int pkvm_host_init_vmx(struct vcpu_vmx *vmx)
{
	vmx->loaded_vmcs = &vmx->vmcs01;
	vmcs_clear(vmx->loaded_vmcs->vmcs);
	vmcs_load(vmx->loaded_vmcs->vmcs);
	vmx->loaded_vmcs->cpu = smp_processor_id();

	init_guest_state_area(vmx);
	init_host_state_area(vmx);
	init_execution_control(vmx);
	init_vmexit_control(vmx);
	init_vmentry_control(vmx);

	return 0;
}

static noinline int local_deprivilege_cpu(void)
{
	int ret;

	asm volatile(
		"pushfq\n"
		"popq %%rax\n"
		"movq %3, %%rdx\n"
		"vmwrite %%rax, %%rdx\n"
		"movq %%rsp, %%rax\n"
		"movq %4, %%rdx\n"
		"vmwrite %%rax, %%rdx\n"
		"movq $host_vm_entry_point, %%rax\n"
		"movq %1, %%rdx\n"
		"vmwrite %%rax, %%rdx\n"
		"movl $0, %0\n"
		"vmlaunch\n"
		/* vmlaunch failed */
		"movl %2, %0\n"
		/* successfully deprivileged */
		"host_vm_entry_point: nop\n"
		: "=m"(ret)
		: "i"(GUEST_RIP), "i"(-EINVAL), "i"(GUEST_RFLAGS), "i"(GUEST_RSP)
		: "rax", "rdx", "memory");

	return ret;
}

static DEFINE_PER_CPU(bool, deprivileged);
static __init void pkvm_host_reprivilege_cpu(void *data)
{
	int cpu = smp_processor_id();
	int ret;

	if (!this_cpu_read(deprivileged))
		return;

	/*
	 * Load the RW GDT page for reprivilege code
	 * to reload TR.
	 */
	load_direct_gdt(cpu);

	/*
	 * Intel CET requires indirect jmp/call to return to
	 * endbr64 instruction. So we can't use kvm_hypercall
	 * here.
	 */
	asm volatile(
		"vmcall\n"
		"endbr64\n"
		: "=a"(ret)
		: "a"(__pkvm__reprivilege_cpu)
		: "memory");

	/* Switch back to RO GDT page */
	load_fixmap_gdt(cpu);

	if (!ret) {
		this_cpu_write(deprivileged, false);
		kvm_cpu_vmxoff();
		pr_info("%s: CPU%d back in host mode\n", __func__, cpu);
	}

	*(int *)data = ret;
}

static __init void pkvm_host_reprivilege_cpus(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		int ret, reprivilege_ret = 0;

		if (!per_cpu(deprivileged, cpu))
			continue;

		ret = smp_call_function_single(cpu, pkvm_host_reprivilege_cpu,
					       &reprivilege_ret, true);
		if (ret || reprivilege_ret)
			panic("CPU%d failed to reprivilege(smp_call=%d, reprivilege=%d)\n",
			      cpu, ret, reprivilege_ret);
	}
}

static __init void pkvm_host_deprivilege_cpu(void *data)
{
	int cpu = smp_processor_id(), *deprivilege_ret = data, ret;
	struct kvm_vcpu *vcpu = this_cpu_read(host_vcpu);

	ret = kvm_cpu_vmxon(__pa(this_cpu_read(pkvm_vmxarea)));
	if (ret) {
		pr_err("CPU%d vmxon failed, ret %d\n", cpu, ret);
		goto done;
	}

	ret = pkvm_host_init_vmx(to_vmx(vcpu));
	if (ret) {
		pr_err("CPU%d init vmx failed, ret %d\n", cpu, ret);
		goto vmxoff;
	}

	ret = local_deprivilege_cpu();
	if (ret) {
		pr_err("CPU%d deprivilege failed, ret %d\n", cpu, ret);
		goto vmxoff;
	}

	vcpu->mode = IN_GUEST_MODE;
	this_cpu_write(deprivileged, true);
	pr_info("CPU%d in guest mode\n", cpu);
	return;
vmxoff:
	kvm_cpu_vmxoff();
done:
	*deprivilege_ret = ret;
}

/*
 * Used in root mode to deprivilege CPUs
 */
static __init int pkvm_host_deprivilege_cpus(struct pkvm_hyp *pkvm)
{
	int cpu, ret = 0, deprivilege_ret = 0;

	pkvm_sym(pkvm_vmx_register_excp_handlers)();

	/*
	 * The pKVM hypervisor's IDT will be programmed into VMCS before
	 * deprivileging the CPU. Once deprivileging is done and the CPU
	 * enters to the root mode, the pKVM's exception handlers should be
	 * functional. So before that, sort pKVM's exception table to make
	 * sure the exception fixup working as expected.
	 */
	if (&pkvm_sym(__stop___ex_table) > &pkvm_sym(__start___ex_table))
		sort_extable(pkvm_sym(__start___ex_table), pkvm_sym(__stop___ex_table));

	for_each_possible_cpu(cpu) {
		ret = smp_call_function_single(cpu, pkvm_host_deprivilege_cpu,
					       &deprivilege_ret, 1);
		if (ret || deprivilege_ret) {
			pr_err("Failed to deprivilege CPU%d: smp_call %d, deprivilege: %d\n",
			       cpu, ret, deprivilege_ret);
			break;
		}
	}

	return ret ? ret : deprivilege_ret;
}

static void do_pkvm_hyp_init(void *data)
{
	unsigned long data_size = data_pages << PAGE_SHIFT;
	struct pkvm_mem_info infos[] = {
		{
			.type	= PKVM_RESERVED_USED_MEMORY,
			.va	= (unsigned long)__va(pkvm_mem_base),
			.pa	= pkvm_mem_base,
			.size	= data_size,
			.prot	= pgprot_val(PAGE_KERNEL),
		},
		{
			.type	= PKVM_RESERVED_UNUSED_MEMORY,
			.va	= (unsigned long)__va(pkvm_mem_base + data_size),
			.pa	= pkvm_mem_base + data_size,
			.size	= pkvm_mem_size - data_size,
			.prot	= pgprot_val(PAGE_KERNEL),
		},
		{
			.type	= PKVM_TEXT_DATA,
			.va	= (unsigned long)pkvm_sym(text_start),
			.pa	= __pa_symbol(pkvm_sym(text_start)),
			.size	= pkvm_sym(text_end) - pkvm_sym(text_start),
			.prot	= pgprot_val(PAGE_KERNEL_EXEC),
		},
		{
			.type	= PKVM_TEXT_DATA,
			.va	= (unsigned long)pkvm_sym(rodata_start),
			.pa	= __pa_symbol(pkvm_sym(rodata_start)),
			.size	= pkvm_sym(rodata_end) - pkvm_sym(rodata_start),
			.prot	= pgprot_val(PAGE_KERNEL_RO),
		},
		{
			.type	= PKVM_TEXT_DATA,
			.va	= (unsigned long)pkvm_sym(data_start),
			.pa	= __pa_symbol(pkvm_sym(data_start)),
			.size	= pkvm_sym(data_end) - pkvm_sym(data_start),
			.prot	= pgprot_val(PAGE_KERNEL),
		},
		{
			.type	= PKVM_TEXT_DATA,
			.va	= (unsigned long)pkvm_sym(bss_start),
			.pa	= __pa_symbol(pkvm_sym(bss_start)),
			.size	= pkvm_sym(bss_end) - pkvm_sym(bss_start),
			.prot	= pgprot_val(PAGE_KERNEL),
		},
	};
	int ret = pkvm_hypercall(init, (unsigned long)infos, ARRAY_SIZE(infos));

	if (data)
		*(int *)data = ret;
}

static __init int pkvm_hyp_init(void)
{
	int ret, cpu, init_ret;

	for_each_possible_cpu(cpu) {
		ret = smp_call_function_single(cpu, do_pkvm_hyp_init,
					       &init_ret, 1);
		if (ret || init_ret) {
			pr_err("Failed to initialize pKVM hyp on CPU%d: smp_call %d, init: %d\n",
			       cpu, ret, init_ret);
			break;
		}
	}

	return ret ? ret : init_ret;
}

static int __init pkvm_firmware_rmem_init(void)
{
	phys_addr_t start, end, size;

	if (!pvmfw_present)
		return 0;

	start = pvmfw_base;
	end = pvmfw_base + pvmfw_size - 1;
	size = pvmfw_size;

	if (!e820__mapped_all(start, end, E820_TYPE_RESERVED)) {
		pr_err("pvmfw memory [0x%llx-0x%llx] is not reserved in e820\n",
		       start, end);
		pvmfw_present = false;
		return -EINVAL;
	}

	if (!PAGE_ALIGNED(start) || !PAGE_ALIGNED(size)) {
		pr_err("pvmfw memory [0x%llx-0x%llx] is not page-aligned\n",
		       start, end);
		pvmfw_present = false;
		return -EINVAL;
	}

	pkvm_sym(pvmfw_present) = true;
	pkvm_sym(pvmfw_base) = start;
	pkvm_sym(pvmfw_size) = size;
	return 0;
}

static int __init pkvm_firmware_rmem_clear(void)
{
	void *addr;
	phys_addr_t size;

	if (!pvmfw_present)
		return 0;

	size = pvmfw_size;
	addr = memremap(pvmfw_base, size, MEMREMAP_WB);
	if (!addr)
		return -EINVAL;

	memset(addr, 0, size);
	clflush_cache_range(addr, size);
	memunmap(addr);

	pr_info("Cleared pvmfw memory\n");
	return 0;
}

int __init vmx_pkvm_init(void)
{
	struct pkvm_hyp *pkvm;
	int ret, cpu;

	pkvm_firmware_rmem_init();

	if (!enable_pkvm) {
		pkvm_firmware_rmem_clear();
		return 0;
	}

	if (!tsc_khz) {
		pr_err("TSC frequency not calibrated\n");
		ret = -ENODEV;
		goto out;
	}

	if (!pkvm_mem_base) {
		pr_err("required memory not reserved\n");
		ret = -ENOMEM;
		goto out;
	}

	data_pages = pkvm_vmx_data_pages();
	pkvm_sym(pkvm_early_alloc_init)(__va(pkvm_mem_base), data_pages << PAGE_SHIFT);

	pkvm = pkvm_sym(pkvm_hyp) = pkvm_sym(pkvm_early_alloc_contig)(PKVM_HYP_PAGES);
	if (!pkvm) {
		pr_err("cannot alloc pkvm_hyp\n");
		ret = -ENOMEM;
		goto out;
	}

	pkvm_setup_syms();

	ret = pkvm_setup_host_vmcs_config();
	if (ret) {
		pr_err("setup host vmcs config failed\n");
		goto out;
	}

	ret = pkvm_setup_host_vm(pkvm);
	if (ret)
		goto out;

	pkvm->num_cpus = 0;

	for_each_possible_cpu(cpu) {
		ret = pkvm_setup_pcpu(cpu);
		if (ret)
			goto out;

		ret = pkvm_setup_host_vcpu(pkvm->host_kvm, cpu);
		if (ret)
			goto out;

		ret = pkvm_setup_per_cpu(cpu);
		if (ret)
			goto out;

		pkvm->pcpus[pkvm->num_cpus] = per_cpu(pkvm_pcpu, cpu);
		pkvm->host_vcpus[pkvm->num_cpus] = per_cpu(host_vcpu, cpu);
		pkvm->num_cpus++;
	}

	/*
	 * Check if there is any CPU bug which cannot be mitigated by the pkvm
	 * hypervisor. As this may need to set the pkvm's per-cpu spec ctrl, do
	 * this after pkvm's per-cpu has been initialized.
	 */
	ret = pkvm_mitigate_cpu_bugs();
	if (ret) {
		if (!relax_cpu_bugs) {
			pr_err("prevent pkvm from running due to unmitigated CPU bugs\n");
			goto out;
		}
		pr_warn("allow pkvm to run with unmitigated CPU bugs\n");
		pr_warn("to prevent pkvm running on such CPU, ");
		pr_cont("reboot with kvm-intel.pkvm_relax_cpu_bugs=false\n");
	}

	ret = pkvm_host_prepare_iommu();
	if (ret)
		goto out;

	pkvm_sym(init_ops) = pkvm_sym(pkvm_vmx_init_ops);

	pkvm_ramoops_init();

	ret = pkvm_host_deprivilege_cpus(pkvm);
	if (ret)
		goto repriv_cpus;

	ret = pkvm_hyp_init();
	if (ret)
		goto repriv_cpus;
	static_branch_enable(&pkvm_enabled_key);

	ret = pkvm_host_init_iommu();
	if (ret) {
		static_branch_disable(&pkvm_enabled_key);
		goto repriv_cpus;
	}

	/*
	 * After host deprivileging succeed, un-present the kernel direct
	 * mappings for the memory pages which are reserved from the memblock
	 * for the pKVM as they are not accessible to the host kernel until the
	 * platform is power cycled. This can avoid unnecessary EPT violation
	 * vmexit for the usage of load_unaligned_zeropad().
	 *
	 * Note: The host memory pages donated to the pKVM are still mapped in
	 * the host's MMU. Those pages are not un-presented right now because
	 * they are sparse allocated from the linux, un-presenting from the
	 * kernel direct mapping may split a huge PTE into smaller ones which
	 * may slightly impact the host's performance. Without unpresenting for
	 * those pages, the usage of load_unaligned_zeropad() can be supported
	 * via injecting #PF by the pKVM.
	 */
	WARN_ON(set_memory_np((unsigned long)__va(pkvm_mem_base),
			      pkvm_mem_size >> PAGE_SHIFT));

	pkvm_hypercall(init_finalize);

	pkvm_init_debugfs();

	pr_info("Hypervisor is up and running!\n");
	return 0;

repriv_cpus:
	pkvm_host_reprivilege_cpus();
out:
	/*
	 * TODO: clear pvmfw before reprivileging, not after, to ensure clearing
	 * it even if the system gets stuck at reprivileging due to possible bugs
	 * in the reprivileging code. To be able to do that, need to let the
	 * hypervisor restore the host's access to the pvmfw memory, so that we
	 * can clear it while we are still in VMX non-root.
	 */
	pkvm_firmware_rmem_clear();

	/* TODO: Try re-initialize IOMMU */

	/*
	 * As the reserved memory at the pkvm_mem_base will not be
	 * released back to the host, no need to de-initialize or
	 * free for the early_alloc.
	 */
	pkvm_sym(pkvm_hyp) = NULL;
	enable_pkvm = false;
	return ret;
}

MODULE_LICENSE("GPL");
