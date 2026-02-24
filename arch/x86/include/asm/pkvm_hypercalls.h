/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PKVM_HC
BUILD_BUG_ON(1)
#endif

#ifndef PKVM_HC_OUT
#define PKVM_HC_OUT PKVM_HC
#endif

#ifndef PKVM_HC_IN
#define PKVM_HC_IN PKVM_HC
#endif

/* Hypercalls used only during pKVM initialization */
PKVM_HC(init)
PKVM_HC(init_finalize)
PKVM_HC(reprivilege_cpu)

/* pKVM vmexit tracing/profiling */
PKVM_HC(enable_vmexit_trace)
PKVM_HC(dump_vmexit_trace)

/* KVM ops */
PKVM_HC(check_processor_compatibility)
PKVM_HC(enable_virtualization_cpu)
PKVM_HC(vm_init)
PKVM_HC(vm_finalize)
PKVM_HC_OUT(vm_destroy)
PKVM_HC(vcpu_create)
PKVM_HC_OUT(vcpu_free)
PKVM_HC(vcpu_load)
PKVM_HC(vcpu_put)
PKVM_HC(vcpu_reset)
PKVM_HC(update_exception_bitmap)
PKVM_HC(set_efer)
PKVM_HC(set_msr)
PKVM_HC_OUT(get_msr)
PKVM_HC_OUT(cache_reg)
PKVM_HC(set_cr4)
PKVM_HC(set_cr0)
PKVM_HC_OUT(get_rflags)
PKVM_HC(set_rflags)
PKVM_HC(set_dr7)
PKVM_HC_IN(set_segment)
PKVM_HC_OUT(get_segment)
PKVM_HC_OUT(get_segment_base)
PKVM_HC_IN(set_idt)
PKVM_HC_OUT(get_idt)
PKVM_HC_IN(set_gdt)
PKVM_HC_OUT(get_gdt)
PKVM_HC(flush_tlb_all)
PKVM_HC(flush_tlb_current)
PKVM_HC(flush_tlb_gva)
PKVM_HC(flush_tlb_guest)
PKVM_HC(set_interrupt_shadow)
PKVM_HC_OUT(get_interrupt_shadow)
PKVM_HC(enable_nmi_window)
PKVM_HC(enable_irq_window)
PKVM_HC(interrupt_allowed)
PKVM_HC(nmi_allowed)
PKVM_HC_OUT(get_nmi_mask)
PKVM_HC(set_nmi_mask)
PKVM_HC(inject_irq)
PKVM_HC(inject_nmi)
PKVM_HC(inject_exception)
PKVM_HC(cancel_injection)
PKVM_HC(update_cr8_intercept)
PKVM_HC(set_virtual_apic_mode)
PKVM_HC(refresh_apicv_exec_ctrl)
PKVM_HC(load_eoi_exitmap)
PKVM_HC(hwapic_isr_update)
PKVM_HC(sync_pir_to_irr)
PKVM_HC_OUT(vcpu_after_set_cpuid)
PKVM_HC_OUT(vcpu_add_fpstate)
PKVM_HC(write_tsc_offset)
PKVM_HC(write_tsc_multiplier)
PKVM_HC(load_mmu_pgd)
PKVM_HC(setup_mce)
PKVM_HC_OUT(vcpu_run)
PKVM_HC(complete_emulated_msr)
PKVM_HC(has_wbinvd_exit)

/* KVM MMU hypercalls */
PKVM_HC(vm_mmu_map)
PKVM_HC(vm_mmu_unmap)
PKVM_HC(vm_mmu_age)

#undef PKVM_HC
#undef PKVM_HC_OUT
#undef PKVM_HC_IN
