/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PKVM_X86_UNDEF_H
#define __PKVM_X86_UNDEF_H

/*
 * Special hack: pKVM runs in the highest privilege level, which is higher than
 * the linux kernel. This means that pKVM cannot use any of the linux kernel
 * symbols. To make pKVM being able to use the linux kernel headers without
 * introducing additional symbols, some kernel configuration options are
 * disabled. (This list needs to be extended when new variants are added.)
 */
#ifndef CONFIG_PKVM_X86_DEBUG
#undef CONFIG_PRINTK
#endif
#undef CONFIG_DEBUG_VIRTUAL
#undef CONFIG_CALL_THUNKS_DEBUG
#undef CONFIG_PREEMPT_DYNAMIC
#undef CONFIG_PARAVIRT
#undef CONFIG_PARAVIRT_XXL
#undef CONFIG_PARAVIRT_SPINLOCKS
#undef CONFIG_KVM_INTEL_TDX
#undef CONFIG_PREEMPT_COUNT
#undef CONFIG_X86_SGX_KVM
#undef CONFIG_USE_X86_SEG_SUPPORT
#undef CONFIG_MATH_EMULATION
#undef CONFIG_X86_DEBUG_FPU
#undef CONFIG_PROVE_LOCKING
#undef CONFIG_DEBUG_IRQFLAGS

#define NOTRACE
#define __NO_FORTIFY

/*
 * Avoid undefining the below options, as it would change the layout of some
 * of the kernel's data structures. Most importantly this applies to structs
 * that are shared between the host kernel and pKVM, e.g. struct kvm and
 * struct kvm_vcpu. They absolutely must have the same size and layout in the
 * host KVM and in pKVM.
 *
 * But even for structs that are not shared between host and pKVM, such as
 * task_struct or vm_area_struct, it is still better to avoid changing their
 * layout. For instance, with CONFIG_DEBUG_INFO_BTF enabled, if those structs
 * are different in the host and in pKVM, multiple BTF type IDs are generated
 * for them, and then the resolve_btfids tool warns about multiple IDs found
 * and uses just one of them when adding the needed debug info to the vmlinux,
 * and there is no guarantee that it will use the correct type ID, i.e. the
 * host's one, not the pKVM's one.
 *
 * The following can be used (with CONFIG_DEBUG_INFO_BTF=y) to inspect layouts
 * of structs in the kernel image, for checking if there are no multiple
 * layouts of the same struct caused by pKVM mangling the config options:
 *
 *     make -C tools/bpf/bpftool
 *     tools/bpf/bpftool/bpftool btf dump file vmlinux.unstripped
 */
/*
#undef CONFIG_BUG
#undef CONFIG_GENERIC_BUG
#undef CONFIG_TRACEPOINTS
#undef CONFIG_DEBUG_PREEMPT
#undef CONFIG_DYNAMIC_DEBUG
#undef CONFIG_DYNAMIC_DEBUG_CORE
*/

#endif /* __PKVM_X86_UNDEF_H */
