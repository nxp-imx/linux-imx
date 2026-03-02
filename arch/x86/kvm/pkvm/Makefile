# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I $(srctree)/arch/x86/kvm
ccflags-y += -I $(srctree)/arch/x86/kvm/pkvm
# TODO: make the stack protector work with the pKVM hypervisor code
ccflags-y += -fno-stack-protector
ccflags-y += -D__PKVM_HYP__
ccflags-y += -include $(srctree)/arch/x86/kvm/pkvm/undef.h

asflags-y += -D__PKVM_HYP__
asflags-y += -include $(srctree)/arch/x86/kvm/pkvm/undef.h

cmd_deps			:= $(src)/compiling_cmds.sh

pkvm-hyp-y			:= early_alloc.o pkvm.o memory.o cpu.o \
				   idt.o entry.o kern_stub.o init.o pgtable.o \
				   mmu.o page_alloc.o lapic.o trace.o fpu.o
pkvm-lib			:= lib
pkvm-hyp-y			+= $(pkvm-lib)/find_bit.o

kernel-x86-lib			:= ../../lib
pkvm-hyp-y			+= $(kernel-x86-lib)/memset_64.o
pkvm-hyp-y			+= $(kernel-x86-lib)/memcpy_64.o

pkvm-hyp-$(CONFIG_MITIGATION_RETPOLINE)	+= retpoline.o
$(obj)/retpoline.pkvm.o: $(obj)/retpoline.pkvm.S FORCE
	$(call if_changed_rule,as_o_S)
$(obj)/retpoline.pkvm.S: $(srctree)/arch/x86/lib/retpoline.S $(cmd_deps) FORCE
	$(call if_changed,patch_retpoline)
quiet_cmd_patch_retpoline = PATCH_RETPOLINE $@
      cmd_patch_retpoline = $(BASH) $(src)/compiling_cmds.sh patch_retpoline $< $@

kernel-lib			:= ../../../../lib
pkvm-hyp-y			+= $(kernel-lib)/sort.o $(kernel-lib)/bsearch.o \
				   $(kernel-lib)/extable.o $(kernel-lib)/hweight.o \
				   $(kernel-lib)/string.o $(kernel-lib)/ctype.o
pkvm-hyp-$(CONFIG_LIST_HARDENED) += $(kernel-lib)/list_debug.o

kvm				:= ..
pkvm-hyp-y			+= $(kvm)/x86.o $(kvm)/cpuid.o $(kvm)/mtrr.o \
				   $(kvm)/lapic.o

pkvm-hyp-$(CONFIG_PKVM_INTEL)	+= vmx/host_vmentry.o vmx/host_vmx.o \
				   $(kvm)/vmx/vmx.o vmx/idt.o vmx/ept.o \
				   vmx/host_repriv.o $(kvm)/vmx/main.o \
				   $(kvm)/vmx/vmenter.o

AFLAGS_$(kvm)/vmx/vmenter.pkvm.o += -iquote $(obj)
$(obj)/$(kvm)/vmx/vmenter.pkvm.o: $(obj)/kvm-asm-offsets.h

fpu				:= ../../kernel/fpu
pkvm-hyp-y			+= $(fpu)/core.o $(fpu)/xstate.o

include $(srctree)/drivers/iommu/intel/pkvm/Makefile.pkvm

pkvm-obj 			:= $(patsubst %.o,%.pkvm.o,$(pkvm-hyp-y))
obj-$(CONFIG_PKVM_X86)		+= pkvm.o

CFLAGS_fpu.pkvm.o		+= -iquote $(srctree)/arch/x86/kernel/fpu

AFLAGS_vmx/host_vmentry.pkvm.o	+= -iquote $(srctree)/arch/x86/kvm/vmx
AFLAGS_vmx/host_vmentry.pkvm.o	+= -iquote $(obj)/
$(obj)/vmx/host_vmentry.pkvm.o:	$(obj)/kvm-asm-offsets.h

$(obj)/kvm-asm-offsets.s: $(obj)/$(kvm)/kvm-asm-offsets.c FORCE
	$(call if_changed_dep,cc_s_c)
$(obj)/kvm-asm-offsets.h: $(obj)/kvm-asm-offsets.s FORCE
	$(call filechk,offsets,__KVM_ASM_OFFSETS_H__)

targets += $(pkvm-obj) pkvm.tmp.o pkvm.lds retpoline.pkvm.S kvm-asm-offsets.s
clean-files += retpoline.pkvm.S kvm-asm-offsets.h

# When compiling with the $(objtool-enabled) is true, a single build-in object
# will be processed by the objtool after all of these objects are linked as a
# vmlinux.o if the objtool processing is delayed (represented by the
# $(delay-objtool) is true), otherwise the processing will be done when
# compiling it.
#
# For the later case, the $(objtool-enabled) is private to the rule of
# $(obj)/%.o. As the pKVM's object file is compiled via using the rule of
# $(obj)/%.pkvm.o, the $(objtool-enabled) is empty which represents a false for
# the pKVM. So if the $(delay-objtool) is false, meaning the objtool processing
# should be done when compiling a single build-in object, but the pKVM's object
# will not be processed by the objtool as $(objtool-enabled) is false. Setting
# pKVM's $(objtool-enabled) variable according to the $(delay-objtool) so that
# the pKVM's objects will be processed by the objtool by either way.
ifdef CONFIG_OBJTOOL
$(obj)/%.pkvm.o: private objtool-enabled = $(if $(delay-objtool),,y)
endif

$(obj)/%.pkvm.o: $(src)/%.c FORCE
	$(call if_changed_rule,cc_o_c)
$(obj)/%.pkvm.o: $(src)/%.S FORCE
	$(call if_changed_rule,as_o_S)

$(obj)/pkvm.lds: $(src)/pkvm.lds.S FORCE
	$(call if_changed_dep,cpp_lds_S)

LDFLAGS_pkvm.tmp.o := -r -T
$(obj)/pkvm.tmp.o: $(obj)/pkvm.lds $(addprefix $(obj)/,$(pkvm-obj)) FORCE
	$(call if_changed,ld)

$(obj)/pkvm.o: $(obj)/pkvm.tmp.o $(cmd_deps) FORCE
	$(call if_changed,pkvmcopy)

quiet_cmd_pkvmcopy = PKVMPCOPY $@
      cmd_pkvmcopy = $(BASH) $(src)/compiling_cmds.sh rename_syms $< $@

# The pKVM hypervisor should be self-contained. All the symbols it uses should
# be implemented in its own text. Remove ftrace to prevent compiler to insert
# the __fentry__ symbols which are not part of the pKVM hypervisor text to
# simplify the implementation.
#
# NOTE: do not remove CC_FLAGS_CFI. Since CFI code chunks don't depend
# on the host kernel's global data, we can keep CFI coverage enabled
# for the hypervisor as well, and we _should_ keep it enabled, for the
# following reasons (apart from its security advantages):
# 1. To avoid problems with the boot time kernel patching which assumes
#    that if CFI is enabled, it covers the entire kernel text (see
#    apply_seal_endbr()).
# 2. To keep the calling convention compatible in cases when hypervisor
#    functions are called from host kernel code (before deprivilege).
KBUILD_CFLAGS := $(filter-out $(CC_FLAGS_FTRACE), $(KBUILD_CFLAGS))
