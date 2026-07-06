// SPDX-License-Identifier: GPL-2.0
#include <linux/align.h>
#include <linux/array_size.h>
#include <asm/kvm_pkvm.h>
#include <asm/page.h>
#include <asm/percpu.h>
#include <asm/processor.h>
#include <asm/sections.h>
#include "memory.h"
#include "pkvm.h"

unsigned long __per_cpu_offset[NR_CPUS];
DEFINE_PER_CPU_CACHE_HOT(unsigned long, this_cpu_off);
DEFINE_PER_CPU_CACHE_HOT(int, cpu_number);
#ifdef CONFIG_X86_64
DEFINE_PER_CPU_CACHE_HOT(u64, __x86_call_depth);
#endif
struct cpuinfo_x86 boot_cpu_data;
struct cpumask __cpu_possible_mask __ro_after_init;
unsigned int nr_cpu_ids;
DEFINE_PER_CPU(u64, x86_spec_ctrl_current);
DEFINE_STATIC_KEY_FALSE(switch_vcpu_ibpb);
u64 x86_pred_cmd = PRED_CMD_IBPB;
unsigned int tsc_khz;
bool msi_dest_mode_logical;

/*
 * Used to switch the FPU state between the host VM and pVMs. The fpu struct is
 * allocated right after the task_struct, in order to reuse x86_task_fpu(). See
 * also comment in fpu_clone().
 */
struct pkvm_task_struct {
	struct task_struct task;
	struct fpu fpu;
};
static DEFINE_PER_CPU(struct pkvm_task_struct, pkvm_task);
DEFINE_PER_CPU_CACHE_HOT(struct task_struct *, current_task);

unsigned int pkvm_per_cpu_nr_pages(void)
{
#ifndef CONFIG_PKVM_X86_DEBUG
	unsigned long per_cpu_size = (unsigned long)__per_cpu_end -
				     (unsigned long)__per_cpu_start;

	return ALIGN(per_cpu_size, PAGE_SIZE) >> PAGE_SHIFT;
#else
	return 0;
#endif
}

int pkvm_setup_per_cpu(int cpu, unsigned long base,
		       unsigned long pcpu_pa, unsigned long vcpu_pa)
{
	struct pkvm_pcpu *pcpu = __pkvm_va(pcpu_pa);
	struct kvm_vcpu *vcpu = __pkvm_va(vcpu_pa);
	struct task_struct *task;

	if (cpu >= ARRAY_SIZE(__per_cpu_offset))
		return -EINVAL;
	if (pcpu->cpu != cpu)
		return -EINVAL;
	if (vcpu->cpu != cpu)
		return -EINVAL;

#ifndef CONFIG_PKVM_X86_DEBUG
	__per_cpu_offset[cpu] = (unsigned long)__pkvm_va(base) -
				(unsigned long)__per_cpu_start;
#else
	__per_cpu_offset[cpu] = (unsigned long)__pkvm_va(base);
#endif
	per_cpu(this_cpu_off, cpu) = __per_cpu_offset[cpu];
	per_cpu(cpu_number, cpu) = cpu;
	per_cpu(phys_cpu, cpu) = pcpu;
	per_cpu(host_vcpu, cpu) = vcpu;
	per_cpu(host_vcpu_fixup, cpu) = true;

	task = &per_cpu_ptr(&pkvm_task, cpu)->task;
	task->group_leader = task;
	per_cpu(current_task, cpu) = task;

	return 0;
}

unsigned long pkvm_per_cpu_offset(int cpu)
{
	if (cpu < 0 || cpu >= ARRAY_SIZE(__per_cpu_offset))
		return 0;

	return __per_cpu_offset[cpu];
}

void set_x86_spec_ctrl(u64 spec_ctrl)
{
	int cpu;

	for_each_possible_cpu(cpu)
		per_cpu(x86_spec_ctrl_current, cpu) |= spec_ctrl;
}
