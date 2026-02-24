// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) "pkvm-host: " fmt

#include <linux/debugfs.h>
#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include <asm/pkvm_trace.h>

static int enable_vmexit_trace(void *data, u64 val)
{
	return pkvm_hypercall(enable_vmexit_trace, val);
}
DEFINE_SIMPLE_ATTRIBUTE(enable_vmexit_trace_fops, NULL, enable_vmexit_trace, "%llu\n");

static const struct vmexit_reason {
	int reason;
	char *name;
} reasons[] = { VMX_EXIT_REASONS, { -1, NULL } };

static const char *get_vmexit_reason(int reason)
{
	const struct vmexit_reason *p = reasons;

	while (p->name) {
		if (p->reason == reason)
			return p->name;
		p++;
	}

	return NULL;
}

static const char *pkvm_hypercalls[MAX_PKVM_HYPERCALLS] = {
	#define PKVM_HC(fn)	[TO_PKVM_HC(fn)] = #fn,
	#include <asm/pkvm_hypercalls.h>
};

static inline void __print_perf_data(struct seq_file *m, const char *prefix,
				     const char *reason, struct vmexit_stats *print,
				     struct vmexit_stats *summary)
{
	if (!print->count)
		return;

	seq_printf(m, "%s%s %lld cycles %lld each-handler-cycle %lld\n",
		   prefix, reason, print->count, print->cycles,
		   print->cycles / print->count);

	if (summary) {
		summary->count += print->count;
		summary->cycles += print->cycles;
	}
}

static int print_perf_data(struct seq_file *m, struct perf_data *summary,
			   struct perf_data *pervcpu)
{
	struct perf_data *print = pervcpu ? pervcpu : summary;
	struct vmexit_stats *stats, *summary_stats;
	const char *prefix;
	int vm_handle;
	int i;

	if (!print)
		return -EINVAL;

	vm_handle = print->vm_handle;
	if (print == pervcpu) {
		if (vm_handle == PKVM_HOST_VM_HANDLE)
			prefix = kasprintf(GFP_KERNEL, "Host-vcpu%d: ", print->vcpu_id);
		else
			prefix = kasprintf(GFP_KERNEL, "VM%d-vcpu%d: ",
					   vm_handle, print->vcpu_id);
	} else {
		if (vm_handle == PKVM_HOST_VM_HANDLE)
			prefix = kasprintf(GFP_KERNEL, "Host: ");
		else
			prefix = kasprintf(GFP_KERNEL, "VM%d: ", vm_handle);
	}

	if (!prefix)
		return -ENOMEM;

#define for_each_stats(event, num)							\
	for (summary_stats = (print == pervcpu && summary) ? summary->event : NULL,	\
	     stats = print->event, i = 0;						\
	     i < num;									\
	     summary_stats = summary_stats ? summary_stats + 1 : NULL, stats++, i++)

	for_each_stats(vmexit_reasons, MAX_EXIT_REASONS)
		__print_perf_data(m, prefix, get_vmexit_reason(i), stats, summary_stats);

	if (vm_handle == PKVM_HOST_VM_HANDLE)
		for_each_stats(hypercalls, MAX_PKVM_HYPERCALLS)
			__print_perf_data(m, prefix, pkvm_hypercalls[i], stats, summary_stats);

#undef for_each_stats

	kfree(prefix);

	return 0;
}

static int dump_host_vmexit_trace(struct seq_file *m, struct perf_data *summary,
				  struct perf_data **dump, unsigned long *size)
{
	struct perf_data *pervcpu;
	int cpu, ret;

	for (cpu = 0, pervcpu = *dump;
	     cpu < num_possible_cpus() && *size >= sizeof(struct perf_data);
	     cpu++, *size -= sizeof(struct perf_data), pervcpu++, *dump = pervcpu) {
		/*
		 * The trace data from the pKVM hypervisor is grouped by VM.
		 * If the vm_handle in this perf data is not for the host, it
		 * means all the perf data for the host are already printed.
		 */
		if (pervcpu->vm_handle != PKVM_HOST_VM_HANDLE)
			break;

		ret = print_perf_data(m, summary, pervcpu);
		if (ret)
			return ret;

		cond_resched();
	}

	return print_perf_data(m, summary, NULL);
}

static int dump_guest_vmexit_trace(struct seq_file *m, struct perf_data *summary,
				   struct perf_data **dump, unsigned long *size)
{
	struct perf_data *pervcpu;
	int ret;

	for (pervcpu = *dump;
	     *size >= sizeof(struct perf_data);
	     *size -= sizeof(struct perf_data), pervcpu++, *dump = pervcpu) {
		/*
		 * The trace data from the pKVM hypervisor is grouped by VM.
		 * If the vm_handle in this perf data is not for this guest VM,
		 * it means all the perf data for this guest VM are already
		 * printed.
		 */
		if (pervcpu->vm_handle != summary->vm_handle)
			break;

		ret = print_perf_data(m, summary, pervcpu);
		if (ret)
			return ret;

		cond_resched();
	}

	return print_perf_data(m, summary, NULL);
}

static int pkvm_dump_vmexit_trace(struct seq_file *m, struct perf_data *dump,
				  unsigned long size)
{
	struct perf_data *summary = kmalloc(sizeof(struct perf_data), GFP_KERNEL_ACCOUNT);
	int ret;

	if (!summary) {
		pr_err("failed to allocate perf summary buffer\n");
		return -ENOMEM;
	}

	while (size >= sizeof(struct perf_data)) {
		memset(summary, 0, sizeof(struct perf_data));
		summary->vm_handle = dump->vm_handle;

		if (dump->vm_handle == PKVM_HOST_VM_HANDLE)
			ret = dump_host_vmexit_trace(m, summary, &dump, &size);
		else
			ret = dump_guest_vmexit_trace(m, summary, &dump, &size);

		if (ret) {
			pr_err("failed to dump vmexit trace for VM handle 0x%x\n",
			       dump->vm_handle);
			break;
		}
	}

	kfree(summary);
	return ret;
}

static int vmexit_trace_show(struct seq_file *m, void *unused)
{
	struct kvm *kvm = (struct kvm *)m->private;
	struct perf_data *perf;
	unsigned long size;
	int ret, vm_handle;

	if (kvm) {
		/* Dump vmexit trace for a specific VM */
		size = atomic_read(&kvm->online_vcpus) * sizeof(struct perf_data);
		vm_handle = kvm->arch.pkvm.handle;
	} else {
		/* Dump vmexit trace for all VMs including the host VM */
		size = sizeof(struct perf_data) * num_possible_cpus();
		mutex_lock(&kvm_lock);
		list_for_each_entry(kvm, &vm_list, vm_list)
			size += atomic_read(&kvm->online_vcpus) * sizeof(struct perf_data);
		mutex_unlock(&kvm_lock);
		vm_handle = PKVM_HOST_VM_HANDLE;
	}

	perf = alloc_pages_exact(size, GFP_KERNEL_ACCOUNT);
	if (!perf) {
		pr_err("failed to allocate perf buffer\n");
		return -ENOMEM;
	}

	ret = pkvm_hypercall(dump_vmexit_trace, __pa(perf), size, vm_handle);
	if (ret) {
		pr_err("failed to get vmexit trace: err %d\n", ret);
		goto out;
	}

	ret = pkvm_dump_vmexit_trace(m, perf, size);
	if (ret)
		pr_err("failed to dump vmexit trace: err %d\n", ret);
out:
	free_pages_exact(perf, size);

	return ret;
}
DEFINE_SHOW_ATTRIBUTE(vmexit_trace);

struct debugfs_item {
	const char *name;
	const umode_t mode;
	const struct file_operations *fops;
	struct dentry *dentry;
};

struct debugfs_item debugfs_files[] = {
	{ "enable_vmexit_trace", 0222, &enable_vmexit_trace_fops},
	{ "vmexit_trace", 0444, &vmexit_trace_fops},
	{ NULL }
};

void pkvm_init_debugfs(void)
{
	struct dentry *debugfs_dir = debugfs_create_dir("pkvm", NULL);
	struct debugfs_item *p;

	if (IS_ERR_OR_NULL(debugfs_dir)) {
		pr_err("failed to add debugfs root\n");
		return;
	}

	for (p = debugfs_files; p->name; ++p) {
		p->dentry = debugfs_create_file(p->name, p->mode,
						debugfs_dir,
						NULL, p->fops);
		if (IS_ERR_OR_NULL(p->dentry)) {
			pr_err("failed to add debugfs %s\n", p->name);
			goto out_dir;
		}
	}

	return;

out_dir:
	for (p = debugfs_files; p->dentry; ++p) {
		debugfs_remove(p->dentry);
		p->dentry = NULL;
	}
	debugfs_remove(debugfs_dir);
}

void pkvm_create_vm_debugfs(struct kvm *kvm)
{
	if (!enable_pkvm)
		return;

	debugfs_create_file("pkvm_vmexit_trace", 0444, kvm->debugfs_dentry,
			    kvm, &vmexit_trace_fops);
}
