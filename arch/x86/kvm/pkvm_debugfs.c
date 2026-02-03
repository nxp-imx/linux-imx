// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) "pkvm-host: " fmt

#include <linux/debugfs.h>
#include <linux/kvm_host.h>
#include <asm/kvm_pkvm.h>
#include <asm/pkvm_trace.h>

static void enable_vmexit_trace_func(void *data)
{
	u64 val;

	if (!data)
		return;

	val = *(u64 *)data;
	pkvm_hypercall(enable_vmexit_trace, val);
}

static int enable_vmexit_trace(void *data, u64 val)
{
	int cpu;

	for_each_possible_cpu(cpu)
		smp_call_function_single(cpu, enable_vmexit_trace_func, &val, true);

	return 0;
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
	int *vcpu_id;
	int i;

	if (!print)
		return -EINVAL;

	vcpu_id = (print == pervcpu) ? &pervcpu->vcpu_id : NULL;
	if (vcpu_id)
		prefix = kasprintf(GFP_KERNEL, "Host-vcpu%d: ", *vcpu_id);
	else
		prefix = kasprintf(GFP_KERNEL, "Host: ");

	if (!prefix)
		return -ENOMEM;

#define for_each_stats(event, num)							\
	for (summary_stats = (print == pervcpu && summary) ? summary->event : NULL,	\
	     stats = print->event, i = 0;						\
	     i < num;									\
	     summary_stats = summary_stats ? summary_stats + 1 : NULL, stats++, i++)

	for_each_stats(vmexit_reasons, MAX_EXIT_REASONS)
		__print_perf_data(m, prefix, get_vmexit_reason(i), stats, summary_stats);

	for_each_stats(hypercalls, MAX_PKVM_HYPERCALLS)
		__print_perf_data(m, prefix, pkvm_hypercalls[i], stats, summary_stats);

#undef for_each_stats

	kfree(prefix);

	return 0;
}

static int pkvm_dump_vmexit_trace(struct seq_file *m, struct perf_data *dump,
				  unsigned long size)
{
	struct perf_data *summary = kmalloc(sizeof(struct perf_data), GFP_KERNEL_ACCOUNT);
	struct perf_data *pervcpu;
	int cpu, ret;

	if (!summary) {
		pr_err("failed to allocate perf summary buffer\n");
		return -ENOMEM;
	}

	for (cpu = 0, pervcpu = dump;
	     cpu < num_possible_cpus() && size >= sizeof(struct perf_data);
	     cpu++, size -= sizeof(struct perf_data), pervcpu++) {
		ret = print_perf_data(m, summary, pervcpu);
		if (ret)
			goto out;

		cond_resched();
	}

	ret = print_perf_data(m, summary, NULL);
out:
	kfree(summary);
	return ret;
}

static int vmexit_trace_show(struct seq_file *m, void *unused)
{
	struct perf_data *perf;
	unsigned long size;
	int ret;

	size = sizeof(struct perf_data) * num_possible_cpus();
	perf = alloc_pages_exact(size, GFP_KERNEL_ACCOUNT);
	if (!perf) {
		pr_err("failed to allocate perf buffer\n");
		return -ENOMEM;
	}

	ret = pkvm_hypercall(dump_vmexit_trace, __pa(perf), size);
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
