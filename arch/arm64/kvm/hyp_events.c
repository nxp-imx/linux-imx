// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Google LLC
 */

#include <linux/tracefs.h>
#include <linux/rcupdate.h>

#include <asm/kvm_host.h>
#include <asm/setup.h>

#include "hyp_trace.h"

static const char *hyp_printk_fmt_from_id(u8 fmt_id);

#include <asm/kvm_define_hypevents.h>

struct hyp_table {
	void		*start;
	unsigned long	nr_entries;
};

struct hyp_mod_tables {
	struct hyp_table	*tables;
	unsigned long		nr_tables;
};

#define nr_entries(__start, __stop) \
	(((unsigned long)__stop - (unsigned long)__start) / sizeof(*__start))

static int hyp_table_add(struct hyp_mod_tables *mod_tables, void *start,
			 unsigned long nr_entries)
{
	struct hyp_table *new, *old;
	int i;

	new = kmalloc_array(mod_tables->nr_tables + 1, sizeof(*new), GFP_KERNEL);
	if (!new)
		return -ENOMEM;

	for (i = 0; i < mod_tables->nr_tables; i++) {
		new[i].start = mod_tables->tables[i].start;
		new[i].nr_entries = mod_tables->tables[i].nr_entries;
	}
	new[i].start = start;
	new[i].nr_entries = nr_entries;

	old = rcu_replace_pointer(mod_tables->tables, new, true);
	synchronize_rcu();
	mod_tables->nr_tables++;
	kfree(old);

	return 0;
}

static void *hyp_table_entry(struct hyp_mod_tables *mod_tables,
			     size_t entry_size, unsigned long id)
{
	struct hyp_table *table;
	void *entry = NULL;

	rcu_read_lock();
	table = rcu_dereference(mod_tables->tables);

	for (int i = 0; i < mod_tables->nr_tables; i++) {
		if (table->nr_entries <= id) {
			id -= table->nr_entries;
			table++;
			continue;
		}

		entry = (void *)((char *)table->start + (id * entry_size));
		break;
	}
	rcu_read_unlock();

	return entry;
}

extern struct hyp_printk_fmt __hyp_printk_fmts_start[];
extern struct hyp_printk_fmt __hyp_printk_fmts_end[];

static struct hyp_mod_tables mod_printk_fmt_tables;
static unsigned long total_printk_fmts;

static const char *hyp_printk_fmt_from_id(u8 fmt_id)
{
	u8 nr_fmts = nr_entries(__hyp_printk_fmts_start, __hyp_printk_fmts_end);
	struct hyp_printk_fmt *fmt = NULL;

	if (fmt_id < nr_fmts)
		return (__hyp_printk_fmts_start + fmt_id)->fmt;

	fmt_id -= nr_fmts;

	fmt = hyp_table_entry(&mod_printk_fmt_tables, sizeof(*fmt), fmt_id);

	return fmt ? fmt->fmt : "Unknown Format";
}

extern struct hyp_event __hyp_events_start[];
extern struct hyp_event __hyp_events_end[];

/* hyp_event section used by the hypervisor */
extern struct hyp_event_id __hyp_event_ids_start[];
extern struct hyp_event_id __hyp_event_ids_end[];

static int enable_hyp_event(struct hyp_event *event, bool enable)
{
	unsigned short id = event->id;
	int ret;

	if (enable == *event->enabled)
		return 0;

	ret = kvm_call_hyp_nvhe(__pkvm_enable_event, id, enable);
	if (ret)
		return ret;

	*event->enabled = enable;

	return 0;
}

static ssize_t
hyp_event_write(struct file *filp, const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct seq_file *seq_file = (struct seq_file *)filp->private_data;
	struct hyp_event *evt = (struct hyp_event *)seq_file->private;
	bool enabling;
	int ret;
	char c;

	if (!cnt || cnt > 2)
		return -EINVAL;

	if (get_user(c, ubuf))
		return -EFAULT;

	switch (c) {
	case '1':
		enabling = true;
		break;
	case '0':
		enabling = false;
		break;
	default:
		return -EINVAL;
	}

	ret = enable_hyp_event(evt, enabling);
	if (ret)
		return ret;

	return cnt;
}

static int hyp_event_show(struct seq_file *m, void *v)
{
	struct hyp_event *evt = (struct hyp_event *)m->private;

	seq_printf(m, "%d\n", *evt->enabled);

	return 0;
}

static int hyp_event_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hyp_event_show, inode->i_private);
}

static const struct file_operations hyp_event_fops = {
	.open		= hyp_event_open,
	.write		= hyp_event_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int hyp_event_id_show(struct seq_file *m, void *v)
{
	struct hyp_event *evt = (struct hyp_event *)m->private;

	seq_printf(m, "%d\n", evt->id);

	return 0;
}

static int hyp_event_id_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hyp_event_id_show, inode->i_private);
}

static const struct file_operations hyp_event_id_fops = {
	.open = hyp_event_id_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int hyp_event_format_show(struct seq_file *m, void *v)
{
	struct hyp_event *evt = (struct hyp_event *)m->private;
	struct trace_event_fields *field;
	unsigned int offset = sizeof(struct hyp_entry_hdr);

	seq_printf(m, "name: %s\n", evt->name);
	seq_printf(m, "ID: %d\n", evt->id);
	seq_puts(m, "format:\n\tfield:unsigned short common_type;\toffset:0;\tsize:2;\tsigned:0;\n");
	seq_puts(m, "\n");

	field = &evt->fields[0];
	while (field->name) {
		seq_printf(m, "\tfield:%s %s;\toffset:%u;\tsize:%u;\tsigned:%d;\n",
			  field->type, field->name, offset, field->size,
			  !!field->is_signed);
		offset += field->size;
		field++;
	}

	if (field != &evt->fields[0])
		seq_puts(m, "\n");

	seq_printf(m, "print fmt: %s\n", evt->print_fmt);

	return 0;
}

static int hyp_event_format_open(struct inode *inode, struct file *file)
{
	return single_open(file, hyp_event_format_show, inode->i_private);
}

static const struct file_operations hyp_event_format_fops = {
	.open = hyp_event_format_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t hyp_header_page_read(struct file *filp, char __user *ubuf,
				   size_t cnt, loff_t *ppos)
{
	struct buffer_data_page field;
	struct trace_seq *s;
	ssize_t r;

	s = kmalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	trace_seq_init(s);
	trace_seq_printf(s, "\tfield: u64 timestamp;\t"
			 "offset:0;\tsize:%u;\tsigned:%u;\n",
			 (unsigned int)sizeof(field.time_stamp),
			 (unsigned int)is_signed_type(u64));

	trace_seq_printf(s, "\tfield: local_t commit;\t"
			 "offset:%u;\tsize:%u;\tsigned:%u;\n",
			 (unsigned int)offsetof(typeof(field), commit),
			 (unsigned int)sizeof(field.commit),
			 (unsigned int)is_signed_type(long));

	trace_seq_printf(s, "\tfield: int overwrite;\t"
			 "offset:%u;\tsize:%u;\tsigned:%u;\n",
			 (unsigned int)offsetof(typeof(field), commit),
			 1,
			 (unsigned int)is_signed_type(long));

	trace_seq_printf(s, "\tfield: char data;\t"
			 "offset:%u;\tsize:%u;\tsigned:%u;\n",
			 (unsigned int)offsetof(typeof(field), data),
			 (unsigned int)BUF_PAGE_SIZE,
			 (unsigned int)is_signed_type(char));

	r = simple_read_from_buffer(ubuf, cnt, ppos, s->buffer,
				    trace_seq_used(s));
	kfree(s);

	return r;
}

static const struct file_operations hyp_header_page_fops = {
	.read = hyp_header_page_read,
	.llseek = default_llseek,
};

static struct dentry *event_tracefs;
static unsigned int last_event_id;

static struct hyp_mod_tables mod_event_tables;

struct hyp_event *__hyp_trace_find_event_name(const char *name,
					      struct hyp_event *start,
					      struct hyp_event *end)
{
	for (; start < end; start++) {
		if (!strncmp(name, start->name, HYP_EVENT_NAME_MAX))
			return start;
	}

	return NULL;
}

struct hyp_event *hyp_trace_find_event_name(const char *name)
{
	struct hyp_table *table;
	struct hyp_event *event =
		__hyp_trace_find_event_name(name, __hyp_events_start,
					    __hyp_events_end);

	if (event)
		return event;

	rcu_read_lock();
	table = rcu_dereference(mod_event_tables.tables);

	for (int i = 0; i < mod_event_tables.nr_tables; i++) {
		struct hyp_event *end = (struct hyp_event *)table->start +
							    table->nr_entries;

		event = __hyp_trace_find_event_name(name, table->start, end);
		if (event)
			break;
	}

	rcu_read_unlock();

	return event;
}

struct hyp_event *hyp_trace_find_event(int id)
{
	struct hyp_event *event = __hyp_events_start + id;

	if ((unsigned long)event >= (unsigned long)__hyp_events_end) {

		id -= nr_entries(__hyp_events_start, __hyp_events_end);

		event = hyp_table_entry(&mod_event_tables, sizeof(*event), id);
	}

	return event;
}

static char early_events[COMMAND_LINE_SIZE];

static __init int setup_hyp_event_early(char *str)
{
	strscpy(early_events, str, COMMAND_LINE_SIZE);

	return 1;
}
__setup("hyp_event=", setup_hyp_event_early);

bool hyp_event_early_probe(void)
{
	char *token, *buf = early_events;
	bool enabled = false;

	while (true) {
		token = strsep(&buf, ",");

		if (!token)
			break;

		if (*token) {
			struct hyp_event *event;
			int ret;

			event = hyp_trace_find_event_name(token);
			if (event) {
				ret = enable_hyp_event(event, true);
				if (ret)
					pr_warn("Couldn't enable hyp event %s:%d\n",
						token, ret);
				else
					enabled = true;
			}
		}

		if (buf)
			*(buf - 1) = ',';
	}

	return enabled;
}

static void hyp_event_table_init_tracefs(struct hyp_event *event, int nr_events)
{
	struct dentry *event_dir;
	int i;

	if (!event_tracefs)
		return;

	for (i = 0; i < nr_events; event++, i++) {
		event_dir = tracefs_create_dir(event->name, event_tracefs);
		if (!event_dir) {
			pr_err("Failed to create events/hypervisor/%s\n", event->name);
			continue;
		}

		tracefs_create_file("enable", 0700, event_dir, (void *)event,
				    &hyp_event_fops);
		tracefs_create_file("id", 0400, event_dir, (void *)event,
				    &hyp_event_id_fops);
		tracefs_create_file("format", 0400, event_dir, (void *)event,
				    &hyp_event_format_fops);
	}
}

/*
 * Register hyp events and write their id into the hyp section _hyp_event_ids.
 */
static int hyp_event_table_init(struct hyp_event *event,
				struct hyp_event_id *event_id, int nr_events)
{
	while (nr_events--) {
		/*
		 * Both the host and the hypervisor rely on the same hyp event
		 * declarations from kvm_hypevents.h. We have then a 1:1
		 * mapping.
		 */
		event->id = event_id->id = last_event_id++;

		event++;
		event_id++;
	}

	return 0;
}

void hyp_trace_init_event_tracefs(struct dentry *parent)
{
	int nr_events = nr_entries(__hyp_events_start, __hyp_events_end);

	parent = tracefs_create_dir("events", parent);
	if (!parent) {
		pr_err("Failed to create tracefs folder for hyp events\n");
		return;
	}

	tracefs_create_file("header_page", 0400, parent, NULL,
			    &hyp_header_page_fops);

	event_tracefs = tracefs_create_dir("hypervisor", parent);
	if (!event_tracefs) {
		pr_err("Failed to create tracefs folder for hyp events\n");
		return;
	}

	hyp_event_table_init_tracefs(__hyp_events_start, nr_events);
}

int hyp_trace_init_events(void)
{
	int nr_events = nr_entries(__hyp_events_start, __hyp_events_end);
	int nr_event_ids = nr_entries(__hyp_event_ids_start, __hyp_event_ids_end);
	int nr_printk_fmts = nr_entries(__hyp_printk_fmts_start, __hyp_printk_fmts_end);

	/* __hyp_printk event only supports U8_MAX different formats */
	WARN_ON(nr_printk_fmts > U8_MAX);

	total_printk_fmts = nr_printk_fmts;

	if (WARN(nr_events != nr_event_ids, "Too many trace_hyp_printk()!"))
		return -EINVAL;

	return hyp_event_table_init(__hyp_events_start, __hyp_event_ids_start,
				    nr_events);
}

int hyp_trace_init_mod_events(struct hyp_event *event,
			      struct hyp_event_id *event_id, int nr_events,
			      struct hyp_printk_fmt *fmt, int nr_fmts)
{
	u8 *hyp_printk_fmt_offsets;
	int ret;

	ret = hyp_event_table_init(event, event_id, nr_events);
	if (ret)
		return ret;

	ret = hyp_table_add(&mod_event_tables, (void *)event, nr_events);
	if (ret)
		return ret;

	hyp_event_table_init_tracefs(event, nr_events);

	if (total_printk_fmts + nr_fmts > U8_MAX) {
		pr_warn("Too many trace_hyp_printk()!");
		return 0;
	}

	if (WARN_ON(nr_fmts && !event_id))
		return 0;

	ret = hyp_table_add(&mod_printk_fmt_tables, (void *)fmt, nr_fmts);
	if (ret) {
		pr_warn("Not enough memory to register trace_hyp_printk()");
		return 0;
	}

	/* format offsets stored after event_ids (see module.lds.S) */
	hyp_printk_fmt_offsets = (u8 *)(event_id + nr_events);
	memset(hyp_printk_fmt_offsets, total_printk_fmts, nr_fmts);

	total_printk_fmts += nr_fmts;

	return 0;
}
