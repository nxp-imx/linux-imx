// SPDX-License-Identifier: (GPL-2.0)
/*
 * Copyright 2025 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/imx_memory_usage.h>
#include <linux/imx_vpu.h>

#define IMX_MUR_DEBUGFS_NAME		"mur"

struct imx_mu_recorder {
	const char *name;
	struct imx_mur_node *root;
	struct dentry *debugfs;

	struct kmem_cache *cache;

	spinlock_t lock; //lock for the mur list
};

struct imx_mur_node {
	struct list_head list;
	const char *name;
	struct imx_mur_node *parent;
	struct list_head children;
	atomic_long_t memory_usage_bytes;
	atomic_long_t count;

	pid_t pid;
	pid_t tgid;

	struct imx_mu_recorder *recorder;
	struct v4l2_ctrl *ctrl;
};

static long imx_mur_long_read_val(struct imx_mur_node *node)
{
	long memory_usage_bytes = atomic_long_read(&node->memory_usage_bytes);
	struct imx_mur_node *child;

	list_for_each_entry(child, &node->children, list)
		memory_usage_bytes += imx_mur_long_read_val(child);

	return memory_usage_bytes;
}

static void imx_mur_update_ctrl(struct imx_mur_node *node)
{
	while (node) {
		if (node->ctrl)
			v4l2_ctrl_s_ctrl_int64(node->ctrl, imx_mur_long_read(node));

		node = node->parent;
	}
}

static void imx_mur_init_node(struct imx_mur_node *node, struct imx_mu_recorder *recorder)
{
	INIT_LIST_HEAD(&node->children);
	INIT_LIST_HEAD(&node->list);
	atomic_long_set(&node->memory_usage_bytes, 0);
	atomic_long_set(&node->count, 0);
	node->recorder = recorder;
	node->tgid = current->tgid;
	node->pid = current->pid;
}

static void imx_mur_show_node(struct seq_file *s, struct imx_mur_node *node, int depth)
{
	struct imx_mur_node *child;
	char str[128];
	int num;
	int i;

	for (i = 0; i < depth; i++) {
		if (seq_write(s, "\t", 1))
			return;
	}

	if (node->name) {
		num = scnprintf(str, sizeof(str), "%s ", node->name);
		if (seq_write(s, str, num))
			return;
	}
	if (node->tgid && node->pid) {
		num = scnprintf(str, sizeof(str), "(tgid = %d, pid = %d) ", node->tgid, node->pid);
		if (seq_write(s, str, num))
			return;
	}

	num = scnprintf(str, sizeof(str),
			"usage: %ld", imx_mur_long_read_val(node));
	if (seq_write(s, str, num))
		return;

	if (atomic_long_read(&node->count) > 1 ||
	    (atomic_long_read(&node->count) == 1 && !list_empty(&node->children))) {
		num = scnprintf(str, sizeof(str), " (count %ld",
				atomic_long_read(&node->count));
		if (!list_empty(&node->children))
			num += scnprintf(str + num, sizeof(str) - num, " : %ld",
					atomic_long_read(&node->memory_usage_bytes));
		num += scnprintf(str + num, sizeof(str) - num, ")");
		if (seq_write(s, str, num))
			return;
	}

	num = scnprintf(str, sizeof(str), "\n");
	if (seq_write(s, str, num))
		return;

	list_for_each_entry(child, &node->children, list)
		imx_mur_show_node(s, child, depth + 1);
}

static void imx_mur_show_memory_usage(struct seq_file *s, struct imx_mu_recorder *recorder)
{
	char str[128];
	int num;

	num = scnprintf(str, sizeof(str),
			"total memory usage: %ld\n", imx_mur_long_read_val(recorder->root));
	if (seq_write(s, str, num))
		return;

	imx_mur_show_node(s, recorder->root, 0);
}

static int imx_mur_loger_show(struct seq_file *s, void *data)
{
	struct imx_mu_recorder *recorder = s->private;

	scoped_guard(spinlock, &recorder->lock)
		imx_mur_show_memory_usage(s, recorder);

	return 0;
}

static int imx_mur_loger_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, imx_mur_loger_show, inode->i_private);
}

static const struct file_operations imx_mur_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = imx_mur_loger_open,
	.release = single_release,
	.read = seq_read,
};

static void imx_mur_create_debugfs(struct imx_mu_recorder *recorder)
{
	struct dentry *mur_folder = debugfs_lookup(IMX_MUR_DEBUGFS_NAME, NULL);

	if (IS_ERR_OR_NULL(mur_folder))
		return;

	recorder->debugfs = debugfs_create_file(recorder->name,
						VERIFY_OCTAL_PERMISSIONS(0444),
						mur_folder,
						recorder,
						&imx_mur_debugfs_fops);
	dput(mur_folder);
}

static struct imx_mu_recorder *imx_mur_create_recoreder(const char *name)
{
	struct imx_mu_recorder *recorder;

	recorder = kzalloc(sizeof(*recorder), GFP_KERNEL);
	if (!recorder)
		return NULL;

	if (name)
		recorder->name = name;
	spin_lock_init(&recorder->lock);
	recorder->cache = kmem_cache_create("mur-node", sizeof(struct imx_mur_node),
					    0, SLAB_PANIC | SLAB_HWCACHE_ALIGN, NULL);
	if (!recorder->cache) {
		kfree(recorder);
		return NULL;
	}

	return recorder;
}

struct imx_mur_node *imx_mur_create_node(struct imx_mur_node *parent, const char *name)
{
	struct imx_mu_recorder *recorder = NULL;
	struct imx_mur_node *node;
	bool is_root;

	if (!parent) {
		recorder = imx_mur_create_recoreder(name);
		is_root = true;
	} else {
		recorder = parent->recorder;
		is_root = false;
	}

	if (!recorder)
		return NULL;

	node = kmem_cache_zalloc(recorder->cache, GFP_KERNEL);
	if (!node) {
		if (is_root)
			kfree(recorder);

		return NULL;
	}

	imx_mur_init_node(node, recorder);
	node->parent = parent;
	if (name)
		node->name = name;

	if (parent) {
		scoped_guard(spinlock, &recorder->lock)
			list_add_tail(&node->list, &parent->children);
	}

	if (is_root) {
		recorder->root = node;
		if (recorder->name)
			imx_mur_create_debugfs(recorder);
	}

	return node;
}
EXPORT_SYMBOL_GPL(imx_mur_create_node);

static void imx_mur_delete_node(struct imx_mur_node *node)
{
	struct imx_mur_node *child, *tmp;

	list_for_each_entry_safe(child, tmp, &node->children, list)
		imx_mur_delete_node(child);

	list_del(&node->list);
	kmem_cache_free(node->recorder->cache, node);
}

void imx_mur_destroy_node(struct imx_mur_node *node)
{
	struct imx_mu_recorder *recorder;
	struct imx_mur_node *parent;
	bool is_root;

	if (!node || !node->recorder)
		return;

	recorder = node->recorder;
	is_root = node->parent ? false : true;
	parent = node->parent;

	scoped_guard(spinlock, &recorder->lock)
		imx_mur_delete_node(node);

	if (is_root) {
		debugfs_remove(recorder->debugfs);
		kmem_cache_destroy(recorder->cache);
		kfree(recorder);
	} else {
		imx_mur_update_ctrl(parent);
	}
}
EXPORT_SYMBOL_GPL(imx_mur_destroy_node);

void imx_mur_long_add(struct imx_mur_node *node, long val)
{
	if (!node || !node->recorder)
		return;

	atomic_long_add(val, &node->memory_usage_bytes);
	atomic_long_inc(&node->count);

	imx_mur_update_ctrl(node);
}
EXPORT_SYMBOL_GPL(imx_mur_long_add);

void imx_mur_long_new_and_add(struct imx_mur_node *node, long val, const char *label)
{
	struct imx_mur_node *sub;

	if (!node || !node->recorder)
		return;

	sub = imx_mur_create_node(node, label);
	if (!sub)
		return;

	imx_mur_long_add(sub, val);
}
EXPORT_SYMBOL_GPL(imx_mur_long_new_and_add);

void imx_mur_long_sub_and_del(struct imx_mur_node *node, long val)
{
	struct imx_mu_recorder *recorder;
	bool found = false;

	if (!node || !node->recorder)
		return;

	recorder = node->recorder;

	scoped_guard(spinlock, &recorder->lock) {
		struct imx_mur_node *child;

		list_for_each_entry(child, &node->children, list) {
			if (!list_empty(&child->children))
				continue;
			if (atomic_long_read(&child->memory_usage_bytes) == val) {
				imx_mur_delete_node(child);
				found = true;
				break;
			}
		}
	}

	if (!found)
		imx_mur_long_sub(node, val);
	else
		imx_mur_update_ctrl(node);
}
EXPORT_SYMBOL_GPL(imx_mur_long_sub_and_del);

void imx_mur_long_sub(struct imx_mur_node *node, long val)
{
	if (!node || !node->recorder)
		return;

	atomic_long_sub(val, &node->memory_usage_bytes);
	atomic_long_dec(&node->count);
	imx_mur_update_ctrl(node);
}
EXPORT_SYMBOL_GPL(imx_mur_long_sub);

void imx_mur_long_set(struct imx_mur_node *node, long val)
{
	if (!node || !node->recorder)
		return;

	atomic_long_set(&node->memory_usage_bytes, val);
	atomic_long_set(&node->count, 1);
	imx_mur_update_ctrl(node);
}
EXPORT_SYMBOL_GPL(imx_mur_long_set);

long imx_mur_long_read(struct imx_mur_node *node)
{
	struct imx_mu_recorder *recorder;
	long memory_usage_bytes = 0;

	if (!node || !node->recorder)
		return 0;

	recorder = node->recorder;
	scoped_guard(spinlock, &recorder->lock)
		memory_usage_bytes = imx_mur_long_read_val(node);

	return memory_usage_bytes;
}
EXPORT_SYMBOL_GPL(imx_mur_long_read);

static const struct v4l2_ctrl_config memory_usage_ctrl = {
	.name = "MEMORY USAGE",
	.id = V4L2_CID_IMX_G_MEMORY_USAGE,
	.type = V4L2_CTRL_TYPE_INTEGER64,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
	.min = 0,
	.max = 0xFFFFFFFFFFFFLL,
	.step = 1,
};

struct v4l2_ctrl *imx_mur_new_v4l2_ctrl(struct v4l2_ctrl_handler *hdl, struct imx_mur_node *node)
{
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_custom(hdl, &memory_usage_ctrl, NULL);
	if (!ctrl)
		return NULL;

	if (node && node->recorder)
		node->ctrl = ctrl;

	return ctrl;
}
EXPORT_SYMBOL_GPL(imx_mur_new_v4l2_ctrl);

void imx_mur_release_v4l2_ctrl(struct imx_mur_node *node)
{
	if (node && node->recorder)
		node->ctrl = NULL;
}
EXPORT_SYMBOL_GPL(imx_mur_release_v4l2_ctrl);

static int __init imx_mur_init(void)
{
	debugfs_create_dir(IMX_MUR_DEBUGFS_NAME, NULL);
	return 0;
}

static void __exit imx_mur_exit(void)
{
	debugfs_lookup_and_remove(IMX_MUR_DEBUGFS_NAME, NULL);
}

subsys_initcall(imx_mur_init);
module_exit(imx_mur_exit);

MODULE_DESCRIPTION("Imx VPU memory usage record driver");
MODULE_LICENSE("GPL");
