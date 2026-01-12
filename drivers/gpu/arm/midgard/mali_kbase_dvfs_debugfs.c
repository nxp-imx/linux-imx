// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2020-2025 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include "mali_kbase_dvfs_debugfs.h"
#include <mali_kbase.h>
#include <linux/seq_file.h>

#if IS_ENABLED(CONFIG_DEBUG_FS)

/**
 * kbasep_dvfs_utilization_debugfs_show() - Print the DVFS utilization info
 *
 * @file: The seq_file for printing to
 * @data: The debugfs dentry private data, a pointer to kbase_context
 *
 * Return: Negative error code or 0 on success.
 */
static int kbasep_dvfs_utilization_debugfs_show(struct seq_file *file, void *data)
{
	struct kbase_device *kbdev = file->private;

	CSTD_UNUSED(data);
	seq_printf(file, "busy_time: %llu idle_time: %llu protm_time: %llu\n",
		   (unsigned long long)kbdev->pm.backend.metrics.values.time_busy,
		   (unsigned long long)kbdev->pm.backend.metrics.values.time_idle,
		   (unsigned long long)kbdev->pm.backend.metrics.values.time_in_protm);

	seq_printf(file, "shader_frag_busy_time: %llu shader_compute_busy_time: %llu tiler_busy_time: %llu\n",
		   kbdev->pm.backend.metrics.values.shader_frag_time_busy,
		   kbdev->pm.backend.metrics.values.shader_compute_time_busy,
		   kbdev->pm.backend.metrics.values.tiler_time_busy);

	seq_printf(file, "mcu_busy_time: %llu idvs_busy_time: %llu ceu_busy_time: %llu lsu_busy_time: %llu\n",
		   kbdev->pm.backend.metrics.values.mcu_time_busy,
		   kbdev->pm.backend.metrics.values.idvs_time_busy,
		   kbdev->pm.backend.metrics.values.ceu_time_busy,
		   kbdev->pm.backend.metrics.values.lsu_time_busy);

	seq_printf(file, "l2_ext_read_busy_time: %llu l2_ext_write_busy_time: %llu shader_starving_busy_time: %llu\n",
		   kbdev->pm.backend.metrics.values.l2_ext_read_time_busy,
		   kbdev->pm.backend.metrics.values.l2_ext_write_time_busy,
		   kbdev->pm.backend.metrics.values.shader_starving_time_busy);


	return 0;
}

static int kbasep_dvfs_utilization_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_dvfs_utilization_debugfs_show, in->i_private);
}

static const struct file_operations kbasep_dvfs_utilization_debugfs_fops = {
	.open = kbasep_dvfs_utilization_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * kbasep_gpu_profile_debugfs_show() - Print the gpu_profile enable state
 *
 * @file: The seq_file for printing to
 * @data: The debugfs dentry private data, a pointer to kbase_context
 *
 * Return: 0 on success.
 */
static int kbasep_gpu_profile_debugfs_show(struct seq_file *file, void *data)
{
	struct kbase_device *kbdev = file->private;

	CSTD_UNUSED(data);
	seq_printf(file, "%d\n", atomic_read(&kbdev->gpu_profile_enabled) ? 1 : 0);
	return 0;
}

static int kbasep_gpu_profile_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_gpu_profile_debugfs_show, in->i_private);
}

/**
 * kbasep_gpu_profile_debugfs_write() - Write 0 or 1 to gpu_profile
 *
 * Writing 1 enables SELECT_CSHW profiling counters.
 * Writing 0 disables SELECT_CSHW (allows GPU auto clock-gating).
 *
 * @file:  file pointer
 * @ubuf:  user buffer containing data to store
 * @count: number of bytes in user buffer
 * @ppos:  file position
 *
 * Return: @count if the function succeeded. An error code on failure.
 */
static ssize_t kbasep_gpu_profile_debugfs_write(struct file *file,
						const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *sfile = file->private_data;
	struct kbase_device *kbdev = sfile->private;
	unsigned long val;
	char buf[8];
	int ret;

	if (!kbdev)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';
	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	atomic_set(&kbdev->gpu_profile_enabled, (val == 1) ? 1 : 0);
	dev_dbg(kbdev->dev, "gpu_profile set to %lu\n", val);

	return count;
}

static const struct file_operations kbasep_gpu_profile_debugfs_fops = {
	.open    = kbasep_gpu_profile_debugfs_open,
	.read    = seq_read,
	.write   = kbasep_gpu_profile_debugfs_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

void kbase_dvfs_status_debugfs_init(struct kbase_device *kbdev)
{
	struct dentry *file;
	const mode_t mode = 0444;

	if (WARN_ON(!kbdev || IS_ERR_OR_NULL(kbdev->mali_debugfs_directory)))
		return;

	file = debugfs_create_file("dvfs_utilization", mode, kbdev->mali_debugfs_directory, kbdev,
				   &kbasep_dvfs_utilization_debugfs_fops);

	if (IS_ERR_OR_NULL(file)) {
		dev_warn(kbdev->dev, "Unable to create dvfs debugfs entry");
	}

	if (kbdev->need_dynamic_config_ipa_counter) {
		file = debugfs_create_file("gpu_profile", 0644, kbdev->mali_debugfs_directory, kbdev,
					&kbasep_gpu_profile_debugfs_fops);
		if (IS_ERR_OR_NULL(file))
			dev_warn(kbdev->dev, "Unable to create gpu_profile debugfs entry");
	}
}

#else
/*
 * Stub functions for when debugfs is disabled
 */
void kbase_dvfs_status_debugfs_init(struct kbase_device *kbdev)
{
}

#endif /* CONFIG_DEBUG_FS */
