// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2020-2026 ARM Limited. All rights reserved.
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

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <mali_kbase_fence.h>
#include <mali_kbase.h>

struct kbase_fence_meta_put_defer {
	struct rcu_head rcu;
	struct kbase_kcpu_dma_fence_meta *metadata;
	struct module *module;
};

static void kbase_fence_meta_put_deferred(struct rcu_head *rcu)
{
	struct kbase_fence_meta_put_defer *meta_defer =
		container_of(rcu, struct kbase_fence_meta_put_defer, rcu);

	kbase_kcpu_dma_fence_meta_put(meta_defer->metadata);
	if (likely(meta_defer->module))
		module_put(meta_defer->module);
	kfree(meta_defer);
}

static const char *kbase_fence_get_driver_name(struct dma_fence *fence)
{
	CSTD_UNUSED(fence);

	return KBASE_DRV_NAME;
}

static const char *kbase_fence_get_timeline_name(struct dma_fence *fence)
{
	struct kbase_kcpu_dma_fence *kcpu_fence = (struct kbase_kcpu_dma_fence *)fence;
	struct kbase_kcpu_dma_fence_meta *metadata = READ_ONCE(kcpu_fence->metadata);

	/* Readers may race with fence release while still protected by the
	 * dma_fence RCU lifetime. Hence the following extends the requirement of
	 * the metadata to be freeed via kfree_rcu().
	 */
	return metadata->timeline_name;
}

static bool kbase_fence_enable_signaling(struct dma_fence *fence)
{
	CSTD_UNUSED(fence);

	return true;
}

#if KERNEL_VERSION(6, 16, 0) > LINUX_VERSION_CODE
static void kbase_fence_fence_value_str(struct dma_fence *fence, char *str, int size)
{
	char *format;

	if (KERNEL_VERSION(5, 1, 0) > LINUX_VERSION_CODE)
		format = "%u";
	else
		format = "%llu";

	if (unlikely(!scnprintf(str, (size_t)size, format, fence->seqno)))
		pr_err("Fail to encode fence seqno to string");
}
#endif

static void kbase_fence_release(struct dma_fence *fence)
{
	struct kbase_kcpu_dma_fence *kcpu_fence =
		container_of(fence, struct kbase_kcpu_dma_fence, base);
	struct kbase_kcpu_dma_fence_meta *metadata = kcpu_fence->metadata;
	struct kbase_fence_meta_put_defer *meta_defer;

	BUILD_BUG_ON(offsetof(struct kbase_kcpu_dma_fence, base) != 0);

	meta_defer = kzalloc(sizeof(*meta_defer), GFP_ATOMIC);
	if (meta_defer) {
		meta_defer->metadata = metadata;
		meta_defer->module = kcpu_fence->module;
	}

	/* Below is a MUST in freeing a kbase_kcpu_dma_fence, i.e. must use
	 * upstream exported dma_fence_free() method. This ensures the alignment
	 * of free-handling in kbase to the kernel upstream framework.
	 */
	dma_fence_free(fence);

	if (meta_defer) {
		/* Ensure the metadata only freed after the fence RCU readers have
		 * gone through a rcu-grace window.
		 */
		call_rcu(&meta_defer->rcu, kbase_fence_meta_put_deferred);
	} else {
		kbase_kcpu_dma_fence_meta_put(metadata);
		if (likely(kcpu_fence->module))
			module_put(kcpu_fence->module);
	}
}

extern const struct dma_fence_ops kbase_fence_ops; /* silence checker warning */
const struct dma_fence_ops kbase_fence_ops = { .wait = dma_fence_default_wait,
					       .get_driver_name = kbase_fence_get_driver_name,
					       .get_timeline_name = kbase_fence_get_timeline_name,
					       .enable_signaling = kbase_fence_enable_signaling,
#if KERNEL_VERSION(6, 16, 0) > LINUX_VERSION_CODE
					       .fence_value_str = kbase_fence_fence_value_str,
#endif
					       .release = kbase_fence_release };

KBASE_EXPORT_TEST_API(kbase_fence_ops);
