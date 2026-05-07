// SPDX-License-Identifier: GPL-2.0-or-later

#define KMSG_COMPONENT "zram_ioctl"
#define pr_fmt(fmt) KMSG_COMPONENT ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/ptrace.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/pagewalk.h>
#include <linux/swapops.h>
#include <uapi/linux/zram_ioctl.h>

#include "zram_drv.h"
#include "zram_ioctl.h"

#define NR_PAGES_UNLIMITED U64_MAX

/* Private data for the page table walker. */
struct zram_process_walk_private {
	struct zram *zram;
	struct zram_pp_ctl *pp_ctl;
	unsigned int cmd;
	u64 nr_remaining_pages;
	unsigned long next_addr;
};

static inline bool can_do_file_pageout(struct vm_area_struct *vma)
{
	if (!vma->vm_file)
		return false;
	/*
	 * paging out pagecache only for non-anonymous mappings that correspond
	 * to the files the calling process could (if tried) open for writing;
	 * otherwise we'd be including shared non-exclusive mappings, which
	 * opens a side channel.
	 */
	return inode_owner_or_capable(&nop_mnt_idmap,
				      file_inode(vma->vm_file)) ||
			file_permission(vma->vm_file, MAY_WRITE) == 0;
}

/*
 * pmd_entry callback for walk_page_range().
 *
 * This function is called for each PMD in a VMA. It checks if the PTE
 * corresponds to a swapped-out page.
 */
static int zram_process_walker(pmd_t *pmd, unsigned long start,
			       unsigned long end, struct mm_walk *walk)
{
	struct zram_process_walk_private *private = walk->private;
	struct zram *zram = private->zram;
	struct zram_pp_ctl *pp_ctl = private->pp_ctl;
	unsigned int cmd = private->cmd;
	struct vm_area_struct *vma = walk->vma;
	struct swap_info_struct *sis;
	pte_t *ptep, pte;
	swp_entry_t entry;
	spinlock_t *ptl;
	unsigned long addr;
	unsigned long index;
	u64 nr_pages = zram->disksize >> PAGE_SHIFT;

	for (addr = start; addr < end; addr += PAGE_SIZE) {
		if (private->nr_remaining_pages == 0) {
			private->next_addr = addr;
			return 1;
		}

		ptep = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
		if (!ptep)
			break;

		pte = ptep_get(ptep);
		pte_unmap_unlock(ptep, ptl);

		/*
		 * We hold pte_offset_map_lock() to take an atomic snapshot of
		 * the PTE into a local variable. Once we release the lock via
		 * pte_unmap_unlock(), the actual page table entry may change
		 * immediately (e.g., the process exits or drops the page).
		 *
		 * This means our local 'pte' variable might technically
		 * contain stale data while we continue to process it. However,
		 * this is safe because:
		 *
		 * 1. If the page was modified (touched, freed or overwritten),
		 *    it will clear the ZRAM_PP_SLOT flag for that entry.
		 * 2. We rely on the ZRAM_PP_SLOT flag as a gatekeeper:
		 * - scan_slots_for_writeback() checks this flag before marking
		 *   a slot for writeback.
		 * - zram_writeback_slots() performs a double-check of this
		 *   flag just before writing.
		 * 3. zram->pp_in_progress also ensures the ZRAM_PP_SLOT flag
		 *    won't be set again since we don't allow concurrent
		 *    post-processing.
		 *
		 * Therefore, even if the PTE changes and our local variable is
		 * stale, the missing ZRAM_PP_SLOT flag will prevent us from
		 * writing back invalid or freed data.
		 */
		entry = pte_to_swp_entry(pte);

		if (!is_swap_pte(pte))
			continue;

		/* prevent the swapoff race condition */
		sis = get_swap_device(entry);
		if (unlikely(!sis))
			continue;
		if (unlikely(!sis->bdev || !sis->bdev->bd_disk ||
			     sis->bdev->bd_disk->private_data != zram))
			goto unlock_swap_device;

		index = swp_offset(entry);
		if (unlikely(index >= nr_pages))
			goto unlock_swap_device;

		if (cmd == ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK) {
			/* Use PAGE_WRITEBACK for single index */
			scan_slots_for_writeback(zram, 0, index, index+1,
						 pp_ctl);
			if (private->nr_remaining_pages != NR_PAGES_UNLIMITED)
				private->nr_remaining_pages--;
		} else if (cmd == ZRAM_ANDROID_IOC_PROCESS_PREFETCH) {
			scan_slot_for_prefetch(zram, index, pp_ctl);
		}

unlock_swap_device:
		put_swap_device(sis);
	}

	cond_resched();
	return 0;
}

static const struct mm_walk_ops zram_walk_ops = {
	.pmd_entry = zram_process_walker,
	.walk_lock = PGWALK_RDLOCK,
};

static int zram_ioctl_process_scan(struct zram *zram, unsigned int cmd,
	u64 pidfd,
	struct zram_android_ioc_process_range_writeback *prwb,
	struct zram_pp_ctl *ctl)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct task_struct *task;
	unsigned long start_addr = 0;
	u64 nr_remaining_pages = 0;
	unsigned int f_flags;
	int ret = 0;

	struct zram_process_walk_private private = {
		.zram = zram,
		.pp_ctl = ctl,
		.cmd = cmd,
	};

	if (cmd == ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK) {
		start_addr = (unsigned long)prwb->start_addr;
		nr_remaining_pages = DIV_ROUND_UP_POW2(prwb->size, PAGE_SIZE);
	}

	if (!nr_remaining_pages)
		private.nr_remaining_pages = NR_PAGES_UNLIMITED;
	else
		private.nr_remaining_pages = nr_remaining_pages;

	task = pidfd_get_task(pidfd, &f_flags);
	if (IS_ERR(task))
		return PTR_ERR(task);

	mm = get_task_mm(task);
	if (!mm) {
		ret = -ESRCH;
		goto release_task;
	}

	if (start_addr >= mm->task_size) {
		ret = -EINVAL;
		goto release_mm;
	}

	if (!IS_ALIGNED(start_addr, PAGE_SIZE)) {
		ret = -EINVAL;
		goto release_mm;
	}

	VMA_ITERATOR(vmi, mm, start_addr);
	/* Iterates through all the VMAs of the process */
	mmap_read_lock(mm);
	for_each_vma(vmi, vma) {
		unsigned long start = max(vma->vm_start, start_addr);

		if (!vma_is_anonymous(vma) && (!can_do_file_pageout(vma) &&
					       (vma->vm_flags & VM_MAYSHARE)))
			continue;

		ret = walk_page_range(mm, start, vma->vm_end,
				      &zram_walk_ops, &private);
		if (ret)
			break;
	}

	if (cmd == ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK) {
		if (ret > 0) {
			prwb->next_addr = private.next_addr;
			ret = 0;
		} else {
			prwb->next_addr = 0;
		}
	}

	mmap_read_unlock(mm);
release_mm:
	mmput(mm);
release_task:
	put_task_struct(task);

	return ret;
}

static int zram_ioctl_process_writeback(struct zram *zram,
	struct zram_android_ioc_process_range_writeback *prwb)
{
	struct zram_pp_ctl *pp_ctl = NULL;
	struct zram_wb_ctl *wb_ctl = NULL;
	int ret;

	/* Require CAP_SYS_NICE for influencing process performance. */
	if (!capable(CAP_SYS_NICE))
		return -EPERM;

	guard(rwsem_read)(&zram->init_lock);
	if (!init_done(zram))
		return -EINVAL;

	if (!zram->backing_dev)
		return -ENODEV;

	/* Do not permit concurrent post-processing actions. */
	if (atomic_xchg(&zram->pp_in_progress, 1))
		return -EAGAIN;

	pp_ctl = init_pp_ctl();
	if (!pp_ctl) {
		ret = -ENOMEM;
		goto clear_pp_in_progress;
	}

	wb_ctl = init_wb_ctl(zram);
	if (!wb_ctl) {
		ret = -ENOMEM;
		goto clear_pp_ctl;
	}

	ret = zram_ioctl_process_scan(zram,
				      ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK,
				      prwb->pidfd, prwb, pp_ctl);
	if (!ret)
		ret = zram_writeback_slots(zram, pp_ctl, wb_ctl);

	prwb->written_bytes = wb_ctl->processed_bytes;

	release_wb_ctl(wb_ctl);
clear_pp_ctl:
	release_pp_ctl(zram, pp_ctl);
clear_pp_in_progress:
	atomic_set(&zram->pp_in_progress, 0);

	return ret;
}

static int zram_ioctl_process_prefetch(struct zram *zram,
	struct zram_android_ioc_process_prefetch *ioc_prefetch)
{
	struct zram_pp_ctl *pp_ctl = NULL;
	int ret;

	/* Require CAP_SYS_NICE for influencing process performance. */
	if (!capable(CAP_SYS_NICE))
		return -EPERM;

	guard(rwsem_read)(&zram->init_lock);
	if (!init_done(zram))
		return -EINVAL;

	if (!zram->backing_dev)
		return -ENODEV;

	/*
	 * Prefetch should preempt writeback for the same process to avoid
	 * blocking the launch. However, Prefetch and Writeback work in
	 * parallel could have a race of block index, even operate in different
	 * processes. Currently, we do not permit concurrent post-processing
	 * actions via pp_in_progress flag.
	 */
	if (atomic_xchg(&zram->pp_in_progress, 1))
		return -EAGAIN;

	pp_ctl = init_pp_ctl();
	if (!pp_ctl) {
		ret = -ENOMEM;
		goto clear_pp_in_progress;
	}

	ret = zram_ioctl_process_scan(zram, ZRAM_ANDROID_IOC_PROCESS_PREFETCH,
				      ioc_prefetch->pidfd, NULL, pp_ctl);
	if (!ret)
		ret = zram_prefetch_slots(zram, pp_ctl);

	release_pp_ctl(zram, pp_ctl);
clear_pp_in_progress:
	atomic_set(&zram->pp_in_progress, 0);
	return ret;
}

int zram_ioctl(struct block_device *bdev, blk_mode_t mode,
	       unsigned int cmd, unsigned long arg)
{
	struct zram *zram = bdev->bd_disk->private_data;
	void __user *argp = (void __user *)arg;
	int ret = -ENOIOCTLCMD;

	if (cmd == ZRAM_ANDROID_IOC_PROCESS_RANGE_WRITEBACK) {
		struct zram_android_ioc_process_range_writeback prwb;

		if (copy_from_user(&prwb, argp, sizeof(prwb)))
			return -EFAULT;

		ret = zram_ioctl_process_writeback(zram, &prwb);

		if (copy_to_user(argp, &prwb, sizeof(prwb)))
			ret = -EFAULT;
	} else if (cmd == ZRAM_ANDROID_IOC_PROCESS_PREFETCH) {
		struct zram_android_ioc_process_prefetch ioc_prefetch;

		if (copy_from_user(&ioc_prefetch, argp, sizeof(ioc_prefetch)))
			return -EFAULT;

		ret = zram_ioctl_process_prefetch(zram, &ioc_prefetch);
	} else if (cmd == ZRAM_ANDROID_IOC_PROCESS_WRITEBACK) {
		/* Legacy: map to unlimited process_range_writeback */
		struct zram_android_ioc_data ioc_data;
		struct zram_android_ioc_process_range_writeback prwb;

		if (copy_from_user(&ioc_data, argp, sizeof(ioc_data)))
			return -EFAULT;

		prwb.pidfd = ioc_data.data.process_writeback.pidfd;
		prwb.start_addr = 0;
		prwb.size = 0;

		ret = zram_ioctl_process_writeback(zram, &prwb);
		ioc_data.data.process_writeback.written_bytes =
			prwb.written_bytes;

		if (copy_to_user(argp, &ioc_data, sizeof(ioc_data)))
			ret = -EFAULT;
	} else if (cmd == ZRAM_ANDROID_IOC_GET_VERSION) {
		return ZRAM_ANDROID_IOC_VERSION;
	}

	return ret;
}
