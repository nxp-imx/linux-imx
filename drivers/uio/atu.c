// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include "atu.h"

static inline bool is_power_of_two64(u64 v)
{
	return v && !(v & (v - 1));
}

static inline bool is_aligned64(u64 addr, u64 align)
{
	return align && ((addr & (align - 1)) == 0);
}

static inline int ranges_overlap(u64 a_base, u64 a_size, u64 b_base, u64 b_size)
{
	u64 a_end = a_base + a_size;
	u64 b_end = b_base + b_size;

	/* Inside: A fully within B */
	if (a_base >= b_base && a_end <= b_end)
		return -EEXIST;

	/* Overlap but not contained, since "inside" is already handled
	 * Half-open intervals overlap iff: a_base < b_end && b_base < a_end
	 */

	if (a_base < b_end && b_base < a_end)
		return -ERANGE;

	return 0;
}

/* Convert size in bytes (1MB..2GB, power-of-two) to hardware size code:
 * code = log2(size) - 20; allowed range [0..0x0B]
 */
static int win_size_bytes_to_code(u64 size, u8 *code_out)
{
	u8 code = 0;

	if (!is_power_of_two64(size) || size < SZ_1M || size > (2ULL * SZ_1G))
		return -EINVAL;

	while (size > SZ_1M) {
		size >>= 1;
		code++;
	}

	*code_out = code;
	return 0;
}

/* Build register values (same bitfields as your atu_regs.h) */
static inline u32 build_OWBAR(u8 domain_id, u32 inbound_base, u8 size_code)
{
	u32 dom = ((u32)domain_id & 0xFu) << OWBAR_DOM_SHIFT;
	u32 wba = ((inbound_base >> 20) & OWBAR_WBA_MASK) << OWBAR_WBA_SHIFT;
	u32 sz  = (u32)(size_code & OWBAR_WIN_SIZE_MASK);

	return dom | wba | sz;
}

static inline u32 build_OTAR(u64 outbound_base)
{
	u32 ta = (u32)((outbound_base >> 20) & OTAR_TA_MASK);

	return ta << OTAR_TA_SHIFT;
}

static inline u32 build_OTEAR(u64 outbound_base)
{
	return (u32)((outbound_base >> 32) & OTEAR_TEA_MASK);
}

static inline void atu_hw_set_valid(void __iomem *base, u8 win, bool enable)
{
	u32 off = OWAR_BASE + (u32)(win - 1) * WIN_STRIDE;
	u32 v   = readl(base + off);

	if (enable)
		v |= OWAR_VALID_BIT;
	else
		v &= ~OWAR_VALID_BIT;

	writel(v, base + off);
	readl(base + off); /* flush */
}

static uint8_t get_atu_window(struct atu_dev *ad)
{
	u8 win;
	u32 off_owar;

	for (win = 0; win < ATU_NUM_WINS; win++) {
		off_owar = OWAR_BASE + win * WIN_STRIDE;

		if ((readl(ad->atu_base + off_owar) & OWAR_VALID_BIT) == 0u)
			return (win + 1);
	}

	return 0; /* All present windows are enabled */
}

static int atu_hw_program_window(struct atu_dev *ad,
				 u8 win,
				 struct atu_conf_params *atu_params)
{
	u8 code;
	u32 off_owbar = OWBAR_BASE + (u32)(win - 1) * WIN_STRIDE;
	u32 off_otear = OTEAR_BASE + (u32)(win - 1) * WIN_STRIDE;
	u32 off_otar  = OTAR_BASE  + (u32)(win - 1) * WIN_STRIDE;

	if (win < 1 || win > ATU_NUM_WINS)
		return -EINVAL;

	if (win_size_bytes_to_code(atu_params->win_size, &code))
		return -EINVAL;

	/* Disable then reprogram in order: BAR -> TAR -> TEAR -> VALID */
	atu_hw_set_valid(ad->atu_base, win, false);
	writel(build_OWBAR(ad->domain_id, atu_params->inbound_addr, code),
	       ad->atu_base + off_owbar);
	readl(ad->atu_base + off_owbar);
	writel(build_OTAR(atu_params->outbound_addr), ad->atu_base + off_otar);
	readl(ad->atu_base + off_otar);
	writel(build_OTEAR(atu_params->outbound_addr), ad->atu_base + off_otear);
	readl(ad->atu_base + off_otear);
	atu_hw_set_valid(ad->atu_base, win, true);

	return 0;
}

/* Initialize the manager once in probe() after ioremap() */
int atu_ll_init(struct atu_dev *ad)
{
	if (!ad)
		return -EINVAL;

	ad->domain_id = DOMAIN_ID_A55;
	ad->max_window_size = 0;
	ad->resv_mem_atu_configured = false;

	mutex_init(&ad->mem_lock);
	INIT_LIST_HEAD(&ad->mappings);
	bitmap_zero(ad->win_bmap, ATU_NUM_WINS + 1);

	ad->inbound_num_chunks = (ATU_INBOUND_END - ATU_INBOUND_START) / CHUNK_SIZE;

	ad->inbound_mem_bitmap = bitmap_zalloc(ad->inbound_num_chunks, GFP_KERNEL);
	if (!ad->inbound_mem_bitmap) {
		pr_err("Failed to allocate inbound mem bitmap\n");
		return -ENOMEM;
	}

	/* If reserved memory exists, mark it in the inbound bitmap */
	if (ad->resv_mem_size) {
		ad->resv_num_chunks = ad->resv_mem_size / CHUNK_SIZE;

		ad->resv_mem_bitmap = bitmap_zalloc(ad->resv_num_chunks, GFP_KERNEL);
		if (!ad->resv_mem_bitmap) {
			pr_err("Failed to allocate resv mem bitmap\n");
			return -ENOMEM;
		}

		unsigned long resv_chunks =
			DIV_ROUND_UP(ad->resv_mem_size, CHUNK_SIZE);

		bitmap_set(ad->inbound_mem_bitmap, 0, resv_chunks);
	}

	return 0;
}
EXPORT_SYMBOL(atu_ll_init);

static void atu_dealloc_inbound_address(struct atu_dev *ad,
					u32 inbound_addr,
					u32 size)
{
	u32 start_bit, num_chunks;

	start_bit   = (inbound_addr - ATU_INBOUND_START) / CHUNK_SIZE;
	num_chunks  = DIV_ROUND_UP(size, CHUNK_SIZE);

	bitmap_clear(ad->inbound_mem_bitmap, start_bit, num_chunks);
}

static int atu_alloc_inbound_address(struct atu_dev *ad, u32 size, u32 *inbound_addr)
{
	u32 num_chunks = DIV_ROUND_UP(size, CHUNK_SIZE);
	u32 start_bit;

	/* Find contiguous free inbound region */
	start_bit = bitmap_find_next_zero_area(ad->inbound_mem_bitmap,
					       ad->inbound_num_chunks,
					       0,
					       num_chunks,
					       0);

	if (start_bit >= ad->inbound_num_chunks)
		return -ENOMEM;

	/* Mark allocated */
	bitmap_set(ad->inbound_mem_bitmap, start_bit, num_chunks);

	/* Convert chunk index → actual inbound address */
	*inbound_addr = ATU_INBOUND_START + (start_bit * CHUNK_SIZE);

	/* Final alignment and range check */
	if (!IS_ALIGNED(*inbound_addr, size))
		return -1;

	return 0;
}

/* Insert into list sorted by inbound_base */
static void list_insert_sorted_by_inbound(struct list_head *head, struct atu_mapping *m)
{
	struct atu_mapping *it;

	list_for_each_entry(it, head, list) {
		if (m->inbound_base < it->inbound_base) {
			list_add_tail(&m->list, &it->list);
			return;
		}
	}
	list_add_tail(&m->list, head); /* end */
}

#ifdef DEBUG
void print_atu_regs(struct atu_dev *atu_dev)
{
	struct atu_regs __iomem *r;

	r = (struct atu_regs __iomem *)atu_dev->atu_base;

	pr_info("ATU Registers:\n");
	pr_info("  atucr        = 0x%08X\n", readl(&r->atucr));
	pr_info("  atusr        = 0x%08X\n", readl(&r->atusr));
	pr_info("  ip_rev_1     = 0x%08X\n", readl(&r->ip_rev_1));
	pr_info("  ip_rev_2     = 0x%08X\n", readl(&r->ip_rev_2));
	pr_info("  pmcr         = 0x%08X\n", readl(&r->pmcr));

	pr_info("ACORE Window Registers:\n");
	pr_info("  owar0        = 0x%08X\n", readl(&r->acore_owar0));
	pr_info("  oteAR0       = 0x%08X\n", readl(&r->acore_otear0));

	pr_info("  owbar1       = 0x%08X\n", readl(&r->acore_owbar1));
	pr_info("  owar1        = 0x%08X\n", readl(&r->acore_owar1));
	pr_info("  oteAR1       = 0x%08X\n", readl(&r->acore_otear1));
	pr_info("  otar1        = 0x%08X\n", readl(&r->acore_otar1));

	pr_info("  owbar2       = 0x%08X\n", readl(&r->acore_owbar2));
	pr_info("  owar2        = 0x%08X\n", readl(&r->acore_owar2));
	pr_info("  oteAR2       = 0x%08X\n", readl(&r->acore_otear2));
	pr_info("  otar2        = 0x%08X\n", readl(&r->acore_otar2));

	pr_info("  owbar3       = 0x%08X\n", readl(&r->acore_owbar3));
	pr_info("  owar3        = 0x%08X\n", readl(&r->acore_owar3));
	pr_info("  oteAR3       = 0x%08X\n", readl(&r->acore_otear3));
	pr_info("  otar3        = 0x%08X\n", readl(&r->acore_otar3));

	pr_info("  owbar4       = 0x%08X\n", readl(&r->acore_owbar4));
	pr_info("  owar4        = 0x%08X\n", readl(&r->acore_owar4));
	pr_info("  oteAR4       = 0x%08X\n", readl(&r->acore_otear4));
	pr_info("  otar4        = 0x%08X\n", readl(&r->acore_otar4));

	pr_info("  owbar5       = 0x%08X\n", readl(&r->acore_owbar5));
	pr_info("  owar5        = 0x%08X\n", readl(&r->acore_owar5));
	pr_info("  oteAR5       = 0x%08X\n", readl(&r->acore_otear5));
	pr_info("  otar5        = 0x%08X\n", readl(&r->acore_otar5));

	pr_info("  owbar6       = 0x%08X\n", readl(&r->acore_owbar6));
	pr_info("  owar6        = 0x%08X\n", readl(&r->acore_owar6));
	pr_info("  oteAR6       = 0x%08X\n", readl(&r->acore_otear6));
	pr_info("  otar6        = 0x%08X\n", readl(&r->acore_otar6));

	pr_info("  owbar7       = 0x%08X\n", readl(&r->acore_owbar7));
	pr_info("  owar7        = 0x%08X\n", readl(&r->acore_owar7));
	pr_info("  oteAR7       = 0x%08X\n", readl(&r->acore_otear7));
	pr_info("  otar7        = 0x%08X\n", readl(&r->acore_otar7));
}
#endif

/* Find mapping by outbound (phys) */
static struct atu_mapping *atu_find_by_phys_locked(struct atu_dev *ad, u64 phys)
{
	struct atu_mapping *m;

	list_for_each_entry(m, &ad->mappings, list)
		if (m->outbound_base == phys)
			return m;

	return NULL;
}

int deconfigure_atu(struct atu_dev *atu_dev,
		    struct atu_conf_params *atu_params)
{
	struct atu_mapping *m;

	mutex_lock(&atu_dev->mem_lock);

	m = atu_find_by_phys_locked(atu_dev, atu_params->outbound_addr);
	if (!m) {
		mutex_unlock(&atu_dev->mem_lock);
		return -ENOENT;
	}

	/* disable HW and free window */
	atu_hw_set_valid(atu_dev->atu_base, m->win, false);

	list_del(&m->list);
	kfree(m);

	atu_dealloc_inbound_address(atu_dev, m->inbound_base, atu_params->win_size);
	mutex_unlock(&atu_dev->mem_lock);

	return 0;
}

int configure_atu(struct atu_dev *atu_dev, struct atu_conf_params *atu_params)
{
	struct atu_mapping *it, *m;
	u8 win;
	int ret;

	/* Validate input */
	if (!is_power_of_two64(atu_params->win_size) || atu_params->win_size < SZ_1M)
		return -EINVAL;

	if (!is_aligned64(atu_params->outbound_addr, atu_params->win_size))
		return -EINVAL;

	/* outbound must be >= 32-bit */
	if (atu_params->outbound_addr < (1ULL << 32))
		return -EINVAL;

	mutex_lock(&atu_dev->mem_lock);

	/* Outbound overlap check */
	list_for_each_entry(it, &atu_dev->mappings, list) {
		ret = ranges_overlap(atu_params->outbound_addr,
				     atu_params->win_size,
				     it->outbound_base,
				     it->size);

		if (ret == -ERANGE) {
			mutex_unlock(&atu_dev->mem_lock);
			return ret;
		} else if (ret == -EEXIST) {
			atu_params->inbound_addr = it->inbound_base +
						   (atu_params->outbound_addr -
						   it->outbound_base);
			mutex_unlock(&atu_dev->mem_lock);
			return ret;
		}
	}

	/* Allocate inbound region */
	if (atu_params->outbound_addr == atu_dev->resv_mem_phys_addr)
		atu_params->inbound_addr = ATU_INBOUND_START;
	else
		ret = atu_alloc_inbound_address(atu_dev,
						atu_params->win_size,
						&atu_params->inbound_addr);
	if (ret < 0 || atu_params->inbound_addr > ATU_INBOUND_END) {
		mutex_unlock(&atu_dev->mem_lock);
		return -ENOMEM;
	}

	/* Get a free window 1..7 */
	win = get_atu_window(atu_dev);
	if (!win) {
		mutex_unlock(&atu_dev->mem_lock);
		return -ENOSPC;
	}

	/* Program HW */
	if (atu_hw_program_window(atu_dev, win, atu_params)) {
		mutex_unlock(&atu_dev->mem_lock);
		return -EINVAL;
	}

	/* Track mapping */
	m = kzalloc(sizeof(struct atu_mapping), GFP_KERNEL);
	if (!m) {
		atu_hw_set_valid(atu_dev->atu_base, win, false);
		mutex_unlock(&atu_dev->mem_lock);
		return -ENOMEM;
	}
	m->win           = win;
	m->inbound_base  = atu_params->inbound_addr;
	m->outbound_base = atu_params->outbound_addr;
	m->size          = atu_params->win_size;

#ifdef DEBUG
	pr_info("ATU configuration done win: %d : 0x%llx : 0x%x : 0x%x\n",
		m->win, m->outbound_base, m->inbound_base, m->size);
#endif
	/* insert keeping inbound order */
	list_insert_sorted_by_inbound(&atu_dev->mappings, m);

	mutex_unlock(&atu_dev->mem_lock);

#ifdef DEBUG
	print_atu_regs(atu_dev);
#endif
	return 0;
}

/*
 * Reserve contiguous memory chunks
 */
int atu_resv_mem(struct atu_dev *atu_dev, struct atu_resv_mem_req *req)
{
	unsigned long resv_num_chunks_needed;
	unsigned long start_bit;
	int ret = 0;

	if (!req->size || req->size > atu_dev->resv_mem_size)
		return -EINVAL;

	/* Calculate number of 1MB chunks needed (round up) */
	resv_num_chunks_needed = DIV_ROUND_UP(req->size, CHUNK_SIZE);

	if (!atu_dev->resv_mem_atu_configured) {
		struct atu_conf_params atu_params;

		atu_params.outbound_addr = atu_dev->resv_mem_phys_addr;
		atu_params.win_size = atu_dev->resv_mem_size;

		ret = configure_atu(atu_dev, &atu_params);
		if (ret) {
			pr_err("Failed to configure ATU for resvd memory\n");
			return ret;
		}
		atu_dev->resv_mem_atu_configured = true;
	}

	mutex_lock(&atu_dev->mem_lock);

	/* Search for contiguous free chunks */
	start_bit = bitmap_find_next_zero_area(atu_dev->resv_mem_bitmap,
					       atu_dev->resv_num_chunks,
					       0, resv_num_chunks_needed, 0);

	if (start_bit >= atu_dev->resv_num_chunks) {
		ret = -ENOMEM;
		goto out;
	}

	/* Mark chunks as allocated */
	bitmap_set(atu_dev->resv_mem_bitmap, start_bit, resv_num_chunks_needed);

	/* Calculate physical address */
	req->phys_addr = atu_dev->resv_mem_phys_addr + (start_bit * CHUNK_SIZE);
	req->inbound_addr = ATU_INBOUND_START + (start_bit * CHUNK_SIZE);

out:
	mutex_unlock(&atu_dev->mem_lock);

	return ret;
}

/*
 * Free previously reserved memory chunks
 */
int atu_free_resv_mem(struct atu_dev *atu_dev, struct atu_resv_mem_req *req)
{
	unsigned long start_bit;
	unsigned long resv_num_chunks;
	u64 offset;

	if (!req->size || req->phys_addr < atu_dev->resv_mem_phys_addr)
		return -EINVAL;

	offset = req->phys_addr - atu_dev->resv_mem_phys_addr;

	if (offset >= atu_dev->resv_mem_size)
		return -EINVAL;

	/* Check alignment */
	if (offset % CHUNK_SIZE)
		return -EINVAL;

	start_bit = offset / CHUNK_SIZE;
	resv_num_chunks = DIV_ROUND_UP(req->size, CHUNK_SIZE);

	if (start_bit + resv_num_chunks > atu_dev->resv_num_chunks)
		return -EINVAL;

	mutex_lock(&atu_dev->mem_lock);
	bitmap_clear(atu_dev->resv_mem_bitmap, start_bit, resv_num_chunks);
	mutex_unlock(&atu_dev->mem_lock);

	return 0;
}

/*
 * Get memory information
 */
int atu_get_resv_mem_info(struct atu_dev *atu_dev,
			  struct atu_resv_mem_info *info)
{
	info->phys_addr = atu_dev->resv_mem_phys_addr;
	info->size = atu_dev->resv_mem_size;
	return 0;
}

/*
 * mmap provided memory in calling process's address space.
 */
int atu_mmap(struct atu_dev *atu_dev, struct vm_area_struct *vma)
{
	unsigned long vsize   = vma->vm_end - vma->vm_start;      // bytes
	unsigned long poff   = vma->vm_pgoff << PAGE_SHIFT;       // bytes
	phys_addr_t   start   = atu_dev->resv_mem_phys_addr + poff;
	unsigned long max_len = (unsigned long)atu_dev->resv_mem_size;

	/* Basic range checks */
	if (poff > max_len || (poff + vsize) > max_len)
		return -EINVAL;

	mutex_lock(&atu_dev->mem_lock);

	/* Perform the actual mapping: user VA -> PFNs */
	unsigned long pfn = (unsigned long)(start >> PAGE_SHIFT);

	if (remap_pfn_range(vma, vma->vm_start, pfn, vsize, vma->vm_page_prot))
		return -EAGAIN;

	mutex_unlock(&atu_dev->mem_lock);

	return 0;
}
EXPORT_SYMBOL(atu_mmap);

/*
 * IOCTL handler for miscdevice
 */
long atu_ioctl(struct atu_dev *atu_dev, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret;

	switch (cmd) {
	case ATU_IOC_CONFIG: {
		struct atu_conf_params atu_params;

		if (copy_from_user(&atu_params, argp, sizeof(atu_params)))
			return -EFAULT;

		ret = configure_atu(atu_dev, &atu_params);
		if (copy_to_user((void __user *)arg, &atu_params, sizeof(atu_params)))
			return -EFAULT;

		return ret;
	}

	case ATU_IOC_DECONFIG: {
		struct atu_conf_params atu_params;

		if (copy_from_user(&atu_params, argp, sizeof(atu_params)))
			return -EFAULT;

		ret = deconfigure_atu(atu_dev, &atu_params);

		return ret;
	}

	case ATU_GET_RESV_MEM: {
		struct atu_resv_mem_req req;

		if (copy_from_user(&req, argp, sizeof(req)))
			return -EFAULT;

		ret = atu_resv_mem(atu_dev, &req);
		if (ret)
			return ret;

		if (copy_to_user(argp, &req, sizeof(req)))
			return -EFAULT;
		break;
	}

	case ATU_FREE_RESV_MEM: {
		struct atu_resv_mem_req req;

		if (copy_from_user(&req, argp, sizeof(req)))
			return -EFAULT;

		ret = atu_free_resv_mem(atu_dev, &req);
		break;
	}

	case ATU_GET_RESV_MEM_INFO: {
		struct atu_resv_mem_info info;

		ret = atu_get_resv_mem_info(atu_dev, &info);
		if (ret)
			return ret;

		if (copy_to_user(argp, &info, sizeof(info)))
			return -EFAULT;
		break;
	}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}
EXPORT_SYMBOL(atu_ioctl);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ATU Driver for UIO");
