// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "gunyah_vm_mgr_cma_mem: " fmt
#include <linux/anon_inodes.h>
#include <linux/cma.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/rwsem.h>

#include <linux/sched/mm.h>
#include "vm_mgr.h"

struct gunyah_cma {
	struct device dev;
	struct mutex lock;
	bool fd_active;
	struct page *page;
	struct miscdevice miscdev;
	struct list_head list;
	unsigned long max_size;
	unsigned long mapped_size;
	bool is_shared_parcel;
	struct rw_semaphore parcel_lock;
};

struct gunyah_cma_parent {
	struct list_head gunyah_cma_children;
};

/*
 * gunyah_cma_alloc - Allocate cma region.
 * @cma: the gunyah cma memory
 * @len: the size of the cma region.
 *
 * Uses cma_alloc to allocate contiguous memory region of size len.
 *
 * Return: The 0 on success or an error.
 */
static int gunyah_cma_alloc(struct gunyah_cma *cma, struct file *file, loff_t len)
{
	pgoff_t pagecount = len >> PAGE_SHIFT;
	unsigned long align = get_order(len);
	loff_t max_size;

	guard(mutex)(&cma->lock);
	if (cma->page)
		return -EINVAL;

	max_size = i_size_read(file_inode(file));
	if (len > max_size)
		return -EINVAL;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	cma->page = cma_alloc(cma->dev.cma_area, pagecount, align, false);
	if (!cma->page)
		return -ENOMEM;

	if (len < max_size)
		dev_dbg(&cma->dev, "client mapped %lld bytes, less than max %lld bytes\n",
			len, max_size);

	cma->mapped_size = len;

	return 0;
}

static int gunyah_cma_release(struct inode *inode, struct file *file)
{
	struct gunyah_cma *cma = file->private_data;
	unsigned int count;

	guard(mutex)(&cma->lock);
	cma->fd_active = false;

	if (!cma->page)
		return 0;

	count = PAGE_ALIGN(cma->mapped_size) >> PAGE_SHIFT;
	cma_release(cma->dev.cma_area, cma->page, count);
	cma->page = NULL;
	cma->mapped_size = 0;

	return 0;
}

static int gunyah_cma_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct gunyah_cma *cma = file->private_data;
	unsigned long len = vma->vm_end - vma->vm_start;
	int nr_pages = PAGE_ALIGN(len) >> PAGE_SHIFT;
	struct page **pages;
	int ret, i;

	file_accessed(file);

	pages = kvmalloc_array(nr_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	ret = gunyah_cma_alloc(cma, file, len);
	if (ret < 0) {
		kvfree(pages);
		return ret;
	}

	for (i = 0; i < nr_pages; i++)
		pages[i] = cma->page + i;

	ret = vm_map_pages_zero(vma, pages, nr_pages);
	if (ret)
		pr_err("Mapping memory failed: %d\n", ret);

	kvfree(pages);
	return ret;
}

static ssize_t gunyah_cma_read(struct file *file, char __user *ubuf,
							size_t count, loff_t *offset)
{
	struct gunyah_cma *cma = file->private_data;
	int ret;

	if (!cma->page)
		return -EINVAL;

	ret = down_read_interruptible(&cma->parcel_lock);
	if (ret)
		return ret;

	if (cma->is_shared_parcel) {
		up_read(&cma->parcel_lock);
		return -EACCES;
	}

	ret = simple_read_from_buffer(ubuf, count, offset,
				       page_to_virt(cma->page), cma->mapped_size);
	up_read(&cma->parcel_lock);
	return ret;
}

static const struct file_operations gunyah_cma_fops = {
	.owner = THIS_MODULE,
	.llseek = generic_file_llseek,
	.mmap = gunyah_cma_mmap,
	.open = generic_file_open,
	.read = gunyah_cma_read,
	.release = gunyah_cma_release,
};

int gunyah_cma_reclaim_parcel(struct gunyah_vm *ghvm, struct gunyah_vm_parcel *vm_parcel,
				struct gunyah_vm_binding *b)
{
	struct gunyah_rm_mem_parcel *parcel = &vm_parcel->parcel;
	struct gunyah_cma *cma;
	int ret;

	if (parcel->mem_handle == GUNYAH_MEM_HANDLE_INVAL)
		return 0;

	cma = b->cma.file->private_data;
	down_write(&cma->parcel_lock);
	ret = gunyah_rm_mem_reclaim(ghvm->rm, parcel);
	if (ret) {
		dev_err(ghvm->parent, "Failed to reclaim parcel: %d\n",
			ret);
		/* We can't reclaim the pages -- hold onto the pages
		 * forever because we don't know what state the memory
		 * is in
		 */
		up_write(&cma->parcel_lock);
		return ret;
	}
	cma->is_shared_parcel = false;
	up_write(&cma->parcel_lock);

	parcel->mem_handle = GUNYAH_MEM_HANDLE_INVAL;
	kfree(parcel->mem_entries);
	kfree(parcel->acl_entries);
	vm_parcel->start = 0;
	vm_parcel->pages = 0;
	b->vm_parcel = NULL;
	return ret;
}

int gunyah_cma_share_parcel(struct gunyah_vm *ghvm, struct gunyah_vm_parcel *vm_parcel,
				struct gunyah_vm_binding *b, u64 *gfn, u64 *nr)
{
	struct gunyah_rm_mem_parcel *parcel = &vm_parcel->parcel;
	u64 offset, page_offset;
	struct gunyah_cma *cma;
	int ret;

	if ((*nr << PAGE_SHIFT) > b->size)
		return -EINVAL;

	cma = b->cma.file->private_data;
	if (!cma->page)
		return -EINVAL;

	offset = gunyah_gfn_to_gpa(*gfn) - b->guest_phys_addr;
	page_offset = (b->cma.offset + offset) >> PAGE_SHIFT;
	if (page_offset + *nr > (cma->mapped_size >> PAGE_SHIFT))
		return -EINVAL;

	parcel->n_mem_entries = 1;
	parcel->mem_entries = kcalloc(parcel->n_mem_entries, sizeof(parcel->mem_entries[0]),
					GFP_KERNEL_ACCOUNT);
	if (!parcel->mem_entries)
		return -ENOMEM;

	parcel->mem_entries[0].size = cpu_to_le64(*nr << PAGE_SHIFT);
	parcel->mem_entries[0].phys_addr =
		cpu_to_le64(page_to_phys(cma->page + page_offset));

	down_write(&cma->parcel_lock);
	ret = gunyah_rm_mem_share(ghvm->rm, parcel);
	if (ret) {
		up_write(&cma->parcel_lock);
		goto free_mem_entries;
	}
	cma->is_shared_parcel = true;
	up_write(&cma->parcel_lock);

	vm_parcel->start = *gfn;
	vm_parcel->pages = *nr;
	b->vm_parcel = vm_parcel;
	return ret;

free_mem_entries:
	kfree(parcel->mem_entries);
	parcel->mem_entries = NULL;
	parcel->n_mem_entries = 0;
	return ret;
}

int gunyah_vm_binding_cma_alloc(struct gunyah_vm *ghvm,
			    struct gunyah_map_cma_mem_args *cma_map)
{
	struct gunyah_vm_binding *binding;
	struct file *file;
	u64 end;
	loff_t max_size;
	int ret = 0;

	if (!cma_map->size || !PAGE_ALIGNED(cma_map->size) ||
		!PAGE_ALIGNED(cma_map->guest_addr))
		return -EINVAL;

	if (overflows_type(cma_map->guest_addr + cma_map->size, u64))
		return -EOVERFLOW;

	/*
	 * The file reference is held for the lifetime of the binding.
	 * The corresponding fput() is done in _gunyah_vm_put() when
	 * the binding is freed.
	 */
	file = fget(cma_map->guest_mem_fd);
	if (!file)
		return -EBADF;

	if (file->f_op != &gunyah_cma_fops) {
		fput(file);
		return -EINVAL;
	}

	max_size = i_size_read(file_inode(file));
	if (check_add_overflow(cma_map->offset, cma_map->size, &end) ||
	    end > max_size) {
		fput(file);
		return -EOVERFLOW;
	}

	binding = kzalloc(sizeof(*binding), GFP_KERNEL_ACCOUNT);
	if (!binding) {
		fput(file);
		return -ENOMEM;
	}

	binding->mem_type = VM_MEM_CMA;
	binding->cma.file = file;
	binding->cma.offset = cma_map->offset;
	binding->guest_phys_addr = cma_map->guest_addr;
	binding->label = cma_map->label;
	binding->size = cma_map->size;
	binding->flags = cma_map->flags;
	binding->vm_parcel = NULL;

	if (binding->flags & GUNYAH_MEM_FORCE_LEND)
		binding->share_type = VM_MEM_LEND;
	else
		binding->share_type = VM_MEM_SHARE;

	down_write(&ghvm->bindings_lock);
	ret = mtree_insert_range(&ghvm->bindings,
				 gunyah_gpa_to_gfn(binding->guest_phys_addr),
				 gunyah_gpa_to_gfn(binding->guest_phys_addr + cma_map->size - 1),
				 binding, GFP_KERNEL);

	if (ret != 0) {
		fput(file);
		kfree(binding);
	}

	up_write(&ghvm->bindings_lock);

	return ret;
}

static long gunyah_cma_create_mem_fd(struct gunyah_cma *cma)
{
	unsigned long flags = 0;
	struct inode *inode;
	struct file *file;
	int fd, err;

	guard(mutex)(&cma->lock);
	/*
	 * Allow only one anon fd at a time. Each open creates a new struct
	 * file with private_data pointing to the same singleton struct
	 * gunyah_cma. A second fd could call cma_release() and free the CMA
	 * pages that were allocated by the first fd's mmap(), leaving the
	 * first fd's VMA mapping freed memory.
	 */
	if (cma->fd_active)
		return -EBUSY;

	flags |= O_CLOEXEC;
	fd = get_unused_fd_flags(flags);
	if (fd < 0)
		return fd;

	file = anon_inode_create_getfile("[gunyah-cma]", &gunyah_cma_fops,
					cma, O_RDWR, NULL);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto err_put_fd;
	}

	inode = file->f_inode;
	inode->i_mode |= S_IFREG;
	/* Platform specific size of CMA per VM */
	i_size_write(inode, cma->max_size);

	file->f_flags |= O_LARGEFILE;
	file->f_mapping = inode->i_mapping;
	fd_install(fd, file);
	cma->fd_active = true;

	return fd;
err_put_fd:
	put_unused_fd(fd);
	return err;
}

static long gunyah_cma_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct miscdevice *miscdev = filp->private_data;
	struct gunyah_cma *cma = container_of(miscdev, struct gunyah_cma, miscdev);

	switch (cmd) {
	case GH_ANDROID_CREATE_CMA_MEM_FD: {
		return gunyah_cma_create_mem_fd(cma);
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations gunyah_cma_dev_fops = {
	/* clang-format off */
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= gunyah_cma_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.llseek		= noop_llseek,
	/* clang-format on */
};

static void gunyah_cma_device_release(struct device *dev)
{
	struct gunyah_cma *cma = container_of(dev, struct gunyah_cma, dev);

	kfree(cma);
}

static int gunyah_cma_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const char **mem_name;
	struct gunyah_cma_parent *pcma;
	int mem_count, i, ret, err = 0;
	struct device_node *mem_node;
	struct reserved_mem *rmem;

	mem_count = of_property_count_strings(node, "memory-region-names");
	if (!mem_count)
		return -EINVAL;

	mem_name = kmalloc_array(mem_count, sizeof(char *), GFP_KERNEL);
	if (!mem_name)
		return -ENOMEM;

	mem_count = of_property_read_string_array(node, "memory-region-names",
							mem_name, mem_count);
	if (mem_count < 0) {
		err = -EINVAL;
		goto err_mem_name;
	}

	pcma = devm_kzalloc(dev, sizeof(*pcma), GFP_KERNEL);
	if (!pcma) {
		err = -ENOMEM;
		goto err_mem_name;
	}
	INIT_LIST_HEAD(&pcma->gunyah_cma_children);

	for (i = 0; i < mem_count; i++) {
		struct gunyah_cma *cma;
		rmem = NULL;

		cma = kzalloc(sizeof(*cma), GFP_KERNEL);
		if (!cma) {
			ret = -ENOMEM;
			goto err_continue;
		}

		mutex_init(&cma->lock);
		init_rwsem(&cma->parcel_lock);
		cma->miscdev.parent = &pdev->dev;
		cma->miscdev.name = mem_name[i];
		cma->miscdev.minor = MISC_DYNAMIC_MINOR;
		cma->miscdev.fops = &gunyah_cma_dev_fops;

		ret = misc_register(&cma->miscdev);
		if (ret) {
			kfree(cma);
			goto err_continue;
		}

		device_initialize(&cma->dev);
		cma->dev.parent = &pdev->dev;
		cma->dev.release = gunyah_cma_device_release;
		cma->dev.init_name = mem_name[i];

		ret = of_reserved_mem_device_init_by_name(&cma->dev,
						dev->of_node, mem_name[i]);
		if (ret)
			goto err_device;

		mem_node = of_parse_phandle(dev->of_node, "memory-region", i);
		if (mem_node)
			rmem = of_reserved_mem_lookup(mem_node);
		of_node_put(mem_node);
		if (!rmem) {
			dev_err(dev, "Failed to find reserved memory for %s\n", mem_name[i]);
			goto err_device;
		}
		cma->max_size = rmem->size;
		cma->page = NULL;
		cma->is_shared_parcel = false;
		list_add(&cma->list, &pcma->gunyah_cma_children);
		dev_dbg(dev, "Created a reserved cma pool for %s\n", mem_name[i]);
		continue;

err_device:
		misc_deregister(&cma->miscdev);
		put_device(&cma->dev);
err_continue:
		dev_err(dev, "Failed to create reserved cma pool for %s %d\n", mem_name[i], ret);
		continue;
	}

	platform_set_drvdata(pdev, pcma);

err_mem_name:
	kfree(mem_name);
	return err;
}

static void gunyah_cma_remove(struct platform_device *pdev)
{
	struct gunyah_cma_parent *pcma = platform_get_drvdata(pdev);
	struct gunyah_cma *cma, *iter;

	list_for_each_entry_safe(cma, iter, &pcma->gunyah_cma_children, list) {
		misc_deregister(&cma->miscdev);
		of_reserved_mem_device_release(&cma->dev);
		put_device(&cma->dev);
	}
}

static const struct of_device_id gunyah_cma_match_table[] = {
	{ .compatible = "gunyah-cma-vm-mem"},
	{}
};

static struct platform_driver gunyah_cma_driver = {
	.probe = gunyah_cma_probe,
	.remove = gunyah_cma_remove,
	.driver = {
		.name = "gunyah_cma_vm_mem_driver",
		.of_match_table = gunyah_cma_match_table,
	},
};

int gunyah_cma_mem_init(void)
{
	return platform_driver_register(&gunyah_cma_driver);
}

void gunyah_cma_mem_exit(void)
{
	platform_driver_unregister(&gunyah_cma_driver);
}
