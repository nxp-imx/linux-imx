// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/gzvm_drv.h>

static struct gzvm_driver gzvm_drv = {
	.drv_version = {
		.major = GZVM_DRV_MAJOR_VERSION,
		.minor = GZVM_DRV_MINOR_VERSION,
		.sub = 0,
	},
};

static ssize_t demand_paging_batch_pages_show(struct file *file,
					      char __user *buf,
					      size_t count,
					      loff_t *ppos)
{
	int len;
	char buffer[16];	/* enough for a u32 integer*/

	len = snprintf(buffer, sizeof(buffer), "%u\n",
		       gzvm_drv.demand_paging_batch_pages);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t demand_paging_batch_pages_store(struct file *file,
					       const char __user *buf,
					       size_t count,
					       loff_t *ppos)
{
	int ret;
	u32 temp;
	char buffer[16];

	if (*ppos != 0)
		return 0;

	if (count > sizeof(buffer))
		return -EINVAL;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	ret = kstrtoint(buf, 10, &temp);
	if (ret < 0)
		return ret;

	if (temp == 0 || (PMD_SIZE % (PAGE_SIZE * temp)) != 0)
		return -EINVAL;

	gzvm_drv.demand_paging_batch_pages = temp;

	return count;
}

/* /sys/kernel/debug/gzvm/demand_paging_batch_pages */
static const struct file_operations demand_paging_batch_pages_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = demand_paging_batch_pages_show,
	.write = demand_paging_batch_pages_store,
};

static ssize_t destroy_batch_pages_show(struct file *file,
					char __user *buf,
					size_t count,
					loff_t *ppos)
{
	int len;
	char buffer[16];	/* enough for a u32 integer*/

	len = snprintf(buffer, sizeof(buffer), "%u\n",
		       gzvm_drv.destroy_batch_pages);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t destroy_batch_pages_store(struct file *file,
					 const char __user *buf,
					 size_t count,
					 loff_t *ppos)
{
	int ret;
	u32 temp;
	char buffer[16];

	if (*ppos != 0)
		return 0;

	if (count > sizeof(buffer))
		return -EINVAL;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	ret = kstrtoint(buf, 10, &temp);
	if (ret < 0)
		return ret;

	/* destroy page batch size should be power of 2 */
	if ((temp & (temp - 1)) != 0)
		return -EINVAL;

	gzvm_drv.destroy_batch_pages = temp;

	return count;
}

/* /sys/kernel/debug/gzvm/destroy_batch_pages */
static const struct file_operations destroy_batch_pages_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = destroy_batch_pages_show,
	.write = destroy_batch_pages_store,
};

static int gzvm_drv_debugfs_init(void)
{
	struct dentry *debugfs_dir;

	debugfs_dir = debugfs_create_dir("gzvm", NULL);
	if (IS_ERR_OR_NULL(debugfs_dir))
		return -ENOMEM;

	gzvm_drv.gzvm_debugfs_dir = debugfs_dir;

	debugfs_create_file("demand_paging_batch_pages", 0660, debugfs_dir,
			    &gzvm_drv, &demand_paging_batch_pages_fops);

	debugfs_create_file("destroy_batch_pages", 0660, debugfs_dir,
			   &gzvm_drv, &destroy_batch_pages_fops);

	return 0;
}

static void gzvm_drv_debugfs_exit(void)
{
	debugfs_remove_recursive(gzvm_drv.gzvm_debugfs_dir);
}

/**
 * gzvm_err_to_errno() - Convert geniezone return value to standard errno
 *
 * @err: Return value from geniezone function return
 *
 * Return: Standard errno
 */
int gzvm_err_to_errno(unsigned long err)
{
	int gz_err = (int)err;

	switch (gz_err) {
	case 0:
		return 0;
	case ERR_NO_MEMORY:
		return -ENOMEM;
	case ERR_INVALID_ARGS:
		return -EINVAL;
	case ERR_NOT_SUPPORTED:
		fallthrough;
	case ERR_NOT_IMPLEMENTED:
		return -EOPNOTSUPP;
	case ERR_FAULT:
		return -EFAULT;
	case ERR_BUSY:
		return -EAGAIN;
	default:
		break;
	}

	return -EINVAL;
}

/**
 * gzvm_dev_ioctl_check_extension() - Check if given capability is support
 *				      or not
 *
 * @gzvm: Pointer to struct gzvm
 * @args: Pointer in u64 from userspace
 *
 * Return:
 * * 0			- Supported, no error
 * * -EOPNOTSUPP	- Unsupported
 * * -EFAULT		- Failed to get data from userspace
 */
long gzvm_dev_ioctl_check_extension(struct gzvm *gzvm, unsigned long args)
{
	__u64 cap;
	void __user *argp = (void __user *)args;

	if (copy_from_user(&cap, argp, sizeof(__u64)))
		return -EFAULT;
	return gzvm_arch_check_extension(gzvm, cap, argp);
}

static long gzvm_dev_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long user_args)
{
	switch (cmd) {
	case GZVM_CREATE_VM:
		return gzvm_dev_ioctl_create_vm(&gzvm_drv, user_args);
	case GZVM_CHECK_EXTENSION:
		return gzvm_dev_ioctl_check_extension(NULL, user_args);
	default:
		break;
	}

	return -ENOTTY;
}

static const struct file_operations gzvm_chardev_ops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl = gzvm_dev_ioctl,
	.llseek		= noop_llseek,
};

static struct miscdevice gzvm_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KBUILD_MODNAME,
	.fops = &gzvm_chardev_ops,
};

static int gzvm_query_hyp_batch_pages(void)
{
	struct gzvm_enable_cap cap = {0};
	int ret;

	gzvm_drv.demand_paging_batch_pages = GZVM_DRV_DEMAND_PAGING_BATCH_PAGES;
	cap.cap = GZVM_CAP_QUERY_HYP_BATCH_PAGES;

	ret = gzvm_arch_query_hyp_batch_pages(&cap, NULL);
	if (!ret)
		gzvm_drv.demand_paging_batch_pages = cap.args[0];

	/*
	 * We have initialized demand_paging_batch_pages, and to maintain
	 * compatibility with older GZ version, we can ignore the return value.
	 */
	if (ret == -EINVAL)
		return 0;
	return ret;
}

static int gzvm_query_destroy_batch_pages(void)
{
	int ret;
	struct gzvm_enable_cap cap = {0};

	gzvm_drv.destroy_batch_pages = GZVM_DRV_DESTROY_PAGING_BATCH_PAGES;
	cap.cap = GZVM_CAP_QUERY_DESTROY_BATCH_PAGES;

	ret = gzvm_arch_query_destroy_batch_pages(&cap, NULL);
	if (!ret)
		gzvm_drv.destroy_batch_pages = cap.args[0];
	return ret;
}

static int gzvm_drv_probe(struct platform_device *pdev)
{
	int ret;

	if (gzvm_arch_probe(gzvm_drv.drv_version, &gzvm_drv.hyp_version) != 0) {
		dev_err(&pdev->dev, "Not found available conduit\n");
		return -ENODEV;
	}

	ret = gzvm_arch_drv_init();
	if (ret)
		return ret;

	ret = misc_register(&gzvm_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register to misc device\n");
		return ret;
	}

	gzvm_drv.dev = gzvm_dev.this_device;
	dev_dbg(gzvm_dev.this_device,
		"Found GenieZone hypervisor version %u.%u.%llu\n",
		gzvm_drv.hyp_version.major, gzvm_drv.hyp_version.minor,
		gzvm_drv.hyp_version.sub);

	ret = gzvm_drv_irqfd_init();
	if (ret)
		goto err_deregister;

	ret = gzvm_query_hyp_batch_pages();
	if (ret)
		goto err_irqfd_exit;

	ret = gzvm_query_destroy_batch_pages();
	if (ret)
		goto err_irqfd_exit;

	ret = gzvm_drv_debugfs_init();
	if (ret)
		goto err_irqfd_exit;

	return 0;

err_irqfd_exit:
	gzvm_drv_irqfd_exit();
err_deregister:
	misc_deregister(&gzvm_dev);
	return ret;
}

static void gzvm_drv_remove(struct platform_device *pdev)
{
	gzvm_drv_irqfd_exit();
	misc_deregister(&gzvm_dev);
	gzvm_drv_debugfs_exit();
}

static const struct of_device_id gzvm_of_match[] = {
	{ .compatible = "mediatek,geniezone" },
	{/* sentinel */},
};

static struct platform_driver gzvm_driver = {
	.probe = gzvm_drv_probe,
	.remove = gzvm_drv_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = gzvm_of_match,
	},
};

module_platform_driver(gzvm_driver);

MODULE_DEVICE_TABLE(of, gzvm_of_match);
MODULE_AUTHOR("MediaTek");
MODULE_DESCRIPTION("GenieZone interface for VMM");
MODULE_LICENSE("GPL");
