/*
 * drivers/gpu/mxc/mxc_ion.c
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/dma-buf.h>

#include "../ion_priv.h"
#include "../ion_of.h"

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
static struct ion_device *idev;
static int num_heaps = 1;
static struct ion_heap **heaps;
static int cacheable;

static struct ion_of_heap mxc_heaps[] = {
	PLATFORM_HEAP("fsl,user-heap", 0, ION_HEAP_TYPE_DMA, "user"),
	PLATFORM_HEAP("fsl,display-heap", 1, ION_HEAP_TYPE_UNMAPPED, "display"),
	PLATFORM_HEAP("fsl,vpu-heap", 2, ION_HEAP_TYPE_UNMAPPED, "vpu"),
	{}
};

static int rmem_imx_device_init(struct reserved_mem *rmem, struct device *dev)
{
	dev_set_drvdata(dev, rmem);
	return 0;
}

static const struct reserved_mem_ops rmem_dma_ops = {
	.device_init	= rmem_imx_device_init,
};

static int __init rmem_imx_ion_setup(struct reserved_mem *rmem)
{
	rmem->ops = &rmem_dma_ops;
	pr_info("Reserved memory: created ION memory pool at %pa, size %ld MiB\n",
		&rmem->base, (unsigned long)rmem->size / SZ_1M);
	return 0;
}

RESERVEDMEM_OF_DECLARE(cma, "imx-ion-pool", rmem_imx_ion_setup);

/* Note: original code could be adapted so we just call ion_parse_dt() */
struct ion_platform_data *mxc_ion_parse_of(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = 0;
	const struct device_node *node = pdev->dev.of_node;
	int ret = 0;
	unsigned int val = 0;
	struct ion_platform_heap *heaps = NULL;
	struct ion_platform_heap *user_heap;

	pdata = ion_parse_dt(pdev, mxc_heaps);
	if (IS_ERR(pdata)) {
		if (PTR_ERR(pdata) != -EINVAL)
			return pdata;

		/* Even if DT has no heap, we still want the user one */
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return ERR_PTR(-ENOMEM);
	} else {
		/* Get rmem info into heap */
		int i;

		for (i = 0; i < pdata->nr; i++) {
			struct ion_platform_heap *heap = &pdata->heaps[i];
			struct reserved_mem *rmem = dev_get_drvdata(heap->priv);

			heap->base = rmem->base;
			heap->size = rmem->size;
		}
	}

	/* Add user heap for legacy + copy device tree ones */
	heaps = devm_kzalloc(&pdev->dev,
			     sizeof(struct ion_platform_heap) * (pdata->nr + 1),
			     GFP_KERNEL);
	if (!heaps) {
		ion_destroy_platform_data(pdata);
		return ERR_PTR(-ENOMEM);
	}

	if (pdata->heaps) {
		memcpy(heaps, pdata->heaps,
		       sizeof(struct ion_platform_heap) * pdata->nr);
		devm_kfree(&pdev->dev, pdata->heaps);
	}

	pdata->heaps = heaps;

	/* Add the VPU heap */
	user_heap = &heaps[pdata->nr];
	user_heap->type = ION_HEAP_TYPE_DMA;
	user_heap->priv = &pdev->dev;
	user_heap->name = "mxc_ion";
	pdata->nr++;

	ret = of_property_read_u32(node, "fsl,heap-cacheable", &val);
	if (!ret)
		cacheable = 1;

	val = 0;
	ret = of_property_read_u32(node, "fsl,heap-id", &val);
	if (!ret)
		user_heap->id = val;
	else
		user_heap->id = 0;

	return pdata;
}

static long
mxc_custom_ioctl(struct ion_client *client, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case ION_IOC_PHYS:
		{
#ifdef CONFIG_ARCH_MXC_ARM64
			/* Don't use ION_IOC_PHYS on ARM64,
			 * use ION_IOC_PHYS_DMA
			 */
			return -EFAULT;
#else
			struct ion_handle *handle;
			struct ion_phys_data data;
			struct device *dev;
			void *vaddr;

			if (copy_from_user(&data, (void __user *)arg,
					   sizeof(struct ion_phys_data)))
				return -EFAULT;
			handle = ion_handle_get_by_id_wrap(client, data.handle);
			if (IS_ERR(handle))
				return PTR_ERR(handle);

			vaddr = ion_map_kernel(client, handle);
			ion_handle_put_wrap(handle);

			if (IS_ERR(vaddr))
				return PTR_ERR(vaddr);

			dev = ion_device_get_by_client(client);
			data.phys = virt_to_dma(dev, vaddr);

			if (copy_to_user((void __user *)arg, &data,
					 sizeof(struct ion_phys_data)))
				return -EFAULT;
			return 0;
#endif
		}
	case ION_IOC_PHYS_DMA:
		{
			int ret = 0;
			struct device *dev = NULL;
			struct dma_buf *dmabuf = NULL;
			unsigned long phys = 0;
			size_t len = 0;
			struct ion_phys_dma_data data;
			const struct vb2_mem_ops *mem_ops =
			    &vb2_dma_contig_memops;
			dma_addr_t *addr;
			void *mem_priv;

			if (copy_from_user(&data, (void __user *)arg,
					   sizeof(struct ion_phys_dma_data)))
				return -EFAULT;

			/* Get physical address from dmafd */
			dmabuf = dma_buf_get(data.dmafd);
			if (!dmabuf)
				return -1;

			dev = ion_device_get_by_client(client);

			mem_priv = mem_ops->attach_dmabuf(dev,
							  dmabuf, dmabuf->size,
							  DMA_BIDIRECTIONAL);
			if (IS_ERR(mem_priv))
				goto err1;
			ret = mem_ops->map_dmabuf(mem_priv);
			if (ret)
				goto err0;

			addr = mem_ops->cookie(mem_priv);
			phys = *addr;
			len = dmabuf->size;

			data.phys = phys;
			data.size = len;
			if (copy_to_user((void __user *)arg, &data,
					 sizeof(struct ion_phys_dma_data))) {
				ret = -EFAULT;
			}

			/* unmap and detach */
			mem_ops->unmap_dmabuf(mem_priv);
err0:
			mem_ops->detach_dmabuf(mem_priv);
err1:
			dma_buf_put(dmabuf);

			if (ret < 0)
				return ret;
			return 0;
		}
	case ION_IOC_PHYS_VIRT:
		{
			struct device *dev = NULL;
			struct ion_phys_virt_data data;
			const struct vb2_mem_ops *mem_ops =
			    &vb2_dma_contig_memops;
			dma_addr_t *addr;
			void *mem_priv;

			if (copy_from_user(&data, (void __user *)arg,
					   sizeof(struct ion_phys_virt_data)))
				return -EFAULT;

			/* Get physical address from virtual address */
			if (!data.virt)
				return -1;

			dev = ion_device_get_by_client(client);

			mem_priv = mem_ops->get_userptr(dev,
							data.virt, data.size,
							DMA_BIDIRECTIONAL);
			if (IS_ERR(mem_priv))
				return -1;

			addr = mem_ops->cookie(mem_priv);
			data.phys = *addr;
			mem_ops->put_userptr(mem_priv);

			if (copy_to_user((void __user *)arg, &data,
					 sizeof(struct ion_phys_virt_data)))
				return -EFAULT;
			return 0;
		}
	default:
		return -ENOTTY;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
struct ion_phys_data32 {
	__s32 handle;		//ion_user_handle_t
	compat_long_t phys;
};

struct ion_phys_dma_data32 {
	compat_long_t phys;
	__u32 size;
	__u32 dmafd;
};

struct ion_phys_virt_data32 {
	compat_long_t virt;
	compat_long_t phys;
	__u32 size;
};

#define ION_IOC_PHYS32             _IOWR(ION_IOC_MAGIC, _IOC_NR(ION_IOC_PHYS), struct ion_phys_data32)
#define ION_IOC_PHYS_DMA32         _IOWR(ION_IOC_MAGIC, _IOC_NR(ION_IOC_PHYS_DMA), struct ion_phys_dma_data32)
#define ION_IOC_PHYS_VIRT32        _IOWR(ION_IOC_MAGIC, _IOC_NR(ION_IOC_PHYS_VIRT), struct ion_phys_virt_data32)

static int get_ion_phys_data32(struct ion_phys_data *kp,
			       struct ion_phys_data32 __user *up)
{
	void __user *up_pln;
	compat_long_t p;

	if (!access_ok(VERIFY_READ, up, sizeof(struct ion_phys_data32)) ||
	    get_user(kp->handle, &up->handle) || get_user(p, &up->phys))
		return -EFAULT;
	up_pln = compat_ptr(p);
	put_user((unsigned long)up_pln, &kp->phys);
	return 0;
}

static int put_ion_phys_data32(struct ion_phys_data *kp,
			       struct ion_phys_data32 __user *up)
{
	u32 tmp = (u32)((unsigned long)kp->phys);

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct ion_phys_data32)) ||
	    put_user((s32)kp->handle, &up->handle) || put_user(tmp, &up->phys))
		return -EFAULT;
	return 0;
}

static int get_ion_phys_dma_data32(struct ion_phys_dma_data *kp,
				   struct ion_phys_dma_data32 __user *up)
{
	void __user *up_pln;
	compat_long_t p;

	if (!access_ok(VERIFY_READ, up, sizeof(struct ion_phys_dma_data32)) ||
	    get_user(kp->size, &up->size) ||
	    get_user(kp->dmafd, &up->dmafd) || get_user(p, &up->phys))
		return -EFAULT;
	up_pln = compat_ptr(p);
	put_user((unsigned long)up_pln, &kp->phys);
	return 0;
}

static int put_ion_phys_dma_data32(struct ion_phys_dma_data *kp,
				   struct ion_phys_dma_data32 __user *up)
{
	u32 tmp = (u32)((unsigned long)kp->phys);

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct ion_phys_dma_data32)) ||
	    put_user((s32)kp->size, &up->size) ||
	    put_user((s32)kp->dmafd, &up->dmafd) || put_user(tmp, &up->phys))
		return -EFAULT;
	return 0;
}

static int get_ion_phys_virt_data32(struct ion_phys_virt_data *kp,
				    struct ion_phys_virt_data32 __user *up)
{
	void __user *up_pln;
	void __user *up_pln2;
	compat_long_t p;
	compat_long_t p2;

	if (!access_ok(VERIFY_READ, up, sizeof(struct ion_phys_virt_data32)) ||
	    get_user(kp->size, &up->size) ||
	    get_user(p, &up->phys) || get_user(p2, &up->virt))
		return -EFAULT;
	up_pln = compat_ptr(p);
	up_pln2 = compat_ptr(p2);
	put_user((unsigned long)up_pln, &kp->phys);
	put_user((unsigned long)up_pln2, &kp->virt);
	return 0;
}

static int put_ion_phys_virt_data32(struct ion_phys_virt_data *kp,
				    struct ion_phys_virt_data32 __user *up)
{
	u32 tmp = (u32)((unsigned long)kp->phys);
	u32 tmp2 = (u32)((unsigned long)kp->virt);

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct ion_phys_virt_data32)) ||
	    put_user((u32)kp->size, &up->size) ||
	    put_user(tmp, &up->phys) || put_user(tmp2, &up->virt))
		return -EFAULT;
	return 0;
}

static long
mxc_custom_compat_ioctl(struct ion_client *client, unsigned int cmd,
			unsigned long arg)
{
#define MXC_CUSTOM_IOCTL32(r, cli, c, arg) {\
		mm_segment_t old_fs = get_fs();\
		set_fs(KERNEL_DS);\
		r = mxc_custom_ioctl(cli, c, arg);\
		set_fs(old_fs);\
}

	union {
		struct ion_phys_data kdata;
		struct ion_phys_dma_data kdmadata;
		struct ion_phys_virt_data kvirtdata;
	} karg;

	void __user *up = compat_ptr(arg);
	long err = 0;

	switch (cmd) {
	case ION_IOC_PHYS32:
		err = get_ion_phys_data32(&karg.kdata, up);
		if (err)
			return err;
		MXC_CUSTOM_IOCTL32(err, client, ION_IOC_PHYS,
				   (unsigned long)&karg);
		err = put_ion_phys_data32(&karg.kdata, up);
		break;
	case ION_IOC_PHYS_DMA32:
		err = get_ion_phys_dma_data32(&karg.kdmadata, up);
		if (err)
			return err;
		MXC_CUSTOM_IOCTL32(err, client, ION_IOC_PHYS_DMA,
				   (unsigned long)&karg);
		err = put_ion_phys_dma_data32(&karg.kdmadata, up);
		break;
	case ION_IOC_PHYS_VIRT32:
		err = get_ion_phys_virt_data32(&karg.kvirtdata, up);
		if (err)
			return err;
		MXC_CUSTOM_IOCTL32(err, client, ION_IOC_PHYS_VIRT,
				   (unsigned long)&karg);
		err = put_ion_phys_virt_data32(&karg.kvirtdata, up);
		break;
	default:
		err = mxc_custom_ioctl(client, cmd, (unsigned long)arg);
		break;
	}

	return err;
}
#endif //CONFIG_COMPAT

int mxc_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = NULL;
	int err;
	int i;

	if (pdev->dev.of_node)
		pdata = mxc_ion_parse_of(pdev);
	else
		pdata = pdev->dev.platform_data;

	if (IS_ERR_OR_NULL(pdata))
		return PTR_ERR(pdata);

	num_heaps = pdata->nr;

	heaps = devm_kzalloc(&pdev->dev, sizeof(struct ion_heap *) * pdata->nr,
			     GFP_KERNEL);

#ifdef CONFIG_COMPAT
	idev = ion_device_create(mxc_custom_compat_ioctl);
#else
	idev = ion_device_create(mxc_custom_ioctl);
#endif
	if (IS_ERR_OR_NULL(idev)) {
		err = PTR_ERR(idev);
		goto err;
	}

	of_dma_configure(idev->dev.this_device, pdev->dev.of_node);

	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		heaps[i] = ion_heap_create(&pdata->heaps[i]);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			heaps[i] = NULL;
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	for (i = 0; i < pdata->nr; i++)
		if (heaps[i])
			ion_heap_destroy(heaps[i]);

	devm_kfree(&pdev->dev, heaps);
	if (pdev->dev.of_node)
		ion_destroy_platform_data(pdata);

	return err;
}

int mxc_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		ion_heap_destroy(heaps[i]);

	devm_kfree(&pdev->dev, heaps);
	return 0;
}

static struct of_device_id ion_match_table[] = {
	{.compatible = "fsl,mxc-ion"},
	{},
};

static struct platform_driver ion_driver = {
	.probe = mxc_ion_probe,
	.remove = mxc_ion_remove,
	.driver = {
		   .name = "ion-mxc",
		   .of_match_table = ion_match_table,
		   },
};

static int __init ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void __exit ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC ion allocator driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
