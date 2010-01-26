/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/device.h>

static int mxs_device_num;
static int mxs_device_done;
static DEFINE_MUTEX(device_mutex);
static struct list_head mxs_device_level[] = {
	LIST_HEAD_INIT(mxs_device_level[0]),
	LIST_HEAD_INIT(mxs_device_level[1]),
	LIST_HEAD_INIT(mxs_device_level[2]),
	LIST_HEAD_INIT(mxs_device_level[3]),
};

static u64 common_dmamask = DMA_BIT_MASK(32);

void mxs_nop_release(struct device *dev)
{
	/* Nothing */
}

int mxs_add_devices(struct platform_device *pdev, int num, int level)
{
	int i, ret = -ENOMEM;
	if (pdev == NULL || IS_ERR(pdev) || num <= 0)
		return -EINVAL;

	if (level < 0)
		level = 0;
	else if (level >= ARRAY_SIZE(mxs_device_level))
		level = ARRAY_SIZE(mxs_device_level) - 1;

	mutex_lock(&device_mutex);
	if (mxs_device_done) {
		ret = 0;
		for (i = 0; i < num; i++)
			ret |= platform_device_register(pdev + i);
		goto out;
	}

	if ((mxs_device_num + num) > MXS_MAX_DEVICES)
		goto out;
	mxs_device_num += num;
	for (i = 0; i < num; i++)
		list_add_tail(&pdev[i].dev.devres_head,
			      &mxs_device_level[level]);
	ret = 0;
out:
	mutex_unlock(&device_mutex);
	return ret;
}

int mxs_add_device(struct platform_device *pdev, int level)
{
	return mxs_add_devices(pdev, 1, level);
}

#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
static struct platform_device mxs_duart = {
	.name = "mxs-duart",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_MXS_DMA_ENGINE)
static struct platform_device mxs_dma[] = {
	{
	 .name = "mxs-dma-apbh",
	 .id = 0,
	 .dev = {
		 .release = mxs_nop_release,
		 },
	 },
	{
	 .name = "mxs-dma-apbx",
	 .id = 0,
	 .dev = {
		 .release = mxs_nop_release,
		 },
	 },
};
#endif

#if defined(CONFIG_MMC_MXS) || \
	defined(CONFIG_MMC_MXS_MODULE)
static struct platform_device mxs_mmc[] = {
	{
	 .name	= "mxs-mmc",
	 .id	= 0,
	 .dev = {
		.dma_mask               = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
	{
	 .name	= "mxs-mmc",
	 .id	= 1,
	 .dev = {
		.dma_mask               = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
};
#endif

static struct mxs_dev_lookup dev_lookup[] = {
#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
	{
	 .name = "mxs-duart",
	 .size = 1,
	 .pdev = &mxs_duart,
	 },
#endif
#if defined(CONFIG_MXS_DMA_ENGINE)
	{
	 .name = "mxs-dma",
	 .size = ARRAY_SIZE(mxs_dma),
	 .pdev = mxs_dma,
	 },
#endif

#if defined(CONFIG_MMC_MXS) || \
	defined(CONFIG_MMC_MXS_MODULE)
	{
	.name = "mxs-mmc",
	.size = ARRAY_SIZE(mxs_mmc),
	.pdev = mxs_mmc,
	}
#endif
};

struct platform_device *mxs_get_device(char *name, int id)
{
	int i, j;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev = (struct platform_device *)-ENODEV;
	if (name == NULL || id < 0 || IS_ERR(name))
		return (struct platform_device *)-EINVAL;

	mutex_lock(&device_mutex);
	for (i = 0; i < ARRAY_SIZE(dev_lookup); i++) {
		lookup = &dev_lookup[i];
		if (!strcmp(name, lookup->name)) {
			if (test_bit(0, &lookup->lock)) {
				pdev = (struct platform_device *)-EBUSY;
				break;
			}

			if (id >= lookup->size)
				break;
			for (j = 0; j < lookup->size; j++) {
				if (id == (lookup->pdev[j]).id) {
					pdev = &lookup->pdev[j];
					break;
				}
			}
			break;
		}

	}
	mutex_unlock(&device_mutex);
	return pdev;
}

struct mxs_dev_lookup *mxs_get_devices(char *name)
{
	int i;
	struct mxs_dev_lookup *lookup;
	if (name == NULL || IS_ERR(name))
		return (struct mxs_dev_lookup *)-EINVAL;

	mutex_lock(&device_mutex);
	for (i = 0; i < ARRAY_SIZE(dev_lookup); i++) {
		lookup = &dev_lookup[i];
		if (!strcmp(name, lookup->name)) {
			if (test_and_set_bit(0, &lookup->lock))
				lookup = (struct mxs_dev_lookup *)-EBUSY;
			mutex_unlock(&device_mutex);
			return lookup;
		}

	}
	mutex_unlock(&device_mutex);
	return (struct mxs_dev_lookup *)-ENODEV;
}

int mxs_device_init(void)
{
	int i, ret = 0;
	struct list_head *p, *n;
	struct device *dev;
	struct platform_device *pdev;
	mutex_lock(&device_mutex);
	mxs_device_done = 1;
	mutex_unlock(&device_mutex);

	for (i = 0; i < ARRAY_SIZE(mxs_device_level); i++) {
		list_for_each_safe(p, n, mxs_device_level + i) {
			dev = list_entry(p, struct device, devres_head);
			list_del(p);
			pdev = container_of(dev, struct platform_device, dev);
			ret |= platform_device_register(pdev);
		}
	}

	return ret;
}

device_initcall(mxs_device_init);
