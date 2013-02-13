/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/concat.h>
#include <linux/io.h>

#include <asm/outercache.h>

#define MAX_RESOURCES		4

struct imx6x_weimnor_info {
	struct mtd_info         *mtd[MAX_RESOURCES];
	struct mtd_info         *cmtd;
	struct map_info         map[MAX_RESOURCES];
	int                     nr_parts;
	struct mtd_partition    *parts;
};

static int imx6x_weimnor_remove(struct platform_device *dev)
{
	struct imx6x_weimnor_info *info;
	int i;

	info = platform_get_drvdata(dev);

	if (info == NULL)
		return 0;

	platform_set_drvdata(dev, NULL);

	if (info->cmtd) {
		mtd_device_unregister(info->cmtd);
		if (info->nr_parts)
			kfree(info->parts);
		if (info->cmtd != info->mtd[0])
			mtd_concat_destroy(info->cmtd);
	}

	for (i = 0; i < MAX_RESOURCES; i++) {
		if (info->mtd[i] != NULL)
			map_destroy(info->mtd[i]);
		if (info->map[i].cached)
			iounmap(info->map[i].cached);
	}

	kfree(info);

	return 0;
}

static void imx6x_set_vpp(struct map_info *map, int state)
{
	struct platform_device *pdev;
	struct physmap_flash_data *flash;

	pdev = (struct platform_device *)map->map_priv_1;
	flash = pdev->dev.platform_data;

	if (flash->set_vpp)
		flash->set_vpp(pdev, state);
}

#define CACHELINESIZE 32
static void imx6x_map_inval_cache(struct map_info *map, unsigned long from,
ssize_t len)
{
	unsigned long start;
	unsigned long end;
	unsigned long phys_start;
	unsigned long phys_end;

	if (from > map->size) {
		start = (unsigned long)map->cached + map->size;
		phys_start = (unsigned long)map->phys +  map->size;
	} else {
		start = (unsigned long)map->cached + from;
		phys_start = (unsigned long)map->phys + from;
	}

	if ((from + len)  > map->size) {
		end = start + map->size;
		phys_end = phys_start + map->size;
	} else {
		end = start + len;
		phys_end = phys_start + len;
	}

	start &= ~(CACHELINESIZE - 1);
	while (start < end) {
		/* invalidate D cache line */
		asm volatile ("mcr p15, 0, %0, c7, c6, 1" : : "r" (start));
		start += CACHELINESIZE;
	}
	outer_inv_range(phys_start, phys_end);
}

static const char *rom_probe_types[] = {
	"cfi_probe",
	"jedec_probe",
	"qinfo_probe",
	"map_rom",
	NULL};

static const char *part_probe_types[] = { "cmdlinepart", "RedBoot", "afs",
	NULL};

static int imx6x_weimnor_probe(struct platform_device *dev)
{
	struct physmap_flash_data *flash;
	struct imx6x_weimnor_info *info;
	const char **probe_type;
	int err = 0;
	int i;
	int devices_found = 0;

	flash = dev->dev.platform_data;
	if (flash == NULL)
		return -ENODEV;

	info = devm_kzalloc(&dev->dev, sizeof(struct imx6x_weimnor_info),
	       GFP_KERNEL);
	if (info == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	if (flash->init) {
		err = flash->init(dev);
		if (err)
			goto err_out;
	}

	platform_set_drvdata(dev, info);

	for (i = 0; i < dev->num_resources; i++) {
		printk(KERN_NOTICE
		       "imx6x-flash (physmap) platform "
		       "flash device: %.8llx at %.8llx\n",
		(unsigned long long)resource_size(&dev->resource[i]),
		(unsigned long long)dev->resource[i].start);

		if (!devm_request_mem_region(&dev->dev,
		dev->resource[i].start,
		resource_size(&dev->resource[i]),
		dev_name(&dev->dev))) {
			dev_err(&dev->dev, "Could not reserve memory region\n");
			err = -ENOMEM;
			goto err_out;
		}

		info->map[i].name = dev_name(&dev->dev);
		info->map[i].phys = dev->resource[i].start;
		info->map[i].size = resource_size(&dev->resource[i]);
		info->map[i].bankwidth = flash->width;
		info->map[i].set_vpp = imx6x_set_vpp;
		info->map[i].pfow_base = 0;
		info->map[i].map_priv_1 = (unsigned long)dev;
		info->map[i].virt = devm_ioremap(&dev->dev, info->map[i].phys,
				    info->map[i].size);
		if (info->map[i].virt == NULL) {
			dev_err(&dev->dev, "Failed to ioremap flash region\n");
			err = -EIO;
			goto err_out;
		}

		info->map[i].cached =
		ioremap_cached(info->map[i].phys, info->map[i].size);
		if (!info->map[i].cached)
			printk(KERN_WARNING "Failed to ioremap cached %s\n",
			info->map[i].name);

		info->map[i].inval_cache = imx6x_map_inval_cache;

		simple_map_init(&info->map[i]);
		probe_type = rom_probe_types;

		for (; info->mtd[i] == NULL && *probe_type != NULL;
		      probe_type++)
			info->mtd[i] = do_map_probe(*probe_type, &info->map[i]);

		if (info->mtd[i] == NULL) {
			dev_err(&dev->dev, "map_probe failed\n");
			err = -ENXIO;
			goto err_out;
		} else {
			devices_found++;
		}
		info->mtd[i]->owner = THIS_MODULE;
		info->mtd[i]->dev.parent = &dev->dev;
	}

	if (devices_found == 1) {
		info->cmtd = info->mtd[0];
	} else if (devices_found > 1) {
		/*
		 * We detected multiple devices. Concatenate them together.
		 */
		info->cmtd = mtd_concat_create(info->mtd, devices_found,
					       dev_name(&dev->dev));
		if (info->cmtd == NULL)
			err = -ENXIO;
	}
	if (err)
		goto err_out;

	err = parse_mtd_partitions(info->cmtd, part_probe_types,
	      &info->parts, 0);
	if (err > 0) {
		mtd_device_register(info->cmtd, info->parts, err);
		info->nr_parts = err;
		return 0;
	}

	if (flash->nr_parts) {
		printk(KERN_NOTICE "Using physmap partition information\n");
		mtd_device_register(info->cmtd, flash->parts,
		flash->nr_parts);
		return 0;
	}

	mtd_device_register(info->cmtd, NULL, 0);

	return 0;

err_out:
	imx6x_weimnor_remove(dev);
	return err;
}

#ifdef CONFIG_PM
static void imx6x_weimnor_shutdown(struct platform_device *dev)
{
	struct imx6x_weimnor_info *info = platform_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_RESOURCES && info->mtd[i]; i++)
		if (info->mtd[i]->suspend && info->mtd[i]->resume)
			if (info->mtd[i]->suspend(info->mtd[i]) == 0)
				info->mtd[i]->resume(info->mtd[i]);
}
#else
#define imx6x_weimnor_shutdown NULL
#endif

static struct platform_driver imx6x_weimnor_driver = {
	.probe          = imx6x_weimnor_probe,
	.remove         = __devexit_p(imx6x_weimnor_remove),
			  .shutdown       = imx6x_weimnor_shutdown,
	.driver         = {
		.name   = "imx6x-weimnor",
		.owner  = THIS_MODULE,
	},
};

static int __init imx6x_init(void)
{
	int err;
	err = platform_driver_register(&imx6x_weimnor_driver);
	return err;
}

static void __exit imx6x_exit(void)
{
	platform_driver_unregister(&imx6x_weimnor_driver);
}

module_init(imx6x_init);
module_exit(imx6x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oliver Brown <oliver.brown@freescale.com>");
MODULE_DESCRIPTION("MTD map driver for Freescale iMX");

