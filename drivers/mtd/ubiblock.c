/*
 * Direct UBI block device access
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Based on mtdblock by:
 * (C) 2000-2003 Nicolas Pitre <nico@cam.org>
 * (C) 1999-2003 David Woodhouse <dwmw2@infradead.org>
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/mtd/ubi.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include "ubi/ubi.h"
#include "mtdcore.h"

static LIST_HEAD(ubiblk_devices);
static struct mutex ubiblk_devices_lock;

/**
 * ubiblk_dev - the structure representing translation layer
 *
 * @m: interface to mtd_blktrans
 * @ubi_num: UBI device number
 * @ubi_vol: UBI volume ID
 * @usecount: reference count
 *
 * @cache_mutex: protects access to cache_data
 * @cache_data: content of the cached LEB
 * @cache_offset: offset of cached data on UBI volume, in bytes
 * @cache_size: cache size in bytes, usually equal to LEB size
 */
struct ubiblk_dev {
	struct mtd_blktrans_dev *m;

	int 			ubi_num;
	int			ubi_vol;
	int			usecount;
	struct ubi_volume_desc *ubi;

	struct mutex 		cache_mutex;
	unsigned char 	       *cache_data;
	unsigned long 		cache_offset;
	unsigned int 		cache_size;

	enum {
		STATE_EMPTY,
		STATE_CLEAN,
		STATE_DIRTY
	} cache_state;

	struct list_head 	list;

	struct work_struct	unbind;
};

static int ubiblock_open(struct mtd_blktrans_dev *mbd);
static int ubiblock_release(struct mtd_blktrans_dev *mbd);
static int ubiblock_flush(struct mtd_blktrans_dev *mbd);
static int ubiblock_readsect(struct mtd_blktrans_dev *mbd,
			      unsigned long block, char *buf);
static int ubiblock_writesect(struct mtd_blktrans_dev *mbd,
			      unsigned long block, char *buf);
static int ubiblock_getgeo(struct mtd_blktrans_dev *mbd,
		struct hd_geometry *geo);
static void *ubiblk_add(int ubi_num, int ubi_vol_id);
static void *ubiblk_add_locked(int ubi_num, int ubi_vol_id);
static int ubiblk_del(struct ubiblk_dev *u);
static int ubiblk_del_locked(struct ubiblk_dev *u);

/*
 * These two routines are just to satify mtd_blkdev's requirements
 */
static void ubiblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	return;
}

static void ubiblock_remove_dev(struct mtd_blktrans_dev *mbd)
{
	return;
}

static struct mtd_blktrans_ops ubiblock_tr = {

	.name		= "ubiblk",
	.major		= 0,			/* assign dynamically  */
	.part_bits	= 3,			/* allow up to 8 parts */
	.blksize 	= 512,

	.open		= ubiblock_open,
	.release	= ubiblock_release,
	.flush		= ubiblock_flush,
	.readsect	= ubiblock_readsect,
	.writesect	= ubiblock_writesect,
	.getgeo		= ubiblock_getgeo,

	.add_mtd	= ubiblock_add_mtd,
	.remove_dev	= ubiblock_remove_dev,
	.owner		= THIS_MODULE,
};

static int ubiblock_getgeo(struct mtd_blktrans_dev *bdev,
		struct hd_geometry *geo)
{
	return -ENOTTY;
}

/**
 * ubiblk_write_cached_data - flush the cache to the UBI volume
 */
static int ubiblk_write_cached_data(struct ubiblk_dev *u)
{
	int ret;

	if (u->cache_state != STATE_DIRTY)
		return 0;

	pr_debug("%s: volume %d:%d, writing at %lx of size %x\n",
		__func__, u->ubi_num, u->ubi_vol,
		u->cache_offset, u->cache_size);

	ret = ubi_write(u->ubi, u->cache_offset / u->cache_size,
			u->cache_data, 0, u->cache_size);
	pr_debug("leb_write status %d\n", ret);

	u->cache_state = STATE_EMPTY;
	return ret;
}

/**
 * ubiblk_do_cached_write - cached write the data to the UBI volume
 *
 * @u: ubiblk_dev
 * @pos: offset on the block device
 * @len: buffer length
 * @buf: buffer itself
 *
 * if buffer contains one or more whole sectors (=LEBs), write them to the
 * volume. Otherwise, fill the cache that will be flushed later
 */
static int ubiblk_do_cached_write(struct ubiblk_dev *u, unsigned long pos,
			    int len, const char *buf)
{
	unsigned int sect_size = u->cache_size,
		size, offset;
	unsigned long sect_start;
	int ret = 0;
	int leb;

	pr_debug("%s: volume %d:%d, writing at pos %lx of size %x\n",
		__func__, u->ubi_num, u->ubi_vol, pos, len);

	while (len > 0) {
		leb = pos / sect_size;
		sect_start = leb * sect_size;
		offset = pos - sect_start;
		size = sect_size - offset;

		if (size > len)
			size = len;

		if (size == sect_size) {
			/*
			 * We are covering a whole sector.  Thus there is no
			 * need to bother with the cache while it may still be
			 * useful for other partial writes.
			 */
			ret = ubi_leb_change(u->ubi, leb,
					buf, size, UBI_UNKNOWN);
			if (ret)
				goto out;
		} else {
			/* Partial sector: need to use the cache */

			if (u->cache_state == STATE_DIRTY &&
			    u->cache_offset != sect_start) {
				ret = ubiblk_write_cached_data(u);
				if (ret)
					goto out;
			}

			if (u->cache_state == STATE_EMPTY ||
			    u->cache_offset != sect_start) {
				/* fill the cache with the current sector */
				u->cache_state = STATE_EMPTY;
				ret = ubi_leb_read(u->ubi, leb,
					u->cache_data, 0, u->cache_size, 0);
				if (ret)
					return ret;
				ret = ubi_leb_unmap(u->ubi, leb);
				if (ret)
					return ret;
				ret = ubi_leb_unmap(u->ubi, leb);
				if (ret)
					return ret;
				u->cache_offset = sect_start;
				u->cache_state = STATE_CLEAN;
			}

			/* write data to our local cache */
			memcpy(u->cache_data + offset, buf, size);
			u->cache_state = STATE_DIRTY;
		}

		buf += size;
		pos += size;
		len -= size;
	}

out:
	return ret;
}

/**
 * ubiblk_do_cached_read - cached read the data from ubi volume
 *
 * @u: ubiblk_dev
 * @pos: offset on the block device
 * @len: buffer length
 * @buf: preallocated buffer
 *
 * Cached LEB will be used if possible; otherwise data will be using
 * ubi_leb_read
 */
static int ubiblk_do_cached_read(struct ubiblk_dev *u, unsigned long pos,
			   int len, char *buf)
{
	unsigned int sect_size = u->cache_size;
	int err = 0;
	unsigned long sect_start;
	unsigned offset, size;
	int leb;

	pr_debug("%s: read at 0x%lx, size 0x%x\n",
			__func__, pos, len);

	while (len > 0) {

		leb = pos/sect_size;
		sect_start = leb*sect_size;
		offset = pos - sect_start;
		size = sect_size - offset;

		if (size > len)
			size = len;

		/*
		 * Check if the requested data is already cached
		 * Read the requested amount of data from our internal
		 * cache if it contains what we want, otherwise we read
		 * the data directly from flash.
		 */
		if (u->cache_state != STATE_EMPTY &&
		    u->cache_offset == sect_start) {
			pr_debug("%s: cached, returning back from cache\n",
					__func__);
			memcpy(buf, u->cache_data + offset, size);
			err = 0;
		} else {
			pr_debug("%s: pos = %ld, reading leb = %d\n",
					__func__, pos, leb);
			err = ubi_leb_read(u->ubi, leb, buf, offset, size, 0);
			if (err)
				goto out;
		}

		buf += size;
		pos += size;
		len -= size;
	}

out:
	return err;
}

static struct ubiblk_dev *ubiblk_find_by_ptr(struct mtd_blktrans_dev *m)
{
	struct ubiblk_dev *pos;

	mutex_lock(&ubiblk_devices_lock);
	list_for_each_entry(pos, &ubiblk_devices, list)
		if (pos->m == m) {
			mutex_unlock(&ubiblk_devices_lock);
			return pos;
		}
	mutex_unlock(&ubiblk_devices_lock);
	BUG();
	return NULL;
}

/**
 * ubiblock_writesect - write the sector
 *
 * Allocate the cache, if necessary and perform actual write using
 * ubiblk_do_cached_write
 */
static int ubiblock_writesect(struct mtd_blktrans_dev *mbd,
			      unsigned long block, char *buf)
{
	struct ubiblk_dev *u = ubiblk_find_by_ptr(mbd);

	if (unlikely(!u->cache_data)) {
		u->cache_data = vmalloc(u->cache_size);
		if (!u->cache_data)
			return -EAGAIN;
	}
	return ubiblk_do_cached_write(u, block<<9, 512, buf);
}

/**
 * ubiblk_readsect - read the sector
 *
 * Allocate the cache, if necessary, and perform actual read using
 * ubiblk_do_cached_read
 */
static int ubiblock_readsect(struct mtd_blktrans_dev *mbd,
			      unsigned long block, char *buf)
{
	struct ubiblk_dev *u = ubiblk_find_by_ptr(mbd);

	if (unlikely(!u->cache_data)) {
		u->cache_data = vmalloc(u->cache_size);
		if (!u->cache_data)
			return -EAGAIN;
	}
	return ubiblk_do_cached_read(u, block<<9, 512, buf);
}

static int ubiblk_flush_locked(struct ubiblk_dev *u)
{
	ubiblk_write_cached_data(u);
	ubi_sync(u->ubi_num);
	return 0;
}

static int ubiblock_flush(struct mtd_blktrans_dev *mbd)
{
	struct ubiblk_dev *u = ubiblk_find_by_ptr(mbd);

	mutex_lock(&u->cache_mutex);
	ubiblk_flush_locked(u);
	mutex_unlock(&u->cache_mutex);
	return 0;
}


static int ubiblock_open(struct mtd_blktrans_dev *mbd)
{
	struct ubiblk_dev *u = ubiblk_find_by_ptr(mbd);

	if (u->usecount == 0) {
		u->ubi = ubi_open_volume(u->ubi_num, u->ubi_vol,
				UBI_READWRITE);
		if (IS_ERR(u->ubi))
			return PTR_ERR(u->ubi);
	}
	u->usecount++;
	return 0;
}

static int ubiblock_release(struct mtd_blktrans_dev *mbd)
{
	struct ubiblk_dev *u = ubiblk_find_by_ptr(mbd);

	if (--u->usecount == 0) {
		mutex_lock(&u->cache_mutex);
		ubiblk_flush_locked(u);
		vfree(u->cache_data);
		u->cache_data = NULL;
		mutex_unlock(&u->cache_mutex);
		ubi_close_volume(u->ubi);
		u->ubi = NULL;
	}
	return 0;
}


/*
 * sysfs routines. The ubiblk creates two entries under /sys/block/ubiblkX:
 *  - volume, R/O, which is read like "ubi0:volume_name"
 *  - unbind, W/O; when user writes something here, the block device is
 *  removed
 *
 *  unbind schedules a work item to perform real unbind, because sysfs entry
 *  handler cannot delete itself :)
 */
ssize_t volume_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gendisk *gd = dev_to_disk(dev);
	struct mtd_blktrans_dev *m = gd->private_data;
	struct ubiblk_dev *u = ubiblk_find_by_ptr(m);

	return sprintf(buf, "%d:%d\n", u->ubi_num, u->ubi_vol);
}

static void ubiblk_unbind(struct work_struct *ws)
{
	struct ubiblk_dev *u = container_of(ws, struct ubiblk_dev, unbind);

	ubiblk_del(u);
}

ssize_t unbind_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct gendisk *gd = dev_to_disk(dev);
	struct mtd_blktrans_dev *m = gd->private_data;
	struct ubiblk_dev *u = ubiblk_find_by_ptr(m);

	INIT_WORK(&u->unbind, ubiblk_unbind);
	schedule_work(&u->unbind);
	return count;
}

DEVICE_ATTR(unbind, 0644, NULL, unbind_store);
DEVICE_ATTR(volume, 0644, volume_show, NULL);

static int ubiblk_sysfs(struct gendisk *hd, int add)
{
	int r = 0;

	if (add) {
		r = device_create_file(disk_to_dev(hd), &dev_attr_unbind);
		if (r < 0)
			goto out;
		r = device_create_file(disk_to_dev(hd), &dev_attr_volume);
		if (r < 0)
			goto out1;
		return 0;
	}

	device_remove_file(disk_to_dev(hd), &dev_attr_unbind);
out1:
	device_remove_file(disk_to_dev(hd), &dev_attr_volume);
out:
	return r;
}


/**
 * add the FTL by registering it with mtd_blkdevs
 */
static void *ubiblk_add(int ubi_num, int ubi_vol_id)
{
	void *p;

	mutex_lock(&mtd_table_mutex);
	p = ubiblk_add_locked(ubi_num, ubi_vol_id);
	mutex_unlock(&mtd_table_mutex);
	return p;
}

static void *ubiblk_add_locked(int ubi_num, int ubi_vol_id)
{
	struct ubiblk_dev *u = kzalloc(sizeof(*u), GFP_KERNEL);
	struct ubi_volume_info uvi;
	struct ubi_volume_desc *ubi;

	if (!u) {
		u = ERR_PTR(-ENOMEM);
		goto out;
	}
	u->m = kzalloc(sizeof(struct mtd_blktrans_dev), GFP_KERNEL);
	if (!u->m) {
		kfree(u);
		u = ERR_PTR(-ENOMEM);
		goto out;
	}

	ubi = ubi_open_volume(ubi_num, ubi_vol_id, UBI_READONLY);
	if (IS_ERR(ubi)) {
		pr_err("cannot open the volume\n");
		kfree(u->m);
		kfree(u);
		u = (void *)ubi;
		goto out;
	}

	ubi_get_volume_info(ubi, &uvi);

	pr_debug("adding volume of size %d (used_size %lld), LEB size %d\n",
		uvi.size, uvi.used_bytes, uvi.usable_leb_size);

	u->m->mtd = ubi->vol->ubi->mtd;
	u->m->devnum = -1;
	u->m->size = uvi.used_bytes >> 9;
	u->m->tr = &ubiblock_tr;

	ubi_close_volume(ubi);

	u->ubi_num = ubi_num;
	u->ubi_vol = ubi_vol_id;

	mutex_init(&u->cache_mutex);
	u->cache_state = STATE_EMPTY;
	u->cache_size = uvi.usable_leb_size;
	u->cache_data = NULL;
	u->usecount = 0;
	INIT_LIST_HEAD(&u->list);

	mutex_lock(&ubiblk_devices_lock);
	list_add_tail(&u->list, &ubiblk_devices);
	mutex_unlock(&ubiblk_devices_lock);
	add_mtd_blktrans_dev(u->m);
	ubiblk_sysfs(u->m->disk, true);
out:
	return u;
}

static int ubiblk_del(struct ubiblk_dev *u)
{
	int r;
	mutex_lock(&mtd_table_mutex);
	r = ubiblk_del_locked(u);
	mutex_unlock(&mtd_table_mutex);
	return r;
}

static int ubiblk_del_locked(struct ubiblk_dev *u)
{
	if (u->usecount != 0)
		return -EBUSY;
	ubiblk_sysfs(u->m->disk, false);
	del_mtd_blktrans_dev(u->m);

	mutex_lock(&ubiblk_devices_lock);
	list_del(&u->list);
	mutex_unlock(&ubiblk_devices_lock);

	BUG_ON(u->cache_data != NULL); /* who did not free the cache ?! */
	kfree(u);
	return 0;
}

static struct ubiblk_dev *ubiblk_find(int num, int vol)
{
	struct ubiblk_dev *pos;

	mutex_lock(&ubiblk_devices_lock);
	list_for_each_entry(pos, &ubiblk_devices, list)
		if (pos->ubi_num == num && pos->ubi_vol == vol) {
			mutex_unlock(&ubiblk_devices_lock);
			return pos;
		}
	mutex_unlock(&ubiblk_devices_lock);
	return NULL;
}

static int ubiblock_notification(struct notifier_block *blk,
		unsigned long type, void *v)
{
	struct ubi_notification *nt = v;
	struct ubiblk_dev *u;

	switch (type) {
	case UBI_VOLUME_ADDED:
		ubiblk_add(nt->vi.ubi_num, nt->vi.vol_id);
		break;
	case UBI_VOLUME_REMOVED:
		u = ubiblk_find(nt->vi.ubi_num, nt->vi.vol_id);
		if (u)
			ubiblk_del(u);
		break;
	case UBI_VOLUME_RENAMED:
	case UBI_VOLUME_RESIZED:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block ubiblock_nb = {
	.notifier_call = ubiblock_notification,
};

static int __init ubiblock_init(void)
{
	int r;

	mutex_init(&ubiblk_devices_lock);
	r = register_mtd_blktrans(&ubiblock_tr);
	if (r)
		goto out;
	r = ubi_register_volume_notifier(&ubiblock_nb, 0);
	if (r)
		goto out_unreg;
	return 0;

out_unreg:
	deregister_mtd_blktrans(&ubiblock_tr);
out:
	return 0;
}

static void __exit ubiblock_exit(void)
{
	ubi_unregister_volume_notifier(&ubiblock_nb);
	deregister_mtd_blktrans(&ubiblock_tr);
}

module_init(ubiblock_init);
module_exit(ubiblock_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Caching block device emulation access to UBI devices");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
