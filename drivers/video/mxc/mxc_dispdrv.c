/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_dispdrv.c
 * @brief mxc display driver framework.
 *
 * A display device driver could call mxc_dispdrv_register(drv) in its dev_probe() function.
 * Move all dev_probe() things into mxc_dispdrv_driver->init(), init() function should init
 * and feedback setting;
 * Move all dev_remove() things into mxc_dispdrv_driver->deinit();
 * Move all dev_suspend() things into fb_notifier for SUSPEND, if there is;
 * Move all dev_resume() things into fb_notifier for RESUME, if there is;
 *
 * ipuv3 fb driver could call mxc_dispdrv_init(setting) before a fb need be added, with fbi param
 * passing by setting, after mxc_dispdrv_init() return, FB driver should get the basic setting
 * about fbi info and ipuv3-hw (ipu_id and disp_id).
 *
 * @ingroup Framebuffer
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/string.h>
#include "mxc_dispdrv.h"

static LIST_HEAD(dispdrv_list);
static DEFINE_MUTEX(dispdrv_lock);

struct mxc_dispdrv_entry {
	const char *name;
	struct list_head list;
	int (*init) (struct mxc_dispdrv_entry *);
	void (*deinit) (struct mxc_dispdrv_entry *);
	bool active;
	struct mxc_dispdrv_setting setting;
	void *priv;
};

struct mxc_dispdrv_entry *mxc_dispdrv_register(struct mxc_dispdrv_driver *drv)
{
	struct mxc_dispdrv_entry *new;

	mutex_lock(&dispdrv_lock);

	new = kzalloc(sizeof(struct mxc_dispdrv_entry), GFP_KERNEL);
	if (!new) {
		mutex_unlock(&dispdrv_lock);
		return ERR_PTR(-ENOMEM);
	}

	new->name = drv->name;
	new->init = drv->init;
	new->deinit = drv->deinit;

	list_add_tail(&new->list, &dispdrv_list);
	mutex_unlock(&dispdrv_lock);

	return new;
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_register);

int mxc_dispdrv_unregister(struct mxc_dispdrv_entry *entry)
{
	if (entry) {
		mutex_lock(&dispdrv_lock);
		if (entry->active && entry->deinit)
			entry->deinit(entry);
		list_del(&entry->list);
		mutex_unlock(&dispdrv_lock);
		kfree(entry);
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_unregister);

int mxc_dispdrv_init(char *name, struct mxc_dispdrv_setting *setting)
{
	int ret = 0, found = 0;
	struct mxc_dispdrv_entry *disp;

	mutex_lock(&dispdrv_lock);
	list_for_each_entry(disp, &dispdrv_list, list) {
		if (!strcmp(disp->name, name)) {
			if (disp->init) {
				memcpy(&disp->setting, setting,
						sizeof(struct mxc_dispdrv_setting));
				ret = disp->init(disp);
				if (ret >= 0) {
					disp->active = true;
					/* setting may need fix-up */
					memcpy(setting, &disp->setting,
							sizeof(struct mxc_dispdrv_setting));
					found = 1;
					break;
				}
			}
		}
	}

	if (!found)
		ret = -EINVAL;

	mutex_unlock(&dispdrv_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_init);

int mxc_dispdrv_setdata(struct mxc_dispdrv_entry *entry, void *data)
{
	if (entry) {
		entry->priv = data;
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_setdata);

void *mxc_dispdrv_getdata(struct mxc_dispdrv_entry *entry)
{
	if (entry) {
		return entry->priv;
	} else
		return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_getdata);

struct mxc_dispdrv_setting
	*mxc_dispdrv_getsetting(struct mxc_dispdrv_entry *entry)
{
	if (entry) {
		return &entry->setting;
	} else
		return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(mxc_dispdrv_getsetting);
