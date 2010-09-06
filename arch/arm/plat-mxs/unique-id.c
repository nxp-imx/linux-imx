/*
 * Unique ID manipulation sysfs access generic functions
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include <mach/unique-id.h>

static int unlock;
static spinlock_t u_lock;
static const unsigned long UID_AUTOLOCK_TIMEOUT = HZ * 60 * 3;
static struct timer_list u_timer;

static void uid_timer_autolock(unsigned long param)
{
	struct timer_list *tmr = (struct timer_list *)param;

	if (spin_trylock(&u_lock)) {
		if (unlock)
			pr_debug("%s: locked down.\n", __func__);
		unlock = 0;
		spin_unlock(&u_lock);
	}
	mod_timer(tmr, jiffies + UID_AUTOLOCK_TIMEOUT);
}

static LIST_HEAD(uid_provider_list);

struct uid_provider {
	struct kobject 	*kobj;
	struct list_head list;
	struct uid_ops  *ops;
	void *context;
};

static struct uid_provider *uid_provider_find(const char *name);

#define UID_FWD_SYSFS_FILE(var, file, param) \
	static ssize_t var##_show(struct kobject *kobj, 		\
			struct kobj_attribute *attr, char *buf) 	\
	{								\
		struct uid_provider *p = 				\
			uid_provider_find(kobject_name(kobj));		\
		ssize_t r;						\
		BUG_ON(p == NULL);					\
		r = (p->ops && p->ops->file##_show) ? 			\
			p->ops->file##_show(p->context, buf, param) : 0;\
		return r;						\
	}								\
									\
	static ssize_t var##_store(struct kobject *kobj, 		\
		struct kobj_attribute *attr, const char *buf, 		\
			size_t count)					\
	{								\
		struct uid_provider *p =				\
			uid_provider_find(kobject_name(kobj));		\
		ssize_t r;						\
		int ul;							\
		BUG_ON(p == NULL);					\
		spin_lock(&u_lock);					\
		ul = unlock;						\
		spin_unlock(&u_lock);					\
		if (ul) 						\
			r =  (p->ops && p->ops->file##_store) ? 	\
		    p->ops->file##_store(p->context, buf, count, param) \
				: count;				\
		else							\
			r = -EACCES;					\
		return r;						\
	}

struct kobject *uid_kobj;

#define UID_ATTR(_name, _varname) \
	static struct kobj_attribute _varname##_attr = \
		__ATTR(_name, 0644, _varname##_show, _varname##_store)

UID_FWD_SYSFS_FILE(id, id, 1);
UID_FWD_SYSFS_FILE(id_bin, id, 0);
UID_ATTR(id, id);
UID_ATTR(id.bin, id_bin);

static struct attribute *uid_attrs[] = {
	&id_attr.attr,
	&id_bin_attr.attr,
	NULL
};

static struct attribute_group uid_attr_group = {
	.attrs = uid_attrs,
};

struct kobject *uid_provider_init(const char *name,
			struct uid_ops *ops, void *context)
{
	struct uid_provider *new;
	int err;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		err = -ENOMEM;
		goto out;
	}

	new->kobj = kobject_create_and_add(name, uid_kobj);
	if (!new->kobj) {
		err = -ENOMEM;
		goto out;
	}
	new->ops = ops;
	new->context = context;

	err = sysfs_create_group(new->kobj, &uid_attr_group);
	if (err)
		goto out2;

	list_add_tail(&new->list, &uid_provider_list);
	return new->kobj;
out2:
	kobject_del(new->kobj);
out:
	kfree(new);
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(uid_provider_init);

static struct uid_provider *uid_provider_find(const char *name)
{
	struct uid_provider *p;

	list_for_each_entry(p, &uid_provider_list, list) {
		if (strcmp(kobject_name(p->kobj), name) == 0)
			return p;
	}
	return NULL;
}

void uid_provider_remove(const char *name)
{
	struct uid_provider *p;

	p = uid_provider_find(name);
	if (!p)
		return;
	kobject_del(p->kobj);
	list_del(&p->list);
	kfree(p);
}
EXPORT_SYMBOL_GPL(uid_provider_remove);

static int uid_sysfs_init(void)
{
	int error;

	uid_kobj = kobject_create_and_add("uid", NULL);
	if (!uid_kobj) {
		error = -ENOMEM;
		goto out1;
	}

	spin_lock_init(&u_lock);
	setup_timer(&u_timer, uid_timer_autolock, (unsigned long)&u_timer);

	/* try to lock each 3 minutes */
	mod_timer(&u_timer, jiffies + UID_AUTOLOCK_TIMEOUT);
	return 0;

out1:
	printk(KERN_ERR"%s failed, error %d.", __func__, error);
	return error;
}

module_param(unlock, int, 0600)
core_initcall(uid_sysfs_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_DESCRIPTION("Unique ID simple framework");
