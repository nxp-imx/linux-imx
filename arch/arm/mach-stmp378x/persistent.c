/*
 * Freescale STMP378X Persistent bits manipulation driver
 *
 * Author: Pantelis Antoniou <pantelis@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>

#include <mach/regs-rtc.h>

struct stmp3xxx_persistent_data {
	struct device *dev;
	struct stmp3xxx_platform_persistent_data *pdata;
	int count;
	struct attribute_group attr_group;
	/* attribute ** follow */
	/* device_attribute follow */
};

#define pd_attribute_ptr(x) \
	((struct attribute **)((x) + 1))
#define pd_device_attribute_ptr(x) \
	((struct device_attribute *)(pd_attribute_ptr(x) + (x)->count + 1))

static inline u32 persistent_reg_read(int reg)
{
	u32 msk;

	/* wait for stable value */
	msk = BF(0x01 << reg, RTC_STAT_STALE_REGS);
	while (__raw_readl(REGS_RTC_BASE + HW_RTC_STAT) & msk)
		cpu_relax();

	return __raw_readl(REGS_RTC_BASE + 0x60 + (reg * 0x10));
}

static inline void persistent_reg_wait_settle(int reg)
{
	u32 msk;

	/* wait until the change is propagated */
	msk = BF(0x01 << reg, RTC_STAT_NEW_REGS);
	while (__raw_readl(REGS_RTC_BASE + HW_RTC_STAT) & msk)
		cpu_relax();
}

static inline void persistent_reg_write(u32 val, int reg)
{
	__raw_writel(val, REGS_RTC_BASE + 0x60 + (reg * 0x10));
	persistent_reg_wait_settle(reg);
}

static inline void persistent_reg_set(u32 val, int reg)
{
	__raw_writel(val, REGS_RTC_BASE + 0x60 + (reg * 0x10) + 0x4);
	persistent_reg_wait_settle(reg);
}

static inline void persistent_reg_clr(u32 val, int reg)
{
	__raw_writel(val, REGS_RTC_BASE + 0x60 + (reg * 0x10) + 0x8);
	persistent_reg_wait_settle(reg);
}

static inline void persistent_reg_tog(u32 val, int reg)
{
	__raw_writel(val, REGS_RTC_BASE + 0x60 + (reg * 0x10) + 0xc);
	persistent_reg_wait_settle(reg);
}

static ssize_t
persistent_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct stmp3xxx_persistent_data *pd = platform_get_drvdata(pdev);
	struct device_attribute *devattr = pd_device_attribute_ptr(pd);
	const struct stmp3xxx_persistent_bit_config *pb;
	int idx;
	u32 val;

	idx = attr - devattr;
	if ((unsigned int)idx >= pd->count)
		return -EINVAL;

	pb = &pd->pdata->bit_config_tab[idx];

	/* read value and shift */
	val = persistent_reg_read(pb->reg);
	val >>= pb->start;
	val &= (1 << pb->width) - 1;

	return sprintf(buf, "%u\n", val);
}

static ssize_t
persistent_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct stmp3xxx_persistent_data *pd = platform_get_drvdata(pdev);
	struct device_attribute *devattr = pd_device_attribute_ptr(pd);
	const struct stmp3xxx_persistent_bit_config *pb;
	int idx, r;
	unsigned long val, msk;

	idx = attr - devattr;
	if ((unsigned int)idx >= pd->count)
		return -EINVAL;

	pb = &pd->pdata->bit_config_tab[idx];

	/* get value to write */
	r = strict_strtoul(buf, 10, &val);
	if (r != 0)
		return r;

	/* verify it fits */
	if ((unsigned int)val > (1 << pb->width) - 1)
		return -EINVAL;

	/* lockless update, first clear the area */
	msk = ((1 << pb->width) - 1) << pb->start;
	persistent_reg_clr(msk, pb->reg);

	/* shift into position */
	val <<= pb->start;
	persistent_reg_set(val, pb->reg);

	return count;
}


static int __devinit stmp3xxx_persistent_probe(struct platform_device *pdev)
{
	struct stmp3xxx_persistent_data *pd;
	struct stmp3xxx_platform_persistent_data *pdata;
	const struct stmp3xxx_persistent_bit_config *pb;
	struct attribute **attr;
	struct device_attribute *devattr;
	int i, cnt, size;
	int err;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	cnt = pdata->bit_config_cnt;
	size = sizeof(*pd) +
		 (cnt + 1) * sizeof(struct atrribute *) +
		 cnt * sizeof(struct device_attribute);
	pd = kzalloc(size, GFP_KERNEL);
	if (pd == NULL)
		return -ENOMEM;
	pd->dev = &pdev->dev;
	pd->pdata = pdata;
	platform_set_drvdata(pdev, pd);
	pd->count = cnt;
	attr = pd_attribute_ptr(pd);
	devattr = pd_device_attribute_ptr(pd);

	/* build the attributes structures */
	pd->attr_group.attrs = attr;
	pb = pdata->bit_config_tab;
	for (i = 0; i < cnt; i++) {
		devattr[i].attr.name = pb[i].name;
		devattr[i].attr.mode = S_IWUSR | S_IRUGO;
		devattr[i].show = persistent_show;
		devattr[i].store = persistent_store;
		attr[i] = &devattr[i].attr;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &pd->attr_group);
	if (err != 0) {
		platform_set_drvdata(pdev, NULL);
		kfree(pd);
		return err;
	}

	return 0;
}

static int stmp3xxx_persistent_remove(struct platform_device *pdev)
{
	struct stmp3xxx_persistent_data *pd;

	pd = platform_get_drvdata(pdev);
	sysfs_remove_group(&pdev->dev.kobj, &pd->attr_group);
	platform_set_drvdata(pdev, NULL);
	kfree(pd);

	return 0;
}

#ifdef CONFIG_PM
static int
stmp3xxx_persistent_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int stmp3xxx_persistent_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define stmp3xxx_persistent_suspend	NULL
#define	stmp3xxx_persistent_resume	NULL
#endif

static struct platform_driver stmp3xxx_persistent_driver = {
	.probe		= stmp3xxx_persistent_probe,
	.remove		= stmp3xxx_persistent_remove,
	.suspend	= stmp3xxx_persistent_suspend,
	.resume		= stmp3xxx_persistent_resume,
	.driver		= {
		.name   = "stmp3xxx-persistent",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_persistent_init(void)
{
	return platform_driver_register(&stmp3xxx_persistent_driver);
}

static void __exit stmp3xxx_persistent_exit(void)
{
	platform_driver_unregister(&stmp3xxx_persistent_driver);
}

MODULE_AUTHOR("Pantelis Antoniou <pantelis@embeddedalley.com>");
MODULE_DESCRIPTION("Persistent bits user-access driver");
MODULE_LICENSE("GPL");

module_init(stmp3xxx_persistent_init);
module_exit(stmp3xxx_persistent_exit);
