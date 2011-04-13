/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

/* 10 secs by default, users can change it via sys */
static int interval = 10;

static struct device *zq_calib_dev;

static void mxc_zq_main(struct work_struct *dummy);

/* Use workqueue */
static struct workqueue_struct *zq_queue;
static DEFINE_SPINLOCK(zq_lock);
static DECLARE_DELAYED_WORK(zq_work, mxc_zq_main);

extern void __iomem *databahn_base;

#define DATABAHN_REG_ZQ_HW_CFG		DATABAHN_CTL_REG73
#define DATABAHN_REG_ZQ_SW_CFG1		DATABAHN_CTL_REG74
#define DATABAHN_REG_ZQ_SW_CFG2		DATABAHN_CTL_REG75
#define DATABAHN_REG_ZQ_STATUS		DATABAHN_CTL_REG83

/*!
 * MXC ZQ interface - Compare PU vs the External Resistor (240/300 ohm)
 *
 * @param pu	u32
 * @param pd	u32
 *
 * @return	Return compare result.
 */
static u32 mxc_zq_pu_compare(u32 pu, u32 pd)
{
	u32 data;
	u32 pu_m1, pd_m1;

	/*
	 * set PU-1 & PD-1 value
	 */
	pu_m1 = (pu <= 0) ? 0 : pu - 1;
	pd_m1 = (pd <= 0) ? 0 : pd - 1;
	data = ((pd_m1) << 24) | ((pu_m1) << 16);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	/*
	 * set PU & PD value
	 */
	data = (pd << 8) | pu;
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);
	/*
	 * Enable the ZQ comparator,
	 * need 300ns to complete a ZQ comparison
	 */
	__raw_writel(1 << 16, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/* TODO:  wait 300ns till comparator output stable */
	ndelay(300);
	/* read status bit[0] */
	data = __raw_readl(databahn_base + DATABAHN_REG_ZQ_STATUS);
	data &= 0x1;
	/* Disable the ZQ comparator to save power */
	__raw_writel(0, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	return data;
}

/*!
 * MXC ZQ interface - Compare PU vs PD
 *
 * @param pu	u32
 * @param pd	u32
 *
 * @return	Return compare result.
 */
static u32 mxc_zq_pd_compare(u32 pu, u32 pd)
{
	u32 data;
	u32 pu_m1, pd_m1;

	/* set bit[4]=1, select PU/PD comparison */
	/* PD range: 0~0xF */
	pu_m1 = (pu <= 0) ? 0 : pu - 1;
	pd_m1 = (pd <= 0) ? 0 : pd - 1;
	data = ((pd_m1) << 24) | ((pu_m1) << 16) | (1 << 4);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	data = (pd << 8) | pu;
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);
	/*
	 * Enable the ZQ comparator,
	 * need 300ns to complete a ZQ comparison
	 */
	__raw_writel(1 << 16, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/* TODO:  wait 300ns till comparator output stable */
	ndelay(300);
	/* read status bit[0] */
	data = __raw_readl(databahn_base + DATABAHN_REG_ZQ_STATUS);
	data &= 0x1;
	/* Disable the ZQ comparator to save power */
	__raw_writel(0, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	return data;
}

/*!
 * MXC ZQ interface - Do a full range search of PU to
 * 			match the external resistor
 *
 * @param start	u32
 *
 * @return	Return pu.
 */
static u32 mxc_zq_pu_calib(u32 start)
{
	u32 i;
	u32 data;
	u32 zq_pu_val = 0;

	/*
	 * Compare PU from 0 to 0x1F
	 * data is the result of the comparator
	 * the result sequence looks like:
	 * 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
	 * Take the First "1" the sequence for PU
	 */
	for (i = start; i < 32; ++i) {
		data = mxc_zq_pu_compare(i, 0);
		if (data) {
			zq_pu_val = i;
			break;
		}
	}

	return zq_pu_val;
}

/*!
 * MXC ZQ interface - Do a full range search of PD to match
 * 			the PU get from za_pu_calib()
 *
 * @param start	u32
 * @param pu	u32
 *
 * @return	Return pd.
 */
static s32 mxc_zq_pd_calib(u32 start, u32 pu)
{
	u32 i;
	u32 data;
	u32 zq_pd_val = 0;

	/*
	 * Compare PD from 0 to 0x0F
	 * data is the result of the comparator
	 * the result sequence looks like:
	 * 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0
	 * Take the Last "1" in the sequence for PD
	 */
	for (i = start; i < 15; ++i) {
		data = mxc_zq_pd_compare(pu, i);
		if (!data && (i > 0)) {
			zq_pd_val = i - 1;
			break;
		}
	}

	return zq_pd_val;
}

/*!
 * MXC ZQ interface - Load the PU/PD value to the ZQ buffers by hardware
 *
 * @param pu	u32
 * @param pd	u32
 */
static void mxc_zq_hw_load(u32 pu, u32 pd, u32 pu_pd_sel)
{
	u32 data;
	u32 pu_m1, pd_m1;

	pu_m1 = (pu <= 0) ? 0 : pu - 1;
	pd_m1 = (pd <= 0) ? 0 : pd - 1;

	/*
	 * The PU/PD values stored in register
	 * DATABAHN_REG_ZQ_SW_CFG1/2 would be loaded
	 */
	data = (pd << 8) | pu;
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);

	data = (pd_m1 << 24) | (pu_m1 << 16);  /* load PD */
	if (pu_pd_sel)
		data |= (1 << 4);  /* load PU */
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);

	/*
	 * bit[0]: enable hardware load
	 * bit[4]: trigger a hardware load.
	 *
	 * When the posedge of bit[4] detected, hardware trigger a load.
	 */
	__raw_writel(0x10011, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/*
	 * Clear the zq_hw_load bit for next loading
	 */
	__raw_writel(0x10001, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/*
	 * Delay at least 10us waiting an ddr auto-refresh occurs
	 * PU PD value are loaded on auto-refresh event
	 */
	udelay(10);
	/*
	 * Clear the calibration_en (bit[16]) to save power consumption
	 */
	__raw_writel(0x1, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
}

/*!
 * MXC ZQ interface - Load the PU/PD value to the ZQ buffers by software
 *
 * @param pu	u32
 * @param pd	u32
 */

static void mxc_zq_sw_load(u32 pu, u32 pd)
{
	u32 data;
	u32 pu_m1, pd_m1;

	/*
	 * The PU/PD values stored in register
	 * DATABAHN_REG_ZQ_SW_CFG1/2 would be loaded.
	 * */
	pu_m1 = (pu <= 0) ? 0 : pu - 1;
	pd_m1 = (pd <= 0) ? 0 : pd - 1;

	data = ((pd_m1) << 24) | ((pu_m1) << 16);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	data = (pd << 8) | pu;
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);

	/* Loading PU value, set pu_pd_sel=0 */
	__raw_writel((0x3 << 20) | (1 << 16),
			databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	__raw_writel(0x1 << 21,
			databahn_base + DATABAHN_REG_ZQ_HW_CFG);

	/* Loading PD value, set pu_pd_sel=1 */
	data |= (1 << 4);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);

	/*
	 * bit[21]: select software load
	 * bit[20]: enable software load
	 */
	__raw_writel((0x3 << 20) | (1 << 16),
			databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/* clear for next load */
	__raw_writel(0x1 << 21, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
}

/*
 * This function is for ZQ pu calibration
 */
static u32 pu_calib_based_on_pd(u32 start, u32 pd)
{
	u32 i;
	u32 data;
	u32 zq_pu_val = 0;

	/*
	 * compare PU from 0 to 0x1F
	 * data is the result of the comparator
	 * the result sequence looks like:
	 * 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
	 * Pleae take the First "1" in the sequence for PU
	 */
	for (i = start; i < 32; i++) {
		data = mxc_zq_pd_compare(i, pd);
		if (data) {
		    zq_pu_val = i;
		    break;
		}
	}

	return zq_pu_val;
}

/*!
 * MXC ZQ interface - PU/PD calib function
 * This function Do a complete PU/PD calib and loading process.
 */
static void mxc_zq_main(struct work_struct *dummy)
{
	u32 pu, pd;

	spin_lock(&zq_lock);
	/* Search pu value start from 0 */
	pu = mxc_zq_pu_calib(1);
	/* Search pd value based on pu */
	pd = mxc_zq_pd_calib(1, pu) + 3;
	pu = pu_calib_based_on_pd(1, pd);

	dev_dbg(zq_calib_dev, "za_calib: pu = %d, pd = %d\n", pu, pd);
	mxc_zq_hw_load(pu, pd, 1);	/* Load Pu */
	mxc_zq_hw_load(pu, pd, 0);	/* Load Pd */
	/* or do software load alternatively */
	/* zq_sw_load(pu, pd); */
	spin_unlock(&zq_lock);

	queue_delayed_work(zq_queue, &zq_work,
			msecs_to_jiffies(interval * 1000));
}

static ssize_t interval_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", interval);
}

static ssize_t interval_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int val;
	if (sscanf(buf, "%d", &val) > 0) {
		interval = val;
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(interval, 0644, interval_show,
		   interval_store);

static int __devinit mxc_zq_calib_probe(struct platform_device *pdev)
{
	int err = 0;

	zq_calib_dev = &pdev->dev;
	zq_queue = create_singlethread_workqueue("zq_calib");;
	if (!zq_queue)
		return -ENOMEM;

	err = device_create_file(&pdev->dev, &dev_attr_interval);
	if (err) {
		dev_err(&pdev->dev,
			"Unable to create file from interval\n");
		destroy_workqueue(zq_queue);
		return err;
	}

	mxc_zq_main(NULL);

	return 0;
}

static int __devexit mxc_zq_calib_remove(struct platform_device *pdev)
{
	cancel_delayed_work(&zq_work);
	flush_workqueue(zq_queue);
	destroy_workqueue(zq_queue);
	device_remove_file(&pdev->dev, &dev_attr_interval);
	return 0;
}

#ifdef CONFIG_PM
static int zq_calib_suspend(struct platform_device *pdev, pm_message_t state)
{
	flush_delayed_work(&zq_work);

	return 0;
}

static int zq_calib_resume(struct platform_device *pdev)
{
	mxc_zq_main(NULL);

	return 0;
}
#else
#define	zq_calib_suspend	NULL
#define	zq_calib_resume		NULL
#endif

static struct platform_driver mxc_zq_calib_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mxc_zq_calib",
	},
	.probe = mxc_zq_calib_probe,
	.remove =  __exit_p(mxc_zq_calib_remove),
	.suspend = zq_calib_suspend,
	.resume = zq_calib_resume,
};

static int __init mxc_zq_calib_init(void)
{
	return platform_driver_register(&mxc_zq_calib_driver);
}

static void __exit mxc_zq_calib_exit(void)
{
	platform_driver_unregister(&mxc_zq_calib_driver);
}

module_init(mxc_zq_calib_init);
module_exit(mxc_zq_calib_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC ZQ Calibration driver");
MODULE_LICENSE("GPL");
