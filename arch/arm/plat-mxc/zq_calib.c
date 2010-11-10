/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <mach/hardware.h>

/* 10 secs, shall support changing this value in use-space later  */
#define ZQ_INTERVAL	(10 * 1000)

static void mxc_zq_main(void);

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

	/* set PU & PD value */
	data = (pd << 24) | (pu << 16);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	/*
	 * set PU+1 & PD+1 value
	 * when pu=0x1F, set (pu+1) = 0x1F
	 */
	data = ((pd + 1) << 8) | (pu + 1);
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

	/* set bit[4]=1, select PU/PD comparison */
	/* PD range: 0~0xE  (0xF has problem, drop it) */
	data = (pd << 24) | (pu << 16) | (1 << 4);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	/* when pu=0x1F, set (pu+1) = 0x1F */
	data = ((pd + 1) << 8) | (pu + 1);
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
	 * Compare PD from 0 to 0x0E  (please ignore 0x0F)
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
static void mxc_zq_hw_load(u32 pu, u32 pd)
{
	u32 data;
	/*
	 * The PU/PD values stored in register
	 * DATABAHN_REG_ZQ_SW_CFG1/2 would be loaded
	 */
	data = (pd << 24) | (pu << 16);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	data = ((pd + 1) << 8) | (pu + 1);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);

	/*
	 * bit[0]: enable hardware load
	 * bit[4]: trigger a hardware load.
	 *
	 * When the posedge of bit[4] detected, hardware trigger a load.
	 */
	__raw_writel(0x11, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/* clear bit[4] for next load */
	__raw_writel(0x01, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
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

	/*
	 * The PU/PD values stored in register
	 * DATABAHN_REG_ZQ_SW_CFG1/2 would be loaded.
	 * */
	data = (pd << 24) | (pu << 16);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG1);
	data = ((pd + 1) << 8) | (pu + 1);
	__raw_writel(data, databahn_base + DATABAHN_REG_ZQ_SW_CFG2);

	/*
	 * bit[21]: select software load
	 * bit[20]: enable software load
	 */
	__raw_writel(0x3 << 20, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
	/* clear for next load */
	__raw_writel(0x0, databahn_base + DATABAHN_REG_ZQ_HW_CFG);
}

/*!
 * MXC ZQ interface - PU/PD calib function
 * This function Do a complete PU/PD calib and loading process.
 */
static void mxc_zq_main(void)
{
	u32 pu, pd;

	spin_lock(&zq_lock);
	/* Search pu value start from 0 */
	pu = mxc_zq_pu_calib(0);
	/* Search pd value start from 0 */
	pd = mxc_zq_pd_calib(0, pu);
	printk("pu = %d, pd = %d\n", pu, pd);
	mxc_zq_hw_load(pu, pd);
	/* or do software load alternatively */
	/* zq_sw_load(pu, pd); */
	spin_unlock(&zq_lock);

	queue_delayed_work(zq_queue, &zq_work, msecs_to_jiffies(ZQ_INTERVAL));
}

static int __init mxc_zq_calib_init(void)
{
	zq_queue = create_singlethread_workqueue("zq_calib");;
	if (!zq_queue)
		return -ENOMEM;

	mxc_zq_main();

	return 0;
}

static void __exit mxc_zq_calib_exit(void)
{
	cancel_delayed_work(&zq_work);
	flush_workqueue(zq_queue);
	destroy_workqueue(zq_queue);
}

module_init(mxc_zq_calib_init);
module_exit(mxc_zq_calib_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC ZQ Calibration driver");
MODULE_LICENSE("GPL");
