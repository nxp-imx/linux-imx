/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file dptc.c
 *
 * @brief Driver for the Freescale Semiconductor MXC DPTC module.
 *
 * The DPTC driver is designed to control the MXC DPTC hardware.
 * hardware. Upon initialization, the DPTC driver initializes the DPTC hardware
 * sets up driver nodes attaches to the DPTC interrupt and initializes internal
 * data structures. When the DPTC interrupt occurs the driver checks the cause
 * of the interrupt (lower frequency, increase frequency or emergency) and changes
 * the CPU voltage according to translation table that is loaded into the driver.
 * The driver read method is used to read the log buffer.
 * Driver ioctls are used to change driver parameters and enable/disable the
 * DVFS operation.
 *
 * @ingroup PM
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/clock.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/mxc_dptc.h>

/*
 * Convenience conversion.
 * Here atm, maybe there is somewhere better for this.
 */
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

enum {
	DPTC_PTVAI_NOCHANGE = 0x0,
	DPTC_PTVAI_DECREASE,
	DPTC_PTVAI_INCREASE,
	DPTC_PTVAI_EMERG,
};

struct device *dev_data0;
struct device *dev_data1;
struct dptc_device *dptc_device_data;

/*!
 * In case the MXC device has multiple DPTC modules, this structure is used to
 * store information specific to each DPTC module.
 */
struct dptc_device {
	/* DPTC delayed work */
	struct delayed_work dptc_work;
	/* DPTC spinlock */
	spinlock_t lock;
	/* DPTC regulator */
	struct regulator *dptc_reg;
	/* DPTC clock */
	struct clk *dptc_clk;
	/* DPTC is active flag */
	int dptc_is_active;
	/* turbo mode active flag */
	int turbo_mode_active;
	/* DPTC current working point */
	int curr_wp;
	/* DPTC vai bits */
	u32 ptvai;
	/* The base address of the DPTC */
	void __iomem *membase;
	/* The interrupt number used by the DPTC device */
	int irq;
	/* DPTC platform data pointer */
	struct mxc_dptc_data *dptc_platform_data;
};

static void update_dptc_wp(struct dptc_device *drv_data, u32 wp)
{
	struct mxc_dptc_data *dptc_data = drv_data->dptc_platform_data;
	int voltage_uV;
	int ret = 0;

	voltage_uV = dptc_data->dptc_wp_allfreq[wp].voltage * 1000;

	__raw_writel(dptc_data->dptc_wp_allfreq[wp].dcvr0,
		     drv_data->membase + dptc_data->dcvr0_reg_addr);
	__raw_writel(dptc_data->dptc_wp_allfreq[wp].dcvr1,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0x4);
	__raw_writel(dptc_data->dptc_wp_allfreq[wp].dcvr2,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0x8);
	__raw_writel(dptc_data->dptc_wp_allfreq[wp].dcvr3,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0xC);

	/* Set the voltage */
	ret = regulator_set_voltage(drv_data->dptc_reg, voltage_uV, voltage_uV);
	if (ret < 0)
		printk(KERN_DEBUG "COULD NOT SET VOLTAGE!!!!!\n");

	pr_debug("dcvr0-3: 0x%x, 0x%x, 0x%x, 0x%x; vol: %d\n",
		 dptc_data->dptc_wp_allfreq[wp].dcvr0,
		 dptc_data->dptc_wp_allfreq[wp].dcvr1,
		 dptc_data->dptc_wp_allfreq[wp].dcvr2,
		 dptc_data->dptc_wp_allfreq[wp].dcvr3,
		 dptc_data->dptc_wp_allfreq[wp].voltage);
}

static irqreturn_t dptc_irq(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct dptc_device *drv_data = dev->driver_data;
	struct mxc_dptc_data *dptc_data = dev->platform_data;
	u32 dptccr = __raw_readl(drv_data->membase
				 + dptc_data->dptccr_reg_addr);
	u32 gpc_cntr = __raw_readl(dptc_data->gpc_cntr_reg_addr);

	gpc_cntr = (gpc_cntr & dptc_data->dptccr);

	if (gpc_cntr) {
		drv_data->ptvai =
		    (dptccr & dptc_data->vai_mask) >> dptc_data->vai_offset;
		pr_debug("dptc_irq: vai = 0x%x (0x%x)!!!!!!!\n",
			 drv_data->ptvai, dptccr);

		/* disable DPTC and mask its interrupt */
		dptccr = (dptccr & ~(dptc_data->dptc_enable_bit)) |
		    (dptc_data->irq_mask);
		dptccr = (dptccr & ~(dptc_data->dptc_nvcr_bit));
		__raw_writel(dptccr, drv_data->membase
			     + dptc_data->dptccr_reg_addr);

		if (drv_data->turbo_mode_active == 1)
			schedule_delayed_work(&drv_data->dptc_work, 0);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void dptc_workqueue_handler(struct work_struct *work1)
{
	struct delayed_work *dptc_work_tmp =
	    container_of(work1, struct delayed_work, work);
	struct dptc_device *drv_data =
	    container_of(dptc_work_tmp, struct dptc_device, dptc_work);
	struct mxc_dptc_data *dptc_data = drv_data->dptc_platform_data;
	u32 dptccr = __raw_readl(drv_data->membase
				 + dptc_data->dptccr_reg_addr);

	switch (drv_data->ptvai) {
	case DPTC_PTVAI_DECREASE:
		drv_data->curr_wp++;
		break;
	case DPTC_PTVAI_INCREASE:
	case DPTC_PTVAI_EMERG:
		drv_data->curr_wp--;
		if (drv_data->curr_wp < 0) {
			/* already max voltage */
			drv_data->curr_wp = 0;
			printk(KERN_WARNING "dptc: already maximum voltage\n");
		}
		break;

		/* Unknown interrupt cause */
	default:
		BUG();
	}

	if (drv_data->curr_wp > dptc_data->dptc_wp_supported
	    || drv_data->curr_wp < 0) {
		panic("Can't support this working point: %d\n",
		      drv_data->curr_wp);
	}
	update_dptc_wp(drv_data, drv_data->curr_wp);

	/* Enable DPTC and unmask its interrupt */
	dptccr = (dptccr & ~(dptc_data->irq_mask)) |
	    dptc_data->dptc_nvcr_bit | dptc_data->dptc_enable_bit;
	__raw_writel(dptccr, drv_data->membase + dptc_data->dptccr_reg_addr);
}

/* Start DPTC unconditionally */
static int start_dptc(struct device *dev)
{
	struct mxc_dptc_data *dptc_data = dev->platform_data;
	struct dptc_device *drv_data = dev->driver_data;
	u32 dptccr;
	unsigned long flags;
	unsigned long clk_rate;
	int voltage_uV;

	/* Get the voltage */
	voltage_uV = regulator_get_voltage(drv_data->dptc_reg);
	drv_data->curr_wp =
	    (dptc_data->dptc_wp_allfreq[0].voltage - (voltage_uV / 1000)) / 25;

	update_dptc_wp(drv_data, drv_data->curr_wp);

	/* Set the voltage */
	spin_lock_irqsave(&drv_data->lock, flags);

	clk_rate = clk_get_rate(drv_data->dptc_clk);

	if (clk_rate < dptc_data->clk_max_val)
		goto err;

	if (dptc_data->gpc_irq_bit != 0x0) {
		/* Enable ARM domain frequency and/or voltage update needed
		   and enable ARM IRQ */
		__raw_writel(dptc_data->gpc_irq_bit | dptc_data->gpc_adu,
			     dptc_data->gpc_cntr_reg_addr);
	}

	dptccr = __raw_readl(drv_data->membase + dptc_data->dptccr_reg_addr);

	/* Enable DPTC and unmask its interrupt */
	dptccr = ((dptccr & ~(dptc_data->irq_mask)) | dptc_data->enable_config);

	__raw_writel(dptccr, drv_data->membase + dptc_data->dptccr_reg_addr);

	spin_unlock_irqrestore(&drv_data->lock, flags);

	drv_data->dptc_is_active = 1;
	drv_data->turbo_mode_active = 1;

	pr_info("DPTC has been started \n");

	return 0;

err:
	spin_unlock_irqrestore(&drv_data->lock, flags);
	pr_info("DPTC is not enabled\n");
	return -1;
}

/* Stop DPTC unconditionally */
static void stop_dptc(struct device *dev)
{
	struct mxc_dptc_data *dptc_data = dev->platform_data;
	struct dptc_device *drv_data = dev->driver_data;
	u32 dptccr;

	dptccr = __raw_readl(drv_data->membase + dptc_data->dptccr_reg_addr);

	/* disable DPTC and mask its interrupt */
	dptccr = ((dptccr & ~(dptc_data->dptc_enable_bit)) |
		  dptc_data->irq_mask) & (~dptc_data->dptc_nvcr_bit);

	__raw_writel(dptccr, drv_data->membase + dptc_data->dptccr_reg_addr);

	/* Restore Turbo Mode voltage to highest wp */
	update_dptc_wp(drv_data, 0);
	drv_data->curr_wp = 0;

	regulator_put(drv_data->dptc_reg);

	pr_info("DPTC has been stopped\n");
}

/*
  This function does not change the working point. It can be
 called from an interrupt context.
*/
void dptc_suspend(int id)
{
	struct mxc_dptc_data *dptc_data;
	struct dptc_device *drv_data;
	u32 dptccr;

	switch (id) {
	case DPTC_GP_ID:
		dptc_data = dev_data0->platform_data;
		drv_data = dev_data0->driver_data;
		break;
	case DPTC_LP_ID:
		if (dev_data1 == NULL)
			return;

		dptc_data = dev_data1->platform_data;
		drv_data = dev_data1->driver_data;
		break;
		/* Unknown DPTC ID */
	default:
		return;
	}

	if (!drv_data->dptc_is_active)
		return;

	dptccr = __raw_readl(drv_data->membase + dptc_data->dptccr_reg_addr);

	/* Disable DPTC and mask its interrupt */
	dptccr = (dptccr & ~(dptc_data->dptc_enable_bit)) | dptc_data->irq_mask;

	__raw_writel(dptccr, drv_data->membase + dptc_data->dptccr_reg_addr);
}
EXPORT_SYMBOL(dptc_suspend);

/*
  This function does not change the working point. It can be
 called from an interrupt context.
*/
void dptc_resume(int id)
{
	struct mxc_dptc_data *dptc_data;
	struct dptc_device *drv_data;
	u32 dptccr;

	switch (id) {
	case DPTC_GP_ID:
		dptc_data = dev_data0->platform_data;
		drv_data = dev_data0->driver_data;
		break;
	case DPTC_LP_ID:
		if (dev_data1 == NULL)
			return;

		dptc_data = dev_data1->platform_data;
		drv_data = dev_data1->driver_data;
		break;
		/* Unknown DPTC ID */
	default:
		return;
	}

	if (!drv_data->dptc_is_active)
		return;

	__raw_writel(dptc_data->dptc_wp_allfreq[0].dcvr0,
		     drv_data->membase + dptc_data->dcvr0_reg_addr);
	__raw_writel(dptc_data->dptc_wp_allfreq[0].dcvr1,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0x4);
	__raw_writel(dptc_data->dptc_wp_allfreq[0].dcvr2,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0x8);
	__raw_writel(dptc_data->dptc_wp_allfreq[0].dcvr3,
		     drv_data->membase + dptc_data->dcvr0_reg_addr + 0xC);

	dptccr = __raw_readl(drv_data->membase + dptc_data->dptccr_reg_addr);

	/* Enable DPTC and unmask its interrupt */
	dptccr = (dptccr & ~(dptc_data->irq_mask)) | dptc_data->dptc_enable_bit;

	__raw_writel(dptccr, drv_data->membase + dptc_data->dptccr_reg_addr);
}
EXPORT_SYMBOL(dptc_resume);

/*!
 * This function is called to put the DPTC in a low power state.
 *
 */
void dptc_disable(struct device *dev)
{
	struct dptc_device *drv_data = dev->driver_data;

	if (!(drv_data->dptc_is_active))
		return;

	stop_dptc(dev);
	drv_data->dptc_is_active = 0;
	drv_data->turbo_mode_active = 0;
}

/*!
 * This function is called to resume the DPTC from a low power state.
 *
 */
int dptc_enable(struct device *dev)
{
	struct dptc_device *drv_data = dev->driver_data;

	if (drv_data->dptc_is_active)
		return 0;

	return start_dptc(dev);
}

static ssize_t dptc_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct dptc_device *drv_data = dev->driver_data;

	if (drv_data->dptc_is_active)
		return sprintf(buf, "DPTC is enabled\n");
	else
		return sprintf(buf, "DPTC is disabled\n");
}

static ssize_t dptc_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	if (strstr(buf, "0") != NULL) {
		dptc_disable(dev);
	} else if (strstr(buf, "1") != NULL) {
		dptc_enable(dev);
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, dptc_show, dptc_store);

/*!
 * This is the probe routine for the DPTC driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit mxc_dptc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	u32 dptccr = 0;
	struct clk *ckih_clk;
	struct mxc_dptc_data *dptc_data = pdev->dev.platform_data;

	if (dptc_data == NULL) {
		printk(KERN_ERR "DPTC: Pointer to DPTC data is NULL\
				not started\n");
		return -1;
	}

	dptc_device_data = kzalloc(sizeof(struct dptc_device), GFP_KERNEL);
	if (!dptc_device_data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto err1;
	}

	dptc_device_data->membase = ioremap(res->start,
					    res->end - res->start + 1);

	/*
	 * Request the DPTC interrupt
	 */
	dptc_device_data->irq = platform_get_irq(pdev, 0);
	if (dptc_device_data->irq < 0) {
		ret = dptc_device_data->irq;
		goto err2;
	}

	ret =
	    request_irq(dptc_device_data->irq, dptc_irq, IRQF_SHARED,
			pdev->name, &pdev->dev);
	if (ret) {
		printk(KERN_ERR "DPTC: Unable to attach to DPTC interrupt\n");
		goto err2;
	}

	dptc_device_data->curr_wp = 0;
	dptc_device_data->dptc_is_active = 0;
	dptc_device_data->turbo_mode_active = 0;
	dptc_device_data->ptvai = 0;

	dptccr = __raw_readl(dptc_device_data->membase
			     + dptc_data->dptccr_reg_addr);

	printk(KERN_INFO "DPTC mxc_dptc_probe()\n");

	spin_lock_init(&dptc_device_data->lock);

	if (dptc_data->dptc_wp_allfreq == NULL) {
		ckih_clk = clk_get(NULL, "ckih");
		if (cpu_is_mx31() &
		    (mxc_cpu_is_rev(CHIP_REV_2_0) < 0) &
		    (clk_get_rate(ckih_clk) == 27000000))
			printk(KERN_ERR "DPTC: DPTC not supported on TO1.x \
					& ckih = 27M\n");
		else
			printk(KERN_ERR "DPTC: Pointer to DPTC table is NULL\
					not started\n");
		goto err3;
	}

	dptc_device_data->dptc_reg = regulator_get(NULL, dptc_data->reg_id);
	if (IS_ERR(dptc_device_data->dptc_reg)) {
		clk_put(dptc_device_data->dptc_clk);
		printk(KERN_ERR "%s: failed to get regulator\n", __func__);
		goto err3;
	}

	INIT_DELAYED_WORK(&dptc_device_data->dptc_work, dptc_workqueue_handler);

	/* Enable Reference Circuits */
	dptccr = (dptccr & ~(dptc_data->dcr_mask)) | dptc_data->init_config;
	__raw_writel(dptccr, dptc_device_data->membase
			     + dptc_data->dptccr_reg_addr);

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);
	if (ret) {
		printk(KERN_ERR
		       "DPTC: Unable to register sysdev entry for dptc");
		goto err3;
	}

	if (ret != 0) {
		printk(KERN_ERR "DPTC: Unable to start");
		goto err3;
	}

	dptc_device_data->dptc_clk = clk_get(NULL, dptc_data->clk_id);

	if (pdev->id == 0)
		dev_data0 = &pdev->dev;
	else
		dev_data1 = &pdev->dev;

	dptc_device_data->dptc_platform_data = pdev->dev.platform_data;

	/* Set driver data */
	platform_set_drvdata(pdev, dptc_device_data);

	return 0;

err3:
	free_irq(dptc_device_data->irq, &pdev->dev);
err2:
	iounmap(dptc_device_data->membase);
err1:
	dev_err(&pdev->dev, "Failed to probe DPTC\n");
	kfree(dptc_device_data);
	return ret;
}

/*!
 * This function is called to put DPTC in a low power state.
 *
 * @param   pdev  the device structure
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_dptc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct dptc_device *drv_data = pdev->dev.driver_data;

	if (drv_data->dptc_is_active)
		stop_dptc(&pdev->dev);

	return 0;
}

/*!
 * This function is called to resume the MU from a low power state.
 *
 * @param   dev   the device structure
 * @param   level the stage in device suspension process that we want the
 *                device to be put in
 *
 * @return  The function always returns 0.
 */
static int mxc_dptc_resume(struct platform_device *pdev)
{
	struct dptc_device *drv_data = pdev->dev.driver_data;

	if (drv_data->dptc_is_active)
		return start_dptc(&pdev->dev);

	return 0;
}

static struct platform_driver mxc_dptc_driver = {
	.driver = {
		   .name = "mxc_dptc",
		   .owner = THIS_MODULE,
		   },
	.probe = mxc_dptc_probe,
	.suspend = mxc_dptc_suspend,
	.resume = mxc_dptc_resume,
};

/*!
 * This function is called to resume the MU from a low power state.
 *
 * @param   dev   the device structure used to give information on which MU
 *                device (0 through 3 channels) to suspend
 * @param   level the stage in device suspension process that we want the
 *                device to be put in
 *
 * @return  The function always returns 0.
 */

static int __init dptc_init(void)
{
	if (platform_driver_register(&mxc_dptc_driver) != 0) {
		printk(KERN_ERR "mxc_dptc_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "DPTC driver module loaded\n");

	return 0;
}

static void __exit dptc_cleanup(void)
{
	free_irq(dptc_device_data->irq, NULL);
	iounmap(dptc_device_data->membase);
	kfree(dptc_device_data);

	/* Unregister the device structure */
	platform_driver_unregister(&mxc_dptc_driver);

	printk("DPTC driver module unloaded\n");
}

module_init(dptc_init);
module_exit(dptc_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("DPTC driver");
MODULE_LICENSE("GPL");
