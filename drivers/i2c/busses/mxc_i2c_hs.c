/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include "mxc_i2c_hs_reg.h"

typedef struct {
	struct device *dev;

	void __iomem *reg_base_virt;
	unsigned long reg_base_phy;
	int irq;
	unsigned int speed;
	struct clk *ipg_clk;
	struct clk *serial_clk;
	bool low_power;

	struct i2c_msg *msg;
	int index;
} mxc_i2c_hs;

struct clk_div_table {
	int reg_value;
	int div;
};

static const struct clk_div_table i2c_clk_table[] = {
	{0x0, 16}, {0x1, 18}, {0x2, 20}, {0x3, 22},
	{0x20, 24}, {0x21, 26}, {0x22, 28}, {0x23, 30},
	{0x4, 32}, {0x5, 36}, {0x6, 40}, {0x7, 44},
	{0x24, 48}, {0x25, 52}, {0x26, 56}, {0x27, 60},
	{0x8, 64}, {0x9, 72}, {0xa, 80}, {0xb, 88},
	{0x28, 96}, {0x29, 104}, {0x2a, 112}, {0x2b, 120},
	{0xc, 128}, {0xd, 144}, {0xe, 160}, {0xf, 176},
	{0x2c, 192}, {0x2d, 208}, {0x2e, 224}, {0x2f, 240},
	{0x10, 256}, {0x11, 288}, {0x12, 320}, {0x13, 352},
	{0x30, 384}, {0x31, 416}, {0x32, 448}, {0x33, 480},
	{0x14, 512}, {0x15, 576}, {0x16, 640}, {0x17, 704},
	{0x34, 768}, {0x35, 832}, {0x36, 896}, {0x37, 960},
	{0x18, 1024}, {0x19, 1152}, {0x1a, 1280}, {0x1b, 1408},
	{0x38, 1536}, {0x39, 1664}, {0x3a, 1792}, {0x3b, 1920},
	{0x1c, 2048}, {0x1d, 2304}, {0x1e, 2560}, {0x1f, 2816},
	{0x3c, 3072}, {0x3d, 3328}, {0x3E, 3584}, {0x3F, 3840},
	{-1, -1}
};

static struct i2c_adapter *adap;

extern void gpio_i2c_hs_inactive(void);
extern void gpio_i2c_hs_active(void);

static u16 reg_read(mxc_i2c_hs *i2c_hs, u32 reg_offset)
{
	return __raw_readw(i2c_hs->reg_base_virt + reg_offset);
}

static void reg_write(mxc_i2c_hs *i2c_hs, u32 reg_offset, u16 data)
{
	__raw_writew(data, i2c_hs->reg_base_virt + reg_offset);
}

static void reg_set_mask(mxc_i2c_hs *i2c_hs, u32 reg_offset, u16 mask)
{
	u16 value;

	value = reg_read(i2c_hs, reg_offset);
	value |= mask;
	reg_write(i2c_hs, reg_offset, value);
}
static void reg_clear_mask(mxc_i2c_hs *i2c_hs, u32 reg_offset, u16 mask)
{
	u16 value;

	value = reg_read(i2c_hs, reg_offset);
	value &= ~mask;
	reg_write(i2c_hs, reg_offset, value);
}

static void mxci2c_hs_set_div(mxc_i2c_hs *i2c_hs)
{
	unsigned long clk_freq;
	int i;
	int div = -1;;

	clk_freq = clk_get_rate(i2c_hs->serial_clk);
	if (i2c_hs->speed) {
		div = (clk_freq + i2c_hs->speed - 1) / i2c_hs->speed;
		for (i = 0; i2c_clk_table[i].div >= 0; i++) {
			if (i2c_clk_table[i].div >= div) {
				div = i2c_clk_table[i].reg_value;
				reg_write(i2c_hs, HIFSFDR, div);
				break;
			}
		}
	}
}

static int mxci2c_hs_enable(mxc_i2c_hs *i2c_hs)
{
	gpio_i2c_hs_active();
	clk_enable(i2c_hs->ipg_clk);
	clk_enable(i2c_hs->serial_clk);
	mxci2c_hs_set_div(i2c_hs);
	reg_write(i2c_hs, HICR, reg_read(i2c_hs, HICR) | HICR_HIEN);

	return 0;
}

static int mxci2c_hs_disable(mxc_i2c_hs *i2c_hs)
{
	reg_write(i2c_hs, HICR, reg_read(i2c_hs, HICR) & (~HICR_HIEN));
	clk_disable(i2c_hs->ipg_clk);
	clk_disable(i2c_hs->serial_clk);

	return 0;
}

static int mxci2c_hs_bus_busy(mxc_i2c_hs *i2c_hs)
{
	u16 value;
	int retry = 1000;

	while (retry--) {
		value = reg_read(i2c_hs, HISR);
		if (value & HISR_HIBB) {
			udelay(1);
		} else {
			break;
		}
	}

	if (retry <= 0) {
		dev_dbg(NULL, "%s: Bus Busy!\n", __func__);
		return 1;
	} else {
		return 0;
	}
}

static int mxci2c_hs_start(mxc_i2c_hs *i2c_hs, int repeat_start, u16 address)
{
	u16 mask;
	int ret = 0;

	mxci2c_hs_bus_busy(i2c_hs);

	/*7 bit address */
	reg_clear_mask(i2c_hs, HICR, HICR_ADDR_MODE);

	/*send start */
	if (repeat_start)
		mask = HICR_RSTA;
	else
		mask = HICR_MSTA;
	reg_set_mask(i2c_hs, HICR, mask);

	return ret;
}

static int mxci2c_hs_stop(mxc_i2c_hs *i2c_hs)
{
	reg_clear_mask(i2c_hs, HICR, HICR_MSTA);
	reg_clear_mask(i2c_hs, HICR, HICR_HIIEN);

	return 0;
}

static int mxci2c_wait_writefifo(mxc_i2c_hs *i2c_hs)
{
	int i, num, left;
	int retry, ret = 0;

	retry = 10000;
	while (retry--) {
		udelay(10);
		if (reg_read(i2c_hs, HISR) & (HISR_TDE | HISR_TDC_ZERO)) {
			if (i2c_hs->index < i2c_hs->msg->len) {
				left = i2c_hs->msg->len - i2c_hs->index;
				num =
				    (left >
				     HITFR_MAX_COUNT) ? HITFR_MAX_COUNT : left;
				for (i = 0; i < num; i++) {
					reg_write(i2c_hs, HITDR,
						  i2c_hs->msg->buf[i2c_hs->
								   index + i]);
				}
				i2c_hs->index += num;
			} else {
				if (reg_read(i2c_hs, HISR) & HISR_TDC_ZERO) {
					msleep(1);
					break;
				}
			}
		}
	}

	if (retry <= 0) {
		printk(KERN_ERR "%s:wait error\n", __func__);
		ret = -1;
	}

	return ret;
}

static int mxci2c_wait_readfifo(mxc_i2c_hs *i2c_hs)
{
	int i, num, left;
	int retry, ret = 0;
	u16 value;

	retry = 10000;
	while (retry--) {
		udelay(10);
		value = reg_read(i2c_hs, HISR);
		if (value & (HISR_RDF | HISR_RDC_ZERO)) {
			if (i2c_hs->index < i2c_hs->msg->len) {
				left = i2c_hs->msg->len - i2c_hs->index;
				num =
				    (left >
				     HITFR_MAX_COUNT) ? HITFR_MAX_COUNT : left;
				for (i = 0; i < num; i++) {
					i2c_hs->msg->buf[i2c_hs->index + i] =
					    reg_read(i2c_hs, HIRDR);
				}
				i2c_hs->index += num;
			} else {
				if (value & HISR_RDC_ZERO) {
					break;
				}
			}
		}
	}

	if (retry <= 0) {
		printk(KERN_ERR "%s:wait error\n", __func__);
		ret = -1;
	}

	return ret;
}

static int mxci2c_hs_read(mxc_i2c_hs *i2c_hs, int repeat_start,
			  struct i2c_msg *msg)
{
	int ret;

	if (msg->len > HIRDCR_MAX_COUNT) {
		printk(KERN_ERR "%s: error: msg too long, max longth 256\n",
		       __func__);
		return -1;
	}

	ret = 0;
	i2c_hs->msg = msg;
	i2c_hs->index = 0;

	/*set address */
	reg_write(i2c_hs, HIMADR, HIMADR_LSB_ADR(msg->addr));

	/*receive mode */
	reg_clear_mask(i2c_hs, HICR, HICR_MTX);

	reg_clear_mask(i2c_hs, HICR, HICR_HIIEN);

	 /*FIFO*/ reg_set_mask(i2c_hs, HIRFR, HIRFR_RFEN | HIRFR_RFWM(7));
	reg_set_mask(i2c_hs, HIRFR, HIRFR_RFLSH);

	 /*TDCR*/
	    reg_write(i2c_hs, HIRDCR, HIRDCR_RDC_EN | HIRDCR_RDC(msg->len));

	mxci2c_hs_start(i2c_hs, repeat_start, msg->addr);

	ret = mxci2c_wait_readfifo(i2c_hs);

	if (ret < 0)
		return ret;
	else
		return msg->len;
}

static int mxci2c_hs_write(mxc_i2c_hs *i2c_hs, int repeat_start,
			   struct i2c_msg *msg)
{
	int ret, i;

	if (msg->len > HITDCR_MAX_COUNT) {
		printk(KERN_ERR "%s: error: msg too long, max longth 256\n",
		       __func__);
		return -1;
	}

	ret = 0;
	i2c_hs->msg = msg;
	i2c_hs->index = 0;

	/*set address */
	reg_write(i2c_hs, HIMADR, HIMADR_LSB_ADR(msg->addr));

	/*transmit mode */
	reg_set_mask(i2c_hs, HICR, HICR_MTX);

	reg_clear_mask(i2c_hs, HICR, HICR_HIIEN);

	/* TDCR */
	reg_write(i2c_hs, HITDCR, HITDCR_TDC_EN | HITDCR_TDC(msg->len));

	/* FIFO */
	reg_set_mask(i2c_hs, HITFR, HITFR_TFEN);
	reg_set_mask(i2c_hs, HITFR, HITFR_TFLSH);

	if (msg->len > HITFR_MAX_COUNT)
		i2c_hs->index = HITFR_MAX_COUNT;
	else {
		i2c_hs->index = msg->len;
	}

	for (i = 0; i < i2c_hs->index; i++) {
		reg_write(i2c_hs, HITDR, msg->buf[i]);
	}

	mxci2c_hs_start(i2c_hs, repeat_start, msg->addr);

	ret = mxci2c_wait_writefifo(i2c_hs);

	if (ret < 0)
		return ret;
	else
		return msg->len;
}

static int mxci2c_hs_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			  int num)
{
	int i;
	int ret = -EIO;

	mxc_i2c_hs *i2c_hs = (mxc_i2c_hs *) (i2c_get_adapdata(adap));

	if (i2c_hs->low_power) {
		dev_err(&adap->dev, "I2C Device in low power mode\n");
		return -EREMOTEIO;
	}

	if (num < 1) {
		return 0;
	}

	mxci2c_hs_enable(i2c_hs);

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD) {
			ret = mxci2c_hs_read(i2c_hs, 0, &msgs[i]);
			if (ret < 0)
				break;
		} else {
			ret = mxci2c_hs_write(i2c_hs, 0, &msgs[i]);
			if (ret < 0)
				break;
		}
		mxci2c_hs_stop(i2c_hs);
	}
	mxci2c_hs_stop(i2c_hs);

	mxci2c_hs_disable(i2c_hs);

	if (ret < 0)
		return ret;

	return i;
}

static u32 mxci2c_hs_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*!
 * Stores the pointers for the i2c algorithm functions. The algorithm functions
 * is used by the i2c bus driver to talk to the i2c bus
 */
static struct i2c_algorithm mxci2c_hs_algorithm = {
	.master_xfer = mxci2c_hs_xfer,
	.functionality = mxci2c_hs_func
};

static int mxci2c_hs_probe(struct platform_device *pdev)
{
	mxc_i2c_hs *i2c_hs;
	struct mxc_i2c_platform_data *i2c_plat_data = pdev->dev.platform_data;
	struct resource *res;
	int id = pdev->id;
	int ret = 0;

	i2c_hs = kzalloc(sizeof(mxc_i2c_hs), GFP_KERNEL);
	if (!i2c_hs) {
		return -ENOMEM;
	}

	i2c_hs->dev = &pdev->dev;

	i2c_hs->speed = i2c_plat_data->i2c_clk;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto err1;
	}
	i2c_hs->reg_base_virt = ioremap(res->start, res->end - res->start + 1);
	i2c_hs->reg_base_phy = res->start;

	i2c_hs->ipg_clk = clk_get(&pdev->dev, "hsi2c_clk");
	i2c_hs->serial_clk = clk_get(&pdev->dev, "hsi2c_serial_clk");

	/*
	 * Request the I2C interrupt
	 */
	i2c_hs->irq = platform_get_irq(pdev, 0);
	if (i2c_hs->irq < 0) {
		ret = i2c_hs->irq;
		goto err1;
	}

	i2c_hs->low_power = false;

	/*
	 * Set the adapter information
	 */
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap) {
		ret = -ENODEV;
		goto err1;
	}
	strlcpy(adap->name, pdev->name, 48);
	adap->id = adap->nr = id;
	adap->algo = &mxci2c_hs_algorithm;
	adap->timeout = 1;
	platform_set_drvdata(pdev, i2c_hs);
	i2c_set_adapdata(adap, i2c_hs);
	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0) {
		goto err2;
	}

	printk(KERN_INFO "MXC HS I2C driver\n");
	return 0;

      err2:
	kfree(adap);
      err1:
	dev_err(&pdev->dev, "failed to probe high speed i2c adapter\n");
	iounmap(i2c_hs->reg_base_virt);
	kfree(i2c_hs);
	return ret;
}

static int mxci2c_hs_suspend(struct platform_device *pdev, pm_message_t state)
{
	mxc_i2c_hs *i2c_hs = platform_get_drvdata(pdev);

	if (i2c_hs == NULL) {
		return -1;
	}

	/* Prevent further calls to be processed */
	i2c_hs->low_power = true;

	gpio_i2c_hs_inactive();

	return 0;
}

static int mxci2c_hs_resume(struct platform_device *pdev)
{
	mxc_i2c_hs *i2c_hs = platform_get_drvdata(pdev);

	if (i2c_hs == NULL)
		return -1;

	i2c_hs->low_power = false;
	gpio_i2c_hs_active();

	return 0;
}

static int mxci2c_hs_remove(struct platform_device *pdev)
{
	mxc_i2c_hs *i2c_hs = platform_get_drvdata(pdev);

	i2c_del_adapter(adap);
	gpio_i2c_hs_inactive();
	platform_set_drvdata(pdev, NULL);
	iounmap(i2c_hs->reg_base_virt);
	kfree(i2c_hs);
	return 0;
}

static struct platform_driver mxci2c_hs_driver = {
	.driver = {
		   .name = "mxc_i2c_hs",
		   .owner = THIS_MODULE,
		   },
	.probe = mxci2c_hs_probe,
	.remove = mxci2c_hs_remove,
	.suspend = mxci2c_hs_suspend,
	.resume = mxci2c_hs_resume,
};

/*!
 * Function requests the interrupts and registers the i2c adapter structures.
 *
 * @return The function returns 0 on success and a non-zero value on failure.
 */
static int __init mxci2c_hs_init(void)
{
	/* Register the device driver structure. */
	return platform_driver_register(&mxci2c_hs_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxci2c_hs_exit(void)
{
	platform_driver_unregister(&mxci2c_hs_driver);
}

subsys_initcall(mxci2c_hs_init);
module_exit(mxci2c_hs_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC HIGH SPEED I2C driver");
MODULE_LICENSE("GPL");
