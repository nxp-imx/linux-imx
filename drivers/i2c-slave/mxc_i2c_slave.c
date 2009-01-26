/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <asm/io.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include "i2c_slave_device.h"
#include "mxc_i2c_slave.h"
#include "mxc_i2c_slave_reg.h"

struct mxc_i2c_slave_clk {
	int reg_value;
	int div;
};

static const struct mxc_i2c_slave_clk i2c_clk_table[] = {
	{0x20, 22}, {0x21, 24}, {0x22, 26}, {0x23, 28},
	{0, 30}, {1, 32}, {0x24, 32}, {2, 36},
	{0x25, 36}, {0x26, 40}, {3, 42}, {0x27, 44},
	{4, 48}, {0x28, 48}, {5, 52}, {0x29, 56},
	{6, 60}, {0x2A, 64}, {7, 72}, {0x2B, 72},
	{8, 80}, {0x2C, 80}, {9, 88}, {0x2D, 96},
	{0xA, 104}, {0x2E, 112}, {0xB, 128}, {0x2F, 128},
	{0xC, 144}, {0xD, 160}, {0x30, 160}, {0xE, 192},
	{0x31, 192}, {0x32, 224}, {0xF, 240}, {0x33, 256},
	{0x10, 288}, {0x11, 320}, {0x34, 320}, {0x12, 384},
	{0x35, 384}, {0x36, 448}, {0x13, 480}, {0x37, 512},
	{0x14, 576}, {0x15, 640}, {0x38, 640}, {0x16, 768},
	{0x39, 768}, {0x3A, 896}, {0x17, 960}, {0x3B, 1024},
	{0x18, 1152}, {0x19, 1280}, {0x3C, 1280}, {0x1A, 1536},
	{0x3D, 1536}, {0x3E, 1792}, {0x1B, 1920}, {0x3F, 2048},
	{0x1C, 2304}, {0x1D, 2560}, {0x1E, 3072}, {0x1F, 3840},
	{0, 0}
};

extern void gpio_i2c_active(int i2c_num);
extern void gpio_i2c_inactive(int i2c_num);

static irqreturn_t interrupt_handler(int irq, void *dev_id)
{
	u16 status, ctl;
	int num;
	u16 data;
	struct mxc_i2c_slave_device *mxc_i2c =
	    (struct mxc_i2c_slave_device *)dev_id;

	status = readw(mxc_i2c->reg_base + MXC_I2SR);
	ctl = readw(mxc_i2c->reg_base + MXC_I2CR);

	dev_dbg(mxc_i2c->dev->dev, "status=%x, ctl=%x\n", status, ctl);

	if (status & MXC_I2SR_IAAS) {
		if (status & MXC_I2SR_SRW) {
			/*slave send */
			num =
			    i2c_slave_device_consume(mxc_i2c->dev, 1,
						     (u8 *) &data);
			if (num < 1) {
				/*FIXME:not ready to send data */
				printk(KERN_ERR
				       " i2c-slave:%s:data not ready\n",
				       __func__);
			} else {
				ctl |= MXC_I2CR_MTX;
				writew(ctl, mxc_i2c->reg_base + MXC_I2CR);
				/*send data */
				writew(data, mxc_i2c->reg_base + MXC_I2DR);
			}

		} else {
			/*slave receive */
			ctl &= ~MXC_I2CR_MTX;
			writew(ctl, mxc_i2c->reg_base + MXC_I2CR);

			/*dummy read */
			data = readw(mxc_i2c->reg_base + MXC_I2DR);
		}
	} else {
		/*slave send */
		if (ctl & MXC_I2CR_MTX) {
			if (!(status & MXC_I2SR_RXAK)) {	/*ACK received */
				num =
				    i2c_slave_device_consume(mxc_i2c->dev, 1,
							     (u8 *) &data);
				if (num < 1) {
					/*FIXME:not ready to send data */
					printk(KERN_ERR
					       " i2c-slave:%s:data not ready\n",
					       __func__);
				} else {
					ctl |= MXC_I2CR_MTX;
					writew(ctl,
					       mxc_i2c->reg_base + MXC_I2CR);
					writew(data,
					       mxc_i2c->reg_base + MXC_I2DR);
				}
			} else {
				/*no ACK. */
				/*dummy read */
				ctl &= ~MXC_I2CR_MTX;
				writew(ctl, mxc_i2c->reg_base + MXC_I2CR);
				data = readw(mxc_i2c->reg_base + MXC_I2DR);
			}
		} else {	/*read */
			ctl &= ~MXC_I2CR_MTX;
			writew(ctl, mxc_i2c->reg_base + MXC_I2CR);

			/*read */
			data = readw(mxc_i2c->reg_base + MXC_I2DR);
			i2c_slave_device_produce(mxc_i2c->dev, 1,
						 (u8 *) &data);
		}

	}

	writew(0x0, mxc_i2c->reg_base + MXC_I2SR);

	return IRQ_HANDLED;
}

static int start(i2c_slave_device_t *device)
{
	volatile unsigned int cr;
	unsigned int addr;
	struct mxc_i2c_slave_device *mxc_dev;

	mxc_dev = (struct mxc_i2c_slave_device *)device->private_data;
	if (!mxc_dev) {
		dev_err(device->dev, "%s: get mxc_dev error\n", __func__);
		return -ENODEV;
	}

	clk_enable(mxc_dev->clk);
	/* Set the frequency divider */
	writew(mxc_dev->clkdiv, mxc_dev->reg_base + MXC_IFDR);

	/* Set the Slave bit */
	cr = readw(mxc_dev->reg_base + MXC_I2CR);
	cr &= (!MXC_I2CR_MSTA);
	writew(cr, mxc_dev->reg_base + MXC_I2CR);

	/*Set Slave Address */
	addr = mxc_dev->dev->address << 1;
	writew(addr, mxc_dev->reg_base + MXC_IADR);

	/* Clear the status register */
	writew(0x0, mxc_dev->reg_base + MXC_I2SR);

	/* Enable I2C and its interrupts */
	writew(MXC_I2CR_IEN, mxc_dev->reg_base + MXC_I2CR);
	writew(MXC_I2CR_IEN | MXC_I2CR_IIEN, mxc_dev->reg_base + MXC_I2CR);

	return 0;

}

static int stop(i2c_slave_device_t *device)
{
	struct mxc_i2c_slave_device *mxc_dev;

	mxc_dev = (struct mxc_i2c_slave_device *)device->private_data;

	writew(0x0, mxc_dev->reg_base + MXC_I2CR);
	clk_disable(mxc_dev->clk);

	return 0;
}

static int mxc_i2c_slave_probe(struct platform_device *pdev)
{
	int i;
	u32 clk_freq;
	struct resource *res;
	struct mxc_i2c_slave_device *mxc_dev;

	mxc_dev = kzalloc(sizeof(struct mxc_i2c_slave_device), GFP_KERNEL);
	if (!mxc_dev) {
		goto error0;
	}
	mxc_dev->dev = i2c_slave_device_alloc();
	if (mxc_dev->dev == 0) {
		dev_err(&pdev->dev, "%s: i2c_slave_device_alloc error\n",
			__func__);
		goto error1;
	}

	i2c_slave_device_set_address(mxc_dev->dev, MXC_I2C_SLAVE_ADDRESS);
	i2c_slave_device_set_freq(mxc_dev->dev, MXC_I2C_SLAVE_FREQ);
	i2c_slave_device_set_name(mxc_dev->dev, MXC_I2C_SLAVE_NAME);
	mxc_dev->dev->start = start;
	mxc_dev->dev->stop = stop;

	mxc_dev->dev->private_data = mxc_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "%s: get resource error\n", __func__);
		goto error2;
	}
	mxc_dev->reg_base = IO_ADDRESS(res->start);

	mxc_dev->irq = platform_get_irq(pdev, 0);
	if (mxc_dev->irq < 0) {
		dev_err(&pdev->dev, "%s: get irq error\n", __func__);
		goto error2;
	}
	if (request_irq(mxc_dev->irq, interrupt_handler,
			0, mxc_dev->dev->name, mxc_dev) < 0) {
		dev_err(&pdev->dev, "%s: request_irq error\n", __func__);
		goto error2;
	}

	/*i2c id on soc */
	mxc_dev->id = pdev->id;

	gpio_i2c_active(mxc_dev->id);

	/*clock */
	mxc_dev->clk = clk_get(&pdev->dev, "i2c_clk");
	clk_freq = clk_get_rate(mxc_dev->clk);
	mxc_dev->clkdiv = -1;
	if (mxc_dev->dev->scl_freq) {
		/* Calculate divider and round up any fractional part */
		int div = (clk_freq + mxc_dev->dev->scl_freq - 1) /
		    mxc_dev->dev->scl_freq;
		for (i = 0; i2c_clk_table[i].div != 0; i++) {
			if (i2c_clk_table[i].div >= div) {
				mxc_dev->clkdiv = i2c_clk_table[i].reg_value;
				break;
			}
		}
	}
	if (mxc_dev->clkdiv == -1) {
		i--;
		mxc_dev->clkdiv = 0x1F;	/* Use max divider */
	}
	dev_dbg(&pdev->dev,
		"i2c slave speed is %d/%d = %d bps, reg val = 0x%02X\n",
		clk_freq, i2c_clk_table[i].div, clk_freq / i2c_clk_table[i].div,
		mxc_dev->clkdiv);

	if (i2c_slave_device_register(mxc_dev->dev) < 0) {
		dev_err(&pdev->dev, "%s: i2c_slave_device_register error\n",
			__func__);
		goto error3;
	}

	platform_set_drvdata(pdev, (void *)mxc_dev);

	/*start(mxc_dev->dev); */
	return 0;

      error3:
      error2:
	i2c_slave_device_free(mxc_dev->dev);
      error1:
	kfree(mxc_dev);
      error0:
	return -ENODEV;
}

static int mxc_i2c_slave_remove(struct platform_device *pdev)
{
	struct mxc_i2c_slave_device *mxc_dev;
	mxc_dev = (struct mxc_i2c_slave_device *)platform_get_drvdata(pdev);

	i2c_slave_device_unregister(mxc_dev->dev);
	kfree((void *)mxc_dev);

	return 0;
}

static int mxc_i2c_slave_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	return 0;
}

static int mxc_i2c_slave_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mxci2c_slave_driver = {
	.driver = {
		   .name = "mxc_i2c_slave",
		   .owner = THIS_MODULE,
		   },
	.probe = mxc_i2c_slave_probe,
	.remove = mxc_i2c_slave_remove,
	.suspend = mxc_i2c_slave_suspend,
	.resume = mxc_i2c_slave_resume,
};

static int __init mxc_i2c_slave_init(void)
{
	/* Register the device driver structure. */
	return platform_driver_register(&mxci2c_slave_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxc_i2c_slave_exit(void)
{
	platform_driver_unregister(&mxci2c_slave_driver);
}

module_init(mxc_i2c_slave_init);
module_exit(mxc_i2c_slave_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC I2C Slave Driver");
MODULE_LICENSE("GPL");
