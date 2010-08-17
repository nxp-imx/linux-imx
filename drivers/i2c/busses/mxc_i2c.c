/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_i2c.c
 *
 * @brief Driver for the Freescale Semiconductor MXC I2C buses.
 *
 * Based on i2c driver algorithm for PCF8584 adapters
 *
 * @ingroup MXCI2C
 */

/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include "mxc_i2c_reg.h"

/*!
 * In case the MXC device has multiple I2C modules, this structure is used to
 * store information specific to each I2C module.
 */
typedef struct {
	/*!
	 * This structure is used to identify the physical i2c bus along with
	 * the access algorithms necessary to access it.
	 */
	struct i2c_adapter adap;

	/*!
	 * This waitqueue is used to wait for the data transfer to complete.
	 */
	wait_queue_head_t wq;

	/*!
	 * The base address of the I2C device.
	 */
	void __iomem *membase;

	/*!
	 * The interrupt number used by the I2C device.
	 */
	int irq;

	/*!
	 * The default clock divider value to be used.
	 */
	unsigned int clkdiv;

	/*!
	 * The clock source for the device.
	 */
	struct clk *clk;

	/*!
	 * The current power state of the device
	 */
	bool low_power;

	/*!
	 * Boolean to indicate if data was transferred
	 */
	bool transfer_done;

	/*!
	 * Boolean to indicate if we received an ACK for the data transmitted
	 */
	bool tx_success;
} mxc_i2c_device;

struct clk_div_table {
	int reg_value;
	int div;
};

static const struct clk_div_table i2c_clk_table[] = {
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

/*!
 * Transmit a \b STOP signal to the slave device.
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 */
static void mxc_i2c_stop(mxc_i2c_device *dev)
{
	unsigned int cr, sr;
	int retry = 16;

	cr = readw(dev->membase + MXC_I2CR);
	cr &= ~(MXC_I2CR_MSTA | MXC_I2CR_MTX);
	writew(cr, dev->membase + MXC_I2CR);

	/* Wait till the Bus Busy bit is reset */
	sr = readw(dev->membase + MXC_I2SR);
	while (retry-- && ((sr & MXC_I2SR_IBB))) {
		udelay(3);
		sr = readw(dev->membase + MXC_I2SR);
	}
	if (retry <= 0)
		dev_err(&dev->adap.dev, "Could not set I2C Bus Busy bit"
			" to zero.\n");
}

/*!
 * Wait for the transmission of the data byte to complete. This function waits
 * till we get a signal from the interrupt service routine indicating completion
 * of the address cycle or we time out.
 *
 * @param   dev         the mxc i2c structure used to get to the right i2c device
 * @param   trans_flag  transfer flag
 *
 *
 * @return  The function returns 0 on success or -1 if an ack was not received
 */

static int mxc_i2c_wait_for_tc(mxc_i2c_device *dev, int trans_flag)
{
	int retry = 16;

	while (retry-- && !dev->transfer_done) {
		wait_event_interruptible_timeout(dev->wq,
						 dev->transfer_done,
						 dev->adap.timeout);
	}
	dev->transfer_done = false;

	if (retry <= 0) {
		/* Unable to send data */
		dev_err(&dev->adap.dev, "Data not transmitted\n");
		return -1;
	}

	if (!dev->tx_success) {
		/* An ACK was not received for transmitted byte */
		dev_err(&dev->adap.dev, "ACK not received \n");
		return -1;
	}

	return 0;
}

/*!
 * Transmit a \b START signal to the slave device.
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 * @param   *msg  pointer to a message structure that contains the slave
 *                address
 *
 * @return  The function returns EBUSY on failure, 0 on success.
 */
static int mxc_i2c_start(mxc_i2c_device *dev, struct i2c_msg *msg)
{
	volatile unsigned int cr, sr;
	unsigned int addr_trans;
	int retry = 16;

	/*
	 * Set the slave address and the requested transfer mode
	 * in the data register
	 */
	addr_trans = msg->addr << 1;
	if (msg->flags & I2C_M_RD) {
		addr_trans |= 0x01;
	}

	/* Set the Master bit */
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_MSTA;
	writew(cr, dev->membase + MXC_I2CR);

	/* Wait till the Bus Busy bit is set */
	sr = readw(dev->membase + MXC_I2SR);
	while (retry-- && (!(sr & MXC_I2SR_IBB))) {
		udelay(3);
		sr = readw(dev->membase + MXC_I2SR);
	}
	if (retry <= 0) {
		dev_err(&dev->adap.dev, "Could not grab Bus ownership\n");
		return -EBUSY;
	}

	/* Set the Transmit bit */
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_MTX;
	writew(cr, dev->membase + MXC_I2CR);

	writew(addr_trans, dev->membase + MXC_I2DR);
	return 0;
}

/*!
 * Transmit a \b REPEAT START to the slave device
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 * @param   *msg  pointer to a message structure that contains the slave
 *                address
 */
static int mxc_i2c_repstart(mxc_i2c_device *dev, struct i2c_msg *msg)
{
	volatile unsigned int cr, sr;
	unsigned int addr_trans;
	int retry = 16;

	/*
	 * Set the slave address and the requested transfer mode
	 * in the data register
	 */
	addr_trans = msg->addr << 1;
	if (msg->flags & I2C_M_RD) {
		addr_trans |= 0x01;
	}
	cr = readw(dev->membase + MXC_I2CR);
	cr |= MXC_I2CR_RSTA;
	writew(cr, dev->membase + MXC_I2CR);
	/* Wait till the Bus Busy bit is set */
	sr = readw(dev->membase + MXC_I2SR);
	while (retry-- && (!(sr & MXC_I2SR_IBB))) {
		udelay(3);
		sr = readw(dev->membase + MXC_I2SR);
	}
	if (retry <= 0) {
		dev_err(&dev->adap.dev, "Could not grab Bus ownership\n");
		return -EBUSY;
	}
	writew(addr_trans, dev->membase + MXC_I2DR);
	return 0;
}

/*!
 * Read the received data. The function waits till data is available or times
 * out. Generates a stop signal if this is the last message to be received.
 * Sends an ack for all the bytes received except the last byte.
 *
 * @param  dev       the mxc i2c structure used to get to the right i2c device
 * @param  *msg      pointer to a message structure that contains the slave
 *                   address and a pointer to the receive buffer
 * @param  last      indicates that this is the last message to be received
 * @param  addr_comp flag indicates that we just finished the address cycle
 *
 * @return  The function returns the number of bytes read or -1 on time out.
 */
static int mxc_i2c_readbytes(mxc_i2c_device *dev, struct i2c_msg *msg,
			     int last, int addr_comp)
{
	int i;
	char *buf = msg->buf;
	int len = msg->len;
	volatile unsigned int cr;

	cr = readw(dev->membase + MXC_I2CR);
	/*
	 * Clear MTX to switch to receive mode.
	 */
	cr &= ~MXC_I2CR_MTX;
	/*
	 * Clear the TXAK bit to gen an ack when receiving only one byte.
	 */
	if (len == 1) {
		cr |= MXC_I2CR_TXAK;
	} else {
		cr &= ~MXC_I2CR_TXAK;
	}
	writew(cr, dev->membase + MXC_I2CR);
	/*
	 * Dummy read only at the end of an address cycle
	 */
	if (addr_comp > 0) {
		readw(dev->membase + MXC_I2DR);
	}

	for (i = 0; i < len; i++) {
		/*
		 * Wait for data transmission to complete
		 */
		if (mxc_i2c_wait_for_tc(dev, msg->flags)) {
			mxc_i2c_stop(dev);
			return -1;
		}
		/*
		 * Do not generate an ACK for the last byte
		 */
		if (i == (len - 2)) {
			cr = readw(dev->membase + MXC_I2CR);
			cr |= MXC_I2CR_TXAK;
			writew(cr, dev->membase + MXC_I2CR);
		} else if (i == (len - 1)) {
			if (last) {
				mxc_i2c_stop(dev);
			}
		}
		/* Read the data */
		*buf++ = readw(dev->membase + MXC_I2DR);
	}

	return i;
}

/*!
 * Write the data to the data register. Generates a stop signal if this is
 * the last message to be sent or if no ack was received for the data sent.
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 * @param   *msg  pointer to a message structure that contains the slave
 *                address and data to be sent
 * @param   last  indicates that this is the last message to be received
 *
 * @return  The function returns the number of bytes written or -1 on time out
 *          or if no ack was received for the data that was sent.
 */
static int mxc_i2c_writebytes(mxc_i2c_device *dev, struct i2c_msg *msg,
			      int last)
{
	int i;
	char *buf = msg->buf;
	int len = msg->len;
	volatile unsigned int cr;

	cr = readw(dev->membase + MXC_I2CR);
	/* Set MTX to switch to transmit mode */
	cr |= MXC_I2CR_MTX;
	writew(cr, dev->membase + MXC_I2CR);

	for (i = 0; i < len; i++) {
		/*
		 * Write the data
		 */
		writew(*buf++, dev->membase + MXC_I2DR);
		if (mxc_i2c_wait_for_tc(dev, msg->flags)) {
			mxc_i2c_stop(dev);
			return -1;
		}
	}
	if (last > 0) {
		mxc_i2c_stop(dev);
	}

	return i;
}

/*!
 * Function enables the I2C module and initializes the registers.
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 * @param   trans_flag  transfer flag
 */
static void mxc_i2c_module_en(mxc_i2c_device *dev, int trans_flag)
{
	clk_enable(dev->clk);
	/* Set the frequency divider */
	writew(dev->clkdiv, dev->membase + MXC_IFDR);
	/* Clear the status register */
	writew(0x0, dev->membase + MXC_I2SR);
	/* Enable I2C and its interrupts */
	writew(MXC_I2CR_IEN, dev->membase + MXC_I2CR);
	writew(MXC_I2CR_IEN | MXC_I2CR_IIEN, dev->membase + MXC_I2CR);
}

/*!
 * Disables the I2C module.
 *
 * @param   dev   the mxc i2c structure used to get to the right i2c device
 */
static void mxc_i2c_module_dis(mxc_i2c_device *dev)
{
	writew(0x0, dev->membase + MXC_I2CR);
	clk_disable(dev->clk);
}

/*!
 * The function is registered in the adapter structure. It is called when an MXC
 * driver wishes to transfer data to a device connected to the I2C device.
 *
 * @param   adap   adapter structure for the MXC i2c device
 * @param   msgs[] array of messages to be transferred to the device
 * @param   num    number of messages to be transferred to the device
 *
 * @return  The function returns the number of messages transferred,
 *          \b -EREMOTEIO on I2C failure and a 0 if the num argument is
 *          less than 0.
 */
static int mxc_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			int num)
{
	mxc_i2c_device *dev = (mxc_i2c_device *) (i2c_get_adapdata(adap));
	int i, ret = 0, addr_comp = 0;
	volatile unsigned int sr;
	int retry = 5;

	if (dev->low_power) {
		dev_err(&dev->adap.dev, "I2C Device in low power mode\n");
		return -EREMOTEIO;
	}

	if (num < 1) {
		return 0;
	}

	mxc_i2c_module_en(dev, msgs[0].flags);
	sr = readw(dev->membase + MXC_I2SR);
	/*
	 * Check bus state
	 */

	while ((sr & MXC_I2SR_IBB) && retry--) {
		udelay(5);
		sr = readw(dev->membase + MXC_I2SR);
	}

	if ((sr & MXC_I2SR_IBB) && retry < 0) {
		mxc_i2c_module_dis(dev);
		dev_err(&dev->adap.dev, "Bus busy\n");
		return -EREMOTEIO;
	}

	dev->transfer_done = false;
	dev->tx_success = false;
	for (i = 0; i < num && ret >= 0; i++) {
		addr_comp = 0;
		/*
		 * Send the slave address and transfer direction in the
		 * address cycle
		 */
		if (i == 0) {
			/*
			 * Send a start or repeat start signal
			 */
			if (mxc_i2c_start(dev, &msgs[0]))
				return -EREMOTEIO;
			/* Wait for the address cycle to complete */
			if (mxc_i2c_wait_for_tc(dev, msgs[0].flags)) {
				mxc_i2c_stop(dev);
				mxc_i2c_module_dis(dev);
				return -EREMOTEIO;
			}
			addr_comp = 1;
		} else {
			/*
			 * Generate repeat start only if required i.e the address
			 * changed or the transfer direction changed
			 */
			if ((msgs[i].addr != msgs[i - 1].addr) ||
			    ((msgs[i].flags & I2C_M_RD) !=
			     (msgs[i - 1].flags & I2C_M_RD))) {
				mxc_i2c_repstart(dev, &msgs[i]);
				/* Wait for the address cycle to complete */
				if (mxc_i2c_wait_for_tc(dev, msgs[i].flags)) {
					mxc_i2c_stop(dev);
					mxc_i2c_module_dis(dev);
					return -EREMOTEIO;
				}
				addr_comp = 1;
			}
		}

		/* Transfer the data */
		if (msgs[i].flags & I2C_M_RD) {
			/* Read the data */
			ret = mxc_i2c_readbytes(dev, &msgs[i], (i + 1 == num),
						addr_comp);
			if (ret < 0) {
				dev_err(&dev->adap.dev, "mxc_i2c_readbytes:"
					" fail.\n");
				break;
			}
		} else {
			/* Write the data */
			ret = mxc_i2c_writebytes(dev, &msgs[i], (i + 1 == num));
			if (ret < 0) {
				dev_err(&dev->adap.dev, "mxc_i2c_writebytes:"
					" fail.\n");
				break;
			}
		}
	}

	mxc_i2c_module_dis(dev);
	/*
	 * Decrease by 1 as we do not want Start message to be included in
	 * the count
	 */
	return (i < 0 ? ret : i);
}

/*!
 * Returns the i2c functionality supported by this driver.
 *
 * @param   adap adapter structure for this i2c device
 *
 * @return Returns the functionality that is supported.
 */
static u32 mxc_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

/*!
 * Stores the pointers for the i2c algorithm functions. The algorithm functions
 * is used by the i2c bus driver to talk to the i2c bus
 */
static struct i2c_algorithm mxc_i2c_algorithm = {
	.master_xfer = mxc_i2c_xfer,
	.functionality = mxc_i2c_func
};

/*!
 * Interrupt Service Routine. It signals to the process about the data transfer
 * completion. Also sets a flag if bus arbitration is lost.
 * @param   irq    the interrupt number
 * @param   dev_id driver private data
 *
 * @return  The function returns \b IRQ_HANDLED.
 */
static irqreturn_t mxc_i2c_handler(int irq, void *dev_id)
{
	mxc_i2c_device *dev = dev_id;
	volatile unsigned int sr, cr;

	sr = readw(dev->membase + MXC_I2SR);
	cr = readw(dev->membase + MXC_I2CR);

	/*
	 * Clear the interrupt bit
	 */
	writew(0x0, dev->membase + MXC_I2SR);

	if (sr & MXC_I2SR_IAL) {
		dev_err(&dev->adap.dev, "Bus Arbitration lost\n");
	} else {
		/* Interrupt due byte transfer completion */
		dev->tx_success = true;
		/* Check if RXAK is received in Transmit mode */
		if ((cr & MXC_I2CR_MTX) && (sr & MXC_I2SR_RXAK)) {
			dev->tx_success = false;
		}
		dev->transfer_done = true;
		wake_up_interruptible(&dev->wq);
	}

	return IRQ_HANDLED;
}

/*!
 * This function is called to put the I2C adapter in a low power state. Refer to the
 * document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which I2C
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and -1 on failure.
 */
static int mxci2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	mxc_i2c_device *mxcdev = platform_get_drvdata(pdev);
	volatile unsigned int sr = 0;

	if (mxcdev == NULL) {
		return -1;
	}

	/* Prevent further calls to be processed */
	mxcdev->low_power = true;
	/* Wait till we finish the current transfer */
	sr = readw(mxcdev->membase + MXC_I2SR);
	while (sr & MXC_I2SR_IBB) {
		msleep(10);
		sr = readw(mxcdev->membase + MXC_I2SR);
	}
	gpio_i2c_inactive(mxcdev->adap.id);

	return 0;
}

/*!
 * This function is called to bring the I2C adapter back from a low power state. Refer
 * to the document driver-model/driver.txt in the kernel source tree for more
 * information.
 *
 * @param   pdev  the device structure used to give information on which I2C
 *                to resume
 *
 * @return  The function returns 0 on success and -1 on failure
 */
static int mxci2c_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	mxc_i2c_device *mxcdev = platform_get_drvdata(pdev);

	if (mxcdev == NULL)
		return -1;

	mxcdev->low_power = false;
	gpio_i2c_active(mxcdev->adap.id);

	return 0;
}

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions
 *
 * @return  The function always returns 0.
 */
static int mxci2c_probe(struct platform_device *pdev)
{
	mxc_i2c_device *mxc_i2c;
	struct mxc_i2c_platform_data *i2c_plat_data = pdev->dev.platform_data;
	struct resource *res;
	int id = pdev->id;
	u32 clk_freq;
	int ret = 0;
	int i;

	mxc_i2c = kzalloc(sizeof(mxc_i2c_device), GFP_KERNEL);
	if (!mxc_i2c) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto err1;
	}
	mxc_i2c->membase = ioremap(res->start, res->end - res->start + 1);

	/*
	 * Request the I2C interrupt
	 */
	mxc_i2c->irq = platform_get_irq(pdev, 0);
	if (mxc_i2c->irq < 0) {
		ret = mxc_i2c->irq;
		goto err2;
	}

	ret = request_irq(mxc_i2c->irq, mxc_i2c_handler,
			  0, pdev->name, mxc_i2c);
	if (ret < 0) {
		goto err2;
	}

	init_waitqueue_head(&mxc_i2c->wq);

	mxc_i2c->low_power = false;

	gpio_i2c_active(id);

	mxc_i2c->clk = clk_get(&pdev->dev, "i2c_clk");
	clk_freq = clk_get_rate(mxc_i2c->clk);
	mxc_i2c->clkdiv = -1;
	if (i2c_plat_data->i2c_clk) {
		/* Calculate divider and round up any fractional part */
		int div = (clk_freq + i2c_plat_data->i2c_clk - 1) /
		    i2c_plat_data->i2c_clk;
		for (i = 0; i2c_clk_table[i].div != 0; i++) {
			if (i2c_clk_table[i].div >= div) {
				mxc_i2c->clkdiv = i2c_clk_table[i].reg_value;
				break;
			}
		}
	}
	if (mxc_i2c->clkdiv == -1) {
		i--;
		mxc_i2c->clkdiv = 0x1F;	/* Use max divider */
	}
	dev_dbg(&pdev->dev, "i2c speed is %d/%d = %d bps, reg val = 0x%02X\n",
		clk_freq, i2c_clk_table[i].div,
		clk_freq / i2c_clk_table[i].div, mxc_i2c->clkdiv);

	/*
	 * Set the adapter information
	 */
	strlcpy(mxc_i2c->adap.name, pdev->name, 48);
	mxc_i2c->adap.id = mxc_i2c->adap.nr = id;
	mxc_i2c->adap.algo = &mxc_i2c_algorithm;
	mxc_i2c->adap.timeout = 1;
	platform_set_drvdata(pdev, mxc_i2c);
	i2c_set_adapdata(&mxc_i2c->adap, mxc_i2c);
	ret = i2c_add_numbered_adapter(&mxc_i2c->adap);
	if (ret < 0)
		goto err3;

	printk(KERN_INFO "MXC I2C driver\n");
	return 0;

      err3:
	free_irq(mxc_i2c->irq, mxc_i2c);
	gpio_i2c_inactive(id);
      err2:
	iounmap(mxc_i2c->membase);
      err1:
	dev_err(&pdev->dev, "failed to probe i2c adapter\n");
	kfree(mxc_i2c);
	return ret;
}

/*!
 * Dissociates the driver from the I2C device.
 *
 * @param   pdev   the device structure used to give information on which I2C
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxci2c_remove(struct platform_device *pdev)
{
	mxc_i2c_device *mxc_i2c = platform_get_drvdata(pdev);
	int id = pdev->id;

	free_irq(mxc_i2c->irq, mxc_i2c);
	i2c_del_adapter(&mxc_i2c->adap);
	gpio_i2c_inactive(id);
	clk_put(mxc_i2c->clk);
	platform_set_drvdata(pdev, NULL);
	iounmap(mxc_i2c->membase);
	kfree(mxc_i2c);
	return 0;
}

static struct dev_pm_ops mxci2c_dev_pm_ops = {
	.suspend_noirq = mxci2c_suspend_noirq,
	.resume_noirq = mxci2c_resume_noirq,
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxci2c_driver = {
	.driver = {
		   .name = "mxc_i2c",
		   .owner = THIS_MODULE,
		   .pm = &mxci2c_dev_pm_ops,
		   },
	.probe = mxci2c_probe,
	.remove = mxci2c_remove,
};

/*!
 * Function requests the interrupts and registers the i2c adapter structures.
 *
 * @return The function returns 0 on success and a non-zero value on failure.
 */
static int __init mxc_i2c_init(void)
{
	/* Register the device driver structure. */
	return platform_driver_register(&mxci2c_driver);
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxc_i2c_exit(void)
{
	platform_driver_unregister(&mxci2c_driver);
}

subsys_initcall(mxc_i2c_init);
module_exit(mxc_i2c_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC I2C driver");
MODULE_LICENSE("GPL");
