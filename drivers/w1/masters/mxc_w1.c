/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup MXC_OWIRE MXC Driver for owire interface
 */

/*!
 * @file mxc_w1.c
 *
 * @brief Driver for the Freescale Semiconductor MXC owire interface.
 *
 *
 * @ingroup MXC_OWIRE
 */

/*!
 * Include Files
 */

#include <asm/atomic.h>
#include <asm/types.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <asm/setup.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_log.h"

/*
 * mxc function declarations
 */

static int __devinit mxc_w1_probe(struct platform_device *pdev);
static int __devexit mxc_w1_remove(struct platform_device *pdev);
static DECLARE_COMPLETION(transmit_done);
extern void gpio_owire_active(void);
extern void gpio_owire_inactive(void);

/*
 * MXC W1 Register offsets
 */
#define MXC_W1_CONTROL          0x00
#define MXC_W1_TIME_DIVIDER     0x02
#define MXC_W1_RESET            0x04
#define MXC_W1_COMMAND          0x06
#define MXC_W1_TXRX             0x08
#define MXC_W1_INTERRUPT        0x0A
#define MXC_W1_INTERRUPT_EN     0x0C
DEFINE_SPINLOCK(w1_lock);

/*!
 * This structure contains pointers to  callback functions.
 */
static struct platform_driver mxc_w1_driver = {
	.driver = {
		   .name = "mxc_w1",
		   },
	.probe = mxc_w1_probe,
	.remove = mxc_w1_remove,
};

/*!
 * This structure is used to store
 * information specific to w1 module.
 */

struct mxc_w1_device {
	char *base_address;
	unsigned long found;
	unsigned int clkdiv;
	struct clk *clk;
	struct w1_bus_master *bus_master;
};

/*
 * this is the low level routine to
 * reset the device on the One Wire interface
 * on the hardware
 * @param data  the data field of the w1 device structure
 * @return the function returns 0 when the reset pulse has
 *  been generated
 */
static u8 mxc_w1_ds2_reset_bus(void *data)
{
	volatile u8 reg_val;
	u8 ret;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;

	__raw_writeb(0x80, (dev->base_address + MXC_W1_CONTROL));

	do {
		reg_val = __raw_readb(dev->base_address + MXC_W1_CONTROL);
	} while (((reg_val >> 7) & 0x1) != 0);
	ret = ((reg_val >> 7) & 0x1);
	return ret;
}

/*!
 * this is the low level routine to read/write a bit on the One Wire
 * interface on the hardware
 * @param data  the data field of the w1 device structure
 * @param bit  0 = write-0 cycle, 1 = write-1/read cycle
 * @return the function returns the bit read (0 or 1)
 */
static u8 mxc_w1_ds2_touch_bit(void *data, u8 bit)
{

	volatile u8 reg_val;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	u8 ret = 0;

	if (0 == bit) {
		__raw_writeb((1 << 5), (dev->base_address + MXC_W1_CONTROL));

		do {
			reg_val =
			    __raw_readb(dev->base_address + MXC_W1_CONTROL);
		} while (0 != ((reg_val >> 5) & 0x1));
	}

	else {
		__raw_writeb((1 << 4), dev->base_address + MXC_W1_CONTROL);
		do {
			reg_val =
			    __raw_readb(dev->base_address + MXC_W1_CONTROL);
		} while (0 != ((reg_val >> 4) & 0x1));

		reg_val =
		    (((__raw_readb(dev->base_address + MXC_W1_CONTROL)) >> 3) &
		     0x1);
		ret = (u8) (reg_val);
	}

	return ret;
}

static void mxc_w1_ds2_write_byte(void *data, u8 byte)
{
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	INIT_COMPLETION(transmit_done);
	__raw_writeb(byte, (dev->base_address + MXC_W1_TXRX));
	__raw_writeb(0x10, (dev->base_address + MXC_W1_INTERRUPT_EN));
	wait_for_completion(&transmit_done);
}
static u8 mxc_w1_ds2_read_byte(void *data)
{
	volatile u8 reg_val;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	mxc_w1_ds2_write_byte(data, 0xFF);
	reg_val = __raw_readb((dev->base_address + MXC_W1_TXRX));
	return reg_val;
}
static u8 mxc_w1_read_byte(void *data)
{
	volatile u8 reg_val;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	reg_val = __raw_readb((dev->base_address + MXC_W1_TXRX));
	return reg_val;
}
static irqreturn_t w1_interrupt_handler(int irq, void *data)
{
	u8 reg_val;
	irqreturn_t ret = IRQ_NONE;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	reg_val = __raw_readb((dev->base_address + MXC_W1_INTERRUPT));
	if ((reg_val & 0x10)) {
		complete(&transmit_done);
		reg_val = __raw_readb((dev->base_address + MXC_W1_TXRX));
		ret = IRQ_HANDLED;
	}
	return ret;
}
void search_ROM_accelerator(void *data, struct w1_master *master, u8 search_type,
			    w1_slave_found_callback cb)
{
	u64 rn[2], last_rn[2], rn2[2];
	u64 rn1, rom_id, temp, temp1;
	int i, j, z, w, last_zero, loop;
	u8 bit, reg_val, bit2;
	u8 byte, byte1;
	int disc, prev_disc, last_disc;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	last_rn[0] = 0;
	last_rn[1] = 0;
	rom_id = 0;
	prev_disc = 0;
	loop = 0;
	disc = -1;
	last_disc = 0;
	last_zero = 0;
	while (!last_zero) {
		/*
		 * Reset bus and all 1-wire device state machines
		 * so they can respond to our requests.
		 *
		 * Return 0 - device(s) present, 1 - no devices present.
		 */
		if (mxc_w1_ds2_reset_bus(data)) {
			pr_debug("No devices present on the wire.\n");
			break;
		}
		rn[0] = 0;
		rn[1] = 0;
		__raw_writeb(0x80, (dev->base_address + MXC_W1_CONTROL));
		mdelay(1);
		mxc_w1_ds2_write_byte(data, 0xF0);
		__raw_writeb(0x02, (dev->base_address + MXC_W1_COMMAND));
		memcpy(rn2, last_rn, 16);
		z = 0;
		w = 0;
		for (i = 0; i < 16; i++) {
			reg_val = rn2[z] >> (8 * w);
			mxc_w1_ds2_write_byte(data, reg_val);
			reg_val = mxc_w1_read_byte(data);
			if ((reg_val & 0x3) == 0x3) {
				pr_debug("Device is Not Responding\n");
				break;
			}
			for (j = 0; j < 8; j += 2) {
				byte = 0xFF;
				byte1 = 1;
				byte ^= byte1 << j;
				bit = (reg_val >> j) & 0x1;
				bit2 = (reg_val >> j);
				if (bit) {
					prev_disc = disc;
					disc = 8 * i + j;
					reg_val &= byte;
				}
			}
			rn1 = 0;
			rn1 = reg_val;
			rn[z] |= rn1 << (8 * w);
			w++;
			if (i == 7) {
				z++;
				w = 0;
			}
		}
		if ((disc == -1) || (disc == prev_disc))
			last_zero = 1;
		if (disc == last_disc)
			disc = prev_disc;
		z = 0;
		rom_id = 0;
		for (i = 0, j = 1; i < 64; i++) {
			temp = 0;
			temp = (rn[z] >> j) & 0x1;
			rom_id |= (temp << i);
			j += 2;
			if (i == 31) {
				z++;
				j = 1;
			}

		}
		if (disc > 63) {
			last_rn[0] = rn[0];
			temp1 = rn[1];
			loop = disc % 64;
			temp = 1;
			temp1 |= (temp << (loop + 1)) - 1;
			temp1 |= (temp << (loop + 1));
			last_rn[1] = temp1;

		} else {
			last_rn[1] = 0;
			temp1 = rn[0];
			temp = 1;
			temp1 |= (temp << (loop + 1)) - 1;
			temp1 |= (temp << (loop + 1));
			last_rn[0] = temp1;
		}
		last_disc = disc;
		cb(master, rom_id);
	}
}

/*!
 * this routine sets the One Wire clock
 * to a value of 1 Mhz, as required by
 * hardware.
 * @param   dev   the device structure for w1
 * @return  The function returns void
 */
static void mxc_w1_hw_init(struct mxc_w1_device *dev)
{
	clk_enable(dev->clk);

	/* set the timer divider clock to divide by 65 */
	/* as the clock to the One Wire is at 66.5MHz */
	__raw_writeb(dev->clkdiv, dev->base_address + MXC_W1_TIME_DIVIDER);

	return;
}

/*!
 * this is the probe routine for the One Wire driver.
 * It is called during the driver initilaization.
 * @param   pdev   the platform device structure for w1
 * @return The function returns 0 on success
 * and a non-zero value on failure
 *
 */
static int __devinit mxc_w1_probe(struct platform_device *pdev)
{
	struct mxc_w1_device *dev;
	struct mxc_w1_config *data =
	    (struct mxc_w1_config *)pdev->dev.platform_data;
	int flag, ret_val, irq = -ENXIO;
	int err = 0;
	ret_val = 0;
	flag = data->search_rom_accelerator;
	dev = kzalloc(sizeof(struct mxc_w1_device) +
		      sizeof(struct w1_bus_master), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	dev->clk = clk_get(&pdev->dev, "owire_clk");
	dev->bus_master = (struct w1_bus_master *)(dev + 1);
	dev->found = 1;
	dev->clkdiv = (clk_get_rate(dev->clk) / 1000000) - 1;
	dev->base_address = (void *)IO_ADDRESS(OWIRE_BASE_ADDR);

	mxc_w1_hw_init(dev);
	dev->bus_master->data = dev;
	dev->bus_master->reset_bus = &mxc_w1_ds2_reset_bus;
	dev->bus_master->touch_bit = &mxc_w1_ds2_touch_bit;
	if (flag) {
		dev->bus_master->write_byte = &mxc_w1_ds2_write_byte;
		dev->bus_master->read_byte = &mxc_w1_ds2_read_byte;
		dev->bus_master->search = &search_ROM_accelerator;
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			err = -ENOENT;
			goto err_out_free_device;
		}
		ret_val =
		    request_irq(irq, w1_interrupt_handler, 0, "mxc_w1", dev);
		if (ret_val) {
			pr_debug("OWire:request_irq(%d) returned error %d\n",
				 irq, ret_val);
			err = -1;
			goto err_out_free_device;
		}
	}
	err = w1_add_master_device(dev->bus_master);
	if (err)
		goto err_irq;

	platform_set_drvdata(pdev, dev);
	return 0;

err_irq:
	if (irq >= 0)
		free_irq(irq, dev);
err_out_free_device:
	kfree(dev);
	return err;
}

/*
 * disassociate the w1 device from the driver
 * @param   dev   the device structure for w1
 * @return  The function returns void
 */
static int mxc_w1_remove(struct platform_device *pdev)
{
	struct mxc_w1_device *dev = platform_get_drvdata(pdev);
	struct mxc_w1_config *data =
	    (struct mxc_w1_config *)pdev->dev.platform_data;
	int irq;

	if (dev->found) {
		w1_remove_master_device(dev->bus_master);
	}

	irq = platform_get_irq(pdev, 0);
	if ((irq >= 0) && (data->search_rom_accelerator))
		free_irq(irq, dev);

	clk_disable(dev->clk);
	clk_put(dev->clk);

	kfree(dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

/*
 * initialize the driver
 * @return The function returns 0 on success
 * and a non-zero value on failure
 */

static int __init mxc_w1_init(void)
{
	int ret;

	printk(KERN_INFO "Serial: MXC OWire driver\n");

	gpio_owire_active();

	ret = platform_driver_register(&mxc_w1_driver);

	return ret;
}

/*
 * cleanup before the driver exits
 */
static void mxc_w1_exit(void)
{
	gpio_owire_inactive();
	platform_driver_unregister(&mxc_w1_driver);
}

module_init(mxc_w1_init);
module_exit(mxc_w1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductors Inc");
MODULE_DESCRIPTION("Driver for One-Wire on MXC");
