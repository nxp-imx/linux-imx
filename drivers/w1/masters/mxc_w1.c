/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Luotao Fu, kernel@pengutronix.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_log.h"

/* According to the mx27 Datasheet the reset procedure should take up to about
 * 1350us. We set the timeout to 500*100us = 50ms for sure */
#define MXC_W1_RESET_TIMEOUT 500

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

static DECLARE_COMPLETION(transmit_done);

struct mxc_w1_device {
	void __iomem *regs;
	unsigned int clkdiv;
	struct clk *clk;
	struct w1_bus_master bus_master;
};

/*
 * this is the low level routine to
 * reset the device on the One Wire interface
 * on the hardware
 */
static u8 mxc_w1_ds2_reset_bus(void *data)
{
	u8 reg_val;
	unsigned int timeout_cnt = 0;
	struct mxc_w1_device *dev = data;

	__raw_writeb(0x80, (dev->regs + MXC_W1_CONTROL));

	while (1) {
		reg_val = __raw_readb(dev->regs + MXC_W1_CONTROL);

		if (((reg_val >> 7) & 0x1) == 0 ||
		    timeout_cnt > MXC_W1_RESET_TIMEOUT)
			break;
		else
			timeout_cnt++;

		udelay(100);
	}
	return (reg_val >> 7) & 0x1;
}

/*
 * this is the low level routine to read/write a bit on the One Wire
 * interface on the hardware. It does write 0 if parameter bit is set
 * to 0, otherwise a write 1/read.
 */
static u8 mxc_w1_ds2_touch_bit(void *data, u8 bit)
{
	struct mxc_w1_device *mdev = data;
	void __iomem *ctrl_addr = mdev->regs + MXC_W1_CONTROL;
	unsigned int timeout_cnt = 400; /* Takes max. 120us according to
					 * datasheet.
					 */

	__raw_writeb((1 << (5 - bit)), ctrl_addr);

	while (timeout_cnt--) {
		if (!((__raw_readb(ctrl_addr) >> (5 - bit)) & 0x1))
			break;

		udelay(1);
	}

	return ((__raw_readb(ctrl_addr)) >> 3) & 0x1;
}

static void mxc_w1_ds2_write_byte(void *data, u8 byte)
{
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	INIT_COMPLETION(transmit_done);
	__raw_writeb(byte, (dev->regs + MXC_W1_TXRX));
	__raw_writeb(0x10, (dev->regs + MXC_W1_INTERRUPT_EN));
	wait_for_completion(&transmit_done);
}
static u8 mxc_w1_ds2_read_byte(void *data)
{
	volatile u8 reg_val;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	mxc_w1_ds2_write_byte(data, 0xFF);
	reg_val = __raw_readb((dev->regs + MXC_W1_TXRX));
	return reg_val;
}
static u8 mxc_w1_read_byte(void *data)
{
	volatile u8 reg_val;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	reg_val = __raw_readb((dev->regs + MXC_W1_TXRX));
	return reg_val;
}
static irqreturn_t w1_interrupt_handler(int irq, void *data)
{
	u8 reg_val;
	irqreturn_t ret = IRQ_NONE;
	struct mxc_w1_device *dev = (struct mxc_w1_device *)data;
	reg_val = __raw_readb((dev->regs + MXC_W1_INTERRUPT));
	if ((reg_val & 0x10)) {
		complete(&transmit_done);
		reg_val = __raw_readb((dev->regs + MXC_W1_TXRX));
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
		__raw_writeb(0x80, (dev->regs + MXC_W1_CONTROL));
		mdelay(1);
		mxc_w1_ds2_write_byte(data, 0xF0);
		__raw_writeb(0x02, (dev->regs + MXC_W1_COMMAND));
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

static int __devinit mxc_w1_probe(struct platform_device *pdev)
{
	struct mxc_w1_device *mdev;
	struct mxc_w1_config *data =
	    (struct mxc_w1_config *)pdev->dev.platform_data;
	struct resource *res;
	int irq = 0;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mdev = kzalloc(sizeof(struct mxc_w1_device), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->clk = clk_get(&pdev->dev, "owire");
	if (!mdev->clk) {
		err = -ENODEV;
		goto failed_clk;
	}

	mdev->clkdiv = (clk_get_rate(mdev->clk) / 1000000) - 1;

	res = request_mem_region(res->start, resource_size(res),
				"mxc_w1");
	if (!res) {
		err = -EBUSY;
		goto failed_req;
	}

	mdev->regs = ioremap(res->start, resource_size(res));
	if (!mdev->regs) {
		printk(KERN_ERR "Cannot map frame buffer registers\n");
		goto failed_ioremap;
	}

	clk_enable(mdev->clk);
	__raw_writeb(mdev->clkdiv, mdev->regs + MXC_W1_TIME_DIVIDER);

	mdev->bus_master.data = mdev;
	mdev->bus_master.reset_bus = mxc_w1_ds2_reset_bus;
	mdev->bus_master.touch_bit = mxc_w1_ds2_touch_bit;
	if (data->search_rom_accelerator) {
		mdev->bus_master.write_byte = &mxc_w1_ds2_write_byte;
		mdev->bus_master.read_byte = &mxc_w1_ds2_read_byte;
		mdev->bus_master.search = &search_ROM_accelerator;
		irq = platform_get_irq(pdev, 0);
		if (irq < 0) {
			err = -ENOENT;
			goto failed_irq;
		}
		err = request_irq(irq, w1_interrupt_handler, 0, "mxc_w1", mdev);
		if (err) {
			pr_debug("OWire:request_irq(%d) returned error %d\n",
				 irq, err);
			goto failed_irq;
		}
	}

	err = w1_add_master_device(&mdev->bus_master);

	if (err)
		goto failed_add;

	platform_set_drvdata(pdev, mdev);
	return 0;

failed_add:
	if (irq)
		free_irq(irq, mdev);
failed_irq:
	iounmap(mdev->regs);
failed_ioremap:
	release_mem_region(res->start, resource_size(res));
failed_req:
	clk_put(mdev->clk);
failed_clk:
	kfree(mdev);
	return err;
}

/*
 * disassociate the w1 device from the driver
 */
static int __devexit mxc_w1_remove(struct platform_device *pdev)
{
	struct mxc_w1_device *mdev = platform_get_drvdata(pdev);
	struct resource *res;
	struct mxc_w1_config *data =
	    (struct mxc_w1_config *)pdev->dev.platform_data;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	w1_remove_master_device(&mdev->bus_master);

	iounmap(mdev->regs);
	release_mem_region(res->start, resource_size(res));

	irq = platform_get_irq(pdev, 0);
	if ((irq >= 0) && (data->search_rom_accelerator))
		free_irq(irq, mdev);

	clk_disable(mdev->clk);
	clk_put(mdev->clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mxc_w1_driver = {
	.driver = {
		   .name = "mxc_w1",
	},
	.probe = mxc_w1_probe,
	.remove = mxc_w1_remove,
};

static int __init mxc_w1_init(void)
{
	return platform_driver_register(&mxc_w1_driver);
}

static void mxc_w1_exit(void)
{
	platform_driver_unregister(&mxc_w1_driver);
}

module_init(mxc_w1_init);
module_exit(mxc_w1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductors Inc");
MODULE_DESCRIPTION("Driver for One-Wire on MXC");
