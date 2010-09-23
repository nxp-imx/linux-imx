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

#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>
#include "board-mx25_3stack.h"
#include "iomux.h"

/*!
 * @file mach-mx25/mx25_3stack_cpld.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX25
 */

extern int mxc_spi_poll_transfer(struct spi_device *spi,
				 struct spi_transfer *t);
static int __init mxc_expio_init(void);

struct spi_device *cpld_spi;

/*!
 * This function is used to tranfer data to CPLD regs over CSPI
 */
static inline int mx25_3ds_cpld_rw(u8 *buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf = (const void *)buf,
		.rx_buf = buf,
		.len = len,
		.cs_change = 0,
		.delay_usecs = 0,
	};

	if (!cpld_spi)
		return -1;

	mxc_spi_poll_transfer(cpld_spi, &t);
	return 0;
}

/*!
 * This function is called to read a CPLD register over CSPI.
 *
 * @param        offset    number of the cpld register to be read
 *
 * @return       Returns 0 on success -1 on failure.
 */
unsigned int spi_cpld_read(unsigned int offset)
{
	unsigned int frame[2];
	unsigned int reg_num = offset >> 1;
	unsigned int data = 0;

	frame[0] = (1 << 13) | ((reg_num & 0x0001FFFF) >> 5) | 0x00001000;
	frame[1] = (((reg_num & 0x0000001F) << 27) | 0x0200001f);
	mx25_3ds_cpld_rw((u8 *) frame, 2);
	data = (frame[1] >> 6) & 0xFFFF;

	reg_num = (offset + 2) >> 1;
	frame[0] = (1 << 13) | ((reg_num & 0x0001FFFF) >> 5) | 0x00001000;
	frame[1] = (((reg_num & 0x0000001F) << 27) | 0x0200001f);
	mx25_3ds_cpld_rw((u8 *) frame, 2);

	data |= (((frame[1] >> 6) & 0xFFFF) << 16);
	return data;
}
EXPORT_SYMBOL(spi_cpld_read);

/*!
 * This function is called to write to a CPLD register over CSPI.
 *
 * @param        offset    number of the cpld register to be written
 * @param        reg_val   value to be written
 *
 * @return       Returns 0 on success -1 on failure.
 */
unsigned int spi_cpld_write(unsigned int offset, unsigned int reg_val)
{
	unsigned int frame[2] = { 0, 0 };
	unsigned int reg_num = offset >> 1;
	unsigned int data = reg_val;

	frame[0] = ((reg_num & 0x0001FFFF) >> 5) | 0x00001000;
	frame[1] = (((reg_num & 0x0000001F) << 27) |
		    ((data & 0x0000FFFF) << 6) | 0x03C00027);
	mx25_3ds_cpld_rw((u8 *) frame, 2);

	reg_num = (offset + 2) >> 1;
	data = reg_val >> 16;
	frame[0] = 0;
	frame[1] = 0;
	frame[0] = ((reg_num & 0x0001FFFF) >> 5) | 0x00001000;
	frame[1] = (((reg_num & 0x0000001F) << 27) |
		    ((data & 0x0000FFFF) << 6) | 0x03C00027);

	mx25_3ds_cpld_rw((u8 *) frame, 2);

	return 0;
}
EXPORT_SYMBOL(spi_cpld_write);

static int __init mx25_3ds_cpld_probe(struct spi_device *spi)
{
	unsigned int i = 0;

	spi->bits_per_word = 46;
	cpld_spi = spi;

	spi_setup(spi);
	i = spi_cpld_read(CPLD_CODE_VER_REG);
	pr_info("3-Stack Debug board detected, rev = 0x%04X\n", i);
	spi_cpld_write(LED_SWITCH_REG, 0xFF);

	/* disable the interrupt and clear the status */
	spi_cpld_write(INTR_MASK_REG, 0);
	spi_cpld_write(INTR_RESET_REG, 0xFFFF);
	spi_cpld_write(INTR_RESET_REG, 0);
	spi_cpld_write(INTR_MASK_REG, 0x1E);

	mxc_expio_init();
	return 0;
}

/*!
 * This structure contains pointers to the CPLD callback functions.
 */
static struct spi_driver mx25_3ds_cpld_driver = {
	.driver = {
		   .name = "cpld_spi",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = mx25_3ds_cpld_probe,
};

static int __init mx25_3ds_cpld_init(void)
{
	pr_info("Registering the CPLD Driver\n");
	return spi_register_driver(&mx25_3ds_cpld_driver);
}
device_initcall(mx25_3ds_cpld_init);

static int __initdata is_dbg_removed = { 0 };
static int __init remove_dbg_setup(char *__unused)
{
	is_dbg_removed = 1;
	return 0;
}
__setup("remove_dbg", remove_dbg_setup);

static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 expio_irq;
	struct irq_desc *d;

	desc->chip->mask(irq);	/* irq = gpio irq number */

	expio_irq = MXC_BOARD_IRQ_START;

	d = irq_desc + expio_irq;
	if (unlikely(!(d->handle_irq))) {
		printk(KERN_ERR "\nEXPIO irq: %d unhandled\n", expio_irq);
		BUG();		/* oops */
	}
	d->handle_irq(expio_irq, d);

	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq		an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	/* clear the interrupt status */
	spi_cpld_write(INTR_RESET_REG, 1);
	spi_cpld_write(INTR_RESET_REG, 0);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq		a expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int __init mxc_expio_init(void)
{
	int i;

	if (is_dbg_removed)
		return 0;

	/*
	 * Configure INT line as GPIO input
	 */
	mxc_request_iomux(MX25_PIN_PWM, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX25_PIN_PWM, PAD_CTL_PUE_PUD);
	gpio_request(IOMUX_TO_GPIO(MX25_PIN_PWM), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(MX25_PIN_PWM));

	for (i = MXC_BOARD_IRQ_START;
	     i < (MXC_BOARD_IRQ_START + MXC_BOARD_IRQS);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(IOMUX_TO_IRQ(MX25_PIN_PWM), IRQF_TRIGGER_LOW);
	set_irq_chained_handler(IOMUX_TO_IRQ(MX25_PIN_PWM),
				mxc_expio_irq_handler);

	return 0;
}

