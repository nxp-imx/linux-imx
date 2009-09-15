/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2006-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/serial_8250.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/keypad.h>
#include <asm/mach/time.h>
#include "gpio_mux.h"
#include "board-mx27ads.h"

/*!
 * @file mach-mx27/mx27ads.c
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX27
 */

extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_cpu_init(void) __init;
extern void mxc_cpu_common_init(void);
extern void __init early_console_setup(char *);

static char command_line[COMMAND_LINE_SIZE];
static int mxc_card_status;
int mxc_board_is_ads = 1;

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

unsigned long board_get_ckih_rate(void)
{
	if ((__raw_readw(PBC_VERSION_REG) & CKIH_27MHZ_BIT_SET) == 0) {
		return 27000000;
	}
	return 26000000;
}

#if defined(CONFIG_CS89x0) || defined(CONFIG_CS89x0_MODULE)
/*! Null terminated portlist used to probe for the CS8900A device on ISA Bus
 * Add 3 to reset the page window before probing (fixes eth probe when deployed
 * using nand_boot)
 */
unsigned int netcard_portlist[] = { CS8900A_BASE_ADDRESS + 3, 0 };

EXPORT_SYMBOL(netcard_portlist);
/*!
 * The CS8900A has 4 IRQ pins, which is software selectable, CS8900A interrupt
 * pin 0 is used for interrupt generation.
 */
unsigned int cs8900_irq_map[] = { CS8900AIRQ, 0, 0, 0 };

EXPORT_SYMBOL(cs8900_irq_map);
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
unsigned int expio_intr_fec = MXC_EXP_IO_BASE + 7;

EXPORT_SYMBOL(expio_intr_fec);
#endif

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)

/*!
 * This array is used for mapping mx27 ADS keypad scancodes to input keyboard
 * keycodes.
 */
static u16 mxckpd_keycodes[] = {
	KEY_KP9, KEY_LEFTSHIFT, KEY_0, KEY_KPASTERISK, KEY_RECORD, KEY_POWER,
	KEY_KP8, KEY_9, KEY_8, KEY_7, KEY_KP5, KEY_VOLUMEDOWN,
	KEY_KP7, KEY_6, KEY_5, KEY_4, KEY_KP4, KEY_VOLUMEUP,
	KEY_KP6, KEY_3, KEY_2, KEY_1, KEY_KP3, KEY_DOWN,
	KEY_BACK, KEY_RIGHT, KEY_ENTER, KEY_LEFT, KEY_HOME, KEY_KP2,
	KEY_END, KEY_F2, KEY_UP, KEY_F1, KEY_F4, KEY_KP1,
};

static struct keypad_data evb_6_by_6_keypad = {
	.rowmax = 6,
	.colmax = 6,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = mxckpd_keycodes,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

/* mxc keypad driver */
static struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &evb_6_by_6_keypad,
		},
};

static void mxc_init_keypad(void)
{
	(void)platform_device_register(&mxc_keypad_device);
}
#else
static inline void mxc_init_keypad(void)
{
}
#endif

/* MTD NOR flash */

#if defined(CONFIG_MTD_MXC) || defined(CONFIG_MTD_MXC_MODULE)

static struct mtd_partition mxc_nor_partitions[] = {
	{
	 .name = "Bootloader",
	 .size = 512 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "nor.Kernel",
	 .size = 2 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.userfs",
	 .size = 14 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.rootfs",
	 .size = 12 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE},
	{
	 .name = "FIS directory",
	 .size = 12 * 1024,
	 .offset = 0x01FE0000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "Redboot config",
	 .size = MTDPART_SIZ_FULL,
	 .offset = 0x01FFF000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
};

static struct flash_platform_data mxc_flash_data = {
	.map_name = "cfi_probe",
	.width = 2,
	.parts = mxc_nor_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nor_partitions),
};

static struct resource mxc_flash_resource = {
	.start = 0xc0000000,
	.end = 0xc0000000 + 0x02000000 - 1,
	.flags = IORESOURCE_MEM,

};

static struct platform_device mxc_nor_mtd_device = {
	.name = "mxc_nor_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_flash_data,
		},
	.num_resources = 1,
	.resource = &mxc_flash_resource,
};

static void mxc_init_nor_mtd(void)
{
	(void)platform_device_register(&mxc_nor_mtd_device);
}
#else
static void mxc_init_nor_mtd(void)
{
}
#endif

/* MTD NAND flash */

#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)

static struct mtd_partition mxc_nand_partitions[4] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 22 * 1024 * 1024},
	{
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static struct platform_device mxc_nand_mtd_device = {
	.name = "mxc_nand_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	(void)platform_device_register(&mxc_nand_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static const char fb_default_mode[] = "Sharp-QVGA";

/* mxc lcd driver */
static struct platform_device mxc_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &fb_default_mode,
		.coherent_dma_mask = 0xFFFFFFFF,
		},
};

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_BACKLIGHT_MXC)
static struct platform_device mxcbl_devices[] = {
#if defined(CONFIG_BACKLIGHT_MXC_PMIC) || defined(CONFIG_BACKLIGHT_MXC_PMIC_MODULE)
	{
	 .name = "mxc_pmic_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)-1,	/* DISP # for this backlight */
		 },
	 },
	{
	 .name = "mxc_pmic_bl",
	 .id = 1,
	 .dev = {
		 .platform_data = (void *)0,	/* DISP # for this backlight */
		 },
	 },
#endif
#if defined(CONFIG_BACKLIGHT_MXC_LCDC) || defined(CONFIG_BACKLIGHT_MXC_LCDC_MODULE)
	{
	 .name = "mxc_lcdc_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)3,	/* DISP # for this backlight */
		 },
	 },
#endif
};
static inline void mxc_init_bl(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mxcbl_devices); i++) {
		platform_device_register(&mxcbl_devices[i]);
	}
}
#else
static inline void mxc_init_bl(void)
{
}
#endif

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "pmic_spi",
	 .irq = IOMUX_TO_IRQ(MX27_PIN_TOUT),
	 .max_speed_hz = 4000000,
	 .bus_num = 1,
	 .chip_select = 0,
	 },
};

#if 0
#define MXC_CARD_DEBUG
#endif

static const int pbc_card_bit[4][3] = {
	/* BSTAT            IMR enable       IMR removal */
	{PBC_BSTAT_SD2_DET, PBC_INTR_SD2_EN, PBC_INTR_SD2_R_EN},
	{PBC_BSTAT_SD3_DET, PBC_INTR_SD3_EN, PBC_INTR_SD3_R_EN},
	{PBC_BSTAT_MS_DET, PBC_INTR_MS_EN, PBC_INTR_MS_R_EN},
	{PBC_BSTAT_SD1_DET, PBC_INTR_SD1_EN, PBC_INTR_SD1_R_EN},
};

/*!
 * Check if a SD card has been inserted or not.
 *
 * @param  num		a card number as defined in \b enum \b mxc_card_no
 * @return 0 if a card is not present; non-zero otherwise.
 */
int mxc_card_detected(enum mxc_card_no num)
{
	u32 status;

	status = __raw_readw(PBC_BSTAT1_REG);
	return ((status & MXC_BSTAT_BIT(num)) == 0);
}

/*
 * Check if there is any state change by reading the IMR register and the
 * previous and current states of the board status register (offset 0x28).
 * A state change is defined to be card insertion OR removal. So the driver
 * may have to call the mxc_card_detected() function to see if it is card
 * insertion or removal.
 *
 * @param  mask		current IMR value
 * @param  s0		previous status register value (offset 0x28)
 * @param  s1		current status register value (offset 0x28)
 *
 * @return 0 if no card status change OR the corresponding bits in the IMR
 *           (passed in as 'mask') is NOT set.
 *         A non-zero value indicates some card state changes. For example,
 *         0b0001 means SD3 has a card state change (bit0 is set) AND its
 *               associated insertion or removal bits in IMR is SET.
 *         0b0100 means SD1 has a card state change (bit2 is set) AND its
 *               associated insertion or removal bits in IMR is SET.
 *         0b1001 means both MS and SD3 have state changes
 */
static u32 mxc_card_state_changed(u32 mask, u32 s0, u32 s1)
{
	u32 i, retval = 0;
	u32 stat = (s0 ^ s1) & 0x7800;

	if (stat == 0)
		return 0;

	for (i = MXC_CARD_MIN; i <= MXC_CARD_MAX; i++) {
		if ((stat & pbc_card_bit[i][0]) != 0 &&
		    (mask & (pbc_card_bit[i][1] | pbc_card_bit[i][2])) != 0) {
			retval |= 1 << i;
		}
	}
#ifdef MXC_CARD_DEBUG
	printk(KERN_INFO "\nmask=%x, s0=%x, s1=%x\n", mask, s0, s1);
	printk(KERN_INFO "retval=%x, stat=%x\n", retval, stat);
#endif
	return retval;
}

/*!
 * Interrupt handler for the expio (CPLD) to deal with interrupts from
 * FEC, external UART, CS8900 Ethernet and SD cards, etc.
 */
static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 imr, card_int, i;
	u32 int_valid;
	u32 expio_irq;
	u32 stat = __raw_readw(PBC_BSTAT1_REG);

	desc->chip->mask(irq);	/* irq = gpio irq number */

	imr = __raw_readw(PBC_INTMASK_SET_REG);

	card_int = mxc_card_state_changed(imr, mxc_card_status, stat);
	mxc_card_status = stat;

	if (card_int != 0) {
		for (i = MXC_CARD_MIN; i <= MXC_CARD_MAX; i++) {
			if ((card_int & (1 << i)) != 0) {
				pr_debug("card no %d state changed\n", i);
			}
		}
	}

	/* Bits defined in PBC_INTSTATUS_REG at 0x2C */
	int_valid = __raw_readw(PBC_INTSTATUS_REG) & imr;
	/*  combined with the card interrupt valid information */
	int_valid = (int_valid & 0x0F8E) | (card_int << PBC_INTR_SD2_EN_BIT);

	if (unlikely(!int_valid)) {
		pr_debug("\nEXPIO: Spurious interrupt:0x%0x\n\n", int_valid);
		pr_debug("CPLD IMR(0x38)=0x%x, BSTAT1(0x28)=0x%x\n", imr, stat);
		goto out;
	}

	expio_irq = MXC_EXP_IO_BASE;
	for (; int_valid != 0; int_valid >>= 1, expio_irq++) {
		struct irq_desc *d;
		if ((int_valid & 1) == 0)
			continue;
		d = irq_desc + expio_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nEXPIO irq: %d unhandeled\n",
			       expio_irq);
			BUG();	/* oops */
		}
		d->handle_irq(expio_irq, d);
	}

      out:
	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

#ifdef MXC_CARD_DEBUG

static irqreturn_t mxc_sd_test_handler(int irq, void *desc)
{
	int s = -1;

	printk(KERN_INFO "%s(irq=%d) for ", __FUNCTION__, irq);
	if (irq == EXPIO_INT_SD1_EN) {
		printk(KERN_INFO "SD1");
		s = MXC_CARD_SD1;
	} else if (irq == EXPIO_INT_SD2_EN) {
		printk(KERN_INFO "SD2");
		s = MXC_CARD_SD2;
	} else if (irq == EXPIO_INT_SD3_EN) {
		printk(KERN_INFO "SD3");
		s = MXC_CARD_SD3;
	} else if (irq == EXPIO_INT_MS_EN) {
		printk(KERN_INFO "MS");
		s = MXC_CARD_MS;
	} else {
		printk(KERN_INFO "None!!!!");
	}
	if (mxc_card_detected(s)) {
		printk(KERN_INFO " inserted\n");
	} else {
		printk(KERN_INFO " removed\n");
	}

	return IRQ_HANDLED;
}
#endif				/* MXC_CARD_DEBUG */

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);

	/* mask the interrupt */
	if (irq < EXPIO_INT_SD2_EN) {
		__raw_writew(1 << expio, PBC_INTMASK_CLEAR_REG);
	} else {
		irq -= EXPIO_INT_SD2_EN;
		/* clear both SDx_EN and SDx_R_EN bits */
		__raw_writew((pbc_card_bit[irq][1] | pbc_card_bit[irq][2]),
			     PBC_INTMASK_CLEAR_REG);
	}
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq		an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* clear the interrupt status */
	__raw_writew(1 << expio, PBC_INTSTATUS_REG);
	/* mask the interrupt */
	expio_mask_irq(irq);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);

	/* unmask the interrupt */
	if (irq < EXPIO_INT_SD2_EN) {
		if (irq == EXPIO_INT_XUART_INTA) {
			/* Set 8250 MCR register bit 3 - Forces the INT (A-B
			 * outputs to the active mode and sets OP2 to logic 0.
			 * This is needed to avoid spurious int caused by the
			 * internal CPLD pull-up for the interrupt pin.
			 */
			u16 val = __raw_readw(MXC_LL_EXTUART_VADDR + 8);
			__raw_writew(val | 0x8, MXC_LL_EXTUART_VADDR + 8);
		}
		__raw_writew(1 << expio, PBC_INTMASK_SET_REG);
	} else {
		irq -= EXPIO_INT_SD2_EN;

		if (mxc_card_detected(irq)) {
			__raw_writew(pbc_card_bit[irq][2], PBC_INTMASK_SET_REG);
		} else {
			__raw_writew(pbc_card_bit[irq][1], PBC_INTMASK_SET_REG);
		}
	}
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int __init mxc_expio_init(void)
{
	int i, ver;

	ver = (__raw_readw(PBC_VERSION_REG) >> 8) & 0xFF;
	if ((ver & 0x80) != 0) {
		pr_info("MX27 ADS EXPIO(CPLD) hardware\n");
		pr_info("CPLD version: 0x%x\n", ver);
	} else {
		mxc_board_is_ads = 0;
		ver &= 0x0F;
		pr_info("MX27 EVB EXPIO(CPLD) hardware\n");
		if (ver == 0xF || ver <= MXC_CPLD_VER_1_50)
			pr_info("Wrong CPLD version: %d\n", ver);
		else {
			pr_info("CPLD version: %d\n", ver);
		}
	}

	mxc_card_status = __raw_readw(PBC_BSTAT1_REG);

#ifdef MXC_CARD_DEBUG
	for (i = MXC_CARD_MIN; i <= MXC_CARD_MAX; i++) {
		if (mxc_card_detected(i)) {
			pr_info("Card %d is detected\n", 3 - i);
		}
	}
#endif
	/*
	 * Configure INT line as GPIO input
	 */
	gpio_config_mux(MX27_PIN_TIN, GPIO_MUX_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_TIN), NULL);
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_TIN));

	/* disable the interrupt and clear the status */
	__raw_writew(0xFFFF, PBC_INTMASK_CLEAR_REG);
	__raw_writew(0xFFFF, PBC_INTSTATUS_REG);

	for (i = MXC_EXP_IO_BASE; i < (MXC_EXP_IO_BASE + MXC_MAX_EXP_IO_LINES);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(EXPIO_PARENT_INT, IRQF_TRIGGER_HIGH);
	set_irq_chained_handler(EXPIO_PARENT_INT, mxc_expio_irq_handler);

	return 0;
}

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)

/*!
 * The serial port definition structure. The fields contain:
 * {UART, CLK, PORT, IRQ, FLAGS}
 */
static struct plat_serial8250_port serial_platform_data[] = {
	{
	 .membase = (void __iomem *)(CS4_BASE_ADDR_VIRT + 0x20000),
	 .mapbase = (unsigned long)(CS4_BASE_ADDR + 0x20000),
	 .irq = EXPIO_INT_XUART_INTA,
	 .uartclk = 3686400,
	 .regshift = 1,
	 .iotype = UPIO_MEM,
	 .flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST | UPF_AUTO_IRQ,
	 /*.pm = serial_platform_pm, */
	 },
	{},
};

/*!
 * REVISIT: document me
 */
static struct platform_device serial_device = {
	.name = "serial8250",
	.id = 0,
	.dev = {
		.platform_data = &serial_platform_data[0],
		},
};

/*!
 * REVISIT: document me
 */
static int __init mxc_init_extuart(void)
{
	int value;
	/*reset ext uart in cpld */
	__raw_writew(PBC_BCTRL1_URST, PBC_BCTRL1_SET_REG);
	/*delay some time for reset finish */
	for (value = 0; value < 1000; value++) ;
	__raw_writew(PBC_BCTRL1_URST, PBC_BCTRL1_CLEAR_REG);
	return platform_device_register(&serial_device);
}
#else
static inline int mxc_init_extuart(void)
{
	return 0;
}
#endif

#if (defined(CONFIG_MXC_PMIC_MC13783) || \
	defined(CONFIG_MXC_PMIC_MC13783_MODULE)) \
	&& (defined(CONFIG_SND_MXC_PMIC) || defined(CONFIG_SND_MXC_PMIC_MODULE))
extern void gpio_ssi_active(int ssi_num);

static void __init mxc_init_pmic_audio(void)
{
	struct clk *ckih_clk;
	struct clk *cko_clk;

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "clko_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CLKO output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	gpio_ssi_active(0);
	gpio_ssi_active(1);
}
#else
static void __inline mxc_init_pmic_audio(void)
{
}
#endif

/* IDE device data */
#if defined(CONFIG_BLK_DEV_IDE_MXC) || defined(CONFIG_BLK_DEV_IDE_MXC_MODULE)

/*! Platform Data for MXC IDE */
static struct mxc_ide_platform_data mxc_ide_data = {
	.power_drive = NULL,
	.power_io = NULL,
};

static struct platform_device mxc_ide_device = {
	.name = "mxc_ide",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ide_data,
		},
};

static inline void mxc_init_ide(void)
{
	if (platform_device_register(&mxc_ide_device) < 0)
		printk(KERN_ERR "Error: Registering the ide.\n");
}
#else
static inline void mxc_init_ide(void)
{
}
#endif

static __init void mxc_board_init(void)
{
	pr_info("AIPI VA base: 0x%x\n", IO_ADDRESS(AIPI_BASE_ADDR));
	mxc_cpu_common_init();
	early_console_setup(saved_command_line);
	mxc_register_gpios();
	mxc_expio_init();
	mxc_init_keypad();
	mxc_init_nor_mtd();
	mxc_init_nand_mtd();
	mxc_init_extuart();
	mxc_init_pmic_audio();
#ifdef MXC_CARD_DEBUG
	request_irq(EXPIO_INT_SD1_EN, mxc_sd_test_handler, 0, "SD_card1", NULL);
	request_irq(EXPIO_INT_SD2_EN, mxc_sd_test_handler, 0, "SD_card2", NULL);
	request_irq(EXPIO_INT_SD3_EN, mxc_sd_test_handler, 0, "SD_card3", NULL);
	request_irq(EXPIO_INT_MS_EN, mxc_sd_test_handler, 0, "MS_card", NULL);
#endif

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

	mxc_init_fb();
	mxc_init_bl();
	mxc_init_ide();
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
#ifdef CONFIG_KGDB_8250
	int i;
	for (i = 0;
	     i <
	     (sizeof(serial_platform_data) / sizeof(serial_platform_data[0]));
	     i += 1)
		kgdb8250_add_platform_port(i, &serial_platform_data[i]);
#endif

	mxc_cpu_init();
	/* Store command line for use on mxc_board_init */
	strcpy(command_line, *cmdline);

#ifdef CONFIG_DISCONTIGMEM
	do {
		int nid;
		mi->nr_banks = MXC_NUMNODES;
		for (nid = 0; nid < mi->nr_banks; nid++) {
			SET_NODE(mi, nid);
		}
	} while (0);
#endif
}

EXPORT_SYMBOL(mxc_card_detected);
EXPORT_SYMBOL(mxc_board_is_ads);

static void __init mx27ads_timer_init(void)
{
	mxc_clocks_init(32768, 0, board_get_ckih_rate(), 0);
	mxc_timer_init("gpt_clk.0");
}

static struct sys_timer mxc_timer = {
	.init = mx27ads_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX27ADS data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX27ADS, "Freescale i.MX27ADS")
	/* maintainer: Freescale Semiconductor, Inc. */
#ifdef CONFIG_SERIAL_8250_CONSOLE
	.phys_io        = CS4_BASE_ADDR,
	.io_pg_offst    = ((CS4_BASE_ADDR_VIRT) >> 18) & 0xfffc,
#else
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
#endif
	.boot_params    = PHYS_OFFSET + 0x100,
	.fixup          = fixup_mxc_board,
	.map_io         = mxc_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &mxc_timer,
MACHINE_END
