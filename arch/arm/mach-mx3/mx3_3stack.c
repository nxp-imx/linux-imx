/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/ata.h>
#include <linux/smsc911x.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/keypad.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/spba.h>
#include <mach/pmic_power.h>

#include "board-mx3_3stack.h"
#include "crm_regs.h"
#include "iomux.h"
/*!
 * @file mach-mx3/mx3_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX31
 */

extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_cpu_init(void) __init;
extern void mxc_cpu_common_init(void);
extern void __init early_console_setup(char *);
extern int mxc_init_devices(void);

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
static u16 keymapping[12] = {
	KEY_UP, KEY_DOWN, 0, 0,
	KEY_RIGHT, KEY_LEFT, KEY_ENTER, 0,
	KEY_F6, KEY_F8, KEY_F9, KEY_F10,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 3,
	.colmax = 4,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

/* mxc keypad driver */
static struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &keypad_plat_data,
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

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

static struct mtd_partition mxc_nand_partitions[] = {
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
	 .size = 180 * 1024 * 1024},
	{
	 .name = "nand.configure",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 8 * 1024 * 1024},
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

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
		mxc_nand_data.width = 2;
	}
	if (cpu_is_mx31()) {
		(void)platform_device_register(&mxc_nand_mtd_device);
	}
	if (cpu_is_mx32()) {
		(void)platform_device_register(&mxc_nandv2_mtd_device);
	}
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static void lcd_reset(void)
{
	/* ensure that LCDIO(1.8V) has been turn on */
	/* active reset line GPIO */
	mxc_request_iomux(MX31_PIN_LCS1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_LCS1), "lcs1");
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_LCS1), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_LCS1), 0);
	/* do reset */
	msleep(10);		/* tRES >= 100us */
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_LCS1), 1);
	msleep(60);
#ifdef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_LCS1), 0);
#endif
}

static struct mxc_lcd_platform_data lcd_data = {
	.io_reg = "VGEN",
#ifdef CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL
	.core_reg = "GPO1",
#else
	.core_reg = "VMMC1",
#endif
	.reset = lcd_reset,
};

static struct mxc_camera_platform_data camera_data = {
	.core_regulator = "VVIB",
	.io_regulator = "VMMC1",
	.analog_regulator = "SW2B",
	.gpo_regulator = "GPO3",
	.mclk = 27000000,
};

struct mxc_tvout_platform_data tvout_data = {
	.io_reg = "VGEN",
	.core_reg = "GPO3",
	.analog_reg = "GPO1",
	.detect_line = 49,
};

void si4702_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SRST0), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SRST0), 1);
	msleep(100);
}

void si4702_clock_ctl(int flag)
{
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_SIMPD0), flag);
}

static void si4702_gpio_get(void)
{
	/* reset pin */
	mxc_request_iomux(MX31_PIN_SRST0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SRST0), "srst0");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_SRST0), 0);

	mxc_request_iomux(MX31_PIN_SIMPD0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SIMPD0), "simpd0");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_SIMPD0), 0);
}

static void si4702_gpio_put(void)
{
	mxc_free_iomux(MX31_PIN_SRST0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_free_iomux(MX31_PIN_SIMPD0, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
}

static struct mxc_fm_platform_data si4702_data = {
	.reg_vio = "GPO3",
	.reg_vdd = "VMMC1",
	.gpio_get = si4702_gpio_get,
	.gpio_put = si4702_gpio_put,
	.reset = si4702_reset,
	.clock_ctl = si4702_clock_ctl,
	.sksnr = 0,
	.skcnt = 0,
	.band = 0,
	.space = 100,
	.seekth = 0xa,
};

/* setup GPIO for mma7450 */
static void gpio_mma7450_get(void)
{
	mxc_request_iomux(MX31_PIN_STX0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_iomux_set_pad(MX31_PIN_STX0, PAD_CTL_PKE_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_STX0), "stx0");
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_STX0));

	mxc_request_iomux(MX31_PIN_SRX0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_iomux_set_pad(MX31_PIN_SRX0, PAD_CTL_PKE_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_SRX0), "srx0");
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_SRX0));
}

static void gpio_mma7450_put(void)
{
	mxc_free_iomux(MX31_PIN_STX0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_free_iomux(MX31_PIN_SRX0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
}

static struct mxc_mma7450_platform_data mma7450_data = {
	.reg_dvdd_io = "GPO3",
	.reg_avdd = "VMMC1",
	.gpio_pin_get = gpio_mma7450_get,
	.gpio_pin_put = gpio_mma7450_put,
	.int1 = IOMUX_TO_IRQ(MX31_PIN_STX0),
	.int2 = IOMUX_TO_IRQ(MX31_PIN_SRX0),
};

static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
	 .type = "ov2640",
	 .addr = 0x30,
	 .platform_data = (void *)&camera_data,},
	{
	 .type = "ch7024",
	 .addr = 0x76,
	 .platform_data = (void *)&tvout_data,
	 .irq = IOMUX_TO_IRQ(MX31_PIN_BATT_LINE),},
	{
	 .type = "si4702",
	 .addr = 0x10,
	 .platform_data = (void *)&si4702_data,
	 },
	{
	 .type = "mma7450",
	 .addr = 0x1d,
	 .platform_data = (void *)&mma7450_data,
	 },
};

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "pmic_spi",
	 .irq = IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
	 .max_speed_hz = 4000000,
	 .bus_num = 2,
	 .chip_select = 2,
	},
	{
	 .modalias = "lcd_spi",
	 .platform_data = (void *)&lcd_data,
	 .max_speed_hz = 5000000,
	 .bus_num = 1,
	 .chip_select = 2,
	},
};

/*lan9217 device*/
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	{
	 .start = LAN9217_BASE_ADDR,
	 .end = LAN9217_BASE_ADDR + 0x100,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = LAN9217_IRQ,
	 .end = LAN9217_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
};

struct smsc911x_platform_config smsc911x_config = {
        .irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .flags = SMSC911X_USE_32BIT | SMSC911X_FORCE_INTERNAL_PHY,
};

static struct platform_device smsc_lan9217_device = {
	.name = "smsc911x",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &smsc911x_config,
		},
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource = smsc911x_resources,
};
static void mxc_init_enet(void)
{
	(void)platform_device_register(&smsc_lan9217_device);
}
#else
static inline void mxc_init_enet(void)
{
}
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static const char fb_default_mode[] = "Epson-VGA";

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

static struct platform_device mxc_fb_wvga_device = {
	.name = "lcd_claa",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &lcd_data,
		},
};

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
	(void)platform_device_register(&mxc_fb_wvga_device);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_BACKLIGHT_MXC)
static struct platform_device mxcbl_devices[] = {
#if defined(CONFIG_BACKLIGHT_MXC_IPU) || defined(CONFIG_BACKLIGHT_MXC_IPU_MODULE)
	{
	 .name = "mxc_ipu_bl",
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

#if defined(CONFIG_FB_MXC_TVOUT_CH7024) || \
    defined(CONFIG_FB_MXC_TVOUT_CH7024_MODULE)
static int mxc_init_ch7024(void)
{
	/* request gpio for phone jack detect */
	mxc_request_iomux(MX31_PIN_BATT_LINE, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	mxc_iomux_set_pad(MX31_PIN_BATT_LINE, PAD_CTL_PKE_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_BATT_LINE), "batt_line");
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_BATT_LINE));

	return 0;
}
#else
static inline int mxc_init_ch7024(void)
{
	return 0;
}
#endif

static u32 brd_io;

static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 imr_val;
	u32 int_valid;
	u32 expio_irq;

	desc->chip->mask(irq);	/* irq = gpio irq number */

	imr_val = __raw_readw(brd_io + INTR_MASK_REG);
	int_valid = __raw_readw(brd_io + INTR_STATUS_REG) & ~imr_val;

	if (unlikely(!int_valid)) {
		goto out;
	}

	expio_irq = MXC_BOARD_IRQ_START;
	for (; int_valid != 0; int_valid >>= 1, expio_irq++) {
		struct irq_desc *d;
		if ((int_valid & 1) == 0)
			continue;
		d = irq_desc + expio_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nEXPIO irq: %d unhandled\n",
			       expio_irq);
			BUG();	/* oops */
		}
		d->handle_irq(expio_irq, d);
	}

      out:
	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
	u16 reg;
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* mask the interrupt */
	reg = __raw_readw(brd_io + INTR_MASK_REG);
	reg |= (1 << expio);
	__raw_writew(reg, brd_io + INTR_MASK_REG);
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq		an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* clear the interrupt status */
	__raw_writew(1 << expio, brd_io + INTR_RESET_REG);
	__raw_writew(0, brd_io + INTR_RESET_REG);
	/* mask the interrupt */
	expio_mask_irq(irq);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq		a expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
	u16 reg;
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* unmask the interrupt */
	reg = __raw_readw(brd_io + INTR_MASK_REG);
	reg &= ~(1 << expio);
	__raw_writew(reg, brd_io + INTR_MASK_REG);
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int __init mxc_expio_init(void)
{
	int i;

	brd_io = (u32) ioremap(BOARD_IO_ADDR, SZ_4K);
	if (brd_io == 0)
		return -ENOMEM;

	if ((__raw_readw(brd_io + MAGIC_NUMBER1_REG) != 0xAAAA) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER2_REG) != 0x5555) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER3_REG) != 0xCAFE)) {
		iounmap((void *)brd_io);
		brd_io = 0;
		return -ENODEV;
	}

	pr_info("3-Stack Debug board detected, rev = 0x%04X\n",
		readw(brd_io + CPLD_CODE_VER_REG));

	/*
	 * Configure INT line as GPIO input
	 */
	mxc_request_iomux(MX31_PIN_GPIO1_1, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_GPIO1_1), "gpio1_1");
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_GPIO1_1));

	/* disable the interrupt and clear the status */
	__raw_writew(0, brd_io + INTR_MASK_REG);
	__raw_writew(0xFFFF, brd_io + INTR_RESET_REG);
	__raw_writew(0, brd_io + INTR_RESET_REG);
	__raw_writew(0x1F, brd_io + INTR_MASK_REG);
	for (i = MXC_BOARD_IRQ_START; i < (MXC_BOARD_IRQ_START + MXC_BOARD_IRQS); i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(EXPIO_PARENT_INT, IRQF_TRIGGER_LOW);
	set_irq_chained_handler(EXPIO_PARENT_INT, mxc_expio_irq_handler);

	return 0;
}

#if (defined(CONFIG_MXC_PMIC_MC13783) || \
	defined(CONFIG_MXC_PMIC_MC13783_MODULE)) \
	&& (defined(CONFIG_SND_MXC_PMIC) || defined(CONFIG_SND_MXC_PMIC_MODULE))
static void __init mxc_init_pmic_audio(void)
{
	struct clk *ckih_clk;
	struct clk *cko_clk;

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "cko1_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CKO1 output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	/* config Audio ports (4 & 5) */
	mxc_request_iomux(MX31_PIN_SCK4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SRXD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_STXD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SFS4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SCK5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SRXD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_STXD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SFS5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
}
#else
static void __inline mxc_init_pmic_audio(void)
{
}
#endif

/* MMC device data */

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.power_mmc = "GPO1",
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC1,
	       .end = MXC_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_fixed = 1,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.power_mmc = "VMMC2",
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC2,
	       .end = MXC_INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxcmci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc1_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name = "mxcmci",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc2_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

static inline void mxc_init_mmc(void)
{
	int cd_irq;

	cd_irq = sdhc_init_card_det(0);
	if (cd_irq) {
		mxcsdhc1_device.resource[2].start = cd_irq;
		mxcsdhc1_device.resource[2].end = cd_irq;
	}

	cd_irq = sdhc_init_card_det(1);
	if (cd_irq) {
		mxcsdhc2_device.resource[2].start = cd_irq;
		mxcsdhc2_device.resource[2].end = cd_irq;
	}

	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc1_device);

	spba_take_ownership(SPBA_SDHC2, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	mxc_cpu_init();

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

/* IDE device data */
#if defined(CONFIG_BLK_DEV_IDE_MXC) || defined(CONFIG_BLK_DEV_IDE_MXC_MODULE)

/*! Platform Data for MXC IDE */
static struct mxc_ide_platform_data mxc_ide_data = {
	.power_drive = "GPO2",
	.power_io = "GPO3",
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

#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
extern void gpio_ata_active(void);
extern void gpio_ata_inactive(void);

static int ata_init(struct platform_device *pdev)
{
	/* Configure the pins */
	gpio_ata_active();

	return 0;
}

static void ata_exit(void)
{
	/* Free the pins */
	gpio_ata_inactive();
}

static struct fsl_ata_platform_data ata_data = {
	.udma_mask = ATA_UDMA3,	/* board can handle up to UDMA3 */
	.mwdma_mask = ATA_MWDMA2,
	.pio_mask = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg = MXC_IDE_DMA_BD_NR,
	.init = ata_init,
	.exit = ata_exit,
	.core_reg = "GPO2",	/*"LDO2", */
	.io_reg = "GPO3",	/*"LDO3", */
};

static struct resource pata_fsl_resources[] = {
	[0] = {			/* I/O */
	       .start = ATA_BASE_ADDR + 0x00,
	       .end = ATA_BASE_ADDR + 0xD8,
	       .flags = IORESOURCE_MEM,
	       },
	[2] = {			/* IRQ */
	       .start = MXC_INT_ATA,
	       .end = MXC_INT_ATA,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device pata_fsl_device = {
	.name = "pata_fsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(pata_fsl_resources),
	.resource = pata_fsl_resources,
	.dev = {
		.platform_data = &ata_data,
		.coherent_dma_mask = ~0,
		},
};

static void __init mxc_init_pata(void)
{
	(void)platform_device_register(&pata_fsl_device);
}
#else				/* CONFIG_PATA_FSL */
static void __init mxc_init_pata(void)
{
}
#endif				/* CONFIG_PATA_FSL */

static void bt_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_DCD_DCE1), 1);
}

static struct mxc_bt_platform_data mxc_bt_data = {
	.bt_vdd = "VMMC2",
	.bt_vdd_parent = "GPO1",
	.bt_vusb = NULL,
	.bt_vusb_parent = "GPO3",
	.bt_reset = bt_reset,
};

static struct platform_device mxc_bt_device = {
	.name = "mxc_bt",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_bt_data,
		},
};

static void mxc_init_bluetooth(void)
{
	(void)platform_device_register(&mxc_bt_device);
}

static void mxc_unifi_hardreset(int pin_level)
{
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_DCD_DCE1), pin_level & 0x01);
}

static struct mxc_unifi_platform_data unifi_data = {
	.hardreset = mxc_unifi_hardreset,

	/* GPO3 -> enables SW2B 1.8V out - this becomes 1V8 on personality
	 * board, then 1V8_EXT, then BT_VUSB
	 */
	.reg_gpo1 = "GPO3",

	/* GPO4 -> WiFi_PWEN, but this signal is not used on current boards */
	.reg_gpo2 = "GPO4",

	.reg_1v5_ana_bb = "VRF1",	/* VRF1 -> WL_1V5ANA and WL_1V5BB */
	.reg_vdd_vpa = "VMMC2",	/* VMMC2 -> WL_VDD and WL_VPA */
	.reg_1v5_dd = "VRF2",	/* VRF2 -> WL_1V5DD */

	.host_id = 1,
};

struct mxc_unifi_platform_data *get_unifi_plat_data(void)
{
	return &unifi_data;
}

EXPORT_SYMBOL(get_unifi_plat_data);

#if defined(CONFIG_GPS_IOCTRL) || defined(CONFIG_GPS_IOCTRL_MODULE)
static struct mxc_gps_platform_data gps_data = {
	.core_reg = "GPO3",
	.analog_reg = "GPO1",
};

static struct platform_device mxc_gps_device = {
	.name = "gps_ioctrl",
	.id = -1,
	.dev = {
		.platform_data = &gps_data,
		},
};

static void __init mxc_init_gps(void)
{
	(void)platform_device_register(&mxc_gps_device);
}
#else
static void __init mxc_init_gps(void)
{
}
#endif

static void __init mx3_3stack_timer_init(void)
{
	mx31_clocks_init(26000000);
}

static struct sys_timer mxc_timer = {
	.init	= mx3_3stack_timer_init,
};

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* config CS5 for debug board */
	mxc_request_iomux(MX31_PIN_CS5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

	mxc_cpu_common_init();
	mxc_register_gpios();
	early_console_setup(saved_command_line);
	mxc_init_devices();

	/*Pull down MX31_PIN_USB_BYP to reset USB3317 */
	mxc_request_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), "usb_byp");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_USB_BYP), 0);
	mxc_free_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	/* Reset BT/WiFi chip */
	mxc_request_iomux(MX31_PIN_DCD_DCE1, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_DCD_DCE1), "dcd_dce1");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_DCD_DCE1), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX31_PIN_DCD_DCE1), 0);

	mxc_init_pmic_audio();
	mxc_expio_init();
	mxc_init_keypad();
	mxc_init_enet();
	mxc_init_nand_mtd();
	mxc_init_ch7024();
	mx3_3stack_init_mc13783();

	i2c_register_board_info(0, mxc_i2c_board_info,
				ARRAY_SIZE(mxc_i2c_board_info));
	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

	mxc_init_fb();
	mxc_init_bl();
	mxc_init_mmc();
	mxc_init_ide();
	mxc_init_pata();
	mxc_init_bluetooth();
	mxc_init_gps();
}

#define PLL_PCTL_REG(pd, mfd, mfi, mfn)		\
	((((pd) - 1) << 26) + (((mfd) - 1) << 16) + ((mfi)  << 10) + mfn)

/* For 26MHz input clock */
#define PLL_532MHZ		PLL_PCTL_REG(1, 13, 10, 3)
#define PLL_399MHZ		PLL_PCTL_REG(1, 52, 7, 35)
#define PLL_133MHZ		PLL_PCTL_REG(2, 26, 5, 3)

#define PDR0_REG(mcu, max, hsp, ipg, nfc)	\
	(MXC_CCM_PDR0_MCU_DIV_##mcu | MXC_CCM_PDR0_MAX_DIV_##max | \
	 MXC_CCM_PDR0_HSP_DIV_##hsp | MXC_CCM_PDR0_IPG_DIV_##ipg | \
	 MXC_CCM_PDR0_NFC_DIV_##nfc)

/* working point(wp): 0 - 133MHz; 1 - 266MHz; 2 - 399MHz; 3 - 532MHz */
/* 26MHz input clock table */
static struct cpu_wp cpu_wp_26[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 4;
	return cpu_wp_26;
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX3_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX31_3DS, "Freescale MX31/MX32 3-Stack Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx31_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
