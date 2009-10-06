/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/mach/keypad.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/pmic_external.h>
#include <linux/ipu.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/spba.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>

#include "board-mx51_3stack.h"
#include "iomux.h"
#include "crm_regs.h"

/*!
 * @file mach-mx51/mx51_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */
extern void __init mx51_3stack_io_init(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

struct cpu_wp *mx51_3stack_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_3stack_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
static u16 keymapping[24] = {
	KEY_1, KEY_2, KEY_3, KEY_F1, KEY_UP, KEY_F2,
	KEY_4, KEY_5, KEY_6, KEY_LEFT, KEY_SELECT, KEY_RIGHT,
	KEY_7, KEY_8, KEY_9, KEY_F3, KEY_DOWN, KEY_F4,
	KEY_0, KEY_OK, KEY_ESC, KEY_ENTER, KEY_MENU, KEY_BACK,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 4,
	.colmax = 6,
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

/* NAND Flash Partitions */
#ifdef CONFIG_MTD_PARTITIONS

static struct mtd_partition nand_flash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 3 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.userfs1",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.userfs2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};

#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3) \
	|| defined(CONFIG_MTD_NAND_MXC_V3_MODULE)

extern void gpio_nand_active(void);
extern void gpio_nand_inactive(void);

static int nand_init(void)
{
	/* Configure the pins */
	gpio_nand_active();
	return 0;
}

static void nand_exit(void)
{
	/* Free the pins */
	gpio_nand_inactive();
}

static struct flash_platform_data mxc_nand_data = {
	#ifdef CONFIG_MTD_PARTITIONS
		.parts = nand_flash_partitions,
		.nr_parts = ARRAY_SIZE(nand_flash_partitions),
	#endif
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
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
	(void)platform_device_register(&mxc_nandv2_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

/* i.MX MTD NAND Flash Controller */

#if defined(CONFIG_MTD_NAND_IMX_NFC) || defined(CONFIG_MTD_NAND_IMX_NFC_MODULE)

/* Resources for this device. */

static struct resource imx_nfc_resources[] = {
	{
		.flags = IORESOURCE_MEM,
		.start = NFC_BASE_ADDR_AXI + 0x0000,
		.end   = NFC_BASE_ADDR_AXI + 0x1200 - 1,
		.name  = IMX_NFC_BUFFERS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = NFC_BASE_ADDR_AXI + 0x1E00,
		.end   = NFC_BASE_ADDR_AXI + 0x1E44 - 1,
		.name  = IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = NFC_BASE_ADDR + 0x00,
		.end   = NFC_BASE_ADDR + 0x34 - 1,
		.name  = IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_IRQ,
		.start = MXC_INT_NFC,
		.end   = MXC_INT_NFC,
		.name  = IMX_NFC_INTERRUPT_RES_NAME,
	},
};

/*
 * Platform-specific information about this device. Some of the details depend
 * on the SoC. See imx_init_nfc() below for code that fills in the rest.
 */

static struct imx_nfc_platform_data imx_nfc_platform_data = {
	.nfc_major_version  = 3,
	.nfc_minor_version  = 2,
	.force_ce           = false,
	.target_cycle_in_ns = 30,
	.clock_name         = "nfc_clk",
	.set_page_size      = 0,
	.interleave         = false,
	#ifdef CONFIG_MTD_PARTITIONS
		.partitions      = nand_flash_partitions,
		.partition_count = ARRAY_SIZE(nand_flash_partitions),
	#endif
};

/* The structure that represents the NFC device. */

static struct platform_device imx_nfc_device = {
	.name = IMX_NFC_DRIVER_NAME,
	.id = 0,
	.dev = {
		.release       = mxc_nop_release,
		.platform_data = &imx_nfc_platform_data,
	},
	.resource      = imx_nfc_resources,
	.num_resources = ARRAY_SIZE(imx_nfc_resources),
};

/**
 * imx_init_nfc() - Sets up the NFC for this platform.
 *
 * This function sets up data structures representing the NFC device on this
 * platform and registers the device with the platform management system.
 */

static void imx_nfc_init(void)
{
	(void)platform_device_register(&imx_nfc_device);
}

#else

static inline void imx_nfc_init(void)
{
}

#endif /* i.MX MTD NAND Flash Controller */

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_YUV444,
	 },
};

static struct platform_device mxc_fb_device[] = {
	{
	 .name = "mxc_sdc_fb",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 .platform_data = &fb_data[0],
		 },
	 },
	{
	 .name = "mxc_sdc_fb",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 .platform_data = &fb_data[1],
		 },
	 },
	{
	 .name = "mxc_sdc_fb",
	 .id = 2,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
	 },
};

static void lcd_reset_to2(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 0);
	ipu_reset_disp_panel();

	return;
}

static void lcd_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 0);
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), "ser_rs");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 0);
	/* do reset */
	msleep(10);		/* tRES >= 100us */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 1);
	msleep(60);
}

static struct mxc_lcd_platform_data lcd_data = {
	.core_reg = "VIOHI",
	.io_reg = "SW4",
	.reset = lcd_reset,
};

static struct platform_device mxc_lcd_device = {
	.name = "lcd_spi",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &lcd_data,
		},
};

static void wvga_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_D1_CS), 1);
}

static struct mxc_lcd_platform_data lcd_wvga_data = {
	.reset = wvga_reset,
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_claa",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &lcd_wvga_data,
		},
};

static void mxc_init_fb(void)
{

	if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0)
		lcd_data.reset = lcd_reset_to2;

	(void)platform_device_register(&mxc_lcd_device);
	(void)platform_device_register(&lcd_wvga_device);

	(void)platform_device_register(&mxc_fb_device[0]);
	(void)platform_device_register(&mxc_fb_device[1]);
	(void)platform_device_register(&mxc_fb_device[2]);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

static struct platform_device mxcbl_device = {
	.name = "mxc_mc13892_bl",
};

static inline void mxc_init_bl(void)
{
	platform_device_register(&mxcbl_device);
}

void si4702_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), 1);
	msleep(100);
}

void si4702_clock_ctl(int flag)
{
}

static void si4702_gpio_get(void)
{
	/* reset pin */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), "eim_dtack");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), 0);
}

static void si4702_gpio_put(void)
{
}

static struct mxc_fm_platform_data si4702_data = {
	.reg_vio = "SW4",
	.reg_vdd = "VIOHI",
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

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
};
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "wm8903-i2c",
	 .addr = 0x1a,
	 },
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "tsc2007",
	 .addr = 0x48,
	 .irq  = IOMUX_TO_IRQ(MX51_PIN_GPIO1_5),
	},
	{
	 .type = "si4702",
	 .addr = 0x10,
	 .platform_data = (void *)&si4702_data,
	 },
};
#endif
#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
static struct mxc_camera_platform_data camera_data = {
	.io_regulator = "SW4",
	.analog_regulator = "VIOHI",
	.mclk = 24000000,
	.csi = 0,
};
static struct mxc_lightsensor_platform_data ls_data = {
	.vdd_reg = NULL,
	.rext = 100,
};

static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
	{
		.type = "ov3640",
		.addr = 0x3C,
		.platform_data = (void *)&camera_data,
	},
	{
	 .type = "isl29003",
	 .addr = 0x44,
	 .platform_data = &ls_data,
	 },
};
#endif

#endif

static u32 cpld_base_addr;

/*lan9217 device*/
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	{
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = LAN9217_IRQ,
	 .end = LAN9217_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
};
static struct platform_device smsc_lan9217_device = {
	.name = "smsc911x",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource = smsc911x_resources,
};
static void mxc_init_enet(void)
{
	if (cpld_base_addr) {
		smsc_lan9217_device.resource[0].start =
		    LAN9217_BASE_ADDR(cpld_base_addr);
		smsc_lan9217_device.resource[0].end =
		    LAN9217_BASE_ADDR(cpld_base_addr) + 0x100;
		(void)platform_device_register(&smsc_lan9217_device);
	}
}
#else
static inline void mxc_init_enet(void)
{
}
#endif

#if defined(CONFIG_IMX_SIM) || defined(CONFIG_IMX_SIM_MODULE)
/* Used to configure the SIM bus */
static struct mxc_sim_platform_data sim_data = {
	.clk_rate = 4000000,
	.clock_sim = "sim_clk",
	.power_sim = NULL,
	.init = NULL,
	.exit = NULL,
	.detect = 0,
};

/*!
 * Resource definition for the SIM
 */
static struct resource mxc_sim_resources[] = {
	[0] = {
	       .start = SIM_BASE_ADDR,
	       .end = SIM_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_SIM_IPB,
	       .end = MXC_INT_SIM_IPB,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = MXC_INT_SIM_DAT,
	       .end = MXC_INT_SIM_DAT,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for IMX SIM */
static struct platform_device mxc_sim_device = {
	.name = "mxc_sim",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &sim_data,
		},
	.num_resources = ARRAY_SIZE(mxc_sim_resources),
	.resource = mxc_sim_resources,
};

static inline void mxc_init_sim(void)
{
	(void)platform_device_register(&mxc_sim_device);
}
#else
static inline void mxc_init_sim(void)
{
}
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
/*!
 * Get WP pin value to detect write protection
 */
int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1));
	else
		rc = 0;
	return rc;
}

/*
 * Probe for the card. If present the GPIO data would be set.
 */
unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
		return ret;
	} else {		/* config the det pin for SDHC2 */
		return 0;
	}
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
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
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .flags = IORESOURCE_IRQ,
	       },
};

/*!
 * Resource definition for the SDHC2
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
	.name = "mxsdhci",
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
	.name = "mxsdhci",
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
	(void)platform_device_register(&mxcsdhc1_device);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

static u32 brd_io;
static void expio_ack_irq(u32 irq);

static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 imr_val;
	u32 int_valid;
	u32 expio_irq;

	desc->chip->mask(irq);	/* irq = gpio irq number */

	imr_val = __raw_readw(brd_io + INTR_MASK_REG);
	int_valid = __raw_readw(brd_io + INTR_STATUS_REG) & ~imr_val;

	if (unlikely(!int_valid))
		goto out;

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

	brd_io = (u32) ioremap(BOARD_IO_ADDR(CS5_BASE_ADDR), SZ_4K);
	if (brd_io == 0)
		return -ENOMEM;

	if ((__raw_readw(brd_io + MAGIC_NUMBER1_REG) != 0xAAAA) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER2_REG) != 0x5555) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER3_REG) != 0xCAFE)) {
		pr_info("3-Stack Debug board not detected \n");
		cpld_base_addr = 0;
		return -ENODEV;
	} else {
		cpld_base_addr = CS5_BASE_ADDR;
	}

	pr_info("3-Stack Debug board detected, rev = 0x%04X\n",
		readw(brd_io + CPLD_CODE_VER_REG));

	/*
	 * Configure INT line as GPIO input
	 */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_6), "gpio1_6");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_6));

	/* disable the interrupt and clear the status */
	__raw_writew(0, brd_io + INTR_MASK_REG);
	__raw_writew(0xFFFF, brd_io + INTR_RESET_REG);
	__raw_writew(0, brd_io + INTR_RESET_REG);
	__raw_writew(0x1F, brd_io + INTR_MASK_REG);
	for (i = MXC_BOARD_IRQ_START; i < (MXC_BOARD_IRQ_START + MXC_BOARD_IRQS);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(EXPIO_PARENT_INT, IRQF_TRIGGER_LOW);
	set_irq_chained_handler(EXPIO_PARENT_INT, mxc_expio_irq_handler);

	return 0;
}

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
	.udma_mask = ATA_UDMA3,
	.mwdma_mask = ATA_MWDMA2,
	.pio_mask = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg = MXC_IDE_DMA_BD_NR,
	.init = ata_init,
	.exit = ata_exit,
	.core_reg = NULL,
	.io_reg = NULL,
};

static struct resource pata_fsl_resources[] = {
	[0] = {
	       .start = ATA_BASE_ADDR,
	       .end = ATA_BASE_ADDR + 0x000000C8,
	       .flags = IORESOURCE_MEM,},
	[2] = {
	       .start = MXC_INT_ATA,
	       .end = MXC_INT_ATA,
	       .flags = IORESOURCE_IRQ,},
};

static struct platform_device pata_fsl_device = {
	.name = "pata_fsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(pata_fsl_resources),
	.resource = pata_fsl_resources,
	.dev = {
		.platform_data = &ata_data,
		.coherent_dma_mask = ~0,},
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

#if defined(CONFIG_TOUCHSCREEN_TSC2007) \
	|| defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

static int __init mxc_init_touchscreen(void)
{
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), "gpio1_5");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

	return 0;
}
#else
static int __init mxc_init_touchscreen(void)
{
	return 0;
}
#endif

static int __init mxc_init_srpgconfig(void)
{
	struct clk *gpcclk = clk_get(NULL, "gpc_dvfs_clk");
	clk_enable(gpcclk);

	/* Setup the number of clock cycles to wait for SRPG
	 * power up and power down requests.
	 */
	__raw_writel(0x010F0201, MXC_SRPG_ARM_PUPSCR);
	__raw_writel(0x010F0201, MXC_SRPG_NEON_PUPSCR);
	__raw_writel(0x00000008, MXC_SRPG_EMPGC0_PUPSCR);
	__raw_writel(0x00000008, MXC_SRPG_EMPGC1_PUPSCR);

	__raw_writel(0x01010101, MXC_SRPG_ARM_PDNSCR);
	__raw_writel(0x01010101, MXC_SRPG_NEON_PDNSCR);
	__raw_writel(0x00000018, MXC_SRPG_EMPGC0_PDNSCR);
	__raw_writel(0x00000018, MXC_SRPG_EMPGC1_PDNSCR);

	clk_disable(gpcclk);
	clk_put(gpcclk);

	return 0;
}

#if defined(CONFIG_SND_SOC_IMX_3STACK_WM8903) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_WM8903_MODULE)
static struct mxc_audio_platform_data wm8903_data;

static struct platform_device mxc_wm8903_device = {
	.name = "imx-3stack-wm8903",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &wm8903_data,
		},
};

static void __init mxc_init_wm8903(void)
{
	wm8903_data.ssi_clk[0] = clk_get(NULL, "ssi_clk.0");
	clk_put(wm8903_data.ssi_clk[0]);

	wm8903_data.ssi_clk[1] = clk_get(NULL, "ssi_clk.1");
	clk_put(wm8903_data.ssi_clk[1]);

	wm8903_data.ssi_num = 1;
	wm8903_data.src_port = 2;
	wm8903_data.ext_port = 3;

	(void)platform_device_register(&mxc_wm8903_device);
}
#else
static void __init mxc_init_wm8903(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static int mxc_sgtl5000_plat_init(void);
static int mxc_sgtl5000_plat_finit(void);
static int mxc_sgtl5000_amp_enable(int enable);

int headphone_det_status(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A26));
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ(MX51_PIN_EIM_A26),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.vddio = 1800000,
	.vdda = 1800000,
	.vddd = 1200000,
	.sysclk = 12000000,
	.init = mxc_sgtl5000_plat_init,
	.finit = mxc_sgtl5000_plat_finit,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &sgtl5000_data,
		},
};

static int mxc_sgtl5000_plat_init(void)
{
	struct regulator *reg;
	reg = regulator_get(&mxc_sgtl5000_device.dev, "GPO2");
	if (IS_ERR(reg))
		return -EINVAL;
	sgtl5000_data.priv = reg;
	return 0;
}

static int mxc_sgtl5000_plat_finit(void)
{
	struct regulator *reg;
	reg = sgtl5000_data.priv;
	if (reg) {
		regulator_put(reg);
		sgtl5000_data.priv = NULL;
	}
	return 0;
}

static int mxc_sgtl5000_amp_enable(int enable)
{
	struct regulator *reg;
	reg = sgtl5000_data.priv;

	if (!reg)
		return -EINVAL;
	if (enable)
		regulator_enable(reg);
	else
		regulator_disable(reg);
	return 0;
}

static void mxc_init_sgtl5000(void)
{
	platform_device_register(&mxc_sgtl5000_device);
}
#else
static inline void mxc_init_sgtl5000(void)
{
}
#endif

static void bt_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D19), 1);
}

static struct mxc_bt_platform_data mxc_bt_data = {
	.bt_vdd = NULL,
	.bt_vdd_parent = NULL,
	.bt_vusb = "SW4",
	.bt_vusb_parent = NULL,
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
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D19), pin_level & 0x01);
}

static struct mxc_unifi_platform_data unifi_data = {
	.hardreset = mxc_unifi_hardreset,
	.reg_vdd_vpa = "VSD",
	.reg_1v5_dd = "VGEN1",
	.host_id = 1,
};

struct mxc_unifi_platform_data *get_unifi_plat_data(void)
{
	return &unifi_data;
}

EXPORT_SYMBOL(get_unifi_plat_data);

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

	get_cpu_wp = mx51_3stack_get_cpu_wp;
	set_num_cpu_wp = mx51_3stack_set_num_cpu_wp;
#ifdef CONFIG_DISCONTIGMEM
	do {
		int nid;
		mi->nr_banks = MXC_NUMNODES;
		for (nid = 0; nid < mi->nr_banks; nid++)
			SET_NODE(mi, nid);

	} while (0);
#endif
}

#if defined(CONFIG_GPS_IOCTRL) || defined(CONFIG_GPS_IOCTRL_MODULE)
static struct mxc_gps_platform_data gps_data = {
	.core_reg = "VIOHI",
	.analog_reg = "SW4",
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

int gpio_gps_access(int para)
{
	iomux_pin_name_t pin;
	pin = (para & 0x1) ? MX51_PIN_EIM_CS2 : MX51_PIN_EIM_CRE;

	if (para & 0x4) /* Read GPIO */
		return gpio_get_value(IOMUX_TO_GPIO(pin));
	else if (para & 0x2) /* Write GPIO */
		gpio_set_value(IOMUX_TO_GPIO(pin), 1);
	else
		gpio_set_value(IOMUX_TO_GPIO(pin), 0);
	return 0;
}
EXPORT_SYMBOL(gpio_gps_access);
#else
static void __init mxc_init_gps(void)
{
}
#endif

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	int err;

	mxc_cpu_common_init();
	mxc_register_gpios();
	mx51_3stack_io_init();
	early_console_setup(saved_command_line);
	mxc_init_devices();

	mxc_expio_init();
	mxc_init_enet();
	mxc_init_pata();
	mxc_init_fb();
	mxc_init_bl();
	mxc_init_keypad();
	mxc_init_nand_mtd();
	imx_nfc_init();
	mxc_init_mmc();
	mxc_init_sim();
	mxc_init_srpgconfig();
	mx51_3stack_init_mc13892();

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
#endif
#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));
#endif

#endif
	mxc_init_touchscreen();
	mxc_init_wm8903();
	mxc_init_sgtl5000();
	mxc_init_bluetooth();
	mxc_init_gps();

	err = mxc_request_iomux(MX51_PIN_EIM_D19, IOMUX_CONFIG_GPIO);
	if (err)
		printk(KERN_ERR "Error: bt reset request gpio failed!\n");
	else {
		gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D19), "eim_d19");
		gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_D19), 0);
	}
}

static void __init mx51_3stack_timer_init(void)
{
	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_3stack_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_3DS, "Freescale MX51 3-Stack Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx51_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
