/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/pmic_external.h>
#include <linux/ipu.h>
#include <linux/pwm_backlight.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/i2c.h>

#include "devices.h"
#include "iomux.h"
#include "mx51_pins.h"
#include "crm_regs.h"
#include "usb.h"

/*!
 * @file mach-mx51/mx51_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */
#define DEBUG_BOARD_BASE_ADDRESS(n)	(n)
/* LAN9217 ethernet base address */
#define LAN9217_BASE_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n))

#define BOARD_IO_ADDR(n)	(DEBUG_BOARD_BASE_ADDRESS(n) + 0x20000)
/* LED switchs */
#define LED_SWITCH_REG		0x00
/* buttons */
#define SWITCH_BUTTONS_REG	0x08
/* status, interrupt */
#define INTR_STATUS_REG	0x10
#define INTR_MASK_REG		0x38
#define INTR_RESET_REG		0x20
/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG	0x40
#define MAGIC_NUMBER2_REG	0x48
/* CPLD code version */
#define CPLD_CODE_VER_REG	0x50
/* magic word for debug CPLD */
#define MAGIC_NUMBER3_REG	0x58
/* module reset register*/
#define MODULE_RESET_REG	0x60
/* CPU ID and Personality ID */
#define MCU_BOARD_ID_REG	0x68

/* interrupts like external uart , external ethernet etc*/
#define EXPIO_PARENT_INT	IOMUX_TO_IRQ(MX51_PIN_GPIO1_6)

#define EXPIO_INT_ENET		(MXC_BOARD_IRQ_START + 0)
#define EXPIO_INT_XUART_A 	(MXC_BOARD_IRQ_START + 1)
#define EXPIO_INT_XUART_B 	(MXC_BOARD_IRQ_START + 2)
#define EXPIO_INT_BUTTON_A 	(MXC_BOARD_IRQ_START + 3)
#define EXPIO_INT_BUTTON_B 	(MXC_BOARD_IRQ_START + 4)

#define MXC_IRQ_TO_EXPIO(irq)	((irq) - MXC_BOARD_IRQ_START)

/*! This is System IRQ used by LAN9217 */
#define LAN9217_IRQ	EXPIO_INT_ENET

extern int __init mx51_3stack_init_mc13892(void);
extern void __init mx51_3stack_io_init(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;
static bool debug_board_present;

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

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static u16 keymapping[24] = {
	KEY_1, KEY_2, KEY_3, KEY_F1, KEY_UP, KEY_F2,
	KEY_4, KEY_5, KEY_6, KEY_LEFT, KEY_SELECT, KEY_RIGHT,
	KEY_7, KEY_8, KEY_9, KEY_F3, KEY_DOWN, KEY_F4,
	KEY_0, KEY_OK, KEY_ESC, KEY_ENTER, KEY_MENU, KEY_BACK,
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 4,
	.colmax = 6,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.reset = mx5_vpu_reset,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx51_3ds_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	u32 gpio;

	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_ALT0);
			mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0,
					  PAD_CTL_HYS_ENABLE |
					  PAD_CTL_PKE_ENABLE |
					  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
			break;
		case 0x2:
			gpio = IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0);
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_GPIO);
			gpio_request(gpio, "cspi1_ss0");
			gpio_direction_output(gpio, 0);
			gpio_set_value(gpio, 1 & (~status));
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx51_3ds_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_ALT0);
			mxc_request_iomux(MX51_PIN_CSPI1_SS0,
					  IOMUX_CONFIG_GPIO);
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		case 0x2:
			mxc_free_iomux(MX51_PIN_CSPI1_SS0, IOMUX_CONFIG_GPIO);
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx51_3ds_gpio_spi_chipselect_active,
	.chipselect_inactive = mx51_3ds_gpio_spi_chipselect_inactive,
};

static struct imxi2c_platform_data mxci2c_data = {
	.bitrate = 100000,
};

static struct mxc_i2c_platform_data mxci2c_hs_data = {
	.i2c_clk = 400000,
};

static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
	.dig_reg = "VDIG",
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
	.num_wp = 3,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
	.gp_reg_id = "SW1",
	.lp_reg_id = "SW2",
};

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1250000,
	.lp_low = 1250000,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};


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

/* i.MX MTD NAND Flash Controller */

#if defined(CONFIG_MTD_NAND_IMX_NFC) || defined(CONFIG_MTD_NAND_IMX_NFC_MODULE)

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

#endif /* i.MX MTD NAND Flash Controller */

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB666,
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_YUV444,
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
};

static struct platform_device mxcbl_device = {
	.name = "mxc_mc13892_bl",
};

static void si4702_reset(void)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_DTACK), 1);
	msleep(100);
}

static void si4702_clock_ctl(int flag)
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

static struct resource smsc911x_resources[] = {
	{
	 .start = CS5_BASE_ADDR,
	 .end = CS5_BASE_ADDR + SZ_4K - 1,
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
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource = smsc911x_resources,
};

static struct mxc_sim_platform_data sim_data = {
	.clk_rate = 4000000,
	.clock_sim = "sim_clk",
	.power_sim = NULL,
	.init = NULL,
	.exit = NULL,
	.detect = 0,
};

/*!
 * Get WP pin value to detect write protection
 */
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1));
	else
		rc = 0;
	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
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
		return -ENODEV;
	} else {
		debug_board_present = true;
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

static struct platform_device mxc_wm8903_device = {
	.name = "imx-3stack-wm8903",
	.id = 0,
};

static struct mxc_audio_platform_data wm8903_data;

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

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

int headphone_det_status(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A26));
}

static struct mxc_audio_platform_data sgtl5000_data;

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

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ(MX51_PIN_EIM_A26),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 12000000,
	.init = mxc_sgtl5000_plat_init,
	.finit = mxc_sgtl5000_plat_finit,
};

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
};

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
	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_3stack_get_cpu_wp;
	set_num_cpu_wp = mx51_3stack_set_num_cpu_wp;
}

static struct mxc_gps_platform_data gps_data = {
	.core_reg = "VIOHI",
	.analog_reg = "SW4",
};

static struct platform_device mxc_gps_device = {
	.name = "gps_ioctrl",
	.id = -1,
};

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

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	int err;

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "csi_mclk1");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "csi_mclk2");

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxc_cpu_common_init();
	mx51_3stack_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_hs_device, &mxci2c_hs_data);
	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, NULL);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	mxc_register_device(&mxc_iim_device, NULL);
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device,
		&mxc_pwm_backlight_data);
	mxc_register_device(&mxc_keypad_device, &keypad_plat_data);
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxc_sim_device, &sim_data);
	mxc_register_device(&pata_fsl_device, &ata_data);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);

	mxc_expio_init();
	if (debug_board_present)
		mxc_register_device(&smsc_lan9217_device, &smsc911x_config);

	if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0)
		lcd_data.reset = lcd_reset_to2;

	mxc_register_device(&mxc_lcd_device, &lcd_data);
	mxc_register_device(&lcd_wvga_device, &lcd_wvga_data);
	mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	mxc_register_device(&mxc_fb_devices[2], NULL);
	mxc_register_device(&mxcbl_device, NULL);

#if defined(CONFIG_MTD_NAND_IMX_NFC) || defined(CONFIG_MTD_NAND_IMX_NFC_MODULE)
	mxc_register_device(&imx_nfc_device, &imx_nfc_platform_data);
#else
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
#endif
	mx51_3stack_init_mc13892();

	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));

	mxc_init_wm8903();
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mxc_register_device(&mxc_bt_device, &mxc_bt_data);
	mxc_register_device(&mxc_gps_device, &gps_data);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);

	mx5_usb_dr_init();
	mx5_usbh1_init();

	/* Setup Touchscreen interrupt */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), "gpio1_5");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

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
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get(NULL, "uart_clk.0");
	early_console_setup(UART1_BASE_ADDR, uart_clk);
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
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
