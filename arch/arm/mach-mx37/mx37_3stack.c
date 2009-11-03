/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/smsc911x.h>
#include <linux/i2c/tsc2007.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

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
#include "board-mx37_3stack.h"
#include "iomux.h"
#include "crm_regs.h"

/*!
 * @file mach-mx37/mx37_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX37
 */
extern void gpio_lcd_active(void);
extern void mxc_init_srpgconfig(void);

/* working point(wp): 0 - 532MHz; 1 - 200MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdf = 0,
	 .mfi = 5,
	 .mfd = 23,
	 .mfn = 13,
	 .cpu_voltage = 1050000,},
	{
	 .pll_rate = 200000000,
	 .cpu_rate = 200000000,
	 .pdf = 3,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_voltage = 850000,},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 2;
	return cpu_wp_auto;
}

#if defined(CONFIG_REGULATOR_MC13892) \
    || defined(CONFIG_REGULATOR_MC13892_MODULE)
static int mc13892_reg_int(void)
{
	int i = 0;
	unsigned int value;
	struct regulator *regulator;
	struct cpu_wp *cpu_wp_tbl1;
	int cpu_wp_nr1;
	char *reg_name[] = {
		"SW1",
		"SW2",
		"SW3",
		"SW4",
		"SW1_STBY",
		"SW2_STBY",
		"SW3_STBY",
		"SW4_STBY",
		"SW1_DVS",
		"SW2_DVS",
		"SWBST",
		"VIOHI",
		"VPLL",
		"VDIG",
		"VSD",
		"VUSB2",
		"VVIDEO",
		"VAUDIO",
		"VCAM",
		"VGEN1",
		"VGEN2",
		"VGEN3",
		"USB",
		"GPO1",
		"GPO2",
		"GPO3",
		"GPO4",
	};

	/* for board v1.1 do nothing */
	if (!board_is_rev(BOARD_REV_2))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(reg_name); i++) {
		regulator = regulator_get(NULL, reg_name[i]);
		if (regulator != ERR_PTR(-ENOENT)) {
			regulator_enable(regulator);
			regulator_put(regulator);
		}
	}
	for (i = 0; i < ARRAY_SIZE(reg_name); i++) {
		if ((strcmp(reg_name[i], "VIOHI") == 0) ||
		    (strcmp(reg_name[i], "VPLL") == 0) ||
		    (strcmp(reg_name[i], "VDIG") == 0) ||
		    (strcmp(reg_name[i], "VGEN2") == 0))
			continue;
		regulator = regulator_get(NULL, reg_name[i]);
		if (regulator != ERR_PTR(-ENOENT)) {
			regulator_disable(regulator);
			regulator_put(regulator);
		}
	}

	/* Set the current working point. */
	cpu_wp_tbl1 = get_cpu_wp(&cpu_wp_nr1);
	for (i = 0; i < cpu_wp_nr1; i++)
		cpu_wp_tbl1[i].cpu_voltage += 50000;

	/* Bit 4 DRM: keep VSRTC and CLK32KMCU on for all states */
	pmic_read_reg(REG_POWER_CTL0, &value, 0xffffff);
	value |= 0x000010;
	pmic_write_reg(REG_POWER_CTL0, value, 0xffffff);

	return 0;
}

late_initcall(mc13892_reg_int);
#endif

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3)

static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 2 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 4 * 1024 * 1024},
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
	 .size = 512 * 1024 * 1024},
	{
	 .name = "nand.userfs3",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
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
//      if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
//              mxc_nand_data.width = 2;
//      }
	(void)platform_device_register(&mxc_nandv2_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static void lcd_reset(void)
{
	static int first;

	/* ensure that LCDIO(1.8V) has been turn on */
	/* active reset line GPIO */
	if (!first) {
		mxc_request_iomux(MX37_PIN_GPIO1_5, IOMUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(MX37_PIN_GPIO1_5), "gpio1_5");
		first = 1;
	}
	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_GPIO1_5), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_GPIO1_5), 0);
	/* do reset */
	msleep(10);		/* tRES >= 100us */
	gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_GPIO1_5), 1);
	msleep(60);
}

static struct mxc_lcd_platform_data lcd_data = {
	.core_reg = "VVIDEO",
	.io_reg = "SW4",
	.reset = lcd_reset,
};

#if defined(CONFIG_KEYBOARD_MPR084) || defined(CONFIG_KEYBOARD_MPR084_MODULE)
/*!
 * These functions are used to configure and the GPIO pins for keypad to
 * activate and deactivate it.
 */
extern void gpio_keypad_active(void);

extern void gpio_keypad_inactive(void);

static u16 keymap[] = {
	KEY_DOWN, KEY_LEFT, KEY_ENTER,
	KEY_RIGHT, KEY_UP, KEY_LEFTALT,
	KEY_TAB, KEY_ESC,
};

static struct mxc_keyp_platform_data keypad_data = {
	.matrix = keymap,
	.active = gpio_keypad_active,
	.inactive = gpio_keypad_inactive,
	.vdd_reg = "VGEN2",
};
#else

static struct mxc_keyp_platform_data keypad_data = {};

#endif

static struct mxc_lightsensor_platform_data ls_data = {
	.vdd_reg = "VGEN2",
	.rext = 100,
};

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
static int tsc2007_get_pendown_state(void)
{
        return !gpio_get_value(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXFS));
}

static int tsc2007_init(void)
{
	int pad_val;

	mxc_request_iomux(MX37_PIN_AUD5_RXFS, IOMUX_CONFIG_GPIO);
	pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU;
	mxc_iomux_set_pad(MX37_PIN_AUD5_RXFS, pad_val);
	gpio_request(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXFS), "aud5_rxfs");
	gpio_direction_input(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXFS));
	return 0;
}

static void tsc2007_exit(void)
{
}

struct tsc2007_platform_data tsc2007_data = {
	.model = 2007,
	.x_plate_ohms = 400,
	.get_pendown_state = tsc2007_get_pendown_state,
	.init_platform_hw = tsc2007_init,
	.exit_platform_hw = tsc2007_exit,
};
#else
struct tsc2007_platform_data tsc2007_data;
#endif

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	 .type = "tsc2007",
	 .addr = 0x48,
	 .irq = IOMUX_TO_IRQ(MX37_PIN_AUD5_RXFS),
	 .platform_data = &tsc2007_data,
	 },
	{
	 .type = "mpr084",
	 .addr = 0x5D,
	 .platform_data = &keypad_data,
	 .irq = IOMUX_TO_IRQ(MX37_PIN_GPIO1_3),
	 },
	{
	 .type = "isl29003",
	 .addr = 0x44,
	 .platform_data = &ls_data,
	 },
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "mc13892",
	 .addr = 0x08,
	 .platform_data = (void *)MX37_PIN_OWIRE_LINE,
	 },
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
};

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "cpld_spi",
	 .max_speed_hz = 27000000,
	 .bus_num = 2,
	 .chip_select = 0,
	 },
	{
	 .modalias = "lcd_spi",
	 .max_speed_hz = 5000000,
	 .bus_num = 2,
	 .platform_data = &lcd_data,
	 .chip_select = 1,},
};

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static struct platform_device mxc_fb_device[] = {
	{
	 .name = "mxc_sdc_fb",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
	 },
	{
	 .name = "mxc_sdc_fb",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
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

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device[0]);
	(void)platform_device_register(&mxc_fb_device[1]);
	(void)platform_device_register(&mxc_fb_device[2]);
	gpio_lcd_active();
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

/*lan9217 device*/
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	{
	 .start = LAN9217_BASE_ADDR,
	 .end = LAN9217_BASE_ADDR + 255,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_BOARD_IRQ_START,
	 .flags = IORESOURCE_IRQ,
	 },
};

struct smsc911x_platform_config smsc911x_config = {
        .irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .flags = 0x8000 | SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
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

static int __init mxc_init_enet(void)
{
	(void)platform_device_register(&smsc_lan9217_device);
	return 0;
}
#else
static int __init mxc_init_enet(void)
{
	return 0;
}
#endif

late_initcall(mxc_init_enet);

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
	.core_reg = NULL,	/*"LDO2", */
	.io_reg = NULL,		/*"LDO3", */
};

static struct resource pata_fsl_resources[] = {
	[0] = {			/* I/O */
	       .start = ATA_BASE_ADDR,
	       .end = ATA_BASE_ADDR + 0x000000C8,
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

static void mxc_unifi_hardreset(int pin_level)
{
	struct regulator *gpo4;

	if (board_is_rev(BOARD_REV_2)) {
		gpo4 = regulator_get(NULL, "GPO4");
		if (!IS_ERR(gpo4)) {
			if (pin_level & 0x01)
				regulator_enable(gpo4);
			else
				regulator_disable(gpo4);
		}
		regulator_put(gpo4);
	} else {
		mxc_request_iomux(MX37_PIN_AUD5_RXC, IOMUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC), "aud5_rxc");
		gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC),
			       pin_level & 0x01);
		gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC), 0);
		mxc_free_iomux(MX37_PIN_AUD5_RXC, IOMUX_CONFIG_GPIO);
	}
}

static struct mxc_unifi_platform_data unifi_data = {
	.hardreset = mxc_unifi_hardreset,
	.enable = NULL,
	.reg_1v5_ana_bb = "VGEN1",
	.reg_vdd_vpa = "VCAM",
	.reg_1v5_dd = "VGEN1",
	.host_id = 1,
};

struct mxc_unifi_platform_data *get_unifi_plat_data(void)
{
	return &unifi_data;
}

EXPORT_SYMBOL(get_unifi_plat_data);

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
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
	int cd_irq;

	cd_irq = sdhc_init_card_det(0);
	if (cd_irq) {
		mxcsdhc1_device.resource[2].start = cd_irq;
		mxcsdhc1_device.resource[2].end = cd_irq;
	}

	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc1_device);
	cd_irq = sdhc_init_card_det(1);
	if (cd_irq) {
		mxcsdhc2_device.resource[2].start = cd_irq;
		mxcsdhc2_device.resource[2].end = cd_irq;
	}
	spba_take_ownership(SPBA_SDHC2, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

static void bt_reset(void)
{
	struct regulator *gpo4;
	if (board_is_rev(BOARD_REV_2)) {
		gpo4 = regulator_get(NULL, "GPO4");
		if (!IS_ERR(gpo4))
			regulator_enable(gpo4);
		regulator_put(gpo4);
	} else {
		mxc_request_iomux(MX37_PIN_AUD5_RXC, IOMUX_CONFIG_GPIO);
		gpio_request(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC), "aud5_rxc");
		gpio_set_value(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC), 1);
		gpio_direction_output(IOMUX_TO_GPIO(MX37_PIN_AUD5_RXC), 0);
	}
}

static struct mxc_bt_platform_data mxc_bt_data = {
	.bt_vdd = "VGEN2",
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

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static int mxc_sgtl5000_plat_init(void);
static int mxc_sgtl5000_plat_finit(void);
static int mxc_sgtl5000_amp_enable(int enable);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = IOMUX_TO_IRQ(MX37_PIN_AUD5_RXFS),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
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
	int err, pin;
	struct clk *cko1, *parent;
	unsigned long rate;

	/* for board v1.1 do nothing */
	if (!board_is_rev(BOARD_REV_2))
		return;

	pin = MX37_PIN_AUD5_RXFS;
	err = mxc_request_iomux(pin, IOMUX_CONFIG_GPIO);
	if (err) {
		sgtl5000_data.hp_irq = -1;
		printk(KERN_ERR "Error: sgtl5000_init request gpio failed!\n");
		return;
	}
	mxc_iomux_set_pad(pin, PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU);
	gpio_request(IOMUX_TO_GPIO(pin), "aud5_rxfs");
	gpio_direction_input(IOMUX_TO_GPIO(pin));

	/* cko1 clock */
	mxc_request_iomux(MX37_PIN_GPIO1_6, IOMUX_CONFIG_ALT2);

	cko1 = clk_get(NULL, "cko1_clk");
	if (IS_ERR(cko1))
		return;
	parent = clk_get(NULL, "ipg_perclk");
	if (IS_ERR(parent))
		return;
	clk_set_parent(cko1, parent);
	rate = clk_round_rate(cko1, 13000000);
	if (rate < 8000000 || rate > 27000000) {
		printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
		       rate);
		clk_put(parent);
		clk_put(cko1);
		return;
	}
	clk_set_rate(cko1, rate);
	clk_enable(cko1);
	sgtl5000_data.sysclk = rate;
	platform_device_register(&mxc_sgtl5000_device);
}
#else
static inline void mxc_init_sgtl5000(void)
{
}
#endif

/*!
 * fixup for mx37 3stack board v1.1(wm8350)
 */
static void mx37_3stack_fixup_for_board_v1(void)
{
	dptc_gp_data.reg_id = "DCDC1";
	dptc_lp_data.reg_id = "DCDC4";
	gp_reg_id = "DCDC1";
	lp_reg_id = "DCDC4";
	tve_data.dac_reg = "LDO2";
	tve_data.dig_reg = "LDO3";
	lcd_data.core_reg = "LDO1";
	lcd_data.io_reg = "DCDC6";
	dvfs_core_data.reg_id = "DCDC1";
	ls_data.vdd_reg = "DCDC3";
	mxc_bt_data.bt_vdd = "DCDC3";
	mxc_bt_data.bt_vusb = "DCDC6";

	unifi_data.reg_1v5_ana_bb = NULL;	/* VMAIN is used on v1 board */
	unifi_data.reg_vdd_vpa = NULL;
	unifi_data.reg_1v5_dd = NULL;
#if defined(CONFIG_KEYBOARD_MPR084) || defined(CONFIG_KEYBOARD_MPR084_MODULE)
	keypad_data.vdd_reg = "DCDC3";
#endif
}

#if defined(CONFIG_GPS_IOCTRL) || defined(CONFIG_GPS_IOCTRL_MODULE)
static struct mxc_gps_platform_data gps_data = {
	.core_reg = "VIOHI",
	.analog_reg = "SW3",
};

static struct platform_device mxc_gps_device = {
	.name = "gps_ioctrl",
	.id = 0,
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

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	mxc_init_srpgconfig();
	mxc_register_gpios();
	early_console_setup(saved_command_line);
	mxc_init_devices();
	if (!board_is_rev(BOARD_REV_2))
		mx37_3stack_fixup_for_board_v1();
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));
	mxc_init_nand_mtd();
	mxc_init_mmc();
	mxc_init_pata();
	mxc_init_fb();
	mxc_init_bl();
	mxc_init_bluetooth();
	mxc_init_gps();
	mxc_init_sgtl5000();
}

static void __init mx37_3stack_timer_init(void)
{
	mx37_clocks_init(32768, 24000000, 22579200, 0);
}

static struct sys_timer mxc_timer = {
	.init	= mx37_3stack_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX37_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX37_3DS, "Freescale MX37 3-Stack Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx37_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
