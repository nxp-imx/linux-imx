/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/ata.h>
#include <linux/pmic_external.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <linux/regulator/consumer.h>
#include <linux/smsc911x.h>
#include <linux/i2c/tsc2007.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

#include <asm/mach/flash.h>
#endif

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/common.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>

#include "board-mx35_3stack.h"
#include "crm_regs.h"
#include "iomux.h"

/*!
 * @file mach-mx35/mx35_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX35
 */

unsigned int mx35_3stack_board_io;

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

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
	 .size = 4 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.userfs",
	 .size = 30 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0},
	{
	 .name = "nor.rootfs",
	 .size = 28 * 1024 * 1024,
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
	.start = 0xa0000000,
	.end = 0xa0000000 + 0x04000000 - 1,
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

#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)	\
|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
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
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B)
		mxc_nand_data.width = 2;

	platform_device_register(&mxc_nand_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static struct mxc_lcd_platform_data lcd_data = {
	.io_reg = "LCD"
};

static struct platform_device lcd_dev = {
	.name = "lcd_claa",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = (void *)&lcd_data,
		},
};

static void mxc_init_lcd(void)
{
	platform_device_register(&lcd_dev);
}

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
/* mxc lcd driver */
static struct platform_device mxc_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
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
#if defined(CONFIG_BACKLIGHT_MXC_IPU) || defined(CONFIG_BACKLIGHT_MXC_IPU_MODULE)
	{
	 .name = "mxc_ipu_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)3,	/* DISP # for this backlight */
		 },
	 }
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

#if defined(CONFIG_MXC_MLB) || defined(CONFIG_MXC_MLB_MODULE)
static struct resource mlb_resource[] = {
	[0] = {
	       .start = MLB_BASE_ADDR,
	       .end = MLB_BASE_ADDR + 0x300,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MLB,
	       .end = MXC_INT_MLB,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct mxc_mlb_platform_data mlb_data = {
	.buf_address = IRAM_BASE_ADDR_VIRT + MLB_IRAM_ADDR_OFFSET,
	.phy_address = IRAM_BASE_ADDR + MLB_IRAM_ADDR_OFFSET,
	.reg_nvcc = "VVIDEO",
	.mlb_clk = "mlb_clk",
};

static struct platform_device mlb_dev = {
	.name = "mxc_mlb",
	.id = 0,
	.dev = {
		.platform_data = &mlb_data,
		},
	.num_resources = ARRAY_SIZE(mlb_resource),
	.resource = mlb_resource,
};

static inline void mxc_init_mlb(void)
{
	platform_device_register(&mlb_dev);
}
#else
static inline void mxc_init_mlb(void)
{
}
#endif

static void mxc_unifi_hardreset(int pin_level)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 1, pin_level & 0x01);
}

static void mxc_unifi_enable(int en)
{
	if (en) {
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 1);
		msleep(10);
	} else
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 5, 0);
}

static struct mxc_unifi_platform_data unifi_data = {
	.hardreset = mxc_unifi_hardreset,
	.enable = mxc_unifi_enable,
	.reg_gpo1 = "GPO2",
	.reg_gpo2 = "GPO3",
	.reg_1v5_ana_bb = "PWGT1",
	.reg_vdd_vpa = "VAUDIO",
	.reg_1v5_dd = "SW1",
	.host_id = 1,
};

struct mxc_unifi_platform_data *get_unifi_plat_data(void)
{
	return &unifi_data;
}

EXPORT_SYMBOL(get_unifi_plat_data);

static int tsc2007_get_pendown_state(void)
{
        return !gpio_get_value(IOMUX_TO_GPIO(MX35_PIN_CAPTURE));
}

static int tsc2007_init(void)
{
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

static struct mxc_camera_platform_data camera_data = {
	.core_regulator = "SW1",
	.io_regulator = "VAUDIO",
	.analog_regulator = NULL,
	.gpo_regulator = "PWGT1",
	.mclk = 27000000,
};

void si4702_reset(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 4, 0);
	msleep(100);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 4, 1);
	msleep(100);
}

void si4702_clock_ctl(int flag)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 7, flag);
}

static void si4702_gpio_get(void)
{
}

static void si4702_gpio_put(void)
{
}

static struct mxc_fm_platform_data si4702_data = {
	.reg_vio = "VSD",
	.reg_vdd = NULL,
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

static void adv7180_pwdn(int pwdn)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 1, pwdn);
}

static void adv7180_reset(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 6, 0);
	msleep(5);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 6, 1);
	msleep(5);
}

static struct mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg = NULL,
	.dvdd_reg = "SW3",
	.avdd_reg = "PWGT2",
	.pvdd_reg = NULL,
	.pwdn = adv7180_pwdn,
	.reset = adv7180_reset,
};

static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
	 .type = "max8660",
	 .addr = 0x34,
	 },
	{
	 .type = "tsc2007",
	 .addr = 0x48,
	 .platform_data = &tsc2007_data,
	 .irq = IOMUX_TO_IRQ(MX35_PIN_CAPTURE),
	 },
	{
	 .type = "ov2640",
	 .addr = 0x30,
	 .platform_data = (void *)&camera_data,
	 },
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "ak4647-i2c",
	 .addr = 0x12,
	 },
#if defined(CONFIG_I2C_SLAVE_CLIENT)
	{
	 .type = "i2c-slave-client",
	 .addr = 0x55,
	 },
#endif
	{
	 .type = "si4702",
	 .addr = 0x10,
	 .platform_data = (void *)&si4702_data,
	 },
	{
	 .type = "adv7180",
	 .addr = 0x21,
	 .platform_data = (void *)&adv7180_data,
	 },
};

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "wm8580_spi",
	 .max_speed_hz = 8000000,	/* max spi SCK clock speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 },
};

#if  defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
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
	 }
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
	platform_device_register(&smsc_lan9217_device);
}
#else
static inline void mxc_init_enet(void)
{
}
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
extern void gpio_fec_active(void);
extern void gpio_fec_inactive(void);
static int fec_enable(void);
static int fec_disable(void);
static struct resource mxc_fec_resources[] = {
	{
		.start	= MXC_FEC_BASE_ADDR,
		.end	= MXC_FEC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ
	},
};

static struct fec_platform_data mxc_fec_data = {
	.init = fec_enable,
	.uninit = fec_disable,
};

struct platform_device mxc_fec_device = {
	.name = "fec",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_fec_data,
	},
	.num_resources = ARRAY_SIZE(mxc_fec_resources),
	.resource = mxc_fec_resources,
};

static int fec_enable(void)
{
	mxc_fec_data.vddio_reg = regulator_get(&mxc_fec_device.dev, "VGEN1");

	if (IS_ERR(mxc_fec_data.vddio_reg))
		return -EINVAL;
	regulator_enable(mxc_fec_data.vddio_reg);
	gpio_fec_active();
	return 0;
}

static int fec_disable(void)
{
	if (IS_ERR(mxc_fec_data.vddio_reg))
		return -EINVAL;

	gpio_fec_inactive();
	regulator_disable(mxc_fec_data.vddio_reg);
	regulator_put(mxc_fec_data.vddio_reg);
	return 0;
}

static __init int mxc_init_fec(void)
{
	return platform_device_register(&mxc_fec_device);
}
#else
static inline int mxc_init_fec(void)
{
	return 0;
}
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
	.caps = MMC_CAP_4_BIT_DATA,
#else
	.caps = MMC_CAP_8_BIT_DATA,
#endif
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "sdhc_clk",
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
	       .start = MX35_INT_MMC_SDHC1,
	       .end = MX35_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = MXC_PSEUDO_IRQ_SD1_CD,
	       .end = MXC_PSEUDO_IRQ_SD1_CD,
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

#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "sdhc_clk",
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
#endif

static inline void mxc_init_mmc(void)
{
	(void)platform_device_register(&mxcsdhc1_device);
#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
	(void)platform_device_register(&mxcsdhc2_device);
#endif
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#ifdef CONFIG_MXC_PSEUDO_IRQS
/*! Device Definition for MXC SDHC1 */
static struct platform_device mxc_pseudo_irq_device = {
	.name = "mxc_pseudo_irq",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static inline int mxc_init_pseudo_irq(void)
{
	return platform_device_register(&mxc_pseudo_irq_device);
}

late_initcall(mxc_init_pseudo_irq);

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pr_info(KERN_INFO "on-off key pressed\n");
	return 0;
}

/*!
 * Power Key initialization.
 */
static int __init mxc_init_power_key(void)
{
	if (!board_is_rev(BOARD_REV_2)) {
		/*Set power key as wakeup resource */
		int irq, ret;
		irq = MXC_PSEUDO_IRQ_POWER_KEY;
		set_irq_type(irq, IRQF_TRIGGER_RISING);
		ret = request_irq(irq, power_key_int, 0, "power_key", 0);
		if (ret)
			pr_info("register on-off key interrupt failed\n");
		else
			enable_irq_wake(irq);
		return ret;
	}
	return 0;
}

late_initcall(mxc_init_power_key);
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
	.adma_flag = 1,		/* 0:smart dma, 1:ADMA */
	.udma_mask = 0x3F,
	.mwdma_mask = 0x1F,
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

#if defined(CONFIG_GPS_IOCTRL) || defined(CONFIG_GPS_IOCTRL_MODULE)
static struct mxc_gps_platform_data gps_data = {
	.core_reg = "SW3",
	.analog_reg = "PWGT2",
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
		for (nid = 0; nid < mi->nr_banks; nid++)
			SET_NODE(mi, nid);
	} while (0);
#endif
}

static void bt_reset(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 2, 0);
	msleep(5);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 2, 1);
}

static struct mxc_bt_platform_data mxc_bt_data = {
	.bt_vdd = "GPO2",
	.bt_vdd_parent = NULL,
	.bt_vusb = NULL,
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

static int sgtl5000_headphone_det_status(void)
{
	int ret = 0;
	if (0 != pmic_gpio_get_designation_bit_val(0, &ret))
		printk(KERN_ERR "Get headphone status error.");
	return ret;
}

static int mxc_sgtl5000_plat_init(void);
static int mxc_sgtl5000_plat_finit(void);
static int mxc_sgtl5000_amp_enable(int enable);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 1,
	.ext_port = 4,
	.hp_irq = MXC_PSEUDO_IRQ_HEADPHONE,
	.hp_status = sgtl5000_headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
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
	reg = regulator_get(&mxc_sgtl5000_device.dev, "SPKR");
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
	struct clk *cko1, *parent;
	unsigned long rate;

	/* for board v1.1 do nothing */
	if (!board_is_rev(BOARD_REV_2))
		return;

	cko1 = clk_get(NULL, "cko1_clk");
	if (IS_ERR(cko1))
		return;
	parent = clk_get(NULL, "ckih");
	if (IS_ERR(parent))
		return;
	clk_set_parent(cko1, parent);
	rate = clk_round_rate(cko1, 12000000);
	if (rate < 8000000 || rate > 27000000) {
		printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
		       (unsigned int)rate);
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
static void mxc_init_sgtl5000(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_AK4647) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_AK4647_MODULE)
static int mxc_ak4647_amp_enable(int enable)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 0, enable);
	return 0;
}

static int mxc_ak4647_plat_init(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_2, 1, 0);
	msleep(1);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_2, 1, 1);
	return 0;
}

static int ak4647_headphone_det_status(void)
{
	int ret = 0;
	if (0 != pmic_gpio_get_designation_bit_val(0, &ret))
		printk(KERN_ERR "Get headphone status error.");
	return ret;
}

static struct mxc_audio_platform_data mxc_ak4647_data = {
	.ssi_num = 1,
	.src_port = 1,
	.ext_port = 4,
	.amp_enable = mxc_ak4647_amp_enable,
	.init = mxc_ak4647_plat_init,
	.hp_status = ak4647_headphone_det_status,
	.intr_id_hp = MXC_PSEUDO_IRQ_HEADPHONE,
};

static struct platform_device mxc_alsa_device = {
	.name = "imx-3stack-ak4647",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ak4647_data,
		},

};

static void mxc_init_ak4647(void)
{
	platform_device_register(&mxc_alsa_device);
}
#else
static void mxc_init_ak4647(void)
{
}
#endif

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
static void flexcan_xcvr_enable(int id, int en)
{
	static int pwdn;

	if (id < 0 || id > 1)
		return;

	if (en) {
		if (!(pwdn++))
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,
					      1, 0);
	} else {
		if (!(--pwdn))
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,
					      1, 1);
	}
}

struct flexcan_platform_data flexcan_data[] = {
	{
	 .core_reg = "GPO2",
	 .io_reg = NULL,
	 .xcvr_enable = flexcan_xcvr_enable,
	 .active = gpio_can_active,
	 .inactive = gpio_can_inactive,},
	{
	 .core_reg = "GPO2",
	 .io_reg = NULL,
	 .xcvr_enable = flexcan_xcvr_enable,
	 .active = gpio_can_active,
	 .inactive = gpio_can_inactive,},
};
#endif

/*!
 * fixup for mx35 3stack board v1.0 (MAX8660)
 */
static void mx35_3stack_fixup_for_board_v1(void)
{
#if defined(CONFIG_MXC_MLB) || defined(CONFIG_MXC_MLB_MODULE)
	mlb_data.reg_nvcc = "LDO6";
#endif

#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
	unifi_data.reg_gpo1 = NULL;
	unifi_data.reg_gpo2 = NULL;
	unifi_data.reg_1v5_ana_bb = "SW4";
	unifi_data.reg_vdd_vpa = "SW1";
	unifi_data.reg_1v5_dd = "SW4";
#endif
	camera_data.analog_regulator = "LDO7";
	camera_data.core_regulator = NULL;
	camera_data.io_regulator = NULL;
	camera_data.gpo_regulator = NULL;
	camera_data.mclk = 20000000;

	adv7180_data.dvddio_reg = NULL;
	adv7180_data.dvdd_reg = NULL;
	adv7180_data.avdd_reg = NULL;
	adv7180_data.pvdd_reg = NULL;

	si4702_data.reg_vio = "SW1";
	si4702_data.reg_vdd = NULL;

#if defined(CONFIG_GPS_IOCTRL) || defined(CONFIG_GPS_IOCTRL_MODULE)
	gps_data.core_reg = "SW1";
	gps_data.analog_reg = "SW2";
#endif

	mxc_bt_data.bt_vdd = "SW1";

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
	flexcan_data[0].core_reg = "SW1";
	flexcan_data[1].core_reg = "SW1";
#endif
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();

	early_console_setup(saved_command_line);
	mxc_register_gpios();
	mxc_init_devices();
	if (!board_is_rev(BOARD_REV_2))
		mx35_3stack_fixup_for_board_v1();
	mx35_3stack_gpio_init();
	mxc_init_enet();
	mxc_init_nor_mtd();
	mxc_init_nand_mtd();

	mx35_3stack_init_mc13892();
	mx35_3stack_init_mc9s08dz60();
	mxc_init_lcd();
	mxc_init_fb();
	mxc_init_bl();
	mxc_init_sgtl5000();
	mxc_init_ak4647();

	i2c_register_board_info(0, mxc_i2c_board_info,
				ARRAY_SIZE(mxc_i2c_board_info));

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));
	mxc_init_mmc();
	mxc_init_pata();
	mxc_init_bluetooth();
	mxc_init_gps();
	mxc_init_mlb();
	mxc_init_fec();
}

#define PLL_PCTL_REG(brmo, pd, mfd, mfi, mfn)		\
		(((brmo) << 31) + (((pd) - 1) << 26) + (((mfd) - 1) << 16) + \
		((mfi)  << 10) + mfn)

/* For 24MHz input clock */
#define PLL_665MHZ		PLL_PCTL_REG(1, 1, 48, 13, 41)
#define PLL_532MHZ		PLL_PCTL_REG(1, 1, 12, 11, 1)
#define PLL_399MHZ		PLL_PCTL_REG(0, 1, 16, 8, 5)

/* working point(wp): 0,1 - 133MHz; 2,3 - 266MHz; 4,5 - 399MHz;*/
/* auto input clock table */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x2 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x1 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x5 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x0 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_AUTO_MUX_DIV_OFFSET),},
};

/* consumer input clock table */
static struct cpu_wp cpu_wp_con[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0x6 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = (0xE << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0x2 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = (0xA << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x1 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = (0x9 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = (0x0 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = (0x8 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
	{
	 .pll_reg = PLL_665MHZ,
	 .pll_rate = 665000000,
	 .cpu_rate = 665000000,
	 .pdr0_reg = (0x7 << MXC_CCM_PDR0_CON_MUX_DIV_OFFSET),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	if (cpu_is_mx35_rev(CHIP_REV_2_0) >= 1) {
		*wp = 9;
		return cpu_wp_con;
	} else {
		if (__raw_readl(MXC_CCM_PDR0) & MXC_CCM_PDR0_AUTO_CON) {
			*wp = 9;
			return cpu_wp_con;
		} else {
			*wp = 6;
			return cpu_wp_auto;
		}
	}
}

static void __init mx35_3stack_timer_init(void)
{
	mx35_clocks_init();
}

static struct sys_timer mxc_timer = {
	.init = mx35_3stack_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX35_3DS data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX35_3DS, "Freescale MX35 3-Stack Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx35_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
