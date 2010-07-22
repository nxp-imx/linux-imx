/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max17135.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/videodev2.h>
#include <linux/mxcfb.h>
#include <linux/fec.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/flash.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include "iomux.h"
#include "mx50_pins.h"
#include "devices.h"
#include "usb.h"

extern void __init mx50_arm2_io_init(void);
extern int __init mx50_arm2_init_mc13892(void);
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

static struct cpu_wp *mx50_arm2_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

static void mx50_arm2_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

extern void mx50_arm2_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void mx50_arm2_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);
static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx50_arm2_gpio_spi_chipselect_active,
	.chipselect_inactive = mx50_arm2_gpio_spi_chipselect_inactive,
};

static struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 7,
	.chipselect_active = mx50_arm2_gpio_spi_chipselect_active,
	.chipselect_inactive = mx50_arm2_gpio_spi_chipselect_inactive,
};

static struct mxc_i2c_platform_data mxci2c_data = {
	.i2c_clk = 100000,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = OCOTP_CTRL_BASE_ADDR + 0x80,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct regulator_init_data max17135_init_data[] __initdata = {
	{
		.constraints = {
			.name = "DISPLAY",
		},
	}, {
		.constraints = {
			.name = "GVDD",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "GVEE",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINN",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINP",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "VCOM",
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
	}, {
		.constraints = {
			.name = "VNEG",
			.min_uV = V_to_uV(-15),
			.max_uV = V_to_uV(-15),
		},
	}, {
		.constraints = {
			.name = "VPOS",
			.min_uV = V_to_uV(15),
			.max_uV = V_to_uV(15),
		},
	},
};

static struct max17135_platform_data max17135_pdata __initdata = {
	.vneg_pwrup = 1,
	.gvee_pwrup = 1,
	.vpos_pwrup = 2,
	.gvdd_pwrup = 1,
	.gvdd_pwrdn = 1,
	.vpos_pwrdn = 2,
	.gvee_pwrdn = 1,
	.vneg_pwrdn = 1,
	.gpio_pmic_pwrgood = IOMUX_TO_GPIO(MX50_PIN_EPDC_PWRSTAT),
	.gpio_pmic_vcom_ctrl = IOMUX_TO_GPIO(MX50_PIN_EPDC_VCOM0),
	.gpio_pmic_wakeup = IOMUX_TO_GPIO(MX50_PIN_UART4_TXD),
	.gpio_pmic_intr = IOMUX_TO_GPIO(MX50_PIN_UART4_RXD),
	.regulator_init = max17135_init_data,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "backlight-i2c",
	 .addr = 0x2c,
	 },
	{
	 .type = "eeprom",
	 .addr = 0x50,
	 },
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	 I2C_BOARD_INFO("max17135", 0x48),
	 .platform_data = &max17135_pdata,
	 },
};

static struct mtd_partition mxc_dataflash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x000100000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},
};

static struct flash_platform_data mxc_spi_flash_data[] = {
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",}
};


static struct spi_board_info mxc_dataflash_device[] __initdata = {
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 3,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],},
};

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_ECSPI2_SS0));
	else if (to_platform_device(dev)->id == 1)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_SD2_WP));
	else if (to_platform_device(dev)->id == 2)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_SD3_WP));

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_EIM_CRE));
	else if (to_platform_device(dev)->id == 1)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_SD2_CD));
	else if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_KEY_COL2));

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};

static int mxc_sgtl5000_amp_enable(int enable)
{
/* TO DO */
	return 0;
}

static int headphone_det_status(void)
{
	return (gpio_get_value(IOMUX_TO_GPIO(MX50_PIN_ECSPI1_SS0)) != 0);
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ(MX50_PIN_ECSPI1_SS0),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static void wvga_reset(void)
{
	/* ELCDIF D0 */
	mxc_free_iomux(MX50_PIN_DISP_D0, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D0, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D0, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D1 */
	mxc_free_iomux(MX50_PIN_DISP_D1, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D1, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D1, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D2 */
	mxc_free_iomux(MX50_PIN_DISP_D2, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D2, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D2, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D3 */
	mxc_free_iomux(MX50_PIN_DISP_D3, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D3, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D3, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D4 */
	mxc_free_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D4, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D4, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D5 */
	mxc_free_iomux(MX50_PIN_DISP_D5, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D5, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D5, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D6 */
	mxc_free_iomux(MX50_PIN_DISP_D6, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D6, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D6, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	/* ELCDIF D7 */
	mxc_free_iomux(MX50_PIN_DISP_D7, IOMUX_CONFIG_ALT2);
	mxc_request_iomux(MX50_PIN_DISP_D7, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX50_PIN_DISP_D7, PAD_CTL_PKE_ENABLE |
			  PAD_CTL_PUE_KEEPER |
			  PAD_CTL_ODE_OPENDRAIN_NONE |
			  PAD_CTL_DRV_HIGH);
	return;
}

static struct mxc_lcd_platform_data lcd_wvga_data = {
	.reset = wvga_reset,
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_claa",
	.dev = {
		.platform_data = &lcd_wvga_data,
		},
};

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

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
	mxc_set_cpu_type(MXC_CPU_MX50);

	get_cpu_wp = mx50_arm2_get_cpu_wp;
	set_num_cpu_wp = mx50_arm2_set_num_cpu_wp;
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* SD card detect irqs */
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_EIM_CRE);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_EIM_CRE);
	mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_SD2_CD);
	mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_SD2_CD);
	mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ(MX50_PIN_KEY_COL2);
	mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ(MX50_PIN_KEY_COL2);

	mxc_cpu_common_init();
	mxc_register_gpios();
	mx50_arm2_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxcspi3_device, &mxcspi3_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mxc_register_device(&mxc_rtc_device, &srtc_data);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&gpu_device, NULL);
	mxc_register_device(&mxc_pxp_device, NULL);
	mxc_register_device(&mxc_pxp_client_device, NULL);
	/*
	mxc_register_device(&mx53_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	*/

/*	mxc_register_device(&mxc_keypad_device, &keypad_plat_data); */

	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_fec_device, &fec_data);
	spi_register_board_info(mxc_dataflash_device,
				ARRAY_SIZE(mxc_dataflash_device));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	mxc_register_device(&epdc_device, NULL);
	mxc_register_device(&lcd_wvga_device, &lcd_wvga_data);
	mxc_register_device(&elcdif_device, &fb_data[0]);

	mx50_arm2_init_mc13892();
/*
	pm_power_off = mxc_power_off;
	*/
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mx5_usb_dr_init();
	mx5_usbh1_init();
}

static void __init mx50_arm2_timer_init(void)
{
	struct clk *uart_clk;

	mx50_clocks_init(32768, 24000000, 22579200);

	uart_clk = clk_get(NULL, "uart_clk.0");
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx50_arm2_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX50_ARM2 data structure.
 */
MACHINE_START(MX50_ARM2, "Freescale MX50 ARM2 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
