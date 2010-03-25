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
#include <linux/spi/flash.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include "board-mx53_evk.h"
#include "iomux.h"
#include "mx53_pins.h"
#include "crm_regs.h"
#include "devices.h"
#include "usb.h"

/*!
 * @file mach-mx53/mx53_evk.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */
extern void __init mx53_evk_io_init(void);
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

static struct fb_videomode video_modes[] = {
	{
	 /* 720p60 TV output */
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT |
			FB_SYNC_EXT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* MITSUBISHI LVDS panel */
	 "XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

struct cpu_wp *mx53_evk_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx53_evk_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.reset = mx5_vpu_reset,
};

extern void mx53_evk_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void mx53_evk_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);
static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx53_evk_gpio_spi_chipselect_active,
	.chipselect_inactive = mx53_evk_gpio_spi_chipselect_inactive,
};

static struct mxc_i2c_platform_data mxci2c_data = {
	.i2c_clk = 100000,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = 0x83F98840,
};

static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "800x480M-16@55",
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "1024x768M-16@60",
	 },
};

static int __initdata enable_vga = { 0 };
static int __initdata enable_tv = { 0 };

static void wvga_reset(void)
{
}

static struct mxc_lcd_platform_data lcd_wvga_data = {
	.reset = wvga_reset,
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_claa",
};

static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_evk())
		return 0;

	/* by default, fb0 is wvga, fb1 is vga or tv */
	if (enable_vga) {
		printk(KERN_INFO "VGA monitor is primary\n");
	} else if (enable_tv == 2)
		printk(KERN_INFO "HDTV is primary\n");
	else
		printk(KERN_INFO "WVGA LCD panel is primary\n");

	if (enable_tv) {
		printk(KERN_INFO "HDTV is specified as %d\n", enable_tv);
		fb_data[1].interface_pix_fmt = IPU_PIX_FMT_YUV444;
		fb_data[1].mode = &(video_modes[0]);
	}

	/* Once a customer knows the platform configuration,
	   this should be simplified to what is desired.
	 */
	if (enable_vga || enable_tv == 2) {
		/*
		 * DI1 -> DP-BG channel:
		 *
		 *    dev    di-out-fmt    default-videmode
		 *
		 * 1. VGA       RGB 	   1024x768M-16@60
		 * 2. TVE       YUV	   video_modes[0]
		 */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
		if (fb_data[0].mode_str || fb_data[0].mode)
			/*
			 * DI0 -> DC channel:
			 *
			 *    dev    di-out-fmt    default-videmode
			 *
			 * 1. WVGA      RGB 	   800x480M-16@55
			 */
			mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		/*
		 * DI0 -> DP-BG channel:
		 *
		 *    dev    di-out-fmt    default-videmode
		 *
		 * 1. WVGA      RGB 	   800x480M-16@55
		 */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
		if (fb_data[1].mode_str || fb_data[1].mode)
			/*
			 * DI1 -> DC channel:
			 *
			 *    dev    di-out-fmt    default-videmode
			 *
			 * 1. VGA       RGB 	   1024x768M-16@60
			 * 2. TVE       YUV	   video_modes[0]
			 */
			mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static int __init vga_setup(char *__unused)
{
	enable_vga = 1;
	return 1;
}
__setup("vga", vga_setup);

static int __init tv_setup(char *s)
{
	enable_tv = 1;
	if (strcmp(s, "2") == 0 || strcmp(s, "=2") == 0)
		enable_tv = 2;
	return 1;
}
__setup("hdtv", tv_setup);

static struct mxc_camera_platform_data camera_data = {
	.io_regulator = "SW4",
	.analog_regulator = "VIOHI",
	.mclk = 24000000,
	.csi = 0,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "ov3640",
	.addr = 0x3C,
	.platform_data = (void *)&camera_data,
	 },
};

/* TO DO add platform data */
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "tsc2007",
	 .addr = 0x48,
	 .irq  = IOMUX_TO_IRQ(MX53_PIN_EIM_A25),
	},
	{
	 .type = "backlight-i2c",
	 .addr = 0x2c,
	 },
	{
	 .type = "vga-ddc",
	 .addr = 0x1f,
	 },
	{
	 .type = "eeprom",
	 .addr = 0x50,
	 },
};

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (!board_is_mx53_arm2()) {
		if (to_platform_device(dev)->id == 0)
			rc = gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_EIM_DA14));
		else
			rc = gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_EIM_DA12));
	}

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	if (board_is_mx53_arm2()) {
		if (to_platform_device(dev)->id == 0)
			ret = gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_GPIO_1));
		else
			ret = 1;
	} else {
		if (to_platform_device(dev)->id == 0) {
			ret = gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_EIM_DA13));
		} else{		/* config the det pin for SDHC3 */
			ret = gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_EIM_DA11));
			}
	}

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
	return (gpio_get_value(IOMUX_TO_GPIO(MX53_PIN_ATA_DATA5)) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = IOMUX_TO_IRQ(MX53_PIN_ATA_DATA5),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.init = mxc_sgtl5000_init,
};

static int mxc_sgtl5000_init(void)
{
	struct clk *ssi_ext1;
	int rate;

	if (board_is_mx53_arm2()) {
		sgtl5000_data.sysclk = 12000000;
	} else {
		ssi_ext1 = clk_get(NULL, "ssi_ext1_clk");
		if (IS_ERR(ssi_ext1))
			return -1;

		rate = clk_round_rate(ssi_ext1, 12000000);
		if (rate < 8000000 || rate > 27000000) {
			printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
			       rate);
			clk_put(ssi_ext1);
			return -1;
		}

		clk_set_rate(ssi_ext1, rate);
		clk_enable(ssi_ext1);
		sgtl5000_data.sysclk = rate;
	}

	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
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
	char *str;
	int size = 0;
	unsigned long orig_size;
	struct tag *t;

	mxc_set_cpu_type(MXC_CPU_MX53);

	get_cpu_wp = mx53_evk_get_cpu_wp;
	set_num_cpu_wp = mx53_evk_set_num_cpu_wp;

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_CMDLINE)
			continue;
		str = t->u.cmdline.cmdline;
		str = strstr(str, "mem=");
		if (str != NULL) {
			str += 4;
			size = memparse(str, &str);
			if (size == 0 || size == SZ_512M)
				return;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_MEM)
			continue;

		orig_size = t->u.mem.size;
		if (!size)
			size = t->u.mem.size - SZ_32M;
		t->u.mem.size = size;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		mxcfb_resources[0].start = t->u.mem.start + size;
		mxcfb_resources[0].end = t->u.mem.start + orig_size - 1;
		break;
#endif
	}
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");

	/* SD card detect irqs */
	if (board_is_mx53_arm2()) {
		mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX53_PIN_GPIO_1);
		mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX53_PIN_GPIO_1);
		mmc3_data.card_inserted_state = 1;
		mmc3_data.status = NULL;
		mmc3_data.wp_status = NULL;
		mmc1_data.wp_status = NULL;
	} else {
		mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ(MX53_PIN_EIM_DA11);
		mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ(MX53_PIN_EIM_DA11);
		mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX53_PIN_EIM_DA13);
		mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX53_PIN_EIM_DA13);
	}

	mxc_cpu_common_init();
	mxc_register_gpios();
	mx53_evk_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mxc_register_device(&mxc_rtc_device, &srtc_data);
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&lcd_wvga_device, &lcd_wvga_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, NULL);
	/*
	mxc_register_device(&mx53_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, NULL);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	*/
	mxc_register_device(&mxc_iim_device, NULL);
	if (!board_is_mx53_arm2()) {
		mxc_register_device(&mxc_pwm2_device, NULL);
		mxc_register_device(&mxc_pwm_backlight_device, &mxc_pwm_backlight_data);
	}
/*	mxc_register_device(&mxc_keypad_device, &keypad_plat_data); */

	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	/*
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	*/
	mxc_register_device(&mxc_fec_device, NULL);
/*
	spi_register_board_info(mxc_spi_nor_device,
					ARRAY_SIZE(mxc_spi_nor_device));
*/

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	mx53_evk_init_mc13892();
/*
	pm_power_off = mxc_power_off;
	*/
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
	mx5_usb_dr_init();
	mx5_usbh1_init();
}

static void __init mx53_evk_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get(NULL, "uart_clk.0");
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_evk_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_EVK data structure.
 */
MACHINE_START(MX53_EVK, "Freescale MX53 EVK Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
