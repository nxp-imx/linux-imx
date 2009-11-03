/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/spba.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include "board-mx51_babbage.h"
#include "iomux.h"
#include "crm_regs.h"
#include <mach/mxc_edid.h>

/*!
 * @file mach-mx51/mx51_babbage.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */
extern void __init mx51_babbage_io_init(void);
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
	 "720P60", 60, 1280, 720, 7418,
	 220, 110,
	 20, 5,
	 40, 5,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT | FB_SYNC_EXT,
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

struct cpu_wp *mx51_babbage_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_babbage_set_num_cpu_wp(int num)
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

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "1024x768M-16@60",
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "1024x768M-16@60",
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
	 .num_resources = ARRAY_SIZE(mxcfb_resources),
	 .resource = mxcfb_resources,
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

static int __initdata enable_vga = { 0 };
static int __initdata enable_wvga = { 0 };
static int __initdata enable_tv = { 0 };
static int __initdata enable_mitsubishi_xga = { 0 };

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

static int handle_edid(int *pixclk)
{
#if 0
	int err = 0;
	int dvi = 0;
	int fb0 = 0;
	int fb1 = 1;
	struct fb_var_screeninfo screeninfo;
	struct i2c_adapter *adp;

	memset(&screeninfo, 0, sizeof(screeninfo));

	adp = i2c_get_adapter(1);

	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0) {
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 1);
		msleep(1);
	}
	err = read_edid(adp, &screeninfo, &dvi);
	if (cpu_is_mx51_rev(CHIP_REV_3_0) > 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_HSYNC), 0);

	if (!err) {
		printk(KERN_INFO " EDID read\n");
		if (!dvi) {
			enable_vga = 1;
			fb0 = 1; /* fb0 will be VGA */
			fb1 = 0; /* fb1 will be DVI or TV */
		}

		/* Handle TV modes */
		/* This logic is fairly complex yet still doesn't handle all
		   possibilities.  Once a customer knows the platform
		   configuration, this should be simplified to what is desired.
		 */
		if (screeninfo.xres == 1920 && screeninfo.yres != 1200) {
			/* MX51 can't handle clock speeds for anything larger.*/
			if (!enable_tv)
				enable_tv = 1;
			if (enable_vga || enable_wvga || enable_tv == 2)
				enable_tv = 2;
			fb_data[0].mode = &(video_modes[0]);
			if (!enable_wvga)
				fb_data[1].mode_str = "800x600M-16@60";
		} else if (screeninfo.xres > 1280 && screeninfo.yres > 1024) {
			if (!enable_wvga) {
				fb_data[fb0].mode_str = "1280x1024M-16@60";
				fb_data[fb1].mode_str = NULL;
			} else {
				/* WVGA is preset so the DVI can't be > this. */
				fb_data[0].mode_str = "1024x768M-16@60";
			}
		} else if (screeninfo.xres > 0 && screeninfo.yres > 0) {
			if (!enable_wvga) {
				fb_data[fb0].mode =
					kzalloc(sizeof(struct fb_videomode),
							GFP_KERNEL);
				fb_var_to_videomode(fb_data[fb0].mode,
						    &screeninfo);
				fb_data[fb0].mode_str = NULL;
				if (screeninfo.xres >= 1280 &&
						screeninfo.yres > 720)
					fb_data[fb1].mode_str = NULL;
				else if (screeninfo.xres > 1024 &&
						screeninfo.yres > 768)
					fb_data[fb1].mode_str =
						"800x600M-16@60";
				else if (screeninfo.xres > 800 &&
						screeninfo.yres > 600)
					fb_data[fb1].mode_str =
						"1024x768M-16@60";
			} else {
				/* A WVGA panel was specified and an EDID was
				   read thus there is a DVI monitor attached. */
				if (screeninfo.xres >= 1024)
					fb_data[0].mode_str = "1024x768M-16@60";
				else if (screeninfo.xres >= 800)
					fb_data[0].mode_str = "800x600M-16@60";
				else
					fb_data[0].mode_str = "640x480M-16@60";
			}
		}
	}
#endif
	return 0;
}

static int __init mxc_init_fb(void)
{
	int pixclk = 0;

	if (!machine_is_mx51_babbage())
		return 0;

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 1) {
		enable_vga = 1;
		fb_data[0].mode_str = NULL;
		fb_data[1].mode_str = NULL;
	}

	if (enable_wvga) {
		fb_data[1].interface_pix_fmt = IPU_PIX_FMT_RGB565;
		fb_data[1].mode_str = "800x480M-16@55";
	}

	if (enable_mitsubishi_xga) {
		fb_data[0].interface_pix_fmt = IPU_PIX_FMT_LVDS666;
		fb_data[0].mode = &(video_modes[1]);

		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), 0);
		msleep(1);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_D0_CS), 1);

		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D12), 1);
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSI2_D13), 1);
	}

	/* DVI Detect */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_NANDF_D12), "nandf_d12");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_NANDF_D12));
	/* DVI Reset - Assert for i2c disabled mode */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), "dispb2_ser_din");
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	/* DVI Power-down */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIO), "dispb2_ser_di0");
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIO), 1);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIO), 0);

	(void)platform_device_register(&lcd_wvga_device);

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2)
		handle_edid(&pixclk);

	if (enable_vga)
		printk(KERN_INFO "VGA monitor is primary\n");
	else if (enable_wvga)
		printk(KERN_INFO "WVGA LCD panel is primary\n");
	else if (!enable_tv)
		printk(KERN_INFO "DVI monitor is primary\n");

	if (enable_tv) {
		printk(KERN_INFO "TV is specified as %d\n", enable_tv);
		if (!fb_data[0].mode) {
			fb_data[0].mode = &(video_modes[0]);
			if (!enable_wvga)
				fb_data[1].mode_str = "800x600M-16@60";
		}
	}

	if (enable_tv) {
		struct clk *clk, *di_clk;
		clk = clk_get(NULL, "pll3");
		di_clk = clk_get(NULL, "ipu_di0_clk");
		clk_disable(clk);
		clk_disable(di_clk);
		clk_set_rate(clk, 297000000);
		clk_set_rate(di_clk, 297000000 / 4);
		clk_enable(clk);
		clk_enable(di_clk);
		clk_put(di_clk);
		clk_put(clk);
	}

	/* Once a customer knows the platform configuration,
	   this should be simplified to what is desired.
	 */
	if (enable_vga || enable_wvga || enable_tv == 2) {
		(void)platform_device_register(&mxc_fb_device[1]); /* VGA */
		if (fb_data[0].mode_str || fb_data[0].mode)
			(void)platform_device_register(&mxc_fb_device[0]);
	} else {
		(void)platform_device_register(&mxc_fb_device[0]); /* DVI */
		if (fb_data[1].mode_str || fb_data[1].mode)
			(void)platform_device_register(&mxc_fb_device[1]);
	}

	(void)platform_device_register(&mxc_fb_device[2]);

	return 0;
}
device_initcall(mxc_init_fb);

static int __init vga_setup(char *__unused)
{
	enable_vga = 1;
	return 1;
}

__setup("vga", vga_setup);

static int __init wvga_setup(char *__unused)
{
	enable_wvga = 1;
	return 1;
}

__setup("wvga", wvga_setup);

static int __init mitsubishi_xga_setup(char *__unused)
{
	enable_mitsubishi_xga = 1;
	return 1;
}

__setup("mitsubishi_xga", mitsubishi_xga_setup);

static int __init tv_setup(char *s)
{
	enable_tv = 1;
	if (strcmp(s, "2") == 0 || strcmp(s, "=2") == 0)
		enable_tv = 2;
	return 1;
}

__setup("tv", tv_setup);
#else
static inline void mxc_init_fb(void)
{
}
#endif

static void dvi_reset(void)
{
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
	msleep(50);

	/* do reset */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 1);
	msleep(20);		/* tRES >= 50us */

	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_DIN), 0);
}

static struct mxc_lcd_platform_data dvi_data = {
	.core_reg = "VGEN1",
	.io_reg = "VGEN3",
	.reset = dvi_reset,
};

static void vga_reset(void)
{
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), "eim_a19");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
	msleep(50);
	/* do reset */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 1);
	msleep(10);		/* tRES >= 50us */
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A19), 0);
}

static struct mxc_lcd_platform_data vga_data = {
	.core_reg = "VCAM",
	.io_reg = "VGEN3",
	.analog_reg = "VAUDIO",
	.reset = vga_reset,
};

static void si4702_reset(void)
{
	return;
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), 0);
	msleep(100);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A21), 1);
	msleep(100);
}

static void si4702_clock_ctl(int flag)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A18), flag);
	msleep(100);
}

static void si4702_gpio_get(void)
{
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A18), "eim_a18");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A18), 0);
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
};

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
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
#endif
#ifdef CONFIG_I2C_MXC_SELECT2
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
};
#endif

#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
	{
	 .type = "sii9022",
	 .addr = 0x39,
	 .platform_data = &dvi_data,
	 },
	{
	 .type = "ch7026",
	 .addr = 0x75,
	 .platform_data = &vga_data,
	 },
	{
	 .type = "si4702",
	 .addr = 0x10,
	 .platform_data = (void *)&si4702_data,
	 },
};
#endif

#endif

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
static struct mtd_partition mxc_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00040000,},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,},

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
	 .name = "mxc_spi_nor",
	 .parts = mxc_spi_nor_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_spi_nor_partitions),
	 .type = "sst25vf016b",},
	{
	 .name = "mxc_dataflash",
	 .parts = mxc_dataflash_partitions,
	 .nr_parts = ARRAY_SIZE(mxc_dataflash_partitions),
	 .type = "at45db321d",}
};
#endif

static struct spi_board_info mxc_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	{
	 .modalias = "mxc_spi_nor",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[0],},
#endif
};

static struct spi_board_info mxc_dataflash_device[] __initdata = {
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	{
	 .modalias = "mxc_dataflash",
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,
	 .platform_data = &mxc_spi_flash_data[1],},
#endif
};

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_1));
	else
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
		return ret;
	} else {		/* config the det pin for SDHC2 */
		if (board_is_rev(BOARD_REV_2))
			/* BB2.5 */
			ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_6));
		else
			/* BB2.0 */
			ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_4));
		return ret;
	}
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_inserted_state = 1,
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
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_6),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_6),
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
	if (board_is_rev(BOARD_REV_2)) {
		/* BB2.5 */
		mxcsdhc2_resources[2].start =
			IOMUX_TO_IRQ(MX51_PIN_GPIO1_6);	/* SD2 CD */
		mxcsdhc2_resources[2].end =
			IOMUX_TO_IRQ(MX51_PIN_GPIO1_6);	/* SD2 CD */
	}

	(void)platform_device_register(&mxcsdhc1_device);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) \
    || defined(CONFIG_SND_SOC_IMX_3STACK_SGTL5000_MODULE)
static int mxc_sgtl5000_amp_enable(int enable);

static int headphone_det_status(void)
{
	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2)
		return (gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_D14)) == 0);

	return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS0));
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_irq = IOMUX_TO_IRQ(MX51_PIN_NANDF_CS0),
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.sysclk = 12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &sgtl5000_data,
		},
};

static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), enable ? 1 : 0);
	return 0;
}

static void mxc_init_sgtl5000(void)
{
	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2) {
		sgtl5000_data.sysclk = 26000000;
	}

	gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), "eim_a23");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_EIM_A23), 0);

	platform_device_register(&mxc_sgtl5000_device);
}
#else
static inline void mxc_init_sgtl5000(void)
{
}
#endif

#if defined CONFIG_FEC
static struct resource mxc_fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ
	},
};

struct platform_device mxc_fec_device = {
	.name = "fec",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_fec_resources),
	.resource = mxc_fec_resources,
};

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

#if defined(CONFIG_GPIO_BUTTON_MXC) || \
	defined(CONFIG_GPIO_BUTTON_MXC_MODULE)

#define MXC_BUTTON_GPIO_PIN MX51_PIN_EIM_DTACK

static struct mxc_gpio_button_data gpio_button_data = {
	.name = "Power Button (CM)",
	.gpio = MXC_BUTTON_GPIO_PIN,
	.irq = IOMUX_TO_IRQ(MXC_BUTTON_GPIO_PIN),
	.key = KEY_POWER,
};

static struct platform_device gpio_button_device = {
	.name = "gpio_button",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &gpio_button_data,
		},
};

static inline void mxc_init_gpio_button(void)
{
	gpio_request(IOMUX_TO_GPIO(MXC_BUTTON_GPIO_PIN), "button");
	gpio_direction_input(IOMUX_TO_GPIO(MXC_BUTTON_GPIO_PIN));
	platform_device_register(&gpio_button_device);
}
#else
static inline void mxc_init_gpio_button(void)
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
	char *str;
	int size = SZ_512M - SZ_32M;
	struct tag *t;

	mxc_cpu_init();

	get_cpu_wp = mx51_babbage_get_cpu_wp;
	set_num_cpu_wp = mx51_babbage_set_num_cpu_wp;

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

		t->u.mem.size = size;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		mxcfb_resources[0].start = t->u.mem.start + size;
		mxcfb_resources[0].end = t->u.mem.start + SZ_512M - 1;
#endif
	}
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
}

/*!
 * Power Key interrupt handler.
 */
static irqreturn_t power_key_int(int irq, void *dev_id)
{
	pr_info(KERN_INFO "PWR key pressed\n");
	return 0;
}

/*!
 * Power Key initialization.
 */
static int __init mxc_init_power_key(void)
{
	/* Set power key as wakeup resource */
	int irq, ret;
	irq = IOMUX_TO_IRQ(MX51_PIN_EIM_A27);
	set_irq_type(irq, IRQF_TRIGGER_RISING);
	ret = request_irq(irq, power_key_int, 0, "power_key", 0);
	if (ret)
		pr_info("register on-off key interrupt failed\n");
	else
		enable_irq_wake(irq);
	return ret;
}

late_initcall(mxc_init_power_key);

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	mxc_register_gpios();
	mx51_babbage_io_init();
	early_console_setup(saved_command_line);

	mxc_init_devices();

	mxc_init_keypad();
	mxc_init_mmc();
	mxc_init_gpio_button();
	mx51_babbage_init_mc13892();

	if (board_is_rev(BOARD_REV_2))
		/* BB2.5 */
		spi_register_board_info(mxc_dataflash_device,
					ARRAY_SIZE(mxc_dataflash_device));
	else
		/* BB2.0 */
		spi_register_board_info(mxc_spi_nor_device,
					ARRAY_SIZE(mxc_spi_nor_device));

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
	if (cpu_is_mx51_rev(CHIP_REV_2_0) >= 1) {
		vga_data.core_reg = NULL;
		vga_data.io_reg = NULL;
		vga_data.analog_reg = NULL;
	}
	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));
#endif

#endif
	pm_power_off = mxc_power_off;
	mxc_init_fec();
	mxc_init_sgtl5000();
}

static void __init mx51_babbage_timer_init(void)
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
	.init	= mx51_babbage_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_BABBAGE data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_BABBAGE, "Freescale MX51 Babbage Board")
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
