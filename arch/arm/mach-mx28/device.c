/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/phy.h>
#include <linux/fec.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/regs-timrot.h>
#include <mach/regs-lradc.h>
#include <mach/device.h>
#include <mach/dma.h>
#include <mach/lradc.h>
#include <mach/lcdif.h>

#include "device.h"
#include "mx28_pins.h"

#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
static struct resource duart_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = DUART_PHYS_ADDR,
	 .end = DUART_PHYS_ADDR + 0x1000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_DUART,
	 .end = IRQ_DUART,
	 },
};

static void __init mx28_init_duart(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-duart", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = duart_resource;
	pdev->num_resources = ARRAY_SIZE(duart_resource);
	mxs_add_device(pdev, 3);
}
#else
static void mx28_init_duart(void)
{
}
#endif

#if defined(CONFIG_MXS_DMA_ENGINE)
static struct resource mxs_ahb_apbh_res = {
	.flags = IORESOURCE_MEM,
	.start = APBH_DMA_PHYS_ADDR,
	.end = APBH_DMA_PHYS_ADDR + 0x2000 - 1,
};

static struct mxs_dma_plat_data mxs_ahb_apbh_data = {
	.chan_base = MXS_DMA_CHANNEL_AHB_APBH,
	.chan_num = 16,
};

static struct resource mxs_ahb_apbx_res = {
	.flags = IORESOURCE_MEM,
	.start = APBX_DMA_PHYS_ADDR,
	.end = APBX_DMA_PHYS_ADDR + 0x2000 - 1,
};

static struct mxs_dma_plat_data mxs_ahb_apbx_data = {
	.chan_base = MXS_DMA_CHANNEL_AHB_APBX,
	.chan_num = 16,
};

static void __init mx28_init_dma(void)
{
	int i;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev;
	lookup = mxs_get_devices("mxs-dma");
	if (lookup == NULL || IS_ERR(lookup))
		return;
	for (i = 0; i < lookup->size; i++) {
		pdev = lookup->pdev + i;
		if (!strcmp(pdev->name, "mxs-dma-apbh")) {
			pdev->resource = &mxs_ahb_apbh_res;
			pdev->dev.platform_data = &mxs_ahb_apbh_data;
		} else if (!strcmp(pdev->name, "mxs-dma-apbx")) {
			pdev->resource = &mxs_ahb_apbx_res;
			pdev->dev.platform_data = &mxs_ahb_apbx_data;
		} else
			continue;
		pdev->num_resources = 1;
		mxs_add_device(pdev, 0);
	}
}
#else
static void mx28_init_dma(void)
{
	;
}
#endif

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static struct resource framebuffer_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LCDIF_PHYS_ADDR,
	 .end   = LCDIF_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LCDIF,
	 .end   = IRQ_LCDIF,
	 },
};

static struct mxs_platform_fb_data mxs_framebuffer_pdata = {
	.list = LIST_HEAD_INIT(mxs_framebuffer_pdata.list),
};

static void __init mx28_init_lcdif(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-fb", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = framebuffer_resource;
	pdev->num_resources = ARRAY_SIZE(framebuffer_resource);
	pdev->dev.platform_data = &mxs_framebuffer_pdata;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_lcdif(void)
{
	;
}
#endif

#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
#ifdef	CONFIG_I2C_MXS_SELECT0
static struct resource i2c0_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = I2C0_PHYS_ADDR,
	 .end   = I2C0_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_I2C0,
	 .end   = MXS_DMA_CHANNEL_AHB_APBX_I2C0,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_I2C0_ERROR,
	 .end   = IRQ_I2C0_ERROR,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_I2C0_DMA,
	 .end   = IRQ_I2C0_DMA,
	 },
};

static struct mxs_i2c_plat_data i2c0_platdata = {
#ifdef	CONFIG_I2C_MXS_SELECT0_PIOQUEUE_MODE
	.pioqueue_mode = 1,
#endif
};
#endif

#ifdef	CONFIG_I2C_MXS_SELECT1
static struct resource i2c1_resource[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = I2C1_PHYS_ADDR,
	 .end   = I2C1_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_DMA,
	 .start = MXS_DMA_CHANNEL_AHB_APBX_I2C1,
	 .end   = MXS_DMA_CHANNEL_AHB_APBX_I2C1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_I2C1_ERROR,
	 .end   = IRQ_I2C1_ERROR,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_I2C1_DMA,
	 .end   = IRQ_I2C1_DMA,
	 },
};

static struct mxs_i2c_plat_data i2c1_platdata = {
#ifdef	CONFIG_I2C_MXS_SELECT1_PIOQUEUE_MODE
	.pioqueue_mode = 1,
#endif
};
#endif

static void __init mx28_init_i2c(void)
{
	int i;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev;

	lookup = mxs_get_devices("mxs-i2c");
	if (lookup == NULL || IS_ERR(lookup))
		return;
	for (i = 0; i < lookup->size; i++) {
		pdev = lookup->pdev + i;
		switch (pdev->id) {
#ifdef	CONFIG_I2C_MXS_SELECT0
		case 0:
			pdev->resource = i2c0_resource;
			pdev->num_resources = ARRAY_SIZE(i2c0_resource);
			pdev->dev.platform_data = &i2c0_platdata;
			break;
#endif
#ifdef	CONFIG_I2C_MXS_SELECT1
		case 1:
			pdev->resource = i2c1_resource;
			pdev->num_resources = ARRAY_SIZE(i2c1_resource);
			pdev->dev.platform_data = &i2c1_platdata;
			break;
#endif
		default:
			return;
		}
		mxs_add_device(pdev, 2);
	}
}
#else
static void __init mx28_init_i2c(void)
{
}
#endif

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
#if defined(CONFIG_MACH_MX28EVK)
#define MMC0_POWER	MXS_PIN_TO_GPIO(PINID_PWM3)
#define MMC1_POWER	MXS_PIN_TO_GPIO(PINID_PWM4)
#define MMC0_WP		MXS_PIN_TO_GPIO(PINID_SSP1_SCK)
#define MMC1_WP		MXS_PIN_TO_GPIO(PINID_GPMI_RESETN)
#endif

static int mxs_mmc_get_wp_ssp0(void)
{
	return gpio_get_value(MMC0_WP);
}

static int mxs_mmc_hw_init_ssp0(void)
{
	int ret = 0;

	/* Configure write protect GPIO pin */
	ret = gpio_request(MMC0_WP, "mmc0_wp");
	if (ret)
		goto out_wp;

	gpio_set_value(MMC0_WP, 0);
	gpio_direction_input(MMC0_WP);

	/* Configure POWER pin as gpio to drive power to MMC slot */
	ret = gpio_request(MMC0_POWER, "mmc0_power");
	if (ret)
		goto out_power;

	gpio_direction_output(MMC0_POWER, 0);
	mdelay(100);

	return 0;

out_power:
	gpio_free(MMC0_WP);
out_wp:
	return ret;
}

static void mxs_mmc_hw_release_ssp0(void)
{
	gpio_free(MMC0_POWER);
	gpio_free(MMC0_WP);

}

static void mxs_mmc_cmd_pullup_ssp0(int enable)
{
	mxs_set_pullup(PINID_SSP0_CMD, enable, "mmc0_cmd");
}

static unsigned long mxs_mmc_setclock_ssp0(unsigned long hz)
{
	struct clk *ssp = clk_get(NULL, "ssp.0"), *parent;

	if (hz > 1000000)
		parent = clk_get(NULL, "ref_io.0");
	else
		parent = clk_get(NULL, "xtal.0");

	clk_set_parent(ssp, parent);
	clk_set_rate(ssp, 2 * hz);
	clk_put(parent);
	clk_put(ssp);

	return hz;
}

static int mxs_mmc_get_wp_ssp1(void)
{
	return gpio_get_value(MMC1_WP);
}

static int mxs_mmc_hw_init_ssp1(void)
{
	int ret = 0;

	/* Configure write protect GPIO pin */
	ret = gpio_request(MMC1_WP, "mmc1_wp");
	if (ret)
		goto out_wp;

	gpio_set_value(MMC1_WP, 0);
	gpio_direction_input(MMC1_WP);

	/* Configure POWER pin as gpio to drive power to MMC slot */
	ret = gpio_request(MMC1_POWER, "mmc1_power");
	if (ret)
		goto out_power;

	gpio_direction_output(MMC1_POWER, 0);
	mdelay(100);

	return 0;

out_power:
	gpio_free(MMC1_WP);
out_wp:
	return ret;
}

static void mxs_mmc_hw_release_ssp1(void)
{
	gpio_free(MMC1_POWER);
	gpio_free(MMC1_WP);
}

static void mxs_mmc_cmd_pullup_ssp1(int enable)
{
	mxs_set_pullup(PINID_GPMI_RDY1, enable, "mmc1_cmd");
}

static unsigned long mxs_mmc_setclock_ssp1(unsigned long hz)
{
	struct clk *ssp = clk_get(NULL, "ssp.1"), *parent;

	if (hz > 1000000)
		parent = clk_get(NULL, "ref_io.0");
	else
		parent = clk_get(NULL, "xtal.0");

	clk_set_parent(ssp, parent);
	clk_set_rate(ssp, 2 * hz);
	clk_put(parent);
	clk_put(ssp);

	return hz;
}

static struct mxs_mmc_platform_data mmc0_data = {
	.hw_init	= mxs_mmc_hw_init_ssp0,
	.hw_release	= mxs_mmc_hw_release_ssp0,
	.get_wp		= mxs_mmc_get_wp_ssp0,
	.cmd_pullup	= mxs_mmc_cmd_pullup_ssp0,
	.setclock	= mxs_mmc_setclock_ssp0,
	.caps 		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk	= 400000,
	.max_clk	= 52000000,
	.read_uA        = 50000,
	.write_uA       = 70000,
	.clock_mmc = "ssp.0",
	.power_mmc = NULL,
};

static struct resource mmc0_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
		.start	= SSP0_PHYS_ADDR,
		.end	= SSP0_PHYS_ADDR + 0x2000 - 1,
	},
	{
		.flags	= IORESOURCE_DMA,
		.start	= MXS_DMA_CHANNEL_AHB_APBH_SSP0,
		.end	= MXS_DMA_CHANNEL_AHB_APBH_SSP0,
	},
	{
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SSP0_DMA,
		.end	= IRQ_SSP0_DMA,
	},
	{
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SSP0,
		.end	= IRQ_SSP0,
	},
};

static struct mxs_mmc_platform_data mmc1_data = {
	.hw_init	= mxs_mmc_hw_init_ssp1,
	.hw_release	= mxs_mmc_hw_release_ssp1,
	.get_wp		= mxs_mmc_get_wp_ssp1,
	.cmd_pullup	= mxs_mmc_cmd_pullup_ssp1,
	.setclock	= mxs_mmc_setclock_ssp1,
	.caps 		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk	= 400000,
	.max_clk	= 52000000,
	.read_uA        = 50000,
	.write_uA       = 70000,
	.clock_mmc = "ssp.1",
	.power_mmc = NULL,
};

static struct resource mmc1_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
		.start	= SSP1_PHYS_ADDR,
		.end	= SSP1_PHYS_ADDR + 0x2000 - 1,
	},
	{
		.flags	= IORESOURCE_DMA,
		.start	= MXS_DMA_CHANNEL_AHB_APBH_SSP1,
		.end	= MXS_DMA_CHANNEL_AHB_APBH_SSP1,
	},
	{
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SSP1_DMA,
		.end	= IRQ_SSP1_DMA,
	},
	{
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_SSP1,
		.end	= IRQ_SSP1,
	},
};

static void __init mx28_init_mmc(void)
{
	int i;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev;

	lookup = mxs_get_devices("mxs-mmc");
	if (lookup == NULL || IS_ERR(lookup))
		return;
	for (i = 0; i < lookup->size; i++) {
		pdev = lookup->pdev + i;
		switch (pdev->id) {
		case 0:
			pdev->resource = mmc0_resource;
			pdev->num_resources = ARRAY_SIZE(mmc0_resource);
			pdev->dev.platform_data = &mmc0_data;
			break;
		case 1:
			pdev->resource = mmc1_resource;
			pdev->num_resources = ARRAY_SIZE(mmc1_resource);
			pdev->dev.platform_data = &mmc1_data;
			break;
		default:
			return;
		}
		mxs_add_device(pdev, 2);
	}
}
#else
static void mx28_init_mmc(void)
{
}
#endif

#if defined(CONFIG_MXS_WATCHDOG) || defined(CONFIG_MXS_WATCHDOG_MODULE)
static struct resource mx28_wdt_res = {
	.flags = IORESOURCE_MEM,
	.start = RTC_PHYS_ADDR,
	.end   = RTC_PHYS_ADDR + 0x2000 - 1,
};

static void __init mx28_init_wdt(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-wdt", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = &mx28_wdt_res;
	pdev->num_resources = 1;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_wdt(void)
{
	;
}
#endif

#if defined(CONFIG_RTC_DRV_MXS) || defined(CONFIG_RTC_DRV_MXS_MODULE)
static struct resource mx28_rtc_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = RTC_PHYS_ADDR,
	 .end   = RTC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_RTC_ALARM,
	 .end   = IRQ_RTC_ALARM,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_RTC_1MSEC,
	 .end   = IRQ_RTC_1MSEC,
	},
};

static void __init mx28_init_rtc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-rtc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx28_rtc_res;
	pdev->num_resources = ARRAY_SIZE(mx28_rtc_res);
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_rtc(void)
{
	;
}
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
static struct resource fec_resources[] = {
	{
		.start  = ENET_PHYS_ADDR,
		.end    = ENET_PHYS_ADDR + 0xffff,
		.flags  = IORESOURCE_MEM
	},
	{
		.start  = IRQ_ENET_SWI,
		.end    = IRQ_ENET_SWI,
		.flags  = IORESOURCE_IRQ
	},
	{
		.start  = IRQ_ENET_MAC0,
		.end    = IRQ_ENET_MAC0,
		.flags  = IORESOURCE_IRQ
	},
	{
		.start  = IRQ_ENET_MAC1,
		.end    = IRQ_ENET_MAC1,
		.flags  = IORESOURCE_IRQ
	},
};

extern int mx28evk_enet_gpio_init(void);
static struct fec_platform_data fec_pdata = {
	.phy = PHY_INTERFACE_MODE_RMII,
	.init = mx28evk_enet_gpio_init,
};

static void __init mx28_init_fec(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-fec", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = fec_resources;
	pdev->num_resources = ARRAY_SIZE(fec_resources);
	pdev->dev.platform_data = &fec_pdata;
	mxs_add_device(pdev, 2);
}
#else
static void __init mx28_init_fec(void)
{
	;
}
#endif
#ifdef CONFIG_MXS_LRADC
struct mxs_lradc_plat_data mx28_lradc_data = {
	.vddio_voltage = BV_LRADC_CTRL4_LRADC6SELECT__CHANNEL10,
	.battery_voltage = BV_LRADC_CTRL4_LRADC7SELECT__CHANNEL7,
};

static struct resource mx28_lradc_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
};

static void __init mx28_init_lradc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-lradc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx28_lradc_res;
	pdev->num_resources = ARRAY_SIZE(mx28_lradc_res);
	pdev->dev.platform_data = &mx28_lradc_data;
	mxs_add_device(pdev, 0);
}
#else
static void __init mx28_init_lradc(void)
{
	;
}
#endif

#if defined(CONFIG_KEYBOARD_MXS) || defined(CONFIG_KEYBOARD_MXS_MODULE)
static struct mxskbd_keypair keyboard_data[] = {
	{ 100, KEY_F4 },
	{ 306, KEY_F5 },
	{ 626, KEY_F6 },
	{ 932, KEY_F7 },
	{ 1260, KEY_F8 },
	{ 1584, KEY_F9 },
	{ 1907, KEY_F10 },
	{ 2207, KEY_F11 },
	{ 2525, KEY_F12 },
	{ 2831, KEY_F13},
	{ 3134, KEY_F14 },
	{ -1, 0 },
};

static struct mxs_kbd_plat_data mxs_kbd_data = {
	.keypair = keyboard_data,
	.channel = LRADC_CH1,
};

static struct resource mx28_kbd_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH1,
	 .end   = IRQ_LRADC_CH1,
	 },
};

static void __init mx28_init_kbd(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-kbd", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx28_kbd_res;
	pdev->num_resources = ARRAY_SIZE(mx28_kbd_res);
	pdev->dev.platform_data = &mxs_kbd_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_kbd(void)
{
	;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)
static struct mxs_touchscreen_plat_data mx28_ts_data = {
	.x_plus_chan = LRADC_TOUCH_X_PLUS,
	.x_minus_chan = LRADC_TOUCH_X_MINUS,
	.y_plus_chan = LRADC_TOUCH_Y_PLUS,
	.y_minus_chan = LRADC_TOUCH_Y_MINUS,
	.x_plus_val = BM_LRADC_CTRL0_XPULSW,
	.x_minus_val = BF_LRADC_CTRL0_XNURSW(2),
	.y_plus_val = BF_LRADC_CTRL0_YPLLSW(1),
	.y_minus_val = BM_LRADC_CTRL0_YNLRSW,
	.x_plus_mask = BM_LRADC_CTRL0_XPULSW,
	.x_minus_mask = BM_LRADC_CTRL0_XNURSW,
	.y_plus_mask = BM_LRADC_CTRL0_YPLLSW,
	.y_minus_mask = BM_LRADC_CTRL0_YNLRSW,
};

static struct resource mx28_ts_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_TOUCH,
	 .end   = IRQ_LRADC_TOUCH,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH5,
	 .end   = IRQ_LRADC_CH5,
	 },
};

static void __init mx28_init_ts(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-ts", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx28_ts_res;
	pdev->num_resources = ARRAY_SIZE(mx28_ts_res);
	pdev->dev.platform_data = &mx28_ts_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28_init_ts(void)
{
	;
}
#endif
int __init mx28_device_init(void)
{
	mx28_init_dma();
	mx28_init_duart();
	mx28_init_i2c();
	mx28_init_lradc();
	mx28_init_auart();
	mx28_init_mmc();
	mx28_init_wdt();
	mx28_init_rtc();
	mx28_init_fec();
	mx28_init_kbd();
	mx28_init_ts();
	mx28_init_lcdif();

	return 0;
}

static struct __initdata map_desc mx28_io_desc[] = {
	{
	 .virtual = MX28_SOC_IO_VIRT_BASE,
	 .pfn = __phys_to_pfn(MX28_SOC_IO_PHYS_BASE),
	 .length = MX28_SOC_IO_AREA_SIZE,
	 .type = MT_DEVICE,
	 },
};

void __init mx28_map_io(void)
{
	iotable_init(mx28_io_desc, ARRAY_SIZE(mx28_io_desc));
}

void __init mx28_irq_init(void)
{
	avic_init_irq(IO_ADDRESS(ICOLL_PHYS_ADDR), ARCH_NR_IRQS);
}

static void mx28_timer_init(void)
{
	int i, reg;
	mx28_clock_init();

	mx28_timer.clk = clk_get(NULL, "clk_32k");
	if (mx28_timer.clk == NULL || IS_ERR(mx28_timer.clk))
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_SFTRST,
		     mx28_timer.base + HW_TIMROT_ROTCTRL_CLR);
	for (i = 0; i < 10000; i++) {
		reg = __raw_readl(mx28_timer.base + HW_TIMROT_ROTCTRL);
		if (!(reg & BM_TIMROT_ROTCTRL_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_CLKGATE,
		     mx28_timer.base + HW_TIMROT_ROTCTRL_CLR);

	reg = __raw_readl(mx28_timer.base + HW_TIMROT_ROTCTRL);
	for (i = 0; i < 4; i++) {
		if (!(reg & (BM_TIMROT_ROTCTRL_TIM0_PRESENT << i)))
			continue;
		mx28_timer.id = i;
		mx28_timer.irq = IRQ_TIMER0 + i;
		mxs_timer_init(&mx28_timer);
		return;
	}
}

struct mxs_sys_timer mx28_timer = {
	.timer = {
		  .init = mx28_timer_init,
		  },
	.clk_sel = BV_TIMROT_TIMCTRLn_SELECT__32KHZ_XTAL,
	.base = IO_ADDRESS(TIMROT_PHYS_ADDR),
};
