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
#include <mach/irqs.h>
#include <mach/lradc.h>
#include <mach/lcdif.h>
#include <mach/ddi_bc.h>

#include "device.h"
#include "mx23_pins.h"
#include "mach/mx23.h"

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
	 .start = IRQ_DEBUG_UART ,
	 .end = IRQ_DEBUG_UART ,
	 },
};

static void __init mx23_init_duart(void)
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
static void mx23_init_duart(void)
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
	.chan_num = 8,
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

static void __init mx23_init_dma(void)
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
static void mx23_init_dma(void)
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
	 .start = IRQ_LCDIF_ERROR,
	 .end   = IRQ_LCDIF_ERROR,
	 },
};

static struct mxs_platform_fb_data mxs_framebuffer_pdata = {
	.list = LIST_HEAD_INIT(mxs_framebuffer_pdata.list),
};

static void __init mx23_init_lcdif(void)
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
static void __init mx23_init_lcdif(void)
{
	;
}
#endif

#if defined(CONFIG_VIDEO_MXS_PXP) || \
	defined(CONFIG_VIDEO_MXS_PXP_MODULE)
static struct resource pxp_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
		.start	= (unsigned int)IO_ADDRESS(PXP_PHYS_ADDR),
		.end	= (unsigned int)IO_ADDRESS(PXP_PHYS_ADDR) + 0x2000 - 1,
	}, {
		.flags	= IORESOURCE_IRQ,
		.start	= IRQ_PXP,
		.end	= IRQ_PXP,
	},
};
static void __init mx23_init_pxp(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-pxp", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = pxp_resource;
	pdev->num_resources = ARRAY_SIZE(pxp_resource);
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_pxp(void)
{
	;
}
#endif

#if defined(CONFIG_MXS_VIIM) || defined(CONFIG_MXS_VIIM_MODULE)
struct resource viim_resources[] = {
	[0] = {
		.start  = DIGCTL_PHYS_ADDR,
		.end    = DIGCTL_PHYS_ADDR + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = OCOTP_PHYS_ADDR,
		.end    = OCOTP_PHYS_ADDR + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};
static void __init mx23_init_viim(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs_viim", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = viim_resources;
	pdev->num_resources = ARRAY_SIZE(viim_resources);

	mxs_add_device(pdev, 2);
}
#else
static void __init mx23_init_viim(void)
{
}
#endif

#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
static struct resource i2c_resource[] = {
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
	 .start = IRQ_I2C_ERROR,
	 .end   = IRQ_I2C_ERROR,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_I2C_DMA,
	 .end   = IRQ_I2C_DMA,
	 },
};

static struct mxs_i2c_plat_data i2c_platdata = {
#ifdef	CONFIG_I2C_MXS_SELECT0_PIOQUEUE_MODE
	.pioqueue_mode = 0,
#endif
};

static void __init mx23_init_i2c(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-i2c", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = i2c_resource;
	pdev->num_resources = ARRAY_SIZE(i2c_resource);
	pdev->dev.platform_data = &i2c_platdata;

	mxs_add_device(pdev, 2);
}
#else
static void __init mx23_init_i2c(void)
{
}
#endif

#if defined(CONFIG_MXS_WATCHDOG) || defined(CONFIG_MXS_WATCHDOG_MODULE)
static struct resource mx23_wdt_res = {
	.flags = IORESOURCE_MEM,
	.start = RTC_PHYS_ADDR,
	.end   = RTC_PHYS_ADDR + 0x2000 - 1,
};

static void __init mx23_init_wdt(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-wdt", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = &mx23_wdt_res;
	pdev->num_resources = 1;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_wdt(void)
{
	;
}
#endif

#if defined(CONFIG_RTC_DRV_MXS) || defined(CONFIG_RTC_DRV_MXS_MODULE)
static struct resource mx23_rtc_res[] = {
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

static void __init mx23_init_rtc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-rtc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_rtc_res;
	pdev->num_resources = ARRAY_SIZE(mx23_rtc_res);
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_rtc(void)
{
	;
}
#endif

#ifdef CONFIG_MXS_LRADC
struct mxs_lradc_plat_data mx23_lradc_data = {
	.vddio_voltage = BV_LRADC_CTRL4_LRADC6SELECT__CHANNEL6,
	.battery_voltage = BV_LRADC_CTRL4_LRADC7SELECT__CHANNEL7,
};

static struct resource mx23_lradc_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
};

static void __init mx23_init_lradc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-lradc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_lradc_res;
	pdev->num_resources = ARRAY_SIZE(mx23_lradc_res);
	pdev->dev.platform_data = &mx23_lradc_data;
	mxs_add_device(pdev, 0);
}
#else
static void __init mx23_init_lradc(void)
{
	;
}
#endif

#if defined(CONFIG_KEYBOARD_MXS) || defined(CONFIG_KEYBOARD_MXS_MODULE)
static struct mxskbd_keypair keyboard_data[] = {
	{ 100, KEY_F1 },
	{ 306, KEY_RIGHT},
	{ 626, KEY_F2},
	{ 932, KEY_LEFT },
	{ 1584, KEY_UP },
	{ 2207, KEY_DOWN },
	{ 1907, KEY_F3 },
	{ 2831, KEY_SELECT },
	{ -1, 0 },
};

static struct mxs_kbd_plat_data mxs_kbd_data = {
	.keypair = keyboard_data,
	.channel = LRADC_CH0,
};

static struct resource mx23_kbd_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH0,
	 .end   = IRQ_LRADC_CH0,
	 },
};

static void __init mx23_init_kbd(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-kbd", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_kbd_res;
	pdev->num_resources = ARRAY_SIZE(mx23_kbd_res);
	pdev->dev.platform_data = &mxs_kbd_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_kbd(void)
{
	;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)
static struct mxs_touchscreen_plat_data mx23_ts_data = {
	.x_plus_chan = LRADC_TOUCH_X_PLUS,
	.x_minus_chan = LRADC_TOUCH_X_MINUS,
	.y_plus_chan = LRADC_TOUCH_Y_PLUS,
	.y_minus_chan = LRADC_TOUCH_Y_MINUS,
	.x_plus_val = BM_LRADC_CTRL0_XPLUS_ENABLE,
	.x_minus_val = BM_LRADC_CTRL0_XMINUS_ENABLE,
	.y_plus_val = BM_LRADC_CTRL0_YPLUS_ENABLE,
	.y_minus_val = BM_LRADC_CTRL0_YMINUS_ENABLE,
	.x_plus_mask = BM_LRADC_CTRL0_XPLUS_ENABLE,
	.x_minus_mask = BM_LRADC_CTRL0_XMINUS_ENABLE,
	.y_plus_mask = BM_LRADC_CTRL0_YPLUS_ENABLE,
	.y_minus_mask = BM_LRADC_CTRL0_YMINUS_ENABLE,
};

static struct resource mx23_ts_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = LRADC_PHYS_ADDR,
	 .end   = LRADC_PHYS_ADDR + 0x2000 - 1,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_TOUCH_DETECT,
	 .end   = IRQ_TOUCH_DETECT,
	 },
	{
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_LRADC_CH5,
	 .end   = IRQ_LRADC_CH5,
	 },
};

static void __init mx23_init_ts(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-ts", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_ts_res;
	pdev->num_resources = ARRAY_SIZE(mx23_ts_res);
	pdev->dev.platform_data = &mx23_ts_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_ts(void)
{
	;
}
#endif

#if defined(CONFIG_CRYPTO_DEV_DCP)

static struct resource dcp_resources[] = {

	{
		.flags = IORESOURCE_MEM,
		.start = DCP_PHYS_ADDR,
		.end   = DCP_PHYS_ADDR + 0x2000 - 1,
	}, {
		.flags = IORESOURCE_IRQ,
		.start = IRQ_DCP_VMI,
		.end = IRQ_DCP_VMI,
	}, {
		.flags = IORESOURCE_IRQ,
		.start = IRQ_DCP,
		.end = IRQ_DCP,
	},
};

static void __init mx23_init_dcp(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("dcp", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = dcp_resources;
	pdev->num_resources = ARRAY_SIZE(dcp_resources);
	mxs_add_device(pdev, 3);
}
#else
static void __init mx23_init_dcp(void)
{
	;
}
#endif

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
#define MMC0_POWER	MXS_PIN_TO_GPIO(PINID_PWM3)
#define MMC0_WP		MXS_PIN_TO_GPIO(PINID_PWM4)

static int mxs_mmc_get_wp_mmc0(void)
{
	return gpio_get_value(MMC0_WP);
}

static int mxs_mmc_hw_init_mmc0(void)
{
	int ret = 0;

	/* Configure write protect GPIO pin */
	ret = gpio_request(MMC0_WP, "mmc0_wp");
	if (ret) {
		pr_err("wp\r\n");
		goto out_wp;
	}
	gpio_set_value(MMC0_WP, 0);
	gpio_direction_input(MMC0_WP);

	/* Configure POWER pin as gpio to drive power to MMC slot */
	ret = gpio_request(MMC0_POWER, "mmc0_power");
	if (ret) {
		pr_err("power\r\n");
		goto out_power;
	}
	gpio_direction_output(MMC0_POWER, 0);
	mdelay(100);

	return 0;

out_power:
	gpio_free(MMC0_WP);
out_wp:
	return ret;
}

static void mxs_mmc_hw_release_mmc0(void)
{
	gpio_free(MMC0_POWER);
	gpio_free(MMC0_WP);

}

static void mxs_mmc_cmd_pullup_mmc0(int enable)
{
	mxs_set_pullup(PINID_SSP1_CMD, enable, "mmc0_cmd");
}

static unsigned long mxs_mmc_setclock_mmc0(unsigned long hz)
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

static struct mxs_mmc_platform_data mx23_mmc0_data = {
	.hw_init	= mxs_mmc_hw_init_mmc0,
	.hw_release	= mxs_mmc_hw_release_mmc0,
	.get_wp		= mxs_mmc_get_wp_mmc0,
	.cmd_pullup	= mxs_mmc_cmd_pullup_mmc0,
	.setclock	= mxs_mmc_setclock_mmc0,
	.caps 		= MMC_CAP_4_BIT_DATA,
	.min_clk	= 400000,
	.max_clk	= 48000000,
	.read_uA        = 50000,
	.write_uA       = 70000,
	.clock_mmc = "ssp.0",
	.power_mmc = NULL,
};

static struct resource mx23_mmc0_resource[] = {
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
		.start	= IRQ_SSP_ERROR,
		.end	= IRQ_SSP_ERROR,
	},
};

static void __init mx23_init_mmc(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-mmc", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = mx23_mmc0_resource;
	pdev->num_resources = ARRAY_SIZE(mx23_mmc0_resource);
	pdev->dev.platform_data = &mx23_mmc0_data;

	mxs_add_device(pdev, 2);
}
#else
static void mx23_init_mmc(void)
{
	;
}
#endif

#if defined(CONFIG_BATTERY_MXS)
/* battery info data */
static ddi_bc_Cfg_t battery_data = {
	.u32StateMachinePeriod		 = 100,		/* ms */
	.u16CurrentRampSlope		 = 75,		/* mA/s */
	.u16ConditioningThresholdVoltage = 2900, 	/* mV */
	.u16ConditioningMaxVoltage	 = 3000,	/* mV */
	.u16ConditioningCurrent		 = 60,		/* mA */
	.u32ConditioningTimeout		 = 4*60*60*1000, /* ms (4 hours) */
	.u16ChargingVoltage		 = 4200,	/* mV */
	/* FIXME: the current comparator could have h/w bugs in current
	 * detection through POWER_STS.CHRGSTS bit */
	.u16ChargingCurrent		 = 600,		/* mA 600 */
	.u16ChargingThresholdCurrent	 = 60,		/* mA 60 */
	.u32ChargingTimeout		 = 4*60*60*1000,/* ms (4 hours) */
	.u32TopOffPeriod		 = 30*60*1000,	/* ms (30 minutes) */
	.monitorDieTemp			 = 1,		/* Monitor the die */
	.u8DieTempHigh			 = 75,		/* deg centigrade */
	.u8DieTempLow			 = 65,		/* deg centigrade */
	.u16DieTempSafeCurrent		 = 0,		/* mA */
	.monitorBatteryTemp		 = 0,		/* Monitor the battery*/
	.u8BatteryTempChannel		 = 1,		/* LRADC 1 */
	.u16BatteryTempHigh		 = 642,		/* Unknown units */
	.u16BatteryTempLow		 = 497,		/* Unknown units */
	.u16BatteryTempSafeCurrent	 = 0,		/* mA */
};

static struct resource battery_resource[] = {
	{/* 0 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_VDD5V,
		.end    = IRQ_VDD5V,
	},
	{/* 1 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_DCDC4P2_BO,
		.end    = IRQ_DCDC4P2_BO,
	},
	{/* 2 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_BATT_BRNOUT,
		.end    = IRQ_BATT_BRNOUT,
	},
	{/* 3 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_VDDD_BRNOUT,
		.end    = IRQ_VDDD_BRNOUT,
	},
	{/* 4 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_VDD18_BRNOUT,
		.end    = IRQ_VDD18_BRNOUT,
	},
	{/* 5 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_VDDIO_BRNOUT,
		.end    = IRQ_VDDIO_BRNOUT,
	},
	{/* 6 */
		.flags  = IORESOURCE_IRQ,
		.start  = IRQ_VDD5V_DROOP,
		.end    = IRQ_VDD5V_DROOP,
	},
};

static void mx23_init_battery(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-battery", 0);
	if (pdev) {
		pdev->resource = battery_resource,
		pdev->num_resources = ARRAY_SIZE(battery_resource),
		pdev->dev.platform_data = &battery_data;
		mxs_add_device(pdev, 3);
	}
}
#else
static void mx23_init_battery(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
       defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
void __init mx23_init_spdif(void)
{	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-spdif", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	mxs_add_device(pdev, 3);
}
#else
static inline mx23_init_spdif(void)
{
}
#endif

int __init mx23_device_init(void)
{
	mx23_init_dma();
	mx23_init_viim();
	mx23_init_duart();
	mx23_init_auart();
	mx23_init_lradc();
	mx23_init_i2c();
	mx23_init_kbd();
	mx23_init_wdt();
	mx23_init_ts();
	mx23_init_rtc();
	mx23_init_dcp();
	mx23_init_mmc();
	mx23_init_spdif();
	mx23_init_lcdif();
	mx23_init_pxp();
	mx23_init_battery();

	return 0;
}

static struct __initdata map_desc mx23_io_desc[] = {
	{
	 .virtual = MX23_SOC_IO_VIRT_BASE,
	 .pfn = __phys_to_pfn(MX23_SOC_IO_PHYS_BASE),
	 .length = MX23_SOC_IO_AREA_SIZE,
	 .type = MT_DEVICE,
	 },
	{
	 .virtual = MX23_OCRAM_BASE,
	 .pfn = __phys_to_pfn(MX23_OCRAM_PHBASE),
	 .length = MX23_OCRAM_SIZE,
	 .type  = MT_DEVICE,
	},
};

void __init mx23_map_io(void)
{
	iotable_init(mx23_io_desc, ARRAY_SIZE(mx23_io_desc));
}

void __init mx23_irq_init(void)
{
	avic_init_irq(IO_ADDRESS(ICOLL_PHYS_ADDR), ARCH_NR_IRQS);
}

static void mx23_timer_init(void)
{
	int i, reg;
	mx23_clock_init();

	mx23_timer.clk = clk_get(NULL, "clk_32k");
	if (mx23_timer.clk == NULL || IS_ERR(mx23_timer.clk))
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_SFTRST,
		     mx23_timer.base + HW_TIMROT_ROTCTRL_CLR);
	for (i = 0; i < 10000; i++) {
		reg = __raw_readl(mx23_timer.base + HW_TIMROT_ROTCTRL);
		if (!(reg & BM_TIMROT_ROTCTRL_SFTRST))
			break;
		udelay(2);
	}
	if (i >= 10000)
		return;
	__raw_writel(BM_TIMROT_ROTCTRL_CLKGATE,
		     mx23_timer.base + HW_TIMROT_ROTCTRL_CLR);

	reg = __raw_readl(mx23_timer.base + HW_TIMROT_ROTCTRL);

	mxs_nomatch_timer_init(&mx23_timer);
}

struct mxs_sys_timer mx23_timer = {
	.timer = {
		  .init = mx23_timer_init,
		  },
	.clk_sel = BV_TIMROT_TIMCTRLn_SELECT__32KHZ_XTAL,
	.base = IO_ADDRESS(TIMROT_PHYS_ADDR),
};
