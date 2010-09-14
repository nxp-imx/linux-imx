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
#include <linux/gpmi-nfc.h>

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
#include "mx23evk.h"
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
	 .start = MXS_DMA_CHANNEL_AHB_APBX_I2C,
	 .end   = MXS_DMA_CHANNEL_AHB_APBX_I2C,
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

#if defined(CONFIG_MTD_NAND_GPMI_NFC)

static int gpmi_nfc_platform_init(unsigned int max_chip_count)
{
	return 0;
}

static void gpmi_nfc_platform_exit(unsigned int max_chip_count)
{
}

static const char *gpmi_nfc_partition_source_types[] = { "cmdlinepart", 0 };

static struct gpmi_nfc_platform_data  gpmi_nfc_platform_data = {
	.nfc_version             = 0,
	.boot_rom_version        = 0,
	.clock_name              = "gpmi",
	.platform_init           = gpmi_nfc_platform_init,
	.platform_exit           = gpmi_nfc_platform_exit,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 2,
	.boot_area_size_in_bytes = 20 * SZ_1M,
	.partition_source_types  = gpmi_nfc_partition_source_types,
	.partitions              = 0,
	.partition_count         = 0,
};

static struct resource gpmi_nfc_resources[] = {
	{
	 .name  = GPMI_NFC_GPMI_REGS_ADDR_RES_NAME,
	 .flags = IORESOURCE_MEM,
	 .start = GPMI_PHYS_ADDR,
	 .end   = GPMI_PHYS_ADDR + SZ_8K - 1,
	 },
	{
	 .name  = GPMI_NFC_GPMI_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_GPMI_ATTENTION,
	 .end   = IRQ_GPMI_ATTENTION,
	},
	{
	 .name  = GPMI_NFC_BCH_REGS_ADDR_RES_NAME,
	 .flags = IORESOURCE_MEM,
	 .start = BCH_PHYS_ADDR,
	 .end   = BCH_PHYS_ADDR + SZ_8K - 1,
	 },
	{
	 .name  = GPMI_NFC_BCH_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_BCH,
	 .end   = IRQ_BCH,
	 },
	{
	 .name  = GPMI_NFC_DMA_CHANNELS_RES_NAME,
	 .flags = IORESOURCE_DMA,
	 .start	= MXS_DMA_CHANNEL_AHB_APBH_GPMI0,
	 .end	= MXS_DMA_CHANNEL_AHB_APBH_GPMI3,
	 },
	{
	 .name  = GPMI_NFC_DMA_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_GPMI_DMA,
	 .end   = IRQ_GPMI_DMA,
	},
};

static void __init mx23_init_gpmi_nfc(void)
{
	struct platform_device  *pdev;

	pdev = mxs_get_device(GPMI_NFC_DRIVER_NAME, 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &gpmi_nfc_platform_data;
	pdev->resource          =  gpmi_nfc_resources;
	pdev->num_resources     = ARRAY_SIZE(gpmi_nfc_resources);
	mxs_add_device(pdev, 1);
}
#else
static void mx23_init_gpmi_nfc(void)
{
}
#endif

#if defined(CONFIG_MMC_MXS) || defined(CONFIG_MMC_MXS_MODULE)
static unsigned long mxs_mmc_setclock_mmc0(unsigned long hz)
{
	struct clk *ssp = clk_get(NULL, "ssp.0");

	clk_set_rate(ssp, 2 * hz);
	clk_put(ssp);

	return hz;
}

static struct mxs_mmc_platform_data mx23_mmc0_data = {
	.hw_init	= mxs_mmc_hw_init_mmc0,
	.hw_release	= mxs_mmc_hw_release_mmc0,
	.get_wp		= mxs_mmc_get_wp_mmc0,
	.cmd_pullup	= mxs_mmc_cmd_pullup_mmc0,
	 /*
	 Don't change ssp clock because ssp1 and ssp2 share one ssp clock source
	 ssp module have own divider.
	 .setclock	= mxs_mmc_setclock_mmc0,
	 */
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

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
static struct mxs_spi_platform_data ssp1_data = {
	.hw_pin_init = mxs_spi_enc_pin_init,
	.hw_pin_release = mxs_spi_enc_pin_release,
	.clk = "ssp.0",
};

static struct resource ssp1_resources[] = {
	{
		.start	= SSP1_PHYS_ADDR,
		.end	= SSP1_PHYS_ADDR + 0x1FFF,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_SSP1_DMA,
		.end	= IRQ_SSP1_DMA,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= IRQ_SSP_ERROR,
		.end	= IRQ_SSP_ERROR,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= MXS_DMA_CHANNEL_AHB_APBH_SSP1,
		.end	= MXS_DMA_CHANNEL_AHB_APBH_SSP1,
		.flags	= IORESOURCE_DMA,
	},
};

static void __init mx23_init_spi1(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-spi", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = ssp1_resources;
	pdev->num_resources = ARRAY_SIZE(ssp1_resources);
	pdev->dev.platform_data = &ssp1_data;

	mxs_add_device(pdev, 3);
}
#else
static void mx23_init_spi1(void)
{
	;
}
#endif

#define CMDLINE_DEVICE_CHOOSE(name, dev1, dev2)			\
	static char *cmdline_device_##name;			\
	static int cmdline_device_##name##_setup(char *dev)	\
	{							\
		cmdline_device_##name = dev + 1;		\
		return 0;					\
	}							\
	__setup(#name, cmdline_device_##name##_setup);		\
	void mx23_init_##name(void)				\
	{							\
		if (!cmdline_device_##name ||			\
			!strcmp(cmdline_device_##name, #dev1))	\
				mx23_init_##dev1();		\
		else if (!strcmp(cmdline_device_##name, #dev2))	\
				mx23_init_##dev2();		\
		else						\
			pr_err("Unknown %s assignment '%s'.\n",	\
				#name, cmdline_device_##name);	\
	}

CMDLINE_DEVICE_CHOOSE(ssp1, mmc, spi1)

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
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = battery_resource,
	pdev->num_resources = ARRAY_SIZE(battery_resource),
	pdev->dev.platform_data = &battery_data;
	mxs_add_device(pdev, 3);

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
static inline void mx23_init_spdif(void)
{
}
#endif

#if defined(CONFIG_MXS_PERSISTENT)
static const struct mxs_persistent_bit_config
mx23_persistent_bit_config[] = {
	{ .reg = 0, .start =  0, .width =  1,
		.name = "CLOCKSOURCE" },
	{ .reg = 0, .start =  1, .width =  1,
		.name = "ALARM_WAKE_EN" },
	{ .reg = 0, .start =  2, .width =  1,
		.name = "ALARM_EN" },
	{ .reg = 0, .start =  3, .width =  1,
		.name = "CLK_SECS" },
	{ .reg = 0, .start =  4, .width =  1,
		.name = "XTAL24MHZ_PWRUP" },
	{ .reg = 0, .start =  5, .width =  1,
		.name = "XTAL32MHZ_PWRUP" },
	{ .reg = 0, .start =  6, .width =  1,
		.name = "XTAL32_FREQ" },
	{ .reg = 0, .start =  7, .width =  1,
		.name = "ALARM_WAKE" },
	{ .reg = 0, .start =  8, .width =  5,
		.name = "MSEC_RES" },
	{ .reg = 0, .start = 13, .width =  1,
		.name = "DISABLE_XTALOK" },
	{ .reg = 0, .start = 14, .width =  2,
		.name = "LOWERBIAS" },
	{ .reg = 0, .start = 16, .width =  1,
		.name = "DISABLE_PSWITCH" },
	{ .reg = 0, .start = 17, .width =  1,
		.name = "AUTO_RESTART" },
	{ .reg = 0, .start = 18, .width = 14,
		.name = "SPARE_ANALOG" },

	{ .reg = 1, .start =  0, .width =  1,
		.name = "FORCE_RECOVERY" },
	{ .reg = 1, .start =  1, .width =  1,
		.name = "NAND_SECONDARY_BOOT" },
	{ .reg = 1, .start =  2, .width =  1,
		.name = "NAND_SDK_BLOCK_REWRITE" },
	{ .reg = 1, .start =  3, .width =  1,
		.name = "SD_SPEED_ENABLE" },
	{ .reg = 1, .start =  4, .width =  1,
		.name = "SD_INIT_SEQ_1_DISABLE" },
	{ .reg = 1, .start =  5, .width =  1,
		.name = "SD_CMD0_DISABLE" },
	{ .reg = 1, .start =  6, .width =  1,
		.name = "SD_INIT_SEQ_2_ENABLE" },
	{ .reg = 1, .start =  7, .width =  1,
		.name = "OTG_ATL_ROLE_BIT" },
	{ .reg = 1, .start =  8, .width =  1,
		.name = "OTG_HNP_BIT" },
	{ .reg = 1, .start =  9, .width =  1,
		.name = "USB_LOW_POWER_MODE" },
	{ .reg = 1, .start = 10, .width =  1,
		.name = "SKIP_CHECKDISK" },
	{ .reg = 1, .start = 11, .width =  1,
		.name = "USB_BOOT_PLAYER_MODE" },
	{ .reg = 1, .start = 12, .width =  1,
		.name = "ENUMERATE_500MA_TWICE" },
	{ .reg = 1, .start = 13, .width = 19,
		.name = "SPARE_GENERAL" },

	{ .reg = 2, .start =  0, .width = 32,
		.name = "SPARE_2" },
	{ .reg = 3, .start =  0, .width = 32,
		.name = "SPARE_3" },
	{ .reg = 4, .start =  0, .width = 32,
		.name = "SPARE_4" },
	{ .reg = 5, .start =  0, .width = 32,
		.name = "SPARE_5" },
};

static struct mxs_platform_persistent_data mx23_persistent_data = {
	.bit_config_tab = mx23_persistent_bit_config,
	.bit_config_cnt = ARRAY_SIZE(mx23_persistent_bit_config),
};

static struct resource mx23_persistent_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = RTC_PHYS_ADDR,
	 .end   = RTC_PHYS_ADDR + 0x2000 - 1,
	 },
};

static void mx23_init_persistent(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-persistent", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &mx23_persistent_data;
	pdev->resource = mx23_persistent_res,
	pdev->num_resources = ARRAY_SIZE(mx23_persistent_res),
	mxs_add_device(pdev, 3);
}
#else
static void mx23_init_persistent()
{
}
#endif

#if defined(CONFIG_FSL_OTP)
/* Building up eight registers's names of a bank */
#define BANK(a, b, c, d, e, f, g, h)	\
	{\
	("HW_OCOTP_"#a), ("HW_OCOTP_"#b), ("HW_OCOTP_"#c), ("HW_OCOTP_"#d), \
	("HW_OCOTP_"#e), ("HW_OCOTP_"#f), ("HW_OCOTP_"#g), ("HW_OCOTP_"#h) \
	}

#define BANKS		(4)
#define BANK_ITEMS	(8)
static const char *bank_reg_desc[BANKS][BANK_ITEMS] = {
	BANK(CUST0, CUST1, CUST2, CUST3, CRYPTO0, CRYPTO1, CRYPTO2, CRYPTO3),
	BANK(HWCAP0, HWCAP1, HWCAP2, HWCAP3, HWCAP4, HWCAP5, SWCAP, CUSTCAP),
	BANK(LOCK, OPS0, OPS1, OPS2, OPS3, UN0, UN1, UN2),
	BANK(ROM0, ROM1, ROM2, ROM3, ROM4, ROM5, ROM6, ROM7),
};

static struct fsl_otp_data otp_data = {
	.fuse_name	= (char **)bank_reg_desc,
	.regulator_name	= "vddio",
	.fuse_num	= BANKS * BANK_ITEMS,
};
#undef BANK
#undef BANKS
#undef BANK_ITEMS

static void mx23_init_otp(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("ocotp", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &otp_data;
	pdev->resource = NULL;
	pdev->num_resources = 0;
	mxs_add_device(pdev, 3);
}
#else
static void mx23_init_otp(void)
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
	mx23_init_ssp1();
	mx23_init_gpmi_nfc();
	mx23_init_spdif();
	mx23_init_lcdif();
	mx23_init_pxp();
	mx23_init_battery();
	mx23_init_persistent();
	mx23_init_otp();

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
