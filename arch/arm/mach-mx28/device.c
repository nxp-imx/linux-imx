/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/etherdevice.h>
#include <linux/fec.h>
#include <linux/gpmi-nfc.h>
#include <linux/fsl_devices.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/regs-timrot.h>
#include <mach/regs-lradc.h>
#include <mach/regs-ocotp.h>
#include <mach/device.h>
#include <mach/dma.h>
#include <mach/lradc.h>
#include <mach/lcdif.h>
#include <mach/ddi_bc.h>
#include <mach/pinctrl.h>

#include "regs-digctl.h"
#include "device.h"
#include "mx28evk.h"
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
static void __init mx28_init_pxp(void)
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
static void __init mx28_init_pxp(void)
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
static void __init mx28_init_viim(void)
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
static void __init mx28_init_viim(void)
{
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

#if defined(CONFIG_MTD_NAND_GPMI_NFC)

extern int enable_gpmi;

static int gpmi_nfc_platform_init(unsigned int max_chip_count)
{
	return !enable_gpmi;
}

static void gpmi_nfc_platform_exit(unsigned int max_chip_count)
{
}

static const char *gpmi_nfc_partition_source_types[] = { "cmdlinepart", 0 };

static struct gpmi_nfc_platform_data  gpmi_nfc_platform_data = {
	.nfc_version             = 1,
	.boot_rom_version        = 1,
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
	 .start = IRQ_GPMI,
	 .end   = IRQ_GPMI,
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
	 .end	= MXS_DMA_CHANNEL_AHB_APBH_GPMI7,
	 },
	{
	 .name  = GPMI_NFC_DMA_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = IRQ_GPMI_DMA,
	 .end   = IRQ_GPMI_DMA,
	},
};

static void __init mx28_init_gpmi_nfc(void)
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
static void mx28_init_gpmi_nfc(void)
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
	.caps 		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
				| MMC_CAP_DATA_DDR,
	.min_clk	= 400000,
	.max_clk	= 48000000,
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
	.caps 		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
				| MMC_CAP_DATA_DDR,
	.min_clk	= 400000,
	.max_clk	= 48000000,
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
	struct platform_device *pdev;

	if (mxs_get_type(PINID_SSP0_CMD) == PIN_FUN1) {
		pdev = mxs_get_device("mxs-mmc", 0);
		if (pdev == NULL || IS_ERR(pdev))
			return;
		pdev->resource = mmc0_resource;
		pdev->num_resources = ARRAY_SIZE(mmc0_resource);
		pdev->dev.platform_data = &mmc0_data;
		mxs_add_device(pdev, 2);
	}

	if (mxs_get_type(PINID_GPMI_RDY1) == PIN_FUN2) {
		pdev = mxs_get_device("mxs-mmc", 1);
		if (pdev == NULL || IS_ERR(pdev))
			return;
		pdev->resource = mmc1_resource;
		pdev->num_resources = ARRAY_SIZE(mmc1_resource);
		pdev->dev.platform_data = &mmc1_data;
		mxs_add_device(pdev, 2);
	}
}
#else
static void mx28_init_mmc(void)
{
}
#endif

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
static struct mxs_spi_platform_data spi_data = {
	.clk = "ssp.2",
	.slave_mode = 0,
};
static struct resource ssp2_resources[] = {
	{
		.start	= SSP2_PHYS_ADDR,
		.end	= SSP2_PHYS_ADDR + 0x2000 - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= MXS_DMA_CHANNEL_AHB_APBH_SSP2,
		.end	= MXS_DMA_CHANNEL_AHB_APBH_SSP2,
		.flags	= IORESOURCE_DMA,
	}, {
		.start	= IRQ_SSP2_DMA,
		.end	= IRQ_SSP2_DMA,
		.flags	= IORESOURCE_IRQ,
	}, {
		.start	= IRQ_SSP2,
		.end	= IRQ_SSP2,
		.flags	= IORESOURCE_IRQ,
	},
};

static void __init mx28_init_spi(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-spi", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = ssp2_resources;
	pdev->num_resources = ARRAY_SIZE(ssp2_resources);
	pdev->dev.platform_data = &spi_data;

	mxs_add_device(pdev, 3);
}
#else
static void mx28_init_spi(void)
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
static struct resource fec0_resource[] = {
	{
		.start  = ENET_PHYS_ADDR,
		.end    = ENET_PHYS_ADDR + 0x3fff,
		.flags  = IORESOURCE_MEM
	},
	{
		.start  = IRQ_ENET_MAC0,
		.end    = IRQ_ENET_MAC0,
		.flags  = IORESOURCE_IRQ
	},
};

static struct resource fec1_resource[] = {
	{
		.start  = ENET_PHYS_ADDR + 0x4000,
		.end    = ENET_PHYS_ADDR + 0x7fff,
		.flags  = IORESOURCE_MEM
	},
	{
		.start  = IRQ_ENET_MAC1,
		.end    = IRQ_ENET_MAC1,
		.flags  = IORESOURCE_IRQ
	},
};

extern int mx28evk_enet_gpio_init(void);
static struct fec_platform_data fec_pdata0 = {
	.phy = PHY_INTERFACE_MODE_RMII,
	.init = mx28evk_enet_gpio_init,
};

static struct fec_platform_data fec_pdata1 = {
	.phy = PHY_INTERFACE_MODE_RMII,
	.init = mx28evk_enet_gpio_init,
};

static void __init mx28_init_fec(void)
{
	struct platform_device *pdev;
	struct mxs_dev_lookup *lookup;
	struct fec_platform_data *pfec;
	int i;
	u32 val;

	__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN,
			IO_ADDRESS(OCOTP_PHYS_ADDR) + HW_OCOTP_CTRL_SET);

	while (BM_OCOTP_CTRL_BUSY &
		__raw_readl(IO_ADDRESS(OCOTP_PHYS_ADDR) + HW_OCOTP_CTRL))
		udelay(10);

	lookup = mxs_get_devices("mxs-fec");
	if (lookup == NULL || IS_ERR(lookup))
		return;

	for (i = 0; i < lookup->size; i++) {
		pdev = lookup->pdev + i;
		val =  __raw_readl(IO_ADDRESS(OCOTP_PHYS_ADDR) +
						HW_OCOTP_CUSTn(pdev->id));
		switch (pdev->id) {
		case 0:
			pdev->resource = fec0_resource;
			pdev->num_resources = ARRAY_SIZE(fec0_resource);
			pdev->dev.platform_data = &fec_pdata0;
			break;
		case 1:
			pdev->resource = fec1_resource;
			pdev->num_resources = ARRAY_SIZE(fec1_resource);
			pdev->dev.platform_data = &fec_pdata1;
			break;
		default:
			return;
		}

		pfec = (struct fec_platform_data *)pdev->dev.platform_data;
		pfec->mac[0] = 0x00;
		pfec->mac[1] = 0x04;
		pfec->mac[2] = (val >> 24) & 0xFF;
		pfec->mac[3] = (val >> 16) & 0xFF;
		pfec->mac[4] = (val >> 8) & 0xFF;
		pfec->mac[5] = (val >> 0) & 0xFF;

		mxs_add_device(pdev, 2);
	}
}
#else
static void __init mx28_init_fec(void)
{
	;
}
#endif

#if defined(CONFIG_FEC_L2SWITCH)
static struct resource l2switch_resources[] = {
	{
		.start  = ENET_PHYS_ADDR,
		.end    = ENET_PHYS_ADDR + 0x17FFC,
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

/* Define the fixed address of the L2 Switch hardware. */
static unsigned int switch_platform_hw[2] = {
	(0x800F8000),
	(0x800FC000),
};

static struct fec_platform_data fec_enet = {
	.phy = PHY_INTERFACE_MODE_RMII,
	.init = mx28evk_enet_gpio_init,
};

static struct switch_platform_data l2switch_data = {
	.id 		= 0,
	.fec_enet	= &fec_enet,
	.hash_table	= 0,
	.switch_hw	= switch_platform_hw,
};

static void __init mx28_init_l2switch(void)
{
	struct platform_device *pdev;
	struct switch_platform_data *pswitch;
	struct fec_platform_data *pfec;
	u32 val;

	__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN,
			IO_ADDRESS(OCOTP_PHYS_ADDR) + HW_OCOTP_CTRL_SET);

	while (BM_OCOTP_CTRL_BUSY &
		__raw_readl(IO_ADDRESS(OCOTP_PHYS_ADDR) + HW_OCOTP_CTRL))
		udelay(10);

	pdev = mxs_get_device("mxs-l2switch", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	val =  __raw_readl(IO_ADDRESS(OCOTP_PHYS_ADDR) +
					HW_OCOTP_CUSTn(pdev->id));
	pdev->resource = l2switch_resources;
	pdev->num_resources = ARRAY_SIZE(l2switch_resources);
	pdev->dev.platform_data = &l2switch_data;

	pswitch = (struct switch_platform_data *)pdev->dev.platform_data;
	pfec = pswitch->fec_enet;
	pfec->mac[0] = 0x00;
	pfec->mac[1] = 0x04;
	pfec->mac[2] = (val >> 24) & 0xFF;
	pfec->mac[3] = (val >> 16) & 0xFF;
	pfec->mac[4] = (val >> 8) & 0xFF;
	pfec->mac[5] = (val >> 0) & 0xFF;

	mxs_add_device(pdev, 2);
}
#else
static void __init mx28_init_l2switch(void)
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
	.btn_enable = BM_LRADC_CTRL0_BUTTON1_DETECT_ENABLE,
	.btn_irq_stat = BM_LRADC_CTRL1_BUTTON1_DETECT_IRQ,
	.btn_irq_ctrl = BM_LRADC_CTRL1_BUTTON1_DETECT_IRQ_EN,
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
	{
	.flags = IORESOURCE_IRQ,
	.start = IRQ_LRADC_BUTTON1,
	.end = IRQ_LRADC_BUTTON1,
	}
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

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)
static void flexcan_xcvr_enable(int id, int en)
{
	static int pwdn;
	if (en) {
		if (!pwdn++)
			gpio_set_value(MXS_PIN_TO_GPIO(PINID_SSP1_CMD), 1);
	} else {
		if (!--pwdn)
			gpio_set_value(MXS_PIN_TO_GPIO(PINID_SSP1_CMD), 0);
	}
}

struct flexcan_platform_data flexcan_data[] = {
	{
	.core_reg = NULL,
	.io_reg = NULL,
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 1,
	.br_rjw = 2,
	.br_presdiv = 2,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 7,
	.bcc = 1,
	.srx_dis = 1,
	.smp = 1,
	.boff_rec = 1,
	.ext_msg = 1,
	.std_msg = 1,
	},
	{
	.core_reg = NULL,
	.io_reg = NULL,
	.xcvr_enable = flexcan_xcvr_enable,
	.br_clksrc = 1,
	.br_rjw = 2,
	.br_presdiv = 2,
	.br_propseg = 2,
	.br_pseg1 = 3,
	.br_pseg2 = 7,
	.bcc = 1,
	.srx_dis = 1,
	.boff_rec = 1,
	.ext_msg = 1,
	.std_msg = 1,
	},
};

static struct resource flexcan0_resources[] = {
	{
	    .start = CAN0_PHYS_ADDR,
	    .end = CAN0_PHYS_ADDR + 0x1FFF,
	    .flags = IORESOURCE_MEM,},
	{
	    .start = IRQ_CAN0,
	    .end = IRQ_CAN0,
	    .flags = IORESOURCE_IRQ,},
};
static struct resource flexcan1_resources[] = {
	{
	    .start = CAN1_PHYS_ADDR,
	    .end = CAN1_PHYS_ADDR + 0x1FFF,
	    .flags = IORESOURCE_MEM,},
	{
	    .start = IRQ_CAN1,
	    .end = IRQ_CAN1,
	    .flags = IORESOURCE_IRQ,},
};

static inline void mx28_init_flexcan(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("FlexCAN", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = flexcan0_resources;
	pdev->num_resources = ARRAY_SIZE(flexcan0_resources);
	pdev->dev.platform_data = &flexcan_data[0];
	mxs_add_device(pdev, 2);

	pdev = mxs_get_device("FlexCAN", 1);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->resource = flexcan1_resources;
	pdev->num_resources = ARRAY_SIZE(flexcan1_resources);
	pdev->dev.platform_data = &flexcan_data[1];
	mxs_add_device(pdev, 2);
}
#else
static inline void mx28_init_flexcan(void)
{
}
#endif
#if defined(CONFIG_BATTERY_MXS)
/* battery info data */
static ddi_bc_Cfg_t battery_data = {
	.u32StateMachinePeriod		 = 100,		/* ms */
	.u16CurrentRampSlope		 = 75,		/* mA/s */
	.u16ConditioningThresholdVoltage = 2900, 	/* mV */
	.u16ConditioningMaxVoltage	 = 3000,	/* mV */
	.u16ConditioningCurrent		 = 160,		/* mA */
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
	.u8BatteryTempChannel		 = 0,		/* LRADC 0 */
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
		.start  = IRQ_DCDC4P2_BRNOUT,
		.end    = IRQ_DCDC4P2_BRNOUT,
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
		.start  = IRQ_VDDA_BRNOUT,
		.end    = IRQ_VDDA_BRNOUT,
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

static void mx28_init_battery(void)
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
static void mx28_init_battery(void)
{
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

static void __init mx28_init_dcp(void)
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
static void __init mx28_init_dcp(void)
{
	;
}
#endif

#if defined(CONFIG_SND_MXS_SOC_DAI) || defined(CONFIG_SND_MXS_SOC_DAI_MODULE)
static int audio_clk_init(struct clk *clk)
{
	struct clk *pll_clk;
	struct clk *saif_mclk0;
	struct clk *saif_mclk1;
	int ret = -EINVAL;
	if (IS_ERR(clk)) {
		pr_err("%s:failed to get clk\n", __func__);
		goto err_clk_init;
	}
	pll_clk = clk_get(NULL, "pll.0");
	if (IS_ERR(pll_clk)) {
		pr_err("%s:failed to get pll_clk\n", __func__);
		goto err_clk_init;
	}
	saif_mclk0 = clk_get(NULL, "saif_mclk.0");
	if (IS_ERR(saif_mclk0)) {
		pr_err("%s:failed to get saif_mclk\n", __func__);
		goto err_clk_init;
	}
	saif_mclk1 = clk_get(NULL, "saif_mclk.1");
	if (IS_ERR(saif_mclk1)) {
		pr_err("%s:failed to get saif_mclk\n", __func__);
		goto err_clk_init;
	}
	ret = clk_set_parent(clk, pll_clk);
	if (ret) {
		pr_err("%s:failed to set parent clk\n", __func__);
		goto err_clk_init;
	}
	ret = 0;
	/*set a default freq of 12M to sgtl5000*/
	clk_set_rate(clk, 12000000);
	clk_enable(clk);
	/*set the saif clk mux, saif0/saif1 both use saif0 clk*/
	__raw_writel(BF_DIGCTL_CTRL_SAIF_CLKMUX_SEL(0x2), \
			IO_ADDRESS(DIGCTL_PHYS_ADDR) + HW_DIGCTL_CTRL);

	/*enable saif0/saif1 clk output*/
	clk_enable(saif_mclk0);
	clk_enable(saif_mclk1);
err_clk_init:
	return ret;
}

static int audio_clk_finit(void)
{
	struct clk *saif_clk;
	struct clk *saif_mclk0;
	struct clk *saif_mclk1;
	int ret = 0;
	saif_clk = clk_get(NULL, "saif.0");
	if (IS_ERR(saif_clk)) {
		pr_err("%s:failed to get saif_clk\n", __func__);
		ret = -EINVAL;
		goto err_clk_finit;
	}
	clk_disable(saif_clk);

	saif_mclk0 = clk_get(NULL, "saif_mclk.0");
	if (IS_ERR(saif_mclk0)) {
		pr_err("%s:failed to get saif_mclk\n", __func__);
		goto err_clk_finit;
	}
	clk_disable(saif_mclk0);

	saif_mclk1 = clk_get(NULL, "saif_mclk.1");
	if (IS_ERR(saif_mclk1)) {
		pr_err("%s:failed to get saif_mclk\n", __func__);
		goto err_clk_finit;
	}
	clk_disable(saif_mclk1);
err_clk_finit:
	return ret;
}

static struct mxs_audio_platform_data audio_plat_data;
#endif

#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
void __init mx28_init_audio(void)
{	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-sgtl5000", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	mxs_add_device(pdev, 3);
	audio_plat_data.saif_mclock = clk_get(NULL, "saif.0");
	audio_clk_init(audio_plat_data.saif_mclock);
	pdev->dev.platform_data = &audio_plat_data;
}
#else
void __init mx28_init_audio(void)
{
}
#endif

#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
       defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
void __init mx28_init_spdif(void)
{	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-spdif", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	mxs_add_device(pdev, 3);
}
#else
static inline mx28_init_spdif(void)
{
}
#endif

#if defined(CONFIG_MXS_PERSISTENT)
static const struct mxs_persistent_bit_config
mx28_persistent_bit_config[] = {
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
	{ .reg = 0, .start = 18, .width = 1,
		.name = "ENABLE_LRADC_PWRUP" },
	{ .reg = 0, .start = 20, .width = 1,
		.name = "THERMAL_RESET" },
	{ .reg = 0, .start = 21, .width = 1,
		.name = "EXTERNAL_RESET" },
	{ .reg = 0, .start = 28, .width = 4,
		.name = "ADJ_POSLIMITBUCK" },
	{ .reg = 1, .start =  0, .width =  1,
		.name = "FORCE_RECOVERY" },
	{ .reg = 1, .start =  1, .width =  1,
		.name = "ROM_REDUNDANT_BOOT" },
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

static struct mxs_platform_persistent_data mx28_persistent_data = {
	.bit_config_tab = mx28_persistent_bit_config,
	.bit_config_cnt = ARRAY_SIZE(mx28_persistent_bit_config),
};

static struct resource mx28_persistent_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = RTC_PHYS_ADDR,
	 .end   = RTC_PHYS_ADDR + 0x2000 - 1,
	 },
};

static void mx28_init_persistent(void)
{
	struct platform_device *pdev;
	pdev = mxs_get_device("mxs-persistent", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &mx28_persistent_data;
	pdev->resource = mx28_persistent_res,
	pdev->num_resources = ARRAY_SIZE(mx28_persistent_res),
	mxs_add_device(pdev, 3);
}
#else
static void mx28_init_persistent()
{
}
#endif

#if defined(CONFIG_MXS_PERFMON)

static struct mxs_perfmon_bit_config
mx28_perfmon_bit_config[] = {
	{.field = (1 << 0),	.name = "MID0-PXP" },
	{.field = (1 << 1),	.name = "MID1-LCDIF" },
	{.field = (1 << 2),	.name = "MID2-BCH" },
	{.field = (1 << 3),	.name = "MID3-DCP" }
};

static struct mxs_platform_perfmon_data mx28_perfmon_data = {
	.bit_config_tab = mx28_perfmon_bit_config,
	.bit_config_cnt = ARRAY_SIZE(mx28_perfmon_bit_config),
};

static struct resource mx28_perfmon_res[] = {
	{
	 .flags = IORESOURCE_MEM,
	 .start = PERFMON_PHYS_ADDR,
	 .end   = PERFMON_PHYS_ADDR + 0x1000 - 1,
    },
};

static void mx28_init_perfmon(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-perfmon", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;
	pdev->dev.platform_data = &mx28_perfmon_data;
	pdev->resource = mx28_perfmon_res,
	pdev->num_resources = ARRAY_SIZE(mx28_perfmon_res),
	mxs_add_device(pdev, 3);
}

#else

static void mx28_init_perfmon()
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

#define BANKS		(5)
#define BANK_ITEMS	(8)
static const char *bank_reg_desc[BANKS][BANK_ITEMS] = {
	BANK(CUST0, CUST1, CUST2, CUST3, CRYPTO0, CRYPTO1, CRYPTO2, CRYPTO3),
	BANK(HWCAP0, HWCAP1, HWCAP2, HWCAP3, HWCAP4, HWCAP5, SWCAP, CUSTCAP),
	BANK(LOCK, OPS0, OPS1, OPS2, OPS3, UN0, UN1, UN2),
	BANK(ROM0, ROM1, ROM2, ROM3, ROM4, ROM5, ROM6, ROM7),
	BANK(SRK0, SRK1, SRK2, SRK3, SRK4, SRK5, SRK6, SRK7),
};

static struct fsl_otp_data otp_data = {
	.fuse_name	= (char **)bank_reg_desc,
	.regulator_name	= "vddio",
	.fuse_num	= BANKS * BANK_ITEMS,
};
#undef BANK
#undef BANKS
#undef BANK_ITEMS

static void __init mx28_init_otp(void)
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
static void mx28_init_otp(void)
{
}
#endif

int __init mx28_device_init(void)
{
	mx28_init_dma();
	mx28_init_viim();
	mx28_init_duart();
	mx28_init_i2c();
	mx28_init_lradc();
	mx28_init_auart();
	mx28_init_mmc();
	mx28_init_spi();
	mx28_init_gpmi_nfc();
	mx28_init_wdt();
	mx28_init_rtc();
	mx28_init_fec();
	mx28_init_l2switch();
	mx28_init_flexcan();
	mx28_init_kbd();
	mx28_init_ts();
	mx28_init_audio();
	mx28_init_spdif();
	mx28_init_lcdif();
	mx28_init_pxp();
	mx28_init_dcp();
	mx28_init_battery();
	mx28_init_persistent();
	mx28_init_perfmon();
	mx28_init_otp();
	return 0;
}

static struct __initdata map_desc mx28_io_desc[] = {
	{
	 .virtual = MX28_SOC_IO_VIRT_BASE,
	 .pfn = __phys_to_pfn(MX28_SOC_IO_PHYS_BASE),
	 .length = MX28_SOC_IO_AREA_SIZE,
	 .type = MT_DEVICE,
	 },
	 {
	 .virtual = MX28_OCRAM_BASE,
	 .pfn = __phys_to_pfn(MX28_OCRAM_PHBASE),
	 .length = MX28_OCRAM_SIZE,
	 .type	= MT_DEVICE,
	 }
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

