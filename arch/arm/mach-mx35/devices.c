/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/spba.h>
#include <mach/sdma.h>

#include "iomux.h"
#include "sdma_script_code.h"
#include "sdma_script_code_v2.h"
#include "board-mx35_3stack.h"

void mxc_sdma_get_script_info(sdma_script_start_addrs * sdma_script_addr)
{
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1) {
		sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
		sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
		sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
		sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;

		sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
		sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;

		sdma_script_addr->mxc_sdma_uart_2_per_addr = uart_2_per_ADDR;
		sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;

		sdma_script_addr->mxc_sdma_per_2_per_addr = p_2_p_ADDR;

		sdma_script_addr->mxc_sdma_uartsh_2_per_addr =
		    uartsh_2_per_ADDR;
		sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr =
		    uartsh_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;

		sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;

		sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR;
		sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR;
		sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;

		sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;

		sdma_script_addr->mxc_sdma_spdif_2_mcu_addr = spdif_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_spdif_addr = mcu_2_spdif_ADDR;

		sdma_script_addr->mxc_sdma_asrc_2_mcu_addr = asrc__mcu_ADDR;

		sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
		sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr =
		    ext_mem__ipu_ram_ADDR;
		sdma_script_addr->mxc_sdma_descrambler_addr = -1;

		sdma_script_addr->mxc_sdma_start_addr =
		    (unsigned short *)sdma_code;
		sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
		sdma_script_addr->mxc_sdma_ram_code_start_addr =
		    RAM_CODE_START_ADDR;
	} else {
		sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR_V2;
		sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
		sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
		sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;

		sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
		sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;

		sdma_script_addr->mxc_sdma_uart_2_per_addr = uart_2_per_ADDR_V2;
		sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR_V2;
		sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR_V2;
		sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR_V2;

		sdma_script_addr->mxc_sdma_per_2_per_addr = p_2_p_ADDR_V2;

		sdma_script_addr->mxc_sdma_uartsh_2_per_addr =
		    uartsh_2_per_ADDR_V2;
		sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr =
		    uartsh_2_mcu_ADDR_V2;
		sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR_V2;
		sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR_V2;

		sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR_V2;
		sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR_V2;

		sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR_V2;
		sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR_V2;
		sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR_V2;
		sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR_V2;

		sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;

		sdma_script_addr->mxc_sdma_spdif_2_mcu_addr =
		    spdif_2_mcu_ADDR_V2;
		sdma_script_addr->mxc_sdma_mcu_2_spdif_addr =
		    mcu_2_spdif_ADDR_V2;

		sdma_script_addr->mxc_sdma_asrc_2_mcu_addr = asrc__mcu_ADDR_V2;

		sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
		sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr =
		    ext_mem__ipu_ram_ADDR_V2;
		sdma_script_addr->mxc_sdma_descrambler_addr = -1;

		sdma_script_addr->mxc_sdma_start_addr =
		    (unsigned short *)sdma_code_v2;
		sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
		sdma_script_addr->mxc_sdma_ram_code_start_addr =
		    RAM_CODE_START_ADDR_V2;
	}
}

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_RTC_MXC) || defined(CONFIG_RTC_MXC_MODULE)
static struct resource rtc_resources[] = {
	{
	 .start = RTC_BASE_ADDR,
	 .end = RTC_BASE_ADDR + 0x30,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_INT_RTC,
	 .flags = IORESOURCE_IRQ,
	 },
};
static struct platform_device mxc_rtc_device = {
	.name = "mxc_rtc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};
static void mxc_init_rtc(void)
{
	(void)platform_device_register(&mxc_rtc_device);
}
#else
static inline void mxc_init_rtc(void)
{
}
#endif

#if defined(CONFIG_MXC_MC9SDZ60_RTC) || defined(CONFIG_MXC_MC9SDZ60_RTC_MODULE)
static struct resource pmic_rtc_resources[] = {
	{
	 .start = MXC_PSEUDO_IRQ_RTC,
	 .flags = IORESOURCE_IRQ,
	 },
};
static struct platform_device pmic_rtc_device = {
	.name = "pmic_rtc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(pmic_rtc_resources),
	.resource = pmic_rtc_resources,
};
static void pmic_init_rtc(void)
{
	platform_device_register(&pmic_rtc_device);
}
#else
static void pmic_init_rtc(void)
{
}
#endif

#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)
static struct resource wdt_resources[] = {
	{
	 .start = WDOG1_BASE_ADDR,
	 .end = WDOG1_BASE_ADDR + 0x30,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device mxc_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};

static void mxc_init_wdt(void)
{
	(void)platform_device_register(&mxc_wdt_device);
}
#else
static inline void mxc_init_wdt(void)
{
}
#endif

#if defined(CONFIG_MXC_IPU) || defined(CONFIG_MXC_IPU_MODULE)
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
};

static struct resource ipu_resources[] = {
	{
	 .start = IPU_CTRL_BASE_ADDR,
	 .end = IPU_CTRL_BASE_ADDR + SZ_4K,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_INT_IPU_SYN,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .start = MXC_INT_IPU_ERR,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device mxc_ipu_device = {
	.name = "mxc_ipu",
	.id = -1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ipu_data,
		},
	.num_resources = ARRAY_SIZE(ipu_resources),
	.resource = ipu_resources,
};

static void mxc_init_ipu(void)
{
	platform_device_register(&mxc_ipu_device);
}
#else
static inline void mxc_init_ipu(void)
{
}
#endif

/* SPI controller and device data */
#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)

#ifdef CONFIG_SPI_MXC_SELECT1
/*!
 * Resource definition for the CSPI1
 */
static struct resource mxcspi1_resources[] = {
	[0] = {
	       .start = CSPI1_BASE_ADDR,
	       .end = CSPI1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI1,
	       .end = MXC_INT_CSPI1,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI1 */
static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 7,
};

/*! Device Definition for MXC CSPI1 */
static struct platform_device mxcspi1_device = {
	.name = "mxc_spi",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi1_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi1_resources),
	.resource = mxcspi1_resources,
};

#endif				/* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
/*!
 * Resource definition for the CSPI2
 */
static struct resource mxcspi2_resources[] = {
	[0] = {
	       .start = CSPI2_BASE_ADDR,
	       .end = CSPI2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI2,
	       .end = MXC_INT_CSPI2,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI2 */
static struct mxc_spi_master mxcspi2_data = {
	.maxchipselect = 4,
	.spi_version = 7,
};

/*! Device Definition for MXC CSPI2 */
static struct platform_device mxcspi2_device = {
	.name = "mxc_spi",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi2_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi2_resources),
	.resource = mxcspi2_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT2 */

static inline void mxc_init_spi(void)
{
	/* SPBA configuration for CSPI2 - MCU is set */
	spba_take_ownership(SPBA_CSPI2, SPBA_MASTER_A);
#ifdef CONFIG_SPI_MXC_SELECT1
	if (platform_device_register(&mxcspi1_device) < 0)
		printk(KERN_ERR "Error: Registering the SPI Controller_1\n");
#endif				/* CONFIG_SPI_MXC_SELECT1 */
#ifdef CONFIG_SPI_MXC_SELECT2
	if (platform_device_register(&mxcspi2_device) < 0)
		printk(KERN_ERR "Error: Registering the SPI Controller_2\n");
#endif				/* CONFIG_SPI_MXC_SELECT2 */
}
#else
static inline void mxc_init_spi(void)
{
}
#endif

/* I2C controller and device data */
#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT1
/*!
 * Resource definition for the I2C1
 */
static struct resource mxci2c1_resources[] = {
	[0] = {
	       .start = I2C_BASE_ADDR,
	       .end = I2C_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C,
	       .end = MXC_INT_I2C,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c1_data = {
	.i2c_clk = 100000,
};
#endif

/*!
 * Resource definition for the I2C2
 */
static struct resource mxci2c2_resources[] = {
	[0] = {
	       .start = I2C2_BASE_ADDR,
	       .end = I2C2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C2,
	       .end = MXC_INT_I2C2,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c2_data = {
	.i2c_clk = 100000,
};

#ifdef CONFIG_I2C_MXC_SELECT3
/*!
 * Resource definition for the I2C3
 */
static struct resource mxci2c3_resources[] = {
	[0] = {
	       .start = I2C3_BASE_ADDR,
	       .end = I2C3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_I2C3,
	       .end = MXC_INT_I2C3,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC I2C */
static struct mxc_i2c_platform_data mxci2c3_data = {
	.i2c_clk = 100000,
};
#endif

/*! Device Definition for MXC I2C1 */
static struct platform_device mxci2c_devices[] = {
#ifdef CONFIG_I2C_MXC_SELECT1
	{
	 .name = "mxc_i2c",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c1_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c1_resources),
	 .resource = mxci2c1_resources,},
#endif
	{
	 .name = "mxc_i2c_slave",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c2_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c2_resources),
	 .resource = mxci2c2_resources,},
#ifdef CONFIG_I2C_MXC_SELECT3
	{
	 .name = "mxc_i2c",
	 .id = 2,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c3_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c3_resources),
	 .resource = mxci2c3_resources,},
#endif
};

static inline void mxc_init_i2c(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mxci2c_devices); i++) {
		if (platform_device_register(&mxci2c_devices[i]) < 0)
			dev_err(&mxci2c_devices[i].dev,
				"Unable to register I2C device\n");
	}
}
#else
static inline void mxc_init_i2c(void)
{
}
#endif

struct mxc_gpio_port mxc_gpio_ports[] = {
	[0] = {
	 .chip.label = "gpio-0",
	 .base = IO_ADDRESS(GPIO1_BASE_ADDR),
	 .irq = MXC_INT_GPIO1,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START
	 },
	[1] = {
	 .chip.label = "gpio-1",
	 .base = IO_ADDRESS(GPIO2_BASE_ADDR),
	 .irq = MXC_INT_GPIO2,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32
	 },
	[2] = {
	 .chip.label = "gpio-2",
	 .base = IO_ADDRESS(GPIO3_BASE_ADDR),
	 .irq = MXC_INT_GPIO3,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	 }
};

int __init mxc_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

static struct platform_device mxc_dma_device = {
	.name = "mxc_dma",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static inline void mxc_init_dma(void)
{
	(void)platform_device_register(&mxc_dma_device);
}

static struct resource spdif_resources[] = {
	{
	 .start = SPDIF_BASE_ADDR,
	 .end = SPDIF_BASE_ADDR + 0x50,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 1,
	.spdif_clk_44100 = 3,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 0,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct platform_device mxc_alsa_spdif_device = {
	.name = "mxc_alsa_spdif",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_spdif_data,
		},
	.num_resources = ARRAY_SIZE(spdif_resources),
	.resource = spdif_resources,
};

static inline void mxc_init_spdif(void)
{
	mxc_spdif_data.spdif_clk = clk_get(NULL, "spdif_ipg_clk");
	clk_put(mxc_spdif_data.spdif_clk);
	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);
	mxc_spdif_data.spdif_audio_clk = clk_get(NULL, "spdif_audio_clk");
	clk_put(mxc_spdif_data.spdif_audio_clk);
	platform_device_register(&mxc_alsa_spdif_device);
}

static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-3stack-wm8580",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static void mxc_init_surround_audio(void)
{
	platform_device_register(&mxc_alsa_surround_device);
}

static struct mxc_audio_platform_data mxc_bt_audio_data;

static struct platform_device mxc_bt_alsa_device = {
	.name = "imx-3stack-bt",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_bt_audio_data,
		},

};

static void mxc_init_bt_audio(void)
{
	mxc_bt_audio_data.src_port = 2;
	mxc_bt_audio_data.ext_port = 5;
	mxc_bt_audio_data.ext_ram = 1;
	platform_device_register(&mxc_bt_alsa_device);
}

static struct resource asrc_resources[] = {
	{
	 .start = ASRC_BASE_ADDR,
	 .end = ASRC_BASE_ADDR + 0x9C,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct mxc_asrc_platform_data mxc_asrc_data;

static struct platform_device mxc_asrc_device = {
	.name = "mxc_asrc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_asrc_data,
		},
	.num_resources = ARRAY_SIZE(asrc_resources),
	.resource = asrc_resources,
};

static inline void mxc_init_asrc(void)
{
	if (cpu_is_mx35_rev(CHIP_REV_2_0) < 1)
		mxc_asrc_data.channel_bits = 3;
	else
		mxc_asrc_data.channel_bits = 4;

	mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	clk_put(mxc_asrc_data.asrc_core_clk);
	mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_audio_clk");
	clk_set_rate(mxc_asrc_data.asrc_audio_clk, 768000);
	clk_put(mxc_asrc_data.asrc_audio_clk);
	platform_device_register(&mxc_asrc_device);
}

#if defined(CONFIG_CAN_FLEXCAN) || defined(CONFIG_CAN_FLEXCAN_MODULE)

static struct resource flexcan1_resources[] = {
	{
	 .start = CAN1_BASE_ADDR,
	 .end = CAN1_BASE_ADDR + 0x97F,
	 .flags = IORESOURCE_MEM,},
	{
	 .start = MXC_INT_CAN1,
	 .end = MXC_INT_CAN1,
	 .flags = IORESOURCE_IRQ,}
};

static struct resource flexcan2_resources[] = {
	{
	 .start = CAN2_BASE_ADDR,
	 .end = CAN2_BASE_ADDR + 0x97F,
	 .flags = IORESOURCE_MEM,},
	{
	 .start = MXC_INT_CAN2,
	 .end = MXC_INT_CAN2,
	 .flags = IORESOURCE_IRQ,}
};

static struct platform_device flexcan_devices[] = {
	{
	 .name = "FlexCAN",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &flexcan_data[0],
		 },
	 .num_resources = ARRAY_SIZE(flexcan1_resources),
	 .resource = flexcan1_resources,},
	{
	 .name = "FlexCAN",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &flexcan_data[1],
		 },
	 .num_resources = ARRAY_SIZE(flexcan2_resources),
	 .resource = flexcan2_resources,},
};

static inline void mxc_init_flexcan(void)
{
	platform_device_register(&flexcan_devices[0]);
	platform_device_register(&flexcan_devices[1]);
}
#else
static inline void mxc_init_flexcan(void)
{
}
#endif

#if defined(CONFIG_HW_RANDOM_FSL_RNGC) || \
defined(CONFIG_HW_RANDOM_FSL_RNGC_MODULE)
static struct resource rngc_resources[] = {
	{
	 .start = RNGC_BASE_ADDR,
	 .end = RNGC_BASE_ADDR + 0x34,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_INT_RNG,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device fsl_rngc_device = {
	.name = "fsl_rngc",
	.id = -1,
	.num_resources = ARRAY_SIZE(rngc_resources),
	.resource = rngc_resources,
};

static inline void mxc_init_rngc(void)
{
	platform_device_register(&fsl_rngc_device);
}
#else
static inline void mxc_init_rngc(void)
{
}
#endif

#if defined(CONFIG_MXC_IIM) || defined(CONFIG_MXC_IIM_MODULE)
static struct resource mxc_iim_resources[] = {
	{
	 .start = IIM_BASE_ADDR,
	 .end = IIM_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device mxc_iim_device = {
	.name = "mxc_iim",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(mxc_iim_resources),
	.resource = mxc_iim_resources
};

static inline void mxc_init_iim(void)
{
	if (platform_device_register(&mxc_iim_device) < 0)
		dev_err(&mxc_iim_device.dev,
			"Unable to register mxc iim device\n");
}
#else
static inline void mxc_init_iim(void)
{
}
#endif

static struct resource mxc_gpu_resources[] = {
	{
		.start = MXC_INT_GPU2D,
		.end = MXC_INT_GPU2D,
		.name = "gpu_2d_irq",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device gpu_device = {
	.name = "mxc_gpu",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(mxc_gpu_resources),
	.resource = mxc_gpu_resources,
};

static void __init mxc_init_gpu(void)
{
	platform_device_register(&gpu_device);
}

int __init mxc_init_devices(void)
{
	mxc_init_wdt();
	mxc_init_ipu();
	mxc_init_spi();
	mxc_init_i2c();
	pmic_init_rtc();
	mxc_init_rtc();
	mxc_init_dma();
	mxc_init_bt_audio();
	mxc_init_spdif();
	mxc_init_surround_audio();
	mxc_init_asrc();
	mxc_init_flexcan();
	mxc_init_rngc();
	mxc_init_iim();
	mxc_init_gpu();

	/* SPBA configuration for SSI2 - SDMA and MCU are set */
	spba_take_ownership(SPBA_SSI2, SPBA_MASTER_C | SPBA_MASTER_A);
	return 0;
}
