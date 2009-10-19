/*
 * Author: MontaVista Software, Inc.
 *       <source@mvista.com>
 *
 * Based on the OMAP devices.c
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pmic_external.h>

#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/pmic_power.h>
#include <mach/spba.h>
#include <mach/sdma.h>
#include <mach/mxc_dptc.h>
#include <mach/gpio.h>

#include "iomux.h"
#include "crm_regs.h"
#include "sdma_script_code.h"
#include "sdma_script_code_pass2.h"

extern struct dptc_wp dptc_wp_allfreq_26ckih[DPTC_WP_SUPPORTED];
extern struct dptc_wp dptc_wp_allfreq_26ckih_TO_2_0[DPTC_WP_SUPPORTED];
extern struct dptc_wp dptc_wp_allfreq_27ckih_TO_2_0[DPTC_WP_SUPPORTED];
/*
 * Clock structures
 */
static struct clk *ckih_clk;

void mxc_sdma_get_script_info(sdma_script_start_addrs * sdma_script_addr)
{
	if (cpu_is_mx31_rev(CHIP_REV_1_0) == 1) {
		sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
		sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
		sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
		sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;
		sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;
		sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_start_addr =
		    (unsigned short *)sdma_code;
		sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr =
		    uartsh_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
		sdma_script_addr->mxc_sdma_ram_code_start_addr =
		    RAM_CODE_START_ADDR;
		sdma_script_addr->mxc_sdma_dptc_dvfs_addr = dptc_dvfs_ADDR;
		sdma_script_addr->mxc_sdma_firi_2_mcu_addr = firi_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = mshc_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_per_2_app_addr = -1;
		sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
		sdma_script_addr->mxc_sdma_per_2_shp_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_firi_addr = mcu_2_firi_ADDR;
		sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = mcu_2_mshc_ADDR;
		sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;
		sdma_script_addr->mxc_sdma_uartsh_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_shp_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_uart_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_app_2_per_addr = -1;
	} else {
		sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR_2;
		sdma_script_addr->mxc_sdma_ap_2_ap_fixed_addr =
		    ap_2_ap_fixed_addr_ADDR_2;
		sdma_script_addr->mxc_sdma_ap_2_bp_addr = ap_2_bp_ADDR_2;
		sdma_script_addr->mxc_sdma_ap_2_ap_fixed_addr =
		    ap_2_ap_fixed_addr_ADDR_2;
		sdma_script_addr->mxc_sdma_bp_2_ap_addr = bp_2_ap_ADDR_2;
		sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
		sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR_2;
		sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR_2;
		sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;
		sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_start_addr =
		    (unsigned short *)sdma_code_2;
		sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr =
		    uartsh_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE_2;
		sdma_script_addr->mxc_sdma_ram_code_start_addr =
		    RAM_CODE_START_ADDR_2;
		sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
		sdma_script_addr->mxc_sdma_firi_2_mcu_addr = firi_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = mshc_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_per_2_app_addr = per_2_app_ADDR_2;
		sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
		sdma_script_addr->mxc_sdma_per_2_shp_addr = per_2_shp_ADDR_2;
		sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR_2;
		sdma_script_addr->mxc_sdma_mcu_2_firi_addr = mcu_2_firi_ADDR_2;
		sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = mcu_2_mshc_ADDR_2;
		sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR_2;
		sdma_script_addr->mxc_sdma_uartsh_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_shp_2_per_addr = shp_2_per_ADDR_2;
		sdma_script_addr->mxc_sdma_uart_2_per_addr = -1;
		sdma_script_addr->mxc_sdma_app_2_per_addr = app_2_per_ADDR_2;
	}
}

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 0,
};

static struct platform_device mxc_w1_devices = {
	.name = "mxc_w1",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_w1_data,
		},
	.id = 0
};

static void mxc_init_owire(void)
{
	(void)platform_device_register(&mxc_w1_devices);
}
#else
static inline void mxc_init_owire(void)
{
}
#endif

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
	.rev = 1,
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

#if  defined(CONFIG_SND_MXC_PMIC) || defined(CONFIG_SND_MXC_PMIC_MODULE)
static struct mxc_audio_platform_data mxc_audio_data;

static struct platform_device mxc_alsa_device = {
	.name = "mxc_alsa",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_audio_data,
		},

};

static void mxc_init_audio(void)
{
	struct clk *pll_clk;
	pll_clk = clk_get(NULL, "usb_pll");
	mxc_audio_data.ssi_clk[0] = clk_get(NULL, "ssi_clk.0");
	clk_set_parent(mxc_audio_data.ssi_clk[0], pll_clk);
	clk_put(mxc_audio_data.ssi_clk[0]);
	if (machine_is_mx31_3ds()) {
		mxc_audio_data.ssi_num = 1;
	} else {
		mxc_audio_data.ssi_num = 2;
		mxc_audio_data.ssi_clk[1] = clk_get(NULL, "ssi_clk.1");
		clk_set_parent(mxc_audio_data.ssi_clk[1], pll_clk);
		clk_put(mxc_audio_data.ssi_clk[1]);
	}
	clk_put(pll_clk);
	mxc_audio_data.src_port = 0;
	platform_device_register(&mxc_alsa_device);
}
#else

static void mxc_init_audio(void)
{
}

#endif

#if defined(CONFIG_MXC_SSI) || defined(CONFIG_MXC_SSI_MODULE)
/*!
 * Resource definition for the SSI
 */
static struct resource mxcssi2_resources[] = {
	[0] = {
	       .start = SSI2_BASE_ADDR,
	       .end = SSI2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static struct resource mxcssi1_resources[] = {
	[0] = {
	       .start = SSI1_BASE_ADDR,
	       .end = SSI1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

/*! Device Definition for MXC SSI */
static struct platform_device mxc_ssi1_device = {
	.name = "mxc_ssi",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_audio_data,
		},
	.num_resources = ARRAY_SIZE(mxcssi1_resources),
	.resource = mxcssi1_resources,
};

static struct platform_device mxc_ssi2_device = {
	.name = "mxc_ssi",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_audio_data,
		},
	.num_resources = ARRAY_SIZE(mxcssi2_resources),
	.resource = mxcssi2_resources,
};

static void mxc_init_ssi(void)
{
	platform_device_register(&mxc_ssi1_device);
	platform_device_register(&mxc_ssi2_device);
}
#else

static void mxc_init_ssi(void)
{
}
#endif

/*!
 * This is platform device structure for adding SCC
 */
#if defined(CONFIG_MXC_SECURITY_SCC) || defined(CONFIG_MXC_SECURITY_SCC_MODULE)
static struct platform_device mxc_scc_device = {
	.name = "mxc_scc",
	.id = 0,
};

static void mxc_init_scc(void)
{
	platform_device_register(&mxc_scc_device);
}
#else
static inline void mxc_init_scc(void)
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
	.spi_version = 4,
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
	.spi_version = 4,
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

#ifdef CONFIG_SPI_MXC_SELECT3
/*!
 * Resource definition for the CSPI3
 */
static struct resource mxcspi3_resources[] = {
	[0] = {
	       .start = CSPI3_BASE_ADDR,
	       .end = CSPI3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CSPI3,
	       .end = MXC_INT_CSPI3,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI3 */
static struct mxc_spi_master mxcspi3_data = {
	.maxchipselect = 4,
	.spi_version = 4,
};

/*! Device Definition for MXC CSPI3 */
static struct platform_device mxcspi3_device = {
	.name = "mxc_spi",
	.id = 2,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi3_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi3_resources),
	.resource = mxcspi3_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT3 */

static inline void mxc_init_spi(void)
{
	/* SPBA configuration for CSPI2 - MCU is set */
	spba_take_ownership(SPBA_CSPI2, SPBA_MASTER_A);
#ifdef CONFIG_SPI_MXC_SELECT1
	if (platform_device_register(&mxcspi1_device) < 0)
		printk("Error: Registering the SPI Controller_1\n");
#endif				/* CONFIG_SPI_MXC_SELECT1 */
#ifdef CONFIG_SPI_MXC_SELECT2
	if (platform_device_register(&mxcspi2_device) < 0)
		printk("Error: Registering the SPI Controller_2\n");
#endif				/* CONFIG_SPI_MXC_SELECT2 */
#ifdef CONFIG_SPI_MXC_SELECT3
	if (platform_device_register(&mxcspi3_device) < 0)
		printk("Error: Registering the SPI Controller_3\n");
#endif				/* CONFIG_SPI_MXC_SELECT3 */
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

#ifdef CONFIG_I2C_MXC_SELECT2
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
#endif

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
#ifdef CONFIG_I2C_MXC_SELECT2
	{
	 .name = "mxc_i2c",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .platform_data = &mxci2c2_data,
		 },
	 .num_resources = ARRAY_SIZE(mxci2c2_resources),
	 .resource = mxci2c2_resources,},
#endif
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
	 .virtual_irq_start = MXC_GPIO_IRQ_START,
	 },
	[1] = {
	 .chip.label = "gpio-1",
	 .base = IO_ADDRESS(GPIO2_BASE_ADDR),
	 .irq = MXC_INT_GPIO2,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 32,
	 },
	[2] = {
	 .chip.label = "gpio-2",
	 .base = IO_ADDRESS(GPIO3_BASE_ADDR),
	 .irq = MXC_INT_GPIO3,
	 .irq_high = 0,
	 .virtual_irq_start = MXC_GPIO_IRQ_START + 64,
	 }
};

int __init mxc_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

#if defined(CONFIG_PCMCIA_MX31ADS) || defined(CONFIG_PCMCIA_MX31ADS_MODULE)

static struct platform_device mx31ads_device = {
	.name = "Mx31ads_pcmcia_socket",
	.id = 0,
	.dev.release = mxc_nop_release,
};
static inline void mxc_init_pcmcia(void)
{
	platform_device_register(&mx31ads_device);
}
#else
static inline void mxc_init_pcmcia(void)
{
}
#endif

#if defined(CONFIG_MXC_HMP4E) || defined(CONFIG_MXC_HMP4E_MODULE)
static struct platform_device hmp4e_device = {
	.name = "mxc_hmp4e",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		}
};

static inline void mxc_init_hmp4e(void)
{
	void __iomem *iim_reg = IO_ADDRESS(IIM_BASE_ADDR);
	if (cpu_is_mx32())
		return;

	/* override fuse for Hantro HW clock */
	if (__raw_readl(iim_reg + 0x808) == 0x4) {
		if (!(__raw_readl(iim_reg + 0x800) & (1 << 5))) {
			writel(__raw_readl(iim_reg + 0x808) & 0xfffffffb,
					   iim_reg + 0x808);
		}
	}

	platform_device_register(&hmp4e_device);
}
#else
static inline void mxc_init_hmp4e(void)
{
}
#endif

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

/*!
 * Resource definition for the DPTC LP
 */
static struct resource dptc_resources[] = {
	[0] = {
	       .start = CCM_BASE_ADDR,
	       .end = CCM_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_CCM,
	       .end = MXC_INT_CCM,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for DPTC */
static struct mxc_dptc_data dptc_data = {
	.reg_id = "SW1A",
	.clk_id = "cpu_clk",
	.dptccr_reg_addr = (unsigned int)MXC_CCM_PMCR0,
	.dcvr0_reg_addr = (int)MXC_CCM_DCVR0,
	.gpc_cntr_reg_addr = (int)MXC_CCM_PMCR0,
	.dptccr = 0xFFFFFFFF,
	.dptc_wp_supported = DPTC_WP_SUPPORTED,
	.dptc_wp_allfreq = dptc_wp_allfreq_26ckih,
	.clk_max_val = 532000000,
	.gpc_adu = 0x0,
	.vai_mask = MXC_CCM_PMCR0_PTVAI_MASK,
	.vai_offset = MXC_CCM_PMCR0_PTVAI_OFFSET,
	.dptc_enable_bit = MXC_CCM_PMCR0_DPTEN,
	.irq_mask = MXC_CCM_PMCR0_PTVAIM,
	.dptc_nvcr_bit = 0x0,
	.gpc_irq_bit = 0x00000000,
	.init_config =
	    MXC_CCM_PMCR0_PTVIS | MXC_CCM_PMCR0_DRCE3 | MXC_CCM_PMCR0_DRCE1,
	.enable_config =
	    MXC_CCM_PMCR0_DPTEN | MXC_CCM_PMCR0_DPVCR | MXC_CCM_PMCR0_DPVV,
	.dcr_mask = MXC_CCM_PMCR0_DCR,
};

/*! Device Definition for MXC DPTC */
static struct platform_device mxc_dptc_device = {
	.name = "mxc_dptc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &dptc_data,
		},
	.num_resources = ARRAY_SIZE(dptc_resources),
	.resource = dptc_resources,
};

static inline void mxc_init_dptc(void)
{
	if (clk_get_rate(ckih_clk) == 27000000) {

		if (mxc_cpu_is_rev(CHIP_REV_2_0) < 0)
			dptc_data.dptc_wp_allfreq = NULL;
		else
			dptc_data.dptc_wp_allfreq =
			    dptc_wp_allfreq_27ckih_TO_2_0;

	} else if (clk_get_rate(ckih_clk) == 26000000
		   && mxc_cpu_is_rev(CHIP_REV_2_0) == 1) {
		dptc_data.dptc_wp_allfreq = dptc_wp_allfreq_26ckih_TO_2_0;
	}

	(void)platform_device_register(&mxc_dptc_device);
}

#ifdef	CONFIG_MXC_VPU
static struct resource vpu_resources[] = {
	{
	 .start = VL2CC_BASE_ADDR,
	 .end = VL2CC_BASE_ADDR + SZ_8K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

/*! Platform Data for MXC VPU */
static struct platform_device mxcvpu_device = {
	.name = "mxc_vpu",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(vpu_resources),
	.resource = vpu_resources,
};

static inline void mxc_init_vpu(void)
{
	if (cpu_is_mx32()) {
		if (platform_device_register(&mxcvpu_device) < 0)
			printk(KERN_ERR "Error: Registering the VPU.\n");
	}
}
#else
static inline void mxc_init_vpu(void)
{
}
#endif

#if defined(CONFIG_HW_RANDOM_FSL_RNGA) || \
defined(CONFIG_HW_RANDOM_FSL_RNGA_MODULE)
static struct resource rnga_resources[] = {
	{
	 .start = RNGA_BASE_ADDR,
	 .end = RNGA_BASE_ADDR + 0x28,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device fsl_rnga_device = {
	.name = "fsl_rnga",
	.id = -1,
	.num_resources = 1,
	.resource = rnga_resources,
};

static inline void mxc_init_rnga(void)
{
	platform_device_register(&fsl_rnga_device);
}
#else
static inline void mxc_init_rnga(void)
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

int __init mxc_init_devices(void)
{
	mxc_init_wdt();
	mxc_init_ipu();
	mxc_init_spi();
	mxc_init_i2c();
	mxc_init_rtc();
	mxc_init_owire();
	mxc_init_pcmcia();
	mxc_init_scc();
	mxc_init_ssi();
	mxc_init_hmp4e();
	mxc_init_dma();
	mxc_init_audio();
	ckih_clk = clk_get(NULL, "ckih");
	mxc_init_dptc();
	mxc_init_vpu();
	mxc_init_rnga();
	mxc_init_iim();

	/* SPBA configuration for SSI2 - SDMA and MCU are set */
	spba_take_ownership(SPBA_SSI2, SPBA_MASTER_C | SPBA_MASTER_A);
	return 0;
}
