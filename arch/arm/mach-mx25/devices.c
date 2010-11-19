/*
 * Copyright 2009 Sascha Hauer, <kernel@pengutronix.de>
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/fsl_devices.h>
#include <mach/mx25.h>
#include <mach/irqs.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/sdma.h>
#include <mach/mxc_iim.h>

#include "iomux.h"
#include "sdma_script_code.h"

void mxc_sdma_get_script_info(sdma_script_start_addrs *sdma_script_addr)
{
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

	sdma_script_addr->mxc_sdma_per_2_per_addr = -1;

	sdma_script_addr->mxc_sdma_uartsh_2_per_addr = uartsh_2_per_ADDR;
	sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr = uartsh_2_mcu_ADDR;
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

	sdma_script_addr->mxc_sdma_spdif_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_spdif_addr = -1;

	sdma_script_addr->mxc_sdma_asrc_2_mcu_addr = -1;

	sdma_script_addr->mxc_sdma_dptc_dvfs_addr = -1;
	sdma_script_addr->mxc_sdma_ext_mem_2_ipu_addr = ext_mem__ipu_ram_ADDR;
	sdma_script_addr->mxc_sdma_descrambler_addr = -1;

	sdma_script_addr->mxc_sdma_start_addr = (unsigned short *)sdma_code;
	sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
	sdma_script_addr->mxc_sdma_ram_code_start_addr = RAM_CODE_START_ADDR;
}

static struct resource sdma_resources[] = {
	{
		.start = SDMA_BASE_ADDR,
		.end = SDMA_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX25_INT_SDMA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_dma_device = {
	.name = "mxc_sdma",
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	.num_resources = ARRAY_SIZE(sdma_resources),
	.resource = sdma_resources,
};

static inline void mxc_init_dma(void)
{
	(void)platform_device_register(&mxc_dma_device);
}

static struct resource uart0[] = {
	{
		.start = 0x43f90000,
		.end = 0x43f93fff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 45,
		.end = 45,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device0 = {
	.name = "imx-uart",
	.id = 0,
	.resource = uart0,
	.num_resources = ARRAY_SIZE(uart0),
};

static struct resource uart1[] = {
	{
		.start = 0x43f94000,
		.end = 0x43f97fff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 32,
		.end = 32,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device1 = {
	.name = "imx-uart",
	.id = 1,
	.resource = uart1,
	.num_resources = ARRAY_SIZE(uart1),
};

static struct resource uart2[] = {
	{
		.start = 0x5000c000,
		.end = 0x5000ffff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 18,
		.end = 18,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device2 = {
	.name = "imx-uart",
	.id = 2,
	.resource = uart2,
	.num_resources = ARRAY_SIZE(uart2),
};

static struct resource uart3[] = {
	{
		.start = 0x50008000,
		.end = 0x5000bfff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 5,
		.end = 5,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device3 = {
	.name = "imx-uart",
	.id = 3,
	.resource = uart3,
	.num_resources = ARRAY_SIZE(uart3),
};

static struct resource uart4[] = {
	{
		.start = 0x5002c000,
		.end = 0x5002ffff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 40,
		.end = 40,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_uart_device4 = {
	.name = "imx-uart",
	.id = 4,
	.resource = uart4,
	.num_resources = ARRAY_SIZE(uart4),
};

#define MX25_OTG_BASE_ADDR 0x53FF4000

static u64 otg_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_otg_resources[] = {
	{
		.start = MX25_OTG_BASE_ADDR,
		.end = MX25_OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 37,
		.end = 37,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_otg = {
	.name = "mxc-ehci",
	.id = 0,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
		.dma_mask = &otg_dmamask,
	},
	.resource = mxc_otg_resources,
	.num_resources = ARRAY_SIZE(mxc_otg_resources),
};

/* OTG gadget device */
struct platform_device otg_udc_device = {
	.name = "fsl-usb2-udc",
	.id   = -1,
	.dev  = {
		.dma_mask          = &otg_dmamask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource = mxc_otg_resources,
	.num_resources = ARRAY_SIZE(mxc_otg_resources),
};

static u64 usbh2_dmamask = DMA_BIT_MASK(32);

static struct resource mxc_usbh2_resources[] = {
	{
		.start = MX25_OTG_BASE_ADDR + 0x400,
		.end = MX25_OTG_BASE_ADDR + 0x5ff,
		.flags = IORESOURCE_MEM,
	}, {
		.start = 35,
		.end = 35,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbh2 = {
	.name = "mxc-ehci",
	.id = 1,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
		.dma_mask = &usbh2_dmamask,
	},
	.resource = mxc_usbh2_resources,
	.num_resources = ARRAY_SIZE(mxc_usbh2_resources),
};

static struct resource mxc_spi_resources0[] = {
	{
	       .start = 0x43fa4000,
	       .end = 0x43fa7fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = 14,
	       .end = 14,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device0 = {
	.name = "spi_imx",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_spi_resources0),
	.resource = mxc_spi_resources0,
};

static struct resource mxc_spi_resources1[] = {
	{
	       .start = 0x50010000,
	       .end = 0x50013fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = 13,
	       .end = 13,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device1 = {
	.name = "spi_imx",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_spi_resources1),
	.resource = mxc_spi_resources1,
};

static struct resource mxc_spi_resources2[] = {
	{
	       .start = 0x50004000,
	       .end = 0x50007fff,
	       .flags = IORESOURCE_MEM,
	}, {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_spi_device2 = {
	.name = "spi_imx",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_spi_resources2),
	.resource = mxc_spi_resources2,
};

static struct resource mxc_pwm_resources0[] = {
	{
		.start	= 0x53fe0000,
		.end	= 0x53fe3fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 26,
		.end     = 26,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device0 = {
	.name = "mxc_pwm",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources0),
	.resource = mxc_pwm_resources0,
};

static struct resource mxc_pwm_resources1[] = {
	{
		.start	= 0x53fa0000,
		.end	= 0x53fa3fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 36,
		.end     = 36,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device1 = {
	.name = "mxc_pwm",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources1),
	.resource = mxc_pwm_resources1,
};

static struct resource mxc_pwm_resources2[] = {
	{
		.start	= 0x53fa8000,
		.end	= 0x53fabfff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 41,
		.end     = 41,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device2 = {
	.name = "mxc_pwm",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources2),
	.resource = mxc_pwm_resources2,
};

static struct resource mxc_pwm_resources3[] = {
	{
		.start	= 0x53fc8000,
		.end	= 0x53fcbfff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start   = 42,
		.end     = 42,
		.flags   = IORESOURCE_IRQ,
	}
};

struct platform_device mxc_pwm_device3 = {
	.name = "mxc_pwm",
	.id = 3,
	.num_resources = ARRAY_SIZE(mxc_pwm_resources3),
	.resource = mxc_pwm_resources3,
};

static struct resource mxc_i2c_1_resources[] = {
	{
		.start	= 0x43f80000,
		.end	= 0x43f83fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 3,
		.end	= 3,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mxc_i2c_device0 = {
	.name = "imx-i2c",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_i2c_1_resources),
	.resource = mxc_i2c_1_resources,
};

static struct resource mxc_i2c_2_resources[] = {
	{
		.start	= 0x43f98000,
		.end	= 0x43f9bfff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 4,
		.end	= 4,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mxc_i2c_device1 = {
	.name = "imx-i2c",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxc_i2c_2_resources),
	.resource = mxc_i2c_2_resources,
};

static struct resource mxc_i2c_3_resources[] = {
	{
		.start	= 0x43f84000,
		.end	= 0x43f87fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 10,
		.end	= 10,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device mxc_i2c_device2 = {
	.name = "imx-i2c",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxc_i2c_3_resources),
	.resource = mxc_i2c_3_resources,
};

static struct mxc_gpio_port imx_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.base = (void __iomem *)MX25_GPIO1_BASE_ADDR_VIRT,
		.irq = 52,
		.virtual_irq_start = MXC_GPIO_IRQ_START,
	}, {
		.chip.label = "gpio-1",
		.base = (void __iomem *)MX25_GPIO2_BASE_ADDR_VIRT,
		.irq = 51,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32,
	}, {
		.chip.label = "gpio-2",
		.base = (void __iomem *)MX25_GPIO3_BASE_ADDR_VIRT,
		.irq = 16,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 64,
	}, {
		.chip.label = "gpio-3",
		.base = (void __iomem *)MX25_GPIO4_BASE_ADDR_VIRT,
		.irq = 23,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 96,
	}
};

int __init mxc_register_gpios(void)
{
	return mxc_gpio_init(imx_gpio_ports, ARRAY_SIZE(imx_gpio_ports));
}

static struct resource mx25_fec_resources[] = {
	{
		.start	= MX25_FEC_BASE_ADDR,
		.end	= MX25_FEC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MX25_INT_FEC,
		.end	= MX25_INT_FEC,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mx25_fec_device = {
	.name	= "fec",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(mx25_fec_resources),
	.resource	= mx25_fec_resources,
};

static struct resource mxc_nand_resources[] = {
	{
		.start	= MX25_NFC_BASE_ADDR,
		.end	= MX25_NFC_BASE_ADDR + 0x1fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MX25_INT_NANDFC,
		.end	= MX25_INT_NANDFC,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mxc_nand_device = {
	.name		= "mxc_nand",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(mxc_nand_resources),
	.resource	= mxc_nand_resources,
};

static struct resource mx25_rtc_resources[] = {
	{
		.start	= MX25_DRYICE_BASE_ADDR,
		.end	= MX25_DRYICE_BASE_ADDR + 0x40,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MX25_INT_DRYICE,
		.flags	= IORESOURCE_IRQ
	},
};

struct platform_device mx25_rtc_device = {
	.name	= "imxdi_rtc",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(mx25_rtc_resources),
	.resource	= mx25_rtc_resources,
};

static struct resource mx25_fb_resources[] = {
	{
		.start	= MX25_LCDC_BASE_ADDR,
		.end	= MX25_LCDC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MX25_INT_LCDC,
		.end	= MX25_INT_LCDC,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mx25_fb_device = {
	.name		= "imx-fb",
	.id		= 0,
	.resource	= mx25_fb_resources,
	.num_resources	= ARRAY_SIZE(mx25_fb_resources),
	.dev		= {
		.coherent_dma_mask = 0xFFFFFFFF,
	},
};

static struct resource mxc_wdt_resources[] = {
	{
		.start = MX25_WDOG_BASE_ADDR,
		.end = MX25_WDOG_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mxc_wdt = {
	.name = "imx2-wdt",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_wdt_resources),
	.resource = mxc_wdt_resources,
};

/* imx adc driver */
#if defined(CONFIG_IMX_ADC) || defined(CONFIG_IMX_ADC_MODULE)

static struct resource imx_adc_resources[] = {
	[0] = {
	       .start = MX25_INT_TSC,
	       .end = MX25_INT_TSC,
	       .flags = IORESOURCE_IRQ,
	       },
	[1] = {
	       .start = MX25_TSC_BASE_ADDR,
	       .end = MX25_TSC_BASE_ADDR + PAGE_SIZE,
	       .flags = IORESOURCE_MEM,
	       }
};

static struct platform_device imx_adc_device = {
	.name = "imx_adc",
	.id = 0,
	.num_resources = ARRAY_SIZE(imx_adc_resources),
	.resource = imx_adc_resources,
	.dev = {
		.release = NULL,
		},
};
static void imx_init_adc(void)
{
	(void)platform_device_register(&imx_adc_device);
}
#else
static void imx_init_adc(void)
{
}
#endif

#if defined(CONFIG_SND_MXC_SOC_ESAI) || defined(CONFIG_SND_MXC_SOC_ESAI_MODULE)

static struct mxc_esai_platform_data esai_data = {
	.activate_esai_ports = gpio_activate_esai_ports,
	.deactivate_esai_ports = gpio_deactivate_esai_ports,
};

static struct resource esai_resources[] = {
	{
		.start = MX25_ESAI_BASE_ADDR,
		.end = MX25_ESAI_BASE_ADDR + 0x100,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX25_INT_ESAI,
		.end = MX25_INT_ESAI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_esai_device = {
	.name = "mxc_esai",
	.id = 0,
	.num_resources = ARRAY_SIZE(esai_resources),
	.resource = esai_resources,
	.dev = {
		.platform_data = &esai_data,
	},
};

static void mxc_init_esai(void)
{
	platform_device_register(&mxc_esai_device);
}
#else
static void mxc_init_esai(void)
{

}
#endif

static struct mxc_audio_platform_data mxc_surround_audio_data = {
	.ext_ram = 1,
};

static struct platform_device mxc_alsa_surround_device = {
	.name = "imx-3stack-wm8580",
	.id = 0,
	.dev = {
		.platform_data = &mxc_surround_audio_data,
		},
};

static void mxc_init_surround_audio(void)
{
	platform_device_register(&mxc_alsa_surround_device);
}

#if defined(CONFIG_MXC_IIM) || defined(CONFIG_MXC_IIM_MODULE)
static struct resource mxc_iim_resources[] = {
	{
	 .start = MX25_IIM_BASE_ADDR,
	 .end = MX25_IIM_BASE_ADDR + SZ_16K - 1,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_BANK_START_ADDR,
	.bank_end   = MXC_IIM_BANK_END_ADDR,
};

static struct platform_device mxc_iim_device = {
	.name = "mxc_iim",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_iim_resources),
	.resource = mxc_iim_resources,
	.dev.platform_data = &iim_data,
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

#if defined(CONFIG_SND_MXC_SOC_SSI) || defined(CONFIG_SND_MXC_SOC_SSI_MODULE)

static struct resource ssi1_resources[] = {
	{
	 .start = MX25_SSI1_BASE_ADDR,
	 .end = MX25_SSI1_BASE_ADDR + 0x5C,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MX25_INT_SSI1,
	 .end = MX25_INT_SSI1,
	 .flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_alsa_ssi1_device = {
	.name = "mxc_ssi",
	.id = 0,
	.num_resources = ARRAY_SIZE(ssi1_resources),
	.resource = ssi1_resources,
};

static struct resource ssi2_resources[] = {
	{
	 .start = MX25_SSI2_BASE_ADDR,
	 .end = MX25_SSI2_BASE_ADDR + 0x5C,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MX25_INT_SSI2,
	 .end = MX25_INT_SSI2,
	 .flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_alsa_ssi2_device = {
	.name = "mxc_ssi",
	.id = 1,
	.num_resources = ARRAY_SIZE(ssi2_resources),
	.resource = ssi2_resources,
};

static inline void mxc_init_ssi(void)
{
	platform_device_register(&mxc_alsa_ssi1_device);
	platform_device_register(&mxc_alsa_ssi2_device);
}
#else
static inline void mxc_init_ssi(void)
{
}
#endif /* CONFIG_SND_MXC_SOC_SSI */

static int __init mxc_init_devices(void)
{
	mxc_init_dma();
	mxc_init_surround_audio();
	imx_init_adc();
	mxc_init_iim();
	mxc_init_ssi();
	mxc_init_esai();
	return 0;
}

arch_initcall(mxc_init_devices);
