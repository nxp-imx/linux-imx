/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/ipu.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/uio_driver.h>
#include <linux/mxc_scc2_driver.h>
#include <linux/iram_alloc.h>
#include <linux/gpmi-nfc.h>
#include <linux/fsl_devices.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/sdma.h>
#include "dma-apbh.h"

/* Flag used to indicate when IRAM has been initialized */
int iram_ready;
/* Flag used to indicate if dvfs_core is active. */
int dvfs_core_is_active;

static struct resource sdma_resources[] = {
	{
		.start = SDMA_BASE_ADDR,
		.end = SDMA_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SDMA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_dma_device = {
	.name = "mxc_sdma",
	.id = -1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	.num_resources = ARRAY_SIZE(sdma_resources),
	.resource = sdma_resources,
};

static struct resource mxc_w1_master_resources[] = {
	{
		.start = OWIRE_BASE_ADDR,
		.end   = OWIRE_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_OWIRE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_w1_master_device = {
	.name = "mxc_w1",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_w1_master_resources),
	.resource = mxc_w1_master_resources,
};

static struct resource mxc_kpp_resources[] = {
	{
		.start = KPP_BASE_ADDR,
		.end = KPP_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_KPP,
		.end = MXC_INT_KPP,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
};

struct platform_device mxc_powerkey_device = {
	.name = "mxcpwrkey",
	.id = 0,
};
static struct resource rtc_resources[] = {
	{
		.start = SRTC_BASE_ADDR,
		.end = SRTC_BASE_ADDR + 0x40,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SRTC_NTZ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_rtc_device = {
	.name = "mxc_rtc",
	.id = 0,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

static struct resource mxc_nand_resources[] = {
	{
		.flags = IORESOURCE_MEM,
		.name  = "NFC_AXI_BASE",
		.start = MX51_NFC_BASE_ADDR_AXI,
		.end   = MX51_NFC_BASE_ADDR_AXI + SZ_8K - 1,
	},
	{
		.flags = IORESOURCE_MEM,
		.name  = "NFC_IP_BASE",
		.start = NFC_BASE_ADDR + 0x00,
		.end   = NFC_BASE_ADDR + 0x34 - 1,
	},
	{
		.flags = IORESOURCE_IRQ,
		.start = MXC_INT_NFC,
		.end   = MXC_INT_NFC,
	},
};

struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.resource = mxc_nand_resources,
	.num_resources = ARRAY_SIZE(mxc_nand_resources),
};

static struct resource gpmi_nfc_resources[] = {
	{
	 .name  = GPMI_NFC_GPMI_REGS_ADDR_RES_NAME,
	 .flags = IORESOURCE_MEM,
	 .start = GPMI_BASE_ADDR,
	 .end   = GPMI_BASE_ADDR + SZ_8K - 1,
	 },
	{
	 .name  = GPMI_NFC_GPMI_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = MXC_INT_RAWNAND_GPMI,
	 .end   = MXC_INT_RAWNAND_GPMI,
	},
	{
	 .name  = GPMI_NFC_BCH_REGS_ADDR_RES_NAME,
	 .flags = IORESOURCE_MEM,
	 .start = BCH_BASE_ADDR,
	 .end   = BCH_BASE_ADDR + SZ_8K - 1,
	 },
	{
	 .name  = GPMI_NFC_BCH_INTERRUPT_RES_NAME,
	 .flags = IORESOURCE_IRQ,
	 .start = MXC_INT_RAWNAND_BCH,
	 .end   = MXC_INT_RAWNAND_BCH,
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
	 .start = MXC_INT_APBHDMA_CHAN0,
	 .end   = MXC_INT_APBHDMA_CHAN7,
	},
};

struct platform_device gpmi_nfc_device = {
	.name = GPMI_NFC_DRIVER_NAME,
	.id = 0,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	.resource = gpmi_nfc_resources,
	.num_resources = ARRAY_SIZE(gpmi_nfc_resources),
};

static struct resource imx_nfc_resources[] = {
	{
		.flags = IORESOURCE_MEM,
		.start = MX51_NFC_BASE_ADDR_AXI,
		.end   = MX51_NFC_BASE_ADDR_AXI + 0x1200 - 1,
		.name  = IMX_NFC_BUFFERS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = MX51_NFC_BASE_ADDR_AXI + 0x1E00,
		.end   = MX51_NFC_BASE_ADDR_AXI + 0x1E44 - 1,
		.name  = IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = NFC_BASE_ADDR + 0x00,
		.end   = NFC_BASE_ADDR + 0x34 - 1,
		.name  = IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME,
	},
	{
		.flags = IORESOURCE_IRQ,
		.start = MXC_INT_NFC,
		.end   = MXC_INT_NFC,
		.name  = IMX_NFC_INTERRUPT_RES_NAME,
	},
};

struct platform_device imx_nfc_device = {
	.name = IMX_NFC_DRIVER_NAME,
	.id = 0,
	.resource      = imx_nfc_resources,
	.num_resources = ARRAY_SIZE(imx_nfc_resources),
};

static struct resource wdt_resources[] = {
	{
		.start = WDOG1_BASE_ADDR,
		.end = WDOG1_BASE_ADDR + 0x30,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mxc_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};

static struct resource pwm1_resources[] = {
	{
		.start = PWM1_BASE_ADDR,
		.end = PWM1_BASE_ADDR + 0x14,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_PWM1,
		.end = MXC_INT_PWM1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_pwm1_device = {
	.name = "mxc_pwm",
	.id = 0,
	.num_resources = ARRAY_SIZE(pwm1_resources),
	.resource = pwm1_resources,
};

static struct resource pwm2_resources[] = {
	{
		.start = PWM2_BASE_ADDR,
		.end = PWM2_BASE_ADDR + 0x14,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_PWM2,
		.end = MXC_INT_PWM2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_pwm2_device = {
	.name = "mxc_pwm",
	.id = 1,
	.num_resources = ARRAY_SIZE(pwm2_resources),
	.resource = pwm2_resources,
};

struct platform_device mxc_pwm1_backlight_device = {
	.name = "pwm-backlight",
	.id = 0,
};

struct platform_device mxc_pwm2_backlight_device = {
	.name = "pwm-backlight",
	.id = 1,
};

static struct resource flexcan0_resources[] = {
	{
	    .start = CAN1_BASE_ADDR,
	    .end = CAN1_BASE_ADDR + 0x3FFF,
	    .flags = IORESOURCE_MEM,
	},
	{
	    .start = MXC_INT_CAN1,
	    .end = MXC_INT_CAN1,
	    .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_flexcan0_device = {
	.name = "FlexCAN",
	.id = 0,
	.num_resources = ARRAY_SIZE(flexcan0_resources),
	.resource = flexcan0_resources,
};

static struct resource flexcan1_resources[] = {
	{
	    .start = CAN2_BASE_ADDR,
	    .end = CAN2_BASE_ADDR + 0x3FFF,
	    .flags = IORESOURCE_MEM,
	},
	{
	    .start = MXC_INT_CAN2,
	    .end = MXC_INT_CAN2,
	    .flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_flexcan1_device = {
	.name = "FlexCAN",
	.id = 1,
	.num_resources = ARRAY_SIZE(flexcan1_resources),
	.resource = flexcan1_resources,
};

static struct resource ipu_resources[] = {
	{
		.start = MX51_IPU_CTRL_BASE_ADDR,
		.end = MX51_IPU_CTRL_BASE_ADDR + SZ_512M,
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

struct platform_device mxc_ipu_device = {
	.name = "mxc_ipu",
	.id = -1,
	.num_resources = ARRAY_SIZE(ipu_resources),
	.resource = ipu_resources,
};

static struct resource epdc_resources[] = {
	{
	 .start = EPDC_BASE_ADDR,
	 .end = EPDC_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	.start  = MXC_INT_EPDC,
	.end = MXC_INT_EPDC,
	.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device epdc_device = {
	.name = "mxc_epdc_fb",
	.id = -1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	.num_resources = ARRAY_SIZE(epdc_resources),
	.resource = epdc_resources,
};

static struct resource elcdif_resources[] = {
	{
	 .start = ELCDIF_BASE_ADDR,
	 .end = ELCDIF_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	.start  = MXC_INT_ELCDIF,
	.end = MXC_INT_ELCDIF,
	.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device elcdif_device = {
	.name = "mxc_elcdif_fb",
	.id = -1,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	.num_resources = ARRAY_SIZE(elcdif_resources),
	.resource = elcdif_resources,
};

struct platform_device mxc_fb_devices[] = {
	{
		.name = "mxc_sdc_fb",
		.id = 0,
		.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	},
	{
		.name = "mxc_sdc_fb",
		.id = 1,
		.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	},
	{
		.name = "mxc_sdc_fb",
		.id = 2,
		.dev = {
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	},
};

static struct resource ldb_resources[] = {
	{
		.start = IOMUXC_BASE_ADDR,
		.end = IOMUXC_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mxc_ldb_device = {
	.name = "mxc_ldb",
	.id = -1,
	.num_resources = ARRAY_SIZE(ldb_resources),
	.resource = ldb_resources,
};

static struct resource vpu_resources[] = {
	{
		.start = VPU_BASE_ADDR,
		.end = VPU_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start	= MXC_INT_VPU,
		.end = MXC_INT_VPU,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mxcvpu_device = {
	.name = "mxc_vpu",
	.id = 0,
	.num_resources = ARRAY_SIZE(vpu_resources),
	.resource = vpu_resources,
};

struct platform_device fixed_volt_reg_device = {
	.name          = "reg-fixed-voltage",
	.id            = -1,
};

static struct resource scc_resources[] = {
	{
		.start = SCC_BASE_ADDR,
		.end = SCC_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start	= MX51_SCC_RAM_BASE_ADDR,
		.end = MX51_SCC_RAM_BASE_ADDR + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device mxcscc_device = {
	.name = "mxc_scc",
	.id = 0,
	.num_resources = ARRAY_SIZE(scc_resources),
	.resource = scc_resources,
};

static struct resource dcp_resources[] = {

	{
		.flags = IORESOURCE_MEM,
		.start = DCP_BASE_ADDR,
		.end   = DCP_BASE_ADDR + 0x2000 - 1,
	}, {
		.flags = IORESOURCE_IRQ,
		.start = MXC_INT_DCP_CHAN0,
		.end = MXC_INT_DCP_CHAN0,
	}, {
		.flags = IORESOURCE_IRQ,
		.start = MXC_INT_DCP_CHAN1_3,
		.end = MXC_INT_DCP_CHAN1_3,
	},
};

struct platform_device dcp_device = {
	.name = "dcp",
	.id = 0,
	.num_resources = ARRAY_SIZE(dcp_resources),
	.resource = dcp_resources,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};


static struct resource rngb_resources[] = {
	{
		.start = RNGB_BASE_ADDR,
		.end = RNGB_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_RNGB_BLOCK,
		.flags = IORESOURCE_IRQ,
	},
};

/* the RNGC driver applies for MX50's RNGB hw */
struct platform_device mxc_rngb_device = {
	.name = "fsl_rngc",
	.id = 0,
	.num_resources = ARRAY_SIZE(rngb_resources),
	.resource = rngb_resources,
};

static struct resource mxc_fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM
	},
	{
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

static struct resource mxc_ptp_resources[] = {
	{
		.start	= PTP_BASE_ADDR,
		.end	= PTP_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= RTC_BASE_ADDR,
		.end	= RTC_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= MXC_INT_PTP,
		.end	= MXC_INT_PTP,
		.flags	= IORESOURCE_IRQ
	},
	{
		.start	= MXC_INT_RTC,
		.end	= MXC_INT_RTC,
		.flags	= IORESOURCE_IRQ
	},
};

struct platform_device mxc_ptp_device = {
	.name = "ptp",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_ptp_resources),
	.resource = mxc_ptp_resources,
};

static struct resource mxcspi1_resources[] = {
	{
		.start = CSPI1_BASE_ADDR,
		.end = CSPI1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI1,
		.end = MXC_INT_CSPI1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxcspi1_device = {
	.name = "mxc_spi",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxcspi1_resources),
	.resource = mxcspi1_resources,
};

static struct resource mxcspi2_resources[] = {
	{
		.start = CSPI2_BASE_ADDR,
		.end = CSPI2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI2,
		.end = MXC_INT_CSPI2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxcspi2_device = {
	.name = "mxc_spi",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxcspi2_resources),
	.resource = mxcspi2_resources,
};

static struct resource mxcspi3_resources[] = {
	{
		.start = CSPI3_BASE_ADDR,
		.end = CSPI3_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_CSPI,
		.end = MXC_INT_CSPI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxcspi3_device = {
	.name = "mxc_spi",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxcspi3_resources),
	.resource = mxcspi3_resources,
};

static struct resource mxci2c1_resources[] = {
	{
		.start = I2C1_BASE_ADDR,
		.end = I2C1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C1,
		.end = MXC_INT_I2C1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mxci2c2_resources[] = {
	{
		.start = I2C2_BASE_ADDR,
		.end = I2C2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_I2C2,
		.end = MXC_INT_I2C2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mxci2c3_resources[] = {
	{
	       .start = I2C3_BASE_ADDR,
	       .end = I2C3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	{
	       .start = MXC_INT_I2C3,
	       .end = MXC_INT_I2C3,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device mxci2c_devices[] = {
	{
		.name = "imx-i2c",
		.id = 0,
		.num_resources = ARRAY_SIZE(mxci2c1_resources),
		.resource = mxci2c1_resources,
	},
	{
		.name = "imx-i2c",
		.id = 1,
		.num_resources = ARRAY_SIZE(mxci2c2_resources),
		.resource = mxci2c2_resources,
	},
	{
		.name = "imx-i2c",
		.id = 2,
		.num_resources = ARRAY_SIZE(mxci2c3_resources),
		.resource = mxci2c3_resources,
	},
};

static struct resource mxci2c_hs_resources[] = {
	{
		.start = HSI2C_DMA_BASE_ADDR,
		.end = HSI2C_DMA_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_HS_I2C,
		.end = MXC_INT_HS_I2C,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxci2c_hs_device = {
	.name = "mxc_i2c_hs",
	.id = 3,
	.num_resources = ARRAY_SIZE(mxci2c_hs_resources),
	.resource = mxci2c_hs_resources
};

static struct resource ssi1_resources[] = {
	{
		.start = SSI1_BASE_ADDR,
		.end = SSI1_BASE_ADDR + 0x5C,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SSI1,
		.end = MXC_INT_SSI1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_ssi1_device = {
	.name = "mxc_ssi",
	.id = 0,
	.num_resources = ARRAY_SIZE(ssi1_resources),
	.resource = ssi1_resources,
};

static struct resource ssi2_resources[] = {
	{
		.start = SSI2_BASE_ADDR,
		.end = SSI2_BASE_ADDR + 0x5C,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SSI2,
		.end = MXC_INT_SSI2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_ssi2_device = {
	.name = "mxc_ssi",
	.id = 1,
	.num_resources = ARRAY_SIZE(ssi2_resources),
	.resource = ssi2_resources,
};

static struct resource ssi3_resources[] = {
	{
		.start = SSI3_BASE_ADDR,
		.end = SSI3_BASE_ADDR + 0x5C,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SSI3,
		.end = MXC_INT_SSI3,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_ssi3_device = {
	.name = "mxc_ssi",
	.id = 2,
	.num_resources = ARRAY_SIZE(ssi3_resources),
	.resource = ssi3_resources,
};

static struct resource esai_resources[] = {
	{
		.start = ESAI_BASE_ADDR,
		.end = ESAI_BASE_ADDR + 0x100,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_ESAI,
		.end = MXC_INT_ESAI,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_esai_device = {
	.name = "mxc_esai",
	.id = 0,
	.num_resources = ARRAY_SIZE(esai_resources),
	.resource = esai_resources,
};

static struct resource tve_resources[] = {
	{
		.start = TVE_BASE_ADDR,
		.end = TVE_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_TVE,
		.end = MXC_INT_TVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_tve_device = {
	.name = "tve",
	.num_resources = ARRAY_SIZE(tve_resources),
	.resource = tve_resources,
};

static struct resource dvfs_core_resources[] = {
	{
		.start = DVFSCORE_BASE_ADDR,
		.end = DVFSCORE_BASE_ADDR + 4 * SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_GPC1,
		.end = MXC_INT_GPC1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_dvfs_core_device = {
	.name = "mxc_dvfs_core",
	.id = 0,
	.num_resources = ARRAY_SIZE(dvfs_core_resources),
	.resource = dvfs_core_resources,
};

static struct resource dvfs_per_resources[] = {
	{
		.start = DVFSPER_BASE_ADDR,
		.end = DVFSPER_BASE_ADDR + 2 * SZ_16 - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_GPC1,
		.end = MXC_INT_GPC1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_dvfs_per_device = {
	 .name = "mxc_dvfsper",
	 .id = 0,
	 .num_resources = ARRAY_SIZE(dvfs_per_resources),
	 .resource = dvfs_per_resources,
};

struct mxc_gpio_port mxc_gpio_ports[] = {
	{
		.chip.label = "gpio-0",
		.base = IO_ADDRESS(GPIO1_BASE_ADDR),
		.irq = MXC_INT_GPIO1_LOW,
		.irq_high = MXC_INT_GPIO1_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START
	},
	{
		.chip.label = "gpio-1",
		.base = IO_ADDRESS(GPIO2_BASE_ADDR),
		.irq = MXC_INT_GPIO2_LOW,
		.irq_high = MXC_INT_GPIO2_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 1
	},
	{
		.chip.label = "gpio-2",
		.base = IO_ADDRESS(GPIO3_BASE_ADDR),
		.irq = MXC_INT_GPIO3_LOW,
		.irq_high = MXC_INT_GPIO3_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 2
	},
	{
		.chip.label = "gpio-3",
		.base = IO_ADDRESS(GPIO4_BASE_ADDR),
		.irq = MXC_INT_GPIO4_LOW,
		.irq_high = MXC_INT_GPIO4_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 3
	},
	{
		.chip.label = "gpio-4",
		.base = IO_ADDRESS(GPIO5_BASE_ADDR),
		.irq = MXC_INT_GPIO5_LOW,
		.irq_high = MXC_INT_GPIO5_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 4
	},
	{
		.chip.label = "gpio-5",
		.base = IO_ADDRESS(GPIO6_BASE_ADDR),
		.irq = MXC_INT_GPIO6_LOW,
		.irq_high = MXC_INT_GPIO6_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 5
	},
	{
		.chip.label = "gpio-6",
		.base = IO_ADDRESS(GPIO7_BASE_ADDR),
		.irq = MXC_INT_GPIO7_LOW,
		.irq_high = MXC_INT_GPIO7_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START + 32 * 6
	},
};

int __init mxc_register_gpios(void)
{
	if (cpu_is_mx51())
		return mxc_gpio_init(mxc_gpio_ports, 4);
	else if (cpu_is_mx50())
		return mxc_gpio_init(mxc_gpio_ports, 6);
	return mxc_gpio_init(mxc_gpio_ports, ARRAY_SIZE(mxc_gpio_ports));
}

static struct resource spdif_resources[] = {
	{
		.start = SPDIF_BASE_ADDR,
		.end = SPDIF_BASE_ADDR + 0x50,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SPDIF_MX51,
		.end = MXC_INT_SPDIF_MX51,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_alsa_spdif_device = {
	.name = "mxc_alsa_spdif",
	.id = 0,
	.num_resources = ARRAY_SIZE(spdif_resources),
	.resource = spdif_resources,
};

struct platform_device mx51_lpmode_device = {
	.name = "mx51_lpmode",
	.id = 0,
};

struct platform_device busfreq_device = {
	.name = "busfreq",
	.id = 0,
};

struct platform_device pm_device = {
	.name = "mx5_pm",
	.id = 0,
};

static struct resource mxc_m4if_resources[] = {
	{
		.start = M4IF_BASE_ADDR,
		.end = M4IF_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device sdram_autogating_device = {
	.name = "sdram_autogating",
	.id = 0,
	.resource = mxc_m4if_resources,
	.num_resources = ARRAY_SIZE(mxc_m4if_resources),
};

static struct resource mxc_iim_resources[] = {
	{
		.start = IIM_BASE_ADDR,
		.end = IIM_BASE_ADDR + SZ_16K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_IIM,
		.end = MXC_INT_IIM,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_iim_device = {
	.name = "mxc_iim",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_iim_resources),
	.resource = mxc_iim_resources
};

static struct resource mxc_sim_resources[] = {
	{
		.start = SIM_BASE_ADDR,
		.end = SIM_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SIM_IPB,
		.end = MXC_INT_SIM_IPB,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MXC_INT_SIM_DAT,
		.end = MXC_INT_SIM_DAT,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_sim_device = {
	.name = "mxc_sim",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_sim_resources),
	.resource = mxc_sim_resources,
};

static struct resource mxcsdhc1_resources[] = {
	{
		.start = MMC_SDHC1_BASE_ADDR,
		.end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_MMC_SDHC1,
		.end = MXC_INT_MMC_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mxcsdhc2_resources[] = {
	{
		.start = MMC_SDHC2_BASE_ADDR,
		.end = MMC_SDHC2_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_MMC_SDHC2,
		.end = MXC_INT_MMC_SDHC2,
		.flags = IORESOURCE_IRQ,
	},
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource mxcsdhc3_resources[] = {
	{
		.start = MMC_SDHC3_BASE_ADDR,
		.end = MMC_SDHC3_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_MMC_SDHC3,
		.end = MXC_INT_MMC_SDHC3,
		.flags = IORESOURCE_IRQ,
	},
	{
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxcsdhc1_device = {
	.name = "mxsdhci",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

struct platform_device mxcsdhc2_device = {
	.name = "mxsdhci",
	.id = 1,
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

struct platform_device mxcsdhc3_device = {
	.name = "mxsdhci",
	.id = 2,
	.num_resources = ARRAY_SIZE(mxcsdhc3_resources),
	.resource = mxcsdhc3_resources,
};

static struct resource pata_fsl_resources[] = {
	{
		.start = ATA_BASE_ADDR,
		.end = ATA_BASE_ADDR + 0x000000C8,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_ATA,
		.end = MXC_INT_ATA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device pata_fsl_device = {
	.name = "pata_fsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(pata_fsl_resources),
	.resource = pata_fsl_resources,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* On-Chip OTP device and resource */
static struct resource otp_resource = {
	.start	= OCOTP_CTRL_BASE_ADDR,
	.end 	= OCOTP_CTRL_BASE_ADDR + SZ_8K - 1,
	.flags 	= IORESOURCE_MEM,
};

struct platform_device fsl_otp_device = {
	.name	= "ocotp",
	.id	= -1,
	.resource      = &otp_resource,
	.num_resources = 1,
};

static struct resource ahci_fsl_resources[] = {
	{
		.start = MX53_SATA_BASE_ADDR,
		.end = MX53_SATA_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_SATA,
		.end = MXC_INT_SATA,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ahci_fsl_device = {
	.name = "ahci",
	.id = 0,
	.num_resources = ARRAY_SIZE(ahci_fsl_resources),
	.resource = ahci_fsl_resources,
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static u64 usb_dma_mask = DMA_BIT_MASK(32);

static struct resource usbotg_host_resources[] = {
	{
		.start = OTG_BASE_ADDR,
		.end = OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource usbotg_udc_resources[] = {
	{
		.start = OTG_BASE_ADDR,
		.end = OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource usbotg_xcvr_resources[] = {
	{
		.start = OTG_BASE_ADDR,
		.end = OTG_BASE_ADDR + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_USB_OTG,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource usbotg_wakeup_resources[] = {
	{
		.start = MXC_INT_USB_OTG,/* wakeup irq */
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MXC_INT_USB_OTG,/* usb core irq , may be equel to wakeup irq for some imx chips */
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device mxc_usbdr_udc_device = {
	.name		= "fsl-usb2-udc",
	.id		= -1,
	.dev  = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource      = usbotg_udc_resources,
	.num_resources = ARRAY_SIZE(usbotg_udc_resources),
};

struct platform_device mxc_usbdr_otg_device = {
	.name = "fsl-usb2-otg",
	.id = -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource      = usbotg_xcvr_resources,
	.num_resources = ARRAY_SIZE(usbotg_xcvr_resources),
};

struct platform_device mxc_usbdr_host_device = {
	.name = "fsl-ehci",
	.id = 0,
	.num_resources = ARRAY_SIZE(usbotg_host_resources),
	.resource = usbotg_host_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

struct platform_device mxc_usbdr_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 0,
	.num_resources = ARRAY_SIZE(usbotg_wakeup_resources),
	.resource = usbotg_wakeup_resources,
};

static struct resource usbh1_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x200,
		.end = OTG_BASE_ADDR + 0x200 + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_USB_H1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource usbh1_wakeup_resources[] = {
	{
		.start = MXC_INT_USB_H1, /*wakeup irq*/
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MXC_INT_USB_H1,
		.flags = IORESOURCE_IRQ,/* usb core irq */
	},
};

struct platform_device mxc_usbh1_device = {
	.name = "fsl-ehci",
	.id = 1,
	.num_resources = ARRAY_SIZE(usbh1_resources),
	.resource = usbh1_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

struct platform_device mxc_usbh1_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 1,
	.num_resources = ARRAY_SIZE(usbh1_wakeup_resources),
	.resource = usbh1_wakeup_resources,
};

static struct resource usbh2_resources[] = {
	{
		.start = OTG_BASE_ADDR + 0x400,
		.end = OTG_BASE_ADDR + 0x400 + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource usbh2_wakeup_resources[] = {
	{
		.start = MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,/* wakeup irq */
	},
	{
		.start = MXC_INT_USB_H2,
		.flags = IORESOURCE_IRQ,/* usb core irq */
	},
};
struct platform_device mxc_usbh2_device = {
	.name = "fsl-ehci",
	.id = 2,
	.num_resources = ARRAY_SIZE(usbh2_resources),
	.resource = usbh2_resources,
	.dev = {
		.dma_mask = &usb_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

struct platform_device mxc_usbh2_wakeup_device = {
	.name = "usb_wakeup",
	.id   = 2,
	.num_resources = ARRAY_SIZE(usbh2_wakeup_resources),
	.resource = usbh2_wakeup_resources,
};

static struct resource mxc_gpu_resources[] = {
	{
		.start = MXC_INT_GPU2_IRQ,
		.end = MXC_INT_GPU2_IRQ,
		.name = "gpu_2d_irq",
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MXC_INT_GPU,
		.end = MXC_INT_GPU,
		.name = "gpu_3d_irq",
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MX51_GPU2D_BASE_ADDR,
		.end = MX51_GPU2D_BASE_ADDR + SZ_4K - 1,
		.name = "gpu_2d_registers",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = GPU_BASE_ADDR,
		.end = GPU_BASE_ADDR + SZ_128K - 1,
		.name = "gpu_3d_registers",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX51_GPU_GMEM_BASE_ADDR,
		.end = MX51_GPU_GMEM_BASE_ADDR + SZ_128K - 1,
		.name = "gpu_graphics_mem",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0,
		.end = 0,
		.name = "gpu_reserved_mem",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device gpu_device = {
	.name = "mxc_gpu",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_gpu_resources),
	.resource = mxc_gpu_resources,
};

int z160_revision;

static struct resource mxc_gpu2d_resources[] = {
	{
		.start = MX51_GPU2D_BASE_ADDR,
		.end = MX51_GPU2D_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.flags = IORESOURCE_MEM,
	},
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct resource mlb_resources[] = {
	[0] = {
	       .start = MLB_BASE_ADDR,
	       .end = MLB_BASE_ADDR + 0x300,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MLB,
	       .end = MXC_INT_MLB,
	       .flags = IORESOURCE_IRQ,
	       },
};

struct platform_device mxc_mlb_device = {
	.name = "mxc_mlb",
	.id = 0,
	.num_resources = ARRAY_SIZE(mlb_resources),
	.resource = mlb_resources,
};

static struct resource pxp_resources[] = {
	{
	 .start = EPXP_BASE_ADDR,
	 .end = EPXP_BASE_ADDR + SZ_4K - 1,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_INT_EPXP,
	 .flags = IORESOURCE_IRQ,
	 },
};

struct platform_device mxc_pxp_device = {
	.name = "mxc-pxp",
	.id = -1,
	.num_resources = ARRAY_SIZE(pxp_resources),
	.resource = pxp_resources,
};

struct platform_device mxc_pxp_client_device = {
	.name = "pxp-device",
	.id = -1,
};

static u64 pxp_dma_mask = DMA_BIT_MASK(32);
struct platform_device mxc_pxp_v4l2 = {
	.name = "pxp-v4l2",
	.id = -1,
	.dev		= {
		.dma_mask		= &pxp_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

struct platform_device mxc_v4l2_device = {
	.name = "mxc_v4l2_capture",
	.id = 0,
};

struct platform_device mxc_v4l2out_device = {
	.name = "mxc_v4l2_output",
	.id = 0,
};

struct resource viim_resources[] = {
	[0] = {
		.start  = (GPT1_BASE_ADDR - 0x20000000),
		.end    = (GPT1_BASE_ADDR - 0x20000000) + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = OCOTP_CTRL_BASE_ADDR,
		.end    = OCOTP_CTRL_BASE_ADDR + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};
struct platform_device mxs_viim = {
	.name   = "mxs_viim",
	.id     = -1,
	.num_resources = ARRAY_SIZE(viim_resources),
	.resource = viim_resources,
};

static struct resource dma_apbh_resources[] = {
	{
		.start = APBHDMA_BASE_ADDR,
		.end = APBHDMA_BASE_ADDR + 0x2000 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device mxs_dma_apbh_device = {
	.name = "mxs-dma-apbh",
	.num_resources = ARRAY_SIZE(dma_apbh_resources),
	.resource = dma_apbh_resources,
};

struct platform_device mxc_zq_calib_device = {
	.name = "mxc_zq_calib",
	.id = -1,
};

void __init mx5_init_irq(void)
{
	unsigned long tzic_addr;

	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0)
		tzic_addr = MX51_TZIC_BASE_ADDR_T01;
	else if (cpu_is_mx51_rev(CHIP_REV_2_0) > 0)
		tzic_addr = MX51_TZIC_BASE_ADDR;
	else /* mx53 and mx50 */
		tzic_addr = MX53_TZIC_BASE_ADDR;

	mxc_tzic_init_irq(tzic_addr);
}

#define SCM_RD_DELAY	1000000 /* in nanoseconds */
#define SEC_TO_NANOSEC  1000000000 /*Second to nanoseconds */
static __init void mxc_init_scc_iram(void)
{
	uint32_t reg_value;
	uint32_t reg_mask = 0;
	uint8_t *UMID_base;
	uint32_t *MAP_base;
	uint8_t i;
	uint32_t partition_no;
	uint32_t scc_partno;
	void *scm_ram_base;
	void *scc_base;
	uint32_t ram_partitions, ram_partition_size, ram_size;
	uint32_t scm_version_register;
	struct timespec stime;
	struct timespec curtime;
	long scm_rd_timeout = 0;
	long cur_ns = 0;
	long start_ns = 0;

	scc_base = ioremap((uint32_t) scc_resources[0].start, 0x140);
	if (scc_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP SCC REGS\n");
		return;
	}

	scm_version_register = __raw_readl(scc_base + SCM_VERSION_REG);
	ram_partitions = 1 + ((scm_version_register & SCM_VER_NP_MASK)
		>> SCM_VER_NP_SHIFT);
	ram_partition_size = (uint32_t) (1 <<
		((scm_version_register & SCM_VER_BPP_MASK)
		>> SCM_VER_BPP_SHIFT));

	ram_size = (uint32_t)(ram_partitions * ram_partition_size);

	scm_ram_base = ioremap((uint32_t) scc_resources[1].start, ram_size);

	if (scm_ram_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP SCC RAM\n");
		return;
	}

	/* Wait for any running SCC operations to finish or fail */
	getnstimeofday(&stime);
	do {
		reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
		getnstimeofday(&curtime);
		if (curtime.tv_nsec > stime.tv_nsec)
			scm_rd_timeout = curtime.tv_nsec - stime.tv_nsec;
		else{
			/*Converted second to nanosecond and add to
			nsec when current nanosec is less than
			start time nanosec.*/
			cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
			curtime.tv_nsec;
			start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
				stime.tv_nsec;
			scm_rd_timeout = cur_ns - start_ns;
		}
	} while (((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
	&& ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_FAIL));

	/* Check for failures */
	if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY) {
		/* Special message for bad secret key fuses */
		if (reg_value & SCM_STATUS_KST_BAD_KEY)
			printk(KERN_ERR "INVALID SCC KEY FUSE PATTERN\n");
		else
		    printk(KERN_ERR "SECURE RAM FAILURE\n");

		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

	scm_rd_timeout = 0;

	/* Release all partitions for SCC2 driver on MX53*/
	if (cpu_is_mx53())
		scc_partno = 0;
	/* Release final two partitions for SCC2 driver on MX51 */
	else
		scc_partno = ram_partitions -
			(SCC_RAM_SIZE / ram_partition_size);

	for (partition_no = scc_partno; partition_no < ram_partitions;
	     partition_no++) {
		reg_value = (((partition_no << SCM_ZCMD_PART_SHIFT) &
			SCM_ZCMD_PART_MASK) | ((0x03 << SCM_ZCMD_CCMD_SHIFT) &
			SCM_ZCMD_CCMD_MASK));
		__raw_writel(reg_value, scc_base + SCM_ZCMD_REG);
		udelay(1);
		/* Wait for zeroization to complete */
		getnstimeofday(&stime);
		do {
			reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
			getnstimeofday(&curtime);
			if (curtime.tv_nsec > stime.tv_nsec)
				scm_rd_timeout = curtime.tv_nsec -
				stime.tv_nsec;
			else {
				/*Converted second to nanosecond and add to
				nsec when current nanosec is less than
				start time nanosec.*/
				cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
				curtime.tv_nsec;
				start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
					stime.tv_nsec;
				scm_rd_timeout = cur_ns - start_ns;
			}
		} while (((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_READY) && ((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_FAIL) && (scm_rd_timeout <= SCM_RD_DELAY));

		if (scm_rd_timeout > SCM_RD_DELAY)
			printk(KERN_ERR "SCM Status Register Read timeout"
			"for Partition No:%d", partition_no);

		if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
			break;
	}

	/* 4 partitions on MX53 */
	if (cpu_is_mx53())
		reg_mask = 0xFF;

	/*Check all expected partitions released */
	reg_value = __raw_readl(scc_base + SCM_PART_OWNERS_REG);
	if ((reg_value & reg_mask) != 0) {
		printk(KERN_ERR "FAILED TO RELEASE IRAM PARTITION\n");
		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

	/* we are done if this is MX53, since no sharing of IRAM and SCC_RAM */
	if (cpu_is_mx53())
		goto exit;

	reg_mask = 0;
	scm_rd_timeout = 0;
	/* Allocate remaining partitions for general use */
	for (partition_no = 0; partition_no < scc_partno; partition_no++) {
		/* Supervisor mode claims a partition for it's own use
		by writing zero to SMID register.*/
		__raw_writel(0, scc_base + (SCM_SMID0_REG + 8 * partition_no));

		/* Wait for any zeroization to complete */
		getnstimeofday(&stime);
		do {
			reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
			getnstimeofday(&curtime);
			if (curtime.tv_nsec > stime.tv_nsec)
				scm_rd_timeout = curtime.tv_nsec -
				stime.tv_nsec;
			else{
				/*Converted second to nanosecond and add to
				nsec when current nanosec is less than
				start time nanosec.*/
				cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
				curtime.tv_nsec;
				start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
					stime.tv_nsec;
				scm_rd_timeout = cur_ns - start_ns;
			}
		} while (((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_READY) && ((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_FAIL) && (scm_rd_timeout <= SCM_RD_DELAY));

		if (scm_rd_timeout > SCM_RD_DELAY)
			printk(KERN_ERR "SCM Status Register Read timeout"
			"for Partition No:%d", partition_no);

		if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
			break;
		/* Set UMID=0 and permissions for universal data
		read/write access */
		MAP_base = scm_ram_base +
			(uint32_t) (partition_no * ram_partition_size);
		UMID_base = (uint8_t *) MAP_base + 0x10;
		for (i = 0; i < 16; i++)
			UMID_base[i] = 0;

		MAP_base[0] = (SCM_PERM_NO_ZEROIZE | SCM_PERM_HD_SUP_DISABLE |
			SCM_PERM_HD_READ | SCM_PERM_HD_WRITE |
			SCM_PERM_HD_EXECUTE | SCM_PERM_TH_READ |
			SCM_PERM_TH_WRITE);
		reg_mask |= (3 << (2 * (partition_no)));
	}

	/* Check all expected partitions allocated */
	reg_value = __raw_readl(scc_base + SCM_PART_OWNERS_REG);
	if ((reg_value & reg_mask) != reg_mask) {
		printk(KERN_ERR "FAILED TO ACQUIRE IRAM PARTITION\n");
		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

exit:
	iounmap(scm_ram_base);
	iounmap(scc_base);
	printk(KERN_INFO "IRAM READY\n");
	iram_ready = 1;
}

#define MX53_OFFSET 0x20000000

int __init mxc_init_devices(void)
{
	if (cpu_is_mx53() || cpu_is_mx50()) {
		sdma_resources[0].start -= MX53_OFFSET;
		sdma_resources[0].end -= MX53_OFFSET;
		mxc_w1_master_resources[0].start -= MX53_OFFSET;
		mxc_w1_master_resources[0].end -= MX53_OFFSET;
		mxc_kpp_resources[0].start -= MX53_OFFSET;
		mxc_kpp_resources[0].end -= MX53_OFFSET;
		rtc_resources[0].start -= MX53_OFFSET;
		rtc_resources[0].end -= MX53_OFFSET;

		wdt_resources[0].start -= MX53_OFFSET;
		wdt_resources[0].end -= MX53_OFFSET;
		pwm1_resources[0].start -= MX53_OFFSET;
		pwm1_resources[0].end -= MX53_OFFSET;
		pwm2_resources[0].start -= MX53_OFFSET;
		pwm2_resources[0].end -= MX53_OFFSET;
		flexcan0_resources[0].start -= MX53_OFFSET;
		flexcan0_resources[0].end -= MX53_OFFSET;
		flexcan1_resources[0].start -= MX53_OFFSET;
		flexcan1_resources[0].end -= MX53_OFFSET;
		mxc_fec_resources[0].start -= MX53_OFFSET;
		mxc_fec_resources[0].end -= MX53_OFFSET;
		mxc_ptp_resources[0].start -= MX53_OFFSET;
		mxc_ptp_resources[0].end -= MX53_OFFSET;
		mxc_ptp_resources[1].start -= MX53_OFFSET;
		mxc_ptp_resources[1].end -= MX53_OFFSET;
		vpu_resources[0].start -= MX53_OFFSET;
		vpu_resources[0].end -= MX53_OFFSET;
		scc_resources[0].start -= MX53_OFFSET;
		scc_resources[0].end -= MX53_OFFSET;
		scc_resources[1].start = MX53_SCC_RAM_BASE_ADDR;
		scc_resources[1].end = MX53_SCC_RAM_BASE_ADDR + SZ_16K - 1;
		rngb_resources[0].start -= MX53_OFFSET;
		rngb_resources[0].end -= MX53_OFFSET;
		mxcspi1_resources[0].start -= MX53_OFFSET;
		mxcspi1_resources[0].end -= MX53_OFFSET;
		mxcspi2_resources[0].start -= MX53_OFFSET;
		mxcspi2_resources[0].end -= MX53_OFFSET;
		mxcspi3_resources[0].start -= MX53_OFFSET;
		mxcspi3_resources[0].end -= MX53_OFFSET;
		mxci2c1_resources[0].start -= MX53_OFFSET;
		mxci2c1_resources[0].end -= MX53_OFFSET;
		mxci2c2_resources[0].start -= MX53_OFFSET;
		mxci2c2_resources[0].end -= MX53_OFFSET;
		mxci2c3_resources[0].start -= MX53_OFFSET;
		mxci2c3_resources[0].end -= MX53_OFFSET;
		ssi1_resources[0].start -= MX53_OFFSET;
		ssi1_resources[0].end -= MX53_OFFSET;
		ssi2_resources[0].start -= MX53_OFFSET;
		ssi2_resources[0].end -= MX53_OFFSET;
		esai_resources[0].start -= MX53_OFFSET;
		esai_resources[0].end -= MX53_OFFSET;
		tve_resources[0].start -= MX53_OFFSET;
		tve_resources[0].end -= MX53_OFFSET;
		dvfs_core_resources[0].start -= MX53_OFFSET;
		dvfs_core_resources[0].end -= MX53_OFFSET;
		dvfs_per_resources[0].start -= MX53_OFFSET;
		dvfs_per_resources[0].end -= MX53_OFFSET;
		spdif_resources[0].start -= MX53_OFFSET;
		spdif_resources[0].end -= MX53_OFFSET;
		spdif_resources[1].start = MXC_INT_SPDIF_MX53;
		spdif_resources[1].end = MXC_INT_SPDIF_MX53;
		mxc_m4if_resources[0].start -= MX53_OFFSET;
		mxc_m4if_resources[0].end -= MX53_OFFSET;
		mxc_iim_resources[0].start -= MX53_OFFSET;
		mxc_iim_resources[0].end -= MX53_OFFSET;
		mxc_sim_resources[0].start -= MX53_OFFSET;
		mxc_sim_resources[0].end -= MX53_OFFSET;
		mxcsdhc1_resources[0].start -= MX53_OFFSET;
		mxcsdhc1_resources[0].end -= MX53_OFFSET;
		mxcsdhc2_resources[0].start -= MX53_OFFSET;
		mxcsdhc2_resources[0].end -= MX53_OFFSET;
		mxcsdhc3_resources[0].start -= MX53_OFFSET;
		mxcsdhc3_resources[0].end -= MX53_OFFSET;
		usbotg_host_resources[0].start -= MX53_OFFSET;
		usbotg_host_resources[0].end -= MX53_OFFSET;
		usbotg_udc_resources[0].start -= MX53_OFFSET;
		usbotg_udc_resources[0].end -= MX53_OFFSET;
		usbotg_xcvr_resources[0].start -= MX53_OFFSET;
		usbotg_xcvr_resources[0].end -= MX53_OFFSET;
		usbh1_resources[0].start -= MX53_OFFSET;
		usbh1_resources[0].end -= MX53_OFFSET;
		usbh2_resources[0].start -= MX53_OFFSET;
		usbh2_resources[0].end -= MX53_OFFSET;
		mxc_gpu_resources[2].start = MX53_GPU2D_BASE_ADDR;
		mxc_gpu_resources[2].end = MX53_GPU2D_BASE_ADDR + SZ_4K - 1;
		mxc_gpu2d_resources[0].start = MX53_GPU2D_BASE_ADDR;
		mxc_gpu2d_resources[0].end = MX53_GPU2D_BASE_ADDR + SZ_4K - 1;
		if (cpu_is_mx53()) {
			mxc_gpu_resources[4].start = MX53_GPU_GMEM_BASE_ADDR;
			mxc_gpu_resources[4].end = MX53_GPU_GMEM_BASE_ADDR
						+ SZ_256K - 1;
			if (cpu_is_mx53_rev(CHIP_REV_2_0) >= 1) {
				z160_revision = 1;
			} else {
				z160_revision = 0;
			}
		} else {
			mxc_gpu_resources[1].start = 0;
			mxc_gpu_resources[1].end = 0;
			mxc_gpu_resources[3].start = 0;
			mxc_gpu_resources[3].end = 0;
			mxc_gpu_resources[4].start = 0;
			mxc_gpu_resources[4].end = 0;
			z160_revision = 1;
		}
		ipu_resources[0].start = MX53_IPU_CTRL_BASE_ADDR;
		ipu_resources[0].end = MX53_IPU_CTRL_BASE_ADDR + SZ_128M - 1;
		mlb_resources[0].start -= MX53_OFFSET;
		mlb_resources[0].end -= MX53_OFFSET;
		mxc_nandv2_mtd_device.resource[0].start =
					MX53_NFC_BASE_ADDR_AXI;
		mxc_nandv2_mtd_device.resource[0].end =
					MX53_NFC_BASE_ADDR_AXI + SZ_8K - 1;
		mxc_nandv2_mtd_device.resource[1].start -= MX53_OFFSET;
		mxc_nandv2_mtd_device.resource[1].end -= MX53_OFFSET;
		ldb_resources[0].start -=  MX53_OFFSET;
		ldb_resources[0].end -=  MX53_OFFSET;
	} else if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		scc_resources[1].start += 0x8000;
		scc_resources[1].end += 0x8000;
	}


	if (cpu_is_mx51() || cpu_is_mx53())
		mxc_init_scc_iram();
	return 0;
}
postcore_initcall(mxc_init_devices);

