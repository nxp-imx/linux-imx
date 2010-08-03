/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/memory.h>
#include <linux/gpio.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2007.h>
#include <linux/mfd/mc9s08dz60/pmic.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

#include <asm/mach/flash.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx35.h>
#include <mach/board-mx35_3ds.h>
#include <mach/3ds_debugboard.h>
#include <mach/mmc.h>
#include <mach/spi.h>
#include <mach/iomux-v3.h>

#include "devices-imx35.h"
#include "devices.h"
#include "crm_regs.h"

/*
 * Board specific pad setting.
 */
#define MX35_ESDHC1_CMD__PAD_CFG 	\
(					\
	PAD_CTL_PUE 		| 	\
	PAD_CTL_PKE 		| 	\
	PAD_CTL_HYS 		| 	\
	PAD_CTL_DSE_MAX 	|	\
	PAD_CTL_PUS_47K_UP 	|	\
	PAD_CTL_SRE_FAST		\
)

#define MX35_ESDHC1_CLK__PAD_CFG	\
(					\
	PAD_CTL_PUE 		|	\
	PAD_CTL_PKE 		| 	\
	PAD_CTL_DSE_MAX 	|	\
	PAD_CTL_PUS_47K_UP 	|	\
	PAD_CTL_SRE_FAST		\
)

#define MX35_ESDHC1_DATA__PAD_CFG 	\
	MX35_ESDHC1_CMD__PAD_CFG

static const struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static int spi_internal_chipselect[] = {
	MXC_SPI_CS(0),
	MXC_SPI_CS(1),
	MXC_SPI_CS(2),
	MXC_SPI_CS(3),
};

static const struct spi_imx_master spi_pdata = {
	.chipselect = spi_internal_chipselect,
	.num_chipselect = ARRAY_SIZE(spi_internal_chipselect),
};

static const struct imxi2c_platform_data i2c_data = {
	.bitrate = 100000,
};

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	unsigned int ret;

	if (board_is_rev(BOARD_REV_2))
		pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2, 7, 1);

	if (to_platform_device(dev)->id == 0) {
		if (0 != pmic_gpio_get_designation_bit_val(2, &ret))
			printk(KERN_ERR "Get cd status error.");
		return ret;
	}

	return 0;
}

static int sdhc_write_protect(struct device *dev)
{
	unsigned int rc = 0;

	if (0 != pmic_gpio_get_designation_bit_val(3, &rc))
		printk(KERN_ERR "Get wp status error.");

	return rc;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_32_33,
#if defined(CONFIG_SDIO_UNIFI_FS) || defined(CONFIG_SDIO_UNIFI_FS_MODULE)
	.caps = MMC_CAP_4_BIT_DATA,
#else
	.caps = MMC_CAP_8_BIT_DATA,
#endif
	.min_clk = 150000,
	.max_clk = 52000000,
	/* Do not disable the eSDHC clk on MX35 3DS board,
	* since SYSTEM can't boot up after the reset key
	* is pressed when the SD/MMC boot mode is used.
	* The root cause is that the ROM code don't ensure
	* the SD/MMC clk is running when boot system.
	* */
	.clk_always_on = 1,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "sdhc_clk",
};

/* MTD NOR flash */
static struct mtd_partition mxc_nor_partitions[] = {
	{
	 .name = "Bootloader",
	 .size = 512 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE,
	}, {
	 .name = "nor.Kernel",
	 .size = 4 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	}, {
	 .name = "nor.userfs",
	 .size = 30 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0,
	}, {
	 .name = "nor.rootfs",
	 .size = 28 * 1024 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE,
	}, {
	 .name = "FIS directory",
	 .size = 12 * 1024,
	 .offset = 0x01FE0000,
	 .mask_flags = MTD_WRITEABLE,
	}, {
	 .name = "Redboot config",
	 .size = MTDPART_SIZ_FULL,
	 .offset = 0x01FFF000,
	 .mask_flags = MTD_WRITEABLE,
	},
};

static struct flash_platform_data mxc_nor_flash_pdata = {
	.map_name = "cfi_probe",
	.width = 2,
	.parts = mxc_nor_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nor_partitions),
};

/* MTD NAND flash */
static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 3 * 1024 * 1024,
	}, {
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * 1024 * 1024,
	}, {
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024,
	}, {
	 .name = "nand.configure",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 8 * 1024 * 1024,
	}, {
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data mxc_nandv2_pdata = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static void __init mxc_register_nandv2_devices(void)
{
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B)
		mxc_nandv2_pdata.width = 2;

	mxc_register_device(&mxc_nandv2_device, &mxc_nandv2_pdata);
}

static void si4702_reset(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 4, 0);
	msleep(100);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 4, 1);
	msleep(100);
}

static void si4702_clock_ctl(int flag)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 7, flag);
}

static struct mxc_fm_platform_data si4702_data = {
	.reg_vio = "VSD",
	.reg_vdd = NULL,
	.reset = si4702_reset,
	.clock_ctl = si4702_clock_ctl,
	.sksnr = 0,
	.skcnt = 0,
	.band = 0,
	.space = 100,
	.seekth = 0xa,
};

static int tsc2007_get_pendown_state(void)
{
	return !gpio_get_value(MX35_GPIO1_4);
}

struct tsc2007_platform_data tsc2007_data = {
	.model = 2007,
	.x_plate_ohms = 400,
	.get_pendown_state = tsc2007_get_pendown_state,
};

static struct mxc_camera_platform_data camera_data = {
	.core_regulator = "SW1",
	.io_regulator = "VAUDIO",
	.analog_regulator = NULL,
	.gpo_regulator = "PWGT1",
	.mclk = 27000000,
};

static void adv7180_pwdn(int pwdn)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_1, 1, ~pwdn);
}

static void adv7180_reset(void)
{
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 6, 0);
	msleep(5);
	pmic_gpio_set_bit_val(MCU_GPIO_REG_RESET_1, 6, 1);
	msleep(5);
}

static struct mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg = NULL,
	.dvdd_reg = "SW3",
	.avdd_reg = "PWGT2",
	.pvdd_reg = NULL,
	.pwdn = adv7180_pwdn,
	.reset = adv7180_reset,
};

static struct i2c_board_info mxc_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max8660", 0x34),
	}, {	I2C_BOARD_INFO("tsc2007", 0x48),
		.platform_data = &tsc2007_data,
		.irq = IOMUX_TO_IRQ(MX35_GPIO1_4),
	}, {
		I2C_BOARD_INFO("ov2640", 0x30),
		.platform_data = (void *)&camera_data,
	}, {
		I2C_BOARD_INFO("sgtl5000-i2c", 0x0a),
	}, {
		I2C_BOARD_INFO("ak4647-i2c", 0x12),
	}, {
#if defined(CONFIG_I2C_SLAVE_CLIENT)
		I2C_BOARD_INFO("i2c-slave-client", 0x55),
#endif
	}, {
		I2C_BOARD_INFO("si4702", 0x10),
		.platform_data = (void *)&si4702_data,
	}, {
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&mxc_fec_device,
	&mxc_dma_device,
};

static struct pad_cfg mx35pdk_pads[] = {
	/* UART1 */
	{MX35_PAD_CTS1__UART1_CTS,             },
	{MX35_PAD_RTS1__UART1_RTS,             },
	{MX35_PAD_TXD1__UART1_TXD_MUX,         },
	{MX35_PAD_RXD1__UART1_RXD_MUX,         },
	/* FEC */
	{MX35_PAD_FEC_TX_CLK__FEC_TX_CLK,      },
	{MX35_PAD_FEC_RX_CLK__FEC_RX_CLK,      },
	{MX35_PAD_FEC_RX_DV__FEC_RX_DV,        },
	{MX35_PAD_FEC_COL__FEC_COL,},
	{MX35_PAD_FEC_RDATA0__FEC_RDATA_0,     },
	{MX35_PAD_FEC_TDATA0__FEC_TDATA_0,     },
	{MX35_PAD_FEC_TX_EN__FEC_TX_EN,        },
	{MX35_PAD_FEC_MDC__FEC_MDC,            },
	{MX35_PAD_FEC_MDIO__FEC_MDIO,          },
	{MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,      },
	{MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,      },
	{MX35_PAD_FEC_CRS__FEC_CRS,            },
	{MX35_PAD_FEC_RDATA1__FEC_RDATA_1,     },
	{MX35_PAD_FEC_TDATA1__FEC_TDATA_1,     },
	{MX35_PAD_FEC_RDATA2__FEC_RDATA_2,     },
	{MX35_PAD_FEC_TDATA2__FEC_TDATA_2,     },
	{MX35_PAD_FEC_RDATA3__FEC_RDATA_3,     },
	{MX35_PAD_FEC_TDATA3__FEC_TDATA_3,     },
	/* USBOTG */
	{MX35_PAD_USBOTG_PWR__USB_TOP_USBOTG_PWR,},
	{MX35_PAD_USBOTG_OC__USB_TOP_USBOTG_OC,  },

	/* SD/MMC-SDHC1 */
	{MX35_PAD_SD1_CMD__ESDHC1_CMD, MX35_ESDHC1_CMD__PAD_CFG},
	{MX35_PAD_SD1_CLK__ESDHC1_CLK, MX35_ESDHC1_CLK__PAD_CFG},
	{MX35_PAD_SD1_DATA0__ESDHC1_DAT0, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD1_DATA1__ESDHC1_DAT1, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD1_DATA2__ESDHC1_DAT2, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD1_DATA3__ESDHC1_DAT3, MX35_ESDHC1_DATA__PAD_CFG},
#if !defined(CONFIG_SDIO_UNIFI_FS) && !defined(CONFIG_SDIO_UNIFI_FS_MODULE)
	{MX35_PAD_SD2_CMD__ESDHC1_DAT4, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD2_CLK__ESDHC1_DAT5, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD2_DATA0__ESDHC1_DAT6, MX35_ESDHC1_DATA__PAD_CFG},
	{MX35_PAD_SD2_DATA1__ESDHC1_DAT7, MX35_ESDHC1_DATA__PAD_CFG},
#endif
	/* Capature */
	{MX35_PAD_CAPTURE__GPIO1_4,},
};

/* OTG config */
static struct fsl_usb2_platform_data usb_pdata = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI_WIDE,
};

/*
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_iomux_v3_setup_multiple_pads_ext(mx35pdk_pads,
						ARRAY_SIZE(mx35pdk_pads));

	platform_add_devices(devices, ARRAY_SIZE(devices));

	imx35_add_imx_uart0(&uart_pdata);
	imx35_add_imx_uart1(&uart_pdata);
	imx35_add_imx_uart2(&uart_pdata);

	imx35_add_spi_imx0(&spi_pdata);
	imx35_add_spi_imx1(&spi_pdata);

	mxc_register_device(&mxc_otg_udc_device, &usb_pdata);

	mxc_register_device(&mxc_nor_mtd_device, &mxc_nor_flash_pdata);

	mxc_register_nandv2_devices();

	i2c_register_board_info(0, mxc_i2c_board_info,
				ARRAY_SIZE(mxc_i2c_board_info));
	imx35_add_imx_i2c0(&i2c_data);

	mx35_3stack_init_mc13892();
	mx35_3stack_init_mc9s08dz60();

	imx35_add_imx_mmc0(&mmc1_data);

	if (mxc_expio_init(CS5_BASE_ADDR, EXPIO_PARENT_INT))
		printk(KERN_WARNING "Init of the debugboard failed, all "
				   "devices on the board are unusable.\n");
}

static void __init mx35pdk_timer_init(void)
{
	mx35_clocks_init();
}

struct sys_timer mx35pdk_timer = {
	.init	= mx35pdk_timer_init,
};

MACHINE_START(MX35_3DS, "Freescale MX35PDK")
	/* Maintainer: Freescale Semiconductor, Inc */
	.phys_io	= MX35_AIPS1_BASE_ADDR,
	.io_pg_offst	= ((MX35_AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = MX3x_PHYS_OFFSET + 0x100,
	.map_io         = mx35_map_io,
	.init_irq       = mx35_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &mx35pdk_timer,
MACHINE_END
