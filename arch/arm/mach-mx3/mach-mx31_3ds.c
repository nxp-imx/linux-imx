/*
 *  Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mfd/mc13783.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/fsl_devices.h>
#include <linux/input/matrix_keypad.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/fsl_devices.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/common.h>
#include <mach/board-mx31_3ds.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx3.h>
#include <mach/3ds_debugboard.h>
#include <mach/mmc.h>
#include <mach/mxc_nand.h>
#include <mach/spi.h>
#include "devices.h"
#include "crm_regs.h"

/*!
 * @file mx31_3ds.c
 *
 * @brief This file contains the board-specific initialization routines.
 *
 * @ingroup System
 */

static int mx31_3ds_pins[] = {
	/* UART1 */
	MX31_PIN_CTS1__CTS1,
	MX31_PIN_RTS1__RTS1,
	MX31_PIN_TXD1__TXD1,
	MX31_PIN_RXD1__RXD1,
	IOMUX_MODE(MX31_PIN_GPIO1_1, IOMUX_CONFIG_GPIO),
	/* SPI 1 */
	MX31_PIN_CSPI2_SCLK__SCLK,
	MX31_PIN_CSPI2_MOSI__MOSI,
	MX31_PIN_CSPI2_MISO__MISO,
	MX31_PIN_CSPI2_SPI_RDY__SPI_RDY,
	MX31_PIN_CSPI2_SS0__SS0,
	MX31_PIN_CSPI2_SS2__SS2, /*CS for MC13783 */
	/* MC13783 IRQ */
	IOMUX_MODE(MX31_PIN_GPIO1_3, IOMUX_CONFIG_GPIO),
	/* SDHC1 */
	MX31_PIN_SD1_DATA3__SD1_DATA3, MX31_PIN_SD1_DATA2__SD1_DATA2,
	MX31_PIN_SD1_DATA1__SD1_DATA1, MX31_PIN_SD1_DATA0__SD1_DATA0,
	MX31_PIN_SD1_CLK__SD1_CLK, MX31_PIN_SD1_CMD__SD1_CMD,
	MX31_PIN_GPIO3_0__GPIO3_0, MX31_PIN_GPIO3_1__GPIO3_1,
	/* SDHC2 */
	MX31_PIN_PC_PWRON__SD2_DATA3, MX31_PIN_PC_VS1__SD2_DATA2,
	MX31_PIN_PC_READY__SD2_DATA1, MX31_PIN_PC_WAIT_B__SD2_DATA0,
	MX31_PIN_PC_CD2_B__SD2_CLK, MX31_PIN_PC_CD1_B__SD2_CMD,
	MX31_PIN_GPIO1_2__GPIO1_2,
	/* AUDMUX (4 & 5) */
	MX31_PIN_SCK4__SCK4, MX31_PIN_SRXD4__SRXD4,
	MX31_PIN_STXD4__STXD4, MX31_PIN_SFS4__SFS4,
	MX31_PIN_STXD5__STXD5, MX31_PIN_SRXD5__SRXD5,
	MX31_PIN_SCK5__SCK5, MX31_PIN_SFS5__SFS5,

	MX31_PIN_LCS0__GPI03_23,
	MX31_PIN_SRST0__GPIO3_2,
	MX31_PIN_SIMPD0__GPIO2_3,
	MX31_PIN_BATT_LINE__GPIO2_17,
	/* USB OTG reset */
	IOMUX_MODE(MX31_PIN_USB_PWR, IOMUX_CONFIG_GPIO),
	/* USB OTG */
	MX31_PIN_USBOTG_DATA0__USBOTG_DATA0,
	MX31_PIN_USBOTG_DATA1__USBOTG_DATA1,
	MX31_PIN_USBOTG_DATA2__USBOTG_DATA2,
	MX31_PIN_USBOTG_DATA3__USBOTG_DATA3,
	MX31_PIN_USBOTG_DATA4__USBOTG_DATA4,
	MX31_PIN_USBOTG_DATA5__USBOTG_DATA5,
	MX31_PIN_USBOTG_DATA6__USBOTG_DATA6,
	MX31_PIN_USBOTG_DATA7__USBOTG_DATA7,
	MX31_PIN_USBOTG_CLK__USBOTG_CLK,
	MX31_PIN_USBOTG_DIR__USBOTG_DIR,
	MX31_PIN_USBOTG_NXT__USBOTG_NXT,
	MX31_PIN_USBOTG_STP__USBOTG_STP,
	/*Keyboard*/
	MX31_PIN_KEY_ROW0_KEY_ROW0,
	MX31_PIN_KEY_ROW1_KEY_ROW1,
	MX31_PIN_KEY_ROW2_KEY_ROW2,
	MX31_PIN_KEY_COL0_KEY_COL0,
	MX31_PIN_KEY_COL1_KEY_COL1,
	MX31_PIN_KEY_COL2_KEY_COL2,
	MX31_PIN_KEY_COL3_KEY_COL3,
};

/*
 * Matrix keyboard
 */

static const uint32_t mx31_3ds_keymap[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_DOWN),
	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_LEFT),
	KEY(1, 2, KEY_ENTER),
	KEY(2, 0, KEY_F6),
	KEY(2, 1, KEY_F8),
	KEY(2, 2, KEY_F9),
	KEY(2, 3, KEY_F10),
};

static struct matrix_keymap_data mx31_3ds_keymap_data = {
	.keymap		= mx31_3ds_keymap,
	.keymap_size	= ARRAY_SIZE(mx31_3ds_keymap),
};

static struct mxc_audio_platform_data mxc_audio_data = {
	.ssi_num = 1,
};

/* Regulators */
static struct regulator_init_data pwgtx_init = {
	.constraints = {
		.boot_on	= 1,
		.always_on	= 1,
	},
};

static struct mc13783_regulator_init_data mx31_3ds_regulators[] = {
	{
		.id = MC13783_REGU_PWGT1SPI, /* Power Gate for ARM core. */
		.init_data = &pwgtx_init,
	}, {
		.id = MC13783_REGU_PWGT2SPI, /* Power Gate for L2 Cache. */
		.init_data = &pwgtx_init,
	},
};

/* MC13783 */
static struct mc13783_platform_data mc13783_pdata __initdata = {
	.regulators = mx31_3ds_regulators,
	.num_regulators = ARRAY_SIZE(mx31_3ds_regulators),
	.flags  = MC13783_USE_REGULATOR,
};

/* SPI */
static int spi1_internal_chipselect[] = {
	MXC_SPI_CS(0),
	MXC_SPI_CS(2),
};

static struct spi_imx_master spi1_pdata = {
	.chipselect	= spi1_internal_chipselect,
	.num_chipselect	= ARRAY_SIZE(spi1_internal_chipselect),
};

static struct spi_board_info mx31_3ds_spi_devs[] __initdata = {
	{
		.modalias	= "mc13783",
		.max_speed_hz	= 1000000,
		.bus_num	= 1,
		.chip_select	= 1, /* SS2 */
		.platform_data	= &mc13783_pdata,
		.irq		= IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
		.mode = SPI_CS_HIGH,
	},
};

/*
 * NAND Flash
 */
static struct mxc_nand_platform_data imx31_3ds_nand_flash_pdata = {
	.width		= 1,
	.hw_ecc		= 1,
#ifdef MACH_MX31_3DS_MXC_NAND_USE_BBT
	.flash_bbt	= 1,
#endif
};

/*
 * USB OTG
 */

#define USB_PAD_CFG (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST | PAD_CTL_HYS_CMOS | \
		     PAD_CTL_ODE_CMOS | PAD_CTL_100K_PU)

#define USBOTG_RST_B IOMUX_TO_GPIO(MX31_PIN_USB_PWR)

static void mx31_3ds_usbotg_init(void)
{
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA0, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA1, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA2, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA3, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA4, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA5, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA6, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA7, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_CLK, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DIR, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_NXT, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_STP, USB_PAD_CFG);

	gpio_request(USBOTG_RST_B, "otgusb-reset");
	gpio_direction_output(USBOTG_RST_B, 0);
	mdelay(1);
	gpio_set_value(USBOTG_RST_B, 1);
}

static struct fsl_usb2_platform_data usbotg_pdata = {
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_ULPI,
};

static struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

#define PLL_PCTL_REG(pd, mfd, mfi, mfn)		\
	((((pd) - 1) << 26) + (((mfd) - 1) << 16) + ((mfi)  << 10) + mfn)

/* For 26MHz input clock */
#define PLL_532MHZ		PLL_PCTL_REG(1, 13, 10, 3)
#define PLL_399MHZ		PLL_PCTL_REG(1, 52, 7, 35)
#define PLL_133MHZ		PLL_PCTL_REG(2, 26, 5, 3)

#define PDR0_REG(mcu, max, hsp, ipg, nfc)	\
	(MXC_CCM_PDR0_MCU_DIV_##mcu | MXC_CCM_PDR0_MAX_DIV_##max | \
	 MXC_CCM_PDR0_HSP_DIV_##hsp | MXC_CCM_PDR0_IPG_DIV_##ipg | \
	 MXC_CCM_PDR0_NFC_DIV_##nfc)

/* working point(wp): 0 - 133MHz; 1 - 266MHz; 2 - 399MHz; 3 - 532MHz */
/* 26MHz input clock table */
static struct cpu_wp cpu_wp_26[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 4;
	return cpu_wp_26;
}

/*
 * Set up static virtual mappings.
 */
static void __init mx31_3ds_map_io(void)
{
	mx31_map_io();
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	struct clk *pll_clk;
	struct clk *ckih_clk;
	struct clk *cko_clk;
	void __iomem *iim_reg = MX31_IO_ADDRESS(MX31_IIM_BASE_ADDR);

	/* request gpio for phone jack detect */
	mxc_iomux_set_pad(MX31_PIN_BATT_LINE, PAD_CTL_PKE_NONE);
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_BATT_LINE), "batt_line");
	gpio_direction_input(IOMUX_TO_GPIO(MX31_PIN_BATT_LINE));

	pll_clk = clk_get(NULL, "usb_pll");
	mxc_audio_data.ssi_clk[0] = clk_get(NULL, "ssi_clk.0");
	if (IS_ERR(pll_clk) || IS_ERR(mxc_audio_data.ssi_clk[0])) {
		pr_err("cannot get usb pll or ssi clks\n");
	} else {
		clk_set_parent(mxc_audio_data.ssi_clk[0], pll_clk);
		clk_put(mxc_audio_data.ssi_clk[0]);
		clk_put(pll_clk);
	}

	/* override fuse for Hantro HW clock */
	if (__raw_readl(iim_reg + 0x808) == 0x4) {
		if (!(__raw_readl(iim_reg + 0x800) & (1 << 5))) {
			writel(__raw_readl(iim_reg + 0x808) & 0xfffffffb,
					   iim_reg + 0x808);
		}
	}

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "cko1_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CKO1 output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	mxc_iomux_setup_multiple_pins(mx31_3ds_pins, ARRAY_SIZE(mx31_3ds_pins),
				      "mx31_3ds");

	mxc_register_device(&mxc_uart_device0, &uart_pdata);
	mxc_register_device(&mxc_nand_device, &imx31_3ds_nand_flash_pdata);

	mxc_register_device(&mxc_spi_device1, &spi1_pdata);
	spi_register_board_info(mx31_3ds_spi_devs,
						ARRAY_SIZE(mx31_3ds_spi_devs));

	mxc_register_device(&imx_kpp_device, &mx31_3ds_keymap_data);

	mx31_3ds_usbotg_init();
	mxc_register_device(&mxc_otg_udc_device, &usbotg_pdata);

	if (mxc_expio_init(CS5_BASE_ADDR, EXPIO_PARENT_INT))
		printk(KERN_WARNING "Init of the debugboard failed, all "
				   "devices on the board are unusable.\n");

}

static void __init mx31_3ds_timer_init(void)
{
	mx31_clocks_init(26000000);
}

static struct sys_timer mx31_3ds_timer = {
	.init	= mx31_3ds_timer_init,
};

/*
 * The following uses standard kernel macros defined in arch.h in order to
 * initialize __mach_desc_MX31_3DS data structure.
 */
MACHINE_START(MX31_3DS, "Freescale MX31PDK (3DS)")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io	= MX31_AIPS1_BASE_ADDR,
	.io_pg_offst	= (MX31_AIPS1_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.boot_params    = MX3x_PHYS_OFFSET + 0x100,
	.map_io         = mx31_3ds_map_io,
	.init_irq       = mx31_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &mx31_3ds_timer,
MACHINE_END
