/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"

#define MX6Q_SABREAUTO_LDB_BACKLIGHT	IMX_GPIO_NR(1, 9)
#define MX6Q_SABREAUTO_ECSPI1_CS0	IMX_GPIO_NR(2, 30)
#define MX6Q_SABREAUTO_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX6Q_SABREAUTO_DISP0_PWR	IMX_GPIO_NR(3, 24)
#define MX6Q_SABREAUTO_DISP0_I2C_EN	IMX_GPIO_NR(3, 28)
#define MX6Q_SABREAUTO_DISP0_DET_INT	IMX_GPIO_NR(3, 31)
#define MX6Q_SABREAUTO_DISP0_RESET	IMX_GPIO_NR(5, 0)
#define MX6Q_SABREAUTO_SD3_CD	IMX_GPIO_NR(6, 11)
#define MX6Q_SABREAUTO_SD3_WP	IMX_GPIO_NR(6, 14)
#define MX6Q_SABREAUTO_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6Q_SABREAUTO_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)
#define MX6Q_SABREAUTO_CAP_TCH_INT	IMX_GPIO_NR(3, 31)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;
static int esai_record;

static iomux_v3_cfg_t mx6q_sabreauto_pads[] = {

	/* UART4 for debug */
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,

	/* ENET */
	MX6Q_PAD_KEY_COL1__ENET_MDIO,
	MX6Q_PAD_KEY_COL2__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_GPIO_0__CCM_CLKO,
	MX6Q_PAD_GPIO_3__CCM_CLKO2,

	/* SD1 */
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
	/* SD2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	/* SD3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7,
	MX6Q_PAD_SD3_RST__USDHC3_RST,
	/* SD3 VSelect */
	MX6Q_PAD_GPIO_18__USDHC3_VSELECT,
	/* SD3_CD and SD3_WP */
	MX6Q_PAD_NANDF_CS0__GPIO_6_11,
	MX6Q_PAD_NANDF_CS1__GPIO_6_14,
	/* SD4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7,
	MX6Q_PAD_NANDF_ALE__USDHC4_RST,
	/* eCSPI1 */
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,

	/* ESAI */
	MX6Q_PAD_ENET_RXD0__ESAI1_HCKT,
	MX6Q_PAD_ENET_CRS_DV__ESAI1_SCKT,
	MX6Q_PAD_ENET_RXD1__ESAI1_FST,
	MX6Q_PAD_ENET_TX_EN__ESAI1_TX3_RX2,
	MX6Q_PAD_ENET_TXD1__ESAI1_TX2_RX3,
	MX6Q_PAD_ENET_TXD0__ESAI1_TX4_RX1,
	MX6Q_PAD_ENET_MDC__ESAI1_TX5_RX0,
	MX6Q_PAD_NANDF_CS2__ESAI1_TX0,
	MX6Q_PAD_NANDF_CS3__ESAI1_TX1,

	/* I2C1 */
	MX6Q_PAD_CSI0_DAT8__I2C1_SDA,
	MX6Q_PAD_CSI0_DAT9__I2C1_SCL,

	/* I2C2 */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,

	MX6Q_PAD_EIM_D24__GPIO_3_24,

	/* ldb: pwm fixme*/
	MX6Q_PAD_GPIO_9__GPIO_1_9,

	/* DISP0 I2C ENABLE*/
	MX6Q_PAD_EIM_D28__GPIO_3_28,

	/* DISP0 DET */
	MX6Q_PAD_EIM_D31__GPIO_3_31,

	/* DISP0 RESET */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,

	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,
	MX6Q_PAD_GPIO_16__I2C3_SDA,

	/* HDMI */
	MX6Q_PAD_EIM_A25__HDMI_TX_CEC_LINE,
	MX6Q_PAD_SD1_DAT1__HDMI_TX_OPHYDTB_0,
	MX6Q_PAD_SD1_DAT0__HDMI_TX_OPHYDTB_1,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,
};

static iomux_v3_cfg_t mx6q_sabreauto_esai_record_pads[] = {
	MX6Q_PAD_ENET_RX_ER__ESAI1_HCKR,
	MX6Q_PAD_ENET_MDIO__ESAI1_SCKR,
	MX6Q_PAD_ENET_REF_CLK__ESAI1_FSR,
};

static const struct esdhc_platform_data mx6q_sabreauto_sd3_data __initconst = {
	.cd_gpio = MX6Q_SABREAUTO_SD3_CD,
	.wp_gpio = MX6Q_SABREAUTO_SD3_WP,
	.support_18v = 1,
	.support_8bit = 1,
};

/* No card detect signal for SD4 */
static const struct esdhc_platform_data mx6q_sabreauto_sd4_data __initconst = {
	.always_present = 1,
	.support_8bit = 1,
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx6q_gpmi_nand[] __initdata = {
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_CS1__RAWNAND_CE1N,
	MX6Q_PAD_NANDF_CS2__RAWNAND_CE2N,
	MX6Q_PAD_NANDF_CS3__RAWNAND_CE3N,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_SD4_DAT0__RAWNAND_DQS,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
};

static int gpmi_nfc_platform_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(mx6q_gpmi_nand,
					ARRAY_SIZE(mx6q_gpmi_nand));
}

static const struct gpmi_nfc_platform_data
mx6q_gpmi_nfc_platform_data __initconst = {
	.platform_init           = gpmi_nfc_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
};

static const struct anatop_thermal_platform_data mx6q_sabreauto_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static inline void mx6q_sabreauto_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(3, NULL);
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
}

static struct fec_platform_data fec_data __initdata = {
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static inline void imx6q_init_fec(void)
{
	random_ether_addr(fec_data.mac);
	imx6q_add_fec(&fec_data);
}

static int mx6q_sabreauto_spi_cs[] = {
	MX6Q_SABREAUTO_ECSPI1_CS0,
	MX6Q_SABREAUTO_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabreauto_spi_data __initconst = {
	.chipselect     = mx6q_sabreauto_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabreauto_spi_cs),
};

static int max7310_1_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	int max7310_gpio_value[] = {
		0, 1, 0, 1, 0, 0, 0, 0,
	};

	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 1 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_platdata = {
	.gpio_base	= MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_1_setup,
};

static int max7310_u48_setup(struct i2c_client *client,
	unsigned gpio_base, unsigned ngpio,
	void *context)
{
	int max7310_gpio_value[] = {
		0, 1, 1, 1, 0, 0, 0, 0,
	};

	int n;

	 for (n = 0; n < ARRAY_SIZE(max7310_gpio_value); ++n) {
		gpio_request(gpio_base + n, "MAX7310 U48 GPIO Expander");
		if (max7310_gpio_value[n] < 0)
			gpio_direction_input(gpio_base + n);
		else
			gpio_direction_output(gpio_base + n,
						max7310_gpio_value[n]);
		gpio_export(gpio_base + n, 0);
	}

	return 0;
}

static struct pca953x_platform_data max7310_u48_platdata = {
	.gpio_base	= MX6Q_SABREAUTO_MAX7310_2_BASE_ADDR,
	.invert		= 0,
	.setup		= max7310_u48_setup,
};

static void ddc_dvi_init(void)
{
	/* enable DVI I2C */
	gpio_set_value(MX6Q_SABREAUTO_DISP0_I2C_EN, 1);
}

static int ddc_dvi_update(void)
{
	/* DVI cable state */
	if (gpio_get_value(MX6Q_SABREAUTO_DISP0_DET_INT) == 1)
		return 1;
	else
		return 0;
}

static struct fsl_mxc_dvi_platform_data sabr_ddc_dvi_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.init = ddc_dvi_init,
	.update = ddc_dvi_update,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs42888", 0x48),
	},
};

static struct imxi2c_platform_data mx6q_sabreauto_i2c_data = {
	.bitrate = 400000,
};

static struct imxi2c_platform_data mx6q_sabreauto_i2c0_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max7310", 0x1F),
		.platform_data = &max7310_platdata,
	},
	{
		I2C_BOARD_INFO("max7310", 0x1B),
		.platform_data = &max7310_u48_platdata,
	},
	{
		I2C_BOARD_INFO("mxc_dvi", 0x50),
		.platform_data = &sabr_ddc_dvi_data,
		.irq = gpio_to_irq(MX6Q_SABREAUTO_DISP0_DET_INT),
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_SABREAUTO_CAP_TCH_INT),
	},

};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_SABREAUTO_CAP_TCH_INT),
	},
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static void imx6q_sabreauto_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6Q_SABREAUTO_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_SABREAUTO_USB_OTG_PWR, 0);
}

static void __init imx6q_sabreauto_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect, or it will affect signal quality at dp */

	ret = gpio_request(MX6Q_SABREAUTO_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO MX6Q_SABREAUTO_USB_OTG_PWR: %d\n", ret);
		return;
	}
	gpio_direction_output(MX6Q_SABREAUTO_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabreauto_usbotg_vbus);
	mx6_usb_dr_init();
	mx6_usb_h1_init();
}
static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabreauto_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR, 1);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	sata_init(addr, tmpdata);

	return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_sabreauto_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR, 0);

}

static struct ahci_platform_data mx6q_sabreauto_sata_data = {
	.init = mx6q_sabreauto_sata_init,
	.exit = mx6q_sabreauto_sata_exit,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data sabr_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static struct fsl_mxc_lcd_platform_data hdmi_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB24,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	}, {
	.rev = 4,
	},
};

static void sabreauto_suspend_enter(void)
{
	/* suspend preparation */
}

static void sabreauto_suspend_exit(void)
{
	/* resmue resore */
}
static const struct pm_platform_data mx6q_sabreauto_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabreauto_suspend_enter,
	.suspend_exit = sabreauto_suspend_exit,
};

static struct mxc_audio_platform_data sab_audio_data = {
	.sysclk = 16934400,
};

static struct platform_device sab_audio_device = {
	.name = "imx-cs42888",
};

static struct imx_esai_platform_data sab_esai_pdata = {
	.flags = IMX_ESAI_NET,
};

static int imx6q_init_audio(void)
{
	struct clk *pll3_pfd, *esai_clk;
	mxc_register_device(&sab_audio_device, &sab_audio_data);
	imx6q_add_imx_esai(0, &sab_esai_pdata);

	esai_clk = clk_get(NULL, "esai_clk");
	if (IS_ERR(esai_clk))
		return PTR_ERR(esai_clk);

	pll3_pfd = clk_get(NULL, "pll3_pfd_508M");
	if (IS_ERR(pll3_pfd))
		return PTR_ERR(pll3_pfd);

	clk_set_parent(esai_clk, pll3_pfd);
	clk_set_rate(esai_clk, 101647058);

	return 0;
}

static int __init early_use_esai_record(char *p)
{
	esai_record = 1;
	return 0;
}

early_param("esai_record", early_use_esai_record);
/*!
 * Board specific initialization.
 */
static void __init mx6_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_pads,
					ARRAY_SIZE(mx6q_sabreauto_pads));

	if (esai_record)
	    mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_esai_record_pads,
			ARRAY_SIZE(mx6q_sabreauto_esai_record_pads));

	mx6q_sabreauto_init_uart();

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
		imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);

	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_i2c(0, &mx6q_sabreauto_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6q_sabreauto_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabreauto_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabreauto_anatop_thermal_data);

	if (!esai_record)
		imx6q_init_fec();

	imx6q_add_pm_imx(0, &mx6q_sabreauto_pm_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabreauto_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabreauto_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabreauto_init_usb();
	imx6q_add_ahci(0, &mx6q_sabreauto_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "mxc_alsa_spdif.1");
	imx6q_add_asrc(&imx_asrc_data);

	/* DISP0 Detect */
	gpio_request(MX6Q_SABREAUTO_DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(MX6Q_SABREAUTO_DISP0_DET_INT);

	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(MX6Q_SABREAUTO_DISP0_RESET, "disp0-reset");
	gpio_direction_output(MX6Q_SABREAUTO_DISP0_RESET, 0);

	/* DISP0 I2C enable */
	gpio_request(MX6Q_SABREAUTO_DISP0_I2C_EN, "disp0-i2c");
	gpio_direction_output(MX6Q_SABREAUTO_DISP0_I2C_EN, 0);

	gpio_request(MX6Q_SABREAUTO_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(MX6Q_SABREAUTO_DISP0_PWR, 1);

	gpio_request(MX6Q_SABREAUTO_LDB_BACKLIGHT, "ldb-backlight");
	gpio_direction_output(MX6Q_SABREAUTO_LDB_BACKLIGHT, 1);
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_gpmi(&mx6q_gpmi_nfc_platform_data);
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6q_reserve(void)
{
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABREAUTO data structure.
 */
MACHINE_START(MX6Q_SABREAUTO, "Freescale i.MX 6Quad SABRE Auto Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_board_init,
	.timer = &mxc_timer,
	.reserve = mx6q_reserve,
MACHINE_END
