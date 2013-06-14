/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
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
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sabresd.h"
#include "board-mx6dl_sabresd.h"

#define SABRESD_USR_DEF_GRN_LED	IMX_GPIO_NR(1, 1)
#define SABRESD_USR_DEF_RED_LED	IMX_GPIO_NR(1, 2)
#define SABRESD_VOLUME_UP	IMX_GPIO_NR(1, 4)
#define SABRESD_VOLUME_DN	IMX_GPIO_NR(1, 5)
#define SABRESD_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define SABRESD_CSI0_PWN	IMX_GPIO_NR(1, 16)
#define SABRESD_CSI0_RST	IMX_GPIO_NR(1, 17)
#define SABRESD_ACCL_INT	IMX_GPIO_NR(1, 18)
#define SABRESD_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define SABRESD_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define SABRESD_RGMII_RST	IMX_GPIO_NR(1, 25)
#define SABRESD_RGMII_INT	IMX_GPIO_NR(1, 26)
#define SABRESD_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define SABRESD_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define SABRESD_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define SABRESD_SD3_CD		IMX_GPIO_NR(2, 0)
#define SABRESD_SD3_WP		IMX_GPIO_NR(2, 1)
#define SABRESD_SD2_CD		IMX_GPIO_NR(2, 2)
#define SABRESD_SD2_WP		IMX_GPIO_NR(2, 3)
#define SABRESD_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define SABRESD_GPS_RESET	IMX_GPIO_NR(2, 28)
#define SABRESD_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define SABRESD_GPS_EN	IMX_GPIO_NR(3, 0)
#define SABRESD_DISP0_RST_B	IMX_GPIO_NR(3, 8)
#define SABRESD_ALS_INT		IMX_GPIO_NR(3, 9)
#define SABRESD_CHARGE_CHG_2_B	IMX_GPIO_NR(3, 13)
#define SABRESD_CHARGE_FLT_2_B	IMX_GPIO_NR(3, 14)
#define SABRESD_BAR0_INT	IMX_GPIO_NR(3, 15)
#define SABRESD_eCOMPASS_INT	IMX_GPIO_NR(3, 16)
#define SABRESD_GPS_PPS		IMX_GPIO_NR(3, 18)
#define SABRESD_PCIE_PWR_EN	IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define SABRESD_USB_H1_PWR	IMX_GPIO_NR(1, 29)
#define SABRESD_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define SABRESD_TS_INT		IMX_GPIO_NR(3, 26)
#define SABRESD_DISP0_RD	IMX_GPIO_NR(3, 28)
#define SABRESD_POWER_OFF	IMX_GPIO_NR(3, 29)

#define SABRESD_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define SABRESD_ECSPI1_CS0  IMX_GPIO_NR(4, 9)
#define SABRESD_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define SABRESD_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define SABRESD_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define SABRESD_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define SABRESD_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define SABRESD_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define SABRESD_CAP_TCH_INT1	IMX_GPIO_NR(6, 7)
#define SABRESD_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define SABRESD_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define SABRESD_CABC_EN0	IMX_GPIO_NR(6, 15)
#define SABRESD_CABC_EN1	IMX_GPIO_NR(6, 16)
#define SABRESD_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define SABRESD_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define SABRESD_AUX_5V_EN	IMX_GPIO_NR(6, 10)
#define SABRESD_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define SABRESD_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define SABRESD_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define SABRESD_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define SABRESD_PFUZE_INT	IMX_GPIO_NR(7, 13)

#define SABRESD_EPDC_SDDO_0	IMX_GPIO_NR(2, 22)
#define SABRESD_EPDC_SDDO_1	IMX_GPIO_NR(3, 10)
#define SABRESD_EPDC_SDDO_2	IMX_GPIO_NR(3, 12)
#define SABRESD_EPDC_SDDO_3	IMX_GPIO_NR(3, 11)
#define SABRESD_EPDC_SDDO_4	IMX_GPIO_NR(2, 27)
#define SABRESD_EPDC_SDDO_5	IMX_GPIO_NR(2, 30)
#define SABRESD_EPDC_SDDO_6	IMX_GPIO_NR(2, 23)
#define SABRESD_EPDC_SDDO_7	IMX_GPIO_NR(2, 26)
#define SABRESD_EPDC_SDDO_8	IMX_GPIO_NR(2, 24)
#define SABRESD_EPDC_SDDO_9	IMX_GPIO_NR(3, 15)
#define SABRESD_EPDC_SDDO_10	IMX_GPIO_NR(3, 16)
#define SABRESD_EPDC_SDDO_11	IMX_GPIO_NR(3, 23)
#define SABRESD_EPDC_SDDO_12	IMX_GPIO_NR(3, 19)
#define SABRESD_EPDC_SDDO_13	IMX_GPIO_NR(3, 13)
#define SABRESD_EPDC_SDDO_14	IMX_GPIO_NR(3, 14)
#define SABRESD_EPDC_SDDO_15	IMX_GPIO_NR(5, 2)
#define SABRESD_EPDC_GDCLK	IMX_GPIO_NR(2, 17)
#define SABRESD_EPDC_GDSP	IMX_GPIO_NR(2, 16)
#define SABRESD_EPDC_GDOE	IMX_GPIO_NR(6, 6)
#define SABRESD_EPDC_GDRL	IMX_GPIO_NR(5, 4)
#define SABRESD_EPDC_SDCLK	IMX_GPIO_NR(3, 31)
#define SABRESD_EPDC_SDOEZ	IMX_GPIO_NR(3, 30)
#define SABRESD_EPDC_SDOED	IMX_GPIO_NR(3, 26)
#define SABRESD_EPDC_SDOE	IMX_GPIO_NR(3, 27)
#define SABRESD_EPDC_SDLE	IMX_GPIO_NR(3, 1)
#define SABRESD_EPDC_SDCLKN	IMX_GPIO_NR(3, 0)
#define SABRESD_EPDC_SDSHR	IMX_GPIO_NR(2, 29)
#define SABRESD_EPDC_PWRCOM	IMX_GPIO_NR(2, 28)
#define SABRESD_EPDC_PWRSTAT	IMX_GPIO_NR(2, 21)
#define SABRESD_EPDC_PWRCTRL0	IMX_GPIO_NR(2, 20)
#define SABRESD_EPDC_PWRCTRL1	IMX_GPIO_NR(2, 19)
#define SABRESD_EPDC_PWRCTRL2	IMX_GPIO_NR(2, 18)
#define SABRESD_EPDC_PWRCTRL3	IMX_GPIO_NR(3, 28)
#define SABRESD_EPDC_BDR0	IMX_GPIO_NR(3, 2)
#define SABRESD_EPDC_BDR1	IMX_GPIO_NR(3, 3)
#define SABRESD_EPDC_SDCE0	IMX_GPIO_NR(3, 4)
#define SABRESD_EPDC_SDCE1	IMX_GPIO_NR(3, 5)
#define SABRESD_EPDC_SDCE2	IMX_GPIO_NR(3, 6)
#define SABRESD_EPDC_SDCE3	IMX_GPIO_NR(3, 7)
#define SABRESD_EPDC_SDCE4	IMX_GPIO_NR(3, 8)
#define SABRESD_EPDC_PMIC_WAKE	IMX_GPIO_NR(3, 20)
#define SABRESD_EPDC_PMIC_INT	IMX_GPIO_NR(2, 25)
#define SABRESD_EPDC_VCOM	IMX_GPIO_NR(3, 17)
#define SABRESD_CHARGE_NOW	IMX_GPIO_NR(1, 2)
#define SABRESD_CHARGE_DONE	IMX_GPIO_NR(1, 1)
#define SABRESD_ELAN_CE		IMX_GPIO_NR(2, 18)
#define SABRESD_ELAN_RST	IMX_GPIO_NR(3, 8)
#define SABRESD_ELAN_INT	IMX_GPIO_NR(3, 28)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

static struct clk *sata_clk;
static struct clk *clko;
static int mma8451_position = 1;
static int mag3110_position = 2;
static int max11801_mode = 1;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;

static int max17135_regulator_init(struct max17135 *max17135);

static const struct esdhc_platform_data mx6q_sabresd_sd2_data __initconst = {
	.cd_gpio = SABRESD_SD2_CD,
	.wp_gpio = SABRESD_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_sabresd_sd3_data __initconst = {
	.cd_gpio = SABRESD_SD3_CD,
	.wp_gpio = SABRESD_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_sabresd_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabresd_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(0, NULL);
}

static int mx6q_sabresd_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_sabresd_spi_cs[] = {
	SABRESD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_sabresd_spi_data __initconst = {
	.chipselect     = mx6q_sabresd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabresd_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabresd_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_sabresd__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabresd_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabresd_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabresd_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabresd__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabresd_spi_nor_device,
				ARRAY_SIZE(imx6_sabresd_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabresd_audio_wm8958_device = {
	.name = "imx-wm8958",
};

static struct mxc_audio_platform_data wm8958_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = SABRESD_HEADPHONE_DET,
	.hp_active_low = 1,
};

static struct wm8994_pdata wm8958_config_data = {
	.gpio_defaults = {
		[0] = WM8994_GP_FN_GPIO | WM8994_GPN_DB,
		[1] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[2] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[3] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[4] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[5] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[7] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[8] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[9] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
		[10] = WM8994_GP_FN_GPIO | WM8994_GPN_DB | WM8994_GPN_PD,
	},
};

static int mxc_wm8958_init(void)
{
	struct clk *clko;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);

	wm8958_data.sysclk = rate;
	clk_set_rate(clko, rate);

	/* enable wm8958 4.2v power supply */
	gpio_request(SABRESD_CODEC_PWR_EN, "aud_4v2");
	gpio_direction_output(SABRESD_CODEC_PWR_EN, 1);
	msleep(1);
	gpio_set_value(SABRESD_CODEC_PWR_EN, 1);

	return 0;
}

static struct platform_device mx6_sabresd_audio_wm8962_device = {
	.name = "imx-wm8962",
};

static struct mxc_audio_platform_data wm8962_data;

static int wm8962_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int mxc_wm8962_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);

	wm8962_data.sysclk = rate;

	return 0;
}

static struct wm8962_pdata wm8962_config_data = {
	.gpio_init = {
		[2] = WM8962_GPIO_FN_DMICCLK,
		[4] = 0x8000 | WM8962_GPIO_FN_DMICDAT,
	},
};

static struct mxc_audio_platform_data wm8962_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = SABRESD_HEADPHONE_DET,
	.hp_active_low = 1,
	.mic_gpio = SABRESD_MICROPHONE_DET,
	.mic_active_low = 1,
	.init = mxc_wm8962_init,
	.clock_enable = wm8962_clk_enable,
};

static struct regulator_consumer_supply sabresd_vwm8962_consumers[] = {
	REGULATOR_SUPPLY("SPKVDD1", "0-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "0-001a"),
};

static struct regulator_init_data sabresd_vwm8962_init = {
	.constraints = {
		.name = "SPKVDD",
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vwm8962_consumers),
	.consumer_supplies = sabresd_vwm8962_consumers,
};

static struct fixed_voltage_config sabresd_vwm8962_reg_config = {
	.supply_name	= "SPKVDD",
	.microvolts		= 4200000,
	.gpio			= SABRESD_CODEC_PWR_EN,
	.enable_high	= 1,
	.enabled_at_boot = 1,
	.init_data		= &sabresd_vwm8962_init,
};

static struct platform_device sabresd_vwm8962_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id		= 4,
	.dev	= {
		.platform_data = &sabresd_vwm8962_reg_config,
	},
};

static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_CSI0_PWN, 1);
	else
		gpio_set_value(SABRESD_CSI0_PWN, 0);

	msleep(2);
}

static void mx6q_csi0_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_csi0_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_sabresd_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_CSI0_RST, "cam-reset");
	gpio_direction_output(SABRESD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_CSI0_PWN, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 0);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(SABRESD_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(SABRESD_MIPICSI_PWN, 1);
	else
		gpio_set_value(SABRESD_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabresd_mipi_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_sabresd_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_MIPICSI_RST, "cam-reset");
	gpio_direction_output(SABRESD_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(SABRESD_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(SABRESD_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(SABRESD_MIPICSI_PWN, 1);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct regulator_consumer_supply display_consumers[] = {
	{
		/* MAX17135 */
		.supply = "DISPLAY",
	},
};

static struct regulator_consumer_supply vcom_consumers[] = {
	{
		/* MAX17135 */
		.supply = "VCOM",
	},
};

static struct regulator_consumer_supply v3p3_consumers[] = {
	{
		/* MAX17135 */
		.supply = "V3P3",
	},
};

static struct regulator_init_data max17135_init_data[] = {
	{
		.constraints = {
			.name = "DISPLAY",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(display_consumers),
		.consumer_supplies = display_consumers,
	}, {
		.constraints = {
			.name = "GVDD",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "GVEE",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINN",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINP",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "VCOM",
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vcom_consumers),
		.consumer_supplies = vcom_consumers,
	}, {
		.constraints = {
			.name = "VNEG",
			.min_uV = V_to_uV(-15),
			.max_uV = V_to_uV(-15),
		},
	}, {
		.constraints = {
			.name = "VPOS",
			.min_uV = V_to_uV(15),
			.max_uV = V_to_uV(15),
		},
	}, {
		.constraints = {
			.name = "V3P3",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(v3p3_consumers),
		.consumer_supplies = v3p3_consumers,
	},
};

static struct platform_device max17135_sensor_device = {
	.name = "max17135_sensor",
	.id = 0,
};

static struct max17135_platform_data max17135_pdata __initdata = {
	.vneg_pwrup = 1,
	.gvee_pwrup = 1,
	.vpos_pwrup = 2,
	.gvdd_pwrup = 1,
	.gvdd_pwrdn = 1,
	.vpos_pwrdn = 2,
	.gvee_pwrdn = 1,
	.vneg_pwrdn = 1,
	.gpio_pmic_pwrgood = SABRESD_EPDC_PWRSTAT,
	.gpio_pmic_vcom_ctrl = SABRESD_EPDC_VCOM,
	.gpio_pmic_wakeup = SABRESD_EPDC_PMIC_WAKE,
	.gpio_pmic_v3p3 = SABRESD_EPDC_PWRCTRL0,
	.gpio_pmic_intr = SABRESD_EPDC_PMIC_INT,
	.regulator_init = max17135_init_data,
	.init = max17135_regulator_init,
};

static int __init max17135_regulator_init(struct max17135 *max17135)
{
	struct max17135_platform_data *pdata = &max17135_pdata;
	int i, ret;

	if (!epdc_enabled) {
		printk(KERN_DEBUG
			"max17135_regulator_init abort: EPDC not enabled\n");
		return 0;
	}

	max17135->gvee_pwrup = pdata->gvee_pwrup;
	max17135->vneg_pwrup = pdata->vneg_pwrup;
	max17135->vpos_pwrup = pdata->vpos_pwrup;
	max17135->gvdd_pwrup = pdata->gvdd_pwrup;
	max17135->gvdd_pwrdn = pdata->gvdd_pwrdn;
	max17135->vpos_pwrdn = pdata->vpos_pwrdn;
	max17135->vneg_pwrdn = pdata->vneg_pwrdn;
	max17135->gvee_pwrdn = pdata->gvee_pwrdn;

	max17135->max_wait = pdata->vpos_pwrup + pdata->vneg_pwrup +
		pdata->gvdd_pwrup + pdata->gvee_pwrup;

	max17135->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	max17135->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	max17135->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	max17135->gpio_pmic_v3p3 = pdata->gpio_pmic_v3p3;
	max17135->gpio_pmic_intr = pdata->gpio_pmic_intr;

	gpio_request(max17135->gpio_pmic_wakeup, "epdc-pmic-wake");
	gpio_direction_output(max17135->gpio_pmic_wakeup, 0);

	gpio_request(max17135->gpio_pmic_vcom_ctrl, "epdc-vcom");
	gpio_direction_output(max17135->gpio_pmic_vcom_ctrl, 0);

	gpio_request(max17135->gpio_pmic_v3p3, "epdc-v3p3");
	gpio_direction_output(max17135->gpio_pmic_v3p3, 0);

	gpio_request(max17135->gpio_pmic_intr, "epdc-pmic-int");
	gpio_direction_input(max17135->gpio_pmic_intr);

	gpio_request(max17135->gpio_pmic_pwrgood, "epdc-pwrstat");
	gpio_direction_input(max17135->gpio_pmic_pwrgood);

	max17135->vcom_setup = false;
	max17135->init_done = false;

	for (i = 0; i < MAX17135_NUM_REGULATORS; i++) {
		ret = max17135_register_regulator(max17135, i,
			&pdata->regulator_init[i]);
		if (ret != 0) {
			printk(KERN_ERR"max17135 regulator init failed: %d\n",
				ret);
			return ret;
		}
	}

	/*
	 * TODO: We cannot enable full constraints for now, since
	 * it results in the PFUZE regulators being disabled
	 * at the end of boot, which disables critical regulators.
	 */
	/*regulator_has_full_constraints();*/

	return 0;
}

static struct imxi2c_platform_data mx6q_sabresd_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 499,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm89**", 0x1a),
	},
	{
		I2C_BOARD_INFO("ov564x", 0x3c),
		.platform_data = (void *)&camera_data,
	},
	{
		I2C_BOARD_INFO("mma8451", 0x1c),
		.platform_data = (void *)&mma8451_position,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(SABRESD_CAP_TCH_INT0),
	},
	{
		I2C_BOARD_INFO("max11801", 0x48),
		.platform_data = (void *)&max11801_mode,
		.irq = gpio_to_irq(SABRESD_TS_INT),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max17135", 0x48),
		.platform_data = &max17135_pdata,
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(SABRESD_CAP_TCH_INT1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.irq = gpio_to_irq(SABRESD_eCOMPASS_INT),
		.platform_data = (void *)&mag3110_position,
	},
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq  = gpio_to_irq(SABRESD_ALS_INT),
		.platform_data = &ls_data,
	}, {
		I2C_BOARD_INFO("elan-touch", 0x10),
		.irq = gpio_to_irq(SABRESD_ELAN_INT),
	},
};

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(SABRESD_EPDC_SDDO_0, "epdc_d0");
	ret |= gpio_request(SABRESD_EPDC_SDDO_1, "epdc_d1");
	ret |= gpio_request(SABRESD_EPDC_SDDO_2, "epdc_d2");
	ret |= gpio_request(SABRESD_EPDC_SDDO_3, "epdc_d3");
	ret |= gpio_request(SABRESD_EPDC_SDDO_4, "epdc_d4");
	ret |= gpio_request(SABRESD_EPDC_SDDO_5, "epdc_d5");
	ret |= gpio_request(SABRESD_EPDC_SDDO_6, "epdc_d6");
	ret |= gpio_request(SABRESD_EPDC_SDDO_7, "epdc_d7");
	ret |= gpio_request(SABRESD_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(SABRESD_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(SABRESD_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(SABRESD_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(SABRESD_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(SABRESD_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(SABRESD_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(SABRESD_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(SABRESD_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(SABRESD_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(SABRESD_EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(SABRESD_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(SABRESD_EPDC_SDDO_0);
	gpio_free(SABRESD_EPDC_SDDO_1);
	gpio_free(SABRESD_EPDC_SDDO_2);
	gpio_free(SABRESD_EPDC_SDDO_3);
	gpio_free(SABRESD_EPDC_SDDO_4);
	gpio_free(SABRESD_EPDC_SDDO_5);
	gpio_free(SABRESD_EPDC_SDDO_6);
	gpio_free(SABRESD_EPDC_SDDO_7);
	gpio_free(SABRESD_EPDC_GDCLK);
	gpio_free(SABRESD_EPDC_GDSP);
	gpio_free(SABRESD_EPDC_GDOE);
	gpio_free(SABRESD_EPDC_GDRL);
	gpio_free(SABRESD_EPDC_SDCLK);
	gpio_free(SABRESD_EPDC_SDOE);
	gpio_free(SABRESD_EPDC_SDLE);
	gpio_free(SABRESD_EPDC_SDSHR);
	gpio_free(SABRESD_EPDC_BDR0);
	gpio_free(SABRESD_EPDC_SDCE0);
	gpio_free(SABRESD_EPDC_SDCE1);
	gpio_free(SABRESD_EPDC_SDCE2);
}

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_epdc_enable_pads, \
				ARRAY_SIZE(mx6dl_sabresd_epdc_enable_pads));

	gpio_direction_input(SABRESD_EPDC_SDDO_0);
	gpio_direction_input(SABRESD_EPDC_SDDO_1);
	gpio_direction_input(SABRESD_EPDC_SDDO_2);
	gpio_direction_input(SABRESD_EPDC_SDDO_3);
	gpio_direction_input(SABRESD_EPDC_SDDO_4);
	gpio_direction_input(SABRESD_EPDC_SDDO_5);
	gpio_direction_input(SABRESD_EPDC_SDDO_6);
	gpio_direction_input(SABRESD_EPDC_SDDO_7);
	gpio_direction_input(SABRESD_EPDC_GDCLK);
	gpio_direction_input(SABRESD_EPDC_GDSP);
	gpio_direction_input(SABRESD_EPDC_GDOE);
	gpio_direction_input(SABRESD_EPDC_GDRL);
	gpio_direction_input(SABRESD_EPDC_SDCLK);
	gpio_direction_input(SABRESD_EPDC_SDOE);
	gpio_direction_input(SABRESD_EPDC_SDLE);
	gpio_direction_input(SABRESD_EPDC_SDSHR);
	gpio_direction_input(SABRESD_EPDC_BDR0);
	gpio_direction_input(SABRESD_EPDC_SDCE0);
	gpio_direction_input(SABRESD_EPDC_SDCE1);
	gpio_direction_input(SABRESD_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_epdc_disable_pads, \
				ARRAY_SIZE(mx6dl_sabresd_epdc_disable_pads));

	gpio_direction_output(SABRESD_EPDC_SDDO_0, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_1, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_2, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_3, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_4, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_5, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_6, 0);
	gpio_direction_output(SABRESD_EPDC_SDDO_7, 0);
	gpio_direction_output(SABRESD_EPDC_GDCLK, 0);
	gpio_direction_output(SABRESD_EPDC_GDSP, 0);
	gpio_direction_output(SABRESD_EPDC_GDOE, 0);
	gpio_direction_output(SABRESD_EPDC_GDRL, 0);
	gpio_direction_output(SABRESD_EPDC_SDCLK, 0);
	gpio_direction_output(SABRESD_EPDC_SDOE, 0);
	gpio_direction_output(SABRESD_EPDC_SDLE, 0);
	gpio_direction_output(SABRESD_EPDC_SDSHR, 0);
	gpio_direction_output(SABRESD_EPDC_BDR0, 0);
	gpio_direction_output(SABRESD_EPDC_SDCE0, 0);
	gpio_direction_output(SABRESD_EPDC_SDCE1, 0);
	gpio_direction_output(SABRESD_EPDC_SDCE2, 0);
}

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	.refresh = 85,
	.xres = 800,
	.yres = 600,
};
static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	}
};

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};

static void imx6q_sabresd_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_OTG_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_OTG_PWR, 0);
}

static void imx6q_sabresd_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_H1_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_H1_PWR, 0);
}

static void __init imx6q_sabresd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SABRESD_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SABRESD_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_H1_PWR, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sabresd_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabresd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

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

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabresd_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabresd_sata_data = {
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};
#endif

static void mx6q_sabresd_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(SABRESD_CAN1_STBY, 1);
	} else {
		gpio_set_value(SABRESD_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_sabresd_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_sabresd_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(SABRESD_DISP_PWR_EN, 1);
	gpio_set_value(SABRESD_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(SABRESD_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(SABRESD_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sabresd_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
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
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabresd board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_sabresd_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_sabresd_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6dl_sabresd_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6q_sabresd_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct max8903_pdata charger1_data = {
	.dok = SABRESD_CHARGE_DOK_B,
	.uok = SABRESD_CHARGE_UOK_B,
	.chg = SABRESD_CHARGE_CHG_1_B,
	.flt = SABRESD_CHARGE_FLT_1_B,
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = true,
};

static struct platform_device sabresd_max8903_charger_1 = {
	.name	= "max8903-charger",
	.id	= 1,
	.dev	= {
		.platform_data = &charger1_data,
	},
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabresd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	if (board_is_mx6_reva()) {
		mxc_register_device(&mx6_sabresd_audio_wm8958_device,
				    &wm8958_data);
		imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

		mxc_wm8958_init();
	} else {
		platform_device_register(&sabresd_vwm8962_reg_devices);
		mxc_register_device(&mx6_sabresd_audio_wm8962_device,
				    &wm8962_data);
		imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

		mxc_wm8962_init();
	}

	return 0;
}

#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
	/* disable PCIE_3V3 first */
	gpio_request(SABRESD_PCIE_PWR_EN, "pcie_3v3_en");
	gpio_direction_output(SABRESD_PCIE_PWR_EN, 0);
	mdelay(10);
	/* enable PCIE_3V3 again */
	gpio_set_value(SABRESD_PCIE_PWR_EN, 1);
	gpio_free(SABRESD_PCIE_PWR_EN);
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(SABRESD_PCIE_RST_B_REVB, "pcie_reset_rebB");
	gpio_direction_output(SABRESD_PCIE_RST_B_REVB, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(SABRESD_PCIE_RST_B_REVB, 1);
	gpio_free(SABRESD_PCIE_RST_B_REVB);
}
#endif

static void gps_power_on(bool on)
{
	/* Enable/disable aux_3v15 */
	gpio_request(SABRESD_AUX_3V15_EN, "aux_3v15_en");
	gpio_direction_output(SABRESD_AUX_3V15_EN, 1);
	gpio_set_value(SABRESD_AUX_3V15_EN, on);
	gpio_free(SABRESD_AUX_3V15_EN);
	/*Enable/disable gps_en*/
	gpio_request(SABRESD_GPS_EN, "gps_en");
	gpio_direction_output(SABRESD_GPS_EN, 1);
	gpio_set_value(SABRESD_GPS_EN, on);
	gpio_free(SABRESD_GPS_EN);

}

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= "max8903-"trigger,		\
}

/* use to show a external power source is connected
 * GPIO_LED(SABRESD_CHARGE_DONE, "chg_detect", 0, 1, "ac-online"),
 */
static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(SABRESD_CHARGE_NOW, "chg_now_led", 0, 1,
		"charger-charging"),
/* For the latest B4 board, this GPIO_1 is connected to POR_B,
which will reset the whole board if this pin's level is changed,
so, for the latest board, we have to avoid using this pin as
GPIO.
	GPIO_LED(SABRESD_CHARGE_DONE, "chg_done_led", 0, 1,
			"charger-full"),
*/
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
	GPIO_BUTTON(SABRESD_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(SABRESD_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	GPIO_BUTTON(SABRESD_POWER_OFF, KEY_POWER, 1, "power", 1, 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabresd_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_sabresd_pcie_data __initconst = {
	.pcie_pwr_en	= SABRESD_PCIE_PWR_EN,
	.pcie_rst	= SABRESD_PCIE_RST_B_REVB,
	.pcie_wake_up	= SABRESD_PCIE_WAKE_B,
	.pcie_dis	= SABRESD_PCIE_DIS_B,
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
};

/*!
 * Board specific initialization.
 */
static void __init mx6_sabresd_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_pads,
			ARRAY_SIZE(mx6q_sabresd_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_pads,
			ARRAY_SIZE(mx6dl_sabresd_pads));
	}

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = sabresd_dvfscore_data.reg_id;
	soc_reg_id = sabresd_dvfscore_data.soc_id;
	mx6q_sabresd_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	if (board_is_mx6_reva()) {
		strcpy(mxc_i2c0_board_info[0].type, "wm8958");
		mxc_i2c0_board_info[0].platform_data = &wm8958_config_data;
	} else {
		strcpy(mxc_i2c0_board_info[0].type, "wm8962");
		mxc_i2c0_board_info[0].platform_data = &wm8962_config_data;
	}
	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6q_sabresd_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(SABRESD_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(SABRESD_PFUZE_INT);
		mx6q_sabresd_init_pfuze100(SABRESD_PFUZE_INT);
	}
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabresd_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabresd_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabresd_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabresd_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
#else
		mx6q_sabresd_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabresd_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(SABRESD_CABC_EN0, "cabc-en0");
	gpio_direction_output(SABRESD_CABC_EN0, 0);
	gpio_request(SABRESD_CABC_EN1, "cabc-en1");
	gpio_direction_output(SABRESD_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabresd_dvfscore_data);
	imx6q_add_device_buttons();

	/* enable sensor 3v3 and 1v8 */
	gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable ecompass intr */
	gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(SABRESD_ALS_INT, "als-int");
	gpio_direction_input(SABRESD_ALS_INT);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
		if (epdc_enabled) {
			mxc_register_device(&max17135_sensor_device, NULL);
			imx6dl_add_imx_epdc(&epdc_data);
		}
	}
	/*
	ret = gpio_request_array(mx6q_sabresd_flexcan_gpios,
			ARRAY_SIZE(mx6q_sabresd_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_sabresd_flexcan0_pdata);
	*/

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Enable Aux_5V */
	gpio_request(SABRESD_AUX_5V_EN, "aux_5v_en");
	gpio_direction_output(SABRESD_AUX_5V_EN, 1);
	gpio_set_value(SABRESD_AUX_5V_EN, 1);

#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_power();
	mdelay(10);
	pcie_3v3_reset();
#endif

	gps_power_on(true);
	/* Register charger chips */
	platform_device_register(&sabresd_max8903_charger_1);
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_sabresd_pcie_data);
	if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_elan_pads,
						ARRAY_SIZE(mx6dl_arm2_elan_pads));

		/* ELAN Touchscreen */
		gpio_request(SABRESD_ELAN_INT, "elan-interrupt");
		gpio_direction_input(SABRESD_ELAN_INT);

		gpio_request(SABRESD_ELAN_CE, "elan-cs");
		gpio_direction_output(SABRESD_ELAN_CE, 1);
		gpio_direction_output(SABRESD_ELAN_CE, 0);

		gpio_request(SABRESD_ELAN_RST, "elan-rst");
		gpio_direction_output(SABRESD_ELAN_RST, 1);
		gpio_direction_output(SABRESD_ELAN_RST, 0);
		mdelay(1);
		gpio_direction_output(SABRESD_ELAN_RST, 1);
		gpio_direction_output(SABRESD_ELAN_CE, 1);
	}

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_sabresd_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabresd_timer = {
	.init   = mx6_sabresd_timer_init,
};

static void __init mx6q_sabresd_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SABRESD, "Freescale i.MX 6Quad/DualLite/Solo Sabre-SD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabresd_board_init,
	.timer = &mx6_sabresd_timer,
	.reserve = mx6q_sabresd_reserve,
MACHINE_END
