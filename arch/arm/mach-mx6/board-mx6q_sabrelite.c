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
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#define MX6Q_SABRELITE_SD3_CD		IMX_GPIO_NR(7, 0)
#define MX6Q_SABRELITE_SD3_WP		IMX_GPIO_NR(7, 1)
#define MX6Q_SABRELITE_SD4_CD		IMX_GPIO_NR(2, 6)
#define MX6Q_SABRELITE_SD4_WP		IMX_GPIO_NR(2, 7)
#define MX6Q_SABRELITE_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX6Q_SABRELITE_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define MX6Q_SABRELITE_CAP_TCH_INT1	IMX_GPIO_NR(1, 9)
#define MX6Q_SABRELITE_USB_HUB_RESET	IMX_GPIO_NR(7, 12)

#define MX6Q_SABRELITE_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

void __init early_console_setup(unsigned long base, struct clk *clk);

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern int (*set_cpu_voltage)(u32 volt);
extern int mx6_set_cpu_voltage(u32 cpu_volt);
static struct regulator *cpu_regulator;
static char *gp_reg_id;

static iomux_v3_cfg_t mx6q_sabrelite_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_SD2_DAT0__AUDMUX_AUD4_RXD,
	MX6Q_PAD_SD2_DAT3__AUDMUX_AUD4_TXC,
	MX6Q_PAD_SD2_DAT2__AUDMUX_AUD4_TXD,
	MX6Q_PAD_SD2_DAT1__AUDMUX_AUD4_TXFS,

	/* CAN1  */
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25,	/* STNDBY */
	MX6Q_PAD_ENET_RXD1__GPIO_1_26,		/* NERR */
	MX6Q_PAD_ENET_RXD0__GPIO_1_27,		/* Enable */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
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
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */

	/* GPIO1 */
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,		/* J9 - Microphone Detect */

	/* GPIO2 */
	MX6Q_PAD_NANDF_D1__GPIO_2_1,	/* J14 - Menu Button */
	MX6Q_PAD_NANDF_D2__GPIO_2_2,	/* J14 - Back Button */
	MX6Q_PAD_NANDF_D3__GPIO_2_3,	/* J14 - Search Button */
	MX6Q_PAD_NANDF_D4__GPIO_2_4,	/* J14 - Home Button */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* GPIO4 */
	MX6Q_PAD_GPIO_19__GPIO_4_5,	/* J14 - Volume Down */

	/* GPIO5 */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A24__GPIO_5_4,	/* J12 - Boot Mode Select */

	/* GPIO6 */
	MX6Q_PAD_EIM_A23__GPIO_6_6,	/* J12 - Boot Mode Select */

	/* GPIO7 */
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* J14 - Volume Up */

	/* I2C1, SGTL5000 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 Camera, MIPI */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* GPIO4[13] */

	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,	/* GPIO1[5] - J7 - Display card */
	MX6Q_PAD_GPIO_16__I2C3_SDA,	/* GPIO7[11] - J15 - RGB connector */

	/* IPU1 Camera */
	MX6Q_PAD_CSI0_DAT8__IPU1_CSI0_D_8,
	MX6Q_PAD_CSI0_DAT9__IPU1_CSI0_D_9,
	MX6Q_PAD_CSI0_DAT10__IPU1_CSI0_D_10,
	MX6Q_PAD_CSI0_DAT11__IPU1_CSI0_D_11,
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_GPIO_6__GPIO_1_6,		/* J5 - Camera GP */
	MX6Q_PAD_GPIO_8__GPIO_1_8,		/* J5 - Camera Reset */
	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/* J5 - Camera GP */
	MX6Q_PAD_NANDF_D5__GPIO_2_5,		/* J16 - MIPI GP */
	MX6Q_PAD_NANDF_WP_B__GPIO_6_9,		/* J16 - MIPI GP */

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
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
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */


	/* PWM1 */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* PWM2 */
	MX6Q_PAD_SD1_DAT2__PWM2_PWMO,		/* GPIO1[19] */

	/* PWM3 */
	MX6Q_PAD_SD1_DAT1__PWM3_PWMO,		/* GPIO1[17] */

	/* PWM4 */
	MX6Q_PAD_SD1_CMD__PWM4_PWMO,		/* GPIO1[18] */

	/* UART1  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2 for debug */
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT5__GPIO_7_0,		/* J18 - SD3_CD */
	NEW_PAD_CTRL(MX6Q_PAD_SD3_DAT4__GPIO_7_1, MX6Q_SABRELITE_SD3_WP_PADCFG),

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_NANDF_D6__GPIO_2_6,		/* J20 - SD4_CD */
	MX6Q_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd3_pad_change(int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_200mhz,
					ARRAY_SIZE(mx6q_sd3_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_100mhz,
					ARRAY_SIZE(mx6q_sd3_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd3_50mhz,
					ARRAY_SIZE(mx6q_sd3_50mhz));
	}
}

static int plt_sd4_pad_change(int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_200mhz,
					ARRAY_SIZE(mx6q_sd4_200mhz));
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_100mhz,
					ARRAY_SIZE(mx6q_sd4_100mhz));
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(mx6q_sd4_50mhz,
					ARRAY_SIZE(mx6q_sd4_50mhz));
	}
}

static const struct esdhc_platform_data mx6q_sabrelite_sd3_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD3_CD,
	.wp_gpio = MX6Q_SABRELITE_SD3_WP,
	.platform_pad_change = plt_sd3_pad_change,
};

static const struct esdhc_platform_data mx6q_sabrelite_sd4_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD4_CD,
	.wp_gpio = MX6Q_SABRELITE_SD4_WP,
	.platform_pad_change = plt_sd4_pad_change,
};

static const struct anatop_thermal_platform_data
	mx6q_sabrelite_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabrelite_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
}

static int mx6q_sabrelite_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x0f00);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabrelite_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_sabrelite_spi_cs[] = {
	MX6Q_SABRELITE_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabrelite_spi_data __initconst = {
	.chipselect     = mx6q_sabrelite_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabrelite_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabrelite_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00040000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_sabrelite__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabrelite_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabrelite_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabrelite_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabrelite__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabrelite_spi_nor_device,
				ARRAY_SIZE(imx6_sabrelite_spi_nor_device));
}

static struct mxc_audio_platform_data mx6_sabrelite_audio_data;

static int mx6_sabrelite_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_sabrelite_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_sabrelite_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_sabrelite_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_sabrelite_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_sabrelite_audio_device = {
	.name = "imx-sgtl5000",
};

static struct imxi2c_platform_data mx6q_sabrelite_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_SABRELITE_CAP_TCH_INT1),
	},
};

static void imx6q_sabrelite_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 0);
}

static void __init imx6q_sabrelite_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_SABRELITE_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_SABRELITE_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_SABRELITE_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabrelite_usbotg_vbus);
	mx6_usb_dr_init();
	mx6_usb_h1_init();
}
static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct ipuv3_fb_platform_data sabrelite_fb_data[] = {
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
	.mode_str = "LDB-SVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
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
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
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
	.disp_id = 0,
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

static void sabrelite_suspend_enter(void)
{
	/* suspend preparation */
}

static void sabrelite_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_sabrelite_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabrelite_suspend_enter,
	.suspend_exit = sabrelite_suspend_exit,
};

static struct regulator_consumer_supply sabrelite_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabrelite_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabrelite_vmmc_consumers),
	.consumer_supplies = sabrelite_vmmc_consumers,
};

static struct fixed_voltage_config sabrelite_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabrelite_vmmc_init,
};

static struct platform_device sabrelite_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabrelite_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 2500000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sabrelite_audio_device,
			    &mx6_sabrelite_audio_data);
	imx6q_add_imx_ssi(1, &mx6_sabrelite_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif
	return 0;
}

static struct platform_pwm_backlight_data mx6_sabrelite_pwm_backlight_data = {
	.pwm_id = 3,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabrelite_dvfscore_data = {
	.reg_id = "cpu_vddgp",
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

static int mx6_sabre_set_cpu_voltage(u32 cpu_volt)
{
	int ret = -EINVAL;

	if (cpu_regulator == NULL)
		cpu_regulator = regulator_get(NULL, gp_reg_id);

	if (!IS_ERR(cpu_regulator))
		ret = regulator_set_voltage(cpu_regulator,
						    cpu_volt, cpu_volt);
	return ret;
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	set_cpu_voltage = mx6_sabre_set_cpu_voltage;
}

/*!
 * Board specific initialization.
 */
static void __init mx6_sabrelite_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_pads,
					ARRAY_SIZE(mx6q_sabrelite_pads));

	mx6q_sabrelite_init_uart();
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(sabrelite_fb_data); i++)
		imx6q_add_ipuv3fb(i, &sabrelite_fb_data[i]);

	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_i2c(0, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabrelite_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabrelite_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabrelite_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_sabrelite_pm_data);
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabrelite_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabrelite_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabrelite_init_usb();
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabrelite_vmmc_reg_devices);
	/* release USB Hub reset */
	gpio_set_value(MX6Q_SABRELITE_USB_HUB_RESET, 1);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(3, &mx6_sabrelite_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabrelite_dvfscore_data);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
}

extern void __iomem *twd_base;
static void __init mx6_sabrelite_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabrelite_timer = {
	.init   = mx6_sabrelite_timer_init,
};

static void __init mx6q_sabrelite_reserve(void)
{
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABRELITE data structure.
 */
MACHINE_START(MX6Q_SABRELITE, "Freescale i.MX 6Quad Sabre-Lite Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabrelite_board_init,
	.timer = &mx6_sabrelite_timer,
	.reserve = mx6q_sabrelite_reserve,
MACHINE_END
