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
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mipi_csi2.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

#define MX6Q_SABREAUTO_LDB_BACKLIGHT	IMX_GPIO_NR(1, 9)
#define MX6Q_SABREAUTO_ECSPI1_CS0	IMX_GPIO_NR(2, 30)
#define MX6Q_SABREAUTO_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX6Q_SABREAUTO_DISP0_PWR	IMX_GPIO_NR(3, 24)
#define MX6Q_SABREAUTO_DISP0_I2C_EN	IMX_GPIO_NR(3, 28)
#define MX6Q_SABREAUTO_DISP0_DET_INT	IMX_GPIO_NR(3, 31)
#define MX6Q_SABREAUTO_DISP0_RESET	IMX_GPIO_NR(5, 0)
#define MX6Q_SABREAUTO_SD3_CD	IMX_GPIO_NR(6, 15)
#define MX6Q_SABREAUTO_SD3_WP	IMX_GPIO_NR(1, 13)
#define MX6Q_SABREAUTO_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6Q_SABREAUTO_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)
#define MX6Q_SABREAUTO_CAP_TCH_INT	IMX_GPIO_NR(3, 31)

#define MX6Q_SABREAUTO_IO_EXP_GPIO1(x) \
		(MX6Q_SABREAUTO_MAX7310_1_BASE_ADDR + (x))
#define MX6Q_SABREAUTO_IO_EXP_GPIO2(x) \
		(MX6Q_SABREAUTO_MAX7310_2_BASE_ADDR + (x))

#define MX6Q_SABREAUTO_CAN1_STBY       IMX_GPIO_NR(7, 12)
#define MX6Q_SABREAUTO_CAN1_EN         IMX_GPIO_NR(7, 13)
#define MX6Q_SABREAUTO_CAN2_STBY       MX6Q_SABREAUTO_IO_EXP_GPIO2(1)
#define MX6Q_SABREAUTO_CAN2_EN         IMX_GPIO_NR(5, 24)

#define MX6Q_SMD_CSI0_RST		IMX_GPIO_NR(4, 5)
#define MX6Q_SMD_CSI0_PWN		IMX_GPIO_NR(5, 23)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;
static int esai_record;
static int spdif_en;
static int mipi_sensor;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern int (*set_cpu_voltage)(u32 volt);
extern int mx6_set_cpu_voltage(u32 cpu_volt);
static struct regulator *cpu_regulator;
static char *gp_reg_id;

static iomux_v3_cfg_t mx6q_sabreauto_pads[] = {

	/* UART4 for debug */
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,
	/* USB HSIC ports use the same pin with ENET */
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	/* USB H2 strobe/data pin */
	MX6Q_PAD_RGMII_TX_CTL__USBOH3_H2_STROBE,
	MX6Q_PAD_RGMII_TXC__USBOH3_H2_DATA,

	/* USB H3 strobe/data pin */
	MX6Q_PAD_RGMII_RXC__USBOH3_H3_STROBE,
	MX6Q_PAD_RGMII_RX_CTL__USBOH3_H3_DATA,
	/* ENET */
#else
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
#endif
	/* MCLK for csi0 */
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
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT4__USDHC3_DAT4_50MHZ,
	MX6Q_PAD_SD3_DAT5__USDHC3_DAT5_50MHZ,
	MX6Q_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ,
	MX6Q_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ,
	/* MX6Q_PAD_SD3_RST__USDHC3_RST, */
	/* SD3 VSelect */
	MX6Q_PAD_GPIO_18__USDHC3_VSELECT,
	/* SD3_CD and SD3_WP */
	MX6Q_PAD_NANDF_CS2__GPIO_6_15,
	MX6Q_PAD_SD2_DAT2__GPIO_1_13,
	/* SD4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,
	MX6Q_PAD_NANDF_ALE__USDHC4_RST,
	/* eCSPI1 */
	MX6Q_PAD_EIM_EB2__ECSPI1_SS0,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D19__ECSPI1_SS1,
	MX6Q_PAD_EIM_EB2__GPIO_2_30,	/*SS0*/
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

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

	/* ipu1 csi0 */
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	/* camera reset */
	MX6Q_PAD_GPIO_19__GPIO_4_5,
	/* camera powerdown */
	MX6Q_PAD_CSI0_DAT5__GPIO_5_23,

	MX6Q_PAD_EIM_D24__GPIO_3_24,

	/* PWM1 */
	MX6Q_PAD_GPIO_9__PWM1_PWMO,

	/* DISP0 I2C ENABLE*/
	MX6Q_PAD_EIM_D28__GPIO_3_28,

	/* DISP0 DET */
	MX6Q_PAD_EIM_D31__GPIO_3_31,

	/* DISP0 RESET */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,

	/* HDMI */
	MX6Q_PAD_EIM_A25__HDMI_TX_CEC_LINE,
	MX6Q_PAD_SD1_DAT1__HDMI_TX_OPHYDTB_0,
	MX6Q_PAD_SD1_DAT0__HDMI_TX_OPHYDTB_1,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,
};

static iomux_v3_cfg_t mx6q_sabreauto_i2c3_pads[] = {
	MX6Q_PAD_GPIO_5__I2C3_SCL,
	MX6Q_PAD_GPIO_16__I2C3_SDA,
};

static iomux_v3_cfg_t mx6q_sabreauto_spdif_pads[] = {
	/* SPDIF */
	MX6Q_PAD_GPIO_16__SPDIF_IN1,
	MX6Q_PAD_GPIO_17__SPDIF_OUT1,
};

static iomux_v3_cfg_t mx6q_sabreauto_can_pads[] = {
	/* CAN1 */
	MX6Q_PAD_GPIO_7__CAN1_TXCAN,
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* CAN1 STBY */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* CAN1 EN */

	/* CAN2 */
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,
	MX6Q_PAD_CSI0_DAT6__GPIO_5_24,	/* CAN2 EN */
};

static iomux_v3_cfg_t mx6q_sabreauto_esai_record_pads[] = {
	MX6Q_PAD_ENET_RX_ER__ESAI1_HCKR,
	MX6Q_PAD_ENET_MDIO__ESAI1_SCKR,
	MX6Q_PAD_ENET_REF_CLK__ESAI1_FSR,
};

static iomux_v3_cfg_t mx6q_sabreauto_mipi_sensor_pads[] = {
	MX6Q_PAD_CSI0_MCLK__CCM_CLKO,
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT4__USDHC##id##_DAT4_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT5__USDHC##id##_DAT5_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT6__USDHC##id##_DAT6_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT7__USDHC##id##_DAT7_##speed##MHZ,	\
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

static const struct esdhc_platform_data mx6q_sabreauto_sd3_data __initconst = {
	.cd_gpio = MX6Q_SABREAUTO_SD3_CD,
	.wp_gpio = MX6Q_SABREAUTO_SD3_WP,
	.support_18v = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.platform_pad_change = plt_sd3_pad_change,
};

/* No card detect signal for SD4 */
static const struct esdhc_platform_data mx6q_sabreauto_sd4_data __initconst = {
	.always_present = 1,
	.support_8bit = 1,
	.platform_pad_change = plt_sd4_pad_change,
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

static const struct anatop_thermal_platform_data
	mx6q_sabreauto_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static inline void mx6q_sabreauto_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(3, NULL);
}

static int mx6q_sabreauto_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static int mx6q_sabreauto_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set AR8031 debug reg 0xb to hibernate power*/
	phy_write(phydev, 0x1d, 0xb);
	val = phy_read(phydev, 0x1e);

	val |= 0x8000;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabreauto_fec_phy_init,
	.power_hibernate = mx6q_sabreauto_fec_power_hibernate,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_sabreauto_spi_cs[] = {
	MX6Q_SABREAUTO_ECSPI1_CS0,
	MX6Q_SABREAUTO_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabreauto_spi_data __initconst = {
	.chipselect     = mx6q_sabreauto_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabreauto_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition m25p32_partitions[] = {
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

static struct flash_platform_data m25p32_spi_flash_data = {
	.name = "m25p32",
	.parts = m25p32_partitions,
	.nr_parts = ARRAY_SIZE(m25p32_partitions),
	.type = "m25p32",
};
#endif

static struct spi_board_info m25p32_spi0_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		/* The modalias must be the same as spi device driver name */
		.modalias = "m25p80",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 1,
		.platform_data = &m25p32_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi0_board_info,
				ARRAY_SIZE(m25p32_spi0_board_info));
}

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
		1, 1, 1, 1, 0, 0, 0, 0,
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

static struct fsl_mxc_camera_platform_data camera_data = {
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
	.mclk = 24000000,
	.csi = 0,
};

static struct fsl_mxc_camera_platform_data ov5640_mipi_data = {
	.mclk = 24000000,
	.csi = 0,
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("cs42888", 0x48),
	},
	{
		I2C_BOARD_INFO("ov3640", 0x3c),
		.platform_data = (void *)&camera_data,
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
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&ov5640_mipi_data,
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
	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6Q_SABREAUTO_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO MX6Q_SABREAUTO_USB_OTG_PWR:"
			" %d\n", ret);
		return;
	}
	gpio_direction_output(MX6Q_SABREAUTO_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabreauto_usbotg_vbus);
	mx6_usb_dr_init();
	mx6_usb_h1_init();
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
	mx6_usb_h3_init();
#endif
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

static void mx6q_sabreauto_reset_mipi_dsi(void)
{
	gpio_set_value(MX6Q_SABREAUTO_DISP0_PWR, 1);
	gpio_set_value(MX6Q_SABREAUTO_DISP0_RESET, 1);
	udelay(10);
	gpio_set_value(MX6Q_SABREAUTO_DISP0_RESET, 0);
	udelay(50);
	gpio_set_value(MX6Q_SABREAUTO_DISP0_RESET, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id	 = 0,
	.disp_id = 0,
	.lcd_panel = "TRULY-WVGA",
	.reset   = mx6q_sabreauto_reset_mipi_dsi,
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

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		printk(KERN_ERR"Invalid IPU select for HDMI: %d. Set to 0\n",
			ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		printk(KERN_ERR"Invalid DI select for HDMI: %d. Set to 0\n",
			disp_id);
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
	.csi_clk[0] = "ccm_clk0",
	}, {
	.rev = 4,
	.csi_clk[0] = "ccm_clk0",
	},
};

static struct platform_pwm_backlight_data mx6_sabreauto_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct gpio mx6q_flexcan_gpios[] = {
	{ MX6Q_SABREAUTO_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ MX6Q_SABREAUTO_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
	{ MX6Q_SABREAUTO_CAN2_EN, GPIOF_OUT_INIT_LOW, "flexcan2-en" },
};

static void mx6q_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6Q_SABREAUTO_CAN1_EN, 1);
		gpio_set_value(MX6Q_SABREAUTO_CAN1_STBY, 1);
	} else {
		gpio_set_value(MX6Q_SABREAUTO_CAN1_EN, 0);
		gpio_set_value(MX6Q_SABREAUTO_CAN1_STBY, 0);
	}
}

static void mx6q_flexcan1_switch(int enable)
{
	if (enable) {
		gpio_set_value(MX6Q_SABREAUTO_CAN2_EN, 1);
		gpio_set_value(MX6Q_SABREAUTO_CAN2_STBY, 1);
	} else {
		gpio_set_value(MX6Q_SABREAUTO_CAN2_EN, 0);
		gpio_set_value(MX6Q_SABREAUTO_CAN2_STBY, 0);
	}
}

static const struct flexcan_platform_data
		mx6q_sabreauto_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6q_flexcan0_switch,
	}, {
		.transceiver_switch = mx6q_flexcan1_switch,
	}
};

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
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

static struct regulator_consumer_supply sabreauto_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabreauto_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabreauto_vmmc_consumers),
	.consumer_supplies = sabreauto_vmmc_consumers,
};

static struct fixed_voltage_config sabreauto_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabreauto_vmmc_init,
};

static struct platform_device sabreauto_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sabreauto_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_CS42888

static struct regulator_consumer_supply cs42888_sabreauto_consumer_va = {
	.supply = "VA",
	.dev_name = "0-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vd = {
	.supply = "VD",
	.dev_name = "0-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vls = {
	.supply = "VLS",
	.dev_name = "0-0048",
};

static struct regulator_consumer_supply cs42888_sabreauto_consumer_vlc = {
	.supply = "VLC",
	.dev_name = "0-0048",
};

static struct regulator_init_data cs42888_sabreauto_va_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &cs42888_sabreauto_consumer_va,
};

static struct regulator_init_data cs42888_sabreauto_vd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &cs42888_sabreauto_consumer_vd,
};

static struct regulator_init_data cs42888_sabreauto_vls_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &cs42888_sabreauto_consumer_vls,
};

static struct regulator_init_data cs42888_sabreauto_vlc_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &cs42888_sabreauto_consumer_vlc,
};

static struct fixed_voltage_config cs42888_sabreauto_va_reg_config = {
	.supply_name		= "VA",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_va_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vd_reg_config = {
	.supply_name		= "VD",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vd_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vls_reg_config = {
	.supply_name		= "VLS",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vls_reg_initdata,
};

static struct fixed_voltage_config cs42888_sabreauto_vlc_reg_config = {
	.supply_name		= "VLC",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &cs42888_sabreauto_vlc_reg_initdata,
};

static struct platform_device cs42888_sabreauto_va_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &cs42888_sabreauto_va_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vd_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vls_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 5,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vls_reg_config,
	},
};

static struct platform_device cs42888_sabreauto_vlc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 6,
	.dev	= {
		.platform_data = &cs42888_sabreauto_vlc_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_CS42888 */

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabreauto_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabreauto_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabreauto_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabreauto_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabreauto_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabreauto_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabreauto_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabreauto_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 7,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabreauto_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 8,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabreauto_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 9,
	.dev	= {
		.platform_data = &sgtl5000_sabreauto_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int __init imx6q_init_audio(void)
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

#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabreauto_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabreauto_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabreauto_vddd_reg_devices);
#endif

#ifdef CONFIG_SND_SOC_CS42888
	platform_device_register(&cs42888_sabreauto_va_reg_devices);
	platform_device_register(&cs42888_sabreauto_vd_reg_devices);
	platform_device_register(&cs42888_sabreauto_vls_reg_devices);
	platform_device_register(&cs42888_sabreauto_vlc_reg_devices);
#endif
	return 0;
}

static int __init early_use_esai_record(char *p)
{
	esai_record = 1;
	return 0;
}

early_param("esai_record", early_use_esai_record);

static struct mxc_dvfs_platform_data sabreauto_dvfscore_data = {
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

static int mx6_sabreauto_set_cpu_voltage(u32 cpu_volt)
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
	set_cpu_voltage = mx6_sabreauto_set_cpu_voltage;
}

static int __init early_enable_mipi_sensor(char *p)
{
	mipi_sensor = 1;
	return 0;
}
early_param("mipi_sensor", early_enable_mipi_sensor);

static inline void __init mx6q_csi0_io_init(void)
{
	/* Camera reset */
	gpio_request(MX6Q_SMD_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6Q_SMD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX6Q_SMD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6Q_SMD_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX6Q_SMD_CSI0_PWN, 0);
	mxc_iomux_set_gpr_register(1, 19, 1, 1);
}

static int __init early_enable_spdif(char *p)
{
	spdif_en = 1;
	return 0;
}

early_param("spdif", early_enable_spdif);

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,		/* enable tx */
	.spdif_rx = 1,		/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100 = 1,	/* tx clk from spdif0_clk_root */
	.spdif_clk_48000 = 1,	/* tx clk from spdif0_clk_root */
	.spdif_div_44100 = 23,
	.spdif_div_48000 = 37,
	.spdif_div_32000 = 37,
	.spdif_rx_clk = 0,	/* rx clk from spdif stream */
	.spdif_clk_set_rate = spdif_clk_set_rate,
	.spdif_clk = NULL,	/* spdif bus clk */
};

/*!
 * Board specific initialization.
 */
static void __init mx6_board_init(void)
{
	int i;
	int ret;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_pads,
					ARRAY_SIZE(mx6q_sabreauto_pads));

	if (esai_record)
	    mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_esai_record_pads,
			ARRAY_SIZE(mx6q_sabreauto_esai_record_pads));

	/*
	 * S/PDIF in and i2c3 are mutually exclusive because both
	 * use GPIO_16.
	 * S/PDIF out and can1 stby are mutually exclusive because both
	 * use GPIO_17.
	 */
	if (spdif_en) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_spdif_pads,
			ARRAY_SIZE(mx6q_sabreauto_spdif_pads));
	} else {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_i2c3_pads,
			ARRAY_SIZE(mx6q_sabreauto_i2c3_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabreauto_can_pads,
			ARRAY_SIZE(mx6q_sabreauto_can_pads));
	}

	if (mipi_sensor)
		mxc_iomux_v3_setup_multiple_pads(
			mx6q_sabreauto_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_sabreauto_mipi_sensor_pads));

	gp_reg_id = sabreauto_dvfscore_data.reg_id;
	mx6q_sabreauto_init_uart();
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);

	for (i = 0; i < ARRAY_SIZE(sabr_fb_data); i++)
		imx6q_add_ipuv3fb(i, &sabr_fb_data[i]);

	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_i2c(0, &mx6q_sabreauto_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6q_sabreauto_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	if (!spdif_en) {
		imx6q_add_imx_i2c(2, &mx6q_sabreauto_i2c_data);
		i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));
	}

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabreauto_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabreauto_anatop_thermal_data);

	if (!esai_record)
		imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_sabreauto_pm_data);
	/* imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabreauto_sd4_data); */
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabreauto_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabreauto_init_usb();
	imx6q_add_ahci(0, &mx6q_sabreauto_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabreauto_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	if (!mipi_sensor)
		mx6q_csi0_io_init();

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

	imx6q_add_dvfs_core(&sabreauto_dvfscore_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabreauto_pwm_backlight_data);

	if (spdif_en) {
		mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0",
			NULL);
		clk_put(mxc_spdif_data.spdif_core_clk);
		imx6q_add_spdif(&mxc_spdif_data);
		imx6q_add_spdif_dai();
		imx6q_add_spdif_audio_device();
	} else {
		ret = gpio_request_array(mx6q_flexcan_gpios,
				ARRAY_SIZE(mx6q_flexcan_gpios));
		if (ret) {
			pr_err("failed to request flexcan-gpios: %d\n", ret);
		} else {
			imx6q_add_flexcan0(&mx6q_sabreauto_flexcan_pdata[0]);
			imx6q_add_flexcan1(&mx6q_sabreauto_flexcan_pdata[1]);
		}
	}

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
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
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
			SZ_4K, SZ_2G);
		memblock_free(phys, imx6q_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_SABREAUTO data structure.
 */
MACHINE_START(MX6Q_SABREAUTO, "Freescale i.MX 6Quad Sabre Auto Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_board_init,
	.timer = &mxc_timer,
	.reserve = mx6q_reserve,
MACHINE_END
