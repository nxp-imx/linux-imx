/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx50.h>
#include <mach/epdc.h>
#include <mach/mxc_dvfs.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "devices-imx50.h"
#include "cpu_op-mx50.h"
#include "devices.h"
#include "usb.h"
#include "crm_regs.h"

#define FEC_EN		IMX_GPIO_NR(6, 23)
#define FEC_RESET_B	IMX_GPIO_NR(4, 12)
#define MX50_RDP_CSPI_CS0	IMX_GPIO_NR(4, 13)
#define MX50_RDP_CSPI_CS1	IMX_GPIO_NR(4, 11)
#define MX50_RDP_EPDC_D0        IMX_GPIO_NR(3, 0)
#define MX50_RDP_EPDC_D1        IMX_GPIO_NR(3, 1)
#define MX50_RDP_EPDC_D2        IMX_GPIO_NR(3, 2)
#define MX50_RDP_EPDC_D3        IMX_GPIO_NR(3, 3)
#define MX50_RDP_EPDC_D4        IMX_GPIO_NR(3, 4)
#define MX50_RDP_EPDC_D5        IMX_GPIO_NR(3, 5)
#define MX50_RDP_EPDC_D6        IMX_GPIO_NR(3, 6)
#define MX50_RDP_EPDC_D7        IMX_GPIO_NR(3, 7)
#define MX50_RDP_EPDC_GDCLK     IMX_GPIO_NR(3, 16)
#define MX50_RDP_EPDC_GDSP      IMX_GPIO_NR(3, 17)
#define MX50_RDP_EPDC_GDOE      IMX_GPIO_NR(3, 18)
#define MX50_RDP_EPDC_GDRL      IMX_GPIO_NR(3, 19)
#define MX50_RDP_EPDC_SDCLK     IMX_GPIO_NR(3, 20)
#define MX50_RDP_EPDC_SDOEZ     IMX_GPIO_NR(3, 21)
#define MX50_RDP_EPDC_SDOED     IMX_GPIO_NR(3, 22)
#define MX50_RDP_EPDC_SDOE      IMX_GPIO_NR(3, 23)
#define MX50_RDP_EPDC_SDLE      IMX_GPIO_NR(3, 24)
#define MX50_RDP_EPDC_SDCLKN    IMX_GPIO_NR(3, 25)
#define MX50_RDP_EPDC_SDSHR     IMX_GPIO_NR(3, 26)
#define MX50_RDP_EPDC_PWRCOM    IMX_GPIO_NR(3, 27)
#define MX50_RDP_EPDC_PWRSTAT   IMX_GPIO_NR(3, 28)
#define MX50_RDP_EPDC_PWRCTRL0  IMX_GPIO_NR(3, 29)
#define MX50_RDP_EPDC_PWRCTRL1  IMX_GPIO_NR(3, 30)
#define MX50_RDP_EPDC_PWRCTRL2  IMX_GPIO_NR(3, 31)
#define MX50_RDP_EPDC_PWRCTRL3  IMX_GPIO_NR(4, 20)
#define MX50_RDP_EPDC_BDR0      IMX_GPIO_NR(4, 23)
#define MX50_RDP_EPDC_BDR1      IMX_GPIO_NR(4, 24)
#define MX50_RDP_EPDC_SDCE0     IMX_GPIO_NR(4, 25)
#define MX50_RDP_EPDC_SDCE1     IMX_GPIO_NR(4, 26)
#define MX50_RDP_EPDC_SDCE2     IMX_GPIO_NR(4, 27)
#define MX50_RDP_EPDC_SDCE3     IMX_GPIO_NR(4, 28)
#define MX50_RDP_EPDC_SDCE4     IMX_GPIO_NR(4, 29)
#define MX50_RDP_EPDC_SDCE5     IMX_GPIO_NR(4, 30)
#define MX50_RDP_EPDC_PMIC_WAKE IMX_GPIO_NR(6, 16)
#define MX50_RDP_EPDC_PMIC_INT  IMX_GPIO_NR(6, 17)
#define MX50_RDP_EPDC_VCOM      IMX_GPIO_NR(4, 21)
#define MX50_RDP_SD1_WP		IMX_GPIO_NR(4, 19)	/*GPIO_4_19 */
#define MX50_RDP_SD1_CD		IMX_GPIO_NR(1, 27)	/*GPIO_1_27 */
#define MX50_RDP_SD2_WP		IMX_GPIO_NR(5, 16)	/*GPIO_5_16 */
#define MX50_RDP_SD2_CD		IMX_GPIO_NR(5, 17) 	/*GPIO_5_17 */
#define MX50_RDP_SD3_WP		IMX_GPIO_NR(5, 28) 	/*GPIO_5_28 */
#define MX50_RDP_USB_OTG_PWR	IMX_GPIO_NR(6, 25)	/*GPIO_6_25*/

extern int mx50_rdp_init_mc13892(void);
extern void mx5_cpu_regulator_init(void);
extern char *lp_reg_id;
extern char *gp_reg_id;
static int max17135_regulator_init(struct max17135 *max17135);

static iomux_v3_cfg_t mx50_rdp_pads[] __initdata = {
	/* SD1 */
	MX50_PAD_ECSPI2_SS0__GPIO_4_19,
	MX50_PAD_EIM_CRE__GPIO_1_27,
	MX50_PAD_SD1_CMD__SD1_CMD,

	MX50_PAD_SD1_CLK__SD1_CLK,
	MX50_PAD_SD1_D0__SD1_D0,
	MX50_PAD_SD1_D1__SD1_D1,
	MX50_PAD_SD1_D2__SD1_D2,
	MX50_PAD_SD1_D3__SD1_D3,

	/* SD2 */
	MX50_PAD_SD2_CD__GPIO_5_17,
	MX50_PAD_SD2_WP__GPIO_5_16,
	MX50_PAD_SD2_CMD__SD2_CMD,
	MX50_PAD_SD2_CLK__SD2_CLK,
	MX50_PAD_SD2_D0__SD2_D0,
	MX50_PAD_SD2_D1__SD2_D1,
	MX50_PAD_SD2_D2__SD2_D2,
	MX50_PAD_SD2_D3__SD2_D3,
	MX50_PAD_SD2_D4__SD2_D4,
	MX50_PAD_SD2_D5__SD2_D5,
	MX50_PAD_SD2_D6__SD2_D6,
	MX50_PAD_SD2_D7__SD2_D7,

	/* SD3 */
	MX50_PAD_SD3_CMD__SD3_CMD,
	MX50_PAD_SD3_CLK__SD3_CLK,
	MX50_PAD_SD3_D0__SD3_D0,
	MX50_PAD_SD3_D1__SD3_D1,
	MX50_PAD_SD3_D2__SD3_D2,
	MX50_PAD_SD3_D3__SD3_D3,
	MX50_PAD_SD3_D4__SD3_D4,
	MX50_PAD_SD3_D5__SD3_D5,
	MX50_PAD_SD3_D6__SD3_D6,
	MX50_PAD_SD3_D7__SD3_D7,

	/* PWR_INT */
	MX50_PAD_ECSPI2_MISO__GPIO_4_18,

	/* UART pad setting */
	MX50_PAD_UART1_TXD__UART1_TXD,
	MX50_PAD_UART1_RXD__UART1_RXD,
	MX50_PAD_UART1_RTS__UART1_RTS,
	MX50_PAD_UART2_TXD__UART2_TXD,
	MX50_PAD_UART2_RXD__UART2_RXD,
	MX50_PAD_UART2_CTS__UART2_CTS,
	MX50_PAD_UART2_RTS__UART2_RTS,

	MX50_PAD_I2C1_SCL__I2C1_SCL,
	MX50_PAD_I2C1_SDA__I2C1_SDA,
	MX50_PAD_I2C2_SCL__I2C2_SCL,
	MX50_PAD_I2C2_SDA__I2C2_SDA,

	/* EPDC pins */
	MX50_PAD_EPDC_PWRSTAT__GPIO_3_28,
	MX50_PAD_EPDC_VCOM0__GPIO_4_21,
	MX50_PAD_EPDC_PWRCTRL0__GPIO_3_29,

	MX50_PAD_DISP_D8__DISP_D8,
	MX50_PAD_DISP_D9__DISP_D9,
	MX50_PAD_DISP_D10__DISP_D10,
	MX50_PAD_DISP_D11__DISP_D11,
	MX50_PAD_DISP_D12__DISP_D12,
	MX50_PAD_DISP_D13__DISP_D13,
	MX50_PAD_DISP_D14__DISP_D14,
	MX50_PAD_DISP_D15__DISP_D15,
	MX50_PAD_DISP_RS__ELCDIF_VSYNC,

	/* EPD PMIC WAKEUP */
	MX50_PAD_UART4_TXD__GPIO_6_16,

	/* EPD PMIC intr */
	MX50_PAD_UART4_RXD__GPIO_6_17,

	MX50_PAD_EPITO__USBH1_PWR,
	/* Need to comment below line if
	 * one needs to debug owire.
	 */
	MX50_PAD_OWIRE__USBH1_OC,
	/* using gpio to control otg pwr */
	MX50_PAD_PWM2__GPIO_6_25,
	MX50_PAD_I2C3_SCL__USBOTG_OC,

	MX50_PAD_SSI_RXC__FEC_MDIO,
	MX50_PAD_SSI_RXFS__FEC_MDC,
	MX50_PAD_DISP_D0__FEC_TXCLK,
	MX50_PAD_DISP_D1__FEC_RX_ER,
	MX50_PAD_DISP_D2__FEC_RX_DV,
	MX50_PAD_DISP_D3__FEC_RXD1,
	MX50_PAD_DISP_D4__FEC_RXD0,
	MX50_PAD_DISP_D5__FEC_TX_EN,
	MX50_PAD_DISP_D6__FEC_TXD1,
	MX50_PAD_DISP_D7__FEC_TXD0,
	MX50_PAD_I2C3_SDA__GPIO_6_23,
	MX50_PAD_ECSPI1_SCLK__GPIO_4_12,

	MX50_PAD_CSPI_SS0__CSPI_SS0,
	MX50_PAD_ECSPI1_MOSI__CSPI_SS1,
	MX50_PAD_CSPI_MOSI__CSPI_MOSI,
	MX50_PAD_CSPI_MISO__CSPI_MISO,

	/* SGTL500_OSC_EN */
	MX50_PAD_UART1_CTS__GPIO_6_8,

	/* SGTL_AMP_SHDN */
	MX50_PAD_UART3_RXD__GPIO_6_15,

	/* Keypad */
	MX50_PAD_KEY_COL0__KEY_COL0,
	MX50_PAD_KEY_ROW0__KEY_ROW0,
	MX50_PAD_KEY_COL1__KEY_COL1,
	MX50_PAD_KEY_ROW1__KEY_ROW1,
	MX50_PAD_KEY_COL2__KEY_COL2,
	MX50_PAD_KEY_ROW2__KEY_ROW2,
	MX50_PAD_KEY_COL3__KEY_COL3,
	MX50_PAD_KEY_ROW3__KEY_ROW3,
	MX50_PAD_EIM_DA0__KEY_COL4,
	MX50_PAD_EIM_DA1__KEY_ROW4,
	MX50_PAD_EIM_DA2__KEY_COL5,
	MX50_PAD_EIM_DA3__KEY_ROW5,
	MX50_PAD_EIM_DA4__KEY_COL6,
	MX50_PAD_EIM_DA5__KEY_ROW6,
	MX50_PAD_EIM_DA6__KEY_COL7,
	MX50_PAD_EIM_DA7__KEY_ROW7,
	/*EIM pads */
	MX50_PAD_EIM_DA8__GPIO_1_8,
	MX50_PAD_EIM_DA9__GPIO_1_9,
	MX50_PAD_EIM_DA10__GPIO_1_10,
	MX50_PAD_EIM_DA11__GPIO_1_11,
	MX50_PAD_EIM_DA12__GPIO_1_12,
	MX50_PAD_EIM_DA13__GPIO_1_13,
	MX50_PAD_EIM_DA14__GPIO_1_14,
	MX50_PAD_EIM_DA15__GPIO_1_15,
	MX50_PAD_EIM_CS2__GPIO_1_16,
	MX50_PAD_EIM_CS1__GPIO_1_17,
	MX50_PAD_EIM_CS0__GPIO_1_18,
	MX50_PAD_EIM_EB0__GPIO_1_19,
	MX50_PAD_EIM_EB1__GPIO_1_20,
	MX50_PAD_EIM_WAIT__GPIO_1_21,
	MX50_PAD_EIM_BCLK__GPIO_1_22,
	MX50_PAD_EIM_RDY__GPIO_1_23,
	MX50_PAD_EIM_OE__GPIO_1_24,
};

/* Serial ports */
static const struct imxuart_platform_data uart_pdata __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct fec_platform_data fec_data __initconst = {
	.phy = PHY_INTERFACE_MODE_RMII,
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
	.gpio_pmic_pwrgood = MX50_RDP_EPDC_PWRSTAT,
	.gpio_pmic_vcom_ctrl = MX50_RDP_EPDC_VCOM,
	.gpio_pmic_wakeup = MX50_RDP_EPDC_PMIC_WAKE,
	.gpio_pmic_v3p3 = MX50_RDP_EPDC_PWRCTRL0,
	.gpio_pmic_intr = MX50_RDP_EPDC_PMIC_INT,
	.regulator_init = max17135_init_data,
	.init = max17135_regulator_init,
};

static int max17135_regulator_init(struct max17135 *max17135)
{
	struct max17135_platform_data *pdata = &max17135_pdata;
	int i, ret;

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

	regulator_has_full_constraints();

	return 0;
}

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	 I2C_BOARD_INFO("max17135", 0x48),
	 .platform_data = &max17135_pdata,
	 },
	 {
	 .type = "mma8450",
	 .addr = 0x1c,
	 },
	 {
	 .type = "eeprom",
	 .addr = 0x50,
	 },
};

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(MX50_RDP_EPDC_D0, "epdc_d0");
	ret |= gpio_request(MX50_RDP_EPDC_D1, "epdc_d1");
	ret |= gpio_request(MX50_RDP_EPDC_D2, "epdc_d2");
	ret |= gpio_request(MX50_RDP_EPDC_D3, "epdc_d3");
	ret |= gpio_request(MX50_RDP_EPDC_D4, "epdc_d4");
	ret |= gpio_request(MX50_RDP_EPDC_D5, "epdc_d5");
	ret |= gpio_request(MX50_RDP_EPDC_D6, "epdc_d6");
	ret |= gpio_request(MX50_RDP_EPDC_D7, "epdc_d7");
	ret |= gpio_request(MX50_RDP_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(MX50_RDP_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(MX50_RDP_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(MX50_RDP_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(MX50_RDP_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(MX50_RDP_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(MX50_RDP_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(MX50_RDP_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(MX50_RDP_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(MX50_RDP_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(MX50_RDP_EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(MX50_RDP_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(MX50_RDP_EPDC_D0);
	gpio_free(MX50_RDP_EPDC_D1);
	gpio_free(MX50_RDP_EPDC_D2);
	gpio_free(MX50_RDP_EPDC_D3);
	gpio_free(MX50_RDP_EPDC_D4);
	gpio_free(MX50_RDP_EPDC_D5);
	gpio_free(MX50_RDP_EPDC_D6);
	gpio_free(MX50_RDP_EPDC_D7);
	gpio_free(MX50_RDP_EPDC_GDCLK);
	gpio_free(MX50_RDP_EPDC_GDSP);
	gpio_free(MX50_RDP_EPDC_GDOE);
	gpio_free(MX50_RDP_EPDC_GDRL);
	gpio_free(MX50_RDP_EPDC_SDCLK);
	gpio_free(MX50_RDP_EPDC_SDOE);
	gpio_free(MX50_RDP_EPDC_SDLE);
	gpio_free(MX50_RDP_EPDC_SDSHR);
	gpio_free(MX50_RDP_EPDC_BDR0);
	gpio_free(MX50_RDP_EPDC_SDCE0);
	gpio_free(MX50_RDP_EPDC_SDCE1);
	gpio_free(MX50_RDP_EPDC_SDCE2);
}

static iomux_v3_cfg_t mx50_epdc_pads_enabled[] = {
	MX50_PAD_EPDC_D0__EPDC_D0,
	MX50_PAD_EPDC_D1__EPDC_D1,
	MX50_PAD_EPDC_D2__EPDC_D2,
	MX50_PAD_EPDC_D3__EPDC_D3,
	MX50_PAD_EPDC_D4__EPDC_D4,
	MX50_PAD_EPDC_D5__EPDC_D5,
	MX50_PAD_EPDC_D6__EPDC_D6,
	MX50_PAD_EPDC_D7__EPDC_D7,
	MX50_PAD_EPDC_GDCLK__EPDC_GDCLK,
	MX50_PAD_EPDC_GDSP__EPDC_GDSP,
	MX50_PAD_EPDC_GDOE__EPDC_GDOE,
	MX50_PAD_EPDC_GDRL__EPDC_GDRL,
	MX50_PAD_EPDC_SDCLK__EPDC_SDCLK,
	MX50_PAD_EPDC_SDOE__EPDC_SDOE,
	MX50_PAD_EPDC_SDLE__EPDC_SDLE,
	MX50_PAD_EPDC_SDSHR__EPDC_SDSHR,
	MX50_PAD_EPDC_BDR0__EPDC_BDR0,
	MX50_PAD_EPDC_SDCE0__EPDC_SDCE0,
	MX50_PAD_EPDC_SDCE1__EPDC_SDCE1,
	MX50_PAD_EPDC_SDCE2__EPDC_SDCE2,
};

static iomux_v3_cfg_t mx50_epdc_pads_disabled[] = {
	MX50_PAD_EPDC_D0__GPIO_3_0,
	MX50_PAD_EPDC_D1__GPIO_3_1,
	MX50_PAD_EPDC_D2__GPIO_3_2,
	MX50_PAD_EPDC_D3__GPIO_3_3,
	MX50_PAD_EPDC_D4__GPIO_3_4,
	MX50_PAD_EPDC_D5__GPIO_3_5,
	MX50_PAD_EPDC_D6__GPIO_3_6,
	MX50_PAD_EPDC_D7__GPIO_3_7,
	MX50_PAD_EPDC_GDCLK__GPIO_3_16,
	MX50_PAD_EPDC_GDSP__GPIO_3_17,
	MX50_PAD_EPDC_GDOE__GPIO_3_18,
	MX50_PAD_EPDC_GDRL__GPIO_3_19,
	MX50_PAD_EPDC_SDCLK__GPIO_3_20,
	MX50_PAD_EPDC_SDOE__GPIO_3_23,
	MX50_PAD_EPDC_SDLE__GPIO_3_24,
	MX50_PAD_EPDC_SDSHR__GPIO_3_26,
	MX50_PAD_EPDC_BDR0__GPIO_4_23,
	MX50_PAD_EPDC_SDCE0__GPIO_4_25,
	MX50_PAD_EPDC_SDCE1__GPIO_4_26,
	MX50_PAD_EPDC_SDCE2__GPIO_4_27,
};

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx50_epdc_pads_enabled, \
				ARRAY_SIZE(mx50_epdc_pads_enabled));

	gpio_direction_input(MX50_RDP_EPDC_D0);
	gpio_direction_input(MX50_RDP_EPDC_D1);
	gpio_direction_input(MX50_RDP_EPDC_D2);
	gpio_direction_input(MX50_RDP_EPDC_D3);
	gpio_direction_input(MX50_RDP_EPDC_D4);
	gpio_direction_input(MX50_RDP_EPDC_D5);
	gpio_direction_input(MX50_RDP_EPDC_D6);
	gpio_direction_input(MX50_RDP_EPDC_D7);
	gpio_direction_input(MX50_RDP_EPDC_GDCLK);
	gpio_direction_input(MX50_RDP_EPDC_GDSP);
	gpio_direction_input(MX50_RDP_EPDC_GDOE);
	gpio_direction_input(MX50_RDP_EPDC_GDRL);
	gpio_direction_input(MX50_RDP_EPDC_SDCLK);
	gpio_direction_input(MX50_RDP_EPDC_SDOE);
	gpio_direction_input(MX50_RDP_EPDC_SDLE);
	gpio_direction_input(MX50_RDP_EPDC_SDSHR);
	gpio_direction_input(MX50_RDP_EPDC_BDR0);
	gpio_direction_input(MX50_RDP_EPDC_SDCE0);
	gpio_direction_input(MX50_RDP_EPDC_SDCE1);
	gpio_direction_input(MX50_RDP_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx50_epdc_pads_disabled, \
				ARRAY_SIZE(mx50_epdc_pads_disabled));

	gpio_direction_output(MX50_RDP_EPDC_D0, 0);
	gpio_direction_output(MX50_RDP_EPDC_D1, 0);
	gpio_direction_output(MX50_RDP_EPDC_D2, 0);
	gpio_direction_output(MX50_RDP_EPDC_D3, 0);
	gpio_direction_output(MX50_RDP_EPDC_D4, 0);
	gpio_direction_output(MX50_RDP_EPDC_D5, 0);
	gpio_direction_output(MX50_RDP_EPDC_D6, 0);
	gpio_direction_output(MX50_RDP_EPDC_D7, 0);
	gpio_direction_output(MX50_RDP_EPDC_GDCLK, 0);
	gpio_direction_output(MX50_RDP_EPDC_GDSP, 0);
	gpio_direction_output(MX50_RDP_EPDC_GDOE, 0);
	gpio_direction_output(MX50_RDP_EPDC_GDRL, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDCLK, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDOE, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDLE, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDSHR, 0);
	gpio_direction_output(MX50_RDP_EPDC_BDR0, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDCE0, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDCE1, 0);
	gpio_direction_output(MX50_RDP_EPDC_SDCE2, 0);
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
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		428,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		1,	/* num_ce */
	},
	{
		&e60_v220_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		465,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		9,	/* gdclk_offs */
		1,	/* num_ce */
	},
	{
		&e97_v110_mode,
		8,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		632,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		1,	/* gdclk_offs */
		3,	/* num_ce */
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

static inline void mx50_rdp_fec_reset(void)
{
	gpio_request(FEC_EN, "fec-en");
	gpio_direction_output(FEC_EN, 0);
	gpio_request(FEC_RESET_B, "fec-reset_b");
	gpio_direction_output(FEC_RESET_B, 0);
	msleep(1);
	gpio_set_value(FEC_RESET_B, 1);
}

static const struct imxi2c_platform_data i2c_data __initconst = {
	.bitrate = 100000,
};

static struct mxc_gpu_platform_data gpu_data __initdata = {
	.z160_revision = 1,
};

static int mx50_rdp_spi_cs[] = {
	MX50_RDP_CSPI_CS0,
	MX50_RDP_CSPI_CS1,
};

static const struct spi_imx_master mx50_rdp_spi_pdata __initconst = {
	.chipselect     = mx50_rdp_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx50_rdp_spi_cs),
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx50_gpmi_nand[] __initdata = {
	MX50_PIN_EIM_DA8__NANDF_CLE,
	MX50_PIN_EIM_DA9__NANDF_ALE,
	MX50_PIN_EIM_DA10__NANDF_CE0,
	MX50_PIN_EIM_DA11__NANDF_CE1,
	MX50_PIN_EIM_DA12__NANDF_CE2,
	MX50_PIN_EIM_DA13__NANDF_CE3,
	MX50_PAD_EIM_DA14__NANDF_READY,
	MX50_PIN_EIM_DA15__NANDF_DQS,
	MX50_PIN_SD3_D4__NANDF_D0,
	MX50_PIN_SD3_D5__NANDF_D1,
	MX50_PIN_SD3_D6__NANDF_D2,
	MX50_PIN_SD3_D7__NANDF_D3,
	MX50_PIN_SD3_D0__NANDF_D4,
	MX50_PIN_SD3_D1__NANDF_D5,
	MX50_PIN_SD3_D2__NANDF_D6,
	MX50_PIN_SD3_D3__NANDF_D7,
	MX50_PIN_SD3_CLK__NANDF_RDN,
	MX50_PIN_SD3_CMD__NANDF_WRN,
	MX50_PIN_SD3_WP__NANDF_RESETN,
};

static int gpmi_nfc_platform_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(mx50_gpmi_nand,
					ARRAY_SIZE(mx50_gpmi_nand));
}

static struct gpmi_nfc_platform_data  mx50_gpmi_nfc_platform_data __initdata = {
	.platform_init           = gpmi_nfc_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
};


static struct mxc_dvfs_platform_data rdp_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
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

static struct mxc_regulator_platform_data rdp_regulator_data = {
	.cpu_reg_id = "cpu_vddgp",
	.vcc_reg_id = "lp_vcc",
};

static const struct esdhc_platform_data mx50_rdp_sd1_data __initconst = {
	.cd_gpio = MX50_RDP_SD1_CD,
	.wp_gpio = MX50_RDP_SD1_WP,
};

static const struct esdhc_platform_data mx50_rdp_sd2_data __initconst = {
	.cd_gpio = MX50_RDP_SD2_CD,
	.wp_gpio = MX50_RDP_SD2_WP,
};

static const struct esdhc_platform_data mx50_rdp_sd3_data __initconst = {
	.wp_gpio = MX50_RDP_SD3_WP,
	.always_present = 1,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static void mx50_rdp_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX50_RDP_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX50_RDP_USB_OTG_PWR, 0);
}

static void __init mx50_rdp_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX50_IO_ADDRESS(MX50_OTG_BASE_ADDR);
	ret = gpio_request(MX50_RDP_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO OTG_VBUS: %d\n", ret);
		return;
	}
	gpio_direction_output(MX50_RDP_USB_OTG_PWR, 0);

	mx5_set_otghost_vbus_func(mx50_rdp_usbotg_vbus);
	mx5_usb_dr_init();
	mx5_usbh1_init();
}
/*
 * Board specific initialization.
 */
static void __init mx50_rdp_board_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx50_rdp_pads,
					ARRAY_SIZE(mx50_rdp_pads));

	pr_info("CPU is iMX50 Revision %u\n",
		mx50_revision());

	gp_reg_id = rdp_regulator_data.cpu_reg_id;
	lp_reg_id = rdp_regulator_data.vcc_reg_id;

	imx50_add_cspi(3, &mx50_rdp_spi_pdata);

	imx50_add_dma();
	imx50_add_imx_uart(0, NULL);
	imx50_add_imx_uart(1, NULL);
	imx50_add_srtc();
	imx50_add_imx_pxp();
	imx50_add_imx_pxp_client();
	mx50_rdp_fec_reset();
	imx50_add_fec(&fec_data);
	imx50_add_gpmi(&mx50_gpmi_nfc_platform_data);
	imx50_add_imx_i2c(0, &i2c_data);
	imx50_add_imx_i2c(1, &i2c_data);
	imx50_add_imx_i2c(2, &i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	mxc_register_device(&max17135_sensor_device, NULL);
	imx50_add_imx_epdc(&epdc_data);
	imx50_add_mxc_gpu(&gpu_data);
	imx50_add_sdhci_esdhc_imx(0, &mx50_rdp_sd1_data);
	imx50_add_sdhci_esdhc_imx(1, &mx50_rdp_sd2_data);
	imx50_add_sdhci_esdhc_imx(2, &mx50_rdp_sd3_data);
	imx50_add_otp();
	imx50_add_viim();
	imx50_add_dcp();
	imx50_add_rngb();
	imx50_add_perfmon();
	mx50_rdp_init_mc13892();

	imx50_add_dvfs_core(&rdp_dvfscore_data);

	imx50_add_busfreq();

	mx50_rdp_init_usb();

	mx5_cpu_regulator_init();
}

static void __init mx50_rdp_timer_init(void)
{
	mx50_clocks_init(32768, 24000000, 22579200);
}

static struct sys_timer mx50_rdp_timer = {
	.init	= mx50_rdp_timer_init,
};

MACHINE_START(MX50_RDP, "Freescale MX50 Reference Design Platform")
	.fixup = fixup_mxc_board,
	.map_io = mx50_map_io,
	.init_early = imx50_init_early,
	.init_irq = mx50_init_irq,
	.timer = &mx50_rdp_timer,
	.init_machine = mx50_rdp_board_init,
MACHINE_END
