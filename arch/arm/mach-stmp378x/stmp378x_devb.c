/*
 * Freescale STMP378X development board support
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/input.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/pins.h>
#include <mach/pinmux.h>
#include <mach/platform.h>
#include <mach/stmp3xxx.h>
#include <mach/gpmi.h>
#include <mach/mmc.h>
#include <mach/lcdif.h>
#include <mach/ddi_bc.h>

#include "stmp378x.h"

static struct platform_device *devices[] = {
	&stmp3xxx_dbguart,
	&stmp3xxx_appuart,
	&stmp3xxx_watchdog,
	&stmp3xxx_touchscreen,
	&stmp3xxx_rtc,
	&stmp3xxx_keyboard,
	&stmp3xxx_framebuffer,
	&stmp3xxx_backlight,
	&stmp3xxx_rotdec,
	&stmp3xxx_persistent,
	&stmp3xxx_dcp_bootstream,
	&stmp3xxx_dcp,
	&stmp3xxx_battery,
	&stmp378x_pxp,
	&stmp378x_i2c,
	&stmp3xxx_spdif,
	&stmp378x_audio,
};

int usb_host_wakeup_irq(struct device *wkup_dev)
{
	        return 0;
}
EXPORT_SYMBOL(usb_host_wakeup_irq);

void usb_host_set_wakeup(struct device *wkup_dev, bool para)
{
}
EXPORT_SYMBOL(usb_host_set_wakeup);

static struct pin_desc i2c_pins_desc[] = {
	{ PINID_I2C_SCL, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_I2C_SDA, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
};

static struct pin_group i2c_pins = {
	.pins		= i2c_pins_desc,
	.nr_pins	= ARRAY_SIZE(i2c_pins_desc),
};

static struct pin_desc dbguart_pins_0[] = {
	{ PINID_PWM0, PIN_FUN3, },
	{ PINID_PWM1, PIN_FUN3, },
};

static struct pin_group dbguart_pins[] = {
	[0] = {
		.pins		= dbguart_pins_0,
		.nr_pins	= ARRAY_SIZE(dbguart_pins_0),
	},
};

static int dbguart_pinmux(int request, int id)
{
	int r = 0;

	if (request)
		r = stmp3xxx_request_pin_group(&dbguart_pins[id], "dbguart");
	else
		stmp3xxx_release_pin_group(&dbguart_pins[id], "dbguart");
	return r;
}

static struct pin_desc appuart_pins_0[] = {
	{ PINID_AUART1_CTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_RTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_RX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART1_TX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
};

static struct pin_desc appuart_pins_1[] = {
#if 0 /* enable these when second appuart will be connected */
	{ PINID_AUART2_CTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_RTS, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_RX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
	{ PINID_AUART2_TX, PIN_FUN1, PIN_4MA, PIN_1_8V, 0, },
#endif
};

static struct pin_group appuart_pins[] = {
	[0] = {
		.pins		= appuart_pins_0,
		.nr_pins	= ARRAY_SIZE(appuart_pins_0),
	},
	[1] = {
		.pins		= appuart_pins_1,
		.nr_pins	= ARRAY_SIZE(appuart_pins_1),
	},
};

static int appuart_pinmux(int req, int id)
{
	if (req)
		return stmp3xxx_request_pin_group(&appuart_pins[id], "appuart");
	else
		stmp3xxx_release_pin_group(&appuart_pins[id], "appuart");
	return 0;
}

static struct pin_desc ssp1_pins_desc[] = {
	{ PINID_SSP1_SCK,	PIN_FUN1, PIN_8MA, PIN_3_3V, 0, },
	{ PINID_SSP1_CMD,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
	{ PINID_SSP1_DATA0,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
	{ PINID_SSP1_DATA3,	PIN_FUN1, PIN_4MA, PIN_3_3V, 0, },
};

static struct pin_desc ssp2_pins_desc[] = {
	{ PINID_GPMI_WRN,	PIN_FUN3, PIN_8MA, PIN_3_3V, 0, },
	{ PINID_GPMI_RDY1,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
	{ PINID_GPMI_D00,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
	{ PINID_GPMI_D03,	PIN_FUN3, PIN_4MA, PIN_3_3V, 0, },
};

static struct pin_group ssp1_pins = {
	.pins = ssp1_pins_desc,
	.nr_pins = ARRAY_SIZE(ssp1_pins_desc),
};

static struct pin_group ssp2_pins = {
	.pins = ssp1_pins_desc,
	.nr_pins = ARRAY_SIZE(ssp2_pins_desc),
};

static struct pin_desc gpmi_pins_desc[] = {
	{ PINID_GPMI_CE0N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_CE1N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GMPI_CE2N, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_CLE, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_ALE, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_WPN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY1, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D00, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D01, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D02, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D03, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D04, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D05, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D06, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D07, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY0, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY2, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDY3, PIN_FUN1, PIN_4MA, PIN_3_3V, 0 },
	{ PINID_GPMI_WRN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
	{ PINID_GPMI_RDN, PIN_FUN1, PIN_12MA, PIN_3_3V, 0 },
};

static struct pin_group gpmi_pins = {
	.pins		= gpmi_pins_desc,
	.nr_pins	= ARRAY_SIZE(gpmi_pins_desc),
};

static int gpmi_pinmux(int req)
{
	if (req)
		return stmp3xxx_request_pin_group(&gpmi_pins, "gpmi");
	else
		stmp3xxx_release_pin_group(&gpmi_pins, "gpmi");
	return 0;
}

const char *gpmi_part_probes[] = { "cmdlinepart", NULL };

#define UID_SIZE	SZ_1M
#define UID_OFFSET 	(20*SZ_1M)

struct mtd_partition gpmi_partitions_chip0[] = {
	[0] = {
		.offset		= 0,
		.size		= UID_OFFSET,
		.name		= "Boot#0",
		.mask_flags	= 0,
	},
	/* This partition is managed by UBI */
	[1] = {
		.offset		= UID_OFFSET + UID_SIZE,
		.size		= MTDPART_SIZ_FULL,
		.name		= "UBI#0",
		.mask_flags	= 0,
	},
};

struct mtd_partition gpmi_partitions_chip1[] = {
	[0] = {
		.offset		= 0,
		.size		= UID_OFFSET,
		.name		= "Boot#1",
		.mask_flags	= 0,
	},
	/* This partition is managed by UBI */
	[1] = {
		.offset		= UID_OFFSET,
		.size		= MTDPART_SIZ_FULL,
		.name		= "UBI#1",
		.mask_flags	= 0,
	},
};

static char *gpmi_concat_parts[] = {
	[0]	= "UBI#0",
	[1]	= "UBI#1",
	[2]	= NULL,
};

static struct gpmi_platform_data gpmi_partitions = {
	.uid_offset = UID_OFFSET,
	.uid_size = UID_SIZE,
	.io_uA = 70000,
	.items = 2,
	.concat_name = "UBI",
	.concat_parts = gpmi_concat_parts,
	.pinmux = gpmi_pinmux,
	.parts = {
		[0] = {
			.part_probe_types = gpmi_part_probes,
			.nr_partitions = ARRAY_SIZE(gpmi_partitions_chip0),
			.partitions = gpmi_partitions_chip0,
		},
		[1] = {
			.part_probe_types = gpmi_part_probes,
			.nr_partitions = ARRAY_SIZE(gpmi_partitions_chip1),
			.partitions = gpmi_partitions_chip1,
		},
	},
};

static struct pin_desc lcd_hx8238a_desc[] = {
	{ PINID_LCD_D00, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D01, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D02, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D03, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D04, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D05, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D06, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D07, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D08, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D09, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D10, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D11, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D12, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D13, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D14, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D15, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D16, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_D17, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_RESET, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_VSYNC, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_HSYNC, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_ENABLE, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_LCD_DOTCK, PIN_FUN1, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D13, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D12, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D11, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D10, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D09, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
	{ PINID_GPMI_D08, PIN_FUN2, PIN_8MA, PIN_3_3V, 0 },
};

struct pin_group lcd_pins = {
	.pins		= lcd_hx8238a_desc,
	.nr_pins	= ARRAY_SIZE(lcd_hx8238a_desc),
};

unsigned lcd_spi_pins[] = {
	[SPI_MOSI] = PINID_LCD_WR,
	[SPI_SCLK] = PINID_LCD_RS,
	[SPI_CS] = PINID_LCD_CS,
};

static struct pin_desc spdif_pins_desc[] = {
	{ PINID_ROTARYA, PIN_FUN3, PIN_4MA, PIN_1_8V, 0, },
};

struct pin_group spdif_pins = {
	.pins		= spdif_pins_desc,
	.nr_pins	= ARRAY_SIZE(spdif_pins_desc),
};

int spdif_pinmux(int req)
{
	if (req)
		return stmp3xxx_request_pin_group(&spdif_pins, "spdif");
	else
		stmp3xxx_release_pin_group(&spdif_pins, "spdif");
	return 0;
}
EXPORT_SYMBOL_GPL(spdif_pinmux);

static struct stmp3xxxmmc_platform_data mmc_data = {
	.hw_init	= stmp3xxxmmc_hw_init_ssp1,
	.hw_release	= stmp3xxxmmc_hw_release_ssp1,
	.get_wp		= stmp3xxxmmc_get_wp,
	.cmd_pullup	= stmp3xxxmmc_cmd_pullup_ssp1,
	.setclock	= stmp3xxxmmc_setclock_ssp1,
	.read_uA        = 50000,
	.write_uA       = 70000,
};

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)
	{
		.modalias       = "enc28j60",
		.max_speed_hz   = 6 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select    = 0,
		.platform_data  = NULL,
	},
#endif
};

/* battery info data */
static ddi_bc_Cfg_t battery_data = {
	.u32StateMachinePeriod		 = 100,		/* ms */
	.u16CurrentRampSlope		 = 75,		/* mA/s */
	.u16ConditioningThresholdVoltage = 2900, 	/* mV */
	.u16ConditioningMaxVoltage	 = 3000,	/* mV */
	.u16ConditioningCurrent		 = 60,		/* mA */
	.u32ConditioningTimeout		 = 4*60*60*1000, /* ms (4 hours) */
	.u16ChargingVoltage		 = 4200,	/* mV */
	/* FIXME: the current comparator could have h/w bugs in current
	 * detection through POWER_STS.CHRGSTS bit */
	.u16ChargingCurrent		 = 600,		/* mA 600 */
	.u16ChargingThresholdCurrent	 = 60,		/* mA 60 */
	.u32ChargingTimeout		 = 4*60*60*1000,/* ms (4 hours) */
	.u32TopOffPeriod		 = 30*60*1000,	/* ms (30 minutes) */
	.monitorDieTemp			 = 1,		/* Monitor the die */
	.u8DieTempHigh			 = 75,		/* deg centigrade */
	.u8DieTempLow			 = 65,		/* deg centigrade */
	.u16DieTempSafeCurrent		 = 0,		/* mA */
	.monitorBatteryTemp		 = 0,		/* Monitor the battery*/
	.u8BatteryTempChannel		 = 1,		/* LRADC 1 */
	.u16BatteryTempHigh		 = 642,		/* Unknown units */
	.u16BatteryTempLow		 = 497,		/* Unknown units */
	.u16BatteryTempSafeCurrent	 = 0,		/* mA */
};

static struct stmpkbd_keypair keyboard_data[] = {
	{ 100, KEY_F4 },
	{ 306, KEY_F5 },
	{ 626, KEY_F6 },
	{ 932, KEY_F7 },
	{ 1260, KEY_F8 },
	{ 1584, KEY_F9 },
	{ 1907, KEY_F10 },
	{ 2207, KEY_F11 },
	{ 2525, KEY_F12 },
	{ 2831, KEY_F13},
	{ 3134, KEY_F14 },
	{ -1, 0 },
};

static void __init stmp378x_devb_init(void)
{
	stmp3xxx_pinmux_init(NR_REAL_IRQS);

	/* init stmp3xxx platform */
	stmp3xxx_init();

	stmp3xxx_dbguart.dev.platform_data = dbguart_pinmux;
	stmp3xxx_appuart.dev.platform_data = appuart_pinmux;
	stmp3xxx_gpmi.dev.platform_data = &gpmi_partitions;
	stmp3xxx_mmc.dev.platform_data = &mmc_data;
	stmp3xxx_spi1.dev.platform_data = &ssp1_pins;
	stmp3xxx_spi2.dev.platform_data = &ssp2_pins;
	stmp378x_i2c.dev.platform_data = &i2c_pins;
	stmp3xxx_battery.dev.platform_data = &battery_data;
	stmp3xxx_keyboard.dev.platform_data = &keyboard_data;

	/* register spi devices */
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));

	/* add board's devices */
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* add devices selected by command line ssp1= and ssp2= options */
	stmp3xxx_ssp1_device_register();
	stmp3xxx_ssp2_device_register();
}

MACHINE_START(STMP378X, "STMP378X")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.map_io		= stmp378x_map_io,
	.init_irq	= stmp378x_init_irq,
	.timer		= &stmp3xxx_timer,
	.init_machine	= stmp378x_devb_init,
MACHINE_END
