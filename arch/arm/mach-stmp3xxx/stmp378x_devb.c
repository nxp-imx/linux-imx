/*
 * Freescale STMP378X development board support
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/stmp3xxx.h>
#include <mach/gpmi.h>
#include <mach/power.h>
#include <mach/regs-power.h>
#include <mach/regs-digctl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-pwm.h>

#include "common.h"

static struct resource stmp378xdb_led_resources[] = {
	{
		.name   = "led0",
		.start  = 0,
		.end    = 0,
		.flags  = IORESOURCE_DISABLED,
	},
	{
		.name   = "led1",
		.start  = 1,
		.end    = 1,
		.flags  = IORESOURCE_DISABLED,
	},
	{
		.name   = "led2",
		.start  = 2,
		.end    = 2,
		.flags  = IORESOURCE_DISABLED,
	},
	{
		.name   = "led3",
		.start  = 3,
		.end    = 3,
		.flags  = IORESOURCE_DISABLED,
	},
	{
		.name   = "led4",
		.start  = 4,
		.end    = 4,
		.flags  = IORESOURCE_DISABLED,
	},
};

static struct platform_device stmp378x_leds = {
	.name	   = "stmp378x-pwm-led",
	.id	     = -1,
	.num_resources  = ARRAY_SIZE(stmp378xdb_led_resources),
	.resource       = stmp378xdb_led_resources,
};

static struct platform_device *devices[] = {
	&stmp3xxx_keyboard,
	&stmp3xxx_touchscreen,
	&stmp3xxx_appuart,
	&stmp3xxx_dbguart,
	&stmp3xxx_watchdog,
	&stmp3xxx_rtc,
	&stmp3xxx_framebuffer,
	&stmp3xxx_backlight,
	&stmp3xxx_rotdec,
	&stmp378x_i2c,
	&stmp3xxx_persistent,
	&stmp3xxx_dcp_bootstream,
	&stmp3xxx_dcp,
	&stmp3xxx_mtest,
	&stmp3xxx_battery,
	&stmp3xxx_pxp,
	&stmp3xxx_viim,
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
	/* there a 1M UID partition here */
/*
	[1] = {
		.offset		= MTDPART_OFS_APPEND,
		.size		= 5 * SZ_1M,
		.name		= "id",
		.mask_flags	= 0,
	},
*/
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

int usb_host_wakeup_irq(struct device *wkup_dev)
{
	return 0;
}
EXPORT_SYMBOL(usb_host_wakeup_irq);

void usb_host_set_wakeup(struct device *wkup_dev, bool para)
{
}
EXPORT_SYMBOL(usb_host_set_wakeup);

static struct stmp37xx_spi_platform_data enc_data = {
	.irq_pin = PINID_SSP1_DATA1,
	.hw_init = stmp37xx_spi_enc_init,
	.hw_release = stmp37xx_spi_enc_release,
};

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)
	{
		.modalias       = "enc28j60",
		.max_speed_hz   = 6 * 1000 * 1000,
		.bus_num	= 1,
		.chip_select    = 0,
		.platform_data  = &enc_data,
	},
#endif
};

/*
 * There are 4 LEDs connected to PWM module available on the STMP378x
 * Dev. board:
 *  PWM0 - shared with Debug UART
 *  PWM1 - shared with Debug UART
 *  PWM2 - shared with LCD panel (LMS350)
 *  PWM3 - shared with SD/MMC card slot
 *
 * pwm_leds= option allows to choose PWM outputs to be used to control LEDs
 * Conflicting kernel modules should be disabled
 */
#define PWM_MAX	4
static int pwm_leds_enable;

int __init pwm_leds_setup(char *str)
{
	char tmp[10];
	char *l, *s;
	unsigned long pwmn;
	int r;

	s = str;

	do {
		memset(tmp, 0, sizeof(tmp));

		l = strchr(s, ',');

		if (l) {
			strncpy(tmp, s, min_t(int, l - s, sizeof(tmp) - 1));
			r = strict_strtoul(tmp, 0, &pwmn);
			s = ++l;
		} else {
			r = strict_strtoul(s, 0, &pwmn);
		}

		if (r == 0 && pwmn < PWM_MAX) {
			stmp378xdb_led_resources[pwmn].flags = 0;
			pwm_leds_enable++;
		}
	} while (l);

	return 0;
}
__setup("pwm_leds=", pwm_leds_setup);

int __init enc28j60_setup(char *str)
{
	char *cur = str,
	     *item;
	unsigned long bus, cs, irqbank, irqpin;
	char tmps[20];
	int r;

	bus = cs = irqbank = irqpin = 0;
	/*
	  we recognize strings like enc28j60=chipselect@bus,irqpin
	  where
		chipselect is 0..4,
		bus is 0/1,
		irqpin is in format bank:pin
	 */
	item = strchr(cur, '@');
	if (item) {
		memset(tmps, 0, sizeof(tmps));
		strncpy(tmps, cur, min_t(int, item - cur, sizeof(tmps - 1)));
		r = strict_strtoul(tmps, 0, &cs);
		if (r < 0)
			return r;
		cur = item + 1;
	}

	item = strchr(cur, ',');
	if (item) {
		memset(tmps, 0, sizeof(tmps));
		strncpy(tmps, cur, min_t(int, item - cur, sizeof(tmps - 1)));
		r = strict_strtoul(tmps, 0, &bus);
		if (r < 0)
			return r;
		cur = item + 1;
	}

	item = strchr(cur, ':');
	if (item) {
		memset(tmps, 0, sizeof(tmps));
		strncpy(tmps, cur, min_t(int, item - cur, sizeof(tmps - 1)));
		r = strict_strtoul(tmps, 0, &irqbank);
		if (r < 0)
			return r;
		cur = item + 1;
	}

	r = strict_strtoul(cur, 0, &irqpin);
	if (r < 0)
		return r;

	pr_info("%s: bus = %ld, cs = %ld, irqpin = %ld + %ld\n",
		__func__, bus, cs, irqbank, irqpin);

	spi_board_info[0].bus_num = bus;
	spi_board_info[0].chip_select = cs;
	enc_data.irq_pin = STMP3XXX_PINID(irqbank, irqpin);
	return 0;
}
__setup("enc28j60=", enc28j60_setup);

static struct i2c_board_info __initdata stmp3xxx_i2c_devices[] = {
	{ I2C_BOARD_INFO("stfm1000", 0xc0), .flags = I2C_M_TEN }
};

static void __init stmp378x_devb_init(void)
{
	struct fsl_usb2_platform_data *udata;
	stmp3xxx_init();

	i2c_register_board_info(0, stmp3xxx_i2c_devices, ARRAY_SIZE(stmp3xxx_i2c_devices));

	stmp3xxx_set_mmc_data(&stmp3xxx_mmc.dev);
	stmp3xxx_gpmi.dev.platform_data = &gpmi_partitions;
	stmp3xxx_keyboard.dev.platform_data = &keyboard_data;

	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	stmp3xxx_ssp1_device_register();	/* MMC or SSP */
	stmp3xxx_ssp2_device_register();	/* MMC or SSP */
	platform_add_devices(devices, ARRAY_SIZE(devices));
	if (pwm_leds_enable)
		platform_device_register(&stmp378x_leds);
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
