/*
 * Freescale STMP37XX/STMP378X core structure and function declarations
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
#ifndef __ASM_PLAT_STMP3XXX_H
#define __ASM_PLAT_STMP3XXX_H

#include <linux/irq.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>

extern struct sys_timer stmp3xxx_timer;

struct stmpkbd_keypair {
	int raw;
	int kcode;
};

struct stmp37xx_spi_platform_data {
	unsigned irq_pin;

	int (*hw_init)(void *spi);
	int (*hw_release)(void *spi);
};

struct stmp3xxx_persistent_bit_config {
	int reg;
	int start;
	int width;
	const char *name;
};

struct stmp3xxx_platform_persistent_data {
	const struct stmp3xxx_persistent_bit_config *bit_config_tab;
	int bit_config_cnt;
};

#define STMP3XXX_USB_DONT_REMAP 	0x00000001
struct stmp3xxx_usb_platform_data {
	unsigned flags;
	int (*phy_enable)(struct platform_device *);
	void (*hw_init)(void);
	void (*hw_release)(void);
};

void stmp3xxx_init_irq(struct irq_chip *chip);
void stmp3xxx_init(void);
int stmp3xxx_reset_block(void __iomem *hwreg, int just_enable);
extern struct platform_device stmp3xxx_dbguart,
			      stmp3xxx_appuart,
			      stmp3xxx_watchdog,
			      stmp3xxx_touchscreen,
			      stmp3xxx_keyboard,
			      stmp3xxx_gpmi,
			      stmp3xxx_mmc,
			      stmp3xxx_mmc2,
			      stmp3xxx_udc,
			      stmp3xxx_ehci,
			      stmp3xxx_usb,
			      stmp3xxx_rtc,
			      stmp3xxx_spi1,
			      stmp3xxx_spi2,
			      stmp3xxx_backlight,
			      stmp3xxx_rotdec,
			      stmp3xxx_ssp1,
			      stmp3xxx_ssp2,
			      stmp3xxx_dcp,
			      stmp3xxx_dcp_bootstream,
			      stmp3xxx_persistent,
			      stmp3xxx_framebuffer,
			      stmp3xxx_battery,
			      stmp378x_i2c,
			      stmp378x_pxp,
			      stmp378x_audio,
			      stmp3xxx_viim,
			      stmp3xxx_spdif;
#ifdef CONFIG_PM
suspend_state_t stmp37xx_pm_get_target(void);
int stmp37xx_pm_sleep_was_deep(void);
#endif

int stmp3xxx_ssp1_device_register(void);
int stmp3xxx_ssp2_device_register(void);

struct pin_group;
void stmp3xxx_release_pin_group(struct pin_group *pin_group, const char *label);
int stmp3xxx_request_pin_group(struct pin_group *pin_group, const char *label);

int get_evk_board_version(void);

#endif /* __ASM_PLAT_STMP3XXX_H */
