/*
 * Copyright 2011 Wolfram Sang <w.sang@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __ASM_ARCH_IMX_ESDHC_H
#define __ASM_ARCH_IMX_ESDHC_H

enum cd_types {
	ESDHC_CD_NONE,          /* no CD, neither controller nor gpio */
	ESDHC_CD_CONTROLLER,    /* mmc controller internal CD */
	ESDHC_CD_GPIO,          /* external gpio pin for CD */
	ESDHC_CD_PERMANENT,     /* no CD, card permanently wired to host */
};

/**
 * struct esdhc_platform_data - optional platform data for esdhc on i.MX
 *
 * strongly recommended for i.MX25/35, not needed for other variants
 *
 * @wp_gpio:	gpio for write_protect (-EINVAL if unused)
 * @cd_gpio:	gpio for card_detect interrupt (-EINVAL if unused)
 */

struct esdhc_platform_data {
	unsigned int wp_gpio;
	unsigned int cd_gpio;
	enum cd_types cd_type;
	unsigned int always_present;
	unsigned int support_18v;
	unsigned int support_8bit;
	unsigned int keep_power_at_suspend;
	unsigned int delay_line;
	bool runtime_pm;
	int (*platform_pad_change)(unsigned int index, int clock);
};
#endif /* __ASM_ARCH_IMX_ESDHC_H */
