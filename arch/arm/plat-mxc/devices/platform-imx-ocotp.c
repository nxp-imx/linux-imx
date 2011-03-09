/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/fsl_devices.h>

#ifdef CONFIG_SOC_IMX50
#define BANK(a, b, c, d, e, f, g, h)   \
	{\
	("HW_OCOTP_"#a), ("HW_OCOTP_"#b), ("HW_OCOTP_"#c), ("HW_OCOTP_"#d), \
	("HW_OCOTP_"#e), ("HW_OCOTP_"#f), ("HW_OCOTP_"#g), ("HW_OCOTP_"#h) \
	}

#define BANKS          (5)
#define BANK_ITEMS     (8)
static const char *bank_reg_desc[BANKS][BANK_ITEMS] = {
	BANK(LOCK, CFG0, CFG1, CFG2, CFG3, CFG4, CFG5, CFG6),
	BANK(MEM0, MEM1, MEM2, MEM3, MEM4, MEM5, GP0, GP1),
	BANK(SCC0, SCC1, SCC2, SCC3, SCC4, SCC5, SCC6, SCC7),
	BANK(SRK0, SRK1, SRK2, SRK3, SRK4, SRK5, SRK6, SRK7),
	BANK(SJC0, SJC1, MAC0, MAC1, HWCAP0, HWCAP1, HWCAP2, SWCAP),
};

static const struct mxc_otp_platform_data imx50_otp_platform_data = {
	.fuse_name = (char **)bank_reg_desc,
	.fuse_num = BANKS * BANK_ITEMS,
	};

const struct imx_otp_data imx50_otp_data = {
	.iobase = MX50_OCOTP_CTRL_BASE_ADDR,
	.pdata = &imx50_otp_platform_data,
};
#undef BANK
#undef BANKS
#undef BANK_ITEMS
#endif /* ifdef CONFIG_SOC_IMX50 */

struct platform_device *__init imx_add_otp(
		const struct imx_otp_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_16 - 1,
			.flags = IORESOURCE_MEM,
		}
	};

	return imx_add_platform_device("imx-ocotp", 0,
			res, ARRAY_SIZE(res), data->pdata,
			sizeof(struct mxc_otp_platform_data));
}

