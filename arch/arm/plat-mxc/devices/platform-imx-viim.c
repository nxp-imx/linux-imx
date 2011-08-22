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

#ifdef CONFIG_SOC_IMX51
#define GPT_REG_BASE_ADDR (MX50_GPT1_BASE_ADDR)
const struct imx_viim_data imx50_viim_data = {
	.iobase = MX50_OCOTP_CTRL_BASE_ADDR,
};
#endif

#ifdef CONFIG_SOC_IMX6Q
#define GPT_REG_BASE_ADDR (GPT_BASE_ADDR)
const struct imx_viim_data imx6q_viim_data = {
	.iobase = OCOTP_BASE_ADDR,
};
#endif

struct platform_device *__init imx_add_viim(
		const struct imx_viim_data *data)
{
	struct resource res[] = {
		[0] = {
			.start = GPT_REG_BASE_ADDR,
			.end   = GPT_REG_BASE_ADDR + PAGE_SIZE - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start  = data->iobase,
			.end    = data->iobase + PAGE_SIZE - 1,
			.flags  = IORESOURCE_MEM,
		},
	};

	return imx_add_platform_device("mxs_viim", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}

