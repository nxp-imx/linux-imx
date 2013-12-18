/*
 * Copyright (C) 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_mxc_epit_data_entry_single(soc, _id, _hwid, _size)		\
	{								\
		.id = _id,						\
		.iobase = soc ## _EPIT ## _hwid ## _BASE_ADDR,		\
		.iosize = _size,					\
		.irq = soc ## _INT_EPIT ## _hwid,			\
	}
#define imx_mxc_epit_data_entry(soc, _id, _hwid, _size)			\
	[_id] = imx_mxc_epit_data_entry_single(soc, _id, _hwid, _size)

#ifdef CONFIG_SOC_IMX6Q
const struct imx_mxc_epit_data imx6q_mxc_epit_data[] __initconst = {
#define imx6q_mxc_epit_data_entry(_id, _hwid)				\
	imx_mxc_epit_data_entry(MX6Q, _id, _hwid, SZ_16K)
	imx6q_mxc_epit_data_entry(0, 1),
	imx6q_mxc_epit_data_entry(1, 2),
};
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_mxc_epit(
		const struct imx_mxc_epit_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("mxc_epit", data->id,
			res, ARRAY_SIZE(res), NULL, 0);
}
