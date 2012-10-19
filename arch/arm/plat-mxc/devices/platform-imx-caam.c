/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

/*
 * security violation interrupt is used as CAAM base _INT_SNVS_SEC
 * SNVS consolidated = _INT_SNVS
 * JR0 = MXC_INT_CAAM_INT0_NUM
 * JR1 = MXC_INT_CAAM_INT1_NUM
 */

const struct imx_caam_data imx6q_imx_caam_data __initconst = {
	.iobase_caam = MXC_CAAM_BASE_ADDR,
	.iobase_caam_sm = CAAM_SECMEM_BASE_ADDR,
	.iobase_snvs = MX6Q_SNVS_BASE_ADDR,
	.irq_sec_vio = MXC_INT_SNVS_SEC,
	.irq_snvs = MX6Q_INT_SNVS,
	.jr[0].offset_jr = 0x1000,
	.jr[0].irq_jr = MXC_INT_CAAM_INT0_NUM,
	.jr[1].offset_jr = 0x2000,
	.jr[1].irq_jr = MXC_INT_CAAM_INT1_NUM,
};

struct platform_device *__init imx_add_caam(
		const struct imx_caam_data *data)
{
	u32 res_count = 0;
	struct resource res[] = {
		{
			/* Define base range for entire CAAM register map */
			.name = "iobase_caam",
			.start = data->iobase_caam,
			.end = data->iobase_caam + SZ_64K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			/* Define range for secure memory */
			.name = "iobase_caam_sm",
			.start = data->iobase_caam_sm,
			.end = data->iobase_caam_sm + SZ_16K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			/* Define range for SNVS */
			.name = "iobase_snvs",
			.start = data->iobase_snvs,
			.end = data->iobase_snvs + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			/* Define interrupt for security violations */
			.name = "irq_sec_vio",
			.start = data->irq_sec_vio,
			.end = data->irq_sec_vio,
			.flags = IORESOURCE_IRQ,
		}, {
			/* Define general SNVS interrupt */
			.name = "irq_snvs",
			.start = data->irq_snvs,
			.end = data->irq_snvs,
			.flags = IORESOURCE_IRQ,
		}, {
			.name = "offset_jr0",
			.start = data->jr[0].offset_jr,
			.end = data->jr[0].offset_jr,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "irq_jr0",
			.start = data->jr[0].irq_jr,
			.end = data->jr[0].irq_jr,
			.flags = IORESOURCE_IRQ,
		}, {
			.name = "offset_jr1",
			.start = data->jr[1].offset_jr,
			.end = data->jr[1].offset_jr,
			.flags = IORESOURCE_MEM,
		}, {
			.name = "irq_jr1",
			.start = data->jr[1].irq_jr,
			.end = data->jr[1].irq_jr,
			.flags = IORESOURCE_IRQ,
		},
	};

	res_count = ARRAY_SIZE(res);
	BUG_ON(!res_count);

	return imx_add_platform_device("caam", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
