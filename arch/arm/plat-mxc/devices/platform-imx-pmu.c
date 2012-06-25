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
#include <asm/pmu.h>

static struct resource mx6_pmu_resources[] = {
	[0] = {
		.start	= MXC_INT_CHEETAH_PERFORM,
		.end	= MXC_INT_CHEETAH_PERFORM,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mx6_pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.num_resources	= ARRAY_SIZE(mx6_pmu_resources),
	.resource	= mx6_pmu_resources,
};

void __init imx_add_imx_armpmu()
{
	platform_device_register(&mx6_pmu_device);
}
