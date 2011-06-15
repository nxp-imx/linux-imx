/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#include <mach/mx6.h>
#include <mach/devices-common.h>

extern const struct imx_imx_uart_1irq_data imx6q_imx_uart_data[] __initconst;
#define imx6q_add_imx_uart(id, pdata)	\
	imx_add_imx_uart_1irq(&imx6q_imx_uart_data[id], pdata)

extern struct platform_device anatop_thermal_device;

extern const struct imx_fec_data imx6q_fec_data __initconst;
#define imx6q_add_fec(pdata)	\
	imx_add_fec(&imx6q_fec_data, pdata)

extern const struct imx_sdhci_esdhc_imx_data
imx6q_sdhci_usdhc_imx_data[] __initconst;
#define imx6q_add_sdhci_usdhc_imx(id, pdata)	\
	imx_add_sdhci_esdhc_imx(&imx6q_sdhci_usdhc_imx_data[id], pdata)

extern const struct imx_spi_imx_data imx6q_ecspi_data[] __initconst;
#define imx6q_add_ecspi(id, pdata)	\
	imx_add_spi_imx(&imx6q_ecspi_data[id], pdata)

extern const struct imx_imx_i2c_data imx6q_imx_i2c_data[] __initconst;
#define imx6q_add_imx_i2c(id, pdata)	\
	imx_add_imx_i2c(&imx6q_imx_i2c_data[id], pdata)
