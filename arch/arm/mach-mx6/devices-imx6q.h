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

extern const struct imx_anatop_thermal_imx_data
imx6q_anatop_thermal_imx_data __initconst;
#define imx6q_add_anatop_thermal_imx(id, pdata)	\
	imx_add_anatop_thermal_imx(&imx6q_anatop_thermal_imx_data, pdata)

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

extern const struct imx_fsl_usb2_udc_data imx6q_fsl_usb2_udc_data __initconst;
#define imx6q_add_fsl_usb2_udc(pdata)	\
	imx_add_fsl_usb2_udc(&imx6q_fsl_usb2_udc_data, pdata)

extern const struct imx_mxc_ehci_data imx6q_mxc_ehci_otg_data __initconst;
#define imx6q_add_fsl_ehci_otg(pdata)	\
	imx_add_fsl_ehci(&imx6q_mxc_ehci_otg_data, pdata)

extern const struct imx_mxc_ehci_data imx6q_mxc_ehci_hs_data[] __initconst;
#define imx6q_add_fsl_ehci_hs(id, pdata)	\
	imx_add_fsl_ehci(&imx6q_mxc_ehci_hs_data[id - 1], pdata)

extern const struct imx_fsl_usb2_otg_data imx6q_fsl_usb2_otg_data __initconst;
#define imx6q_add_fsl_usb2_otg(pdata)	\
	imx_add_fsl_usb2_otg(&imx6q_fsl_usb2_otg_data, pdata)

extern const struct imx_fsl_usb2_wakeup_data imx6q_fsl_otg_wakeup_data __initconst;
#define imx6q_add_fsl_usb2_otg_wakeup(pdata)	\
	imx_add_fsl_usb2_wakeup(&imx6q_fsl_otg_wakeup_data, pdata)

extern const struct imx_fsl_usb2_wakeup_data imx6q_fsl_hs_wakeup_data[] __initconst;
#define imx6q_add_fsl_usb2_hs_wakeup(id, pdata)	\
	imx_add_fsl_usb2_wakeup(&imx6q_fsl_hs_wakeup_data[id - 1], pdata)

extern const struct imx_viv_gpu_data imx6_gc2000_data __initconst;
extern const struct imx_viv_gpu_data imx6_gc320_data __initconst;
extern const struct imx_viv_gpu_data imx6_gc355_data __initconst;

extern const struct imx_ahci_data imx6q_ahci_data __initconst;
#define imx6q_add_ahci(id, pdata)   \
	imx_add_ahci(&imx6q_ahci_data, pdata)
extern const struct imx_ipuv3_data imx6q_ipuv3_data[] __initconst;
#define imx6q_add_ipuv3(id, pdata)	imx_add_ipuv3(id, &imx6q_ipuv3_data[id], pdata)
#define imx6q_add_ipuv3fb(id, pdata)	imx_add_ipuv3_fb(id, pdata)

#define imx6q_add_lcdif(pdata)	\
	platform_device_register_resndata(NULL, "mxc_lcdif",\
			0, NULL, 0, pdata, sizeof(*pdata));

extern const struct imx_ldb_data imx6q_ldb_data __initconst;
#define imx6q_add_ldb(pdata) \
	imx_add_ldb(&imx6q_ldb_data, pdata);

#define imx6q_add_v4l2_output(id)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_output",\
			id, NULL, 0, NULL, 0);

#define imx6q_add_v4l2_capture(id)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_capture",\
			id, NULL, 0, NULL, 0);
