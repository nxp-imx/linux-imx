/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <mach/mx50.h>
#include <mach/devices-common.h>

extern const struct imx_imx_uart_1irq_data imx50_imx_uart_data[];
#define imx50_add_imx_uart(id, pdata)	\
	imx_add_imx_uart_1irq(&imx50_imx_uart_data[id], pdata)

extern const struct imx_srtc_data imx50_imx_srtc_data __initconst;
#define imx50_add_srtc()	\
	imx_add_srtc(&imx50_imx_srtc_data)

extern const struct imx_dma_data imx50_dma_data __initconst;
#define imx50_add_dma()	imx_add_dma(&imx50_dma_data);

extern const struct imx_fec_data imx50_fec_data;
#define imx50_add_fec(pdata)	\
	imx_add_fec(&imx50_fec_data, pdata)

extern const struct imx_imx_i2c_data imx50_imx_i2c_data[];
#define imx50_add_imx_i2c(id, pdata)	\
	imx_add_imx_i2c(&imx50_imx_i2c_data[id], pdata)
extern const struct imx_mxc_gpu_data imx50_gpu_data __initconst;
extern const struct imx_pxp_data imx50_pxp_data __initconst;
#define imx50_add_imx_pxp()   \
    imx_add_imx_pxp(&imx50_pxp_data)

#define imx50_add_imx_pxp_client()   \
    imx_add_imx_pxp_client()

extern const struct imx_epdc_data imx50_epdc_data __initconst;
#define imx50_add_imx_epdc(pdata)	\
	imx_add_imx_epdc(&imx50_epdc_data, pdata)

#define imx50_add_mxc_gpu(pdata) \
	imx_add_mxc_gpu(&imx50_gpu_data, pdata)
extern const struct imx_sdhci_esdhc_imx_data imx50_sdhci_esdhc_imx_data[] __initconst;
#define imx50_add_sdhci_esdhc_imx(id, pdata)	\
	imx_add_sdhci_esdhc_imx(&imx50_sdhci_esdhc_imx_data[id], pdata)

extern const struct imx_otp_data imx50_otp_data __initconst;
#define imx50_add_otp() \
	imx_add_otp(&imx50_otp_data);

extern const struct imx_viim_data imx50_viim_data  __initconst;
#define imx50_add_viim() \
	imx_add_viim(&imx50_viim_data)

extern const struct imx_dcp_data imx50_dcp_data __initconst;
#define imx50_add_dcp() \
	imx_add_dcp(&imx50_dcp_data);

extern const struct imx_rngb_data imx50_rngb_data __initconst;
#define imx50_add_rngb() \
	imx_add_rngb(&imx50_rngb_data);
extern const struct imx_perfmon_data imx50_perfmon_data __initconst;
#define imx50_add_perfmon() \
	imx_add_perfmon(&imx50_perfmon_data);
#define imx50_add_gpmi(platform_data)	imx_add_gpmi(platform_data);

extern const struct imx_perfmon_data imx50_perfmon_data __initconst;
#define imx50_add_perfmon() \
	imx_add_perfmon(&imx50_perfmon_data);

extern const struct imx_spi_imx_data imx50_cspi_data[] __initconst;
#define imx50_add_cspi(id, pdata)	\
	imx_add_spi_imx(&imx50_cspi_data[id], pdata)

extern const struct imx_dvfs_core_data imx50_dvfs_core_data __initconst;
#define imx50_add_dvfs_core(pdata)	\
	imx_add_dvfs_core(&imx50_dvfs_core_data, pdata)

#define imx50_add_busfreq(pdata)	imx_add_busfreq(pdata)


