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

extern const struct imx_snvs_rtc_data imx6q_imx_snvs_rtc_data __initconst;
#define imx6q_add_imx_snvs_rtc()	\
	imx_add_snvs_rtc(&imx6q_imx_snvs_rtc_data)

extern const struct imx_anatop_thermal_imx_data
imx6q_anatop_thermal_imx_data __initconst;
#define imx6q_add_anatop_thermal_imx(id, pdata)	\
	imx_add_anatop_thermal_imx(&imx6q_anatop_thermal_imx_data, pdata)

extern const struct imx_dma_data imx6q_dma_data __initconst;
#define imx6q_add_dma()	imx_add_dma(&imx6q_dma_data);

#define imx6q_add_gpmi(platform_data)	imx_add_gpmi(platform_data);

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

extern const struct imx_imx_esai_data imx6q_imx_esai_data[] __initconst;
#define imx6q_add_imx_esai(id, pdata) \
	imx_add_imx_esai(&imx6q_imx_esai_data[id], pdata)

extern const struct imx_viv_gpu_data imx6_gpu_data __initconst;

extern const struct imx_ahci_data imx6q_ahci_data __initconst;
#define imx6q_add_ahci(id, pdata)   \
	imx_add_ahci(&imx6q_ahci_data, pdata)

extern const struct imx_imx_ssi_data imx6_imx_ssi_data[] __initconst;
#define imx6q_add_imx_ssi(id, pdata)            \
	imx_add_imx_ssi(&imx6_imx_ssi_data[id], pdata)

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

extern const struct imx_mxc_hdmi_data imx6q_mxc_hdmi_data __initconst;
#define imx6q_add_mxc_hdmi(pdata)	\
	imx_add_mxc_hdmi(&imx6q_mxc_hdmi_data, pdata)

extern const struct imx_mxc_hdmi_core_data imx6q_mxc_hdmi_core_data __initconst;
#define imx6q_add_mxc_hdmi_core(pdata)	\
	imx_add_mxc_hdmi_core(&imx6q_mxc_hdmi_core_data, pdata)

extern const struct imx_vpu_data imx6q_vpu_data __initconst;
#define imx6q_add_vpu() imx_add_vpu(&imx6q_vpu_data)

extern const struct imx_otp_data imx6q_otp_data __initconst;
#define imx6q_add_otp() \
	imx_add_otp(&imx6q_otp_data)

extern const struct imx_viim_data imx6q_viim_data __initconst;
#define imx6q_add_viim() \
	imx_add_viim(&imx6q_viim_data)

extern const struct imx_imx2_wdt_data imx6q_imx2_wdt_data[] __initconst;
#define imx6q_add_imx2_wdt(id, pdata)   \
	imx_add_imx2_wdt(&imx6q_imx2_wdt_data[id])

extern const struct imx_pm_imx_data imx6q_pm_imx_data __initconst;
#define imx6q_add_pm_imx(id, pdata)	\
	imx_add_pm_imx(&imx6q_pm_imx_data, pdata)

extern const struct imx_imx_asrc_data imx6q_imx_asrc_data[] __initconst;
#define imx6q_add_asrc(pdata)	\
	imx_add_imx_asrc(imx6q_imx_asrc_data, pdata)

extern const struct imx_spi_imx_data imx6q_ecspi_data[] __initconst;
#define imx6q_add_ecspi(id, pdata)      \
	imx_add_spi_imx(&imx6q_ecspi_data[id], pdata)

extern const struct imx_dvfs_core_data imx6q_dvfs_core_data __initconst;
#define imx6q_add_dvfs_core(pdata)	\
	imx_add_dvfs_core(&imx6q_dvfs_core_data, pdata)

extern const struct imx_viv_gpu_data imx6_gc2000_data __initconst;
extern const struct imx_viv_gpu_data imx6_gc320_data __initconst;
extern const struct imx_viv_gpu_data imx6_gc355_data __initconst;

extern const struct imx_mxc_pwm_data imx6q_mxc_pwm_data[] __initconst;
#define imx6q_add_mxc_pwm(id)	\
	imx_add_mxc_pwm(&imx6q_mxc_pwm_data[id])

#define imx6q_add_mxc_pwm_backlight(id, pdata)	   \
	platform_device_register_resndata(NULL, "pwm-backlight",\
			id, NULL, 0, pdata, sizeof(*pdata));

extern const struct imx_spdif_data imx6q_imx_spdif_data __initconst;
#define imx6q_add_spdif(pdata)	imx_add_spdif(&imx6q_imx_spdif_data, pdata)

extern const struct imx_spdif_dai_data imx6q_spdif_dai_data __initconst;
#define imx6q_add_spdif_dai()	imx_add_spdif_dai(&imx6q_spdif_dai_data)

#define imx6q_add_spdif_audio_device(pdata)	imx_add_spdif_audio_device()

#define imx6q_add_hdmi_soc() imx_add_hdmi_soc()
extern const struct imx_hdmi_soc_data imx6q_imx_hdmi_soc_dai_data __initconst;
#define imx6q_add_hdmi_soc_dai() \
	imx_add_hdmi_soc_dai(&imx6q_imx_hdmi_soc_dai_data)

extern const struct imx_mipi_dsi_data imx6q_mipi_dsi_data __initconst;
#define imx6q_add_mipi_dsi(pdata)	\
	imx_add_mipi_dsi(&imx6q_mipi_dsi_data, pdata)

extern const struct imx_flexcan_data imx6q_flexcan_data[] __initconst;
#define imx6q_add_flexcan(id, pdata)	\
	imx_add_flexcan(&imx6q_flexcan_data[id], pdata)
#define imx6q_add_flexcan0(pdata)	imx6q_add_flexcan(0, pdata)
#define imx6q_add_flexcan1(pdata)	imx6q_add_flexcan(1, pdata)

extern const struct imx_mipi_csi2_data imx6q_mipi_csi2_data __initconst;
#define imx6q_add_mipi_csi2(pdata)	\
	imx_add_mipi_csi2(&imx6q_mipi_csi2_data, pdata)
