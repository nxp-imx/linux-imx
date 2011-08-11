/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/mx51.h>
#include <mach/devices-common.h>

extern const struct imx_fec_data imx51_fec_data;
#define imx51_add_fec(pdata)	\
	imx_add_fec(&imx51_fec_data, pdata)

extern const struct imx_imx_i2c_data imx51_imx_i2c_data[];
#define imx51_add_imx_i2c(id, pdata)	\
extern const struct imx_srtc_data imx51_imx_srtc_data __initconst;
#define imx51_add_srtc()	\
	imx_add_srtc(&imx51_imx_srtc_data)

	imx_add_imx_i2c(&imx51_imx_i2c_data[id], pdata)

extern const struct imx_imx_ssi_data imx51_imx_ssi_data[];
#define imx51_add_imx_ssi(id, pdata)	\
	imx_add_imx_ssi(&imx51_imx_ssi_data[id], pdata)

extern const struct imx_imx_uart_1irq_data imx51_imx_uart_data[];
#define imx51_add_imx_uart(id, pdata)	\
	imx_add_imx_uart_1irq(&imx51_imx_uart_data[id], pdata)

extern const struct imx_mxc_nand_data imx51_mxc_nand_data;
#define imx51_add_mxc_nand(pdata)	\
	imx_add_mxc_nand(&imx51_mxc_nand_data, pdata)

extern const struct imx_sdhci_esdhc_imx_data imx51_sdhci_esdhc_imx_data[];
#define imx51_add_sdhci_esdhc_imx(id, pdata)	\
	imx_add_sdhci_esdhc_imx(&imx51_sdhci_esdhc_imx_data[id], pdata)

extern const struct imx_spi_imx_data imx51_cspi_data;
#define imx51_add_cspi(pdata)	\
	imx_add_spi_imx(&imx51_cspi_data, pdata)

extern const struct imx_spi_imx_data imx51_ecspi_data[];
#define imx51_add_ecspi(id, pdata)	\
	imx_add_spi_imx(&imx51_ecspi_data[id], pdata)

extern const struct imx_imx2_wdt_data imx51_imx2_wdt_data[];
#define imx51_add_imx2_wdt(id, pdata)	\
	imx_add_imx2_wdt(&imx51_imx2_wdt_data[id])

extern const struct imx_mxc_pwm_data imx51_mxc_pwm_data[];
#define imx51_add_mxc_pwm(id)	\
	imx_add_mxc_pwm(&imx51_mxc_pwm_data[id])

extern const struct imx_imx_keypad_data imx51_imx_keypad_data;
#define imx51_add_imx_keypad(pdata)	\
	imx_add_imx_keypad(&imx51_imx_keypad_data, pdata)
extern const struct imx_mxc_gpu_data imx51_gpu_data __initconst;
#define imx51_add_mxc_gpu(pdata) \
	imx_add_mxc_gpu(&imx51_gpu_data, pdata)

extern const struct imx_mxc_scc2_data imx51_mxc_scc2_data __initconst;
#define imx51_add_mxc_scc2() \
	imx_add_mxc_scc2(&imx51_mxc_scc2_data)

extern const struct imx_mxc_pwm_data imx51_mxc_pwm_data[] __initconst;
#define imx51_add_mxc_pwm(id)	\
	imx_add_mxc_pwm(&imx51_mxc_pwm_data[id])

#define imx51_add_mxc_pwm_backlight(id, pdata)                 \
	platform_device_register_resndata(NULL, "pwm-backlight",\
			id, NULL, 0, pdata, sizeof(*pdata));

extern const struct imx_ipuv3_data imx51_ipuv3_data __initconst;
#define imx51_add_ipuv3(id, pdata)	imx_add_ipuv3(id, &imx51_ipuv3_data, pdata)
#define imx51_add_ipuv3fb(id, pdata)	imx_add_ipuv3_fb(id, pdata)

extern const struct imx_vpu_data imx51_vpu_data __initconst;
#define imx51_add_vpu()	imx_add_vpu(&imx51_vpu_data)

extern const struct imx_tve_data imx51_tve_data __initconst;
#define imx51_add_tve(pdata)	\
	imx_add_tve(&imx51_tve_data, pdata)

#define imx51_add_lcdif(pdata)	\
	platform_device_register_resndata(NULL, "mxc_lcdif",\
			0, NULL, 0, pdata, sizeof(*pdata));

#define imx51_add_v4l2_output(id)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_output",\
			id, NULL, 0, NULL, 0);

#define imx51_add_v4l2_capture(id)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_capture",\
			id, NULL, 0, NULL, 0);

extern const struct imx_spdif_data imx51_imx_spdif_data __initconst;
#define imx51_add_spdif(pdata)	imx_add_spdif(&imx51_imx_spdif_data, pdata)

extern const struct imx_spdif_dai_data imx51_spdif_dai_data __initconst;
#define imx51_add_spdif_dai()	imx_add_spdif_dai(&imx51_spdif_dai_data)

#define imx51_add_spdif_audio_device(pdata)	imx_add_spdif_audio_device()

extern const struct imx_dvfs_core_data imx51_dvfs_core_data __initconst;
#define imx51_add_dvfs_core(pdata)	\
	imx_add_dvfs_core(&imx51_dvfs_core_data, pdata)

#define imx51_add_busfreq(pdata)	imx_add_busfreq(pdata)

