/*
 * Copyright (C) 2010 Yong Shen. <Yong.Shen@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/mx53.h>
#include <mach/devices-common.h>

extern const struct imx_fec_data imx53_fec_data;
#define imx53_add_fec(pdata)   \
	imx_add_fec(&imx53_fec_data, pdata)

extern const struct imx_imx_uart_1irq_data imx53_imx_uart_data[];
#define imx53_add_imx_uart(id, pdata)	\
	imx_add_imx_uart_1irq(&imx53_imx_uart_data[id], pdata)


extern const struct imx_imx_i2c_data imx53_imx_i2c_data[];
#define imx53_add_imx_i2c(id, pdata)	\
	imx_add_imx_i2c(&imx53_imx_i2c_data[id], pdata)

extern const struct imx_sdhci_esdhc_imx_data imx53_sdhci_esdhc_imx_data[];
#define imx53_add_sdhci_esdhc_imx(id, pdata)	\
	imx_add_sdhci_esdhc_imx(&imx53_sdhci_esdhc_imx_data[id], pdata)

extern const struct imx_spi_imx_data imx53_ecspi_data[];
#define imx53_add_ecspi(id, pdata)	\
	imx_add_spi_imx(&imx53_ecspi_data[id], pdata)

extern const struct imx_imx2_wdt_data imx53_imx2_wdt_data[];
#define imx53_add_imx2_wdt(id, pdata)	\
	imx_add_imx2_wdt(&imx53_imx2_wdt_data[id])

extern const struct imx_mxc_pwm_data imx53_mxc_pwm_data[] __initconst;
#define imx53_add_mxc_pwm(id)	\
	imx_add_mxc_pwm(&imx53_mxc_pwm_data[id])

#define imx53_add_mxc_pwm_backlight(id, pdata)                 \
	platform_device_register_resndata(NULL, "pwm-backlight",\
			id, NULL, 0, pdata, sizeof(*pdata));

extern const struct imx_ipuv3_data imx53_ipuv3_data __initconst;
#define imx53_add_ipuv3(id, pdata)	imx_add_ipuv3(id, &imx53_ipuv3_data, pdata)
#define imx53_add_ipuv3fb(id, pdata)	imx_add_ipuv3_fb(id, pdata)

extern const struct imx_vpu_data imx53_vpu_data __initconst;
#define imx53_add_vpu()	imx_add_vpu(&imx53_vpu_data)

extern const struct imx_imx_asrc_data imx53_imx_asrc_data[] __initconst;
#define imx53_add_asrc(pdata)	\
	imx_add_imx_asrc(imx53_imx_asrc_data, pdata)

extern const struct imx_tve_data imx53_tve_data __initconst;
#define imx53_add_tve(pdata)	\
	imx_add_tve(&imx53_tve_data, pdata)

extern const struct imx_dvfs_core_data imx53_dvfs_core_data __initconst;
#define imx53_add_dvfs_core(pdata)	\
	imx_add_dvfs_core(&imx53_dvfs_core_data, pdata)

#define imx53_add_busfreq(pdata)	imx_add_busfreq(pdata)

extern const struct imx_srtc_data imx53_imx_srtc_data __initconst;
#define imx53_add_srtc()	\
	imx_add_srtc(&imx53_imx_srtc_data)

#define imx53_add_lcdif(pdata)	\
	platform_device_register_resndata(NULL, "mxc_lcdif",\
			0, NULL, 0, pdata, sizeof(*pdata));

#define imx53_add_v4l2_output(id)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_output",\
			id, NULL, 0, NULL, 0);

#define imx53_add_v4l2_capture(id, pdata)	\
	platform_device_register_resndata(NULL, "mxc_v4l2_capture",\
			id, NULL, 0, pdata, sizeof(*pdata));

extern const struct imx_ahci_data imx53_ahci_data[] __initconst;
#define imx53_add_ahci(id, pdata)		\
	imx_add_ahci(&imx53_ahci_data[id], pdata)

extern const struct imx_imx_ssi_data imx53_imx_ssi_data[] __initconst;
#define imx53_add_imx_ssi(id, pdata)		\
	imx_add_imx_ssi(&imx53_imx_ssi_data[id], pdata)

extern const struct imx_iim_data imx53_imx_iim_data __initconst;
#define imx53_add_iim(pdata) \
	imx_add_iim(&imx53_imx_iim_data, pdata)

extern struct imx_mxc_gpu_data imx53_gpu_data __initconst;
#define imx53_add_mxc_gpu(pdata) \
	imx_add_mxc_gpu(&imx53_gpu_data, pdata)

extern const struct imx_ldb_data imx53_ldb_data __initconst;
#define imx53_add_ldb(pdata) \
	imx_add_ldb(&imx53_ldb_data, pdata);

extern const struct imx_mxc_scc2_data imx53_mxc_scc2_data __initconst;
#define imx53_add_mxc_scc2() \
	imx_add_mxc_scc2(&imx53_mxc_scc2_data)

extern const struct imx_spdif_data imx53_imx_spdif_data __initconst;
#define imx53_add_spdif(pdata)	imx_add_spdif(&imx53_imx_spdif_data, pdata)

extern const struct imx_spdif_dai_data imx53_spdif_dai_data __initconst;
#define imx53_add_spdif_dai()	imx_add_spdif_dai(&imx53_spdif_dai_data)

#define imx53_add_spdif_audio_device(pdata)	imx_add_spdif_audio_device()
extern const struct imx_imx_esai_data imx53_imx_esai_data[] __initconst;
#define imx53_add_imx_esai(id, pdata) \
       imx_add_imx_esai(&imx53_imx_esai_data[id], pdata)

extern struct platform_device imx_ahci_device_hwmon;

#define imx53_add_ion(id, pdata, size)	\
	platform_device_register_resndata(NULL, "ion-mxc",\
			id, NULL, 0, pdata, size);

