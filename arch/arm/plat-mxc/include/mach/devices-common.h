/*
 * Copyright (C) 2009-2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>

struct platform_device *imx_add_platform_device_dmamask(
		const char *name, int id,
		const struct resource *res, unsigned int num_resources,
		const void *data, size_t size_data, u64 dmamask);

static inline struct platform_device *imx_add_platform_device(
		const char *name, int id,
		const struct resource *res, unsigned int num_resources,
		const void *data, size_t size_data)
{
	return imx_add_platform_device_dmamask(
			name, id, res, num_resources, data, size_data, 0);
}

struct imx_dma_data {
	resource_size_t iobase;
};
struct platform_device *__init imx_add_dma(const struct imx_dma_data *data);

#include <linux/fec.h>
struct imx_fec_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_fec(
		const struct imx_fec_data *data,
		const struct fec_platform_data *pdata);

#include <linux/can/platform/flexcan.h>
struct imx_flexcan_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_flexcan(
		const struct imx_flexcan_data *data,
		const struct flexcan_platform_data *pdata);

#include <linux/fsl_devices.h>
struct imx_fsl_usb2_udc_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_fsl_usb2_udc(
		const struct imx_fsl_usb2_udc_data *data,
		const struct fsl_usb2_platform_data *pdata);

#include <linux/gpio_keys.h>
struct platform_device *__init imx_add_gpio_keys(
		const struct gpio_keys_platform_data *pdata);

#include <mach/mx21-usbhost.h>
struct imx_imx21_hcd_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx21_hcd(
		const struct imx_imx21_hcd_data *data,
		const struct mx21_usbh_platform_data *pdata);

struct imx_imx2_wdt_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
};
struct platform_device *__init imx_add_imx2_wdt(
		const struct imx_imx2_wdt_data *data);

struct imx_imxdi_rtc_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imxdi_rtc(
		const struct imx_imxdi_rtc_data *data);

struct imx_srtc_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_srtc(
		const struct imx_srtc_data *data);

#include <mach/imxfb.h>
struct imx_imx_fb_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx_fb(
		const struct imx_imx_fb_data *data,
		const struct imx_fb_platform_data *pdata);

#include <mach/i2c.h>
struct imx_imx_i2c_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx_i2c(
		const struct imx_imx_i2c_data *data,
		const struct imxi2c_platform_data *pdata);

#include <linux/input/matrix_keypad.h>
struct imx_imx_keypad_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx_keypad(
		const struct imx_imx_keypad_data *data,
		const struct matrix_keymap_data *pdata);

#include <mach/ssi.h>
struct imx_imx_ssi_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
	resource_size_t dmatx0;
	resource_size_t dmarx0;
	resource_size_t dmatx1;
	resource_size_t dmarx1;
};
struct platform_device *__init imx_add_imx_ssi(
		const struct imx_imx_ssi_data *data,
		const struct imx_ssi_platform_data *pdata);

#include <mach/imx-uart.h>
struct imx_imx_uart_3irq_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irqrx;
	resource_size_t irqtx;
	resource_size_t irqrts;
};
struct platform_device *__init imx_add_imx_uart_3irq(
		const struct imx_imx_uart_3irq_data *data,
		const struct imxuart_platform_data *pdata);

struct imx_imx_uart_1irq_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx_uart_1irq(
		const struct imx_imx_uart_1irq_data *data,
		const struct imxuart_platform_data *pdata);

#include <mach/usb.h>
struct imx_imx_udc_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq0;
	resource_size_t irq1;
	resource_size_t irq2;
	resource_size_t irq3;
	resource_size_t irq4;
	resource_size_t irq5;
	resource_size_t irq6;
};
struct platform_device *__init imx_add_imx_udc(
		const struct imx_imx_udc_data *data,
		const struct imxusb_platform_data *pdata);

#include <mach/ipu.h>
#include <mach/mx3fb.h>
#include <mach/mx3_camera.h>
struct imx_ipu_core_data {
	resource_size_t iobase;
	resource_size_t synirq;
	resource_size_t errirq;
};
struct platform_device *__init imx_add_ipu_core(
		const struct imx_ipu_core_data *data,
		const struct ipu_platform_data *pdata);
struct platform_device *__init imx_alloc_mx3_camera(
		const struct imx_ipu_core_data *data,
		const struct mx3_camera_pdata *pdata);
struct platform_device *__init imx_add_mx3_sdc_fb(
		const struct imx_ipu_core_data *data,
		struct mx3fb_platform_data *pdata);

#include <mach/mx1_camera.h>
struct imx_mx1_camera_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_mx1_camera(
		const struct imx_mx1_camera_data *data,
		const struct mx1_camera_pdata *pdata);

#include <mach/mx2_cam.h>
struct imx_mx2_camera_data {
	resource_size_t iobasecsi;
	resource_size_t iosizecsi;
	resource_size_t irqcsi;
	resource_size_t iobaseemmaprp;
	resource_size_t iosizeemmaprp;
	resource_size_t irqemmaprp;
};
struct platform_device *__init imx_add_mx2_camera(
		const struct imx_mx2_camera_data *data,
		const struct mx2_camera_platform_data *pdata);

#include <mach/mxc_ehci.h>
struct imx_mxc_ehci_data {
	int id;
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_mxc_ehci(
		const struct imx_mxc_ehci_data *data,
		const struct mxc_usbh_platform_data *pdata);

#include <mach/mmc.h>
struct imx_mxc_mmc_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
	resource_size_t dmareq;
};
struct platform_device *__init imx_add_mxc_mmc(
		const struct imx_mxc_mmc_data *data,
		const struct imxmmc_platform_data *pdata);

#include <mach/mxc_nand.h>
struct imx_mxc_nand_data {
	/*
	 * id is traditionally 0, but -1 is more appropriate.  We use -1 for new
	 * machines but don't change existing devices as the nand device usually
	 * appears in the kernel command line to pass its partitioning.
	 */
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t axibase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_mxc_nand(
		const struct imx_mxc_nand_data *data,
		const struct mxc_nand_platform_data *pdata);

struct imx_mxc_pwm_data {
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_mxc_pwm(
		const struct imx_mxc_pwm_data *data);

/* mxc_rtc */
struct imx_mxc_rtc_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_mxc_rtc(
		const struct imx_mxc_rtc_data *data);

/* mxc_w1 */
struct imx_mxc_w1_data {
	resource_size_t iobase;
};
struct platform_device *__init imx_add_mxc_w1(
		const struct imx_mxc_w1_data *data);

#include <mach/esdhc.h>
struct imx_sdhci_esdhc_imx_data {
	int id;
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_sdhci_esdhc_imx(
		const struct imx_sdhci_esdhc_imx_data *data,
		const struct esdhc_platform_data *pdata);

#include <mach/spi.h>
struct imx_spi_imx_data {
	const char *devid;
	int id;
	resource_size_t iobase;
	resource_size_t iosize;
	int irq;
};
struct platform_device *__init imx_add_spi_imx(
		const struct imx_spi_imx_data *data,
		const struct spi_imx_master *pdata);

#include <mach/ipu-v3.h>
struct imx_ipuv3_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq_err;
	resource_size_t irq;
	int (*init) (void);
	void (*pg) (int);
};
struct platform_device *__init imx_add_ipuv3(
		const struct imx_ipuv3_data *data,
		struct imx_ipuv3_platform_data *pdata);

#include <mach/mxc_vpu.h>
struct imx_vpu_data {
	resource_size_t iobase;
	resource_size_t irq;
	bool iram_enable;
	int iram_size;
	void (*reset) (void);
	void (*pg) (int);
};
struct platform_device *__init imx_add_vpu(
		const struct imx_vpu_data *data);

#include <mach/mxc_dvfs.h>
struct imx_dvfs_core_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_dvfs_core(
		const struct imx_dvfs_core_data *data,
		const struct mxc_dvfs_platform_data *pdata);
struct platform_device *__init imx_add_busfreq(
		const struct mxc_bus_freq_platform_data *pdata);

#include <linux/fsl_devices.h>
struct imx_tve_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_tve(
		const struct imx_tve_data *data,
		const struct fsl_mxc_tve_platform_data *pdata);

#include <linux/ahci_platform.h>
struct imx_ahci_imx_data {
	int id;
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_ahci_imx(
		const struct imx_ahci_imx_data *data,
		const struct ahci_platform_data *pdata);

struct imx_iim_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_iim(
		const struct imx_iim_data *data,
		const struct mxc_iim_platform_data *pdata);

#include <mach/mxc_gpu.h>
struct imx_mxc_gpu_data {
	resource_size_t irq_2d;
	resource_size_t irq_3d;
	resource_size_t iobase_2d;
	resource_size_t iobase_3d;
	resource_size_t gmem_base;
	resource_size_t gmem_size;
};

struct platform_device *__init imx_add_mxc_gpu(
		const struct imx_mxc_gpu_data *data,
struct imx_perfmon_data {
	resource_size_t iobase;
	struct mxs_platform_perfmon_data *pdata;
};

struct platform_device *__init imx_add_perfmon(
		const struct imx_perfmon_data *data);

struct imx_ldb_data {
	resource_size_t iobase;
	resource_size_t iosize;
};

struct platform_device *__init imx_add_ldb(
	const struct imx_ldb_data *data,
	struct fsl_mxc_ldb_platform_data *pdata);

struct imx_mxc_scc2_data {
	resource_size_t iobase;
	resource_size_t ram_start;
	resource_size_t irq_smn;
	resource_size_t irq_scm;
};

struct platform_device *__init imx_add_mxc_scc2(
		const struct imx_mxc_scc2_data *data);

struct imx_perfmon_data {
	resource_size_t iobase;
	struct mxs_platform_perfmon_data *pdata;
};

struct imx_pxp_data {
    resource_size_t iobase;
    resource_size_t iosize;
    resource_size_t irq;
};
struct platform_device *__init imx_add_imx_pxp(
	const struct imx_pxp_data *data);
struct platform_device *__init imx_add_imx_pxp_client(void);

#include <mach/epdc.h>
struct imx_epdc_data {
	resource_size_t iobase;
	resource_size_t iosize;
	resource_size_t irq;
};
struct platform_device *__init imx_add_imx_epdc(
		const struct imx_epdc_data *data,
		const struct imx_epdc_fb_platform_data *pdata);

struct imx_spdif_data {
	resource_size_t iobase;
	resource_size_t irq;
};
struct platform_device *__init imx_add_spdif(
		const struct imx_spdif_data *data,
		const struct mxc_spdif_platform_data *pdata);

struct imx_spdif_dai_data {
	resource_size_t iobase;
	resource_size_t dmatx;
	resource_size_t dmarx;
};
struct platform_device *__init imx_add_spdif_dai(
		const struct imx_spdif_dai_data *data);

struct platform_device *__init imx_add_spdif_audio_device(void);
struct platform_device *__init imx_add_perfmon(
		const struct imx_perfmon_data *data);
		const struct mxc_gpu_platform_data *pdata);
struct imx_otp_data {
	resource_size_t iobase;
	struct mxc_otp_platform_data *pdata;
};

struct platform_device *__init imx_add_otp(
		const struct imx_otp_data *data);

struct imx_dcp_data {
	resource_size_t iobase;
	resource_size_t irq1;
	resource_size_t irq2;
};

struct platform_device *__init imx_add_dcp(
		const struct imx_dcp_data *data);

/* gpmi-nfc */
#include <mach/gpmi-nfc.h>
struct platform_device *__init imx_add_gpmi(
		const struct gpmi_nfc_platform_data *data);

struct imx_rngb_data {
	resource_size_t iobase;
	resource_size_t irq;
};

struct platform_device *__init imx_add_rngb(
		const struct imx_rngb_data *data);
