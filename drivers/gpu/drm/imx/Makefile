# SPDX-License-Identifier: GPL-2.0

imxdrm-objs := imx-drm-core.o

obj-$(CONFIG_DRM_IMX) += imxdrm.o

obj-$(CONFIG_DRM_IMX_LCDIF_MUX_DISPLAY) += lcdif-mux-display.o
obj-$(CONFIG_DRM_IMX_PARALLEL_DISPLAY) += parallel-display.o
obj-$(CONFIG_DRM_IMX_TVE) += imx-tve.o
obj-$(CONFIG_DRM_IMX_LDB) += imx-ldb.o

obj-$(CONFIG_DRM_IMX_DPU) += dpu/
obj-$(CONFIG_DRM_IMX_IPUV3) += ipuv3/
obj-$(CONFIG_DRM_IMX_HDMI) += dw_hdmi-imx.o
obj-$(CONFIG_IMX8MP_HDMI_PAVI) += imx8mp-hdmi-pavi.o
obj-$(CONFIG_DRM_IMX_LCDIF) += lcdif/
obj-$(CONFIG_DRM_IMX_LCDIFV3) += lcdifv3/
obj-$(CONFIG_DRM_IMX_SEC_DSIM) += sec_mipi_dsim-imx.o
obj-$(CONFIG_DRM_IMX_DCSS) += dcss/

cdns_mhdp_imx-objs := cdn-mhdp-imxdrv.o cdn-mhdp-dp-phy.o cdn-mhdp-hdmi-phy.o cdn-mhdp-imx8qm.o cdn-mhdp-ls1028a.o
obj-$(CONFIG_DRM_IMX_CDNS_MHDP) += cdns_mhdp_imx.o
