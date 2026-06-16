# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 8Q (mek_8q)

load(
    "//build/kernel/kleaf:kernel.bzl",
    "kernel_abi",
    "kernel_build",
    "kernel_modules_install",
    "initramfs",
    "vendor_boot_image",
    "vendor_dlkm_image",
)
load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix")

# =============================================================================
# i.MX 8Q Module Lists
# Extracted from device/nxp/imx8q/mek_8q/SharedBoardConfig.mk
# =============================================================================

# =============================================================================
# i.MX 8Q Device Tree Blobs (DTB/DTBO)
# Extracted from device/nxp/imx8q/mek_8q/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# Conditions: PRODUCT_IMX_CAR=false, TARGET_USE_DYNAMIC_PARTITIONS=true,
#             IMX_NO_PRODUCT_PARTITION=false
# =============================================================================

# All DTB files needed for DTBO image generation
# Format: arch/arm64/boot/dts/freescale/<filename>.dtb
_IMX8Q_DTB_OUTS = [
    # imx8qm standard android; MIPI-HDMI display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-ov5640-dual-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-ov5640-csi0-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-ov5640-csi1-rpmsg.dtb",

    # imx8qm standard android; MIPI panel display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-dsi-rm67199.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-dsi-rm67191.dtb",

    # imx8qm standard android; HDMI display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-hdmi.dtb",

    # imx8qm standard android; LVDS1 panel display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-jdi-wuxga-lvds1-panel.dtb",

    # imx8qm standard android; Multiple display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-md.dtb",

    # imx8qm standard android; SOF
    "arch/arm64/boot/dts/freescale/imx8qm-mek-sof-wm8960.dtb",

    # imx8qm revd variants; MIPI-HDMI display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-ov5640-dual-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-ov5640-csi0-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-ov5640-csi1-rpmsg.dtb",

    # imx8qm revd variants; MIPI panel display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-dsi-rm67199.dtb",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-dsi-rm67191.dtb",

    # imx8qm revd variants; HDMI display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-hdmi.dtb",

    # imx8qm revd variants; Multiple display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-md.dtb",

    # imx8qm revd variants; LVDS1 panel display
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-jdi-wuxga-lvds1-panel.dtb",

    # imx8qm revd variants; SOF
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd-sof-wm8962.dtb",

    # imx8qxp standard android; MIPI-HDMI display
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-ov5640-dual-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-ov5640-csi-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-ov5640-parallel-rpmsg.dtb",

    # imx8qxp standard android; MIPI panel display
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-dsi-rm67199-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-dsi-rm67191-rpmsg.dtb",

    # imx8qxp standard android; LVDS panel display
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-jdi-wuxga-lvds0-panel-rpmsg.dtb",

    # imx8qxp standard android; SOF
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-sof-wm8960.dtb",

    # imx8dx
    "arch/arm64/boot/dts/freescale/imx8dx-mek.dtb",
]

# DTBO overlay files (device tree overlays)
# These are applied on top of base DTB files
_IMX8Q_DTBO_OUTS = [
    "arch/arm64/boot/dts/freescale/imx8qm-mek-ov5640-csi0.dtbo",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-ov5640-csi1.dtbo",
    "arch/arm64/boot/dts/freescale/imx8qm-mek-revd.dtbo",
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-ov5640-csi.dtbo",
    "arch/arm64/boot/dts/freescale/imx8qxp-mek-ov5640-cpi.dtbo",
]

# Modules for vendor ramdisk (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# These are required for early boot before vendor_dlkm is mounted
_IMX8Q_VENDOR_RAMDISK_MODULES = [
    # Mailbox & RPMSG
    "drivers/mailbox/imx-mailbox.ko",
    "drivers/rpmsg/rpmsg_ns.ko",
    "drivers/rpmsg/virtio_rpmsg_bus.ko",

    # Firmware
    "drivers/firmware/imx/imx-scu-firmware.ko",
    "drivers/firmware/imx/sm-cpu.ko",
    "drivers/firmware/imx/sm-lmm.ko",

    # Power domain
    "drivers/pmdomain/imx/scu-pd.ko",

    # Clock
    "drivers/clk/imx/mxc-clk.ko",
    "drivers/clk/imx/clk-imx-scu.ko",
    "drivers/clk/imx/clk-imx-lpcg-scu.ko",
    "drivers/clk/imx/clk-imx-acm.ko",

    # Clocksource & IRQ
    "drivers/irqchip/irq-imx-irqsteer.ko",

    # Pinctrl & GPIO
    "drivers/pinctrl/freescale/pinctrl-imx.ko",
    "drivers/pinctrl/freescale/pinctrl-scu.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8qxp.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8qm.ko",
    "drivers/gpio/gpio-max732x.ko",
    "drivers/gpio/gpio-pca953x.ko",
    "drivers/gpio/gpio-mxc.ko",

    # Power & Reset
    "drivers/power/reset/imx-sm-reset.ko",
    "drivers/reset/reset-imx-scu.ko",

    # IOMMU
    "drivers/iommu/arm/arm-smmu/arm_smmu.ko",

    # CPUFreq
    "drivers/cpufreq/cpufreq-dt.ko",
    "drivers/cpufreq/cpufreq-dt-platdev.ko",

    # Serial
    "drivers/tty/serial/fsl_lpuart.ko",

    # Trusty
    "drivers/trusty/trusty-smc.ko",
    "drivers/trusty/trusty-core.ko",
    "drivers/trusty/trusty-log.ko",
    "drivers/trusty/trusty-ipc.ko",
    "drivers/trusty/trusty-virtio.ko",

    # PWM & Backlight
    "drivers/pwm/pwm-imx27.ko",
    "drivers/video/backlight/pwm_bl.ko",

    # I2C
    "drivers/i2c/busses/i2c-rpmsg-imx.ko",
    "drivers/i2c/busses/i2c-imx-lpi2c.ko",
    "drivers/i2c/i2c-mux.ko",
    "drivers/i2c/muxes/i2c-mux-gpio.ko",

    # SPI & MTD
    "drivers/spi/spi-fsl-lpspi.ko",
    "drivers/spi/spi-nxp-fspi.ko",
    "drivers/spi/spidev.ko",

    # IIO (sensors)
    "drivers/iio/buffer/kfifo_buf.ko",
    "drivers/iio/buffer/industrialio-triggered-buffer.ko",
    "drivers/iio/light/isl29018.ko",
    "drivers/iio/pressure/mpl3115.ko",
    "drivers/iio/gyro/fxas21002c_core.ko",
    "drivers/iio/gyro/fxas21002c_i2c.ko",
    "drivers/iio/imu/fxos8700_core.ko",
    "drivers/iio/imu/fxos8700_i2c.ko",
    "drivers/iio/industrialio-configfs.ko",
    "drivers/iio/industrialio-sw-trigger.ko",
    "drivers/iio/trigger/iio-trig-hrtimer.ko",
    "drivers/iio/trigger/iio-trig-sysfs.ko",
    "drivers/iio/common/st_sensors/st_sensors.ko",
    "drivers/iio/common/st_sensors/st_sensors_i2c.ko",
    "drivers/iio/accel/st_accel.ko",
    "drivers/iio/accel/st_accel_i2c.ko",
    "drivers/iio/magnetometer/st_magn.ko",
    "drivers/iio/magnetometer/st_magn_i2c.ko",
    "drivers/iio/gyro/st_gyro.ko",
    "drivers/iio/gyro/st_gyro_i2c.ko",

    # SoC
    "drivers/soc/imx/busfreq-imx8mq.ko",

    # MMC
    "drivers/mmc/host/cqhci.ko",
    "drivers/mmc/host/sdhci-esdhc-imx.ko",

    # Remoteproc
    "drivers/remoteproc/imx_rproc.ko",

    # USB
    "drivers/usb/typec/mux/gpio-sbu-mux.ko",
    "drivers/usb/phy/phy-mxs-usb.ko",
    "drivers/usb/chipidea/usbmisc_imx.ko",
    "drivers/usb/common/ulpi.ko",
    "drivers/usb/chipidea/ci_hdrc.ko",
    "drivers/usb/chipidea/ci_hdrc_imx.ko",
    "drivers/phy/cadence/phy-cadence-salvo.ko",
    "drivers/usb/cdns3/cdns-usb-common.ko",
    "drivers/usb/cdns3/cdns3-imx.ko",
    "drivers/usb/cdns3/cdns3.ko",

    # DMA buffer heaps
    "drivers/dma-buf/heaps/system_heap.ko",
    "drivers/dma-buf/heaps/dsp_heap.ko",
    "drivers/dma-buf/heaps/cma_heap.ko",
    "drivers/dma-buf/dma-buf-imx.ko",

    # DMA
    "drivers/dma/mxs-dma.ko",

    # Input
    "drivers/input/keyboard/imx_sc_key.ko",

    # Framebuffer
    "drivers/video/fbdev/core/fb.ko",
    "drivers/video/fbdev/core/cfbfillrect.ko",
    "drivers/video/fbdev/core/cfbcopyarea.ko",
    "drivers/video/fbdev/core/cfbimgblt.ko",
    "drivers/video/fbdev/core/fb_io_fops.ko",
    "drivers/video/fbdev/core/fb_sys_fops.ko",
    "drivers/video/fbdev/core/sysimgblt.ko",
    "drivers/video/fbdev/core/syscopyarea.ko",
    "drivers/video/fbdev/core/sysfillrect.ko",
    "drivers/video/logo/linux_logo.ko",

    # DRM core
    "drivers/gpu/drm/clients/drm_client_lib.ko",
    "drivers/gpu/drm/drm_dma_helper.ko",
    "drivers/gpu/drm/drm_fbdev_helper.ko",
    "drivers/gpu/drm/display/drm_display_helper.ko",

    # DRM DPU
    "drivers/gpu/imx/imx8_prg.ko",
    "drivers/gpu/imx/imx8_dprc.ko",
    "drivers/gpu/imx/imx8_pc.ko",
    "drivers/gpu/imx/dpu/imx-dpu-core.ko",
    "drivers/gpu/imx/dpu-blit/imx-dpu-blit.ko",
    "drivers/gpu/drm/imx/dpu/imx-dpu-render.ko",
    "drivers/gpu/drm/imx/dpu/imx-dpu-crtc.ko",

    # DRM IMX
    "drivers/gpu/drm/imx/imxdrm.ko",

    # DRM PHY
    "drivers/phy/phy-mixel-lvds.ko",
    "drivers/phy/phy-mixel-lvds-combo.ko",
    "drivers/phy/freescale/phy-fsl-imx8-mipi-dphy.ko",

    # DRM bridge
    "drivers/gpu/drm/bridge/fsl-imx-ldb.ko",
    "drivers/gpu/drm/bridge/cadence/cdns_mhdp_drmcore.ko",
    "drivers/gpu/drm/bridge/it6263.ko",
    "drivers/gpu/drm/bridge/adv7511/adv7511.ko",
    "drivers/gpu/drm/bridge/nwl-dsi.ko",

    # DRM IMX display
    "drivers/gpu/drm/imx/imx8qxp-ldb-nxp.ko",
    "drivers/gpu/drm/imx/imx8qm-ldb-nxp.ko",
    "drivers/gpu/drm/imx/mhdp/cdns_mhdp_imx.ko",

    # DRM mux
    "drivers/mux/mux-core.ko",
    "drivers/mux/mux-mmio.ko",

    # DRM panel
    "drivers/gpu/drm/panel/panel-simple.ko",
    "drivers/gpu/drm/panel/panel-raydium-rm67191.ko",

    # Camera (GMSL)
    "drivers/staging/media/imx/gmsl-max9286.ko",

    # Misc
    "lib/stmp_device.ko",
]

# Modules for vendor_dlkm.img (BOARD_VENDOR_KERNEL_MODULES)
# These are loaded later during boot, not required for initial boot
_IMX8Q_VENDOR_DLKM_MODULES = [
    # Firmware
    "drivers/firmware/imx/imx-dsp.ko",
    "drivers/firmware/imx/sm-misc.ko",

    # SOF (Sound Open Firmware)
    "sound/soc/sof/snd-sof-utils.ko",
    "sound/soc/sof/snd-sof.ko",
    "sound/soc/sof/snd-sof-of.ko",
    "sound/soc/generic/snd-soc-audio-graph-card2.ko",
    "sound/soc/sof/xtensa/snd-sof-xtensa-dsp.ko",
    "sound/soc/sof/imx/imx-common.ko",
    "sound/soc/sof/imx/snd-sof-imx8.ko",

    # Wireless
    "net/wireless/cfg80211.ko",
    "lib/crypto/libarc4.ko",
    "net/mac80211/mac80211.ko",

    # GPU
    "drivers/mxc/gpu-viv/galcore.ko",

    # Thermal
    "drivers/thermal/imx_sc_thermal.ko",

    # VPU
    "drivers/mxc/vpu/memory_usage/memory_usage.ko",
    "drivers/media/v4l2-core/v4l2-jpeg.ko",
    "drivers/media/platform/nxp/imx-jpeg/mxc-jpeg-encdec.ko",
    "drivers/media/platform/amphion/amphion-vpu.ko",

    # Power
    "drivers/power/supply/dummy_battery.ko",

    # DMA
    "drivers/dma/fsl-edma.ko",

    # Audio - PCM/DMA
    "sound/soc/fsl/imx-pcm-dma.ko",
    "sound/soc/fsl/snd-soc-imx-audmux.ko",
    "sound/soc/fsl/snd-soc-fsl-audmix.ko",
    "sound/soc/fsl/snd-soc-fsl-asrc.ko",
    "sound/soc/fsl/snd-soc-fsl-easrc.ko",
    "sound/soc/fsl/snd-soc-fsl-utils.ko",
    "sound/soc/fsl/snd-soc-fsl-sai.ko",
    "sound/soc/fsl/snd-soc-fsl-esai.ko",

    # Audio codecs
    "sound/soc/codecs/snd-soc-wm8960.ko",
    "sound/soc/codecs/snd-soc-wm8962.ko",
    "sound/soc/codecs/snd-soc-cs42xx8.ko",
    "sound/soc/codecs/snd-soc-cs42xx8-i2c.ko",
    "sound/soc/codecs/snd-soc-bt-sco.ko",
    "sound/soc/codecs/snd-soc-hdmi-codec.ko",

    # Audio cards
    "sound/soc/fsl/snd-soc-fsl-asoc-card.ko",
    "sound/soc/fsl/snd-soc-imx-audmix.ko",
    "sound/soc/generic/snd-soc-simple-card-utils.ko",
    "sound/soc/generic/snd-soc-simple-card.ko",
    "sound/soc/fsl/snd-soc-imx-hdmi.ko",
    "sound/soc/fsl/snd-soc-fsl-spdif.ko",

    # Remoteproc
    "drivers/remoteproc/imx_dsp_rproc.ko",

    # PHY (HSIO)
    "drivers/phy/freescale/phy-fsl-imx8qm-hsio.ko",

    # PCIe
    "drivers/pci/controller/dwc/pci-imx6.ko",

    # Ethernet PHY
    "drivers/net/phy/realtek/realtek.ko",
    "drivers/net/phy/qcom/qcom-phy-lib.ko",
    "drivers/net/phy/qcom/at803x.ko",

    # FEC Ethernet
    "drivers/net/ethernet/freescale/fec.ko",

    # MTD & NAND
    "drivers/mtd/mtd.ko",
    "drivers/mtd/nand/nandcore.ko",
    "drivers/mtd/nand/raw/nand.ko",
    "drivers/mtd/nand/raw/gpmi-nand/gpmi-nand.ko",

    # Perf
    "drivers/perf/fsl_imx8_ddr_perf.ko",

    # ADC
    "drivers/iio/adc/imx8qxp-adc.ko",

    # CAN
    "drivers/net/can/flexcan/flexcan.ko",

    # Watchdog
    "drivers/watchdog/imx_sc_wdt.ko",

    # RTC
    "drivers/rtc/rtc-imx-sc.ko",

    # NVMEM
    "drivers/nvmem/nvmem-imx-ocotp-scu.ko",

    # Security
    "drivers/soc/imx/secvio/soc-imx-secvio-sc.ko",

    # Media / Camera
    "drivers/media/platform/nxp/imx8-isi/imx8-isi.ko",
    "drivers/media/platform/nxp/imx8mq-mipi-csi2.ko",
    "drivers/media/platform/nxp/imx-parallel-csi.ko",
    "drivers/media/i2c/ov5640.ko",
    "drivers/media/platform/nxp/hdmirx/cdns_mhdp_hdmirx.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX8Q_IN_TREE_MODULES = _IMX8Q_VENDOR_RAMDISK_MODULES + _IMX8Q_VENDOR_DLKM_MODULES

# Additional modules detected by build (dependencies from imx8q_gki.fragment)
# These are built due to Kconfig dependencies but not explicitly required.
# Using module_implicit_outs to avoid build errors when modules are not built.
_IMX8Q_IMPLICIT_MODULES = [
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-lmm.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-cpu.ko",
    "drivers/firmware/imx/sec_enclave.ko",
    "drivers/trusty/trusty-test.ko",
    "drivers/trusty/trusty-populate.ko",
    "drivers/soc/imx/soc-imx8m.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mq.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mm.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mn.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mp.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8dxl.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8ulp.ko",
    "drivers/pmdomain/imx/imx8m-blk-ctrl.ko",
    "drivers/pmdomain/imx/imx8mp-blk-ctrl.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "drivers/mfd/wm8994.ko",
    "drivers/video/fbdev/core/fb_notify.ko",
    "drivers/video/fbdev/mxc/mxc_edid.ko",
    "drivers/gpu/drm/bridge/ite-it6263.ko",
    "drivers/nvmem/nvmem-imx-ocotp-ele.ko",
    "drivers/usb/chipidea/ci_hdrc_usb2.ko",
    "drivers/usb/chipidea/ci_hdrc_tegra.ko",
    "drivers/usb/chipidea/ci_hdrc_msm.ko",
    "drivers/usb/chipidea/ci_hdrc_npcm.ko",
    "drivers/iio/gyro/fxas21002c_spi.ko",
    "drivers/iio/gyro/st_gyro_spi.ko",
    "drivers/iio/accel/st_accel_spi.ko",
    "drivers/iio/magnetometer/st_magn_spi.ko",
    "drivers/iio/common/st_sensors/st_sensors_spi.ko",
    "drivers/mtd/chips/chipreg.ko",
    "drivers/mtd/parsers/ofpart.ko",
    "drivers/staging/media/imx/imx8-capture.ko",
    "drivers/staging/media/imx/imx8-isi-capture.ko",
    "drivers/staging/media/imx/imx8-isi-hw.ko",
    "drivers/staging/media/imx/imx8-isi-mem2mem.ko",
    "drivers/staging/media/imx/imx8-media-dev.ko",
    "drivers/staging/media/imx/imx8-mipi-csi2-sam.ko",
    "sound/soc/codecs/snd-soc-wm8994.ko",
    "sound/soc/codecs/snd-soc-wm-hubs.ko",
    "sound/soc/codecs/snd-soc-tlv320aic31xx.ko",
    "sound/soc/fsl/snd-soc-fsl-mqs.ko",
    "sound/soc/fsl/snd-soc-fsl-aud2htx.ko",
    "sound/soc/fsl/snd-soc-fsl-ssi.ko",
]

# External modules (built separately)
_IMX8Q_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx8q",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX8Q_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

def define_imx8q():
    """Define Bazel targets for i.MX 8Q kernel build."""

    # ==========================================================================
    # Module list files for initramfs
    # ==========================================================================

    # Modules list for vendor ramdisk (initramfs)
    write_file(
        name = "imx8q_vendor_ramdisk_modules_list",
        out = "imx8q_vendor_ramdisk_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8Q_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Explicit module load order for vendor ramdisk
    # Matches the order defined in _IMX8Q_VENDOR_RAMDISK_MODULES
    write_file(
        name = "imx8q_modules_load_order",
        out = "imx8q_modules.load",
        content = [m.split("/")[-1] for m in _IMX8Q_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Modules list for vendor_dlkm
    write_file(
        name = "imx8q_vendor_dlkm_modules_list",
        out = "imx8q_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8Q_VENDOR_DLKM_MODULES] + _IMX8Q_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm
    # Matches the order defined in _IMX8Q_VENDOR_DLKM_MODULES
    write_file(
        name = "imx8q_vendor_dlkm_modules_load_order",
        out = "imx8q_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX8Q_VENDOR_DLKM_MODULES] + _IMX8Q_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build
    # ==========================================================================

    kernel_build(
        name = "imx8q",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
        ] + _IMX8Q_DTB_OUTS + _IMX8Q_DTBO_OUTS,
        arch = "arm64",
        # Mixed build: use GKI as base
        # Use 8Q-specific GKI kernel with CONFIG_IMX_GKI_8Q_FIX
        base_kernel = ":kernel_aarch64_8q",
        # Use gki_defconfig + imx8q fragment
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx8q_gki.fragment",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX8Q_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX8Q_IMPLICIT_MODULES,
        # Symbol list for ABI
        kmi_symbol_list = "gki/aarch64/symbols/imx",
        # Collect unstripped modules for debugging
        collect_unstripped_modules = True,
        # Strip modules in release builds
        strip_modules = select({
            "//build/kernel/kleaf:debug_is_true": False,
            "//conditions:default": True,
        }),
        visibility = [
            "//visibility:public",
        ],
    )

    kernel_abi(
        name = "imx8q_abi",
        kernel_build = ":imx8q",
        kernel_modules = _IMX8Q_EXT_MODULES,
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )

    kernel_modules_install(
        name = "imx8q_modules_install",
        kernel_build = ":imx8q",
        kernel_modules = _IMX8Q_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor boot image
    # ==========================================================================

    # Initramfs for vendor_boot.img
    # Contains only _IMX8Q_VENDOR_RAMDISK_MODULES
    initramfs(
        name = "imx8q_initramfs",
        kernel_modules_install = ":imx8q_modules_install",
        ramdisk_compression = "lz4",
        # Only include vendor ramdisk modules
        modules_list = ":imx8q_vendor_ramdisk_modules_list",
        # Explicit load order matching _IMX8Q_VENDOR_RAMDISK_MODULES
        modules_load = ":imx8q_modules_load_order",
        # Remove modules not in modules_list from initramfs
        trim_unused_modules = True,
    )

    # vendor_boot.img - contains vendor ramdisk with ramdisk.lz4
    vendor_boot_image(
        name = "imx8q_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx8q_initramfs",
        kernel_build = ":imx8q",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # vendor_dlkm.img - contains only _IMX8Q_VENDOR_DLKM_MODULES
    vendor_dlkm_image(
        name = "imx8q_vendor_dlkm",
        kernel_modules_install = ":imx8q_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx8q_vendor_dlkm_modules_list",
        # Explicit load order matching _IMX8Q_VENDOR_DLKM_MODULES
        modules_load = ":imx8q_vendor_dlkm_modules_load_order",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx8q_initramfs",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules (shared base)
    pkg_files(
        name = "imx8q_kernel_files",
        srcs = [
            ":imx8q",
            ":imx8q_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx8q_vendor_boot_files",
        srcs = [
            ":imx8q_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM
    pkg_files(
        name = "imx8q_vendor_dlkm_files",
        srcs = [
            ":imx8q_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI boot.img variants
    pkg_files(
        name = "imx8q_boot_files",
        srcs = [
            ":kernel_aarch64_8q_gki_artifacts",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files for DTBO image generation
    _dtb_srcs = [
        ":imx8q/" + f for f in _IMX8Q_DTB_OUTS + _IMX8Q_DTBO_OUTS
    ]

    pkg_files(
        name = "imx8q_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx8q/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # GKI system_dlkm
    pkg_files(
        name = "imx8q_system_dlkm_files",
        srcs = [
            ":kernel_aarch64_8q_system_dlkm_image",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # ==========================================================================
    # Distribution targets
    # ==========================================================================

    # Full distribution (all images)
    # Command: tools/bazel run //kernel_imx:imx8q_dist
    pkg_install(
        name = "imx8q_dist",
        srcs = [
            ":imx8q_kernel_files",
            ":imx8q_vendor_boot_files",
            ":imx8q_vendor_dlkm_files",
            ":imx8q_boot_files",
            ":imx8q_system_dlkm_files",
            ":imx8q_dtb_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # Vendor boot only (initramfs.img)
    # Command: tools/bazel run //kernel_imx:imx8q_vendor_boot_dist
    pkg_install(
        name = "imx8q_vendor_boot_dist",
        srcs = [
            ":imx8q_kernel_files",
            ":imx8q_vendor_boot_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //kernel_imx:imx8q_vendor_dlkm_dist
    pkg_install(
        name = "imx8q_vendor_dlkm_dist",
        srcs = [
            ":imx8q_kernel_files",
            ":imx8q_vendor_dlkm_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # GKI boot.img only
    # Command: tools/bazel run //kernel_imx:imx8q_boot_dist
    pkg_install(
        name = "imx8q_boot_dist",
        srcs = [
            ":imx8q_boot_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # GKI system_dlkm only
    # Command: tools/bazel run //kernel_imx:imx8q_system_dlkm_dist
    pkg_install(
        name = "imx8q_system_dlkm_dist",
        srcs = [
            ":imx8q_system_dlkm_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # DTB only distribution (for DTBO image generation)
    # Command: tools/bazel run //kernel_imx:imx8q_dtb_dist
    pkg_install(
        name = "imx8q_dtb_dist",
        srcs = [
            ":imx8q_dtb_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

    # GKI combined (boot.img + system_dlkm)
    # Command: tools/bazel run //kernel_imx:imx8q_gki_dist
    pkg_install(
        name = "imx8q_gki_dist",
        srcs = [
            ":imx8q_boot_files",
            ":imx8q_system_dlkm_files",
        ],
        destdir = "out/imx_mek_8q_aarch64/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX8Q_VENDOR_DLKM_MODULES = _IMX8Q_VENDOR_DLKM_MODULES
IMX8Q_VENDOR_RAMDISK_MODULES = _IMX8Q_VENDOR_RAMDISK_MODULES
IMX8Q_IN_TREE_MODULES = _IMX8Q_IN_TREE_MODULES
IMX8Q_IMPLICIT_MODULES = _IMX8Q_IMPLICIT_MODULES
IMX8Q_DTB_OUTS = _IMX8Q_DTB_OUTS
IMX8Q_DTBO_OUTS = _IMX8Q_DTBO_OUTS
IMX8Q_EXT_MODULES = _IMX8Q_EXT_MODULES
