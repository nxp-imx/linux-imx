# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 8M Plus (evk_8mp)

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
# i.MX 8MP Module Lists
# Extracted from device/nxp/imx8m/evk_8mp/SharedBoardConfig.mk
# =============================================================================

# =============================================================================
# i.MX 8MP Device Tree Blobs (DTB/DTBO)
# Extracted from device/nxp/imx8m/evk_8mp/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# =============================================================================

# All DTB files needed for DTBO image generation
# Format: arch/arm64/boot/dts/freescale/<filename>.dtb
_IMX8MP_DTB_OUTS = [
    # Base EVK DTBs
    "arch/arm64/boot/dts/freescale/imx8mp-evk.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-dual-os08a20.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-os08a20-ov5640.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-os08a20.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-dual-basler.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-basler-ov5640.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-basler.dtb",

    # RPMSG support
    "arch/arm64/boot/dts/freescale/imx8mp-evk-rpmsg.dtb",

    # Display interfaces
    "arch/arm64/boot/dts/freescale/imx8mp-evk-it6263-lvds-dual-channel.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-jdi-wuxga-lvds-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-rm67199.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-rm67191.dtb",

    # Audio (SOF)
    "arch/arm64/boot/dts/freescale/imx8mp-evk-sof-wm8960.dtb",

    # RevB4 variants
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-dual-os08a20.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-os08a20-ov5640.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-os08a20.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-dual-basler.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-basler-ov5640.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-basler.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-it6263-lvds-dual-channel.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-jdi-wuxga-lvds-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-rm67199.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-rm67191.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-sof-wm8962.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4-rpmsg.dtb",

    # FRDM board variants
    "arch/arm64/boot/dts/freescale/imx8mp-frdm.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-frdm-os08a20-dual.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-frdm-boe-wuxga-lvds0-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx8mp-frdm-waveshare-7inch-c-panel.dtb",
]

# DTBO overlay files (device tree overlays)
# These are applied on top of base DTB files
_IMX8MP_DTBO_OUTS = [
    "arch/arm64/boot/dts/freescale/imx8mp-evk-revb4.dtbo",
    "arch/arm64/boot/dts/freescale/imx8mp-evk-pcie.dtbo",
    "arch/arm64/boot/dts/freescale/imx8mp-frdm-waveshare-7inch-c-panel.dtbo",
]

# Modules for vendor_dlkm.img (BOARD_VENDOR_KERNEL_MODULES)
# These are loaded later during boot, not required for initial boot
_IMX8MP_VENDOR_DLKM_MODULES = [
    # Wireless
    "net/wireless/cfg80211.ko",
    "net/mac80211/mac80211.ko",

    # GPU
    "drivers/mxc/gpu-viv/galcore.ko",

    # Thermal
    "drivers/thermal/imx8mm_thermal.ko",

    # Reset
    "drivers/reset/reset-imx8mp-audiomix.ko",

    # Firmware
    "drivers/firmware/imx/sm-misc.ko",
    "drivers/firmware/imx/imx-dsp.ko",

    # Audio - PCM/DMA
    "sound/soc/fsl/imx-pcm-dma.ko",
    "sound/soc/fsl/snd-soc-fsl-utils.ko",
    "sound/soc/fsl/snd-soc-fsl-micfil.ko",
    "sound/soc/fsl/snd-soc-fsl-aud2htx.ko",
    "sound/soc/fsl/snd-soc-fsl-asrc.ko",
    "sound/soc/fsl/snd-soc-fsl-easrc.ko",
    "sound/soc/fsl/snd-soc-fsl-sai.ko",

    # HDMI audio
    "drivers/gpu/drm/bridge/synopsys/dw-hdmi-cec.ko",
    "drivers/gpu/drm/bridge/synopsys/dw-hdmi-gp-audio.ko",

    # Audio codecs
    "sound/soc/codecs/snd-soc-hdmi-codec.ko",
    "sound/soc/codecs/snd-soc-wm8960.ko",
    "sound/soc/codecs/snd-soc-bt-sco.ko",
    "sound/soc/codecs/snd-soc-wm8962.ko",

    # Audio cards
    "sound/soc/generic/snd-soc-simple-card-utils.ko",
    "sound/soc/generic/snd-soc-simple-card.ko",
    "sound/soc/fsl/snd-soc-imx-card.ko",
    "sound/soc/fsl/snd-soc-imx-hdmi.ko",
    "sound/soc/fsl/snd-soc-imx-audmux.ko",
    "sound/soc/fsl/snd-soc-fsl-asoc-card.ko",

    # VPU
    "drivers/mxc/vpu/hantrodec/hantro-dec.ko",
    "drivers/mxc/vpu/hantroenc/hantro-enc.ko",
    "drivers/mxc/vpu/memory_usage/memory_usage.ko",
    "drivers/mxc/hantro_v4l2/vsiv4l2.ko",

    # Mailbox & RPMSG
    "drivers/mailbox/imx-mailbox.ko",
    "drivers/rpmsg/rpmsg_ns.ko",
    "drivers/rpmsg/virtio_rpmsg_bus.ko",

    # SCMI CPU/LMM
    "drivers/firmware/imx/sm-cpu.ko",
    "drivers/firmware/imx/sm-lmm.ko",

    # Power sequence
    "drivers/mmc/core/pwrseq_simple.ko",

    # Remoteproc
    "drivers/remoteproc/imx_rproc.ko",
    "drivers/remoteproc/imx_dsp_rproc.ko",

    # RPMSG I2C
    "drivers/i2c/busses/i2c-rpmsg-imx.ko",

    # RPMSG audio
    "sound/soc/fsl/imx-pcm-rpmsg.ko",
    "sound/soc/fsl/snd-soc-fsl-rpmsg.ko",
    "sound/soc/fsl/imx-audio-rpmsg.ko",
    "sound/soc/codecs/snd-soc-rpmsg-wm8960.ko",
    "sound/soc/fsl/snd-soc-imx-pcm512x-rpmsg.ko",
    "sound/soc/fsl/snd-soc-imx-rpmsg.ko",

    # SOF (Sound Open Firmware)
    "sound/soc/sof/snd-sof-utils.ko",
    "sound/soc/sof/snd-sof.ko",
    "sound/soc/sof/snd-sof-of.ko",
    "sound/soc/generic/snd-soc-audio-graph-card2.ko",
    "sound/soc/sof/xtensa/snd-sof-xtensa-dsp.ko",
    "sound/soc/sof/imx/imx-common.ko",
    "sound/soc/sof/imx/snd-sof-imx8.ko",

    # RTC
    "drivers/rtc/rtc-snvs.ko",

    # PCIe
    "drivers/pci/controller/dwc/pci-imx6.ko",

    # Ethernet PHY
    "drivers/net/phy/realtek/realtek.ko",

    # FEC Ethernet
    "drivers/net/ethernet/freescale/fec.ko",

    # STMMAC Ethernet
    "drivers/net/pcs/pcs_xpcs.ko",
    "drivers/net/ethernet/stmicro/stmmac/stmmac.ko",
    "drivers/net/ethernet/stmicro/stmmac/stmmac-platform.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-imx.ko",

    # CAN
    "drivers/net/can/flexcan/flexcan.ko",
]

# Additional modules detected by build (dependencies from imx8mp_gki.fragment)
# These are built due to Kconfig dependencies but not explicitly required.
# Using module_implicit_outs to avoid build errors when modules are not built.
_IMX8MP_IMPLICIT_MODULES = [
    "drivers/pinctrl/freescale/pinctrl-imx8mq.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mm.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mn.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8ulp.ko",
    "drivers/video/fbdev/mxc/mxc_edid.ko",
    "drivers/soc/imx/lpa_ctrl.ko",
    "drivers/trusty/trusty-test.ko",
    "drivers/trusty/trusty-populate.ko",
    "drivers/mfd/wm8994.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-lmm.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-cpu.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "drivers/cpufreq/cpufreq-dt-platdev.ko",
    "drivers/mtd/parsers/ofpart.ko",
    "drivers/mtd/chips/chipreg.ko",
    "drivers/net/mdio/mdio-mux.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-sunxi.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-sun8i.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-sun55i.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-ipq806x.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-qcom-ethqos.ko",
    "drivers/net/ethernet/stmicro/stmmac/dwmac-generic.ko",
    "drivers/gpu/drm/bridge/ite-it6263.ko",
    "sound/soc/fsl/snd-soc-fsl-ssi.ko",
    "sound/soc/fsl/snd-soc-fsl-esai.ko",
    "sound/soc/fsl/snd-soc-fsl-spdif.ko",
    "sound/soc/codecs/snd-soc-tlv320aic31xx.ko",
    "sound/soc/codecs/snd-soc-ak5558.ko",
    "sound/soc/codecs/snd-soc-ak4458.ko",
    "sound/soc/codecs/snd-soc-wm8994.ko",
    "sound/soc/codecs/snd-soc-wm-hubs.ko",
    "sound/soc/codecs/snd-soc-tpa6130a2.ko",
    "sound/soc/codecs/snd-soc-rpmsg-pcm512x.ko",
    "sound/soc/codecs/snd-soc-rpmsg-pcm512x-i2c.ko",
]

# Modules for vendor ramdisk (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# These are required for early boot before vendor_dlkm is mounted
_IMX8MP_VENDOR_RAMDISK_MODULES = [
    # Clock
    "drivers/clk/imx/mxc-clk.ko",
    "drivers/clk/imx/clk-imx8mp.ko",
    "drivers/clk/imx/clk-imx8mp-audiomix.ko",

    # SoC
    "drivers/soc/imx/soc-imx8m.ko",
    "drivers/soc/imx/busfreq-imx8mq.ko",
    "drivers/soc/imx/imx8m_pm_domains.ko",

    # Trusty
    "drivers/trusty/trusty-smc.ko",
    "drivers/trusty/trusty-core.ko",
    "drivers/trusty/trusty-log.ko",
    "drivers/trusty/trusty-ipc.ko",
    "drivers/trusty/trusty-virtio.ko",

    # Power domain
    "drivers/pmdomain/imx/imx8m-blk-ctrl.ko",
    "drivers/pmdomain/imx/imx8mp-blk-ctrl.ko",
    "drivers/pmdomain/imx/gpcv2.ko",
    "drivers/pmdomain/imx/gpcv2-imx.ko",

    # Clocksource & IRQ
    "drivers/clocksource/timer-imx-sysctr.ko",
    "drivers/irqchip/irq-imx-irqsteer.ko",

    # Pinctrl & GPIO
    "drivers/pinctrl/freescale/pinctrl-imx.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mp.ko",
    "drivers/gpio/gpio-mxc.ko",
    "drivers/gpio/gpio-pca953x.ko",
    "drivers/leds/leds-gpio.ko",

    # Serial & Watchdog
    "drivers/tty/serial/imx.ko",
    "drivers/watchdog/imx2_wdt.ko",

    # I2C
    "drivers/i2c/busses/i2c-imx.ko",
    "drivers/i2c/i2c-dev.ko",

    # MTD & SPI
    "drivers/mtd/mtd.ko",
    "drivers/spi/spi-nxp-fspi.ko",
    "drivers/spi/spi-imx.ko",
    "drivers/spi/spidev.ko",
    "drivers/mtd/spi-nor/spi-nor.ko",

    # Regulator & PWM
    "drivers/regulator/pca9450-regulator.ko",
    "drivers/pwm/pwm-imx27.ko",
    "drivers/video/backlight/pwm_bl.ko",

    # MMC
    "drivers/mmc/host/cqhci.ko",
    "drivers/mmc/host/sdhci-esdhc-imx.ko",

    # DMA buffer heaps
    "drivers/dma-buf/heaps/system_heap.ko",
    "drivers/dma-buf/heaps/cma_heap.ko",
    "drivers/dma-buf/heaps/dsp_heap.ko",
    "drivers/dma-buf/dma-buf-imx.ko",

    # Reset
    "drivers/reset/reset-imx7.ko",

    # PHY
    "drivers/phy/freescale/phy-fsl-imx8mp-lvds.ko",
    "drivers/phy/freescale/phy-fsl-samsung-hdmi.ko",
    "drivers/phy/freescale/phy-fsl-imx8mq-usb.ko",
    "drivers/phy/freescale/phy-fsl-imx8m-pcie.ko",

    # Input
    "drivers/input/keyboard/snvs_pwrkey.ko",
    "drivers/input/touchscreen/goodix_ts.ko",
    "drivers/input/touchscreen/synaptics_dsx/synaptics_dsx_i2c.ko",
    "drivers/input/touchscreen/exc3000.ko",

    # Framebuffer
    "drivers/video/fbdev/core/fb.ko",
    "drivers/video/fbdev/core/fb_notify.ko",
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

    # DRM LCDIF
    "drivers/gpu/imx/lcdif/imx8mm-lcdif-core.ko",
    "drivers/gpu/imx/lcdifv3/imx-lcdifv3-core.ko",
    "drivers/gpu/drm/imx/lcdif/imx8mm-lcdif-crtc.ko",
    "drivers/gpu/drm/imx/lcdifv3/imx-lcdifv3-crtc.ko",

    # DRM bridge
    "drivers/gpu/drm/bridge/adv7511/adv7511.ko",
    "drivers/gpu/drm/bridge/cadence/cdns_mhdp_drmcore.ko",
    "drivers/gpu/drm/bridge/fsl-imx-ldb.ko",
    "drivers/gpu/drm/bridge/it6263.ko",
    "drivers/gpu/drm/bridge/sec-dsim.ko",
    "drivers/gpu/drm/bridge/waveshare-dsi.ko",
    "drivers/gpu/drm/bridge/synopsys/dw-hdmi.ko",

    # DRM IMX
    "drivers/gpu/drm/imx/imxdrm.ko",
    "drivers/gpu/drm/imx/mhdp/cdns_mhdp_imx.ko",
    "drivers/gpu/drm/imx/dw_hdmi-imx.ko",
    "drivers/gpu/drm/imx/imx8mp-hdmi-pavi.ko",
    "drivers/gpu/drm/imx/imx8mp-ldb.ko",
    "drivers/gpu/drm/imx/sec_mipi_dsim-imx.ko",

    # DRM panel
    "drivers/gpu/drm/panel/panel-raydium-rm67191.ko",
    "drivers/gpu/drm/panel/panel-simple.ko",

    # USB
    "drivers/usb/dwc3/dwc3-imx8mp.ko",
    "drivers/usb/typec/mux/gpio-switch.ko",

    # Power & NVMEM
    "drivers/power/supply/dummy_battery.ko",
    "drivers/nvmem/nvmem-imx-ocotp.ko",

    # Perf & CPUFreq
    "drivers/perf/fsl_imx8_ddr_perf.ko",
    "drivers/cpufreq/cpufreq-dt.ko",
    "drivers/cpufreq/imx-cpufreq-dt.ko",

    # Media / Camera
    "drivers/media/i2c/ov5640.ko",
    "drivers/staging/media/imx/imx8-capture.ko",
    "drivers/staging/media/imx/imx8-isi-capture.ko",
    "drivers/staging/media/imx/imx8-isi-hw.ko",
    "drivers/staging/media/imx/imx8-isi-mem2mem.ko",
    "drivers/staging/media/imx/imx8-mipi-csi2-sam.ko",
    "drivers/staging/media/imx/imx8-media-dev.ko",

    # Misc
    "lib/stmp_device.ko",
    "drivers/bus/imx-aipstz.ko",
    "drivers/dma/imx-sdma.ko",
    "drivers/dma/mxs-dma.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX8MP_IN_TREE_MODULES = _IMX8MP_VENDOR_RAMDISK_MODULES + _IMX8MP_VENDOR_DLKM_MODULES

# External modules (built separately)
_IMX8MP_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx8mp",
    "//verisilicon_sw_isp_vvcam/vvcam:vvcam_modules_imx8mp",
]

# External modules to include in vendor ramdisk (vendor_boot.img)
# These are loaded after in-tree dependencies are ready
_IMX8MP_EXT_VENDOR_RAMDISK_MODULES = [
    # ISP camera modules
    "vvcam-dwe.ko",
    "vvcam-isp.ko",
    "vvcam-video.ko",
    "os08a20.ko",
    "basler-camera-driver-vvcam.ko",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX8MP_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

# Helper function to generate modules list file content
def _modules_list_content(modules):
    """Generate module list content with just .ko filenames."""
    return "\n".join([m.split("/")[-1] for m in modules]) + "\n"

def define_imx8mp():
    """Define Bazel targets for i.MX 8M Plus kernel build."""

    # ==========================================================================
    # Module list files for initramfs and vendor_dlkm
    # ==========================================================================

    # Modules list for vendor ramdisk (initramfs)
    # Combines in-tree modules + external ISP modules
    write_file(
        name = "imx8mp_vendor_ramdisk_modules_list",
        out = "imx8mp_vendor_ramdisk_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8MP_VENDOR_RAMDISK_MODULES] + _IMX8MP_EXT_VENDOR_RAMDISK_MODULES + [""],
    )

    # Modules list for vendor_dlkm
    # Combines in-tree modules + external WiFi modules
    write_file(
        name = "imx8mp_vendor_dlkm_modules_list",
        out = "imx8mp_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8MP_VENDOR_DLKM_MODULES] + _IMX8MP_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build
    # ==========================================================================

    kernel_build(
        name = "imx8mp",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
        ] + _IMX8MP_DTB_OUTS + _IMX8MP_DTBO_OUTS,
        arch = "arm64",
        # Mixed build: use GKI as base
        base_kernel = ":kernel_aarch64",
        # Use gki_defconfig + imx8mp fragment
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx8mp_gki.fragment",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX8MP_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX8MP_IMPLICIT_MODULES,
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
        name = "imx8mp_abi",
        kernel_build = ":imx8mp",
        kernel_modules = _IMX8MP_EXT_MODULES,
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )

    kernel_modules_install(
        name = "imx8mp_modules_install",
        kernel_build = ":imx8mp",
        kernel_modules = _IMX8MP_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor images
    # ==========================================================================

    # Initramfs for vendor_boot.img
    # Contains only _IMX8MP_VENDOR_RAMDISK_MODULES
    initramfs(
        name = "imx8mp_initramfs",
        kernel_modules_install = ":imx8mp_modules_install",
        ramdisk_compression = "lz4",
        # Only include vendor ramdisk modules
        modules_list = ":imx8mp_vendor_ramdisk_modules_list",
    )

    # vendor_boot.img - contains vendor ramdisk with ramdisk.lz4
    vendor_boot_image(
        name = "imx8mp_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx8mp_initramfs",
        kernel_build = ":imx8mp",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # vendor_dlkm.img - contains only _IMX8MP_VENDOR_DLKM_MODULES
    vendor_dlkm_image(
        name = "imx8mp_vendor_dlkm",
        kernel_modules_install = ":imx8mp_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx8mp_vendor_dlkm_modules_list",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx8mp_initramfs",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules (shared base)
    pkg_files(
        name = "imx8mp_kernel_files",
        srcs = [
            ":imx8mp",
            ":imx8mp_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files for DTBO image generation
    # Directly list DTB/DTBO files from kernel_build outs
    # Note: output_group="kernel_build_outs" is not valid for kernel_build
    # We need to explicitly list the DTB/DTBO output files
    _dtb_srcs = [
        ":imx8mp/" + f for f in _IMX8MP_DTB_OUTS + _IMX8MP_DTBO_OUTS
    ]

    pkg_files(
        name = "imx8mp_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx8mp/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx8mp_vendor_boot_files",
        srcs = [
            ":imx8mp_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM
    pkg_files(
        name = "imx8mp_vendor_dlkm_files",
        srcs = [
            ":imx8mp_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI boot.img variants
    pkg_files(
        name = "imx8mp_boot_files",
        srcs = [
            ":kernel_aarch64_gki_artifacts",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI system_dlkm
    pkg_files(
        name = "imx8mp_system_dlkm_files",
        srcs = [
            ":kernel_aarch64_system_dlkm_image",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # ==========================================================================
    # Distribution targets
    # ==========================================================================

    # Full distribution (all images)
    # Command: tools/bazel run //kernel_imx:imx8mp_dist
    pkg_install(
        name = "imx8mp_dist",
        srcs = [
            ":imx8mp_kernel_files",
            ":imx8mp_vendor_boot_files",
            ":imx8mp_vendor_dlkm_files",
            ":imx8mp_boot_files",
            ":imx8mp_system_dlkm_files",
            ":imx8mp_dtb_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # Vendor boot only (initramfs.img)
    # Command: tools/bazel run //common:imx8mp_vendor_boot_dist
    pkg_install(
        name = "imx8mp_vendor_boot_dist",
        srcs = [
            ":imx8mp_kernel_files",
            ":imx8mp_vendor_boot_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //common:imx8mp_vendor_dlkm_dist
    pkg_install(
        name = "imx8mp_vendor_dlkm_dist",
        srcs = [
            ":imx8mp_kernel_files",
            ":imx8mp_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # GKI boot.img only
    # Command: tools/bazel run //common:imx8mp_boot_dist
    pkg_install(
        name = "imx8mp_boot_dist",
        srcs = [
            ":imx8mp_boot_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # GKI system_dlkm only
    # Command: tools/bazel run //common:imx8mp_system_dlkm_dist
    pkg_install(
        name = "imx8mp_system_dlkm_dist",
        srcs = [
            ":imx8mp_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # GKI combined (boot.img + system_dlkm)
    # Command: tools/bazel run //kernel_imx:imx8mp_gki_dist
    pkg_install(
        name = "imx8mp_gki_dist",
        srcs = [
            ":imx8mp_boot_files",
            ":imx8mp_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

    # DTB only distribution (for DTBO image generation)
    # Command: tools/bazel run //kernel_imx:imx8mp_dtb_dist
    pkg_install(
        name = "imx8mp_dtb_dist",
        srcs = [
            ":imx8mp_dtb_files",
        ],
        destdir = "out/imx_evk_8mp_aarch64/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX8MP_VENDOR_DLKM_MODULES = _IMX8MP_VENDOR_DLKM_MODULES
IMX8MP_VENDOR_RAMDISK_MODULES = _IMX8MP_VENDOR_RAMDISK_MODULES
IMX8MP_IN_TREE_MODULES = _IMX8MP_IN_TREE_MODULES
IMX8MP_IMPLICIT_MODULES = _IMX8MP_IMPLICIT_MODULES
IMX8MP_EXT_MODULES = _IMX8MP_EXT_MODULES
IMX8MP_DTB_OUTS = _IMX8MP_DTB_OUTS
IMX8MP_DTBO_OUTS = _IMX8MP_DTBO_OUTS
