# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 8M Quad (evk_8mq)

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
# i.MX 8M Quad Device Tree Blobs (DTB)
# Extracted from device/nxp/imx8m/evk_8mq/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# Conditions: TARGET_USE_DYNAMIC_PARTITIONS=true, IMX_NO_PRODUCT_PARTITION=false
# =============================================================================

# All DTB files needed for DTBO image generation
_IMX8MQ_DTB_OUTS = [
    # imx8mq GKI with HDMI display (PCIe M.2)
    "arch/arm64/boot/dts/freescale/imx8mq-evk-pcie1-m2-gki.dtb",

    # imx8mq GKI with SDIO WiFi/BT module support
    "arch/arm64/boot/dts/freescale/imx8mq-evk-usdhc2-m2-gki.dtb",

    # imx8mq GKI with HDMI display on WEVK board
    "arch/arm64/boot/dts/freescale/imx8mq-evk-gki.dtb",

    # imx8mq GKI with MIPI-HDMI display
    "arch/arm64/boot/dts/freescale/imx8mq-evk-lcdif-adv7535-gki.dtb",

    # imx8mq GKI with HDMI and MIPI-HDMI display
    "arch/arm64/boot/dts/freescale/imx8mq-evk-dual-display-gki.dtb",

    # imx8mq GKI with rm67199 MIPI panel display
    "arch/arm64/boot/dts/freescale/imx8mq-evk-dcss-rm67199-gki.dtb",

    # imx8mq GKI with rm67191 MIPI panel display
    "arch/arm64/boot/dts/freescale/imx8mq-evk-dcss-rm67191-gki.dtb",
]

# =============================================================================
# i.MX 8M Quad Module Lists
# Extracted from device/nxp/imx8m/evk_8mq/SharedBoardConfig.mk
# =============================================================================

# Modules for vendor ramdisk (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# These are required for early boot before vendor_dlkm is mounted
_IMX8MQ_VENDOR_RAMDISK_MODULES = [
    # Clock
    "drivers/clk/imx/mxc-clk.ko",
    "drivers/clk/imx/clk-imx8mq.ko",

    # SoC
    "drivers/soc/imx/soc-imx8m.ko",

    # Power domain
    "drivers/pmdomain/imx/gpcv2.ko",
    "drivers/pmdomain/imx/gpcv2-imx.ko",
    "drivers/pmdomain/imx/imx8m-blk-ctrl.ko",

    # Clocksource
    "drivers/clocksource/timer-imx-sysctr.ko",

    # SoC bus freq
    "drivers/soc/imx/busfreq-imx8mq.ko",

    # Trusty
    "drivers/trusty/trusty-smc.ko",
    "drivers/trusty/trusty-core.ko",
    "drivers/trusty/trusty-log.ko",
    "drivers/trusty/trusty-ipc.ko",
    "drivers/trusty/trusty-virtio.ko",

    # IRQ
    "drivers/irqchip/irq-imx-irqsteer.ko",

    # Pinctrl
    "drivers/pinctrl/freescale/pinctrl-imx.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mq.ko",

    # Serial
    "drivers/tty/serial/imx.ko",

    # Watchdog
    "drivers/watchdog/imx2_wdt.ko",

    # Regulator
    "drivers/regulator/pca9450-regulator.ko",
    "drivers/regulator/gpio-regulator.ko",
    "drivers/regulator/pfuze100-regulator.ko",

    # GPIO
    "drivers/gpio/gpio-mxc.ko",

    # Perf
    "drivers/perf/fsl_imx8_ddr_perf.ko",

    # CPUFreq
    "drivers/cpufreq/cpufreq-dt.ko",
    "drivers/cpufreq/imx-cpufreq-dt.ko",

    # NVMEM
    "drivers/nvmem/nvmem-imx-ocotp.ko",

    # PWM
    "drivers/pwm/pwm-imx27.ko",

    # MMC power sequence
    "drivers/mmc/core/pwrseq_simple.ko",

    # MMC
    "drivers/mmc/host/sdhci-esdhc-imx.ko",
    "drivers/mmc/host/cqhci.ko",

    # I2C
    "drivers/i2c/busses/i2c-imx.ko",
    "drivers/i2c/i2c-dev.ko",

    # MTD
    "drivers/mtd/mtd.ko",

    # SPI
    "drivers/spi/spi-fsl-qspi.ko",
    "drivers/spi/spi-bitbang.ko",
    "drivers/spi/spi-imx.ko",

    # MTD SPI-NOR
    "drivers/mtd/spi-nor/spi-nor.ko",

    # Misc
    "lib/stmp_device.ko",

    # DMA
    "drivers/dma/mxs-dma.ko",
    "drivers/dma/imx-sdma.ko",

    # Mailbox
    "drivers/mailbox/imx-mailbox.ko",

    # DMA buffer heaps
    "drivers/dma-buf/heaps/system_heap.ko",
    "drivers/dma-buf/heaps/cma_heap.ko",
    "drivers/dma-buf/dma-buf-imx.ko",

    # Reset
    "drivers/reset/reset-imx7.ko",

    # PHY
    "drivers/phy/freescale/phy-fsl-imx8-mipi-dphy.ko",
    "drivers/phy/freescale/phy-fsl-imx8mq-usb.ko",

    # Input
    "drivers/input/keyboard/snvs_pwrkey.ko",
    "drivers/input/touchscreen/goodix_ts.ko",
    "drivers/input/touchscreen/synaptics_dsx/synaptics_dsx_i2c.ko",

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
    "drivers/gpu/drm/panel/panel-raydium-rm67191.ko",
    "drivers/gpu/drm/imx/dcss/imx-dcss.ko",
    "drivers/gpu/imx/lcdif/imx8mm-lcdif-core.ko",
    "drivers/gpu/drm/drm_dma_helper.ko",
    "drivers/gpu/drm/drm_fbdev_helper.ko",
    "drivers/gpu/drm/display/drm_display_helper.ko",

    # DRM bridge
    "drivers/gpu/drm/bridge/adv7511/adv7511.ko",
    "drivers/gpu/drm/bridge/cadence/cdns_mhdp_drmcore.ko",
    "drivers/gpu/drm/imx/mhdp/cdns_mhdp_imx.ko",

    # DRM IMX
    "drivers/gpu/drm/imx/imxdrm.ko",
    "drivers/gpu/drm/imx/lcdif/imx8mm-lcdif-crtc.ko",
    "drivers/gpu/drm/mxsfb/mxsfb.ko",
    "drivers/gpu/drm/bridge/nwl-dsi.ko",

    # DRM mux
    "drivers/mux/mux-mmio.ko",
    "drivers/mux/mux-core.ko",

    # USB
    "drivers/usb/dwc3/dwc3-imx8mp.ko",
    "drivers/usb/typec/mux/gpio-switch.ko",

    # Power
    "drivers/power/supply/dummy_battery.ko",

    # IR
    "drivers/media/rc/gpio-ir-recv.ko",
]

# Modules for vendor_dlkm.img (BOARD_VENDOR_KERNEL_MODULES)
# These are loaded later during boot, not required for initial boot
_IMX8MQ_VENDOR_DLKM_MODULES = [
    # Wireless
    "net/wireless/cfg80211.ko",
    "lib/crypto/libarc4.ko",
    "net/mac80211/mac80211.ko",

    # GPU
    "drivers/mxc/gpu-viv/galcore.ko",

    # Thermal
    "drivers/thermal/qoriq_thermal.ko",

    # Media / Camera
    "drivers/media/platform/nxp/imx7-media-csi.ko",
    "drivers/media/platform/nxp/imx8mq-mipi-csi2.ko",
    "drivers/media/i2c/ov5640.ko",

    # Firmware
    "drivers/firmware/imx/sm-misc.ko",

    # Audio - PCM/DMA
    "sound/soc/fsl/imx-pcm-dma.ko",
    "sound/soc/fsl/snd-soc-fsl-aud2htx.ko",
    "sound/soc/fsl/snd-soc-fsl-utils.ko",
    "sound/soc/fsl/snd-soc-fsl-sai.ko",
    "sound/soc/codecs/snd-soc-hdmi-codec.ko",
    "sound/soc/fsl/snd-soc-imx-hdmi.ko",
    "sound/soc/fsl/snd-soc-fsl-spdif.ko",
    "sound/soc/fsl/snd-soc-imx-audmux.ko",
    "sound/soc/fsl/snd-soc-fsl-asoc-card.ko",
    "sound/soc/fsl/snd-soc-imx-card.ko",

    # Audio codecs
    "sound/soc/codecs/snd-soc-wm8524.ko",
    "sound/soc/codecs/snd-soc-ak5558.ko",
    "sound/soc/codecs/snd-soc-ak4458.ko",
    "sound/soc/codecs/snd-soc-bt-sco.ko",

    # Audio cards
    "sound/soc/generic/snd-soc-simple-card.ko",
    "sound/soc/generic/snd-soc-simple-card-utils.ko",

    # VPU
    "drivers/mxc/vpu/hantrodec/hantro-dec.ko",
    "drivers/mxc/vpu/memory_usage/memory_usage.ko",
    "drivers/mxc/hantro_v4l2/vsiv4l2.ko",

    # RTC
    "drivers/rtc/rtc-snvs.ko",

    # PCIe
    "drivers/pci/controller/dwc/pci-imx6.ko",

    # Ethernet PHY
    "drivers/net/phy/qcom/qcom-phy-lib.ko",
    "drivers/net/phy/qcom/at803x.ko",

    # FEC Ethernet
    "drivers/net/ethernet/freescale/fec.ko",
]

# Additional modules detected by build (dependencies from imx8mq_gki.fragment)
# These are built due to Kconfig dependencies but not explicitly required.
# Using module_implicit_outs to avoid build errors when modules are not built.
_IMX8MQ_IMPLICIT_MODULES = [
    "drivers/gpu/drm/mxsfb/imx-lcdif.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mm.ko",
    "sound/soc/codecs/snd-soc-tlv320aic31xx.ko",
    "drivers/cpufreq/cpufreq-dt-platdev.ko",
    "drivers/trusty/trusty-populate.ko",
    "drivers/rpmsg/imx_rpmsg.ko",
    "drivers/rpmsg/imx_rpmsg_chre.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8ulp.ko",
    "drivers/video/fbdev/mxc/mxc_edid.ko",
    "drivers/video/fbdev/core/fb_notify.ko",
    "drivers/pmdomain/imx/imx8mp-blk-ctrl.ko",
    "drivers/rpmsg/imx_rpmsg_pingpong.ko",
    "drivers/rpmsg/rpmsg_ns.ko",
    "sound/soc/fsl/snd-soc-fsl-esai.ko",
    "drivers/rpmsg/virtio_rpmsg_bus.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mp.ko",
    "drivers/mxc/vpu/hantroenc/hantro-enc.ko",
    "drivers/mfd/wm8994.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mn.ko",
    "sound/soc/fsl/snd-soc-fsl-ssi.ko",
    "drivers/mtd/parsers/ofpart.ko",
    "drivers/trusty/trusty-test.ko",
    "drivers/mtd/chips/chipreg.ko",
    "sound/soc/codecs/snd-soc-wm8994.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "sound/soc/codecs/snd-soc-wm-hubs.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX8MQ_IN_TREE_MODULES = _IMX8MQ_VENDOR_RAMDISK_MODULES + _IMX8MQ_VENDOR_DLKM_MODULES

# External modules (built separately)
_IMX8MQ_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx8mq",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX8MQ_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

def define_imx8mq():
    """Define Bazel targets for i.MX 8M Quad kernel build."""

    # ==========================================================================
    # Module list files for initramfs
    # ==========================================================================

    # Modules list for vendor ramdisk (initramfs)
    write_file(
        name = "imx8mq_vendor_ramdisk_modules_list",
        out = "imx8mq_vendor_ramdisk_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8MQ_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Explicit module load order for vendor ramdisk
    # Matches the order defined in _IMX8MQ_VENDOR_RAMDISK_MODULES
    write_file(
        name = "imx8mq_modules_load_order",
        out = "imx8mq_modules.load",
        content = [m.split("/")[-1] for m in _IMX8MQ_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Modules list for vendor_dlkm
    write_file(
        name = "imx8mq_vendor_dlkm_modules_list",
        out = "imx8mq_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8MQ_VENDOR_DLKM_MODULES] + _IMX8MQ_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm
    # Matches the order defined in _IMX8MQ_VENDOR_DLKM_MODULES
    write_file(
        name = "imx8mq_vendor_dlkm_modules_load_order",
        out = "imx8mq_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX8MQ_VENDOR_DLKM_MODULES] + _IMX8MQ_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build
    # ==========================================================================

    kernel_build(
        name = "imx8mq",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
        ] + _IMX8MQ_DTB_OUTS,
        arch = "arm64",
        # Mixed build: use GKI as base
        base_kernel = ":kernel_aarch64",
        # Use gki_defconfig + imx8mq fragment
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx8mq_gki.fragment",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX8MQ_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX8MQ_IMPLICIT_MODULES,
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
        name = "imx8mq_abi",
        kernel_build = ":imx8mq",
        kernel_modules = _IMX8MQ_EXT_MODULES,
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )

    kernel_modules_install(
        name = "imx8mq_modules_install",
        kernel_build = ":imx8mq",
        kernel_modules = _IMX8MQ_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor boot image
    # ==========================================================================

    # Initramfs for vendor_boot.img
    # Contains only _IMX8MQ_VENDOR_RAMDISK_MODULES
    initramfs(
        name = "imx8mq_initramfs",
        kernel_modules_install = ":imx8mq_modules_install",
        ramdisk_compression = "lz4",
        # Only include vendor ramdisk modules
        modules_list = ":imx8mq_vendor_ramdisk_modules_list",
        # Explicit load order matching _IMX8MQ_VENDOR_RAMDISK_MODULES
        modules_load = ":imx8mq_modules_load_order",
        # Remove modules not in modules_list from initramfs
        trim_unused_modules = True,
    )

    # vendor_boot.img - contains vendor ramdisk with ramdisk.lz4
    vendor_boot_image(
        name = "imx8mq_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx8mq_initramfs",
        kernel_build = ":imx8mq",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # vendor_dlkm.img - contains only _IMX8MQ_VENDOR_DLKM_MODULES
    vendor_dlkm_image(
        name = "imx8mq_vendor_dlkm",
        kernel_modules_install = ":imx8mq_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx8mq_vendor_dlkm_modules_list",
        # Explicit load order matching _IMX8MQ_VENDOR_DLKM_MODULES
        modules_load = ":imx8mq_vendor_dlkm_modules_load_order",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx8mq_initramfs",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules (shared base)
    pkg_files(
        name = "imx8mq_kernel_files",
        srcs = [
            ":imx8mq",
            ":imx8mq_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx8mq_vendor_boot_files",
        srcs = [
            ":imx8mq_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM
    pkg_files(
        name = "imx8mq_vendor_dlkm_files",
        srcs = [
            ":imx8mq_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files
    _dtb_srcs = [
        ":imx8mq/" + f for f in _IMX8MQ_DTB_OUTS
    ]

    pkg_files(
        name = "imx8mq_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx8mq/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # GKI boot.img variants
    pkg_files(
        name = "imx8mq_boot_files",
        srcs = [
            ":kernel_aarch64_gki_artifacts",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI system_dlkm
    pkg_files(
        name = "imx8mq_system_dlkm_files",
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
    # Command: tools/bazel run //kernel_imx:imx8mq_dist
    pkg_install(
        name = "imx8mq_dist",
        srcs = [
            ":imx8mq_kernel_files",
            ":imx8mq_vendor_boot_files",
            ":imx8mq_vendor_dlkm_files",
            ":imx8mq_boot_files",
            ":imx8mq_system_dlkm_files",
            ":imx8mq_dtb_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # Vendor boot only (initramfs.img)
    # Command: tools/bazel run //kernel_imx:imx8mq_vendor_boot_dist
    pkg_install(
        name = "imx8mq_vendor_boot_dist",
        srcs = [
            ":imx8mq_kernel_files",
            ":imx8mq_vendor_boot_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //kernel_imx:imx8mq_vendor_dlkm_dist
    pkg_install(
        name = "imx8mq_vendor_dlkm_dist",
        srcs = [
            ":imx8mq_kernel_files",
            ":imx8mq_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # DTB only distribution
    # Command: tools/bazel run //kernel_imx:imx8mq_dtb_dist
    pkg_install(
        name = "imx8mq_dtb_dist",
        srcs = [
            ":imx8mq_dtb_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # GKI boot.img only
    # Command: tools/bazel run //kernel_imx:imx8mq_boot_dist
    pkg_install(
        name = "imx8mq_boot_dist",
        srcs = [
            ":imx8mq_boot_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # GKI system_dlkm only
    # Command: tools/bazel run //kernel_imx:imx8mq_system_dlkm_dist
    pkg_install(
        name = "imx8mq_system_dlkm_dist",
        srcs = [
            ":imx8mq_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

    # GKI combined (boot.img + system_dlkm)
    # Command: tools/bazel run //kernel_imx:imx8mq_gki_dist
    pkg_install(
        name = "imx8mq_gki_dist",
        srcs = [
            ":imx8mq_boot_files",
            ":imx8mq_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8mq_aarch64/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX8MQ_VENDOR_DLKM_MODULES = _IMX8MQ_VENDOR_DLKM_MODULES
IMX8MQ_VENDOR_RAMDISK_MODULES = _IMX8MQ_VENDOR_RAMDISK_MODULES
IMX8MQ_IN_TREE_MODULES = _IMX8MQ_IN_TREE_MODULES
IMX8MQ_IMPLICIT_MODULES = _IMX8MQ_IMPLICIT_MODULES
IMX8MQ_DTB_OUTS = _IMX8MQ_DTB_OUTS
IMX8MQ_EXT_MODULES = _IMX8MQ_EXT_MODULES
