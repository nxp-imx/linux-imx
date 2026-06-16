# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 8ULP (evk_8ulp)

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
# i.MX 8ULP Device Tree Blobs (DTB)
# Extracted from device/nxp/imx8ulp/evk_8ulp/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# Conditions: TARGET_USE_DYNAMIC_PARTITIONS=true, IMX_NO_PRODUCT_PARTITION=false
# =============================================================================

# All DTB files needed for DTBO image generation
_IMX8ULP_DTB_OUTS = [
    # imx8ulp EVK; MIPI panel (rk055hdmipi4mv2)
    "arch/arm64/boot/dts/freescale/imx8ulp-evk-rk055hdmipi4mv2.dtb",

    # imx8ulp EVK; HDMI display
    "arch/arm64/boot/dts/freescale/imx8ulp-evk.dtb",

    # imx8ulp EVK; EPDC display
    "arch/arm64/boot/dts/freescale/imx8ulp-evk-epdc.dtb",

    # imx8ulp 9x9 EVK; MIPI panel (rk055hdmipi4mv2)
    "arch/arm64/boot/dts/freescale/imx8ulp-9x9-evk-rk055hdmipi4mv2.dtb",

    # imx8ulp 9x9 EVK; HDMI display
    "arch/arm64/boot/dts/freescale/imx8ulp-9x9-evk.dtb",

    # imx8ulp EVK; SOF audio
    "arch/arm64/boot/dts/freescale/imx8ulp-evk-sof-btsco.dtb",

    # imx8ulp EVK; LPA mode
    "arch/arm64/boot/dts/freescale/imx8ulp-evk-lpa.dtb",

    # imx8ulp EVK; LPD mode
    "arch/arm64/boot/dts/freescale/imx8ulp-evk-lpd.dtb",
]

# =============================================================================
# i.MX 8ULP Module Lists
# Extracted from device/nxp/imx8ulp/evk_8ulp/SharedBoardConfig.mk
# =============================================================================

# Modules for vendor ramdisk (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# These are required for early boot before vendor_dlkm is mounted
_IMX8ULP_VENDOR_RAMDISK_MODULES = [
    # HWmon
    "drivers/hwmon/hwmon.ko",
    "drivers/hwmon/scmi-hwmon.ko",

    # Power domain
    "drivers/pmdomain/arm/scmi_pm_domain.ko",

    # Clock
    "drivers/clk/imx/mxc-clk.ko",
    "drivers/clk/imx/clk-imx8ulp.ko",

    # Mailbox
    "drivers/mailbox/imx-mailbox.ko",

    # Firmware
    "drivers/firmware/imx/sm-cpu.ko",
    "drivers/firmware/imx/sm-lmm.ko",

    # Remoteproc
    "drivers/remoteproc/imx_rproc.ko",

    # Security
    "drivers/firmware/imx/sec_enclave.ko",

    # RPMSG
    "drivers/rpmsg/rpmsg_ns.ko",
    "drivers/rpmsg/virtio_rpmsg_bus.ko",

    # Pinctrl
    "drivers/pinctrl/freescale/pinctrl-imx.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8ulp.ko",

    # Serial
    "drivers/tty/serial/fsl_lpuart.ko",

    # MFD
    "drivers/mfd/imx-flexio.ko",

    # I2C
    "drivers/i2c/busses/i2c-imx-lpi2c.ko",
    "drivers/i2c/busses/i2c-rpmsg-imx.ko",
    "drivers/i2c/busses/i2c-flexio.ko",
    "drivers/gpu/drm/imx/display-imx-rpmsg.ko",
    "drivers/i2c/i2c-dev.ko",

    # I3C
    "drivers/i3c/master/svc-i3c-master.ko",

    # GPIO
    "drivers/gpio/gpio-pca953x.ko",
    "drivers/gpio/gpio-vf610.ko",
    "drivers/gpio/gpio-imx-rpmsg.ko",

    # PXP
    "drivers/dma/pxp/pxp_device.ko",
    "drivers/dma/pxp/pxp_dma_v3.ko",

    # DMA
    "drivers/dma/fsl-edma.ko",

    # Clocksource
    "drivers/clocksource/timer-imx-tpm.ko",

    # Misc
    "lib/stmp_device.ko",
    "drivers/dma/mxs-dma.ko",

    # MMC
    "drivers/mmc/core/pwrseq_simple.ko",
    "drivers/mmc/host/cqhci.ko",
    "drivers/mmc/host/sdhci-esdhc-imx.ko",

    # NVMEM
    "drivers/nvmem/nvmem-imx-ocotp-ele.ko",

    # Watchdog
    "drivers/watchdog/imx7ulp_wdt.ko",

    # PWM & Backlight
    "drivers/pwm/pwm-rpmsg-imx.ko",
    "drivers/video/backlight/pwm_bl.ko",

    # Reset
    "drivers/reset/reset-imx8ulp-sim.ko",

    # RTC
    "drivers/rtc/rtc-imx-rpmsg.ko",

    # Life cycle
    "drivers/soc/imx/rpmsg_life_cycle.ko",

    # Power
    "drivers/power/supply/dummy_battery.ko",

    # DMA buffer heaps
    "drivers/dma-buf/heaps/system_heap.ko",
    "drivers/dma-buf/heaps/cma_heap.ko",
    "drivers/dma-buf/heaps/dsp_heap.ko",
    "drivers/dma-buf/dma-buf-imx.ko",

    # USB
    "drivers/usb/chipidea/usbmisc_imx.ko",
    "drivers/usb/phy/phy-mxs-usb.ko",
    "drivers/usb/common/ulpi.ko",
    "drivers/usb/chipidea/ci_hdrc.ko",
    "drivers/usb/chipidea/ci_hdrc_imx.ko",

    # MUX
    "drivers/mux/mux-core.ko",
    "drivers/mux/mux-mmio.ko",

    # Input
    "drivers/input/touchscreen/goodix_ts.ko",

    # PHY
    "drivers/phy/freescale/phy-fsl-imx8-mipi-dphy.ko",

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
    "drivers/video/fbdev/core/fb_notify.ko",
    "drivers/video/fbdev/mxc/fb_fence.ko",
    "drivers/video/fbdev/mxc/mxc_epdc_v2_fb.ko",
    "drivers/video/fbdev/mxc/mxc_edid.ko",
    "drivers/video/logo/linux_logo.ko",

    # DRM core
    "drivers/gpu/drm/clients/drm_client_lib.ko",
    "drivers/gpu/drm/drm_dma_helper.ko",
    "drivers/gpu/drm/drm_fbdev_helper.ko",
    "drivers/gpu/drm/display/drm_display_helper.ko",

    # DRM bridge
    "drivers/gpu/drm/bridge/nwl-dsi.ko",
    "drivers/gpu/drm/bridge/it6161.ko",

    # DRM IMX
    "drivers/gpu/drm/imx/dcnano/imx-dcnano-drm.ko",

    # DRM panel
    "drivers/gpu/drm/panel/panel-rocktech-hx8394f.ko",

    # Reset
    "drivers/reset/reset-imx8ulp-csr.ko",

    # Media / Camera
    "drivers/media/platform/nxp/imx8-isi/imx8-isi.ko",
    "drivers/media/platform/nxp/imx8mq-mipi-csi2.ko",
    "drivers/media/i2c/ov5640.ko",

    # Trusty
    "drivers/trusty/trusty-smc.ko",
    "drivers/trusty/trusty-core.ko",
    "drivers/trusty/trusty-log.ko",
    "drivers/trusty/trusty-ipc.ko",
    "drivers/trusty/trusty-virtio.ko",
]

# Modules for vendor_dlkm.img (BOARD_VENDOR_KERNEL_MODULES)
# These are loaded later during boot, not required for initial boot
_IMX8ULP_VENDOR_DLKM_MODULES = [
    # Wireless
    "net/wireless/cfg80211.ko",
    "lib/crypto/libarc4.ko",
    "net/mac80211/mac80211.ko",

    # GPU
    "drivers/mxc/gpu-viv/galcore.ko",

    # EPDC PMIC
    "drivers/mfd/fp9931-core.ko",
    "drivers/regulator/fp9931-regulator.ko",
    "drivers/hwmon/fp9931-hwmon.ko",

    # Audio codecs
    "sound/soc/codecs/snd-soc-bt-sco.ko",
    "sound/soc/codecs/snd-soc-wm8960.ko",

    # Firmware
    "drivers/firmware/imx/sm-misc.ko",

    # Audio - PCM/DMA
    "sound/soc/fsl/imx-pcm-dma.ko",
    "sound/soc/fsl/snd-soc-fsl-utils.ko",
    "sound/soc/fsl/snd-soc-fsl-spdif.ko",
    "sound/soc/fsl/snd-soc-imx-audmux.ko",
    "sound/soc/fsl/snd-soc-fsl-asoc-card.ko",

    # Audio cards
    "sound/soc/generic/snd-soc-simple-card-utils.ko",
    "sound/soc/generic/snd-soc-simple-card.ko",
    "sound/soc/fsl/snd-soc-fsl-sai.ko",

    # Audio RPMSG
    "sound/soc/fsl/imx-pcm-rpmsg.ko",
    "sound/soc/fsl/imx-audio-rpmsg.ko",
    "sound/soc/fsl/snd-soc-fsl-rpmsg.ko",
    "sound/soc/fsl/snd-soc-imx-rpmsg.ko",

    # DSP Remoteproc
    "drivers/remoteproc/imx_dsp_rproc.ko",
    "drivers/firmware/imx/imx-dsp.ko",

    # SOF (Sound Open Firmware)
    "sound/soc/sof/snd-sof-utils.ko",
    "sound/soc/sof/snd-sof.ko",
    "sound/soc/sof/snd-sof-of.ko",
    "sound/soc/sof/xtensa/snd-sof-xtensa-dsp.ko",
    "sound/soc/sof/imx/imx-common.ko",
    "sound/soc/sof/imx/snd-sof-imx8.ko",

    # Input
    "drivers/input/keyboard/rpmsg-keys.ko",

    # IIO sensors
    "drivers/iio/buffer/kfifo_buf.ko",
    "drivers/iio/imu/st_lsm6dsx/st_lsm6dsx.ko",
    "drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_i2c.ko",
    "drivers/iio/industrialio-configfs.ko",
    "drivers/iio/industrialio-sw-trigger.ko",
    "drivers/iio/trigger/iio-trig-hrtimer.ko",
    "drivers/iio/trigger/iio-trig-sysfs.ko",
    "drivers/iio/buffer/industrialio-triggered-buffer.ko",
    "drivers/iio/pressure/mpl3115.ko",
    "drivers/iio/imu/rpmsg_iio_pedometer.ko",
    "drivers/iio/accel/fxls8962af-core.ko",
    "drivers/iio/accel/fxls8962af-i2c.ko",

    # MTD
    "drivers/mtd/mtd.ko",
    "drivers/mtd/chips/chipreg.ko",
    "drivers/mtd/parsers/ofpart.ko",

    # SPI
    "drivers/spi/spi-fsl-lpspi.ko",
    "drivers/spi/spidev.ko",
    "drivers/spi/spi-nxp-fspi.ko",

    # MTD SPI-NOR
    "drivers/mtd/spi-nor/spi-nor.ko",

    # Ethernet PHY
    "drivers/net/phy/micrel.ko",

    # FEC Ethernet
    "drivers/net/ethernet/freescale/fec.ko",

    # LPM
    "drivers/soc/imx/imx8ulp_lpm.ko",
]

# Additional modules detected by build (dependencies from imx8ulp_gki.fragment)
# These are built due to Kconfig dependencies but not explicitly required.
# Using module_implicit_outs to avoid build errors when modules are not built.
_IMX8ULP_IMPLICIT_MODULES = [
    "drivers/iio/health/max30102.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-cpu.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.ko",
    "drivers/power/supply/max17042_battery.ko",
    "sound/soc/codecs/snd-soc-wm-hubs.ko",
    "drivers/usb/chipidea/ci_hdrc_msm.ko",
    "drivers/trusty/trusty-test.ko",
    "drivers/iio/light/tsl2540.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "sound/soc/codecs/snd-soc-wm8994.ko",
    "drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_spi.ko",
    "drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_i3c.ko",
    "sound/soc/fsl/snd-soc-fsl-ssi.ko",
    "drivers/base/regmap/regmap-i3c.ko",
    "drivers/mfd/wm8994.ko",
    "sound/soc/codecs/snd-soc-tlv320aic31xx.ko",
    "sound/soc/fsl/snd-soc-fsl-esai.ko",
    "drivers/trusty/trusty-populate.ko",
    "drivers/leds/leds-pwm.ko",
    "drivers/usb/chipidea/ci_hdrc_usb2.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-lmm.ko",
    "drivers/input/touchscreen/elants_i2c.ko",
    "drivers/usb/chipidea/ci_hdrc_npcm.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX8ULP_IN_TREE_MODULES = _IMX8ULP_VENDOR_RAMDISK_MODULES + _IMX8ULP_VENDOR_DLKM_MODULES

# External modules (built separately)
_IMX8ULP_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx8ulp",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX8ULP_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

def define_imx8ulp():
    """Define Bazel targets for i.MX 8ULP kernel build."""

    # ==========================================================================
    # Module list files for initramfs
    # ==========================================================================

    # Modules list for vendor ramdisk (initramfs)
    write_file(
        name = "imx8ulp_vendor_ramdisk_modules_list",
        out = "imx8ulp_vendor_ramdisk_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8ULP_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Explicit module load order for vendor ramdisk
    # Matches the order defined in _IMX8ULP_VENDOR_RAMDISK_MODULES
    write_file(
        name = "imx8ulp_modules_load_order",
        out = "imx8ulp_modules.load",
        content = [m.split("/")[-1] for m in _IMX8ULP_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Modules list for vendor_dlkm
    write_file(
        name = "imx8ulp_vendor_dlkm_modules_list",
        out = "imx8ulp_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX8ULP_VENDOR_DLKM_MODULES] + _IMX8ULP_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm
    # Matches the order defined in _IMX8ULP_VENDOR_DLKM_MODULES
    write_file(
        name = "imx8ulp_vendor_dlkm_modules_load_order",
        out = "imx8ulp_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX8ULP_VENDOR_DLKM_MODULES] + _IMX8ULP_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build
    # ==========================================================================

    kernel_build(
        name = "imx8ulp",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
        ] + _IMX8ULP_DTB_OUTS,
        arch = "arm64",
        # Mixed build: use GKI as base
        base_kernel = ":kernel_aarch64",
        # Use gki_defconfig + imx8ulp fragment
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx8ulp_gki.fragment",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX8ULP_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX8ULP_IMPLICIT_MODULES,
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
        name = "imx8ulp_abi",
        kernel_build = ":imx8ulp",
        kernel_modules = _IMX8ULP_EXT_MODULES,
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )

    kernel_modules_install(
        name = "imx8ulp_modules_install",
        kernel_build = ":imx8ulp",
        kernel_modules = _IMX8ULP_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor boot image
    # ==========================================================================

    # Initramfs for vendor_boot.img
    # Contains only _IMX8ULP_VENDOR_RAMDISK_MODULES
    initramfs(
        name = "imx8ulp_initramfs",
        kernel_modules_install = ":imx8ulp_modules_install",
        ramdisk_compression = "lz4",
        # Only include vendor ramdisk modules
        modules_list = ":imx8ulp_vendor_ramdisk_modules_list",
        # Explicit load order matching _IMX8ULP_VENDOR_RAMDISK_MODULES
        modules_load = ":imx8ulp_modules_load_order",
        # Remove modules not in modules_list from initramfs
        trim_unused_modules = True,
    )

    # vendor_boot.img - contains vendor ramdisk with ramdisk.lz4
    vendor_boot_image(
        name = "imx8ulp_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx8ulp_initramfs",
        kernel_build = ":imx8ulp",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # vendor_dlkm.img - contains only _IMX8ULP_VENDOR_DLKM_MODULES
    vendor_dlkm_image(
        name = "imx8ulp_vendor_dlkm",
        kernel_modules_install = ":imx8ulp_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx8ulp_vendor_dlkm_modules_list",
        # Explicit load order matching _IMX8ULP_VENDOR_DLKM_MODULES
        modules_load = ":imx8ulp_vendor_dlkm_modules_load_order",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx8ulp_initramfs",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules (shared base)
    pkg_files(
        name = "imx8ulp_kernel_files",
        srcs = [
            ":imx8ulp",
            ":imx8ulp_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx8ulp_vendor_boot_files",
        srcs = [
            ":imx8ulp_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM
    pkg_files(
        name = "imx8ulp_vendor_dlkm_files",
        srcs = [
            ":imx8ulp_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files
    _dtb_srcs = [
        ":imx8ulp/" + f for f in _IMX8ULP_DTB_OUTS
    ]

    pkg_files(
        name = "imx8ulp_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx8ulp/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # GKI boot.img variants
    pkg_files(
        name = "imx8ulp_boot_files",
        srcs = [
            ":kernel_aarch64_gki_artifacts",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI system_dlkm
    pkg_files(
        name = "imx8ulp_system_dlkm_files",
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
    # Command: tools/bazel run //kernel_imx:imx8ulp_dist
    pkg_install(
        name = "imx8ulp_dist",
        srcs = [
            ":imx8ulp_kernel_files",
            ":imx8ulp_vendor_boot_files",
            ":imx8ulp_vendor_dlkm_files",
            ":imx8ulp_boot_files",
            ":imx8ulp_system_dlkm_files",
            ":imx8ulp_dtb_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # Vendor boot only (initramfs.img)
    # Command: tools/bazel run //kernel_imx:imx8ulp_vendor_boot_dist
    pkg_install(
        name = "imx8ulp_vendor_boot_dist",
        srcs = [
            ":imx8ulp_kernel_files",
            ":imx8ulp_vendor_boot_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //kernel_imx:imx8ulp_vendor_dlkm_dist
    pkg_install(
        name = "imx8ulp_vendor_dlkm_dist",
        srcs = [
            ":imx8ulp_kernel_files",
            ":imx8ulp_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # DTB only distribution
    # Command: tools/bazel run //kernel_imx:imx8ulp_dtb_dist
    pkg_install(
        name = "imx8ulp_dtb_dist",
        srcs = [
            ":imx8ulp_dtb_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # GKI boot.img only
    # Command: tools/bazel run //kernel_imx:imx8ulp_boot_dist
    pkg_install(
        name = "imx8ulp_boot_dist",
        srcs = [
            ":imx8ulp_boot_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # GKI system_dlkm only
    # Command: tools/bazel run //kernel_imx:imx8ulp_system_dlkm_dist
    pkg_install(
        name = "imx8ulp_system_dlkm_dist",
        srcs = [
            ":imx8ulp_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

    # GKI combined (boot.img + system_dlkm)
    # Command: tools/bazel run //kernel_imx:imx8ulp_gki_dist
    pkg_install(
        name = "imx8ulp_gki_dist",
        srcs = [
            ":imx8ulp_boot_files",
            ":imx8ulp_system_dlkm_files",
        ],
        destdir = "out/imx_evk_8ulp_aarch64/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX8ULP_VENDOR_DLKM_MODULES = _IMX8ULP_VENDOR_DLKM_MODULES
IMX8ULP_VENDOR_RAMDISK_MODULES = _IMX8ULP_VENDOR_RAMDISK_MODULES
IMX8ULP_IMPLICIT_MODULES = _IMX8ULP_IMPLICIT_MODULES
IMX8ULP_IN_TREE_MODULES = _IMX8ULP_IN_TREE_MODULES
IMX8ULP_DTB_OUTS = _IMX8ULP_DTB_OUTS
IMX8ULP_EXT_MODULES = _IMX8ULP_EXT_MODULES
