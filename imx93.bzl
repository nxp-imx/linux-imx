# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 93 (evk_93)
# Note: Standalone build (not GKI mixed build)
# boot.img is built by Android make using Bazel-built Image.lz4

load(
    "//build/kernel/kleaf:kernel.bzl",
    "initramfs",
    "kernel_build",
    "kernel_modules_install",
    "vendor_boot_image",
    "vendor_dlkm_image",
)
load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix")

# =============================================================================
# i.MX 93 Device Tree Blobs (DTB)
# Extracted from device/nxp/imx93/evk_93/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# =============================================================================

_IMX93_DTB_OUTS = [
    # imx93 11x11 EVK; base
    "arch/arm64/boot/dts/freescale/imx93-11x11-evk.dtb",

    # imx93 11x11 EVK; IW612 OTBR
    "arch/arm64/boot/dts/freescale/imx93-11x11-evk-iw612-otbr.dtb",

    # imx93 11x11 FRDM; IW612 OTBR
    "arch/arm64/boot/dts/freescale/imx93-11x11-frdm-iw612-otbr.dtb",

    # imx93 11x11 FRDM; Tianma WVGA panel
    "arch/arm64/boot/dts/freescale/imx93-11x11-frdm-tianma-wvga-panel.dtb",
]

# =============================================================================
# i.MX 93 Module Lists
# Extracted from device/nxp/imx9/evk_93/SharedBoardConfig.mk
# =============================================================================

# Modules for vendor ramdisk (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# Empty for imx93 — all boot-critical modules are built-in to kernel Image
_IMX93_VENDOR_RAMDISK_MODULES = []

# Modules for vendor_dlkm.img (BOARD_VENDOR_KERNEL_MODULES)
# These are loaded later during boot, not required for initial boot
_IMX93_VENDOR_DLKM_MODULES = [
    # Camera
    "drivers/media/i2c/ap1302.ko",
    "drivers/staging/media/imx/imx8-media-dev.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX93_IN_TREE_MODULES = _IMX93_VENDOR_DLKM_MODULES

# External modules (built separately)
_IMX93_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx93",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX93_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

# Implicit modules (optional - build continues if missing)
# These are Kconfig dependencies that may or may not be built
_IMX93_IMPLICIT_MODULES = [
    "drivers/net/ppp/ppp_generic.ko",
    "kernel/time/time_test.ko",
    "net/tipc/tipc.ko",
    "drivers/net/macsec.ko",
    "drivers/virtio/virtio_pci_modern_dev.ko",
    "drivers/input/touchscreen/goodix_ts.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "drivers/net/ppp/bsd_comp.ko",
    "drivers/net/usb/cdc_ncm.ko",
    "drivers/gpu/drm/panthor/panthor.ko",
    "drivers/usb/serial/ftdi_sio.ko",
    "drivers/net/usb/rtl8150.ko",
    "drivers/power/sequencing/pwrseq-core.ko",
    "lib/kunit/kunit-test.ko",
    "drivers/rpmsg/imx_rpmsg_tty.ko",
    "fs/fat/fat_test.ko",
    "drivers/gpu/drm/drm_gpuvm.ko",
    "drivers/android/binder/rust_binder.ko",
    "drivers/remoteproc/imx_dsp_rproc.ko",
    "drivers/misc/vcpu_stall_detector.ko",
    "drivers/base/regmap/regmap-raw-ram.ko",
    "drivers/net/usb/r8153_ecm.ko",
    "drivers/rtc/test_rtc_lib.ko",
    "drivers/net/usb/cdc_eem.ko",
    "drivers/bluetooth/btbcm.ko",
    "sound/soc/soc-utils-test.ko",
    "net/core/dev_addr_lists_test.ko",
    "net/tipc/tipc_diag.ko",
    "arch/arm64/geniezone/gzvm.ko",
    "drivers/net/ppp/ppp_mppe.ko",
    "drivers/input/touchscreen/synaptics_dsx/synaptics_dsx_i2c.ko",
    "drivers/clk/clk_kunit_helpers.ko",
    "drivers/net/usb/cdc_ether.ko",
    "sound/soc/soc-topology-test.ko",
    "drivers/gnss/gnss.ko",
    "drivers/perf/dwc_pcie_pmu.ko",
    "drivers/net/usb/asix.ko",
    "drivers/virtio/virtio_balloon.ko",
    "drivers/hid/hid-uclogic-test.ko",
    "drivers/net/ppp/ppp_deflate.ko",
    "drivers/gpu/drm/scheduler/gpu-sched.ko",
    "drivers/thunderbolt/thunderbolt.ko",
    "lib/kunit/kunit.ko",
    "drivers/net/usb/r8152.ko",
    "kernel/kheaders.ko",
    "drivers/gpu/arm/pma/protected_heap.ko",
    "drivers/net/usb/aqc111.ko",
    "drivers/base/regmap/regmap-ram.ko",
    "drivers/usb/serial/usbserial.ko",
    "drivers/net/usb/ax88179_178a.ko",
    "drivers/char/virtio_console.ko",
    "drivers/bluetooth/hci_uart.ko",
    "drivers/gpu/arm/pma/protected_memory_allocator.ko",
    "drivers/usb/mon/usbmon.ko",
    "lib/kunit/kunit-example-test.ko",
    "drivers/char/hw_random/cctrng.ko",
    "drivers/net/ppp/pptp.ko",
    "net/vmw_vsock/vmw_vsock_virtio_transport.ko",
    "drivers/usb/class/cdc-acm.ko",
    "drivers/bluetooth/btqca.ko",
    "drivers/gpu/arm/midgard/mali_kbase.ko",
    "drivers/input/tests/input_test.ko",
    "drivers/net/ppp/pppox.ko",
    "drivers/block/virtio_blk.ko",
    "drivers/net/slip/slhc.ko",
    "drivers/of/of_kunit_helpers.ko",
    "net/l2tp/l2tp_ppp.ko",
    "lib/kunit/platform-test.ko",
    "drivers/net/usb/usbnet.ko",
    "drivers/trusty/trusty-populate.ko",
    "drivers/input/touchscreen/elants_i2c.ko",
    "drivers/misc/open-dice.ko",
    "drivers/clk/clk-gate_test.ko",
    "drivers/rpmsg/imx_rpmsg_chre.ko",
    "drivers/rpmsg/imx_rpmsg_pingpong.ko",
    "drivers/clk/clk-test.ko",
    "drivers/remoteproc/imx_neutron_rproc.ko",
    "drivers/base/regmap/regmap-kunit.ko",
    "drivers/gpu/drm/drm_exec.ko",
    "fs/ext4/ext4-inode-test.ko",
    "drivers/virtio/virtio_pci_legacy_dev.ko",
    "drivers/staging/neutron/neutron.ko",
    "drivers/iio/test/iio-test-format.ko",
    "drivers/virtio/virtio_pci.ko",
    "drivers/android/tests/binder_alloc_kunit.ko",
]

def define_imx93():
    """Define Bazel targets for i.MX 93 kernel build."""

    # ==========================================================================
    # Module list files for initramfs and vendor_dlkm
    # ==========================================================================

    # Empty vendor ramdisk modules list
    write_file(
        name = "imx93_vendor_ramdisk_modules_list",
        out = "imx93_vendor_ramdisk_modules.txt",
        content = [""],
    )

    # Modules list for vendor_dlkm
    write_file(
        name = "imx93_vendor_dlkm_modules_list",
        out = "imx93_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX93_VENDOR_DLKM_MODULES] + _IMX93_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm
    # Matches the order defined in _IMX93_VENDOR_DLKM_MODULES
    write_file(
        name = "imx93_vendor_dlkm_modules_load_order",
        out = "imx93_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX93_VENDOR_DLKM_MODULES] + _IMX93_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build (standalone — not GKI mixed build)
    # ==========================================================================

    kernel_build(
        name = "imx93",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
            "System.map",
            "vmlinux",
        ] + _IMX93_DTB_OUTS,
        arch = "arm64",
        # Standalone build (not GKI mixed build)
        # imx93 uses its own kernel Image, not GKI boot.img
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx_v8_android_defconfig",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "vmlinux",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX93_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX93_IMPLICIT_MODULES,
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

    kernel_modules_install(
        name = "imx93_modules_install",
        kernel_build = ":imx93",
        kernel_modules = _IMX93_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor boot
    # vendor_boot is empty (all boot-critical modules are built-in)
    # ==========================================================================

    # Empty initramfs (no vendor ramdisk modules)
    initramfs(
        name = "imx93_initramfs",
        kernel_modules_install = ":imx93_modules_install",
        ramdisk_compression = "lz4",
        modules_list = ":imx93_vendor_ramdisk_modules_list",
        trim_unused_modules = True,
    )

    # vendor_boot.img - empty vendor ramdisk (ramdisk.lz4)
    vendor_boot_image(
        name = "imx93_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx93_initramfs",
        kernel_build = ":imx93",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # ==========================================================================
    # Vendor DLKM
    # ==========================================================================

    # vendor_dlkm.img - contains neutron NPU + camera + WiFi modules
    vendor_dlkm_image(
        name = "imx93_vendor_dlkm",
        kernel_modules_install = ":imx93_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx93_vendor_dlkm_modules_list",
        # Explicit load order matching _IMX93_VENDOR_DLKM_MODULES
        modules_load = ":imx93_vendor_dlkm_modules_load_order",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules
    pkg_files(
        name = "imx93_kernel_files",
        srcs = [
            ":imx93",
            ":imx93_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx93_vendor_boot_files",
        srcs = [
            ":imx93_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM
    pkg_files(
        name = "imx93_vendor_dlkm_files",
        srcs = [
            ":imx93_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files
    _dtb_srcs = [
        ":imx93/" + f for f in _IMX93_DTB_OUTS
    ]

    pkg_files(
        name = "imx93_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx93/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # ==========================================================================
    # Distribution targets
    # ==========================================================================

    # Full distribution (kernel + vendor_boot + vendor_dlkm + DTBs)
    # Note: boot.img is built by Android make (not Bazel) for imx93
    # Command: tools/bazel run //kernel_imx:imx93_dist
    pkg_install(
        name = "imx93_dist",
        srcs = [
            ":imx93_kernel_files",
            ":imx93_vendor_boot_files",
            ":imx93_vendor_dlkm_files",
            ":imx93_dtb_files",
        ],
        destdir = "out/imx_evk_93_aarch64/dist",
    )

    # Vendor boot only (empty vendor ramdisk)
    # Command: tools/bazel run //kernel_imx:imx93_vendor_boot_dist
    pkg_install(
        name = "imx93_vendor_boot_dist",
        srcs = [
            ":imx93_kernel_files",
            ":imx93_vendor_boot_files",
        ],
        destdir = "out/imx_evk_93_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //kernel_imx:imx93_vendor_dlkm_dist
    pkg_install(
        name = "imx93_vendor_dlkm_dist",
        srcs = [
            ":imx93_kernel_files",
            ":imx93_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_93_aarch64/dist",
    )

    # DTB only distribution
    # Command: tools/bazel run //kernel_imx:imx93_dtb_dist
    pkg_install(
        name = "imx93_dtb_dist",
        srcs = [
            ":imx93_dtb_files",
        ],
        destdir = "out/imx_evk_93_aarch64/dist",
    )

    # System DLKM (empty — standalone build, no GKI system_dlkm)
    # This target exists for imx-make.sh compatibility
    # Command: tools/bazel run //kernel_imx:imx93_system_dlkm_dist
    pkg_install(
        name = "imx93_system_dlkm_dist",
        srcs = [
            ":imx93_kernel_files",
        ],
        destdir = "out/imx_evk_93_aarch64/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX93_VENDOR_DLKM_MODULES = _IMX93_VENDOR_DLKM_MODULES
IMX93_IMPLICIT_MODULES = _IMX93_IMPLICIT_MODULES
IMX93_IN_TREE_MODULES = _IMX93_IN_TREE_MODULES
IMX93_DTB_OUTS = _IMX93_DTB_OUTS
IMX93_EXT_MODULES = _IMX93_EXT_MODULES
