load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@rules_pkg//pkg:install.bzl", "pkg_install")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "strip_prefix")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//build/kernel/kleaf:kernel.bzl",
    "ddk_headers",
    "ddk_module",
    "kernel_abi",
    "kernel_build",
)
load(":logo_rules.bzl", "logo_gen")

def define_imx_ddk_modules():
    # Build kernel module
    #_IMX_COMMON_MODULES is built from the common code. Refer to imx_core.fragment.
    _IMX_COMMON_MODULES = [
        "net/wireless/cfg80211.ko",
        "net/mac80211/mac80211.ko",
    ]

    ddk_headers(
        name = "imx_common_headers",
        # Include all .h files recursively
        hdrs = native.glob(["**/*.h"]),
        # Define include paths relative to this BUILD file's directory.
        linux_includes = [
            ".",
            "include",
            "include/uapi",
        ],
    )

    # Group Kconfig files used for module configuration
    native.filegroup(
        name = "imx_kconfigs",
        srcs = [
            "drivers/soc/imx/Kconfig",
            "arch/arm/mach-imx/Kconfig",
            "drivers/gpu/drm/Kconfig",
            "drivers/trusty/Kconfig",
            "drivers/firmware/arm_scmi/vendors/imx/Kconfig",
            "drivers/android/Kconfig",
            "drivers/firmware/imx/Kconfig",
            "drivers/tty/serial/Kconfig",
            "drivers/gpu/drm/imx/dpu95/Kconfig",
            "drivers/net/ethernet/freescale/enetc/Kconfig",
            "drivers/ptp/Kconfig",
            "drivers/gpu/arm/midgard/Kconfig",
            "drivers/gpu/arm/midgard/platform/Kconfig",
            "drivers/media/i2c/ox05b1s/Kconfig",
        ],
    )
    ddk_module(
        name = "mxc-clk",
        out = "mxc-clk.ko",
        srcs = [
            "drivers/clk/imx/clk-busy.c",
            "drivers/clk/imx/clk.c",
            "drivers/clk/imx/clk-composite-7ulp.c",
            "drivers/clk/imx/clk-composite-8m.c",
            "drivers/clk/imx/clk-composite-93.c",
            "drivers/clk/imx/clk-fracn-gppll.c",
            "drivers/clk/imx/clk-cpu.c",
            "drivers/clk/imx/clk-divider-gate.c",
            "drivers/clk/imx/clk-fixup-div.c",
            "drivers/clk/imx/clk-fixup-mux.c",
            "drivers/clk/imx/clk-frac-pll.c",
            "drivers/clk/imx/clk-gate2.c",
            "drivers/clk/imx/clk-gate-93.c",
            "drivers/clk/imx/clk-gate-exclusive.c",
            "drivers/clk/imx/clk-pfd.c",
            "drivers/clk/imx/clk-pfdv2.c",
            "drivers/clk/imx/clk-pllv1.c",
            "drivers/clk/imx/clk-pllv2.c",
            "drivers/clk/imx/clk-pllv3.c",
            "drivers/clk/imx/clk-pllv4.c",
            "drivers/clk/imx/clk-pll14xx.c",
            "drivers/clk/imx/clk-sscg-pll.c",
            "drivers/clk/imx/clk-gpr-mux.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "clk-imx8mm",
        out = "clk-imx8mm.ko",
        srcs = ["drivers/clk/imx/clk-imx8mm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":mxc-clk",
        ],
    )

    ddk_module(
        name = "soc-imx8m",
        out = "soc-imx8m.ko",
        srcs = ["drivers/soc/imx/soc-imx8m.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx8m-blk-ctrl",
        out = "imx8m-blk-ctrl.ko",
        srcs = ["drivers/pmdomain/imx/imx8m-blk-ctrl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "imx8m_pm_domains",
        out = "imx8m_pm_domains.ko",
        srcs = ["drivers/soc/imx/imx8m_pm_domains.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpcv2",
        out = "gpcv2.ko",
        srcs = ["drivers/pmdomain/imx/gpcv2.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpcv2-imx",
        out = "gpcv2-imx.ko",
        srcs = ["drivers/pmdomain/imx/gpcv2-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":gpcv2",
        ],
    )

    ddk_module(
        name = "timer-imx-sysctr",
        out = "timer-imx-sysctr.ko",
        srcs = ["drivers/clocksource/timer-imx-sysctr.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "busfreq-imx8mq",
        out = "busfreq-imx8mq.ko",
        srcs = ["drivers/soc/imx/busfreq-imx8mq.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pinctrl-imx",
        out = "pinctrl-imx.ko",
        srcs = ["drivers/pinctrl/freescale/pinctrl-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pinctrl-imx8mm",
        out = "pinctrl-imx8mm.ko",
        srcs = ["drivers/pinctrl/freescale/pinctrl-imx8mm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":pinctrl-imx",
        ],
    )

    ddk_module(
        name = "imx-tty",
        out = "imx.ko",
        srcs = ["drivers/tty/serial/imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "imx2_wdt",
        out = "imx2_wdt.ko",
        srcs = ["drivers/watchdog/imx2_wdt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pca9450-regulator",
        out = "pca9450-regulator.ko",
        srcs = ["drivers/regulator/pca9450-regulator.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpio-mxc",
        out = "gpio-mxc.ko",
        srcs = ["drivers/gpio/gpio-mxc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl_imx8_ddr_perf",
        out = "fsl_imx8_ddr_perf.ko",
        srcs = ["drivers/perf/fsl_imx8_ddr_perf.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )


    ddk_module(
        name = "rohm-bd718x7",
        out = "rohm-bd718x7.ko",
        srcs = ["drivers/mfd/rohm-bd718x7.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "rohm-regulator",
        out = "rohm-regulator.ko",
        srcs = ["drivers/regulator/rohm-regulator.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "bd718x7-regulator",
        out = "bd718x7-regulator.ko",
        srcs = ["drivers/regulator/bd718x7-regulator.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        deps = [
            ":rohm-regulator",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "cpufreq-dt",
        out = "cpufreq-dt.ko",
        srcs = ["drivers/cpufreq/cpufreq-dt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-cpufreq-dt",
        out = "imx-cpufreq-dt.ko",
        srcs = ["drivers/cpufreq/imx-cpufreq-dt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "nvmem-imx-ocotp",
        out = "nvmem-imx-ocotp.ko",
        srcs = ["drivers/nvmem/imx-ocotp.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "sdhci-esdhc-imx",
        out = "sdhci-esdhc-imx.ko",
        srcs = ["drivers/mmc/host/sdhci-esdhc-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        deps = [
            ":cqhci",
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "cqhci",
        out = "cqhci.ko",
        srcs = [
            "drivers/mmc/host/cqhci-core.c",
            "drivers/mmc/host/cqhci-crypto.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "i2c-imx",
        out = "i2c-imx.ko",
        srcs = ["drivers/i2c/busses/i2c-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "i2c-dev",
        out = "i2c-dev.ko",
        srcs = ["drivers/i2c/i2c-dev.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spidev",
        out = "spidev.ko",
        srcs = ["drivers/spi/spidev.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spi-bitbang",
        out = "spi-bitbang.ko",
        srcs = ["drivers/spi/spi-bitbang.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spi-nxp-fspi",
        out = "spi-nxp-fspi.ko",
        srcs = ["drivers/spi/spi-nxp-fspi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spi-imx",
        out = "spi-imx.ko",
        srcs = ["drivers/spi/spi-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "stmp_device",
        out = "stmp_device.ko",
        srcs = ["lib/stmp_device.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mxs-dma",
        out = "mxs-dma.ko",
        srcs = ["drivers/dma/mxs-dma.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":stmp_device",
        ],
    )

    ddk_module(
        name = "pwrseq_simple",
        out = "pwrseq_simple.ko",
        srcs = ["drivers/mmc/core/pwrseq_simple.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-mailbox",
        out = "imx-mailbox.ko",
        srcs = ["drivers/mailbox/imx-mailbox.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "system_heap",
        out = "system_heap.ko",
        srcs = ["drivers/dma-buf/heaps/system_heap.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "cma_heap",
        out = "cma_heap.ko",
        srcs = ["drivers/dma-buf/heaps/cma_heap.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "dma-buf-imx",
        out = "dma-buf-imx.ko",
        srcs = ["drivers/dma-buf/dma-buf-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snvs_pwrkey",
        out = "snvs_pwrkey.ko",
        srcs = ["drivers/input/keyboard/snvs_pwrkey.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "goodix_ts",
        out = "goodix_ts.ko",
        srcs = [
            "drivers/input/touchscreen/goodix.c",
            "drivers/input/touchscreen/goodix_fwupload.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "synaptics_dsx_i2c",
        out = "synaptics_dsx_i2c.ko",
        srcs = [
            "drivers/input/touchscreen/synaptics_dsx/synaptics_dsx_i2c.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "reset-dispmix",
        out = "reset-dispmix.ko",
        srcs = ["drivers/reset/reset-dispmix.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "reset-imx7",
        out = "reset-imx7.ko",
        srcs = ["drivers/reset/reset-imx7.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    # pnmtologo tool
    cc_binary(
        name = "pnmtologo",
        srcs = ["drivers/video/logo/pnmtologo.c"],
        visibility = ["//visibility:public"],
    )

    logo_gen("logo_linux_mono",      "mono",    "pbm")
    logo_gen("logo_linux_vga16",     "vga16",   "ppm")
    logo_gen("logo_linux_clut224",   "clut224", "ppm")
    logo_gen("logo_dec_clut224",     "clut224", "ppm")
    logo_gen("logo_mac_clut224",     "clut224", "ppm")
    logo_gen("logo_parisc_clut224",  "clut224", "ppm")
    logo_gen("logo_sgi_clut224",     "clut224", "ppm")
    logo_gen("logo_sun_clut224",     "clut224", "ppm")
    logo_gen("logo_spe_clut224",     "clut224", "ppm")
    logo_gen("logo_superh_mono",     "mono",    "pbm")
    logo_gen("logo_superh_vga16",    "vga16",   "ppm")
    logo_gen("logo_superh_clut224",  "clut224", "ppm")

    ddk_module(
        name = "linux_logo",
        out  = "linux_logo.ko",
        srcs = [
            "drivers/video/logo/logo.c",
            "drivers/video/logo/logo_linux_mono.c",
            "drivers/video/logo/logo_linux_vga16.c",
            "drivers/video/logo/logo_linux_clut224.c",
            "drivers/video/logo/logo_dec_clut224.c",
            "drivers/video/logo/logo_mac_clut224.c",
            "drivers/video/logo/logo_parisc_clut224.c",
            "drivers/video/logo/logo_sgi_clut224.c",
            "drivers/video/logo/logo_sun_clut224.c",
            "drivers/video/logo/logo_spe_clut224.c",
            "drivers/video/logo/logo_superh_mono.c",
            "drivers/video/logo/logo_superh_vga16.c",
            "drivers/video/logo/logo_superh_clut224.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fb",
        out = "fb.ko",
        srcs = [
            "drivers/video/fbdev/core/fb_info.c",
            "drivers/video/fbdev/core/fbmem.c",
            "drivers/video/fbdev/core/fbcmap.c",
            "drivers/video/fbdev/core/modedb.c",
            "drivers/video/fbdev/core/fbcvt.c",
            "drivers/video/fbdev/core/fb_cmdline.c",
            "drivers/video/fbdev/core/fb_backlight.c",
            "drivers/video/fbdev/core/fbmon.c",
            "drivers/video/fbdev/core/fb_chrdev.c",
            "drivers/video/fbdev/core/fb_procfs.c",
            "drivers/video/fbdev/core/fbsysfs.c",
            "drivers/video/fbdev/core/fb_logo.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":linux_logo",
            ":fb_notify",
        ],
    )

    ddk_module(
        name = "fb_notify",
        out = "fb_notify.ko",
        srcs = ["drivers/video/fbdev/core/fb_notify.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "cfbcopyarea",
        out = "cfbcopyarea.ko",
        srcs = ["drivers/video/fbdev/core/cfbcopyarea.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fb_io_fops",
        out = "fb_io_fops.ko",
        srcs = ["drivers/video/fbdev/core/fb_io_fops.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fb_sys_fops",
        out = "fb_sys_fops.ko",
        srcs = ["drivers/video/fbdev/core/fb_sys_fops.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        defconfig = "imx_evk_8mm_module.fragment",
        kconfig = "imx_kconfigs",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sysimgblt",
        out = "sysimgblt.ko",
        srcs = ["drivers/video/fbdev/core/sysimgblt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "syscopyarea",
        out = "syscopyarea.ko",
        srcs = ["drivers/video/fbdev/core/syscopyarea.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sysfillrect",
        out = "sysfillrect.ko",
        srcs = ["drivers/video/fbdev/core/sysfillrect.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "cfbfillrect",
        out = "cfbfillrect.ko",
        srcs = ["drivers/video/fbdev/core/cfbfillrect.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "cfbimgblt",
        out = "cfbimgblt.ko",
        srcs = ["drivers/video/fbdev/core/cfbimgblt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "drm_fbdev_helper",
        out = "drm_fbdev_helper.ko",
        srcs = [
            "drivers/gpu/drm/drm_fbdev_client.c",
            "drivers/gpu/drm/drm_fb_helper.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":fb",
        ],
    )

    ddk_module(
        name = "imx8mm-lcdif-core",
        out = "imx8mm-lcdif-core.ko",
        srcs = ["drivers/gpu/imx/lcdif/lcdif-common.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":busfreq-imx8mq",
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "drm_dma_helper",
        out = "drm_dma_helper.ko",
        srcs = [
            "drivers/gpu/drm/drm_gem_dma_helper.c",
            "drivers/gpu/drm/drm_fbdev_dma.c",
            "drivers/gpu/drm/drm_fb_dma_helper.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":drm_fbdev_helper",
            ":fb_sys_fops",
            ":sysfillrect",
            ":sysimgblt",
            ":syscopyarea",
        ],
    )

    ddk_module(
        name = "adv7511",
        out = "adv7511.ko",
        srcs = [
            "drivers/gpu/drm/bridge/adv7511/adv7511_drv.c",
            "drivers/gpu/drm/bridge/adv7511/adv7533.c",
            "drivers/gpu/drm/bridge/adv7511/adv7511_cec.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sec-dsim",
        out = "sec-dsim.ko",
        srcs = ["drivers/gpu/drm/bridge/sec-dsim.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imxdrm",
        out = "imxdrm.ko",
        srcs = ["drivers/gpu/drm/imx/imx-drm-core.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":drm_dma_helper",
        ],
    )

    ddk_module(
        name = "imx8mm-lcdif-crtc",
        out = "imx8mm-lcdif-crtc.ko",
        srcs = [
            "drivers/gpu/drm/imx/lcdif/lcdif-crtc.c",
            "drivers/gpu/drm/imx/lcdif/lcdif-plane.c",
            "drivers/gpu/drm/imx/lcdif/lcdif-kms.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        includes = ["drivers/gpu/drm/imx"],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":imx8mm-lcdif-core",
            ":drm_dma_helper",
        ],
    )

    ddk_module(
        name = "panel-raydium-rm67191",
        out = "panel-raydium-rm67191.ko",
        srcs = ["drivers/gpu/drm/panel/panel-raydium-rm67191.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sec_mipi_dsim-imx",
        out = "sec_mipi_dsim-imx.ko",
        srcs = ["drivers/gpu/drm/imx/sec_mipi_dsim-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        deps = [
            ":imxdrm",
            ":sec-dsim",
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "usbmisc_imx",
        out = "usbmisc_imx.ko",
        srcs = ["drivers/usb/chipidea/usbmisc_imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ulpi",
        out = "ulpi.ko",
        srcs = ["drivers/usb/common/ulpi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ci_hdrc",
        out = "ci_hdrc.ko",
        srcs = [
            "drivers/usb/chipidea/core.c",
            "drivers/usb/chipidea/otg.c",
            "drivers/usb/chipidea/debug.c",
            "drivers/usb/chipidea/ulpi.c",
            "drivers/usb/chipidea/udc.c",
            "drivers/usb/chipidea/trace.c",
            "drivers/usb/chipidea/host.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        includes = [
            "drivers/usb/chipidea",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":ulpi",
        ],
    )

    ddk_module(
        name = "ci_hdrc_imx",
        out = "ci_hdrc_imx.ko",
        srcs = ["drivers/usb/chipidea/ci_hdrc_imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        deps = [
            ":usbmisc_imx",
            ":ci_hdrc",
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "phy-generic",
        out = "phy-generic.ko",
        srcs = ["drivers/usb/phy/phy-generic.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "dummy_battery",
        out = "dummy_battery.ko",
        srcs = ["drivers/power/supply/dummy_battery.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sdma",
        out = "imx-sdma.ko",
        srcs = ["drivers/dma/imx-sdma.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "trusty-core",
        out = "trusty-core.ko",
        srcs = [
            "drivers/trusty/trusty.c",
            "drivers/trusty/trusty-mem.c",
            "drivers/trusty/trusty-sched-share.c",
            "drivers/trusty/trusty-smc-arm64.S",
            "drivers/trusty/trusty-irq.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "trusty-log",
        out = "trusty-log.ko",
        srcs = ["drivers/trusty/trusty-log.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "trusty-ipc",
        out = "trusty-ipc.ko",
        srcs = ["drivers/trusty/trusty-ipc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "trusty-virtio",
        out = "trusty-virtio.ko",
        srcs = ["drivers/trusty/trusty-virtio.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_headers(
        name = "imx_galcore_headers",
        # Include all .h files.
        hdrs = native.glob([
            "drivers/mxc/gpu-viv/hal/**/*.h",
        ]),
        # Define include paths relative to this BUILD file's directory.
        includes = [
            "drivers/mxc/gpu-viv/hal/kernel",
            "drivers/mxc/gpu-viv/hal/kernel/inc",
            "drivers/mxc/gpu-viv/hal/kernel/arch",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel",
            "drivers/mxc/gpu-viv/hal/security_v1",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/freescale",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/platform/freescale",
            "drivers/mxc/gpu-viv/hal/kernel/archvg",
        ],
    )

    ddk_module(
        name = "galcore",
        out = "galcore.ko",
        srcs = [
            # hal/os/linux/kernel core
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_device.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_linux.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_math.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_os.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_iommu.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_debug.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_debugfs.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_allocator.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_trace.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_driver.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/default/gc_hal_kernel_allocator_user_memory.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/default/gc_hal_kernel_allocator_dma.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/default/gc_hal_kernel_allocator_gfp.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/default/gc_hal_kernel_allocator_reserved_mem.c",

            ## platform driver (freescale-imx)
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/platform/freescale/gc_hal_kernel_platform_imx.c",
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/freescale/gc_hal_kernel_allocator_cma.c",

            # hal/kernel core
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_command.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_db.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_event.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_heap.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_mmu.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_video_memory.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_power.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_security_v1.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_preemption.c",

            # hal/kernel/arch
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_context.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware_func.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware_func_flop_reset.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware_async_fe.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware_mc_fe.c",
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_hardware_waitlink_fe.c",

            #3D support
            "drivers/mxc/gpu-viv/hal/kernel/arch/gc_hal_kernel_recorder.c",

            #VIVANTE_ENABLE_VG=1
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_vg.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_command_vg.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_interrupt_vg.c",
            "drivers/mxc/gpu-viv/hal/kernel/gc_hal_kernel_mmu_vg.c",
            "drivers/mxc/gpu-viv/hal/kernel/archvg/gc_hal_kernel_hardware_command_vg.c",
            "drivers/mxc/gpu-viv/hal/kernel/archvg/gc_hal_kernel_hardware_vg.c",

            # security-emulator
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_security_channel_emulator.c",

            # trust-application (TA)
            "drivers/mxc/gpu-viv/hal/security_v1/gc_hal_ta.c",
            "drivers/mxc/gpu-viv/hal/security_v1/gc_hal_ta_hardware.c",
            "drivers/mxc/gpu-viv/hal/security_v1/gc_hal_ta_mmu.c",
            "drivers/mxc/gpu-viv/hal/security_v1/os/emulator/gc_hal_ta_emulator.c",

            #shared DMA buffer support
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/allocator/default/gc_hal_kernel_allocator_dmabuf.c",

            # DRM integration
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_drm.c",

            #CONFIG_SYNC_FILE
            "drivers/mxc/gpu-viv/hal/os/linux/kernel/gc_hal_kernel_sync.c",
        ],
        hdrs = [
            ":imx_common_headers",
            ":imx_galcore_headers",
        ],
        copts = [
            # do not sort
            #gc_hal_kernel_platform_$(soc_board).config
            "-DgcdDEFAULT_CONTIGUOUS_SIZE=~0U",
            "-DgcdFSL_CONTIGUOUS_SIZE=134217728",
            "-DgcdANDROID",
            "-DgcdANDROID_NATIVE_FENCE_SYNC=2",
            "-DIMX8_PHYS_BASE=0x0", #CONFIG_SOC_IMX8M=m
            "-DIMX8_PHYS_SIZE=0x0", #CONFIG_SOC_IMX8M=m
            "-DLINUX_CMA_FSL=1",
            "-DgcdREGISTER_READ_FROM_USER=1",
            "-DgcdREGISTER_WRITE_FROM_USER=1",
            "-DCLASS_NAME=\"gpu_class\"",
            "-DgcdGPU_2D_TIMEOUT=20000",
            "-DNO_DMA_COHERENT=1",
            "-DgcdSYS_FREE_MEMORY_LIMIT=51200",
            "-DGALCORE_BUILD_BY=\"$(shell whoami)\"",
            "-DGALCORE_BUILD_TM=\"$(shell date)\"",

            #config
            "-DVIVANTE_ENABLE_3D=1",
            "-DVIVANTE_ENABLE_2D=1",
            "-DVIVANTE_ENABLE_VG=1",
            "-DVIVANTE_ENABLE_DRM=1",
            "-DUSE_PLATFORM_DRIVER=1",
            "-DENABLE_GPU_CLOCK_BY_DRIVER=0",
            "-DCACHE_FUNCTION_UNIMPLEMENTED=0",
            "-DUSE_BANK_ALIGNMENT=1",
            "-DBANK_BIT_START=13",
            "-DBANK_BIT_END=15",
            "-DBANK_CHANNEL_BIT=12",
            #"-DSECURITY=0",
            "-DSOC_PLATFORM=\"freescale-imx\"",

            #CONFIG_SYNC_FILE
            "-DgcdLINUX_SYNC_FILE=1",

            #Kbuild
            "-DLINUX",
            "-DDRIVER",
            "-DDBG=0",
            "-DNO_DMA_COHERENT",
            "-DVIVANTE_PROFILER=1",
            "-DVIVANTE_PROFILER_CONTEXT=1",
            "-DgcdCACHE_FUNCTION_UNIMPLEMENTED=0",
            "-DgcdENABLE_3D=1",
            "-DgcdENABLE_2D=1",
             "-DgcdENABLE_VG=1",
            "-DgcdENABLE_BANK_ALIGNMENT=1",
            "-DgcdBANK_BIT_START=13",
            "-DgcdBANK_BIT_END=15",
            "-DgcdBANK_CHANNEL_BIT=12",
            "-DgcdENABLE_DRM=1",
            "-DgcdENABLE_TRUST_APPLICATION=1",
            "-DHOST=\"$(shell hostname)\"",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "imx8mm_thermal",
        out = "imx8mm_thermal.ko",
        srcs = ["drivers/thermal/imx8mm_thermal.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx7-media-csi",
        out = "imx7-media-csi.ko",
        srcs = ["drivers/media/platform/nxp/imx7-media-csi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-mipi-csis",
        out = "imx-mipi-csis.ko",
        srcs = ["drivers/media/platform/nxp/imx-mipi-csis.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ov5640",
        out = "ov5640.ko",
        srcs = ["drivers/media/i2c/ov5640.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "leds-gpio",
        out = "leds-gpio.ko",
        srcs = ["drivers/leds/leds-gpio.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "leds-pca995x",
        out = "leds-pca995x.ko",
        srcs = ["drivers/leds/leds-pca995x.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpio-pca953x",
        out = "gpio-pca953x.ko",
        srcs = ["drivers/gpio/gpio-pca953x.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-pcm-dma",
        out = "imx-pcm-dma.ko",
        srcs = ["sound/soc/fsl/imx-pcm-dma.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-utils",
        out = "snd-soc-fsl-utils.ko",
        srcs = ["sound/soc/fsl/fsl_utils.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-micfil",
        out = "snd-soc-fsl-micfil.ko",
        srcs = ["sound/soc/fsl/fsl_micfil.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":snd-soc-fsl-utils",
        ],
    )

    ddk_module(
        name = "snd-soc-fsl-asrc",
        out = "snd-soc-fsl-asrc.ko",
        srcs = [
            "sound/soc/fsl/fsl_asrc.c",
            "sound/soc/fsl/fsl_asrc_dma.c",
            "sound/soc/fsl/fsl_asrc_m2m.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-easrc",
        out = "snd-soc-fsl-easrc.ko",
        srcs = ["sound/soc/fsl/fsl_easrc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":snd-soc-fsl-asrc",
        ],
    )

    ddk_module(
        name = "snd-soc-fsl-sai",
        out = "snd-soc-fsl-sai.ko",
        srcs = [
            "sound/soc/fsl/fsl_sai_sysfs.c",
            "sound/soc/fsl/fsl_sai.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        deps = [
            ":snd-soc-fsl-utils",
            ":imx-pcm-dma",
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "snd-soc-fsl-spdif",
        out = "snd-soc-fsl-spdif.ko",
        srcs = ["sound/soc/fsl/fsl_spdif.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        deps = [
            ":snd-soc-fsl-utils",
            ":imx-pcm-dma",
            "busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "snd-soc-imx-audmux",
        out = "snd-soc-imx-audmux.ko",
        srcs = ["sound/soc/fsl/imx-audmux.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-asoc-card",
        out = "snd-soc-fsl-asoc-card.ko",
        srcs = ["sound/soc/fsl/fsl-asoc-card.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":snd-soc-imx-audmux",
            ":snd-soc-simple-card-utils",
        ],
    )

    ddk_module(
        name = "snd-soc-wm8524",
        out = "snd-soc-wm8524.ko",
        srcs = ["sound/soc/codecs/wm8524.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-ak4458",
        out = "snd-soc-ak4458.ko",
        srcs = ["sound/soc/codecs/ak4458.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-ak5558",
        out = "snd-soc-ak5558.ko",
        srcs = ["sound/soc/codecs/ak5558.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-bt-sco",
        out = "snd-soc-bt-sco.ko",
        srcs = ["sound/soc/codecs/bt-sco.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-simple-card",
        out = "snd-soc-simple-card.ko",
        srcs = ["sound/soc/generic/simple-card.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":snd-soc-simple-card-utils",
        ],
    )

    ddk_module(
        name = "snd-soc-simple-card-utils",
        out = "snd-soc-simple-card-utils.ko",
        srcs = ["sound/soc/generic/simple-card-utils.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "rpmsg_ns",
        out = "rpmsg_ns.ko",
        srcs = ["drivers/rpmsg/rpmsg_ns.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "virtio_rpmsg_bus",
        out = "virtio_rpmsg_bus.ko",
        srcs = ["drivers/rpmsg/virtio_rpmsg_bus.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":rpmsg_ns",
        ],
    )

    ddk_module(
        name = "sm-cpu",
        out = "sm-cpu.ko",
        srcs = ["drivers/firmware/imx/sm-cpu.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sm-lmm",
        out = "sm-lmm.ko",
        srcs = ["drivers/firmware/imx/sm-lmm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx_rproc",
        out = "imx_rproc.ko",
        srcs = ["drivers/remoteproc/imx_rproc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":sm-cpu",
            ":sm-lmm",
        ],
    )

    ddk_module(
        name = "i2c-rpmsg-imx",
        out = "i2c-rpmsg-imx.ko",
        srcs = ["drivers/i2c/busses/i2c-rpmsg-imx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-pcm-rpmsg",
        out = "imx-pcm-rpmsg.ko",
        srcs = ["sound/soc/fsl/imx-pcm-rpmsg.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-rpmsg",
        out = "snd-soc-fsl-rpmsg.ko",
        srcs = ["sound/soc/fsl/fsl_rpmsg.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-audio-rpmsg",
        out = "imx-audio-rpmsg.ko",
        srcs = ["sound/soc/fsl/imx-audio-rpmsg.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-rpmsg-ak4497",
        out = "snd-soc-rpmsg-ak4497.ko",
        srcs = ["sound/soc/codecs/rpmsg_ak4497.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-imx-rpmsg",
        out = "snd-soc-imx-rpmsg.ko",
        srcs = ["sound/soc/fsl/imx-rpmsg.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-imx-card",
        out = "snd-soc-imx-card.ko",
        srcs = ["sound/soc/fsl/imx-card.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":snd-soc-simple-card-utils",
        ],
    )

    ddk_module(
        name = "hantrodec_845s",
        out = "hantrodec_845s.ko",
        srcs = ["drivers/mxc/hantro_845/hantrodec_845s.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "hx280enc",
        out = "hx280enc.ko",
        srcs = ["drivers/mxc/hantro_845_h1/hx280enc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "vsiv4l2",
        out = "vsiv4l2.ko",
        srcs = [
            "drivers/mxc/hantro_v4l2/vsi-v4l2.c",
            "drivers/mxc/hantro_v4l2/vsi-v4l2daemon.c",
            "drivers/mxc/hantro_v4l2/vsi-v4l2-config.c",
            "drivers/mxc/hantro_v4l2/vsi-v4l2-dec.c",
            "drivers/mxc/hantro_v4l2/vsi-v4l2-enc.c",
        ],
        copts = [
            "-DV4L2_DRIVER",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        includes = [
            "drivers/mxc/hantro_v4l2",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":memory_usage",
        ],
    )

    ddk_module(
        name = "rtc-snvs",
        out = "rtc-snvs.ko",
        srcs = ["drivers/rtc/rtc-snvs.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "qcom-phy-lib",
        out = "qcom-phy-lib.ko",
        srcs = ["drivers/net/phy/qcom/qcom-phy-lib.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "at803x",
        out = "at803x.ko",
        srcs = ["drivers/net/phy/qcom/at803x.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        deps = [
            ":qcom-phy-lib",
        ],
    )

    ddk_module(
        name = "realtek",
        out = "realtek.ko",
        srcs = ["drivers/net/phy/realtek.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
    )

    ddk_module(
        name = "pps_core",
        out = "pps_core.ko",
        srcs = [
            "drivers/pps/pps.c",
            "drivers/pps/kapi.c",
            "drivers/pps/sysfs.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ptp",
        out = "ptp.ko",
        srcs = [
            "drivers/ptp/ptp_clock.c",
            "drivers/ptp/ptp_chardev.c",
            "drivers/ptp/ptp_sysfs.c",
            "drivers/ptp/ptp_vclock.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fec",
        out = "fec.ko",
        srcs = [
            "drivers/net/ethernet/freescale/fec_main.c",
            "drivers/net/ethernet/freescale/fec_ptp.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "hwmon",
        out = "hwmon.ko",
        srcs = ["drivers/hwmon/hwmon.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "scmi-hwmon",
        out = "scmi-hwmon.ko",
        srcs = ["drivers/hwmon/scmi-hwmon.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":hwmon",
        ],
    )

    ddk_module(
        name = "imx-sm-lmm",
        out = "imx-sm-lmm.ko",
        srcs = ["drivers/firmware/arm_scmi/vendors/imx/imx-sm-lmm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sm-cpu",
        out = "imx-sm-cpu.ko",
        srcs = ["drivers/firmware/arm_scmi/vendors/imx/imx-sm-cpu.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sm-bbm",
        out = "imx-sm-bbm.ko",
        srcs = ["drivers/firmware/arm_scmi/vendors/imx/imx-sm-bbm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sm-misc",
        out = "imx-sm-misc.ko",
        srcs = ["drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "scmi_power_control",
        out = "scmi_power_control.ko",
        srcs = ["drivers/firmware/arm_scmi/scmi_power_control.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "arm_smmu_v3",
        out = "arm_smmu_v3.ko",
        srcs = [
            "drivers/iommu/arm/arm-smmu-v3/arm-smmu-v3.c",
            "drivers/iommu/arm/arm-smmu-v3/arm-smmu-v3-common.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "clk-scmi",
        out = "clk-scmi.ko",
        srcs = ["drivers/clk/clk-scmi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "clk-imx95-blk-ctl",
        out = "clk-imx95-blk-ctl.ko",
        srcs = ["drivers/clk/imx/clk-imx95-blk-ctl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pinctrl-imx-scmi",
        out = "pinctrl-imx-scmi.ko",
        srcs = ["drivers/pinctrl/freescale/pinctrl-imx-scmi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl-edma",
        out = "fsl-edma.ko",
        srcs = [
            "drivers/dma/fsl-edma-main.c",
            "drivers/dma/fsl-edma-common.c",
            "drivers/dma/fsl-edma-trace.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        includes = ["drivers/dma"],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl_lpuart",
        out = "fsl_lpuart.ko",
        srcs = ["drivers/tty/serial/fsl_lpuart.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl_lpuart_95",
        out = "fsl_lpuart_95.ko",
        srcs = ["drivers/tty/serial/fsl_lpuart.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "i2c-imx-lpi2c",
        out = "i2c-imx-lpi2c.ko",
        srcs = ["drivers/i2c/busses/i2c-imx-lpi2c.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "i2c-mux",
        out = "i2c-mux.ko",
        srcs = ["drivers/i2c/i2c-mux.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "irq-imx-irqsteer",
        out = "irq-imx-irqsteer.ko",
        srcs = ["drivers/irqchip/irq-imx-irqsteer.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":trusty-core",
        ],
    )

    ddk_module(
        name = "rtc-imx-sm-bbm",
        out = "rtc-imx-sm-bbm.ko",
        srcs = ["drivers/rtc/rtc-imx-sm-bbm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sm-misc",
        out = "sm-misc.ko",
        srcs = ["drivers/firmware/imx/sm-misc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx7ulp_wdt",
        out = "imx7ulp_wdt.ko",
        srcs = ["drivers/watchdog/imx7ulp_wdt.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "sec_enclave",
        out = "sec_enclave.ko",
        srcs = [
            "drivers/firmware/imx/se_ctrl.c",
            "drivers/firmware/imx/ele_common.c",
            "drivers/firmware/imx/ele_base_msg.c",
            "drivers/firmware/imx/se_msg_sqfl_ctrl.c",
            "drivers/firmware/imx/ele_fw_api.c",
            "drivers/firmware/imx/v2x_base_msg.c",
            "drivers/firmware/imx/v2x_common.c",
            "drivers/firmware/imx/seco_init.c",
            "drivers/firmware/imx/ele_trng.c",
            "drivers/firmware/imx/ele_bbsm.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_8mm_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":imx-mailbox",
        ],
    )

    ddk_module(
        name = "nvmem-imx-ocotp-fsb-s400",
        out = "nvmem-imx-ocotp-fsb-s400.ko",
        srcs = ["drivers/nvmem/imx-ocotp-fsb-s400.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":sec_enclave",
        ],
    )

    ddk_module(
        name = "pwm-imx-tpm",
        out = "pwm-imx-tpm.ko",
        srcs = ["drivers/pwm/pwm-imx-tpm.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "soc-imx9",
        out = "soc-imx9.ko",
        srcs = ["drivers/soc/imx/soc-imx9.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpio-adp5585",
        out = "gpio-adp5585.ko",
        srcs = ["drivers/gpio/gpio-adp5585.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpio-vf610",
        out = "gpio-vf610.ko",
        srcs = ["drivers/gpio/gpio-vf610.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "led_bl",
        out = "led_bl.ko",
        srcs = ["drivers/video/backlight/led_bl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pwm_bl",
        out = "pwm_bl.ko",
        srcs = ["drivers/video/backlight/pwm_bl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "dsp_heap",
        out = "dsp_heap.ko",
        srcs = ["drivers/dma-buf/heaps/dsp_heap.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "maxim_serdes",
        out = "maxim_serdes.ko",
        srcs = ["drivers/mfd/maxim_serdes.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96752-core",
        out = "max96752-core.ko",
        srcs = ["drivers/mfd/max96752-core.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96752-i2c",
        out = "max96752-i2c.ko",
        srcs = ["drivers/mfd/max96752-i2c.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":max96752-core",
            ":i2c-mux",
            ":maxim_serdes",
        ],
    )

    ddk_module(
        name = "adp5585",
        out = "adp5585.ko",
        srcs = ["drivers/mfd/adp5585.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96789-core",
        out = "max96789-core.ko",
        srcs = ["drivers/mfd/max96789-core.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":maxim_serdes",
        ],
    )

    ddk_module(
        name = "max96789-i2c",
        out = "max96789-i2c.ko",
        srcs = ["drivers/mfd/max96789-i2c.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "max96789-core",
            "i2c-mux",
        ],
    )

    ddk_module(
        name = "pwm-adp5585",
        out = "pwm-adp5585.ko",
        srcs = ["drivers/pwm/pwm-adp5585.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "phy-fsl-imx8mq-usb",
        out = "phy-fsl-imx8mq-usb.ko",
        srcs = ["drivers/phy/freescale/phy-fsl-imx8mq-usb.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "focaltech_ts",
        out = "focaltech_ts.ko",
        srcs = [
            "drivers/input/touchscreen/focaltech_core.c",
            "drivers/input/touchscreen/focaltech_i2c.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "fb_notify",
        ],
    )

    ddk_module(
        name = "ilitek_ts_i2c",
        out = "ilitek_ts_i2c.ko",
        srcs = ["drivers/input/touchscreen/ilitek_ts_i2c.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "exc3000",
        out = "exc3000.ko",
        srcs = ["drivers/input/touchscreen/exc3000.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sm-bbm-key",
        out = "imx-sm-bbm-key.ko",
        srcs = ["drivers/input/keyboard/imx-sm-bbm-key.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "dwc3-imx8mp",
        out = "dwc3-imx8mp.ko",
        srcs = ["drivers/usb/dwc3/dwc3-imx8mp.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "busfreq-imx8mq",
        ],
    )

    ddk_module(
        name = "gpio-switch",
        out = "gpio-switch.ko",
        srcs = ["drivers/usb/typec/mux/gpio-switch.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mux-core",
        out = "mux-core.ko",
        srcs = ["drivers/mux/core.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mux-mmio",
        out = "mux-mmio.ko",
        srcs = ["drivers/mux/mmio.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "mux-core",
        ],
    )

    ddk_module(
        name = "phy-fsl-imx9-dphy-rx",
        out = "phy-fsl-imx9-dphy-rx.ko",
        srcs = ["drivers/phy/freescale/phy-fsl-imx9-dphy-rx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "phy-fsl-imx8mp-lvds",
        out = "phy-fsl-imx8mp-lvds.ko",
        srcs = ["drivers/phy/freescale/phy-fsl-imx8mp-lvds.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "it6161",
        out = "it6161.ko",
        srcs = ["drivers/gpu/drm/bridge/it6161.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96752-lvds",
        out = "max96752-lvds.ko",
        srcs = ["drivers/gpu/drm/bridge/max96752-lvds.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "display-connector",
        out = "display-connector.ko",
        srcs = ["drivers/gpu/drm/bridge/display-connector.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl-imx-ldb",
        out = "fsl-imx-ldb.ko",
        srcs = ["drivers/gpu/drm/bridge/fsl-imx-ldb.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96789-dsi",
        out = "max96789-dsi.ko",
        srcs = ["drivers/gpu/drm/bridge/max96789-dsi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "it6263",
        out = "it6263.ko",
        srcs = ["drivers/gpu/drm/bridge/it6263.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ti-sn65dsi83",
        out = "ti-sn65dsi83.ko",
        srcs = ["drivers/gpu/drm/bridge/ti-sn65dsi83.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "nwl-dsi",
        out = "nwl-dsi.ko",
        srcs = ["drivers/gpu/drm/bridge/nwl-dsi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "mux-core",
        ],
    )

    ddk_module(
        name = "lontium-lt8912b",
        out = "lontium-lt8912b.ko",
        srcs = ["drivers/gpu/drm/bridge/lontium-lt8912b.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "lontium-lt9611uxc",
        out = "lontium-lt9611uxc.ko",
        srcs = ["drivers/gpu/drm/bridge/lontium-lt9611uxc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx95-pixel-link",
        out = "imx95-pixel-link.ko",
        srcs = ["drivers/gpu/drm/bridge/imx/imx95-pixel-link.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx95-pixel-interleaver",
        out = "imx95-pixel-interleaver.ko",
        srcs = ["drivers/gpu/drm/bridge/imx/imx95-pixel-interleaver.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-ldb-helper",
        out = "imx-ldb-helper.ko",
        srcs = ["drivers/gpu/drm/bridge/imx/imx-ldb-helper.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx95-ldb",
        out = "imx95-ldb.ko",
        srcs = ["drivers/gpu/drm/bridge/imx/imx95-ldb.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "imx-ldb-helper",
        ],
    )

    ddk_module(
        name = "dw-mipi-dsi",
        out = "dw-mipi-dsi.ko",
        srcs = ["drivers/gpu/drm/bridge/synopsys/dw-mipi-dsi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx95-mipi-dsi",
        out = "imx95-mipi-dsi.ko",
        srcs = ["drivers/gpu/drm/bridge/imx/imx95-mipi-dsi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "mux-core",
            "dw-mipi-dsi",
        ],
    )

    ddk_module(
        name = "imx-lcdif",
        out = "imx-lcdif.ko",
        srcs = [
            "drivers/gpu/drm/mxsfb/lcdif_drv.c",
            "drivers/gpu/drm/mxsfb/lcdif_kms.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "drm_dma_helper",
        ],
    )

    ddk_module(
        name = "panel-nxp-rm67162",
        out = "panel-nxp-rm67162.ko",
        srcs = ["drivers/gpu/drm/panel/panel-nxp-rm67162.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "panel-simple",
        out = "panel-simple.ko",
        srcs = ["drivers/gpu/drm/panel/panel-simple.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "panel-raydium-rm692c9",
        out = "panel-raydium-rm692c9.ko",
        srcs = ["drivers/gpu/drm/panel/panel-raydium-rm692c9.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "panel-rocktech-hx8394f",
        out = "panel-rocktech-hx8394f.ko",
        srcs = ["drivers/gpu/drm/panel/panel-rocktech-hx8394f.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "panel-lvds",
        out = "panel-lvds.ko",
        srcs = ["drivers/gpu/drm/panel/panel-lvds.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "drm_display_helper",
        out = "drm_display_helper.ko",
        srcs = [
            "drivers/gpu/drm/display/drm_display_helper_mod.c",
            "drivers/gpu/drm/display/drm_bridge_connector.c",
            "drivers/gpu/drm/display/drm_hdmi_helper.c",
            "drivers/gpu/drm/display/drm_scdc_helper.c",
            "drivers/gpu/drm/display/drm_hdmi_state_helper.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx95-dpu-drm",
        out = "imx95-dpu-drm.ko",
        srcs = [
            "drivers/gpu/drm/imx/dpu95/dpu95-blit.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-constframe.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-core.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-crtc.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-dither.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-domainblend.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-drv.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-extdst.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-fetcheco.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-fetchlayer.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-fetchyuv.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-fetchunit.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-framegen.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-hscaler.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-kms.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-layerblend.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-plane.c",
            "drivers/gpu/drm/imx/dpu95/dpu95-vscaler.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            "drm_dma_helper",
            "drm_display_helper",
            "trusty-core",
        ],
    )

    ddk_module(
        name = "display-imx-rpmsg",
        out = "display-imx-rpmsg.ko",
        srcs = [
            "drivers/gpu/drm/imx/display-imx-rpmsg.c",
            "drivers/gpu/drm/imx/watch_dial.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx8-isi",
        out = "imx8-isi.ko",
        srcs = [
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-core.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-crossbar.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-gasket.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-hw.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-pipe.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-video.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-debug.c",
            "drivers/media/platform/nxp/imx8-isi/imx8-isi-m2m.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-csi-formatter",
        out = "imx-csi-formatter.ko",
        srcs = ["drivers/media/platform/nxp/imx-csi-formatter.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "dwc-mipi-csi2",
        out = "dwc-mipi-csi2.ko",
        srcs = ["drivers/media/platform/nxp/dwc-mipi-csi2.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ap1302",
        out = "ap1302.ko",
        srcs = ["drivers/media/i2c/ap1302.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ox03c10",
        out = "ox03c10.ko",
        srcs = ["drivers/media/i2c/ox03c10.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "max96717_lib",
        out = "max96717_lib.ko",
        srcs = ["drivers/media/i2c/max96717_lib.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mx95mbcam",
        out = "mx95mbcam.ko",
        srcs = ["drivers/media/i2c/mx95mbcam.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "max96717_lib",
            "ox03c10",
        ],
    )

    ddk_module(
        name = "max96724",
        out = "max96724.ko",
        srcs = ["drivers/media/i2c/max96724.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "i2c-mux",
        ],
    )

    ddk_module(
        name = "libarc4",
        out = "libarc4.ko",
        srcs = ["lib/crypto/arc4.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl_imx9_ddr_perf",
        out = "fsl_imx9_ddr_perf.ko",
        srcs = ["drivers/perf/fsl_imx9_ddr_perf.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx93_adc",
        out = "imx93_adc.ko",
        srcs = ["drivers/iio/adc/imx93_adc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "gpio-reset",
        out = "gpio-reset.ko",
        srcs = ["drivers/reset/gpio-reset.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "imx-sm-reset",
        out = "imx-sm-reset.ko",
        srcs = ["drivers/power/reset/imx-sm-reset.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pci-imx6",
        out = "pci-imx6.ko",
        srcs = ["drivers/pci/controller/dwc/pci-imx6.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spi-fsl-lpspi",
        out = "spi-fsl-lpspi.ko",
        srcs = ["drivers/spi/spi-fsl-lpspi.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mtd",
        out = "mtd.ko",
        srcs = [
            "drivers/mtd/mtdcore.c",
            "drivers/mtd/mtdsuper.c",
            "drivers/mtd/mtdconcat.c",
            "drivers/mtd/mtdpart.c",
            "drivers/mtd/mtdchar.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "spi-nor",
        out = "spi-nor.ko",
        srcs = [
            "drivers/mtd/spi-nor/core.c",
            "drivers/mtd/spi-nor/sfdp.c",
            "drivers/mtd/spi-nor/swp.c",
            "drivers/mtd/spi-nor/otp.c",
            "drivers/mtd/spi-nor/sysfs.c",
            "drivers/mtd/spi-nor/atmel.c",
            "drivers/mtd/spi-nor/eon.c",
            "drivers/mtd/spi-nor/esmt.c",
            "drivers/mtd/spi-nor/everspin.c",
            "drivers/mtd/spi-nor/gigadevice.c",
            "drivers/mtd/spi-nor/intel.c",
            "drivers/mtd/spi-nor/issi.c",
            "drivers/mtd/spi-nor/macronix.c",
            "drivers/mtd/spi-nor/micron-st.c",
            "drivers/mtd/spi-nor/spansion.c",
            "drivers/mtd/spi-nor/sst.c",
            "drivers/mtd/spi-nor/winbond.c",
            "drivers/mtd/spi-nor/xmc.c",
            "drivers/mtd/spi-nor/debugfs.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "mtd",
        ],
    )

    ddk_module(
        name = "leds-pca963x",
        out = "leds-pca963x.ko",
        srcs = ["drivers/leds/leds-pca963x.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "wave6-vpu-ctrl",
        out = "wave6-vpu-ctrl.ko",
        srcs = ["drivers/mxc/vpu/wave6/wave6-vpu-ctrl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "trusty-core",
            "memory_usage",
        ],
    )

    ddk_module(
        name = "wave6",
        out = "wave6.ko",
        srcs = [
            "drivers/mxc/vpu/wave6/wave6-vpu.c",
            "drivers/mxc/vpu/wave6/wave6-vdi.c",
            "drivers/mxc/vpu/wave6/wave6-vpuapi.c",
            "drivers/mxc/vpu/wave6/wave6-vpu-dec.c",
            "drivers/mxc/vpu/wave6/wave6-vpu-enc.c",
            "drivers/mxc/vpu/wave6/wave6-hw.c",
            "drivers/mxc/vpu/wave6/wave6-vpu-v4l2.c",
            "drivers/mxc/vpu/wave6/wave6-vpu-dbg.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        includes = [
            "drivers/mxc/vpu/wave6",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "wave6-vpu-ctrl",
            "trusty-core",
            "memory_usage",
        ],
    )

    ddk_module(
        name = "ox05b1s",
        out = "ox05b1s.ko",
        srcs = [
            "drivers/media/i2c/ox05b1s/ox05b1s_mipi.c",
            "drivers/media/i2c/ox05b1s/ox05b1s_modes.c",
	],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            "v4l2-cci",
        ],
    )

    ddk_module(
        name = "v4l2-jpeg",
        out = "v4l2-jpeg.ko",
        srcs = ["drivers/media/v4l2-core/v4l2-jpeg.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "mxc-jpeg-encdec",
        out = "mxc-jpeg-encdec.ko",
        srcs = [
            "drivers/media/platform/nxp/imx-jpeg/mxc-jpeg.c",
            "drivers/media/platform/nxp/imx-jpeg/mxc-jpeg-hw.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "v4l2-jpeg",
        ],
    )

    ddk_module(
        name = "neoisp",
        out = "neoisp.ko",
        srcs = [
            "drivers/media/platform/nxp/neoisp/neoisp_ctx.c",
            "drivers/media/platform/nxp/neoisp/neoisp_debugfs.c",
            "drivers/media/platform/nxp/neoisp/neoisp_fmt.c",
            "drivers/media/platform/nxp/neoisp/neoisp_main.c",
            "drivers/media/platform/nxp/neoisp/neoisp_nodes.c",
            "drivers/media/platform/nxp/neoisp/neoisp_params.c",
            "drivers/media/platform/nxp/neoisp/neoisp_regs.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-fsl-mqs",
        out = "snd-soc-fsl-mqs.ko",
        srcs = ["sound/soc/fsl/fsl_mqs.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "sm-misc",
        ],
    )

    ddk_module(
        name = "snd-soc-audio-graph-card2",
        out = "snd-soc-audio-graph-card2.ko",
        srcs = ["sound/soc/generic/audio-graph-card2.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "snd-soc-simple-card-utils",
        ],
    )

    ddk_module(
        name = "snd-soc-wm8962",
        out = "snd-soc-wm8962.ko",
        srcs = [
            "sound/soc/codecs/wm8962.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        copts = ["-DCREATE_TRACE_POINTS"],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-wm8904",
        out = "snd-soc-wm8904.ko",
        srcs = [
            "sound/soc/codecs/wm8904.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-cs42xx8",
        out = "snd-soc-cs42xx8.ko",
        srcs = [
            "sound/soc/codecs/cs42xx8.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-cs42xx8-i2c",
        out = "snd-soc-cs42xx8-i2c.ko",
        srcs = [
            "sound/soc/codecs/cs42xx8-i2c.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "snd-soc-cs42xx8",
        ],
    )

    ddk_module(
        name = "crc-itu-t",
        out = "crc-itu-t.ko",
        srcs = ["lib/crc-itu-t.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "aquantia",
        out = "aquantia.ko",
        srcs = [
            "drivers/net/phy/aquantia/aquantia_main.c",
            "drivers/net/phy/aquantia/aquantia_firmware.c",
            "drivers/net/phy/aquantia/aquantia_leds.c",
            "drivers/net/phy/aquantia/aquantia_hwmon.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            "crc-itu-t",
            "hwmon",
        ],
    )

    ddk_module(
        name = "nxp-netc-lib",
        out = "nxp-netc-lib.ko",
        srcs = [
            "drivers/net/ethernet/freescale/enetc/ntmp.c",
            "drivers/net/ethernet/freescale/enetc/netc_tc_lib.c",
            "drivers/net/ethernet/freescale/enetc/netc_debugfs_lib.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "nxp-netc-blk-ctrl",
        out = "nxp-netc-blk-ctrl.ko",
        srcs = ["drivers/net/ethernet/freescale/enetc/netc_blk_ctrl.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "ptp_netc",
        out = "ptp_netc.ko",
        srcs = ["drivers/ptp/ptp_netc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            "nxp-netc-blk-ctrl",
        ],
    )

    ddk_module(
        name = "ptp-qoriq",
        out = "ptp-qoriq.ko",
        srcs = [
            "drivers/ptp/ptp_qoriq.c",
            "drivers/ptp/ptp_qoriq_debugfs.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl-enetc-ptp",
        out = "fsl-enetc-ptp.ko",
        srcs = ["drivers/net/ethernet/freescale/enetc/enetc_ptp.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "ptp-qoriq",
        ],
    )

    ddk_module(
        name = "pcs-lynx",
        out = "pcs-lynx.ko",
        srcs = ["drivers/net/pcs/pcs-lynx.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pcs_xpcs",
        out = "pcs_xpcs.ko",
        srcs = [
            "drivers/net/pcs/pcs-xpcs.c",
            "drivers/net/pcs/pcs-xpcs-plat.c",
            "drivers/net/pcs/pcs-xpcs-nxp.c",
            "drivers/net/pcs/pcs-xpcs-wx.c",
            "drivers/net/pcs/pcs-xpcs-phy.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl-enetc-mdio",
        out = "fsl-enetc-mdio.ko",
        srcs = [
            "drivers/net/ethernet/freescale/enetc/enetc_pci_mdio.c",
            "drivers/net/ethernet/freescale/enetc/enetc_mdio.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "fsl-enetc-core",
        out = "fsl-enetc-core.ko",
        srcs = [
            "drivers/net/ethernet/freescale/enetc/enetc.c",
            "drivers/net/ethernet/freescale/enetc/enetc_cbdr.c",
            "drivers/net/ethernet/freescale/enetc/enetc_ethtool.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "fsl-enetc-mdio",
            "nxp-netc-lib",
            "nxp-netc-blk-ctrl",
            "ptp_netc",
            "fsl-enetc-ptp",
        ],
    )

    ddk_module(
        name = "fsl-enetc-vf",
        out = "fsl-enetc-vf.ko",
        srcs = ["drivers/net/ethernet/freescale/enetc/enetc_vf.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            "fsl-enetc-core",
            "crc-itu-t",
            "fsl-enetc-mdio",
        ],
    )

    ddk_module(
        name = "fsl-enetc4",
        out = "fsl-enetc4.ko",
        srcs = [
            "drivers/net/ethernet/freescale/enetc/enetc4_pf.c",
            "drivers/net/ethernet/freescale/enetc/enetc_pf_common.c",
            "drivers/net/ethernet/freescale/enetc/enetc_devlink.c",
            "drivers/net/ethernet/freescale/enetc/enetc_msg.c",
            "drivers/net/ethernet/freescale/enetc/enetc_qos.c",
            "drivers/net/ethernet/freescale/enetc/enetc_debugfs.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
        deps = [
            "fsl-enetc-core",
            "fsl-enetc-mdio",
            "pcs-lynx",
            "nxp-netc-blk-ctrl",
            "nxp-netc-lib",
            "pcs_xpcs",
            "crc-itu-t",
        ],
    )

    ddk_module(
        name = "imx_neutron_rproc",
        out = "imx_neutron_rproc.ko",
        srcs = ["drivers/remoteproc/imx_neutron_rproc.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "neutron",
        out = "neutron.ko",
        srcs = [
            "drivers/staging/neutron/neutron_driver.c",
            "drivers/staging/neutron/neutron_device.c",
            "drivers/staging/neutron/neutron_buffer.c",
            "drivers/staging/neutron/neutron_inference.c",
            "drivers/staging/neutron/neutron_mailbox.c",
        ],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_headers(
        name = "mali_kbase_headers",
        # Include all .h files.
        hdrs = native.glob([
            "drivers/gpu/arm/midgard/**/*.h",
        ]),
        # Define include paths relative to this BUILD file's directory.
        includes = [
            "drivers/gpu/arm/midgard",
            "drivers/gpu/arm/midgard/platform/imx",
            "drivers/gpu/arm/valhall-inc",
        ],
    )

    ddk_module(
        name = "mali_kbase",
        out = "mali_kbase.ko",
        # Kbuild mali_kbase-y
        srcs = [
            "drivers/gpu/arm/midgard/mali_kbase_cache_policy.c",
            "drivers/gpu/arm/midgard/mali_kbase_ccswe.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem.c",
            "drivers/gpu/arm/midgard/mali_kbase_reg_track.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_migrate.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_pool_group.c",
            "drivers/gpu/arm/midgard/mali_kbase_native_mgm.c",
            "drivers/gpu/arm/midgard/mali_kbase_ctx_sched.c",
            "drivers/gpu/arm/midgard/mali_kbase_gpuprops.c",
            "drivers/gpu/arm/midgard/mali_kbase_pm.c",
            "drivers/gpu/arm/midgard/mali_kbase_config.c",
            "drivers/gpu/arm/midgard/mali_kbase_kinstr_prfcnt.c",
            "drivers/gpu/arm/midgard/mali_kbase_softjobs.c",
            "drivers/gpu/arm/midgard/mali_kbase_hw.c",
            "drivers/gpu/arm/midgard/mali_kbase_debug.c",
            "drivers/gpu/arm/midgard/mali_kbase_gpu_memory_debugfs.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_linux.c",
            "drivers/gpu/arm/midgard/mali_kbase_core_linux.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_profile_debugfs.c",
            "drivers/gpu/arm/midgard/mali_kbase_disjoint_events.c",
            "drivers/gpu/arm/midgard/mali_kbase_debug_mem_view.c",
            "drivers/gpu/arm/midgard/mali_kbase_debug_mem_zones.c",
            "drivers/gpu/arm/midgard/mali_kbase_debug_mem_allocs.c",
            "drivers/gpu/arm/midgard/mali_kbase_smc.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_pool.c",
            "drivers/gpu/arm/midgard/mali_kbase_mem_pool_debugfs.c",
            "drivers/gpu/arm/midgard/mali_kbase_debugfs_helper.c",
            "drivers/gpu/arm/midgard/mali_kbase_as_fault_debugfs.c",
            "drivers/gpu/arm/midgard/mali_kbase_dvfs_debugfs.c",
            "drivers/gpu/arm/midgard/mali_power_gpu_frequency_trace.c",
            "drivers/gpu/arm/midgard/mali_kbase_trace_gpu_mem.c",
            "drivers/gpu/arm/midgard/mali_kbase_pbha.c",
            "drivers/gpu/arm/midgard/mali_kbase_io.c",

            #Conditionally compiling source files
            "drivers/gpu/arm/midgard/mali_kbase_pbha_debugfs.c",        # CONFIG_DEBUG_FS
            "drivers/gpu/arm/midgard/mali_kbase_fence_ops.c",           # CONFIG_SYNC_FILE
            "drivers/gpu/arm/midgard/mali_kbase_sync_file.c",           # CONFIG_SYNC_FILE
            "drivers/gpu/arm/midgard/mali_kbase_sync_common.c",         # CONFIG_SYNC_FILE
            "drivers/gpu/arm/midgard/mali_power_gpu_work_period_trace.c", # CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD
            "drivers/gpu/arm/midgard/mali_kbase_gpu_metrics.c",         # CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD

            #Subdirectory source files
            "drivers/gpu/arm/midgard/arbiter/mali_kbase_arbif.c",
            "drivers/gpu/arm/midgard/arbiter/mali_kbase_arbiter_pm.c",
            "drivers/gpu/arm/midgard/context/mali_kbase_context.c",
            "drivers/gpu/arm/midgard/context/backend/mali_kbase_context_csf.c",
            "drivers/gpu/arm/midgard/debug/mali_kbase_debug_ktrace.c",
            "drivers/gpu/arm/midgard/debug/backend/mali_kbase_debug_ktrace_csf.c",
            "drivers/gpu/arm/midgard/device/mali_kbase_device.c",
            "drivers/gpu/arm/midgard/device/mali_kbase_device_hw.c",
            "drivers/gpu/arm/midgard/device/backend/mali_kbase_device_csf.c",
            "drivers/gpu/arm/midgard/device/backend/mali_kbase_device_hw_csf.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_cache_policy_backend.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_gpuprops_backend.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_irq_linux.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_backend.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_driver.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_metrics.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_ca.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_always_on.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_coarse_demand.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_pm_policy.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_time.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_l2_mmu_config.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_clk_rate_trace_mgr.c",
            "drivers/gpu/arm/midgard/backend/gpu/mali_kbase_devfreq.c",
            "drivers/gpu/arm/midgard/mmu/mali_kbase_mmu.c",
            "drivers/gpu/arm/midgard/mmu/mali_kbase_mmu_hw_direct.c",
            "drivers/gpu/arm/midgard/mmu/mali_kbase_mmu_faults_decoder_luts.c",
            "drivers/gpu/arm/midgard/mmu/mali_kbase_mmu_faults_decoder.c",
            "drivers/gpu/arm/midgard/mmu/mali_kbase_mmu_mode_aarch64.c",
            "drivers/gpu/arm/midgard/mmu/backend/mali_kbase_mmu_csf.c",
            "drivers/gpu/arm/midgard/mmu/backend/mali_kbase_mmu_faults_decoder_luts_csf.c",
            "drivers/gpu/arm/midgard/tl/mali_kbase_timeline.c",
            "drivers/gpu/arm/midgard/tl/mali_kbase_timeline_io.c",
            "drivers/gpu/arm/midgard/tl/mali_kbase_tlstream.c",
            "drivers/gpu/arm/midgard/tl/mali_kbase_tracepoints.c",
            "drivers/gpu/arm/midgard/tl/backend/mali_kbase_timeline_csf.c",
            "drivers/gpu/arm/midgard/hwcnt/mali_kbase_hwcnt.c",
            "drivers/gpu/arm/midgard/hwcnt/mali_kbase_hwcnt_gpu.c",
            "drivers/gpu/arm/midgard/hwcnt/mali_kbase_hwcnt_types.c",
            "drivers/gpu/arm/midgard/hwcnt/mali_kbase_hwcnt_virtualizer.c",
            "drivers/gpu/arm/midgard/hwcnt/mali_kbase_hwcnt_watchdog_if_timer.c",
            "drivers/gpu/arm/midgard/hwcnt/backend/mali_kbase_hwcnt_backend_csf.c",
            "drivers/gpu/arm/midgard/hwcnt/backend/mali_kbase_hwcnt_backend_csf_if_fw.c",
            "drivers/gpu/arm/midgard/gpu/mali_kbase_gpu.c",
            "drivers/gpu/arm/midgard/gpu/backend/mali_kbase_gpu_fault_csf.c",
            "drivers/gpu/arm/midgard/hw_access/mali_kbase_hw_access.c",
            "drivers/gpu/arm/midgard/hw_access/regmap/mali_kbase_regmap_csf.c",
            "drivers/gpu/arm/midgard/hw_access/backend/mali_kbase_hw_access_real_hw.c",
            "drivers/gpu/arm/midgard/thirdparty/mali_kbase_mmap.c",
            "drivers/gpu/arm/midgard/platform/imx/mali_kbase_config_imx.c",
            "drivers/gpu/arm/midgard/platform/imx/mali_kbase_runtime_pm.c",
            "drivers/gpu/arm/midgard/platform/imx/mali_kbase_clk_rate_trace.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_util.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_firmware_cfg.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_trace_buffer.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_scheduler.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_kcpu.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_tiler_heap.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_timeout.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_tl_reader.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_heap_context_alloc.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_reset_gpu.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_csg.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_csg_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_kcpu_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_sync.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_sync_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_kcpu_fence_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_protected_memory.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_tiler_heap_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_cpu_queue.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_cpu_queue_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_event.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_firmware_log.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_firmware_core_dump.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_tiler_heap_reclaim.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_mcu_shared_reg.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_ne_debugfs.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_firmware.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_csf_fw_io.c",
            "drivers/gpu/arm/midgard/csf/mali_kbase_debug_csf_fault.c",
            "drivers/gpu/arm/midgard/csf/ipa_control/mali_kbase_csf_ipa_control.c",
            "drivers/gpu/arm/midgard/ipa/mali_kbase_ipa_simple.c",
            "drivers/gpu/arm/midgard/ipa/mali_kbase_ipa.c",
            "drivers/gpu/arm/midgard/ipa/backend/mali_kbase_ipa_counter_csf.c",
            "drivers/gpu/arm/midgard/ipa/backend/mali_kbase_ipa_counter_common_csf.c",
            "drivers/gpu/arm/midgard/ipa/mali_kbase_ipa_debugfs.c",
        ],
        hdrs = [
            ":imx_common_headers",
            ":mali_kbase_headers",
        ],
        copts = [
            "-DMALI_CUSTOMER_RELEASE=1",
            "-DMALI_KERNEL_TEST_API=0",
            "-DMALI_UNIT_TEST=0",
            "-DMALI_COVERAGE=0",
            "-DMALI_RELEASE_NAME=\"r54p1-11eac0\"",
            "-DMALI_JIT_PRESSURE_LIMIT_BASE=0",
            "-DMALI_PLATFORM_DIR=imx",
            "-DMALI_KBASE_PLATFORM_PATH=platform/imx",

            #"-DCONFIG_MALI_PLATFORM_NAME='\"imx\"'",

            "-Wall", "-Werror",

            "-Wextra", "-Wunused", "-Wno-unused-parameter",
            "-Wmissing-declarations",
            "-Wmissing-format-attribute",
            "-Wmissing-prototypes",
            "-Wold-style-definition",
            "-Wno-unused-function",

            "-Wno-sign-compare",
            "-Wno-shift-negative-value",
            "-Wno-cast-function-type",
            "-Wno-ignored-qualifiers",
            "-Wno-type-limits",

            "-Wdisabled-optimization",
            "-Wmissing-field-initializers",
            "-Wunused-macros",
            "-Wframe-larger-than=4096",

            "-DKBUILD_EXTRA_WARN1",
            "-DKBUILD_EXTRA_WARN2",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "snd-soc-dmic",
        out = "snd-soc-dmic.ko",
        srcs = ["sound/soc/codecs/dmic.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "pwm-fan",
        out = "pwm-fan.ko",
        srcs = ["drivers/hwmon/pwm-fan.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
        deps = [
            ":hwmon",
        ],
    )

    ddk_module(
        name = "memory_usage",
        out = "memory_usage.ko",
        srcs = ["drivers/mxc/vpu/memory_usage/memory_usage.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kernel_build = ":imx_ddk_modules",
    )

    ddk_module(
        name = "v4l2-cci",
        out = "v4l2-cci.ko",
        srcs = ["drivers/media/v4l2-core/v4l2-cci.c"],
        hdrs = [
            ":imx_common_headers",
        ],
        kconfig = "imx_kconfigs",
        defconfig = "imx_evk_95_module.fragment",
        kernel_build = ":imx_ddk_modules",
    )

    kernel_build(
        name = "imx_ddk_modules",
        srcs = ["//common:kernel_aarch64_sources"],
        outs = [],
        base_kernel = "//common:kernel_aarch64",
        kmi_symbol_list = "//common:gki/aarch64/symbols/imx",
        build_config = "build.config.imx_ddk",
        make_goals = [
            "modules",
        ],
        makefile = "//common:Makefile",
        kbuild_symtypes="true",
        module_outs = _IMX_COMMON_MODULES,
        post_defconfig_fragments = [
            "imx_core.fragment",
        ],
        visibility = [
            "//nxp-mwifiex:__pkg__",
            "//verisilicon_sw_isp_vvcam/vvcam:__pkg__",
        ],
    )

    kernel_abi(
        name = "imx_ddk_modules_abi",
        kernel_build = ":imx_ddk_modules",
        kernel_modules = [
            ":imx_evk_8mm_vendor_boot_modules",
            ":imx_evk_8mm_vendor_dlkm_modules",
            ":imx_evk_95_vendor_boot_modules",
            ":imx_evk_95_vendor_dlkm_modules",
        ],
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )
