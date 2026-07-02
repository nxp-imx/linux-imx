# SPDX-License-Identifier: GPL-2.0
# Copyright 2026 NXP
# Bazel build configuration for i.MX 95 (evk_95)

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
# i.MX 95 Module Lists
# Extracted from device/nxp/imx9/evk_95/SharedBoardConfig.mk
# =============================================================================

# IMX_ANDROID_FIRST_STAGE_MODULES - modules loaded at first stage boot
_IMX95_FIRST_STAGE_MODULES = [
    # HWMON & SCMI
    "drivers/hwmon/hwmon.ko",
    "drivers/hwmon/scmi-hwmon.ko",
    "drivers/pmdomain/arm/scmi_pm_domain.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-lmm.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-cpu.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-bbm.ko",
    "drivers/firmware/arm_scmi/vendors/imx/imx-sm-misc.ko",
    "drivers/firmware/arm_scmi/scmi_power_control.ko",

    # IOMMU
    "drivers/iommu/arm/arm-smmu-v3/arm_smmu_v3.ko",

    # Clock
    "drivers/clk/clk-scmi.ko",
    "drivers/clk/imx/mxc-clk.ko",
    "drivers/clk/imx/clk-imx95-blk-ctl.ko",

    # Timer & Mailbox & RPMSG
    "drivers/clocksource/timer-imx-sysctr.ko",
    "drivers/mailbox/imx-mailbox.ko",
    "drivers/rpmsg/rpmsg_ns.ko",
    "drivers/rpmsg/virtio_rpmsg_bus.ko",

    # Firmware & Remoteproc
    "drivers/firmware/imx/sm-cpu.ko",
    "drivers/firmware/imx/sm-lmm.ko",
    "drivers/remoteproc/imx_rproc.ko",

    # Pinctrl
    "drivers/pinctrl/freescale/pinctrl-imx.ko",
    "drivers/pinctrl/freescale/pinctrl-imx-scmi.ko",

    # DMA
    "drivers/dma/fsl-edma.ko",

    # Serial
    "drivers/tty/serial/fsl_lpuart.ko",

    # Trusty
    "drivers/trusty/trusty-smc.ko",
    "drivers/trusty/trusty-core.ko",
    "drivers/trusty/trusty-log.ko",
    "drivers/trusty/trusty-ipc.ko",
    "drivers/trusty/trusty-virtio.ko",

    # I2C
    "drivers/i2c/busses/i2c-imx-lpi2c.ko",
    "drivers/i2c/i2c-dev.ko",
    "drivers/i2c/busses/i2c-rpmsg-imx.ko",
    "drivers/i2c/i2c-mux.ko",

    # I3C
    "drivers/i3c/master/svc-i3c-master.ko",

    # IRQ & RTC
    "drivers/irqchip/irq-imx-irqsteer.ko",
    "drivers/rtc/rtc-imx-sm-bbm.ko",

    # Misc firmware
    "drivers/firmware/imx/sm-misc.ko",

    # CPUFreq & Watchdog
    "drivers/cpufreq/cpufreq-dt.ko",
    "drivers/watchdog/imx7ulp_wdt.ko",

    # Security
    "drivers/firmware/imx/sec_enclave.ko",

    # MMC
    "drivers/mmc/host/cqhci.ko",
    "drivers/soc/imx/busfreq-imx8mq.ko",
    "drivers/mmc/host/sdhci-esdhc-imx.ko",

    # NVMEM
    "drivers/nvmem/nvmem-imx-ocotp.ko",
    "drivers/nvmem/nvmem-imx-ocotp-ele.ko",

    # Power sequence & PWM
    "drivers/mmc/core/pwrseq_simple.ko",
    "drivers/pwm/pwm-imx-tpm.ko",

    # SoC & GPIO
    "drivers/soc/imx/soc-imx9.ko",
    "drivers/gpio/gpio-adp5585.ko",
    "drivers/gpio/gpio-pca953x.ko",
    "drivers/gpio/gpio-vf610.ko",
]

# IMX_RECOVERY_FIRST_STAGE_ADDITION_MODULES - additional modules for recovery
_IMX95_RECOVERY_ADDITION_MODULES = [
    # Backlight
    "drivers/video/backlight/led_bl.ko",
    "drivers/video/backlight/pwm_bl.ko",

    # DMA buffer heaps
    "drivers/dma-buf/heaps/system_heap.ko",
    "drivers/dma-buf/heaps/dsp_heap.ko",
    "drivers/dma-buf/heaps/cma_heap.ko",
    "drivers/dma-buf/dma-buf-imx.ko",

    # MFD
    "drivers/mfd/maxim_serdes.ko",
    "drivers/mfd/max96752-core.ko",
    "drivers/mfd/max96752-i2c.ko",
    "drivers/mfd/adp5585.ko",
    "drivers/mfd/max96789-core.ko",
    "drivers/mfd/max96789-i2c.ko",

    # Power & PWM
    "drivers/power/supply/dummy_battery.ko",
    "drivers/pwm/pwm-adp5585.ko",

    # PHY
    "drivers/phy/freescale/phy-fsl-imx8mq-usb.ko",

    # Framebuffer notify
    "drivers/video/fbdev/core/fb_notify.ko",

    # Input
    "drivers/input/touchscreen/focaltech_ts.ko",
    "drivers/input/touchscreen/ilitek_ts_i2c.ko",
    "drivers/input/touchscreen/exc3000.ko",
    "drivers/input/touchscreen/goodix_ts.ko",
    "drivers/input/keyboard/imx-sm-bbm-key.ko",

    # USB
    "drivers/usb/chipidea/usbmisc_imx.ko",
    "drivers/usb/common/ulpi.ko",
    "drivers/usb/chipidea/ci_hdrc_imx.ko",
    "drivers/usb/chipidea/ci_hdrc.ko",
    "drivers/usb/phy/phy-generic.ko",
    "drivers/usb/dwc3/dwc3-imx8mp.ko",
    "drivers/usb/typec/mux/gpio-switch.ko",

    # Mux
    "drivers/mux/mux-core.ko",
    "drivers/mux/mux-mmio.ko",

    # PHY (display)
    "drivers/phy/freescale/phy-fsl-imx9-dphy-rx.ko",
    "drivers/phy/freescale/phy-fsl-imx8mp-lvds.ko",

    # Framebuffer core
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

    # Display MU
    "drivers/firmware/imx/disp-mu.ko",

    # DRM core
    "drivers/gpu/drm/clients/drm_client_lib.ko",
    "drivers/gpu/drm/drm_dma_helper.ko",
    "drivers/gpu/drm/drm_fbdev_helper.ko",

    # DRM bridge
    "drivers/gpu/drm/bridge/it6161.ko",
    "drivers/gpu/drm/bridge/max96752-lvds.ko",
    "drivers/gpu/drm/bridge/display-connector.ko",
    "drivers/gpu/drm/bridge/adv7511/adv7511.ko",
    "drivers/gpu/drm/bridge/fsl-imx-ldb.ko",
    "drivers/gpu/drm/bridge/max96789-dsi.ko",
    "drivers/gpu/drm/bridge/it6263.ko",
    "drivers/gpu/drm/bridge/ti-sn65dsi83.ko",
    "drivers/gpu/drm/bridge/nwl-dsi.ko",
    "drivers/gpu/drm/bridge/lontium-lt8912b.ko",
    "drivers/gpu/drm/bridge/lontium-lt9611uxc.ko",
    "drivers/gpu/drm/bridge/waveshare-dsi.ko",
    "drivers/gpu/drm/bridge/imx/imx95-pixel-link.ko",
    "drivers/gpu/drm/bridge/imx/imx95-pixel-interleaver.ko",
    "drivers/gpu/drm/bridge/imx/imx-ldb-helper.ko",
    "drivers/gpu/drm/bridge/imx/imx95-ldb.ko",
    "drivers/gpu/drm/bridge/synopsys/dw-mipi-dsi.ko",
    "drivers/gpu/drm/bridge/imx/imx95-mipi-dsi.ko",

    # DRM LCDIF
    "drivers/gpu/drm/mxsfb/imx-lcdif.ko",

    # DRM panel
    "drivers/gpu/drm/panel/panel-raydium-rm67191.ko",
    "drivers/gpu/drm/panel/panel-simple.ko",
    "drivers/gpu/drm/panel/panel-raydium-rm692c9.ko",
    "drivers/gpu/drm/panel/panel-rocktech-hx8394f.ko",
    "drivers/gpu/drm/panel/panel-lvds.ko",

    # DRM display helper
    "drivers/gpu/drm/display/drm_display_helper.ko",

    # DRM IMX DPU95
    "drivers/gpu/drm/imx/dpu95/imx95-dpu-drm.ko",
    "drivers/gpu/drm/imx/display-imx-rpmsg.ko",
]

# Combined vendor ramdisk modules (BOARD_VENDOR_RAMDISK_KERNEL_MODULES)
# = IMX_ANDROID_FIRST_STAGE_MODULES + IMX_RECOVERY_FIRST_STAGE_ADDITION_MODULES
_IMX95_VENDOR_RAMDISK_MODULES = _IMX95_FIRST_STAGE_MODULES + _IMX95_RECOVERY_ADDITION_MODULES

# Mali GPU modules (USE_GPU_DRIVERS=mali)
_IMX95_MALI_GPU_MODULES = [
    "drivers/gpu/arm/pma/protected_memory_allocator.ko",
    "drivers/gpu/arm/pma/protected_heap.ko",
    "drivers/gpu/arm/midgard/mali_kbase.ko",
]

# BOARD_VENDOR_KERNEL_MODULES - modules loaded later during boot (vendor_dlkm)
# Note: vendor_dlkm also includes _IMX95_RECOVERY_ADDITION_MODULES
_IMX95_VENDOR_DLKM_MODULES = [
    # Media / Camera
    "drivers/media/platform/nxp/imx8-isi/imx8-isi.ko",
    "drivers/media/platform/nxp/imx-csi-formatter.ko",
    "drivers/media/platform/nxp/dwc-mipi-csi2.ko",

    # Camera
    "drivers/media/i2c/ap1302.ko",
    "drivers/media/i2c/ox03c10.ko",
    "drivers/media/i2c/max96717_lib.ko",
    "drivers/media/i2c/mx95mbcam.ko",
    "drivers/media/i2c/max96724.ko",

    # Wireless
    "net/wireless/cfg80211.ko",
    "lib/crypto/libarc4.ko",
    "net/mac80211/mac80211.ko",

    # Perf & ADC
    "drivers/perf/fsl_imx9_ddr_perf.ko",
    "drivers/iio/adc/imx93_adc.ko",

    # Power
    "drivers/power/reset/imx-sm-reset.ko",

    # PCIe
    "drivers/pci/controller/dwc/pci-imx6.ko",

    # SPI & MTD
    "drivers/spi/spidev.ko",
    "drivers/spi/spi-bitbang.ko",
    "drivers/spi/spi-nxp-fspi.ko",
    "drivers/spi/spi-fsl-lpspi.ko",
    "drivers/mtd/mtd.ko",
    "drivers/mtd/spi-nor/spi-nor.ko",

    # LEDs
    "drivers/leds/leds-gpio.ko",
    "drivers/leds/leds-pca995x.ko",
    "drivers/leds/leds-pca963x.ko",

    # VPU
    "drivers/mxc/vpu/memory_usage/memory_usage.ko",
    "drivers/mxc/vpu/wave6/wave6-vpu-ctrl.ko",
    "drivers/mxc/vpu/wave6/wave6.ko",

    # Media
    "drivers/media/v4l2-core/v4l2-cci.ko",
    "drivers/media/i2c/ox05b1s/ox05b1s.ko",
    "drivers/media/v4l2-core/v4l2-jpeg.ko",
    "drivers/media/platform/nxp/imx-jpeg/mxc-jpeg-encdec.ko",
    "drivers/media/platform/nxp/neoisp/neoisp.ko",
    "drivers/media/v4l2-core/v4l2-isp.ko",

    # Audio
    "sound/soc/fsl/imx-pcm-dma.ko",
    "sound/soc/fsl/imx-pcm-rpmsg.ko",
    "sound/soc/fsl/snd-soc-fsl-utils.ko",
    "sound/soc/codecs/snd-soc-dmic.ko",
    "sound/soc/fsl/snd-soc-fsl-micfil.ko",
    "sound/soc/fsl/snd-soc-fsl-mqs.ko",
    "sound/soc/fsl/snd-soc-fsl-asrc.ko",
    "sound/soc/fsl/snd-soc-fsl-sai.ko",
    "sound/soc/codecs/snd-soc-bt-sco.ko",
    "sound/soc/generic/snd-soc-simple-card-utils.ko",
    "sound/soc/generic/snd-soc-simple-card.ko",
    "sound/soc/generic/snd-soc-audio-graph-card2.ko",
    "sound/soc/fsl/snd-soc-imx-card.ko",
    "sound/soc/fsl/snd-soc-imx-audmux.ko",
    "sound/soc/fsl/snd-soc-imx-rpmsg.ko",
    "sound/soc/fsl/snd-soc-fsl-asoc-card.ko",
    "sound/soc/fsl/imx-audio-rpmsg.ko",
    "sound/soc/fsl/snd-soc-fsl-rpmsg.ko",
    "sound/soc/codecs/snd-soc-ak4458.ko",
    "sound/soc/codecs/snd-soc-ak5558.ko",
    "sound/soc/codecs/snd-soc-wm8962.ko",
    "sound/soc/codecs/snd-soc-wm8904.ko",
    "sound/soc/codecs/snd-soc-cs42xx8.ko",
    "sound/soc/codecs/snd-soc-cs42xx8-i2c.ko",

    # SOF (Sound Open Firmware)
    "drivers/firmware/imx/imx-dsp.ko",
    "sound/soc/sof/snd-sof-utils.ko",
    "sound/soc/sof/snd-sof.ko",
    "sound/soc/sof/snd-sof-of.ko",
    "sound/soc/sof/xtensa/snd-sof-xtensa-dsp.ko",
    "sound/soc/sof/imx/imx-common.ko",
    "sound/soc/sof/imx/snd-sof-imx9.ko",

    # CAN
    "drivers/net/can/flexcan/flexcan.ko",

    # Ethernet PHY & ENETC
    "drivers/net/phy/aquantia/aquantia.ko",
    "drivers/net/ethernet/freescale/enetc/nxp-netc-lib.ko",
    "drivers/net/ethernet/freescale/enetc/nxp-netc-blk-ctrl.ko",
    "drivers/ptp/ptp_netc.ko",
    "drivers/ptp/ptp_qoriq.ko",
    "drivers/net/ethernet/freescale/enetc/fsl-enetc-ptp.ko",
    "drivers/net/pcs/pcs_xpcs.ko",
    "drivers/net/ethernet/freescale/enetc/fsl-enetc-mdio.ko",
    "drivers/net/ethernet/freescale/enetc/fsl-enetc-core.ko",
    "lib/crc/crc-itu-t.ko",
    "drivers/net/ethernet/freescale/enetc/nxp-enetc-pf-common.ko",
    "drivers/net/ethernet/freescale/enetc/fsl-enetc-vf.ko",
    "drivers/net/ethernet/freescale/enetc/nxp-enetc4.ko",
    "drivers/net/phy/realtek/realtek.ko",

    # HWMON
    "drivers/hwmon/pwm-fan.ko",

    # Neutron
    "drivers/remoteproc/imx_neutron_rproc.ko",
    "drivers/staging/neutron/neutron.ko",
]

# External modules to include in vendor_dlkm.img
# WiFi modules loaded after cfg80211/mac80211
_IMX95_EXT_VENDOR_DLKM_MODULES = [
    "mlan.ko",
    "moal.ko",
]

# Mesa GPU modules (USE_GPU_DRIVERS=mesa)
_IMX95_MESA_GPU_MODULES = [
    "drivers/gpu/drm/drm_exec.ko",
    "drivers/gpu/drm/drm_gpuvm.ko",
    "drivers/gpu/drm/scheduler/gpu-sched.ko",
    "drivers/gpu/drm/panthor/panthor.ko",
]

# Combined list of all in-tree modules for kernel_build
_IMX95_IN_TREE_MODULES = _IMX95_VENDOR_RAMDISK_MODULES + _IMX95_VENDOR_DLKM_MODULES + _IMX95_MALI_GPU_MODULES

# Implicit modules (optional - build continues if missing)
# These are Kconfig dependencies that may or may not be built
_IMX95_IMPLICIT_MODULES = [
    "sound/soc/fsl/snd-soc-fsl-esai.ko",
    "drivers/mfd/wm8994.ko",
    "net/sched/sch_cbs.ko",
    "drivers/rpmsg/imx_rpmsg_tty.ko",
    "drivers/soc/imx/imx93-src.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mn.ko",
    "net/sched/sch_mqprio_lib.ko",
    "drivers/rpmsg/imx_rpmsg.ko",
    "sound/soc/codecs/snd-soc-wm8994.ko",
    "net/sched/sch_etf.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mm.ko",
    "sound/soc/codecs/snd-soc-hdmi-codec.ko",
    "sound/soc/fsl/snd-soc-fsl-ssi.ko",
    "drivers/usb/chipidea/ci_hdrc_tegra.ko",
    "drivers/irqchip/irq-imx-mu-msi.ko",
    "drivers/pmdomain/imx/imx93-pd.ko",
    "drivers/video/fbdev/mxc/mxc_edid.ko",
    "drivers/trusty/trusty-test.ko",
    "drivers/mtd/parsers/ofpart.ko",
    "drivers/cpufreq/cpufreq-dt-platdev.ko",
    "net/sched/act_gate.ko",
    "drivers/gpu/drm/drm_gpuvm.ko",
    "drivers/gpu/drm/scheduler/gpu-sched.ko",
    "sound/soc/codecs/snd-soc-wm-hubs.ko",
    "drivers/trusty/trusty-populate.ko",
    "net/sched/sch_taprio.ko",
    "drivers/gpu/drm/panthor/panthor.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8ulp.ko",
    "drivers/usb/chipidea/ci_hdrc_msm.ko",
    "net/sched/sch_mqprio.ko",
    "drivers/usb/chipidea/ci_hdrc_usb2.ko",
    "drivers/usb/chipidea/ci_hdrc_npcm.ko",
    "sound/soc/fsl/snd-soc-fsl-spdif.ko",
    "drivers/cpufreq/imx-cpufreq-dt.ko",
    "drivers/soc/imx/soc-imx8m.ko",
    "drivers/mtd/chips/chipreg.ko",
    "drivers/gpu/drm/drm_exec.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mp.ko",
    "drivers/rpmsg/imx_rpmsg_chre.ko",
    "drivers/gpu/drm/bridge/ite-it6263.ko",
    "drivers/pinctrl/freescale/pinctrl-imx8mq.ko",
    "net/sched/cls_flower.ko",
    "drivers/pinctrl/freescale/pinctrl-imx93.ko",
    "drivers/rpmsg/imx_rpmsg_pingpong.ko",
    "sound/soc/codecs/snd-soc-tlv320aic31xx.ko",
]

# External modules (built separately)
_IMX95_EXT_MODULES = [
    "//nxp-mwifiex:mwifiex_modules_imx95",
]

# =============================================================================
# i.MX 95 Device Tree Blobs (DTB)
# Extracted from device/nxp/imx9/evk_95/BoardConfig.mk TARGET_BOARD_DTS_CONFIG
# =============================================================================

# All DTB files needed for DTBO image generation
# Format: arch/arm64/boot/dts/freescale/<filename>.dtb
_IMX95_DTB_OUTS = [
    # 19x19 EVK variants
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-os08a20-isp-adv7535.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-ox03c10-isp-adv7535.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-adv7535-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-adv7535-it6263-lvds1-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-rm692c9.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-os08a20-isp-it6263-lvds0.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-dual-os08a20-isp-it6263-lvds0.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-os08a20-isp-lvds-two-disp.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-jdi-wuxga-lvds-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-os08a20-isp-adv7535-cs42888.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-adv7535-rpmsg.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-lt9611uxc-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-dsi-serdes.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-evk-sof-wm8962-adv7535.dtb",

    # 15x15 EVK variants
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-os08a20-isp-adv7535.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-ox03c10-isp-adv7535.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-adv7535-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-rm692c9.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-adv7535-aud-hat.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-adv7535-mqs.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-lt9611uxc-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-evk-boe-wxga-lvds1-panel.dtb",

    # 15x15 FRDM board variants
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-os08a20-isp.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-ap1302.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-dual-os08a20-isp.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-boe-wxga-lvds-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-waveshare-7inch-c-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx95-15x15-frdm-aud-hat.dtb",

    # 19x19 FRDM Pro board variants
    "arch/arm64/boot/dts/freescale/imx95-19x19-frdm-pro-os08a20-isp.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-frdm-pro-dual-os08a20-isp.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-frdm-pro-waveshare-7inch-c-panel.dtb",
    "arch/arm64/boot/dts/freescale/imx95-19x19-frdm-pro-aud-hat.dtb",
]

# Module load lists for initramfs
# BOARD_VENDOR_RAMDISK_KERNEL_MODULES_LOAD (normal boot, first stage only)
_IMX95_MODULES_LIST = _IMX95_FIRST_STAGE_MODULES

# BOARD_VENDOR_RAMDISK_RECOVERY_KERNEL_MODULES_LOAD (recovery mode, all vendor ramdisk)
_IMX95_MODULES_RECOVERY_LIST = _IMX95_VENDOR_RAMDISK_MODULES

# Charger mode module list (same as recovery)
_IMX95_MODULES_CHARGER_LIST = _IMX95_VENDOR_RAMDISK_MODULES


def define_imx95():
    """Define Bazel targets for i.MX 95 kernel build."""

    # ==========================================================================
    # Module list files for initramfs and vendor_dlkm
    # ==========================================================================

    # Modules list for normal boot (first stage only)
    write_file(
        name = "imx95_modules_list",
        out = "imx95_modules.txt",
        content = [m.split("/")[-1] for m in _IMX95_MODULES_LIST] + [""],
    )

    # Modules list for recovery mode
    write_file(
        name = "imx95_modules_recovery_list",
        out = "imx95_modules_recovery.txt",
        content = [m.split("/")[-1] for m in _IMX95_MODULES_RECOVERY_LIST] + [""],
    )

    # Modules list for charger mode
    write_file(
        name = "imx95_modules_charger_list",
        out = "imx95_modules_charger.txt",
        content = [m.split("/")[-1] for m in _IMX95_MODULES_CHARGER_LIST] + [""],
    )

    # Modules list for vendor ramdisk (initramfs)
    # Combines first stage + recovery modules
    write_file(
        name = "imx95_vendor_ramdisk_modules_list",
        out = "imx95_vendor_ramdisk_modules.txt",
        content = [m.split("/")[-1] for m in _IMX95_VENDOR_RAMDISK_MODULES] + [""],
    )

    # Explicit module load order for vendor ramdisk
    # Matches the order defined in _IMX95_MODULES_LIST
    write_file(
        name = "imx95_modules_load_order",
        out = "imx95_modules.load",
        content = [m.split("/")[-1] for m in _IMX95_MODULES_LIST] + [""],
    )

    # Modules list for vendor_dlkm
    # Combines recovery + vendor dlkm + GPU + external WiFi modules
    write_file(
        name = "imx95_vendor_dlkm_modules_list",
        out = "imx95_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX95_RECOVERY_ADDITION_MODULES + _IMX95_VENDOR_DLKM_MODULES + _IMX95_MALI_GPU_MODULES] + _IMX95_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm
    # Matches the order defined in recovery + vendor dlkm + GPU modules
    write_file(
        name = "imx95_vendor_dlkm_modules_load_order",
        out = "imx95_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX95_RECOVERY_ADDITION_MODULES + _IMX95_VENDOR_DLKM_MODULES + _IMX95_MALI_GPU_MODULES] + _IMX95_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Modules list for vendor_dlkm (mesa GPU mode)
    write_file(
        name = "imx95_mesa_vendor_dlkm_modules_list",
        out = "imx95_mesa_vendor_dlkm_modules.txt",
        content = [m.split("/")[-1] for m in _IMX95_RECOVERY_ADDITION_MODULES + _IMX95_VENDOR_DLKM_MODULES + _IMX95_MESA_GPU_MODULES] + _IMX95_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # Explicit module load order for vendor_dlkm (mesa GPU mode)
    write_file(
        name = "imx95_mesa_vendor_dlkm_modules_load_order",
        out = "imx95_mesa_vendor_dlkm_modules.load",
        content = [m.split("/")[-1] for m in _IMX95_RECOVERY_ADDITION_MODULES + _IMX95_VENDOR_DLKM_MODULES + _IMX95_MESA_GPU_MODULES] + _IMX95_EXT_VENDOR_DLKM_MODULES + [""],
    )

    # ==========================================================================
    # Kernel build
    # ==========================================================================

    kernel_build(
        name = "imx95",
        srcs = [":common_kernel_sources"],
        outs = [
            "Image",
            "Image.lz4",
        ] + _IMX95_DTB_OUTS,
        arch = "arm64",
        # Mixed build: use GKI as base
        base_kernel = ":kernel_aarch64",
        # Use gki_defconfig + imx95 fragment
        defconfig = "arch/arm64/configs/gki_defconfig",
        pre_defconfig_fragments = [
            "arch/arm64/configs/imx95_gki.fragment",
        ],
        make_goals = [
            "Image",
            "Image.lz4",
            "modules",
            "dtbs",
        ],
        makefile = ":Makefile",
        # In-tree modules (required - build fails if missing)
        module_outs = _IMX95_IN_TREE_MODULES,
        # Implicit modules (optional - build continues if missing)
        # These are Kconfig dependencies that may or may not be built
        module_implicit_outs = _IMX95_IMPLICIT_MODULES,
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
        name = "imx95_abi",
        kernel_build = ":imx95",
        kernel_modules = _IMX95_EXT_MODULES,
        module_grouping = False,
        kmi_symbol_list_add_only = True,
    )

    kernel_modules_install(
        name = "imx95_modules_install",
        kernel_build = ":imx95",
        kernel_modules = _IMX95_EXT_MODULES,
    )

    # ==========================================================================
    # Initramfs and vendor images
    # ==========================================================================

    # Initramfs for vendor_boot.img
    # Contains _IMX95_VENDOR_RAMDISK_MODULES with separate load lists per boot mode
    initramfs(
        name = "imx95_initramfs",
        kernel_modules_install = ":imx95_modules_install",
        ramdisk_compression = "lz4",
        # Normal boot: first stage modules only
        modules_list = ":imx95_modules_list",
        # Explicit load order matching _IMX95_MODULES_LIST
        modules_load = ":imx95_modules_load_order",
        # Recovery mode: all vendor ramdisk modules
        modules_recovery_list = ":imx95_modules_recovery_list",
        # Charger mode: same as recovery
        modules_charger_list = ":imx95_modules_charger_list",
        # Remove modules not in modules_list from initramfs
        trim_unused_modules = True,
        vendor_boot_name = "vendor_boot",
    )

    # vendor_boot.img - contains vendor ramdisk with ramdisk.lz4
    vendor_boot_image(
        name = "imx95_vendor_boot",
        outs = [
            "ramdisk.lz4",
        ],
        initramfs = ":imx95_initramfs",
        kernel_build = ":imx95",
        unpack_ramdisk = True,
        ramdisk_compression = "lz4",
        vendor_boot_name = "vendor_boot",
    )

    # vendor_dlkm.img - contains recovery + vendor dlkm + Mali GPU modules
    vendor_dlkm_image(
        name = "imx95_vendor_dlkm",
        kernel_modules_install = ":imx95_modules_install",
        # Only include vendor dlkm modules
        modules_list = ":imx95_vendor_dlkm_modules_list",
        # Explicit load order matching recovery + vendor dlkm + GPU modules
        modules_load = ":imx95_vendor_dlkm_modules_load_order",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx95_initramfs",
        fs_type = "erofs",
    )

    # vendor_dlkm.img (mesa) - contains recovery + vendor dlkm + Mesa GPU modules
    vendor_dlkm_image(
        name = "imx95_mesa_vendor_dlkm",
        kernel_modules_install = ":imx95_modules_install",
        # Include vendor dlkm + Mesa GPU modules (panthor, drm_exec, etc.)
        modules_list = ":imx95_mesa_vendor_dlkm_modules_list",
        # Explicit load order matching recovery + vendor dlkm + Mesa GPU modules
        modules_load = ":imx95_mesa_vendor_dlkm_modules_load_order",
        # Strip modules already in initramfs to avoid duplication
        vendor_boot_modules_load = ":imx95_initramfs",
        fs_type = "erofs",
    )

    # ==========================================================================
    # Distribution file groups
    # ==========================================================================

    # Kernel image and modules (shared base)
    pkg_files(
        name = "imx95_kernel_files",
        srcs = [
            ":imx95",
            ":imx95_modules_install",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor boot (vendor_boot.img + ramdisk.lz4)
    pkg_files(
        name = "imx95_vendor_boot_files",
        srcs = [
            ":imx95_vendor_boot",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM (Mali GPU — default)
    pkg_files(
        name = "imx95_vendor_dlkm_files",
        srcs = [
            ":imx95_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # Vendor DLKM (Mesa GPU — USE_GPU_DRIVERS=mesa)
    pkg_files(
        name = "imx95_mesa_vendor_dlkm_files",
        srcs = [
            ":imx95_mesa_vendor_dlkm",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # DTB files for DTBO image generation
    # Directly list DTB output files from kernel_build
    _dtb_srcs = [
        ":imx95/" + f for f in _IMX95_DTB_OUTS
    ]

    pkg_files(
        name = "imx95_dtb_files",
        srcs = _dtb_srcs,
        strip_prefix = strip_prefix.from_pkg("imx95/arch/arm64/boot/dts/freescale"),
        visibility = ["//visibility:private"],
    )

    # GKI boot.img variants
    pkg_files(
        name = "imx95_boot_files",
        srcs = [
            ":kernel_aarch64_gki_artifacts",
        ],
        strip_prefix = strip_prefix.files_only(),
        visibility = ["//visibility:private"],
    )

    # GKI system_dlkm
    pkg_files(
        name = "imx95_system_dlkm_files",
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
    # Command: tools/bazel run //kernel_imx:imx95_dist
    pkg_install(
        name = "imx95_dist",
        srcs = [
            ":imx95_kernel_files",
            ":imx95_vendor_boot_files",
            ":imx95_vendor_dlkm_files",
            ":imx95_boot_files",
            ":imx95_system_dlkm_files",
            ":imx95_dtb_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # Vendor boot only (initramfs.img)
    # Command: tools/bazel run //kernel_imx:imx95_vendor_boot_dist
    pkg_install(
        name = "imx95_vendor_boot_dist",
        srcs = [
            ":imx95_kernel_files",
            ":imx95_vendor_boot_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # Vendor DLKM only (vendor_dlkm.img)
    # Command: tools/bazel run //kernel_imx:imx95_vendor_dlkm_dist
    pkg_install(
        name = "imx95_vendor_dlkm_dist",
        srcs = [
            ":imx95_kernel_files",
            ":imx95_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # GKI boot.img only
    # Command: tools/bazel run //kernel_imx:imx95_boot_dist
    pkg_install(
        name = "imx95_boot_dist",
        srcs = [
            ":imx95_boot_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # GKI system_dlkm only
    # Command: tools/bazel run //kernel_imx:imx95_system_dlkm_dist
    pkg_install(
        name = "imx95_system_dlkm_dist",
        srcs = [
            ":imx95_system_dlkm_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # GKI combined (boot.img + system_dlkm)
    # Command: tools/bazel run //kernel_imx:imx95_gki_dist
    pkg_install(
        name = "imx95_gki_dist",
        srcs = [
            ":imx95_boot_files",
            ":imx95_system_dlkm_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # DTB only distribution (for DTBO image generation)
    # Command: tools/bazel run //kernel_imx:imx95_dtb_dist
    pkg_install(
        name = "imx95_dtb_dist",
        srcs = [
            ":imx95_dtb_files",
        ],
        destdir = "out/imx_evk_95_aarch64/dist",
    )

    # ==========================================================================
    # Distribution targets — Mesa GPU mode (USE_GPU_DRIVERS=mesa)
    # ==========================================================================

    # Full distribution (mesa — panthor GPU instead of Mali)
    # Command: tools/bazel run //kernel_imx:imx95_mesa_dist
    pkg_install(
        name = "imx95_mesa_dist",
        srcs = [
            ":imx95_kernel_files",
            ":imx95_vendor_boot_files",
            ":imx95_mesa_vendor_dlkm_files",
            ":imx95_boot_files",
            ":imx95_system_dlkm_files",
            ":imx95_dtb_files",
        ],
        destdir = "out/imx_evk_95_aarch64_mesa/dist",
    )

    # Vendor DLKM only (mesa — panthor GPU modules)
    # Command: tools/bazel run //kernel_imx:imx95_mesa_vendor_dlkm_dist
    pkg_install(
        name = "imx95_mesa_vendor_dlkm_dist",
        srcs = [
            ":imx95_kernel_files",
            ":imx95_mesa_vendor_dlkm_files",
        ],
        destdir = "out/imx_evk_95_aarch64_mesa/dist",
    )

# Export module lists for use in BUILD.bazel or other .bzl files
IMX95_VENDOR_RAMDISK_MODULES = _IMX95_VENDOR_RAMDISK_MODULES
IMX95_VENDOR_DLKM_MODULES = _IMX95_VENDOR_DLKM_MODULES
IMX95_FIRST_STAGE_MODULES = _IMX95_FIRST_STAGE_MODULES
IMX95_RECOVERY_ADDITION_MODULES = _IMX95_RECOVERY_ADDITION_MODULES
IMX95_MALI_GPU_MODULES = _IMX95_MALI_GPU_MODULES
IMX95_IN_TREE_MODULES = _IMX95_IN_TREE_MODULES
IMX95_EXT_MODULES = _IMX95_EXT_MODULES
IMX95_MODULES_LIST = _IMX95_MODULES_LIST
IMX95_MODULES_RECOVERY_LIST = _IMX95_MODULES_RECOVERY_LIST
IMX95_MODULES_CHARGER_LIST = _IMX95_MODULES_CHARGER_LIST
IMX95_DTB_OUTS = _IMX95_DTB_OUTS
