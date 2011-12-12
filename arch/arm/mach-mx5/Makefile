#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y   := cpu.o mm.o devices.o ehci.o bus_freq.o sdram_autogating.o  \
pm.o system.o suspend.o usb_dr.o usb_h1.o usb_h2.o cpu_regulator-mx5.o

obj-$(CONFIG_SOC_IMX50) += clock_mx50.o mm-mx50.o mx50_wfi.o mx50_suspend.o mx50_freq.o mx50_ddr_freq.o
obj-$(CONFIG_SOC_IMX51) += clock.o
obj-$(CONFIG_SOC_IMX53) += clock.o

obj-$(CONFIG_CPU_FREQ_IMX)    += cpu_op-mx51.o cpu_op-mx53.o cpu_op-mx50.o
obj-$(CONFIG_MACH_MX51_BABBAGE) += board-mx51_babbage.o mx51_babbage_pmic_mc13892.o
obj-$(CONFIG_MACH_MX51_3DS) += board-mx51_3ds.o
obj-$(CONFIG_MACH_MX53_EVK) += board-mx53_evk.o
obj-$(CONFIG_MACH_MX53_SMD) += board-mx53_smd.o mx53_smd_pmic_da9053.o
obj-$(CONFIG_MACH_IMX_BLUETOOTH_RFKILL) += imx_bt_rfkill.o
obj-$(CONFIG_MACH_MX53_LOCO) += board-mx53_loco.o mx53_loco_pmic_da9053.o mx53_loco_pmic_mc34708.o
obj-$(CONFIG_MACH_MX53_ARD) += board-mx53_ard.o
obj-$(CONFIG_MACH_EUKREA_CPUIMX51) += board-cpuimx51.o
obj-$(CONFIG_MACH_EUKREA_MBIMX51_BASEBOARD) += eukrea_mbimx51-baseboard.o
obj-$(CONFIG_MACH_EUKREA_CPUIMX51SD) += board-cpuimx51sd.o
obj-$(CONFIG_MACH_EUKREA_MBIMXSD51_BASEBOARD) += eukrea_mbimxsd-baseboard.o
obj-$(CONFIG_MX51_EFIKA_COMMON) += mx51_efika.o
obj-$(CONFIG_MACH_MX51_EFIKAMX) += board-mx51_efikamx.o
obj-$(CONFIG_MACH_MX51_EFIKASB) += board-mx51_efikasb.o
obj-$(CONFIG_MACH_MX50_RDP) += board-mx50_rdp.o mx50_rdp_pmic_mc13892.o
