#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y   := system.o iomux.o cpu.o mm.o devices.o serial.o dma.o lpmodes.o pm.o \
sdram_autogating.o bus_freq.o usb_dr.o usb_h1.o usb_h2.o dummy_gpio.o  early_setup.o

obj-$(CONFIG_ARCH_MX51) += clock.o suspend.o
obj-$(CONFIG_ARCH_MX53) += clock.o suspend.o
obj-$(CONFIG_ARCH_MX50) += clock_mx50.o dmaengine.o dma-apbh.o mx50_suspend.o mx50_ddr_freq.o mx50_wfi.o

obj-$(CONFIG_MACH_MX51_3DS) += mx51_3stack.o mx51_3stack_gpio.o mx51_3stack_pmic_mc13892.o
obj-$(CONFIG_MACH_MX51_BABBAGE) += mx51_babbage.o mx51_babbage_pmic_mc13892.o
obj-$(CONFIG_MACH_MX53_EVK) += mx53_evk.o mx53_evk_pmic_mc13892.o
obj-$(CONFIG_MACH_MX53_ARD) += mx53_ard.o mx53_ard_pmic_ltc3589.o
obj-$(CONFIG_MACH_MX50_ARM2) += mx50_arm2.o mx50_arm2_pmic_mc13892.o
obj-$(CONFIG_MACH_MX50_RDP) += mx50_rdp.o mx50_rdp_pmic_mc13892.o

