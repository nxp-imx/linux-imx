#
# Makefile for the linux kernel.
#

# Object file lists.


obj-y   := system.o iomux.o cpu.o mm.o clock.o devices.o serial.o dma.o lpmodes.o pm.o \
sdram_autogating.o bus_freq.o usb_dr.o usb_h1.o usb_h2.o dummy_gpio.o wfi.o suspend.o

obj-$(CONFIG_MACH_MX51_3DS) += mx51_3stack.o mx51_3stack_gpio.o mx51_3stack_pmic_mc13892.o
obj-$(CONFIG_MACH_MX51_BABBAGE) += mx51_babbage.o mx51_babbage_gpio.o mx51_babbage_pmic_mc13892.o early_setup.o
obj-$(CONFIG_MACH_MX53_EVK) += mx53_evk.o mx53_evk_gpio.o mx53_evk_pmic_mc13892.o early_setup.o

