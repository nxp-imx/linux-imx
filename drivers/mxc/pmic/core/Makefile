#
# Makefile for the PMIC core drivers.
#
#obj-$(CONFIG_MXC_PMIC_MC13783) += pmic_mc13783_mod.o
#pmic_mc13783_mod-objs := pmic_external.o pmic_event.o pmic_common.o pmic_core_spi.o mc13783.o

obj-$(findstring y, $(CONFIG_MXC_PMIC_MC13892) $(CONFIG_MXC_PMIC_MC34708))	+= pmic_mxc_mod.o
pmic_mxc_mod-objs	:= pmic_external.o pmic_event.o pmic_common.o pmic_adc_interface.o

ifneq ($(CONFIG_MXC_PMIC_MC13892),)
pmic_mxc_mod-objs	+= mc13892.o
endif

ifneq ($(CONFIG_MXC_PMIC_MC34708),)
pmic_mxc_mod-objs	+= mc34708.o
endif

ifneq ($(CONFIG_MXC_PMIC_SPI),)
pmic_mxc_mod-objs += pmic_core_spi.o
endif

ifneq ($(CONFIG_MXC_PMIC_I2C),)
pmic_mc13892_mod-objs += pmic_core_i2c.o
endif

obj-$(CONFIG_MXC_PMIC_MC34704) += pmic_mc34704_mod.o
pmic_mc34704_mod-objs := pmic_external.o pmic_event.o mc34704.o

obj-$(CONFIG_MXC_PMIC_CHARDEV)	+= pmic-dev.o
