#
# Makefile for regulator drivers.
#

obj-$(CONFIG_REGULATOR) += core.o
obj-$(CONFIG_REGULATOR_FIXED_VOLTAGE) += fixed.o
obj-$(CONFIG_REGULATOR_VIRTUAL_CONSUMER) += virtual.o

obj-$(CONFIG_REGULATOR_BQ24022) += bq24022.o
obj-$(CONFIG_REGULATOR_WM8350) += wm8350-regulator.o
obj-$(CONFIG_REGULATOR_WM8400) += wm8400-regulator.o
obj-$(CONFIG_REGULATOR_DA903X)	+= da903x.o

obj-$(CONFIG_REGULATOR_MC13892) += reg-mc13892.o
obj-$(CONFIG_REGULATOR_MC13783) += reg-mc13783.o

ccflags-$(CONFIG_REGULATOR_DEBUG) += -DDEBUG
