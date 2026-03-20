# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_ARM_PKVM_GUEST) += arm-pkvm-guest.o

pkvm-device-power-$(CONFIG_PM_GENERIC_DOMAINS) += pkvm-device-power.o
obj-$(CONFIG_ARM_PKVM_GUEST) += $(pkvm-device-power-y)
