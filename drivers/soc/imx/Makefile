# SPDX-License-Identifier: GPL-2.0-only
ifeq ($(CONFIG_ARM),y)
obj-$(CONFIG_ARCH_MXC) += soc-imx.o
endif
obj-$(CONFIG_HAVE_IMX_GPC) += gpc.o
obj-$(CONFIG_IMX_GPCV2_PM_DOMAINS) += gpcv2.o gpcv2-imx.o
obj-$(CONFIG_SOC_IMX8M) += soc-imx8m.o
obj-$(CONFIG_IMX8M_BUSFREQ) += busfreq-imx8mq.o
obj-$(CONFIG_SOC_IMX_MU) += mu/
obj-${CONFIG_SECVIO_SC} += secvio/
obj-$(CONFIG_IMX8M_PM_DOMAINS) += imx8m_pm_domains.o
obj-$(CONFIG_RPMSG_LIFE_CYCLE) += rpmsg_life_cycle.o
