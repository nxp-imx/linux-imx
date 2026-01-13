# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_IMX_DSP)		+= imx-dsp.o
obj-$(CONFIG_IMX_SCU)		+= imx-scu.o misc.o imx-scu-irq.o rm.o imx-scu-soc.o seco.o
obj-${CONFIG_IMX_SCMI_CPU_DRV}	+= sm-cpu.o
obj-${CONFIG_IMX_SCMI_MISC_DRV}	+= sm-misc.o
obj-${CONFIG_IMX_SCMI_LMM_DRV}	+= sm-lmm.o
sec_enclave-objs		= se_ctrl.o ele_common.o ele_base_msg.o se_msg_sqfl_ctrl.o ele_fw_api.o ele_bbsm.o v2x_base_msg.o v2x_common.o seco_init.o
ifdef CONFIG_IMX_ELE_TRNG
sec_enclave-objs		+= ele_trng.o
endif
obj-${CONFIG_IMX_SEC_ENCLAVE}	+= sec_enclave.o
obj-$(CONFIG_IMX_DISPMU)	+= disp-mu.o
