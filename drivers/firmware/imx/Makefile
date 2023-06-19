# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_IMX_DSP)		+= imx-dsp.o
obj-$(CONFIG_IMX_SCU)		+= imx-scu-firmware.o
imx-scu-firmware-objs 		+= imx-scu.o misc.o imx-scu-irq.o rm.o imx-scu-soc.o seco.o
obj-$(CONFIG_IMX_SCU_PD)	+= scu-pd.o
obj-${CONFIG_IMX_SECO_MU}	+= seco_mu.o
el_enclave-objs			= ele_mu.o ele_base_msg.o ele_fw_api.o
obj-${CONFIG_IMX_EL_ENCLAVE}	+= el_enclave.o
el_enclave-${CONFIG_IMX_ELE_TRNG} += ele_trng.o
