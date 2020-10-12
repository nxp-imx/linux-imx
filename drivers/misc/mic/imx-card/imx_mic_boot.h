/* SPDX-License-Identifier: GPL-2.0 */
// Copyright 2020 NXP

#ifndef IMX_MIC_BOOT_H
#define IMX_MIC_BOOT_H

#include "../bus/vop_bus.h"

/**
 * struct imx_mic_device -  MIC device information.
 *
 * @mmio: MMIO bar information.
 */
struct imx_mic_device {
	struct mic_mw mmio;
};

/**
 * struct mic_irq - opaque pointer used as cookie.
 */
struct mic_irq;

/**
 * struct imx_mic_intr - MIC interrupt information.
 */
struct imx_mic_intr {
	struct device	*dev;
	void __iomem	*base;
	struct clk	*clk_aside;
	struct clk	*clk_bside;
	struct device	*pd_aside;
	struct device	*pd_bside;
	int		irq;
	u32 doorbell_reg_base;
	u32 doorbell_reg_size;
	irqreturn_t	(*config_intr)(int irq, void *data);
	void		*config_intr_priv;
	irqreturn_t	(*data_intr)(int irq, void *data);
	void		*data_intr_priv;
};

/**
 * struct imx_mic_driver - MIC card driver information.
 *
 * @name: Name for MIC driver.
 * @dev: The device backing this MIC.
 * @mdev: MIC device information for the host.
 * @mintr: MIC interrupt information fot this MIC device.
 * @vpdev: Virtio over PCIe device on the VOP virtual bus.
 * @mepf: endpoint function for MIC.
 */
struct imx_mic_driver {
	char name[20];
	struct device *dev;
	struct imx_mic_device mdev;
	struct imx_mic_intr	*mintr;
	struct vop_device *vpdev;
	struct imx_mic_epf *mepf;
};

/**
 * export symbol from mic boot.
 */
void imx_mic_send_intr(struct imx_mic_driver *mdrv, int db);

/**
 * export symbol from mic intr.
 */
int imx_mic_intr_init(struct imx_mic_driver *mdrv);
void imx_mic_intr_uninit(struct imx_mic_driver *mdrv);
void imx_mic_intr_enable(struct imx_mic_driver *mdrv, int index);
void imx_mic_intr_disable(struct imx_mic_driver *mdrv, int index);
void imx_mic_intr_ack(struct imx_mic_driver *mdrv, int index);

#endif
