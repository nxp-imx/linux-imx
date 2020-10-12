/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IMX MIC Platform Software Stack
 *
 * Copyright(c) 2020 NXP.
 *
 * Intel MIC Host driver.
 */
#ifndef _IMX_MIC_MAIN_H_
#define _IMX_MIC_MAIN_H_

#include <linux/cma.h>
#include <linux/dma-contiguous.h>
#include <linux/idr.h>
#include <linux/irqreturn.h>
#include <linux/mic_common.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "../bus/vop_bus.h"
#include "../bus/cosm_bus.h"
#include "../common/imx_mic_common.h"

#define PCI_DEVICE_ID_IMX_MIC		0xCBED

#define IMX_MIC_TEMP_BAR           0
#define IMX_MIC_MMIO_BAR           2
#define IMX_MIC_APER_BAR           4

#define RC_SHMEM_ADDR_OFFSET	0x0
#define RC_SHMEM_SIZE_OFFSET	0x8
#define RC_MAP_ADDR_OFFSET	0x10

/**
 * struct imx_mic_device -  MIC device information for each card.
 *
 * @mmio: MMIO bar information.
 * @aper: Aperture bar information.
 * @id: The unique device id for this MIC device.
 * @dp: virtio device page.
 * @dp_dma_addr: virtio device page DMA address.
 * @num_vectors: The number of MSI vectors that have been allocated.
 * @temp_bar_base: Virtual address of temp bar.
 * @pdev: Underlying PCI device.
 * @vpdev: Virtio over PCIe device on the VOP virtual bus.
 * @cosm_dev: COSM device
 */
struct imx_mic_device {
	struct mic_mw mmio;
	struct mic_mw aper;
	int id;
	void *dp;
	dma_addr_t dp_dma_addr;
	u32 num_vectors;
	void __iomem *temp_bar_base;
	struct pci_dev *pdev;
	struct vop_device *vpdev;
	struct cosm_device *cosm_dev;
};

extern struct cosm_hw_ops cosm_hw_ops;
#endif
