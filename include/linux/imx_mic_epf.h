/* SPDX-License-Identifier: GPL-2.0 */
// Copyright 2020 NXP

#ifndef IMX_MIC_EPF_H
#define IMX_MIC_EPF_H

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#define SWIOTLB_REGION_ADDR		0xf8000000

struct imx_mic_mw {
	phys_addr_t		pci_pa;
	void __iomem		*pci_va;
	resource_size_t		pci_len;
	phys_addr_t		rc_shmem_pa;
	phys_addr_t		rc_map_addr;
};

struct imx_mic_epf {
	struct device			*dev;
	struct pci_epf			*epf;
	void				*reg[6];
	const struct pci_epc_features	*epc_features;
	struct delayed_work		bar0_handler;
	struct imx_mic_mw		aper;
};

void imx_mic_get_doorbell_info(u32 *doorbell_reg_base, u32 *doorbell_reg_size);
int imx_mic_probe(struct imx_mic_epf *mic_epf);
void imx_mic_remove(void);
#endif
