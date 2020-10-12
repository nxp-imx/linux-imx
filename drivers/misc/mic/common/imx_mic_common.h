/* SPDX-License-Identifier: GPL-2.0 */
// Copyright 2020 NXP

#ifndef _IMX_MIC_COMMON_H_
#define _IMX_MIC_COMMON_H_

#define MIC_CONFIG_DB			0
#define MIC_DATA_DB			1

/* PCI MSI index to interrupt RC side */
#define MIC_PCI_MSI_INDEX		1

/* the offset of MU registers definition */
#define MIC_INTR_MU_SR			0x20
#define MIC_INTR_MU_CR			0x24

/* General Purpose Interrupt Enable */
#define MIC_INTR_MU_ACR_GIEn(x)		BIT(28 + (3 - (x)))
/* General Interrupt Request Pending */
#define MIC_INTR_MU_ASR_GIPn(x)		BIT(28 + (3 - (x)))
/* General Purpose Interrupt Request */
#define MIC_INTR_MU_BCR_GIRn(x)		BIT(16 + (3 - (x)))

#endif
