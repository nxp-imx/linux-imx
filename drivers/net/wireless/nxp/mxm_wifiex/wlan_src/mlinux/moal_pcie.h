/** @file moal_pcie.h
 *
 *  @brief This file contains definitions for PCIE interface.
 *  driver.
 *
 *
 * Copyright 2014-2020 NXP
 *
 * This software file (the File) is distributed by NXP
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the License).  You may use, redistribute and/or modify the File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    02/01/2012: initial version
********************************************************/

#ifndef _MOAL_PCIE_H_
#define _MOAL_PCIE_H_

#define PCIE_VENDOR_ID_NXP (0x11ab)
#define PCIE_VENDOR_ID_V2_NXP (0x1b4b)
#ifdef PCIE8997
/** PCIE device ID for 8997 card */
#define PCIE_DEVICE_ID_NXP_88W8997P (0x2b42)
#endif
#ifdef PCIE8897
/** PCIE device ID for 8897 card */
#define PCIE_DEVICE_ID_NXP_88W8897P (0x2b38)
#endif

#ifdef PCIE9097
/** PCIE device ID for 9097 card */
#define PCIE_DEVICE_ID_NXP_88W9097 (0x2b56)
#endif

#ifdef PCIE9098
/** PCIE device ID for 9098 card FN0 */
#define PCIE_DEVICE_ID_NXP_88W9098P_FN0 (0x2b43)
/** PCIE device ID for 9098 card FN1 */
#define PCIE_DEVICE_ID_NXP_88W9098P_FN1 (0x2b44)
#endif

#include <linux/version.h>
#include <linux/pci.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
#include <linux/pcieport_if.h>
#endif
#include <linux/interrupt.h>

#include "moal_main.h"

/** Default firmware name */
#ifdef PCIE8997
#define PCIE8997_DEFAULT_COMBO_FW_NAME "nxp/pcieusb8997_combo_v4.bin"
#define PCIEUART8997_DEFAULT_COMBO_FW_NAME "nxp/pcieuart8997_combo_v4.bin"
#define PCIEUSB8997_DEFAULT_COMBO_FW_NAME "nxp/pcieusb8997_combo_v4.bin"
#define PCIE8997_DEFAULT_WLAN_FW_NAME "nxp/pcie8997_wlan_v4.bin"
/** PCIE8997 chip revision ID */
#define PCIE8997_A0 0x10
#define PCIE8997_A1 0x11
#endif /* PCIE8997 */

#ifdef PCIE8897
#define PCIE8897_DEFAULT_COMBO_FW_NAME "nxp/pcie8897_uapsta.bin"
#define PCIE8897_DEFAULT_WLAN_FW_NAME "nxp/pcie8897_wlan.bin"
#endif /* PCIE8897*/

#ifdef PCIE9098
#define PCIE9098_Z1Z2 0x00
#define PCIE9098_A0 0x01
#define PCIE9098_A1 0x02
#define PCIE9098_A2 0x03
#define PCIE9098_DEFAULT_COMBO_FW_NAME "nxp/pcieusb9098_combo.bin"
#define PCIEUART9098_DEFAULT_COMBO_FW_NAME "nxp/pcieuart9098_combo.bin"
#define PCIEUSB9098_DEFAULT_COMBO_FW_NAME "nxp/pcieusb9098_combo.bin"
#define PCIEPCIE9098_DEFAULT_COMBO_FW_NAME "nxp/pciepcie9098_combo.bin"
#define PCIEUART9098_COMBO_V1_FW_NAME "nxp/pcieuart9098_combo_v1.bin"
#define PCIEUSB9098_COMBO_V1_FW_NAME "nxp/pcieusb9098_combo_v1.bin"
#define PCIEPCIE9098_COMBO_V1_FW_NAME "nxp/pciepcie9098_combo_v1.bin"
#define PCIE9098_DEFAULT_WLAN_FW_NAME "nxp/pcie9098_wlan.bin"
#define PCIE9098_WLAN_V1_FW_NAME "nxp/pcie9098_wlan_v1.bin"
#endif /* PCIE9098 */

#ifdef PCIE9097
#define PCIE9097_A0 0x00
#define PCIE9097_B0 0x01
#define PCIE9097_B1 0x02
#define PCIE9097_DEFAULT_COMBO_FW_NAME "nxp/pcieusb9097_combo.bin"
#define PCIEUART9097_DEFAULT_COMBO_FW_NAME "nxp/pcieuart9097_combo.bin"
#define PCIEUSB9097_DEFAULT_COMBO_FW_NAME "nxp/pcieusb9097_combo.bin"
#define PCIEUART9097_COMBO_V1_FW_NAME "nxp/pcieuart9097_combo_v1.bin"
#define PCIEUSB9097_COMBO_V1_FW_NAME "nxp/pcieusb9097_combo_v1.bin"
#define PCIE9097_DEFAULT_WLAN_FW_NAME "nxp/pcie9097_wlan.bin"
#define PCIE9097_WLAN_V1_FW_NAME "nxp/pcie9097_wlan_v1.bin"
#endif /* PCIE9097 */

#if defined(PCIE9098) || defined(PCIE9097)
#define PCIE_NUM_MSIX_VECTORS 32
#else
#define PCIE_NUM_MSIX_VECTORS 4
#endif

typedef struct _msix_context {
	/** pci_dev structure pointer */
	struct pci_dev *dev;
	/** message id related to msix vector */
	t_u16 msg_id;
} msix_context;

/** Structure: PCIE service card */
typedef struct _pcie_service_card {
	/** pci_dev structure pointer */
	struct pci_dev *dev;
	/** moal_handle structure pointer */
	moal_handle *handle;
	/** I/O memory regions pointer to the bus */
	void __iomem *pci_mmap;
	/** I/O memory regions pointer to the bus */
	void __iomem *pci_mmap1;
#if defined(PCIE)
	struct msix_entry msix_entries[PCIE_NUM_MSIX_VECTORS];
	msix_context msix_contexts[PCIE_NUM_MSIX_VECTORS];
#endif
} pcie_service_card, *ppcie_service_card;

/** Register to bus driver function */
mlan_status woal_pcie_bus_register(void);
/** Unregister from bus driver function */
void woal_pcie_bus_unregister(void);

/* pmqos busfreq request handler*/
void woal_request_pmqos_busfreq_high(void);
/* pmqos busfreq release handler*/
void woal_release_pmqos_busfreq_high(void);

#endif /* _MOAL_PCIE_H_ */
