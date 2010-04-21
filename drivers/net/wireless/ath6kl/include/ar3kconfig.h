//------------------------------------------------------------------------------
// <copyright file="ar3Kconfig.h" company="Atheros">
//    Copyright (c) 2009 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

/* AR3K module configuration APIs for HCI-bridge operation */

#ifndef AR3KCONFIG_H_
#define AR3KCONFIG_H_

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AR3K_CONFIG_FLAG_FORCE_MINBOOT_EXIT         (1 << 0)
#define AR3K_CONFIG_FLAG_SET_AR3K_BAUD              (1 << 1)
#define AR3K_CONFIG_FLAG_AR3K_BAUD_CHANGE_DELAY     (1 << 2)
#define AR3K_CONFIG_FLAG_SET_AR6K_SCALE_STEP        (1 << 3)


typedef struct {
    A_UINT32                 Flags;           /* config flags */
    void                     *pHCIDev;        /* HCI bridge device     */
    HCI_TRANSPORT_PROPERTIES *pHCIProps;      /* HCI bridge props      */
    HIF_DEVICE               *pHIFDevice;     /* HIF layer device      */
    
    A_UINT32                 AR3KBaudRate;    /* AR3K operational baud rate */
    A_UINT16                 AR6KScale;       /* AR6K UART scale value */    
    A_UINT16                 AR6KStep;        /* AR6K UART step value  */
    struct hci_dev           *pBtStackHCIDev; /* BT Stack HCI dev */
} AR3K_CONFIG_INFO;
                                                                                        
A_STATUS AR3KConfigure(AR3K_CONFIG_INFO *pConfigInfo);

A_STATUS AR3KConfigureExit(void *config);

#ifdef __cplusplus
}
#endif

#endif /*AR3KCONFIG_H_*/
