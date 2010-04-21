//------------------------------------------------------------------------------
// <copyright file="hci_bridge.c" company="Atheros">
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
// HCI bridge implementation
//
// Author(s): ="Atheros"
//==============================================================================

#include "hci_transport_api.h"
#include "common_drv.h"

extern HCI_TRANSPORT_HANDLE (*_HCI_TransportAttach)(void *HTCHandle, HCI_TRANSPORT_CONFIG_INFO *pInfo);
extern void (*_HCI_TransportDetach)(HCI_TRANSPORT_HANDLE HciTrans);
extern A_STATUS    (*_HCI_TransportAddReceivePkts)(HCI_TRANSPORT_HANDLE HciTrans, HTC_PACKET_QUEUE *pQueue);
extern A_STATUS    (*_HCI_TransportSendPkt)(HCI_TRANSPORT_HANDLE HciTrans, HTC_PACKET *pPacket, A_BOOL Synchronous);
extern void        (*_HCI_TransportStop)(HCI_TRANSPORT_HANDLE HciTrans);
extern A_STATUS    (*_HCI_TransportStart)(HCI_TRANSPORT_HANDLE HciTrans);
extern A_STATUS    (*_HCI_TransportEnableDisableAsyncRecv)(HCI_TRANSPORT_HANDLE HciTrans, A_BOOL Enable);
extern A_STATUS    (*_HCI_TransportRecvHCIEventSync)(HCI_TRANSPORT_HANDLE HciTrans, 
                                          HTC_PACKET           *pPacket,
                                          int                  MaxPollMS);
extern A_STATUS    (*_HCI_TransportSetBaudRate)(HCI_TRANSPORT_HANDLE HciTrans, A_UINT32 Baud);


#define HCI_TransportAttach(HTCHandle, pInfo)   \
            _HCI_TransportAttach((HTCHandle), (pInfo))
#define HCI_TransportDetach(HciTrans)    \
            _HCI_TransportDetach(HciTrans)
#define HCI_TransportAddReceivePkts(HciTrans, pQueue)   \
            _HCI_TransportAddReceivePkts((HciTrans), (pQueue))
#define HCI_TransportSendPkt(HciTrans, pPacket, Synchronous)  \
            _HCI_TransportSendPkt((HciTrans), (pPacket), (Synchronous))
#define HCI_TransportStop(HciTrans)  \
            _HCI_TransportStop((HciTrans))
#define HCI_TransportStart(HciTrans)  \
            _HCI_TransportStart((HciTrans))
#define HCI_TransportEnableDisableAsyncRecv(HciTrans, Enable)   \
            _HCI_TransportEnableDisableAsyncRecv((HciTrans), (Enable))
#define HCI_TransportRecvHCIEventSync(HciTrans, pPacket, MaxPollMS)   \
            _HCI_TransportRecvHCIEventSync((HciTrans), (pPacket), (MaxPollMS))
#define HCI_TransportSetBaudRate(HciTrans, Baud)    \
            _HCI_TransportSetBaudRate((HciTrans), (Baud))


extern A_STATUS ar6000_register_hci_transport(HCI_TRANSPORT_CALLBACKS *hciTransCallbacks);

extern A_STATUS ar6000_get_hif_dev(HIF_DEVICE *device, void *config);

extern A_STATUS ar6000_set_uart_config(HIF_DEVICE *hifDevice, A_UINT32 scale, A_UINT32 step);

/* get core clock register settings
 * data: 0 - 40/44MHz
 *       1 - 80/88MHz
 *       where (5G band/2.4G band)
 * assume 2.4G band for now
 */
extern A_STATUS ar6000_get_core_clock_config(HIF_DEVICE *hifDevice, A_UINT32 *data);
