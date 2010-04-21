/*
 *
 * Copyright (c) 2004-2010 Atheros Communications Inc.
 * All rights reserved.
 *
 * 
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
 *
 */

#ifndef _AR6K_CFG80211_H_
#define _AR6K_CFG80211_H_

struct wireless_dev *ar6k_cfg80211_init(struct device *dev);
void ar6k_cfg80211_deinit(AR_SOFTC_T *ar);

void ar6k_cfg80211_scanComplete_event(AR_SOFTC_T *ar, A_STATUS status);

void ar6k_cfg80211_connect_event(AR_SOFTC_T *ar, A_UINT16 channel,
                                A_UINT8 *bssid, A_UINT16 listenInterval,
                                A_UINT16 beaconInterval,NETWORK_TYPE networkType,
                                A_UINT8 beaconIeLen, A_UINT8 assocReqLen,
                                A_UINT8 assocRespLen, A_UINT8 *assocInfo);

void ar6k_cfg80211_disconnect_event(AR_SOFTC_T *ar, A_UINT8 reason,
                                    A_UINT8 *bssid, A_UINT8 assocRespLen,
                                    A_UINT8 *assocInfo, A_UINT16 protocolReasonStatus);

void ar6k_cfg80211_tkip_micerr_event(AR_SOFTC_T *ar, A_UINT8 keyid, A_BOOL ismcast);

#endif /* _AR6K_CFG80211_H_ */






