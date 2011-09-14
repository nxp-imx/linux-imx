//------------------------------------------------------------------------------
// <copyright file="roaming.h" company="Atheros">
//    Copyright (c) 2004-2007 Atheros Corporation.  All rights reserved.
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

#ifndef _ROAMING_H_
#define _ROAMING_H_

/* 
 * The signal quality could be in terms of either snr or rssi. We should 
 * have an enum for both of them. For the time being, we are going to move 
 * it to wmi.h that is shared by both host and the target, since we are 
 * repartitioning the code to the host 
 */
#define SIGNAL_QUALITY_NOISE_FLOOR        -96
#define SIGNAL_QUALITY_METRICS_NUM_MAX    2
typedef enum {
    SIGNAL_QUALITY_METRICS_SNR = 0,
    SIGNAL_QUALITY_METRICS_RSSI,
    SIGNAL_QUALITY_METRICS_ALL,
} SIGNAL_QUALITY_METRICS_TYPE;

#endif  /* _ROAMING_H_ */
