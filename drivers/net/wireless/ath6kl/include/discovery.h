//------------------------------------------------------------------------------
// <copyright file="discovery.h" company="Atheros">
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

#ifndef _DISCOVERY_H_
#define _DISCOVERY_H_

/*
 * DC_SCAN_PRIORITY is an 8-bit bitmap of the scan priority of a channel
 */
typedef enum {
    DEFAULT_SCPRI = 0x01,
    POPULAR_SCPRI = 0x02,
    SSIDS_SCPRI   = 0x04,
    PROF_SCPRI    = 0x08,
} DC_SCAN_PRIORITY;

/* The following search type construct can be used to manipulate the behavior of the search module based on different bits set */
typedef enum {
    SCAN_RESET                     = 0,
    SCAN_ALL                       = (DEFAULT_SCPRI | POPULAR_SCPRI |  \
                                      SSIDS_SCPRI | PROF_SCPRI),

    SCAN_POPULAR                   = (POPULAR_SCPRI | SSIDS_SCPRI | PROF_SCPRI),
    SCAN_SSIDS                     = (SSIDS_SCPRI | PROF_SCPRI),
    SCAN_PROF_MASK                 = (PROF_SCPRI),
    SCAN_MULTI_CHANNEL             = 0x000100,
    SCAN_DETERMINISTIC             = 0x000200,
    SCAN_PROFILE_MATCH_TERMINATED  = 0x000400,
    SCAN_HOME_CHANNEL_SKIP         = 0x000800,
    SCAN_CHANNEL_LIST_CONTINUE     = 0x001000,
    SCAN_CURRENT_SSID_SKIP         = 0x002000,
    SCAN_ACTIVE_PROBE_DISABLE      = 0x004000,
    SCAN_CHANNEL_HINT_ONLY         = 0x008000,
    SCAN_ACTIVE_CHANNELS_ONLY      = 0x010000,
    SCAN_UNUSED1                   = 0x020000, /* unused */
    SCAN_PERIODIC                  = 0x040000,
    SCAN_FIXED_DURATION            = 0x080000,
    SCAN_AP_ASSISTED               = 0x100000,
} DC_SCAN_TYPE;

typedef enum {
    BSS_REPORTING_DEFAULT = 0x0,
    EXCLUDE_NON_SCAN_RESULTS = 0x1, /* Exclude results outside of scan */
} DC_BSS_REPORTING_POLICY;

typedef enum {
    DC_IGNORE_WPAx_GROUP_CIPHER = 0x01,
    DC_PROFILE_MATCH_DONE = 0x02,
    DC_IGNORE_AAC_BEACON = 0x04, 
    DC_CSA_FOLLOW_BSS = 0x08,
} DC_PROFILE_FILTER;

#define DEFAULT_DC_PROFILE_FILTER   (DC_CSA_FOLLOW_BSS)

#endif  /* _DISCOVERY_H_ */
