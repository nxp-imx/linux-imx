//------------------------------------------------------------------------------
//    Copyright (c) 2005 Atheros Corporation.  All rights reserved.
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

#ifndef __PKT_LOG_H__
#define __PKT_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif


/* Pkt log info */
typedef PREPACK struct pkt_log_t {
    struct info_t {
        A_UINT16    st;
        A_UINT16    end;
        A_UINT16    cur;
    }info[4096];
    A_UINT16    last_idx;
}POSTPACK PACKET_LOG;


#ifdef __cplusplus
}
#endif
#endif  /* __PKT_LOG_H__ */
