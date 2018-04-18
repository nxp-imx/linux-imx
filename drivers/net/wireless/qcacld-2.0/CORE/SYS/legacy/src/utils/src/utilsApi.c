/*
 * Copyright (c) 2013, 2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

//==================================================================
//
//  File:         utilsApi.cc
//
//  Description:  Implemention of a few utility routines.
//
//  Author:       Neelay Das
//
////
//  Change gHistory:
//  12/15/2003 - NDA - Initial version.
//
//===================================================================


#include "utilsApi.h"

void
sirDumpBuf(tpAniSirGlobal pMac, tANI_U8 modId, tANI_U32 level, tANI_U8 *buf, tANI_U32 size)
{
    tANI_U32 i;

    if (level > pMac->utils.gLogDbgLevel[LOG_INDEX_FOR_MODULE(modId)])
        return;

    logDbg(pMac, modId, level, FL("Dumping %d bytes in host order\n"), size);

    for (i=0; (i+7)<size; i+=8)
    {
        logDbg(pMac, modId, level,
                 "%02x %02x %02x %02x %02x %02x %02x %02x \n",
                 buf[i],
                 buf[i+1],
                 buf[i+2],
                 buf[i+3],
                 buf[i+4],
                 buf[i+5],
                 buf[i+6],
                 buf[i+7]);
    }

    // Dump the bytes in the last line
    for (; i < size; i++)
    {
        logDbg(pMac, modId, level, "%02x ", buf[i]);

        if((i+1) == size)
            logDbg(pMac, modId, level, "\n");
    }

}/*** end sirDumpBuf() ***/

#ifdef WLAN_FEATURE_DSRC
struct sir_ocb_config *
sir_copy_sir_ocb_config(const struct sir_ocb_config *src)
{
	struct sir_ocb_config *dst;
	uint32_t length;
	void *cursor;

	length = sizeof(*src) +
		src->channel_count * sizeof(*src->channels) +
		src->schedule_size * sizeof(*src->schedule) +
		src->dcc_ndl_chan_list_len +
		src->dcc_ndl_active_state_list_len +
		src->def_tx_param_size;

	dst = vos_mem_malloc(length);
	if (!dst)
		return NULL;

	*dst = *src;

	cursor = dst;
	cursor += sizeof(*dst);
	dst->channels = cursor;
	cursor += src->channel_count * sizeof(*src->channels);
	vos_mem_copy(dst->channels, src->channels,
		     src->channel_count * sizeof(*src->channels));
	dst->schedule = cursor;
	cursor += src->schedule_size * sizeof(*src->schedule);
	vos_mem_copy(dst->schedule, src->schedule,
		     src->schedule_size * sizeof(*src->schedule));
	dst->dcc_ndl_chan_list = cursor;
	cursor += src->dcc_ndl_chan_list_len;
	vos_mem_copy(dst->dcc_ndl_chan_list, src->dcc_ndl_chan_list,
		     src->dcc_ndl_chan_list_len);
	dst->dcc_ndl_active_state_list = cursor;
	cursor += src->dcc_ndl_active_state_list_len;
	vos_mem_copy(dst->dcc_ndl_active_state_list,
		     src->dcc_ndl_active_state_list,
		     src->dcc_ndl_active_state_list_len);

	if (src->def_tx_param && src->def_tx_param_size) {
		dst->def_tx_param = cursor;
		vos_mem_copy(dst->def_tx_param, src->def_tx_param,
			     src->def_tx_param_size);
	}

	return dst;
}
#endif /* WLAN_FEATURE_DSRC */
