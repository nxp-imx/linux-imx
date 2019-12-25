/******************************************************************************
 *
 * Copyright (C) 2016-2019 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2019 NXP
 *
 ******************************************************************************
 *
 * API_HDCPRX.c
 *
 ******************************************************************************
 */


#include "API_HDCPRX.h"
#include "util.h" // state

#define HDCP_REC_MODULE_ID 0x08

#define OP_SEND_RX_CERT 0x00
#define OP_SEND_PRIVATE_KEY 0x01
#define OP_SET_DEBUG_RANDOM_NUMBERS 0x02
#define OP_SEND_KEYS 0x03
#define OP_SET_CONFIG 0x04
#define OP_GET_STATUS 0x05
#define OP_NOT_SYNC 0x06
#define OP_REC_KSV_LIST 0x07
#define OP_DEBUG_SET_LONG_DELAY 0x08
#define OP_DEBUG_SET_WRONG_ANSWER 0x09
#define OP_SET_KM_KEY 0x0A

#define HDCP_GENERAL_MODULE_ID 0x09

#define OP_SET_LC128 0x00
#define OP_SET_SEED 0x01

const char *CDN_API_HDCPRX_ErrToStr(const CDN_API_HDCPRX_Status *status)
{
	switch (status->error) {
	case CDN_API_HDCPRX_ERR_NONE:
		return "None";
	case CDN_API_HDCPRX_ERR_5V:
		return "5V is down";
	case CDN_API_HDCPRX_ERR_DDC:
		return "DDC error";
	case CDN_API_HDCPRX_ERR_SYNC:
		return "Out of sync";
	default:
		return "Unknown";
	}
}

CDN_API_STATUS CDN_API_HDCPRX_SetRxCert(state_struct *state,
					CDN_API_HDCPRX_RxCert const *cert,
					CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_SEND_RX_CERT, 2,
				      -sizeof(cert->certrx), cert->certrx,
				      -sizeof(cert->rxcaps), cert->rxcaps);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetRxCert_blocking(state_struct *state,
				  CDN_API_HDCPRX_RxCert const *cert,
				  CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetRxCert(state,
							 cert, bus_type));
}

CDN_API_STATUS
CDN_API_HDCPRX_SetPrivateKey(state_struct *state,
			     CDN_API_HDCPRX_PrivateKey const *pkey,
			     CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_SEND_PRIVATE_KEY, 5,
				      -sizeof(pkey->p), pkey->p,
				      -sizeof(pkey->q), pkey->q,
				      -sizeof(pkey->dp), pkey->dp,
				      -sizeof(pkey->dq), pkey->dq,
				      -sizeof(pkey->qp), pkey->qp);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetPrivateKey_blocking(state_struct *state,
				      CDN_API_HDCPRX_PrivateKey const *pkey,
				      CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetPrivateKey(state,
							     pkey, bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_SetDebugRandomNumber(state_struct *state,
						   const u8 rrx[HDCP_RRX_SIZE],
						   CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_SET_DEBUG_RANDOM_NUMBERS, 1,
				      -HDCP_RRX_SIZE, rrx);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetDebugRandomNumber_blocking(state_struct *state,
					     const u8 rrx[HDCP_RRX_SIZE],
					     CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetDebugRandomNumber(state,
								    rrx,
								    bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_SetKeys(state_struct *state,
				      CDN_API_HDCPRX_DeviceKeys const *keys,
				      CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID, OP_SEND_KEYS,
				      3, -sizeof(keys->bksv), keys->bksv,
				      -sizeof(keys->keys), keys->keys,
				      sizeof(keys->bcaps), keys->bcaps);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetKeys_blocking(state_struct *state,
				CDN_API_HDCPRX_DeviceKeys const *keys,
				CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetKeys(state, keys, bus_type));
}

CDN_API_STATUS
CDN_API_HDCPRX_SetConfig(state_struct *state,
			 CDN_API_HDCPRX_Config const *cfg,
			 CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		u8 cfg_val;

		if (!internal_apb_available(state))
			return CDN_BSY;

		cfg_val = cfg->activate;
		cfg_val |= cfg->version << 1;
		cfg_val |= cfg->repeater << 3;
		cfg_val |= cfg->use_secondary_link << 4;
		cfg_val |= cfg->use_km_key << 5;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_SET_CONFIG, 3,
				      sizeof(cfg_val), cfg_val,
				      sizeof(cfg->bcaps), cfg->bcaps,
				      sizeof(cfg->bstatus), cfg->bstatus);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetConfig_blocking(state_struct *state,
				  CDN_API_HDCPRX_Config const *cfg,
				  CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetConfig(state,
							 cfg, bus_type));
}

CDN_API_STATUS
CDN_API_HDCPRX_GetStatus(state_struct *state,
			 CDN_API_HDCPRX_Status *status,
			 CDN_BUS_TYPE bus_type)
{
	CDN_API_STATUS ret;
	u8 flags;

	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_GET_STATUS, 0);

		state->bus_type = bus_type;
		state->rxEnable = 1;

		return CDN_STARTED;
	}

	internal_process_messages(state);

	ret = internal_test_rx_head(state, HDCP_REC_MODULE_ID, OP_GET_STATUS);
	if (ret != CDN_OK)
		return ret;

	internal_readmsg(state, 3,
			 sizeof(flags), &flags,
			 -sizeof(status->aksv), status->aksv,
			 sizeof(status->ainfo), &status->ainfo);
	status->key_arrived = flags & 0x1;
	status->hdcp_ver = flags >> 1 & 0x3;
	status->error = flags >> 4 & 0xF;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_GetStatus_blocking(state_struct *state,
				  CDN_API_HDCPRX_Status *status,
				  CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_GetStatus(state,
							 status, bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_NotSync(state_struct *state,
				      CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_NOT_SYNC, 0);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCPRX_NotSync_blocking(state_struct *state,
					       CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_NotSync(state, bus_type));
}

CDN_API_STATUS
CDN_API_HDCPRX_DebugSetLongDelay(state_struct *state,
				 u8 delays[HDCPRX_NUM_LONG_DELAYS],
				 CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_DEBUG_SET_LONG_DELAY, 1,
				      -HDCPRX_NUM_LONG_DELAYS,
				      delays);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_DebugSetLongDelay_blocking(state_struct *state,
					  u8 delays[HDCPRX_NUM_LONG_DELAYS],
					  CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_DebugSetLongDelay(state, delays,
								 bus_type));
}

CDN_API_STATUS
CDN_API_HDCPRX_DebugSetWrongAnswer(state_struct *state,
				   u8 errors[HDCPRX_DEBUG_NUM_ERRORS],
				   CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_DEBUG_SET_WRONG_ANSWER, 1,
				      -HDCPRX_DEBUG_NUM_ERRORS,
				      errors);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_DebugSetWrongAnswer_blocking(state_struct *state,
					    u8 errors[HDCPRX_DEBUG_NUM_ERRORS],
					    CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_DebugSetWrongAnswer(state,
								   errors,
								   bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_SetLc128(state_struct *state,
				       const u8 lc128[16],
				       CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_GENERAL_MODULE_ID,
				      OP_SET_LC128, 1,
				      -16 /* fixme*/, lc128);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCPRX_SetLc128_blocking(state_struct *state,
						const u8 lc128[16],
						CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetLc128(state,
							lc128, bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_SetSeed(state_struct *state,
				      u8 seed[32],
				      CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_GENERAL_MODULE_ID,
				      OP_SET_SEED, 1,
				      -32 /*sizeof(seed)*/, seed);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetSeed_blocking(state_struct *state,
				u8 seed[32],
				CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetSeed(state,
						       seed, bus_type));
}

CDN_API_STATUS CDN_API_HDCPRX_SetKmKey(state_struct *state,
				       const u8 km_key[16],
				       CDN_BUS_TYPE bus_type)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, HDCP_REC_MODULE_ID,
				      OP_SET_KM_KEY, 1, -16, km_key);

		state->bus_type = bus_type;

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;

	state->running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCPRX_SetKmKey_blocking(state_struct *state,
				 const u8 km_key[16],
				 CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex,
				CDN_API_HDCPRX_SetKmKey(state,
							km_key, bus_type));
}
