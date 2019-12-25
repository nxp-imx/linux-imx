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
 * API_HDCP_REC.h
 *
 ******************************************************************************
 */

#ifndef API_HDCPRX_H_
#define API_HDCPRX_H_

/**
 * \addtogroup HDCPRX_API
 * \{
 */

#include "API_General.h"

/* From HDCP standard */
#define HDCP_RRX_SIZE 8
#define HDCP_KSV_SIZE 5
#define HDCP_NUM_DEVICE_KEY_SIZE 7
#define HDCP_NUM_OF_DEVICE_KEYS 40

#define HDCP_REC_CERTRX_SIZE 522
#define HDCP_REC_RXCAPS_SIZE 3

#define CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE 64

#define HDCPRX_NUM_LONG_DELAYS 5
#define HDCPRX_DEBUG_NUM_ERRORS 5

#define CDN_API_HDCPRX_ERR_NONE 0
#define CDN_API_HDCPRX_ERR_5V 1
#define CDN_API_HDCPRX_ERR_DDC 2
#define CDN_API_HDCPRX_ERR_SYNC 3

#define CDN_API_HDCPRX_VERSION_2 0x0
#define CDN_API_HDCPRX_VERSION_1 0x1
#define CDN_API_HDCPRX_VERSION_BOTH 0x2

/**
 * RX Certificate.
 *
 * @field certrx [in] Cert RX value.
 * @field rxcaps [in] RX Caps value.
 */
typedef struct {
	u8 certrx[HDCP_REC_CERTRX_SIZE];
	u8 rxcaps[HDCP_REC_RXCAPS_SIZE];
} CDN_API_HDCPRX_RxCert;

/**
 * Private keys for receiver.
 *
 * @field q P
 * @field q Q
 * @field dp d mod (p-1)
 * @field dq d mod (q-1)
 * @field qp q^-1 mod p
 */
typedef struct {
	u8 p[CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE];
	u8 q[CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE];
	u8 dp[CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE];
	u8 dq[CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE];
	u8 qp[CDN_API_HDCPRX_PRIVATE_KEY_PARAM_SIZE];
} CDN_API_HDCPRX_PrivateKey;

/**
 * Device Keys for HDCP Sink.
 *
 * @filed bksv [in] Bksv value.
 * @field keys [in] Device Keys.
 * @field bcaps [in] Bcaps value.
 */
typedef struct {
	u8 bksv[HDCP_KSV_SIZE];
	u8 keys[HDCP_NUM_OF_DEVICE_KEYS][HDCP_NUM_DEVICE_KEY_SIZE];
	u8 bcaps;
} CDN_API_HDCPRX_DeviceKeys;

/**
 * HDCP config.
 *
 * @field activate [in] Activate HDCP module.
 * @field version [in] Supported HDCP versions.
 *      0 - HDCP 2.2 only.
 *      1 - HDCP 1.3/1.4 only.
 *      2 - HDCP 1.3/1.4 and 2.2.
 *      3 - reserved
 * @field repeater [in] Set for HDCP Repeater.
 * @field use_secondary_link [in] Set to use the Secondary Link.
 * @field use_km_key [in] Set to enable km-key encryption.
 * @field bcaps [in] Bcaps value. HDCP 1.3/1.4 only.
 * @field bstatus [in] Bstatus value. HDCP 1.3/1.4 only.
 */
typedef struct {
	u8 activate : 1;
	u8 version : 2;
	u8 repeater : 1;
	u8 use_secondary_link : 1;
	u8 use_km_key : 1;
	u8 bcaps; // for HDCP 1.4
	u16 bstatus; // for HDCP 1.4
} CDN_API_HDCPRX_Config;

/**
 * HDCP Receiver Status.
 *
 * @field key_arrived [out] TODO: what key?
 * @field hdcp_ver [out] HDCP version.
 *      0 - no HDCP
 *      1 - HDCP 1.3/1.4
 *      2 - HDCP 2.2
 *      3 - not valid / reserved
 * @field error [out] Last error code.
 * @field aksv [out] Aksv value sent by HDCP Source (HDCP 1.3/1.4 only).
 * @field ainfo [out] Ainfo value sent by HDCP Source (HDCP 1.3/1.4 only).
 */
typedef struct {
	u8 key_arrived : 1;
	u8 hdcp_ver : 2;
	u8 error : 4;
	u8 aksv[HDCP_KSV_SIZE];
	u8 ainfo;
} CDN_API_HDCPRX_Status;


/**
 * Converts error code to error string.
 */
char const *CDN_API_HDCPRX_ErrToStr(CDN_API_HDCPRX_Status const *status);


/**
 * \brief send HDCP2_SEND_RX_CERT command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCPRX_SetRxCert(state_struct *state,
					CDN_API_HDCPRX_RxCert const *cert,
					CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_SetRxCert
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetRxCert_blocking(state_struct *state,
				  CDN_API_HDCPRX_RxCert const *cert,
				  CDN_BUS_TYPE bus_type);
/**
 * \brief send HDCP2_REC_SEND_PRIVATE_KEY command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetPrivateKey(state_struct *state,
			     CDN_API_HDCPRX_PrivateKey const *pkey,
			     CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_SetPrivateKey
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetPrivateKey_blocking(state_struct *state,
				      CDN_API_HDCPRX_PrivateKey const *pkey,
				      CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_REC_SET_DEBUG_RANDOM_NUMBERS command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetDebugRandomNumber(state_struct *state,
				    const u8 rrx[HDCP_RRX_SIZE],
				    CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_SetDebugRandomNumber
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetDebugRandomNumber_blocking(state_struct *state,
					     const u8 rrx[HDCP_RRX_SIZE],
					     CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP1_REC_SEND_KEYS command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCPRX_SetKeys(state_struct *state,
				      CDN_API_HDCPRX_DeviceKeys const *keys,
				      CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_SetKeys
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetKeys_blocking(state_struct *state,
				CDN_API_HDCPRX_DeviceKeys const *keys,
				CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_REC_CONFIG command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetConfig(state_struct *state,
			 CDN_API_HDCPRX_Config const *status,
			 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_SetConfig
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetConfig_blocking(state_struct *state,
				  CDN_API_HDCPRX_Config const *status,
				  CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_REC_STATUS command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_GetStatus(state_struct *state,
			 CDN_API_HDCPRX_Status *cfg,
			 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_GetStatus
 */
CDN_API_STATUS
CDN_API_HDCPRX_GetStatus(state_struct *state,
			 CDN_API_HDCPRX_Status *cfg,
			 CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_REC_NOT_SYNC command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCPRX_NotSync(state_struct *state,
				      CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_NotSync
 */
CDN_API_STATUS CDN_API_HDCPRX_NotSync_blocking(state_struct *state,
					       CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_REC_DEBUG_SET_LONG_DELAY command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_DebugSetLongDelay(state_struct *state,
				 u8 delays[HDCPRX_NUM_LONG_DELAYS],
				 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_DebugSetLongDelay
 */
CDN_API_STATUS
CDN_API_HDCPRX_DebugSetLongDelay_blocking(state_struct *state,
					  u8 delays[HDCPRX_NUM_LONG_DELAYS],
					  CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_REC_DEBUG_SET_WRONG_ANSWER command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_DebugSetWrongAnswer(state_struct *state,
				   u8 errors[HDCPRX_DEBUG_NUM_ERRORS],
				   CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_DebugSetWrongAnswer
 */
CDN_API_STATUS
CDN_API_HDCPRX_DebugSetWrongAnswer_blocking(state_struct *state,
					    u8 errors[HDCPRX_DEBUG_NUM_ERRORS],
					    CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_GENERAL_2_SET_LC command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetLc128(state_struct *state,
			const u8 lc128[16], CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_SetLc128
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetLc128_blocking(state_struct *state,
				 const u8 lc128[16], CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_SET_SEED command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetSeed(state_struct *state,
		       u8 seed[32], CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_SetSeed
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetSeed_blocking(state_struct *state,
				u8 seed[32], CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_SET_KM_KEY command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetKmKey(state_struct *state,
			const u8 km_key[16], CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCPRX_SetKmKey
 */
CDN_API_STATUS
CDN_API_HDCPRX_SetKmKey_blocking(state_struct *state,
				 const u8 km_key[16], CDN_BUS_TYPE bus_type);

#endif
